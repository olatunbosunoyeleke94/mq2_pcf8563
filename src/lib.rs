// src/lib.rs
#![no_std]

use embedded_hal::i2c::I2c;
use pcf8563::{PCF8563, DateTime, Error as RtcError};
use ufmt::{uWrite, uwrite, uwriteln};

/// Configuration options for the MQ-2 gas sensor + RTC logger.
/// 
/// This struct holds user-configurable settings for alert thresholds and smoothing.
#[derive(Clone, Copy, Debug)]
pub struct GasRtcLoggerConfig {
    /// The INDEX value above which a high-gas alert is triggered.
    /// 
    /// Typical values: 300–600 depending on sensitivity needs.
    pub alert_threshold: u16,

    /// Number of previous INDEX readings to include in the moving average.
    /// 
    /// Controls smoothing: higher = smoother but slower response.
    /// Must be 1–16 (fixed buffer size).
    pub smoothing_samples: usize,
}

/// Combined driver for MQ-2 gas sensor (analog input) and PCF8563/HZ-8563 RTC (I²C).
/// 
/// Provides:
/// - Baseline calibration in clean air
/// - Smoothed "gas index" calculation (higher = more gas detected)
/// - Timestamped logging with ufmt
/// - High-gas alert detection
/// 
/// The ADC reading and pin setup are handled externally (passed in via `new`).
/// The RTC is initialized automatically on creation.
pub struct GasRtcLogger<I2C> {
    /// The underlying PCF8563 RTC driver.
    rtc: PCF8563<I2C>,

    /// Raw ADC value measured in clean air (set via `set_baseline`).
    baseline_raw: u16,

    /// User configuration (alert threshold, smoothing).
    config: GasRtcLoggerConfig,

    /// Circular buffer for moving-average smoothing of the gas index.
    last_indices: [u16; 16],

    /// Current write position in the circular buffer.
    index_pos: usize,
}

impl<I2C, E> GasRtcLogger<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Creates a new logger instance.
    /// 
    /// # Arguments
    /// * `i2c` - I²C interface (e.g., from `arduino_hal::I2c`)
    /// * `config` - Alert threshold and smoothing settings
    /// 
    /// The RTC is initialized automatically (best effort).
    /// Baseline must be set later with `set_baseline` before gas_index works.
    pub fn new(i2c: I2C, config: GasRtcLoggerConfig) -> Self {
        let mut rtc = PCF8563::new(i2c);
        let _ = rtc.rtc_init();

        Self {
            rtc,
            baseline_raw: 0,
            config,
            last_indices: [0; 16],
            index_pos: 0,
        }
    }

    /// Sets the clean-air baseline raw ADC value.
    /// 
    /// This should be called once after measuring in fresh air (e.g., via external calibration).
    /// 
    /// # Arguments
    /// * `baseline` - Raw ADC reading in clean air (typically 50–200)
    pub fn set_baseline(&mut self, baseline: u16) {
        self.baseline_raw = baseline;
    }

    /// Computes a smoothed "gas index" from the current raw ADC reading.
    /// 
    /// The index is based on the difference from the clean-air baseline,
    /// scaled by 50, capped at u16::MAX, and smoothed with a moving average.
    /// 
    /// # Arguments
    /// * `raw` - Current raw ADC value from MQ-2 (0–1023)
    /// 
    /// # Returns
    /// The smoothed INDEX (higher = more gas detected).
    /// Returns 0 if baseline is not set.
    pub fn gas_index(&mut self, raw: u16) -> u16 {
        if self.baseline_raw == 0 {
            return 0;
        }

        let delta = self.baseline_raw.saturating_sub(raw);
        let mut index = delta as u32 * 50;
        index = index.min(65_535);

        // Update circular buffer
        self.last_indices[self.index_pos] = index as u16;
        self.index_pos = (self.index_pos + 1) % self.config.smoothing_samples;

        // Compute moving average
        let mut sum: u32 = 0;
        for i in 0..self.config.smoothing_samples {
            sum += self.last_indices[i] as u32;
        }

        (sum / self.config.smoothing_samples as u32) as u16
    }

    /// Checks whether the given gas INDEX exceeds the configured alert threshold.
    /// 
    /// # Arguments
    /// * `index` - Current gas index value
    /// 
    /// # Returns
    /// `true` if INDEX > alert_threshold (high gas detected)
    pub fn is_high_gas(&self, index: u16) -> bool {
        index > self.config.alert_threshold
    }

    /// Retrieves the current date and time from the RTC.
    /// 
    /// # Returns
    /// `Ok(DateTime)` on success, or `Err` if I²C communication fails.
    pub fn get_datetime(&mut self) -> Result<DateTime, RtcError<E>> {
        self.rtc.get_datetime()
    }

    /// Helper: writes a zero-padded two-digit number using ufmt.
    fn pad2<W: uWrite>(w: &mut W, n: u8) {
        if n < 10 {
            let _ = uwrite!(w, "0{}", n);
        } else {
            let _ = uwrite!(w, "{}", n);
        }
    }

    /// Helper: writes a zero-padded four-digit year using ufmt.
    fn pad4<W: uWrite>(w: &mut W, n: u16) {
        if n < 10 {
            let _ = uwrite!(w, "000{}", n);
        } else if n < 100 {
            let _ = uwrite!(w, "00{}", n);
        } else if n < 1000 {
            let _ = uwrite!(w, "0{}", n);
        } else {
            let _ = uwrite!(w, "{}", n);
        }
    }

    /// Logs one reading to the provided serial writer.
    /// 
    /// Format: `YYYY-MM-DD HH:MM:SS | RAW=xxx INDEX=xxx`
    /// If INDEX > alert threshold, appends "ALERT: HIGH GAS DETECTED!"
    /// 
    /// # Arguments
    /// * `serial` - Writer (e.g. serial console)
    /// * `raw` - Current raw ADC value from MQ-2
    pub fn log<W: uWrite>(&mut self, serial: &mut W, raw: u16) {
        if let Ok(dt) = self.get_datetime() {
            Self::pad4(serial, dt.year as u16 + 2000);
            let _ = uwrite!(serial, "-");
            Self::pad2(serial, dt.month);
            let _ = uwrite!(serial, "-");
            Self::pad2(serial, dt.day);
            let _ = uwrite!(serial, " ");
            Self::pad2(serial, dt.hours);
            let _ = uwrite!(serial, ":");
            Self::pad2(serial, dt.minutes);
            let _ = uwrite!(serial, ":");
            Self::pad2(serial, dt.seconds);
            let _ = uwrite!(serial, " | ");
        } else {
            let _ = uwrite!(serial, "RTC_ERR | ");
        }

        let index = self.gas_index(raw);
        let _ = uwriteln!(serial, "RAW={} INDEX={}", raw, index);

        if self.is_high_gas(index) {
            let _ = uwriteln!(serial, "ALERT: HIGH GAS DETECTED!");
        }
    }
}
