// src/lib.rs
#![no_std]

use arduino_hal::{
    Adc,
    delay_ms,
    hal::port::{Pin, mode::Analog},
};
use pcf8563::{PCF8563, DateTime, Error as RtcError};
use ufmt::{uWrite, uwrite, uwriteln};

/// Configuration options for the MQ-2 + RTC logger.
#[derive(Clone, Copy)]
pub struct GasRtcLoggerConfig {
    /// Threshold value for the gas INDEX to trigger an alert.
    pub alert_threshold: u16,

    /// Number of previous readings to use for moving average smoothing.
    /// Must be between 1 and 16 (buffer size).
    pub smoothing_samples: usize,
}

/// Combined logger for MQ-2 gas sensor (via ADC) and PCF8563/HZ-8563 RTC (via I²C).
///
/// This struct owns the RTC driver and ADC resources, and provides methods
/// for calibration, reading, smoothing, alerting, and timestamped logging.
pub struct GasRtcLogger<I2C> {
    /// The PCF8563 RTC driver.
    rtc: PCF8563<I2C>,

    /// ADC peripheral for reading the MQ-2 sensor.
    adc: Adc,

    /// Analog pin connected to MQ-2 AOUT (usually A0 = PC0).
    mq2_pin: Pin<Analog, arduino_hal::hal::port::PC0>,

    /// Raw ADC value measured in clean air (set via `calibrate_baseline`).
    baseline_raw: u16,

    /// User-provided configuration.
    config: GasRtcLoggerConfig,

    /// Circular buffer for moving average smoothing of the gas index.
    last_indices: [u16; 16],

    /// Current position in the circular buffer.
    index_pos: usize,
}

impl<I2C> GasRtcLogger<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    /// Creates a new logger instance.
    ///
    /// Initializes the RTC (best effort) but does **not** calibrate the baseline.
    /// Call `calibrate_baseline` once in clean air before using gas readings.
    pub fn new(
        i2c: I2C,
        adc: Adc,
        mq2_pin: Pin<Analog, arduino_hal::hal::port::PC0>,
        config: GasRtcLoggerConfig,
    ) -> Self {
        let mut rtc = PCF8563::new(i2c);
        let _ = rtc.rtc_init();

        Self {
            rtc,
            adc,
            mq2_pin,
            baseline_raw: 0,
            config,
            last_indices: [0; 16],
            index_pos: 0,
        }
    }

    /// Calibrates the clean-air baseline by averaging 20 ADC readings.
    ///
    /// Prints progress and result to the provided serial writer.
    /// Should be called once in fresh air after sensor preheat.
    ///
    /// # Returns
    /// The computed baseline raw ADC value (also stored internally).
    pub fn calibrate_baseline<W: uWrite>(&mut self, serial: &mut W) -> u16 {
        let _ = uwriteln!(serial, "Calibrating baseline...");
        delay_ms(10_000);

        let mut sum: u32 = 0;
        for _ in 0..20 {
            sum += self.adc.read_blocking(&mut self.mq2_pin) as u32;
            delay_ms(300);
        }

        let avg = (sum / 20) as u16;
        self.baseline_raw = avg;

        let _ = uwriteln!(serial, "Baseline raw: {}", avg);
        avg
    }

    /// Reads the current raw ADC value from the MQ-2 sensor.
    pub fn read_gas_raw(&mut self) -> u16 {
        self.adc.read_blocking(&mut self.mq2_pin)
    }

    /// Computes a smoothed gas INDEX value based on the difference from baseline.
    ///
    /// Uses a moving average over `smoothing_samples` previous readings.
    /// The index is scaled (delta × 50) and capped at u16::MAX.
    ///
    /// # Returns
    /// The smoothed INDEX value (higher = more gas detected).
    pub fn gas_index(&mut self, raw: u16) -> u16 {
        if self.baseline_raw == 0 {
            return 0;
        }

        let delta = self.baseline_raw.saturating_sub(raw);
        let mut index = delta as u32 * 50;
        index = index.min(65_535);

        // Circular buffer update
        self.last_indices[self.index_pos] = index as u16;
        self.index_pos = (self.index_pos + 1) % self.config.smoothing_samples;

        // Compute moving average
        let mut sum: u32 = 0;
        for i in 0..self.config.smoothing_samples {
            sum += self.last_indices[i] as u32;
        }

        (sum / self.config.smoothing_samples as u32) as u16
    }

    /// Checks if the current gas INDEX exceeds the configured alert threshold.
    pub fn is_high_gas(&self, index: u16) -> bool {
        index > self.config.alert_threshold
    }

    /// Retrieves the current date and time from the RTC.
    pub fn get_datetime(&mut self) -> Result<DateTime, RtcError<I2C::Error>> {
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

    /// Logs a timestamped reading to the provided serial writer.
    ///
    /// Format: `YYYY-MM-DD HH:MM:SS | RAW=xxx INDEX=xxx`
    /// If INDEX > threshold, appends "ALERT: HIGH GAS DETECTED!"
    pub fn log_data<W: uWrite>(&mut self, serial: &mut W) {
        match self.get_datetime() {
            Ok(dt) => {
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
            }
            Err(_) => {
                let _ = uwrite!(serial, "RTC_ERR | ");
            }
        }

        let raw = self.read_gas_raw();
        let index = self.gas_index(raw);

        let _ = uwriteln!(serial, "RAW={} INDEX={}", raw, index);

        if self.is_high_gas(index) {
            let _ = uwriteln!(serial, "ALERT: HIGH GAS DETECTED!");
        }
    }
}
