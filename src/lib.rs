#![no_std]

use embedded_hal::i2c::I2c;
use pcf8563::{PCF8563, DateTime, Error as RtcError};
use ufmt::{uWrite, uwrite, uwriteln};

/// Configuration options
#[derive(Clone, Copy)]
pub struct GasRtcLoggerConfig {
    pub alert_threshold: u16,
    pub smoothing_samples: usize,
}

/// MQ-2 gas logic + PCF8563 RTC logger
pub struct GasRtcLogger<I2C> {
    rtc: PCF8563<I2C>,
    baseline_raw: u16,
    config: GasRtcLoggerConfig,
    last_indices: [u16; 16],
    index_pos: usize,
}

impl<I2C, E> GasRtcLogger<I2C>
where
    I2C: I2c<Error = E>,
{
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

    /// Set baseline from clean-air ADC readings (done externally)
    pub fn set_baseline(&mut self, baseline: u16) {
        self.baseline_raw = baseline;
    }

    /// Compute smoothed gas index from a raw ADC reading
    pub fn gas_index(&mut self, raw: u16) -> u16 {
        if self.baseline_raw == 0 {
            return 0;
        }

        let delta = self.baseline_raw.saturating_sub(raw);
        let index = (delta as u32 * 50).min(u16::MAX as u32) as u16;

        self.last_indices[self.index_pos] = index;
        self.index_pos = (self.index_pos + 1) % self.config.smoothing_samples;

        let mut sum: u32 = 0;
        for i in 0..self.config.smoothing_samples {
            sum += self.last_indices[i] as u32;
        }

        (sum / self.config.smoothing_samples as u32) as u16
    }

    pub fn is_high_gas(&self, index: u16) -> bool {
        index > self.config.alert_threshold
    }

    pub fn get_datetime(&mut self) -> Result<DateTime, RtcError<E>> {
        self.rtc.get_datetime()
    }

    fn pad2<W: uWrite>(w: &mut W, n: u8) {
        if n < 10 {
            let _ = uwrite!(w, "0{}", n);
        } else {
            let _ = uwrite!(w, "{}", n);
        }
    }

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

    /// Log one reading (RAW passed in)
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
        }

        let index = self.gas_index(raw);
        let _ = uwriteln!(serial, "RAW={} INDEX={}", raw, index);

        if self.is_high_gas(index) {
            let _ = uwriteln!(serial, "ALERT: HIGH GAS DETECTED!");
        }
    }
}
