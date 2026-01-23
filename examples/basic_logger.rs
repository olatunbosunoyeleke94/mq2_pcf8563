// examples/basic_logger.rs
#![no_std]
#![no_main]

use panic_halt as _;

use arduino_hal::{
    entry,
    default_serial,
    delay_ms,
    Adc,
    Peripherals,
    pins,
};

use mq2_pcf8563::{GasRtcLogger, GasRtcLoggerConfig};

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let pins = pins!(dp);

    let mut serial = default_serial!(dp, pins, 57600);

    let mut adc = Adc::new(dp.ADC, Default::default());
    let mq2_pin = pins.a0.into_analog_input(&mut adc);

    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        100_000,
    );

    let config = GasRtcLoggerConfig {
        alert_threshold: 300,
        smoothing_samples: 8,
    };

    let mut logger = GasRtcLogger::new(i2c, config);

    // Calibrate once (in clean air) — prints to serial
    logger.calibrate_baseline(&mut serial);

    loop {
        // Read raw ADC (external to library)
        let raw = adc.read_blocking(&mut mq2_pin);

        // Log using library method (passes raw)
        logger.log(&mut serial, raw);

        delay_ms(2000);
    }
}
