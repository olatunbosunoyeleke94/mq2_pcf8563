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
use ufmt::uwriteln;

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let pins = pins!(dp);

    let mut serial = default_serial!(dp, pins, 57600);

    let mut adc = Adc::new(dp.ADC, Default::default());
    let mut mq2_pin = pins.a0.into_analog_input(&mut adc);

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

    // Set RTC time
    let dt = pcf8563::DateTime {
        year: 26,
        month: 1,
        day: 31,
        weekday: 5,
        hours: 14,
        minutes: 52,
        seconds: 0,
    };

    let _ = logger.set_datetime(&dt);

    // Baseline calibration
    let _ = uwriteln!(&mut serial, "Measuring baseline in clean air...");
    delay_ms(10_000);

    let mut sum: u32 = 0;
    for _ in 0..20 {
        sum += adc.read_blocking(&mut mq2_pin) as u32;
        delay_ms(500);
    }
    let baseline = (sum / 20) as u16;
    let _ = uwriteln!(&mut serial, "Baseline raw: {}", baseline);
    logger.set_baseline(baseline);

    loop {
        let raw = adc.read_blocking(&mut mq2_pin);
        logger.log(&mut serial, raw);
        delay_ms(2000);
    }
}
