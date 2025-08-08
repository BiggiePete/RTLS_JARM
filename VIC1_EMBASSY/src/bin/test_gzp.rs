#![no_std]
#![no_main]

#[path = "../aht20.rs"]
mod aht20;
use core::cell::RefCell;


#[path = "../GZP6816D.rs"]
mod gz6816d;
use crate::gz6816d::Gzp6816d;

#[path = "../i2c_search.rs"]
mod i2c_search;

#[path = "../TLV4930D_2.rs"]
mod tlv4930d;

#[path = "../icm42688.rs"]
mod icm42688;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, peripherals};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    // USART2 => usart::InterruptHandler<peripherals::USART2>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World! from JARM");
    // lets initialize all of the components necessary for runtime

    let mut debugLED1 = Output::new(p.PB15, Level::High, Speed::Low);
    let mut debugLED2 = Output::new(p.PB14, Level::High, Speed::Low);
    let mut debugLED3 = Output::new(p.PB13, Level::High, Speed::Low);
    let powerSelect = Output::new(p.PB12, Level::High, Speed::Low);

    let mut i2c_config = embassy_stm32::i2c::Config::default();
    i2c_config.timeout = Duration::from_millis(100); // Set a 100ms timeout

    let i2c = I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH0,
        Hertz(100_000),
        i2c_config,
    );

    debugLED1.set_low();
    debugLED2.set_low();
    debugLED3.set_low();

    spawner
        .spawn(gather_data(i2c, debugLED1, debugLED2, debugLED3))
        .unwrap();
}

// In main.rs

#[embassy_executor::task]
async fn gather_data(
    i2c: embassy_stm32::i2c::I2c<'static, Async>,
    mut debug_led1: Output<'static>, // We will use this LED
    mut debug_led2: Output<'static>,
    mut debug_led3: Output<'static>,
) {
    debug_led1.set_high();
    Timer::after_millis(50).await;
    info!("gather_data task started. LED1 should be ON.");

    let i2c_ref: RefCell<I2c<'static, Async>> = RefCell::new(i2c);

    let mut gz6816d_sensor = Gzp6816d::new(&i2c_ref);
    info!("Gzp6816d instance created.");

    loop {
        info!("Attempting to read GZP6816D...");
        debug_led2.set_high(); // Turn on LED2 before reading
        match gz6816d_sensor.get_pressure_temperature().await {
            Ok((pressure, temperature)) => {
                info!("Pressure: {} Pa, Temperature: {} C", pressure, temperature);
                debug_led2.set_low(); // Turn off LED2 after successful read
                if debug_led3.is_set_high() {
                    // Turn off LED3 (error LED) if it was on
                    debug_led3.set_low();
                }
            }
            Err(e) => {
                error!("Failed to read GZP6816D: {:?}", e);
                debug_led2.set_low(); // Turn off LED2
                debug_led3.set_high(); // Turn on LED3 to indicate error
            }
        }
        info!("Waiting 2 seconds...");
        Timer::after_secs(2).await;
    }
}
