#![no_std]
#![no_main]

#[path = "../aht20.rs"]
mod aht20;
use core::cell::RefCell;

use crate::aht20::AHT20;

#[path = "../GZP6816D.rs"]
mod gz6816d;
use crate::gz6816d::Gzp6816d;

#[path = "../i2c_search.rs"]
mod i2c_search;
use crate::i2c_search::I2cScanner;

#[path = "../TLV4930D_2.rs"]
mod tlv4930d;
use crate::tlv4930d::{PowerMode, TLV493D};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{bind_interrupts, i2c, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Delay, Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    // USART2 => usart::InterruptHandler<peripherals::USART2>;
});

// 2 spots
static DEVICE_DATA: Channel<CriticalSectionRawMutex, DataMessage, 2> = Channel::new();

#[derive(Debug)]
struct DataMessage {
    temperature: f32,
    humidity: f32,
    pressure: f32,
    gps: (f32, f32, f32),     // latitude, longitude, altitude
    accel: (f32, f32, f32),   // x, y, z acceleration
    gyro: (f32, f32, f32),    // x, y, z gyroscope
    magnetometer: (f32, f32), // a,r magnetic field
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World! from JARM");
    // lets initialize all of the components necessary for runtime

    let mut debugLED1 = Output::new(p.PB15, Level::High, Speed::Low);
    let mut debugLED2 = Output::new(p.PB14, Level::High, Speed::Low);
    let mut debugLED3 = Output::new(p.PB13, Level::High, Speed::Low);
    let mut powerSelect = Output::new(p.PB12, Level::High, Speed::Low);

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

    // let mut i2c = i2c::I2c::new_blocking(p.I2C1, p.PB6, p.PB7, Hertz(100_000), Default::default());
    // let mut aht = AHT20::new(i2c);
    // let mut pressure_sensor = Gzp6816d::new(i2c);
    // let scanner = I2cScanner::new();

    // let scan_results = I2cScanner::scan_bus(&mut i2c);

    // Print the results
    // i2c_search::print_scan_results(&scan_results);
}

#[embassy_executor::task]
async fn gather_data(
    i2c: embassy_stm32::i2c::I2c<'static, Async>, // Corrected Async mode path
    mut debug_led1: Output<'static>,
    mut debug_led2: Output<'static>,
    mut debug_led3: Output<'static>,
) {
    // DEVICE_DATA.send(DataMessage::Temperature(0.0)).await

    // Initialize I2C devices
    let mut i2c_ref: RefCell<I2c<'static, embassy_stm32::mode::Async>> = RefCell::new(i2c);

    // Create a Delay instance. In Embassy, Delay is often obtained from a singleton
    // or created if your HAL provides a way. For stm32, it's often `Delay`.
    let mut delay_source = Delay; // embassy_time::Delay is a ZST and can be used directly

    info!("Initializing TLV493D sensor...");

    // Initialize the TLV493D sensor
    // Use the i2c_ref, address, the delay source, and the soft reset flag.
    // Setting perform_soft_reset_for_addr_select to false as pull-ups should set the address.
    let mut tlv4930d = TLV493D::new(&mut i2c_ref, true);
    match tlv4930d.init(PowerMode::MasterControlled).await {
        Ok(_) => {
            info!("TLV493D initialized successfully.");
        }
        Err(e) => {
            error!("Failed to initialize TLV493D: {}", defmt::Debug2Format(&e));
            // If initialization fails, we probably can't proceed with this sensor.
        }
    }
    loop {
        match tlv4930d.read().await {
            Ok(sensor) => {
                info!(
                    "TLV493D reading: x: {}, y: {}, z: {}",
                    sensor.bx_mt, sensor.by_mt, sensor.bz_mt
                );
            }
            Err(e) => {
                error!("Failed to initialize TLV493D: {}", defmt::Debug2Format(&e));
                // If initialization fails, we probably can't proceed with this sensor.
                // You might want to signal an error or panic.
                // For this example, we'll just loop infinitely doing nothing with the sensor.
                loop {
                    debug_led1.set_high(); // Indicate error state
                    Timer::after_millis(500).await;
                    debug_led1.set_low();
                    Timer::after_millis(500).await;
                }
            }
        };
        Timer::after_secs(2).await;
    }
}
