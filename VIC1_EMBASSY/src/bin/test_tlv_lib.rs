#![no_std]
#![no_main]

use core::cell::RefCell;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::mode::Blocking;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::{bind_interrupts, i2c, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use sensor_tlv493d::Tlv493d;
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

    let i2c_blocking = embassy_stm32::i2c::I2c::new_blocking(
        p.I2C1,
        p.PB6,
        p.PB7,
        Hertz(100_000),
        embassy_stm32::i2c::Config::default(),
    );

    debugLED1.set_low();
    debugLED2.set_low();
    debugLED3.set_low();

    spawner
        .spawn(gather_data(i2c_blocking, debugLED1, debugLED2, debugLED3))
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
    i2c: embassy_stm32::i2c::I2c<'static, Blocking>,
    mut debug_led1: Output<'static>,
    mut debug_led2: Output<'static>,
    mut debug_led3: Output<'static>,
) {
    // DEVICE_DATA.send(DataMessage::Temperature(0.0)).await
    // Use the I2C instance directly with the sensor driver
    match Tlv493d::new(i2c, 0x5E, sensor_tlv493d::Mode::Master) {
        Ok(mut tlv) => loop {
            match tlv.read_raw() {
                Ok(r) => {
                    println!(
                        "TLV493D reading: x : {:?}, y : {:?}, z : {:?}",
                        r[0], r[1], r[2]
                    );
                }
                Err(e) => {
                    info!("Error reading TLV493D {:?}", Debug2Format(&e));
                }
            }
            Timer::after_micros(500).await;
        },
        Err(e) => {
            info!("Error initializing TLV493D {:?}", Debug2Format(&e));
        }
    }
}
