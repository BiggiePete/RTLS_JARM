#![no_std]
#![no_main]

#[path = "../aht20.rs"]
mod aht20;
use core::cell::RefCell;


#[path = "../GZP6816D.rs"]
mod gz6816d;

#[path = "../i2c_search.rs"]
mod i2c_search;

#[path = "../TLV4930D_2.rs"]
mod tlv4930d;

#[path = "../icm42688.rs"]
mod icm42688;

#[path = "../gy271.rs"]
mod gy271;
use crate::gy271::GY271;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use libm::{atan2, floor};

// Define PI for calculations, as it's not always in scope in `no_std`
const PI: f64 = core::f64::consts::PI;

// Define the 16 cardinal directions, starting from North and going clockwise.
const DIRECTIONS: [&str; 16] = [
    "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW",
    "NNW",
];
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

#[embassy_executor::task]
async fn gather_data(
    i2c: embassy_stm32::i2c::I2c<'static, Async>,
    debug_led1: Output<'static>,
    mut debug_led2: Output<'static>,
    debug_led3: Output<'static>,
) {
    // DEVICE_DATA.send(DataMessage::Temperature(0.0)).await
    // initialize i2c devices
    let i2c_ref: RefCell<I2c<'static, Async>> = RefCell::new(i2c);
    let mut gy271 = GY271::new(&i2c_ref)
        .await
        .expect("Failed to initialize GY-271");
    info!("GY-271 instance created.");
    gy271
        .continuous()
        .await
        .expect("Failed to set GY-271 to continuous mode");

    loop {
        info!("Attempting to read GY-271...");
        debug_led2.set_high(); // Turn on LED2 before reading
        match gy271.mag().await {
            Ok((x, y, _z)) => {
                let angle_rad = atan2(y as f64, x as f64);

                let mut angle_deg = (angle_rad * (180.0 / PI)) - 70.0;

                if angle_deg < 0.0 {
                    angle_deg += 360.0;
                }
                let direction_index = floor((angle_deg + 11.25) / 22.5) as usize % 16;
                let direction_str = DIRECTIONS[direction_index];

                // 5. Print the angle in degrees and the 3-letter cardinal direction.
                info!("Angle: {:?}°, Direction: {:?}", angle_deg, direction_str);
            }
            Err(e) => {
                error!("Failed to read GY271: {:?}", defmt::Debug2Format(&e));
            }
        }
        debug_led2.set_low(); // Turn off LED2 after reading
        Timer::after(Duration::from_secs(1)).await;
    }
}
