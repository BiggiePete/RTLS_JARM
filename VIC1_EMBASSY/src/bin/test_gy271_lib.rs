#![no_std]
#![no_main]

#[path = "../aht20.rs"]
mod aht20;


#[path = "../GZP6816D.rs"]
mod gz6816d;

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
use embassy_stm32::mode::Blocking;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use libm::atan;
use num_traits::abs;
use qmc5883l::QMC5883L;
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

    let i2c = I2c::new_blocking(p.I2C1, p.PB6, p.PB7, Hertz(100_000), i2c_config);

    debugLED1.set_low();
    debugLED2.set_low();
    debugLED3.set_low();

    spawner
        .spawn(gather_data(i2c, debugLED1, debugLED2, debugLED3))
        .unwrap();
}

#[embassy_executor::task]
async fn gather_data(
    i2c: embassy_stm32::i2c::I2c<'static, Blocking>,
    debug_led1: Output<'static>,
    debug_led2: Output<'static>,
    debug_led3: Output<'static>,
) {
    // DEVICE_DATA.send(DataMessage::Temperature(0.0)).await
    // initialize i2c devices
    // let i2c_ref: RefCell<I2c<'static, Async>> = RefCell::new(i2c);
    let mut mag = QMC5883L::new(i2c).expect("Failed to initialize QMC5883L magnetometer");
    mag.continuous().unwrap();

    loop {
        match mag.mag() {
            Ok((x, y, z)) => {
                // info!("Mag Field: X={} G, Y={} G, Z={} G", x, y, z);
                let dir = atan(y as f64 / x as f64) * (y / abs(y)) as f64;
                info!("Direction: {} radians", dir);
            }
            Err(e) => {
                error!("Failed to read QMC5883L: {:?}", defmt::Debug2Format(&e));
            }
        }
        Timer::after(Duration::from_secs(1)).await;
    }
}
