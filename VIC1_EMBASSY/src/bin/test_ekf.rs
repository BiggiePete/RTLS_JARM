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
use crate::tlv4930d::TLV493D;

#[path = "../icm42688.rs"]
mod icm42688;
use crate::icm42688::{Icm42688p, ICM42688P_ADDR_AD0_LOW};

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
use embassy_time::{Duration, Instant, Timer};
use rust_ekf::EKF;
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
    i2c: embassy_stm32::i2c::I2c<'static, Async>,
    mut debug_led1: Output<'static>,
    mut debug_led2: Output<'static>,
    mut debug_led3: Output<'static>,
) {
    // DEVICE_DATA.send(DataMessage::Temperature(0.0)).await
    // initialize i2c devices
    let i2c_ref: RefCell<I2c<'static, Async>> = RefCell::new(i2c);
    // let mut aht = AHT20::new(&i2c_ref);
    let mut icm42688p = Icm42688p::new(&i2c_ref, ICM42688P_ADDR_AD0_LOW);
    icm42688p.init().await.unwrap();

    // lets now start the filter
    let mut ekf = EKF::new(None);
    let mut curr_time = Instant::now(); // get current time in seconds
    let mut dt = 0 as f64; // init var
    loop {
        match icm42688p.read_all_data().await {
            Ok(d) => {
                let now = Instant::now();

                dt = (now.as_micros() - curr_time.as_micros()) as f64 / 1_000_000.0; // calculate delta time in seconds
                curr_time = now; // update current time
                ekf.predict([d.gyro.x as f64, d.gyro.y as f64, d.gyro.z as f64], dt);
                ekf.update([
                    (d.accel.x * 9.81) as f64,
                    (d.accel.y * 9.81) as f64,
                    (d.accel.z * 9.81) as f64,
                ]);
                let state = ekf.get_state();
                let state_arr = state.as_slice();
                info!(
                    "[{}, {}, {}, {}, {}, {}, {}]",
                    state_arr[0],
                    state_arr[1],
                    state_arr[2],
                    state_arr[3],
                    state_arr[4],
                    state_arr[5],
                    state_arr[6]
                );
            }
            Err(e) => {
                info!("ICM42688P read error: {}", defmt::Debug2Format(&e));
            }
        }
        // Timer::after_millis(100).await;
    }
}
