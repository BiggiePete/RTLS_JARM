#![no_std]
#![no_main]

#[path = "../aht20.rs"]
mod aht20;
use crate::aht20::AHT20;

#[path = "../GZP6816D.rs"]
mod gz6816d;
use crate::gz6816d::Gzp6816d;

#[path = "../i2c_search.rs"]
mod i2c_search;
use crate::i2c_search::I2cScanner;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;

use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World! from JARM");
    // lets initialize all of the components necessary for runtime

    let mut debugLED1 = Output::new(p.PB15, Level::High, Speed::Low);
    let mut debugLED2 = Output::new(p.PB14, Level::High, Speed::Low);
    let mut debugLED3 = Output::new(p.PB13, Level::High, Speed::Low);

    // Initialize I2C for AHT20 and GZP6816D sensors
    let mut i2c = I2c::new_blocking(p.I2C1, p.PB6, p.PB7, Hertz(100_000), Default::default());
    // let  aht = AHT20::new(&mut i2c);
    let mut pressure_sensor = Gzp6816d::new(i2c);
    // let scanner = I2cScanner::new();

    // let scan_results = I2cScanner::scan_bus(&mut i2c);

    // Print the results
    // i2c_search::print_scan_results(&scan_results);

    loop {
        debugLED1.set_low();
        debugLED2.set_low();
        debugLED3.set_low();
        Timer::after(Duration::from_millis(1000)).await;

        debugLED1.set_high();
        debugLED2.set_high();
        debugLED3.set_high();
        Timer::after(Duration::from_millis(1000)).await;

        // match aht.read() {
        //     Ok((t, h)) => {
        //         info!("Temperature: {:?} , Humidity: {:?}", t, h);
        //     }
        //     Err(_e) => {
        //         warn!("Error reading AHT20 sensor");
        //     }
        // }

        match pressure_sensor.read_calibrated_data() {
            Ok((temp, pressure)) => {
                info!("Temperature: {:?}°C, Pressure: {:?} Pa", temp, pressure);
            }
            Err(e) => {
                error!("Failed to read GZP6816D sensor: {:?}", e);
            }
        }
    }
}
