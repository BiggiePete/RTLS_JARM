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

#[path = "../TLV4930D.rs"]
mod tlv4930d;
use crate::tlv4930d::{Tlv493d, TLV493D_ADDR_HIGH};

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
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    USART2 => usart::InterruptHandler<peripherals::USART2>;
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
    spawner.spawn(process_data()).unwrap();

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
    let mut tlv4930d = Tlv493d::new(&i2c_ref, TLV493D_ADDR_HIGH);
    // TODO: fix issue with magnetrometer initializing, but not making new readings, or reading only 0s, or being weird in general
    if let Err(e) = tlv4930d.init().await {
        info!(
            "TLV4930D initialization failed: {}",
            defmt::Debug2Format(&e)
        );
    } else {
        info!("TLV4930D initialized successfully.");
    }
    // tlv4930d
    //     .set_power_mode(tlv4930d::PowerMode::Fast)
    //     .await
    //     .unwrap();
    // let mut pressure_sensor = Gzp6816d::new(&i2c_ref);

    loop {
        // debug_led1.set_high();
        // let mut data = DataMessage {
        //     temperature: 0.0,         // Replace with actual temperature reading
        //     humidity: 0.0,            // Replace with actual humidity reading
        //     pressure: 0.0,            // Replace with actual pressure reading
        //     gps: (0.0, 0.0, 0.0),     // Replace with actual GPS data
        //     accel: (0.0, 0.0, 0.0),   // Replace with actual accelerometer data
        //     gyro: (0.0, 0.0, 0.0),    // Replace with actual gyroscope data
        //     magnetometer: (0.0, 0.0), // Replace with actual magnetometer data
        // };

        // match aht.read().await {
        //     Ok((temperature, humidity)) => {
        //         data.temperature = temperature;
        //         data.humidity = humidity;
        //         info!(
        //             "AHT20 - Temperature: {:?}°C, Humidity: {:?}",
        //             temperature, humidity
        //         );
        //     }
        //     Err(e) => {
        //         warn!("Error reading AHT20 sensor: {}", defmt::Debug2Format(&e));
        //     }
        // }
        match tlv4930d.read_3d_components().await {
            Ok((x, y, z)) => {
                info!("TLV4930D - X: {:?}, Y: {:?}, Z: {:?}", x, y, z);
                // data.accel = (x, y, z); // Assuming these are accelerometer values
            }
            Err(e) => {
                warn!("Error reading TLV4930D sensor: {}", defmt::Debug2Format(&e));
            }
        }
        Timer::after_secs(2).await;
        // TODO, figure out why board #5 is not working with the pressure sensor, or test on other boards
        // match pressure_sensor.read_calibrated_data().await {
        //     Ok((pressure, temp)) => {
        //         info!(
        //             "GZP6816D - Pressure: {:?} kPa, Temperature: {:?}°C",
        //             pressure, temp
        //         );
        //     }
        //     Err(e) => {
        //         error!(
        //             "Failed to read GZP6816D sensor: {}",
        //             defmt::Debug2Format(&e)
        //         );
        //     }
        // }

        // we will allocate all data to a struct then send it through the channel
        // debug_led1.set_low();
        // DEVICE_DATA.send(data).await;
    }
}

#[embassy_executor::task]
async fn process_data() {
    loop {
        // Wait for data to be available
        let data = DEVICE_DATA.receive().await;

        // Process the received data
        info!(
            "Received Data - Temperature: {:?}°C, Humidity: {:?}, Pressure: {:?} kPa",
            data.temperature, data.humidity, data.pressure
        );
        info!(
            "GPS: ({:?}, {:?}, {:?}), Accel: ({:?}, {:?}, {:?}), Gyro: ({:?}, {:?}, {:?}), Magnetometer: ({:?}, {:?})",
            data.gps.0, data.gps.1, data.gps.2,
            data.accel.0, data.accel.1, data.accel.2,
            data.gyro.0, data.gyro.1, data.gyro.2,
            data.magnetometer.0, data.magnetometer.1
        );
    }
}
