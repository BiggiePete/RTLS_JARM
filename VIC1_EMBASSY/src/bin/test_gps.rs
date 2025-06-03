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

#[path = "../NEO6.rs"]
mod neo6;
use crate::neo6::NeoGps;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::{self, Async};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{self, Config, Uart, UartRx};
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
        p.DMA1_CH1,
        p.DMA1_CH0,
        Hertz(100_000),
        i2c_config,
    );

    debugLED1.set_low();
    debugLED2.set_low();
    debugLED3.set_low();
    let uart_config = embassy_stm32::usart::Config::default(); // Set baud rate etc.
    let mut uart_rx = UartRx::new(p.USART2, Irqs, p.PA3, p.DMA1_CH5, uart_config).unwrap();
    let _ = uart_rx.set_baudrate(9600);

    let mut gps = NeoGps::new(&mut uart_rx);

    // TODO: check to see if the GPS works outdoors, currently, the readings dont come through when inside a building
    loop {
        match gps.read_gga_data().await {
            Ok(gga) => {
                info!(
                    "GPS Data: lat={} lon={} alt={}",
                    gga.latitude.unwrap_or(0.0),
                    gga.longitude.unwrap_or(0.0),
                    gga.altitude_msl.unwrap_or(0.0),
                );
                let gps_data = (
                    gga.latitude.unwrap_or(0.0),
                    gga.longitude.unwrap_or(0.0),
                    gga.altitude_msl.unwrap_or(0.0),
                );
            }
            Err(e) => {
                error!("Failed to read GPS data: {}", defmt::Debug2Format(&e));
            }
        }
    }
}
