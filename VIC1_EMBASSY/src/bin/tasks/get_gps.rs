#[path = "../../aht20.rs"]
mod aht20;
use crate::aht20::AHT20;

#[path = "../../icm42688.rs"]
mod icm42688;
use crate::icm42688::{Icm42688p, ICM42688P_ADDR_AD0_LOW};

#[path = "../../inertial_navigator2.rs"]
mod inertial;
use crate::inertial::{CalibrationState, InertialNavigator, Vec3};

#[path = "../../gy271.rs"]
mod gy271;
use crate::gy271::GY271;

use core::cell::RefCell;
use core::f64::consts::PI;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::UartRx;
use embassy_stm32::{bind_interrupts, i2c, peripherals, usart};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use libm::atan2;
use {defmt_rtt as _, panic_probe as _};

#[derive(Debug)]
struct DataMessageGPS {
    latitude: f32,
    longitude: f32,
    altitude: f32,
    gps_ready: bool,
}

static DEVICE_DATA_GPS: Channel<CriticalSectionRawMutex, DataMessageGPS, 2> = Channel::new();

#[embassy_executor::task]
pub async fn gather_gps_data(
    mut usart: UartRx<'static, Async>,
    mut debug_led1: Output<'static>,
    mut debug_led2: Output<'static>,
    mut debug_led3: Output<'static>,
) {
    // Initialize GPS here
    // This is a placeholder for GPS initialization logic
    // You would typically use a GPS library to read data from the USART

    // Simulate GPS data gathering
    loop {
        // Simulate reading GPS data
        let gps_data = DataMessageGPS {
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0.0,
            gps_ready: true,
        };

        DEVICE_DATA_GPS.send(gps_data).await;

        Timer::after(Duration::from_secs(1)).await; // Simulate delay for GPS data reading
    }
}
