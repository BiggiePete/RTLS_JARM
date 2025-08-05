use defmt::*;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

use crate::{get_gps_task::DEVICE_DATA_GPS, get_i2c_task::DEVICE_DATA_I2C};

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::task]
pub async fn consume_data() {
    trace!("Starting Demo Consumer Task");
    loop {
        match DEVICE_DATA_GPS.try_receive() {
            Ok(gps_data) => {}
            Err(_) => {
                warn!("No GPS Data");
            }
        }
        match DEVICE_DATA_I2C.try_receive() {
            Ok(i2c_data) => {}
            Err(_) => {
                warn!("No I2C Data");
            }
        }
        Timer::after_millis(200).await;
    }
}
