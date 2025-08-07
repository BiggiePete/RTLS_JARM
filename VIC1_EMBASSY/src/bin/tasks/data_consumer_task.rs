use defmt::*;
use embassy_stm32::usart::UartRx;
use embassy_stm32::{i2c, mode::Async};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

use crate::{get_gps_task::DEVICE_DATA_GPS, get_i2c_task::DEVICE_DATA_I2C};

use {defmt_rtt as _, panic_probe as _};
#[derive(Debug)]
struct ConsumerObject {
    rotation: (f64, f64, f64),
    lat_lon_alt: (f32, f32, f32),
    direction: f32,   // direction in degrees
    temperature: f32, // in deg C
    humidity: f32,

    sys_ready: bool,
}

#[embassy_executor::task]
pub async fn consume_data() {
    trace!("Starting Demo Consumer Task");
    loop {
        let mut consumer_obj = ConsumerObject {
            rotation: (0.0, 0.0, 0.0),
            lat_lon_alt: (0.0, 0.0, 0.0),
            direction: (0.0),
            temperature: (0.0),
            humidity: (0.0),
            sys_ready: true,
        };
        match DEVICE_DATA_GPS.try_receive() {
            Ok(gps_data) => {
                // we need to find out if the GPS data is good for us to read, what we can do is create an object with combined data, and see if we can populate every field
                if !gps_data.gps_ready || gps_data.altitude == 0.0 {
                    consumer_obj.sys_ready = false;
                }
                // reguardless, lets move data from the GPS object over to the main consumer object
                consumer_obj.lat_lon_alt =
                    (gps_data.latitude, gps_data.longitude, gps_data.altitude);
            }
            Err(_) => {
                warn!("No GPS Data");
            }
        }
        match DEVICE_DATA_I2C.try_receive() {
            Ok(i2c_data) => {
                // our first concern is to find out if the devices have been initialized properly, and if the data is good
                // if any of the sensors are not active, then we halt sys ready
                if !i2c_data.aht_ready || !i2c_data.gyro_ready || !i2c_data.mag_ready {
                    consumer_obj.sys_ready = false;
                }
                // reguardless, lets copy data over to the main object
                consumer_obj.direction = i2c_data.direction;
                consumer_obj.humidity = i2c_data.humidity;
                consumer_obj.temperature = i2c_data.temperature;
                consumer_obj.rotation = i2c_data.rotation;
            }
            Err(_) => {
                warn!("No I2C Data");
            }
        }
        if !consumer_obj.sys_ready {
            continue;
        } // skip this run if the system is not ready
          // otherwise lets get to work, lets find out from the global state variable, where we are

        Timer::after_millis(100).await;
    }
}
