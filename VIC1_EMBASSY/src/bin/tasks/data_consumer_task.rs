use defmt::*;
use embassy_stm32::usart::UartRx;
use embassy_stm32::{i2c, mode::Async};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

use crate::{get_gps_task::DEVICE_DATA_GPS, get_i2c_task::DEVICE_DATA_I2C};
use crate::{GLOBAL_STATE, STATE};

use crate::motor_task::DEVICE_MOTOR_DATA;
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
    trace!("Starting Consumer Task");
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
        {
            let state = GLOBAL_STATE.lock().await;
            match *state {
                STATE::PREPARE => {
                    info!("System is in PREPARE state");
                    // in the prepare state, lets control the LEDs, and let the user know if we have a GPS lock or not, but otherwise, we do nothing, but wait for the SET_POS button to be pressed
                }
                STATE::SETUP => {
                    info!("System is in SETUP state");
                    // once the SET_POS button has been pressed, we are now going to hold the RED leds until we have 10 readings from the GPS, to get an accurate reading of our current poisition
                    // once we have 10 readings, turn on the WHITE LED, and wait until we see an altitude change of +100ft
                }
                STATE::LAUNCH => {
                    info!("System is in LAUNCH state");
                    // since we are now +100ft above the original starting position, we can now track our altitude and position,
                    // keep a record of our highest altitude, we are going to wait until we have either dropped 30ft from this alt, or if we have lost altitude for 3 readings, in which we will stich to landing mode
                }
                STATE::LANDING => {
                    info!("System is in LANDING state");
                    // in landing mode, we are going to deploy our motors, and attempt to correct for our angle, and try to make ourselves level, once we are level we will:
                    // attempt to face north,
                    // slowly decend, while trying to maintain a position as close as possible to the landing place, treat Lat and Lon as X and Y coords, no need for fancy spherical coords handling
                    // once we are below 30ft of altitude, we will switch to FINAL mode
                }
                STATE::FINAL => {
                    info!("System is in FINAL state")
                    // in final mode, we disable everything, and celebrate, as we are either in one piece, or a million
                }
            }
        }

        Timer::after_millis(100).await;
    }
}
