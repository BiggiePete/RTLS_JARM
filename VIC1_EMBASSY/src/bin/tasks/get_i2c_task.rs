use crate::aht20::AHT20;
use crate::gy271::GY271;
use crate::icm42688::{Icm42688p, ICM42688P_ADDR_AD0_LOW};
use crate::inertial::{CalibrationState, InertialNavigator, Vec3};

use core::cell::RefCell;
use core::f64::consts::PI;
use defmt::*;
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Instant;
use libm::atan2;
use {defmt_rtt as _, panic_probe as _};

#[derive(Debug)]
pub(crate) struct DataMessageI2C {
    pub(crate) temperature: f32,
    pub(crate) humidity: f32,
    // pressure: f32,
    pub(crate) position: (f64, f64, f64), // x, y, z position
    pub(crate) rotation: (f64, f64, f64), // x, y, z rotation
    pub(crate) direction: f32,            // direction in degrees

    // display if a certain sensor is ready
    pub(crate) aht_ready: bool,
    pub(crate) mag_ready: bool,
    pub(crate) gyro_ready: bool,
}

pub static DEVICE_DATA_I2C: Channel<CriticalSectionRawMutex, DataMessageI2C, 2> = Channel::new();

#[embassy_executor::task]
pub async fn gather_i2c_data(i2c: embassy_stm32::i2c::I2c<'static, Async>) {
    trace!("Setting up I2C TASK");
    // DEVICE_DATA.send(DataMessage::Temperature(0.0)).await
    // initialize i2c devices
    let i2c_ref: RefCell<I2c<'static, Async>> = RefCell::new(i2c);
    // initialize any other I2c Devices here
    let mut aht = AHT20::new(&i2c_ref);
    let mut icm42688p = Icm42688p::new(&i2c_ref, ICM42688P_ADDR_AD0_LOW);
    let mut gy271 = GY271::new(&i2c_ref)
        .await
        .expect("Failed to initialize GY-271");
    gy271
        .continuous()
        .await
        .expect("Failed to set GY-271 to continuous mode");
    info!("Finished setting up I2C Vars");

    aht.init().await.unwrap();
    gy271.continuous().await.unwrap();
    icm42688p.init().await.unwrap();
    info!("I2C devices initialized.");

    let mut curr_time = Instant::now(); // get current time in seconds
    let mut dt: f64; // init var

    let mut nav = InertialNavigator::new();

    // nav.set_position(Vec3::new(0.0, 0.0, 0.0));
    while !nav.is_calibrated() {
        match icm42688p.read_all_data().await {
            Ok(d) => {
                let now = Instant::now();

                dt = (now.as_micros() - curr_time.as_micros()) as f64 / 1_000_000.0; // calculate delta time in seconds
                curr_time = now; // update current time
                let (accel, gyro) = (
                    Vec3::new((d.accel.x) as f64, (d.accel.y) as f64, (d.accel.z) as f64),
                    Vec3::new(d.gyro.x as f64, d.gyro.y as f64, d.gyro.z as f64),
                );
                println!("dt: {}", dt);
                nav.update(accel, gyro, dt);
                // nav.force_zero_velocity_update();

                match nav.get_calibration_state() {
                    CalibrationState::WaitingForStability => {
                        // Device needs to be stationary
                        println!(
                            "Please keep device still for calibration... {} {}",
                            defmt::Debug2Format(&accel),
                            defmt::Debug2Format(&gyro)
                        );
                    }
                    CalibrationState::Calibrating => {
                        // Taking samples
                        println!("Calibrating... please wait");
                    }
                    CalibrationState::Complete => {
                        println!("Calibration complete!");
                        break;
                    }
                }
            }
            Err(e) => {
                info!("ICM42688P read error: {}", defmt::Debug2Format(&e));
            }
        }
    }
    loop {
        let mut message = DataMessageI2C {
            temperature: 0.0,
            humidity: 0.0,
            position: (0.0, 0.0, 0.0), // x, y, z position, likely incorrect, but fed from the accelerometer
            rotation: (0.0, 0.0, 0.0), // x, y, z rotation
            direction: 0.0,
            aht_ready: false,
            mag_ready: false,
            gyro_ready: false,
        };
        match icm42688p.read_all_data().await {
            Ok(d) => {
                let now = Instant::now();

                dt = (now.as_micros() - curr_time.as_micros()) as f64 / 1_000_000.0; // calculate delta time in seconds
                curr_time = now; // update current time

                nav.update(
                    Vec3::new(d.accel.x as f64, d.accel.y as f64, d.accel.z as f64),
                    Vec3::new(d.gyro.x as f64, d.gyro.y as f64, d.gyro.z as f64),
                    dt,
                );
                let state = nav.get_state();
                // nav.force_zero_velocity_update();

                info!(
                    "[{:?} {:?} {:?} {:?} {:?} {:?}]",
                    state.position.x,
                    state.position.y,
                    state.position.z,
                    state.rotation.x,
                    state.rotation.y,
                    state.rotation.z
                );

                // add data to the message
                message.position = (state.position.x, state.position.y, state.position.z);
                message.rotation = (
                    state.rotation.x * -1.0,
                    state.rotation.y * -1.0,
                    state.rotation.z,
                );
                message.gyro_ready = true;
            }
            Err(e) => {
                info!("ICM42688P read error: {}", defmt::Debug2Format(&e));
            }
        }
        match gy271.mag().await {
            Ok((x, y, _z)) => {
                let angle_rad = atan2(y as f64, x as f64);

                // twist the compass 180, so south is 0degrees, since we want more range around looking north
                let mut angle_deg = (angle_rad * (180.0 / PI)) - 70.0 + 180.0; // change this multiplier to whatever angle offset is needed

                if angle_deg < 0.0 {
                    angle_deg += 360.0;
                }

                // 5. Print the angle in degrees and the 3-letter cardinal direction.
                info!("Angle: {:?}°", angle_deg);

                // add data to the message
                message.direction = angle_deg as f32;
                message.mag_ready = true;
            }
            Err(e) => {
                error!("Failed to read GY271: {:?}", defmt::Debug2Format(&e));
            }
        }
        match aht.read().await {
            Ok(data) => {
                info!(
                    "AHT20 Data: Temperature: {}°C, Humidity: {}%",
                    data.0, data.1
                );
                message.temperature = data.0;
                message.humidity = data.1;
                message.aht_ready = true;
            }
            Err(e) => {
                error!("Failed to read AHT20: {:?}", defmt::Debug2Format(&e));
            }
        }
        DEVICE_DATA_I2C.send(message).await;
        // Timer::after_millis(100).await;
    }
}
