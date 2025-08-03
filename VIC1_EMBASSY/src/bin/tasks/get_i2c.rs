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

#[derive(Debug)]
struct DataMessageI2C {
    temperature: f32,
    humidity: f32,
    // pressure: f32,
    position: (f64, f64, f64), // x, y, z position
    rotation: (f64, f64, f64), // x, y, z rotation
    direction: f32,            // direction in degrees

    // display if a certain sensor is ready
    aht_ready: bool,
    mag_ready: bool,
    gyro_ready: bool,
}

static DEVICE_DATA_I2C: Channel<CriticalSectionRawMutex, DataMessageI2C, 2> = Channel::new();

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

#[embassy_executor::task]
pub async fn gather_i2c_data(
    i2c: embassy_stm32::i2c::I2c<'static, Async>,
    mut debug_led1: Output<'static>,
    mut debug_led2: Output<'static>,
    mut debug_led3: Output<'static>,
) {
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

    aht.init().await.unwrap();
    gy271.continuous().await.unwrap();
    icm42688p.init().await.unwrap();
    info!("I2C devices initialized.");

    let mut curr_time = Instant::now(); // get current time in seconds
    let mut dt = 0 as f64; // init var

    let mut nav = InertialNavigator::new();

    // nav.set_position(Vec3::new(0.0, 0.0, 0.0));
    while !nav.is_calibrated() {
        match icm42688p.read_all_data().await {
            Ok(d) => {
                let now = Instant::now();

                dt = (now.as_micros() - curr_time.as_micros()) as f64 / 1_000_000.0; // calculate delta time in seconds
                curr_time = now; // update current time
                let (accel, gyro) = (
                    Vec3::new(
                        (d.accel.x * 9.81) as f64,
                        (d.accel.y * 9.81) as f64,
                        (d.accel.z * 9.81) as f64,
                    ),
                    Vec3::new(d.gyro.x as f64, d.gyro.y as f64, d.gyro.z as f64),
                );
                println!("dt: {}", dt);
                nav.update(accel, gyro, dt);

                match nav.get_calibration_state() {
                    CalibrationState::WaitingForStability => {
                        // Device needs to be stationary
                        println!("Please keep device still for calibration...");
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
                message.rotation = (state.rotation.x, state.rotation.y, state.rotation.z);
                message.gyro_ready = true;
            }
            Err(e) => {
                info!("ICM42688P read error: {}", defmt::Debug2Format(&e));
            }
        }
        match gy271.mag().await {
            Ok((x, y, _z)) => {
                let angle_rad = atan2(y as f64, x as f64);

                let mut angle_deg = (angle_rad * (180.0 / PI)) - 70.0; // change this multiplier to whatever angle offset is needed

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

        // Timer::after_millis(100).await;
    }
}
