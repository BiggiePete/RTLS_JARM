use crate::motor_task::{DataMessageMotors, DEVICE_MOTOR_DATA};
use crate::{get_gps_task::DEVICE_DATA_GPS, get_i2c_task::DEVICE_DATA_I2C};
use crate::{GLOBAL_STATE, STATE};
use advanced_pid::{prelude::*, Pid, PidConfig};
use defmt::*;
use embassy_stm32::gpio::{Input, Output};
use embassy_time::{Instant, Timer};
use heapless::Vec;
use num_traits::Float;
use num_traits::{abs, clamp};
use {defmt_rtt as _, panic_probe as _};

#[macro_export]
macro_rules! deg_to_rad {
    ($degrees:expr) => {
        ($degrees as f64) * (PI / 180.0)
    };
}

/// Converts an expression from radians to degrees.
#[macro_export]
macro_rules! rad_to_deg {
    ($radians:expr) => {
        ($radians as f64) * (180.0 / PI)
    };
}

macro_rules! m_to_ft {
    ($value:expr) => {
        ($value as f32) * (1.0 / 0.3048)
    };
}

macro_rules! ft_to_m {
    ($value:expr) => {
        ($value as f32) * 0.3048
    };
}

#[derive(Debug)]
struct ConsumerObject {
    rotation: (f64, f64, f64),
    lat_lon_alt: (f32, f32, f32),
    direction: f32,   // direction in degrees
    temperature: f32, // in deg C
    humidity: f32,
    sys_ready: bool,
    gps_update: bool,
}

pub const FALLING_COUNT_THRESHOLD: u8 = 3;

#[embassy_executor::task]
pub async fn consume_data(
    mut debug_led1: Output<'static>,
    mut debug_led2: Output<'static>,
    mut debug_led3: Output<'static>,
    set_pos_button: Input<'static>,
) {
    trace!("Starting Consumer Task");
    // allocate variables for the consumer task
    let mut gps_pre_positions: Vec<(f32, f32, f32), 10> = Vec::new();
    let mut gps_home: (f32, f32, f32) = (0.0, 0.0, 0.0);
    let mut highest_alt = 0.0;
    let mut falling_count: u8 = 0;
    let mut corrected_attitude = false;
    let mut dt: f32;
    let mut curr_time = Instant::now(); // get current time in seconds

    let mut pid_yaw = Pid::new(PidConfig::new(0.1, 0.005, 0.01).with_limits(-1.0, 1.0));
    let mut pid_pit = Pid::new(PidConfig::new(0.1, 0.005, 0.01).with_limits(-1.0, 1.0));
    let mut pid_rol = Pid::new(PidConfig::new(0.1, 0.005, 0.01).with_limits(-1.0, 1.0));

    let mut pid_lat = Pid::new(PidConfig::new(0.1, 0.005, 0.01).with_limits(-1.0, 1.0));
    let mut pid_lon = Pid::new(PidConfig::new(0.1, 0.005, 0.01).with_limits(-1.0, 1.0));

    loop {
        let mut consumer_obj = ConsumerObject {
            rotation: (0.0, 0.0, 0.0),
            lat_lon_alt: (0.0, 0.0, 0.0),
            direction: (0.0),
            temperature: (0.0),
            humidity: (0.0),
            sys_ready: true,
            gps_update: true,
        };
        // this is a VERY fast operation, so we are not waiting long
        let i2c_data = DEVICE_DATA_I2C.receive().await;

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

        // lets do a partial update of the GPS data
        match DEVICE_DATA_GPS.try_receive() {
            Ok(gps_data) => {
                // we need to find out if the GPS data is good for us to read, what we can do is create an object with combined data, and see if we can populate every field
                if !gps_data.gps_ready || gps_data.altitude == 0.0 {
                    consumer_obj.sys_ready = false;
                }
                // reguardless, lets move data from the GPS object over to the main consumer object
                consumer_obj.lat_lon_alt =
                    (gps_data.latitude, gps_data.longitude, gps_data.altitude);

                // only grab sensor data if the GPS updates
            }
            Err(_) => {
                warn!("No GPS Data");
                consumer_obj.gps_update = false;
            }
        }

        if !consumer_obj.sys_ready {
            error!("System is partially ready! Skipping this cycle");
            Timer::after_millis(100).await;
            continue;
        } // skip this run if the system is not ready
          // otherwise lets get to work, lets find out from the global state variable, where we are
        {
            let mut state = GLOBAL_STATE.lock().await;
            match *state {
                STATE::PREPARE => {
                    info!("System is in PREPARE state");
                    // this code is for testing
                    if set_pos_button.is_low() {
                        // we are now ready to go to setup mode
                        *state = STATE::SETUP;
                        debug_led2.set_low();
                        debug_led3.set_low();
                    }
                    // in the prepare state, lets control the LEDs, and let the user know if we have a GPS lock or not, but otherwise, we do nothing, but wait for the SET_POS button to be pressed
                    Timer::after_millis(500).await;
                    debug_led2.toggle();
                    debug_led3.toggle();
                }
                STATE::SETUP => {
                    info!("System is in SETUP state");
                    if !consumer_obj.gps_update {
                        warn!("GPS is not ready here, so we will skip");
                        continue; // we dont have GPS data to work with
                    }
                    info!(
                        "Current GPS position: {}, {}, {}",
                        consumer_obj.lat_lon_alt.0,
                        consumer_obj.lat_lon_alt.1,
                        consumer_obj.lat_lon_alt.2
                    );

                    // once the SET_POS button has been pressed, we are now going to hold the RED leds until we have 10 readings from the GPS, to get an accurate reading of our current poisition
                    // once we have 10 readings, turn on the WHITE LED, and wait until we see an altitude change of +100ft

                    // lets get 10 readings
                    if !gps_pre_positions.is_full() {
                        // while the vec isnt full
                        let _ = gps_pre_positions.push(consumer_obj.lat_lon_alt);
                    } else {
                        // we only want to compute this once, so lets make this simple
                        if gps_home == (0.0, 0.0, 0.0) {
                            // if the GPS home has not been set, lets compute it
                            let mut gps_pos_sum = (0.0, 0.0, 0.0);
                            for pos in &gps_pre_positions {
                                trace!("Summing {} {} {}", pos.0, pos.1, pos.2);
                                gps_pos_sum.0 += pos.0;
                                gps_pos_sum.1 += pos.1;
                                gps_pos_sum.2 += pos.2;
                            }
                            // average all of them
                            gps_home = (
                                gps_pos_sum.0 / gps_pre_positions.len() as f32,
                                gps_pos_sum.1 / gps_pre_positions.len() as f32,
                                gps_pos_sum.2 / gps_pre_positions.len() as f32,
                            );
                            debug_led1.set_high();
                            continue;
                        }
                    }
                    // now we will wait until the GPS position is 100ft above the original position
                    info!(
                        "Current Alt: {}, Target Alt: {}",
                        m_to_ft!(consumer_obj.lat_lon_alt.2),
                        m_to_ft!(ft_to_m!(100.0) + gps_home.2)
                    );
                    if consumer_obj.lat_lon_alt.2 > (ft_to_m!(100.0) + gps_home.2) {
                        // we are over 100 ft in increase
                        *state = STATE::LAUNCH;
                        debug_led1.set_low();
                    }
                }
                STATE::LAUNCH => {
                    // since we are now +100ft above the original starting position, we can now track our altitude and position,
                    info!("System is in LAUNCH state");
                    if !consumer_obj.gps_update {
                        continue; // we dont have GPS data to work with
                    }
                    if consumer_obj.lat_lon_alt.2 > highest_alt {
                        highest_alt = consumer_obj.lat_lon_alt.2;
                        falling_count = 0;
                    } else {
                        falling_count += 1;
                    }
                    // keep a record of our highest altitude, we are going to wait until we have either dropped 30ft from this alt, or if we have lost altitude for 3 readings, in which we will stich to landing mode

                    // if we have fallen over 100ft or are lower for 3 consistent readings
                    if (consumer_obj.lat_lon_alt.2 + ft_to_m!(100) < highest_alt)
                        || (falling_count > FALLING_COUNT_THRESHOLD)
                    {
                        // switch to landing mode
                        *state = STATE::LANDING;
                    }
                }
                STATE::LANDING => {
                    info!("System is in LANDING state");
                    let now = Instant::now();

                    dt = (now.as_micros() - curr_time.as_micros()) as f32 / 1_000_000.0; // calculate delta
                    curr_time = now; // update current time

                    // once we are below 30ft of altitude to home, we will switch to FINAL mode
                    // set condition for final mode
                    if consumer_obj.gps_update // if the GPS isnt ready, then this wont work
                        && consumer_obj.lat_lon_alt.2 < gps_home.2 + ft_to_m!(30)
                    {
                        *state = STATE::FINAL;
                    }
                    if !corrected_attitude
                        && abs(consumer_obj.rotation.0) <10.0 // if we are within 10degrees on the X
                        && abs(consumer_obj.rotation.1) <10.0
                    // if we are within 10degrees on the Y
                    // if we are within 10degrees on the Y
                    {
                        corrected_attitude = true;
                    }

                    // general Corrections for Pitch and Roll
                    let pitch_drive = pid_pit.update(0.0, consumer_obj.rotation.0 as f32, dt);
                    let roll_drive = pid_rol.update(0.0, consumer_obj.rotation.1 as f32, dt);
                    let yaw_drive = pid_yaw.update(180.0, consumer_obj.direction, dt);

                    // TODO: add controls for shifting the pitch and yaw drive requirements based on lat and long
                    // TODO: vary the amount the lat and lon can effect the output based on sin and cos of the angle
                    let lat_drive = (pid_lat.update(gps_home.0, consumer_obj.lat_lon_alt.0, dt)); //positive = moving north
                    let lon_drive =
                        pid_lon.update(gps_home.1, consumer_obj.lat_lon_alt.1, dt) * -1.0; //positive = moving east

                    // place motor drives in the 1st quadrant, and apply force as necessary
                    match DEVICE_MOTOR_DATA.try_send(DataMessageMotors {
                        motor1frac100: clamp(
                            ((0.3 + (pitch_drive + roll_drive) * 0.25 + yaw_drive * 0.1) * 100.0)
                                as u8,
                            0,
                            100,
                        ),
                        motor2frac100: clamp(
                            ((0.3 + (pitch_drive - roll_drive) * 0.25 - yaw_drive * 0.1) * 100.0)
                                as u8,
                            0,
                            100,
                        ),
                        motor3frac100: clamp(
                            ((0.3 + ((pitch_drive * -1.0) + roll_drive) * 0.25 - yaw_drive * 0.1)
                                * 100.0) as u8,
                            0,
                            100,
                        ),
                        motor4frac100: clamp(
                            ((0.3 + ((pitch_drive * -1.0) - roll_drive) * 0.25 + yaw_drive * 0.1)
                                * 100.0) as u8,
                            0,
                            100,
                        ),
                    }) {
                        Ok(_) => {
                            trace!("Successfully updated MOTORS")
                        }
                        Err(_e) => {
                            warn!("Failed to send to Motors!");
                        }
                    }

                    //

                    // in landing mode, we are going to deploy our motors, and attempt to correct for our angle, and try to make ourselves level, once we are level we will:
                    // attempt to face north,
                    // slowly decend, while trying to maintain a position as close as possible to the landing place, treat Lat and Lon as X and Y coords, no need for fancy spherical coords handling
                }
                STATE::FINAL => {
                    info!("System is in FINAL state")
                    // in final mode, we disable everything, and celebrate, as we are either in one piece, or a million
                }
            }
        }

        // Timer::after_millis(100).await;
    }
}
