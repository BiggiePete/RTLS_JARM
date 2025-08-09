use crate::{GLOBAL_STATE, STATE};
use defmt::*;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;

use {defmt_rtt as _, panic_probe as _};

// Define standard ESC pulse widths in microseconds
pub(crate) const MIN_PULSE_WIDTH_US: u32 = 1000;
pub(crate) const MAX_PULSE_WIDTH_US: u32 = 2000;
// Standard ESCs expect a 50Hz PWM signal
pub(crate) const ESC_PWM_FREQ_HZ: u32 = 50;

#[derive(Debug)]
pub(crate) struct DataMessageMotors {
    pub(crate) motor1frac100: u8,
    pub(crate) motor2frac100: u8,
    pub(crate) motor3frac100: u8,
    pub(crate) motor4frac100: u8,
}

pub static DEVICE_MOTOR_DATA: Channel<CriticalSectionRawMutex, DataMessageMotors, 2> =
    Channel::new();

#[embassy_executor::task]
pub async fn motor_operation_task(pwm: SimplePwm<'static, embassy_stm32::peripherals::TIM1>) {
    trace!("Initializing Motors inside the motors task");

    let max_duty = pwm.max_duty_cycle();
    info!("PWM initialized at {} Hz", ESC_PWM_FREQ_HZ);
    info!("Max duty value: {}", max_duty);

    // --- Calculate Duty Cycles for ESC Control ---
    // The period in microseconds is 1,000,000 / frequency
    const PERIOD_US: u32 = 1_000_000 / ESC_PWM_FREQ_HZ;

    // Calculate the duty cycle value corresponding to the min throttle pulse
    let min_throttle_duty = (max_duty as u32 * MIN_PULSE_WIDTH_US / PERIOD_US) as u16;
    // Calculate the duty cycle value corresponding to the max throttle pulse
    let max_throttle_duty = (max_duty as u32 * MAX_PULSE_WIDTH_US / PERIOD_US) as u16;

    info!(
        "Min throttle duty ({} us): {}",
        MIN_PULSE_WIDTH_US, min_throttle_duty
    );
    info!(
        "Max throttle duty ({} us): {}",
        MAX_PULSE_WIDTH_US, max_throttle_duty
    );

    let channels = pwm.split();
    let mut motor1 = channels.ch1;
    let mut motor2 = channels.ch2;
    let mut motor3 = channels.ch3;
    let mut motor4 = channels.ch4;

    // Enable the motors
    motor1.enable();
    motor2.enable();
    motor3.enable();
    motor4.enable();

    // make sure to safely turn down the motors from their last state
    info!("--- Starting ESC Calibration for Motors ---");
    motor1.set_duty_cycle(min_throttle_duty);
    motor2.set_duty_cycle(min_throttle_duty);
    motor3.set_duty_cycle(min_throttle_duty);
    motor4.set_duty_cycle(min_throttle_duty);
    Timer::after_millis(500).await;

    warn!("*** REMOVE PROPELLERS FOR SAFETY! ***");
    info!("Setting MAX throttle.");
    motor1.set_duty_cycle(max_throttle_duty);
    motor2.set_duty_cycle(max_throttle_duty);
    motor3.set_duty_cycle(max_throttle_duty);
    motor4.set_duty_cycle(max_throttle_duty);

    info!("You now have 5 seconds to connect power to the ESC...");
    Timer::after_secs(5).await; // Wait for you to power on the ESC

    info!("Setting MIN throttle to arm the ESC.");
    motor1.set_duty_cycle(min_throttle_duty);
    motor2.set_duty_cycle(min_throttle_duty);
    motor3.set_duty_cycle(min_throttle_duty);
    motor4.set_duty_cycle(min_throttle_duty);

    // Give the ESC time to process the command and arm itself.
    // The final beeps from the ESC should happen during this delay.
    Timer::after_secs(5).await;

    info!("ESC calibration complete. Motor should be armed.");
    loop {
        {
            let state = GLOBAL_STATE.lock().await;
            if *state != STATE::LANDING {
                debug!("Waiting for landing operation to kick the motors on");
            } else {
                break;
            }
        }
        // make sure to keep the sleep in a different scope, otherwise it locks up the other mutex
        Timer::after_millis(1020).await;
    }
    trace!("Time to start using the motors");
    loop {
        trace!("Getting Motor values");
        let motor_fracs = DEVICE_MOTOR_DATA.receive().await;
        // we trust that the input from the tasks responsible for giving the information will give values between 0-100
        // set motors to their allocated strengths
        motor1.set_duty_cycle_percent(motor_fracs.motor1frac100);
        motor2.set_duty_cycle_percent(motor_fracs.motor2frac100);
        motor3.set_duty_cycle_percent(motor_fracs.motor3frac100);
        motor4.set_duty_cycle_percent(motor_fracs.motor4frac100);

        {
            match GLOBAL_STATE.try_lock() {
                Ok(state) => {
                    if *state == STATE::FINAL {
                        motor1.set_duty_cycle(min_throttle_duty);
                        motor2.set_duty_cycle(min_throttle_duty);
                        motor3.set_duty_cycle(min_throttle_duty);
                        motor4.set_duty_cycle(min_throttle_duty);
                        break;
                    }
                }
                Err(_e) => {
                    warn!("Failed to lock state")
                }
            }
        }
    }
}
