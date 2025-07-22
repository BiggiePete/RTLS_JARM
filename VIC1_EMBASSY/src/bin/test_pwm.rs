#![no_std]
#![no_main]

// --- Unchanged peripheral definitions from your starter ---
// ---

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::time::hz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

// Define standard ESC pulse widths in microseconds
const MIN_PULSE_WIDTH_US: u32 = 1000;
const MAX_PULSE_WIDTH_US: u32 = 2000;
// Standard ESCs expect a 50Hz PWM signal
const ESC_PWM_FREQ_HZ: u32 = 50;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World! from JARM Flight Controller");

    let mut debug_led1 = Output::new(p.PB15, Level::Low, Speed::Low);
    let _power_select = Output::new(p.PB12, Level::High, Speed::Low);

    // --- Initialize PWM ---
    // Note: The specific pins for TIM1 channels are MCU-dependent.
    // These (PA8-PA11) are correct for many common STM32F4s.
    let ch1 = PwmPin::new_ch1(p.PA8, OutputType::PushPull);
    let ch2 = PwmPin::new_ch2(p.PA9, OutputType::PushPull);
    let ch3 = PwmPin::new_ch3(p.PA10, OutputType::PushPull);
    let ch4 = PwmPin::new_ch4(p.PA11, OutputType::PushPull); // This is our target motor

    let mut pwm = SimplePwm::new(
        p.TIM1,
        Some(ch1),
        Some(ch2),
        Some(ch3),
        Some(ch4),
        hz(ESC_PWM_FREQ_HZ), // Set correct 50Hz frequency for ESC
        Default::default(),
    );

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

    // Get a handle to the specific channel for motor 4
    let mut motor4 = pwm.ch4();
    motor4.enable();

    // =================================================================
    //                    ESC CALIBRATION SEQUENCE
    // =================================================================
    //
    // WARNING: Ensure propellers are REMOVED from the motor before running this.
    // The motor will spin up at the end of this sequence.
    //
    // The process:
    // 1. We set the throttle to MAX.
    // 2. You will then power on the ESC.
    // 3. The ESC will beep to register the MAX signal.
    // 4. We then set the throttle to MIN.
    // 5. The ESC will beep again to register the MIN signal and arm.
    //
    info!("--- Starting ESC Calibration for Motor 4 ---");
    warn!("*** REMOVE PROPELLERS FOR SAFETY! ***");
    info!("Setting MAX throttle.");
    motor4.set_duty_cycle(max_throttle_duty);

    info!("You now have 5 seconds to connect power to the ESC...");
    Timer::after_secs(5).await; // Wait for you to power on the ESC

    info!("Setting MIN throttle to arm the ESC.");
    motor4.set_duty_cycle(min_throttle_duty);

    // Give the ESC time to process the command and arm itself.
    // The final beeps from the ESC should happen during this delay.
    Timer::after_secs(5).await;

    info!("ESC calibration complete. Motor should be armed.");
    // =================================================================

    // --- Spin motor up to 10% throttle ---
    info!("Spinning motor to 10% throttle...");

    // Calculate 10% of the throttle range
    let throttle_range = max_throttle_duty - min_throttle_duty;
    let ten_percent_throttle = min_throttle_duty + (throttle_range as u32 * 10 / 100) as u16;

    motor4.set_duty_cycle(ten_percent_throttle);
    info!("Motor 4 spinning at duty cycle: {}", ten_percent_throttle);

    // The motor will continue to spin at 10%.
    // The main loop will now just blink the LED to show the MCU is alive.
    loop {
        debug_led1.toggle();
        Timer::after_millis(500).await;
    }
}
