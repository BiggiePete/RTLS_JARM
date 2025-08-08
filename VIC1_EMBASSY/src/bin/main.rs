#![no_std]
#![no_main]

#[path = "../aht20.rs"]
mod aht20;

#[path = "../icm42688.rs"]
mod icm42688;

#[path = "../inertial_navigator3.rs"]
mod inertial;

#[path = "../gy271.rs"]
mod gy271;


use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::time::{hz, Hertz};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::UartRx;
use embassy_stm32::{bind_interrupts, i2c, peripherals, usart};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[path = "./tasks/get_i2c_task.rs"]
mod get_i2c_task;
use crate::get_i2c_task::gather_i2c_data;

#[path = "./tasks/get_gps_task.rs"]
mod get_gps_task;
use crate::get_gps_task::gather_gps_data;

#[path = "./tasks/data_consumer_task.rs"]
mod data_consumer_task;
use crate::data_consumer_task::consume_data;

#[path = "./tasks/motor_task.rs"]
mod motor_task;
use crate::motor_task::{motor_operation_task, ESC_PWM_FREQ_HZ};

#[derive(Debug, PartialEq)]
enum STATE {
    PREPARE,
    SETUP,
    LAUNCH,
    LANDING,
    FINAL,
}

pub(crate) static GLOBAL_STATE: Mutex<ThreadModeRawMutex, STATE> = Mutex::new(STATE::PREPARE);
// so what we should do here is set up a system of tasks, the tasks likely have the ability to do something on their first run, then do something else in loop
// we will use messages and message queues, to determine the state of the system, so we can have a queue called I2C data, that has the acceleration data, the gyro, and the mag,
// we can then also have another channel called gps
// and another channel called motor_sig, where we update the motor signals, like how much drive should be going to the motor, but we also need to set up a 3 state system

// setup
// launch / ascent
// descent / landing

// the setup stage should blink the LEDs, initialize the motors, and start gathering information about the current GPS location
// the launch and scent stages can only be activated after the button has been pressed in the setup stage.
// in the launch stage, we will track our current altitude and wait until our altitude is > 300ft && over the last 3 readings, our altitude has been decreasing, once this is true, we move to descent
// in the descent stage, we will track our current altitude and attempt to correct for the descent speed, we will initially though, prepare the system by spinning up all the motors for 5 seconds to full power, following this, we will then attempt to correct for our current YAW PITCH ROLL, by trying to become level. once we are level, we can use the mag data to attempt to point north. once we are within 5deg of north, we can begin the descent phase, where we will attempt to monitor and control our descent speed, and maintain a mostly level flight profile, if we need to angle ourselves, it will be in the direction that puts us closer to the landing site, but we will tilt no more than 15deg over the minimum required to break wind.

// we will do all of this while sending back data messages back to the main server over LoRa, and maintaining comms, allowing the user to change settings about the launch, and choose to do different things if necessary.
bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World! from JARM");
    // guarentee that the static object has been reset
    {
        let mut state = GLOBAL_STATE.lock().await;
        *state = STATE::PREPARE;
    }
    // lets initialize all of the components necessary for runtime
    let debug_led1 = Output::new(p.PB13, Level::Low, Speed::Low);
    let debug_led2 = Output::new(p.PB14, Level::Low, Speed::Low);
    let debug_led3 = Output::new(p.PB15, Level::Low, Speed::Low);
    let set_pos_button = Input::new(p.PC8, Pull::Up);

    let _power_select = Output::new(p.PB12, Level::High, Speed::Low);

    // Initialize I2C for sensors
    let mut i2c_config = embassy_stm32::i2c::Config::default();
    i2c_config.timeout = Duration::from_millis(100); // Set a 100ms timeout

    let i2c = I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH0,
        Hertz(100_000),
        i2c_config,
    );
    info!("done configuring I2C");

    let ch1 = PwmPin::new_ch1(p.PA8, OutputType::PushPull);
    let ch2 = PwmPin::new_ch2(p.PA9, OutputType::PushPull);
    let ch3 = PwmPin::new_ch3(p.PA10, OutputType::PushPull);
    let ch4 = PwmPin::new_ch4(p.PA11, OutputType::PushPull); // This is our target motor

    let pwm: SimplePwm<'_, embassy_stm32::peripherals::TIM1> = SimplePwm::new(
        p.TIM1,
        Some(ch1),
        Some(ch2),
        Some(ch3),
        Some(ch4),
        hz(ESC_PWM_FREQ_HZ), // Set correct 50Hz frequency for ESC
        Default::default(),
    );

    info!("done configuring PWM handlers");

    let mut uart_config = embassy_stm32::usart::Config::default(); // Set baud rate etc.
    uart_config.baudrate = 9600;
    let uart_rx = UartRx::new(p.USART2, Irqs, p.PA3, p.DMA1_CH5, uart_config).unwrap();

    info!("done configuring UART");

    spawner.spawn(gather_i2c_data(i2c)).unwrap();
    spawner.spawn(gather_gps_data(uart_rx)).unwrap();
    spawner
        .spawn(consume_data(
            debug_led1,
            debug_led2,
            debug_led3,
            set_pos_button,
        ))
        .unwrap();
    spawner.spawn(motor_operation_task(pwm)).unwrap();

    info!("Done Initializing Tasks");
    loop {
        info!("Tick");
        Timer::after_secs(1).await;
    }
}
