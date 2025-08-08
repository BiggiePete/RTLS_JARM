#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    // USART2 => usart::InterruptHandler<peripherals::USART2>;
});

// 2 spots
static DEVICE_DATA: Channel<CriticalSectionRawMutex, DataMessage, 2> = Channel::new();

#[derive(Debug)]
struct DataMessage {
    temperature: f32,
    humidity: f32,
    pressure: f32,
    gps: (f32, f32, f32),     // latitude, longitude, altitude
    accel: (f32, f32, f32),   // x, y, z acceleration
    gyro: (f32, f32, f32),    // x, y, z gyroscope
    magnetometer: (f32, f32), // a,r magnetic field
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World! from JARM");
    // lets initialize all of the components necessary for runtime

    let mut debug_led1 = Output::new(p.PB15, Level::High, Speed::Low);
    let mut debug_led2 = Output::new(p.PB14, Level::High, Speed::Low);
    let mut debug_led3 = Output::new(p.PB13, Level::High, Speed::Low);
    let mut button = ExtiInput::new(p.PC8, p.EXTI8, Pull::Up);

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

    debug_led1.set_low();
    debug_led2.set_low();
    debug_led3.set_low();

    button.wait_for_low().await; // Wait for button press

    spawner.spawn(blink(debug_led3)).unwrap();
    spawner.spawn(gather_data(button)).unwrap();
}

#[embassy_executor::task]
async fn blink(mut debug_led3: Output<'static>) {
    info!("Blinking LED3");
    loop {
        debug_led3.set_high();
        Timer::after_secs(2).await;
        debug_led3.set_low();
        Timer::after_secs(2).await;
    }
}

#[embassy_executor::task]
async fn gather_data(mut button: ExtiInput<'static>) {
    info!("waiting on button");
    loop {
        button.wait_for_low().await;
        // Read sensor data and send it through the channel
        info!("Button pressed");
        button.wait_for_high().await; // Wait for button release
        info!("Button released");
    }
}
