use defmt::*;
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::Spi;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

pub static LORA_MESSAGE: Channel<CriticalSectionRawMutex, String<200>, 2> = Channel::new();

#[embassy_executor::task]
pub async fn lora_tx(
    mut spi: Spi<'static, Blocking>,
    mut cs: Output<'static>,
    servo_en_pin: Input<'static>,
    pwm: SimplePwm<'static, embassy_stm32::peripherals::TIM3>,
) {
    trace!("Setting up LoRa Skynet0");

    let channels = pwm.split();
    let mut servo1 = channels.ch1;
    let mut servo2 = channels.ch2;
    let mut servo3 = channels.ch3;
    let mut servo4 = channels.ch4;

    servo1.enable();
    servo2.enable();
    servo3.enable();
    servo4.enable();
    trace!("Servos done initializing");

    loop {
        if servo_en_pin.is_high() {
            // HOLY SHIT DEPLOY THE PARACHUTE
            servo1.set_duty_cycle_fully_on();
            servo2.set_duty_cycle_fully_on();
            servo3.set_duty_cycle_fully_on();
            servo4.set_duty_cycle_fully_on();
        }
        let message = LORA_MESSAGE.receive().await;
        cs.set_high();
        Timer::after_millis(5).await;
        match spi.blocking_write(message.as_bytes()) {
            Ok(_) => {}
            Err(_e) => {
                error!("Failed to write to skynet0")
            }
        };
        Timer::after_millis(5).await;
        cs.set_low();
    }
}
