use embassy_stm32::gpio::{Level, Output};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

use {defmt_rtt as _, panic_probe as _};

#[derive(Debug)]
pub enum LedState {
    Off,
    On,
    Blinking,
}

#[derive(Debug)]
pub struct LedMessage {
    pub debug_led1: core::option::Option<LedState>,
    pub debug_led2: core::option::Option<LedState>,
    pub debug_led3: core::option::Option<LedState>,
}

pub static LED_DATA: Channel<CriticalSectionRawMutex, LedMessage, 2> = Channel::new();

#[embassy_executor::task]
pub async fn led_task(
    mut debug_led1: Output<'static>,
    mut debug_led2: Output<'static>,
    mut debug_led3: Output<'static>,
) {
    // loop over the LEDs, and set them to the correct state, if there is a new LED message, see what leds need to be updated, and to what state
    let mut state: LedMessage;
    loop {
        if let Ok(message) = LED_DATA.try_receive() {
            state = message;
        } else {
            // If no message, continue with the last known state
            continue;
        }

        // Update LEDs based on the received message
        if let Some(state1) = state.debug_led1 {
            debug_led1.set_level(match state1 {
                LedState::On => Level::High,
                LedState::Off => Level::Low,
                LedState::Blinking => {
                    Timer::after(Duration::from_millis(500)).await;
                    Level::High // Blink logic can be more complex
                }
            });
        }

        if let Some(state2) = state.debug_led2 {
            debug_led2.set_level(match state2 {
                LedState::On => Level::High,
                LedState::Off => Level::Low,
                LedState::Blinking => {
                    Timer::after(Duration::from_millis(500)).await;
                    Level::High // Blink logic can be more complex
                }
            });
        }

        if let Some(state3) = state.debug_led3 {
            debug_led3.set_level(match state3 {
                LedState::On => Level::High,
                LedState::Off => Level::Low,
                LedState::Blinking => {
                    Timer::after(Duration::from_millis(500)).await;
                    Level::High // Blink logic can be more complex
                }
            });
        }
    }
}
