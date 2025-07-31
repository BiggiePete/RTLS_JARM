#![no_std]
#![no_main]

#[path = "../aht20.rs"]
mod aht20;
use crate::aht20::AHT20;

#[path = "../GZP6816D.rs"]
mod gz6816d;
use crate::gz6816d::Gzp6816d;

#[path = "../i2c_search.rs"]
mod i2c_search;
use crate::i2c_search::I2cScanner;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;

use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World! from JARM");
    // lets initialize all of the components necessary for runtime

    let mut debug_led1 = Output::new(p.PB15, Level::High, Speed::Low);
    let mut debug_led2 = Output::new(p.PB14, Level::High, Speed::Low);
    let mut debug_led3 = Output::new(p.PB13, Level::High, Speed::Low);

    // Initialize I2C for AHT20 and GZP6816D sensors
    let mut i2c = I2c::new_blocking(p.I2C1, p.PB6, p.PB7, Hertz(100_000), Default::default());
    // let  aht = AHT20::new(&mut i2c);
    // let scanner = I2cScanner::new();

    match i2c.blocking_read(0x47, &mut [0; 1]) {
        Ok(data) => {
            info!("Read data: {:?}", data);
        }
        Err(e) => {
            error!("Failed to read I2C data: {}", e);
        }
    }
    // let scan_results = I2cScanner::scan_bus(&mut i2c);

    // Print the results
    // i2c_search::print_scan_results(&scan_results);

    loop {
        debug_led1.set_low();
        debug_led2.set_low();
        debug_led3.set_low();
        Timer::after(Duration::from_millis(1000)).await;

        debug_led1.set_high();
        debug_led2.set_high();
        debug_led3.set_high();
        Timer::after(Duration::from_millis(1000)).await;
    }
}
