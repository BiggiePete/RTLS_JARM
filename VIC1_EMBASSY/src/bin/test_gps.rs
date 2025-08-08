#![no_std]
#![no_main]

#[path = "../NEO6.rs"]
mod neo6;
use crate::neo6::NeoGps;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::usart::{self, UartRx};
use embassy_stm32::{bind_interrupts, i2c, peripherals};

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World! from JARM");

    let uart_config = embassy_stm32::usart::Config::default(); // Set baud rate etc.
    let mut uart_rx = UartRx::new(p.USART2, Irqs, p.PA3, p.DMA1_CH5, uart_config).unwrap();
    let _ = uart_rx.set_baudrate(9600);

    let mut gps = NeoGps::new(&mut uart_rx);

    // TODO: check to see if the GPS works outdoors, currently, the readings dont come through when inside a building
    loop {
        match gps.read_gga_data().await {
            Ok(gga) => {
                info!(
                    "GPS Data: lat={} lon={} alt={}",
                    gga.latitude.unwrap_or(0.0),
                    gga.longitude.unwrap_or(0.0),
                    gga.altitude_msl.unwrap_or(0.0),
                );
            }
            Err(e) => {
                error!("Failed to read GPS data: {}", defmt::Debug2Format(&e));
            }
        }
    }
}
