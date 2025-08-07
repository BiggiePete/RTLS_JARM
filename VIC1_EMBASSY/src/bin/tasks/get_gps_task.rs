#[path = "../../NEO6.rs"]
mod neo6;
use crate::get_gps_task::neo6::NeoGps;

#[path = "./led_task.rs"]
mod led_task;

use defmt::*;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

use {defmt_rtt as _, panic_probe as _};

#[derive(Debug)]
pub(crate) struct DataMessageGPS {
    pub(crate) latitude: f32,
    pub(crate) longitude: f32,
    pub(crate) altitude: f32,
    pub(crate) gps_ready: bool,
}

pub static DEVICE_DATA_GPS: Channel<CriticalSectionRawMutex, DataMessageGPS, 2> = Channel::new();

#[embassy_executor::task]
pub async fn gather_gps_data(mut uart_rx: UartRx<'static, Async>) {
    trace!("Setting up GPS TASK");
    // Initialize GPS here
    // This is a placeholder for GPS initialization logic
    // You would typically use a GPS library to read data from the USART
    info!("Initializing GPS");
    let mut gps = NeoGps::new(&mut uart_rx);
    loop {
        let mut gps_data = DataMessageGPS {
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0.0,
            gps_ready: true,
        };
        // Simulate reading GPS data
        match gps.read_gga_data().await {
            Ok(gga) => {
                info!(
                    "GPS Data: lat={} lon={} alt={}",
                    gga.latitude.unwrap_or(0.0),
                    gga.longitude.unwrap_or(0.0),
                    gga.altitude_msl.unwrap_or(0.0),
                );

                gps_data.latitude = gga.latitude.unwrap_or(0.0);
                gps_data.longitude = gga.longitude.unwrap_or(0.0);
                gps_data.altitude = gga.altitude_msl.unwrap_or(0.0);
                // if any of the data is 0.0 the GPS module must be dead
                if gps_data.latitude == 0.0 || gps_data.longitude == 0.0 || gps_data.altitude == 0.0
                {
                    gps_data.gps_ready = false; // Mark as not ready if no valid data
                } else {
                    gps_data.gps_ready = true; // Mark as ready if valid data is present
                }
            }
            Err(e) => {
                error!("Failed to read GPS data: {}", defmt::Debug2Format(&e));
            }
        }

        DEVICE_DATA_GPS.send(gps_data).await;
        Timer::after_millis(100).await;
    }
}
