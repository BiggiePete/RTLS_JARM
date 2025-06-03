// src/i2c_search.rs

use defmt::*;
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::mode::Async; // Make sure this is Async
use heapless::Vec;

const MAX_I2C_DEVICES: usize = 16;

#[derive(Debug, PartialEq, Eq, Clone, Copy, defmt::Format)]
pub enum KnownDevice {
    AHT20,
    GZP6816D,
    BMP280,
    SSD1306Oled,
    Pca9685,
    Mpu6050,
    Generic(u8),
    Unknown,
}

impl KnownDevice {
    pub fn from_address(addr: u8) -> Self {
        match addr {
            0x38 => KnownDevice::AHT20,
            0x78 => KnownDevice::GZP6816D,
            0x76 | 0x77 => KnownDevice::BMP280,
            0x3C | 0x3D => KnownDevice::SSD1306Oled,
            0x40 => KnownDevice::Pca9685, // PCA9685 base address
            // For PCA9685 address range, you might want more specific logic if needed
            0x41..=0x7F => KnownDevice::Generic(addr), // Catching others in range, or make more specific
            0x68 | 0x69 => KnownDevice::Mpu6050,
            _ => KnownDevice::Generic(addr),
        }
    }

    pub fn get_address(&self) -> Option<u8> {
        match self {
            KnownDevice::AHT20 => Some(0x38),
            KnownDevice::GZP6816D => Some(0x78),
            KnownDevice::BMP280 => Some(0x76),
            KnownDevice::SSD1306Oled => Some(0x3C),
            KnownDevice::Pca9685 => Some(0x40),
            KnownDevice::Mpu6050 => Some(0x68),
            KnownDevice::Generic(addr) => Some(*addr),
            KnownDevice::Unknown => None,
        }
    }
}

pub struct I2cScanner; // No need for `new` if it has no state

impl I2cScanner {
    // Removed new() as it was empty. If you need state later, add it back.

    pub async fn scan_bus(
        i2c: &mut I2c<'static, Async>,
    ) -> Result<Vec<(u8, KnownDevice), MAX_I2C_DEVICES>, Error> {
        let mut found_devices = Vec::new();
        info!("Starting I2C bus scan (async)...");

        // Buffer for the read probe. Content doesn't matter, only the act of reading.
        let mut probe_buffer = [0u8; 1];

        for addr in 0x01..=0x7F {
            // Standard 7-bit address range, plus GZP6816D's 0x78
            // Attempt to read 1 byte. If a device ACKs its address, this will proceed.
            // If the address is NACKed, `i2c.read` returns `Err(Error::Nack)`.
            match i2c.read(addr, &mut probe_buffer).await {
                Ok(_) => {
                    // Device ACKed its address and the read operation was successful.
                    let device_type = KnownDevice::from_address(addr);
                    info!("Device FOUND at address {:#04x} -> {:?}", addr, device_type);
                    if found_devices.push((addr, device_type)).is_err() {
                        warn!("MAX_I2C_DEVICES limit reached ({}), cannot store more devices. Scan might be incomplete.", MAX_I2C_DEVICES);
                        break;
                    }
                }
                Err(Error::Nack) => {
                    // No device responded at this address (address was NACKed).
                    // This is the expected outcome for empty addresses.
                    // trace!("No device at {:#04x}", addr);
                }
                Err(e) => {
                    // An I2C error other than NACK occurred (e.g., Timeout, Bus error).
                    // This might indicate a problem with the bus, the specific address,
                    // or a device that ACKed but then misbehaved during the read.
                    error!("Error probing address {:#04x} with read: {:?}", addr, e);
                    // Optionally, depending on the error, you might want to stop the scan.
                    // For now, we log and continue, similar to the original behavior.
                }
            }
        }

        if found_devices.is_empty() {
            info!("No I2C devices found on the bus.");
        } else {
            info!(
                "I2C scan complete. Found {} device(s).",
                found_devices.len()
            );
        }
        Ok(found_devices)
    }
}

pub fn print_scan_results(results: &Result<Vec<(u8, KnownDevice), MAX_I2C_DEVICES>, Error>) {
    match results {
        Ok(devices) => {
            if devices.is_empty() {
                info!("Scan successful: No I2C devices detected.");
            } else {
                info!("Scan successful. Detected I2C devices:");
                for (addr, device_type) in devices.iter() {
                    info!("  - Address: {:#04x} ({}): {:?}", addr, addr, device_type);
                }
            }
        }
        Err(e) => {
            error!("I2C scan failed: {:?}", e);
        }
    }
}
