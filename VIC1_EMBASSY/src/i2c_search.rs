use defmt::*;
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::mode::Blocking;
use heapless::Vec; // Assuming blocking I2C for simplicity

const MAX_I2C_DEVICES: usize = 16; // Adjust as needed for your application

// Define an enum for known I2C devices
// Add more devices as needed
#[derive(Debug, PartialEq, Eq, Clone, Copy, defmt::Format)]
pub enum KnownDevice {
    AHT20,       // Example: 0x38
    GZP6816D, // Example: 0x78 (though this is a reserved address for write, it's what the datasheet uses)
    BMP280,   // Example: 0x76 or 0x77
    SSD1306Oled, // Example: 0x3C or 0x3D
    Pca9685,  // Example: 0x40 (base address)
    Mpu6050,  // Example: 0x68 or 0x69
    Generic(u8), // For devices found but not in the specific list
    Unknown,
}

impl KnownDevice {
    // Function to map an address to a KnownDevice
    // This is where you'd maintain your list of device addresses
    pub fn from_address(addr: u8) -> Self {
        match addr {
            0x38 => KnownDevice::AHT20,
            0x78 => KnownDevice::GZP6816D, // Note: Datasheet uses 0x78 as the 7-bit address
            0x76 | 0x77 => KnownDevice::BMP280,
            0x3C | 0x3D => KnownDevice::SSD1306Oled,
            0x40..=0x7F => {
                // PCA9685 has a range of addresses depending on A0-A5 pins
                if addr == 0x40 {
                    KnownDevice::Pca9685
                }
                // Assuming base for now
                else {
                    KnownDevice::Generic(addr)
                }
            }
            0x68 | 0x69 => KnownDevice::Mpu6050,
            _ => KnownDevice::Generic(addr), // If found but not specifically known
        }
    }

    pub fn get_address(&self) -> Option<u8> {
        match self {
            KnownDevice::AHT20 => Some(0x38),
            KnownDevice::GZP6816D => Some(0x78),
            KnownDevice::BMP280 => Some(0x76), // Could be 0x77, this is an example primary
            KnownDevice::SSD1306Oled => Some(0x3C), // Could be 0x3D
            KnownDevice::Pca9685 => Some(0x40),
            KnownDevice::Mpu6050 => Some(0x68),
            KnownDevice::Generic(addr) => Some(*addr),
            KnownDevice::Unknown => None,
        }
    }
}

pub struct I2cScanner {
    // We don't store the I2C bus here to allow the scanner
    // to be used with different bus instances if needed, or
    // to avoid lifetime issues if the bus is managed elsewhere.
    // The bus is passed to the scan method.
}

impl I2cScanner {
    pub fn new() -> Self {
        I2cScanner {}
    }

    /// Scans the I2C bus for devices.
    ///
    /// Args:
    ///   i2c: A mutable reference to the blocking I2C peripheral.
    ///
    /// Returns:
    ///   A `Vec` of `(u8, KnownDevice)` tuples, where `u8` is the 7-bit address
    ///   and `KnownDevice` is the identified or generic device type.
    ///   Returns an I2C Error if a fundamental bus error occurs during probing an address.
    pub fn scan_bus(
        i2c: &mut I2c<'static, Blocking>,
    ) -> Result<Vec<(u8, KnownDevice), MAX_I2C_DEVICES>, Error> {
        let mut found_devices = Vec::new();
        info!("Starting I2C bus scan...");

        // I2C 7-bit addresses range from 0x08 to 0x77.
        // Addresses 0x00-0x07 and 0x78-0x7F are reserved.
        // However, GZP6816D uses 0x78. The `blocking_write` will handle the R/W bit.
        for addr in 0x01..=0x7F {
            // Scan a slightly wider range to catch edge cases like GZP6816D
            if addr < 0x08 && addr != 0x00 { // Skip most reserved low addresses, but allow probing all
                 // debug!("Skipping reserved low address: {:#04x}", addr);
                 // continue;
            }
            if addr > 0x77 {
                // debug!("Skipping reserved high address: {:#04x}", addr);
                // For GZP6816D, its address is 0x78, so we must scan up to 0x78.
                // The general high reserved range is 0x78-0x7F.
            }

            // Attempt a simple write. A single dummy byte (0x00) is often used.
            // Some devices might require a specific register to be written for a "ping",
            // but a simple address ACK check is the most generic.
            // `blocking_write` with an empty slice effectively just checks for an ACK after the address.
            match i2c.blocking_write(addr, &[]) {
                Ok(_) => {
                    // Device acknowledged!
                    let device_type = KnownDevice::from_address(addr);
                    info!("Device FOUND at address {:#04x} -> {:?}", addr, device_type);
                    let _ = found_devices.push((addr, device_type));
                }
                Err(Error::Nack) => {
                    // No device at this address, this is expected for most addresses
                    // trace!("No device at {:#04x}", addr);
                }
                Err(e) => {
                    // A more serious I2C error occurred (e.g., bus busy, arbitration lost)
                    // This might indicate a problem with the bus itself or the specific address probe.
                    error!("Error probing address {:#04x}: {:?}", addr, e);
                    // Optionally, we could return the error immediately or try to continue.
                    // For a simple scanner, let's try to continue but log the error.
                    // return Err(e); // Uncomment to fail fast
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
        Ok(_devices) => match results {
            Ok(devices) => {
                if devices.is_empty() {
                    info!("Scan successful: No I2C devices detected.");
                } else {
                    info!("Scan successful. Detected I2C devices:");
                    for (addr, device_type) in devices {
                        info!("  - Address: {:#04x} ({}): {:?}", addr, addr, device_type);
                    }
                }
            }
            Err(e) => {
                error!("I2C scan failed: {:?}", e);
            }
        },
        Err(e) => {
            error!("I2C scan failed: {:?}", e);
        }
    }
}
