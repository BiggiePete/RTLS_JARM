use core::cell::RefCell;
use defmt::*;
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer};
use num_traits::float::Float;

// TLV493D I2C addresses (determined by ADDR pin during power-up)
const TLV493D_ADDRESS_LOW: u8 = 0x5E; // ADDR pin low
const TLV493D_ADDRESS_HIGH: u8 = 0x68; // ADDR pin high

// Register addresses
const REG_BX_MSB: u8 = 0x00;
const REG_BY_MSB: u8 = 0x01;
const REG_BZ_MSB: u8 = 0x02;
const REG_TEMP_MSB: u8 = 0x03;
const REG_BX_LSB: u8 = 0x04;
const REG_BY_LSB: u8 = 0x05;
const REG_BZ_LSB: u8 = 0x06;
const REG_TEMP_LSB: u8 = 0x07;
const REG_FACTORY_ID: u8 = 0x08;
const REG_CONFIG: u8 = 0x10;
const REG_MOD1: u8 = 0x11;
const REG_MOD2: u8 = 0x13;

// Configuration bits
const CONFIG_TEMP_DISABLE: u8 = 0x80; // Bit 7: Disable temperature measurement
const CONFIG_LP: u8 = 0x40; // Bit 6: Low power mode
const CONFIG_PT: u8 = 0x20; // Bit 5: Parity test enable

// MOD1 register bits
const MOD1_P: u8 = 0x80; // Bit 7: Parity bit
const MOD1_IICADR: u8 = 0x60; // Bits 6-5: IIC address
const MOD1_PR: u8 = 0x18; // Bits 4-3: Protocol (reserved)
const MOD1_CA: u8 = 0x06; // Bits 2-1: Channel access
const MOD1_INT: u8 = 0x01; // Bit 0: Interrupt enable

// MOD2 register bits
const MOD2_T: u8 = 0x80; // Bit 7: Temperature disable
const MOD2_LP: u8 = 0x40; // Bit 6: Low power mode
const MOD2_PT: u8 = 0x20; // Bit 5: Parity test

// Power modes
#[derive(Debug, Clone, Copy)]
pub enum PowerMode {
    PowerDown,
    UltraLowPower,    // 10Hz update rate
    LowPower,         // 100Hz update rate
    MasterControlled, // Triggered by I2C
}

// Delays from datasheet
const DELAY_POWER_ON_MS: u64 = 1; // Wait after power-on
const DELAY_MEASUREMENT_MS: u64 = 1; // Wait for measurement completion

#[derive(Debug)]
pub enum Tlv493dError {
    I2c(Error),
    InvalidConfiguration,
    NotReady,
    ParityError,
}

impl From<Error> for Tlv493dError {
    fn from(e: Error) -> Self {
        Tlv493dError::I2c(e)
    }
}

#[derive(Debug, Clone)]
pub struct MagneticReading {
    pub bx_mt: f32,         // X-axis magnetic field in mT
    pub by_mt: f32,         // Y-axis magnetic field in mT
    pub bz_mt: f32,         // Z-axis magnetic field in mT
    pub temperature_c: f32, // Temperature in Celsius
}

pub struct TLV493D<'a> {
    i2c: &'a RefCell<I2c<'static, Async>>,
    address: u8,
    initialized: bool,
    temp_enabled: bool,
}

impl<'a> TLV493D<'a> {
    /// Creates a new TLV493D driver
    /// addr_pin_high: true if ADDR pin was high during power-up, false if low
    pub fn new(i2c: &'a RefCell<I2c<'static, Async>>, addr_pin_high: bool) -> Self {
        let address = if addr_pin_high {
            TLV493D_ADDRESS_HIGH
        } else {
            TLV493D_ADDRESS_LOW
        };

        TLV493D {
            i2c,
            address,
            initialized: false,
            temp_enabled: true,
        }
    }

    /// Initialize the sensor with specified power mode
    pub async fn init(&mut self, power_mode: PowerMode) -> Result<(), Tlv493dError> {
        // Small delay after power-on
        Timer::after(Duration::from_millis(DELAY_POWER_ON_MS)).await;

        // Read current configuration to verify communication
        let mut config_data = [0u8; 1];
        self.i2c
            .borrow_mut()
            .write_read(self.address, &[REG_CONFIG], &mut config_data)
            .await
            .map_err(|e| {
                error!("TLV493D: Failed to read config register: {:?}", e);
                Tlv493dError::I2c(e)
            })?;

        info!("TLV493D: Initial config register: 0x{:02X}", config_data[0]);

        // Configure the sensor based on power mode
        self.configure_power_mode(power_mode).await?;

        self.initialized = true;
        info!(
            "TLV493D: Initialized successfully at address 0x{:02X}",
            self.address
        );
        Ok(())
    }

    /// Configure the power mode of the sensor
    async fn configure_power_mode(&mut self, mode: PowerMode) -> Result<(), Tlv493dError> {
        let mut config_val = 0u8;

        match mode {
            PowerMode::PowerDown => {
                // Power down mode - sensor will enter low power state
                config_val = 0x00;
            }
            PowerMode::UltraLowPower => {
                // Ultra low power mode - 10Hz internal triggering
                config_val = CONFIG_LP;
            }
            PowerMode::LowPower => {
                // Low power mode - 100Hz internal triggering
                config_val = 0x00; // Neither LP nor master controlled
            }
            PowerMode::MasterControlled => {
                // Master controlled - triggered by I2C reads
                config_val = 0x00;
            }
        }

        // Keep temperature measurement enabled by default
        if !self.temp_enabled {
            config_val |= CONFIG_TEMP_DISABLE;
        }

        // Write configuration
        self.i2c
            .borrow_mut()
            .write(self.address, &[REG_CONFIG, config_val])
            .await
            .map_err(|e| {
                error!("TLV493D: Failed to write config: {:?}", e);
                Tlv493dError::I2c(e)
            })?;

        info!(
            "TLV493D: Configured power mode, config = 0x{:02X}",
            config_val
        );
        Ok(())
    }

    /// Enable or disable temperature measurement
    pub async fn set_temperature_enabled(&mut self, enabled: bool) -> Result<(), Tlv493dError> {
        self.temp_enabled = enabled;

        // Re-read current config
        let mut current_config = [0u8; 1];
        self.i2c
            .borrow_mut()
            .write_read(self.address, &[REG_CONFIG], &mut current_config)
            .await?;

        let mut new_config = current_config[0];
        if enabled {
            new_config &= !CONFIG_TEMP_DISABLE; // Clear the disable bit
        } else {
            new_config |= CONFIG_TEMP_DISABLE; // Set the disable bit
        }

        self.i2c
            .borrow_mut()
            .write(self.address, &[REG_CONFIG, new_config])
            .await?;

        info!(
            "TLV493D: Temperature measurement {}",
            if enabled { "enabled" } else { "disabled" }
        );
        Ok(())
    }

    /// Read magnetic field and temperature data
    pub async fn read(&mut self) -> Result<MagneticReading, Tlv493dError> {
        if !self.initialized {
            warn!(
                "TLV493D: Sensor not initialized, attempting to initialize with default settings"
            );
            self.init(PowerMode::MasterControlled).await?;
        }

        // For master-controlled mode, trigger a measurement by reading
        // In other modes, measurements happen automatically

        // Read all sensor data (8 bytes: Bx_MSB, By_MSB, Bz_MSB, Temp_MSB, Bx_LSB, By_LSB, Bz_LSB, Temp_LSB)
        let mut data = [0u8; 8];
        self.i2c
            .borrow_mut()
            .write_read(self.address, &[REG_BX_MSB], &mut data)
            .await
            .map_err(|e| {
                error!("TLV493D: Failed to read sensor data: {:?}", e);
                Tlv493dError::I2c(e)
            })?;

        // Parse the magnetic field data (12-bit signed values)
        // Bx: data[0] (MSB) + lower 4 bits of data[4] (LSB)
        let bx_raw = ((data[0] as i16) << 4) | (((data[4] >> 4) & 0x0F) as i16);
        let bx_signed = if bx_raw > 2047 { bx_raw - 4096 } else { bx_raw };

        // By: data[1] (MSB) + lower 4 bits of data[5] (LSB)
        let by_raw = ((data[1] as i16) << 4) | (((data[5] >> 4) & 0x0F) as i16);
        let by_signed = if by_raw > 2047 { by_raw - 4096 } else { by_raw };

        // Bz: data[2] (MSB) + lower 4 bits of data[6] (LSB)
        let bz_raw = ((data[2] as i16) << 4) | (((data[6] >> 4) & 0x0F) as i16);
        let bz_signed = if bz_raw > 2047 { bz_raw - 4096 } else { bz_raw };

        // Temperature: data[3] (MSB) + upper 4 bits of data[7] (LSB)
        let temp_raw = ((data[3] as u16) << 4) | (((data[7] >> 4) & 0x0F) as u16);

        // Convert raw values to physical units
        // Magnetic field: sensitivity is ~10.2 LSB/mT, so 1 LSB = ~0.098 mT
        const MAG_SENSITIVITY: f32 = 0.098; // mT per LSB
        let bx_mt = (bx_signed as f32) * MAG_SENSITIVITY;
        let by_mt = (by_signed as f32) * MAG_SENSITIVITY;
        let bz_mt = (bz_signed as f32) * MAG_SENSITIVITY;

        // Temperature: Digital value @ 25°C is ~340 LSB, resolution is 1.1°C/LSB
        const TEMP_OFFSET: f32 = 340.0; // LSB at 25°C
        const TEMP_SENSITIVITY: f32 = 1.1; // °C per LSB
        let temperature_c = 25.0 + ((temp_raw as f32) - TEMP_OFFSET) * TEMP_SENSITIVITY;

        debug!(
            "TLV493D: Raw data - Bx:{}, By:{}, Bz:{}, Temp:{}",
            bx_signed, by_signed, bz_signed, temp_raw
        );

        Ok(MagneticReading {
            bx_mt,
            by_mt,
            bz_mt,
            temperature_c,
        })
    }

    /// Get the magnetic field magnitude
    pub fn get_magnitude(reading: &MagneticReading) -> f32 {
        (reading.bx_mt * reading.bx_mt
            + reading.by_mt * reading.by_mt
            + reading.bz_mt * reading.bz_mt)
            .sqrt()
    }

    /// Get the azimuth angle (rotation around Z-axis) in degrees
    pub fn get_azimuth_degrees(reading: &MagneticReading) -> f32 {
        let angle_rad = reading.by_mt.atan2(reading.bx_mt);
        angle_rad * 180.0 / core::f32::consts::PI
    }

    /// Get the elevation angle (tilt from XY plane) in degrees  
    pub fn get_elevation_degrees(reading: &MagneticReading) -> f32 {
        let magnitude_xy = (reading.bx_mt * reading.bx_mt + reading.by_mt * reading.by_mt).sqrt();
        let angle_rad = reading.bz_mt.atan2(magnitude_xy);
        angle_rad * 180.0 / core::f32::consts::PI
    }

    /// Perform a soft reset (if supported - this may need I2C reset command)
    pub async fn reset(&mut self) -> Result<(), Tlv493dError> {
        // The TLV493D doesn't have a software reset command in the I2C interface
        // Reset is typically done by power cycling or using the reset pin
        // For now, we'll just reinitialize
        self.initialized = false;
        self.init(PowerMode::MasterControlled).await
    }

    /// Check if sensor is responding
    pub async fn is_connected(&mut self) -> bool {
        let mut test_data = [0u8; 1];
        self.i2c
            .borrow_mut()
            .write_read(self.address, &[REG_FACTORY_ID], &mut test_data)
            .await
            .is_ok()
    }
}
