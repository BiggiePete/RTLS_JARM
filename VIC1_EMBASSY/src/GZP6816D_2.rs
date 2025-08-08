// GZP6816D_2.rs

use core::cell::RefCell;
use defmt::*;
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::mode::Async;
use embassy_time::Timer;
use num_traits::Float;

const GZP6816D_ADDRESS: u8 = 0x78; // GZP6816D sensor I2C address
const CMD_MEASURE: u8 = 0xA0; // Measurement command

// Pressure range constants from datasheet
const PRESSURE_MIN_KPA: f32 = 30.0; // Minimum range pressure (30 kPa)
const PRESSURE_MAX_KPA: f32 = 110.0; // Maximum range pressure (110 kPa)
const AD_MIN: f32 = 1677722.0; // 10% of 2^24 (minimum AD value)
const AD_MAX: f32 = 15099494.0; // 90% of 2^24 (maximum AD value)

// Temperature range constants from datasheet page 10
// The calculation seems to be: Temp = (raw/65535) * (150 - (-40)) + (-40)
// This simplifies to: Temp = raw/65535 * 190 - 40
// And further: Temp = (raw * 19000 - 4000) / 100 is wrong.
// The example calc on P10: (150 - (-40)) * 33.85% - 40 = 24.32 C. This suggests the formula is `(Tmax-Tmin)*percentage + Tmin`
const TEMP_MIN_C: f32 = -40.0;
const TEMP_MAX_C: f32 = 150.0;

#[derive(Debug, Format)]
pub enum Gzp6816dError {
    I2c(Error),
    MeasurementBusy,
    MeasurementTimeout, // More specific error
    NotPoweredOn,       // We'll keep this but use it differently
    InvalidData,
}

impl From<Error> for Gzp6816dError {
    fn from(e: Error) -> Self {
        Gzp6816dError::I2c(e)
    }
}

/// Status byte bit definitions
#[derive(Debug, Format)]
pub struct StatusByte {
    pub powered_on: bool,        // Bit 6: Power on indicator
    pub measurement_ready: bool, // Bit 5: 0 = ready, 1 = busy
}

impl From<u8> for StatusByte {
    fn from(status: u8) -> Self {
        StatusByte {
            powered_on: (status & 0x40) != 0,        // Bit 6
            measurement_ready: (status & 0x20) == 0, // Bit 5: 0 means ready
        }
    }
}

pub struct GZP6816D<'a> {
    i2c: &'a RefCell<I2c<'static, Async>>,
}

impl<'a> GZP6816D<'a> {
    /// Creates a new GZP6816D driver instance
    pub fn new(i2c: &'a RefCell<I2c<'static, Async>>) -> Self {
        GZP6816D { i2c }
    }

    /// Reads just the status byte from the sensor. Useful for initial checks.
    pub async fn read_status(&mut self) -> Result<StatusByte, Gzp6816dError> {
        let mut status_byte = [0u8; 1];
        self.i2c
            .borrow_mut()
            .read(GZP6816D_ADDRESS, &mut status_byte)
            .await?;
        Ok(StatusByte::from(status_byte[0]))
    }

    /// Checks if the sensor is busy by polling the status register.
    async fn is_busy(&mut self) -> Result<bool, Gzp6816dError> {
        let status = self.read_status().await?;
        Ok(!status.measurement_ready)
    }

    /// Waits for the sensor to complete measurement by polling the busy flag.
    async fn wait_for_measurement(&mut self) -> Result<(), Gzp6816dError> {
        // Datasheet page 5 states measurement time is ~203ms for default OSR.
        // Polling is fine, but we need a timeout.
        for _ in 0..10 {
            // Max wait time of ~250ms (5 * 50ms)
            if !self.is_busy().await? {
                return Ok(());
            }
            Timer::after_millis(25).await;
        }
        error!("GZP6816D: Measurement timeout");
        Err(Gzp6816dError::MeasurementTimeout)
    }

    /// Triggers a measurement and reads pressure (kPa) and temperature (°C).
    /// This is the corrected and robust implementation.
    pub async fn read_measurement(&mut self) -> Result<(f32, f32), Gzp6816dError> {
        // Step 1: Send measurement command
        let cmd = [CMD_MEASURE]; // This should still be 0xA0 for now

        info!("GZP6816D: Attempting to WRITE command..."); // LOG 1
        self.i2c
            .borrow_mut()
            .write(GZP6816D_ADDRESS, &cmd)
            .await
            .map_err(Gzp6816dError::I2c)?;
        info!("GZP6816D: WRITE command successful."); // LOG 2

        // Step 2: Wait for measurement to complete.
        info!("GZP6816D: Waiting for measurement..."); // LOG 3
        Timer::after_millis(205).await;
        info!("GZP6816D: Wait complete."); // LOG 4

        // Step 3: Read measurement data
        info!("GZP6816D: Attempting to READ data..."); // LOG 5
        let mut data = [0u8; 6];
        self.i2c
            .borrow_mut()
            .read(GZP6816D_ADDRESS, &mut data)
            .await
            .map_err(Gzp6816dError::I2c)?;
        info!("GZP6816D: READ data successful."); // LOG 6

        // Step 4: Verify status byte from the read data
        let status = StatusByte::from(data[0]);
        if !status.measurement_ready {
            // This can happen if the wait was too short.
            error!("GZP6816D: Measurement still busy after wait");
            return Err(Gzp6816dError::MeasurementBusy);
        }

        // The powered_on flag being true is a transient state.
        // We can log it but shouldn't treat it as an error.
        if status.powered_on {
            info!("GZP6816D: Sensor reported power-on event.");
        }

        // Step 5: Parse and calculate
        let raw_pressure = ((data[1] as u32) << 16) | ((data[2] as u32) << 8) | (data[3] as u32);
        let raw_temperature = ((data[4] as u16) << 8) | (data[5] as u16);

        // Convert raw pressure to kPa using formula from datasheet
        let pressure_kpa = if raw_pressure != 0 {
            (PRESSURE_MAX_KPA - PRESSURE_MIN_KPA) / (AD_MAX - AD_MIN)
                * (raw_pressure as f32 - AD_MIN)
                + PRESSURE_MIN_KPA
        } else {
            0.0
        };

        // Convert raw temperature to Celsius
        let temp_percentage = raw_temperature as f32 / 65535.0;
        let temperature_c = (temp_percentage * (TEMP_MAX_C - TEMP_MIN_C)) + TEMP_MIN_C;

        Ok((pressure_kpa, temperature_c))
    }

    // --- Convenience methods (no changes needed below this line) ---

    pub async fn read_pressure_pa(&mut self) -> Result<u32, Gzp6816dError> {
        let (pressure_kpa, _) = self.read_measurement().await?;
        Ok((pressure_kpa * 1000.0) as u32)
    }

    pub async fn read_pressure_kpa(&mut self) -> Result<f32, Gzp6816dError> {
        let (pressure_kpa, _) = self.read_measurement().await?;
        Ok(pressure_kpa)
    }

    pub async fn read_temperature(&mut self) -> Result<f32, Gzp6816dError> {
        let (_, temperature_c) = self.read_measurement().await?;
        Ok(temperature_c)
    }

    pub async fn read_altitude_m(&mut self) -> Result<f32, Gzp6816dError> {
        let pressure_kpa = self.read_pressure_kpa().await?;
        let pressure_pa = pressure_kpa * 1000.0;

        const SEA_LEVEL_PRESSURE_PA: f32 = 101325.0;
        let altitude_m = 44330.0 * (1.0 - (pressure_pa / SEA_LEVEL_PRESSURE_PA).powf(0.1903));

        Ok(altitude_m)
    }
}
