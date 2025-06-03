#![allow(dead_code)] // Allow unused constants for clarity

use core::cell::RefCell;
use defmt::*;
use embassy_stm32::i2c::{Error as I2cError, I2c}; // Use Async explicitly
use embassy_stm32::mode::Async;
use embassy_time::Timer;

// These might be in your main.rs or lib.rs, ensure they are accessible
// use defmt_rtt as _;
// use panic_probe as _;

// GZP6816D Constants
const GZP6816D_I2C_ADDRESS_7BIT: u8 = 0x78;

// Commands
const CMD_GET_CAL: u8 = 0xAC; // Start calibrated measurement

// Status byte bit masks
const STATUS_BUSY_BIT: u8 = 1 << 5; // Bit 5: 1 = busy/collecting, 0 = data ready

// Conversion constants
const PMIN_KPA: f32 = 30.0;
const PMAX_KPA: f32 = 110.0;
const DMIN_ADC: f32 = 1_677_722.0; // 10% of 2^24 (  262144 * 0.1 * 2^6 = 1677721.6 for 18bit, 2^24 * 0.1 for 24bit)
                                   // Datasheet Example (C51 code) uses 0x19999A for DMIN and 0xE66666 for DMAX.
                                   // 0x19999A = 1677722
                                   // 0xE66666 = 15099494
                                   // These correspond to 10% and 90% of 2^24 (16777216)
const DMAX_ADC: f32 = 15_099_494.0; // 90% of 2^24

const TEMP_RAW_MAX_F32: f32 = 65536.0; // For 16-bit raw temperature (2^16)
const TEMP_CAL_FACTOR: f32 = 190.0; // (150.0 - (-40.0))
const TEMP_CAL_OFFSET: f32 = -40.0;

// Polling parameters
const POLL_DELAY_MS: u64 = 40; // 40ms polling interval (datasheet: typ conversion time 40ms, max 80ms)
const MAX_POLL_ATTEMPTS: u8 = 50; // 50 attempts * 40ms = 2 seconds timeout

#[derive(Debug)]
pub enum Gzp6816dError {
    I2c(I2cError),
    Timeout,
    DataNotReady,      // If status byte still indicates busy after polling
    MeasurementFailed, // Generic failure
}

impl From<I2cError> for Gzp6816dError {
    fn from(e: I2cError) -> Self {
        Gzp6816dError::I2c(e)
    }
}

pub struct Gzp6816d<'a> {
    i2c: &'a RefCell<I2c<'static, Async>>,
}

impl<'a> Gzp6816d<'a> {
    pub fn new(i2c: &'a RefCell<I2c<'static, Async>>) -> Self {
        Gzp6816d { i2c }
    }

    /// Attempts to "wake up" the sensor or clear a stuck bus condition.
    /// This is based on the original code's approach.
    async fn wake_up_sensor(&mut self) {
        let mut dummy_read_buf = [0u8; 1];
        // Try a read, ignore errors as the bus might be NACKing
        let _ = self
            .i2c
            .borrow_mut()
            .read(GZP6816D_I2C_ADDRESS_7BIT, &mut dummy_read_buf)
            .await;
        // Try a write of a benign value (e.g. a known "stop" or "no-op" if available, or just address)
        // The original code wrote 0x00. This is not a GZP6816D command.
        // A simple write to the address might be enough to reset its internal I2C state.
        // Let's omit the write of 0x00 as it's not standard. A simple write command would be better.
        // If problems persist, consider sending a STOP condition if the HAL allows, or a repeated START.
        // For now, the initial read attempt might be enough.
        Timer::after_millis(5).await; // Small delay after attempting recovery
    }

    /// Reads calibrated pressure in kPa and temperature in Celsius.
    pub async fn read_calibrated_data(&mut self) -> Result<(f32, f32), Gzp6816dError> {
        // Optional: Attempt to wake up/clear bus. Consider if this is truly needed.
        // self.wake_up_sensor().await;

        // 1. Send 0xAC command to start measurement
        self.i2c
            .borrow_mut()
            .write(GZP6816D_I2C_ADDRESS_7BIT, &[CMD_GET_CAL])
            .await
            .map_err(|e| {
                error!("GZP6816D: I2C write CMD_GET_CAL failed: {:?}", e);
                Gzp6816dError::I2c(e)
            })?;

        // 2. Poll for status (measurement complete)
        let mut attempts = 0;
        loop {
            Timer::after_millis(POLL_DELAY_MS).await;
            let mut status_buffer = [0u8; 1];
            match self
                .i2c
                .borrow_mut()
                .read(GZP6816D_I2C_ADDRESS_7BIT, &mut status_buffer)
                .await
            {
                Ok(_) => {
                    if (status_buffer[0] & STATUS_BUSY_BIT) == 0 {
                        // Data ready
                        break;
                    } else {
                        // Still busy
                        trace!(
                            "GZP6816D: Poll attempt {}: status busy ({:#04x})",
                            attempts,
                            status_buffer[0]
                        );
                    }
                }
                Err(e) => {
                    warn!(
                        "GZP6816D: Polling I2C read error: {:?}, attempt {}",
                        e, attempts
                    );
                    // If I2C error during polling, we might want to retry a few times or fail.
                    // For now, we continue polling up to MAX_POLL_ATTEMPTS.
                }
            }
            attempts += 1;
            if attempts >= MAX_POLL_ATTEMPTS {
                error!("GZP6816D: Timeout waiting for sensor data ready.");
                // Attempt one last read of the full data packet for debugging, then return Timeout
                let mut dbg_buf = [0u8; 6];
                match self.i2c.borrow_mut().read(GZP6816D_I2C_ADDRESS_7BIT, &mut dbg_buf).await {
                    Ok(_) => error!("GZP6816D: Final 6-byte read on timeout: [{:#04x}, {:#04x}, {:#04x}, {:#04x}, {:#04x}, {:#04x}]", dbg_buf[0], dbg_buf[1], dbg_buf[2], dbg_buf[3], dbg_buf[4], dbg_buf[5]),
                    Err(_) => error!("GZP6816D: Final 6-byte read on timeout failed."),
                }
                return Err(Gzp6816dError::Timeout);
            }
        }

        // 3. Read the 6-byte data packet (status + 3 P_data + 2 T_data)
        let mut data_buffer = [0u8; 6];
        self.i2c
            .borrow_mut()
            .read(GZP6816D_I2C_ADDRESS_7BIT, &mut data_buffer)
            .await
            .map_err(|e| {
                error!("GZP6816D: I2C read data packet failed: {:?}", e);
                Gzp6816dError::I2c(e)
            })?;

        // Re-check status from the 6-byte read. Bit 5 should be 0 (not busy).
        // Bit 6 should be 1 (calibrated data). Datasheet page 9.
        if (data_buffer[0] & STATUS_BUSY_BIT) != 0 {
            error!(
                "GZP6816D: Data inconsistency. Status byte in packet ({:#04x}) indicates busy.",
                data_buffer[0]
            );
            return Err(Gzp6816dError::DataNotReady);
        }
        if (data_buffer[0] & (1 << 6)) == 0 {
            // Bit 6: Calibrated data flag
            warn!(
                "GZP6816D: Status byte in packet ({:#04x}) indicates data is not calibrated data. This is unexpected after 0xAC.",
                data_buffer[0]
            );
            // Depending on strictness, this could be an error.
        }

        // 4. Parse raw ADC values
        // Pressure data: Bytes 1, 2, 3 (MSB first)
        let raw_pressure_adc = ((data_buffer[1] as u32) << 16)
            | ((data_buffer[2] as u32) << 8)
            | (data_buffer[3] as u32);

        // Temperature data: Bytes 4, 5 (MSB first)
        let raw_temp_adc = ((data_buffer[4] as u16) << 8) | (data_buffer[5] as u16);

        // 5. Convert to physical values
        // Pressure conversion: P(kPa) = [(Pout - Dmin) / (Dmax - Dmin)] * (Pmax - Pmin) + Pmin
        // Ensure raw_pressure_adc is within DMIN_ADC and DMAX_ADC or handle clamping/error
        let pressure_kpa = if raw_pressure_adc == 0 {
            // The C51 example checks if Dtest (raw_pressure_adc) is 0 and returns 0.0.
            // This might be a sensor-specific way to indicate an error or no valid reading.
            warn!("GZP6816D: Raw pressure ADC is 0, interpreting as 0.0 kPa.");
            0.0
        } else if raw_pressure_adc < DMIN_ADC as u32 {
            // Value is below the calibrated 10% range
            // Output Pmin. Alternatively, could return an error or a more complex extrapolation.
            warn!(
                "GZP6816D: Raw pressure ADC ({}) is below DMIN_ADC ({}). Clamping to PMIN_KPA.",
                raw_pressure_adc, DMIN_ADC
            );
            PMIN_KPA
        } else if raw_pressure_adc > DMAX_ADC as u32 {
            // Value is above the calibrated 90% range
            // Output Pmax.
            warn!(
                "GZP6816D: Raw pressure ADC ({}) is above DMAX_ADC ({}). Clamping to PMAX_KPA.",
                raw_pressure_adc, DMAX_ADC
            );
            PMAX_KPA
        } else {
            ((raw_pressure_adc as f32 - DMIN_ADC) / (DMAX_ADC - DMIN_ADC)) * (PMAX_KPA - PMIN_KPA)
                + PMIN_KPA
        };

        // Temperature conversion: T(°C) = (Tout / 2^16) * (Tmax - Tmin) + Tmin
        // Here, Tmax - Tmin = 150 - (-40) = 190 (TEMP_CAL_FACTOR)
        // And Tmin = -40 (TEMP_CAL_OFFSET)
        let temperature_c =
            (raw_temp_adc as f32 / TEMP_RAW_MAX_F32) * TEMP_CAL_FACTOR + TEMP_CAL_OFFSET;

        debug!(
            "GZP6816D: Status={:#04x}, Raw P_adc={}, Raw T_adc={}, P_kPa={}, T_C={}",
            data_buffer[0], raw_pressure_adc, raw_temp_adc, pressure_kpa, temperature_c
        );

        Ok((pressure_kpa, temperature_c))
    }
}
