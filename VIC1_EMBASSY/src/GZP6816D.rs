#![allow(dead_code)] // Allow unused constants for clarity

use cortex_m::asm::delay;
use defmt::*;
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::mode::Blocking; // Assuming you're using Blocking mode

// GZP6816D Constants
const GZP6816D_I2C_ADDRESS_7BIT: u8 = 0x78; // *** CORRECTED ADDRESS ***

// Commands
const CMD_GET_CAL: u8 = 0xAC; // Start calibrated measurement

// Status byte bit masks
const STATUS_BUSY_BIT: u8 = 1 << 5; // Bit 5: 1 = busy/collecting, 0 = data ready

// Conversion constants (from datasheet page 10 & C51 example page 15, 20)
// Pressure
const PMIN_KPA: f32 = 30.0;
const PMAX_KPA: f32 = 110.0;
const DMIN_ADC: f32 = 1677722.0; // 10% of 2^24
const DMAX_ADC: f32 = 15099494.0; // 90% of 2^24

// Temperature
const TEMP_RAW_MAX_F32: f32 = 65536.0; // For 16-bit raw temperature
const TEMP_CAL_FACTOR: f32 = 190.0; // (150.0 - (-40.0))
const TEMP_CAL_OFFSET: f32 = -40.0;

// Polling parameters
// Datasheet indicates ~203ms for default OSR.
// We'll poll. This delay is for cortex_m::asm::delay, units are CPU cycles.
// Adjust cycles based on your CPU clock. e.g., 8000 cycles is ~1ms on an 8MHz clock.
// For a 100MHz clock, 1ms is ~100_000 cycles.
// This example uses a placeholder that results in ~20ms if 8000 cycles = 1ms.
// **ADJUST THIS BASED ON YOUR ACTUAL CPU CLOCK SPEED**
const POLL_DELAY_MS_EQUIVALENT_CYCLES: u32 = 20 * 8000; // Example: 20ms polling interval
const MAX_POLL_ATTEMPTS: u8 = 15; // Example: 15 attempts * 20ms = 300ms timeout

pub struct Gzp6816d {
    i2c: I2c<'static, Blocking>,
}

impl Gzp6816d {
    pub fn new(i2c: I2c<'static, Blocking>) -> Self {
        Gzp6816d { i2c }
    }

    /// Reads calibrated pressure in kPa and temperature in Celsius.
    pub fn read_calibrated_data(&mut self) -> Result<(f32, f32), Error> {
        // 1. Send 0xAC command to start measurement
        self.i2c
            .blocking_write(GZP6816D_I2C_ADDRESS_7BIT, &[CMD_GET_CAL])
            .map_err(|e| {
                error!("GZP6816D: I2C write CMD_GET_CAL failed: {:?}", e);
                e
            })?;

        // 2. Poll for status (measurement complete)
        let mut attempts = 0;
        loop {
            // Delay before polling. The first delay is important as measurement takes time.
            delay(POLL_DELAY_MS_EQUIVALENT_CYCLES); // Ensure this delay is meaningful for your clock

            let mut status_buffer = [0u8; 1];
            // A simple read will fetch the status byte first
            match self
                .i2c
                .blocking_read(GZP6816D_I2C_ADDRESS_7BIT, &mut status_buffer)
            {
                Ok(_) => {
                    if (status_buffer[0] & STATUS_BUSY_BIT) == 0 {
                        // Bit 5 is 0, data is ready
                        break;
                    }
                    // Data not ready, continue polling
                    // trace!("GZP6816D: Polling, status busy: {:#04x}", status_buffer[0]);
                }
                Err(e) => {
                    // If there's an I2C error during polling
                    warn!(
                        "GZP6816D: Polling I2C read error: {:?}, attempt {}",
                        e, attempts
                    );
                    // Depending on the error, you might want to break or retry
                    // If it's a Nack, the device might just not be ready, or address is wrong
                    // If it's a bus error, something more serious might be happening
                }
            }

            attempts += 1;
            if attempts >= MAX_POLL_ATTEMPTS {
                error!("GZP6816D: Timeout waiting for sensor data ready.");
                return Err(Error::Timeout);
            }
        }

        // 3. Read the 6-byte data packet (status + 3 P_data + 2 T_data)
        let mut data_buffer = [0u8; 6];
        self.i2c
            .blocking_read(GZP6816D_I2C_ADDRESS_7BIT, &mut data_buffer)
            .map_err(|e| {
                error!("GZP6816D: I2C read data packet failed: {:?}", e);
                e
            })?;

        // Optional: Re-check status from the 6-byte read
        if (data_buffer[0] & STATUS_BUSY_BIT) != 0 {
            error!(
                "GZP6816D: Data inconsistency. Status byte in packet ({:#04x}) indicates busy.",
                data_buffer[0]
            );
            return Err(Error::Bus); // Or another suitable error like Overrun or custom
        }

        // 4. Parse raw ADC values
        // Pressure data: Bytes 1, 2, 3 (MSB first)
        let raw_pressure_adc = ((data_buffer[1] as u32) << 16)
            | ((data_buffer[2] as u32) << 8)
            | (data_buffer[3] as u32);

        // Temperature data: Bytes 4, 5 (MSB first)
        let raw_temp_adc = ((data_buffer[4] as u16) << 8) | (data_buffer[5] as u16);

        // 5. Convert to physical values
        let pressure_kpa = if raw_pressure_adc == 0 {
            // As per C51 example (page 20, Dtest != 0 check)
            // if Dtest is 0, pressure_kpa is set to 0.0
            0.0
        } else {
            ((PMAX_KPA - PMIN_KPA) / (DMAX_ADC - DMIN_ADC)) * (raw_pressure_adc as f32 - DMIN_ADC)
                + PMIN_KPA
        };

        let temperature_c =
            (raw_temp_adc as f32 / TEMP_RAW_MAX_F32) * TEMP_CAL_FACTOR + TEMP_CAL_OFFSET;

        debug!(
            "GZP6816D: Status={:#04x}, Raw P_adc={}, Raw T_adc={}, P_kPa={}, T_C={}",
            data_buffer[0], raw_pressure_adc, raw_temp_adc, pressure_kpa, temperature_c
        );

        Ok((pressure_kpa, temperature_c))
    }
}
