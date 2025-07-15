#![no_std]

use core::cell::RefCell;
use defmt::*; // Make sure defmt is in scope for info!, error!, warn!
use embassy_stm32::i2c::{Error as I2cError, I2c as Stm32I2c};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer, WithTimeout};

const GZP6816D_I2C_ADDRESS: u8 = 0x78;
const CMD_GET_CAL: u8 = 0xAC;

// Pressure conversion constants
const PMIN_KPA: f32 = 30.0;
const PMAX_KPA: f32 = 110.0;
const DMIN_ADC: f32 = 1_677_722.0;
const DMAX_ADC: f32 = 15_099_494.0;

// Temperature conversion constants
const TEMP_RAW_DIVISOR: f32 = 65536.0;
const TEMP_CONV_SCALE: f32 = 190.0;
const TEMP_CONV_OFFSET: f32 = -40.0;

// Status byte interpretation
const STATUS_BUSY_BIT: u8 = 0x20;
const STATUS_FIXED_BITS_MASK: u8 = 0b10011011;
const STATUS_FIXED_BITS_EXPECTED_VALUE: u8 = 0b00000100;

// Timing for polling
const INITIAL_DELAY_MS: u64 = 20;
const POLL_INTERVAL_MS: u64 = 20;
const MAX_MEASUREMENT_TIME_MS: u64 = 203; // Datasheet typical measurement time
const POLLING_TIMEOUT_MS: u64 = INITIAL_DELAY_MS + MAX_MEASUREMENT_TIME_MS + 50; // Add some buffer
const MAX_POLL_ATTEMPTS: u32 =
    (((POLLING_TIMEOUT_MS - INITIAL_DELAY_MS) / POLL_INTERVAL_MS) + 1) as u32;

#[derive(Debug, PartialEq)]
pub enum Gzp6816dError {
    I2c(I2cError),
    BusyTimeout,
    InvalidStatusByte(u8),
}

// Implement defmt::Format for Gzp6816dError if not already done
impl Format for Gzp6816dError {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Gzp6816dError::I2c(e) => {
                // Assuming I2cError itself implements Format or you provide a wrapper
                defmt::write!(f, "Gzp6816dError::I2c({:?})", e)
            }
            Gzp6816dError::BusyTimeout => {
                defmt::write!(f, "Gzp6816dError::BusyTimeout")
            }
            Gzp6816dError::InvalidStatusByte(byte) => {
                defmt::write!(f, "Gzp6816dError::InvalidStatusByte({=u8:08b})", byte)
            }
        }
    }
}

impl From<I2cError> for Gzp6816dError {
    fn from(e: I2cError) -> Self {
        Gzp6816dError::I2c(e)
    }
}

pub struct Gzp6816d<'a> {
    i2c: &'a RefCell<Stm32I2c<'static, Async>>,
}

impl<'a> Gzp6816d<'a> {
    pub fn new(i2c: &'a RefCell<Stm32I2c<'static, Async>>) -> Self {
        info!("GZP: Gzp6816d::new called");
        Self { i2c }
    }

    async fn trigger_measurement(&mut self, command: u8) -> Result<(), Gzp6816dError> {
        info!("GZP: trigger_measurement - CMD: {=u8:02X}", command);
        match self
            .i2c
            .borrow_mut()
            .write(GZP6816D_I2C_ADDRESS, &[command])
            .await
        {
            Ok(_) => {
                info!("GZP: trigger_measurement - I2C write successful");
                Ok(())
            }
            Err(e) => {
                error!("GZP: trigger_measurement - I2C write failed: {:?}", e);
                Err(Gzp6816dError::I2c(e))
            }
        }
    }

    async fn read_status_byte_for_polling(&mut self) -> Result<u8, Gzp6816dError> {
        let mut status_buf = [0u8; 1];
        info!("GZP: read_status_byte_for_polling - Attempting I2C read for status");
        match self
            .i2c
            .borrow_mut()
            .read(GZP6816D_I2C_ADDRESS, &mut status_buf)
            .await
        {
            Ok(_) => {
                info!(
                    "GZP: read_status_byte_for_polling - I2C read successful. Status: {=u8:08b}",
                    status_buf[0]
                );
                Ok(status_buf[0])
            }
            Err(e) => {
                error!(
                    "GZP: read_status_byte_for_polling - I2C read failed: {:?}",
                    e
                );
                Err(Gzp6816dError::I2c(e))
            }
        }
    }

    async fn is_busy_polling(&mut self) -> Result<bool, Gzp6816dError> {
        info!("GZP: is_busy_polling - Checking sensor busy status");
        let status_byte = self.read_status_byte_for_polling().await?;
        let busy = (status_byte & STATUS_BUSY_BIT) != 0;
        info!("GZP: is_busy_polling - Sensor busy: {}", busy);
        Ok(busy)
    }

    async fn read_raw_data_after_polling(&mut self) -> Result<(u32, u16), Gzp6816dError> {
        let mut buffer = [0u8; 6];
        info!("GZP: read_raw_data_after_polling - Attempting I2C read for 6 data bytes");
        match self
            .i2c
            .borrow_mut()
            .read(GZP6816D_I2C_ADDRESS, &mut buffer)
            .await
        {
            Ok(_) => {
                info!(
                    "GZP: read_raw_data_after_polling - I2C read successful. Buffer[0] (status): {=u8:08b}",
                    buffer[0]
                );
            }
            Err(e) => {
                error!(
                    "GZP: read_raw_data_after_polling - I2C read failed: {:?}",
                    e
                );
                return Err(Gzp6816dError::I2c(e));
            }
        }

        let status_byte = buffer[0];

        if (status_byte & STATUS_BUSY_BIT) != 0 {
            error!(
                "GZP: Sensor reported busy in final data read. Status: {=u8:08b}",
                status_byte
            );
            return Err(Gzp6816dError::InvalidStatusByte(status_byte));
        }

        // Check fixed bits, but only warn if they are not as expected, as per datasheet note
        // (some variants might have different fixed bits or this might not be critical for data validity)
        if (status_byte & STATUS_FIXED_BITS_MASK) != STATUS_FIXED_BITS_EXPECTED_VALUE {
            warn!(
                "GZP: Unexpected pattern in status byte fixed bits. Status: {=u8:08b}, Expected pattern for masked bits: {=u8:08b}",
                status_byte, STATUS_FIXED_BITS_EXPECTED_VALUE
            );
        }

        let raw_pressure_adc = u32::from_be_bytes([0, buffer[1], buffer[2], buffer[3]]);
        let raw_temp_adc = u16::from_be_bytes([buffer[4], buffer[5]]);
        info!(
            "GZP: read_raw_data_after_polling - Raw P_adc: {}, Raw T_adc: {}",
            raw_pressure_adc, raw_temp_adc
        );

        Ok((raw_pressure_adc, raw_temp_adc))
    }

    pub async fn get_pressure_temperature(&mut self) -> Result<(f32, f32), Gzp6816dError> {
        info!("GZP: get_pressure_temperature - START");

        info!("GZP: get_pressure_temperature - Triggering measurement (CMD_GET_CAL)");
        self.trigger_measurement(CMD_GET_CAL).await?;
        info!(
            "GZP: get_pressure_temperature - Measurement triggered. Waiting initial delay ({}ms)...",
            INITIAL_DELAY_MS
        );

        Timer::after(Duration::from_millis(INITIAL_DELAY_MS)).await;
        info!("GZP: get_pressure_temperature - Initial delay done. Starting polling loop (max attempts: {}).", MAX_POLL_ATTEMPTS);

        let mut attempts = 0;
        loop {
            info!(
                "GZP: get_pressure_temperature - Polling attempt #{}",
                attempts
            );
            match self.is_busy_polling().await {
                Ok(busy) => {
                    if !busy {
                        info!(
                            "GZP: get_pressure_temperature - Sensor NOT busy. Exiting poll loop."
                        );
                        break;
                    }
                    info!("GZP: get_pressure_temperature - Sensor IS busy.");
                }
                Err(e) => {
                    error!(
                        "GZP: get_pressure_temperature - Error during is_busy_polling: {:?}",
                        e
                    );
                    return Err(e); // Propagate I2C error from polling
                }
            }

            attempts += 1;
            if attempts >= MAX_POLL_ATTEMPTS {
                error!(
                    "GZP: Busy timeout after {} poll attempts (total ~{}ms).",
                    attempts,
                    INITIAL_DELAY_MS + attempts as u64 * POLL_INTERVAL_MS
                );
                return Err(Gzp6816dError::BusyTimeout);
            }

            info!(
                "GZP: get_pressure_temperature - Waiting poll interval ({}ms)...",
                POLL_INTERVAL_MS
            );
            Timer::after(Duration::from_millis(POLL_INTERVAL_MS)).await;
            info!("GZP: get_pressure_temperature - Poll interval delay done.");
        }

        info!("GZP: get_pressure_temperature - Polling complete. Reading raw data...");
        let (raw_p_adc, raw_t_adc) = match self.read_raw_data_after_polling().await {
            Ok(data) => data,
            Err(e) => {
                error!(
                    "GZP: get_pressure_temperature - Error reading raw data: {:?}",
                    e
                );
                return Err(e);
            }
        };
        info!(
            "GZP: get_pressure_temperature - Raw data received: P_adc={}, T_adc={}",
            raw_p_adc, raw_t_adc
        );

        let pressure_pa = if raw_p_adc == 0 {
            // As per datasheet or common practice, if ADC is 0, pressure might be considered 0 or invalid.
            // Or handle as an error/specific case if needed.
            warn!("GZP: Raw pressure ADC is 0, reporting 0.0 Pa");
            0.0
        } else {
            let pressure_kpa = ((PMAX_KPA - PMIN_KPA) / (DMAX_ADC - DMIN_ADC))
                * (raw_p_adc as f32 - DMIN_ADC)
                + PMIN_KPA;
            pressure_kpa * 1000.0
        };

        let temperature_c =
            (raw_t_adc as f32 / TEMP_RAW_DIVISOR) * TEMP_CONV_SCALE + TEMP_CONV_OFFSET;

        info!(
            "GZP: get_pressure_temperature - Calculated: Pressure: {} Pa, Temperature: {} C",
            pressure_pa, temperature_c
        );
        info!("GZP: get_pressure_temperature - END");
        Ok((pressure_pa, temperature_c))
    }
}
