#![allow(dead_code)]

use core::cell::RefCell;
use defmt::*;
use embassy_stm32::i2c::{Error as I2cError, I2c};
use embassy_stm32::mode::Async;
use embassy_time::Timer;
use num_traits::float::Float;

// TLV493D-A1B6 Constants
pub const TLV493D_ADDR_LOW: u8 = 0x1F;
pub const TLV493D_ADDR_HIGH: u8 = 0x5E; // Correct for your schematic

const GENERAL_CALL_ADDRESS: u8 = 0x00;
const MASTER_RESET_CMD: u8 = 0xFF;

const NUM_MEASUREMENT_BYTES: usize = 7; // Reg 0x00 - 0x06 (Data + Status)
const NUM_ALL_REGS_BYTES: usize = 11; // Reg 0x00 - 0x0A (Data + Status + Config Read Regs)

// --- Offsets for Read Registers (within the 11-byte read block) ---
// const REG_DATA_BX_MSB: usize = 0;
// ...
const REG_STATUS_OFFSET: usize = 6;
const REG_MOD1_READ_OFFSET: usize = 7;
const REG_MOD1_ADDR_READ_OFFSET: usize = 8;
const REG_MOD2_READ_OFFSET: usize = 9;
const REG_MOD2_ANG_READ_OFFSET: usize = 10;

// --- Bits for WR0 (MOD1_WRITE) ---
const WR0_TST_MODE: u8 = 1 << 0; // Test Mode (must be 0)
const WR0_LP_MODE: u8 = 1 << 1; // Low Power Mode (e.g., 100Hz)
const WR0_TLP_MODE: u8 = 1 << 2; // Trigger Low Power
const WR0_FAST_MODE: u8 = 1 << 3; // Fast Mode (e.g., 1kHz)
const WR0_LOW_BIT: u8 = 1 << 4; // Used with LP for INT config
const WR0_INT_EN: u8 = 1 << 5; // Interrupt Enable

// --- Bits for WR1 (MOD1_ADDR_WRITE) ---
const WR1_Z_EN: u8 = 1 << 0; // Enable Z-axis measurement
const WR1_Y_EN: u8 = 1 << 1; // Enable Y-axis measurement
const WR1_X_EN: u8 = 1 << 2; // Enable X-axis measurement
const WR1_TRIG_OFF: u8 = 0b00 << 3; // Trigger off (for polling / timed reads)
                                    // Other TRIG options exist if using INT pin

// --- Bits for WR2 (MOD2_WRITE) ---
const WR2_PARITY_DISABLE: u8 = 1 << 0; // PF bit (1=disable parity check)

// Status Register (Read Reg 0x06) Bits
const STATUS_PD_FLAG: u8 = 1 << 0; // Power Down flag
const STATUS_TEMP_DATA_READY: u8 = 1 << 2; // Temp measurement data available

// Conversion factors
const MAGNETIC_SENSITIVITY_MT_PER_LSB: f32 = 0.098;
const TEMP_OFFSET_LSB_AT_25C: i16 = 340;
const TEMP_SENSITIVITY_C_PER_LSB: f32 = 1.1;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum PowerMode {
    PowerDown,
    // UltraLowPower modes are more complex to set precisely via LP/TLP/LOW/INT combinations
    // For simplicity, focusing on standard modes:
    LowPower, // ~100Hz, good default
    Fast,     // ~1kHz
}

#[derive(Debug)]
pub enum Tlv493dError {
    I2c(I2cError),
    ConfigError(&'static str),
    DataNotReady, // Could be used if polling status byte
    NotInitialized,
}

impl From<I2cError> for Tlv493dError {
    fn from(e: I2cError) -> Self {
        Tlv493dError::I2c(e)
    }
}

pub struct Tlv493d<'a> {
    i2c: &'a RefCell<I2c<'static, Async>>,
    address: u8,
    // Cache for the 4 Write Registers (WR0, WR1, WR2, WR3)
    write_regs_cache: [u8; 4],
    initialized: bool,
}

impl<'a> Tlv493d<'a> {
    pub fn new(i2c: &'a RefCell<I2c<'static, Async>>, address: u8) -> Self {
        Tlv493d {
            i2c,
            address,
            write_regs_cache: [0; 4], // Populated during init
            initialized: false,
        }
    }

    pub async fn init(&mut self) -> Result<(), Tlv493dError> {
        // 1. Master Reset
        match self
            .i2c
            .borrow_mut()
            .write(GENERAL_CALL_ADDRESS, &[MASTER_RESET_CMD])
            .await
        {
            Ok(_) => info!("TLV493D: Master reset command sent."),
            Err(e) => warn!(
                "TLV493D: Master reset failed (may be normal, e.g., NACK): {:?}",
                e
            ),
        }
        Timer::after_millis(2).await; // t_RES_MASTER

        // 2. First read to wake up and get current register values
        let mut initial_read_buffer = [0u8; NUM_ALL_REGS_BYTES];
        match self
            .i2c
            .borrow_mut()
            .read(self.address, &mut initial_read_buffer)
            .await
        {
            Ok(_) => info!("TLV493D: Initial 11-byte read successful."),
            Err(e) => {
                error!(
                    "TLV493D: Initial 11-byte read failed, cannot proceed: {:?}",
                    e
                );
                return Err(Tlv493dError::I2c(e));
            }
        }
        // Optional: Add a small delay after wake-up read if needed, though often not required before config.
        // Timer::after_millis(1).await;

        // 3. Prepare the desired default configuration values
        let mut wr0 = initial_read_buffer[REG_MOD1_READ_OFFSET];
        wr0 &=
            !(WR0_TST_MODE | WR0_LP_MODE | WR0_TLP_MODE | WR0_FAST_MODE | WR0_LOW_BIT | WR0_INT_EN);
        wr0 |= WR0_LP_MODE;

        let mut wr1 = initial_read_buffer[REG_MOD1_ADDR_READ_OFFSET];
        wr1 &= !0b11111000;
        wr1 |= WR1_X_EN | WR1_Y_EN | WR1_Z_EN;
        wr1 |= WR1_TRIG_OFF;

        let mut wr2 = initial_read_buffer[REG_MOD2_READ_OFFSET];
        wr2 |= WR2_PARITY_DISABLE;

        let wr3 = 0x00; // Force WR3 to 0x00

        // --- Attempting configuration with two 2-byte writes ---

        // 4.A Write the first 2 bytes (WR0, WR1)
        let config_part1: [u8; 2] = [wr0, wr1];
        info!(
            "TLV493D: Preparing to write config Part 1 (2 bytes): WR0={:#04x}, WR1={:#04x}",
            config_part1[0], config_part1[1]
        );

        if let Err(e) = self
            .i2c
            .borrow_mut()
            .write(self.address, &config_part1)
            .await
        {
            error!("TLV493D: Failed to write config Part 1: {:?}", e);
            return Err(Tlv493dError::I2c(e));
        }
        info!("TLV493D: Config Part 1 write successful (apparently).");
        Timer::after_millis(1).await; // Small delay between I2C transactions

        // 4.B Write the next 2 bytes (WR2, WR3)
        // IMPORTANT: The sensor's register pointer might auto-increment or reset after a STOP.
        // For typical I2C register writes, you specify the starting register address for each transaction.
        // The TLV493D configuration registers WR0-WR3 are contiguous from the perspective of a single
        // 4-byte write. It's NOT standard for I2C devices to accept partial config writes
        // to an auto-incrementing pointer across separate transactions for a single logical config block.
        // This attempt assumes the sensor might be tolerant or that the write implicitly targets WR0, then WR1,
        // and the next transaction also starts effectively at "the next config location" or that the sensor
        // specifically handles this sequence. This is highly speculative.

        let config_part2: [u8; 2] = [wr2, wr3];
        info!(
            "TLV493D: Preparing to write config Part 2 (2 bytes): WR2={:#04x}, WR3={:#04x}",
            config_part2[0], config_part2[1]
        );

        if let Err(e) = self
            .i2c
            .borrow_mut()
            .write(self.address, &config_part2)
            .await
        {
            error!("TLV493D: Failed to write config Part 2: {:?}", e);
            // If this fails, the sensor might not have liked the split.
            return Err(Tlv493dError::I2c(e));
        }
        info!("TLV493D: Config Part 2 write successful (apparently).");

        // 5. Update internal cache with the full configuration we attempted
        self.write_regs_cache = [wr0, wr1, wr2, wr3];
        self.initialized = true;

        // 6. Wait for configuration to apply and first conversion
        Timer::after_millis(15).await; // For Low Power mode

        // 7. Verification Step (Highly Recommended - uncomment to use)
        /*
        let mut verify_buffer = [0u8; NUM_ALL_REGS_BYTES];
        match self.i2c.borrow_mut().read(self.address, &mut verify_buffer).await {
            Ok(_) => {
                let actual_wr0 = verify_buffer[REG_MOD1_READ_OFFSET];
                let actual_wr1 = verify_buffer[REG_MOD1_ADDR_READ_OFFSET];
                let actual_wr2 = verify_buffer[REG_MOD2_READ_OFFSET];
                let actual_wr3 = verify_buffer[REG_MOD2_ANG_READ_OFFSET];

                info!("TLV493D: Config after split writes (VERIFIED READ-BACK): MOD1_R={:#04x}, MOD1_ADDR_R={:#04x}, MOD2_R={:#04x}, MOD2_ANG_R={:#04x}",
                      actual_wr0, actual_wr1, actual_wr2, actual_wr3);

                // Update cache with actual values read back
                self.write_regs_cache[0] = actual_wr0;
                self.write_regs_cache[1] = actual_wr1;
                self.write_regs_cache[2] = actual_wr2;
                self.write_regs_cache[3] = actual_wr3;

                let expected_config = [wr0, wr1, wr2, wr3];
                if expected_config[0] != actual_wr0 || expected_config[1] != actual_wr1 ||
                   expected_config[2] != actual_wr2 || expected_config[3] != actual_wr3 {
                    warn!("TLV493D: !!! Mismatch between expected config and read-back config after split writes !!!");
                } else {
                    info!("TLV493D: Verified config matches expected values after split writes.");
                }
            }
            Err(e) => {
                warn!("TLV493D: Failed to read back config for verification after split writes: {:?}", e);
            }
        }
        */

        info!("TLV493D: Initialization sequence with split writes complete.");
        Ok(())
    }

    pub async fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), Tlv493dError> {
        if !self.initialized {
            return Err(Tlv493dError::NotInitialized);
        }

        let mut wr0 = self.write_regs_cache[0]; // Current WR0 (MOD1)

        // Clear previous mode bits
        wr0 &= !(WR0_FAST_MODE | WR0_LP_MODE | WR0_TLP_MODE); // TST, LOW, INT are preserved unless explicitly changed

        match mode {
            PowerMode::PowerDown => {
                // Already cleared Fast and LP. Sensor enters power down if no mode active.
                // TST should be 0. INT could be 0.
                // Note: Official PowerDown is achieved by clearing LP and FAST bits.
            }
            PowerMode::LowPower => {
                wr0 |= WR0_LP_MODE;
            }
            PowerMode::Fast => {
                wr0 |= WR0_FAST_MODE;
            }
        }

        // WR1, WR2, WR3 are kept as they are (axes enabled, parity disabled from init)
        let final_write_buf = [
            wr0,
            self.write_regs_cache[1], // Has X,Y,Z enabled
            self.write_regs_cache[2], // Has Parity Disabled
            self.write_regs_cache[3],
        ];

        self.i2c
            .borrow_mut()
            .write(self.address, &final_write_buf)
            .await
            .map_err(|e| {
                error!("TLV493D: Failed to write power mode config: {:?}", e);
                Tlv493dError::I2c(e)
            })?;
        self.write_regs_cache = final_write_buf;

        Timer::after_millis(match mode {
            PowerMode::Fast => 2,      // t_conv_fast ~1.3ms
            PowerMode::LowPower => 12, // t_conv_lp ~10.4ms
            PowerMode::PowerDown => 1, // Small delay
        })
        .await;
        debug!("TLV493D: Power mode set. WR0={:#04x}", wr0);
        Ok(())
    }

    async fn read_measurement_data_block(
        &mut self,
        buffer: &mut [u8; NUM_MEASUREMENT_BYTES],
    ) -> Result<(), Tlv493dError> {
        if !self.initialized {
            warn!("TLV493D: Reading before init. Attempting auto-init.");
            self.init().await?; // Try to init if not already
        }
        self.i2c.borrow_mut().read(self.address, buffer).await?;
        // trace!("TLV493D: Read measurement block: {:02x?}", buffer); // Debug: print raw data
        Ok(())
    }

    fn parse_measurements(&self, data: &[u8; NUM_MEASUREMENT_BYTES]) -> (i16, i16, i16, i16, u8) {
        let bx_msb = data[0];
        let by_msb = data[1];
        let bz_msb = data[2];
        let temp_msb = data[3];

        let bx_lsb_nib = (data[4] & 0xF0) >> 4;
        let by_lsb_nib = data[4] & 0x0F;

        let bz_lsb_nib = (data[5] & 0xF0) >> 4;
        let temp_lsb_nib = data[5] & 0x0F;

        let raw_bx_u12 = ((bx_msb as u16) << 4) | (bx_lsb_nib as u16);
        let raw_by_u12 = ((by_msb as u16) << 4) | (by_lsb_nib as u16);
        let raw_bz_u12 = ((bz_msb as u16) << 4) | (bz_lsb_nib as u16);
        let raw_temp_u12 = ((temp_msb as u16) << 4) | (temp_lsb_nib as u16);

        let status_byte = data[REG_STATUS_OFFSET];

        (
            twos_complement_12bit(raw_bx_u12),
            twos_complement_12bit(raw_by_u12),
            twos_complement_12bit(raw_bz_u12),
            raw_temp_u12 as i16, // Temp data is typically not two's complement by sensor default
            status_byte,
        )
    }

    pub async fn read_3d_components(&mut self) -> Result<(f32, f32, f32), Tlv493dError> {
        let mut buffer = [0u8; NUM_MEASUREMENT_BYTES];
        self.read_measurement_data_block(&mut buffer).await?;
        let (bx_raw, by_raw, bz_raw, _temp_raw, status) = self.parse_measurements(&buffer);

        // Your TRACE log: `Raw Bx: 0, By: 0, Bz: 3`
        // This means after all the changes, the sensor is *still* reporting these raw values.
        trace!(
            "TLV493D Raw Values: Bx={}, By={}, Bz={}, Status={:#04x}",
            bx_raw,
            by_raw,
            bz_raw,
            status
        );

        let bx_mt = bx_raw as f32 * MAGNETIC_SENSITIVITY_MT_PER_LSB;
        let by_mt = by_raw as f32 * MAGNETIC_SENSITIVITY_MT_PER_LSB;
        let bz_mt = bz_raw as f32 * MAGNETIC_SENSITIVITY_MT_PER_LSB;

        Ok((bx_mt, by_mt, bz_mt))
    }

    pub async fn read_2d_angle_magnitude(&mut self) -> Result<(f32, f32), Tlv493dError> {
        let mut buffer = [0u8; NUM_MEASUREMENT_BYTES];
        self.read_measurement_data_block(&mut buffer).await?;
        let (bx_raw, by_raw, _bz_raw, _temp_raw, status) = self.parse_measurements(&buffer);

        trace!(
            "TLV493D Raw 2D: Bx={}, By={}, Status={:#04x}",
            bx_raw,
            by_raw,
            status
        );
        if bx_raw == 0 && by_raw == 0 {
            warn!("TLV493D: Both Bx and By raw values are 0. Angle calculation will be indeterminate.");
            // Return 0 angle, 0 magnitude, or an error, depending on desired behavior
        }

        let bx_mt = bx_raw as f32 * MAGNETIC_SENSITIVITY_MT_PER_LSB;
        let by_mt = by_raw as f32 * MAGNETIC_SENSITIVITY_MT_PER_LSB;

        let magnitude = (bx_mt.powi(2) + by_mt.powi(2)).sqrt();
        let mut angle_rad = by_mt.atan2(bx_mt);

        if angle_rad < 0.0 {
            angle_rad += 2.0 * core::f32::consts::PI;
        }
        let angle_deg = angle_rad.to_degrees(); // Using num_traits Float::to_degrees()

        Ok((angle_deg, magnitude))
    }

    pub async fn read_temperature(&mut self) -> Result<f32, Tlv493dError> {
        let mut buffer = [0u8; NUM_MEASUREMENT_BYTES];
        self.read_measurement_data_block(&mut buffer).await?;
        let (_bx_raw, _by_raw, _bz_raw, temp_raw, status) = self.parse_measurements(&buffer);

        if (status & STATUS_TEMP_DATA_READY) == 0 {
            warn!("TLV493D: Status byte indicates temperature data may not be ready/valid.");
        }

        let temperature_c =
            (temp_raw - TEMP_OFFSET_LSB_AT_25C) as f32 * TEMP_SENSITIVITY_C_PER_LSB + 25.0;
        trace!("TLV493D Raw Temp: {}, Status={:#04x}", temp_raw, status);
        Ok(temperature_c)
    }
}

fn twos_complement_12bit(val_u12: u16) -> i16 {
    if val_u12 & (1 << 11) != 0 {
        (val_u12 | 0xF000) as i16
    } else {
        val_u12 as i16
    }
}

// Example usage in main.rs would be similar to before,
// ensuring TLV493D_ADDR_HIGH is used for your schematic.
// Make sure to add `num-traits = { version = "0.2", default-features = false, features = ["libm"] }`
// or similar to your Cargo.toml for `to_degrees()` and other float functions.
