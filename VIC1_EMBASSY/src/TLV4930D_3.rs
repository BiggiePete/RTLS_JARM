#![no_std]

use core::cell::RefCell;
use embassy_stm32::i2c::{self, Instance as I2cInstance}; // Use the correct I2C trait for embassy-rs
use embassy_time::{Delay, Duration}; // Duration is not strictly used here but good for context
use num_traits::float;

// Default I2C addresses (7-bit format)
pub const TLV493D_ADDRESS_ADDR_HIGH: u8 = 0x5E; // ADDR pin pulled HIGH (as per schematic)
pub const TLV493D_ADDRESS_ADDR_LOW: u8 = 0x1F; // ADDR pin pulled LOW

// Constants for converting raw sensor data to physical units
const B_MULT: f32 = 0.098; // mT per LSB for B-field measurements
const TEMP_MULT: f32 = 1.1; // °C per LSB for Temperature measurements
const TEMP_OFFSET_LSB: i16 = 315; // LSB offset for temperature: (RawTempLSB - Offset) * Multiplier

// Timing constants
const TLV493D_STARTUP_DELAY_MS: u64 = 40; // Delay after power-on before sensor is ready
const TLV493D_RESET_RECOVERY_MS: u64 = 5; // Delay after a soft reset command

// I2C buffer sizes
const MAX_READ_REGS: usize = 10; // Sensor has 10 readable registers (addresses 0x00-0x09)
const CONFIG_WRITE_REGS: usize = 4; // Sensor has 4 writeable MOD registers (MOD1, MOD2, MOD3, MOD4)
const MEASUREMENT_READ_COUNT: usize = 7; // Typically, registers 0-6 are read for Bx,By,Bz,Temp,FrameCounter,ChannelType

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum AccessMode {
    PowerDownMode = 0,
    FastMode,
    LowPowerMode,
    UltraLowPowerMode,
    MasterControlledMode, // Also known as Master Triggered Mode
}

// Configuration parameters for each access mode
struct AccessModeConfig {
    fast: u8,                 // Setting for the FAST bit (MOD1 Register)
    lp: u8,                   // Setting for the LP bit (MOD1 Register)
    lp_period: u8,            // Setting for the LP_Period bit (MOD2 Register)
    measurement_time_ms: u16, // Typical time for one measurement cycle in this mode
}

// Predefined configurations for access modes, mirroring the C++ library's `accModes`
static ACC_MODES_CONFIG: [AccessModeConfig; 5] = [
    AccessModeConfig {
        fast: 0,
        lp: 0,
        lp_period: 0,
        measurement_time_ms: 1000,
    }, // PowerDownMode
    AccessModeConfig {
        fast: 1,
        lp: 0,
        lp_period: 0,
        measurement_time_ms: 1,
    }, // FastMode (Typical 0.5ms cycle time, max 2kHz update rate)
    AccessModeConfig {
        fast: 0,
        lp: 1,
        lp_period: 1,
        measurement_time_ms: 10,
    }, // LowPowerMode (Typical 10ms cycle time, 100Hz update rate)
    AccessModeConfig {
        fast: 0,
        lp: 1,
        lp_period: 0,
        measurement_time_ms: 100,
    }, // UltraLowPowerMode (Typical 100ms cycle time, 10Hz update rate)
    AccessModeConfig {
        fast: 1,
        lp: 1,
        lp_period: 1,
        measurement_time_ms: 10,
    }, // MasterControlledMode (Uses Low Power Mode settings for a single shot)
];

// Differentiates read/write masks if needed, though C++ uses a single struct with an rw field
const REGMASK_READ: u8 = 0;
const REGMASK_WRITE: u8 = 1;

// Defines a bit field within a register byte for easier manipulation
struct RegMask {
    _rw: u8, // REGMASK_READ or REGMASK_WRITE (mainly for type clarity if masks were distinct)
    byte_addr: usize, // Index in the respective read_regs or write_regs buffer
    bit_mask: u8, // Bitmask to isolate the field (e.g., 0xF0 for upper nibble)
    shift: u8, // Right-shift required to get the field's LSB to bit 0
}

// Register masks based on the C++ library's `Tlv493d_conf.h -> regMasks[]`
// IMPORTANT: The `byte_addr` for WRITE masks refers to indices within the 4-byte `write_regs`
// buffer. This buffer is then sent over I2C as [MOD1, MOD2, MOD3, MOD4].
// The C++ masks are used directly, meaning their `byte_addr` values dictate how
// the `write_regs` buffer is populated, aiming to replicate the C++ library's exact behavior.
static REGMASKS: [RegMask; 25] = [
    RegMask {
        _rw: REGMASK_READ,
        byte_addr: 0,
        bit_mask: 0xFF,
        shift: 0,
    }, // R_BX1 (Read Reg 0)
    RegMask {
        _rw: REGMASK_READ,
        byte_addr: 4,
        bit_mask: 0xF0,
        shift: 4,
    }, // R_BX2 (Upper 4 bits of Read Reg 4)
    RegMask {
        _rw: REGMASK_READ,
        byte_addr: 1,
        bit_mask: 0xFF,
        shift: 0,
    }, // R_BY1 (Read Reg 1)
    RegMask {
        _rw: REGMASK_READ,
        byte_addr: 4,
        bit_mask: 0x0F,
        shift: 0,
    }, // R_BY2 (Lower 4 bits of Read Reg 4)
    RegMask {
        _rw: REGMASK_READ,
        byte_addr: 2,
        bit_mask: 0xFF,
        shift: 0,
    }, // R_BZ1 (Read Reg 2)
    RegMask {
        _rw: REGMASK_READ,
        byte_addr: 5,
        bit_mask: 0x0F,
        shift: 0,
    }, // R_BZ2 (Lower 4 bits of Read Reg 5)
    RegMask {
        _rw: REGMASK_READ,
        byte_addr: 3,
        bit_mask: 0xF0,
        shift: 4,
    }, // R_TEMP1 (Upper 4 bits of Read Reg 3)
    RegMask {
        _rw: REGMASK_READ,
        byte_addr: 6,
        bit_mask: 0xFF,
        shift: 0,
    }, // R_TEMP2 (All of Read Reg 6)
    RegMask {
        _rw: REGMASK_READ,
        byte_addr: 3,
        bit_mask: 0x0C,
        shift: 2,
    }, // R_FRAMECOUNTER (Read Reg 3, bits 3:2)
    RegMask {
        _rw: REGMASK_READ,
        byte_addr: 3,
        bit_mask: 0x03,
        shift: 0,
    }, // R_CHANNEL (Read Reg 3, bits 1:0)
    RegMask {
        _rw: REGMASK_READ,
        byte_addr: 5,
        bit_mask: 0x10,
        shift: 4,
    }, // R_POWERDOWNFLAG (Read Reg 5, bit 4)
    RegMask {
        _rw: REGMASK_READ,
        byte_addr: 7,
        bit_mask: 0x18,
        shift: 3,
    }, // R_RES1_RD (Read Reg 7, factory reserved bits)
    RegMask {
        _rw: REGMASK_READ,
        byte_addr: 8,
        bit_mask: 0xFF,
        shift: 0,
    }, // R_RES2_RD (Read Reg 8, factory reserved bits)
    RegMask {
        _rw: REGMASK_READ,
        byte_addr: 9,
        bit_mask: 0x1F,
        shift: 0,
    }, // R_RES3_RD (Read Reg 9, factory reserved bits)
    RegMask {
        _rw: REGMASK_WRITE,
        byte_addr: 1,
        bit_mask: 0x80,
        shift: 7,
    }, // W_PARITY (Targets write_regs[1])
    RegMask {
        _rw: REGMASK_WRITE,
        byte_addr: 1,
        bit_mask: 0x60,
        shift: 5,
    }, // W_ADDR (Diagnosis, not for setting I2C addr; Targets write_regs[1])
    RegMask {
        _rw: REGMASK_WRITE,
        byte_addr: 1,
        bit_mask: 0x04,
        shift: 2,
    }, // W_INT (Interrupt Enable; Targets write_regs[1])
    RegMask {
        _rw: REGMASK_WRITE,
        byte_addr: 1,
        bit_mask: 0x02,
        shift: 1,
    }, // W_FAST (Fast Mode Select; Targets write_regs[1])
    RegMask {
        _rw: REGMASK_WRITE,
        byte_addr: 1,
        bit_mask: 0x01,
        shift: 0,
    }, // W_LOWPOWER (Low Power Mode Select; Targets write_regs[1])
    RegMask {
        _rw: REGMASK_WRITE,
        byte_addr: 3,
        bit_mask: 0x80,
        shift: 7,
    }, // W_TEMP_NEN (Disable Temp Sensor; Targets write_regs[3])
    RegMask {
        _rw: REGMASK_WRITE,
        byte_addr: 3,
        bit_mask: 0x40,
        shift: 6,
    }, // W_LP_PERIOD (Low Power Period Select; Targets write_regs[3])
    RegMask {
        _rw: REGMASK_WRITE,
        byte_addr: 3,
        bit_mask: 0x20,
        shift: 5,
    }, // W_PARITY_EN (Enable parity check by sensor; Targets write_regs[3])
    RegMask {
        _rw: REGMASK_WRITE,
        byte_addr: 1,
        bit_mask: 0x18,
        shift: 3,
    }, // W_RES1_WR (Factory reserved bits; Targets write_regs[1])
    RegMask {
        _rw: REGMASK_WRITE,
        byte_addr: 2,
        bit_mask: 0xFF,
        shift: 0,
    }, // W_RES2_WR (Factory reserved bits; Targets write_regs[2])
    RegMask {
        _rw: REGMASK_WRITE,
        byte_addr: 3,
        bit_mask: 0x1F,
        shift: 0,
    }, // W_RES3_WR (Factory reserved bits; Targets write_regs[3])
];

// Symbolic names for REGMASKS array indices for improved code readability
const IDX_R_BX1: usize = 0;
const IDX_R_BX2: usize = 1;
const IDX_R_BY1: usize = 2;
const IDX_R_BY2: usize = 3;
const IDX_R_BZ1: usize = 4;
const IDX_R_BZ2: usize = 5;
const IDX_R_TEMP1: usize = 6;
const IDX_R_TEMP2: usize = 7;
// const IDX_R_FRAMECOUNTER: usize = 8; // Available, but not directly exposed in this API
// const IDX_R_CHANNEL: usize = 9;      // Available, but not directly exposed in this API
const IDX_R_RES1_RD: usize = 11;
const IDX_R_RES2_RD: usize = 12;
const IDX_R_RES3_RD: usize = 13;
const IDX_W_PARITY: usize = 14;
const IDX_W_INT: usize = 16;
const IDX_W_FAST: usize = 17;
const IDX_W_LOWPOWER: usize = 18;
const IDX_W_TEMP_NEN: usize = 19;
const IDX_W_LP_PERIOD: usize = 20;
const IDX_W_PARITY_EN: usize = 21;
const IDX_W_RES1_WR: usize = 22;
const IDX_W_RES2_WR: usize = 23;
const IDX_W_RES3_WR: usize = 24;

#[derive(Debug)]
pub enum TlvError<E> {
    I2cError(E), // Encapsulates I2C communication errors
    FrameError, // Indicates inconsistent data read (e.g., values from different measurement frames)
}

pub struct Tlv493d<I2C, E>
where
    I2C: I2cInstance<Error = E>, // Requires an I2C peripheral that implements embassy's I2cInstance trait
{
    i2c_ref: RefCell<I2C>, // Mutable reference to the I2C bus, shared via RefCell
    address: u8,           // I2C address of the sensor
    read_regs: [u8; MAX_READ_REGS], // Buffer for data read from the sensor
    write_regs: [u8; CONFIG_WRITE_REGS], // Buffer for configuration data to be written (MOD1-4)
    current_mode: AccessMode, // Current operational mode of the sensor
}

impl<I2C, E> Tlv493d<I2C, E>
where
    I2C: I2cInstance<Error = E>,
{
    /// Creates a new Tlv493d driver instance and initializes the sensor.
    pub async fn new(
        i2c_dev: RefCell<I2C>,
        address: u8, // Typically TLV493D_ADDRESS_ADDR_HIGH (0x5E) or TLV493D_ADDRESS_ADDR_LOW (0x1F)
        delay_source: &mut Delay, // Embassy's Delay provider for timed delays
        perform_soft_reset_for_addr_select: bool, // If true, attempts I2C address selection via general call
    ) -> Result<Self, TlvError<E>> {
        let mut sensor = Self {
            i2c_ref: i2c_dev,
            address,
            read_regs: [0; MAX_READ_REGS],
            write_regs: [0; CONFIG_WRITE_REGS], // Initialized to all zeros
            current_mode: AccessMode::PowerDownMode, // Initial assumed state before configuration
        };

        delay_source.delay_ms(TLV493D_STARTUP_DELAY_MS).await;

        if perform_soft_reset_for_addr_select {
            sensor.perform_i2c_address_selection_reset().await?;
            delay_source.delay_ms(TLV493D_RESET_RECOVERY_MS).await; // Allow time for sensor to recover from reset
        }

        // Read all 10 registers to get factory settings and current sensor state
        sensor
            .i2c_ref
            .borrow_mut()
            .read(sensor.address, &mut sensor.read_regs)
            .await
            .map_err(TlvError::I2cError)?;

        // Copy factory-programmed reserved values from read registers to the write_regs buffer cache.
        // This mirrors the C++ library's `begin()` logic, ensuring these values are preserved
        // when other configuration bits are modified.
        let r_res1_val = sensor.get_bits_from_buffer(IDX_R_RES1_RD, &sensor.read_regs);
        sensor.set_bits_in_buffer(IDX_W_RES1_WR, r_res1_val, &mut sensor.write_regs);

        let r_res2_val = sensor.get_bits_from_buffer(IDX_R_RES2_RD, &sensor.read_regs);
        sensor.set_bits_in_buffer(IDX_W_RES2_WR, r_res2_val, &mut sensor.write_regs);

        let r_res3_val = sensor.get_bits_from_buffer(IDX_R_RES3_RD, &sensor.read_regs);
        sensor.set_bits_in_buffer(IDX_W_RES3_WR, r_res3_val, &mut sensor.write_regs);

        // Enable the sensor's internal parity checking for write operations (as per C++ library)
        sensor.set_bits_in_buffer(IDX_W_PARITY_EN, 1, &mut sensor.write_regs);

        // Set a known default operational mode. The C++ library defaults to PowerDownMode after initialization.
        sensor.set_access_mode(AccessMode::PowerDownMode).await?;

        Ok(sensor)
    }

    // Performs a soft reset using the I2C general call address (0x00).
    // The byte sent after the general call (0xFF or 0x00) selects the sensor's I2C address.
    async fn perform_i2c_address_selection_reset(&mut self) -> Result<(), TlvError<E>> {
        let reset_command_byte = if self.address == TLV493D_ADDRESS_ADDR_LOW {
            0x00
        } else {
            0xFF
        };
        // Address 0x00 is the I2C General Call Address
        self.i2c_ref
            .borrow_mut()
            .write(0x00, &[reset_command_byte])
            .await
            .map_err(TlvError::I2cError)
    }

    // Helper to get a bit field from a buffer (typically self.read_regs)
    fn get_bits_from_buffer(&self, reg_mask_index: usize, buffer: &[u8]) -> u8 {
        let mask_def = ®MASKS[reg_mask_index];
        (buffer[mask_def.byte_addr] & mask_def.bit_mask) >> mask_def.shift
    }

    // Helper to set a bit field in a buffer (typically self.write_regs)
    fn set_bits_in_buffer(&self, reg_mask_index: usize, data: u8, buffer: &mut [u8]) {
        let mask_def = ®MASKS[reg_mask_index];
        let current_byte_val = buffer[mask_def.byte_addr];
        // Clear the bits in the field, then OR with the new data shifted into place
        let new_byte_val = (current_byte_val & !mask_def.bit_mask)
            | ((data << mask_def.shift) & mask_def.bit_mask);
        buffer[mask_def.byte_addr] = new_byte_val;
    }

    // Calculates odd parity over the `self.write_regs` buffer and sets the W_PARITY bit.
    fn calculate_and_set_parity_bit(&mut self) {
        // Initialize the W_PARITY bit to 1. The subsequent algorithm calculates an even parity value.
        // By setting W_PARITY to 1 first, the final stored value effectively creates an odd parity check.
        self.set_bits_in_buffer(IDX_W_PARITY, 1, &mut self.write_regs);

        let mut combined_byte_for_parity_calc: u8 = 0;
        // The C++ library calculates parity over all 4 bytes of its `regWriteData` buffer.
        for i in 0..CONFIG_WRITE_REGS {
            combined_byte_for_parity_calc ^= self.write_regs[i]; // XOR all bytes together
        }

        // Compute parity of the `combined_byte_for_parity_calc` itself (an XNOR sum of its bits)
        combined_byte_for_parity_calc =
            combined_byte_for_parity_calc ^ (combined_byte_for_parity_calc >> 4);
        combined_byte_for_parity_calc =
            combined_byte_for_parity_calc ^ (combined_byte_for_parity_calc >> 2);
        combined_byte_for_parity_calc =
            combined_byte_for_parity_calc ^ (combined_byte_for_parity_calc >> 1);
        // The LSB of `combined_byte_for_parity_calc` now holds the even parity. This is written to W_PARITY.
        self.set_bits_in_buffer(
            IDX_W_PARITY,
            combined_byte_for_parity_calc & 0x01,
            &mut self.write_regs,
        );
    }

    // Writes the `self.write_regs` buffer (MOD1-4) to the sensor after calculating parity.
    async fn write_mod_registers_to_sensor(&mut self) -> Result<(), TlvError<E>> {
        self.calculate_and_set_parity_bit();
        self.i2c_ref
            .borrow_mut()
            .write(self.address, &self.write_regs)
            .await
            .map_err(TlvError::I2cError)
    }

    /// Sets the operational mode of the sensor.
    pub async fn set_access_mode(&mut self, mode: AccessMode) -> Result<(), TlvError<E>> {
        let mode_config_idx = mode as usize;
        let config = &ACC_MODES_CONFIG[mode_config_idx];

        // Apply mode-specific bit settings to the write_regs buffer cache
        self.set_bits_in_buffer(IDX_W_FAST, config.fast, &mut self.write_regs);
        self.set_bits_in_buffer(IDX_W_LOWPOWER, config.lp, &mut self.write_regs);
        self.set_bits_in_buffer(IDX_W_LP_PERIOD, config.lp_period, &mut self.write_regs);

        self.write_mod_registers_to_sensor().await?; // Write the updated configuration to the sensor
        self.current_mode = mode; // Update the stored current mode
        Ok(())
    }

    /// Returns the recommended delay (in ms) for the current access mode before reading new data.
    pub fn get_measurement_delay_ms(&self) -> u16 {
        ACC_MODES_CONFIG[self.current_mode as usize].measurement_time_ms
    }

    /// Enables the interrupt signal generation by the sensor.
    pub async fn enable_interrupt(&mut self) -> Result<(), TlvError<E>> {
        self.set_bits_in_buffer(IDX_W_INT, 1, &mut self.write_regs);
        self.write_mod_registers_to_sensor().await
    }

    /// Disables the interrupt signal generation by the sensor.
    pub async fn disable_interrupt(&mut self) -> Result<(), TlvError<E>> {
        self.set_bits_in_buffer(IDX_W_INT, 0, &mut self.write_regs);
        self.write_mod_registers_to_sensor().await
    }

    /// Enables the temperature measurement.
    pub async fn enable_temp_measurement(&mut self) -> Result<(), TlvError<E>> {
        self.set_bits_in_buffer(IDX_W_TEMP_NEN, 0, &mut self.write_regs); // 0 enables temperature measurement
        self.write_mod_registers_to_sensor().await
    }

    /// Disables the temperature measurement to reduce power consumption.
    pub async fn disable_temp_measurement(&mut self) -> Result<(), TlvError<E>> {
        self.set_bits_in_buffer(IDX_W_TEMP_NEN, 1, &mut self.write_regs); // 1 disables temperature measurement
        self.write_mod_registers_to_sensor().await
    }

    /// Reads measurement registers (0-6) from the sensor into the internal `read_regs` buffer.
    pub async fn update_data(&mut self) -> Result<(), TlvError<E>> {
        // Note: If the sensor is in PowerDownMode, the C++ library includes logic to temporarily
        // switch modes for a measurement. This simplified version expects the user to manage modes.
        // Reading while in PowerDownMode without an explicit wake-up command or mode change
        // might yield stale or zero data.
        let mut measurement_buffer = [0u8; MEASUREMENT_READ_COUNT]; // Buffer for registers 0-6
        self.i2c_ref
            .borrow_mut()
            .read(self.address, &mut measurement_buffer)
            .await
            .map_err(TlvError::I2cError)?;

        // Copy the newly read measurement data into the main read_regs buffer
        self.read_regs[0..MEASUREMENT_READ_COUNT].copy_from_slice(&measurement_buffer);

        // TODO: Optionally, check R_FRAMECOUNTER and R_CHANNEL here for data consistency
        // and return TlvError::FrameError if issues are detected.
        Ok(())
    }

    // Concatenates MSB and LSB parts from registers into a 12-bit value and performs sign extension.
    fn concat_to_i12(&self, msb_part_val: u8, lsb_part_val: u8, is_b_field_data: bool) -> i16 {
        let raw_12bit_pattern: u16 = if is_b_field_data {
            // For B-field: msb_part_val (8 bits from Bx1, By1, or Bz1) are the MSBs.
            //              lsb_part_val (4 bits from Bx2, By2, or Bz2) are the LSBs.
            ((msb_part_val as u16) << 4) | (lsb_part_val as u16)
        } else {
            // For Temperature: msb_part_val (4 bits from Temp1) are the MSBs.
            //                  lsb_part_val (8 bits from Temp2) are the LSBs.
            ((msb_part_val as u16) << 8) | (lsb_part_val as u16)
        };

        // Sign extend the 12-bit pattern (stored in the lower bits of u16) to a full i16.
        if (raw_12bit_pattern & 0x0800) != 0 {
            // Check bit 11 (0-indexed), which is the sign bit for 12-bit
            (raw_12bit_pattern | 0xF000) as i16 // Extend with 1s in the upper 4 bits to make it negative in i16
        } else {
            raw_12bit_pattern as i16 // Positive number, no change needed in upper bits
        }
    }

    /// Gets the raw 12-bit signed integer for the X-axis magnetic field from buffered data.
    pub fn get_x_raw(&self) -> i16 {
        let bx1_val = self.get_bits_from_buffer(IDX_R_BX1, &self.read_regs); // Full 8 bits
        let bx2_val = self.get_bits_from_buffer(IDX_R_BX2, &self.read_regs); // 4-bit value (already right-shifted by get_bits_from_buffer)
        self.concat_to_i12(bx1_val, bx2_val, true)
    }

    /// Gets the raw 12-bit signed integer for the Y-axis magnetic field from buffered data.
    pub fn get_y_raw(&self) -> i16 {
        let by1_val = self.get_bits_from_buffer(IDX_R_BY1, &self.read_regs);
        let by2_val = self.get_bits_from_buffer(IDX_R_BY2, &self.read_regs);
        self.concat_to_i12(by1_val, by2_val, true)
    }

    /// Gets the raw 12-bit signed integer for the Z-axis magnetic field from buffered data.
    pub fn get_z_raw(&self) -> i16 {
        let bz1_val = self.get_bits_from_buffer(IDX_R_BZ1, &self.read_regs);
        let bz2_val = self.get_bits_from_buffer(IDX_R_BZ2, &self.read_regs);
        self.concat_to_i12(bz1_val, bz2_val, true)
    }

    /// Gets the raw 12-bit signed integer for the temperature from buffered data.
    pub fn get_temp_raw(&self) -> i16 {
        let temp1_val = self.get_bits_from_buffer(IDX_R_TEMP1, &self.read_regs); // 4-bit value
        let temp2_val = self.get_bits_from_buffer(IDX_R_TEMP2, &self.read_regs); // Full 8 bits
        self.concat_to_i12(temp1_val, temp2_val, false)
    }

    // --- Conversion to Physical Units ---
    pub fn get_x_mt(&self) -> f32 {
        self.get_x_raw() as f32 * B_MULT
    }
    pub fn get_y_mt(&self) -> f32 {
        self.get_y_raw() as f32 * B_MULT
    }
    pub fn get_z_mt(&self) -> f32 {
        self.get_z_raw() as f32 * B_MULT
    }
    pub fn get_temp_celsius(&self) -> f32 {
        (self.get_temp_raw() - TEMP_OFFSET_LSB) as f32 * TEMP_MULT
    }

    /// Reads and returns the 3-axis magnetic field strengths in milliTesla (mT).
    /// Calls `update_data()` internally to refresh sensor readings.
    pub async fn read_3_axis_strengths(&mut self) -> Result<(f32, f32, f32), TlvError<E>> {
        self.update_data().await?;
        Ok((self.get_x_mt(), self.get_y_mt(), self.get_z_mt()))
    }

    /// Calculates the magnitude of the magnetic field vector (in mT) from current buffered readings.
    pub fn get_magnitude_mt(&self) -> f32 {
        let x = self.get_x_mt();
        let y = self.get_y_mt();
        let z = self.get_z_mt();
        sqrtf(x * x + y * y + z * z)
    }

    /// Calculates the azimuth angle (longitude) in radians from current buffered readings.
    /// Range: -PI to PI. Angle in XY plane from X-axis towards Y-axis.
    pub fn get_azimuth_rad(&self) -> f32 {
        atan2f(self.get_y_mt(), self.get_x_mt())
    }

    /// Calculates the polar angle (latitude from XY plane) in radians from current buffered readings.
    /// Range: -PI/2 to PI/2. Angle from XY plane towards Z-axis.
    pub fn get_polar_rad(&self) -> f32 {
        let x_val = self.get_x_mt();
        let y_val = self.get_y_mt();
        let z_val = self.get_z_mt();
        let xy_plane_magnitude = sqrtf(x_val * x_val + y_val * y_val);

        if xy_plane_magnitude == 0.0 {
            // Avoid division by zero if X and Y components are zero
            if z_val > 0.0 {
                return core::f32::consts::FRAC_PI_2;
            } // Pointing straight up (+90 degrees)
            if z_val < 0.0 {
                return -core::f32::consts::FRAC_PI_2;
            } // Pointing straight down (-90 degrees)
            return 0.0; // If Z is also zero, magnitude is zero; angle is undefined (or 0 by convention)
        }
        atan2f(z_val, xy_plane_magnitude)
    }

    /// Reads and returns the magnetic field vector in spherical coordinates:
    /// (magnitude_mT, azimuth_radians, polar_radians).
    /// Calls `update_data()` internally.
    pub async fn read_spherical_coords(&mut self) -> Result<(f32, f32, f32), TlvError<E>> {
        self.update_data().await?;
        Ok((
            self.get_magnitude_mt(),
            self.get_azimuth_rad(),
            self.get_polar_rad(),
        ))
    }

    /// Reads and returns a compass heading (azimuth) in degrees, ignoring the Z component.
    /// Range: 0 to 360 degrees (0 degrees = positive X-axis, 90 = positive Y-axis).
    /// Calls `update_data()` internally.
    pub async fn compass_read_degrees(&mut self) -> Result<f32, TlvError<E>> {
        self.update_data().await?;
        let mut azimuth_in_degrees = self.get_azimuth_rad() * (180.0 / core::f32::consts::PI);
        if azimuth_in_degrees < 0.0 {
            azimuth_in_degrees += 360.0;
        } // Normalize to 0-360 range
        Ok(azimuth_in_degrees)
    }
}
