use core::cell::RefCell;
use defmt::*;
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer};

// Default I2C address when AP_AD0 pin is low.
pub const ICM42688P_ADDR_AD0_LOW: u8 = 0x68;
// Default I2C address when AP_AD0 pin is high.
// const ICM42688P_ADDR_AD0_HIGH: u8 = 0x69;

// Register Map (Bank 0 unless specified)
mod reg {
    pub const DEVICE_CONFIG: u8 = 0x11;
    pub const PWR_MGMT0: u8 = 0x4E;

    pub const TEMP_DATA1: u8 = 0x1D;
    pub const TEMP_DATA0: u8 = 0x1E;

    pub const ACCEL_DATA_X1: u8 = 0x1F;
    pub const ACCEL_DATA_X0: u8 = 0x20;
    pub const ACCEL_DATA_Y1: u8 = 0x21;
    pub const ACCEL_DATA_Y0: u8 = 0x22;
    pub const ACCEL_DATA_Z1: u8 = 0x23;
    pub const ACCEL_DATA_Z0: u8 = 0x24;

    pub const GYRO_DATA_X1: u8 = 0x25;
    pub const GYRO_DATA_X0: u8 = 0x26;
    pub const GYRO_DATA_Y1: u8 = 0x27;
    pub const GYRO_DATA_Y0: u8 = 0x28;
    pub const GYRO_DATA_Z1: u8 = 0x29;
    pub const GYRO_DATA_Z0: u8 = 0x2A;

    pub const WHO_AM_I: u8 = 0x75;
    // pub const REG_BANK_SEL: u8 = 0x76; // For future use
}

const WHO_AM_I_EXPECTED: u8 = 0x47;

// Default sensitivities
const DEFAULT_GYRO_SENSITIVITY: f32 = 16.4; // LSB per °/s for ±2000 dps
const DEFAULT_ACCEL_SENSITIVITY: f32 = 2048.0; // LSB per g for ±16g
const TEMP_SENSITIVITY: f32 = 132.48;
const TEMP_OFFSET: f32 = 25.0;

// Delays
const DELAY_SOFT_RESET_MS: u64 = 2; // Datasheet says 1ms, use a bit more.
const DELAY_GYRO_STARTUP_MS: u64 = 50; // Datasheet says 45ms, use a bit more.
const DELAY_MODE_CHANGE_US: u64 = 250; // Datasheet says 200us, use a bit more.

#[derive(Debug, Clone, Copy)]
pub struct AccelData {
    pub x: f32, // in g
    pub y: f32, // in g
    pub z: f32, // in g
}

#[derive(Debug, Clone, Copy)]
pub struct GyroData {
    pub x: f32, // in °/s
    pub y: f32, // in °/s
    pub z: f32, // in °/s
}

#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    pub accel: AccelData,
    pub gyro: GyroData,
    pub temp_c: f32,
}

#[derive(Debug)]
pub enum Icm42688pError {
    I2c(Error),
    WhoAmIFailed,
    NotInitialized,
    // MeasurementBusy, // Might not be directly applicable like AHT20
}

impl From<Error> for Icm42688pError {
    fn from(e: Error) -> Self {
        Icm42688pError::I2c(e)
    }
}

pub struct Icm42688p<'a> {
    i2c: &'a RefCell<I2c<'static, Async>>,
    address: u8,
    initialized: bool,
    // For future use if FSR is configurable
    // gyro_sensitivity: f32,
    // accel_sensitivity: f32,
}

impl<'a> Icm42688p<'a> {
    pub fn new(i2c: &'a RefCell<I2c<'static, Async>>, address: u8) -> Self {
        Self {
            i2c,
            address,
            initialized: false,
            // gyro_sensitivity: DEFAULT_GYRO_SENSITIVITY,
            // accel_sensitivity: DEFAULT_ACCEL_SENSITIVITY,
        }
    }

    pub fn new_default_address(i2c: &'a RefCell<I2c<'static, Async>>) -> Self {
        Self::new(i2c, ICM42688P_ADDR_AD0_LOW)
    }

    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Icm42688pError> {
        self.i2c
            .borrow_mut()
            .write(self.address, &[reg, value])
            .await
            .map_err(|e| {
                error!("ICM42688P: I2C Write to reg {=u8:#X} error: {:?}", reg, e);
                Icm42688pError::I2c(e)
            })
    }

    async fn read_register(&mut self, reg: u8) -> Result<u8, Icm42688pError> {
        let mut data = [0u8; 1];
        self.i2c
            .borrow_mut()
            .write_read(self.address, &[reg], &mut data)
            .await
            .map_err(|e| {
                error!("ICM42688P: I2C Read from reg {=u8:#X} error: {:?}", reg, e);
                Icm42688pError::I2c(e)
            })?;
        Ok(data[0])
    }

    async fn read_data_block(
        &mut self,
        start_reg: u8,
        buffer: &mut [u8],
    ) -> Result<(), Icm42688pError> {
        self.i2c
            .borrow_mut()
            .write_read(self.address, &[start_reg], buffer)
            .await
            .map_err(|e| {
                error!(
                    "ICM42688P: I2C Read block from reg {=u8:#X} error: {:?}",
                    start_reg, e
                );
                Icm42688pError::I2c(e)
            })
    }

    pub async fn init(&mut self) -> Result<(), Icm42688pError> {
        // Select Bank 0 (should be default, but good practice)
        // self.write_register(reg::REG_BANK_SEL, 0x00).await?; // This might be tricky if REG_BANK_SEL behaves unusually

        // 1. Check WHO_AM_I
        let who_am_i = self.read_register(reg::WHO_AM_I).await?;
        if who_am_i != WHO_AM_I_EXPECTED {
            error!(
                "ICM42688P: WHO_AM_I check failed. Expected {=u8:#X}, got {=u8:#X}",
                WHO_AM_I_EXPECTED, who_am_i
            );
            return Err(Icm42688pError::WhoAmIFailed);
        }
        info!("ICM42688P: WHO_AM_I successful ({=u8:#X}).", who_am_i);

        // 2. Soft Reset
        // info!("ICM42688P: Performing soft reset...");
        // self.write_register(reg::DEVICE_CONFIG, 0x01).await?; // Bit 0 SOFT_RESET_CONFIG
        // Timer::after(Duration::from_millis(DELAY_SOFT_RESET_MS)).await;
        // The above is a soft reset of bank 0 registers. Full chip reset is tricky.
        // For robust init, let's ensure sensors are ON from whatever state.

        // 3. Configure Power Management: Enable Accel (LN), Gyro (LN), Temp Sensor
        // ACCEL_MODE = 0b11 (LN), GYRO_MODE = 0b11 (LN), TEMP_DIS = 0 (Enabled)
        // Other bits are 0. So, value = 0b0000_1111
        let pwr_mgmt0_val = 0b0000_1111;
        self.write_register(reg::PWR_MGMT0, pwr_mgmt0_val).await?;
        info!("ICM42688P: PWR_MGMT0 set to {=u8:#04X}", pwr_mgmt0_val);

        // 4. Wait for sensors to stabilize
        // Gyro takes 45ms from OFF. Mode changes take 200us.
        // If sensors were already in some ON state, 200us is enough.
        // If gyro was OFF, 45ms is needed. We assume it might have been OFF.
        Timer::after(Duration::from_millis(DELAY_GYRO_STARTUP_MS)).await;
        info!("ICM42688P: Sensor startup delay complete.");

        // TODO: Configure FSR and ODR if needed. For now, defaults are used.

        self.initialized = true;
        info!("ICM42688P: Initialized successfully.");
        Ok(())
    }

    fn raw_to_i16(msb: u8, lsb: u8) -> i16 {
        i16::from_be_bytes([msb, lsb])
    }

    pub async fn read_temperature_c(&mut self) -> Result<f32, Icm42688pError> {
        if !self.initialized {
            return Err(Icm42688pError::NotInitialized);
        }
        let mut data = [0u8; 2];
        self.read_data_block(reg::TEMP_DATA1, &mut data).await?;

        let temp_raw = Self::raw_to_i16(data[0], data[1]);
        // Formula: Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
        let temperature_c = (temp_raw as f32 / TEMP_SENSITIVITY) + TEMP_OFFSET;
        Ok(temperature_c)
    }

    pub async fn read_accel(&mut self) -> Result<AccelData, Icm42688pError> {
        if !self.initialized {
            return Err(Icm42688pError::NotInitialized);
        }
        let mut data = [0u8; 6];
        self.read_data_block(reg::ACCEL_DATA_X1, &mut data).await?;

        let x_raw = Self::raw_to_i16(data[0], data[1]);
        let y_raw = Self::raw_to_i16(data[2], data[3]);
        let z_raw = Self::raw_to_i16(data[4], data[5]);

        Ok(AccelData {
            x: x_raw as f32 / DEFAULT_ACCEL_SENSITIVITY,
            y: y_raw as f32 / DEFAULT_ACCEL_SENSITIVITY,
            z: z_raw as f32 / DEFAULT_ACCEL_SENSITIVITY,
        })
    }

    pub async fn read_gyro(&mut self) -> Result<GyroData, Icm42688pError> {
        if !self.initialized {
            return Err(Icm42688pError::NotInitialized);
        }
        let mut data = [0u8; 6];
        self.read_data_block(reg::GYRO_DATA_X1, &mut data).await?;

        let x_raw = Self::raw_to_i16(data[0], data[1]);
        let y_raw = Self::raw_to_i16(data[2], data[3]);
        let z_raw = Self::raw_to_i16(data[4], data[5]);

        Ok(GyroData {
            x: x_raw as f32 / DEFAULT_GYRO_SENSITIVITY,
            y: y_raw as f32 / DEFAULT_GYRO_SENSITIVITY,
            z: z_raw as f32 / DEFAULT_GYRO_SENSITIVITY,
        })
    }

    /// Reads all IMU data (accelerometer, gyroscope, temperature) in a single call.
    pub async fn read_all_data(&mut self) -> Result<ImuData, Icm42688pError> {
        if !self.initialized {
            return Err(Icm42688pError::NotInitialized);
        }

        // The sensor data registers are contiguous: TEMP_DATA1..GYRO_DATA_Z0
        // TEMP: 2 bytes (0x1D, 0x1E)
        // ACCEL: 6 bytes (0x1F..0x24)
        // GYRO: 6 bytes (0x25..0x2A)
        // Total = 14 bytes
        let mut data_block = [0u8; 14];
        self.read_data_block(reg::TEMP_DATA1, &mut data_block)
            .await?;

        let temp_raw = Self::raw_to_i16(data_block[0], data_block[1]); // TEMP_DATA1, TEMP_DATA0
        let temperature_c = (temp_raw as f32 / TEMP_SENSITIVITY) + TEMP_OFFSET;

        let accel_x_raw = Self::raw_to_i16(data_block[2], data_block[3]); // ACCEL_X1, ACCEL_X0
        let accel_y_raw = Self::raw_to_i16(data_block[4], data_block[5]); // ACCEL_Y1, ACCEL_Y0
        let accel_z_raw = Self::raw_to_i16(data_block[6], data_block[7]); // ACCEL_Z1, ACCEL_Z0

        let accel_data = AccelData {
            x: accel_x_raw as f32 / DEFAULT_ACCEL_SENSITIVITY,
            y: accel_y_raw as f32 / DEFAULT_ACCEL_SENSITIVITY,
            z: accel_z_raw as f32 / DEFAULT_ACCEL_SENSITIVITY,
        };

        let gyro_x_raw = Self::raw_to_i16(data_block[8], data_block[9]); // GYRO_X1, GYRO_X0
        let gyro_y_raw = Self::raw_to_i16(data_block[10], data_block[11]); // GYRO_Y1, GYRO_Y0
        let gyro_z_raw = Self::raw_to_i16(data_block[12], data_block[13]); // GYRO_Z1, GYRO_Z0

        let gyro_data = GyroData {
            x: gyro_x_raw as f32 / DEFAULT_GYRO_SENSITIVITY,
            y: gyro_y_raw as f32 / DEFAULT_GYRO_SENSITIVITY,
            z: gyro_z_raw as f32 / DEFAULT_GYRO_SENSITIVITY,
        };

        Ok(ImuData {
            accel: accel_data,
            gyro: gyro_data,
            temp_c: temperature_c,
        })
    }
}
