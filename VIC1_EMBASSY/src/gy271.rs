use core::cell::RefCell;
use defmt::*;
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer};

// Ensure these are accessible in your project
// use defmt_rtt as _;
// use panic_probe as _;

// HMC5883L Default I2C Address
const HMC5883L_ADDRESS: u8 = 0x1E; // Or 0x0D for QMC5883L, make this configurable if needed

// HMC5883L Registers
const REG_CONFIG_A: u8 = 0x00;
const REG_CONFIG_B: u8 = 0x01;
const REG_MODE: u8 = 0x02;
const REG_DATA_OUT_X_MSB: u8 = 0x03; // Data registers are X_MSB, X_LSB, Z_MSB, Z_LSB, Y_MSB, Y_LSB
                                     // const REG_DATA_OUT_X_LSB: u8 = 0x04;
                                     // const REG_DATA_OUT_Z_MSB: u8 = 0x05;
                                     // const REG_DATA_OUT_Z_LSB: u8 = 0x06;
                                     // const REG_DATA_OUT_Y_MSB: u8 = 0x07;
                                     // const REG_DATA_OUT_Y_LSB: u8 = 0x08;
const REG_STATUS: u8 = 0x09;
const REG_ID_A: u8 = 0x0A; // Expected 'H'
const REG_ID_B: u8 = 0x0B; // Expected '4'
const REG_ID_C: u8 = 0x0C; // Expected '3'

// Configuration values
const DEFAULT_CONFIG_A: u8 = 0b0111_0000; // 8 samples averaged, 15 Hz output rate, normal measurement
const DEFAULT_CONFIG_B: u8 = 0b0010_0000; // Gain: +/- 1.3 Ga (implies sensitivity_lsb_gauss = 1090)
const MODE_CONTINUOUS: u8 = 0x00;
// const MODE_SINGLE: u8 = 0x01;
// const MODE_IDLE: u8 = 0x02;

// Sensitivity based on DEFAULT_CONFIG_B (Gain +/- 1.3 Ga -> 1090 LSB/Gauss)
// Other options:
// +/- 0.88 Ga -> 1370 LSB/Gauss (0x00)
// +/- 1.9 Ga -> 820 LSB/Gauss  (0x40)
// +/- 2.5 Ga -> 660 LSB/Gauss  (0x60)
// +/- 4.0 Ga -> 440 LSB/Gauss  (0x80)
// +/- 4.7 Ga -> 390 LSB/Gauss  (0xA0)
// +/- 5.6 Ga -> 330 LSB/Gauss  (0xC0)
// +/- 8.1 Ga -> 230 LSB/Gauss  (0xE0)
const SENSITIVITY_LSB_PER_GAUSS: f32 = 1090.0;

// Delays
const DELAY_POWER_ON_MS: u64 = 10; // Sensor needs some time after power on
const DELAY_MEASUREMENT_MS: u64 = 70; // Corresponds to ~15Hz (1/15s ~= 67ms)
                                      // For single shot mode, datasheet says ~6ms for 1 sample.

#[derive(Debug)]
pub enum Gy271Error {
    I2c(Error),
    NotInitialized,
    IdentificationError,
    DataNotReady,
    // ReadError, // Could be more specific if needed
}

impl From<Error> for Gy271Error {
    fn from(e: Error) -> Self {
        Gy271Error::I2c(e)
    }
}

pub struct GY271<'a> {
    i2c: &'a RefCell<I2c<'static, Async>>,
    address: u8,
    initialized: bool,
    sensitivity: f32, // LSB per Gauss, determined by gain setting
}

impl<'a> GY271<'a> {
    /// Creates a new GY-271 (HMC5883L) driver.
    /// It's recommended to call `init()` after this.
    ///
    /// # Arguments
    /// * `i2c` - A reference to the I2C peripheral.
    /// * `address` - The I2C address of the sensor (typically 0x1E for HMC5883L).
    pub fn new(i2c: &'a RefCell<I2c<'static, Async>>, address: u8) -> Self {
        GY271 {
            i2c,
            address,
            initialized: false,
            sensitivity: SENSITIVITY_LSB_PER_GAUSS, // Default, can be made configurable
        }
    }

    /// Initializes the GY-271 sensor.
    /// This checks the device ID and configures it for continuous measurement.
    pub async fn init(&mut self) -> Result<(), Gy271Error> {
        Timer::after(Duration::from_millis(DELAY_POWER_ON_MS)).await;

        // 1. Check Identification Registers
        let mut id_bytes = [0u8; 3];
        self.i2c
            .borrow_mut()
            .write_read(self.address, &[REG_ID_A], &mut id_bytes) // Write register address, then read 3 bytes
            .await
            .map_err(|e| {
                error!("GY-271: I2C Read ID Error: {:?}", e);
                Gy271Error::I2c(e)
            })?;

        if id_bytes[0] != b'H' || id_bytes[1] != b'4' || id_bytes[2] != b'3' {
            error!(
                "GY-271: Identification failed. Expected H, 4, 3 but got {=u8:X}, {=u8:X}, {=u8:X}",
                id_bytes[0], id_bytes[1], id_bytes[2]
            );
            return Err(Gy271Error::IdentificationError);
        }
        info!("GY-271: Identification successful (H43).");

        // 2. Configure Sensor
        // Config Register A: Samples averaged, data output rate, measurement mode
        self.i2c
            .borrow_mut()
            .write(self.address, &[REG_CONFIG_A, DEFAULT_CONFIG_A])
            .await
            .map_err(|e| {
                error!("GY-271: I2C Write Config A Error: {:?}", e);
                Gy271Error::I2c(e)
            })?;

        // Config Register B: Gain (this also sets our sensitivity)
        self.i2c
            .borrow_mut()
            .write(self.address, &[REG_CONFIG_B, DEFAULT_CONFIG_B])
            .await
            .map_err(|e| {
                error!("GY-271: I2C Write Config B Error: {:?}", e);
                Gy271Error::I2c(e)
            })?;
        // Here you could parse DEFAULT_CONFIG_B to set self.sensitivity if it's dynamic
        // For this example, it's fixed based on the constant.

        // Mode Register: Set to continuous measurement mode
        self.i2c
            .borrow_mut()
            .write(self.address, &[REG_MODE, MODE_CONTINUOUS])
            .await
            .map_err(|e| {
                error!("GY-271: I2C Write Mode Register Error: {:?}", e);
                Gy271Error::I2c(e)
            })?;

        // Small delay to allow the first measurement to complete
        Timer::after(Duration::from_millis(DELAY_MEASUREMENT_MS)).await;

        info!("GY-271: Initialized for continuous measurement.");
        self.initialized = true;
        Ok(())
    }

    /// Reads the magnetic field strength along X, Y, and Z axes.
    /// Returns a tuple (x, y, z) in Gauss.
    /// Ensures the sensor is initialized before reading.
    pub async fn read_magnetic_field(&mut self) -> Result<(f32, f32, f32), Gy271Error> {
        if !self.initialized {
            warn!("GY-271: Sensor not initialized. Attempting to initialize now.");
            self.init().await.map_err(|e| {
                error!("GY-271: Failed to auto-initialize sensor");
                e
            })?;
        }

        // In continuous mode, we can just read.
        // Optionally, check status register for data ready bit (DRDY, bit 0 of REG_STATUS)
        // For simplicity, we'll assume data is ready after the initial delay in init()
        // and subsequent reads will get new data due to continuous mode.
        // A more robust implementation might poll REG_STATUS.

        // Read the 6 data bytes (X_MSB, X_LSB, Z_MSB, Z_LSB, Y_MSB, Y_LSB)
        let mut data = [0u8; 6];
        // Set the register pointer to the first data output register (0x03)
        // Some I2C peripherals might require separate write (for register pointer) then read.
        // embassy-stm32's write_read often handles this if the device supports auto-increment.
        // HMC5883L auto-increments the register pointer after a read.
        self.i2c
            .borrow_mut()
            .write_read(self.address, &[REG_DATA_OUT_X_MSB], &mut data)
            .await
            .map_err(|e| {
                error!("GY-271: I2C Read Data Error: {:?}", e);
                Gy271Error::I2c(e)
            })?;

        // Convert 2's complement 16-bit data
        let raw_x = (data[0] as i16) << 8 | data[1] as i16;
        let raw_z = (data[2] as i16) << 8 | data[3] as i16; // Note Z is before Y
        let raw_y = (data[4] as i16) << 8 | data[5] as i16;

        // Convert raw values to Gauss
        let x_gauss = raw_x as f32 / self.sensitivity;
        let y_gauss = raw_y as f32 / self.sensitivity;
        let z_gauss = raw_z as f32 / self.sensitivity;

        Ok((x_gauss, y_gauss, z_gauss))
    }

    /// Reads the status register of the HMC5883L.
    /// Bit 0: RDY (Data Ready)
    /// Bit 1: LOCK (Data Output Register Lock)
    pub async fn read_status(&mut self) -> Result<u8, Gy271Error> {
        let mut status_byte = [0u8; 1];
        self.i2c
            .borrow_mut()
            .write_read(self.address, &[REG_STATUS], &mut status_byte)
            .await?;
        Ok(status_byte[0])
    }
}

/*
// Example Usage (conceptual, within an embassy task)
// Assuming `i2c_bus` is a `RefCell<I2c<'static, Async>>` properly configured.

async fn sensor_task(i2c_bus: &'static RefCell<I2c<'static, Async>>) {
    let mut magnetometer = GY271::new(i2c_bus, HMC5883L_ADDRESS);

    match magnetometer.init().await {
        Ok(()) => {
            info!("GY-271 initialized successfully!");
            loop {
                match magnetometer.read_magnetic_field().await {
                    Ok((x, y, z)) => {
                        info!("Mag Field: X={:.2} G, Y={:.2} G, Z={:.2} G", x, y, z);

                        // Example: Calculate heading (simple, assumes sensor is flat)
                        // let heading_rad = libm::atan2(y as f64, x as f64);
                        // let mut heading_deg = heading_rad * (180.0 / core::f64::consts::PI);
                        // // Normalize to 0-360
                        // if heading_deg < 0.0 {
                        //     heading_deg += 360.0;
                        // }
                        // info!("Approx. Heading: {:.1}°", heading_deg);

                    }
                    Err(e) => {
                        error!("Failed to read GY-271: {:?}", e);
                    }
                }
                Timer::after(Duration::from_secs(1)).await;
            }
        }
        Err(e) => {
            error!("Failed to initialize GY-271: {:?}", e);
        }
    }
}
*/
