//! A platform agnostic driver to interface with the QMC5883L magnetometer,
//! designed for the Embassy asynchronous ecosystem.
//!
//! This driver uses `embassy-hal-common` traits for asynchronous I2C communication.

#![deny(missing_docs)]
#![no_std]

use core::cell::RefCell;
// Use the common I2C trait from embassy-hal-common
use embassy_stm32::{i2c::Error, i2c::I2c, mode::Async}; // Use Async explicitly

const I2C_ADDRESS: u8 = 0x0D;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
#[repr(u8)]
enum Register {
    DATA_OUT_X_L = 0,
    DATA_OUT_X_H = 1,
    DATA_OUT_Y_L = 2,
    DATA_OUT_Y_H = 3,
    DATA_OUT_Z_L = 4,
    DATA_OUT_Z_H = 5,
    STATUS = 6,
    TOUT_L = 7,
    TOUT_H = 8,
    CONTROL1 = 9,
    CONTROL2 = 10,
    PERIOD = 11,
    CHIP_ID = 13,
}

const STATUS_OVL: u8 = 0b010;
const STATUS_DRDY: u8 = 0b001;

const MODE_CONTINUOUS: u8 = 0b01;

const CTRL2_SOFT_RST: u8 = 1 << 7;
const CTRL2_ROL_PNT: u8 = 1 << 6;
const CTRL2_INT_ENB: u8 = 1 << 0;

/// Update frequency; 10Hz recommended for low power consumption.
#[repr(u8)]
pub enum OutputDataRate {
    /// 10Hz update rate.
    Rate10Hz = 0,
    /// 50Hz update rate.
    Rate50Hz = 0b0100,
    /// 100Hz update rate.
    Rate100Hz = 0b1000,
    /// 200Hz update rate.
    Rate200Hz = 0b1100,
}

/// Oversampling rate; controls bandwidth of internal digital filter.
/// Larger oversampling gets less in-band noise but higher power consumption.
#[repr(u8)]
pub enum OversampleRate {
    /// Oversample by 64.
    Rate64 = 3 << 6,
    /// Oversample by 128.
    Rate128 = 1 << 7,
    /// Oversample by 256.
    Rate256 = 1 << 6,
    /// Oversample by 512.
    Rate512 = 0,
}

/// Field range of magnetic sensor.
#[repr(u8)]
pub enum FieldRange {
    /// ± 2 gauss
    Range2Gauss = 0,
    /// ± 8 gauss
    Range8Gauss = 1 << 3,
}

/// QMC5883L Error
#[derive(Debug, Copy, Clone)]
pub enum GY271Error<E> {
    /// CHIP_ID returned invalid value (returned value is argument).
    InvalidDevice(u8),
    /// Read taken from magnetometer before ready.
    NotReady,
    /// Reading overflowed.
    Overflow,
    /// Underlying I2C bus error.
    BusError(E),
}

impl From<Error> for GY271Error<Error> {
    fn from(e: Error) -> Self {
        GY271Error::BusError(e)
    }
}

/// QMC5883L driver for Embassy async I2C.
///
/// The lifetime parameter `'a` ensures that the `RefCell` containing the I2C
/// peripheral outlives the driver instance.
pub struct GY271<'a> {
    i2c: &'a RefCell<I2c<'static, Async>>,
}

impl<'a> GY271<'a> {
    /// Creates a new GY271 device from a RefCell-wrapped I2C peripheral.
    /// It performs a soft reset and checks the device ID.
    pub async fn new(i2c: &'a RefCell<I2c<'static, Async>>) -> Result<Self, GY271Error<Error>> {
        let mut dev = GY271 { i2c };
        let id = dev.read_u8(Register::CHIP_ID).await?;
        if id != 0xff {
            return Err(GY271Error::InvalidDevice(id));
        }
        dev.reset().await?;
        Ok(dev)
    }

    /// Soft reset the device.
    pub async fn reset(&mut self) -> Result<(), GY271Error<Error>> {
        self.write_u8(Register::CONTROL2, CTRL2_SOFT_RST).await?;
        self.write_u8(Register::CONTROL2, CTRL2_ROL_PNT | CTRL2_INT_ENB)
            .await?;
        self.write_u8(Register::PERIOD, 1).await
    }

    /// Set the device field range.
    pub async fn set_field_range(&mut self, rng: FieldRange) -> Result<(), GY271Error<Error>> {
        let ctrl1 = self.read_u8(Register::CONTROL1).await?;
        let v = (ctrl1 & !(FieldRange::Range8Gauss as u8)) | (rng as u8);
        self.write_u8(Register::CONTROL1, v).await
    }

    /// Set the device oversampling rate.
    pub async fn set_oversample(&mut self, osr: OversampleRate) -> Result<(), GY271Error<Error>> {
        let ctrl1 = self.read_u8(Register::CONTROL1).await?;
        let v = (ctrl1 & !(OversampleRate::Rate64 as u8)) | (osr as u8);
        self.write_u8(Register::CONTROL1, v).await
    }

    /// Set the device output data rate.
    pub async fn set_output_data_rate(
        &mut self,
        odr: OutputDataRate,
    ) -> Result<(), GY271Error<Error>> {
        let ctrl1 = self.read_u8(Register::CONTROL1).await?;
        let v = (ctrl1 & !(OutputDataRate::Rate200Hz as u8)) | (odr as u8);
        self.write_u8(Register::CONTROL1, v).await
    }

    /// Put device in continuous mode.
    pub async fn continuous(&mut self) -> Result<(), GY271Error<Error>> {
        let ctrl1 = self.read_u8(Register::CONTROL1).await?;
        self.write_u8(Register::CONTROL1, ctrl1 | MODE_CONTINUOUS)
            .await
    }

    /// Put device in standby mode.
    pub async fn standby(&mut self) -> Result<(), GY271Error<Error>> {
        let ctrl1 = self.read_u8(Register::CONTROL1).await?;
        self.write_u8(Register::CONTROL1, ctrl1 & !MODE_CONTINUOUS)
            .await
    }

    /// Enable interrupt pin.
    pub async fn enable_interrupt(&mut self) -> Result<(), GY271Error<Error>> {
        self.write_u8(Register::CONTROL2, CTRL2_ROL_PNT).await
    }

    /// Disable interrupt pin.
    pub async fn disable_interrupt(&mut self) -> Result<(), GY271Error<Error>> {
        self.write_u8(Register::CONTROL2, CTRL2_ROL_PNT | CTRL2_INT_ENB)
            .await
    }

    /// Read temperature sensor; temperature coefficient is about 100 LSB/°C.
    pub async fn temp(&mut self) -> Result<i16, GY271Error<Error>> {
        let temp_l = self.read_u8(Register::TOUT_L).await? as i16;
        let temp_h = self.read_u8(Register::TOUT_H).await? as i16;
        Ok((temp_h << 8) | temp_l)
    }

    /// Read raw (x,y,z) from magnetometer.
    pub async fn mag(&mut self) -> Result<(i16, i16, i16), GY271Error<Error>> {
        let mut buf = [0u8; 7];
        self.i2c
            .borrow_mut()
            .write_read(I2C_ADDRESS, &[Register::STATUS as u8], &mut buf)
            .await?;

        let status = buf[0];
        if (status & STATUS_DRDY) == 0 {
            return Err(GY271Error::NotReady);
        } else if (status & STATUS_OVL) != 0 {
            return Err(GY271Error::Overflow);
        }
        let x = i16::from_le_bytes([buf[1], buf[2]]);
        let y = i16::from_le_bytes([buf[3], buf[4]]);
        let z = i16::from_le_bytes([buf[5], buf[6]]);
        Ok((x, y, z))
    }

    async fn read_u8(&mut self, reg: Register) -> Result<u8, GY271Error<Error>> {
        let mut buf = [0u8; 1];
        self.i2c
            .borrow_mut()
            .write_read(I2C_ADDRESS, &[reg as u8], &mut buf)
            .await?;
        Ok(buf[0])
    }

    async fn write_u8(&mut self, reg: Register, v: u8) -> Result<(), GY271Error<Error>> {
        match self
            .i2c
            .borrow_mut()
            .write(I2C_ADDRESS, &[reg as u8, v])
            .await
        {
            Ok(()) => Ok(()),
            Err(e) => Err(GY271Error::BusError(e)),
        }
    }
}
