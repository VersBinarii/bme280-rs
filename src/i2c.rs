//! BME280 driver for sensors attached via I2C.

use embedded_hal::delay::blocking::DelayUs;
use embedded_hal::i2c::{blocking::I2c, ErrorType};

use super::{
    BME280Common, Error, Interface, Measurements, BME280_H_CALIB_DATA_LEN,
    BME280_P_T_CALIB_DATA_LEN, BME280_P_T_H_DATA_LEN,
};

const BME280_I2C_ADDR_PRIMARY: u8 = 0x76;
const BME280_I2C_ADDR_SECONDARY: u8 = 0x77;

/// Representation of a BME280
#[derive(Debug, Default)]
pub struct BME280<I2C, D> {
    common: BME280Common<I2CInterface<I2C>, D>,
}

impl<I2C, D> BME280<I2C, D>
where
    I2C: I2c + ErrorType,
    D: DelayUs,
{
    /// Create a new BME280 struct using the primary I²C address `0x76`
    pub fn new_primary(i2c: I2C, delay: D) -> Self {
        Self::new(i2c, BME280_I2C_ADDR_PRIMARY, delay)
    }

    /// Create a new BME280 struct using the secondary I²C address `0x77`
    pub fn new_secondary(i2c: I2C, delay: D) -> Self {
        Self::new(i2c, BME280_I2C_ADDR_SECONDARY, delay)
    }

    /// Create a new BME280 struct using a custom I²C address
    pub fn new(i2c: I2C, address: u8, delay: D) -> Self {
        BME280 {
            common: BME280Common {
                interface: I2CInterface { i2c, address },
                delay,
                calibration: None,
            },
        }
    }

    /// Initializes the BME280
    pub fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        self.common.init()
    }

    /// Captures and processes sensor data for temperature, pressure, and humidity
    pub fn measure(&mut self) -> Result<Measurements<I2C::Error>, Error<I2C::Error>> {
        self.common.measure()
    }
}

/// Register access functions for I2C
#[derive(Debug, Default)]
struct I2CInterface<I2C> {
    /// concrete I²C device implementation
    i2c: I2C,
    /// I²C device address
    address: u8,
}

impl<I2C> Interface for I2CInterface<I2C>
where
    I2C: I2c + ErrorType,
{
    type Error = I2C::Error;

    fn read_register(&mut self, register: u8) -> Result<u8, Error<I2C::Error>> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::Bus)?;
        Ok(data[0])
    }

    fn read_data(
        &mut self,
        register: u8,
    ) -> Result<[u8; BME280_P_T_H_DATA_LEN], Error<I2C::Error>> {
        let mut data: [u8; BME280_P_T_H_DATA_LEN] = [0; BME280_P_T_H_DATA_LEN];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::Bus)?;
        Ok(data)
    }

    fn read_pt_calib_data(
        &mut self,
        register: u8,
    ) -> Result<[u8; BME280_P_T_CALIB_DATA_LEN], Error<I2C::Error>> {
        let mut data: [u8; BME280_P_T_CALIB_DATA_LEN] = [0; BME280_P_T_CALIB_DATA_LEN];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::Bus)?;
        Ok(data)
    }

    fn read_h_calib_data(
        &mut self,
        register: u8,
    ) -> Result<[u8; BME280_H_CALIB_DATA_LEN], Error<I2C::Error>> {
        let mut data: [u8; BME280_H_CALIB_DATA_LEN] = [0; BME280_H_CALIB_DATA_LEN];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::Bus)?;
        Ok(data)
    }

    fn write_register(&mut self, register: u8, payload: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[register, payload])
            .map_err(Error::Bus)
    }
}
