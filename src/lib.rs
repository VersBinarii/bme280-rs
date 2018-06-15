//! A platform agnostic Rust driver for the Bosch BME280, based on the
//! [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.
//!
//! ## The Device
//!
//! The Bosch BME280 is a highly accurate sensor for atmospheric temperature,
//! pressure, and relative humidity.
//!
//! The device has I²C and SPI interfaces (SPI is not currently supported).
//!
//! - [Details and datasheet](https://www.bosch-sensortec.com/bst/products/all_products/bme280)
//!
//! ## Usage
//!
//! ```no_run
//! extern crate linux_embedded_hal as hal;
//! extern crate bme280;
//!
//! use hal::{Delay, I2cdev};
//! use bme280::BME280;
//!
//! // using Linux I2C Bus #1 in this example
//! let i2c_bus = I2cdev::new("/dev/i2c-1").unwrap();
//!
//! // initialize the BME280 using the primary I2C address 0x77
//! let mut bme280 = BME280::new_primary(i2c_bus, Delay);
//!
//! // or, initialize the BME280 using the secondary I2C address 0x78
//! // let mut bme280 = BME280::new_secondary(i2c_bus, Delay);
//!
//! // or, initialize the BME280 using a custom I2C address
//! // let bme280_i2c_addr = 0x88;
//! // let mut bme280 = BME280::new(i2c_bus, bme280_i2c_addr, Delay);
//!
//! // initialize the sensor
//! bme280.init().unwrap();
//!
//! // measure temperature, pressure, and humidity
//! let measurements = bme280.measure().unwrap();
//!
//! println!("Relative Humidity = {}%", measurements.humidity);
//! println!("Temperature = {} deg C", measurements.temperature);
//! println!("Pressure = {} pascals", measurements.pressure);
//! ```
#![no_std]

extern crate embedded_hal;

use core::marker::PhantomData;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

const BME280_I2C_ADDR_PRIMARY: u8 = 0x76;
const BME280_I2C_ADDR_SECONDARY: u8 = 0x77;

const BME280_RESET_ADDR: u8 = 0xE0;
const BME280_SOFT_RESET_CMD: u8 = 0xB6;

const BME280_CHIP_ID: u8 = 0x60;
const BME280_CHIP_ID_ADDR: u8 = 0xD0;

const BME280_DATA_ADDR: u8 = 0xF7;

const BME280_P_T_H_DATA_LEN: usize = 8;

const BME280_TEMP_MIN: f32 = -40.0;
const BME280_TEMP_MAX: f32 = 85.0;

const BME280_PRESSURE_MIN: f32 = 30000.0;
const BME280_PRESSURE_MAX: f32 = 110000.0;

const BME280_HUMIDITY_MIN: f32 = 0.0;
const BME280_HUMIDITY_MAX: f32 = 100.0;

#[derive(Debug)]
pub enum Error<E> {
    /// Failed to compensate a raw measurement
    CompensationFailed,
    /// I2C bus error
    I2c(E),
    /// Failed to parse sensor data
    InvalidData,
    /// No calibration data is available (probably forgot to call or check BME280::init for failure)
    NoCalibrationData,
    /// Chip ID doesn't match expected value
    UnsupportedChip,
}

#[derive(Debug)]
struct CalibrationData {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
    t_fine: i32,
}

#[derive(Debug)]
pub struct Measurements<E> {
    pub temperature: f32,
    pub pressure: f32,
    pub humidity: f32,
    _e: PhantomData<E>,
}

impl<E> Measurements<E> {
    fn parse(
        data: [u8; BME280_P_T_H_DATA_LEN],
        calibration: &mut CalibrationData,
    ) -> Result<Self, Error<E>> {
        let data_msb: u32 = (data[0] as u32) << 12;
        let data_lsb: u32 = (data[1] as u32) << 4;
        let data_xlsb: u32 = (data[2] as u32) >> 4;
        let pressure = data_msb | data_lsb | data_xlsb;

        let data_msb: u32 = (data[3] as u32) << 12;
        let data_lsb: u32 = (data[4] as u32) << 4;
        let data_xlsb: u32 = (data[5] as u32) >> 4;
        let temperature = data_msb | data_lsb | data_xlsb;

        let data_msb: u32 = (data[6] as u32) << 8;
        let data_lsb: u32 = data[7] as u32;
        let humidity = data_msb | data_lsb;

        let temperature = Measurements::compensate_temperature(temperature, calibration)?;
        let pressure = Measurements::compensate_pressure(pressure, calibration)?;
        let humidity = Measurements::compensate_humidity(humidity, calibration)?;

        Ok(Measurements {
            temperature,
            pressure,
            humidity,
            _e: PhantomData,
        })
    }

    fn compensate_temperature(
        uncompensated: u32,
        calibration: &mut CalibrationData,
    ) -> Result<f32, Error<E>> {
        let var1: f32 = uncompensated as f32 / 16384.0 - calibration.dig_t1 as f32 / 1024.0;
        let var1 = var1 * calibration.dig_t2 as f32;

        let var2 = uncompensated as f32 / 131072.0 - calibration.dig_t1 as f32 / 8192.0;
        let var2 = var2 * var2 * calibration.dig_t3 as f32;

        calibration.t_fine = (var1 + var2) as i32;

        let temperature = (var1 + var2) / 5120.0;

        Ok(if temperature < BME280_TEMP_MIN {
            BME280_TEMP_MIN
        } else if temperature > BME280_TEMP_MAX {
            BME280_TEMP_MAX
        } else {
            temperature
        })
    }

    fn compensate_pressure(
        uncompensated: u32,
        calibration: &mut CalibrationData,
    ) -> Result<f32, Error<E>> {
        let var1: f32 = calibration.t_fine as f32 / 2.0 - 64000.0;
        let var2: f32 = var1 * var1 * calibration.dig_p6 as f32 / 32768.0;
        let var2: f32 = var2 + var1 * calibration.dig_p5 as f32 * 2.0;
        let var2: f32 = var2 / 4.0 + calibration.dig_p4 as f32 * 65536.0;
        let var3: f32 = calibration.dig_p3 as f32 * var1 * var1 / 524288.0;
        let var1: f32 = (var3 + calibration.dig_p2 as f32 * var1) / 524288.0;
        let var1: f32 = (1.0 + var1 / 32768.0) * calibration.dig_p1 as f32;

        let pressure = if var1 > 0.0 {
            let pressure: f32 = 1048576.0 - uncompensated as f32;
            let pressure: f32 = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
            let var1: f32 = calibration.dig_p9 as f32 * pressure * pressure / 2147483648.0;
            let var2: f32 = pressure * calibration.dig_p8 as f32 / 32768.0;
            let pressure: f32 = pressure + (var1 + var2 + calibration.dig_p7 as f32) / 16.0;
            if pressure < BME280_PRESSURE_MIN {
                BME280_PRESSURE_MIN
            } else if pressure > BME280_PRESSURE_MAX {
                BME280_PRESSURE_MAX
            } else {
                pressure
            }
        } else {
            return Err(Error::InvalidData);
        };
        Ok(pressure)
    }

    fn compensate_humidity(
        uncompensated: u32,
        calibration: &mut CalibrationData,
    ) -> Result<f32, Error<E>> {
        let var1: f32 = calibration.t_fine as f32 - 76800.0;
        let var2: f32 =
            calibration.dig_h4 as f32 * 64.0 + (calibration.dig_h5 as f32 / 16384.0) * var1;
        let var3: f32 = uncompensated as f32 - var2;
        let var4: f32 = calibration.dig_h2 as f32 / 65536.0;
        let var5: f32 = 1.0 + (calibration.dig_h3 as f32 / 67108864.0) * var1;
        let var6: f32 = 1.0 + (calibration.dig_h6 as f32 / 67108864.0) * var1 * var5;
        let var6: f32 = var3 * var4 * (var5 * var6);
        let humidity: f32 = var6 * (1.0 - calibration.dig_h1 as f32 * var6 / 524288.0);
        let humidity = if humidity < BME280_HUMIDITY_MIN {
            BME280_HUMIDITY_MIN
        } else if humidity > BME280_HUMIDITY_MAX {
            BME280_HUMIDITY_MAX
        } else {
            humidity
        };
        Ok(humidity)
    }
}

#[derive(Debug, Default)]
pub struct BME280<I2C, D> {
    /// concrete I²C device implementation
    i2c: I2C,
    /// I²C device address
    address: u8,
    /// concrete Delay implementation
    delay: D,
    /// calibration data
    calibration: Option<CalibrationData>,
}

impl<I2C, D, E> BME280<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u8>,
{
    pub fn new_primary(i2c: I2C, delay: D) -> Self {
        Self::new(i2c, BME280_I2C_ADDR_PRIMARY, delay)
    }

    pub fn new_secondary(i2c: I2C, delay: D) -> Self {
        Self::new(i2c, BME280_I2C_ADDR_SECONDARY, delay)
    }

    pub fn new(i2c: I2C, address: u8, delay: D) -> Self {
        BME280 {
            i2c,
            address,
            delay,
            calibration: None,
        }
    }

    pub fn init(&mut self) -> Result<(), Error<E>> {
        self.verify_chip_id()?;
        self.soft_reset()
    }

    fn verify_chip_id(&mut self) -> Result<(), Error<E>> {
        let chip_id = self.read_register(BME280_CHIP_ID_ADDR)?;
        if chip_id == BME280_CHIP_ID {
            Ok(())
        } else {
            Err(Error::UnsupportedChip)
        }
    }

    pub fn soft_reset(&mut self) -> Result<(), Error<E>> {
        self.write_register(BME280_RESET_ADDR, BME280_SOFT_RESET_CMD)?;
        self.delay.delay_ms(2); // startup time is 2ms
        Ok(())
    }

    pub fn measure(&mut self) -> Result<Measurements<E>, Error<E>> {
        let measurements = self.read_data(BME280_DATA_ADDR)?;
        match self.calibration.as_mut() {
            Some(calibration) => {
                let measurements = Measurements::parse(measurements, &mut *calibration)?;
                Ok(measurements)
            }
            None => Err(Error::NoCalibrationData),
        }
    }

    fn read_register(&mut self, register: u8) -> Result<u8, Error<E>> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::I2c)?;
        Ok(data[0])
    }

    fn read_data(&mut self, register: u8) -> Result<[u8; BME280_P_T_H_DATA_LEN], Error<E>> {
        let mut data: [u8; BME280_P_T_H_DATA_LEN] = [0; BME280_P_T_H_DATA_LEN];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .map_err(Error::I2c)?;
        Ok(data)
    }

    fn write_register(&mut self, register: u8, payload: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &[register, payload])
            .map_err(Error::I2c)
    }
}
