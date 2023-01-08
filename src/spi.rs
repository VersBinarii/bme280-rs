//! BME280 driver for sensors attached via SPI.

use embedded_hal::spi::blocking::{SpiBusRead, SpiBusWrite};
use embedded_hal::{delay::blocking::DelayUs, spi::blocking::SpiDevice};

use super::{
    BME280Common, Configuration, Error, IIRFilter, Interface, Measurements, Oversampling,
    BME280_H_CALIB_DATA_LEN, BME280_P_T_CALIB_DATA_LEN, BME280_P_T_H_DATA_LEN,
};

/// Representation of a BME280
#[derive(Debug, Default)]
pub struct BME280<DEVICE> {
    common: BME280Common<SPIInterface<DEVICE>>,
}

impl<DEVICE> BME280<DEVICE>
where
    DEVICE: SpiDevice,
    DEVICE::Bus: SpiBusWrite + SpiBusRead,
{
    /// Create a new BME280 struct
    pub fn new(device: DEVICE) -> Self {
        Self {
            common: BME280Common {
                interface: SPIInterface { device },
                calibration: None,
            },
        }
    }

    /// Initializes the BME280.
    /// This configures 2x temperature oversampling, 16x pressure oversampling, and the IIR filter
    /// coefficient 16.
    pub fn init<D: DelayUs>(&mut self, delay: &mut D) -> Result<(), Error<DEVICE::Error>> {
        self.common.init(
            delay,
            Configuration::default()
                .with_humidity_oversampling(Oversampling::Oversampling1X)
                .with_pressure_oversampling(Oversampling::Oversampling16X)
                .with_temperature_oversampling(Oversampling::Oversampling2X)
                .with_iir_filter(IIRFilter::Coefficient16),
        )
    }

    /// Initializes the BME280, applying the given configuration.
    pub fn init_with_config<D: DelayUs>(
        &mut self,
        delay: &mut D,
        config: Configuration,
    ) -> Result<(), Error<DEVICE::Error>> {
        self.common.init(delay, config)
    }

    /// Captures and processes sensor data for temperature, pressure, and humidity
    pub fn measure<D: DelayUs>(
        &mut self,
        delay: &mut D,
    ) -> Result<Measurements<DEVICE::Error>, Error<DEVICE::Error>> {
        self.common.measure(delay)
    }
}

/// Register access functions for SPI
#[derive(Debug, Default)]
struct SPIInterface<DEVICE> {
    /// concrete SPI device implementation
    device: DEVICE,
}

impl<DEVICE> Interface for SPIInterface<DEVICE>
where
    DEVICE: SpiDevice,
    DEVICE::Bus: SpiBusWrite + SpiBusRead,
{
    type Error = DEVICE::Error;

    fn read_register(&mut self, register: u8) -> Result<u8, Error<Self::Error>> {
        let mut result = [0u8];
        self.read_any_register(register, &mut result)?;
        Ok(result[0])
    }

    fn read_data(
        &mut self,
        register: u8,
    ) -> Result<[u8; BME280_P_T_H_DATA_LEN], Error<Self::Error>> {
        let mut data = [0; BME280_P_T_H_DATA_LEN];
        self.read_any_register(register, &mut data)?;
        Ok(data)
    }

    fn read_pt_calib_data(
        &mut self,
        register: u8,
    ) -> Result<[u8; BME280_P_T_CALIB_DATA_LEN], Error<Self::Error>> {
        let mut data = [0; BME280_P_T_CALIB_DATA_LEN];
        self.read_any_register(register, &mut data)?;
        Ok(data)
    }

    fn read_h_calib_data(
        &mut self,
        register: u8,
    ) -> Result<[u8; BME280_H_CALIB_DATA_LEN], Error<Self::Error>> {
        let mut data = [0; BME280_H_CALIB_DATA_LEN];
        self.read_any_register(register, &mut data)?;
        Ok(data)
    }

    fn write_register(&mut self, register: u8, payload: u8) -> Result<(), Error<Self::Error>> {
        // If the first bit is 0, the register is written.
        let data = [register & 0x7f, payload];
        self.device.write(&data).map_err(Error::Bus)
    }
}

impl<DEVICE> SPIInterface<DEVICE>
where
    DEVICE: SpiDevice,
    DEVICE::Bus: SpiBusWrite + SpiBusRead,
{
    fn read_any_register(
        &mut self,
        register: u8,
        data: &mut [u8],
    ) -> Result<(), Error<DEVICE::Error>> {
        self.device
            .transaction(|bus| {
                bus.write(&[register])?;
                bus.read(data)
            })
            .map_err(Error::Bus)?;
        Ok(())
    }
}
