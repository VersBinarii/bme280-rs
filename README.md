# bme280

A rust device driver for the Bosch BME280 temperature, humidity, and atmospheric pressure sensor and the Bosch BMP280 temperature and atmospheric pressure sensor.

## Usage

[View Complete Documentation Here](https://docs.rs/bme280)

```rust
use linux_embedded_hal as hal;

use linux_embedded_hal::{Delay, I2cdev};
use bme280::i2c::BME280;

// using Linux I2C Bus #1 in this example
let i2c_bus = I2cdev::new("/dev/i2c-1").unwrap();

// initialize the BME280 using the primary I2C address 0x76
let mut bme280 = BME280::new_primary(i2c_bus, Delay);

// or, initialize the BME280 using the secondary I2C address 0x77
// let mut bme280 = BME280::new_secondary(i2c_bus, Delay);

// or, initialize the BME280 using a custom I2C address
// let bme280_i2c_addr = 0x88;
// let mut bme280 = BME280::new(i2c_bus, bme280_i2c_addr, Delay);

// initialize the sensor
bme280.init().unwrap();

// measure temperature, pressure, and humidity
let measurements = bme280.measure().unwrap();

println!("Relative Humidity = {}%", measurements.humidity);
println!("Temperature = {} deg C", measurements.temperature);
println!("Pressure = {} pascals", measurements.pressure);
```

## Serde Support

To enable optional serde serialization support for the [measurements struct](https://docs.rs/bme280/0.1.2/bme280/struct.Measurements.html), simply enable the `serde` feature, like so in `Cargo.toml`:

```toml
[dependencies]
bme280 = { version = "0.2", features = ["serde"] }
```

## License

Licensed under either of:

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
