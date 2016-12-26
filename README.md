# BMP085 Temperature & Barometer Sensor Driver for Rust

[![Build Status](https://travis-ci.org/celaus/rust-bmp085.svg?branch=dev)](https://travis-ci.org/celaus/rust-bmp085)
[![Crates.io](https://img.shields.io/crates/v/bmp085.svg)](https://crates.io/crates/bmp085)

# Usage

Add the bmp085 driver to your `Cargo.toml`. [`i2cdev`](https://github.com/rust-embedded/rust-i2cdev) is also required to use common interfaces:

```
[dependencies]
bmp085 = "0.1.1"
i2cdev = "*"
```

Afterwards you can use the sensor:

```rust
extern crate bmp085;
extern crate i2cdev;

use bmp085::*;
use i2cdev::linux::*;
use i2cdev::sensors::{Barometer, Thermometer};


use std::thread;
use std::time::Duration;

fn main() {

    let i2c_dev = LinuxI2CDevice::new("/dev/i2c-1", BMP085_I2C_ADDR).unwrap();

    let mut s = BMP085BarometerThermometer::new(i2c_dev, SamplingMode::Standard).unwrap();

    loop {
        println!("Temperature: {:?} C",
                 s.temperature_celsius().unwrap());
        println!("Pressure:    {:?} kPa", s.pressure_kpa().unwrap());
        thread::sleep(Duration::from_millis(1000));
    }
}
```

Use `cargo build` to build the program, run with `sudo target/debug/myprog`.

*For device access, root access is commonly required.*

## Tests

Run `cargo tests` on any compatible device/OS and you should see the following
output:
```
running 6 tests
test tests::test_basic_temp_read ... ok
test tests::test_max_temp_read ... ok
test tests::test_basic_pressure_read ... ok
test tests::test_zero_temp_read ... ok
test tests::test_zero_pressure_read ... ok
test tests::test_rand_temp_read ... ok

test result: ok. 6 passed; 0 failed; 0 ignored; 0 measured

   Doc-tests bmp085

running 1 test
test BMP085BarometerThermometer<T>::new_0 ... ignored

test result: ok. 0 passed; 0 failed; 1 ignored; 0 measured
```

# Resources

[C++ implementation](http://svn.code.sf.net/p/bosch-ros-pkg/code/trunk/stacks/bosch_drivers/bmp085_driver/)

[Data Sheet](https://cdn-shop.adafruit.com/datasheets/BMP085_DataSheet_Rev.1.0_01July2008.pdf)

# License

Licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0), (c) 2016 Claus Matzinger
