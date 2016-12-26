// Copyright 2016 Claus Matzinger
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

extern crate i2cdev;

use std::thread;
use std::time::Duration;
use std::error::Error;
use i2cdev::core::*;
use i2cdev::sensors::{Thermometer, Barometer};


// -------------------------------------------------------------------------------

pub const BMP085_I2C_ADDR: u16 = 0x77; //BMP085 default address.

///
/// Pressure data sampling mode
///
#[derive(Clone, Debug)]
pub enum SamplingMode {
    UltraLowPower = 0,
    Standard = 1,
    HighRes = 2,
    UltraHighRes = 3,
}

// -------------------------------------------------------------------------------

// BMP085 register addresses
enum Register {
    Bmp085CalAC1 = 0xAA, // R   Calibration data (16 bits)
    Bmp085CalAC2 = 0xAC, // R   Calibration data (16 bits)
    Bmp085CalAC3 = 0xAE, // R   Calibration data (16 bits)
    Bmp085CalAC4 = 0xB0, // R   Calibration data (16 bits)
    Bmp085CalAC5 = 0xB2, // R   Calibration data (16 bits)
    Bmp085CalAC6 = 0xB4, // R   Calibration data (16 bits)
    Bmp085CalB1 = 0xB6, // R   Calibration data (16 bits)
    Bmp085CalB2 = 0xB8, // R   Calibration data (16 bits)
    Bmp085CalMb = 0xBA, // R   Calibration data (16 bits)
    Bmp085CalMc = 0xBC, // R   Calibration data (16 bits)
    Bmp085CalMd = 0xBE, // R   Calibration data (16 bits)
    Bmp085Control = 0xF4,
    Bmp085Data = 0xF6, // Pressure & Temp
}

// BMP085 command register addresses
enum Command {
    Bmp085ReadTempCmd = 0x2E,
    Bmp085ReadPressureCmd = 0x34,
}

// -------------------------------------------------------------------------------

// Helper method to convert an u16 x endian integer to a i16 big endian integer
fn i_to_be(r: u16) -> i16 {
    let a = r as i16;
    a.to_be()
}

// Helper method to convert an u16 x endian integer to a u16 big endian integer
fn u_to_be(r: u16) -> u16 {
    r.to_be()
}

// ---------------------------------------------------------------------------------

///
/// Reads the raw temperature data...
///
fn read_raw_temp<E: Error>(dev: &mut I2CDevice<Error = E>,
                           coeff: &BMP085SensorCoefficients)
                           -> Result<(i32, i32), E> {
    try!(dev.smbus_write_byte_data(Register::Bmp085Control as u8,
                                   Command::Bmp085ReadTempCmd as u8));

    thread::sleep(Duration::from_millis(5)); // sleep for 4.5 ms

    let ut: i32 = i_to_be(try!(dev.smbus_read_word_data(Register::Bmp085Data as u8))) as i32;
    let ac6: i32 = coeff.cal_ac6 as i32;
    let ac5: i32 = coeff.cal_ac5 as i32;
    let md: i32 = coeff.cal_md as i32;
    let mc: i32 = coeff.cal_mc as i32;


    let _ac5 = ac5 as i64;
    let x1: i32 = ((ut - ac6) as i64 * _ac5 >> 15) as i32; // Note: X>>15 == X/(pow(2,15))
    let x2: i32 = (mc << 11) / (x1 + md); // Note: X<<11 == X<<(pow(2,11))
    let b5: i32 = x1 + x2;
    let t: i32 = (b5 + 8) >> 4;
    Ok((t, b5))
}

///
/// Reads the raw pressure data...
///
fn read_raw_pressure<E: Error>(dev: &mut I2CDevice<Error = E>,
                               accuracy: &SamplingMode)
                               -> Result<i32, E> {
    let pressure_cmd = Command::Bmp085ReadPressureCmd as u8;
    let sampling = accuracy.clone() as u8;
    try!(dev.smbus_write_byte_data(Register::Bmp085Control as u8,
                                   pressure_cmd + (sampling << 6)));

    let duration = match *accuracy {
        SamplingMode::UltraLowPower => Duration::from_millis(5),
        SamplingMode::Standard => Duration::from_millis(8),
        SamplingMode::HighRes => Duration::from_millis(14),
        SamplingMode::UltraHighRes => Duration::from_millis(26),
    };
    thread::sleep(duration);

    let data_registers = vec![Register::Bmp085Data as u8,
                              Register::Bmp085Data as u8 + 1,
                              Register::Bmp085Data as u8 + 2];

    // read bytes (u8) and cast directly to u32 for bitshifts
    let msb = try!(dev.smbus_read_byte_data(data_registers[0])) as u32;
    let lsb = try!(dev.smbus_read_byte_data(data_registers[1])) as u32;
    let xlsb = try!(dev.smbus_read_byte_data(data_registers[2])) as u32;

    let up: i32 = ((msb << 16) + (lsb << 8) + xlsb >> (8 - sampling)) as i32;
    Ok(up)
}

///
/// Read temperature from the provided device.
///
fn read_temp<E: Error>(dev: &mut I2CDevice<Error = E>,
                       coeff: &BMP085SensorCoefficients)
                       -> Result<f32, E> {
    let (t, _) = try!(read_raw_temp(dev, coeff));
    Ok((t as f32) * 0.1)
}

///
/// Read pressure from the provided device.
///
fn read_pressure<E: Error>(dev: &mut I2CDevice<Error = E>,
                           coeff: &BMP085SensorCoefficients,
                           accuracy: &SamplingMode)
                           -> Result<u32, E> {
    let (_, b5) = try!(read_raw_temp(dev, coeff));
    let sampling = accuracy.clone() as u8;

    let up = try!(read_raw_pressure(dev, accuracy));

    let b1: i32 = coeff.cal_b1 as i32;
    let b2: i32 = coeff.cal_b2 as i32;
    let ac1: i32 = coeff.cal_ac1 as i32;
    let ac2: i32 = coeff.cal_ac2 as i32;
    let ac3: i32 = coeff.cal_ac3 as i32;
    let ac4: u32 = coeff.cal_ac4 as u32;


    let b6: i32 = b5 - 4000i32;

    let _t = (b6 as i32).pow(2) >> 12;
    let mut x1: i32 = (b2 * _t) >> 11;
    let mut x2: i32 = (ac2 * b6) >> 11;
    let x3: u32 = (x1 + x2) as u32;
    let b3: i32 = (((ac1 * 4 + (x3 as i32)) << sampling) + 2) / 4;
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * _t) >> 16;
    let x3: i32 = (x1 + x2 + 2) >> 2;

    let _x3: u32 = (x3 + 32768i32) as u32;
    let b4: u32 = (ac4 * _x3) >> 15;
    let b7: u32 = (up - b3) as u32 * (50000 >> sampling);
    let p = if b7 < 0x80000000 {
        (b7 << 1) / b4
    } else {
        (b7 / b4) << 1
    } as i32;

    x1 = (p >> 8).pow(2);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * (p)) >> 16;

    Ok(((p) + ((x1 + x2 + 3791) >> 4)) as u32) // return as Pa
}


// -------------------------------------------------------------------------------

///
/// The BMP085 barometer and thermometer
///
pub struct BMP085BarometerThermometer<T: I2CDevice + Sized> {
    coeff: BMP085SensorCoefficients,
    pub dev: T,
    pub accuracy: SamplingMode,
}

impl<T> BMP085BarometerThermometer<T>
    where T: I2CDevice + Sized
{
    ///
    /// Calibrates and creates a sensor representation.
    ///
    /// # Examples
    /// ```rust,ignore
    /// use i2cdev::linux::*;
    /// use bmp085::*;
    /// use i2cdev::sensors::{Barometer, Thermometer};
    /// let i2c_dev = LinuxI2CDevice::new("/dev/i2c-1", BMP085_I2C_ADDR).unwrap();
    /// let mut s = BMP085BarometerThermometer::new(i2c_dev,
    ///                     SamplingMode::Standard).unwrap();
    /// println!("Temperature: {:?} C", s.temperature_celsius().unwrap());
    /// println!("Pressure:    {:?} kPa", s.pressure_kpa().unwrap());
    /// ```
    pub fn new(mut dev: T,
               accuracy: SamplingMode)
               -> Result<BMP085BarometerThermometer<T>, T::Error> {
        let coeff = try!(BMP085SensorCoefficients::new(&mut dev));
        Ok(BMP085BarometerThermometer {
            dev: dev,
            coeff: coeff,
            accuracy: accuracy,
        })
    }
}

impl<T> Barometer for BMP085BarometerThermometer<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;

    ///
    /// Read pressure data in kPascal.
    ///
    fn pressure_kpa(&mut self) -> Result<f32, T::Error> {
        let reading = try!(read_pressure(&mut self.dev, &self.coeff, &self.accuracy)) as f32;
        Ok(reading / 1000f32)
    }
}

impl<T> Thermometer for BMP085BarometerThermometer<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;

    ///
    /// Read temperature data in degrees Celsius.
    ///
    fn temperature_celsius(&mut self) -> Result<f32, T::Error> {
        let reading = try!(read_temp(&mut self.dev, &self.coeff));
        Ok(reading)
    }
}

// ---------------------------------------------------------------------------------

///
/// Calibration coefficients, as in the specification
///
struct BMP085SensorCoefficients {
    cal_ac1: i16,
    cal_ac2: i16,
    cal_ac3: i16,
    cal_ac4: u16,
    cal_ac5: u16,
    cal_ac6: u16,
    cal_b1: i16,
    cal_b2: i16,
    cal_mb: i16, // unused?
    cal_mc: i16,
    cal_md: i16,
}

impl BMP085SensorCoefficients {
    ///
    /// Read calibration data from the provided device
    ///
    pub fn new<E: Error>(dev: &mut I2CDevice<Error = E>) -> Result<BMP085SensorCoefficients, E> {
        let t_ac1 = try!(dev.smbus_read_word_data(Register::Bmp085CalAC1 as u8));
        let t_ac2 = try!(dev.smbus_read_word_data(Register::Bmp085CalAC2 as u8));
        let t_ac3 = try!(dev.smbus_read_word_data(Register::Bmp085CalAC3 as u8));
        let t_ac4 = try!(dev.smbus_read_word_data(Register::Bmp085CalAC4 as u8));
        let t_ac5 = try!(dev.smbus_read_word_data(Register::Bmp085CalAC5 as u8));
        let t_ac6 = try!(dev.smbus_read_word_data(Register::Bmp085CalAC6 as u8));
        let t_b1 = try!(dev.smbus_read_word_data(Register::Bmp085CalB1 as u8));
        let t_b2 = try!(dev.smbus_read_word_data(Register::Bmp085CalB2 as u8));
        let t_mb = try!(dev.smbus_read_word_data(Register::Bmp085CalMb as u8));
        let t_mc = try!(dev.smbus_read_word_data(Register::Bmp085CalMc as u8));
        let t_md = try!(dev.smbus_read_word_data(Register::Bmp085CalMd as u8));

        Ok(BMP085SensorCoefficients {
            cal_ac1: i_to_be(t_ac1) as i16,
            cal_ac2: i_to_be(t_ac2) as i16,
            cal_ac3: i_to_be(t_ac3) as i16,
            cal_ac4: u_to_be(t_ac4) as u16,
            cal_ac5: u_to_be(t_ac5) as u16,
            cal_ac6: u_to_be(t_ac6) as u16,
            cal_b1: i_to_be(t_b1) as i16,
            cal_b2: i_to_be(t_b2) as i16,
            cal_mb: i_to_be(t_mb) as i16,
            cal_mc: i_to_be(t_mc) as i16,
            cal_md: i_to_be(t_md) as i16,
        })
    }
}


#[cfg(test)]
mod tests {

    extern crate byteorder;
    extern crate rand;

    use super::{Command, Register, SamplingMode, BMP085SensorCoefficients, BMP085BarometerThermometer};
    use i2cdev::sensors::{Thermometer, Barometer};
    use i2cdev::core::I2CDevice;
    use self::byteorder::{BigEndian, ByteOrder};
    use self::rand::Rng;
    use std::io;

    pub struct MockBMP085 {
        coeff: BMP085SensorCoefficients,
        reg: Register,
        offset: usize,
        t_data: u16,
        p_data: u16,
        last_cmd: Command,
    }


    impl I2CDevice for MockBMP085 {
        type Error = io::Error;

        fn read(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
            let mut buf = [0; 3]; // array of size 3 for simpler offsets
            let reading = match self.last_cmd {
                Command::Bmp085ReadTempCmd => self.t_data,
                Command::Bmp085ReadPressureCmd => self.p_data, // Pressure command :)
            };

            match self.reg {
                Register::Bmp085CalAC1 => BigEndian::write_i16(&mut buf, self.coeff.cal_ac1),
                Register::Bmp085CalAC2 => BigEndian::write_i16(&mut buf, self.coeff.cal_ac2),
                Register::Bmp085CalAC3 => BigEndian::write_i16(&mut buf, self.coeff.cal_ac3),
                Register::Bmp085CalAC4 => BigEndian::write_u16(&mut buf, self.coeff.cal_ac4),
                Register::Bmp085CalAC5 => BigEndian::write_u16(&mut buf, self.coeff.cal_ac5),
                Register::Bmp085CalAC6 => BigEndian::write_u16(&mut buf, self.coeff.cal_ac6),
                Register::Bmp085CalB1 => BigEndian::write_i16(&mut buf, self.coeff.cal_b1),
                Register::Bmp085CalB2 => BigEndian::write_i16(&mut buf, self.coeff.cal_b2),
                Register::Bmp085CalMb => BigEndian::write_i16(&mut buf, self.coeff.cal_mb),
                Register::Bmp085CalMc => BigEndian::write_i16(&mut buf, self.coeff.cal_mc),
                Register::Bmp085CalMd => BigEndian::write_i16(&mut buf, self.coeff.cal_md),
                Register::Bmp085Control => BigEndian::write_i16(&mut buf, 0),
                Register::Bmp085Data => BigEndian::write_u16(&mut buf, reading),
            };

            for (i, elem) in data.iter_mut().enumerate() {
                *elem = buf[i + self.offset]
            }
            Ok(())
        }

        fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
            let d = [0, data[0]];
            self.offset = 0;
            self.reg = match BigEndian::read_u16(&d) {
                0xAA => Register::Bmp085CalAC1,
                0xAC => Register::Bmp085CalAC2,
                0xAE => Register::Bmp085CalAC3,
                0xB0 => Register::Bmp085CalAC4,
                0xB2 => Register::Bmp085CalAC5,
                0xB4 => Register::Bmp085CalAC6,
                0xB6 => Register::Bmp085CalB1,
                0xB8 => Register::Bmp085CalB2,
                0xBA => Register::Bmp085CalMb,
                0xBC => Register::Bmp085CalMc,
                0xBE => Register::Bmp085CalMd,
                0xF4 => {
                    self.last_cmd = if data[1] == (Command::Bmp085ReadTempCmd as u8) {
                        Command::Bmp085ReadTempCmd
                    } else {
                        // don't do exact matching to prevent sampling mode influences
                        Command::Bmp085ReadPressureCmd
                    };
                    Register::Bmp085Control
                }
                0xF6 => Register::Bmp085Data,
                0xF7 => {
                    self.offset = 1;
                    Register::Bmp085Data
                }
                0xF8 => {
                    self.offset = 2;
                    Register::Bmp085Data
                }
                _ => unimplemented!(),
            };
            Ok(())
        }

        fn smbus_write_quick(&mut self, _bit: bool) -> Result<(), Self::Error> {
            unimplemented!()
        }

        fn smbus_read_block_data(&mut self, _register: u8) -> Result<Vec<u8>, Self::Error> {
            unimplemented!()
        }

        fn smbus_write_block_data(&mut self,
                                  _register: u8,
                                  _values: &[u8])
                                  -> Result<(), Self::Error> {
            unimplemented!()
        }

        fn smbus_process_block(&mut self,
                               _register: u8,
                               _values: &[u8])
                               -> Result<(), Self::Error> {
            unimplemented!()
        }

        fn smbus_read_i2c_block_data(&mut self,
                                     _register: u8,
                                     _len: u8)
                                     -> Result<Vec<u8>, Self::Error> {
            unimplemented!()
        }
    }

    fn new_i2c_mock(temperature: u16, pressure: u16) -> MockBMP085 {
        let coeff = BMP085SensorCoefficients {
            cal_ac1: 408i16,
            cal_ac2: -72i16,
            cal_ac3: -14383i16,
            cal_ac4: 32741u16,
            cal_ac5: 32757u16,
            cal_ac6: 23153u16,
            cal_b1: 6190i16,
            cal_b2: 4i16,
            cal_mb: -32768i16,
            cal_mc: -8711i16,
            cal_md: 2868i16,
        };
        MockBMP085 {
            coeff: coeff,
            reg: Register::Bmp085CalAC1,
            offset: 0,
            t_data: temperature,
            p_data: pressure,
            last_cmd: Command::Bmp085ReadTempCmd,
        }
    }

    fn make_dev(i2cdev: MockBMP085) -> BMP085BarometerThermometer<MockBMP085> {
        BMP085BarometerThermometer::new(i2cdev, SamplingMode::UltraLowPower).unwrap()
    }

    #[test]
    fn test_basic_pressure_read() {
        let i2cdev = new_i2c_mock(27898, 23843);
        let mut dev = make_dev(i2cdev);
        assert_eq!(dev.pressure_kpa().unwrap(), 69.964);
    }

    #[test]
    #[should_panic(expected = "attempt to multiply with overflow")]
    fn test_zero_pressure_read() {
        let i2cdev = new_i2c_mock(0, 0);
        let mut dev = make_dev(i2cdev);
        assert_eq!(dev.pressure_kpa().unwrap(), 1.234);
    }

    #[test]
    fn test_basic_temp_read() {
        let i2cdev = new_i2c_mock(27898, 0);
        let mut dev = make_dev(i2cdev);
        assert_eq!(dev.temperature_celsius().unwrap(), 15.0);
    }

    #[test]
    fn test_zero_temp_read() {
        let i2cdev = new_i2c_mock(0, 0);
        let mut dev = make_dev(i2cdev);
        assert_eq!(dev.temperature_celsius().unwrap(), -139.2);
    }

    #[test]
    fn test_max_temp_read() {
        let i2cdev = new_i2c_mock(u16::max_value(), 0);
        let mut dev = make_dev(i2cdev);
        assert_eq!(dev.temperature_celsius().unwrap(), -139.2);
    }

    #[test]
    fn test_rand_temp_read() {
        let n = 2_000;
        let mut rng = rand::thread_rng();
        for i in 0..n {
            let i2cdev = new_i2c_mock(rng.gen::<u16>(), 0);
            let mut dev = make_dev(i2cdev);
            let _ = dev.temperature_celsius().unwrap();
        }

    }

}
