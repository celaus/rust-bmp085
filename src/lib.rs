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
    Bmp085CalAc1 = 0xAA, // R   Calibration data (16 bits)
    Bmp085CalAc2 = 0xAC, // R   Calibration data (16 bits)
    Bmp085CalAc3 = 0xAE, // R   Calibration data (16 bits)
    Bmp085CalAc4 = 0xB0, // R   Calibration data (16 bits)
    Bmp085CalAc5 = 0xB2, // R   Calibration data (16 bits)
    Bmp085CalAc6 = 0xB4, // R   Calibration data (16 bits)
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
    return a.to_be();
}

// Helper method to convert an u16 x endian integer to a u16 big endian integer
fn u_to_be(r: u16) -> u16 {
    return r.to_be();
}

// ---------------------------------------------------------------------------------

fn get_raw_temp<E: Error>(dev: &mut I2CDevice<Error = E>,
                          coeff: &BMP085SensorCoefficients)
                          -> Result<(i32, i32), E> {
    try!(dev.smbus_write_byte_data(Register::Bmp085Control as u8,
                                   Command::Bmp085ReadTempCmd as u8));

    thread::sleep(Duration::from_millis(5)); // sleep for 4.5 ms

    let ut = i_to_be(try!(dev.smbus_read_word_data(Register::Bmp085Data as u8))) as i32;
    let ac6 = coeff.cal_ac6 as i32;
    let ac5 = coeff.cal_ac5 as i32;
    let md = coeff.cal_md;
    let mc = coeff.cal_mc;

    let x1: i32 = ((ut - ac6) * ac5) >> 15; // Note: X>>15 == X/(pow(2,15))
    let x2: i32 = (mc << 11) / (x1 + md); // Note: X<<11 == X<<(pow(2,11))
    let b5: i32 = x1 + x2;
    let t: i32 = (b5 + 8) >> 4;

    return Ok((t, b5));
}

fn get_raw_pressure<E: Error>(dev: &mut I2CDevice<Error = E>,
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
    let msb = try!(dev.smbus_read_byte_data(data_registers[0])) as u32;
    let lsb = try!(dev.smbus_read_byte_data(data_registers[1])) as u32;
    let xlsb = try!(dev.smbus_read_byte_data(data_registers[2])) as u32;

    let up: i32 = (((msb << 16) | (lsb << 8) | xlsb) >> (8 - sampling)) as i32;
    return Ok(up);
}

fn get_temp<E: Error>(dev: &mut I2CDevice<Error = E>,
                      coeff: &BMP085SensorCoefficients)
                      -> Result<f32, E> {
    let (t, _) = try!(get_raw_temp(dev, coeff));
    return Ok((t as f32) * 0.1);
}

fn get_pressure<E: Error>(dev: &mut I2CDevice<Error = E>,
                          coeff: &BMP085SensorCoefficients,
                          accuracy: &SamplingMode)
                          -> Result<u32, E> {
    let (_, b5) = try!(get_raw_temp(dev, coeff));
    let sampling = accuracy.clone() as u8;

    let up = try!(get_raw_pressure(dev, accuracy));

    let b1 = coeff.cal_b1;
    let b2 = coeff.cal_b2;
    let ac1 = coeff.cal_ac1;
    let ac2 = coeff.cal_ac2;
    let ac3 = coeff.cal_ac3;
    let ac4 = coeff.cal_ac4;


    let b6: i32 = b5 - 4000;
    let mut x1: i32 = (b2 * (b6.pow(2) >> 12)) >> 11;
    let mut x2: i32 = (ac2 * b6) >> 11;
    let x3: u32 = (x1 + x2) as u32;
    let b3: i32 = (((ac1 * 4 + (x3 as i32)) << sampling) + 2) / 4;

    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * (b6.pow(2) >> 12)) >> 16;
    let x3: i32 = (x1 + x2 + 2) >> 2;

    let b4: u32 = (ac4 * ((x3 as u32) + 32768)) >> 15;
    let b7: u32 = (up - b3) as u32 * (50000 >> sampling);

    let p = if b7 < 0x80000000 {
        (b7 << 1) / b4
    } else {
        (b7 / b4) << 1
    } as i32;
    x1 = (p >> 8).pow(2);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * (p)) >> 16;

    return Ok(((p) + ((x1 + x2 + 3791) >> 4)) as u32); // return as Pa
}


// -------------------------------------------------------------------------------

pub struct BMP085BarometerThermometer<T: I2CDevice + Sized> {
    pub dev: T,
    pub coeff: BMP085SensorCoefficients,
    pub accuracy: SamplingMode,
}

impl<T> BMP085BarometerThermometer<T>
    where T: I2CDevice + Sized
{
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

    fn pressure_kpa(&mut self) -> Result<f32, T::Error> {
        let reading = try!(get_pressure(&mut self.dev, &self.coeff, &self.accuracy)) as f32;
        Ok(reading / 1000.0)
    }
}

impl<T> Thermometer for BMP085BarometerThermometer<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;

    fn temperature_celsius(&mut self) -> Result<f32, T::Error> {
        let reading = try!(get_temp(&mut self.dev, &self.coeff));
        Ok(reading)
    }
}

// ---------------------------------------------------------------------------------

///
///
///
pub struct BMP085SensorCoefficients {
    cal_ac1: i32,
    cal_ac2: i32,
    cal_ac3: i32,
    cal_ac4: u32,
    cal_ac5: u32,
    cal_ac6: u32,
    cal_b1: i32,
    cal_b2: i32,
    cal_mb: i32, // unused?
    cal_mc: i32,
    cal_md: i32,
}

impl BMP085SensorCoefficients {
    ///
    ///
    ///
    pub fn new<E: Error>(dev: &mut I2CDevice<Error = E>) -> Result<BMP085SensorCoefficients, E> {
        let t_ac1 = try!(dev.smbus_read_word_data(Register::Bmp085CalAc1 as u8));
        let t_ac2 = try!(dev.smbus_read_word_data(Register::Bmp085CalAc2 as u8));
        let t_ac3 = try!(dev.smbus_read_word_data(Register::Bmp085CalAc3 as u8));
        let t_ac4 = try!(dev.smbus_read_word_data(Register::Bmp085CalAc4 as u8));
        let t_ac5 = try!(dev.smbus_read_word_data(Register::Bmp085CalAc5 as u8));
        let t_ac6 = try!(dev.smbus_read_word_data(Register::Bmp085CalAc6 as u8));
        let t_b1 = try!(dev.smbus_read_word_data(Register::Bmp085CalB1 as u8));
        let t_b2 = try!(dev.smbus_read_word_data(Register::Bmp085CalB2 as u8));
        let t_mb = try!(dev.smbus_read_word_data(Register::Bmp085CalMb as u8));
        let t_mc = try!(dev.smbus_read_word_data(Register::Bmp085CalMc as u8));
        let t_md = try!(dev.smbus_read_word_data(Register::Bmp085CalMd as u8));

        return Ok(BMP085SensorCoefficients {
            cal_ac1: i_to_be(t_ac1) as i32,
            cal_ac2: i_to_be(t_ac2) as i32,
            cal_ac3: i_to_be(t_ac3) as i32,
            cal_ac4: u_to_be(t_ac4) as u32,
            cal_ac5: u_to_be(t_ac5) as u32,
            cal_ac6: u_to_be(t_ac6) as u32,
            cal_b1: i_to_be(t_b1) as i32,
            cal_b2: i_to_be(t_b2) as i32,
            cal_mb: i_to_be(t_mb) as i32,
            cal_mc: i_to_be(t_mc) as i32,
            cal_md: i_to_be(t_md) as i32,
        });
    }
}



#[cfg(test)]
mod tests {

    #[test]
    fn it_works() {}
}
