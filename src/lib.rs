extern crate i2cdev;

pub mod bmp085 {
    //! A module for retrieving data from the (Adafruit)
    //! BMP085 temperature/pressure sensor

    use std::thread;
    use std::time::Duration;
    use i2cdev::core::*;
    use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

    const BMP085I2CADDR: u16 = 0x77; //BMP085 default address.

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

    // Converts and u16 x endian integer to a i16 big endian integer
    fn i_to_be(r: u16) -> Result<i16, LinuxI2CError> {
        let a = r as i16;
        return Ok(a.to_be());
    }

    // Converts and u16 x endian integer to a u16 big endian integer
    fn u_to_be(r: u16) -> Result<u16, LinuxI2CError> {
        return Ok(r.to_be());
    }


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

    ///
    /// A representation of the BMP085 temperature/pressure sensor.
    ///
    pub struct BMP085Sensor {
        device: LinuxI2CDevice,
        cal_ac1: i16,
        cal_ac2: i16,
        cal_ac3: i16,
        cal_ac4: u16,
        cal_ac5: u16,
        cal_ac6: u16,
        cal_b1: i16,
        cal_b2: i16,
        cal_mb: i16,
        cal_mc: i16,
        cal_md: i16,
        pressure: i32,
    }

    impl BMP085Sensor {
        ///
        /// Create & calibrates a new sensor instance, with the device path and its address.
        ///
        /// # Example
        /// ```
        /// let mut s = BMP085Sensor::new("/dev/i2c-1", None);
        /// let mut s = BMP085Sensor::new("/dev/i2c-1", 0x77);
        /// ```
        pub fn new(path: &'static str, addr: Option<u16>) -> BMP085Sensor {
            let address = addr.unwrap_or(BMP085I2CADDR);

            let mut dev = LinuxI2CDevice::new(path, address).unwrap();

            let cal_ac1: i16 = dev.smbus_read_word_data(Register::Bmp085CalAc1 as u8)
                .and_then(i_to_be)
                .unwrap_or(408);
            let cal_ac2: i16 = dev.smbus_read_word_data(Register::Bmp085CalAc2 as u8)
                .and_then(i_to_be)
                .unwrap_or(-72);
            let cal_ac3: i16 = dev.smbus_read_word_data(Register::Bmp085CalAc3 as u8)
                .and_then(i_to_be)
                .unwrap_or(-14383);
            let cal_ac4: u16 = dev.smbus_read_word_data(Register::Bmp085CalAc4 as u8)
                .and_then(u_to_be)
                .unwrap_or(32741);
            let cal_ac5: u16 = dev.smbus_read_word_data(Register::Bmp085CalAc5 as u8)
                .and_then(u_to_be)
                .unwrap_or(32757);
            let cal_ac6: u16 = dev.smbus_read_word_data(Register::Bmp085CalAc6 as u8)
                .and_then(u_to_be)
                .unwrap_or(23153);
            let cal_b1: i16 = dev.smbus_read_word_data(Register::Bmp085CalB1 as u8)
                .and_then(i_to_be)
                .unwrap_or(6190);
            let cal_b2: i16 = dev.smbus_read_word_data(Register::Bmp085CalB2 as u8)
                .and_then(i_to_be)
                .unwrap_or(4);
            let cal_mb: i16 = dev.smbus_read_word_data(Register::Bmp085CalMb as u8)
                .and_then(i_to_be)
                .unwrap_or(-32767);
            let cal_mc: i16 = dev.smbus_read_word_data(Register::Bmp085CalMc as u8)
                .and_then(i_to_be)
                .unwrap_or(-8711);
            let cal_md: i16 = dev.smbus_read_word_data(Register::Bmp085CalMd as u8)
                .and_then(i_to_be)
                .unwrap_or(2868);

            BMP085Sensor {
                device: dev,
                cal_ac1: cal_ac1,
                cal_ac2: cal_ac2,
                cal_ac3: cal_ac3,
                cal_ac4: cal_ac4,
                cal_ac5: cal_ac5,
                cal_ac6: cal_ac6,
                cal_b1: cal_b1,
                cal_b2: cal_b2,
                cal_mb: cal_mb,
                cal_mc: cal_mc,
                cal_md: cal_md,
                pressure: 0,
            }
        }

        fn get_raw_temp(&mut self) -> Result<(i32, i32), LinuxI2CError> {
            try!(self.device.smbus_write_byte_data(Register::Bmp085Control as u8,
                                                   Command::Bmp085ReadTempCmd as u8));

            thread::sleep(Duration::from_millis(5)); // sleep for 4.5 ms

            let ut = try!(self.device
                .smbus_read_word_data(Register::Bmp085Data as u8)
                .and_then(i_to_be)) as i32;

            let ac6 = self.cal_ac6 as i32;
            let ac5 = self.cal_ac5 as i32;
            let md = self.cal_md as i32;
            let mc = self.cal_mc as i32;

            let x1: i32 = ((ut - ac6) * ac5) >> 15; // Note: X>>15 == X/(pow(2,15))
            let x2: i32 = (mc << 11) / (x1 + md); // Note: X<<11 == X<<(pow(2,11))
            let b5: i32 = x1 + x2;
            let t: i32 = (b5 + 8) >> 4;
            return Ok((t, b5));
        }

        ///
        /// Retrieves the temperature value of the sensor in ° Celsius
        ///
        pub fn get_temp(&mut self) -> Result<f32, LinuxI2CError> {
            let (t, b5) = try!(self.get_raw_temp());
            return Ok((t as f32) * 0.1);
        }

        fn get_raw_pressure(&mut self, accuracy: SamplingMode) -> Result<u32, LinuxI2CError> {
            let pressure_cmd = Command::Bmp085ReadPressureCmd as u8;
            let sampling = accuracy.clone() as u8;
            try!(self.device.smbus_write_byte_data(Register::Bmp085Control as u8,
                                                   pressure_cmd + (sampling << 6)));

            let duration = match accuracy {
                SamplingMode::UltraLowPower => Duration::from_millis(5),
                SamplingMode::Standard => Duration::from_millis(8),
                SamplingMode::HighRes => Duration::from_millis(14),
                SamplingMode::UltraHighRes => Duration::from_millis(26),
            };
            thread::sleep(duration);

            let raw_up: Vec<u32> = try!(self.device
                    .smbus_read_block_data(Register::Bmp085Data as u8))
                .into_iter()
                .map(|d| d as u32)
                .collect();

            let up = (raw_up[0] << 16) | (raw_up[1] << 8) | raw_up[2] >> (8 - sampling);
            return Ok(up);
        }


        pub fn get_pressure(&mut self, accuracy: SamplingMode) -> Result<f32, LinuxI2CError> {
            let (temp, b5) = try!(self.get_raw_temp());
            let sampling = accuracy.clone() as u8;

            let up = try!(self.get_raw_pressure(accuracy));
            println!("hello1");

            let b1 = self.cal_b1 as i32;
            let b2 = self.cal_b2 as i32;
            let ac1 = self.cal_ac1 as i32;
            let ac2 = self.cal_ac2 as i32;
            let ac3 = self.cal_ac3 as i32;
            let ac4 = self.cal_ac4 as u32;


            let b6: i32 = b5 - 4000; // magic!
            let mut x1: i32 = (b2 * (b6.pow(2) >> 12)) >> 11;
            let mut x2: i32 = (ac2 * b6) >> 11;
            let mut x3: i32 = x1 + x2;  // coefficients derived from existing coefficients.
            let b3: i32 = (((ac1 * 4 + x3) << sampling) + 2) >> 2;

            x1 = (ac3 * b6) >> 13;
            x2 = ((b1 * b6.pow(2)) >> 12) >> 16;
            x3 = (x1 + x2 + 2) >> 2;

            let b4: u32 = (ac4 * (x3 as u32) + 32768) >> 15;
            let b7: u32 = (up - b3 as u32) * (50000 >> sampling); // more magic
            println!("hello");
            self.pressure = if b7 < 0x80000000 { // moar magic !
                (b7 << 1) / b4
            } else {
                (b7 / b4) << 1
            } as i32;
            x1 = (self.pressure >> 8).pow(2);
            x1 = (x1 * 3038) >> 16; // another number.
            x2 = (-7357 * self.pressure) >> 16;
            self.pressure = self.pressure + ((x1 + x2 + 3791) >> 4); // even moar magic!
            return Ok(self.pressure as f32 / 1000f32); // return as kPa

            // B6 = B5 - 4000;
            // X1 = ( (B2 * ((B6 * B6)>>12)) >> 11 );
            // X2 = ( ((long)AC2* B6) >> 11);
            // X3 = X1 + X2;
            // B3 = ( ( ( ((long)AC1*4) + X3)<< oss) + 2) >> 2;

            // X1 = (((long)AC3*B6) >> 13);
            // X2 = ( B1*((B6*B6) >> 12) ) >> 16;
            // X3 = ( (X1 + X2) + 2 ) >> 2;
            // B4 = ((long)AC4* (ulong)(X3 + 32768)) >> 15;
            //
            // B7 =  (ulong)(UP - B3)*(50000>>oss);
            // if( B7 < 0x80000000 )
            // {
            //  p = (B7 << 1)/B4;
            // }
            // else
            // {
            //  p = (B7/B4) << 1;
            // }
            // X1 = (p >> 8)*(p >> 8);
            // X1 = (X1*3038) >> 16;
            // X2 = ((-7357)*p)>>16;
            // p = p + ( ( X1 + X2 + 3791 ) >> 4);
            //
            // pressure_ = p / 1000.0; // in [kPa]
            // ROS_INFO("pressure: %f",pressure_);
        }
    }
}



#[cfg(test)]
mod tests {

    #[test]
    fn it_works() {}
}
