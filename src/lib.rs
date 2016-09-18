extern crate i2cdev;

pub mod bmp085 {
    use std::io::Read;
    use std::thread;
    use std::time::Duration;
    use std::error::Error;

    use i2cdev::core::*;
    use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

    //BMP085 default address.
    const BMP085_I2CADDR: u16          = 0x77;

    enum Modes {
        BMP085_ULTRALOWPOWER     = 0,
        BMP085_STANDARD         = 1,
        BMP085_HIGHRES           = 2,
        BMP085_ULTRAHIGHRES      = 3
    }

    enum Registers {
        BMP085_CAL_AC1           = 0xAA,  // R   Calibration data (16 bits)
        BMP085_CAL_AC2           = 0xAC, // R   Calibration data (16 bits)
        BMP085_CAL_AC3           = 0xAE,  // R   Calibration data (16 bits)
        BMP085_CAL_AC4           = 0xB0,  // R   Calibration data (16 bits)
        BMP085_CAL_AC5           = 0xB2,  // R   Calibration data (16 bits)
        BMP085_CAL_AC6           = 0xB4,  // R   Calibration data (16 bits)
        BMP085_CAL_B1            = 0xB6,  // R   Calibration data (16 bits)
        BMP085_CAL_B2            = 0xB8,  // R   Calibration data (16 bits)
        BMP085_CAL_MB            = 0xBA,  // R   Calibration data (16 bits)
        BMP085_CAL_MC            = 0xBC,  // R   Calibration data (16 bits)
        BMP085_CAL_MD            = 0xBE,  // R   Calibration data (16 bits)
        BMP085_CONTROL           = 0xF4,
        BMP085_DATA              = 0xF6 // Pressure & Temp
    }

    enum Commands {
        BMP085_READTEMPCMD       = 0x2E,
        BMP085_READPRESSURECMD   = 0x34
    }

    fn i_to_be(r: u16) -> Result<i16, LinuxI2CError> {
        let a = r as i16;
        return Ok(a.to_be())
    }

    fn u_to_be(r: u16) -> Result<u16, LinuxI2CError> {
        return Ok(r.to_be())
    }

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
        cal_md: i16
    }

    impl BMP085Sensor {
        pub fn new(path:&'static str, addr: Option<u16>) -> BMP085Sensor {
            let address = addr.unwrap_or(BMP085_I2CADDR);

            let mut dev = LinuxI2CDevice::new(path, address).unwrap();

            let cal_ac1: i16 = dev.smbus_read_word_data(Registers::BMP085_CAL_AC1 as u8)
                .and_then(i_to_be).unwrap_or(408); // INT16
            let cal_ac2: i16  = dev.smbus_read_word_data(Registers::BMP085_CAL_AC2 as u8)
                .and_then(i_to_be).unwrap_or(-72);   // INT16
            let cal_ac3: i16 = dev.smbus_read_word_data(Registers::BMP085_CAL_AC3 as u8)
                .and_then(i_to_be).unwrap_or(-14383);   // INT16
            let cal_ac4: u16 = dev.smbus_read_word_data(Registers::BMP085_CAL_AC4 as u8)
                .and_then(u_to_be).unwrap_or(32741);   // UINT16
            let cal_ac5: u16 = dev.smbus_read_word_data(Registers::BMP085_CAL_AC5 as u8)
                .and_then(u_to_be).unwrap_or(32757);   // UINT16
            let cal_ac6: u16 = dev.smbus_read_word_data(Registers::BMP085_CAL_AC6 as u8)
                .and_then(u_to_be).unwrap_or(23153);   // UINT16
            let cal_b1: i16 = dev.smbus_read_word_data(Registers::BMP085_CAL_B1 as u8)
                .and_then(i_to_be).unwrap_or(6190);     // INT16
            let cal_b2: i16 = dev.smbus_read_word_data(Registers::BMP085_CAL_B2 as u8)
                .and_then(i_to_be).unwrap_or(4);     // INT16
            let cal_mb: i16 = dev.smbus_read_word_data(Registers::BMP085_CAL_MB as u8)
                .and_then(i_to_be).unwrap_or(-32767);     // INT16
            let cal_mc: i16 = dev.smbus_read_word_data(Registers::BMP085_CAL_MC as u8)
                .and_then(i_to_be).unwrap_or(-8711);     // INT16
            let cal_md: i16 = dev.smbus_read_word_data(Registers::BMP085_CAL_MD as u8)
                .and_then(i_to_be).unwrap_or(2868);

            BMP085Sensor{
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
                cal_md: cal_md
            }
        }

            pub fn get_temp(&mut self) -> Result<i16, LinuxI2CError> {
                try!(self.device.smbus_write_byte_data(Registers::BMP085_CONTROL as u8,  Commands::BMP085_READTEMPCMD as u8));
                thread::sleep(Duration::from_millis(5));
                let ut = try!(self.device.smbus_read_word_data(Registers::BMP085_DATA as u8).and_then(i_to_be));
                let ac6 = self.cal_ac6 as i16;
                let ac5 = self.cal_ac5 as i16;
                let x1: i16 = ((ut - ac6) * ac5) >> 15; // Note: X>>15 == X/(pow(2,15))
                let x2: i16 = (self.cal_mc << 11) / (x1 + self.cal_md); // Note: X<<11 == X<<(pow(2,11))
                let t = (x1 + x2 + 8) >> 4;
                println!("TEMP {}", t);
                return Ok(t);
            }


            //pub fn get_pressure(&mut self) -> Result<i16, LinuxI2CError> {
            //
            //}


        //pub fn asdf(&self, ) -> Result<(), LinuxI2CError> {
        //
        //    // init sequence
        //    try!(dev.smbus_write_byte_data(0xF0, 0x55));
        //    try!(dev.smbus_write_byte_data(0xFB, 0x00));
        //    thread::sleep(Duration::from_millis(100));
        //
        //    loop {
        //        let mut buf: [u8; 6] = [0; 6];
        //        dev.smbus_write_byte(0x00).unwrap();
        //        thread::sleep(Duration::from_millis(10));
        //        dev.read(&mut buf).unwrap();
        //        println!("Reading: {:?}", buf);
        //    }
        //}


    }

}



#[cfg(test)]
mod tests {

    #[test]
    fn it_works() {
    }
}
