
extern crate i2cdev;

use std::thread;
use std::time::Duration;

use i2cdev::core::*;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

mod BMP085 {
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

    pub struct BMP085Sensor {
        mut device: LinuxI2CDevice,
        cal_AC1: i16,
        cal_AC2: i16,
        cal_AC3: i16,
        cal_AC4: u16,
        cal_AC5: u16,
        cal_AC6: u16,
        cal_B1: i16,
        cal_B2: i16,
        cal_MB: i16,
        cal_MC: i16,
        cal_MD: i16
    }

    impl BMP085Sensor {
        pub fn new(path:&'static str, addr: Option<u16>) -> BMP085Sensor {
            let address = addr.unwrap_or(BMP085_I2CADDR);
            let mut dev = try!(LinuxI2CDevice::new(path, address))
            let cal_AC1: i16 = dev.smbus_read_word_data(Registers.BMP085_CAL_AC1 as u8)   // INT16
            let cal_AC2: i16  = dev.smbus_read_word_data(Registers.BMP085_CAL_AC2 as u8)   // INT16
            let cal_AC3: i16 = dev.smbus_read_word_data(Registers.BMP085_CAL_AC3 as u8)   // INT16
            let cal_AC4: u16 = dev.smbus_read_word_data(Registers.BMP085_CAL_AC4 as u8)   // UINT16
            let cal_AC5: u16 = dev.smbus_read_word_data(Registers.BMP085_CAL_AC5 as u8)   // UINT16
            let cal_AC6: u16 = dev.smbus_read_word_data(Registers.BMP085_CAL_AC6 as u8)   // UINT16
            let cal_B1: i16 = dev.smbus_read_word_data(Registers.BMP085_CAL_B1 as u8)     // INT16
            let cal_B2: i16 = dev.smbus_read_word_data(Registers.BMP085_CAL_B2 as u8)     // INT16
            let cal_MB: i16 = dev.smbus_read_word_data(Registers.BMP085_CAL_MB as u8)     // INT16
            let cal_MC: i16 = dev.smbus_read_word_data(Registers.BMP085_CAL_MC as u8)     // INT16
            let cal_MD: i16 = dev.smbus_read_word_data(Registers.BMP085_CAL_MD as u8)

            BMP085Sensor{
                device: dev,
                cal_AC1: cal_AC1,
                cal_AC2: cal_AC2,
                cal_AC3: cal_AC3,
                cal_AC4: cal_AC4,
                cal_AC5: cal_AC5,
                cal_AC6: cal_AC6,
                cal_B1: cal_B1,
                cal_B2: cal_B2,
                cal_MB: cal_MB,
                cal_MC: cal_MC,
                cal_MD: cal_MD
            }
        }

        fn calibrate() -> {
            self.device.smbus_read_word_data()
        }

        pub fn asdf(&self, ) -> Result<(), LinuxI2CError> {

            // init sequence
            try!(dev.smbus_write_byte_data(0xF0, 0x55));
            try!(dev.smbus_write_byte_data(0xFB, 0x00));
            thread::sleep(Duration::from_millis(100));

            loop {
                let mut buf: [u8; 6] = [0; 6];
                dev.smbus_write_byte(0x00).unwrap();
                thread::sleep(Duration::from_millis(10));
                dev.read(&mut buf).unwrap();
                println!("Reading: {:?}", buf);
            }
        }


    }

}



#[cfg(test)]
mod tests {

    #[test]
    fn it_works() {
    }
}
