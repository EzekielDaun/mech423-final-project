#![allow(dead_code)]
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy)]
pub enum TMAG5273Address {
    TMAG5273_I2C_ADDRESS_0X22 = 0x22,
}

#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy)]
pub enum TMAG5273Register {
    TMAG5273_REG_DEVICE_CONFIG_1 = 0x00,
    TMAG5273_REG_DEVICE_CONFIG_2 = 0x01,
    TMAG5273_REG_SENSOR_CONFIG_1 = 0x02,
    TMAG5273_REG_SENSOR_CONFIG_2 = 0x03,
    TMAG5273_REG_X_THR_CONFIG = 0x04,
    TMAG5273_REG_Y_THR_CONFIG = 0x05,
    TMAG5273_REG_Z_THR_CONFIG = 0x06,
    TMAG5273_REG_T_CONFIG = 0x07,
    TMAG5273_REG_INT_CONFIG_1 = 0x08,
    TMAG5273_REG_MAG_GAIN_CONFIG = 0x09,
    TMAG5273_REG_MAG_OFFSET_CONFIG_1 = 0x0A,
    TMAG5273_REG_MAG_OFFSET_CONFIG_2 = 0x0B,
    TMAG5273_REG_I2C_ADDRESS = 0x0C,
    TMAG5273_REG_DEVICE_ID = 0x0D,
    TMAG5273_REG_MANUFACTURER_ID_LSB = 0x0E,
    TMAG5273_REG_MANUFACTURER_ID_MSB = 0x0F,
    TMAG5273_REG_T_MSB_RESULT = 0x10,
    TMAG5273_REG_T_LSB_RESULT = 0x11,
    TMAG5273_REG_X_MSB_RESULT = 0x12,
    TMAG5273_REG_X_LSB_RESULT = 0x13,
    TMAG5273_REG_Y_MSB_RESULT = 0x14,
    TMAG5273_REG_Y_LSB_RESULT = 0x15,
    TMAG5273_REG_Z_MSB_RESULT = 0x16,
    TMAG5273_REG_Z_LSB_RESULT = 0x17,
    TMAG5273_REG_CONV_STATUS = 0x18,
    TMAG5273_REG_ANGLE_RESULT_MSB = 0x19,
    TMAG5273_REG_ANGLE_RESULT_LSB = 0x1A,
    TMAG5273_REG_MAGNITUDE_RESULT = 0x1B,
    TMAG5273_REG_DEVICE_STATUS = 0x1C,
}

impl From<u8> for TMAG5273Address {
    fn from(value: u8) -> Self {
        match value {
            0x22 => TMAG5273Address::TMAG5273_I2C_ADDRESS_0X22,
            _ => panic!("Invalid address"),
        }
    }
}

pub struct TMAG5273<'a, I> {
    i2c: &'a mut I,
    i2c_address: TMAG5273Address,
}

impl<'a, I, E> TMAG5273<'a, I>
where
    I: Write<Error = E> + Read<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: &'a mut I, i2c_address: TMAG5273Address) -> Self {
        Self { i2c, i2c_address }
    }

    pub fn read(&mut self) -> Result<u8, E> {
        let mut data = [0x0E];
        let mut buffer = [0x00];

        self.i2c
            .write_read(self.i2c_address as u8, &mut data, &mut buffer)?;
        Ok(buffer[0])
    }

    pub fn read_manufacturer_id(&mut self) -> Result<u16, E> {
        let mut data = [TMAG5273Register::TMAG5273_REG_MANUFACTURER_ID_LSB as u8];
        let mut buffer = [0x00, 0x00];

        self.i2c
            .write_read(self.i2c_address as u8, &mut data, &mut buffer)?;
        Ok(u16::from_be_bytes(buffer))
    }

    pub fn set_operating_mode(&mut self) -> Result<(), E> {
        self.i2c.write(
            self.i2c_address as u8,
            &[
                TMAG5273Register::TMAG5273_REG_DEVICE_CONFIG_2 as u8,
                0b0000_0010,
            ],
        )
    }

    pub fn set_mag_ch_en(&mut self) -> Result<(), E> {
        self.i2c.write(
            self.i2c_address as u8,
            &[
                TMAG5273Register::TMAG5273_REG_SENSOR_CONFIG_1 as u8,
                0x03 << 4,
            ],
        )
    }

    pub fn set_angle_en(&mut self) -> Result<(), E> {
        self.i2c.write(
            self.i2c_address as u8,
            &[
                TMAG5273Register::TMAG5273_REG_SENSOR_CONFIG_2 as u8,
                0x01 << 2,
            ],
        )
    }

    pub fn read_angle(&mut self) -> Result<f32, E> {
        let mut data = [TMAG5273Register::TMAG5273_REG_ANGLE_RESULT_MSB as u8];
        let mut buffer = [0x00, 0x00];

        self.i2c
            .write_read(self.i2c_address as u8, &mut data, &mut buffer)?;

        let angle_reg = u16::from_be_bytes(buffer);
        let decimal: f32 = (buffer[1] & 0b1111) as f32 / 16 as f32;

        Ok((angle_reg >> 4) as f32 + decimal)
    }
}
