use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use hal::i2c::I2c;
use hal::interrupt;
use hal::pac::I2C3;
use stm32f4xx_hal as hal;

use embedded_hal::blocking::delay::DelayUs;

const STMPE811_ADDR: u8 = 0x41;
const STMPE811_ID: u8 = 0x00;

const STMPE811_IO_FCT: u8 = 0x04;
const STMPE811_TOUCH_IO_ALL: u8 = 0xf0;
const STMPE811_ADC_FCT: u8 = 0x01;
const STMPE811_TS_FCT: u8 = 0x02;

const STMPE811_REG_SYS_CTRL1: u8 = 0x03;
const STMPE811_REG_SYS_CTRL2: u8 = 0x04;
const STMPE811_REG_IO_AF: u8 = 0x17;
const STMPE811_REG_ADC_CTRL1: u8 = 0x20;
const STMPE811_REG_ADC_CTRL2: u8 = 0x21;
const STMPE811_REG_TSC_CFG: u8 = 0x41;
const STMPE811_REG_FIFO_TH: u8 = 0x4A;
const STMPE811_REG_FIFO_STA: u8 = 0x4B;
const STMPE811_REG_TSC_FRACT_XYZ: u8 = 0x56;
const STMPE811_REG_TSC_I_DRIVE: u8 = 0x58;
const STMPE811_REG_TSC_CTRL: u8 = 0x40;
const STMPE811_REG_INT_STA: u8 = 0x0B;
const STMPE811_REG_TSC_DATA_NON_INC: u8 = 0xD7;

pub struct TouchScreen {
    pub i2c_dev: I2c<I2C3>,
}

impl TouchScreen {
    pub fn new<D>(&mut self, delay: &mut D) -> bool
    where
        D: DelayUs<u32>,
    {
        let mut rx_buffer = [0_u8; 2];
        let mut rx_byte: u8 = 0;

        self.i2c_dev.write(STMPE811_ADDR, &[STMPE811_ID]).unwrap();
        self.i2c_dev.read(STMPE811_ADDR, &mut rx_buffer).unwrap();

        info!("I2C3 TS ID:{:04x}", rx_buffer);

        // Reset TS Device
        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_SYS_CTRL1, 2])
            .unwrap();

        delay.delay_us(10_000_u32);

        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_SYS_CTRL1, 2])
            .unwrap();

        delay.delay_us(2_000_u32);

        // Start TS Device

        // Set the Functionalities to be Enabled
        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_SYS_CTRL2])
            .unwrap();
        self.i2c_dev.read(STMPE811_ADDR, &mut rx_buffer).unwrap();

        let mut mode = rx_buffer[0] | !STMPE811_IO_FCT;

        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_SYS_CTRL2, mode])
            .unwrap();

        // Select TSC pins in TSC alternate mode
        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_IO_AF])
            .unwrap();
        self.i2c_dev.read(STMPE811_ADDR, &mut rx_buffer).unwrap();

        let alt_mode = rx_buffer[0] | !STMPE811_TOUCH_IO_ALL;

        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_IO_AF, alt_mode])
            .unwrap();

        // Set the Functionalities to be Enabled
        mode &= !(STMPE811_TS_FCT | STMPE811_ADC_FCT);

        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_SYS_CTRL2, mode])
            .unwrap();

        // Select Sample Time, bit number and ADC Reference
        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_ADC_CTRL1, 0x49])
            .unwrap();
        delay.delay_us(2_000_u32);

        // Select the ADC clock speed: 3.25 MHz
        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_ADC_CTRL2, 0x01])
            .unwrap();

        /* Select 2 nF filter capacitor */
        /* Configuration:
           - Touch average control    : 4 samples
           - Touch delay time         : 500 uS
           - Panel driver setting time: 500 uS
        */
        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_TSC_CFG, 0x9A])
            .unwrap();

        // Configure the Touch FIFO threshold: single point reading
        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_FIFO_TH, 0x01])
            .unwrap();

        // Clear the FIFO memory content.
        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_FIFO_STA, 0x01])
            .unwrap();

        // Put the FIFO back into operation mode
        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_FIFO_STA, 0x00])
            .unwrap();

        /* Set the range and accuracy pf the pressure measurement (Z) :
           - Fractional part :7
           - Whole part      :1
        */
        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_TSC_FRACT_XYZ, 0x01])
            .unwrap();

        /* Set the driving capability (limit) of the device for TSC pins: 50mA */
        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_TSC_I_DRIVE, 0x01])
            .unwrap();

        /* Touch screen control configuration (enable TSC):
          - No window tracking index
          - XYZ acquisition mode
        */
        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_TSC_CTRL, 0x01])
            .unwrap();

        /*  Clear all the status pending bits if any */
        self.i2c_dev
            .write(STMPE811_ADDR, &[STMPE811_REG_INT_STA, 0xff])
            .unwrap();

        /* Wait for 2 ms delay */
        delay.delay_us(2_000_u32);

        loop {
            let mut rx_xyz = [0_u8; 4];
            self.i2c_dev
                .write(STMPE811_ADDR, &[STMPE811_REG_TSC_DATA_NON_INC])
                .unwrap();
            self.i2c_dev.read(STMPE811_ADDR, &mut rx_xyz).unwrap();

            self.i2c_dev
                .write(0x41, &[STMPE811_REG_FIFO_STA, 1])
                .unwrap();
            self.i2c_dev
                .write(0x41, &[STMPE811_REG_FIFO_STA, 0])
                .unwrap();

            let data_xyz: u64 = ((rx_xyz[0] as u64) << 24)
                | ((rx_xyz[1] as u64) << 16)
                | ((rx_xyz[2] as u64) << 8)
                | ((rx_xyz[3] as u64) << 0);

            let x = (data_xyz >> 20) & 0x00000FFF;
            let y = (data_xyz >> 8) & 0x00000FFF;
            let z = data_xyz & 0xff;

            info!("TS XYZ: {} - {} - {}", x, y, z);
            delay.delay_us(1_000_000_u32);
        }

        return true;
    }
}

#[interrupt]
fn I2C3_EV() {
    cortex_m::interrupt::free(|_| {
        let _i2c3_dev = unsafe { &*stm32f4xx_hal::pac::LTDC::ptr() };
    });
}

#[interrupt]
fn I2C3_ER() {
    cortex_m::interrupt::free(|_| {
        let _i2c3_dev = unsafe { &*stm32f4xx_hal::pac::LTDC::ptr() };
    });
}
