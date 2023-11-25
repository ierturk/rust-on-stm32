use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use hal::gpio::GpioExt;
use hal::i2c::I2c;
use hal::i2c::I2c3;
use hal::i2c::I2cExt;
use hal::i2c::Instance;
use hal::i2c::Mode;
use hal::interrupt;
use hal::pac::can1::rx;
use hal::pac::Interrupt;
use hal::pac::I2C3;
use hal::pac::NVIC;
use hal::prelude::*;
use hal::rcc::Clocks;
use stm32f4xx_hal as hal;

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::i2c::WriteRead;

const STMPE811_ADDR: u8 = 0x41;
const STMPE811_ID: u8 = 0x00;

pub struct TouchScreen {
    pub dev: I2c<I2C3>,
}

impl TouchScreen {
    pub fn init(clocks: &Clocks, delay: &dyn DelayUs<u16>) -> stm32f4xx_hal::i2c::I2c<I2C3> {
        let dp = unsafe { stm32f4xx_hal::pac::Peripherals::steal() };
        let gpioa = dp.GPIOA.split();
        let scl = gpioa.pa8.into_alternate_open_drain::<4>();
        let gpioc = dp.GPIOC.split();
        let sda = gpioc.pc9.into_alternate_open_drain::<4>();

        dp.I2C3.i2c(
            (scl, sda),
            Mode::Standard {
                frequency: 100.kHz(),
            },
            &clocks,
        )
    }

    pub fn read_ts_id(&mut self) -> [u8; 3] {
        let mut rx_buffer = [0_u8; 3];
        self.dev.write(STMPE811_ADDR, &[STMPE811_ID]).unwrap();
        self.dev.read(STMPE811_ADDR, &mut rx_buffer).unwrap();
        rx_buffer
    }
}

#[interrupt]
fn I2C3_EV() {
    cortex_m::interrupt::free(|_| {
        let i2c3_dev = unsafe { &*stm32f4xx_hal::pac::LTDC::ptr() };
    });
}

#[interrupt]
fn I2C3_ER() {
    cortex_m::interrupt::free(|_| {
        let i2c3_dev = unsafe { &*stm32f4xx_hal::pac::LTDC::ptr() };
    });
}
