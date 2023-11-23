use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embedded_hal::blocking::i2c;
use hal::interrupt;
use hal::pac::Interrupt;
use hal::pac::NVIC;
use stm32f4xx_hal as hal;

use embedded_hal::blocking::delay::DelayUs;

pub struct TouchScreen {}
impl TouchScreen {
    pub fn new<D>(delay: &mut D) -> bool
    where
        D: DelayUs<u32>,
    {
        let rcc = unsafe { &*stm32f4xx_hal::pac::RCC::ptr() };

        // Clock config
        rcc.apb1enr.modify(|_, w| w.i2c3en().enabled());
        rcc.ahb1enr
            .modify(|_, w| w.gpioaen().set_bit().gpiocen().set_bit());

        // GPIOs Configuration
        let gpioa = unsafe { &*stm32f4xx_hal::pac::GPIOA::ptr() };
        let gpioc = unsafe { &*stm32f4xx_hal::pac::GPIOC::ptr() };

        // PA 8
        gpioa.moder.modify(|_, w| w.moder8().alternate());
        gpioa.otyper.modify(|_, w| w.ot8().open_drain());
        gpioa.afrh.modify(|_, w| w.afrh8().af4());
        gpioa.ospeedr.modify(|_, w| w.ospeedr8().high_speed());

        // PC 9
        gpioc.moder.modify(|_, w| w.moder9().alternate());
        gpioc.otyper.modify(|_, w| w.ot9().open_drain());
        gpioa.afrh.modify(|_, w| w.afrh9().af4());
        gpioa.ospeedr.modify(|_, w| w.ospeedr9().high_speed());

        //
        rcc.apb1rstr.modify(|_, w| w.i2c3rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.i2c3rst().clear_bit());

        let i2c3_dev = unsafe { &*stm32f4xx_hal::pac::I2C3::ptr() };

        unsafe { NVIC::unmask(Interrupt::I2C3_EV) };
        unsafe { NVIC::unmask(Interrupt::I2C3_ER) };
        // TODO: set priority

        return true;
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
