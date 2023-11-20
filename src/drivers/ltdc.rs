use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use hal::gpio::GpioExt;
use hal::gpio::Speed;
use hal::interrupt;
use hal::pac::Interrupt;
use hal::pac::Peripherals as device;
use hal::pac::NVIC;
use stm32f4xx_hal as hal;

use embedded_hal::blocking::delay::DelayUs;

macro_rules! ltdc_pins {
    ($($pin:expr),*) => {
        (
            $(
                $pin.into_push_pull_output()
                    .speed(Speed::VeryHigh)
                    .into_alternate::<14>()
                    .internal_pull_up(false)
            ),*
        )
    };
}

pub struct Ltdc {}
impl Ltdc {
    pub fn new<D>(delay: &mut D) -> bool
    where
        D: DelayUs<u8>,
    {
        let rcc = unsafe { &*stm32f4xx_hal::pac::RCC::ptr() };

        // IO config
        rcc.apb2enr.write(|w| w.ltdcen().set_bit());
        rcc.ahb1enr.write(|w| {
            w.dma2den()
                .set_bit()
                .gpioaen()
                .set_bit()
                .gpioben()
                .set_bit()
                .gpiocen()
                .set_bit()
                .gpioden()
                .set_bit()
                .gpiofen()
                .set_bit()
                .gpiogen()
                .set_bit()
        });

        // Clock config
        // PLLSAI_VCO Input = HSE_VALUE/PLL_M = 2 Mhz
        // PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 Mhz
        // PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/4 = 48 Mhz */
        // LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_8 = 48/8 = 6Mhz
        rcc.cr.write(|w| w.pllsaion().clear_bit());
        while rcc.cr.read().pllsairdy().bit_is_set() {}
        rcc.cr.write(|w| w.pllsaion().clear_bit());
        rcc.pllsaicfgr
            .write(|w| unsafe { w.pllsain().bits(96).pllsaiq().bits(4).pllsair().bits(8) });
        rcc.cr.write(|w| w.pllsaion().set_bit());
        while rcc.cr.read().pllsairdy().bit_is_clear() {}

        info!("LTDC Clock config is OK!");

        return true;
    }
}
