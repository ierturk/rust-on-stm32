use core::borrow::BorrowMut;

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

pub struct Ltdc {}
impl Ltdc {
    pub fn new<D>(delay: &mut D) -> bool
    where
        D: DelayUs<u8>,
    {
        let rcc = unsafe { &*stm32f4xx_hal::pac::RCC::ptr() };

        /* GPIOs Configuration */
        /*
        +------------------------+-----------------------+----------------------------+
        +                       LCD pins assignment                                   +
        +------------------------+-----------------------+----------------------------+
        |  LCD_TFT R2 <-> PC.10  |  LCD_TFT G2 <-> PA.06 |  LCD_TFT B2 <-> PD.06      |
        |  LCD_TFT R3 <-> PB.00  |  LCD_TFT G3 <-> PG.10 |  LCD_TFT B3 <-> PG.11      |
        |  LCD_TFT R4 <-> PA.11  |  LCD_TFT G4 <-> PB.10 |  LCD_TFT B4 <-> PG.12      |
        |  LCD_TFT R5 <-> PA.12  |  LCD_TFT G5 <-> PB.11 |  LCD_TFT B5 <-> PA.03      |
        |  LCD_TFT R6 <-> PB.01  |  LCD_TFT G6 <-> PC.07 |  LCD_TFT B6 <-> PB.08      |
        |  LCD_TFT R7 <-> PG.06  |  LCD_TFT G7 <-> PD.03 |  LCD_TFT B7 <-> PB.09      |
        -------------------------------------------------------------------------------
                |  LCD_TFT HSYNC <-> PC.06  | LCDTFT VSYNC <->  PA.04 |
                |  LCD_TFT CLK   <-> PG.07  | LCD_TFT DE   <->  PF.10 |
                 -----------------------------------------------------
        */

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

        /*
                #[rustfmt::skip]
                let _ = ltdc_pins!(
                    // R2-R7
                    gc.pc10,
                    // gb.pb0,
                    ga.pa11,
                    ga.pa12,
                    // gb.pb1,
                    gg.pg6,
                    // G2-G7
                    ga.pa6,
                    gg.pg10,
                    gb.pb10,
                    gb.pb11,
                    gc.pc7,
                    gd.pd3,
                    // B2-B7
                    gd.pd6,
                    gg.pg11,
                    gg.pg12,
                    ga.pa3,
                    gb.pb8,
                    gb.pb9,
                    // LCD_TFT HSYNC
                    gc.pc6,
                    // LCDTFT VSYNC
                    ga.pa4,
                    // LCD_TFT CLK
                    gg.pg7,
                    // LCD_TFT DE
                    gf.pf10
                );
        */

        // PA 3, 4, 6, 11, 12
        let gpioa = unsafe { &*stm32f4xx_hal::pac::GPIOA::ptr() };
        gpioa.otyper.modify(|_, w| {
            w.ot3()
                .push_pull()
                .ot4()
                .push_pull()
                .ot6()
                .push_pull()
                .ot11()
                .push_pull()
                .ot12()
                .push_pull()
        });

        gpioa
            .afrl
            .modify(|_, w| w.afrl3().af14().afrl4().af14().afrl6().af14());
        gpioa.afrh.modify(|_, w| w.afrh11().af14().afrh12().af14());

        gpioa.moder.modify(|_, w| {
            w.moder3()
                .bits(0x02)
                .moder4()
                .bits(0x02)
                .moder5()
                .bits(0x02)
                .moder11()
                .bits(0x02)
                .moder12()
                .bits(0x02)
        });

        gpioa.pupdr.modify(|_, w| {
            w.pupdr3()
                .floating()
                .pupdr4()
                .floating()
                .pupdr6()
                .floating()
                .pupdr11()
                .floating()
                .pupdr12()
                .floating()
        });

        gpioa.ospeedr.modify(|_, w| {
            w.ospeedr3()
                .very_high_speed()
                .ospeedr4()
                .very_high_speed()
                .ospeedr6()
                .very_high_speed()
                .ospeedr11()
                .very_high_speed()
                .ospeedr12()
                .very_high_speed()
        });

        // PB 0, 1, 8, 9, 10, 11
        let gpiob = unsafe { &*stm32f4xx_hal::pac::GPIOB::ptr() };
        gpiob.otyper.modify(|_, w| {
            w.ot0()
                .push_pull()
                .ot1()
                .push_pull()
                .ot8()
                .push_pull()
                .ot9()
                .push_pull()
                .ot10()
                .push_pull()
                .ot11()
                .push_pull()
        });

        gpiob.afrl.modify(|_, w| w.afrl0().af14().afrl1().af14());
        gpiob.afrh.modify(|_, w| {
            w.afrh8()
                .af14()
                .afrh9()
                .af14()
                .afrh10()
                .af14()
                .afrh11()
                .af14()
        });

        gpiob.moder.modify(|_, w| {
            w.moder0()
                .bits(0x02)
                .moder1()
                .bits(0x02)
                .moder8()
                .bits(0x02)
                .moder9()
                .bits(0x02)
                .moder10()
                .bits(0x02)
                .moder11()
                .bits(0x02)
        });

        gpiob.pupdr.modify(|_, w| {
            w.pupdr0()
                .floating()
                .pupdr1()
                .floating()
                .pupdr8()
                .floating()
                .pupdr9()
                .floating()
                .pupdr10()
                .floating()
                .pupdr11()
                .floating()
        });

        gpiob.ospeedr.modify(|_, w| {
            w.ospeedr0()
                .very_high_speed()
                .ospeedr1()
                .very_high_speed()
                .ospeedr8()
                .very_high_speed()
                .ospeedr9()
                .very_high_speed()
                .ospeedr10()
                .very_high_speed()
                .ospeedr11()
                .very_high_speed()
        });

        // PC 6, 7, 10
        let gpioc = unsafe { &*stm32f4xx_hal::pac::GPIOC::ptr() };
        gpioc
            .otyper
            .modify(|_, w| w.ot6().push_pull().ot7().push_pull().ot10().push_pull());

        gpioc.afrl.modify(|_, w| w.afrl6().af14().afrl7().af14());
        gpioc.afrh.modify(|_, w| w.afrh10().af14());

        gpioc.moder.modify(|_, w| {
            w.moder6()
                .bits(0x02)
                .moder7()
                .bits(0x02)
                .moder10()
                .bits(0x02)
        });

        gpioc.pupdr.modify(|_, w| {
            w.pupdr6()
                .floating()
                .pupdr7()
                .floating()
                .pupdr10()
                .floating()
        });

        gpioc.ospeedr.modify(|_, w| {
            w.ospeedr6()
                .very_high_speed()
                .ospeedr7()
                .very_high_speed()
                .ospeedr10()
                .very_high_speed()
        });

        // PD 3, 6
        let gpiod = unsafe { &*stm32f4xx_hal::pac::GPIOD::ptr() };
        gpiod
            .otyper
            .modify(|_, w| w.ot3().push_pull().ot6().push_pull());

        gpiod.afrl.modify(|_, w| w.afrl3().af14().afrl6().af14());

        gpiod
            .moder
            .modify(|_, w| w.moder3().bits(0x02).moder6().bits(0x02));

        gpiod
            .pupdr
            .modify(|_, w| w.pupdr3().floating().pupdr6().floating());

        gpiod
            .ospeedr
            .modify(|_, w| w.ospeedr3().very_high_speed().ospeedr6().very_high_speed());

        // PF 10
        let gpiof = unsafe { &*stm32f4xx_hal::pac::GPIOF::ptr() };
        gpiof.otyper.modify(|_, w| w.ot10().push_pull());

        gpiof.afrh.modify(|_, w| w.afrh10().af14());

        gpiof.moder.modify(|_, w| w.moder10().bits(0x02));

        gpiof.pupdr.modify(|_, w| w.pupdr10().floating());

        gpiof.ospeedr.modify(|_, w| w.ospeedr10().very_high_speed());

        // PG 6, 7, 10, 11, 12
        let gpiog = unsafe { &*stm32f4xx_hal::pac::GPIOG::ptr() };
        gpiog.otyper.modify(|_, w| {
            w.ot6()
                .push_pull()
                .ot7()
                .push_pull()
                .ot10()
                .push_pull()
                .ot11()
                .push_pull()
                .ot12()
                .push_pull()
        });

        gpiog.afrl.modify(|_, w| w.afrl6().af14().afrl7().af14());
        gpiog
            .afrh
            .modify(|_, w| w.afrh10().af14().afrh11().af14().afrh10().af12());

        gpiog.moder.modify(|_, w| {
            w.moder6()
                .bits(0x02)
                .moder7()
                .bits(0x02)
                .moder10()
                .bits(0x02)
                .moder11()
                .bits(0x02)
                .moder12()
                .bits(0x02)
        });

        gpiog.pupdr.modify(|_, w| {
            w.pupdr6()
                .floating()
                .pupdr7()
                .floating()
                .pupdr10()
                .floating()
                .pupdr11()
                .floating()
                .pupdr12()
                .floating()
        });

        gpiog.ospeedr.modify(|_, w| {
            w.ospeedr6()
                .very_high_speed()
                .ospeedr7()
                .very_high_speed()
                .ospeedr10()
                .very_high_speed()
                .ospeedr11()
                .very_high_speed()
                .ospeedr12()
                .very_high_speed()
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
