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

macro_rules! lcd_wrx_high {
    () => {
        let gpiod = unsafe { &*stm32f4xx_hal::pac::GPIOD::ptr() };
        gpiod.bsrr.write(|w| w.bs13().set_bit());
    };
}

macro_rules! lcd_wrx_low {
    () => {
        let gpiod = unsafe { &*stm32f4xx_hal::pac::GPIOD::ptr() };
        gpiod.bsrr.write(|w| w.br13().set_bit());
    };
}

macro_rules! lcd_cs_high {
    () => {
        let gpioc = unsafe { &*stm32f4xx_hal::pac::GPIOC::ptr() };
        gpioc.bsrr.write(|w| w.bs2().set_bit());
    };
}

macro_rules! lcd_cs_low {
    () => {
        let gpioc = unsafe { &*stm32f4xx_hal::pac::GPIOC::ptr() };
        gpioc.bsrr.write(|w| w.br2().set_bit());
    };
}

macro_rules! spi5_send {
    ($in:expr) => {
        lcd_wrx_low!();
        lcd_cs_low!();
        let spi5_dev = unsafe { &*stm32f4xx_hal::pac::SPI5::ptr() };

        if spi5_dev.cr1.read().spe().bit_is_clear() {
            spi5_dev.cr1.modify(|_, w| w.spe().set_bit());
        }

        spi5_dev.dr.write(|w| unsafe { w.bits($in) });

        while spi5_dev.sr.read().bsy().bit_is_set() || spi5_dev.sr.read().txe().bit_is_clear() {}

        lcd_cs_high!();
    };
}

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

        rcc.apb2enr.modify(|_, w| w.ltdcen().set_bit());
        rcc.ahb1enr.modify(|_, w| {
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
        rcc.cr.modify(|_, w| w.pllsaion().clear_bit());
        while rcc.cr.read().pllsairdy().bit_is_set() {}
        rcc.cr.modify(|_, w| w.pllsaion().clear_bit());
        rcc.pllsaicfgr
            .modify(|_, w| unsafe { w.pllsain().bits(96).pllsaiq().bits(4).pllsair().bits(8) });
        rcc.cr.modify(|_, w| w.pllsaion().set_bit());
        while rcc.cr.read().pllsairdy().bit_is_clear() {}

        // info!("LTDC Clock config is OK!");

        // LTDC config
        let ltd_dev = unsafe { &*stm32f4xx_hal::pac::LTDC::ptr() };

        ltd_dev.gcr.modify(|_, w| {
            w.hspol()
                .active_low()
                .vspol()
                .active_low()
                .hspol()
                .active_low()
                .pcpol()
                .rising_edge()
        });

        ltd_dev.sscr.modify(|_, w| w.hsw().bits(9).vsh().bits(1));
        ltd_dev.bpcr.modify(|_, w| w.ahbp().bits(29).avbp().bits(3));
        ltd_dev
            .awcr
            .modify(|_, w| w.aaw().bits(269).aah().bits(323));
        ltd_dev
            .twcr
            .modify(|_, w| w.totalw().bits(279).totalh().bits(320));
        ltd_dev
            .bccr
            .modify(|_, w| w.bcred().bits(0).bcgreen().bits(0).bcblue().bits(0));

        ltd_dev
            .ier
            .modify(|_, w| w.fuie().enabled().terrie().enabled());

        ltd_dev.gcr.modify(|_, w| w.ltdcen().enabled());

        // unsafe { NVIC::unmask(Interrupt::LCD_TFT) };
        // unsafe { NVIC::unmask(Interrupt::LCD_TFT_1) };
        // unsafe { NVIC::unmask(Interrupt::DMA2D) };

        // ILI931 LCD IO init
        // GPIO init
        // GPIOD is already enabled
        // LCD RDX PD12
        // LCD WRX PD 13
        gpiod
            .otyper
            .modify(|_, w| w.ot12().push_pull().ot13().push_pull());
        gpiod
            .pupdr
            .modify(|_, w| w.pupdr12().floating().pupdr12().floating());
        gpiod.ospeedr.modify(|_, w| {
            w.ospeedr12()
                .very_high_speed()
                .ospeedr13()
                .very_high_speed()
        });
        // LCD NCS PC2
        gpioc.otyper.modify(|_, w| w.ot2().push_pull());
        gpioc.pupdr.modify(|_, w| w.pupdr2().floating());
        gpioc.ospeedr.modify(|_, w| w.ospeedr2().very_high_speed());

        lcd_cs_low!();
        lcd_cs_high!();

        // LCD SPI init
        /* SPI baudrate is set to 5.6 MHz (PCLK2/SPI_BaudRatePrescaler = 84/16 = 5.25 MHz)
           to verify these constraints:
           - ILI9341 LCD SPI interface max baudrate is 10MHz for write and 6.66MHz for read
           - l3gd20 SPI interface max baudrate is 10MHz for write/read
           - PCLK2 frequency is set to 84 MHz
        */
        // GPIO SPI5 CLK PF7, SPI5 MISO PF8, SPI5 MSI PF9
        rcc.apb2enr.modify(|_, w| w.spi5en().set_bit());

        gpiof
            .otyper
            .modify(|_, w| w.ot7().push_pull().ot8().push_pull().ot9().push_pull());

        gpiof.afrl.modify(|_, w| w.afrl7().af5());
        gpiof.afrh.modify(|_, w| w.afrh8().af5().afrh9().af5());

        gpiof.moder.modify(|_, w| {
            w.moder7()
                .bits(0x02)
                .moder8()
                .bits(0x02)
                .moder9()
                .bits(0x02)
        });

        gpiof.pupdr.modify(|_, w| {
            w.pupdr7()
                .pull_down()
                .pupdr8()
                .pull_down()
                .pupdr9()
                .pull_down()
        });

        gpiof.ospeedr.modify(|_, w| {
            w.ospeedr7()
                .medium_speed()
                .ospeedr8()
                .medium_speed()
                .ospeedr9()
                .medium_speed()
        });

        // SPI5 init
        let spi5_dev = unsafe { &*stm32f4xx_hal::pac::SPI5::ptr() };

        spi5_dev.cr1.modify(|_, w| {
            w.mstr()
                .set_bit()
                .dff()
                .eight_bit()
                .cpol()
                .idle_low()
                .cpha()
                .first_edge()
                .br()
                .div16()
                .ssi()
                .set_bit()
                .ssm()
                .set_bit()
        });

        spi5_send!(0x55);
        info!("LCD SP5 seems to be functional!");

        return true;
    }
}

#[interrupt]
fn LCD_TFT() {
    cortex_m::interrupt::free(|_| {
        let ltd_dev = unsafe { &*stm32f4xx_hal::pac::LTDC::ptr() };

        if ltd_dev.isr.read().terrif().bit_is_set() && ltd_dev.ier.read().terrie().bit_is_set() {
            ltd_dev.ier.modify(|_, w| w.terrie().clear_bit());
            ltd_dev.icr.write(|w| w.cterrif().set_bit());
        }

        if ltd_dev.isr.read().fuif().bit_is_set() && ltd_dev.ier.read().fuie().bit_is_set() {
            ltd_dev.ier.modify(|_, w| w.fuie().clear_bit());
            ltd_dev.icr.write(|w| w.cfuif().set_bit());
        }

        if ltd_dev.isr.read().lif().bit_is_set() && ltd_dev.ier.read().lie().bit_is_set() {
            ltd_dev.ier.modify(|_, w| w.lie().clear_bit());
            ltd_dev.icr.write(|w| w.clif().set_bit());
        }

        if ltd_dev.isr.read().rrif().bit_is_set() && ltd_dev.ier.read().rrie().bit_is_set() {
            ltd_dev.ier.modify(|_, w| w.rrie().clear_bit());
            ltd_dev.icr.write(|w| w.crrif().set_bit());
        }
    });
}

#[interrupt]
fn LCD_TFT_1() {
    cortex_m::interrupt::free(|_| {
        let ltd_dev = unsafe { &*stm32f4xx_hal::pac::LTDC::ptr() };

        if ltd_dev.isr.read().fuif().is_underrun() {
            ltd_dev.icr.write(|w| w.cfuif().set_bit())
        }
    });
}

#[interrupt]
fn DMA2D() {
    cortex_m::interrupt::free(|_| {});
}
