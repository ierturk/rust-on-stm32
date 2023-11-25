use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use hal::interrupt;
use hal::pac::Interrupt;
use hal::pac::NVIC;
use hal::spi::Spi5;
use stm32f4xx_hal as hal;

use embedded_hal::blocking::delay::DelayUs;

// const LCD_SWRESET: u32 = 0x01;
const LCD_SLEEP_OUT: u8 = 0x11;
// const LCD_WRITE_MEM_CONTINUE: u8 = 0x3c;
// const LCD_PIXEL_FORMAT: u8 = 0x3a;
const LCD_DISPLAY_ON: u8 = 0x29;

const LCD_INTERFACE: u8 = 0xf6;
const LCD_COLUMN_ADDR: u8 = 0x2a;
const LCD_PAGE_ADDR: u8 = 0x2b;
const LCD_POWERB: u8 = 0xCF;
const LCD_POWER_SEQ: u8 = 0xED;
const LCD_DTCA: u8 = 0xE8;
const LCD_POWERA: u8 = 0xCB;
const LCD_PRC: u8 = 0xF7;
const LCD_DTCB: u8 = 0xEA;
const LCD_FRMCTR1: u8 = 0xb1;
const LCD_POWER1: u8 = 0xC0;
const LCD_POWER2: u8 = 0xC1;
const LCD_VCOM1: u8 = 0xC5;
const LCD_VCOM2: u8 = 0xC7;
const LCD_MAC: u8 = 0x36;
const LCD_3GAMMA_EN: u8 = 0xF2;
const LCD_RGB_INTERFACE: u8 = 0xb0;
const LCD_DFC: u8 = 0xb6;
const LCD_GRAM: u8 = 0x2C;
const LCD_GAMMA: u8 = 0x26;
const LCD_PGAMMA: u8 = 0xE0;
const LCD_NGAMMA: u8 = 0xE1;

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

macro_rules! spi_tx {
    ($device:expr, $data:expr) => {
        lcd_cs_low!();
        $device.write(&[$data]).unwrap();
        lcd_cs_high!();
    };
}

macro_rules! LCD_IO_WriteData {
    ($device:expr, $data:expr) => {
        lcd_wrx_high!();
        spi_tx!($device, $data);
    };
}

macro_rules! LCD_IO_WriteReg {
    ($device:expr, $data:expr) => {
        lcd_wrx_low!();
        spi_tx!($device, $data);
    };
}

pub struct Ltdc {
    pub spi_dev: Spi5,
}
impl Ltdc {
    pub fn new<D>(&mut self, delay: &mut D) -> bool
    where
        D: DelayUs<u32>,
    {
        let _ = self.spi_dev.write(&[0x00]);

        let rcc = unsafe { &*stm32f4xx_hal::pac::RCC::ptr() };
        rcc.apb2enr.modify(|_, w| w.ltdcen().set_bit());

        // Clock config
        // PLLSAI_VCO Input = HSE_VALUE/PLL_M = 2 Mhz
        // PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 Mhz
        // PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/4 = 48 Mhz */
        // LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_8 = 48/8 = 6Mhz
        rcc.cr.modify(|_, w| w.pllsaion().clear_bit());
        while rcc.cr.read().pllsairdy().bit_is_set() {}
        rcc.cr.modify(|_, w| w.pllsaion().clear_bit());
        rcc.pllsaicfgr
            .modify(|_, w| unsafe { w.pllsain().bits(180).pllsair().bits(5) });
        rcc.dckcfgr.modify(|_, w| unsafe { w.pllsaidivr().div8() });
        rcc.cr.modify(|_, w| w.pllsaion().set_bit());
        while rcc.cr.read().pllsairdy().bit_is_clear() {}

        // LTDC config
        let ltd_dev = unsafe { &*stm32f4xx_hal::pac::LTDC::ptr() };

        ltd_dev.gcr.modify(|_, w| {
            w.hspol()
                .active_low()
                .vspol()
                .active_low()
                .depol()
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
            .modify(|_, w| w.totalw().bits(279).totalh().bits(327));
        ltd_dev
            .bccr
            .modify(|_, w| w.bcred().bits(0).bcgreen().bits(0).bcblue().bits(0));

        ltd_dev
            .gcr
            .modify(|r, w| unsafe { w.bits(r.bits() | 0x2220) });

        ltd_dev.gcr.modify(|_, w| w.den().enabled());

        ltd_dev.gcr.modify(|_, w| w.ltdcen().enabled());

        unsafe { NVIC::unmask(Interrupt::LCD_TFT) };
        unsafe { NVIC::unmask(Interrupt::LCD_TFT_1) };
        // unsafe { NVIC::unmask(Interrupt::DMA2D) };

        info!("LTDC configured!");

        // Configure LCD
        LCD_IO_WriteReg!(self.spi_dev, 0xCA); //???
        LCD_IO_WriteData!(self.spi_dev, 0xC3);
        LCD_IO_WriteData!(self.spi_dev, 0x08);
        LCD_IO_WriteData!(self.spi_dev, 0x50);

        LCD_IO_WriteReg!(self.spi_dev, LCD_POWERB);
        LCD_IO_WriteData!(self.spi_dev, 0x00);
        LCD_IO_WriteData!(self.spi_dev, 0xC1);
        LCD_IO_WriteData!(self.spi_dev, 0x30);

        LCD_IO_WriteReg!(self.spi_dev, LCD_POWER_SEQ);
        LCD_IO_WriteData!(self.spi_dev, 0x64);
        LCD_IO_WriteData!(self.spi_dev, 0x03);
        LCD_IO_WriteData!(self.spi_dev, 0x12);
        LCD_IO_WriteData!(self.spi_dev, 0x81);

        LCD_IO_WriteReg!(self.spi_dev, LCD_DTCA);
        LCD_IO_WriteData!(self.spi_dev, 0x85);
        LCD_IO_WriteData!(self.spi_dev, 0x00);
        LCD_IO_WriteData!(self.spi_dev, 0x78);

        LCD_IO_WriteReg!(self.spi_dev, LCD_POWERA);
        LCD_IO_WriteData!(self.spi_dev, 0x39);
        LCD_IO_WriteData!(self.spi_dev, 0x2C);
        LCD_IO_WriteData!(self.spi_dev, 0x00);
        LCD_IO_WriteData!(self.spi_dev, 0x34);
        LCD_IO_WriteData!(self.spi_dev, 0x02);

        LCD_IO_WriteReg!(self.spi_dev, LCD_PRC);
        LCD_IO_WriteData!(self.spi_dev, 0x20);

        LCD_IO_WriteReg!(self.spi_dev, LCD_DTCB);
        LCD_IO_WriteData!(self.spi_dev, 0x00);
        LCD_IO_WriteData!(self.spi_dev, 0x00);

        LCD_IO_WriteReg!(self.spi_dev, LCD_FRMCTR1);
        LCD_IO_WriteData!(self.spi_dev, 0x00);
        LCD_IO_WriteData!(self.spi_dev, 0x1B);

        LCD_IO_WriteReg!(self.spi_dev, LCD_DFC);
        LCD_IO_WriteData!(self.spi_dev, 0x0A);
        LCD_IO_WriteData!(self.spi_dev, 0xA2);

        LCD_IO_WriteReg!(self.spi_dev, LCD_POWER1);
        LCD_IO_WriteData!(self.spi_dev, 0x10);

        LCD_IO_WriteReg!(self.spi_dev, LCD_POWER2);
        LCD_IO_WriteData!(self.spi_dev, 0x10);

        LCD_IO_WriteReg!(self.spi_dev, LCD_VCOM1);
        LCD_IO_WriteData!(self.spi_dev, 0x45);
        LCD_IO_WriteData!(self.spi_dev, 0x15);

        LCD_IO_WriteReg!(self.spi_dev, LCD_VCOM2);
        LCD_IO_WriteData!(self.spi_dev, 0x90);

        LCD_IO_WriteReg!(self.spi_dev, LCD_MAC);
        LCD_IO_WriteData!(self.spi_dev, 0xC8);

        LCD_IO_WriteReg!(self.spi_dev, LCD_3GAMMA_EN);
        LCD_IO_WriteData!(self.spi_dev, 0x00);

        LCD_IO_WriteReg!(self.spi_dev, LCD_RGB_INTERFACE);
        LCD_IO_WriteData!(self.spi_dev, 0xC2);

        LCD_IO_WriteReg!(self.spi_dev, LCD_DFC);
        LCD_IO_WriteData!(self.spi_dev, 0x0A);
        LCD_IO_WriteData!(self.spi_dev, 0xA7);
        LCD_IO_WriteData!(self.spi_dev, 0x27);
        LCD_IO_WriteData!(self.spi_dev, 0x04);

        LCD_IO_WriteReg!(self.spi_dev, LCD_COLUMN_ADDR);
        LCD_IO_WriteData!(self.spi_dev, 0x00);
        LCD_IO_WriteData!(self.spi_dev, 0x00);
        LCD_IO_WriteData!(self.spi_dev, 0x00);
        LCD_IO_WriteData!(self.spi_dev, 0xEF);

        LCD_IO_WriteReg!(self.spi_dev, LCD_PAGE_ADDR);
        LCD_IO_WriteData!(self.spi_dev, 0x00);
        LCD_IO_WriteData!(self.spi_dev, 0x00);
        LCD_IO_WriteData!(self.spi_dev, 0x01);
        LCD_IO_WriteData!(self.spi_dev, 0x3F);

        LCD_IO_WriteReg!(self.spi_dev, LCD_INTERFACE);
        LCD_IO_WriteData!(self.spi_dev, 0x01);
        LCD_IO_WriteData!(self.spi_dev, 0x00);
        LCD_IO_WriteData!(self.spi_dev, 0x06);

        LCD_IO_WriteReg!(self.spi_dev, LCD_GRAM);
        delay.delay_us(200_000);

        LCD_IO_WriteReg!(self.spi_dev, LCD_GAMMA);
        LCD_IO_WriteData!(self.spi_dev, 0x01);

        LCD_IO_WriteReg!(self.spi_dev, LCD_PGAMMA);
        LCD_IO_WriteData!(self.spi_dev, 0x0F);
        LCD_IO_WriteData!(self.spi_dev, 0x29);
        LCD_IO_WriteData!(self.spi_dev, 0x24);
        LCD_IO_WriteData!(self.spi_dev, 0x0C);
        LCD_IO_WriteData!(self.spi_dev, 0x0E);
        LCD_IO_WriteData!(self.spi_dev, 0x09);
        LCD_IO_WriteData!(self.spi_dev, 0x4E);
        LCD_IO_WriteData!(self.spi_dev, 0x78);
        LCD_IO_WriteData!(self.spi_dev, 0x3C);
        LCD_IO_WriteData!(self.spi_dev, 0x09);
        LCD_IO_WriteData!(self.spi_dev, 0x13);
        LCD_IO_WriteData!(self.spi_dev, 0x05);
        LCD_IO_WriteData!(self.spi_dev, 0x17);
        LCD_IO_WriteData!(self.spi_dev, 0x11);
        LCD_IO_WriteData!(self.spi_dev, 0x00);

        LCD_IO_WriteReg!(self.spi_dev, LCD_NGAMMA);
        LCD_IO_WriteData!(self.spi_dev, 0x00);
        LCD_IO_WriteData!(self.spi_dev, 0x16);
        LCD_IO_WriteData!(self.spi_dev, 0x1B);
        LCD_IO_WriteData!(self.spi_dev, 0x04);
        LCD_IO_WriteData!(self.spi_dev, 0x11);
        LCD_IO_WriteData!(self.spi_dev, 0x07);
        LCD_IO_WriteData!(self.spi_dev, 0x31);
        LCD_IO_WriteData!(self.spi_dev, 0x33);
        LCD_IO_WriteData!(self.spi_dev, 0x42);
        LCD_IO_WriteData!(self.spi_dev, 0x05);
        LCD_IO_WriteData!(self.spi_dev, 0x0C);
        LCD_IO_WriteData!(self.spi_dev, 0x0A);
        LCD_IO_WriteData!(self.spi_dev, 0x28);
        LCD_IO_WriteData!(self.spi_dev, 0x2F);
        LCD_IO_WriteData!(self.spi_dev, 0x0F);

        LCD_IO_WriteReg!(self.spi_dev, LCD_SLEEP_OUT);
        delay.delay_us(200_000);

        LCD_IO_WriteReg!(self.spi_dev, LCD_DISPLAY_ON);
        // GRAM start writing
        LCD_IO_WriteReg!(self.spi_dev, LCD_GRAM);

        // Layer Config
        // Taken hard coded

        // Horizontal start
        ltd_dev
            .layer1
            .whpcr
            .modify(|_, w| w.whsppos().bits(0x010d).whstpos().bits(0x1e));

        // Vertical start
        ltd_dev
            .layer1
            .wvpcr
            .modify(|_, w| w.wvsppos().bits(0x0143).wvstpos().bits(0x04));

        // Pixel format
        ltd_dev.layer1.pfcr.modify(|_, w| w.pf().bits(0x02));

        // Default colours
        ltd_dev.layer1.dccr.modify(|_, w| {
            w.dcalpha()
                .bits(0x00)
                .dcred()
                .bits(0x00)
                .dcgreen()
                .bits(0x00)
                .dcblue()
                .bits(0x00)
        });

        // Specifies the constant alpha value
        ltd_dev.layer1.cacr.modify(|_, w| w.consta().bits(0xff));

        // Specifies the blending factors
        ltd_dev
            .layer1
            .bfcr
            .modify(|_, w| unsafe { w.bf1().bits(0x06).bf2().bits(0x07) });

        // Configure the color frame buffer start address
        ltd_dev
            .layer1
            .cfbar
            .modify(|_, w| w.cfbadd().bits(0xD000_0000));

        // Configure the color frame buffer pitch in byte
        ltd_dev
            .layer1
            .cfblr
            .modify(|_, w| w.cfbll().bits(0x01e3).cfbp().bits(0x01e0));

        // Configure the frame buffer line number
        ltd_dev
            .layer1
            .cfblnr
            .modify(|_, w| w.cfblnbr().bits(0x0140));

        // Enable LTDC_Layer by setting LEN bit
        ltd_dev.layer1.cr.modify(|_, w| w.len().enabled());

        // Set the Immediate Reload
        ltd_dev.srcr.modify(|_, w| w.imr().set_bit());

        // Enable dither
        ltd_dev.gcr.modify(|_, w| w.den().set_bit());

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
        };
    });
}

#[interrupt]
fn DMA2D() {
    cortex_m::interrupt::free(|_| {});
}
