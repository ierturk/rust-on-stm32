#![no_std]
#![no_main]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::pixelcolor::RgbColor;
use hal::gpio::gpiob;
use hal::i2c::Mode;
use hal::pac::Interrupt;
use hal::pac::Peripherals as device;
use hal::pac::NVIC;
use hal::prelude::*;
use stm32f4xx_hal as hal;

mod drivers;
use drivers::display::LtdcDisplay;
use drivers::fmc::Sdram;
use drivers::ltdc::Ltdc;
use drivers::touchscreen::TouchScreen;

use embedded_graphics::primitives::Primitive;
use embedded_graphics::{
    mono_font::{ascii::FONT_9X18, MonoTextStyle},
    text::Text,
};
use embedded_graphics::{
    prelude::Point,
    primitives::{Circle, PrimitiveStyle},
};
use embedded_graphics_core::Drawable;

use hal::gpio::GpioExt;
use hal::gpio::Speed;

use stm32f4xx_hal::{
    prelude::*,
    spi::{Mode as SpiMode, Phase, Polarity},
};

use hal::interrupt;

use embedded_graphics::geometry::Size;
use embedded_graphics::mono_font::ascii;
use embedded_graphics::prelude::WebColors;
use embedded_graphics::primitives::StyledDrawable;

use kolibri_embedded_gui::button::Button;
use kolibri_embedded_gui::checkbox::Checkbox;
use kolibri_embedded_gui::icon::IconWidget;
use kolibri_embedded_gui::iconbutton::IconButton;
use kolibri_embedded_gui::icons::{size12px, size24px, size32px};
use kolibri_embedded_gui::label::Label;
use kolibri_embedded_gui::prelude::*;
use kolibri_embedded_gui::smartstate::{Smartstate, SmartstateProvider};
use kolibri_embedded_gui::spacer::Spacer;
use kolibri_embedded_gui::style::{medsize_rgb565_debug_style, medsize_rgb565_style};
use kolibri_embedded_gui::ui::{Interaction, Ui};

const PERIOD: lilos::time::Millis = lilos::time::Millis(1000);

macro_rules! fmc_pins {
    ($($pin:expr),*) => {
        (
            $(
                $pin.into_push_pull_output()
                    .speed(Speed::VeryHigh)
                    .into_alternate::<12>()
                    .internal_pull_up(false)
            ),*
        )
    };
}

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

#[cortex_m_rt::entry]
fn main() -> ! {
    info!("Program start");

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = device::take().unwrap();

    // Debug probe fix for RTT
    dp.RCC.apb2enr.write(|w| w.syscfgen().enabled());
    dp.DBGMCU.cr.modify(|_, w| {
        w.dbg_sleep().set_bit();
        w.dbg_standby().set_bit();
        w.dbg_stop().set_bit()
    });
    dp.RCC.ahb1enr.modify(|_, w| w.dma1en().enabled());
    dp.RCC.ahb1enr.modify(|_, w| w.gpiogen().enabled());

    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(168.MHz())
        .hclk(168.MHz())
        .pclk1(42.MHz())
        .pclk2(84.MHz())
        // .require_pll48clk()
        .freeze();

    let rcc_r = unsafe { &*stm32f4xx_hal::pac::RCC::ptr() };
    info!("pllm: {}", rcc_r.pllcfgr.read().pllm().bits());
    info!("plln: {}", rcc_r.pllcfgr.read().plln().bits());
    info!("pllp: {}", 2 * (rcc_r.pllcfgr.read().pllp().bits() + 1));
    info!("pllsrc: {}", rcc_r.pllcfgr.read().pllsrc().bit());
    info!("pllq: {}", rcc_r.pllcfgr.read().pllq().bits());

    let mut delay = dp.TIM1.delay_us(&clocks);

    let gpio_a = dp.GPIOA.split();
    let gpio_b = dp.GPIOB.split();
    let gpio_c = dp.GPIOC.split();
    let gpio_d = dp.GPIOD.split();
    let gpio_e = dp.GPIOE.split();
    let gpio_f = dp.GPIOF.split();
    let gpio_g = dp.GPIOG.split();

    // Init and test FMC/SDRAM that start at 0xD000_0000
    // GPIO Config
    #[rustfmt::skip]
    let _ = fmc_pins!(
        // A0-A11
        gpio_f.pf0,
        gpio_f.pf1,
        gpio_f.pf2,
        gpio_f.pf3,
        gpio_f.pf4,
        gpio_f.pf5,
        gpio_f.pf12,
        gpio_f.pf13,
        gpio_f.pf14,
        gpio_f.pf15,
        gpio_g.pg0,
        gpio_g.pg1,
        // BA0-BA1
        gpio_g.pg4,
        gpio_g.pg5,
        // D0-D15
        gpio_d.pd14,
        gpio_d.pd15,
        gpio_d.pd0,
        gpio_d.pd1,
        gpio_e.pe7,
        gpio_e.pe8,
        gpio_e.pe9,
        gpio_e.pe10,
        gpio_e.pe11,
        gpio_e.pe12,
        gpio_e.pe13,
        gpio_e.pe14,
        gpio_e.pe15,
        gpio_d.pd8,
        gpio_d.pd9,
        gpio_d.pd10,
        // NBL0 - NBL1
        gpio_e.pe0,
        gpio_e.pe1,
        // SDCKE1
        gpio_b.pb5,
        // SDCLK
        gpio_g.pg8,
        // SDNCAS
        gpio_g.pg15,
        // SDNE1
        gpio_b.pb6,
        // SDNRAS
        gpio_f.pf11,
        // SDNWE
        gpio_c.pc0
    );

    let sdram_ptr = Sdram::new(&mut delay);

    let sdram_size = 8 * 1024 * 1024; // 8MiB
    let sdram = unsafe {
        core::slice::from_raw_parts_mut(sdram_ptr, sdram_size / core::mem::size_of::<u16>())
    };

    /*
        // SDRAM Test
       sdram.fill(0x0000);

       for n in 0..240 {
           sdram[n] = 0x1f;
           sdram[n + 10 * 240] = 0x1f;
           sdram[n + 20 * 240] = 0x3f << 5;
           sdram[n + 30 * 240] = 0x1f << 11;
       }

       for n in 0..320 {
           sdram[n * 240 + 120] = 0x1f | (0x3f << 5);
       }
    */

    // Init LCD/LTDC Display
    // GPIO Config
    #[rustfmt::skip]
    let _ = ltdc_pins!(
        // R2-R7
        gpio_c.pc10,
        gpio_b.pb0,
        gpio_a.pa11,
        gpio_a.pa12,
        gpio_b.pb1,
        gpio_g.pg6,
        // G2-G7
        gpio_a.pa6,
        gpio_g.pg10,
        gpio_b.pb10,
        gpio_b.pb11,
        gpio_c.pc7,
        gpio_d.pd3,
        // B2-B7
        gpio_d.pd6,
        gpio_g.pg11,
        gpio_g.pg12,
        gpio_a.pa3,
        gpio_b.pb8,
        gpio_b.pb9,
        // LCD_TFT HSYNC
        gpio_c.pc6,
        // LCDTFT VSYNC
        gpio_a.pa4,
        // LCD_TFT CLK
        gpio_g.pg7,
        // LCD_TFT DE
        gpio_f.pf10
    );

    // SPI Config
    // ILI931 LCD IO init
    // GPIO init
    // GPIOD is already enabled
    // LCD RDX PD12
    // LCD WRX PD 13
    // LCD NCS PC2

    // LCD SPI init
    /* SPI baudrate is set to 5.6 MHz (PCLK2/SPI_BaudRatePrescaler = 84/16 = 5.25 MHz)
       to verify these constraints:
       - ILI9341 LCD SPI interface max baudrate is 10MHz for write and 6.66MHz for read
       - l3gd20 SPI interface max baudrate is 10MHz for write/read
       - PCLK2 frequency is set to 84 MHz
    */
    // GPIO SPI5 CLK PF7, SPI5 MISO PF8, SPI5 MOSI PF9

    let mut lcd_rdx = gpio_d.pd12.into_push_pull_output().speed(Speed::VeryHigh);
    let mut lcd_wrx = gpio_d.pd13.into_push_pull_output().speed(Speed::VeryHigh);
    let mut lcd_ncs = gpio_c.pc2.into_push_pull_output().speed(Speed::VeryHigh);

    lcd_rdx.set_high();
    lcd_wrx.set_high();
    lcd_ncs.set_high();

    let lcd_clk = gpio_f
        .pf7
        .into_push_pull_output()
        .speed(Speed::VeryHigh)
        .into_alternate::<5>()
        .internal_pull_up(false);

    let lcd_miso = gpio_f
        .pf8
        .into_push_pull_output()
        .speed(Speed::VeryHigh)
        .into_alternate::<5>()
        .internal_pull_up(false);

    let lcd_mosi = gpio_f
        .pf9
        .into_push_pull_output()
        .speed(Speed::VeryHigh)
        .into_alternate::<5>()
        .internal_pull_up(false);

    let lcd_mode = SpiMode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let lcd_spi = dp
        .SPI5
        .spi((lcd_clk, lcd_miso, lcd_mosi), lcd_mode, 2.MHz(), &clocks);

    let mut ltdc_dev = Ltdc { spi_dev: lcd_spi };
    let _ = Ltdc::new(&mut ltdc_dev, &mut delay);

    let mut display = LtdcDisplay::new(sdram_ptr, 320, 240);

    /*
       // Display Test
       let _ = Circle::new(Point::new(100, 20), 240)
           .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
           .draw(&mut display);

       let style = MonoTextStyle::new(&FONT_9X18, Rgb565::RED);
       let _ = Text::new("Hello Rust!", Point::new(160, 120), style).draw(&mut display);
    */

    // kolibri test
    // counter for incrementing thingy
    let mut i = 0u8;

    let mut ui = Ui::new_fullscreen(&mut display, medsize_rgb565_style());
    ui.clear_background().unwrap();

    ui.add(Label::new("Basic Example").with_font(ascii::FONT_10X20));

    ui.add(Label::new("Basic Counter (7LOC)"));

    if ui.add_horizontal(Button::new("-")).clicked() {
        i = i.saturating_sub(1);
    }

    let mut buf = [0u8; 64];
    let s: &str = format_no_std::show(&mut buf, format_args!("Clicked {} times", i)).unwrap();

    ui.add_horizontal(Label::new(s.as_ref()));
    if ui.add_horizontal(Button::new("+")).clicked() {
        i = i.saturating_add(1);
    }

    // Init Touch Screen
    let i2c3_scl = gpio_a.pa8.into_alternate_open_drain::<4>();
    let i2c3_sda = gpio_c.pc9.into_alternate_open_drain::<4>();

    let i2c3_dev = dp.I2C3.i2c(
        (i2c3_scl, i2c3_sda),
        Mode::Standard {
            frequency: 100.kHz(),
        },
        &clocks,
    );

    let mut ts_dev = TouchScreen { i2c_dev: i2c3_dev };
    let _ = TouchScreen::new(&mut ts_dev, &mut delay);

    delay.release();

    // Idle async app for heartbeat
    let mut led_green = gpio_g.pg13.into_push_pull_output();
    let _led_red = gpio_g.pg14.into_push_pull_output();

    let blink = core::pin::pin!(async {
        let mut gate = lilos::time::PeriodicGate::from(PERIOD);

        loop {
            led_green.set_high();
            gate.next_time().await;
            led_green.set_low();
            gate.next_time().await;
        }
    });

    let tim6 = dp.TIM6;
    let rcc = unsafe { &*stm32f4xx_hal::pac::RCC::ptr() };
    rcc.apb1enr.write(|w| w.tim6en().set_bit());
    cortex_m::interrupt::free(|_| {
        tim6.cr1.write(|w| w.cen().clear_bit());
        tim6.psc.write(|w| w.psc().bits(21000));
        tim6.arr.write(|w| w.arr().bits(4000));
        tim6.egr.write(|w| w.ug().set_bit());
        tim6.dier.write(|w| w.uie().set_bit());
        tim6.cr1.write(|w| w.cen().set_bit());
    });

    unsafe {
        NVIC::unmask(Interrupt::TIM6_DAC);
    };

    lilos::time::initialize_sys_tick(&mut cp.SYST, 168_000_000);
    lilos::exec::run_tasks(&mut [blink], lilos::exec::ALL_TASKS)
}

// Timer Interrupt
#[interrupt]
fn TIM6_DAC() {
    cortex_m::interrupt::free(|_| {
        let gpiog = unsafe { &*stm32f4xx_hal::pac::GPIOG::ptr() };

        if gpiog.odr.read().odr14().bit_is_clear() {
            gpiog.odr.modify(|_, w| w.odr14().set_bit());
        } else {
            gpiog.odr.modify(|_, w| w.odr14().clear_bit());
        }

        let tim6 = unsafe { &*stm32f4xx_hal::pac::TIM6::ptr() };
        tim6.sr.write(|w| w.uif().clear_bit());
    });
}
