#![no_std]
#![no_main]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::pixelcolor::RgbColor;
use hal::pac::Interrupt;
use hal::pac::Peripherals as device;
use hal::pac::NVIC;
use hal::prelude::*;
use stm32f4xx_hal as hal;

mod drivers;
use drivers::display::LtdcDisplay;
use drivers::fmc::Sdram;
use drivers::ltdc::Ltdc;

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

use hal::interrupt;

const PERIOD: lilos::time::Millis = lilos::time::Millis(1000);

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
        .require_pll48clk()
        .freeze();

    let rcc_r = unsafe { &*stm32f4xx_hal::pac::RCC::ptr() };
    info!("pllm: {}", rcc_r.pllcfgr.read().pllm().bits());
    info!("plln: {}", rcc_r.pllcfgr.read().plln().bits());
    info!("pllp: {}", 2 * (rcc_r.pllcfgr.read().pllp().bits() + 1));
    info!("pllsrc: {}", rcc_r.pllcfgr.read().pllsrc().bit());
    info!("pllq: {}", rcc_r.pllcfgr.read().pllq().bits());

    let mut delay = dp.TIM1.delay_us(&clocks);

    // Init and test FMC/SDRAM that start at 0xD000_0000
    let sdram_ptr = Sdram::new(&mut delay);
    let sdram_size = 8 * 1024 * 1024; // 8MiB
    let sdram = unsafe {
        core::slice::from_raw_parts_mut(sdram_ptr, sdram_size / core::mem::size_of::<u16>())
    };
    /*
       for n in 0..ram_slice.len() {
           ram_slice[n] = n as u16;
       }

       for n in 0..ram_slice.len() {
           crate::assert_eq!(ram_slice[n], n as u16);
       }
    */
    info!("FMC/SDRAM module seems to be functional!");
    sdram.fill(0x00);

    for n in 0..240 {
        sdram[n] = 0x1f;
        sdram[n + 10 * 240] = 0x1f;
        sdram[n + 20 * 240] = 0x3f << 5;
        sdram[n + 30 * 240] = 0x1f << 11;
    }

    for n in 0..320 {
        sdram[n * 240 + 120] = 0x1f | (0x3f << 5);
    }

    // Init LCD/LTDC Display
    let _ltdc_ok = Ltdc::new(&mut delay);
    delay.release();

    let mut display = LtdcDisplay::new(sdram_ptr, sdram_size);

    let _ = Circle::new(Point::new(100, 100), 40)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(&mut display);

    let style = MonoTextStyle::new(&FONT_9X18, Rgb565::RED);
    let _ = Text::new("Hello Rust!", Point::new(100, 300), style).draw(&mut display);

    // Idle async app for heartbeat
    dp.GPIOG
        .moder
        .modify(|_, w| w.moder13().output().moder14().output());

    let blink = core::pin::pin!(async {
        let mut gate = lilos::time::PeriodicGate::from(PERIOD);

        loop {
            dp.GPIOG.bsrr.write(|w| w.bs13().set_bit());
            gate.next_time().await;
            dp.GPIOG.bsrr.write(|w| w.br13().set_bit());
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
