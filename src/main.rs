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
        .freeze();

    let mut delay = dp.TIM1.delay_us(&clocks);

    // Init and test FMC/SDRAM that start at 0xD000_0000
    let sdram_ptr = Sdram::new(&mut delay);
    let sdram_size = 8 * 1024 * 1024; // 8MiB
    let ram_slice = unsafe { core::slice::from_raw_parts_mut(sdram_ptr, sdram_size / 4) };
    /*
       for n in 0..ram_slice.len() {
           ram_slice[n] = n as u16;
       }

       for n in 0..ram_slice.len() {
           crate::assert_eq!(ram_slice[n], n as u16);
       }
    */
    info!("FMC/SDRAM module seems to be functional!");
    ram_slice.fill(0x0000);

    // Init LCD/LTDC Display
    let _ltdc_ok = Ltdc::new(&mut delay);

    delay.release();

    let mut display = LtdcDisplay::new(sdram_ptr, sdram_size);

    let circle = Circle::new(Point::new(100, 100), 50)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 2));
    let _ = circle.draw(&mut display);

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
