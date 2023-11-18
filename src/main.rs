#![no_std]
#![no_main]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use hal::pac::Peripherals as device;
use hal::prelude::*;
use stm32f4xx_hal as hal;

mod ts_disp;
use ts_disp::TsDisp;

const PERIOD: lilos::time::Millis = lilos::time::Millis(1000);

#[cortex_m_rt::entry]
fn main() -> ! {
    info!("Program start");

    let _ts_disp = TsDisp::new();

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
    let mut disp = TsDisp::new();
    let mut _disp = disp.sdram.init(&mut delay);
    delay.release();

    let sdram_size = 8 * 1024 * 1024;
    let sdram_ptr = 0xD000_0000 as *mut u16;
    let ram_slice = unsafe {
        core::slice::from_raw_parts_mut(sdram_ptr, sdram_size / core::mem::size_of::<u16>())
    };

    info!("RAM contents before writing: {:x}", ram_slice[..12]);

    ram_slice[0] = 1;
    ram_slice[1] = 2;
    ram_slice[2] = 3;
    ram_slice[3] = 4;
    ram_slice[4] = 5;
    ram_slice[5] = 6;
    ram_slice[6] = 7;
    ram_slice[7] = 8;
    ram_slice[8] = 9;
    ram_slice[9] = 10;

    info!("RAM contents after writing: {:x}", ram_slice[..12]);

    crate::assert_eq!(ram_slice[0], 1);
    crate::assert_eq!(ram_slice[1], 2);
    crate::assert_eq!(ram_slice[2], 3);
    crate::assert_eq!(ram_slice[3], 4);
    crate::assert_eq!(ram_slice[4], 5);
    crate::assert_eq!(ram_slice[5], 6);
    crate::assert_eq!(ram_slice[6], 7);
    crate::assert_eq!(ram_slice[7], 8);
    crate::assert_eq!(ram_slice[8], 9);
    crate::assert_eq!(ram_slice[9], 10);

    info!("FMC/SDRAM module seems to be functional!");

    dp.GPIOG
        .moder
        .modify(|_, w| w.moder13().output().moder14().output());

    let blink = core::pin::pin!(async {
        let mut gate = lilos::time::PeriodicGate::from(PERIOD);

        loop {
            dp.GPIOG.bsrr.write(|w| w.bs13().set_bit());
            dp.GPIOG.bsrr.write(|w| w.br14().set_bit());
            gate.next_time().await;
            dp.GPIOG.bsrr.write(|w| w.br13().set_bit());
            dp.GPIOG.bsrr.write(|w| w.bs14().set_bit());
            gate.next_time().await;
        }
    });

    lilos::time::initialize_sys_tick(&mut cp.SYST, 168_000_000);
    lilos::exec::run_tasks(&mut [blink], lilos::exec::ALL_TASKS)
}
