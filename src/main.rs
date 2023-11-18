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
    let _clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(168.MHz())
        .hclk(168.MHz())
        .pclk1(42.MHz())
        .pclk2(84.MHz())
        .freeze();

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
