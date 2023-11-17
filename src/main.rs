#![no_std]
#![no_main]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use core::convert::Infallible;
use core::pin::pin;
use core::time::Duration;
use lilos::time::sleep_for;

use stm32f4::stm32f429 as device;

#[cortex_m_rt::entry]
fn main() -> ! {
    info!("Program start");

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = device::Peripherals::take().unwrap();

    // Debug probe fix
    dp.RCC.apb2enr.write(|w| w.syscfgen().enabled());
    dp.DBGMCU.cr.modify(|_, w| {
        w.dbg_sleep().set_bit();
        w.dbg_standby().set_bit();
        w.dbg_stop().set_bit()
    });
    dp.RCC.ahb1enr.modify(|_, w| w.dma1en().enabled());

    // Enable clock to GPIOD.
    dp.RCC.ahb1enr.modify(|_, w| w.gpiogen().enabled());
    // Set pins to outputs.
    dp.GPIOG
        .moder
        .modify(|_, w| w.moder13().output().moder14().output());

    let fut1 = pin!(blinky(1 << 13, Duration::from_millis(800), &dp.GPIOG));
    let fut2 = pin!(blinky(1 << 14, Duration::from_millis(400), &dp.GPIOG));

    lilos::time::initialize_sys_tick(&mut cp.SYST, 16_000_000);

    lilos::exec::run_tasks(&mut [fut1, fut2], lilos::exec::ALL_TASKS)
}

async fn blinky(pin_mask: u16, interval: Duration, gpiog: &device::GPIOG) -> Infallible {
    let pin_mask = u32::from(pin_mask);

    loop {
        // on
        gpiog.bsrr.write(|w| unsafe { w.bits(pin_mask) });
        sleep_for(interval).await;
        // off (same bits set in top 16 bits)
        gpiog.bsrr.write(|w| unsafe { w.bits(pin_mask << 16) });
        sleep_for(interval).await;
    }
}
