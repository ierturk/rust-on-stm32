#![no_std]
#![no_main]

extern crate alloc;

// use defmt::*;
use {defmt_rtt as _, panic_probe as _};

mod drivers;

const PERIOD: lilos::time::Millis = lilos::time::Millis(500);
const SYSCLK: u32 = 168_000_000;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();

    let stm_backend_inner = drivers::bsp::StmBackendInner::default();

    let led_task = core::pin::pin!(async {
        let mut gate = lilos::time::PeriodicGate::from(PERIOD);
        let gpio_dev = unsafe { &*stm32f4xx_hal::pac::GPIOG::ptr() };

        loop {
            gpio_dev.bsrr.write(|w| w.bs13().set_bit());
            gpio_dev.bsrr.write(|w| w.br14().set_bit());
            gate.next_time().await;
            gpio_dev.bsrr.write(|w| w.br13().set_bit());
            gpio_dev.bsrr.write(|w| w.bs14().set_bit());
            gate.next_time().await;
        }
    });

    let gui_task = core::pin::pin!(async {
        drivers::bsp::slint_async_run_event_loop(stm_backend_inner).await;
        loop {}
    });

    lilos::time::initialize_sys_tick(&mut cp.SYST, SYSCLK);

    unsafe {
        lilos::exec::run_tasks_with_preemption(
            &mut [led_task, gui_task],
            lilos::exec::ALL_TASKS,
            lilos::exec::Interrupts::Filtered(7),
        );
    }
}
