#![no_std]
#![no_main]

extern crate alloc;

// use defmt::*;
use {defmt_rtt as _, panic_probe as _};

mod drivers;
use alloc::boxed::Box;
use drivers::bsp::*;

slint::include_modules!();

#[cortex_m_rt::entry]
fn main() -> ! {
    slint::platform::set_platform(Box::new(StmBackend::default()))
        .expect("backend already initialized");

    MainWindow::new().unwrap().run().unwrap();

    defmt::panic!("The MCU demo should not quit")
}
