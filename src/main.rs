#![no_std]
#![no_main]

extern crate alloc;

// use defmt::*;
use {defmt_rtt as _, panic_probe as _};

mod drivers;
use alloc::boxed::Box;
use drivers::bsp::*;

slint::include_modules!();

fn create_slint_app() -> AppWindow {
    let ui = AppWindow::new().expect("Failed to load UI");

    let ui_handle = ui.as_weak();
    ui.on_request_increase_value(move || {
        let ui = ui_handle.unwrap();
        ui.set_counter(ui.get_counter() + 1);
    });
    ui
}

#[cortex_m_rt::entry]
fn main() -> ! {
    slint::platform::set_platform(Box::new(StmBackend::default()))
        .expect("backend already initialized");

    create_slint_app().run().unwrap();

    defmt::panic!("The MCU demo should not quit")
}
