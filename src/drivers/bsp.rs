extern crate alloc;

use alloc::rc::Rc;
use core::cell::RefCell;
use embedded_alloc::Heap;

// use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use hal::i2c::Mode;
use hal::pac::Peripherals as device;
use stm32f4xx_hal as hal;

// use super::display::LtdcDisplay;
use super::fmc::Sdram;
use super::ltdc::Ltdc;
use super::touchscreen::TouchScreen;

use slint::platform::software_renderer;

use hal::gpio::GpioExt;
use hal::gpio::Speed;

use stm32f4xx_hal::{
    prelude::*,
    spi::{Mode as SpiMode, Phase, Polarity},
};

const HEAP_SIZE: usize = 200 * 1024;
#[link_section = ".embedded_alloc_heap"]
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

const DISPLAY_WIDTH: usize = 240;
const DISPLAY_HEIGHT: usize = 320;

pub type TargetPixel = software_renderer::Rgb565Pixel;

#[link_section = ".frame_buffer"]
static mut FB1: [TargetPixel; DISPLAY_WIDTH * DISPLAY_HEIGHT] =
    [software_renderer::Rgb565Pixel(0); DISPLAY_WIDTH * DISPLAY_HEIGHT];
#[link_section = ".frame_buffer"]
static mut FB2: [TargetPixel; DISPLAY_WIDTH * DISPLAY_HEIGHT] =
    [software_renderer::Rgb565Pixel(0); DISPLAY_WIDTH * DISPLAY_HEIGHT];

macro_rules! fmc_pins {
    ($($pin:expr),*) => {
        (
            $(
                $pin.into_push_pull_output()
                    .speed(Speed::VeryHigh)
                    .into_alternate::<12>()
                    .internal_pull_up(true)
            ),*
        )
    };
}

macro_rules! ltdc_pins_af14 {
    ($($pin:expr),*) => {
        (
            $(
                $pin.into_push_pull_output()
                    .speed(Speed::VeryHigh)
                    .into_alternate::<14>()
                    .internal_pull_up(true)
            ),*
        )
    };
}

macro_rules! ltdc_pins_af9 {
    ($($pin:expr),*) => {
        (
            $(
                $pin.into_push_pull_output()
                    .speed(Speed::VeryHigh)
                    .into_alternate::<9>()
                    .internal_pull_up(true)
            ),*
        )
    };
}

struct StmBackendInner {
    scb: cortex_m::peripheral::SCB,
    delay: stm32f4xx_hal::timer::delay::SysDelay,
    // layer: stm32f4xx_hal::ltdc::LtdcLayer1,
    // ts_i2c: stm32f4xx_hal::i2c::I2c<stm32f4xx_hal::i2c::I2c3>,
}

pub struct StmBackend {
    window: RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
    // timer: hal::timer::Timer<stm32f4xx_hal::pac::TIM2>,
    inner: RefCell<StmBackendInner>,
}

impl Default for StmBackend {
    fn default() -> Self {
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

        let mut delay = cp.SYST.delay(&clocks);

        cp.SCB.invalidate_icache();
        cp.SCB.enable_icache();
        cp.SCB.enable_dcache(&mut cp.CPUID);
        cp.DWT.enable_cycle_counter();

        let gpio_a = dp.GPIOA.split();
        let gpio_b = dp.GPIOB.split();
        let gpio_c = dp.GPIOC.split();
        let gpio_d = dp.GPIOD.split();
        let gpio_e = dp.GPIOE.split();
        let gpio_f = dp.GPIOF.split();
        let gpio_g = dp.GPIOG.split();

        // Init FMC/SDRAM that start at 0xD000_0000
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

        let _ = Sdram::new(&mut delay);

        unsafe {
            ALLOCATOR.init(
                &mut HEAP as *const u8 as usize,
                core::mem::size_of_val(&HEAP),
            )
        }

        // Init LCD/LTDC Display
        // GPIO Config
        #[rustfmt::skip]
        let _ = ltdc_pins_af14!(
            // R2-R7
            gpio_c.pc10,
            // gpio_b.pb0,
            gpio_a.pa11,
            gpio_a.pa12,
            // gpio_b.pb1,
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

        #[rustfmt::skip]
        let _ = ltdc_pins_af9!(
            gpio_b.pb0, // R3
            gpio_b.pb1  // R6
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

        let (fb1, _fb2) = unsafe { (&mut FB1, &mut FB2) };

        let mut ltdc_dev = Ltdc { spi_dev: lcd_spi };
        let _ = Ltdc::new(&mut ltdc_dev, fb1.as_ptr() as *const u16, &mut delay);

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

        // Init Timer
        // let timer = stm32f4xx_hal::timer::Timer::new(dp.TIM2, &clocks);
        // timer.listen(hal::timer::Event::Update);

        // Init RNG

        Self {
            window: RefCell::default(),
            // timer,
            inner: RefCell::new(StmBackendInner {
                scb: cp.SCB,
                delay,
                // layer,
                // touch_i2c,
            }),
        }
    }
}

impl slint::platform::Platform for StmBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
            slint::platform::software_renderer::RepaintBufferType::SwappedBuffers,
        );
        self.window.replace(Some(window.clone()));
        Ok(window)
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        let _inner = &mut *self.inner.borrow_mut();

        // Init TouchScreen

        // Safety: The Refcell at the beginning of `run_event_loop` prevents re-entrancy and thus multiple mutable references to FB1/FB2.
        let (fb1, fb2) = unsafe { (&mut FB1, &mut FB2) };

        let displayed_fb: &mut [TargetPixel] = fb1;
        let _work_fb: &mut [TargetPixel] = fb2;

        // let mut last_touch = None;

        self.window
            .borrow()
            .as_ref()
            .unwrap()
            .set_size(slint::PhysicalSize::new(
                DISPLAY_WIDTH as u32,
                DISPLAY_HEIGHT as u32,
            ));
        loop {
            slint::platform::update_timers_and_animations();

            if let Some(window) = self.window.borrow().clone() {
                window.draw_if_needed(|renderer| {
                    renderer.render(displayed_fb, DISPLAY_WIDTH);
                });

                // handle touch event
            }

            // FIXME: cortex_m::asm::wfe();
        }
    }

    fn duration_since_start(&self) -> core::time::Duration {
        // FIXME! the timer can overflow
        // let val = self.timer.counter() / 10;
        core::time::Duration::from_millis(100)
    }

    fn debug_log(&self, arguments: core::fmt::Arguments) {
        use alloc::string::ToString;
        defmt::println!("{=str}", arguments.to_string());
    }
}
