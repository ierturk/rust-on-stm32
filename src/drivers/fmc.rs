use hal::gpio::GpioExt;
use hal::gpio::Speed;
use hal::interrupt;
use hal::pac::Interrupt;
use hal::pac::Peripherals as device;
use hal::pac::NVIC;
use stm32f4xx_hal as hal;

use embedded_hal::blocking::delay::DelayUs;

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

pub struct Sdram {}
impl Sdram {
    pub fn new<D>(delay: &mut D) -> *mut u16
    where
        D: DelayUs<u8>,
    {
        let dp = unsafe { device::steal() };
        dp.RCC.ahb3enr.modify(|_, w| w.fmcen().enabled());
        dp.RCC.ahb1enr.modify(|_, w| w.dma2den().enabled());

        /*-- GPIOs Configuration -----------------------------------------------------*/
        /*
         +-------------------+--------------------+--------------------+--------------------+
         +                       SDRAM pins assignment                                      +
         +-------------------+--------------------+--------------------+--------------------+
         | PD0  <-> FMC_D2   | PE0  <-> FMC_NBL0  | PF0  <-> FMC_A0    | PG0  <-> FMC_A10   |
         | PD1  <-> FMC_D3   | PE1  <-> FMC_NBL1  | PF1  <-> FMC_A1    | PG1  <-> FMC_A11   |
         | PD8  <-> FMC_D13  | PE7  <-> FMC_D4    | PF2  <-> FMC_A2    | PG8  <-> FMC_SDCLK |
         | PD9  <-> FMC_D14  | PE8  <-> FMC_D5    | PF3  <-> FMC_A3    | PG15 <-> FMC_NCAS  |
         | PD10 <-> FMC_D15  | PE9  <-> FMC_D6    | PF4  <-> FMC_A4    |--------------------+
         | PD14 <-> FMC_D0   | PE10 <-> FMC_D7    | PF5  <-> FMC_A5    |
         | PD15 <-> FMC_D1   | PE11 <-> FMC_D8    | PF11 <-> FMC_NRAS  |
         +-------------------| PE12 <-> FMC_D9    | PF12 <-> FMC_A6    |
                             | PE13 <-> FMC_D10   | PF13 <-> FMC_A7    |
                             | PE14 <-> FMC_D11   | PF14 <-> FMC_A8    |
                             | PE15 <-> FMC_D12   | PF15 <-> FMC_A9    |
         +-------------------+--------------------+--------------------+
         | PB5 <-> FMC_SDCKE1|
         | PB6 <-> FMC_SDNE1 |
         | PC0 <-> FMC_SDNWE |
         +-------------------+

        */

        // SDRAM GPIO Init
        dp.RCC.ahb1enr.modify(|_, w| w.gpioben().enabled());
        dp.RCC.ahb1enr.modify(|_, w| w.gpiocen().enabled());
        dp.RCC.ahb1enr.modify(|_, w| w.gpioden().enabled());
        dp.RCC.ahb1enr.modify(|_, w| w.gpioeen().enabled());
        dp.RCC.ahb1enr.modify(|_, w| w.gpiofen().enabled());
        dp.RCC.ahb1enr.modify(|_, w| w.gpiogen().enabled());

        let gb = dp.GPIOB.split();
        let gc = dp.GPIOC.split();
        let gd = dp.GPIOD.split();
        let ge = dp.GPIOE.split();
        let gf = dp.GPIOF.split();
        let gg = dp.GPIOG.split();

        #[rustfmt::skip]
        let _fmc_pins = fmc_pins!(
            // A0-A11
            gf.pf0,
            gf.pf1,
            gf.pf2,
            gf.pf3,
            gf.pf4,
            gf.pf5,
            gf.pf12,
            gf.pf13,
            gf.pf14,
            gf.pf15,
            gg.pg0,
            gg.pg1,
            // BA0-BA1
            gg.pg4,
            gg.pg5,
            // D0-D31
            gd.pd14,
            gd.pd15,
            gd.pd0,
            gd.pd1,
            ge.pe7,
            ge.pe8,
            ge.pe9,
            ge.pe10,
            ge.pe11,
            ge.pe12,
            ge.pe13,
            ge.pe14,
            ge.pe15,
            gd.pd8,
            gd.pd9,
            gd.pd10,
            // NBL0 - NBL1
            ge.pe0,
            ge.pe1,
            // SDCKE1
            gb.pb5,
            // SDCLK
            gg.pg8,
            // SDNCAS
            gg.pg15,
            // SDNE1
            gb.pb6,
            // SDNRAS
            gf.pf11,
            // SDNWE
            gc.pc0
        );

        // SDRAM DMA Init
        // Deinit
        let dma2_stream0 = dp.DMA2.st.get(0).unwrap();
        dma2_stream0.cr.modify(|_, w| w.en().clear_bit());
        dma2_stream0.cr.modify(|_, w| unsafe { w.bits(0x00) });
        dma2_stream0.ndtr.modify(|_, w| unsafe { w.bits(0x00) });
        dma2_stream0.par.modify(|_, w| unsafe { w.bits(0x00) });
        dma2_stream0.m0ar.modify(|_, w| unsafe { w.bits(0x00) });
        dma2_stream0.m1ar.modify(|_, w| unsafe { w.bits(0x00) });
        dma2_stream0.fcr.modify(|_, w| unsafe { w.bits(0x21) });

        // Init
        // Check if the DMA Stream is effectively disabled
        while dma2_stream0.cr.read().en().bit_is_set() {}

        dma2_stream0.cr.modify(|_, w| {
            w.chsel()
                .bits(0)
                .dir()
                .memory_to_memory()
                .pinc()
                .set_bit()
                .minc()
                .set_bit()
                .psize()
                .bits16()
                .msize()
                .bits16()
                .circ()
                .clear_bit()
                .pfctrl()
                .clear_bit()
        });

        dp.DMA2.lifcr.write(|w| {
            w.cdmeif0()
                .clear_bit()
                .cfeif0()
                .clear_bit()
                .chtif0()
                .clear_bit()
                .ctcif0()
                .clear_bit()
                .cteif0()
                .clear_bit()
        });

        // TODO: calculate priority for
        // Device           : DMA2 Stream0
        // PreemptPriority  : 0x0f
        // SubPriority      : 0x00
        // NVIC::set_priority(&mut self, interrupt, prio);
        unsafe { NVIC::unmask(Interrupt::DMA2_STREAM0) };

        // SDRAM FMC Init
        dp.FMC
            .sdcr1()
            .modify(|_, w| w.rpipe().clocks1().rburst().disabled().sdclk().div2());

        dp.FMC.sdcr2().modify(|_, w| {
            w.rpipe()
                .no_delay()
                .rburst()
                .disabled()
                .sdclk()
                .disabled()
                .wp()
                .disabled()
                .cas()
                .clocks3()
                .nb()
                .nb4()
                .mwid()
                .bits16()
                .nr()
                .bits12()
                .nc()
                .bits8()
        });

        dp.FMC.sdtr1().modify(|_, w| w.trc().bits(7).trp().bits(2));

        dp.FMC.sdtr2().modify(|_, w| {
            w.trcd()
                .bits(2)
                .trp()
                .bits(0)
                .twr()
                .bits(2)
                .trc()
                .bits(0)
                .tras()
                .bits(4)
                .txsr()
                .bits(7)
                .tmrd()
                .bits(2)
        });

        // Wait for SDRAM module is ready
        while dp.FMC.sdsr.read().busy().bit_is_set() {}

        let _ = &dp.FMC.sdcmr.modify(|_, w| {
            w.mode()
                .clock_configuration_enable()
                .ctb2()
                .set_bit()
                .ctb1()
                .clear_bit()
                .nrfs()
                .bits(1)
                .mrd()
                .bits(0)
        });

        delay.delay_us(100);

        // Wait for SDRAM module is ready
        while dp.FMC.sdsr.read().busy().bit_is_set() {}

        let _ = &dp.FMC.sdcmr.modify(|_, w| {
            w.mode()
                .pall()
                .ctb2()
                .set_bit()
                .ctb1()
                .clear_bit()
                .nrfs()
                .bits(1)
                .mrd()
                .bits(0)
        });

        // Wait for SDRAM module is ready
        while dp.FMC.sdsr.read().busy().bit_is_set() {}

        let _ = &dp.FMC.sdcmr.modify(|_, w| {
            w.mode()
                .load_mode_register()
                .ctb2()
                .set_bit()
                .ctb1()
                .clear_bit()
                .nrfs()
                .bits(4)
                .mrd()
                .bits(0x0230)
        });

        // Wait for SDRAM module is ready
        while dp.FMC.sdsr.read().busy().bit_is_set() {}

        let _ = &dp.FMC.sdrtr.modify(|_, w| w.count().bits(1386));

        // Wait for SDRAM module is ready
        while dp.FMC.sdsr.read().busy().bit_is_set() {}

        return 0xD000_0000 as *mut u16;
    }
}

// DMA2 Stream0 Interrupt
#[interrupt]
fn DMA2_STREAM0() {
    cortex_m::interrupt::free(|_| {
        let dma2 = unsafe { &*stm32f4xx_hal::pac::DMA2::ptr() };

        dma2.lifcr.write(|w| {
            w.cdmeif0()
                .clear_bit()
                .cfeif0()
                .clear_bit()
                .chtif0()
                .clear_bit()
                .ctcif0()
                .clear_bit()
                .cteif0()
                .clear_bit()
        });
    });
}
