use hal::interrupt;
use hal::pac::Interrupt;
use hal::pac::Peripherals as device;
use hal::pac::NVIC;
use stm32f4xx_hal as hal;

use embedded_hal::blocking::delay::DelayUs;

pub struct Sdram {}
impl Sdram {
    pub fn new<D>(delay: &mut D) -> *mut u16
    where
        D: DelayUs<u16>,
    {
        let dp = unsafe { device::steal() };
        dp.RCC.ahb3enr.modify(|_, w| w.fmcen().enabled());
        dp.RCC.ahb1enr.modify(|_, w| w.dma2den().enabled());

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

        delay.delay_us(1);

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

        return 0xd000_0000 as *mut u16;
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
