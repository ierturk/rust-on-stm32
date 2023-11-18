use hal::gpio::GpioExt;
use hal::gpio::Speed;
use hal::pac::Peripherals as device;
use stm32f4xx_hal as hal;

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

pub struct TsDisp {
    pub ts: Ts,
    pub sdram: Sdram,
}
impl TsDisp {
    pub fn new() -> TsDisp {
        return TsDisp {
            ts: Ts::new(),
            sdram: Sdram::new(),
        };
    }
}

pub struct Ts {}
impl Ts {
    pub fn new() -> Ts {
        return Ts {};
    }
}

pub struct Sdram {}
impl Sdram {
    pub fn new() -> Sdram {
        // BSP_SDRAM_MspInit
        let dp = device::take().unwrap();
        dp.RCC.ahb3enr.modify(|_, w| w.fmcen().enabled());
        dp.RCC.ahb1enr.modify(|_, w| w.dma2den().enabled());

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

        // dp.FMC.sdcm().modify(|_, w| w.trc().bits(7).trp().bits(2));

        return Sdram {};
    }
}
