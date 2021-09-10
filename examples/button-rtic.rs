#![no_std]
#![no_main]

// pick a panicking behavior
//use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m::peripheral::itm::Stim;
use cortex_m_rt::exception;

trait StimExt {
    /// Write byte, blocking
    fn write_u8b(&mut self, value: u8);
    /// Write half word, 16bit, blocking
    fn write_u16b(&mut self, value: u16);
}

impl StimExt for Stim {
    #[inline]
    fn write_u8b(&mut self, value: u8) {
        while !self.is_fifo_ready() {}
        self.write_u8(value);
    }

    fn write_u16b(&mut self, value: u16) {
        while !self.is_fifo_ready() {}
        self.write_u16(value);
    }
}


#[rtic::app(device = hal::pac, peripherals = true)]
mod app {
    use cortex_m::{
        iprintln,
        peripheral::NVIC,
    };
    use stm32wb_hal as hal;
    use hal::{
        delay::DelayCM,
        flash::{Parts},
        gpio::{
            Edge, ExtiPin, Input, Output, PullUp, PushPull,
            gpiob::{PBx, PB0, PB1, PB5},
            gpioc::{PCx, PC4},
            gpiod::{PDx, PD0, PD1},
        },
        hal::digital::v2::{PinState, OutputPin},
        prelude::*,
        rcc::{ApbDivider, Config, HDivider, HseDivider, PllConfig, PllSrc, Rcc, RtcClkSrc, RfWakeupClock, SysClkSrc, },
        stm32,
    };
    use crate::StimExt;

    // I hate that these have to be differently typed...
    pub type Sw1 = PC4<Input<PullUp>>;
    pub type Sw2 = PD0<Input<PullUp>>;
    pub type Sw3 = PD1<Input<PullUp>>;
    pub type LedPB = PBx<Output<PushPull>>;


    #[shared]
    struct Shared {
        #[lock_free]
        sw1: Sw1,
        #[lock_free]
        sw2: Sw2,
        #[lock_free]
        sw3: Sw3,
        // stim: [Stim; 256],
        #[lock_free]
        ledr: LedPB,
        #[lock_free]
        ledg: LedPB,
        #[lock_free]
        ledb: LedPB,
    }

    #[local]
    struct Local {
        delay: DelayCM,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {

        let mut cp: cortex_m::Peripherals = ctx.core;
        let mut dp: hal::stm32::Peripherals = ctx.device;

        let rcc = dp.RCC.constrain();
        let flash = dp.FLASH.constrain();
        let mut rcc = setup_clocks(rcc, flash);

        let delay_naiive = hal::delay::DelayCM::new(rcc.clocks);

        let stim = &mut cp.ITM.stim;
        iprintln!(&mut stim[0], "RTIC's alive clock is {:?}", rcc.clocks.sysclk());

        let mut gpiob = dp.GPIOB.split(&mut rcc);
        let mut gpioc = dp.GPIOC.split(&mut rcc);
        let mut gpiod = dp.GPIOD.split(&mut rcc);
        let ledg = gpiob.pb0.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        let ledr = gpiob.pb1.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        let ledb = gpiob.pb5.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        // exit 15_10 pc 13 if sb48 is on,
        //let mut sw1 = gpioc.pc13.into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);
        // exti 4, pc4 if sb48 is off and sb47 is on (default)
        let mut sw1 = gpioc.pc4.into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);
        // exti 0
        let mut sw2 = gpiod.pd0.into_pull_up_input(&mut gpiod.moder, &mut gpiod.pupdr);
        // exti 1
        let mut sw3 = gpiod.pd1.into_pull_up_input(&mut gpiod.moder, &mut gpiod.pupdr);

        // Enable interrupts
        sw1.make_interrupt_source(&mut dp.SYSCFG);
        sw1.enable_interrupt(&mut dp.EXTI);
        sw1.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
        sw2.make_interrupt_source(&mut dp.SYSCFG);
        sw2.enable_interrupt(&mut dp.EXTI);
        sw2.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
        sw3.make_interrupt_source(&mut dp.SYSCFG);
        sw3.enable_interrupt(&mut dp.EXTI);
        sw3.trigger_on_edge(&mut dp.EXTI, Edge::Falling);

        unsafe {
            NVIC::unmask(stm32::Interrupt::EXTI0);
            NVIC::unmask(stm32::Interrupt::EXTI1);
            NVIC::unmask(stm32::Interrupt::EXTI4);
        }

        (Shared{
            sw1, sw2, sw3,
            // stim,
            ledr: ledr.downgrade(), ledg: ledg.downgrade(), ledb: ledb.downgrade(),
        },
         Local{ delay: delay_naiive},
         init::Monotonics())
    }

    #[idle(local=[delay], shared=[ledr, ledg, ledb])]
    fn idle(ctx: idle::Context) -> ! {
        let stim = unsafe {&mut cortex_m::Peripherals::steal().ITM.stim};
        iprintln!(&mut stim[0], "idle task entered...");

        let mut i = 0;
        loop {
            iprintln!(&mut stim[0], "loopy da loop: {}", i);
            stim[1].write_u16b(i);
            stim[2].write_u16b(i*2);
            i += 1;
            ctx.local.delay.delay_ms(300 as u16);

            // both work, one way gives you completion in intellij
            let ledr: &mut LedPB = ctx.shared.ledr;
            ledr.set_state(PinState::from((i & 1) > 0));
            ctx.shared.ledg.set_state(PinState::from((i & 2) > 0));
            ctx.shared.ledb.set_state(PinState::from((i & 4) > 0));
        }
    }


    fn setup_clocks(rcc: Rcc, mut flash: Parts) -> Rcc {
        // 32MHz is enough for us for now, and uses less power.
        let clock_config_hse32_nopll = Config::new(SysClkSrc::HseSys(HseDivider::NotDivided))
            .cpu1_hdiv(HDivider::NotDivided)
            .cpu2_hdiv(HDivider::NotDivided)
            .apb1_div(ApbDivider::NotDivided)
            .apb2_div(ApbDivider::NotDivided)
            .rtc_src(RtcClkSrc::Lse)
            .rf_wkp_sel(RfWakeupClock::Lse)
            ;
        rcc.apply_clock_config(clock_config_hse32_nopll, &mut flash.acr)
    }

    #[task(binds = EXTI4, shared = [sw1])]
    fn handle_sw1(ctx: handle_sw1::Context) {
        let stim = unsafe {&mut cortex_m::Peripherals::steal().ITM.stim};
        iprintln!(&mut stim[0], "KKK button sw1 pressed");
        // we don't actually need locks...
        //ctx.shared.sw1.lock(|b| b.clear_interrupt_pending_bit());
        ctx.shared.sw1.clear_interrupt_pending_bit();
    }
    
    #[task(binds = EXTI0, shared = [sw2])]
    fn handle_sw2(ctx: handle_sw2::Context) {
        let stim = unsafe {&mut cortex_m::Peripherals::steal().ITM.stim};
        iprintln!(&mut stim[0], "KKK button sw2 pressed");
        // we don't actually need locks...
        //ctx.shared.sw2.lock(|b| b.clear_interrupt_pending_bit());
        ctx.shared.sw2.clear_interrupt_pending_bit();
    }

    #[task(binds = EXTI1, shared = [sw3])]
    fn handle_sw3(ctx: handle_sw3::Context) {
        let stim = unsafe {&mut cortex_m::Peripherals::steal().ITM.stim};
        iprintln!(&mut stim[0], "KKK button sw3 pressed");
        // we don't actually need locks...
        //ctx.shared.sw3.lock(|b| b.clear_interrupt_pending_bit());
        ctx.shared.sw3.clear_interrupt_pending_bit();
    }
}

#[exception]
#[allow(non_snake_case)]
unsafe fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
