#![no_std]
#![no_main]

// pick a panicking behavior
//use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m::{
    asm,
    interrupt::{free, Mutex},
    iprintln,
    Peripherals,
    peripheral::{
        NVIC,
        itm::Stim,
    },
};
use cortex_m_rt::{entry,exception};
use stm32wb_hal as hal;
use hal::{
    hal::digital::v2::{PinState, OutputPin},
    interrupt,
    flash::{Parts},
    gpio::{Edge, ExtiPin},
    prelude::*,
    rcc::{ApbDivider, Config, HDivider, HseDivider, PllConfig, PllSrc, Rcc, RtcClkSrc, RfWakeupClock, SysClkSrc, },
    stm32,
};
use core::borrow::Borrow;
// these are for the mutexes, can we simplify them better?
use stm32wb_hal::gpio::{Input, PullUp};
use stm32wb_hal::gpio::gpioc::PC4;
use stm32wb_hal::gpio::gpiod::PD0;

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

// WTF why does this need to be typed so explicitly!
static MY_SW1: Mutex<RefCell<Option<PC4<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));
static MY_SW2: Mutex<RefCell<Option<PD0<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));


#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = hal::stm32::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = setup_clocks(rcc, flash);

    let stim = &mut cp.ITM.stim;
    iprintln!(&mut stim[0], "booty boot clock is {:?}", rcc.clocks.sysclk());

    //let clocks = hal::rcc::Clocks::default();
    let mut delay_naiive = hal::delay::DelayCM::new(rcc.clocks);

    // On STM32WB55-NUCLEO a green LED is connected to the pin PB0
    let mut gpiob = dp.GPIOB.split(&mut rcc);
    let mut gpioc = dp.GPIOC.split(&mut rcc);
    let mut gpiod = dp.GPIOD.split(&mut rcc);
    let mut ledg = gpiob.pb0.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut ledr = gpiob.pb1.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut ledb = gpiob.pb5.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    // exit 15_10 pc 13 if sb48 is on,
    //let mut sw1 = gpioc.pc13.into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);
    // exti 4, pc4 if sb48 is off and sb47 is on (default)
    let mut sw1 = gpioc.pc4.into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);
    // exti 0
    let mut sw2 = gpiod.pd0.into_pull_up_input(&mut gpiod.moder, &mut gpiod.pupdr);
    // exti 1
    let mut sw3 = gpiod.pd1.into_pull_up_input(&mut gpiod.moder, &mut gpiod.pupdr);

    sw1.make_interrupt_source(&mut dp.SYSCFG);
    sw1.enable_interrupt(&mut dp.EXTI);
    sw1.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
    sw2.make_interrupt_source(&mut dp.SYSCFG);
    sw2.enable_interrupt(&mut dp.EXTI);
    sw2.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
    sw3.make_interrupt_source(&mut dp.SYSCFG);
    sw3.enable_interrupt(&mut dp.EXTI);
    sw3.trigger_on_edge(&mut dp.EXTI, Edge::Falling);

    // Store the switches in the mutices, moving it.
    free(|cs| {
        MY_SW1.borrow(cs).replace(Some(sw1));
        MY_SW2.borrow(cs).replace(Some(sw2));
    });


    // Enable interrupts
    unsafe {
        NVIC::unmask(stm32::Interrupt::EXTI0);
        NVIC::unmask(stm32::Interrupt::EXTI1);
        NVIC::unmask(stm32::Interrupt::EXTI4);
    }

    let mut i = 0;
    loop {
        iprintln!(&mut stim[0], "loopy da loop: {}", i);
        stim[1].write_u16b(i);
        stim[2].write_u16b(i*2);
        i += 1;
        // delay....
        delay_naiive.delay_ms(300 as u16);

        ledr.set_state(PinState::from((i & 1) > 0));
        ledg.set_state(PinState::from((i & 2) > 0));
        ledb.set_state(PinState::from((i & 4) > 0));

    }
}

fn setup_clocks(rcc: Rcc, mut flash: Parts) -> Rcc {

    let clock_config_hse32_pll64 = Config::new(SysClkSrc::Pll(PllSrc::Hse(HseDivider::NotDivided)))
        .cpu1_hdiv(HDivider::NotDivided)
        .cpu2_hdiv(HDivider::Div2)
        .apb1_div(ApbDivider::NotDivided)
        .apb2_div(ApbDivider::NotDivided)
        .pll_cfg(PllConfig {
            m: 2,
            n: 12,
            r: 3,
            q: Some(4),
            p: Some(3),
        });

    let clock_config_hse32_nopll = Config::new(SysClkSrc::HseSys(HseDivider::NotDivided))
        .cpu1_hdiv(HDivider::NotDivided)
        .cpu2_hdiv(HDivider::NotDivided)
        .apb1_div(ApbDivider::NotDivided)
        .apb2_div(ApbDivider::NotDivided)
        .rtc_src(RtcClkSrc::Lse)
        .rf_wkp_sel(RfWakeupClock::Lse)
        ;
    //rcc.apply_clock_config(clock_config_hse32_pll64, &mut flash.acr)
    rcc.apply_clock_config(clock_config_hse32_nopll, &mut flash.acr)
}

#[exception]
#[allow(non_snake_case)]
unsafe fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}

#[interrupt]
fn EXTI0() {
    let mut stim = unsafe {&mut cortex_m::Peripherals::steal().ITM.stim};
    iprintln!(&mut stim[0], "SW2 pressed");
    free(|cs| {
        let mut btn_ref = MY_SW2.borrow(cs).borrow_mut();
        if let Some(ref mut btn) = btn_ref.deref_mut() {
            btn.clear_interrupt_pending_bit();
            // iprintln!(&mut stim[0], "button {:?} pressed", btn);  "pin doesn't implement debug blah blah"
        }
    });
}

#[interrupt]
fn EXTI4() {
    let mut stim = unsafe {&mut cortex_m::Peripherals::steal().ITM.stim};
    iprintln!(&mut stim[0], "SW1 pressed");
    free(|cs| {
        let mut btn_ref = MY_SW1.borrow(cs).borrow_mut();
        if let Some(ref mut btn) = btn_ref.deref_mut() {
            btn.clear_interrupt_pending_bit();
            // iprintln!(&mut stim[0], "button {:?} pressed", btn);
        }
    });
}
