#![no_std]
#![no_main]

// pick a panicking behavior
//use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m::{
    asm,
    iprintln,
    Peripherals,
    peripheral::{NVIC},
};
use cortex_m_rt::{entry,exception};
use stm32wb_hal as hal;
use hal::{
    hal::digital::v2::{PinState, OutputPin},
    interrupt,
    flash::{Parts},
    gpio::{Edge, ExtiPin},
    prelude::*,
    rcc::{ApbDivider, Config, HDivider, HseDivider, PllConfig, PllSrc, Rcc, SysClkSrc, },
    stm32,
};



#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = hal::stm32::Peripherals::take().unwrap();
    let mut stim = &mut cp.ITM.stim;

    // Use default clock frequency of 4 MHz running from MSI
    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();

    let mut rcc = setup_clocks(rcc, flash);
    //let clocks = hal::rcc::Clocks::default();
    let mut delay_naiive = hal::delay::DelayCM::new(rcc.clocks);

    // On STM32WB55-NUCLEO a green LED is connected to the pin PB0
    let mut gpiob = dp.GPIOB.split(&mut rcc);
    let mut gpioc = dp.GPIOC.split(&mut rcc);
    let mut gpiod = dp.GPIOD.split(&mut rcc);
    let mut ledg = gpiob.pb0.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut ledr = gpiob.pb1.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut ledb = gpiob.pb5.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    // exit 15_10
    let mut sw1 = gpioc.pc13.into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);
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

    // Enable interrupts
    unsafe {
        NVIC::unmask(stm32::Interrupt::EXTI0);
        NVIC::unmask(stm32::Interrupt::EXTI1);
        NVIC::unmask(stm32::Interrupt::EXTI10_15);
    }

    iprintln!(& mut stim[0], "booty boot");
    let mut i = 0;
    loop {
        iprintln!(&mut stim[0], "loopy da loop: {}", i);
        i += 1;
        stim[1].write_u16(i);
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
    // rcc.apply_clock_config(clock_config, &mut dp.FLASH.constrain().acr)
    rcc.apply_clock_config(clock_config_hse32_pll64, &mut flash.acr)
}

#[exception]
#[allow(non_snake_case)]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}

#[interrupt]
fn EXTI10_15() {
    // just fucking what?! why is it so fucking hard to get access to the stimulus port!!
    // iprintln!("button 2 pressed");
}
