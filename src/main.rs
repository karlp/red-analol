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
};
use cortex_m_rt::entry;
use stm32wb_hal as hal;
use hal::prelude::*;
use hal::hal::digital::v2::{PinState, OutputPin};

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::stm32::Peripherals::take().unwrap();
    let mut stim = &mut cp.ITM.stim;

    // Use default clock frequency of 4 MHz running from MSI
    let mut rcc = dp.RCC.constrain();

    //let clocks = setup_clocks();
    let clocks = hal::rcc::Clocks::default();
    let mut delay_naiive = hal::delay::DelayCM::new(clocks);

    // On STM32WB55-NUCLEO a green LED is connected to the pin PB0
    let mut gpiob = dp.GPIOB.split(&mut rcc);
    let mut gpioc = dp.GPIOC.split(&mut rcc);
    let mut gpiod = dp.GPIOD.split(&mut rcc);
    let mut ledg = gpiob.pb0.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut ledr = gpiob.pb1.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut ledb = gpiob.pb5.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut sw1 = gpioc.pc13.into_pull_up_input(&mut gpioc.moder, &mut gpioc.pupdr);
    let mut sw2 = gpiod.pd0.into_pull_up_input(&mut gpiod.moder, &mut gpiod.pupdr);
    let mut sw3 = gpiod.pd1.into_pull_up_input(&mut gpiod.moder, &mut gpiod.pupdr);


    iprintln!(& mut stim[0], "booty boot");
    let mut i = 0;
    loop {
        iprintln!(&mut stim[0], "loopy da loop: {}", i);
        i += 1;
        stim[1].write_u16(i);
        // delay....
        delay_naiive.delay_ms(300 as u16);

        // Ideally, in c, where the bit and results in a bool...
        // ledg.set_state(i & 1);
        // ledr.set_state(i & 2);
        // ledb.set_state(i & 4);
        // but I need a "PinState"
        ledr.set_state(PinState::from((i & 1) > 0));
        ledg.set_state(PinState::from((i & 2) > 0));
        ledb.set_state(PinState::from((i & 4) > 0));

    }
}

fn setup_clocks() {
}