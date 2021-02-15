#![no_main]
#![no_std]

use hal::gpio::{gpiob::PB1, Analog};
use panic_halt as _;

use stm32f0xx_hal as hal;

use crate::hal::{adc::Adc, delay::Delay, pac, prelude::*, serial::Serial};

use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;

use core::fmt::Write;

use debugless_unwrap::*;

#[entry]
fn main() -> ! {
    if let (Some(mut p), Some(cp)) = (pac::Peripherals::take(), Peripherals::take()) {
        let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);

        let gpioa = p.GPIOA.split(&mut rcc);
        let gpiob = p.GPIOB.split(&mut rcc);

        let (sel0, sel1, sel2, sel3, en, mut led, mut an_in, tx, rx) =
            cortex_m::interrupt::free(move |cs| {
                (
                    // mux pins
                    gpioa.pa0.into_push_pull_output(cs).downgrade(),
                    gpioa.pa1.into_push_pull_output(cs).downgrade(),
                    gpioa.pa2.into_push_pull_output(cs).downgrade(),
                    gpioa.pa3.into_push_pull_output(cs).downgrade(),
                    gpioa.pa4.into_push_pull_output(cs).downgrade(),
                    // (Re-)configure PA1 as output
                    gpioa.pa5.into_open_drain_output(cs),
                    // (Re-)configure PA0 as analog input
                    gpiob.pb1.into_analog(cs),
                    // serial pins
                    gpioa.pa9.into_alternate_af1(cs),
                    gpioa.pa10.into_alternate_af1(cs),
                )
            });

        let mut delay = Delay::new(cp.SYST, &rcc);

        let mut serial = Serial::usart1(p.USART1, (tx, rx), 9600.bps(), &mut rcc);

        let mut mux = cd74hc4067::CD74HC4067::new(sel0, sel1, sel2, sel3, en).debugless_unwrap();

        let _ = writeln!(serial, "Starting up angle sensor");

        let mut adc = Adc::new(p.ADC, &mut rcc);

        let mut values = [0u16; 16];
        loop {
            led.toggle().ok();

            for i in 0..16 {
                let _ = mux.set_output_active(i);
                let enabled = mux.enable().debugless_unwrap();

                let val = sample_n_avg(&mut adc, &mut an_in, 16);
                values[i as usize] = val;
                mux = enabled.disable().debugless_unwrap();
            }
            if writeln!(serial, "{:?}", values).is_ok() {}
            delay.delay_ms(200u32);
        }
    }

    loop {
        continue;
    }
}

fn sample_n_avg(adc: &mut Adc, pin: &mut PB1<Analog>, n: usize) -> u16 {
    let mut sum = 0u64;
    for _ in 0..n {
        let sample: u16 = adc.read(pin).debugless_unwrap();
        sum += sample as u64;
    }
    (sum / n as u64) as u16
}
