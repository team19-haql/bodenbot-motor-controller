//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.

#![no_std]
#![no_main]

mod sx1509;

extern crate embedded_hal as hal;

use bsp::entry;
use core::cell::RefCell;
use defmt::*;
use defmt_rtt as _;
use fugit::RateExtU32;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    I2C,
};
use hal::digital::v2::OutputPin;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let clocks = init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let i2c = RefCell::new(I2C::i2c0(
        pac.I2C0,
        pins.gpio0.into_function(),
        pins.gpio1.into_function(),
        400u32.kHz(),
        &mut pac.RESETS,
        125_000_000.Hz(),
    ));

    let expander = sx1509::Sx1509::new(&i2c, sx1509::DEFAULT_ADDRESS).unwrap();
    let extra_pins = expander.split();

    // let mut pwm0 = pins.gpio0.into_analog_output().unwrap();
    let mut a0 = extra_pins.gpio0.into_analog_output().unwrap();
    let mut d0 = pins.gpio2.into_push_pull_output();

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    loop {
        info!("on!");
        d0.set_high().unwrap();
        for i in 0..255 {
            a0.write_analog(i).unwrap();
            delay.delay_ms(20);
        }
        info!("off!");
        d0.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
