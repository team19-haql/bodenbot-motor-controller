//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.

#![no_std]
#![no_main]

mod encoder;
mod motor;
mod serial;
// mod sx1509;

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio;
use embassy_rp::pwm::{Config, Pwm};
use embassy_time::{Duration, Instant, Ticker};
use fixed::traits::FromFixed;
use gpio::{AnyPin, Input};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Start Motor Controller!");

    // start serial
    unwrap!(spawner.spawn(serial::serial_task(p.USB)));
    log::info!("Start motor controllr serial");

    // create encoder
    use crate::encoder::{encoder_task, Encoder, EncoderMutex};
    static ENC: EncoderMutex = EncoderMutex::new(Encoder::new());
    let clk_pin = Input::new(AnyPin::from(p.PIN_0), gpio::Pull::None);
    ENC.lock().await.set_direction(encoder::Direction::Forward);
    unwrap!(spawner.spawn(encoder_task(&ENC, clk_pin)));
    // create pwm
    let mut c = Config::default();
    c.top = 0x8000;
    c.compare_b = 0x0;
    let mut pwm = Pwm::new_output_b(p.PWM_CH6, p.PIN_29, c.clone());
    let mut btn = Input::new(AnyPin::from(p.PIN_9), gpio::Pull::Up);

    let mut motor_control = motor::Pid::new();

    let mut ticker = Ticker::every(Duration::from_hz(10));
    let mut last_time = Instant::now();
    loop {
        // led.toggle();
        let value = ENC.lock().await.read_reset();

        // update pid
        let current_time = Instant::now();
        let control = motor_control.update(value, current_time - last_time);

        log::info!("control: {:?}", control);

        last_time = current_time;

        // update pwm
        c.compare_b = u16::from_fixed(control).min(c.top);
        pwm.set_config(&c);

        // Timer::after_millis(20).await;
        ticker.next().await;
    }

    // motor_service!(
    //     motor1: (PIN_2, PIN_0, PIN_1, PWM_CH0),
    //     motor2: (PIN_5, PIN_3, PIN_4, PWM_CH1),
    //     motor3: (PIN_8, PIN_6, PIN_7, PWM_CH2),
    //     motor4: (PIN_11, PIN_9, PIN_10, PWM_CH3),
    //     motor5: (PIN_14, PIN_12, PIN_13, PWM_CH6),
    //     motor6: (PIN_17, PIN_15, PIN_16, PWM_CH0),
    // );
}
