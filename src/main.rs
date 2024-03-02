//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.

#![no_std]
#![no_main]

mod encoder;
mod motor;
// mod sx1509;

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio;
use embassy_rp::pwm::{Config, Pwm};
use embassy_time::{Duration, Instant, Ticker};
use fixed::traits::FromFixed;
use gpio::{AnyPin, Input};
use {defmt_rtt as _, panic_probe as _};

use motor::MotorControl;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Hello World!");

    // create encoder
    use crate::encoder::{encoder_task, Encoder, EncoderMutex};
    static ENC: EncoderMutex = EncoderMutex::new(Encoder::new());
    let clk_pin = Input::new(AnyPin::from(p.PIN_0), gpio::Pull::None);
    unwrap!(spawner.spawn(encoder_task(&ENC, clk_pin)));

    // create pwm
    let mut c = Config::default();
    c.top = 0x800;
    c.compare_b = 0x0;
    let mut pwm = Pwm::new_output_b(p.PWM_CH0, p.PIN_1, c.clone());

    let mut motor_control = motor::Pid::new();

    let mut ticker = Ticker::every(Duration::from_hz(1000));
    let mut last_time = Instant::now();
    loop {
        let value = ENC.lock().await.read();
        let current_time = Instant::now();
        let control = motor_control.update(value.into(), current_time - last_time);
        last_time = current_time;
        c.compare_b = u16::from_fixed(control * 0x800);
        pwm.set_config(&c);

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
