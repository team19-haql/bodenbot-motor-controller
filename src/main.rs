#![no_std]
#![no_main]

mod encoder;
mod i2c;
mod motor;
mod pwm;
mod serial;
mod utils;

use defmt::*;
use embassy_executor::{Executor, Spawner};
use embassy_rp::gpio;
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
use fixed_macro::fixed;
use gpio::{AnyPin, Input, Output};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Start Motor Controller!");

    // start serial
    spawner.must_spawn(serial::serial_task(p.USB));
    // // start i2c
    spawner.must_spawn(i2c::device_task(p.I2C1, p.PIN_26, p.PIN_27));

    // spawn_core1(
    //     p.CORE1,
    //     unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
    //     move || {
    //         let executor1 = EXECUTOR1.init(Executor::new());
    //         executor1.run(|spawner| {
    //             spawner.must_spawn(i2c::device_task(p.I2C1, p.PIN_26, p.PIN_27));
    //         });
    //     },
    // );

    // create encoder
    use crate::encoder::{encoder_task, Encoder, EncoderMutex};
    static ENC: EncoderMutex = EncoderMutex::new(Encoder::new());
    let clk_pin = Input::new(AnyPin::from(p.PIN_10), gpio::Pull::None);
    spawner.must_spawn(encoder_task(&ENC, clk_pin));

    let btn_signal: Signal<NoopRawMutex, ()> = Signal::new();
    let direction_pin = Output::new(AnyPin::from(p.PIN_0), gpio::Level::Low);
    join!(
        // create pwm worker
        pwm::slice_worker_b(p.PWM_CH0, p.PIN_1, &pwm::MOTOR0_PWM),
        // read button
        async {
            let mut btn = Input::new(AnyPin::from(p.PIN_2), gpio::Pull::Up);
            loop {
                btn.wait_for_falling_edge().await;
                info!("press");
                btn_signal.signal(());
                Timer::after_millis(50).await;
                btn.wait_for_rising_edge().await;
                Timer::after_millis(50).await;
            }
        },
        // set pwm value with btn
        async {
            loop {
                btn_signal.wait().await;
                motor::MOTOR0_DRIVER
                    .lock()
                    .await
                    .set_target(fixed!(0.25: I16F16));
                btn_signal.wait().await;
                motor::MOTOR0_DRIVER
                    .lock()
                    .await
                    .set_target(fixed!(0.5: I16F16));
                btn_signal.wait().await;
                motor::MOTOR0_DRIVER
                    .lock()
                    .await
                    .set_target(fixed!(0.75: I16F16));
                btn_signal.wait().await;
                motor::MOTOR0_DRIVER
                    .lock()
                    .await
                    .set_target(fixed!(0: I16F16));
            }
        },
        // motor driver
        motor::motor_driver(&pwm::MOTOR0_PWM, &ENC, &motor::MOTOR0_DRIVER, direction_pin),
        // blink LED
        async {
            let mut led = Output::new(AnyPin::from(p.PIN_25), gpio::Level::Low);
            let mut ticker = Ticker::every(Duration::from_hz(1));
            loop {
                led.toggle();
                ticker.next().await;
            }
        },
        // logger
        async {
            let mut ticker = Ticker::every(Duration::from_hz(1));
            loop {
                let value = motor::MOTOR0_DRIVER.lock().await.get_target();
                info!(
                    "value: {}",
                    (value.saturating_mul(encoder::Fixed::from_num(1000))).to_num::<i32>() as f32
                        / 1000.0
                );
                ticker.next().await;
            }
        },
    )
    .await;
}
