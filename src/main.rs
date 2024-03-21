#![no_std]
#![no_main]

mod encoder;
mod i2c;
mod motor;
mod pwm;
mod serial;
mod utils;

use defmt::*;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_rp::gpio;
use embassy_rp::interrupt;
use embassy_rp::interrupt::{InterruptExt, Priority};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
use fixed_macro::fixed;
use gpio::{AnyPin, Input, Output};
use {defmt_rtt as _, panic_probe as _};

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SWI_IRQ_1() {
    EXECUTOR_HIGH.on_interrupt()
}

#[interrupt]
unsafe fn SWI_IRQ_0() {
    EXECUTOR_MED.on_interrupt()
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Start Motor Controller!");

    // High-priority executor: SWI_IRQ_1, priority level 2
    interrupt::SWI_IRQ_1.set_priority(Priority::P2);
    EXECUTOR_HIGH.start(interrupt::SWI_IRQ_1);

    // Medium-priority executor: SWI_IRQ_0, priority level 3
    interrupt::SWI_IRQ_0.set_priority(Priority::P3);
    EXECUTOR_MED.start(interrupt::SWI_IRQ_0);

    // start serial
    spawner.must_spawn(serial::serial_task(p.USB));
    spawner.must_spawn(i2c::device_task(p.I2C1, p.PIN_26, p.PIN_27));

    // // start core1
    // spawn_core1(
    //     p.CORE1,
    //     unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
    //     move || {
    //         let executor = CORE1_EXECUTOR.init(Executor::new());
    //         executor
    //             .run(|spawner| spawner.must_spawn(i2c::device_task(p.I2C1, p.PIN_26, p.PIN_27)));
    //     },
    // );

    // create encoder
    let btn_signal: Signal<NoopRawMutex, ()> = Signal::new();
    let direction_pin = Output::new(AnyPin::from(p.PIN_0), gpio::Level::Low);
    join!(
        // create pwm worker
        pwm::slice_worker_b(p.PWM_CH0, p.PIN_1, &pwm::MOTOR0_PWM),
        // motor driver
        motor::motor_driver(
            &pwm::MOTOR0_PWM,
            &motor::MOTOR0_DRIVER,
            direction_pin,
            p.PIN_2,
        ),
        // read button
        async {
            let mut btn = Input::new(AnyPin::from(p.PIN_4), gpio::Pull::Up);
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
                    .set_target(fixed!(1.0: I16F16));
                btn_signal.wait().await;
                motor::MOTOR0_DRIVER
                    .lock()
                    .await
                    .set_target(fixed!(-1.0: I16F16));
            }
        },
        // blink LED
        async {
            let mut led = Output::new(AnyPin::from(p.PIN_25), gpio::Level::Low);
            let mut ticker = Ticker::every(Duration::from_hz(1));
            loop {
                led.toggle();
                ticker.next().await;
            }
        },
        // // logger
        // async {
        //     let mut ticker = Ticker::every(Duration::from_hz(10));
        //     loop {
        //         let value = motor::MOTOR0_DRIVER.lock().await.get_measure_value();
        //         info!(
        //             "value: {}",
        //             (value.saturating_mul(encoder::Fixed::from_num(2 << 12))).to_num::<i32>()
        //                 as f32
        //                 / (2 << 12) as f32
        //         );
        //         ticker.next().await;
        //     }
        // },
    )
    .await;
}
