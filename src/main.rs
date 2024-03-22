#![no_std]
#![no_main]

mod encoder;
mod i2c;
mod motor;
mod pwm;
mod serial;
mod utils;

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_rp::gpio;
use embassy_rp::interrupt;
use embassy_rp::interrupt::{InterruptExt, Priority};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
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
    defmt::info!("Start Motor Controller!");

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

    // Set to watchdog to reset if it's not fed within 1.05 seconds, and start it
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_secs(5));

    // create encoder
    let btn_signal: Signal<NoopRawMutex, ()> = Signal::new();
    join!(
        // create pwm worker
        // pwm::slice_worker_b(p.PWM_CH0, p.PIN_1, &pwm::MOTOR0_PWM),
        pwm::slice_worker_ab(
            p.PWM_CH0,
            p.PIN_16,
            &pwm::MOTOR5_PWM,
            p.PIN_1,
            &pwm::MOTOR0_PWM
        ),
        pwm::slice_worker_ab(
            p.PWM_CH1,
            p.PIN_18,
            &pwm::LED0_PWM,
            p.PIN_19,
            &pwm::FAN0_PWM
        ),
        pwm::slice_worker_ab(
            p.PWM_CH2,
            p.PIN_4,
            &pwm::MOTOR1_PWM,
            p.PIN_21,
            &pwm::FAN1_PWM
        ),
        pwm::slice_worker_b(p.PWM_CH3, p.PIN_7, &pwm::MOTOR2_PWM),
        pwm::slice_worker_a(p.PWM_CH5, p.PIN_10, &pwm::MOTOR3_PWM),
        pwm::slice_worker_b(p.PWM_CH6, p.PIN_13, &pwm::MOTOR4_PWM),
        // motor drivers
        motor::motor_driver(&pwm::MOTOR0_PWM, &motor::MOTOR0_DRIVER, p.PIN_0, p.PIN_2,),
        motor::motor_driver(&pwm::MOTOR1_PWM, &motor::MOTOR1_DRIVER, p.PIN_3, p.PIN_5,),
        motor::motor_driver(&pwm::MOTOR2_PWM, &motor::MOTOR2_DRIVER, p.PIN_6, p.PIN_8,),
        motor::motor_driver(&pwm::MOTOR3_PWM, &motor::MOTOR3_DRIVER, p.PIN_9, p.PIN_11,),
        motor::motor_driver(&pwm::MOTOR4_PWM, &motor::MOTOR4_DRIVER, p.PIN_12, p.PIN_14,),
        motor::motor_driver(&pwm::MOTOR5_PWM, &motor::MOTOR5_DRIVER, p.PIN_15, p.PIN_17,),
        // read button
        async {
            let mut btn = Input::new(AnyPin::from(p.PIN_28), gpio::Pull::Up);
            loop {
                btn.wait_for_falling_edge().await;
                btn_signal.signal(());
                Timer::after_millis(50).await;
                btn.wait_for_rising_edge().await;
                Timer::after_millis(50).await;
            }
        },
        // blink LED
        async {
            let mut led = Output::new(AnyPin::from(p.PIN_25), gpio::Level::Low);
            let mut ticker = Ticker::every(Duration::from_hz(1));
            loop {
                led.toggle();
                watchdog.feed();
                ticker.next().await;
            }
        },
        // logger
        async {
            loop {
                btn_signal.wait().await;
                let value = motor::MOTOR0_DRIVER.lock().await.get_target();
                defmt::info!("value: {}", value.to_num::<f32>());
            }
        },
    )
    .await;
}
