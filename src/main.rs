#![no_std]
#![no_main]

mod encoder;
mod i2c;
mod motor;
mod pwm;
mod serial;
mod utils;

use embassy_executor::Spawner;
use embassy_rp::gpio;
use embassy_rp::watchdog::Watchdog;
use embassy_time::{Duration, Ticker, Timer};
use gpio::{AnyPin, Input, Output};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    defmt::info!("Start Motor Controller!");

    // start serial
    spawner.must_spawn(serial::serial_task(p.USB));
    spawner.must_spawn(i2c::device_task(p.I2C1, p.PIN_26, p.PIN_27));

    // Set to watchdog to reset if it's not fed within 1.05 seconds, and start it
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    // watchdog.start(Duration::from_secs(10));

    join!(
        // create pwm workers
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
        pwm::slice_worker_b(p.PWM_CH5, p.PIN_11, &pwm::MOTOR3_PWM),
        pwm::slice_worker_a(p.PWM_CH7, p.PIN_14, &pwm::MOTOR4_PWM),
        // start motor drivers
        motor::motor_driver(&pwm::MOTOR0_PWM, &motor::MOTOR0_DRIVER, p.PIN_0, p.PIN_2,),
        motor::motor_driver(&pwm::MOTOR1_PWM, &motor::MOTOR1_DRIVER, p.PIN_3, p.PIN_5,),
        motor::motor_driver(&pwm::MOTOR2_PWM, &motor::MOTOR2_DRIVER, p.PIN_6, p.PIN_8,),
        motor::motor_driver(&pwm::MOTOR3_PWM, &motor::MOTOR3_DRIVER, p.PIN_9, p.PIN_10,),
        motor::motor_driver(&pwm::MOTOR4_PWM, &motor::MOTOR4_DRIVER, p.PIN_12, p.PIN_13,),
        motor::motor_driver(&pwm::MOTOR5_PWM, &motor::MOTOR5_DRIVER, p.PIN_15, p.PIN_17,),
        // read button
        async {
            let mut btn = Input::new(AnyPin::from(p.PIN_28), gpio::Pull::Up);
            loop {
                btn.wait_for_falling_edge().await;
                defmt::info!("Button pressed!");
                // wait to debounce
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
                watchdog.feed();
                led.set_high();
                Timer::after_millis(200).await;
                led.set_low();
                ticker.next().await;
            }
        },
    )
    .await;
}
