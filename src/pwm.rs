#![allow(dead_code)]

use crate::join;
use embassy_futures::select::select;
use embassy_futures::select::Either;
use embassy_rp::peripherals;
use embassy_rp::pwm::{Channel, Config, Pwm, PwmPinA, PwmPinB};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;

pub type PwmSignal = Signal<ThreadModeRawMutex, u16>;

const TOP_CLOCK: u16 = 0x8000;
pub static MOTOR0_PWM: PwmSignal = PwmSignal::new();
pub static MOTOR1_PWM: PwmSignal = PwmSignal::new();
pub static MOTOR2_PWM: PwmSignal = PwmSignal::new();
pub static MOTOR3_PWM: PwmSignal = PwmSignal::new();
pub static MOTOR4_PWM: PwmSignal = PwmSignal::new();
pub static MOTOR5_PWM: PwmSignal = PwmSignal::new();
pub static LED0_PWM: PwmSignal = PwmSignal::new();
pub static FAN0_PWM: PwmSignal = PwmSignal::new();
pub static FAN1_PWM: PwmSignal = PwmSignal::new();

struct PwmPeripherals {
    chan0: peripherals::PWM_CH0,
    chan1: peripherals::PWM_CH1,
    chan2: peripherals::PWM_CH2,
    chan3: peripherals::PWM_CH3,
    chan4: peripherals::PWM_CH4,
    chan5: peripherals::PWM_CH5,
    chan6: peripherals::PWM_CH6,
    chan7: peripherals::PWM_CH7,
    motor0: peripherals::PIN_1,  // PWM_CH0B
    motor1: peripherals::PIN_4,  // PWM CH2A
    motor2: peripherals::PIN_7,  // PWM CH3B
    motor3: peripherals::PIN_10, // PWM CH5A
    motor4: peripherals::PIN_13, // PWM CH6B
    motor5: peripherals::PIN_16, // PWM CH0A
    led0: peripherals::PIN_18,   // PWM CH1A
    fan0: peripherals::PIN_19,   // PWM CH1B
    fan1: peripherals::PIN_21,   // PWM CH2B
}

pub async fn slice_worker_ab<C, U, V>(
    chan: C,
    p_a: U,
    sig_a: &'static PwmSignal,
    p_b: V,
    sig_b: &'static PwmSignal,
) -> !
where
    C: Channel,
    U: PwmPinA<C>,
    V: PwmPinB<C>,
{
    let mut config = Config::default();
    config.top = TOP_CLOCK;
    let mut pwm = Pwm::new_output_ab(chan, p_a, p_b, config.clone());
    loop {
        match select(sig_a.wait(), sig_b.wait()).await {
            Either::First(c) => {
                config.compare_a = c;
            }
            Either::Second(c) => {
                config.compare_b = c;
            }
        };
        pwm.set_config(&config);
    }
}

pub async fn slice_worker_a<C, U>(chan: C, p: U, sig: &'static PwmSignal) -> !
where
    C: Channel,
    U: PwmPinA<C>,
{
    let mut config = Config::default();
    config.top = TOP_CLOCK;
    let mut pwm = Pwm::new_output_a(chan, p, config.clone());
    loop {
        config.compare_a = sig.wait().await;
        pwm.set_config(&config);
    }
}

pub async fn slice_worker_b<C, U>(chan: C, p: U, sig: &'static PwmSignal) -> !
where
    C: Channel,
    U: PwmPinB<C>,
{
    let mut config = Config::default();
    config.top = TOP_CLOCK;
    let mut pwm = Pwm::new_output_b(chan, p, config.clone());
    loop {
        config.compare_b = sig.wait().await;
        pwm.set_config(&config);
    }
}

#[embassy_executor::task]
pub async fn pwm_worker(p: PwmPeripherals) {
    join!(
        slice_worker_ab(p.chan0, p.motor5, &MOTOR5_PWM, p.motor0, &MOTOR0_PWM),
        slice_worker_ab(p.chan1, p.led0, &LED0_PWM, p.fan0, &FAN0_PWM),
        slice_worker_ab(p.chan2, p.motor1, &MOTOR1_PWM, p.fan1, &FAN1_PWM),
        slice_worker_b(p.chan3, p.motor2, &MOTOR2_PWM),
        slice_worker_a(p.chan5, p.motor3, &MOTOR3_PWM),
        slice_worker_b(p.chan6, p.motor4, &MOTOR4_PWM),
    )
    .await;
}
