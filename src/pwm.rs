#![allow(dead_code)]

use embassy_futures::select::select;
use embassy_futures::select::Either;
use embassy_rp::pwm::{Channel, Config, Pwm, PwmPinA, PwmPinB};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;

pub type PwmSignal = Signal<ThreadModeRawMutex, u16>;

pub const TOP_CLOCK: u16 = 0x8000;
pub static MOTOR0_PWM: PwmSignal = PwmSignal::new();
pub static MOTOR1_PWM: PwmSignal = PwmSignal::new();
pub static MOTOR2_PWM: PwmSignal = PwmSignal::new();
pub static MOTOR3_PWM: PwmSignal = PwmSignal::new();
pub static MOTOR4_PWM: PwmSignal = PwmSignal::new();
pub static MOTOR5_PWM: PwmSignal = PwmSignal::new();
pub static LED0_PWM: PwmSignal = PwmSignal::new();
pub static FAN0_PWM: PwmSignal = PwmSignal::new();
pub static FAN1_PWM: PwmSignal = PwmSignal::new();

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
