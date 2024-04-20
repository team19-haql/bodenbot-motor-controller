//! The motor encoder spawns a new motor task loop. The task
//! will manage the pid motor controller for the given pins.
//!
//! The current state of the driver such as target value and
//! measured value can be read over i2c.
use crate::encoder::{Direction, Encoder};
use crate::pwm::{PwmSignal, TOP_CLOCK};
use crate::utils::Mutex;
use embassy_rp::gpio::{AnyPin, Level, Output};
use embassy_time::{Duration, Instant, Ticker};
use num_traits::float::FloatCore;
use rand::prelude::*;
use rand::rngs::SmallRng;

/// Proportional PID controller constant
const K_P: f32 = 8.0;
/// Integral PID controller constant
const K_I: f32 = 0.0;
/// Derivative PID controller constant
const K_D: f32 = 0.0;
const FREQUENCY: u64 = 30000;

pub type DriverMutex = Mutex<Driver>;

macro_rules! motor_drivers {
    ($($motor:ident),*$(,)?) => {
        $(
        #[doc = concat!("The global singleton for motor driver ", stringify!($motor))]
        pub static $motor: DriverMutex = DriverMutex::new(Driver::new());
        )*
    };
}

#[rustfmt::skip]
motor_drivers!(
    MOTOR0,
    MOTOR1,
    MOTOR2,
    MOTOR3,
    MOTOR4,
    MOTOR5,
);

/// PID controller for motor speed control
pub struct Driver {
    target_value: f32,
    previous_value: f32,
    integral: f32,
    output: f32,
    measured_value: f32,
}

impl Driver {
    pub const fn new() -> Self {
        Self {
            target_value: 0.0,
            previous_value: 0.0,
            integral: 0.0,
            output: 0.0,
            measured_value: 0.0,
        }
    }

    /// Read the target value for the PID controller
    pub fn set_target(&mut self, target: f32) {
        self.target_value = target
    }

    /// Read the current target value for the PID controller
    pub fn get_target(&self) -> f32 {
        self.target_value
    }

    /// Read the last measured_value for the PID controller
    pub fn get_measure_value(&self) -> f32 {
        self.measured_value
    }

    /// Update the PID controller with a new measured_value and time delta.
    pub fn update(&mut self, radians: f32, delta: Duration) -> f32 {
        // should not be longer than a few milliseconds
        // Divide by 1000000 to convert micros to seconds
        let dt = (delta.as_micros() as f32) / 1e-6;
        if dt.abs() < 1e-6 {
            return self.output;
        }

        self.measured_value = radians / dt;

        let error = self.target_value - self.measured_value;
        let proportional = error; // Proportional term
        self.integral += error * dt; // Integral term
        if self.integral.is_nan() {
            self.integral = 0.0;
        }

        // Windup guard
        // 80 RPM -> 0.00838 rad/ms
        self.integral = self.integral.clamp(-10.00, 10.00);

        let derivative = (self.previous_value - self.measured_value) / dt; // Derivative term
        self.previous_value = self.measured_value;

        self.output += (K_P * proportional) + (K_I * self.integral) + (K_D * derivative);
        self.output = self.output.clamp(-(TOP_CLOCK as f32), TOP_CLOCK as f32);
        if self.output.is_nan() {
            self.output = 0.0;
        }
        self.output
    }
}

/// Motor driver task runs a loop that reads the encoder
/// value and updates the motor driver
pub async fn motor_driver<'a, D>(
    pwm_signal: &'a PwmSignal,
    driver: &'a DriverMutex,
    direction: D,
    mut encoder: Encoder<'a>,
) -> !
where
    D: Into<AnyPin>,
{
    defmt::info!("Starting motor driver");
    log::info!("Starting motor driver");
    let mut direction = Output::new(direction.into(), Level::Low);
    let mut last_update = Instant::now();
    let mut ticker = Ticker::every(Duration::from_hz(FREQUENCY));
    let mut rng = SmallRng::seed_from_u64(10043);
    let mut i = 0;
    loop {
        let start_time = Instant::now();
        let value = encoder.read_and_reset().await;
        let value = value + rng.gen_range(-4.0..4.0);
        // make sure the time step doesn't become too long
        // reading from encoder may stall when not moving.
        let elapsed = last_update.elapsed();
        let control = driver.lock().await.update(value, elapsed);

        // Use a threshold to set encoder direction to avoid oscillation
        // when switching directions
        if control > 2000.0 {
            direction.set_low();
            encoder.set_direction(Direction::Forward).await;
        } else if control < -2000.0 {
            direction.set_high();
            encoder.set_direction(Direction::Backward).await;
        } else {
            encoder.set_direction(Direction::None).await;
        }

        pwm_signal.signal(control.abs().min(u16::MAX as f32) as u16);

        last_update = Instant::now();
        let end_time = Instant::now();
        let elapsed = (end_time - start_time).as_micros();
        i = (i + 1) % FREQUENCY;
        if i == 0 {
            defmt::info!("Motor driver loop took: {:?} us", elapsed);
        }

        // use Timer instead of Ticker so time steps remain constant
        // even if the loop takes longer due to encoder stall
        ticker.next().await;
    }
}
