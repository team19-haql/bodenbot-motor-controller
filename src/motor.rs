//! The motor encoder spawns a new motor task loop. The task
//! will manage the pid motor controller for the given pins.
//!
//! The current state of the driver such as target value and
//! measured value can be read over i2c.
use crate::encoder::{Direction, Encoder, Fixed};
use crate::pwm::PwmSignal;
use crate::utils::Mutex;
use embassy_rp::gpio::{AnyPin, Level, Output};
use embassy_time::{Duration, Instant, Timer};
use fixed_macro::fixed;

/// Proportional PID controller constant
const K_P: Fixed = fixed!(8.0: I16F16);
/// Integral PID controller constant
const K_I: Fixed = fixed!(0.0: I16F16);
/// Derivative PID controller constant
const K_D: Fixed = fixed!(0.0: I16F16);

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
    target_value: Fixed,
    previous_value: Fixed,
    integral: Fixed,
    output: Fixed,
    measured_value: Fixed,
}

impl Driver {
    pub const fn new() -> Self {
        Self {
            target_value: fixed!(0: I16F16),
            previous_value: fixed!(0: I16F16),
            integral: fixed!(0: I16F16),
            output: fixed!(0: I16F16),
            measured_value: fixed!(0: I16F16),
        }
    }

    /// Read the target value for the PID controller
    pub fn set_target(&mut self, target: Fixed) {
        self.target_value = target;
    }

    /// Read the current target value for the PID controller
    pub fn get_target(&self) -> Fixed {
        self.target_value
    }

    /// Read the last measured_value for the PID controller
    pub fn get_measure_value(&self) -> Fixed {
        self.measured_value
    }

    /// Update the PID controller with a new measured_value and time delta.
    pub fn update(&mut self, rotations: Fixed, delta: Duration) -> Fixed {
        // should not be longer than a few milliseconds
        // Divide by 1000000 to convert micros to seconds
        let dt = Fixed::from_num(delta.as_micros() as i32) / 1000000;
        if dt == 0 {
            return self.output;
        }
        let angular_velocity = rotations.saturating_div(dt);
        self.measured_value = angular_velocity;

        let error = self.target_value - angular_velocity;
        let proportional = error; // Proportional term
        self.integral = error.saturating_mul_add(dt, self.integral); // Integral term

        // Windup guard
        // 80 RPM -> 0.00838 rad/ms
        self.integral = self
            .integral
            .clamp(-fixed!(0.05: I16F16), fixed!(0.05: I16F16));

        let derivative = (self.previous_value - angular_velocity) / dt; // Derivative term
        self.previous_value = angular_velocity;
        self.output = K_P
            .saturating_mul(proportional)
            .saturating_add(K_I.saturating_mul(self.integral))
            .saturating_add(K_D.saturating_mul(derivative))
            .saturating_add(self.output);
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
    loop {
        let start_time = Instant::now();
        let value = encoder.read_and_reset().await;
        // make sure the time step doesn't become too long
        // reading from encoder may stall when not moving.
        let elapsed = last_update.elapsed().min(Duration::from_millis(10));
        let control = driver.lock().await.update(value, elapsed);
        let control = control
            .saturating_mul(Fixed::from_num(1 << 8))
            .to_num::<i32>()
            .saturating_mul(8);

        // Use a threshold to set encoder direction to avoid oscillation
        // when switching directions
        if control > 2000 {
            direction.set_low();
            encoder.set_direction(Direction::Forward).await;
        } else if control < -2000 {
            direction.set_high();
            encoder.set_direction(Direction::Backward).await;
        } else {
            encoder.set_direction(Direction::None).await;
        }

        pwm_signal.signal(control.unsigned_abs() as u16);

        last_update = Instant::now();
        let end_time = Instant::now();
        let elapsed = (end_time - start_time).as_micros();
        defmt::info!("Motor driver loop took: {:?} us", elapsed);

        // use Timer instead of Ticker so time steps remain constant
        // even if the loop takes longer due to encoder stall
        Timer::after(Duration::from_hz(10)).await;
    }
}
