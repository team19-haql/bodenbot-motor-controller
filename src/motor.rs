use crate::encoder::{spawn_encoder, Direction, Fixed};
use crate::pwm::PwmSignal;
use crate::utils::Mutex;
use embassy_rp::gpio::{AnyPin, Output};
use embassy_time::{Duration, Instant, Ticker};
use fixed_macro::fixed;

const K_P: Fixed = fixed!(8.0: I16F16);
const K_I: Fixed = fixed!(0.0: I16F16);
const K_D: Fixed = fixed!(0.0: I16F16);

pub type DriverMutex = Mutex<Driver>;

pub static MOTOR0_DRIVER: DriverMutex = DriverMutex::new(Driver::new());
pub static MOTOR1_DRIVER: DriverMutex = DriverMutex::new(Driver::new());
pub static MOTOR2_DRIVER: DriverMutex = DriverMutex::new(Driver::new());
pub static MOTOR3_DRIVER: DriverMutex = DriverMutex::new(Driver::new());
pub static MOTOR4_DRIVER: DriverMutex = DriverMutex::new(Driver::new());
pub static MOTOR5_DRIVER: DriverMutex = DriverMutex::new(Driver::new());

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

    pub fn set_target(&mut self, target: Fixed) {
        self.target_value = target;
    }

    pub fn get_target(&mut self) -> Fixed {
        self.target_value
    }

    pub fn get_measure_value(&self) -> Fixed {
        self.measured_value
    }

    pub fn update(&mut self, rotations: Fixed, delta: Duration) -> Fixed {
        let dt = Fixed::from_num(delta.as_micros() as i32) / 1000000; // should not be longer than a few milliseconds
        if dt == 0 {
            return self.output;
        }
        let angular_velocity = rotations.saturating_div(dt);
        self.measured_value = angular_velocity;

        let error = self.target_value - angular_velocity;
        let proportional = error; // Proportional term
        self.integral += error.saturating_mul(dt); // Integral term

        // Windup guard
        // 80 RPM -> 0.00838 rad/ms
        self.integral = self
            .integral
            .clamp(-fixed!(0.05: I16F16), fixed!(0.05: I16F16));

        let derivative = (self.previous_value - angular_velocity) / dt; // Derivative term
        self.previous_value = angular_velocity;
        self.output += K_P.saturating_mul_add(
            proportional,
            K_I.saturating_mul_add(self.integral, K_D.saturating_mul(derivative)),
        );
        self.output
    }
}

pub async fn motor_driver<'a, P>(
    pwm_signal: &'a PwmSignal,
    driver: &'a DriverMutex,
    mut direction: Output<'a>,
    clk_pin: P,
) -> !
where
    P: Into<AnyPin>,
{
    let encoder = spawn_encoder(clk_pin).await;
    let mut ticker = Ticker::every(Duration::from_hz(100));
    let mut last_update = Instant::now();
    loop {
        let value = encoder.lock().await.read_reset();
        let control = driver.lock().await.update(value, last_update.elapsed());
        let control = control
            .saturating_mul(Fixed::from_num(1 << 8))
            .to_num::<i32>();

        let _target = driver
            .lock()
            .await
            .get_target()
            .saturating_mul(Fixed::from_num(1 << 8))
            .to_num::<i32>();

        if control > 2000 {
            direction.set_low();
            encoder.lock().await.set_direction(Direction::Forward);
        } else if control < -2000 {
            direction.set_high();
            encoder.lock().await.set_direction(Direction::Backward);
        } else {
            encoder.lock().await.set_direction(Direction::None);
        }

        pwm_signal.signal(control.unsigned_abs() as u16);

        last_update = Instant::now();
        ticker.next().await;
    }
}
