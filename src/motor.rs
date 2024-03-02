use embassy_time::Duration;
use fixed::types::I24F8;
use fixed_macro::fixed;

const K_P: I24F8 = fixed!(0.5: I24F8);
const K_I: I24F8 = fixed!(0.0: I24F8);
const K_D: I24F8 = fixed!(0.0: I24F8);

type Float = I24F8;

pub struct Pid {
    setpoint: Float,
    previous_error: Float,
    integral: Float,
    output: Float,
}

pub trait MotorControl {
    fn set_target(&mut self, target: Float);
    fn update(&mut self, measured_value: Float, delta: Duration) -> Float;
}

impl Pid {
    pub fn new() -> Self {
        Self {
            setpoint: fixed!(0.0: I24F8),
            previous_error: fixed!(0.0: I24F8),
            integral: fixed!(0.0: I24F8),
            output: fixed!(0.0: I24F8),
        }
    }
}

impl MotorControl for Pid {
    fn set_target(&mut self, target: Float) {
        self.setpoint = target;
    }

    fn update(&mut self, measured_value: Float, delta: Duration) -> Float {
        let error = self.setpoint - measured_value;
        let proportional = error;
        let dt = delta.as_millis() as i32;
        self.integral += error * dt;
        let derivative = (error - self.previous_error) / dt;
        self.previous_error = error;
        self.output += K_P * proportional + K_I * self.integral + K_D * derivative;
        self.output
    }
}
