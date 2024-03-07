use crate::encoder::Fixed;

use embassy_time::Duration;
use fixed_macro::fixed;

const K_P: Fixed = fixed!(0.5: I16F16);
const K_I: Fixed = fixed!(0.0: I16F16);
const K_D: Fixed = fixed!(0.0: I16F16);

pub struct Pid {
    target_value: Fixed,
    previous_value: Fixed,
    integral: Fixed,
    output: Fixed,
    measured_value: Fixed,
}

impl Pid {
    pub fn new() -> Self {
        Self {
            target_value: Fixed::from_num(0),
            previous_value: Fixed::from_num(0),
            integral: Fixed::from_num(0),
            output: Fixed::from_num(0),
            measured_value: Fixed::from_num(0),
        }
    }

    pub fn set_target(&mut self, target: Fixed) {
        self.target_value = target;
    }

    pub fn get_measure_value(&self) -> Fixed {
        self.measured_value
    }

    pub fn update(&mut self, rotations: Fixed, delta: Duration) -> Fixed {
        let dt = delta.as_micros() as i32; // should not be longer than a few milliseconds
        let angular_velocity = rotations / dt;
        self.measured_value = angular_velocity;

        let error = self.target_value - angular_velocity;
        let proportional = error; // Proportional term
        self.integral += error * dt; // Integral term

        // Windup guard
        // 80 RPM -> 0.00838 rad/ms
        self.integral = self
            .integral
            .clamp(-fixed!(0.05: I16F16), fixed!(0.05: I16F16));

        let derivative = (self.previous_value - angular_velocity) / dt; // Derivative term
        self.previous_value = angular_velocity;
        self.output += K_P * proportional + K_I * self.integral + K_D * derivative;
        self.output
    }
}
