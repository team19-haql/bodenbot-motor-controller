#![no_std]
#![no_main]

use bodenbot_motor_controller as motor_control;
use embassy_executor::Spawner;
use embassy_time::Timer;
use fixed::traits::ToFixed;
use motor_control::join;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let motors = [
        &motor_control::motor::MOTOR0_DRIVER,
        &motor_control::motor::MOTOR1_DRIVER,
        &motor_control::motor::MOTOR2_DRIVER,
        &motor_control::motor::MOTOR3_DRIVER,
        &motor_control::motor::MOTOR4_DRIVER,
        &motor_control::motor::MOTOR5_DRIVER,
    ];

    join!(motor_control::motor_control(&spawner), async {
        loop {
            for (i, motor) in motors.iter().enumerate() {
                log::info!(
                    "motor{}: {}",
                    i,
                    motor.lock().await.get_measure_value().to_num::<f32>()
                );
                motor.lock().await.set_target(1.to_fixed());
            }
            Timer::after_millis(500).await;
        }
    },)
    .await;
}
