//! This example shows how to use USB (Universal Serial Bus) in the RP2040 chip.
//!
//! This creates a USB serial port that echos.

use crate::encoder::Fixed;
use crate::i2c::registers;
use crate::join;
use crate::motor;
use crate::motor::DriverMutex;
use crate::pwm;
use core::fmt::Write;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, Instance, InterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use fixed::traits::ToFixed;
use heapless::Vec;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::task]
pub async fn serial_motor_task(usb: USB) {
    defmt::info!("Starting serial motor interface!");

    // Create the driver, from the HAL.
    let driver = Driver::new(usb, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("HAQL ~ Team 19");
    config.product = Some("bodenbot_driver");
    config.serial_number = Some("2205195");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        loop {
            defmt::info!("Waiting for connection");
            class.wait_connection().await;
            defmt::info!("Connected");
            let _ = motor_control(&mut class).await;
            defmt::info!("Disconnected");
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join!(usb_fut, echo_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

fn read_motor(motor: &DriverMutex) -> Fixed {
    let value = if let Ok(m) = motor.try_lock() {
        m.get_measure_value()
        // m.get_target().to_le_bytes()
    } else {
        0.to_fixed()
    };
    value
}

async fn write_motor(motor: &DriverMutex, value: Fixed) {
    defmt::info!("Setting motor target: {}", value.to_num::<f32>());
    motor.lock().await.set_target(value);
}

async fn handle_command<'d, T: Instance + 'd>(
    data: &[u8],
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), &'static str> {
    defmt::info!(
        "received command: {:?}",
        core::str::from_utf8(data).unwrap_or("bad data")
    );
    let mut command = data.split(|b| *b == b' ');

    match (command.next(), command.next(), command.next()) {
        (Some(b"write"), Some(b"all"), Some(value)) => {
            let value: Fixed = core::str::from_utf8(value)
                .map_err(|_| "parse value error")?
                .parse()
                .map_err(|_| "parse vlaue error")?;

            write_motor(&motor::MOTOR0, value).await;
            write_motor(&motor::MOTOR1, value).await;
            write_motor(&motor::MOTOR2, value).await;
            write_motor(&motor::MOTOR3, value).await;
            write_motor(&motor::MOTOR4, value).await;
            write_motor(&motor::MOTOR5, value).await;

            class
                .write_packet(b"OK\n")
                .await
                .map_err(|_| "write error")?;
        }
        (Some(b"write"), Some(reg), Some(value)) => {
            let reg_num = core::str::from_utf8(reg)
                .map_err(|_| "parse reg error")?
                .parse()
                .map_err(|_| "parse reg error")?;
            let value: Fixed = core::str::from_utf8(value)
                .map_err(|_| "parse value error")?
                .parse()
                .map_err(|_| "parse vlaue error")?;

            match reg_num {
                registers::MOTOR0 => write_motor(&motor::MOTOR0, value).await,
                registers::MOTOR1 => write_motor(&motor::MOTOR1, value).await,
                registers::MOTOR2 => write_motor(&motor::MOTOR2, value).await,
                registers::MOTOR3 => write_motor(&motor::MOTOR3, value).await,
                registers::MOTOR4 => write_motor(&motor::MOTOR4, value).await,
                registers::MOTOR5 => write_motor(&motor::MOTOR5, value).await,
                registers::LED0 => pwm::LED0_PWM.signal(value.to_num::<u16>()),
                registers::FAN0 => pwm::FAN0_PWM.signal(value.to_num::<u16>()),
                registers::FAN1 => pwm::FAN1_PWM.signal(value.to_num::<u16>()),
                _ => {
                    defmt::info!("bad register number: {}", reg_num);
                    return Err("bad register number");
                }
            }

            class
                .write_packet(b"OK\n")
                .await
                .map_err(|_| "write error")?;
        }
        (Some(b"read"), Some(reg), _) => {
            let reg_num = core::str::from_utf8(reg)
                .map_err(|_| "parse reg error")?
                .parse()
                .map_err(|_| "parse reg error")?;

            let value = match reg_num {
                registers::MOTOR0 => read_motor(&motor::MOTOR0),
                registers::MOTOR1 => read_motor(&motor::MOTOR1),
                registers::MOTOR2 => read_motor(&motor::MOTOR2),
                registers::MOTOR3 => read_motor(&motor::MOTOR3),
                registers::MOTOR4 => read_motor(&motor::MOTOR4),
                registers::MOTOR5 => read_motor(&motor::MOTOR5),
                _ => {
                    defmt::info!("bad register number: {}", reg_num);
                    return Err("bad register number");
                }
            };

            let mut write_buffer: Vec<u8, 256> = Vec::new();
            writeln!(&mut write_buffer, "{}\n", value).map_err(|_| "failed to write to buffer")?;

            class
                .write_packet(&write_buffer)
                .await
                .map_err(|_| "failed to send packet")?;
        }

        _ => {
            defmt::info!("bad command: {:x}", data);
            return Err("bad command");
        }
    }

    Ok(())
}

async fn motor_control<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    let mut command_buffer = [0; 256];
    let mut position = 0;
    loop {
        let n = class.read_packet(&mut buf).await?;

        let mut newline = false;

        let data = &mut buf[..n];

        for c in data.iter_mut() {
            if *c == b'\r' {
                *c = b'\n';
            }
            if *c == b'\n' {
                newline = true;
            } else {
                command_buffer[position] = *c;
                position += 1;
            }
        }

        defmt::info!(
            "received: {:?}",
            core::str::from_utf8(data).unwrap_or("bad data")
        );

        class.write_packet(data).await?;

        if newline {
            if let Err(e) = handle_command(&command_buffer[..position], class).await {
                class.write_packet(b"error: ").await?;
                class.write_packet(e.as_bytes()).await?;
                class.write_packet(b"\n").await?;
                defmt::info!("failed to parse command");
            }
            position = 0;
        }
    }
}
