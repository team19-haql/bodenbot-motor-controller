use defmt::*;

use embassy_rp::gpio::Input;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

pub use fixed::types::I16F16 as Fixed;

const RATIO: i32 = 100;
const PPR: i32 = 16 * RATIO;

#[allow(dead_code)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Backward,
    None,
}

pub struct Encoder {
    direction: Direction,
    pulses: i32,
}

pub type EncoderMutex = Mutex<ThreadModeRawMutex, Encoder>;

#[allow(dead_code)]
impl Encoder {
    pub const fn new() -> Self {
        Encoder {
            direction: Direction::None,
            pulses: 0,
        }
    }
    pub fn read(&self) -> Fixed {
        Fixed::from_num(self.pulses) / PPR
    }
    pub fn read_reset(&mut self) -> Fixed {
        let rot = self.read();
        self.reset();
        rot
    }

    pub fn reset(&mut self) {
        self.pulses = 0;
    }
    pub fn set_direction(&mut self, direction: Direction) {
        self.direction = direction;
    }
    pub fn direction(&self) -> Direction {
        self.direction
    }

    pub fn update(&mut self) {
        match self.direction {
            Direction::Forward => self.pulses += 1,
            Direction::Backward => self.pulses -= 1,
            Direction::None => (),
        }
    }
}

#[embassy_executor::task(pool_size = 6)]
pub async fn encoder_task(encoder: &'static EncoderMutex, mut clk_pin: Input<'static>) {
    info!("Starting encoder task");

    loop {
        clk_pin.wait_for_rising_edge().await;

        encoder.lock().await.update()
    }
}
