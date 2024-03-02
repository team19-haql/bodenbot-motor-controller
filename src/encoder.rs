use defmt::*;

use embassy_rp::gpio::Input;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

#[allow(dead_code)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Backward,
    None,
}

pub struct Encoder {
    direction: Direction,
    rotation: i16,
}

pub type EncoderMutex = Mutex<CriticalSectionRawMutex, Encoder>;

#[allow(dead_code)]
impl Encoder {
    pub const fn new() -> Self {
        Encoder {
            direction: Direction::None,
            rotation: 0,
        }
    }
    pub fn read(&self) -> i16 {
        self.rotation
    }
    pub fn read_reset(&mut self) -> i16 {
        let rot = self.rotation;
        self.reset();
        rot
    }

    pub fn reset(&mut self) {
        self.rotation = 0;
    }
    pub fn set_direction(&mut self, direction: Direction) {
        self.direction = direction;
    }
    pub fn direction(&self) -> Direction {
        self.direction
    }

    pub fn update(&mut self) {
        match self.direction {
            Direction::Forward => self.rotation += 1,
            Direction::Backward => self.rotation -= 1,
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
