use embassy_executor::Spawner;
use embassy_rp::gpio::{AnyPin, Input, Pull};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use heapless::Vec;

pub use fixed::types::I16F16 as Fixed;

const RATIO: i32 = 100;
const PPR: i32 = 16 * RATIO;

#[allow(dead_code)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Backward,
    None,
}

#[derive(Debug)]
pub struct Encoder {
    direction: Direction,
    pulses: i32,
}

pub type EncoderMutex = Mutex<CriticalSectionRawMutex, Encoder>;

#[allow(dead_code)]
impl Encoder {
    pub const fn new() -> Self {
        Encoder {
            direction: Direction::None,
            pulses: 0,
        }
    }
    pub fn read(&self) -> Fixed {
        Fixed::from_num(self.pulses) * (Fixed::PI * 2 / PPR)
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
async fn encoder_task(encoder: &'static EncoderMutex, mut clk_pin: Input<'static>) {
    defmt::info!("Starting encoder task");

    loop {
        clk_pin.wait_for_rising_edge().await;

        encoder.lock().await.update()
    }
}

pub async fn spawn_encoder<P>(clk_pin: P) -> &'static EncoderMutex
where
    P: Into<AnyPin>,
{
    let clk_pin = Input::new(clk_pin.into(), Pull::None);
    static mut ENCODERS: Vec<EncoderMutex, 6> = Vec::new();

    let encoders = unsafe { &mut ENCODERS };

    if encoders.push(EncoderMutex::new(Encoder::new())).is_err() {
        defmt::panic!("Too many encoders");
    }

    let id = encoders.len() - 1;
    // crate::EXECUTOR_MED
    //     .spawner()
    Spawner::for_current_executor()
        .await
        .must_spawn(encoder_task(&encoders[id], clk_pin));
    &encoders[id]
}
