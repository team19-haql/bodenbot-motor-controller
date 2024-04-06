//! We use the PIO for the encoder to allow high throughput without
//! interrupting the CPU. The PIO state machine is configured to
//! count the encoder pin pulses until a signal is sent to push the
//! pulse count to the RX FIFO which is read by the CPU.
use embassy_rp::gpio::Pull;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::{bind_interrupts, pio};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use fixed::traits::ToFixed;
use pio::{Common, Config, Instance, InterruptHandler, Pio, PioPin, StateMachine};

pub use fixed::types::I16F16 as Fixed;

/// The gear ratio of the bodenbot motors
const RATIO: i32 = 100;

/// The pulses per revolution of the encoder
const PPR: i32 = 16 * RATIO;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

/// The internal encoder PIO state machine
struct PioEncoderInner<'d, T: Instance, const SM: usize> {
    sm: StateMachine<'d, T, SM>,
}

/// The encoder direction
#[allow(dead_code)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Backward,
    None,
}

/// A wrapper for the PIO encoder state machine to allow
/// multiple generic instances of the PIO state machine to be
/// used in a non generic function.
enum PioEncoder<'d> {
    P0SM0(PioEncoderInner<'d, PIO0, 0>),
    P0SM1(PioEncoderInner<'d, PIO0, 1>),
    P0SM2(PioEncoderInner<'d, PIO0, 2>),
    P0SM3(PioEncoderInner<'d, PIO0, 3>),
    P1SM0(PioEncoderInner<'d, PIO1, 0>),
    P1SM1(PioEncoderInner<'d, PIO1, 1>),
    P1SM2(PioEncoderInner<'d, PIO1, 2>),
    P1SM3(PioEncoderInner<'d, PIO1, 3>),
}

/// The public interface for reading the encoder..
pub struct Encoder<'d> {
    direction: Direction,
    pulses: i32,
    pio: PioEncoder<'d>,
}

impl<'d, T: Instance, const SM: usize> PioEncoderInner<'d, T, SM> {
    fn new(
        pio: &mut Common<'d, T>,
        mut sm: StateMachine<'d, T, SM>,
        pulse_pin: impl PioPin,
    ) -> Self {
        // The PIO program is written in PIO assembly
        // it decrements the y register using a jmp instuction
        // since that is the only PIO instruction that allows
        // arithmetic operations. The value is negated before pushing
        // to the RX FIFO to fix the negative value.
        //
        // x must be set to 0 before `pull noblock` since that instruction
        // defaults to reading the x register if there is nothing in the TX FIFO.
        //
        // if x is not zero after the pull instruction, that signals that a read command
        // occured and the pulse count is pushed to the CPU.
        //
        // the arithmetic logic is based on x + y = ~(~x - y)
        #[rustfmt::skip]
        let pio_program = pio_proc::pio_asm!(
            "start:",
            "  set y 0",
            "  mov y ~y", // negate y so counter is accurate
            "loop:",
            "  wait 0 pin 0 [8]",
            "  wait 1 pin 0 [8]",
            "  jmp y-- test",
            "test:",
            "  set x 0",
            "  pull noblock",
            "  out x 32",
            "  jmp !x loop",
            "output:",
            "  mov isr ~y",
            "  push",
        );

        let mut cfg = Config::default();

        let mut pulse_pin = pio.make_pio_pin(pulse_pin);
        pulse_pin.set_pull(Pull::None);
        sm.set_pin_dirs(pio::Direction::In, &[&pulse_pin]);
        cfg.set_in_pins(&[&pulse_pin]);
        // limit clock speed to avoid reading jitter
        cfg.clock_divider = 100.to_fixed();

        cfg.use_program(&pio.load_program(&pio_program.program), &[]);
        sm.set_config(&cfg);
        sm.set_enable(true);
        Self { sm }
    }

    async fn read(&mut self) -> i32 {
        if let Some(value) = self.sm.rx().try_pull() {
            // it is possible that the rx register has unread values.
            // we will just read that since we read several hundred times
            // per second, so it should be fine.
            value as i32
        } else {
            // signal a read to the PIO state machine
            if self.sm.tx().empty() {
                self.sm.tx().push(5);
            } else {
                defmt::warn!("PIO TX FIFO full");
                log::warn!("PIO TX FIFO full");
            }

            // wait for the read to complete
            Timer::after_micros(1).await;
            // don't wait for the pull because it could stall
            self.sm.rx().try_pull().unwrap_or(0) as i32
        }
    }
}

impl PioEncoder<'_> {
    pub async fn read(&mut self) -> i32 {
        match self {
            PioEncoder::P0SM0(sm) => sm.read().await,
            PioEncoder::P0SM1(sm) => sm.read().await,
            PioEncoder::P0SM2(sm) => sm.read().await,
            PioEncoder::P0SM3(sm) => sm.read().await,
            PioEncoder::P1SM0(sm) => sm.read().await,
            PioEncoder::P1SM1(sm) => sm.read().await,
            PioEncoder::P1SM2(sm) => sm.read().await,
            PioEncoder::P1SM3(sm) => sm.read().await,
        }
    }
}

impl<'d> Encoder<'d> {
    const fn new(pio: PioEncoder<'d>) -> Self {
        Self {
            direction: Direction::None,
            pulses: 0,
            pio,
        }
    }

    /// Read the encoder angle in radians
    pub async fn read(&mut self) -> Fixed {
        self.update().await;
        Fixed::from_num(self.pulses) * (Fixed::PI * 2 / PPR)
    }

    /// Read the encoder angle in radians then reset the pulse count
    pub async fn read_reset(&mut self) -> Fixed {
        let rot = self.read().await;
        self.reset();
        rot
    }

    /// Reset the pulse count
    pub fn reset(&mut self) {
        self.pulses = 0;
    }

    /// The encoder direction needs to be manually set since
    /// we are using a single pulse encoder.
    pub async fn set_direction(&mut self, direction: Direction) {
        // Update the encoder before changing the direction so that
        // previous encoder pulses are counted correctly
        self.update().await;
        self.direction = direction;
    }

    /// Updates the interrnal pulse count depending on its set direction
    async fn update(&mut self) {
        let pulses = self.pio.read().await;
        match self.direction {
            Direction::Forward => self.pulses += pulses,
            Direction::Backward => self.pulses -= pulses,
            Direction::None => (),
        }
    }
}

/// Creates and sets up a new encoder state machine.
pub async fn spawn_encoder<'d>(clk_pin: impl PioPin) -> Encoder<'d> {
    static ENCODER_COUNT: Mutex<CriticalSectionRawMutex, u32> = Mutex::new(0);

    // This macro creates a PIO encoder state machine and wraps it in
    // an enum for the proper encoder.
    macro_rules! pio_encoder {
        ($machine:ident: ($pio:ident, $sm:ident)) => {{
            let p = unsafe { embassy_rp::peripherals::$pio::steal() };
            let Pio {
                mut common, $sm, ..
            } = Pio::new(p, Irqs);

            let pio_encoder = PioEncoderInner::new(&mut common, $sm, clk_pin);
            defmt::info!("Starting encoder {}", stringify!($machine));
            log::info!("Starting encoder {}", stringify!($machine));

            PioEncoder::$machine(pio_encoder)
        }};
    }

    // each time this function is called a new PIO machine has
    // to be used. The `ENCODER_COUNT` is keeping track of
    // how many have been created
    let machine = match *ENCODER_COUNT.lock().await {
        0 => pio_encoder!(P0SM0: (PIO0, sm0)),
        1 => pio_encoder!(P0SM1: (PIO0, sm1)),
        2 => pio_encoder!(P0SM2: (PIO0, sm2)),
        3 => pio_encoder!(P0SM3: (PIO0, sm3)),
        4 => pio_encoder!(P1SM0: (PIO1, sm0)),
        5 => pio_encoder!(P1SM1: (PIO1, sm1)),
        6 => pio_encoder!(P1SM2: (PIO1, sm2)),
        7 => pio_encoder!(P1SM3: (PIO1, sm3)),
        _ => panic!("Exceeded the number of supported encoders"),
    };

    *ENCODER_COUNT.lock().await += 1;

    let encoder = Encoder::new(machine);
    encoder
}
