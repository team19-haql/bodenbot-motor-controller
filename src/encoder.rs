use embassy_rp::gpio::Pull;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::{bind_interrupts, pio};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use pio::{Common, Config, Instance, InterruptHandler, Pio, PioPin, StateMachine};

pub use fixed::types::I16F16 as Fixed;

const RATIO: i32 = 100;
const PPR: i32 = 16 * RATIO;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

struct PioEncoder<'d, T: Instance, const SM: usize> {
    sm: StateMachine<'d, T, SM>,
}

impl<'d, T: Instance, const SM: usize> PioEncoder<'d, T, SM> {
    fn new(
        pio: &mut Common<'d, T>,
        mut sm: StateMachine<'d, T, SM>,
        pulse_pin: impl PioPin,
    ) -> Self {
        #[rustfmt::skip]
        let pio_program = pio_proc::pio_asm!(
            "start:",
            "  set y 0",
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

        cfg.use_program(&pio.load_program(&pio_program.program), &[]);
        sm.set_config(&cfg);
        sm.set_enable(true);
        Self { sm }
    }

    async fn read(&mut self) -> i32 {
        // signal a read
        self.sm.tx().push(5);

        // wait for the read to complete
        self.sm.rx().wait_pull().await as i32
    }
}

#[allow(dead_code)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Backward,
    None,
}

enum PioEncoderMachine<'d> {
    P0SM0(PioEncoder<'d, PIO0, 0>),
    P0SM1(PioEncoder<'d, PIO0, 1>),
    P0SM2(PioEncoder<'d, PIO0, 2>),
    P0SM3(PioEncoder<'d, PIO0, 3>),
    P1SM0(PioEncoder<'d, PIO1, 0>),
    P1SM1(PioEncoder<'d, PIO1, 1>),
    P1SM2(PioEncoder<'d, PIO1, 2>),
    P1SM3(PioEncoder<'d, PIO1, 3>),
}

impl PioEncoderMachine<'_> {
    pub async fn read(&mut self) -> i32 {
        match self {
            PioEncoderMachine::P0SM0(sm) => sm.read().await,
            PioEncoderMachine::P0SM1(sm) => sm.read().await,
            PioEncoderMachine::P0SM2(sm) => sm.read().await,
            PioEncoderMachine::P0SM3(sm) => sm.read().await,
            PioEncoderMachine::P1SM0(sm) => sm.read().await,
            PioEncoderMachine::P1SM1(sm) => sm.read().await,
            PioEncoderMachine::P1SM2(sm) => sm.read().await,
            PioEncoderMachine::P1SM3(sm) => sm.read().await,
        }
    }
}

pub struct Encoder<'d> {
    direction: Direction,
    pulses: i32,
    pio: PioEncoderMachine<'d>,
}

#[allow(dead_code)]
impl<'d> Encoder<'d> {
    const fn new(pio: PioEncoderMachine<'d>) -> Self {
        Self {
            direction: Direction::None,
            pulses: 0,
            pio,
        }
    }
    pub async fn read(&mut self) -> Fixed {
        self.update().await;
        Fixed::from_num(self.pulses) * (Fixed::PI * 2 / PPR)
    }
    pub async fn read_reset(&mut self) -> Fixed {
        let rot = self.read().await;
        self.reset();
        rot
    }

    pub fn reset(&mut self) {
        self.pulses = 0;
    }
    pub async fn set_direction(&mut self, direction: Direction) {
        self.update().await;
        self.direction = direction;
    }
    pub fn direction(&self) -> Direction {
        self.direction
    }

    pub async fn update(&mut self) {
        let pulses = self.pio.read().await;
        match self.direction {
            Direction::Forward => self.pulses += pulses,
            Direction::Backward => self.pulses -= pulses,
            Direction::None => (),
        }
    }
}

pub async fn spawn_encoder<'d>(clk_pin: impl PioPin) -> Encoder<'d> {
    defmt::info!("Starting encoder task");

    static ENCODER_COUNT: Mutex<CriticalSectionRawMutex, u32> = Mutex::new(0);

    macro_rules! pio_encoder {
        ($machine:ident: ($pio:ident, $sm:ident)) => {{
            let p = unsafe { embassy_rp::peripherals::$pio::steal() };
            let Pio {
                mut common, $sm, ..
            } = Pio::new(p, Irqs);

            let pio_encoder = PioEncoder::new(&mut common, $sm, clk_pin);
            PioEncoderMachine::$machine(pio_encoder)
        }};
    }

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
