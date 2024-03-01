use hal::digital::v2::InputPin;

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Backward,
    None,
}

const PIN_EDGE: u8 = 0b01;
const PIN_MASK: u8 = 0b11;

pub struct Encoder<CLK> {
    pin_clk: CLK,
    direction: Direction,
    rotation: i32,
    state: u8,
}

impl<CLK> Encoder<CLK>
where
    CLK: InputPin,
{
    pub fn new(pin: CLK) -> Self {
        Encoder {
            pin_clk: pin,
            direction: Direction::None,
            state: 0,
            rotation: 0,
        }
    }
    pub fn read(&self) -> i32 {
        self.rotation
    }
    pub fn read_reset(&mut self) -> i32 {
        let rot = self.rotation;
        self.reset();
        rot
    }

    pub fn pin_mut(&mut self) -> &mut CLK {
        &mut self.pin_clk
    }

    pub fn release(self) -> CLK {
        self.pin_clk
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
        self.state = (self.state << 1) | self.pin_clk.is_high().unwrap_or_default() as u8;

        if (self.state & PIN_MASK) == PIN_EDGE {
            match self.direction {
                Direction::Forward => self.rotation += 1,
                Direction::Backward => self.rotation += 1,
                Direction::None => (),
            }
        }
    }
}
