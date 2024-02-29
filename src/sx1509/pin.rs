use crate::sx1509::register::Register;
use core::marker::PhantomData;

const REG_I_ON: [Register; 16] = [
    Register::RegIOn0,
    Register::RegIOn1,
    Register::RegIOn2,
    Register::RegIOn3,
    Register::RegIOn4,
    Register::RegIOn5,
    Register::RegIOn6,
    Register::RegIOn7,
    Register::RegIOn8,
    Register::RegIOn9,
    Register::RegIOn10,
    Register::RegIOn11,
    Register::RegIOn12,
    Register::RegIOn13,
    Register::RegIOn14,
    Register::RegIOn15,
];

mod sealed {
    pub trait Sealed {}
}

pub trait PinState: sealed::Sealed {}
pub trait OutputState: sealed::Sealed {}
pub trait InputState: sealed::Sealed {}

pub struct Output<S: OutputState> {
    _p: PhantomData<S>,
}

impl<S: OutputState> PinState for Output<S> {}
impl<S: OutputState> sealed::Sealed for Output<S> {}

pub struct Input<S: InputState> {
    _p: PhantomData<S>,
}

impl<S: InputState> PinState for Input<S> {}
impl<S: InputState> sealed::Sealed for Input<S> {}

pub struct Offline;

impl PinState for Offline {}
impl sealed::Sealed for Offline {}

pub struct Analog;
impl OutputState for Analog {}
impl sealed::Sealed for Analog {}

pub struct Digital;
impl OutputState for Digital {}
impl sealed::Sealed for Digital {}

pub struct PullUp;
impl InputState for PullUp {}
impl sealed::Sealed for PullUp {}

pub struct PullDown;
impl InputState for PullDown {}
impl sealed::Sealed for PullDown {}

pub trait Board {
    type Error;

    fn read_u8(&self, register: Register) -> Result<u8, Self::Error>;
    fn read_u16(&self, register: Register) -> Result<u16, Self::Error>;
    fn write_u8(&self, register: Register, data: u8) -> Result<(), Self::Error>;
    fn write_u16(&self, register: Register, data: u16) -> Result<(), Self::Error>;
}

pub struct Pin<'a, S: PinState, B> {
    pub _p: PhantomData<S>,
    pub pin: u8,
    pub board: &'a B,
}

impl<'a, S: PinState, B, E> Pin<'a, S, B>
where
    B: Board<Error = E>,
{
    fn read_u8(&self, register: Register) -> Result<u8, E> {
        self.board.read_u8(register)
    }
    fn read_u16(&self, register: Register) -> Result<u16, E> {
        self.board.read_u16(register)
    }
    fn write_u8(&self, register: Register, data: u8) -> Result<(), E> {
        self.board.write_u8(register, data)
    }
    fn write_u16(&self, register: Register, data: u16) -> Result<(), E> {
        self.board.write_u16(register, data)
    }
    pub fn into_pull_up_input(self) -> Result<Pin<'a, Input<PullUp>, B>, E> {
        let mut temp_reg_dir = self.read_u16(Register::RegDirB)?;
        temp_reg_dir |= 1 << self.pin;
        self.write_u16(Register::RegDirB, temp_reg_dir)?;

        // set pull up
        let mut temp_pull_up = self.read_u16(Register::RegPullUpB)?;
        let mut temp_pull_down = self.read_u16(Register::RegPullDownB)?;

        temp_pull_up |= 1 << self.pin;
        temp_pull_down &= !(1 << self.pin);

        self.write_u16(Register::RegPullUpB, temp_pull_up)?;
        self.write_u16(Register::RegPullDownB, temp_pull_down)?;

        Ok(Pin {
            _p: PhantomData,
            pin: self.pin,
            board: self.board,
        })
    }

    pub fn into_pull_down_input(self) -> Result<Pin<'a, Input<PullDown>, B>, E> {
        let mut temp_reg_dir = self.read_u16(Register::RegDirB)?;
        temp_reg_dir |= 1 << self.pin;
        self.write_u16(Register::RegDirB, temp_reg_dir)?;

        // set pull down
        let mut temp_pull_up = self.read_u16(Register::RegPullUpB)?;
        let mut temp_pull_down = self.read_u16(Register::RegPullDownB)?;

        temp_pull_down |= 1 << self.pin;
        temp_pull_up &= !(1 << self.pin);

        self.write_u16(Register::RegPullUpB, temp_pull_up)?;
        self.write_u16(Register::RegPullDownB, temp_pull_down)?;
        Ok(Pin {
            _p: PhantomData,
            pin: self.pin,
            board: self.board,
        })
    }

    pub fn into_analog_output(self) -> Result<Pin<'a, Output<Analog>, B>, E> {
        // set output
        let mut temp_reg_dir = self.read_u16(Register::RegDirB)?;
        temp_reg_dir &= !(1 << self.pin);
        self.write_u16(Register::RegDirB, temp_reg_dir)?;

        // temp disable
        let mut temp_word = self.read_u16(Register::RegInputDisableB)?;
        temp_word |= 1 << self.pin;
        self.write_u16(Register::RegInputDisableB, temp_word)?;

        // disable pull up
        temp_word = self.read_u16(Register::RegInputDisableB)?;
        temp_word &= !(1 << self.pin);
        self.write_u16(Register::RegInputDisableB, temp_word)?;

        // enable oscillator
        let mut temp_byte = self.read_u8(Register::RegClock)?;
        temp_byte |= 1 << 6;
        temp_byte &= !(1 << 5);
        self.write_u8(Register::RegClock, temp_byte)?;

        // configure led driver clock (no log)
        temp_byte = self.read_u8(Register::RegMisc)?;
        temp_byte &= !(1 << 7);
        temp_byte &= !(1 << 3);
        let freq = 0x7 << 4;
        temp_byte |= freq;
        self.write_u8(Register::RegMisc, temp_byte)?;

        // enable LED driver
        temp_word = self.read_u16(Register::RegLEDDriverEnableB)?;
        temp_word |= 1 << self.pin;
        self.write_u16(Register::RegLEDDriverEnableB, temp_word)?;

        // led driver start
        temp_word = self.read_u16(Register::RegDataB)?;
        temp_word &= !(1 << self.pin);
        self.write_u16(Register::RegDataB, temp_word)?;

        Ok(Pin {
            _p: PhantomData,
            pin: self.pin,
            board: self.board,
        })
    }

    pub fn into_digital_output(self) -> Result<Pin<'a, Output<Digital>, B>, E> {
        let mut temp_reg_dir = self.read_u16(Register::RegDirB)?;
        temp_reg_dir &= !(1 << self.pin);
        self.write_u16(Register::RegDirB, temp_reg_dir)?;
        Ok(Pin {
            _p: PhantomData,
            pin: self.pin,
            board: self.board,
        })
    }
}

impl<'a, N: InputState, B, E> Pin<'a, Input<N>, B>
where
    B: Board<Error = E>,
{
    pub fn read(&self) -> Result<bool, E> {
        let data = self.read_u16(Register::RegDataB)?;
        if data & (1 << self.pin) != 0 {
            Ok(true)
        } else {
            Ok(false)
        }
    }
}

impl<'a, B, E> Pin<'a, Output<Digital>, B>
where
    B: Board<Error = E>,
{
    pub fn write(&mut self, is_high: bool) -> Result<(), E> {
        let mut data = self.read_u16(Register::RegDataB)?;
        if is_high {
            data |= 1 << self.pin;
        } else {
            data &= !(1 << self.pin);
        }
        self.write_u16(Register::RegDataB, data)?;

        Ok(())
    }
}

impl<'a, B, E> Pin<'a, Output<Analog>, B>
where
    B: Board<Error = E>,
{
    pub fn write_analog(&mut self, intensity: u8) -> Result<(), E> {
        self.write_u8(REG_I_ON[self.pin as usize], intensity)?;

        Ok(())
    }
}
