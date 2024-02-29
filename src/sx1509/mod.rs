#![allow(dead_code)]

mod pin;
mod register;

use core::cell::RefCell;
use core::marker::PhantomData;
use hal::blocking::i2c::{Write, WriteRead};
use register::Register;

use pin::{Board, Pin};

pub const DEFAULT_ADDRESS: u8 = 0x3a;

pub struct Sx1509<'a, I2C> {
    i2c: &'a RefCell<I2C>,
    address: u8,
}

pub struct Pins<'a, B> {
    pub gpio0: Pin<'a, pin::Offline, B>,
    pub gpio1: Pin<'a, pin::Offline, B>,
    pub gpio2: Pin<'a, pin::Offline, B>,
    pub gpio3: Pin<'a, pin::Offline, B>,
    pub gpio4: Pin<'a, pin::Offline, B>,
    pub gpio5: Pin<'a, pin::Offline, B>,
    pub gpio6: Pin<'a, pin::Offline, B>,
    pub gpio7: Pin<'a, pin::Offline, B>,
    pub gpio8: Pin<'a, pin::Offline, B>,
    pub gpio9: Pin<'a, pin::Offline, B>,
    pub gpio10: Pin<'a, pin::Offline, B>,
    pub gpio11: Pin<'a, pin::Offline, B>,
    pub gpio12: Pin<'a, pin::Offline, B>,
    pub gpio13: Pin<'a, pin::Offline, B>,
    pub gpio14: Pin<'a, pin::Offline, B>,
    pub gpio15: Pin<'a, pin::Offline, B>,
}

impl<'a, I2C, E> Sx1509<'a, I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: &RefCell<I2C>, address: u8) -> Result<Sx1509<I2C>, E> {
        let board = Sx1509 { i2c, address };

        // set clock to default value
        // 0b10 is internal 2Mz clock
        let source = 0b10 << 5;
        let pin_function = 0;
        let freq_out = 0;
        let reg_clock = source | pin_function | freq_out;
        board.write_u8(Register::RegClock, reg_clock)?;

        let divider = 1 << 4;
        let mut reg_misc = board.read_u8(Register::RegMisc)?;
        reg_misc &= !(0b111 << 4);
        reg_misc |= divider;
        board.write_u8(Register::RegMisc, reg_misc)?;

        Ok(board)
    }

    pub fn software_reset(&mut self) -> Result<(), E> {
        self.write_u8(Register::RegReset, 0x12)?;
        self.write_u8(Register::RegReset, 0x34)?;
        Ok(())
    }

    pub fn split(&'a self) -> Pins<'a, Self> {
        Pins {
            gpio0: Pin {
                _p: PhantomData,
                pin: 0,
                board: self,
            },
            gpio1: Pin {
                _p: PhantomData,
                pin: 1,
                board: self,
            },
            gpio2: Pin {
                _p: PhantomData,
                pin: 2,
                board: self,
            },
            gpio3: Pin {
                _p: PhantomData,
                pin: 3,
                board: self,
            },
            gpio4: Pin {
                _p: PhantomData,
                pin: 4,
                board: self,
            },
            gpio5: Pin {
                _p: PhantomData,
                pin: 5,
                board: self,
            },
            gpio6: Pin {
                _p: PhantomData,
                pin: 6,
                board: self,
            },
            gpio7: Pin {
                _p: PhantomData,
                pin: 7,
                board: self,
            },
            gpio8: Pin {
                _p: PhantomData,
                pin: 8,
                board: self,
            },
            gpio9: Pin {
                _p: PhantomData,
                pin: 9,
                board: self,
            },
            gpio10: Pin {
                _p: PhantomData,
                pin: 10,
                board: self,
            },
            gpio11: Pin {
                _p: PhantomData,
                pin: 11,
                board: self,
            },
            gpio12: Pin {
                _p: PhantomData,
                pin: 12,
                board: self,
            },
            gpio13: Pin {
                _p: PhantomData,
                pin: 13,
                board: self,
            },
            gpio14: Pin {
                _p: PhantomData,
                pin: 14,
                board: self,
            },
            gpio15: Pin {
                _p: PhantomData,
                pin: 15,
                board: self,
            },
        }
    }
}

impl<'a, I2C, E> Board for Sx1509<'a, I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    type Error = E;

    fn write_u8(&self, register: Register, data: u8) -> Result<(), E> {
        self.i2c
            .borrow_mut()
            .write(self.address, &[register as u8, data])?;
        Ok(())
    }

    fn write_u16(&self, register: Register, data: u16) -> Result<(), E> {
        let msb = ((data >> 8) & 0xff) as u8;
        let lsb = (data & 0xff) as u8;
        self.i2c
            .borrow_mut()
            .write(self.address, &[register as u8, msb, lsb])?;
        Ok(())
    }

    fn read_u8(&self, register: Register) -> Result<u8, E> {
        let mut buffer = [0; 1];
        self.i2c
            .borrow_mut()
            .write_read(self.address, &[register as u8], &mut buffer)?;
        Ok(buffer[0])
    }

    fn read_u16(&self, register: Register) -> Result<u16, E> {
        let mut buffer = [0; 2];
        self.i2c
            .borrow_mut()
            .write_read(self.address, &[register as u8], &mut buffer)?;
        Ok(((buffer[0] as u16) << 8) | (buffer[1] as u16))
    }
}
