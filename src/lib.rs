#![no_std]
#![forbid(clippy::inline_asm_x86_att_syntax)]
#![deny(
    clippy::semicolon_if_nothing_returned,
    clippy::debug_assert_with_mut_call,
    clippy::float_arithmetic
)]
#![warn(clippy::cargo, clippy::pedantic, clippy::undocumented_unsafe_blocks)]
#![allow(clippy::must_use_candidate, clippy::upper_case_acronyms)]

#[cfg(feature = "writer")]
pub mod writer;

#[cfg(any(target_arch = "x86_64", target_arch = "x86"))]
pub use x86::*;
#[cfg(any(target_arch = "x86_64", target_arch = "x86"))]
mod x86 {
    use super::UartAddress;

    /// Address of the first COM port.
    /// This port is VERY likely to be at this address.
    pub const COM1: UartAddress = UartAddress::Io(0x3F8);
    /// Address of the second COM port.
    /// This port is likely to be at this address.
    pub const COM2: UartAddress = UartAddress::Io(0x2F8);
    /// Address of the third COM port.
    /// This address is configurable on some BIOS, so it is not a very reliable port address.
    pub const COM3: UartAddress = UartAddress::Io(0x3E8);
    /// Address of the fourth COM port.
    /// This address is configurable on some BIOS, so it is not a very reliable port address.
    pub const COM4: UartAddress = UartAddress::Io(0x2E8);
}

use bitflags::bitflags;
use core::marker::PhantomData;

bitflags! {
    #[repr(transparent)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct InterruptEnable: u8 {
        /// Interrupt when received data is available.
        const RECEIVED_DATA  = 1 << 0;

        /// Interrupt when the transmitter holding register is empty.
        const TRANSMIT_EMPTY = 1 << 1;

        /// Interrupt when the receiver line status register changes.
        const RECEIVE_STATUS = 1 << 2;

        /// Interrupt when the modem status reguster changes.
        const MODEM_STATUS   = 1 << 3;

        // Bits 4-7 are reserved
    }
}

bitflags! {
    #[repr(transparent)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct InterruptIdent: u8 {
        /// Indicates an interrupt is pending.
        const INTERRUPT_PENDING = 1 << 0;

        /// Indicates a parity error, overrun error, framing error, or break interrupt.
        ///
        /// Reset by reading the line status register.
        const RECV_LINE_STATUS  = 0b011 << 1;

        /// Indicates the FIFO trigger level has been reached.
        ///
        /// Reset when FIFO drops below the trigger level.
        const RECV_DATA_AVAIL   = 0b010 << 1;

        /// Indicates there's at least 1 character in the FIFO, but no character has
        /// been input to the FIFO or read from it since the last 4 character entries.
        ///
        /// Reset when reading from the receiver buffer register.
        const TIMEOUT           = 0b110 << 1;

        /// Indicates the transmitter holding register is empty.
        ///
        /// Reset by writing to the transmitter holding register or reading the interrupt identification register.
        const TX_EMPTY          = 0b001 << 1;

        /// CTS, DSR, RI, or DCD.
        ///
        /// Reset by reading the modem status register.
        const MODEM_STATUS      = 0b000 << 1;

        // Bits 4 & 5 are 0
        // Bits 6 & 7 are 1
    }
}

/// Serial port speed, measured in bauds.
#[repr(u16)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Baud {
    B115200 = 1,
    B57600 = 2,
    B38400 = 3,
    B19200 = 6,
    B9600 = 12,
    B4800 = 24,
    B2400 = 48,
    B1200 = 96,
    B300 = 384,
    B50 = 2304,
}

bitflags! {
    #[repr(transparent)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct FifoControl: u8 {
        /// Enables the FIFO queue operation.
        const ENABLE     = 1 << 0;

        /// Clears the receiving queue of data.
        const CLEAR_RX   = 1 << 1;

        /// Clears the transmitting queue of data.
        const CLEAR_TX   = 1 << 2;

        /// The FIFO queue will trigger an interrupt when it contains 1 byte.
        const INT_LVL_1  = 0b00 << 5;

        /// The FIFO queue will trigger an interrupt when it contains 4 bytes.
        const INT_LVL_4  = 0b01 << 5;

        /// The FIFO queue will trigger an interrupt when it contains 8 bytes.
        const INT_LVL_8  = 0b10 << 5;

        /// The FIFO queue will trigger an interrupt when it contains 14 bytes.
        const INT_LVL_14 = 0b11 << 5;
    }
}

bitflags! {
    #[repr(transparent)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct LineControl: u8 {
        /// Each character is composed of 5 bits.
        const BITS_5        = 0b00;

        /// Each character is composed of 6 bits.
        const BITS_6        = 0b01;

        /// Each character is composed of 7 bits.
        const BITS_7        = 0b10;

        /// Each character is composed of 8 bits.
        const BITS_8        = 0b11;

        /// Each character transmission is followed by an extra stop bit.
        const EXTRA_STOP    = 1 << 2;

        /// Enables parity checking on the receiving end of the port.
        const PARITY_ENABLE = 1 << 3;

        /// Enables using even-1s based parity; otherwise, odd-1s based parity.
        const EVEN_PARITY   = 1 << 4;

        /// If even parity is enabled, the parity bit is transmitted and checked as
        /// logic ‘0’. If odd parity is enabled, then the parity bit is transmitted
        /// and checked as ‘1’.
        const STICK_PARITY  = 1 << 5;

        /// Forces the serial output into a logic '0' break state (and triggers the
        /// interrupt, if enabled).
        const BREAK_SIGNAL  = 1 << 6;

        /// Enables access to the divisor latch registers, which allow setting the baud rate.
        const DLAB          = 1 << 7;
    }
}

bitflags! {
    #[repr(transparent)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ModemControl: u8 {
        /// Signals the chip is ready to transmit and receive data.
        const TERMINAL_READY     = 1 << 0;

        /// Request the other end of the port to send more data.
        const REQUEST_TO_SEND    = 1 << 1;

        /// In loopback mode, connected Ring Indicator (RI) signal input.
        const AUXILIARY_OUTPUT_1 = 1 << 2;

        /// In loopback mode, connected Data Carrier Detect (DCD) signal input.
        const AUXILIARY_OUTPUT_2 = 1 << 3;

        /// Enables loopback mode, in which transmitted bits are able to be read
        // on the same port. This is useful for testing chip operation.
        const LOOPBACK_MODE      = 1 << 4;
    }
}

bitflags! {
    #[repr(transparent)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct LineStatus: u8 {
        /// Indicates that the FIFO buffer has data awaiting transmission.
        const DATA_AVAILABLE  = 1 << 0;

        /// Indicates the FIFO was full when another character was transmitted.
        const OVERRUN_ERROR   = 1 << 1;

        /// Indicates the character at the top of the FIFO has failed the parity check.
        const PARITY_ERROR    = 1 << 2;

        /// Indicates the received character at the FIFO did not have a valid stop bit.
        const FRAMING_ERROR   = 1 << 3;

        /// Indicates a break condition has been reached in the current character.
        const BREAK_INDICATOR = 1 << 4;

        /// Indicates the transmitter holding register is empty (or FIFO is being used).
        const THR_EMPTY       = 1 << 5;

        /// Indicates the transmitter holding register AND shift register are empty (there is no more data).
        const THR_SHR_EMPTY   = 1 << 6;

        /// Indicates at least one parity error, framing error, or break indicators have been received.
        const IMPENDING_ERROR = 1 << 7;
    }
}

bitflags! {
    #[repr(transparent)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ModemStatus: u8 {
        /// Data Clear To Send (DCTS) indicator.
        const CLEAR_TO_SEND_CHANGED        = 1 << 0;

        /// Delta Data Set Ready (DDSR) indicator.
        const DATA_SET_READY_CHANGED       = 1 << 1;

        /// Trailing Edge of Ring Indicator (TERI) detector.
        /// The RI line has changed from low to high state.
        const TRAILING_EDGE_RING_INDICATOR = 1 << 2;

        /// Delta Data Carrier Detect (DDCD) indicator.
        const CARRIER_DETECT_CHANGE        = 1 << 3;

        /// Complement of the CTS input, or RTS in loopback mode.
        const CLEAR_TO_SEND                = 1 << 4;

        /// Complement of the DSR input, or DTR in loopback mode.
        const DATA_SET_READY               = 1 << 5;

        /// Complement of the RI input, or AUX1 in loopback mode.
        const RING_INDICATOR               = 1 << 6;

        /// Complement of the RI input, or AUX2 in loopback mode.
        const CARRIER_DETECT               = 1 << 7;
    }
}

pub enum UartAddress {
    Io(u16),
    Mmio(*mut u8),
}

#[repr(usize)]
#[allow(dead_code)]
enum ReadOffset {
    /// Receiver Holding Register
    RHR = 0x0,

    /// Interrupt Enable Register
    IER = 0x1,

    /// Interrupt Identification Register
    IIR = 0x2,

    /// Line Control Register
    LCR = 0x3,

    /// Line Status Register
    LSR = 0x5,

    /// Modem Status Register
    MSR = 0x6,
}

#[repr(usize)]
enum WriteOffset {
    /// Transmitter Holding Register
    THR = 0x0,

    /// Interrupt enable Register
    IER = 0x1,

    /// Fifo Control Register
    FCR = 0x2,

    /// Line Control Register
    LCR = 0x3,

    /// Modem Control Register
    MCR = 0x4,
}

pub trait Mode {}
pub struct Data;
impl Mode for Data {}
pub struct Configure;
impl Mode for Configure {}

pub struct Uart<M: Mode>(UartAddress, PhantomData<M>);

impl<M: Mode> Uart<M> {
    fn read(&self, offset: ReadOffset) -> u8 {
        // Safety: Constructor requires a valid base address.
        unsafe {
            match self.0 {
                UartAddress::Io(port) => {
                    let value: u8;

                    #[cfg(target_arch = "x86_64")]
                    core::arch::asm!("in al, dx", out("al") value, in("dx") port + (offset as u16), options(nostack, nomem, preserves_flags));

                    #[cfg(not(target_arch = "x86_64"))]
                    unimplemented!();

                    value
                }

                UartAddress::Mmio(ptr) => ptr.add(offset as usize).read_volatile(),
            }
        }
    }

    fn write(&mut self, offset: WriteOffset, value: u8) {
        // Safety: Constructor requires a valid base address.
        unsafe {
            match self.0 {
                UartAddress::Io(port) => {
                    #[cfg(target_arch = "x86_64")]
                    core::arch::asm!("out dx, al", in("dx") port + (offset as u16), in("al") value, options(nostack, nomem, preserves_flags));

                    #[cfg(not(target_arch = "x86_64"))]
                    unimplemented!();
                }

                UartAddress::Mmio(ptr) => ptr.add(offset as usize).write_volatile(value),
            }
        }
    }

    pub fn write_fifo_control(&mut self, fifo_control: FifoControl) {
        self.write(WriteOffset::FCR, fifo_control.bits());
    }

    pub fn read_line_control(&self) -> LineControl {
        // Safety: `self.read(...)` returns a single byte, of which `LineControl` uses all bits (so no unknown bits are possible).
        unsafe { LineControl::from_bits(self.read(ReadOffset::LCR)).unwrap_unchecked() }
    }

    pub fn write_line_control(&mut self, value: LineControl) {
        self.write(WriteOffset::LCR, value.bits());
    }

    pub fn write_modem_control(&mut self, value: ModemControl) {
        self.write(WriteOffset::MCR, value.bits());
    }

    pub fn read_line_status(&self) -> LineStatus {
        LineStatus::from_bits_truncate(self.read(ReadOffset::LSR))
    }

    pub fn read_modem_status(&self) -> ModemStatus {
        ModemStatus::from_bits_truncate(self.read(ReadOffset::MSR))
    }
}

impl Uart<Configure> {
    fn read_divisor_latch(&self) -> u16 {
        // When DLAB is enabled, RHR and IER become the LSB and MSB of the
        // divisor latch register, respectively.

        let lsb = u16::from(self.read(ReadOffset::RHR));
        let msb = u16::from(self.read(ReadOffset::IER));

        (msb << 8) | lsb
    }

    fn write_divisor_latch(&mut self, value: u16) {
        // When DLAB is enabled, THR and IER become the LSB and MSB of the
        // divisor latch register, respectively.
        let value_le_bytes = value.to_le_bytes();
        self.write(WriteOffset::THR, value_le_bytes[0]);
        self.write(WriteOffset::IER, value_le_bytes[1]);
    }

    pub fn get_baud(&self) -> Baud {
        match self.read_divisor_latch() {
            1 => Baud::B115200,
            2 => Baud::B57600,
            3 => Baud::B38400,
            6 => Baud::B19200,
            12 => Baud::B9600,
            24 => Baud::B4800,
            48 => Baud::B2400,
            96 => Baud::B1200,
            384 => Baud::B300,
            2304 => Baud::B50,
            _ => unimplemented!(),
        }
    }

    pub fn set_baud(&mut self, baud: Baud) {
        self.write_divisor_latch(baud as u16);
    }

    /// Disables access to the divisor latch registers.
    pub fn into_data_mode(mut self) -> Uart<Data> {
        let mut line_control = self.read_line_control();
        line_control.remove(LineControl::DLAB);

        self.write(WriteOffset::LCR, line_control.bits());

        Uart::<Data>(self.0, PhantomData)
    }
}

impl Uart<Data> {
    /// ### Safety
    ///
    /// - Provided address must be valid for reading as a UART device.
    /// - Provided address must not be otherwise mutably aliased.
    pub unsafe fn new(address: UartAddress) -> Self {
        Self(address, PhantomData)
    }

    pub fn read_byte(&self) -> u8 {
        self.read(ReadOffset::RHR)
    }

    pub fn write_byte(&mut self, byte: u8) {
        self.write(WriteOffset::THR, byte);
    }

    pub fn read_interrupt_enable(&self) -> InterruptEnable {
        InterruptEnable::from_bits_truncate(self.read(ReadOffset::IER))
    }

    pub fn write_interrupt_enable(&mut self, value: InterruptEnable) {
        self.write(WriteOffset::IER, value.bits());
    }

    /// Enables access to the divisor latch registers.
    pub fn into_configure_mode(mut self) -> Uart<Configure> {
        let mut line_control = self.read_line_control();
        line_control.insert(LineControl::DLAB);

        self.write(WriteOffset::LCR, line_control.bits());

        Uart::<Configure>(self.0, PhantomData)
    }
}
