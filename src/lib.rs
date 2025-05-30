#![no_std]
#![forbid(clippy::inline_asm_x86_att_syntax)]
#![deny(
    clippy::semicolon_if_nothing_returned,
    clippy::debug_assert_with_mut_call,
    clippy::float_arithmetic
)]
#![warn(clippy::cargo, clippy::pedantic, clippy::undocumented_unsafe_blocks)]
#![allow(clippy::must_use_candidate, clippy::upper_case_acronyms)]

#[cfg(feature = "address_impl")]
pub mod address;
#[cfg(feature = "writer_impl")]
pub mod writer;

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
    pub struct InterruptStatus: u8 {
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
        const TERMINAL_READY  = 1 << 0;

        /// Signal the chip is ready to receive data from the port.
        const REQUEST_TO_SEND = 1 << 1;

        /// In loopback mode, connected Ring Indicator (RI) signal input.
        const OUT_1           = 1 << 2;

        /// Enables or disables interrupts when not in loopback mode (bit 4).
        ///
        /// In loopback mode, connected Data Carrier Detect (DCD) signal input.
        const OUT_2           = 1 << 3;

        /// Enables loopback mode, in which transmitted bits are able to be read
        /// on the same port. This is useful for testing chip operation.
        const LOOPBACK_MODE   = 1 << 4;
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

pub enum RegisterAddress {
    Port(u16),
    Mmio(*mut u8),
}

#[repr(u16)]
#[derive(Debug, Clone, Copy)]
pub enum ReadableRegister {
    ReceiverHolding = 0x0,
    InterruptEnable = 0x1,
    InterruptStatus = 0x2,
    LineControl = 0x3,
    LineStatus = 0x5,
    ModemStatus = 0x6,
}

#[repr(u16)]
#[derive(Debug, Clone, Copy)]
pub enum WriteableRegister {
    TransmitterHolding = 0x0,
    InterruptEnable = 0x1,
    FifoControl = 0x2,
    LineControl = 0x3,
    ModemControl = 0x4,
}

/// Represents an address type for a UART device, allowing idiomatic access to register
/// addresses, and allowing any number of constraints to be encoded based on the hardware
/// implemenation of the UART IO (whether port or memory mapped).
///
/// ## Safety
///
/// - The type may only be constructed only with a valid UART base address.
/// - Ensure returned addresses are valid to read or write the register
///   passed in [`UartAddress::get_read_address`] or [`UartAddress::get_write_address`].
pub unsafe trait UartAddress {
    fn get_read_address(&self, register: ReadableRegister) -> RegisterAddress;
    fn get_write_address(&self, register: WriteableRegister) -> RegisterAddress;
}

pub trait Mode {}
pub struct Data;
impl Mode for Data {}
pub struct DLAB;
impl Mode for DLAB {}

pub struct Uart<A: UartAddress, M: Mode> {
    base_address: A,
    marker: PhantomData<M>,
}

impl<A: UartAddress, M: Mode> Uart<A, M> {
    fn read(&self, register: ReadableRegister) -> u8 {
        match self.base_address.get_read_address(register) {
            RegisterAddress::Port(port_address) => {
                let value: u8;

                #[cfg(target_arch = "x86_64")]
                // Safety: `UartAddress::get_read_address` must ensure `port_address` is valid for reading as UART register.
                unsafe {
                    core::arch::asm!(
                        "in al, dx",
                        out("al") value,
                        in("dx") port_address,
                        options(nostack, nomem, preserves_flags)
                    );
                }

                #[cfg(not(target_arch = "x86_64"))]
                unimplemented!();

                value
            }

            RegisterAddress::Mmio(mmio_address) => {
                // Safety: `UartAddress::get_read_address` must ensure `mmio_address` is valid for reading as UART register.
                unsafe { mmio_address.read_volatile() }
            }
        }
    }

    fn write(&mut self, register: WriteableRegister, value: u8) {
        match self.base_address.get_write_address(register) {
            RegisterAddress::Port(port_address) => {
                #[cfg(target_arch = "x86_64")]
                // Safety: `UartAddress::get_write_address` must ensure `port_address` is valid for writing as UART register.
                unsafe {
                    core::arch::asm!(
                        "out dx, al",
                        in("dx") port_address,
                        in("al") value,
                        options(nostack, nomem, preserves_flags)
                    );
                }

                #[cfg(not(target_arch = "x86_64"))]
                unimplemented!();
            }

            RegisterAddress::Mmio(mmio_address) => {
                // Safety: `UartAddress::get_write_address` must ensure `mmio_address` is valid for writing as UART register.
                unsafe { mmio_address.write_volatile(value) }
            }
        }
    }

    /// Read from the interrupt status register.
    pub fn read_interrupt_status(&self) -> InterruptStatus {
        InterruptStatus::from_bits_retain(self.read(ReadableRegister::InterruptStatus))
    }

    /// Write to the FIFO control register.
    pub fn write_fifo_control(&mut self, fifo_control: FifoControl) {
        self.write(WriteableRegister::FifoControl, fifo_control.bits());
    }

    /// Read from the line control register.
    pub fn read_line_control(&self) -> LineControl {
        // Safety: `self.read(...)` returns a single byte, of which `LineControl` uses all bits (so no unknown bits are possible).
        unsafe {
            LineControl::from_bits(self.read(ReadableRegister::LineControl)).unwrap_unchecked()
        }
    }

    /// Write to the line control register.
    pub fn write_line_control(&mut self, value: LineControl) {
        self.write(WriteableRegister::LineControl, value.bits());
    }

    /// Write to the modem control register.
    pub fn write_modem_control(&mut self, value: ModemControl) {
        self.write(WriteableRegister::ModemControl, value.bits());
    }

    /// Read from the line status register.
    pub fn read_line_status(&self) -> LineStatus {
        LineStatus::from_bits_truncate(self.read(ReadableRegister::LineStatus))
    }

    /// Read from the modem status register.
    pub fn read_modem_status(&self) -> ModemStatus {
        ModemStatus::from_bits_truncate(self.read(ReadableRegister::ModemStatus))
    }
}

impl<A: UartAddress> Uart<A, DLAB> {
    /// Read from the divisor latch from the 1st & 2nd registers.
    fn read_divisor_latch(&self) -> u16 {
        // When DLAB is enabled, RHR and IER become the LSB and MSB of the
        // divisor latch register, respectively.

        let lsb = u16::from(self.read(ReadableRegister::ReceiverHolding));
        let msb = u16::from(self.read(ReadableRegister::InterruptEnable));

        (msb << 8) | lsb
    }

    /// Write the divisor latch to the 1st & 2nd registers.
    fn write_divisor_latch(&mut self, value: u16) {
        // When DLAB is enabled, THR and IER become the LSB and MSB of the
        // divisor latch register, respectively.

        let value_le_bytes = value.to_le_bytes();

        self.write(WriteableRegister::TransmitterHolding, value_le_bytes[0]);
        self.write(WriteableRegister::InterruptEnable, value_le_bytes[1]);
    }

    /// Read from the baud rate from the divisor latch registers.
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

    /// Set the baud rate using the divisor latch registers.
    pub fn set_baud(&mut self, baud: Baud) {
        self.write_divisor_latch(baud as u16);
    }

    /// Disables access to the divisor latch registers.
    pub fn into_data_mode(mut self) -> Uart<A, Data> {
        let mut line_control = self.read_line_control();
        line_control.remove(LineControl::DLAB);

        self.write(WriteableRegister::LineControl, line_control.bits());

        Uart::<A, Data> {
            base_address: self.base_address,
            marker: PhantomData,
        }
    }
}

impl<A: UartAddress> Uart<A, Data> {
    /// ### Safety
    ///
    /// - Provided address must be valid as the base address of a UART device.
    /// - Provided address must not be otherwise mutably aliased.
    pub unsafe fn new(base_address: A) -> Self {
        Self {
            base_address,
            marker: PhantomData,
        }
    }

    /// Reads a byte from the 1st register (RHR).
    pub fn read_byte(&self) -> u8 {
        self.read(ReadableRegister::ReceiverHolding)
    }

    /// Writes a byte to the 2nd register (THR).
    pub fn write_byte(&mut self, byte: u8) {
        self.write(WriteableRegister::TransmitterHolding, byte);
    }

    /// Reads from the interrupt enable register.
    pub fn read_interrupt_enable(&self) -> InterruptEnable {
        InterruptEnable::from_bits_truncate(self.read(ReadableRegister::InterruptEnable))
    }

    /// Writes to the interrupt enable register.
    pub fn write_interrupt_enable(&mut self, value: InterruptEnable) {
        self.write(WriteableRegister::InterruptEnable, value.bits());
    }

    /// Enables access to the divisor latch registers.
    pub fn into_dlab_mode(mut self) -> Uart<A, DLAB> {
        let mut line_control = self.read_line_control();
        line_control.insert(LineControl::DLAB);

        self.write(WriteableRegister::LineControl, line_control.bits());

        Uart::<A, DLAB> {
            base_address: self.base_address,
            marker: PhantomData,
        }
    }
}
