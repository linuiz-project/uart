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
    /// Represents the possible states of the interrupt enable register.
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
    /// Represents the possible states of the interrupt status register.
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
    /// Represents the possible states of the FIFO control register.
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
    /// Represents the possible states of the line control register.
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
    /// Represents the possible states of the modem control register.
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
    /// Represents the possible states of the line status register.
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
    /// Represents the possible states of the modem status register.
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

/// The read-enabled UART registers and their offset index.
#[derive(Debug, Clone, Copy)]
pub enum ReadableRegister {
    ReceiverHolding,
    InterruptEnable,
    InterruptStatus,
    LineControl,
    LineStatus,
    ModemStatus,

    DivisorLatchLow,
    DivisorLatchHigh,
}

impl ReadableRegister {
    /// Offset index of the provided [`ReadableRegister`].
    pub const fn as_index(self) -> u16 {
        match self {
            ReadableRegister::ReceiverHolding | ReadableRegister::DivisorLatchLow => 0x0,
            ReadableRegister::InterruptEnable | ReadableRegister::DivisorLatchHigh => 0x1,
            ReadableRegister::InterruptStatus => 0x2,
            ReadableRegister::LineControl => 0x3,
            ReadableRegister::LineStatus => 0x5,
            ReadableRegister::ModemStatus => 0x6,
        }
    }
}

/// The write-enabled UART registers and their offset index.
#[repr(u16)]
#[derive(Debug, Clone, Copy)]
pub enum WriteableRegister {
    TransmitterHolding,
    InterruptEnable,
    FifoControl,
    LineControl,
    ModemControl,

    DivisorLatchLow,
    DivisorLatchHigh,
}

impl WriteableRegister {
    /// Offset index of the provided [`WriteableRegister`].
    pub const fn as_index(self) -> u16 {
        match self {
            WriteableRegister::TransmitterHolding | WriteableRegister::DivisorLatchLow => 0x0,
            WriteableRegister::InterruptEnable | WriteableRegister::DivisorLatchHigh => 0x1,
            WriteableRegister::FifoControl => 0x2,
            WriteableRegister::LineControl => 0x3,
            WriteableRegister::ModemControl => 0x4,
        }
    }
}

/// Represents an address type for a UART device, allowing idiomatic access to register
/// addresses, and allowing any number of constraints to be encoded based on the hardware
/// implemenation of the UART IO (whether port or memory mapped).
///
/// # Safety
///
/// - The type implementing this trait should only be constructed with a valid UART base
///   address (this may be a safety invariant on an `unsafe fn` constructor).
pub unsafe trait UartAddress {
    /// Writes `value` to `register`.
    ///
    /// # Safety
    ///
    /// - Writing `value` to the provided `register` has the potential to change
    ///   the value of other registers. For instance, writing [`FifoControl::CLEAR_TX`] will
    ///   change the contents of (in this case, zero out) [`WriteableRegister::TransmitterHolding`].
    unsafe fn write(&self, register: WriteableRegister, value: u8);

    /// Reads a raw [`u8`] from `register`.
    ///
    /// # Safety
    ///
    /// - Reading a value from certain registers will affect the contents of others.
    ///   For instance, reading the line status will always clear the [`InterruptStatus::INTERRUPT_PENDING`] bit.
    ///   Because of this, any calling context that passes [`ReadableRegister::LineStatus`] should be `&mut self`, or
    ///   otherwise exclusively alias the address that is being read from.
    unsafe fn read(&self, register: ReadableRegister) -> u8;
}

/// Identifies which mode a UART is in ([`Data`] or [`DLAB`]).
pub trait Mode {}

/// Data operation mode (rx/tx) for UART.
pub struct Data;
impl Mode for Data {}

/// Divisor latch configuration mode for UART.
pub struct DLAB;
impl Mode for DLAB {}

pub struct Uart<A: UartAddress, M: Mode> {
    base_address: A,
    marker: PhantomData<M>,
}

impl<A: UartAddress, M: Mode> Uart<A, M> {
    /// Read from the interrupt status register.
    pub fn read_interrupt_status(&self) -> InterruptStatus {
        // Safety: Reading the interrupt status register has no side effects, so `self` is immutably aliased.
        let value = unsafe { self.base_address.read(ReadableRegister::InterruptStatus) };

        InterruptStatus::from_bits_retain(value)
    }

    /// Write to the FIFO control register.
    pub fn write_fifo_control(&mut self, fifo_control: FifoControl) {
        // Safety: Writing the FIFO control register has side effects, so `self` is mutably aliased.
        unsafe {
            self.base_address
                .write(WriteableRegister::FifoControl, fifo_control.bits());
        }
    }

    /// Read from the line control register.
    pub fn read_line_control(&self) -> LineControl {
        // Safety: Reading the line control register has no side effects, so `self` is immutably aliased.
        let value = unsafe { self.base_address.read(ReadableRegister::LineControl) };

        // Safety: `self.read(...)` returns a single byte, of which `LineControl` uses all bits (so no unknown bits are possible).
        unsafe { LineControl::from_bits(value).unwrap_unchecked() }
    }

    /// Write to the line control register.
    pub fn write_line_control(&mut self, value: LineControl) {
        // Safety: Writing the line control register has side effects, so `self` is mutably aliased.
        unsafe {
            self.base_address
                .write(WriteableRegister::LineControl, value.bits());
        }
    }

    /// Write to the modem control register.
    pub fn write_modem_control(&mut self, value: ModemControl) {
        // Safety: Writing the modem control register has side effects, so `self` is mutably aliased.
        unsafe {
            self.base_address
                .write(WriteableRegister::ModemControl, value.bits());
        }
    }

    /// Read from the line status register.
    pub fn read_line_status(&mut self) -> LineStatus {
        // Safety: Reading the line status register has side effects, so `self` is mutably aliased.
        let value = unsafe { self.base_address.read(ReadableRegister::LineStatus) };

        LineStatus::from_bits_retain(value)
    }

    /// Read from the modem status register.
    pub fn read_modem_status(&self) -> ModemStatus {
        // Safety: Reading the modem status register has no side effects, so `self` is immutably aliased.
        let value = unsafe { self.base_address.read(ReadableRegister::ModemStatus) };

        ModemStatus::from_bits_retain(value)
    }
}

impl<A: UartAddress> Uart<A, DLAB> {
    /// Creates a new [`Uart`] pointing to `base_address`.
    ///
    /// # Safety
    ///
    /// - Provided address must be valid as the base address of a UART device.
    /// - Provided address must not be otherwise mutably aliased.
    /// - Device must have bit 7 of the line control register (DLAB enable) set to 1.
    pub unsafe fn new(base_address: A) -> Self {
        Self {
            base_address,
            marker: PhantomData,
        }
    }

    /// Read from the divisor latch from the 1st & 2nd registers.
    fn read_divisor_latch(&self) -> u16 {
        // When DLAB is enabled, offset index 1 and 2 become the least- and most- significant bits of the divisor latch, respectively.

        // Safety: While in DLAB mode, there are no side effects to reading the divisor latch registers, so `self` is immutably aliased.
        unsafe {
            let lsb = u16::from(self.base_address.read(ReadableRegister::ReceiverHolding));
            let msb = u16::from(self.base_address.read(ReadableRegister::InterruptEnable));

            (msb << 8) | lsb
        }
    }

    /// Write the divisor latch to the 1st & 2nd registers.
    fn write_divisor_latch(&mut self, value: u16) {
        // When DLAB is enabled, offset index 1 and 2 become the least- and most- significant bits of the divisor latch, respectively.

        let value_le_bytes = value.to_le_bytes();

        // Safety: Writing to the divisor latch registers  While in DLAB mode, there are no side effects to reading the divisor latch registers, so `self` is immutably aliased.
        unsafe {
            self.base_address
                .write(WriteableRegister::TransmitterHolding, value_le_bytes[0]);
            self.base_address
                .write(WriteableRegister::InterruptEnable, value_le_bytes[1]);
        }
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
    pub fn into_data_mode(self) -> Uart<A, Data> {
        let mut line_control = self.read_line_control();
        line_control.remove(LineControl::DLAB);

        // Safety: Unsetting the DLAB bit has side effects, so `self` is mutably aliased.
        unsafe {
            self.base_address
                .write(WriteableRegister::LineControl, line_control.bits());
        }

        Uart::<A, Data> {
            base_address: self.base_address,
            marker: PhantomData,
        }
    }
}

impl<A: UartAddress> Uart<A, Data> {
    /// Creates a new [`Uart`] pointing to `base_address`.
    ///
    /// # Safety
    ///
    /// - UART must have bit 7 of the line control register (DLAB enable) set to 0.
    pub unsafe fn new(base_address: A) -> Self {
        Self {
            base_address,
            marker: PhantomData,
        }
    }

    /// Creates a new [`Uart`] pointing to `base_address`, and resets all the control and holding registers.
    pub fn new_reset(base_address: A) -> Self {
        // Safety: We're going to reset the device, so we don't care if it's correct or not.
        let mut uart = unsafe { Self::new(base_address) };

        uart.write_fifo_control(FifoControl::CLEAR_RX | FifoControl::CLEAR_TX);
        uart.write_line_control(LineControl::empty());
        uart.write_modem_control(ModemControl::empty());

        uart
    }

    /// Reads a byte from the receiver holding register.
    pub fn read_byte(&mut self) -> u8 {
        // Safety: Reading from the receiver holding register clears it, so `self` is mutably aliased.
        unsafe { self.base_address.read(ReadableRegister::ReceiverHolding) }
    }

    /// Writes a byte to the transmitter holding register
    pub fn write_byte(&mut self, byte: u8) {
        // Safety: Writing to the transmitter holding register sets it, so `self` is mutably aliased.
        unsafe {
            self.base_address
                .write(WriteableRegister::TransmitterHolding, byte);
        }
    }

    /// Reads from the interrupt enable register.
    pub fn read_interrupt_enable(&self) -> InterruptEnable {
        // Safety: Reading the interrupt enable register has no side effects, so `self` is immutably aliased.
        let value = unsafe { self.base_address.read(ReadableRegister::InterruptEnable) };

        InterruptEnable::from_bits_retain(value)
    }

    /// Writes to the interrupt enable register.
    pub fn write_interrupt_enable(&mut self, value: InterruptEnable) {
        // Safety: Writing the interrupt enable register has side effects, so `self` is mutably aliased.
        unsafe {
            self.base_address
                .write(WriteableRegister::InterruptEnable, value.bits());
        }
    }

    /// Enables access to the divisor latch registers.
    pub fn into_dlab_mode(self) -> Uart<A, DLAB> {
        let mut line_control = self.read_line_control();
        line_control.insert(LineControl::DLAB);

        // Safety: Setting the DLAB bit has side effects, so `self` is mutably aliased.
        unsafe {
            self.base_address
                .write(WriteableRegister::LineControl, line_control.bits());
        }

        Uart::<A, DLAB> {
            base_address: self.base_address,
            marker: PhantomData,
        }
    }
}
