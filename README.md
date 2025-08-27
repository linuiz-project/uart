# UART

The [Universional Asynchronous Receiver-Transmitter](https://en.wikipedia.org/wiki/Universal_asynchronous_receiver-transmitter) (UART for short) is an asychronous serial communication device found in most computers, often including even embedded or IoT devices. It is a ubiquoutous model/specification to allow communication between two devices through a serial port.

This crate aims for full implementation support of features from the [16550 UART](https://en.wikipedia.org/wiki/16550_UART) ([detailed specification here](https://www.ti.com/lit/ug/sprugp1/sprugp1.pdf)).

## Usage

Below is an example of a `core::fmt::Write`-implementing type that initializes, tests, and is able to write to a UART.

***note**: found at `uart::writer::UartWriter`, enabled by default feature `"writer_impl"`.*

```rust
use crate::{
    Baud, Data, FifoControl, InterruptEnable, LineControl, LineStatus, ModemControl, Uart,
    UartAddress,
};

/// Convenience type for interacting unidirectionally (write-only) with a 16550
/// UART device. It should be noted that this implementation is meant to be a
/// refernece or testing implementation, as it is extremely slow, uses blocking
/// IO, and does not utilize the FIFO.
pub struct UartWriter<A: UartAddress>(Uart<A, Data>);

impl<A: UartAddress> UartWriter<A> {
    /// ### Safety
    ///
    /// - `address` must be a valid serial address pointing to a UART 16550
    ///   device.
    /// - `address` must not be read from or written to by another context.
    pub unsafe fn new(address: A) -> Option<Self> {
        // Safety: Function invariants provide safety guarantees.
        let mut uart = unsafe { Uart::<A, Data>::new(address) };

        // Bring UART to a known state.
        uart.write_line_control(LineControl::empty());
        uart.write_fifo_control(FifoControl::empty());
        uart.write_interrupt_enable(InterruptEnable::empty());

        // Configure the baud rate (tx/rx speed).
        let mut uart = uart.into_dlab_mode();
        uart.set_baud(Baud::B115200);
        let mut uart = uart.into_data_mode();

        // Set character size to 8 bits with no parity.
        uart.write_line_control(LineControl::BITS_8);

        // Test the UART to ensure it's functioning correctly.
        uart.write_modem_control(
            ModemControl::REQUEST_TO_SEND
                | ModemControl::OUT_1
                | ModemControl::OUT_2
                | ModemControl::LOOPBACK_MODE,
        );

        uart.write_byte(0x1F);
        if uart.read_byte() != 0x1F {
            return None;
        }

        // Configure modem control for actual UART usage.
        uart.write_modem_control(
            ModemControl::TERMINAL_READY | ModemControl::REQUEST_TO_SEND | ModemControl::OUT_1,
        );

        Some(Self(uart))
    }
}

impl<A: UartAddress> core::fmt::Write for UartWriter<A> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        s.chars().try_for_each(|c| self.write_char(c))
    }

    fn write_char(&mut self, c: char) -> core::fmt::Result {
        while !self.0.read_line_status().contains(LineStatus::THR_EMPTY) {
            core::hint::spin_loop();
        }

        self.0.write_byte({
            // We can practically only write ASCII to a UART, so replace
            // any char that isn't ASCII with a question mark.
            if c.is_ascii() { c as u8 } else { b'?' }
        });

        Ok(())
    }
}
```

Notably, the above implementation is I/O-agnostic, and uses the crate features to accept both [port-mapped and memory-mapped I/O](https://en.wikipedia.org/wiki/Memory-mapped_I/O_and_port-mapped_I/O). Below are examples within the crate of implementations that address both types of I/O.


### Port-mapped I/O

***note**: found at `uart::address::PortAddress`, enabled by default feature `"address_impl"`.*

```rust
use crate::{ReadableRegister, RegisterAddress, UartAddress, WriteableRegister};

/// A port-based UART address.
pub struct PortAddress(NonZero<u16>);

impl PortAddress {
    /// Creates a new [`PortAddress`] with `base` as the base port address.
    ///
    /// # Safety
    ///
    /// - Base address must be a port-based UART device.
    pub const unsafe fn new(base: NonZero<u16>) -> Self {
        Self(base)
    }
}

// Safety: Constructor requires that the base address be valid, and register
//         impls are correctly offset from that.
unsafe impl UartAddress for PortAddress {
    unsafe fn read(&self, register: ReadableRegister) -> u8 {
        let port_address = self.0.checked_add(register.as_index()).unwrap();
        let value: u8;

        // Safety: Caller is required to ensure that reading from port `port_address` is valid.
        #[cfg(target_arch = "x86_64")]
        unsafe {
            core::arch::asm!(
                "in al, dx",
                out("al") value,
                in("dx") port_address.get(),
                options(nostack, nomem, preserves_flags)
            );
        }

        #[cfg(not(target_arch = "x86_64"))]
        unimplemented!();

        value
    }

    unsafe fn write(&self, register: WriteableRegister, value: u8) {
        let port_address = self.0.checked_add(register.as_index()).unwrap();

        // Safety: Caller is required to ensure that writing `value` to port `port_address` is valid.
        #[cfg(target_arch = "x86_64")]
        unsafe {
            core::arch::asm!(
                "out dx, al",
                in("dx") port_address.get(),
                in("al") value,
                options(nostack, nomem, preserves_flags)
            );
        }

        #[cfg(not(target_arch = "x86_64"))]
        unimplemented!();
    }
}
```

### Memory-mapped I/O

***note**: found at `uart::address::MmioAddress`, enabled by default feature `"address_impl"`.*

```rust
use crate::{ReadableRegister, RegisterAddress, UartAddress, WriteableRegister};

/// An MMIO-based UART address.
pub struct MmioAddress {
    base: NonNull<u8>,
    stride: usize,
}

impl MmioAddress {
    /// Creates a new [`MmioAddress`] with `base` as the base memory address.
    ///
    /// # Safety
    ///
    /// - `base` must be a pointer to an MMIO-based UART device.
    /// - `stride` must be the uniform distance (in bytes) between each UART register.
    pub const unsafe fn new(base: NonNull<u8>, stride: usize) -> Self {
        Self { base, stride }
    }
}

// Safety: Constructor requires that the base address be valid, and register
//         impls are correctly offset from that.
unsafe impl UartAddress for MmioAddress {
    unsafe fn write(&self, register: WriteableRegister, value: u8) {
        // Safety: - `self.base` is required to be a valid base address.
        //         - `register` is a valid offset.
        //         - `self.stride` is required to be the distance (in bytes) between each UART register.
        //         - Writing `value` is required to not cause undefined behaviour.
        unsafe {
            self.base
                .byte_add(usize::from(register.as_index()) * self.stride)
                .write_volatile(value);
        }
    }

    unsafe fn read(&self, register: ReadableRegister) -> u8 {
        // Safety: - `self.base` is required to be a valid base address.
        //         - `register` is a valid offset.
        //         - `self.stride` is required to be the distance (in bytes) between each UART register.
        //         - Reading `value` is required to not cause undefined behaviour.
        unsafe {
            self.base
                .byte_add(usize::from(register.as_index()) * self.stride)
                .read_volatile()
        }
    }
}

```
