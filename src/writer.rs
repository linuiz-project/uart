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
    /// Constructs a new [`UartWriter`].
    ///
    /// # Safety
    ///
    /// - `address` must be a valid serial address pointing to a UART 16550
    ///   device.
    /// - `address` must not be read from or written to by another context.
    ///
    /// # Panics
    ///
    /// Panics if `loopback_test` is `true`, and the loopback test fails.
    pub unsafe fn new(address: A, loopback_test: bool) -> Self {
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

        if loopback_test {
            // Test the UART to ensure it's functioning correctly.
            uart.write_modem_control(
                ModemControl::REQUEST_TO_SEND
                    | ModemControl::OUT_1
                    | ModemControl::OUT_2
                    | ModemControl::LOOPBACK_MODE,
            );

            // Here, we write a byte to the output, which in loopback mode
            // *should* insert that same byte into the receiver register
            // to be read. If this fails, the UART either:
            // A) doesn't support loopback mode, or
            // B) is malfunctioning and/or faulty
            uart.write_byte(0x1F);
            assert!(uart.read_byte() == 0x1F, "loopback test failed");
        }

        // Configure modem control for actual UART usage.
        uart.write_modem_control(
            ModemControl::TERMINAL_READY | ModemControl::REQUEST_TO_SEND | ModemControl::OUT_1,
        );

        Self(uart)
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
            // any char that isn't with a question mark.
            if c.is_ascii() { c as u8 } else { b'?' }
        });

        Ok(())
    }
}
