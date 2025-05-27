use crate::{
    Baud, Data, InterruptEnable, LineControl, LineStatus, ModemControl, Uart, UartAddress,
};

/// Convenience type for interacting unidirectionally (write-only) with a 16550 UART device.
///
/// It should be noted that this implementation is meant to be a refernece or testing
/// implementation, as it is extremely slow and uses blocking IO to write.
pub struct UartWriter(Uart<Data>);

impl UartWriter {
    /// ### Safety
    ///
    /// - `address` must be a valid serial address pointing to a UART 16550 device.
    /// - `address` must not be read from or written to by another context.
    pub unsafe fn new(address: UartAddress) -> Option<Self> {
        // Safety: Function invariants provide safety guarantees.
        let mut uart = unsafe { Uart::<Data>::new(address) };

        // Bring UART to a known state.
        uart.write_line_control(LineControl::empty());
        uart.write_interrupt_enable(InterruptEnable::empty());

        // Configure the baud rate (tx/rx speed).
        let mut uart = uart.into_configure_mode();
        uart.set_baud(Baud::B115200);
        let mut uart = uart.into_data_mode();

        // Set character size to 8 bits with no parity.
        uart.write_line_control(LineControl::BITS_8);

        // Test the UART to ensure it's functioning correctly.
        uart.write_modem_control(
            ModemControl::REQUEST_TO_SEND
                | ModemControl::AUXILIARY_OUTPUT_1
                | ModemControl::AUXILIARY_OUTPUT_2
                | ModemControl::LOOPBACK_MODE,
        );

        uart.write_byte(0x1F);
        if uart.read_byte() != 0x1F {
            return None;
        }

        // Configure modem control for actual UART usage.
        uart.write_modem_control(
            ModemControl::TERMINAL_READY
                | ModemControl::REQUEST_TO_SEND
                | ModemControl::AUXILIARY_OUTPUT_1
                | ModemControl::AUXILIARY_OUTPUT_2,
        );

        Some(Self(uart))
    }
}

impl core::fmt::Write for UartWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for c in s.chars() {
            self.write_char(c)?;
        }

        Ok(())
    }

    fn write_char(&mut self, c: char) -> core::fmt::Result {
        while !self.0.read_line_status().contains(LineStatus::THR_EMPTY) {
            core::hint::spin_loop();
        }
        self.0.write_byte(u8::try_from(c).unwrap_or(b'?'));

        Ok(())
    }
}
