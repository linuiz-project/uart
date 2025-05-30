use crate::{ReadableRegister, RegisterAddress, UartAddress, WriteableRegister};

/// A port-based UART address.
pub struct PortAddress(u16);

impl PortAddress {
    /// Creates a new [`PortAddress`] with `base` as the base port address.
    ///
    /// ## Safety
    ///
    /// - Base address must be a port-based UART device.
    pub const unsafe fn new(base: u16) -> Self {
        Self(base)
    }
}

// Safety: Constructor requires that the base address be valid, and register
//         impls are correctly offset from that.
unsafe impl UartAddress for PortAddress {
    fn get_read_address(&self, register: ReadableRegister) -> RegisterAddress {
        RegisterAddress::Port(self.0 + (register as u16))
    }

    fn get_write_address(&self, register: WriteableRegister) -> RegisterAddress {
        RegisterAddress::Port(self.0 + (register as u16))
    }
}

/// An MMIO-based UART address.
pub struct MmioAddress(*mut u8);

impl MmioAddress {
    /// Creates a new [`MmioAddress`] with `base` as the base memory address.
    ///
    /// ## Safety
    ///
    /// - Base address must be an MMIO-based UART device.
    pub const unsafe fn new(base: *mut u8) -> Self {
        Self(base)
    }
}

// Safety: Constructor requires that the base address be valid, and register
//         impls are correctly offset from that.
unsafe impl UartAddress for MmioAddress {
    fn get_read_address(&self, register: ReadableRegister) -> RegisterAddress {
        RegisterAddress::Mmio({
            // Safety: `self.0` is required to be a valid base address, and `register` is a valid register offset.
            unsafe { self.0.add(register as usize) }
        })
    }

    fn get_write_address(&self, register: WriteableRegister) -> RegisterAddress {
        RegisterAddress::Mmio({
            // Safety: `self.0` is required to be a valid base address, and `register` is a valid register offset.
            unsafe { self.0.add(register as usize) }
        })
    }
}
