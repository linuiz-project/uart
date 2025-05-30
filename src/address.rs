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
pub struct MmioAddress {
    base: *mut u8,
    stride: usize,
}
impl MmioAddress {
    /// Creates a new [`MmioAddress`] with `base` as the base memory address.
    ///
    /// ## Safety
    ///
    /// - `base` must be a pointer to an MMIO-based UART device.
    /// - `stride` must be the uniform distance (in bytes) between each UART register.
    pub const unsafe fn new(base: *mut u8, stride: usize) -> Self {
        Self { base, stride }
    }
}

// Safety: Constructor requires that the base address be valid, and register
//         impls are correctly offset from that.
unsafe impl UartAddress for MmioAddress {
    fn get_read_address(&self, register: ReadableRegister) -> RegisterAddress {
        RegisterAddress::Mmio({
            // Safety: `self.base` is required to be a valid base address, `register` is a
            //         valid offset, and `self.stride` is required to be the distance (in bytes)
            //         between each UART register.
            unsafe { self.base.add((register as usize) * self.stride) }
        })
    }

    fn get_write_address(&self, register: WriteableRegister) -> RegisterAddress {
        RegisterAddress::Mmio({
            // Safety: `self.base` is required to be a valid base address, `register` is a
            //         valid offset, and `self.stride` is required to be the distance (in bytes)
            //         between each UART register.
            unsafe { self.base.add((register as usize) * self.stride) }
        })
    }
}
