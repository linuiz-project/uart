use core::{num::NonZero, ptr::NonNull};

use crate::{ReadableRegister, UartAddress, WriteableRegister};

/// A port-based UART address.
pub struct PortAddress(NonZero<u16>);

impl PortAddress {
    /// Creates a new [`PortAddress`] with `base` as the base port address.
    ///
    /// ## Safety
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
        let port_address = self.0.checked_add(register as u16).unwrap();
        let value: u8;

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
        let port_address = self.0.checked_add(register as u16).unwrap();

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

/// An MMIO-based UART address.
pub struct MmioAddress {
    base: NonNull<u8>,
    stride: usize,
}

impl MmioAddress {
    /// Creates a new [`MmioAddress`] with `base` as the base memory address.
    ///
    /// ## Safety
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
                .byte_add((register as usize) * self.stride)
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
                .byte_add((register as usize) * self.stride)
                .read_volatile()
        }
    }
}
