//! Random number generator using the Ring Oscillator.
//!
//! Mostly cribbed from the embassy-rp crate.

use rand_chacha::rand_core;

pub struct RoscRng;

impl RoscRng {
    fn next_u8() -> u8 {
        let rosc = unsafe { &*rp2040_pac::ROSC::ptr() };
        let random_reg = rosc.randombit();
        let mut acc = 0;
        for _ in 0..u8::BITS {
            acc <<= 1;
            acc |= random_reg.read().randombit().bit() as u8;
        }
        acc
    }
}

impl rand_core::RngCore for RoscRng {
    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        Ok(self.fill_bytes(dest))
    }

    fn next_u32(&mut self) -> u32 {
        rand_core::impls::next_u32_via_fill(self)
    }

    fn next_u64(&mut self) -> u64 {
        rand_core::impls::next_u64_via_fill(self)
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        dest.fill_with(Self::next_u8)
    }
}
