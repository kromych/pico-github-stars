//! Floating-point math functions

#![allow(dead_code)]

#[cfg(feature = "use-rom-instrinsics")]
pub mod rom_instrinsics {
    use rp2040_hal::rom_data::float_funcs::fexp;
    use rp2040_hal::rom_data::float_funcs::fln;

    pub fn powf32(base: f32, exp: f32) -> f32 {
        fexp(exp * fln(base))
    }
}

pub mod handrolled {
    pub fn floorf(x: f32) -> f32 {
        let xi = x as i32 as f32;
        if x < xi {
            xi - 1.0
        } else {
            xi
        }
    }

    pub fn fracf(x: f32) -> f32 {
        x - floorf(x)
    }

    pub fn int_pow(mut base: f32, mut exp: i32) -> f32 {
        if exp == 0 {
            return 1.0;
        }
        if exp < 0 {
            base = 1.0 / base;
            exp = -exp;
        }
        let mut result = 1.0;
        while exp > 0 {
            if exp % 2 == 1 {
                result *= base;
            }
            base *= base;
            exp /= 2;
        }
        result
    }

    // Implement square root using Carmack's Trick
    pub fn _sqrt_approx_carmack(x: f32) -> f32 {
        if x == 0.0 {
            return 0.0;
        }
        // Initial approximation
        let x_half = 0.5 * x;
        let mut i = x.to_bits();
        i = 0x5f3759df - (i >> 1);
        let mut y = f32::from_bits(i);

        // Newton-Raphson few times
        y = y * (1.5 - (x_half * y * y));
        y = y * (1.5 - (x_half * y * y));
        y = y * (1.5 - (x_half * y * y));

        // Invert the inverse square root to get the square root
        1.0 / y
    }

    pub fn sqrt_newton(x: f32) -> f32 {
        if x == 0.0 {
            return 0.0;
        }

        // Initial guess can be x / 2 or another approximation
        let mut approx = x / 2.0;
        // Number of iterations of the Newton-Raphson method; adjust for desired accuracy
        for _ in 0..8 {
            approx = 0.5 * (approx + x / approx);
        }

        approx
    }

    fn frac_pow(base: f32, mut frac: f32) -> f32 {
        let mut result = 1.0;
        let mut current_power = sqrt_newton(base); // Approximate a^{1/2}
        let mut factor = 0.5; // Corresponds to 1/2^1

        while frac > 0.0 && current_power != 1.0 {
            if frac >= factor {
                result *= current_power;
                frac -= factor;
            }
            current_power = sqrt_newton(current_power); // Approximate next root
            factor *= 0.5;
        }

        result
    }

    pub fn powf32(base: f32, exp: f32) -> f32 {
        if base == 0.0 {
            return if exp > 0.0 { 0.0 } else { f32::INFINITY };
        }
        if base < 0.0 && fracf(exp) != 0.0 {
            // Negative base with fractional exponent results in a complex number
            return f32::NAN;
        }
        if exp == 0.0 {
            return 1.0;
        }

        // Split exponent into integer and fractional parts
        let int_part = floorf(exp);
        let frac_part = fracf(exp);

        let int_result = int_pow(base, int_part as i32);
        let frac_result = frac_pow(base, frac_part);

        int_result * frac_result
    }
}
