use pico_display::{Bpp1, Display2_8, PicoDisplay};

pub type MyDisplay = PicoDisplay<Display2_8, Bpp1>;

pub const RADIUS: i16 = 16;

/// Draw a filled circle using the midpoint algorithm.
pub fn fill_circle(display: &mut MyDisplay, cx: i16, cy: i16, r: i16) {
    let w = display.width() as i16;
    let h = display.height() as i16;

    for dy in -r..=r {
        let sy = cy + dy;
        if sy < 0 || sy >= h {
            continue;
        }
        // Integer sqrt: dx² + dy² ≤ r²
        let dx_max = isqrt((r * r - dy * dy) as u32) as i16;
        let x0 = (cx - dx_max).max(0) as u16;
        let x1 = ((cx + dx_max).min(w - 1)) as u16;
        for x in x0..=x1 {
            display.set_pixel(x, sy as u16, true);
        }
    }
}

/// Integer square root.
fn isqrt(n: u32) -> u32 {
    if n == 0 {
        return 0;
    }
    let mut x = n;
    let mut y = x.div_ceil(2);
    while y < x {
        x = y;
        y = (x + n / x) / 2;
    }
    x
}

pub struct BounceState {
    pub cx: i16,
    pub cy: i16,
    pub dx: i16,
    pub dy: i16,
    pub w: i16,
    pub h: i16,
}

impl BounceState {
    pub fn new(display: &MyDisplay) -> Self {
        let w = display.width() as i16;
        let h = display.height() as i16;
        Self {
            cx: w / 4,
            cy: h / 3,
            dx: 2,
            dy: 1,
            w,
            h,
        }
    }

    pub fn draw_and_advance(&mut self, display: &mut MyDisplay) {
        display.clear();
        fill_circle(display, self.cx, self.cy, RADIUS);

        self.cx += self.dx;
        self.cy += self.dy;

        if self.cx - RADIUS <= 0 || self.cx + RADIUS >= self.w {
            self.dx = -self.dx;
            self.cx += self.dx;
        }
        if self.cy - RADIUS <= 0 || self.cy + RADIUS >= self.h {
            self.dy = -self.dy;
            self.cy += self.dy;
        }
    }
}
