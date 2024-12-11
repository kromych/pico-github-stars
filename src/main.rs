#![no_std]
#![no_main]

use defmt_rtt as _;
use embedded_graphics::prelude::*;
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use fugit::RateExtU32;
use matrix_symbols::BRIGHTNESS_LEVELS;
use matrix_symbols::GLYPH_COUNT;
use matrix_symbols::GLYPH_HEIGHT;
use matrix_symbols::GLYPH_WIDTH;
use panic_probe as _;
use pico_display_pimoroni::DisplayKind;
use pico_display_pimoroni::DisplayRotation;
use rand::RngCore;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::gpio;
use rp2040_hal::pwm;
use rp2040_hal::rom_data;
use rp2040_hal::Clock;

mod lax_dma;
mod matrix_symbols;
mod pico_display_pimoroni;
mod rng;

#[allow(dead_code)]
mod time {
    pub fn time_us() -> u32 {
        unsafe { (*rp2040_pac::TIMER::PTR).timerawl().read().bits() }
    }

    pub fn time_us64() -> u64 {
        unsafe {
            (*rp2040_pac::TIMER::PTR).timelr().read().bits() as u64
                | (((*rp2040_pac::TIMER::PTR).timehr().read().bits() as u64) << 32)
        }
    }
}

#[rp_pico::entry]
fn main() -> ! {
    const DISPLAY_KIND: DisplayKind = DisplayKind::PicoDisplay2_8;
    const DISPLAY_ROTATION: DisplayRotation = DisplayRotation::Rotate0;
    const DISPLAY_WIDTH: u32 = DISPLAY_KIND.width() as u32;
    const DISPLAY_HEIGHT: u32 = DISPLAY_KIND.height() as u32;

    let mut buffer = [0u16; DISPLAY_WIDTH as usize * DISPLAY_HEIGHT as usize];

    defmt::info!(
        "Board {}, git revision {:x}, ROM verion {:x}, time {:x} us",
        rom_data::copyright_string(),
        rom_data::git_revision(),
        rom_data::rom_version_number(),
        time::time_us64()
    );

    let mut pac = rp2040_pac::Peripherals::take().unwrap();
    let core = rp2040_pac::CorePeripherals::take().unwrap();
    let mut watchdog = rp2040_hal::watchdog::Watchdog::new(pac.WATCHDOG);

    let clocks = rp2040_hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = rp2040_hal::sio::Sio::new(pac.SIO);
    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut red_led_pin = pins.gpio26.into_push_pull_output();
    let mut green_led_pin = pins.gpio27.into_push_pull_output();
    let mut blue_led_pin = pins.gpio28.into_push_pull_output();

    red_led_pin.set_high().unwrap();
    green_led_pin.set_high().unwrap();
    blue_led_pin.set_high().unwrap();

    let backlight_pin = pins.gpio20.into_function::<gpio::FunctionPwm>();
    let dc_pin = pins.gpio16.into_push_pull_output();
    let cs_pin = pins.gpio17.into_push_pull_output();
    let sck_pin = pins.gpio18.into_function::<gpio::FunctionSpi>();
    let mosi_pin = pins.gpio19.into_function::<gpio::FunctionSpi>();
    let vsync_pin = pins.gpio21.into_floating_input();

    let dma = pac.DMA.split(&mut pac.RESETS);
    //lax_dma::tests::run_dma_tests();

    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut backlight_pwm = pwm_slices.pwm2;
    backlight_pwm.set_ph_correct();
    backlight_pwm.enable();
    backlight_pwm.channel_a.output_to(backlight_pin);
    backlight_pwm.channel_a.set_duty_cycle(0).unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut display = pico_display_pimoroni::Display::new(
        DISPLAY_KIND,
        DISPLAY_ROTATION,
        backlight_pwm.channel_a,
        dc_pin,
        cs_pin,
        vsync_pin,
        pac.SPI0,
        (mosi_pin, sck_pin),
        dma.ch0,
        dma.ch1,
        &mut delay,
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        62_500.kHz(),
    );

    let mut frame = display.whole_screen(&mut buffer).unwrap();
    let mut symbol_data = [0u16; GLYPH_WIDTH as usize * GLYPH_HEIGHT as usize];
    let size = Size::new(GLYPH_WIDTH as u32, GLYPH_HEIGHT as u32);
    let mut rng = rng::RoscRng;

    // {
    //     use embedded_graphics::draw_target::DrawTarget;
    //     use embedded_graphics::pixelcolor::Rgb565;
    //     use embedded_graphics::primitives::PrimitiveStyleBuilder;
    //     use embedded_graphics::primitives::Rectangle;

    //     frame.clear(Rgb565::BLACK).unwrap();
    //     let colors = [Rgb565::RED, Rgb565::GREEN, Rgb565::BLUE];
    //     for i in 0..3 {
    //         Rectangle::new(
    //             Point::new(
    //                 (i * DISPLAY_WIDTH / 3) as i32,
    //                 (i * DISPLAY_HEIGHT / 4) as i32,
    //             ),
    //             Size::new(DISPLAY_WIDTH / 3, DISPLAY_HEIGHT / 2),
    //         )
    //         .into_styled(
    //             PrimitiveStyleBuilder::new()
    //                 .fill_color(colors[i as usize])
    //                 .stroke_color(Rgb565::WHITE)
    //                 .stroke_width(2)
    //                 .build(),
    //         )
    //         .draw(&mut frame)
    //         .unwrap();
    //     }
    //     frame.flush();

    //     delay.delay_ms(1000);
    // }

    let mut draw_some = || {
        let glyphs_per_x = DISPLAY_WIDTH as usize / GLYPH_WIDTH as usize;
        let glyphs_per_y = DISPLAY_HEIGHT as usize / GLYPH_HEIGHT as usize;
        let padding_x = (DISPLAY_WIDTH as usize - glyphs_per_x * GLYPH_WIDTH as usize) / 2;
        let padding_y = (DISPLAY_HEIGHT as usize - glyphs_per_y * GLYPH_HEIGHT as usize) / 2;

        for _ in 0..300 {
            let brightness = rng.next_u32() as usize % BRIGHTNESS_LEVELS;
            let symbol = rng.next_u32() as usize % GLYPH_COUNT;
            let grid_x = rng.next_u32() as usize % glyphs_per_x;
            let grid_y = rng.next_u32() as usize % glyphs_per_y;
            let x = padding_x + grid_x * GLYPH_WIDTH as usize;
            let y = padding_y + grid_y * GLYPH_HEIGHT as usize;

            matrix_symbols::get_matrix_symbol_rgb565(
                symbol as u8,
                brightness as u8,
                &mut symbol_data,
            );
            frame.copy_raw_data(&symbol_data, size, Point::new(x as i32, y as i32));
        }
        frame.flush();
    };

    loop {
        draw_some();
        delay.delay_ms(70);
        // cortex_m::asm::wfe();
        // defmt::info!("WFE time: {:x}", time::time_us64());
    }
}
