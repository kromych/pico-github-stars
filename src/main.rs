#![no_std]
#![no_main]

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use fugit::RateExtU32;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::gpio;
use rp2040_hal::pwm;
use rp2040_hal::rom_data;
use rp2040_hal::Clock;

use defmt_rtt as _;
use panic_probe as _;

mod float;
mod lax_dma;
mod pico_display_pimoroni;

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
    display.set_backlight(40);
    display.fill(Rgb565::GREEN.into_storage());

    display.clear(Rgb565::RED).unwrap();
    display.flush();

    loop {
        cortex_m::asm::wfe();
        defmt::info!("WFE time: {:x}", time::time_us64());
    }
}
