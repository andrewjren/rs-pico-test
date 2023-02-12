//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![allow(incomplete_features)]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    image::ImageRaw,
    image::Image,
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Circle, Line, Triangle, PrimitiveStyleBuilder},
    text::{Baseline, Text, TextStyleBuilder},
};
use embedded_hal::{
    digital::v2::OutputPin,
    spi::MODE_0,
    prelude::*,
};

use fugit::RateExtU32;
use epd_waveshare::{
    color::*, 
    epd7in5_v2::{Display7in5, Epd7in5}, 
    graphics::DisplayRotation, 
    prelude::*,
};
//use embedded_time::fixed_point::FixedPoint;
//use embedded_time::rate::Extensions;
use panic_probe as _;

// not using linux-embedded-hal - need to import SPI bus functionality, somehow

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
//use rp_pico::hal::prelude::*;
use rp_pico as bsp;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    
    // set up SPI bus
    let _spi_sclk = pins.gpio10.into_mode::<bsp::hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<bsp::hal::gpio::FunctionSpi>();
    let spi = bsp::hal::spi::Spi::<_, _, 8>::new(pac.SPI1);

    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        // you can put cookie (increase the speed) in it but I don't recommend it.
        4_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    
    // Start the rest of pins needed to communicate with the screen
    let mut cs = pins.gpio9.into_push_pull_output(); // CS
    cs.set_high().unwrap();
    let busy = pins.gpio13.into_pull_up_input(); // BUSY
    let dc = pins.gpio8.into_push_pull_output(); // DC
    let rst = pins.gpio12.into_push_pull_output(); // RST

    // Start the EPD struct
    let mut epd = Epd7in5::new(
        &mut spi,   // SPI
        cs,         // CS
        busy,       // BUSY
        dc,         // DC
        rst,        // RST
        &mut delay, // DELAY
    )
    .unwrap();
    // Start the display buffer
    let mut display = Display7in5::default();

    epd.wake_up(&mut spi, &mut delay).unwrap();
    // epd.set_background_color(epd_waveshare::color::Color::Black);
    // epd.clear_frame(&mut spi).unwrap();
    // epd.set_background_color(epd_waveshare::color::Color::White);
    epd.clear_frame(&mut spi, &mut delay).unwrap();

    display.clear_buffer(Color::Black);

    // Start the fun
    // draw white on black background
    let style = MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::ascii::FONT_6X10)
        .text_color(BinaryColor::Off)
        .background_color(BinaryColor::On)
        .build();
    let text_style = TextStyleBuilder::new().baseline(Baseline::Top).build();

    let _ = Text::with_text_style("andrew was here!", Point::new(175, 250), style, text_style)
        .draw(&mut display);
    epd.update_frame(&mut spi, &display.buffer(), &mut delay);
    epd.display_frame(&mut spi, &mut delay);
    display.clear_buffer(Color::Black);
    let _ = Text::with_text_style("now he's not.", Point::new(175, 250), style, text_style)
        .draw(&mut display);
    epd.update_frame(&mut spi, &display.buffer(), &mut delay);
    epd.display_frame(&mut spi, &mut delay);

    const DATA: &[u8] = &[
    0b11101111, 0b0101_0000,
    0b10001000, 0b0101_0000,
    0b11101011, 0b0101_0000,
    0b10001001, 0b0101_0000,
    0b11101111, 0b0101_0000,
    ];

    let raw_image = ImageRaw::<BinaryColor>::new(DATA, 12);
    let image = Image::new(&raw_image, Point::zero())
        .draw(&mut display);
    epd.update_frame(&mut spi, &display.buffer(), &mut delay);
    epd.display_frame(&mut spi, &mut delay);


    /*
    for i in 0..(WIDTH / 10) as i32 {
        Line::new(Point::new(i * 10, 0), Point::new(i * 10, HEIGHT as i32))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::Off, 1))
            .draw(&mut display)
            .unwrap();
    }
    for i in 0..(HEIGHT / 10) as i32 {
        Line::new(Point::new(0, i * 10), Point::new(WIDTH as i32, i * 10))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::Off, 1))
            .draw(&mut display)
            .unwrap();
    }
*/
//    display.set_rotation(DisplayRotation::Rotate270);

    loop {
        delay.delay_ms(500);

    }

}

// End of file
