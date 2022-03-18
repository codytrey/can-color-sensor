#![no_main]
#![no_std]

use cortex_m_rt::entry;
use panic_probe as _;
use embedded_graphics::{
    mono_font::{ascii::FONT_9X18, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
    primitives::{PrimitiveStyleBuilder, Rectangle},
};
use rtt_target::{rtt_init_print, rprintln};
use stm32f1xx_hal::{
    can::Can,
    pac,
    i2c::{BlockingI2c, DutyCycle, Mode},
    prelude::*,
};
use bxcan::{self, filter::Mask32};
use core::str;
use nb::block;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};



#[entry]
fn main() -> ! {
    rtt_init_print!();
    let dp = pac::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).freeze(&mut flash.acr);

    
    let mut afio = dp.AFIO.constrain();
    

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    let mut can1 = {
        let can = Can::new(dp.CAN1, dp.USB);

        // let mut gpioa = dp.GPIOA.split();
        let rx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let tx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
        can.assign_pins((tx, rx), &mut afio.mapr);

        // APB1 (PCLK1): 8MHz, Bit rate: 125kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        bxcan::Can::builder(can)
            .set_bit_timing(0x001c_0003)
            .leave_disabled()
    };

    // Configure filters so that can frames can be received.
    let mut filters = can1.modify_filters();
    filters.enable_bank(0, Mask32::accept_all());

    drop(filters);

    rprintln!("CAN perif configured");

    // Optional: to configure sensitivity if needed
    // rotary_encoder.set_sensitivity(Sensitivity::Low);

    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    let i2c = BlockingI2c::i2c1(
        dp.I2C1, (scl, sda), 
        &mut afio.mapr, 
        Mode::Fast { 
            frequency: 400_000.Hz(),
            duty_cycle: DutyCycle::Ratio2to1 
        }, 
        clocks, 
        1000, 
        10,
        1000,
        1000,
    );

    let interface = I2CDisplayInterface::new(i2c);
    rprintln!("i2c configured");
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let yoffset = 20;

    let style = PrimitiveStyleBuilder::new()
        .stroke_width(1)
        .stroke_color(BinaryColor::On)
        .build();
    rprintln!("about to draw rectangle");
    Rectangle::new(Point::new(0, 0), Size::new(127, 31))
        .into_styled(style)
        .draw(&mut display)
        .unwrap();
    let text_style = MonoTextStyle::new(&FONT_9X18, BinaryColor::On);
    rprintln!("first display flush");
    display.flush().unwrap();

    // Select the interface.
    let mut can = can1;

    // Split the peripheral into transmitter and receiver parts.
    block!(can.enable_non_blocking()).unwrap();

    loop {
        rprintln!("blocking until receive frame");
        if let Ok(frame) = block!(can.receive()) {
            rprintln!("frame received");
            display.clear();
            display.flush().unwrap();
            Rectangle::new(Point::new(0, 0), Size::new(127, 31))
                .into_styled(style)
                .draw(&mut display)
                .unwrap();
            let received_data = str::from_utf8(frame.data().unwrap().as_ref()).unwrap_or("err");
            rprintln!("received can data {:#}", received_data);
            Text::new(received_data, Point::new(16, yoffset), text_style).draw(&mut display).unwrap();
            display.flush().unwrap()
        }
    }
}
