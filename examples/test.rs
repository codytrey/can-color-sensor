#![no_main]
#![no_std]


use cortex_m_rt::entry;
use panic_probe as _;
use rtt_target::{rtt_init_print, rprintln};
use stm32f1xx_hal::{
    can::Can,
    pac,
    prelude::*,
};
use shared_bus;
use rotary_encoder_embedded::{RotaryEncoder, Direction};
use bxcan::{self, filter::Mask32, Frame, StandardId};

use nb::block;


#[entry]
fn main() -> ! {
    rtt_init_print!();
    let dp = pac::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    rcc.cfgr.use_hse(8.MHz()).freeze(&mut flash.acr);

    
    let mut afio = dp.AFIO.constrain();
    

    let mut gpioa = dp.GPIOA.split();

    // let mut can1 = {
    //     // #[cfg(not(feature = "connectivity"))]
    //     let can = Can::new(dp.CAN1, dp.USB);
    //     // #[cfg(feature = "connectivity")]
    //     // let can = Can::new(dp.CAN1);

    //     // let mut gpioa = dp.GPIOA.split();
    //     let rx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
    //     let tx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
    //     can.assign_pins((tx, rx), &mut afio.mapr);

    //     // APB1 (PCLK1): 8MHz, Bit rate: 125kBit/s, Sample Point 87.5%
    //     // Value was calculated with http://www.bittiming.can-wiki.info/
    //     bxcan::Can::builder(can)
    //         .set_bit_timing(0x001c_0003)
    //         .leave_disabled()
    // };
    let can = Can::new(dp.CAN1, dp.USB);
    let rx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
    let tx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
    can.assign_pins((tx, rx), &mut afio.mapr);


    

    // Configure filters so that can frames can be received.
    // let mut filters = can1.modify_filters();
    // filters.enable_bank(0, Mask32::accept_all());

    // drop(filters);

    rprintln!("CAN perif configured");

    let rotary_dt = gpioa.pa1.into_pull_up_input(&mut gpioa.crl);
    let rotary_clk = gpioa.pa0.into_pull_up_input(&mut gpioa.crl);
    
    let mut rotary_encoder = RotaryEncoder::new(
        rotary_dt,
        rotary_clk,
    );

    rprintln!("encoder configured");
    // Optional: to configure sensitivity if needed
    // rotary_encoder.set_sensitivity(Sensitivity::Low);

    let test = ["A", "B", "C", "D", "E", "F", "G", "H" , "I", "J", "K", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z"];
    let mut index: usize = 0;
    rprintln!("entering loop");
    // hprintln!("entering loop").unwrap();

    // Select the interface.
    // let mut can = can1;

    // Split the peripheral into transmitter and receiver parts.
    // block!(can.enable_non_blocking()).unwrap();

    
    let bus = shared_bus::BusManagerSimple::new(can);
    let can_dev1 = bus.acquire_can();

    loop {
        // Update the encoder, which will compute its direction
        rotary_encoder.update();

        // Get the rotary values
        let direction = rotary_encoder.direction();
        if direction == Direction::Clockwise {
            rprintln!("CW turn");
            // Increment some value
            if index < (test.len() -1) {
                index = index +1;
                let frame = Frame::new_data(StandardId::new(0).unwrap(), bxcan::Data::new(test[index].as_bytes()).unwrap());
                block!(can_dev1.transmit(&frame)).unwrap();
                // if let Ok(frame) = block!(can.receive()) {
                //     block!(can.transmit(&frame)).unwrap();
                // }
                rprintln!("transmit done");
            }
        } else if direction == Direction::Anticlockwise {
            rprintln!("CCW turn");
            // Decrement some value
            if index > 0 {
                index = index - 1;
                let frame = Frame::new_data(StandardId::new(0).unwrap(), bxcan::Data::new(test[index].as_bytes()).unwrap());
                block!(can_dev1.transmit(&frame)).unwrap();
                // if let Ok(frame) = block!(can.receive()) {
                //     block!(can.transmit(&frame)).unwrap();
                // }
                rprintln!("transit done");
            }
        }
    }
}
