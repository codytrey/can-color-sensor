#![no_std]
#![no_main]


use rtic::app;
use rtt_target::rprintln;
use core::panic::PanicInfo;
use cortex_m_rt::{exception, ExceptionFrame};




#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [EXTI3, EXTI4])]
mod app {
    use bxcan::{Tx, Rx};
    use systick_monotonic::*;
    use rtt_target::{rtt_init_print, rprintln};
    use rtic::Monotonic;
    use stm32f1xx_hal::{
        can::Can,
        pac::{I2C1, CAN1},
        i2c::{BlockingI2c, DutyCycle, Mode},
        prelude::*,
    };
    use bxcan::{self, filter::Mask32, Frame, ExtendedId};
    use nb::block;
    use cortex_m::{asm::delay, peripheral::DWT};
    use apds9151::Apds9151;
    // A monotonic timer to enable scheduling in RTIC
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        color_sensor: Apds9151<BlockingI2c<I2C1, (stm32f1xx_hal::gpio::Pin<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>, stm32f1xx_hal::gpio::CRH, 'B', 8_u8>, stm32f1xx_hal::gpio::Pin<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain> , stm32f1xx_hal::gpio::CRH, 'B', 9_u8>)>>,
        can_tx: Tx<Can<CAN1>>,
        can_rx: Rx<Can<CAN1>>,
    }
    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        rtt_init_print!();

        cx.core.DCB.enable_trace();
        DWT::unlock();
        cx.core.DWT.enable_cycle_counter();
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        
        let clocks = rcc.cfgr
            .use_hse(8.MHz())
            .sysclk(64.MHz())
            .hclk(64.MHz())
            .pclk1(16.MHz())
            .pclk2(64.MHz())
            .freeze(&mut flash.acr);

        let systick = cx.core.SYST;
        let mono = Systick::new(systick, 12_000_000);

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies
        // in `clocks`
        // let clocks = rcc.cfgr.freeze(&mut flash.acr);

        let mut afio = cx.device.AFIO.constrain();
        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();

        let mut can1 = {
            #[cfg(not(feature = "connectivity"))]
            let can = Can::new(cx.device.CAN1, cx.device.USB);
            #[cfg(feature = "connectivity")]
            let can = Can::new(cx.device.CAN1);
    
            // let mut gpioa = cx.device.GPIOA.split();
            let rx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
            let tx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
            can.assign_pins((tx, rx), &mut afio.mapr);
                
            // Change bit timing to 1mBit/s to match FRC CAN bit rate
            bxcan::Can::builder(can)
                .set_bit_timing(0x001c_0000)
                .set_loopback(false)
                .set_silent(false)
                .set_automatic_retransmit(false)
                .leave_disabled()
        };

        // Configure filters so that can frames can be received.
        let mut filters = can1.modify_filters();
        // Only rioHeart Beat frame will be received. Currently not using it
        filters.enable_bank(0, Mask32::frames_with_ext_id(bxcan::ExtendedId::new(0x01011840).unwrap(), bxcan::ExtendedId::new(0x1FFF_FFFF).unwrap()));

        drop(filters);

        // Select the interface.
        let mut can = can1;

        can.enable_interrupts(
            bxcan::Interrupts::FIFO0_MESSAGE_PENDING,
        );

        // Split the peripheral into transmitter and receiver parts.
        block!(can.enable_non_blocking()).unwrap();

        let (can_tx, can_rx) = can.split();

        rprintln!("CAN perif configured");

        let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

        let i2c = BlockingI2c::i2c1(
            cx.device.I2C1, (scl, sda), 
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
        delay(clocks.sysclk().raw() / 100);
        rprintln!("creating color sensor");
        let mut color_sensor = Apds9151::new_apda9151(i2c);
        rprintln!("init color sensor");
        color_sensor.initialize().unwrap();
        rprintln!("spawn task");
        send_color_sensor_value::spawn_after(systick_monotonic::ExtU64::secs(1)).unwrap();
        (Shared {}, Local {color_sensor, can_tx, can_rx}, init::Monotonics(mono))
    }

    #[task(local = [can_tx, color_sensor])]
    fn send_color_sensor_value(cx: send_color_sensor_value::Context) {
        let r_raw = cx.local.color_sensor.get_red().unwrap();
        let r16 = u16::try_from(r_raw/16).unwrap();
        rprintln!("Red: {:#}", r16);
        let g_raw = cx.local.color_sensor.get_green().unwrap();
        let g16 = u16::try_from(g_raw/16).unwrap();
        rprintln!("Green: {:#}", g16);
        let b_raw = cx.local.color_sensor.get_blue().unwrap();
        let b16 = u16::try_from(b_raw/16).unwrap();
        rprintln!("Blue: {:#}", b16);
        let mut buffer: [u8; 6] = [0; 6];
        buffer[..2].clone_from_slice(&r16.to_be_bytes());
        buffer[2..4].clone_from_slice(&g16.to_be_bytes());
        buffer[4..6].clone_from_slice(&b16.to_be_bytes());
        let misc_dev_type: u32 = 10;
        let team_use_manufacturer: u32 = 8;
        let api_index: u32 = 0;
        let dev_number: u32 = 60;
        let id = ExtendedId::new((misc_dev_type & 0x1F) << 24 | (team_use_manufacturer & 0xFF) << 16 | (api_index & 0x3FF) << 6 | (dev_number & 0x3F)).unwrap();
        let frame = Frame::new_data(id, buffer);
        if cx.local.can_tx.is_idle() {
            block!(cx.local.can_tx.transmit(&frame)).unwrap();
        }
        send_color_sensor_value::spawn_after(systick_monotonic::ExtU64::secs(1)/20).unwrap();
    }

    #[task(binds = USB_LP_CAN_RX0, local = [can_rx])]
    fn listen_heart_beat(cx: listen_heart_beat::Context) {
        let id = bxcan::ExtendedId::new(0x01011840).unwrap();
        loop {
            match cx.local.can_rx.receive() {
                Ok(frame) => {
                    if frame.id() == bxcan::Id::Extended(id) {
                        break;
                    }
                },
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {} // Ignore overrun errors.
            }
        }
    }
    
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        // cortex_m::asm::wfi();
        loop {}
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}