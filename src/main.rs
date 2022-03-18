#![no_std]
#![no_main]


#[allow(unused_imports)]
use rtic::app;
use rtt_target::rprintln;
use core::panic::PanicInfo;
use cortex_m_rt::{exception, ExceptionFrame};
use apds9151::Apds9151;




#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [EXTI3, EXTI4])]
mod app {
    use systick_monotonic::*;
    use rtt_target::{rtt_init_print, rprintln};
    use rtic::Monotonic;
    // use embedded_hal::digital::v2::OutputPin;
    #[allow(unused_imports)]
    use stm32f1xx_hal::{
        can::Can,
        pac,
        pac::I2C1,
        i2c::{BlockingI2c, DutyCycle, Mode},
        prelude::*,
    };
    #[allow(unused_imports)]
    use bxcan::{self, filter::Mask32, Frame, ExtendedId};
    use nb::block;
    use cortex_m::{asm::delay, peripheral::DWT};
    use crate::apds9151::Apds9151;
    // A monotonic timer to enable scheduling in RTIC
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[shared]
    struct Shared {}
    #[local]
    struct Local {
        can: bxcan::Can<Can<pac::CAN1>>,
        // i2c: BlockingI2c<I2C1, (stm32f1xx_hal::gpio::Pin<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>, stm32f1xx_hal::gpio::CRH, 'B', 8_u8>, stm32f1xx_hal::gpio::Pin<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain> , stm32f1xx_hal::gpio::CRH, 'B', 9_u8>)>,
        color_sensor: crate::apds9151::Apds9151<BlockingI2c<I2C1, (stm32f1xx_hal::gpio::Pin<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>, stm32f1xx_hal::gpio::CRH, 'B', 8_u8>, stm32f1xx_hal::gpio::Pin<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain> , stm32f1xx_hal::gpio::CRH, 'B', 9_u8>)>>
        // color_sensor: crate::apds9151::Apds9151<I2c<I2C1, (stm32f1xx_hal::gpio::Pin<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>, stm32f1xx_hal::gpio::CRH, 'B', 8_u8>, stm32f1xx_hal::gpio::Pin<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain> , stm32f1xx_hal::gpio::CRH, 'B', 9_u8>)>>
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
            .sysclk(72.MHz())
            .pclk1(16.MHz())
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

        // Select the interface.
        let mut can = can1;

        // Split the peripheral into transmitter and receiver parts.
        block!(can.enable_non_blocking()).unwrap();

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
        // let i2c = I2c::i2c1(
        //     cx.device.I2C1, (scl, sda), 
        //     &mut afio.mapr, 
        //     Mode::Fast { 
        //         frequency: 400_000.Hz(),
        //         duty_cycle: DutyCycle::Ratio2to1 
        //     }, 
        //     clocks,
        // );
        rprintln!("creating color sensor");
        let mut color_sensor = Apds9151::new_apda9151(i2c);
        rprintln!("init color sensor");
        color_sensor.initialize().unwrap();
        rprintln!("spawn task");
        send_color_sensor_value::spawn_after(systick_monotonic::ExtU64::secs(2)/100).unwrap();

        // TODO implement i2c driver for Rev Robotics Color Sensor v3 (based on APDS-9151 datasheet: https://docs.broadcom.com/doc/APDS-9151-DS)
        (Shared {}, Local {can, color_sensor}, init::Monotonics(mono))
    }

    #[task(local = [can, color_sensor])]
    fn send_color_sensor_value(cx: send_color_sensor_value::Context) {
        let r_raw = cx.local.color_sensor.get_red().unwrap();
        let r16 = u16::try_from(r_raw/16).unwrap();
        let g_raw = cx.local.color_sensor.get_green().unwrap();
        let g16 = u16::try_from(g_raw/16).unwrap();
        let b_raw = cx.local.color_sensor.get_blue().unwrap();
        let b16 = u16::try_from(b_raw/16).unwrap();
        let mut buffer: [u8; 6] = [0; 6];
        buffer[..2].clone_from_slice(&r16.to_be_bytes());
        buffer[2..4].clone_from_slice(&g16.to_be_bytes());
        buffer[4..].clone_from_slice(&b16.to_be_bytes());
        let misc_dev_type = 0b1010;
        let team_use_manufacturer = 0b1000;
        let api_class = 0b000000;
        let api_index = 0b0000;
        let dev_number = 0b111100;
        let id = ExtendedId::new((((((((misc_dev_type << 8) ^ team_use_manufacturer) << 6) ^ api_class) << 4) ^ api_index) << 6) ^ dev_number).unwrap();
        let frame = Frame::new_data(id, buffer);
        block!(cx.local.can.transmit(&frame)).unwrap();
        // send_color_sensor_value::spawn_after(systick_monotonic::ExtU64::secs(2)/100).unwrap();
        send_color_sensor_value::spawn_after(systick_monotonic::ExtU64::secs(2)).unwrap();
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