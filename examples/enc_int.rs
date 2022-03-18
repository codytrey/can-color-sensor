#![no_std]
#![no_main]



// use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use panic_semihosting as _;
#[allow(unused_imports)]
use rtic::{
    app,
    cyccnt::{Instant, U32Ext},
    Exclusive
};
// use embedded_hal::digital::v2::{OutputPin, InputPin, IoPin};
use embedded_graphics::{
    mono_font::{ascii::FONT_9X18, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use ssd1306::{I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode, prelude::*};
use stm32f1xx_hal::{
    gpio::{
        // PullDown, 
        PullUp, OpenDrain, Alternate, Input,
        gpioa::{PA0, PA1, 
            // PA2
        }, 
        gpiob::{PB8, PB9}, ExtiPin, Edge},
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac::{Interrupt, I2C1, TIM1},
    prelude::*, 
    timer::{CountDownTimer, Event, Timer},
    // timer::{Event, Timer}
    // stm32
};
use rotary_encoder_embedded::{RotaryEncoder, Direction};
use cortex_m_semihosting::hprintln;

type Display = Ssd1306<
    I2CInterface<
        BlockingI2c<
            I2C1,
            (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>>,
        DisplaySize128x32,
        BufferedGraphicsMode<ssd1306::prelude::DisplaySize128x32>
>;


#[app(device = stm32f1xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        rotary_enc: RotaryEncoder<PA1<Input<PullUp>>, PA0<Input<PullUp>>>,
        // btn: PA2<Input<PullDown>>,
        display: Display,
        // #[init(MonoTextStyle::new(&FONT_9X18, BinaryColor::On))]
        // style: MonoTextStyle<FONT_9X18, BinaryColor>
        #[init(true)]
        cw: bool,
        timer: CountDownTimer<TIM1>,
    }
    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies
        // in `clocks`
        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        let mut afio = cx.device.AFIO.constrain(&mut rcc.apb2);

        let exti = cx.device.EXTI;
        hprintln!("init start").unwrap();
        // Acquire the GPIOC peripheral
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.apb2);

        let mut rotary_dt = gpioa.pa1.into_pull_up_input(&mut gpioa.crl);
        let mut rotary_clk = gpioa.pa0.into_pull_up_input(&mut gpioa.crl);
        rotary_dt.make_interrupt_source(&mut afio);
        rotary_dt.trigger_on_edge(&exti, Edge::RISING_FALLING);
        rotary_clk.make_interrupt_source(&mut afio);
        rotary_clk.trigger_on_edge(&exti, Edge::RISING_FALLING);
        // let btn = gpioa.pa2.into_pull_down_input(&mut gpioa.crl);

        let mut rotary_enc: RotaryEncoder<PA1<Input<PullUp>>, PA0<Input<PullUp>>> = RotaryEncoder::new(
            rotary_dt,
            rotary_clk,
        );

        let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

        let i2c = BlockingI2c::i2c1(
            cx.device.I2C1, (scl, sda), 
            &mut afio.mapr, 
            Mode::Fast { 
                frequency: 400_000.hz(),
                duty_cycle: DutyCycle::Ratio2to1 
            }, 
            clocks, 
            &mut rcc.apb1, 
            1000, 
            10,
            1000,
            1000,
        );
        // let style = MonoTextStyle::new(&FONT_9X18, BinaryColor::On);
        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();
        Text::new("Clockwise", Point::new(8, 20), MonoTextStyle::new(&FONT_9X18, BinaryColor::On)).draw(&mut display).unwrap();
        display.flush().unwrap();

        rtic::pend(Interrupt::EXTI0);
        rtic::pend(Interrupt::EXTI1);
        // rtic::pend(Interrupt::EXTI2);

        let mut timer = Timer::tim1(cx.device.TIM1, &clocks, &mut rcc.apb2).start_count_down(10.hz());
        timer.listen(Event::Update);

        rotary_enc.borrow_pins().0.enable_interrupt(&exti);
        rotary_enc.borrow_pins().1.enable_interrupt(&exti);
        hprintln!("init end").unwrap();

        init::LateResources {
            rotary_enc,
            // btn,
            display,
            timer
        }
    }

    #[task(binds = TIM1_UP, priority = 2, resources = [display, cw, timer])]
    fn update_display(cx: update_display::Context) {
        // hprintln!("update_display start").unwrap();
        let update_display::Resources {
            display,
            cw,
            timer
        } = cx.resources;
        // hprintln!("CW is {}", *cw).unwrap();
        display.clear();
        if *cw {
            Text::new("Clockwise", Point::new(8, 20), MonoTextStyle::new(&FONT_9X18, BinaryColor::On)).draw(display).unwrap();
        } else {
            Text::new("AntiClockwise", Point::new(8, 20), MonoTextStyle::new(&FONT_9X18, BinaryColor::On)).draw(display).unwrap();
        }
        display.flush().unwrap();
        timer.clear_update_interrupt_flag();
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        cortex_m::asm::wfi();
        loop {}
    }

    #[task(binds = EXTI1, spawn = [handle_rotary], resources = [rotary_enc])]
    fn exti1(cx: exti1::Context) {
        let pin = cx.resources.rotary_enc.borrow_pins().0;
        if pin.check_interrupt() {  
            cx.spawn.handle_rotary().unwrap();
            pin.clear_interrupt_pending_bit();
        }
    }

    #[task(binds = EXTI0, spawn = [handle_rotary], resources = [rotary_enc])]
    fn exti0(cx: exti0::Context) {
        let pin = cx.resources.rotary_enc.borrow_pins().1;
        if pin.check_interrupt() {  
            cx.spawn.handle_rotary().unwrap();
            pin.clear_interrupt_pending_bit();
        }
    }
    
    #[allow(unused_mut)]
    #[task(resources = [rotary_enc, cw], capacity = 6)]
    fn handle_rotary(mut cx: handle_rotary::Context) {
        // hprintln!("handle_rotary start").unwrap();
        cx.resources.rotary_enc.update();
        match cx.resources.rotary_enc.direction() {
            Direction::Clockwise => {
                // hprintln!("Direction is Clockwise").unwrap();
                cx.resources.cw.lock(|cw| {
                    *cw = false;
                });
            },
            Direction::Anticlockwise => {
                cx.resources.cw.lock(|cw| {
                    *cw = true;
                });
            },
            Direction::None => {
            }
        };
    }

    extern "C" {
        fn EXTI4();
        fn EXTI2();
    }
};