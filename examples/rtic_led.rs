#![no_main]
#![no_std]

extern crate panic_semihosting;

#[rtic::app(device = lpc55_hal::raw, peripherals = true, dispatchers = [PIN_INT0])]
mod app {
    use cortex_m_semihosting::dbg;

    use hal::{drivers::pins, drivers::pins::Level, prelude::*, typestates::pin};
    use lpc55_hal as hal;

    type RedLed = hal::Pin<pins::Pio1_6, pin::state::Gpio<pin::gpio::direction::Output>>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: RedLed,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let _cp = c.core;
        let dp = c.device;

        let mut syscon = hal::Syscon::from(dp.SYSCON);
        let mut gpio = hal::Gpio::from(dp.GPIO).enabled(&mut syscon);
        let mut iocon = hal::Iocon::from(dp.IOCON).enabled(&mut syscon);

        let pins = hal::Pins::take().unwrap();
        let red_led = pins
            .pio1_6
            .into_gpio_pin(&mut iocon, &mut gpio)
            .into_output(Level::High);

        (Shared {}, Local { led: red_led })
    }

    #[idle(local = [led])]
    fn idle(ctx: idle::Context) -> ! {
        let led = ctx.local.led;
        loop {
            dbg!("low");
            led.set_low().unwrap();

            dbg!("high");
            led.set_high().unwrap();
        }
    }
}
