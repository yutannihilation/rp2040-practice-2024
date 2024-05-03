// ## The pin layout of A-551SR:
//
//   10 9 8 7 6
//   ┌───────┐
//   │       │
//   │       │
//   │       │
//   │       │
//   │       │
//   └───────┘
//   1 2 3 4 5
//
// Each pin corresponds to the following positions of the 7 segments.
// (3 & 8 are Vin, and 5 is for the right-bottom period)
//
//     ┌─ 7 ─┐
//     9     6
//     ├─10 ─┤
//     1     4
//     └─ 2 ─┘
//
// ## 74HC595
//
//    ┌─────v─────┐
//  1 │           │ 16
//  2 │           │ 15
//  3 │           │ 14 Input  <------------ GPIO2
//  4 │           │ 13
//  5 │           │ 12 Clock for input  <-- GPIO3
//  6 │           │ 11 Clock for output  <- GPIO4
//  7 │           │ 10
//  8 │           │  9
//    └───────────┘

#![no_std]
#![no_main]

// These math functions come from https://github.com/tarcieri/micromath/blob/main/src/float/floor.rs

#[inline]
fn floor(x: f32) -> f32 {
    let res = (x as i32) as f32;

    if x < res {
        res - 1.0
    } else {
        res
    }
}

use rtic_monotonics::rp2040::prelude::*;
rp2040_timer_monotonic!(Mono);

// repeat_pwm_1 is chosen probably because repeat_pwm_0 is used by the Timer by default?
// cf., https://github.com/rtic-rs/rtic/blob/ef8046b060a375fd5e6b23d62c3a9a303bbd6e11/rtic-monotonics/src/rp2040.rs#L170
#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use super::*;

    use defmt_rtt as _;
    use panic_halt as _;

    use rp_pico::hal::{self, Sio};

    // Import pio crates
    use hal::pio::{PIOBuilder, Tx};
    use pio_proc::pio_file;

    // Pull in any important traits
    use rp_pico::hal::prelude::*;

    #[derive(Debug, Clone, Copy)]
    struct PwmStep {
        length: u32,
        data: u32,
    }

    pub struct PwmData {
        pwm_levels: [u32; 8],
        pwm_steps: [PwmStep; 9],
    }

    impl PwmData {
        fn new() -> Self {
            let null_step = PwmStep {
                length: 255,
                data: 0,
            };

            Self {
                pwm_levels: [0; 8],
                pwm_steps: [null_step; 9],
            }
        }

        fn reflect(&mut self) {
            let mut indices: [usize; 8] = [0, 1, 2, 3, 4, 5, 6, 7];
            indices.sort_unstable_by_key(|&i| self.pwm_levels[i]);

            let mut data = 255;
            let mut prev_level = 0;
            let mut cur_level = 0;

            for (i, &cur_index) in indices.iter().enumerate() {
                cur_level = self.pwm_levels[cur_index];

                self.pwm_steps[i] = PwmStep {
                    length: cur_level - prev_level,
                    data,
                };

                data &= !(1 << cur_index);
                // info!("{:b}", data);

                prev_level = cur_level;
            }

            // period after all pins are set low
            self.pwm_steps[8] = PwmStep {
                length: 255 - cur_level,
                data: 0,
            };
        }
    }

    #[shared]
    struct Shared {
        data: PwmData,
    }

    #[local]
    struct Local {
        // tx ix is used in only one task, so this can be Local
        tx: Tx<rp_pico::hal::pio::PIO0SM0>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let mut resets = c.device.RESETS;
        Mono::start(c.device.TIMER, &resets);

        // While this doesn't use the `clock` object, it seems this code is
        // needed to initialize the clock.
        let mut watchdog = rp_pico::hal::watchdog::Watchdog::new(c.device.WATCHDOG);
        let _clocks = rp_pico::hal::clocks::init_clocks_and_plls(
            rp_pico::XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }

        let sio = Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // Note: while the compiler never complains, we cannot use pac::Peripherals::take().unwrap() directly
        let (mut pio0, sm0, _, _, _) = c.device.PIO0.split(&mut resets);

        // Create a pio program
        let program = pio_file!("./src/shift_register.pio", select_program("shift_register"),);
        let installed = pio0.install(&program.program).unwrap();

        let out_pin = pins.gpio2.into_function::<hal::gpio::FunctionPio0>();
        let _clock_pin = pins.gpio3.into_function::<hal::gpio::FunctionPio0>();
        let _ratch_pin = pins.gpio4.into_function::<hal::gpio::FunctionPio0>();

        // Build the pio program and set pin both for set and side set!
        // We are running with the default divider which is 1 (max speed)
        let out_pin_id = out_pin.id().num;
        let (mut sm, _, tx) = PIOBuilder::from_program(installed)
            .out_pins(out_pin_id, 1)
            .side_set_pin_base(out_pin_id + 1)
            .build(sm0);

        #[rustfmt::skip]
        sm.set_pindirs([
            (out_pin_id,     hal::pio::PinDir::Output),
            (out_pin_id + 1, hal::pio::PinDir::Output),
            (out_pin_id + 2, hal::pio::PinDir::Output),
        ]);

        // Start state machine
        let _sm = sm.start();

        let mut data = PwmData::new();

        data.pwm_levels = [0; 8];
        data.reflect();

        repeat_pwm::spawn().ok();
        update_data::spawn().ok();

        (Shared { data }, Local { tx })
    }

    #[task(
        shared = [data],
        local = [cur_pos: f32  = 0.0]
    )]
    async fn update_data(c: update_data::Context) {
        let mut data = c.shared.data;
        loop {
            data.lock(|data| {
                let cur_index = super::floor(*c.local.cur_pos);
                let fract = *c.local.cur_pos - cur_index;

                let cur_index = cur_index as usize;
                let prev_index = (cur_index + 8 - 1) % 8;
                let next_index = (cur_index + 8 + 1) % 8;

                data.pwm_levels[prev_index] = 0;
                data.pwm_levels[cur_index] = (255. * (1.0 - fract)) as u32;
                data.pwm_levels[next_index] = (255. * (fract - 0.4) * 1.667) as u32;

                data.reflect();
            });

            *c.local.cur_pos = (*c.local.cur_pos + 0.03) % 8.0;

            Mono::delay(15.millis()).await;
        }
    }

    #[task(
        shared = [data],
        local = [tx, step: u8 = 0],
    )]
    async fn repeat_pwm(c: repeat_pwm::Context) {
        let mut data = c.shared.data;
        let tx = c.local.tx;

        loop {
            let steps = data.lock(|data| data.pwm_steps);
            for step in steps {
                tx.write(step.data << 24);

                let delay_ms = ((step.length * 100) as u64).micros();
                Mono::delay(delay_ms).await;
            }
            *c.local.step = (*c.local.step + 1) % 8;
        }
    }
}
