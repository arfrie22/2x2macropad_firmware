extern crate alloc;

use alloc::boxed::Box;
use cortex_m::prelude::_embedded_hal_timer_CountDown;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use fugit::MicrosDurationU32;
use rp2040_hal::{timer::CountDown, gpio::Pins, Timer};

use crate::Config;


#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum KeyState {
    Idle,
    Intermediate,
    TapIntermediate,
    Tap,
    Hold,
    TTapIntermediate,
    TTap,
    THold,
    Active,
    Done,
}


const MATRIX_SCAN_DELAY: MicrosDurationU32 = MicrosDurationU32::millis(1);

const INPUT_PIN_COUNT: usize = 2;
const OUTPUT_PIN_COUNT: usize = 2;
const KEY_COUNT: usize = 4;


type MatrixState = [bool; KEY_COUNT];
type KeyStates = [KeyState; KEY_COUNT];
type KeyTimers<'a> = [CountDown<'a>; KEY_COUNT];

pub struct Matrix {
    input_pins: [Box<dyn InputPin<Error = core::convert::Infallible>>; INPUT_PIN_COUNT],
    output_pins: [Box<dyn OutputPin<Error = core::convert::Infallible>>; OUTPUT_PIN_COUNT],
    matrix_scan_line: usize,    
    matrix_state: MatrixState,
    matrix_state_temp: MatrixState,
    timer: CountDown<'static>,
    key_states: KeyStates,
    previous_key_states: KeyStates,
    key_timers: KeyTimers<'static>,
}

impl Matrix {
    pub fn new(pins: Pins, timer: Timer) -> Self {
        let mut output = Self {
            input_pins: [
                Box::new(pins.gpio19.into_pull_down_input()),
                Box::new(pins.gpio18.into_pull_down_input()),
            ],
            output_pins: [
                Box::new(pins.gpio21.into_push_pull_output()),
                Box::new(pins.gpio20.into_push_pull_output()),
            ],
            matrix_scan_line: 0,
            matrix_state: [false; 4],
            matrix_state_temp: [false; 4],
            timer: timer.count_down(),
            key_states: [KeyState::Idle; 4],
            previous_key_states: [KeyState::Idle; 4],
            key_timers: [
                timer.count_down(),
                timer.count_down(),
                timer.count_down(),
                timer.count_down(),
            ],
        };

        output.output_pins[0].set_high().unwrap();
        output.timer.start(MATRIX_SCAN_DELAY);

        output
    }

    pub fn scan(&mut self) -> MatrixState {
        if self.timer.wait().is_ok() {
            for (i, pin) in self.input_pins.iter().enumerate() {
                self.matrix_state_temp[self.matrix_scan_line * OUTPUT_PIN_COUNT + i] = pin.is_high().unwrap();
            }

            self.output_pins[self.matrix_scan_line].set_low().unwrap();

            self.matrix_scan_line += 1;

            if self.matrix_scan_line == OUTPUT_PIN_COUNT {
                self.matrix_scan_line = 0;
                self.matrix_state = self.matrix_state_temp;
            }

            self.output_pins[self.matrix_scan_line].set_high().unwrap();
        }

        self.matrix_state
    }

    pub fn get_key_states(&mut self, config: &Config) -> KeyStates {
        self.previous_key_states = self.key_states.clone();

        for i in 0..KEY_COUNT {
            if config.key_configs[i].key_mode == macropad_protocol::data_protocol::KeyMode::KeyboardMode
                || config.key_configs[i].key_mode == macropad_protocol::data_protocol::KeyMode::ConsumerMode
            {
                self.key_states[i] = KeyState::Idle;
            }
            match self.previous_key_states[i] {
                KeyState::Idle => {
                    if self.matrix_state[i] {
                        self.key_states[i] = KeyState::Intermediate;
                        self.key_timers[i].start(MicrosDurationU32::micros(config.hold_speed));
                    }
                }
                KeyState::Intermediate => {
                    if self.matrix_state[i] {
                        if self.key_timers[i].wait().is_ok() {
                            self.key_states[i] = KeyState::Hold;
                        }
                    } else if config.key_configs[i].key_mode == macropad_protocol::data_protocol::KeyMode::SingleTapMode {
                        self.key_states[i] = KeyState::Tap;
                    } else {
                        self.key_states[i] = KeyState::TapIntermediate;
                        self.key_timers[i].start(MicrosDurationU32::micros(config.tap_speed));
                    }
                }
                KeyState::TapIntermediate => {
                    if self.matrix_state[i] {
                        self.key_states[i] = KeyState::TTapIntermediate;
                        self.key_timers[i].start(MicrosDurationU32::micros(config.hold_speed));
                    } else if self.key_timers[i].wait().is_ok() {
                        self.key_states[i] = KeyState::Tap;
                    }
                }
                KeyState::TTapIntermediate => {
                    if self.matrix_state[i] {
                        if self.key_timers[i].wait().is_ok() {
                            self.key_states[i] = KeyState::THold;
                        }
                    } else {
                        self.key_states[i] = KeyState::TTap;
                    }
                }

                KeyState::Tap
                | KeyState::Hold
                | KeyState::TTap
                | KeyState::THold
                | KeyState::Active => continue,

                KeyState::Done => {
                    if !self.matrix_state[i] {
                        self.key_states[i] = KeyState::Idle;
                    }
                }
            }
        }

        self.key_states
    }
}