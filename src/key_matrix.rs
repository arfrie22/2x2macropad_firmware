use embedded_hal::digital::v2::{InputPin, OutputPin};use embedded_hal::timer::CountDown;
use fugit::MicrosDurationU32;
use rp2040_hal::gpio::DynPin;

use crate::{Config, MacroType};


#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum KeyState {
    Idle,
    Intermediate,
    TapIntermediate,
    TTapIntermediate,
    Active,
}


const MATRIX_SCAN_DELAY: MicrosDurationU32 = MicrosDurationU32::millis(1);

const INPUT_PIN_COUNT: usize = 2;
const OUTPUT_PIN_COUNT: usize = 2;
const KEY_COUNT: usize = 4;


type MatrixState = [bool; KEY_COUNT];
type KeyStates = [KeyState; KEY_COUNT];
type KeyTimers<C> where C: CountDown, <C as cortex_m::prelude::_embedded_hal_timer_CountDown>::Time: From<fugit::Duration<u32, 1, 1000000>> = [C; KEY_COUNT];

pub struct Matrix<C> 
where C: CountDown,
<C as cortex_m::prelude::_embedded_hal_timer_CountDown>::Time: From<fugit::Duration<u32, 1, 1000000>> {
    input_pins: [DynPin; INPUT_PIN_COUNT],
    output_pins: [DynPin; OUTPUT_PIN_COUNT],
    matrix_scan_line: usize,    
    matrix_state: MatrixState,
    matrix_state_temp: MatrixState,
    timer: C,
    key_states: KeyStates,
    previous_key_states: KeyStates,
    key_timers: KeyTimers<C>,
    active_key: Option<(usize, MacroType)>,
}

impl<C> Matrix<C> 
where C: CountDown, <C as cortex_m::prelude::_embedded_hal_timer_CountDown>::Time: core::convert::From<fugit::Duration<u32, 1, 1000000>> {
    pub fn new(input_pins: [DynPin; 2], output_pins: [DynPin; 2], timer: C, key_timers: KeyTimers<C>) -> Self {
        let mut output = Self {
            input_pins,
            output_pins,
            matrix_scan_line: 0,
            matrix_state: [false; 4],
            matrix_state_temp: [false; 4],
            timer,
            key_states: [KeyState::Idle; 4],
            previous_key_states: [KeyState::Idle; 4],
            key_timers,
            active_key: None,
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
                            self.activate(i, MacroType::Hold);
                        }
                    } else if config.key_configs[i].key_mode == macropad_protocol::data_protocol::KeyMode::SingleTapMode {
                        self.activate(i, MacroType::Tap);
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
                        self.activate(i, MacroType::Tap);
                    }
                }
                KeyState::TTapIntermediate => {
                    if self.matrix_state[i] {
                        if self.key_timers[i].wait().is_ok() {
                            self.activate(i, MacroType::TapHold);
                        }
                    } else {
                        self.activate(i, MacroType::DoubleTap);
                    }
                }

                KeyState::Active => continue
            }
        }

        self.key_states
    }

    pub fn activate(&mut self, key: usize, macro_type: MacroType) {
        self.active_key = Some((key, macro_type));

        for i in 0..KEY_COUNT {
            if i != key {
                self.key_states[i] = KeyState::Idle;
            } else {
                self.key_states[i] = KeyState::Active;
            }
        }
    }

    pub fn finish(&mut self) {
        if let Some((key, _)) = self.active_key {
            self.key_states[key] = KeyState::Idle;
        }

        self.active_key = None;
    }

    pub fn get_active_key(&self) -> Option<(usize, MacroType)> {
        self.active_key
    }
}