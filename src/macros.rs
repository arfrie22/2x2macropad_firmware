use core::cell::UnsafeCell;

use embedded_hal::timer::CountDown;
use crc::{Crc, CRC_32_CKSUM};
use fugit::MicrosDurationU32;
use macropad_protocol::{macro_protocol::MacroCommand, data_protocol::{KeyMode, PROTOCOL_VERSION}};
use packed_struct::prelude::PackedStruct;
use smart_leds::RGB8;
use strum::IntoEnumIterator;
use strum_macros::EnumIter;
use usbd_human_interface_device::page::{Keyboard, Consumer};

use crate::{FlashBlock, rp2040_flash::flash, led_effect::LedConfig};

pub const CKSUM: Crc<u32> = Crc::<u32>::new(&CRC_32_CKSUM);

impl FlashBlock {
    #[inline(never)]
    fn addr(&self) -> u32 {
        &self.data as *const _ as u32
    }

    #[inline(never)]
    fn read(&self) -> &[u8; 4096] {
        // Make sure the compiler can't know that
        // we actually access a specific static
        // variable, to avoid unexpected optimizations
        //
        // (Don't try this with strict provenance.)
        let addr = self.addr();

        unsafe { &*(&*(addr as *const Self)).data.get() }
    }

    unsafe fn write_flash(&self, data: &[u8; 4096]) {
        let addr = self.addr() - 0x10000000;
        defmt::assert!(addr & 0xfff == 0);

        cortex_m::interrupt::free(|_cs| {
            flash::flash_range_erase_and_program(addr, data, true);
        });
    }
}

// TODO safety analysis - this is probably not sound
unsafe impl Sync for FlashBlock {}

impl FlashBlock {
    fn validate(&self) -> bool {
        let data = self.read();
        self.get_checksum() == u32::from_le_bytes(data[4092..].try_into().unwrap())
    }

    fn get_checksum(&self) -> u32 {
        let data = self.read();
        CKSUM.checksum(&data[0..4092])
    }

    fn set_checksum(&self) {
        let mut data = *self.read();
        let checksum = CKSUM.checksum(&data[0..4092]);
        data[4092..].copy_from_slice(&checksum.to_le_bytes());
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        unsafe {
            self.write_flash(&data);
        }
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }

    fn clear_flash(&self) {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        let data = [0u8; 4096];
        unsafe {
            self.write_flash(&data);
        }
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }

    fn initialize_flash(&self) {
        self.clear_flash();
        self.set_checksum();
    }
}

#[link_section = ".rodata"]
static CONFIG: FlashBlock = FlashBlock {
    data: UnsafeCell::new([0; 4096]),
};

pub struct KeyMacro {
    tap: FlashBlock,
    hold: FlashBlock,
    ttap: FlashBlock,
    thold: FlashBlock,
}

#[derive(Debug, EnumIter, Clone, Copy)]
pub enum MacroType {
    Tap,
    Hold,
    DoubleTap,
    TapHold,
}

impl KeyMacro {
    pub fn validate(&self, t: &MacroType) -> bool {
        match t {
            MacroType::Tap => self.tap.validate(),
            MacroType::Hold => self.hold.validate(),
            MacroType::DoubleTap => self.ttap.validate(),
            MacroType::TapHold => self.thold.validate(),
        }
    }

    pub fn get_checksum(&self, t: &MacroType) -> u32 {
        match t {
            MacroType::Tap => self.tap.get_checksum(),
            MacroType::Hold => self.hold.get_checksum(),
            MacroType::DoubleTap => self.ttap.get_checksum(),
            MacroType::TapHold => self.thold.get_checksum(),
        }
    }

    pub fn set_checksum(&self, t: &MacroType) {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        match t {
            MacroType::Tap => self.tap.set_checksum(),
            MacroType::Hold => self.hold.set_checksum(),
            MacroType::DoubleTap => self.ttap.set_checksum(),
            MacroType::TapHold => self.thold.set_checksum(),
        }
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }

    pub fn write_flash(&self, t: &MacroType, data: &[u8; 4096]) {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        unsafe {
            match t {
                MacroType::Tap => self.tap.write_flash(data),
                MacroType::Hold => self.hold.write_flash(data),
                MacroType::DoubleTap => self.ttap.write_flash(data),
                MacroType::TapHold => self.thold.write_flash(data),
            }
        }
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }

    #[allow(dead_code)]
    pub fn clear_flash(&self, t: &MacroType) {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        match t {
            MacroType::Tap => self.tap.clear_flash(),
            MacroType::Hold => self.hold.clear_flash(),
            MacroType::DoubleTap => self.ttap.clear_flash(),
            MacroType::TapHold => self.thold.clear_flash(),
        }
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }

    pub fn initialize_flash(&self, t: &MacroType) {
        match t {
            MacroType::Tap => self.tap.initialize_flash(),
            MacroType::Hold => self.hold.initialize_flash(),
            MacroType::DoubleTap => self.ttap.initialize_flash(),
            MacroType::TapHold => self.thold.initialize_flash(),
        }
    }

    pub fn read(&self, t: &MacroType) -> &[u8; 4096] {
        match t {
            MacroType::Tap => self.tap.read(),
            MacroType::Hold => self.hold.read(),
            MacroType::DoubleTap => self.ttap.read(),
            MacroType::TapHold => self.thold.read(),
        }
    }

    pub fn get_macro(&'static self, t: &MacroType) -> &'static FlashBlock {
        return match t {
            MacroType::Tap => &self.tap,
            MacroType::Hold => &self.hold,
            MacroType::DoubleTap => &self.ttap,
            MacroType::TapHold => &self.thold,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KeyConfig {
    pub key_mode: KeyMode,
    pub keyboard_data: u8,
    pub consumer_data: u16,
    pub key_color: RGB8,
}

impl PackedStruct for KeyConfig {
    type ByteArray = [u8; 7];

    fn pack(&self) -> packed_struct::PackingResult<Self::ByteArray> {
        let mut bytes = [0u8; 7];

        bytes[0] = self.key_mode as u8;
        bytes[1] = self.keyboard_data;
        bytes[2..4].copy_from_slice(&self.consumer_data.to_le_bytes());
        bytes[4] = self.key_color.r;
        bytes[5] = self.key_color.g;
        bytes[6] = self.key_color.b;

        Ok(bytes)
    }

    fn unpack(src: &Self::ByteArray) -> packed_struct::PackingResult<Self> {
        Ok(KeyConfig {
            key_mode: KeyMode::try_from(src[0]).unwrap_or(KeyMode::MacroMode),
            keyboard_data: src[1],
            consumer_data: u16::from_le_bytes([src[2], src[3]]),
            key_color: RGB8 {
                r: src[4],
                g: src[5],
                b: src[6],
            },
        })
    }
}

impl Default for KeyConfig {
    fn default() -> Self {
        Self {
            key_mode: KeyMode::MacroMode,
            keyboard_data: 0,
            consumer_data: 0,
            key_color: RGB8::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, PackedStruct)]
#[packed_struct(size_bytes = "4092")]
pub struct Config {
    #[packed_field(endian = "lsb")]
    pub version: u16,
    #[packed_field(endian = "lsb")]
    pub tap_speed: u32,
    #[packed_field(endian = "lsb")]
    pub hold_speed: u32,

    //...
    #[packed_field(element_size_bytes = "7")]
    pub key_configs: [KeyConfig; 4],
    #[packed_field(element_size_bytes = "13")]
    pub led_config: LedConfig,
}

impl Config {
    pub fn get_config() -> Config {
        if CONFIG.validate() {
            let data = CONFIG.read();
            let data = {
                let mut data_1 = [0; 4092];
                data_1.copy_from_slice(&data[0..4092]);
                data_1
            };
            let config = Config::unpack(&data).unwrap();
            if config.version == PROTOCOL_VERSION {
                return config;
            }
        }
    
        let config = Config::default();
        config.write();
        config
    }

    pub fn write(&self) {
        let mut data = [0; 4096];
        data[0..4092].copy_from_slice(&self.pack().unwrap());
        unsafe {
            CONFIG.write_flash(&data);
        }
        CONFIG.set_checksum();
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            version: PROTOCOL_VERSION,
            tap_speed: MicrosDurationU32::millis(200).to_micros(),
            hold_speed: MicrosDurationU32::millis(200).to_micros(),
            key_configs: [KeyConfig::default(); 4],
            led_config: LedConfig::default(),
        }
    }
}

#[link_section = ".rodata"]
static MACRO_1: KeyMacro = KeyMacro {
    tap: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
    hold: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
    ttap: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
    thold: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
};

#[link_section = ".rodata"]
static MACRO_2: KeyMacro = KeyMacro {
    tap: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
    hold: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
    ttap: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
    thold: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
};

#[link_section = ".rodata"]
static MACRO_3: KeyMacro = KeyMacro {
    tap: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
    hold: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
    ttap: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
    thold: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
};

#[link_section = ".rodata"]
static MACRO_4: KeyMacro = KeyMacro {
    tap: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
    hold: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
    ttap: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
    thold: FlashBlock {
        data: UnsafeCell::new([0; 4096]),
    },
};

pub const MACRO_LENGTH: u16 = 4096 - 2;

#[derive(Clone, Copy)]
pub struct ActiveMacro {
    flash: &'static FlashBlock,
    pub index: usize,
    pub macro_type: MacroType,
}

// #[derive(Clone)]
pub struct KeyboardMacros<C> 
where C: CountDown,
<C as cortex_m::prelude::_embedded_hal_timer_CountDown>::Time: From<fugit::Duration<u32, 1, 1000000>> {
    macros: [&'static KeyMacro; 4],
    active_macro: Option<ActiveMacro>,
    macro_state: MacroState,
    macro_timer: C,
}

impl<C> KeyboardMacros<C>
where C: CountDown,
<C as cortex_m::prelude::_embedded_hal_timer_CountDown>::Time: From<fugit::Duration<u32, 1, 1000000>>
 {
    pub fn new(timer: C) -> Self {
        Self {
            macros: [&MACRO_1, &MACRO_2, &MACRO_3, &MACRO_4],
            active_macro: None,
            macro_state: MacroState::default(),
            macro_timer: timer,
        }
    }

    pub fn read(&self, key: usize, macro_type: &MacroType) -> &[u8; 4096] {
        return self.macros[key].read(&macro_type);
    }

    pub fn write_flash(&self, key: usize, macro_type: &MacroType, data: &[u8; 4096]) {
        self.macros[key].write_flash(&macro_type, data);
    }

    pub fn initialize_flash(&self, key: usize, macro_type: &MacroType) {
        self.macros[key].initialize_flash(&macro_type);
    }

    pub fn initialize_all_flash(&self) {
        for key in self.macros.iter() {
            for t in MacroType::iter() {
                if !key.validate(&t) {
                    key.initialize_flash(&t);
                }
            }
        }
    }

    pub fn get_checksum(&self, key: usize, macro_type: &MacroType) -> u32 {
        self.macros[key].get_checksum(&macro_type)
    }

    pub fn set_checksum(&self, key: usize, macro_type: &MacroType) {
        self.macros[key].set_checksum(&macro_type)
    }

    pub fn get(&self, key: usize) -> &KeyMacro {
        &self.macros[key]
    }

    pub fn get_mut(&mut self, key: usize) -> &KeyMacro {
        &self.macros[key]
    }

    pub fn delay(&mut self, delay: MicrosDurationU32) {
        self.macro_timer.start(delay);
    }

    pub fn timer_ok(&mut self) -> bool {
        self.macro_timer.wait().is_ok()
    }

    pub fn initialize_macro(&mut self, key: usize, macro_type: &MacroType) -> Option<ActiveMacro> {
        let flash = self.macros[key].get_macro(&macro_type);

        self.active_macro = if flash.validate() {
            Some(
                ActiveMacro { flash, index: key, macro_type: *macro_type }
            )
        } else {
            None
        };

        self.delay(MicrosDurationU32::millis(0));
        self.macro_state = MacroState::default();

        self.active_macro
    }

    pub fn end_macro(&mut self) {
        self.active_macro = None;
        self.delay(MicrosDurationU32::millis(0));
        self.macro_state = MacroState::default();
    }
    
    pub fn get_active_macro(&self) -> Option<ActiveMacro> {
        self.active_macro
    }

    pub fn get_consuer(&mut self) -> Option<Consumer> {
        self.macro_state.consumer.take()
    }

    pub fn get_keys(&self) -> &[Keyboard; 256] {
        &self.macro_state.keys
    }

    pub fn get_backlight(&self) -> Option<RGB8> {
        self.macro_state.backlight
    }

pub fn read_macro(
    &mut self


    // is done
) -> bool {
    let mut delay_bytes;
    let mut delay = 0;
    let mut is_done = false;

    let macro_data = if let Some(active_macro) = self.active_macro {
        active_macro.flash.read()
    } else {
        self.end_macro();

        return true;
    };

    while delay == 0 {
        let current_command = MacroCommand::from(macro_data[self.macro_state.current_offset] >> 2);
        let delay_count = (macro_data[self.macro_state.current_offset] & 0b11) + 1;

        // info!("Command: {:?}, delay: {}", current_command as u8, delay_count);
        self.macro_state.current_offset += 1;

        delay_bytes = [0; 4];
        delay_bytes[0..delay_count as usize]
            .copy_from_slice(&macro_data[self.macro_state.current_offset..self.macro_state.current_offset + delay_count as usize]);
        delay = u32::from_le_bytes(delay_bytes);
        self.macro_state.current_offset += delay_count as usize;

        match current_command {
            MacroCommand::Empty => {
                if delay == 0 {
                    self.end_macro();

                    return true;
                }
            }
            MacroCommand::LoopBegin => {
                if self.macro_state.current_loop_index == 0 {
                    self.macro_state.current_loop_index += 1;
                    self.macro_state.loop_states[self.macro_state.current_loop_index - 1].loop_offset =
                        self.macro_state.current_offset - 1 - delay_count as usize;
                        self.macro_state.loop_states[self.macro_state.current_loop_index - 1].loop_iteration = 1;
                    delay = 0;
                } else if self.macro_state.loop_states[self.macro_state.current_loop_index - 1].loop_offset
                    == self.macro_state.current_offset - 1 - delay_count as usize
                {
                    self.macro_state.loop_states[self.macro_state.current_loop_index - 1].loop_iteration += 1;
                } else {
                    self.macro_state.current_loop_index += 1;
                    self.macro_state.loop_states[self.macro_state.current_loop_index - 1].loop_offset =
                        self.macro_state.current_offset - 1 - delay_count as usize;
                        self.macro_state.loop_states[self.macro_state.current_loop_index - 1].loop_iteration = 1;
                    delay = 0;
                }
            }
            MacroCommand::LoopEnd => {
                let loop_count = macro_data[self.macro_state.current_offset];

                if self.macro_state.loop_states[self.macro_state.current_loop_index - 1].loop_iteration < loop_count {
                    self.macro_state.current_offset = self.macro_state.loop_states[self.macro_state.current_loop_index - 1].loop_offset;
                } else {
                    self.macro_state.loop_states[self.macro_state.current_loop_index - 1] = LoopState::default();

                    self.macro_state.current_loop_index -= 1;
                    self.macro_state.current_offset += 1;
                }
            }
            MacroCommand::SetLed => {
                let mut color_bytes = [0; 3];
                color_bytes.copy_from_slice(&macro_data[self.macro_state.current_offset..self.macro_state.current_offset + 3]);
                self.macro_state.backlight = Some(RGB8::from(color_bytes));
                self.macro_state.current_offset += 3;
            }
            MacroCommand::ClearLed => {
                self.macro_state.backlight = None;
            }
            MacroCommand::KeyDown => {
                let key = Keyboard::from(macro_data[self.macro_state.current_offset]);
                self.macro_state.keys[macro_data[self.macro_state.current_offset] as usize] = key;
                self.macro_state.current_offset += 1;
            }
            MacroCommand::KeyUp => {
                self.macro_state.keys[macro_data[self.macro_state.current_offset] as usize] = Keyboard::NoEventIndicated;
                self.macro_state.current_offset += 1;
            }
            MacroCommand::KeyPress => {
                let key = Keyboard::from(macro_data[self.macro_state.current_offset]);
                if self.macro_state.command_memeory.command_iteration == 0 {
                    self.macro_state.keys[macro_data[self.macro_state.current_offset] as usize] = key;
                    delay_bytes[0..4].copy_from_slice(&macro_data[self.macro_state.current_offset + 1..self.macro_state.current_offset + 5]);
                    delay = u32::from_le_bytes(delay_bytes);

                    self.macro_state.current_offset -= 1 + delay_count as usize;
                    self.macro_state.command_memeory.command_iteration = 1;
                } else {
                    self.macro_state.keys[macro_data[self.macro_state.current_offset] as usize] = Keyboard::NoEventIndicated;
                    self.macro_state.command_memeory = CommandState::default();
                    self.macro_state.current_offset += 5;
                }
            }
            MacroCommand::ConsumerPress => {
                // TODO: FIX
                let consumer = Consumer::from(u16::from_le_bytes([
                    macro_data[self.macro_state.current_offset],
                    macro_data[self.macro_state.current_offset + 1],
                ]));
                if self.macro_state.command_memeory.command_iteration == 0 {
                    self.macro_state.consumer = Some(consumer);
                    delay_bytes[0..4].copy_from_slice(&macro_data[self.macro_state.current_offset + 2..self.macro_state.current_offset + 6]);
                    delay = u32::from_le_bytes(delay_bytes);

                    self.macro_state.current_offset -= 1 + delay_count as usize;
                    self.macro_state.command_memeory.command_iteration = 1;
                } else {
                    self.macro_state.consumer = Some(Consumer::Unassigned);
                    self.macro_state.command_memeory = CommandState::default();
                    self.macro_state.current_offset += 6;
                }
            }
            MacroCommand::TypeString => {
                let temp_offset = self.macro_state.current_offset - 1 - delay_count as usize;
                let temp_delay = delay;

                delay_bytes[0..4].copy_from_slice(&macro_data[self.macro_state.current_offset..self.macro_state.current_offset + 4]);
                delay = u32::from_le_bytes(delay_bytes);
                self.macro_state.current_offset += 4;

                self.macro_state.current_offset += self.macro_state.command_memeory.command_offset;

                if macro_data[self.macro_state.current_offset] != 0x00 {
                    if self.macro_state.command_memeory.command_iteration == 0 {
                        let (key, shift) = key_from_ascii(macro_data[self.macro_state.current_offset] as char);
                        self.macro_state.keys[macro_data[self.macro_state.current_offset] as usize] = key;

                        if let Some(shift) = shift {
                            if shift {
                                self.macro_state.keys[0xE1] = Keyboard::LeftShift;
                            } else {
                                self.macro_state.keys[0xE1] = Keyboard::NoEventIndicated;
                            }
                        }

                        self.macro_state.current_offset = temp_offset;
                        self.macro_state.command_memeory.command_iteration = 1;
                    } else {
                        self.macro_state.keys[macro_data[self.macro_state.current_offset] as usize] = Keyboard::NoEventIndicated;

                        self.macro_state.command_memeory.command_offset += 1;

                        if macro_data[self.macro_state.current_offset + 1] != macro_data[self.macro_state.current_offset] {
                            let (key, shift) = key_from_ascii(macro_data[self.macro_state.current_offset + 1] as char);
                            self.macro_state.keys[macro_data[self.macro_state.current_offset + 1] as usize] = key;

                            if let Some(shift) = shift {
                                if shift {
                                    self.macro_state.keys[0xE1] = Keyboard::LeftShift;
                                } else {
                                    self.macro_state.keys[0xE1] = Keyboard::NoEventIndicated;
                                }
                            }
                        } else {
                            self.macro_state.command_memeory.command_iteration = 0;
                        }

                        self.macro_state.current_offset = temp_offset;
                    }
                } else {
                    self.macro_state.command_memeory = CommandState::default();
                    self.macro_state.keys[0xE1] = Keyboard::NoEventIndicated;
                    delay = temp_delay;
                    self.macro_state.current_offset += 1;
                }
            }
            MacroCommand::Chord => {
                let temp_offset = self.macro_state.current_offset - 1 - delay_count as usize;

                if self.macro_state.command_memeory.command_iteration == 0 {
                    delay_bytes[0..4].copy_from_slice(&macro_data[self.macro_state.current_offset..self.macro_state.current_offset + 4]);
                    delay = u32::from_le_bytes(delay_bytes);
                    self.macro_state.current_offset += 4;

                    while macro_data[self.macro_state.current_offset] != 0x00 {
                        let key = Keyboard::from(macro_data[self.macro_state.current_offset]);
                        self.macro_state.keys[macro_data[self.macro_state.current_offset] as usize] = key;
                        self.macro_state.current_offset += 1;
                    }

                    self.macro_state.current_offset = temp_offset;
                    self.macro_state.command_memeory.command_iteration = 1;
                } else {
                    self.macro_state.current_offset += 3;

                    while macro_data[self.macro_state.current_offset] != 0x00 {
                        self.macro_state.keys[macro_data[self.macro_state.current_offset] as usize] = Keyboard::NoEventIndicated;
                        self.macro_state.current_offset += 1;
                    }

                    self.macro_state.command_memeory = CommandState::default();
                }
            }
        }
    }

    self.delay(MicrosDurationU32::micros(delay));
    is_done
}
}

#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct LoopState {
    loop_offset: usize,
    loop_iteration: u8,
}

#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct CommandState {
    command_offset: usize,
    command_iteration: u8,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MacroState {
    pub current_offset: usize,
    pub keys: [Keyboard; 256],
    pub consumer: Option<Consumer>,
    pub backlight: Option<RGB8>,
    pub loop_states: [LoopState; 256],
    pub current_loop_index: usize,
    pub command_memeory: CommandState,
}

impl Default for MacroState {
    fn default() -> Self {
        Self {
            current_offset: 0,
            keys: [Keyboard::NoEventIndicated; 256],
            consumer: None,
            backlight: None,
            loop_states: [LoopState::default(); 256],
            current_loop_index: 0,
            command_memeory: CommandState::default(),
        }
    }
}

fn key_from_ascii(char: char) -> (Keyboard, Option<bool>) {
    match char {
        '`' => (Keyboard::Grave, Some(false)),
        '1' => (Keyboard::Keyboard1, Some(false)),
        '2' => (Keyboard::Keyboard2, Some(false)),
        '3' => (Keyboard::Keyboard3, Some(false)),
        '4' => (Keyboard::Keyboard4, Some(false)),
        '5' => (Keyboard::Keyboard5, Some(false)),
        '6' => (Keyboard::Keyboard6, Some(false)),
        '7' => (Keyboard::Keyboard7, Some(false)),
        '8' => (Keyboard::Keyboard8, Some(false)),
        '9' => (Keyboard::Keyboard9, Some(false)),
        '0' => (Keyboard::Keyboard0, Some(false)),
        '-' => (Keyboard::Minus, Some(false)),
        '=' => (Keyboard::Equal, Some(false)),
        'q' => (Keyboard::Q, Some(false)),
        'w' => (Keyboard::W, Some(false)),
        'e' => (Keyboard::E, Some(false)),
        'r' => (Keyboard::R, Some(false)),
        't' => (Keyboard::T, Some(false)),
        'y' => (Keyboard::Y, Some(false)),
        'u' => (Keyboard::U, Some(false)),
        'i' => (Keyboard::I, Some(false)),
        'o' => (Keyboard::O, Some(false)),
        'p' => (Keyboard::P, Some(false)),
        '[' => (Keyboard::LeftBrace, Some(false)),
        ']' => (Keyboard::RightBrace, Some(false)),
        '\\' => (Keyboard::Backslash, Some(false)),
        'a' => (Keyboard::A, Some(false)),
        's' => (Keyboard::S, Some(false)),
        'd' => (Keyboard::D, Some(false)),
        'f' => (Keyboard::F, Some(false)),
        'g' => (Keyboard::G, Some(false)),
        'h' => (Keyboard::H, Some(false)),
        'j' => (Keyboard::J, Some(false)),
        'k' => (Keyboard::K, Some(false)),
        'l' => (Keyboard::L, Some(false)),
        ';' => (Keyboard::Semicolon, Some(false)),
        '\'' => (Keyboard::Apostrophe, Some(false)),
        'z' => (Keyboard::Z, Some(false)),
        'x' => (Keyboard::X, Some(false)),
        'c' => (Keyboard::C, Some(false)),
        'v' => (Keyboard::V, Some(false)),
        'b' => (Keyboard::B, Some(false)),
        'n' => (Keyboard::N, Some(false)),
        'm' => (Keyboard::M, Some(false)),
        ',' => (Keyboard::Comma, Some(false)),
        '.' => (Keyboard::Dot, Some(false)),
        '/' => (Keyboard::ForwardSlash, Some(false)),

        '~' => (Keyboard::Grave, Some(true)),
        '!' => (Keyboard::Keyboard1, Some(true)),
        '@' => (Keyboard::Keyboard2, Some(true)),
        '#' => (Keyboard::Keyboard3, Some(true)),
        '$' => (Keyboard::Keyboard4, Some(true)),
        '%' => (Keyboard::Keyboard5, Some(true)),
        '^' => (Keyboard::Keyboard6, Some(true)),
        '&' => (Keyboard::Keyboard7, Some(true)),
        '*' => (Keyboard::Keyboard8, Some(true)),
        '(' => (Keyboard::Keyboard9, Some(true)),
        ')' => (Keyboard::Keyboard0, Some(true)),
        '_' => (Keyboard::Minus, Some(true)),
        '+' => (Keyboard::Equal, Some(true)),
        'Q' => (Keyboard::Q, Some(true)),
        'W' => (Keyboard::W, Some(true)),
        'E' => (Keyboard::E, Some(true)),
        'R' => (Keyboard::R, Some(true)),
        'T' => (Keyboard::T, Some(true)),
        'Y' => (Keyboard::Y, Some(true)),
        'U' => (Keyboard::U, Some(true)),
        'I' => (Keyboard::I, Some(true)),
        'O' => (Keyboard::O, Some(true)),
        'P' => (Keyboard::P, Some(true)),
        '{' => (Keyboard::LeftBrace, Some(true)),
        '}' => (Keyboard::RightBrace, Some(true)),
        '|' => (Keyboard::Backslash, Some(true)),
        'A' => (Keyboard::A, Some(true)),
        'S' => (Keyboard::S, Some(true)),
        'D' => (Keyboard::D, Some(true)),
        'F' => (Keyboard::F, Some(true)),
        'G' => (Keyboard::G, Some(true)),
        'H' => (Keyboard::H, Some(true)),
        'J' => (Keyboard::J, Some(true)),
        'K' => (Keyboard::K, Some(true)),
        'L' => (Keyboard::L, Some(true)),
        ':' => (Keyboard::Semicolon, Some(true)),
        '"' => (Keyboard::Apostrophe, Some(true)),
        'Z' => (Keyboard::Z, Some(true)),
        'X' => (Keyboard::X, Some(true)),
        'C' => (Keyboard::C, Some(true)),
        'V' => (Keyboard::V, Some(true)),
        'B' => (Keyboard::B, Some(true)),
        'N' => (Keyboard::N, Some(true)),
        'M' => (Keyboard::M, Some(true)),
        '<' => (Keyboard::Comma, Some(true)),
        '>' => (Keyboard::Dot, Some(true)),
        '?' => (Keyboard::ForwardSlash, Some(true)),

        ' ' => (Keyboard::Space, None),
        '\t' => (Keyboard::Tab, None),
        '\n' => (Keyboard::ReturnEnter, None),

        _ => (Keyboard::NoEventIndicated, None),
    }
}

pub fn initialize_flash<C: embedded_hal::timer::CountDown>(macros: KeyboardMacros<C>) where <C as cortex_m::prelude::_embedded_hal_timer_CountDown>::Time: core::convert::From<fugit::Duration<u32, 1, 1000000>> {
    if !CONFIG.validate() {
        CONFIG.initialize_flash();
    }

    macros.initialize_all_flash();
}