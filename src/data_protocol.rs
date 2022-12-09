use core::mem;

use strum::EnumCount;

pub const PROTOCOL_VERSION: u16 = 1;

#[repr(u8)]
#[derive(Debug, Clone, Copy, EnumCount)]
pub enum DataCommand {
    None = 0x00,
    GetProtocolVersion = 0x01,
    ReadMacro = 0x02,
    WriteMacro = 0x03,
    ClearMacro = 0x04,
    ValidateMacro = 0x05,
    ReadConfig = 0x06,
    WriteConfig = 0x07,
    GetLed = 0x08,
    SetLed = 0x09,
    ReadKeyConfig = 0x0A,
    WriteKeyConfig = 0x0B,


    // Extra commands not included in the count
    EnterBootloader = 0xFE,
    Error = 0xFF
}

impl DataCommand {
    pub fn from_u8(n: u8) -> Option<DataCommand> {
        if !(DataCommand::COUNT as u8 - 2..0xFE).contains(&n) {
            Some(unsafe { mem::transmute(n) })
        } else {
            None
        }
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, EnumCount)]
pub enum ConfigElements {
    Version = 0x00,
    TapSpeed = 0x01,
    HoldSpeed = 0x02,
    DefaultDelay = 0x03,

    //...
    Error = 0xFF
}

impl ConfigElements {
    pub fn from_u8(n: u8) -> Option<ConfigElements> {
        if n < ConfigElements::COUNT as u8 - 1 || n == 0xFF {
            Some(unsafe { mem::transmute(n) })
        } else {
            None
        }
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, EnumCount)]
pub enum KeyConfigElements {
    SingleTapMode = 0x00,

    //...
    Error = 0xFF
}

impl KeyConfigElements {
    pub fn from_u8(n: u8) -> Option<KeyConfigElements> {
        if n < KeyConfigElements::COUNT as u8 - 1 || n == 0xFF {
            Some(unsafe { mem::transmute(n) })
        } else {
            None
        }
    }
}



#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum LedCommand {
    None = 0x00,

    // Single LED Control
    BaseColor = 0x01,
    Effect = 0x02,
    Brightness = 0x03,
    EffectSpeed = 0x04,
    EffectOffset = 0x05,

    //...
    Error = 0xFF
}


impl LedCommand {
    pub fn from_u8(n: u8) -> Option<LedCommand> {
        if n <= 0x05 || n == 0xFF {
            Some(unsafe { mem::transmute(n) })
        } else {
            None
        }
    }
}