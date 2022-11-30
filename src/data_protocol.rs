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
    ValidateMacro = 0x04,
    ReadConfig = 0x05,
    WriteConfig = 0x06,
    GetLed = 0x07,
    SetLed = 0x08,


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
#[derive(Debug, Clone, Copy)]
pub enum LedSetCommand {
    None = 0x00,

    // Single LED Control
    SetSingleBaseColor = 0x01,
    SetSingleEffect = 0x02,
    SetSingleBrightness = 0x03,
    SetSingleEffectSpeed = 0x04,
    SetSingleEffectOffset = 0x05,

    // Multiple LED Control
    SetMultipleBaseColor = 0x11,
    SetMultipleEffect = 0x12,
    SetMultipleBrightness = 0x13,
    SetMultipleEffectSpeed = 0x14,
    SetMultipleEffectOffset = 0x15,

    // All LED Control
    SetAllBaseColor = 0x21,
    SetAllEffect = 0x22,
    SetAllBrightness = 0x23,
    SetAllEffectSpeed = 0x24,
    SetAllEffectOffset = 0x25,
    SetAllEffectOffsetSpaced = 0x26,

    //...
    Error = 0xFF
}


impl LedSetCommand {
    pub fn from_u8(n: u8) -> Option<LedSetCommand> {
        if n <= 0x05 || (0x11..=0x15).contains(&n) || (0x21..=0x26).contains(&n) || n == 0xFF {
            Some(unsafe { mem::transmute(n) })
        } else {
            None
        }
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum LedGetCommand {
    None = 0x00,

    // Single LED Control
    GetSingleBaseColor = 0x01,
    GetSingleEffect = 0x02,
    GetSingleBrightness = 0x03,
    GetSingleEffectSpeed = 0x04,
    GetSingleEffectOffset = 0x05,

    // All LED Control
    GetAllBaseColor = 0x11,
    GetAllEffect = 0x12,
    GetAllBrightness = 0x13,
    GetAllEffectSpeed = 0x14,
    GetAllEffectOffset = 0x15,

    //...
    Error = 0xFF
}


impl LedGetCommand {
    pub fn from_u8(n: u8) -> Option<LedGetCommand> {
        if n <= 0x05 || (0x11..=0x15).contains(&n) || n == 0xFF {
            Some(unsafe { mem::transmute(n) })
        } else {
            None
        }
    }
}