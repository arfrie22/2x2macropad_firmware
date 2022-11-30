use core::mem;

pub const PROTOCOL_VERSION: u16 = 1;

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum DataCommand {
    None = 0x00,
    GetProtocolVersion = 0x01,
    ReadMacro = 0x02,
    WriteMacro = 0x03,
    ValidateMacro = 0x04,
    GetLed = 0x05,
    SetLed = 0x06,


    // Extra commands not included in the count
    EnterBootloader = 0xFE,
    Error = 0xFF
}
const PROTOCOL_COMMAND_COUNT: u8 = 9;

impl DataCommand {
    pub fn from_u8(n: u8) -> Option<DataCommand> {
        if n < PROTOCOL_COMMAND_COUNT || n >= 0xFE {
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


impl LedCommand {
    pub fn from_u8(n: u8) -> Option<LedCommand> {
        if n <= 0x05 || (0x11..=0x15).contains(&n) || (0x21..=0x26).contains(&n) || n == 0xFF {
            Some(unsafe { mem::transmute(n) })
        } else {
            None
        }
    }
}