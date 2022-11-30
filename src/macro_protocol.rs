use core::mem;

use strum::EnumCount;

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, EnumCount)]
pub enum MacroCommand {
    CommandTerminator = 0x00,
    CommandDelay = 0x01,
    CommandPressKey = 0x02,
    CommandReleaseKey = 0x03,
    CommandConsumer = 0x04,
    CommandLed = 0x05,
}

impl MacroCommand {
    pub fn from_u8(n: u8) -> Option<MacroCommand> {
        if n < MacroCommand::COUNT as u8 {
            Some(unsafe { mem::transmute(n) })
        } else {
            None
        }
    }
}