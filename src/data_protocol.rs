use core::mem;

pub const PROTOCOL_VERSION: u16 = 1;

#[repr(u8)]
pub enum DataCommand {
    None = 0x00,
    GetProtocolVersion = 0x01,
    ReadMacro = 0x02,
    WriteMacro = 0x03,
    ValidateMacro = 0x04,
    GetLed = 0x05,
    SetLed = 0x06,


    //...
    GetPortName = 0xFD,
    EnterBootloader = 0xFE,
    Error = 0xFF
}

impl DataCommand {
    pub fn from_u8(n: u8) -> Option<DataCommand> {
        if (n <= 0x06) || (n >= 0xFD) {
            Some(unsafe { mem::transmute(n) })
        } else {
            None
        }
    }
}
