//!Raw HID
use usbd_human_interface_device::hid_class::descriptor::HidProtocol;
use core::default::Default;
use delegate::delegate;
use embedded_time::duration::Milliseconds;
use packed_struct::prelude::*;
use usb_device::bus::{InterfaceNumber, StringIndex, UsbBus};
use usb_device::class_prelude::DescriptorWriter;

use usbd_human_interface_device::hid_class::prelude::*;
use usbd_human_interface_device::interface::raw::{RawInterface, RawInterfaceConfig};
use usbd_human_interface_device::interface::{InterfaceClass, WrappedInterface, WrappedInterfaceConfig};
use usbd_human_interface_device::UsbHidError;

/// HID Mouse report descriptor conforming to the Boot specification
///
/// This aims to be compatible with BIOS and other reduced functionality USB hosts
///
/// This is defined in Appendix B.2 & E.10 of [Device Class Definition for Human
/// Interface Devices (Hid) Version 1.11](<https://www.usb.org/sites/default/files/hid1_11.pdf>)
#[rustfmt::skip]
pub const GENERIC_HID_IN_OUT_REPORT_DESCRIPTOR: &[u8] = &[
    0x06, 0x00, 0xFF, // Usage Page (Vender),
    0x09, 0x01,       // Usage (Generic In/Out),
    0xA1, 0x01,       // Collection (Application),
    0x09, 0x02,       //   Usage (Input),
    0x15, 0x00,       //     Logical Minimum (0),
    0x26, 0xFF, 0x00, //     Logical Maximum (255),
    0x75, 0x08,       //     Report Size (8),
    0x95, 0x40,       //     Report Count (64),
    0x81, 0x02,       //     Input (Data, Variable, Absolute),
    0x09, 0x03,       //   Usage (Output),
    0x15, 0x00,       //     Logical Minimum (0),
    0x26, 0xFF, 0x00, //     Logical Maximum (255),
    0x75, 0x08,       //     Report Size (8),
    0x95, 0x40,       //     Report Count (64),
    0x91, 0x02,       //     Output (Data, Variable, Absolute),
    0xC0,             // End Collection
];

#[derive(Clone, Copy, Debug, Eq, PartialEq, PackedStruct)]
#[packed_struct(endian = "lsb", size_bytes = "64")]
pub struct GenericInOutMsg {
    #[packed_field]
    pub packet: [u8; 64]

}

impl Default for GenericInOutMsg {
    fn default() -> GenericInOutMsg {
        GenericInOutMsg {
            packet: [0u8; 64]
        }
    }
}

pub struct GenericInOutInterface<'a, B: UsbBus> {
    inner: RawInterface<'a, B>,
}

impl<'a, B: UsbBus> GenericInOutInterface<'a, B> {
    pub fn write_report(&self, report: &GenericInOutMsg) -> Result<(), UsbHidError> {
        self.inner
            .write_report(&report.packet)
            .map(|_| ())
            .map_err(UsbHidError::from)
    }
    pub fn read_report(&self) -> usb_device::Result<GenericInOutMsg> {
        let mut report = GenericInOutMsg::default();
        match self.inner.read_report(&mut report.packet) {
            Err(e) => Err(e),
            Ok(_) => Ok(report),
        }
    }

    pub fn default_config() -> WrappedInterfaceConfig<Self, RawInterfaceConfig<'a>> {
        WrappedInterfaceConfig::new(
            RawInterfaceBuilder::new(GENERIC_HID_IN_OUT_REPORT_DESCRIPTOR)
                .description("GenericInOut")
                .in_endpoint(UsbPacketSize::Bytes64, Milliseconds(5))
                .unwrap()
                .with_out_endpoint(UsbPacketSize::Bytes64, Milliseconds(5))
                .unwrap()
                .build(),
            (),
        )
    }
}

impl<'a, B: UsbBus> InterfaceClass<'a> for GenericInOutInterface<'a, B> {
    delegate! {
        to self.inner{
           fn report_descriptor(&self) -> &'_ [u8];
           fn id(&self) -> InterfaceNumber;
           fn write_descriptors(&self, writer: &mut DescriptorWriter) -> usb_device::Result<()>;
           fn get_string(&self, index: StringIndex, _lang_id: u16) -> Option<&'_ str>;
           fn reset(&mut self);
           fn set_report(&mut self, data: &[u8]) -> usb_device::Result<()>;
           fn get_report(&mut self, data: &mut [u8]) -> usb_device::Result<usize>;
           fn get_report_ack(&mut self) -> usb_device::Result<()>;
           fn set_idle(&mut self, report_id: u8, value: u8);
           fn get_idle(&self, report_id: u8) -> u8;
           fn set_protocol(&mut self, protocol: HidProtocol);
           fn get_protocol(&self) -> HidProtocol;
        }
    }
}

impl<'a, B: UsbBus> WrappedInterface<'a, B, RawInterface<'a, B>> for GenericInOutInterface<'a, B> {
    fn new(interface: RawInterface<'a, B>, _: ()) -> Self {
        Self { inner: interface }
    }
}
