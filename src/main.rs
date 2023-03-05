#![no_std]
#![no_main]

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

use crc::{Crc, CRC_32_CKSUM};
use macropad_protocol::data_protocol::BuildInfoElements;
use macropad_protocol::data_protocol::ConfigElements;
use macropad_protocol::data_protocol::KeyConfigElements;
use macropad_protocol::data_protocol::KeyMode;
use macropad_protocol::data_protocol::LedCommand;

use hal::timer::CountDown;
use led_effect::LedConfig;
use led_effect::STRIP_LEN;
use macropad_protocol::data_protocol::LedEffect;
use macropad_protocol::macro_protocol::MacroCommand;
use macros::Config;
use macros::MACRO_LENGTH;
use macros::MacroType;
use packed_struct::prelude::*;
use smart_leds::gamma;

use core::cell::UnsafeCell;
use core::convert::Infallible;
use core::default::Default;

use hal::rom_data::reset_to_usb_boot;
use macropad_protocol::data_protocol::{DataCommand, PROTOCOL_VERSION};
use rp2040_hal as hal;

use cortex_m::delay::Delay;
use cortex_m::prelude::*;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::*;

use fugit::{ExtU32, MicrosDurationU32};
use hal::entry;

use hal::pac;
// Pull in any important traits
use hal::prelude::*;
use panic_probe as _;

use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_human_interface_device::device::consumer::{
    ConsumerControlInterface, MultipleConsumerReport,
};
use usbd_human_interface_device::device::keyboard::NKROBootKeyboardInterface;
use usbd_human_interface_device::page::Consumer;
use usbd_human_interface_device::page::Keyboard;
use usbd_human_interface_device::prelude::*;

use strum::IntoEnumIterator;
use strum_macros::EnumIter;

// PIOExt for the split() method that is needed to bring
// PIO0 into useable form for Ws2812:
use hal::pio::PIOExt;

// Import useful traits to handle the ws2812 LEDs:
use smart_leds::{brightness, SmartLedsWrite, RGB8};

// Import the actual crate to handle the Ws2812 protocol:
use ws2812_pio::Ws2812;

pub mod led_effect;

pub mod key_matrix;

#[repr(C, align(4096))]
pub struct FlashBlock {
    data: UnsafeCell<[u8; 4096]>,
}

use crate::hid::raw_hid;
use crate::key_matrix::Matrix;
use crate::macros::KeyboardMacros;
use crate::rp2040_flash::flash;

pub mod rp2040_flash;
pub mod macros;
pub mod hid;

use hid::raw_hid::{GenericInOutInterface, GenericInOutMsg};

const KEYBOARD_POLL: MicrosDurationU32 = MicrosDurationU32::millis(1);
const CONSUMER_POLL: MicrosDurationU32 = MicrosDurationU32::millis(1);
const RAW_HID_POLL: MicrosDurationU32 = MicrosDurationU32::millis(1);
const LED_POLL: MicrosDurationU32 = MicrosDurationU32::millis(16);

const KEY_COLS: usize = 2;
const KEY_ROWS: usize = 2;
const KEY_COUNT: usize = KEY_COLS * KEY_ROWS;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    info!("Starting");

    //USB
    static mut USB_ALLOC: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    //Safety: interrupts not enabled yet
    let usb_alloc = unsafe {
        USB_ALLOC = Some(UsbBusAllocator::new(hal::usb::UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        )));
        USB_ALLOC.as_ref().unwrap()
    };

    let mut composite = UsbHidClassBuilder::new()
        .add_interface(
            {
                let mut cfg = raw_hid::GenericInOutInterface::default_config();
                cfg.inner_config.description = Some("HID Data Control Interface");
                cfg
            },
        )
        .add_interface(
            {
                usbd_human_interface_device::interface::WrappedInterfaceConfig::new(
                    usbd_human_interface_device::interface::managed::ManagedInterfaceConfig::new(
                        usbd_human_interface_device::interface::raw::RawInterfaceBuilder::new(usbd_human_interface_device::device::keyboard::NKRO_BOOT_KEYBOARD_REPORT_DESCRIPTOR)
                            .description("NKRO Keyboard")
                            .boot_device(usbd_human_interface_device::hid_class::prelude::InterfaceProtocol::Keyboard)
                            .idle_default(embedded_time::duration::Milliseconds(500))
                            .unwrap()
                            .in_endpoint(usbd_human_interface_device::hid_class::UsbPacketSize::Bytes32, embedded_time::duration::Milliseconds(KEYBOARD_POLL.to_millis()))
                            .unwrap()
                            .with_out_endpoint(usbd_human_interface_device::hid_class::UsbPacketSize::Bytes8, embedded_time::duration::Milliseconds(100))
                            .unwrap()
                            .build(),
                    ),
                    (),
                )
            },
        )
        .add_interface(
            {
                let mut cfg = usbd_human_interface_device::device::consumer::ConsumerControlInterface::default_config();
                cfg.inner_config.in_endpoint.poll_interval = CONSUMER_POLL.to_millis() as u8;
                cfg
            },
        )
        //Build
        .build(usb_alloc);

    //https://pid.codes
    let mut usb_dev = UsbDeviceBuilder::new(usb_alloc, UsbVidPid(0x554D, 0x2020))
        .manufacturer("UMass Amherst All-Campus Makerspace")
        .product("2x2 Macropad")
        .serial_number("MACROPAD")
        .build();

    let mut consumer_input_timer = timer.count_down();
    consumer_input_timer.start(CONSUMER_POLL);
    let mut last_consumer_report = MultipleConsumerReport::default();

    let mut keyboard_input_timer = timer.count_down();
    keyboard_input_timer.start(KEYBOARD_POLL);

    let mut raw_hid_timer = timer.count_down();
    raw_hid_timer.start(RAW_HID_POLL);

    let mut tick_timer = timer.count_down();
    tick_timer.start(1.millis());

    let mut led_timer = timer.count_down();
    led_timer.start(LED_POLL);

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let mut ws = Ws2812::new(
        pins.gpio0.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let psm = pac.PSM;

    // Reset core1 so it's guaranteed to be running
    // ROM code, waiting for the wakeup sequence
    psm.frce_off.modify(|_, w| w.proc1().set_bit());
    while !psm.frce_off.read().proc1().bit_is_set() {
        cortex_m::asm::nop();
    }
    psm.frce_off.modify(|_, w| w.proc1().clear_bit());

    let mut backlight: [RGB8; STRIP_LEN] = [(0, 0, 0).into(); STRIP_LEN];
    let mut time = 0u32;

    for (_, led) in backlight.iter_mut().enumerate() {
        *led = (0, 0, 0).into();
    }

    let mut config = Config::get_config();
    ws.write(brightness(
        backlight.iter().copied(),
        config.led_config.brightness,
    ))
    .unwrap();

    let mut keys = [Keyboard::NoEventIndicated; 260];
    let mut key_change = false;
    let mut consumers = [Consumer::Unassigned; 4];
    let mut consumer_change = false;
    
    let mut matrix = Matrix::new([
        pins.gpio19.into_pull_down_input().into(),
        pins.gpio18.into_pull_down_input().into(),
    ],
    [
        pins.gpio21.into_push_pull_output().into(),
        pins.gpio20.into_push_pull_output().into(),
    ], 
    timer.count_down(),
    [
        timer.count_down(),
        timer.count_down(),
        timer.count_down(),
        timer.count_down()
    ]);
    
    let mut keyboard_macros = KeyboardMacros::new(timer.count_down());

    loop {
        let matrix_scan = matrix.scan();
        let key_states = matrix.get_key_states(&config);

        if !key_change && !consumer_change && keyboard_macros.timer_ok() {
            if let Some(active_macro) = keyboard_macros.get_active_macro() {
                if keyboard_macros.read_macro() {
                    matrix.finish();
                }

                keys[0..256].copy_from_slice(keyboard_macros.get_keys());

                if let Some(consumer) = keyboard_macros.get_consuer() {
                    consumers[keyboard_macros.get_active_macro().unwrap().index] = consumer;
                    consumer_change = true;
                }

                key_change = true;
            } else if let Some((key, macro_type)) = matrix.get_active_key() {
                if keyboard_macros.initialize_macro(key, &macro_type).is_none() {
                    matrix.finish();
                }
            }
        }

        if keyboard_input_timer.wait().is_ok() {
            for (i, key) in matrix_scan.iter().enumerate() {
                if config.key_configs[i].key_mode == KeyMode::KeyboardMode {
                    if *key {
                        keys[259 - i] = Keyboard::from(config.key_configs[i].keyboard_data);
                    } else {
                        keys[259 - i] = Keyboard::NoEventIndicated;
                    }
                }
            }

            let keyboard = composite.interface::<NKROBootKeyboardInterface<'_, _>, _>();
            match keyboard.write_report(&keys) {
                Err(UsbHidError::WouldBlock) => {}
                Err(UsbHidError::Duplicate) => {
                    key_change = false;
                }
                Ok(_) => {
                    key_change = false;
                }
                Err(e) => {
                    core::panic!("Failed to write keyboard report: {:?}", e)
                }
            };

            // info!("Wrote keyboard report, keychange: {}", key_change);
        }

        if consumer_input_timer.wait().is_ok() {
            for (i, key) in matrix_scan.iter().enumerate() {
                if config.key_configs[i].key_mode == KeyMode::ConsumerMode {
                    if *key {
                        consumers[i] = Consumer::from(config.key_configs[i].consumer_data);
                    } else {
                        consumers[i] = Consumer::Unassigned;
                    }
                }
            }

            let consumer_report = MultipleConsumerReport { codes: consumers };

            if last_consumer_report != consumer_report {
                let consumer = composite.interface::<ConsumerControlInterface<'_, _>, _>();
                match consumer.write_report(&consumer_report) {
                    Err(UsbError::WouldBlock) => {}
                    Ok(_) => {
                        consumer_change = false;
                        last_consumer_report = consumer_report;
                    }
                    Err(e) => {
                        core::panic!("Failed to write consumer report: {:?}", e)
                    }
                };
            }
        }

        //Tick once per ms
        if tick_timer.wait().is_ok() {
            //Process any managed functionality
            match composite
                .interface::<NKROBootKeyboardInterface<'_, _>, _>()
                .tick()
            {
                Err(UsbHidError::WouldBlock) => {}
                Ok(_) => {}
                Err(e) => {
                    core::panic!("Failed to process keyboard tick: {:?}", e)
                }
            };
        }

        if usb_dev.poll(&mut [&mut composite]) {
            let keyboard = composite.interface::<NKROBootKeyboardInterface<'_, _>, _>();
            match keyboard.read_report() {
                Err(UsbError::WouldBlock) => {}
                Err(e) => {
                    core::panic!("Failed to read keyboard report: {:?}", e)
                }
                Ok(_) => {}
            }

            let raw_hid = composite.interface::<GenericInOutInterface<'_, _>, _>();
            match raw_hid.read_report() {
                Err(UsbError::WouldBlock) => {}
                Err(e) => {
                    core::panic!("Failed to read raw_hid report: {:?}", e)
                }
                Ok(data) => {
                    let data = parse_command(&data, &mut config, &timer, &mut keyboard_macros);

                    match raw_hid.write_report(&data) {
                        Err(UsbHidError::WouldBlock) => {}
                        Err(UsbHidError::Duplicate) => {}
                        Ok(_) => {}
                        Err(e) => {
                            core::panic!("Failed to write raw hid report: {:?}", e)
                        }
                    };
                }
            }
        }

        if led_timer.wait().is_ok() {
            config.led_config.update(&mut backlight, &timer);

            if let Some(key) = keyboard_macros.get_active_macro() {
                if let Some(led) = keyboard_macros.get_backlight() {
                    backlight[key.index] = led;
                }
            }

            for (i, key) in matrix_scan.iter().enumerate() {
                if *key
                    && (config.key_configs[i].key_mode == KeyMode::KeyboardMode
                        || config.key_configs[i].key_mode == KeyMode::ConsumerMode)
                {
                    backlight[i] = config.key_configs[i].key_color;
                }
            }

            time = time.wrapping_add(1);

            ws.write(brightness(
                gamma(backlight.iter().copied()),
                config.led_config.brightness,
            ))
            .unwrap();
        };
    }
}

fn parse_command<C: embedded_hal::timer::CountDown>(
    data: &GenericInOutMsg,
    config: &mut Config,
    timer: &hal::Timer,
    keyboard_macros: &mut KeyboardMacros<C>,
) -> GenericInOutMsg where <C as cortex_m::prelude::_embedded_hal_timer_CountDown>::Time: core::convert::From<fugit::Duration<u32, 1, 1000000>> {
    let mut output = data.packet;
    let command = DataCommand::from(output[0]);

    match command {
        DataCommand::None => {}

        DataCommand::GetProtocolVersion => {
            output[1..3].copy_from_slice(&PROTOCOL_VERSION.to_le_bytes());
        }

        DataCommand::ReadMacro => {
            let index = (output[1] >> 2) as usize;
            let t = (output[1] & 0b11) as usize;
            if index < KEY_COUNT && t < 4 {
                let offset = u16::from_le_bytes(output[2..4].try_into().unwrap());
                if offset < MACRO_LENGTH {
                    let length = output[4] as usize;
                    if length > 0 && length < 60 && (offset + (length as u16)) < MACRO_LENGTH {
                        let t = match t {
                            1 => MacroType::Hold,
                            2 => MacroType::DoubleTap,
                            3 => MacroType::TapHold,
                            _ => MacroType::Tap,
                        };
                        let macro_data: &[u8; 4096] = keyboard_macros.read(index, &t);
                        output[5..].copy_from_slice(
                            &macro_data[(offset) as usize..(offset as usize + length)],
                        );
                    } else {
                        output[0] = DataCommand::Error as u8;
                    }
                } else {
                    output[0] = DataCommand::Error as u8;
                }
            } else {
                output[0] = DataCommand::Error as u8;
            }
        }

        DataCommand::WriteMacro => {
            let index = (output[1] >> 2) as usize;
            let t = (output[1] & 0b11) as usize;
            if index < KEY_COUNT && t < 4 {
                let offset = u16::from_le_bytes(output[2..4].try_into().unwrap());
                if offset < MACRO_LENGTH {
                    let length = output[4] as usize;
                    if length > 0 && length < 60 && (offset + (length as u16)) < MACRO_LENGTH {
                        let t = match t {
                            0 => MacroType::Tap,
                            1 => MacroType::Hold,
                            2 => MacroType::DoubleTap,
                            3 => MacroType::TapHold,

                            _ => MacroType::Tap,
                        };
                        let mut macro_data: [u8; 4096] = keyboard_macros.read(index, &t).clone();
                        macro_data[(offset) as usize..(offset as usize + length)]
                            .copy_from_slice(&output[5..]);
                        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
                        keyboard_macros.write_flash(index, &t, &macro_data);
                        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
                    } else {
                        output[0] = DataCommand::Error as u8;
                    }
                } else {
                    output[0] = DataCommand::Error as u8;
                }
            } else {
                output[0] = DataCommand::Error as u8;
            }
        }

        DataCommand::ClearMacro => {
            let index = (output[1] >> 2) as usize;
            let t = (output[1] & 0b11) as usize;
            if index < KEY_COUNT && t < 4 {
                let t = match t {
                    0 => MacroType::Tap,
                    1 => MacroType::Hold,
                    2 => MacroType::DoubleTap,
                    3 => MacroType::TapHold,
                    _ => MacroType::Tap,
                };
                keyboard_macros.initialize_flash(index, &t);
            } else {
                output[0] = DataCommand::Error as u8;
            }
        }

        DataCommand::ValidateMacro => {
            let index = (output[1] >> 2) as usize;
            let t = (output[1] & 0b11) as usize;
            if index < KEY_COUNT {
                let valid_checksum = u32::from_le_bytes(output[2..6].try_into().unwrap());
                let t = match t {
                    1 => MacroType::Hold,
                    2 => MacroType::DoubleTap,
                    3 => MacroType::TapHold,
                    _ => MacroType::Tap,
                };
                let checksum = keyboard_macros.get_checksum(index, &t);
                output[6..10].copy_from_slice(&checksum.to_le_bytes());
                if checksum == valid_checksum {
                    keyboard_macros.set_checksum(index, &t);
                } else {
                    output[0] = DataCommand::Error as u8;
                }
            } else {
                output[0] = DataCommand::Error as u8;
            }
        }

        DataCommand::ReadConfig => {
            let config_command = ConfigElements::from(output[1]);

            match config_command {
                ConfigElements::Version => {
                    output[2..4].copy_from_slice(&PROTOCOL_VERSION.to_le_bytes());
                }
                ConfigElements::TapSpeed => {
                    output[2..6].copy_from_slice(&config.tap_speed.to_le_bytes());
                }
                ConfigElements::HoldSpeed => {
                    output[2..6].copy_from_slice(&config.hold_speed.to_le_bytes());
                }
                ConfigElements::Error => {
                    output[0] = DataCommand::Error as u8;
                    output[1] = ConfigElements::Error as u8;
                }
            }
        }

        DataCommand::WriteConfig => {
            let config_command = ConfigElements::from(output[1]);

            match config_command {
                ConfigElements::Version => {
                    output[0] = DataCommand::Error as u8;
                    output[1] = ConfigElements::Error as u8;
                }
                ConfigElements::TapSpeed => {
                    let new_tap_speed = u32::from_le_bytes(output[2..6].try_into().unwrap());
                    if new_tap_speed > 0 {
                        config.tap_speed = new_tap_speed;
                    } else {
                        output[0] = DataCommand::Error as u8;
                        output[1] = ConfigElements::Error as u8;
                    }
                }
                ConfigElements::HoldSpeed => {
                    let new_hold_speed = u32::from_le_bytes(output[2..6].try_into().unwrap());
                    if new_hold_speed > 0 {
                        config.hold_speed = new_hold_speed;
                        config.write()
                    } else {
                        output[0] = DataCommand::Error as u8;
                        output[1] = ConfigElements::Error as u8;
                    }
                }
                ConfigElements::Error => {
                    output[0] = DataCommand::Error as u8;
                    output[1] = ConfigElements::Error as u8;
                }
            }
        }

        DataCommand::GetLed => {
            let led_command = LedCommand::from(output[1]);

            match led_command {
                LedCommand::None => {}
                LedCommand::BaseColor => {
                    let color = config.led_config.base_color;
                    output[2] = color.r;
                    output[3] = color.g;
                    output[4] = color.b;
                }
                LedCommand::Effect => {
                    output[2] = config.led_config.effect as u8;
                }
                LedCommand::Brightness => {
                    output[2] = config.led_config.brightness;
                }
                LedCommand::EffectPeriod => {
                    output[2..6].copy_from_slice(&config.led_config.effect_period.to_le_bytes());
                }
                LedCommand::EffectOffset => {
                    output[2..6].copy_from_slice(&config.led_config.effect_offset.to_le_bytes());
                }
                LedCommand::Error => {
                    output[0] = DataCommand::Error as u8;
                    output[1] = LedCommand::Error as u8;
                }
            }
        }

        DataCommand::SetLed => {
            let led_command = LedCommand::from(output[1]);

            match led_command {
                LedCommand::None => {}
                LedCommand::BaseColor => {
                    config.led_config.base_color = RGB8 {
                        r: output[2],
                        g: output[3],
                        b: output[4],
                    };

                    config.write();
                }
                LedCommand::Effect => {
                    config.led_config.effect = LedEffect::from(output[2]);

                    config.led_config.reset_timer(timer);
                    config.write();
                }
                LedCommand::Brightness => {
                    config.led_config.brightness = output[2];

                    config.write();
                }
                LedCommand::EffectPeriod => {
                    config.led_config.effect_period =
                        f32::from_le_bytes(output[2..6].try_into().unwrap());

                    config.write();
                }
                LedCommand::EffectOffset => {
                    config.led_config.effect_offset =
                        f32::from_le_bytes(output[2..6].try_into().unwrap());

                    config.write();
                }
                LedCommand::Error => {
                    output[0] = DataCommand::Error as u8;
                    output[1] = LedCommand::Error as u8;
                }
            }
        }

        DataCommand::ReadKeyConfig => {
            let key_config_command = KeyConfigElements::from(output[1]);

            match key_config_command {
                KeyConfigElements::KeyMode => {
                    let index = output[2] as usize;
                    if index < KEY_COUNT {
                        output[3] = config.key_configs[index].key_mode as u8;
                    } else {
                        output[0] = DataCommand::Error as u8;
                        output[1] = KeyConfigElements::Error as u8;
                    }
                }

                KeyConfigElements::KeyboardData => {
                    let index = output[2] as usize;
                    if index < KEY_COUNT {
                        output[3] = config.key_configs[index].keyboard_data;
                    } else {
                        output[0] = DataCommand::Error as u8;
                        output[1] = KeyConfigElements::Error as u8;
                    }
                }

                KeyConfigElements::ConsumerData => {
                    let index = output[2] as usize;
                    if index < KEY_COUNT {
                        let consumer_data = config.key_configs[index].consumer_data;
                        output[3..5].copy_from_slice(&consumer_data.to_le_bytes());
                    } else {
                        output[0] = DataCommand::Error as u8;
                        output[1] = KeyConfigElements::Error as u8;
                    }
                }

                KeyConfigElements::KeyColor => {
                    let index = output[2] as usize;
                    if index < KEY_COUNT {
                        let color = config.key_configs[index].key_color;
                        output[3] = color.r;
                        output[4] = color.g;
                        output[5] = color.b;
                    } else {
                        output[0] = DataCommand::Error as u8;
                        output[1] = KeyConfigElements::Error as u8;
                    }
                }

                KeyConfigElements::Error => {
                    output[0] = DataCommand::Error as u8;
                    output[1] = KeyConfigElements::Error as u8;
                }
            }
        }

        DataCommand::WriteKeyConfig => {
            let key_config_command = KeyConfigElements::from(output[1]);

            match key_config_command {
                KeyConfigElements::KeyMode => {
                    let index = output[2] as usize;
                    if index < KEY_COUNT {
                        config.key_configs[index].key_mode = KeyMode::from(output[3]);
                        config.write();
                    } else {
                        output[0] = DataCommand::Error as u8;
                        output[1] = KeyConfigElements::Error as u8;
                    }
                }

                KeyConfigElements::KeyboardData => {
                    let index = output[2] as usize;
                    if index < KEY_COUNT {
                        config.key_configs[index].keyboard_data = output[3];
                        config.write();
                    } else {
                        output[0] = DataCommand::Error as u8;
                        output[1] = KeyConfigElements::Error as u8;
                    }
                }

                KeyConfigElements::ConsumerData => {
                    let index = output[2] as usize;
                    if index < KEY_COUNT {
                        config.key_configs[index].consumer_data =
                            u16::from_le_bytes(output[3..5].try_into().unwrap());
                        config.write();
                    } else {
                        output[0] = DataCommand::Error as u8;
                        output[1] = KeyConfigElements::Error as u8;
                    }
                }

                KeyConfigElements::KeyColor => {
                    let index = output[2] as usize;
                    if index < KEY_COUNT {
                        config.key_configs[index].key_color = RGB8 {
                            r: output[3],
                            g: output[4],
                            b: output[5],
                        };
                        config.write();
                    } else {
                        output[0] = DataCommand::Error as u8;
                        output[1] = KeyConfigElements::Error as u8;
                    }
                }

                KeyConfigElements::Error => {
                    output[0] = DataCommand::Error as u8;
                    output[1] = KeyConfigElements::Error as u8;
                }
            }
        }

        DataCommand::GetBuildInfo => {
            let build_info_command = BuildInfoElements::from(output[1]);

            match build_info_command {
                BuildInfoElements::FirmwareVersion => {
                    const VERSION: &str = env!("CARGO_PKG_VERSION");
                    output[2] = VERSION.len() as u8;
                    output[3..VERSION.len() + 3].copy_from_slice(VERSION.as_bytes());
                }
                BuildInfoElements::BuildDate => {
                    const DATE: &str = env!("VERGEN_BUILD_DATE");
                    output[2] = DATE.len() as u8;
                    output[3..DATE.len() + 3].copy_from_slice(DATE.as_bytes());
                }
                BuildInfoElements::BuildTimestamp => {
                    const TIME: &str = env!("VERGEN_BUILD_TIMESTAMP");
                    output[2] = TIME.len() as u8;
                    output[3..TIME.len() + 3].copy_from_slice(TIME.as_bytes());
                }
                BuildInfoElements::BuildProfile => {
                    const BUILD_TYPE: &str = env!("BUILD_PROFILE");
                    output[2] = BUILD_TYPE.len() as u8;
                    output[3..BUILD_TYPE.len() + 3].copy_from_slice(BUILD_TYPE.as_bytes());
                }
                BuildInfoElements::GitHash => {
                    const HASH: &str = env!("VERGEN_GIT_SHA");
                    output[2] = HASH.len() as u8;
                    output[3..HASH.len() + 3].copy_from_slice(HASH.as_bytes());
                }
                BuildInfoElements::GitBranch => {
                    const BRANCH: &str = env!("VERGEN_GIT_BRANCH");
                    output[2] = BRANCH.len() as u8;
                    output[3..BRANCH.len() + 3].copy_from_slice(BRANCH.as_bytes());
                }
                BuildInfoElements::GitSemver => {
                    const SEMVER: &str = env!("VERGEN_GIT_DESCRIBE");
                    output[2] = SEMVER.len() as u8;
                    output[3..SEMVER.len() + 3].copy_from_slice(SEMVER.as_bytes());
                }
                BuildInfoElements::Error => {
                    output[0] = DataCommand::Error as u8;
                    output[1] = BuildInfoElements::Error as u8;
                }
            }
        }

        DataCommand::EnterBootloader => {
            reset_to_usb_boot(0, 0);
        }

        DataCommand::Error => {
            output[0] = DataCommand::Error as u8;
        }
    }

    GenericInOutMsg { packet: output }
}
