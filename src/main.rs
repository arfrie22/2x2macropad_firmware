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
use data_protocol::ConfigElements;
use data_protocol::LedCommand;

use hal::timer::CountDown;
use led_effect::LedConfig;
use led_effect::LedEffect;
use led_effect::STRIP_LEN;
use macro_protocol::MacroCommand;
use packed_struct::prelude::*;
use smart_leds::gamma;
pub const CKSUM: Crc<u32> = Crc::<u32>::new(&CRC_32_CKSUM);

use core::cell::UnsafeCell;
use core::convert::Infallible;
use core::default::Default;
use core::mem;

use arrayvec::ArrayVec;

use data_protocol::{DataCommand, PROTOCOL_VERSION};
use hal::rom_data::reset_to_usb_boot;
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

pub mod data_protocol;

pub mod led_effect;

pub mod macro_protocol;

#[repr(C, align(4096))]
struct FlashBlock {
    data: UnsafeCell<[u8; 4096]>,
}

use crate::rp2040_flash::flash;
pub mod rp2040_flash;

pub mod raw_hid;
use raw_hid::{GenericInOutInterface, GenericInOutMsg};

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

struct KeyMacro {
    tap: FlashBlock,
    hold: FlashBlock,
    ttap: FlashBlock,
    thold: FlashBlock,
}

#[derive(Debug, EnumIter, Clone)]
enum MacroType {
    Tap,
    Hold,
    DoubleTap,
    TapHold,
}

impl KeyMacro {
    fn validate(&self, t: &MacroType) -> bool {
        match t {
            MacroType::Tap => self.tap.validate(),
            MacroType::Hold => self.hold.validate(),
            MacroType::DoubleTap => self.ttap.validate(),
            MacroType::TapHold => self.thold.validate(),
        }
    }

    fn get_checksum(&self, t: &MacroType) -> u32 {
        match t {
            MacroType::Tap => self.tap.get_checksum(),
            MacroType::Hold => self.hold.get_checksum(),
            MacroType::DoubleTap => self.ttap.get_checksum(),
            MacroType::TapHold => self.thold.get_checksum(),
        }
    }

    fn set_checksum(&self, t: &MacroType) {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        match t {
            MacroType::Tap => self.tap.set_checksum(),
            MacroType::Hold => self.hold.set_checksum(),
            MacroType::DoubleTap => self.ttap.set_checksum(),
            MacroType::TapHold => self.thold.set_checksum(),
        }
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }

    fn write_flash(&self, t: &MacroType, data: &[u8; 4096]) {
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

    fn clear_flash(&self, t: &MacroType) {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        match t {
            MacroType::Tap => self.tap.clear_flash(),
            MacroType::Hold => self.hold.clear_flash(),
            MacroType::DoubleTap => self.ttap.clear_flash(),
            MacroType::TapHold => self.thold.clear_flash(),
        }
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }

    fn initialize_flash(&self, t: &MacroType) {
        match t {
            MacroType::Tap => self.tap.initialize_flash(),
            MacroType::Hold => self.hold.initialize_flash(),
            MacroType::DoubleTap => self.ttap.initialize_flash(),
            MacroType::TapHold => self.thold.initialize_flash(),
        }
    }

    fn read(&self, t: &MacroType) -> &[u8; 4096] {
        match t {
            MacroType::Tap => self.tap.read(),
            MacroType::Hold => self.hold.read(),
            MacroType::DoubleTap => self.ttap.read(),
            MacroType::TapHold => self.thold.read(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, PackedStruct)]
#[packed_struct(size_bytes = "4092")]
struct Config {
    #[packed_field(endian = "lsb")]
    version: u16,
    #[packed_field(endian = "lsb")]
    tap_speed: u32,
    #[packed_field(endian = "lsb")]
    hold_speed: u32,
    #[packed_field(endian = "lsb")]
    default_delay: u32,
    #[packed_field(element_size_bytes = "13")]
    led_config: LedConfig,
}

impl Config {
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
            hold_speed: MicrosDurationU32::millis(500).to_micros(),
            default_delay: MicrosDurationU32::millis(100).to_micros(),
            led_config: LedConfig::default(),
        }
    }
}

fn get_config() -> Config {
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

static MACROS: [&KeyMacro; 4] = [&MACRO_1, &MACRO_2, &MACRO_3, &MACRO_4];

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

const MACRO_LENGTH: u16 = 4096 - 2;

const KEYBOARD_POLL: MicrosDurationU32 = MicrosDurationU32::millis(10);
const CONSUMER_POLL: MicrosDurationU32 = MicrosDurationU32::millis(50);
const RAW_HID_POLL: MicrosDurationU32 = MicrosDurationU32::millis(10);
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
            raw_hid::GenericInOutInterface::default_config(),
        )
        .add_interface(
            usbd_human_interface_device::device::keyboard::NKROBootKeyboardInterface::default_config(),
        )
        .add_interface(
            usbd_human_interface_device::device::consumer::ConsumerControlInterface::default_config(),
        )
        //Build
        .build(usb_alloc);

    //https://pid.codes
    let mut usb_dev = UsbDeviceBuilder::new(usb_alloc, UsbVidPid(0x1209, 0x0001))
        .manufacturer("usbd-human-interface-device")
        .product("Keyboard & Consumer & RawHid")
        .serial_number("TEST")
        .build();

    let row_pins: &[&dyn InputPin<Error = core::convert::Infallible>] = &[
        &pins.gpio19.into_pull_down_input(),
        &pins.gpio18.into_pull_down_input(),
    ];

    let col_pins: &mut [&mut dyn OutputPin<Error = core::convert::Infallible>] = &mut [
        &mut pins.gpio21.into_push_pull_output(),
        &mut pins.gpio20.into_push_pull_output(),
    ];

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

    let mut key_timers = [
        timer.count_down(),
        timer.count_down(),
        timer.count_down(),
        timer.count_down(),
    ];

    let mut macro_delay = timer.count_down();
    macro_delay.start(MicrosDurationU32::millis(0));

    // Setup a delay for the LED blink signals:
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Import the `sin` function for a smooth hue animation from the
    // Pico rp2040 ROM:
    let sin = hal::rom_data::float_funcs::fsin::ptr();

    // Split the PIO state machine 0 into individual objects, so that
    // Ws2812 can use it:
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Instanciate a Ws2812 LED strip:
    let mut ws = Ws2812::new(
        // Use pin 6 on the Raspberry Pi Pico (which is GPIO4 of the rp2040 chip)
        // for the LED data output:
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

    if !CONFIG.validate() {
        CONFIG.initialize_flash();
    }

    for key in MACROS.iter() {
        for t in MacroType::iter() {
            if !key.validate(&t) {
                // key.initialize_flash(&t);
                let mut data = [0u8; 4096];
                data[0] = MacroCommand::CommandConsumer as u8;
                data[1..=2].copy_from_slice(&(Consumer::VolumeIncrement as u16).to_le_bytes());
                data[3] = MacroCommand::CommandDelay as u8;
                data[3] = 0;
                data[5] = 0xFF;
                data[5] = 0;
                data[6] = 0;
                data[7] = MacroCommand::CommandSetLed as u8;
                data[8] = 0;
                data[9] = 0xFF;
                data[10] = 0xFF;

                key.write_flash(&t, &data);
                key.set_checksum(&t)
            }
        }
    }

    let mut backlight: [RGB8; STRIP_LEN] = [(0, 0, 0).into(); STRIP_LEN];
    let mut time = 0u32;

    for (_, led) in backlight.iter_mut().enumerate() {
        *led = (0, 0, 0).into();
    }

    let mut config = get_config();
    ws.write(brightness(backlight.iter().copied(), config.led_config.brightness))
        .unwrap();

    // delay.delay_ms(100);

    let mut raw_hid_queue = ArrayVec::<GenericInOutMsg, 32>::new();
    let mut previous_key_states = [KeyState::Idle; 4];

    let mut current_macro = None;
    let mut current_macro_index = 0;
    let mut current_offset = 0;
    let mut macro_backlight = None;

    let mut keys = [Keyboard::NoEventIndicated; 256];

    let mut key_change = false;
    let mut consumer_change = false;

    loop {
        let matrix = scan_matrix(&mut delay, col_pins, row_pins);
        let mut key_states =
            get_key_states(&matrix, &mut key_timers, &previous_key_states, &config);

        let mut consumers = [Consumer::Unassigned; 4];

        if !key_change && !consumer_change && macro_delay.wait().is_ok() {
            if let Some(c_macro) = current_macro {

                let (new_offset, new_consumers, delay, is_done) = read_macro(
                    current_offset,
                    c_macro,
                    &config,
                    &mut macro_backlight,
                    &mut keys,
                );

                if is_done {
                    current_macro = None;
                    macro_backlight = None;

                    for (_, key) in key_states.iter_mut().enumerate() {
                        if *key == KeyState::Active {
                            *key = KeyState::Done;
                        }
                    }
                }

                current_offset = new_offset;
                macro_delay.start(delay.unwrap_or_else( || MicrosDurationU32::micros(config.default_delay)));

                key_change = true;

                consumers = [Consumer::Unassigned; 4];

                for (i, c) in new_consumers.iter().enumerate() {
                    consumers[i] = *c;
                }

                if consumers != last_consumer_report.codes {
                    consumer_change = true;
                }
            } else {
                for (i, key) in key_states.iter().enumerate() {
                    match key {
                        KeyState::Tap => {
                            current_macro = Some(&MACROS[i].tap);
                            current_macro_index = i;
                            current_offset = 0;
                            key_states[i] = KeyState::Active;
                            macro_backlight = None;
                            break;
                        }

                        KeyState::Hold => {
                            current_macro = Some(&MACROS[i].hold);
                            current_macro_index = i;
                            current_offset = 0;
                            key_states[i] = KeyState::Active;
                            macro_backlight = None;
                            break;
                        }

                        KeyState::TTap => {
                            current_macro = Some(&MACROS[i].ttap);
                            current_macro_index = i;
                            current_offset = 0;
                            key_states[i] = KeyState::Active;
                            macro_backlight = None;
                            break;
                        }

                        KeyState::THold => {
                            current_macro = Some(&MACROS[i].thold);
                            current_macro_index = i;
                            current_offset = 0;
                            key_states[i] = KeyState::Active;
                            macro_backlight = None;
                            break;
                        }

                        _ => {}
                    }
                }
            }
        }

        if key_change && keyboard_input_timer.wait().is_ok() {
            key_change = false;
            let keyboard = composite.interface::<NKROBootKeyboardInterface<'_, _>, _>();
            match keyboard.write_report(&keys) {
                Err(UsbHidError::WouldBlock) => {}
                Err(UsbHidError::Duplicate) => {}
                Ok(_) => {}
                Err(e) => {
                    core::panic!("Failed to write keyboard report: {:?}", e)
                }
            };
        }

        if consumer_change && consumer_input_timer.wait().is_ok() {
            consumer_change = false;

            let consumer_report = MultipleConsumerReport { codes: consumers };

            if last_consumer_report != consumer_report {
                let consumer = composite.interface::<ConsumerControlInterface<'_, _>, _>();
                match consumer.write_report(&consumer_report) {
                    Err(UsbError::WouldBlock) => {}
                    Ok(_) => {
                        last_consumer_report = consumer_report;
                    }
                    Err(e) => {
                        core::panic!("Failed to write consumer report: {:?}", e)
                    }
                };
            }
        }

        if (!raw_hid_queue.is_empty() && raw_hid_timer.wait().is_ok()) {
            let raw_hid = composite.interface::<GenericInOutInterface<'_, _>, _>();
            let data = raw_hid_queue.pop().unwrap();
            match raw_hid.write_report(&data) {
                Err(UsbHidError::WouldBlock) => {
                    raw_hid_queue.insert(0, data);
                }
                Err(UsbHidError::Duplicate) => {}
                Ok(_) => {}
                Err(e) => {
                    core::panic!("Failed to write raw hid report: {:?}", e)
                }
            };
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
                    let data = parse_command(
                        &data,
                        &mut config,
                    );
                    raw_hid_queue.push(data);
                    raw_hid_queue.push(data);
                }
            }
        }

        if led_timer.wait().is_ok() {
            config.led_config.update(&mut backlight);

            if let Some(led) = macro_backlight {
                backlight[current_macro_index] = led;
            }

            time = time.wrapping_add(1);

            ws.write(brightness(
                gamma(backlight.iter().copied()),
                config.led_config.brightness,
            ))
            .unwrap();
        };

        previous_key_states = key_states;
    }
}

fn scan_matrix(
    delay: &mut Delay,
    col_pins: &mut [&mut dyn OutputPin<Error = Infallible>],
    row_pins: &[&dyn InputPin<Error = Infallible>],
) -> [bool; KEY_COUNT] {
    let mut keys = [false; KEY_COUNT];

    for i in 0..KEY_COLS {
        col_pins[i].set_high().ok();
        delay.delay_ms(1);

        for j in 0..KEY_ROWS {
            keys[(KEY_ROWS * i) + j] = row_pins[j].is_high().unwrap_or(false);
        }

        col_pins[i].set_low().ok();
    }

    keys
}

fn get_key_states(
    keys: &[bool; KEY_COUNT],
    key_timers: &mut [CountDown; KEY_COUNT],
    previous_key_state: &[KeyState; KEY_COUNT],
    config: &Config,
) -> [KeyState; KEY_COUNT] {
    let mut key_states = *previous_key_state;

    for i in 0..KEY_COUNT {
        match previous_key_state[i] {
            KeyState::Idle => {
                if keys[i] {
                    key_states[i] = KeyState::Intermediate;
                    key_timers[i].start(MicrosDurationU32::micros(config.hold_speed));
                }
            }
            KeyState::Intermediate => {
                if keys[i] {
                    if key_timers[i].wait().is_ok() {
                        key_states[i] = KeyState::Hold;
                    }
                } else {
                    key_states[i] = KeyState::TapIntermediate;
                    key_timers[i].start(MicrosDurationU32::micros(config.tap_speed));
                }
            }
            KeyState::TapIntermediate => {
                if keys[i] {
                    key_states[i] = KeyState::TTapIntermediate;
                    key_timers[i].start(MicrosDurationU32::micros(config.hold_speed));
                } else if key_timers[i].wait().is_ok() {
                    key_states[i] = KeyState::Tap;
                }
            }
            KeyState::TTapIntermediate => {
                if keys[i] {
                    if key_timers[i].wait().is_ok() {
                        key_states[i] = KeyState::THold;
                    }
                } else {
                    key_states[i] = KeyState::TTap;
                }
            }

            KeyState::Tap
            | KeyState::Hold
            | KeyState::TTap
            | KeyState::THold
            | KeyState::Active => continue,

            KeyState::Done => {
                if !keys[i] {
                    key_states[i] = KeyState::Idle;
                }
            }
        }
    }

    key_states
}

fn read_macro(
    current_offset: usize,
    current_macro: &FlashBlock,
    config: &Config,
    backlight: &mut Option<RGB8>,
    keys: &mut [Keyboard; 256],
) -> (
    usize,
    ArrayVec<Consumer, 4>,
    Option<MicrosDurationU32>,
    bool,
) {
    let mut consumers = ArrayVec::<Consumer, 4>::new();
    let mut offset = current_offset;
    let mut delay = None;
    let mut is_done = false;

    let macro_data = current_macro.read();

    let mut current_macro =
        MacroCommand::from_u8(macro_data[offset]).unwrap_or(MacroCommand::CommandTerminator);
    if current_macro == MacroCommand::CommandTerminator {
        is_done = true;
        offset = 0;
        *keys = [Keyboard::NoEventIndicated; 256];
        *backlight = None;
    }

    while current_macro != MacroCommand::CommandTerminator {
        offset += 1;
        match current_macro {
            MacroCommand::CommandTerminator => {}
            MacroCommand::CommandDelay => {
                let delay_bytes = macro_data[offset..offset + 4].try_into().unwrap();
                delay = Some(MicrosDurationU32::micros(u32::from_le_bytes(delay_bytes)));
                offset += 4;
            }
            MacroCommand::CommandPressKey => {
                let key = macro_data[offset];
                if (0x00..=0xA4).contains(&key) || (0xE0..=0xE7).contains(&key) {
                    keys[key as usize] = unsafe { mem::transmute(key) };
                }

                offset += 1;
            }
            MacroCommand::CommandReleaseKey => {
                let key = macro_data[offset];
                keys[key as usize] = Keyboard::NoEventIndicated;

                offset += 1;
            }
            MacroCommand::CommandConsumer => {
                let consumer_value =
                    u16::from_le_bytes(macro_data[offset..offset + 2].try_into().unwrap());

                if (0x00..=0x06).contains(&consumer_value)
                    || (0x20..=0x22).contains(&consumer_value)
                    || (0x30..=0x36).contains(&consumer_value)
                    || (0x40..=0x48).contains(&consumer_value)
                    || (0x60..=0x66).contains(&consumer_value)
                    || (0x80..=0x9E).contains(&consumer_value)
                    || (0x90..=0x92).contains(&consumer_value)
                    || (0xA0..=0xA4).contains(&consumer_value)
                    || (0xB0..=0xCE).contains(&consumer_value)
                    || (0xE0..=0xEA).contains(&consumer_value)
                    || (0xF0..=0xF5).contains(&consumer_value)
                    || (0x100..=0x10D).contains(&consumer_value)
                    || (0x150..=0x155).contains(&consumer_value)
                    || (0x160..=0x16A).contains(&consumer_value)
                    || (0x170..=0x174).contains(&consumer_value)
                    || (0x180..=0x1BA).contains(&consumer_value)
                    || (0x1BC..=0x1C7).contains(&consumer_value)
                    || (0x200..=0x29C).contains(&consumer_value)
                {
                    consumers.push(unsafe { mem::transmute(consumer_value) });
                }

                offset += 2;
            }
            MacroCommand::CommandSetLed => {
                let color = (macro_data[offset], macro_data[offset + 1], macro_data[offset + 2]).into();
                *backlight = Some(color);

                offset += 3;
            }
            MacroCommand::CommandClearLed => {
                *backlight = None;
            }
        };
        current_macro =
            MacroCommand::from_u8(macro_data[offset]).unwrap_or(MacroCommand::CommandTerminator);
    }

    (offset, consumers, delay, is_done)
}

fn parse_command(
    data: &GenericInOutMsg,
    config: &mut Config,
) -> GenericInOutMsg {
    let mut output = data.packet;
    let command = DataCommand::from_u8(output[0]).unwrap_or(DataCommand::Error);

    match command {
        DataCommand::None => {}

        DataCommand::GetProtocolVersion => {
            output[1] = (PROTOCOL_VERSION >> 8) as u8;
            output[2] = PROTOCOL_VERSION as u8;
        }

        DataCommand::ReadMacro => {
            let index = (output[1] >> 2) as usize;
            let t = (output[1] & 0b11) as usize;
            if index < KEY_COUNT {
                let offset = ((output[1] as u16) << 8) | output[2] as u16;
                if offset < MACRO_LENGTH {
                    let length = output[3] as usize;
                    if length > 0 && length < 60 && (offset + (length as u16)) < MACRO_LENGTH {
                        let t = match t {
                            1 => MacroType::Hold,
                            2 => MacroType::DoubleTap,
                            3 => MacroType::TapHold,
                            _ => MacroType::Tap,
                        };
                        let macro_data: [u8; 4096] = *MACROS[index].read(&t);
                        output[4..].copy_from_slice(
                            &macro_data[(offset + 2) as usize..(offset as usize + length + 2)],
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
            if index < KEY_COUNT {
                let offset = ((output[1] as u16) << 8) | output[2] as u16;
                if offset < MACRO_LENGTH {
                    let length = output[3] as usize;
                    if length > 0 && length < 60 && (offset + (length as u16)) < MACRO_LENGTH {
                        let t = match t {
                            1 => MacroType::Hold,
                            2 => MacroType::DoubleTap,
                            3 => MacroType::TapHold,
                            _ => MacroType::Tap,
                        };
                        let mut macro_data: [u8; 4096] = *MACROS[index].read(&t);
                        macro_data[1] = 0;
                        macro_data[(offset + 2) as usize..(offset as usize + length + 2)]
                            .copy_from_slice(&output[4..]);
                        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
                        MACROS[index].write_flash(&t, &macro_data);
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

        DataCommand::ValidateMacro => {
            let index = (output[1] >> 2) as usize;
            let t = (output[1] & 0b11) as usize;
            if index < KEY_COUNT {
                let valid_checksum = ((output[2] as u32) << 24)
                    | ((output[3] as u32) << 16)
                    | ((output[4] as u32) << 8)
                    | output[5] as u32;
                let t = match t {
                    1 => MacroType::Hold,
                    2 => MacroType::DoubleTap,
                    3 => MacroType::TapHold,
                    _ => MacroType::Tap,
                };
                let checksum = MACROS[index].get_checksum(&t);
                output[6] = (checksum >> 24) as u8;
                output[7] = (checksum >> 16) as u8;
                output[8] = (checksum >> 8) as u8;
                output[9] = checksum as u8;
                if checksum != valid_checksum {
                    output[0] = DataCommand::Error as u8;
                }
            } else {
                output[0] = DataCommand::Error as u8;
            }
        }

        DataCommand::ReadConfig => {
            let config_command = ConfigElements::from_u8(output[1]).unwrap_or(ConfigElements::Error);

            match config_command {
                ConfigElements::Version => {
                    output[2] = (PROTOCOL_VERSION >> 8) as u8;
                    output[3] = PROTOCOL_VERSION as u8;
                },
                ConfigElements::TapSpeed => {
                    output[2] = (config.tap_speed >> 24) as u8;
                    output[3] = (config.tap_speed >> 16) as u8;
                    output[4] = (config.tap_speed >> 8) as u8;
                    output[5] = config.tap_speed as u8;
                },
                ConfigElements::HoldSpeed => {
                    output[2] = (config.hold_speed >> 24) as u8;
                    output[3] = (config.hold_speed >> 16) as u8;
                    output[4] = (config.hold_speed >> 8) as u8;
                    output[5] = config.hold_speed as u8;
                },
                ConfigElements::DefaultDelay => {
                    output[2] = (config.default_delay >> 24) as u8;
                    output[3] = (config.default_delay >> 16) as u8;
                    output[4] = (config.default_delay >> 8) as u8;
                    output[5] = config.default_delay as u8;
                },
                ConfigElements::Error => {
                    output[0] = DataCommand::Error as u8;
                    output[1] = ConfigElements::Error as u8;
                },
            }
        }

        DataCommand::WriteConfig => {
            let config_command = ConfigElements::from_u8(output[1]).unwrap_or(ConfigElements::Error);

            match config_command {
                ConfigElements::Version => {
                    output[0] = DataCommand::Error as u8;
                    output[1] = ConfigElements::Error as u8;
                },
                ConfigElements::TapSpeed => {
                    let new_tap_speed = ((output[2] as u32) << 24)
                        | ((output[3] as u32) << 16)
                        | ((output[4] as u32) << 8)
                        | output[5] as u32;
                    if new_tap_speed > 0 {
                        config.tap_speed = new_tap_speed;
                        
                    } else {
                        output[0] = DataCommand::Error as u8;
                        output[1] = ConfigElements::Error as u8;
                    }
                },
                ConfigElements::HoldSpeed => {
                    let new_hold_speed = ((output[2] as u32) << 24)
                        | ((output[3] as u32) << 16)
                        | ((output[4] as u32) << 8)
                        | output[5] as u32;
                    if new_hold_speed > 0 {
                        config.hold_speed = new_hold_speed;
                        config.write()
                    } else {
                        output[0] = DataCommand::Error as u8;
                        output[1] = ConfigElements::Error as u8;
                    }
                },
                ConfigElements::DefaultDelay => {
                    let new_default_delay = ((output[2] as u32) << 24)
                        | ((output[3] as u32) << 16)
                        | ((output[4] as u32) << 8)
                        | output[5] as u32;
                    if new_default_delay > 0 {
                        config.default_delay = new_default_delay;
                        config.write()
                    } else {
                        output[0] = DataCommand::Error as u8;
                        output[1] = ConfigElements::Error as u8;
                    }
                },
                ConfigElements::Error => {
                    output[0] = DataCommand::Error as u8;
                    output[1] = ConfigElements::Error as u8;
                },
            }
        }

        DataCommand::GetLed => {
            let led_command = LedCommand::from_u8(output[1]).unwrap_or(LedCommand::Error);

            match led_command {
                LedCommand::None => {},
                LedCommand::BaseColor => {
                    let color = config.led_config.base_color;
                    output[2] = color.r;
                    output[3] = color.g;
                    output[4] = color.b;
                },
                LedCommand::Effect => {
                    output[2] = config.led_config.effect as u8;
                },
                LedCommand::Brightness => {
                    output[2] = config.led_config.brightness;
                },
                LedCommand::EffectSpeed => {
                    output[2..6].copy_from_slice(&config.led_config.effect_speed.to_le_bytes());
                },
                LedCommand::EffectOffset => {
                    output[2..6].copy_from_slice(&config.led_config.effect_offset.to_le_bytes());
                },
                LedCommand::Error => {
                    output[0] = DataCommand::Error as u8;
                    output[1] = LedCommand::Error as u8;
                },
            }
        }

        DataCommand::SetLed => {
            let led_command = LedCommand::from_u8(output[1]).unwrap_or(LedCommand::Error);

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
                    config.led_config.effect =
                            LedEffect::from_u8(output[2]).unwrap_or(LedEffect::None);

                    config.write();
                }
                LedCommand::Brightness => {
                    config.led_config.brightness = output[2];

                    config.write();
                }
                LedCommand::EffectSpeed => {
                    config.led_config.effect_speed =
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

        DataCommand::EnterBootloader => {
            reset_to_usb_boot(0, 0);
        }

        DataCommand::Error => {
            output[0] = DataCommand::Error as u8;
        }
    }

    GenericInOutMsg { packet: output }
}
