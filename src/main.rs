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

use core::cell::{Cell, RefCell};
use core::convert::Infallible;
use core::default::Default;
use core::cell::UnsafeCell;

use arrayvec::ArrayVec;

use rp2040_hal as hal;

use hal::entry;
use cortex_m::delay::Delay;
use cortex_m::interrupt::Mutex;
use cortex_m::{prelude::*};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::*;
use frunk::HList;
use fugit::{ExtU32, MicrosDurationU32};
use hal::gpio::bank0::*;
use hal::gpio::{Output, Pin, PushPull};
use hal::pac;
// Pull in any important traits
use pac::interrupt;
use panic_probe as _;
use hal::prelude::*;
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_human_interface_device::device::consumer::{
    ConsumerControlInterface, MultipleConsumerReport,
};
use usbd_human_interface_device::device::keyboard::{
    NKROBootKeyboardInterface
};
use usbd_human_interface_device::page::Consumer;
use usbd_human_interface_device::page::Keyboard;
use usbd_human_interface_device::prelude::*;

// PIOExt for the split() method that is needed to bring
// PIO0 into useable form for Ws2812:
use hal::pio::PIOExt;

// Import useful traits to handle the ws2812 LEDs:
use smart_leds::{brightness, SmartLedsWrite, RGB8};

// Import the actual crate to handle the Ws2812 protocol:
use ws2812_pio::Ws2812;

const STRIP_LEN: usize = 4;

#[repr(C, align(4096))]
struct FlashBlock {
    data: UnsafeCell<[u8; 4096]>,
}

use crate::rp2040_flash::flash;
pub mod rp2040_flash;

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

#[link_section = ".rodata"]
static TEST: FlashBlock = FlashBlock {
    data: UnsafeCell::new([0x55u8; 4096]),
};

type UsbDevices = (
    UsbDevice<'static, hal::usb::UsbBus>,
    UsbHidClass<
        hal::usb::UsbBus,
        HList!(
            ConsumerControlInterface<'static, hal::usb::UsbBus>,
            NKROBootKeyboardInterface<'static, hal::usb::UsbBus>,
        ),
    >,
);
type LedPin = Pin<Gpio13, Output<PushPull>>;

static IRQ_SHARED: Mutex<RefCell<Option<UsbDevices>>> = Mutex::new(RefCell::new(None));
static USBCTRL: Mutex<Cell<Option<LedPin>>> = Mutex::new(Cell::new(None));

const KEYBOARD_POLL: MicrosDurationU32 = MicrosDurationU32::millis(10);
const CONSUMER_POLL: MicrosDurationU32 = MicrosDurationU32::millis(50);
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

    let composite = UsbHidClassBuilder::new()
        .add_interface(
            usbd_human_interface_device::device::keyboard::NKROBootKeyboardInterface::default_config(),
        )
        .add_interface(
            usbd_human_interface_device::device::consumer::ConsumerControlInterface::default_config(),
        )
        //Build
        .build(usb_alloc);

    //https://pid.codes
    let usb_dev = UsbDeviceBuilder::new(usb_alloc, UsbVidPid(0x1209, 0x0001))
        .manufacturer("usbd-human-interface-device")
        .product("Keyboard & Consumer")
        .serial_number("TEST")
        .build();

    cortex_m::interrupt::free(|cs| {
        IRQ_SHARED.borrow(cs).replace(Some((usb_dev, composite)));
        USBCTRL
            .borrow(cs)
            .replace(Some(pins.gpio13.into_push_pull_output()));
    });

    let row_pins: &[&dyn InputPin<Error = core::convert::Infallible>] = &[
        &pins.gpio19.into_pull_down_input(),
        &pins.gpio18.into_pull_down_input(),
    ];

    let mut col_pins: &mut [&mut dyn OutputPin<Error = core::convert::Infallible>] = &mut [
        &mut pins.gpio21.into_push_pull_output(),
        &mut pins.gpio20.into_push_pull_output(),
    ];

    let mut consumer_input_timer = timer.count_down();
    consumer_input_timer.start(CONSUMER_POLL);
    let mut last_consumer_report = MultipleConsumerReport::default();

    let mut keyboard_input_timer = timer.count_down();
    keyboard_input_timer.start(KEYBOARD_POLL);

    let mut tick_timer = timer.count_down();
    tick_timer.start(1.millis());

    let mut led_timer = timer.count_down();
    led_timer.start(LED_POLL);

    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

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

    let mut leds: [RGB8; STRIP_LEN] = [(0, 0, 0).into(); STRIP_LEN];
    let mut t = 0.0;

    let strip_brightness = 64u8; // Limit brightness to 64/256

    // Slow down timer by this factor (0.1 will result in 10 seconds):
    let animation_speed = 0.1;

    for (_, led) in leds.iter_mut().enumerate() {
        *led = (0, 255, 0).into();
    }

    // Here the magic happens and the `leds` buffer is written to the
    // ws2812 LEDs:
    ws.write(brightness(leds.iter().copied(), strip_brightness))
        .unwrap();

        delay.delay_ms(100);


    let psm = pac.PSM;

    // Reset core1 so it's guaranteed to be running
    // ROM code, waiting for the wakeup sequence
    psm.frce_off.modify(|_, w| w.proc1().set_bit());
    while !psm.frce_off.read().proc1().bit_is_set() {
        cortex_m::asm::nop();
    }
    psm.frce_off.modify(|_, w| w.proc1().clear_bit());

    let read_data: [u8; 4096] = *TEST.read();

    info!("Addr of flash block is {:x}", TEST.addr());
    info!("Contents start with {=[u8]}", read_data[0..4]);
    let mut data: [u8; 4096] = *TEST.read();
    data[0] = data[0].wrapping_add(1);
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    if read_data[0] != 0x56 {
        unsafe { TEST.write_flash(&data) };
    }

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    let read_data: [u8; 4096] = *TEST.read();
    info!("Contents start with {=[u8]}", read_data[0..4]);

    if read_data[0] != 0x56 {
        for (_, led) in leds.iter_mut().enumerate() {
            *led = (255, 0, 0).into();
        }

        // Here the magic happens and the `leds` buffer is written to the
        // ws2812 LEDs:
        ws.write(brightness(leds.iter().copied(), strip_brightness))
            .unwrap();

        delay.delay_ms(100);
        defmt::panic!("unexpected");
    }

    loop {
        // let keys = [true; KEY_COUNT];
        let keys = scan_matrix(&mut delay, &mut col_pins, &row_pins);

        if keyboard_input_timer.wait().is_ok() {
            cortex_m::interrupt::free(|cs| {
                let mut usb_ref = IRQ_SHARED.borrow(cs).borrow_mut();
                let (_, ref mut composite) = usb_ref.as_mut().unwrap();
                let keys = get_keyboard_keys(&keys);

                let keyboard = composite.interface::<NKROBootKeyboardInterface<'_, _>, _>();
                match keyboard.write_report(&keys) {
                    Err(UsbHidError::WouldBlock) => {}
                    Err(UsbHidError::Duplicate) => {}
                    Ok(_) => {}
                    Err(e) => {
                        core::panic!("Failed to write keyboard report: {:?}", e)
                    }
                };
            });
        }

        if consumer_input_timer.wait().is_ok() {
            cortex_m::interrupt::free(|cs| {
                let mut usb_ref = IRQ_SHARED.borrow(cs).borrow_mut();
                let (_, ref mut composite) = usb_ref.as_mut().unwrap();

                let codes = get_consumer_codes(&keys);
                let consumer_report = MultipleConsumerReport {
                    codes: [
                        codes[0],
                        codes[1],
                        Consumer::Unassigned,
                        Consumer::Unassigned,
                    ],
                };

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
            });
        }

        //Tick once per ms
        if tick_timer.wait().is_ok() {
            cortex_m::interrupt::free(|cs| {
                let mut usb_ref = IRQ_SHARED.borrow(cs).borrow_mut();
                let (_, ref mut composite) = usb_ref.as_mut().unwrap();

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
            });
        }

        if led_timer.wait().is_ok() {
            for (i, led) in leds.iter_mut().enumerate() {
                // An offset to give 3 consecutive LEDs a different color:
                let hue_offs = match i % 4 {
                    1 => 0.25,
                    2 => 0.5,
                    3 => 0.75,
                    _ => 0.0,
                };

                let sin_11 = sin((t + hue_offs) * 2.0 * core::f32::consts::PI);
                // Bring -1..1 sine range to 0..1 range:
                let sin_01 = (sin_11 + 1.0) * 0.5;

                let hue = 360.0 * sin_01;
                let sat = 1.0;
                let val = 1.0;

                let rgb = if keys[i] {
                    hsv2rgb_u8(hue, sat, val)
                } else {
                    (0, 0, 0)
                };

                *led = rgb.into();
            }

            // Here the magic happens and the `leds` buffer is written to the
            // ws2812 LEDs:
            ws.write(brightness(leds.iter().copied(), strip_brightness))
                .unwrap();

            // Increase the time counter variable and make sure it
            // stays inbetween 0.0 to 1.0 range:
            t += (LED_POLL.to_millis() as f32 / 1000.0) * animation_speed;
            while t > 1.0 {
                t -= 1.0;
            }
        }
    }
}

pub fn hsv2rgb(hue: f32, sat: f32, val: f32) -> (f32, f32, f32) {
    let c = val * sat;
    let v = (hue / 60.0) % 2.0 - 1.0;
    let v = if v < 0.0 { -v } else { v };
    let x = c * (1.0 - v);
    let m = val - c;
    let (r, g, b) = if hue < 60.0 {
        (c, x, 0.0)
    } else if hue < 120.0 {
        (x, c, 0.0)
    } else if hue < 180.0 {
        (0.0, c, x)
    } else if hue < 240.0 {
        (0.0, x, c)
    } else if hue < 300.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };
    (r + m, g + m, b + m)
}

pub fn hsv2rgb_u8(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    let r = hsv2rgb(h, s, v);

    (
        (r.0 * 255.0) as u8,
        (r.1 * 255.0) as u8,
        (r.2 * 255.0) as u8,
    )
}

//noinspection Annotator
#[allow(non_snake_case)]
#[interrupt]
fn USBCTRL_IRQ() {
    static mut LED_PIN: Option<LedPin> = None;
    if LED_PIN.is_none() {
        cortex_m::interrupt::free(|cs| {
            *LED_PIN = USBCTRL.borrow(cs).take();
        });
    }

    cortex_m::interrupt::free(|cs| {
        let mut usb_ref = IRQ_SHARED.borrow(cs).borrow_mut();
        if usb_ref.is_none() {
            return;
        }

        let (ref mut usb_device, ref mut composite) = usb_ref.as_mut().unwrap();
        if usb_device.poll(&mut [composite]) {
            let keyboard = composite.interface::<NKROBootKeyboardInterface<'_, _>, _>();
            match keyboard.read_report() {
                Err(UsbError::WouldBlock) => {}
                Err(e) => {
                    core::panic!("Failed to read keyboard report: {:?}", e)
                }
                Ok(leds) => {
                    LED_PIN
                        .as_mut()
                        .map(|p| p.set_state(PinState::from(leds.num_lock)).ok());
                }
            }
        }
    });

    cortex_m::asm::sev();
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

fn get_keyboard_keys(keys: &[bool]) -> ArrayVec::<Keyboard, 32> {
    let mut array = ArrayVec::<Keyboard, 32>::new();
    
    if keys[0] {
        array.push(Keyboard::A);
    } else {
        array.push(Keyboard::NoEventIndicated);
    }

    if keys[1] {
        array.push(Keyboard::C);
    } else {
        array.push(Keyboard::NoEventIndicated);
    }

    if keys[2] {
        array.push(Keyboard::B);
    } else {
        array.push(Keyboard::NoEventIndicated);
    }

    if keys[3] {
        array.push(Keyboard::D);
    } else {
        array.push(Keyboard::NoEventIndicated);
    }

    array
}

fn get_consumer_codes(keys: &[bool]) -> [Consumer; 2] {
    [
        if keys[0] {
            Consumer::VolumeDecrement
        } else {
            Consumer::Unassigned
        },
        if keys[3] {
            Consumer::VolumeIncrement
        } else {
            Consumer::Unassigned
        },
    ]
}
