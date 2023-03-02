use macropad_protocol::data_protocol::LedEffect;

use packed_struct::prelude::*;
use rp2040_hal::Timer;
use smart_leds::RGB8;

pub fn apply(backlight: &mut [RGB8; STRIP_LEN], led_config: &mut LedConfig) {
    match led_config.effect {
        LedEffect::None => effect_none(backlight, led_config),
        LedEffect::Static => effect_static(backlight, led_config),
        LedEffect::Breathing => effect_breathing(backlight, led_config),
        LedEffect::BreathingSpaced => effect_breathing_spaced(backlight, led_config),
        LedEffect::ColorCycle => effect_color_cycle(backlight, led_config),
        LedEffect::Rainbow => effect_rainbow(backlight, led_config),
    };
}

pub const STRIP_LEN: usize = 4;

pub fn effect_none(backlight: &mut [RGB8; STRIP_LEN], _led_config: &mut LedConfig) {
    *backlight = [RGB8::default(); STRIP_LEN];
}

pub fn effect_static(backlight: &mut [RGB8; STRIP_LEN], led_config: &mut LedConfig) {
    *backlight = [RGB8 {
        r: led_config.base_color.r,
        g: led_config.base_color.g,
        b: led_config.base_color.b,
    }; STRIP_LEN];
}

pub fn effect_breathing(backlight: &mut [RGB8; STRIP_LEN], led_config: &mut LedConfig) {
    let mut color = led_config.base_color;

    let mut time = led_config.timer * (100.0 / 1000.0);

    if time > 50.0 {
        time = 100.0 - time;
    }

    color.r = (color.r as f32 * time / 50.0) as u8;
    color.g = (color.g as f32 * time / 50.0) as u8;
    color.b = (color.b as f32 * time / 50.0) as u8;

    *backlight = [RGB8 {
        r: color.r,
        g: color.g,
        b: color.b,
    }; STRIP_LEN];
}

pub fn effect_breathing_spaced(backlight: &mut [RGB8; STRIP_LEN], led_config: &mut LedConfig) {
    let timer = led_config.timer * ((100.0 * STRIP_LEN as f32) / 1000.0);

    for (index, led) in backlight.iter_mut().enumerate() {
        let mut color = led_config.base_color;
        let mut time = timer;
        time -= index as f32 * 100.0;

        if !(0.0..=100.0).contains(&time) {
            *led = (0, 0, 0).into();
            continue;
        }

        if time > 50.0 {
            time = 100.0 - time;
        }

        color.r = (color.r as f32 * time / 50.0) as u8;
        color.g = (color.g as f32 * time / 50.0) as u8;
        color.b = (color.b as f32 * time / 50.0) as u8;

        *led = color;
    }
}

pub fn effect_color_cycle(backlight: &mut [RGB8; STRIP_LEN], led_config: &mut LedConfig) {
    let timer = led_config.timer * (360.0 / 1000.0);

    *backlight = [hsv2rgb_u8(timer, 1.0, 1.0).into(); STRIP_LEN];
}

pub fn effect_rainbow(backlight: &mut [RGB8; STRIP_LEN], led_config: &mut LedConfig) {
    let timer = led_config.timer * (360.0 / 1000.0);

    for (index, led) in backlight.iter_mut().enumerate() {
        *led = hsv2rgb_u8(
            (timer + (index as f32 * 360.0 / STRIP_LEN as f32)) % 360.0,
            1.0,
            1.0,
        )
        .into();
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

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct LedConfig {
    pub base_color: RGB8,
    pub effect: LedEffect,
    pub brightness: u8,
    pub effect_period: f32,
    pub effect_offset: f32,
    timer: f32,
    prev_time: u64,
}

impl PackedStruct for LedConfig {
    type ByteArray = [u8; 13];

    fn pack(&self) -> packed_struct::PackingResult<Self::ByteArray> {
        let mut bytes = [0u8; 13];

        bytes[0] = self.base_color.r;
        bytes[1] = self.base_color.g;
        bytes[2] = self.base_color.b;
        bytes[3] = self.effect as u8;
        bytes[4] = self.brightness;
        bytes[5..9].copy_from_slice(&self.effect_period.to_le_bytes());
        bytes[9..13].copy_from_slice(&self.effect_offset.to_le_bytes());

        Ok(bytes)
    }

    fn unpack(src: &Self::ByteArray) -> packed_struct::PackingResult<Self> {
        Ok(LedConfig {
            base_color: RGB8 {
                r: src[0],
                g: src[1],
                b: src[2],
            },
            effect: LedEffect::from(src[3]),
            brightness: src[4],
            effect_period: f32::from_le_bytes([src[5], src[6], src[7], src[8]]),
            effect_offset: f32::from_le_bytes([src[9], src[10], src[11], src[12]]),
            timer: 0.0,
            prev_time: 0,
        })
    }
}

impl Default for LedConfig {
    fn default() -> Self {
        Self {
            base_color: (0, 0, 0).into(),
            effect: LedEffect::None,
            brightness: 0xA0,
            effect_period: 1.0,
            effect_offset: 0.0,
            timer: 0.0,
            prev_time: 0,
        }
    }
}

impl LedConfig {
    pub fn update(&mut self, backlight: &mut [RGB8; STRIP_LEN], timer: &Timer) {
        if self.effect_period != 0.0 {
            self.timer += (1.0 / self.effect_period)
                * ((timer.get_counter().duration_since_epoch().to_millis() - self.prev_time)
                    as f32);

            self.prev_time = timer.get_counter().duration_since_epoch().to_millis();

            self.timer %= 1000.0;

            if self.timer < 0.0 {
                self.timer += 1000.0;
            }
        }

        apply(backlight, self);
    }

    pub fn reset_timer(&mut self, timer: &Timer) {
        self.timer = 0.0;
        self.prev_time = timer.get_counter().duration_since_epoch().to_millis();
    }
}


pub struct LedManager {
    
}