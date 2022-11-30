use core::mem;

use smart_leds::RGB8;

const GAMMA_CORRECTION_TABLE: [u8; 256] = [
    // Brightness ramp for LEDs
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6,
    6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15,
    16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 23, 23, 24, 24, 25, 26, 26, 27, 28, 28, 29,
    30, 30, 31, 32, 32, 33, 34, 35, 36, 36, 37, 38, 39, 40, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    49, 50, 51, 52, 53, 54, 55, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 68, 69, 70, 71, 72, 74, 75,
    76, 77, 79, 80, 81, 83, 84, 85, 87, 88, 89, 91, 92, 94, 95, 97, 98, 99, 101, 103, 104, 106,
    107, 109, 110, 112, 113, 115, 117, 118, 120, 122, 123, 125, 127, 129, 130, 132, 134, 136, 138,
    139, 141, 143, 145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175,
    177, 179, 182, 184, 186, 188, 190, 193, 195, 197, 200, 202, 204, 207, 209, 211, 214, 216, 219,
    221, 223, 226, 228, 231, 234, 236, 239, 241, 244, 247, 249, 252, 255,
];

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum LedEffect {
    None = 0x00,
    Static = 0x01,
    BreathingUp = 0x02,
    BreathingDown = 0x03,
    ColorCycle = 0x04,
}

const LED_EFFECTS: u8 = 9;

impl LedEffect {
    pub fn from_u8(n: u8) -> Option<LedEffect> {
        if n < LED_EFFECTS {
            Some(unsafe { mem::transmute(n) })
        } else {
            None
        }
    }

    pub fn apply(self, time: u32, backlight: &mut RGB8, base_color: &mut RGB8, effect: &mut LedEffect, brightness: &mut u8, effect_speed: &mut u8, effect_offset: &mut u8) {
        match self {
            LedEffect::None => effect_none(backlight, base_color, effect, brightness, effect_speed, effect_offset),
            LedEffect::Static => effect_static(backlight, base_color, effect, brightness, effect_speed, effect_offset),
            LedEffect::BreathingUp => effect_breathing_up(backlight, base_color, effect, brightness, effect_speed, effect_offset),
            LedEffect::BreathingDown => effect_breathing_down(backlight, base_color, effect, brightness, effect_speed, effect_offset),
            LedEffect::ColorCycle => effect_color_cycle(backlight, base_color, effect, brightness, effect_speed, effect_offset),
        };
    }
}

pub const STRIP_LEN: usize = 4;

pub fn effect_none(backlight: &mut RGB8, _base_color: &mut RGB8, _effect: &mut LedEffect, _brightness: &mut u8, _effect_speed: &mut u8, _effect_offset: &mut u8) {
    *backlight = (0, 0, 0).into();
}

pub fn effect_static(backlight: &mut RGB8, base_color: &mut RGB8, _effect: &mut LedEffect, _brightness: &mut u8, _effect_speed: &mut u8, _effect_offset: &mut u8) {
    *backlight = *base_color;
}

pub fn effect_breathing_up(backlight: &mut RGB8, base_color: &mut RGB8, effect: &mut LedEffect, _brightness: &mut u8, _effect_speed: &mut u8, effect_offset: &mut u8) {
    let r = base_color.r as f32 / 255.0 * (*effect_offset as f32 / 255.0) * 255.0;
    let g = base_color.g as f32 / 255.0 * (*effect_offset as f32 / 255.0) * 255.0;
    let b = base_color.b as f32 / 255.0 * (*effect_offset as f32 / 255.0) * 255.0;

    if *effect_offset == 255 {
        *effect = LedEffect::BreathingDown;
        *effect_offset = 0;
    }

    *backlight = (r as u8, g as u8, b as u8).into();
}

pub fn effect_breathing_down(backlight: &mut RGB8, base_color: &mut RGB8, effect: &mut LedEffect, _brightness: &mut u8, _effect_speed: &mut u8, effect_offset: &mut u8) {
    let r = base_color.r as f32 / 255.0 * (1.0 - (*effect_offset as f32 / 255.0)) * 255.0;
    let g = base_color.g as f32 / 255.0 * (1.0 - (*effect_offset as f32 / 255.0)) * 255.0;
    let b = base_color.b as f32 / 255.0 * (1.0 - (*effect_offset as f32 / 255.0)) * 255.0;

    if *effect_offset == 255 {
        *effect = LedEffect::BreathingUp;
        *effect_offset = 0;
    }

    *backlight = (r as u8, g as u8, b as u8).into();
}

pub fn effect_color_cycle(backlight: &mut RGB8, _base_color: &mut RGB8, _effect: &mut LedEffect, _brightness: &mut u8, _effect_speed: &mut u8, effect_offset: &mut u8) {
    *backlight = hsv2rgb_u8((*effect_offset as f32 * 360.0) / 255.0, 1.0, 1.0).into();
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

pub fn rgb_gamma_correct(r: u8, g: u8, b: u8) -> (u8, u8, u8) {
    (
        GAMMA_CORRECTION_TABLE[r as usize],
        GAMMA_CORRECTION_TABLE[g as usize],
        GAMMA_CORRECTION_TABLE[b as usize],
    )
}

pub fn rgb_gamma_correct_rgb8(rgb: RGB8) -> RGB8 {
    rgb_gamma_correct(rgb.r, rgb.g, rgb.b).into()
}