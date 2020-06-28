pub const OAM_SIZE: usize = 0x100; // address for OAM
const FRAMEBUFFER_SIZE: usize = DISPLAY_WIDTH * DISPLAY_HEIGHT; // address for the full frame

const CLKS_SCREEN_REFRESH: u32 = 70224; // refresh every 70224 clks
pub const DISPLAY_WIDTH: usize = 160;
pub const DISPLAY_HEIGHT: usize = 144;

pub const VRAM_SIZE: usize = 1024*16; // 16KB Vram

const MODE_HBLANK: u32 = 0;
const MODE_VBLANK: u32 = 1;
const MODE_OAM: u32 = 2;
const MODE_VRAM: u32 = 3;

const HBLANK_CYCLES: u32 = 204;
const VBLANK_CYCLES: u32 = 456;
const OAM_CYCLES: u32 = 80;
const VRAM_CYCLES: u32 = 172;

#[derive(Debug,PartialEq,Eq)]
struct Color {
    r: u8,
    g: u8,
    b: u8,
    a: u8,
}

const WHITE: Color = Color {
    r: 224,
    g: 248,
    b: 208,
    a: 255,
};
const LIGHT_GRAY: Color = Color {
    r: 136,
    g: 192,
    b: 112,
    a: 255,
};
const DARK_GRAY: Color = Color {
    r: 39,
    g: 80,
    b: 70,
    a: 255,
};
const BLACK: Color = Color {
    r: 8,
    g: 24,
    b: 32,
    a: 255,
};

struct LCDC {
    lcd_display_enable: bool,
    window_tile_map_display_select: bool,
    window_display_enable: bool,
    bg_window_tile_data_select: bool,
    bg_tile_map_display_select: bool,
    sprite_size: bool,
    sprite_display_enable: bool,
    bg_window_display_priority: bool,
};

impl LCDC {
    pub fn new() -> Self {
        LCDC {
            lcd_display_enable = true,
            window_tile_map_display_select = false,
            window_display_enable = false,
            bg_window_tile_data_select = true,
            bg_tile_map_display_select = false,
            sprite_size = false,
            sprite_display_enable = false,
            bg_window_display_priority = true,
        } 
    }

    pub fn set_flag(&mut self, flags: u8) {
        self.lcd_display_enable = (flags & 0x80) == 1;
        self.window_tile_map_display_select: = (flags & 0x40) == 1;
        self.window_display_enable: = (flags & 0x20) == 1;
        self.bg_window_tile_data_select: = (flags & 0x10) == 1;
        self.bg_tile_map_display_select: = (flags & 0x08) == 1;
        self.sprite_size: = (flags & 0x04) == 1;
        self.sprite_display_enable: = (flags & 0x02) == 1;
        self.bg_window_display_priority: = (flags & 0x01) == 1;
    }

    pub fn get_flag(&mut self) -> u8 {
        (lcd_display_enable as u8) << 7 
            + (window_tile_map_display_select as u8) << 6
            + (window_display_enable as u8) << 5
            + (bg_window_display_priority as u8) << 4
            + (bg_tile_map_display_select as u8) << 3
            + (sprite_size as u8) << 2
            + (sprite_display_enable as u8) << 1
            + (bg_window_display_priority as u8)
    }
}

enum Mode {
    hblank,
    vblank,
    oam,
    vram,
}

impl Mode {
    fn get_flags(&self) -> u8 {
        let flag = match self {
            Mode::hblank => MODE_HBLANK,
            Mode::vblank => MODE_VBLANK,
            Mode::oam => MODE_OAM,
            Mode::vram => MODE_VRAM,
        };
        flag as u8
    }

    fn get_mode(code: u8) -> Self {
        match code {
            MODE_HBLANK => Mode::hblank,
            MODE_VBLANK => Mode::vblank,
            MODE_OAM => Mode::oam,
            MODE_VRAM => Mode::vram,
        }
    }
}


