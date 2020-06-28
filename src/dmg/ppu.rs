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


