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

    pub fn set_flags(&mut self, flags: u8) {
        self.lcd_display_enable = (flags & 0x80) == 1;
        self.window_tile_map_display_select: = (flags & 0x40) == 1;
        self.window_display_enable: = (flags & 0x20) == 1;
        self.bg_window_tile_data_select: = (flags & 0x10) == 1;
        self.bg_tile_map_display_select: = (flags & 0x08) == 1;
        self.sprite_size: = (flags & 0x04) == 1;
        self.sprite_display_enable: = (flags & 0x02) == 1;
        self.bg_window_display_priority: = (flags & 0x01) == 1;
    }

    pub fn get_flags(&mut self) -> u8 {
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

struct LCDStat {
    lcd_ly_coincidence_interrupt: bool,
    mode_2_oam_interrupt: bool,
    mode_1_vblank_interupt: bool,
    mode_0_hblank_interrupt: bool,
    coincidence_flag: bool,
    mode_flag: Mode,
}

impl LCDStat {
    pub fn new() -> Self {
        LCDC {
            lcd_ly_coincidence_interrupt: false,    // RW
            mode_2_oam_interrupt: false,            // RW
            mode_1_vblank_interupt: false,          // RW
            mode_0_hblank_interrupt: false,         // RW
            coincidence_flag: false,                // R
            mode_flag: Mode::vblank,                // R
        } 
    }

    pub fn set_flags(&mut self, flags: u8) {
        lcd_ly_coincidence_interrupt = (flags & 0x40) == 1;
        mode_2_oam_interrupt = (flags & 0x20) == 1;
        mode_1_vblank_interupt = (flags & 0x10) == 1;
        mode_0_hblank_interrupt = (flags & 0x8) == 1;
        //coincidence_flag read only
        //mode_flag read only
    }

    pub fn get_flags(&mut self) -> u8 {
        (lcd_ly_coincidence_interrupt as u8) << 6
            + (mode_2_oam_interrupt as u8) << 5
            + (mode_1_vblank_interupt as u8) << 4
            + (mode_0_hblank_interrupt as u8) << 3
            + (coincidence_flag as u8) << 2
            + mode_flag.get_flags() 
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

pub struct PPU {
    lcdc: LCDC,
    lcdstat: LCDStat,
    scx: u8,
    scy: u8,
    ly: u8,     // read-only?
    lyc: u8,
    wy: u8,
    wx: u8,
}

impl PPU {
    pub fn new() -> Self {
        PPU {
            lcdc: LCDC::new(),
            lcdstat: LCDStat::new(),
            scx: 0,
            scy: 0,
            ly: 0,
            lyc: false,
            wy: 0,
            wx: 7,
            vram: [u8; VRAM_SIZE],
            oam: [u8; OAM_SIZE),
        }
    }

    pub fn write(&mut self, addr: u16, val: u8) {
        match addr {
            0x8000..0x97ff => { // tile data
                let addr = addr - 0x8000;
                self.vram[addr as usize] = val;
            },
            0xFF40 => self.lcdc.set_flags(val),
            0xFF41 => self.lcdstat.set_flags(val),
            0xFF42 => self.scy = val,
            0xFF43 => self.scx = val,
            0xFF44 => self.ly = val,
            0xFF45 => self.lyc = val,
            0xFF4A => self.wy = val,
            0xFF4B => self.wx = val,
        }
    }

    pub fn read(&mut self, addr: u16) -> u8 {
        match addr {
            0x8000..0x97ff => { // tile data
                let addr = addr - 0x8000;
                self.vram[addr as usize]
            },  
            0xFF40 => self.lcdc.get_flags(),
            0xFF41 => self.lcdstat.get_flags(),
            0xFF42 => self.scy,
            0xFF43 => self.scx,
            0xFF44 => self.ly,
            0xFF45 => self.lyc,
            0xFF4A => self.wy,
            0xFF4B => self.wx,
        }
    }


}
