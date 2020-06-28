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

struct Lcdc {
    lcd_display_enable: bool,
    window_tile_map_display_select: bool,
    window_display_enable: bool,
    bg_window_tile_data_select: bool,
    bg_tile_map_display_select: bool,
    sprite_size: bool,
    sprite_display_enable: bool,
    bg_window_display_priority: bool,
};

impl Lcdc {
    pub fn new() -> Self {
        Lcdc {
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
        LCDStat {
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
    lcdc: Lcdc,
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
            lcdc: Lcdc::new(),
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

    pub fn oam_dma_transfer(&mut self, oam: Box<[u8]>) {
        self.oam = oam;
    }

    pub fn draw_scanline(&mut self) {
        if self.lcdc.bg_window_display_priority {
            self.render_tiles();
        }

        if self.lcdc.sprite_display_enable {
            self.render_sprites();
        }
    }

    pub fn render_tiles(&mut self) {
        let scanline = self.ly;
        let scroll_x = self.scx;
        let scroll_y = self.scy;
        let window_x = self.wx;
        let window_y = self.wy.wrapping_sub(7);

        // Window used if the flag in LCDC is true and the window is below scanline
        let use_window = self.lcdc.window_display_enable && window_y <= scanline;

        // Check which VRAM tile data is used
        // Based on LCDC flag
        // See VRAM Tile Data in PanDocs
        let (tile_data, signed): (u16, bool) = if self.lcdc.bg_window_tile_data_select {
            (0x8000, false)
        } else {
            (0x8800, true)
        }   

        // See VRAM Background Maps in PanDocs
        let background_mem = if use_window {
            // Window used. Background defaults to window tiles.
            if self.lcdc.window_tile_map_display_select {
                0x9c00
            } else {
                0x9800
            }
        } else {
            // Window not used. Background tiles used.
            if self.lcdc.bg_window_tile_data_select {
                0x9c00
            } else {
                0x9800
            }
        }

        // set the y-position (top row)
        let y_pos = if use_window {
            scanline.wrapping_sub(window_y)
        } else {
            scroll_y.wrapping_add(scanline)
        }

        // 32 tiles per row, 8 pixels each
        let tile_row: u16 = (y_pos / 8) as u16 * 32;

        // Display: 160 x 144 on the screen
        // We do line by line
        for pixel in 0..160 {
            let pixel = pixel as u8;
            
            // Window used?
            let x_pos = if use_window && pixel >= window_x {
                pixel.wrapping_sub(window_x)
            } else {
                scroll_x.wrapping_add(scroll_x)
            };

            let tile_col: u16 = (x_pos / 8);

            // Base address of the tile
            let tile_address = background_mem + tile_row + tile_col;

            // sets the offset from the base address
            let tile_num: i16 = if !signed {
                // u8 -> u16 (still unsigned) -> i16 (no op)
                self.read(tile_address) as u16 as i16 
            } else {
                // u8 -> i8 (sign) -> i16
                self.read(tile_address) as i8 as i16
            };

            // Actual tile location address
            let tile_location: u16 = if !signed {
                tile_data + (tile_num as u16 * 16)
            } else {
                tile_data + ((tile_num + 128) * 16) as u16
            };

            // Color code (position in memory): base address
            let line = (y_pos as u16 % 8) * 2;

            // Get a line of bytes that signifies the y-coordinate lsb/msb color
            let lsb_line = self.read(line + tile_location);
            let msb_line = self.read(line + tile_location + 1);

            // See how many bits needed to locate the actual pixel's msb/lsb
            // i.e. the pixel's location in the line
            let color_bit = ((x_pos as i32 % 8) - 7) * -1;

            // 0, 1, 2, or 3: white, light grey, dark grey, black
            let color_num = (((msb_line >> color_bit) & 0b1) << 1) | ((lsb_line >> color_bit) & 0b1);

            // get color from color enum
            let color = self.get_color(color_num, self.bgp);

            // set the pixel
            self.set_pixel(pixel as u32, scanline as u32, color)
        }
    }
    
    pub fn render_sprites(&mut self) {
        let use_8x16 = self.lcdc.sprite_size;
        
        // maximum 40 sprites in the screen
        for sprite in 0..40 { 
            // each sprite takes up 4 bytes(8*8 pixel)
            let index: u8 = sprite * 4;
            // y-coordinate of top left corner
            let y_pos = self.oam[index as usize].wrapping_sub(16); 
            // x_coord of top left corner
            let x_pos = self.oam[(index + 1) as usize].wrapping_sub(8);
            // address of tile
            let sprite_tile_addr = self.oam[(index + 2) as usize] as u16; 
            // flags that represent attributes of sprite
            let attributes = self.oam[(index + 3) as usize];
            // extract info from attributes flag
            let obj_to_bg_priority = (attributes & 0b1000_0000) >> 7;
            let y_flip = (attributes & 0b0100_0000) >> 6;
            let x_flip = (attributes & 0b0010_0000) >> 5;
            let palette_num = (attributes & 0b0001_0000) >> 4;
           
            // will display the first 10 sprites appearing on this line
            let scanline = self.ly;

            let y_size = if use_8x16 { 16 } else { 8 };
            let x_size = 8;

            // Compares scanline to self.ly to find the 10 sprites on the line that appear first
            // on OAM. (FE00-FE03 = first sprite, FE04 - FE07 2nd sprite and so on.
            if scanline >= y_pos && scanline < (y_pos.wrapping_add(y_size)) {
                // Finding out which line sprite is at in the OAM.
                let line: i32 = scanline as i32 - y_pos as i32;
                // if y_flip: mirror the line over the y-axis, so find in the other direction.
                let line = if (y_flip as bool) {
                    (line - y_size as i32) * (-1)
                } else {
                    line
                };
            
                // each sprite is represented by 2 bytes, so distance is x2.
                let line = line * 2;
                
                // addr = base_addr + wtf is going on
                let sprite_addr = 0x8000 + (sprite_tile_addr * 16) + line as u16;
                let lsb_line = self.read(sprite_addr as usize);
                let msb_line = self.read((sprite_addr + 1) as usize);

                // looking at every pair of bit from 7 to 0, if x_flip we look at them from 0 to 7.
                for tile_pixel in (0..8).rev() {
                    let color_bit = tile_pixel as i32;
                    let color_bit = if x_flip as bool {
                        (color_bit - 7) * (-1)
                    } else {
                        color_bit
                    };

                    // Put together the color bits
                    let color_num = (((msb_line >> color_bit) & 0b01) << 1) | ((lsb_line >> color_bit) & 0b01);
                    
                    if color_num == 0 { // transparent, do not draw
                        continue;
                    }
                    // get sprite color
                    let color = self.get_color(color_num, palette_num);
                    
                    let x_pix = (0 as u8).wrapping_sub(tile_pixel as u8).wrapping_add(7); // ??
                    let pixel = x_pos.wrapping_add(x_pix); //??
                    
                    if scanline > 143 || pixel > 159 {
                        continue;
                    }

                    self.set_sprite_pixel(pixel as u32, scanline as u32, obj_to_bg_priority, color);
                }
            }
        }
    }



}
