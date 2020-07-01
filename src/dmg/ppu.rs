use super::Interrupts;
use super::console::VideoSink;

pub const OAM_SIZE: usize = 0x100; // address for OAM
const FRAMEBUFFER_SIZE: usize = DISPLAY_WIDTH * DISPLAY_HEIGHT; // address for the full frame,

 const CLKS_SCREEN_REFRESH: u32 = 70224; // refresh every 70224 clks not used at the moment
pub const DISPLAY_WIDTH: usize = 160;
pub const DISPLAY_HEIGHT: usize = 144;

pub const VRAM_SIZE: usize = 1024*16; // 16KB Vram

const MODE_HBLANK: u8 = 0;
const MODE_VBLANK: u8 = 1;
const MODE_OAM: u8 = 2;
const MODE_VRAM: u8 = 3;

// const HBLANK_CYCLES: u32 = 204;
// const VBLANK_CYCLES: u32 = 456;
// const OAM_CYCLES: u32 = 80;
// const VRAM_CYCLES: u32 = 172;
// 
// const SPRITES_PER_Y_LINE: u16 = 40;
const TILE_BYTES: u16 = 16;
const TILE_BASE_ADDR: u16 = 0x8000;

#[derive(Debug,PartialEq,Eq)]
pub struct Color {
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
}

impl Lcdc {
    pub fn new() -> Self {
        Lcdc {
            lcd_display_enable : true,
            window_tile_map_display_select : false,
            window_display_enable : false,
            bg_window_tile_data_select : true,
            bg_tile_map_display_select : false,
            sprite_size : false,
            sprite_display_enable : false,
            bg_window_display_priority : true,
        } 
    }

    pub fn set_flags(&mut self, flags: u8) {
        self.lcd_display_enable = (flags & 0x80) == 1;
        self.window_tile_map_display_select = (flags & 0x40) == 1;
        self.window_display_enable = (flags & 0x20) == 1;
        self.bg_window_tile_data_select = (flags & 0x10) == 1;
        self.bg_tile_map_display_select = (flags & 0x08) == 1;
        self.sprite_size = (flags & 0x04) == 1;
        self.sprite_display_enable = (flags & 0x02) == 1;
        self.bg_window_display_priority = (flags & 0x01) == 1;
    }

    pub fn get_flags(&mut self) -> u8 {
        (self.lcd_display_enable as u8) << 7 
            + (self.window_tile_map_display_select as u8) << 6
            + (self.window_display_enable as u8) << 5
            + (self.bg_window_display_priority as u8) << 4
            + (self.bg_tile_map_display_select as u8) << 3
            + (self.sprite_size as u8) << 2
            + (self.sprite_display_enable as u8) << 1
            + (self.bg_window_display_priority as u8)
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
            mode_flag: Mode::Vblank,                // R
        } 
    }

    pub fn set_flags(&mut self, flags: u8) {
        self.lcd_ly_coincidence_interrupt = (flags & 0x40) == 1;
        self.mode_2_oam_interrupt = (flags & 0x20) == 1;
        self.mode_1_vblank_interupt = (flags & 0x10) == 1;
        self.mode_0_hblank_interrupt = (flags & 0x8) == 1;
        //coincidence_flag read only
        //mode_flag read only
    }

    pub fn get_flags(&mut self) -> u8 {
        (self.lcd_ly_coincidence_interrupt as u8) << 6
            + (self.mode_2_oam_interrupt as u8) << 5
            + (self.mode_1_vblank_interupt as u8) << 4
            + (self.mode_0_hblank_interrupt as u8) << 3
            + (self.coincidence_flag as u8) << 2
            + self.mode_flag.get_flags() 
    }
}

enum Mode {
    Hblank,
    Vblank,
    Oam,
    Vram,
}

impl Mode {
    fn get_flags(&self) -> u8 {
        let flag = match self {
            Mode::Hblank => MODE_HBLANK,
            Mode::Vblank => MODE_VBLANK,
            Mode::Oam => MODE_OAM,
            Mode::Vram => MODE_VRAM,
        };
        flag as u8
    }
/*
    fn get_mode(code: u8) -> Self {
        match code {
            MODE_HBLANK => Mode::Hblank,
            MODE_VBLANK => Mode::Vblank,
            MODE_OAM => Mode::Oam,
            MODE_VRAM => Mode::Vram,
            _ => panic!("Unknown mode code"),
        }
    }
*/
}

pub struct Ppu {
    lcdc: Lcdc,
    lcdstat: LCDStat,
    scx: u8,
    scy: u8,
    ly: u8,     // read-only?
    lyc: u8,
    wy: u8,
    wx: u8,
    // information for palette
    bgp: u8,    // BG Palette Data, addr at FF47
    obp0: u8,   // Object Palette 0 Data, addr at FF48
    obp1: u8,   // Object palette 1 data, addr at FF49
    vram: [u8; VRAM_SIZE],
    oam: [u8; OAM_SIZE],
    lcd_tiles: [u32; DISPLAY_WIDTH * DISPLAY_HEIGHT], // array of bytes representing all lcd tiles
    cycles: u8, // keep track of number of cycles
    mode_cycles: u8,    // Keep track of timing in each interrupt mode
    framebuffer: [u8; FRAMEBUFFER_SIZE],    // To render images before showing to the screen
}

impl Ppu {
    pub fn new() -> Self {
        Ppu {
            lcdc: Lcdc::new(),
            lcdstat: LCDStat::new(),
            scx: 0,
            scy: 0,
            ly: 0,
            lyc: 0,
            wy: 0,
            wx: 7,
            bgp: 0,
            obp0: 0,
            obp1: 0,
            vram: [0; VRAM_SIZE],
            oam: [0; OAM_SIZE],
            lcd_tiles: [0; DISPLAY_WIDTH * DISPLAY_HEIGHT], // array of bytes representing lcd_screen
            cycles: 0,
            mode_cycles: 0,
            framebuffer: [0; FRAMEBUFFER_SIZE],
        }
    }

    pub fn write(&mut self, addr: u16, val: u8) {
        match addr {
            0x8000..=0x97ff => { // tile data
                let addr = addr - TILE_BASE_ADDR;
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
            0xFF47 => self.bgp = val,
            0xFF48 => self.obp0 = val,
            0xFF49 => self.obp1 = val,
            _ => panic!("Unsupported address to write to"),
        }
    }

    pub fn read(&mut self, addr: u16) -> u8 {
        match addr {
            0x8000..=0x97ff => { // tile data
                let addr = addr - TILE_BASE_ADDR;
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
            0xFF47 => self.bgp,
            0xFF48 => self.obp0,
            0xFF49 => self.obp1,
            _ => panic!("Unsupported address to write to"),
        }
    }

    pub fn cycle_flush(&mut self, cycle_count: u32, video_sink: &mut dyn VideoSink) -> Interrupts {
        Interrupts::empty() // temporary, will change later        


    }

    pub fn oam_dma_transfer(&mut self, oam: [u8; OAM_SIZE]) {
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
        };   

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
        };

        // set the y-position (top row)
        let y_pos = if use_window {
            scanline.wrapping_sub(window_y)
        } else {
            scroll_y.wrapping_add(scanline)
        };

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

            let tile_col: u16 = (x_pos / 8) as u16;

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
        let is_size_8x16: bool = self.lcdc.sprite_size;
        
        // maximum 40 sprites in the screen
        for sprite in 0..40 { 
            // sprite information takes up 4 bytes in OAM
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
            let palette_bit = (attributes & 0b0001_0000) >> 4;
           
            // will display the first 10 sprites appearing on this line
            let scanline = self.ly;

            let y_size = if is_size_8x16 { 16 } else { 8 };

            // Compares scanline to self.ly to find the 10 sprites on the line that appear first
            // on OAM. (FE00-FE03 = first sprite, FE04 - FE07 2nd sprite and so on. Rank is used to
            // store order of appearance
            if scanline >= y_pos && scanline < (y_pos.wrapping_add(y_size)) {
                // Finding out which line sprite is at in the OAM.
                let rank: i32 = scanline as i32 - y_pos as i32;
                // if y_flip: mirror the line over the y-axis, so find in the other direction.
                let rank = if y_flip > 0 {
                    (rank - y_size as i32) * (-1)
                } else {
                    rank
                };
                // tile data is stored in Vram at base addr 0x8000, each tile is 16-byte long.
                // From base addr, go to specified 16-byte tile, then identify the exact starting addr of sprite color info.
                let sprite_addr = TILE_BASE_ADDR + (sprite_tile_addr * TILE_BYTES) + (rank as u16) * 2;
                let lsb_line = self.read(sprite_addr as u16);
                let msb_line = self.read((sprite_addr + 1) as u16);

                // looking at every pair of bit from 7 to 0, if x_flip we look at them from 0 to 7.
                for tile_pixel in (0..8).rev() {
                    let color_bit = tile_pixel as i32;
                    let color_bit = if x_flip > 0 {
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
                    let palette_num = if palette_bit == 0 {
                        self.obp0
                    } else {
                        self.obp1
                    };

                    let color = self.get_color(color_num, palette_num);
                    
                    // x_pix goes opposite direction with tile_pixel (if tile_pixel goes from 7 to
                    // 0, x_pix goes from 0 to 7 (FIFO)
                    let x_pix = (0 as u8).wrapping_sub(tile_pixel as u8).wrapping_add(7);
                    // Go to the specific pixel's x-coordinate, y-coordinate is the scanline
                    let pixel_x = x_pos.wrapping_add(x_pix);
                   
                    // scanline > 143 => VBlank => Nothing in background
                    // pixel_x > 159 => not drawn
                    if scanline > 143 || pixel_x > 159 {
                        continue;
                    }

                    self.set_sprite_pixel(pixel_x as u32, scanline as u32, obj_to_bg_priority > 0, color);
                }
            }
        }
    }

    pub fn get_color(&mut self, color_id: u8, palette_num: u8) -> Color {
        // Determine which bit to look at in palette num, based on color number 0 1 2 or 3
        let (msb, lsb) = match color_id {
            0 => (1, 0),
            1 => (3, 2),
            2 => (5, 4),
            3 => (7, 6),
            _ => panic!("Unsupported color"),
        };

        // put specified bits together from palette num
        let color = (((palette_num >> msb) & 0x01) << 1) | ((palette_num >> lsb) & 0x01);
        
        // Return color based on specified number in color
        match color  {
            0 => WHITE,
            1 => LIGHT_GRAY,
            2 => DARK_GRAY,
            3 => BLACK,
            _ => panic!("Invalid color!!: 0x{:x}", color),
        }
    }

    pub fn set_sprite_pixel(&mut self, pixel_x: u32, y_line: u32, priority: bool, color: Color) {
        // tile_index: from coordinates, derive index of tile in array of bytes representing lcd_screen. 
        // Each y coordinate can contain 160 (display width) tiles
        let tile_index = ((y_line * DISPLAY_WIDTH as u32) + pixel_x) as usize;

        let prev_pixel = Color {
            a: (self.lcd_tiles[tile_index] >> 24) as u8,
            r: (self.lcd_tiles[tile_index] >> 16) as u8,
            g: (self.lcd_tiles[tile_index] >> 8) as u8,
            b: self.lcd_tiles[tile_index] as u8,
        };

        // if color of previous tile is not white and it has higher priority, don't draw next tile
        if prev_pixel != WHITE && priority {
            return;
        } else {
            self.set_pixel(pixel_x, y_line, color)
        }
    }

    pub fn set_pixel(&mut self, x: u32, y: u32, color: Color) {
        let tile_index = ((y * DISPLAY_WIDTH as u32) + x) as usize;
        
        let c = ((color.a as u32) << 24) | ((color.r as u32) << 16) | ((color.g as u32) << 8) | (color.b as u32);

        self.lcd_tiles[tile_index] = c;
    }

}
