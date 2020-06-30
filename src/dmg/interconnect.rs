use super::ppu::Ppu;
use super::cart::Cart;
// use super::timer::Timer;
use super::gamepad::Gamepad;

const RAM_SIZE: usize = 32 * 1024; // Memory for the last 32KB as first 32KB is for ROM
const ZERO_PAGE: usize = 0x7f;

pub struct Interconnect {
    pub cart: Cart,
    ppu: Ppu,
    ram: Box<[u8]>,      
    zero_page: Box<[u8]>,
    ppu_dma: u8, // DMA Transfer and Start Address, 0xFF46
    pub int_enable: u8,
    pub int_flags: u8,
    pub gamepad: Gamepad,
    // TODO: Sound Processing unit, gamepad, timer
}

impl Interconnect {
    pub fn new(cart: Cart,
               ppu: Ppu,
               gamepad: Gamepad)
               -> Interconnect {
        Interconnect {
            cart: cart,
            ppu: ppu,
            // spu: spu,
            // timer: Timer::new(),
            ram: vec![0; RAM_SIZE].into_boxed_slice(),
            zero_page: vec![0; ZERO_PAGE].into_boxed_slice(),
            ppu_dma: 0,
            int_enable: 0,
            int_flags: 0,
            gamepad: gamepad,
        }
    }

    pub fn read(&mut self, addr: u16) -> u8 {
        match addr {
            // For more information: http://gameboy.mongenel.com/dmg/asmmemmap.html
            0x0000..= 0x7fff => self.cart.read(addr), // Cartridge ROM
            0x8000..= 0x9fff => self.ppu.read(addr), // Picture Processing Unit
            0xa000..= 0xbfff => 0, // Cartridge swappable RAM
            0xc000..= 0xdfff => self.ram[(addr - 0xc000) as usize], // Internal RAM
            // Might cause problems in GBC implementation but for DMG should be ok
            0xe000..= 0xfdff => self.read(addr - 0xe000 + 0xc000), 
            // Echo memory. Just copies over 0xc000..oxcfff

            // PPU addresses
            0xfe00..= 0xfe9f // Object Attribute Memory, in PPU / Sprite RAM
                | 0xff40..= 0xff45 // LCDC, LCDStat, SCY, SCX, LY, LYC
                | 0xff47..= 0xff4b // BGP, Object Palette Data 0-1, WY, WX, 
                | 0xff68..= 0xff69 // LCD Color Palette for CGB
                | 0xff4f => { // Destination Memory Bank
                self.ppu.read(addr)
            }

            // Unused memory
            0xfea0..= 0xfeff => 0,

            // 0xFF00 - 0xFF7F: Hardware I/O Registers
            // Details http://marc.rawer.de/Gameboy/Docs/GBCPUman.pdf pg35
            // 0xFF00: Gamepad (TODO)
            // 0xff00 => self.gamepad.read(),

            // 0xFF01 - 0xFF02: serial I/O, used for linking up to other gameboy
            0xff01..= 0xff02 => 0,
            
            // 0xFF04: DIV/Divider Register, incremented 16384 times a second.
            //         Needs to be implemented in timer.
            // 0xFF05: TIMA / Timer Counter - Interrupt when overflow.
            // 0xFF06: TMA / Timer Modulo - used when TIMA overflows.
            // 0xFF07: TAC / Timer Control. 3 bits: MSB is stop/start timer (0/1)
            //         2LSB is input clock speed (4096kHz, 262114 kHz, 65536kHz, 16384kHz)
            0xff04..= 0xff07 => 0,

            // 0xFF08 - 0xFFOE unused

            // 0xFFOF - IF / Interrupt Flag
            0xff0f => self.int_flags,

            // 0xFFFF - IE / Interupt Enable
            0xffff => self.int_enable,

            // 0xFF10 - 0xFF3F: SPU (Not implemented yet)
            0xff10..= 0xff3f => 0,

            // http://marc.rawer.de/Gameboy/Docs/GBCPUman.pdf pg 55
            0xff46 => self.ppu_dma,

            // Unusable memory, used as a speed switch (TODO)
            // 0xff4d => 0, 
            0xff80..= 0xfffe => self.zero_page[(addr - 0xff80) as usize],
            
            _ => 0 //panic!("Read: addr not in range: 0x{:x}", addr),
        }
    }

    pub fn write(&mut self, addr: u16, val: u8) {
        match addr {
            // Cartridge rom
            0x0000..= 0x7FFF => self.cart.write(addr, val),
            // character ram (basically tile data)
            0x8000..= 0x9FFF => self.ppu.write(addr, val),
            // Cartridge RAM to switch, now not available
            0xA000..= 0xBFFF => {},
            // Internal RAM (bank 0)
            0xC000..= 0xCFFF => self.ram[(addr - 0xc000) as usize] = val,
            // Internal RAM (Now fixed, will become switchable
            0xD000..= 0xDFFF => self.ram[(addr - 0xc000) as usize] = val,
            // Reserved part of RAM
            0xE000..= 0xFDFF => self.write(addr - 0xe000 + 0xc000, val),

            //0xFF00 => self.gamepad.write(val),
            0xFF00 => {},

            // Reserved memory for serial I/O Port
            0xFF01..= 0xFF02 => {},

            //0xFF04..= 0xFF07 =>self.timer.write(addr, val),
            0xFF04..= 0xFF07 => {},

            // Serial Interrupt
            0xFF0F => self.int_flags = val,
            
            //0xFF10..= 0xFF3F => self.spu.write(addr, val),
            0xFF10..=0xFF3F => {},
            
            // DMA Transfer, val is start address of DMA Transfer
            0xFF46 => {
                self.ppu_dma = val;
                self.ppu_dma_transfer()
            }

            // VRAM Sprite Attribute Table
            0xFE00..= 0xFE9F | 0xFF40..= 0xFF45 | 0xFF47..= 0xFF4B | 0xFF4F => {
                        self.ppu.write(addr, val);
            }

            // Speedswitch TODO, not implemented yet. Uses unused mem.
            // 0xFF4D => {},
            // for update_ram_offset(GBC?)
            0xFF70 => {},
            // Tetris uses this address for some reason
            0xFF7F => {},
            // Set hwram
            0xFF80..= 0xFFFE => self.zero_page[(addr-0xFF80) as usize] = val,
            // Set interrupt enable flag 
            0xFFFF => self.int_enable = val,
            _ => {} // panic!("Write: addr not in range!! 0x{:x} - val: 0x{:x}", addr, val),
        }
    }

    fn ppu_dma_transfer(&mut self) {
        // From PanDocs:
        // Writing to this register launches a DMA transfer 
        // from ROM or RAM to OAM memory (sprite attribute table). 
        // The written value specifies the transfer source address 
        // divided by 0x100, ie. source & destination are:
        // Source:      XX00-XX9F   ;XX in range from 00-F1h
        // Destination: FE00-FE9F

        let dma_start = (self.ppu_dma as u16) << 8;
        let dma_end = dma_start | 0x009f; //127, size of DMA

        // OAM_SIZE in ppu is the address for OAM, 0x100
        let mut oam = [0; super::ppu::OAM_SIZE];

        for a in dma_start..dma_end {
            oam[(a - dma_start) as usize] = self.read(a)
        }

        // just sets OAM memory
        self.ppu.oam_dma_transfer(oam);
    }
}
