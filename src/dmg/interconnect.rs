use super::ppu::Ppu;
use super::cart::Cart;
use super::timer::Timer;
use super::gamepad::Gamepad;

const RAM_SIZE: usize = 32 * 1024; // Memory for the last 32KB as first 32KB is for ROM
const HW_IO_REG: usize = 0x7f; //Hardware IO register memory FF00-FF7F; impl as hwram

pub struct Interconnect {
    pub cart: Cart,
    ppu: Ppu,
    ram: Box<[u8]>,      
    hwram: Box<[u8]>,
    ppu_dma: u8,
    pub int_enable: u8,
    pub int_flags: u8,
    ram_offset: usize,
    // TODO: Sound Processing unit, gamepad, timer
}

impl Interconnect {
    pub fn new(gameboy_type: GameboyType,
               cart: Cart,
               ppu: Ppu)
               -> Interconnect {
        Interconnect {
            cart: cart,
            ppu: ppu,
            // spu: spu,
            // timer: Timer::new(),
            // gamepad: gamepad,
            ram: vec![0; RAM_SIZE].into_boxed_slice(),
            hwram: vec![0; HW_IO_REG].into_boxed_slice(),
            ppu_dma: 0,
            int_enable: 0,
            int_flags: 0,
            ram_offset: 0,
        }
    }

    pub fn write(&mut self, addr: u16, val: u8) {
        match addr {
            // Cartridge rom
            0x0000...0x7FFF => self.cart.write(addr, val),
            // character ram (basically tile data)
            0x8000...0x9FFF => self.ppu.write(addr, val),
            // Cartridge RAM to switch, now not available
            0xA000...0xBFFF => {},
            // Internal RAM (bank 0)
            0xC000...0xCFFF => self.ram[(addr - 0xc000) as usize] = val,
            // Internal RAM (Now fixed, will become switchable
            0xD000...0xDFFF => self.ram[(addr - 0xc000) as usize] = val,
            // Reserved part of RAM
            0xE000...0xFDFF => self.write(addr - 0xe000 + 0xc000, val),

            //0xFF00 => self.gamepad.write(val),
            0xFF00 => {},

            // Reserved memory for serial I/O Port
            0xFF01...0xFF02 => {},

            //0xFF04...0xFF07 =>self.timer.write(addr, val),
            0xFF04...0xFF07 => {},

            // Serial Interrupt
            0xFF0F => self.int_flags = val,
            
            //0xFF10...0xFF3F => self.spu.write(addr, val),
            0xFF10..0xFF3F => {},
            
            // DMA Transfer, val is start address of DMA Transfer
            0xFF46 => {
                self.ppu_dma = val;
                self.ppu_dma_transfer()
            }

            // VRAM Sprite Attribute Table
            0xFE00...0xFE9F | 0xFF40...0xFF45 | 0xFF47...0xFF4B | 0xFF4F => {
                        self.ppu.write(addr, val);
            }

            // Speedswitch (GBC?)
            0xFF4D => {},
            // for update_ram_offset(GBC?)
            0xFF70 => {},
            // Tetris uses this address for some reason
            0xFF7F => {},
            // Set hwram
            0xFF80...0xFFFE => self.hwram[(addr-0xFF80) as usize] = val,
            // Set interrupt enable flag 
            0xFFFF => self.int_enable = val,
            _ => panic!("Write: addr not in range!! 0x{:x} - val: 0x{:x}", addr, val),
        }
    }
}
