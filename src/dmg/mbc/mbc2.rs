// MBC2 implementation
// 256KByte ROM and 512x4 bits RAM

use super::Mbc; // trait
use super::RamInfo; // struct
use super::MbcInfo; // struct

pub struct Mbc2 {
    ram_flag: bool,
    rom_bank_0: u8,
    rom_bank_1: u8,
    rom_offset: usize,
    ram: [u8; 512],
}

impl Mbc2 {
    pub fn new(mbc_info: MbcInfo, _ram: Option<Box<[u8]>>) -> Mbc2 {
        // Problematic code???
        // let ram = if let Some(extern_ram) = mbc_info.ram_info {
        //     extern_ram.make_external_ram(ram)
        // } else {
        //     vec![0; 512]
        // };
        
        Mbc2 {
            ram_flag: true,
            rom_bank_0: 0,
            rom_bank_1: 1,
            rom_offset: 0x4000,
            ram: [0; 512],
        }
    }
}

impl Mbc for Mbc2 {
    fn read_rom(&self, rom: &Box<[u8]>, addr: u16) -> u8 {
        match addr {
            0x0000..=0x3FFF => rom[addr as usize],
            0x4000..=0x7FFF => rom[addr as usize + self.rom_offset],
            _ => panic!("Unsupported address 0x{:x}", addr),
        }   
    }
    
    #[allow(dead_code)]
    // TODO: check logic
    fn write_rom(&mut self, addr: u16, content: u8) {
        match addr {
            0x0000..=0x1FFF => if (addr & 0x0100) == 0 {
                self.ram_flag = !self.ram_flag; // Depends on content, not address
            },
            0xA000..=0xA1FF => self.ram[(addr - 0xA000) as usize] = content,
            0x2000..=0x3FFF => if (addr | 0x0100) == 0 {
                let mut new_rom: u8 = 1;
                if content | 0xF == 0 {
                    new_rom = 1;
                } else {
                    new_rom = content | 0xF;
                }
                self.rom_offset = ((new_rom - 1) as usize * 0x4000 as usize) as usize; // update rom offset. why new_rom - 1? Overflow error
            },
            _ => panic!("unsupported address 0x{:x}", addr),
        }
    }

    fn read_ram(&self, addr: u16) -> u8 {
        if self.ram_flag {
            self.ram[addr as usize]
        } else {
            0
        }
    }

    fn write_ram(&mut self, addr: u16, content: u8) {
        if self.ram_flag {
            self.ram[addr as usize] = content;
        }
    }

    fn copy_ram(&self) -> Option<Box<[u8]>> {
        if self.ram.len() > 0 {
            let ram_box = Box::new(self.ram.clone());
            Some(ram_box)
        } else {
            None
        }
    }
}
