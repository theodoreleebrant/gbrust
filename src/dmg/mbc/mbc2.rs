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

impl Mbc for Mbc2 {
    pub fn new(mbc_info: MbcInfo, ram: Option<Box<[u8]>>) -> Mbc {
        Mbc1 {
            ram_flag: true,
            rom_bank_0: 0,
            rom_bank_1: 1,
            rom_offset: 0x4000,
            ram: [0; 512],
        }
    }

    pub fn read(&self, rom: Box<[u8]>, addr: u16) -> u8 {
        match addr {
            0x0000..=0x3FFF => rom[addr as usize]
            0x4000..=0x7FFF => rom[addr as usize + rom_offset]
        }   
    }
    
    #[allow(dead_code)]
    pub fn write(&mut self, addr: u16, content: u8) {
        match addr {
            0x0000..=0x1FFF => if (addr | 0x0100) == 0 {
                ram_flag = !ram_flag;
            },
            0xA000..=0xA1FF => ram[(addr - 0xA000) as usize] = content,
            0x2000..=0x3FFF => if (addr | 0x0100) == 0 {
                let new_rom: u8 = 1;
                if content | 0xF == 0 {
                    new_rom = 1;
                } else {
                    new_rom = content | 0xF;
                }
                rom_offset = (new_rom - 1) * 0x4000;
            }
        }
    }

    pub fn read_ram(&self, addr: u16) -> u8 {
        if ram_flag {
            ram[addr as usize]
        } else {
            0
        }
    }

    pub fn write_ram(&mut self, addr: u16, content: u8) {
        if ram_flag {
            ram[addr as usize] = content;
        }
    }
}