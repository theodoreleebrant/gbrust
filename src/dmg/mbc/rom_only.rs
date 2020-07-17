// rom_only contains implementation for MBC if cartridge has no mbc
// standard imports. Use for other Mbcs too
use super::Mbc; // trait
use super::RamInfo; // struct
use super::MbcInfo; // struct

pub struct RomOnly {}

impl Mbc for RomOnly {
    pub fn read(&self, rom: Box<[u8]>, addr: u16) -> u8 {
        rom[addr as usize]
    }
    
    #[allow(dead_code)]
    pub fn write(&mut self, addr: u16, content: u8) {
        // does nothing
    }

    pub fn read_ram(&self, addr: u16) -> u8 {
        0
    }

    pub fn write_ram(&mut self, addr: u16, content: u8) {

    }
}
