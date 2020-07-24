// rom_only contains implementation for MBC if cartridge has no mbc
// standard imports. Use for other Mbcs too
use super::Mbc; // trait
use super::RamInfo; // struct
use super::MbcInfo; // struct

pub struct RomOnly {}

impl Mbc for RomOnly {
    fn read_rom(&self, rom: &Box<[u8]>, addr: u16) -> u8 {
        rom[addr as usize]
    }
    
    #[allow(dead_code)]
    fn write_rom(&mut self, addr: u16, content: u8) {
        // does nothing
    }

    fn read_ram(&self, addr: u16) -> u8 {
        0
    }

    fn write_ram(&mut self, addr: u16, content: u8) {

    }

    fn copy_ram(&self) -> Option<Box<[u8]>> {
        None
    }
}
