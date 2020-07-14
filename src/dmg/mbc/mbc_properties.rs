// Memory Bank Controller (MBC):
// For ROMs larger than 32kb, Memory Bank Controller expands available address space. These MBC
// Chips are located in the Catridge, not the Gameboy itself. 

#[derive(Debug)]
pub enum MbcType { // Should be specified at byte (0x0147) in ROM.
    None, // No MBC
    Mbc1,
    Mbc2,
    Mbc3,
    Mbc5,
}

// MBC should be able to read and write to any bank, given an address.
// MBC should be able to read and write to RAM as well, to interact with other hardware such as
// Display Control Registers etc...
pub trait Mbc {
    // read / write operations for Mbc
    fn read(&self, rom: &Box<[u8]>, addr: u16) -> u8;
    fn write(&mut self, addr: u16, content: u8);
    // read / write operations interacting with RAM
    fn read_ram(&self, addr: u16) -> u8;
    fn write_ram(&mut self, addr: u16, val: u8);
    // Return RAM. Read up first
    // fn copy_ram(&self) -> Option<Box<[u8]>>; // ????
}

// ROM Bank (read_only). Contains the first 16kb of Catridge ROM, 
// and any of the further 16kb banks of the ROM.
pub struct RomInfo {
    

}

// RAM Bank (Read / Write). Helps store states even when gameboy is turned off 
// such as high score, game positions.
pub struct RamInfo {
    size: u32,

}


