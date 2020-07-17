// Memory Bank Controller (MBC):
// For ROMs larger than 32kb, Memory Bank Controller expands available address space. These MBC
// Chips are located in the Catridge, not the Gameboy itself.

//use super::rom_only::RomOnly;
//use super::mbc1::Mbc1;
//use super::mbc2::Mbc2;
//use super::mbc3::Mbc3;
//use super::mbc5::Mbc5;

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

/* MOVE THIS FUNCTION TO MOD.RS SO THAT IT HAS ALL THE MBC IMPLEMENTED
// Why box tho?
pub fn new_mbc(mbc_info: MbcInfo, ram: Option<Box<[u8]>>) -> Box<Mbc> {
    match mbc_info.mbc_type {
        MbcType::None => Box::new(RomOnly {}),
        MbcType::Mbc1 => Box::new(Mbc1::new(mbc_info, ram)),
        MbcType::Mbc2 => Box::new(Mbc2::new(mbc_info, ram)),
        MbcType::Mbc3 => Box::new(Mbc3::new(mbc_info, ram)),
        MbcType::Mbc5 => Box::new(Mbc5::new(mbc_info, ram)),
        _ => panic!("Unimplemented MBC"),
    }
}
*/
// Each MBC should carry following information:
// Type: Informs the configuration of how MBC switches banks
// ram_info: Information about RAM (Size and bank_ID 00 - 03)
// has_battery: RAM Bank accessed by MBC is battery_buffered. While still have battery, RAM Bank
// can store info even after cartridge is removed or GB is turned off
pub struct MbcInfo {
    mbc_type: MbcType,
    ram_info: Option<RamInfo>,
    has_battery: bool,
}

impl MbcInfo {
    pub fn new(mbc_type: MbcType, ram_info: Option<RamInfo>, has_battery: bool) -> Self {
        MbcInfo {
            mbc_type: mbc_type,
            ram_info: ram_info,
            has_battery: has_battery,
        }
    }
}

// RAM Bank (Read / Write). Helps store states even when gameboy is turned off 
// such as high score, game positions.
// Ram should contain info: size and bank count (00 - 03)
pub struct RamInfo {
    size: u32,
    bank_count: u32,
}

impl RamInfo {
    pub fn new(size: u32, bank_count: u32) -> Self {
        RamInfo {
            size: size,
            bank_count: bank_count,
        }
    }


    // Enable external RAM if any exists. If none exists, create a blank external RAM
    pub fn make_external_ram(self, saved_ram: Option<Box<[u8]>>) -> Box<[u8]> {
        match saved_ram {
            Some(extern_ram) => {
                if extern_ram.len() == self.size as usize { // if exisiting RAM matches the specified RAM size for MBC
                    extern_ram
                } else {
                    panic!("Saved_ram size mismatch! MBC expected {}, existing ram has size {}", self.size, extern_ram.len());
                }
            }
            // No external RAM => Create a blank one
            None => vec![0; self.size as usize].into_boxed_slice(),
        }
    }
}

