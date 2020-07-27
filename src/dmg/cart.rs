// Cartridge file!!
// Handles all reading files
// use std::fmt;  Unused for now, removed to not incur the wrath of compiler
use std::fmt;
use std::fmt::Debug;
use std::string::String;
use super::mbc::mbc_properties::{MbcType, MbcInfo, RamInfo};

pub struct Cart {
    program: Box<[u8]>,
    mbc_info: MbcInfo,
}

#[derive(Debug)]
pub enum DestinationCode {
    Japanese,
    NonJapanese,
}

// will be more in the future
// pub enum CartType {
//     RomOnly,
// }

impl Cart {
    pub fn new(program: Box<[u8]>) -> Self {
        Cart {
            program: program,
            mbc_info: Cart::get_mbc_info(&program),
        }
    }

    pub fn write(&self, _addr: u16, _val: u8) { 
        // Gameboy: read only, does not do anything
    }

    pub fn get_logo(&self) -> &[u8] {
        let slice = &self.program[0x0104..0x0133];
        slice
    }
        

    pub fn get_title(&self) -> String {     // title lies at 0x0134 - 0x0143
        let mut title = Vec::new();
        for i in 0x0134..0x0143 {
            title.push(self.program[i]);
        }
        String::from_utf8(title).unwrap()
    }

    pub fn get_mbc_info(program: &Box<[u8]>) -> MbcInfo {
        let ram_size = Cart::get_ram_size(program);
        let ram_info = if ram_size == 0 {
            None 
        } else {
            Some(
                RamInfo {
                    size: ram_size,
                    bank_count: Cart::ram_bank_count(program),
                }
            )
        };

        match program[0x0147] {
            0x00 => MbcInfo::new(MbcType::None, ram_info, false),
            0x01 => MbcInfo::new(MbcType::Mbc1, ram_info, false),
            0x02 => MbcInfo::new(MbcType::Mbc1, ram_info, false),
            0x03 => MbcInfo::new(MbcType::Mbc1, ram_info, true),
            0x05 => MbcInfo::new(MbcType::Mbc2, ram_info, false),
            0x06 => MbcInfo::new(MbcType::Mbc2, ram_info, true),
            0x0F => MbcInfo::new(MbcType::Mbc3, ram_info, true),
            0x10 => MbcInfo::new(MbcType::Mbc3, ram_info, true),
            0x11 => MbcInfo::new(MbcType::Mbc3, ram_info, false),
            // For mbc5
            //0x00 => MbcInfo::new(MbcType::None, ram_info, false),
            //0x00 => MbcInfo::new(MbcType::None, ram_info, false),
            //0x00 => MbcInfo::new(MbcType::None, ram_info, false),
            _ => panic!("Haven't developed MBCs yet!"),
        }
    }

    pub fn get_rom_size(&self) -> u32 {
        match self.program[0x0148] {
            0x00 => 1024 * 32,
            0x01 => 1024 * 64,
            0x02 => 1024 * 128,
            0x03 => 1024 * 256,
            0x04 => 1024 * 512,
            0x05 => 1024 * 1024, // 1MB
            0x06 => 1024 * 1024 * 2,
            0x07 => 1024 * 1024 * 4,
            0x08 => 1024 * 1024 * 8,
            _ => panic!("Invalid ROM size"),
        }
    }

    pub fn rom_bank_count(&self) -> u32 {
        if self.get_rom_size() == 1024 * 32 {
            0
        } else {
            self.get_rom_size() / (1024 * 16)
        }
    }
    
    // Do not take in &self as this is needed for initialisation
    pub fn get_ram_size(program: &Box<[u8]>) -> u32 {
        match program[0x0149] {
            0 => 0,
            1 => 1024 * 2,
            2 => 1024 * 8,
            3 => 1024 * 32,
            4 => 1024 * 128, // in bytes
            5 => 1024 * 64,
            _ => panic!("Unsupported ram size: {:x}", program[0x0149]),
        }
    }

    // Do not take in &self as this is needed for initialisation
    pub fn ram_bank_count(program: &Box<[u8]>) -> u32 {
        let ram_size = Cart::get_ram_size(program) / 1024; // number of kb

        match ram_size {
            0 => 0,
            2..=8 => 1,
            32..=128 => ram_size / 8,
            _ => panic!("Invalid ram size: {} kb", ram_size),
        }
    }

    pub fn get_dest(&self) -> DestinationCode {
        match self.program[0x014A] {
            0 => DestinationCode::Japanese,
            1 => DestinationCode::NonJapanese,
            _ => panic!("Unknown destination"),
        }
    }

    pub fn check_sum(&self) -> bool {
        let default = self.program[0x014D];

        let mut x: i16 = 0;
        let final_x: u8;
        for i in 0x0134..0x014C {
            x = x - (self.program[i] as i16) - 1;
        }
        final_x = ((x as u16) & 0x00FF) as u8;
        
        final_x == default
    }

    pub fn read(&self, addr: u16) -> u8 {
        // Change to support MBC
        self.program[addr as usize]
    }
}

impl Debug for Cart {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,
               "Cart {{
                    title: {},
                    size: {:?},
                    destination_code: {:?},
                }}",
               self.get_title(),
               // self.mbc_info(),
               self.get_rom_size(),
               // self.rom_bank_count(),
               self.get_dest())
    }
}

