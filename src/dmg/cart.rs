// Cartride file!!
// Handles all reading files
// use std::fmt;  Unused for now, removed to not incur the wrath of compiler
use std::fmt;
use std::fmt::Debug;
use std::string::String;

// use super::dmg_cpu::*;
#[derive(Debug)]
pub struct Cart {
    program: Box<[u8]>,
}
/* // Don't enable for bootrom test. Bootrom program is only 256 bytes long, does not contain info below
impl Debug for Cart {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f,
               "Cart {{
        title: {},
        size: {:?},
        destination_code: {:?},
        }}", 
                self.get_title(),
                self.get_rom_size(),
                self.get_dest())
    }
}
*/

#[derive(Debug)]
pub enum DestinationCode {
    Japanese,
    NonJapanese,
}

// will be more in the future
pub enum CartType {
    RomOnly,
}

impl Cart {
    pub fn new(program: Box<[u8]>) -> Self {
        Cart {
            program: program,
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

    pub fn get_type(&self) -> CartType {
        match self.program[0x0147] {
            0x00 => CartType::RomOnly,
            _ => panic!("Haven't developed MBCs yet!"),
        }
    }

    pub fn get_rom_size(&self) -> u32 {
        match self.program[0x0148] {
            0x00 => 1024 * 32,
            _ => panic!("Have not implemented banking for > 32kb"),
        }
    }
    
    pub fn get_ram_size(&self) -> u32 {
        match self.program[0x0149] {
            0 => 0,
            1 => 1024 * 2,
            2 => 1024 * 8,
            3 => 1024 * 32,
            4 => 1024 * 128, // in bytes
            5 => 1024 * 64,
            _ => panic!("Unsupported ram size: {:x}", self.program[0x0149]),
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
        self.program[addr as usize]
    }
}


