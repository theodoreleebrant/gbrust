// Implementing Mbc1. 
// Mbc1 consists of registers:
// RAM Enable (0x0000-0x1FFF)
// ROM bank no. (0x2000 - 0x3FFF)
// RAM bank no. (0x4000 - 0x5FFF)
// Banking Mode Select (0x6000 - 0x7FFF)
// and an external RAM, and a ram offset.

const ROM_BANK_SIZE: usize = 0x4000;
const RAM_BASE_ADDR: usize = 0xA000;

pub mut struct Mbc1 {
    extern_ram_enable: bool,
    rom_bank_num: u8,
    ram_bank_num: u8,
    rom_offset: usize,
    ram_offset: usize,
    ram_mode: bool, // mode 0 (false) or mode 1 (true)
    ram: Box<[u8]>,
}

impl Mbc1 {
    pub fn new(ram: Box<[u8]>) -> Self {
        Mbc1 {
            extern_ram_enable: false, // default disabled
            rom_bank_num: 0,
            ram_bank_num: 0,
            rom_offset: ROM_BANK_SIZE,
            ram_offset: 0,
            ram_mode: false, // default 0
            ram: ram,
        }
    }

    pub fn update_rom_offset(&mut self) {
        let bank_id = match rom_bank_num {
           0 => 1,
           _ => {
               match self.rom_bank_0 & 0xf0 {
                   0x20 | 0x40 | 0x60 => self.rom_bank_0 | 0x01,
                   _ => self.rom_bank_0,
               }
           }
        } as usize;

        self.rom_offset = bank_id * ROM_BANK_SIZE;
    }

    pub fn update_ram_offset(&mut self) {
        self.ram_offset = if self.ram_mode { // ram banking mode
            self.ram_bank_num as usize * 8 * 1024 // 8kb each ram bank, treating RAM as a giant array
        } else { // simple ROM banking mode
            0
        };
    }
}

impl Mbc for Mbc1 {
    pub fn read_rom(&self, rom: Box<[u8]>, addr: u16) -> u8 {
        match addr {
            0x0000..=0x3FFF => rom[addr as usize],
            0x4000..=0x7FFF => rom[addr as usize - ROM_BANK_SIZE + self.rom_offset],
            _ => panic!("Unsupported address"),
        }
    }

    pub fn write_rom(&mut self, addr: u16, content: u8) {
        match addr {
            0x0000..=0x1FFF => self.extern_ram_enable = content == 0x0A
            0x2000..=0x3FFF => self.rom_bank_num = content & 0x1F,
            0x4000..=0x5FFF => self.ram_bank_num = content & 0x03,
            0x6000..=0x7FFF => self.ram_mode = content == 0x01,
        }
        self.update_rom_offset();
        self.update_ram_offset();
    }

    pub fn read_ram(&self, addr: u16) -> u8 {
        self.ram[addr as usize - RAM_BASE_ADDR + self.ram_offset]
    }

    pub fn write_ram(&mut self, addr: u16, content: u8) {
        if self.extern_ram_enable {
            self.ram[addr as usize - RAM_BASE_ADDR + self.ram_offset] = val;
        }
    }

    pub fn copy_ram(&self) -> Option<Box<[u8]>> { // Pass RAM over to another hardware to use
        if self.ram.len() > 0 {
            Some(self.ram.clone())
        } else {
            None 
        }
    }
}
