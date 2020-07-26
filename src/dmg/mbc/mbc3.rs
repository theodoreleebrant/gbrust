// MBC3
// 2MBytes (128 banks) ROM and/or 32KByte (4 banks) of RAM and TIMER
// RTC: Real Time Clock. requires 32.768 kHZ Quartz Oscillator and external battery
// Supports 127 ROM Banks and 4 RAM banks, supports access for banks 20,40,60
// Real Time Clock, how it works:
// RAM Bank: 08  09  0A  0B        0C(bit0)  0C(bit6) 0C(bit7)
//           Sec Min Hrs Days(lsb) Days(msb) halt     overflow flag, set when 9-bit day counter overflows

use super::Mbc;

const ROM_BANK_BASE: usize = 0x4000;
const RAM_BANK_BASE: usize = 0xA000;
const TICK_RATE: f64 = 32.768;

#[derive(Debug, Copy, Clone)]
pub struct Timer {
    sec: u8,
    min: u8,
    hrs: u8,
    days_lo: u8,
    days_hi: u8, // bit 0: msb of day counter, bit 6: halt, bit 7: day counter overflow
}

pub struct Mbc3 {
    timer_write_only: Timer,
    timer_read_only: Timer,
    timer_latch: bool, // When from false to true, clone timer_write_only to timer_read_only
    extern_ram_enable: bool,
    rom_bank_num: u8,
    ram_bank_num: u8,
    rom_offset: usize,
    ram_offset: usize,
    ram_mode: bool, // mode 0 (false) or mode 1 (true)
    ram: Box<[u8]>,
}

impl Mbc3 {
    pub fn new(ram: Option<Box<[u8]>>) -> Self {
        let ram = match ram {
            Some(boxed_ram) => boxed_ram,
            None => vec![0; 0].into_boxed_slice(),
        };

        let timer_std = Timer {
            sec: 0,
            min: 0,
            hrs: 0,
            days_lo: 0,
            days_hi: 0,
        };

        Mbc3 {
            timer_write_only: timer_std,
            timer_read_only: timer_std,
            timer_latch: false,
            extern_ram_enable: false, // default disabled
            rom_bank_num: 0,
            ram_bank_num: 0,
            rom_offset: ROM_BANK_BASE,
            ram_offset: 0,
            ram_mode: true, // default true for MBC3
            ram: ram,
        }
    }

    // Supports banks 20,40,60 here
    pub fn update_rom_offset(&mut self) {
        let bank_id = match self.rom_bank_num {
           0 => 1,
           _ => self.rom_bank_num & 0x7F, // msb is always reset
        } as usize;

        self.rom_offset = bank_id * 16 * 1024; // 16kb each bank
    }

    pub fn update_ram_offset(&mut self) {
        self.ram_offset = if self.ram_mode { // ram banking mode
            self.ram_bank_num as usize * 8 * 1024 // 8kb each ram bank, treating RAM as a giant array
        } else { // simple ROM banking mode
            0
        };
    }
}

impl Mbc for Mbc3 {
    fn read_rom(&self, rom: &Box<[u8]>, addr: u16) -> u8 {
        match addr {
            0x0000..=0x3FFF => rom[addr as usize],
            0x4000..=0x7FFF => rom[addr as usize - ROM_BANK_BASE + self.rom_offset],
            _ => panic!("Unsupported address 0x{:x}", addr),
        }
    }

    // Addr 0x0000 - 0x1FFF en/disables both RAM and timer
    fn write_rom(&mut self, addr: u16, content: u8) {
        match addr {
            0x0000..=0x1FFF => self.extern_ram_enable = content == 0x0A,
            0x2000..=0x3FFF => self.rom_bank_num = content & 0x7F,
            0x4000..=0x5FFF => self.ram_bank_num = content & 0x0F, // bank number will determine timer register to write to also
            0x6000..=0x7FFF => {
                if !self.timer_latch && content == 1 {
                    self.timer_read_only = self.timer_write_only.clone();
                }
                self.timer_latch = content == 1;
            },
            _ => panic!("Unsupported address 0x{:x}", addr),
        }
        self.update_rom_offset();
        self.update_ram_offset();
    }

    // different from mbc1: might access ram OR RTC Register depending on bank number / RTC
    // register selection
    fn read_ram(&self, addr: u16) -> u8 {
        match self.ram_bank_num {
            0..=3 => self.ram[addr as usize - RAM_BANK_BASE + self.ram_offset],
            0x08 => self.timer_read_only.sec,
            0x09 => self.timer_read_only.min,
            0x0A => self.timer_read_only.hrs,
            0x0B => self.timer_read_only.days_lo,
            0x0C => self.timer_read_only.days_hi,
            _ => panic!("unsupported RAM bank 0x{:x}", self.ram_bank_num),
        }
    }

    // RAM or timer register depending on bank number / RTC register selection.
    fn write_ram(&mut self, addr: u16, content: u8) {
        if self.extern_ram_enable {
            match self.ram_bank_num {
                0..=3 => self.ram[addr as usize - RAM_BANK_BASE + self.ram_offset] = content,
                0x08 => self.timer_write_only.sec = content & 0x3F, // <= 60s
                0x09 => self.timer_write_only.min = content & 0x3F, // <= 60m
                0x0A => self.timer_write_only.hrs = content & 0x1F, // <= 24
                0x0B => self.timer_write_only.days_lo = content,
                0x0C => self.timer_write_only.days_hi = content & 0b1100_0001, // extracts day counter, carry bit, halt flag
                _ => panic!("Unsupported ram bank number 0x{:x}", self.ram_bank_num),
            }
        }
    }

    fn copy_ram(&self) -> Option<Box<[u8]>> { // Pass RAM over to another hardware to use
        if self.ram.len() > 0 {
            Some(self.ram.clone())
        } else {
            None 
        }
    }
}
