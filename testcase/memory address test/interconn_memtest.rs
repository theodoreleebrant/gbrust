use std::fs::File;
use std::io::{self, BufRead};
use std::path::Path;

fn main() {
    // let path = Path::new("res.txt");
    // let display = path.display();

    // // Open a file in write-only mode, returns `io::Result<File>`
    // let mut file2 = match File::create(&path) {
    //     Err(why) => panic!("couldn't create {}: {}", display, why),
    //     Ok(file2) => file2,
    // };

    // // File hosts must exist in current path before this produces output
    // if let Ok(lines) = read_lines("./All0000FFFF") {
    //     // Consumes the iterator, returns an (Optional) String
    //     for line in lines {
    //         if let Ok(ip) = line {
    //             let res = u16::from_str_radix("ip", 16);
    //             // Write the `LOREM_IPSUM` string to `file`, returns `io::Result<()>`
    //             match res {
    //                 Ok(sth) => match file2.write_all(sth) {
    //                     Err(why) => panic!("couldn't write to {}: {}", display, why),
    //                     Ok(_) => println!("successfully wrote to {}", display),
    //                 },
    //                 _ => panic!("couldn't write to {}: res Err", display)
    //             }
                
    //         }
    //     }
    // }

    println!("start");

    let lines = read_lines("./hex0000toFFFF.txt");

    match lines {
        Err(why) => panic!("couldn't read {}", why),
        Ok(l) => {
            for line in l {
                if let Ok(ip) = line {
                    let res = u16::from_str_radix(&ip, 16);
                    match res {
                        Ok(thing) => {
                            let to_print = read(thing);
                            if to_print > 15 {
                                println!("{}", to_print);
                            }
                        }
                        Err(why) => panic!("no opcode? {}", why)
                    }
                    
                }
            }
        }
    }

    println!("done");
}

// extracted from interconn
pub fn read(addr: u16) -> u16 {
    match addr {
        // For more information: http://gameboy.mongenel.com/dmg/asmmemmap.html
        0x0000..=0x7fff => 0, // self.cart.read(addr), // Cartridge ROM
        0x8000..=0x9fff => 1, //self.ppu.read(addr), // Picture Processing Unit
        0xa000..=0xbfff => 2, // Cartridge swappable RAM
        0xc000..=0xdfff => 3, // Internal RAM
        // Might cause problems in GBC implementation but for DMG should be ok
        0xe000..=0xfdff => 4, 
        // Echo memory. Just copies over 0xc000..oxcfff

        // PPU addresses
        0xfe00..=0xfe9f // Object Attribute Memory, in PPU / Sprite RAM
            | 0xff40..=0xff45 // LCDC, LCDStat, SCY, SCX, LY, LYC
            | 0xff47..=0xff4b // BGP, Object Palette Data 0-1, WY, WX, 
            | 0xff68..=0xff69 // LCD Color Palette for CGB
            | 0xff4f => { // Destination Memory Bank
            5
        }

        // Unused memory
        0xfea0..=0xfeff => 6,

        // 0xFF00 - 0xFF7F: Hardware I/O Registers
        // Details http://marc.rawer.de/Gameboy/Docs/GBCPUman.pdf pg35
        // 0xFF00: Gamepad
        0xff00 => 7,

        // 0xFF01 - 0xFF02: serial I/O, used for linking up to other gameboy
        0xff01..=0xff02 => 8,
        
        // 0xFF04: DIV/Divider Register, incremented 16384 times a second.
        //         Needs to be implemented in timer.
        // 0xFF05: TIMA / Timer Counter - Interrupt when overflow.
        // 0xFF06: TMA / Timer Modulo - used when TIMA overflows.
        // 0xFF07: TAC / Timer Control. 3 bits: MSB is stop/start timer (0/1)
        //         2LSB is input clock speed (4096kHz, 262114 kHz, 65536kHz, 16384kHz)
        0xff04..=0xff07 => 9,

        // 0xFF08 - 0xFFOE unused

        // 0xFFOF - IF / Interrupt Flag
        0xff0f => 10,

        // 0xFFFF - IE / Interupt Enable
        0xffff => 11,

        // 0xFF10 - 0xFF3F: SPU (Not implemented yet)
        0xff10..=0xff3f => 12,

        // http://marc.rawer.de/Gameboy/Docs/GBCPUman.pdf pg 55
        0xff46 => 13,

        // Unusable memory, used as a speed switch (TODO)
        // 0xff4d => 0, 
        0xff80..=0xfffe => 14,
        
        //should be readable, not implemented?
        0xff03 | 0xff08 | 0xff09 => 15,

        _ => addr
        // panic!("Read: addr not in range: 0x{:x}", addr),
    }
}

// The output is wrapped in a Result to allow matching on errors
// Returns an Iterator to the Reader of the lines of the file.
fn read_lines<P>(filename: P) -> io::Result<io::Lines<io::BufReader<File>>>
where P: AsRef<Path>, {
    let file = File::open(filename)?;
    Ok(io::BufReader::new(file).lines())
}
