use super::interconnect::Interconnect;
use super::console::VideoSink;
use std::{thread, time};

// Flags
const ZF: u8 = 0x80; // 0b10000000
const NF: u8 = 0x40; // 0b01000000
const HF: u8 = 0x20; // 0b00100000
const CF: u8 = 0x10; // 0b00010000

// 8-bit Register IDs
const A_ID: u8 = 0b111;
const B_ID: u8 = 0b000;
const C_ID: u8 = 0b001;
const D_ID: u8 = 0b010;
const E_ID: u8 = 0b011;
const H_ID: u8 = 0b100;
const L_ID: u8 = 0b101;

// 16-bit Register IDs
const BC_ID: u8 = 0b00;
const DE_ID: u8 = 0b01;
const HL_ID: u8 = 0b10;
const SP_ID: u8 = 0b11;

// Places to jump to during interrupts

/// GB has 8 8-bit registers (including special flag register).
/// 3 16-bit pair registers, which is a combination from pairing 2 8-bit registers together.
/// 2 special registers: SP and PC.
/// 3 Interrupt Registers: IME (master), IE: Interrupt Enable -> Enables interrupts, IF: Interrupt
///   Flag -> Requests Interrupts
pub struct Registers {
	a: u8,      // Accumulator register, done
	b: u8,      // done
	c: u8,      // done
	d: u8,
	e: u8,
	h: u8,
	l: u8,

	// 16-bit pair registers
	bc: u16,    // done
	de: u16,
	hl: u16,    // done

	// Special registers
	f: u8,      // Special flag register, done
	sp: u16,    // Stack pointer. SP will start at 65536. Done
	pc: u16,

	// Registers for interrupt.
	// IME: 0 -> Disable all Interrupts, 1 -> Enable all Interrupts enabled in IE
	ime: bool,    // Enable / Disable all interrupts
}

impl Registers {
    pub fn new() -> Self {
        // Values taken from gbc_rs repo adn matched with Pan Docs
        // This is after start-up sequence
        Registers {
            a: 0x01,
            b: 0x00,
            c: 0x13,
            d: 0x00,
            e: 0xD8,
            h: 0x01,
            l: 0x4D,

            bc: 0x0013,
            de: 0x00D8,
            hl: 0x014D,

            f: 0xB0,
            sp: 0xFFFE,
            pc: 0x0100,

            ime: true,
        }
    }
}

pub struct Cpu {
	reg: Registers,     // Set of registers

	//mem: [u8; 65536],   // 64KB memory
	stack: [u8; 65536], // Stack for PC

	halt_mode: bool,    // true -> enter halt mode
	stop_mode: bool,    // true -> enter stop mode

	pub interconnect: Interconnect, // in charge of everything else. Needs to be pub to be accessed by console
}

pub enum ProgramCounter { // Each returned ProgramCounter will return number of bytes of instruction, then number of cycles 
    Next(i16, u32),
    Jump(u16, u32),
}

impl Cpu {
    pub fn new(interconnect: Interconnect) -> Self {
        Cpu {
            reg: Registers::new(),
            //mem: [0; 65536],
            stack: [0; 065536],
            interconnect: interconnect,

            halt_mode: false,
            stop_mode: false,
        }
    }

    pub fn step(&mut self, video_sink: &mut dyn VideoSink) -> u32 {
        // elapsed_cycles calculates how many cycles are spent carrying out the instruction and
        // corresponding interrupt (if produced) = time to execute + time to handle interrupt
        println!("current pc: {:?}", self.reg.pc);
        //thread::sleep(time::Duration::from_millis(1));
        let elapsed_cycles = {
            self.execute_opcode() + self.handle_interrupt() 
        };
        self.interconnect.cycle_flush(elapsed_cycles, video_sink);
        
        elapsed_cycles        
    }

    // Implement how to handle interrupts, depending on registers IME, IF, IE
    pub fn handle_interrupt(&mut self) -> u32 {
        // int_flags(IF) indicate the interrupt signals requested.
        // int_enable(IE) indicate which I/O device can send interrupt.
        // all_ints: I/O devices with enabled interrupt AND sending signal.
        let all_ints = self.interconnect.int_flags & self.interconnect.int_enable;
        // if in halt mode: Any interrupt will cause program to continue. If no interrupt,no change
        if self.halt_mode {
            self.halt_mode = all_ints == 0;
        }

        // Either: ime = false which means ALL interrupts are disabled OR none of I/O devices
        // requested / are allowed to request interrupt 
        if !self.reg.ime || all_ints == 0 {
            return 0;
        }
        
        // all_ints.trailing_zeros():
        // identify the first interrupt bit requested. Choose hardware to handle accordingly.
        let interrupt_bit = all_ints.trailing_zeros();
        let int_hardware = match interrupt_bit {
            0 => 0x40,  // VBlank
            1 => 0x48,  // LCDCStat
            2 => 0x50,  // Timer Overflow
            3 => 0x58,  // Serial Transfer Complete
            4 => 0x60,  // P10-P13 Input Signal
            _ => panic!("Invalid interrupt! {:x}", interrupt_bit),
        };
        
        // After handling request, reset correspoding bit
        self.interconnect.int_flags &= 0xff << (interrupt_bit + 1);
        // reset ime
        self.reg.ime = false;

        let pc = self.reg.pc;
        self.push_u16(pc);
        self.reg.pc = int_hardware as u16;

        20 // y tho, in PanDoc says 5 machine cycles. TODO: confirm this
    }

    pub fn execute_opcode(&mut self) -> u32 {
        let opcode: u8 = self.interconnect.read(self.reg.pc);
        
        let is_aa0: bool = (opcode & 0b0000_1000) == 0; 
        let is_0bb: bool = (opcode & 0b0010_0000) == 0;  
        
        let parts = (
            opcode >> 6, // bit 7 6
            (opcode & 0b0011_1000) >> 3, // bit 543
            (opcode & 0b0000_0111), // bit 210,
            is_aa0,
            is_0bb,
        );

        println!("opcode: 0x{:x}", opcode);

        let pc_change = match parts {
            // opcodes starting with 00
            (0b00, 0b110, 0b110, _, _) => self.ld_addr_hl_n(),
            (0b00, 0b001, 0b010, _, _) => self.ld_a_addr_bc(),
            (0b00, 0b011, 0b010, _, _) => self.ld_a_addr_de(),
            (0b00, 0b000, 0b010, _, _) => self.ld_addr_bc_a(),
            (0b00, 0b010, 0b010, _, _) => self.ld_addr_de_a(),
            (0b00, 0b111, 0b010, _, _) => self.ld_a_addr_hl_dec(),
            (0b00, 0b110, 0b010, _, _) => self.ld_addr_hl_a_dec(),
            (0b00, 0b101, 0b010, _, _) => self.ld_a_addr_hl_inc(),
            (0b00, 0b100, 0b010, _, _) => self.ld_addr_hl_a_inc(),
            (0b00, 0b001, 0b000, _, _) => self.ld_addr_nn_sp(),
            (0b00, 0b011, 0b000, _, _) => self.jr_e(),
            (0b00, 0b111, 0b111, _, _) => self.ccf(),
            (0b00, 0b110, 0b111, _, _) => self.scf(),
            (0b00, 0b000, 0b000, _, _) => self.nop(),
            (0b00, 0b100, 0b111, _, _) => self.daa(),
            (0b00, 0b101, 0b111, _, _) => self.cpl(),
            (0b00, 0b110, 0b100, _, _) => self.inc_hl(),
            (0b00, 0b110, 0b101, _, _) => self.dec_hl(),
            (0b00, 0b000, 0b111, _, _) => self.rlca(),
            (0b00, 0b010, 0b111, _, _) => self.rla(),
            (0b00, 0b001, 0b111, _, _) => self.rrca(),
            (0b00, 0b011, 0b111, _, _) => self.rra(),
            
            (0b00, _, 0b011, true, _) => self.inc_ss(), // ss0
            (0b00, _, 0b011, false, _) => self.dec_ss(), // ss1
            (0b00, _, 0b001, false, _) => self.add_hlss(), // ss1
            (0b00, _, 0b001, true, _) => self.ld_rr_nn(), // rr0
            (0b00, _, 0b000, _, false) => self.jr_cc_e(),  // 1cc
            (0b00, _, 0b110, _, _) => self.ld_r_n(),   
            (0b00, _, 0b101, _, _) => self.dec_r(),   
            (0b00, _, 0b100, _, _) => self.inc_r(),

            // opcodes starting with 01
            (0b01, 0b110, _, _, _) => self.ld_addr_hl_r(),
            (0b01, _, 0b110, _, _) => self.ld_r_addr_hl(),
            (0b01, _, _, _, _) => self.ld_rx_ry(),

            // opcodes starting with 10:
            (0b10, 0b000, 0b110, _, _) => self.add_ahl(),
            (0b10, 0b001, 0b110, _, _) => self.adc_ahl(),
            (0b10, 0b010, 0b110, _, _) => self.sub_hl(),
            (0b10, 0b011, 0b110, _, _) => self.sbc_ahl(),
            (0b10, 0b100, 0b110, _, _) => self.and_hl(),
            (0b10, 0b110, 0b110, _, _) => self.or_hl(),
            (0b10, 0b101, 0b110, _, _) => self.xor_hl(),
            (0b10, 0b111, 0b110, _, _) => self.cp_hl(),
            (0b10, 0b000, _, _, _) => self.add_ar(),
            (0b10, 0b001, _, _, _) => self.adc_ar(),
            (0b10, 0b010, _, _, _) => self.sub_r(),
            (0b10, 0b011, _, _, _) => self.sbc_ar(),
            (0b10, 0b100, _, _, _) => self.and_r(),
            (0b10, 0b110, _, _, _) => self.or_r(),
            (0b10, 0b101, _, _, _) => self.xor_r(),
            (0b10, 0b111, _, _, _) => self.cp_r(),
            
            // opcodes starting with 11
            (0b11, 0b111, 0b010, _, _) => self.ld_a_addr_nn(),
            (0b11, 0b101, 0b010, _, _) => self.ld_addr_nn_a(),
            (0b11, 0b110, 0b010, _, _) => self.ldh_a_addr_offset_c(),
            (0b11, 0b100, 0b010, _, _) => self.ldh_addr_offset_c_a(),
            (0b11, 0b110, 0b000, _, _) => self.ldh_a_addr_offset_n(),
            (0b11, 0b100, 0b000, _, _) => self.ldh_addr_offset_n_a(),
            (0b11, 0b111, 0b001, _, _) => self.ld_sp_hl(),
            (0b11, 0b000, 0b110, _, _) => self.add_an(), // arithmetic
            (0b11, 0b001, 0b110, _, _) => self.adc_an(),
            (0b11, 0b010, 0b110, _, _) => self.sub_n(),
            (0b11, 0b011, 0b110, _, _) => self.sbc_an(),
            (0b11, 0b100, 0b110, _, _) => self.and_n(),
            (0b11, 0b110, 0b110, _, _) => self.or_n(),
            (0b11, 0b101, 0b110, _, _) => self.xor_n(),
            (0b11, 0b111, 0b110, _, _) => self.cp_n(),
            (0b11, 0b101, 0b000, _, _) => self.add_spe(),
            (0b11, 0b000, 0b011, _, _) => self.jp_nn(),
            (0b11, 0b101, 0b001, _, _) => self.jp_hl(),
            (0b11, 0b001, 0b101, _, _) => self.call_nn(),
            (0b11, 0b001, 0b001, _, _) => self.ret(),
            (0b11, 0b011, 0b001, _, _) => self.reti(),
            (0b11, 0b110, 0b011, _, _) => self.di(),
            (0b11, 0b111, 0b011, _, _) => self.ei(),
            (0b11, 0b001, 0b011, _, _) => self.execute_bc(self.reg.pc),
            
            (0b11, _, 0b101, true, _) => self.push_rr(), // xx0
            (0b11, _, 0b001, true, _) => self.pop_rr(), // xx0
            (0b11, _, 0b010, _, true) => self.jp_cc_nn(), // 0cc
            (0b11, _, 0b100, _, true) => self.call_cc_nn(),// 0cc
            (0b11, _, 0b000, _, true) => self.ret_cc(),   // 0cc
            (0b11, _, 0b111, _, _) => self.rst_n(), 
            
            // The rest: panik
            _ => panic!("No such opcode"),
        };
        
        let cycles_taken: u32 = match pc_change {
            ProgramCounter::Next(bytes, cycles) => {
                let offset: u16;
                if bytes < 0 {
                    offset = (bytes * (-1)) as u16;
                    self.reg.pc -= offset;
                } else {
                    offset = bytes as u16;
                    self.reg.pc = self.reg.pc.wrapping_add(offset);
                }
                //println!("Next pc is: {:x}", self.reg.pc);
                cycles
            },
            ProgramCounter::Jump(addr, cycles) => {
                self.reg.pc = addr;
                cycles
            },
        };

        cycles_taken

    }

    pub fn execute_bc(&mut self, pc_current: u16) -> ProgramCounter {
        let suffix = self.interconnect.read(pc_current + 1);
        let parts = (
            suffix >> 6, //  bit 76
            (suffix & 0b0011_1000) >> 3, // bit 543
            (suffix & 0b0000_0111), // bit 210
        );
        
        let pc_change = match parts {
            // starting with 00
            (0b00, 0b000, _) => self.rlc(),
            (0b00, 0b010, _) => self.rl(),
            (0b00, 0b001, _) => self.rrc(),
            (0b00, 0b011, _) => self.rr(),
            (0b00, 0b100, _) => self.sla(),
            (0b00, 0b101, _) => self.sra(),
            (0b00, 0b111, _) => self.srl(),
            (0b00, 0b110, _) => self.swap(),

            // starting with 01
            (0b01, _, 0b110) => self.bit_b_hl(),
            (0b01, _, _) => self.bit_b_r(),

            // starting with 10
            (0b10, _, 0b110) => self.res_b_hl(),
            (0b10, _, _) => self.res_b_r(),

            // starting with 11
            (0b11, _, 0b110) => self.set_b_hl(),
            (0b11, _, _) => self.set_b_r(),
            
            // panik if no match
            _ => panic!("No such opcode in BC"),
        };

        pc_change
    }

    // Some reusable code (for opcodes)
    
    /// write_to_r8: write content to appropriate 8-bit register based on register ID.
    /// @param r8_id: ID of register
    /// @param content: content to write to register
    /// # Examples
    ///
    /// ```
    /// assert_eq!(1, 0);
    /// ```
    pub fn write_to_r8(&mut self, r8_id: u8, content: u8) {
        match r8_id {
            A_ID => self.reg.a = content,
            B_ID => {
                self.reg.b = content;
                self.reg.bc = (self.reg.bc & 0x00ff) | ((content as u16) << 8);
            },
            C_ID => {
                self.reg.c = content;
                self.reg.bc = (self.reg.bc & 0xff00) | (content as u16);
            },
            D_ID => {
                self.reg.d = content;
                self.reg.de = (self.reg.de & 0x00ff) | ((content as u16) << 8);
            },
            E_ID => {
                self.reg.e = content;
                self.reg.de = (self.reg.de & 0xff00) | (content as u16);
            },
            H_ID => {
                self.reg.h = content;
                self.reg.hl = (self.reg.hl & 0x00ff) | ((content as u16) << 8);
            },
            L_ID => {
                self.reg.l = content;
                self.reg.hl = (self.reg.hl & 0xff00) | (content as u16);
            },
            _ => panic!("Invalid register!"),
        }
    }

    /// read_from_r8: Read data from the appropriate register.
    /// @param r8_id: ID of 8-bit register
    /// @return Option<u8>. returns None if r8_id is invalid or register is empty.
    pub fn read_from_r8(&mut self, r8_id: u8) -> Option<u8> {
        let result: u8;
        
        match r8_id {
            A_ID => result = self.reg.a,
            B_ID => result = self.reg.b,
            C_ID => result = self.reg.c,
            D_ID => result = self.reg.d,
            E_ID => result = self.reg.e,
            H_ID => result = self.reg.h,
            L_ID => result = self.reg.l,
            _ => return None,
        }

        Some(result)
    }
    
    /// load_mem_to_r8: Loads content from memory specified by addr into register r8_id.
    /// @param r8_id: ID of some 8-bit register
    /// @param addr: 16-bit address for memory
    /// @return boolean whether ID is valid
    pub fn load_mem_to_r8(&mut self, r8_id: u8, addr: u16) {
        let res = self.interconnect.read(addr);
        self.write_to_r8(r8_id, res);
    }

    /// save_r8_to_mem: Saves content from register r8_id into memory specified by addr.
    /// @param r8_id: ID of some 8-bit register with content
    /// @param addr: 16-bit address for memory to be saved to
    pub fn save_r8_to_mem(&mut self, r8_id: u8, addr: u16) {
        match self.read_from_r8(r8_id) {
            Some(content) => self.interconnect.write(addr, content),
            None => (),
        }
    }

    /// get_n: gets 8-bit immediate n right after opcode
    pub fn get_n(&mut self) -> u8 {
        self.interconnect.read(self.reg.pc + 1) 
    }

    /// get_r8_to: gets 3-bit register ID from opcode. Register ID takes bit 3, 4, 5 for register
    /// written to.
    pub fn get_r8_to(&mut self) -> u8 {
        ((self.interconnect.read(self.reg.pc) & 0b00111000) >> 3) as u8
    }
    
    /// get_r8_from: gets 3-bit register ID from opcode. Register ID takes bit 0,1,2 for register
    /// written to.
    pub fn get_r8_from(&mut self) -> u8 {
        (self.interconnect.read(self.reg.pc) & 0b00000111) as u8
    }

    /// write_to_r16: Write content onto a 16-byte register.
    /// @param r16_id: ID of 16-byte reg
    /// @param content: content to be written
    /// @return bool value if ID was valid.
    pub fn write_to_r16(&mut self, r16_id: u8, content: u16) {
        let msb = (content >> 8) as u8;
        let lsb = (content & 0x00FF) as u8;

        match r16_id {
            BC_ID => {
                self.reg.bc = content;
                self.reg.b = msb;
                self.reg.c = lsb;
            },
            DE_ID => {
                self.reg.de = content;
                self.reg.d = msb;
                self.reg.e = lsb;
            },
            HL_ID => {
                self.reg.hl = content;
                self.reg.h = msb;
                self.reg.l = lsb;
            },
            SP_ID => self.reg.sp = content,
            _ => panic!("Invalid register"),
        }
    }

    /// read_from_r16: reads content of a 16-bit register.
    /// @param r16_id: ID of a 16-byte register.
    /// @return Some<u16> if ID is valid, None if not valid.
    pub fn read_from_r16(&mut self, r16_id: u8) -> Option<u16> {
        let result: u16;

        match r16_id {
            BC_ID => result = self.reg.bc,
            DE_ID => result = self.reg.de,
            HL_ID => result = self.reg.hl,
            SP_ID => result = self.reg.sp,
            _ => return None,
        }

        Some(result)
    }

    /// save_r16_to_mem: Saves lower-byte of 16-bit register to addr, and higher-byte to addr + 1.
    /// @param r16_id: ID of 16-byte register.
    /// @param addr: address to write content to.
    pub fn save_r16_to_mem(&mut self, r16_id: u8, addr: u16) {
        match self.read_from_r16(r16_id) {
            Some(value) => {
                self.interconnect.write(addr, (value & 0x00FF) as u8);
                self.interconnect.write(addr + 1, (value >> 8) as u8);
            },
            None => (),
        }
    }

    /// get_nn: gets 16-bit immediate nn right after opcode
    pub fn get_nn(&mut self) -> u16 {
        let nn_low = self.interconnect.read(self.reg.pc + 1);
        let nn_high = self.interconnect.read(self.reg.pc + 2);
        let nn = ((nn_high as u16) << 8) | (nn_low as u16); 

        nn
    }

    pub fn get_r16(&mut self) -> u8 {
        let res = ((self.interconnect.read(self.reg.pc) & 0b00110000) >> 4) as u8;
        println!("get_r16: {:?}", res);
        res
    }

    // Reusable code for 8-bit Rotate, Shift instructions
    
    pub fn set_flag(&mut self, flag: u8) {
        self.reg.f = self.reg.f | flag;
    }

    pub fn reset_flag(&mut self, flag: u8) {
        match flag {
            ZF => self.reg.f &= 0b01111111,
            NF => self.reg.f &= 0b10111111,
            HF => self.reg.f &= 0b11011111,
            CF => self.reg.f &= 0b11101111,
            _ => (),
        }
    }

    /// rotate_r8: Rotate function for 8-bit registers. Toggle between lpeft or right using bool
    /// is_rotate_left.
    /// There are 2 types of rotate operations: Has carry or no carry.
    /// If operation has carry: bit A7 is copied to flag CY AND bit 0 of A.
    /// If operation has no carry: bit 0 of A is replaced by CY, and then bit A7 is copied to CY 
    /// after rotation, write data back to register.
    
    pub fn rotate_r8(&mut self, r8_id: u8, is_rotate_left: bool, has_carry: bool) {
        let mut data: u8;
        let c: bool;

        match self.read_from_r8(r8_id) {
            Some(value) => data = value,
            None => return (),
        }

        let bit_cf: u8 = (self.reg.f & CF) >> 4;

        if is_rotate_left {
            let bit_a7: u8 = (data & 0x80) >> 7;
            data = (data << 1) as u8; // a7 diasppeared, a0 = 0
            
            // setting bit a7
            if has_carry {
                data = data | bit_a7;
            } else {
                data = data | bit_cf;
            }
            
            c = bit_a7 > 0;
        } else {
            let bit_a0: u8 = data & 0x01;
            data = (data >> 1) as u8; // a0 diasppeared, a7 = 0

            // setting bit a0
            if has_carry {
                data = data | (bit_a0 << 7);
            } else {
                data = data | (bit_cf << 7);
            }

            c = bit_a0 > 0;
        }
        
        // write back to register
        self.write_to_r8(r8_id, data); 
        
        // set flags
        self.set_hcnz(false, c, false, data == 0);
    }

    /// rotate_mem: Rotate left function for values in memory. Can toggle with is_left_rotate bool.
    /// Implementing 2 types of rotate operations as well: has carry and no carry, similar to
    /// register rotation.
    
    pub fn rotate_mem(&mut self, addr: u16, is_left_rotate: bool, has_carry: bool) {
        let mut data = self.interconnect.read(addr);
        let c: bool;
        let bit_cf = (self.reg.f & CF) >> 4;
    
        if is_left_rotate {
            let bit_a7 = (data & 0x80) >> 7;
            data = data << 1; // bit a7 gone, bit a0 = 0

            // setting bit a7
            if has_carry {
                data = data | bit_a7;
            } else {
                data = data | bit_cf;
            }

            c = bit_a7 > 0;
        } else {
            let bit_a0: u8 = data & 0x01;
            data = (data >> 1) as u8; // a0 diasppeared, a7 = 0

            // setting bit a0
            if has_carry {
                data = data | (bit_a0 << 7);
            } else {
                data = data | (bit_cf << 7);
            }

            c = bit_a0 > 0;
        }

        self.interconnect.write(addr, data); // write back to memory

        // setting cf to bit_a7
        self.set_hcnz(false, c, false, data == 0);
    }

    pub fn write_a(&mut self, to_write: u8) {
    	self.write_to_r8(A_ID, to_write);
    }

    pub fn set_hcnz(&mut self, h: bool, c: bool, n: bool, z: bool) {
	    if h {self.set_flag(HF)} else {self.reset_flag(HF)};
	    if c {self.set_flag(CF)} else {self.reset_flag(CF)};
	    if n {self.set_flag(NF)} else {self.reset_flag(NF)};
	    if z {self.set_flag(ZF)} else {self.reset_flag(ZF)};
	}

	pub fn set_hnz(&mut self, h: bool, n: bool, z: bool) {
	    if h {self.set_flag(HF)} else {self.reset_flag(HF)};
	    if n {self.set_flag(NF)} else {self.reset_flag(NF)};
	    if z {self.set_flag(ZF)} else {self.reset_flag(ZF)};
	}

	pub fn set_hcn(&mut self, h: bool, c: bool, n: bool) {
	    if h {self.set_flag(HF)} else {self.reset_flag(HF)};
	    if c {self.set_flag(CF)} else {self.reset_flag(CF)};
	    if n {self.set_flag(NF)} else {self.reset_flag(NF)};
	}
    
    /// check_cc extracts condition cc from opcode, and check whether condition is true.
    /// cc is a 2-bit number, at bit 3 and 4 of opcode, representing:
    /// 00 -> Z == 0; 01 -> Z == 1; 10 -> C == 0; 11 -> C == 1
    pub fn check_cc(&mut self) -> bool {
        // extract cc from opcode
        let opcode = self.interconnect.read(self.reg.pc);
        let cc: u8 = (opcode & 0b00011000) >> 3;
        let result: bool;
        
        // match cc with respective outcomes
        match cc {
            0b00 => result = self.reg.f & ZF == 0,
            0b01 => result = self.reg.f & ZF != 0,
            0b10 => result = self.reg.f & CF == 0,
            0b11 => result = self.reg.f & CF != 0,
            _ => panic!("Invalid cc: 0b{:b}", cc),
        }

        result
    }
   
    /// push_u16: push a u16 value onto the stack.
    /// Most significant byte (MSB) goes to SP - 1
    /// Least significant byte (LSB)  goes to SP - 2
    pub fn push_u16(&mut self, val: u16) {
        self.stack[(self.reg.sp - 1) as usize] = (val >> 8) as u8; // most sig. byte
        self.stack[(self.reg.sp - 2) as usize] = (val & 0x00FF) as u8; // least sig. byte.

        self.reg.sp -= 2;
    }

    /// pop_u16: pop a u16 value off the stack and return it.
    /// LSB is at SP. MSB is at SP + 1. After that, increment SP by 2
    pub fn pop_u16(&mut self) -> u16 {
        let lsb = self.stack[self.reg.sp as usize] as u16;
        let msb = self.stack[(self.reg.sp + 1) as usize] as u16;

        self.reg.sp += 2;

        (msb << 8) | lsb
    }

    // Opcodes goes here!!
    
    // 8-bit load instructions
    
    /// ld_rx_ry: load contents of ry to rx. 1-byte instruction
    /// @param rx, ry: ID for register rx and ry (8-bit)
    // Cycles: 1
    pub fn ld_rx_ry(&mut self) -> ProgramCounter {
        let rx = self.get_r8_to();
        let ry = self.get_r8_from();

        match self.read_from_r8(ry) {
            Some(value) => self.write_to_r8(rx, value),
            None => {},
        }

        ProgramCounter::Next(1, 1)
    }

    /// ld_r_n: Load 8-bit data n into register r. 2-byte instruction
    /// @param: r: register ID; n: intermediate
    // Cycles: 2
    pub fn ld_r_n(&mut self) -> ProgramCounter {
        let r = self.get_r8_to();
        let n = self.get_n();

        println!("(ld_r_n) r:{:?}, n:{:?}", r, n);

        self.write_to_r8(r, n);

        ProgramCounter::Next(2, 2)
    }

    /// ld_r_addr_hl: loads contents of memory specified at (HL) to register r. 1-byte instruction
    /// @param r: 8-bit register ID
    // Cycles: 2
    pub fn ld_r_addr_hl(&mut self) -> ProgramCounter {
        let r = self.get_r8_to();

        self.load_mem_to_r8(r, self.reg.hl);

        ProgramCounter::Next(1, 2)
    }

    /// ld_addr_hl_r: stores contents of register r into memory specified by register pair HL.
    /// 1-byte instruction.
    /// @param: r: ID of 8-bit register
    // Cycles: 2
    pub fn ld_addr_hl_r(&mut self) -> ProgramCounter {
        let r = self.get_r8_from();
    
        self.save_r8_to_mem(r, self.reg.hl);
        
        ProgramCounter::Next(1, 2)
    }

    /// ld_addr_hl_n: stores 8-bit immediate data in memory specified by register pair HL.
    /// 2-byte instruction.
    /// @param n: 8-bit immediate.
    // Cycles: 3
    pub fn ld_addr_hl_n(&mut self) -> ProgramCounter {
        let n = self.get_n();

        self.interconnect.write(self.reg.hl, n);

        ProgramCounter::Next(2, 3)
    }

    /// ld_a_addr_bc: Load contents of memory specified by BC into A.
    /// 1-byte instruction
    // 
    pub fn ld_a_addr_bc(&mut self) -> ProgramCounter {
        self.load_mem_to_r8(A_ID, self.reg.bc);

        ProgramCounter::Next(1, 2)
    }

    /// ld_a_addr_de: Load contents of memory specified by DE into A.
    /// 1-byte instruction
    pub fn ld_a_addr_de(&mut self) -> ProgramCounter {
        self.load_mem_to_r8(A_ID, self.reg.de);

        ProgramCounter::Next(1, 2)
    }

    /// ldh_a_addr_offset_c: Load contents of memory specified by C + 0xFF00 into A.
    /// 1-byte instruction
    pub fn ldh_a_addr_offset_c(&mut self) -> ProgramCounter {
        self.load_mem_to_r8(A_ID, 0xFF00 + (self.reg.c as u16));

        ProgramCounter::Next(1, 2)
    }

    /// ldh_addr_offset_c_a: Load contents of A into memory specified by 0xFF00 + C.
    /// 1-byte instruction
    pub fn ldh_addr_offset_c_a(&mut self) -> ProgramCounter {
        self.save_r8_to_mem(A_ID, 0xFF00 + (self.reg.c as u16));

        ProgramCounter::Next(1, 2)
    }

    /// ldh_a_addr_offset_n: Load contents of memory specified by nn + 0xFF00 into A.
    /// 1-byte instruction
    pub fn ldh_a_addr_offset_n(&mut self) -> ProgramCounter {
        let n = self.get_n();

        self.load_mem_to_r8(A_ID, 0xFF00 + (n as u16));
        
        ProgramCounter::Next(2, 3)
    }
    
    /// ldh_addr_offset_n_a: Load contents of A into memory specified by 0xFF00 + n.
    /// 1-byte instruction
    pub fn ldh_addr_offset_n_a(&mut self) -> ProgramCounter {
        let n = self.get_n();

        self.save_r8_to_mem(A_ID, 0xFF00 + (n as u16));

        ProgramCounter::Next(2, 3)
    }

    /// ld_a_addr_nn: Load content at memory specified by address nn into register A.
    /// 3-byte instruction.
    /// @param nn: 16-bit address
    pub fn ld_a_addr_nn(&mut self) -> ProgramCounter {
        let nn = self.get_nn();

        self.load_mem_to_r8(A_ID, nn);

        ProgramCounter::Next(3, 4)
    }

    /// ld_addr_nn_a: Save content of register A into memory specified by address nn.
    /// 3-byte instruction.
    /// @param nn: 16-bit address.
    pub fn ld_addr_nn_a(&mut self) -> ProgramCounter {
        let nn = self.get_nn();

        self.save_r8_to_mem(A_ID, nn);
    
        ProgramCounter::Next(3, 4)
    } 

    /// ld_a_addr_hl_inc: Load content of memory specified by HL into register A, then increment
    /// content in HL.
    /// 1-byte instruction.
    pub fn ld_a_addr_hl_inc(&mut self) -> ProgramCounter {
        self.load_mem_to_r8(A_ID, self.reg.hl);
        self.reg.hl += 1;

        ProgramCounter::Next(1, 2)
    }

    /// ld_a_addr_hl_dec: Load content of memory specified by HL into register A, then deccrement
    /// content in HL.
    /// 1-byte instruction.
    pub fn ld_a_addr_hl_dec(&mut self) -> ProgramCounter {
        self.load_mem_to_r8(A_ID, self.reg.hl);
        self.reg.hl -= 1;

        ProgramCounter::Next(1, 2)
    }

    /// ld_addr_bc_a: Save content of register A to memory specified by BC.
    /// 1-byte instruction
    pub fn ld_addr_bc_a(&mut self) -> ProgramCounter {
        self.save_r8_to_mem(A_ID, self.reg.bc);

        ProgramCounter::Next(1, 2)
    }

    /// ld_addr_de_a: Save content of register A to memory specified by DE.
    /// 1-byte instruction
    pub fn ld_addr_de_a(&mut self) -> ProgramCounter {
        self.save_r8_to_mem(A_ID, self.reg.de);

        ProgramCounter::Next(1, 2)
    }

    /// ld_addr_hl_a_inc: Load content of register A into memory specified by HL, then increment
    /// content in HL.
    /// 1-byte instruction.
    pub fn ld_addr_hl_a_inc(&mut self) -> ProgramCounter {
        self.save_r8_to_mem(A_ID, self.reg.hl);
        self.reg.hl = self.reg.hl.wrapping_add(1);

        ProgramCounter::Next(1, 2)
    }

    /// ld_addr_hl_a_dec: Load content of register A into memory specified by HL, then deccrement
    /// content in HL.
    /// 1-byte instruction.
    pub fn ld_addr_hl_a_dec(&mut self) -> ProgramCounter {
        self.save_r8_to_mem(A_ID, self.reg.hl);
        self.reg.hl = self.reg.hl.wrapping_sub(1);

        ProgramCounter::Next(1, 2)
    }

    // 16-bit load instructions
    
    /// ld_rr_nn: load 16-bit immediate nn to 16-bit register rr.
    /// 3-byte instruction
    /// @param rr: ID of 16-bit instruction
    pub fn ld_rr_nn(&mut self) -> ProgramCounter {
        let rr = self.get_r16();
        let nn = self.get_nn();
        
        self.write_to_r16(rr, nn);

        ProgramCounter::Next(3, 3)
    }

    /// ld_addr_nn_sp: load lower-byte of SP to (nn), load higher-byte of SP to (nn+1)
    /// 3-byte instruction
    pub fn ld_addr_nn_sp(&mut self) -> ProgramCounter {
        let nn = self.get_nn();

        self.save_r16_to_mem(SP_ID, nn);

        ProgramCounter::Next(3, 5)
    }

    /// ld_sp_hl: load data from HL register to SP register.
    /// 1-byte instruction
    pub fn ld_sp_hl(&mut self) -> ProgramCounter {
        self.reg.sp = self.reg.hl;

        ProgramCounter::Next(1, 2)
    }

    /// push_rr: push data from register rr to stack memory
    /// 1-byte instruction
    pub fn push_rr(&mut self) -> ProgramCounter {
        let rr = self.get_r16();
        let val = self.read_from_r16(rr).unwrap();

        self.push_u16(val);

        ProgramCounter::Next(1, 4)
    }

    /// pop_rr: pop data from stack to the 16-bit register rr.
    /// 1-byte instruction
    pub fn pop_rr(&mut self) -> ProgramCounter {
        let rr = self.get_r16();
        let val_pop = self.pop_u16();
        
        self.write_to_r16(rr, val_pop);

        ProgramCounter::Next(1, 3)
    }


    // 8 Bit Arithmetic Operation Instruction
    // ADD A,r: Add the value in register r to A, set it to A. 
    // Cycles: 1
    pub fn add_ar(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx).unwrap();

	    // processing
	    let res: u16 = (a as u16) + (r as u16);

	    // flags and writing
	    let h: bool = ((a & 0x0F) + (r & 0x0F)) > 0x0F;
	    let c: bool = res > 0xFF;
	    let n: bool = false;
	    let to_write: u8 = (res & 0xFF) as u8;
	    let z: bool = to_write == 0;

	    self.write_a(to_write);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 1)
	}

	// ADD A, n: add immediate operand n to register A.
	// Cycles: 2
	pub fn add_an(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let r: u8 = self.get_n();

	    // processing
	    let res: u16 = (a as u16) + (r as u16);

	    // flags and writing
	    let h: bool = ((a & 0x0F) + (r & 0x0F)) > 0x0F;
	    let c: bool = res > 0xFF;
	    let n: bool = false;
	    let to_write: u8 = (res & 0xFF) as u8;
	    let z: bool = to_write == 0;

	    self.write_a(to_write);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(2, 2)
	}

    pub fn add_ahl(&mut self) -> ProgramCounter {
        // reading
        let a: u8 = self.read_from_r8(A_ID).unwrap();
        let r: u8 = self.interconnect.read(self.reg.hl);

        // processing
        let res: u16 = (a as u16) + (r as u16);

        // flags and writing
	    let h: bool = ((a & 0x0F) + (r & 0x0F)) > 0x0F;
	    let c: bool = res > 0xFF;
	    let n: bool = false;
	    let to_write: u8 = (res & 0xFF) as u8;
	    let z: bool = to_write == 0;

	    self.write_a(to_write);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 2)
    }
        
    pub fn adc_ar(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let carry: u8 = ((self.reg.f & CF) > 0) as u8; 
	    let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx).unwrap();

	    // processing
	    let res: u16 = (a as u16) + (r as u16) + (carry as u16);

	    // flags and writing
	    let h: bool = ((a & 0x0F) + (r & 0x0F) + (carry & 0x0F)) > 0x0F;
	    let c: bool = res > 0xFF;
	    let n: bool = false;
	    let to_write: u8 = (res & 0xFF) as u8;
	    let z: bool = to_write == 0;

	    self.write_a(to_write);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 1)
	}

	// ADD A, n: add immediate operand n to register A.
	// Cycles: 2
	pub fn adc_an(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let carry: u8 = ((self.reg.f & CF) > 0) as u8; 
	    let r: u8 = self.get_n();

	    // processing
	    let res: u16 = (a as u16) + (r as u16) + (carry as u16);

	    // flags and writing
	    let h: bool = ((a & 0x0F) + (r & 0x0F) + (carry & 0x0F)) > 0x0F;
	    let c: bool = res > 0xFF;
	    let n: bool = false;
	    let to_write: u8 = (res & 0xFF) as u8;
	    let z: bool = to_write == 0;

	    self.write_a(to_write);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(2, 2)
	}

    pub fn adc_ahl(&mut self) -> ProgramCounter {
        // reading
        let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let carry: u8 = ((self.reg.f & CF) > 0) as u8; 
        let r: u8 = self.interconnect.read(self.reg.hl);

        // processing
        let res: u16 = (a as u16) + (r as u16) + (carry as u16);

        // flags and writing
	    let h: bool = ((a & 0x0F) + (r & 0x0F) + (carry & 0x0F)) > 0x0F;
	    let c: bool = res > 0xFF;
	    let n: bool = false;
	    let to_write: u8 = (res & 0xFF) as u8;
	    let z: bool = to_write == 0;

	    self.write_a(to_write);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 2)
    }

    pub fn sub_r(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx).unwrap();

	    // processing
	    let res: u8 = a.wrapping_sub(r);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub(r & 0x0F) == None;
	    let c: bool = (a).checked_sub(r) == None;
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 1)
	}

	// Cycles: 2
	pub fn sub_n(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let r: u8 = self.get_n();

	    // processing
	    let res: u8 = a.wrapping_sub(r);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub(r & 0x0F) == None;
	    let c: bool = (a).checked_sub(r) == None;
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(2, 2)
	}

    pub fn sub_hl(&mut self) -> ProgramCounter {
        // reading
        let a: u8 = self.read_from_r8(A_ID).unwrap();
        let r: u8 = self.interconnect.read(self.reg.hl);

        // processing
	    let res: u8 = a.wrapping_sub(r);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub(r & 0x0F) == None;
	    let c: bool = (a).checked_sub(r) == None;
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 2)
    }
        
    pub fn sbc_ar(&mut self) -> ProgramCounter {
	    // reading
	    let carry: u8 = ((self.reg.f & CF) > 0) as u8; 
        let a: u8 = self.read_from_r8(A_ID).unwrap();
        let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx).unwrap();

        // processing
	    let res: u8 = a.wrapping_sub(r).wrapping_sub(carry);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub((r & 0x0F) + carry) == None;
	    let c: bool = (a as u16) < (r as u16 + carry as u16);
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 1)
	}

	// ADD A, n: add immediate operand n to register A.
	// Cycles: 2
	pub fn sbc_an(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let carry: u8 = ((self.reg.f & CF) > 0) as u8; 
        let r: u8 = self.get_n();

        // processing
	    let res: u8 = a.wrapping_sub(r).wrapping_sub(carry);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub((r & 0x0F) + carry) == None;
	    let c: bool = (a as u16) < (r as u16 + carry as u16);
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(2, 2)
	}

    pub fn sbc_ahl(&mut self) -> ProgramCounter {
        // reading
	    let carry: u8 = ((self.reg.f & CF) > 0) as u8; 
        let a: u8 = self.read_from_r8(A_ID).unwrap();
        let r: u8 = self.interconnect.read(self.reg.hl);

        // processing
	    let res: u8 = a.wrapping_sub(r).wrapping_sub(carry);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub((r & 0x0F) + carry) == None;
	    let c: bool = (a as u16) < (r as u16 + carry as u16);
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 2)
    }

    pub fn and_r(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx).unwrap();

	    // processing
	    let res: u8 = a & r;

	    // flags and writing
	    let h: bool = true;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 1)
	}

	// ADD A, n: add immediate operand n to register A.
	// Cycles: 2
	pub fn and_n(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let r: u8 = self.get_n();

	    // processing
	    let res: u8 = a & r;

	    // flags and writing
	    let h: bool = true;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(2, 2)
	}

    pub fn and_hl(&mut self) -> ProgramCounter {
        // reading
        let a: u8 = self.read_from_r8(A_ID).unwrap();
        let r: u8 = self.interconnect.read(self.reg.hl);

        // processing
	    let res: u8 = a & r;

	    // flags and writing
	    let h: bool = true;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 2)
    }

    pub fn or_r(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx).unwrap();

	    // processing
	    let res: u8 = a | r;
	    // flags and writing
	    let h: bool = false;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 1)
	}

	// ADD A, n: add immediate operand n to register A.
	// Cycles: 2
	pub fn or_n(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let r: u8 = self.get_n();

	    // processing
	    let res: u8 = a | r;

	    // flags and writing
	    let h: bool = false;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(2, 2)
	}

    pub fn or_hl(&mut self) -> ProgramCounter {
        // reading
        let a: u8 = self.read_from_r8(A_ID).unwrap();
        let r: u8 = self.interconnect.read(self.reg.hl);

        // processing
	    let res: u8 = a | r;

	    // flags and writing
	    let h: bool = false;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 2)
    }

    pub fn xor_r(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx).unwrap();

	    // processing
	    let res: u8 = a ^ r;

	    // flags and writing
	    let h: bool = false;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 1)
	}

	// ADD A, n: add immediate operand n to register A.
	// Cycles: 2
	pub fn xor_n(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let r: u8 = self.get_n();

	    // processing
	    let res: u8 = a ^ r;

	    // flags and writing
	    let h: bool = false;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(2, 2)
	}

    pub fn xor_hl(&mut self) -> ProgramCounter {
        // reading
        let a: u8 = self.read_from_r8(A_ID).unwrap();
        let r: u8 = self.interconnect.read(self.reg.hl);

        // processing
	    let res: u8 = a ^ r;

	    // flags and writing
	    let h: bool = false;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 2)
    }

    pub fn cp_r(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx).unwrap();

	    // processing
	    let res: u8 = a.wrapping_sub(r);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub(r & 0x0F) == None;
	    let c: bool = (a).checked_sub(r) == None;
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 1)
	}

	// Cycles: 2
	pub fn cp_n(&mut self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID).unwrap();
	    let r: u8 = self.get_n();

	    // processing
	    let res: u8 = a.wrapping_sub(r);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub(r & 0x0F) == None;
	    let c: bool = (a).checked_sub(r) == None;
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(2, 2)
	}

    pub fn cp_hl(&mut self) -> ProgramCounter {
        // reading
        let a: u8 = self.read_from_r8(A_ID).unwrap();
        let r: u8 = self.interconnect.read(self.reg.hl);

        // processing
	    let res: u8 = a.wrapping_sub(r);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub(r & 0x0F) == None;
	    let c: bool = (a).checked_sub(r) == None;
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1, 2)
    }

    pub fn inc_r(&mut self) -> ProgramCounter {
	    // reading
	    let idx: u8 = self.get_r8_to();
	    let r: u8 = self.read_from_r8(idx).unwrap();

	    // processing
	    let res: u8 = if r == std::u8::MAX {0} else {r + 1};

	    // flags and writing
	    let h: bool = (r & 0xF) == 0xF;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_to_r8(idx, res);
	    self.set_hnz(h, n, z);

	    ProgramCounter::Next(1, 1)
	}

	pub fn inc_hl(&mut self) -> ProgramCounter {
	    // reading
	    let r: u8 = self.interconnect.read(self.reg.hl);

	    // processing
	    let res: u8 = if r == std::u8::MAX {0} else {r + 1};

	    // flags and writing
	    let h: bool = (r & 0xF) == 0xF;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.interconnect.write(self.reg.hl, res);
	    self.set_hnz(h, n, z);

	    ProgramCounter::Next(1, 3)
	}

	pub fn dec_r(&mut self) -> ProgramCounter {
	    // reading
	    let idx: u8 = self.get_r8_to();
	    let r: u8 = self.read_from_r8(idx).unwrap();

	    // processing
	    let res: u8 = if r == 0 {std::u8::MAX} else {r - 1};

	    // flags and writing
	    let h: bool = r & 0xF == 0x0;
	    let n: bool = true;
	    let z: bool = res == 0;

        if res == 0 {
            println!(" ******** Register ID {:x} REACHED 0********", idx);
        }
         
	    self.write_to_r8(idx, res);
	    self.set_hnz(h, n, z);

	    ProgramCounter::Next(1, 1)
	}

	pub fn dec_hl(&mut self) -> ProgramCounter {
	    // reading
	    let r: u8 = self.interconnect.read(self.reg.hl);

	    // processing
	    let res: u8 = if r == 0 {std::u8::MAX} else {r - 1};

	    // flags and writing
	    let h: bool = (r & 0xF) == 0x0;
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.interconnect.write(self.reg.hl, res);
	    self.set_hnz(h, n, z);

	    ProgramCounter::Next(1, 3)
	}

	// 2.4 16-bit intstructions

	pub fn add_hlss(&mut self) -> ProgramCounter {
		// reading
	    let idx: u8 = (self.get_r8_to() & 0b110) >> 1;
	    let r: u16 = self.read_from_r16(idx).unwrap();
	    let hl: u16 = self.read_from_r16(HL_ID).unwrap();

	    // processing
	    let res: u32 = r as u32 + hl as u32;

	    // flags and writing
	    let h: bool = ((hl & 0x0FFF) + (r & 0x0FFF)) > 0x0FFF;
	    let c: bool = res > 0xFFFF;
	    let n: bool = false;
	    let to_write: u16 = (res & 0xFFFF) as u16;

	    self.write_to_r16(HL_ID, to_write);
	    self.set_hcn(h, c, n);

	    ProgramCounter::Next(1, 2)
	}

	pub fn add_spe(&mut self) -> ProgramCounter {
		// reading
	    let r: u16 = self.get_n() as u16;
	    let sp: u16 = self.read_from_r16(SP_ID).unwrap();

	    // processing
	    let res: u32 = r as u32 + sp as u32;

	    // flags and writing
	    let h: bool = ((sp & 0x0FFF) + (r & 0x0FFF)) > 0x0FFF;
	    let c: bool = res > 0xFFFF;
	    let n: bool = false;
	    let z: bool = false;
	    let to_write: u16 = (res & 0xFFFF) as u16;

	    self.write_to_r16(SP_ID, to_write);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(2, 4)
	}

	pub fn inc_ss(&mut self) -> ProgramCounter {
		// reading
	    let idx: u8 = (self.get_r8_to() & 0b110) >> 1;
	    let r: u16 = self.read_from_r16(idx).unwrap();

	    // processing
	    let res: u16 = if r == std::u16::MAX {0} else {r + 1};

	    self.write_to_r16(idx, res);
	    
	    ProgramCounter::Next(1, 2)
	}

	pub fn dec_ss(&mut self) -> ProgramCounter {
		// reading
	    let idx: u8 = (self.get_r8_to() & 0b110) >> 1;
	    let r: u16 = self.read_from_r16(idx).unwrap();

	    // processing
	    let res: u16 = if r == 0 {std::u16::MAX} else {r - 1};

	    self.write_to_r16(idx, res);
	    
	    ProgramCounter::Next(1, 2)
	}

    // 2.5 Shift and Rotate instructions
    
    /// rlca: Rotates content of register A to the left. a7 <- a0
    /// is_left_rotate = true, has_carry = true
    /// 1-byte instruction
    pub fn rlca(&mut self) -> ProgramCounter {
        self.rotate_r8(A_ID, true, true);
        
        ProgramCounter::Next(1, 1)
    }

    /// rla: Rotates content of register A to the left. a7 <- cf
    /// is_left_rotate = true, has_carry = false
    /// 1-byte instruction.
    pub fn rla(&mut self) -> ProgramCounter {
        self.rotate_r8(A_ID, true, false);

        ProgramCounter::Next(1, 1)
    }

    /// rrca: Rotates content of register A to the right. a0 <- a7
    /// is_left_rotate = false, has_carry = true.
    /// 1-byte instruction
    pub fn rrca(&mut self) -> ProgramCounter {
        self.rotate_r8(A_ID, false, true);

        ProgramCounter::Next(1, 1)
    }

    /// rra: Rotates content of register A to the right. a0 <- cf
    /// is_left_rotate = false, has_carry = false.
    /// 1-byte instruction
    pub fn rra(&mut self) -> ProgramCounter {
        self.rotate_r8(A_ID, false, false);

        ProgramCounter::Next(1, 1)
    }

    /// rlc: Rotates content of either some register r or memory pointed to by HL, depending on
    /// opcode. to the left, with carry.
    pub fn rlc(&mut self) -> ProgramCounter {
        self.reg.pc += 1;
        let r = self.get_r8_from();
        self.reg.pc -= 1;

        let cycles = match r {
            0x06 => {
                self.rotate_mem(self.reg.hl, true, true);
                4
            },
            _ => {
                self.rotate_r8(r, true, true);
                2
            }
        };

        ProgramCounter::Next(2, cycles)
    }

    /// rl: Rotates content of either some register r or memory pointed to by HL, depending on
    /// opcode. to the left, without carry.
    pub fn rl(&mut self) -> ProgramCounter {
        self.reg.pc += 1;
        let r = self.get_r8_from();
        self.reg.pc -= 1;

        let cycles = match r {
            0x06 => {
                self.rotate_mem(self.reg.hl, true, false);
                4
            },
            _ => {
                self.rotate_r8(r, true, false);
                2
            },
        };

        ProgramCounter::Next(2, cycles)
    }
    
    /// rrc: Rotates content of either some register r or memory pointed to by HL, depending on
    /// opcode. to the right, with carry.
    pub fn rrc(&mut self) -> ProgramCounter {
        self.reg.pc += 1;
        let r = self.get_r8_from();
        self.reg.pc -= 1;

        let cycles = match r {
            0x06 => {
                self.rotate_mem(self.reg.hl, false, true);
                4
            },
            _ => {
                self.rotate_r8(r, false, true);
                2
            },
        };

        ProgramCounter::Next(2, cycles)
    }

    /// rr: Rotates content of either some register r or memory pointed to by HL, depending on
    /// opcode. to the right, without carry.
    pub fn rr(&mut self) -> ProgramCounter {
        self.reg.pc += 1;
        let r = self.get_r8_from();
        self.reg.pc -= 1;

        let cycles = match r {
            0x06 => {
                self.rotate_mem(self.reg.hl, false, false);
                4
            },
            _ => {
                self.rotate_r8(r, false, false);
                2
            },
        };

        ProgramCounter::Next(2, cycles)
    }

    /// SLA: Shift content of operand m to the left. Bit 7 is copied to CF, bit 0 is reset.
    /// 2-byte instruction
    pub fn sla(&mut self) -> ProgramCounter {
        self.reg.pc += 1;
        let r = self.get_r8_from();
        self.reg.pc -= 1;

        let mut data: u8;
        let bit_7: u8;

        let cycles = match r {
            0x06 => {
                data = self.interconnect.read(self.reg.hl);
                bit_7 = (data & 0x80) >> 7;
                
                // processing
                data = data << 1;
                
                // write back
                self.interconnect.write(self.reg.hl, data);
                4
            },
            _ => {
                data = self.read_from_r8(r).unwrap();
                bit_7 = (data & 0x80) >> 7;
                
                // processing
                data = data << 1;
                
                // write back
                self.write_to_r8(r, data);
                2
            },
        };

        // set flags
        self.set_hcnz(false, bit_7 > 0, false, data == 0);

        ProgramCounter::Next(2, cycles)
    }
        
    /// SRA: Shift content of operand m to the right. Bit 0 is copied to CF, bit 7 stays the same!.
    /// 2-byte instruction
    pub fn sra(&mut self) -> ProgramCounter {
        self.reg.pc += 1;
        let r = self.get_r8_from();
        self.reg.pc -= 1;

        let mut data: u8;
        let bit_0: u8;
        let bit_7: u8;

        let cycles = match r {
            0x06 => {
                data = self.interconnect.read(self.reg.hl);
                bit_7 = (data & 0x80) >> 7;
                bit_0 = data & 0x01;
                
                // processing
                data = data >> 1;
                data |= bit_7 << 7;
                
                // write back
                self.interconnect.write(self.reg.hl, data);
                
                4
            },
            _ => {
                data = self.read_from_r8(r).unwrap();
                bit_7 = (data & 0x80) >> 7;
                bit_0 = data & 0x01;
                
                // processing
                data = data >> 1;
                data |= bit_7 << 7;

                // write back
                self.write_to_r8(r, data);
                
                2
            },
        };

        // set flags
        self.set_hcnz(false, bit_0 > 0, false, data == 0);

        ProgramCounter::Next(2, cycles)
    }

    /// SRL: Shift content of operand m to the right. Bit 0 is copied to CF, bit 7 is reset.
    /// 2-byte instruction
    pub fn srl(&mut self) -> ProgramCounter {
        self.reg.pc += 1;
        let r = self.get_r8_from();
        self.reg.pc -= 1;

        let mut data: u8;
        let bit_0: u8;

        let cycles = match r {
            0x06 => {
                data = self.interconnect.read(self.reg.hl);
                bit_0 = data & 0x01;
                
                // processing
                data = data >> 1;
                
                // write back
                self.interconnect.write(self.reg.hl, data);
                4
            },
            _ => {
                data = self.read_from_r8(r).unwrap();
                bit_0 = data & 0x01;
                
                // processing
                data = data >> 1;

                // write back
                self.write_to_r8(r, data);
                2
            },
        };

        // set flags
        self.set_hcnz(false, bit_0 > 0, false, data == 0);

        ProgramCounter::Next(2, cycles)
    }

    /// SWAP: Shift content of lower-order 4 bits to higher-order 4 bits, and vice versa. Reset all
    /// flags except ZF.
    /// 2-byte instruction.
    pub fn swap(&mut self) -> ProgramCounter {
        self.reg.pc += 1;
        let r = self.get_r8_from();
        self.reg.pc -= 1;

        let mut data: u8;
       
        let cycles = match r {
            0x06 => {
                // read
                data = self.interconnect.read(self.reg.hl);
                
                // process
                let lower = data & 0x0F;
                let higher = (data & 0xF0) >> 4;
                data = (lower << 4) | higher;

                // write back
                self.interconnect.write(self.reg.hl, data);
                4
            },
            _ => {
                // read
                data = self.read_from_r8(r).unwrap();

                // process
                let lower = data & 0x0F;
                let higher = (data & 0xF0) >> 4;
                data = (lower << 4) | higher;
                
                // write back
                self.write_to_r8(r, data);
                2
            }
        };
        self.set_hcnz(false, false, false, data == 0);
        
        ProgramCounter::Next(2, cycles)
    }

    // CB (bit operation)
    
    /// bit_b_r: Copies complement of bit_b of register r to Z flag.
    /// 2 bytes, 2 cycles
    pub fn bit_b_r(&mut self) -> ProgramCounter {
        let br_info = self.get_n();
        let b = (br_info & 0x38) >> 3;
        let r = br_info & 0x07;
        
        let mut val: u8 = self.read_from_r8(r).unwrap();
        val = (val >> b) & 0x01;

        // set the flag
        self.set_hnz(true, false, val == 0);

        ProgramCounter::Next(2, 2)
    }

    /// bit_b_hl: Copies complement of bit_b of memory content at HL to Z flag
    /// 2 bytes, 3 cycles
    pub fn bit_b_hl(&mut self) -> ProgramCounter {
        let b_info = self.get_n();
        let b = (b_info & 0x38) >> 3;
        
        let mut val: u8 = self.interconnect.read(self.reg.hl);
        val = (val >> b) & 0x01;

        // set the flag
        self.set_hnz(true, false, val == 0);

        ProgramCounter::Next(2, 3)
    }
    
    /// set_b_r: Set bit_b of register r to 1.
    /// 2 bytes, 2 cycles
    pub fn set_b_r(&mut self) -> ProgramCounter {
        let br_info = self.get_nn();
        let b = (br_info & 0x38) >> 3;
        let r = (br_info & 0x07) as u8;

        let mut val: u8 = self.read_from_r8(r).unwrap();
        val = val | (0x01 << b);

        // write back to register
        self.write_to_r8(r, val);

        ProgramCounter::Next(2, 2)
    }

    /// set_b_hl: set bit_b of memory content at HL to 1.
    /// 2 bytes, 4 cycles
    pub fn set_b_hl(&mut self) -> ProgramCounter {
        let b_info = self.get_nn();
        let b = (b_info & 0x38) >> 3;
        
        let mut val: u8 = self.interconnect.read(self.reg.hl);
        val = val | (0x01 << b);

        // write back
        self.interconnect.write(self.reg.hl, val);

        ProgramCounter::Next(2, 4)
    }

    /// res_b_r: set bit_b of register r to 0.
    /// 2 bytes, 2 cycles
    pub fn res_b_r(&mut self) -> ProgramCounter {
        let br_info = self.get_nn();
        let b = (br_info & 0x38) >> 3;
        let r = (br_info & 0x07) as u8;

        let mut val: u8 = self.read_from_r8(r).unwrap();
        val &= !(0x01 << b);

        // write back to register
        self.write_to_r8(r, val);

        ProgramCounter::Next(2, 2)
    }

    /// res_b_hl: set bit_b of memory content at HL to 0.
    /// 2 bytes, 4 cycles
    pub fn res_b_hl(&mut self) -> ProgramCounter {
        let b_info = self.get_nn();
        let b = (b_info & 0x38) >> 3;
        
        let mut val: u8 = self.interconnect.read(self.reg.hl);
        val &= !(0x01 << b);

        // write back
        self.interconnect.write(self.reg.hl, val);

        ProgramCounter::Next(2, 4)
    }

    // 2.6 Control Flow Instruction

    /// jp_nn: unconditional jump to absolute address specified by 16-bit immediate. Set PC = nn
    /// 3-byte instruction, 4 cycles.
    pub fn jp_nn(&mut self) -> ProgramCounter {
        print!("self.get_nn:");
        println!("{:?}", self.get_nn());
        ProgramCounter::Jump(self.get_nn(), 4)
    }

    /// jp_hl: unconditional jump to absolute address specified by 16-bit register HL. Set PC = HL.
    /// 1-byte instruction, 1 cycle.
    pub fn jp_hl(&mut self) -> ProgramCounter {
        ProgramCounter::Jump(self.reg.hl, 1)
    }

    /// jp_cc_nn: Conditional jump to absolute address nn, depending on condition cc.
    /// cc is 2-bit number that refers to:
    /// 00 -> Z == 0; 01 -> Z == 1; 10 -> C == 0; 11 -> C == 1
    /// 3-byte instruction
    pub fn jp_cc_nn(&mut self) -> ProgramCounter {
        let abs_addr = self.get_nn();
        let cc = self.check_cc();
        let pc_final: ProgramCounter;

        if cc {
            pc_final = ProgramCounter::Jump(abs_addr, 4);
        } else {
            pc_final = ProgramCounter::Next(3, 3);
        }

        pc_final
    }

    /// jr_e: Unconditional jump to relative address specified by signed 8-bit operand e.
    /// 2 bytes, 3 cycles.
    pub fn jr_e(&mut self) -> ProgramCounter {
        let e = (self.get_n() as i8) as i16;
        println!("{:?}", e);
        ProgramCounter::Next(e + 2, 3)
    }

    /// jr_cc_e: Conditional jump to relative address specified by signed 8-bit operand e, depending on condition cc.
    /// 2 bytes, 2 cycles if cc == false, 3 cycles if cc == true.
    pub fn jr_cc_e(&mut self) -> ProgramCounter {
        let e = (self.get_n() as i8) as i16;
        let cc = self.check_cc();
        let pc_final: ProgramCounter;
        
        if cc {
            pc_final = ProgramCounter::Next(e + 2, 3);
        } else {
            pc_final = ProgramCounter::Next(2, 2);
        }

        pc_final
    }    

    /// call_nn: unconditional function call to absolute address specified by 16-bit operand nn
    /// 3 bytes. 6 cycles
    pub fn call_nn(&mut self) -> ProgramCounter {
        let nn = self.get_nn();
        self.push_u16(self.reg.pc + 3); // Push NEXT PC (the one after calling call_nn) onto the stack
        
        ProgramCounter::Jump(nn, 6)
    }

    /// call_cc_nn: Conditional function call to absolute address specified by 16-bit operand nn,
    /// depending on condition cc.
    /// 3 bytes, 3 cycles if cc == false, 6 cycles if cc ==  true
    pub fn call_cc_nn(&mut self) -> ProgramCounter {
        let nn = self.get_nn();
        let cc = self.check_cc();

        let pc_final: ProgramCounter;

        if cc { // execute function call
            self.push_u16(self.reg.pc + 3);
            pc_final = ProgramCounter::Jump(nn, 6);
        } else {
            pc_final = ProgramCounter::Next(3, 3);
        }

        pc_final
    }

    /// ret: Unconditional return from a function. Pop PC from stack.
    /// 1 byte, 4 cycles.
    pub fn ret(&mut self) -> ProgramCounter {
        let pop_val = self.pop_u16();

        ProgramCounter::Jump(pop_val, 4)
    }

    /// ret_cc: Conditional return from a function, depending on condition cc.
    /// Only pop if cc is true.
    /// 1 byte, 2 cycles if cc = false, 5 cycles if cc = true
    pub fn ret_cc(&mut self) -> ProgramCounter {
        let cc = self.check_cc();
        let pc_final: ProgramCounter;

        if cc {
            let pop_val = self.pop_u16();
            pc_final = ProgramCounter::Jump(pop_val, 5);
        } else {
            pc_final = ProgramCounter::Next(1, 2);
        }

        pc_final
    }

    /// reti: Unconditional return from a function. Enables IME signal.
    /// IME is Interrupt Master Enable. When this is enabled, interrupts can happen
    /// same as ret, but set register IME.
    pub fn reti(&mut self) -> ProgramCounter {
        let pop_val = self.pop_u16();
        self.reg.ime = true;

        ProgramCounter::Jump(pop_val, 4)
    }

    /// rst_n: Unconditional function call to absolute fixed address defined by opcode.
    /// opcode specifies rst_address in xxx: bits 3 4 5. Each combination of xxx indicates an
    /// address.
    /// 1 byte, 4 cycles.
    pub fn rst_n(&mut self) -> ProgramCounter {
        // push pc onto stack
        self.push_u16(self.reg.pc + 1);

        let xxx = self.get_r8_to(); // same bits
        let pc_msb: u16 = 0x00;
        let pc_lsb: u16;

        match xxx {
            0 => pc_lsb = 0x00,
            1 => pc_lsb = 0x08,
            2 => pc_lsb = 0x10,
            3 => pc_lsb = 0x18,
            4 => pc_lsb = 0x20,
            5 => pc_lsb = 0x28,
            6 => pc_lsb = 0x30,
            7 => pc_lsb = 0x38,
            _ => panic!("Invalid pc lsb"),
        }

        let addr = (pc_msb << 8) | pc_lsb;

        ProgramCounter::Jump(addr, 4)
    }
        
    /// halt: Cpu enters "halt mode" and stops system clock. Oscillator circuit and LCD Controller
    /// continue to operate. "halt mode" can be cancelled with an interrupt or reset signal.
    /// PC is halted as well. After interrupted / reset, program starts from PC address.
    pub fn halt(&mut self) -> ProgramCounter {
        self.halt_mode = true;

        ProgramCounter::Next(1, 0)     // does not incrememt
    }
    
    /// stop: Cpu enters "stop mode" and stops everything including system clock, 
    /// oscillator circuit and LCD Controller.
    /// 1 byte, 1 cycle
    pub fn stop(&mut self) -> ProgramCounter {
        self.stop_mode = true;

        ProgramCounter::Next(1, 0)     // does not increment
    }

    /// di: Disables interrupt handling by setting IME = 0, cancelling any scheduled effects of the
    /// EI instruction if any.
    /// 1 byte, 1 cycle
    pub fn di(&mut self) -> ProgramCounter {
        self.reg.ime = false;

        ProgramCounter::Next(1, 1)
    }

    /// ei: schedules interrupt handling to be enabled THE NEXT MACHINE CYCLE
    /// 1 byte, 1 cycle + 1 cycle for EI effect.
    pub fn ei(&mut self) -> ProgramCounter {
        self.reg.ime = true;

        ProgramCounter::Next(1, 1)
    }

    /// ccf: Flips carry flag, reset N and H flags
    /// 1 byte, 1 cycle.
    pub fn ccf(&mut self) -> ProgramCounter {
        let c_bit = self.reg.f & CF;

        // set all the flags
        self.set_hcn(false, c_bit == 0, false);

        ProgramCounter::Next(1, 1)
    }

    /// scf: Sets carry flag, reset N and H flags.
    /// 1 byte, 1 cycle
    pub fn scf(&mut self) -> ProgramCounter {
        // set carry, reset n and h
        self.set_hcn(false, true, false);

        ProgramCounter::Next(1, 1)
    }

    /// nop: this doesn't do anything lmao, but add one cycle and increment PC by 1.
    /// 1 byte, 1 cycle
    pub fn nop(&mut self) -> ProgramCounter {
        ProgramCounter::Next(1, 1)
    }

    /// daa: decimal adjust acc.
    /// This is binary arithmetic acting as binary numbers...
    /// 1 byte, 1 cycle.
    pub fn daa(&mut self) -> ProgramCounter {
        let mut a: u8 = self.read_from_r8(A_ID).unwrap();

        let is_addition: bool = (self.reg.f & NF) == 0;
        let c_flag: bool = (self.reg.f & CF) > 0;
        let h_flag: bool = (self.reg.f & HF) > 0;
        let n_flag: bool = (self.reg.f & NF) > 0;
        let mut has_carry: bool = false;

        if is_addition { // after addition, adjust if half-carry occured or if results out of bounds.
            if a > 0x90 || c_flag {
                a += 0x60;
                has_carry = true;
            }

            if (a & 0x0F) > 0x09 || h_flag {
                a += 0x06;
            }
        } else { // after subtraction, adjust if half-carry occured.
            if c_flag {
                a -= 0x60;
            }

            if h_flag {
                a -= 0x06;
            }
        }

        // Write back data to reg A
        self.write_to_r8(A_ID, a);

        // Add set flags
        self.set_hcnz(has_carry, false, n_flag, a == 0);

        ProgramCounter::Next(1, 1)
    }

    /// cpl: flip all bits in the A-register, sets N and H to 1.
    /// 1 byte, 1 cycle
    pub fn cpl(&mut self) -> ProgramCounter {
        let mut a: u8 = self.read_from_r8(A_ID).unwrap();

        let mut n = 0;

        while n < 8 {
            // reverse every bit in a
            a = a ^ (0x01 << n);
            n += 1;
        }

        self.write_to_r8(A_ID, a);

        // Add set flags
        self.set_hnz(true, true, self.reg.f & ZF > 0);

        ProgramCounter::Next(1, 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A_DEF: u8 = 0x88;
    const B_DEF: u8 = 0x44;
    const C_DEF: u8 = 0x22;
    const D_DEF: u8 = 0x11;
    const HL_DEF: u16 = 0xFF00;
    const DE_DEF: u16 = 0x00FF;

    const MEM_HL_DEF: u8 = 0x08;
    const MEM_DE_DEF: u8 = 0x04;

    const N_DEF: u8 = 0xAB;
    const NN_DEF: u16 = 0xABCD;

    fn set_up_cpu() -> Cpu {
        let mut cpu = Cpu::new(Interconnect::new(Cart::new(vec![0; 36452].into_boxed_slice())));
        cpu.reg.a = A_DEF;
        cpu.reg.b = B_DEF;
        cpu.reg.c = C_DEF;
        cpu.reg.d = D_DEF;
        cpu.reg.hl= HL_DEF;
        cpu.reg.de= DE_DEF;
        cpu.mem[cpu.reg.hl as usize] = MEM_HL_DEF;
        cpu.mem[cpu.reg.de as usize] = MEM_DE_DEF;
        
        cpu
    }

    fn set_1byte_op(cpu: &mut Cpu, opcode: u8) {
        cpu.mem[cpu.reg.pc as usize] = opcode;
    }

    fn set_2byte_op(cpu: &mut Cpu, opcode: u16) {
        cpu.mem[cpu.reg.pc as usize] = (opcode >> 8) as u8;
        cpu.mem[(cpu.reg.pc + 1) as usize] = opcode as u8;
    }

    fn set_3byte_op(cpu: &mut Cpu, opcode: u32) {
        cpu.mem[cpu.reg.pc as usize] = (opcode >> 16) as u8;
        cpu.mem[(cpu.reg.pc + 1) as usize] = (opcode >> 8)  as u8;
        cpu.mem[(cpu.reg.pc + 2) as usize] = opcode as u8;
    }

    fn set_4byte_op(cpu: &mut Cpu, opcode: u32) {
        cpu.mem[cpu.reg.pc as usize] = (opcode >> 24) as u8;
        cpu.mem[(cpu.reg.pc + 1) as usize] = (opcode >> 16) as u8;
        cpu.mem[(cpu.reg.pc + 2) as usize] = (opcode >> 8) as u8;
        cpu.mem[(cpu.reg.pc + 3) as usize] = opcode as u8;
    }

    fn get_from_hl(cpu: &Cpu) -> u8 {
        cpu.mem[cpu.reg.hl as usize]
    }

    #[test]
    fn test_initial_state() {
        let cpu = Cpu::new(Interconnect::new(Cart::new(vec![0; 32645].into_boxed_slice())));
        // registers
        assert_eq!(cpu.reg.pc, 0x0100);
        assert_eq!(cpu.reg.a, 0x01);
        assert_eq!(cpu.reg.f, 0xB0);
        assert_eq!(cpu.reg.b, 0x00);
        assert_eq!(cpu.reg.c, 0x13);
        assert_eq!(cpu.reg.d, 0x00);
        assert_eq!(cpu.reg.e, 0xD8);
        assert_eq!(cpu.reg.h, 0x01);
        assert_eq!(cpu.reg.l, 0x4D);
        assert_eq!(cpu.reg.bc, 0x0013);
        assert_eq!(cpu.reg.de, 0x00D8);
        assert_eq!(cpu.reg.hl, 0x014D);
        assert_eq!(cpu.reg.sp, 0xFFFE);
        assert_eq!(cpu.reg.ime, true);
    }

    /*
    #[test]
    fn test_8bit_ld() {
        let mut cpu = set_up_cpu(); 
        // ld_addr_hl_n
        set_2byte_op(&mut cpu, 0b0011_0110_0000_0000 | (N_DEF as u16));
        cpu.execute_opcode();
        assert_eq!(get_from_hl(&cpu), N_DEF);
    }
    */

    #[test]
    fn test_write_to_r8() {
        let mut cpu = set_up_cpu();
        cpu.write_to_r8(D_ID, N_DEF);
        cpu.write_to_r8(E_ID, 0x00);
        assert_eq!(cpu.reg.d, N_DEF);
        assert_eq!(cpu.reg.e, 0x00);
        assert_eq!(cpu.reg.de, (N_DEF as u16) << 8);
    }

    #[test]
    fn test_read_from_r8() {
        let mut cpu = set_up_cpu();
        assert_eq!(cpu.read_from_r8(B_ID), Some(B_DEF));
    }
    
    #[test]
    fn test_load_mem_to_r8() {
        let mut cpu = set_up_cpu();
        cpu.load_mem_to_r8(A_ID, HL_DEF);
        assert_eq!(cpu.reg.a, MEM_HL_DEF);
    }
    
    #[test]
    fn test_save_r8_to_mem() {
        let mut cpu = set_up_cpu();
        cpu.save_r8_to_mem(A_ID, HL_DEF);
        assert_eq!(cpu.reg.a, get_from_hl(&cpu));
    }
    
    #[test]
    fn get_n() {
        let mut cpu = set_up_cpu();
        set_2byte_op(&mut cpu, 0xFF00 | (N_DEF as u16));
        assert_eq!(cpu.get_n(), N_DEF);
    }

    #[test]
    fn get_r8_to() {
        let mut cpu = set_up_cpu();
        set_1byte_op(&mut cpu, 0b00_000_000 | (B_ID << 3) | (C_ID));
        assert_eq!(cpu.get_r8_to(), B_ID);
        assert_eq!(cpu.get_r8_from(), C_ID);
    }

    #[test]
    fn get_nn() {
        let mut cpu = set_up_cpu();
        set_3byte_op(&mut cpu, 0x0000_0000 | NN_DEF as u32);

        assert_eq!(cpu.get_nn(), 0xCDAB);
    }

    #[test]
    fn get_r16() {
        let mut cpu = set_up_cpu();
        set_1byte_op(&mut cpu, 0b_00_000_001 | (SP_ID << 4));
        assert_eq!(cpu.get_r16(), SP_ID);
    }
    
    #[test]
    fn rotate_r8() {
        let mut cpu = set_up_cpu();
        // left, carry
        cpu.reg.a = 0x85;
        cpu.reg.f = 0x00;
        cpu.rotate_r8(A_ID, true, true);
        assert_eq!(cpu.reg.a, 0x0B);
        assert!(cpu.reg.f & CF > 0);
        // left, no carry
        cpu.reg.b = 0x80;
        cpu.reg.f = 0x00;
        cpu.rotate_r8(B_ID, true, false);
        assert_eq!(cpu.reg.b, 0x00);
        assert!(cpu.reg.f & CF > 0);
        // right, carry
        cpu.reg.c = 0x01;
        cpu.reg.f = 0x00;
        cpu.rotate_r8(C_ID, false, true);
        assert_eq!(cpu.reg.c, 0x80);
        assert!(cpu.reg.f & CF > 0);
        // right, no carry
        cpu.reg.d = 0x01;
        cpu.reg.f = 0x00;
        cpu.rotate_r8(D_ID, false, false);
        assert_eq!(cpu.reg.d, 0x00);
        assert!(cpu.reg.f & CF > 0);
    }

    #[test]
    fn push_u16() {
        let mut cpu = set_up_cpu();
        let last_sp = cpu.reg.sp;
        cpu.push_u16(0x3648);
        assert_eq!(cpu.stack[cpu.reg.sp as usize], 0x48);
        assert_eq!(cpu.stack[(cpu.reg.sp + 1) as usize], 0x36);
        assert!(last_sp - 2 == cpu.reg.sp);
    }

    #[test]
    fn write_a() {
        let mut cpu = set_up_cpu();
        cpu.write_a(N_DEF);
        assert_eq!(cpu.reg.a, N_DEF);
    }

    // 16-bit transfer

    #[test] 
    fn pop_u16() {
        let mut cpu = set_up_cpu();
        cpu.push_u16(0xFF23);
        assert_eq!(cpu.pop_u16(), 0xFF23);
    }










    // Test for 8-bit arithmetic instructions

    #[test]
    pub fn add_ar() {
        let mut cpu = set_up_cpu();
        cpu.reg.a = 0x3A;
        cpu.reg.b = 0xC6;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0x80 | B_ID);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x00);
        assert_eq!(cpu.reg.f, ZF+HF+CF);
        assert_eq!(cpu.reg.pc, pc_after);
    }


    #[test]
    fn add_an() {
        let mut cpu = set_up_cpu();
        cpu.reg.a = 0x3C;
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xC600 | 0xFF);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x3B);
        assert_eq!(cpu.reg.f, HF + CF);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn add_ahl() {
        let mut cpu = set_up_cpu();
        cpu.reg.a = 0x3C;
        cpu.mem[HL_DEF as usize] = 0x12;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0x86);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x4E);
        assert_eq!(cpu.reg.f, 0x00);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn adc_as() {
        let mut cpu = set_up_cpu();
        
        // ADC A, E
        cpu.reg.a = 0xE1;
        cpu.reg.e = 0x0F;
        set_1byte_op(&mut cpu, 0x88 | E_ID);
        let pc_after = cpu.reg.pc + 1;
        // set CY flag
        cpu.set_flag(CF);
        cpu.execute_opcode(); // adc_ar
        assert_eq!(cpu.reg.a, 0xF1);
        assert_eq!(cpu.reg.f, HF);
        assert_eq!(cpu.reg.pc, pc_after);
        cpu.reg.f = 0;
    
        // ADC A, m
        cpu.reg.a = 0xE1;
        let n = 0x3B;
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCE00 | n);
        cpu.set_flag(CF);
        cpu.execute_opcode(); // adc_an
        assert_eq!(cpu.reg.a, 0x1D);
        assert_eq!(cpu.reg.f, CF);
        assert_eq!(cpu.reg.pc, pc_after);
        cpu.reg.f = 0;

        // ADC A, (HL)
        cpu.reg.a = 0xE1;
        cpu.mem[HL_DEF as usize] = 0x1E;
        set_1byte_op(&mut cpu, 0x8E); 
        let pc_after = cpu.reg.pc + 1;
        cpu.set_flag(CF);
        cpu.execute_opcode(); // adc_hl
        assert_eq!(cpu.reg.a, 0x00);
        assert_eq!(cpu.reg.f, ZF+HF+CF);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn sub_s() {
        let mut cpu = set_up_cpu();
        
        //  SUB A, E
        cpu.reg.a = 0x3E;
        cpu.reg.e = 0x3E;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0x90 | E_ID);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x00);
        assert_eq!(cpu.reg.f, ZF + NF);
        assert_eq!(cpu.reg.pc, pc_after);
        cpu.reg.f = 0;

        // SUB A, n
        cpu.reg.a = 0x3E;
        let n = 0x0F;
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xD600 | n);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x2F);
        assert_eq!(cpu.reg.f, HF+NF);
        assert_eq!(cpu.reg.pc, pc_after);
        cpu.reg.f = 0;

        // SUB A, (HL)
        cpu.reg.a = 0x3E;
        cpu.mem[HL_DEF as usize] = 0x40;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0x96);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a,0xFE);
        assert_eq!(cpu.reg.f, NF + CF);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn sbc_s() {
        let mut cpu = set_up_cpu();
        
        //  SBC A, E
        cpu.reg.a = 0x3B;
        cpu.reg.e = 0x2A;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0x98 | E_ID);
        cpu.set_flag(CF);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x10);
        assert_eq!(cpu.reg.f, NF);
        assert_eq!(cpu.reg.pc, pc_after);
        cpu.reg.f = 0;

        // SUB A, n
        cpu.reg.a = 0x3B;
        let n = 0x3A;
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xDE00 | n);
        cpu.set_flag(CF);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x00);
        assert_eq!(cpu.reg.f, ZF+NF);
        assert_eq!(cpu.reg.pc, pc_after);
        cpu.reg.f = 0;

        // SUB A, (HL)
        cpu.reg.a = 0x3B;
        cpu.mem[HL_DEF as usize] = 0x4F;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0x9E);
        cpu.set_flag(CF);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a,0xEB);
        assert_eq!(cpu.reg.f, HF + NF + CF);
        assert_eq!(cpu.reg.pc, pc_after);
    }


    #[test]
    fn and_s() {
        let mut cpu = set_up_cpu();
        cpu.reg.a = 0x5A;
        cpu.reg.b = 0x18;
        cpu.mem[HL_DEF as usize] = 0x00;

        //and B
        cpu.set_hcnz(false, false, false, false);
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0b10100000 | B_ID);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x5A & 0x18);
        assert_eq!(cpu.reg.f, 0b00100000);
        assert_eq!(cpu.reg.pc, pc_after);

        //and n
        cpu.set_hcnz(false, false, false, false);
        cpu.reg.a = 0x5A;
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0b1110011000000000 | 0x38);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x5A & 0x38);
        assert_eq!(cpu.reg.f, 0b00100000);
        assert_eq!(cpu.reg.pc, pc_after);

        //and HL
        cpu.set_hcnz(false, false, false, false);
        cpu.reg.a = 0x5A;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0b10100110);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x5A & 0x00);
        assert_eq!(cpu.reg.f, 0b10100000);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn or_s() {
        let mut cpu = set_up_cpu();
        cpu.reg.a = 0x5A;
        cpu.mem[HL_DEF as usize] = 0x0F;

        //and A
        cpu.set_hcnz(false, false, false, false);
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0b10110000 | A_ID);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x5A | 0x5A);
        assert_eq!(cpu.reg.f, 0b00000000);
        assert_eq!(cpu.reg.pc, pc_after);

        //and n
        cpu.set_hcnz(false, false, false, false);
        cpu.reg.a = 0x5A;
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0b11_110_110_0000_0000 | 0x3);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x5A | 0x3);
        assert_eq!(cpu.reg.f, 0b00000000);
        assert_eq!(cpu.reg.pc, pc_after);

        //and HL
        cpu.set_hcnz(false, false, false, false);
        cpu.reg.a = 0x5A;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0b10_110_110);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x5A | 0x0F);
        assert_eq!(cpu.reg.f, 0b00000000);
        assert_eq!(cpu.reg.pc, pc_after);
    }


    #[test]
    fn xor_s() {
        let mut cpu = set_up_cpu();
        cpu.reg.a = 0xFF;
        cpu.mem[HL_DEF as usize] = 0x8A;

        //and A
        cpu.set_hcnz(false, false, false, false);
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0b10_101_000 | A_ID);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0xFF ^ 0xFF);
        assert_eq!(cpu.reg.f, 0b10000000);
        assert_eq!(cpu.reg.pc, pc_after);

        //and n
        cpu.set_hcnz(false, false, false, false);
        cpu.reg.a = 0xFF;
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0b11_101_110_0000_0000 | 0x0F);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0xFF ^ 0x0F);
        assert_eq!(cpu.reg.f, 0b00000000);
        assert_eq!(cpu.reg.pc, pc_after);

        //and HL
        cpu.set_hcnz(false, false, false, false);
        cpu.reg.a = 0xFF;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0b10_101_110);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0xFF ^ 0x8A);
        assert_eq!(cpu.reg.f, 0b00000000);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn cp_s() {
        let mut cpu = set_up_cpu();
        cpu.reg.a = 0x3C;
        cpu.reg.b = 0x2F;
        cpu.mem[HL_DEF as usize] = 0x40;

        // cp B
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0xB8 | B_ID);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.f, HF + NF);
        assert_eq!(cpu.reg.pc, pc_after);
        
        // cp 3C
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xFE00 | 0x3C);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.f, ZF+NF);
        assert_eq!(cpu.reg.pc, pc_after);

        // cp (HL)
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0xBE);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.f, NF + CF);
        assert_eq!(cpu.reg.pc, pc_after);
        cpu.reg.f = 0;
    }
    
    #[test]
    pub fn dec_hl() {
        let mut cpu = set_up_cpu();
        cpu.mem[HL_DEF as usize] = 0x00;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0x35);
        cpu.execute_opcode();
        assert_eq!(cpu.mem[HL_DEF as usize], 0xFF);
        assert_eq!(cpu.reg.pc, pc_after);
        cpu.reset_flag(CF); // CF stays the same as before
        assert_eq!(cpu.reg.f, HF+NF);
    }
    
    #[test]
    pub fn dec_r() {
        let mut cpu = set_up_cpu();
        cpu.reg.l = 0x01;
        cpu.reg.f = 0;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0b00_000_101 | (L_ID << 3));
        cpu.execute_opcode();
        assert_eq!(cpu.reg.l, 0x00);
        assert_eq!(cpu.reg.f, ZF+NF);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    pub fn inc_hl() {
        let mut cpu = set_up_cpu();
        cpu.set_hcnz(false, false, false, false);
        cpu.mem[HL_DEF as usize] = 0x50;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0b00_110_100);
        cpu.execute_opcode();
        assert_eq!(cpu.mem[HL_DEF as usize], 0x51);
        assert_eq!(cpu.reg.f, 0);
        assert_eq!(cpu.reg.pc, pc_after);
    }
    
    #[test]
    pub fn inc_r() {
        let mut cpu = set_up_cpu();
        cpu.reg.l = 0xFF;
        cpu.set_hcnz(false, false, false, false);
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0b00_000_100 | (L_ID << 3));
        cpu.execute_opcode();
        assert_eq!(cpu.reg.l, 0x00);
        assert_eq!(cpu.reg.f, ZF+HF);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    pub fn add_hlss() {
        let mut cpu = set_up_cpu();
        
        // ADD HL, BC
        cpu.reg.hl = 0x8A23;
        cpu.reg.bc = 0x0605;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0b00_001_001 | (BC_ID << 4));
        cpu.reg.f = 0;
        cpu.execute_opcode();
        assert_eq!(cpu.reg.hl, 0x9028);
        assert_eq!(cpu.reg.f, HF);
        assert_eq!(cpu.reg.pc, pc_after);
        
        // ADD HL, HL
        cpu.reg.hl = 0x8A23;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0b00_001_001 | (HL_ID << 4));
        cpu.reg.f = 0;
        cpu.execute_opcode();
        assert_eq!(cpu.reg.hl, 0x1446);
        assert_eq!(cpu.reg.f, HF + CF);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    pub fn add_spe() {
        let mut cpu = set_up_cpu();
        cpu.reg.sp = 0xFFF8;
        let e = 0x02;
        cpu.reg.f = 0;
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xE800 | e);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.sp, 0xFFFA);
        assert_eq!(cpu.reg.f, 0);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn inc_ss() {
        let mut cpu = set_up_cpu();
        cpu.reg.de = 0x235F;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0x03 | (DE_ID << 4));
        cpu.execute_opcode();
        assert_eq!(cpu.reg.de, 0x2360);
        assert_eq!(cpu.reg.pc, pc_after);
    }
    
    #[test]
    fn dec_ss() { // 1-byte instruction, why set 2byte?
        let mut cpu = set_up_cpu();
        cpu.reg.de = 0x235F;
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0x0B | (DE_ID << 4));
        cpu.execute_opcode();
        assert_eq!(cpu.reg.de, 0x235E);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn rlca() {
        let mut cpu = set_up_cpu();
        cpu.reg.a = 0x85;
        cpu.reset_flag(CF); // C = 0
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0x07);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.f, CF);
        assert_eq!(cpu.reg.a, 0x0B); //Example in manual is wrong
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn rla() {
        let mut cpu = set_up_cpu();
        cpu.reg.a = 0x95;
        cpu.set_flag(CF); // C = 1
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0x017);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.f, CF);
        assert_eq!(cpu.reg.a, 0x2B);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn rrca() {
        let mut cpu = set_up_cpu();
        cpu.reg.a = 0x3B;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0x0F);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x9D);
        assert_eq!(cpu.reg.f, CF);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn rra() {
        let mut cpu = set_up_cpu();
        cpu.reg.a = 0x81;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 1;
        set_1byte_op(&mut cpu, 0x1F);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x40);
        assert_eq!(cpu.reg.f, CF);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn rlc() {
        let mut cpu = set_up_cpu();
        // RLC B
        cpu.reg.b = 0x85;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB00 | (B_ID as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.reg.b, 0x0B);
        assert_eq!(cpu.reg.f, CF);
        assert_eq!(cpu.reg.pc, pc_after);

        // RLC (HL)
        cpu.mem[HL_DEF as usize] = 0x00;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB00 | (0x06 as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.mem[HL_DEF as usize], 0x00);
        assert_eq!(cpu.reg.f, ZF);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn rl() {
        let mut cpu = set_up_cpu();
        // RLC B
        cpu.reg.b = 0x80;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB10 | (B_ID as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.reg.b, 0x00);
        assert_eq!(cpu.reg.f, CF + ZF);
        assert_eq!(cpu.reg.pc, pc_after);

        // RLC (HL)
        cpu.mem[HL_DEF as usize] = 0x11;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB10 | (0x06 as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.mem[HL_DEF as usize], 0x22);
        assert_eq!(cpu.reg.f, 0);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn rrc() {
        let mut cpu = set_up_cpu();
        // RLC B
        cpu.reg.b = 0x01;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB08 | (B_ID as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.reg.b, 0x80);
        assert_eq!(cpu.reg.f, CF);
        assert_eq!(cpu.reg.pc, pc_after);

        // RLC (HL)
        cpu.mem[HL_DEF as usize] = 0x00;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB08 | (0x06 as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.mem[HL_DEF as usize], 0x00);
        assert_eq!(cpu.reg.f, ZF);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn rr() {
        let mut cpu = set_up_cpu();
        // RLC B
        cpu.reg.b = 0x01;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB18 | (B_ID as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.reg.b, 0x00);
        assert_eq!(cpu.reg.f, CF + ZF);
        assert_eq!(cpu.reg.pc, pc_after);

        // RLC (HL)
        cpu.mem[HL_DEF as usize] = 0x8A;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB18 | (0x06 as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.mem[HL_DEF as usize], 0x45);
        assert_eq!(cpu.reg.f, 0);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn sla() {
        let mut cpu = set_up_cpu();
        // SLA D
        cpu.reg.d = 0x80;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB20 | (D_ID as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.reg.d, 0x00);
        assert_eq!(cpu.reg.f, CF + ZF);
        assert_eq!(cpu.reg.pc, pc_after);

        // SLA (HL)
        cpu.mem[HL_DEF as usize] = 0xFF;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB20 | (0x06 as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.mem[HL_DEF as usize], 0xFE);
        assert_eq!(cpu.reg.f, CF);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn sra() {
        let mut cpu = set_up_cpu();
        // SRA A
        cpu.reg.a = 0x8A;
        cpu.reset_flag(CF); // C = 0
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB28 | (A_ID as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0xC5);
        assert_eq!(cpu.reg.f, 0);
        assert_eq!(cpu.reg.pc, pc_after);
        
        // SRA (HL)
        cpu.mem[HL_DEF as usize] = 0x01;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB28 | (0x06 as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.mem[HL_DEF as usize], 0x00);
        assert_eq!(cpu.reg.f, CF + ZF);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn srl() {
        let mut cpu = set_up_cpu();
        // SRL A
        cpu.reg.a = 0x01;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB38 | (A_ID as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0x00);
        assert_eq!(cpu.reg.f, CF + ZF);
        assert_eq!(cpu.reg.pc, pc_after);

        // SRL(HL)
        cpu.mem[HL_DEF as usize] = 0xFF;
        cpu.reset_flag(CF);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB38 | (0x06 as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.mem[HL_DEF as usize], 0x7F);
        assert_eq!(cpu.reg.f, CF);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn swap() {
        let mut cpu = set_up_cpu();
        // SWAP A
        cpu.reg.a = 0x0F;
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB30 | (A_ID as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.reg.a, 0xF0);
        assert_eq!(cpu.reg.f, 0x00);
        assert_eq!(cpu.reg.pc, pc_after);

        // SWAP (HL)
        cpu.mem[HL_DEF as usize] = 0xF0;
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0xCB30 | (0x06 as u16));
        cpu.execute_opcode();
        assert_eq!(cpu.mem[HL_DEF as usize], 0x0F);
        assert_eq!(cpu.reg.f, 0x00);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn bit_b_r() {
        let mut cpu = set_up_cpu();

        // BIT 7, A
        cpu.reg.a = 0x80;
        cpu.set_hcnz(false, false, false, false);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0b11_001_011_01_111_111);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.f, 0b0010_0000);
        assert_eq!(cpu.reg.pc, pc_after);

        // BIT 4, L
        cpu.reg.l = 0xEF;
        cpu.set_hcnz(false, false, false, false);
        let pc_after = cpu.reg.pc + 2;
        set_2byte_op(&mut cpu, 0b11_001_011_01_100_101);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.f, 0b1010_0000);
        assert_eq!(cpu.reg.pc, pc_after);
    }

    #[test]
    fn bit_b_hl() {
        let mut cpu = set_up_cpu();

        // BIT 0, HL
        cpu.mem[HL_DEF as usize] = 0xFE;
        cpu.set_hcnz(false, false, false, false);
        set_2byte_op(&mut cpu, 0b11_001_011_01_000_110);
        cpu.bit_b_hl();
        assert_eq!(cpu.reg.f, 0b1010_0000);

        // BIT 1, HL
        cpu.mem[HL_DEF as usize] = 0xFE;
        cpu.set_hcnz(false, false, false, false);
        set_2byte_op(&mut cpu, 0b11_001_011_01_001_110);
        cpu.bit_b_hl();
        assert_eq!(cpu.reg.f, 0b0010_0000);
    }

    #[test]
    fn set_b_r() {
        let mut cpu = set_up_cpu();

        // SET 3, A
        cpu.reg.a = 0b1000_0000;
        set_2byte_op(&mut cpu, 0b11_001_011_11_011_111);
        cpu.set_b_r();
        assert_eq!(cpu.reg.a, 0b1000_1000);

        // SET 7, L
        cpu.reg.l = 0b0011_1011;
        set_2byte_op(&mut cpu, 0b11_001_011_01_111_101);
        cpu.set_b_r();
        assert_eq!(cpu.reg.l, 0b1011_1011);

        // SET 7, B
        cpu.reg.b = 0b1111_1011;
        set_2byte_op(&mut cpu, 0b11_001_011_01_111_000);
        cpu.set_b_r();
        assert_eq!(cpu.reg.b, 0b1111_1011);
    }

    #[test]
    fn set_res_bhl() {
        let mut cpu = set_up_cpu();

        // SET 3, HL
        cpu.mem[HL_DEF as usize] = 0b1001_0010;
        set_2byte_op(&mut cpu, 0b11_001_011_11_011_110);
        cpu.set_b_hl();
        assert_eq!(cpu.mem[HL_DEF as usize], 0b1001_1010);

        // SET 1, HL
        cpu.mem[HL_DEF as usize] = 0b1001_0010;
        set_2byte_op(&mut cpu, 0b11_001_011_11_001_110);
        cpu.set_b_hl();
        assert_eq!(cpu.mem[HL_DEF as usize], 0b1001_0010);

        // RES 3, HL
        cpu.mem[HL_DEF as usize] = 0b1001_0010;
        set_2byte_op(&mut cpu, 0b11_001_011_11_011_110);
        cpu.res_b_hl();
        assert_eq!(cpu.mem[HL_DEF as usize], 0b1001_0010);

        // RES 1, HL
        cpu.mem[HL_DEF as usize] = 0b1001_0010;
        set_2byte_op(&mut cpu, 0b11_001_011_11_001_110);
        cpu.res_b_hl();
        assert_eq!(cpu.mem[HL_DEF as usize], 0b1001_0000);
    }

    #[test]
    fn res_b_r() {
        let mut cpu = set_up_cpu();

        // RES 3, A
        cpu.reg.a = 0b1000_0000;
        set_2byte_op(&mut cpu, 0b11_001_011_01_011_111);
        cpu.res_b_r();
        assert_eq!(cpu.reg.a, 0b1000_0000);

        // RES 7, L
        cpu.reg.l = 0b0011_1011;
        set_2byte_op(&mut cpu, 0b11_001_011_01_111_101);
        cpu.res_b_r();
        assert_eq!(cpu.reg.l, 0b0011_1011);

        // RES 7, B
        cpu.reg.b = 0b1111_1011;
        set_2byte_op(&mut cpu, 0b11_001_011_01_111_000);
        cpu.res_b_r();
        assert_eq!(cpu.reg.b, 0b0111_1011);
    }

    // Tests for 2.8
    #[test]
    fn call_nn() {
        let mut cpu = set_up_cpu();
        cpu.reg.pc = 0x8000;
        cpu.reg.sp = 0xFFFE;
        set_3byte_op(&mut cpu, 0x00CD_3412);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.pc, 0x1234);
        assert_eq!(cpu.reg.sp, 0xFFFC);
        assert_eq!(cpu.stack[0xFFFD], 0x80);
        assert_eq!(cpu.stack[0xFFFC], 0x03);
    }

    #[test]
    fn call_cc_nn() {
        let mut cpu = set_up_cpu();
        cpu.set_flag(ZF); // ZF = 1
        cpu.reg.pc = 0x7FFD;
        cpu.reg.sp = 0xFFFE;
        // Let cc be wrong
        let cc: u8 = 00;// corresponding to Z=0 => increment to next instr
        set_3byte_op(&mut cpu, 0b11_000_100_00110100_00010010 | ((cc as u32) << 19)); // nn = 0x1234
        cpu.execute_opcode();
        assert_eq!(cpu.reg.pc, 0x8000);
        assert_eq!(cpu.reg.sp, 0xFFFE);
        
        // let cc be true
        cpu.set_flag(ZF); // ZF = 1
        let cc: u8 = 01; // Corresponding to Z=1 => jump
        set_3byte_op(&mut cpu, 0b11_000_100_00110100_00010010 | ((cc as u32) << 19)); // nn = 0x1234
        cpu.execute_opcode();
        assert_eq!(cpu.reg.pc, 0x1234);
        assert_eq!(cpu.reg.sp, 0xFFFC);
        assert_eq!(cpu.stack[0xFFFD], 0x80);
        assert_eq!(cpu.stack[0xFFFC], 0x03);
    }

    #[test]
    fn ret() {
        let mut cpu = set_up_cpu();
        cpu.reg.pc = 0x8000;
        cpu.reg.sp = 0xFFFE;
        set_3byte_op(&mut cpu, 0x00CD_0090);
        cpu.execute_opcode(); // call 0x9000
        set_1byte_op(&mut cpu, 0b1100_1001);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.pc, 0x8003);
        assert_eq!(cpu.reg.sp, 0xFFFE);
    }

    #[test]
    fn reti() {
        let mut cpu = set_up_cpu();
        cpu.reg.pc = 0x8000;
        cpu.reg.sp = 0xFFFE;
        set_3byte_op(&mut cpu, 0x00CD_0090);
        cpu.execute_opcode(); // call 0x9000. Now stack has 0x8003
        set_1byte_op(&mut cpu, 0b1101_1001);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.pc, 0x8003);
        assert_eq!(cpu.reg.sp, 0xFFFE);
    }

    #[test]
    fn ret_cc() {
        // cc wrong
        let mut cpu = set_up_cpu();
        cpu.reg.pc = 0x8000;
        cpu.reg.sp = 0xFFFE;
        cpu.set_flag(ZF); // z = 1
        let cc: u8 = 00; // jump if z = 0
        set_3byte_op(&mut cpu, 0x00CD_0090);
        cpu.execute_opcode(); // call 0x9000
        set_1byte_op(&mut cpu, 0b1100_0000 | cc << 3);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.pc, 0x9001);
        assert_eq!(cpu.reg.sp, 0xFFFC);
        
        // cc true
        cpu.reset_flag(ZF); // z = 0
        set_1byte_op(&mut cpu, 0b1100_0000 | cc << 3); // set up for ret_cc()
        cpu.execute_opcode();
        assert_eq!(cpu.reg.pc, 0x8003);
        assert_eq!(cpu.reg.sp, 0xFFFE);
    }

    #[test]
    fn rst_n() {
        let mut cpu = set_up_cpu();
        for t in 0b000..= 0b111 { // test all possible ts
            cpu.reg.pc = 0x8000;
            cpu.reg.sp = 0xFFFE;
            set_1byte_op(&mut cpu, 0b1100_0111 | (t << 3));        
            cpu.execute_opcode();
            assert_eq!(cpu.reg.sp, 0xFFFC);
            assert_eq!(cpu.stack[cpu.reg.sp as usize], 0x01);
            assert_eq!(cpu.stack[(cpu.reg.sp+1) as usize], 0x80);
            match t {
                0b000 => assert_eq!(cpu.reg.pc, 0x0000),
                0b001 => assert_eq!(cpu.reg.pc, 0x0008),
                0b010 => assert_eq!(cpu.reg.pc, 0x0010),
                0b011 => assert_eq!(cpu.reg.pc, 0x0018),
                0b100 => assert_eq!(cpu.reg.pc, 0x0020),
                0b101 => assert_eq!(cpu.reg.pc, 0x0028),
                0b110 => assert_eq!(cpu.reg.pc, 0x0030),
                0b111 => assert_eq!(cpu.reg.pc, 0x0038),
                _ => assert!(false), // panic
            }
        }
    }
            


    // Tests for 2.9
    #[test]
    fn daa() {
        let mut cpu = set_up_cpu();
        cpu.reg.a = 0x7D; // skip the add step
        cpu.reset_flag(NF); // NF = 0 if addition!! Fix now
        cpu.reset_flag(CF); // C = 0 as no carry
        set_1byte_op(&mut cpu, 0x27);
        cpu.daa();
        assert_eq!(cpu.reg.a, 0x83);
        assert_eq!(cpu.reg.f & CF, 0);
    }
       
    #[test]
    fn cpl() {
        let mut cpu = set_up_cpu();
        cpu.reg.a = 0x35;
        set_1byte_op(&mut cpu, 0x2F);
        cpu.cpl();
        assert_eq!(cpu.reg.a, 0xCA);
    }

    #[test]
    fn ccf() {
        let mut cpu = set_up_cpu();
        cpu.set_flag(CF); // C = 1;
        set_1byte_op(&mut cpu, 0x3F);
        cpu.ccf();
        assert_eq!(cpu.reg.f & CF, 0);
    }

    #[test]
    fn scf() {
        let mut cpu = set_up_cpu();
        cpu.reset_flag(CF); // CF = 0;
        set_1byte_op(&mut cpu, 0x37);
        cpu.scf();
        assert!(cpu.reg.f & CF > 0);
    }

    #[test]
    fn di() {
        let mut cpu = set_up_cpu();
        cpu.reg.ime = true; // set ime first
        cpu.di(); // reset
        assert!(!cpu.reg.ime);
    }

    #[test]
    fn ei() {
        let mut cpu = set_up_cpu();
        cpu.reg.ime = false; // set ime first
        cpu.ei(); // reset
        assert!(cpu.reg.ime);
    }

    #[test]
    fn halt() {
        let mut cpu = set_up_cpu();
        cpu.halt_mode = false;
        cpu.halt();
        assert!(cpu.halt_mode);
    }

    #[test]
    fn stop() {
        let mut cpu = set_up_cpu();
        cpu.stop_mode = false;
        cpu.stop();
        assert!(cpu.stop_mode);
    }

    #[test]
    fn jp_nn() {
        let mut cpu = set_up_cpu();
        set_3byte_op(&mut cpu, 
            0b11000011_11011111_11010101);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.pc, 0b11010101_11011111);

        set_3byte_op(&mut cpu, 0xC30C02);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.pc, 0x020C);
    }

    // Tests for 2.7
    #[test]
    fn jp_cc_nn() {
    // 8 testcases:
    // Opcode 11_0cc_010
    // CC conditions:
        // 00: jump if Z = 0 (2 testcases each for z)
        // 01: jump if Z = 1 (2 testcases each for z)
        // 10: jump if C = 0 (2 testcases each for c)
        // 11: jump is C = 1 (2 testcases each for c)
    let mut cpu = set_up_cpu();
    // Z = 0, no jump
    cpu.reset_flag(ZF); // Z = 0
    let no_jump_pc = cpu.reg.pc + 3;
    set_3byte_op(&mut cpu, 
            0b11001010_11011111_11010101); // cc = 01
            cpu.execute_opcode();
            assert_eq!(cpu.reg.pc, no_jump_pc);
    // Z = 0, jump
    cpu.reset_flag(ZF); // Z = 0
    set_3byte_op(&mut cpu, 
            0b11000010_11011111_11010101); // cc = 00
            cpu.execute_opcode();
            assert_eq!(cpu.reg.pc, 0b11010101_11011111);
    
    // Z = 1, no jump
    cpu.set_flag(ZF); // Z = 1
    let no_jump_pc = cpu.reg.pc + 3;
    set_3byte_op(&mut cpu, 
            0b11000010_11011111_11010101); // cc = 00
            cpu.execute_opcode();
            assert_eq!(cpu.reg.pc, no_jump_pc);
    // Z = 1, jump
    cpu.set_flag(ZF); // Z = 1
    set_3byte_op(&mut cpu, 
            0b11001010_11011111_11010101); // cc = 01
            cpu.execute_opcode();
            assert_eq!(cpu.reg.pc, 0b11010101_11011111);
    
    // C = 0, no jumps
    cpu.reset_flag(CF); // C = 0
    let no_jump_pc = cpu.reg.pc + 3;
    set_3byte_op(&mut cpu, 
            0b11011010_11011111_11010101); // cc = 11
            cpu.execute_opcode();
            assert_eq!(cpu.reg.pc, no_jump_pc);
    // C = 0, jumps
    cpu.reset_flag(CF); // C = 0
    set_3byte_op(&mut cpu, 
            0b11010010_11011111_11010101); // cc = 10
            cpu.execute_opcode();
            assert_eq!(cpu.reg.pc, 0b11010101_11011111);
    
    // C = 1, no jumps
    cpu.set_flag(CF); // C = 1
    let no_jump_pc = cpu.reg.pc + 3;
    set_3byte_op(&mut cpu, 
            0b11010010_11011111_11010101); // cc = 10
            cpu.execute_opcode();
            assert_eq!(cpu.reg.pc, no_jump_pc);
    // C = 1, jumps
    cpu.set_flag(CF); // C = 1
    set_3byte_op(&mut cpu, 
            0b11011010_11011111_11010101); // cc = 11
            cpu.execute_opcode();
            assert_eq!(cpu.reg.pc, 0b11010101_11011111);
    
    }

    
    #[test]
    fn jr_e() {
        let mut cpu = set_up_cpu();
        cpu.reg.pc = 0x8000;

        // offset: -128 (+2)
        cpu.reg.pc = 0x8000;
        set_2byte_op(&mut cpu, 0b00011000_10000000);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.pc, 0x8000 - 126);

        // offset: 0 (+2)
        cpu.reg.pc = 0x8000;
        set_2byte_op(&mut cpu, 0b00011000_00000000);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.pc, 0x8000 + 2);

        // offset: +127 (+2)
        cpu.reg.pc = 0x8000;
        set_2byte_op(&mut cpu, 0b00011000_01111111);
        cpu.execute_opcode();
        assert_eq!(cpu.reg.pc, 0x8000 + 129);
    }
}
