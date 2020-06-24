// Flags
const ZF: u8 = 0x80; // 0b10000000
const NF: u8 = 0x40; // 0b01000000
const HF: u8 = 0x20; // 0b00100000
const CF: u8 = 0x10; // 0b00010000

// 8-bit Register IDs
const A_ID: u8 = 0b111;
const B_ID: u8 = 0b000;
const C_ID: u8 = 0b001;
const D_ID: u8 = 0b101;
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
pub struct Registers {
    pub mut A: u8,      // Accumulator register
    pub mut B: u8,
    pub mut C: u8,
    pub mut D: u8,
    pub mut E: u8,
    pub mut H: u8,
    pub mut L: u8,

    // 16-bit pair registers
    pub mut BC: u16,
    pub mut DE: u16,
    pub mut HL: u16,

    // Special registers
    pub mut F: u8,      // Special flag register
    pub mut SP: u16,    // Stack pointer. SP will start at 65536
    pub mut PC: u16,

    // Registers for interrupt. Each of these is only used for 1 bit, maybe can combine to become
    // like register F
    pub mut IE: u8, 
    pub mut IF: u8,
    pub mut IME: u8,    // Enable / Disable all interrupts
}

pub struct CPU {
    pub mut reg: Registers,     // Set of registers
    
    pub mut mem: [u8; 65536],   // 64KB memory
    pub mut stack: [u8; 65536],    // Stack for PC

    pub mut clock: u8,          // For timing in GB
}

impl CPU {
    /*
    pub fn initialize() -> Self {
        // Initializing a Gameboy CPU (initial state)
    }

    pub fn tick() {
        // For when CPU runs
    }

    pub fn read_opcode() {
        // Obtain opcode
    }

    pub fn run_opcode(opcode: u8) {
        // Let it run boi
    }
    */

    // Some reusable code (for opcodes)
    
    /// write_to_r8: write content to appropriate 8-bit register based on register ID.
    /// @param r8_id: ID of register
    /// @param content: content to write to register
    /// returns boolean to indicate ID is valid.
    pub fn write_to_r8(&self, r8_id: u8, content: u8) -> bool {
        match r8_id {
            A_ID => self.reg.A = content,
            B_ID => self.reg.B = content,
            C_ID => self.reg.C = content,
            D_ID => self.reg.D = content,
            E_ID => self.reg.E = content,
            H_ID => self.reg.H = content,
            L_ID => self.reg.L = content,
            .. => return false;
        }

        true
    }

    /// read_from_r8: Read data from the appropriate register.
    /// @param r8_id: ID of 8-bit register
    /// @return Option<u8>. returns None if r8_id is invalid or register is empty.
    pub fn read_from_r8(&self, r8_id: u8) -> Option<u8> {
        let result: u8;
        
        match r8_id {
            A_ID => result = self.reg.A,
            B_ID => result = self.reg.B,
            C_ID => result = self.reg.C,
            D_ID => result = self.reg.D,
            E_ID => result = self.reg.E,
            H_ID => result = self.reg.H,
            L_ID => result = self.reg.L,
            .. => return None,
        }

        Some(result)
    }
    
    /// load_mem_to_r8: Loads content from memory specified by addr into register r8_id.
    /// @param r8_id: ID of some 8-bit register
    /// @param addr: 16-bit address for memory
    /// @return boolean whether ID is valid
    pub fn load_mem_to_r8(&self, r8_id: u8, addr: u16) -> bool{
        self.write_to_r8(r8_id, self.mem[addr as usize])
    }

    /// save_r8_to_mem: Saves content from register r8_id into memory specified by addr.
    /// @param r8_id: ID of some 8-bit register with content
    /// @param addr: 16-bit address for memory to be saved to
    pub fn save_r8_to_mem(&self, r8_id: u8, addr: u16) {
        match self.read_from_r8(r8_id) {
            Some(content) => self.mem[addr as usize] = content,
            None => (),
        }
    }

    /// get_n: gets 8-bit immediate n right after opcode
    pub fn get_n(&self) -> u8 {
        self.mem[(self.reg.PC + 1) as usize]
    }

    /// get_r8_to: gets 3-bit register ID from opcode. Register ID takes bit 3, 4, 5 for register
    /// written to.
    pub fn get_r8_to(&self) -> u8 {
        ((self.mem[self.reg.PC as usize] & 0b00111000) >> 3) as u8
    }
    
    /// get_r8_from: gets 3-bit register ID from opcode. Register ID takes bit 0,1,2 for register
    /// written to.
    pub fn get_r8_from(&self) -> u8 {
        (self.mem[self.reg.PC as usize] & 0b00000111) as u8
    }

    /// write_to_r16: Write content onto a 16-byte register.
    /// @param r16_id: ID of 16-byte reg
    /// @param content: content to be written
    /// @return bool value if ID was valid.
    pub fn write_to_r16(&self, r16_id: u8, content: u16) -> boolean {
        let msb = content >> 8 as u8;
        let lsb = content & 0x00FF as u8;

        match r16_id {
            BC_ID => {
                self.reg.BC = content;
                self.reg.B = msb;
                self.reg.C = lsb;
            },
            DE_ID => {
                self.reg.DE = content;
                self.reg.D = msb;
                self.reg.E = lsb;
            },
            HL_ID => {
                self.reg.HL = content;
                self.reg.H = msb;
                self.reg.L = lsb;
            },
            SP_ID => self.reg.SP = content,
            .. => return false;
        }

        true
    }

    /// read_from_r16: reads content of a 16-bit register.
    /// @param r16_id: ID of a 16-byte register.
    /// @return Some<u16> if ID is valid, None if not valid.
    pub fn read_from_r16(&self, r16_id: u8) -> Option<u16> {
        let result: u16;

        match r16_id {
            BC_ID => result = self.reg.BC,
            DE_ID => result = self.reg.DE,
            HL_ID => result = self.reg.HL,
            SP_ID => result = self.reg.SP,
            .. => return None,
        }

        Some(result)
    }

    /// save_r16_to_mem: Saves lower-byte of 16-bit register to addr, and higher-byte to addr + 1.
    /// @param r16_id: ID of 16-byte register.
    /// @param addr: address to write content to.
    pub fn save_r16_to_mem(&self, r16_id: u8, addr: u16) {
        match read_from_r16(r16_id) {
            Some(value) => {
                self.mem[addr as usize] = (value & 0x00FF) as u8;
                self.mem[(addr + 1)  as usize] = (value >> 8) as u8;
            },
            None => (),
        }
    }

    /// get_nn: gets 16-bit immediate nn right after opcode
    pub fn get_nn(&self) -> u16 {
        let nn_low = self.mem[(self.reg.PC + 1) as usize];
        let nn_high = self.mem[(self.reg.PC + 2) as usize];
        let nn = (nn_high << 8) | nn_low; 

        nn
    }

    pub fn get_r16(&self) -> u8 {
        ((self.mem[self.reg.PC as usize] & 0b00110000) >> 4) as u8
    }

    // Reusable code for 8-bit Rotate, Shift instructions
    
    pub fn set_flag(&self, flag: u8) {
        self.reg.F = self.reg.F | flag;
    }

    pub fn reset_flag(&self, flag: u8) {
        match flag {
            ZF => self.reg.F &= 0b01111111,
            NF => self.reg.F &= 0b10111111,
            HF => self.reg.F &= 0b11011111,
            CF => self.reg.F &= 0b11101111,
            .. => (),
        }
    }

    /// rotate_r8: Rotate function for 8-bit registers. Toggle between lpeft or right using bool
    /// is_rotate_left.
    /// There are 2 types of rotate operations: Has carry or no carry.
    /// If operation has carry: bit A7 is copied to flag CY AND bit 0 of A.
    /// If operation has no carry: bit 0 of A is replaced by CY, and then bit A7 is copied to CY 
    /// after rotation, write data back to register.
    
    pub fn rotate_r8(&self, r8_id: u8, is_rotate_left: bool, has_carry: bool) {
        let mut data: u8;
        let mut c: bool;

        match self.read_from_r8(r8_id) {
            Some(value) => data = value,
            None => return (),
        }

        let bit_cf: u8 = (self.reg.F & CF) >> 4;

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
    
    pub fn rotate_mem(&self, addr: u16, is_left_rotate: bool, has_carry: bool) {
        let mut data = self.mem[addr as usize];
        let mut c = bool;
        let bit_cf = (self.reg.F & CF) >> 4;
    
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

        self.mem[addr] = data; // write back to memory

        // setting cf to bit_a7
        self.set_hcnz(false, c, false, data == 0);
    }

    pub fn write_a(&self, to_write: u8) {
    	self.write_to_r8(A_ID, to_write);
    }

    pub fn set_hcnz(&self, h: bool, c: bool, n: bool, z: bool) {
	    if h {self.set_flag(H_ID)} else {self.reset_flag(H_ID)};
	    if c {self.set_flag(C_ID)} else {self.reset_flag(C_ID)};
	    if n {self.set_flag(N_ID)} else {self.reset_flag(N_ID)};
	    if z {self.set_flag(Z_ID)} else {self.reset_flag(Z_ID)};
	}

	pub fn set_hnz(&self, h: bool, c: bool, n: bool, z: bool) {
	    if h {self.set_flag(H_ID)} else {self.reset_flag(H_ID)};
	    if n {self.set_flag(N_ID)} else {self.reset_flag(N_ID)};
	    if z {self.set_flag(Z_ID)} else {self.reset_flag(Z_ID)};
	}
    
    /// check_cc extracts condition cc from opcode, and check whether condition is true.
    /// cc is a 2-bit number, at bit 3 and 4 of opcode, representing:
    /// 00 -> Z == 0; 01 -> Z == 1; 10 -> C == 0; 11 -> C == 1
    pub fn check_cc(&self) -> bool {
        // extract cc from opcode
        let opcode = self.mem[self.reg.PC];
        let cc: u8 = (opcode & 0b00011000) >> 3;
        let mut result: bool;
        
        // match cc with respective outcomes
        match cc {
            00 => result = self.reg.F & ZF == 0,
            01 => result = self.reg.F & ZF != 0,
            10 => result = self.reg.F & CF == 0,
            11 => result = self.reg.F & CF != 0,
        }

        result
    }
   
    /// push_u16: push a u16 value onto the stack.
    /// Most significant byte (MSB) goes to SP - 1
    /// Least significant byte (LSB)  goes to SP - 2
    pub fn push_u16(&self, val: u16) {
        self.stack[(self.reg.SP - 1) as usize] = (val >> 8) as u8; // most sig. byte
        self.stack[(self.reg.SP - 2) as usize] = (val & 0x00FF) as u8; // least sig. byte.

        self.reg.SP -= 2;
    }

    /// pop_u16: pop a u16 value off the stack and return it.
    /// LSB is at SP. MSB is at SP + 1. After that, increment SP by 2
    pub fn pop_u16(&self) -> u16 {
        let lsb = self.stack[self.reg.SP as usize] as u16;
        let msb = self.stack[(self.reg.SP + 1) as usize] as u16;

        self.reg.SP += 2;

        (msb << 8) | lsb
    }

    // Opcodes goes here!!
    
    // 8-bit load instructions
    
    /// ld_rx_ry: load contents of ry to rx. 1-byte instruction
    /// @param rx, ry: ID for register rx and ry (8-bit)
    pub fn ld_rx_ry(&self) -> ProgramCounter {
        let rx = self.get_r8_to();
        let ry =  self.get_r8_from();

        match self.read_from_r8(ry) {
            Some(value) => self.write_to_r8(rx),
            None => (),
        }

        ProgramCounter::Next(1)
    }

    /// ld_r_n: Load 8-bit data n into register r. 2-byte instruction
    /// @param: r: register ID; n: intermediate
    pub fn ld_r_n(&self) -> ProgramCounter {
        let r = self.get_r8_to();
        let n = self.get_n();

        self.write_to_r8(r, n);

        ProgramCounter::Next(2)
    }

    /// ld_r_addr_HL: loads contents of memory specified at (HL) to register r. 1-byte instruction
    /// @param r: 8-bit register ID
    pub fn ld_r_addr_HL(&self) -> ProgramCounter {
        let r = self.get_r8_to();

        self.load_mem_to_r8(r, self.reg.HL);

        ProgramCounter::Next(1)
    }

    /// ld_addr_HL_r: stores contents of register r into memory specified by register pair HL.
    /// 1-byte instruction.
    /// @param: r: ID of 8-bit register
    pub fn ld_addr_HL_r(&self) -> ProgramCounter {
        let r = self.get_r8_from();
    
        self.save_r8_to_mem(r, self.reg.HL);
        
        ProgramCounter::Next(1)
    }

    /// ld_addr_HL_n: stores 8-bit immediate data in memory specified by register pair HL.
    /// 2-byte instruction.
    /// @param n: 8-bit immediate.
    pub fn ld_addr_HL_n(&self) -> ProgramCounter {
        let n = self.get_n();

        self.mem[self.reg.HL as usize] = n;

        ProgramCounter::Next(2)
    }

    /// ld_A_addr_BC: Load contents of memory specified by BC into A.
    /// 1-byte instruction
    pub fn ld_A_addr_BC(&self) -> ProgramCounter {
        self.load_mem_to_r8(A_ID, self.reg.BC);

        ProgramCounter::Next(1)
    }

    /// ld_A_addr_DE: Load contents of memory specified by DE into A.
    /// 1-byte instruction
    pub fn ld_A_addr_DE(&self) -> ProgramCounter {
        self.load_mem_to_r8(A_ID, self.reg.DE);

        ProgramCounter::Next(1)
    }

    /// ld_A_addr_offset_C: Load contents of memory specified by C + 0xFF00 into A.
    /// 1-byte instruction
    pub fn ld_A_addr_offset_C(&self) -> ProgramCounter {
        self.load_mem_to_r8(A_ID, (0xFF00 + self.reg.C));

        ProgramCounter::Next(1)
    }

    /// ld_addr_offset_C_A: Load contents of A into memory specified by 0xFF00 + C.
    /// 1-byte instruction
    pub fn ld_addr_offset_C_A(&self) -> ProgramCounter {
        self.save_r8_to_mem(A_ID, (0xFF00 + self.reg.C));

        ProgramCounter::Next(1)
    }

    /// ld_A_addr_offset_nn: Load contents of memory specified by nn + 0xFF00 into A.
    /// 1-byte instruction
    pub fn ld_A_addr_offset_n(&self) -> ProgramCounter {
        let n = self.get_n();

        self.load_mem_to_r8(A_ID, (0xFF00 + n));
        
        ProgramCounter::Next(2)
    }
    
    /// ld_addr_offset_n_A: Load contents of A into memory specified by 0xFF00 + n.
    /// 1-byte instruction
    pub fn ld_addr_offset_n_A(&self) -> ProgramCounter {
        let n = self.get_n();

        self.save_r8_to_mem(A_ID, (0xFF00 + n));

        ProgramCounter::Next(2)
    }

    /// ld_A_addr_nn: Load content at memory specified by address nn into register A.
    /// 3-byte instruction.
    /// @param nn: 16-bit address
    pub fn ld_A_addr_nn(&self) -> ProgramCounter {
        let nn = self.get_nn;

        self.load_mem_to_r8(A_ID, nn);

        ProgramCounter::Next(3)
    }

    /// ld_addr_nn_A: Save content of register A into memory specified by address nn.
    /// 3-byte instruction.
    /// @param nn: 16-bit address.
    pub fn ld_addr_nn_A(&self) -> ProgramCounter {
        let nn = self.get_nn();

        self.save_r8_to_mem(A_ID, nn);
    
        ProgramCounter::Next(3)
    } 

    /// ld_A_addr_HL_inc: Load content of memory specified by HL into register A, then increment
    /// content in HL.
    /// 1-byte instruction.
    pub fn ld_A_addr_HL_inc(&self) -> ProgramCounter {
        self.load_mem_to_r8(A_ID, self.reg.HL);
        HL += 1;

        ProgramCounter::Next(1)
    }

    /// ld_A_addr_HL_dec: Load content of memory specified by HL into register A, then deccrement
    /// content in HL.
    /// 1-byte instruction.
    pub fn ld_A_addr_HL_dec(&self) -> ProgramCounter {
        self.load_mem_to_r8(A_ID, self.reg.HL);
        HL -= 1;

        ProgramCounter::Next(1)
    }

    /// ld_addr_BC_A: Save content of register A to memory specified by BC.
    /// 1-byte instruction
    pub fn ld_addr_BC_A(&self) -> ProgramCounter {
        self.save_r8_to_mem(A_ID, self.reg.BC);
    }

    /// ld_addr_DE_A: Save content of register A to memory specified by DE.
    /// 1-byte instruction
    pub fn ld_addr_DE_A(&self) -> ProgramCounter {
        self.save_r8_to_mem(A_ID, self.reg.DE);
    }

    /// ld_addr_HL_A_inc: Load content of register A into memory specified by HL, then increment
    /// content in HL.
    /// 1-byte instruction.
    pub fn ld_A_addr_HL_inc(&self) -> ProgramCounter {
        self.save_r8_to_mem(A_ID, self.reg.HL);
        HL += 1;

        ProgramCounter::Next(1)
    }

    /// ld_addr_HL_A_dec: Load content of register A into memory specified by HL, then deccrement
    /// content in HL.
    /// 1-byte instruction.
    pub fn ld_A_addr_HL_dec(&self) -> ProgramCounter {
        self.save_r8_to_mem(A_ID, self.reg.HL);
        HL -= 1;

        ProgramCounter::Next(1)
    }

    // 16-bit load instructions
    
    /// ld_rr_nn: load 16-bit immediate nn to 16-bit register rr.
    /// 3-byte instruction
    /// @param rr: ID of 16-bit instruction
    pub fn ld_rr_nn(&self) -> ProgramCounter {
        let rr = self.get_r16();
        let nn = self.get_nn();
        
        self.write_to_r16(rr, nn);

        ProgramCounter::Next(3)
    }

    /// ld_addr_nn_SP: load lower-byte of SP to (nn), load higher-byte of SP to (nn+1)
    /// 3-byte instruction
    pub fn ld_addr_nn_SP(&self) -> ProgramCounter {
        let nn = self.get_nn();

        save_r16_to_mem(SP_ID, nn);

        ProgramCounter::Next(3)
    }

    /// ld_SP_HL: load data from HL register to SP register.
    /// 1-byte instruction
    pub fn ld_SP_HL(&self) -> ProgramCounter {
        self.reg.SP = self.reg.HL;

        ProgramCounter::Next(1)
    }

    /// push_rr: push data from register rr to stack memory
    /// 1-byte instruction
    pub fn push_rr(&self) -> ProgramCounter {
        let rr = self.get_r16();
        let val = read_from_r16(rr)?;

        self.push_u16(val);

        ProgramCounter::Next(1)
    }

    /// pop_rr: pop data from stack to the 16-bit register rr.
    /// 1-byte instruction
    pub fn pop_rr(&self) -> ProgramCounter {
        let rr = self.get_r16();
        
        self.write_to_r16(rr, self.pop_u16());

        ProgramCounter::Next(1)
    }


    // 8 Bit Arithmetic Operation Instruction
    // ADD A,r: Add the value in register r to A, set it to A. 
    // Cycles: 1
    pub fn add_ar(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx)?;

	    // processing
	    let res: u16 = (a as u16) + (r as u16);

	    // flags and writing
	    let h: bool = ((a & 0x0F) + (r & 0x0F)) > 0x0F;
	    let c: bool = res > 0xFF;
	    let n: bool = false;
	    let to_write: u8 = (a & 0xFF) as u8;
	    let z: bool = to_write == 0;

	    self.write_a(to_write);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
	}

	// ADD A, n: add immediate operand n to register A.
	// Cycles: 2
	pub fn add_an(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let r: u8 = self.get_n();

	    // processing
	    let res: u16 = (a as u16) + (r as u16);

	    // flags and writing
	    let h: bool = ((a & 0x0F) + (r & 0x0F)) > 0x0F;
	    let c: bool = res > 0xFF;
	    let n: bool = false;
	    let to_write: u8 = (a & 0xFF) as u8;
	    let z: bool = to_write == 0;

	    self.write_a(to_write);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(2)
	}

    pub fn add_ahl(&self) -> ProgramCounter {
        // reading
        let a: u8 = self.read_from_r8(A_ID)?;
        let r: u8 = self.mem[self.reg.HL as usize];

        // processing
        let res: u16 = (a as u16) + (r as u16);

        // flags and writing
	    let h: bool = ((a & 0x0F) + (r & 0x0F)) > 0x0F;
	    let c: bool = res > 0xFF;
	    let n: bool = false;
	    let to_write: u8 = (a & 0xFF) as u8;
	    let z: bool = to_write == 0;

	    self.write_a(to_write);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
    }
        
    pub fn adc_ar(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let carry: u8 = self.read_from_r8(C_ID)?;
	    let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx)?;

	    // processing
	    let res: u16 = (a as u16) + (r as u16) + (carry as u16);

	    // flags and writing
	    let h: bool = ((a & 0x0F) + (r & 0x0F) + (carry & 0x0F)) > 0x0F;
	    let c: bool = res > 0xFF;
	    let n: bool = false;
	    let to_write: u8 = (a & 0xFF) as u8;
	    let z: bool = to_write == 0;

	    self.write_a(to_write);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
	}

	// ADD A, n: add immediate operand n to register A.
	// Cycles: 2
	pub fn adc_an(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let carry: u8 = self.read_from_r8(C_ID)?;
	    let r: u8 = self.get_n();

	    // processing
	    let res: u16 = (a as u16) + (r as u16) + (carry as u16);

	    // flags and writing
	    let h: bool = ((a & 0x0F) + (r & 0x0F) + (carry & 0x0F)) > 0x0F;
	    let c: bool = res > 0xFF;
	    let n: bool = false;
	    let to_write: u8 = (a & 0xFF) as u8;
	    let z: bool = to_write == 0;

	    self.write_a(to_write);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(2)
	}

    pub fn adc_ahl(&self) -> ProgramCounter {
        // reading
        let a: u8 = self.read_from_r8(A_ID)?;
        let carry: u8 = self.read_from_r8(C_ID)?;
        let r: u8 = self.mem[self.reg.HL as usize];

        // processing
        let res: u16 = (a as u16) + (r as u16) + (carry as u16);

        // flags and writing
	    let h: bool = ((a & 0x0F) + (r & 0x0F) + (carry & 0x0F)) > 0x0F;
	    let c: bool = res > 0xFF;
	    let n: bool = false;
	    let to_write: u8 = (a & 0xFF) as u8;
	    let z: bool = to_write == 0;

	    self.write_a(to_write);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
    }

    pub fn sub_r(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx)?;

	    // processing
	    let res: u8 = a.wrapping_sub(r);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub(r & 0x0F) == None;
	    let c: bool = (a).checked_sub(r) == None;
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
	}

	// Cycles: 2
	pub fn sub_n(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
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

	    ProgramCounter::Next(2)
	}

    pub fn sub_hl(&self) -> ProgramCounter {
        // reading
        let a: u8 = self.read_from_r8(A_ID)?;
        let r: u8 = self.mem[self.reg.HL as usize];

        // processing
	    let res: u8 = a.wrapping_sub(r);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub(r & 0x0F) == None;
	    let c: bool = (a).checked_sub(r) == None;
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
    }
        
    pub fn sbc_ar(&self) -> ProgramCounter {
	    // reading
	    let carry: u8 = self.read_from_r8(C_ID)?;
        let a: u8 = self.read_from_r8(A_ID)?;
        let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx)?;

        // processing
	    let res: u8 = a.wrapping_sub(r).wrapping_sub(carry);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub((r & 0x0F) + carry) == None;
	    let c: bool = (a as u16) < (r as u16 + carry as u16);
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
	}

	// ADD A, n: add immediate operand n to register A.
	// Cycles: 2
	pub fn sbc_an(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let carry: u8 = self.read_from_r8(C_ID)?;
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

	    ProgramCounter::Next(1)
	}

    pub fn sbc_ahl(&self) -> ProgramCounter {
        // reading
        let carry: u8 = self.read_from_r8(C_ID)?;
        let a: u8 = self.read_from_r8(A_ID)?;
        let r: u8 = self.mem[self.reg.HL as usize];

        // processing
	    let res: u8 = a.wrapping_sub(r).wrapping_sub(carry);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub((r & 0x0F) + carry) == None;
	    let c: bool = (a as u16) < (r as u16 + carry as u16);
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
    }

    pub fn and_r(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx)?;

	    // processing
	    let res: u16 = a & r;

	    // flags and writing
	    let h: bool = true;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
	}

	// ADD A, n: add immediate operand n to register A.
	// Cycles: 2
	pub fn and_n(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let r: u8 = self.get_n();

	    // processing
	    let res: u16 = a & r;

	    // flags and writing
	    let h: bool = true;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
	}

    pub fn and_hl(&self) -> ProgramCounter {
        // reading
        let a: u8 = self.read_from_r8(A_ID)?;
        let r: u8 = self.mem[self.reg.HL as usize];

        // processing
	    let res: u16 = a & r;

	    // flags and writing
	    let h: bool = true;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
    }

    pub fn or_r(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx)?;

	    // processing
	    let res: u16 = a | r;
	    // flags and writing
	    let h: bool = false;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
	}

	// ADD A, n: add immediate operand n to register A.
	// Cycles: 2
	pub fn or_n(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let r: u8 = self.get_n();

	    // processing
	    let res: u16 = a | r;

	    // flags and writing
	    let h: bool = false;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
	}

    pub fn or_hl(&self) -> ProgramCounter {
        // reading
        let a: u8 = self.read_from_r8(A_ID)?;
        let r: u8 = self.mem[self.reg.HL as usize];

        // processing
	    let res: u16 = a | r;

	    // flags and writing
	    let h: bool = false;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
    }

    pub fn xor_r(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx)?;

	    // processing
	    let res: u16 = a ^ r;

	    // flags and writing
	    let h: bool = false;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
	}

	// ADD A, n: add immediate operand n to register A.
	// Cycles: 2
	pub fn xor_n(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let r: u8 = self.get_n();

	    // processing
	    let res: u16 = a ^ r;

	    // flags and writing
	    let h: bool = false;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
	}

    pub fn xor_hl(&self) -> ProgramCounter {
        // reading
        let a: u8 = self.read_from_r8(A_ID)?;
        let r: u8 = self.mem[self.reg.HL as usize];

        // processing
	    let res: u16 = a ^ r;

	    // flags and writing
	    let h: bool = false;
	    let c: bool = false;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_a(res);
	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
    }

    pub fn cp_r(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let idx: u8 = self.get_r8_from();
	    let r: u8 = self.read_from_r8(idx)?;

	    // processing
	    let res: u8 = a.wrapping_sub(r);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub(r & 0x0F) == None;
	    let c: bool = (a).checked_sub(r) == None;
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
	}

	// Cycles: 2
	pub fn cp_n(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let r: u8 = self.get_n();

	    // processing
	    let res: u8 = a.wrapping_sub(r);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub(r & 0x0F) == None;
	    let c: bool = (a).checked_sub(r) == None;
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(2)
	}

    pub fn cp_hl(&self) -> ProgramCounter {
        // reading
        let a: u8 = self.read_from_r8(A_ID)?;
        let r: u8 = self.mem[self.reg.HL as usize];

        // processing
	    let res: u8 = a.wrapping_sub(r);

	    // flags and writing
	    let h: bool = (a & 0x0F).checked_sub(r & 0x0F) == None;
	    let c: bool = (a).checked_sub(r) == None;
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.set_hcnz(h, c, n, z);

	    ProgramCounter::Next(1)
    }

    pub fn inc_r(&self) -> ProgramCounter {
	    // reading
	    let idx: u8 = self.get_r8_to();
	    let r: u8 = self.read_from_r8(idx)?;

	    // processing
	    let res: u8 = if r == u8::MAX {0} else {r + 1};

	    // flags and writing
	    let h: bool = (r & 0xF) == 0xF;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.write_to_r8(idx);
	    self.set_hnz(h, n, z);

	    ProgramCounter::Next(1)
	}

	pub fn inc_hl(&self) -> ProgramCounter {
	    // reading
	    let idx: u8 = self.get_r8_to();
	    let r: u8 = self.mem[self.reg.HL as usize];

	    // processing
	    let res: u8 = if r == u8::MAX {0} else {r + 1};

	    // flags and writing
	    let h: bool = (r & 0xF) == 0xF;
	    let n: bool = false;
	    let z: bool = res == 0;

	    self.mem[self.reg.HL as usize] = res;
	    self.set_hnz(h, n, z);

	    ProgramCounter::Next(1)
	}

	pub fn dec_r(&self) -> ProgramCounter {
	    // reading
	    let idx: u8 = self.get_r8_to();
	    let r: u8 = self.read_from_r8(idx)?;

	    // processing
	    let res: u8 = if r == 0 {u8::MAX} else {r - 1};

	    // flags and writing
	    let h: bool = (r & 0xF) == 0xF;
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.write_to_r8(idx);
	    self.set_hnz(h, n, z);

	    ProgramCounter::Next(1)
	}

	pub fn dec_hl(&self) -> ProgramCounter {
	    // reading
	    let idx: u8 = self.get_r8_to();
	    let r: u8 = self.mem[self.reg.HL as usize];

	    // processing
	    let res: u8 = if r == 0 {u8::MAX} else {r - 1};

	    // flags and writing
	    let h: bool = (r & 0xF) == 0xF;
	    let n: bool = true;
	    let z: bool = res == 0;

	    self.mem[self.reg.HL as usize] = res;
	    self.set_hnz(h, n, z);

	    ProgramCounter::Next(1)
	}





    // 2.5 Shift and Rotate instructions
    
    /// rlca: Rotates content of register A to the left. a7 <- a0
    /// is_left_rotate = true, has_carry = true
    /// 1-byte instruction
    pub fn rlca(&self) -> ProgramCounter {
        self.rotate_r8(A_ID, true, true);
        
        ProgramCounter::Next(1)
    }

    /// rla: Rotates content of register A to the left. a7 <- cf
    /// is_left_rotate = true, has_carry = false
    /// 1-byte instruction.
    pub fn rla(&self) -> ProgramCounter {
        self.rotate_r8(A_ID, true, false);

        ProgramCounter::Next(1)
    }

    /// rrca: Rotates content of register A to the right. a0 <- a7
    /// is_left_rotate = false, has_carry = true.
    /// 1-byte instruction
    pub fn rrca(&self) -> ProgramCounter {
        self.rotate_r8(A_ID, false, true);

        ProgramCounter::Next(1)
    }

    /// rra: Rotates content of register A to the right. a0 <- cf
    /// is_left_rotate = false, has_carry = false.
    /// 1-byte instruction
    pub fn rra(&self) -> ProgramCounter {
        self.rotate_r8(A_ID, false, false);

        ProgramCounter::Next(1)
    }

    /// rlc: Rotates content of either some register r or memory pointed to by HL, depending on
    /// opcode. to the left, with carry.
    pub fn rlc(&self) -> ProgramCounter {
        self.reg.PC += 1;
        let r = self.get_r8_from();
        self.reg.PC -= 1;

        match r {
            0x06 => self.rotate_mem(self.reg.HL, true, true),
            .. => self.rotate_r8(r, true, true),
        }

        ProgramCounter::Next(2)
    }

    /// rl: Rotates content of either some register r or memory pointed to by HL, depending on
    /// opcode. to the left, without carry.
    pub fn rlc(&self) -> ProgramCounter {
        self.reg.PC += 1;
        let r = self.get_r8_from();
        self.reg.PC -= 1;

        match r {
            0x06 => self.rotate_mem(self.reg.HL, true, false),
            .. => self.rotate_r8(r, true, false),
        }

        ProgramCounter::Next(2)
    }
    
    /// rrc: Rotates content of either some register r or memory pointed to by HL, depending on
    /// opcode. to the right, with carry.
    pub fn rlc(&self) -> ProgramCounter {
        self.reg.PC += 1;
        let r = self.get_r8_from();
        self.reg.PC -= 1;

        match r {
            0x06 => self.rotate_mem(self.reg.HL, false, true),
            .. => self.rotate_r8(r, false, true),
        }

        ProgramCounter::Next(2)
    }

    /// rr: Rotates content of either some register r or memory pointed to by HL, depending on
    /// opcode. to the right, without carry.
    pub fn rlc(&self) -> ProgramCounter {
        self.reg.PC += 1;
        let r = self.get_r8_from();
        self.reg.PC -= 1;

        match r {
            0x06 => self.rotate_mem(self.reg.HL, false, false),
            .. => self.rotate_r8(r, false, false),
        }

        ProgramCounter::Next(2)
    }

    /// SLA: Shift content of operand m to the left. Bit 7 is copied to CF, bit 0 is reset.
    /// 2-byte instruction
    pub fn sla(&self) -> ProgramCounter {
        self.reg.PC += 1;
        let r = self.get_r8_from();
        self.reg.PC -= 1;

        let mut data: u8;
        let mut bit_7: u8;

        match r {
            0x06 => {
                data = self.mem[self.reg.HL as usize];
                bit_7 = (data & 0x80) >> 7;
                
                // processing
                data = data << 1;
                
                // write back
                self.mem[self.reg.HL as usize] = data;
            },
            .. => {
                data = read_from_r8(r)?;
                bit_7 = (data & 0x80) >> 7;
                
                // processing
                data = data << 1;
                
                // write back
                self.write_to_r8(r, data);
            },
        }

        // set flags
        self.set_hcnz(false, bit_7 > 0, false, data == 0);

        ProgramCounter::Next(2)
    }
        
    /// SRA: Shift content of operand m to the right. Bit 0 is copied to CF, bit 7 stays the same!.
    /// 2-byte instruction
    pub fn sra(&self) -> ProgramCounter {
        self.reg.PC += 1;
        let r = self.get_r8_from();
        self.reg.PC -= 1;

        let mut data: u8;
        let mut bit_0: u8;
        let mut bit_7: u8;

        match r {
            0x06 => {
                data = self.mem[self.reg.HL as usize];
                bit_7 = (data & 0x80) >> 7;
                bit_0 = (data & 0x01);
                
                // processing
                data = data >> 1;
                data |= (bit_7 << 7)
                
                // write back
                self.mem[self.reg.HL as usize] = data;
            },
            .. => {
                data = read_from_r8(r)?;
                bit_7 = (data & 0x80) >> 7;
                bit_0 = (data & 0x01);
                
                // processing
                data = data << 1;
                data |= (bit_7 << 7);

                // write back
                self.write_to_r8(r, data);
            },
        }

        // set flags
        self.set_hcnz(false, bit_0 > 0, false, data == 0);

        ProgramCounter::Next(2)
    }

    /// SRL: Shift content of operand m to the right. Bit 0 is copied to CF, bit 7 is reset.
    /// 2-byte instruction
    pub fn srl(&self) -> ProgramCounter {
        self.reg.PC += 1;
        let r = self.get_r8_from();
        self.reg.PC -= 1;

        let mut data: u8;
        let mut bit_0: u8;

        match r {
            0x06 => {
                data = self.mem[self.reg.HL as usize];
                bit_0 = (data & 0x01);
                
                // processing
                data = data >> 1;
                
                // write back
                self.mem[self.reg.HL as usize] = data;
            },
            .. => {
                data = read_from_r8(r)?;
                bit_0 = (data & 0x01);
                
                // processing
                data = data << 1;

                // write back
                self.write_to_r8(r, data);
            },
        }

        // set flags
        self.set_hcnz(false, bit_0 > 0, false, data == 0);

        ProgramCounter::Next(2)
    }

    /// SWAP: Shift content of lower-order 4 bits to higher-order 4 bits, and vice versa. Reset all
    /// flags except ZF.
    /// 2-byte instruction.
    pub fn swap(&self) -> ProgramCounter {
        self.reg.PC += 1;
        let r = self.get_r8_from();
        self.reg.PC -= 1;

        let mut data: u8;
       
        match r {
            0x06 => {
                // read
                data = self.mem[self.reg.HL as usize];
                
                // process
                let lower = data & 0x0F;
                let higher = (data & 0xF0) >> 4;
                data = (lower << 4) | higher;

                // write back
                self.mem[self.reg.HL as usize] = data;
            },
            .. => {
                // read
                data = self.read_from_r8(r)?;

                // process
                let lower = data & 0x0F;
                let higher = (data & 0xF0) >> 4;
                data = (lower << 4) | higher;
                
                // write back
                self.write_to_r8(r, data);
            }
        }
        self.set_hcnz(false, false, false, data == 0);
        
        ProgramCounter::Next(2)
    }

    // 2.6 Control Flow Instruction

    /// jp_nn: unconditional jump to absolute address specified by 16-bit immediate. Set PC = nn
    /// 3-byte instruction, 4 cycles.
    pub fn jp_nn(&self) -> ProgramCounter {
        ProgramCounter::Jump(self.get_nn())
    }

    /// jp_hl: unconditional jump to absolute address specified by 16-bit register HL. Set PC = HL.
    /// 1-byte instruction, 1 cycle.
    pub fn jp_HL(&self) -> ProgramCounter {
        ProgramCounter::Jump(self.reg.HL)
    }

    /// jp_cc_nn: Conditional jump to absolute address nn, depending on condition cc.
    /// cc is 2-bit number that refers to:
    /// 00 -> Z == 0; 01 -> Z == 1; 10 -> C == 0; 11 -> C == 1
    /// 3-byte instruction
    pub fn jp_cc_nn(&self) -> ProgramCounter {
        let abs_addr = self.get_nn();
        let cc = self.check_cc();
        let mut pc_final: ProgramCounter;

        if cc {
            pc_final = ProgramCounter::Jump(abs_addr);
        } else {
            pc_final = ProgramCounter::Next(3);
        }

        pc_final
    }

    /// jr_e: Unconditional jump to relative address specified by signed 8-bit operand e.
    /// 2 bytes, 3 cycles.
    pub fn jr_e(&self) -> ProgramCounter {
        let e = self.get_n() as i8; // idk if this works
        ProgramCounter::Next(e) // idk if this works also... needa try implementing ProgramCounter enum.
    }

    /// jr_cc_e: Conditional jump to relative address specified by signed 8-bit operand e, depending on condition cc.
    /// 2 bytes, 2 cycles if cc == false, 3 cycles if cc == true.
    pub fn jr_cc_e(&self) -> ProgramCounter {
        let e = self.get_n() as i8;
        let cc = self.check_cc();
        let mut pc_final: ProgramCounter;
        
        if cc {
            pc_final = ProgramCounter::Next(e);
        } else {
            pc_final = ProgramCounter::Next(2);
        }

        pc_final
    }    

    /// call_nn: unconditional function call to absolute address specified by 16-bit operand nn
    /// 3 bytes. 6 cycles
    pub fn call_nn(&self) -> ProgramCounter {
        let nn = self.get_nn();
        self.push_u16(self.reg.PC); // Push PC onto the stacc
        
        ProgramCounter::Jump(nn);
    }

    /// call_cc_nn: Conditional function call to absolute address specified by 16-bit operand nn,
    /// depending on condition cc.
    /// 3 bytes, 3 cycles if cc == false, 6 cycles if cc ==  true
    pub fn call_cc_nn(&self) -> ProgramCounter {
        let nn = self.get_nn();
        let cc = self.check_cc();

        let mut pc_final: ProgramCounter;

        if cc { // execute function call
            self.push_u16(self.reg.PC);
            pc_final = ProgramCounter::Jump(nn);
        } else {
            pc_final = ProgramCounter::Next(3);
        }

        pc_final
    }

    /// ret: Unconditional return from a function. Pop PC from stack.
    /// 1 byte, 4 cycles.
    pub fn ret(&self) -> ProgramCounter {
        let pop_val = self.pop_u16();

        ProgramCounter::Jump(pop_val)
    }

    /// ret_cc: Conditional return from a function, depending on condition cc.
    /// Only pop if cc is true.
    /// 1 byte, 2 cycles if cc = false, 5 cycles if cc = true
    pub fn ret_cc(&self) -> ProgramCounter {
        let cc = self.check_cc();
        let mut pc_final: ProgramCounter;

        if cc {
            let pop_val = self.pop_u16();
            pc_final = ProgramCounter::Jump(pop_val);
        } else {
            pc_final = ProgramCounter::Next(1);
        }

        pc_final
    }

    /// reti: Unconditional return from a function. Enables IME signal.
    /// IME is Interrupt Master Enable. When this is enabled, interrupts can happen
    /// same as ret, but set register IME.
    pun fn reti(&self) -> ProgramCounter {
        let pop_val = self.pop_u16();
        self.reg.IME = 1;

        ProgramCounter::Jump(pop_val)
    }

    /// rst_n: Unconditional function call to absolute fixed address defined by opcode.
    /// opcode specifies rst_address in xxx: bits 3 4 5. Each combination of xxx indicates an
    /// address.
    /// 1 byte, 4 cycles.
    pub fn rst_n(&self) -> ProgramCounter {
        // push pc onto stack
        self.push_u16(self.reg.PC);

        let xxx = self.get_r_to(); // same bits
        let pc_msb: u16 = 0x00;
        let mut pc_lsb: u16;

        match xxx {
            0 => pc_lsb = 0x00,
            1 => pc_lsb = 0x08,
            2 => pc_lsb = 0x10,
            3 => pc_lsb = 0x18,
            4 => pc_lsb = 0x20,
            5 => pc_lsb = 0x28,
            6 => pc_lsb = 0x30,
            7 => pc_lsb = 0x38,
        }

        let addr = (pc_msb << 8) | pc_lsb;

        ProgramCounter::Jump(addr)
    }
        
    /// halt: CPU enters "halt mode" and stops system clock. Oscillator circuit and LCD Controller
    /// continue to operate. "halt mode" can be cancelled with an interrupt or reset signal.
    /// PC is halted as well. After interrupted / reset, program starts from PC address.
    /// TODO: find a way to implement
    
    /// stop: CPU enters "stop mode" and stops everything including system clock, 
    /// oscillator circuit and LCD Controller.
    /// TODO: find a way to implement

    /// di: Disables interrupt handling by setting IME = 0, cancelling any scheduled effects of the
    /// EI instruction if any.
    /// 1 byte, 1 cycle
    pub fn di(&self) -> ProgramCounter {
        self.reg.ime = 0;

        ProgramCounter::Next(1)
    }

    /// ei: schedules interrupt handling to be enabled THE NEXT MACHINE CYCLE
    /// 1 byte, 1 cycle + 1 cycle for EI effect.
    /// TODO: find a way to implement

    /// ccf: Flips carry flag, reset N and H flags
    /// 1 byte, 1 cycle.
    pub fn ccf(&self) -> ProgramCounter {
        let c_bit = (self.reg.F & CF) >> 4;
        let z_bit = (self.reg.F & ZH) >> 7;

        // set all the flags
        self.set_hcnz(false, c_bit == 0, false, z_bit == 1);

        ProgramCounter::Next(1)
    }

    /// scf: Sets carry flag, reset N and H flags.
    /// 1 byte, 1 cycle
    pub fn scf(&self) -> ProgramCounter {
        let z_bit = (self.reg.F & ZF) >> 7;

        // set carry, reset n and h
        self.set_hcnz(false, true, false, z_bit == 1);

        ProgramCounter::Next(1)
    }

    /// nop: this doesn't do anything lmao, but add one cycle and increment PC by 1.
    /// 1 byte, 1 cycle
    pub fn nop(&self) -> ProgramCounter {
        ProgramCounter::Next(1)
    }

    /// daa: decimal adjust acc.
    /// This is binary arithmetic acting as binary numbers...
    /// 1 byte, 1 cycle.
    pub fn daa(&self) -> ProgramCounter {
        let mut a: u8 = self.read_from_r8(A_ID)?;

        let is_addition: bool = (self.reg.F & NF) > 0;
        let c_flag: bool = (self.reg.F & CF) > 0;
        let h_flag: bool = (self.reg.F & HF) > 0;
        let mut has_carry: bool = false;

        if is_addition { // after addition, adjust if half-carry occured or if results out of bounds.
            if (a > 0x90 || c_flag) {
                a += 0x60;
                has_carry = true;
            }

            if ((a & 0x0F) > 0x09 || h_flag) {
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

        ProgramCounter::Next(1)
    }

    /// cpl: flip all bits in the A-register, sets N and H to 1.
    /// 1 byte, 1 cycle
    pub fn cpl(&self) -> ProgramCounter {
        let mut a: u8 = self.read_from_u8(A_ID)?;

        let mut n = 0;

        while n < 8 {
            // reverse every bit in a
            a = a ^ (0x01 << n);
            n += 1;
        }

        self.write_to_r8(A_ID, a);

        ProgramCounter::Next(1)
    }
