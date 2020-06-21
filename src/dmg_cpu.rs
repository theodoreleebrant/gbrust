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
    pub mut SP: u16,    // Stack pointer
    pub mut PC: u16,
}

pub struct CPU {
    pub mut reg: Registers,     // Set of registers
    
    pub mut mem: [u8; 65536],   // 64KB memory
    pub mut stack: [u8; 256],    // Stack for PC

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

    /// rotate_left_r8: Rotate left function for 8-bit registers.\
    /// There are 2 types of rotate operations: Has carry or no carry.
    /// If operation has carry: bit A7 is copied to flag CY AND bit 0 of A.
    /// If operation has no carry: bit 0 of A is replaced by CY, and then bit A7 is copied to CY 
    pub fn rotate_left_r8(&self, r8_id: u8, has_carry: bool) {
        let mut data: u8;

        match self.read_from_r8(r8_id) {
            Some(value) => data = value,
            None => return (), // terminate
        }

        let bit_a7: u8 = (data & 0x80) >> 7;
        let bit_cf: u8 = (self.reg.F & CF) >> 4;
        data = (data << 1) as u8; // a7 diasppeared, a0 = 0

        // setting bit a0
        if has_carry {
            data = data | bit_a7;
        } else {
            data = data | bit_cf;
        } 
        
        // set CF to bit_a7
        if bit_a7 > 0 {
            self.set_flag(CF);
        } else {
            self.reset_flag(CF);
        }
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

        self.stack[self.reg.SP - 1] = (rr >> 8) as u8;
        self.stack[self.reg.SP - 2] = (rr & 0x00FF) as u8;

        self.reg.SP -= 2;

        ProgramCounter::Next(1)
    }

    /// pop_rr: pop data from stack to the 16-bit register rr.
    /// 1-byte instruction
    pub fn pop_rr(&self) -> ProgramCounter {
        let rr = self.get_r16();
        
        let nn_low = self.stack[self.reg.SP as usize];
        let nn_high = self.stack[(self.reg.SP + 1) as usize];

        self.write_to_r16(rr, nn_low | (nn_high << 8));

        ProgramCounter::Next(1)
    }


    // 8 Bit Arithmetic Operation Instruction
    // ADD A,r: Add the value in register r to A, set it to A. 
    pub fn add_ar(&self) -> ProgramCounter {
	    // reading
	    let a: u8 = self.read_from_r8(A_ID)?;
	    let r: u8 = self.get_r8_from();

	    // processing
	    let res: u16 = (a as u16) + (r as u16);

	    // flags and writing
	    let h: bool = ((a & 0x0F) + (r & 0x0F)) > 0x0F;
	    let c: bool = res > 0xFF;
	    let n: bool = false;
	    let to_write: u8 = (a & 0xFF) as u8;
	    let z: bool = to_write == 0;

	    self.write_to_r8(A_ID, to_write);
	    if h {self.set_flag(H_ID)} else {self.reset_flag(H_ID)};
	    if c {self.set_flag(C_ID)} else {self.reset_flag(C_ID)};
	    if n {self.set_flag(N_ID)} else {self.reset_flag(N_ID)};
	    if z {self.set_flag(Z_ID)} else {self.reset_flag(Z_ID)};

	    ProgramCounter::Next(1)
	}





}
