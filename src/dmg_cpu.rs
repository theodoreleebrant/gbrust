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
    pub mut stack: Vec<u16>,    // Stack for PC

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
            .. => result = None,
        }

        if result == None {
            return None;
        }

        Some(result)
    }

    pub fn load_mem_to_r8(&self, r8_id: u8, addr: u16) -> bool{
        self.write_to_r8(r8_id, self.mem[addr as usize])
    }

    pub fn save_r8_to_mem(&self, r8_id: u8, addr: u16) {
        match self.read_from_r8(r8_id) {
            Some(content) => mem[addr as usize] = content,
            None => (),
        }
    }


    // Opcodes goes here!!
    
    // 8-bit load instructions
    
    /// ld_rx_ry: load contents of ry to rx. 1-byte instruction
    /// @param rx, ry: ID for register rx and ry (8-bit)
    pub fn ld_rx_ry(&self, rx: u8, ry: u8) -> ProgramCounter {
        match self.read_from_r8(ry) {
            Some(value) => self.write_to_r8(rx),
            None => (),
        }

        ProgramCounter::Next(1)
    }

    /// ld_r_n: Load 8-bit data n into register r. 2-byte instruction
    /// @param: r: register ID; n: intermediate
    pub fn ld_r_n(&self, r: u8, n: u8) -> ProgramCounter {
        self.write_to_r8(r, n);

        ProgramCounter::Next(2)
    }

    /// ld_r_addr_HL: loads contents of memory specified at (HL) to register r. 1-byte instruction
    pub fn ld_r_addr_HL(&self, r: u8) -> ProgramCounter {
        load_mem_to_r8(r, self.reg.HL);
    }

    






}
