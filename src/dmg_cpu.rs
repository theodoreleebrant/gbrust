// Declare all register ID and necessary constants

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

    // Opcodes goes here!!




}
