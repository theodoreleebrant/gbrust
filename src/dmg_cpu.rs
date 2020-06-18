// Declare all register ID and necessary constants

pub struct Registers {
    // 8 8-bit registers (including register F)
    // 5 16-bit register (all pairs, not including AF, including PC)
    
    pub mut A: u8,      // Accumulator register
    pub mut F: u8,      // Special flag register
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
    pub mut SP: u16,
    pub mut PC: u16,

}


pub struct CPU {
    // All the components of CPU
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
