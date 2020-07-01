/*
use super::*;
use dmg::cart::Cart;
use dmg::dmg_cpu::Cpu;

const A_DEF: u8 = 0x80;
const B_DEF: u8 = 0x40;
const C_DEF: u8 = 0x20;
const D_DEF: u8 = 0x10;
const HL_DEF: u16 = 0xFF00;
const DE_DEF: u16 = 0x00FF;

const MEM_HL_DEF: u8 = 0x08;
const MEM_DE_DEF: u8 = 0x04;

const N_DEF: u8 = 0xAB;
const NN_DEF: u16 = 0xABCD;

fn setup_cpu() -> Cpu {
    let mut cpu = Cpu::new(Interconnect::new(Cart::new(vec![0; 32000].into_boxed_slice())));
    cpu.reg.a = A_DEF;
    cpu.reg.b = B_DEF;
    cpu.reg.c = C_DEF;
    cpu.reg.d = D_DEF;
    cpu.reg.hl= HL_DEF;
    cpu.reg.de= DE_DEF;
    cpu.mem[cpu.reg.hl as usize] = MEM_HL_DEF;
    cpu.mem[cpu.reg.de as usize] = MEM_DE_DEF;
    
    cpu;
}

fn set_1byte_op(cpu: &mut Cpu, opcode: u8) {
    cpu.mem[cpu.reg.pc as usize] = opcode;
    cpu.execute_opcode();
}

fn set_2byte_op(cpu: &mut Cpu, opcode: u16) {
    cpu.mem[cpu.reg.pc as usize] = (opcode >> 8) as u8;
    cpu.mem[(cpu.reg.pc + 1) as usize] = opcode as u8;
    cpu.execute_opcode();
}

fn set_3byte_op(cpu: &mut Cpu, opcode: u32) {
    cpu.mem[cpu.reg.pc as usize] = (opcode >> 16) as u8;
    cpu.mem[(cpu.reg.pc + 1) as usize] = (opcode >> 8) as u8;
    cpu.mem[(cpu.reg.pc + 2) as usize] = opcode as u8;
    cpu.execute_opcode();
}

fn set_4byte_op(cpu: &mut Cpu, opcode: u32) {
    cpu.mem[cpu.reg.pc as usize] = (opcode >> 24) as u8;
    cpu.mem[(cpu.reg.pc + 1) as usize] = (opcode >> 16) as u8;
    cpu.mem[(cpu.reg.pc + 2) as usize] = (opcode >> 8) as u8;
    cpu.mem[(cpu.reg.pc + 3) as usize] = opcode as u8;
    cpu.execute_opcode();
}

fn get_from_hl(cpu: &Cpu) -> u8 {
    cpu.mem[cpu.reg.hl]
}

#[test]
fn test_initial_state() {
    let cpu = Cpu::new();
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

#[test]
fn test_8bit_ld() {
    let mut cpu = setup_cpu();
    
    // ld_addr_hl_n
    set_2byte_op(&mut cpu, 0b0011_0110_0000_0000 | (N_DEF as u16));
    cpu.execute_opcode();
    assert_eq!(get_from_hl(&cpu), N_DEF);
}
*/
#[cfg(test)]
mod tests {
    use super::*;

    const A_DEF: u8 = 0x80;
    const B_DEF: u8 = 0x40;
    const C_DEF: u8 = 0x20;
    const D_DEF: u8 = 0x10;
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
        cpu.execute_opcode();
    }

    fn set_2byte_op(cpu: &mut Cpu, opcode: u16) {
        cpu.mem[cpu.reg.pc as usize] = (opcode >> 8) as u8;
        cpu.mem[(cpu.reg.pc + 1) as usize] = opcode as u8;
        cpu.execute_opcode();
    }

    fn set_3byte_op(cpu: &mut Cpu, opcode: u32) {
        cpu.mem[cpu.reg.pc as usize] = (opcode >> 16) as u8;
        cpu.mem[(cpu.reg.pc + 1) as usize] = (opcode >> 8) as u8;
        cpu.mem[(cpu.reg.pc + 2) as usize] = opcode as u8;
        cpu.execute_opcode();
    }

    fn set_4byte_op(cpu: &mut Cpu, opcode: u32) {
        cpu.mem[cpu.reg.pc as usize] = (opcode >> 24) as u8;
        cpu.mem[(cpu.reg.pc + 1) as usize] = (opcode >> 16) as u8;
        cpu.mem[(cpu.reg.pc + 2) as usize] = (opcode >> 8) as u8;
        cpu.mem[(cpu.reg.pc + 3) as usize] = opcode as u8;
        cpu.execute_opcode();
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
        cpu.write_to_r8(A_ID, N_DEF);
        assert_eq!(cpu.reg.a, N_DEF);
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
}
