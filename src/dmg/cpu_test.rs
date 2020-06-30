use super::*;

fn setup_cpu() -> CPU {
    let mut cpu = CPU::new();
    cpu.reg.a = 0x80;
    cpu.reg.b = 0x40;
    cpu.reg.c = 0x20;
    cpu.reg.d = 0x10;
    cpu.reg.hl= 0xFF00;
    cpu.reg.de= 0x00FF;
    cpu.mem[cpu.reg.hl as usize] = 0x08;
    cpu.mem[cpu.reg.de as usize] = 0x04;
    
    cpu;
}

fn set_1byte_op(cpu: &mut CPU, opcode: u8) {
    cpu.mem[cpu.reg.pc as usize] = opcode;
    cpu.execute_opcode();
}

fn set_2byte_op(cpu: &mut CPU, opcode: u16) {
    cpu.mem[cpu.reg.pc as usize] = (opcode >> 8) as u8;
    cpu.mem[(cpu.reg.pc + 1) as usize] = opcode as u8;
    cpu.execute_opcode();
}

fn set_3byte_op(cpu: &mut CPU, opcode: u32) {
    cpu.mem[cpu.reg.pc as usize] = (opcode >> 16) as u8;
    cpu.mem[(cpu.reg.pc + 1) as usize] = (opcode >> 8) as u8;
    cpu.mem[(cpu.reg.pc + 2) as usize] = opcode as u8;
    cpu.execute_opcode();
}

fn set_4byte_op(cpu: &mut CPU, opcode: u32) {
    cpu.mem[cpu.reg.pc as usize] = (opcode >> 24) as u8;
    cpu.mem[(cpu.reg.pc + 1) as usize] = (opcode >> 16) as u8;
    cpu.mem[(cpu.reg.pc + 2) as usize] = (opcode >> 8) as u8;
    cpu.mem[(cpu.reg.pc + 3) as usize] = opcode as u8;
    cpu.execute_opcode();
}

#[test]
fn test_initial_state() {
    let cpu = CPU::new();
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
    
    cpu.mem[cpu.reg.pc as usize] = 0b0011_0110; // ld_addr_hl_n()
    cpu.mem[(cpu.reg.pc + 1) as usize] = 0x80;
    cpu.execute_opcode();
    assert_eq!(cpu.mem[

