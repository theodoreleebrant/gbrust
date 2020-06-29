use super::ppu::Ppu;
use super::cart::Cart;
use super::timer::Timer;
use super::gamepad::Gamepad;

const RAM_SIZE: usize = 32 * 1024; // Memory for the last 32KB as first 32KB is for ROM
const HW_IO_REG: usize = 0x7f; //Hardware IO register memory FF00-FF7F; impl as hwram

pub struct Interconnect {
    pub cart: Cart,
    ppu: Ppu,
    ram: Box<[u8]>,      
    hwram: Box<[u8]>,
    ppu_dma: u8,
    pub int_enable: u8,
    pub int_flags: u8,
    ram_offset: usize,
    // TODO: Sound Processing unit, gamepad, timer
}

impl Interconnect {
    pub fn new(gameboy_type: GameboyType,
               cart: Cart,
               ppu: Ppu)
               -> Interconnect {
        Interconnect {
            cart: cart,
            ppu: ppu,
            // spu: spu,
            // timer: Timer::new(),
            // gamepad: gamepad,
            ram: vec![0; RAM_SIZE].into_boxed_slice(),
            hwram: vec![0; HW_IO_REG].into_boxed_slice(),
            ppu_dma: 0,
            int_enable: 0,
            int_flags: 0,
            ram_offset: 0,
        }
    }

}