pub mod dmg_cpu;
pub mod cart;
pub mod ppu;
pub mod interconnect;
pub mod gamepad;
pub mod console;
pub mod timer;
pub mod cpu_test;

pub use self::cart::*;
pub use self::dmg_cpu::*;
pub use self::ppu::*;
pub use self::interconnect::*;
pub use self::gamepad::*;
pub use self::console::*;
pub use self::timer::*;

bitflags! {
    pub struct Interrupts: u8 {
        const INT_VBLANK = 0b0000_0001;
        const INT_LCDSTAT = 0b0000_0010;
        const INT_TIMEROVERFLOW = 0b0000_0100;
        const INT_SERIAL = 0b0000_1000;
        const INT_JOYPAD = 0b0001_0000;
    }
}
