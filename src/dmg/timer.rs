use std::u8;
use super::Interrupts;
// use super::INT_TIMEROVERFLOW;

const INT_TIMEROVERFLOW: Interrupts = Interrupts::INT_TIMEROVERFLOW;

// Clock speed
// See PanDocs: https://gbdev.io/pandocs/#timer-and-divider-registers
const CLOCKS: [u32; 4] = [1024, 16, 64, 256];

#[derive(Debug)]
pub struct Timer {
    // FF04 - DIV - Divider Register (R/W)
    // This register is incremented at rate of 16384Hz. 
    // Writing any value to this register resets it to 00h.
    // For simplicity, we can write div in terms of cycles instead of ticks.
    // 1 cycle = 4 clocks
    div: u8,
    div_cycles: u8,

    // FF05 - TIMA - Timer counter (R/W)
    // This timer is incremented by a clock frequency specified by 
    // the TAC register ($FF07). When the value overflows (gets bigger than FFh) 
    // then it will be reset to the value specified in TMA (FF06), 
    // and an interrupt will be requested, as described below.
    // For simplicity, we can write tima in terms of cycles instead of ticks.
    tima: u8,
    tima_cycles: u32,

    // FF06 - TMA - Timer Modulo (R/W)
    // When the TIMA overflows, this data will be loaded.
    tma: u8,

     // FF07 - TAC - Timer Control (R/W)
     //    Bit  2   - Timer Enable
     //    Bits 1-0 - Input Clock Select - corresponds to CLOCKS[]
     //       00: CPU Clock / 1024 (DMG, CGB:   4096 Hz, SGB:   ~4194 Hz)
     //       01: CPU Clock / 16   (DMG, CGB: 262144 Hz, SGB: ~268400 Hz)
     //       10: CPU Clock / 64   (DMG, CGB:  65536 Hz, SGB:  ~67110 Hz)
     //       11: CPU Clock / 256  (DMG, CGB:  16384 Hz, SGB:  ~16780 Hz)
    enabled: bool,
    clock_select: u8, // the bits
    clock_rate: u32,  // the rate as defined above
}

impl Timer {
    pub fn new() -> Timer {
        Timer {
            div: 0,
            div_cycles: 0,
            tima: 0,
            tima_cycles: 0,
            tma: 0,
            enabled: false,
            clock_select: 0,
            clock_rate: CLOCKS[0],
        }
    }

    pub fn read(&self, addr: u16) -> u8 {
        match addr {
            0xff04 => self.div,
            0xff05 => self.tima,
            0xff06 => self.tma,
            0xff07 => (self.clock_select & 0b11) | if self.enabled { 0b100 } else { 0 },
            // Shouldn't go here, as specified by the bigass switch statement in interconnect
            _ => panic!("Address not in range 0x{:x}", addr),
        }
    }

    pub fn write(&mut self, addr: u16, val: u8) {
        match addr {
            0xff04 => self.div = 0,
            0xff05 => self.tima = val,
            0xff06 => self.tma = val,
            0xff07 => {
                self.clock_select = val & 0b11;
                self.enabled = (val & 0b100) != 0;
                self.clock_rate = CLOCKS[self.clock_select as usize]
            }
            // Shouldn't go here, as specified by the bigass switch statement in interconnect 
            _ => panic!("Address not in range 0x{:x}", addr),
        }
    }

     //Interrupt
     pub fn cycle_flush(&mut self, cycle_count: u32) -> Interrupts {
         self.flush_div(cycle_count);

         if self.flush_tima(cycle_count) {
             INT_TIMEROVERFLOW
         } else {
             Interrupts::empty()
         }
     }

    fn flush_tima(&mut self, cycle_count: u32) -> bool {
        // returns if it overflows
        let tima_cycles = self.tima_cycles + cycle_count;
        let rate = self.clock_rate;
        let ticks = tima_cycles / rate;

        self.tima_cycles = tima_cycles - rate * ticks;

        if self.enabled {
            // Timer is enabled
            // increment tima by number of ticks
            // pub const fn overflowing_add(self, rhs: u16) -> (u16, bool)
            let (tima, overflow) = self.tima.overflowing_add(ticks as u8);
            self.tima = if overflow {
                // we set tma if tima is overloaded
                // [possible bug? is tma overwritten? (should be but too tired to think)]
                self.tma.wrapping_add(tima)
            } else { tima };
            overflow
        } else {
            false
        }
    }

    fn flush_div(&mut self, cycle_count: u32) {
        // we shall use div and div_cycles as defined in here, not pandocs
        // div counts how many times div_cycle overflows
        // result is u8 + u8 = u16 counter where the 8 MSB is div and 8 LSB is div_cyles
        // this one is independent of the tim/tima counter ticks
        // basically div++ everytime cycle count exceeds 255.
        let div_ticks = cycle_count >> 8;

        self.div = self.div.wrapping_add(div_ticks as u8);

        // the 8 LSB (0xFF)
        let div_inc_ticks = (cycle_count - (div_ticks << 8)) as u8;
        let (div_cycles, overflow) = self.div_cycles.overflowing_add(div_inc_ticks);

        self.div_cycles = div_cycles;

        if overflow {
            self.div = self.div.wrapping_add(1)
        }
    }
}
