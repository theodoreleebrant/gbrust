use super::Interrupts;

#[derive(Debug)]
pub enum ButtonState {
    Up,
    Down,
}

#[derive(Debug,Copy,Clone)]
pub enum Button {
    Up,
    Down,
    Left,
    Right,
    A,
    B,
    Start,
    Select,
}


// From PanDocs
// FF00 - P1/JOYP - Joypad (R/W)
// Bit 7 - Not used
// Bit 6 - Not used
// Bit 5 - P15 Select Button Keys      (0=Select)
// Bit 4 - P14 Select Direction Keys   (0=Select)
// Bit 3 - P13 Input Down  or Start    (0=Pressed) (Read Only)
// Bit 2 - P12 Input Up    or Select   (0=Pressed) (Read Only)
// Bit 1 - P11 Input Left  or Button B (0=Pressed) (Read Only)
// Bit 0 - P10 Input Right or Button A (0=Pressed) (Read Only)
impl Button {
    fn flag(&mut self) -> u8 {
        use self::Button::*;
        match self {
            Right | A => 0b0001,
            Left | B => 0b0010,
            Up | Select => 0b0100,
            Down | Start => 0b1000,
        }
    }
}

#[derive(Debug)]
pub struct InputEvent {
    button: Button,
    state: ButtonState,
}

impl InputEvent {
    pub fn new(button: Button, state: ButtonState) -> InputEvent {
        InputEvent {
            button: button,
            state: state,
        }
    }
}

pub struct Gamepad {
    direction_keys: u8,
    button_keys: u8,
    port: u8,
}

impl Gamepad {
    pub fn new() -> Gamepad {
        Gamepad {
            // State of keys
            // None pressed at initialisation
            direction_keys: 0b0000_1111,
            button_keys: 0b0000_1111,

            // Bits: unused, unused, direction, button
            port: 0b1111_0000, 
        }
    }

    pub fn read(&mut self) -> u8 {
        // Expected output: 0b0000_xxxx
        // xxxx indicates the buttons pressed
        // needs an indicator whether reading button or direction
        //      which is in the 0b**xx_**** bits of self
        //      and can be set using write

        let mut input = self.port | 0b1100_0000;
        // Ignore first two bit

        if (self.port & 0b0001_0000) != 0 {
            input |= self.button_keys & 0b0000_1111
        }

        if (self.port & 0b0010_0000) != 0 {
            input |= self.direction_keys & 0b0000_1111
        }

        input
    }

    pub fn write(&mut self, val: u8) {
        // Sets which set of buttons will be read
        self.port = val & 0b0011_0000
    }

    pub fn cycle_flush(&mut self, _cycle_count: u32) -> Interrupts {
        Interrupts::empty()
    }

    pub fn handle_event(&mut self, mut event: InputEvent) {
        use self::Button::*;

        match event.state {
            // Press button down
            // Idea: (curr) & (flag) 
            //      => will turn 0 everytime pressed(0)
            //      => other keys stays same
            // Flag: if pressed, then zero
            // Curr: if pressed, then zero
            ButtonState::Down => {
                let flag = !event.button.flag(); // 0b****_xxxx -> xxxx indicate the button pressed
                match event.button {
                    Up | Down | Left | Right => self.direction_keys = self.direction_keys & flag,
                    A | B | Start | Select => self.button_keys = self.button_keys & flag,
                }
            }

            // Release button
            // Idea: (curr) | (flag)
            //      => if true (released), will always turn 1
            //      => other keys stays same
            // Flag: if pressed, then 1
            // Curr: if pressed, then 0
            ButtonState::Up => {
                let flag = event.button.flag(); // 0b****_xxxx -> xxxx indicate the button pressed
                match event.button {
                    Up | Down | Left | Right => self.direction_keys = self.direction_keys | flag,
                    A | B | Start | Select => self.button_keys = self.button_keys | flag,
                }

            }
        }
    }
}
