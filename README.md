# gbrust: a Gameboy emulator built in Rust

This is a Gameboy emulator, built in Rust with sdl2 for sound processing.
## Requirements
You will need to install Rust, as well as sdl2 with headers.  
Instruction to install Rust can be seen at the [Rust installation guide](https://www.rust-lang.org/tools/install)  
Instruction to install sdl2:

> Homebrew

`brew install sdl2`

> Or using apt

`sudo apt-get install libsdl2-dev libsdl2-gfx-dev`

## Run the program

Running a gameboy game is done through the following command
`````
cargo build --release
cargo run filename
`````

For example:
`````
cargo build --release
cargo run somegame.gb
`````

Please obtain your ROMs legally.

## Controls
This emulator takes in input from the following keyboard keys:
Directional keys: Arrow Keys (Up, Down, Left, Right)
A button: Spacebar
B button: Left Control
Start button: Enter
Select button: Right Shift
