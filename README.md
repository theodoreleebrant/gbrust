# gbrust: a Gameboy emulator built in Rust

This is a Gameboy emulator, built in Rust with sdl2 for sound processing.
Made this with a friend solely to learn Rust and emulation.

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

### Credits
This project is indebted to the numerous documentations as well as other similar projects. In particular, we have taken reference from:  
[Awesome Gameboy Documentation List](https://gbdev.io/list.html) - One-stop documentation, has most of the below inside.  
[Pandocs](https://gbdev.io/pandocs/) - "The single, most comprehensive technical reference to Game Boy available to the public".  
[Gameboy Complete Technical Reference](https://gekkio.fi/files/gb-docs/gbctr.pdf) - Nicely formatted reference to start building the emulator.  
[Nintendo's Gameboy Programming Manual](https://archive.org/download/GameBoyProgManVer1.1/GameBoyProgManVer1.1.pdf) - The official Nintendo manual for Gameboy / GBC / GBA for game makers.  
[DMG Memory Map](http://gameboy.mongenel.com/dmg/asmmemmap.html) - A nice visual reference for map addresses in the Gameboy  
[opcode table](https://gbdev.io/gb-opcodes/optables/) - Also a nice visual for opcode table  
[Natesh's Dev Blog](https://nnarain.github.io/2016/09/09/Gameboy-LCD-Controller.html) - Used it to understand LCD controller for the Gameboy  


Other Gameboy emulators written in Rust:
[by simias](https://github.com/simias/gb-rs)  
[by ev-wilt](https://github.com/ev-wilt/rusty_boy_DMG)  
[by dimitribobkov](https://github.com/dimitribobkov/gameboy)

Other projects:
[Blargg's test suite](https://gbdev.gg8.se/files/roms/blargg-gb-tests/) - Used for testing components of the Gameboy.  
[Yet another CHIP-8 Rust Emulator](https://github.com/theodoreleebrant/YARC) - Used to model the structure of this emulator
