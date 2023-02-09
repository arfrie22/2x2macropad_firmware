# Firmware for the 2x2 Macropad

This is the firmware for the 2x2 Macropad. It is a completely open source project, and you can find the files for the PCB on [GitHub](https://github.com/arfrie22/2x2macropad_pcbs). The firmware was written in Rust and was my first real use of rust.

## Features

- Custom lighting effects
- Support for normal keys and media keys
- Keys can act as a key or used for a custom macro
- Up to 4 macros per key
- Customizable timing

## Building

To build the firmware, you will need to install the Rust toolchain. You can find instructions on how to do that [here](https://www.rust-lang.org/tools/install). Once you have the toolchain installed, you can build the firmware by running `cargo build --release`. The firmware will be located in `target/release/2x2macropad`.

If you use `cargo run`, the firmware will be built and flashed to the board. You will need to have a programmer connected to the board for this to work or to have the board in dfu mode. The way to change the flasher is to change the file in [.cargo/config.toml](.cargo/config.toml).

You can also find prebuilt firmware in the [releases](releases) page.

You can also use the [configurator](https://arfrie22.github.io/2x2macropad_configurator/) to flash the firmware to the board.
