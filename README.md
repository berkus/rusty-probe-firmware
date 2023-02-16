# Rusty Probe

This firmware implements an CMSIS-DAP v1 and v2 compatible probe.

## Building

You can build the project and generate a `.uf2` file as follows:

```console
# Install elf2uf2-rs
cargo install elf2uf2-rs

# Build the ELF without logging
DEFMT_LOG=off cargo build --release --bin app

# Generate .uf2 file
elf2uf2-rs target/thumbv6m-none-eabi/release/app app
```

Start the Pico in bootloader mode and drop the `app.uf2` file to it, done! 

## TODO

- [x] Move SWD impl to PIO
- [ ] Add support for SWO (Manchester encoding or UART via PIO)
- [ ] Add support for VCP (it enumerates now, but ignores all data)
- [ ] Add the automatic polling of RTT buffers
- [ ] Document the `dap-rs` traits and helpers
- [ ] Document the firmware

