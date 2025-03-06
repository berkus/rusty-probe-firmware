# build default firmware image
build: build-bin make-uf2
    ls -la app.uf2

alias b := build

# (hidden) build binary without logging
[private]
build-bin: install-flip-link
    DEFMT_LOG=off cargo build --release --bin app

# call it like `just r /dev/tty.usbmodem21203`
# run binary with logging
run-bin-defmt SERIAL='/dev/ttyACM0': install-flip-link install-defmt-print
    XTASK_SERIAL={{ SERIAL }} XTASK_DEFMT_VERBOSE=true DEFMT_LOG=trace,dap_rs=trace cargo run-usb

alias r := run-bin-defmt

# (hidden) convert binary to uf2
[private]
make-uf2: install-elf2uf2
    elf2uf2-rs target/thumbv6m-none-eabi/release/app app

#===============================================================================

# (hidden) install defmt-print crate
[private]
install-defmt-print:
    @defmt-print --help > /dev/null || cargo install defmt-print

# (hidden) install elf2uf2 tool
[private]
install-elf2uf2:
    @elf2uf2-rs --help > /dev/null || cargo install elf2uf2-rs

# (hidden) install flip-link tool
[private]
install-flip-link:
    @flip-link --help 2> /dev/null || cargo install flip-link
