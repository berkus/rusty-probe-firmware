# build default configuration
build: build-bin make-uf2
    ls -la app.uf2

alias b := build

# build binary without logging
build-bin: flip-link
    DEFMT_LOG=off cargo build --release --bin app

# run binary with logging
run-bin-defmt SERIAL='/dev/ttyACM0': flip-link defmt-print
    XTASK_SERIAL={{ SERIAL }} XTASK_DEFMT_VERBOSE=true DEFMT_LOG=trace cargo rrb-usb app

# convert binary to uf2
make-uf2: elf2uf2
    elf2uf2-rs target/thumbv6m-none-eabi/release/app app

defmt-print:
    @defmt-print --help > /dev/null || cargo install defmt-print

elf2uf2:
    @elf2uf2-rs --help > /dev/null || cargo install elf2uf2-rs

flip-link:
    # someday knurling folks will autorelease new flip-link with --help arg support
    @which flip-link || cargo install flip-link
