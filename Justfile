# build default configuration
build: deps-up build-bin make-uf2

# build binary without logging
build-bin:
    DEFMT_LOG=off cargo build --release --bin app

# run binary with logging
run-bin-defmt SERIAL='/dev/ttyACM0':
    echo "Taking control on {{ SERIAL }}"
    XTASK_SERIAL={{ SERIAL }} XTASK_DEFMT_VERBOSE=true DEFMT_LOG=trace cargo rrb-usb app

# convert binary to uf2
make-uf2:
    elf2uf2-rs target/thumbv6m-none-eabi/release/app app
    ls -la app.uf2

# install build dependencies, if not installed
deps-up:
    @elf2uf2-rs --help > /dev/null || cargo install elf2uf2-rs
    cargo install flip-link
    cargo install defmt-print

alias b := build
