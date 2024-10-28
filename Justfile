# build default configuration
build: deps-up
    DEFMT_LOG=off cargo build --release --bin app
    elf2uf2-rs target/thumbv6m-none-eabi/release/app app
    ls -la app.uf2

# install build dependencies, if not installed
deps-up:
    @elf2uf2-rs --help > /dev/null || cargo install elf2uf2-rs
    cargo install flip-link

alias b := build
