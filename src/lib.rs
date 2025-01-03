#![no_std]

// use panic_probe as _;

use defmt_brtt as _;

use rtic_monotonics::Monotonic;
use setup::Mono;

pub mod dap;
pub mod device_signature;
mod jtag;
pub mod leds;
mod panic;
pub mod pio;
pub mod setup;
pub mod systick_delay;
mod taps;
pub mod usb;

defmt::timestamp! {"{=u64:us}", {
    Mono::now().duration_since_epoch().to_micros()
}
}
