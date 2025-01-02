use crate::{
    setup::{DirSwclkPin, DirSwdioPin, ResetPin, SwclkTckPin, SwdioTmsPin, TdiPin, TdoSwoPin},
    systick_delay::Delay,
};
use dap_rs::{swj::Dependencies, *};
use defmt::trace;
use embedded_hal::{
    delay::DelayNs,
    digital::{OutputPin, PinState},
};

// const DAP2_PACKET_SIZE: u16 = 512;

pub struct Context {
    max_frequency: u32,
    cpu_frequency: u32,
    cycles_per_us: u32,
    half_period_ticks: u32,
    delay: &'static Delay,
    swdio_tms: SwdioTmsPin, // Shared by SWD and JTAG
    swclk_tck: SwclkTckPin, // Shared by SWD and JTAG
    tdo: TdoSwoPin,
    tdi: TdiPin,
    nreset: ResetPin, // Shared by SWD and JTAG
    dir_swdio: DirSwdioPin,
    dir_swclk: DirSwclkPin,
}

// struct JtagPins<'ctx> {
//     tms: &'ctx SwdioPin, // Shared by SWDIO
//     tck: &'ctx SwclkPin, // Shared by SWCLK
//     tdo: &'ctx TdoSwoPin,
//     tdi: &'ctx TdiPin,
//     nreset: &'ctx ResetPin,
// }

// impl<'a> From<&'a Context> for JtagPins<'a> {
//     fn from(value: &'a Context) -> Self {
//         Self {
//             tms: &value.swdio_tms,
//             tck: &value.swclk_tck,
//             tdo: &value.tdo,
//             tdi: &value.tdi,
//             nreset: &value.nreset,
//         }
//     }
// }

// struct SwdPins {
//     swdio: SwdioPin, // Shared by TMS
//     swclk: SwclkPin, // Shared by TCK
//     nreset: ResetPin,
//     dir_swdio: DirSwdioPin,
//     dir_swclk: DirSwclkPin,
// }

// impl From<Context> for SwdPins {
//     fn from(value: Context) -> Self {
//         Self {
//             swdio: value.swdio_tms,
//             swclk: value.swclk_tck,
//             nreset: value.nreset,
//             dir_swdio: value.dir_swdio,
//             dir_swclk: value.dir_swclk,
//         }
//     }
// }

impl defmt::Format for Context {
    fn format(&self, f: defmt::Formatter) {
        // format the bitfields of the register as struct fields
        defmt::write!(
           f,
           "Context {{ max_frequency: {}, cpu_frequency: {}, cycles_per_us: {}, half_period_ticks: {} }}",
            self.max_frequency,
            self.cpu_frequency,
            self.cycles_per_us,
            self.half_period_ticks,
        )
    }
}

impl core::fmt::Debug for Context {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Context")
            .field("max_frequency", &self.max_frequency)
            .field("cpu_frequency", &self.cpu_frequency)
            .field("cycles_per_us", &self.cycles_per_us)
            .field("half_period_ticks", &self.half_period_ticks)
            .finish()
    }
}

impl Context {
    fn swdio_to_input(&mut self) {
        defmt::trace!("SWDIO -> input");
        self.dir_swdio.set_low().ok();
        self.swdio_tms.into_input();
    }

    fn swdio_to_output(&mut self) {
        defmt::trace!("SWDIO/TMS -> output");
        self.swdio_tms.into_output_in_state(PinState::High);
        self.dir_swdio.set_high().ok();
    }

    fn swclk_to_input(&mut self) {
        defmt::trace!("SWCLK -> input");
        self.dir_swclk.set_low().ok();
        self.swclk_tck.into_input();
    }

    fn swclk_to_output(&mut self) {
        defmt::trace!("SWCLK/TCK -> output");
        self.swclk_tck.into_output_in_state(PinState::High);
        self.dir_swclk.set_high().ok();
    }

    fn from_pins(
        swdio: SwdioTmsPin,
        swclk: SwclkTckPin,
        tdi: TdiPin,
        tdo: TdoSwoPin,
        nreset: ResetPin,
        mut dir_swdio: DirSwdioPin,
        mut dir_swclk: DirSwclkPin,
        cpu_frequency: u32,
        delay: &'static Delay,
    ) -> Self {
        dir_swdio.set_low().ok();
        dir_swclk.set_low().ok();
        defmt::trace!("SWCLK -> input");
        defmt::trace!("SWDIO -> input");

        let max_frequency = 100_000;
        let half_period_ticks = cpu_frequency / max_frequency / 2;
        Context {
            max_frequency,
            cpu_frequency,
            cycles_per_us: cpu_frequency / 1_000_000,
            half_period_ticks,
            delay,
            swdio_tms: swdio,
            swclk_tck: swclk,
            tdi,
            tdo,
            nreset,
            dir_swdio,
            dir_swclk,
        }
    }
}

// DAP transaction stalled (WAIT) - slowing down
// ^ auto slowdown

impl swj::Dependencies<Swd, Jtag> for Context {
    fn process_swj_pins(&mut self, output: swj::Pins, mask: swj::Pins, wait_us: u32) -> swj::Pins {
        if mask.contains(swj::Pins::SWCLK) {
            self.swclk_to_output();
            if output.contains(swj::Pins::SWCLK) {
                self.swclk_tck.set_high();
            } else {
                self.swclk_tck.set_low();
            }
        }

        if mask.contains(swj::Pins::SWDIO) {
            self.swdio_to_output();
            if output.contains(swj::Pins::SWDIO) {
                self.swdio_tms.set_high();
            } else {
                self.swdio_tms.set_low();
            }
        }

        if mask.contains(swj::Pins::NRESET) {
            if output.contains(swj::Pins::NRESET) {
                // "open drain disconnect"
                self.nreset.into_input();
            } else {
                self.nreset.into_output_in_state(PinState::Low);
            }
        }

        // Delay until desired output state or timeout.
        let mut last = self.delay.get_current();
        for _ in 0..wait_us {
            last = self.delay.delay_ticks_from_last(self.cycles_per_us, last);

            // If a pin is selected, make sure its output equals the desired output state, or else
            // continue waiting.
            let swclk_not_in_desired_state = mask.contains(swj::Pins::SWCLK)
                && output.contains(swj::Pins::SWCLK) != self.swclk_tck.is_high();
            let swdio_not_in_desired_state = mask.contains(swj::Pins::SWDIO)
                && output.contains(swj::Pins::SWDIO) != self.swdio_tms.is_high();
            let nreset_not_in_desired_state = mask.contains(swj::Pins::NRESET)
                && output.contains(swj::Pins::NRESET) != self.nreset.is_high();

            if swclk_not_in_desired_state
                || swdio_not_in_desired_state
                || nreset_not_in_desired_state
            {
                continue;
            }

            break;
        }

        let mut ret = swj::Pins::empty();
        ret.set(swj::Pins::SWCLK, self.swclk_tck.is_high());
        ret.set(swj::Pins::SWDIO, self.swdio_tms.is_high());
        ret.set(swj::Pins::NRESET, self.nreset.is_high());

        trace!(
            "Running SWJ_pins: mask {:08b}, output: {:08b}, read: {:08b}",
            mask.bits(),
            output.bits(),
            ret.bits()
        );

        ret
    }

    fn process_swj_sequence(&mut self, data: &[u8], mut bits: usize) {
        self.swclk_to_output();
        self.swdio_to_output();

        let half_period_ticks = self.half_period_ticks;
        let mut last = self.delay.get_current();
        last = self.delay.delay_ticks_from_last(half_period_ticks, last);

        trace!("Running SWJ sequence: {:08b}, len = {}", data, bits);
        for byte in data {
            let mut byte = *byte;
            let frame_bits = core::cmp::min(bits, 8);
            for _ in 0..frame_bits {
                let bit = byte & 1;
                byte >>= 1;
                if bit != 0 {
                    self.swdio_tms.set_high();
                } else {
                    self.swdio_tms.set_low();
                }
                self.swclk_tck.set_low();
                last = self.delay.delay_ticks_from_last(half_period_ticks, last);
                self.swclk_tck.set_high();
                last = self.delay.delay_ticks_from_last(half_period_ticks, last);
            }
            bits -= frame_bits;
        }
    }

    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        trace!("Running SWJ clock");
        if max_frequency < self.cpu_frequency {
            self.max_frequency = max_frequency;
            self.half_period_ticks = self.cpu_frequency / self.max_frequency / 2;
            trace!("  freq = {}", max_frequency);
            trace!("  half_period_ticks = {}", self.half_period_ticks);
            true
        } else {
            false
        }
    }

    fn high_impedance_mode(&mut self) {
        self.swdio_to_input();
        self.swclk_to_input();
        self.nreset.into_input();
    }
}

//=======================================================================================
// JTAG interface
//=======================================================================================

pub struct Jtag {
    context: Context,
    // pins: &'ctx JtagPins,
}

impl From<Jtag> for Context {
    fn from(value: Jtag) -> Self {
        value.context
    }
}

impl From<Context> for Jtag {
    fn from(mut value: Context) -> Self {
        value.swdio_to_output();
        value.swclk_to_output();
        value.tdo.into_input();
        value.tdi.into_output_in_state(PinState::High);
        value.nreset.into_output_in_state(PinState::High);
        trace!("Context to JTAG");

        Self { context: value }
    }
}

// https://github.com/probe-rs/hs-probe-firmware/blob/master/firmware/src/jtag.rs
impl jtag::Jtag<Context> for Jtag {
    const AVAILABLE: bool = true;

    /// Handle a sequence request. The request data follows the CMSIS-DAP
    /// DAP_JTAG_Sequence command:
    /// * First byte contains the number of sequences, then
    /// * First byte of each sequence contains:
    ///     * Bits 5..0: Number of clock cycles, where 0 means 64 cycles
    ///     * Bit 6: TMS value
    ///     * Bit 7: TDO capture enable
    /// * Subsequent bytes of each sequence contain TDI data, one bit per
    ///   clock cycle, with the final byte padded. Data is transmitted from
    ///   successive bytes, least significant bit first.
    ///
    /// Captured TDO data is written least significant bit first to successive
    /// bytes of `rxbuf`, which must be long enough for the requested capture,
    /// or conservatively as long as `data`.
    /// The final byte of TDO data for each sequence is padded, in other words,
    /// as many TDO bytes will be returned as there were TDI bytes in sequences
    /// with capture enabled.
    ///
    /// Returns the number of bytes of rxbuf which were written to.
    fn sequences(&mut self, data: &[u8], rxbuf: &mut [u8]) -> u32 {
        trace!("JTAG sequences");
        // Read request header containing number of sequences.
        if data.is_empty() {
            return 0;
        };
        let nseqs = data[0];
        let mut data = &data[1..];
        let mut rxidx = 0;

        // Sanity check
        if nseqs == 0 || data.is_empty() {
            return 0;
        }

        let half_period_ticks = self.context.half_period_ticks;
        self.context.delay.delay_ticks(half_period_ticks);

        // Process alike sequences in one shot
        // if !self.context.use_bitbang.load(Ordering::SeqCst) {
        // let mut buffer = [0u8; DAP2_PACKET_SIZE as usize];
        // let mut buffer_idx = 0;
        // let transfer_type = data[0] & 0b1100_0000;
        // while nseqs > 0 {
        //     // Read header byte for this sequence.
        //     if data.is_empty() {
        //         break;
        //     };
        //     let header = data[0];
        //     if (header & 0b1100_0000) != transfer_type {
        //         // This sequence can't be processed in the same way
        //         break;
        //     }
        //     let nbits = header & 0b0011_1111;
        //     if nbits & 7 != 0 {
        //         // We can handle only 8*N bit sequences here
        //         break;
        //     }
        //     let nbits = if nbits == 0 { 64 } else { nbits as usize };
        //     let nbytes = Self::bytes_for_bits(nbits);

        //     if data.len() < (nbytes + 1) {
        //         break;
        //     };
        //     data = &data[1..];

        //     buffer[buffer_idx..buffer_idx + nbytes].copy_from_slice(&data[..nbytes]);
        //     buffer_idx += nbytes;
        //     nseqs -= 1;
        //     data = &data[nbytes..];
        // }
        // if buffer_idx > 0 {
        //     let capture = transfer_type & 0b1000_0000;
        //     let tms = transfer_type & 0b0100_0000;

        //     // Set TMS for this transfer.
        //     if tms != 0 {
        //         self.pins.tms.set_high();
        //     } else {
        //         self.pins.tms.set_low();
        //     }

        //     // self.spi_mode();
        //     // self.spi
        //     //     .jtag_exchange(self.dma, &buffer[..buffer_idx], &mut rxbuf[rxidx..]);
        //     if capture != 0 {
        //         rxidx += buffer_idx;
        //     }
        //     // Set TDI GPIO to the last bit the SPI peripheral transmitted,
        //     // to prevent it changing state when we set it to an output.
        //     self.pins.tdi.set_bool((buffer[buffer_idx - 1] >> 7) != 0);
        //     self.bitbang_mode();
        //     self.spi.disable();
        // }
        // }

        // Process each sequence.
        for _ in 0..nseqs {
            // Read header byte for this sequence.
            if data.is_empty() {
                break;
            }
            let header = data[0];
            data = &data[1..];
            let capture = header & 0b1000_0000;
            let tms = header & 0b0100_0000;
            let nbits = header & 0b0011_1111;
            let nbits = if nbits == 0 { 64 } else { nbits as usize };
            let nbytes = Self::bytes_for_bits(nbits);
            if data.len() < nbytes {
                break;
            }

            // Split data into TDI data for this sequence and data for remaining sequences.
            let tdi = &data[..nbytes];
            data = &data[nbytes..];

            // Set TMS for this transfer.
            if tms != 0 {
                self.context.swdio_tms.set_high();
            } else {
                self.context.swdio_tms.set_low();
            }

            // Run one transfer, either read-write or write-only.
            if capture != 0 {
                self.transfer_rw(nbits, tdi, &mut rxbuf[rxidx..]);
                rxidx += nbytes;
            } else {
                self.transfer_wo(nbits, tdi);
            }
        }

        rxidx as u32 // @fixme: coalescing for now
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        trace!("JTAG set clock {}", max_frequency);
        self.context.process_swj_clock(max_frequency)
    }
}

impl Jtag {
    #[inline(always)]
    fn wait_half_period(&self, last: u32) -> u32 {
        self.context
            .delay
            .delay_ticks_from_last(self.context.half_period_ticks, last)
    }

    /// Send a sequence of TMS bits.
    #[allow(dead_code)]
    #[inline(never)]
    fn tms_sequence(&mut self, data: &[u8], mut bits: usize) {
        // self.bitbang_mode();

        let mut last = self.context.delay.get_current();
        last = self.wait_half_period(last);

        for byte in data {
            let mut byte = *byte;
            let frame_bits = core::cmp::min(bits, 8);
            for _ in 0..frame_bits {
                let bit = byte & 1;
                byte >>= 1;

                if bit != 0 {
                    self.context.swdio_tms.set_high();
                } else {
                    self.context.swdio_tms.set_low();
                }
                self.context.swclk_tck.set_low();
                last = self.wait_half_period(last);
                self.context.swclk_tck.set_high();
                last = self.wait_half_period(last);
            }
            bits -= frame_bits;
        }
    }

    /// Write-only JTAG transfer without capturing TDO.
    ///
    /// Writes `n` bits from successive bytes of `tdi`, LSbit first.
    ///
    /// <----- bits of byte are transmitted in this order
    /// ----------------------------------------------------> bytes are transmitted in this order
    /// 8......0 8......0 8......0 8......0 8......0 8......0 8......0 8......0
    /// 01010101 10101010 01010101 10101010 01010101 10101010 01010101 10101010
    ///
    #[inline(never)]
    fn transfer_wo(&mut self, n: usize, tdi: &[u8]) {
        let mut last = self.context.delay.get_current();

        for (byte_idx, byte) in tdi.iter().enumerate() {
            for bit_idx in 0..8 {
                // Stop after transmitting `n` bits.
                if byte_idx * 8 + bit_idx == n {
                    return;
                }

                // Set TDI and toggle TCK.
                if byte & (1 << bit_idx) != 0 {
                    self.context.tdi.set_high();
                } else {
                    self.context.tdi.set_low();
                }
                last = self.wait_half_period(last);
                self.context.swclk_tck.set_high();
                last = self.wait_half_period(last);
                self.context.swclk_tck.set_low();
            }
        }
    }

    /// Read-write JTAG transfer, with TDO capture.
    ///
    /// Writes `n` bits from successive bytes of `tdi`, LSbit first.
    /// Captures `n` bits from TDO and writes into successive bytes of `tdo`, LSbit first.
    #[inline(never)]
    fn transfer_rw(&mut self, n: usize, tdi: &[u8], tdo: &mut [u8]) {
        let mut last = self.context.delay.get_current();

        for (byte_idx, (tdi, tdo)) in tdi.iter().zip(tdo.iter_mut()).enumerate() {
            *tdo = 0;
            for bit_idx in 0..8 {
                // Stop after transmitting `n` bits.
                if byte_idx * 8 + bit_idx == n {
                    return;
                }

                // We set TDI half a period before the clock rising edge where it is sampled
                // by the target, and we sample TDO immediately before the clock falling edge
                // where it is updated by the target.
                if tdi & (1 << bit_idx) != 0 {
                    self.context.tdi.set_high();
                } else {
                    self.context.tdi.set_low();
                }
                last = self.wait_half_period(last);
                self.context.swclk_tck.set_high();
                last = self.wait_half_period(last);
                if self.context.tdo.is_high() {
                    *tdo |= 1 << bit_idx;
                }
                self.context.swclk_tck.set_low();
            }
        }
    }

    /// Compute required number of bytes to store a number of bits.
    fn bytes_for_bits(bits: usize) -> usize {
        (bits + 7) / 8
    }
}

//=======================================================================================
// SWD interface
//=======================================================================================

#[derive(Debug, defmt::Format)]
pub struct Swd {
    context: Context,
}

impl From<Swd> for Context {
    fn from(value: Swd) -> Self {
        value.context
    }
}

impl From<Context> for Swd {
    fn from(mut value: Context) -> Self {
        // Maybe this should go to some `Swd::new`
        value.swdio_to_output();
        value.swclk_to_output();
        value.nreset.into_input();

        Self {
            context: value,
            // pins: value.into(),
        }
    }
}

impl swd::Swd<Context> for Swd {
    const AVAILABLE: bool = true;

    fn read_inner(&mut self, apndp: swd::APnDP, a: swd::DPRegister) -> swd::Result<u32> {
        trace!("SWD read, apndp: {}, addr: {}", apndp, a,);
        // Send request
        let req = swd::make_request(apndp, swd::RnW::R, a);
        trace!("SWD tx request");
        self.tx8(req);

        trace!("SWD rx ack");
        // Read ack, 1 clock for turnaround and 3 for ACK
        let ack = self.rx4() >> 1;

        match swd::Ack::try_ok(ack as u8) {
            Ok(_) => trace!("    ack ok"),
            Err(e) => {
                trace!("    ack error: {}", e);
                // On non-OK ACK, target has released the bus but
                // is still expecting a turnaround clock before
                // the next request, and we need to take over the bus.
                self.tx8(0);
                return Err(e);
            }
        }

        // Read data and parity
        trace!("SWD rx data");
        let (data, parity) = self.read_data();

        // Turnaround + trailing
        let mut last = self.context.delay.get_current();
        self.read_bit(&mut last);
        self.tx8(0); // Drive the SWDIO line to 0 to not float

        if parity as u8 == (data.count_ones() as u8 & 1) {
            trace!("    data: 0x{:x}", data);
            Ok(data)
        } else {
            Err(swd::Error::BadParity)
        }
    }

    fn write_inner(&mut self, apndp: swd::APnDP, a: swd::DPRegister, data: u32) -> swd::Result<()> {
        trace!(
            "SWD write, apndp: {}, addr: {}, data: 0x{:x}",
            apndp,
            a,
            data
        );

        // Send request
        let req = swd::make_request(apndp, swd::RnW::W, a);
        trace!("SWD tx request");
        self.tx8(req);

        // Read ack, 1 clock for turnaround and 3 for ACK and 1 for turnaround
        trace!("SWD rx ack");
        let ack = (self.rx5() >> 1) & 0b111;
        match swd::Ack::try_ok(ack as u8) {
            Ok(_) => trace!("    ack ok"),
            Err(e) => {
                trace!("    ack err: {}, data: {:b}", e, ack);
                // On non-OK ACK, target has released the bus but
                // is still expecting a turnaround clock before
                // the next request, and we need to take over the bus.
                self.tx8(0);
                return Err(e);
            }
        }

        // Send data and parity
        trace!("SWD tx data");
        let parity = data.count_ones() & 1 == 1;
        self.send_data(data, parity);

        // Send trailing idle
        self.tx8(0);

        Ok(())
    }

    fn write_sequence(&mut self, mut num_bits: usize, data: &[u8]) -> swd::Result<()> {
        self.context.swdio_to_output();
        let mut last = self.context.delay.get_current();

        for b in data {
            let bit_count = core::cmp::min(num_bits, 8);
            for i in 0..bit_count {
                self.write_bit((b >> i) & 0x1, &mut last);
            }
            num_bits -= bit_count;
        }

        Ok(())
    }

    fn read_sequence(&mut self, mut num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.context.swdio_to_input();
        let mut last = self.context.delay.get_current();

        for b in data {
            let bit_count = core::cmp::min(num_bits, 8);
            for i in 0..bit_count {
                let bit = self.read_bit(&mut last);
                *b |= bit << i;
            }
            num_bits -= bit_count;
        }

        Ok(())
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        trace!("SWD set clock: freq = {}", max_frequency);
        self.context.process_swj_clock(max_frequency)
    }
}

impl Swd {
    fn wait_half_period(&self, last: u32) -> u32 {
        self.context
            .delay
            .delay_ticks_from_last(self.context.half_period_ticks, last)
    }

    fn tx8(&mut self, mut data: u8) {
        self.context.swdio_to_output();

        let mut last = self.context.delay.get_current();

        for _ in 0..8 {
            self.write_bit(data & 1, &mut last);
            data >>= 1;
        }
    }

    fn rx4(&mut self) -> u8 {
        self.context.swdio_to_input();

        let mut data = 0;
        let mut last = self.context.delay.get_current();

        for i in 0..4 {
            data |= (self.read_bit(&mut last) & 1) << i;
        }

        data
    }

    fn rx5(&mut self) -> u8 {
        self.context.swdio_to_input();

        let mut last = self.context.delay.get_current();

        let mut data = 0;

        for i in 0..5 {
            data |= (self.read_bit(&mut last) & 1) << i;
        }

        data
    }

    fn send_data(&mut self, mut data: u32, parity: bool) {
        self.context.swdio_to_output();

        let mut last = self.context.delay.get_current();

        for _ in 0..32 {
            self.write_bit((data & 1) as u8, &mut last);
            data >>= 1;
        }

        self.write_bit(parity as u8, &mut last);
    }

    fn read_data(&mut self) -> (u32, bool) {
        self.context.swdio_to_input();

        let mut data = 0;

        let mut last = self.context.delay.get_current();

        for i in 0..32 {
            data |= (self.read_bit(&mut last) as u32 & 1) << i;
        }

        let parity = self.read_bit(&mut last) != 0;

        (data, parity)
    }

    #[inline(always)]
    fn write_bit(&mut self, bit: u8, last: &mut u32) {
        if bit != 0 {
            self.context.swdio_tms.set_high();
        } else {
            self.context.swdio_tms.set_low();
        }

        self.context.swclk_tck.set_low();
        *last = self.wait_half_period(*last);

        self.context.swclk_tck.set_high();
        *last = self.wait_half_period(*last);
    }

    #[inline(always)]
    fn read_bit(&mut self, last: &mut u32) -> u8 {
        self.context.swclk_tck.set_low();
        *last = self.wait_half_period(*last);

        let bit = self.context.swdio_tms.is_high() as u8;

        self.context.swclk_tck.set_high();
        *last = self.wait_half_period(*last);

        bit
    }
}

#[derive(Debug, defmt::Format)]
pub struct Swo {}

impl swo::Swo for Swo {
    fn set_transport(&mut self, _transport: swo::SwoTransport) {}

    fn set_mode(&mut self, _mode: swo::SwoMode) {}

    fn set_baudrate(&mut self, _baudrate: u32) -> u32 {
        0
    }

    fn set_control(&mut self, _control: swo::SwoControl) {}

    fn polling_data(&mut self, _buf: &mut [u8]) -> u32 {
        0
    }

    fn streaming_data(&mut self) {}

    fn is_active(&self) -> bool {
        false
    }

    fn bytes_available(&self) -> u32 {
        0
    }

    fn buffer_size(&self) -> u32 {
        0
    }

    fn support(&self) -> swo::SwoSupport {
        swo::SwoSupport {
            uart: false,
            manchester: false,
        }
    }

    fn status(&mut self) -> swo::SwoStatus {
        swo::SwoStatus {
            active: false,
            trace_error: false,
            trace_overrun: false,
            bytes_available: 0,
        }
    }
}

pub struct Wait {
    delay: &'static Delay,
}

impl Wait {
    pub fn new(delay: &'static Delay) -> Self {
        Wait { delay }
    }
}

impl DelayNs for Wait {
    fn delay_ns(&mut self, ns: u32) {
        self.delay.delay_us((ns / 1_000).max(1));
    }
}

#[inline(always)]
pub fn create_dap(
    version_string: &'static str,
    swdio_tms: SwdioTmsPin,
    swclk_tck: SwclkTckPin,
    tdi: TdiPin,
    swo_tdo: TdoSwoPin,
    nreset: ResetPin,
    dir_swdio: DirSwdioPin,
    dir_swclk: DirSwclkPin,
    cpu_frequency: u32,
    delay: &'static Delay,
    leds: crate::leds::HostStatusToken,
) -> crate::setup::DapHandler {
    let context = Context::from_pins(
        swdio_tms,
        swclk_tck,
        tdi,
        swo_tdo,
        nreset,
        dir_swdio,
        dir_swclk,
        cpu_frequency,
        delay,
    );
    let wait = Wait::new(delay);
    let swo = None;

    defmt::info!("Making dap interface with context: {}", context);

    dap::Dap::new(context, leds, wait, swo, version_string)
}
