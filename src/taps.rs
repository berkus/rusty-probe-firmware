use crate::jtag::JtagState;

#[derive(Default, Copy, Clone)]
struct Tap {
    ir_len: usize,
}

const MAX_TAPS: usize = 8; // @fixme As per ADI v5.2 A1-35 "Choices for JTAG-APs"

pub struct Taps {
    // jtag state: we control state ourselves and the probe-rs knows to restart the SM after DAP_Transfer cmd...
    jtag_state: JtagState, // for switching read-write modes
    taps: [Tap; MAX_TAPS],
    num_taps: usize,
    active_tap_index: usize,
    active_tap: TapSelect,
}

#[derive(Default)]
struct TapSelect {
    /// This many IR bits before selected TAP
    ir_prev: usize,
    /// This many TAPs before selected
    dr_prev: usize,
    /// IR bits in the selected TAP
    ir_len: usize,
    /// This many IR bits after the selected TAP
    ir_post: usize,
    /// This many TAPs after the selected TAP
    dr_post: usize,
}

impl Default for Taps {
    fn default() -> Self {
        Self {
            jtag_state: JtagState::Reset,
            taps: [Tap::default(); MAX_TAPS],
            num_taps: 0,
            active_tap_index: 0,
            active_tap: TapSelect::default(),
        }
    }
}

impl Taps {
    pub fn setup(&mut self, chain_count: usize, ir_lens: &[u8]) {
        assert!(chain_count <= MAX_TAPS); // replace with ERROR?

        for i in 0..chain_count {
            self.taps[i].ir_len = ir_lens[i].into();
        }
        self.num_taps = chain_count;
    }

    /// Select which TAP in the scan chain to operate upon.  `ir` will be shifted into its
    /// instruction register, and the other TAPs put into bypass.
    pub fn select_tap(&mut self, tap: usize, ir: &[u8]) {
        assert!(tap <= self.num_taps); // make into an ERROR!

        // Reset JTAG sm!
        // tms_sequence([1,1,1,1,1,1], boom); // self.jtag_state = JtagState::Reset;

        // self.sm.mode_reset();
        self.active_tap_index = tap;
        self.write_ir(ir);
    }

    /// Write IR for the currently selected TAP
    fn write_ir(&mut self, ir: &[u8]) {}

    /// Read IR from the currently selected TAP
    fn read_ir() {}

    /// Write DR for the currently selected TAP
    fn write_dr() {}

    /// Read DR from the currently selected TAP
    fn read_dr() {}
}
