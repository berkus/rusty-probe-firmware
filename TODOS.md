- DebugProbe::select_jtag_tap()
- ChainParams::from_jtag_chain()

---

impl<Probe: DebugProbe + RawJtagIo + 'static> JTAGAccess for Probe {
    fn scan_chain(&mut self) -> Result<(), DebugProbeError> {

DebugProbe::has_arm_interface()
DebugProbe::try_as_dap_probe()

/// Low-Level Access to the JTAG protocol
///
/// This trait should be implemented by all probes which offer low-level access to
/// the JTAG protocol, i.e. direction control over the bytes sent and received.
pub trait JTAGAccess: DebugProbe {

/// Represents a Jtag Tap within the chain.
#[derive(Debug)]
pub struct JtagChainItem {
    /// The IDCODE of the device.
    pub idcode: Option<IdCode>,

    /// The length of the instruction register.
    pub irlen: usize,
}

JTAGAccess?::write_register() -- missing for ARM JTAG?
