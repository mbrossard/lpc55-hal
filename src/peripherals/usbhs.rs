use core::ops::Deref;
use embedded_time::duration::Extensions;

use crate::drivers::timer;
use crate::peripherals::{anactrl, ctimer, pmc, syscon};
use crate::raw;
use crate::typestates::{
    // ValidUsbClockToken,
    // Fro96MHzEnabledToken,
    ClocksSupportUsbhsToken,
    init_state,
    usbhs_mode,
};

use crate::traits::usb::{Usb, UsbSpeed};

// USBPHY_CTRL bits, written through the one-shot CTRL_SET/CTRL_CLR aliases.
const CTRL_ENAUTOCLR_CLKGATE: u32 = 1 << 19;
const CTRL_CLKGATE: u32 = 1 << 30;
const CTRL_SFTRST: u32 = 1 << 31;

// USBPHY_PLL_SIC bits, written through the one-shot PLL_SIC_SET/PLL_SIC_CLR aliases.
const PLL_SIC_EN_USB_CLKS: u32 = 1 << 6;
const PLL_SIC_POWER: u32 = 1 << 12;
// Reserved; UM11126 rev 2.8 Table 850 says software must write it 0, and it resets to 1.
const PLL_SIC_RESERVED_16: u32 = 1 << 16;
const PLL_SIC_REG_ENABLE: u32 = 1 << 21;

/// How many times to issue `CTRL_CLR = CLKGATE` before giving up on the read-back confirming it.
///
/// The clear is not reliably taken on the first write after the PHY has been through a soft
/// reset, and the vendor's guidance for this PHY macro is to read the bit back rather than assume
/// one write lands. A second write has always been enough; four bounds the loop with margin and
/// costs nothing on the path where the first write works.
const CLKGATE_CLEAR_ATTEMPTS: u32 = 4;

// Main struct
pub struct Usbhs<
    State: init_state::InitState = init_state::Unknown,
    Mode: usbhs_mode::UsbhsMode = usbhs_mode::Unknown,
> {
    pub(crate) raw_phy: raw::USBPHY,
    pub(crate) raw_hsd: raw::USB1,
    pub(crate) raw_hsh: raw::USBHSH,
    _state: State,
    _mode: Mode,
}

pub type EnabledUsbhsDevice = Usbhs<init_state::Enabled, usbhs_mode::Device>;
pub type EnabledUsbhsHost = Usbhs<init_state::Enabled, usbhs_mode::Host>;

impl Deref for EnabledUsbhsDevice {
    type Target = raw::usb1::RegisterBlock;
    fn deref(&self) -> &Self::Target {
        &self.raw_hsd
    }
}

unsafe impl Sync for EnabledUsbhsDevice {}

impl Usb<init_state::Enabled> for EnabledUsbhsDevice {
    const SPEED: UsbSpeed = UsbSpeed::HighSpeed;
    // const NUM_ENDPOINTS: usize = 1 + 5;
}

impl Usbhs {
    pub fn new(raw_phy: raw::USBPHY, raw_hsd: raw::USB1, raw_hsh: raw::USBHSH) -> Self {
        Usbhs {
            raw_phy,
            raw_hsd,
            raw_hsh,
            _state: init_state::Unknown,
            _mode: usbhs_mode::Unknown,
        }
    }
}

impl<State: init_state::InitState, Mode: usbhs_mode::UsbhsMode> Usbhs<State, Mode> {
    pub fn release(self) -> (raw::USB1, raw::USBHSH) {
        (self.raw_hsd, self.raw_hsh)
    }

    pub fn enabled_as_device(
        mut self,
        anactrl: &mut anactrl::Anactrl,
        pmc: &mut pmc::Pmc,
        syscon: &mut syscon::Syscon,
        timer: &mut timer::Timer<impl ctimer::Ctimer<init_state::Enabled>>,
        // lock_fro_to_sof: bool, // we always lock to SOF
        _clocks_token: ClocksSupportUsbhsToken,
    ) -> EnabledUsbhsDevice {
        // Reset devices
        syscon.reset(&mut self.raw_hsh);
        syscon.reset(&mut self.raw_hsd);
        syscon.reset(&mut self.raw_phy);

        // Briefly turn on host controller to enable device control of USB1 port
        syscon.enable_clock(&mut self.raw_hsh);

        self.raw_hsh
            .portmode
            .modify(|_, w| w.dev_enable().set_bit());

        syscon.disable_clock(&mut self.raw_hsh);

        // Power on 32M crystal for HS PHY and connect to USB PLL
        pmc.raw
            .pdruncfg0
            .modify(|_, w| w.pden_xtal32m().poweredon());
        pmc.raw
            .pdruncfg0
            .modify(|_, w| w.pden_ldoxo32m().poweredon());
        anactrl
            .raw
            .xo32m_ctrl
            .modify(|_, w| w.enable_pll_usb_out().set_bit());

        pmc.power_on(&mut self.raw_phy);

        // Give long delay for PHY to be ready
        timer.start(5000_u32.microseconds());
        nb::block!(timer.wait()).ok();

        syscon.enable_clock(&mut self.raw_phy);

        // Initial config of PHY control registers.
        //
        // Every access below goes through the one-shot SET/CLR aliases, never a read-modify-write
        // of a live PHY control register. UM11126 rev 2.8, section 44.3 and the SDK
        // (`CLOCK_EnableUsbhs0PhyPllClock`) both do it that way: an RMW of USBPHY_CTRL rewrites
        // all 32 bits, including the reset and clock-gate controls, which is not the same
        // operation as clearing one bit.
        //
        // SFTRST and CLKGATE are cleared by two separate writes, and in that order. Hardware
        // forces CLKGATE set while SFTRST is asserted, so a combined clear would release the
        // reset and silently leave the PHY gated.
        self.raw_phy
            .ctrl_clr
            .write(|w| unsafe { w.bits(CTRL_SFTRST) });

        self.raw_phy
            .pll_sic
            .modify(|_, w| w.pll_div_sel().bits(6) /* 16MHz = xtal32m */);

        self.raw_phy
            .pll_sic_set
            .write(|w| unsafe { w.bits(PLL_SIC_REG_ENABLE) });

        self.raw_phy
            .pll_sic_clr
            .write(|w| unsafe { w.bits(PLL_SIC_RESERVED_16) });

        // Must wait at least 15 us for pll-reg to stabilize
        timer.start(15.microseconds());
        nb::block!(timer.wait()).ok();

        self.raw_phy
            .pll_sic_set
            .write(|w| unsafe { w.bits(PLL_SIC_POWER) });

        self.raw_phy
            .pll_sic_set
            .write(|w| unsafe { w.bits(PLL_SIC_EN_USB_CLKS) });

        // Ungate the PHY here, after PLL_EN_USB_CLKS, which is where 44.3 puts it -- and confirm
        // it. A clear issued after the PHY has been soft-reset can be dropped, and a PHY left
        // gated has a locked PLL, a responsive register file and no D+ pull-up, so nothing
        // downstream of this point would notice.
        for _ in 0..CLKGATE_CLEAR_ATTEMPTS {
            self.raw_phy
                .ctrl_clr
                .write(|w| unsafe { w.bits(CTRL_CLKGATE) });
            if self.raw_phy.ctrl.read().bits() & CTRL_CLKGATE == 0 {
                break;
            }
        }

        // Turn on everything in PHY
        self.raw_phy.pwd.write(|w| unsafe { w.bits(0) });

        // ENAUTOCLR_PHY_PWD is already 0 out of reset, so nothing here needs to clear it.
        self.raw_phy
            .ctrl_set
            .write(|w| unsafe { w.bits(CTRL_ENAUTOCLR_CLKGATE) });

        // turn on USB1 device controller access
        syscon.enable_clock(&mut self.raw_hsd);

        Usbhs {
            raw_phy: self.raw_phy,
            raw_hsd: self.raw_hsd,
            raw_hsh: self.raw_hsh,
            _state: init_state::Enabled(()),
            _mode: usbhs_mode::Device,
        }
    }

    pub fn borrow<F: Fn(&mut Self)>(&mut self, func: F) {
        func(self);
    }
}

#[derive(Debug)]
pub struct UsbHsDevInfo {
    pub maj_rev: u8,
    pub min_rev: u8,
    pub err_code: u8,
    pub frame_nr: u16,
}

impl EnabledUsbhsDevice {
    pub fn info(&self) -> UsbHsDevInfo {
        // technically, e.g. maj/min rev need only the clock, and not the power enabled
        UsbHsDevInfo {
            maj_rev: self.raw_hsd.info.read().majrev().bits(),
            min_rev: self.raw_hsd.info.read().minrev().bits(),
            err_code: self.raw_hsd.info.read().err_code().bits(),
            frame_nr: self.raw_hsd.info.read().frame_nr().bits(),
        }
    }

    pub fn disable_high_speed(&mut self) {
        // Note: Application Note https://www.nxp.com/docs/en/application-note/TN00071.zip
        // states that devcmdstat.force_fs (bit 21) might also be used.
        self.raw_phy.pwd_set.write(|w| unsafe {
            w.bits(1 << 12) /* TXPWDV2I */
        });
    }
}

impl<State: init_state::InitState> Usbhs<State, usbhs_mode::Device> {
    /// Disables the USB HS peripheral, assumed in device mode
    pub fn disabled(
        mut self,
        pmc: &mut pmc::Pmc,
        syscon: &mut syscon::Syscon,
    ) -> Usbhs<init_state::Disabled, usbhs_mode::Device> {
        syscon.disable_clock(&mut self.raw_hsd);

        syscon.disable_clock(&mut self.raw_phy);

        pmc.power_off(&mut self.raw_phy);

        pmc.raw
            .pdruncfg0
            .modify(|_, w| w.pden_xtal32m().poweredoff());
        pmc.raw
            .pdruncfg0
            .modify(|_, w| w.pden_ldoxo32m().poweredoff());

        Usbhs {
            raw_phy: self.raw_phy,
            raw_hsd: self.raw_hsd,
            raw_hsh: self.raw_hsh,
            _state: init_state::Disabled,
            _mode: usbhs_mode::Device,
        }
    }
}

impl From<(raw::USBPHY, raw::USB1, raw::USBHSH)> for Usbhs {
    fn from(raw: (raw::USBPHY, raw::USB1, raw::USBHSH)) -> Self {
        Usbhs::new(raw.0, raw.1, raw.2)
    }
}
