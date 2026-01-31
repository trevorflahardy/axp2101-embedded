//! Register addresses and constants for AXP2101
//!
//! This module defines all register addresses, voltage ranges, and step sizes
//! for the AXP2101 Power Management IC.

/// AXP2101 Register Addresses
/// Based on AXP2101 datasheet
///
/// Device type/ID register - Contains chip identification
pub const AXP2101_IC_TYPE: u8 = 0x03;

/// Expected chip ID value for AXP2101
pub const AXP2101_CHIP_ID: u8 = 0x47;

/// Status register 1 - Contains VBUS good, BATFET state, battery connection/active status, thermal regulation, and current limit status
pub const AXP2101_STATUS1: u8 = 0x00;

/// Status register 2 - Contains power-on status, charging state (tri-charge, pre-charge, CC, CV, done), and charger status
pub const AXP2101_STATUS2: u8 = 0x01;

/// Data buffer register 1 - First of 6 bytes for user data storage
pub const AXP2101_DATA_BUFFER1: u8 = 0x04;

/// Size of the data buffer in bytes (6 bytes total)
pub const AXP2101_DATA_BUFFER_SIZE: usize = 6;

/// Common configuration register - Controls internal discharge, PWROK pin, PWRON shutdown, reset, and system shutdown
pub const AXP2101_COMMON_CONFIG: u8 = 0x10;

/// BATFET control register - Manages battery FET enable/disable and discharge control
pub const AXP2101_BATFET_CTRL: u8 = 0x12;

/// Die temperature control register - Configuration for internal temperature monitoring
pub const AXP2101_DIE_TEMP_CTRL: u8 = 0x13;

/// Minimum system voltage control - Sets the minimum voltage threshold for system operation
pub const AXP2101_MIN_SYS_VOL_CTRL: u8 = 0x14;

/// Input voltage limit control - Sets VBUS voltage limit thresholds
pub const AXP2101_INPUT_VOL_LIMIT_CTRL: u8 = 0x15;

/// Input current limit control - Configures maximum input current from VBUS
pub const AXP2101_INPUT_CUR_LIMIT_CTRL: u8 = 0x16;

/// Reset fuel gauge register - Triggers reset of battery percentage tracking
pub const AXP2101_RESET_FUEL_GAUGE: u8 = 0x18;

/// Fuel gauge control register - Controls fuel gauge ROM access and data interface
pub const AXP2101_FUEL_GAUGE_CTRL: u8 = 0x19;

/// Battery parameters register - Stores or reads 128-byte fuel gauge configuration data
pub const AXP2101_BAT_PARAMS: u8 = 0x1A;

/// DCDC on/off and DVM control - Enables/disables DC1-5, CCM mode, and DVM ramp speed
pub const AXP2101_DC_ONOFF_DVM_CTRL: u8 = 0x80;

/// DCDC1 voltage control register - Sets output voltage for DC1 (1500-3400mV in 100mV steps)
pub const AXP2101_DC_VOL0_CTRL: u8 = 0x82;

/// DCDC2 voltage control register - Sets output voltage for DC2 (500-1540mV)
pub const AXP2101_DC_VOL1_CTRL: u8 = 0x83;

/// DCDC3 voltage control register - Sets output voltage for DC3 (500-3400mV)
pub const AXP2101_DC_VOL2_CTRL: u8 = 0x84;

/// DCDC4 voltage control register - Sets output voltage for DC4 (500-1840mV)
pub const AXP2101_DC_VOL3_CTRL: u8 = 0x85;

/// DCDC5 voltage control register - Sets output voltage for DC5 (1400-3700mV or 1200mV)
pub const AXP2101_DC_VOL4_CTRL: u8 = 0x86;

/// LDO control
pub const AXP2101_LDO_ONOFF_CTRL0: u8 = 0x90;
pub const AXP2101_LDO_ONOFF_CTRL1: u8 = 0x91;
pub const AXP2101_LDO_VOL0_CTRL: u8 = 0x92; // ALDO1
pub const AXP2101_LDO_VOL1_CTRL: u8 = 0x93; // ALDO2
pub const AXP2101_LDO_VOL2_CTRL: u8 = 0x94; // ALDO3
pub const AXP2101_LDO_VOL3_CTRL: u8 = 0x95; // ALDO4
pub const AXP2101_LDO_VOL4_CTRL: u8 = 0x96; // BLDO1
pub const AXP2101_LDO_VOL5_CTRL: u8 = 0x97; // BLDO2
pub const AXP2101_LDO_VOL6_CTRL: u8 = 0x98; // CPUSLDO
pub const AXP2101_LDO_VOL7_CTRL: u8 = 0x99; // DLDO1
pub const AXP2101_LDO_VOL8_CTRL: u8 = 0x9A; // DLDO2

/// Watchdog and gauge control
pub const AXP2101_CHARGE_GAUGE_WDT_CTRL: u8 = 0x18;
pub const AXP2101_WDT_CTRL: u8 = 0x19;
pub const AXP2101_LOW_BAT_WARN_SET: u8 = 0x15;

/// Power on/off control
pub const AXP2101_PWRON_STATUS: u8 = 0x20;
pub const AXP2101_PWROFF_STATUS: u8 = 0x21;
pub const AXP2101_PWROFF_EN: u8 = 0x22;
pub const AXP2101_DC_OVP_UVP_CTRL: u8 = 0x23;
pub const AXP2101_VOFF_SET: u8 = 0x24;
pub const AXP2101_PWROK_SEQU_CTRL: u8 = 0x25;
pub const AXP2101_SLEEP_WAKEUP_CTRL: u8 = 0x26;
pub const AXP2101_IRQ_OFF_ON_LEVEL_CTRL: u8 = 0x27;

/// Fast power on control
pub const AXP2101_FAST_PWRON_SET0: u8 = 0x28;
pub const AXP2101_FAST_PWRON_SET1: u8 = 0x29;
pub const AXP2101_FAST_PWRON_SET2: u8 = 0x2A;
pub const AXP2101_FAST_PWRON_CTRL: u8 = 0x2B;

/// DC force PWM control
pub const AXP2101_DC_FORCE_PWM_CTRL: u8 = 0x2C;

/// Button battery charge
pub const AXP2101_BTN_BAT_CHG_VOL_SET: u8 = 0x7C;

/// Battery detection
pub const AXP2101_BAT_DET_CTRL: u8 = 0x34;

/// ADC control
pub const AXP2101_ADC_CHANNEL_CTRL: u8 = 0x30;
pub const AXP2101_ADC_DATA_RELUST0: u8 = 0x34; // Battery voltage H
pub const AXP2101_ADC_DATA_RELUST1: u8 = 0x35; // Battery voltage L
pub const AXP2101_ADC_DATA_RELUST2: u8 = 0x36; // TS pin H
pub const AXP2101_ADC_DATA_RELUST3: u8 = 0x37; // TS pin L
pub const AXP2101_ADC_DATA_RELUST4: u8 = 0x38; // VBUS voltage H
pub const AXP2101_ADC_DATA_RELUST5: u8 = 0x39; // VBUS voltage L
pub const AXP2101_ADC_DATA_RELUST6: u8 = 0x3A; // VSYS voltage H
pub const AXP2101_ADC_DATA_RELUST7: u8 = 0x3B; // VSYS voltage L
pub const AXP2101_ADC_DATA_RELUST8: u8 = 0x3C; // Die temperature H
pub const AXP2101_ADC_DATA_RELUST9: u8 = 0x3D; // Die temperature L

/// Battery percentage
pub const AXP2101_BAT_PERCENT_DATA: u8 = 0xA4;

/// TS pin control
pub const AXP2101_TS_PIN_CTRL: u8 = 0x50;

/// Charging control
pub const AXP2101_CHGLED_SET_CTRL: u8 = 0x69;
pub const AXP2101_IPRECHG_SET: u8 = 0x61;
pub const AXP2101_ICC_CHG_SET: u8 = 0x62;
pub const AXP2101_ITERM_CHG_SET_CTRL: u8 = 0x63;
pub const AXP2101_CV_CHG_VOL_SET: u8 = 0x64;
pub const AXP2101_THE_REGU_THRES_SET: u8 = 0x65;

/// Interrupt control
pub const AXP2101_INTEN1: u8 = 0x40;
pub const AXP2101_INTEN2: u8 = 0x41;
pub const AXP2101_INTEN3: u8 = 0x42;
pub const AXP2101_INTSTS1: u8 = 0x48;
pub const AXP2101_INTSTS2: u8 = 0x49;
pub const AXP2101_INTSTS3: u8 = 0x4A;
pub const AXP2101_INTSTS_CNT: usize = 3;

/// I2C Address
pub const AXP2101_SLAVE_ADDRESS: u8 = 0x34;

// DCDC voltage ranges and steps
pub const XPOWERS_AXP2101_DCDC1_VOL_MIN: u16 = 1500;
pub const XPOWERS_AXP2101_DCDC1_VOL_MAX: u16 = 3400;
pub const XPOWERS_AXP2101_DCDC1_VOL_STEPS: u16 = 100;

pub const XPOWERS_AXP2101_DCDC2_VOL1_MIN: u16 = 500;
pub const XPOWERS_AXP2101_DCDC2_VOL1_MAX: u16 = 1200;
pub const XPOWERS_AXP2101_DCDC2_VOL_STEPS1: u16 = 10;
pub const XPOWERS_AXP2101_DCDC2_VOL2_MIN: u16 = 1220;
pub const XPOWERS_AXP2101_DCDC2_VOL2_MAX: u16 = 1540;
pub const XPOWERS_AXP2101_DCDC2_VOL_STEPS2: u16 = 20;
pub const XPOWERS_AXP2101_DCDC2_VOL_STEPS2_BASE: u16 = 71;

pub const XPOWERS_AXP2101_DCDC3_VOL_MIN: u16 = 500;
pub const XPOWERS_AXP2101_DCDC3_VOL1_MIN: u16 = 500;
pub const XPOWERS_AXP2101_DCDC3_VOL1_MAX: u16 = 1200;
pub const XPOWERS_AXP2101_DCDC3_VOL_STEPS1: u16 = 10;
pub const XPOWERS_AXP2101_DCDC3_VOL2_MIN: u16 = 1220;
pub const XPOWERS_AXP2101_DCDC3_VOL2_MAX: u16 = 1540;
pub const XPOWERS_AXP2101_DCDC3_VOL_STEPS2: u16 = 20;
pub const XPOWERS_AXP2101_DCDC3_VOL_STEPS2_BASE: u16 = 71;
pub const XPOWERS_AXP2101_DCDC3_VOL3_MIN: u16 = 1600;
pub const XPOWERS_AXP2101_DCDC3_VOL3_MAX: u16 = 3400;
pub const XPOWERS_AXP2101_DCDC3_VOL_STEPS3: u16 = 100;
pub const XPOWERS_AXP2101_DCDC3_VOL_STEPS3_BASE: u16 = 88;

pub const XPOWERS_AXP2101_DCDC4_VOL1_MIN: u16 = 500;
pub const XPOWERS_AXP2101_DCDC4_VOL1_MAX: u16 = 1200;
pub const XPOWERS_AXP2101_DCDC4_VOL_STEPS1: u16 = 10;
pub const XPOWERS_AXP2101_DCDC4_VOL2_MIN: u16 = 1220;
pub const XPOWERS_AXP2101_DCDC4_VOL2_MAX: u16 = 1840;
pub const XPOWERS_AXP2101_DCDC4_VOL_STEPS2: u16 = 20;
pub const XPOWERS_AXP2101_DCDC4_VOL_STEPS2_BASE: u16 = 71;

pub const XPOWERS_AXP2101_DCDC5_VOL_MIN: u16 = 1400;
pub const XPOWERS_AXP2101_DCDC5_VOL_MAX: u16 = 3700;
pub const XPOWERS_AXP2101_DCDC5_VOL_STEPS: u16 = 100;
pub const XPOWERS_AXP2101_DCDC5_VOL_1200MV: u16 = 1200;
pub const XPOWERS_AXP2101_DCDC5_VOL_VAL: u16 = 24;

// LDO voltage ranges and steps
pub const XPOWERS_AXP2101_ALDO1_VOL_MIN: u16 = 500;
pub const XPOWERS_AXP2101_ALDO1_VOL_MAX: u16 = 3500;
pub const XPOWERS_AXP2101_ALDO1_VOL_STEPS: u16 = 100;

pub const XPOWERS_AXP2101_ALDO2_VOL_MIN: u16 = 500;
pub const XPOWERS_AXP2101_ALDO2_VOL_MAX: u16 = 3500;
pub const XPOWERS_AXP2101_ALDO2_VOL_STEPS: u16 = 100;

pub const XPOWERS_AXP2101_ALDO3_VOL_MIN: u16 = 500;
pub const XPOWERS_AXP2101_ALDO3_VOL_MAX: u16 = 3500;
pub const XPOWERS_AXP2101_ALDO3_VOL_STEPS: u16 = 100;

pub const XPOWERS_AXP2101_ALDO4_VOL_MIN: u16 = 500;
pub const XPOWERS_AXP2101_ALDO4_VOL_MAX: u16 = 3500;
pub const XPOWERS_AXP2101_ALDO4_VOL_STEPS: u16 = 100;

pub const XPOWERS_AXP2101_BLDO1_VOL_MIN: u16 = 500;
pub const XPOWERS_AXP2101_BLDO1_VOL_MAX: u16 = 3500;
pub const XPOWERS_AXP2101_BLDO1_VOL_STEPS: u16 = 100;

pub const XPOWERS_AXP2101_BLDO2_VOL_MIN: u16 = 500;
pub const XPOWERS_AXP2101_BLDO2_VOL_MAX: u16 = 3500;
pub const XPOWERS_AXP2101_BLDO2_VOL_STEPS: u16 = 100;

pub const XPOWERS_AXP2101_CPUSLDO_VOL_MIN: u16 = 500;
pub const XPOWERS_AXP2101_CPUSLDO_VOL_MAX: u16 = 1400;
pub const XPOWERS_AXP2101_CPUSLDO_VOL_STEPS: u16 = 50;

pub const XPOWERS_AXP2101_DLDO1_VOL_MIN: u16 = 500;
pub const XPOWERS_AXP2101_DLDO1_VOL_MAX: u16 = 3400;
pub const XPOWERS_AXP2101_DLDO1_VOL_STEPS: u16 = 100;

pub const XPOWERS_AXP2101_DLDO2_VOL_MIN: u16 = 500;
pub const XPOWERS_AXP2101_DLDO2_VOL_MAX: u16 = 1400;
pub const XPOWERS_AXP2101_DLDO2_VOL_STEPS: u16 = 50;

// Button battery voltage
pub const XPOWERS_AXP2101_BTN_VOL_MIN: u16 = 2000;
pub const XPOWERS_AXP2101_BTN_VOL_MAX: u16 = 3500;
pub const XPOWERS_AXP2101_BTN_VOL_STEPS: u16 = 200;

// VSYS threshold
pub const XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_MIN: u16 = 2600;
pub const XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_MAX: u16 = 3300;
pub const XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_STEPS: u16 = 100;

// Temperature conversion
pub const XPOWERS_AXP2101_CONVERSION_FACTOR: f32 = 0.1;
#[inline]
pub fn temp_conversion(raw: u16) -> f32 {
    (raw as f32) * XPOWERS_AXP2101_CONVERSION_FACTOR - 267.7
}
