//! Type definitions and enumerations for AXP2101 configuration
//!
//! This module provides strongly-typed enumerations for configuring
//! various aspects of the AXP2101 PMIC.

/// Power channel identifiers
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PowerChannel {
    Dcdc1 = 0,
    Dcdc2 = 1,
    Dcdc3 = 2,
    Dcdc4 = 3,
    Dcdc5 = 4,
    Aldo1 = 5,
    Aldo2 = 6,
    Aldo3 = 7,
    Aldo4 = 8,
    Bldo1 = 9,
    Bldo2 = 10,
    CpuSldo = 11,
    Dldo1 = 12,
    Dldo2 = 13,
}

/// IRQ timing configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum IrqTime {
    Time1s = 0,
    Time1s5 = 1,
    Time2s = 2,
    Time2s5 = 3,
}

/// Precharge current settings
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PrechargeCurrent {
    I0mA = 0,
    I25mA = 1,
    I50mA = 2,
    I75mA = 3,
    I100mA = 4,
    I125mA = 5,
    I150mA = 6,
    I175mA = 7,
    I200mA = 8,
}

/// Charge termination current
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ChargeTerminationCurrent {
    I0mA = 0,
    I25mA = 1,
    I50mA = 2,
    I75mA = 3,
    I100mA = 4,
    I125mA = 5,
    I150mA = 6,
    I175mA = 7,
    I200mA = 8,
}

/// Thermal regulation threshold
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ThermalThreshold {
    Temp60C = 0,
    Temp80C = 1,
    Temp100C = 2,
    Temp120C = 3,
}

/// Charge status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChargeStatus {
    TriCharge = 0,
    PreCharge = 1,
    ConstantCurrent = 2,
    ConstantVoltage = 3,
    ChargeDone = 4,
    NotCharging = 5,
}

/// Wakeup configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeupSource {
    IrqPinLow = 0b10000,
    PwrOkLow = 0b01000,
    DcDlo = 0b00100,
}

/// Fast power-on options
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FastPowerOnChannel {
    Dcdc1 = 0,
    Dcdc2 = 1,
    Dcdc3 = 2,
    Dcdc4 = 3,
    Dcdc5 = 4,
    Aldo1 = 5,
    Aldo2 = 6,
    Aldo3 = 7,
    Aldo4 = 8,
    Bldo1 = 9,
    Bldo2 = 10,
    CpuSldo = 11,
    Dldo1 = 12,
    Dldo2 = 13,
}

/// Start sequence level
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SequenceLevel {
    Level0 = 0,
    Level1 = 1,
    Level2 = 2,
    Disable = 3,
}

/// Watchdog configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WatchdogConfig {
    IrqOnly = 0,
    IrqAndReset = 1,
    IrqResetPullDownPwrOk = 2,
    IrqResetAllOff = 3,
}

/// Watchdog timeout
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WatchdogTimeout {
    Timeout1s = 0,
    Timeout2s = 1,
    Timeout4s = 2,
    Timeout8s = 3,
    Timeout16s = 4,
    Timeout32s = 5,
    Timeout64s = 6,
    Timeout128s = 7,
}

/// VSYS DPM voltage levels
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum VsysDpmVoltage {
    V4_1 = 0,
    V4_2 = 1,
    V4_3 = 2,
    V4_4 = 3,
    V4_5 = 4,
    V4_6 = 5,
    V4_7 = 6,
    V4_8 = 7,
}

/// Power-on sources
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerOnSource {
    PwrOnLow = 0,
    IrqLow = 1,
    VbusInsert = 2,
    BatCharge = 3,
    BatInsert = 4,
    EnMode = 5,
    Unknown = 6,
}

/// Power-off sources
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerOffSource {
    PwrKeyPullDown = 0,
    SoftOff = 1,
    PwrKeyLow = 2,
    UnderVsys = 3,
    OverVbus = 4,
    UnderVoltage = 5,
    OverVoltage = 6,
    OverTemp = 7,
    Unknown = 8,
}

/// PWROK delay options
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PwrOkDelay {
    Delay8ms = 0,
    Delay16ms = 1,
    Delay32ms = 2,
    Delay64ms = 3,
}

/// Charge LED modes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChargeLedMode {
    Off = 0,
    Blink1Hz = 1,
    Blink4Hz = 2,
    On = 3,
    ControlledByCharger = 4,
}

/// DVM ramp speed
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DvmRamp {
    Ramp15_625us = 0,
    Ramp31_250us = 1,
}

/// Interrupt flags for AXP2101
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterruptFlags {
    pub status1: u8,
    pub status2: u8,
    pub status3: u8,
}

impl InterruptFlags {
    pub fn new() -> Self {
        Self {
            status1: 0,
            status2: 0,
            status3: 0,
        }
    }

    pub fn as_u32(&self) -> u32 {
        ((self.status1 as u32) << 16) | ((self.status2 as u32) << 8) | (self.status3 as u32)
    }

    pub fn from_u32(val: u32) -> Self {
        Self {
            status1: ((val >> 16) & 0xFF) as u8,
            status2: ((val >> 8) & 0xFF) as u8,
            status3: (val & 0xFF) as u8,
        }
    }
}

impl Default for InterruptFlags {
    fn default() -> Self {
        Self::new()
    }
}

// Interrupt bit masks (STATUS1 / IRQ1)
pub const IRQ_WARNING_LEVEL2: u32 = 1 << 23;
pub const IRQ_WARNING_LEVEL1: u32 = 1 << 22;
pub const IRQ_WDT_TIMEOUT: u32 = 1 << 21;
pub const IRQ_GAUGE_NEW_SOC: u32 = 1 << 20;
pub const IRQ_BAT_CHG_OVER_TEMP: u32 = 1 << 19;
pub const IRQ_BAT_CHG_UNDER_TEMP: u32 = 1 << 18;
pub const IRQ_BAT_WORK_OVER_TEMP: u32 = 1 << 17;
pub const IRQ_BAT_WORK_UNDER_TEMP: u32 = 1 << 16;

// Interrupt bit masks (STATUS2 / IRQ2)
pub const IRQ_VBUS_INSERT: u32 = 1 << 15;
pub const IRQ_VBUS_REMOVE: u32 = 1 << 14;
pub const IRQ_BAT_INSERT: u32 = 1 << 13;
pub const IRQ_BAT_REMOVE: u32 = 1 << 12;
pub const IRQ_PKEY_SHORT: u32 = 1 << 11;
pub const IRQ_PKEY_LONG: u32 = 1 << 10;
pub const IRQ_PKEY_NEGATIVE: u32 = 1 << 9;
pub const IRQ_PKEY_POSITIVE: u32 = 1 << 8;

// Interrupt bit masks (STATUS3 / IRQ3)
pub const IRQ_WDT_EXPIRE: u32 = 1 << 7;
pub const IRQ_LDO_OVER_CURR: u32 = 1 << 6;
pub const IRQ_BATFET_OVER_CURR: u32 = 1 << 5;
pub const IRQ_BAT_CHG_DONE: u32 = 1 << 4;
pub const IRQ_BAT_CHG_START: u32 = 1 << 3;
pub const IRQ_DIE_OVER_TEMP: u32 = 1 << 2;
pub const IRQ_CHARGER_TIMER: u32 = 1 << 1;
pub const IRQ_BAT_OVER_VOL: u32 = 1 << 0;
