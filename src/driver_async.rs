//! Async AXP2101 PMIC driver implementation

use crate::{error::Error, registers::*, temp_conversion, types::*, AXP2101_SLAVE_ADDRESS};

#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c as AsyncI2c;

/// Async AXP2101 PMIC driver
///
/// This implementation provides async operations for all AXP2101 functionality
/// when the `async` feature is enabled. All methods mirror the synchronous API
/// but return futures that can be awaited.
///
/// # Example
/// ```no_run
/// # #[cfg(feature = "async")]
/// # async fn example<I: embedded_hal_async::i2c::I2c>(i2c: I) -> Result<(), axp2101::Error<I::Error>> {
/// use axp2101::AsyncAxp2101;
///
/// let mut pmic = AsyncAxp2101::new(i2c);
///
/// // Initialize and verify chip
/// pmic.init().await?;
///
/// // Enable DC1 at 3.3V
/// pmic.enable_dc1().await?;
/// pmic.set_dc1_voltage(3300).await?;
///
/// // Check battery status
/// if pmic.is_battery_connected().await? {
///     let voltage = pmic.get_battery_voltage().await?;
///     // ... use battery voltage
/// }
/// # Ok(())
/// # }
/// ```
#[cfg(feature = "async")]
pub struct AsyncAxp2101<I> {
    i2c: I,
    addr: u8,
    int_register: [u8; 3],
}

#[cfg(feature = "async")]
impl<I> AsyncAxp2101<I>
where
    I: AsyncI2c,
{
    /// Create a new async AXP2101 driver instance
    ///
    /// # Arguments
    /// * `i2c` - Async I2C bus instance
    ///
    /// # Example
    /// ```no_run
    /// # #[cfg(feature = "async")]
    /// # async fn example<I: embedded_hal_async::i2c::I2c>(i2c: I) {
    /// use axp2101::AsyncAxp2101;
    /// let pmic = AsyncAxp2101::new(i2c);
    /// # }
    /// ```
    pub fn new(i2c: I) -> Self {
        Self::with_address(i2c, AXP2101_SLAVE_ADDRESS)
    }

    /// Create a new async AXP2101 driver instance with custom I2C address
    pub fn with_address(i2c: I, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            int_register: [0; 3],
        }
    }

    /// Initialize the AXP2101 and verify chip ID
    ///
    /// Returns `Error::DeviceNotFound` if the chip ID doesn't match
    pub async fn init(&mut self) -> Result<(), Error<I::Error>> {
        let chip_id = self.read_register(AXP2101_IC_TYPE).await?;
        if chip_id != AXP2101_CHIP_ID {
            return Err(Error::DeviceNotFound);
        }
        // Disable NTC temperature detection by default
        self.disable_ts_pin_measure().await?;
        Ok(())
    }

    // ========================================
    // Low-level I2C operations
    // ========================================

    /// Read a single register
    async fn read_register(&mut self, reg: u8) -> Result<u8, Error<I::Error>> {
        let mut buf = [0u8];
        self.i2c
            .write_read(self.addr, &[reg], &mut buf)
            .await
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    /// Write a single register
    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I::Error>> {
        self.i2c
            .write(self.addr, &[reg, value])
            .await
            .map_err(Error::I2c)
    }

    /// Read multiple registers
    async fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I::Error>> {
        self.i2c
            .write_read(self.addr, &[reg], buf)
            .await
            .map_err(Error::I2c)
    }

    /// Write multiple registers
    async fn write_registers(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<I::Error>> {
        let mut buf = [0u8; 33]; // max: 1 reg + 32 data bytes
        if data.len() > 32 {
            return Err(Error::InvalidParameter);
        }
        buf[0] = reg;
        buf[1..=data.len()].copy_from_slice(data);
        self.i2c
            .write(self.addr, &buf[..=data.len()])
            .await
            .map_err(Error::I2c)
    }

    /// Set a bit in a register
    async fn set_register_bit(&mut self, reg: u8, bit: u8) -> Result<(), Error<I::Error>> {
        let val = self.read_register(reg).await?;
        self.write_register(reg, val | (1 << bit)).await
    }

    /// Clear a bit in a register
    async fn clear_register_bit(&mut self, reg: u8, bit: u8) -> Result<(), Error<I::Error>> {
        let val = self.read_register(reg).await?;
        self.write_register(reg, val & !(1 << bit)).await
    }

    /// Get a bit from a register
    async fn get_register_bit(&mut self, reg: u8, bit: u8) -> Result<bool, Error<I::Error>> {
        let val = self.read_register(reg).await?;
        Ok((val & (1 << bit)) != 0)
    }

    /// Read H5L8 format (high 5 bits + low 8 bits = 13 bits)
    async fn read_register_h5l8(
        &mut self,
        high_reg: u8,
        low_reg: u8,
    ) -> Result<u16, Error<I::Error>> {
        let high = self.read_register(high_reg).await? as u16;
        let low = self.read_register(low_reg).await? as u16;
        Ok(((high & 0x1F) << 8) | low)
    }

    /// Read H6L8 format (high 6 bits + low 8 bits = 14 bits)
    async fn read_register_h6l8(
        &mut self,
        high_reg: u8,
        low_reg: u8,
    ) -> Result<u16, Error<I::Error>> {
        let high = self.read_register(high_reg).await? as u16;
        let low = self.read_register(low_reg).await? as u16;
        Ok(((high & 0x3F) << 8) | low)
    }

    // ========================================
    // Status functions
    // ========================================

    /// Get combined status (STATUS1 << 8 | STATUS2)
    pub async fn status(&mut self) -> Result<u16, Error<I::Error>> {
        let status1 = self.read_register(AXP2101_STATUS1).await? & 0x1F;
        let status2 = self.read_register(AXP2101_STATUS2).await? & 0x1F;
        Ok(((status1 as u16) << 8) | (status2 as u16))
    }

    /// Check if VBUS voltage is good
    pub async fn is_vbus_good(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_STATUS1, 5).await
    }

    /// Check BATFET state
    pub async fn get_batfet_state(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_STATUS1, 4).await
    }

    /// Check if battery is connected
    pub async fn is_battery_connected(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_STATUS1, 3).await
    }

    /// Check if battery is in active mode
    pub async fn is_battery_active(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_STATUS1, 2).await
    }

    /// Get thermal regulation status
    pub async fn get_thermal_regulation_status(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_STATUS1, 1).await
    }

    /// Get current limit status
    pub async fn get_current_limit_status(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_STATUS1, 0).await
    }

    /// Check if battery is charging
    pub async fn is_charging(&mut self) -> Result<bool, Error<I::Error>> {
        let val = self.read_register(AXP2101_STATUS2).await?;
        Ok((val >> 5) == 0x01)
    }

    /// Check if battery is discharging
    pub async fn is_discharging(&mut self) -> Result<bool, Error<I::Error>> {
        let val = self.read_register(AXP2101_STATUS2).await?;
        Ok((val >> 5) == 0x02)
    }

    /// Check if in standby mode
    pub async fn is_standby(&mut self) -> Result<bool, Error<I::Error>> {
        let val = self.read_register(AXP2101_STATUS2).await?;
        Ok((val >> 5) == 0x00)
    }

    /// Check if powered on
    pub async fn is_power_on(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_STATUS2, 4).await
    }

    /// Check if VBUS is inserted and good
    pub async fn is_vbus_in(&mut self) -> Result<bool, Error<I::Error>> {
        let bit3 = self.get_register_bit(AXP2101_STATUS2, 3).await?;
        let vbus_good = self.is_vbus_good().await?;
        Ok(!bit3 && vbus_good)
    }

    /// Get charger status
    pub async fn get_charger_status(&mut self) -> Result<ChargeStatus, Error<I::Error>> {
        let val = self.read_register(AXP2101_STATUS2).await? & 0x07;
        Ok(match val {
            0 => ChargeStatus::TriCharge,
            1 => ChargeStatus::PreCharge,
            2 => ChargeStatus::ConstantCurrent,
            3 => ChargeStatus::ConstantVoltage,
            4 => ChargeStatus::ChargeDone,
            _ => ChargeStatus::NotCharging,
        })
    }

    // ========================================
    // Data Buffer
    // ========================================

    /// Write data to buffer (max 6 bytes)
    pub async fn write_data_buffer(&mut self, data: &[u8]) -> Result<(), Error<I::Error>> {
        if data.len() > AXP2101_DATA_BUFFER_SIZE {
            return Err(Error::InvalidParameter);
        }
        self.write_registers(AXP2101_DATA_BUFFER1, data).await
    }

    /// Read data from buffer (max 6 bytes)
    pub async fn read_data_buffer(&mut self, buf: &mut [u8]) -> Result<(), Error<I::Error>> {
        if buf.len() > AXP2101_DATA_BUFFER_SIZE {
            return Err(Error::InvalidParameter);
        }
        self.read_registers(AXP2101_DATA_BUFFER1, buf).await
    }

    // ========================================
    // Common Configuration
    // ========================================

    /// Enable internal off-discharge for DCDC & LDO & SWITCH
    pub async fn enable_internal_discharge(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_COMMON_CONFIG, 5).await
    }

    /// Disable internal off-discharge
    pub async fn disable_internal_discharge(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_COMMON_CONFIG, 5).await
    }

    /// Enable PWROK pin pull low to restart
    pub async fn enable_pwrok_pin_pull_low(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_COMMON_CONFIG, 3).await
    }

    /// Disable PWROK pin pull low
    pub async fn disable_pwrok_pin_pull_low(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_COMMON_CONFIG, 3).await
    }

    /// Enable PWRON to shut down PMIC
    pub async fn enable_pwron_shutdown_pmic(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_COMMON_CONFIG, 2).await
    }

    /// Disable PWRON to shut down PMIC
    pub async fn disable_pwron_shutdown_pmic(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_COMMON_CONFIG, 2).await
    }

    /// Reset the SoC system and related registers
    pub async fn reset(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_COMMON_CONFIG, 1).await
    }

    /// Shutdown - turns off all power channels except VRTC
    pub async fn shutdown(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_COMMON_CONFIG, 0).await
    }

    /// Enable BATFET
    pub async fn enable_batfet(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_BATFET_CTRL, 3).await
    }

    /// Disable BATFET
    pub async fn disable_batfet(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_BATFET_CTRL, 3).await
    }

    /// Set discharge control
    pub async fn enable_discharge(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_BATFET_CTRL, 2).await
    }

    /// Disable discharge control
    pub async fn disable_discharge(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_BATFET_CTRL, 2).await
    }

    // ========================================
    // DCDC1 Control
    // ========================================

    /// Check if DC1 is enabled
    pub async fn is_dc1_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 0).await
    }

    /// Enable DC1 output
    pub async fn enable_dc1(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 0).await
    }

    /// Disable DC1 output
    pub async fn disable_dc1(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 0).await
    }

    /// Set DC1 voltage (1500-3400mV, 100mV steps)
    pub async fn set_dc1_voltage(&mut self, millivolt: u16) -> Result<(), Error<I::Error>> {
        if !millivolt.is_multiple_of(XPOWERS_AXP2101_DCDC1_VOL_STEPS) {
            return Err(Error::InvalidVoltage);
        }
        if !(XPOWERS_AXP2101_DCDC1_VOL_MIN..=XPOWERS_AXP2101_DCDC1_VOL_MAX).contains(&millivolt) {
            return Err(Error::InvalidVoltage);
        }
        let val = (millivolt - XPOWERS_AXP2101_DCDC1_VOL_MIN) / XPOWERS_AXP2101_DCDC1_VOL_STEPS;
        self.write_register(AXP2101_DC_VOL0_CTRL, val as u8).await
    }

    /// Get DC1 voltage
    pub async fn get_dc1_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_DC_VOL0_CTRL).await? & 0x1F;
        Ok((val as u16) * XPOWERS_AXP2101_DCDC1_VOL_STEPS + XPOWERS_AXP2101_DCDC1_VOL_MIN)
    }

    // ========================================
    // DCDC2 Control
    // ========================================

    /// Check if DC2 is enabled
    pub async fn is_dc2_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 1).await
    }

    /// Enable DC2 output
    pub async fn enable_dc2(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 1).await
    }

    /// Disable DC2 output
    pub async fn disable_dc2(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 1).await
    }

    /// Set DC2 voltage (500-1200mV @ 10mV steps, 1220-1540mV @ 20mV steps)
    pub async fn set_dc2_voltage(&mut self, millivolt: u16) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_DC_VOL1_CTRL).await? & 0x80;

        if (XPOWERS_AXP2101_DCDC2_VOL1_MIN..=XPOWERS_AXP2101_DCDC2_VOL1_MAX).contains(&millivolt) {
            if !millivolt.is_multiple_of(XPOWERS_AXP2101_DCDC2_VOL_STEPS1) {
                return Err(Error::InvalidVoltage);
            }
            val |= ((millivolt - XPOWERS_AXP2101_DCDC2_VOL1_MIN) / XPOWERS_AXP2101_DCDC2_VOL_STEPS1)
                as u8;
        } else if (XPOWERS_AXP2101_DCDC2_VOL2_MIN..=XPOWERS_AXP2101_DCDC2_VOL2_MAX)
            .contains(&millivolt)
        {
            if !millivolt.is_multiple_of(XPOWERS_AXP2101_DCDC2_VOL_STEPS2) {
                return Err(Error::InvalidVoltage);
            }
            val |= (((millivolt - XPOWERS_AXP2101_DCDC2_VOL2_MIN)
                / XPOWERS_AXP2101_DCDC2_VOL_STEPS2)
                + XPOWERS_AXP2101_DCDC2_VOL_STEPS2_BASE) as u8;
        } else {
            return Err(Error::InvalidVoltage);
        }

        self.write_register(AXP2101_DC_VOL1_CTRL, val).await
    }

    /// Get DC2 voltage
    pub async fn get_dc2_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_DC_VOL1_CTRL).await? & 0x7F;
        if val < XPOWERS_AXP2101_DCDC2_VOL_STEPS2_BASE as u8 {
            Ok((val as u16) * XPOWERS_AXP2101_DCDC2_VOL_STEPS1 + XPOWERS_AXP2101_DCDC2_VOL1_MIN)
        } else {
            Ok((val as u16) * XPOWERS_AXP2101_DCDC2_VOL_STEPS2 - 200)
        }
    }

    // ========================================
    // DCDC3 Control
    // ========================================

    /// Check if DC3 is enabled
    pub async fn is_dc3_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 2).await
    }

    /// Enable DC3 output
    pub async fn enable_dc3(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 2).await
    }

    /// Disable DC3 output
    pub async fn disable_dc3(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 2).await
    }

    /// Set DC3 voltage (500-1200mV @ 10mV, 1220-1540mV @ 20mV, 1600-3400mV @ 100mV)
    pub async fn set_dc3_voltage(&mut self, millivolt: u16) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_DC_VOL2_CTRL).await? & 0x80;

        if (XPOWERS_AXP2101_DCDC3_VOL1_MIN..=XPOWERS_AXP2101_DCDC3_VOL1_MAX).contains(&millivolt) {
            if !millivolt.is_multiple_of(XPOWERS_AXP2101_DCDC3_VOL_STEPS1) {
                return Err(Error::InvalidVoltage);
            }
            val |= ((millivolt - XPOWERS_AXP2101_DCDC3_VOL_MIN) / XPOWERS_AXP2101_DCDC3_VOL_STEPS1)
                as u8;
        } else if (XPOWERS_AXP2101_DCDC3_VOL2_MIN..=XPOWERS_AXP2101_DCDC3_VOL2_MAX)
            .contains(&millivolt)
        {
            if !millivolt.is_multiple_of(XPOWERS_AXP2101_DCDC3_VOL_STEPS2) {
                return Err(Error::InvalidVoltage);
            }
            val |= (((millivolt - XPOWERS_AXP2101_DCDC3_VOL2_MIN)
                / XPOWERS_AXP2101_DCDC3_VOL_STEPS2)
                + XPOWERS_AXP2101_DCDC3_VOL_STEPS2_BASE) as u8;
        } else if (XPOWERS_AXP2101_DCDC3_VOL3_MIN..=XPOWERS_AXP2101_DCDC3_VOL3_MAX)
            .contains(&millivolt)
        {
            if !millivolt.is_multiple_of(XPOWERS_AXP2101_DCDC3_VOL_STEPS3) {
                return Err(Error::InvalidVoltage);
            }
            val |= (((millivolt - XPOWERS_AXP2101_DCDC3_VOL3_MIN)
                / XPOWERS_AXP2101_DCDC3_VOL_STEPS3)
                + XPOWERS_AXP2101_DCDC3_VOL_STEPS3_BASE) as u8;
        } else {
            return Err(Error::InvalidVoltage);
        }

        self.write_register(AXP2101_DC_VOL2_CTRL, val).await
    }

    /// Get DC3 voltage
    pub async fn get_dc3_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_DC_VOL2_CTRL).await? & 0x7F;
        if val < XPOWERS_AXP2101_DCDC3_VOL_STEPS2_BASE as u8 {
            Ok((val as u16) * XPOWERS_AXP2101_DCDC3_VOL_STEPS1 + XPOWERS_AXP2101_DCDC3_VOL_MIN)
        } else if val < XPOWERS_AXP2101_DCDC3_VOL_STEPS3_BASE as u8 {
            Ok((val as u16) * XPOWERS_AXP2101_DCDC3_VOL_STEPS2 - 200)
        } else {
            Ok((val as u16) * XPOWERS_AXP2101_DCDC3_VOL_STEPS3 - 7200)
        }
    }

    // ========================================
    // DCDC4 Control
    // ========================================

    /// Check if DC4 is enabled
    pub async fn is_dc4_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 3).await
    }

    /// Enable DC4 output
    pub async fn enable_dc4(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 3).await
    }

    /// Disable DC4 output
    pub async fn disable_dc4(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 3).await
    }

    /// Set DC4 voltage (500-1200mV @ 10mV steps, 1220-1840mV @ 20mV steps)
    pub async fn set_dc4_voltage(&mut self, millivolt: u16) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_DC_VOL3_CTRL).await? & 0x80;

        if (XPOWERS_AXP2101_DCDC4_VOL1_MIN..=XPOWERS_AXP2101_DCDC4_VOL1_MAX).contains(&millivolt) {
            if !millivolt.is_multiple_of(XPOWERS_AXP2101_DCDC4_VOL_STEPS1) {
                return Err(Error::InvalidVoltage);
            }
            val |= ((millivolt - XPOWERS_AXP2101_DCDC4_VOL1_MIN) / XPOWERS_AXP2101_DCDC4_VOL_STEPS1)
                as u8;
        } else if (XPOWERS_AXP2101_DCDC4_VOL2_MIN..=XPOWERS_AXP2101_DCDC4_VOL2_MAX)
            .contains(&millivolt)
        {
            if !millivolt.is_multiple_of(XPOWERS_AXP2101_DCDC4_VOL_STEPS2) {
                return Err(Error::InvalidVoltage);
            }
            val |= (((millivolt - XPOWERS_AXP2101_DCDC4_VOL2_MIN)
                / XPOWERS_AXP2101_DCDC4_VOL_STEPS2)
                + XPOWERS_AXP2101_DCDC4_VOL_STEPS2_BASE) as u8;
        } else {
            return Err(Error::InvalidVoltage);
        }

        self.write_register(AXP2101_DC_VOL3_CTRL, val).await
    }

    /// Get DC4 voltage
    pub async fn get_dc4_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_DC_VOL3_CTRL).await? & 0x7F;
        if val < XPOWERS_AXP2101_DCDC4_VOL_STEPS2_BASE as u8 {
            Ok((val as u16) * XPOWERS_AXP2101_DCDC4_VOL_STEPS1 + XPOWERS_AXP2101_DCDC4_VOL1_MIN)
        } else {
            Ok((val as u16) * XPOWERS_AXP2101_DCDC4_VOL_STEPS2 - 200)
        }
    }

    // ========================================
    // DCDC5 Control
    // ========================================

    /// Check if DC5 is enabled
    pub async fn is_dc5_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 4).await
    }

    /// Enable DC5 output
    pub async fn enable_dc5(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 4).await
    }

    /// Disable DC5 output
    pub async fn disable_dc5(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 4).await
    }

    /// Set DC5 voltage (1400-3700mV @ 100mV steps, or 1200mV)
    pub async fn set_dc5_voltage(&mut self, millivolt: u16) -> Result<(), Error<I::Error>> {
        if !millivolt.is_multiple_of(XPOWERS_AXP2101_DCDC5_VOL_STEPS)
            && millivolt != XPOWERS_AXP2101_DCDC5_VOL_1200MV
        {
            return Err(Error::InvalidVoltage);
        }
        if millivolt != XPOWERS_AXP2101_DCDC5_VOL_1200MV
            && !(XPOWERS_AXP2101_DCDC5_VOL_MIN..=XPOWERS_AXP2101_DCDC5_VOL_MAX).contains(&millivolt)
        {
            return Err(Error::InvalidVoltage);
        }

        let mut val = self.read_register(AXP2101_DC_VOL4_CTRL).await? & 0xE0;
        if millivolt == XPOWERS_AXP2101_DCDC5_VOL_1200MV {
            val |= XPOWERS_AXP2101_DCDC5_VOL_VAL as u8;
        } else {
            val |= ((millivolt - XPOWERS_AXP2101_DCDC5_VOL_MIN) / XPOWERS_AXP2101_DCDC5_VOL_STEPS)
                as u8;
        }
        self.write_register(AXP2101_DC_VOL4_CTRL, val).await
    }

    /// Get DC5 voltage
    pub async fn get_dc5_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_DC_VOL4_CTRL).await? & 0x1F;
        if val == XPOWERS_AXP2101_DCDC5_VOL_VAL as u8 {
            Ok(XPOWERS_AXP2101_DCDC5_VOL_1200MV)
        } else {
            Ok((val as u16) * XPOWERS_AXP2101_DCDC5_VOL_STEPS + XPOWERS_AXP2101_DCDC5_VOL_MIN)
        }
    }

    // ========================================
    // ALDO1 Control
    // ========================================

    /// Check if ALDO1 is enabled
    pub async fn is_aldo1_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_LDO_ONOFF_CTRL0, 0).await
    }

    /// Enable ALDO1 output
    pub async fn enable_aldo1(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_LDO_ONOFF_CTRL0, 0).await
    }

    /// Disable ALDO1 output
    pub async fn disable_aldo1(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_LDO_ONOFF_CTRL0, 0).await
    }

    /// Set ALDO1 voltage (500-3500mV, 100mV steps)
    pub async fn set_aldo1_voltage(&mut self, millivolt: u16) -> Result<(), Error<I::Error>> {
        if !millivolt.is_multiple_of(XPOWERS_AXP2101_ALDO1_VOL_STEPS) {
            return Err(Error::InvalidVoltage);
        }
        if !(XPOWERS_AXP2101_ALDO1_VOL_MIN..=XPOWERS_AXP2101_ALDO1_VOL_MAX).contains(&millivolt) {
            return Err(Error::InvalidVoltage);
        }
        let mut val = self.read_register(AXP2101_LDO_VOL0_CTRL).await? & 0xE0;
        val |=
            ((millivolt - XPOWERS_AXP2101_ALDO1_VOL_MIN) / XPOWERS_AXP2101_ALDO1_VOL_STEPS) as u8;
        self.write_register(AXP2101_LDO_VOL0_CTRL, val).await
    }

    /// Get ALDO1 voltage
    pub async fn get_aldo1_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_LDO_VOL0_CTRL).await? & 0x1F;
        Ok((val as u16) * XPOWERS_AXP2101_ALDO1_VOL_STEPS + XPOWERS_AXP2101_ALDO1_VOL_MIN)
    }

    // ========================================
    // ALDO2 Control
    // ========================================

    /// Check if ALDO2 is enabled
    pub async fn is_aldo2_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_LDO_ONOFF_CTRL0, 1).await
    }

    /// Enable ALDO2 output
    pub async fn enable_aldo2(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_LDO_ONOFF_CTRL0, 1).await
    }

    /// Disable ALDO2 output
    pub async fn disable_aldo2(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_LDO_ONOFF_CTRL0, 1).await
    }

    /// Set ALDO2 voltage (500-3500mV, 100mV steps)
    pub async fn set_aldo2_voltage(&mut self, millivolt: u16) -> Result<(), Error<I::Error>> {
        if !millivolt.is_multiple_of(XPOWERS_AXP2101_ALDO2_VOL_STEPS) {
            return Err(Error::InvalidVoltage);
        }
        if !(XPOWERS_AXP2101_ALDO2_VOL_MIN..=XPOWERS_AXP2101_ALDO2_VOL_MAX).contains(&millivolt) {
            return Err(Error::InvalidVoltage);
        }
        let mut val = self.read_register(AXP2101_LDO_VOL1_CTRL).await? & 0xE0;
        val |=
            ((millivolt - XPOWERS_AXP2101_ALDO2_VOL_MIN) / XPOWERS_AXP2101_ALDO2_VOL_STEPS) as u8;
        self.write_register(AXP2101_LDO_VOL1_CTRL, val).await
    }

    /// Get ALDO2 voltage
    pub async fn get_aldo2_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_LDO_VOL1_CTRL).await? & 0x1F;
        Ok((val as u16) * XPOWERS_AXP2101_ALDO2_VOL_STEPS + XPOWERS_AXP2101_ALDO2_VOL_MIN)
    }

    // ========================================
    // ALDO3 Control
    // ========================================

    /// Check if ALDO3 is enabled
    pub async fn is_aldo3_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_LDO_ONOFF_CTRL0, 2).await
    }

    /// Enable ALDO3 output
    pub async fn enable_aldo3(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_LDO_ONOFF_CTRL0, 2).await
    }

    /// Disable ALDO3 output
    pub async fn disable_aldo3(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_LDO_ONOFF_CTRL0, 2).await
    }

    /// Set ALDO3 voltage (500-3500mV, 100mV steps)
    pub async fn set_aldo3_voltage(&mut self, millivolt: u16) -> Result<(), Error<I::Error>> {
        if !millivolt.is_multiple_of(XPOWERS_AXP2101_ALDO3_VOL_STEPS) {
            return Err(Error::InvalidVoltage);
        }
        if !(XPOWERS_AXP2101_ALDO3_VOL_MIN..=XPOWERS_AXP2101_ALDO3_VOL_MAX).contains(&millivolt) {
            return Err(Error::InvalidVoltage);
        }
        let mut val = self.read_register(AXP2101_LDO_VOL2_CTRL).await? & 0xE0;
        val |=
            ((millivolt - XPOWERS_AXP2101_ALDO3_VOL_MIN) / XPOWERS_AXP2101_ALDO3_VOL_STEPS) as u8;
        self.write_register(AXP2101_LDO_VOL2_CTRL, val).await
    }

    /// Get ALDO3 voltage
    pub async fn get_aldo3_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_LDO_VOL2_CTRL).await? & 0x1F;
        Ok((val as u16) * XPOWERS_AXP2101_ALDO3_VOL_STEPS + XPOWERS_AXP2101_ALDO3_VOL_MIN)
    }

    // ========================================
    // ALDO4 Control
    // ========================================

    /// Check if ALDO4 is enabled
    pub async fn is_aldo4_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_LDO_ONOFF_CTRL0, 3).await
    }

    /// Enable ALDO4 output
    pub async fn enable_aldo4(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_LDO_ONOFF_CTRL0, 3).await
    }

    /// Disable ALDO4 output
    pub async fn disable_aldo4(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_LDO_ONOFF_CTRL0, 3).await
    }

    /// Set ALDO4 voltage (500-3500mV, 100mV steps)
    pub async fn set_aldo4_voltage(&mut self, millivolt: u16) -> Result<(), Error<I::Error>> {
        if !millivolt.is_multiple_of(XPOWERS_AXP2101_ALDO4_VOL_STEPS) {
            return Err(Error::InvalidVoltage);
        }
        if !(XPOWERS_AXP2101_ALDO4_VOL_MIN..=XPOWERS_AXP2101_ALDO4_VOL_MAX).contains(&millivolt) {
            return Err(Error::InvalidVoltage);
        }
        let mut val = self.read_register(AXP2101_LDO_VOL3_CTRL).await? & 0xE0;
        val |=
            ((millivolt - XPOWERS_AXP2101_ALDO4_VOL_MIN) / XPOWERS_AXP2101_ALDO4_VOL_STEPS) as u8;
        self.write_register(AXP2101_LDO_VOL3_CTRL, val).await
    }

    /// Get ALDO4 voltage
    pub async fn get_aldo4_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_LDO_VOL3_CTRL).await? & 0x1F;
        Ok((val as u16) * XPOWERS_AXP2101_ALDO4_VOL_STEPS + XPOWERS_AXP2101_ALDO4_VOL_MIN)
    }

    // ========================================
    // BLDO1 Control
    // ========================================

    /// Check if BLDO1 is enabled
    pub async fn is_bldo1_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_LDO_ONOFF_CTRL0, 4).await
    }

    /// Enable BLDO1 output
    pub async fn enable_bldo1(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_LDO_ONOFF_CTRL0, 4).await
    }

    /// Disable BLDO1 output
    pub async fn disable_bldo1(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_LDO_ONOFF_CTRL0, 4).await
    }

    /// Set BLDO1 voltage (500-3500mV, 100mV steps)
    pub async fn set_bldo1_voltage(&mut self, millivolt: u16) -> Result<(), Error<I::Error>> {
        if !millivolt.is_multiple_of(XPOWERS_AXP2101_BLDO1_VOL_STEPS) {
            return Err(Error::InvalidVoltage);
        }
        if !(XPOWERS_AXP2101_BLDO1_VOL_MIN..=XPOWERS_AXP2101_BLDO1_VOL_MAX).contains(&millivolt) {
            return Err(Error::InvalidVoltage);
        }
        let mut val = self.read_register(AXP2101_LDO_VOL4_CTRL).await? & 0xE0;
        val |=
            ((millivolt - XPOWERS_AXP2101_BLDO1_VOL_MIN) / XPOWERS_AXP2101_BLDO1_VOL_STEPS) as u8;
        self.write_register(AXP2101_LDO_VOL4_CTRL, val).await
    }

    /// Get BLDO1 voltage
    pub async fn get_bldo1_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_LDO_VOL4_CTRL).await? & 0x1F;
        Ok((val as u16) * XPOWERS_AXP2101_BLDO1_VOL_STEPS + XPOWERS_AXP2101_BLDO1_VOL_MIN)
    }

    // ========================================
    // BLDO2 Control
    // ========================================

    /// Check if BLDO2 is enabled
    pub async fn is_bldo2_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_LDO_ONOFF_CTRL0, 5).await
    }

    /// Enable BLDO2 output
    pub async fn enable_bldo2(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_LDO_ONOFF_CTRL0, 5).await
    }

    /// Disable BLDO2 output
    pub async fn disable_bldo2(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_LDO_ONOFF_CTRL0, 5).await
    }

    /// Set BLDO2 voltage (500-3500mV, 100mV steps)
    pub async fn set_bldo2_voltage(&mut self, millivolt: u16) -> Result<(), Error<I::Error>> {
        if !millivolt.is_multiple_of(XPOWERS_AXP2101_BLDO2_VOL_STEPS) {
            return Err(Error::InvalidVoltage);
        }
        if !(XPOWERS_AXP2101_BLDO2_VOL_MIN..=XPOWERS_AXP2101_BLDO2_VOL_MAX).contains(&millivolt) {
            return Err(Error::InvalidVoltage);
        }
        let mut val = self.read_register(AXP2101_LDO_VOL5_CTRL).await? & 0xE0;
        val |=
            ((millivolt - XPOWERS_AXP2101_BLDO2_VOL_MIN) / XPOWERS_AXP2101_BLDO2_VOL_STEPS) as u8;
        self.write_register(AXP2101_LDO_VOL5_CTRL, val).await
    }

    /// Get BLDO2 voltage
    pub async fn get_bldo2_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_LDO_VOL5_CTRL).await? & 0x1F;
        Ok((val as u16) * XPOWERS_AXP2101_BLDO2_VOL_STEPS + XPOWERS_AXP2101_BLDO2_VOL_MIN)
    }

    // ========================================
    // CPUSLDO Control
    // ========================================

    /// Check if CPUSLDO is enabled
    pub async fn is_cpusldo_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_LDO_ONOFF_CTRL0, 6).await
    }

    /// Enable CPUSLDO output
    pub async fn enable_cpusldo(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_LDO_ONOFF_CTRL0, 6).await
    }

    /// Disable CPUSLDO output
    pub async fn disable_cpusldo(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_LDO_ONOFF_CTRL0, 6).await
    }

    /// Set CPUSLDO voltage (500-1400mV, 50mV steps)
    pub async fn set_cpusldo_voltage(&mut self, millivolt: u16) -> Result<(), Error<I::Error>> {
        if !millivolt.is_multiple_of(XPOWERS_AXP2101_CPUSLDO_VOL_STEPS) {
            return Err(Error::InvalidVoltage);
        }
        if !(XPOWERS_AXP2101_CPUSLDO_VOL_MIN..=XPOWERS_AXP2101_CPUSLDO_VOL_MAX).contains(&millivolt)
        {
            return Err(Error::InvalidVoltage);
        }
        let mut val = self.read_register(AXP2101_LDO_VOL6_CTRL).await? & 0xE0;
        val |= ((millivolt - XPOWERS_AXP2101_CPUSLDO_VOL_MIN) / XPOWERS_AXP2101_CPUSLDO_VOL_STEPS)
            as u8;
        self.write_register(AXP2101_LDO_VOL6_CTRL, val).await
    }

    /// Get CPUSLDO voltage
    pub async fn get_cpusldo_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_LDO_VOL6_CTRL).await? & 0x1F;
        Ok((val as u16) * XPOWERS_AXP2101_CPUSLDO_VOL_STEPS + XPOWERS_AXP2101_CPUSLDO_VOL_MIN)
    }

    // ========================================
    // DLDO1 Control
    // ========================================

    /// Check if DLDO1 is enabled
    pub async fn is_dldo1_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_LDO_ONOFF_CTRL0, 7).await
    }

    /// Enable DLDO1 output
    pub async fn enable_dldo1(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_LDO_ONOFF_CTRL0, 7).await
    }

    /// Disable DLDO1 output
    pub async fn disable_dldo1(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_LDO_ONOFF_CTRL0, 7).await
    }

    /// Set DLDO1 voltage (500-3400mV, 100mV steps)
    pub async fn set_dldo1_voltage(&mut self, millivolt: u16) -> Result<(), Error<I::Error>> {
        if !millivolt.is_multiple_of(XPOWERS_AXP2101_DLDO1_VOL_STEPS) {
            return Err(Error::InvalidVoltage);
        }
        if !(XPOWERS_AXP2101_DLDO1_VOL_MIN..=XPOWERS_AXP2101_DLDO1_VOL_MAX).contains(&millivolt) {
            return Err(Error::InvalidVoltage);
        }
        let mut val = self.read_register(AXP2101_LDO_VOL7_CTRL).await? & 0xE0;
        val |=
            ((millivolt - XPOWERS_AXP2101_DLDO1_VOL_MIN) / XPOWERS_AXP2101_DLDO1_VOL_STEPS) as u8;
        self.write_register(AXP2101_LDO_VOL7_CTRL, val).await
    }

    /// Get DLDO1 voltage
    pub async fn get_dldo1_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_LDO_VOL7_CTRL).await? & 0x1F;
        Ok((val as u16) * XPOWERS_AXP2101_DLDO1_VOL_STEPS + XPOWERS_AXP2101_DLDO1_VOL_MIN)
    }

    // ========================================
    // DLDO2 Control
    // ========================================

    /// Check if DLDO2 is enabled
    pub async fn is_dldo2_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_LDO_ONOFF_CTRL1, 0).await
    }

    /// Enable DLDO2 output
    pub async fn enable_dldo2(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_LDO_ONOFF_CTRL1, 0).await
    }

    /// Disable DLDO2 output
    pub async fn disable_dldo2(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_LDO_ONOFF_CTRL1, 0).await
    }

    /// Set DLDO2 voltage (500-1400mV, 50mV steps)
    pub async fn set_dldo2_voltage(&mut self, millivolt: u16) -> Result<(), Error<I::Error>> {
        if !millivolt.is_multiple_of(XPOWERS_AXP2101_DLDO2_VOL_STEPS) {
            return Err(Error::InvalidVoltage);
        }
        if !(XPOWERS_AXP2101_DLDO2_VOL_MIN..=XPOWERS_AXP2101_DLDO2_VOL_MAX).contains(&millivolt) {
            return Err(Error::InvalidVoltage);
        }
        let mut val = self.read_register(AXP2101_LDO_VOL8_CTRL).await? & 0xE0;
        val |=
            ((millivolt - XPOWERS_AXP2101_DLDO2_VOL_MIN) / XPOWERS_AXP2101_DLDO2_VOL_STEPS) as u8;
        self.write_register(AXP2101_LDO_VOL8_CTRL, val).await
    }

    /// Get DLDO2 voltage
    pub async fn get_dldo2_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_LDO_VOL8_CTRL).await? & 0x1F;
        Ok((val as u16) * XPOWERS_AXP2101_DLDO2_VOL_STEPS + XPOWERS_AXP2101_DLDO2_VOL_MIN)
    }

    // ========================================
    // Button Battery Charging
    // ========================================

    /// Enable button battery charging
    pub async fn enable_button_battery_charge(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_CHARGE_GAUGE_WDT_CTRL, 2)
            .await
    }

    /// Disable button battery charging
    pub async fn disable_button_battery_charge(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_CHARGE_GAUGE_WDT_CTRL, 2)
            .await
    }

    /// Check if button battery charging is enabled
    pub async fn is_button_battery_charge_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_CHARGE_GAUGE_WDT_CTRL, 2)
            .await
    }

    /// Set button battery charge voltage (2000-3500mV, 200mV steps)
    pub async fn set_button_battery_charge_voltage(
        &mut self,
        millivolt: u16,
    ) -> Result<(), Error<I::Error>> {
        if !millivolt.is_multiple_of(XPOWERS_AXP2101_BTN_VOL_STEPS) {
            return Err(Error::InvalidVoltage);
        }
        if !(XPOWERS_AXP2101_BTN_VOL_MIN..=XPOWERS_AXP2101_BTN_VOL_MAX).contains(&millivolt) {
            return Err(Error::InvalidVoltage);
        }
        let mut val = self.read_register(AXP2101_BTN_BAT_CHG_VOL_SET).await? & 0xF8;
        val |= ((millivolt - XPOWERS_AXP2101_BTN_VOL_MIN) / XPOWERS_AXP2101_BTN_VOL_STEPS) as u8;
        self.write_register(AXP2101_BTN_BAT_CHG_VOL_SET, val).await
    }

    /// Get button battery charge voltage
    pub async fn get_button_battery_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_BTN_BAT_CHG_VOL_SET).await? & 0x07;
        Ok((val as u16) * XPOWERS_AXP2101_BTN_VOL_STEPS + XPOWERS_AXP2101_BTN_VOL_MIN)
    }

    /// Enable cell battery charging
    pub async fn enable_cell_battery_charge(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_CHARGE_GAUGE_WDT_CTRL, 1)
            .await
    }

    /// Disable cell battery charging
    pub async fn disable_cell_battery_charge(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_CHARGE_GAUGE_WDT_CTRL, 1)
            .await
    }

    // ========================================
    // Watchdog
    // ========================================

    /// Enable watchdog and watchdog IRQ
    pub async fn enable_watchdog(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_CHARGE_GAUGE_WDT_CTRL, 0)
            .await?;
        self.enable_irq(IRQ_WDT_EXPIRE).await
    }

    /// Disable watchdog and watchdog IRQ
    pub async fn disable_watchdog(&mut self) -> Result<(), Error<I::Error>> {
        self.disable_irq(IRQ_WDT_EXPIRE).await?;
        self.clear_register_bit(AXP2101_CHARGE_GAUGE_WDT_CTRL, 0)
            .await
    }

    /// Set watchdog configuration
    pub async fn set_watchdog_config(
        &mut self,
        config: WatchdogConfig,
    ) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_WDT_CTRL).await? & 0xCF;
        val |= (config as u8) << 4;
        self.write_register(AXP2101_WDT_CTRL, val).await
    }

    /// Get watchdog configuration
    pub async fn get_watchdog_config(&mut self) -> Result<WatchdogConfig, Error<I::Error>> {
        let val = (self.read_register(AXP2101_WDT_CTRL).await? & 0x30) >> 4;
        Ok(match val {
            0 => WatchdogConfig::IrqOnly,
            1 => WatchdogConfig::IrqAndReset,
            2 => WatchdogConfig::IrqResetPullDownPwrOk,
            _ => WatchdogConfig::IrqResetAllOff,
        })
    }

    /// Clear watchdog timer
    pub async fn clear_watchdog(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_WDT_CTRL, 3).await
    }

    /// Set watchdog timeout
    pub async fn set_watchdog_timeout(
        &mut self,
        timeout: WatchdogTimeout,
    ) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_WDT_CTRL).await? & 0xF8;
        val |= timeout as u8;
        self.write_register(AXP2101_WDT_CTRL, val).await
    }

    /// Get watchdog timeout
    pub async fn get_watchdog_timeout(&mut self) -> Result<WatchdogTimeout, Error<I::Error>> {
        let val = self.read_register(AXP2101_WDT_CTRL).await? & 0x07;
        Ok(match val {
            0 => WatchdogTimeout::Timeout1s,
            1 => WatchdogTimeout::Timeout2s,
            2 => WatchdogTimeout::Timeout4s,
            3 => WatchdogTimeout::Timeout8s,
            4 => WatchdogTimeout::Timeout16s,
            5 => WatchdogTimeout::Timeout32s,
            6 => WatchdogTimeout::Timeout64s,
            _ => WatchdogTimeout::Timeout128s,
        })
    }

    // ========================================
    // Low Battery Warnings
    // ========================================

    /// Set low battery warning threshold (5-20%, 1% per step)
    pub async fn set_low_battery_warn_threshold(
        &mut self,
        percentage: u8,
    ) -> Result<(), Error<I::Error>> {
        if !(5..=20).contains(&percentage) {
            return Err(Error::InvalidParameter);
        }
        let mut val = self.read_register(AXP2101_LOW_BAT_WARN_SET).await? & 0x0F;
        val |= (percentage - 5) << 4;
        self.write_register(AXP2101_LOW_BAT_WARN_SET, val).await
    }

    /// Get low battery warning threshold
    pub async fn get_low_battery_warn_threshold(&mut self) -> Result<u8, Error<I::Error>> {
        let val = (self.read_register(AXP2101_LOW_BAT_WARN_SET).await? & 0xF0) >> 4;
        Ok(val + 5)
    }

    /// Set low battery shutdown threshold (0-15%, 1% per step)
    pub async fn set_low_battery_shutdown_threshold(
        &mut self,
        percentage: u8,
    ) -> Result<(), Error<I::Error>> {
        let percentage = if percentage > 15 { 15 } else { percentage };
        let mut val = self.read_register(AXP2101_LOW_BAT_WARN_SET).await? & 0xF0;
        val |= percentage;
        self.write_register(AXP2101_LOW_BAT_WARN_SET, val).await
    }

    /// Get low battery shutdown threshold
    pub async fn get_low_battery_shutdown_threshold(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(self.read_register(AXP2101_LOW_BAT_WARN_SET).await? & 0x0F)
    }

    // ========================================
    // Power On/Off Source Detection
    // ========================================

    /// Check if power-on always high (EN mode)
    pub async fn is_poweron_always_high_source(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_PWRON_STATUS, 5).await
    }

    /// Check if battery insert is power-on source
    pub async fn is_battery_insert_on_source(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_PWRON_STATUS, 4).await
    }

    /// Check if battery normal (>3.3V) is power-on source
    pub async fn is_battery_normal_on_source(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_PWRON_STATUS, 3).await
    }

    /// Check if VBUS insert is power-on source
    pub async fn is_vbus_insert_on_source(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_PWRON_STATUS, 2).await
    }

    /// Check if IRQ pin low is power-on source
    pub async fn is_irq_low_on_source(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_PWRON_STATUS, 1).await
    }

    /// Check if PWRON low is power-on source
    pub async fn is_pwron_low_on_source(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_PWRON_STATUS, 0).await
    }

    /// Check if over temperature is power-off source
    pub async fn is_over_temperature_off_source(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_PWROFF_STATUS, 7).await
    }

    /// Check if DC over voltage is power-off source
    pub async fn is_dc_over_voltage_off_source(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_PWROFF_STATUS, 6).await
    }

    /// Check if DC under voltage is power-off source
    pub async fn is_dc_under_voltage_off_source(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_PWROFF_STATUS, 5).await
    }

    /// Check if VBUS over voltage is power-off source
    pub async fn is_vbus_over_voltage_off_source(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_PWROFF_STATUS, 4).await
    }

    /// Check if VSYS under voltage is power-off source
    pub async fn is_vsys_under_voltage_off_source(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_PWROFF_STATUS, 3).await
    }

    // ========================================
    // Power Sequencing
    // ========================================

    /// Enable PWROK output
    pub async fn enable_pwrok(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_PWROK_SEQU_CTRL, 4).await
    }

    /// Disable PWROK output
    pub async fn disable_pwrok(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_PWROK_SEQU_CTRL, 4).await
    }

    /// Enable power-off delay
    pub async fn enable_poweroff_delay(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_PWROK_SEQU_CTRL, 3).await
    }

    /// Disable power-off delay
    pub async fn disable_poweroff_delay(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_PWROK_SEQU_CTRL, 3).await
    }

    /// Enable power sequence (reverse of startup)
    pub async fn enable_power_sequence(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_PWROK_SEQU_CTRL, 2).await
    }

    /// Disable power sequence (all at same time)
    pub async fn disable_power_sequence(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_PWROK_SEQU_CTRL, 2).await
    }

    /// Set PWROK delay
    pub async fn set_pwrok_delay(&mut self, delay: PwrOkDelay) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_PWROK_SEQU_CTRL).await? & 0xFC;
        val |= delay as u8;
        self.write_register(AXP2101_PWROK_SEQU_CTRL, val).await
    }

    /// Get PWROK delay
    pub async fn get_pwrok_delay(&mut self) -> Result<PwrOkDelay, Error<I::Error>> {
        let val = self.read_register(AXP2101_PWROK_SEQU_CTRL).await? & 0x03;
        Ok(match val {
            0 => PwrOkDelay::Delay8ms,
            1 => PwrOkDelay::Delay16ms,
            2 => PwrOkDelay::Delay32ms,
            _ => PwrOkDelay::Delay64ms,
        })
    }

    /// Enable fast power-on
    pub async fn enable_fast_power_on(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_FAST_PWRON_CTRL, 7).await
    }

    /// Disable fast power-on
    pub async fn disable_fast_power_on(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_FAST_PWRON_CTRL, 7).await
    }

    /// Enable fast wakeup
    pub async fn enable_fast_wakeup(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_FAST_PWRON_CTRL, 6).await
    }

    /// Disable fast wakeup
    pub async fn disable_fast_wakeup(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_FAST_PWRON_CTRL, 6).await
    }

    // ========================================
    // VBUS Control
    // ========================================

    /// Set VBUS voltage limit
    pub async fn set_vbus_voltage_limit(&mut self, opt: u8) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_INPUT_VOL_LIMIT_CTRL).await? & 0xF0;
        val |= opt & 0x0F;
        self.write_register(AXP2101_INPUT_VOL_LIMIT_CTRL, val).await
    }

    /// Get VBUS voltage limit
    pub async fn get_vbus_voltage_limit(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(self.read_register(AXP2101_INPUT_VOL_LIMIT_CTRL).await? & 0x0F)
    }

    /// Set VBUS current limit
    pub async fn set_vbus_current_limit(&mut self, opt: u8) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_INPUT_CUR_LIMIT_CTRL).await? & 0xF8;
        val |= opt & 0x07;
        self.write_register(AXP2101_INPUT_CUR_LIMIT_CTRL, val).await
    }

    /// Get VBUS current limit
    pub async fn get_vbus_current_limit(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(self.read_register(AXP2101_INPUT_CUR_LIMIT_CTRL).await? & 0x07)
    }

    /// Set system power down voltage (2600-3300mV, 100mV steps)
    pub async fn set_sys_power_down_voltage(
        &mut self,
        millivolt: u16,
    ) -> Result<(), Error<I::Error>> {
        if !millivolt.is_multiple_of(XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_STEPS) {
            return Err(Error::InvalidVoltage);
        }
        if !(XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_MIN..=XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_MAX)
            .contains(&millivolt)
        {
            return Err(Error::InvalidVoltage);
        }
        let mut val = self.read_register(AXP2101_VOFF_SET).await? & 0xF8;
        val |= ((millivolt - XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_MIN)
            / XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_STEPS) as u8;
        self.write_register(AXP2101_VOFF_SET, val).await
    }

    /// Get system power down voltage
    pub async fn get_sys_power_down_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        let val = self.read_register(AXP2101_VOFF_SET).await? & 0x07;
        Ok((val as u16) * XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_STEPS
            + XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_MIN)
    }

    // ========================================
    // ADC Control and Measurements
    // ========================================

    /// Enable general ADC channel
    pub async fn enable_general_adc_channel(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_ADC_CHANNEL_CTRL, 5).await
    }

    /// Disable general ADC channel
    pub async fn disable_general_adc_channel(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_ADC_CHANNEL_CTRL, 5).await
    }

    /// Enable temperature measurement
    pub async fn enable_temperature_measure(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_ADC_CHANNEL_CTRL, 4).await
    }

    /// Disable temperature measurement
    pub async fn disable_temperature_measure(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_ADC_CHANNEL_CTRL, 4).await
    }

    /// Get die temperature in Celsius (14-bit ADC)
    pub async fn get_temperature(&mut self) -> Result<f32, Error<I::Error>> {
        let raw = self
            .read_register_h6l8(AXP2101_ADC_DATA_RELUST8, AXP2101_ADC_DATA_RELUST9)
            .await?;
        if raw > 16383 {
            return Err(Error::InvalidParameter);
        }
        Ok(temp_conversion(raw))
    }

    /// Enable system voltage measurement
    pub async fn enable_system_voltage_measure(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_ADC_CHANNEL_CTRL, 3).await
    }

    /// Disable system voltage measurement
    pub async fn disable_system_voltage_measure(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_ADC_CHANNEL_CTRL, 3).await
    }

    /// Get system voltage in mV
    pub async fn get_system_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        self.read_register_h6l8(AXP2101_ADC_DATA_RELUST6, AXP2101_ADC_DATA_RELUST7)
            .await
    }

    /// Enable VBUS voltage measurement
    pub async fn enable_vbus_voltage_measure(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_ADC_CHANNEL_CTRL, 2).await
    }

    /// Disable VBUS voltage measurement
    pub async fn disable_vbus_voltage_measure(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_ADC_CHANNEL_CTRL, 2).await
    }

    /// Get VBUS voltage in mV
    pub async fn get_vbus_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        if !self.is_vbus_in().await? {
            return Ok(0);
        }
        self.read_register_h6l8(AXP2101_ADC_DATA_RELUST4, AXP2101_ADC_DATA_RELUST5)
            .await
    }

    /// Enable TS pin measurement (affects charger)
    pub async fn enable_ts_pin_measure(&mut self) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_TS_PIN_CTRL).await? & 0xE0;
        val |= 0x07;
        self.write_register(AXP2101_TS_PIN_CTRL, val).await?;
        self.set_register_bit(AXP2101_ADC_CHANNEL_CTRL, 1).await
    }

    /// Disable TS pin measurement (fixed input, doesn't affect charger)
    pub async fn disable_ts_pin_measure(&mut self) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_TS_PIN_CTRL).await? & 0xF0;
        val |= 0x10;
        self.write_register(AXP2101_TS_PIN_CTRL, val).await?;
        self.clear_register_bit(AXP2101_ADC_CHANNEL_CTRL, 1).await
    }

    /// Get TS pin raw ADC value
    pub async fn get_ts_pin_value(&mut self) -> Result<u16, Error<I::Error>> {
        self.read_register_h6l8(AXP2101_ADC_DATA_RELUST2, AXP2101_ADC_DATA_RELUST3)
            .await
    }

    /// Enable battery voltage measurement
    pub async fn enable_battery_voltage_measure(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_ADC_CHANNEL_CTRL, 0).await
    }

    /// Disable battery voltage measurement
    pub async fn disable_battery_voltage_measure(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_ADC_CHANNEL_CTRL, 0).await
    }

    /// Enable battery detection
    pub async fn enable_battery_detection(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_BAT_DET_CTRL, 0).await
    }

    /// Disable battery detection
    pub async fn disable_battery_detection(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_BAT_DET_CTRL, 0).await
    }

    /// Get battery voltage in mV
    pub async fn get_battery_voltage(&mut self) -> Result<u16, Error<I::Error>> {
        if !self.is_battery_connected().await? {
            return Ok(0);
        }
        self.read_register_h5l8(AXP2101_ADC_DATA_RELUST0, AXP2101_ADC_DATA_RELUST1)
            .await
    }

    /// Get battery percentage (0-100, or -1 if not connected)
    pub async fn get_battery_percent(&mut self) -> Result<i16, Error<I::Error>> {
        if !self.is_battery_connected().await? {
            return Ok(-1);
        }
        Ok(self.read_register(AXP2101_BAT_PERCENT_DATA).await? as i16)
    }

    // ========================================
    // Charging Control
    // ========================================

    /// Set charging LED mode
    pub async fn set_charging_led_mode(
        &mut self,
        mode: ChargeLedMode,
    ) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_CHGLED_SET_CTRL).await?;
        match mode {
            ChargeLedMode::Off
            | ChargeLedMode::Blink1Hz
            | ChargeLedMode::Blink4Hz
            | ChargeLedMode::On => {
                val &= 0xC8;
                val |= 0x05; // manual control
                val |= (mode as u8) << 4;
            }
            ChargeLedMode::ControlledByCharger => {
                val &= 0xF9;
                val |= 0x01; // type A mode
            }
        }
        self.write_register(AXP2101_CHGLED_SET_CTRL, val).await
    }

    /// Set precharge current
    pub async fn set_precharge_current(
        &mut self,
        current: PrechargeCurrent,
    ) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_IPRECHG_SET).await? & 0xFC;
        val |= current as u8;
        self.write_register(AXP2101_IPRECHG_SET, val).await
    }

    /// Get precharge current
    pub async fn get_precharge_current(&mut self) -> Result<PrechargeCurrent, Error<I::Error>> {
        let val = self.read_register(AXP2101_IPRECHG_SET).await? & 0x03;
        Ok(match val {
            0 => PrechargeCurrent::I0mA,
            1 => PrechargeCurrent::I25mA,
            2 => PrechargeCurrent::I50mA,
            3 => PrechargeCurrent::I75mA,
            4 => PrechargeCurrent::I100mA,
            5 => PrechargeCurrent::I125mA,
            6 => PrechargeCurrent::I150mA,
            7 => PrechargeCurrent::I175mA,
            _ => PrechargeCurrent::I200mA,
        })
    }

    /// Set charger constant current
    pub async fn set_charger_constant_current(&mut self, opt: u8) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_ICC_CHG_SET).await? & 0xE0;
        val |= opt & 0x1F;
        self.write_register(AXP2101_ICC_CHG_SET, val).await
    }

    /// Get charger constant current
    pub async fn get_charger_constant_current(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(self.read_register(AXP2101_ICC_CHG_SET).await? & 0x1F)
    }

    /// Set charger termination current
    pub async fn set_charger_termination_current(
        &mut self,
        current: ChargeTerminationCurrent,
    ) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_ITERM_CHG_SET_CTRL).await? & 0xF0;
        val |= current as u8;
        self.write_register(AXP2101_ITERM_CHG_SET_CTRL, val).await
    }

    /// Get charger termination current
    pub async fn get_charger_termination_current(
        &mut self,
    ) -> Result<ChargeTerminationCurrent, Error<I::Error>> {
        let val = self.read_register(AXP2101_ITERM_CHG_SET_CTRL).await? & 0x0F;
        Ok(match val {
            0 => ChargeTerminationCurrent::I0mA,
            1 => ChargeTerminationCurrent::I25mA,
            2 => ChargeTerminationCurrent::I50mA,
            3 => ChargeTerminationCurrent::I75mA,
            4 => ChargeTerminationCurrent::I100mA,
            5 => ChargeTerminationCurrent::I125mA,
            6 => ChargeTerminationCurrent::I150mA,
            7 => ChargeTerminationCurrent::I175mA,
            _ => ChargeTerminationCurrent::I200mA,
        })
    }

    /// Enable charger termination limit
    pub async fn enable_charger_termination_limit(&mut self) -> Result<(), Error<I::Error>> {
        let val = self.read_register(AXP2101_ITERM_CHG_SET_CTRL).await?;
        self.write_register(AXP2101_ITERM_CHG_SET_CTRL, val | 0x10)
            .await
    }

    /// Disable charger termination limit
    pub async fn disable_charger_termination_limit(&mut self) -> Result<(), Error<I::Error>> {
        let val = self.read_register(AXP2101_ITERM_CHG_SET_CTRL).await?;
        self.write_register(AXP2101_ITERM_CHG_SET_CTRL, val & 0xEF)
            .await
    }

    /// Set charge target voltage
    pub async fn set_charge_target_voltage(&mut self, opt: u8) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_CV_CHG_VOL_SET).await? & 0xF8;
        val |= opt & 0x07;
        self.write_register(AXP2101_CV_CHG_VOL_SET, val).await
    }

    /// Get charge target voltage
    pub async fn get_charge_target_voltage(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(self.read_register(AXP2101_CV_CHG_VOL_SET).await? & 0x07)
    }

    /// Set thermal regulation threshold
    pub async fn set_thermal_threshold(
        &mut self,
        threshold: ThermalThreshold,
    ) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_THE_REGU_THRES_SET).await? & 0xFC;
        val |= threshold as u8;
        self.write_register(AXP2101_THE_REGU_THRES_SET, val).await
    }

    /// Get thermal regulation threshold
    pub async fn get_thermal_threshold(&mut self) -> Result<ThermalThreshold, Error<I::Error>> {
        let val = self.read_register(AXP2101_THE_REGU_THRES_SET).await? & 0x03;
        Ok(match val {
            0 => ThermalThreshold::Temp60C,
            1 => ThermalThreshold::Temp80C,
            2 => ThermalThreshold::Temp100C,
            _ => ThermalThreshold::Temp120C,
        })
    }

    // ========================================
    // Interrupt Control
    // ========================================

    /// Get interrupt status as 32-bit value
    pub async fn get_irq_status(&mut self) -> Result<u32, Error<I::Error>> {
        let mut status = [0u8; 3];
        status[0] = self.read_register(AXP2101_INTSTS1).await?;
        status[1] = self.read_register(AXP2101_INTSTS2).await?;
        status[2] = self.read_register(AXP2101_INTSTS3).await?;
        Ok(((status[0] as u32) << 16) | ((status[1] as u32) << 8) | (status[2] as u32))
    }

    /// Clear all interrupt status
    pub async fn clear_irq_status(&mut self) -> Result<(), Error<I::Error>> {
        self.write_register(AXP2101_INTSTS1, 0xFF).await?;
        self.write_register(AXP2101_INTSTS2, 0xFF).await?;
        self.write_register(AXP2101_INTSTS3, 0xFF).await
    }

    /// Enable specific IRQ
    pub async fn enable_irq(&mut self, irq_mask: u32) -> Result<(), Error<I::Error>> {
        self.set_interrupt_impl(irq_mask, true).await
    }

    /// Disable specific IRQ
    pub async fn disable_irq(&mut self, irq_mask: u32) -> Result<(), Error<I::Error>> {
        self.set_interrupt_impl(irq_mask, false).await
    }

    /// Internal interrupt implementation
    async fn set_interrupt_impl(&mut self, opts: u32, enable: bool) -> Result<(), Error<I::Error>> {
        if opts & 0x0000FF != 0 {
            let value = (opts & 0xFF) as u8;
            let data = self.read_register(AXP2101_INTEN1).await?;
            self.int_register[0] = if enable { data | value } else { data & !value };
            self.write_register(AXP2101_INTEN1, self.int_register[0])
                .await?;
        }
        if opts & 0x00FF00 != 0 {
            let value = ((opts >> 8) & 0xFF) as u8;
            let data = self.read_register(AXP2101_INTEN2).await?;
            self.int_register[1] = if enable { data | value } else { data & !value };
            self.write_register(AXP2101_INTEN2, self.int_register[1])
                .await?;
        }
        if opts & 0xFF0000 != 0 {
            let value = ((opts >> 16) & 0xFF) as u8;
            let data = self.read_register(AXP2101_INTEN3).await?;
            self.int_register[2] = if enable { data | value } else { data & !value };
            self.write_register(AXP2101_INTEN3, self.int_register[2])
                .await?;
        }
        Ok(())
    }

    // ========================================
    // Gauge Data Operations
    // ========================================

    /// Write gauge data (128 bytes)
    pub async fn write_gauge_data(&mut self, data: &[u8]) -> Result<bool, Error<I::Error>> {
        if data.len() != 128 {
            return Err(Error::InvalidParameter);
        }

        // Reset gauge first
        self.set_register_bit(AXP2101_RESET_FUEL_GAUGE, 2).await?;
        self.clear_register_bit(AXP2101_RESET_FUEL_GAUGE, 2).await?;

        // Enable ROM register
        self.clear_register_bit(AXP2101_FUEL_GAUGE_CTRL, 0).await?;
        self.set_register_bit(AXP2101_FUEL_GAUGE_CTRL, 0).await?;

        // Write data to buffer
        for byte in data {
            self.write_register(AXP2101_BAT_PARAMS, *byte).await?;
        }

        // Re-enable ROM register
        self.clear_register_bit(AXP2101_FUEL_GAUGE_CTRL, 0).await?;
        self.set_register_bit(AXP2101_FUEL_GAUGE_CTRL, 0).await?;

        self.compare_gauge_data(data).await
    }

    /// Compare gauge data (128 bytes)
    pub async fn compare_gauge_data(&mut self, data: &[u8]) -> Result<bool, Error<I::Error>> {
        if data.len() != 128 {
            return Err(Error::InvalidParameter);
        }

        let mut buffer = [0u8; 128];

        // Re-enable ROM register
        self.clear_register_bit(AXP2101_FUEL_GAUGE_CTRL, 0).await?;
        self.set_register_bit(AXP2101_FUEL_GAUGE_CTRL, 0).await?;

        // Read data
        for item in &mut buffer {
            *item = self.read_register(AXP2101_BAT_PARAMS).await?;
        }

        // Disable ROM register
        self.clear_register_bit(AXP2101_FUEL_GAUGE_CTRL, 0).await?;

        // Set data interface
        self.set_register_bit(AXP2101_FUEL_GAUGE_CTRL, 4).await?;

        // Reset gauge
        self.set_register_bit(AXP2101_RESET_FUEL_GAUGE, 2).await?;
        self.clear_register_bit(AXP2101_RESET_FUEL_GAUGE, 2).await?;

        Ok(data == &buffer[..])
    }

    // ========================================
    // DVM and PWM Control
    // ========================================

    /// Enable CCM mode
    pub async fn enable_ccm(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 6).await
    }

    /// Disable CCM mode
    pub async fn disable_ccm(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 6).await
    }

    /// Check if CCM is enabled
    pub async fn is_ccm_enabled(&mut self) -> Result<bool, Error<I::Error>> {
        self.get_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 6).await
    }

    /// Set DVM ramp speed
    pub async fn set_dvm_ramp(&mut self, ramp: DvmRamp) -> Result<(), Error<I::Error>> {
        if ramp as u8 == 0 {
            self.clear_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 5).await
        } else {
            self.set_register_bit(AXP2101_DC_ONOFF_DVM_CTRL, 5).await
        }
    }

    /// Set DCDC work mode to PWM
    pub async fn set_dc1_work_mode_pwm(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        if enable {
            self.set_register_bit(AXP2101_DC_FORCE_PWM_CTRL, 2).await
        } else {
            self.clear_register_bit(AXP2101_DC_FORCE_PWM_CTRL, 2).await
        }
    }

    /// Set DCDC2 work mode to PWM
    pub async fn set_dc2_work_mode_pwm(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        if enable {
            self.set_register_bit(AXP2101_DC_FORCE_PWM_CTRL, 3).await
        } else {
            self.clear_register_bit(AXP2101_DC_FORCE_PWM_CTRL, 3).await
        }
    }

    /// Set DCDC3 work mode to PWM
    pub async fn set_dc3_work_mode_pwm(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        if enable {
            self.set_register_bit(AXP2101_DC_FORCE_PWM_CTRL, 4).await
        } else {
            self.clear_register_bit(AXP2101_DC_FORCE_PWM_CTRL, 4).await
        }
    }

    /// Set DCDC4 work mode to PWM
    pub async fn set_dc4_work_mode_pwm(&mut self, enable: bool) -> Result<(), Error<I::Error>> {
        if enable {
            self.set_register_bit(AXP2101_DC_FORCE_PWM_CTRL, 5).await
        } else {
            self.clear_register_bit(AXP2101_DC_FORCE_PWM_CTRL, 5).await
        }
    }

    /// Enable wakeup
    pub async fn enable_wakeup(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_SLEEP_WAKEUP_CTRL, 1).await
    }

    /// Disable wakeup
    pub async fn disable_wakeup(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_SLEEP_WAKEUP_CTRL, 1).await
    }

    /// Enable sleep mode
    pub async fn enable_sleep(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_SLEEP_WAKEUP_CTRL, 0).await
    }

    /// Disable sleep mode
    pub async fn disable_sleep(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_SLEEP_WAKEUP_CTRL, 0).await
    }

    /// Enable long press shutdown
    pub async fn enable_long_press_shutdown(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_PWROFF_EN, 1).await
    }

    /// Disable long press shutdown
    pub async fn disable_long_press_shutdown(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_PWROFF_EN, 1).await
    }

    /// Set long press to restart
    pub async fn set_long_press_restart(&mut self) -> Result<(), Error<I::Error>> {
        self.set_register_bit(AXP2101_PWROFF_EN, 0).await
    }

    /// Set long press to power off
    pub async fn set_long_press_poweroff(&mut self) -> Result<(), Error<I::Error>> {
        self.clear_register_bit(AXP2101_PWROFF_EN, 0).await
    }

    /// Get chip ID
    pub async fn get_chip_id(&mut self) -> Result<u8, Error<I::Error>> {
        self.read_register(AXP2101_IC_TYPE).await
    }
}
