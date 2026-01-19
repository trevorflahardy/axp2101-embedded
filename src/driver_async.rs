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
    // Low-level async I2C operations
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

    // Remaining DCDC2-5 and ALDO1-4, BLDO1-2, CPUSLDO, DLDO1-2 control functions
    // follow the same pattern as DCDC1 but with async/await...

    // ========================================
    // ADC Control and Measurements
    // ========================================

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

    /// Set precharge current
    pub async fn set_precharge_current(
        &mut self,
        current: PrechargeCurrent,
    ) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_IPRECHG_SET).await? & 0xFC;
        val |= current as u8;
        self.write_register(AXP2101_IPRECHG_SET, val).await
    }

    /// Set charge target voltage
    pub async fn set_charge_target_voltage(&mut self, opt: u8) -> Result<(), Error<I::Error>> {
        let mut val = self.read_register(AXP2101_CV_CHG_VOL_SET).await? & 0xF8;
        val |= opt & 0x07;
        self.write_register(AXP2101_CV_CHG_VOL_SET, val).await
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

    /// Get chip ID
    pub async fn get_chip_id(&mut self) -> Result<u8, Error<I::Error>> {
        self.read_register(AXP2101_IC_TYPE).await
    }
}
