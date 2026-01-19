//! Basic AXP2101 usage example
//!
//! This example demonstrates initializing the AXP2101 and configuring power rails.
//!
//! Note: You'll need to adapt this to your specific platform's I2C implementation.

#![no_std]
#![no_main]

use axp2101::{Axp2101, Error};
use embedded_hal::i2c::I2c;

// Platform-specific panic handler and I2C setup would go here

fn configure_pmic<I: I2c>(i2c: I) -> Result<(), Error<I::Error>> {
    // Create the PMIC driver
    let mut pmic = Axp2101::new(i2c);

    // Initialize and verify the chip
    pmic.init()?;

    // Configure main 3.3V rail for system
    pmic.enable_dc1()?;
    pmic.set_dc1_voltage(3300)?;

    // Configure 1.8V rail for peripherals
    pmic.enable_dc3()?;
    pmic.set_dc3_voltage(1800)?;

    // Configure LDO for sensors
    pmic.enable_aldo1()?;
    pmic.set_aldo1_voltage(3000)?;

    // Enable ADC channels for monitoring
    pmic.enable_battery_voltage_measure()?;
    pmic.enable_vbus_voltage_measure()?;
    pmic.enable_temperature_measure()?;

    // Configure battery charging
    pmic.set_charger_constant_current(0x08)?; // 200mA charging current
    pmic.set_charge_target_voltage(0x03)?; // 4.2V target

    // Set up low battery warning at 15%
    pmic.set_low_battery_warn_threshold(15)?;

    // Enable interrupts for important events
    pmic.enable_irq(
        axp2101::IRQ_VBUS_INSERT | axp2101::IRQ_VBUS_REMOVE | axp2101::IRQ_BAT_CHG_DONE,
    )?;

    // Read initial status
    if pmic.is_battery_connected()? {
        let voltage = pmic.get_battery_voltage()?;
        let percent = pmic.get_battery_percent()?;
        // Platform-specific: print or log battery status
        // println!("Battery: {}mV ({}%)", voltage, percent);
    }

    if pmic.is_vbus_in()? {
        let vbus_voltage = pmic.get_vbus_voltage()?;
        // println!("VBUS: {}mV", vbus_voltage);
    }

    Ok(())
}

// Interrupt handler example (platform-specific)
// fn handle_pmic_interrupt<I: I2c>(pmic: &mut Axp2101<I>) -> Result<(), Error<I::Error>> {
//     let status = pmic.get_irq_status()?;
//
//     if status.is_vbus_insert_irq() {
//         // Handle VBUS insertion
//     }
//
//     if status.is_vbus_remove_irq() {
//         // Handle VBUS removal
//     }
//
//     if status.is_bat_charge_done_irq() {
//         // Handle battery fully charged
//     }
//
//     pmic.clear_irq_status()?;
//     Ok(())
// }
