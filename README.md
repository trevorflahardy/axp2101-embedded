# AXP2101 Embedded Rust Driver

A complete `no_std` Rust driver for the AXP2101 Power Management IC (PMIC) using the `embedded-hal` traits.

[![Crates.io](https://img.shields.io/crates/v/axp2101-embedded.svg)](https://crates.io/crates/axp2101-embedded)
[![Documentation](https://docs.rs/axp2101-embedded/badge.svg)](https://docs.rs/axp2101-embedded)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Features

- ✅ **Full Power Rail Control**: DCDC1-5, ALDO1-4, BLDO1-2, DLDO1-2, CPUSLDO
- ✅ **Battery Charging**: Configurable charging parameters, LED control, thermal regulation
- ✅ **ADC Measurements**: Battery voltage/percentage, VBUS, temperature, system voltage
- ✅ **Interrupt Handling**: Complete IRQ support with enable/disable/status
- ✅ **Watchdog Timer**: Configurable timeout and behavior
- ✅ **Power Sequencing**: Fast power-on, wake-up control, PWROK management
- ✅ **Status Monitoring**: VBUS, battery connection, charging state
- ✅ **Button Battery**: Support for coin cell charging
- ✅ **`no_std` Compatible**: Perfect for embedded systems
- ✅ **Embedded HAL 1.0**: Uses modern traits
- ✅ **Async Support**: Optional async API with `embedded-hal-async`

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
axp2101-embedded = "0.3"
embedded-hal = "1.0"
```

### Async Support

For async operations, enable the `async` feature:

```toml
[dependencies]
axp2101-embedded = { version = "0.3", features = ["async"] }
embedded-hal-async = "1.0"
```

### Basic Example

```rust
use axp2101_embedded::{Axp2101, PowerChannel};
use embedded_hal::i2c::I2c;

fn main() -> Result<(), axp2101_embedded::Error<impl embedded_hal::i2c::Error>> {
    // Get your I2C peripheral (platform-specific)
    let i2c = /* your I2C peripheral */;

    // Create the driver
    let mut pmic = Axp2101::new(i2c);

    // Initialize and verify chip
    pmic.init()?;

    // Enable DC1 at 3.3V
    pmic.enable_dc1()?;
    pmic.set_dc1_voltage(3300)?;

    // Enable ALDO1 at 1.8V
    pmic.enable_aldo1()?;
    pmic.set_aldo1_voltage(1800)?;

    // Read battery status
    if pmic.is_battery_connected()? {
        let voltage = pmic.get_battery_voltage()?;
        let percent = pmic.get_battery_percent()?;
        println!("Battery: {}mV ({}%)", voltage, percent);
    }

    // Check if charging
    if pmic.is_charging()? {
        println!("Battery is charging");
    }

    Ok(())
}
```

### Async Example

When the `async` feature is enabled, you can use the async API:

```rust
use axp2101_embedded::AsyncAxp2101;
use embedded_hal_async::i2c::I2c;

async fn example() -> Result<(), axp2101_embedded::Error<impl embedded_hal_async::i2c::Error>> {
    // Get your async I2C peripheral (platform-specific)
    let i2c = /* your async I2C peripheral */;

    // Create the async driver
    let mut pmic = AsyncAxp2101::new(i2c);

    // Initialize and verify chip
    pmic.init().await?;

    // Enable DC1 at 3.3V
    pmic.enable_dc1().await?;
    pmic.set_dc1_voltage(3300).await?;

    // Read battery status
    if pmic.is_battery_connected().await? {
        let voltage = pmic.get_battery_voltage().await?;
        let percent = pmic.get_battery_percent().await?;
        println!("Battery: {}mV ({}%)", voltage, percent);
    }

    // Check if charging
    if pmic.is_charging().await? {
        println!("Battery is charging");
    }

    Ok(())
}
```

The async API provides the same functionality as the synchronous version, but all methods return futures that must be awaited. This is perfect for async runtimes like `embassy` or `tokio` (for std environments).

### Configure Battery Charging

```rust
use axp2101_embedded::{Axp2101, ThermalThreshold, ChargeLedMode};

// Set charging parameters
pmic.set_charger_constant_curr(0x08)?;  // 200mA
pmic.set_charge_target_voltage(0x03)?;  // 4.2V
pmic.set_precharge_curr(PrechargeCurrent::I50mA)?;
pmic.set_thermal_threshold(ThermalThreshold::Temp100C)?;

// Control charging LED
pmic.set_charging_led_mode(ChargeLedMode::Blink1Hz)?;
```

### Handle Interrupts

```rust
use axp2101_embedded::*;

// Enable specific interrupts
pmic.enable_irq(IRQ_VBUS_INSERT | IRQ_BAT_CHG_DONE)?;

// In your interrupt handler:
let status = pmic.get_irq_status()?;

if status.is_vbus_insert_irq() {
    println!("VBUS inserted!");
}

if status.is_bat_charge_done_irq() {
    println!("Battery charging complete!");
}

// Clear interrupts
pmic.clear_irq_status()?;
```

## Power Rails

### DCDC Converters

| Rail | Voltage Range | Step Size | Notes |
|------|---------------|-----------|-------|
| DCDC1 | 1500-3400mV | 100mV | High efficiency |
| DCDC2 | 500-1540mV | 10/20mV | Dual range |
| DCDC3 | 500-3400mV | 10/20/100mV | Three ranges |
| DCDC4 | 500-1840mV | 10/20mV | Dual range |
| DCDC5 | 1400-3700mV | 100mV | GPIO output |

### LDO Regulators

| Rail | Voltage Range | Step Size | Typical Use |
|------|---------------|-----------|-------------|
| ALDO1-4 | 500-3500mV | 100mV | Always-on LDO |
| BLDO1-2 | 500-3500mV | 100mV | Battery LDO |
| CPUSLDO | 500-1400mV | 50mV | CPU supply |
| DLDO1 | 500-3400mV | 100mV | Digital LDO |
| DLDO2 | 500-1400mV | 50mV | Digital LDO |

## Documentation

Full API documentation is available at [docs.rs/axp2101](https://docs.rs/axp2101).

## Compatibility

- **Rust**: 2021 edition or later
- **embedded-hal**: 1.0
- **Platforms**: Any platform with I2C support via embedded-hal

Tested on:
- ESP32-C3/S3 (M5Stack CoreS3)
- STM32F4
- RP2040

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Resources
- [AXP2101 Datasheet](https://github.com/m5stack/M5-Schematic/blob/master/Core/AXP2101%20Datasheet_v1.1_en_draft_2211.pdf)
- [X-Powers AXP2101 Documentation](https://www.x-powers.com/en.php/Info/download/id/34.html)
- [Original C++ Implementation](https://github.com/lewisxhe/XPowersLib)

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Author

Trevor Flahardy

Based on the original C++ implementation by Lewis He and initial Rust port by Juraj Michálek.

