#![no_std]
//! # AXP2101 Power Management IC Driver
//!
//! This crate provides a complete embedded driver for the AXP2101 Power Management IC (PMIC).
//! It supports all features including:
//! - Status monitoring (VBUS, battery, charging state)
//! - DCDC1-5 voltage regulators
//! - ALDO1-4, BLDO1-2, DLDO1-2, CPUSLDO voltage regulators
//! - Battery charging control
//! - ADC measurements
//! - Interrupt handling
//! - Watchdog timer
//! - Power sequencing
//!
//! ## Example
//!
//! ```no_run
//! use axp2101::{Axp2101, Error};
//! # use embedded_hal::i2c::I2c;
//! # fn example<I: I2c>(i2c: I) -> Result<(), Error<I::Error>> {
//! let mut pmic = Axp2101::new(i2c);
//!
//! // Initialize and verify chip
//! pmic.init()?;
//!
//! // Enable DC1 at 3.3V
//! pmic.enable_dc1()?;
//! pmic.set_dc1_voltage(3300)?;
//!
//! // Check battery status
//! if pmic.is_battery_connected()? {
//!     let voltage = pmic.get_battery_voltage()?;
//!     // ... use battery voltage
//! }
//! # Ok(())
//! # }
//! ```
//!
//! ## Async Support
//!
//! When the `async` feature is enabled, the crate provides `AsyncAxp2101`
//! with the same API but async/await support:
//!
//! ```no_run
//! # #[cfg(feature = "async")]
//! # async fn example<I: embedded_hal_async::i2c::I2c>(i2c: I) -> Result<(), axp2101::Error<I::Error>> {
//! use axp2101::AsyncAxp2101;
//!
//! let mut pmic = AsyncAxp2101::new(i2c);
//! pmic.init().await?;
//! pmic.enable_dc1().await?;
//! # Ok(())
//! # }
//! ```

mod driver;
#[cfg(feature = "async")]
mod driver_async;
mod error;
mod registers;
mod types;

// Re-export main types
pub use driver::Axp2101;
#[cfg(feature = "async")]
pub use driver_async::AsyncAxp2101;
pub use error::Error;
pub use registers::*;
pub use types::*;
