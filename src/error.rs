//! Error types for AXP2101 operations
//!
//! This module defines the error types that can occur when using the AXP2101 driver.

/// Error types for AXP2101 operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// I2C communication error
    I2c(E),
    /// Invalid parameter value
    InvalidParameter,
    /// Invalid voltage value
    InvalidVoltage,
    /// Device not found or wrong chip ID
    DeviceNotFound,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::I2c(error)
    }
}
