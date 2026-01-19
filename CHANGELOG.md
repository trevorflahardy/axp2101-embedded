# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2026-01-19

### Added
- Complete implementation of all AXP2101 functionality based on C++ reference
- Full power rail control for all DCDC (1-5) and LDO (ALDO1-4, BLDO1-2, DLDO1-2, CPUSLDO) channels
- Comprehensive battery charging configuration
  - Precharge current control
  - Constant current charging
  - Charge termination current
  - Target voltage settings
  - Thermal regulation
  - LED control modes
- ADC measurement support
  - Battery voltage and percentage
  - VBUS voltage
  - System voltage
  - Die temperature with conversion
  - TS pin measurement
- Complete interrupt handling
  - Enable/disable individual interrupts
  - Read interrupt status with helper methods
  - Clear interrupt status
- Watchdog timer support
  - Configurable timeout
  - Multiple operation modes
  - Timer clearing
- Power sequencing and control
  - Fast power-on configuration
  - Fast wakeup control
  - PWROK management
  - Power on/off source detection
- Button battery charging support
- Low battery warning and shutdown thresholds
- Fuel gauge data programming
- Status monitoring functions
- Data buffer read/write
- Comprehensive documentation for all public APIs
- Example code demonstrating usage
- Detailed README with feature overview and examples

### Changed
- Replaced `DataFormat` pattern with direct `&[u8]` usage for cleaner API
- Updated to use embedded-hal 1.0 I2c trait
- Improved error handling with specific error types
- Refactored into clean modules (registers, types, error)
- Updated author to Trevor Flahardy
- Enhanced Cargo.toml metadata

### Fixed
- Proper voltage range validation for all power rails
- Correct step size checking for voltage settings
- Proper register bit manipulation

### Removed
- Old partial implementation
- DataFormat enum (replaced with direct byte slice usage)
- Unused constants and outdated comments

## [0.1.0] - Previous

### Added
- Initial partial implementation
- Basic I2C communication
- Limited power rail support

