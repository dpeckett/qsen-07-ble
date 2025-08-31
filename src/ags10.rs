//! Minimal, `no_std` driver for the Aosong AGS10 TVOC sensor.
//!
//! Implements the I²C protocol described in the datasheet: TVOC acquire, read
//! resistance, read firmware version, zero-point calibration (factory / current
//! resistance / nominated resistance), and changing the slave address.
//!
//! ## Bus speed and timing
//! * The AGS10 specifies an I²C speed ≤ 15 kHz.
//! * Leave ≥30 ms between commands.
//! * The data-acquisition command (TVOC read) must not be issued faster than
//!   once every 1.5 s.
//! * Typical pre-heat time is ~120 s after power-up before measurements
//!   stabilize.
//!
//! Enforcing those timings is left to the application.
//!
//! ## Example
//! ```no_run
//! use ags10::Ags10;
//! use embedded_hal::i2c::I2c;
//!
//! // `I2C` is your MCU/HAL's I²C peripheral type implementing `embedded_hal::i2c::I2c`.
//! fn demo<I2C, E>(mut i2c: I2C) -> Result<(), ags10::Error<E>>
//! where
//!     I2C: embedded_hal::i2c::I2c<Error = E>,
//! {
//!     let mut ags = Ags10::new(i2c);
//!
//!     // Wait for sensor warm-up in your app before reading…
//!     let (tvoc_ppb, status) = ags.read_tvoc()?;
//!     // status.raw bit0 is RDY flag per datasheet
//!     let ohms_0_1k = ags.read_resistance()?; // unit: 0.1 kΩ
//!     let version = ags.read_version()?;
//!
//!     // Optional: zero-point calibrations
//!     // ags.calibrate_factory()?;                  // reset zero-point to factory
//!     // ags.calibrate_current_resistance()?;       // set current R as zero-point
//!     // ags.calibrate_with_resistance(0x1CBC)?;    // nominated R (0.1 kΩ)
//!
//!     // Optional: change the device address (persists across power cycles)
//!     // ags.set_address(0x2A)?;
//!
//!     // Recover the I²C bus
//!     let _i2c = ags.free();
//!     Ok(())
//! }
//! ```

use embedded_hal::i2c::I2c;

/// Default 7-bit I²C address
pub const DEFAULT_ADDRESS: u8 = 0x1A;

// Registers
const TVOC_REG: u8 = 0x00;
const CALIB_REG: u8 = 0x01;
const VERS_REG: u8 = 0x11;
const RESIST_REG: u8 = 0x20;
const SLAVE_REG: u8 = 0x21;

/// Driver error type: wraps I²C errors and CRC mismatches.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum Error<E> {
    /// Underlying I²C bus error
    I2c(E),
    /// CRC8 mismatch on a response
    Crc,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Self::I2c(e)
    }
}

/// Status byte returned by `read_tvoc`.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub struct Status {
    /// Raw status byte. Bit0 is RDY in the datasheet.
    pub raw: u8,
}

/// AGS10 driver
pub struct Ags10<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> Ags10<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Create a new driver at the default address (0x1A).
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            address: DEFAULT_ADDRESS,
        }
    }

    /// Create a new driver for a custom 7-bit address.
    pub fn with_address(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Return the underlying I²C peripheral.
    pub fn free(self) -> I2C {
        self.i2c
    }

    /// Read TVOC in ppb and the status byte. Performs CRC8 validation.
    pub fn read_tvoc(&mut self) -> Result<(u32, Status), Error<E>> {
        let mut buf = [0u8; 5];
        self.read_reg(TVOC_REG, &mut buf)?;
        if !crc8_ok(&buf) {
            return Err(Error::Crc);
        }
        let status = Status { raw: buf[0] };
        let tvoc = u32::from(buf[1]) << 16 | u32::from(buf[2]) << 8 | u32::from(buf[3]);
        Ok((tvoc, status))
    }

    /// Read current sensor resistance (unit: 0.1 kΩ). Performs CRC8 validation.
    pub fn read_resistance(&mut self) -> Result<u32, Error<E>> {
        let mut buf = [0u8; 5];
        self.read_reg(RESIST_REG, &mut buf)?;
        if !crc8_ok(&buf) {
            return Err(Error::Crc);
        }
        let r = u32::from(buf[0]) << 24
            | u32::from(buf[1]) << 16
            | u32::from(buf[2]) << 8
            | u32::from(buf[3]);
        Ok(r)
    }

    /// Read firmware version (byte 4 in the response). Performs CRC8 validation.
    pub fn read_version(&mut self) -> Result<u8, Error<E>> {
        let mut buf = [0u8; 5];
        self.read_reg(VERS_REG, &mut buf)?;
        if !crc8_ok(&buf) {
            return Err(Error::Crc);
        }
        Ok(buf[3])
    }

    /// Reset zero-point to factory value.
    pub fn calibrate_factory(&mut self) -> Result<(), Error<E>> {
        self.calibrate([0x00, 0x0C, 0xFF, 0xFF])
    }

    /// Set zero-point to the *current* measured resistance.
    pub fn calibrate_current_resistance(&mut self) -> Result<(), Error<E>> {
        self.calibrate([0x00, 0x0C, 0x00, 0x00])
    }

    /// Set zero-point to a nominated resistance (big-endian, unit: 0.1 kΩ).
    pub fn calibrate_with_resistance(&mut self, r_0_1k: u16) -> Result<(), Error<E>> {
        let bytes = r_0_1k.to_be_bytes();
        self.calibrate([0x00, 0x0C, bytes[0], bytes[1]])
    }

    /// Permanently change the 7-bit I²C address. Takes effect immediately and
    /// persists across power cycles. Updates the driver's internal address.
    pub fn set_address(&mut self, new_addr: u8) -> Result<(), Error<E>> {
        let inv = !new_addr;
        let data = [new_addr, inv, new_addr, inv];
        self.write_with_crc(SLAVE_REG, &data)?;
        self.address = new_addr;
        Ok(())
    }

    // --- helpers ---

    fn calibrate(&mut self, d: [u8; 4]) -> Result<(), Error<E>> {
        self.write_with_crc(CALIB_REG, &d)
    }

    fn read_reg(&mut self, reg: u8, buf: &mut [u8; 5]) -> Result<(), Error<E>> {
        self.i2c
            .write_read(self.address, core::slice::from_ref(&reg), buf)
            .map_err(Error::I2c)
    }

    fn write_with_crc(&mut self, reg: u8, payload4: &[u8; 4]) -> Result<(), Error<E>> {
        let crc = crc8(&payload4[..]);
        let mut frame = [0u8; 1 + 4 + 1];
        frame[0] = reg;
        frame[1..5].copy_from_slice(payload4);
        frame[5] = crc;
        self.i2c.write(self.address, &frame).map_err(Error::I2c)
    }
}

/// Compute CRC8 with init 0xFF and poly 0x31 over an arbitrary-length slice.
fn crc8(d: &[u8]) -> u8 {
    let mut crc = 0xFFu8;
    for &b in d {
        crc ^= b;
        for _ in 0..8 {
            crc = if (crc & 0x80) != 0 {
                (crc << 1) ^ 0x31
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Check CRC8 of a 5-byte response: CRC is byte 4, computed over bytes 0..=3.
fn crc8_ok(r: &[u8; 5]) -> bool {
    crc8(&r[..4]) == r[4]
}
