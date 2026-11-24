//! Unsigned variable-length integers (`hkUintVle`).
//!
//! Direct port of `original/Common/Source/Common/Base/System/Io/Detail/hkUintVle.cpp`.
//! The encoding is big-endian after the prefix bits are masked off; Havok
//! guarantees up to nine bytes of padding after every VLE in a tagfile, but
//! this reader validates every access so malformed input is rejected safely.

/// Errors produced while decoding a VLE integer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VleError {
    /// The first byte is not a valid VLE prefix.
    InvalidPrefix {
        /// Offending prefix byte.
        byte: u8,
    },
    /// The encoded integer needs more bytes than are available.
    Truncated {
        /// Number of bytes required.
        needed: usize,
        /// Number of bytes available.
        available: usize,
    },
}

impl core::fmt::Display for VleError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidPrefix { byte } => write!(f, "invalid VLE prefix byte {byte:02x}"),
            Self::Truncated { needed, available } => {
                write!(f, "VLE needs {needed} bytes, only {available} available")
            }
        }
    }
}

impl std::error::Error for VleError {}

/// Decode one VLE integer starting at `bytes[0]`.
///
/// Returns the value and the number of bytes consumed.
pub fn read(bytes: &[u8]) -> Result<(u64, usize), VleError> {
    let p0 = *bytes.first().ok_or(VleError::Truncated {
        needed: 1,
        available: 0,
    })?;

    if p0 & 0x80 == 0 {
        return Ok((p0 as u64, 1));
    }

    let shift_group = p0 >> 3;
    let needed = match shift_group {
        0x10..=0x17 => 2,
        0x18..=0x1b => 3,
        0x1c => 4,
        0x1d => 5,
        0x1e => 8,
        0x1f => match p0 & 0x07 {
            0x00 => 6,
            0x01 => 9,
            _ => return Err(VleError::InvalidPrefix { byte: p0 }),
        },
        _ => return Err(VleError::InvalidPrefix { byte: p0 }),
    };

    if bytes.len() < needed {
        return Err(VleError::Truncated {
            needed,
            available: bytes.len(),
        });
    }

    let first8 = if needed <= 8 {
        let mut buf = [0u8; 8];
        buf[..needed].copy_from_slice(&bytes[..needed]);
        u64::from_be_bytes(buf)
    } else {
        0
    };

    let value = match shift_group {
        0x10..=0x17 => (first8 >> 48) & ((1u64 << 14) - 1),
        0x18..=0x1b => (first8 >> 40) & ((1u64 << 21) - 1),
        0x1c => (first8 >> 32) & ((1u64 << 27) - 1),
        0x1d => (first8 >> 24) & ((1u64 << 35) - 1),
        0x1e => first8 & ((1u64 << 59) - 1),
        0x1f => match p0 & 0x07 {
            0x00 => (first8 >> 16) & ((1u64 << 40) - 1),
            0x01 => {
                let mut buf = [0u8; 8];
                buf.copy_from_slice(&bytes[1..9]);
                u64::from_be_bytes(buf)
            }
            _ => unreachable!("prefix validated above"),
        },
        _ => unreachable!("prefix validated above"),
    };

    Ok((value, needed))
}

/// Decode a VLE integer and advance a byte cursor.
pub fn read_at(bytes: &[u8], cursor: &mut usize) -> Result<u64, VleError> {
    let (value, consumed) = read(bytes.get(*cursor..).unwrap_or(&[]))?;
    *cursor += consumed;
    Ok(value)
}

/// Encode a value with the shortest Havok VLE representation.
///
/// Appends to `out` and returns the number of bytes written.
pub fn write(value: u64, out: &mut Vec<u8>) -> usize {
    let fits = |bits: u32| value < (1u64 << bits);
    if fits(7) {
        out.push(value as u8);
        1
    } else if fits(14) {
        out.push(0x80 | (value >> 8) as u8);
        out.push(value as u8);
        2
    } else if fits(21) {
        out.push(0xc0 | (value >> 16) as u8);
        out.push((value >> 8) as u8);
        out.push(value as u8);
        3
    } else if fits(27) {
        out.push(0xe0 | (value >> 24) as u8);
        out.push((value >> 16) as u8);
        out.push((value >> 8) as u8);
        out.push(value as u8);
        4
    } else if fits(35) {
        out.push(0xe8 | (value >> 32) as u8);
        out.push((value >> 24) as u8);
        out.push((value >> 16) as u8);
        out.push((value >> 8) as u8);
        out.push(value as u8);
        5
    } else if fits(40) {
        out.push(0xf8);
        out.push((value >> 32) as u8);
        out.push((value >> 24) as u8);
        out.push((value >> 16) as u8);
        out.push((value >> 8) as u8);
        out.push(value as u8);
        6
    } else if fits(59) {
        out.push(0xf0 | (value >> 56) as u8);
        out.push((value >> 48) as u8);
        out.push((value >> 40) as u8);
        out.push((value >> 32) as u8);
        out.push((value >> 24) as u8);
        out.push((value >> 16) as u8);
        out.push((value >> 8) as u8);
        out.push(value as u8);
        8
    } else {
        out.push(0xf9);
        out.push((value >> 56) as u8);
        out.push((value >> 48) as u8);
        out.push((value >> 40) as u8);
        out.push((value >> 32) as u8);
        out.push((value >> 24) as u8);
        out.push((value >> 16) as u8);
        out.push((value >> 8) as u8);
        out.push(value as u8);
        9
    }
}

/// Encode a signed value using `hkUintVle::encodeSigned`.
pub fn write_signed(value: i64, out: &mut Vec<u8>) -> usize {
    let encoded = if value >= 0 {
        (value as u64) << 1
    } else {
        ((-(value + 1) as u64) << 1) | 1
    };
    write(encoded, out)
}

/// Decode a signed value written by [`write_signed`].
pub fn decode_signed(value: u64) -> i64 {
    let magnitude = value >> 1;
    if value & 1 == 0 {
        magnitude as i64
    } else {
        -(magnitude as i64) - 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrips_all_widths() {
        for value in [
            0,
            1,
            0x7f,
            0x80,
            0x3fff,
            0x4000,
            0x1f_ffff,
            0x20_0000,
            0x7ff_ffff,
            0x800_0000,
            0x7_ffff_ffff,
            0x80_0000_0000,
            0xff_ffff_ffff,
            0x7ff_ffff_ffff_ffff,
            0x800_0000_0000_0000,
            u64::MAX,
        ] {
            let mut bytes = Vec::new();
            let n = write(value, &mut bytes);
            let (decoded, consumed) = read(&bytes).expect("vle");
            assert_eq!(decoded, value);
            assert_eq!(consumed, n);
        }
    }

    #[test]
    fn roundtrips_signed_values() {
        for value in [0, 1, -1, 63, -64, i64::MAX, i64::MIN] {
            let mut bytes = Vec::new();
            write_signed(value, &mut bytes);
            let (encoded, _) = read(&bytes).expect("vle");
            assert_eq!(decode_signed(encoded), value);
        }
    }

    #[test]
    fn rejects_malformed_input() {
        assert_eq!(
            read(&[]),
            Err(VleError::Truncated {
                needed: 1,
                available: 0
            })
        );
        assert_eq!(read(&[0xff]), Err(VleError::InvalidPrefix { byte: 0xff }));
        assert!(matches!(
            read(&[0x80]),
            Err(VleError::Truncated { needed: 2, .. })
        ));
    }
}
