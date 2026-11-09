//! `hkResult`-style error codes.
//!
//! Port of the `hkResult` value type and the `HK_SUCCESS` / `HK_FAILURE` /
//! `HK_E_*` codes in
//! `original/Common/Source/Common/Base/Types/hkBaseTypes.h` (lines 731–760).
//!
//! Havok reports failures through return codes rather than exceptions; the port
//! keeps the numeric codes (they appear in logs and in the content toolchain) but
//! offers a small Rust wrapper with `Result`-like helpers. `HkResult` is deliberately
//! **not** `Result<T, E>`: the original API returns a status and writes through
//! out-parameters, and that shape is friendlier to the solver hot paths.
//!
//! ```
//! use hk_types::HkResult;
//! assert!(HkResult::SUCCESS.is_success());
//! assert!(!HkResult::FAILURE.is_success());
//! assert_eq!(HkResult::FAILURE.code(), 0x8004_0200u32 as i32);
//! ```

use core::fmt;

/// Return code (`hkResult` is a wrapper around a signed 32-bit code; success is
/// `>= 0`, failure uses `0x8004xxxx`-style negative codes).
#[repr(transparent)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct HkResult(i32);

impl HkResult {
    /// `HK_SUCCESS`.
    pub const SUCCESS: Self = Self(0);

    /// `HK_FAILURE` — generic failure.
    pub const FAILURE: Self = Self(0x8004_0200u32 as i32);
    /// `HK_E_FILE_NOT_FOUND`.
    pub const E_FILE_NOT_FOUND: Self = Self(0x8004_0202u32 as i32);
    /// `HK_E_OUTOFMEMORY`.
    pub const E_OUTOFMEMORY: Self = Self(0x8004_020E_u32 as i32);
    /// `HK_E_WRITE_PROTECT`.
    pub const E_WRITE_PROTECT: Self = Self(0x8004_0213u32 as i32);
    /// `HK_E_HANDLE_EOF`.
    pub const E_HANDLE_EOF: Self = Self(0x8004_0226u32 as i32);
    /// `HK_E_NOT_SUPPORTED`.
    pub const E_NOT_SUPPORTED: Self = Self(0x8004_0232u32 as i32);
    /// `HK_E_INVALID_PARAMETER`.
    pub const E_INVALID_PARAMETER: Self = Self(0x8004_0257u32 as i32);
    /// `HK_E_NOT_IMPLEMENTED`.
    pub const E_NOT_IMPLEMENTED: Self = Self(0x8004_0278u32 as i32);
    /// `HK_E_ALREADY_EXISTS`.
    pub const E_ALREADY_EXISTS: Self = Self(0x8004_02B7u32 as i32);
    /// `HK_E_NOTFOUND`.
    pub const E_NOTFOUND: Self = Self(0x8004_0490u32 as i32);
    /// `HK_E_UNEXPECTED`.
    pub const E_UNEXPECTED: Self = Self(0x8004_FFFFu32 as i32);
    /// `HK_E_FEATURE_MISSING`.
    pub const E_FEATURE_MISSING: Self = Self(0x8004_013C_u32 as i32);
    /// `HK_E_UNUSED`.
    pub const E_UNUSED: Self = Self(0x8004_8005u32 as i32);
    /// `HK_E_RETVAL_INVALID_USAGE`.
    pub const E_RETVAL_INVALID_USAGE: Self = Self(0x8004_4001u32 as i32);

    /// Wrap a raw status code (e.g. read back from a binary blob).
    #[inline]
    pub const fn from_code(code: i32) -> Self {
        Self(code)
    }

    /// The raw status code.
    #[inline]
    pub const fn code(self) -> i32 {
        self.0
    }

    /// Success means a non-negative code (`hkResult::isSuccess` semantics).
    #[inline]
    pub const fn is_success(self) -> bool {
        self.0 >= 0
    }

    /// Negation of [`HkResult::is_success`].
    #[inline]
    pub const fn is_failure(self) -> bool {
        self.0 < 0
    }
}

impl From<i32> for HkResult {
    #[inline]
    fn from(code: i32) -> Self {
        Self(code)
    }
}

impl From<HkResult> for i32 {
    #[inline]
    fn from(result: HkResult) -> Self {
        result.0
    }
}

impl fmt::Display for HkResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (name, description) = match self.0 {
            0 => ("HK_SUCCESS", "Success."),
            x if x == Self::FAILURE.0 => ("HK_FAILURE", "Generic failure."),
            x if x == Self::E_FILE_NOT_FOUND.0 => ("HK_E_FILE_NOT_FOUND", "File not found."),
            x if x == Self::E_OUTOFMEMORY.0 => (
                "HK_E_OUTOFMEMORY",
                "Out of memory, cannot perform operation.",
            ),
            x if x == Self::E_WRITE_PROTECT.0 => (
                "HK_E_WRITE_PROTECT",
                "Cannot access write-protected entity.",
            ),
            x if x == Self::E_HANDLE_EOF.0 => ("HK_E_HANDLE_EOF", "End of file."),
            x if x == Self::E_NOT_SUPPORTED.0 => ("HK_E_NOT_SUPPORTED", "Operation not supported."),
            x if x == Self::E_INVALID_PARAMETER.0 => {
                ("HK_E_INVALID_PARAMETER", "Invalid operation parameter.")
            }
            x if x == Self::E_NOT_IMPLEMENTED.0 => {
                ("HK_E_NOT_IMPLEMENTED", "Operation not implemented.")
            }
            x if x == Self::E_ALREADY_EXISTS.0 => (
                "HK_E_ALREADY_EXISTS",
                "Object already exists, cannot create or modify it.",
            ),
            x if x == Self::E_NOTFOUND.0 => ("HK_E_NOTFOUND", "Element not found."),
            x if x == Self::E_UNEXPECTED.0 => ("HK_E_UNEXPECTED", "Unexpected internal state."),
            x if x == Self::E_FEATURE_MISSING.0 => (
                "HK_E_FEATURE_MISSING",
                "A needed feature is missing from the current configuration.",
            ),
            x if x == Self::E_UNUSED.0 => ("HK_E_UNUSED", "A parameter was unused."),
            x if x == Self::E_RETVAL_INVALID_USAGE.0 => (
                "HK_E_RETVAL_INVALID_USAGE",
                "hkRetVal was built from invalid arguments.",
            ),
            // Unknown code, e.g. produced by `fromErrorCode(HRESULT)`.
            x if x >= 0 => return write!(f, "HK_SUCCESS_{:08X}", x as u32),
            x => return write!(f, "HK_E_{:08X}", x as u32),
        };

        write!(f, "{name}: {description}")
    }
}

impl std::error::Error for HkResult {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn codes_match_the_sdk_constants() {
        assert_eq!(HkResult::SUCCESS.code(), 0);
        assert_eq!(HkResult::FAILURE.code(), 0x80040200u32 as i32);
        assert_eq!(HkResult::E_FILE_NOT_FOUND.code(), 0x80040202u32 as i32);
        assert_eq!(HkResult::E_OUTOFMEMORY.code(), 0x8004020Eu32 as i32);
        assert_eq!(HkResult::E_NOTFOUND.code(), 0x80040490u32 as i32);
    }

    #[test]
    fn success_is_non_negative() {
        assert!(HkResult::SUCCESS.is_success());
        assert!(HkResult::from_code(5).is_success());
        assert!(!HkResult::FAILURE.is_success());
        assert!(HkResult::FAILURE.is_failure());
    }

    #[test]
    fn round_trip_through_i32() {
        let result = HkResult::from(0x80040202u32 as i32);
        assert_eq!(i32::from(result), 0x80040202u32 as i32);
        assert_eq!(result, HkResult::E_FILE_NOT_FOUND);
    }

    #[test]
    fn display_names() {
        assert_eq!(HkResult::SUCCESS.to_string(), "HK_SUCCESS: Success.");
        assert_eq!(
            HkResult::E_NOTFOUND.to_string(),
            "HK_E_NOTFOUND: Element not found."
        );
        assert_eq!(
            HkResult::from_code(0x1234).to_string(),
            "HK_SUCCESS_00001234"
        );
    }
}
