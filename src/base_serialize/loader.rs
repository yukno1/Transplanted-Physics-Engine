//! Unified hkx loading entry point.
//!
//! Existing readers remain unchanged: [`crate::base_serialize::object::Bundle`] handles `TAG0`,
//! [`crate::base_serialize::compendium::Compendium`] handles `TCM0`, and
//! [`crate::base_serialize::tagfile2014::Bundle2014`] handles legacy `CAB00D1E` streams. This
//! module only formats-detects, routes packfiles through the external
//! conversion boundary, and optionally applies version patches.

use std::borrow::Cow;

use crate::base_serialize::compendium::Compendium;
use crate::base_serialize::object::{Bundle, LoadError};
use crate::base_serialize::packfile::{
    PackfileConverter, PackfileError, PackfilePolicy, convert_packfile,
};
use crate::base_serialize::tagfile2014::{Bundle2014, Error as Tagfile2014Error};
use crate::base_serialize::version::{
    PatchError, PatchReport, PatchSet, SerializeFormat, detect_format,
};

/// Options for [`load_hkx`].
#[derive(Clone, Default)]
pub struct LoadOptions<'a> {
    /// Compendia used by `TAG0` bundles carrying `TCRF`.
    pub compendia: &'a [Compendium],
    /// Optional header constraints applied to binary packfiles.
    pub packfile_policy: PackfilePolicy,
    /// External converter for 2014 binary packfiles.
    pub packfile_converter: Option<&'a dyn PackfileConverter>,
    /// Patch table applied after loading when present.
    pub patches: Option<&'a PatchSet>,
}

/// Errors produced by the unified loader.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LoadHkxError {
    /// The byte prefix did not name a supported container.
    UnsupportedFormat(SerializeFormat),
    /// A `TCM0` container was supplied where a data bundle was required.
    CompendiumAsBundle,
    /// A `TAG0` bundle could not be materialised.
    Bundle(LoadError),
    /// A legacy tagfile could not be materialised.
    Tagfile2014(Tagfile2014Error),
    /// A packfile header or conversion step failed.
    Packfile(PackfileError),
    /// Version patching failed.
    Patch(PatchError),
    /// Patches were requested for a container without generic `Value` objects.
    PatchTargetUnsupported,
}

impl core::fmt::Display for LoadHkxError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnsupportedFormat(format) => write!(f, "unsupported hkx format {format:?}"),
            Self::CompendiumAsBundle => write!(f, "TCM0 compendium is not a data bundle"),
            Self::Bundle(error) => write!(f, "tagfile bundle error: {error}"),
            Self::Tagfile2014(error) => write!(f, "legacy tagfile error: {error}"),
            Self::Packfile(error) => write!(f, "binary packfile error: {error}"),
            Self::Patch(error) => write!(f, "version patch error: {error}"),
            Self::PatchTargetUnsupported => {
                write!(f, "version patches require a TAG0 generic bundle")
            }
        }
    }
}

impl std::error::Error for LoadHkxError {}

impl From<LoadError> for LoadHkxError {
    fn from(value: LoadError) -> Self {
        Self::Bundle(value)
    }
}

impl From<Tagfile2014Error> for LoadHkxError {
    fn from(value: Tagfile2014Error) -> Self {
        Self::Tagfile2014(value)
    }
}

impl From<PackfileError> for LoadHkxError {
    fn from(value: PackfileError) -> Self {
        Self::Packfile(value)
    }
}

impl From<PatchError> for LoadHkxError {
    fn from(value: PatchError) -> Self {
        Self::Patch(value)
    }
}

/// A container loaded through [`load_hkx`].
#[derive(Clone, Debug, PartialEq)]
pub enum LoadedHkx {
    /// A `TAG0` generic value bundle.
    Bundle(Bundle),
    /// A `TCM0` type compendium.
    Compendium(Compendium),
    /// A legacy 2014 binary tagfile bundle.
    Tagfile2014(Box<Bundle2014>),
}

impl LoadedHkx {
    /// Borrow the generic `TAG0` bundle when available.
    pub fn bundle(&self) -> Option<&Bundle> {
        match self {
            Self::Bundle(bundle) => Some(bundle),
            _ => None,
        }
    }

    /// Mutably borrow the generic `TAG0` bundle when available.
    pub fn bundle_mut(&mut self) -> Option<&mut Bundle> {
        match self {
            Self::Bundle(bundle) => Some(bundle),
            _ => None,
        }
    }

    /// Apply a patch table to a generic bundle.
    ///
    /// `chain` selects version-chain semantics; `false` preserves the original
    /// single-step application used by [`PatchSet::apply`].
    pub fn apply_patches(
        &mut self,
        patches: &PatchSet,
        chain: bool,
    ) -> Result<PatchReport, LoadHkxError> {
        let Some(bundle) = self.bundle_mut() else {
            return Err(LoadHkxError::PatchTargetUnsupported);
        };
        let report = if chain {
            patches.apply_chain(bundle)?
        } else {
            patches.apply(bundle)
        };
        Ok(report)
    }
}

/// Detect the container, convert packfiles when a converter is present, and
/// optionally apply a patch table.
pub fn load_hkx(bytes: &[u8], options: &LoadOptions<'_>) -> Result<LoadedHkx, LoadHkxError> {
    let (source, format) = match detect_format(bytes) {
        SerializeFormat::BinaryPackfile2014 => {
            let (_, converted) =
                convert_packfile(bytes, &options.packfile_policy, options.packfile_converter)?;
            let format = detect_format(&converted);
            (Cow::Owned(converted), format)
        }
        format => (Cow::Borrowed(bytes), format),
    };

    let mut loaded = match format {
        SerializeFormat::BinaryTagfile => {
            LoadedHkx::Bundle(Bundle::load_with_compendia(&source, options.compendia)?)
        }
        SerializeFormat::BinaryTagfileCompendium => {
            LoadedHkx::Compendium(Compendium::load(&source)?)
        }
        SerializeFormat::LegacyBinaryTagfile => {
            LoadedHkx::Tagfile2014(Box::new(Bundle2014::load(&source)?))
        }
        other => return Err(LoadHkxError::UnsupportedFormat(other)),
    };

    if let Some(patches) = options.patches {
        loaded.apply_patches(patches, true)?;
    }
    Ok(loaded)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_unsupported_prefix() {
        let error = load_hkx(b"not hkx", &LoadOptions::default()).expect_err("unsupported");
        assert_eq!(
            error,
            LoadHkxError::UnsupportedFormat(SerializeFormat::Unknown)
        );
    }

    #[test]
    fn routes_compendium_container() {
        use crate::base_serialize::hff::{four_cc, pack_scope};

        let mut bytes = pack_scope(0, 8).to_be_bytes().to_vec();
        bytes.extend_from_slice(&four_cc(b"TCM0").to_be_bytes());
        let error = load_hkx(&bytes, &LoadOptions::default()).expect_err("not a data bundle");
        assert_eq!(
            error,
            LoadHkxError::Bundle(LoadError::MissingSection("TCID"))
        );
    }
}
