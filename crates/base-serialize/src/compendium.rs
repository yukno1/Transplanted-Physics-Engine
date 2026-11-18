//! Type compendium (`TCM0`) reader and type-table merge.
//!
//! Rust subset of `hkSerialize::TagfileReadFormat::Impl::Compendium` and
//! `handleCompendiumId` from `hkTagfileReadFormat.cpp`.  A compendium carries
//! the shared `TYPE` section plus a `TCID` array of little-endian signatures;
//! a data bundle references one of them through `TCRF`.
//!
//! Only the metadata needed to materialise generic values is retained.  The
//! SDK's in-place type pointer array, rolling type hash (`TSHA`/`TSEQ`) and
//! incremental type checker are not ported.

use crate::hff::HffNode;
use crate::object::{LoadError, type_info_from_tagfile};
use crate::tagfile::{self, Tagfile};
use crate::type_meta::TypeInfo;

/// A parsed `TCM0` type compendium.
#[derive(Clone, Debug, PartialEq)]
pub struct Compendium {
    /// Container the signatures and types came from.
    pub tagfile: Tagfile,
    /// Reflection metadata shared with referencing bundles.
    pub type_info: TypeInfo,
    /// Signatures from `TCID`, one per intermediate compendium state.
    pub signatures: Vec<u64>,
}

impl Compendium {
    /// Load a `TCM0` compendium.
    pub fn load(bytes: &[u8]) -> Result<Self, LoadError> {
        let tagfile = Tagfile::parse(bytes)?;
        if !tagfile.is_compendium() {
            return Err(LoadError::Unsupported("TCM0 compendium required"));
        }
        let signatures = parse_signatures(tagfile.section(tagfile::COMPENDIUM_ID))?;
        let type_info = type_info_from_tagfile(&tagfile)?;
        Ok(Self {
            tagfile,
            type_info,
            signatures,
        })
    }

    /// Whether this compendium has the signature a bundle asks for.
    pub fn contains_signature(&self, signature: u64) -> bool {
        self.signatures.contains(&signature)
    }
}

fn parse_signatures(node: Option<&HffNode>) -> Result<Vec<u64>, LoadError> {
    let node = node.ok_or(LoadError::MissingSection("TCID"))?;
    if !node.is_leaf() {
        return Err(LoadError::Unsupported("TCID"));
    }
    if node.data.len() % 8 != 0 {
        return Err(LoadError::InvalidSignatureSection);
    }
    Ok(node
        .data
        .chunks_exact(8)
        .map(|chunk| u64::from_le_bytes(chunk.try_into().expect("chunk of 8 bytes")))
        .collect())
}

/// Merge `source` into `target`; entries in `source` win on equal type ids.
///
/// Compendia are applied in caller order first, then a bundle's own `TYPE`
/// section, mirroring how the SDK registers shared types before bundle-local
/// additions.  `None` entries never overwrite an existing definition.
pub(crate) fn merge_type_info(target: &mut TypeInfo, source: &TypeInfo) {
    if target.identities.len() < source.identities.len() {
        target.identities.resize(source.identities.len(), None);
    }
    if target.types.len() < source.types.len() {
        target.types.resize(source.types.len(), None);
    }
    for (index, identity) in source.identities.iter().enumerate() {
        if identity.is_some() {
            target.identities[index] = identity.clone();
        }
    }
    for (index, meta) in source.types.iter().enumerate() {
        if meta.is_some() {
            target.types[index] = meta.clone();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hff::HffScope;

    fn node(ident: u32, scope: HffScope, data: Vec<u8>, children: Vec<HffNode>) -> HffNode {
        HffNode {
            ident,
            scope,
            data,
            children,
        }
    }

    #[test]
    fn parses_signature_array_as_little_endian_u64() {
        let mut payload = Vec::new();
        for signature in [0x0102_0304_0506_0708u64, 0x1122_3344_5566_7788] {
            payload.extend_from_slice(&signature.to_le_bytes());
        }
        let signatures = parse_signatures(Some(&node(
            tagfile::COMPENDIUM_ID,
            HffScope::Leaf,
            payload,
            Vec::new(),
        )))
        .expect("signatures");
        assert_eq!(
            signatures,
            vec![0x0102_0304_0506_0708, 0x1122_3344_5566_7788]
        );
    }

    #[test]
    fn merge_keeps_existing_entries_and_grows_tables() {
        use crate::type_meta::TypeIdentity;

        let identity = |name: &str| {
            Some(TypeIdentity {
                name: Some(name.to_string()),
                params: Vec::new(),
            })
        };
        let mut target = TypeInfo {
            identities: vec![None, identity("local")],
            types: vec![None],
        };
        let source = TypeInfo {
            identities: vec![None, None, identity("shared")],
            types: vec![None, None],
        };
        merge_type_info(&mut target, &source);
        assert_eq!(target.identities.len(), 3);
        assert_eq!(
            target.identities[1]
                .as_ref()
                .and_then(|id| id.name.as_deref()),
            Some("local")
        );
        assert_eq!(
            target.identities[2]
                .as_ref()
                .and_then(|id| id.name.as_deref()),
            Some("shared")
        );
        assert_eq!(target.types.len(), 2);
    }

    #[test]
    fn rejects_malformed_signature_array() {
        assert_eq!(
            parse_signatures(None),
            Err(LoadError::MissingSection("TCID"))
        );
        let branch = node(
            tagfile::COMPENDIUM_ID,
            HffScope::Branch,
            vec![0; 8],
            Vec::new(),
        );
        assert_eq!(
            parse_signatures(Some(&branch)),
            Err(LoadError::Unsupported("TCID"))
        );
        let short = node(
            tagfile::COMPENDIUM_ID,
            HffScope::Leaf,
            vec![0; 7],
            Vec::new(),
        );
        assert_eq!(
            parse_signatures(Some(&short)),
            Err(LoadError::InvalidSignatureSection)
        );
    }
}
