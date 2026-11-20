//! Facade-level hkx format sniffing and synthetic version-patching scenarios.

use proton::serialize::{
    Bundle, LegacyType, PatchComponent, PatchDefault, PatchInfo, PatchSet, SerializeFormat, Value,
    detect_format, four_cc, pack_scope, vle,
};

fn leaf(ident: u32, payload: &[u8]) -> Vec<u8> {
    let size = 8 + payload.len();
    let mut bytes = pack_scope(1, size as u32).to_be_bytes().to_vec();
    bytes.extend_from_slice(&ident.to_be_bytes());
    bytes.extend_from_slice(payload);
    bytes
}

fn branch(ident: u32, children: &[Vec<u8>]) -> Vec<u8> {
    let size = 8 + children.iter().map(Vec::len).sum::<usize>();
    let mut bytes = pack_scope(0, size as u32).to_be_bytes().to_vec();
    bytes.extend_from_slice(&ident.to_be_bytes());
    for child in children {
        bytes.extend_from_slice(child);
    }
    bytes
}

/// `Demo` record with declared TYPE version 1 and a single `value: i32`.
fn versioned_tagfile() -> Vec<u8> {
    let mut identities = Vec::new();
    vle::write(3, &mut identities);
    vle::write(0, &mut identities); // "Demo"
    vle::write(0, &mut identities);
    vle::write(1, &mut identities); // "i32"
    vle::write(0, &mut identities);

    let mut bodies = Vec::new();
    vle::write(1, &mut bodies); // type id
    vle::write(0, &mut bodies); // parent
    // compressed opt bits: FORMAT | VERSION | SIZE_ALIGN | DECLS
    vle::write((1 << 0) | (1 << 2) | (1 << 3) | (1 << 5), &mut bodies);
    vle::write(7, &mut bodies); // KIND_RECORD
    vle::write(1, &mut bodies); // declared version
    vle::write(4, &mut bodies); // size
    vle::write(4, &mut bodies); // align
    vle::write(1, &mut bodies); // one field
    vle::write(0, &mut bodies); // "value"
    vle::write(0x20, &mut bodies); // DECL_DATA_FIELD
    vle::write(0, &mut bodies);
    vle::write(2, &mut bodies);
    // id 2: signed int32 used by the `value` field.
    vle::write(2, &mut bodies);
    vle::write(0, &mut bodies);
    vle::write((1 << 0) | (1 << 3), &mut bodies);
    vle::write(4 | (1 << 9) | (32 << 10), &mut bodies);
    vle::write(4, &mut bodies);
    vle::write(4, &mut bodies);
    bodies.push(0);

    let mut items = Vec::new();
    items.extend_from_slice(&0u32.to_be_bytes());
    items.extend_from_slice(&0u32.to_be_bytes());
    items.extend_from_slice(&0u32.to_be_bytes());
    items.extend_from_slice(&(0x1000_0000u32 | 1).to_be_bytes());
    items.extend_from_slice(&0u32.to_be_bytes());
    items.extend_from_slice(&1u32.to_be_bytes());

    branch(
        four_cc(b"TAG0"),
        &[
            leaf(four_cc(b"SDKV"), b"20211202"),
            leaf(four_cc(b"DATA"), &123i32.to_le_bytes()),
            branch(
                four_cc(b"TYPE"),
                &[
                    leaf(four_cc(b"TST1"), b"Demo\0i32\0"),
                    leaf(four_cc(b"FST1"), b"value\0"),
                    leaf(four_cc(b"TNA1"), &identities),
                    leaf(four_cc(b"TBDY"), &bodies),
                ],
            ),
            branch(four_cc(b"INDX"), &[leaf(four_cc(b"ITEM"), &items)]),
        ],
    )
}

#[test]
fn v1_sdk_version_is_retained_and_declared_type_version_is_decoded() {
    let bundle = Bundle::load(&versioned_tagfile()).expect("bundle");
    assert_eq!(bundle.sdk_version.as_deref(), Some("20211202"));
    let meta = bundle.type_info.types[1].as_ref().expect("Demo metadata");
    assert_eq!(meta.name.as_deref(), Some("Demo"));
    assert_eq!(meta.version, Some(1));
}

#[test]
fn v2_patch_set_renames_removes_adds_and_overwrites_fields() {
    let mut bundle = Bundle::load(&versioned_tagfile()).expect("bundle");
    let mut patches = PatchSet::new();
    patches.add(
        PatchInfo::new("Demo", "DemoV2", 1, 2)
            .with_component(PatchComponent::MemberRenamed {
                old: "value".into(),
                new: "m_value".into(),
            })
            .with_component(PatchComponent::MemberDefaultSet {
                name: "m_value".into(),
                default: PatchDefault::Int(42),
            })
            .with_component(PatchComponent::MemberAdded {
                name: "m_flag".into(),
                ty: LegacyType::Byte,
                default: PatchDefault::Bool(true),
            }),
    );

    let report = patches.apply(&mut bundle);
    assert_eq!(report.records_patched, 1);
    assert_eq!(report.fields_renamed, 1);
    assert_eq!(report.defaults_set, 1);
    assert_eq!(report.fields_added, 1);

    let Value::Record(record) = bundle.object(1).expect("root object") else {
        panic!("root object is not a record");
    };
    assert_eq!(record.fields.len(), 2);
    assert_eq!(record.fields[0], ("m_value".into(), Value::Int(42)));
    assert_eq!(record.fields[1], ("m_flag".into(), Value::Bool(true)));
}

#[test]
fn v3_format_sniffer_distinguishes_packfile_and_tagfile_headers() {
    let mut packfile = vec![0u8; 16];
    packfile[0..4].copy_from_slice(&0x57e0_e057u32.to_le_bytes());
    packfile[4..8].copy_from_slice(&0x10c0_c010u32.to_le_bytes());
    assert_eq!(
        detect_format(&packfile),
        SerializeFormat::BinaryPackfile2014
    );

    let tagfile = versioned_tagfile();
    assert_eq!(detect_format(&tagfile), SerializeFormat::BinaryTagfile);
    assert_eq!(detect_format(b"<no hkx>"), SerializeFormat::Unknown);
}
