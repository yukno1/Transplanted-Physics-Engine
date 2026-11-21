//! Integration coverage for packfile routing and extended version patches.

use proton::serialize::{
    BINARY_PACKFILE_MAGIC_0, BINARY_PACKFILE_MAGIC_1, CURRENT_FILE_VERSION, LoadOptions, LoadedHkx,
    PackfileConverter, PackfileError, PatchComponent, PatchError, PatchFunctionHelper, PatchInfo,
    PatchSet, Record, Value, four_cc, load_hkx, pack_scope, vle,
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

/// `Demo` record with declared TYPE version 1 and one `value: i32` field.
fn versioned_tagfile() -> Vec<u8> {
    let mut identities = Vec::new();
    vle::write(3, &mut identities);
    vle::write(0, &mut identities);
    vle::write(0, &mut identities);
    vle::write(1, &mut identities);
    vle::write(0, &mut identities);

    let mut bodies = Vec::new();
    vle::write(1, &mut bodies);
    vle::write(0, &mut bodies);
    vle::write((1 << 0) | (1 << 2) | (1 << 3) | (1 << 5), &mut bodies);
    vle::write(7, &mut bodies);
    vle::write(1, &mut bodies);
    vle::write(4, &mut bodies);
    vle::write(4, &mut bodies);
    vle::write(1, &mut bodies);
    vle::write(0, &mut bodies);
    vle::write(0x20, &mut bodies);
    vle::write(0, &mut bodies);
    vle::write(2, &mut bodies);
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

fn packfile() -> Vec<u8> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&BINARY_PACKFILE_MAGIC_0.to_le_bytes());
    bytes.extend_from_slice(&BINARY_PACKFILE_MAGIC_1.to_le_bytes());
    bytes.extend_from_slice(&7i32.to_le_bytes());
    bytes.extend_from_slice(&CURRENT_FILE_VERSION.to_le_bytes());
    bytes.extend_from_slice(b"packfile body is converted externally");
    bytes
}

struct StaticConverter(Vec<u8>);

impl PackfileConverter for StaticConverter {
    fn convert(
        &self,
        _header: &proton::serialize::PackfileHeader,
        _bytes: &[u8],
    ) -> Result<Vec<u8>, PackfileError> {
        Ok(self.0.clone())
    }
}

fn demo_record(loaded: &LoadedHkx) -> &Record {
    let Value::Record(record) = loaded
        .bundle()
        .expect("generic bundle")
        .object(1)
        .expect("Demo object")
    else {
        panic!("Demo object is not a record");
    };
    record
}

fn add_flag(record: &mut Record, _helper: &PatchFunctionHelper) {
    record
        .fields
        .push(("m_flag".to_string(), Value::Bool(true)));
}

fn double_value(value: &mut Value, _helper: &PatchFunctionHelper) {
    if let Value::Int(number) = value {
        *number *= 2;
    }
}

fn int_to_text(value: &mut Value, old_type: &str, new_type: &str, helper: &PatchFunctionHelper) {
    assert_eq!(old_type, "int");
    assert_eq!(new_type, "string");
    if let Value::Int(number) = value {
        *value = helper.new_string(number.to_string());
    }
}

#[test]
fn packfile_converter_routes_into_existing_tagfile_reader() {
    let converter = StaticConverter(versioned_tagfile());
    let options = LoadOptions {
        packfile_converter: Some(&converter),
        ..LoadOptions::default()
    };
    let loaded = load_hkx(&packfile(), &options).expect("converted packfile");
    let record = demo_record(&loaded);
    assert_eq!(record.fields[0], ("value".to_string(), Value::Int(123)));
}

#[test]
fn packfile_without_converter_reports_explicit_error() {
    let error = load_hkx(&packfile(), &LoadOptions::default()).expect_err("missing converter");
    assert_eq!(
        error,
        proton::serialize::LoadHkxError::Packfile(PackfileError::ConverterMissing)
    );
}

#[test]
fn function_patches_mutate_record_and_field() {
    let mut patches = PatchSet::new();
    patches.add(
        PatchInfo::new("Demo", "Demo", 1, 2)
            .with_component(PatchComponent::DataObjectFunction {
                name: "add_flag".to_string(),
                function: add_flag,
            })
            .with_component(PatchComponent::VarFunction {
                name: "double_value".to_string(),
                field: "value".to_string(),
                function: double_value,
            }),
    );
    let options = LoadOptions {
        patches: Some(&patches),
        ..LoadOptions::default()
    };
    let loaded = load_hkx(&versioned_tagfile(), &options).expect("patched bundle");
    let record = demo_record(&loaded);
    assert_eq!(record.fields[0], ("value".to_string(), Value::Int(246)));
    assert_eq!(record.fields[1], ("m_flag".to_string(), Value::Bool(true)));
}

#[test]
fn type_change_patch_converts_legacy_field() {
    let mut patches = PatchSet::new();
    patches.add(PatchInfo::new("Demo", "Demo", 1, 2).with_component(
        PatchComponent::VarTypeChanged {
            name: "value".to_string(),
            old_type: "int".to_string(),
            new_type: "string".to_string(),
            function: int_to_text,
        },
    ));
    let options = LoadOptions {
        patches: Some(&patches),
        ..LoadOptions::default()
    };
    let loaded = load_hkx(&versioned_tagfile(), &options).expect("patched bundle");
    assert_eq!(
        demo_record(&loaded).fields[0],
        ("value".to_string(), Value::String("123".to_string()))
    );
}

#[test]
fn version_chain_updates_metadata_and_applies_multiple_steps() {
    let mut patches = PatchSet::new();
    patches.add(PatchInfo::new("Demo", "DemoV2", 1, 2).with_component(
        PatchComponent::MemberRenamed {
            old: "value".to_string(),
            new: "m_value".to_string(),
        },
    ));
    patches.add(PatchInfo::new("DemoV2", "DemoV3", 2, 3).with_component(
        PatchComponent::MemberAdded {
            name: "m_flag".to_string(),
            ty: proton::serialize::LegacyType::Byte,
            default: proton::serialize::PatchDefault::Bool(true),
        },
    ));
    assert!(patches.conflicts().is_empty());

    let options = LoadOptions {
        patches: Some(&patches),
        ..LoadOptions::default()
    };
    let loaded = load_hkx(&versioned_tagfile(), &options).expect("chained bundle");
    let record = demo_record(&loaded);
    assert_eq!(record.fields[0], ("m_value".to_string(), Value::Int(123)));
    assert_eq!(record.fields[1], ("m_flag".to_string(), Value::Bool(true)));
}

#[test]
fn duplicate_and_cyclic_patches_are_reported() {
    let mut duplicate = PatchSet::new();
    duplicate.add(PatchInfo::new("Demo", "DemoV2", 1, 2));
    duplicate.add(PatchInfo::new("Demo", "DemoV3", 1, 3));
    assert_eq!(duplicate.conflicts(), vec![("Demo".to_string(), 1)]);
    let bytes = versioned_tagfile();
    let mut bundle = proton::serialize::Bundle::load(&bytes).expect("bundle");
    assert_eq!(
        duplicate.apply_chain(&mut bundle),
        Err(PatchError::DuplicatePatch {
            type_name: "Demo".to_string(),
            version: 1,
        })
    );

    let mut cycle = PatchSet::new();
    cycle.add(PatchInfo::new("Demo", "Demo", 1, 1));
    let mut bundle = proton::serialize::Bundle::load(&bytes).expect("bundle");
    assert_eq!(
        cycle.apply_chain(&mut bundle),
        Err(PatchError::PatchCycle {
            type_name: "Demo".to_string(),
            version: 1,
        })
    );
}

#[test]
fn patched_loads_are_deterministic() {
    let mut patches = PatchSet::new();
    patches.add(
        PatchInfo::new("Demo", "Demo", 1, 2).with_component(PatchComponent::VarFunction {
            name: "double_value".to_string(),
            field: "value".to_string(),
            function: double_value,
        }),
    );
    let options = LoadOptions {
        patches: Some(&patches),
        ..LoadOptions::default()
    };
    let first = load_hkx(&versioned_tagfile(), &options).expect("first load");
    let second = load_hkx(&versioned_tagfile(), &options).expect("second load");
    assert_eq!(first, second);
}
