use proton::serialize::vle;
use proton::serialize::{Bundle, Bundle2014, Compendium, Value, Value2014, four_cc, pack_scope};

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

fn minimal_type_section() -> Vec<u8> {
    let mut identities = Vec::new();
    vle::write(3, &mut identities);
    for name in [0, 1] {
        vle::write(name, &mut identities);
        vle::write(0, &mut identities);
    }

    let mut bodies = Vec::new();
    // id 1: Demo record, one int32 field.
    vle::write(1, &mut bodies);
    vle::write(0, &mut bodies);
    vle::write((1 << 0) | (1 << 3) | (1 << 5), &mut bodies);
    vle::write(7, &mut bodies); // KIND_RECORD
    vle::write(4, &mut bodies);
    vle::write(4, &mut bodies);
    vle::write(1, &mut bodies); // one field
    vle::write(0, &mut bodies); // "value"
    vle::write(0x20, &mut bodies); // DECL_DATA_FIELD
    vle::write(0, &mut bodies);
    vle::write(2, &mut bodies);
    // id 2: signed int32.
    vle::write(2, &mut bodies);
    vle::write(0, &mut bodies);
    vle::write((1 << 0) | (1 << 3), &mut bodies);
    vle::write(4 | (1 << 9) | (32 << 10), &mut bodies);
    vle::write(4, &mut bodies);
    vle::write(4, &mut bodies);
    bodies.push(0);

    branch(
        four_cc(b"TYPE"),
        &[
            leaf(four_cc(b"TST1"), b"Demo\0i32\0"),
            leaf(four_cc(b"FST1"), b"value\0"),
            leaf(four_cc(b"TNA1"), &identities),
            leaf(four_cc(b"TBDY"), &bodies),
        ],
    )
}

fn minimal_index_section() -> Vec<u8> {
    let mut items = Vec::new();
    items.extend_from_slice(&0u32.to_be_bytes());
    items.extend_from_slice(&0u32.to_be_bytes());
    items.extend_from_slice(&0u32.to_be_bytes());
    items.extend_from_slice(&(0x1000_0000u32 | 1).to_be_bytes());
    items.extend_from_slice(&0u32.to_be_bytes());
    items.extend_from_slice(&1u32.to_be_bytes());
    branch(four_cc(b"INDX"), &[leaf(four_cc(b"ITEM"), &items)])
}

fn minimal_tagfile() -> Vec<u8> {
    branch(
        four_cc(b"TAG0"),
        &[
            leaf(four_cc(b"SDKV"), b"20211202"),
            leaf(four_cc(b"DATA"), &123i32.to_le_bytes()),
            minimal_type_section(),
            minimal_index_section(),
        ],
    )
}

#[test]
fn facade_loads_generic_hkx_object() {
    let bundle = Bundle::load(&minimal_tagfile()).expect("bundle");
    let Value::Record(record) = bundle.object(1).expect("root object") else {
        panic!("root object is not a record");
    };
    assert_eq!(record.fields.len(), 1);
    assert_eq!(record.fields[0].0, "value");
    assert_eq!(record.fields[0].1, Value::Int(123));
}

#[test]
fn facade_rejects_non_tagfile_input() {
    let bytes = branch(four_cc(b"NOPE"), &[]);
    assert!(Bundle::load(&bytes).is_err());
}

#[test]
fn facade_loads_bundle_with_compendium_types() {
    let signature = 0x0123_4567_89ab_cdefu64;
    let compendium = Compendium::load(&branch(
        four_cc(b"TCM0"),
        &[
            leaf(four_cc(b"TCID"), &signature.to_le_bytes()),
            minimal_type_section(),
        ],
    ))
    .expect("compendium");

    let bundle_bytes = branch(
        four_cc(b"TAG0"),
        &[
            leaf(four_cc(b"SDKV"), b"20211202"),
            leaf(four_cc(b"DATA"), &123i32.to_le_bytes()),
            leaf(four_cc(b"TCRF"), &signature.to_le_bytes()),
            minimal_index_section(),
        ],
    );
    assert!(Bundle::load(&bundle_bytes).is_err());

    let bundle = Bundle::load_with_compendia(&bundle_bytes, &[compendium]).expect("bundle");
    let Value::Record(record) = bundle.object(1).expect("root object") else {
        panic!("root object is not a record");
    };
    assert_eq!(record.fields[0].0, "value");
    assert_eq!(record.fields[0].1, Value::Int(123));
}

fn put_varint(mut value: u64, negative: bool, out: &mut Vec<u8>) {
    let first = ((value & 0x3f) << 1) | u64::from(negative);
    value >>= 6;
    if value == 0 {
        out.push(first as u8);
        return;
    }
    out.push((first | 0x80) as u8);
    while value != 0 {
        let byte = value & 0x7f;
        value >>= 7;
        out.push((byte | if value == 0 { 0 } else { 0x80 }) as u8);
    }
}

fn put_int(value: i64, out: &mut Vec<u8>) {
    put_varint(value.unsigned_abs(), value < 0, out);
}

fn put_legacy_string(text: &str, out: &mut Vec<u8>) {
    put_int(text.len() as i64, out);
    out.extend_from_slice(text.as_bytes());
}

#[test]
fn facade_loads_legacy_binary_tagfile() {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&0xCAB0_0D1Eu32.to_le_bytes());
    bytes.extend_from_slice(&0xD011_FACEu32.to_le_bytes());
    put_int(1, &mut bytes); // TAG_FILE_INFO
    put_int(2, &mut bytes); // version
    put_int(2, &mut bytes); // TAG_METADATA
    put_legacy_string("Demo", &mut bytes);
    put_int(1, &mut bytes); // class version
    put_int(0, &mut bytes); // parent
    put_int(1, &mut bytes); // one field
    put_legacy_string("value", &mut bytes);
    put_int(2, &mut bytes); // TYPE_INT
    put_int(3, &mut bytes); // TAG_OBJECT
    put_int(1, &mut bytes); // class index
    bytes.push(0b1); // field present
    put_int(77, &mut bytes);
    put_int(7, &mut bytes); // TAG_FILE_END

    let bundle = Bundle2014::load(&bytes).expect("legacy bundle");
    assert_eq!(bundle.version, 2);
    let Value2014::Record(record) = &bundle.objects[0] else {
        panic!("root object is not a record");
    };
    assert_eq!(record.fields[0].0, "value");
    assert_eq!(record.fields[0].1, Value2014::Int(77));
}
