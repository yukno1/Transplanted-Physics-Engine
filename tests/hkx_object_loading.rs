use proton::serialize::vle;
use proton::serialize::{Bundle, Value, four_cc, pack_scope};

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

fn minimal_tagfile() -> Vec<u8> {
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

    let type_section = branch(
        four_cc(b"TYPE"),
        &[
            leaf(four_cc(b"TST1"), b"Demo\0i32\0"),
            leaf(four_cc(b"FST1"), b"value\0"),
            leaf(four_cc(b"TNA1"), &identities),
            leaf(four_cc(b"TBDY"), &bodies),
        ],
    );

    let mut items = Vec::new();
    items.extend_from_slice(&0u32.to_be_bytes());
    items.extend_from_slice(&0u32.to_be_bytes());
    items.extend_from_slice(&0u32.to_be_bytes());
    items.extend_from_slice(&(0x1000_0000u32 | 1).to_be_bytes());
    items.extend_from_slice(&0u32.to_be_bytes());
    items.extend_from_slice(&1u32.to_be_bytes());
    let index_section = branch(four_cc(b"INDX"), &[leaf(four_cc(b"ITEM"), &items)]);

    branch(
        four_cc(b"TAG0"),
        &[
            leaf(four_cc(b"SDKV"), b"20211202"),
            leaf(four_cc(b"DATA"), &123i32.to_le_bytes()),
            type_section,
            index_section,
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
