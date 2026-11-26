//! End-to-end test: a synthetic `TAG0` hkx bundle is materialised by
//! `base-serialize`, converted into `phys-shape` / `phys-body` objects by the
//! facade `convert` module, and then simulated by `phys-world`.

use proton::convert::{ConvertError, body_cinfo_from_item, scene_from_bundle, shape_from_item};
use proton::math::{Vector4, Vec4, Vec4Ext};
use proton::np_types::MotionType;
use proton::serialize::vle;
use proton::serialize::{Bundle, Value, four_cc, pack_scope};
use proton::shape::{BoxShape, SphereShape};
use proton::world::WorldCinfo;

const T_BOX: u32 = 1;
const T_SPHERE: u32 = 2;
const T_BODY: u32 = 3;
const T_VEC4: u32 = 4;
const T_F32: u32 = 5;
const T_PTR: u32 = 6;
const T_I32: u32 = 7;

const FMT_F32: u32 = 5 | (1 << 9) | (1 << 10) | (8 << 11) | (23 << 16);
const FMT_I32: u32 = 4 | (1 << 9) | (32 << 10);
const FMT_VEC4: u32 = 8 | (1 << 5) | (4 << 8);
const FMT_POINTER: u32 = 6;
const FMT_RECORD: u32 = 7;

const OPT_FORMAT: u32 = 1 << 0;
const OPT_SUBTYPE: u32 = 1 << 1;
const OPT_SIZE_ALIGN: u32 = 1 << 3;
const OPT_DECLS: u32 = 1 << 5;

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

fn write_record_type(
    out: &mut Vec<u8>,
    id: u32,
    size: u32,
    align: u32,
    fields: &[(u32, u32, u32)],
) {
    vle::write(id as u64, out);
    vle::write(0, out);
    vle::write((OPT_FORMAT | OPT_SIZE_ALIGN | OPT_DECLS) as u64, out);
    vle::write(FMT_RECORD as u64, out);
    vle::write(size as u64, out);
    vle::write(align as u64, out);
    vle::write(fields.len() as u64, out);
    for &(name, offset, type_id) in fields {
        vle::write(name as u64, out);
        vle::write(0x20, out); // DECL_DATA_FIELD
        vle::write(offset as u64, out);
        vle::write(type_id as u64, out);
    }
}

fn write_leaf_type(
    out: &mut Vec<u8>,
    id: u32,
    format: u32,
    subtype: Option<u32>,
    size: u32,
    align: u32,
) {
    vle::write(id as u64, out);
    vle::write(0, out);
    let mut optbits = OPT_FORMAT | OPT_SIZE_ALIGN;
    if subtype.is_some() {
        optbits |= OPT_SUBTYPE;
    }
    vle::write(optbits as u64, out);
    vle::write(format as u64, out);
    if let Some(subtype) = subtype {
        vle::write(subtype as u64, out);
    }
    vle::write(size as u64, out);
    vle::write(align as u64, out);
}

fn type_section() -> Vec<u8> {
    let type_names = b"hknpBoxShape\0hknpSphereShape\0hknpBody\0vec4\0f32\0ptr\0i32\0";
    let field_names = b"m_halfExtents\0m_convexRadius\0m_center\0m_radius\0m_shape\0m_motionType\0m_position\0m_mass\0";

    let mut identities = Vec::new();
    vle::write(8, &mut identities); // null type + ids 1..7
    for tid in 1..=7 {
        vle::write(tid - 1, &mut identities);
        vle::write(0, &mut identities); // no template params
    }

    let mut bodies = Vec::new();
    // Box: halfExtents(vec4) @0, convexRadius(f32) @16.
    write_record_type(&mut bodies, T_BOX, 24, 4, &[(0, 0, T_VEC4), (1, 16, T_F32)]);
    // Sphere: center(vec4) @0, radius(f32) @16.
    write_record_type(
        &mut bodies,
        T_SPHERE,
        24,
        4,
        &[(2, 0, T_VEC4), (3, 16, T_F32)],
    );
    // Body: shape(ptr) @0, motionType(i32) @8, position(vec4) @12, mass(f32) @28.
    write_record_type(
        &mut bodies,
        T_BODY,
        32,
        8,
        &[
            (4, 0, T_PTR),
            (5, 8, T_I32),
            (6, 12, T_VEC4),
            (7, 28, T_F32),
        ],
    );
    write_leaf_type(&mut bodies, T_VEC4, FMT_VEC4, Some(T_F32), 16, 4);
    write_leaf_type(&mut bodies, T_F32, FMT_F32, None, 4, 4);
    write_leaf_type(&mut bodies, T_PTR, FMT_POINTER, Some(T_BOX), 8, 8);
    write_leaf_type(&mut bodies, T_I32, FMT_I32, None, 4, 4);

    branch(
        four_cc(b"TYPE"),
        &[
            leaf(four_cc(b"TST1"), type_names),
            leaf(four_cc(b"FST1"), field_names),
            leaf(four_cc(b"TNA1"), &identities),
            leaf(four_cc(b"TBDY"), &bodies),
        ],
    )
}

fn put_f32(data: &mut [u8], offset: usize, value: f32) {
    data[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put_i32(data: &mut [u8], offset: usize, value: i32) {
    data[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put_u32(data: &mut [u8], offset: usize, value: u32) {
    data[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn data_section() -> Vec<u8> {
    let mut data = vec![0u8; 112];
    // item 1: box shape @0 — halfExtents (10, 0.5, 10), convexRadius 0.
    for (index, value) in [10.0f32, 0.5, 10.0, 0.0].iter().enumerate() {
        put_f32(&mut data, index * 4, *value);
    }
    put_f32(&mut data, 16, 0.0);

    // item 2: sphere shape @24 — center origin, radius 0.5.
    for value in [0.0f32, 0.0, 0.0, 0.0] {
        // already zero, keeps intent explicit
        let _ = value;
    }
    put_f32(&mut data, 40, 0.5);

    // item 3: static ground body @48 — shape→item 1, static, y=-0.5, mass 0.
    put_u32(&mut data, 48, 1);
    put_i32(&mut data, 56, MotionType::Static as i32);
    for (index, value) in [0.0f32, -0.5, 0.0, 0.0].iter().enumerate() {
        put_f32(&mut data, 60 + index * 4, *value);
    }
    put_f32(&mut data, 76, 0.0);

    // item 4: dynamic ball body @80 — shape→item 2, dynamic, y=10, mass 1.
    put_u32(&mut data, 80, 2);
    put_i32(&mut data, 88, MotionType::Dynamic as i32);
    for (index, value) in [0.0f32, 10.0, 0.0, 0.0].iter().enumerate() {
        put_f32(&mut data, 92 + index * 4, *value);
    }
    put_f32(&mut data, 108, 1.0);
    data
}

fn index_section() -> Vec<u8> {
    let mut items = Vec::new();
    for (kind, tid, offset) in [
        (0u32, 0u32, 0u32),
        (0x1000_0000, T_BOX, 0),
        (0x1000_0000, T_SPHERE, 24),
        (0x1000_0000, T_BODY, 48),
        (0x1000_0000, T_BODY, 80),
    ] {
        items.extend_from_slice(&(kind | tid).to_be_bytes());
        items.extend_from_slice(&offset.to_be_bytes());
        items.extend_from_slice(&1u32.to_be_bytes());
    }
    // Two internal fixups: ground.m_shape → item 1, ball.m_shape → item 2.
    let patch = [T_PTR, 1, 48, T_PTR, 1, 80];
    let mut patch_bytes = Vec::new();
    for word in patch {
        patch_bytes.extend_from_slice(&word.to_be_bytes());
    }
    branch(
        four_cc(b"INDX"),
        &[
            leaf(four_cc(b"ITEM"), &items),
            leaf(four_cc(b"PTCH"), &patch_bytes),
        ],
    )
}

fn scene_tagfile() -> Vec<u8> {
    branch(
        four_cc(b"TAG0"),
        &[
            leaf(four_cc(b"SDKV"), b"20211202"),
            leaf(four_cc(b"DATA"), &data_section()),
            type_section(),
            index_section(),
        ],
    )
}

fn field<'a>(record: &'a proton::serialize::Record, name: &str) -> &'a Value {
    record
        .fields
        .iter()
        .find(|(field_name, _)| field_name == name)
        .map(|(_, value)| value)
        .unwrap_or_else(|| panic!("missing field {name}"))
}

#[test]
fn converts_shape_items_to_concrete_shapes() {
    let bundle = Bundle::load(&scene_tagfile()).expect("bundle");
    let box_shape = shape_from_item(&bundle, 1).expect("box");
    assert_eq!(box_shape.shape_type(), proton::np_types::ShapeType::Box);
    let box_shape = box_shape
        .as_any()
        .downcast_ref::<BoxShape>()
        .expect("downcast box");
    assert_eq!(box_shape.half_extents, Vec4::from_xyz(10.0, 0.5, 10.0));
    assert_eq!(box_shape.convex_radius, 0.0);

    let sphere = shape_from_item(&bundle, 2).expect("sphere");
    assert_eq!(sphere.shape_type(), proton::np_types::ShapeType::Sphere);
    let sphere = sphere
        .as_any()
        .downcast_ref::<SphereShape>()
        .expect("downcast sphere");
    assert_eq!(sphere.radius, 0.5);
    assert_eq!(sphere.center, Vec4::ZERO);
}

#[test]
fn converts_body_fields_and_reports_bad_items() {
    let bundle = Bundle::load(&scene_tagfile()).expect("bundle");
    let cinfo = body_cinfo_from_item(&bundle, 4).expect("ball body");
    assert_eq!(cinfo.motion_type, MotionType::Dynamic);
    assert_eq!(cinfo.mass, 1.0);
    assert_eq!(cinfo.position, Vector4::from_xyz(0.0, 10.0, 0.0));

    assert!(matches!(
        shape_from_item(&bundle, 99),
        Err(ConvertError::ItemOutOfRange(99))
    ));
    // item 0 is an Empty item (materialised as Void), not a record.
    assert!(matches!(
        body_cinfo_from_item(&bundle, 0),
        Err(ConvertError::NotARecord { item: 0 })
    ));
}

#[test]
fn loads_bundle_into_world_and_simulates_fall() {
    let bundle = Bundle::load(&scene_tagfile()).expect("bundle");
    let scene = scene_from_bundle(&bundle).expect("scene");
    assert_eq!(scene.bodies.len(), 2);
    assert_eq!(scene.bodies[0].motion_type, MotionType::Static);
    assert_eq!(scene.bodies[1].motion_type, MotionType::Dynamic);

    let mut world = proton::world::World::new(WorldCinfo::new());
    let ids = scene.populate(&mut world);
    assert_eq!(ids.len(), 2);
    assert_eq!(world.body_count(), 2);
    assert_eq!(
        world.body_position(ids[1]),
        Some(Vector4::from_xyz(0.0, 10.0, 0.0))
    );

    for _ in 0..240 {
        world.step(1.0 / 60.0);
    }
    let y = world.body_position(ids[1]).expect("ball alive").y;
    // Ball starts at y=10, ground top is y=0, ball radius 0.5 → rest near 0.5.
    assert!(
        (0.35..=1.5).contains(&y),
        "ball did not settle on ground: y={y}"
    );
    assert_eq!(
        world.body_position(ids[0]),
        Some(Vector4::from_xyz(0.0, -0.5, 0.0))
    );
}

#[test]
fn conversion_is_deterministic_across_loads() {
    let first = {
        let bundle = Bundle::load(&scene_tagfile()).expect("bundle");
        let mut world = proton::world::World::new(WorldCinfo::new());
        let ids = scene_from_bundle(&bundle)
            .expect("scene")
            .populate(&mut world);
        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }
        let position = world.body_position(ids[1]).unwrap();
        [
            position.x.to_bits(),
            position.y.to_bits(),
            position.z.to_bits(),
            position.w.to_bits(),
        ]
    };
    let second = {
        let bundle = Bundle::load(&scene_tagfile()).expect("bundle");
        let mut world = proton::world::World::new(WorldCinfo::new());
        let ids = scene_from_bundle(&bundle)
            .expect("scene")
            .populate(&mut world);
        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }
        let position = world.body_position(ids[1]).unwrap();
        [
            position.x.to_bits(),
            position.y.to_bits(),
            position.z.to_bits(),
            position.w.to_bits(),
        ]
    };
    assert_eq!(first, second);
}

#[test]
fn bundle_type_name_drives_shape_family() {
    let bundle = Bundle::load(&scene_tagfile()).expect("bundle");
    let Value::Record(record) = bundle.object(1).expect("box item") else {
        panic!("not a record");
    };
    assert_eq!(record.type_id, T_BOX);
    assert!(matches!(field(record, "m_halfExtents"), Value::Array(_)));
    let name = bundle
        .type_info
        .types
        .get(record.type_id as usize)
        .and_then(|meta| meta.as_ref())
        .and_then(|meta| meta.name.as_deref())
        .expect("type name");
    assert_eq!(name, "hknpBoxShape");
}
