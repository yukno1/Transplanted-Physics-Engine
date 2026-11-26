//! hkx generic value → physics object conversion.
//!
//! `base-serialize` deliberately does not depend on any `phys-*` crate: it
//! materialises a `TAG0` tagfile into a generic [`Value`] tree.  This module
//! lives in the facade crate and bridges that tree to `phys-shape` /
//! `phys-body` / `phys-world`, keeping the dependency direction strictly
//! one-way (`serialize → convert → shape/body/world`).
//!
//! # Field-name convention
//!
//! Havok tagfiles name record fields with the `m_` prefix (e.g. `m_radius`,
//! `m_halfExtents`).  Every field lookup below accepts both the prefixed and
//! the bare spelling, so bundles produced by hand-written writers work as
//! well.  Shape identity comes from the `TypeMeta.name` attached to the
//! record's [`TypeId`]; matching is by substring on that name (e.g.
//! `hknpSphereShape` → sphere) followed by the relevant fields.
//!
//! This is a *generic bundle* conversion: it does not claim bit-exact
//! compatibility with real Havok assets (no version patching, no compendium,
//! no vtable fixups — see the README Phase 16 notes).  It makes the `Value`
//! tree produced by Phase 16 actionable for simulation.

use std::sync::Arc;

use glam::{Quat, Vec4};

use crate::body::BodyCinfo;
use crate::math::Transform;
use crate::np_types::{BodyId, MotionType};
use crate::serialize::{Bundle, Record, TypeId, Value};
use crate::shape::{
    BoxShape, CapsuleShape, CompoundShape, ConvexHullShape, CylinderShape, HeightFieldShape,
    MeshShape, Shape, ShapeInstance, SphereShape,
};
use crate::world::{World, WorldCinfo};

use crate::base_math::Vec4Ext;
use crate::shape::{to_quaternion, to_vector4};

/// Errors raised while converting a generic `Value` tree to physics objects.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConvertError {
    /// The referenced item index is out of range for the bundle.
    ItemOutOfRange(u32),
    /// The item exists but is not a materialised record.
    NotARecord { item: u32 },
    /// A record's type name does not match any supported shape/body family.
    UnknownType {
        type_id: TypeId,
        name: Option<String>,
    },
    /// A required field is absent from the record.
    MissingField(&'static str),
    /// A field held a value that cannot be interpreted as a float.
    ExpectedFloat(&'static str),
    /// A field held a value that cannot be interpreted as an integer.
    ExpectedInt(&'static str),
    /// A field held a value that cannot be interpreted as an array.
    ExpectedArray(&'static str),
    /// An array element had the wrong number of components.
    InvalidArray(&'static str),
    /// A field held a value that cannot be interpreted as a record.
    ExpectedRecord(&'static str),
    /// A vector field did not contain three/four finite components.
    InvalidVector(&'static str),
    /// An enum field carried an unrecognised raw value.
    InvalidEnum { field: &'static str, value: i128 },
    /// A shape could not be constructed from otherwise well-formed fields.
    InvalidShape(String),
    /// A body could not be constructed from otherwise well-formed fields.
    InvalidBody(String),
}

impl core::fmt::Display for ConvertError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::ItemOutOfRange(item) => write!(f, "item {item} out of range"),
            Self::NotARecord { item } => write!(f, "item {item} is not a record"),
            Self::UnknownType { type_id, name } => {
                write!(
                    f,
                    "unknown type {type_id} ({})",
                    name.as_deref().unwrap_or("<unnamed>")
                )
            }
            Self::MissingField(name) => write!(f, "missing field {name}"),
            Self::ExpectedFloat(name) => write!(f, "field {name} is not a float"),
            Self::ExpectedInt(name) => write!(f, "field {name} is not an integer"),
            Self::ExpectedArray(name) => write!(f, "field {name} is not an array"),
            Self::InvalidArray(name) => write!(f, "field {name} has malformed array elements"),
            Self::ExpectedRecord(name) => write!(f, "field {name} is not a record"),
            Self::InvalidVector(name) => write!(f, "field {name} is not a valid 3d/4d vector"),
            Self::InvalidEnum { field, value } => {
                write!(f, "invalid raw value {value} for enum field {field}")
            }
            Self::InvalidShape(msg) => write!(f, "invalid shape: {msg}"),
            Self::InvalidBody(msg) => write!(f, "invalid body: {msg}"),
        }
    }
}

impl std::error::Error for ConvertError {}

/// A scene materialised from a bundle: one [`BodyCinfo`] per recognised body.
#[derive(Clone, Debug)]
pub struct LoadedScene {
    /// Body descriptions in bundle item order.
    pub bodies: Vec<BodyCinfo>,
}

/// Look up a field by its Havok-style name, accepting the `m_` prefix or the
/// bare spelling.  Returns the first match in declaration order.
fn field<'a>(record: &'a Record, names: &[&str]) -> Option<&'a Value> {
    for name in names {
        if let Some((_, value)) = record
            .fields
            .iter()
            .find(|(field_name, _)| field_name == name)
        {
            return Some(value);
        }
    }
    None
}

/// Type name of a record, looked up through the bundle's reflection metadata.
fn type_name(bundle: &Bundle, type_id: TypeId) -> Option<String> {
    bundle
        .type_info
        .types
        .get(type_id as usize)
        .and_then(|meta| meta.as_ref())
        .and_then(|meta| meta.name.clone())
}

fn as_float(value: &Value, name: &'static str) -> Result<f32, ConvertError> {
    match value {
        Value::Float(v) => Ok(*v as f32),
        Value::Int(v) => Ok(*v as f32),
        _ => Err(ConvertError::ExpectedFloat(name)),
    }
}

fn as_int(value: &Value, name: &'static str) -> Result<i128, ConvertError> {
    match value {
        Value::Int(v) => Ok(*v),
        Value::Float(v) => Ok(*v as i128),
        _ => Err(ConvertError::ExpectedInt(name)),
    }
}

fn as_floats(value: &Value, name: &'static str) -> Result<Vec<f32>, ConvertError> {
    let Value::Array(items) = value else {
        return Err(ConvertError::ExpectedArray(name));
    };
    items.iter().map(|item| as_float(item, name)).collect()
}

/// Read a 3d/4d vector from either a fixed float array (`[x,y,z,w]`) or a
/// record with `x`/`y`/`z` (`m_x`/`m_y`/`m_z`) fields.  The fourth component
/// defaults to zero.
fn as_vector(value: &Value, name: &'static str) -> Result<Vec4, ConvertError> {
    match value {
        Value::Array(_) => {
            let values = as_floats(value, name)?;
            if values.len() < 3 {
                return Err(ConvertError::InvalidVector(name));
            }
            Ok(Vec4::from_xyz(values[0], values[1], values[2]))
        }
        Value::Record(_) => {
            let Value::Record(record) = value else {
                unreachable!()
            };
            let component = |names: &[&str]| -> Result<f32, ConvertError> {
                let value = field(record, names).ok_or(ConvertError::MissingField("x/y/z"))?;
                as_float(value, "x/y/z")
            };
            let x = component(&["m_x", "x"])?;
            let y = component(&["m_y", "y"])?;
            let z = component(&["m_z", "z"])?;
            Ok(Vec4::from_xyz(x, y, z))
        }
        _ => Err(ConvertError::InvalidVector(name)),
    }
}

/// Read a quaternion from a float array (`[x,y,z,w]`) or a record with
/// `x`/`y`/`z`/`w` fields.
fn as_quaternion(value: &Value, name: &'static str) -> Result<Quat, ConvertError> {
    let (x, y, z, w) = match value {
        Value::Array(_) => {
            let values = as_floats(value, name)?;
            if values.len() != 4 {
                return Err(ConvertError::InvalidVector(name));
            }
            (values[0], values[1], values[2], values[3])
        }
        Value::Record(_) => {
            let Value::Record(record) = value else {
                unreachable!()
            };
            let component = |names: &[&str]| -> Result<f32, ConvertError> {
                let value = field(record, names).ok_or(ConvertError::MissingField("x/y/z/w"))?;
                as_float(value, "x/y/z/w")
            };
            (
                component(&["m_x", "x"])?,
                component(&["m_y", "y"])?,
                component(&["m_z", "z"])?,
                component(&["m_w", "w"])?,
            )
        }
        _ => return Err(ConvertError::InvalidVector(name)),
    };
    Ok(Quat::from_xyzw(x, y, z, w))
}

/// Read a quaternion + translation pair as a [`Transform`].
fn as_transform(value: &Value, name: &'static str) -> Result<Transform, ConvertError> {
    let Value::Record(record) = value else {
        return Err(ConvertError::ExpectedRecord(name));
    };
    let rotation_value = field(record, &["m_rotation", "rotation"])
        .ok_or(ConvertError::MissingField("m_rotation"))?;
    let translation_value = field(record, &["m_translation", "translation"])
        .ok_or(ConvertError::MissingField("m_translation"))?;
    Ok(Transform::from_quaternion(
        to_quaternion(as_quaternion(rotation_value, "m_rotation")?),
        to_vector4(as_vector(translation_value, "m_translation")?),
    ))
}

/// Read an array of triangle vertex indices (`[[a,b,c], ...]`).
fn as_triangles(value: &Value, name: &'static str) -> Result<Vec<[u32; 3]>, ConvertError> {
    let Value::Array(items) = value else {
        return Err(ConvertError::ExpectedArray(name));
    };
    items
        .iter()
        .map(|item| {
            let values = as_floats(item, name)?;
            if values.len() != 3 {
                return Err(ConvertError::InvalidArray(name));
            }
            let mut triangle = [0u32; 3];
            for (slot, value) in triangle.iter_mut().zip(values) {
                *slot = value as u32;
            }
            Ok(triangle)
        })
        .collect()
}

/// Build a shape from a record value and its resolved type name.
fn shape_from_record(
    bundle: &Bundle,
    record: &Record,
    type_name: &str,
) -> Result<Arc<dyn Shape>, ConvertError> {
    let shape: Arc<dyn Shape> = if type_name.contains("Sphere") {
        let center = field(record, &["m_center", "center"])
            .map(|value| as_vector(value, "m_center"))
            .transpose()?
            .unwrap_or(Vec4::ZERO);
        let radius =
            field(record, &["m_radius", "radius"]).ok_or(ConvertError::MissingField("m_radius"))?;
        Arc::new(SphereShape::new(center, as_float(radius, "m_radius")?))
    } else if type_name.contains("Box") {
        let half_extents = field(record, &["m_halfExtents", "half_extents"])
            .ok_or(ConvertError::MissingField("m_halfExtents"))?;
        let convex_radius = field(record, &["m_convexRadius", "convex_radius"])
            .map(|value| as_float(value, "m_convexRadius"))
            .transpose()?
            .unwrap_or(crate::shape::DEFAULT_CONVEX_RADIUS);
        Arc::new(BoxShape::with_convex_radius(
            as_vector(half_extents, "m_halfExtents")?,
            convex_radius,
        ))
    } else if type_name.contains("Capsule") {
        let a = field(record, &["m_vertexA", "m_a", "vertex_a"])
            .ok_or(ConvertError::MissingField("m_vertexA"))?;
        let b = field(record, &["m_vertexB", "m_b", "vertex_b"])
            .ok_or(ConvertError::MissingField("m_vertexB"))?;
        let radius =
            field(record, &["m_radius", "radius"]).ok_or(ConvertError::MissingField("m_radius"))?;
        Arc::new(CapsuleShape::new(
            as_vector(a, "m_vertexA")?,
            as_vector(b, "m_vertexB")?,
            as_float(radius, "m_radius")?,
        ))
    } else if type_name.contains("Cylinder") {
        let a = field(record, &["m_vertexA", "m_a", "vertex_a"])
            .ok_or(ConvertError::MissingField("m_vertexA"))?;
        let b = field(record, &["m_vertexB", "m_b", "vertex_b"])
            .ok_or(ConvertError::MissingField("m_vertexB"))?;
        let radius =
            field(record, &["m_radius", "radius"]).ok_or(ConvertError::MissingField("m_radius"))?;
        let convex_radius = field(record, &["m_convexRadius", "convex_radius"])
            .map(|value| as_float(value, "m_convexRadius"))
            .transpose()?
            .unwrap_or(crate::shape::DEFAULT_CONVEX_RADIUS);
        let num_sides = field(record, &["m_numSides", "num_sides"])
            .map(|value| as_int(value, "m_numSides"))
            .transpose()?
            .unwrap_or(16) as u32;
        Arc::new(CylinderShape::with_params(
            as_vector(a, "m_vertexA")?,
            as_vector(b, "m_vertexB")?,
            as_float(radius, "m_radius")?,
            num_sides,
            convex_radius,
        ))
    } else if type_name.contains("ConvexPolytope") || type_name.contains("ConvexHull") {
        let vertices = field(record, &["m_vertices", "vertices"])
            .ok_or(ConvertError::MissingField("m_vertices"))?;
        let Value::Array(vertex_values) = vertices else {
            return Err(ConvertError::ExpectedArray("m_vertices"));
        };
        let vertices = vertex_values
            .iter()
            .map(|value| as_vector(value, "m_vertices"))
            .collect::<Result<Vec<_>, _>>()?;
        let faces = field(record, &["m_faces", "faces"])
            .map(|value| as_triangles(value, "m_faces"))
            .transpose()?
            .unwrap_or_default();
        let convex_radius = field(record, &["m_convexRadius", "convex_radius"])
            .map(|value| as_float(value, "m_convexRadius"))
            .transpose()?
            .unwrap_or(crate::shape::DEFAULT_CONVEX_RADIUS);
        Arc::new(ConvexHullShape::new(vertices, faces, convex_radius))
    } else if type_name.contains("MeshShape") {
        let vertices = field(record, &["m_vertices", "vertices"])
            .ok_or(ConvertError::MissingField("m_vertices"))?;
        let Value::Array(vertex_values) = vertices else {
            return Err(ConvertError::ExpectedArray("m_vertices"));
        };
        let vertices = vertex_values
            .iter()
            .map(|value| as_vector(value, "m_vertices"))
            .collect::<Result<Vec<_>, _>>()?;
        let triangles = field(record, &["m_triangles", "triangles"])
            .ok_or(ConvertError::MissingField("m_triangles"))?;
        Arc::new(MeshShape::new(
            vertices,
            as_triangles(triangles, "m_triangles")?,
        ))
    } else if type_name.contains("HeightField") {
        let size_x = field(record, &["m_numElemsX", "size_x"])
            .ok_or(ConvertError::MissingField("m_numElemsX"))?;
        let size_z = field(record, &["m_numElemsZ", "size_z"])
            .ok_or(ConvertError::MissingField("m_numElemsZ"))?;
        let scale = field(record, &["m_scale", "scale"])
            .map(|value| as_vector(value, "m_scale"))
            .transpose()?
            .unwrap_or(Vec4::from_xyz(1.0, 1.0, 1.0));
        let heights = field(record, &["m_heights", "heights"])
            .ok_or(ConvertError::MissingField("m_heights"))?;
        let heights = as_floats(heights, "m_heights")?;
        Arc::new(
            HeightFieldShape::new(
                as_int(size_x, "m_numElemsX")? as usize,
                as_int(size_z, "m_numElemsZ")? as usize,
                scale,
                heights,
            )
            .map_err(|error| ConvertError::InvalidShape(error.to_string()))?,
        )
    } else if type_name.contains("CompoundShape") {
        let children = field(record, &["m_children", "children"])
            .ok_or(ConvertError::MissingField("m_children"))?;
        let Value::Array(child_values) = children else {
            return Err(ConvertError::ExpectedArray("m_children"));
        };
        let mut instances = Vec::with_capacity(child_values.len());
        for child in child_values {
            let Value::Record(child_record) = child else {
                return Err(ConvertError::ExpectedRecord("m_children[]"));
            };
            let transform_value = field(
                child_record,
                &["m_transform", "m_localTransform", "transform"],
            )
            .ok_or(ConvertError::MissingField("m_transform"))?;
            let transform = as_transform(transform_value, "m_transform")?;
            let shape_value = field(child_record, &["m_shape", "shape"])
                .ok_or(ConvertError::MissingField("m_shape"))?;
            let shape = shape_from_value(bundle, shape_value)?;
            instances.push(ShapeInstance::new(transform, shape));
        }
        Arc::new(CompoundShape::new(instances))
    } else {
        return Err(ConvertError::UnknownType {
            type_id: record.type_id,
            name: Some(type_name.to_owned()),
        });
    };
    Ok(shape)
}

/// Resolve a shape value: an inlined record or a `Ref` to another bundle item.
fn shape_from_value(bundle: &Bundle, value: &Value) -> Result<Arc<dyn Shape>, ConvertError> {
    match value {
        Value::Record(_) => shape_from_record_value(bundle, value),
        Value::Ref(item) => shape_from_item(bundle, *item),
        _ => Err(ConvertError::ExpectedRecord("m_shape")),
    }
}

fn shape_from_record_value(bundle: &Bundle, value: &Value) -> Result<Arc<dyn Shape>, ConvertError> {
    let Value::Record(record) = value else {
        return Err(ConvertError::ExpectedRecord("shape"));
    };
    let name = type_name(bundle, record.type_id).unwrap_or_default();
    shape_from_record(bundle, record, &name)
}

/// Build a shape from a bundle item index.  The item must be a record whose
/// type name contains one of the supported shape families.
pub fn shape_from_item(bundle: &Bundle, item: u32) -> Result<Arc<dyn Shape>, ConvertError> {
    let value = bundle
        .object(item)
        .ok_or(ConvertError::ItemOutOfRange(item))?;
    shape_from_record_value(bundle, value)
}

/// Build a [`BodyCinfo`] from a body record, resolving its `m_shape` reference
/// and reading motion type / transform / mass / filter info / user data.
pub fn body_cinfo_from_item(bundle: &Bundle, item: u32) -> Result<BodyCinfo, ConvertError> {
    let value = bundle
        .object(item)
        .ok_or(ConvertError::ItemOutOfRange(item))?;
    let Value::Record(record) = value else {
        return Err(ConvertError::NotARecord { item });
    };
    let shape_value =
        field(record, &["m_shape", "shape"]).ok_or(ConvertError::MissingField("m_shape"))?;
    let shape = shape_from_value(bundle, shape_value)?;
    let mut cinfo = BodyCinfo::new(shape);

    if let Some(motion_type) = field(record, &["m_motionType", "motion_type"]) {
        let raw = as_int(motion_type, "m_motionType")? as u32;
        let motion_type = MotionType::from_raw(raw).ok_or(ConvertError::InvalidEnum {
            field: "m_motionType",
            value: raw as i128,
        })?;
        cinfo = cinfo.with_motion_type(motion_type);
    }
    if let Some(position) = field(record, &["m_position", "position"]) {
        cinfo = cinfo.with_position(as_vector(position, "m_position")?);
    }
    if let Some(orientation) = field(record, &["m_orientation", "orientation"]) {
        cinfo = cinfo.with_orientation(as_quaternion(orientation, "m_orientation")?);
    }
    if let Some(mass) = field(record, &["m_mass", "mass"]) {
        cinfo = cinfo.with_mass(as_float(mass, "m_mass")?);
    }
    if let Some(filter) = field(record, &["m_collisionFilterInfo", "collision_filter_info"]) {
        cinfo = cinfo.with_collision_filter_info(as_int(filter, "m_collisionFilterInfo")? as u32);
    }
    if let Some(user_data) = field(record, &["m_userData", "user_data"]) {
        cinfo = cinfo.with_user_data(as_int(user_data, "m_userData")? as u64);
    }
    if let Some(Value::String(name)) = field(record, &["m_name", "name"]) {
        cinfo = cinfo.with_name(name.clone());
    }
    Ok(cinfo)
}

/// Scan a bundle for records whose type name contains `Body` and convert each
/// to a [`BodyCinfo`], preserving item order.
pub fn scene_from_bundle(bundle: &Bundle) -> Result<LoadedScene, ConvertError> {
    let mut bodies = Vec::new();
    for (index, value) in bundle.objects.iter().enumerate() {
        let Value::Record(record) = value else {
            continue;
        };
        let Some(name) = type_name(bundle, record.type_id) else {
            continue;
        };
        if !name.contains("Body") {
            continue;
        }
        bodies.push(body_cinfo_from_item(bundle, index as u32)?);
    }
    Ok(LoadedScene { bodies })
}

impl LoadedScene {
    /// Create every body in the scene and return the assigned [`BodyId`]s in
    /// the same order as [`LoadedScene::bodies`].
    pub fn populate(&self, world: &mut World) -> Vec<BodyId> {
        self.bodies
            .iter()
            .map(|cinfo| world.create_body(cinfo))
            .collect()
    }
}

/// Convenience: load a bundle and populate a fresh [`World`] with all bodies.
pub fn load_world(
    bundle: &Bundle,
    cinfo: WorldCinfo,
) -> Result<(World, Vec<BodyId>), ConvertError> {
    let scene = scene_from_bundle(bundle)?;
    let mut world = World::new(cinfo);
    let bodies = scene.populate(&mut world);
    Ok((world, bodies))
}
