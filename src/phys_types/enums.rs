//! Enumerations shared by the physics runtime.
//!
//! Ports of `hknpMotionType`, `hknpShapeType` and `hknpCollisionDispatchType`
//! from `original/Physics/Source/Physics/Physics/hknpTypes.h` and
//! `Collide/Shape/hknpShapeType.h`.

/// `hknpMotionType` — how a body is simulated.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u32)]
pub enum MotionType {
    /// `STATIC` — never moves unless set manually.
    #[default]
    Static = 0,
    /// `KEYFRAMED` — infinite mass/inertia, moved by animation.
    Keyframed = 1,
    /// `DYNAMIC` — fully simulated.
    Dynamic = 2,
}

impl MotionType {
    /// `NUM_TYPES` — number of motion types.
    pub const NUM_TYPES: u32 = 3;

    /// Decode a raw enum value (`hknpMotionType::Enum`).
    #[inline]
    pub const fn from_raw(raw: u32) -> Option<Self> {
        match raw {
            0 => Some(Self::Static),
            1 => Some(Self::Keyframed),
            2 => Some(Self::Dynamic),
            _ => None,
        }
    }

    /// Raw enum value.
    #[inline]
    pub const fn as_raw(self) -> u32 {
        self as u32
    }

    /// Whether this is a static body.
    #[inline]
    pub const fn is_static(self) -> bool {
        matches!(self, Self::Static)
    }

    /// Whether this is a dynamic body.
    #[inline]
    pub const fn is_dynamic(self) -> bool {
        matches!(self, Self::Dynamic)
    }
}

/// `hknpShapeType` — concrete shape type, mainly for dispatch and debugging.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u32)]
pub enum ShapeType {
    /// `CONVEX` — general convex shape.
    Convex = 0,
    /// `SPHERE`.
    Sphere = 1,
    /// `CAPSULE`.
    Capsule = 2,
    /// `CYLINDER`.
    Cylinder = 3,
    /// `TRIANGLE` (also used for quads).
    Triangle = 4,
    /// `BOX`.
    #[default]
    Box = 5,
    /// `DEBRIS`.
    Debris = 6,
    /// `COMPOSITE` — abstract type for shapes with children.
    Composite = 7,
    /// `MESH`.
    Mesh = 8,
    /// `COMPRESSED_MESH`.
    CompressedMesh = 9,
    /// `EXTERN_MESH`.
    ExternMesh = 10,
    /// `COMPOUND`.
    Compound = 11,
    /// `DISTANCE_FIELD_BASE`.
    DistanceFieldBase = 12,
    /// `HEIGHT_FIELD`.
    HeightField = 13,
    /// `DISTANCE_FIELD`.
    DistanceField = 14,
    /// `PARTICLE_SYSTEM`.
    ParticleSystem = 15,
    /// `SCALED_CONVEX`.
    ScaledConvex = 16,
    /// `TRANSFORMED`.
    Transformed = 17,
    /// `MASKED`.
    Masked = 18,
    /// `MASKED_COMPOUND`.
    MaskedCompound = 19,
    /// `BREAKABLE_COMPOUND`.
    BreakableCompound = 20,
    /// `LOD`.
    Lod = 21,
    /// `DUMMY`.
    Dummy = 22,
    /// `USER_0`.
    User0 = 23,
    /// `USER_1`.
    User1 = 24,
    /// `USER_2`.
    User2 = 25,
    /// `USER_3`.
    User3 = 26,
    /// `USER_4`.
    User4 = 27,
    /// `USER_5`.
    User5 = 28,
    /// `USER_6`.
    User6 = 29,
    /// `USER_7`.
    User7 = 30,
    /// `NUM_SHAPE_TYPES`.
    NumShapeTypes = 31,
    /// `INVALID`.
    Invalid = 32,
}

impl ShapeType {
    /// Decode a raw enum value.
    #[inline]
    pub const fn from_raw(raw: u32) -> Option<Self> {
        match raw {
            0 => Some(Self::Convex),
            1 => Some(Self::Sphere),
            2 => Some(Self::Capsule),
            3 => Some(Self::Cylinder),
            4 => Some(Self::Triangle),
            5 => Some(Self::Box),
            6 => Some(Self::Debris),
            7 => Some(Self::Composite),
            8 => Some(Self::Mesh),
            9 => Some(Self::CompressedMesh),
            10 => Some(Self::ExternMesh),
            11 => Some(Self::Compound),
            12 => Some(Self::DistanceFieldBase),
            13 => Some(Self::HeightField),
            14 => Some(Self::DistanceField),
            15 => Some(Self::ParticleSystem),
            16 => Some(Self::ScaledConvex),
            17 => Some(Self::Transformed),
            18 => Some(Self::Masked),
            19 => Some(Self::MaskedCompound),
            20 => Some(Self::BreakableCompound),
            21 => Some(Self::Lod),
            22 => Some(Self::Dummy),
            23 => Some(Self::User0),
            24 => Some(Self::User1),
            25 => Some(Self::User2),
            26 => Some(Self::User3),
            27 => Some(Self::User4),
            28 => Some(Self::User5),
            29 => Some(Self::User6),
            30 => Some(Self::User7),
            31 => Some(Self::NumShapeTypes),
            32 => Some(Self::Invalid),
            _ => None,
        }
    }

    /// Raw enum value.
    #[inline]
    pub const fn as_raw(self) -> u32 {
        self as u32
    }

    /// Name used by `hknpShapeType::toString`.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Convex => "CONVEX",
            Self::Sphere => "SPHERE",
            Self::Capsule => "CAPSULE",
            Self::Cylinder => "CYLINDER",
            Self::Triangle => "TRIANGLE",
            Self::Box => "BOX",
            Self::Debris => "DEBRIS",
            Self::Composite => "COMPOSITE",
            Self::Mesh => "MESH",
            Self::CompressedMesh => "COMPRESSED_MESH",
            Self::ExternMesh => "EXTERN_MESH",
            Self::Compound => "COMPOUND",
            Self::DistanceFieldBase => "DISTANCE_FIELD_BASE",
            Self::HeightField => "HEIGHT_FIELD",
            Self::DistanceField => "DISTANCE_FIELD",
            Self::ParticleSystem => "PARTICLE_SYSTEM",
            Self::ScaledConvex => "SCALED_CONVEX",
            Self::Transformed => "TRANSFORMED",
            Self::Masked => "MASKED",
            Self::MaskedCompound => "MASKED_COMPOUND",
            Self::BreakableCompound => "BREAKABLE_COMPOUND",
            Self::Lod => "LOD",
            Self::Dummy => "DUMMY",
            Self::User0 => "USER_0",
            Self::User1 => "USER_1",
            Self::User2 => "USER_2",
            Self::User3 => "USER_3",
            Self::User4 => "USER_4",
            Self::User5 => "USER_5",
            Self::User6 => "USER_6",
            Self::User7 => "USER_7",
            Self::NumShapeTypes => "NUM_SHAPE_TYPES",
            Self::Invalid => "INVALID",
        }
    }
}

/// `hknpCollisionDispatchType` — which narrow-phase path a shape uses.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u32)]
pub enum CollisionDispatchType {
    /// `NONE` — ignored by collision detection.
    #[default]
    None = 0,
    /// `DEBRIS` — fast approximate manifold code path.
    Debris = 1,
    /// `CONVEX` — convex interface methods.
    Convex = 2,
    /// `COMPOSITE` — composite interface methods.
    Composite = 3,
    /// `DISTANCE_FIELD`.
    DistanceField = 4,
    /// `USER_0`.
    User0 = 5,
    /// `USER_1`.
    User1 = 6,
    /// `USER_2`.
    User2 = 7,
    /// `USER_3`.
    User3 = 8,
    /// `USER_4`.
    User4 = 9,
    /// `USER_5`.
    User5 = 10,
    /// `USER_6`.
    User6 = 11,
    /// `USER_7`.
    User7 = 12,
    /// `NUM_TYPES`.
    NumTypes = 13,
}

impl CollisionDispatchType {
    /// Decode a raw enum value.
    #[inline]
    pub const fn from_raw(raw: u32) -> Option<Self> {
        match raw {
            0 => Some(Self::None),
            1 => Some(Self::Debris),
            2 => Some(Self::Convex),
            3 => Some(Self::Composite),
            4 => Some(Self::DistanceField),
            5 => Some(Self::User0),
            6 => Some(Self::User1),
            7 => Some(Self::User2),
            8 => Some(Self::User3),
            9 => Some(Self::User4),
            10 => Some(Self::User5),
            11 => Some(Self::User6),
            12 => Some(Self::User7),
            13 => Some(Self::NumTypes),
            _ => None,
        }
    }

    /// Raw enum value.
    #[inline]
    pub const fn as_raw(self) -> u32 {
        self as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motion_type_values_match_the_sdk() {
        assert_eq!(MotionType::Static.as_raw(), 0);
        assert_eq!(MotionType::Keyframed.as_raw(), 1);
        assert_eq!(MotionType::Dynamic.as_raw(), 2);
        assert_eq!(MotionType::default(), MotionType::Static);
        assert!(MotionType::from_raw(0).unwrap().is_static());
        assert!(MotionType::from_raw(2).unwrap().is_dynamic());
        assert_eq!(MotionType::from_raw(3), None);
        assert_eq!(MotionType::NUM_TYPES, 3);
    }

    #[test]
    fn shape_type_discriminants_and_names() {
        assert_eq!(ShapeType::Sphere.as_raw(), 1);
        assert_eq!(ShapeType::Box.as_raw(), 5);
        assert_eq!(ShapeType::Compound.as_raw(), 11);
        assert_eq!(ShapeType::HeightField.as_raw(), 13);
        assert_eq!(ShapeType::User7.as_raw(), 30);
        assert_eq!(ShapeType::Invalid.as_raw(), 32);
        for raw in 0..=32 {
            let shape = ShapeType::from_raw(raw).expect("known shape type");
            assert_eq!(shape.as_raw(), raw);
            assert!(!shape.as_str().is_empty());
        }
        assert_eq!(ShapeType::from_raw(33), None);
    }

    #[test]
    fn dispatch_type_values() {
        assert_eq!(CollisionDispatchType::None.as_raw(), 0);
        assert_eq!(CollisionDispatchType::Convex.as_raw(), 2);
        assert_eq!(CollisionDispatchType::Composite.as_raw(), 3);
        for raw in 0..=13 {
            let dispatch = CollisionDispatchType::from_raw(raw).expect("known dispatch type");
            assert_eq!(dispatch.as_raw(), raw);
        }
        assert_eq!(CollisionDispatchType::from_raw(14), None);
    }
}
