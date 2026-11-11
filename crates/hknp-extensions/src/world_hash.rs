//! Deterministic world hash (`hknp-WorldHash` subset).
//!
//! FNV-1a over the body snapshots in slot order. All floats are hashed by
//! `f32::to_bits`, so the hash is stable across runs and does not depend on
//! textual formatting.

use hknp_world::{World, WorldSnapshot};

/// FNV-1a 64-bit offset basis.
pub const FNV_OFFSET_BASIS: u64 = 0xCBF2_9CE4_8422_2325;
/// FNV-1a 64-bit prime.
pub const FNV_PRIME: u64 = 0x0000_0100_0000_01B3;

/// Hash of a world snapshot.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct WorldHash(u64);

impl WorldHash {
    /// Hash a snapshot.
    pub fn of(snapshot: &WorldSnapshot) -> Self {
        let mut state = FNV_OFFSET_BASIS;
        mix(&mut state, snapshot.step_count);
        mix(&mut state, snapshot.bodies.len() as u64);
        for body in &snapshot.bodies {
            mix(&mut state, body.body.value());
            for component in [
                body.position.x,
                body.position.y,
                body.position.z,
                body.orientation.x,
                body.orientation.y,
                body.orientation.z,
                body.orientation.w,
                body.linear_velocity.x,
                body.linear_velocity.y,
                body.linear_velocity.z,
                body.angular_velocity.x,
                body.angular_velocity.y,
                body.angular_velocity.z,
            ] {
                mix(&mut state, u64::from(component.to_bits()));
            }
        }
        Self(state)
    }

    /// Raw hash value.
    pub fn value(self) -> u64 {
        self.0
    }

    /// Lowercase hex rendering (`0x…`).
    pub fn to_hex(self) -> String {
        format!("{:#018x}", self.0)
    }
}

/// Hash a snapshot (convenience).
pub fn hash_snapshot(snapshot: &WorldSnapshot) -> u64 {
    WorldHash::of(snapshot).value()
}

/// Hash the current state of a world.
pub fn hash_world(world: &World) -> u64 {
    WorldHash::of(&world.snapshot()).value()
}

fn mix(state: &mut u64, word: u64) {
    for byte in word.to_le_bytes() {
        *state ^= u64::from(byte);
        *state = state.wrapping_mul(FNV_PRIME);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hk_math::{Quaternion, Vector4};
    use hknp_world::BodySnapshot;

    fn body(id: u64, position: Vector4) -> BodySnapshot {
        BodySnapshot {
            body: hknp_types::BodyId::new(id),
            position,
            orientation: Quaternion::IDENTITY,
            linear_velocity: Vector4::ZERO,
            angular_velocity: Vector4::ZERO,
        }
    }

    fn snapshot() -> WorldSnapshot {
        WorldSnapshot {
            step_count: 7,
            bodies: vec![
                body(1, Vector4::ZERO),
                body(2, Vector4::from_xyz(1.0, 2.0, 3.0)),
            ],
        }
    }

    #[test]
    fn equal_snapshots_have_equal_hashes() {
        let first = snapshot();
        let second = snapshot();
        assert_eq!(WorldHash::of(&first), WorldHash::of(&second));
        assert_eq!(hash_snapshot(&first), hash_snapshot(&second));
    }

    #[test]
    fn changed_state_changes_hash() {
        let original = snapshot();
        let mut changed = snapshot();
        changed.step_count += 1;
        assert_ne!(hash_snapshot(&original), hash_snapshot(&changed));

        let mut changed = snapshot();
        changed.bodies[1].position.x = 1.000_001;
        assert_ne!(hash_snapshot(&original), hash_snapshot(&changed));
    }

    #[test]
    fn body_order_is_part_of_the_hash() {
        let mut first = snapshot();
        let mut second = snapshot();
        second.bodies.swap(0, 1);
        assert_ne!(hash_snapshot(&first), hash_snapshot(&second));
        first.bodies.swap(0, 1);
        assert_eq!(hash_snapshot(&first), hash_snapshot(&second));
    }

    #[test]
    fn hex_rendering_is_stable() {
        let hash = WorldHash::of(&snapshot());
        assert!(hash.to_hex().starts_with("0x"));
        assert_eq!(hash.to_hex().len(), 18);
    }

    #[test]
    fn world_hash_matches_snapshot_hash() {
        let world = hknp_world::World::new(hknp_world::WorldCinfo::new());
        assert_eq!(hash_world(&world), hash_snapshot(&world.snapshot()));
    }
}
