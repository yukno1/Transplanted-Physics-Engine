//! Constraint island partitioning (`hknpCollisionIsland` subset).
//!
//! An island is a set of dynamic bodies connected by contacts or joints. Islands
//! are disjoint by construction, so solving them one after another produces the
//! same impulses as one global sweep while keeping each island's data local (the
//! SDK's solver batches use the same decomposition).
//!
//! Static and keyframed bodies do **not** merge islands: two separate stacks on
//! the same ground are two islands, which is what enables per-island sleeping
//! and parallel solving later.

use crate::phys_types::BodyId;

/// Statistics view of one constraint island (`hknpCollisionIsland` analog).
///
/// Bodies are listed in ascending body slot / ID order; the islands themselves
/// are ordered by their lowest dynamic body, so the list is deterministic.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IslandInfo {
    /// Body IDs referenced by the island's contacts/joints, ascending.
    pub bodies: Vec<BodyId>,
    /// Number of dynamic bodies among [`IslandInfo::bodies`].
    pub dynamic_bodies: usize,
    /// Contact constraints solved by the island.
    pub contacts: usize,
    /// Joint constraints solved by the island.
    pub joints: usize,
}

/// Internal island description in solver-body index space.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct Island {
    /// Solver body indices referenced by the island, ascending.
    pub bodies: Vec<usize>,
    /// Indices into the step's contact array, ascending.
    pub contacts: Vec<usize>,
    /// Indices into the step's joint array, ascending.
    pub joints: Vec<usize>,
    /// Number of dynamic bodies in `bodies`.
    pub dynamic_bodies: usize,
}

/// Partition the dynamic bodies into constraint islands.
///
/// `contacts` / `joints` are `(body_a, body_b)` solver indices. Only pairs of
/// dynamic bodies are unioned; every constraint is assigned to the island of its
/// dynamic endpoint (constraints with no dynamic body are ignored). Touched
/// static/keyframed bodies are listed in [`Island::bodies`] but never connect
/// islands.
pub(crate) fn build_islands(
    body_ids: &[BodyId],
    is_dynamic: &[bool],
    contacts: &[(usize, usize)],
    joints: &[(usize, usize)],
) -> Vec<Island> {
    let count = body_ids.len();
    debug_assert_eq!(body_ids.len(), is_dynamic.len());
    let mut parent: Vec<usize> = (0..count).collect();
    let mut touched = vec![false; count];

    let valid = |a: usize, b: usize| a < count && b < count && a != b;

    for &(body_a, body_b) in contacts {
        if !valid(body_a, body_b) {
            continue;
        }
        touched[body_a] = true;
        touched[body_b] = true;
        if is_dynamic[body_a] && is_dynamic[body_b] {
            union(&mut parent, body_a, body_b);
        }
    }
    for &(body_a, body_b) in joints {
        if !valid(body_a, body_b) {
            continue;
        }
        touched[body_a] = true;
        touched[body_b] = true;
        if is_dynamic[body_a] && is_dynamic[body_b] {
            union(&mut parent, body_a, body_b);
        }
    }

    // Create islands in ascending dynamic-body order; the representative root
    // is always the lowest index in the set, so this order is stable.
    let mut root_to_island: Vec<Option<usize>> = vec![None; count];
    let mut islands: Vec<Island> = Vec::new();
    for index in 0..count {
        if !touched[index] || !is_dynamic[index] {
            continue;
        }
        let root = find(&mut parent, index);
        if root_to_island[root].is_none() {
            root_to_island[root] = Some(islands.len());
            islands.push(Island::default());
        }
    }

    // Dynamic bodies belong to exactly one island (their union-find root).
    for index in 0..count {
        if !touched[index] || !is_dynamic[index] {
            continue;
        }
        let root = find(&mut parent, index);
        let Some(island_index) = root_to_island[root] else {
            continue;
        };
        let island = &mut islands[island_index];
        island.bodies.push(index);
        island.dynamic_bodies += 1;
    }

    // Static/keyframed bodies never merge islands, so a shared ground body is
    // listed in every island it touches (it contributes no dynamic DOF).
    for index in 0..count {
        if !touched[index] || is_dynamic[index] {
            continue;
        }
        let mut memberships: Vec<usize> = Vec::new();
        for &(body_a, body_b) in contacts.iter().chain(joints.iter()) {
            if !valid(body_a, body_b) {
                continue;
            }
            let neighbour = if body_a == index {
                body_b
            } else if body_b == index {
                body_a
            } else {
                continue;
            };
            if !is_dynamic[neighbour] {
                continue;
            }
            if let Some(island_index) = root_to_island[find(&mut parent, neighbour)] {
                memberships.push(island_index);
            }
        }
        memberships.sort_unstable();
        memberships.dedup();
        for island_index in memberships {
            islands[island_index].bodies.push(index);
        }
    }

    for island in &mut islands {
        island.bodies.sort_unstable();
    }

    for (contact_index, &(body_a, body_b)) in contacts.iter().enumerate() {
        if !valid(body_a, body_b) {
            continue;
        }
        let root = if is_dynamic[body_a] {
            find(&mut parent, body_a)
        } else if is_dynamic[body_b] {
            find(&mut parent, body_b)
        } else {
            continue;
        };
        if let Some(island_index) = root_to_island[root] {
            islands[island_index].contacts.push(contact_index);
        }
    }

    for (joint_index, &(body_a, body_b)) in joints.iter().enumerate() {
        if !valid(body_a, body_b) {
            continue;
        }
        let root = if is_dynamic[body_a] {
            find(&mut parent, body_a)
        } else if is_dynamic[body_b] {
            find(&mut parent, body_b)
        } else {
            continue;
        };
        if let Some(island_index) = root_to_island[root] {
            islands[island_index].joints.push(joint_index);
        }
    }

    islands
}

/// Union-find `find` with path halving.
fn find(parent: &mut [usize], mut index: usize) -> usize {
    while parent[index] != index {
        parent[index] = parent[parent[index]];
        index = parent[index];
    }
    index
}

/// Union-find `union`: the lower root always wins, keeping representatives
/// minimal and the result independent of insertion order.
fn union(parent: &mut [usize], a: usize, b: usize) {
    let root_a = find(parent, a);
    let root_b = find(parent, b);
    if root_a == root_b {
        return;
    }
    if root_a < root_b {
        parent[root_b] = root_a;
    } else {
        parent[root_a] = root_b;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bodies(count: usize) -> Vec<BodyId> {
        (0..count as u64).map(BodyId::new).collect()
    }

    /// Body 0 is the static ground; the rest are dynamic.
    fn dynamics(count: usize) -> Vec<bool> {
        (0..count).map(|index| index != 0).collect()
    }

    #[test]
    fn separate_stacks_on_one_ground_form_separate_islands() {
        let ids = bodies(5);
        let dynamic = dynamics(5);
        // Two stacks share the static ground 0: 1-2 and 3-4.
        let contacts = [(0, 1), (1, 2), (0, 3), (3, 4)];
        let islands = build_islands(&ids, &dynamic, &contacts, &[]);
        assert_eq!(islands.len(), 2);
        assert_eq!(islands[0].dynamic_bodies, 2);
        assert_eq!(islands[0].bodies, vec![0, 1, 2]);
        assert_eq!(islands[0].contacts, vec![0, 1]);
        assert_eq!(islands[1].bodies, vec![0, 3, 4]);
        assert_eq!(islands[1].contacts, vec![2, 3]);
    }

    #[test]
    fn contacts_and_joints_transitively_merge_islands() {
        let ids = bodies(4);
        let dynamic = dynamics(4);
        // 0 (static) - 1, then contact 1-2 and joint 2-3: one island.
        let contacts = [(0, 1), (1, 2)];
        let joints = [(2, 3)];
        let islands = build_islands(&ids, &dynamic, &contacts, &joints);
        assert_eq!(islands.len(), 1);
        assert_eq!(islands[0].dynamic_bodies, 3);
        assert_eq!(islands[0].bodies, vec![0, 1, 2, 3]);
        assert_eq!(islands[0].contacts, vec![0, 1]);
        assert_eq!(islands[0].joints, vec![0]);
    }

    #[test]
    fn static_static_and_unconstrained_bodies_are_excluded() {
        let ids = bodies(4);
        let dynamic = dynamics(4);
        // A static-static contact and a dynamic body with no constraints.
        let islands = build_islands(&ids, &dynamic, &[(0, 0), (0, 3)], &[]);
        // (0, 0) is invalid and (0, 3) touches body 3 only.
        assert_eq!(islands.len(), 1);
        assert_eq!(islands[0].bodies, vec![0, 3]);
        assert_eq!(islands[0].contacts, vec![1]);
    }

    #[test]
    fn ordering_does_not_depend_on_contact_order() {
        let ids = bodies(5);
        let dynamic = dynamics(5);
        let forward = build_islands(&ids, &dynamic, &[(0, 1), (0, 2), (0, 3), (0, 4)], &[]);
        let shuffled = build_islands(&ids, &dynamic, &[(0, 4), (0, 2), (0, 3), (0, 1)], &[]);
        assert_eq!(forward.len(), shuffled.len());
        for (left, right) in forward.iter().zip(shuffled.iter()) {
            assert_eq!(left.bodies, right.bodies);
            assert_eq!(left.dynamic_bodies, right.dynamic_bodies);
        }
    }

    #[test]
    fn invalid_indices_are_ignored() {
        let ids = bodies(2);
        let dynamic = dynamics(2);
        let islands = build_islands(&ids, &dynamic, &[(0, 1), (0, 99)], &[]);
        assert_eq!(islands.len(), 1);
        assert_eq!(islands[0].bodies, vec![0, 1]);
        assert_eq!(islands[0].contacts, vec![0]);
    }
}
