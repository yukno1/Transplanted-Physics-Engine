//! Character proxy manager (`hknpCharacterProxyManager` bridge subset).
//!
//! Owns a set of proxies, connects them to [`phys_world::PostSolveSignal`],
//! dispatches per-proxy callbacks in id order, and offers a batched character
//! step. Heavy per-character work is partitioned with
//! [`base_thread::parallel::parallel_for_chunks`]; listener callbacks and
//! deferred interaction impulses are always applied serially in id order so
//! traces stay bit-identical between 1 and N threads.

use std::sync::{Arc, Mutex};

use base_math::Aabb;
use base_thread::parallel::parallel_for_chunks;
use phys_world::{
    ContactModifier, ContactModifierId, PostCollideConnectionId, PostSolveConnectionId, World,
};

use super::contact_editing::CharacterContactBridge;
use super::listener::CharacterInteractionInfo;
use super::proxy::CharacterProxy;
use super::rigid_body::CharacterRigidBody;
use super::surface::SurfaceInfo;

fn safe_normal(velocity: base_math::Vector4) -> base_math::Vector4 {
    let length = velocity.length3();
    if length > 1e-10 {
        velocity * (1.0 / length)
    } else {
        base_math::Vector4::ZERO
    }
}

#[derive(Default)]
struct ManagerState {
    proxies: Vec<(u64, CharacterProxy)>,
    rigid_bodies: Vec<(u64, CharacterRigidBody)>,
    next_id: u64,
    next_rigid_body_id: u64,
}

/// Manages character proxies and bridges world post-solve notifications.
#[derive(Clone, Default)]
pub struct CharacterProxyManager {
    state: Arc<Mutex<ManagerState>>,
    connections: Arc<Mutex<Vec<PostSolveConnectionId>>>,
    contact_bridge: CharacterContactBridge,
    contact_modifiers: Arc<Mutex<Vec<ContactModifierId>>>,
    post_collide_connections: Arc<Mutex<Vec<PostCollideConnectionId>>>,
}

impl CharacterProxyManager {
    /// Create an empty manager.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a proxy and return its stable id (starting at 1).
    ///
    /// The manager takes ownership of the proxy's `character_callback`, so it
    /// is fired from the world's post-solve signal rather than at the end of
    /// `CharacterProxy::integrate`.
    pub fn add_proxy(&self, mut proxy: CharacterProxy) -> u64 {
        let mut state = self.state.lock().expect("character manager lock poisoned");
        state.next_id += 1;
        let id = state.next_id;
        proxy.set_id(id);
        proxy.set_post_solve_callback(true);
        state.proxies.push((id, proxy));
        id
    }

    /// Unregister a proxy and return it (with standalone callbacks restored).
    pub fn remove_proxy(&self, id: u64) -> Option<CharacterProxy> {
        let mut state = self.state.lock().expect("character manager lock poisoned");
        let index = state.proxies.iter().position(|(other, _)| *other == id)?;
        let (_, mut proxy) = state.proxies.remove(index);
        proxy.set_post_solve_callback(false);
        Some(proxy)
    }

    /// Register a rigid-body character and return its stable id (starting at 1).
    pub fn add_rigid_body(&self, character: CharacterRigidBody) -> u64 {
        let mut state = self.state.lock().expect("character manager lock poisoned");
        state.next_rigid_body_id += 1;
        let id = state.next_rigid_body_id;
        let body = character.body();
        self.contact_bridge
            .register(body, character.listener_slot());
        state.rigid_bodies.push((id, character));
        id
    }

    /// Unregister a rigid-body character.
    pub fn remove_rigid_body(&self, id: u64) -> Option<CharacterRigidBody> {
        let mut state = self.state.lock().expect("character manager lock poisoned");
        let index = state
            .rigid_bodies
            .iter()
            .position(|(other, _)| *other == id)?;
        let (_, character) = state.rigid_bodies.remove(index);
        self.contact_bridge.unregister(character.body());
        Some(character)
    }

    /// Number of registered rigid-body characters.
    pub fn rigid_body_count(&self) -> usize {
        self.state
            .lock()
            .expect("character manager lock poisoned")
            .rigid_bodies
            .len()
    }

    /// Step every rigid-body character and dispatch its post-solve listener
    /// callback after the world step.
    pub fn step_rigid_bodies(
        &self,
        world: &mut World,
        delta_time: f32,
        surfaces: &[(u64, SurfaceInfo)],
    ) {
        let mut state = self.state.lock().expect("character manager lock poisoned");
        for (id, character) in state.rigid_bodies.iter_mut() {
            let surface = surfaces
                .iter()
                .find(|(other, _)| other == id)
                .map(|(_, surface)| *surface)
                .unwrap_or(SurfaceInfo::UNSUPPORTED);
            character.step(world, delta_time, &surface);
        }
    }

    /// Visit a registered proxy mutably (for listeners/configuration).
    pub fn with_proxy<R>(
        &self,
        id: u64,
        visit: impl FnOnce(&mut CharacterProxy) -> R,
    ) -> Option<R> {
        let mut state = self.state.lock().expect("character manager lock poisoned");
        state
            .proxies
            .iter_mut()
            .find(|(other, _)| *other == id)
            .map(|(_, proxy)| visit(proxy))
    }

    /// Visit a registered rigid-body character mutably.
    pub fn with_rigid_body<R>(
        &self,
        id: u64,
        visit: impl FnOnce(&mut CharacterRigidBody) -> R,
    ) -> Option<R> {
        let mut state = self.state.lock().expect("character manager lock poisoned");
        state
            .rigid_bodies
            .iter_mut()
            .find(|(other, _)| *other == id)
            .map(|(_, character)| visit(character))
    }

    /// Number of registered proxies.
    pub fn len(&self) -> usize {
        self.state
            .lock()
            .expect("character manager lock poisoned")
            .proxies
            .len()
    }

    /// Whether no proxies are registered.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Registered proxy ids in ascending order.
    pub fn ids(&self) -> Vec<u64> {
        let state = self.state.lock().expect("character manager lock poisoned");
        state.proxies.iter().map(|(id, _)| *id).collect()
    }

    /// Connect to `world`'s post-solve signal. Calling this twice registers a
    /// second independent connection; use [`Self::detach`] to remove them all.
    pub fn attach(&self, world: &World) -> PostSolveConnectionId {
        let modifier = Arc::new(self.contact_bridge.clone()) as Arc<dyn ContactModifier>;
        let modifier_id = world.register_contact_modifier(modifier);
        self.contact_modifiers
            .lock()
            .expect("character manager lock poisoned")
            .push(modifier_id);

        let bridge = self.contact_bridge.clone();
        let post_collide =
            world.subscribe_post_collide(move |event| bridge.dispatch_post_collide(event));
        self.post_collide_connections
            .lock()
            .expect("character manager lock poisoned")
            .push(post_collide);

        let state = Arc::clone(&self.state);
        let connection = world.subscribe_post_solve(move |_event| {
            let mut state = state.lock().expect("character manager lock poisoned");
            let snapshots: Vec<(u64, Aabb, base_math::Vector4, base_math::Vector4)> = state
                .proxies
                .iter()
                .map(|(id, proxy)| {
                    (
                        *id,
                        proxy.shape().calc_aabb(proxy.transform()),
                        proxy.position(),
                        proxy.linear_velocity(),
                    )
                })
                .collect();
            let mut interactions = Vec::new();
            for i in 0..snapshots.len() {
                for j in (i + 1)..snapshots.len() {
                    if !snapshots[i].1.overlaps(&snapshots[j].1) {
                        continue;
                    }
                    let (first, second) = if snapshots[i].0 < snapshots[j].0 {
                        (snapshots[i], snapshots[j])
                    } else {
                        (snapshots[j], snapshots[i])
                    };
                    interactions.push((
                        first.0,
                        second.0,
                        CharacterInteractionInfo {
                            other_id: second.0,
                            position: second.2,
                            normal: -safe_normal(second.3),
                        },
                    ));
                }
            }
            interactions.sort_unstable_by_key(|(first, second, _)| (*first, *second));
            for (id, proxy) in state.proxies.iter_mut() {
                for (first, second, info) in &interactions {
                    if *id == *first {
                        proxy.fire_character_interaction(info);
                    } else if *id == *second {
                        proxy.fire_character_interaction(&CharacterInteractionInfo {
                            other_id: *first,
                            position: info.position,
                            normal: -info.normal,
                        });
                    }
                }
                proxy.fire_post_solve_callback();
            }
            for (_, character) in state.rigid_bodies.iter_mut() {
                // The event carries no world reference, but the manager is
                // invoked between world steps, so cached body state is stable.
                character.fire_post_solve_from_snapshot();
            }
        });
        self.connections
            .lock()
            .expect("character manager lock poisoned")
            .push(connection);
        connection
    }

    /// Remove all world connections created by [`Self::attach`].
    pub fn detach(&self, world: &World) {
        let mut connections = self
            .connections
            .lock()
            .expect("character manager lock poisoned");
        for connection in connections.drain(..) {
            let _ = world.unsubscribe_post_solve(connection);
        }
        drop(connections);

        let mut modifiers = self
            .contact_modifiers
            .lock()
            .expect("character manager lock poisoned");
        for modifier in modifiers.drain(..) {
            world.unregister_contact_modifier(modifier);
        }
        drop(modifiers);

        let mut post_collide = self
            .post_collide_connections
            .lock()
            .expect("character manager lock poisoned");
        for connection in post_collide.drain(..) {
            let _ = world.unsubscribe_post_collide(connection);
        }
    }

    /// Number of live post-solve connections.
    pub fn connection_count(&self) -> usize {
        self.connections
            .lock()
            .expect("character manager lock poisoned")
            .len()
    }

    /// Number of live contact-edit / post-collide connections.
    pub fn contact_connection_count(&self) -> usize {
        let modifiers = self
            .contact_modifiers
            .lock()
            .expect("character manager lock poisoned")
            .len();
        let post_collide = self
            .post_collide_connections
            .lock()
            .expect("character manager lock poisoned")
            .len();
        modifiers + post_collide
    }

    /// Step all registered characters and apply their deferred pushes.
    ///
    /// `surfaces` carries one [`SurfaceInfo`] per proxy id; missing entries use
    /// [`SurfaceInfo::UNSUPPORTED`]. Interaction candidates are computed with
    /// `threads` workers, then callbacks and pushes are applied in id order.
    pub fn step_characters(
        &self,
        world: &mut World,
        delta_time: f32,
        surfaces: &[(u64, SurfaceInfo)],
        threads: usize,
    ) {
        let mut state = self.state.lock().expect("character manager lock poisoned");
        let snapshots: Vec<(u64, Aabb, base_math::Vector4)> = state
            .proxies
            .iter()
            .map(|(id, proxy)| {
                (
                    *id,
                    proxy.shape().calc_aabb(proxy.transform()),
                    proxy.position(),
                )
            })
            .collect();

        let pairs = Mutex::new(Vec::new());
        parallel_for_chunks(snapshots.len(), threads, |start, end| {
            let mut local = Vec::new();
            for i in start..end {
                for j in (i + 1)..snapshots.len() {
                    if snapshots[i].1.overlaps(&snapshots[j].1) {
                        local.push((snapshots[i].0, snapshots[j].0));
                    }
                }
            }
            if !local.is_empty() {
                pairs
                    .lock()
                    .expect("character interaction lock poisoned")
                    .extend(local);
            }
        });
        let mut pairs = pairs
            .into_inner()
            .expect("character interaction lock poisoned");
        pairs.sort_unstable();
        pairs.dedup();

        for (first, second) in pairs {
            let info = {
                let other = state
                    .proxies
                    .iter()
                    .find(|(id, _)| *id == second)
                    .map(|(_, proxy)| (proxy.position(), proxy.linear_velocity()));
                match other {
                    Some((position, velocity)) => CharacterInteractionInfo {
                        other_id: second,
                        position,
                        normal: -safe_normal(velocity),
                    },
                    None => continue,
                }
            };
            for (id, proxy) in state.proxies.iter_mut() {
                if *id == first {
                    proxy.fire_character_interaction(&info);
                } else if *id == second {
                    proxy.fire_character_interaction(&CharacterInteractionInfo {
                        other_id: first,
                        position: info.position,
                        normal: -info.normal,
                    });
                }
            }
        }

        for (id, proxy) in state.proxies.iter_mut() {
            let surface = surfaces
                .iter()
                .find(|(other, _)| other == id)
                .map(|(_, surface)| *surface)
                .unwrap_or(SurfaceInfo::UNSUPPORTED);
            proxy.integrate(world, delta_time, &surface);
            proxy.apply_deferred_updates(world);
        }
    }
}
