//! Signal / slot observer pattern.
//!
//! Port of `hkSignal`, `hkSlot`, `hkSignal0` … `hkSignal6`
//! (`original/Common/Source/Common/Base/Types/hkSignalSlots.*`).
//!
//! # Differences from the C++ implementation
//!
//! The original is an intrusive linked list of polymorphic `hkSlot` objects
//! identified by object pointer + member-function-pointer (+ optional user data).
//! Rust closures express the same thing more directly, so this port keeps the
//! *behaviour* — ordered delivery, `ConnectionId`-based disconnect, and
//! disconnect-while-firing — while using `Rc<dyn Fn(&Args)>` slots and a `Vec`
//! internally.
//!
//! * `hkSignal<...>::fire(...)` → [`Signal::emit`].
//! * `hkSignal<...>::subscribe(function)` → [`Signal::subscribe`] returning a
//!   [`ConnectionId`]; `unsubscribe(ConnectionId)` mirrors the C++ overload.
//! * `getNumSubscriptions()` → [`Signal::num_subscriptions`].
//! * `hkSignal0` … `hkSignal6` → `Signal<()>`, `Signal<(P0,)>`, `Signal<(P0, P1)>`,
//!   and so on; tuples carry multiple parameters.
//! * Method-pointer matching (`find(object, method, ...)`, `hkViewPtr`-style weak
//!   capture) is not reproduced; closures capture what they need.
//!
//! # Firing semantics
//!
//! Subscribers are called in subscription order. A slot unsubscribed *while the
//! signal is firing* is not called if it has not been reached yet; removal from
//! the slot list is deferred until the outermost `emit` returns (the C++
//! implementation uses the last bit of `m_slots` for the same purpose). Calling
//! `emit` re-entrantly from inside a callback is supported.
//!
//! ```
//! use hk_types::Signal;
//!
//! let signal = Signal::<(i32,)>::new();
//! let id = signal.subscribe(|args: &(i32,)| println!("got {}", args.0));
//! signal.emit(&(42,));
//! assert!(signal.unsubscribe(id));
//! signal.emit(&(1,));
//! ```
//!
//! The signal is **not** `Send`/`Sync` in this phase (the C++ SDK offers
//! `_fireMtUncheckedNoCleanup` for multithreaded firing; that is Phase 10 work).

use std::cell::RefCell;
use std::rc::Rc;

/// Identifier of one subscription, returned by [`Signal::subscribe`].
///
/// The C++ API uses the address of the slot as a `ConnectionId`; ids are opaque
/// anyway, so a monotonically increasing integer is used here. Ids are never
/// reused.
pub type ConnectionId = u64;

type Callback<A> = Rc<dyn Fn(&A)>;

struct Slot<A> {
    id: ConnectionId,
    active: bool,
    callback: Callback<A>,
}

struct Inner<A> {
    slots: Vec<Slot<A>>,
    next_id: ConnectionId,
    /// Number of `emit` frames currently on the stack.
    firing: u32,
}

/// Result of inspecting the next slot during `emit`.
enum Step<A> {
    Call(Callback<A>),
    Skip,
    Done,
}

/// A named, ordered set of callbacks (port of `hkSignalN`).
///
/// `A = ()` for zero-argument signals; use tuples for the 1..=6 argument variants
/// of the original (`Signal<(P0,)>` … `Signal<(P0, P1, P2, P3, P4, P5)>`).
pub struct Signal<A = ()> {
    name: &'static str,
    inner: RefCell<Inner<A>>,
}

impl<A> Signal<A> {
    /// Create an unnamed signal (`hkSignalN("")`).
    #[inline]
    pub const fn new() -> Self {
        Self::named("")
    }

    /// Create a signal with a debug name (the C++ constructors take a name).
    #[inline]
    pub const fn named(name: &'static str) -> Self {
        Self {
            name,
            inner: RefCell::new(Inner {
                slots: Vec::new(),
                next_id: 1,
                firing: 0,
            }),
        }
    }

    /// The debug name passed to [`Signal::named`].
    #[inline]
    pub const fn name(&self) -> &'static str {
        self.name
    }

    /// Connect `callback`; it is called with the emitted arguments in subscription
    /// order.
    ///
    /// Returns the [`ConnectionId`] needed to disconnect later.
    pub fn subscribe<F>(&self, callback: F) -> ConnectionId
    where
        F: Fn(&A) + 'static,
    {
        let mut inner = self.inner.borrow_mut();
        let id = inner.next_id;
        inner.next_id = inner
            .next_id
            .checked_add(1)
            .expect("connection id space exhausted");
        inner.slots.push(Slot {
            id,
            active: true,
            callback: Rc::new(callback),
        });
        id
    }

    /// Disconnect a connection; returns `false` if the id was unknown or already
    /// disconnected.
    ///
    /// If called while the signal is firing, the slot is only marked inactive and
    /// skipped when the firing loop reaches it; it is removed from the list when
    /// the outermost `emit` returns.
    pub fn unsubscribe(&self, id: ConnectionId) -> bool {
        let mut inner = self.inner.borrow_mut();
        let Some(index) = inner.slots.iter().position(|slot| slot.id == id) else {
            return false;
        };
        if inner.firing > 0 {
            inner.slots[index].active = false;
        } else {
            inner.slots.remove(index);
        }
        true
    }

    /// Disconnect every subscription (`hkSignal::unsubscribeAll` / `reset`).
    pub fn unsubscribe_all(&self) {
        let mut inner = self.inner.borrow_mut();
        if inner.firing > 0 {
            for slot in &mut inner.slots {
                slot.active = false;
            }
        } else {
            inner.slots.clear();
        }
    }

    /// Number of live subscriptions (`hkSignal::getNumSubscriptions`).
    ///
    /// Slots already marked inactive count as disconnected even if their removal
    /// has been deferred.
    pub fn num_subscriptions(&self) -> usize {
        self.inner
            .borrow()
            .slots
            .iter()
            .filter(|slot| slot.active)
            .count()
    }

    /// Whether at least one live subscription exists.
    pub fn has_subscriptions(&self) -> bool {
        self.num_subscriptions() > 0
    }

    /// Whether an `emit` is currently in progress (including nested emits).
    pub fn is_firing(&self) -> bool {
        self.inner.borrow().firing > 0
    }

    /// Fire the signal (`hkSignalN::fire`).
    pub fn emit(&self, args: &A) {
        // Snapshot the number of slots. Slots appended during this firing (the
        // callback subscribed something) are *not* called in the same pass,
        // matching the C++ intrusive list where a new slot is prepended at the
        // head, behind the cursor.
        let limit = {
            let mut inner = self.inner.borrow_mut();
            if inner.slots.is_empty() {
                return;
            }
            inner.firing += 1;
            inner.slots.len()
        };

        let mut index = 0;
        loop {
            let step = {
                let inner = self.inner.borrow();
                if index >= limit {
                    Step::Done
                } else {
                    match inner.slots.get(index) {
                        None => Step::Done,
                        Some(slot) if !slot.active => Step::Skip,
                        Some(slot) => Step::Call(Rc::clone(&slot.callback)),
                    }
                }
            };

            match step {
                Step::Done => break,
                Step::Skip => index += 1,
                Step::Call(callback) => {
                    callback(args);
                    index += 1;
                }
            }
        }

        let mut inner = self.inner.borrow_mut();
        inner.firing -= 1;
        if inner.firing == 0 {
            inner.slots.retain(|slot| slot.active);
        }
    }
}

impl<A> Default for Signal<A> {
    fn default() -> Self {
        Self::new()
    }
}

impl<A> core::fmt::Debug for Signal<A> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Signal")
            .field("name", &self.name)
            .field("subscriptions", &self.num_subscriptions())
            .finish()
    }
}

impl Signal<()> {
    /// Zero-argument convenience form of [`Signal::subscribe`].
    pub fn subscribe_no_args<F>(&self, callback: F) -> ConnectionId
    where
        F: Fn() + 'static,
    {
        self.subscribe(move |_unit| callback())
    }

    /// Zero-argument convenience form of [`Signal::emit`].
    pub fn emit_no_args(&self) {
        self.emit(&());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;

    #[test]
    fn delivers_in_subscription_order() {
        let signal = Signal::<(i32,)>::new();
        let order = Rc::new(RefCell::new(Vec::new()));

        for expected in [1, 2, 3] {
            let order = Rc::clone(&order);
            signal.subscribe(move |args: &(i32,)| {
                assert_eq!(args.0, 7);
                order.borrow_mut().push(expected);
            });
        }

        signal.emit(&(7,));
        assert_eq!(*order.borrow(), vec![1, 2, 3]);
        assert_eq!(signal.num_subscriptions(), 3);
    }

    #[test]
    fn zero_argument_signal() {
        let signal = Signal::<()>::new();
        let hits = Rc::new(Cell::new(0));
        let hits2 = Rc::clone(&hits);
        signal.subscribe_no_args(move || hits2.set(hits2.get() + 1));
        signal.emit_no_args();
        signal.emit_no_args();
        assert_eq!(hits.get(), 2);
        assert_eq!(signal.num_subscriptions(), 1);
    }

    #[test]
    fn unsubscribe_outside_firing_removes_immediately() {
        let signal = Signal::<()>::new();
        let id = signal.subscribe_no_args(|| {});
        assert_eq!(signal.num_subscriptions(), 1);
        assert!(signal.unsubscribe(id));
        assert_eq!(signal.num_subscriptions(), 0);
        assert!(!signal.unsubscribe(id), "already removed");
        assert!(!signal.unsubscribe(12345), "unknown id");
    }

    #[test]
    fn unsubscribe_inside_callback_skips_later_slot() {
        let signal = Rc::new(Signal::<()>::new());
        let calls = Rc::new(Cell::new(0));
        let third_id: Rc<Cell<Option<ConnectionId>>> = Rc::new(Cell::new(None));

        {
            let signal_for_callback = Rc::clone(&signal);
            let calls = Rc::clone(&calls);
            let third_id = Rc::clone(&third_id);
            signal.subscribe_no_args(move || {
                calls.set(calls.get() + 1);
                if let Some(id) = third_id.get() {
                    assert!(signal_for_callback.unsubscribe(id));
                }
            });
        }
        {
            let calls = Rc::clone(&calls);
            signal.subscribe_no_args(move || calls.set(calls.get() + 1));
        }
        let id3 = {
            let calls = Rc::clone(&calls);
            signal.subscribe_no_args(move || calls.set(calls.get() + 1))
        };
        third_id.set(Some(id3));

        signal.emit_no_args();
        assert_eq!(calls.get(), 2, "the third slot must be skipped");
        assert_eq!(signal.num_subscriptions(), 2);
        assert_eq!(signal.inner.borrow().slots.len(), 2, "cleanup after emit");
    }

    #[test]
    fn unsubscribe_self_inside_callback() {
        let signal = Rc::new(Signal::<()>::new());
        let id_cell: Rc<Cell<Option<ConnectionId>>> = Rc::new(Cell::new(None));
        let calls = Rc::new(Cell::new(0));

        {
            let signal_for_callback = Rc::clone(&signal);
            let id_cell = Rc::clone(&id_cell);
            let calls = Rc::clone(&calls);
            signal.subscribe_no_args(move || {
                calls.set(calls.get() + 1);
                if let Some(id) = id_cell.get() {
                    signal_for_callback.unsubscribe(id);
                }
            });
        }
        let id = signal.subscribe_no_args(|| {});
        id_cell.set(Some(id));

        signal.emit_no_args();
        assert_eq!(calls.get(), 1);
        assert_eq!(signal.num_subscriptions(), 1, "second slot survives");
        assert!(!signal.is_firing());
    }

    #[test]
    fn nested_emit_is_allowed() {
        let signal = Rc::new(Signal::<()>::new());
        let depth = Rc::new(Cell::new(0));
        let calls = Rc::new(Cell::new(0));

        {
            let signal_for_callback = Rc::clone(&signal);
            let depth = Rc::clone(&depth);
            let calls = Rc::clone(&calls);
            signal.subscribe_no_args(move || {
                calls.set(calls.get() + 1);
                if depth.get() == 0 {
                    depth.set(1);
                    signal_for_callback.emit_no_args();
                    depth.set(0);
                }
            });
        }
        let calls2 = Rc::clone(&calls);
        signal.subscribe_no_args(move || calls2.set(calls2.get() + 1));

        signal.emit_no_args();
        assert_eq!(
            calls.get(),
            4,
            "inner emit runs slot1+slot2, outer runs slot1+slot2"
        );
        assert_eq!(signal.num_subscriptions(), 2);
    }

    #[test]
    fn subscribe_during_firing_is_deferred_to_the_next_fire() {
        let signal = Rc::new(Signal::<()>::new());
        let calls = Rc::new(Cell::new(0));

        {
            let signal_for_callback = Rc::clone(&signal);
            let calls_first = Rc::clone(&calls);
            signal.subscribe_no_args(move || {
                calls_first.set(calls_first.get() + 1);
                signal_for_callback.subscribe_no_args(|| {});
            });
        }
        let calls_second = Rc::clone(&calls);
        signal.subscribe_no_args(move || calls_second.set(calls_second.get() + 1));

        signal.emit_no_args();
        assert_eq!(calls.get(), 2, "new slot must not run in the current pass");
        assert_eq!(signal.num_subscriptions(), 3);
    }

    #[test]
    fn unsubscribe_all_during_firing() {
        let signal = Rc::new(Signal::<()>::new());
        let calls = Rc::new(Cell::new(0));

        {
            let signal_for_callback = Rc::clone(&signal);
            let calls = Rc::clone(&calls);
            signal.subscribe_no_args(move || {
                calls.set(calls.get() + 1);
                signal_for_callback.unsubscribe_all();
            });
        }
        let calls2 = Rc::clone(&calls);
        signal.subscribe_no_args(move || calls2.set(calls2.get() + 1));

        signal.emit_no_args();
        assert_eq!(calls.get(), 1);
        assert_eq!(signal.num_subscriptions(), 0);
    }
}
