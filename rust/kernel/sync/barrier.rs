// SPDX-License-Identifier: GPL-2.0

//! Memory barriers.
//!
//! These primitives have the same semantics as their C counterparts: and the precise definitions
//! of semantics can be found at [`LKMM`].
//!
//! [`LKMM`]: srctree/tools/memory-model/

/// Ordering requested from a memory barrier.
#[derive(Copy, Clone)]
pub enum BarrierKind {
    /// Orders preceding loads before succeeding loads.
    Read,

    /// Orders preceding stores before succeeding stores.
    Write,

    /// Orders all preceding memory accesses before all succeeding accesses.
    Full,
}

pub use BarrierKind::{
    Full,
    Read,
    Write,
};

/// A compiler barrier.
///
/// A barrier that prevents compiler from reordering memory accesses across the barrier.
#[inline(always)]
pub(crate) fn barrier() {
    // By default, Rust inline asms are treated as being able to access any memory or flags, hence
    // it suffices as a compiler barrier.
    //
    // SAFETY: An empty asm block.
    unsafe { core::arch::asm!("") };
}

/// Memory barrier.
///
/// This orders memory accesses according to `kind`.
///
/// - `mb(Read)` is equivalent to C `rmb()`.
/// - `mb(Write)` is equivalent to C `wmb()`.
/// - `mb(Full)` is equivalent to C `mb()`.
#[inline]
pub fn mb(kind: BarrierKind) {
    // SAFETY:
    // Memory barrier helpers are safe to invoke.
    unsafe {
        match kind {
            Read => bindings::rmb(),
            Write => bindings::wmb(),
            Full => bindings::mb(),
        }
    }
}

/// DMA memory barrier.
///
/// This orders accesses between the local CPU and bus-mastering devices.
///
/// - `dma_mb(Read)` is equivalent to C `dma_rmb()`.
/// - `dma_mb(Write)` is equivalent to C `dma_wmb()`.
/// - `dma_mb(Full)` is equivalent to C `dma_mb()`.
#[inline]
pub fn dma_mb(kind: BarrierKind) {
    // SAFETY:
    // DMA memory barrier helpers are safe to invoke.
    unsafe {
        match kind {
            Read => bindings::dma_rmb(),
            Write => bindings::dma_wmb(),
            Full => bindings::dma_mb(),
        }
    }
}

/// A full memory barrier.
///
/// A barrier that prevents compiler and CPU from reordering memory accesses across the barrier.
#[inline(always)]
pub fn smp_mb() {
    if cfg!(CONFIG_SMP) {
        // SAFETY: `smp_mb()` is safe to call.
        unsafe { bindings::smp_mb() };
    } else {
        barrier();
    }
}

/// A write-write memory barrier.
///
/// A barrier that prevents compiler and CPU from reordering memory write accesses across the
/// barrier.
#[inline(always)]
pub fn smp_wmb() {
    if cfg!(CONFIG_SMP) {
        // SAFETY: `smp_wmb()` is safe to call.
        unsafe { bindings::smp_wmb() };
    } else {
        barrier();
    }
}

/// A read-read memory barrier.
///
/// A barrier that prevents compiler and CPU from reordering memory read accesses across the
/// barrier.
#[inline(always)]
pub fn smp_rmb() {
    if cfg!(CONFIG_SMP) {
        // SAFETY: `smp_rmb()` is safe to call.
        unsafe { bindings::smp_rmb() };
    } else {
        barrier();
    }
}
