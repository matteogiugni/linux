// SPDX-License-Identifier: GPL-2.0

//! Hardware Random Number Generator abstractions.
//!
//! C header: [`include/linux/hw_random.h`](srctree/include/linux/hw_random.h)

// miscdevice.rs come astrazione da seguire

use crate::{
    bindings,
    error::{to_result, Error, Result, VTABLE_DEFAULT_ERROR},
    prelude::*,
    str::CStr,
    types::Opaque,
};
use core::marker::PhantomData;

/// Options for creating a hwrng registration.
#[derive(Copy, Clone)]
pub struct HwRngOptions {
    /// The name of the hwrng, shown in `/sys/class/misc/hw_random/rng_available`.
    pub name: &'static CStr,
    /// Estimated entropy in bits per 1024 bits of output.
    /// Valid values are 0..=1024.
    pub quality: u16,
}

impl HwRngOptions {
    /// Create a raw `struct hwrng` ready for registration.
    pub const fn into_raw<T: HwRng + 'static>(self) -> bindings::hwrng {
        // Start from the C default layout and initialize the fields required
        // for registration.
        debug_assert!(self.quality <= 1024);
        let mut result: bindings::hwrng = pin_init::zeroed();
        result.name = crate::str::as_char_ptr_in_const_context(self.name);
        result.quality = self.quality;
        result.read = Some(HwRngVTable::<T>::read);
        if T::HAS_INIT {
            result.init = Some(HwRngVTable::<T>::init);
        }
        result.cleanup = Some(HwRngVTable::<T>::cleanup);
        
        result
    }
}

/// Trait implemented by drivers that provide hardware random data.
///
/// Use `#[vtable]` on the `impl` block to generate `HAS_*` constants that
/// allow the framework to install `NULL` for unimplemented optional callbacks.
#[vtable]
pub trait HwRng: Send + Sync {
    /// Optional — called once before the first read.
    fn init(_this: &Self) -> Result {
        build_error!(VTABLE_DEFAULT_ERROR)
    }

    /// Optional — called when the last reader closes `/dev/hwrng`.
    fn cleanup(_this: &Self) {}

    /// Required — fill `buf` with random bytes.
    ///
    /// If `wait` is `true` the call may block until data is available.
    /// Returns the number of bytes written, or an error.
    fn read(this: &Self, buf: &mut [u8], wait: bool) -> Result<usize>;
}

/// VTable holding the C callbacks for a [`HwRng`] implementation.
struct HwRngVTable<T>(PhantomData<T>);

impl<T: HwRng + 'static> HwRngVTable<T> {
    #[inline]
    fn driver(rng: *mut bindings::hwrng) -> &'static T {
        // SAFETY:
        // `priv_` was initialized by `Registration::register` with a pointer
        // to the driver instance and remains valid while the hwrng is registered.
        unsafe { &*((*rng).priv_ as *const T) }
    }

    unsafe extern "C" fn init(rng: *mut bindings::hwrng) -> core::ffi::c_int {
        // SAFETY: `rng` is provided by the hwrng subsystem and contains a valid
        // pointer to the registered driver in `priv_`.
        let driver = Self::driver(rng);
        
        match T::init(driver) {
            Ok(()) => 0,
            Err(e) => e.to_errno(),
        }
    }

    unsafe extern "C" fn cleanup(rng: *mut bindings::hwrng) {
        // SAFETY: `priv_` was set in `HwRngRegistration::register` to a valid `*const T`.
        let driver = Self::driver(rng);
        T::cleanup(driver);
    }

    unsafe extern "C" fn read(
        rng: *mut bindings::hwrng,
        data: *mut core::ffi::c_void,
        max: usize,
        wait: bindings::bool_,
    ) -> core::ffi::c_int {
        // SAFETY: `priv_` was set in `HwRngRegistration::register` to a valid `*const T`.
        let driver = Self::driver(rng);

        // SAFETY: `data` is valid for `max` bytes as guaranteed by the hwrng subsystem.
        let buf = unsafe { core::slice::from_raw_parts_mut(data as *mut u8, max) };

        match T::read(driver, buf, wait) {
            Ok(n) => match i32::try_from(n) {
                Ok(n) => n,
                Err(_) => i32::MAX,
            },
            Err(e) => e.to_errno(),
        }
    }
}

/// A registration of a hardware RNG with the kernel hwrng subsystem.
///
/// Automatically unregisters the hardware RNG when dropped.
///
/// # Invariants
///
/// - `hw_rng` contains a `struct hwrng` successfully registered via `hwrng_register`.
/// - `hw_rng` remains registered for the lifetime of this object.
/// - The instance referenced by `priv_` outlives this registration.
/// - `priv_` stores the pointer passed to `register` and is never modified while registered.
/// - Deregistration occurs exactly once in [`Drop`] via `hwrng_unregister`.
#[repr(transparent)]
#[pin_data(PinnedDrop)]
pub struct HwRngRegistration<T: HwRng + 'static> {
    #[pin]
    hw_rng: Opaque<bindings::hwrng>,
    _t: PhantomData<T>,
}

// SAFETY: The registration does not have thread affinity, and moving it
// to another thread does not affect its correctness.
unsafe impl<T: HwRng + 'static> Send for HwRngRegistration<T> {}

// SAFETY: All `&self` methods are thread-safe.
unsafe impl<T: HwRng + 'static> Sync for HwRngRegistration<T> {}

impl<T: HwRng + 'static> HwRngRegistration<T> {
    /// Register a hardware RNG.
    ///
    /// `opts` contains the name and quality of the RNG.
    /// `driver` is a raw pointer to the driver implementing [`HwRng`],
    /// stored in `priv_` and used by the vtable callbacks.
    ///
    /// # Safety
    ///
    /// `driver` must point to a valid instance of `T`.
    /// - The pointed object must remain alive until this registration is dropped.
    /// - The pointed object must not move after registration because the hwrng
    ///   subsystem may invoke the callbacks at any time.
    pub fn register(opts: HwRngOptions, driver: *const T) -> impl PinInit<Self, Error> {
        try_pin_init!(Self {
            hw_rng <- Opaque::try_ffi_init(move |slot: *mut bindings::hwrng| {
                // SAFETY: The initializer can write to the provided `slot`.
                unsafe { slot.write(opts.into_raw::<T>()) };

                // SAFETY: `slot` was just written above, it is valid.
                unsafe { (*slot).priv_ = driver as usize };

                // SAFETY: `slot` contains a valid, initialized `struct hwrng`.
                // INVARIANT: If this returns Ok(()), `slot` contains a registered hwrng.
                to_result(unsafe { bindings::hwrng_register(slot) })
            }),
            _t: PhantomData,
        })
    }

    /// Returns a raw pointer to the underlying `struct hwrng`.
    pub fn as_raw(&self) -> *mut bindings::hwrng {
        self.hw_rng.get()
    }
}

#[pinned_drop]
impl<T: HwRng + 'static> PinnedDrop for HwRngRegistration<T> {
    fn drop(self: Pin<&mut Self>) {
        // SAFETY: By the type invariants, `hw_rng` is registered exactly once
        // and has not yet been unregistered.
        unsafe { bindings::hwrng_unregister(self.hw_rng.get()) };
    }
}