// SPDX-License-Identifier: GPL-2.0

//! VIRTIO abstraction.
//!
//! To implement a VIRTIO driver:
//!
//! - Implement the [`Driver`] trait for your driver type (use [`virtio_device_table`] macro to
//!   declare the `ID_TABLE` associated item)
//! - Use the [`module_virtio_driver`] macro to declare your module

use crate::{
    bindings,
    device_id::RawDeviceId,
    error::{
        code::{
            EINVAL,
            ENOTSUPP,
        },
        from_result,
        to_result,
        Error,
        Result, //
    },
    ffi::c_uint,
    prelude::*,
    sync::aref::ARef,
    types::Opaque, //
};

use core::{
    cell::UnsafeCell,
    marker::PhantomData,
    pin::Pin,
    ptr::NonNull, //
};

use self::virtqueue::{LinuxHal, VIRTIO_RING_F_EVENT_IDX, VIRTIO_RING_F_INDIRECT_DESC};

/// Utilities for VIRTIO.
pub mod utils;
/// VirtQueue implementation for Linux.
pub mod virtqueue;

/// IdTable type for virtio drivers.
pub type IdTable<T> = &'static dyn crate::device_id::IdTable<DeviceId, T>;

/// A VIRTIO device id.
///
/// [`struct virtio_device_id`]: srctree/include/linux/mod_devicetable.h
#[repr(transparent)]
#[derive(Clone, Copy)]
pub struct DeviceId(bindings::virtio_device_id);

// SAFETY: `DeviceId` is a `#[repr(transparent)]` wrapper of `struct virtio_device_id` and
// does not add additional invariants, so it's safe to transmute to `RawType`.
unsafe impl RawDeviceId for DeviceId {
    type RawType = bindings::virtio_device_id;
}

impl DeviceId {
    #[inline]
    /// Create a new device id
    pub const fn new(device: VirtioID) -> Self {
        Self::new_with_vendor(device, VIRTIO_DEV_ANY_ID)
    }

    #[inline]
    /// Create a new device id with vendor
    pub const fn new_with_vendor(device: VirtioID, vendor: u32) -> Self {
        // Replace with `bindings::virtio_device_id::default()` once stabilized for `const`.
        // SAFETY: FFI type is valid to be zero-initialized.
        let mut ret: bindings::virtio_device_id = unsafe { core::mem::zeroed() };
        ret.device = device as u32;
        ret.vendor = vendor;
        Self(ret)
    }

    /// Returns a reference to the underlying `bindings::virtio_device_id` type.
    #[inline]
    pub const fn as_raw(&self) -> &bindings::virtio_device_id {
        &self.0
    }

    /// Consumes `Self` and returns the underlying `bindings::virtio_device_id` type.
    #[inline]
    pub const fn into_raw(self) -> bindings::virtio_device_id {
        self.0
    }
}

impl From<bindings::virtio_device_id> for DeviceId {
    fn from(id: bindings::virtio_device_id) -> Self {
        Self(id)
    }
}

/// Create a virtio `IdTable` with its alias for modpost.
#[macro_export]
macro_rules! virtio_device_table {
    ($table_name:ident, $module_table_name:ident, $id_info_type: ty, $table_data:expr) => {
        const $table_name: $crate::device_id::IdArray<
            $crate::virtio::DeviceId,
            $id_info_type,
            { $table_data.len() },
        > = $crate::device_id::IdArray::new_without_index($table_data);

        $crate::module_device_table!("virtio", $module_table_name, $table_name);
    };
}

/// Declares a kernel module that exposes a single virtio driver.
#[macro_export]
macro_rules! module_virtio_driver {
($($f:tt)*) => {
    $crate::module_driver!(<T>, $crate::virtio::Adapter<T>, { $($f)* });
};
}

/// The Virtio driver trait.
///
/// Drivers must implement this trait in order to get a virtio driver registered.
pub trait Driver: Send {
    /// The type holding information about each device id supported by the driver.
    // TODO: Use `associated_type_defaults` once stabilized:
    //
    // ```
    // type IdInfo: 'static = ();
    // ```
    type IdInfo: 'static;

    /// The table of device ids supported by the driver.
    const ID_TABLE: IdTable<Self::IdInfo>;

    /// virtio driver probe.
    ///
    /// Called when a new virtio device is added or discovered. Implementers should
    /// attempt to initialize the device here, but should try not sleep since driver data is set
    /// after this method returns successfully.
    fn probe(dev: &Device<crate::device::Core>) -> impl PinInit<Self, Error>;

    /// virtio driver init.
    ///
    /// Called after a virtio device is probed successfully, can sleep.
    fn init(&self, dev: &Device<crate::device::Bound>) -> Result;

    /// virtio driver remove.
    ///
    /// Called when a [`Device`] is removed from its [`Driver`]. Implementing this callback
    /// is optional.
    ///
    /// This callback serves as a place for drivers to perform teardown operations that require a
    /// `&Device<Core>` or `&Device<Bound>` reference. For instance, drivers may try to perform I/O
    /// operations to gracefully tear down the device.
    ///
    /// Otherwise, release operations for driver resources should be performed in `Self::drop`.
    fn remove(dev: &Device<crate::device::Core>, this: Pin<&Self>) {
        _ = (dev, this);
    }

    /// Called after probe to register the device with subsystems (Optional)
    fn scan(_dev: &Device<crate::device::Bound>, _this: Pin<&Self>) {}
}

/// Abstraction for the virtio device structure (`struct virtio_device`).
///
/// [`struct virtio_device`]: srctree/include/linux/virtio.h
#[repr(transparent)]
pub struct Device<Ctx: crate::device::DeviceContext = crate::device::Normal>(
    Opaque<bindings::virtio_device>,
    PhantomData<Ctx>,
);

impl<Ctx: crate::device::DeviceContext> Device<Ctx> {
    /// Returns a reference to the underlying `bindings::virtio_device` type.
    #[inline]
    fn as_raw(&self) -> *mut bindings::virtio_device {
        self.0.get()
    }

    /// Consumes `Self` and returns the underlying `bindings::virtio_device_id` type.
    #[inline]
    fn raw_device(&self) -> *mut bindings::device {
        // SAFETY: By the type invariant of `Self`, `self.as_raw()` is a pointer to a valid
        // `struct virtio_device`. 
        unsafe { core::ptr::addr_of_mut!((*self.as_raw()).dev) }
    }
}

// SAFETY: `virtio::Device` is a transparent wrapper of `struct virtio_device`.
// The offset is guaranteed to point to a valid device field inside `virtio::Device`.
unsafe impl<Ctx: crate::device::DeviceContext> crate::device::AsBusDevice<Ctx> for Device<Ctx> {
    const OFFSET: usize = core::mem::offset_of!(bindings::virtio_device, dev);
}

// SAFETY: `Device` is a transparent wrapper of a type that doesn't depend on `Device`'s generic
// argument.
kernel::impl_device_context_deref!(unsafe { Device });

/*
TODO OPTIMIZATION
kernel::impl_device_context_into_aref!(Device);

unsafe impl crate::sync::aref::AlwaysRefCounted for Device {
    fn inc_ref(&self) {
        // SAFETY:
        // `raw_device()` points to the embedded struct device and
        // the existence of `&self` guarantees a live reference.
        unsafe {
            bindings::get_device(self.raw_device());
        }
    }

    unsafe fn dec_ref(obj: NonNull<Self>) {
        let vdev = obj.cast::<bindings::virtio_device>().as_ptr();

        // SAFETY:
        // `vdev` is valid while this reference is owned.
        let dev = unsafe {
            core::ptr::addr_of_mut!((*vdev).dev)
        };

        unsafe {
            bindings::put_device(dev);
        }
    }
}*/

impl<Ctx: crate::device::DeviceContext> Device<Ctx> {
    /// Returns the `DeviceId` associated with this VirtIO device.
    #[inline]
    pub fn id(&self) -> DeviceId {
        // SAFETY: 
        unsafe { (*self.as_raw()).id.into() }
    }

    // TODO: return VirtioID
    /// Returns the virtio device ID.
    #[inline]
    pub fn device_id(&self) -> u32 {
        // SAFETY: By its type invariant `self.as_raw` is always a valid pointer to a
        // `struct virtio_device`.
        unsafe { (*self.as_raw()).id.device }
    }

    /// Returns the virtio vendor ID.
    #[inline]
    pub fn vendor_id(&self) -> u32 {
        // SAFETY: `self.as_raw` is a valid pointer to a `struct virtio_device`.
        unsafe { (*self.as_raw()).id.vendor }
    }

    /// Reset device.
    #[doc(alias = "virtio_reset_device")]
    #[inline]
    pub fn reset(&self) {
        // SAFETY: By its type invariant `self.as_raw` is always a valid pointer to a
        // `struct virtio_device`.
        unsafe { bindings::virtio_reset_device(self.as_raw()) }
    }

    /// Mark device as ready.
    #[doc(alias = "virtio_device_ready")]
    #[inline]
    pub fn device_ready(&self) {
        // SAFETY: By its type invariant `self.as_raw` is always a valid pointer to a
        // `struct virtio_device`.
        unsafe { bindings::virtio_device_ready(self.as_raw()) }
    }

    /// Checks if the device has a feature bit.
    #[inline]
    pub fn has_feature(&self, fbit: c_uint) -> bool {
        // SAFETY: By its type invariant `self.as_raw` is always a valid pointer to a
        // `struct virtio_device`.
        unsafe { bindings::virtio_has_feature(self.as_raw(), fbit) }
    }

    /// Returns the DMA device associated with this VirtIO device.
    pub fn dma_device(
        &self,
    ) -> Result<&crate::device::Device<crate::device::Bound>> {
        let parent = self
            .as_ref()
            .parent()
            .ok_or(ENODEV)?;

        // SAFETY:
        // A virtio_device exists while its transport device is bound.
        // The parent is the transport device backing this VirtIO device
        // and remains alive for the lifetime of the VirtIO device.
        Ok(unsafe {
            crate::device::Device::<crate::device::Bound>::from_raw(
                parent.as_raw(),
            )
        })
    }
}

impl Device<crate::device::Core> {
    /// Return virtqueues for this device.
    #[doc(alias = "virtio_find_vqs")]
    pub fn find_vqs(
        &self,
        info: &[VirtqueueInfo],
    ) -> Result<Virtqueues> {
        // SAFETY:
        // By the Device type invariant, self.as_raw() points to a valid
        // struct virtio_device and its config pointer is valid.
        let config = unsafe {
            &*(*self.as_raw()).config
        };


        // CALL 1: what happens before creating the virtqueues.
        let prepare = config
            .prepare_rust_vqs
            .ok_or(ENOTSUPP)?;

        // CALL 2: what happens after creating the virtqueues.
        let setup = config
            .setup_rust_vqs
            .ok_or(ENOTSUPP)?;

        // Teardown support is required if setup succeeds.
        config
            .del_rust_vqs
            .ok_or(ENOTSUPP)?;

        /*
        * Build the descriptions passed to the transport.
        *
        * At this point only the driver-provided queue name is known.
        */
        let mut configs =
            KVec::with_capacity(info.len(), GFP_KERNEL)?;

        for vqi in info {
            //TODO context support
            // Context support is not implemented by the Rust virtqueue yet.
            if vqi.ctx {
                return Err(ENOTSUPP);
            }

            configs.push(
                bindings::virtio_rust_vq_info {
                    name: vqi.name.as_char_ptr(),

                    // Filled by prepare_rust_vqs().
                    index: 0,
                    size: 0,

                    // Filled after constructing the Rust VirtQueue.
                    desc_addr: 0,
                    avail_addr: 0,
                    used_addr: 0,

                    // Filled after every Virtqueue has reached its final address.
                    interrupt: None,
                    interrupt_data: core::ptr::null_mut(),

                    // Defined by setup_rust_vqs().
                    notify: None,
                    notify_data: core::ptr::null_mut(),
                },
                GFP_KERNEL,
            )?;
        }

        /*
        * CALL 1.
        *
        * Transport discovery only: obtain hardware queue index and size.
        * This call must not enable queues or install IRQ state.
        */
        to_result(unsafe {
            prepare(
                self.as_raw(),
                configs.len().try_into()?,
                configs.as_mut_ptr(),
            )
        })?;

        let event_idx =
            self.has_feature(VIRTIO_RING_F_EVENT_IDX);

        let indirect =
            self.has_feature(VIRTIO_RING_F_INDIRECT_DESC);

        /*
        * Create all Rust virtqueues.
        *
        * Capacity is fixed to the final number of queues. Once this loop
        * succeeds, `inner` is never resized, so pointers to its Virtqueue
        * elements remain stable until transport teardown.
        */
        let mut inner =
            KVec::with_capacity(info.len(), GFP_KERNEL)?;

        for (cfg, vqi) in configs.iter().zip(info.iter()) {
            let hal = LinuxHal::new(self)?;

            let queue = virtqueue::VirtQueue::new(
                hal,
                cfg.index.try_into()?,
                cfg.size,
                virtqueue::VirtQueueFeatures {
                    event_idx,
                    indirect,
                },
            )
            .map_err(Self::map_virtqueue_error)?;

            inner.push(
                VirtQueue {
                    inner: UnsafeCell::new(queue),

                    // SAFETY:
                    // Device::as_raw() is non-null by the Device type invariant.
                    vdev: unsafe {
                        NonNull::new_unchecked(self.as_raw())
                    },

                    callback: vqi.callback,
                    // Transport state is installed only after setup_rust_vqs()
                    // completes successfully.
                    notify: None,
                    notify_data: core::ptr::null_mut(),
                },
                GFP_KERNEL,
            )?;
        }

        /*
        * Every Virtqueue now has its final address.
        *
        * Fill in the ring DMA addresses and the Rust equivalent of
        * vring_interrupt().
        */
        for i in 0..inner.len() {
            {
                let core_vq =
                    inner[i].inner.get_mut();

                configs[i].desc_addr =
                    core_vq
                        .descriptor_dma_addr()
                        .map_err(|_| EINVAL)?;

                configs[i].avail_addr =
                    core_vq
                        .driver_area_dma_addr()
                        .map_err(|_| EINVAL)?;

                configs[i].used_addr =
                    core_vq
                        .device_area_dma_addr()
                        .map_err(|_| EINVAL)?;
            }

            if inner[i].callback.is_some() {
                let vq =
                    core::ptr::from_mut(&mut inner[i]);

                configs[i].interrupt =
                    Some(rust_vring_interrupt);

                configs[i].interrupt_data =
                    vq.cast();
            }
        }

        /*
        * CALL 2.
        *
        * The transport may now configure MSI-X/INTx state, program the
        * ring addresses and enable the queues.
        *
        * On failure, setup_rust_vqs() must completely roll back any
        * transport resources it created. `inner` is then dropped normally.
        */
        let mut transport_data =
            core::ptr::null_mut();

        to_result(unsafe {
            setup(
                self.as_raw(),
                configs.len().try_into()?,
                configs.as_mut_ptr(),
                core::ptr::null_mut(), //TODO irq_affinity unsupported for now
                &mut transport_data,
            )
        })?;

        /*
        * CALL 2 succeeded.
        *
        * The transport state is now complete and owns the per-VQ notify
        * mappings. Publish the non-owning notify handles into the Rust
        * wrappers.
        *
        * Nothing fallible should happen after this point.
        */
        for i in 0..inner.len() {
            inner[i].notify = configs[i].notify;
            inner[i].notify_data = configs[i].notify_data;
        }

        Ok(Virtqueues {
            inner,
            transport_data,

            // SAFETY:
            // Device::as_raw() is non-null by its type invariant.
            vdev: unsafe {
                NonNull::new_unchecked(self.as_raw())
            },

            _device: unsafe {
                // SAFETY:
                // `self.raw_device()` points to the embedded `struct device`
                // of this live VirtIO device. Since `self` is alive here, its
                // device reference count is non-zero. `get_device()` acquires
                // an independent reference which is held by `Virtqueues`.
                crate::device::Device::get_device(
                    self.raw_device(),
                )
            },
        })
    }

    fn map_virtqueue_error(
        err: virtqueue::Error<Error>,
    ) -> Error {
        match err {
            virtqueue::Error::Hal(err) => err,
            virtqueue::Error::Queue(_) => EINVAL,
        }
    }
}

impl<Ctx: crate::device::DeviceContext> AsRef<crate::device::Device<Ctx>> for Device<Ctx> {
    #[inline]
    fn as_ref(&self) -> &crate::device::Device<Ctx> {
        // SAFETY: `dev` points to a valid `struct device`.
       unsafe { crate::device::Device::from_raw(self.raw_device()) }
    }
}

// SAFETY: `virtio::Device<Core>` provides access to a valid DMA-capable device.
impl crate::dma::Device for Device<crate::device::Core> {}

/// An adapter for the registration of virtio drivers.
pub struct Adapter<T: Driver>(T);

// SAFETY:
// - `bindings::virtio_driver` is a C type declared as `repr(C)`.
// - `T` is the type of the driver's device private data.
// - `struct virtio_driver` embeds a `struct device_driver`.
// - `DEVICE_DRIVER_OFFSET` is the correct byte offset to the embedded `struct device_driver`.
unsafe impl<T: Driver + 'static> crate::driver::DriverLayout for Adapter<T> {
    type DriverType = bindings::virtio_driver;
    type DriverData = T;
    const DEVICE_DRIVER_OFFSET: usize = core::mem::offset_of!(Self::DriverType, driver);
}

// SAFETY: A call to `unregister` for a given instance of `DriverType` is guaranteed to be valid if
// a preceding call to `register` has been successful.
unsafe impl<T: Driver + 'static> crate::driver::RegistrationOps for Adapter<T> {
    unsafe fn register(
        vdrv: &Opaque<Self::DriverType>,
        name: &'static CStr,
        module: &'static ThisModule,
    ) -> Result {
        // SAFETY: It's safe to set the fields of `struct virtio_driver` on initialization.
        unsafe {
            (*vdrv.get()).driver.name = name.as_char_ptr();
            (*vdrv.get()).id_table = T::ID_TABLE.as_ptr();
            (*vdrv.get()).probe = Some(Self::probe_callback);
            (*vdrv.get()).remove = Some(Self::remove_callback);
            (*vdrv.get()).scan = Some(Self::scan_callback);
        }

        // SAFETY: `vdrv` is guaranteed to be a valid `DriverType`.
        to_result(unsafe { bindings::__register_virtio_driver(vdrv.get(), module.0) })
    }

    unsafe fn unregister(vdrv: &Opaque<Self::DriverType>) {
        // SAFETY: `vdrv` is guaranteed to be a valid `DriverType`.
        unsafe { bindings::unregister_virtio_driver(vdrv.get()) }
    }
}

impl<T: Driver + 'static> Adapter<T> {
    extern "C" fn probe_callback(vdev: *mut bindings::virtio_device) -> c_int {
        // SAFETY: The kernel only ever calls the probe callback with a valid pointer to a `struct
        // virtio_device`.
        //
        // INVARIANT: `vdev` is valid for the duration of `probe_callback()`.
        let dev = unsafe { &*vdev.cast::<Device<crate::device::CoreInternal>>() };
        from_result(|| {
            let data = T::probe(dev);

            dev.as_ref().set_drvdata(data)?;
            // SAFETY: `Device::set_drvdata()` was just called so it's safe to borrow the data.
            let data = unsafe { dev.as_ref().drvdata_borrow::<T>() };
            dev.device_ready();
            if let Err(err) = T::init(&data, dev) {
                // SAFETY: `Device::set_drvdata()` was just called so it's safe to re-obtain the
                // data.
                let data = unsafe { dev.as_ref().drvdata_obtain::<T>() }.unwrap();
                T::remove(dev, data.as_ref());
                drop(data);
                return Err(err);
            }
            Ok(0)
        })
    }

    extern "C" fn remove_callback(vdev: *mut bindings::virtio_device) {
        // SAFETY: The kernel only ever calls the remove callback with a valid pointer to a `struct
        // virtio_device`.
        //
        // INVARIANT: `vdev` is valid for the duration of `remove_callback()`.
        let dev = unsafe { &*vdev.cast::<Device<crate::device::CoreInternal>>() };

        // SAFETY: `remove_callback` is only ever called after a successful call to
        // `probe_callback`, hence it's guaranteed that `Device::set_drvdata()` has been called
        // and stored a `Pin<KBox<T>>`.
        let data = unsafe { dev.as_ref().drvdata_borrow::<T>() };

        T::remove(dev, data);
        dev.reset();
    }

    extern "C" fn scan_callback(vdev: *mut bindings::virtio_device) {
        // SAFETY:
        let dev = unsafe { &*vdev.cast::<Device<crate::device::CoreInternal>>() };

        // SAFETY:
        let data = unsafe { dev.as_ref().drvdata_borrow::<T>() };

        T::scan(dev, data);
    }
}

/// Virtqueue callback function for interrupt handling.
pub type VirtqueueCallback = fn(&VirtQueue);

/// A struct to hold the information needed to create a virtqueue.
pub struct VirtqueueInfo {
    pub(crate) name: &'static CStr,
    pub(crate) ctx: bool,
    pub(crate) callback: Option<VirtqueueCallback>,
}

impl VirtqueueInfo {
    /// Create a new virtqueue info struct.
    pub const fn new(
        name: &'static CStr,
        ctx: bool,
        callback: Option<VirtqueueCallback>,
    ) -> Self {
        Self {
            name,
            ctx,
            callback,
        }
    }
}

/// A function that is called when a virtqueue needs to be notified.
type VirtqueueNotify =
    unsafe extern "C" fn(*mut core::ffi::c_void, u32) -> bool;

/// Linux-specific VirtIO virtqueue.
///
/// `inner` contains the portable Rust virtqueue implementation, while
/// this wrapper stores the Linux VirtIO state associated with it.
pub struct VirtQueue {
    inner: UnsafeCell<virtqueue::VirtQueue<LinuxHal>>,
    callback: Option<VirtqueueCallback>,
    vdev: NonNull<bindings::virtio_device>,
    notify: Option<VirtqueueNotify>,
    notify_data: *mut core::ffi::c_void,
}

impl VirtQueue {
    /// Returns a reference to the underlying `bindings::virtio_device` type.
    #[inline]
    pub fn dev(&self) -> &Device<crate::device::Bound> {
        // SAFETY:
        // The queue belongs to this VirtIO device. Callbacks can only be
        // delivered while the device and the queue are alive.
        unsafe {
            &*self
                .vdev
                .as_ptr()
                .cast::<Device<crate::device::Bound>>()
        }
    }

    #[inline]
    fn with_inner<R>(
        &self,
        f: impl FnOnce(&virtqueue::VirtQueue<LinuxHal>) -> R,
    ) -> R {
        // SAFETY:
        // VirtQueue operations follow the virtqueue no-reentry
        // requirement. Shared access is limited to this call.
        unsafe { f(&*self.inner.get()) }
    }

    #[inline]
    fn with_inner_mut<R>(
        &self,
        f: impl FnOnce(&mut virtqueue::VirtQueue<LinuxHal>) -> R,
    ) -> R {
        // SAFETY:
        // VirtQueue operations follow the virtqueue no-reentry
        // requirement. The mutable reference does not escape this call.
        unsafe { f(&mut *self.inner.get()) }
    }

    #[inline]
    fn is_broken(&self) -> bool {
        self.with_inner(|inner| inner.is_broken())
    }

    #[inline]
    fn can_pop(&self) -> bool {
        self.with_inner(|inner| inner.can_pop())
    }

    fn notify_with_data(
        &self,
        notification_data: u32,
    ) -> bool {
        if self.is_broken() {
            return false;
        }

        let Some(notify) = self.notify else {
            return false;
        };

        if !unsafe {
            notify(self.notify_data, notification_data)
        } {
            self.with_inner_mut(|inner| {
                inner.mark_broken();
            });

            return false;
        }

        true
    }

    /// Notify the device that new buffers are available.
    pub fn notify(&self) -> bool {
        let notification_data =
            self.with_inner(|inner| {
                inner.notification_data()
            });

        self.notify_with_data(notification_data)
    }

    /// Notify the device that new buffers are available, but only if the device has used some buffers.
    pub fn kick(&self) -> bool {
        /*
        let notification_data =
            self.with_inner_mut(|inner| {
                if inner.kick_prepare() {
                    Some(inner.notification_data())
                } else {
                    None
                }
            });

        let Some(notification_data) =
            notification_data
        else {
            return true;
        };

        self.notify_with_data(notification_data)
        */
        //TODO DELETE FROM HERE ON AFTER THE TESTS and uncomment above
        let (
            avail_idx,
            avail_event,
            used_idx,
            last_used_idx,
            needs_kick,
            notification_data,
        ) = self.with_inner_mut(|inner| {
            let avail_idx = inner.debug_avail_idx();
            let avail_event = inner.debug_avail_event();
            let used_idx = inner.debug_used_idx();
            let last_used_idx = inner.debug_last_used_idx();

            let needs_kick = inner.kick_prepare();
            let notification_data =
                inner.notification_data();

            (
                avail_idx,
                avail_event,
                used_idx,
                last_used_idx,
                needs_kick,
                notification_data,
            )
        });

        pr_info!(
            "vq: avail={} avail_event={} used={} last_used={} kick={}\n",
            avail_idx,
            avail_event,
            used_idx,
            last_used_idx,
            needs_kick
        );

        if !needs_kick {
            pr_info!("virtqueue: MMIO notify suppressed\n");
            return true;
        }

        pr_info!("virtqueue: doing MMIO notify\n");

        self.notify_with_data(notification_data)
    }

    //TODO we may want the driver to pass any address and then DMA map it here
    /// Adds one device-writable DMA buffer.
    ///
    /// # Safety
    ///
    /// `dma_addr..dma_addr + len` must remain a valid DMA mapping for this
    /// device until the request is returned by `get_buf()`.
    pub unsafe fn add_inbuf(
        &self,
        dma_addr: crate::dma::DmaAddress,
        len: u32,
        token: virtqueue::Token,
    ) -> Result {
        self.with_inner_mut(|inner| {
            // SAFETY:
            // Forwarded from this function's safety requirements.
            unsafe {
                inner.add_inbuf(
                    dma_addr,
                    len,
                    token,
                    virtqueue::DescriptorLayout::Direct,
                )
            }
        })
        .map_err(|err| match err {
            virtqueue::Error::Hal(err) => err,
            virtqueue::Error::Queue(_) => EIO,
        })
    }

    /// Gets one used buffer from the device.
    pub fn get_buf(
        &self,
    ) -> Result<Option<(virtqueue::Token, u32)>> {
        self.with_inner_mut(|inner| {
            inner.get_buf()
        })
        .map_err(|_| EIO)
    }
}

/// A collection of virtqueues for a given virtio device.
pub struct Virtqueues {
    inner: KVec<VirtQueue>,

    transport_data: *mut core::ffi::c_void,

    vdev: NonNull<bindings::virtio_device>,

    // Keeps the underlying device alive until all queues have been destroyed.
    _device: ARef<crate::device::Device>,

    //TODO OPTIMIZATION make virtio device ARef counted
    // device: ARef<Device>,
}

impl core::ops::Deref for Virtqueues {
    type Target = [VirtQueue];

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl Virtqueues {
    /// Get the virtqueue at the given index.
    #[inline]
    pub fn get(&self, index: usize) -> Option<&VirtQueue> {
        self.inner.get(index)
    }

    /// Get the mutable virtqueue at the given index.
    #[inline]
    pub fn get_mut(
        &mut self,
        index: usize,
    ) -> Option<&mut VirtQueue> {
        self.inner.get_mut(index)
    }

    /// Get the number of virtqueues in this collection.
    #[inline]
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Returns `true` if the collection is empty, `false` otherwise.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }
}

impl Drop for Virtqueues {
    fn drop(&mut self) {
        let vdev = self.vdev.as_ptr();

        pr_info!("virtio: Virtqueues::drop: reset begin\n");
        
        // Stop device DMA/interrupt generation first.
        unsafe {
            bindings::virtio_reset_device(vdev);
        }

        pr_info!("virtio: Virtqueues::drop: reset done\n");

        let config = unsafe { (*vdev).config };

        if let Some(del_rust_vqs) =
            unsafe { (*config).del_rust_vqs }
        {

            pr_info!(
                "virtio: Virtqueues::drop: transport teardown begin\n"
            );
            // SAFETY:
            // transport_data was returned by setup_rust_vqs() for this device.
            // Virtqueue objects are still alive while transport IRQ state is
            // being destroyed.
            unsafe {
                del_rust_vqs(
                    vdev,
                    self.transport_data,
                );
            }

            pr_info!(
                "virtio: Virtqueues::drop: transport teardown done\n"
            );
        }

        //TODO tokens teardown
        
        pr_info!(
            "virtio: Virtqueues::drop: core virtqueues about to drop\n"
        );
        /*
         * After Drop::drop returns:
         *
         *     inner -> Virtqueue -> VirtQueue<LinuxHal>
         *
         * are destroyed automatically.
         */
    }
}

unsafe extern "C" fn rust_vring_interrupt(
    data: *mut core::ffi::c_void,
) -> bool {
    let vq = unsafe {
        &*data.cast::<VirtQueue>()
    };

    if !vq.can_pop() {
        return false;
    }

    if vq.is_broken() {
        return true;
    }

    if let Some(callback) = vq.callback {
        callback(vq);
    }

    true
}

/// Any vendor
pub const VIRTIO_DEV_ANY_ID: u32 = 0xffffffff;

/// Virtio IDs
///
/// C header: [`include/uapi/linux/virtio_ids.h`](srctree/include/uapi/linux/virtio_ids.h)
#[repr(u32)]
pub enum VirtioID {
    /// virtio net
    Net = bindings::VIRTIO_ID_NET,
    /// virtio block
    Block = bindings::VIRTIO_ID_BLOCK,
    /// virtio console
    Console = bindings::VIRTIO_ID_CONSOLE,
    /// virtio rng
    Rng = bindings::VIRTIO_ID_RNG,
    /// virtio balloon
    Balloon = bindings::VIRTIO_ID_BALLOON,
    /// virtio ioMemory
    IOMem = bindings::VIRTIO_ID_IOMEM,
    /// virtio remote processor messaging
    RPMSG = bindings::VIRTIO_ID_RPMSG,
    /// virtio scsi
    Scsi = bindings::VIRTIO_ID_SCSI,
    /// 9p virtio console
    NineP = bindings::VIRTIO_ID_9P,
    /// virtio WLAN MAC
    Mac80211Wlan = bindings::VIRTIO_ID_MAC80211_WLAN,
    /// virtio remoteproc serial link
    RPROCSerial = bindings::VIRTIO_ID_RPROC_SERIAL,
    /// Virtio caif
    CAIF = bindings::VIRTIO_ID_CAIF,
    /// virtio memory balloon
    MemoryBalloon = bindings::VIRTIO_ID_MEMORY_BALLOON,
    /// virtio GPU
    GPU = bindings::VIRTIO_ID_GPU,
    /// virtio clock/timer
    Clock = bindings::VIRTIO_ID_CLOCK,
    /// virtio input
    Input = bindings::VIRTIO_ID_INPUT,
    /// virtio vsock transport
    VSock = bindings::VIRTIO_ID_VSOCK,
    /// virtio crypto
    Crypto = bindings::VIRTIO_ID_CRYPTO,
    /// virtio signal distribution device
    SignalDist = bindings::VIRTIO_ID_SIGNAL_DIST,
    /// virtio pstore device
    Pstore = bindings::VIRTIO_ID_PSTORE,
    /// virtio IOMMU
    Iommu = bindings::VIRTIO_ID_IOMMU,
    /// virtio mem
    Mem = bindings::VIRTIO_ID_MEM,
    /// virtio sound
    Sound = bindings::VIRTIO_ID_SOUND,
    /// virtio filesystem
    FS = bindings::VIRTIO_ID_FS,
    /// virtio pmem
    PMem = bindings::VIRTIO_ID_PMEM,
    /// virtio rpmb
    RPMB = bindings::VIRTIO_ID_RPMB,
    /// virtio mac80211-hwsim
    Mac80211Hwsim = bindings::VIRTIO_ID_MAC80211_HWSIM,
    /// virtio video encoder
    VideoEncoder = bindings::VIRTIO_ID_VIDEO_ENCODER,
    /// virtio video decoder
    VideoDecoder = bindings::VIRTIO_ID_VIDEO_DECODER,
    /// virtio SCMI
    SCMI = bindings::VIRTIO_ID_SCMI,
    /// virtio nitro secure module
    NitroSecMod = bindings::VIRTIO_ID_NITRO_SEC_MOD,
    /// virtio i2c adapter
    I2CAdapter = bindings::VIRTIO_ID_I2C_ADAPTER,
    /// virtio watchdog
    Watchdog = bindings::VIRTIO_ID_WATCHDOG,
    /// virtio can
    CAN = bindings::VIRTIO_ID_CAN,
    /// virtio dmabuf
    DMABuf = bindings::VIRTIO_ID_DMABUF,
    /// virtio parameter server
    ParamServ = bindings::VIRTIO_ID_PARAM_SERV,
    /// virtio audio policy
    AudioPolicy = bindings::VIRTIO_ID_AUDIO_POLICY,
    /// virtio bluetooth
    BT = bindings::VIRTIO_ID_BT,
    /// virtio gpio
    GPIO = bindings::VIRTIO_ID_GPIO,
    /// virtio spi
    SPI = bindings::VIRTIO_ID_SPI,
}
