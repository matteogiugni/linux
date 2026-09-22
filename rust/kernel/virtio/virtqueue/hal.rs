// SPDX-License-Identifier: ?

//! Hardware Abstraction Layer for virtqueue implementations.
//!
//! This trait defines the OS-specific operations needed by the
//! virtqueue logic. Implement this trait for each target OS.

use core::{
    marker::PhantomData,
    ptr::NonNull,
};

/// A DMA-accessible memory address.
pub(super) type DmaAddress = u64;
//TODO cpu and dma address, virtaddr and physaddr, or vaddr and paddr?
//TODO impl Drop for DmaRegion to automatically free the memory when out of scope?
/// A DMA-accessible memory region.
pub struct DmaRegion {
    /// The CPU-accessible address of the memory region.
    pub cpu_addr: core::ptr::NonNull<u8>,
    /// The device-accessible address of the memory region.
    pub dma_addr: DmaAddress,
    /// The size of the memory region in bytes.
    pub size: usize,
}

impl DmaRegion {
    pub(super) unsafe fn new(
        cpu_addr: NonNull<u8>,
        dma_addr: DmaAddress,
        size: usize,
    ) -> Self {
        Self {
            cpu_addr,
            dma_addr,
            size,
        }
    }
}

pub struct MemoryRegion<A: Allocator> {
    pub cpu_addr: NonNull<u8>,
    pub size: usize,
    pub align: usize,
    // This field is used to determine which allocator should handle the drop of this MemoryRegion.
    _allocator: PhantomData<A>,
}

/// # Safety
///
/// Implementors must guarantee that:
/// - `dma_alloc` returns memory accessible by both CPU and device, DMA coherent.
/// - `dma_free` only frees memory previously returned by `dma_alloc`.
pub unsafe trait Allocator {
    type Error;

    /// Allocates `size` bytes aligned to `align`.
    fn alloc(
        size: usize,
        align: usize,
    ) -> Result<MemoryRegion<Self>, Self::Error>
    where
        Self: Sized;

    /// Releases an allocation previously returned by `Self::alloc`.
    ///
    /// # Safety
    ///
    /// `ptr`, `size`, and `align` must describe a live allocation
    /// returned by this allocator.
    unsafe fn dealloc(
        ptr: NonNull<u8>,
        size: usize,
        align: usize,
    );

    /// Allocate a DMA-accessible memory region of `size` bytes.
    ///
    /// Returns a [`DmaRegion`] containing the size and both 
    /// the CPU-accessible address and the device-accessible address.
    ///
    /// On success, the returned region must:
    /// - contain at least `size` bytes.
    /// - satisfy the requested alignment.
    /// - be accessible by both CPU and device.
    /// - not require explicit cache synchronization for ordinary virtqueue.
    ///   shared-memory accesses.
    /// - be zero-initialized.  ***TODO to decide***
    fn dma_alloc(
        &self,
        size: usize,
        align: usize,
    ) -> Result<DmaRegion, Self::Error>;

    /// Free memory previously allocated by [`Hal::dma_alloc`].
    ///
    /// # Safety
    ///
    /// `region` must have been returned by a previous call to `dma_alloc`
    /// and must not be used after this call, device included.
    unsafe fn dma_free(
        &self,
        region: DmaRegion,
    );
}

impl<A: Allocator> MemoryRegion<A> {
    /// # Safety
    ///
    /// `cpu_addr`, `size`, and `align` must describe an allocation
    /// obtained from `A`.
    pub unsafe fn new(
        cpu_addr: NonNull<u8>,
        size: usize,
        align: usize,
    ) -> Self {
        Self {
            cpu_addr,
            size,
            align,
            _allocator: PhantomData,
        }
    }

    #[inline]
    pub fn cpu_addr(&self) -> NonNull<u8> {
        self.cpu_addr
    }

    #[inline]
    pub fn size(&self) -> usize {
        self.size
    }

    #[inline]
    pub fn align(&self) -> usize {
        self.align
    }
}

impl<A: Allocator> Drop for MemoryRegion<A> {
    fn drop(&mut self) {
        // SAFETY:
        // `MemoryRegion<A>` is constructed only from an allocation
        // returned by `A`, uniquely owns that allocation, and stores
        // the original size and alignment.
        unsafe {
            A::dealloc(
                self.cpu_addr,
                self.size,
                self.align,
            );
        }
    }
}

/// Hardware abstraction layer for virtqueue OS-specific operations.
///
/// Implement this trait to port the virtqueue logic to a new OS or
/// environment.
///
/// # Safety
///
/// - `write_barrier` ensures all preceding writes are visible to the device.
/// - `read_barrier` ensures all preceding device writes are visible to the CPU.
pub unsafe trait Hal: Allocator {

    //CHECK at the moment it's useless
    // type Error;
    
    //TODO linux implements weak barriers with VIRTIO_F_ORDER_PLATFORM feature
    //     add extra param to barrier functions to implement this feature if needed

    /// Issue a write memory barrier.
    ///
    /// Ensures that all preceding writes to the descriptor table and
    /// available ring are visible to the device before it processes them.
    ///
    /// Equivalent to `virtio_wmb()` in the C kernel.
    fn write_barrier(&self);

    /// Issue a read memory barrier.
    ///
    /// Ensures that all device writes to the used ring are visible to
    /// the CPU before we read them.
    ///
    /// Equivalent to `virtio_rmb()` in the C kernel.
    fn read_barrier(&self);

    /// Full memory barrier for device-visible shared memory.
    fn mb(&self);
}