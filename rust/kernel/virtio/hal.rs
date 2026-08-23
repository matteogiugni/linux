// SPDX-License-Identifier: ?

//! Hardware Abstraction Layer for virtqueue implementations.
//!
//! This trait defines the OS-specific operations needed by the
//! virtqueue logic. Implement this trait for each target OS.

pub type DmaAddress = u64;
//TODO cpu and dma address, virtaddr and physaddr, or vaddr and paddr?
pub struct DmaRegion {
    pub cpu_addr: core::ptr::NonNull<u8>,
    pub dma_addr: DmaAddress,
    pub size: usize,
}

/// Hardware abstraction layer for virtqueue OS-specific operations.
///
/// Implement this trait to port the virtqueue logic to a new OS or
/// environment.
///
/// # Safety
///
/// Implementors must guarantee that:
/// - `dma_alloc` returns memory accessible by both CPU and device.
/// - `dma_free` only frees memory previously returned by `dma_alloc`.
/// - `write_barrier` ensures all preceding writes are visible to the device.
/// - `read_barrier` ensures all preceding device writes are visible to the CPU.
pub unsafe trait Hal {

    type Error;

    /// Allocate a DMA-capable memory region of `size` bytes.
    ///
    /// Returns a [`DmaRegion`] containing the size and both 
    /// the CPU-accessible address and the device-accessible address.
    ///
    /// The memory must be zeroed.  ***TODO to decide***
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

    //TODO to decide wheter or not desc_state/extra 
    //     should be allocated by the virtqueue logic
    //     otherwise remove these
    fn alloc<T>(&self, count: usize) -> Result<NonNull<T>, Self::Error>;
    unsafe fn dealloc<T>(&self, ptr: NonNull<T>, count: usize);
    
    //TODO linux implements weak barriers with VIRTIO_F_ORDER_PLATFORM feature
    //     add extra param to barrier functions to implement this feature if needed

    /// Issue a write memory barrier.
    ///
    /// Ensures that all preceding writes to the descriptor table and
    /// available ring are visible to the device before it processes them.
    ///
    /// Equivalent to `virtio_wmb()` in the C kernel.
    fn write_barrier();

    /// Issue a read memory barrier.
    ///
    /// Ensures that all device writes to the used ring are visible to
    /// the CPU before we read them.
    ///
    /// Equivalent to `virtio_rmb()` in the C kernel.
    fn read_barrier();

    /// Full memory barrier for device-visible shared memory.
    fn mb(&self);
}