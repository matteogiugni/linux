use core::{
    alloc::Layout,
    ptr::NonNull,
};

use crate::{
    alloc::{
        allocator::KVmalloc,
        flags::{GFP_KERNEL, __GFP_ZERO},
        Allocator as KernelAllocator,
        NumaNode,
    },
    device,
    error::{
        code::{EINVAL, ENOMEM},
        Error,
    },
    sync::{
        aref::ARef,
        barrier::{
            self,
            Full,
            Read,
            Write,
        },
    },
};

use super::hal::{
    Allocator as VirtQueueAllocator,
    DmaRegion,
    Hal,
    MemoryRegion,
};

pub(crate) struct LinuxHal {
    device: ARef<device::Device>,
}

impl LinuxHal {
    pub(crate) fn new<Ctx: device::DeviceContext>(
        dev: &crate::virtio::Device<Ctx>,
    ) -> Result<Self, Error> {
        let dma_dev = dev.dma_device()?;

        Ok(Self {
            device: dma_dev.into(),
        })
    }
}

unsafe impl VirtQueueAllocator for LinuxHal {
    type Error = Error;

    fn alloc(
        size: usize,
        align: usize,
    ) -> Result<MemoryRegion<Self>, Self::Error> {
        let layout =
            Layout::from_size_align(size, align)
                .map_err(|_| EINVAL)?;

        let allocation =
            <KVmalloc as KernelAllocator>::alloc(
                layout,
                GFP_KERNEL,
                NumaNode::NO_NODE,
            )?;

        let cpu_addr = allocation.cast::<u8>();

        // SAFETY:
        // `cpu_addr` was allocated by KVmalloc with exactly this
        // size/alignment contract.
        Ok(unsafe {
            MemoryRegion::new(
                cpu_addr,
                size,
                align,
            )
        })
    }

    /// SAFETY:
    ///
    /// `ptr`, `size`, and `align` must describe an allocation
    /// obtained from `Self::alloc`.
    unsafe fn dealloc(
        ptr: NonNull<u8>,
        size: usize,
        align: usize,
    ) {
        // SAFETY:
        // These are the same size/alignment values used by `alloc`.
        let layout =
            unsafe {
                Layout::from_size_align_unchecked(size, align)
            };

        // SAFETY:
        // `ptr` was returned by KVmalloc for this layout.
        unsafe {
            <KVmalloc as KernelAllocator>::free(
                ptr,
                layout,
            );
        }
    }

    fn dma_alloc(
        &self,
        size: usize,
        align: usize,
    ) -> Result<DmaRegion, Self::Error> {
        if size == 0
            || align == 0
            || !align.is_power_of_two()
        {
            return Err(EINVAL);
        }

        let dma_align_mask: bindings::dma_addr_t =
            (align - 1)
                .try_into()
                .map_err(|_| EINVAL)?;

        // Keep the native Linux `dma_addr_t` representation.
        // Its width is architecture-dependent.
        let mut dma_addr: bindings::dma_addr_t = 0;

        // SAFETY:
        // - `self.device` keeps the underlying `struct device` alive;
        // - `size` is non-zero;
        // - GFP_KERNEL permits sleeping, which is valid during virtqueue
        //   setup/allocation;
        // - __GFP_ZERO implements the HAL contract that newly allocated
        //   DMA regions are zero-initialized.
        let ptr = unsafe {
            bindings::dma_alloc_attrs(
                self.device.as_raw(),
                size,
                &mut dma_addr,
                //CHECK kernel panic when called in add_indirect or atomic operations? GFP_ATOMIC?
                (GFP_KERNEL | __GFP_ZERO).as_raw(),
                0,
            )
        };

        let cpu_addr =
            NonNull::new(ptr.cast::<u8>())
                .ok_or(ENOMEM)?;

        if cpu_addr.as_ptr().addr() & (align - 1) != 0
            || dma_addr & dma_align_mask != 0
        {
            // SAFETY:
            // This allocation has not been exposed to the device yet.
            unsafe {
                bindings::dma_free_attrs(
                    self.device.as_raw(),
                    size,
                    cpu_addr.as_ptr().cast(),
                    dma_addr,
                    0,
                );
            }

            return Err(EINVAL);
        }

        Ok(unsafe {
            DmaRegion::new(
                cpu_addr,
                dma_addr,
                size,
            )
        })
    }

    /// SAFETY:
    ///
    /// `region` must have been returned by a previous call to `dma_alloc`
    /// and must not be used after this call, device included.
    unsafe fn dma_free(
        &self,
        region: DmaRegion,
    ) {
        // SAFETY:
        // By the Hal contract:
        // - `region` was returned by `dma_alloc` for this HAL/device;
        // - it has not already been freed;
        // - the caller guarantees that the device can no longer access it.
        unsafe {
            bindings::dma_free_attrs(
                self.device.as_raw(),
                region.size,
                region.cpu_addr.as_ptr().cast(),
                region.dma_addr,
                0,
            );
        }
    }
}

unsafe impl Hal for LinuxHal {
    //TODO dma barriers are supported by the upstream but not in this current version of the kernel
    #[inline]
    fn write_barrier(&self) {
        barrier::dma_mb(Write);
    }

    #[inline]
    fn read_barrier(&self) {
        barrier::dma_mb(Read);
    }

    #[inline]
    fn mb(&self) {
        barrier::mb(Full);
    }
}
