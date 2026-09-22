// SPDX-License-Identifier: GPL-2.0 OR Apache-2.0

use core::{
    mem::{size_of, align_of},
    ptr::NonNull,
};

mod hal;
mod linux_virtqueue;

use self::hal::{
    DmaAddress,
    DmaRegion,
    Hal,
    MemoryRegion,
};

pub(crate) use self::linux_virtqueue::LinuxHal;

const DESC_ALIGN: usize = 16;
const AVAIL_ALIGN: usize = 2;
const USED_ALIGN: usize = 4;

/// Descriptor flag: buffer continues via `next` field.
pub const VIRTQ_DESC_F_NEXT: u16 = 1;

/// Descriptor flag: buffer is device-writable.
pub const VIRTQ_DESC_F_WRITE: u16 = 2;

/// Descriptor flag: buffer contains indirect descriptor table.
pub const VIRTQ_DESC_F_INDIRECT: u16 = 4;

/// Available ring flag: suppress used buffer notifications.
pub const VIRTQ_AVAIL_F_NO_INTERRUPT: u16 = 1;

/// Used ring flag: suppress available buffer notifications.
pub const VIRTQ_USED_F_NO_NOTIFY: u16 = 1;

/// Virtqueue feature bit: indirect descriptors.
pub const VIRTIO_RING_F_INDIRECT_DESC: u32 = 28;
/// Virtqueue feature bit: event index support.
pub const VIRTIO_RING_F_EVENT_IDX: u32 = 29;

//TODO refactor the code and divide into files
//TODO packed virtqueue
//TODO HAL as PhantomData or as instance? after HAL complete definition
//TODO if the VirtQueueFeatures will get a lot of fields, we may save this struct directly as a field
//TODO VIRTIO_F_IN_ORDER feature?
//TODO VIRTIO_F_RING_RESET feature?
//TODO VIRTIO_F_ORDER_PLATFORM feature for weak barriers?
//TODO when packed vqs are implemented, abstract the split vq struct from here
//TODO this struct cannot be created by the driver -> only virtio framework
//CHECK rollback on failed operation in every function
/// A virtqueue data structure.
pub struct VirtQueue<H: Hal> {
    //CHECK is the option needed?
    ring_memory: Option<DmaRegion>,

    desc: NonNull<Descriptor>,
    avail: AvailRing,
    used: UsedRing,

    avail_offset: usize,
    used_offset: usize,

    size: u16,
    queue_idx: u16,

    free_head: u16,
    num_free: u16,
    num_added: u16,

    desc_state_memory: MemoryRegion<H>,
    desc_extra_memory: MemoryRegion<H>,

    avail_idx: u16,
    //TODO possible future need
    // avail_flags: u16,
    last_used_idx: u16,

    event_idx: bool,
    indirect: bool,

    broken: bool,

    hal: H,
}

impl<H: Hal> VirtQueue<H> {

    pub(super) fn new(
        hal: H,
        queue_idx: u16,
        size: u16,
        features: VirtQueueFeatures,
    ) -> Result<Self, Error<H::Error>> {
        let (desc_size, avail_size, used_size) = queue_part_sizes(size)?;

        let n = usize::from(size);

        // SAFETY:
        // - `queue_part_sizes` guarantees `size <= 32768`.
        // - The maximum allocation size for the current `DescState` and
        //   `DescExtra` layouts is therefore representable in `usize`.
        let desc_state_size = size_of::<DescState>() * n;
        let desc_extra_size = size_of::<DescExtra>() * n;

        // SAFETY:
        // These memory allocations are dropped by the MemoryRegion Drop impl,
        // which calls the appropriate deallocation function.
        let desc_state_memory = H::alloc(
                desc_state_size,
                align_of::<DescState>(),
            )
            .map_err(Error::Hal)?;
        
        let desc_extra_memory = H::alloc(
                desc_extra_size,
                align_of::<DescExtra>(),
            )
            .map_err(Error::Hal)?;
        
        //CHECK if the alloc() guarantees at least the requested size we can remove this
        if desc_extra_memory.size < desc_extra_size ||
            desc_state_memory.size < desc_state_size {
            return Err(Error::Queue(
                    VirtQueueError::MemoryAllocationFailed,
                ));
        }

        // SAFETY:
        // - desc_size, avail_size, and used_size are computed from the queue size, 
        //   which is guaranteed to be <= 32768 and multiplied by small constants, 
        //   so they are all representable in u32.
        // - The sum of u32 values is representable in usize.
        let desc_offset = 0;

        let avail_offset =
            align_up(desc_offset + desc_size, AVAIL_ALIGN)?;

        let used_offset =
            align_up(avail_offset + avail_size, USED_ALIGN)?;

        let total_size = used_offset + used_size;

        let ring_memory = hal.dma_alloc(
                total_size,
                DESC_ALIGN,
            )
            .map_err(Error::Hal)?;

        // TODO if this fails, try to divide the 3 rings into 3 different memory spaces?
        //      or directly allocate them separetely?
        //      Slower but more likely to succeed
        if ring_memory.size < total_size {
            // SAFETY:
            // The region was allocated by this function and has not yet been
            // exposed to the device.
            unsafe {
                hal.dma_free(ring_memory);
            }

            return Err(Error::Queue(
                VirtQueueError::MemoryAllocationFailed,
            ));
        }

        // TODO to decide whether the HAL should zero out all
        //      allocated memory or if the virtqueue logic should.
        //      The alloc() implementation may already zero out the memory by default
        //      so maybe it's better to let it handle it.

        let desc_state_ptr =
            desc_state_memory
                .cpu_addr()
                .cast::<DescState>();

        let desc_extra_ptr =
            desc_extra_memory
                .cpu_addr()
                .cast::<DescExtra>();

        // SAFETY:
        // - `desc_state_memory` contains space for exactly `n` `DescState`s.
        // - its address satisfies `align_of::<DescState>()`.
        // - every element is currently uninitialized and exclusively owned here.
        //
        // The same holds for `desc_extra_memory`.
        unsafe {
            for i in 0..n {
                desc_state_ptr
                    .as_ptr()
                    .add(i)
                    .write(DescState::empty());

                // SAFETY:
                // - `n = self.size`.
                // - `self.size` is a `u16` and a power of 2.
                let next = if i + 1 < n {
                    (i + 1) as u16
                } else {
                    0
                };

                desc_extra_ptr
                    .as_ptr()
                    .add(i)
                    .write(DescExtra { next });
            }
        }

        let base = ring_memory.cpu_addr;

        // SAFETY: descriptor area begins at offset 0 and the allocation is
        // large enough for the computed layout.
        let desc = unsafe {
            NonNull::new_unchecked(
                base.as_ptr().cast::<Descriptor>(),
            )
        };

        // SAFETY: `avail_offset < total_size`.
        let avail_ptr = unsafe {
            NonNull::new_unchecked(
                base.as_ptr().add(avail_offset),
            )
        };

        // SAFETY: `used_offset < total_size`.
        let used_ptr = unsafe {
            NonNull::new_unchecked(
                base.as_ptr().add(used_offset),
            )
        };

        // SAFETY: both pointers refer to correctly-sized portions of the
        // allocated virtqueue memory.
        let avail = unsafe {
            AvailRing::new(avail_ptr, size)
        };

        let used = unsafe {
            UsedRing::new(used_ptr, size)
        };

        let queue = Self {
            ring_memory: Some(ring_memory),
            desc,
            avail,
            used,
            size,
            queue_idx,
            num_free: size,
            num_added: 0,
            free_head: 0,
            desc_state_memory,
            desc_extra_memory,
            avail_idx: 0,
            // avail_flags: 0,
            last_used_idx: 0,
            event_idx: features.event_idx,
            indirect: features.indirect,
            avail_offset,
            used_offset,
            hal,
            broken: false,
        };

        Ok(queue)
    }

    fn add_direct(
        &mut self,
        segments: &[DmaSegment],
    ) -> Result<u16, VirtQueueError> {
        if segments.len() > usize::from(self.num_free) {
            return Err(VirtQueueError::QueueFull);
        }

        let head = self.free_head;

        for (i, segment) in segments.iter().enumerate() {
            let index = self.free_head;

            let next_free =
                self.desc_extras()[usize::from(index)].next;

            let mut flags = DescFlags::empty();

            let next = if i + 1 < segments.len() {
                flags.insert(DescFlags::NEXT);

                next_free
            } else {
                0
            };

            if segment.direction == BufferDirection::DeviceToDriver {
                flags.insert(DescFlags::WRITE);
            }

            // SAFETY: The index is checked and the descriptor has not yet been published to the device.
            unsafe {
                self.write_desc(
                    index,
                    segment.dma_addr,
                    segment.len,
                    flags,
                    next,
                );
            }

            self.free_head = next_free;
        }

        // SAFETY:
        // - `segments.len() <= self.num_free`.
        // - `self.num_free` is a `u16`.
        self.num_free -= segments.len() as u16;

        Ok(head)
    }

    fn add_indirect(
        &mut self,
        segments: &[DmaSegment],
    ) -> Result<u16, Error<H::Error>> {
        //CHECK linux does not check this 
        //      but the VIRTIO spec says that the indirect descriptors must not exceed the queue size
        if segments.len() > usize::from(self.size) {
            return Err(Error::Queue(VirtQueueError::InvalidParam));
        }

        if self.num_free == 0 {
            return Err(Error::Queue(VirtQueueError::QueueFull));
        }

        // SAFETY:
        // - `segments.len() <= self.size`.
        // - `self.size` is a `u16`.
        // - `size_of::<Descriptor>()` is 16 bytes.
        let indirect_size = segments.len() * size_of::<Descriptor>();

        let memory = self
            .hal
            .dma_alloc(indirect_size, DESC_ALIGN)
            .map_err(Error::Hal)?;

        if memory.size < indirect_size {
            // SAFETY:
            // The region was allocated by this function and has not yet been
            // exposed to the device.
            unsafe {
                self.hal.dma_free(memory);
            }

            return Err(Error::Queue(VirtQueueError::MemoryAllocationFailed));
        }

        //TODO to decide about zeroing out the allocated memory

        let indirect_desc =
            memory.cpu_addr.as_ptr().cast::<Descriptor>();

        for (i, segment) in segments.iter().enumerate() {
            let mut flags = DescFlags::empty();

            // SAFETY:
            // - `segments.len() <= self.size`.
            // - `self.size` is a `u16` and a power of 2.
            let next : u16 = if i + 1 < segments.len() {
                flags.insert(DescFlags::NEXT);

                (i + 1) as u16
            } else {
                0
            };

            if segment.direction == BufferDirection::DeviceToDriver {
                flags.insert(DescFlags::WRITE);
            }

            let descriptor = Descriptor {
                addr: segment.dma_addr.to_le(),
                len: segment.len.to_le(),
                flags: flags.bits().to_le(),
                next: next.to_le(),
            };

            // SAFETY:
            // indirect_desc points to a valid and correctly aligned memory region 
            // of at least `indirect_size` bytes, 
            // which is large enough to hold `segments.len()` descriptors.
            unsafe {
                core::ptr::write_volatile(
                    indirect_desc.add(i),
                    descriptor,
                );
            }
        }

        let head = self.free_head;
        let next_free = self.desc_extras()[usize::from(head)].next;

        // SAFETY: The index is checked and the descriptor has not yet been published to the device.
        unsafe {
            self.write_desc(
                head,
                memory.dma_addr,
                // SAFETY:
                // - Descriptor size is 16 bytes.
                // - `indirect_size <= u16::MAX * 16`.
                indirect_size as u32,
                DescFlags::INDIRECT,
                0,
            );
        }

        // SAFETY: `self.num_free > 0` for the above check.
        self.num_free -= 1;
        self.free_head = next_free;

        self.desc_states_mut()[usize::from(head)].indirect = Some(memory);

        Ok(head)
    }

    //TODO extract the common part of add_indirect and add_direct_then_indirect 
    //     to a separate function that builds the indirect table
    fn add_direct_then_indirect(
        &mut self,
        direct_segments: &[DmaSegment],
        indirect_segments: &[DmaSegment],
    ) -> Result<u16, Error<H::Error>> {
        if direct_segments.is_empty()
            || indirect_segments.is_empty()
        {
            return Err(Error::Queue(
                VirtQueueError::InvalidParam,
            ));
        }
        
        if self.num_free == 0 
        || (direct_segments.len() > usize::from(self.num_free) - 1) {
            return Err(Error::Queue(
                VirtQueueError::QueueFull,
            ));
        }

        //CHECK descriptor chain > queuesize per type of chain or together?
        //      if it's the sum, refactor validate_segments.
        //      VIRTIO spec is not clear about this check
        if indirect_segments.len() > usize::from(self.size) {
            return Err(Error::Queue(
                VirtQueueError::InvalidParam,
            ));
        }

        // SAFETY:
        // - `indirect_segments.len() <= self.size`. ***CHECK above***
        // - `self.size` is a `u16`.
        // - `size_of::<Descriptor>()` is 16 bytes.
        let indirect_size = indirect_segments.len() * size_of::<Descriptor>();

        let indirect_memory = self
            .hal
            .dma_alloc(indirect_size, DESC_ALIGN)
            .map_err(Error::Hal)?;

        if indirect_memory.size < indirect_size {
            // SAFETY:
            // The region was allocated by this function and has not yet been
            // exposed to the device.
            unsafe {
                self.hal.dma_free(indirect_memory);
            }

            return Err(Error::Queue(
                VirtQueueError::MemoryAllocationFailed,
            ));
        }

        //TODO to decide about zeroing out the allocated memory

        let indirect_desc = indirect_memory
            .cpu_addr
            .as_ptr()
            .cast::<Descriptor>();

        for (i, segment) in indirect_segments.iter().enumerate() {
            let mut flags = DescFlags::empty();

            // SAFETY:
            // - `indirect_segments.len() <= self.size`. ***CHECK above***
            // - `self.size` is a `u16` and a power of 2.
            let next = if i + 1 < indirect_segments.len() {
                flags.insert(DescFlags::NEXT);

                (i + 1) as u16
            } else {
                0
            };

            if segment.direction
                == BufferDirection::DeviceToDriver {
                flags.insert(DescFlags::WRITE);
            }

            let descriptor = Descriptor {
                addr: segment.dma_addr.to_le(),
                len: segment.len.to_le(),
                flags: flags.bits().to_le(),
                next: next.to_le(),
            };

            
            // SAFETY:
            // indirect_desc points to a valid and correctly aligned memory region 
            // of at least `indirect_size` bytes, 
            // which is large enough to hold `indirect_segments.len()` descriptors.
            unsafe {
                core::ptr::write_volatile(
                    indirect_desc.add(i),
                    descriptor,
                );
            }
        }

        let head = self.free_head;
        let mut current = self.free_head;

        for segment in direct_segments {
            let next =
                self.desc_extras()[usize::from(current)].next;

            let mut flags = DescFlags::NEXT;

            if segment.direction
                == BufferDirection::DeviceToDriver {
                flags.insert(DescFlags::WRITE);
            }

            // SAFETY: The index is checked and the descriptor has not yet been published to the device.
            unsafe {
                self.write_desc(
                    current,
                    segment.dma_addr,
                    segment.len,
                    flags,
                    next,
                );
            }

            current = next;
        }

        let next_free =
            self.desc_extras()[usize::from(current)].next;

        // SAFETY: The index is checked and the descriptor has not yet been published to the device.
        unsafe {
            self.write_desc(
                current,
                indirect_memory.dma_addr,
                // SAFETY:
                // - `self.size` is a `u16`.
                // - Descriptor size is 16 bytes.
                // - `indirect_size <= u16::MAX * 16`.
                indirect_size as u32,
                DescFlags::INDIRECT,
                0,
            );
        }

        self.free_head = next_free;

        // SAFETY:
        // - Cannot underflow for the above checks.
        self.num_free -= (direct_segments.len() + 1) as u16;

        self.desc_states_mut()[usize::from(head)].indirect =
            Some(indirect_memory);

        Ok(head)
    }

    /// Builds the descriptor chain and then
    /// adds it to the available ring.
    /// Returns the head descriptor index of the chain.
    ///
    /// # Safety
    ///
    /// The caller must guarantee that all mappings described by `segments`
    /// remain valid until this descriptor chain is returned by the device.
    pub unsafe fn add(
        &mut self,
        segments: &[DmaSegment],
        token: Token,
        layout: DescriptorLayout,
    ) -> Result<(), Error<H::Error>> {
        if self.broken {
            return Err(Error::Queue(VirtQueueError::BrokenVirtQueue));
        }

        Self::validate_segments(segments)?;

        let head = match layout {
            DescriptorLayout::Direct => {
                self.add_direct(segments)?
            }

            DescriptorLayout::Indirect => {
                if !self.indirect {
                    return Err(Error::Queue(
                        VirtQueueError::FeatureNotNegotiated,
                    ));
                }

                //CHECK optimization, but we can also always 
                //      allocate the indirect table even for a single segment
                if segments.len() == 1 {
                    self.add_direct(segments)?
                } else {
                    self.add_indirect(segments)?
                }
            }
            //CHECK linux does not permit this
            //      but the VIRTIO spec says this is allowed
            DescriptorLayout::DirectThenIndirect { indirect_from } => {
                if !self.indirect {
                    return Err(Error::Queue(
                        VirtQueueError::FeatureNotNegotiated,
                    ));
                }

                //CHECK what if len is 1? right now checked in add_direct_then_indirect
                if indirect_from == 0
                    || indirect_from >= segments.len()
                {
                    return Err(Error::Queue(
                        VirtQueueError::InvalidParam,
                    ));
                }

                self.add_direct_then_indirect(
                    &segments[..indirect_from],
                    &segments[indirect_from..],
                )?
            }
        };

        self.desc_states_mut()[usize::from(head)].token = Some(token);

        let avail_slot = self.avail_idx & (self.size - 1);

        self.avail.write_ring(avail_slot, head);

        self.hal.write_barrier();

        self.avail_idx = self.avail_idx.wrapping_add(1);
        self.avail.set_idx(self.avail_idx);

        //TODO if it wraps maybe we should force a notify?
        self.num_added = self.num_added.wrapping_add(1);

        Ok(())
    }

    /// Adds a single input buffer (device-writable) to the virtqueue.
    ///
    /// - `addr` is the DMA address of the buffer.
    /// - `len` is the buffer size in bytes.
    /// - `token` is an opaque driver-provided value returned by [`VirtQueue::get_buf`]
    ///    when the buffer is used.
    ///
    /// # Safety
    ///
    /// The buffer at `addr` must remain valid and unmodified by the CPU
    /// until the corresponding `get_buf` call returns.
    pub unsafe fn add_inbuf(
        &mut self,
        addr: DmaAddress,
        len: u32,
        token: Token,
        layout: DescriptorLayout,
    ) -> Result<(), Error<H::Error>> {
        let segment = DmaSegment {
            dma_addr: addr,
            len,
            direction: BufferDirection::DeviceToDriver,
        };

        unsafe { self.add(core::slice::from_ref(&segment), token, layout) }
    }


    /// Adds a single output buffer to the virtqueue.
    ///
    /// - `addr` is the DMA address of the buffer.
    /// - `len` is the buffer size in bytes.
    /// - `token` is an opaque driver-provided value returned by [`VirtQueue::get_buf`]
    ///    when the buffer is used.
    ///
    /// # Safety
    ///
    /// The buffer at `addr` must remain valid and unmodified by the CPU
    /// until the corresponding `get_buf` call returns.
    pub unsafe fn add_outbuf(
        &mut self,
        addr: DmaAddress,
        len: u32,
        token: Token,
        layout: DescriptorLayout,
    ) -> Result<(), Error<H::Error>> {
        let segment = DmaSegment {
            dma_addr: addr,
            len,
            direction: BufferDirection::DriverToDevice,
        };

        unsafe { self.add(core::slice::from_ref(&segment), token, layout) }
    }

    /// Retrieves the next used buffer, if any.
    ///
    /// Returns the token and the number of bytes written by the device,
    /// or `None` if no buffer has been processed yet.
    ///
    /// Matches `virtqueue_get_buf` in the C kernel.
    pub fn get_buf(
        &mut self,
    ) -> Result<Option<(Token, u32)>, VirtQueueError> {
        if self.broken {
            return Err(VirtQueueError::BrokenVirtQueue);
        }

        if !self.can_pop() {
            return Ok(None);
        }

        self.hal.read_barrier();

        let slot = self.last_used_idx & (self.size - 1);
        let elem = match self.used.read_elem(slot) {
            Ok(elem) => elem,
            Err(err) => {
                self.mark_broken();
                return Err(err);
            }
        };

        if elem.id >= u32::from(self.size) {
            self.mark_broken();
            return Err(VirtQueueError::CorruptedUsedElem);
        }

        let head = elem.id as u16;
        
        let token = match self.desc_states()[usize::from(head)].token {
            Some(token) => token,
            None => {
                self.mark_broken();
                return Err(VirtQueueError::CorruptedDescriptor);
            }
        };

        // SAFETY: The device has finished with this request, so it can no longer access the descriptor chain.
        unsafe { self.recycle_chain(head)? };

        self.last_used_idx = self.last_used_idx.wrapping_add(1);

        if self.event_idx
            //TODO add this check && (self.avail_flags & VIRTQ_AVAIL_F_NO_INTERRUPT) == 0
        {
            self.avail.set_used_event(self.last_used_idx);
            self.hal.mb();
        }

        Ok(Some((token, elem.len)))
    }

    /// Recycles a descriptor chain back to the free list.
    /// 
    /// # Safety
    ///
    /// The caller must guarantee that the device can no longer access or consume descriptors from this descriptor chain.
    /// The caller must guarantee that the head index is valid and that the descriptor has a token associated with it.
    unsafe fn recycle_chain(
        &mut self,
        head: u16,
    ) -> Result<(), VirtQueueError> {
        let mut current = head;
        let mut count = 0u16;

        /*
        * Count how many descriptors are in the chain, and check that the
        * chain is valid.  After the loop, current will be the index of the last descriptor in the chain.
        * This chain will be reused to attach the free list to the end of the chain.
        */
        loop {

            //CHECK a properly constructed chain cannot fail this check
            if count == self.size {
                self.mark_broken();
                return Err(VirtQueueError::CorruptedDescriptor);
            }

            count += 1;

            let descriptor = unsafe {
                core::ptr::read_volatile(
                    self.desc.as_ptr().add(usize::from(current)),
                )
            };

            let flags = u16::from_le(descriptor.flags);

            if flags & VIRTQ_DESC_F_NEXT == 0 {
                break;
            }

            let next = self.desc_extras()[usize::from(current)].next;

            //CHECK a properly constructed chain cannot fail this check
            if next >= self.size {
                return Err(VirtQueueError::CorruptedDescriptor);
            }

            current = next;
        }

        //CHECK a properly constructed chain cannot fail this check
        if count > self.size - self.num_free {
            self.mark_broken();
            return Err(VirtQueueError::CorruptedDescriptor);
        }

        self.desc_extras_mut()[usize::from(current)].next = self.free_head;
        self.free_head = head;
        self.num_free += count;

        self.desc_states_mut()[usize::from(head)].token = None;

        if let Some(indirect) =
            self.desc_states_mut()[usize::from(head)].indirect.take()
        {
            // SAFETY:
            // The used-ring entry tells us that the device has finished with
            // this request, so it can no longer access the indirect table.
            unsafe {
                self.hal.dma_free(indirect);
            }
        }

        Ok(())
    }

    /// # Safety
    ///
    /// The caller must guarantee that the index is valid and that the descriptor has not yet been published to the device.
    unsafe fn write_desc(
        &mut self,
        index: u16,
        addr: DmaAddress,
        len: u32,
        flags: DescFlags,
        next: u16,
    ) {
        let descriptor = Descriptor {
            addr: addr.to_le(),
            len: len.to_le(),
            flags: flags.bits().to_le(),
            next: next.to_le(),
        };

        // SAFETY:
        // - `index < self.size`.
        // - `self.desc` points to a descriptor table containing `self.size`
        //   entries.
        // - This descriptor has not yet been published to the device through
        //   the available ring.
        unsafe {
            core::ptr::write_volatile(
                self.desc.as_ptr().add(usize::from(index)),
                descriptor,
            );
        }
    }

    fn init_free_list(&mut self) {
        for i in 0..self.size {
            self.desc_states_mut()[usize::from(i)] = DescState::empty();

            self.desc_extras_mut()[usize::from(i)] = DescExtra {
                next: i + 1,
            };
        }

        let last = usize::from(self.size - 1);
        self.desc_extras_mut()[last].next = 0;

        self.free_head = 0;
        self.num_free = self.size;
    }

    /// Returns `true` if the device should be notified.
    pub(crate) fn kick_prepare(&mut self) -> bool {
        //CHECK linux doesnt check broken here but on the kick() function
        if self.broken || self.num_added == 0 {
            return false;
        }
        
        self.hal.mb();

        let new = self.avail_idx;
        let old = new.wrapping_sub(self.num_added);
        
        self.num_added = 0;

        let needs_kick = if self.event_idx {
            Self::need_event(self.used.avail_event(), new, old)
        } else {
            (self.used.flags() & VIRTQ_USED_F_NO_NOTIFY) == 0
        };

        needs_kick
    }

    pub(crate) fn notification_data(&self) -> u32 {
        (u32::from(self.avail_idx) << 16)
            | u32::from(self.queue_idx)
    }

    /// Validates the segments of a descriptor chain.
    //CHECK the input->output order of the segments must be validated also
    //      for the direct then indirect layout, or only for the direct and indirect parts of it?
    //      VIRTIO spec is not clear about this check
    //CHECK the maximum number of bytes of the chain must be validated also
    //      for the direct then indirect layout, or only for the direct and indirect parts of it?
    //      VIRTIO spec is not clear about this check
    fn validate_segments(
        segments: &[DmaSegment],
    ) -> Result<(), VirtQueueError> {
        if segments.is_empty() {
            return Err(VirtQueueError::InvalidParam);
        }

        let mut total_len = 0u64;
        let mut seen_write = false;

        for segment in segments {
            // SAFETY: segment.len is a u32, so it cannot overflow u64
            // due to the next check.
            total_len = total_len + u64::from(segment.len);

            if total_len > (1u64 << 32) {
                return Err(VirtQueueError::InvalidParam);
            }

            match segment.direction {
                BufferDirection::DriverToDevice => {
                    if seen_write {
                        return Err(VirtQueueError::InvalidParam);
                    }
                }

                BufferDirection::DeviceToDriver => {
                    seen_write = true;
                }
            }
        }

        Ok(())
    }

    /// # Safety
    ///
    /// The caller must guarantee that the device can no longer access the
    /// virtqueue or any outstanding descriptor chain and it has been reset.
    unsafe fn init_ring_state(&mut self) {
        self.avail.set_flags(0);
        self.avail.set_idx(0);
        self.avail.set_used_event(0);

        // SAFETY: The caller guarantees that the device can no longer access
        // the virtqueue or any outstanding descriptor chain.
        unsafe { 
            self.used.reset(); 
        }
    }

    /// Disables callbacks from the device.
    //TODO driver should be able to specificy the idx from which it wants to be notified
    pub fn disable_cb(&mut self) {
        if self.event_idx {
            self.avail
                .set_used_event(self.last_used_idx.wrapping_sub(1));
        } else {
            self.avail
            .set_flags(VIRTQ_AVAIL_F_NO_INTERRUPT);
        }
    }
    
    /// Enables callbacks from the device after a fixed number of used buffers have been processed.
    pub fn enable_cb_delayed(&mut self) -> bool {
        let outstanding =
            self.avail_idx.wrapping_sub(self.last_used_idx);

        //CHECK threshold taken from linux, we may want to tune it or set it at maximum outstanding requests
        let threshold =
            ((u32::from(outstanding) * 3) / 4) as u16;

        self.enable_cb_after(threshold)
    }

    /// Enables callbacks from the device after a certain number of used buffers have been processed.
    pub fn enable_cb_after(
        &mut self,
        threshold: u16,
    ) -> bool {
        let last_used_idx =
            self.enable_cb_after_prepare(threshold);

        self.hal.mb();

        if self.event_idx {
            let used = self.used.idx();
            let completed =
                used.wrapping_sub(last_used_idx);

            completed <= threshold
        } else {
            !self.poll(last_used_idx)
        }
    }

    #[inline]
    fn enable_cb_after_prepare(&mut self, threshold: u16) -> u16 {
        if self.event_idx {
            self.avail.set_used_event(
                self.last_used_idx.wrapping_add(threshold),
            );
        } else {
            self.avail.set_flags(0);
        }

        self.last_used_idx
    }
    
    /// Re-enables device callbacks.
    ///
    /// Returns `true` if no used buffer became available while callbacks
    /// were being enabled. Returns `false` if the caller should process
    /// the queue again.
    pub fn enable_cb(&mut self) -> bool {
        self.enable_cb_after(0)
    }

    fn poll(&self, last_used_idx: u16) -> bool {
        last_used_idx != self.used.idx()
    }
    
    /// Detaches one outstanding request from a quiesced virtqueue.
    /// This recycles its descriptor chain and returns the associated token.
    ///
    /// # Safety
    ///
    /// The caller must guarantee that the device can no longer access or
    /// consume descriptors from this virtqueue.
    /// To be used upon reset and reuse of the virtqueue, after the device has been reset.
    //CHECK we may want to remove this and keep take_outstanding_token for the teardown
    //      since this is more expensive but keeps the state consistent, even though the reset
    //      will clear everything regardless.
    //      May be even better if virtqueue is broken, since recycle_chain may fail for corruption
    pub unsafe fn detach_unused(
        &mut self,
    ) -> Result<Option<Token>, VirtQueueError> {
        let mut head = None;

        if self.broken {
            return Err(VirtQueueError::BrokenVirtQueue);
        }

        for index in 0..self.size {
            if self.desc_states()[usize::from(index)]
                .token
                .is_some()
            {
                head = Some(index);
                break;
            }
        }

        let Some(head) = head else {
            return Ok(None);
        };

        let token = self.desc_states()[usize::from(head)]
            .token
            .ok_or(VirtQueueError::CorruptedDescriptor)?;

        // SAFETY: The device has been reset.
        unsafe { self.recycle_chain(head)? };

        self.avail_idx = self.avail_idx.wrapping_sub(1);
        self.avail.set_idx(self.avail_idx);

        Ok(Some(token))
    }

    /// Resets the local virtqueue state to its initial empty state.
    /// This does not reset or disable the device.
    ///
    /// # Safety
    ///
    /// The caller must guarantee that the device can no longer access the
    /// virtqueue or any outstanding descriptor chain.
    pub unsafe fn reset_virtqueue_state(
        &mut self,
    ) -> Result<(), VirtQueueError> {
        if self.has_outstanding_requests() {
            return Err(VirtQueueError::QueueNotEmpty);
        }

        // SAFETY: The device has been reset and the driver has detached all outstanding requests.
        unsafe { self.free_indirect_tables(); }

        //CHECK we could directly zero out all the memory, as the VIRTIO spec says

        self.avail_idx = 0;
        self.last_used_idx = 0;
        self.num_added = 0;
        self.broken = false;

        // SAFETY:
        // The caller guarantees that the device can no longer access the
        // virtqueue or any outstanding descriptor chain.
        unsafe {
            self.init_ring_state();
        }

        self.init_free_list();

        Ok(())
    }

    /// Returns whether the device has placed at least one entry in the used
    /// ring which has not yet been consumed by the driver.
    #[inline]
    pub fn can_pop(&self) -> bool {
        self.last_used_idx != self.used.idx()
    }

    /// Returns the number of free descriptor slots.
    #[inline]
    pub fn num_free(&self) -> u16 {
        self.num_free
    }

    /// Returns the virtqueue index.
    #[inline]
    pub fn queue_index(&self) -> u16 {
        self.queue_idx
    }

    /// Returns the virtqueue size (number of descriptors).
    #[inline]
    pub fn size(&self) -> u16 {
        self.size
    }

    /// Returns whether the virtqueue has outstanding requests.
    #[inline]
    fn has_outstanding_requests(&self) -> bool {
        self.desc_states()
            .iter()
            .any(DescState::is_request_head)
    }

    /// Returns whether the virtqueue is empty (all free descriptors).
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.num_free == self.size
    }

    /// Returns whether the virtqueue has free descriptors.
    #[inline]
    pub fn has_free_descriptors(&self) -> bool {
        self.num_free != 0
    }
    
    /// Returns whether the virtqueue is in a broken state.
    #[inline]
    pub fn is_broken(&self) -> bool {
        self.broken
    }
    
    //TODO pub(super) rather than pub(crate)
    /// Marks the virtqueue as broken.
    #[inline]
    pub(super) fn mark_broken(&mut self) {
        self.broken = true;
    }

    #[inline]
    fn need_event(event: u16, new: u16, old: u16) -> bool {
        new.wrapping_sub(event).wrapping_sub(1)
            < new.wrapping_sub(old)
    }

    #[inline]
    fn desc_states(&self) -> &[DescState] {
        // SAFETY:
        // - `desc_state_memory` was allocated with sufficient size and
        //   alignment for `self.size` DescState objects;
        // - all elements were initialized in `new`;
        // - the allocation remains alive for the lifetime of `self`.
        unsafe {
            core::slice::from_raw_parts(
                self.desc_state_memory
                    .cpu_addr()
                    .cast::<DescState>()
                    .as_ptr(),
                usize::from(self.size),
            )
        }
    }

    #[inline]
    fn desc_states_mut(&mut self) -> &mut [DescState] {
        // SAFETY:
        // Same invariants as `desc_states`; `&mut self` guarantees
        // exclusive access to the allocation.
        unsafe {
            core::slice::from_raw_parts_mut(
                self.desc_state_memory
                    .cpu_addr()
                    .cast::<DescState>()
                    .as_ptr(),
                usize::from(self.size),
            )
        }
    }

    #[inline]
    fn desc_extras(&self) -> &[DescExtra] {
        unsafe {
            core::slice::from_raw_parts(
                self.desc_extra_memory
                    .cpu_addr()
                    .cast::<DescExtra>()
                    .as_ptr(),
                usize::from(self.size),
            )
        }
    }

    #[inline]
    fn desc_extras_mut(&mut self) -> &mut [DescExtra] {
        unsafe {
            core::slice::from_raw_parts_mut(
                self.desc_extra_memory
                    .cpu_addr()
                    .cast::<DescExtra>()
                    .as_ptr(),
                usize::from(self.size),
            )
        }
    }


    #[inline]
    fn ring_memory(&self) -> Result<&DmaRegion, VirtQueueError> {
        self.ring_memory
            .as_ref()
            .ok_or(VirtQueueError::InvalidState)
    }

    //CHECK save directly the addresses instead of offsets, in particular if regions will be separeted
    /// Returns the DMA address of the descriptor table.
    pub fn descriptor_dma_addr(&self) -> Result<DmaAddress, VirtQueueError> {
        Ok(self.ring_memory()?.dma_addr)
    }

    /// Returns the DMA address of the available ring.
    pub fn driver_area_dma_addr(&self) -> Result<DmaAddress, VirtQueueError> {
        Ok(
            self.ring_memory()?.dma_addr
                + self.avail_offset as DmaAddress
        )
    }

    /// Returns the DMA address of the used ring.
    pub fn device_area_dma_addr(&self) -> Result<DmaAddress, VirtQueueError> {
        Ok(
            self.ring_memory()?.dma_addr
                + self.used_offset as DmaAddress
        )
    }

    /// Frees all indirect tables that are still allocated.
    ///
    /// # Safety
    ///
    /// The caller must guarantee that the device can no longer access
    /// any of the indirect tables.
    unsafe fn free_indirect_tables(&mut self) {
        for i in 0..self.size {
            let indirect = self.desc_states_mut()
                [usize::from(i)]
                .indirect
                .take();

            if let Some(indirect) = indirect {
                // SAFETY:
                // The caller guarantees that the device no longer
                // accesses this indirect table.
                unsafe {
                    self.hal.dma_free(indirect);
                }
            }
        }
    }

    /// Removes and returns one outstanding request token or None if there are no outstanding requests.
    /// Can be used even if the virtqueue is broken, to recover the token and free the descriptor chain.
    ///
    /// # Safety
    ///
    /// The caller must guarantee that the device can no longer access
    /// this virtqueue. 
    /// To be used upon destruction of the virtqueue, after the device has been reset.
    pub unsafe fn take_outstanding_token(&mut self) -> Option<Token> {
        for state in self.desc_states_mut().iter_mut() {
            if let Some(token) = state.token.take() {
                return Some(token);
            }
        }

        None
    }

    
    //TODO DELETE FROM HERE ON
    /// Debug functions to inspect the virtqueue state.
    #[inline]
    pub(crate) fn debug_avail_event(&self) -> u16 {
        self.used.avail_event()
    }

    /// Debug functions to inspect the virtqueue state.
    #[inline]
    pub(crate) fn debug_avail_idx(&self) -> u16 {
        self.avail_idx
    }

    /// Debug functions to inspect the virtqueue state.
    #[inline]
    pub(crate) fn debug_used_idx(&self) -> u16 {
        self.used.idx()
    }

    /// Debug functions to inspect the virtqueue state.
    #[inline]
    pub(crate) fn debug_last_used_idx(&self) -> u16 {
        self.last_used_idx
    }
}

//CHECK at the moment we can drop with DMA just configured, causing a possible use after free
//      if misused, moreover the driver loses the tokens
//      we may want to remove this drop impl and make a dedicated function that can be called only after
//      the device has been reset and the driver has detached all outstanding requests
//TODO maybe the virtio framework should contain the virtqueue in a struct where it has to define 
//     drop and perform device reset then this virtqueue drop when struct fields are dropped
//     or directly define here the struct and the trait that implements the device reset and virtqueue drop
impl<H: Hal> Drop for VirtQueue<H> {
    fn drop(&mut self) {

        // SAFETY: The device has been reset and the driver has detached all outstanding requests.
        unsafe { self.free_indirect_tables(); }

        //CHECK drop_in_place() on the MemoryRegion may be necessary for future extensions on the Desc structs
        //      to drop internal droppable fields: for example we may implement drop for DmaRegion and 
        //      automatically drop the indirect tables too in the DescState

        if let Some(memory) = self.ring_memory.take() {
            // SAFETY:
            // The queue owns the allocation and teardown guarantees that
            // the device no longer accesses it.
            unsafe {
                self.hal.dma_free(memory);
            }
        }
    }
}

//CHECK impl send and sync? Are they safe to implement?
//unsafe impl<H: Hal> Send for VirtQueue<H> {}
//unsafe impl<H: Hal> Sync for VirtQueue<H> {}

#[repr(C, align(16))]
#[derive(Copy, Clone)]
pub(crate) struct Descriptor {
    addr: u64,
    len: u32,
    flags: u16,
    next: u16,
}

/*
ptr points to "vring_avail"
C layout:
struct vring_avail {
    u16 flags;      
    u16 idx;        
    u16 ring[n];    
    u16 used_event; (VIRTIO_RING_F_EVENT_IDX)
};
*/
struct AvailRing {
    ptr: NonNull<u8>,
    size: u16,
}

impl AvailRing {
    /// # Safety
    ///
    /// `ptr` must point to an initialized available-ring region large enough
    /// for `size` entries and the optional `used_event` field.
    unsafe fn new(ptr: NonNull<u8>, size: u16) -> Self {
        Self { ptr, size }
    }

    fn flags_ptr(&self) -> *mut u16 {
        self.ptr.as_ptr().cast()
    }

    fn idx_ptr(&self) -> *mut u16 {
        // SAFETY: `idx` immediately follows `flags`.
        unsafe {
            self.ptr
                .as_ptr()
                .add(size_of::<u16>())
                .cast()
        }
    }

    fn ring_ptr(&self) -> *mut u16 {
        // SAFETY: ring[] immediately follows `flags` and `idx`.
        unsafe {
            self.ptr
                .as_ptr()
                .add(2 * size_of::<u16>())
                .cast()
        }
    }

    fn used_event_ptr(&self) -> *mut u16 {
        // SAFETY: used_event immediately follows ring[size].
        unsafe {
            self.ring_ptr()
                .add(usize::from(self.size))
                .cast()
        }
    }

    fn write_ring(
        &self,
        slot: u16,
        descriptor: u16,
    ) {
        unsafe {
            core::ptr::write_volatile(
                self.ring_ptr().add(usize::from(slot)),
                descriptor.to_le(),
            );
        }
    }

    fn set_idx(&self, idx: u16) {
        unsafe {
            core::ptr::write_volatile(
                self.idx_ptr(),
                idx.to_le(),
            );
        }
    }

    fn set_flags(&self, flags: u16) {
        // SAFETY: `flags_ptr` points to the valid available-ring flags.
        unsafe {
            core::ptr::write_volatile(
                self.flags_ptr(),
                flags.to_le(),
            );
        }
    }

    fn set_used_event(&self, idx: u16) {
        // SAFETY: `used_event_ptr` points to the valid event-index field.
        unsafe {
            core::ptr::write_volatile(
                self.used_event_ptr(),
                idx.to_le(),
            );
        }
    }
}

/*
ptr points to "vring_used"
C layout:
struct vring_used {
    u16 flags;        
    u16 idx;             
    vring_used_elem ring[n]; 
    u16 avail_event;  (opzionale)
};
*/
struct UsedRing {
    ptr: NonNull<u8>,
    size: u16,
}
impl UsedRing {
    /// # Safety
    ///
    /// `ptr` must point to an initialized used-ring region large enough for
    /// `size` entries and the optional `avail_event` field.
    unsafe fn new(ptr: NonNull<u8>, size: u16) -> Self {
        Self { ptr, size }
    }

    fn flags_ptr(&self) -> *mut u16 {
        self.ptr.as_ptr().cast()
    }

    fn idx_ptr(&self) -> *mut u16 {
        // SAFETY: `idx` immediately follows `flags`.
        unsafe {
            self.ptr
                .as_ptr()
                .add(size_of::<u16>())
                .cast()
        }
    }

    /// # Safety
    ///
    /// The caller must guarantee that the device can no longer access the
    /// used ring or any outstanding descriptor chain.
    unsafe fn reset(&self) {
        unsafe {
            core::ptr::write_volatile(
                self.flags_ptr(),
                0u16.to_le(),
            );

            core::ptr::write_volatile(
                self.idx_ptr(),
                0u16.to_le(),
            );

            core::ptr::write_volatile(
                self.avail_event_ptr(),
                0u16.to_le(),
            );
        }
    }

    fn ring_ptr(&self) -> *mut UsedElem {
        // SAFETY: ring[] immediately follows `flags` and `idx`.
        unsafe {
            self.ptr
                .as_ptr()
                .add(2 * size_of::<u16>())
                .cast()
        }
    }

    fn avail_event_ptr(&self) -> *mut u16 {
        // SAFETY: avail_event immediately follows ring[size].
        unsafe {
            self.ring_ptr()
                .add(usize::from(self.size))
                .cast()
        }
    }

    fn idx(&self) -> u16 {
        u16::from_le(unsafe {
            core::ptr::read_volatile(self.idx_ptr())
        })
    }

    fn flags(&self) -> u16 {
        // SAFETY: `flags_ptr` points to the valid used-ring flags field.
        u16::from_le(unsafe {
            core::ptr::read_volatile(self.flags_ptr())
        })
    }

    fn read_elem(&self, slot: u16) -> Result<UsedElem, VirtQueueError> {
        if slot >= self.size {
            return Err(VirtQueueError::InvalidParam);
        }

        // SAFETY:
        // - `slot < self.size`.
        // - The used ring contains `self.size` entries.
        let elem = unsafe {
            core::ptr::read_volatile(
                self.ring_ptr().add(usize::from(slot))
            )
        };

        Ok(UsedElem {
            id: u32::from_le(elem.id),
            len: u32::from_le(elem.len),
        })
    }

    fn avail_event(&self) -> u16 {
        // SAFETY: `avail_event_ptr` points to the valid event-index field.
        u16::from_le(unsafe {
            core::ptr::read_volatile(self.avail_event_ptr())
        })
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
struct UsedElem {
    id: u32,
    len: u32,
}

#[repr(transparent)]
#[derive(Copy, Clone, Eq, PartialEq)]
struct DescFlags(u16);

impl DescFlags {
    const NEXT: Self = Self(VIRTQ_DESC_F_NEXT);
    const WRITE: Self = Self(VIRTQ_DESC_F_WRITE);
    const INDIRECT: Self = Self(VIRTQ_DESC_F_INDIRECT);

    #[inline]
    const fn empty() -> Self {
        Self(0)
    }

    #[inline]
    const fn bits(self) -> u16 {
        self.0
    }

    #[inline]
    fn insert(&mut self, flags: Self) {
        self.0 |= flags.0;
    }

    #[allow(dead_code)]
    #[inline]
    fn remove(&mut self, flags: Self) {
        self.0 &= !flags.0;
    }

    #[allow(dead_code)]
    #[inline]
    const fn contains(self, flags: Self) -> bool {
        (self.0 & flags.0) == flags.0
    }
}

/// Errors that can occur when using a virtqueue.
#[derive(Copy, Clone, Eq, PartialEq)]
pub enum VirtQueueError {
    /// Memory allocation of the virtqueue rings or indirect tables failed.
    MemoryAllocationFailed,
    /// The virtqueue size is invalid (zero or not a power of two).
    InvalidQueueSize,
    /// A parameter passed to a virtqueue function is invalid.
    InvalidParam,
    /// The virtqueue is full and cannot accept more buffers.
    QueueFull,
    /// The virtqueue has outstanding requests and cannot be reset.
    QueueNotEmpty,
    /// The device has not negotiated the required feature for the operation.
    FeatureNotNegotiated,
    /// The virtqueue is in a broken state due to this error.
    CorruptedDescriptor,
    /// The virtqueue is in a broken state due to this error.
    InvalidState,
    /// The virtqueue is in a broken state due to this error.
    CorruptedUsedElem,
    /// The virtqueue is in a broken state due to a previous error.
    BrokenVirtQueue,
}

/// Errors that can occur when using a virtqueue, including errors from the HAL.
pub enum Error<E> {
    /// Errors that can occur when using a virtqueue.
    Queue(VirtQueueError),
    /// Errors from the HAL.
    Hal(E),
}

impl<E> From<VirtQueueError> for Error<E> {
    fn from(err: VirtQueueError) -> Self {
        Self::Queue(err)
    }
}

//CHECK changed from rcore -> buffers are mapped outside and directly passed to the 
//      virtqueue functions => less safety (we lose the lifetimes) more flexibilty.
//      caller is responsible for mapping the addresses outside virtqueue core logic,
//      caller prepares DmaSegment(s), may be helpful for VIRTIO_F_ACCESS_PLATFORM,
//      IOMMU implementations, particular Dma mappings without the need 
//      of chainging the core logic. Virtqueue core only implements the logic.
//      Can be extended in the future to support buffer mappings inside virtqueue core logic.
/// A DMA segment that can be added to a virtqueue as a buffer.
pub struct DmaSegment {
    dma_addr: DmaAddress,
    len: u32,
    direction: BufferDirection,
}

impl DmaSegment {
    /// Creates a new DMA segment.
    pub const fn new(
        dma_addr: DmaAddress,
        len: u32,
        direction: BufferDirection,
    ) -> Self {
        Self {
            dma_addr,
            len,
            direction,
        }
    }
}

//CHECK input and output buffer better names?
/// The direction of a buffer in a virtqueue.
#[derive(Copy, Clone, Eq, PartialEq)]
pub enum BufferDirection {
    /// Output buffer from the driver to the device.
    DriverToDevice,
    /// Input buffer from the device to the driver.
    DeviceToDriver,
}

fn queue_part_sizes(
    queue_size: u16,
) -> Result<(usize, usize, usize), VirtQueueError> {
    if queue_size == 0 || !queue_size.is_power_of_two() {
        return Err(VirtQueueError::InvalidQueueSize);
    }

    let n = usize::from(queue_size);
    // SAFETY:
    // - `n` is a power of 2 and `n <= 2^15`, so `n + 3 < 2^16`.
    // - size_of::<Descriptor>() is 16.
    // - size_of::<u16>() is 2.
    // - size_of::<UsedElem>() is 8.
    let desc = size_of::<Descriptor>() * n;
    let avail = size_of::<u16>() * (n + 3);
    let used = size_of::<u16>() * 3 + size_of::<UsedElem>() * n;

    Ok((desc, avail, used))
}

/// Rounds `val` up to the nearest multiple of `align`.
/// `align` must be a power of 2.
fn align_up(
    value: usize,
    align: usize,
) -> Result<usize, VirtQueueError> {
    if align == 0 || !align.is_power_of_two() {
        return Err(VirtQueueError::InvalidParam);
    }

    if align - 1 > usize::MAX - value {
        return Err(VirtQueueError::InvalidParam);
    }

    let adjusted = value + align - 1;

    Ok(adjusted & !(align - 1))
}

/// per-request, meaningful on the head
struct DescState {
    token: Option<Token>,
    indirect: Option<DmaRegion>,
}

impl DescState {
    const fn empty() -> Self {
        Self {
            token: None,
            indirect: None,
        }
    }
    
    #[inline]
    fn is_request_head(&self) -> bool {
        self.token.is_some()
    }
}

/// per-descriptor
#[derive(Copy, Clone)]
struct DescExtra {
    next: u16,
}

/// A token that can be associated with a descriptor chain in a virtqueue.
#[repr(transparent)]
#[derive(Copy, Clone)]
pub struct Token(usize);

impl Token {
    /// Creates a new token from a `usize` value.
    pub const fn new(value: usize) -> Self {
        Self(value)
    }

    /// Returns the `usize` value associated with the token.
    pub const fn value(self) -> usize {
        self.0
    }

    /// Creates a new token from a raw pointer.
    pub fn from_ptr<T>(ptr: *mut T) -> Self {
        Self(ptr.expose_provenance())
    }

    //CHECK deref on the result is still unsafe
    //      remove this function?
    /// Converts the token back to a raw pointer.
    pub fn as_ptr<T>(self) -> *mut T {
        core::ptr::with_exposed_provenance_mut(self.0)
    }
}

/// `indirect_from` must be the index of the first indirect descriptor.
pub enum DescriptorLayout {
    /// Direct layout: all descriptors are direct.
    Direct,
    /// Indirect layout: indirect table.
    Indirect,
    /// Direct then indirect layout: direct descriptors followed by an indirect table.
    DirectThenIndirect {
        /// The index of the first indirect descriptor in the layout.
        indirect_from: usize,
    },
}

/// VIRTIO features negotiated by the device and driver for a virtqueue.
pub struct VirtQueueFeatures {
    /// Whether the device supports the VIRTIO_RING_F_EVENT_IDX feature.
    pub event_idx: bool,
    /// Whether the device supports the VIRTIO_RING_F_INDIRECT_DESC feature.
    pub indirect: bool,
}

/// A Trait that must be implemented by a wrapper of the VirtQueue struct to
/// safely implement the drop of the VirtQueue and guarantee that the device 
/// has been reset and can no longer access the virtqueue or any outstanding descriptor chain
/// before the VirtQueue is dropped.
pub unsafe trait DeviceReset {
    /// Stops the device and guarantees that, when this function returns,
    /// it can no longer access any virtqueue memory associated with it.
    fn reset(&self);
}

/*
pub struct VirtQueues<H, D>
where
    H: Hal,
    D: DeviceReset,
{
    device: D,
    queues: /* storage of VirtQueue<H> */,
} 


impl<H, D> Drop for VirtQueues<H, D>
where
    H: Hal,
    D: DeviceReset,
{
    fn drop(&mut self) {
        self.device.reset();

        
        //CHECK manually drop the virtqueues to guarantuee correct ordering?
        // SAFETY:
        // DeviceReset guarantees that after reset() returns the device
        // can no longer access the virtqueues or their DMA memory.
        unsafe {
            ManuallyDrop::drop(&mut self.queues);
        }
    }
}
*/

/*** 
*
*
* Extra features and optimizations that may be implemented in the future.
*
*


//TODO possible legacy support as argument to new
pub enum QueueLayout {
    Modern,
    Legacy {
        align: usize,
    },
}

//TODO possible optimization for recyling a chain in O(1) instead of O(n).
//     To be returned by the add_* functions
//     then save these data in the desc_state, to be used in recycle_chain
//     without reading the possible corrupted state from descriptors.
//     Otherwise we may use a sentinel guard in DescExtra to find the last descriptor in a chain 
//     from desc_extras instead of Dma shared memory
#[derive(Copy, Clone)]
struct DescriptorChain {
    head: u16,
    last: u16,
    num: u16,
}
***/

//TODO possible optimization for indirect table memory allocation in DescState. We dont
//     need Dma Coherent memory for the indirect table, so we may use Hal::alloc()
//     and then map it to Dma address with Hal::dma_map() and unmap it with Hal::dma_unmap()
//     this also supports future extensions for buffer mappings inside virtqueue core logic
//     and the drop implementation for the indirect table memory
/*
* In HAL:
* pub struct DmaMapping {
*   dma_addr: DmaAddress,
*   size: usize,
*   direction: DmaDirection,
* }
* pub enum DmaDirection {
*     ToDevice,
*     FromDevice,
*     Bidirectional,
* }
*
* struct IndirectState<H: Hal> {
*     memory: MemoryRegion<H>,
*     mapping: DmaMapping,
* }
*/

/*
//CHECK recycle chain optimized and with no errors returned
fn recycle_chain(&mut self, head: u16) {
    let state = &mut self.desc_state[usize::from(head)];

    let num = state.num;
    let last = state.last;

    debug_assert!(state.token.is_some());
    debug_assert!(num != 0);
    debug_assert!(last < self.size);
    debug_assert!(num <= self.size - self.num_free);

    self.desc_extra[usize::from(last)].next = self.free_head;
    self.free_head = head;
    self.num_free += num;

    state.token = None;
    state.num = 0;
    state.last = 0;

    if let Some(indirect) = state.indirect.take() {
        // SAFETY:
        // The device has completed the request, so it can no longer
        // access the indirect descriptor table.
        unsafe {
            self.hal.dma_free(indirect);
        }
    }
}
    */