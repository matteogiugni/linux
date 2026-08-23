// SPDX-License-Identifier: GPL-2.0 OR Apache-2.0

use core::{
    mem::size_of,
    ptr::NonNull,
    sync::atomic::{AtomicU16, fence, Ordering},
};

use super::hal::{
    DmaAddress,
    DmaRegion,
    Hal,
};

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

//TODO refactor the code and divide into files
//TODO packed virtqueue
//TODO rollback on failed operation in every function
//TODO HAL as PhantomData or as instance? after HAL complete definition
//TODO if the VirtQueueFeatures will get a lot of fields, we may save this struct directly as a field
//TODO VIRTIO_F_IN_ORDER feature?
//TODO VIRTIO_F_RING_RESET feature?
//TODO VIRTIO_F_ORDER_PLATFORM feature for weak barriers?
//TODO add a generic alloc function in HAL for desc_state and desc_extra for better handling of clean up ops?
//TODO DMA mapping inside or outside the virtqueue core logic?
pub struct VirtQueue<'a, H: Hal> {

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

    desc_state: &'a mut [DescState],
    desc_extra: &'a mut [DescExtra],

    avail_idx: u16,
    //TODO possible future need
    avail_flags: u16,
    last_used_idx: u16,

    event_idx: bool,
    indirect: bool,

    hal: H,
}

impl<'a, H: Hal> VirtQueue<'a, H> {

    pub fn new(
        hal: H,
        queue_idx: u16,
        size: u16,
        desc_state: &'a mut [DescState],
        desc_extra: &'a mut [DescExtra],
        features: VirtQueueFeatures,
    ) -> Result<Self, Error<H::Error>> {
        if desc_state.len() != usize::from(size)
            || desc_extra.len() != usize::from(size)
        {
            return Err(Error::Queue(
                VirtQueueError::InvalidParam,
            ));
        }

        let (desc_size, avail_size, used_size) = queue_part_sizes(size)?;

        let desc_offset = 0;

        let avail_offset =
            align_up(desc_offset + desc_size, AVAIL_ALIGN)?;

        let used_offset =
            align_up(avail_offset + avail_size, USED_ALIGN)?;

        let total_size = used_offset + used_size;

        let ring_memory = hal
            .dma_alloc(total_size, DESC_ALIGN)
            .map_err(Error::Hal)?;

        // TODO if this fails, try to divide the 3 rings into 3 different memory spaces?
        //      or directly allocate them separetely?
        if ring_memory.size < total_size {
            // SAFETY:
            // The region was allocated by this HAL and has not yet been
            // exposed to the device.
            unsafe {
                hal.dma_free(ring_memory);
            }

            return Err(Error::Queue(
                VirtQueueError::MemoryAllocationFailed,
            ));
        }

        // TODO to decide whether the HAL should zero out all
        //      allocated memory or if the virtqueue logic should
        unsafe {
            core::ptr::write_bytes(
                ring_memory.cpu_addr.as_ptr(),
                0,
                total_size,
            );
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

        let mut queue = Self {
            ring_memory: Some(ring_memory),
            desc,
            avail,
            used,
            size,
            queue_idx,
            num_free: size,
            num_added: 0,
            free_head: 0,
            desc_state,
            desc_extra,
            avail_idx: 0,
            avail_flags: 0,
            last_used_idx: 0,
            event_idx: features.event_idx,
            indirect: features.indirect,
            avail_offset,
            used_offset,
            hal,
        };

        queue.init_free_list();

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
                self.desc_extra[usize::from(index)].next;

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

            self.write_desc(
                index,
                segment.dma_addr,
                segment.len,
                flags,
                next,
            );

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
            // The region was allocated by this HAL and has not yet been
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
            // - `self.size` <= 2^15 hence `self.size` + 1 < 2^16.
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

            unsafe {
                core::ptr::write_volatile(
                    indirect_desc.add(i),
                    descriptor,
                );
            }
        }

        let head = self.free_head;
        let next_free = self.desc_extra[usize::from(head)].next;

        self.write_desc(
            head,
            memory.dma_addr,
            // SAFETY:
            // - `indirect_size <= u16 * 16`.
            indirect_size as u32,
            DescFlags::INDIRECT,
            0,
        );

        self.free_head = next_free;
        self.num_free -= 1;

        self.desc_state[usize::from(head)].indirect = Some(memory);

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
        //      if it's the sum, refactor validate_segments
        //TODO add a check to avoid overflow  
        if indirect_segments.len() + direct_segments.len() 
            > usize::from(self.size) - 1 {
            return Err(Error::Queue(
                VirtQueueError::InvalidParam,
            ));
        }

        // SAFETY:
        // - `indirect_segments.len() <= self.size`. ***CHECK***
        // - `self.size` is a `u16`.
        // - `size_of::<Descriptor>()` is 16 bytes.
        let indirect_size = indirect_segments.len() * size_of::<Descriptor>();

        let indirect_memory = self
            .hal
            .dma_alloc(indirect_size, DESC_ALIGN)
            .map_err(Error::Hal)?;

        if indirect_memory.size < indirect_size {
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
            // - `indirect_segments.len() <= self.size`.
            // - `self.size` is a `u16` and a power of 2.
            // - `self.size` <= 2^15 hence `self.size` + 1 < 2^16.
            let next = if i + 1 < indirect_segments.len() {
                flags.insert(DescFlags::NEXT);

                (i + 1) as u16
            } else {
                0
            };

            if segment.direction
                == BufferDirection::DeviceToDriver
            {
                flags.insert(DescFlags::WRITE);
            }

            let descriptor = Descriptor {
                addr: segment.dma_addr.to_le(),
                len: segment.len.to_le(),
                flags: flags.bits().to_le(),
                next: next.to_le(),
            };

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
                self.desc_extra[usize::from(current)].next;

            let mut flags = DescFlags::NEXT;

            if segment.direction
                == BufferDirection::DeviceToDriver
            {
                flags.insert(DescFlags::WRITE);
            }

            self.write_desc(
                current,
                segment.dma_addr,
                segment.len,
                flags,
                next,
            );

            current = next;
        }

        let next_free =
            self.desc_extra[usize::from(current)].next;

        self.write_desc(
            current,
            indirect_memory.dma_addr,
            // SAFETY:
            // - `indirect_size <= u16 * 16`.
            indirect_size as u32,
            DescFlags::INDIRECT,
            0,
        );

        self.free_head = next_free;

        // SAFETY:
        // - Cannot overflow for the above checks.
        self.num_free -= (direct_segments.len() + 1) as u16;

        self.desc_state[usize::from(head)].indirect =
            Some(indirect_memory);

        Ok(head)
    }

    /// Builds the descriptor chain and then
    /// adds it to the available ring.
    ///
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

        self.desc_state[usize::from(head)].token = Some(token);

        let avail_slot = self.avail_idx & (self.size - 1);

        self.avail.write_ring(avail_slot, head);

        self.hal.write_barrier();

        self.avail_idx = self.avail_idx.wrapping_add(1);
        self.avail.set_idx(self.avail_idx);

        self.num_added = self.num_added.wrapping_add(1);

        Ok(())
    }

    /// Adds a single input buffer (device-writable) to the virtqueue.
    ///
    /// `addr` is the DMA address of the buffer.
    /// `len` is the buffer size in bytes.
    /// `token` is an opaque driver-provided value returned by [`Queue::get_buf`]
    ///  when the buffer is used.
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
    /// `addr` is the DMA address of the buffer.
    /// `len` is the buffer size in bytes.
    /// `token` is an opaque driver-provided value returned by [`Queue::get_buf`]
    ///  when the buffer is used.
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
        if !self.can_pop() {
            return Ok(None);
        }

        self.hal.read_barrier();

        let slot = self.last_used_idx & (self.size - 1);
        let elem = self.used.read_elem(slot)?;

        //TODO skip the corrupted entry otherwise it comes back? manage it in someway
        if elem.id >= u32::from(self.size) {
            return Err(VirtQueueError::CorruptedUsedElem);
        }

        let head = elem.id as u16;
        
        let Ok(token) = self.desc_state[usize::from(head)]
            .token
            .ok_or(VirtQueueError::CorruptedDescriptor)?;

        self.recycle_chain(head)?;

        self.last_used_idx = self.last_used_idx.wrapping_add(1);

        Ok(Some((token, elem.len)))
    }

    /// Recycles a descriptor chain back to the free list.
    fn recycle_chain(
        &mut self,
        head: u16,
    ) -> Result<(), VirtQueueError> {

        //CHECK head >= self.size already tested outside the function, but we can move it here

        //CHECK token is none already tested outside the function, but we can move it here

        let mut current = head;
        let mut count = 0u16;

        /*
        * Count how many descriptors are in the chain, and check that the
        * chain is valid.  After the loop, current will be the index of the last descriptor in the chain.
        * This chain will be reused to attach the free list to the end of the chain.
        */
        loop {

            count = count + 1;

            //CHECK a properly constructed chain cannot fail this check
            if count > self.size {
                return Err(VirtQueueError::CorruptedDescriptor);
            }

            let descriptor = unsafe {
                core::ptr::read_volatile(
                    self.desc.as_ptr().add(usize::from(current)),
                )
            };

            let flags = u16::from_le(descriptor.flags);

            if flags & VIRTQ_DESC_F_NEXT == 0 {
                break;
            }

            let next = self.desc_extra[usize::from(current)].next;

            //CHECK a properly constructed chain cannot fail this check
            if next >= self.size {
                return Err(VirtQueueError::CorruptedDescriptor);
            }

            current = next;
        }

        self.desc_extra[usize::from(current)].next = self.free_head;
        self.free_head = head;

        //CHECK a properly constructed chain cannot overflow or surpass self.size, 
        //      but we can add a check anyway
        self.num_free = self.num_free + count;

        self.desc_state[usize::from(head)].token = None;

        if let Some(indirect) =
            self.desc_state[usize::from(head)].indirect.take()
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

    /// Returns whether the virtqueue has any used buffers that can be popped.
    pub fn peek_used(&self) -> Result<bool, VirtQueueError> {
        if !self.can_pop() {
            return Ok(false);
        }

        self.hal.read_barrier();

        let slot = self.last_used_idx & (self.size - 1);
        let elem = self.used.read_elem(slot)?;

        if elem.id >= u32::from(self.size) {
            return Err(VirtQueueError::CorruptedDescriptor);
        }

        Ok(true)
    }

    fn write_desc(
        &mut self,
        index: u16,
        addr: DmaAddress,
        len: u32,
        flags: DescFlags,
        next: u16,
    ) {
        //CHECK index < self.size already tested outside the function, but we can move it here
        //      or directly assume the index is valid

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
            self.desc_state[usize::from(i)] = DescState {
                token: None,
                indirect: None,
            };

            self.desc_extra[usize::from(i)] = DescExtra {
                next: i + 1,
            };
        }

        self.desc_extra[usize::from(self.size - 1)].next = 0;

        self.free_head = 0;
        self.num_free = self.size;
    }

    /// Returns `true` if the device should be notified.
    pub fn kick_prepare(&mut self) -> bool {
        if self.num_added == 0 {
            return false;
        }

        //CHECK is num_added necessary?
        let new = self.avail_idx;
        let old = new.wrapping_sub(self.num_added);

        self.hal.mb();

        let needs_kick = if self.event_idx {
            need_event(self.used.avail_event(), new, old)
        } else {
            (self.used.flags() & VIRTQ_USED_F_NO_NOTIFY) == 0
        };
        //CHECK what if the device removes the no_notify? maybe it's better not to reset it
        //      also a double call to this function will return false the second time
        self.num_added = 0;

        needs_kick
    }

    //CHECK the input->output order of the segments must be validated also
    //      for the direct then indirect layout, or only for the direct and indirect parts of it?
    //CHECK the maximum number of bytes of the chain must be validated also
    //      for the direct then indirect layout, or only for the direct and indirect parts of it?
    fn validate_segments(
        segments: &[DmaSegment],
    ) -> Result<(), VirtQueueError> {
        if segments.is_empty() {
            return Err(VirtQueueError::InvalidParam);
        }

        let mut total_len = 0u64;
        let mut seen_write = false;

        for segment in segments {
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

    //TODO is it necessary? remove it otherwise
    fn init_ring_state(&mut self) {
        self.avail.set_flags(0);
        self.avail.set_idx(0);
        self.avail.set_used_event(0);

        self.used.set_flags(0);
        self.used.set_idx(0);
        self.used.set_avail_event(0);
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

    pub fn virtqueue_enable_cb_delayed() {
        //TODO
    }

    pub fn enable_cb_prepare(&mut self) -> u16 {
        if self.event_idx {
            self.avail.set_used_event(self.last_used_idx);
        } else {
            self.avail.set_flags(0);
        }

        self.last_used_idx
    }
    
    pub fn poll(&self, last_used_idx: u16) -> bool {
        last_used_idx != self.used.idx()
    }
    
    /// Re-enables device callbacks.
    ///
    /// Returns `true` if no used buffer became available while callbacks
    /// were being enabled. Returns `false` if the caller should process
    /// the queue again.
    pub fn enable_cb(&mut self) -> bool {
        let last_used_idx = self.enable_cb_prepare();

        self.hal.mb();

        !self.poll(last_used_idx)
    }
    
    /// Detaches one outstanding buffer that was not used by the device.
    /// Should be called after the device has been reset and the driver has ensured that no more buffers will be used.
    ///
    /// # Safety
    ///
    /// The caller must guarantee that the device can no longer access or
    /// consume descriptors from this virtqueue.
    pub unsafe fn detach_unused(
        &mut self,
    ) -> Result<Option<Token>, VirtQueueError> {
        let mut head = None;

        for index in 0..self.size {
            if self.desc_state[usize::from(index)]
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

        let token = self.desc_state[usize::from(head)]
            .token
            .ok_or(VirtQueueError::CorruptedDescriptor)?;

        self.recycle_chain(head)?;

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
    //CHECK driver loses the saved tokens -> memory leak, should be called after looping detach unused
    pub unsafe fn reset_virtqueue_state(
        &mut self,
    ) -> Result<(), VirtQueueError> {
        //CHECK this should prevent the call before the tokens have been detached
        if self.has_outstanding_requests() {
            return Err(VirtQueueError::QueueNotEmpty);
        }

        for state in self.desc_state.iter_mut() {
            if let Some(indirect) = state.indirect.take() {
                unsafe {
                    self.hal.dma_free(indirect.memory);
                }
            }
        }

        self.avail_idx = 0;
        self.last_used_idx = 0;
        self.num_added = 0;

        self.init_ring_state();

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
    pub fn has_outstanding_requests(&self) -> bool {
        self.num_free != self.size
    }

    /// Returns whether the virtqueue has free descriptors.
    #[inline]
    pub fn has_free_descriptors(&self) -> bool {
        self.num_free != 0
    }

    #[inline]
    fn need_event(event: u16, new: u16, old: u16) -> bool {
        new.wrapping_sub(event).wrapping_sub(1)
            < new.wrapping_sub(old)
    }

    #[inline]
    fn ring_memory(&self) -> Result<&DmaRegion, VirtQueueError> {
        self.ring_memory
            .as_ref()
            .ok_or(VirtQueueError::InvalidState)
    }

    //TODO save directly the addresses instead of offsets, in particular if regions will be separeted
    pub fn descriptor_dma_addr(&self) -> Result<DmaAddress, VirtQueueError> {
        Ok(self.ring_memory()?.dma_addr)
    }

    pub fn driver_area_dma_addr(&self) -> Result<DmaAddress, VirtQueueError> {
        Ok(
            self.ring_memory()?.dma_addr
                + self.avail_offset as DmaAddress
        )
    }

    pub fn device_area_dma_addr(&self) -> Result<DmaAddress, VirtQueueError> {
        Ok(
            self.ring_memory()?.dma_addr
                + self.used_offset as DmaAddress
        )
    }

    fn free_indirect_tables(&mut self) {
        for state in self.desc_state.iter_mut() {
            if let Some(indirect) = state.indirect.take() {
                // SAFETY:
                // The device no longer accesses this queue.
                unsafe {
                    self.hal.dma_free(indirect);
                }
            }
        }
    }
}

//CHECK at the moment we can drop with DMA just configured, causing a possible use after free
//      if misused, moreover the driver loses the tokens
//      we may want to remove this drop impl and make a dedicated function that can be called only after
//      the device has been reset and the driver has detached all outstanding requests
impl<H: Hal> Drop for VirtQueue<'_, H> {
    fn drop(&mut self) {
        self.free_indirect_tables();

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

// CHECK impl send and sync? Are they safe to implement?
//unsafe impl<H: Hal> Send for VirtQueue<H> {}
//unsafe impl<H: Hal> Sync for VirtQueue<H> {}

#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, FromBytes, Immutable, IntoBytes, KnownLayout)]
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
#[derive(Debug)]
struct AvailRing {
    ptr: NonNull<u8>,
    size: u16,
}
//CHECK Atomics are really necessary? is volatile enough?
impl AvailRing {
    /// # Safety
    ///
    /// `ptr` must point to an initialized available-ring region large enough
    /// for `size` entries and the optional `used_event` field.
    unsafe fn new(ptr: NonNull<u8>, size: u16) -> Self {
        Self { ptr, size }
    }

    fn flags_ptr(&self) -> *mut AtomicU16 {
        self.ptr.as_ptr().cast()
    }

    fn idx_ptr(&self) -> *mut AtomicU16 {
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

    fn used_event_ptr(&self) -> *mut AtomicU16 {
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

        Ok(())
    }

    fn set_idx(&self, idx: u16) {
        unsafe {
            (*self.idx_ptr()).store(
                idx.to_le(),
                Ordering::Release,
            );
        }
    }

    fn set_flags(&self, flags: u16) {
        // SAFETY: `flags_ptr` points to the valid available-ring flags.
        unsafe {
            (*self.flags_ptr()).store(flags.to_le(), Ordering::Release);
        }
    }

    fn set_used_event(&self, idx: u16) {
        // SAFETY: `used_event_ptr` points to the event-index field.
        unsafe {
            (*self.used_event_ptr()).store(idx.to_le(), Ordering::Release);
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
#[derive(Debug)]
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

    fn flags_ptr(&self) -> *mut AtomicU16 {
        self.ptr.as_ptr().cast()
    }

    fn idx_ptr(&self) -> *mut AtomicU16 {
        // SAFETY: `idx` immediately follows `flags`.
        unsafe {
            self.ptr
                .as_ptr()
                .add(size_of::<u16>())
                .cast()
        }
    }

    fn set_idx(&self, idx: u16) {
        /*unsafe {
            core::ptr::write_volatile(
                self.idx_ptr(),
                idx.to_le(),
            );
        }*/
        unsafe {
            (*self.idx_ptr()).store(idx.to_le(), Ordering::Release);
        }
    }

    fn set_flags(&self, flags: u16) {
        unsafe {
            (*self.flags_ptr()).store(flags.to_le(), Ordering::Release);
        }
    }

    fn set_avail_event(&self, idx: u16) {
        unsafe {
            (*self.avail_event_ptr()).store(idx.to_le(), Ordering::Release);
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

    fn avail_event_ptr(&self) -> *mut AtomicU16 {
        // SAFETY: avail_event immediately follows ring[size].
        unsafe {
            self.ring_ptr()
                .add(usize::from(self.size))
                .cast()
        }
    }

    fn idx(&self) -> u16 {
        u16::from_le(unsafe {
            (*self.idx_ptr()).load(Ordering::Acquire)
        })
    }

    fn flags(&self) -> u16 {
        // SAFETY: `flags_ptr` points to valid used-ring flags.
        u16::from_le(unsafe {
            (*self.flags_ptr()).load(Ordering::Acquire)
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
        // SAFETY: `avail_event_ptr` points to the event-index field.
        u16::from_le(unsafe {
            (*self.avail_event_ptr()).load(Ordering::Acquire)
        })
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct UsedElem {
    id: u32,
    len: u32,
}

#[derive(
    Copy, Clone, Debug, Default, Eq, FromBytes, Immutable, IntoBytes, KnownLayout, PartialEq,
)]
#[repr(transparent)]
struct DescFlags(u16);

bitflags! {
    impl DescFlags: u16 {
        const NEXT = 1;
        const WRITE = 2;
        const INDIRECT = 4;
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum VirtQueueError {
    MemoryAllocationFailed,
    InvalidQueueSize,
    InvalidParam,
    QueueFull,
    FeatureNotNegotiated,
    CorruptedDescriptor,
    InvalidState,
    CorruptedUsedElem,
}

#[derive(Debug)]
pub enum Error<E> {
    Queue(VirtQueueError),
    Hal(E),
}

impl<E> From<VirtQueueError> for Error<E> {
    fn from(err: VirtQueueError) -> Self {
        Self::Queue(err)
    }
}

//CHECK changed from rcore -> now buffers are mapped outside and directly passed to the 
//      virtqueue functions => less safety (we lose the lifetimes) more flexibilty
//      caller is responsible for mapping the addresses outside virtqueue core logic,
//      caller prepares DmaSegment(s), may be helpful for VIRTIO_F_ACCESS_PLATFORM,
//      IOMMU implementations, particular Dma mappings without chaing the core logic.
//      Maybe it's better to implement a DMA mapping function in HAL trait?
pub struct DmaSegment {
    dma_addr: DmaAddress,
    len: u32,
    direction: BufferDirection,
}

impl DmaSegment {
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
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum BufferDirection {
    DriverToDevice,
    DeviceToDriver,
}

fn queue_part_sizes(
    queue_size: u16,
) -> Result<(usize, usize, usize), VirtQueueError> {
    if queue_size == 0 || !queue_size.is_power_of_two() {
        return Err(VirtQueueError::InvalidQueueSize);
    }

    let n = usize::from(queue_size);
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

    //CHECK can this wrap around? in case add a check
    let adjusted = value + align - 1;

    Ok(adjusted & !(align - 1))
}

struct DescState {
    token: Option<Token>,
    indirect: Option<IndirectState>,
}

impl DescState {
    const fn empty() -> Self {
        Self {
            token: None,
            indirect: None,
        }
    }
    
    //TODO rename it?
    #[inline]
    fn is_request_head(&self) -> bool {
        self.token.is_some()
    }
}

struct IndirectState {
    memory: DmaRegion,
}

#[derive(Clone, Copy)]
struct DescExtra {
    next: u16,
}

#[repr(transparent)]
#[derive(Copy, Clone)]
pub struct Token(usize);

impl Token {
    pub const fn new(value: usize) -> Self {
        Self(value)
    }

    pub const fn value(self) -> usize {
        self.0
    }

    pub fn from_ptr<T>(ptr: *mut T) -> Self {
        Self(ptr.expose_provenance())
    }

    //CHECK deref on the result is still unsafe
    //      remove this function?
    pub fn as_ptr<T>(self) -> *mut T {
        core::ptr::with_exposed_provenance_mut(self.0)
    }
}

pub enum DescriptorLayout {
    Direct,
    Indirect,
    DirectThenIndirect {
        indirect_from: usize,
    },
}

pub struct VirtQueueFeatures {
    pub event_idx: bool,
    pub indirect: bool,
}

//TODO possible legacy support as argument to new
pub enum QueueLayout {
    Modern,
    Legacy {
        align: usize,
    },
}