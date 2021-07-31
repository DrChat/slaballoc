use core::mem::MaybeUninit;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::Ordering;
use core::{marker::PhantomData, ptr::NonNull, result::Result, sync::atomic::AtomicU8};

/// Atomically test and set a bit in an array.
fn atomic_bts(arr: &[AtomicU8], idx: usize) -> Option<bool> {
    let el = arr.get(idx / 8)?;
    let bit = (idx % 8) as u8;

    let val = el.fetch_or(1 << bit, Ordering::Relaxed);
    Some((val & (1 << bit)) != 0)
}

/// Atomically test and clear a bit in an array.
fn atomic_btc(arr: &[AtomicU8], idx: usize) -> Option<bool> {
    let el = arr.get(idx / 8)?;
    let bit = (idx % 8) as u8;

    let val = el.fetch_and(!(1 << bit), Ordering::Relaxed);
    Some((val & (1 << bit)) != 0)
}

/// Perform a division, preferring to round up.
fn div_ceil(num: usize, dem: usize) -> usize {
    (num + dem) / dem
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlabError {
    /// The base address of the heap is not aligned properly.
    BadBaseAlignment,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlabAllocError {
    /// The heap was observed to be exhausted at the time of the allocation.
    HeapExhausted,
}

unsafe impl<T> Send for SlabAllocator<T> {}
unsafe impl<T> Sync for SlabAllocator<T> {}

/// A basic fixed-size slab allocator covering a range of memory.
#[derive(Debug)]
pub struct SlabAllocator<T: Sized> {
    _data: PhantomData<T>,

    /// The total number of elements in this slab allocator.
    num_elems: usize,
    /// The bitmap slice.
    bitmap: &'static mut [AtomicU8],
    /// The data slice.
    data: *mut [MaybeUninit<T>],
}

impl<T: Sized> SlabAllocator<T> {
    /// Create a new instance of this allocator.
    /// If the input parameters are invalid, this will return a [SlabError].
    pub fn new(mem: *mut MaybeUninit<u8>, size: usize) -> Result<Self, SlabError> {
        // Ensure the size is aligned to the size of the contained objects.
        if size & (core::mem::size_of::<T>() - 1) != 0 {
            Err(SlabError::BadBaseAlignment)?;
        }

        // Calculate the size of the data segment, subtracting out the ideal
        // bitmap size.
        let data_size = size - div_ceil(size / core::mem::size_of::<T>(), 8);

        // Partition off the data first.
        // FIXME: Does this ensure the alignment of elements?
        let data = core::ptr::slice_from_raw_parts_mut(
            mem as *mut MaybeUninit<T>,
            data_size / core::mem::size_of::<MaybeUninit<T>>(),
        );

        // Calculate the actual number of elements that can be stored in the data segment.
        let num_elems = data_size / core::mem::size_of::<T>();
        let bitmap_size = div_ceil(num_elems, 8);

        // Slice off the bitmap, taking care to initialize it.
        let bitmap = {
            let bitmap = unsafe {
                core::slice::from_raw_parts_mut(
                    mem.add(size - bitmap_size) as *mut MaybeUninit<AtomicU8>,
                    bitmap_size,
                )
            };

            for b in bitmap.iter_mut() {
                b.write(AtomicU8::new(0));
            }

            unsafe { MaybeUninit::slice_assume_init_mut(bitmap) }
        };

        Ok(Self {
            _data: PhantomData,

            num_elems,
            bitmap,
            data,
        })
    }

    /// Attempts to allocate a slot in our bitmap.
    fn allocate_slot(&self) -> Option<usize> {
        for i in 0..self.num_elems {
            if let Some(false) = atomic_bts(self.bitmap, i) {
                return Some(i);
            }
        }

        None
    }

    fn deallocate_slot(&self, slot: usize) {
        atomic_btc(self.bitmap, slot);
    }

    fn allocate_raw(&self) -> Result<&mut MaybeUninit<T>, SlabAllocError> {
        let slot = self.allocate_slot().ok_or(SlabAllocError::HeapExhausted)?;

        let data = unsafe { &*self.data };
        let el_ptr = &data[slot] as *const _ as *mut MaybeUninit<T>;

        // Since we have an exclusive slot, we can safely pass out a mutable pointer.
        Ok(unsafe { &mut *el_ptr })
    }

    unsafe fn deallocate_raw(&self, elm: *mut T) {
        // Calculate the slot by finding the offset from our data buffer.
        let slot = elm.offset_from(self.data as *const T);
        if slot < 0 || (slot as usize) >= self.num_elems {
            panic!("Deallocate given an element that does not belong to us!");
        }

        self.deallocate_slot(slot as usize);
    }

    pub fn allocate<'a>(&'a self, item: T) -> Result<SlabAlloc<'a, T>, SlabAllocError> {
        let e = unsafe {
            let e = self.allocate_raw()?;
            e.as_mut_ptr().write(item);
            e.assume_init_mut()
        };

        Ok(SlabAlloc(self, unsafe { NonNull::new_unchecked(e) }))
    }
}

/// A container structure for an allocation made in a [SlabAllocator].
/// This is similar to a [core::alloc::Box].
#[derive(Debug)]
pub struct SlabAlloc<'a, T>(&'a SlabAllocator<T>, NonNull<T>);

impl<T> Drop for SlabAlloc<'_, T> {
    fn drop(&mut self) {
        unsafe {
            core::ptr::drop_in_place(self.1.as_ptr());
            self.0.deallocate_raw(self.1.as_ptr())
        };
    }
}

impl<T> Deref for SlabAlloc<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe { self.1.as_ref() }
    }
}

impl<T> DerefMut for SlabAlloc<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { self.1.as_mut() }
    }
}

#[cfg(test)]
mod test {
    // Allow use of std in tests.
    extern crate std;

    use core::alloc::Layout;
    use std::vec;
    use std::vec::Vec;

    #[test]
    fn test_atomic_bt() {
        use super::*;

        let arr = vec![AtomicU8::new(0), AtomicU8::new(0)];

        // Twiddle a single bit.
        assert_eq!(Some(false), atomic_bts(&arr, 0));
        assert_eq!(Some(true), atomic_bts(&arr, 0));
        assert_eq!(Some(true), atomic_btc(&arr, 0));
        assert_eq!(Some(false), atomic_bts(&arr, 0));
        assert_eq!(Some(true), atomic_btc(&arr, 0));

        // Twiddle two bits, ensuring the two do not interfere with one another.
        assert_eq!(Some(false), atomic_bts(&arr, 1));
        assert_eq!(Some(false), atomic_bts(&arr, 2));
        assert_eq!(Some(true), atomic_bts(&arr, 2));
        assert_eq!(Some(true), atomic_btc(&arr, 2));
        assert_eq!(Some(true), atomic_bts(&arr, 1));
        assert_eq!(Some(true), atomic_btc(&arr, 1));

        // Ditto, in two separate bytes.
        assert_eq!(Some(false), atomic_bts(&arr, 0));
        assert_eq!(Some(false), atomic_bts(&arr, 8));
        assert_eq!(Some(true), atomic_bts(&arr, 8));
        assert_eq!(Some(true), atomic_btc(&arr, 8));
        assert_eq!(Some(true), atomic_bts(&arr, 0));
        assert_eq!(Some(true), atomic_btc(&arr, 0));

        // Test OOB access.
        assert_eq!(None, atomic_bts(&arr, 20));
    }

    #[test]
    fn test_basic() {
        use super::*;

        #[derive(Debug)]
        struct Element {
            data: usize,
        }

        let layout = Layout::from_size_align(16384, 128).unwrap();
        let mem = unsafe { std::alloc::alloc(layout) };

        let alloc: SlabAllocator<Element> =
            SlabAllocator::new(mem as *mut MaybeUninit<u8>, layout.size()).unwrap();

        let mut allocs = Vec::new();

        // N.B: There should be 2016 slots available.
        for i in 0..32 {
            allocs.push(alloc.allocate(Element { data: i as usize }).unwrap());
        }

        drop(allocs);
        drop(alloc);
        unsafe { std::alloc::dealloc(mem, layout) };
    }

    /// Test a small slab allocator and ensure it properly reports heap exhaustion.
    #[test]
    fn test_small() {
        use super::*;

        #[derive(Debug)]
        struct Element {
            data: usize,
        }

        let layout = Layout::from_size_align(32, 128).unwrap();
        let mem = unsafe { std::alloc::alloc(layout) };

        let alloc: SlabAllocator<Element> =
            SlabAllocator::new(mem as *mut MaybeUninit<u8>, layout.size()).unwrap();

        let mut allocs = Vec::new();

        // N.B: There are only 3 slots available.
        allocs.push(alloc.allocate(Element { data: 0 as usize }).unwrap());
        allocs.push(alloc.allocate(Element { data: 1 as usize }).unwrap());
        allocs.push(alloc.allocate(Element { data: 2 as usize }).unwrap());
        assert_eq!(
            SlabAllocError::HeapExhausted,
            alloc.allocate(Element { data: 3 as usize }).unwrap_err()
        );

        assert_eq!(allocs[0].data, 0);
        assert_eq!(allocs[1].data, 1);
        assert_eq!(allocs[2].data, 2);

        drop(allocs);
        drop(alloc);
        unsafe { std::alloc::dealloc(mem, layout) };
    }
}
