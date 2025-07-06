#![doc = include_str!("../README.md")]
#![no_std]

mod slab;
pub use slab::{SlabAlloc, SlabAllocError, SlabAllocator};
