#![doc = include_str!("../README.md")]
#![no_std]
#![feature(allocator_api, maybe_uninit_slice)]

mod slab;
pub use slab::SlabAllocator;
