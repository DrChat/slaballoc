# slaballoc: A `no-std` compatible, thread-safe fixed-size memory allocator
Do you need to allocate objects of a single type on a bare-metal system using `#[no_std]`?
Do you have the requirement that an allocator be concurrently accessed from multiple
threads or CPU cores?

Look no further!

This crate provides a lockless `no_std` compatible slab allocator, for fixed-size
allocations.

This slab allocator takes a fixed-size block of memory, as well as a [Sized]
type for allocations.
The slab allocator will partition the block of memory into two parts: an area
for allocated objects, and an area for an allocation bitmap.

The bitmap will have a bit size of `mem_size / size_of::<T>()`, such that there
is a bit for every possible slot that an object can take.
So, for a 4096 byte block of memory and objects that are 16 bytes each, the
bitmap will take 256 bits, or 32 bytes.

```ignore
| objects........................................ - bitmap |
```

Note that the bitmap will be a little larger than necessary because it accounts
for the entire block of memory (including the part that it takes up). This can
be accounted for, but it's an annoying self-dependency so it'll take some
iterations.

The allocation bitmap can then be accessed locklessly using atomic intrinsics.
When allocating, we'll simply set a bit in the bitmap, and if successful, return
the memory corresponding to that bit.
When deallocating, we can simply unset the bit and the memory is freed.

## Alternatives
If you're using Rust on a hosted environment with `std` available, you might
be interested in using [sharded-slab][] instead. That crate gets you the same
semantics as this one, without having to used a fixed-size memory block.

[sharded-slab]: https://crates.io/crates/sharded-slab
