#![feature(allocator_api)]
// #![feature(alloc)]
#![feature(strict_provenance)]
#![no_std]

use core::alloc::Allocator;
use embedded_alloc::Heap;

#[global_allocator]
pub static HEAP: Heap = Heap::empty();

extern crate alloc;
use alloc::vec::Vec;
use core::cmp::min;

static mut HEAP_ALLOC_SIZE: Vec<(bool, usize, usize)> = Vec::new();

#[macro_export]
macro_rules! heap_init {
    ($SZ:expr) => {
        static mut HEAP_MEM: [core::mem::MaybeUninit<u8>; $SZ] =
            [core::mem::MaybeUninit::uninit(); $SZ];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, $SZ) }
    };
}

#[no_mangle]
pub extern "C" fn malloc(size: usize) -> *mut u8 {
    if size == 0 {
        return core::ptr::null_mut();
    }
    let p = (unsafe {
        HEAP.allocate(core::alloc::Layout::from_size_align_unchecked(size, 1))
            .unwrap()
            .as_ptr()
    }) as *mut u8;
    if !p.is_null() {
        unsafe {
            // HEAP_ALLOC_SIZE.push((true, p.addr(), size));
            // find first invalid field
            let mut i = 0;
            while i < HEAP_ALLOC_SIZE.len() {
                if !HEAP_ALLOC_SIZE[i].0 {
                    break;
                }
                i += 1;
            }
            if i == HEAP_ALLOC_SIZE.len() {
                HEAP_ALLOC_SIZE.push((true, p as usize, size));
            } else {
                HEAP_ALLOC_SIZE[i] = (true, p as usize, size);
            }
        }
    }
    p
}

#[no_mangle]
pub extern "C" fn free(ptr: *mut u8) {
    if ptr.is_null() {
        return;
    }
    // set addr to invalid
    unsafe {
        let addr = ptr as usize;
        for i in 0..HEAP_ALLOC_SIZE.len() {
            if HEAP_ALLOC_SIZE[i].1 == addr {
                HEAP_ALLOC_SIZE[i].0 = false;
                break;
            }
        }
    }
    unsafe {
        HEAP.deallocate(
            core::ptr::NonNull::new_unchecked(ptr),
            core::alloc::Layout::from_size_align_unchecked(1, 1),
        );
    }
}

#[no_mangle]
pub extern "C" fn realloc(ptr: *mut u8, size: usize) -> *mut u8 {
    if ptr.is_null() {
        return malloc(size);
    }
    if size == 0 {
        free(ptr);
        return core::ptr::null_mut();
    }
    // search HEAP_ALLOC_SIZE for allocated size
    let mut old_size = 0;
    unsafe {
        let addr = ptr as usize;
        for i in 0..HEAP_ALLOC_SIZE.len() {
            if HEAP_ALLOC_SIZE[i].1 == addr {
                old_size = HEAP_ALLOC_SIZE[i].2;
                break;
            }
        }
    }
    if old_size == 0 {
        // not found
        return core::ptr::null_mut();
    }
    let p = malloc(size);
    if !p.is_null() {
        let copy_size = min(size, old_size);
        unsafe {
            core::ptr::copy_nonoverlapping(ptr, p, copy_size);
            free(ptr);
        }
    }
    p
}
