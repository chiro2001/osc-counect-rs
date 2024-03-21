#![feature(allocator_api)]
#![no_std]

use core::alloc::{Allocator, GlobalAlloc};
use embedded_alloc::Heap;

#[global_allocator]
pub static HEAP: Heap = Heap::empty();

// pub fn heap_init<const SZ: usize>() {
//     static mut HEAP_MEM: [MaybeUninit<u8>; SZ] = [MaybeUninit::uninit(); SZ];
//     unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, SZ) }
// }
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
    // unsafe { HEAP.alloc(core::alloc::Layout::from_size_align_unchecked(size, 1)) }
    (unsafe {
        HEAP.allocate(core::alloc::Layout::from_size_align_unchecked(size, 1))
            .unwrap()
            .as_ptr()
    }) as *mut u8
}

#[no_mangle]
pub extern "C" fn free(ptr: *mut u8) {
    if ptr.is_null() {
        return;
    }
    // unsafe { HEAP.dealloc(ptr, core::alloc::Layout::from_size_align_unchecked(1, 1)) }
    unsafe {
        HEAP.deallocate(
            core::ptr::NonNull::new_unchecked(ptr),
            core::alloc::Layout::from_size_align_unchecked(1, 1),
        );
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
        let p = malloc(size);
        if !p.is_null() {
            unsafe {
                core::ptr::copy_nonoverlapping(ptr, p, size);
                free(ptr);
            }
        }
        p
    }
}
