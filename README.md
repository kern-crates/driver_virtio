# driver_virtio

Wrappers of some devices in the `virtio-drivers` crate, that implement traits in the `driver_common` series crates.

Like the `virtio-drivers` crate, you must implement the `VirtIoHal` trait (alias of `virtio-drivers::Hal`), to allocate DMA regions and translate between physical addresses (as seen by devices) and virtual addresses (as seen by your program).

## Examples

```rust
#![no_std]
#![no_main]

#[macro_use]
extern crate axlog2;
extern crate alloc;
use alloc::vec;

use core::panic::PanicInfo;
use driver_common::{BaseDriverOps, DeviceType};
use driver_block::BlockDriverOps;

const DISK_SIZE: usize = 0x1000_0000;    // 256M
const BLOCK_SIZE: usize = 0x200;        // 512

/// Entry
#[no_mangle]
pub extern "Rust" fn runtime_main(cpu_id: usize, _dtb_pa: usize) {
    axlog2::init("debug");
    info!("[rt_driver_virtio]: ...");

    axhal::arch_init_early(cpu_id);

    info!("Initialize global memory allocator...");
    axalloc::init();

    info!("Found physcial memory regions:");
    for r in axhal::mem::memory_regions() {
        info!(
            "  [{:x?}, {:x?}) {} ({:?})",
            r.paddr,
            r.paddr + r.size,
            r.name,
            r.flags
        );
    }

    info!("Initialize kernel page table...");
    page_table::init();

    let mut alldevs = axdriver::init_drivers2();
    let mut disk = alldevs.block.take_one().unwrap();

    assert_eq!(disk.device_type(), DeviceType::Block);
    assert_eq!(disk.device_name(), "virtio-blk");
    assert_eq!(disk.block_size(), BLOCK_SIZE);
    assert_eq!(disk.num_blocks() as usize, DISK_SIZE/BLOCK_SIZE);

    let block_id = 1;

    let mut buf = vec![0u8; BLOCK_SIZE];
    assert!(disk.read_block(block_id, &mut buf).is_ok());

    buf[0] = b'0';
    buf[1] = b'1';
    buf[2] = b'2';
    buf[3] = b'3';

    assert!(disk.write_block(block_id, &buf).is_ok());
    assert!(disk.flush().is_ok());

    assert!(disk.read_block(block_id, &mut buf).is_ok());
    assert!(buf[0..4] == *b"0123");

    info!("[rt_driver_virtio]: ok!");
    axhal::misc::terminate();
}

#[panic_handler]
pub fn panic(info: &PanicInfo) -> ! {
    error!("{}", info);
    arch_boot::panic(info)
}

```

## Re-exports

### `self::blk::VirtIoBlkDev`

```rust
pub struct VirtIoBlkDev<H: Hal, T: Transport> { /* private fields */ }
```

Available on crate feature block only.
The VirtIO block device driver.

#### Implementations

##### `impl<H: Hal, T: Transport> VirtIoBlkDev<H, T>`

```rust
pub fn try_new(transport: T) -> DevResult<Self>
```

Creates a new driver instance and initializes the device, or returns an error if any step fails.

#### Trait Implementations

##### `impl<H: Hal, T: Transport> BaseDriverOps for VirtIoBlkDev<H, T>`

```rust
fn device_name(&self) -> &str
```

The name of the device.

```rust
fn device_type(&self) -> DeviceType
```

The type of the device.

##### `impl<H: Hal, T: Transport> BlockDriverOps for VirtIoBlkDev<H, T>`

```rust
fn num_blocks(&self) -> u64
```

The number of blocks in this storage device. Read more

```rust
fn block_size(&self) -> usize
```

The size of each block in bytes.

```rust
fn read_block(&mut self, block_id: u64, buf: &mut [u8]) -> DevResult
```

Reads blocked data from the given block. Read more

```rust
fn write_block(&mut self, block_id: u64, buf: &[u8]) -> DevResult
```

Writes blocked data to the given block. Read more

```rust
fn flush(&mut self) -> DevResult
```

Flushes the device to write all pending data to the storage.

##### `impl<H: Hal, T: Transport> Send for VirtIoBlkDev<H, T>`

##### `impl<H: Hal, T: Transport> Sync for VirtIoBlkDev<H, T>`

## Modules

### `blk`

Available on crate feature block only.

#### Structs

##### `VirtIoBlkDev`

```rust
pub struct VirtIoBlkDev<H: Hal, T: Transport> { /* private fields */ }
```

The VirtIO block device driver.

### `pci`

Module for dealing with a PCI bus in general, without anything specific to VirtIO.

#### Structs

##### `BusDeviceIterator`

```rust
pub struct BusDeviceIterator { /* private fields */ }
```

An iterator which enumerates PCI devices and functions on a given bus.

##### `CapabilityInfo`

```rust
pub struct CapabilityInfo {
    pub offset: u8,
    pub id: u8,
    pub private_header: u16,
}
```

Information about a PCI device capability.

##### `CapabilityIterator`

```rust
pub struct CapabilityIterator<'a> { /* private fields */ }
```

Iterator over capabilities for a device.

##### `Command`

```rust
pub struct Command { /* private fields */ }
```

The command register in PCI configuration space.

##### `DeviceFunction`

```rust
pub struct DeviceFunction {
    pub bus: u8,
    pub device: u8,
    pub function: u8,
}
```

An identifier for a PCI bus, device and function.

##### `DeviceFunctionInfo`

```rust
pub struct DeviceFunctionInfo {
    pub vendor_id: u16,
    pub device_id: u16,
    pub class: u8,
    pub subclass: u8,
    pub prog_if: u8,
    pub revision: u8,
    pub header_type: HeaderType,
}
```

Information about a PCI device function.

##### `PciRoot`

```rust
pub struct PciRoot { /* private fields */ }
```

The root complex of a PCI bus.

##### `Status`

```rust
pub struct Status { /* private fields */ }
```

The status register in PCI configuration space.

#### Enums

##### `BarInfo`

```rust
pub enum BarInfo {
    Memory {
        address_type: MemoryBarType,
        prefetchable: bool,
        address: u64,
        size: u32,
    },
    IO {
        address: u32,
        size: u32,
    },
}
```

Information about a PCI Base Address Register.

##### `Cam`

```rust
pub enum Cam {
    MmioCam,
    Ecam,
}
```

A PCI Configuration Access Mechanism.

##### `HeaderType`

```rust
pub enum HeaderType {
    Standard,
    PciPciBridge,
    PciCardbusBridge,
    Unrecognised(u8),
}
```

The type of a PCI device function header.

##### `MemoryBarType`

```rust
pub enum MemoryBarType {
    Width32,
    Below1MiB,
    Width64,
}
```

The location allowed for a memory BAR.

##### `PciError`

```rust
pub enum PciError {
    InvalidBarType,
}
```

Errors accessing a PCI device.

## Structs

### `MmioTransport`

```rust
pub struct MmioTransport { /* private fields */ }
```

MMIO Device Register Interface.
Ref: 4.2.2 MMIO Device Register Layout and 4.2.4 Legacy interface

### `PciTransport`

```rust
pub struct PciTransport { /* private fields */ }
```

PCI transport for VirtIO.
Ref: 4.1 Virtio Over PCI Bus

## Enums

### `BufferDirection`

```rust
pub enum BufferDirection {
    DriverToDevice,
    DeviceToDriver,
    Both,
}
```

The direction in which a buffer is passed.

#### `Variants`

**DriverToDevice**
The buffer may be read or written by the driver, but only read by the device.

**DeviceToDriver**
The buffer may be read or written by the device, but only read by the driver.

**Both**
The buffer may be read or written by both the device and the driver.

## Traits

### `Transport`

```rust
pub trait Transport {
    // Required methods
    fn device_type(&self) -> DeviceType;
    fn read_device_features(&mut self) -> u64;
    fn write_driver_features(&mut self, driver_features: u64);
    fn max_queue_size(&mut self, queue: u16) -> u32;
    fn notify(&mut self, queue: u16);
    fn get_status(&self) -> DeviceStatus;
    fn set_status(&mut self, status: DeviceStatus);
    fn set_guest_page_size(&mut self, guest_page_size: u32);
    fn requires_legacy_layout(&self) -> bool;
    fn queue_set(
        &mut self,
        queue: u16,
        size: u32,
        descriptors: usize,
        driver_area: usize,
        device_area: usize
    );
    fn queue_unset(&mut self, queue: u16);
    fn queue_used(&mut self, queue: u16) -> bool;
    fn ack_interrupt(&mut self) -> bool;
    fn config_space<T>(&self) -> Result<NonNull<T>, Error>
       where T: 'static;

    // Provided methods
    fn begin_init(&mut self, negotiate_features: impl FnOnce(u64) -> u64) { ... }
    fn finish_init(&mut self) { ... }
}
```

A VirtIO transport layer.

### `VirtIoHal`

```rust
pub unsafe trait VirtIoHal {
    // Required methods
    fn dma_alloc(
        pages: usize,
        direction: BufferDirection
    ) -> (usize, NonNull<u8>);
    unsafe fn dma_dealloc(paddr: usize, vaddr: NonNull<u8>, pages: usize) -> i32;
    unsafe fn mmio_phys_to_virt(paddr: usize, size: usize) -> NonNull<u8>;
    unsafe fn share(buffer: NonNull<[u8]>, direction: BufferDirection) -> usize;
    unsafe fn unshare(
        paddr: usize,
        buffer: NonNull<[u8]>,
        direction: BufferDirection
    );
}
```

The interface which a particular hardware implementation must implement.

## Functions

### `probe_mmio_device`

```rust
pub fn probe_mmio_device(
    reg_base: *mut u8,
    _reg_size: usize
) -> Option<(DeviceType, MmioTransport)>
```

Try to probe a VirtIO MMIO device from the given memory region.

If the device is recognized, returns the device type and a transport object for later operations. Otherwise, returns None.

### `probe_pci_device`

```rust
pub fn probe_pci_device<H: VirtIoHal>(
    root: &mut PciRoot,
    bdf: DeviceFunction,
    dev_info: &DeviceFunctionInfo
) -> Option<(DeviceType, PciTransport)>
```

Try to probe a VirtIO PCI device from the given PCI address.

If the device is recognized, returns the device type and a transport object for later operations. Otherwise, returns None

## Type Aliases

### `PhysAddr`

```rust
pub type PhysAddr = usize;
```

A physical address as used for virtio.
