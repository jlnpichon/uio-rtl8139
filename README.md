This GIT repository contains the uio driver and the userspace driver for
the rtl8139 chip. It also demonstrates how to allocate and share a DMA buffer
in with the userland.
The ultimate goal is to achieve zero copy packet processing in userspace.

# How it works

There is two main components, a minimal kernel driver on one side,
and the userspace that actually drive the network card on the other side.

## Kernel driver

The kernel module has several responsabilities. First it probes the PCI device,
and retrieve iomem address (Base Address Register). Then it exposes through the
UIO system a device in /dev (/dev/uio0) directory. Finally it allocates a DMA buffer
that will be used for the rx ring, and makes it available to userspace through a char
device (/dev/dma_rtl8139).

## Userspace driver

The userspace driver scan the sysfs to retrieve io resources about the PCI device.
It then mmaps the /dev/uio0 device to access the rtl8139 internal registers.
Finally it opens and map the DMA buffer allocated by the kernel module.

# Build & test

'make' will produce the kernel module uio_rtl8139.ko and the user application
user/rtl8139_user

An easy way to test it is to use qemu with an emulated rtl8139. For example:
```
qemu-system-x86_64 -netdev tap,ifname=tap0,id=network0,script=no,downscript=no \
                   -device rtl8139,netdev=network0                             \
                   -kernel ./bzImage -initrd ./rootfs.cpio.gz                  \
                   -append "rdinit=/sbin/init console=ttyS0"                   \
                   -nographic
```

qemu will be started with a rtl8139 linked to the host tap0 device. So you can
send packets to the rtl8139 by using the host tap0 interface. e.g.:
```
tcpreplay -i tap0 mycapfile.pcap
```

On the guest, you have to load the kernel, and then execute the userspace program
```
insmod uio_rtl8139.ko
./rtl8139_user
```

The rtl8139_user binary will dump all incoming packets on the rtl8139 NIC.
