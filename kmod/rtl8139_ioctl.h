#ifndef RTL8139_IOCTL_H
#define RTL8139_IOCTL_H

#include <linux/ioctl.h>

#define RX_BUF_LEN_SHIFT 2
#define RX_BUF_LEN (8192 << RX_BUF_LEN_SHIFT)
#define RX_BUF_PAD 16
#define RX_BUF_WRAP_PAD 2048
#define RX_BUF_TOT_LEN (RX_BUF_LEN + RX_BUF_PAD + RX_BUF_WRAP_PAD)

#ifndef __KERNEL__
#define PCI_VENDOR_ID_REALTEK				0x10ec
#define PCI_DEVICE_ID_REALTEK_8139	0x8139
#endif

struct dma_buffer {
#ifdef __KERNEL__
	dma_addr_t dma_addr; /* bus address */
#else
	uint32_t dma_addr; /* bus address */
#endif
	void *virtual_addr; /* kernel virtual address */
	void *mmap_addr; /* user virtual address */
	size_t size;
};

#define IOCTL_DMA_BUFFER _IOR('d', 1, struct dma_buffer)

#endif /* RTL8139_IOCTL_H */
