#ifndef RTL8139_USER_H
#define RTL8139_USER_H

#define MAX_IOS	7

struct iomem_resource {
	char name[32];
	uint64_t addr;
	char *mmapaddr; /* return by mmap */
	uint64_t offset;
	uint64_t size;
};

struct rtl8139_dev {
	char uio_device_name[32]; /* /dev/uio%d */
	int fd; /* handle on /dev/uio%d */

	struct iomem_resource ios[MAX_IOS];
	void *base_addr;

	struct dma_buffer rx_ring;
	int rx_idx; /* current read index */
	int rx_ring_fd;
};

#endif /* RTL8139_USER_H */
