#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>

#include "rtl8139_ioctl.h"
#include "dma.h"
#include "rtl8139_user.h"

#define CHARDEV_NAME "/dev/dma_rtl8139"

int dma_buffer_open(struct rtl8139_dev *nic)
{
	int r;

	nic->rx_ring_fd = open(CHARDEV_NAME, O_RDWR);
	if (nic->rx_ring_fd < 0) {
		fprintf(stderr, "Can't open %s\n", CHARDEV_NAME);
		return -1;
	}
	
	r = ioctl(nic->rx_ring_fd, IOCTL_DMA_BUFFER, &nic->rx_ring);
	if (r < 0) {
		fprintf(stderr, "ioctl failed: %s\n", errno);
		close(nic->rx_ring_fd);
		return -1;
	}

	nic->rx_ring.mmap_addr = mmap(NULL, RX_BUF_TOT_LEN, PROT_READ | PROT_WRITE,
			MAP_SHARED, nic->rx_ring_fd, 0);
	if (nic->rx_ring.mmap_addr == (char *) MAP_FAILED) {
		close(nic->rx_ring_fd);
		fprintf(stderr, "mmap failed: %s\n", strerror(errno));
		return -1;
	}

	fprintf(stderr, "mmap_addr: %p virtual_addr: %p dma_addr: %p size: %ld\n",
			nic->rx_ring.mmap_addr, nic->rx_ring.virtual_addr,
			nic->rx_ring.dma_addr, nic->rx_ring.size);

	return 1;
}

void dma_buffer_close(struct rtl8139_dev *nic)
{
	close(nic->rx_ring_fd);
}

