#include <errno.h>
#include <fcntl.h>
#include <dirent.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "dma.h"
#include "rtl8139_ioctl.h"
#include "rtl8139_reg.h"
#include "rtl8139_user.h"
#include "sysfs.h"

static int map_resource(struct iomem_resource *io, char *uio_filename)
{
	int fd;

	fd = open(uio_filename, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "open %s failed: %s\n", uio_filename, strerror(errno));
		return -1;
	}

	io->mmapaddr = mmap((void *) io->addr, io->size, PROT_READ | PROT_WRITE,
			MAP_SHARED, fd, io->offset);

	if (io->mmapaddr == (void *) -1) {
		fprintf(stderr, "mmap failed: %s\n", strerror(errno));
		close(fd);
		return -1;
	}

	close(fd);

	return 1;
}

static void unmap_resource(struct iomem_resource *io)
{
	int r;

	r = munmap(io->mmapaddr, io->size);
	if (r < 0)
		fprintf(stderr, "munmap failed: %s\n", strerror(errno));
}

int pci_resources_map(struct rtl8139_dev *nic, char *directory)
{
	DIR *dir;
	int i;
	char path[2048];
	char filename[2048];
	int r;

	for (i = 0; i < MAX_IOS; i++) {
		snprintf(path, sizeof(path), "%s/maps/map%d", directory, i);

		r = access(path, F_OK);
		if (r != 0)
			continue;

		snprintf(filename, sizeof(filename), "%s/name", path);
		r = parse_sysfs_string(filename, nic->ios[i].name, sizeof(nic->ios[i].name));
		if (r <= 0)
			return -1;

		snprintf(filename, sizeof(filename), "%s/addr", path);
		r = parse_sysfs_int(filename, &nic->ios[i].addr);
		if (r <= 0)
			return -1;

		snprintf(filename, sizeof(filename), "%s/offset", path);
		r = parse_sysfs_int(filename, &nic->ios[i].offset);
		if (r <= 0)
			return -1;

		snprintf(filename, sizeof(filename), "%s/size", path);
		r = parse_sysfs_int(filename, &nic->ios[i].size);
		if (r <= 0)
			return -1;

		r = map_resource(&nic->ios[i], nic->uio_device_name);
		if (r <= 0)
			return -1;

		fprintf(stderr, "resource %s addr: %lx offset: %lx size: %lx mmap'ed at %p\n",
				nic->ios[i].name, nic->ios[i].addr,
				nic->ios[i].offset, nic->ios[i].size,
				nic->ios[i].mmapaddr);
	}

	return 1;
}

void pci_resources_unmap(struct rtl8139_dev *nic)
{
	int i;

	for (i = 0; i < MAX_IOS; i++)
		unmap_resource(&nic->ios[i]);
}

int uio_device_find(struct rtl8139_dev *nic,
		char *directory, size_t directory_len)
{
	DIR *dir;
	struct dirent *e;
	char *endptr;
	unsigned long n;

	dir = opendir("/sys/bus/pci/devices/0000:00:04.0/uio/");
	if (dir == NULL)
		return -1;

	while ((e = readdir(dir)) != NULL) {
		if (strncmp("uio", e->d_name, 3) != 0)
			continue;

		errno = 0;
		endptr = NULL;
		n = strtoul(e->d_name + 3, &endptr, 10);
		if (errno != 0 || endptr == NULL || *endptr) {
			fprintf(stderr, "strtoul failed\n");
		 continue;	
		}

		snprintf(nic->uio_device_name, sizeof(nic->uio_device_name),
				"/dev/uio%d", n);
		snprintf(directory, directory_len,
				"/sys/bus/pci/devices/0000:00:04.0/uio/uio%d", n);
		break;
	}

	closedir(dir);

	return 1;
}

void rtl8139_print_mac_addr(struct rtl8139_dev *nic)
{
	int i;
	uint8_t v;
	uint32_t low;
	uint32_t high;

	for (i = 0; i < 6; i++) {
		v = ioread8(nic->base_addr + i);
		fprintf(stderr, "%02x", v);
		if (i != 5)
			fprintf(stderr, ":");
	}
	fprintf(stderr, "\n");
}

void rtl8139_init(struct rtl8139_dev *nic)
{
	uint32_t r;

	/* software reset */
	r = register_read8(nic, R_CR);
	register_write8(nic, R_CR, r | (1 << RST));
	do {
		/* memory barrier */
		__asm__ __volatile__("" : : : "memory");
		r = register_read8(nic, R_CR);
	} while (r & (1 << RST));

	register_write8(nic, R_CR, r | (1 << RE));

	register_write32(nic, R_RCR, 
			/* (1 << AB) | (1 << AM) | (1 << APM) | */
			(1 << AAP) |
			(1 << WRAP) |
			RX_BUFFER_LEN_32k |
			MXDMA_UNLIMITED);

	register_write32(nic, R_RBSTART, htole32((uint32_t)nic->rx_ring.dma_addr));

	register_write32(nic, R_MPC, 0);

	/* no early-rx interrupts */
	r = register_read16(nic, R_MISR);
	register_write16(nic, R_MISR, r & 0xf000);

	r = register_read32(nic, R_TCR);
	fprintf(stderr, "hardware version id: 0x%02x\n", ((r >> 22) & 0x03) | ((r >> 24) & 0x7c));

	/* enable all known interrupts */
	register_write16(nic, R_IMR, INT_MASK);
}

static int rtl8139_interrupt_enable(struct rtl8139_dev *nic)
{
	int r;
	int value;

	value = 1;
	r = write(nic->fd, &value, sizeof(value));
	if (r != sizeof(value)) {
		fprintf(stderr, "write failed: %s\n", strerror(errno));
		return -1;
	}

	return 1;
}

int packet_header_check(uint16_t pkt_hdr)
{
	if (!(pkt_hdr & (1 << ROK)) ||
			(pkt_hdr & (1 << RUNT)) ||
			(pkt_hdr & (1 << LONG)) ||
			(pkt_hdr & (1 << CRCE)) ||
			(pkt_hdr & (1 << FAE))) {
		return -1;
	}

	return 1;
}

static void rtl8139_reset_rx(struct rtl8139_dev *nic)
{
	uint8_t tmp;

	tmp = register_read8(nic, R_CR);
	register_write8(nic, R_CR, tmp & (0 << RE));
	register_write8(nic, R_CR, tmp);

	register_write32(nic, R_RCR, 
			(1 << AAP) |
			(1 << WRAP) |
			RX_BUFFER_LEN_32k |
			MXDMA_UNLIMITED);

	nic->rx_idx = 0;
}

static void recv_callback(uint8_t *payload, uint16_t len)
{
	int i;

	fprintf(stderr, "Packet length: 0x%04x\n", len);

	for (i = 0; i < len; i++)
		fprintf(stderr, "0x%02x ", payload[i]);
	fprintf(stderr, "\n");
}

static int rtl8139_receive_packets(struct rtl8139_dev *nic)
{
	uint8_t *rx_ring;
	int recvd;
	int r;

#define RECV_PKT_MAX 64 
	rx_ring = (uint8_t *) nic->rx_ring.mmap_addr;
	recvd = 0;

	while (recvd < RECV_PKT_MAX) {
		uint8_t cmd;
		uint16_t pkt_len;

		/* rx buffer empty ? */
		cmd = register_read8(nic, R_CR);
		if (cmd & (1 << BUFE)) {
			fprintf(stderr, "buffer empty (BUFE)\n");
			break;
		}

		r = packet_header_check(le16toh(*((uint16_t *) &rx_ring[nic->rx_idx])));
		if (r <= 0) {
			fprintf(stderr, "invalid packet, dropping it, and resetting Rx\n");
			rtl8139_reset_rx(nic);
			return 0;
		}

		pkt_len = le16toh(*((uint16_t *) &rx_ring[nic->rx_idx + 2]));

		recv_callback(&rx_ring[nic->rx_idx + 4], pkt_len);

		recvd++;

		/* update read pointer */
		nic->rx_idx = (nic->rx_idx + pkt_len + 4 + 3) & (~3);
		register_write16(nic, R_CAPR, nic->rx_idx - 0x10);

		nic->rx_idx %= RX_BUF_TOT_LEN;
	}

	return recvd;
}

static void rtl8139_interrupt(struct rtl8139_dev *nic)
{
	uint16_t isr;

	isr = register_read16(nic, R_ISR);
	/* clear ISR to acknowledge irq */
	register_write16(nic, R_ISR, 0xffff);

	if (isr == 0)
		goto out;

	if (isr & (1 << ROK))
		rtl8139_receive_packets(nic);

out:
	/* re-enable low level pci interrupts that has been disabled
	 * by the UIO driver */
	rtl8139_interrupt_enable(nic);
}

void handle_interrupt(int fd, struct rtl8139_dev *nic)
{
	struct epoll_event ev;
	struct epoll_event events[10];
	int epfd;
	int r;
	int i;

	epfd = epoll_create(10);

	ev.events = EPOLLIN | EPOLLPRI | EPOLLERR | EPOLLHUP;
	ev.data.fd = fd;
	r = epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev);
	if (r < 0) {
		fprintf(stderr, "epoll_ctl failed: %s\n", strerror(errno));
		return;
	}

	while (1) {
		int nfds;
		int n;

		nfds = epoll_wait(epfd, events, 10, -1);
		if (nfds < 0) {
			fprintf(stderr, "epoll_wait failed: %s\n", strerror(errno));
			return;
		}

		for (i = 0; i < nfds; i++) {
			read(events[i].data.fd, &n, sizeof(n));
			rtl8139_interrupt(nic);
		}
	}

	close(epfd);
}

int main(int argc, char *argv[])
{
	struct rtl8139_dev nic;
	int r;
	char resources_directory[1024];

	memset(&nic, 0, sizeof(nic));

	uio_device_find(&nic, resources_directory, 1024);
	pci_resources_map(&nic, resources_directory);

	/* prevent from being swapped out */
	mlockall(MCL_CURRENT | MCL_FUTURE);

	/* The first io resource is the base addresse to
	 * access the NIC registers */
	nic.base_addr = nic.ios[0].mmapaddr;

	r = dma_buffer_open(&nic);
	if (r < 0) {
		fprintf(stderr, "dma_buffer_open failed\n");
		goto out;
	}

	rtl8139_init(&nic);
	rtl8139_print_mac_addr(&nic);

	nic.fd = open(nic.uio_device_name, O_RDWR);
	handle_interrupt(nic.fd, &nic);
	close(nic.fd);

	dma_buffer_close(&nic);
out:
	pci_resources_unmap(&nic);

	munlockall();

	return 0;	
}
