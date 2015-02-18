#ifndef RTL8139_REG_H
#define RTL8139_REG_H

#define R_RBSTART 0x30 /* Receive Buffer Start Address */

#define	R_ERSR	0x36 /* Early Rx Status Register */

#define ERGood	3 /* Early Rx Good packet */
#define ERBad		2 /* Early Rx Bad packet */
#define EROVW		1 /* Early Rx OverWrite */
#define EROK		0 /* Early Rx OK */

#define R_CR	0x37 /* Command Register */

#define RST		4 /* Reset */
#define RE		3 /* Receiver Enable */
#define TE		2 /* Transmitter Enable */
#define BUFE	0 /* Buffer Empty */

#define R_CAPR 0x38 /* Current Address of Packet Read */

#define R_IMR			0x3c /* Interrupt Mask Register */

#define SERR		15 /* System Error Interrupt */
#define TimeOut	14 /* Time Out Interrupt */
#define LenChg	13 /* Cable Length Change Interrupt */
#define FOVW		6 /* Rx FIFO Overflow Interrupt */
#define PUN			5 /* Packet Underrun */
#define LinkChg	5 /* Link Change Interrrupt */
#define RXOVW		4 /* Rx Buffer Overflow Interrupt */
#define TER			3 /* Transmit Error Interrupt */
#define TOK			2 /* Transmit OK Interrupt */
#define RER			1 /* Receive Error Interrupt */
#define ROK			0 /* Receive OK Interrupt */
#define INT_MASK	(1 << SERR) | (1 << TimeOut) |					\
									(1 << PUN) | (1 << FOVW) |							\
									(1 << TER) | (1 << TOK) | (1 << RER) |	\
									(1 << RXOVW) | (1 << ROK)

#define R_ISR		0x3e /* Interrupt Status Register */

#define SERR		15 /* System Error */
#define TimeOut	14 /* Time Out */
#define LenChg	13 /* Cable Lenght Change */
#define FOVW		6 /* Rx FIFO Overflow */
#define PUN			5 /* Packet Underrun */
#define LinkChg	5 /* Link Change */
#define RXOVW		4 /* Rx Buffer Overflow */
#define TER			3 /* Transmit Error */
#define TOK			2 /* Transmit OK */
#define RER			1 /* Receive Rx Error */
#define ROK			0 /* Receive Rx OK */

#define R_TCR		0x40 /* Transmit Configuration Register */

#define HWVERID_A
#define IFG1			25
#define IFG0			24
#define HWVERID_B
#define LBK1			18
#define LBK0			17
#define CRC				16

#define R_RCR			0x44 /* Receive Configuration Register */

#define ERTH3			27 /* Early Rx Threshold bits 3 */
#define ERTH2			26 /* Early Rx Threshold bits 2 */
#define ERTH1			25 /* Early Rx Threshold bits 1 */
#define ERTH0			24 /* Early Rx Threshold bits 0 */
#define MulERINT	17 /* Multiple early interrupt select */
#define RER8			16 /* Receive Error */
#define RXFTH2		15 /* Rx FIFO Threshold2 */
#define RXFTH1		14 /* Rx FIFO Threshold1 */
#define RXFTH0		13 /* Rx FIFO Threshold0 */
#define NO_RX_FIFO_THRESHOLD (0x07 << 13)
#define RBLEN1		12 /* Rx Buffer Length */
#define RBLEN0		11 /* Rx Buffer Length */
#define RX_BUFFER_LEN_8k  (0 << 11)
#define RX_BUFFER_LEN_16k (1 << 11)
#define RX_BUFFER_LEN_32k (2 << 11)
#define RX_BUFFER_LEN_64k (3 << 11)
#define MXDMA2		10 /* Max DMA Burst Size */
#define MXDMA1		9 /* Max DMA Burst Size */
#define MXDMA0		8 /* Max DMA Burst Size */
#define MXDMA_UNLIMITED (0x07 << 8)
#define WRAP			7 /* */
#define AER				5 /* Accept Error Packet */
#define AR				4 /* Accept Runt */
#define AB				3 /* Accept Broadcast packets */
#define AM				2 /* Accept Multicast packets */
#define APM				1 /* Accept Physical Match packets */
#define AAP				0 /* Accept All Packets */

#define R_MPC	0x4c /* Missed Packet Counter */

#define C9346CR				0x50 /* 93C46 Command Register */
#define ConfigLock		0x00
#define ConfigUnlock	0xc0

#define R_MISR	0x5c /* Multiple Interrupt Select Register */

#define R_MSR		0x58 /* Media Status Register */

/* Packet header bits */
#define FAE		1
#define CRCE	2
#define LONG	3
#define RUNT	4
#define ISE		5
#define BAR		13
#define PAM		14
#define MAR		15

static inline uint8_t ioread8(volatile void *addr)
{
	return *(volatile uint8_t *) addr;
}

static inline uint16_t ioread16(volatile void *addr)
{
	return *(volatile uint16_t *) addr;
}

static inline uint32_t ioread32(volatile void *addr)
{
	return *(volatile uint32_t *) addr;
}

static inline uint64_t ioread64(volatile void *addr)
{
	return *(volatile uint64_t *) addr;
}


static inline void iowrite8(volatile void *addr, uint8_t val)
{
	*(volatile uint8_t *) addr = val;
}

static inline void iowrite16(volatile void *addr, uint16_t val)
{
	*(volatile uint16_t *) addr = val;
}

static inline void iowrite32(volatile void *addr, uint32_t val)
{
	*(volatile uint32_t *) addr = val;
}

#define register_write8(nic, reg, value)	\
	iowrite8(nic->base_addr + reg, value)

#define register_write16(nic, reg, value)	\
	iowrite16((nic)->base_addr + reg, value)

#define register_write32(nic, reg, value)	\
	iowrite32(nic->base_addr + reg, value)

#define register_read8(nic, reg) \
	ioread8(nic->base_addr + reg)

#define register_read16(nic, reg) \
	ioread16((nic)->base_addr + reg)

#define register_read32(nic, reg) \
	ioread32(nic->base_addr + reg)

#endif /* RTL8139_REG_H */
