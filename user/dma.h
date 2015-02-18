#ifndef DMA_H
#define DMA_H

struct rtl8139_dev;

int dma_buffer_open(struct rtl8139_dev *nic);
void dma_buffer_close(struct rtl8139_dev *nic);

#endif /* DMA_H */
