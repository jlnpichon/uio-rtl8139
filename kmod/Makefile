PWD = $(shell pwd)

DEBUG = y

ifeq ($(DEBUG),y)
  DEBUG_CFLAGS = -O -g -DDEBUG_RTL8139
else
  DEBUG_CFLAGS =
endif

EXTRA_CFLAGS += $(DEBUG_CFLAGS)

#In tree kernel module build ?
ifneq ($(KERNELRELEASE),)

obj-m += uio_rtl8139.o

else

KDIR ?= /lib/modules/$(shell uname -r)/build

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

endif

clean:
	rm -rf *~ core .depend Module.symvers modules.order
	rm -rf *.o .*.cmd *.ko *.mod.c .tmp_versions

depend .depend dep:
	$(CC) $(EXTRA_CFLAGS) -M *.c > .depend


ifeq (.depend,$(wildcard .depend))
	include .depend
endif
