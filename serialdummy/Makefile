ifeq ($(KERNELRELEASE),)

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	rm -rf *.o *.ko .depend *.mod.o *.mod.c Module.* modules.*

.PHONY:	modules modules_installclean

else

obj-m :=serialdummy.o

endif
