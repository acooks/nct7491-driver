PWD           := $(shell pwd)
DRV_NAME      := nct7491

obj-m += $(DRV_NAME).o
$(DRV_NAME)-objs := $(DRV_OBJS)

.PHONY: clean modules_install

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules 

clean: 
	$(MAKE) -C $(KDIR) M=$(PWD) clean

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
