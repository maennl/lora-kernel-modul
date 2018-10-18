PWD := $(shell pwd)
obj-m += lora.o

all:
	make ARCH=arm64 CROSS_COMPILE=$(CROSS) -C $(KERNEL) SUBDIRS=$(PWD) modules
clean:
	make -C $(KERNEL) SUBDIRS=$(PWD) clean
