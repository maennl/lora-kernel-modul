PWD := $(shell pwd)
TARGET = lora_module
lora_module-objs := ./lora.o

obj-m += $(TARGET).o


all:
	make ARCH=arm64 CROSS_COMPILE=$(CROSS) -C $(KERNEL) SUBDIRS=$(PWD) modules
clean:
	make -C $(KERNEL) SUBDIRS=$(PWD) clean
