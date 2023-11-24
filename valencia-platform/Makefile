KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build

prefix ?= /usr/local
includedir = $(prefix)/include

obj-m += valencia-platform.o

valencia-platform-objs := silicom-mec.o

KERNEL_MAKE_OPTS := -C $(KERNEL_DIR) M=$(CURDIR)
ifneq ($(ARCH),)
KERNEL_MAKE_OPTS += ARCH=$(ARCH)
endif
ifneq ($(CROSS_COMPILE),)
KERNEL_MAKE_OPTS += CROSS_COMPILE=$(CROSS_COMPILE)
endif

build:
	$(MAKE) $(KERNEL_MAKE_OPTS) modules

install: modules_install

modules_install:
	$(MAKE) $(KERNEL_MAKE_OPTS) modules_install

clean: 
	$(MAKE) $(KERNEL_MAKE_OPTS) clean
