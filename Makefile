# Out-of-tree build for the hid-nintendo2 module.
obj-m := hid-nintendo2.o

KVERSION ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVERSION)/build
PWD := $(shell pwd)

# Build with the same toolchain as the target kernel. Kernels built with clang
# (e.g. CachyOS) require LLVM=1; detect that from the kernel config so the same
# package works on both clang- and gcc-built kernels.
ifeq ($(LLVM),)
LLVM := $(shell grep -qs '^CONFIG_CC_IS_CLANG=y' $(KDIR)/.config && echo 1)
endif
ifeq ($(LLVM),1)
MAKEARGS := LLVM=1
endif

all:
	$(MAKE) -C $(KDIR) M=$(PWD) $(MAKEARGS) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) $(MAKEARGS) clean

install: all
	$(MAKE) -C $(KDIR) M=$(PWD) $(MAKEARGS) modules_install
	depmod -a $(KVERSION)

.PHONY: all clean install
