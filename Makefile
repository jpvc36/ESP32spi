# Toolchain and kernel
CROSS_COMPILE :=
KERNEL := /lib/modules/$(shell uname -r)/build
ARCH := arm

# Normalize ARCH
ifeq ($(ARCH), x86_64)
    ARCH := x86
endif
ifeq ($(ARCH), aarch64)
    ARCH := arm64
endif

MODULE_NAME := esp32_spi

# Debug and AP support
    EXTRA_CFLAGS += -DCONFIG_AP_MODE

# Source and include paths
PWD := $(shell pwd)
ccflags-y += -I$(src) -I$(src)/include -I$(CURDIR) -I$(CURDIR)/include
EXTRA_CFLAGS += -I$(M) -I$(M)/include

# Common source files
module_objects += esp_bt.o main.o esp_cmd.o esp_utils.o esp_cfg80211.o esp_stats.o esp_debugfs.o esp_log.o esp_spi.o 
CFLAGS_esp_log.o = -DDEBUG

# Module build rules
obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-y := $(module_objects)

# Build targets
all: clean
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) modules

clean:
	rm -rf *.o */*.o *.ko *.mod.c *.symvers *.order .*.cmd .tmp_versions
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) clean

check:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) $(module_objects)

