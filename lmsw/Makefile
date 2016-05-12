#------------------------------------------------------------------------------
# To build the driver:  
#   make
#
# To clean-up the built components:  
#   make clean
#------------------------------------------------------------------------------
obj-m   := lmsw.o

DRIVER_DIR := $(patsubst %/,%, $(dir $(realpath $(lastword $(MAKEFILE_LIST)))))
INSTALL_DIR = $(INSTALL_DRV_DIR)

all:
	$(MAKE) -C $(KERNEL_DIR) M=$(DRIVER_DIR) ARCH=arm 
clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c .*.d *.order *.symvers
	rm -f -r .tmp_versions

install:
	-install -d $(INSTALL_DIR)
	-install -c lmsw.ko $(INSTALL_DIR)

refresh:
	-cp -f lmsw.ko $(INSTALL_DIR)/lmsw.ko

uninstall:
	-rm -f $(INSTALL_DIR)/lmsw.ko
