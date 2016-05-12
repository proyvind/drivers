#------------------------------------------------------------------------------
# To build the driver:  
#   make
#
# To clean-up the built components:  
#   make clean
#------------------------------------------------------------------------------
obj-m   := dac088.o
obj-m   += vref.o

DRIVER_DIR := $(patsubst %/,%, $(dir $(realpath $(lastword $(MAKEFILE_LIST)))))
INSTALL_DIR = $(INSTALL_DRV_DIR)

all:
	$(MAKE) -C $(KERNEL_DIR) M=$(DRIVER_DIR) ARCH=arm 

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c .*.d *.order *.symvers
	rm -f -r .tmp_versions

install:
	-install -d $(INSTALL_DIR)
	-install -c dac088.ko $(INSTALL_DIR)
	-install -c vref.ko $(INSTALL_DIR)

refresh:
	-cp -f dac088.ko $(INSTALL_DIR)/dac088.ko
	-cp -f vref.ko $(INSTALL_DIR)/vref.ko

uninstall:
	-rm -f $(INSTALL_DIR)/dac088.ko
	-rm -f $(INSTALL_DIR)/vref.ko
