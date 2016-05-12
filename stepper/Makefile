#------------------------------------------------------------------------------
# To build the driver:  
#   make
#
# To clean-up the built components:  
#   make clean
#------------------------------------------------------------------------------
obj-m   := stepper_spi.o

DRIVER_DIR := $(patsubst %/,%, $(dir $(realpath $(lastword $(MAKEFILE_LIST)))))
INSTALL_DIR = $(INSTALL_DRV_DIR)

all:
	$(MAKE) -C $(KERNEL_DIR) M=$(DRIVER_DIR) ARCH=arm 

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c .*.d *.order *.symvers
	rm -f -r .tmp_versions

install:
	-install -d $(INSTALL_DIR)
	-install -c stepper_spi.ko $(INSTALL_DIR)

refresh:
	-cp -f stepper_spi.ko $(INSTALL_DIR)/stepper_spi.ko

uninstall:
	-rm -f $(INSTALL_DIR)/stepper_spi.ko
