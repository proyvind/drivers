#------------------------------------------------------------------------------
# Makefile for stepper_spi test routine
#------------------------------------------------------------------------------
CC      = arm-linux-gnueabihf-gcc
CFLAGS  = -g -O2 -Wall -I../

REG_BIN  = stepper

REG_OBJS = stepper.o

all:    Makefile $(REG_BIN) $(IMG_BIN) 

$(REG_BIN): $(REG_OBJS)
	$(CC) $(CFLAGS) -o $@ $(REG_OBJS)

%.o: %.c 
	$(CC) -c $(CFLAGS) -o $@ $<

clean: 
	@rm -f $(REG_BIN) $(REG_OBJS) 
