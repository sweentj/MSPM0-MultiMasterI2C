SDK ?= /path/to/mspm0-sdk
CC := arm-none-eabi-gcc
DEVICE ?= __MSPM0G3519__

CFLAGS := -std=c99 -Wall -Wextra -Werror -Os -mcpu=cortex-m0plus -mthumb \
	-D$(DEVICE) -Iinclude -I$(SDK)/source \
	-I$(SDK)/source/third_party/CMSIS/Core/Include

.PHONY: all clean check-g3519 check-l1116

all: mmi2c.o

mmi2c.o: src/mmi2c.c include/mmi2c.h
	$(CC) $(CFLAGS) -c $< -o $@

check-g3519:
	$(MAKE) clean all DEVICE=__MSPM0G3519__

check-l1116:
	$(MAKE) clean all DEVICE=__MSPM0L1116__

clean:
	rm -f mmi2c.o
