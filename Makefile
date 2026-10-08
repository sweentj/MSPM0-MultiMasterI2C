SDK ?= /path/to/mspm0-sdk
CC := arm-none-eabi-gcc
DEVICE ?= __MSPM0G3519__

CFLAGS := -std=c99 -Wall -Wextra -Werror -Os -mcpu=cortex-m0plus -mthumb \
	-D$(DEVICE) -Iinclude -I$(SDK)/source \
	-I$(SDK)/source/third_party/CMSIS/Core/Include

.PHONY: all clean check-g3519 check-l1116 check-example-g3519 \
	check-example-l1116

all: mmi2c.o

mmi2c.o: src/mmi2c.c include/mmi2c.h
	$(CC) $(CFLAGS) -c $< -o $@

check-g3519:
	$(MAKE) clean all DEVICE=__MSPM0G3519__

check-l1116:
	$(MAKE) clean all DEVICE=__MSPM0L1116__

check-example-g3519:
	$(MAKE) clean example-core DEVICE=__MSPM0G3519__

check-example-l1116:
	$(MAKE) clean example-core DEVICE=__MSPM0L1116__

example-core: mmi2c.o ios_packet_sender.o sensor_stub.o

ios_packet_sender.o: examples/apollo3_ios_packet/ios_packet_sender.c \
		examples/apollo3_ios_packet/ios_packet_sender.h include/mmi2c.h
	$(CC) $(CFLAGS) -Iexamples/apollo3_ios_packet -c $< -o $@

sensor_stub.o: examples/apollo3_ios_packet/sensor_stub.c \
		examples/apollo3_ios_packet/sensor_stub.h
	$(CC) $(CFLAGS) -Iexamples/apollo3_ios_packet -c $< -o $@

clean:
	rm -f mmi2c.o ios_packet_sender.o sensor_stub.o
