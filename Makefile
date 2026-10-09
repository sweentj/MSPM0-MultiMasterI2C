SDK ?= /path/to/mspm0-sdk
CC := arm-none-eabi-gcc
DEVICE ?= __MSPM0G3519__
BOARD ?= LP_MSPM0G3519
FREERTOS_CONFIG_DIR ?= $(SDK)/kernel/freertos/builds/$(BOARD)/release

CFLAGS := -std=c99 -Wall -Wextra -Werror -Os -mcpu=cortex-m0plus -mthumb \
	-D$(DEVICE) -Iinclude -I$(SDK)/source \
	-I$(SDK)/source/third_party/CMSIS/Core/Include \
	-I$(SDK)/kernel/freertos/Source/include \
	-I$(SDK)/kernel/freertos/Source/portable/GCC/ARM_CM0 \
	-I$(FREERTOS_CONFIG_DIR)

.PHONY: all clean check-g3519 check-l1116 check-example-g3519 \
	check-example-l1116

all: mmi2c.o

mmi2c.o: src/mmi2c.c include/mmi2c.h
	$(CC) $(CFLAGS) -c $< -o $@

check-g3519:
	$(MAKE) clean all DEVICE=__MSPM0G3519__ BOARD=LP_MSPM0G3519

check-l1116:
	$(MAKE) clean all DEVICE=__MSPM0L1116__ BOARD=LP_MSPM0L1117

check-example-g3519:
	$(MAKE) clean example-core DEVICE=__MSPM0G3519__ BOARD=LP_MSPM0G3519

check-example-l1116:
	$(MAKE) clean example-core DEVICE=__MSPM0L1116__ BOARD=LP_MSPM0L1117

example-core: mmi2c.o ios_packet_sender.o sensor_stub.o

ios_packet_sender.o: examples/apollo3_ios_packet/ios_packet_sender.c \
		examples/apollo3_ios_packet/ios_packet_sender.h include/mmi2c.h
	$(CC) $(CFLAGS) -Iexamples/apollo3_ios_packet -c $< -o $@

sensor_stub.o: examples/apollo3_ios_packet/sensor_stub.c \
		examples/apollo3_ios_packet/sensor_stub.h
	$(CC) $(CFLAGS) -Iexamples/apollo3_ios_packet -c $< -o $@

clean:
	rm -f mmi2c.o ios_packet_sender.o sensor_stub.o
