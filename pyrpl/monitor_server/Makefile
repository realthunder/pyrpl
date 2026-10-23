.PHONY: all monitor_server_zcu216
SHELL:=/bin/bash
VIVADO_PATH:=/opt/Xilinx/Vivado/2015.4/settings64.sh
SOURCE_FILE=monitor_server.c
	
all: clean monitor_server monitor_server_0.95

monitor_server:  # for version 0.92
	source $(VIVADO_PATH) && arm-xilinx-linux-gnueabi-gcc -o monitor_server $(SOURCE_FILE) -lpthread

monitor_server_0.95:
	source $(VIVADO_PATH) && arm-linux-gnueabihf-gcc -o monitor_server_0.95 $(SOURCE_FILE) -lpthread

# ZCU216 (Zynq UltraScale+, aarch64): the register map sits at 0x8000_0000.
# Static, so it runs on whatever glibc the board's Linux has.
AARCH64_GCC:=/opt/Xilinx/2025.2/gnu/aarch64/lin/aarch64-linux/bin/aarch64-linux-gnu-gcc
monitor_server_zcu216:
	$(AARCH64_GCC) -O2 -Wall -static -DREG_BASE=0x80000000UL -o monitor_server_zcu216 $(SOURCE_FILE) -lpthread

clean: 
	rm -f monitor_server_*
