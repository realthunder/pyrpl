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
# Static against musl, built with zig (python3 -m venv ~/.venvs/zig &&
# ~/.venvs/zig/bin/pip install ziglang), so it runs on any aarch64 kernel. A
# static glibc build does not: the Vitis 2025.2 one needs kernel 5.15+, and an
# Ubuntu 24.04 one starts but its threads crash in __ctype_init on the 5.4
# kernel of the PetaLinux 2020.1 SD card.
ZIG_CC:=$(HOME)/.venvs/zig/bin/python -m ziglang cc
monitor_server_zcu216:
	$(ZIG_CC) -target aarch64-linux-musl -static -O2 -Wall -Wno-misleading-indentation -DREG_BASE=0x80000000UL -DDMA_REG_BASE=0x80200180UL -o monitor_server_zcu216 $(SOURCE_FILE) -lpthread

clean: 
	rm -f monitor_server_*
