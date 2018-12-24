include ${TOPDIR}/.config
include ${TOPDIR}/tools/Config.mk
include ${TOPDIR}/arch/xtensa/src/lx6/Toolchain.defs

CC = $(CROSSDEV)gcc
CXX = $(CROSSDEV)g++
CPP = $(CROSSDEV)gcc -E
LD = $(CROSSDEV)ld
STRIP = $(CROSSDEV)strip --strip-unneeded
AR = $(ARCROSSDEV)ar rcs
NM = $(ARCROSSDEV)nm
OBJCOPY = $(CROSSDEV)objcopy
OBJDUMP = $(CROSSDEV)objdump

PWD=$(shell pwd)

CFLAGS += -I${TOPDIR}/include
CFLAGS += -I${PWD}/../include 
CFLAGS += -I${PWD}/../nvs_flash/include
CFLAGS += -I${PWD}/../log/include
CFLAGS += -I${PWD}/../wpa_supplicant/include 
CFLAGS += -I${PWD}/../wpa_supplicant/port/include 
CFLAGS += -I${PWD}/../tcpip_adapter/include
CFLAGS += -I${PWD}/../
CFLAGS += -I$(TOPDIR)/arch/xtensa/src/esp32
CFLAGS += -I$(TOPDIR)/arch/xtensa/src/esp32/chip
CFLAGS += -fno-builtin -ffunction-sections -fdata-sections -pipe -mlongcalls 


ESP32_CSRCS += clk.c dport_access.c fast_crypto_ops.c 
ESP32_CSRCS += hw_random.c intr_alloc.c lib_printf.c phy_init.c system_api.c wifi_init.c 

ESP32_COBJS = $(ESP32_CSRCS:.c=$(OBJEXT))
BIN = libesp32$(LIBEXT)

all: $(BIN)
$(ESP32_COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(ESP32_COBJS)
	$(call ARCHIVE, $@, $(ESP32_COBJS))

clean:
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(ESP32_COBJS))
	$(call CLEAN)

