############################################################################
#
# Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
# Cypress Semiconductor Corporation. All Rights Reserved.
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific
# language governing permissions and limitations under the License.
#
############################################################################
-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs

#DEPPATH	= --dep-path .
VPATH = .
CXXSRCS =

SUBDIRS = spi_flash soc esp32 

CSRCS += esp32_wifi_os_adapter.c event_groups.c esp32_queue_api.c mq_tryreceive.c 

# NVS module
CXXSRCS += nvs_flash/src/nvs_item_hash_list.cpp 
CXXSRCS += nvs_flash/src/nvs_page.cpp
CXXSRCS += nvs_flash/src/nvs_pagemanager.cpp
CXXSRCS += nvs_flash/src/nvs_storage.cpp
CXXSRCS += nvs_flash/src/nvs_types.cpp
CXXSRCS += nvs_flash/src/nvs_api.cpp 

CXXFLAGS += -I$(TOPDIR)/arch/xtensa/src/esp32
CXXFLAGS += -I./include
CXXFLAGS += -I./spi_flash/include
CXXFLAGS += -I./log/include
CXXFLAGS += -I./nvs_flash/include -DESP_PLATFORM
CXXFLAGS += -std=c++11 -Wall -Werror

CFLAGS += -I$(TOPDIR)/arch/xtensa/src/esp32
CFLAGS += -I$(TOPDIR)/arch/xtensa/src/esp32/chip
CFLAGS += -I./nvs_flash/include
CFLAGS += -Iinclude -I./spi_flash/include
CFLAGS += -I$(TOPDIR)/kernel
CFLAGS += -I$(TOPDIR)/include/net
CFLAGS += -I./lwip_port/include

#ESP log module
CSRCS += log/log.c
CFLAGS += -I./log/include

#ESP32 external SPIRAM
ifeq ($(CONFIG_SPIRAM_SUPPORT), y)
CFLAGS += -I./heap/include
CSRCS += spi_ram/spiram.c  spi_ram/spiram_psram.c gpio_periph.c
endif

#ESP32 heap allocator 
ifeq ($(CONFIG_SPIRAM_USE_CAPS_ALLOC), y)
CSRCS += heap/heap_caps.c  heap/heap_caps_init.c  heap/multi_heap.c 
endif

#ESP32 event loop module
CSRCS +=  event_loop/event_loop.c event_loop/event_default_handlers.c
CFLAGS += -I./tcpip_adapter/include

#ESP32 tcpip adapter
CSRCS += tcpip_adapter/tcpip_adapter_lwip.c lwip_port/wlanif.c  lwip_port/dhcpserver.c lwip_port/espdns.c

#ESP32 wpa_supplicant crypto
CSRCS += $(wildcard ./wpa_supplicant/port/*.c)
CSRCS += $(wildcard ./wpa_supplicant/src/crypto/*.c)
CSRCS += $(wildcard ./wpa_supplicant/src/fast_crypto/*.c)
CSRCS += $(wildcard ./wpa_supplicant/src/wps/*.c)
CSRCS += $(wildcard ./wpa_supplicant/src/wpa2/tls/*.c)
CSRCS += $(wildcard ./wpa_supplicant/src/wpa2/utils/*.c)
CSRCS += $(wildcard ./wpa_supplicant/src/wpa2/eap_peer/*.c)
CFLAGS += -I./wpa_supplicant/include
CFLAGS += -I./wpa_supplicant/port/include
CFLAGS += \
		-D__ets__ \
		-DEMBEDDED_SUPP\
		-DIEEE8021X_EAPOL\
		-DEAP_PEER_METHOD\
		-DEAP_MSCHAPv2\
		-DEAP_TTLS\
		-DEAP_TLS\
		-DEAP_PEAP\
		-DUSE_WPA2_TASK\
		-DCONFIG_WPS2\
		-DCONFIG_WPS_PIN\
		-DUSE_WPS_TASK\
		-DESPRESSIF_USE\
		-DESP32_WORKAROUND \
		-DCONFIG_ECC 

#ESP32 mbedtls
CSRCS += $(wildcard ./mbedtls/mbedtls/library/*.c)
CSRCS += $(wildcard ./mbedtls/port/*.c)
CFLAGS += -I./mbedtls/mbedtls/include
CFLAGS += -I./mbedtls/port/include
CFLAGS += -DMBEDTLS_CONFIG_FILE='"mbedtls/port/include/mbedtls/esp_config.h"'


#ESP32 hardware crypto
CSRCS += $(wildcard ./hwcrypto/*.c)

CXXOBJS = $(CXXSRCS:.cpp=$(OBJEXT))
COBJS   = $(CSRCS:.c=$(OBJEXT))

BIN       = ../libexternal$(LIBEXT)
OBJS        = $(CXXOBJS) $(COBJS)

# Common build
all: $(SUBDIRS) .built
.PHONY: clean distclean

ECHO:
	@echo $(SUBDIRS)

$(SUBDIRS): ECHO
	make -C $@

.built: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))

$(CXXOBJS) : %$(OBJEXT): %.cpp
	$(call COMPILEXX, $<, $@)

$(COBJS): %$(OBJEXT): %.c 
    $(call COMPILE, $<, $@)

clean:
	$(call DELFILE, .built)
	$(call DELFILE, esp32/*.o)
	$(call CLEAN)
	rm -f $(OBJS)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
