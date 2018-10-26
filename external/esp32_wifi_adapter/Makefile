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


# Include wireless drivers
#CXXSRCS += nvs_flash/src/nvs_item_hash_list.cpp 
#CXXSRCS += nvs_flash/src/nvs_page.cpp
#CXXSRCS += nvs_flash/src/nvs_pagemanager.cpp
#CXXSRCS += nvs_flash/src/nvs_storage.cpp
#CXXSRCS += nvs_flash/src/nvs_types.cpp
#CXXSRCS += nvs_flash/src/nvs_api.cpp 

CXXFLAGS += -I./include
CXXFLAGS += -I./nvs_flash/include
CXXFLAGS += -I./spi_flash/include
CXXFLAGS +=  -std=c++11 -Wall -Werror


CSRCS = phy_init.c hw_random.c  system_api.c  esp32_wifi_os_adapter.c  system_api.c event_groups.c esp_ota_eps.c intr_alloc.c clk.c
#CSRCS +=  $(wildcard ./spi_flash/*.c)

CFLAGS += -I$(TOPDIR)/arch/xtensa/src/esp32
CFLAGS += -I./spi_flash/include
CFLAGS += -I./nvs_flash/include
CFLAGS += -Iinclude
CFLAGS += -I$(TOPDIR)/kernel

CXXOBJS = $(CXXSRCS:.cpp=$(OBJEXT))
COBJS   = $(CSRCS:.c=$(OBJEXT))

BIN       = ../libexternal$(LIBEXT)
OBJS        = $(CXXOBJS) $(COBJS)

# Common build
all: .built
.PHONY: clean distclean

.built: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))

$(CXXOBJS) : %$(OBJEXT): %.cpp
	$(call COMPILEXX, $<, $@)

$(COBJS): %$(OBJEXT): %.c 
    $(call COMPILE, $<, $@)

clean:
	$(call DELFILE, .built)
	$(call CLEAN)
	rm -f $(OBJS)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
