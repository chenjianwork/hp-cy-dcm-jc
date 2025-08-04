####################################################################################################
# CFG
####################################################################################################
-include .config

HOSTOS := $(shell uname -s | tr '[:lower:]' '[:upper:]')

####################################################################################################
GIT_DIR = $(if $(wildcard $(CURDIR)/.git),y,)
ifneq ($(GIT_DIR),y)
KBUILD_VERSION_STRING = "0.0.2"
else
KBUILD_VERSION_STRING = "$(shell git describe --tags --always)"
endif

####################################################################################################
# APP
####################################################################################################
PROJECT := HP-CY-DCM
INCLUDE := .
SOURCES :=
DEFINES := -DCONFIG_VERSION=\"$(KBUILD_VERSION_STRING)\"
TEXT_OF := 0x40000
#TEXT_OF := 0x00000
LD_PATH :=
LD_LIBS :=
BINFILE := $(PROJECT)-$(KBUILD_VERSION_STRING:"%"=%)
HEXFILE := $(PROJECT)-$(KBUILD_VERSION_STRING:"%"=%)

####################################################################################################
# CONFIG
####################################################################################################
CPU_FLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard

####################################################################################################
# COMPILER
####################################################################################################
AR		 = arm-none-eabi-ar
AS		 = arm-none-eabi-as
CC		 = arm-none-eabi-gcc
CPP		 = $(CC) -E
CX		 = arm-none-eabi-g++
LD		 = arm-none-eabi-ld
OBJCOPY	 = arm-none-eabi-objcopy
SIZE	 = arm-none-eabi-size

####################################################################################################
# FILE SOURCES
####################################################################################################
include sources.mk

####################################################################################################
# FILE SOURCES
####################################################################################################
override OBJECTS := $(SOURCES)
override OBJECTS := $(OBJECTS:%.S=%.o)
override OBJECTS := $(OBJECTS:%.c=%.o)
override OBJECTS := $(OBJECTS:%.cxx=%.o)
override DEPENDS := $(SOURCES)
override DEPENDS := $(DEPENDS:%.S=%.d)
override DEPENDS := $(DEPENDS:%.c=%.d)
override DEPENDS := $(DEPENDS:%.cxx=%.d)

-include $(DEPENDS)

####################################################################################################
# COMPILER OPTIONS
####################################################################################################
AS_FLAGS := $(CPU_FLAGS)
AS_FLAGS += -std=gnu99
AS_FLAGS += -Wall
AS_FLAGS += -g2
AS_FLAGS += -gdwarf-2
AS_FLAGS += -gstrict-dwarf
AS_FLAGS += -x
AS_FLAGS += assembler-with-cpp
AS_FLAGS += -Xassembler
AS_FLAGS += --no-warn 
AS_FLAGS += -MMD
AS_FLAGS += -MP

####################################################################################################
CC_FLAGS := $(CPU_FLAGS)
CC_FLAGS += -std=gnu99
CC_FLAGS += -ffunction-sections
CC_FLAGS += -fdata-sections
CC_FLAGS += -Wall
CC_FLAGS += -g2
CC_FLAGS += -gdwarf-2
CC_FLAGS += -gstrict-dwarf
CC_FLAGS += -fstrict-aliasing
CC_FLAGS += -Wunused-local-typedefs
CC_FLAGS += -Wmissing-braces
CC_FLAGS += -Wno-switch
CC_FLAGS += -Wunused-value
CC_FLAGS += -Wunused-variable
CC_FLAGS += -Wunused-but-set-variable
CC_FLAGS += -Wno-pointer-to-int-cast
CC_FLAGS += -Wunused-function
CC_FLAGS += -Wunused-label
CC_FLAGS += -Wno-char-subscripts
CC_FLAGS += -Wno-int-to-pointer-cast
CC_FLAGS += -O0
CC_FLAGS += -MMD
CC_FLAGS += -MP

####################################################################################################
CX_FLAGS := $(CPU_FLAGS)
CX_FLAGS += -fno-exceptions
CX_FLAGS += -ffunction-sections
CX_FLAGS += -fdata-sections
CX_FLAGS += -Wall
CX_FLAGS += -g2
CX_FLAGS += -gdwarf-2
CX_FLAGS += -gstrict-dwarf
CX_FLAGS += -fstrict-aliasing
CX_FLAGS += -Wno-missing-braces
CX_FLAGS += -Wno-switch
CX_FLAGS += -Wunused-value
CX_FLAGS += -Wunused-variable
CX_FLAGS += -Wunused-but-set-variable
CX_FLAGS += -Wunused-function
CX_FLAGS += -Wunused-label
CX_FLAGS += -Wno-char-subscripts
CX_FLAGS += -Wno-int-to-pointer-cast
CX_FLAGS += -O0
CX_FLAGS += -MMD
CX_FLAGS += -MP

####################################################################################################
LD_FLAGS := $(CPU_FLAGS)
LD_FLAGS += -std=gnu99
LD_FLAGS += -Xlinker --gc-sections
LD_FLAGS += -Xlinker -cref
LD_FLAGS += -Xlinker -static
LD_FLAGS += -g2
LD_FLAGS += -gdwarf-2
LD_FLAGS += -gstrict-dwarf
LD_FLAGS += -Xlinker -z
LD_FLAGS += -Xlinker muldefs
LD_FLAGS += -specs=nosys.specs

####################################################################################################
# BUILD
####################################################################################################
all: $(BINFILE).bin $(PROJECT).siz

# Each subdirectory must supply rules for building sources it contributes
$(HEXFILE).hex: $(PROJECT).elf
	@echo 'OBJCOPY $@'
	@$(OBJCOPY) -O ihex "$(PROJECT).elf" "$(HEXFILE).hex"

$(BINFILE).bin: $(PROJECT).elf
	@echo 'OBJCOPY $@'
	@$(OBJCOPY) -O binary "$(PROJECT).elf" "$(BINFILE).bin"

$(PROJECT).siz: $(PROJECT).elf
	@echo 'INFO    $@'
	@echo 'VERSION $(KBUILD_VERSION_STRING)'
	@$(SIZE) --format=berkeley "$(PROJECT).elf"

$(PROJECT).elf: prepare flash.lds $(OBJECTS) FORCE
	@echo 'LD      $@'
	@$(CC) $(CC_FLAGS) $(LD_FLAGS) -Wl,-Map,"$(PROJECT).map" -T flash.lds -o $@ $(OBJECTS) -L$(LD_PATH) -l$(LD_LIBS)

PHONY += FORCE
FORCE:

# Generate linker script
LDPPFLAGS = -DTEXT_OFFSET=$(TEXT_OF)
PHONY += flash.lds
flash.lds: $(LDSCRIPT)
	@$(CPP) $(LDPPFLAGS) -D__ASSEMBLY__ -x assembler-with-cpp -std=c99 -P -o $@ $<

PHONY += prepare
prepare:
	@find $(CURDIR) \
		\( -name '*.elf' \
		   -o -name '*.hex' \
		   -o -name '*.bin' \
		   -o -name '*.map' \) -type f -print | xargs rm -f

####################################################################################################
PHONY += clean
clean:
	@rm -rf $(CURDIR)/flash.lds
	@find $(CURDIR) \
		\( -name '*.o' \
		   -o -name '*.d' \
		   -o -name '*.elf' \
		   -o -name '*.hex' \
		   -o -name '*.bin' \
		   -o -name '*.map' \) -type f -print | xargs rm -f

####################################################################################################
%.o:%.S
	@echo 'AS      $(notdir $<)'
	@$(CC) $(AS_FLAGS) $(DEFINES) $(addprefix -I,$(INCLUDE)) -c $< -o $@

%.o:%.c
	@echo 'CC      $(notdir $<)'
	@$(CC) $(CC_FLAGS) $(DEFINES) $(addprefix -I,$(INCLUDE)) -c $< -o $@

%.o:%.cxx
	@echo 'CX      $(notdir $<)'
	@$(CX) $(CX_FLAGS) $(DEFINES) $(addprefix -I,$(INCLUDE)) -c $< -o $@

.PHONY: $(PHONY)
