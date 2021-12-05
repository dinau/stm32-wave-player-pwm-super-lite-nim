# This file is release support file.
# All projects are build at a time.

SAMPLE_DIRS :=	src/F4/f401xe/nucleo_f401re\
				src/F4/f401xe/nucleo_f411re\
				src/F4/f401xe/STM32F4Discovery\
				src/F3/f303xc/STM32F3Discovery\
				src/F0/f030x8/STM32F0Discovery\
				src/F0/f030x8/nucleo_f030r8\
				src/L1/l152xe/nucleo_l152re
all:
	-@mkdir -p doc/hex
	$(foreach exdir,$(SAMPLE_DIRS), $(call def_make,$(exdir) ) )

test:
	@echo $(notdir $(SAMPLE_DIRS))


clean:
	$(foreach exdir,$(SAMPLE_DIRS), $(call def_clean,$(exdir) ))

# definition loop funciton
DIR_NAME = $(strip $(1))
FNAME   = $(notdir $(DIR_NAME))
define def_make
	@$(MAKE) -C $(1) clean all
	cp $(DIR_NAME)/BINHEX/$(FNAME).hex doc/hex/$(FNAME).hex
	cp $(DIR_NAME)/BINHEX/$(FNAME).bin doc/hex/$(FNAME).bin

endef

define def_clean
	@$(MAKE) -C $(1) clean

endef

