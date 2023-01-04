
TARGET =  $(notdir $(CURDIR))
all:
	@nim make
	@arm-none-eabi-objdump -hSC BINHEX/$(TARGET).elf > BINHEX/$(TARGET).lst
	@arm-none-eabi-objdump -hdC BINHEX/$(TARGET).elf > BINHEX/$(TARGET).lst2
clean:
	@nim clean
w:
	@nim w
