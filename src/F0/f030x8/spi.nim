import conf_sys
import board

#[  SPI1
    SPI_MOSI    = PA_7,
    SPI_MISO    = PA_6,
    SPI_SCK     = PA_5,
    SPI_CS      = PB_6, ]#

# forward definition
proc send*(spi:auto,data:uint8): uint8    {.used,inline.}
proc send16*(spi:auto,data:uint16):uint16 {.used,inline.}

#########################
# for sd card interface
#########################
proc send_ff*():auto {.inline.} =
    SPI1.send(0xff.uint8)

proc send16_ff*():auto {.inline.} = # 16bit send/receive
    SPI1.send16(0xffff)

proc spi_write*(d:byte) {.inline.} =
    discard SPI1.send(d)

proc spi_read*():byte {.inline.} =
    SPI1.send(0xFF.uint8)

proc sdSpiDev*():auto =
    return SPI1

proc sd_chip_select*(state:int) {.inline.} =
    if state==1: cs_on() else: cs_off()
proc sd_cs_enable*()  = sd_chip_select(0)
proc sd_cs_disable*() = sd_chip_select(1)

template spiEnable*() =
    SPI1.CR1.bset(SPI_CR1_SPE_Pos)

proc sdSpiEnable*() {.inline.} =
    spiEnable()

####################
# SPI send
####################
# 8bit send / receive
proc send*(spi:auto,data:uint8): uint8 {.used,inline.} =
    while spi.SR.bitIsClr(SPI_SR_TXE_Pos):
        discard
    spi.DR.st data   # 8bit transfer
    while spi.SR.bitIsClr(SPI_SR_RXNE_Pos):
        discard
    return spi.DR.ld8   # 8bit receive

# 16bit send / receive (no check, so far)
proc send16*(spi:auto,data:uint16): uint16 {.used,inline.} =
    # It must be set and confirm,
    # CR2_DS = 0x07 # 16bit data width
    # CR2_FRXT = 0  # RXNE event
    #
    # set 16bit data size
    spi.CR2.bset( SPI_CR2_DS_Pos+3)
    spi.CR2.bclr( SPI_CR2_FRXTH_Pos)

    while spi.SR.bitIsClr(SPI_SR_TXE_Pos):
        discard
    spi.DR.st data  # 16bit transfer
    while spi.SR.bitIsClr(SPI_SR_RXNE_Pos):
        discard
    result = spi.DR.ld16         # 16bit receive
    # set 8bit data size
    spi.CR2.bclr( SPI_CR2_DS_Pos+3)
    spi.CR2.bset( SPI_CR2_FRXTH_Pos)

###########################
# initSPI
###########################
proc initSPI*(spi:auto) {.inline.} =
    if spi==SPI1:
        # spi port setting
        RCC.APB2ENR.bset(RCC_APB2ENR_SPI1EN_Pos)          # SPI1  clock enable
        RCC.AHBENR.bset(RCC_AHBENR_GPIOAEN_Pos)           # GPIOA clock enable
        GPIOA.MODER.bset(   GPIO_MODER_MODER5_Pos+1,      # PA5(SCK)
                            GPIO_MODER_MODER6_Pos+1,      # PA6(MISO)
                            GPIO_MODER_MODER7_Pos+1)      # PA7(MOSI)
        GPIOA.OSPEEDR.bset( GPIO_OSPEEDR_OSPEEDR5_Pos,
                            GPIO_OSPEEDR_OSPEEDR5_Pos+1,
                            GPIO_OSPEEDR_OSPEEDR6_Pos,
                            GPIO_OSPEEDR_OSPEEDR6_Pos+1,
                            GPIO_OSPEEDR_OSPEEDR7_Pos,
                            GPIO_OSPEEDR_OSPEEDR7_Pos+1)
        GPIOA.PUPDR.bset(   GPIO_PUPDR_PUPDR5_Pos+1,      # PA5(SCK)  Pull.down
                            GPIO_PUPDR_PUPDR6_Pos)        # PA6(MISO) Pull.up
                                                          # PA7(MOSI) P.P default
    when false:
        if spi==SPI2:
            # If you'd like to use SPI2, you must recheck its settings.
            # spi port setting
            RCC.APB1ENR.bset(RCC_APB1ENR_SPI2EN_Pos)      # SPI2  clock enable
            RCC.AHBENR.bset(RCC_AHBENR_GPIOBEN_Pos)       # GPIOB clock enable
            GPIOB.MODER.bset(   GPIO_MODER_MODER13_Pos+1, # PB13(SCK)
                                GPIO_MODER_MODER14_Pos+1, # PB14(MISO)
                                GPIO_MODER_MODER15_Pos+1) # PB15(MOSI)
            GPIOB.OSPEEDR.bset( GPIO_OSPEEDR_OSPEEDR13_Pos,
                                GPIO_OSPEEDR_OSPEEDR13_Pos+1,
                                GPIO_OSPEEDR_OSPEEDR14_Pos,
                                GPIO_OSPEEDR_OSPEEDR14_Pos+1,
                                GPIO_OSPEEDR_OSPEEDR15_Pos,
                                GPIO_OSPEEDR_OSPEEDR15_Pos+1)
            GPIOB.PUPDR.bset(   GPIO_PUPDR_PUPDR13_Pos+1,  # PA5(SCK)  Pull.down
                                GPIO_PUPDR_PUPDR14_Pos)    # PA6(MISO) Pull.up
                                                           # PA7(MOSI) P.P default
    #######################
    # SPI register settings
    #######################
    spi.CR1.bset(SPI_CR1_MSTR_Pos)
    spi.CR1.bset(SPI_CR1_BR_Pos) # sck=12MHz

    # NSS_SOFT [Important setting] or set SSOE=1
    spi.CR1.bset(SPI_CR1_SSM_Pos, SPI_CR1_SSI_Pos)

    # [Important setting if used 8bit transfer]
    spi.CR2.bset(SPI_CR2_FRXTH_Pos)

    # Enable SPI output
    spi.CR1.bset(SPI_CR1_SPE_Pos)

