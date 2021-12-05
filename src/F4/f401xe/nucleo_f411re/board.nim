import conf_sys
import gpio

###########################
# Push button port
###########################
proc initUserButton*() {.inline.} =
    # PC13 Button SW . input PORT setup [[[ pull-up ]]] by hardware
    RCC.AHB1ENR.bset(RCC_AHB1ENR_GPIOCEN_Pos)  # GPIOC clock enable
    GPIOC.MODER.bclr(GPIO_MODER_MODER13_Pos,GPIO_MODER_MODER13_Pos+1)

proc btn_bit_now*():bool=
    const BTN_SW_PIN = 13 # PC13
    ( GPIOC.IDR.ld and (1 shl BTN_SW_PIN) ) != 0

###########################
# Init SPI Chip select port
###########################
# PB6(CS) chip select port
const CS_Pin = 6 # PB6
proc cs_on*() {.inline.} =
    GPIOB.atomic_set(CS_Pin)

proc cs_off*() {.inline.} =
    GPIOB.atomic_clr(CS_Pin)

proc initCSPort*() {.inline.} =
    # PB6(CS) chip select port
    RCC.AHB1ENR.bset(RCC_AHB1ENR_GPIOBEN_Pos)  # GPIOB clock enable
    GPIOB.MODER.bset(GPIO_MODER_MODER6_Pos) # set output port 
    GPIOB.PUPDR.bset(GPIO_PUPDR_PUPD6_Pos) # set pllup 

###########################
# Init LED Indicator port
###########################
const LED_PIN = 10 # PB10
proc initLEDIndicator*() {.inline.} =
    # PB10 LED indicator PORT setup
    RCC.AHB1ENR.bset(RCC_AHB1ENR_GPIOBEN_Pos)  # GPIOB clock enable
    GPIOB.MODER.bset(GPIO_MODER_MODER10_Pos)
#
proc led*(state:int) {.used.}=
    GPIOB.atomic_out(LED_Pin,state)

proc ledToggle*() {.used.} =
    GPIOB.ODR.st (GPIOB.ODR.ld xor (1 shl LED_PIN))

###########################
# Init SPI clock Hi/Low speed
###########################
proc sdLowSpeed*() {.inline.} =
    # fHCLK = 96MHz for f411re
    # set fHCLK / 256 = 96MHz / 256 = 375KHz
    SPI1.CR1.bset(SPI_CR1_BR_Pos)
    SPI1.CR1.bset(SPI_CR1_BR_Pos+1,
                  SPI_CR1_BR_Pos+2)

proc sdHiSpeed*() {.inline.} =
    # for f411re
    # set fHCLK / 8 = 96MHz / 8 = 12MHz
    SPI1.CR1.bclr(SPI_CR1_BR_Pos)
    SPI1.CR1.bset(SPI_CR1_BR_Pos+1)
    SPI1.CR1.bclr(SPI_CR1_BR_Pos+2)

###########################
# Board init
###########################
proc initBoard*() {.inline.} =
    initUserButton()
    initCSPort()
    initLEDIndicator()

