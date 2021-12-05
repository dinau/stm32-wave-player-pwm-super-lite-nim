import conf_sys
import gpio

###########################
# Push button port
###########################
proc initUserButton*() {.inline.} =
    # PA0 Button SW . input PORT setup [[[ pull-down ]]] by hardware
    RCC.AHBENR.bset(RCC_AHBENR_GPIOAEN_Pos)  # GPIOA clock enable
    GPIOA.MODER.bclr(GPIO_MODER_MODER0_Pos,GPIO_MODER_MODER0_Pos+1)

proc btn_bit_now*():bool=
    const BTN_SW_PIN = 0 # PA0
    not ( ( GPIOA.IDR.ld and (1 shl BTN_SW_PIN) ) != 0 )

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
    RCC.AHBENR.bset(RCC_AHBENR_GPIOBEN_Pos)  # GPIOB clock enable
    GPIOB.MODER.bset(GPIO_MODER_MODER6_Pos)

###########################
# Init LED Indicator port
###########################
const LED_PIN = 10 # PB10
proc initLEDIndicator*() {.inline.} =
    # PB10 LED indicator PORT setup
    RCC.AHBENR.bset(RCC_AHBENR_GPIOBEN_Pos)  # GPIOB clock enable
    GPIOB.MODER.bset(GPIO_MODER_MODER10_Pos)
#
proc led*(state:int) {.used.}=
    GPIOB.atomic_out(LED_Pin,state)

proc ledToggle*() {.used.} =
    GPIOB.ODR.st (GPIOB.ODR.ld xor (1 shl LED_PIN))

###########################
# initOnBoardLED
###########################
# For test purpose,only init one of them.
# PC9,LD3,(green)
proc initOnBoardLED() {.inline,used.} =
    RCC.AHBENR.bset(RCC_AHBENR_GPIOCEN_Pos)
    GPIOC.MODER.bset(GPIO_MODER_MODER9_Pos)

proc toggleOnBoardLED(){.used.} =
    const LED_PIN = 9 # PC9
    GPIOC.ODR.st (GPIOC.ODR.ld xor (1 shl LED_PIN))

###########################
# Init SPI clock Hi/Low speed
###########################
proc sdLowSpeed*() {.inline.} =
    # set fHCLK / 128 = 48MHz / 128 = 375KHz
    SPI1.CR1.bset(SPI_CR1_BR_Pos + 1,SPI_CR1_BR_Pos + 2)
    SPI1.CR1.bclr(SPI_CR1_BR_Pos)

proc sdHiSpeed*() {.inline.} =
    # set fHCLK / 4 = 48MHz / 4 = 12MHz
    SPI1.CR1.bset(SPI_CR1_BR_Pos)
    SPI1.CR1.bclr( SPI_CR1_BR_Pos + 1,
                   SPI_CR1_BR_Pos + 2)

###########################
# Board init
###########################
proc initBoard*() {.inline.} =
    initUserButton()
    initCSPort()
    initLEDIndicator()

