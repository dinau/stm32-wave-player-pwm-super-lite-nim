import conf_sys
import gpio

###########################
# Push button port
###########################
proc initUserButton*() {.inline.} =
    # PA0 Button SW . input PORT setup [[[ pull-down ]]] by hardware
    RCC.AHB1ENR.bset(RCC_AHB1ENR_GPIOAEN_Pos)  # GPIOA clock enable
    GPIOA.MODER.bclr(GPIO_MODER_MODER0_Pos,GPIO_MODER_MODER0_Pos+1)

proc btn_bit_now*():bool=
    const BTN_SW_PIN = 0 # PA0
    not ( ( GPIOA.IDR.ld and (1 shl BTN_SW_PIN) ) != 0 )

###########################
# Init SPI Chip select port
###########################
# PB8(CS) chip select port
const CS_Pin = 8 # PB8
proc cs_on*() {.inline.} =
    GPIOB.atomic_set(CS_Pin)

proc cs_off*() {.inline.} =
    GPIOB.atomic_clr(CS_Pin)

proc initCSPort*() {.inline.} =
    # PB8(CS) chip select port
    RCC.AHB1ENR.bset(RCC_AHB1ENR_GPIOBEN_Pos)  # GPIOB clock enable
    GPIOB.MODER.bset(GPIO_MODER_MODER8_Pos)

###########################
# Init LED Indicator port
###########################
const LED_PIN = 12 # PD12 (Green)
proc initLEDIndicator*() {.inline.} =
    # PD12 LED indicator PORT setup
    RCC.AHB1ENR.bset(RCC_AHB1ENR_GPIODEN_Pos)  # GPIOD clock enable
    GPIOD.MODER.bset(GPIO_MODER_MODER12_Pos)
#
proc led*(state:int) {.used.}=
    GPIOD.atomic_out(LED_Pin,state)

proc ledToggle*() {.used.} =
    GPIOD.ODR.st (GPIOD.ODR.ld xor (1 shl LED_PIN))

###########################
# Init SPI clock Hi/Low speed
###########################
proc sdLowSpeed*() {.inline.} =
    # set fHCLK / 256 = 84MHz / 256 = 328KHz
    SPI1.CR1.bset(SPI_CR1_BR_Pos,
                  SPI_CR1_BR_Pos + 2,
                  SPI_CR1_BR_Pos + 1)

proc sdHiSpeed*() {.inline.}=
    # set fHCLK2 / 8 = 84MHz / 8 = 10.5MHz
    SPI1.CR1.bset(SPI_CR1_BR_Pos + 1)
    SPI1.CR1.bclr(SPI_CR1_BR_Pos,
                  SPI_CR1_BR_Pos + 2)

###########################
# Board init
###########################
proc initBoard*() {.inline.} =
    initUserButton()
    initCSPort()
    initLEDIndicator()

