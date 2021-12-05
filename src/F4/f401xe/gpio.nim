import conf_sys

###########################
# initGPIO
###########################
proc initGPIO*() {.inline.}=
    # PA8 TEST PORT setup
    RCC.AHB1ENR.bset(RCC_AHB1ENR_GPIOAEN_Pos)  # GPIOA clock enable
    GPIOA.MODER.bset(GPIO_MODER_MODER8_Pos)

###########################
# port on/off
###########################
proc atomic_set*(gpio:auto,pin:auto) {.used,inline.} =
    gpio.BSRR.st (1 shl pin).uint32

proc atomic_clr*(gpio:auto,pin:auto) {.used,inline.} =
    gpio.BSRR.st (1 shl (pin + 16)).uint32

proc atomic_out*(gpio:auto,mask,state:int) {.used,inline.} =
    if state==0:
        gpio.BSRR.st (1 shl (mask+16)).uint32
    else:
        gpio.BSRR.st (1 shl mask).uint32

###########################
# test port
###########################
proc test_port*(state:int) {.used,inline.} =
    const TEST_PIN = 8 # PA8
    GPIOA.atomic_out(TEST_Pin,state)

template test_on*() =
    test_port(1)

template test_off*() =
    test_port(0)



