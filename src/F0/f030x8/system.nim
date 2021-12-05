import stm32f030x8
export stm32f030x8

# ----------------------
# -- For System constant
# ----------------------
const
    SYSTEM_CLOCK*   = 48_000_000'u32
    UART_BAUDRATE*  = 115200
    BAUD_GEN* = SYSTEM_CLOCK div UART_BAUDRATE
    TIM_PWM_BASE_CLOCK* = SYSTEM_CLOCK # = 48MHz

proc initSystem*() {.inline.} =
    discard

# Clock up to 48MHzith PLL
proc initSysclock*() {.inline.} =           #
    if RCC.CFGR.bitIsSet(1):                # (1) Test if PLL is used as System clock
        RCC.CFGR.bclr(1,0)                  # (2) Select HSI as system clock
        while not RCC.CFGR.bitIsClr(3,2):
            discard                         # (3) Wait for switched to HSI
    RCC.CR.bclr(RCC_CR_PLLON_Pos)           # (4) Disable the PLL
    while RCC.CR.bitIsSet(RCC_CR_PLLRDY_Pos):
        discard                             # (5) Wait until PLLRDY is cleared
    # set parameters
    RCC.CFGR.bclr(21,20,19,18)
    RCC.CFGR.bset(21,19)                    # (6) Set the PLL multiplier to 12
    RCC.CR.bset(RCC_CR_PLLON_Pos)           # (7) Enable PLL
    while RCC.CR.bitIsClr(RCC_CR_PLLRDY_Pos):
        discard                             # (8) Wait until PLLRDY is set
    RCC.CFGR.bset(1)                        # (9) Select PLL as system clock
    while RCC.CFGR.bitIsClr(3):             # (10) Wait until the PLL is switched on
        discard

