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
proc initSysclock*() {.inline.} =            #
    FLASH.ACR.bset(FLASH_ACR_LATENCY_Pos)   # 1 wate state
    while FLASH.ACR.bitIsClr(FLASH_ACR_LATENCY_Pos):
        discard
    RCC.CR.bset(RCC_CR_HSION_Pos)           # HSI on
    while RCC.CR.bitIsClr(RCC_CR_HSIRDY_Pos):# Wait till HSI is ready
        discard
    # set parameters
    #RCC.CFGR.bclr(RCC_CFGR_PLLSRC_Pos)     # PLL source HSI
    #RCC.CFGR.bclr(21,20,19,18)             # First,all clear
    RCC.CFGR.bset(21,19)                    # (6) Set the PLL multiplier to 12
    RCC.CR.bset(RCC_CR_PLLON_Pos)           # (7) Enable PLL
    while RCC.CR.bitIsClr(RCC_CR_PLLRDY_Pos):
        discard                             # (8) Wait until PLLRDY is set
                                            # default: HPRE  div 1
                                            # default: PPRE  div 1
                                            # default: PPRE2 div 1
    RCC.CFGR.bset(1)                        # (9) Select PLL as system clock
    while RCC.CFGR.bitIsClr(3):             # (10) Wait until the PLL is switched on
        discard

