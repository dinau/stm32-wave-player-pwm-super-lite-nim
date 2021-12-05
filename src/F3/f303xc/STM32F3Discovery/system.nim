import stm32f303xc
export stm32f303xc

# ----------------------
# -- For System constant
# ----------------------
const
    SYSTEM_CLOCK*   = 64_000_000'u32
    UART_BAUDRATE*  = 115200
    BAUD_GEN* = SYSTEM_CLOCK div UART_BAUDRATE
    TIM_PWM_BASE_CLOCK* = SYSTEM_CLOCK # = 64MHz

proc initSystem*() {.inline.} =
    # enable FPU for Cortex-M4
    SCB.CPACR.st (SCB.CPACR.ld or ((3 shl 10*2) or (3 shl 11*2 ))) # /* set CP10 and CP11 Full Access */

# Clock up to 64MHz PLL
proc initSysclock*() {.inline.} =           #
    FLASH.ACR.bset(FLASH_ACR_LATENCY_Pos + 1)   # 2 wate state
    while FLASH.ACR.bitIsClr(FLASH_ACR_LATENCY_Pos + 1):
        discard
    RCC.CR.bset(RCC_CR_HSION_Pos)           # HSI on
    while RCC.CR.bitIsClr(RCC_CR_HSIRDY_Pos):# Wait till HSI is ready
        discard
    # set parameters
    #RCC.CFGR.bclr(RCC_CFGR_PLLSRC_Pos)     # PLL source HSI
    RCC.CFGR.bclr(18)                       # clear
    RCC.CFGR.bset(21,20,19)                 # (6) Set the PLL multiplier to 16
    RCC.CR.bset(RCC_CR_PLLON_Pos)           # (7) Enable PLL
    while RCC.CR.bitIsClr(RCC_CR_PLLRDY_Pos):
        discard                             # (8) Wait until PLLRDY is set
                                            # default: HPRE  div 1
                                            # default: PPRE  div 1
                                            # default: PPRE2 div 1
    RCC.CFGR.bclr(0)                      # (9) Select PLL as system clock
    RCC.CFGR.bset(1)                      #
    while RCC.CFGR.bitIsClr(3):           # (10) Wait until the PLL is switched on
        discard

