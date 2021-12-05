import stm32l152xe
export stm32l152xe

# ----------------------
# -- For System constant
# ----------------------
const
    SYSTEM_CLOCK*   = 32_000_000'u32
    UART_BAUDRATE*  = 115200
    BAUD_GEN* = SYSTEM_CLOCK div UART_BAUDRATE
    TIM_PWM_BASE_CLOCK* = SYSTEM_CLOCK # = 32MHz

proc initSystem*() {.inline.} =
    discard

# set PLL 32MHz for STM32L152RE
proc initSysclock*() {.inline.} =           #
    FLASH.ACR.bset(FLASH_ACR_ACC64_Pos)     # set ACC64
    while FLASH.ACR.bitIsClr(FLASH_ACR_ACC64_Pos):
        discard
    FLASH.ACR.bset(FLASH_ACR_LATENCY_Pos)   # 1 wate state
    while FLASH.ACR.bitIsClr(FLASH_ACR_LATENCY_Pos):
        discard
    FLASH.ACR.bset(FLASH_ACR_PRFTEN_Pos)   # 64bit prefetch enable

    RCC.APB1ENR.bset(RCC_APB1ENR_PWREN_Pos) # RCC PWR_CR power on
    PWR.CR.bset(PWR_CR_VOS_Pos)             # set Voltage scale 1
    PWR.CR.bclr(PWR_CR_VOS_Pos + 1)

    RCC.CR.bset(RCC_CR_HSION_Pos)           # HSI on
    while RCC.CR.bitIsClr(RCC_CR_HSIRDY_Pos):# Wait till HSI is ready
        discard
    RCC.ICSCR.bset(RCC_ICSCR_HSITRIM_Pos + 4)# Set HSI Trim 16
    # set parameters
    #RCC.CFGR.bclr(RCC_CFGR_PLLSRC_Pos)     # PLL source HSI (default)
    #RCC.CFGR.bclr(21,20,19,18)             # PLLMUL(21:18): First,all clear (default)
    RCC.CFGR.bset(19)                       # (6) Set the PLL multiplier to 6
    RCC.CFGR.bset(23)                       # PLLDIV(23:22): Set the PLL DIV to 3
    RCC.CR.bset(RCC_CR_PLLON_Pos)           # (7) Enable PLL
    while RCC.CR.bitIsClr(RCC_CR_PLLRDY_Pos):
        discard                             # (8) Wait until PLLRDY is set
                                            # default: HPRE  div 1
                                            # default: PPRE  div 1
                                            # default: PPRE2 div 1
    RCC.CFGR.bset(1,0)                      # (9) Select PLL as system clock
    while RCC.CFGR.bitIsClr(3,2):           # (10) Wait until the PLL is switched on
        discard

