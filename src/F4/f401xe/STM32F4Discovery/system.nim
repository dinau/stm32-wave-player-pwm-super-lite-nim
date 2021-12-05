import stm32f407xx
export stm32f407xx

# ----------------------
# -- For System constant
# ----------------------
const
    SYSTEM_CLOCK*   = 168_000_000'u32
    UART_BAUDRATE*  = 115200
    # USART2 base clock is SYSTEM_CLOCK / 4 = 42MHz
    BAUD_GEN* = (SYSTEM_CLOCK shr 2) div UART_BAUDRATE
    TIM_PWM_BASE_CLOCK* = SYSTEM_CLOCK div 2  # = 84MHz

proc initSystem*() {.inline.} =
    # enable FPU for Cortex-M4
    SCB.CPACR.st (SCB.CPACR.ld or ((3 shl 10*2) or (3 shl 11*2 ))) # /* set CP10 and CP11 Full Access */

# Clock up to 168MHz PLL
proc initSysclock*() {.inline.} =            #
    FLASH.ACR.bset(FLASH_ACR_LATENCY_Pos,FLASH_ACR_LATENCY_Pos + 2)   # 5 wate state
    while FLASH.ACR.bitIsClr(FLASH_ACR_LATENCY_Pos,FLASH_ACR_LATENCY_Pos + 2):
        discard # Fatal Error
    # PWR.CR: scale 1 == reset value
    RCC.CR.bset(RCC_CR_HSION_Pos)           # HSI on
    while RCC.CR.bitIsClr(RCC_CR_HSIRDY_Pos):# Wait till HSI is ready
        discard
    # set parameters SYSTEM_CLOCK = 168MHz
    # PLLM=1/16,PLLN=384,PLLP=1/2
    let PLLN = (336 shl RCC_PLLCFGR_PLLN_Pos)
    let PLLM = 16
    let PLLP = 0 # 1/2 : == reset value
    RCC.PLLCFGR.st ( PLLN or PLLM or PLLP ) # (6) Set the PLL parameters

    RCC.CR.bset(RCC_CR_PLLON_Pos)           # (7) Enable PLL
    while RCC.CR.bitIsClr(RCC_CR_PLLRDY_Pos):
        discard                             # (8) Wait until PLLRDY is set
    # AHBpreScaler(/1):HPRE=0, default 0
    # APB1DiV(/4):PPRE1=b101
    # APB2Div(/2):PPRE2=b100
    RCC.CFGR.st (RCC.CFGR.ld or (RCC_CFGR_PPRE1_DIV4 or RCC_CFGR_PPRE2_DIV2)) #  set all prescaler
    RCC.CFGR.bset(1)                        # (9) Select PLL as system clock
    while RCC.CFGR.bitIsClr(3):             # (10) Wait until the PLL is switched on
        discard

