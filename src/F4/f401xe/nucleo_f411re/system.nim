import stm32f401xe
export stm32f401xe

# ----------------------
# -- For System constant
# ----------------------
const
    SYSTEM_CLOCK*   = 96_000_000'u32
    UART_BAUDRATE*  = 115200
    # USART2 base clock is SYSTEM_CLOCK / 2 = 48MHz
    BAUD_GEN* = (SYSTEM_CLOCK shr 1) div UART_BAUDRATE
    TIM_PWM_BASE_CLOCK* = SYSTEM_CLOCK # = 96MHz

proc initSystem*() {.inline.} =
    # enable FPU for Cortex-M4
    SCB.CPACR.st (SCB.CPACR.ld or ((3 shl 10*2) or (3 shl 11*2 ))) # /* set CP10 and CP11 Full Access */

# set PLL 96MHz for STM32F411
proc initSysclock*() {.inline.} =           #
    FLASH.ACR.st FLASH_ACR_LATENCY_3WS      # Latency: 3WS(4cycle) :  90MHz  < HCLK < 100MHz
    if RCC.CFGR.bitIsSet(1):                # (1) Test if PLL is used as System clock
        RCC.CFGR.bclr(1,0)                  # (2) Select HSI as system clock
        while not RCC.CFGR.bitIsClr(3,2):
            discard                         # (3) Wait for switched to HSI
    RCC.CR.bclr(RCC_CR_PLLON_Pos)           # (4) Disable the PLL
    while RCC.CR.bitIsSet(RCC_CR_PLLRDY_Pos):
        discard                             # (5) Wait until PLLRDY is cleared
    # set parameters SYSTEM_CLOCK = 96MHz
    # PLLM=16,PLLN=384,PLLP=4
    let PLLN = (384 shl RCC_PLLCFGR_PLLN_Pos)
    let PLLM = 16
    let PLLP = RCC_PLLCFGR_PLLP_0
    RCC.PLLCFGR.st ( PLLN or PLLM or PLLP ) # (6) Set the PLL parameters
    RCC.CR.bset(RCC_CR_PLLON_Pos)           # (7) Enable PLL
    while RCC.CR.bitIsClr(RCC_CR_PLLRDY_Pos):
        discard                             # (8) Wait until PLLRDY is set
    # AHBpreScaler(/1):HPRE=0, default 0         AHB:  96MHz
    # APB1DiV(/2):PPRE1=b100,                    APB1: 48MHz
    # APB2Div(/1):PPRE2=b0xx, default 0          APB2: 96MHz
    RCC.CFGR.st RCC_CFGR_PPRE1_2            #  set all prescale
    RCC.CFGR.bset(1)                        # (9) Select PLL as system clock
    while RCC.CFGR.bitIsClr(3):             # (10) Wait until the PLL is switched on
        discard

