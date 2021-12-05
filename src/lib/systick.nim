
var tickcounter {.volatile.}:int

proc getTickCounter*() :int=
    tickcounter

proc setTickCounter*(ms:int)=
    tickcounter = ms

proc wait_ms*(ms:int)=
   let prev = tickcounter
   while tickcounter < prev + ms:
        discard

proc SysTick_Handler {.exportc.} = # Called every 1msec
        inc(tickcounter)

# Direct setting without library
#proc initSystick*() =
#    const SYSTICK_RELOAD_VAL   = ( SYSTEM_CLOCK  div 1000 ) - 1 # 1msec
#    # use cpu clock = SYSTEM_CLOCK
#    stm32f0_core.SysTick.LOAD.st SYSTICK_RELOAD_VAL
#    stm32f0_core.SysTick.VAL.st 0    # clear counter
#    # bit2: Use cpu clock
#    # bit0: Count down timer start
#    # bit1: Enable Systick interrupt
#    stm32f0_core.Systick.CTRL.bset(2,0,1)
#
#    SysTick_IRQn.NVIC_EnableIRQ
#    SysTick_IRQn.NVIC_SetPriority 3

