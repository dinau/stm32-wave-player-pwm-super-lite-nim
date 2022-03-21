import conf_sys
import system, uart, gpio
import board
import systick

{.compile:"xprintf.c".}
# set PLL 96MHz for STM32F411

proc main*(){.inline.} = # main function
    initSystem()
    initSysclock()
    initGPIO()
    initBoard()
    # Start systick timer and interrupt
    SysTick_Config(SYSTEM_CLOCK div 1000) # 1msec (1000Hz)
    # Set LED port
    RCC.AHB1ENR.bset(RCC_AHB1ENR_GPIOAEN_Pos)  # GPIOA clock enable
    GPIOA.MODER.bset(GPIO_MODER_MODER5_Pos)

    when UART_INFO:
        initSerial(USART2)                # 115200bps

    var ix = 0
    while true:
        xprintf("\nhello :%d",ix)
        inc(ix)
        GPIOA.atomic_out(5,1)
        wait_ms(500)
        GPIOA.atomic_out(5,0)
        wait_ms(500)


# run main program
main()

