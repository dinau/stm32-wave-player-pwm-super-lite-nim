import conf_sys
import system,systick, uart, pwm, gpio, spi
import board
import regInfo

{.compile:"xprintf.c".}
# set PLL 96MHz for STM32F411

proc main*(){.inline.} = # main function
    const MSS = 1000000
    initSystem()
    initSysclock()
    initGPIO()
    initBoard()
    #RCC.AHB1ENR.bset(RCC_AHB1ENR_GPIOAEN_Pos)  # GPIOA clock enable
    #GPIOA.MODER.bset(GPIO_MODER_MODER5_Pos)
    when UART_INFO:
        initSerial(USART2)                # 115200bps
    while true:
        xprintf("\nhello")
    while false:
        for _ in 0..MSS:
            GPIOA.atomic_out(5,1)
        for _ in 0..MSS:
            GPIOA.atomic_out(5,0)


# run main program
main()

