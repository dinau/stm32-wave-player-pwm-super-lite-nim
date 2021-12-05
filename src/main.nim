{.experimental, deadCodeElim: on.}

import conf_sys
import reginfo
import system, uart, pwm, gpio, spi, board
import sd_card,wave_player_main, fat_lib
import systick
#[ /* SD card pin
 Pin side
 --------------\
         9     = \    DAT2/NC
             1 ===|   CS/DAT3    [CS]
             2 ===|   CMD/DI     [DI]
             3 ===|   VSS1
 Bottom      4 ===|   VDD
 View        5 ===|   CLK        [CLK]
             6 ===|   VSS2
             7 ===|   DO/DAT0    [DO]
         8       =|   DAT1/IRQ
 -----------------

                                         Arduino      NUCLEO-F411       NUCLEO-F030R8
 Logo side
 -----------------
         8       =|   DAT1/IRQ
             7 ===|   DO/DAT0    [DO]     D12           D12/PA_6           D12/PA_6
             6 ===|   VSS2
 Top         5 ===|   CLK        [CLK]    D13           D13/PA_5           D13/PA_5
 View        4 ===|   VDD
             3 ===|   VSS1
             2 ===|   CMD/DI     [DI]     D11           D11/PA_7           D11/PA_7
             1 ===|   CS/DAT3    [CS]     D8            D10/PB_6           D10/PB_6
         9     = /    DAT2/NC
 --------------/
 */
]#

# Peripheral map:  See "port_setting.txt"
#
{.compile:"xprintf.c".}

##################
# main function
##################
proc main*() =
    initSystem()
    initSysclock()
    initGPIO()
    initBoard()
   # Start systick timer and interrupt
    SysTick_Config(SYSTEM_CLOCK div 1000) # 1msec (1000Hz)
    when UART_INFO:
        initSerial(USART2)                # 115200bps
    initSPI(SPI1)                         #
    initPwm()                             # Period fs=44.1kHz
    regInfo()                             # Display MCU registor values.
    # SDSC,SDHC initialize
    while sd_init()==false:
        wait_ms(1000)
    FAT_init()                            # Accept FAT16 and FAT32
    regInfo()                             # 2nd
    wave_player_main()                    #






