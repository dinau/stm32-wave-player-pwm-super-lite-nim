import conf_sys

when UART_INFO:
    proc UART_putc(c:char){.used,exportc.} =
        while USART2.ISR.bitIsClr(USART_ISR_TXE_Pos):
            discard
        USART2.TDR.st c.uint8

    proc putString*(s:cstring){.used.} =
        for c in s:
            UART_putc(c)

    # Use ChaN's xprintf()
    let xfunc_out{.importc,used.} = UART_putc

    ###########################
    # initSerial
    ###########################
    proc initSerial*(usart:auto) {.inline.} = # USART2
        if usart == USART2:
            block:
                # PA2     ------> USART2_TX
                # PA3     ------> USART2_RX
                RCC.APB1ENR.bset(RCC_APB1ENR_USART2EN_Pos) # Enable USART2 clock
                RCC.AHBENR.bset(RCC_AHBENR_GPIOAEN_Pos)    # GPIOA clock enable
                # TX PA2, RX PA3
                GPIOA.MODER.bset(GPIO_MODER_MODER3_Pos + 1,GPIO_MODER_MODER2_Pos + 1)
                # Select AF1 function -> USART
                GPIOA.AFR[0].bset(GPIO_AFRL_AFSEL3_Pos,GPIO_AFRL_AFSEL2_Pos)
            #### Set parameters ###
            usart.BRR.st BAUD_GEN
            ########################
            usart.CR1.bset(USART_CR1_TE_Pos,
                           USART_CR1_RE_Pos,
                           USART_CR1_UE_Pos)  # TX,RX,USART enable 'UE'

