import conf_sys
when REG_INFO and UART_INFO:          # For debug
    import  reginfo_sub

proc regInfo*(){.used.} =
    when REG_INFO and UART_INFO:          # For debug
        xprintf("\n========== Reg info ==========")
        xprintf("\n FLASH.ACR    = %08X (%08X)",FLASH.ACR,   addr FLASH.ACR)
        xprintf("\n RCC.CR       = %08X (%08X)",RCC.CR,      addr RCC.CR)
        xprintf("\n RCC.CFGR     = %08X (%08X)",RCC.CFGR,    addr RCC.CFGR)
        xprintf("\n RCC.ICSCR    = %08X (%08X)",RCC.ICSCR,   addr RCC.ICSCR)
        xprintf("\n RCC.SCR      = %08X (%08X)",RCC.CSR,     addr RCC.CSR)
        regInfoSub()
        # TIM3
        xprintf("\n TIM3.CR1     = %08X (%08X)",TIM3.CR1,    addr TIM3.CR1)
        xprintf("\n TIM3.CR2     = %08X (%08X)",TIM3.CR2,    addr TIM3.CR2)
        xprintf("\n TIM3.CCMR1   = %08X (%08X)",TIM3.CCMR1,  addr TIM3.CCMR1)
        xprintf("\n TIM3.CCMR2   = %08X (%08X)",TIM3.CCMR2,  addr TIM3.CCMR2)
        xprintf("\n TIM3.CCER    = %08X (%08X)",TIM3.CCER,   addr TIM3.CCER)
        xprintf("\n TIM3.CNT     = %08X (%08X)",TIM3.CNT,    addr TIM3.CNT)
        xprintf("\n TIM3.PSC     = %08X (%08X)",TIM3.PSC,    addr TIM3.PSC)
        xprintf("\n TIM3.ARR     = %08X (%08X)",TIM3.ARR,    addr TIM3.ARR)
        xprintf("\n TIM3.CCR1    = %08X (%08X)",TIM3.CCR1,   addr TIM3.CCR1)
        xprintf("\n TIM3.CCR2    = %08X (%08X)",TIM3.CCR2,   addr TIM3.CCR2)
        xprintf("\n TIM3.CCR3    = %08X (%08X)",TIM3.CCR3,   addr TIM3.CCR3)
        xprintf("\n TIM3.CCR4    = %08X (%08X)",TIM3.CCR4,   addr TIM3.CCR4)
        # SPI1
        xprintf("\n SPI1.CR1     = %08X (%08X)",SPI1.CR1,    addr SPI1.CR1)
        xprintf("\n SPI1.CR2     = %08X (%08X)",SPI1.CR2,    addr SPI1.CR2)
        xprintf("\n GPIOA.MODER  = %08X (%08X)",GPIOA.MODER ,addr GPIOA.MODER)
        xprintf("\n GPIOB.MODER  = %08X (%08X)",GPIOB.MODER ,addr GPIOB.MODER)
        xprintf("\n GPIOC.MODER  = %08X (%08X)",GPIOC.MODER ,addr GPIOC.MODER)
        xprintf("\n GPIOD.MODER  = %08X (%08X)",GPIOD.MODER ,addr GPIOD.MODER)
        xprintf("\n GPIOA.AFRL   = %08X (%08X)",GPIOA.AFR[0],addr GPIOA.AFR[0])
        xprintf("\n GPIOA.AFRH   = %08X (%08X)",GPIOA.AFR[1],addr GPIOA.AFR[1])
        xprintf("\n GPIOB.AFRL   = %08X (%08X)",GPIOB.AFR[0],addr GPIOB.AFR[0])
        xprintf("\n GPIOB.AFRH   = %08X (%08X)",GPIOB.AFR[1],addr GPIOB.AFR[1])
        xprintf("\n GPIOC.AFRL   = %08X (%08X)",GPIOC.AFR[0],addr GPIOC.AFR[0])
        xprintf("\n GPIOC.AFRH   = %08X (%08X)",GPIOC.AFR[1],addr GPIOC.AFR[1])
        xprintf("\n GPIOD.AFRL   = %08X (%08X)",GPIOD.AFR[0],addr GPIOD.AFR[0])
        xprintf("\n GPIOD.AFRH   = %08X (%08X)",GPIOD.AFR[1],addr GPIOD.AFR[1])
        xprintf("\n GPIOA.PUPDR  = %08X (%08X)",GPIOA.PUPDR,addr GPIOA.PUPDR)
        xprintf("\n GPIOB.PUPDR  = %08X (%08X)",GPIOB.PUPDR,addr GPIOB.PUPDR)
        xprintf("\n GPIOC.PUPDR  = %08X (%08X)",GPIOC.PUPDR,addr GPIOC.PUPDR)
        xprintf("\n GPIOD.PUPDR  = %08X (%08X)",GPIOD.PUPDR,addr GPIOD.PUPDR)

        xprintf("\n GPIOA.OTYPER = %08X (%08X)",GPIOA.OTYPER,addr GPIOA.OTYPER)
        xprintf("\n GPIOB.OTYPER = %08X (%08X)",GPIOB.OTYPER,addr GPIOB.OTYPER)
        xprintf("\n GPIOC.OTYPER = %08X (%08X)",GPIOC.OTYPER,addr GPIOC.OTYPER)
        xprintf("\n GPIOD.OTYPER = %08X (%08X)",GPIOD.OTYPER,addr GPIOD.OTYPER)

        xprintf("\n GPIOA.OSPEEDR= %08X (%08X)",GPIOA.OSPEEDR,addr GPIOA.OSPEEDR)
        xprintf("\n GPIOB.OSPEEDR= %08X (%08X)",GPIOB.OSPEEDR,addr GPIOB.OSPEEDR)
        xprintf("\n GPIOC.OSPEEDR= %08X (%08X)",GPIOC.OSPEEDR,addr GPIOC.OSPEEDR)
        xprintf("\n GPIOD.OSPEEDR= %08X (%08X)",GPIOD.OSPEEDR,addr GPIOD.OSPEEDR)

        xprintf("\n GPIOA.ODR    = %08X (%08X)",GPIOA.ODR,addr GPIOA.ODR)
        xprintf("\n GPIOB.ODR    = %08X (%08X)",GPIOB.ODR,addr GPIOB.ODR)
        xprintf("\n GPIOC.ODR    = %08X (%08X)",GPIOC.ODR,addr GPIOC.ODR)
        xprintf("\n GPIOD.ODR    = %08X (%08X)",GPIOD.ODR,addr GPIOD.ODR)
    else:discard

