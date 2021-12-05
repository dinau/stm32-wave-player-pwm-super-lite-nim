import conf_sys

proc regInfoSub*() =
    xprintf("\n RCC.AHBENR   = %08X (%08X)",RCC.AHBENR,  addr RCC.AHBENR)
    xprintf("\n RCC.APB1ENR  = %08X (%08X)",RCC.APB1ENR, addr RCC.APB1ENR)
    xprintf("\n RCC.APB2ENR  = %08X (%08X)",RCC.APB2ENR, addr RCC.APB2ENR)

