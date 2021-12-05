import conf_sys

proc regInfoSub*() =
    xprintf("\n RCC.AHB1ENR  = %08X (%08X)",RCC.AHB1ENR, addr RCC.AHB1ENR)
    xprintf("\n RCC.APB1ENR  = %08X (%08X)",RCC.APB1ENR, addr RCC.APB1ENR)
    xprintf("\n RCC.APB2ENR  = %08X (%08X)",RCC.APB2ENR, addr RCC.APB2ENR)

