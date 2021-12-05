import system
export system


const
    UART_INFO* = true
    PWM16BIT*  = false
    REG_INFO*  = false

# ----------------------
# -- xprintf
# ----------------------
proc xprintf*(formatstr: cstring){.header: "xprintf.h", importc: "xprintf", varargs,used.}

