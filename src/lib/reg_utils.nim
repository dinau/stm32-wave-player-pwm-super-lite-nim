#[
MIT License

Copyright (c) 2018 audin
    Modified and added source code.

Copyright (c) 2018 shima-529

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.]#

import volatile

# m0 core utils
# from stm32f0Cube\Drivers\CMSIS\Include\cmsis_gcc.h
{.push stackTrace:off.}
template nop*() =
    asm "nop"
template Nop*() =
    asm "nop"

template wfi*()  =
    asm "wfi"

template sleepIntr*()  = wfi()

template wfe*()  =
    asm "wfe"

template sleepEvent*()  = wfe()

proc enable_irq*() {.used,inline.} =
    asm """
        "cpsie i" : : : "memory"
    """
proc disable_irq*() {.used,inline.} =
    asm """
        "cpsid i" : : : "memory"
    """
proc rev16*(val:uint32):uint32 {.used,inline.} =
    asm """
        rev16 %0,%1
        :"=l"(`result`)
        :"l"(`val`)
    """
proc DSB*() =
    asm """
       "dsb 0xF":::"memory"
    """


proc volatile_load8*(src: ptr uint8):uint8 {.inline.} =
    {.emit: [result, " = *(", uint8, " volatile *)", src, ";"].}

proc volatileStore8*(dst: ptr uint8, val: uint8) {.inline.} =
    {.emit: ["*(", uint8, " volatile *)", dst, " = ", val, ";"].}

proc volatile_load16*(src: ptr uint16):uint16 {.inline.} =
    {.emit: [result, " = *(", uint16, " volatile *)", src, ";"].}

proc volatileStore16*(dst: ptr uint16, val: uint16){.inline.} =
    {.emit: ["*(", uint16, " volatile *)", dst, " = ", val, ";"].}
{.pop.}

## *
##  @}
##
## * @addtogroup Exported_macros
##  @{
##
when false:
    template SET_BIT*(REG, BIT: untyped): untyped =
      ( volatileStore( (REG).addr, volatileLoad( (REG).addr ) or (BIT) )  )

    template CLEAR_BIT*(REG, BIT: untyped): untyped =
      ( volatileStore( (REG).addr, volatileload( (REG).addr ) and not (BIT) ) )

    template READ_BIT*(REG, BIT: untyped): untyped =
      ( volatileLoad( (REG).addr ) and (BIT) )

    template CLEAR_REG*(REG: untyped): untyped =
      volatileStore((REG).addr , (0x00000000))

    template WRITE_REG*(REG, VAL: untyped): untyped =
      volatileStore((REG).addr, (VAL))

    template READ_REG*(REG: untyped): untyped =
      volatileload((REG).addr)

    template MODIFY_REG*(REG, CLEARMASK, SETMASK: untyped): untyped =
      WRITE_REG((REG), (((READ_REG(REG)) and (not (CLEARMASK.uint32))) or (SETMASK.uint32)))

template IS_BITSET*(REG, BIT_POS: untyped): untyped =
  ( xVolatileLoad( (REG).addr ) and (1 shl BIT_POS) ) == (1 shl BIT_POS)
template IS_BITCLR*(REG, BIT_POS: untyped): untyped =
  ( xVolatileLoad( (REG).addr ) and (1 shl BIT_POS) ) == 0
## *
##  @}
##

template ld*[T: SomeInteger](reg: T): T =
  volatileLoad(reg.addr)

template st*[T,U: SomeInteger](reg: T, val: U) =
  volatileStore(cast[ptr U](reg.addr), val)

#[
template st8*[T:,U: SomeInteger](reg: T, val: U) =
  volatileStore8(cast[ptr uint8](reg.addr), cast[uint8](val))

template st16*[T,U: SomeInteger](reg: T, val: U) =
  volatileStore16(cast[ptr uint16](reg.addr), cast[uint16](val))
]#

template ld8*[T: SomeInteger](reg: T): uint8 =
  volatile_Load8(cast[ptr uint8](reg.addr) )

template ld16*[T: SomeInteger](reg: T): uint16 =
  volatile_Load16(cast[ptr uint16](reg.addr) )

template bit*[T: SomeInteger](n: varargs[T]): T =
  var ret: T = 0;
  for i in n:
    ret = ret or (1 shl i)
  ret

template shift*[T, U: SomeInteger](reg: T, n: U): T =
  reg shl n

template bset*[T, U: SomeInteger](reg: T, n :varargs[U]) =
  reg.st ((reg.ld) or cast[T](bit(n)))

template bclr*[T, U: SomeInteger](reg: T, n :varargs[U]) =
  reg.st ((reg.ld) and not cast[T](bit(n)))

template bitIsSet*[T, U: SomeInteger](reg: T, n: varargs[U]): bool =
  (((reg.ld) and cast[T](bit(n))) != 0)

template bitIsClr*[T, U: SomeInteger](reg: T, n: varargs[U]): bool =
   (not bitIsSet(reg, n))




