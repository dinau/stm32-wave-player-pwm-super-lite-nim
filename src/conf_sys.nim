import system
export system

# ----------------------
# -- Selectable Options
# ----------------------
const
    UART_INFO*                = true # Display "*.wav" file name with xprintf() (UART 11520bps)
    PWM16BIT*                 = true
    DATA_8BIT_SUPPORT*        = true

    HAVE_BUTTON_SW*           = true

    # ---
    HAVE_LED_IND_PWM*         = true # Select HAVE_LED_IND_PWM or HAVE_LED_IND_BLINK
    HAVE_LED_IND_BLINK*       = false
    # ---

    HAVE_LED_PAUSE_INDICATOR* = true  # When pause state, speed up the period of LED blink
    HAVE_POWER_OFF_MODE*      = false # N/A

    HAVE_FAT32*               = true  # if true:FAT32/FAT16, false:FAT16 only
    READ_WAV_HEADER_INFO*     = true
    FS_48KHZ_QUP*             = true  # Quality up

    CUT_LAST_TAG_NOISE*       = (50 * 1024) # 50k bytes

# ----------------------
# -- For debug purpose
# ----------------------
const
    TEST_PORT_ENABLE*         = true
    TOP_LEVEL_INFO*           = false
    SD_CARD_INFO*             = false  # Display SD card initialize info to UART.
    REG_INFO*                 = false  # Display MCU registor values.

# ----------------------
# -- For system type
# ----------------------
type
  word*  = uint16
  dword* = uint32
  sbyte* = int8
  sbit*  = bool
  sword* = int16

# ----------------------
# -- xprintf
# ----------------------
proc xprintf*(formatstr: cstring){.header: "xprintf.h", importc: "xprintf", varargs,used.}

