import conf_sys
import sd_card,fat_lib,systick
import pwm,gpio,spi,board

# *******************************
#  external referenced functions
# *******************************
# proc wave_player_main*()

# *******************************
#  forward definitiion
# *******************************
proc calcPcmValidBits(fs:dword):uint8 {.inline.}

# *******************************
# local template
# *******************************
when HAVE_LED_IND_PWM:
    template ind_on()  = led(1)
    template ind_off() = led(0)
template music_stop() =
    pwm_period_timer_intr_stop()
    fPlaying = false
template music_start() =
    pwm_period_timer_intr_start()
    fPlaying = true

##################################
const
    bCH_STEREO      = 2    # CH_MONO:1
    wREAD_COUNT     = 512
    bHEADER_COUNT   = 44
var
    lbSample_bits   = 8.byte
    ldwSample_freq  = 44100.dword
    lbCh_mode:byte  = bCH_STEREO
    lwReadCount     = wREAD_COUNT
    fPlaying:bool
    ldwSongFileSectors:dword
    bGainNormalizeFactor:uint8
when not PWM16BIT:
    var bPCM_SHIFT_NUM:uint8

#################################
# TIM3_IRQHandler
#################################
{.emit:"""
#pragma GCC Push_options
#pragma GCC optimize ("O3")
""".}
proc TIM3_IRQHandler {.exportc.} =

    if get_pwm_period_timer_intr_if():
        when TEST_PORT_ENABLE: test_on()

        when PWM16BIT:
            var bL_low,bL_hi,bR_low,bR_hi:byte
            if lbSample_bits == 16:
                bL_low = send_ff()
                bL_hi  = send_ff() + 0x80'u8
                # copy for mono
                bR_low = bL_low
                bR_hi  = bL_hi
                lwReadCount -= 2
                if lbCh_mode == bCH_STEREO:
                    bR_low = send_ff()
                    bR_hi  = send_ff() + 0x80'u8
                    lwReadCount -= 2
            else:#/* 8bit data */
                when DATA_8BIT_SUPPORT:
                    bL_low= 0
                    bR_low= 0
                    bL_hi = send_ff()
                    # copy for mono
                    bR_hi = bL_hi
                    lwReadCount -= 1
                    if lbCh_mode == bCH_STEREO:#/* for stereo */
                        bR_hi = send_ff()
                        lwReadCount -= 1
            # Gain up
            var dwL:dword = (bL_hi.word shl 8 ) or  bL_low
            dwL = dwL shl bGainNormalizeFactor
            var dwR:dword = (bR_hi.word shl 8 )  or  bR_low
            dwR = dwR shl bGainNormalizeFactor

            # change pwm duties
            pwm_dutyL_hi(  dwL shr 8  )
            pwm_dutyL_low( dwL and 0xff)
            pwm_dutyR_hi(  dwR shr 8)
            pwm_dutyR_low( dwR and 0xff)

        else: # Not PWM16BIT (pwm: 9bit, 10bit, 11bit)
            var wL,wR:word
            if lbSample_bits == 16:
                wL = ( send_ff().word + (send_ff().word shl 8) + 0x8000) shr bPCM_SHIFT_NUM
                wR = wL
                lwReadCount -= 2
                if lbCh_mode == bCH_STEREO:
                    wR = (send_ff().word + (send_ff().word shl 8) + 0x8000) shr bPCM_SHIFT_NUM
                    lwReadCount -= 2
            else:#/* 8bit data */
                when DATA_8BIT_SUPPORT:
                    wL = send_ff().word shl bGainNormalizeFactor #
                    lwReadCount -= 1
                    if lbCh_mode == bCH_STEREO:#/* for stereo */
                        wR = send_ff().word shl bGainNormalizeFactor
                        lwReadCount -= 1
            # change  pwm duties
            pwm_dutyL_hi( wL )
            pwm_dutyR_hi( wR )

        #----------------------------------
        # check boundery of 512byte sector
        #----------------------------------
        if lwReadCount == 0:   #//; end one sector
            var bDummy :byte
            bDummy = send_ff() #// dummy read. discard CRC
            bDummy = send_ff() # 2nd dummy read.
            lwReadCount = wREAD_COUNT
            ldwSongFileSectors -= 1
            while send_ff() != 0xFE:discard
            if ldwSongFileSectors == 0:
                music_stop()

    when TEST_PORT_ENABLE: test_off()
    # end interruput
    clear_pwm_period_timer_intr_if()
    # end interruput
{.emit:"""
#pragma GCC Pop_options
""".}


# ***********************
#  varialble definitions
# ***********************
when HAVE_LED_IND_BLINK:
    const
        LED_PERIOD_PLAYNG  = 75
        LED_PERIOD_PAUSING = 10

# *****************
#  wave_player_main
# *****************
template fbtn_next_song_on: untyped = fbtn_short_on
template fbtn_pause_on:     untyped = fbtn_long_on

proc wave_player_main*() {.inline.} =
    when HAVE_LED_IND_BLINK:
        var bTimeout_led: byte
    var
        swBtnLowCount  :sword
        fbtn_bit_prev  :sbit = true
        fbtn_short_on  :sbit = false
        fbtn_long_on   :sbit = false
        fbtn_pause_prev:sbit = false

    when HAVE_POWER_OFF_MODE:
        fbtn_long_on2 = false
        fbtn_power_off_on = fbtn_long_on2

    when HAVE_LED_IND_PWM: #  pseudo PWM setting
        const
            IND_PERIOD         = 125.int8
            IND_DUTY_LOW_SPEED = 1.int8
            IND_DUTY_HI_SPEED  = 3.int8
        var
            sbIndDuty    = 0.int8
            sbIndCurrPos = 0.int8
            sbIndSpeed   = IND_DUTY_LOW_SPEED
            sbIndDelta   = sbIndSpeed

    while true:
        var prev = getTickCounter()
        while getTickCounter() < prev + 10: # ; wait 10msec
            if ldwSongFileSectors == 0:     # ; found end of file
                break                       # : promptly exit and prepare next song
            when HAVE_LED_IND_PWM:
                # ---------------------
                # pseudo PWM for LED
                # ---------------------
                if sbIndCurrPos < sbIndDuty: ind_on()
                else: ind_off()
                inc(sbIndCurrPos)
                if sbIndCurrPos == IND_PERIOD: sbIndCurrPos = 0

        # end while.  wait 10msec

        # ---------------------------------------------------
        # Start main process from here, called every 10msec.
        # ---------------------------------------------------

        # ------------------
        # Next song and start
        # ------------------
        if (ldwSongFileSectors == 0) or fbtn_next_song_on:
            music_stop()
            fbtn_next_song_on = false
            sd_stop_read()

            # ------------------
            # Search next song
            # ------------------
            searchNextFile()
            # Seek to Target file sector
            sd_start_read(gdwTargetFileSector)

            # ------------------
            # Cut music tag data
            # ------------------
            # Delete music tag data in end of wav file.
            # Music tag data size assumes about CUT_LAST_TAG_NOISE bytes at this moment.
            ldwSongFileSectors = (getBPB_FileSize() - CUT_LAST_TAG_NOISE.dword) shr 9 # convert to sectors

            when TOP_LEVEL_INFO:
                xprintf("\n getBPB_FileSize()=%d",getBPB_FileSize())
                xprintf("\n ldwSongFileSectors=%d",ldwsongFileSectors)

            # ------------------
            # Get wav header info
            # ------------------
            when READ_WAV_HEADER_INFO:
                sd_read_pulse_byte(22)    # pos(22) skip to channel info
                lbCh_mode = sd_data_byte()# pos(23)
                sd_data_byte()            # pos(24) skip to sampling freq.
                ldwSample_freq = sd_data_byte().dword + (sd_data_byte().dword shl 8) +
                                                        (sd_data_byte().dword shl 16) +
                                                        (sd_data_byte().dword shl 24) # pos(28)
                sd_read_pulse_byte(6)         # pos(34) skip to sample bits
                lbSample_bits = sd_data_byte()# pos(35)
                sd_read_pulse_byte(9)         # pos(44) skip to last position of header

                when TOP_LEVEL_INFO:
                    xprintf("\n gCh_mod = %d, ldwSample_freq = %d, lbSample_bits = %d",lbCh_mode ,ldwSample_freq,lbSample_bits)
            else:
                sd_read_pulse_byte(44)    # pos(44) just skip all wav header

            # ------------------
            # Set sampling frequency
            # ------------------
            setPwmPeriod(ldwSample_freq.dword)
            when PWM16BIT:
                bGainNormalizeFactor = calcPcmValidBits(ldwSample_freq) - 8'u8
                when TOP_LEVEL_INFO:
                    xprintf("\n bGainNormalizeFactor = %u",bGainNormalizeFactor)
            else:
                bPCM_SHIFT_NUM  = 16'u8 - calcPcmValidBits(ldwSample_freq)
                when TOP_LEVEL_INFO:
                    xprintf("\n bPCM_SHIFT_NUM = %u",bPCM_SHIFT_NUM)

            # ------------------
            # Music start
            # ------------------
            lwReadCount = wREAD_COUNT - bHEADER_COUNT
            music_start()

            when TOP_LEVEL_INFO:
                xprintf("\n Music start")

        # --------------------
        # LED indicator 1  --- pseudo PWM
        # ------------------*/
        when HAVE_LED_IND_PWM:
            sbIndDuty += sbIndDelta
            if sbIndDuty > IND_PERIOD: sbIndDelta = -1 * sbIndSpeed
            if sbIndDuty == 0:         sbIndDelta =      sbIndSpeed
            if fPlaying: sbIndSpeed = IND_DUTY_LOW_SPEED
            else:          sbIndSpeed = IND_DUTY_HI_SPEED
        # -------------------
        # LED indicator 2 --- simple ON/OFF
        # -------------------
        when HAVE_LED_IND_BLINK:
            if bTimeout_led == 0:
                if fPlaying:
                    bTimeout_led = LED_PERIOD_PLAYNG #  during Playing, on/off
                    ledToggle()
                else:
                    when HAVE_LED_PAUSE_INDICATOR:
                        bTimeout_led = LED_PERIOD_PAUSING #  during Pause,   on/off
                        ledToggle()
                    else:
                        bTimeout_led = 1
                        ind_off()
            bTimeout_led = bTimeout_led - 1
        # -------------------
        #  button sw input
        # -------------------
        when HAVE_BUTTON_SW:
            var btn = btn_bit_now()
            if fbtn_bit_prev xor btn: # input from port by btn_bit_now()
                if btn: #  ; 0 --> 1: btn released
                    if (swBtnLowCount > 10) and (swBtnLowCount < 130): # 100msec < x < 1.3sec
                        fbtn_short_on = true
                    swBtnLowCount = 0
            fbtn_bit_prev = btn
            if btn == false:
                swBtnLowCount += 1
            if swBtnLowCount > 120: #   ; 1.2sec >
                fbtn_long_on = true
            when HAVE_POWER_OFF_MODE:
                if swBtnLowCount > 400: # ; 4sec >
                    # ; "long on2" is meaning go to sleep mode
                    fbtn_long_on2 = true

            # -------------------
            #  Pause release
            # -------------------
            if not fPlaying: # if during pause
                if fbtn_next_song_on:
                    music_start()
                    fbtn_next_song_on = false
                    fbtn_pause_on = false
            # ------------------
            # Enter Pause
            # ------------------*/
            if fbtn_pause_prev xor fbtn_pause_on:
                if fbtn_pause_on:
                    music_stop()
            fbtn_pause_prev = fbtn_pause_on

    # ; [forever loop end]


#[/* WAVE signal effective bits
 * if fs==44.1kHz then:
 * PCM_SIGNIFICANT_BITS = int[ log_2[ (PERIOD_TIMER_INPUT_CLK / 44100 ) ] ]
 */]#
proc calcPcmValidBits(fs:dword):uint8 =
    result = 8
    when FS_48KHZ_QUP:
        if fs >= 48_000.dword:
            if TIM_PWM_BASE_CLOCK >= 99_000_000'u32:
                result = 11
            elif TIM_PWM_BASE_CLOCK >= 50_000_000'u32:
                result = 10
            elif TIM_PWM_BASE_CLOCK >= 25_000_000'u32:
                result = 9
            return result

    # --- fs <= 44.1KHz
    if TIM_PWM_BASE_CLOCK >= 91_000_000'u32:
        result = 11
    elif TIM_PWM_BASE_CLOCK >= 46_000_000'u32:
        result = 10
    elif TIM_PWM_BASE_CLOCK >= 23_000_000'u32:
        result = 9

