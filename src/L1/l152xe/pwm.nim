import conf_sys

#[
// Use PWM3-1,PWM3-2 with TIM3
#define PWM_PORT_LEFT_HI    PB_5    // TIM3_CH2
#define PWM_PORT_RIGHT_HI   PB_4    // TIM3_CH1
// Use PWM3-3,PWM3-4 with TIM3
#define PWM_PORT_LEFT_LOW   PC_9    // TIM3_CH4
#define PWM_PORT_RIGHT_LOW  PC_8    // TIM3_CH3
]#

const HERZ_44100 = 44100

###########################
# pwm_period_timer_start/stop
###########################
proc pwm_period_timer_intr_start*() {.inline.} =
    TIM3_IRQn.NVIC_EnableIRQ
   #TIM3.EGR.bset(TIM_EGR_UG_Pos)

proc pwm_period_timer_intr_stop*() {.inline.} =
    TIM3_IRQn.NVIC_disableIRQ
   #TIM3.EGR.bclr(TIM_EGR_UG_Pos)

###########################
# timer3seup
###########################
proc timer3seup() {.inline.} =
    RCC.APB1ENR.bset(RCC_APB1ENR_TIM3EN_Pos)           # TIM3 base clock enable
    TIM3.CR1.bset(TIM_CR1_CEN_Pos, TIM_CR1_ARPE_Pos)   # (CEN,ARPE)
    TIM3.DIER.bset(TIM_DIER_UIE_Pos)                   # (UIE) Update. Enable period interrupt.
    TIM3.PSC.st  0                                     # No prescalor
    TIM3_IRQn.NVIC_SetPriority 0                       # Set max priorty.

###########################
# pwm_duty
###########################
proc pwm_dutyR_hi*(duty:auto) {.inline.} =
    TIM3.CCR1.st duty
proc pwm_dutyL_hi*(duty:auto) {.inline.} =
    TIM3.CCR2.st duty
proc pwm_dutyR_low*(duty:auto) {.inline.} =
    TIM3.CCR3.st duty
proc pwm_dutyL_low*(duty:auto) {.inline.} =
    TIM3.CCR4.st duty

proc pwmL_duty*(duty:auto) {.inline.} =
    pwmDutyLHi(duty)
proc pwmR_duty*(duty:auto) {.inline.} =
    pwmDutyRHi(duty)

###########################
# setPwmPeriod
###########################
proc setPwmPeriod*(freq:uint32) {.inline.} =
    TIM3.CCER.st 0       # ch1～ch4 stop output. Inhibit during setting.
    TIM3.ARR.st (TIM_PWM_BASE_CLOCK div freq) # set PWM period.
    when PWM16BIT:
        TIM3.CCER.st 0x1111  # ch1～ch4 enable output
    else:
        TIM3.CCER.st 0x0011  # ch1,ch2 enable output

###########################
# initPwm
###########################
proc initPwm*() {.inline.} =
    timer3seup()
    block: # setup output port and pwm
        RCC.AHBENR.bset(RCC_AHBENR_GPIOBEN_Pos)    # GPIOB clock enable
        # PB4,[AF2],TIM3_CH1/PWM: Right Hi
        # PB5,[AF2],TIM3_CH2/PWM: Left  Hi
        GPIOB.MODER.bset(GPIO_MODER_MODER4_Pos + 1,GPIO_MODER_MODER5_Pos + 1)
        # [AF2] Select function -> PWM CHx
        GPIOB.AFR[0].bset(GPIO_AFRL_AFSEL4_Pos + 1 ,GPIO_AFRL_AFSEL5_Pos + 1)

        GPIOB.PUPDR.bclr(GPIO_MODER_MODER4_Pos) # PB4 none pullup
        GPIOB.PUPDR.bclr(GPIO_MODER_MODER5_Pos) # PB5 none pullup

        when PWM16BIT:
            RCC.AHBENR.bset(RCC_AHBENR_GPIOCEN_Pos)    # GPIOC clock enable
            # PC8,[AF2],TIM3_CH3/PWM
            # PC9,[AF2],TIM3_CH4/PWM
            GPIOC.MODER.bset(GPIO_MODER_MODER8_Pos + 1,GPIO_MODER_MODER9_Pos + 1)
            # [AF2] Select function -> PWM CHx
            GPIOC.AFR[1].bset(GPIO_AFRH_AFSEL8_Pos + 1 ,GPIO_AFRH_AFSEL9_Pos + 1)

            GPIOB.PUPDR.bclr(GPIO_MODER_MODER8_Pos) # PC8 none pullup
            GPIOB.PUPDR.bclr(GPIO_MODER_MODER9_Pos) # PC9 none pullup

    #
    TIM3.CCER.st 0       # ch1..ch4 stop output. Inhibit during setting.
    # Start setting
    TIM3.ARR.st (TIM_PWM_BASE_CLOCK div HERZ_44100) # set PWM period.
    when PWM16BIT:
        TIM3.CCMR1.st 0x6868 # ch1,ch2: set PWM1 function
        TIM3.CCMR2.st 0x6868 # ch3,ch4: set PWM1 function
        # End setting
        TIM3.CCER.st 0x1111  # ch1..ch4 enable output
    else:
        TIM3.CCMR1.st 0x6868 # ch1,ch2:set PWM1 function
        # End setting
        TIM3.CCER.st 0x0011  # ch1,ch2:set PWM1 function

    # Start PWM period timer
    TIM3.EGR.bset(TIM_EGR_UG_Pos)     # (EGR_UG) set counter = 0 when overflow

proc get_pwm_period_timer_intr_if*():bool =
    TIM3.SR.bitIsSet(TIM_SR_UIF_Pos)

proc clear_pwm_period_timer_intr_if*() =
    TIM3.SR.bclr(TIM_SR_UIF_Pos)

