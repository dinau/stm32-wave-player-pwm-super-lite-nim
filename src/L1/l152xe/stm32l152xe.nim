## *
## *****************************************************************************
##  @file    stm32l152xe.h
##  @author  MCD Application Team
##  @brief   CMSIS Cortex-M3 Device Peripheral Access Layer Header File.
##           This file contains all the peripheral register's definitions, bits
##           definitions and memory mapping for STM32L1xx devices.
##
##           This file contains:
##            - Data structures and the address mapping for all peripherals
##            - Peripheral's registers declarations and bits definition
##            - Macros to access peripheral’s registers hardware
##
## *****************************************************************************
##  @attention
##
##  <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
##
##  Redistribution and use in source and binary forms, with or without modification,
##  are permitted provided that the following conditions are met:
##    1. Redistributions of source code must retain the above copyright notice,
##       this list of conditions and the following disclaimer.
##    2. Redistributions in binary form must reproduce the above copyright notice,
##       this list of conditions and the following disclaimer in the documentation
##       and/or other materials provided with the distribution.
##    3. Neither the name of STMicroelectronics nor the names of its contributors
##       may be used to endorse or promote products derived from this software
##       without specific prior written permission.
##
##  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
##  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
##  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
##  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
##  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
##  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
##  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
##  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
##  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
##  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
## *****************************************************************************
##
## * @addtogroup CMSIS
##  @{
##
## * @addtogroup stm32l152xe
##  @{
##

## * @addtogroup Configuration_section_for_CMSIS
##  @{
##
## *
##  @brief Configuration of the Cortex-M3 Processor and Core Peripherals
##

const
  CM3_REV* = 0x00000200
  MPU_PRESENT* = 1
  NVIC_PRIO_BITS* = 4
  Vendor_SysTickConfig* = 0

## *
##  @}
##
## * @addtogroup Peripheral_interrupt_number_definition
##  @{
##
## *
##  @brief STM32L1xx Interrupt Number Definition, according to the selected device
##         in @ref Library_configuration_section
##
## !< Interrupt Number Definition

type                          ## *****  Cortex-M3 Processor Exceptions Numbers *****************************************************
  IRQn_Type* = enum
    NonMaskableInt_IRQn = -14,  ## !< 2 Non Maskable Interrupt
    HardFault_IRQn = -13,       ## !< 3 Cortex-M3 Hard Fault Interrupt
    MemoryManagement_IRQn = -12, ## !< 4 Cortex-M3 Memory Management Interrupt
    BusFault_IRQn = -11,        ## !< 5 Cortex-M3 Bus Fault Interrupt
    UsageFault_IRQn = -10,      ## !< 6 Cortex-M3 Usage Fault Interrupt
    SVC_IRQn = -5,              ## !< 11 Cortex-M3 SV Call Interrupt
    DebugMonitor_IRQn = -4,     ## !< 12 Cortex-M3 Debug Monitor Interrupt
    PendSV_IRQn = -2,           ## !< 14 Cortex-M3 Pend SV Interrupt
    SysTick_IRQn = -1,          ## !< 15 Cortex-M3 System Tick Interrupt
                    ## *****  STM32L specific Interrupt Numbers **********************************************************
    WWDG_IRQn = 0,              ## !< Window WatchDog Interrupt
    PVD_IRQn = 1,               ## !< PVD through EXTI Line detection Interrupt
    TAMPER_STAMP_IRQn = 2,      ## !< Tamper and TimeStamp interrupts through the EXTI line
    RTC_WKUP_IRQn = 3,          ## !< RTC Wakeup Timer through EXTI Line Interrupt
    FLASH_IRQn = 4,             ## !< FLASH global Interrupt
    RCC_IRQn = 5,               ## !< RCC global Interrupt
    EXTI0_IRQn = 6,             ## !< EXTI Line0 Interrupt
    EXTI1_IRQn = 7,             ## !< EXTI Line1 Interrupt
    EXTI2_IRQn = 8,             ## !< EXTI Line2 Interrupt
    EXTI3_IRQn = 9,             ## !< EXTI Line3 Interrupt
    EXTI4_IRQn = 10,            ## !< EXTI Line4 Interrupt
    DMA1_Channel1_IRQn = 11,    ## !< DMA1 Channel 1 global Interrupt
    DMA1_Channel2_IRQn = 12,    ## !< DMA1 Channel 2 global Interrupt
    DMA1_Channel3_IRQn = 13,    ## !< DMA1 Channel 3 global Interrupt
    DMA1_Channel4_IRQn = 14,    ## !< DMA1 Channel 4 global Interrupt
    DMA1_Channel5_IRQn = 15,    ## !< DMA1 Channel 5 global Interrupt
    DMA1_Channel6_IRQn = 16,    ## !< DMA1 Channel 6 global Interrupt
    DMA1_Channel7_IRQn = 17,    ## !< DMA1 Channel 7 global Interrupt
    ADC1_IRQn = 18,             ## !< ADC1 global Interrupt
    USB_HP_IRQn = 19,           ## !< USB High Priority Interrupt
    USB_LP_IRQn = 20,           ## !< USB Low Priority Interrupt
    DAC_IRQn = 21,              ## !< DAC Interrupt
    COMP_IRQn = 22,             ## !< Comparator through EXTI Line Interrupt
    EXTI9_5_IRQn = 23,          ## !< External Line[9:5] Interrupts
    LCD_IRQn = 24,              ## !< LCD Interrupt
    TIM9_IRQn = 25,             ## !< TIM9 global Interrupt
    TIM10_IRQn = 26,            ## !< TIM10 global Interrupt
    TIM11_IRQn = 27,            ## !< TIM11 global Interrupt
    TIM2_IRQn = 28,             ## !< TIM2 global Interrupt
    TIM3_IRQn = 29,             ## !< TIM3 global Interrupt
    TIM4_IRQn = 30,             ## !< TIM4 global Interrupt
    I2C1_EV_IRQn = 31,          ## !< I2C1 Event Interrupt
    I2C1_ER_IRQn = 32,          ## !< I2C1 Error Interrupt
    I2C2_EV_IRQn = 33,          ## !< I2C2 Event Interrupt
    I2C2_ER_IRQn = 34,          ## !< I2C2 Error Interrupt
    SPI1_IRQn = 35,             ## !< SPI1 global Interrupt
    SPI2_IRQn = 36,             ## !< SPI2 global Interrupt
    USART1_IRQn = 37,           ## !< USART1 global Interrupt
    USART2_IRQn = 38,           ## !< USART2 global Interrupt
    USART3_IRQn = 39,           ## !< USART3 global Interrupt
    EXTI15_10_IRQn = 40,        ## !< External Line[15:10] Interrupts
    RTC_Alarm_IRQn = 41,        ## !< RTC Alarm through EXTI Line Interrupt
    USB_FS_WKUP_IRQn = 42,      ## !< USB FS WakeUp from suspend through EXTI Line Interrupt
    TIM6_IRQn = 43,             ## !< TIM6 global Interrupt
    TIM7_IRQn = 44,             ## !< TIM7 global Interrupt
    TIM5_IRQn = 46,             ## !< TIM5 global Interrupt
    SPI3_IRQn = 47,             ## !< SPI3 global Interrupt
    UART4_IRQn = 48,            ## !< UART4 global Interrupt
    UART5_IRQn = 49,            ## !< UART5 global Interrupt
    DMA2_Channel1_IRQn = 50,    ## !< DMA2 Channel 1 global Interrupt
    DMA2_Channel2_IRQn = 51,    ## !< DMA2 Channel 2 global Interrupt
    DMA2_Channel3_IRQn = 52,    ## !< DMA2 Channel 3 global Interrupt
    DMA2_Channel4_IRQn = 53,    ## !< DMA2 Channel 4 global Interrupt
    DMA2_Channel5_IRQn = 54,    ## !< DMA2 Channel 5 global Interrupt
    COMP_ACQ_IRQn = 56


## *
##  @}
##

## * @addtogroup Peripheral_registers_structures
##  @{
##
## *
##  @brief Analog to Digital Converter
##

type
  ADC_TypeDef* {.bycopy.} = object
    SR*: uint32                ## !< ADC status register,                         Address offset: 0x00
    CR1*: uint32               ## !< ADC control register 1,                      Address offset: 0x04
    CR2*: uint32               ## !< ADC control register 2,                      Address offset: 0x08
    SMPR1*: uint32             ## !< ADC sample time register 1,                  Address offset: 0x0C
    SMPR2*: uint32             ## !< ADC sample time register 2,                  Address offset: 0x10
    SMPR3*: uint32             ## !< ADC sample time register 3,                  Address offset: 0x14
    JOFR1*: uint32             ## !< ADC injected channel data offset register 1, Address offset: 0x18
    JOFR2*: uint32             ## !< ADC injected channel data offset register 2, Address offset: 0x1C
    JOFR3*: uint32             ## !< ADC injected channel data offset register 3, Address offset: 0x20
    JOFR4*: uint32             ## !< ADC injected channel data offset register 4, Address offset: 0x24
    HTR*: uint32               ## !< ADC watchdog higher threshold register,      Address offset: 0x28
    LTR*: uint32               ## !< ADC watchdog lower threshold register,       Address offset: 0x2C
    SQR1*: uint32              ## !< ADC regular sequence register 1,             Address offset: 0x30
    SQR2*: uint32              ## !< ADC regular sequence register 2,             Address offset: 0x34
    SQR3*: uint32              ## !< ADC regular sequence register 3,             Address offset: 0x38
    SQR4*: uint32              ## !< ADC regular sequence register 4,             Address offset: 0x3C
    SQR5*: uint32              ## !< ADC regular sequence register 5,             Address offset: 0x40
    JSQR*: uint32              ## !< ADC injected sequence register,              Address offset: 0x44
    JDR1*: uint32              ## !< ADC injected data register 1,                Address offset: 0x48
    JDR2*: uint32              ## !< ADC injected data register 2,                Address offset: 0x4C
    JDR3*: uint32              ## !< ADC injected data register 3,                Address offset: 0x50
    JDR4*: uint32              ## !< ADC injected data register 4,                Address offset: 0x54
    DR*: uint32                ## !< ADC regular data register,                   Address offset: 0x58
    SMPR0*: uint32             ## !< ADC sample time register 0,                  Address offset: 0x5C

  ADC_Common_TypeDef* {.bycopy.} = object
    CSR*: uint32               ## !< ADC common status register,                  Address offset: ADC1 base address + 0x300
    CCR*: uint32               ## !< ADC common control register,                 Address offset: ADC1 base address + 0x304


## *
##  @brief Comparator
##

type
  COMP_TypeDef* {.bycopy.} = object
    CSR*: uint32               ## !< COMP control and status register, Address offset: 0x00

  COMP_Common_TypeDef* {.bycopy.} = object
    CSR*: uint32               ## !< COMP control and status register, used for bits common to several COMP instances, Address offset: 0x00


## *
##  @brief CRC calculation unit
##

type
  CRC_TypeDef* {.bycopy.} = object
    DR*: uint32                ## !< CRC Data register,                           Address offset: 0x00
    IDR*: uint8                ## !< CRC Independent data register,               Address offset: 0x04
    RESERVED0*: uint8          ## !< Reserved,                                    Address offset: 0x05
    RESERVED1*: uint16         ## !< Reserved,                                    Address offset: 0x06
    CR*: uint32                ## !< CRC Control register,                        Address offset: 0x08


## *
##  @brief Digital to Analog Converter
##

type
  DAC_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< DAC control register,                                     Address offset: 0x00
    SWTRIGR*: uint32           ## !< DAC software trigger register,                            Address offset: 0x04
    DHR12R1*: uint32           ## !< DAC channel1 12-bit right-aligned data holding register,  Address offset: 0x08
    DHR12L1*: uint32           ## !< DAC channel1 12-bit left aligned data holding register,   Address offset: 0x0C
    DHR8R1*: uint32            ## !< DAC channel1 8-bit right aligned data holding register,   Address offset: 0x10
    DHR12R2*: uint32           ## !< DAC channel2 12-bit right aligned data holding register,  Address offset: 0x14
    DHR12L2*: uint32           ## !< DAC channel2 12-bit left aligned data holding register,   Address offset: 0x18
    DHR8R2*: uint32            ## !< DAC channel2 8-bit right-aligned data holding register,   Address offset: 0x1C
    DHR12RD*: uint32           ## !< Dual DAC 12-bit right-aligned data holding register,      Address offset: 0x20
    DHR12LD*: uint32           ## !< DUAL DAC 12-bit left aligned data holding register,       Address offset: 0x24
    DHR8RD*: uint32            ## !< DUAL DAC 8-bit right aligned data holding register,       Address offset: 0x28
    DOR1*: uint32              ## !< DAC channel1 data output register,                        Address offset: 0x2C
    DOR2*: uint32              ## !< DAC channel2 data output register,                        Address offset: 0x30
    SR*: uint32                ## !< DAC status register,                                      Address offset: 0x34


## *
##  @brief Debug MCU
##

type
  DBGMCU_TypeDef* {.bycopy.} = object
    IDCODE*: uint32            ## !< MCU device ID code,                          Address offset: 0x00
    CR*: uint32                ## !< Debug MCU configuration register,            Address offset: 0x04
    APB1FZ*: uint32            ## !< Debug MCU APB1 freeze register,              Address offset: 0x08
    APB2FZ*: uint32            ## !< Debug MCU APB2 freeze register,              Address offset: 0x0C


## *
##  @brief DMA Controller
##

type
  DMA_Channel_TypeDef* {.bycopy.} = object
    CCR*: uint32               ## !< DMA channel x configuration register
    CNDTR*: uint32             ## !< DMA channel x number of data register
    CPAR*: uint32              ## !< DMA channel x peripheral address register
    CMAR*: uint32              ## !< DMA channel x memory address register

  DMA_TypeDef* {.bycopy.} = object
    ISR*: uint32               ## !< DMA interrupt status register,               Address offset: 0x00
    IFCR*: uint32              ## !< DMA interrupt flag clear register,           Address offset: 0x04


## *
##  @brief External Interrupt/Event Controller
##

type
  EXTI_TypeDef* {.bycopy.} = object
    IMR*: uint32               ## !<EXTI Interrupt mask register,                 Address offset: 0x00
    EMR*: uint32               ## !<EXTI Event mask register,                     Address offset: 0x04
    RTSR*: uint32              ## !<EXTI Rising trigger selection register ,      Address offset: 0x08
    FTSR*: uint32              ## !<EXTI Falling trigger selection register,      Address offset: 0x0C
    SWIER*: uint32             ## !<EXTI Software interrupt event register,       Address offset: 0x10
    PR*: uint32                ## !<EXTI Pending register,                        Address offset: 0x14


## *
##  @brief FLASH Registers
##

type
  FLASH_TypeDef* {.bycopy.} = object
    ACR*: uint32               ## !< Access control register,                     Address offset: 0x00
    PECR*: uint32              ## !< Program/erase control register,              Address offset: 0x04
    PDKEYR*: uint32            ## !< Power down key register,                     Address offset: 0x08
    PEKEYR*: uint32            ## !< Program/erase key register,                  Address offset: 0x0c
    PRGKEYR*: uint32           ## !< Program memory key register,                 Address offset: 0x10
    OPTKEYR*: uint32           ## !< Option byte key register,                    Address offset: 0x14
    SR*: uint32                ## !< Status register,                             Address offset: 0x18
    OBR*: uint32               ## !< Option byte register,                        Address offset: 0x1c
    WRPR1*: uint32             ## !< Write protection register 1,                 Address offset: 0x20
    RESERVED*: array[23, uint32] ## !< Reserved,                                    Address offset: 0x24
    WRPR2*: uint32             ## !< Write protection register 2,                 Address offset: 0x80
    WRPR3*: uint32             ## !< Write protection register 3,                 Address offset: 0x84
    WRPR4*: uint32             ## !< Write protection register 4,                 Address offset: 0x88


## *
##  @brief Option Bytes Registers
##

type
  OB_TypeDef* {.bycopy.} = object
    RDP*: uint32               ## !< Read protection register,               Address offset: 0x00
    USER*: uint32              ## !< user register,                          Address offset: 0x04
    WRP01*: uint32             ## !< write protection register 0 1,          Address offset: 0x08
    WRP23*: uint32             ## !< write protection register 2 3,          Address offset: 0x0C
    WRP45*: uint32             ## !< write protection register 4 5,          Address offset: 0x10
    WRP67*: uint32             ## !< write protection register 6 7,          Address offset: 0x14
    WRP89*: uint32             ## !< write protection register 8 9,          Address offset: 0x18
    WRP1011*: uint32           ## !< write protection register 10 11,        Address offset: 0x1C
    RESERVED*: array[24, uint32] ## !< Reserved,                                    0x20 -> 0x7C
    WRP1213*: uint32           ## !< write protection register 12 13,        Address offset: 0x80
    WRP1415*: uint32           ## !< write protection register 14 15,        Address offset: 0x84


## *
##  @brief Operational Amplifier (OPAMP)
##

type
  OPAMP_TypeDef* {.bycopy.} = object
    CSR*: uint32               ## !< OPAMP control and status register,                 Address offset: 0x00
    OTR*: uint32               ## !< OPAMP offset trimming register for normal mode,    Address offset: 0x04
    LPOTR*: uint32             ## !< OPAMP offset trimming register for low power mode, Address offset: 0x08

  OPAMP_Common_TypeDef* {.bycopy.} = object
    CSR*: uint32               ## !< OPAMP control and status register, used for bits common to several OPAMP instances,              Address offset: 0x00
    OTR*: uint32               ## !< OPAMP offset trimming register for normal mode, used for bits common to several OPAMP instances, Address offset: 0x04


## *
##  @brief General Purpose IO
##

type
  GPIO_TypeDef* {.bycopy.} = object
    MODER*: uint32             ## !< GPIO port mode register,                     Address offset: 0x00
    OTYPER*: uint32            ## !< GPIO port output type register,              Address offset: 0x04
    OSPEEDR*: uint32           ## !< GPIO port output speed register,             Address offset: 0x08
    PUPDR*: uint32             ## !< GPIO port pull-up/pull-down register,        Address offset: 0x0C
    IDR*: uint32               ## !< GPIO port input data register,               Address offset: 0x10
    ODR*: uint32               ## !< GPIO port output data register,              Address offset: 0x14
    BSRR*: uint32              ## !< GPIO port bit set/reset registerBSRR,        Address offset: 0x18
    LCKR*: uint32              ## !< GPIO port configuration lock register,       Address offset: 0x1C
    AFR*: array[2, uint32]      ## !< GPIO alternate function register,            Address offset: 0x20-0x24
    BRR*: uint32               ## !< GPIO bit reset register,                     Address offset: 0x28


## *
##  @brief SysTem Configuration
##

type
  SYSCFG_TypeDef* {.bycopy.} = object
    MEMRMP*: uint32            ## !< SYSCFG memory remap register,                      Address offset: 0x00
    PMC*: uint32               ## !< SYSCFG peripheral mode configuration register,     Address offset: 0x04
    EXTICR*: array[4, uint32]   ## !< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14


## *
##  @brief Inter-integrated Circuit Interface
##

type
  I2C_TypeDef* {.bycopy.} = object
    CR1*: uint32               ## !< I2C Control register 1,                      Address offset: 0x00
    CR2*: uint32               ## !< I2C Control register 2,                      Address offset: 0x04
    OAR1*: uint32              ## !< I2C Own address register 1,                  Address offset: 0x08
    OAR2*: uint32              ## !< I2C Own address register 2,                  Address offset: 0x0C
    DR*: uint32                ## !< I2C Data register,                           Address offset: 0x10
    SR1*: uint32               ## !< I2C Status register 1,                       Address offset: 0x14
    SR2*: uint32               ## !< I2C Status register 2,                       Address offset: 0x18
    CCR*: uint32               ## !< I2C Clock control register,                  Address offset: 0x1C
    TRISE*: uint32             ## !< I2C TRISE register,                          Address offset: 0x20


## *
##  @brief Independent WATCHDOG
##

type
  IWDG_TypeDef* {.bycopy.} = object
    KR*: uint32                ## !< Key register,                                Address offset: 0x00
    PR*: uint32                ## !< Prescaler register,                          Address offset: 0x04
    RLR*: uint32               ## !< Reload register,                             Address offset: 0x08
    SR*: uint32                ## !< Status register,                             Address offset: 0x0C


## *
##  @brief LCD
##

type
  LCD_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< LCD control register,                           Address offset: 0x00
    FCR*: uint32               ## !< LCD frame control register,                     Address offset: 0x04
    SR*: uint32                ## !< LCD status register,                            Address offset: 0x08
    CLR*: uint32               ## !< LCD clear register,                             Address offset: 0x0C
    RESERVED*: uint32          ## !< Reserved,                                       Address offset: 0x10
    RAM*: array[16, uint32]     ## !< LCD display memory,                             Address offset: 0x14-0x50


## *
##  @brief Power Control
##

type
  PWR_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< PWR power control register,                          Address offset: 0x00
    CSR*: uint32               ## !< PWR power control/status register,                   Address offset: 0x04


## *
##  @brief Reset and Clock Control
##

type
  RCC_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< RCC clock control register,                                   Address offset: 0x00
    ICSCR*: uint32             ## !< RCC Internal clock sources calibration register,              Address offset: 0x04
    CFGR*: uint32              ## !< RCC Clock configuration register,                             Address offset: 0x08
    CIR*: uint32               ## !< RCC Clock interrupt register,                                 Address offset: 0x0C
    AHBRSTR*: uint32           ## !< RCC AHB peripheral reset register,                            Address offset: 0x10
    APB2RSTR*: uint32          ## !< RCC APB2 peripheral reset register,                           Address offset: 0x14
    APB1RSTR*: uint32          ## !< RCC APB1 peripheral reset register,                           Address offset: 0x18
    AHBENR*: uint32            ## !< RCC AHB peripheral clock enable register,                     Address offset: 0x1C
    APB2ENR*: uint32           ## !< RCC APB2 peripheral clock enable register,                    Address offset: 0x20
    APB1ENR*: uint32           ## !< RCC APB1 peripheral clock enable register,                    Address offset: 0x24
    AHBLPENR*: uint32          ## !< RCC AHB peripheral clock enable in low power mode register,   Address offset: 0x28
    APB2LPENR*: uint32         ## !< RCC APB2 peripheral clock enable in low power mode register,  Address offset: 0x2C
    APB1LPENR*: uint32         ## !< RCC APB1 peripheral clock enable in low power mode register,  Address offset: 0x30
    CSR*: uint32               ## !< RCC Control/status register,                                  Address offset: 0x34


## *
##  @brief Routing Interface
##

type
  RI_TypeDef* {.bycopy.} = object
    ICR*: uint32               ## !< RI input capture register,                     Address offset: 0x00
    ASCR1*: uint32             ## !< RI analog switches control register,           Address offset: 0x04
    ASCR2*: uint32             ## !< RI analog switch control register 2,           Address offset: 0x08
    HYSCR1*: uint32            ## !< RI hysteresis control register,                Address offset: 0x0C
    HYSCR2*: uint32            ## !< RI Hysteresis control register,                Address offset: 0x10
    HYSCR3*: uint32            ## !< RI Hysteresis control register,                Address offset: 0x14
    HYSCR4*: uint32            ## !< RI Hysteresis control register,                Address offset: 0x18
    ASMR1*: uint32             ## !< RI Analog switch mode register 1,              Address offset: 0x1C
    CMR1*: uint32              ## !< RI Channel mask register 1,                    Address offset: 0x20
    CICR1*: uint32             ## !< RI Channel Iden for capture register 1,        Address offset: 0x24
    ASMR2*: uint32             ## !< RI Analog switch mode register 2,              Address offset: 0x28
    CMR2*: uint32              ## !< RI Channel mask register 2,                    Address offset: 0x2C
    CICR2*: uint32             ## !< RI Channel Iden for capture register 2,        Address offset: 0x30
    ASMR3*: uint32             ## !< RI Analog switch mode register 3,              Address offset: 0x34
    CMR3*: uint32              ## !< RI Channel mask register 3,                    Address offset: 0x38
    CICR3*: uint32             ## !< RI Channel Iden for capture register 3,        Address offset: 0x3C
    ASMR4*: uint32             ## !< RI Analog switch mode register 4,              Address offset: 0x40
    CMR4*: uint32              ## !< RI Channel mask register 4,                    Address offset: 0x44
    CICR4*: uint32             ## !< RI Channel Iden for capture register 4,        Address offset: 0x48
    ASMR5*: uint32             ## !< RI Analog switch mode register 5,              Address offset: 0x4C
    CMR5*: uint32              ## !< RI Channel mask register 5,                    Address offset: 0x50
    CICR5*: uint32             ## !< RI Channel Iden for capture register 5,        Address offset: 0x54


## *
##  @brief Real-Time Clock
##

type
  RTC_TypeDef* {.bycopy.} = object
    TR*: uint32                ## !< RTC time register,                                         Address offset: 0x00
    DR*: uint32                ## !< RTC date register,                                         Address offset: 0x04
    CR*: uint32                ## !< RTC control register,                                      Address offset: 0x08
    ISR*: uint32               ## !< RTC initialization and status register,                    Address offset: 0x0C
    PRER*: uint32              ## !< RTC prescaler register,                                    Address offset: 0x10
    WUTR*: uint32              ## !< RTC wakeup timer register,                                 Address offset: 0x14
    CALIBR*: uint32            ## !< RTC calibration register,                                  Address offset: 0x18
    ALRMAR*: uint32            ## !< RTC alarm A register,                                      Address offset: 0x1C
    ALRMBR*: uint32            ## !< RTC alarm B register,                                      Address offset: 0x20
    WPR*: uint32               ## !< RTC write protection register,                             Address offset: 0x24
    SSR*: uint32               ## !< RTC sub second register,                                   Address offset: 0x28
    SHIFTR*: uint32            ## !< RTC shift control register,                                Address offset: 0x2C
    TSTR*: uint32              ## !< RTC time stamp time register,                              Address offset: 0x30
    TSDR*: uint32              ## !< RTC time stamp date register,                              Address offset: 0x34
    TSSSR*: uint32             ## !< RTC time-stamp sub second register,                        Address offset: 0x38
    CALR*: uint32              ## !< RRTC calibration register,                                 Address offset: 0x3C
    TAFCR*: uint32             ## !< RTC tamper and alternate function configuration register,  Address offset: 0x40
    ALRMASSR*: uint32          ## !< RTC alarm A sub second register,                           Address offset: 0x44
    ALRMBSSR*: uint32          ## !< RTC alarm B sub second register,                           Address offset: 0x48
    RESERVED7*: uint32         ## !< Reserved, 0x4C
    BKP0R*: uint32             ## !< RTC backup register 0,                                     Address offset: 0x50
    BKP1R*: uint32             ## !< RTC backup register 1,                                     Address offset: 0x54
    BKP2R*: uint32             ## !< RTC backup register 2,                                     Address offset: 0x58
    BKP3R*: uint32             ## !< RTC backup register 3,                                     Address offset: 0x5C
    BKP4R*: uint32             ## !< RTC backup register 4,                                     Address offset: 0x60
    BKP5R*: uint32             ## !< RTC backup register 5,                                     Address offset: 0x64
    BKP6R*: uint32             ## !< RTC backup register 6,                                     Address offset: 0x68
    BKP7R*: uint32             ## !< RTC backup register 7,                                     Address offset: 0x6C
    BKP8R*: uint32             ## !< RTC backup register 8,                                     Address offset: 0x70
    BKP9R*: uint32             ## !< RTC backup register 9,                                     Address offset: 0x74
    BKP10R*: uint32            ## !< RTC backup register 10,                                    Address offset: 0x78
    BKP11R*: uint32            ## !< RTC backup register 11,                                    Address offset: 0x7C
    BKP12R*: uint32            ## !< RTC backup register 12,                                    Address offset: 0x80
    BKP13R*: uint32            ## !< RTC backup register 13,                                    Address offset: 0x84
    BKP14R*: uint32            ## !< RTC backup register 14,                                    Address offset: 0x88
    BKP15R*: uint32            ## !< RTC backup register 15,                                    Address offset: 0x8C
    BKP16R*: uint32            ## !< RTC backup register 16,                                    Address offset: 0x90
    BKP17R*: uint32            ## !< RTC backup register 17,                                    Address offset: 0x94
    BKP18R*: uint32            ## !< RTC backup register 18,                                    Address offset: 0x98
    BKP19R*: uint32            ## !< RTC backup register 19,                                    Address offset: 0x9C
    BKP20R*: uint32            ## !< RTC backup register 20,                                    Address offset: 0xA0
    BKP21R*: uint32            ## !< RTC backup register 21,                                    Address offset: 0xA4
    BKP22R*: uint32            ## !< RTC backup register 22,                                    Address offset: 0xA8
    BKP23R*: uint32            ## !< RTC backup register 23,                                    Address offset: 0xAC
    BKP24R*: uint32            ## !< RTC backup register 24,                                    Address offset: 0xB0
    BKP25R*: uint32            ## !< RTC backup register 25,                                    Address offset: 0xB4
    BKP26R*: uint32            ## !< RTC backup register 26,                                    Address offset: 0xB8
    BKP27R*: uint32            ## !< RTC backup register 27,                                    Address offset: 0xBC
    BKP28R*: uint32            ## !< RTC backup register 28,                                    Address offset: 0xC0
    BKP29R*: uint32            ## !< RTC backup register 29,                                    Address offset: 0xC4
    BKP30R*: uint32            ## !< RTC backup register 30,                                    Address offset: 0xC8
    BKP31R*: uint32            ## !< RTC backup register 31,                                    Address offset: 0xCC


## *
##  @brief Serial Peripheral Interface
##

type
  SPI_TypeDef* {.bycopy.} = object
    CR1*: uint32               ## !< SPI Control register 1 (not used in I2S mode),      Address offset: 0x00
    CR2*: uint32               ## !< SPI Control register 2,                             Address offset: 0x04
    SR*: uint32                ## !< SPI Status register,                                Address offset: 0x08
    DR*: uint32                ## !< SPI data register,                                  Address offset: 0x0C
    CRCPR*: uint32             ## !< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10
    RXCRCR*: uint32            ## !< SPI Rx CRC register (not used in I2S mode),         Address offset: 0x14
    TXCRCR*: uint32            ## !< SPI Tx CRC register (not used in I2S mode),         Address offset: 0x18
    I2SCFGR*: uint32           ## !< SPI_I2S configuration register,                     Address offset: 0x1C
    I2SPR*: uint32             ## !< SPI_I2S prescaler register,                         Address offset: 0x20


## *
##  @brief TIM
##

type
  TIM_TypeDef* {.bycopy.} = object
    CR1*: uint32               ## !< TIM control register 1,              Address offset: 0x00
    CR2*: uint32               ## !< TIM control register 2,              Address offset: 0x04
    SMCR*: uint32              ## !< TIM slave Mode Control register,     Address offset: 0x08
    DIER*: uint32              ## !< TIM DMA/interrupt enable register,   Address offset: 0x0C
    SR*: uint32                ## !< TIM status register,                 Address offset: 0x10
    EGR*: uint32               ## !< TIM event generation register,       Address offset: 0x14
    CCMR1*: uint32             ## !< TIM capture/compare mode register 1, Address offset: 0x18
    CCMR2*: uint32             ## !< TIM capture/compare mode register 2, Address offset: 0x1C
    CCER*: uint32              ## !< TIM capture/compare enable register, Address offset: 0x20
    CNT*: uint32               ## !< TIM counter register,                Address offset: 0x24
    PSC*: uint32               ## !< TIM prescaler register,              Address offset: 0x28
    ARR*: uint32               ## !< TIM auto-reload register,            Address offset: 0x2C
    RESERVED12*: uint32        ## !< Reserved, 0x30
    CCR1*: uint32              ## !< TIM capture/compare register 1,      Address offset: 0x34
    CCR2*: uint32              ## !< TIM capture/compare register 2,      Address offset: 0x38
    CCR3*: uint32              ## !< TIM capture/compare register 3,      Address offset: 0x3C
    CCR4*: uint32              ## !< TIM capture/compare register 4,      Address offset: 0x40
    RESERVED17*: uint32        ## !< Reserved, 0x44
    DCR*: uint32               ## !< TIM DMA control register,            Address offset: 0x48
    DMAR*: uint32              ## !< TIM DMA address for full transfer,   Address offset: 0x4C
    OR*: uint32                ## !< TIM option register,                 Address offset: 0x50


## *
##  @brief Universal Synchronous Asynchronous Receiver Transmitter
##

type
  USART_TypeDef* {.bycopy.} = object
    SR*: uint32                ## !< USART Status register,                   Address offset: 0x00
    DR*: uint32                ## !< USART Data register,                     Address offset: 0x04
    BRR*: uint32               ## !< USART Baud rate register,                Address offset: 0x08
    CR1*: uint32               ## !< USART Control register 1,                Address offset: 0x0C
    CR2*: uint32               ## !< USART Control register 2,                Address offset: 0x10
    CR3*: uint32               ## !< USART Control register 3,                Address offset: 0x14
    GTPR*: uint32              ## !< USART Guard time and prescaler register, Address offset: 0x18


## *
##  @brief Universal Serial Bus Full Speed Device
##

type
  USB_TypeDef* {.bycopy.} = object
    EP0R*: uint16              ## !< USB Endpoint 0 register,                Address offset: 0x00
    RESERVED0*: uint16         ## !< Reserved
    EP1R*: uint16              ## !< USB Endpoint 1 register,                Address offset: 0x04
    RESERVED1*: uint16         ## !< Reserved
    EP2R*: uint16              ## !< USB Endpoint 2 register,                Address offset: 0x08
    RESERVED2*: uint16         ## !< Reserved
    EP3R*: uint16              ## !< USB Endpoint 3 register,                Address offset: 0x0C
    RESERVED3*: uint16         ## !< Reserved
    EP4R*: uint16              ## !< USB Endpoint 4 register,                Address offset: 0x10
    RESERVED4*: uint16         ## !< Reserved
    EP5R*: uint16              ## !< USB Endpoint 5 register,                Address offset: 0x14
    RESERVED5*: uint16         ## !< Reserved
    EP6R*: uint16              ## !< USB Endpoint 6 register,                Address offset: 0x18
    RESERVED6*: uint16         ## !< Reserved
    EP7R*: uint16              ## !< USB Endpoint 7 register,                Address offset: 0x1C
    RESERVED7*: array[17, uint16] ## !< Reserved
    CNTR*: uint16              ## !< Control register,                       Address offset: 0x40
    RESERVED8*: uint16         ## !< Reserved
    ISTR*: uint16              ## !< Interrupt status register,              Address offset: 0x44
    RESERVED9*: uint16         ## !< Reserved
    FNR*: uint16               ## !< Frame number register,                  Address offset: 0x48
    RESERVEDA*: uint16         ## !< Reserved
    DADDR*: uint16             ## !< Device address register,                Address offset: 0x4C
    RESERVEDB*: uint16         ## !< Reserved
    BTABLE*: uint16            ## !< Buffer Table address register,          Address offset: 0x50
    RESERVEDC*: uint16         ## !< Reserved


## *
##  @brief Window WATCHDOG
##

type
  WWDG_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< WWDG Control register,       Address offset: 0x00
    CFR*: uint32               ## !< WWDG Configuration register, Address offset: 0x04
    SR*: uint32                ## !< WWDG Status register,        Address offset: 0x08


## *
##  @brief Universal Serial Bus Full Speed Device
##
## *
##  @}
##
## * @addtogroup Peripheral_memory_map
##  @{
##

const
  FLASH_BASE* = (cast[uint32](0x08000000)) ## !< FLASH base address in the alias region
  FLASH_EEPROM_BASE* = ((uint32)(FLASH_BASE + 0x00080000)) ## !< FLASH EEPROM base address in the alias region
  SRAM_BASE* = (cast[uint32](0x20000000)) ## !< SRAM base address in the alias region
  PERIPH_BASE* = (cast[uint32](0x40000000)) ## !< Peripheral base address in the alias region
  SRAM_BB_BASE* = (cast[uint32](0x22000000)) ## !< SRAM base address in the bit-band region
  PERIPH_BB_BASE* = (cast[uint32](0x42000000)) ## !< Peripheral base address in the bit-band region
  FLASH_BANK2_BASE* = (cast[uint32](0x08040000)) ## !< FLASH BANK2 base address in the alias region
  FLASH_BANK1_END* = (cast[uint32](0x0803FFFF)) ## !< Program end FLASH BANK1 address
  FLASH_BANK2_END* = (cast[uint32](0x0807FFFF)) ## !< Program end FLASH BANK2 address
  FLASH_EEPROM_END* = (cast[uint32](0x08083FFF)) ## !< FLASH EEPROM end address (16KB)

## !< Peripheral memory map

const
  APB1PERIPH_BASE* = PERIPH_BASE
  APB2PERIPH_BASE* = (PERIPH_BASE + 0x00010000)
  AHBPERIPH_BASE* = (PERIPH_BASE + 0x00020000)

## !< APB1 peripherals

const
  TIM2_BASE* = (APB1PERIPH_BASE + 0x00000000)
  TIM3_BASE* = (APB1PERIPH_BASE + 0x00000400)
  TIM4_BASE* = (APB1PERIPH_BASE + 0x00000800)
  TIM5_BASE* = (APB1PERIPH_BASE + 0x00000C00)
  TIM6_BASE* = (APB1PERIPH_BASE + 0x00001000)
  TIM7_BASE* = (APB1PERIPH_BASE + 0x00001400)
  LCD_BASE* = (APB1PERIPH_BASE + 0x00002400)
  RTC_BASE* = (APB1PERIPH_BASE + 0x00002800)
  WWDG_BASE* = (APB1PERIPH_BASE + 0x00002C00)
  IWDG_BASE* = (APB1PERIPH_BASE + 0x00003000)
  SPI2_BASE* = (APB1PERIPH_BASE + 0x00003800)
  SPI3_BASE* = (APB1PERIPH_BASE + 0x00003C00)
  USART2_BASE* = (APB1PERIPH_BASE + 0x00004400)
  USART3_BASE* = (APB1PERIPH_BASE + 0x00004800)
  UART4_BASE* = (APB1PERIPH_BASE + 0x00004C00)
  UART5_BASE* = (APB1PERIPH_BASE + 0x00005000)
  I2C1_BASE* = (APB1PERIPH_BASE + 0x00005400)
  I2C2_BASE* = (APB1PERIPH_BASE + 0x00005800)

##  USB device FS

const
  USB_BASE* = (APB1PERIPH_BASE + 0x00005C00) ## !< USB_IP Peripheral Registers base address
  USB_PMAADDR* = (APB1PERIPH_BASE + 0x00006000) ## !< USB_IP Packet Memory Area base address

##  USB device FS SRAM

const
  PWR_BASE* = (APB1PERIPH_BASE + 0x00007000)
  DAC_BASE* = (APB1PERIPH_BASE + 0x00007400)
  COMP_BASE* = (APB1PERIPH_BASE + 0x00007C00)
  RI_BASE* = (APB1PERIPH_BASE + 0x00007C04)
  OPAMP_BASE* = (APB1PERIPH_BASE + 0x00007C5C)

## !< APB2 peripherals

const
  SYSCFG_BASE* = (APB2PERIPH_BASE + 0x00000000)
  EXTI_BASE* = (APB2PERIPH_BASE + 0x00000400)
  TIM9_BASE* = (APB2PERIPH_BASE + 0x00000800)
  TIM10_BASE* = (APB2PERIPH_BASE + 0x00000C00)
  TIM11_BASE* = (APB2PERIPH_BASE + 0x00001000)
  ADC1_BASE* = (APB2PERIPH_BASE + 0x00002400)
  ADC_BASE* = (APB2PERIPH_BASE + 0x00002700)
  SPI1_BASE* = (APB2PERIPH_BASE + 0x00003000)
  USART1_BASE* = (APB2PERIPH_BASE + 0x00003800)

## !< AHB peripherals

const
  GPIOA_BASE* = (AHBPERIPH_BASE + 0x00000000)
  GPIOB_BASE* = (AHBPERIPH_BASE + 0x00000400)
  GPIOC_BASE* = (AHBPERIPH_BASE + 0x00000800)
  GPIOD_BASE* = (AHBPERIPH_BASE + 0x00000C00)
  GPIOE_BASE* = (AHBPERIPH_BASE + 0x00001000)
  GPIOH_BASE* = (AHBPERIPH_BASE + 0x00001400)
  GPIOF_BASE* = (AHBPERIPH_BASE + 0x00001800)
  GPIOG_BASE* = (AHBPERIPH_BASE + 0x00001C00)
  CRC_BASE* = (AHBPERIPH_BASE + 0x00003000)
  RCC_BASE* = (AHBPERIPH_BASE + 0x00003800)
  FLASH_R_BASE* = (AHBPERIPH_BASE + 0x00003C00) ## !< FLASH registers base address
  OB_BASE* = (cast[uint32](0x1FF80000)) ## !< FLASH Option Bytes base address
  FLASHSIZE_BASE* = (cast[uint32](0x1FF800CC)) ## !< FLASH Size register base address for Cat.3, Cat.4, Cat.5 and Cat.6 devices
  UID_BASE* = (cast[uint32](0x1FF800D0)) ## !< Unique device ID register base address for Cat.3, Cat.4, Cat.5 and Cat.6 devices
  DMA1_BASE* = (AHBPERIPH_BASE + 0x00006000)
  DMA1_Channel1_BASE* = (DMA1_BASE + 0x00000008)
  DMA1_Channel2_BASE* = (DMA1_BASE + 0x0000001C)
  DMA1_Channel3_BASE* = (DMA1_BASE + 0x00000030)
  DMA1_Channel4_BASE* = (DMA1_BASE + 0x00000044)
  DMA1_Channel5_BASE* = (DMA1_BASE + 0x00000058)
  DMA1_Channel6_BASE* = (DMA1_BASE + 0x0000006C)
  DMA1_Channel7_BASE* = (DMA1_BASE + 0x00000080)
  DMA2_BASE* = (AHBPERIPH_BASE + 0x00006400)
  DMA2_Channel1_BASE* = (DMA2_BASE + 0x00000008)
  DMA2_Channel2_BASE* = (DMA2_BASE + 0x0000001C)
  DMA2_Channel3_BASE* = (DMA2_BASE + 0x00000030)
  DMA2_Channel4_BASE* = (DMA2_BASE + 0x00000044)
  DMA2_Channel5_BASE* = (DMA2_BASE + 0x00000058)
  DBGMCU_BASE* = 0xE0042000'u32 ## !< Debug MCU registers base address

## *
##  @}
##
## * @addtogroup Peripheral_declaration
##  @{
##

const
  TIM2* = (cast[ptr TIM_TypeDef](TIM2_BASE))
  TIM3* = (cast[ptr TIM_TypeDef](TIM3_BASE))
  TIM4* = (cast[ptr TIM_TypeDef](TIM4_BASE))
  TIM5* = (cast[ptr TIM_TypeDef](TIM5_BASE))
  TIM6* = (cast[ptr TIM_TypeDef](TIM6_BASE))
  TIM7* = (cast[ptr TIM_TypeDef](TIM7_BASE))
  LCD* = (cast[ptr LCD_TypeDef](LCD_BASE))
  RTC* = (cast[ptr RTC_TypeDef](RTC_BASE))
  WWDG* = (cast[ptr WWDG_TypeDef](WWDG_BASE))
  IWDG* = (cast[ptr IWDG_TypeDef](IWDG_BASE))
  SPI2* = (cast[ptr SPI_TypeDef](SPI2_BASE))
  SPI3* = (cast[ptr SPI_TypeDef](SPI3_BASE))
  USART2* = (cast[ptr USART_TypeDef](USART2_BASE))
  USART3* = (cast[ptr USART_TypeDef](USART3_BASE))
  UART4* = (cast[ptr USART_TypeDef](UART4_BASE))
  UART5* = (cast[ptr USART_TypeDef](UART5_BASE))
  I2C1* = (cast[ptr I2C_TypeDef](I2C1_BASE))
  I2C2* = (cast[ptr I2C_TypeDef](I2C2_BASE))

##  USB device FS

const
  USB* = (cast[ptr USB_TypeDef](USB_BASE))

##  USB device FS SRAM

const
  PWR* = (cast[ptr PWR_TypeDef](PWR_BASE))
  DAC1* = (cast[ptr DAC_TypeDef](DAC_BASE))

##  Legacy define

const
  DAC* = DAC1
  COMP* = (cast[ptr COMP_TypeDef](COMP_BASE)) ##  COMP generic instance include bits of COMP1 and COMP2 mixed in the same register
  COMP1* = (cast[ptr COMP_TypeDef](COMP_BASE)) ##  COMP1 instance definition to differentiate COMP1 and COMP2, not to be used to access comparator register
  COMP2* = (cast[ptr COMP_TypeDef]((COMP_BASE + 0x00000001))) ##  COMP2 instance definition to differentiate COMP1 and COMP2, not to be used to access comparator register
  COMP12_COMMON* = (cast[ptr COMP_Common_TypeDef](COMP_BASE)) ##  COMP common instance definition to access comparator register bits used by both comparator instances (window mode)
  RI* = (cast[ptr RI_TypeDef](RI_BASE))
  OPAMP* = (cast[ptr OPAMP_TypeDef](OPAMP_BASE))
  OPAMP1* = (cast[ptr OPAMP_TypeDef](OPAMP_BASE))
  OPAMP2* = (cast[ptr OPAMP_TypeDef]((OPAMP_BASE + 0x00000001)))
  OPAMP12_COMMON* = (cast[ptr OPAMP_Common_TypeDef](OPAMP_BASE))
  SYSCFG* = (cast[ptr SYSCFG_TypeDef](SYSCFG_BASE))
  EXTI* = (cast[ptr EXTI_TypeDef](EXTI_BASE))
  TIM9* = (cast[ptr TIM_TypeDef](TIM9_BASE))
  TIM10* = (cast[ptr TIM_TypeDef](TIM10_BASE))
  TIM11* = (cast[ptr TIM_TypeDef](TIM11_BASE))
  ADC1* = (cast[ptr ADC_TypeDef](ADC1_BASE))
  ADC1_COMMON* = (cast[ptr ADC_Common_TypeDef](ADC_BASE))

##  Legacy defines

const
  ADC* = ADC1_COMMON
  SPI1* = (cast[ptr SPI_TypeDef](SPI1_BASE))
  USART1* = (cast[ptr USART_TypeDef](USART1_BASE))
  GPIOA* = (cast[ptr GPIO_TypeDef](GPIOA_BASE))
  GPIOB* = (cast[ptr GPIO_TypeDef](GPIOB_BASE))
  GPIOC* = (cast[ptr GPIO_TypeDef](GPIOC_BASE))
  GPIOD* = (cast[ptr GPIO_TypeDef](GPIOD_BASE))
  GPIOE* = (cast[ptr GPIO_TypeDef](GPIOE_BASE))
  GPIOH* = (cast[ptr GPIO_TypeDef](GPIOH_BASE))
  GPIOF* = (cast[ptr GPIO_TypeDef](GPIOF_BASE))
  GPIOG* = (cast[ptr GPIO_TypeDef](GPIOG_BASE))
  CRC* = (cast[ptr CRC_TypeDef](CRC_BASE))
  RCC* = (cast[ptr RCC_TypeDef](RCC_BASE))
  FLASH* = (cast[ptr FLASH_TypeDef](FLASH_R_BASE))
  OB* = (cast[ptr OB_TypeDef](OB_BASE))
  DMA1* = (cast[ptr DMA_TypeDef](DMA1_BASE))
  DMA1_Channel1* = (cast[ptr DMA_Channel_TypeDef](DMA1_Channel1_BASE))
  DMA1_Channel2* = (cast[ptr DMA_Channel_TypeDef](DMA1_Channel2_BASE))
  DMA1_Channel3* = (cast[ptr DMA_Channel_TypeDef](DMA1_Channel3_BASE))
  DMA1_Channel4* = (cast[ptr DMA_Channel_TypeDef](DMA1_Channel4_BASE))
  DMA1_Channel5* = (cast[ptr DMA_Channel_TypeDef](DMA1_Channel5_BASE))
  DMA1_Channel6* = (cast[ptr DMA_Channel_TypeDef](DMA1_Channel6_BASE))
  DMA1_Channel7* = (cast[ptr DMA_Channel_TypeDef](DMA1_Channel7_BASE))
  DMA2* = (cast[ptr DMA_TypeDef](DMA2_BASE))
  DMA2_Channel1* = (cast[ptr DMA_Channel_TypeDef](DMA2_Channel1_BASE))
  DMA2_Channel2* = (cast[ptr DMA_Channel_TypeDef](DMA2_Channel2_BASE))
  DMA2_Channel3* = (cast[ptr DMA_Channel_TypeDef](DMA2_Channel3_BASE))
  DMA2_Channel4* = (cast[ptr DMA_Channel_TypeDef](DMA2_Channel4_BASE))
  DMA2_Channel5* = (cast[ptr DMA_Channel_TypeDef](DMA2_Channel5_BASE))
  DBGMCU* = (cast[ptr DBGMCU_TypeDef](DBGMCU_BASE))

## *
##  @}
##
## * @addtogroup Exported_constants
##  @{
##
## * @addtogroup Peripheral_Registers_Bits_Definition
##  @{
##
## ****************************************************************************
##                          Peripheral Registers Bits Definition
## ****************************************************************************
## ****************************************************************************
##
##                       Analog to Digital Converter (ADC)
##
## ****************************************************************************
## *******************  Bit definition for ADC_SR register  *******************

const
  ADC_SR_AWD_Pos* = (0)
  ADC_SR_AWD_Msk* = (0x00000001 shl ADC_SR_AWD_Pos) ## !< 0x00000001
  ADC_SR_AWD* = ADC_SR_AWD_Msk
  ADC_SR_EOCS_Pos* = (1)
  ADC_SR_EOCS_Msk* = (0x00000001 shl ADC_SR_EOCS_Pos) ## !< 0x00000002
  ADC_SR_EOCS* = ADC_SR_EOCS_Msk
  ADC_SR_JEOS_Pos* = (2)
  ADC_SR_JEOS_Msk* = (0x00000001 shl ADC_SR_JEOS_Pos) ## !< 0x00000004
  ADC_SR_JEOS* = ADC_SR_JEOS_Msk
  ADC_SR_JSTRT_Pos* = (3)
  ADC_SR_JSTRT_Msk* = (0x00000001 shl ADC_SR_JSTRT_Pos) ## !< 0x00000008
  ADC_SR_JSTRT* = ADC_SR_JSTRT_Msk
  ADC_SR_STRT_Pos* = (4)
  ADC_SR_STRT_Msk* = (0x00000001 shl ADC_SR_STRT_Pos) ## !< 0x00000010
  ADC_SR_STRT* = ADC_SR_STRT_Msk
  ADC_SR_OVR_Pos* = (5)
  ADC_SR_OVR_Msk* = (0x00000001 shl ADC_SR_OVR_Pos) ## !< 0x00000020
  ADC_SR_OVR* = ADC_SR_OVR_Msk
  ADC_SR_ADONS_Pos* = (6)
  ADC_SR_ADONS_Msk* = (0x00000001 shl ADC_SR_ADONS_Pos) ## !< 0x00000040
  ADC_SR_ADONS* = ADC_SR_ADONS_Msk
  ADC_SR_RCNR_Pos* = (8)
  ADC_SR_RCNR_Msk* = (0x00000001 shl ADC_SR_RCNR_Pos) ## !< 0x00000100
  ADC_SR_RCNR* = ADC_SR_RCNR_Msk
  ADC_SR_JCNR_Pos* = (9)
  ADC_SR_JCNR_Msk* = (0x00000001 shl ADC_SR_JCNR_Pos) ## !< 0x00000200
  ADC_SR_JCNR* = ADC_SR_JCNR_Msk

##  Legacy defines

const
  ADC_SR_EOC* = (ADC_SR_EOCS)
  ADC_SR_JEOC* = (ADC_SR_JEOS)

## ******************  Bit definition for ADC_CR1 register  *******************

const
  ADC_CR1_AWDCH_Pos* = (0)
  ADC_CR1_AWDCH_Msk* = (0x0000001F shl ADC_CR1_AWDCH_Pos) ## !< 0x0000001F
  ADC_CR1_AWDCH* = ADC_CR1_AWDCH_Msk
  ADC_CR1_AWDCH_0* = (0x00000001 shl ADC_CR1_AWDCH_Pos) ## !< 0x00000001
  ADC_CR1_AWDCH_1* = (0x00000002 shl ADC_CR1_AWDCH_Pos) ## !< 0x00000002
  ADC_CR1_AWDCH_2* = (0x00000004 shl ADC_CR1_AWDCH_Pos) ## !< 0x00000004
  ADC_CR1_AWDCH_3* = (0x00000008 shl ADC_CR1_AWDCH_Pos) ## !< 0x00000008
  ADC_CR1_AWDCH_4* = (0x00000010 shl ADC_CR1_AWDCH_Pos) ## !< 0x00000010
  ADC_CR1_EOCSIE_Pos* = (5)
  ADC_CR1_EOCSIE_Msk* = (0x00000001 shl ADC_CR1_EOCSIE_Pos) ## !< 0x00000020
  ADC_CR1_EOCSIE* = ADC_CR1_EOCSIE_Msk
  ADC_CR1_AWDIE_Pos* = (6)
  ADC_CR1_AWDIE_Msk* = (0x00000001 shl ADC_CR1_AWDIE_Pos) ## !< 0x00000040
  ADC_CR1_AWDIE* = ADC_CR1_AWDIE_Msk
  ADC_CR1_JEOSIE_Pos* = (7)
  ADC_CR1_JEOSIE_Msk* = (0x00000001 shl ADC_CR1_JEOSIE_Pos) ## !< 0x00000080
  ADC_CR1_JEOSIE* = ADC_CR1_JEOSIE_Msk
  ADC_CR1_SCAN_Pos* = (8)
  ADC_CR1_SCAN_Msk* = (0x00000001 shl ADC_CR1_SCAN_Pos) ## !< 0x00000100
  ADC_CR1_SCAN* = ADC_CR1_SCAN_Msk
  ADC_CR1_AWDSGL_Pos* = (9)
  ADC_CR1_AWDSGL_Msk* = (0x00000001 shl ADC_CR1_AWDSGL_Pos) ## !< 0x00000200
  ADC_CR1_AWDSGL* = ADC_CR1_AWDSGL_Msk
  ADC_CR1_JAUTO_Pos* = (10)
  ADC_CR1_JAUTO_Msk* = (0x00000001 shl ADC_CR1_JAUTO_Pos) ## !< 0x00000400
  ADC_CR1_JAUTO* = ADC_CR1_JAUTO_Msk
  ADC_CR1_DISCEN_Pos* = (11)
  ADC_CR1_DISCEN_Msk* = (0x00000001 shl ADC_CR1_DISCEN_Pos) ## !< 0x00000800
  ADC_CR1_DISCEN* = ADC_CR1_DISCEN_Msk
  ADC_CR1_JDISCEN_Pos* = (12)
  ADC_CR1_JDISCEN_Msk* = (0x00000001 shl ADC_CR1_JDISCEN_Pos) ## !< 0x00001000
  ADC_CR1_JDISCEN* = ADC_CR1_JDISCEN_Msk
  ADC_CR1_DISCNUM_Pos* = (13)
  ADC_CR1_DISCNUM_Msk* = (0x00000007 shl ADC_CR1_DISCNUM_Pos) ## !< 0x0000E000
  ADC_CR1_DISCNUM* = ADC_CR1_DISCNUM_Msk
  ADC_CR1_DISCNUM_0* = (0x00000001 shl ADC_CR1_DISCNUM_Pos) ## !< 0x00002000
  ADC_CR1_DISCNUM_1* = (0x00000002 shl ADC_CR1_DISCNUM_Pos) ## !< 0x00004000
  ADC_CR1_DISCNUM_2* = (0x00000004 shl ADC_CR1_DISCNUM_Pos) ## !< 0x00008000
  ADC_CR1_PDD_Pos* = (16)
  ADC_CR1_PDD_Msk* = (0x00000001 shl ADC_CR1_PDD_Pos) ## !< 0x00010000
  ADC_CR1_PDD* = ADC_CR1_PDD_Msk
  ADC_CR1_PDI_Pos* = (17)
  ADC_CR1_PDI_Msk* = (0x00000001 shl ADC_CR1_PDI_Pos) ## !< 0x00020000
  ADC_CR1_PDI* = ADC_CR1_PDI_Msk
  ADC_CR1_JAWDEN_Pos* = (22)
  ADC_CR1_JAWDEN_Msk* = (0x00000001 shl ADC_CR1_JAWDEN_Pos) ## !< 0x00400000
  ADC_CR1_JAWDEN* = ADC_CR1_JAWDEN_Msk
  ADC_CR1_AWDEN_Pos* = (23)
  ADC_CR1_AWDEN_Msk* = (0x00000001 shl ADC_CR1_AWDEN_Pos) ## !< 0x00800000
  ADC_CR1_AWDEN* = ADC_CR1_AWDEN_Msk
  ADC_CR1_RES_Pos* = (24)
  ADC_CR1_RES_Msk* = (0x00000003 shl ADC_CR1_RES_Pos) ## !< 0x03000000
  ADC_CR1_RES* = ADC_CR1_RES_Msk
  ADC_CR1_RES_0* = (0x00000001 shl ADC_CR1_RES_Pos) ## !< 0x01000000
  ADC_CR1_RES_1* = (0x00000002 shl ADC_CR1_RES_Pos) ## !< 0x02000000
  ADC_CR1_OVRIE_Pos* = (26)
  ADC_CR1_OVRIE_Msk* = (0x00000001 shl ADC_CR1_OVRIE_Pos) ## !< 0x04000000
  ADC_CR1_OVRIE* = ADC_CR1_OVRIE_Msk

##  Legacy defines

const
  ADC_CR1_EOCIE* = (ADC_CR1_EOCSIE)
  ADC_CR1_JEOCIE* = (ADC_CR1_JEOSIE)

## ******************  Bit definition for ADC_CR2 register  *******************

const
  ADC_CR2_ADON_Pos* = (0)
  ADC_CR2_ADON_Msk* = (0x00000001 shl ADC_CR2_ADON_Pos) ## !< 0x00000001
  ADC_CR2_ADON* = ADC_CR2_ADON_Msk
  ADC_CR2_CONT_Pos* = (1)
  ADC_CR2_CONT_Msk* = (0x00000001 shl ADC_CR2_CONT_Pos) ## !< 0x00000002
  ADC_CR2_CONT* = ADC_CR2_CONT_Msk
  ADC_CR2_CFG_Pos* = (2)
  ADC_CR2_CFG_Msk* = (0x00000001 shl ADC_CR2_CFG_Pos) ## !< 0x00000004
  ADC_CR2_CFG* = ADC_CR2_CFG_Msk
  ADC_CR2_DELS_Pos* = (4)
  ADC_CR2_DELS_Msk* = (0x00000007 shl ADC_CR2_DELS_Pos) ## !< 0x00000070
  ADC_CR2_DELS* = ADC_CR2_DELS_Msk
  ADC_CR2_DELS_0* = (0x00000001 shl ADC_CR2_DELS_Pos) ## !< 0x00000010
  ADC_CR2_DELS_1* = (0x00000002 shl ADC_CR2_DELS_Pos) ## !< 0x00000020
  ADC_CR2_DELS_2* = (0x00000004 shl ADC_CR2_DELS_Pos) ## !< 0x00000040
  ADC_CR2_DMA_Pos* = (8)
  ADC_CR2_DMA_Msk* = (0x00000001 shl ADC_CR2_DMA_Pos) ## !< 0x00000100
  ADC_CR2_DMA* = ADC_CR2_DMA_Msk
  ADC_CR2_DDS_Pos* = (9)
  ADC_CR2_DDS_Msk* = (0x00000001 shl ADC_CR2_DDS_Pos) ## !< 0x00000200
  ADC_CR2_DDS* = ADC_CR2_DDS_Msk
  ADC_CR2_EOCS_Pos* = (10)
  ADC_CR2_EOCS_Msk* = (0x00000001 shl ADC_CR2_EOCS_Pos) ## !< 0x00000400
  ADC_CR2_EOCS* = ADC_CR2_EOCS_Msk
  ADC_CR2_ALIGN_Pos* = (11)
  ADC_CR2_ALIGN_Msk* = (0x00000001 shl ADC_CR2_ALIGN_Pos) ## !< 0x00000800
  ADC_CR2_ALIGN* = ADC_CR2_ALIGN_Msk
  ADC_CR2_JEXTSEL_Pos* = (16)
  ADC_CR2_JEXTSEL_Msk* = (0x0000000F shl ADC_CR2_JEXTSEL_Pos) ## !< 0x000F0000
  ADC_CR2_JEXTSEL* = ADC_CR2_JEXTSEL_Msk
  ADC_CR2_JEXTSEL_0* = (0x00000001 shl ADC_CR2_JEXTSEL_Pos) ## !< 0x00010000
  ADC_CR2_JEXTSEL_1* = (0x00000002 shl ADC_CR2_JEXTSEL_Pos) ## !< 0x00020000
  ADC_CR2_JEXTSEL_2* = (0x00000004 shl ADC_CR2_JEXTSEL_Pos) ## !< 0x00040000
  ADC_CR2_JEXTSEL_3* = (0x00000008 shl ADC_CR2_JEXTSEL_Pos) ## !< 0x00080000
  ADC_CR2_JEXTEN_Pos* = (20)
  ADC_CR2_JEXTEN_Msk* = (0x00000003 shl ADC_CR2_JEXTEN_Pos) ## !< 0x00300000
  ADC_CR2_JEXTEN* = ADC_CR2_JEXTEN_Msk
  ADC_CR2_JEXTEN_0* = (0x00000001 shl ADC_CR2_JEXTEN_Pos) ## !< 0x00100000
  ADC_CR2_JEXTEN_1* = (0x00000002 shl ADC_CR2_JEXTEN_Pos) ## !< 0x00200000
  ADC_CR2_JSWSTART_Pos* = (22)
  ADC_CR2_JSWSTART_Msk* = (0x00000001 shl ADC_CR2_JSWSTART_Pos) ## !< 0x00400000
  ADC_CR2_JSWSTART* = ADC_CR2_JSWSTART_Msk
  ADC_CR2_EXTSEL_Pos* = (24)
  ADC_CR2_EXTSEL_Msk* = (0x0000000F shl ADC_CR2_EXTSEL_Pos) ## !< 0x0F000000
  ADC_CR2_EXTSEL* = ADC_CR2_EXTSEL_Msk
  ADC_CR2_EXTSEL_0* = (0x00000001 shl ADC_CR2_EXTSEL_Pos) ## !< 0x01000000
  ADC_CR2_EXTSEL_1* = (0x00000002 shl ADC_CR2_EXTSEL_Pos) ## !< 0x02000000
  ADC_CR2_EXTSEL_2* = (0x00000004 shl ADC_CR2_EXTSEL_Pos) ## !< 0x04000000
  ADC_CR2_EXTSEL_3* = (0x00000008 shl ADC_CR2_EXTSEL_Pos) ## !< 0x08000000
  ADC_CR2_EXTEN_Pos* = (28)
  ADC_CR2_EXTEN_Msk* = (0x00000003 shl ADC_CR2_EXTEN_Pos) ## !< 0x30000000
  ADC_CR2_EXTEN* = ADC_CR2_EXTEN_Msk
  ADC_CR2_EXTEN_0* = (0x00000001 shl ADC_CR2_EXTEN_Pos) ## !< 0x10000000
  ADC_CR2_EXTEN_1* = (0x00000002 shl ADC_CR2_EXTEN_Pos) ## !< 0x20000000
  ADC_CR2_SWSTART_Pos* = (30)
  ADC_CR2_SWSTART_Msk* = (0x00000001 shl ADC_CR2_SWSTART_Pos) ## !< 0x40000000
  ADC_CR2_SWSTART* = ADC_CR2_SWSTART_Msk

## *****************  Bit definition for ADC_SMPR1 register  ******************

const
  ADC_SMPR1_SMP20_Pos* = (0)
  ADC_SMPR1_SMP20_Msk* = (0x00000007 shl ADC_SMPR1_SMP20_Pos) ## !< 0x00000007
  ADC_SMPR1_SMP20* = ADC_SMPR1_SMP20_Msk
  ADC_SMPR1_SMP20_0* = (0x00000001 shl ADC_SMPR1_SMP20_Pos) ## !< 0x00000001
  ADC_SMPR1_SMP20_1* = (0x00000002 shl ADC_SMPR1_SMP20_Pos) ## !< 0x00000002
  ADC_SMPR1_SMP20_2* = (0x00000004 shl ADC_SMPR1_SMP20_Pos) ## !< 0x00000004
  ADC_SMPR1_SMP21_Pos* = (3)
  ADC_SMPR1_SMP21_Msk* = (0x00000007 shl ADC_SMPR1_SMP21_Pos) ## !< 0x00000038
  ADC_SMPR1_SMP21* = ADC_SMPR1_SMP21_Msk
  ADC_SMPR1_SMP21_0* = (0x00000001 shl ADC_SMPR1_SMP21_Pos) ## !< 0x00000008
  ADC_SMPR1_SMP21_1* = (0x00000002 shl ADC_SMPR1_SMP21_Pos) ## !< 0x00000010
  ADC_SMPR1_SMP21_2* = (0x00000004 shl ADC_SMPR1_SMP21_Pos) ## !< 0x00000020
  ADC_SMPR1_SMP22_Pos* = (6)
  ADC_SMPR1_SMP22_Msk* = (0x00000007 shl ADC_SMPR1_SMP22_Pos) ## !< 0x000001C0
  ADC_SMPR1_SMP22* = ADC_SMPR1_SMP22_Msk
  ADC_SMPR1_SMP22_0* = (0x00000001 shl ADC_SMPR1_SMP22_Pos) ## !< 0x00000040
  ADC_SMPR1_SMP22_1* = (0x00000002 shl ADC_SMPR1_SMP22_Pos) ## !< 0x00000080
  ADC_SMPR1_SMP22_2* = (0x00000004 shl ADC_SMPR1_SMP22_Pos) ## !< 0x00000100
  ADC_SMPR1_SMP23_Pos* = (9)
  ADC_SMPR1_SMP23_Msk* = (0x00000007 shl ADC_SMPR1_SMP23_Pos) ## !< 0x00000E00
  ADC_SMPR1_SMP23* = ADC_SMPR1_SMP23_Msk
  ADC_SMPR1_SMP23_0* = (0x00000001 shl ADC_SMPR1_SMP23_Pos) ## !< 0x00000200
  ADC_SMPR1_SMP23_1* = (0x00000002 shl ADC_SMPR1_SMP23_Pos) ## !< 0x00000400
  ADC_SMPR1_SMP23_2* = (0x00000004 shl ADC_SMPR1_SMP23_Pos) ## !< 0x00000800
  ADC_SMPR1_SMP24_Pos* = (12)
  ADC_SMPR1_SMP24_Msk* = (0x00000007 shl ADC_SMPR1_SMP24_Pos) ## !< 0x00007000
  ADC_SMPR1_SMP24* = ADC_SMPR1_SMP24_Msk
  ADC_SMPR1_SMP24_0* = (0x00000001 shl ADC_SMPR1_SMP24_Pos) ## !< 0x00001000
  ADC_SMPR1_SMP24_1* = (0x00000002 shl ADC_SMPR1_SMP24_Pos) ## !< 0x00002000
  ADC_SMPR1_SMP24_2* = (0x00000004 shl ADC_SMPR1_SMP24_Pos) ## !< 0x00004000
  ADC_SMPR1_SMP25_Pos* = (15)
  ADC_SMPR1_SMP25_Msk* = (0x00000007 shl ADC_SMPR1_SMP25_Pos) ## !< 0x00038000
  ADC_SMPR1_SMP25* = ADC_SMPR1_SMP25_Msk
  ADC_SMPR1_SMP25_0* = (0x00000001 shl ADC_SMPR1_SMP25_Pos) ## !< 0x00008000
  ADC_SMPR1_SMP25_1* = (0x00000002 shl ADC_SMPR1_SMP25_Pos) ## !< 0x00010000
  ADC_SMPR1_SMP25_2* = (0x00000004 shl ADC_SMPR1_SMP25_Pos) ## !< 0x00020000
  ADC_SMPR1_SMP26_Pos* = (18)
  ADC_SMPR1_SMP26_Msk* = (0x00000007 shl ADC_SMPR1_SMP26_Pos) ## !< 0x001C0000
  ADC_SMPR1_SMP26* = ADC_SMPR1_SMP26_Msk
  ADC_SMPR1_SMP26_0* = (0x00000001 shl ADC_SMPR1_SMP26_Pos) ## !< 0x00040000
  ADC_SMPR1_SMP26_1* = (0x00000002 shl ADC_SMPR1_SMP26_Pos) ## !< 0x00080000
  ADC_SMPR1_SMP26_2* = (0x00000004 shl ADC_SMPR1_SMP26_Pos) ## !< 0x00100000
  ADC_SMPR1_SMP27_Pos* = (21)
  ADC_SMPR1_SMP27_Msk* = (0x00000007 shl ADC_SMPR1_SMP27_Pos) ## !< 0x00E00000
  ADC_SMPR1_SMP27* = ADC_SMPR1_SMP27_Msk
  ADC_SMPR1_SMP27_0* = (0x00000001 shl ADC_SMPR1_SMP27_Pos) ## !< 0x00200000
  ADC_SMPR1_SMP27_1* = (0x00000002 shl ADC_SMPR1_SMP27_Pos) ## !< 0x00400000
  ADC_SMPR1_SMP27_2* = (0x00000004 shl ADC_SMPR1_SMP27_Pos) ## !< 0x00800000
  ADC_SMPR1_SMP28_Pos* = (24)
  ADC_SMPR1_SMP28_Msk* = (0x00000007 shl ADC_SMPR1_SMP28_Pos) ## !< 0x07000000
  ADC_SMPR1_SMP28* = ADC_SMPR1_SMP28_Msk
  ADC_SMPR1_SMP28_0* = (0x00000001 shl ADC_SMPR1_SMP28_Pos) ## !< 0x01000000
  ADC_SMPR1_SMP28_1* = (0x00000002 shl ADC_SMPR1_SMP28_Pos) ## !< 0x02000000
  ADC_SMPR1_SMP28_2* = (0x00000004 shl ADC_SMPR1_SMP28_Pos) ## !< 0x04000000
  ADC_SMPR1_SMP29_Pos* = (27)
  ADC_SMPR1_SMP29_Msk* = (0x00000007 shl ADC_SMPR1_SMP29_Pos) ## !< 0x38000000
  ADC_SMPR1_SMP29* = ADC_SMPR1_SMP29_Msk
  ADC_SMPR1_SMP29_0* = (0x00000001 shl ADC_SMPR1_SMP29_Pos) ## !< 0x08000000
  ADC_SMPR1_SMP29_1* = (0x00000002 shl ADC_SMPR1_SMP29_Pos) ## !< 0x10000000
  ADC_SMPR1_SMP29_2* = (0x00000004 shl ADC_SMPR1_SMP29_Pos) ## !< 0x20000000

## *****************  Bit definition for ADC_SMPR2 register  ******************

const
  ADC_SMPR2_SMP10_Pos* = (0)
  ADC_SMPR2_SMP10_Msk* = (0x00000007 shl ADC_SMPR2_SMP10_Pos) ## !< 0x00000007
  ADC_SMPR2_SMP10* = ADC_SMPR2_SMP10_Msk
  ADC_SMPR2_SMP10_0* = (0x00000001 shl ADC_SMPR2_SMP10_Pos) ## !< 0x00000001
  ADC_SMPR2_SMP10_1* = (0x00000002 shl ADC_SMPR2_SMP10_Pos) ## !< 0x00000002
  ADC_SMPR2_SMP10_2* = (0x00000004 shl ADC_SMPR2_SMP10_Pos) ## !< 0x00000004
  ADC_SMPR2_SMP11_Pos* = (3)
  ADC_SMPR2_SMP11_Msk* = (0x00000007 shl ADC_SMPR2_SMP11_Pos) ## !< 0x00000038
  ADC_SMPR2_SMP11* = ADC_SMPR2_SMP11_Msk
  ADC_SMPR2_SMP11_0* = (0x00000001 shl ADC_SMPR2_SMP11_Pos) ## !< 0x00000008
  ADC_SMPR2_SMP11_1* = (0x00000002 shl ADC_SMPR2_SMP11_Pos) ## !< 0x00000010
  ADC_SMPR2_SMP11_2* = (0x00000004 shl ADC_SMPR2_SMP11_Pos) ## !< 0x00000020
  ADC_SMPR2_SMP12_Pos* = (6)
  ADC_SMPR2_SMP12_Msk* = (0x00000007 shl ADC_SMPR2_SMP12_Pos) ## !< 0x000001C0
  ADC_SMPR2_SMP12* = ADC_SMPR2_SMP12_Msk
  ADC_SMPR2_SMP12_0* = (0x00000001 shl ADC_SMPR2_SMP12_Pos) ## !< 0x00000040
  ADC_SMPR2_SMP12_1* = (0x00000002 shl ADC_SMPR2_SMP12_Pos) ## !< 0x00000080
  ADC_SMPR2_SMP12_2* = (0x00000004 shl ADC_SMPR2_SMP12_Pos) ## !< 0x00000100
  ADC_SMPR2_SMP13_Pos* = (9)
  ADC_SMPR2_SMP13_Msk* = (0x00000007 shl ADC_SMPR2_SMP13_Pos) ## !< 0x00000E00
  ADC_SMPR2_SMP13* = ADC_SMPR2_SMP13_Msk
  ADC_SMPR2_SMP13_0* = (0x00000001 shl ADC_SMPR2_SMP13_Pos) ## !< 0x00000200
  ADC_SMPR2_SMP13_1* = (0x00000002 shl ADC_SMPR2_SMP13_Pos) ## !< 0x00000400
  ADC_SMPR2_SMP13_2* = (0x00000004 shl ADC_SMPR2_SMP13_Pos) ## !< 0x00000800
  ADC_SMPR2_SMP14_Pos* = (12)
  ADC_SMPR2_SMP14_Msk* = (0x00000007 shl ADC_SMPR2_SMP14_Pos) ## !< 0x00007000
  ADC_SMPR2_SMP14* = ADC_SMPR2_SMP14_Msk
  ADC_SMPR2_SMP14_0* = (0x00000001 shl ADC_SMPR2_SMP14_Pos) ## !< 0x00001000
  ADC_SMPR2_SMP14_1* = (0x00000002 shl ADC_SMPR2_SMP14_Pos) ## !< 0x00002000
  ADC_SMPR2_SMP14_2* = (0x00000004 shl ADC_SMPR2_SMP14_Pos) ## !< 0x00004000
  ADC_SMPR2_SMP15_Pos* = (15)
  ADC_SMPR2_SMP15_Msk* = (0x00000007 shl ADC_SMPR2_SMP15_Pos) ## !< 0x00038000
  ADC_SMPR2_SMP15* = ADC_SMPR2_SMP15_Msk
  ADC_SMPR2_SMP15_0* = (0x00000001 shl ADC_SMPR2_SMP15_Pos) ## !< 0x00008000
  ADC_SMPR2_SMP15_1* = (0x00000002 shl ADC_SMPR2_SMP15_Pos) ## !< 0x00010000
  ADC_SMPR2_SMP15_2* = (0x00000004 shl ADC_SMPR2_SMP15_Pos) ## !< 0x00020000
  ADC_SMPR2_SMP16_Pos* = (18)
  ADC_SMPR2_SMP16_Msk* = (0x00000007 shl ADC_SMPR2_SMP16_Pos) ## !< 0x001C0000
  ADC_SMPR2_SMP16* = ADC_SMPR2_SMP16_Msk
  ADC_SMPR2_SMP16_0* = (0x00000001 shl ADC_SMPR2_SMP16_Pos) ## !< 0x00040000
  ADC_SMPR2_SMP16_1* = (0x00000002 shl ADC_SMPR2_SMP16_Pos) ## !< 0x00080000
  ADC_SMPR2_SMP16_2* = (0x00000004 shl ADC_SMPR2_SMP16_Pos) ## !< 0x00100000
  ADC_SMPR2_SMP17_Pos* = (21)
  ADC_SMPR2_SMP17_Msk* = (0x00000007 shl ADC_SMPR2_SMP17_Pos) ## !< 0x00E00000
  ADC_SMPR2_SMP17* = ADC_SMPR2_SMP17_Msk
  ADC_SMPR2_SMP17_0* = (0x00000001 shl ADC_SMPR2_SMP17_Pos) ## !< 0x00200000
  ADC_SMPR2_SMP17_1* = (0x00000002 shl ADC_SMPR2_SMP17_Pos) ## !< 0x00400000
  ADC_SMPR2_SMP17_2* = (0x00000004 shl ADC_SMPR2_SMP17_Pos) ## !< 0x00800000
  ADC_SMPR2_SMP18_Pos* = (24)
  ADC_SMPR2_SMP18_Msk* = (0x00000007 shl ADC_SMPR2_SMP18_Pos) ## !< 0x07000000
  ADC_SMPR2_SMP18* = ADC_SMPR2_SMP18_Msk
  ADC_SMPR2_SMP18_0* = (0x00000001 shl ADC_SMPR2_SMP18_Pos) ## !< 0x01000000
  ADC_SMPR2_SMP18_1* = (0x00000002 shl ADC_SMPR2_SMP18_Pos) ## !< 0x02000000
  ADC_SMPR2_SMP18_2* = (0x00000004 shl ADC_SMPR2_SMP18_Pos) ## !< 0x04000000
  ADC_SMPR2_SMP19_Pos* = (27)
  ADC_SMPR2_SMP19_Msk* = (0x00000007 shl ADC_SMPR2_SMP19_Pos) ## !< 0x38000000
  ADC_SMPR2_SMP19* = ADC_SMPR2_SMP19_Msk
  ADC_SMPR2_SMP19_0* = (0x00000001 shl ADC_SMPR2_SMP19_Pos) ## !< 0x08000000
  ADC_SMPR2_SMP19_1* = (0x00000002 shl ADC_SMPR2_SMP19_Pos) ## !< 0x10000000
  ADC_SMPR2_SMP19_2* = (0x00000004 shl ADC_SMPR2_SMP19_Pos) ## !< 0x20000000

## *****************  Bit definition for ADC_SMPR3 register  ******************

const
  ADC_SMPR3_SMP0_Pos* = (0)
  ADC_SMPR3_SMP0_Msk* = (0x00000007 shl ADC_SMPR3_SMP0_Pos) ## !< 0x00000007
  ADC_SMPR3_SMP0* = ADC_SMPR3_SMP0_Msk
  ADC_SMPR3_SMP0_0* = (0x00000001 shl ADC_SMPR3_SMP0_Pos) ## !< 0x00000001
  ADC_SMPR3_SMP0_1* = (0x00000002 shl ADC_SMPR3_SMP0_Pos) ## !< 0x00000002
  ADC_SMPR3_SMP0_2* = (0x00000004 shl ADC_SMPR3_SMP0_Pos) ## !< 0x00000004
  ADC_SMPR3_SMP1_Pos* = (3)
  ADC_SMPR3_SMP1_Msk* = (0x00000007 shl ADC_SMPR3_SMP1_Pos) ## !< 0x00000038
  ADC_SMPR3_SMP1* = ADC_SMPR3_SMP1_Msk
  ADC_SMPR3_SMP1_0* = (0x00000001 shl ADC_SMPR3_SMP1_Pos) ## !< 0x00000008
  ADC_SMPR3_SMP1_1* = (0x00000002 shl ADC_SMPR3_SMP1_Pos) ## !< 0x00000010
  ADC_SMPR3_SMP1_2* = (0x00000004 shl ADC_SMPR3_SMP1_Pos) ## !< 0x00000020
  ADC_SMPR3_SMP2_Pos* = (6)
  ADC_SMPR3_SMP2_Msk* = (0x00000007 shl ADC_SMPR3_SMP2_Pos) ## !< 0x000001C0
  ADC_SMPR3_SMP2* = ADC_SMPR3_SMP2_Msk
  ADC_SMPR3_SMP2_0* = (0x00000001 shl ADC_SMPR3_SMP2_Pos) ## !< 0x00000040
  ADC_SMPR3_SMP2_1* = (0x00000002 shl ADC_SMPR3_SMP2_Pos) ## !< 0x00000080
  ADC_SMPR3_SMP2_2* = (0x00000004 shl ADC_SMPR3_SMP2_Pos) ## !< 0x00000100
  ADC_SMPR3_SMP3_Pos* = (9)
  ADC_SMPR3_SMP3_Msk* = (0x00000007 shl ADC_SMPR3_SMP3_Pos) ## !< 0x00000E00
  ADC_SMPR3_SMP3* = ADC_SMPR3_SMP3_Msk
  ADC_SMPR3_SMP3_0* = (0x00000001 shl ADC_SMPR3_SMP3_Pos) ## !< 0x00000200
  ADC_SMPR3_SMP3_1* = (0x00000002 shl ADC_SMPR3_SMP3_Pos) ## !< 0x00000400
  ADC_SMPR3_SMP3_2* = (0x00000004 shl ADC_SMPR3_SMP3_Pos) ## !< 0x00000800
  ADC_SMPR3_SMP4_Pos* = (12)
  ADC_SMPR3_SMP4_Msk* = (0x00000007 shl ADC_SMPR3_SMP4_Pos) ## !< 0x00007000
  ADC_SMPR3_SMP4* = ADC_SMPR3_SMP4_Msk
  ADC_SMPR3_SMP4_0* = (0x00000001 shl ADC_SMPR3_SMP4_Pos) ## !< 0x00001000
  ADC_SMPR3_SMP4_1* = (0x00000002 shl ADC_SMPR3_SMP4_Pos) ## !< 0x00002000
  ADC_SMPR3_SMP4_2* = (0x00000004 shl ADC_SMPR3_SMP4_Pos) ## !< 0x00004000
  ADC_SMPR3_SMP5_Pos* = (15)
  ADC_SMPR3_SMP5_Msk* = (0x00000007 shl ADC_SMPR3_SMP5_Pos) ## !< 0x00038000
  ADC_SMPR3_SMP5* = ADC_SMPR3_SMP5_Msk
  ADC_SMPR3_SMP5_0* = (0x00000001 shl ADC_SMPR3_SMP5_Pos) ## !< 0x00008000
  ADC_SMPR3_SMP5_1* = (0x00000002 shl ADC_SMPR3_SMP5_Pos) ## !< 0x00010000
  ADC_SMPR3_SMP5_2* = (0x00000004 shl ADC_SMPR3_SMP5_Pos) ## !< 0x00020000
  ADC_SMPR3_SMP6_Pos* = (18)
  ADC_SMPR3_SMP6_Msk* = (0x00000007 shl ADC_SMPR3_SMP6_Pos) ## !< 0x001C0000
  ADC_SMPR3_SMP6* = ADC_SMPR3_SMP6_Msk
  ADC_SMPR3_SMP6_0* = (0x00000001 shl ADC_SMPR3_SMP6_Pos) ## !< 0x00040000
  ADC_SMPR3_SMP6_1* = (0x00000002 shl ADC_SMPR3_SMP6_Pos) ## !< 0x00080000
  ADC_SMPR3_SMP6_2* = (0x00000004 shl ADC_SMPR3_SMP6_Pos) ## !< 0x00100000
  ADC_SMPR3_SMP7_Pos* = (21)
  ADC_SMPR3_SMP7_Msk* = (0x00000007 shl ADC_SMPR3_SMP7_Pos) ## !< 0x00E00000
  ADC_SMPR3_SMP7* = ADC_SMPR3_SMP7_Msk
  ADC_SMPR3_SMP7_0* = (0x00000001 shl ADC_SMPR3_SMP7_Pos) ## !< 0x00200000
  ADC_SMPR3_SMP7_1* = (0x00000002 shl ADC_SMPR3_SMP7_Pos) ## !< 0x00400000
  ADC_SMPR3_SMP7_2* = (0x00000004 shl ADC_SMPR3_SMP7_Pos) ## !< 0x00800000
  ADC_SMPR3_SMP8_Pos* = (24)
  ADC_SMPR3_SMP8_Msk* = (0x00000007 shl ADC_SMPR3_SMP8_Pos) ## !< 0x07000000
  ADC_SMPR3_SMP8* = ADC_SMPR3_SMP8_Msk
  ADC_SMPR3_SMP8_0* = (0x00000001 shl ADC_SMPR3_SMP8_Pos) ## !< 0x01000000
  ADC_SMPR3_SMP8_1* = (0x00000002 shl ADC_SMPR3_SMP8_Pos) ## !< 0x02000000
  ADC_SMPR3_SMP8_2* = (0x00000004 shl ADC_SMPR3_SMP8_Pos) ## !< 0x04000000
  ADC_SMPR3_SMP9_Pos* = (27)
  ADC_SMPR3_SMP9_Msk* = (0x00000007 shl ADC_SMPR3_SMP9_Pos) ## !< 0x38000000
  ADC_SMPR3_SMP9* = ADC_SMPR3_SMP9_Msk
  ADC_SMPR3_SMP9_0* = (0x00000001 shl ADC_SMPR3_SMP9_Pos) ## !< 0x08000000
  ADC_SMPR3_SMP9_1* = (0x00000002 shl ADC_SMPR3_SMP9_Pos) ## !< 0x10000000
  ADC_SMPR3_SMP9_2* = (0x00000004 shl ADC_SMPR3_SMP9_Pos) ## !< 0x20000000

## *****************  Bit definition for ADC_JOFR1 register  ******************

const
  ADC_JOFR1_JOFFSET1_Pos* = (0)
  ADC_JOFR1_JOFFSET1_Msk* = (0x00000FFF shl ADC_JOFR1_JOFFSET1_Pos) ## !< 0x00000FFF
  ADC_JOFR1_JOFFSET1* = ADC_JOFR1_JOFFSET1_Msk

## *****************  Bit definition for ADC_JOFR2 register  ******************

const
  ADC_JOFR2_JOFFSET2_Pos* = (0)
  ADC_JOFR2_JOFFSET2_Msk* = (0x00000FFF shl ADC_JOFR2_JOFFSET2_Pos) ## !< 0x00000FFF
  ADC_JOFR2_JOFFSET2* = ADC_JOFR2_JOFFSET2_Msk

## *****************  Bit definition for ADC_JOFR3 register  ******************

const
  ADC_JOFR3_JOFFSET3_Pos* = (0)
  ADC_JOFR3_JOFFSET3_Msk* = (0x00000FFF shl ADC_JOFR3_JOFFSET3_Pos) ## !< 0x00000FFF
  ADC_JOFR3_JOFFSET3* = ADC_JOFR3_JOFFSET3_Msk

## *****************  Bit definition for ADC_JOFR4 register  ******************

const
  ADC_JOFR4_JOFFSET4_Pos* = (0)
  ADC_JOFR4_JOFFSET4_Msk* = (0x00000FFF shl ADC_JOFR4_JOFFSET4_Pos) ## !< 0x00000FFF
  ADC_JOFR4_JOFFSET4* = ADC_JOFR4_JOFFSET4_Msk

## ******************  Bit definition for ADC_HTR register  *******************

const
  ADC_HTR_HT_Pos* = (0)
  ADC_HTR_HT_Msk* = (0x00000FFF shl ADC_HTR_HT_Pos) ## !< 0x00000FFF
  ADC_HTR_HT* = ADC_HTR_HT_Msk

## ******************  Bit definition for ADC_LTR register  *******************

const
  ADC_LTR_LT_Pos* = (0)
  ADC_LTR_LT_Msk* = (0x00000FFF shl ADC_LTR_LT_Pos) ## !< 0x00000FFF
  ADC_LTR_LT* = ADC_LTR_LT_Msk

## ******************  Bit definition for ADC_SQR1 register  ******************

const
  ADC_SQR1_L_Pos* = (20)
  ADC_SQR1_L_Msk* = (0x0000001F shl ADC_SQR1_L_Pos) ## !< 0x01F00000
  ADC_SQR1_L* = ADC_SQR1_L_Msk
  ADC_SQR1_L_0* = (0x00000001 shl ADC_SQR1_L_Pos) ## !< 0x00100000
  ADC_SQR1_L_1* = (0x00000002 shl ADC_SQR1_L_Pos) ## !< 0x00200000
  ADC_SQR1_L_2* = (0x00000004 shl ADC_SQR1_L_Pos) ## !< 0x00400000
  ADC_SQR1_L_3* = (0x00000008 shl ADC_SQR1_L_Pos) ## !< 0x00800000
  ADC_SQR1_L_4* = (0x00000010 shl ADC_SQR1_L_Pos) ## !< 0x01000000
  ADC_SQR1_SQ28_Pos* = (15)
  ADC_SQR1_SQ28_Msk* = (0x0000001F shl ADC_SQR1_SQ28_Pos) ## !< 0x000F8000
  ADC_SQR1_SQ28* = ADC_SQR1_SQ28_Msk
  ADC_SQR1_SQ28_0* = (0x00000001 shl ADC_SQR1_SQ28_Pos) ## !< 0x00008000
  ADC_SQR1_SQ28_1* = (0x00000002 shl ADC_SQR1_SQ28_Pos) ## !< 0x00010000
  ADC_SQR1_SQ28_2* = (0x00000004 shl ADC_SQR1_SQ28_Pos) ## !< 0x00020000
  ADC_SQR1_SQ28_3* = (0x00000008 shl ADC_SQR1_SQ28_Pos) ## !< 0x00040000
  ADC_SQR1_SQ28_4* = (0x00000010 shl ADC_SQR1_SQ28_Pos) ## !< 0x00080000
  ADC_SQR1_SQ27_Pos* = (10)
  ADC_SQR1_SQ27_Msk* = (0x0000001F shl ADC_SQR1_SQ27_Pos) ## !< 0x00007C00
  ADC_SQR1_SQ27* = ADC_SQR1_SQ27_Msk
  ADC_SQR1_SQ27_0* = (0x00000001 shl ADC_SQR1_SQ27_Pos) ## !< 0x00000400
  ADC_SQR1_SQ27_1* = (0x00000002 shl ADC_SQR1_SQ27_Pos) ## !< 0x00000800
  ADC_SQR1_SQ27_2* = (0x00000004 shl ADC_SQR1_SQ27_Pos) ## !< 0x00001000
  ADC_SQR1_SQ27_3* = (0x00000008 shl ADC_SQR1_SQ27_Pos) ## !< 0x00002000
  ADC_SQR1_SQ27_4* = (0x00000010 shl ADC_SQR1_SQ27_Pos) ## !< 0x00004000
  ADC_SQR1_SQ26_Pos* = (5)
  ADC_SQR1_SQ26_Msk* = (0x0000001F shl ADC_SQR1_SQ26_Pos) ## !< 0x000003E0
  ADC_SQR1_SQ26* = ADC_SQR1_SQ26_Msk
  ADC_SQR1_SQ26_0* = (0x00000001 shl ADC_SQR1_SQ26_Pos) ## !< 0x00000020
  ADC_SQR1_SQ26_1* = (0x00000002 shl ADC_SQR1_SQ26_Pos) ## !< 0x00000040
  ADC_SQR1_SQ26_2* = (0x00000004 shl ADC_SQR1_SQ26_Pos) ## !< 0x00000080
  ADC_SQR1_SQ26_3* = (0x00000008 shl ADC_SQR1_SQ26_Pos) ## !< 0x00000100
  ADC_SQR1_SQ26_4* = (0x00000010 shl ADC_SQR1_SQ26_Pos) ## !< 0x00000200
  ADC_SQR1_SQ25_Pos* = (0)
  ADC_SQR1_SQ25_Msk* = (0x0000001F shl ADC_SQR1_SQ25_Pos) ## !< 0x0000001F
  ADC_SQR1_SQ25* = ADC_SQR1_SQ25_Msk
  ADC_SQR1_SQ25_0* = (0x00000001 shl ADC_SQR1_SQ25_Pos) ## !< 0x00000001
  ADC_SQR1_SQ25_1* = (0x00000002 shl ADC_SQR1_SQ25_Pos) ## !< 0x00000002
  ADC_SQR1_SQ25_2* = (0x00000004 shl ADC_SQR1_SQ25_Pos) ## !< 0x00000004
  ADC_SQR1_SQ25_3* = (0x00000008 shl ADC_SQR1_SQ25_Pos) ## !< 0x00000008
  ADC_SQR1_SQ25_4* = (0x00000010 shl ADC_SQR1_SQ25_Pos) ## !< 0x00000010

## ******************  Bit definition for ADC_SQR2 register  ******************

const
  ADC_SQR2_SQ19_Pos* = (0)
  ADC_SQR2_SQ19_Msk* = (0x0000001F shl ADC_SQR2_SQ19_Pos) ## !< 0x0000001F
  ADC_SQR2_SQ19* = ADC_SQR2_SQ19_Msk
  ADC_SQR2_SQ19_0* = (0x00000001 shl ADC_SQR2_SQ19_Pos) ## !< 0x00000001
  ADC_SQR2_SQ19_1* = (0x00000002 shl ADC_SQR2_SQ19_Pos) ## !< 0x00000002
  ADC_SQR2_SQ19_2* = (0x00000004 shl ADC_SQR2_SQ19_Pos) ## !< 0x00000004
  ADC_SQR2_SQ19_3* = (0x00000008 shl ADC_SQR2_SQ19_Pos) ## !< 0x00000008
  ADC_SQR2_SQ19_4* = (0x00000010 shl ADC_SQR2_SQ19_Pos) ## !< 0x00000010
  ADC_SQR2_SQ20_Pos* = (5)
  ADC_SQR2_SQ20_Msk* = (0x0000001F shl ADC_SQR2_SQ20_Pos) ## !< 0x000003E0
  ADC_SQR2_SQ20* = ADC_SQR2_SQ20_Msk
  ADC_SQR2_SQ20_0* = (0x00000001 shl ADC_SQR2_SQ20_Pos) ## !< 0x00000020
  ADC_SQR2_SQ20_1* = (0x00000002 shl ADC_SQR2_SQ20_Pos) ## !< 0x00000040
  ADC_SQR2_SQ20_2* = (0x00000004 shl ADC_SQR2_SQ20_Pos) ## !< 0x00000080
  ADC_SQR2_SQ20_3* = (0x00000008 shl ADC_SQR2_SQ20_Pos) ## !< 0x00000100
  ADC_SQR2_SQ20_4* = (0x00000010 shl ADC_SQR2_SQ20_Pos) ## !< 0x00000200
  ADC_SQR2_SQ21_Pos* = (10)
  ADC_SQR2_SQ21_Msk* = (0x0000001F shl ADC_SQR2_SQ21_Pos) ## !< 0x00007C00
  ADC_SQR2_SQ21* = ADC_SQR2_SQ21_Msk
  ADC_SQR2_SQ21_0* = (0x00000001 shl ADC_SQR2_SQ21_Pos) ## !< 0x00000400
  ADC_SQR2_SQ21_1* = (0x00000002 shl ADC_SQR2_SQ21_Pos) ## !< 0x00000800
  ADC_SQR2_SQ21_2* = (0x00000004 shl ADC_SQR2_SQ21_Pos) ## !< 0x00001000
  ADC_SQR2_SQ21_3* = (0x00000008 shl ADC_SQR2_SQ21_Pos) ## !< 0x00002000
  ADC_SQR2_SQ21_4* = (0x00000010 shl ADC_SQR2_SQ21_Pos) ## !< 0x00004000
  ADC_SQR2_SQ22_Pos* = (15)
  ADC_SQR2_SQ22_Msk* = (0x0000001F shl ADC_SQR2_SQ22_Pos) ## !< 0x000F8000
  ADC_SQR2_SQ22* = ADC_SQR2_SQ22_Msk
  ADC_SQR2_SQ22_0* = (0x00000001 shl ADC_SQR2_SQ22_Pos) ## !< 0x00008000
  ADC_SQR2_SQ22_1* = (0x00000002 shl ADC_SQR2_SQ22_Pos) ## !< 0x00010000
  ADC_SQR2_SQ22_2* = (0x00000004 shl ADC_SQR2_SQ22_Pos) ## !< 0x00020000
  ADC_SQR2_SQ22_3* = (0x00000008 shl ADC_SQR2_SQ22_Pos) ## !< 0x00040000
  ADC_SQR2_SQ22_4* = (0x00000010 shl ADC_SQR2_SQ22_Pos) ## !< 0x00080000
  ADC_SQR2_SQ23_Pos* = (20)
  ADC_SQR2_SQ23_Msk* = (0x0000001F shl ADC_SQR2_SQ23_Pos) ## !< 0x01F00000
  ADC_SQR2_SQ23* = ADC_SQR2_SQ23_Msk
  ADC_SQR2_SQ23_0* = (0x00000001 shl ADC_SQR2_SQ23_Pos) ## !< 0x00100000
  ADC_SQR2_SQ23_1* = (0x00000002 shl ADC_SQR2_SQ23_Pos) ## !< 0x00200000
  ADC_SQR2_SQ23_2* = (0x00000004 shl ADC_SQR2_SQ23_Pos) ## !< 0x00400000
  ADC_SQR2_SQ23_3* = (0x00000008 shl ADC_SQR2_SQ23_Pos) ## !< 0x00800000
  ADC_SQR2_SQ23_4* = (0x00000010 shl ADC_SQR2_SQ23_Pos) ## !< 0x01000000
  ADC_SQR2_SQ24_Pos* = (25)
  ADC_SQR2_SQ24_Msk* = (0x0000001F shl ADC_SQR2_SQ24_Pos) ## !< 0x3E000000
  ADC_SQR2_SQ24* = ADC_SQR2_SQ24_Msk
  ADC_SQR2_SQ24_0* = (0x00000001 shl ADC_SQR2_SQ24_Pos) ## !< 0x02000000
  ADC_SQR2_SQ24_1* = (0x00000002 shl ADC_SQR2_SQ24_Pos) ## !< 0x04000000
  ADC_SQR2_SQ24_2* = (0x00000004 shl ADC_SQR2_SQ24_Pos) ## !< 0x08000000
  ADC_SQR2_SQ24_3* = (0x00000008 shl ADC_SQR2_SQ24_Pos) ## !< 0x10000000
  ADC_SQR2_SQ24_4* = (0x00000010 shl ADC_SQR2_SQ24_Pos) ## !< 0x20000000

## ******************  Bit definition for ADC_SQR3 register  ******************

const
  ADC_SQR3_SQ13_Pos* = (0)
  ADC_SQR3_SQ13_Msk* = (0x0000001F shl ADC_SQR3_SQ13_Pos) ## !< 0x0000001F
  ADC_SQR3_SQ13* = ADC_SQR3_SQ13_Msk
  ADC_SQR3_SQ13_0* = (0x00000001 shl ADC_SQR3_SQ13_Pos) ## !< 0x00000001
  ADC_SQR3_SQ13_1* = (0x00000002 shl ADC_SQR3_SQ13_Pos) ## !< 0x00000002
  ADC_SQR3_SQ13_2* = (0x00000004 shl ADC_SQR3_SQ13_Pos) ## !< 0x00000004
  ADC_SQR3_SQ13_3* = (0x00000008 shl ADC_SQR3_SQ13_Pos) ## !< 0x00000008
  ADC_SQR3_SQ13_4* = (0x00000010 shl ADC_SQR3_SQ13_Pos) ## !< 0x00000010
  ADC_SQR3_SQ14_Pos* = (5)
  ADC_SQR3_SQ14_Msk* = (0x0000001F shl ADC_SQR3_SQ14_Pos) ## !< 0x000003E0
  ADC_SQR3_SQ14* = ADC_SQR3_SQ14_Msk
  ADC_SQR3_SQ14_0* = (0x00000001 shl ADC_SQR3_SQ14_Pos) ## !< 0x00000020
  ADC_SQR3_SQ14_1* = (0x00000002 shl ADC_SQR3_SQ14_Pos) ## !< 0x00000040
  ADC_SQR3_SQ14_2* = (0x00000004 shl ADC_SQR3_SQ14_Pos) ## !< 0x00000080
  ADC_SQR3_SQ14_3* = (0x00000008 shl ADC_SQR3_SQ14_Pos) ## !< 0x00000100
  ADC_SQR3_SQ14_4* = (0x00000010 shl ADC_SQR3_SQ14_Pos) ## !< 0x00000200
  ADC_SQR3_SQ15_Pos* = (10)
  ADC_SQR3_SQ15_Msk* = (0x0000001F shl ADC_SQR3_SQ15_Pos) ## !< 0x00007C00
  ADC_SQR3_SQ15* = ADC_SQR3_SQ15_Msk
  ADC_SQR3_SQ15_0* = (0x00000001 shl ADC_SQR3_SQ15_Pos) ## !< 0x00000400
  ADC_SQR3_SQ15_1* = (0x00000002 shl ADC_SQR3_SQ15_Pos) ## !< 0x00000800
  ADC_SQR3_SQ15_2* = (0x00000004 shl ADC_SQR3_SQ15_Pos) ## !< 0x00001000
  ADC_SQR3_SQ15_3* = (0x00000008 shl ADC_SQR3_SQ15_Pos) ## !< 0x00002000
  ADC_SQR3_SQ15_4* = (0x00000010 shl ADC_SQR3_SQ15_Pos) ## !< 0x00004000
  ADC_SQR3_SQ16_Pos* = (15)
  ADC_SQR3_SQ16_Msk* = (0x0000001F shl ADC_SQR3_SQ16_Pos) ## !< 0x000F8000
  ADC_SQR3_SQ16* = ADC_SQR3_SQ16_Msk
  ADC_SQR3_SQ16_0* = (0x00000001 shl ADC_SQR3_SQ16_Pos) ## !< 0x00008000
  ADC_SQR3_SQ16_1* = (0x00000002 shl ADC_SQR3_SQ16_Pos) ## !< 0x00010000
  ADC_SQR3_SQ16_2* = (0x00000004 shl ADC_SQR3_SQ16_Pos) ## !< 0x00020000
  ADC_SQR3_SQ16_3* = (0x00000008 shl ADC_SQR3_SQ16_Pos) ## !< 0x00040000
  ADC_SQR3_SQ16_4* = (0x00000010 shl ADC_SQR3_SQ16_Pos) ## !< 0x00080000
  ADC_SQR3_SQ17_Pos* = (20)
  ADC_SQR3_SQ17_Msk* = (0x0000001F shl ADC_SQR3_SQ17_Pos) ## !< 0x01F00000
  ADC_SQR3_SQ17* = ADC_SQR3_SQ17_Msk
  ADC_SQR3_SQ17_0* = (0x00000001 shl ADC_SQR3_SQ17_Pos) ## !< 0x00100000
  ADC_SQR3_SQ17_1* = (0x00000002 shl ADC_SQR3_SQ17_Pos) ## !< 0x00200000
  ADC_SQR3_SQ17_2* = (0x00000004 shl ADC_SQR3_SQ17_Pos) ## !< 0x00400000
  ADC_SQR3_SQ17_3* = (0x00000008 shl ADC_SQR3_SQ17_Pos) ## !< 0x00800000
  ADC_SQR3_SQ17_4* = (0x00000010 shl ADC_SQR3_SQ17_Pos) ## !< 0x01000000
  ADC_SQR3_SQ18_Pos* = (25)
  ADC_SQR3_SQ18_Msk* = (0x0000001F shl ADC_SQR3_SQ18_Pos) ## !< 0x3E000000
  ADC_SQR3_SQ18* = ADC_SQR3_SQ18_Msk
  ADC_SQR3_SQ18_0* = (0x00000001 shl ADC_SQR3_SQ18_Pos) ## !< 0x02000000
  ADC_SQR3_SQ18_1* = (0x00000002 shl ADC_SQR3_SQ18_Pos) ## !< 0x04000000
  ADC_SQR3_SQ18_2* = (0x00000004 shl ADC_SQR3_SQ18_Pos) ## !< 0x08000000
  ADC_SQR3_SQ18_3* = (0x00000008 shl ADC_SQR3_SQ18_Pos) ## !< 0x10000000
  ADC_SQR3_SQ18_4* = (0x00000010 shl ADC_SQR3_SQ18_Pos) ## !< 0x20000000

## ******************  Bit definition for ADC_SQR4 register  ******************

const
  ADC_SQR4_SQ7_Pos* = (0)
  ADC_SQR4_SQ7_Msk* = (0x0000001F shl ADC_SQR4_SQ7_Pos) ## !< 0x0000001F
  ADC_SQR4_SQ7* = ADC_SQR4_SQ7_Msk
  ADC_SQR4_SQ7_0* = (0x00000001 shl ADC_SQR4_SQ7_Pos) ## !< 0x00000001
  ADC_SQR4_SQ7_1* = (0x00000002 shl ADC_SQR4_SQ7_Pos) ## !< 0x00000002
  ADC_SQR4_SQ7_2* = (0x00000004 shl ADC_SQR4_SQ7_Pos) ## !< 0x00000004
  ADC_SQR4_SQ7_3* = (0x00000008 shl ADC_SQR4_SQ7_Pos) ## !< 0x00000008
  ADC_SQR4_SQ7_4* = (0x00000010 shl ADC_SQR4_SQ7_Pos) ## !< 0x00000010
  ADC_SQR4_SQ8_Pos* = (5)
  ADC_SQR4_SQ8_Msk* = (0x0000001F shl ADC_SQR4_SQ8_Pos) ## !< 0x000003E0
  ADC_SQR4_SQ8* = ADC_SQR4_SQ8_Msk
  ADC_SQR4_SQ8_0* = (0x00000001 shl ADC_SQR4_SQ8_Pos) ## !< 0x00000020
  ADC_SQR4_SQ8_1* = (0x00000002 shl ADC_SQR4_SQ8_Pos) ## !< 0x00000040
  ADC_SQR4_SQ8_2* = (0x00000004 shl ADC_SQR4_SQ8_Pos) ## !< 0x00000080
  ADC_SQR4_SQ8_3* = (0x00000008 shl ADC_SQR4_SQ8_Pos) ## !< 0x00000100
  ADC_SQR4_SQ8_4* = (0x00000010 shl ADC_SQR4_SQ8_Pos) ## !< 0x00000200
  ADC_SQR4_SQ9_Pos* = (10)
  ADC_SQR4_SQ9_Msk* = (0x0000001F shl ADC_SQR4_SQ9_Pos) ## !< 0x00007C00
  ADC_SQR4_SQ9* = ADC_SQR4_SQ9_Msk
  ADC_SQR4_SQ9_0* = (0x00000001 shl ADC_SQR4_SQ9_Pos) ## !< 0x00000400
  ADC_SQR4_SQ9_1* = (0x00000002 shl ADC_SQR4_SQ9_Pos) ## !< 0x00000800
  ADC_SQR4_SQ9_2* = (0x00000004 shl ADC_SQR4_SQ9_Pos) ## !< 0x00001000
  ADC_SQR4_SQ9_3* = (0x00000008 shl ADC_SQR4_SQ9_Pos) ## !< 0x00002000
  ADC_SQR4_SQ9_4* = (0x00000010 shl ADC_SQR4_SQ9_Pos) ## !< 0x00004000
  ADC_SQR4_SQ10_Pos* = (15)
  ADC_SQR4_SQ10_Msk* = (0x0000001F shl ADC_SQR4_SQ10_Pos) ## !< 0x000F8000
  ADC_SQR4_SQ10* = ADC_SQR4_SQ10_Msk
  ADC_SQR4_SQ10_0* = (0x00000001 shl ADC_SQR4_SQ10_Pos) ## !< 0x00008000
  ADC_SQR4_SQ10_1* = (0x00000002 shl ADC_SQR4_SQ10_Pos) ## !< 0x00010000
  ADC_SQR4_SQ10_2* = (0x00000004 shl ADC_SQR4_SQ10_Pos) ## !< 0x00020000
  ADC_SQR4_SQ10_3* = (0x00000008 shl ADC_SQR4_SQ10_Pos) ## !< 0x00040000
  ADC_SQR4_SQ10_4* = (0x00000010 shl ADC_SQR4_SQ10_Pos) ## !< 0x00080000
  ADC_SQR4_SQ11_Pos* = (20)
  ADC_SQR4_SQ11_Msk* = (0x0000001F shl ADC_SQR4_SQ11_Pos) ## !< 0x01F00000
  ADC_SQR4_SQ11* = ADC_SQR4_SQ11_Msk
  ADC_SQR4_SQ11_0* = (0x00000001 shl ADC_SQR4_SQ11_Pos) ## !< 0x00100000
  ADC_SQR4_SQ11_1* = (0x00000002 shl ADC_SQR4_SQ11_Pos) ## !< 0x00200000
  ADC_SQR4_SQ11_2* = (0x00000004 shl ADC_SQR4_SQ11_Pos) ## !< 0x00400000
  ADC_SQR4_SQ11_3* = (0x00000008 shl ADC_SQR4_SQ11_Pos) ## !< 0x00800000
  ADC_SQR4_SQ11_4* = (0x00000010 shl ADC_SQR4_SQ11_Pos) ## !< 0x01000000
  ADC_SQR4_SQ12_Pos* = (25)
  ADC_SQR4_SQ12_Msk* = (0x0000001F shl ADC_SQR4_SQ12_Pos) ## !< 0x3E000000
  ADC_SQR4_SQ12* = ADC_SQR4_SQ12_Msk
  ADC_SQR4_SQ12_0* = (0x00000001 shl ADC_SQR4_SQ12_Pos) ## !< 0x02000000
  ADC_SQR4_SQ12_1* = (0x00000002 shl ADC_SQR4_SQ12_Pos) ## !< 0x04000000
  ADC_SQR4_SQ12_2* = (0x00000004 shl ADC_SQR4_SQ12_Pos) ## !< 0x08000000
  ADC_SQR4_SQ12_3* = (0x00000008 shl ADC_SQR4_SQ12_Pos) ## !< 0x10000000
  ADC_SQR4_SQ12_4* = (0x00000010 shl ADC_SQR4_SQ12_Pos) ## !< 0x20000000

## ******************  Bit definition for ADC_SQR5 register  ******************

const
  ADC_SQR5_SQ1_Pos* = (0)
  ADC_SQR5_SQ1_Msk* = (0x0000001F shl ADC_SQR5_SQ1_Pos) ## !< 0x0000001F
  ADC_SQR5_SQ1* = ADC_SQR5_SQ1_Msk
  ADC_SQR5_SQ1_0* = (0x00000001 shl ADC_SQR5_SQ1_Pos) ## !< 0x00000001
  ADC_SQR5_SQ1_1* = (0x00000002 shl ADC_SQR5_SQ1_Pos) ## !< 0x00000002
  ADC_SQR5_SQ1_2* = (0x00000004 shl ADC_SQR5_SQ1_Pos) ## !< 0x00000004
  ADC_SQR5_SQ1_3* = (0x00000008 shl ADC_SQR5_SQ1_Pos) ## !< 0x00000008
  ADC_SQR5_SQ1_4* = (0x00000010 shl ADC_SQR5_SQ1_Pos) ## !< 0x00000010
  ADC_SQR5_SQ2_Pos* = (5)
  ADC_SQR5_SQ2_Msk* = (0x0000001F shl ADC_SQR5_SQ2_Pos) ## !< 0x000003E0
  ADC_SQR5_SQ2* = ADC_SQR5_SQ2_Msk
  ADC_SQR5_SQ2_0* = (0x00000001 shl ADC_SQR5_SQ2_Pos) ## !< 0x00000020
  ADC_SQR5_SQ2_1* = (0x00000002 shl ADC_SQR5_SQ2_Pos) ## !< 0x00000040
  ADC_SQR5_SQ2_2* = (0x00000004 shl ADC_SQR5_SQ2_Pos) ## !< 0x00000080
  ADC_SQR5_SQ2_3* = (0x00000008 shl ADC_SQR5_SQ2_Pos) ## !< 0x00000100
  ADC_SQR5_SQ2_4* = (0x00000010 shl ADC_SQR5_SQ2_Pos) ## !< 0x00000200
  ADC_SQR5_SQ3_Pos* = (10)
  ADC_SQR5_SQ3_Msk* = (0x0000001F shl ADC_SQR5_SQ3_Pos) ## !< 0x00007C00
  ADC_SQR5_SQ3* = ADC_SQR5_SQ3_Msk
  ADC_SQR5_SQ3_0* = (0x00000001 shl ADC_SQR5_SQ3_Pos) ## !< 0x00000400
  ADC_SQR5_SQ3_1* = (0x00000002 shl ADC_SQR5_SQ3_Pos) ## !< 0x00000800
  ADC_SQR5_SQ3_2* = (0x00000004 shl ADC_SQR5_SQ3_Pos) ## !< 0x00001000
  ADC_SQR5_SQ3_3* = (0x00000008 shl ADC_SQR5_SQ3_Pos) ## !< 0x00002000
  ADC_SQR5_SQ3_4* = (0x00000010 shl ADC_SQR5_SQ3_Pos) ## !< 0x00004000
  ADC_SQR5_SQ4_Pos* = (15)
  ADC_SQR5_SQ4_Msk* = (0x0000001F shl ADC_SQR5_SQ4_Pos) ## !< 0x000F8000
  ADC_SQR5_SQ4* = ADC_SQR5_SQ4_Msk
  ADC_SQR5_SQ4_0* = (0x00000001 shl ADC_SQR5_SQ4_Pos) ## !< 0x00008000
  ADC_SQR5_SQ4_1* = (0x00000002 shl ADC_SQR5_SQ4_Pos) ## !< 0x00010000
  ADC_SQR5_SQ4_2* = (0x00000004 shl ADC_SQR5_SQ4_Pos) ## !< 0x00020000
  ADC_SQR5_SQ4_3* = (0x00000008 shl ADC_SQR5_SQ4_Pos) ## !< 0x00040000
  ADC_SQR5_SQ4_4* = (0x00000010 shl ADC_SQR5_SQ4_Pos) ## !< 0x00080000
  ADC_SQR5_SQ5_Pos* = (20)
  ADC_SQR5_SQ5_Msk* = (0x0000001F shl ADC_SQR5_SQ5_Pos) ## !< 0x01F00000
  ADC_SQR5_SQ5* = ADC_SQR5_SQ5_Msk
  ADC_SQR5_SQ5_0* = (0x00000001 shl ADC_SQR5_SQ5_Pos) ## !< 0x00100000
  ADC_SQR5_SQ5_1* = (0x00000002 shl ADC_SQR5_SQ5_Pos) ## !< 0x00200000
  ADC_SQR5_SQ5_2* = (0x00000004 shl ADC_SQR5_SQ5_Pos) ## !< 0x00400000
  ADC_SQR5_SQ5_3* = (0x00000008 shl ADC_SQR5_SQ5_Pos) ## !< 0x00800000
  ADC_SQR5_SQ5_4* = (0x00000010 shl ADC_SQR5_SQ5_Pos) ## !< 0x01000000
  ADC_SQR5_SQ6_Pos* = (25)
  ADC_SQR5_SQ6_Msk* = (0x0000001F shl ADC_SQR5_SQ6_Pos) ## !< 0x3E000000
  ADC_SQR5_SQ6* = ADC_SQR5_SQ6_Msk
  ADC_SQR5_SQ6_0* = (0x00000001 shl ADC_SQR5_SQ6_Pos) ## !< 0x02000000
  ADC_SQR5_SQ6_1* = (0x00000002 shl ADC_SQR5_SQ6_Pos) ## !< 0x04000000
  ADC_SQR5_SQ6_2* = (0x00000004 shl ADC_SQR5_SQ6_Pos) ## !< 0x08000000
  ADC_SQR5_SQ6_3* = (0x00000008 shl ADC_SQR5_SQ6_Pos) ## !< 0x10000000
  ADC_SQR5_SQ6_4* = (0x00000010 shl ADC_SQR5_SQ6_Pos) ## !< 0x20000000

## ******************  Bit definition for ADC_JSQR register  ******************

const
  ADC_JSQR_JSQ1_Pos* = (0)
  ADC_JSQR_JSQ1_Msk* = (0x0000001F shl ADC_JSQR_JSQ1_Pos) ## !< 0x0000001F
  ADC_JSQR_JSQ1* = ADC_JSQR_JSQ1_Msk
  ADC_JSQR_JSQ1_0* = (0x00000001 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00000001
  ADC_JSQR_JSQ1_1* = (0x00000002 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00000002
  ADC_JSQR_JSQ1_2* = (0x00000004 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00000004
  ADC_JSQR_JSQ1_3* = (0x00000008 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00000008
  ADC_JSQR_JSQ1_4* = (0x00000010 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00000010
  ADC_JSQR_JSQ2_Pos* = (5)
  ADC_JSQR_JSQ2_Msk* = (0x0000001F shl ADC_JSQR_JSQ2_Pos) ## !< 0x000003E0
  ADC_JSQR_JSQ2* = ADC_JSQR_JSQ2_Msk
  ADC_JSQR_JSQ2_0* = (0x00000001 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00000020
  ADC_JSQR_JSQ2_1* = (0x00000002 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00000040
  ADC_JSQR_JSQ2_2* = (0x00000004 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00000080
  ADC_JSQR_JSQ2_3* = (0x00000008 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00000100
  ADC_JSQR_JSQ2_4* = (0x00000010 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00000200
  ADC_JSQR_JSQ3_Pos* = (10)
  ADC_JSQR_JSQ3_Msk* = (0x0000001F shl ADC_JSQR_JSQ3_Pos) ## !< 0x00007C00
  ADC_JSQR_JSQ3* = ADC_JSQR_JSQ3_Msk
  ADC_JSQR_JSQ3_0* = (0x00000001 shl ADC_JSQR_JSQ3_Pos) ## !< 0x00000400
  ADC_JSQR_JSQ3_1* = (0x00000002 shl ADC_JSQR_JSQ3_Pos) ## !< 0x00000800
  ADC_JSQR_JSQ3_2* = (0x00000004 shl ADC_JSQR_JSQ3_Pos) ## !< 0x00001000
  ADC_JSQR_JSQ3_3* = (0x00000008 shl ADC_JSQR_JSQ3_Pos) ## !< 0x00002000
  ADC_JSQR_JSQ3_4* = (0x00000010 shl ADC_JSQR_JSQ3_Pos) ## !< 0x00004000
  ADC_JSQR_JSQ4_Pos* = (15)
  ADC_JSQR_JSQ4_Msk* = (0x0000001F shl ADC_JSQR_JSQ4_Pos) ## !< 0x000F8000
  ADC_JSQR_JSQ4* = ADC_JSQR_JSQ4_Msk
  ADC_JSQR_JSQ4_0* = (0x00000001 shl ADC_JSQR_JSQ4_Pos) ## !< 0x00008000
  ADC_JSQR_JSQ4_1* = (0x00000002 shl ADC_JSQR_JSQ4_Pos) ## !< 0x00010000
  ADC_JSQR_JSQ4_2* = (0x00000004 shl ADC_JSQR_JSQ4_Pos) ## !< 0x00020000
  ADC_JSQR_JSQ4_3* = (0x00000008 shl ADC_JSQR_JSQ4_Pos) ## !< 0x00040000
  ADC_JSQR_JSQ4_4* = (0x00000010 shl ADC_JSQR_JSQ4_Pos) ## !< 0x00080000
  ADC_JSQR_JL_Pos* = (20)
  ADC_JSQR_JL_Msk* = (0x00000003 shl ADC_JSQR_JL_Pos) ## !< 0x00300000
  ADC_JSQR_JL* = ADC_JSQR_JL_Msk
  ADC_JSQR_JL_0* = (0x00000001 shl ADC_JSQR_JL_Pos) ## !< 0x00100000
  ADC_JSQR_JL_1* = (0x00000002 shl ADC_JSQR_JL_Pos) ## !< 0x00200000

## ******************  Bit definition for ADC_JDR1 register  ******************

const
  ADC_JDR1_JDATA_Pos* = (0)
  ADC_JDR1_JDATA_Msk* = (0x0000FFFF shl ADC_JDR1_JDATA_Pos) ## !< 0x0000FFFF
  ADC_JDR1_JDATA* = ADC_JDR1_JDATA_Msk

## ******************  Bit definition for ADC_JDR2 register  ******************

const
  ADC_JDR2_JDATA_Pos* = (0)
  ADC_JDR2_JDATA_Msk* = (0x0000FFFF shl ADC_JDR2_JDATA_Pos) ## !< 0x0000FFFF
  ADC_JDR2_JDATA* = ADC_JDR2_JDATA_Msk

## ******************  Bit definition for ADC_JDR3 register  ******************

const
  ADC_JDR3_JDATA_Pos* = (0)
  ADC_JDR3_JDATA_Msk* = (0x0000FFFF shl ADC_JDR3_JDATA_Pos) ## !< 0x0000FFFF
  ADC_JDR3_JDATA* = ADC_JDR3_JDATA_Msk

## ******************  Bit definition for ADC_JDR4 register  ******************

const
  ADC_JDR4_JDATA_Pos* = (0)
  ADC_JDR4_JDATA_Msk* = (0x0000FFFF shl ADC_JDR4_JDATA_Pos) ## !< 0x0000FFFF
  ADC_JDR4_JDATA* = ADC_JDR4_JDATA_Msk

## *******************  Bit definition for ADC_DR register  *******************

const
  ADC_DR_DATA_Pos* = (0)
  ADC_DR_DATA_Msk* = (0x0000FFFF shl ADC_DR_DATA_Pos) ## !< 0x0000FFFF
  ADC_DR_DATA* = ADC_DR_DATA_Msk

## *****************  Bit definition for ADC_SMPR0 register  ******************

const
  ADC_SMPR0_SMP30_Pos* = (0)
  ADC_SMPR0_SMP30_Msk* = (0x00000007 shl ADC_SMPR0_SMP30_Pos) ## !< 0x00000007
  ADC_SMPR0_SMP30* = ADC_SMPR0_SMP30_Msk
  ADC_SMPR0_SMP30_0* = (0x00000001 shl ADC_SMPR0_SMP30_Pos) ## !< 0x00000001
  ADC_SMPR0_SMP30_1* = (0x00000002 shl ADC_SMPR0_SMP30_Pos) ## !< 0x00000002
  ADC_SMPR0_SMP30_2* = (0x00000004 shl ADC_SMPR0_SMP30_Pos) ## !< 0x00000004
  ADC_SMPR0_SMP31_Pos* = (3)
  ADC_SMPR0_SMP31_Msk* = (0x00000007 shl ADC_SMPR0_SMP31_Pos) ## !< 0x00000038
  ADC_SMPR0_SMP31* = ADC_SMPR0_SMP31_Msk
  ADC_SMPR0_SMP31_0* = (0x00000001 shl ADC_SMPR0_SMP31_Pos) ## !< 0x00000008
  ADC_SMPR0_SMP31_1* = (0x00000002 shl ADC_SMPR0_SMP31_Pos) ## !< 0x00000010
  ADC_SMPR0_SMP31_2* = (0x00000004 shl ADC_SMPR0_SMP31_Pos) ## !< 0x00000020

## ******************  Bit definition for ADC_CSR register  *******************

const
  ADC_CSR_AWD1_Pos* = (0)
  ADC_CSR_AWD1_Msk* = (0x00000001 shl ADC_CSR_AWD1_Pos) ## !< 0x00000001
  ADC_CSR_AWD1* = ADC_CSR_AWD1_Msk
  ADC_CSR_EOCS1_Pos* = (1)
  ADC_CSR_EOCS1_Msk* = (0x00000001 shl ADC_CSR_EOCS1_Pos) ## !< 0x00000002
  ADC_CSR_EOCS1* = ADC_CSR_EOCS1_Msk
  ADC_CSR_JEOS1_Pos* = (2)
  ADC_CSR_JEOS1_Msk* = (0x00000001 shl ADC_CSR_JEOS1_Pos) ## !< 0x00000004
  ADC_CSR_JEOS1* = ADC_CSR_JEOS1_Msk
  ADC_CSR_JSTRT1_Pos* = (3)
  ADC_CSR_JSTRT1_Msk* = (0x00000001 shl ADC_CSR_JSTRT1_Pos) ## !< 0x00000008
  ADC_CSR_JSTRT1* = ADC_CSR_JSTRT1_Msk
  ADC_CSR_STRT1_Pos* = (4)
  ADC_CSR_STRT1_Msk* = (0x00000001 shl ADC_CSR_STRT1_Pos) ## !< 0x00000010
  ADC_CSR_STRT1* = ADC_CSR_STRT1_Msk
  ADC_CSR_OVR1_Pos* = (5)
  ADC_CSR_OVR1_Msk* = (0x00000001 shl ADC_CSR_OVR1_Pos) ## !< 0x00000020
  ADC_CSR_OVR1* = ADC_CSR_OVR1_Msk
  ADC_CSR_ADONS1_Pos* = (6)
  ADC_CSR_ADONS1_Msk* = (0x00000001 shl ADC_CSR_ADONS1_Pos) ## !< 0x00000040
  ADC_CSR_ADONS1* = ADC_CSR_ADONS1_Msk

##  Legacy defines

const
  ADC_CSR_EOC1* = (ADC_CSR_EOCS1)
  ADC_CSR_JEOC1* = (ADC_CSR_JEOS1)

## ******************  Bit definition for ADC_CCR register  *******************

const
  ADC_CCR_ADCPRE_Pos* = (16)
  ADC_CCR_ADCPRE_Msk* = (0x00000003 shl ADC_CCR_ADCPRE_Pos) ## !< 0x00030000
  ADC_CCR_ADCPRE* = ADC_CCR_ADCPRE_Msk
  ADC_CCR_ADCPRE_0* = (0x00000001 shl ADC_CCR_ADCPRE_Pos) ## !< 0x00010000
  ADC_CCR_ADCPRE_1* = (0x00000002 shl ADC_CCR_ADCPRE_Pos) ## !< 0x00020000
  ADC_CCR_TSVREFE_Pos* = (23)
  ADC_CCR_TSVREFE_Msk* = (0x00000001 shl ADC_CCR_TSVREFE_Pos) ## !< 0x00800000
  ADC_CCR_TSVREFE* = ADC_CCR_TSVREFE_Msk

## ****************************************************************************
##
##                       Analog Comparators (COMP)
##
## ****************************************************************************
## *****************  Bit definition for COMP_CSR register  *******************

const
  COMP_CSR_10KPU* = (0x00000001) ## !< Comparator 1 input plus 10K pull-up resistor
  COMP_CSR_400KPU* = (0x00000002) ## !< Comparator 1 input plus 400K pull-up resistor
  COMP_CSR_10KPD* = (0x00000004) ## !< Comparator 1 input plus 10K pull-down resistor
  COMP_CSR_400KPD* = (0x00000008) ## !< Comparator 1 input plus 400K pull-down resistor
  COMP_CSR_CMP1EN_Pos* = (4)
  COMP_CSR_CMP1EN_Msk* = (0x00000001 shl COMP_CSR_CMP1EN_Pos) ## !< 0x00000010
  COMP_CSR_CMP1EN* = COMP_CSR_CMP1EN_Msk
  COMP_CSR_CMP1OUT_Pos* = (7)
  COMP_CSR_CMP1OUT_Msk* = (0x00000001 shl COMP_CSR_CMP1OUT_Pos) ## !< 0x00000080
  COMP_CSR_CMP1OUT* = COMP_CSR_CMP1OUT_Msk
  COMP_CSR_SPEED_Pos* = (12)
  COMP_CSR_SPEED_Msk* = (0x00000001 shl COMP_CSR_SPEED_Pos) ## !< 0x00001000
  COMP_CSR_SPEED* = COMP_CSR_SPEED_Msk
  COMP_CSR_CMP2OUT_Pos* = (13)
  COMP_CSR_CMP2OUT_Msk* = (0x00000001 shl COMP_CSR_CMP2OUT_Pos) ## !< 0x00002000
  COMP_CSR_CMP2OUT* = COMP_CSR_CMP2OUT_Msk
  COMP_CSR_WNDWE_Pos* = (17)
  COMP_CSR_WNDWE_Msk* = (0x00000001 shl COMP_CSR_WNDWE_Pos) ## !< 0x00020000
  COMP_CSR_WNDWE* = COMP_CSR_WNDWE_Msk
  COMP_CSR_INSEL_Pos* = (18)
  COMP_CSR_INSEL_Msk* = (0x00000007 shl COMP_CSR_INSEL_Pos) ## !< 0x001C0000
  COMP_CSR_INSEL* = COMP_CSR_INSEL_Msk
  COMP_CSR_INSEL_0* = (0x00000001 shl COMP_CSR_INSEL_Pos) ## !< 0x00040000
  COMP_CSR_INSEL_1* = (0x00000002 shl COMP_CSR_INSEL_Pos) ## !< 0x00080000
  COMP_CSR_INSEL_2* = (0x00000004 shl COMP_CSR_INSEL_Pos) ## !< 0x00100000
  COMP_CSR_OUTSEL_Pos* = (21)
  COMP_CSR_OUTSEL_Msk* = (0x00000007 shl COMP_CSR_OUTSEL_Pos) ## !< 0x00E00000
  COMP_CSR_OUTSEL* = COMP_CSR_OUTSEL_Msk
  COMP_CSR_OUTSEL_0* = (0x00000001 shl COMP_CSR_OUTSEL_Pos) ## !< 0x00200000
  COMP_CSR_OUTSEL_1* = (0x00000002 shl COMP_CSR_OUTSEL_Pos) ## !< 0x00400000
  COMP_CSR_OUTSEL_2* = (0x00000004 shl COMP_CSR_OUTSEL_Pos) ## !< 0x00800000

##  Bits present in COMP register but not related to comparator
##  (or partially related to comparator, in addition to other peripherals)

const
  COMP_CSR_SW1_Pos* = (5)
  COMP_CSR_SW1_Msk* = (0x00000001 shl COMP_CSR_SW1_Pos) ## !< 0x00000020
  COMP_CSR_SW1* = COMP_CSR_SW1_Msk
  COMP_CSR_VREFOUTEN_Pos* = (16)
  COMP_CSR_VREFOUTEN_Msk* = (0x00000001 shl COMP_CSR_VREFOUTEN_Pos) ## !< 0x00010000
  COMP_CSR_VREFOUTEN* = COMP_CSR_VREFOUTEN_Msk
  COMP_CSR_FCH3_Pos* = (26)
  COMP_CSR_FCH3_Msk* = (0x00000001 shl COMP_CSR_FCH3_Pos) ## !< 0x04000000
  COMP_CSR_FCH3* = COMP_CSR_FCH3_Msk
  COMP_CSR_FCH8_Pos* = (27)
  COMP_CSR_FCH8_Msk* = (0x00000001 shl COMP_CSR_FCH8_Pos) ## !< 0x08000000
  COMP_CSR_FCH8* = COMP_CSR_FCH8_Msk
  COMP_CSR_RCH13_Pos* = (28)
  COMP_CSR_RCH13_Msk* = (0x00000001 shl COMP_CSR_RCH13_Pos) ## !< 0x10000000
  COMP_CSR_RCH13* = COMP_CSR_RCH13_Msk
  COMP_CSR_CAIE_Pos* = (29)
  COMP_CSR_CAIE_Msk* = (0x00000001 shl COMP_CSR_CAIE_Pos) ## !< 0x20000000
  COMP_CSR_CAIE* = COMP_CSR_CAIE_Msk
  COMP_CSR_CAIF_Pos* = (30)
  COMP_CSR_CAIF_Msk* = (0x00000001 shl COMP_CSR_CAIF_Pos) ## !< 0x40000000
  COMP_CSR_CAIF* = COMP_CSR_CAIF_Msk
  COMP_CSR_TSUSP_Pos* = (31)
  COMP_CSR_TSUSP_Msk* = (0x00000001 shl COMP_CSR_TSUSP_Pos) ## !< 0x80000000
  COMP_CSR_TSUSP* = COMP_CSR_TSUSP_Msk

## ****************************************************************************
##
##                          Operational Amplifier (OPAMP)
##
## ****************************************************************************
## ******************  Bit definition for OPAMP_CSR register  *****************

const
  OPAMP_CSR_OPA1PD_Pos* = (0)
  OPAMP_CSR_OPA1PD_Msk* = (0x00000001 shl OPAMP_CSR_OPA1PD_Pos) ## !< 0x00000001
  OPAMP_CSR_OPA1PD* = OPAMP_CSR_OPA1PD_Msk
  OPAMP_CSR_S3SEL1_Pos* = (1)
  OPAMP_CSR_S3SEL1_Msk* = (0x00000001 shl OPAMP_CSR_S3SEL1_Pos) ## !< 0x00000002
  OPAMP_CSR_S3SEL1* = OPAMP_CSR_S3SEL1_Msk
  OPAMP_CSR_S4SEL1_Pos* = (2)
  OPAMP_CSR_S4SEL1_Msk* = (0x00000001 shl OPAMP_CSR_S4SEL1_Pos) ## !< 0x00000004
  OPAMP_CSR_S4SEL1* = OPAMP_CSR_S4SEL1_Msk
  OPAMP_CSR_S5SEL1_Pos* = (3)
  OPAMP_CSR_S5SEL1_Msk* = (0x00000001 shl OPAMP_CSR_S5SEL1_Pos) ## !< 0x00000008
  OPAMP_CSR_S5SEL1* = OPAMP_CSR_S5SEL1_Msk
  OPAMP_CSR_S6SEL1_Pos* = (4)
  OPAMP_CSR_S6SEL1_Msk* = (0x00000001 shl OPAMP_CSR_S6SEL1_Pos) ## !< 0x00000010
  OPAMP_CSR_S6SEL1* = OPAMP_CSR_S6SEL1_Msk
  OPAMP_CSR_OPA1CAL_L_Pos* = (5)
  OPAMP_CSR_OPA1CAL_L_Msk* = (0x00000001 shl OPAMP_CSR_OPA1CAL_L_Pos) ## !< 0x00000020
  OPAMP_CSR_OPA1CAL_L* = OPAMP_CSR_OPA1CAL_L_Msk
  OPAMP_CSR_OPA1CAL_H_Pos* = (6)
  OPAMP_CSR_OPA1CAL_H_Msk* = (0x00000001 shl OPAMP_CSR_OPA1CAL_H_Pos) ## !< 0x00000040
  OPAMP_CSR_OPA1CAL_H* = OPAMP_CSR_OPA1CAL_H_Msk
  OPAMP_CSR_OPA1LPM_Pos* = (7)
  OPAMP_CSR_OPA1LPM_Msk* = (0x00000001 shl OPAMP_CSR_OPA1LPM_Pos) ## !< 0x00000080
  OPAMP_CSR_OPA1LPM* = OPAMP_CSR_OPA1LPM_Msk
  OPAMP_CSR_OPA2PD_Pos* = (8)
  OPAMP_CSR_OPA2PD_Msk* = (0x00000001 shl OPAMP_CSR_OPA2PD_Pos) ## !< 0x00000100
  OPAMP_CSR_OPA2PD* = OPAMP_CSR_OPA2PD_Msk
  OPAMP_CSR_S3SEL2_Pos* = (9)
  OPAMP_CSR_S3SEL2_Msk* = (0x00000001 shl OPAMP_CSR_S3SEL2_Pos) ## !< 0x00000200
  OPAMP_CSR_S3SEL2* = OPAMP_CSR_S3SEL2_Msk
  OPAMP_CSR_S4SEL2_Pos* = (10)
  OPAMP_CSR_S4SEL2_Msk* = (0x00000001 shl OPAMP_CSR_S4SEL2_Pos) ## !< 0x00000400
  OPAMP_CSR_S4SEL2* = OPAMP_CSR_S4SEL2_Msk
  OPAMP_CSR_S5SEL2_Pos* = (11)
  OPAMP_CSR_S5SEL2_Msk* = (0x00000001 shl OPAMP_CSR_S5SEL2_Pos) ## !< 0x00000800
  OPAMP_CSR_S5SEL2* = OPAMP_CSR_S5SEL2_Msk
  OPAMP_CSR_S6SEL2_Pos* = (12)
  OPAMP_CSR_S6SEL2_Msk* = (0x00000001 shl OPAMP_CSR_S6SEL2_Pos) ## !< 0x00001000
  OPAMP_CSR_S6SEL2* = OPAMP_CSR_S6SEL2_Msk
  OPAMP_CSR_OPA2CAL_L_Pos* = (13)
  OPAMP_CSR_OPA2CAL_L_Msk* = (0x00000001 shl OPAMP_CSR_OPA2CAL_L_Pos) ## !< 0x00002000
  OPAMP_CSR_OPA2CAL_L* = OPAMP_CSR_OPA2CAL_L_Msk
  OPAMP_CSR_OPA2CAL_H_Pos* = (14)
  OPAMP_CSR_OPA2CAL_H_Msk* = (0x00000001 shl OPAMP_CSR_OPA2CAL_H_Pos) ## !< 0x00004000
  OPAMP_CSR_OPA2CAL_H* = OPAMP_CSR_OPA2CAL_H_Msk
  OPAMP_CSR_OPA2LPM_Pos* = (15)
  OPAMP_CSR_OPA2LPM_Msk* = (0x00000001 shl OPAMP_CSR_OPA2LPM_Pos) ## !< 0x00008000
  OPAMP_CSR_OPA2LPM* = OPAMP_CSR_OPA2LPM_Msk
  OPAMP_CSR_ANAWSEL1_Pos* = (24)
  OPAMP_CSR_ANAWSEL1_Msk* = (0x00000001 shl OPAMP_CSR_ANAWSEL1_Pos) ## !< 0x01000000
  OPAMP_CSR_ANAWSEL1* = OPAMP_CSR_ANAWSEL1_Msk
  OPAMP_CSR_ANAWSEL2_Pos* = (25)
  OPAMP_CSR_ANAWSEL2_Msk* = (0x00000001 shl OPAMP_CSR_ANAWSEL2_Pos) ## !< 0x02000000
  OPAMP_CSR_ANAWSEL2* = OPAMP_CSR_ANAWSEL2_Msk
  OPAMP_CSR_S7SEL2_Pos* = (27)
  OPAMP_CSR_S7SEL2_Msk* = (0x00000001 shl OPAMP_CSR_S7SEL2_Pos) ## !< 0x08000000
  OPAMP_CSR_S7SEL2* = OPAMP_CSR_S7SEL2_Msk
  OPAMP_CSR_AOP_RANGE_Pos* = (28)
  OPAMP_CSR_AOP_RANGE_Msk* = (0x00000001 shl OPAMP_CSR_AOP_RANGE_Pos) ## !< 0x10000000
  OPAMP_CSR_AOP_RANGE* = OPAMP_CSR_AOP_RANGE_Msk
  OPAMP_CSR_OPA1CALOUT_Pos* = (29)
  OPAMP_CSR_OPA1CALOUT_Msk* = (0x00000001 shl OPAMP_CSR_OPA1CALOUT_Pos) ## !< 0x20000000
  OPAMP_CSR_OPA1CALOUT* = OPAMP_CSR_OPA1CALOUT_Msk
  OPAMP_CSR_OPA2CALOUT_Pos* = (30)
  OPAMP_CSR_OPA2CALOUT_Msk* = (0x00000001 shl OPAMP_CSR_OPA2CALOUT_Pos) ## !< 0x40000000
  OPAMP_CSR_OPA2CALOUT* = OPAMP_CSR_OPA2CALOUT_Msk

## ******************  Bit definition for OPAMP_OTR register  *****************

const
  OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LOW_Pos* = (0)
  OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LOW_Msk* = (
    0x0000001F shl OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LOW_Pos) ## !< 0x0000001F
  OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LOW* = OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LOW_Msk
  OPAMP_OTR_AO1_OPT_OFFSET_TRIM_HIGH_Pos* = (5)
  OPAMP_OTR_AO1_OPT_OFFSET_TRIM_HIGH_Msk* = (
    0x0000001F shl OPAMP_OTR_AO1_OPT_OFFSET_TRIM_HIGH_Pos) ## !< 0x000003E0
  OPAMP_OTR_AO1_OPT_OFFSET_TRIM_HIGH* = OPAMP_OTR_AO1_OPT_OFFSET_TRIM_HIGH_Msk
  OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LOW_Pos* = (10)
  OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LOW_Msk* = (
    0x0000001F shl OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LOW_Pos) ## !< 0x00007C00
  OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LOW* = OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LOW_Msk
  OPAMP_OTR_AO2_OPT_OFFSET_TRIM_HIGH_Pos* = (15)
  OPAMP_OTR_AO2_OPT_OFFSET_TRIM_HIGH_Msk* = (
    0x0000001F shl OPAMP_OTR_AO2_OPT_OFFSET_TRIM_HIGH_Pos) ## !< 0x000F8000
  OPAMP_OTR_AO2_OPT_OFFSET_TRIM_HIGH* = OPAMP_OTR_AO2_OPT_OFFSET_TRIM_HIGH_Msk
  OPAMP_OTR_OT_USER_Pos* = (31)
  OPAMP_OTR_OT_USER_Msk* = (0x00000001 shl OPAMP_OTR_OT_USER_Pos) ## !< 0x80000000
  OPAMP_OTR_OT_USER* = OPAMP_OTR_OT_USER_Msk

## ******************  Bit definition for OPAMP_LPOTR register  ***************

const
  OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LP_LOW_Pos* = (0)
  OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LP_LOW_Msk* = (
    0x0000001F shl OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LP_LOW_Pos) ## !< 0x0000001F
  OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LP_LOW* = OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LP_LOW_Msk
  OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LP_HIGH_Pos* = (5)
  OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LP_HIGH_Msk* = (
    0x0000001F shl OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LP_HIGH_Pos) ## !< 0x000003E0
  OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LP_HIGH* = OPAMP_OTR_AO1_OPT_OFFSET_TRIM_LP_HIGH_Msk
  OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LP_LOW_Pos* = (10)
  OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LP_LOW_Msk* = (
    0x0000001F shl OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LP_LOW_Pos) ## !< 0x00007C00
  OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LP_LOW* = OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LP_LOW_Msk
  OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LP_HIGH_Pos* = (15)
  OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LP_HIGH_Msk* = (
    0x0000001F shl OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LP_HIGH_Pos) ## !< 0x000F8000
  OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LP_HIGH* = OPAMP_OTR_AO2_OPT_OFFSET_TRIM_LP_HIGH_Msk

## ****************************************************************************
##
##                        CRC calculation unit (CRC)
##
## ****************************************************************************
## ******************  Bit definition for CRC_DR register  ********************

const
  CRC_DR_DR_Pos* = (0)
  CRC_DR_DR_Msk* = (0xFFFFFFFF shl CRC_DR_DR_Pos) ## !< 0xFFFFFFFF
  CRC_DR_DR* = CRC_DR_DR_Msk

## ******************  Bit definition for CRC_IDR register  *******************

const
  CRC_IDR_IDR_Pos* = (0)
  CRC_IDR_IDR_Msk* = (0x000000FF shl CRC_IDR_IDR_Pos) ## !< 0x000000FF
  CRC_IDR_IDR* = CRC_IDR_IDR_Msk

## *******************  Bit definition for CRC_CR register  *******************

const
  CRC_CR_RESET_Pos* = (0)
  CRC_CR_RESET_Msk* = (0x00000001 shl CRC_CR_RESET_Pos) ## !< 0x00000001
  CRC_CR_RESET* = CRC_CR_RESET_Msk

## ****************************************************************************
##
##                     Digital to Analog Converter (DAC)
##
## ****************************************************************************
## *******************  Bit definition for DAC_CR register  *******************

const
  DAC_CR_EN1_Pos* = (0)
  DAC_CR_EN1_Msk* = (0x00000001 shl DAC_CR_EN1_Pos) ## !< 0x00000001
  DAC_CR_EN1* = DAC_CR_EN1_Msk
  DAC_CR_BOFF1_Pos* = (1)
  DAC_CR_BOFF1_Msk* = (0x00000001 shl DAC_CR_BOFF1_Pos) ## !< 0x00000002
  DAC_CR_BOFF1* = DAC_CR_BOFF1_Msk
  DAC_CR_TEN1_Pos* = (2)
  DAC_CR_TEN1_Msk* = (0x00000001 shl DAC_CR_TEN1_Pos) ## !< 0x00000004
  DAC_CR_TEN1* = DAC_CR_TEN1_Msk
  DAC_CR_TSEL1_Pos* = (3)
  DAC_CR_TSEL1_Msk* = (0x00000007 shl DAC_CR_TSEL1_Pos) ## !< 0x00000038
  DAC_CR_TSEL1* = DAC_CR_TSEL1_Msk
  DAC_CR_TSEL1_0* = (0x00000001 shl DAC_CR_TSEL1_Pos) ## !< 0x00000008
  DAC_CR_TSEL1_1* = (0x00000002 shl DAC_CR_TSEL1_Pos) ## !< 0x00000010
  DAC_CR_TSEL1_2* = (0x00000004 shl DAC_CR_TSEL1_Pos) ## !< 0x00000020
  DAC_CR_WAVE1_Pos* = (6)
  DAC_CR_WAVE1_Msk* = (0x00000003 shl DAC_CR_WAVE1_Pos) ## !< 0x000000C0
  DAC_CR_WAVE1* = DAC_CR_WAVE1_Msk
  DAC_CR_WAVE1_0* = (0x00000001 shl DAC_CR_WAVE1_Pos) ## !< 0x00000040
  DAC_CR_WAVE1_1* = (0x00000002 shl DAC_CR_WAVE1_Pos) ## !< 0x00000080
  DAC_CR_MAMP1_Pos* = (8)
  DAC_CR_MAMP1_Msk* = (0x0000000F shl DAC_CR_MAMP1_Pos) ## !< 0x00000F00
  DAC_CR_MAMP1* = DAC_CR_MAMP1_Msk
  DAC_CR_MAMP1_0* = (0x00000001 shl DAC_CR_MAMP1_Pos) ## !< 0x00000100
  DAC_CR_MAMP1_1* = (0x00000002 shl DAC_CR_MAMP1_Pos) ## !< 0x00000200
  DAC_CR_MAMP1_2* = (0x00000004 shl DAC_CR_MAMP1_Pos) ## !< 0x00000400
  DAC_CR_MAMP1_3* = (0x00000008 shl DAC_CR_MAMP1_Pos) ## !< 0x00000800
  DAC_CR_DMAEN1_Pos* = (12)
  DAC_CR_DMAEN1_Msk* = (0x00000001 shl DAC_CR_DMAEN1_Pos) ## !< 0x00001000
  DAC_CR_DMAEN1* = DAC_CR_DMAEN1_Msk
  DAC_CR_DMAUDRIE1_Pos* = (13)
  DAC_CR_DMAUDRIE1_Msk* = (0x00000001 shl DAC_CR_DMAUDRIE1_Pos) ## !< 0x00002000
  DAC_CR_DMAUDRIE1* = DAC_CR_DMAUDRIE1_Msk
  DAC_CR_EN2_Pos* = (16)
  DAC_CR_EN2_Msk* = (0x00000001 shl DAC_CR_EN2_Pos) ## !< 0x00010000
  DAC_CR_EN2* = DAC_CR_EN2_Msk
  DAC_CR_BOFF2_Pos* = (17)
  DAC_CR_BOFF2_Msk* = (0x00000001 shl DAC_CR_BOFF2_Pos) ## !< 0x00020000
  DAC_CR_BOFF2* = DAC_CR_BOFF2_Msk
  DAC_CR_TEN2_Pos* = (18)
  DAC_CR_TEN2_Msk* = (0x00000001 shl DAC_CR_TEN2_Pos) ## !< 0x00040000
  DAC_CR_TEN2* = DAC_CR_TEN2_Msk
  DAC_CR_TSEL2_Pos* = (19)
  DAC_CR_TSEL2_Msk* = (0x00000007 shl DAC_CR_TSEL2_Pos) ## !< 0x00380000
  DAC_CR_TSEL2* = DAC_CR_TSEL2_Msk
  DAC_CR_TSEL2_0* = (0x00000001 shl DAC_CR_TSEL2_Pos) ## !< 0x00080000
  DAC_CR_TSEL2_1* = (0x00000002 shl DAC_CR_TSEL2_Pos) ## !< 0x00100000
  DAC_CR_TSEL2_2* = (0x00000004 shl DAC_CR_TSEL2_Pos) ## !< 0x00200000
  DAC_CR_WAVE2_Pos* = (22)
  DAC_CR_WAVE2_Msk* = (0x00000003 shl DAC_CR_WAVE2_Pos) ## !< 0x00C00000
  DAC_CR_WAVE2* = DAC_CR_WAVE2_Msk
  DAC_CR_WAVE2_0* = (0x00000001 shl DAC_CR_WAVE2_Pos) ## !< 0x00400000
  DAC_CR_WAVE2_1* = (0x00000002 shl DAC_CR_WAVE2_Pos) ## !< 0x00800000
  DAC_CR_MAMP2_Pos* = (24)
  DAC_CR_MAMP2_Msk* = (0x0000000F shl DAC_CR_MAMP2_Pos) ## !< 0x0F000000
  DAC_CR_MAMP2* = DAC_CR_MAMP2_Msk
  DAC_CR_MAMP2_0* = (0x00000001 shl DAC_CR_MAMP2_Pos) ## !< 0x01000000
  DAC_CR_MAMP2_1* = (0x00000002 shl DAC_CR_MAMP2_Pos) ## !< 0x02000000
  DAC_CR_MAMP2_2* = (0x00000004 shl DAC_CR_MAMP2_Pos) ## !< 0x04000000
  DAC_CR_MAMP2_3* = (0x00000008 shl DAC_CR_MAMP2_Pos) ## !< 0x08000000
  DAC_CR_DMAEN2_Pos* = (28)
  DAC_CR_DMAEN2_Msk* = (0x00000001 shl DAC_CR_DMAEN2_Pos) ## !< 0x10000000
  DAC_CR_DMAEN2* = DAC_CR_DMAEN2_Msk
  DAC_CR_DMAUDRIE2_Pos* = (29)
  DAC_CR_DMAUDRIE2_Msk* = (0x00000001 shl DAC_CR_DMAUDRIE2_Pos) ## !< 0x20000000
  DAC_CR_DMAUDRIE2* = DAC_CR_DMAUDRIE2_Msk

## ****************  Bit definition for DAC_SWTRIGR register  *****************

const
  DAC_SWTRIGR_SWTRIG1_Pos* = (0)
  DAC_SWTRIGR_SWTRIG1_Msk* = (0x00000001 shl DAC_SWTRIGR_SWTRIG1_Pos) ## !< 0x00000001
  DAC_SWTRIGR_SWTRIG1* = DAC_SWTRIGR_SWTRIG1_Msk
  DAC_SWTRIGR_SWTRIG2_Pos* = (1)
  DAC_SWTRIGR_SWTRIG2_Msk* = (0x00000001 shl DAC_SWTRIGR_SWTRIG2_Pos) ## !< 0x00000002
  DAC_SWTRIGR_SWTRIG2* = DAC_SWTRIGR_SWTRIG2_Msk

## ****************  Bit definition for DAC_DHR12R1 register  *****************

const
  DAC_DHR12R1_DACC1DHR_Pos* = (0)
  DAC_DHR12R1_DACC1DHR_Msk* = (0x00000FFF shl DAC_DHR12R1_DACC1DHR_Pos) ## !< 0x00000FFF
  DAC_DHR12R1_DACC1DHR* = DAC_DHR12R1_DACC1DHR_Msk

## ****************  Bit definition for DAC_DHR12L1 register  *****************

const
  DAC_DHR12L1_DACC1DHR_Pos* = (4)
  DAC_DHR12L1_DACC1DHR_Msk* = (0x00000FFF shl DAC_DHR12L1_DACC1DHR_Pos) ## !< 0x0000FFF0
  DAC_DHR12L1_DACC1DHR* = DAC_DHR12L1_DACC1DHR_Msk

## *****************  Bit definition for DAC_DHR8R1 register  *****************

const
  DAC_DHR8R1_DACC1DHR_Pos* = (0)
  DAC_DHR8R1_DACC1DHR_Msk* = (0x000000FF shl DAC_DHR8R1_DACC1DHR_Pos) ## !< 0x000000FF
  DAC_DHR8R1_DACC1DHR* = DAC_DHR8R1_DACC1DHR_Msk

## ****************  Bit definition for DAC_DHR12R2 register  *****************

const
  DAC_DHR12R2_DACC2DHR_Pos* = (0)
  DAC_DHR12R2_DACC2DHR_Msk* = (0x00000FFF shl DAC_DHR12R2_DACC2DHR_Pos) ## !< 0x00000FFF
  DAC_DHR12R2_DACC2DHR* = DAC_DHR12R2_DACC2DHR_Msk

## ****************  Bit definition for DAC_DHR12L2 register  *****************

const
  DAC_DHR12L2_DACC2DHR_Pos* = (4)
  DAC_DHR12L2_DACC2DHR_Msk* = (0x00000FFF shl DAC_DHR12L2_DACC2DHR_Pos) ## !< 0x0000FFF0
  DAC_DHR12L2_DACC2DHR* = DAC_DHR12L2_DACC2DHR_Msk

## *****************  Bit definition for DAC_DHR8R2 register  *****************

const
  DAC_DHR8R2_DACC2DHR_Pos* = (0)
  DAC_DHR8R2_DACC2DHR_Msk* = (0x000000FF shl DAC_DHR8R2_DACC2DHR_Pos) ## !< 0x000000FF
  DAC_DHR8R2_DACC2DHR* = DAC_DHR8R2_DACC2DHR_Msk

## ****************  Bit definition for DAC_DHR12RD register  *****************

const
  DAC_DHR12RD_DACC1DHR_Pos* = (0)
  DAC_DHR12RD_DACC1DHR_Msk* = (0x00000FFF shl DAC_DHR12RD_DACC1DHR_Pos) ## !< 0x00000FFF
  DAC_DHR12RD_DACC1DHR* = DAC_DHR12RD_DACC1DHR_Msk
  DAC_DHR12RD_DACC2DHR_Pos* = (16)
  DAC_DHR12RD_DACC2DHR_Msk* = (0x00000FFF shl DAC_DHR12RD_DACC2DHR_Pos) ## !< 0x0FFF0000
  DAC_DHR12RD_DACC2DHR* = DAC_DHR12RD_DACC2DHR_Msk

## ****************  Bit definition for DAC_DHR12LD register  *****************

const
  DAC_DHR12LD_DACC1DHR_Pos* = (4)
  DAC_DHR12LD_DACC1DHR_Msk* = (0x00000FFF shl DAC_DHR12LD_DACC1DHR_Pos) ## !< 0x0000FFF0
  DAC_DHR12LD_DACC1DHR* = DAC_DHR12LD_DACC1DHR_Msk
  DAC_DHR12LD_DACC2DHR_Pos* = (20)
  DAC_DHR12LD_DACC2DHR_Msk* = (0x00000FFF shl DAC_DHR12LD_DACC2DHR_Pos) ## !< 0xFFF00000
  DAC_DHR12LD_DACC2DHR* = DAC_DHR12LD_DACC2DHR_Msk

## *****************  Bit definition for DAC_DHR8RD register  *****************

const
  DAC_DHR8RD_DACC1DHR_Pos* = (0)
  DAC_DHR8RD_DACC1DHR_Msk* = (0x000000FF shl DAC_DHR8RD_DACC1DHR_Pos) ## !< 0x000000FF
  DAC_DHR8RD_DACC1DHR* = DAC_DHR8RD_DACC1DHR_Msk
  DAC_DHR8RD_DACC2DHR_Pos* = (8)
  DAC_DHR8RD_DACC2DHR_Msk* = (0x000000FF shl DAC_DHR8RD_DACC2DHR_Pos) ## !< 0x0000FF00
  DAC_DHR8RD_DACC2DHR* = DAC_DHR8RD_DACC2DHR_Msk

## ******************  Bit definition for DAC_DOR1 register  ******************

const
  DAC_DOR1_DACC1DOR_Pos* = (0)
  DAC_DOR1_DACC1DOR_Msk* = (0x00000FFF shl DAC_DOR1_DACC1DOR_Pos) ## !< 0x00000FFF
  DAC_DOR1_DACC1DOR* = DAC_DOR1_DACC1DOR_Msk

## ******************  Bit definition for DAC_DOR2 register  ******************

const
  DAC_DOR2_DACC2DOR_Pos* = (0)
  DAC_DOR2_DACC2DOR_Msk* = (0x00000FFF shl DAC_DOR2_DACC2DOR_Pos) ## !< 0x00000FFF
  DAC_DOR2_DACC2DOR* = DAC_DOR2_DACC2DOR_Msk

## *******************  Bit definition for DAC_SR register  *******************

const
  DAC_SR_DMAUDR1_Pos* = (13)
  DAC_SR_DMAUDR1_Msk* = (0x00000001 shl DAC_SR_DMAUDR1_Pos) ## !< 0x00002000
  DAC_SR_DMAUDR1* = DAC_SR_DMAUDR1_Msk
  DAC_SR_DMAUDR2_Pos* = (29)
  DAC_SR_DMAUDR2_Msk* = (0x00000001 shl DAC_SR_DMAUDR2_Pos) ## !< 0x20000000
  DAC_SR_DMAUDR2* = DAC_SR_DMAUDR2_Msk

## ****************************************************************************
##
##                            Debug MCU (DBGMCU)
##
## ****************************************************************************
## ***************  Bit definition for DBGMCU_IDCODE register  ****************

const
  DBGMCU_IDCODE_DEV_ID_Pos* = (0)
  DBGMCU_IDCODE_DEV_ID_Msk* = (0x00000FFF shl DBGMCU_IDCODE_DEV_ID_Pos) ## !< 0x00000FFF
  DBGMCU_IDCODE_DEV_ID* = DBGMCU_IDCODE_DEV_ID_Msk
  DBGMCU_IDCODE_REV_ID_Pos* = (16)
  DBGMCU_IDCODE_REV_ID_Msk* = (0x0000FFFF shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0xFFFF0000
  DBGMCU_IDCODE_REV_ID* = DBGMCU_IDCODE_REV_ID_Msk
  DBGMCU_IDCODE_REV_ID_0* = (0x00000001 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x00010000
  DBGMCU_IDCODE_REV_ID_1* = (0x00000002 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x00020000
  DBGMCU_IDCODE_REV_ID_2* = (0x00000004 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x00040000
  DBGMCU_IDCODE_REV_ID_3* = (0x00000008 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x00080000
  DBGMCU_IDCODE_REV_ID_4* = (0x00000010 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x00100000
  DBGMCU_IDCODE_REV_ID_5* = (0x00000020 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x00200000
  DBGMCU_IDCODE_REV_ID_6* = (0x00000040 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x00400000
  DBGMCU_IDCODE_REV_ID_7* = (0x00000080 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x00800000
  DBGMCU_IDCODE_REV_ID_8* = (0x00000100 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x01000000
  DBGMCU_IDCODE_REV_ID_9* = (0x00000200 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x02000000
  DBGMCU_IDCODE_REV_ID_10* = (0x00000400 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x04000000
  DBGMCU_IDCODE_REV_ID_11* = (0x00000800 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x08000000
  DBGMCU_IDCODE_REV_ID_12* = (0x00001000 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x10000000
  DBGMCU_IDCODE_REV_ID_13* = (0x00002000 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x20000000
  DBGMCU_IDCODE_REV_ID_14* = (0x00004000 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x40000000
  DBGMCU_IDCODE_REV_ID_15* = (0x00008000 shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0x80000000

## *****************  Bit definition for DBGMCU_CR register  ******************

const
  DBGMCU_CR_DBG_SLEEP_Pos* = (0)
  DBGMCU_CR_DBG_SLEEP_Msk* = (0x00000001 shl DBGMCU_CR_DBG_SLEEP_Pos) ## !< 0x00000001
  DBGMCU_CR_DBG_SLEEP* = DBGMCU_CR_DBG_SLEEP_Msk
  DBGMCU_CR_DBG_STOP_Pos* = (1)
  DBGMCU_CR_DBG_STOP_Msk* = (0x00000001 shl DBGMCU_CR_DBG_STOP_Pos) ## !< 0x00000002
  DBGMCU_CR_DBG_STOP* = DBGMCU_CR_DBG_STOP_Msk
  DBGMCU_CR_DBG_STANDBY_Pos* = (2)
  DBGMCU_CR_DBG_STANDBY_Msk* = (0x00000001 shl DBGMCU_CR_DBG_STANDBY_Pos) ## !< 0x00000004
  DBGMCU_CR_DBG_STANDBY* = DBGMCU_CR_DBG_STANDBY_Msk
  DBGMCU_CR_TRACE_IOEN_Pos* = (5)
  DBGMCU_CR_TRACE_IOEN_Msk* = (0x00000001 shl DBGMCU_CR_TRACE_IOEN_Pos) ## !< 0x00000020
  DBGMCU_CR_TRACE_IOEN* = DBGMCU_CR_TRACE_IOEN_Msk
  DBGMCU_CR_TRACE_MODE_Pos* = (6)
  DBGMCU_CR_TRACE_MODE_Msk* = (0x00000003 shl DBGMCU_CR_TRACE_MODE_Pos) ## !< 0x000000C0
  DBGMCU_CR_TRACE_MODE* = DBGMCU_CR_TRACE_MODE_Msk
  DBGMCU_CR_TRACE_MODE_0* = (0x00000001 shl DBGMCU_CR_TRACE_MODE_Pos) ## !< 0x00000040
  DBGMCU_CR_TRACE_MODE_1* = (0x00000002 shl DBGMCU_CR_TRACE_MODE_Pos) ## !< 0x00000080

## *****************  Bit definition for DBGMCU_APB1_FZ register  *************

const
  DBGMCU_APB1_FZ_DBG_TIM2_STOP_Pos* = (0)
  DBGMCU_APB1_FZ_DBG_TIM2_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_TIM2_STOP_Pos) ## !< 0x00000001
  DBGMCU_APB1_FZ_DBG_TIM2_STOP* = DBGMCU_APB1_FZ_DBG_TIM2_STOP_Msk
  DBGMCU_APB1_FZ_DBG_TIM3_STOP_Pos* = (1)
  DBGMCU_APB1_FZ_DBG_TIM3_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_TIM3_STOP_Pos) ## !< 0x00000002
  DBGMCU_APB1_FZ_DBG_TIM3_STOP* = DBGMCU_APB1_FZ_DBG_TIM3_STOP_Msk
  DBGMCU_APB1_FZ_DBG_TIM4_STOP_Pos* = (2)
  DBGMCU_APB1_FZ_DBG_TIM4_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_TIM4_STOP_Pos) ## !< 0x00000004
  DBGMCU_APB1_FZ_DBG_TIM4_STOP* = DBGMCU_APB1_FZ_DBG_TIM4_STOP_Msk
  DBGMCU_APB1_FZ_DBG_TIM5_STOP_Pos* = (3)
  DBGMCU_APB1_FZ_DBG_TIM5_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_TIM5_STOP_Pos) ## !< 0x00000008
  DBGMCU_APB1_FZ_DBG_TIM5_STOP* = DBGMCU_APB1_FZ_DBG_TIM5_STOP_Msk
  DBGMCU_APB1_FZ_DBG_TIM6_STOP_Pos* = (4)
  DBGMCU_APB1_FZ_DBG_TIM6_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_TIM6_STOP_Pos) ## !< 0x00000010
  DBGMCU_APB1_FZ_DBG_TIM6_STOP* = DBGMCU_APB1_FZ_DBG_TIM6_STOP_Msk
  DBGMCU_APB1_FZ_DBG_TIM7_STOP_Pos* = (5)
  DBGMCU_APB1_FZ_DBG_TIM7_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_TIM7_STOP_Pos) ## !< 0x00000020
  DBGMCU_APB1_FZ_DBG_TIM7_STOP* = DBGMCU_APB1_FZ_DBG_TIM7_STOP_Msk
  DBGMCU_APB1_FZ_DBG_RTC_STOP_Pos* = (10)
  DBGMCU_APB1_FZ_DBG_RTC_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_RTC_STOP_Pos) ## !< 0x00000400
  DBGMCU_APB1_FZ_DBG_RTC_STOP* = DBGMCU_APB1_FZ_DBG_RTC_STOP_Msk
  DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos* = (11)
  DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos) ## !< 0x00000800
  DBGMCU_APB1_FZ_DBG_WWDG_STOP* = DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk
  DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos* = (12)
  DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos) ## !< 0x00001000
  DBGMCU_APB1_FZ_DBG_IWDG_STOP* = DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk
  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos* = (21)
  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos) ## !< 0x00200000
  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT* = DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk
  DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Pos* = (22)
  DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Pos) ## !< 0x00400000
  DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT* = DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Msk

## *****************  Bit definition for DBGMCU_APB2_FZ register  *************

const
  DBGMCU_APB2_FZ_DBG_TIM9_STOP_Pos* = (2)
  DBGMCU_APB2_FZ_DBG_TIM9_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM9_STOP_Pos) ## !< 0x00000004
  DBGMCU_APB2_FZ_DBG_TIM9_STOP* = DBGMCU_APB2_FZ_DBG_TIM9_STOP_Msk
  DBGMCU_APB2_FZ_DBG_TIM10_STOP_Pos* = (3)
  DBGMCU_APB2_FZ_DBG_TIM10_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM10_STOP_Pos) ## !< 0x00000008
  DBGMCU_APB2_FZ_DBG_TIM10_STOP* = DBGMCU_APB2_FZ_DBG_TIM10_STOP_Msk
  DBGMCU_APB2_FZ_DBG_TIM11_STOP_Pos* = (4)
  DBGMCU_APB2_FZ_DBG_TIM11_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM11_STOP_Pos) ## !< 0x00000010
  DBGMCU_APB2_FZ_DBG_TIM11_STOP* = DBGMCU_APB2_FZ_DBG_TIM11_STOP_Msk

## ****************************************************************************
##
##                            DMA Controller (DMA)
##
## ****************************************************************************
## ******************  Bit definition for DMA_ISR register  *******************

const
  DMA_ISR_GIF1_Pos* = (0)
  DMA_ISR_GIF1_Msk* = (0x00000001 shl DMA_ISR_GIF1_Pos) ## !< 0x00000001
  DMA_ISR_GIF1* = DMA_ISR_GIF1_Msk
  DMA_ISR_TCIF1_Pos* = (1)
  DMA_ISR_TCIF1_Msk* = (0x00000001 shl DMA_ISR_TCIF1_Pos) ## !< 0x00000002
  DMA_ISR_TCIF1* = DMA_ISR_TCIF1_Msk
  DMA_ISR_HTIF1_Pos* = (2)
  DMA_ISR_HTIF1_Msk* = (0x00000001 shl DMA_ISR_HTIF1_Pos) ## !< 0x00000004
  DMA_ISR_HTIF1* = DMA_ISR_HTIF1_Msk
  DMA_ISR_TEIF1_Pos* = (3)
  DMA_ISR_TEIF1_Msk* = (0x00000001 shl DMA_ISR_TEIF1_Pos) ## !< 0x00000008
  DMA_ISR_TEIF1* = DMA_ISR_TEIF1_Msk
  DMA_ISR_GIF2_Pos* = (4)
  DMA_ISR_GIF2_Msk* = (0x00000001 shl DMA_ISR_GIF2_Pos) ## !< 0x00000010
  DMA_ISR_GIF2* = DMA_ISR_GIF2_Msk
  DMA_ISR_TCIF2_Pos* = (5)
  DMA_ISR_TCIF2_Msk* = (0x00000001 shl DMA_ISR_TCIF2_Pos) ## !< 0x00000020
  DMA_ISR_TCIF2* = DMA_ISR_TCIF2_Msk
  DMA_ISR_HTIF2_Pos* = (6)
  DMA_ISR_HTIF2_Msk* = (0x00000001 shl DMA_ISR_HTIF2_Pos) ## !< 0x00000040
  DMA_ISR_HTIF2* = DMA_ISR_HTIF2_Msk
  DMA_ISR_TEIF2_Pos* = (7)
  DMA_ISR_TEIF2_Msk* = (0x00000001 shl DMA_ISR_TEIF2_Pos) ## !< 0x00000080
  DMA_ISR_TEIF2* = DMA_ISR_TEIF2_Msk
  DMA_ISR_GIF3_Pos* = (8)
  DMA_ISR_GIF3_Msk* = (0x00000001 shl DMA_ISR_GIF3_Pos) ## !< 0x00000100
  DMA_ISR_GIF3* = DMA_ISR_GIF3_Msk
  DMA_ISR_TCIF3_Pos* = (9)
  DMA_ISR_TCIF3_Msk* = (0x00000001 shl DMA_ISR_TCIF3_Pos) ## !< 0x00000200
  DMA_ISR_TCIF3* = DMA_ISR_TCIF3_Msk
  DMA_ISR_HTIF3_Pos* = (10)
  DMA_ISR_HTIF3_Msk* = (0x00000001 shl DMA_ISR_HTIF3_Pos) ## !< 0x00000400
  DMA_ISR_HTIF3* = DMA_ISR_HTIF3_Msk
  DMA_ISR_TEIF3_Pos* = (11)
  DMA_ISR_TEIF3_Msk* = (0x00000001 shl DMA_ISR_TEIF3_Pos) ## !< 0x00000800
  DMA_ISR_TEIF3* = DMA_ISR_TEIF3_Msk
  DMA_ISR_GIF4_Pos* = (12)
  DMA_ISR_GIF4_Msk* = (0x00000001 shl DMA_ISR_GIF4_Pos) ## !< 0x00001000
  DMA_ISR_GIF4* = DMA_ISR_GIF4_Msk
  DMA_ISR_TCIF4_Pos* = (13)
  DMA_ISR_TCIF4_Msk* = (0x00000001 shl DMA_ISR_TCIF4_Pos) ## !< 0x00002000
  DMA_ISR_TCIF4* = DMA_ISR_TCIF4_Msk
  DMA_ISR_HTIF4_Pos* = (14)
  DMA_ISR_HTIF4_Msk* = (0x00000001 shl DMA_ISR_HTIF4_Pos) ## !< 0x00004000
  DMA_ISR_HTIF4* = DMA_ISR_HTIF4_Msk
  DMA_ISR_TEIF4_Pos* = (15)
  DMA_ISR_TEIF4_Msk* = (0x00000001 shl DMA_ISR_TEIF4_Pos) ## !< 0x00008000
  DMA_ISR_TEIF4* = DMA_ISR_TEIF4_Msk
  DMA_ISR_GIF5_Pos* = (16)
  DMA_ISR_GIF5_Msk* = (0x00000001 shl DMA_ISR_GIF5_Pos) ## !< 0x00010000
  DMA_ISR_GIF5* = DMA_ISR_GIF5_Msk
  DMA_ISR_TCIF5_Pos* = (17)
  DMA_ISR_TCIF5_Msk* = (0x00000001 shl DMA_ISR_TCIF5_Pos) ## !< 0x00020000
  DMA_ISR_TCIF5* = DMA_ISR_TCIF5_Msk
  DMA_ISR_HTIF5_Pos* = (18)
  DMA_ISR_HTIF5_Msk* = (0x00000001 shl DMA_ISR_HTIF5_Pos) ## !< 0x00040000
  DMA_ISR_HTIF5* = DMA_ISR_HTIF5_Msk
  DMA_ISR_TEIF5_Pos* = (19)
  DMA_ISR_TEIF5_Msk* = (0x00000001 shl DMA_ISR_TEIF5_Pos) ## !< 0x00080000
  DMA_ISR_TEIF5* = DMA_ISR_TEIF5_Msk
  DMA_ISR_GIF6_Pos* = (20)
  DMA_ISR_GIF6_Msk* = (0x00000001 shl DMA_ISR_GIF6_Pos) ## !< 0x00100000
  DMA_ISR_GIF6* = DMA_ISR_GIF6_Msk
  DMA_ISR_TCIF6_Pos* = (21)
  DMA_ISR_TCIF6_Msk* = (0x00000001 shl DMA_ISR_TCIF6_Pos) ## !< 0x00200000
  DMA_ISR_TCIF6* = DMA_ISR_TCIF6_Msk
  DMA_ISR_HTIF6_Pos* = (22)
  DMA_ISR_HTIF6_Msk* = (0x00000001 shl DMA_ISR_HTIF6_Pos) ## !< 0x00400000
  DMA_ISR_HTIF6* = DMA_ISR_HTIF6_Msk
  DMA_ISR_TEIF6_Pos* = (23)
  DMA_ISR_TEIF6_Msk* = (0x00000001 shl DMA_ISR_TEIF6_Pos) ## !< 0x00800000
  DMA_ISR_TEIF6* = DMA_ISR_TEIF6_Msk
  DMA_ISR_GIF7_Pos* = (24)
  DMA_ISR_GIF7_Msk* = (0x00000001 shl DMA_ISR_GIF7_Pos) ## !< 0x01000000
  DMA_ISR_GIF7* = DMA_ISR_GIF7_Msk
  DMA_ISR_TCIF7_Pos* = (25)
  DMA_ISR_TCIF7_Msk* = (0x00000001 shl DMA_ISR_TCIF7_Pos) ## !< 0x02000000
  DMA_ISR_TCIF7* = DMA_ISR_TCIF7_Msk
  DMA_ISR_HTIF7_Pos* = (26)
  DMA_ISR_HTIF7_Msk* = (0x00000001 shl DMA_ISR_HTIF7_Pos) ## !< 0x04000000
  DMA_ISR_HTIF7* = DMA_ISR_HTIF7_Msk
  DMA_ISR_TEIF7_Pos* = (27)
  DMA_ISR_TEIF7_Msk* = (0x00000001 shl DMA_ISR_TEIF7_Pos) ## !< 0x08000000
  DMA_ISR_TEIF7* = DMA_ISR_TEIF7_Msk

## ******************  Bit definition for DMA_IFCR register  ******************

const
  DMA_IFCR_CGIF1_Pos* = (0)
  DMA_IFCR_CGIF1_Msk* = (0x00000001 shl DMA_IFCR_CGIF1_Pos) ## !< 0x00000001
  DMA_IFCR_CGIF1* = DMA_IFCR_CGIF1_Msk
  DMA_IFCR_CTCIF1_Pos* = (1)
  DMA_IFCR_CTCIF1_Msk* = (0x00000001 shl DMA_IFCR_CTCIF1_Pos) ## !< 0x00000002
  DMA_IFCR_CTCIF1* = DMA_IFCR_CTCIF1_Msk
  DMA_IFCR_CHTIF1_Pos* = (2)
  DMA_IFCR_CHTIF1_Msk* = (0x00000001 shl DMA_IFCR_CHTIF1_Pos) ## !< 0x00000004
  DMA_IFCR_CHTIF1* = DMA_IFCR_CHTIF1_Msk
  DMA_IFCR_CTEIF1_Pos* = (3)
  DMA_IFCR_CTEIF1_Msk* = (0x00000001 shl DMA_IFCR_CTEIF1_Pos) ## !< 0x00000008
  DMA_IFCR_CTEIF1* = DMA_IFCR_CTEIF1_Msk
  DMA_IFCR_CGIF2_Pos* = (4)
  DMA_IFCR_CGIF2_Msk* = (0x00000001 shl DMA_IFCR_CGIF2_Pos) ## !< 0x00000010
  DMA_IFCR_CGIF2* = DMA_IFCR_CGIF2_Msk
  DMA_IFCR_CTCIF2_Pos* = (5)
  DMA_IFCR_CTCIF2_Msk* = (0x00000001 shl DMA_IFCR_CTCIF2_Pos) ## !< 0x00000020
  DMA_IFCR_CTCIF2* = DMA_IFCR_CTCIF2_Msk
  DMA_IFCR_CHTIF2_Pos* = (6)
  DMA_IFCR_CHTIF2_Msk* = (0x00000001 shl DMA_IFCR_CHTIF2_Pos) ## !< 0x00000040
  DMA_IFCR_CHTIF2* = DMA_IFCR_CHTIF2_Msk
  DMA_IFCR_CTEIF2_Pos* = (7)
  DMA_IFCR_CTEIF2_Msk* = (0x00000001 shl DMA_IFCR_CTEIF2_Pos) ## !< 0x00000080
  DMA_IFCR_CTEIF2* = DMA_IFCR_CTEIF2_Msk
  DMA_IFCR_CGIF3_Pos* = (8)
  DMA_IFCR_CGIF3_Msk* = (0x00000001 shl DMA_IFCR_CGIF3_Pos) ## !< 0x00000100
  DMA_IFCR_CGIF3* = DMA_IFCR_CGIF3_Msk
  DMA_IFCR_CTCIF3_Pos* = (9)
  DMA_IFCR_CTCIF3_Msk* = (0x00000001 shl DMA_IFCR_CTCIF3_Pos) ## !< 0x00000200
  DMA_IFCR_CTCIF3* = DMA_IFCR_CTCIF3_Msk
  DMA_IFCR_CHTIF3_Pos* = (10)
  DMA_IFCR_CHTIF3_Msk* = (0x00000001 shl DMA_IFCR_CHTIF3_Pos) ## !< 0x00000400
  DMA_IFCR_CHTIF3* = DMA_IFCR_CHTIF3_Msk
  DMA_IFCR_CTEIF3_Pos* = (11)
  DMA_IFCR_CTEIF3_Msk* = (0x00000001 shl DMA_IFCR_CTEIF3_Pos) ## !< 0x00000800
  DMA_IFCR_CTEIF3* = DMA_IFCR_CTEIF3_Msk
  DMA_IFCR_CGIF4_Pos* = (12)
  DMA_IFCR_CGIF4_Msk* = (0x00000001 shl DMA_IFCR_CGIF4_Pos) ## !< 0x00001000
  DMA_IFCR_CGIF4* = DMA_IFCR_CGIF4_Msk
  DMA_IFCR_CTCIF4_Pos* = (13)
  DMA_IFCR_CTCIF4_Msk* = (0x00000001 shl DMA_IFCR_CTCIF4_Pos) ## !< 0x00002000
  DMA_IFCR_CTCIF4* = DMA_IFCR_CTCIF4_Msk
  DMA_IFCR_CHTIF4_Pos* = (14)
  DMA_IFCR_CHTIF4_Msk* = (0x00000001 shl DMA_IFCR_CHTIF4_Pos) ## !< 0x00004000
  DMA_IFCR_CHTIF4* = DMA_IFCR_CHTIF4_Msk
  DMA_IFCR_CTEIF4_Pos* = (15)
  DMA_IFCR_CTEIF4_Msk* = (0x00000001 shl DMA_IFCR_CTEIF4_Pos) ## !< 0x00008000
  DMA_IFCR_CTEIF4* = DMA_IFCR_CTEIF4_Msk
  DMA_IFCR_CGIF5_Pos* = (16)
  DMA_IFCR_CGIF5_Msk* = (0x00000001 shl DMA_IFCR_CGIF5_Pos) ## !< 0x00010000
  DMA_IFCR_CGIF5* = DMA_IFCR_CGIF5_Msk
  DMA_IFCR_CTCIF5_Pos* = (17)
  DMA_IFCR_CTCIF5_Msk* = (0x00000001 shl DMA_IFCR_CTCIF5_Pos) ## !< 0x00020000
  DMA_IFCR_CTCIF5* = DMA_IFCR_CTCIF5_Msk
  DMA_IFCR_CHTIF5_Pos* = (18)
  DMA_IFCR_CHTIF5_Msk* = (0x00000001 shl DMA_IFCR_CHTIF5_Pos) ## !< 0x00040000
  DMA_IFCR_CHTIF5* = DMA_IFCR_CHTIF5_Msk
  DMA_IFCR_CTEIF5_Pos* = (19)
  DMA_IFCR_CTEIF5_Msk* = (0x00000001 shl DMA_IFCR_CTEIF5_Pos) ## !< 0x00080000
  DMA_IFCR_CTEIF5* = DMA_IFCR_CTEIF5_Msk
  DMA_IFCR_CGIF6_Pos* = (20)
  DMA_IFCR_CGIF6_Msk* = (0x00000001 shl DMA_IFCR_CGIF6_Pos) ## !< 0x00100000
  DMA_IFCR_CGIF6* = DMA_IFCR_CGIF6_Msk
  DMA_IFCR_CTCIF6_Pos* = (21)
  DMA_IFCR_CTCIF6_Msk* = (0x00000001 shl DMA_IFCR_CTCIF6_Pos) ## !< 0x00200000
  DMA_IFCR_CTCIF6* = DMA_IFCR_CTCIF6_Msk
  DMA_IFCR_CHTIF6_Pos* = (22)
  DMA_IFCR_CHTIF6_Msk* = (0x00000001 shl DMA_IFCR_CHTIF6_Pos) ## !< 0x00400000
  DMA_IFCR_CHTIF6* = DMA_IFCR_CHTIF6_Msk
  DMA_IFCR_CTEIF6_Pos* = (23)
  DMA_IFCR_CTEIF6_Msk* = (0x00000001 shl DMA_IFCR_CTEIF6_Pos) ## !< 0x00800000
  DMA_IFCR_CTEIF6* = DMA_IFCR_CTEIF6_Msk
  DMA_IFCR_CGIF7_Pos* = (24)
  DMA_IFCR_CGIF7_Msk* = (0x00000001 shl DMA_IFCR_CGIF7_Pos) ## !< 0x01000000
  DMA_IFCR_CGIF7* = DMA_IFCR_CGIF7_Msk
  DMA_IFCR_CTCIF7_Pos* = (25)
  DMA_IFCR_CTCIF7_Msk* = (0x00000001 shl DMA_IFCR_CTCIF7_Pos) ## !< 0x02000000
  DMA_IFCR_CTCIF7* = DMA_IFCR_CTCIF7_Msk
  DMA_IFCR_CHTIF7_Pos* = (26)
  DMA_IFCR_CHTIF7_Msk* = (0x00000001 shl DMA_IFCR_CHTIF7_Pos) ## !< 0x04000000
  DMA_IFCR_CHTIF7* = DMA_IFCR_CHTIF7_Msk
  DMA_IFCR_CTEIF7_Pos* = (27)
  DMA_IFCR_CTEIF7_Msk* = (0x00000001 shl DMA_IFCR_CTEIF7_Pos) ## !< 0x08000000
  DMA_IFCR_CTEIF7* = DMA_IFCR_CTEIF7_Msk

## ******************  Bit definition for DMA_CCR register  ******************

const
  DMA_CCR_EN_Pos* = (0)
  DMA_CCR_EN_Msk* = (0x00000001 shl DMA_CCR_EN_Pos) ## !< 0x00000001
  DMA_CCR_EN* = DMA_CCR_EN_Msk
  DMA_CCR_TCIE_Pos* = (1)
  DMA_CCR_TCIE_Msk* = (0x00000001 shl DMA_CCR_TCIE_Pos) ## !< 0x00000002
  DMA_CCR_TCIE* = DMA_CCR_TCIE_Msk
  DMA_CCR_HTIE_Pos* = (2)
  DMA_CCR_HTIE_Msk* = (0x00000001 shl DMA_CCR_HTIE_Pos) ## !< 0x00000004
  DMA_CCR_HTIE* = DMA_CCR_HTIE_Msk
  DMA_CCR_TEIE_Pos* = (3)
  DMA_CCR_TEIE_Msk* = (0x00000001 shl DMA_CCR_TEIE_Pos) ## !< 0x00000008
  DMA_CCR_TEIE* = DMA_CCR_TEIE_Msk
  DMA_CCR_DIR_Pos* = (4)
  DMA_CCR_DIR_Msk* = (0x00000001 shl DMA_CCR_DIR_Pos) ## !< 0x00000010
  DMA_CCR_DIR* = DMA_CCR_DIR_Msk
  DMA_CCR_CIRC_Pos* = (5)
  DMA_CCR_CIRC_Msk* = (0x00000001 shl DMA_CCR_CIRC_Pos) ## !< 0x00000020
  DMA_CCR_CIRC* = DMA_CCR_CIRC_Msk
  DMA_CCR_PINC_Pos* = (6)
  DMA_CCR_PINC_Msk* = (0x00000001 shl DMA_CCR_PINC_Pos) ## !< 0x00000040
  DMA_CCR_PINC* = DMA_CCR_PINC_Msk
  DMA_CCR_MINC_Pos* = (7)
  DMA_CCR_MINC_Msk* = (0x00000001 shl DMA_CCR_MINC_Pos) ## !< 0x00000080
  DMA_CCR_MINC* = DMA_CCR_MINC_Msk
  DMA_CCR_PSIZE_Pos* = (8)
  DMA_CCR_PSIZE_Msk* = (0x00000003 shl DMA_CCR_PSIZE_Pos) ## !< 0x00000300
  DMA_CCR_PSIZE* = DMA_CCR_PSIZE_Msk
  DMA_CCR_PSIZE_0* = (0x00000001 shl DMA_CCR_PSIZE_Pos) ## !< 0x00000100
  DMA_CCR_PSIZE_1* = (0x00000002 shl DMA_CCR_PSIZE_Pos) ## !< 0x00000200
  DMA_CCR_MSIZE_Pos* = (10)
  DMA_CCR_MSIZE_Msk* = (0x00000003 shl DMA_CCR_MSIZE_Pos) ## !< 0x00000C00
  DMA_CCR_MSIZE* = DMA_CCR_MSIZE_Msk
  DMA_CCR_MSIZE_0* = (0x00000001 shl DMA_CCR_MSIZE_Pos) ## !< 0x00000400
  DMA_CCR_MSIZE_1* = (0x00000002 shl DMA_CCR_MSIZE_Pos) ## !< 0x00000800
  DMA_CCR_PL_Pos* = (12)
  DMA_CCR_PL_Msk* = (0x00000003 shl DMA_CCR_PL_Pos) ## !< 0x00003000
  DMA_CCR_PL* = DMA_CCR_PL_Msk
  DMA_CCR_PL_0* = (0x00000001 shl DMA_CCR_PL_Pos) ## !< 0x00001000
  DMA_CCR_PL_1* = (0x00000002 shl DMA_CCR_PL_Pos) ## !< 0x00002000
  DMA_CCR_MEM2MEM_Pos* = (14)
  DMA_CCR_MEM2MEM_Msk* = (0x00000001 shl DMA_CCR_MEM2MEM_Pos) ## !< 0x00004000
  DMA_CCR_MEM2MEM* = DMA_CCR_MEM2MEM_Msk

## *****************  Bit definition generic for DMA_CNDTR register  ******************

const
  DMA_CNDTR_NDT_Pos* = (0)
  DMA_CNDTR_NDT_Msk* = (0x0000FFFF shl DMA_CNDTR_NDT_Pos) ## !< 0x0000FFFF
  DMA_CNDTR_NDT* = DMA_CNDTR_NDT_Msk

## *****************  Bit definition for DMA_CNDTR1 register  *****************

const
  DMA_CNDTR1_NDT_Pos* = (0)
  DMA_CNDTR1_NDT_Msk* = (0x0000FFFF shl DMA_CNDTR1_NDT_Pos) ## !< 0x0000FFFF
  DMA_CNDTR1_NDT* = DMA_CNDTR1_NDT_Msk

## *****************  Bit definition for DMA_CNDTR2 register  *****************

const
  DMA_CNDTR2_NDT_Pos* = (0)
  DMA_CNDTR2_NDT_Msk* = (0x0000FFFF shl DMA_CNDTR2_NDT_Pos) ## !< 0x0000FFFF
  DMA_CNDTR2_NDT* = DMA_CNDTR2_NDT_Msk

## *****************  Bit definition for DMA_CNDTR3 register  *****************

const
  DMA_CNDTR3_NDT_Pos* = (0)
  DMA_CNDTR3_NDT_Msk* = (0x0000FFFF shl DMA_CNDTR3_NDT_Pos) ## !< 0x0000FFFF
  DMA_CNDTR3_NDT* = DMA_CNDTR3_NDT_Msk

## *****************  Bit definition for DMA_CNDTR4 register  *****************

const
  DMA_CNDTR4_NDT_Pos* = (0)
  DMA_CNDTR4_NDT_Msk* = (0x0000FFFF shl DMA_CNDTR4_NDT_Pos) ## !< 0x0000FFFF
  DMA_CNDTR4_NDT* = DMA_CNDTR4_NDT_Msk

## *****************  Bit definition for DMA_CNDTR5 register  *****************

const
  DMA_CNDTR5_NDT_Pos* = (0)
  DMA_CNDTR5_NDT_Msk* = (0x0000FFFF shl DMA_CNDTR5_NDT_Pos) ## !< 0x0000FFFF
  DMA_CNDTR5_NDT* = DMA_CNDTR5_NDT_Msk

## *****************  Bit definition for DMA_CNDTR6 register  *****************

const
  DMA_CNDTR6_NDT_Pos* = (0)
  DMA_CNDTR6_NDT_Msk* = (0x0000FFFF shl DMA_CNDTR6_NDT_Pos) ## !< 0x0000FFFF
  DMA_CNDTR6_NDT* = DMA_CNDTR6_NDT_Msk

## *****************  Bit definition for DMA_CNDTR7 register  *****************

const
  DMA_CNDTR7_NDT_Pos* = (0)
  DMA_CNDTR7_NDT_Msk* = (0x0000FFFF shl DMA_CNDTR7_NDT_Pos) ## !< 0x0000FFFF
  DMA_CNDTR7_NDT* = DMA_CNDTR7_NDT_Msk

## *****************  Bit definition generic for DMA_CPAR register  *******************

const
  DMA_CPAR_PA_Pos* = (0)
  DMA_CPAR_PA_Msk* = (0xFFFFFFFF shl DMA_CPAR_PA_Pos) ## !< 0xFFFFFFFF
  DMA_CPAR_PA* = DMA_CPAR_PA_Msk

## *****************  Bit definition for DMA_CPAR1 register  ******************

const
  DMA_CPAR1_PA_Pos* = (0)
  DMA_CPAR1_PA_Msk* = (0xFFFFFFFF shl DMA_CPAR1_PA_Pos) ## !< 0xFFFFFFFF
  DMA_CPAR1_PA* = DMA_CPAR1_PA_Msk

## *****************  Bit definition for DMA_CPAR2 register  ******************

const
  DMA_CPAR2_PA_Pos* = (0)
  DMA_CPAR2_PA_Msk* = (0xFFFFFFFF shl DMA_CPAR2_PA_Pos) ## !< 0xFFFFFFFF
  DMA_CPAR2_PA* = DMA_CPAR2_PA_Msk

## *****************  Bit definition for DMA_CPAR3 register  ******************

const
  DMA_CPAR3_PA_Pos* = (0)
  DMA_CPAR3_PA_Msk* = (0xFFFFFFFF shl DMA_CPAR3_PA_Pos) ## !< 0xFFFFFFFF
  DMA_CPAR3_PA* = DMA_CPAR3_PA_Msk

## *****************  Bit definition for DMA_CPAR4 register  ******************

const
  DMA_CPAR4_PA_Pos* = (0)
  DMA_CPAR4_PA_Msk* = (0xFFFFFFFF shl DMA_CPAR4_PA_Pos) ## !< 0xFFFFFFFF
  DMA_CPAR4_PA* = DMA_CPAR4_PA_Msk

## *****************  Bit definition for DMA_CPAR5 register  ******************

const
  DMA_CPAR5_PA_Pos* = (0)
  DMA_CPAR5_PA_Msk* = (0xFFFFFFFF shl DMA_CPAR5_PA_Pos) ## !< 0xFFFFFFFF
  DMA_CPAR5_PA* = DMA_CPAR5_PA_Msk

## *****************  Bit definition for DMA_CPAR6 register  ******************

const
  DMA_CPAR6_PA_Pos* = (0)
  DMA_CPAR6_PA_Msk* = (0xFFFFFFFF shl DMA_CPAR6_PA_Pos) ## !< 0xFFFFFFFF
  DMA_CPAR6_PA* = DMA_CPAR6_PA_Msk

## *****************  Bit definition for DMA_CPAR7 register  ******************

const
  DMA_CPAR7_PA_Pos* = (0)
  DMA_CPAR7_PA_Msk* = (0xFFFFFFFF shl DMA_CPAR7_PA_Pos) ## !< 0xFFFFFFFF
  DMA_CPAR7_PA* = DMA_CPAR7_PA_Msk

## *****************  Bit definition generic for DMA_CMAR register  *******************

const
  DMA_CMAR_MA_Pos* = (0)
  DMA_CMAR_MA_Msk* = (0xFFFFFFFF shl DMA_CMAR_MA_Pos) ## !< 0xFFFFFFFF
  DMA_CMAR_MA* = DMA_CMAR_MA_Msk

## *****************  Bit definition for DMA_CMAR1 register  ******************

const
  DMA_CMAR1_MA_Pos* = (0)
  DMA_CMAR1_MA_Msk* = (0xFFFFFFFF shl DMA_CMAR1_MA_Pos) ## !< 0xFFFFFFFF
  DMA_CMAR1_MA* = DMA_CMAR1_MA_Msk

## *****************  Bit definition for DMA_CMAR2 register  ******************

const
  DMA_CMAR2_MA_Pos* = (0)
  DMA_CMAR2_MA_Msk* = (0xFFFFFFFF shl DMA_CMAR2_MA_Pos) ## !< 0xFFFFFFFF
  DMA_CMAR2_MA* = DMA_CMAR2_MA_Msk

## *****************  Bit definition for DMA_CMAR3 register  ******************

const
  DMA_CMAR3_MA_Pos* = (0)
  DMA_CMAR3_MA_Msk* = (0xFFFFFFFF shl DMA_CMAR3_MA_Pos) ## !< 0xFFFFFFFF
  DMA_CMAR3_MA* = DMA_CMAR3_MA_Msk

## *****************  Bit definition for DMA_CMAR4 register  ******************

const
  DMA_CMAR4_MA_Pos* = (0)
  DMA_CMAR4_MA_Msk* = (0xFFFFFFFF shl DMA_CMAR4_MA_Pos) ## !< 0xFFFFFFFF
  DMA_CMAR4_MA* = DMA_CMAR4_MA_Msk

## *****************  Bit definition for DMA_CMAR5 register  ******************

const
  DMA_CMAR5_MA_Pos* = (0)
  DMA_CMAR5_MA_Msk* = (0xFFFFFFFF shl DMA_CMAR5_MA_Pos) ## !< 0xFFFFFFFF
  DMA_CMAR5_MA* = DMA_CMAR5_MA_Msk

## *****************  Bit definition for DMA_CMAR6 register  ******************

const
  DMA_CMAR6_MA_Pos* = (0)
  DMA_CMAR6_MA_Msk* = (0xFFFFFFFF shl DMA_CMAR6_MA_Pos) ## !< 0xFFFFFFFF
  DMA_CMAR6_MA* = DMA_CMAR6_MA_Msk

## *****************  Bit definition for DMA_CMAR7 register  ******************

const
  DMA_CMAR7_MA_Pos* = (0)
  DMA_CMAR7_MA_Msk* = (0xFFFFFFFF shl DMA_CMAR7_MA_Pos) ## !< 0xFFFFFFFF
  DMA_CMAR7_MA* = DMA_CMAR7_MA_Msk

## ****************************************************************************
##
##                   External Interrupt/Event Controller (EXTI)
##
## ****************************************************************************
## ******************  Bit definition for EXTI_IMR register  ******************

const
  EXTI_IMR_MR0_Pos* = (0)
  EXTI_IMR_MR0_Msk* = (0x00000001 shl EXTI_IMR_MR0_Pos) ## !< 0x00000001
  EXTI_IMR_MR0* = EXTI_IMR_MR0_Msk
  EXTI_IMR_MR1_Pos* = (1)
  EXTI_IMR_MR1_Msk* = (0x00000001 shl EXTI_IMR_MR1_Pos) ## !< 0x00000002
  EXTI_IMR_MR1* = EXTI_IMR_MR1_Msk
  EXTI_IMR_MR2_Pos* = (2)
  EXTI_IMR_MR2_Msk* = (0x00000001 shl EXTI_IMR_MR2_Pos) ## !< 0x00000004
  EXTI_IMR_MR2* = EXTI_IMR_MR2_Msk
  EXTI_IMR_MR3_Pos* = (3)
  EXTI_IMR_MR3_Msk* = (0x00000001 shl EXTI_IMR_MR3_Pos) ## !< 0x00000008
  EXTI_IMR_MR3* = EXTI_IMR_MR3_Msk
  EXTI_IMR_MR4_Pos* = (4)
  EXTI_IMR_MR4_Msk* = (0x00000001 shl EXTI_IMR_MR4_Pos) ## !< 0x00000010
  EXTI_IMR_MR4* = EXTI_IMR_MR4_Msk
  EXTI_IMR_MR5_Pos* = (5)
  EXTI_IMR_MR5_Msk* = (0x00000001 shl EXTI_IMR_MR5_Pos) ## !< 0x00000020
  EXTI_IMR_MR5* = EXTI_IMR_MR5_Msk
  EXTI_IMR_MR6_Pos* = (6)
  EXTI_IMR_MR6_Msk* = (0x00000001 shl EXTI_IMR_MR6_Pos) ## !< 0x00000040
  EXTI_IMR_MR6* = EXTI_IMR_MR6_Msk
  EXTI_IMR_MR7_Pos* = (7)
  EXTI_IMR_MR7_Msk* = (0x00000001 shl EXTI_IMR_MR7_Pos) ## !< 0x00000080
  EXTI_IMR_MR7* = EXTI_IMR_MR7_Msk
  EXTI_IMR_MR8_Pos* = (8)
  EXTI_IMR_MR8_Msk* = (0x00000001 shl EXTI_IMR_MR8_Pos) ## !< 0x00000100
  EXTI_IMR_MR8* = EXTI_IMR_MR8_Msk
  EXTI_IMR_MR9_Pos* = (9)
  EXTI_IMR_MR9_Msk* = (0x00000001 shl EXTI_IMR_MR9_Pos) ## !< 0x00000200
  EXTI_IMR_MR9* = EXTI_IMR_MR9_Msk
  EXTI_IMR_MR10_Pos* = (10)
  EXTI_IMR_MR10_Msk* = (0x00000001 shl EXTI_IMR_MR10_Pos) ## !< 0x00000400
  EXTI_IMR_MR10* = EXTI_IMR_MR10_Msk
  EXTI_IMR_MR11_Pos* = (11)
  EXTI_IMR_MR11_Msk* = (0x00000001 shl EXTI_IMR_MR11_Pos) ## !< 0x00000800
  EXTI_IMR_MR11* = EXTI_IMR_MR11_Msk
  EXTI_IMR_MR12_Pos* = (12)
  EXTI_IMR_MR12_Msk* = (0x00000001 shl EXTI_IMR_MR12_Pos) ## !< 0x00001000
  EXTI_IMR_MR12* = EXTI_IMR_MR12_Msk
  EXTI_IMR_MR13_Pos* = (13)
  EXTI_IMR_MR13_Msk* = (0x00000001 shl EXTI_IMR_MR13_Pos) ## !< 0x00002000
  EXTI_IMR_MR13* = EXTI_IMR_MR13_Msk
  EXTI_IMR_MR14_Pos* = (14)
  EXTI_IMR_MR14_Msk* = (0x00000001 shl EXTI_IMR_MR14_Pos) ## !< 0x00004000
  EXTI_IMR_MR14* = EXTI_IMR_MR14_Msk
  EXTI_IMR_MR15_Pos* = (15)
  EXTI_IMR_MR15_Msk* = (0x00000001 shl EXTI_IMR_MR15_Pos) ## !< 0x00008000
  EXTI_IMR_MR15* = EXTI_IMR_MR15_Msk
  EXTI_IMR_MR16_Pos* = (16)
  EXTI_IMR_MR16_Msk* = (0x00000001 shl EXTI_IMR_MR16_Pos) ## !< 0x00010000
  EXTI_IMR_MR16* = EXTI_IMR_MR16_Msk
  EXTI_IMR_MR17_Pos* = (17)
  EXTI_IMR_MR17_Msk* = (0x00000001 shl EXTI_IMR_MR17_Pos) ## !< 0x00020000
  EXTI_IMR_MR17* = EXTI_IMR_MR17_Msk
  EXTI_IMR_MR18_Pos* = (18)
  EXTI_IMR_MR18_Msk* = (0x00000001 shl EXTI_IMR_MR18_Pos) ## !< 0x00040000
  EXTI_IMR_MR18* = EXTI_IMR_MR18_Msk
  EXTI_IMR_MR19_Pos* = (19)
  EXTI_IMR_MR19_Msk* = (0x00000001 shl EXTI_IMR_MR19_Pos) ## !< 0x00080000
  EXTI_IMR_MR19* = EXTI_IMR_MR19_Msk
  EXTI_IMR_MR20_Pos* = (20)
  EXTI_IMR_MR20_Msk* = (0x00000001 shl EXTI_IMR_MR20_Pos) ## !< 0x00100000
  EXTI_IMR_MR20* = EXTI_IMR_MR20_Msk
  EXTI_IMR_MR21_Pos* = (21)
  EXTI_IMR_MR21_Msk* = (0x00000001 shl EXTI_IMR_MR21_Pos) ## !< 0x00200000
  EXTI_IMR_MR21* = EXTI_IMR_MR21_Msk
  EXTI_IMR_MR22_Pos* = (22)
  EXTI_IMR_MR22_Msk* = (0x00000001 shl EXTI_IMR_MR22_Pos) ## !< 0x00400000
  EXTI_IMR_MR22* = EXTI_IMR_MR22_Msk
  EXTI_IMR_MR23_Pos* = (23)
  EXTI_IMR_MR23_Msk* = (0x00000001 shl EXTI_IMR_MR23_Pos) ## !< 0x00800000
  EXTI_IMR_MR23* = EXTI_IMR_MR23_Msk

##  References Defines

const
  EXTI_IMR_IM0* = EXTI_IMR_MR0
  EXTI_IMR_IM1* = EXTI_IMR_MR1
  EXTI_IMR_IM2* = EXTI_IMR_MR2
  EXTI_IMR_IM3* = EXTI_IMR_MR3
  EXTI_IMR_IM4* = EXTI_IMR_MR4
  EXTI_IMR_IM5* = EXTI_IMR_MR5
  EXTI_IMR_IM6* = EXTI_IMR_MR6
  EXTI_IMR_IM7* = EXTI_IMR_MR7
  EXTI_IMR_IM8* = EXTI_IMR_MR8
  EXTI_IMR_IM9* = EXTI_IMR_MR9
  EXTI_IMR_IM10* = EXTI_IMR_MR10
  EXTI_IMR_IM11* = EXTI_IMR_MR11
  EXTI_IMR_IM12* = EXTI_IMR_MR12
  EXTI_IMR_IM13* = EXTI_IMR_MR13
  EXTI_IMR_IM14* = EXTI_IMR_MR14
  EXTI_IMR_IM15* = EXTI_IMR_MR15
  EXTI_IMR_IM16* = EXTI_IMR_MR16
  EXTI_IMR_IM17* = EXTI_IMR_MR17
  EXTI_IMR_IM18* = EXTI_IMR_MR18
  EXTI_IMR_IM19* = EXTI_IMR_MR19
  EXTI_IMR_IM20* = EXTI_IMR_MR20
  EXTI_IMR_IM21* = EXTI_IMR_MR21
  EXTI_IMR_IM22* = EXTI_IMR_MR22

##  Category 3, 4 & 5

const
  EXTI_IMR_IM23* = EXTI_IMR_MR23
  EXTI_IMR_IM_Pos* = (0)
  EXTI_IMR_IM_Msk* = (0x00FFFFFF shl EXTI_IMR_IM_Pos) ## !< 0x00FFFFFF
  EXTI_IMR_IM* = EXTI_IMR_IM_Msk

## ******************  Bit definition for EXTI_EMR register  ******************

const
  EXTI_EMR_MR0_Pos* = (0)
  EXTI_EMR_MR0_Msk* = (0x00000001 shl EXTI_EMR_MR0_Pos) ## !< 0x00000001
  EXTI_EMR_MR0* = EXTI_EMR_MR0_Msk
  EXTI_EMR_MR1_Pos* = (1)
  EXTI_EMR_MR1_Msk* = (0x00000001 shl EXTI_EMR_MR1_Pos) ## !< 0x00000002
  EXTI_EMR_MR1* = EXTI_EMR_MR1_Msk
  EXTI_EMR_MR2_Pos* = (2)
  EXTI_EMR_MR2_Msk* = (0x00000001 shl EXTI_EMR_MR2_Pos) ## !< 0x00000004
  EXTI_EMR_MR2* = EXTI_EMR_MR2_Msk
  EXTI_EMR_MR3_Pos* = (3)
  EXTI_EMR_MR3_Msk* = (0x00000001 shl EXTI_EMR_MR3_Pos) ## !< 0x00000008
  EXTI_EMR_MR3* = EXTI_EMR_MR3_Msk
  EXTI_EMR_MR4_Pos* = (4)
  EXTI_EMR_MR4_Msk* = (0x00000001 shl EXTI_EMR_MR4_Pos) ## !< 0x00000010
  EXTI_EMR_MR4* = EXTI_EMR_MR4_Msk
  EXTI_EMR_MR5_Pos* = (5)
  EXTI_EMR_MR5_Msk* = (0x00000001 shl EXTI_EMR_MR5_Pos) ## !< 0x00000020
  EXTI_EMR_MR5* = EXTI_EMR_MR5_Msk
  EXTI_EMR_MR6_Pos* = (6)
  EXTI_EMR_MR6_Msk* = (0x00000001 shl EXTI_EMR_MR6_Pos) ## !< 0x00000040
  EXTI_EMR_MR6* = EXTI_EMR_MR6_Msk
  EXTI_EMR_MR7_Pos* = (7)
  EXTI_EMR_MR7_Msk* = (0x00000001 shl EXTI_EMR_MR7_Pos) ## !< 0x00000080
  EXTI_EMR_MR7* = EXTI_EMR_MR7_Msk
  EXTI_EMR_MR8_Pos* = (8)
  EXTI_EMR_MR8_Msk* = (0x00000001 shl EXTI_EMR_MR8_Pos) ## !< 0x00000100
  EXTI_EMR_MR8* = EXTI_EMR_MR8_Msk
  EXTI_EMR_MR9_Pos* = (9)
  EXTI_EMR_MR9_Msk* = (0x00000001 shl EXTI_EMR_MR9_Pos) ## !< 0x00000200
  EXTI_EMR_MR9* = EXTI_EMR_MR9_Msk
  EXTI_EMR_MR10_Pos* = (10)
  EXTI_EMR_MR10_Msk* = (0x00000001 shl EXTI_EMR_MR10_Pos) ## !< 0x00000400
  EXTI_EMR_MR10* = EXTI_EMR_MR10_Msk
  EXTI_EMR_MR11_Pos* = (11)
  EXTI_EMR_MR11_Msk* = (0x00000001 shl EXTI_EMR_MR11_Pos) ## !< 0x00000800
  EXTI_EMR_MR11* = EXTI_EMR_MR11_Msk
  EXTI_EMR_MR12_Pos* = (12)
  EXTI_EMR_MR12_Msk* = (0x00000001 shl EXTI_EMR_MR12_Pos) ## !< 0x00001000
  EXTI_EMR_MR12* = EXTI_EMR_MR12_Msk
  EXTI_EMR_MR13_Pos* = (13)
  EXTI_EMR_MR13_Msk* = (0x00000001 shl EXTI_EMR_MR13_Pos) ## !< 0x00002000
  EXTI_EMR_MR13* = EXTI_EMR_MR13_Msk
  EXTI_EMR_MR14_Pos* = (14)
  EXTI_EMR_MR14_Msk* = (0x00000001 shl EXTI_EMR_MR14_Pos) ## !< 0x00004000
  EXTI_EMR_MR14* = EXTI_EMR_MR14_Msk
  EXTI_EMR_MR15_Pos* = (15)
  EXTI_EMR_MR15_Msk* = (0x00000001 shl EXTI_EMR_MR15_Pos) ## !< 0x00008000
  EXTI_EMR_MR15* = EXTI_EMR_MR15_Msk
  EXTI_EMR_MR16_Pos* = (16)
  EXTI_EMR_MR16_Msk* = (0x00000001 shl EXTI_EMR_MR16_Pos) ## !< 0x00010000
  EXTI_EMR_MR16* = EXTI_EMR_MR16_Msk
  EXTI_EMR_MR17_Pos* = (17)
  EXTI_EMR_MR17_Msk* = (0x00000001 shl EXTI_EMR_MR17_Pos) ## !< 0x00020000
  EXTI_EMR_MR17* = EXTI_EMR_MR17_Msk
  EXTI_EMR_MR18_Pos* = (18)
  EXTI_EMR_MR18_Msk* = (0x00000001 shl EXTI_EMR_MR18_Pos) ## !< 0x00040000
  EXTI_EMR_MR18* = EXTI_EMR_MR18_Msk
  EXTI_EMR_MR19_Pos* = (19)
  EXTI_EMR_MR19_Msk* = (0x00000001 shl EXTI_EMR_MR19_Pos) ## !< 0x00080000
  EXTI_EMR_MR19* = EXTI_EMR_MR19_Msk
  EXTI_EMR_MR20_Pos* = (20)
  EXTI_EMR_MR20_Msk* = (0x00000001 shl EXTI_EMR_MR20_Pos) ## !< 0x00100000
  EXTI_EMR_MR20* = EXTI_EMR_MR20_Msk
  EXTI_EMR_MR21_Pos* = (21)
  EXTI_EMR_MR21_Msk* = (0x00000001 shl EXTI_EMR_MR21_Pos) ## !< 0x00200000
  EXTI_EMR_MR21* = EXTI_EMR_MR21_Msk
  EXTI_EMR_MR22_Pos* = (22)
  EXTI_EMR_MR22_Msk* = (0x00000001 shl EXTI_EMR_MR22_Pos) ## !< 0x00400000
  EXTI_EMR_MR22* = EXTI_EMR_MR22_Msk
  EXTI_EMR_MR23_Pos* = (23)
  EXTI_EMR_MR23_Msk* = (0x00000001 shl EXTI_EMR_MR23_Pos) ## !< 0x00800000
  EXTI_EMR_MR23* = EXTI_EMR_MR23_Msk

##  References Defines

const
  EXTI_EMR_EM0* = EXTI_EMR_MR0
  EXTI_EMR_EM1* = EXTI_EMR_MR1
  EXTI_EMR_EM2* = EXTI_EMR_MR2
  EXTI_EMR_EM3* = EXTI_EMR_MR3
  EXTI_EMR_EM4* = EXTI_EMR_MR4
  EXTI_EMR_EM5* = EXTI_EMR_MR5
  EXTI_EMR_EM6* = EXTI_EMR_MR6
  EXTI_EMR_EM7* = EXTI_EMR_MR7
  EXTI_EMR_EM8* = EXTI_EMR_MR8
  EXTI_EMR_EM9* = EXTI_EMR_MR9
  EXTI_EMR_EM10* = EXTI_EMR_MR10
  EXTI_EMR_EM11* = EXTI_EMR_MR11
  EXTI_EMR_EM12* = EXTI_EMR_MR12
  EXTI_EMR_EM13* = EXTI_EMR_MR13
  EXTI_EMR_EM14* = EXTI_EMR_MR14
  EXTI_EMR_EM15* = EXTI_EMR_MR15
  EXTI_EMR_EM16* = EXTI_EMR_MR16
  EXTI_EMR_EM17* = EXTI_EMR_MR17
  EXTI_EMR_EM18* = EXTI_EMR_MR18
  EXTI_EMR_EM19* = EXTI_EMR_MR19
  EXTI_EMR_EM20* = EXTI_EMR_MR20
  EXTI_EMR_EM21* = EXTI_EMR_MR21
  EXTI_EMR_EM22* = EXTI_EMR_MR22
  EXTI_EMR_EM23* = EXTI_EMR_MR23

## *****************  Bit definition for EXTI_RTSR register  ******************

const
  EXTI_RTSR_TR0_Pos* = (0)
  EXTI_RTSR_TR0_Msk* = (0x00000001 shl EXTI_RTSR_TR0_Pos) ## !< 0x00000001
  EXTI_RTSR_TR0* = EXTI_RTSR_TR0_Msk
  EXTI_RTSR_TR1_Pos* = (1)
  EXTI_RTSR_TR1_Msk* = (0x00000001 shl EXTI_RTSR_TR1_Pos) ## !< 0x00000002
  EXTI_RTSR_TR1* = EXTI_RTSR_TR1_Msk
  EXTI_RTSR_TR2_Pos* = (2)
  EXTI_RTSR_TR2_Msk* = (0x00000001 shl EXTI_RTSR_TR2_Pos) ## !< 0x00000004
  EXTI_RTSR_TR2* = EXTI_RTSR_TR2_Msk
  EXTI_RTSR_TR3_Pos* = (3)
  EXTI_RTSR_TR3_Msk* = (0x00000001 shl EXTI_RTSR_TR3_Pos) ## !< 0x00000008
  EXTI_RTSR_TR3* = EXTI_RTSR_TR3_Msk
  EXTI_RTSR_TR4_Pos* = (4)
  EXTI_RTSR_TR4_Msk* = (0x00000001 shl EXTI_RTSR_TR4_Pos) ## !< 0x00000010
  EXTI_RTSR_TR4* = EXTI_RTSR_TR4_Msk
  EXTI_RTSR_TR5_Pos* = (5)
  EXTI_RTSR_TR5_Msk* = (0x00000001 shl EXTI_RTSR_TR5_Pos) ## !< 0x00000020
  EXTI_RTSR_TR5* = EXTI_RTSR_TR5_Msk
  EXTI_RTSR_TR6_Pos* = (6)
  EXTI_RTSR_TR6_Msk* = (0x00000001 shl EXTI_RTSR_TR6_Pos) ## !< 0x00000040
  EXTI_RTSR_TR6* = EXTI_RTSR_TR6_Msk
  EXTI_RTSR_TR7_Pos* = (7)
  EXTI_RTSR_TR7_Msk* = (0x00000001 shl EXTI_RTSR_TR7_Pos) ## !< 0x00000080
  EXTI_RTSR_TR7* = EXTI_RTSR_TR7_Msk
  EXTI_RTSR_TR8_Pos* = (8)
  EXTI_RTSR_TR8_Msk* = (0x00000001 shl EXTI_RTSR_TR8_Pos) ## !< 0x00000100
  EXTI_RTSR_TR8* = EXTI_RTSR_TR8_Msk
  EXTI_RTSR_TR9_Pos* = (9)
  EXTI_RTSR_TR9_Msk* = (0x00000001 shl EXTI_RTSR_TR9_Pos) ## !< 0x00000200
  EXTI_RTSR_TR9* = EXTI_RTSR_TR9_Msk
  EXTI_RTSR_TR10_Pos* = (10)
  EXTI_RTSR_TR10_Msk* = (0x00000001 shl EXTI_RTSR_TR10_Pos) ## !< 0x00000400
  EXTI_RTSR_TR10* = EXTI_RTSR_TR10_Msk
  EXTI_RTSR_TR11_Pos* = (11)
  EXTI_RTSR_TR11_Msk* = (0x00000001 shl EXTI_RTSR_TR11_Pos) ## !< 0x00000800
  EXTI_RTSR_TR11* = EXTI_RTSR_TR11_Msk
  EXTI_RTSR_TR12_Pos* = (12)
  EXTI_RTSR_TR12_Msk* = (0x00000001 shl EXTI_RTSR_TR12_Pos) ## !< 0x00001000
  EXTI_RTSR_TR12* = EXTI_RTSR_TR12_Msk
  EXTI_RTSR_TR13_Pos* = (13)
  EXTI_RTSR_TR13_Msk* = (0x00000001 shl EXTI_RTSR_TR13_Pos) ## !< 0x00002000
  EXTI_RTSR_TR13* = EXTI_RTSR_TR13_Msk
  EXTI_RTSR_TR14_Pos* = (14)
  EXTI_RTSR_TR14_Msk* = (0x00000001 shl EXTI_RTSR_TR14_Pos) ## !< 0x00004000
  EXTI_RTSR_TR14* = EXTI_RTSR_TR14_Msk
  EXTI_RTSR_TR15_Pos* = (15)
  EXTI_RTSR_TR15_Msk* = (0x00000001 shl EXTI_RTSR_TR15_Pos) ## !< 0x00008000
  EXTI_RTSR_TR15* = EXTI_RTSR_TR15_Msk
  EXTI_RTSR_TR16_Pos* = (16)
  EXTI_RTSR_TR16_Msk* = (0x00000001 shl EXTI_RTSR_TR16_Pos) ## !< 0x00010000
  EXTI_RTSR_TR16* = EXTI_RTSR_TR16_Msk
  EXTI_RTSR_TR17_Pos* = (17)
  EXTI_RTSR_TR17_Msk* = (0x00000001 shl EXTI_RTSR_TR17_Pos) ## !< 0x00020000
  EXTI_RTSR_TR17* = EXTI_RTSR_TR17_Msk
  EXTI_RTSR_TR18_Pos* = (18)
  EXTI_RTSR_TR18_Msk* = (0x00000001 shl EXTI_RTSR_TR18_Pos) ## !< 0x00040000
  EXTI_RTSR_TR18* = EXTI_RTSR_TR18_Msk
  EXTI_RTSR_TR19_Pos* = (19)
  EXTI_RTSR_TR19_Msk* = (0x00000001 shl EXTI_RTSR_TR19_Pos) ## !< 0x00080000
  EXTI_RTSR_TR19* = EXTI_RTSR_TR19_Msk
  EXTI_RTSR_TR20_Pos* = (20)
  EXTI_RTSR_TR20_Msk* = (0x00000001 shl EXTI_RTSR_TR20_Pos) ## !< 0x00100000
  EXTI_RTSR_TR20* = EXTI_RTSR_TR20_Msk
  EXTI_RTSR_TR21_Pos* = (21)
  EXTI_RTSR_TR21_Msk* = (0x00000001 shl EXTI_RTSR_TR21_Pos) ## !< 0x00200000
  EXTI_RTSR_TR21* = EXTI_RTSR_TR21_Msk
  EXTI_RTSR_TR22_Pos* = (22)
  EXTI_RTSR_TR22_Msk* = (0x00000001 shl EXTI_RTSR_TR22_Pos) ## !< 0x00400000
  EXTI_RTSR_TR22* = EXTI_RTSR_TR22_Msk
  EXTI_RTSR_TR23_Pos* = (23)
  EXTI_RTSR_TR23_Msk* = (0x00000001 shl EXTI_RTSR_TR23_Pos) ## !< 0x00800000
  EXTI_RTSR_TR23* = EXTI_RTSR_TR23_Msk

##  References Defines

const
  EXTI_RTSR_RT0* = EXTI_RTSR_TR0
  EXTI_RTSR_RT1* = EXTI_RTSR_TR1
  EXTI_RTSR_RT2* = EXTI_RTSR_TR2
  EXTI_RTSR_RT3* = EXTI_RTSR_TR3
  EXTI_RTSR_RT4* = EXTI_RTSR_TR4
  EXTI_RTSR_RT5* = EXTI_RTSR_TR5
  EXTI_RTSR_RT6* = EXTI_RTSR_TR6
  EXTI_RTSR_RT7* = EXTI_RTSR_TR7
  EXTI_RTSR_RT8* = EXTI_RTSR_TR8
  EXTI_RTSR_RT9* = EXTI_RTSR_TR9
  EXTI_RTSR_RT10* = EXTI_RTSR_TR10
  EXTI_RTSR_RT11* = EXTI_RTSR_TR11
  EXTI_RTSR_RT12* = EXTI_RTSR_TR12
  EXTI_RTSR_RT13* = EXTI_RTSR_TR13
  EXTI_RTSR_RT14* = EXTI_RTSR_TR14
  EXTI_RTSR_RT15* = EXTI_RTSR_TR15
  EXTI_RTSR_RT16* = EXTI_RTSR_TR16
  EXTI_RTSR_RT17* = EXTI_RTSR_TR17
  EXTI_RTSR_RT18* = EXTI_RTSR_TR18
  EXTI_RTSR_RT19* = EXTI_RTSR_TR19
  EXTI_RTSR_RT20* = EXTI_RTSR_TR20
  EXTI_RTSR_RT21* = EXTI_RTSR_TR21
  EXTI_RTSR_RT22* = EXTI_RTSR_TR22
  EXTI_RTSR_RT23* = EXTI_RTSR_TR23

## *****************  Bit definition for EXTI_FTSR register  ******************

const
  EXTI_FTSR_TR0_Pos* = (0)
  EXTI_FTSR_TR0_Msk* = (0x00000001 shl EXTI_FTSR_TR0_Pos) ## !< 0x00000001
  EXTI_FTSR_TR0* = EXTI_FTSR_TR0_Msk
  EXTI_FTSR_TR1_Pos* = (1)
  EXTI_FTSR_TR1_Msk* = (0x00000001 shl EXTI_FTSR_TR1_Pos) ## !< 0x00000002
  EXTI_FTSR_TR1* = EXTI_FTSR_TR1_Msk
  EXTI_FTSR_TR2_Pos* = (2)
  EXTI_FTSR_TR2_Msk* = (0x00000001 shl EXTI_FTSR_TR2_Pos) ## !< 0x00000004
  EXTI_FTSR_TR2* = EXTI_FTSR_TR2_Msk
  EXTI_FTSR_TR3_Pos* = (3)
  EXTI_FTSR_TR3_Msk* = (0x00000001 shl EXTI_FTSR_TR3_Pos) ## !< 0x00000008
  EXTI_FTSR_TR3* = EXTI_FTSR_TR3_Msk
  EXTI_FTSR_TR4_Pos* = (4)
  EXTI_FTSR_TR4_Msk* = (0x00000001 shl EXTI_FTSR_TR4_Pos) ## !< 0x00000010
  EXTI_FTSR_TR4* = EXTI_FTSR_TR4_Msk
  EXTI_FTSR_TR5_Pos* = (5)
  EXTI_FTSR_TR5_Msk* = (0x00000001 shl EXTI_FTSR_TR5_Pos) ## !< 0x00000020
  EXTI_FTSR_TR5* = EXTI_FTSR_TR5_Msk
  EXTI_FTSR_TR6_Pos* = (6)
  EXTI_FTSR_TR6_Msk* = (0x00000001 shl EXTI_FTSR_TR6_Pos) ## !< 0x00000040
  EXTI_FTSR_TR6* = EXTI_FTSR_TR6_Msk
  EXTI_FTSR_TR7_Pos* = (7)
  EXTI_FTSR_TR7_Msk* = (0x00000001 shl EXTI_FTSR_TR7_Pos) ## !< 0x00000080
  EXTI_FTSR_TR7* = EXTI_FTSR_TR7_Msk
  EXTI_FTSR_TR8_Pos* = (8)
  EXTI_FTSR_TR8_Msk* = (0x00000001 shl EXTI_FTSR_TR8_Pos) ## !< 0x00000100
  EXTI_FTSR_TR8* = EXTI_FTSR_TR8_Msk
  EXTI_FTSR_TR9_Pos* = (9)
  EXTI_FTSR_TR9_Msk* = (0x00000001 shl EXTI_FTSR_TR9_Pos) ## !< 0x00000200
  EXTI_FTSR_TR9* = EXTI_FTSR_TR9_Msk
  EXTI_FTSR_TR10_Pos* = (10)
  EXTI_FTSR_TR10_Msk* = (0x00000001 shl EXTI_FTSR_TR10_Pos) ## !< 0x00000400
  EXTI_FTSR_TR10* = EXTI_FTSR_TR10_Msk
  EXTI_FTSR_TR11_Pos* = (11)
  EXTI_FTSR_TR11_Msk* = (0x00000001 shl EXTI_FTSR_TR11_Pos) ## !< 0x00000800
  EXTI_FTSR_TR11* = EXTI_FTSR_TR11_Msk
  EXTI_FTSR_TR12_Pos* = (12)
  EXTI_FTSR_TR12_Msk* = (0x00000001 shl EXTI_FTSR_TR12_Pos) ## !< 0x00001000
  EXTI_FTSR_TR12* = EXTI_FTSR_TR12_Msk
  EXTI_FTSR_TR13_Pos* = (13)
  EXTI_FTSR_TR13_Msk* = (0x00000001 shl EXTI_FTSR_TR13_Pos) ## !< 0x00002000
  EXTI_FTSR_TR13* = EXTI_FTSR_TR13_Msk
  EXTI_FTSR_TR14_Pos* = (14)
  EXTI_FTSR_TR14_Msk* = (0x00000001 shl EXTI_FTSR_TR14_Pos) ## !< 0x00004000
  EXTI_FTSR_TR14* = EXTI_FTSR_TR14_Msk
  EXTI_FTSR_TR15_Pos* = (15)
  EXTI_FTSR_TR15_Msk* = (0x00000001 shl EXTI_FTSR_TR15_Pos) ## !< 0x00008000
  EXTI_FTSR_TR15* = EXTI_FTSR_TR15_Msk
  EXTI_FTSR_TR16_Pos* = (16)
  EXTI_FTSR_TR16_Msk* = (0x00000001 shl EXTI_FTSR_TR16_Pos) ## !< 0x00010000
  EXTI_FTSR_TR16* = EXTI_FTSR_TR16_Msk
  EXTI_FTSR_TR17_Pos* = (17)
  EXTI_FTSR_TR17_Msk* = (0x00000001 shl EXTI_FTSR_TR17_Pos) ## !< 0x00020000
  EXTI_FTSR_TR17* = EXTI_FTSR_TR17_Msk
  EXTI_FTSR_TR18_Pos* = (18)
  EXTI_FTSR_TR18_Msk* = (0x00000001 shl EXTI_FTSR_TR18_Pos) ## !< 0x00040000
  EXTI_FTSR_TR18* = EXTI_FTSR_TR18_Msk
  EXTI_FTSR_TR19_Pos* = (19)
  EXTI_FTSR_TR19_Msk* = (0x00000001 shl EXTI_FTSR_TR19_Pos) ## !< 0x00080000
  EXTI_FTSR_TR19* = EXTI_FTSR_TR19_Msk
  EXTI_FTSR_TR20_Pos* = (20)
  EXTI_FTSR_TR20_Msk* = (0x00000001 shl EXTI_FTSR_TR20_Pos) ## !< 0x00100000
  EXTI_FTSR_TR20* = EXTI_FTSR_TR20_Msk
  EXTI_FTSR_TR21_Pos* = (21)
  EXTI_FTSR_TR21_Msk* = (0x00000001 shl EXTI_FTSR_TR21_Pos) ## !< 0x00200000
  EXTI_FTSR_TR21* = EXTI_FTSR_TR21_Msk
  EXTI_FTSR_TR22_Pos* = (22)
  EXTI_FTSR_TR22_Msk* = (0x00000001 shl EXTI_FTSR_TR22_Pos) ## !< 0x00400000
  EXTI_FTSR_TR22* = EXTI_FTSR_TR22_Msk
  EXTI_FTSR_TR23_Pos* = (23)
  EXTI_FTSR_TR23_Msk* = (0x00000001 shl EXTI_FTSR_TR23_Pos) ## !< 0x00800000
  EXTI_FTSR_TR23* = EXTI_FTSR_TR23_Msk

##  References Defines

const
  EXTI_FTSR_FT0* = EXTI_FTSR_TR0
  EXTI_FTSR_FT1* = EXTI_FTSR_TR1
  EXTI_FTSR_FT2* = EXTI_FTSR_TR2
  EXTI_FTSR_FT3* = EXTI_FTSR_TR3
  EXTI_FTSR_FT4* = EXTI_FTSR_TR4
  EXTI_FTSR_FT5* = EXTI_FTSR_TR5
  EXTI_FTSR_FT6* = EXTI_FTSR_TR6
  EXTI_FTSR_FT7* = EXTI_FTSR_TR7
  EXTI_FTSR_FT8* = EXTI_FTSR_TR8
  EXTI_FTSR_FT9* = EXTI_FTSR_TR9
  EXTI_FTSR_FT10* = EXTI_FTSR_TR10
  EXTI_FTSR_FT11* = EXTI_FTSR_TR11
  EXTI_FTSR_FT12* = EXTI_FTSR_TR12
  EXTI_FTSR_FT13* = EXTI_FTSR_TR13
  EXTI_FTSR_FT14* = EXTI_FTSR_TR14
  EXTI_FTSR_FT15* = EXTI_FTSR_TR15
  EXTI_FTSR_FT16* = EXTI_FTSR_TR16
  EXTI_FTSR_FT17* = EXTI_FTSR_TR17
  EXTI_FTSR_FT18* = EXTI_FTSR_TR18
  EXTI_FTSR_FT19* = EXTI_FTSR_TR19
  EXTI_FTSR_FT20* = EXTI_FTSR_TR20
  EXTI_FTSR_FT21* = EXTI_FTSR_TR21
  EXTI_FTSR_FT22* = EXTI_FTSR_TR22
  EXTI_FTSR_FT23* = EXTI_FTSR_TR23

## *****************  Bit definition for EXTI_SWIER register  *****************

const
  EXTI_SWIER_SWIER0_Pos* = (0)
  EXTI_SWIER_SWIER0_Msk* = (0x00000001 shl EXTI_SWIER_SWIER0_Pos) ## !< 0x00000001
  EXTI_SWIER_SWIER0* = EXTI_SWIER_SWIER0_Msk
  EXTI_SWIER_SWIER1_Pos* = (1)
  EXTI_SWIER_SWIER1_Msk* = (0x00000001 shl EXTI_SWIER_SWIER1_Pos) ## !< 0x00000002
  EXTI_SWIER_SWIER1* = EXTI_SWIER_SWIER1_Msk
  EXTI_SWIER_SWIER2_Pos* = (2)
  EXTI_SWIER_SWIER2_Msk* = (0x00000001 shl EXTI_SWIER_SWIER2_Pos) ## !< 0x00000004
  EXTI_SWIER_SWIER2* = EXTI_SWIER_SWIER2_Msk
  EXTI_SWIER_SWIER3_Pos* = (3)
  EXTI_SWIER_SWIER3_Msk* = (0x00000001 shl EXTI_SWIER_SWIER3_Pos) ## !< 0x00000008
  EXTI_SWIER_SWIER3* = EXTI_SWIER_SWIER3_Msk
  EXTI_SWIER_SWIER4_Pos* = (4)
  EXTI_SWIER_SWIER4_Msk* = (0x00000001 shl EXTI_SWIER_SWIER4_Pos) ## !< 0x00000010
  EXTI_SWIER_SWIER4* = EXTI_SWIER_SWIER4_Msk
  EXTI_SWIER_SWIER5_Pos* = (5)
  EXTI_SWIER_SWIER5_Msk* = (0x00000001 shl EXTI_SWIER_SWIER5_Pos) ## !< 0x00000020
  EXTI_SWIER_SWIER5* = EXTI_SWIER_SWIER5_Msk
  EXTI_SWIER_SWIER6_Pos* = (6)
  EXTI_SWIER_SWIER6_Msk* = (0x00000001 shl EXTI_SWIER_SWIER6_Pos) ## !< 0x00000040
  EXTI_SWIER_SWIER6* = EXTI_SWIER_SWIER6_Msk
  EXTI_SWIER_SWIER7_Pos* = (7)
  EXTI_SWIER_SWIER7_Msk* = (0x00000001 shl EXTI_SWIER_SWIER7_Pos) ## !< 0x00000080
  EXTI_SWIER_SWIER7* = EXTI_SWIER_SWIER7_Msk
  EXTI_SWIER_SWIER8_Pos* = (8)
  EXTI_SWIER_SWIER8_Msk* = (0x00000001 shl EXTI_SWIER_SWIER8_Pos) ## !< 0x00000100
  EXTI_SWIER_SWIER8* = EXTI_SWIER_SWIER8_Msk
  EXTI_SWIER_SWIER9_Pos* = (9)
  EXTI_SWIER_SWIER9_Msk* = (0x00000001 shl EXTI_SWIER_SWIER9_Pos) ## !< 0x00000200
  EXTI_SWIER_SWIER9* = EXTI_SWIER_SWIER9_Msk
  EXTI_SWIER_SWIER10_Pos* = (10)
  EXTI_SWIER_SWIER10_Msk* = (0x00000001 shl EXTI_SWIER_SWIER10_Pos) ## !< 0x00000400
  EXTI_SWIER_SWIER10* = EXTI_SWIER_SWIER10_Msk
  EXTI_SWIER_SWIER11_Pos* = (11)
  EXTI_SWIER_SWIER11_Msk* = (0x00000001 shl EXTI_SWIER_SWIER11_Pos) ## !< 0x00000800
  EXTI_SWIER_SWIER11* = EXTI_SWIER_SWIER11_Msk
  EXTI_SWIER_SWIER12_Pos* = (12)
  EXTI_SWIER_SWIER12_Msk* = (0x00000001 shl EXTI_SWIER_SWIER12_Pos) ## !< 0x00001000
  EXTI_SWIER_SWIER12* = EXTI_SWIER_SWIER12_Msk
  EXTI_SWIER_SWIER13_Pos* = (13)
  EXTI_SWIER_SWIER13_Msk* = (0x00000001 shl EXTI_SWIER_SWIER13_Pos) ## !< 0x00002000
  EXTI_SWIER_SWIER13* = EXTI_SWIER_SWIER13_Msk
  EXTI_SWIER_SWIER14_Pos* = (14)
  EXTI_SWIER_SWIER14_Msk* = (0x00000001 shl EXTI_SWIER_SWIER14_Pos) ## !< 0x00004000
  EXTI_SWIER_SWIER14* = EXTI_SWIER_SWIER14_Msk
  EXTI_SWIER_SWIER15_Pos* = (15)
  EXTI_SWIER_SWIER15_Msk* = (0x00000001 shl EXTI_SWIER_SWIER15_Pos) ## !< 0x00008000
  EXTI_SWIER_SWIER15* = EXTI_SWIER_SWIER15_Msk
  EXTI_SWIER_SWIER16_Pos* = (16)
  EXTI_SWIER_SWIER16_Msk* = (0x00000001 shl EXTI_SWIER_SWIER16_Pos) ## !< 0x00010000
  EXTI_SWIER_SWIER16* = EXTI_SWIER_SWIER16_Msk
  EXTI_SWIER_SWIER17_Pos* = (17)
  EXTI_SWIER_SWIER17_Msk* = (0x00000001 shl EXTI_SWIER_SWIER17_Pos) ## !< 0x00020000
  EXTI_SWIER_SWIER17* = EXTI_SWIER_SWIER17_Msk
  EXTI_SWIER_SWIER18_Pos* = (18)
  EXTI_SWIER_SWIER18_Msk* = (0x00000001 shl EXTI_SWIER_SWIER18_Pos) ## !< 0x00040000
  EXTI_SWIER_SWIER18* = EXTI_SWIER_SWIER18_Msk
  EXTI_SWIER_SWIER19_Pos* = (19)
  EXTI_SWIER_SWIER19_Msk* = (0x00000001 shl EXTI_SWIER_SWIER19_Pos) ## !< 0x00080000
  EXTI_SWIER_SWIER19* = EXTI_SWIER_SWIER19_Msk
  EXTI_SWIER_SWIER20_Pos* = (20)
  EXTI_SWIER_SWIER20_Msk* = (0x00000001 shl EXTI_SWIER_SWIER20_Pos) ## !< 0x00100000
  EXTI_SWIER_SWIER20* = EXTI_SWIER_SWIER20_Msk
  EXTI_SWIER_SWIER21_Pos* = (21)
  EXTI_SWIER_SWIER21_Msk* = (0x00000001 shl EXTI_SWIER_SWIER21_Pos) ## !< 0x00200000
  EXTI_SWIER_SWIER21* = EXTI_SWIER_SWIER21_Msk
  EXTI_SWIER_SWIER22_Pos* = (22)
  EXTI_SWIER_SWIER22_Msk* = (0x00000001 shl EXTI_SWIER_SWIER22_Pos) ## !< 0x00400000
  EXTI_SWIER_SWIER22* = EXTI_SWIER_SWIER22_Msk
  EXTI_SWIER_SWIER23_Pos* = (23)
  EXTI_SWIER_SWIER23_Msk* = (0x00000001 shl EXTI_SWIER_SWIER23_Pos) ## !< 0x00800000
  EXTI_SWIER_SWIER23* = EXTI_SWIER_SWIER23_Msk

##  References Defines

const
  EXTI_SWIER_SWI0* = EXTI_SWIER_SWIER0
  EXTI_SWIER_SWI1* = EXTI_SWIER_SWIER1
  EXTI_SWIER_SWI2* = EXTI_SWIER_SWIER2
  EXTI_SWIER_SWI3* = EXTI_SWIER_SWIER3
  EXTI_SWIER_SWI4* = EXTI_SWIER_SWIER4
  EXTI_SWIER_SWI5* = EXTI_SWIER_SWIER5
  EXTI_SWIER_SWI6* = EXTI_SWIER_SWIER6
  EXTI_SWIER_SWI7* = EXTI_SWIER_SWIER7
  EXTI_SWIER_SWI8* = EXTI_SWIER_SWIER8
  EXTI_SWIER_SWI9* = EXTI_SWIER_SWIER9
  EXTI_SWIER_SWI10* = EXTI_SWIER_SWIER10
  EXTI_SWIER_SWI11* = EXTI_SWIER_SWIER11
  EXTI_SWIER_SWI12* = EXTI_SWIER_SWIER12
  EXTI_SWIER_SWI13* = EXTI_SWIER_SWIER13
  EXTI_SWIER_SWI14* = EXTI_SWIER_SWIER14
  EXTI_SWIER_SWI15* = EXTI_SWIER_SWIER15
  EXTI_SWIER_SWI16* = EXTI_SWIER_SWIER16
  EXTI_SWIER_SWI17* = EXTI_SWIER_SWIER17
  EXTI_SWIER_SWI18* = EXTI_SWIER_SWIER18
  EXTI_SWIER_SWI19* = EXTI_SWIER_SWIER19
  EXTI_SWIER_SWI20* = EXTI_SWIER_SWIER20
  EXTI_SWIER_SWI21* = EXTI_SWIER_SWIER21
  EXTI_SWIER_SWI22* = EXTI_SWIER_SWIER22
  EXTI_SWIER_SWI23* = EXTI_SWIER_SWIER23

## ******************  Bit definition for EXTI_PR register  *******************

const
  EXTI_PR_PR0_Pos* = (0)
  EXTI_PR_PR0_Msk* = (0x00000001 shl EXTI_PR_PR0_Pos) ## !< 0x00000001
  EXTI_PR_PR0* = EXTI_PR_PR0_Msk
  EXTI_PR_PR1_Pos* = (1)
  EXTI_PR_PR1_Msk* = (0x00000001 shl EXTI_PR_PR1_Pos) ## !< 0x00000002
  EXTI_PR_PR1* = EXTI_PR_PR1_Msk
  EXTI_PR_PR2_Pos* = (2)
  EXTI_PR_PR2_Msk* = (0x00000001 shl EXTI_PR_PR2_Pos) ## !< 0x00000004
  EXTI_PR_PR2* = EXTI_PR_PR2_Msk
  EXTI_PR_PR3_Pos* = (3)
  EXTI_PR_PR3_Msk* = (0x00000001 shl EXTI_PR_PR3_Pos) ## !< 0x00000008
  EXTI_PR_PR3* = EXTI_PR_PR3_Msk
  EXTI_PR_PR4_Pos* = (4)
  EXTI_PR_PR4_Msk* = (0x00000001 shl EXTI_PR_PR4_Pos) ## !< 0x00000010
  EXTI_PR_PR4* = EXTI_PR_PR4_Msk
  EXTI_PR_PR5_Pos* = (5)
  EXTI_PR_PR5_Msk* = (0x00000001 shl EXTI_PR_PR5_Pos) ## !< 0x00000020
  EXTI_PR_PR5* = EXTI_PR_PR5_Msk
  EXTI_PR_PR6_Pos* = (6)
  EXTI_PR_PR6_Msk* = (0x00000001 shl EXTI_PR_PR6_Pos) ## !< 0x00000040
  EXTI_PR_PR6* = EXTI_PR_PR6_Msk
  EXTI_PR_PR7_Pos* = (7)
  EXTI_PR_PR7_Msk* = (0x00000001 shl EXTI_PR_PR7_Pos) ## !< 0x00000080
  EXTI_PR_PR7* = EXTI_PR_PR7_Msk
  EXTI_PR_PR8_Pos* = (8)
  EXTI_PR_PR8_Msk* = (0x00000001 shl EXTI_PR_PR8_Pos) ## !< 0x00000100
  EXTI_PR_PR8* = EXTI_PR_PR8_Msk
  EXTI_PR_PR9_Pos* = (9)
  EXTI_PR_PR9_Msk* = (0x00000001 shl EXTI_PR_PR9_Pos) ## !< 0x00000200
  EXTI_PR_PR9* = EXTI_PR_PR9_Msk
  EXTI_PR_PR10_Pos* = (10)
  EXTI_PR_PR10_Msk* = (0x00000001 shl EXTI_PR_PR10_Pos) ## !< 0x00000400
  EXTI_PR_PR10* = EXTI_PR_PR10_Msk
  EXTI_PR_PR11_Pos* = (11)
  EXTI_PR_PR11_Msk* = (0x00000001 shl EXTI_PR_PR11_Pos) ## !< 0x00000800
  EXTI_PR_PR11* = EXTI_PR_PR11_Msk
  EXTI_PR_PR12_Pos* = (12)
  EXTI_PR_PR12_Msk* = (0x00000001 shl EXTI_PR_PR12_Pos) ## !< 0x00001000
  EXTI_PR_PR12* = EXTI_PR_PR12_Msk
  EXTI_PR_PR13_Pos* = (13)
  EXTI_PR_PR13_Msk* = (0x00000001 shl EXTI_PR_PR13_Pos) ## !< 0x00002000
  EXTI_PR_PR13* = EXTI_PR_PR13_Msk
  EXTI_PR_PR14_Pos* = (14)
  EXTI_PR_PR14_Msk* = (0x00000001 shl EXTI_PR_PR14_Pos) ## !< 0x00004000
  EXTI_PR_PR14* = EXTI_PR_PR14_Msk
  EXTI_PR_PR15_Pos* = (15)
  EXTI_PR_PR15_Msk* = (0x00000001 shl EXTI_PR_PR15_Pos) ## !< 0x00008000
  EXTI_PR_PR15* = EXTI_PR_PR15_Msk
  EXTI_PR_PR16_Pos* = (16)
  EXTI_PR_PR16_Msk* = (0x00000001 shl EXTI_PR_PR16_Pos) ## !< 0x00010000
  EXTI_PR_PR16* = EXTI_PR_PR16_Msk
  EXTI_PR_PR17_Pos* = (17)
  EXTI_PR_PR17_Msk* = (0x00000001 shl EXTI_PR_PR17_Pos) ## !< 0x00020000
  EXTI_PR_PR17* = EXTI_PR_PR17_Msk
  EXTI_PR_PR18_Pos* = (18)
  EXTI_PR_PR18_Msk* = (0x00000001 shl EXTI_PR_PR18_Pos) ## !< 0x00040000
  EXTI_PR_PR18* = EXTI_PR_PR18_Msk
  EXTI_PR_PR19_Pos* = (19)
  EXTI_PR_PR19_Msk* = (0x00000001 shl EXTI_PR_PR19_Pos) ## !< 0x00080000
  EXTI_PR_PR19* = EXTI_PR_PR19_Msk
  EXTI_PR_PR20_Pos* = (20)
  EXTI_PR_PR20_Msk* = (0x00000001 shl EXTI_PR_PR20_Pos) ## !< 0x00100000
  EXTI_PR_PR20* = EXTI_PR_PR20_Msk
  EXTI_PR_PR21_Pos* = (21)
  EXTI_PR_PR21_Msk* = (0x00000001 shl EXTI_PR_PR21_Pos) ## !< 0x00200000
  EXTI_PR_PR21* = EXTI_PR_PR21_Msk
  EXTI_PR_PR22_Pos* = (22)
  EXTI_PR_PR22_Msk* = (0x00000001 shl EXTI_PR_PR22_Pos) ## !< 0x00400000
  EXTI_PR_PR22* = EXTI_PR_PR22_Msk
  EXTI_PR_PR23_Pos* = (23)
  EXTI_PR_PR23_Msk* = (0x00000001 shl EXTI_PR_PR23_Pos) ## !< 0x00800000
  EXTI_PR_PR23* = EXTI_PR_PR23_Msk

##  References Defines

const
  EXTI_PR_PIF0* = EXTI_PR_PR0
  EXTI_PR_PIF1* = EXTI_PR_PR1
  EXTI_PR_PIF2* = EXTI_PR_PR2
  EXTI_PR_PIF3* = EXTI_PR_PR3
  EXTI_PR_PIF4* = EXTI_PR_PR4
  EXTI_PR_PIF5* = EXTI_PR_PR5
  EXTI_PR_PIF6* = EXTI_PR_PR6
  EXTI_PR_PIF7* = EXTI_PR_PR7
  EXTI_PR_PIF8* = EXTI_PR_PR8
  EXTI_PR_PIF9* = EXTI_PR_PR9
  EXTI_PR_PIF10* = EXTI_PR_PR10
  EXTI_PR_PIF11* = EXTI_PR_PR11
  EXTI_PR_PIF12* = EXTI_PR_PR12
  EXTI_PR_PIF13* = EXTI_PR_PR13
  EXTI_PR_PIF14* = EXTI_PR_PR14
  EXTI_PR_PIF15* = EXTI_PR_PR15
  EXTI_PR_PIF16* = EXTI_PR_PR16
  EXTI_PR_PIF17* = EXTI_PR_PR17
  EXTI_PR_PIF18* = EXTI_PR_PR18
  EXTI_PR_PIF19* = EXTI_PR_PR19
  EXTI_PR_PIF20* = EXTI_PR_PR20
  EXTI_PR_PIF21* = EXTI_PR_PR21
  EXTI_PR_PIF22* = EXTI_PR_PR22
  EXTI_PR_PIF23* = EXTI_PR_PR23

## ****************************************************************************
##
##                 FLASH, DATA EEPROM and Option Bytes Registers
##                         (FLASH, DATA_EEPROM, OB)
##
## ****************************************************************************
## ******************  Bit definition for FLASH_ACR register  *****************

const
  FLASH_ACR_LATENCY_Pos* = (0)
  FLASH_ACR_LATENCY_Msk* = (0x00000001 shl FLASH_ACR_LATENCY_Pos) ## !< 0x00000001
  FLASH_ACR_LATENCY* = FLASH_ACR_LATENCY_Msk
  FLASH_ACR_PRFTEN_Pos* = (1)
  FLASH_ACR_PRFTEN_Msk* = (0x00000001 shl FLASH_ACR_PRFTEN_Pos) ## !< 0x00000002
  FLASH_ACR_PRFTEN* = FLASH_ACR_PRFTEN_Msk
  FLASH_ACR_ACC64_Pos* = (2)
  FLASH_ACR_ACC64_Msk* = (0x00000001 shl FLASH_ACR_ACC64_Pos) ## !< 0x00000004
  FLASH_ACR_ACC64* = FLASH_ACR_ACC64_Msk
  FLASH_ACR_SLEEP_PD_Pos* = (3)
  FLASH_ACR_SLEEP_PD_Msk* = (0x00000001 shl FLASH_ACR_SLEEP_PD_Pos) ## !< 0x00000008
  FLASH_ACR_SLEEP_PD* = FLASH_ACR_SLEEP_PD_Msk
  FLASH_ACR_RUN_PD_Pos* = (4)
  FLASH_ACR_RUN_PD_Msk* = (0x00000001 shl FLASH_ACR_RUN_PD_Pos) ## !< 0x00000010
  FLASH_ACR_RUN_PD* = FLASH_ACR_RUN_PD_Msk

## ******************  Bit definition for FLASH_PECR register  *****************

const
  FLASH_PECR_PELOCK_Pos* = (0)
  FLASH_PECR_PELOCK_Msk* = (0x00000001 shl FLASH_PECR_PELOCK_Pos) ## !< 0x00000001
  FLASH_PECR_PELOCK* = FLASH_PECR_PELOCK_Msk
  FLASH_PECR_PRGLOCK_Pos* = (1)
  FLASH_PECR_PRGLOCK_Msk* = (0x00000001 shl FLASH_PECR_PRGLOCK_Pos) ## !< 0x00000002
  FLASH_PECR_PRGLOCK* = FLASH_PECR_PRGLOCK_Msk
  FLASH_PECR_OPTLOCK_Pos* = (2)
  FLASH_PECR_OPTLOCK_Msk* = (0x00000001 shl FLASH_PECR_OPTLOCK_Pos) ## !< 0x00000004
  FLASH_PECR_OPTLOCK* = FLASH_PECR_OPTLOCK_Msk
  FLASH_PECR_PROG_Pos* = (3)
  FLASH_PECR_PROG_Msk* = (0x00000001 shl FLASH_PECR_PROG_Pos) ## !< 0x00000008
  FLASH_PECR_PROG* = FLASH_PECR_PROG_Msk
  FLASH_PECR_DATA_Pos* = (4)
  FLASH_PECR_DATA_Msk* = (0x00000001 shl FLASH_PECR_DATA_Pos) ## !< 0x00000010
  FLASH_PECR_DATA* = FLASH_PECR_DATA_Msk
  FLASH_PECR_FTDW_Pos* = (8)
  FLASH_PECR_FTDW_Msk* = (0x00000001 shl FLASH_PECR_FTDW_Pos) ## !< 0x00000100
  FLASH_PECR_FTDW* = FLASH_PECR_FTDW_Msk
  FLASH_PECR_ERASE_Pos* = (9)
  FLASH_PECR_ERASE_Msk* = (0x00000001 shl FLASH_PECR_ERASE_Pos) ## !< 0x00000200
  FLASH_PECR_ERASE* = FLASH_PECR_ERASE_Msk
  FLASH_PECR_FPRG_Pos* = (10)
  FLASH_PECR_FPRG_Msk* = (0x00000001 shl FLASH_PECR_FPRG_Pos) ## !< 0x00000400
  FLASH_PECR_FPRG* = FLASH_PECR_FPRG_Msk
  FLASH_PECR_PARALLBANK_Pos* = (15)
  FLASH_PECR_PARALLBANK_Msk* = (0x00000001 shl FLASH_PECR_PARALLBANK_Pos) ## !< 0x00008000
  FLASH_PECR_PARALLBANK* = FLASH_PECR_PARALLBANK_Msk
  FLASH_PECR_EOPIE_Pos* = (16)
  FLASH_PECR_EOPIE_Msk* = (0x00000001 shl FLASH_PECR_EOPIE_Pos) ## !< 0x00010000
  FLASH_PECR_EOPIE* = FLASH_PECR_EOPIE_Msk
  FLASH_PECR_ERRIE_Pos* = (17)
  FLASH_PECR_ERRIE_Msk* = (0x00000001 shl FLASH_PECR_ERRIE_Pos) ## !< 0x00020000
  FLASH_PECR_ERRIE* = FLASH_PECR_ERRIE_Msk
  FLASH_PECR_OBL_LAUNCH_Pos* = (18)
  FLASH_PECR_OBL_LAUNCH_Msk* = (0x00000001 shl FLASH_PECR_OBL_LAUNCH_Pos) ## !< 0x00040000
  FLASH_PECR_OBL_LAUNCH* = FLASH_PECR_OBL_LAUNCH_Msk

## *****************  Bit definition for FLASH_PDKEYR register  *****************

const
  FLASH_PDKEYR_PDKEYR_Pos* = (0)
  FLASH_PDKEYR_PDKEYR_Msk* = (0xFFFFFFFF shl FLASH_PDKEYR_PDKEYR_Pos) ## !< 0xFFFFFFFF
  FLASH_PDKEYR_PDKEYR* = FLASH_PDKEYR_PDKEYR_Msk

## *****************  Bit definition for FLASH_PEKEYR register  *****************

const
  FLASH_PEKEYR_PEKEYR_Pos* = (0)
  FLASH_PEKEYR_PEKEYR_Msk* = (0xFFFFFFFF shl FLASH_PEKEYR_PEKEYR_Pos) ## !< 0xFFFFFFFF
  FLASH_PEKEYR_PEKEYR* = FLASH_PEKEYR_PEKEYR_Msk

## *****************  Bit definition for FLASH_PRGKEYR register  *****************

const
  FLASH_PRGKEYR_PRGKEYR_Pos* = (0)
  FLASH_PRGKEYR_PRGKEYR_Msk* = (0xFFFFFFFF shl FLASH_PRGKEYR_PRGKEYR_Pos) ## !< 0xFFFFFFFF
  FLASH_PRGKEYR_PRGKEYR* = FLASH_PRGKEYR_PRGKEYR_Msk

## *****************  Bit definition for FLASH_OPTKEYR register  *****************

const
  FLASH_OPTKEYR_OPTKEYR_Pos* = (0)
  FLASH_OPTKEYR_OPTKEYR_Msk* = (0xFFFFFFFF shl FLASH_OPTKEYR_OPTKEYR_Pos) ## !< 0xFFFFFFFF
  FLASH_OPTKEYR_OPTKEYR* = FLASH_OPTKEYR_OPTKEYR_Msk

## *****************  Bit definition for FLASH_SR register  ******************

const
  FLASH_SR_BSY_Pos* = (0)
  FLASH_SR_BSY_Msk* = (0x00000001 shl FLASH_SR_BSY_Pos) ## !< 0x00000001
  FLASH_SR_BSY* = FLASH_SR_BSY_Msk
  FLASH_SR_EOP_Pos* = (1)
  FLASH_SR_EOP_Msk* = (0x00000001 shl FLASH_SR_EOP_Pos) ## !< 0x00000002
  FLASH_SR_EOP* = FLASH_SR_EOP_Msk
  FLASH_SR_ENDHV_Pos* = (2)
  FLASH_SR_ENDHV_Msk* = (0x00000001 shl FLASH_SR_ENDHV_Pos) ## !< 0x00000004
  FLASH_SR_ENDHV* = FLASH_SR_ENDHV_Msk
  FLASH_SR_READY_Pos* = (3)
  FLASH_SR_READY_Msk* = (0x00000001 shl FLASH_SR_READY_Pos) ## !< 0x00000008
  FLASH_SR_READY* = FLASH_SR_READY_Msk
  FLASH_SR_WRPERR_Pos* = (8)
  FLASH_SR_WRPERR_Msk* = (0x00000001 shl FLASH_SR_WRPERR_Pos) ## !< 0x00000100
  FLASH_SR_WRPERR* = FLASH_SR_WRPERR_Msk
  FLASH_SR_PGAERR_Pos* = (9)
  FLASH_SR_PGAERR_Msk* = (0x00000001 shl FLASH_SR_PGAERR_Pos) ## !< 0x00000200
  FLASH_SR_PGAERR* = FLASH_SR_PGAERR_Msk
  FLASH_SR_SIZERR_Pos* = (10)
  FLASH_SR_SIZERR_Msk* = (0x00000001 shl FLASH_SR_SIZERR_Pos) ## !< 0x00000400
  FLASH_SR_SIZERR* = FLASH_SR_SIZERR_Msk
  FLASH_SR_OPTVERR_Pos* = (11)
  FLASH_SR_OPTVERR_Msk* = (0x00000001 shl FLASH_SR_OPTVERR_Pos) ## !< 0x00000800
  FLASH_SR_OPTVERR* = FLASH_SR_OPTVERR_Msk
  FLASH_SR_OPTVERRUSR_Pos* = (12)
  FLASH_SR_OPTVERRUSR_Msk* = (0x00000001 shl FLASH_SR_OPTVERRUSR_Pos) ## !< 0x00001000
  FLASH_SR_OPTVERRUSR* = FLASH_SR_OPTVERRUSR_Msk

## *****************  Bit definition for FLASH_OBR register  ******************

const
  FLASH_OBR_RDPRT_Pos* = (0)
  FLASH_OBR_RDPRT_Msk* = (0x000000FF shl FLASH_OBR_RDPRT_Pos) ## !< 0x000000FF
  FLASH_OBR_RDPRT* = FLASH_OBR_RDPRT_Msk
  FLASH_OBR_BOR_LEV_Pos* = (16)
  FLASH_OBR_BOR_LEV_Msk* = (0x0000000F shl FLASH_OBR_BOR_LEV_Pos) ## !< 0x000F0000
  FLASH_OBR_BOR_LEV* = FLASH_OBR_BOR_LEV_Msk
  FLASH_OBR_USER_Pos* = (20)
  FLASH_OBR_USER_Msk* = (0x0000000F shl FLASH_OBR_USER_Pos) ## !< 0x00F00000
  FLASH_OBR_USER* = FLASH_OBR_USER_Msk
  FLASH_OBR_IWDG_SW_Pos* = (20)
  FLASH_OBR_IWDG_SW_Msk* = (0x00000001 shl FLASH_OBR_IWDG_SW_Pos) ## !< 0x00100000
  FLASH_OBR_IWDG_SW* = FLASH_OBR_IWDG_SW_Msk
  FLASH_OBR_nRST_STOP_Pos* = (21)
  FLASH_OBR_nRST_STOP_Msk* = (0x00000001 shl FLASH_OBR_nRST_STOP_Pos) ## !< 0x00200000
  FLASH_OBR_nRST_STOP* = FLASH_OBR_nRST_STOP_Msk
  FLASH_OBR_nRST_STDBY_Pos* = (22)
  FLASH_OBR_nRST_STDBY_Msk* = (0x00000001 shl FLASH_OBR_nRST_STDBY_Pos) ## !< 0x00400000
  FLASH_OBR_nRST_STDBY* = FLASH_OBR_nRST_STDBY_Msk
  FLASH_OBR_nRST_BFB2_Pos* = (23)
  FLASH_OBR_nRST_BFB2_Msk* = (0x00000001 shl FLASH_OBR_nRST_BFB2_Pos) ## !< 0x00800000
  FLASH_OBR_nRST_BFB2* = FLASH_OBR_nRST_BFB2_Msk

## *****************  Bit definition for FLASH_WRPR register  *****************

const
  FLASH_WRPR1_WRP_Pos* = (0)
  FLASH_WRPR1_WRP_Msk* = (0xFFFFFFFF shl FLASH_WRPR1_WRP_Pos) ## !< 0xFFFFFFFF
  FLASH_WRPR1_WRP* = FLASH_WRPR1_WRP_Msk
  FLASH_WRPR2_WRP_Pos* = (0)
  FLASH_WRPR2_WRP_Msk* = (0xFFFFFFFF shl FLASH_WRPR2_WRP_Pos) ## !< 0xFFFFFFFF
  FLASH_WRPR2_WRP* = FLASH_WRPR2_WRP_Msk
  FLASH_WRPR3_WRP_Pos* = (0)
  FLASH_WRPR3_WRP_Msk* = (0xFFFFFFFF shl FLASH_WRPR3_WRP_Pos) ## !< 0xFFFFFFFF
  FLASH_WRPR3_WRP* = FLASH_WRPR3_WRP_Msk
  FLASH_WRPR4_WRP_Pos* = (0)
  FLASH_WRPR4_WRP_Msk* = (0xFFFFFFFF shl FLASH_WRPR4_WRP_Pos) ## !< 0xFFFFFFFF
  FLASH_WRPR4_WRP* = FLASH_WRPR4_WRP_Msk

## ****************************************************************************
##
##                             General Purpose I/O
##
## ****************************************************************************
## *****************  Bits definition for GPIO_MODER register  ****************

const
  GPIO_MODER_MODER0_Pos* = (0)
  GPIO_MODER_MODER0_Msk* = (0x00000003 shl GPIO_MODER_MODER0_Pos) ## !< 0x00000003
  GPIO_MODER_MODER0* = GPIO_MODER_MODER0_Msk
  GPIO_MODER_MODER0_0* = (0x00000001 shl GPIO_MODER_MODER0_Pos) ## !< 0x00000001
  GPIO_MODER_MODER0_1* = (0x00000002 shl GPIO_MODER_MODER0_Pos) ## !< 0x00000002
  GPIO_MODER_MODER1_Pos* = (2)
  GPIO_MODER_MODER1_Msk* = (0x00000003 shl GPIO_MODER_MODER1_Pos) ## !< 0x0000000C
  GPIO_MODER_MODER1* = GPIO_MODER_MODER1_Msk
  GPIO_MODER_MODER1_0x* = (0x00000001 shl GPIO_MODER_MODER1_Pos) ## !< 0x00000004
  GPIO_MODER_MODER1_1x* = (0x00000002 shl GPIO_MODER_MODER1_Pos) ## !< 0x00000008
  GPIO_MODER_MODER2_Pos* = (4)
  GPIO_MODER_MODER2_Msk* = (0x00000003 shl GPIO_MODER_MODER2_Pos) ## !< 0x00000030
  GPIO_MODER_MODER2* = GPIO_MODER_MODER2_Msk
  GPIO_MODER_MODER2_0* = (0x00000001 shl GPIO_MODER_MODER2_Pos) ## !< 0x00000010
  GPIO_MODER_MODER2_1* = (0x00000002 shl GPIO_MODER_MODER2_Pos) ## !< 0x00000020
  GPIO_MODER_MODER3_Pos* = (6)
  GPIO_MODER_MODER3_Msk* = (0x00000003 shl GPIO_MODER_MODER3_Pos) ## !< 0x000000C0
  GPIO_MODER_MODER3* = GPIO_MODER_MODER3_Msk
  GPIO_MODER_MODER3_0* = (0x00000001 shl GPIO_MODER_MODER3_Pos) ## !< 0x00000040
  GPIO_MODER_MODER3_1* = (0x00000002 shl GPIO_MODER_MODER3_Pos) ## !< 0x00000080
  GPIO_MODER_MODER4_Pos* = (8)
  GPIO_MODER_MODER4_Msk* = (0x00000003 shl GPIO_MODER_MODER4_Pos) ## !< 0x00000300
  GPIO_MODER_MODER4* = GPIO_MODER_MODER4_Msk
  GPIO_MODER_MODER4_0* = (0x00000001 shl GPIO_MODER_MODER4_Pos) ## !< 0x00000100
  GPIO_MODER_MODER4_1* = (0x00000002 shl GPIO_MODER_MODER4_Pos) ## !< 0x00000200
  GPIO_MODER_MODER5_Pos* = (10)
  GPIO_MODER_MODER5_Msk* = (0x00000003 shl GPIO_MODER_MODER5_Pos) ## !< 0x00000C00
  GPIO_MODER_MODER5* = GPIO_MODER_MODER5_Msk
  GPIO_MODER_MODER5_0* = (0x00000001 shl GPIO_MODER_MODER5_Pos) ## !< 0x00000400
  GPIO_MODER_MODER5_1* = (0x00000002 shl GPIO_MODER_MODER5_Pos) ## !< 0x00000800
  GPIO_MODER_MODER6_Pos* = (12)
  GPIO_MODER_MODER6_Msk* = (0x00000003 shl GPIO_MODER_MODER6_Pos) ## !< 0x00003000
  GPIO_MODER_MODER6* = GPIO_MODER_MODER6_Msk
  GPIO_MODER_MODER6_0* = (0x00000001 shl GPIO_MODER_MODER6_Pos) ## !< 0x00001000
  GPIO_MODER_MODER6_1* = (0x00000002 shl GPIO_MODER_MODER6_Pos) ## !< 0x00002000
  GPIO_MODER_MODER7_Pos* = (14)
  GPIO_MODER_MODER7_Msk* = (0x00000003 shl GPIO_MODER_MODER7_Pos) ## !< 0x0000C000
  GPIO_MODER_MODER7* = GPIO_MODER_MODER7_Msk
  GPIO_MODER_MODER7_0* = (0x00000001 shl GPIO_MODER_MODER7_Pos) ## !< 0x00004000
  GPIO_MODER_MODER7_1* = (0x00000002 shl GPIO_MODER_MODER7_Pos) ## !< 0x00008000
  GPIO_MODER_MODER8_Pos* = (16)
  GPIO_MODER_MODER8_Msk* = (0x00000003 shl GPIO_MODER_MODER8_Pos) ## !< 0x00030000
  GPIO_MODER_MODER8* = GPIO_MODER_MODER8_Msk
  GPIO_MODER_MODER8_0* = (0x00000001 shl GPIO_MODER_MODER8_Pos) ## !< 0x00010000
  GPIO_MODER_MODER8_1* = (0x00000002 shl GPIO_MODER_MODER8_Pos) ## !< 0x00020000
  GPIO_MODER_MODER9_Pos* = (18)
  GPIO_MODER_MODER9_Msk* = (0x00000003 shl GPIO_MODER_MODER9_Pos) ## !< 0x000C0000
  GPIO_MODER_MODER9* = GPIO_MODER_MODER9_Msk
  GPIO_MODER_MODER9_0* = (0x00000001 shl GPIO_MODER_MODER9_Pos) ## !< 0x00040000
  GPIO_MODER_MODER9_1* = (0x00000002 shl GPIO_MODER_MODER9_Pos) ## !< 0x00080000
  GPIO_MODER_MODER10_Pos* = (20)
  GPIO_MODER_MODER10_Msk* = (0x00000003 shl GPIO_MODER_MODER10_Pos) ## !< 0x00300000
  GPIO_MODER_MODER10* = GPIO_MODER_MODER10_Msk
  GPIO_MODER_MODER10_0* = (0x00000001 shl GPIO_MODER_MODER10_Pos) ## !< 0x00100000
  GPIO_MODER_MODER10_1* = (0x00000002 shl GPIO_MODER_MODER10_Pos) ## !< 0x00200000
  GPIO_MODER_MODER11_Pos* = (22)
  GPIO_MODER_MODER11_Msk* = (0x00000003 shl GPIO_MODER_MODER11_Pos) ## !< 0x00C00000
  GPIO_MODER_MODER11* = GPIO_MODER_MODER11_Msk
  GPIO_MODER_MODER11_0* = (0x00000001 shl GPIO_MODER_MODER11_Pos) ## !< 0x00400000
  GPIO_MODER_MODER11_1* = (0x00000002 shl GPIO_MODER_MODER11_Pos) ## !< 0x00800000
  GPIO_MODER_MODER12_Pos* = (24)
  GPIO_MODER_MODER12_Msk* = (0x00000003 shl GPIO_MODER_MODER12_Pos) ## !< 0x03000000
  GPIO_MODER_MODER12* = GPIO_MODER_MODER12_Msk
  GPIO_MODER_MODER12_0* = (0x00000001 shl GPIO_MODER_MODER12_Pos) ## !< 0x01000000
  GPIO_MODER_MODER12_1* = (0x00000002 shl GPIO_MODER_MODER12_Pos) ## !< 0x02000000
  GPIO_MODER_MODER13_Pos* = (26)
  GPIO_MODER_MODER13_Msk* = (0x00000003 shl GPIO_MODER_MODER13_Pos) ## !< 0x0C000000
  GPIO_MODER_MODER13* = GPIO_MODER_MODER13_Msk
  GPIO_MODER_MODER13_0* = (0x00000001 shl GPIO_MODER_MODER13_Pos) ## !< 0x04000000
  GPIO_MODER_MODER13_1* = (0x00000002 shl GPIO_MODER_MODER13_Pos) ## !< 0x08000000
  GPIO_MODER_MODER14_Pos* = (28)
  GPIO_MODER_MODER14_Msk* = (0x00000003 shl GPIO_MODER_MODER14_Pos) ## !< 0x30000000
  GPIO_MODER_MODER14* = GPIO_MODER_MODER14_Msk
  GPIO_MODER_MODER14_0* = (0x00000001 shl GPIO_MODER_MODER14_Pos) ## !< 0x10000000
  GPIO_MODER_MODER14_1* = (0x00000002 shl GPIO_MODER_MODER14_Pos) ## !< 0x20000000
  GPIO_MODER_MODER15_Pos* = (30)
  GPIO_MODER_MODER15_Msk* = (0x00000003 shl GPIO_MODER_MODER15_Pos) ## !< 0xC0000000
  GPIO_MODER_MODER15* = GPIO_MODER_MODER15_Msk
  GPIO_MODER_MODER15_0* = (0x00000001 shl GPIO_MODER_MODER15_Pos) ## !< 0x40000000
  GPIO_MODER_MODER15_1* = (0x00000002 shl GPIO_MODER_MODER15_Pos) ## !< 0x80000000

## *****************  Bits definition for GPIO_OTYPER register  ***************

const
  GPIO_OTYPER_OT_0* = (0x00000001)
  GPIO_OTYPER_OT_1* = (0x00000002)
  GPIO_OTYPER_OT_2* = (0x00000004)
  GPIO_OTYPER_OT_3* = (0x00000008)
  GPIO_OTYPER_OT_4* = (0x00000010)
  GPIO_OTYPER_OT_5* = (0x00000020)
  GPIO_OTYPER_OT_6* = (0x00000040)
  GPIO_OTYPER_OT_7* = (0x00000080)
  GPIO_OTYPER_OT_8* = (0x00000100)
  GPIO_OTYPER_OT_9* = (0x00000200)
  GPIO_OTYPER_OT_10* = (0x00000400)
  GPIO_OTYPER_OT_11* = (0x00000800)
  GPIO_OTYPER_OT_12* = (0x00001000)
  GPIO_OTYPER_OT_13* = (0x00002000)
  GPIO_OTYPER_OT_14* = (0x00004000)
  GPIO_OTYPER_OT_15* = (0x00008000)

## *****************  Bits definition for GPIO_OSPEEDR register  **************

const
  GPIO_OSPEEDER_OSPEEDR0_Pos* = (0)
  GPIO_OSPEEDER_OSPEEDR0_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR0_Pos) ## !< 0x00000003
  GPIO_OSPEEDER_OSPEEDR0* = GPIO_OSPEEDER_OSPEEDR0_Msk
  GPIO_OSPEEDER_OSPEEDR0_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR0_Pos) ## !< 0x00000001
  GPIO_OSPEEDER_OSPEEDR0_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR0_Pos) ## !< 0x00000002
  GPIO_OSPEEDER_OSPEEDR1_Pos* = (2)
  GPIO_OSPEEDER_OSPEEDR1_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR1_Pos) ## !< 0x0000000C
  GPIO_OSPEEDER_OSPEEDR1* = GPIO_OSPEEDER_OSPEEDR1_Msk
  GPIO_OSPEEDER_OSPEEDR1_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR1_Pos) ## !< 0x00000004
  GPIO_OSPEEDER_OSPEEDR1_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR1_Pos) ## !< 0x00000008
  GPIO_OSPEEDER_OSPEEDR2_Pos* = (4)
  GPIO_OSPEEDER_OSPEEDR2_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR2_Pos) ## !< 0x00000030
  GPIO_OSPEEDER_OSPEEDR2* = GPIO_OSPEEDER_OSPEEDR2_Msk
  GPIO_OSPEEDER_OSPEEDR2_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR2_Pos) ## !< 0x00000010
  GPIO_OSPEEDER_OSPEEDR2_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR2_Pos) ## !< 0x00000020
  GPIO_OSPEEDER_OSPEEDR3_Pos* = (6)
  GPIO_OSPEEDER_OSPEEDR3_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR3_Pos) ## !< 0x000000C0
  GPIO_OSPEEDER_OSPEEDR3* = GPIO_OSPEEDER_OSPEEDR3_Msk
  GPIO_OSPEEDER_OSPEEDR3_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR3_Pos) ## !< 0x00000040
  GPIO_OSPEEDER_OSPEEDR3_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR3_Pos) ## !< 0x00000080
  GPIO_OSPEEDER_OSPEEDR4_Pos* = (8)
  GPIO_OSPEEDER_OSPEEDR4_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR4_Pos) ## !< 0x00000300
  GPIO_OSPEEDER_OSPEEDR4* = GPIO_OSPEEDER_OSPEEDR4_Msk
  GPIO_OSPEEDER_OSPEEDR4_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR4_Pos) ## !< 0x00000100
  GPIO_OSPEEDER_OSPEEDR4_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR4_Pos) ## !< 0x00000200
  GPIO_OSPEEDER_OSPEEDR5_Pos* = (10)
  GPIO_OSPEEDER_OSPEEDR5_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR5_Pos) ## !< 0x00000C00
  GPIO_OSPEEDER_OSPEEDR5* = GPIO_OSPEEDER_OSPEEDR5_Msk
  GPIO_OSPEEDER_OSPEEDR5_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR5_Pos) ## !< 0x00000400
  GPIO_OSPEEDER_OSPEEDR5_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR5_Pos) ## !< 0x00000800
  GPIO_OSPEEDER_OSPEEDR6_Pos* = (12)
  GPIO_OSPEEDER_OSPEEDR6_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR6_Pos) ## !< 0x00003000
  GPIO_OSPEEDER_OSPEEDR6* = GPIO_OSPEEDER_OSPEEDR6_Msk
  GPIO_OSPEEDER_OSPEEDR6_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR6_Pos) ## !< 0x00001000
  GPIO_OSPEEDER_OSPEEDR6_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR6_Pos) ## !< 0x00002000
  GPIO_OSPEEDER_OSPEEDR7_Pos* = (14)
  GPIO_OSPEEDER_OSPEEDR7_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR7_Pos) ## !< 0x0000C000
  GPIO_OSPEEDER_OSPEEDR7* = GPIO_OSPEEDER_OSPEEDR7_Msk
  GPIO_OSPEEDER_OSPEEDR7_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR7_Pos) ## !< 0x00004000
  GPIO_OSPEEDER_OSPEEDR7_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR7_Pos) ## !< 0x00008000
  GPIO_OSPEEDER_OSPEEDR8_Pos* = (16)
  GPIO_OSPEEDER_OSPEEDR8_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR8_Pos) ## !< 0x00030000
  GPIO_OSPEEDER_OSPEEDR8* = GPIO_OSPEEDER_OSPEEDR8_Msk
  GPIO_OSPEEDER_OSPEEDR8_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR8_Pos) ## !< 0x00010000
  GPIO_OSPEEDER_OSPEEDR8_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR8_Pos) ## !< 0x00020000
  GPIO_OSPEEDER_OSPEEDR9_Pos* = (18)
  GPIO_OSPEEDER_OSPEEDR9_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR9_Pos) ## !< 0x000C0000
  GPIO_OSPEEDER_OSPEEDR9* = GPIO_OSPEEDER_OSPEEDR9_Msk
  GPIO_OSPEEDER_OSPEEDR9_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR9_Pos) ## !< 0x00040000
  GPIO_OSPEEDER_OSPEEDR9_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR9_Pos) ## !< 0x00080000
  GPIO_OSPEEDER_OSPEEDR10_Pos* = (20)
  GPIO_OSPEEDER_OSPEEDR10_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR10_Pos) ## !< 0x00300000
  GPIO_OSPEEDER_OSPEEDR10x* = GPIO_OSPEEDER_OSPEEDR10_Msk
  GPIO_OSPEEDER_OSPEEDR10_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR10_Pos) ## !< 0x00100000
  GPIO_OSPEEDER_OSPEEDR10_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR10_Pos) ## !< 0x00200000
  GPIO_OSPEEDER_OSPEEDR11_Pos* = (22)
  GPIO_OSPEEDER_OSPEEDR11_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR11_Pos) ## !< 0x00C00000
  GPIO_OSPEEDER_OSPEEDR11x* = GPIO_OSPEEDER_OSPEEDR11_Msk
  GPIO_OSPEEDER_OSPEEDR11_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR11_Pos) ## !< 0x00400000
  GPIO_OSPEEDER_OSPEEDR11_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR11_Pos) ## !< 0x00800000
  GPIO_OSPEEDER_OSPEEDR12_Pos* = (24)
  GPIO_OSPEEDER_OSPEEDR12_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR12_Pos) ## !< 0x03000000
  GPIO_OSPEEDER_OSPEEDR12* = GPIO_OSPEEDER_OSPEEDR12_Msk
  GPIO_OSPEEDER_OSPEEDR12_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR12_Pos) ## !< 0x01000000
  GPIO_OSPEEDER_OSPEEDR12_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR12_Pos) ## !< 0x02000000
  GPIO_OSPEEDER_OSPEEDR13_Pos* = (26)
  GPIO_OSPEEDER_OSPEEDR13_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR13_Pos) ## !< 0x0C000000
  GPIO_OSPEEDER_OSPEEDR13* = GPIO_OSPEEDER_OSPEEDR13_Msk
  GPIO_OSPEEDER_OSPEEDR13_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR13_Pos) ## !< 0x04000000
  GPIO_OSPEEDER_OSPEEDR13_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR13_Pos) ## !< 0x08000000
  GPIO_OSPEEDER_OSPEEDR14_Pos* = (28)
  GPIO_OSPEEDER_OSPEEDR14_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR14_Pos) ## !< 0x30000000
  GPIO_OSPEEDER_OSPEEDR14* = GPIO_OSPEEDER_OSPEEDR14_Msk
  GPIO_OSPEEDER_OSPEEDR14_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR14_Pos) ## !< 0x10000000
  GPIO_OSPEEDER_OSPEEDR14_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR14_Pos) ## !< 0x20000000
  GPIO_OSPEEDER_OSPEEDR15_Pos* = (30)
  GPIO_OSPEEDER_OSPEEDR15_Msk* = (0x00000003 shl GPIO_OSPEEDER_OSPEEDR15_Pos) ## !< 0xC0000000
  GPIO_OSPEEDER_OSPEEDR15* = GPIO_OSPEEDER_OSPEEDR15_Msk
  GPIO_OSPEEDER_OSPEEDR15_0* = (0x00000001 shl GPIO_OSPEEDER_OSPEEDR15_Pos) ## !< 0x40000000
  GPIO_OSPEEDER_OSPEEDR15_1* = (0x00000002 shl GPIO_OSPEEDER_OSPEEDR15_Pos) ## !< 0x80000000

## *****************  Bits definition for GPIO_PUPDR register  ****************

const
  GPIO_PUPDR_PUPDR0_Pos* = (0)
  GPIO_PUPDR_PUPDR0_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR0_Pos) ## !< 0x00000003
  GPIO_PUPDR_PUPDR0* = GPIO_PUPDR_PUPDR0_Msk
  GPIO_PUPDR_PUPDR0_0* = (0x00000001 shl GPIO_PUPDR_PUPDR0_Pos) ## !< 0x00000001
  GPIO_PUPDR_PUPDR0_1* = (0x00000002 shl GPIO_PUPDR_PUPDR0_Pos) ## !< 0x00000002
  GPIO_PUPDR_PUPDR1_Pos* = (2)
  GPIO_PUPDR_PUPDR1_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR1_Pos) ## !< 0x0000000C
  GPIO_PUPDR_PUPDR1* = GPIO_PUPDR_PUPDR1_Msk
  GPIO_PUPDR_PUPDR1_0* = (0x00000001 shl GPIO_PUPDR_PUPDR1_Pos) ## !< 0x00000004
  GPIO_PUPDR_PUPDR1_1* = (0x00000002 shl GPIO_PUPDR_PUPDR1_Pos) ## !< 0x00000008
  GPIO_PUPDR_PUPDR2_Pos* = (4)
  GPIO_PUPDR_PUPDR2_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR2_Pos) ## !< 0x00000030
  GPIO_PUPDR_PUPDR2* = GPIO_PUPDR_PUPDR2_Msk
  GPIO_PUPDR_PUPDR2_0* = (0x00000001 shl GPIO_PUPDR_PUPDR2_Pos) ## !< 0x00000010
  GPIO_PUPDR_PUPDR2_1* = (0x00000002 shl GPIO_PUPDR_PUPDR2_Pos) ## !< 0x00000020
  GPIO_PUPDR_PUPDR3_Pos* = (6)
  GPIO_PUPDR_PUPDR3_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR3_Pos) ## !< 0x000000C0
  GPIO_PUPDR_PUPDR3* = GPIO_PUPDR_PUPDR3_Msk
  GPIO_PUPDR_PUPDR3_0* = (0x00000001 shl GPIO_PUPDR_PUPDR3_Pos) ## !< 0x00000040
  GPIO_PUPDR_PUPDR3_1* = (0x00000002 shl GPIO_PUPDR_PUPDR3_Pos) ## !< 0x00000080
  GPIO_PUPDR_PUPDR4_Pos* = (8)
  GPIO_PUPDR_PUPDR4_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR4_Pos) ## !< 0x00000300
  GPIO_PUPDR_PUPDR4* = GPIO_PUPDR_PUPDR4_Msk
  GPIO_PUPDR_PUPDR4_0* = (0x00000001 shl GPIO_PUPDR_PUPDR4_Pos) ## !< 0x00000100
  GPIO_PUPDR_PUPDR4_1* = (0x00000002 shl GPIO_PUPDR_PUPDR4_Pos) ## !< 0x00000200
  GPIO_PUPDR_PUPDR5_Pos* = (10)
  GPIO_PUPDR_PUPDR5_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR5_Pos) ## !< 0x00000C00
  GPIO_PUPDR_PUPDR5* = GPIO_PUPDR_PUPDR5_Msk
  GPIO_PUPDR_PUPDR5_0* = (0x00000001 shl GPIO_PUPDR_PUPDR5_Pos) ## !< 0x00000400
  GPIO_PUPDR_PUPDR5_1* = (0x00000002 shl GPIO_PUPDR_PUPDR5_Pos) ## !< 0x00000800
  GPIO_PUPDR_PUPDR6_Pos* = (12)
  GPIO_PUPDR_PUPDR6_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR6_Pos) ## !< 0x00003000
  GPIO_PUPDR_PUPDR6* = GPIO_PUPDR_PUPDR6_Msk
  GPIO_PUPDR_PUPDR6_0* = (0x00000001 shl GPIO_PUPDR_PUPDR6_Pos) ## !< 0x00001000
  GPIO_PUPDR_PUPDR6_1* = (0x00000002 shl GPIO_PUPDR_PUPDR6_Pos) ## !< 0x00002000
  GPIO_PUPDR_PUPDR7_Pos* = (14)
  GPIO_PUPDR_PUPDR7_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR7_Pos) ## !< 0x0000C000
  GPIO_PUPDR_PUPDR7* = GPIO_PUPDR_PUPDR7_Msk
  GPIO_PUPDR_PUPDR7_0* = (0x00000001 shl GPIO_PUPDR_PUPDR7_Pos) ## !< 0x00004000
  GPIO_PUPDR_PUPDR7_1* = (0x00000002 shl GPIO_PUPDR_PUPDR7_Pos) ## !< 0x00008000
  GPIO_PUPDR_PUPDR8_Pos* = (16)
  GPIO_PUPDR_PUPDR8_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR8_Pos) ## !< 0x00030000
  GPIO_PUPDR_PUPDR8* = GPIO_PUPDR_PUPDR8_Msk
  GPIO_PUPDR_PUPDR8_0* = (0x00000001 shl GPIO_PUPDR_PUPDR8_Pos) ## !< 0x00010000
  GPIO_PUPDR_PUPDR8_1* = (0x00000002 shl GPIO_PUPDR_PUPDR8_Pos) ## !< 0x00020000
  GPIO_PUPDR_PUPDR9_Pos* = (18)
  GPIO_PUPDR_PUPDR9_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR9_Pos) ## !< 0x000C0000
  GPIO_PUPDR_PUPDR9* = GPIO_PUPDR_PUPDR9_Msk
  GPIO_PUPDR_PUPDR9_0* = (0x00000001 shl GPIO_PUPDR_PUPDR9_Pos) ## !< 0x00040000
  GPIO_PUPDR_PUPDR9_1* = (0x00000002 shl GPIO_PUPDR_PUPDR9_Pos) ## !< 0x00080000
  GPIO_PUPDR_PUPDR10_Pos* = (20)
  GPIO_PUPDR_PUPDR10_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR10_Pos) ## !< 0x00300000
  GPIO_PUPDR_PUPDR10x* = GPIO_PUPDR_PUPDR10_Msk
  GPIO_PUPDR_PUPDR10_0* = (0x00000001 shl GPIO_PUPDR_PUPDR10_Pos) ## !< 0x00100000
  GPIO_PUPDR_PUPDR10_1* = (0x00000002 shl GPIO_PUPDR_PUPDR10_Pos) ## !< 0x00200000
  GPIO_PUPDR_PUPDR11_Pos* = (22)
  GPIO_PUPDR_PUPDR11_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR11_Pos) ## !< 0x00C00000
  GPIO_PUPDR_PUPDR11x* = GPIO_PUPDR_PUPDR11_Msk
  GPIO_PUPDR_PUPDR11_0* = (0x00000001 shl GPIO_PUPDR_PUPDR11_Pos) ## !< 0x00400000
  GPIO_PUPDR_PUPDR11_1* = (0x00000002 shl GPIO_PUPDR_PUPDR11_Pos) ## !< 0x00800000
  GPIO_PUPDR_PUPDR12_Pos* = (24)
  GPIO_PUPDR_PUPDR12_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR12_Pos) ## !< 0x03000000
  GPIO_PUPDR_PUPDR12* = GPIO_PUPDR_PUPDR12_Msk
  GPIO_PUPDR_PUPDR12_0* = (0x00000001 shl GPIO_PUPDR_PUPDR12_Pos) ## !< 0x01000000
  GPIO_PUPDR_PUPDR12_1* = (0x00000002 shl GPIO_PUPDR_PUPDR12_Pos) ## !< 0x02000000
  GPIO_PUPDR_PUPDR13_Pos* = (26)
  GPIO_PUPDR_PUPDR13_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR13_Pos) ## !< 0x0C000000
  GPIO_PUPDR_PUPDR13* = GPIO_PUPDR_PUPDR13_Msk
  GPIO_PUPDR_PUPDR13_0* = (0x00000001 shl GPIO_PUPDR_PUPDR13_Pos) ## !< 0x04000000
  GPIO_PUPDR_PUPDR13_1* = (0x00000002 shl GPIO_PUPDR_PUPDR13_Pos) ## !< 0x08000000
  GPIO_PUPDR_PUPDR14_Pos* = (28)
  GPIO_PUPDR_PUPDR14_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR14_Pos) ## !< 0x30000000
  GPIO_PUPDR_PUPDR14* = GPIO_PUPDR_PUPDR14_Msk
  GPIO_PUPDR_PUPDR14_0* = (0x00000001 shl GPIO_PUPDR_PUPDR14_Pos) ## !< 0x10000000
  GPIO_PUPDR_PUPDR14_1* = (0x00000002 shl GPIO_PUPDR_PUPDR14_Pos) ## !< 0x20000000
  GPIO_PUPDR_PUPDR15_Pos* = (30)
  GPIO_PUPDR_PUPDR15_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR15_Pos) ## !< 0xC0000000
  GPIO_PUPDR_PUPDR15* = GPIO_PUPDR_PUPDR15_Msk
  GPIO_PUPDR_PUPDR15_0* = (0x00000001 shl GPIO_PUPDR_PUPDR15_Pos) ## !< 0x40000000
  GPIO_PUPDR_PUPDR15_1* = (0x00000002 shl GPIO_PUPDR_PUPDR15_Pos) ## !< 0x80000000

## *****************  Bits definition for GPIO_IDR register  ******************

const
  GPIO_IDR_IDR_0* = (0x00000001)
  GPIO_IDR_IDR_1* = (0x00000002)
  GPIO_IDR_IDR_2* = (0x00000004)
  GPIO_IDR_IDR_3* = (0x00000008)
  GPIO_IDR_IDR_4* = (0x00000010)
  GPIO_IDR_IDR_5* = (0x00000020)
  GPIO_IDR_IDR_6* = (0x00000040)
  GPIO_IDR_IDR_7* = (0x00000080)
  GPIO_IDR_IDR_8* = (0x00000100)
  GPIO_IDR_IDR_9* = (0x00000200)
  GPIO_IDR_IDR_10* = (0x00000400)
  GPIO_IDR_IDR_11* = (0x00000800)
  GPIO_IDR_IDR_12* = (0x00001000)
  GPIO_IDR_IDR_13* = (0x00002000)
  GPIO_IDR_IDR_14* = (0x00004000)
  GPIO_IDR_IDR_15* = (0x00008000)

## *****************  Bits definition for GPIO_ODR register  ******************

const
  GPIO_ODR_ODR_0* = (0x00000001)
  GPIO_ODR_ODR_1* = (0x00000002)
  GPIO_ODR_ODR_2* = (0x00000004)
  GPIO_ODR_ODR_3* = (0x00000008)
  GPIO_ODR_ODR_4* = (0x00000010)
  GPIO_ODR_ODR_5* = (0x00000020)
  GPIO_ODR_ODR_6* = (0x00000040)
  GPIO_ODR_ODR_7* = (0x00000080)
  GPIO_ODR_ODR_8* = (0x00000100)
  GPIO_ODR_ODR_9* = (0x00000200)
  GPIO_ODR_ODR_10* = (0x00000400)
  GPIO_ODR_ODR_11* = (0x00000800)
  GPIO_ODR_ODR_12* = (0x00001000)
  GPIO_ODR_ODR_13* = (0x00002000)
  GPIO_ODR_ODR_14* = (0x00004000)
  GPIO_ODR_ODR_15* = (0x00008000)

## *****************  Bits definition for GPIO_BSRR register  *****************

const
  GPIO_BSRR_BS_0* = (0x00000001)
  GPIO_BSRR_BS_1* = (0x00000002)
  GPIO_BSRR_BS_2* = (0x00000004)
  GPIO_BSRR_BS_3* = (0x00000008)
  GPIO_BSRR_BS_4* = (0x00000010)
  GPIO_BSRR_BS_5* = (0x00000020)
  GPIO_BSRR_BS_6* = (0x00000040)
  GPIO_BSRR_BS_7* = (0x00000080)
  GPIO_BSRR_BS_8* = (0x00000100)
  GPIO_BSRR_BS_9* = (0x00000200)
  GPIO_BSRR_BS_10* = (0x00000400)
  GPIO_BSRR_BS_11* = (0x00000800)
  GPIO_BSRR_BS_12* = (0x00001000)
  GPIO_BSRR_BS_13* = (0x00002000)
  GPIO_BSRR_BS_14* = (0x00004000)
  GPIO_BSRR_BS_15* = (0x00008000)
  GPIO_BSRR_BR_0* = (0x00010000)
  GPIO_BSRR_BR_1* = (0x00020000)
  GPIO_BSRR_BR_2* = (0x00040000)
  GPIO_BSRR_BR_3* = (0x00080000)
  GPIO_BSRR_BR_4* = (0x00100000)
  GPIO_BSRR_BR_5* = (0x00200000)
  GPIO_BSRR_BR_6* = (0x00400000)
  GPIO_BSRR_BR_7* = (0x00800000)
  GPIO_BSRR_BR_8* = (0x01000000)
  GPIO_BSRR_BR_9* = (0x02000000)
  GPIO_BSRR_BR_10* = (0x04000000)
  GPIO_BSRR_BR_11* = (0x08000000)
  GPIO_BSRR_BR_12* = (0x10000000)
  GPIO_BSRR_BR_13* = (0x20000000)
  GPIO_BSRR_BR_14* = (0x40000000)
  GPIO_BSRR_BR_15* = (0x80000000)

## ***************** Bit definition for GPIO_LCKR register  *******************

const
  GPIO_LCKR_LCK0_Pos* = (0)
  GPIO_LCKR_LCK0_Msk* = (0x00000001 shl GPIO_LCKR_LCK0_Pos) ## !< 0x00000001
  GPIO_LCKR_LCK0* = GPIO_LCKR_LCK0_Msk
  GPIO_LCKR_LCK1_Pos* = (1)
  GPIO_LCKR_LCK1_Msk* = (0x00000001 shl GPIO_LCKR_LCK1_Pos) ## !< 0x00000002
  GPIO_LCKR_LCK1* = GPIO_LCKR_LCK1_Msk
  GPIO_LCKR_LCK2_Pos* = (2)
  GPIO_LCKR_LCK2_Msk* = (0x00000001 shl GPIO_LCKR_LCK2_Pos) ## !< 0x00000004
  GPIO_LCKR_LCK2* = GPIO_LCKR_LCK2_Msk
  GPIO_LCKR_LCK3_Pos* = (3)
  GPIO_LCKR_LCK3_Msk* = (0x00000001 shl GPIO_LCKR_LCK3_Pos) ## !< 0x00000008
  GPIO_LCKR_LCK3* = GPIO_LCKR_LCK3_Msk
  GPIO_LCKR_LCK4_Pos* = (4)
  GPIO_LCKR_LCK4_Msk* = (0x00000001 shl GPIO_LCKR_LCK4_Pos) ## !< 0x00000010
  GPIO_LCKR_LCK4* = GPIO_LCKR_LCK4_Msk
  GPIO_LCKR_LCK5_Pos* = (5)
  GPIO_LCKR_LCK5_Msk* = (0x00000001 shl GPIO_LCKR_LCK5_Pos) ## !< 0x00000020
  GPIO_LCKR_LCK5* = GPIO_LCKR_LCK5_Msk
  GPIO_LCKR_LCK6_Pos* = (6)
  GPIO_LCKR_LCK6_Msk* = (0x00000001 shl GPIO_LCKR_LCK6_Pos) ## !< 0x00000040
  GPIO_LCKR_LCK6* = GPIO_LCKR_LCK6_Msk
  GPIO_LCKR_LCK7_Pos* = (7)
  GPIO_LCKR_LCK7_Msk* = (0x00000001 shl GPIO_LCKR_LCK7_Pos) ## !< 0x00000080
  GPIO_LCKR_LCK7* = GPIO_LCKR_LCK7_Msk
  GPIO_LCKR_LCK8_Pos* = (8)
  GPIO_LCKR_LCK8_Msk* = (0x00000001 shl GPIO_LCKR_LCK8_Pos) ## !< 0x00000100
  GPIO_LCKR_LCK8* = GPIO_LCKR_LCK8_Msk
  GPIO_LCKR_LCK9_Pos* = (9)
  GPIO_LCKR_LCK9_Msk* = (0x00000001 shl GPIO_LCKR_LCK9_Pos) ## !< 0x00000200
  GPIO_LCKR_LCK9* = GPIO_LCKR_LCK9_Msk
  GPIO_LCKR_LCK10_Pos* = (10)
  GPIO_LCKR_LCK10_Msk* = (0x00000001 shl GPIO_LCKR_LCK10_Pos) ## !< 0x00000400
  GPIO_LCKR_LCK10* = GPIO_LCKR_LCK10_Msk
  GPIO_LCKR_LCK11_Pos* = (11)
  GPIO_LCKR_LCK11_Msk* = (0x00000001 shl GPIO_LCKR_LCK11_Pos) ## !< 0x00000800
  GPIO_LCKR_LCK11* = GPIO_LCKR_LCK11_Msk
  GPIO_LCKR_LCK12_Pos* = (12)
  GPIO_LCKR_LCK12_Msk* = (0x00000001 shl GPIO_LCKR_LCK12_Pos) ## !< 0x00001000
  GPIO_LCKR_LCK12* = GPIO_LCKR_LCK12_Msk
  GPIO_LCKR_LCK13_Pos* = (13)
  GPIO_LCKR_LCK13_Msk* = (0x00000001 shl GPIO_LCKR_LCK13_Pos) ## !< 0x00002000
  GPIO_LCKR_LCK13* = GPIO_LCKR_LCK13_Msk
  GPIO_LCKR_LCK14_Pos* = (14)
  GPIO_LCKR_LCK14_Msk* = (0x00000001 shl GPIO_LCKR_LCK14_Pos) ## !< 0x00004000
  GPIO_LCKR_LCK14* = GPIO_LCKR_LCK14_Msk
  GPIO_LCKR_LCK15_Pos* = (15)
  GPIO_LCKR_LCK15_Msk* = (0x00000001 shl GPIO_LCKR_LCK15_Pos) ## !< 0x00008000
  GPIO_LCKR_LCK15* = GPIO_LCKR_LCK15_Msk
  GPIO_LCKR_LCKK_Pos* = (16)
  GPIO_LCKR_LCKK_Msk* = (0x00000001 shl GPIO_LCKR_LCKK_Pos) ## !< 0x00010000
  GPIO_LCKR_LCKK* = GPIO_LCKR_LCKK_Msk

## ***************** Bit definition for GPIO_AFRL register  *******************

const
  GPIO_AFRL_AFSEL0_Pos* = (0)
  GPIO_AFRL_AFSEL0_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL0_Pos) ## !< 0x0000000F
  GPIO_AFRL_AFSEL0* = GPIO_AFRL_AFSEL0_Msk
  GPIO_AFRL_AFSEL1_Pos* = (4)
  GPIO_AFRL_AFSEL1_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL1_Pos) ## !< 0x000000F0
  GPIO_AFRL_AFSEL1* = GPIO_AFRL_AFSEL1_Msk
  GPIO_AFRL_AFSEL2_Pos* = (8)
  GPIO_AFRL_AFSEL2_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL2_Pos) ## !< 0x00000F00
  GPIO_AFRL_AFSEL2* = GPIO_AFRL_AFSEL2_Msk
  GPIO_AFRL_AFSEL3_Pos* = (12)
  GPIO_AFRL_AFSEL3_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL3_Pos) ## !< 0x0000F000
  GPIO_AFRL_AFSEL3* = GPIO_AFRL_AFSEL3_Msk
  GPIO_AFRL_AFSEL4_Pos* = (16)
  GPIO_AFRL_AFSEL4_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL4_Pos) ## !< 0x000F0000
  GPIO_AFRL_AFSEL4* = GPIO_AFRL_AFSEL4_Msk
  GPIO_AFRL_AFSEL5_Pos* = (20)
  GPIO_AFRL_AFSEL5_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL5_Pos) ## !< 0x00F00000
  GPIO_AFRL_AFSEL5* = GPIO_AFRL_AFSEL5_Msk
  GPIO_AFRL_AFSEL6_Pos* = (24)
  GPIO_AFRL_AFSEL6_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL6_Pos) ## !< 0x0F000000
  GPIO_AFRL_AFSEL6* = GPIO_AFRL_AFSEL6_Msk
  GPIO_AFRL_AFSEL7_Pos* = (28)
  GPIO_AFRL_AFSEL7_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL7_Pos) ## !< 0xF0000000
  GPIO_AFRL_AFSEL7* = GPIO_AFRL_AFSEL7_Msk

## ***************** Bit definition for GPIO_AFRH register  *******************

const
  GPIO_AFRH_AFSEL8_Pos* = (0)
  GPIO_AFRH_AFSEL8_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL8_Pos) ## !< 0x0000000F
  GPIO_AFRH_AFSEL8* = GPIO_AFRH_AFSEL8_Msk
  GPIO_AFRH_AFSEL9_Pos* = (4)
  GPIO_AFRH_AFSEL9_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL9_Pos) ## !< 0x000000F0
  GPIO_AFRH_AFSEL9* = GPIO_AFRH_AFSEL9_Msk
  GPIO_AFRH_AFSEL10_Pos* = (8)
  GPIO_AFRH_AFSEL10_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL10_Pos) ## !< 0x00000F00
  GPIO_AFRH_AFSEL10* = GPIO_AFRH_AFSEL10_Msk
  GPIO_AFRH_AFSEL11_Pos* = (12)
  GPIO_AFRH_AFSEL11_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL11_Pos) ## !< 0x0000F000
  GPIO_AFRH_AFSEL11* = GPIO_AFRH_AFSEL11_Msk
  GPIO_AFRH_AFSEL12_Pos* = (16)
  GPIO_AFRH_AFSEL12_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL12_Pos) ## !< 0x000F0000
  GPIO_AFRH_AFSEL12* = GPIO_AFRH_AFSEL12_Msk
  GPIO_AFRH_AFSEL13_Pos* = (20)
  GPIO_AFRH_AFSEL13_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL13_Pos) ## !< 0x00F00000
  GPIO_AFRH_AFSEL13* = GPIO_AFRH_AFSEL13_Msk
  GPIO_AFRH_AFSEL14_Pos* = (24)
  GPIO_AFRH_AFSEL14_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL14_Pos) ## !< 0x0F000000
  GPIO_AFRH_AFSEL14* = GPIO_AFRH_AFSEL14_Msk
  GPIO_AFRH_AFSEL15_Pos* = (28)
  GPIO_AFRH_AFSEL15_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL15_Pos) ## !< 0xF0000000
  GPIO_AFRH_AFSEL15* = GPIO_AFRH_AFSEL15_Msk

## ***************** Bit definition for GPIO_BRR register  ********************

const
  GPIO_BRR_BR_0* = (0x00000001)
  GPIO_BRR_BR_1* = (0x00000002)
  GPIO_BRR_BR_2* = (0x00000004)
  GPIO_BRR_BR_3* = (0x00000008)
  GPIO_BRR_BR_4* = (0x00000010)
  GPIO_BRR_BR_5* = (0x00000020)
  GPIO_BRR_BR_6* = (0x00000040)
  GPIO_BRR_BR_7* = (0x00000080)
  GPIO_BRR_BR_8* = (0x00000100)
  GPIO_BRR_BR_9* = (0x00000200)
  GPIO_BRR_BR_10* = (0x00000400)
  GPIO_BRR_BR_11* = (0x00000800)
  GPIO_BRR_BR_12* = (0x00001000)
  GPIO_BRR_BR_13* = (0x00002000)
  GPIO_BRR_BR_14* = (0x00004000)
  GPIO_BRR_BR_15* = (0x00008000)

## ****************************************************************************
##
##                    Inter-integrated Circuit Interface (I2C)
##
## ****************************************************************************
## ******************  Bit definition for I2C_CR1 register  *******************

const
  I2C_CR1_PE_Pos* = (0)
  I2C_CR1_PE_Msk* = (0x00000001 shl I2C_CR1_PE_Pos) ## !< 0x00000001
  I2C_CR1_PE* = I2C_CR1_PE_Msk
  I2C_CR1_SMBUS_Pos* = (1)
  I2C_CR1_SMBUS_Msk* = (0x00000001 shl I2C_CR1_SMBUS_Pos) ## !< 0x00000002
  I2C_CR1_SMBUS* = I2C_CR1_SMBUS_Msk
  I2C_CR1_SMBTYPE_Pos* = (3)
  I2C_CR1_SMBTYPE_Msk* = (0x00000001 shl I2C_CR1_SMBTYPE_Pos) ## !< 0x00000008
  I2C_CR1_SMBTYPE* = I2C_CR1_SMBTYPE_Msk
  I2C_CR1_ENARP_Pos* = (4)
  I2C_CR1_ENARP_Msk* = (0x00000001 shl I2C_CR1_ENARP_Pos) ## !< 0x00000010
  I2C_CR1_ENARP* = I2C_CR1_ENARP_Msk
  I2C_CR1_ENPEC_Pos* = (5)
  I2C_CR1_ENPEC_Msk* = (0x00000001 shl I2C_CR1_ENPEC_Pos) ## !< 0x00000020
  I2C_CR1_ENPEC* = I2C_CR1_ENPEC_Msk
  I2C_CR1_ENGC_Pos* = (6)
  I2C_CR1_ENGC_Msk* = (0x00000001 shl I2C_CR1_ENGC_Pos) ## !< 0x00000040
  I2C_CR1_ENGC* = I2C_CR1_ENGC_Msk
  I2C_CR1_NOSTRETCH_Pos* = (7)
  I2C_CR1_NOSTRETCH_Msk* = (0x00000001 shl I2C_CR1_NOSTRETCH_Pos) ## !< 0x00000080
  I2C_CR1_NOSTRETCH* = I2C_CR1_NOSTRETCH_Msk
  I2C_CR1_START_Pos* = (8)
  I2C_CR1_START_Msk* = (0x00000001 shl I2C_CR1_START_Pos) ## !< 0x00000100
  I2C_CR1_START* = I2C_CR1_START_Msk
  I2C_CR1_STOP_Pos* = (9)
  I2C_CR1_STOP_Msk* = (0x00000001 shl I2C_CR1_STOP_Pos) ## !< 0x00000200
  I2C_CR1_STOP* = I2C_CR1_STOP_Msk
  I2C_CR1_ACK_Pos* = (10)
  I2C_CR1_ACK_Msk* = (0x00000001 shl I2C_CR1_ACK_Pos) ## !< 0x00000400
  I2C_CR1_ACK* = I2C_CR1_ACK_Msk
  I2C_CR1_POS_Pos* = (11)
  I2C_CR1_POS_Msk* = (0x00000001 shl I2C_CR1_POS_Pos) ## !< 0x00000800
  I2C_CR1_POS* = I2C_CR1_POS_Msk
  I2C_CR1_PEC_Pos* = (12)
  I2C_CR1_PEC_Msk* = (0x00000001 shl I2C_CR1_PEC_Pos) ## !< 0x00001000
  I2C_CR1_PEC* = I2C_CR1_PEC_Msk
  I2C_CR1_ALERT_Pos* = (13)
  I2C_CR1_ALERT_Msk* = (0x00000001 shl I2C_CR1_ALERT_Pos) ## !< 0x00002000
  I2C_CR1_ALERT* = I2C_CR1_ALERT_Msk
  I2C_CR1_SWRST_Pos* = (15)
  I2C_CR1_SWRST_Msk* = (0x00000001 shl I2C_CR1_SWRST_Pos) ## !< 0x00008000
  I2C_CR1_SWRST* = I2C_CR1_SWRST_Msk

## ******************  Bit definition for I2C_CR2 register  *******************

const
  I2C_CR2_FREQ_Pos* = (0)
  I2C_CR2_FREQ_Msk* = (0x0000003F shl I2C_CR2_FREQ_Pos) ## !< 0x0000003F
  I2C_CR2_FREQ* = I2C_CR2_FREQ_Msk
  I2C_CR2_FREQ_0* = (0x00000001 shl I2C_CR2_FREQ_Pos) ## !< 0x00000001
  I2C_CR2_FREQ_1* = (0x00000002 shl I2C_CR2_FREQ_Pos) ## !< 0x00000002
  I2C_CR2_FREQ_2* = (0x00000004 shl I2C_CR2_FREQ_Pos) ## !< 0x00000004
  I2C_CR2_FREQ_3* = (0x00000008 shl I2C_CR2_FREQ_Pos) ## !< 0x00000008
  I2C_CR2_FREQ_4* = (0x00000010 shl I2C_CR2_FREQ_Pos) ## !< 0x00000010
  I2C_CR2_FREQ_5* = (0x00000020 shl I2C_CR2_FREQ_Pos) ## !< 0x00000020
  I2C_CR2_ITERREN_Pos* = (8)
  I2C_CR2_ITERREN_Msk* = (0x00000001 shl I2C_CR2_ITERREN_Pos) ## !< 0x00000100
  I2C_CR2_ITERREN* = I2C_CR2_ITERREN_Msk
  I2C_CR2_ITEVTEN_Pos* = (9)
  I2C_CR2_ITEVTEN_Msk* = (0x00000001 shl I2C_CR2_ITEVTEN_Pos) ## !< 0x00000200
  I2C_CR2_ITEVTEN* = I2C_CR2_ITEVTEN_Msk
  I2C_CR2_ITBUFEN_Pos* = (10)
  I2C_CR2_ITBUFEN_Msk* = (0x00000001 shl I2C_CR2_ITBUFEN_Pos) ## !< 0x00000400
  I2C_CR2_ITBUFEN* = I2C_CR2_ITBUFEN_Msk
  I2C_CR2_DMAEN_Pos* = (11)
  I2C_CR2_DMAEN_Msk* = (0x00000001 shl I2C_CR2_DMAEN_Pos) ## !< 0x00000800
  I2C_CR2_DMAEN* = I2C_CR2_DMAEN_Msk
  I2C_CR2_LAST_Pos* = (12)
  I2C_CR2_LAST_Msk* = (0x00000001 shl I2C_CR2_LAST_Pos) ## !< 0x00001000
  I2C_CR2_LAST* = I2C_CR2_LAST_Msk

## ******************  Bit definition for I2C_OAR1 register  ******************

const
  I2C_OAR1_ADD1_7* = (0x000000FE) ## !< Interface Address
  I2C_OAR1_ADD8_9* = (0x00000300) ## !< Interface Address
  I2C_OAR1_ADD0_Pos* = (0)
  I2C_OAR1_ADD0_Msk* = (0x00000001 shl I2C_OAR1_ADD0_Pos) ## !< 0x00000001
  I2C_OAR1_ADD0* = I2C_OAR1_ADD0_Msk
  I2C_OAR1_ADD1_Pos* = (1)
  I2C_OAR1_ADD1_Msk* = (0x00000001 shl I2C_OAR1_ADD1_Pos) ## !< 0x00000002
  I2C_OAR1_ADD1* = I2C_OAR1_ADD1_Msk
  I2C_OAR1_ADD2_Pos* = (2)
  I2C_OAR1_ADD2_Msk* = (0x00000001 shl I2C_OAR1_ADD2_Pos) ## !< 0x00000004
  I2C_OAR1_ADD2* = I2C_OAR1_ADD2_Msk
  I2C_OAR1_ADD3_Pos* = (3)
  I2C_OAR1_ADD3_Msk* = (0x00000001 shl I2C_OAR1_ADD3_Pos) ## !< 0x00000008
  I2C_OAR1_ADD3* = I2C_OAR1_ADD3_Msk
  I2C_OAR1_ADD4_Pos* = (4)
  I2C_OAR1_ADD4_Msk* = (0x00000001 shl I2C_OAR1_ADD4_Pos) ## !< 0x00000010
  I2C_OAR1_ADD4* = I2C_OAR1_ADD4_Msk
  I2C_OAR1_ADD5_Pos* = (5)
  I2C_OAR1_ADD5_Msk* = (0x00000001 shl I2C_OAR1_ADD5_Pos) ## !< 0x00000020
  I2C_OAR1_ADD5* = I2C_OAR1_ADD5_Msk
  I2C_OAR1_ADD6_Pos* = (6)
  I2C_OAR1_ADD6_Msk* = (0x00000001 shl I2C_OAR1_ADD6_Pos) ## !< 0x00000040
  I2C_OAR1_ADD6* = I2C_OAR1_ADD6_Msk
  I2C_OAR1_ADD7_Pos* = (7)
  I2C_OAR1_ADD7_Msk* = (0x00000001 shl I2C_OAR1_ADD7_Pos) ## !< 0x00000080
  I2C_OAR1_ADD7* = I2C_OAR1_ADD7_Msk
  I2C_OAR1_ADD8_Pos* = (8)
  I2C_OAR1_ADD8_Msk* = (0x00000001 shl I2C_OAR1_ADD8_Pos) ## !< 0x00000100
  I2C_OAR1_ADD8* = I2C_OAR1_ADD8_Msk
  I2C_OAR1_ADD9_Pos* = (9)
  I2C_OAR1_ADD9_Msk* = (0x00000001 shl I2C_OAR1_ADD9_Pos) ## !< 0x00000200
  I2C_OAR1_ADD9* = I2C_OAR1_ADD9_Msk
  I2C_OAR1_ADDMODE_Pos* = (15)
  I2C_OAR1_ADDMODE_Msk* = (0x00000001 shl I2C_OAR1_ADDMODE_Pos) ## !< 0x00008000
  I2C_OAR1_ADDMODE* = I2C_OAR1_ADDMODE_Msk

## ******************  Bit definition for I2C_OAR2 register  ******************

const
  I2C_OAR2_ENDUAL_Pos* = (0)
  I2C_OAR2_ENDUAL_Msk* = (0x00000001 shl I2C_OAR2_ENDUAL_Pos) ## !< 0x00000001
  I2C_OAR2_ENDUAL* = I2C_OAR2_ENDUAL_Msk
  I2C_OAR2_ADD2_Pos* = (1)
  I2C_OAR2_ADD2_Msk* = (0x0000007F shl I2C_OAR2_ADD2_Pos) ## !< 0x000000FE
  I2C_OAR2_ADD2* = I2C_OAR2_ADD2_Msk

## *******************  Bit definition for I2C_DR register  *******************

const
  I2C_DR_DR_Pos* = (0)
  I2C_DR_DR_Msk* = (0x000000FF shl I2C_DR_DR_Pos) ## !< 0x000000FF
  I2C_DR_DR* = I2C_DR_DR_Msk

## ******************  Bit definition for I2C_SR1 register  *******************

const
  I2C_SR1_SB_Pos* = (0)
  I2C_SR1_SB_Msk* = (0x00000001 shl I2C_SR1_SB_Pos) ## !< 0x00000001
  I2C_SR1_SB* = I2C_SR1_SB_Msk
  I2C_SR1_ADDR_Pos* = (1)
  I2C_SR1_ADDR_Msk* = (0x00000001 shl I2C_SR1_ADDR_Pos) ## !< 0x00000002
  I2C_SR1_ADDR* = I2C_SR1_ADDR_Msk
  I2C_SR1_BTF_Pos* = (2)
  I2C_SR1_BTF_Msk* = (0x00000001 shl I2C_SR1_BTF_Pos) ## !< 0x00000004
  I2C_SR1_BTF* = I2C_SR1_BTF_Msk
  I2C_SR1_ADD10_Pos* = (3)
  I2C_SR1_ADD10_Msk* = (0x00000001 shl I2C_SR1_ADD10_Pos) ## !< 0x00000008
  I2C_SR1_ADD10* = I2C_SR1_ADD10_Msk
  I2C_SR1_STOPF_Pos* = (4)
  I2C_SR1_STOPF_Msk* = (0x00000001 shl I2C_SR1_STOPF_Pos) ## !< 0x00000010
  I2C_SR1_STOPF* = I2C_SR1_STOPF_Msk
  I2C_SR1_RXNE_Pos* = (6)
  I2C_SR1_RXNE_Msk* = (0x00000001 shl I2C_SR1_RXNE_Pos) ## !< 0x00000040
  I2C_SR1_RXNE* = I2C_SR1_RXNE_Msk
  I2C_SR1_TXE_Pos* = (7)
  I2C_SR1_TXE_Msk* = (0x00000001 shl I2C_SR1_TXE_Pos) ## !< 0x00000080
  I2C_SR1_TXE* = I2C_SR1_TXE_Msk
  I2C_SR1_BERR_Pos* = (8)
  I2C_SR1_BERR_Msk* = (0x00000001 shl I2C_SR1_BERR_Pos) ## !< 0x00000100
  I2C_SR1_BERR* = I2C_SR1_BERR_Msk
  I2C_SR1_ARLO_Pos* = (9)
  I2C_SR1_ARLO_Msk* = (0x00000001 shl I2C_SR1_ARLO_Pos) ## !< 0x00000200
  I2C_SR1_ARLO* = I2C_SR1_ARLO_Msk
  I2C_SR1_AF_Pos* = (10)
  I2C_SR1_AF_Msk* = (0x00000001 shl I2C_SR1_AF_Pos) ## !< 0x00000400
  I2C_SR1_AF* = I2C_SR1_AF_Msk
  I2C_SR1_OVR_Pos* = (11)
  I2C_SR1_OVR_Msk* = (0x00000001 shl I2C_SR1_OVR_Pos) ## !< 0x00000800
  I2C_SR1_OVR* = I2C_SR1_OVR_Msk
  I2C_SR1_PECERR_Pos* = (12)
  I2C_SR1_PECERR_Msk* = (0x00000001 shl I2C_SR1_PECERR_Pos) ## !< 0x00001000
  I2C_SR1_PECERR* = I2C_SR1_PECERR_Msk
  I2C_SR1_TIMEOUT_Pos* = (14)
  I2C_SR1_TIMEOUT_Msk* = (0x00000001 shl I2C_SR1_TIMEOUT_Pos) ## !< 0x00004000
  I2C_SR1_TIMEOUT* = I2C_SR1_TIMEOUT_Msk
  I2C_SR1_SMBALERT_Pos* = (15)
  I2C_SR1_SMBALERT_Msk* = (0x00000001 shl I2C_SR1_SMBALERT_Pos) ## !< 0x00008000
  I2C_SR1_SMBALERT* = I2C_SR1_SMBALERT_Msk

## ******************  Bit definition for I2C_SR2 register  *******************

const
  I2C_SR2_MSL_Pos* = (0)
  I2C_SR2_MSL_Msk* = (0x00000001 shl I2C_SR2_MSL_Pos) ## !< 0x00000001
  I2C_SR2_MSL* = I2C_SR2_MSL_Msk
  I2C_SR2_BUSY_Pos* = (1)
  I2C_SR2_BUSY_Msk* = (0x00000001 shl I2C_SR2_BUSY_Pos) ## !< 0x00000002
  I2C_SR2_BUSY* = I2C_SR2_BUSY_Msk
  I2C_SR2_TRA_Pos* = (2)
  I2C_SR2_TRA_Msk* = (0x00000001 shl I2C_SR2_TRA_Pos) ## !< 0x00000004
  I2C_SR2_TRA* = I2C_SR2_TRA_Msk
  I2C_SR2_GENCALL_Pos* = (4)
  I2C_SR2_GENCALL_Msk* = (0x00000001 shl I2C_SR2_GENCALL_Pos) ## !< 0x00000010
  I2C_SR2_GENCALL* = I2C_SR2_GENCALL_Msk
  I2C_SR2_SMBDEFAULT_Pos* = (5)
  I2C_SR2_SMBDEFAULT_Msk* = (0x00000001 shl I2C_SR2_SMBDEFAULT_Pos) ## !< 0x00000020
  I2C_SR2_SMBDEFAULT* = I2C_SR2_SMBDEFAULT_Msk
  I2C_SR2_SMBHOST_Pos* = (6)
  I2C_SR2_SMBHOST_Msk* = (0x00000001 shl I2C_SR2_SMBHOST_Pos) ## !< 0x00000040
  I2C_SR2_SMBHOST* = I2C_SR2_SMBHOST_Msk
  I2C_SR2_DUALF_Pos* = (7)
  I2C_SR2_DUALF_Msk* = (0x00000001 shl I2C_SR2_DUALF_Pos) ## !< 0x00000080
  I2C_SR2_DUALF* = I2C_SR2_DUALF_Msk
  I2C_SR2_PEC_Pos* = (8)
  I2C_SR2_PEC_Msk* = (0x000000FF shl I2C_SR2_PEC_Pos) ## !< 0x0000FF00
  I2C_SR2_PEC* = I2C_SR2_PEC_Msk

## ******************  Bit definition for I2C_CCR register  *******************

const
  I2C_CCR_CCR_Pos* = (0)
  I2C_CCR_CCR_Msk* = (0x00000FFF shl I2C_CCR_CCR_Pos) ## !< 0x00000FFF
  I2C_CCR_CCR* = I2C_CCR_CCR_Msk
  I2C_CCR_DUTY_Pos* = (14)
  I2C_CCR_DUTY_Msk* = (0x00000001 shl I2C_CCR_DUTY_Pos) ## !< 0x00004000
  I2C_CCR_DUTY* = I2C_CCR_DUTY_Msk
  I2C_CCR_FS_Pos* = (15)
  I2C_CCR_FS_Msk* = (0x00000001 shl I2C_CCR_FS_Pos) ## !< 0x00008000
  I2C_CCR_FS* = I2C_CCR_FS_Msk

## *****************  Bit definition for I2C_TRISE register  ******************

const
  I2C_TRISE_TRISE_Pos* = (0)
  I2C_TRISE_TRISE_Msk* = (0x0000003F shl I2C_TRISE_TRISE_Pos) ## !< 0x0000003F
  I2C_TRISE_TRISE* = I2C_TRISE_TRISE_Msk

## ****************************************************************************
##
##                         Independent WATCHDOG (IWDG)
##
## ****************************************************************************
## ******************  Bit definition for IWDG_KR register  *******************

const
  IWDG_KR_KEY_Pos* = (0)
  IWDG_KR_KEY_Msk* = (0x0000FFFF shl IWDG_KR_KEY_Pos) ## !< 0x0000FFFF
  IWDG_KR_KEY* = IWDG_KR_KEY_Msk

## ******************  Bit definition for IWDG_PR register  *******************

const
  IWDG_PR_PR_Pos* = (0)
  IWDG_PR_PR_Msk* = (0x00000007 shl IWDG_PR_PR_Pos) ## !< 0x00000007
  IWDG_PR_PR* = IWDG_PR_PR_Msk
  IWDG_PR_PR_0* = (0x00000001 shl IWDG_PR_PR_Pos) ## !< 0x00000001
  IWDG_PR_PR_1* = (0x00000002 shl IWDG_PR_PR_Pos) ## !< 0x00000002
  IWDG_PR_PR_2* = (0x00000004 shl IWDG_PR_PR_Pos) ## !< 0x00000004

## ******************  Bit definition for IWDG_RLR register  ******************

const
  IWDG_RLR_RL_Pos* = (0)
  IWDG_RLR_RL_Msk* = (0x00000FFF shl IWDG_RLR_RL_Pos) ## !< 0x00000FFF
  IWDG_RLR_RL* = IWDG_RLR_RL_Msk

## ******************  Bit definition for IWDG_SR register  *******************

const
  IWDG_SR_PVU_Pos* = (0)
  IWDG_SR_PVU_Msk* = (0x00000001 shl IWDG_SR_PVU_Pos) ## !< 0x00000001
  IWDG_SR_PVU* = IWDG_SR_PVU_Msk
  IWDG_SR_RVU_Pos* = (1)
  IWDG_SR_RVU_Msk* = (0x00000001 shl IWDG_SR_RVU_Pos) ## !< 0x00000002
  IWDG_SR_RVU* = IWDG_SR_RVU_Msk

## ****************************************************************************
##
##                           LCD Controller (LCD)
##
## ****************************************************************************
## ******************  Bit definition for LCD_CR register  ********************

const
  LCD_CR_LCDEN_Pos* = (0)
  LCD_CR_LCDEN_Msk* = (0x00000001 shl LCD_CR_LCDEN_Pos) ## !< 0x00000001
  LCD_CR_LCDEN* = LCD_CR_LCDEN_Msk
  LCD_CR_VSEL_Pos* = (1)
  LCD_CR_VSEL_Msk* = (0x00000001 shl LCD_CR_VSEL_Pos) ## !< 0x00000002
  LCD_CR_VSEL* = LCD_CR_VSEL_Msk
  LCD_CR_DUTY_Pos* = (2)
  LCD_CR_DUTY_Msk* = (0x00000007 shl LCD_CR_DUTY_Pos) ## !< 0x0000001C
  LCD_CR_DUTY* = LCD_CR_DUTY_Msk
  LCD_CR_DUTY_0* = (0x00000001 shl LCD_CR_DUTY_Pos) ## !< 0x00000004
  LCD_CR_DUTY_1* = (0x00000002 shl LCD_CR_DUTY_Pos) ## !< 0x00000008
  LCD_CR_DUTY_2* = (0x00000004 shl LCD_CR_DUTY_Pos) ## !< 0x00000010
  LCD_CR_BIAS_Pos* = (5)
  LCD_CR_BIAS_Msk* = (0x00000003 shl LCD_CR_BIAS_Pos) ## !< 0x00000060
  LCD_CR_BIAS* = LCD_CR_BIAS_Msk
  LCD_CR_BIAS_0* = (0x00000001 shl LCD_CR_BIAS_Pos) ## !< 0x00000020
  LCD_CR_BIAS_1* = (0x00000002 shl LCD_CR_BIAS_Pos) ## !< 0x00000040
  LCD_CR_MUX_SEG_Pos* = (7)
  LCD_CR_MUX_SEG_Msk* = (0x00000001 shl LCD_CR_MUX_SEG_Pos) ## !< 0x00000080
  LCD_CR_MUX_SEG* = LCD_CR_MUX_SEG_Msk

## ******************  Bit definition for LCD_FCR register  *******************

const
  LCD_FCR_HD_Pos* = (0)
  LCD_FCR_HD_Msk* = (0x00000001 shl LCD_FCR_HD_Pos) ## !< 0x00000001
  LCD_FCR_HD* = LCD_FCR_HD_Msk
  LCD_FCR_SOFIE_Pos* = (1)
  LCD_FCR_SOFIE_Msk* = (0x00000001 shl LCD_FCR_SOFIE_Pos) ## !< 0x00000002
  LCD_FCR_SOFIE* = LCD_FCR_SOFIE_Msk
  LCD_FCR_UDDIE_Pos* = (3)
  LCD_FCR_UDDIE_Msk* = (0x00000001 shl LCD_FCR_UDDIE_Pos) ## !< 0x00000008
  LCD_FCR_UDDIE* = LCD_FCR_UDDIE_Msk
  LCD_FCR_PON_Pos* = (4)
  LCD_FCR_PON_Msk* = (0x00000007 shl LCD_FCR_PON_Pos) ## !< 0x00000070
  LCD_FCR_PON* = LCD_FCR_PON_Msk
  LCD_FCR_PON_0* = (0x00000001 shl LCD_FCR_PON_Pos) ## !< 0x00000010
  LCD_FCR_PON_1* = (0x00000002 shl LCD_FCR_PON_Pos) ## !< 0x00000020
  LCD_FCR_PON_2* = (0x00000004 shl LCD_FCR_PON_Pos) ## !< 0x00000040
  LCD_FCR_DEAD_Pos* = (7)
  LCD_FCR_DEAD_Msk* = (0x00000007 shl LCD_FCR_DEAD_Pos) ## !< 0x00000380
  LCD_FCR_DEAD* = LCD_FCR_DEAD_Msk
  LCD_FCR_DEAD_0* = (0x00000001 shl LCD_FCR_DEAD_Pos) ## !< 0x00000080
  LCD_FCR_DEAD_1* = (0x00000002 shl LCD_FCR_DEAD_Pos) ## !< 0x00000100
  LCD_FCR_DEAD_2* = (0x00000004 shl LCD_FCR_DEAD_Pos) ## !< 0x00000200
  LCD_FCR_CC_Pos* = (10)
  LCD_FCR_CC_Msk* = (0x00000007 shl LCD_FCR_CC_Pos) ## !< 0x00001C00
  LCD_FCR_CC* = LCD_FCR_CC_Msk
  LCD_FCR_CC_0* = (0x00000001 shl LCD_FCR_CC_Pos) ## !< 0x00000400
  LCD_FCR_CC_1* = (0x00000002 shl LCD_FCR_CC_Pos) ## !< 0x00000800
  LCD_FCR_CC_2* = (0x00000004 shl LCD_FCR_CC_Pos) ## !< 0x00001000
  LCD_FCR_BLINKF_Pos* = (13)
  LCD_FCR_BLINKF_Msk* = (0x00000007 shl LCD_FCR_BLINKF_Pos) ## !< 0x0000E000
  LCD_FCR_BLINKF* = LCD_FCR_BLINKF_Msk
  LCD_FCR_BLINKF_0* = (0x00000001 shl LCD_FCR_BLINKF_Pos) ## !< 0x00002000
  LCD_FCR_BLINKF_1* = (0x00000002 shl LCD_FCR_BLINKF_Pos) ## !< 0x00004000
  LCD_FCR_BLINKF_2* = (0x00000004 shl LCD_FCR_BLINKF_Pos) ## !< 0x00008000
  LCD_FCR_BLINK_Pos* = (16)
  LCD_FCR_BLINK_Msk* = (0x00000003 shl LCD_FCR_BLINK_Pos) ## !< 0x00030000
  LCD_FCR_BLINK* = LCD_FCR_BLINK_Msk
  LCD_FCR_BLINK_0* = (0x00000001 shl LCD_FCR_BLINK_Pos) ## !< 0x00010000
  LCD_FCR_BLINK_1* = (0x00000002 shl LCD_FCR_BLINK_Pos) ## !< 0x00020000
  LCD_FCR_DIV_Pos* = (18)
  LCD_FCR_DIV_Msk* = (0x0000000F shl LCD_FCR_DIV_Pos) ## !< 0x003C0000
  LCD_FCR_DIV* = LCD_FCR_DIV_Msk
  LCD_FCR_PS_Pos* = (22)
  LCD_FCR_PS_Msk* = (0x0000000F shl LCD_FCR_PS_Pos) ## !< 0x03C00000
  LCD_FCR_PS* = LCD_FCR_PS_Msk

## ******************  Bit definition for LCD_SR register  ********************

const
  LCD_SR_ENS_Pos* = (0)
  LCD_SR_ENS_Msk* = (0x00000001 shl LCD_SR_ENS_Pos) ## !< 0x00000001
  LCD_SR_ENS* = LCD_SR_ENS_Msk
  LCD_SR_SOF_Pos* = (1)
  LCD_SR_SOF_Msk* = (0x00000001 shl LCD_SR_SOF_Pos) ## !< 0x00000002
  LCD_SR_SOF* = LCD_SR_SOF_Msk
  LCD_SR_UDR_Pos* = (2)
  LCD_SR_UDR_Msk* = (0x00000001 shl LCD_SR_UDR_Pos) ## !< 0x00000004
  LCD_SR_UDR* = LCD_SR_UDR_Msk
  LCD_SR_UDD_Pos* = (3)
  LCD_SR_UDD_Msk* = (0x00000001 shl LCD_SR_UDD_Pos) ## !< 0x00000008
  LCD_SR_UDD* = LCD_SR_UDD_Msk
  LCD_SR_RDY_Pos* = (4)
  LCD_SR_RDY_Msk* = (0x00000001 shl LCD_SR_RDY_Pos) ## !< 0x00000010
  LCD_SR_RDY* = LCD_SR_RDY_Msk
  LCD_SR_FCRSR_Pos* = (5)
  LCD_SR_FCRSR_Msk* = (0x00000001 shl LCD_SR_FCRSR_Pos) ## !< 0x00000020
  LCD_SR_FCRSR* = LCD_SR_FCRSR_Msk

## ******************  Bit definition for LCD_CLR register  *******************

const
  LCD_CLR_SOFC_Pos* = (1)
  LCD_CLR_SOFC_Msk* = (0x00000001 shl LCD_CLR_SOFC_Pos) ## !< 0x00000002
  LCD_CLR_SOFC* = LCD_CLR_SOFC_Msk
  LCD_CLR_UDDC_Pos* = (3)
  LCD_CLR_UDDC_Msk* = (0x00000001 shl LCD_CLR_UDDC_Pos) ## !< 0x00000008
  LCD_CLR_UDDC* = LCD_CLR_UDDC_Msk

## ******************  Bit definition for LCD_RAM register  *******************

const
  LCD_RAM_SEGMENT_DATA_Pos* = (0)
  LCD_RAM_SEGMENT_DATA_Msk* = (0xFFFFFFFF shl LCD_RAM_SEGMENT_DATA_Pos) ## !< 0xFFFFFFFF
  LCD_RAM_SEGMENT_DATA* = LCD_RAM_SEGMENT_DATA_Msk

## ****************************************************************************
##
##                           Power Control (PWR)
##
## ****************************************************************************

const
  PWR_PVD_SUPPORT* = true       ## !< PWR feature available only on specific devices: Power Voltage Detection feature

## *******************  Bit definition for PWR_CR register  *******************

const
  PWR_CR_LPSDSR_Pos* = (0)
  PWR_CR_LPSDSR_Msk* = (0x00000001 shl PWR_CR_LPSDSR_Pos) ## !< 0x00000001
  PWR_CR_LPSDSR* = PWR_CR_LPSDSR_Msk
  PWR_CR_PDDS_Pos* = (1)
  PWR_CR_PDDS_Msk* = (0x00000001 shl PWR_CR_PDDS_Pos) ## !< 0x00000002
  PWR_CR_PDDS* = PWR_CR_PDDS_Msk
  PWR_CR_CWUF_Pos* = (2)
  PWR_CR_CWUF_Msk* = (0x00000001 shl PWR_CR_CWUF_Pos) ## !< 0x00000004
  PWR_CR_CWUF* = PWR_CR_CWUF_Msk
  PWR_CR_CSBF_Pos* = (3)
  PWR_CR_CSBF_Msk* = (0x00000001 shl PWR_CR_CSBF_Pos) ## !< 0x00000008
  PWR_CR_CSBF* = PWR_CR_CSBF_Msk
  PWR_CR_PVDE_Pos* = (4)
  PWR_CR_PVDE_Msk* = (0x00000001 shl PWR_CR_PVDE_Pos) ## !< 0x00000010
  PWR_CR_PVDE* = PWR_CR_PVDE_Msk
  PWR_CR_PLS_Pos* = (5)
  PWR_CR_PLS_Msk* = (0x00000007 shl PWR_CR_PLS_Pos) ## !< 0x000000E0
  PWR_CR_PLS* = PWR_CR_PLS_Msk
  PWR_CR_PLS_0* = (0x00000001 shl PWR_CR_PLS_Pos) ## !< 0x00000020
  PWR_CR_PLS_1* = (0x00000002 shl PWR_CR_PLS_Pos) ## !< 0x00000040
  PWR_CR_PLS_2* = (0x00000004 shl PWR_CR_PLS_Pos) ## !< 0x00000080

## !< PVD level configuration

const
  PWR_CR_PLS_LEV0* = (0x00000000) ## !< PVD level 0
  PWR_CR_PLS_LEV1* = (0x00000020) ## !< PVD level 1
  PWR_CR_PLS_LEV2* = (0x00000040) ## !< PVD level 2
  PWR_CR_PLS_LEV3* = (0x00000060) ## !< PVD level 3
  PWR_CR_PLS_LEV4* = (0x00000080) ## !< PVD level 4
  PWR_CR_PLS_LEV5* = (0x000000A0) ## !< PVD level 5
  PWR_CR_PLS_LEV6* = (0x000000C0) ## !< PVD level 6
  PWR_CR_PLS_LEV7* = (0x000000E0) ## !< PVD level 7
  PWR_CR_DBP_Pos* = (8)
  PWR_CR_DBP_Msk* = (0x00000001 shl PWR_CR_DBP_Pos) ## !< 0x00000100
  PWR_CR_DBP* = PWR_CR_DBP_Msk
  PWR_CR_ULP_Pos* = (9)
  PWR_CR_ULP_Msk* = (0x00000001 shl PWR_CR_ULP_Pos) ## !< 0x00000200
  PWR_CR_ULP* = PWR_CR_ULP_Msk
  PWR_CR_FWU_Pos* = (10)
  PWR_CR_FWU_Msk* = (0x00000001 shl PWR_CR_FWU_Pos) ## !< 0x00000400
  PWR_CR_FWU* = PWR_CR_FWU_Msk
  PWR_CR_VOS_Pos* = (11)
  PWR_CR_VOS_Msk* = (0x00000003 shl PWR_CR_VOS_Pos) ## !< 0x00001800
  PWR_CR_VOS* = PWR_CR_VOS_Msk
  PWR_CR_VOS_0* = (0x00000001 shl PWR_CR_VOS_Pos) ## !< 0x00000800
  PWR_CR_VOS_1* = (0x00000002 shl PWR_CR_VOS_Pos) ## !< 0x00001000
  PWR_CR_LPRUN_Pos* = (14)
  PWR_CR_LPRUN_Msk* = (0x00000001 shl PWR_CR_LPRUN_Pos) ## !< 0x00004000
  PWR_CR_LPRUN* = PWR_CR_LPRUN_Msk

## ******************  Bit definition for PWR_CSR register  *******************

const
  PWR_CSR_WUF_Pos* = (0)
  PWR_CSR_WUF_Msk* = (0x00000001 shl PWR_CSR_WUF_Pos) ## !< 0x00000001
  PWR_CSR_WUF* = PWR_CSR_WUF_Msk
  PWR_CSR_SBF_Pos* = (1)
  PWR_CSR_SBF_Msk* = (0x00000001 shl PWR_CSR_SBF_Pos) ## !< 0x00000002
  PWR_CSR_SBF* = PWR_CSR_SBF_Msk
  PWR_CSR_PVDO_Pos* = (2)
  PWR_CSR_PVDO_Msk* = (0x00000001 shl PWR_CSR_PVDO_Pos) ## !< 0x00000004
  PWR_CSR_PVDO* = PWR_CSR_PVDO_Msk
  PWR_CSR_VREFINTRDYF_Pos* = (3)
  PWR_CSR_VREFINTRDYF_Msk* = (0x00000001 shl PWR_CSR_VREFINTRDYF_Pos) ## !< 0x00000008
  PWR_CSR_VREFINTRDYF* = PWR_CSR_VREFINTRDYF_Msk
  PWR_CSR_VOSF_Pos* = (4)
  PWR_CSR_VOSF_Msk* = (0x00000001 shl PWR_CSR_VOSF_Pos) ## !< 0x00000010
  PWR_CSR_VOSF* = PWR_CSR_VOSF_Msk
  PWR_CSR_REGLPF_Pos* = (5)
  PWR_CSR_REGLPF_Msk* = (0x00000001 shl PWR_CSR_REGLPF_Pos) ## !< 0x00000020
  PWR_CSR_REGLPF* = PWR_CSR_REGLPF_Msk
  PWR_CSR_EWUP1_Pos* = (8)
  PWR_CSR_EWUP1_Msk* = (0x00000001 shl PWR_CSR_EWUP1_Pos) ## !< 0x00000100
  PWR_CSR_EWUP1* = PWR_CSR_EWUP1_Msk
  PWR_CSR_EWUP2_Pos* = (9)
  PWR_CSR_EWUP2_Msk* = (0x00000001 shl PWR_CSR_EWUP2_Pos) ## !< 0x00000200
  PWR_CSR_EWUP2* = PWR_CSR_EWUP2_Msk
  PWR_CSR_EWUP3_Pos* = (10)
  PWR_CSR_EWUP3_Msk* = (0x00000001 shl PWR_CSR_EWUP3_Pos) ## !< 0x00000400
  PWR_CSR_EWUP3* = PWR_CSR_EWUP3_Msk

## ****************************************************************************
##
##                       Reset and Clock Control (RCC)
##
## ****************************************************************************
##
##  @brief Specific device feature definitions  (not present on all devices in the STM32F0 serie)
##

const
  RCC_LSECSS_SUPPORT* = true    ## !< LSE CSS feature support

## *******************  Bit definition for RCC_CR register  *******************

const
  RCC_CR_HSION_Pos* = (0)
  RCC_CR_HSION_Msk* = (0x00000001 shl RCC_CR_HSION_Pos) ## !< 0x00000001
  RCC_CR_HSION* = RCC_CR_HSION_Msk
  RCC_CR_HSIRDY_Pos* = (1)
  RCC_CR_HSIRDY_Msk* = (0x00000001 shl RCC_CR_HSIRDY_Pos) ## !< 0x00000002
  RCC_CR_HSIRDY* = RCC_CR_HSIRDY_Msk
  RCC_CR_MSION_Pos* = (8)
  RCC_CR_MSION_Msk* = (0x00000001 shl RCC_CR_MSION_Pos) ## !< 0x00000100
  RCC_CR_MSION* = RCC_CR_MSION_Msk
  RCC_CR_MSIRDY_Pos* = (9)
  RCC_CR_MSIRDY_Msk* = (0x00000001 shl RCC_CR_MSIRDY_Pos) ## !< 0x00000200
  RCC_CR_MSIRDY* = RCC_CR_MSIRDY_Msk
  RCC_CR_HSEON_Pos* = (16)
  RCC_CR_HSEON_Msk* = (0x00000001 shl RCC_CR_HSEON_Pos) ## !< 0x00010000
  RCC_CR_HSEON* = RCC_CR_HSEON_Msk
  RCC_CR_HSERDY_Pos* = (17)
  RCC_CR_HSERDY_Msk* = (0x00000001 shl RCC_CR_HSERDY_Pos) ## !< 0x00020000
  RCC_CR_HSERDY* = RCC_CR_HSERDY_Msk
  RCC_CR_HSEBYP_Pos* = (18)
  RCC_CR_HSEBYP_Msk* = (0x00000001 shl RCC_CR_HSEBYP_Pos) ## !< 0x00040000
  RCC_CR_HSEBYP* = RCC_CR_HSEBYP_Msk
  RCC_CR_PLLON_Pos* = (24)
  RCC_CR_PLLON_Msk* = (0x00000001 shl RCC_CR_PLLON_Pos) ## !< 0x01000000
  RCC_CR_PLLON* = RCC_CR_PLLON_Msk
  RCC_CR_PLLRDY_Pos* = (25)
  RCC_CR_PLLRDY_Msk* = (0x00000001 shl RCC_CR_PLLRDY_Pos) ## !< 0x02000000
  RCC_CR_PLLRDY* = RCC_CR_PLLRDY_Msk
  RCC_CR_CSSON_Pos* = (28)
  RCC_CR_CSSON_Msk* = (0x00000001 shl RCC_CR_CSSON_Pos) ## !< 0x10000000
  RCC_CR_CSSON* = RCC_CR_CSSON_Msk
  RCC_CR_RTCPRE_Pos* = (29)
  RCC_CR_RTCPRE_Msk* = (0x00000003 shl RCC_CR_RTCPRE_Pos) ## !< 0x60000000
  RCC_CR_RTCPRE* = RCC_CR_RTCPRE_Msk
  RCC_CR_RTCPRE_0* = (0x20000000) ## !< Bit0
  RCC_CR_RTCPRE_1* = (0x40000000) ## !< Bit1

## *******************  Bit definition for RCC_ICSCR register  ****************

const
  RCC_ICSCR_HSICAL_Pos* = (0)
  RCC_ICSCR_HSICAL_Msk* = (0x000000FF shl RCC_ICSCR_HSICAL_Pos) ## !< 0x000000FF
  RCC_ICSCR_HSICAL* = RCC_ICSCR_HSICAL_Msk
  RCC_ICSCR_HSITRIM_Pos* = (8)
  RCC_ICSCR_HSITRIM_Msk* = (0x0000001F shl RCC_ICSCR_HSITRIM_Pos) ## !< 0x00001F00
  RCC_ICSCR_HSITRIM* = RCC_ICSCR_HSITRIM_Msk
  RCC_ICSCR_MSIRANGE_Pos* = (13)
  RCC_ICSCR_MSIRANGE_Msk* = (0x00000007 shl RCC_ICSCR_MSIRANGE_Pos) ## !< 0x0000E000
  RCC_ICSCR_MSIRANGE* = RCC_ICSCR_MSIRANGE_Msk
  RCC_ICSCR_MSIRANGE_0* = (0x00000000 shl RCC_ICSCR_MSIRANGE_Pos) ## !< 0x00000000
  RCC_ICSCR_MSIRANGE_1* = (0x00000001 shl RCC_ICSCR_MSIRANGE_Pos) ## !< 0x00002000
  RCC_ICSCR_MSIRANGE_2* = (0x00000002 shl RCC_ICSCR_MSIRANGE_Pos) ## !< 0x00004000
  RCC_ICSCR_MSIRANGE_3* = (0x00000003 shl RCC_ICSCR_MSIRANGE_Pos) ## !< 0x00006000
  RCC_ICSCR_MSIRANGE_4* = (0x00000004 shl RCC_ICSCR_MSIRANGE_Pos) ## !< 0x00008000
  RCC_ICSCR_MSIRANGE_5* = (0x00000005 shl RCC_ICSCR_MSIRANGE_Pos) ## !< 0x0000A000
  RCC_ICSCR_MSIRANGE_6* = (0x00000006 shl RCC_ICSCR_MSIRANGE_Pos) ## !< 0x0000C000
  RCC_ICSCR_MSICAL_Pos* = (16)
  RCC_ICSCR_MSICAL_Msk* = (0x000000FF shl RCC_ICSCR_MSICAL_Pos) ## !< 0x00FF0000
  RCC_ICSCR_MSICAL* = RCC_ICSCR_MSICAL_Msk
  RCC_ICSCR_MSITRIM_Pos* = (24)
  RCC_ICSCR_MSITRIM_Msk* = (0x000000FF shl RCC_ICSCR_MSITRIM_Pos) ## !< 0xFF000000
  RCC_ICSCR_MSITRIM* = RCC_ICSCR_MSITRIM_Msk

## *******************  Bit definition for RCC_CFGR register  *****************

const
  RCC_CFGR_SW_Pos* = (0)
  RCC_CFGR_SW_Msk* = (0x00000003 shl RCC_CFGR_SW_Pos) ## !< 0x00000003
  RCC_CFGR_SW* = RCC_CFGR_SW_Msk
  RCC_CFGR_SW_0* = (0x00000001 shl RCC_CFGR_SW_Pos) ## !< 0x00000001
  RCC_CFGR_SW_1* = (0x00000002 shl RCC_CFGR_SW_Pos) ## !< 0x00000002

## !< SW configuration

const
  RCC_CFGR_SW_MSI* = (0x00000000) ## !< MSI selected as system clock
  RCC_CFGR_SW_HSI* = (0x00000001) ## !< HSI selected as system clock
  RCC_CFGR_SW_HSE* = (0x00000002) ## !< HSE selected as system clock
  RCC_CFGR_SW_PLL* = (0x00000003) ## !< PLL selected as system clock
  RCC_CFGR_SWS_Pos* = (2)
  RCC_CFGR_SWS_Msk* = (0x00000003 shl RCC_CFGR_SWS_Pos) ## !< 0x0000000C
  RCC_CFGR_SWS* = RCC_CFGR_SWS_Msk
  RCC_CFGR_SWS_0* = (0x00000001 shl RCC_CFGR_SWS_Pos) ## !< 0x00000004
  RCC_CFGR_SWS_1* = (0x00000002 shl RCC_CFGR_SWS_Pos) ## !< 0x00000008

## !< SWS configuration

const
  RCC_CFGR_SWS_MSI* = (0x00000000) ## !< MSI oscillator used as system clock
  RCC_CFGR_SWS_HSI* = (0x00000004) ## !< HSI oscillator used as system clock
  RCC_CFGR_SWS_HSE* = (0x00000008) ## !< HSE oscillator used as system clock
  RCC_CFGR_SWS_PLL* = (0x0000000C) ## !< PLL used as system clock
  RCC_CFGR_HPRE_Pos* = (4)
  RCC_CFGR_HPRE_Msk* = (0x0000000F shl RCC_CFGR_HPRE_Pos) ## !< 0x000000F0
  RCC_CFGR_HPRE* = RCC_CFGR_HPRE_Msk
  RCC_CFGR_HPRE_0* = (0x00000001 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000010
  RCC_CFGR_HPRE_1* = (0x00000002 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000020
  RCC_CFGR_HPRE_2* = (0x00000004 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000040
  RCC_CFGR_HPRE_3* = (0x00000008 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000080

## !< HPRE configuration

const
  RCC_CFGR_HPRE_DIV1* = (0x00000000) ## !< SYSCLK not divided
  RCC_CFGR_HPRE_DIV2* = (0x00000080) ## !< SYSCLK divided by 2
  RCC_CFGR_HPRE_DIV4* = (0x00000090) ## !< SYSCLK divided by 4
  RCC_CFGR_HPRE_DIV8* = (0x000000A0) ## !< SYSCLK divided by 8
  RCC_CFGR_HPRE_DIV16* = (0x000000B0) ## !< SYSCLK divided by 16
  RCC_CFGR_HPRE_DIV64* = (0x000000C0) ## !< SYSCLK divided by 64
  RCC_CFGR_HPRE_DIV128* = (0x000000D0) ## !< SYSCLK divided by 128
  RCC_CFGR_HPRE_DIV256* = (0x000000E0) ## !< SYSCLK divided by 256
  RCC_CFGR_HPRE_DIV512* = (0x000000F0) ## !< SYSCLK divided by 512
  RCC_CFGR_PPRE1_Pos* = (8)
  RCC_CFGR_PPRE1_Msk* = (0x00000007 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00000700
  RCC_CFGR_PPRE1* = RCC_CFGR_PPRE1_Msk
  RCC_CFGR_PPRE1_0* = (0x00000001 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00000100
  RCC_CFGR_PPRE1_1* = (0x00000002 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00000200
  RCC_CFGR_PPRE1_2* = (0x00000004 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00000400

## !< PPRE1 configuration

const
  RCC_CFGR_PPRE1_DIV1* = (0x00000000) ## !< HCLK not divided
  RCC_CFGR_PPRE1_DIV2* = (0x00000400) ## !< HCLK divided by 2
  RCC_CFGR_PPRE1_DIV4* = (0x00000500) ## !< HCLK divided by 4
  RCC_CFGR_PPRE1_DIV8* = (0x00000600) ## !< HCLK divided by 8
  RCC_CFGR_PPRE1_DIV16* = (0x00000700) ## !< HCLK divided by 16
  RCC_CFGR_PPRE2_Pos* = (11)
  RCC_CFGR_PPRE2_Msk* = (0x00000007 shl RCC_CFGR_PPRE2_Pos) ## !< 0x00003800
  RCC_CFGR_PPRE2* = RCC_CFGR_PPRE2_Msk
  RCC_CFGR_PPRE2_0* = (0x00000001 shl RCC_CFGR_PPRE2_Pos) ## !< 0x00000800
  RCC_CFGR_PPRE2_1* = (0x00000002 shl RCC_CFGR_PPRE2_Pos) ## !< 0x00001000
  RCC_CFGR_PPRE2_2* = (0x00000004 shl RCC_CFGR_PPRE2_Pos) ## !< 0x00002000

## !< PPRE2 configuration

const
  RCC_CFGR_PPRE2_DIV1* = (0x00000000) ## !< HCLK not divided
  RCC_CFGR_PPRE2_DIV2* = (0x00002000) ## !< HCLK divided by 2
  RCC_CFGR_PPRE2_DIV4* = (0x00002800) ## !< HCLK divided by 4
  RCC_CFGR_PPRE2_DIV8* = (0x00003000) ## !< HCLK divided by 8
  RCC_CFGR_PPRE2_DIV16* = (0x00003800) ## !< HCLK divided by 16

## !< PLL entry clock source

const
  RCC_CFGR_PLLSRC_Pos* = (16)
  RCC_CFGR_PLLSRC_Msk* = (0x00000001 shl RCC_CFGR_PLLSRC_Pos) ## !< 0x00010000
  RCC_CFGR_PLLSRC* = RCC_CFGR_PLLSRC_Msk
  RCC_CFGR_PLLSRC_HSI* = (0x00000000) ## !< HSI as PLL entry clock source
  RCC_CFGR_PLLSRC_HSE* = (0x00010000) ## !< HSE as PLL entry clock source

## !< PLLMUL configuration

const
  RCC_CFGR_PLLMUL_Pos* = (18)
  RCC_CFGR_PLLMUL_Msk* = (0x0000000F shl RCC_CFGR_PLLMUL_Pos) ## !< 0x003C0000
  RCC_CFGR_PLLMUL* = RCC_CFGR_PLLMUL_Msk
  RCC_CFGR_PLLMUL_0* = (0x00000001 shl RCC_CFGR_PLLMUL_Pos) ## !< 0x00040000
  RCC_CFGR_PLLMUL_1* = (0x00000002 shl RCC_CFGR_PLLMUL_Pos) ## !< 0x00080000
  RCC_CFGR_PLLMUL_2* = (0x00000004 shl RCC_CFGR_PLLMUL_Pos) ## !< 0x00100000
  RCC_CFGR_PLLMUL_3* = (0x00000008 shl RCC_CFGR_PLLMUL_Pos) ## !< 0x00200000

## !< PLLMUL configuration

const
  RCC_CFGR_PLLMUL3x* = (0x00000000) ## !< PLL input clock * 3
  RCC_CFGR_PLLMUL4* = (0x00040000) ## !< PLL input clock * 4
  RCC_CFGR_PLLMUL6* = (0x00080000) ## !< PLL input clock * 6
  RCC_CFGR_PLLMUL8* = (0x000C0000) ## !< PLL input clock * 8
  RCC_CFGR_PLLMUL12* = (0x00100000) ## !< PLL input clock * 12
  RCC_CFGR_PLLMUL16* = (0x00140000) ## !< PLL input clock * 16
  RCC_CFGR_PLLMUL24* = (0x00180000) ## !< PLL input clock * 24
  RCC_CFGR_PLLMUL32* = (0x001C0000) ## !< PLL input clock * 32
  RCC_CFGR_PLLMUL48* = (0x00200000) ## !< PLL input clock * 48

## !< PLLDIV configuration

const
  RCC_CFGR_PLLDIV_Pos* = (22)
  RCC_CFGR_PLLDIV_Msk* = (0x00000003 shl RCC_CFGR_PLLDIV_Pos) ## !< 0x00C00000
  RCC_CFGR_PLLDIV* = RCC_CFGR_PLLDIV_Msk
  RCC_CFGR_PLLDIV_0* = (0x00000001 shl RCC_CFGR_PLLDIV_Pos) ## !< 0x00400000
  RCC_CFGR_PLLDIV_1* = (0x00000002 shl RCC_CFGR_PLLDIV_Pos) ## !< 0x00800000

## !< PLLDIV configuration

const
  RCC_CFGR_PLLDIV1x* = (0x00000000) ## !< PLL clock output = CKVCO / 1
  RCC_CFGR_PLLDIV2_Pos* = (22)
  RCC_CFGR_PLLDIV2_Msk* = (0x00000001 shl RCC_CFGR_PLLDIV2_Pos) ## !< 0x00400000
  RCC_CFGR_PLLDIV2* = RCC_CFGR_PLLDIV2_Msk
  RCC_CFGR_PLLDIV3_Pos* = (23)
  RCC_CFGR_PLLDIV3_Msk* = (0x00000001 shl RCC_CFGR_PLLDIV3_Pos) ## !< 0x00800000
  RCC_CFGR_PLLDIV3* = RCC_CFGR_PLLDIV3_Msk
  RCC_CFGR_PLLDIV4_Pos* = (22)
  RCC_CFGR_PLLDIV4_Msk* = (0x00000003 shl RCC_CFGR_PLLDIV4_Pos) ## !< 0x00C00000
  RCC_CFGR_PLLDIV4* = RCC_CFGR_PLLDIV4_Msk
  RCC_CFGR_MCOSEL_Pos* = (24)
  RCC_CFGR_MCOSEL_Msk* = (0x00000007 shl RCC_CFGR_MCOSEL_Pos) ## !< 0x07000000
  RCC_CFGR_MCOSEL* = RCC_CFGR_MCOSEL_Msk
  RCC_CFGR_MCOSEL_0* = (0x00000001 shl RCC_CFGR_MCOSEL_Pos) ## !< 0x01000000
  RCC_CFGR_MCOSEL_1* = (0x00000002 shl RCC_CFGR_MCOSEL_Pos) ## !< 0x02000000
  RCC_CFGR_MCOSEL_2* = (0x00000004 shl RCC_CFGR_MCOSEL_Pos) ## !< 0x04000000

## !< MCO configuration

const
  RCC_CFGR_MCOSEL_NOCLOCK* = (0x00000000) ## !< No clock
  RCC_CFGR_MCOSEL_SYSCLK_Pos* = (24)
  RCC_CFGR_MCOSEL_SYSCLK_Msk* = (0x00000001 shl RCC_CFGR_MCOSEL_SYSCLK_Pos) ## !< 0x01000000
  RCC_CFGR_MCOSEL_SYSCLK* = RCC_CFGR_MCOSEL_SYSCLK_Msk
  RCC_CFGR_MCOSEL_HSI_Pos* = (25)
  RCC_CFGR_MCOSEL_HSI_Msk* = (0x00000001 shl RCC_CFGR_MCOSEL_HSI_Pos) ## !< 0x02000000
  RCC_CFGR_MCOSEL_HSI* = RCC_CFGR_MCOSEL_HSI_Msk
  RCC_CFGR_MCOSEL_MSI_Pos* = (24)
  RCC_CFGR_MCOSEL_MSI_Msk* = (0x00000003 shl RCC_CFGR_MCOSEL_MSI_Pos) ## !< 0x03000000
  RCC_CFGR_MCOSEL_MSI* = RCC_CFGR_MCOSEL_MSI_Msk
  RCC_CFGR_MCOSEL_HSE_Pos* = (26)
  RCC_CFGR_MCOSEL_HSE_Msk* = (0x00000001 shl RCC_CFGR_MCOSEL_HSE_Pos) ## !< 0x04000000
  RCC_CFGR_MCOSEL_HSE* = RCC_CFGR_MCOSEL_HSE_Msk
  RCC_CFGR_MCOSEL_PLL_Pos* = (24)
  RCC_CFGR_MCOSEL_PLL_Msk* = (0x00000005 shl RCC_CFGR_MCOSEL_PLL_Pos) ## !< 0x05000000
  RCC_CFGR_MCOSEL_PLL* = RCC_CFGR_MCOSEL_PLL_Msk
  RCC_CFGR_MCOSEL_LSI_Pos* = (25)
  RCC_CFGR_MCOSEL_LSI_Msk* = (0x00000003 shl RCC_CFGR_MCOSEL_LSI_Pos) ## !< 0x06000000
  RCC_CFGR_MCOSEL_LSI* = RCC_CFGR_MCOSEL_LSI_Msk
  RCC_CFGR_MCOSEL_LSE_Pos* = (24)
  RCC_CFGR_MCOSEL_LSE_Msk* = (0x00000007 shl RCC_CFGR_MCOSEL_LSE_Pos) ## !< 0x07000000
  RCC_CFGR_MCOSEL_LSE* = RCC_CFGR_MCOSEL_LSE_Msk
  RCC_CFGR_MCOPRE_Pos* = (28)
  RCC_CFGR_MCOPRE_Msk* = (0x00000007 shl RCC_CFGR_MCOPRE_Pos) ## !< 0x70000000
  RCC_CFGR_MCOPRE* = RCC_CFGR_MCOPRE_Msk
  RCC_CFGR_MCOPRE_0* = (0x00000001 shl RCC_CFGR_MCOPRE_Pos) ## !< 0x10000000
  RCC_CFGR_MCOPRE_1* = (0x00000002 shl RCC_CFGR_MCOPRE_Pos) ## !< 0x20000000
  RCC_CFGR_MCOPRE_2* = (0x00000004 shl RCC_CFGR_MCOPRE_Pos) ## !< 0x40000000

## !< MCO Prescaler configuration

const
  RCC_CFGR_MCOPRE_DIV1* = (0x00000000) ## !< MCO is divided by 1
  RCC_CFGR_MCOPRE_DIV2* = (0x10000000) ## !< MCO is divided by 2
  RCC_CFGR_MCOPRE_DIV4* = (0x20000000) ## !< MCO is divided by 4
  RCC_CFGR_MCOPRE_DIV8* = (0x30000000) ## !< MCO is divided by 8
  RCC_CFGR_MCOPRE_DIV16* = (0x40000000) ## !< MCO is divided by 16

##  Legacy aliases

const
  RCC_CFGR_MCO_DIV1* = RCC_CFGR_MCOPRE_DIV1
  RCC_CFGR_MCO_DIV2* = RCC_CFGR_MCOPRE_DIV2
  RCC_CFGR_MCO_DIV4* = RCC_CFGR_MCOPRE_DIV4
  RCC_CFGR_MCO_DIV8* = RCC_CFGR_MCOPRE_DIV8
  RCC_CFGR_MCO_DIV16* = RCC_CFGR_MCOPRE_DIV16
  RCC_CFGR_MCO_NOCLOCK* = RCC_CFGR_MCOSEL_NOCLOCK
  RCC_CFGR_MCO_SYSCLK* = RCC_CFGR_MCOSEL_SYSCLK
  RCC_CFGR_MCO_HSI* = RCC_CFGR_MCOSEL_HSI
  RCC_CFGR_MCO_MSI* = RCC_CFGR_MCOSEL_MSI
  RCC_CFGR_MCO_HSE* = RCC_CFGR_MCOSEL_HSE
  RCC_CFGR_MCO_PLL* = RCC_CFGR_MCOSEL_PLL
  RCC_CFGR_MCO_LSI* = RCC_CFGR_MCOSEL_LSI
  RCC_CFGR_MCO_LSE* = RCC_CFGR_MCOSEL_LSE

## !<******************  Bit definition for RCC_CIR register  *******************

const
  RCC_CIR_LSIRDYF_Pos* = (0)
  RCC_CIR_LSIRDYF_Msk* = (0x00000001 shl RCC_CIR_LSIRDYF_Pos) ## !< 0x00000001
  RCC_CIR_LSIRDYF* = RCC_CIR_LSIRDYF_Msk
  RCC_CIR_LSERDYF_Pos* = (1)
  RCC_CIR_LSERDYF_Msk* = (0x00000001 shl RCC_CIR_LSERDYF_Pos) ## !< 0x00000002
  RCC_CIR_LSERDYF* = RCC_CIR_LSERDYF_Msk
  RCC_CIR_HSIRDYF_Pos* = (2)
  RCC_CIR_HSIRDYF_Msk* = (0x00000001 shl RCC_CIR_HSIRDYF_Pos) ## !< 0x00000004
  RCC_CIR_HSIRDYF* = RCC_CIR_HSIRDYF_Msk
  RCC_CIR_HSERDYF_Pos* = (3)
  RCC_CIR_HSERDYF_Msk* = (0x00000001 shl RCC_CIR_HSERDYF_Pos) ## !< 0x00000008
  RCC_CIR_HSERDYF* = RCC_CIR_HSERDYF_Msk
  RCC_CIR_PLLRDYF_Pos* = (4)
  RCC_CIR_PLLRDYF_Msk* = (0x00000001 shl RCC_CIR_PLLRDYF_Pos) ## !< 0x00000010
  RCC_CIR_PLLRDYF* = RCC_CIR_PLLRDYF_Msk
  RCC_CIR_MSIRDYF_Pos* = (5)
  RCC_CIR_MSIRDYF_Msk* = (0x00000001 shl RCC_CIR_MSIRDYF_Pos) ## !< 0x00000020
  RCC_CIR_MSIRDYF* = RCC_CIR_MSIRDYF_Msk
  RCC_CIR_LSECSSF_Pos* = (6)
  RCC_CIR_LSECSSF_Msk* = (0x00000001 shl RCC_CIR_LSECSSF_Pos) ## !< 0x00000040
  RCC_CIR_LSECSSF* = RCC_CIR_LSECSSF_Msk
  RCC_CIR_CSSF_Pos* = (7)
  RCC_CIR_CSSF_Msk* = (0x00000001 shl RCC_CIR_CSSF_Pos) ## !< 0x00000080
  RCC_CIR_CSSF* = RCC_CIR_CSSF_Msk
  RCC_CIR_LSIRDYIE_Pos* = (8)
  RCC_CIR_LSIRDYIE_Msk* = (0x00000001 shl RCC_CIR_LSIRDYIE_Pos) ## !< 0x00000100
  RCC_CIR_LSIRDYIE* = RCC_CIR_LSIRDYIE_Msk
  RCC_CIR_LSERDYIE_Pos* = (9)
  RCC_CIR_LSERDYIE_Msk* = (0x00000001 shl RCC_CIR_LSERDYIE_Pos) ## !< 0x00000200
  RCC_CIR_LSERDYIE* = RCC_CIR_LSERDYIE_Msk
  RCC_CIR_HSIRDYIE_Pos* = (10)
  RCC_CIR_HSIRDYIE_Msk* = (0x00000001 shl RCC_CIR_HSIRDYIE_Pos) ## !< 0x00000400
  RCC_CIR_HSIRDYIE* = RCC_CIR_HSIRDYIE_Msk
  RCC_CIR_HSERDYIE_Pos* = (11)
  RCC_CIR_HSERDYIE_Msk* = (0x00000001 shl RCC_CIR_HSERDYIE_Pos) ## !< 0x00000800
  RCC_CIR_HSERDYIE* = RCC_CIR_HSERDYIE_Msk
  RCC_CIR_PLLRDYIE_Pos* = (12)
  RCC_CIR_PLLRDYIE_Msk* = (0x00000001 shl RCC_CIR_PLLRDYIE_Pos) ## !< 0x00001000
  RCC_CIR_PLLRDYIE* = RCC_CIR_PLLRDYIE_Msk
  RCC_CIR_MSIRDYIE_Pos* = (13)
  RCC_CIR_MSIRDYIE_Msk* = (0x00000001 shl RCC_CIR_MSIRDYIE_Pos) ## !< 0x00002000
  RCC_CIR_MSIRDYIE* = RCC_CIR_MSIRDYIE_Msk
  RCC_CIR_LSECSSIE_Pos* = (14)
  RCC_CIR_LSECSSIE_Msk* = (0x00000001 shl RCC_CIR_LSECSSIE_Pos) ## !< 0x00004000
  RCC_CIR_LSECSSIE* = RCC_CIR_LSECSSIE_Msk
  RCC_CIR_LSIRDYC_Pos* = (16)
  RCC_CIR_LSIRDYC_Msk* = (0x00000001 shl RCC_CIR_LSIRDYC_Pos) ## !< 0x00010000
  RCC_CIR_LSIRDYC* = RCC_CIR_LSIRDYC_Msk
  RCC_CIR_LSERDYC_Pos* = (17)
  RCC_CIR_LSERDYC_Msk* = (0x00000001 shl RCC_CIR_LSERDYC_Pos) ## !< 0x00020000
  RCC_CIR_LSERDYC* = RCC_CIR_LSERDYC_Msk
  RCC_CIR_HSIRDYC_Pos* = (18)
  RCC_CIR_HSIRDYC_Msk* = (0x00000001 shl RCC_CIR_HSIRDYC_Pos) ## !< 0x00040000
  RCC_CIR_HSIRDYC* = RCC_CIR_HSIRDYC_Msk
  RCC_CIR_HSERDYC_Pos* = (19)
  RCC_CIR_HSERDYC_Msk* = (0x00000001 shl RCC_CIR_HSERDYC_Pos) ## !< 0x00080000
  RCC_CIR_HSERDYC* = RCC_CIR_HSERDYC_Msk
  RCC_CIR_PLLRDYC_Pos* = (20)
  RCC_CIR_PLLRDYC_Msk* = (0x00000001 shl RCC_CIR_PLLRDYC_Pos) ## !< 0x00100000
  RCC_CIR_PLLRDYC* = RCC_CIR_PLLRDYC_Msk
  RCC_CIR_MSIRDYC_Pos* = (21)
  RCC_CIR_MSIRDYC_Msk* = (0x00000001 shl RCC_CIR_MSIRDYC_Pos) ## !< 0x00200000
  RCC_CIR_MSIRDYC* = RCC_CIR_MSIRDYC_Msk
  RCC_CIR_LSECSSC_Pos* = (22)
  RCC_CIR_LSECSSC_Msk* = (0x00000001 shl RCC_CIR_LSECSSC_Pos) ## !< 0x00400000
  RCC_CIR_LSECSSC* = RCC_CIR_LSECSSC_Msk
  RCC_CIR_CSSC_Pos* = (23)
  RCC_CIR_CSSC_Msk* = (0x00000001 shl RCC_CIR_CSSC_Pos) ## !< 0x00800000
  RCC_CIR_CSSC* = RCC_CIR_CSSC_Msk

## ****************  Bit definition for RCC_AHBRSTR register  *****************

const
  RCC_AHBRSTR_GPIOARST_Pos* = (0)
  RCC_AHBRSTR_GPIOARST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOARST_Pos) ## !< 0x00000001
  RCC_AHBRSTR_GPIOARST* = RCC_AHBRSTR_GPIOARST_Msk
  RCC_AHBRSTR_GPIOBRST_Pos* = (1)
  RCC_AHBRSTR_GPIOBRST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOBRST_Pos) ## !< 0x00000002
  RCC_AHBRSTR_GPIOBRST* = RCC_AHBRSTR_GPIOBRST_Msk
  RCC_AHBRSTR_GPIOCRST_Pos* = (2)
  RCC_AHBRSTR_GPIOCRST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOCRST_Pos) ## !< 0x00000004
  RCC_AHBRSTR_GPIOCRST* = RCC_AHBRSTR_GPIOCRST_Msk
  RCC_AHBRSTR_GPIODRST_Pos* = (3)
  RCC_AHBRSTR_GPIODRST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIODRST_Pos) ## !< 0x00000008
  RCC_AHBRSTR_GPIODRST* = RCC_AHBRSTR_GPIODRST_Msk
  RCC_AHBRSTR_GPIOERST_Pos* = (4)
  RCC_AHBRSTR_GPIOERST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOERST_Pos) ## !< 0x00000010
  RCC_AHBRSTR_GPIOERST* = RCC_AHBRSTR_GPIOERST_Msk
  RCC_AHBRSTR_GPIOHRST_Pos* = (5)
  RCC_AHBRSTR_GPIOHRST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOHRST_Pos) ## !< 0x00000020
  RCC_AHBRSTR_GPIOHRST* = RCC_AHBRSTR_GPIOHRST_Msk
  RCC_AHBRSTR_GPIOFRST_Pos* = (6)
  RCC_AHBRSTR_GPIOFRST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOFRST_Pos) ## !< 0x00000040
  RCC_AHBRSTR_GPIOFRST* = RCC_AHBRSTR_GPIOFRST_Msk
  RCC_AHBRSTR_GPIOGRST_Pos* = (7)
  RCC_AHBRSTR_GPIOGRST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOGRST_Pos) ## !< 0x00000080
  RCC_AHBRSTR_GPIOGRST* = RCC_AHBRSTR_GPIOGRST_Msk
  RCC_AHBRSTR_CRCRST_Pos* = (12)
  RCC_AHBRSTR_CRCRST_Msk* = (0x00000001 shl RCC_AHBRSTR_CRCRST_Pos) ## !< 0x00001000
  RCC_AHBRSTR_CRCRST* = RCC_AHBRSTR_CRCRST_Msk
  RCC_AHBRSTR_FLITFRST_Pos* = (15)
  RCC_AHBRSTR_FLITFRST_Msk* = (0x00000001 shl RCC_AHBRSTR_FLITFRST_Pos) ## !< 0x00008000
  RCC_AHBRSTR_FLITFRST* = RCC_AHBRSTR_FLITFRST_Msk
  RCC_AHBRSTR_DMA1RST_Pos* = (24)
  RCC_AHBRSTR_DMA1RST_Msk* = (0x00000001 shl RCC_AHBRSTR_DMA1RST_Pos) ## !< 0x01000000
  RCC_AHBRSTR_DMA1RST* = RCC_AHBRSTR_DMA1RST_Msk
  RCC_AHBRSTR_DMA2RST_Pos* = (25)
  RCC_AHBRSTR_DMA2RST_Msk* = (0x00000001 shl RCC_AHBRSTR_DMA2RST_Pos) ## !< 0x02000000
  RCC_AHBRSTR_DMA2RST* = RCC_AHBRSTR_DMA2RST_Msk

## ****************  Bit definition for RCC_APB2RSTR register  ****************

const
  RCC_APB2RSTR_SYSCFGRST_Pos* = (0)
  RCC_APB2RSTR_SYSCFGRST_Msk* = (0x00000001 shl RCC_APB2RSTR_SYSCFGRST_Pos) ## !< 0x00000001
  RCC_APB2RSTR_SYSCFGRST* = RCC_APB2RSTR_SYSCFGRST_Msk
  RCC_APB2RSTR_TIM9RST_Pos* = (2)
  RCC_APB2RSTR_TIM9RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM9RST_Pos) ## !< 0x00000004
  RCC_APB2RSTR_TIM9RST* = RCC_APB2RSTR_TIM9RST_Msk
  RCC_APB2RSTR_TIM10RST_Pos* = (3)
  RCC_APB2RSTR_TIM10RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM10RST_Pos) ## !< 0x00000008
  RCC_APB2RSTR_TIM10RST* = RCC_APB2RSTR_TIM10RST_Msk
  RCC_APB2RSTR_TIM11RST_Pos* = (4)
  RCC_APB2RSTR_TIM11RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM11RST_Pos) ## !< 0x00000010
  RCC_APB2RSTR_TIM11RST* = RCC_APB2RSTR_TIM11RST_Msk
  RCC_APB2RSTR_ADC1RST_Pos* = (9)
  RCC_APB2RSTR_ADC1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_ADC1RST_Pos) ## !< 0x00000200
  RCC_APB2RSTR_ADC1RST* = RCC_APB2RSTR_ADC1RST_Msk
  RCC_APB2RSTR_SPI1RST_Pos* = (12)
  RCC_APB2RSTR_SPI1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_SPI1RST_Pos) ## !< 0x00001000
  RCC_APB2RSTR_SPI1RST* = RCC_APB2RSTR_SPI1RST_Msk
  RCC_APB2RSTR_USART1RST_Pos* = (14)
  RCC_APB2RSTR_USART1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_USART1RST_Pos) ## !< 0x00004000
  RCC_APB2RSTR_USART1RST* = RCC_APB2RSTR_USART1RST_Msk

## ****************  Bit definition for RCC_APB1RSTR register  ****************

const
  RCC_APB1RSTR_TIM2RST_Pos* = (0)
  RCC_APB1RSTR_TIM2RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM2RST_Pos) ## !< 0x00000001
  RCC_APB1RSTR_TIM2RST* = RCC_APB1RSTR_TIM2RST_Msk
  RCC_APB1RSTR_TIM3RST_Pos* = (1)
  RCC_APB1RSTR_TIM3RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM3RST_Pos) ## !< 0x00000002
  RCC_APB1RSTR_TIM3RST* = RCC_APB1RSTR_TIM3RST_Msk
  RCC_APB1RSTR_TIM4RST_Pos* = (2)
  RCC_APB1RSTR_TIM4RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM4RST_Pos) ## !< 0x00000004
  RCC_APB1RSTR_TIM4RST* = RCC_APB1RSTR_TIM4RST_Msk
  RCC_APB1RSTR_TIM5RST_Pos* = (3)
  RCC_APB1RSTR_TIM5RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM5RST_Pos) ## !< 0x00000008
  RCC_APB1RSTR_TIM5RST* = RCC_APB1RSTR_TIM5RST_Msk
  RCC_APB1RSTR_TIM6RST_Pos* = (4)
  RCC_APB1RSTR_TIM6RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM6RST_Pos) ## !< 0x00000010
  RCC_APB1RSTR_TIM6RST* = RCC_APB1RSTR_TIM6RST_Msk
  RCC_APB1RSTR_TIM7RST_Pos* = (5)
  RCC_APB1RSTR_TIM7RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM7RST_Pos) ## !< 0x00000020
  RCC_APB1RSTR_TIM7RST* = RCC_APB1RSTR_TIM7RST_Msk
  RCC_APB1RSTR_LCDRST_Pos* = (9)
  RCC_APB1RSTR_LCDRST_Msk* = (0x00000001 shl RCC_APB1RSTR_LCDRST_Pos) ## !< 0x00000200
  RCC_APB1RSTR_LCDRST* = RCC_APB1RSTR_LCDRST_Msk
  RCC_APB1RSTR_WWDGRST_Pos* = (11)
  RCC_APB1RSTR_WWDGRST_Msk* = (0x00000001 shl RCC_APB1RSTR_WWDGRST_Pos) ## !< 0x00000800
  RCC_APB1RSTR_WWDGRST* = RCC_APB1RSTR_WWDGRST_Msk
  RCC_APB1RSTR_SPI2RST_Pos* = (14)
  RCC_APB1RSTR_SPI2RST_Msk* = (0x00000001 shl RCC_APB1RSTR_SPI2RST_Pos) ## !< 0x00004000
  RCC_APB1RSTR_SPI2RST* = RCC_APB1RSTR_SPI2RST_Msk
  RCC_APB1RSTR_SPI3RST_Pos* = (15)
  RCC_APB1RSTR_SPI3RST_Msk* = (0x00000001 shl RCC_APB1RSTR_SPI3RST_Pos) ## !< 0x00008000
  RCC_APB1RSTR_SPI3RST* = RCC_APB1RSTR_SPI3RST_Msk
  RCC_APB1RSTR_USART2RST_Pos* = (17)
  RCC_APB1RSTR_USART2RST_Msk* = (0x00000001 shl RCC_APB1RSTR_USART2RST_Pos) ## !< 0x00020000
  RCC_APB1RSTR_USART2RST* = RCC_APB1RSTR_USART2RST_Msk
  RCC_APB1RSTR_USART3RST_Pos* = (18)
  RCC_APB1RSTR_USART3RST_Msk* = (0x00000001 shl RCC_APB1RSTR_USART3RST_Pos) ## !< 0x00040000
  RCC_APB1RSTR_USART3RST* = RCC_APB1RSTR_USART3RST_Msk
  RCC_APB1RSTR_UART4RST_Pos* = (19)
  RCC_APB1RSTR_UART4RST_Msk* = (0x00000001 shl RCC_APB1RSTR_UART4RST_Pos) ## !< 0x00080000
  RCC_APB1RSTR_UART4RST* = RCC_APB1RSTR_UART4RST_Msk
  RCC_APB1RSTR_UART5RST_Pos* = (20)
  RCC_APB1RSTR_UART5RST_Msk* = (0x00000001 shl RCC_APB1RSTR_UART5RST_Pos) ## !< 0x00100000
  RCC_APB1RSTR_UART5RST* = RCC_APB1RSTR_UART5RST_Msk
  RCC_APB1RSTR_I2C1RST_Pos* = (21)
  RCC_APB1RSTR_I2C1RST_Msk* = (0x00000001 shl RCC_APB1RSTR_I2C1RST_Pos) ## !< 0x00200000
  RCC_APB1RSTR_I2C1RST* = RCC_APB1RSTR_I2C1RST_Msk
  RCC_APB1RSTR_I2C2RST_Pos* = (22)
  RCC_APB1RSTR_I2C2RST_Msk* = (0x00000001 shl RCC_APB1RSTR_I2C2RST_Pos) ## !< 0x00400000
  RCC_APB1RSTR_I2C2RST* = RCC_APB1RSTR_I2C2RST_Msk
  RCC_APB1RSTR_USBRST_Pos* = (23)
  RCC_APB1RSTR_USBRST_Msk* = (0x00000001 shl RCC_APB1RSTR_USBRST_Pos) ## !< 0x00800000
  RCC_APB1RSTR_USBRST* = RCC_APB1RSTR_USBRST_Msk
  RCC_APB1RSTR_PWRRST_Pos* = (28)
  RCC_APB1RSTR_PWRRST_Msk* = (0x00000001 shl RCC_APB1RSTR_PWRRST_Pos) ## !< 0x10000000
  RCC_APB1RSTR_PWRRST* = RCC_APB1RSTR_PWRRST_Msk
  RCC_APB1RSTR_DACRST_Pos* = (29)
  RCC_APB1RSTR_DACRST_Msk* = (0x00000001 shl RCC_APB1RSTR_DACRST_Pos) ## !< 0x20000000
  RCC_APB1RSTR_DACRST* = RCC_APB1RSTR_DACRST_Msk
  RCC_APB1RSTR_COMPRST_Pos* = (31)
  RCC_APB1RSTR_COMPRST_Msk* = (0x00000001 shl RCC_APB1RSTR_COMPRST_Pos) ## !< 0x80000000
  RCC_APB1RSTR_COMPRST* = RCC_APB1RSTR_COMPRST_Msk

## *****************  Bit definition for RCC_AHBENR register  *****************

const
  RCC_AHBENR_GPIOAEN_Pos* = (0)
  RCC_AHBENR_GPIOAEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOAEN_Pos) ## !< 0x00000001
  RCC_AHBENR_GPIOAEN* = RCC_AHBENR_GPIOAEN_Msk
  RCC_AHBENR_GPIOBEN_Pos* = (1)
  RCC_AHBENR_GPIOBEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOBEN_Pos) ## !< 0x00000002
  RCC_AHBENR_GPIOBEN* = RCC_AHBENR_GPIOBEN_Msk
  RCC_AHBENR_GPIOCEN_Pos* = (2)
  RCC_AHBENR_GPIOCEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOCEN_Pos) ## !< 0x00000004
  RCC_AHBENR_GPIOCEN* = RCC_AHBENR_GPIOCEN_Msk
  RCC_AHBENR_GPIODEN_Pos* = (3)
  RCC_AHBENR_GPIODEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIODEN_Pos) ## !< 0x00000008
  RCC_AHBENR_GPIODEN* = RCC_AHBENR_GPIODEN_Msk
  RCC_AHBENR_GPIOEEN_Pos* = (4)
  RCC_AHBENR_GPIOEEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOEEN_Pos) ## !< 0x00000010
  RCC_AHBENR_GPIOEEN* = RCC_AHBENR_GPIOEEN_Msk
  RCC_AHBENR_GPIOHEN_Pos* = (5)
  RCC_AHBENR_GPIOHEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOHEN_Pos) ## !< 0x00000020
  RCC_AHBENR_GPIOHEN* = RCC_AHBENR_GPIOHEN_Msk
  RCC_AHBENR_GPIOFEN_Pos* = (6)
  RCC_AHBENR_GPIOFEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOFEN_Pos) ## !< 0x00000040
  RCC_AHBENR_GPIOFEN* = RCC_AHBENR_GPIOFEN_Msk
  RCC_AHBENR_GPIOGEN_Pos* = (7)
  RCC_AHBENR_GPIOGEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOGEN_Pos) ## !< 0x00000080
  RCC_AHBENR_GPIOGEN* = RCC_AHBENR_GPIOGEN_Msk
  RCC_AHBENR_CRCEN_Pos* = (12)
  RCC_AHBENR_CRCEN_Msk* = (0x00000001 shl RCC_AHBENR_CRCEN_Pos) ## !< 0x00001000
  RCC_AHBENR_CRCEN* = RCC_AHBENR_CRCEN_Msk
  RCC_AHBENR_FLITFEN_Pos* = (15)
  RCC_AHBENR_FLITFEN_Msk* = (0x00000001 shl RCC_AHBENR_FLITFEN_Pos) ## !< 0x00008000
  RCC_AHBENR_FLITFEN* = RCC_AHBENR_FLITFEN_Msk
  RCC_AHBENR_DMA1EN_Pos* = (24)
  RCC_AHBENR_DMA1EN_Msk* = (0x00000001 shl RCC_AHBENR_DMA1EN_Pos) ## !< 0x01000000
  RCC_AHBENR_DMA1EN* = RCC_AHBENR_DMA1EN_Msk
  RCC_AHBENR_DMA2EN_Pos* = (25)
  RCC_AHBENR_DMA2EN_Msk* = (0x00000001 shl RCC_AHBENR_DMA2EN_Pos) ## !< 0x02000000
  RCC_AHBENR_DMA2EN* = RCC_AHBENR_DMA2EN_Msk

## *****************  Bit definition for RCC_APB2ENR register  ****************

const
  RCC_APB2ENR_SYSCFGEN_Pos* = (0)
  RCC_APB2ENR_SYSCFGEN_Msk* = (0x00000001 shl RCC_APB2ENR_SYSCFGEN_Pos) ## !< 0x00000001
  RCC_APB2ENR_SYSCFGEN* = RCC_APB2ENR_SYSCFGEN_Msk
  RCC_APB2ENR_TIM9EN_Pos* = (2)
  RCC_APB2ENR_TIM9EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM9EN_Pos) ## !< 0x00000004
  RCC_APB2ENR_TIM9EN* = RCC_APB2ENR_TIM9EN_Msk
  RCC_APB2ENR_TIM10EN_Pos* = (3)
  RCC_APB2ENR_TIM10EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM10EN_Pos) ## !< 0x00000008
  RCC_APB2ENR_TIM10EN* = RCC_APB2ENR_TIM10EN_Msk
  RCC_APB2ENR_TIM11EN_Pos* = (4)
  RCC_APB2ENR_TIM11EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM11EN_Pos) ## !< 0x00000010
  RCC_APB2ENR_TIM11EN* = RCC_APB2ENR_TIM11EN_Msk
  RCC_APB2ENR_ADC1EN_Pos* = (9)
  RCC_APB2ENR_ADC1EN_Msk* = (0x00000001 shl RCC_APB2ENR_ADC1EN_Pos) ## !< 0x00000200
  RCC_APB2ENR_ADC1EN* = RCC_APB2ENR_ADC1EN_Msk
  RCC_APB2ENR_SPI1EN_Pos* = (12)
  RCC_APB2ENR_SPI1EN_Msk* = (0x00000001 shl RCC_APB2ENR_SPI1EN_Pos) ## !< 0x00001000
  RCC_APB2ENR_SPI1EN* = RCC_APB2ENR_SPI1EN_Msk
  RCC_APB2ENR_USART1EN_Pos* = (14)
  RCC_APB2ENR_USART1EN_Msk* = (0x00000001 shl RCC_APB2ENR_USART1EN_Pos) ## !< 0x00004000
  RCC_APB2ENR_USART1EN* = RCC_APB2ENR_USART1EN_Msk

## ****************  Bit definition for RCC_APB1ENR register  *****************

const
  RCC_APB1ENR_TIM2EN_Pos* = (0)
  RCC_APB1ENR_TIM2EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM2EN_Pos) ## !< 0x00000001
  RCC_APB1ENR_TIM2EN* = RCC_APB1ENR_TIM2EN_Msk
  RCC_APB1ENR_TIM3EN_Pos* = (1)
  RCC_APB1ENR_TIM3EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM3EN_Pos) ## !< 0x00000002
  RCC_APB1ENR_TIM3EN* = RCC_APB1ENR_TIM3EN_Msk
  RCC_APB1ENR_TIM4EN_Pos* = (2)
  RCC_APB1ENR_TIM4EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM4EN_Pos) ## !< 0x00000004
  RCC_APB1ENR_TIM4EN* = RCC_APB1ENR_TIM4EN_Msk
  RCC_APB1ENR_TIM5EN_Pos* = (3)
  RCC_APB1ENR_TIM5EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM5EN_Pos) ## !< 0x00000008
  RCC_APB1ENR_TIM5EN* = RCC_APB1ENR_TIM5EN_Msk
  RCC_APB1ENR_TIM6EN_Pos* = (4)
  RCC_APB1ENR_TIM6EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM6EN_Pos) ## !< 0x00000010
  RCC_APB1ENR_TIM6EN* = RCC_APB1ENR_TIM6EN_Msk
  RCC_APB1ENR_TIM7EN_Pos* = (5)
  RCC_APB1ENR_TIM7EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM7EN_Pos) ## !< 0x00000020
  RCC_APB1ENR_TIM7EN* = RCC_APB1ENR_TIM7EN_Msk
  RCC_APB1ENR_LCDEN_Pos* = (9)
  RCC_APB1ENR_LCDEN_Msk* = (0x00000001 shl RCC_APB1ENR_LCDEN_Pos) ## !< 0x00000200
  RCC_APB1ENR_LCDEN* = RCC_APB1ENR_LCDEN_Msk
  RCC_APB1ENR_WWDGEN_Pos* = (11)
  RCC_APB1ENR_WWDGEN_Msk* = (0x00000001 shl RCC_APB1ENR_WWDGEN_Pos) ## !< 0x00000800
  RCC_APB1ENR_WWDGEN* = RCC_APB1ENR_WWDGEN_Msk
  RCC_APB1ENR_SPI2EN_Pos* = (14)
  RCC_APB1ENR_SPI2EN_Msk* = (0x00000001 shl RCC_APB1ENR_SPI2EN_Pos) ## !< 0x00004000
  RCC_APB1ENR_SPI2EN* = RCC_APB1ENR_SPI2EN_Msk
  RCC_APB1ENR_SPI3EN_Pos* = (15)
  RCC_APB1ENR_SPI3EN_Msk* = (0x00000001 shl RCC_APB1ENR_SPI3EN_Pos) ## !< 0x00008000
  RCC_APB1ENR_SPI3EN* = RCC_APB1ENR_SPI3EN_Msk
  RCC_APB1ENR_USART2EN_Pos* = (17)
  RCC_APB1ENR_USART2EN_Msk* = (0x00000001 shl RCC_APB1ENR_USART2EN_Pos) ## !< 0x00020000
  RCC_APB1ENR_USART2EN* = RCC_APB1ENR_USART2EN_Msk
  RCC_APB1ENR_USART3EN_Pos* = (18)
  RCC_APB1ENR_USART3EN_Msk* = (0x00000001 shl RCC_APB1ENR_USART3EN_Pos) ## !< 0x00040000
  RCC_APB1ENR_USART3EN* = RCC_APB1ENR_USART3EN_Msk
  RCC_APB1ENR_UART4EN_Pos* = (19)
  RCC_APB1ENR_UART4EN_Msk* = (0x00000001 shl RCC_APB1ENR_UART4EN_Pos) ## !< 0x00080000
  RCC_APB1ENR_UART4EN* = RCC_APB1ENR_UART4EN_Msk
  RCC_APB1ENR_UART5EN_Pos* = (20)
  RCC_APB1ENR_UART5EN_Msk* = (0x00000001 shl RCC_APB1ENR_UART5EN_Pos) ## !< 0x00100000
  RCC_APB1ENR_UART5EN* = RCC_APB1ENR_UART5EN_Msk
  RCC_APB1ENR_I2C1EN_Pos* = (21)
  RCC_APB1ENR_I2C1EN_Msk* = (0x00000001 shl RCC_APB1ENR_I2C1EN_Pos) ## !< 0x00200000
  RCC_APB1ENR_I2C1EN* = RCC_APB1ENR_I2C1EN_Msk
  RCC_APB1ENR_I2C2EN_Pos* = (22)
  RCC_APB1ENR_I2C2EN_Msk* = (0x00000001 shl RCC_APB1ENR_I2C2EN_Pos) ## !< 0x00400000
  RCC_APB1ENR_I2C2EN* = RCC_APB1ENR_I2C2EN_Msk
  RCC_APB1ENR_USBEN_Pos* = (23)
  RCC_APB1ENR_USBEN_Msk* = (0x00000001 shl RCC_APB1ENR_USBEN_Pos) ## !< 0x00800000
  RCC_APB1ENR_USBEN* = RCC_APB1ENR_USBEN_Msk
  RCC_APB1ENR_PWREN_Pos* = (28)
  RCC_APB1ENR_PWREN_Msk* = (0x00000001 shl RCC_APB1ENR_PWREN_Pos) ## !< 0x10000000
  RCC_APB1ENR_PWREN* = RCC_APB1ENR_PWREN_Msk
  RCC_APB1ENR_DACEN_Pos* = (29)
  RCC_APB1ENR_DACEN_Msk* = (0x00000001 shl RCC_APB1ENR_DACEN_Pos) ## !< 0x20000000
  RCC_APB1ENR_DACEN* = RCC_APB1ENR_DACEN_Msk
  RCC_APB1ENR_COMPEN_Pos* = (31)
  RCC_APB1ENR_COMPEN_Msk* = (0x00000001 shl RCC_APB1ENR_COMPEN_Pos) ## !< 0x80000000
  RCC_APB1ENR_COMPEN* = RCC_APB1ENR_COMPEN_Msk

## *****************  Bit definition for RCC_AHBLPENR register  ***************

const
  RCC_AHBLPENR_GPIOALPEN_Pos* = (0)
  RCC_AHBLPENR_GPIOALPEN_Msk* = (0x00000001 shl RCC_AHBLPENR_GPIOALPEN_Pos) ## !< 0x00000001
  RCC_AHBLPENR_GPIOALPEN* = RCC_AHBLPENR_GPIOALPEN_Msk
  RCC_AHBLPENR_GPIOBLPEN_Pos* = (1)
  RCC_AHBLPENR_GPIOBLPEN_Msk* = (0x00000001 shl RCC_AHBLPENR_GPIOBLPEN_Pos) ## !< 0x00000002
  RCC_AHBLPENR_GPIOBLPEN* = RCC_AHBLPENR_GPIOBLPEN_Msk
  RCC_AHBLPENR_GPIOCLPEN_Pos* = (2)
  RCC_AHBLPENR_GPIOCLPEN_Msk* = (0x00000001 shl RCC_AHBLPENR_GPIOCLPEN_Pos) ## !< 0x00000004
  RCC_AHBLPENR_GPIOCLPEN* = RCC_AHBLPENR_GPIOCLPEN_Msk
  RCC_AHBLPENR_GPIODLPEN_Pos* = (3)
  RCC_AHBLPENR_GPIODLPEN_Msk* = (0x00000001 shl RCC_AHBLPENR_GPIODLPEN_Pos) ## !< 0x00000008
  RCC_AHBLPENR_GPIODLPEN* = RCC_AHBLPENR_GPIODLPEN_Msk
  RCC_AHBLPENR_GPIOELPEN_Pos* = (4)
  RCC_AHBLPENR_GPIOELPEN_Msk* = (0x00000001 shl RCC_AHBLPENR_GPIOELPEN_Pos) ## !< 0x00000010
  RCC_AHBLPENR_GPIOELPEN* = RCC_AHBLPENR_GPIOELPEN_Msk
  RCC_AHBLPENR_GPIOHLPEN_Pos* = (5)
  RCC_AHBLPENR_GPIOHLPEN_Msk* = (0x00000001 shl RCC_AHBLPENR_GPIOHLPEN_Pos) ## !< 0x00000020
  RCC_AHBLPENR_GPIOHLPEN* = RCC_AHBLPENR_GPIOHLPEN_Msk
  RCC_AHBLPENR_GPIOFLPEN_Pos* = (6)
  RCC_AHBLPENR_GPIOFLPEN_Msk* = (0x00000001 shl RCC_AHBLPENR_GPIOFLPEN_Pos) ## !< 0x00000040
  RCC_AHBLPENR_GPIOFLPEN* = RCC_AHBLPENR_GPIOFLPEN_Msk
  RCC_AHBLPENR_GPIOGLPEN_Pos* = (7)
  RCC_AHBLPENR_GPIOGLPEN_Msk* = (0x00000001 shl RCC_AHBLPENR_GPIOGLPEN_Pos) ## !< 0x00000080
  RCC_AHBLPENR_GPIOGLPEN* = RCC_AHBLPENR_GPIOGLPEN_Msk
  RCC_AHBLPENR_CRCLPEN_Pos* = (12)
  RCC_AHBLPENR_CRCLPEN_Msk* = (0x00000001 shl RCC_AHBLPENR_CRCLPEN_Pos) ## !< 0x00001000
  RCC_AHBLPENR_CRCLPEN* = RCC_AHBLPENR_CRCLPEN_Msk
  RCC_AHBLPENR_FLITFLPEN_Pos* = (15)
  RCC_AHBLPENR_FLITFLPEN_Msk* = (0x00000001 shl RCC_AHBLPENR_FLITFLPEN_Pos) ## !< 0x00008000
  RCC_AHBLPENR_FLITFLPEN* = RCC_AHBLPENR_FLITFLPEN_Msk
  RCC_AHBLPENR_SRAMLPEN_Pos* = (16)
  RCC_AHBLPENR_SRAMLPEN_Msk* = (0x00000001 shl RCC_AHBLPENR_SRAMLPEN_Pos) ## !< 0x00010000
  RCC_AHBLPENR_SRAMLPEN* = RCC_AHBLPENR_SRAMLPEN_Msk
  RCC_AHBLPENR_DMA1LPEN_Pos* = (24)
  RCC_AHBLPENR_DMA1LPEN_Msk* = (0x00000001 shl RCC_AHBLPENR_DMA1LPEN_Pos) ## !< 0x01000000
  RCC_AHBLPENR_DMA1LPEN* = RCC_AHBLPENR_DMA1LPEN_Msk
  RCC_AHBLPENR_DMA2LPEN_Pos* = (25)
  RCC_AHBLPENR_DMA2LPEN_Msk* = (0x00000001 shl RCC_AHBLPENR_DMA2LPEN_Pos) ## !< 0x02000000
  RCC_AHBLPENR_DMA2LPEN* = RCC_AHBLPENR_DMA2LPEN_Msk

## *****************  Bit definition for RCC_APB2LPENR register  **************

const
  RCC_APB2LPENR_SYSCFGLPEN_Pos* = (0)
  RCC_APB2LPENR_SYSCFGLPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_SYSCFGLPEN_Pos) ## !< 0x00000001
  RCC_APB2LPENR_SYSCFGLPEN* = RCC_APB2LPENR_SYSCFGLPEN_Msk
  RCC_APB2LPENR_TIM9LPEN_Pos* = (2)
  RCC_APB2LPENR_TIM9LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_TIM9LPEN_Pos) ## !< 0x00000004
  RCC_APB2LPENR_TIM9LPEN* = RCC_APB2LPENR_TIM9LPEN_Msk
  RCC_APB2LPENR_TIM10LPEN_Pos* = (3)
  RCC_APB2LPENR_TIM10LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_TIM10LPEN_Pos) ## !< 0x00000008
  RCC_APB2LPENR_TIM10LPEN* = RCC_APB2LPENR_TIM10LPEN_Msk
  RCC_APB2LPENR_TIM11LPEN_Pos* = (4)
  RCC_APB2LPENR_TIM11LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_TIM11LPEN_Pos) ## !< 0x00000010
  RCC_APB2LPENR_TIM11LPEN* = RCC_APB2LPENR_TIM11LPEN_Msk
  RCC_APB2LPENR_ADC1LPEN_Pos* = (9)
  RCC_APB2LPENR_ADC1LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_ADC1LPEN_Pos) ## !< 0x00000200
  RCC_APB2LPENR_ADC1LPEN* = RCC_APB2LPENR_ADC1LPEN_Msk
  RCC_APB2LPENR_SPI1LPEN_Pos* = (12)
  RCC_APB2LPENR_SPI1LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_SPI1LPEN_Pos) ## !< 0x00001000
  RCC_APB2LPENR_SPI1LPEN* = RCC_APB2LPENR_SPI1LPEN_Msk
  RCC_APB2LPENR_USART1LPEN_Pos* = (14)
  RCC_APB2LPENR_USART1LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_USART1LPEN_Pos) ## !< 0x00004000
  RCC_APB2LPENR_USART1LPEN* = RCC_APB2LPENR_USART1LPEN_Msk

## ****************  Bit definition for RCC_APB1LPENR register  ***************

const
  RCC_APB1LPENR_TIM2LPEN_Pos* = (0)
  RCC_APB1LPENR_TIM2LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_TIM2LPEN_Pos) ## !< 0x00000001
  RCC_APB1LPENR_TIM2LPEN* = RCC_APB1LPENR_TIM2LPEN_Msk
  RCC_APB1LPENR_TIM3LPEN_Pos* = (1)
  RCC_APB1LPENR_TIM3LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_TIM3LPEN_Pos) ## !< 0x00000002
  RCC_APB1LPENR_TIM3LPEN* = RCC_APB1LPENR_TIM3LPEN_Msk
  RCC_APB1LPENR_TIM4LPEN_Pos* = (2)
  RCC_APB1LPENR_TIM4LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_TIM4LPEN_Pos) ## !< 0x00000004
  RCC_APB1LPENR_TIM4LPEN* = RCC_APB1LPENR_TIM4LPEN_Msk
  RCC_APB1LPENR_TIM5LPEN_Pos* = (3)
  RCC_APB1LPENR_TIM5LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_TIM5LPEN_Pos) ## !< 0x00000008
  RCC_APB1LPENR_TIM5LPEN* = RCC_APB1LPENR_TIM5LPEN_Msk
  RCC_APB1LPENR_TIM6LPEN_Pos* = (4)
  RCC_APB1LPENR_TIM6LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_TIM6LPEN_Pos) ## !< 0x00000010
  RCC_APB1LPENR_TIM6LPEN* = RCC_APB1LPENR_TIM6LPEN_Msk
  RCC_APB1LPENR_TIM7LPEN_Pos* = (5)
  RCC_APB1LPENR_TIM7LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_TIM7LPEN_Pos) ## !< 0x00000020
  RCC_APB1LPENR_TIM7LPEN* = RCC_APB1LPENR_TIM7LPEN_Msk
  RCC_APB1LPENR_LCDLPEN_Pos* = (9)
  RCC_APB1LPENR_LCDLPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_LCDLPEN_Pos) ## !< 0x00000200
  RCC_APB1LPENR_LCDLPEN* = RCC_APB1LPENR_LCDLPEN_Msk
  RCC_APB1LPENR_WWDGLPEN_Pos* = (11)
  RCC_APB1LPENR_WWDGLPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_WWDGLPEN_Pos) ## !< 0x00000800
  RCC_APB1LPENR_WWDGLPEN* = RCC_APB1LPENR_WWDGLPEN_Msk
  RCC_APB1LPENR_SPI2LPEN_Pos* = (14)
  RCC_APB1LPENR_SPI2LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_SPI2LPEN_Pos) ## !< 0x00004000
  RCC_APB1LPENR_SPI2LPEN* = RCC_APB1LPENR_SPI2LPEN_Msk
  RCC_APB1LPENR_SPI3LPEN_Pos* = (15)
  RCC_APB1LPENR_SPI3LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_SPI3LPEN_Pos) ## !< 0x00008000
  RCC_APB1LPENR_SPI3LPEN* = RCC_APB1LPENR_SPI3LPEN_Msk
  RCC_APB1LPENR_USART2LPEN_Pos* = (17)
  RCC_APB1LPENR_USART2LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_USART2LPEN_Pos) ## !< 0x00020000
  RCC_APB1LPENR_USART2LPEN* = RCC_APB1LPENR_USART2LPEN_Msk
  RCC_APB1LPENR_USART3LPEN_Pos* = (18)
  RCC_APB1LPENR_USART3LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_USART3LPEN_Pos) ## !< 0x00040000
  RCC_APB1LPENR_USART3LPEN* = RCC_APB1LPENR_USART3LPEN_Msk
  RCC_APB1LPENR_UART4LPEN_Pos* = (19)
  RCC_APB1LPENR_UART4LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_UART4LPEN_Pos) ## !< 0x00080000
  RCC_APB1LPENR_UART4LPEN* = RCC_APB1LPENR_UART4LPEN_Msk
  RCC_APB1LPENR_UART5LPEN_Pos* = (20)
  RCC_APB1LPENR_UART5LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_UART5LPEN_Pos) ## !< 0x00100000
  RCC_APB1LPENR_UART5LPEN* = RCC_APB1LPENR_UART5LPEN_Msk
  RCC_APB1LPENR_I2C1LPEN_Pos* = (21)
  RCC_APB1LPENR_I2C1LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_I2C1LPEN_Pos) ## !< 0x00200000
  RCC_APB1LPENR_I2C1LPEN* = RCC_APB1LPENR_I2C1LPEN_Msk
  RCC_APB1LPENR_I2C2LPEN_Pos* = (22)
  RCC_APB1LPENR_I2C2LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_I2C2LPEN_Pos) ## !< 0x00400000
  RCC_APB1LPENR_I2C2LPEN* = RCC_APB1LPENR_I2C2LPEN_Msk
  RCC_APB1LPENR_USBLPEN_Pos* = (23)
  RCC_APB1LPENR_USBLPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_USBLPEN_Pos) ## !< 0x00800000
  RCC_APB1LPENR_USBLPEN* = RCC_APB1LPENR_USBLPEN_Msk
  RCC_APB1LPENR_PWRLPEN_Pos* = (28)
  RCC_APB1LPENR_PWRLPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_PWRLPEN_Pos) ## !< 0x10000000
  RCC_APB1LPENR_PWRLPEN* = RCC_APB1LPENR_PWRLPEN_Msk
  RCC_APB1LPENR_DACLPEN_Pos* = (29)
  RCC_APB1LPENR_DACLPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_DACLPEN_Pos) ## !< 0x20000000
  RCC_APB1LPENR_DACLPEN* = RCC_APB1LPENR_DACLPEN_Msk
  RCC_APB1LPENR_COMPLPEN_Pos* = (31)
  RCC_APB1LPENR_COMPLPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_COMPLPEN_Pos) ## !< 0x80000000
  RCC_APB1LPENR_COMPLPEN* = RCC_APB1LPENR_COMPLPEN_Msk

## ******************  Bit definition for RCC_CSR register  *******************

const
  RCC_CSR_LSION_Pos* = (0)
  RCC_CSR_LSION_Msk* = (0x00000001 shl RCC_CSR_LSION_Pos) ## !< 0x00000001
  RCC_CSR_LSION* = RCC_CSR_LSION_Msk
  RCC_CSR_LSIRDY_Pos* = (1)
  RCC_CSR_LSIRDY_Msk* = (0x00000001 shl RCC_CSR_LSIRDY_Pos) ## !< 0x00000002
  RCC_CSR_LSIRDY* = RCC_CSR_LSIRDY_Msk
  RCC_CSR_LSEON_Pos* = (8)
  RCC_CSR_LSEON_Msk* = (0x00000001 shl RCC_CSR_LSEON_Pos) ## !< 0x00000100
  RCC_CSR_LSEON* = RCC_CSR_LSEON_Msk
  RCC_CSR_LSERDY_Pos* = (9)
  RCC_CSR_LSERDY_Msk* = (0x00000001 shl RCC_CSR_LSERDY_Pos) ## !< 0x00000200
  RCC_CSR_LSERDY* = RCC_CSR_LSERDY_Msk
  RCC_CSR_LSEBYP_Pos* = (10)
  RCC_CSR_LSEBYP_Msk* = (0x00000001 shl RCC_CSR_LSEBYP_Pos) ## !< 0x00000400
  RCC_CSR_LSEBYP* = RCC_CSR_LSEBYP_Msk
  RCC_CSR_LSECSSON_Pos* = (11)
  RCC_CSR_LSECSSON_Msk* = (0x00000001 shl RCC_CSR_LSECSSON_Pos) ## !< 0x00000800
  RCC_CSR_LSECSSON* = RCC_CSR_LSECSSON_Msk
  RCC_CSR_LSECSSD_Pos* = (12)
  RCC_CSR_LSECSSD_Msk* = (0x00000001 shl RCC_CSR_LSECSSD_Pos) ## !< 0x00001000
  RCC_CSR_LSECSSD* = RCC_CSR_LSECSSD_Msk
  RCC_CSR_RTCSEL_Pos* = (16)
  RCC_CSR_RTCSEL_Msk* = (0x00000003 shl RCC_CSR_RTCSEL_Pos) ## !< 0x00030000
  RCC_CSR_RTCSEL* = RCC_CSR_RTCSEL_Msk
  RCC_CSR_RTCSEL_0* = (0x00000001 shl RCC_CSR_RTCSEL_Pos) ## !< 0x00010000
  RCC_CSR_RTCSEL_1* = (0x00000002 shl RCC_CSR_RTCSEL_Pos) ## !< 0x00020000

## !< RTC congiguration

const
  RCC_CSR_RTCSEL_NOCLOCK* = (0x00000000) ## !< No clock
  RCC_CSR_RTCSEL_LSE_Pos* = (16)
  RCC_CSR_RTCSEL_LSE_Msk* = (0x00000001 shl RCC_CSR_RTCSEL_LSE_Pos) ## !< 0x00010000
  RCC_CSR_RTCSEL_LSE* = RCC_CSR_RTCSEL_LSE_Msk
  RCC_CSR_RTCSEL_LSI_Pos* = (17)
  RCC_CSR_RTCSEL_LSI_Msk* = (0x00000001 shl RCC_CSR_RTCSEL_LSI_Pos) ## !< 0x00020000
  RCC_CSR_RTCSEL_LSI* = RCC_CSR_RTCSEL_LSI_Msk
  RCC_CSR_RTCSEL_HSE_Pos* = (16)
  RCC_CSR_RTCSEL_HSE_Msk* = (0x00000003 shl RCC_CSR_RTCSEL_HSE_Pos) ## !< 0x00030000
  RCC_CSR_RTCSEL_HSE* = RCC_CSR_RTCSEL_HSE_Msk
  RCC_CSR_RTCEN_Pos* = (22)
  RCC_CSR_RTCEN_Msk* = (0x00000001 shl RCC_CSR_RTCEN_Pos) ## !< 0x00400000
  RCC_CSR_RTCEN* = RCC_CSR_RTCEN_Msk
  RCC_CSR_RTCRST_Pos* = (23)
  RCC_CSR_RTCRST_Msk* = (0x00000001 shl RCC_CSR_RTCRST_Pos) ## !< 0x00800000
  RCC_CSR_RTCRST* = RCC_CSR_RTCRST_Msk
  RCC_CSR_RMVF_Pos* = (24)
  RCC_CSR_RMVF_Msk* = (0x00000001 shl RCC_CSR_RMVF_Pos) ## !< 0x01000000
  RCC_CSR_RMVF* = RCC_CSR_RMVF_Msk
  RCC_CSR_OBLRSTF_Pos* = (25)
  RCC_CSR_OBLRSTF_Msk* = (0x00000001 shl RCC_CSR_OBLRSTF_Pos) ## !< 0x02000000
  RCC_CSR_OBLRSTF* = RCC_CSR_OBLRSTF_Msk
  RCC_CSR_PINRSTF_Pos* = (26)
  RCC_CSR_PINRSTF_Msk* = (0x00000001 shl RCC_CSR_PINRSTF_Pos) ## !< 0x04000000
  RCC_CSR_PINRSTF* = RCC_CSR_PINRSTF_Msk
  RCC_CSR_PORRSTF_Pos* = (27)
  RCC_CSR_PORRSTF_Msk* = (0x00000001 shl RCC_CSR_PORRSTF_Pos) ## !< 0x08000000
  RCC_CSR_PORRSTF* = RCC_CSR_PORRSTF_Msk
  RCC_CSR_SFTRSTF_Pos* = (28)
  RCC_CSR_SFTRSTF_Msk* = (0x00000001 shl RCC_CSR_SFTRSTF_Pos) ## !< 0x10000000
  RCC_CSR_SFTRSTF* = RCC_CSR_SFTRSTF_Msk
  RCC_CSR_IWDGRSTF_Pos* = (29)
  RCC_CSR_IWDGRSTF_Msk* = (0x00000001 shl RCC_CSR_IWDGRSTF_Pos) ## !< 0x20000000
  RCC_CSR_IWDGRSTF* = RCC_CSR_IWDGRSTF_Msk
  RCC_CSR_WWDGRSTF_Pos* = (30)
  RCC_CSR_WWDGRSTF_Msk* = (0x00000001 shl RCC_CSR_WWDGRSTF_Pos) ## !< 0x40000000
  RCC_CSR_WWDGRSTF* = RCC_CSR_WWDGRSTF_Msk
  RCC_CSR_LPWRRSTF_Pos* = (31)
  RCC_CSR_LPWRRSTF_Msk* = (0x00000001 shl RCC_CSR_LPWRRSTF_Pos) ## !< 0x80000000
  RCC_CSR_LPWRRSTF* = RCC_CSR_LPWRRSTF_Msk

## ****************************************************************************
##
##                            Real-Time Clock (RTC)
##
## ****************************************************************************
##
##  @brief Specific device feature definitions  (not present on all devices in the STM32F0 serie)
##

const
  RTC_TAMPER1_SUPPORT* = true   ## !< TAMPER 1 feature support
  RTC_TAMPER2_SUPPORT* = true   ## !< TAMPER 2 feature support
  RTC_TAMPER3_SUPPORT* = true   ## !< TAMPER 3 feature support
  RTC_BACKUP_SUPPORT* = true    ## !< BACKUP register feature support
  RTC_WAKEUP_SUPPORT* = true    ## !< WAKEUP feature support
  RTC_SMOOTHCALIB_SUPPORT* = true ## !< Smooth digital calibration feature support
  RTC_SUBSECOND_SUPPORT* = true ## !< Sub-second feature support

## *******************  Bits definition for RTC_TR register  ******************

const
  RTC_TR_PM_Pos* = (22)
  RTC_TR_PM_Msk* = (0x00000001 shl RTC_TR_PM_Pos) ## !< 0x00400000
  RTC_TR_PM* = RTC_TR_PM_Msk
  RTC_TR_HT_Pos* = (20)
  RTC_TR_HT_Msk* = (0x00000003 shl RTC_TR_HT_Pos) ## !< 0x00300000
  RTC_TR_HT* = RTC_TR_HT_Msk
  RTC_TR_HT_0* = (0x00000001 shl RTC_TR_HT_Pos) ## !< 0x00100000
  RTC_TR_HT_1* = (0x00000002 shl RTC_TR_HT_Pos) ## !< 0x00200000
  RTC_TR_HU_Pos* = (16)
  RTC_TR_HU_Msk* = (0x0000000F shl RTC_TR_HU_Pos) ## !< 0x000F0000
  RTC_TR_HU* = RTC_TR_HU_Msk
  RTC_TR_HU_0* = (0x00000001 shl RTC_TR_HU_Pos) ## !< 0x00010000
  RTC_TR_HU_1* = (0x00000002 shl RTC_TR_HU_Pos) ## !< 0x00020000
  RTC_TR_HU_2* = (0x00000004 shl RTC_TR_HU_Pos) ## !< 0x00040000
  RTC_TR_HU_3* = (0x00000008 shl RTC_TR_HU_Pos) ## !< 0x00080000
  RTC_TR_MNT_Pos* = (12)
  RTC_TR_MNT_Msk* = (0x00000007 shl RTC_TR_MNT_Pos) ## !< 0x00007000
  RTC_TR_MNT* = RTC_TR_MNT_Msk
  RTC_TR_MNT_0* = (0x00000001 shl RTC_TR_MNT_Pos) ## !< 0x00001000
  RTC_TR_MNT_1* = (0x00000002 shl RTC_TR_MNT_Pos) ## !< 0x00002000
  RTC_TR_MNT_2* = (0x00000004 shl RTC_TR_MNT_Pos) ## !< 0x00004000
  RTC_TR_MNU_Pos* = (8)
  RTC_TR_MNU_Msk* = (0x0000000F shl RTC_TR_MNU_Pos) ## !< 0x00000F00
  RTC_TR_MNU* = RTC_TR_MNU_Msk
  RTC_TR_MNU_0* = (0x00000001 shl RTC_TR_MNU_Pos) ## !< 0x00000100
  RTC_TR_MNU_1* = (0x00000002 shl RTC_TR_MNU_Pos) ## !< 0x00000200
  RTC_TR_MNU_2* = (0x00000004 shl RTC_TR_MNU_Pos) ## !< 0x00000400
  RTC_TR_MNU_3* = (0x00000008 shl RTC_TR_MNU_Pos) ## !< 0x00000800
  RTC_TR_ST_Pos* = (4)
  RTC_TR_ST_Msk* = (0x00000007 shl RTC_TR_ST_Pos) ## !< 0x00000070
  RTC_TR_ST* = RTC_TR_ST_Msk
  RTC_TR_ST_0* = (0x00000001 shl RTC_TR_ST_Pos) ## !< 0x00000010
  RTC_TR_ST_1* = (0x00000002 shl RTC_TR_ST_Pos) ## !< 0x00000020
  RTC_TR_ST_2* = (0x00000004 shl RTC_TR_ST_Pos) ## !< 0x00000040
  RTC_TR_SU_Pos* = (0)
  RTC_TR_SU_Msk* = (0x0000000F shl RTC_TR_SU_Pos) ## !< 0x0000000F
  RTC_TR_SU* = RTC_TR_SU_Msk
  RTC_TR_SU_0* = (0x00000001 shl RTC_TR_SU_Pos) ## !< 0x00000001
  RTC_TR_SU_1* = (0x00000002 shl RTC_TR_SU_Pos) ## !< 0x00000002
  RTC_TR_SU_2* = (0x00000004 shl RTC_TR_SU_Pos) ## !< 0x00000004
  RTC_TR_SU_3* = (0x00000008 shl RTC_TR_SU_Pos) ## !< 0x00000008

## *******************  Bits definition for RTC_DR register  ******************

const
  RTC_DR_YT_Pos* = (20)
  RTC_DR_YT_Msk* = (0x0000000F shl RTC_DR_YT_Pos) ## !< 0x00F00000
  RTC_DR_YT* = RTC_DR_YT_Msk
  RTC_DR_YT_0* = (0x00000001 shl RTC_DR_YT_Pos) ## !< 0x00100000
  RTC_DR_YT_1* = (0x00000002 shl RTC_DR_YT_Pos) ## !< 0x00200000
  RTC_DR_YT_2* = (0x00000004 shl RTC_DR_YT_Pos) ## !< 0x00400000
  RTC_DR_YT_3* = (0x00000008 shl RTC_DR_YT_Pos) ## !< 0x00800000
  RTC_DR_YU_Pos* = (16)
  RTC_DR_YU_Msk* = (0x0000000F shl RTC_DR_YU_Pos) ## !< 0x000F0000
  RTC_DR_YU* = RTC_DR_YU_Msk
  RTC_DR_YU_0* = (0x00000001 shl RTC_DR_YU_Pos) ## !< 0x00010000
  RTC_DR_YU_1* = (0x00000002 shl RTC_DR_YU_Pos) ## !< 0x00020000
  RTC_DR_YU_2* = (0x00000004 shl RTC_DR_YU_Pos) ## !< 0x00040000
  RTC_DR_YU_3* = (0x00000008 shl RTC_DR_YU_Pos) ## !< 0x00080000
  RTC_DR_WDU_Pos* = (13)
  RTC_DR_WDU_Msk* = (0x00000007 shl RTC_DR_WDU_Pos) ## !< 0x0000E000
  RTC_DR_WDU* = RTC_DR_WDU_Msk
  RTC_DR_WDU_0* = (0x00000001 shl RTC_DR_WDU_Pos) ## !< 0x00002000
  RTC_DR_WDU_1* = (0x00000002 shl RTC_DR_WDU_Pos) ## !< 0x00004000
  RTC_DR_WDU_2* = (0x00000004 shl RTC_DR_WDU_Pos) ## !< 0x00008000
  RTC_DR_MT_Pos* = (12)
  RTC_DR_MT_Msk* = (0x00000001 shl RTC_DR_MT_Pos) ## !< 0x00001000
  RTC_DR_MT* = RTC_DR_MT_Msk
  RTC_DR_MU_Pos* = (8)
  RTC_DR_MU_Msk* = (0x0000000F shl RTC_DR_MU_Pos) ## !< 0x00000F00
  RTC_DR_MU* = RTC_DR_MU_Msk
  RTC_DR_MU_0* = (0x00000001 shl RTC_DR_MU_Pos) ## !< 0x00000100
  RTC_DR_MU_1* = (0x00000002 shl RTC_DR_MU_Pos) ## !< 0x00000200
  RTC_DR_MU_2* = (0x00000004 shl RTC_DR_MU_Pos) ## !< 0x00000400
  RTC_DR_MU_3* = (0x00000008 shl RTC_DR_MU_Pos) ## !< 0x00000800
  RTC_DR_DT_Pos* = (4)
  RTC_DR_DT_Msk* = (0x00000003 shl RTC_DR_DT_Pos) ## !< 0x00000030
  RTC_DR_DT* = RTC_DR_DT_Msk
  RTC_DR_DT_0* = (0x00000001 shl RTC_DR_DT_Pos) ## !< 0x00000010
  RTC_DR_DT_1* = (0x00000002 shl RTC_DR_DT_Pos) ## !< 0x00000020
  RTC_DR_DU_Pos* = (0)
  RTC_DR_DU_Msk* = (0x0000000F shl RTC_DR_DU_Pos) ## !< 0x0000000F
  RTC_DR_DU* = RTC_DR_DU_Msk
  RTC_DR_DU_0* = (0x00000001 shl RTC_DR_DU_Pos) ## !< 0x00000001
  RTC_DR_DU_1* = (0x00000002 shl RTC_DR_DU_Pos) ## !< 0x00000002
  RTC_DR_DU_2* = (0x00000004 shl RTC_DR_DU_Pos) ## !< 0x00000004
  RTC_DR_DU_3* = (0x00000008 shl RTC_DR_DU_Pos) ## !< 0x00000008

## *******************  Bits definition for RTC_CR register  ******************

const
  RTC_CR_COE_Pos* = (23)
  RTC_CR_COE_Msk* = (0x00000001 shl RTC_CR_COE_Pos) ## !< 0x00800000
  RTC_CR_COE* = RTC_CR_COE_Msk
  RTC_CR_OSEL_Pos* = (21)
  RTC_CR_OSEL_Msk* = (0x00000003 shl RTC_CR_OSEL_Pos) ## !< 0x00600000
  RTC_CR_OSEL* = RTC_CR_OSEL_Msk
  RTC_CR_OSEL_0* = (0x00000001 shl RTC_CR_OSEL_Pos) ## !< 0x00200000
  RTC_CR_OSEL_1* = (0x00000002 shl RTC_CR_OSEL_Pos) ## !< 0x00400000
  RTC_CR_POL_Pos* = (20)
  RTC_CR_POL_Msk* = (0x00000001 shl RTC_CR_POL_Pos) ## !< 0x00100000
  RTC_CR_POL* = RTC_CR_POL_Msk
  RTC_CR_COSEL_Pos* = (19)
  RTC_CR_COSEL_Msk* = (0x00000001 shl RTC_CR_COSEL_Pos) ## !< 0x00080000
  RTC_CR_COSEL* = RTC_CR_COSEL_Msk
  RTC_CR_BKP_Pos* = (18)
  RTC_CR_BKP_Msk* = (0x00000001 shl RTC_CR_BKP_Pos) ## !< 0x00040000
  RTC_CR_BKP* = RTC_CR_BKP_Msk
  RTC_CR_SUB1H_Pos* = (17)
  RTC_CR_SUB1H_Msk* = (0x00000001 shl RTC_CR_SUB1H_Pos) ## !< 0x00020000
  RTC_CR_SUB1H* = RTC_CR_SUB1H_Msk
  RTC_CR_ADD1H_Pos* = (16)
  RTC_CR_ADD1H_Msk* = (0x00000001 shl RTC_CR_ADD1H_Pos) ## !< 0x00010000
  RTC_CR_ADD1H* = RTC_CR_ADD1H_Msk
  RTC_CR_TSIE_Pos* = (15)
  RTC_CR_TSIE_Msk* = (0x00000001 shl RTC_CR_TSIE_Pos) ## !< 0x00008000
  RTC_CR_TSIE* = RTC_CR_TSIE_Msk
  RTC_CR_WUTIE_Pos* = (14)
  RTC_CR_WUTIE_Msk* = (0x00000001 shl RTC_CR_WUTIE_Pos) ## !< 0x00004000
  RTC_CR_WUTIE* = RTC_CR_WUTIE_Msk
  RTC_CR_ALRBIE_Pos* = (13)
  RTC_CR_ALRBIE_Msk* = (0x00000001 shl RTC_CR_ALRBIE_Pos) ## !< 0x00002000
  RTC_CR_ALRBIE* = RTC_CR_ALRBIE_Msk
  RTC_CR_ALRAIE_Pos* = (12)
  RTC_CR_ALRAIE_Msk* = (0x00000001 shl RTC_CR_ALRAIE_Pos) ## !< 0x00001000
  RTC_CR_ALRAIE* = RTC_CR_ALRAIE_Msk
  RTC_CR_TSE_Pos* = (11)
  RTC_CR_TSE_Msk* = (0x00000001 shl RTC_CR_TSE_Pos) ## !< 0x00000800
  RTC_CR_TSE* = RTC_CR_TSE_Msk
  RTC_CR_WUTE_Pos* = (10)
  RTC_CR_WUTE_Msk* = (0x00000001 shl RTC_CR_WUTE_Pos) ## !< 0x00000400
  RTC_CR_WUTE* = RTC_CR_WUTE_Msk
  RTC_CR_ALRBE_Pos* = (9)
  RTC_CR_ALRBE_Msk* = (0x00000001 shl RTC_CR_ALRBE_Pos) ## !< 0x00000200
  RTC_CR_ALRBE* = RTC_CR_ALRBE_Msk
  RTC_CR_ALRAE_Pos* = (8)
  RTC_CR_ALRAE_Msk* = (0x00000001 shl RTC_CR_ALRAE_Pos) ## !< 0x00000100
  RTC_CR_ALRAE* = RTC_CR_ALRAE_Msk
  RTC_CR_DCE_Pos* = (7)
  RTC_CR_DCE_Msk* = (0x00000001 shl RTC_CR_DCE_Pos) ## !< 0x00000080
  RTC_CR_DCE* = RTC_CR_DCE_Msk
  RTC_CR_FMT_Pos* = (6)
  RTC_CR_FMT_Msk* = (0x00000001 shl RTC_CR_FMT_Pos) ## !< 0x00000040
  RTC_CR_FMT* = RTC_CR_FMT_Msk
  RTC_CR_BYPSHAD_Pos* = (5)
  RTC_CR_BYPSHAD_Msk* = (0x00000001 shl RTC_CR_BYPSHAD_Pos) ## !< 0x00000020
  RTC_CR_BYPSHAD* = RTC_CR_BYPSHAD_Msk
  RTC_CR_REFCKON_Pos* = (4)
  RTC_CR_REFCKON_Msk* = (0x00000001 shl RTC_CR_REFCKON_Pos) ## !< 0x00000010
  RTC_CR_REFCKON* = RTC_CR_REFCKON_Msk
  RTC_CR_TSEDGE_Pos* = (3)
  RTC_CR_TSEDGE_Msk* = (0x00000001 shl RTC_CR_TSEDGE_Pos) ## !< 0x00000008
  RTC_CR_TSEDGE* = RTC_CR_TSEDGE_Msk
  RTC_CR_WUCKSEL_Pos* = (0)
  RTC_CR_WUCKSEL_Msk* = (0x00000007 shl RTC_CR_WUCKSEL_Pos) ## !< 0x00000007
  RTC_CR_WUCKSEL* = RTC_CR_WUCKSEL_Msk
  RTC_CR_WUCKSEL_0* = (0x00000001 shl RTC_CR_WUCKSEL_Pos) ## !< 0x00000001
  RTC_CR_WUCKSEL_1* = (0x00000002 shl RTC_CR_WUCKSEL_Pos) ## !< 0x00000002
  RTC_CR_WUCKSEL_2* = (0x00000004 shl RTC_CR_WUCKSEL_Pos) ## !< 0x00000004

##  Legacy defines

const
  RTC_CR_BCK_Pos* = RTC_CR_BKP_Pos
  RTC_CR_BCK_Msk* = RTC_CR_BKP_Msk
  RTC_CR_BCK* = RTC_CR_BKP

## *******************  Bits definition for RTC_ISR register  *****************

const
  RTC_ISR_RECALPF_Pos* = (16)
  RTC_ISR_RECALPF_Msk* = (0x00000001 shl RTC_ISR_RECALPF_Pos) ## !< 0x00010000
  RTC_ISR_RECALPF* = RTC_ISR_RECALPF_Msk
  RTC_ISR_TAMP3F_Pos* = (15)
  RTC_ISR_TAMP3F_Msk* = (0x00000001 shl RTC_ISR_TAMP3F_Pos) ## !< 0x00008000
  RTC_ISR_TAMP3F* = RTC_ISR_TAMP3F_Msk
  RTC_ISR_TAMP2F_Pos* = (14)
  RTC_ISR_TAMP2F_Msk* = (0x00000001 shl RTC_ISR_TAMP2F_Pos) ## !< 0x00004000
  RTC_ISR_TAMP2F* = RTC_ISR_TAMP2F_Msk
  RTC_ISR_TAMP1F_Pos* = (13)
  RTC_ISR_TAMP1F_Msk* = (0x00000001 shl RTC_ISR_TAMP1F_Pos) ## !< 0x00002000
  RTC_ISR_TAMP1F* = RTC_ISR_TAMP1F_Msk
  RTC_ISR_TSOVF_Pos* = (12)
  RTC_ISR_TSOVF_Msk* = (0x00000001 shl RTC_ISR_TSOVF_Pos) ## !< 0x00001000
  RTC_ISR_TSOVF* = RTC_ISR_TSOVF_Msk
  RTC_ISR_TSF_Pos* = (11)
  RTC_ISR_TSF_Msk* = (0x00000001 shl RTC_ISR_TSF_Pos) ## !< 0x00000800
  RTC_ISR_TSF* = RTC_ISR_TSF_Msk
  RTC_ISR_WUTF_Pos* = (10)
  RTC_ISR_WUTF_Msk* = (0x00000001 shl RTC_ISR_WUTF_Pos) ## !< 0x00000400
  RTC_ISR_WUTF* = RTC_ISR_WUTF_Msk
  RTC_ISR_ALRBF_Pos* = (9)
  RTC_ISR_ALRBF_Msk* = (0x00000001 shl RTC_ISR_ALRBF_Pos) ## !< 0x00000200
  RTC_ISR_ALRBF* = RTC_ISR_ALRBF_Msk
  RTC_ISR_ALRAF_Pos* = (8)
  RTC_ISR_ALRAF_Msk* = (0x00000001 shl RTC_ISR_ALRAF_Pos) ## !< 0x00000100
  RTC_ISR_ALRAF* = RTC_ISR_ALRAF_Msk
  RTC_ISR_INIT_Pos* = (7)
  RTC_ISR_INIT_Msk* = (0x00000001 shl RTC_ISR_INIT_Pos) ## !< 0x00000080
  RTC_ISR_INIT* = RTC_ISR_INIT_Msk
  RTC_ISR_INITF_Pos* = (6)
  RTC_ISR_INITF_Msk* = (0x00000001 shl RTC_ISR_INITF_Pos) ## !< 0x00000040
  RTC_ISR_INITF* = RTC_ISR_INITF_Msk
  RTC_ISR_RSF_Pos* = (5)
  RTC_ISR_RSF_Msk* = (0x00000001 shl RTC_ISR_RSF_Pos) ## !< 0x00000020
  RTC_ISR_RSF* = RTC_ISR_RSF_Msk
  RTC_ISR_INITS_Pos* = (4)
  RTC_ISR_INITS_Msk* = (0x00000001 shl RTC_ISR_INITS_Pos) ## !< 0x00000010
  RTC_ISR_INITS* = RTC_ISR_INITS_Msk
  RTC_ISR_SHPF_Pos* = (3)
  RTC_ISR_SHPF_Msk* = (0x00000001 shl RTC_ISR_SHPF_Pos) ## !< 0x00000008
  RTC_ISR_SHPF* = RTC_ISR_SHPF_Msk
  RTC_ISR_WUTWF_Pos* = (2)
  RTC_ISR_WUTWF_Msk* = (0x00000001 shl RTC_ISR_WUTWF_Pos) ## !< 0x00000004
  RTC_ISR_WUTWF* = RTC_ISR_WUTWF_Msk
  RTC_ISR_ALRBWF_Pos* = (1)
  RTC_ISR_ALRBWF_Msk* = (0x00000001 shl RTC_ISR_ALRBWF_Pos) ## !< 0x00000002
  RTC_ISR_ALRBWF* = RTC_ISR_ALRBWF_Msk
  RTC_ISR_ALRAWF_Pos* = (0)
  RTC_ISR_ALRAWF_Msk* = (0x00000001 shl RTC_ISR_ALRAWF_Pos) ## !< 0x00000001
  RTC_ISR_ALRAWF* = RTC_ISR_ALRAWF_Msk

## *******************  Bits definition for RTC_PRER register  ****************

const
  RTC_PRER_PREDIV_A_Pos* = (16)
  RTC_PRER_PREDIV_A_Msk* = (0x0000007F shl RTC_PRER_PREDIV_A_Pos) ## !< 0x007F0000
  RTC_PRER_PREDIV_A* = RTC_PRER_PREDIV_A_Msk
  RTC_PRER_PREDIV_S_Pos* = (0)
  RTC_PRER_PREDIV_S_Msk* = (0x00007FFF shl RTC_PRER_PREDIV_S_Pos) ## !< 0x00007FFF
  RTC_PRER_PREDIV_S* = RTC_PRER_PREDIV_S_Msk

## *******************  Bits definition for RTC_WUTR register  ****************

const
  RTC_WUTR_WUT_Pos* = (0)
  RTC_WUTR_WUT_Msk* = (0x0000FFFF shl RTC_WUTR_WUT_Pos) ## !< 0x0000FFFF
  RTC_WUTR_WUT* = RTC_WUTR_WUT_Msk

## *******************  Bits definition for RTC_CALIBR register  **************

const
  RTC_CALIBR_DCS_Pos* = (7)
  RTC_CALIBR_DCS_Msk* = (0x00000001 shl RTC_CALIBR_DCS_Pos) ## !< 0x00000080
  RTC_CALIBR_DCS* = RTC_CALIBR_DCS_Msk
  RTC_CALIBR_DC_Pos* = (0)
  RTC_CALIBR_DC_Msk* = (0x0000001F shl RTC_CALIBR_DC_Pos) ## !< 0x0000001F
  RTC_CALIBR_DC* = RTC_CALIBR_DC_Msk

## *******************  Bits definition for RTC_ALRMAR register  **************

const
  RTC_ALRMAR_MSK4_Pos* = (31)
  RTC_ALRMAR_MSK4_Msk* = (0x00000001 shl RTC_ALRMAR_MSK4_Pos) ## !< 0x80000000
  RTC_ALRMAR_MSK4* = RTC_ALRMAR_MSK4_Msk
  RTC_ALRMAR_WDSEL_Pos* = (30)
  RTC_ALRMAR_WDSEL_Msk* = (0x00000001 shl RTC_ALRMAR_WDSEL_Pos) ## !< 0x40000000
  RTC_ALRMAR_WDSEL* = RTC_ALRMAR_WDSEL_Msk
  RTC_ALRMAR_DT_Pos* = (28)
  RTC_ALRMAR_DT_Msk* = (0x00000003 shl RTC_ALRMAR_DT_Pos) ## !< 0x30000000
  RTC_ALRMAR_DT* = RTC_ALRMAR_DT_Msk
  RTC_ALRMAR_DT_0* = (0x00000001 shl RTC_ALRMAR_DT_Pos) ## !< 0x10000000
  RTC_ALRMAR_DT_1* = (0x00000002 shl RTC_ALRMAR_DT_Pos) ## !< 0x20000000
  RTC_ALRMAR_DU_Pos* = (24)
  RTC_ALRMAR_DU_Msk* = (0x0000000F shl RTC_ALRMAR_DU_Pos) ## !< 0x0F000000
  RTC_ALRMAR_DU* = RTC_ALRMAR_DU_Msk
  RTC_ALRMAR_DU_0* = (0x00000001 shl RTC_ALRMAR_DU_Pos) ## !< 0x01000000
  RTC_ALRMAR_DU_1* = (0x00000002 shl RTC_ALRMAR_DU_Pos) ## !< 0x02000000
  RTC_ALRMAR_DU_2* = (0x00000004 shl RTC_ALRMAR_DU_Pos) ## !< 0x04000000
  RTC_ALRMAR_DU_3* = (0x00000008 shl RTC_ALRMAR_DU_Pos) ## !< 0x08000000
  RTC_ALRMAR_MSK3_Pos* = (23)
  RTC_ALRMAR_MSK3_Msk* = (0x00000001 shl RTC_ALRMAR_MSK3_Pos) ## !< 0x00800000
  RTC_ALRMAR_MSK3* = RTC_ALRMAR_MSK3_Msk
  RTC_ALRMAR_PM_Pos* = (22)
  RTC_ALRMAR_PM_Msk* = (0x00000001 shl RTC_ALRMAR_PM_Pos) ## !< 0x00400000
  RTC_ALRMAR_PM* = RTC_ALRMAR_PM_Msk
  RTC_ALRMAR_HT_Pos* = (20)
  RTC_ALRMAR_HT_Msk* = (0x00000003 shl RTC_ALRMAR_HT_Pos) ## !< 0x00300000
  RTC_ALRMAR_HT* = RTC_ALRMAR_HT_Msk
  RTC_ALRMAR_HT_0* = (0x00000001 shl RTC_ALRMAR_HT_Pos) ## !< 0x00100000
  RTC_ALRMAR_HT_1* = (0x00000002 shl RTC_ALRMAR_HT_Pos) ## !< 0x00200000
  RTC_ALRMAR_HU_Pos* = (16)
  RTC_ALRMAR_HU_Msk* = (0x0000000F shl RTC_ALRMAR_HU_Pos) ## !< 0x000F0000
  RTC_ALRMAR_HU* = RTC_ALRMAR_HU_Msk
  RTC_ALRMAR_HU_0* = (0x00000001 shl RTC_ALRMAR_HU_Pos) ## !< 0x00010000
  RTC_ALRMAR_HU_1* = (0x00000002 shl RTC_ALRMAR_HU_Pos) ## !< 0x00020000
  RTC_ALRMAR_HU_2* = (0x00000004 shl RTC_ALRMAR_HU_Pos) ## !< 0x00040000
  RTC_ALRMAR_HU_3* = (0x00000008 shl RTC_ALRMAR_HU_Pos) ## !< 0x00080000
  RTC_ALRMAR_MSK2_Pos* = (15)
  RTC_ALRMAR_MSK2_Msk* = (0x00000001 shl RTC_ALRMAR_MSK2_Pos) ## !< 0x00008000
  RTC_ALRMAR_MSK2* = RTC_ALRMAR_MSK2_Msk
  RTC_ALRMAR_MNT_Pos* = (12)
  RTC_ALRMAR_MNT_Msk* = (0x00000007 shl RTC_ALRMAR_MNT_Pos) ## !< 0x00007000
  RTC_ALRMAR_MNT* = RTC_ALRMAR_MNT_Msk
  RTC_ALRMAR_MNT_0* = (0x00000001 shl RTC_ALRMAR_MNT_Pos) ## !< 0x00001000
  RTC_ALRMAR_MNT_1* = (0x00000002 shl RTC_ALRMAR_MNT_Pos) ## !< 0x00002000
  RTC_ALRMAR_MNT_2* = (0x00000004 shl RTC_ALRMAR_MNT_Pos) ## !< 0x00004000
  RTC_ALRMAR_MNU_Pos* = (8)
  RTC_ALRMAR_MNU_Msk* = (0x0000000F shl RTC_ALRMAR_MNU_Pos) ## !< 0x00000F00
  RTC_ALRMAR_MNU* = RTC_ALRMAR_MNU_Msk
  RTC_ALRMAR_MNU_0* = (0x00000001 shl RTC_ALRMAR_MNU_Pos) ## !< 0x00000100
  RTC_ALRMAR_MNU_1* = (0x00000002 shl RTC_ALRMAR_MNU_Pos) ## !< 0x00000200
  RTC_ALRMAR_MNU_2* = (0x00000004 shl RTC_ALRMAR_MNU_Pos) ## !< 0x00000400
  RTC_ALRMAR_MNU_3* = (0x00000008 shl RTC_ALRMAR_MNU_Pos) ## !< 0x00000800
  RTC_ALRMAR_MSK1_Pos* = (7)
  RTC_ALRMAR_MSK1_Msk* = (0x00000001 shl RTC_ALRMAR_MSK1_Pos) ## !< 0x00000080
  RTC_ALRMAR_MSK1* = RTC_ALRMAR_MSK1_Msk
  RTC_ALRMAR_ST_Pos* = (4)
  RTC_ALRMAR_ST_Msk* = (0x00000007 shl RTC_ALRMAR_ST_Pos) ## !< 0x00000070
  RTC_ALRMAR_ST* = RTC_ALRMAR_ST_Msk
  RTC_ALRMAR_ST_0* = (0x00000001 shl RTC_ALRMAR_ST_Pos) ## !< 0x00000010
  RTC_ALRMAR_ST_1* = (0x00000002 shl RTC_ALRMAR_ST_Pos) ## !< 0x00000020
  RTC_ALRMAR_ST_2* = (0x00000004 shl RTC_ALRMAR_ST_Pos) ## !< 0x00000040
  RTC_ALRMAR_SU_Pos* = (0)
  RTC_ALRMAR_SU_Msk* = (0x0000000F shl RTC_ALRMAR_SU_Pos) ## !< 0x0000000F
  RTC_ALRMAR_SU* = RTC_ALRMAR_SU_Msk
  RTC_ALRMAR_SU_0* = (0x00000001 shl RTC_ALRMAR_SU_Pos) ## !< 0x00000001
  RTC_ALRMAR_SU_1* = (0x00000002 shl RTC_ALRMAR_SU_Pos) ## !< 0x00000002
  RTC_ALRMAR_SU_2* = (0x00000004 shl RTC_ALRMAR_SU_Pos) ## !< 0x00000004
  RTC_ALRMAR_SU_3* = (0x00000008 shl RTC_ALRMAR_SU_Pos) ## !< 0x00000008

## *******************  Bits definition for RTC_ALRMBR register  **************

const
  RTC_ALRMBR_MSK4_Pos* = (31)
  RTC_ALRMBR_MSK4_Msk* = (0x00000001 shl RTC_ALRMBR_MSK4_Pos) ## !< 0x80000000
  RTC_ALRMBR_MSK4* = RTC_ALRMBR_MSK4_Msk
  RTC_ALRMBR_WDSEL_Pos* = (30)
  RTC_ALRMBR_WDSEL_Msk* = (0x00000001 shl RTC_ALRMBR_WDSEL_Pos) ## !< 0x40000000
  RTC_ALRMBR_WDSEL* = RTC_ALRMBR_WDSEL_Msk
  RTC_ALRMBR_DT_Pos* = (28)
  RTC_ALRMBR_DT_Msk* = (0x00000003 shl RTC_ALRMBR_DT_Pos) ## !< 0x30000000
  RTC_ALRMBR_DT* = RTC_ALRMBR_DT_Msk
  RTC_ALRMBR_DT_0* = (0x00000001 shl RTC_ALRMBR_DT_Pos) ## !< 0x10000000
  RTC_ALRMBR_DT_1* = (0x00000002 shl RTC_ALRMBR_DT_Pos) ## !< 0x20000000
  RTC_ALRMBR_DU_Pos* = (24)
  RTC_ALRMBR_DU_Msk* = (0x0000000F shl RTC_ALRMBR_DU_Pos) ## !< 0x0F000000
  RTC_ALRMBR_DU* = RTC_ALRMBR_DU_Msk
  RTC_ALRMBR_DU_0* = (0x00000001 shl RTC_ALRMBR_DU_Pos) ## !< 0x01000000
  RTC_ALRMBR_DU_1* = (0x00000002 shl RTC_ALRMBR_DU_Pos) ## !< 0x02000000
  RTC_ALRMBR_DU_2* = (0x00000004 shl RTC_ALRMBR_DU_Pos) ## !< 0x04000000
  RTC_ALRMBR_DU_3* = (0x00000008 shl RTC_ALRMBR_DU_Pos) ## !< 0x08000000
  RTC_ALRMBR_MSK3_Pos* = (23)
  RTC_ALRMBR_MSK3_Msk* = (0x00000001 shl RTC_ALRMBR_MSK3_Pos) ## !< 0x00800000
  RTC_ALRMBR_MSK3* = RTC_ALRMBR_MSK3_Msk
  RTC_ALRMBR_PM_Pos* = (22)
  RTC_ALRMBR_PM_Msk* = (0x00000001 shl RTC_ALRMBR_PM_Pos) ## !< 0x00400000
  RTC_ALRMBR_PM* = RTC_ALRMBR_PM_Msk
  RTC_ALRMBR_HT_Pos* = (20)
  RTC_ALRMBR_HT_Msk* = (0x00000003 shl RTC_ALRMBR_HT_Pos) ## !< 0x00300000
  RTC_ALRMBR_HT* = RTC_ALRMBR_HT_Msk
  RTC_ALRMBR_HT_0* = (0x00000001 shl RTC_ALRMBR_HT_Pos) ## !< 0x00100000
  RTC_ALRMBR_HT_1* = (0x00000002 shl RTC_ALRMBR_HT_Pos) ## !< 0x00200000
  RTC_ALRMBR_HU_Pos* = (16)
  RTC_ALRMBR_HU_Msk* = (0x0000000F shl RTC_ALRMBR_HU_Pos) ## !< 0x000F0000
  RTC_ALRMBR_HU* = RTC_ALRMBR_HU_Msk
  RTC_ALRMBR_HU_0* = (0x00000001 shl RTC_ALRMBR_HU_Pos) ## !< 0x00010000
  RTC_ALRMBR_HU_1* = (0x00000002 shl RTC_ALRMBR_HU_Pos) ## !< 0x00020000
  RTC_ALRMBR_HU_2* = (0x00000004 shl RTC_ALRMBR_HU_Pos) ## !< 0x00040000
  RTC_ALRMBR_HU_3* = (0x00000008 shl RTC_ALRMBR_HU_Pos) ## !< 0x00080000
  RTC_ALRMBR_MSK2_Pos* = (15)
  RTC_ALRMBR_MSK2_Msk* = (0x00000001 shl RTC_ALRMBR_MSK2_Pos) ## !< 0x00008000
  RTC_ALRMBR_MSK2* = RTC_ALRMBR_MSK2_Msk
  RTC_ALRMBR_MNT_Pos* = (12)
  RTC_ALRMBR_MNT_Msk* = (0x00000007 shl RTC_ALRMBR_MNT_Pos) ## !< 0x00007000
  RTC_ALRMBR_MNT* = RTC_ALRMBR_MNT_Msk
  RTC_ALRMBR_MNT_0* = (0x00000001 shl RTC_ALRMBR_MNT_Pos) ## !< 0x00001000
  RTC_ALRMBR_MNT_1* = (0x00000002 shl RTC_ALRMBR_MNT_Pos) ## !< 0x00002000
  RTC_ALRMBR_MNT_2* = (0x00000004 shl RTC_ALRMBR_MNT_Pos) ## !< 0x00004000
  RTC_ALRMBR_MNU_Pos* = (8)
  RTC_ALRMBR_MNU_Msk* = (0x0000000F shl RTC_ALRMBR_MNU_Pos) ## !< 0x00000F00
  RTC_ALRMBR_MNU* = RTC_ALRMBR_MNU_Msk
  RTC_ALRMBR_MNU_0* = (0x00000001 shl RTC_ALRMBR_MNU_Pos) ## !< 0x00000100
  RTC_ALRMBR_MNU_1* = (0x00000002 shl RTC_ALRMBR_MNU_Pos) ## !< 0x00000200
  RTC_ALRMBR_MNU_2* = (0x00000004 shl RTC_ALRMBR_MNU_Pos) ## !< 0x00000400
  RTC_ALRMBR_MNU_3* = (0x00000008 shl RTC_ALRMBR_MNU_Pos) ## !< 0x00000800
  RTC_ALRMBR_MSK1_Pos* = (7)
  RTC_ALRMBR_MSK1_Msk* = (0x00000001 shl RTC_ALRMBR_MSK1_Pos) ## !< 0x00000080
  RTC_ALRMBR_MSK1* = RTC_ALRMBR_MSK1_Msk
  RTC_ALRMBR_ST_Pos* = (4)
  RTC_ALRMBR_ST_Msk* = (0x00000007 shl RTC_ALRMBR_ST_Pos) ## !< 0x00000070
  RTC_ALRMBR_ST* = RTC_ALRMBR_ST_Msk
  RTC_ALRMBR_ST_0* = (0x00000001 shl RTC_ALRMBR_ST_Pos) ## !< 0x00000010
  RTC_ALRMBR_ST_1* = (0x00000002 shl RTC_ALRMBR_ST_Pos) ## !< 0x00000020
  RTC_ALRMBR_ST_2* = (0x00000004 shl RTC_ALRMBR_ST_Pos) ## !< 0x00000040
  RTC_ALRMBR_SU_Pos* = (0)
  RTC_ALRMBR_SU_Msk* = (0x0000000F shl RTC_ALRMBR_SU_Pos) ## !< 0x0000000F
  RTC_ALRMBR_SU* = RTC_ALRMBR_SU_Msk
  RTC_ALRMBR_SU_0* = (0x00000001 shl RTC_ALRMBR_SU_Pos) ## !< 0x00000001
  RTC_ALRMBR_SU_1* = (0x00000002 shl RTC_ALRMBR_SU_Pos) ## !< 0x00000002
  RTC_ALRMBR_SU_2* = (0x00000004 shl RTC_ALRMBR_SU_Pos) ## !< 0x00000004
  RTC_ALRMBR_SU_3* = (0x00000008 shl RTC_ALRMBR_SU_Pos) ## !< 0x00000008

## *******************  Bits definition for RTC_WPR register  *****************

const
  RTC_WPR_KEY_Pos* = (0)
  RTC_WPR_KEY_Msk* = (0x000000FF shl RTC_WPR_KEY_Pos) ## !< 0x000000FF
  RTC_WPR_KEY* = RTC_WPR_KEY_Msk

## *******************  Bits definition for RTC_SSR register  *****************

const
  RTC_SSR_SS_Pos* = (0)
  RTC_SSR_SS_Msk* = (0x0000FFFF shl RTC_SSR_SS_Pos) ## !< 0x0000FFFF
  RTC_SSR_SS* = RTC_SSR_SS_Msk

## *******************  Bits definition for RTC_SHIFTR register  **************

const
  RTC_SHIFTR_SUBFS_Pos* = (0)
  RTC_SHIFTR_SUBFS_Msk* = (0x00007FFF shl RTC_SHIFTR_SUBFS_Pos) ## !< 0x00007FFF
  RTC_SHIFTR_SUBFS* = RTC_SHIFTR_SUBFS_Msk
  RTC_SHIFTR_ADD1S_Pos* = (31)
  RTC_SHIFTR_ADD1S_Msk* = (0x00000001 shl RTC_SHIFTR_ADD1S_Pos) ## !< 0x80000000
  RTC_SHIFTR_ADD1S* = RTC_SHIFTR_ADD1S_Msk

## *******************  Bits definition for RTC_TSTR register  ****************

const
  RTC_TSTR_PM_Pos* = (22)
  RTC_TSTR_PM_Msk* = (0x00000001 shl RTC_TSTR_PM_Pos) ## !< 0x00400000
  RTC_TSTR_PM* = RTC_TSTR_PM_Msk
  RTC_TSTR_HT_Pos* = (20)
  RTC_TSTR_HT_Msk* = (0x00000003 shl RTC_TSTR_HT_Pos) ## !< 0x00300000
  RTC_TSTR_HT* = RTC_TSTR_HT_Msk
  RTC_TSTR_HT_0* = (0x00000001 shl RTC_TSTR_HT_Pos) ## !< 0x00100000
  RTC_TSTR_HT_1* = (0x00000002 shl RTC_TSTR_HT_Pos) ## !< 0x00200000
  RTC_TSTR_HU_Pos* = (16)
  RTC_TSTR_HU_Msk* = (0x0000000F shl RTC_TSTR_HU_Pos) ## !< 0x000F0000
  RTC_TSTR_HU* = RTC_TSTR_HU_Msk
  RTC_TSTR_HU_0* = (0x00000001 shl RTC_TSTR_HU_Pos) ## !< 0x00010000
  RTC_TSTR_HU_1* = (0x00000002 shl RTC_TSTR_HU_Pos) ## !< 0x00020000
  RTC_TSTR_HU_2* = (0x00000004 shl RTC_TSTR_HU_Pos) ## !< 0x00040000
  RTC_TSTR_HU_3* = (0x00000008 shl RTC_TSTR_HU_Pos) ## !< 0x00080000
  RTC_TSTR_MNT_Pos* = (12)
  RTC_TSTR_MNT_Msk* = (0x00000007 shl RTC_TSTR_MNT_Pos) ## !< 0x00007000
  RTC_TSTR_MNT* = RTC_TSTR_MNT_Msk
  RTC_TSTR_MNT_0* = (0x00000001 shl RTC_TSTR_MNT_Pos) ## !< 0x00001000
  RTC_TSTR_MNT_1* = (0x00000002 shl RTC_TSTR_MNT_Pos) ## !< 0x00002000
  RTC_TSTR_MNT_2* = (0x00000004 shl RTC_TSTR_MNT_Pos) ## !< 0x00004000
  RTC_TSTR_MNU_Pos* = (8)
  RTC_TSTR_MNU_Msk* = (0x0000000F shl RTC_TSTR_MNU_Pos) ## !< 0x00000F00
  RTC_TSTR_MNU* = RTC_TSTR_MNU_Msk
  RTC_TSTR_MNU_0* = (0x00000001 shl RTC_TSTR_MNU_Pos) ## !< 0x00000100
  RTC_TSTR_MNU_1* = (0x00000002 shl RTC_TSTR_MNU_Pos) ## !< 0x00000200
  RTC_TSTR_MNU_2* = (0x00000004 shl RTC_TSTR_MNU_Pos) ## !< 0x00000400
  RTC_TSTR_MNU_3* = (0x00000008 shl RTC_TSTR_MNU_Pos) ## !< 0x00000800
  RTC_TSTR_ST_Pos* = (4)
  RTC_TSTR_ST_Msk* = (0x00000007 shl RTC_TSTR_ST_Pos) ## !< 0x00000070
  RTC_TSTR_ST* = RTC_TSTR_ST_Msk
  RTC_TSTR_ST_0* = (0x00000001 shl RTC_TSTR_ST_Pos) ## !< 0x00000010
  RTC_TSTR_ST_1* = (0x00000002 shl RTC_TSTR_ST_Pos) ## !< 0x00000020
  RTC_TSTR_ST_2* = (0x00000004 shl RTC_TSTR_ST_Pos) ## !< 0x00000040
  RTC_TSTR_SU_Pos* = (0)
  RTC_TSTR_SU_Msk* = (0x0000000F shl RTC_TSTR_SU_Pos) ## !< 0x0000000F
  RTC_TSTR_SU* = RTC_TSTR_SU_Msk
  RTC_TSTR_SU_0* = (0x00000001 shl RTC_TSTR_SU_Pos) ## !< 0x00000001
  RTC_TSTR_SU_1* = (0x00000002 shl RTC_TSTR_SU_Pos) ## !< 0x00000002
  RTC_TSTR_SU_2* = (0x00000004 shl RTC_TSTR_SU_Pos) ## !< 0x00000004
  RTC_TSTR_SU_3* = (0x00000008 shl RTC_TSTR_SU_Pos) ## !< 0x00000008

## *******************  Bits definition for RTC_TSDR register  ****************

const
  RTC_TSDR_WDU_Pos* = (13)
  RTC_TSDR_WDU_Msk* = (0x00000007 shl RTC_TSDR_WDU_Pos) ## !< 0x0000E000
  RTC_TSDR_WDU* = RTC_TSDR_WDU_Msk
  RTC_TSDR_WDU_0* = (0x00000001 shl RTC_TSDR_WDU_Pos) ## !< 0x00002000
  RTC_TSDR_WDU_1* = (0x00000002 shl RTC_TSDR_WDU_Pos) ## !< 0x00004000
  RTC_TSDR_WDU_2* = (0x00000004 shl RTC_TSDR_WDU_Pos) ## !< 0x00008000
  RTC_TSDR_MT_Pos* = (12)
  RTC_TSDR_MT_Msk* = (0x00000001 shl RTC_TSDR_MT_Pos) ## !< 0x00001000
  RTC_TSDR_MT* = RTC_TSDR_MT_Msk
  RTC_TSDR_MU_Pos* = (8)
  RTC_TSDR_MU_Msk* = (0x0000000F shl RTC_TSDR_MU_Pos) ## !< 0x00000F00
  RTC_TSDR_MU* = RTC_TSDR_MU_Msk
  RTC_TSDR_MU_0* = (0x00000001 shl RTC_TSDR_MU_Pos) ## !< 0x00000100
  RTC_TSDR_MU_1* = (0x00000002 shl RTC_TSDR_MU_Pos) ## !< 0x00000200
  RTC_TSDR_MU_2* = (0x00000004 shl RTC_TSDR_MU_Pos) ## !< 0x00000400
  RTC_TSDR_MU_3* = (0x00000008 shl RTC_TSDR_MU_Pos) ## !< 0x00000800
  RTC_TSDR_DT_Pos* = (4)
  RTC_TSDR_DT_Msk* = (0x00000003 shl RTC_TSDR_DT_Pos) ## !< 0x00000030
  RTC_TSDR_DT* = RTC_TSDR_DT_Msk
  RTC_TSDR_DT_0* = (0x00000001 shl RTC_TSDR_DT_Pos) ## !< 0x00000010
  RTC_TSDR_DT_1* = (0x00000002 shl RTC_TSDR_DT_Pos) ## !< 0x00000020
  RTC_TSDR_DU_Pos* = (0)
  RTC_TSDR_DU_Msk* = (0x0000000F shl RTC_TSDR_DU_Pos) ## !< 0x0000000F
  RTC_TSDR_DU* = RTC_TSDR_DU_Msk
  RTC_TSDR_DU_0* = (0x00000001 shl RTC_TSDR_DU_Pos) ## !< 0x00000001
  RTC_TSDR_DU_1* = (0x00000002 shl RTC_TSDR_DU_Pos) ## !< 0x00000002
  RTC_TSDR_DU_2* = (0x00000004 shl RTC_TSDR_DU_Pos) ## !< 0x00000004
  RTC_TSDR_DU_3* = (0x00000008 shl RTC_TSDR_DU_Pos) ## !< 0x00000008

## *******************  Bits definition for RTC_TSSSR register  ***************

const
  RTC_TSSSR_SS_Pos* = (0)
  RTC_TSSSR_SS_Msk* = (0x0000FFFF shl RTC_TSSSR_SS_Pos) ## !< 0x0000FFFF
  RTC_TSSSR_SS* = RTC_TSSSR_SS_Msk

## *******************  Bits definition for RTC_CAL register  ****************

const
  RTC_CALR_CALP_Pos* = (15)
  RTC_CALR_CALP_Msk* = (0x00000001 shl RTC_CALR_CALP_Pos) ## !< 0x00008000
  RTC_CALR_CALP* = RTC_CALR_CALP_Msk
  RTC_CALR_CALW8_Pos* = (14)
  RTC_CALR_CALW8_Msk* = (0x00000001 shl RTC_CALR_CALW8_Pos) ## !< 0x00004000
  RTC_CALR_CALW8* = RTC_CALR_CALW8_Msk
  RTC_CALR_CALW16_Pos* = (13)
  RTC_CALR_CALW16_Msk* = (0x00000001 shl RTC_CALR_CALW16_Pos) ## !< 0x00002000
  RTC_CALR_CALW16* = RTC_CALR_CALW16_Msk
  RTC_CALR_CALM_Pos* = (0)
  RTC_CALR_CALM_Msk* = (0x000001FF shl RTC_CALR_CALM_Pos) ## !< 0x000001FF
  RTC_CALR_CALM* = RTC_CALR_CALM_Msk
  RTC_CALR_CALM_0* = (0x00000001 shl RTC_CALR_CALM_Pos) ## !< 0x00000001
  RTC_CALR_CALM_1* = (0x00000002 shl RTC_CALR_CALM_Pos) ## !< 0x00000002
  RTC_CALR_CALM_2* = (0x00000004 shl RTC_CALR_CALM_Pos) ## !< 0x00000004
  RTC_CALR_CALM_3* = (0x00000008 shl RTC_CALR_CALM_Pos) ## !< 0x00000008
  RTC_CALR_CALM_4* = (0x00000010 shl RTC_CALR_CALM_Pos) ## !< 0x00000010
  RTC_CALR_CALM_5* = (0x00000020 shl RTC_CALR_CALM_Pos) ## !< 0x00000020
  RTC_CALR_CALM_6* = (0x00000040 shl RTC_CALR_CALM_Pos) ## !< 0x00000040
  RTC_CALR_CALM_7* = (0x00000080 shl RTC_CALR_CALM_Pos) ## !< 0x00000080
  RTC_CALR_CALM_8* = (0x00000100 shl RTC_CALR_CALM_Pos) ## !< 0x00000100

## *******************  Bits definition for RTC_TAFCR register  ***************

const
  RTC_TAFCR_ALARMOUTTYPE_Pos* = (18)
  RTC_TAFCR_ALARMOUTTYPE_Msk* = (0x00000001 shl RTC_TAFCR_ALARMOUTTYPE_Pos) ## !< 0x00040000
  RTC_TAFCR_ALARMOUTTYPE* = RTC_TAFCR_ALARMOUTTYPE_Msk
  RTC_TAFCR_TAMPPUDIS_Pos* = (15)
  RTC_TAFCR_TAMPPUDIS_Msk* = (0x00000001 shl RTC_TAFCR_TAMPPUDIS_Pos) ## !< 0x00008000
  RTC_TAFCR_TAMPPUDIS* = RTC_TAFCR_TAMPPUDIS_Msk
  RTC_TAFCR_TAMPPRCH_Pos* = (13)
  RTC_TAFCR_TAMPPRCH_Msk* = (0x00000003 shl RTC_TAFCR_TAMPPRCH_Pos) ## !< 0x00006000
  RTC_TAFCR_TAMPPRCH* = RTC_TAFCR_TAMPPRCH_Msk
  RTC_TAFCR_TAMPPRCH_0* = (0x00000001 shl RTC_TAFCR_TAMPPRCH_Pos) ## !< 0x00002000
  RTC_TAFCR_TAMPPRCH_1* = (0x00000002 shl RTC_TAFCR_TAMPPRCH_Pos) ## !< 0x00004000
  RTC_TAFCR_TAMPFLT_Pos* = (11)
  RTC_TAFCR_TAMPFLT_Msk* = (0x00000003 shl RTC_TAFCR_TAMPFLT_Pos) ## !< 0x00001800
  RTC_TAFCR_TAMPFLT* = RTC_TAFCR_TAMPFLT_Msk
  RTC_TAFCR_TAMPFLT_0* = (0x00000001 shl RTC_TAFCR_TAMPFLT_Pos) ## !< 0x00000800
  RTC_TAFCR_TAMPFLT_1* = (0x00000002 shl RTC_TAFCR_TAMPFLT_Pos) ## !< 0x00001000
  RTC_TAFCR_TAMPFREQ_Pos* = (8)
  RTC_TAFCR_TAMPFREQ_Msk* = (0x00000007 shl RTC_TAFCR_TAMPFREQ_Pos) ## !< 0x00000700
  RTC_TAFCR_TAMPFREQ* = RTC_TAFCR_TAMPFREQ_Msk
  RTC_TAFCR_TAMPFREQ_0* = (0x00000001 shl RTC_TAFCR_TAMPFREQ_Pos) ## !< 0x00000100
  RTC_TAFCR_TAMPFREQ_1* = (0x00000002 shl RTC_TAFCR_TAMPFREQ_Pos) ## !< 0x00000200
  RTC_TAFCR_TAMPFREQ_2* = (0x00000004 shl RTC_TAFCR_TAMPFREQ_Pos) ## !< 0x00000400
  RTC_TAFCR_TAMPTS_Pos* = (7)
  RTC_TAFCR_TAMPTS_Msk* = (0x00000001 shl RTC_TAFCR_TAMPTS_Pos) ## !< 0x00000080
  RTC_TAFCR_TAMPTS* = RTC_TAFCR_TAMPTS_Msk
  RTC_TAFCR_TAMP3TRG_Pos* = (6)
  RTC_TAFCR_TAMP3TRG_Msk* = (0x00000001 shl RTC_TAFCR_TAMP3TRG_Pos) ## !< 0x00000040
  RTC_TAFCR_TAMP3TRG* = RTC_TAFCR_TAMP3TRG_Msk
  RTC_TAFCR_TAMP3E_Pos* = (5)
  RTC_TAFCR_TAMP3E_Msk* = (0x00000001 shl RTC_TAFCR_TAMP3E_Pos) ## !< 0x00000020
  RTC_TAFCR_TAMP3E* = RTC_TAFCR_TAMP3E_Msk
  RTC_TAFCR_TAMP2TRG_Pos* = (4)
  RTC_TAFCR_TAMP2TRG_Msk* = (0x00000001 shl RTC_TAFCR_TAMP2TRG_Pos) ## !< 0x00000010
  RTC_TAFCR_TAMP2TRG* = RTC_TAFCR_TAMP2TRG_Msk
  RTC_TAFCR_TAMP2E_Pos* = (3)
  RTC_TAFCR_TAMP2E_Msk* = (0x00000001 shl RTC_TAFCR_TAMP2E_Pos) ## !< 0x00000008
  RTC_TAFCR_TAMP2E* = RTC_TAFCR_TAMP2E_Msk
  RTC_TAFCR_TAMPIE_Pos* = (2)
  RTC_TAFCR_TAMPIE_Msk* = (0x00000001 shl RTC_TAFCR_TAMPIE_Pos) ## !< 0x00000004
  RTC_TAFCR_TAMPIE* = RTC_TAFCR_TAMPIE_Msk
  RTC_TAFCR_TAMP1TRG_Pos* = (1)
  RTC_TAFCR_TAMP1TRG_Msk* = (0x00000001 shl RTC_TAFCR_TAMP1TRG_Pos) ## !< 0x00000002
  RTC_TAFCR_TAMP1TRG* = RTC_TAFCR_TAMP1TRG_Msk
  RTC_TAFCR_TAMP1E_Pos* = (0)
  RTC_TAFCR_TAMP1E_Msk* = (0x00000001 shl RTC_TAFCR_TAMP1E_Pos) ## !< 0x00000001
  RTC_TAFCR_TAMP1E* = RTC_TAFCR_TAMP1E_Msk

## *******************  Bits definition for RTC_ALRMASSR register  ************

const
  RTC_ALRMASSR_MASKSS_Pos* = (24)
  RTC_ALRMASSR_MASKSS_Msk* = (0x0000000F shl RTC_ALRMASSR_MASKSS_Pos) ## !< 0x0F000000
  RTC_ALRMASSR_MASKSS* = RTC_ALRMASSR_MASKSS_Msk
  RTC_ALRMASSR_MASKSS_0* = (0x00000001 shl RTC_ALRMASSR_MASKSS_Pos) ## !< 0x01000000
  RTC_ALRMASSR_MASKSS_1* = (0x00000002 shl RTC_ALRMASSR_MASKSS_Pos) ## !< 0x02000000
  RTC_ALRMASSR_MASKSS_2* = (0x00000004 shl RTC_ALRMASSR_MASKSS_Pos) ## !< 0x04000000
  RTC_ALRMASSR_MASKSS_3* = (0x00000008 shl RTC_ALRMASSR_MASKSS_Pos) ## !< 0x08000000
  RTC_ALRMASSR_SS_Pos* = (0)
  RTC_ALRMASSR_SS_Msk* = (0x00007FFF shl RTC_ALRMASSR_SS_Pos) ## !< 0x00007FFF
  RTC_ALRMASSR_SS* = RTC_ALRMASSR_SS_Msk

## *******************  Bits definition for RTC_ALRMBSSR register  ************

const
  RTC_ALRMBSSR_MASKSS_Pos* = (24)
  RTC_ALRMBSSR_MASKSS_Msk* = (0x0000000F shl RTC_ALRMBSSR_MASKSS_Pos) ## !< 0x0F000000
  RTC_ALRMBSSR_MASKSS* = RTC_ALRMBSSR_MASKSS_Msk
  RTC_ALRMBSSR_MASKSS_0* = (0x00000001 shl RTC_ALRMBSSR_MASKSS_Pos) ## !< 0x01000000
  RTC_ALRMBSSR_MASKSS_1* = (0x00000002 shl RTC_ALRMBSSR_MASKSS_Pos) ## !< 0x02000000
  RTC_ALRMBSSR_MASKSS_2* = (0x00000004 shl RTC_ALRMBSSR_MASKSS_Pos) ## !< 0x04000000
  RTC_ALRMBSSR_MASKSS_3* = (0x00000008 shl RTC_ALRMBSSR_MASKSS_Pos) ## !< 0x08000000
  RTC_ALRMBSSR_SS_Pos* = (0)
  RTC_ALRMBSSR_SS_Msk* = (0x00007FFF shl RTC_ALRMBSSR_SS_Pos) ## !< 0x00007FFF
  RTC_ALRMBSSR_SS* = RTC_ALRMBSSR_SS_Msk

## *******************  Bits definition for RTC_BKP0R register  ***************

const
  RTC_BKP0R_Pos* = (0)
  RTC_BKP0R_Msk* = (0xFFFFFFFF shl RTC_BKP0R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP0R* = RTC_BKP0R_Msk

## *******************  Bits definition for RTC_BKP1R register  ***************

const
  RTC_BKP1R_Pos* = (0)
  RTC_BKP1R_Msk* = (0xFFFFFFFF shl RTC_BKP1R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP1R* = RTC_BKP1R_Msk

## *******************  Bits definition for RTC_BKP2R register  ***************

const
  RTC_BKP2R_Pos* = (0)
  RTC_BKP2R_Msk* = (0xFFFFFFFF shl RTC_BKP2R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP2R* = RTC_BKP2R_Msk

## *******************  Bits definition for RTC_BKP3R register  ***************

const
  RTC_BKP3R_Pos* = (0)
  RTC_BKP3R_Msk* = (0xFFFFFFFF shl RTC_BKP3R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP3R* = RTC_BKP3R_Msk

## *******************  Bits definition for RTC_BKP4R register  ***************

const
  RTC_BKP4R_Pos* = (0)
  RTC_BKP4R_Msk* = (0xFFFFFFFF shl RTC_BKP4R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP4R* = RTC_BKP4R_Msk

## *******************  Bits definition for RTC_BKP5R register  ***************

const
  RTC_BKP5R_Pos* = (0)
  RTC_BKP5R_Msk* = (0xFFFFFFFF shl RTC_BKP5R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP5R* = RTC_BKP5R_Msk

## *******************  Bits definition for RTC_BKP6R register  ***************

const
  RTC_BKP6R_Pos* = (0)
  RTC_BKP6R_Msk* = (0xFFFFFFFF shl RTC_BKP6R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP6R* = RTC_BKP6R_Msk

## *******************  Bits definition for RTC_BKP7R register  ***************

const
  RTC_BKP7R_Pos* = (0)
  RTC_BKP7R_Msk* = (0xFFFFFFFF shl RTC_BKP7R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP7R* = RTC_BKP7R_Msk

## *******************  Bits definition for RTC_BKP8R register  ***************

const
  RTC_BKP8R_Pos* = (0)
  RTC_BKP8R_Msk* = (0xFFFFFFFF shl RTC_BKP8R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP8R* = RTC_BKP8R_Msk

## *******************  Bits definition for RTC_BKP9R register  ***************

const
  RTC_BKP9R_Pos* = (0)
  RTC_BKP9R_Msk* = (0xFFFFFFFF shl RTC_BKP9R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP9R* = RTC_BKP9R_Msk

## *******************  Bits definition for RTC_BKP10R register  **************

const
  RTC_BKP10R_Pos* = (0)
  RTC_BKP10R_Msk* = (0xFFFFFFFF shl RTC_BKP10R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP10R* = RTC_BKP10R_Msk

## *******************  Bits definition for RTC_BKP11R register  **************

const
  RTC_BKP11R_Pos* = (0)
  RTC_BKP11R_Msk* = (0xFFFFFFFF shl RTC_BKP11R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP11R* = RTC_BKP11R_Msk

## *******************  Bits definition for RTC_BKP12R register  **************

const
  RTC_BKP12R_Pos* = (0)
  RTC_BKP12R_Msk* = (0xFFFFFFFF shl RTC_BKP12R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP12R* = RTC_BKP12R_Msk

## *******************  Bits definition for RTC_BKP13R register  **************

const
  RTC_BKP13R_Pos* = (0)
  RTC_BKP13R_Msk* = (0xFFFFFFFF shl RTC_BKP13R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP13R* = RTC_BKP13R_Msk

## *******************  Bits definition for RTC_BKP14R register  **************

const
  RTC_BKP14R_Pos* = (0)
  RTC_BKP14R_Msk* = (0xFFFFFFFF shl RTC_BKP14R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP14R* = RTC_BKP14R_Msk

## *******************  Bits definition for RTC_BKP15R register  **************

const
  RTC_BKP15R_Pos* = (0)
  RTC_BKP15R_Msk* = (0xFFFFFFFF shl RTC_BKP15R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP15R* = RTC_BKP15R_Msk

## *******************  Bits definition for RTC_BKP16R register  **************

const
  RTC_BKP16R_Pos* = (0)
  RTC_BKP16R_Msk* = (0xFFFFFFFF shl RTC_BKP16R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP16R* = RTC_BKP16R_Msk

## *******************  Bits definition for RTC_BKP17R register  **************

const
  RTC_BKP17R_Pos* = (0)
  RTC_BKP17R_Msk* = (0xFFFFFFFF shl RTC_BKP17R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP17R* = RTC_BKP17R_Msk

## *******************  Bits definition for RTC_BKP18R register  **************

const
  RTC_BKP18R_Pos* = (0)
  RTC_BKP18R_Msk* = (0xFFFFFFFF shl RTC_BKP18R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP18R* = RTC_BKP18R_Msk

## *******************  Bits definition for RTC_BKP19R register  **************

const
  RTC_BKP19R_Pos* = (0)
  RTC_BKP19R_Msk* = (0xFFFFFFFF shl RTC_BKP19R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP19R* = RTC_BKP19R_Msk

## *******************  Bits definition for RTC_BKP20R register  **************

const
  RTC_BKP20R_Pos* = (0)
  RTC_BKP20R_Msk* = (0xFFFFFFFF shl RTC_BKP20R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP20R* = RTC_BKP20R_Msk

## *******************  Bits definition for RTC_BKP21R register  **************

const
  RTC_BKP21R_Pos* = (0)
  RTC_BKP21R_Msk* = (0xFFFFFFFF shl RTC_BKP21R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP21R* = RTC_BKP21R_Msk

## *******************  Bits definition for RTC_BKP22R register  **************

const
  RTC_BKP22R_Pos* = (0)
  RTC_BKP22R_Msk* = (0xFFFFFFFF shl RTC_BKP22R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP22R* = RTC_BKP22R_Msk

## *******************  Bits definition for RTC_BKP23R register  **************

const
  RTC_BKP23R_Pos* = (0)
  RTC_BKP23R_Msk* = (0xFFFFFFFF shl RTC_BKP23R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP23R* = RTC_BKP23R_Msk

## *******************  Bits definition for RTC_BKP24R register  **************

const
  RTC_BKP24R_Pos* = (0)
  RTC_BKP24R_Msk* = (0xFFFFFFFF shl RTC_BKP24R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP24R* = RTC_BKP24R_Msk

## *******************  Bits definition for RTC_BKP25R register  **************

const
  RTC_BKP25R_Pos* = (0)
  RTC_BKP25R_Msk* = (0xFFFFFFFF shl RTC_BKP25R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP25R* = RTC_BKP25R_Msk

## *******************  Bits definition for RTC_BKP26R register  **************

const
  RTC_BKP26R_Pos* = (0)
  RTC_BKP26R_Msk* = (0xFFFFFFFF shl RTC_BKP26R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP26R* = RTC_BKP26R_Msk

## *******************  Bits definition for RTC_BKP27R register  **************

const
  RTC_BKP27R_Pos* = (0)
  RTC_BKP27R_Msk* = (0xFFFFFFFF shl RTC_BKP27R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP27R* = RTC_BKP27R_Msk

## *******************  Bits definition for RTC_BKP28R register  **************

const
  RTC_BKP28R_Pos* = (0)
  RTC_BKP28R_Msk* = (0xFFFFFFFF shl RTC_BKP28R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP28R* = RTC_BKP28R_Msk

## *******************  Bits definition for RTC_BKP29R register  **************

const
  RTC_BKP29R_Pos* = (0)
  RTC_BKP29R_Msk* = (0xFFFFFFFF shl RTC_BKP29R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP29R* = RTC_BKP29R_Msk

## *******************  Bits definition for RTC_BKP30R register  **************

const
  RTC_BKP30R_Pos* = (0)
  RTC_BKP30R_Msk* = (0xFFFFFFFF shl RTC_BKP30R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP30R* = RTC_BKP30R_Msk

## *******************  Bits definition for RTC_BKP31R register  **************

const
  RTC_BKP31R_Pos* = (0)
  RTC_BKP31R_Msk* = (0xFFFFFFFF shl RTC_BKP31R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP31R* = RTC_BKP31R_Msk

## ******************* Number of backup registers *****************************

const
  RTC_BKP_NUMBER* = 32

## ****************************************************************************
##
##                      Serial Peripheral Interface (SPI)
##
## ****************************************************************************
##
##  @brief Specific device feature definitions (not present on all devices in the STM32F3 serie)
##

const
  SPI_I2S_SUPPORT* = true

## ******************  Bit definition for SPI_CR1 register  *******************

const
  SPI_CR1_CPHA_Pos* = (0)
  SPI_CR1_CPHA_Msk* = (0x00000001 shl SPI_CR1_CPHA_Pos) ## !< 0x00000001
  SPI_CR1_CPHA* = SPI_CR1_CPHA_Msk
  SPI_CR1_CPOL_Pos* = (1)
  SPI_CR1_CPOL_Msk* = (0x00000001 shl SPI_CR1_CPOL_Pos) ## !< 0x00000002
  SPI_CR1_CPOL* = SPI_CR1_CPOL_Msk
  SPI_CR1_MSTR_Pos* = (2)
  SPI_CR1_MSTR_Msk* = (0x00000001 shl SPI_CR1_MSTR_Pos) ## !< 0x00000004
  SPI_CR1_MSTR* = SPI_CR1_MSTR_Msk
  SPI_CR1_BR_Pos* = (3)
  SPI_CR1_BR_Msk* = (0x00000007 shl SPI_CR1_BR_Pos) ## !< 0x00000038
  SPI_CR1_BR* = SPI_CR1_BR_Msk
  SPI_CR1_BR_0* = (0x00000001 shl SPI_CR1_BR_Pos) ## !< 0x00000008
  SPI_CR1_BR_1* = (0x00000002 shl SPI_CR1_BR_Pos) ## !< 0x00000010
  SPI_CR1_BR_2* = (0x00000004 shl SPI_CR1_BR_Pos) ## !< 0x00000020
  SPI_CR1_SPE_Pos* = (6)
  SPI_CR1_SPE_Msk* = (0x00000001 shl SPI_CR1_SPE_Pos) ## !< 0x00000040
  SPI_CR1_SPE* = SPI_CR1_SPE_Msk
  SPI_CR1_LSBFIRST_Pos* = (7)
  SPI_CR1_LSBFIRST_Msk* = (0x00000001 shl SPI_CR1_LSBFIRST_Pos) ## !< 0x00000080
  SPI_CR1_LSBFIRST* = SPI_CR1_LSBFIRST_Msk
  SPI_CR1_SSI_Pos* = (8)
  SPI_CR1_SSI_Msk* = (0x00000001 shl SPI_CR1_SSI_Pos) ## !< 0x00000100
  SPI_CR1_SSI* = SPI_CR1_SSI_Msk
  SPI_CR1_SSM_Pos* = (9)
  SPI_CR1_SSM_Msk* = (0x00000001 shl SPI_CR1_SSM_Pos) ## !< 0x00000200
  SPI_CR1_SSM* = SPI_CR1_SSM_Msk
  SPI_CR1_RXONLY_Pos* = (10)
  SPI_CR1_RXONLY_Msk* = (0x00000001 shl SPI_CR1_RXONLY_Pos) ## !< 0x00000400
  SPI_CR1_RXONLY* = SPI_CR1_RXONLY_Msk
  SPI_CR1_DFF_Pos* = (11)
  SPI_CR1_DFF_Msk* = (0x00000001 shl SPI_CR1_DFF_Pos) ## !< 0x00000800
  SPI_CR1_DFF* = SPI_CR1_DFF_Msk
  SPI_CR1_CRCNEXT_Pos* = (12)
  SPI_CR1_CRCNEXT_Msk* = (0x00000001 shl SPI_CR1_CRCNEXT_Pos) ## !< 0x00001000
  SPI_CR1_CRCNEXT* = SPI_CR1_CRCNEXT_Msk
  SPI_CR1_CRCEN_Pos* = (13)
  SPI_CR1_CRCEN_Msk* = (0x00000001 shl SPI_CR1_CRCEN_Pos) ## !< 0x00002000
  SPI_CR1_CRCEN* = SPI_CR1_CRCEN_Msk
  SPI_CR1_BIDIOE_Pos* = (14)
  SPI_CR1_BIDIOE_Msk* = (0x00000001 shl SPI_CR1_BIDIOE_Pos) ## !< 0x00004000
  SPI_CR1_BIDIOE* = SPI_CR1_BIDIOE_Msk
  SPI_CR1_BIDIMODE_Pos* = (15)
  SPI_CR1_BIDIMODE_Msk* = (0x00000001 shl SPI_CR1_BIDIMODE_Pos) ## !< 0x00008000
  SPI_CR1_BIDIMODE* = SPI_CR1_BIDIMODE_Msk

## ******************  Bit definition for SPI_CR2 register  *******************

const
  SPI_CR2_RXDMAEN_Pos* = (0)
  SPI_CR2_RXDMAEN_Msk* = (0x00000001 shl SPI_CR2_RXDMAEN_Pos) ## !< 0x00000001
  SPI_CR2_RXDMAEN* = SPI_CR2_RXDMAEN_Msk
  SPI_CR2_TXDMAEN_Pos* = (1)
  SPI_CR2_TXDMAEN_Msk* = (0x00000001 shl SPI_CR2_TXDMAEN_Pos) ## !< 0x00000002
  SPI_CR2_TXDMAEN* = SPI_CR2_TXDMAEN_Msk
  SPI_CR2_SSOE_Pos* = (2)
  SPI_CR2_SSOE_Msk* = (0x00000001 shl SPI_CR2_SSOE_Pos) ## !< 0x00000004
  SPI_CR2_SSOE* = SPI_CR2_SSOE_Msk
  SPI_CR2_FRF_Pos* = (4)
  SPI_CR2_FRF_Msk* = (0x00000001 shl SPI_CR2_FRF_Pos) ## !< 0x00000010
  SPI_CR2_FRF* = SPI_CR2_FRF_Msk
  SPI_CR2_ERRIE_Pos* = (5)
  SPI_CR2_ERRIE_Msk* = (0x00000001 shl SPI_CR2_ERRIE_Pos) ## !< 0x00000020
  SPI_CR2_ERRIE* = SPI_CR2_ERRIE_Msk
  SPI_CR2_RXNEIE_Pos* = (6)
  SPI_CR2_RXNEIE_Msk* = (0x00000001 shl SPI_CR2_RXNEIE_Pos) ## !< 0x00000040
  SPI_CR2_RXNEIE* = SPI_CR2_RXNEIE_Msk
  SPI_CR2_TXEIE_Pos* = (7)
  SPI_CR2_TXEIE_Msk* = (0x00000001 shl SPI_CR2_TXEIE_Pos) ## !< 0x00000080
  SPI_CR2_TXEIE* = SPI_CR2_TXEIE_Msk

## *******************  Bit definition for SPI_SR register  *******************

const
  SPI_SR_RXNE_Pos* = (0)
  SPI_SR_RXNE_Msk* = (0x00000001 shl SPI_SR_RXNE_Pos) ## !< 0x00000001
  SPI_SR_RXNE* = SPI_SR_RXNE_Msk
  SPI_SR_TXE_Pos* = (1)
  SPI_SR_TXE_Msk* = (0x00000001 shl SPI_SR_TXE_Pos) ## !< 0x00000002
  SPI_SR_TXE* = SPI_SR_TXE_Msk
  SPI_SR_CHSIDE_Pos* = (2)
  SPI_SR_CHSIDE_Msk* = (0x00000001 shl SPI_SR_CHSIDE_Pos) ## !< 0x00000004
  SPI_SR_CHSIDE* = SPI_SR_CHSIDE_Msk
  SPI_SR_UDR_Pos* = (3)
  SPI_SR_UDR_Msk* = (0x00000001 shl SPI_SR_UDR_Pos) ## !< 0x00000008
  SPI_SR_UDR* = SPI_SR_UDR_Msk
  SPI_SR_CRCERR_Pos* = (4)
  SPI_SR_CRCERR_Msk* = (0x00000001 shl SPI_SR_CRCERR_Pos) ## !< 0x00000010
  SPI_SR_CRCERR* = SPI_SR_CRCERR_Msk
  SPI_SR_MODF_Pos* = (5)
  SPI_SR_MODF_Msk* = (0x00000001 shl SPI_SR_MODF_Pos) ## !< 0x00000020
  SPI_SR_MODF* = SPI_SR_MODF_Msk
  SPI_SR_OVR_Pos* = (6)
  SPI_SR_OVR_Msk* = (0x00000001 shl SPI_SR_OVR_Pos) ## !< 0x00000040
  SPI_SR_OVR* = SPI_SR_OVR_Msk
  SPI_SR_BSY_Pos* = (7)
  SPI_SR_BSY_Msk* = (0x00000001 shl SPI_SR_BSY_Pos) ## !< 0x00000080
  SPI_SR_BSY* = SPI_SR_BSY_Msk
  SPI_SR_FRE_Pos* = (8)
  SPI_SR_FRE_Msk* = (0x00000001 shl SPI_SR_FRE_Pos) ## !< 0x00000100
  SPI_SR_FRE* = SPI_SR_FRE_Msk

## *******************  Bit definition for SPI_DR register  *******************

const
  SPI_DR_DR_Pos* = (0)
  SPI_DR_DR_Msk* = (0x0000FFFF shl SPI_DR_DR_Pos) ## !< 0x0000FFFF
  SPI_DR_DR* = SPI_DR_DR_Msk

## ******************  Bit definition for SPI_CRCPR register  *****************

const
  SPI_CRCPR_CRCPOLY_Pos* = (0)
  SPI_CRCPR_CRCPOLY_Msk* = (0x0000FFFF shl SPI_CRCPR_CRCPOLY_Pos) ## !< 0x0000FFFF
  SPI_CRCPR_CRCPOLY* = SPI_CRCPR_CRCPOLY_Msk

## *****************  Bit definition for SPI_RXCRCR register  *****************

const
  SPI_RXCRCR_RXCRC_Pos* = (0)
  SPI_RXCRCR_RXCRC_Msk* = (0x0000FFFF shl SPI_RXCRCR_RXCRC_Pos) ## !< 0x0000FFFF
  SPI_RXCRCR_RXCRC* = SPI_RXCRCR_RXCRC_Msk

## *****************  Bit definition for SPI_TXCRCR register  *****************

const
  SPI_TXCRCR_TXCRC_Pos* = (0)
  SPI_TXCRCR_TXCRC_Msk* = (0x0000FFFF shl SPI_TXCRCR_TXCRC_Pos) ## !< 0x0000FFFF
  SPI_TXCRCR_TXCRC* = SPI_TXCRCR_TXCRC_Msk

## *****************  Bit definition for SPI_I2SCFGR register  ****************

const
  SPI_I2SCFGR_CHLEN_Pos* = (0)
  SPI_I2SCFGR_CHLEN_Msk* = (0x00000001 shl SPI_I2SCFGR_CHLEN_Pos) ## !< 0x00000001
  SPI_I2SCFGR_CHLEN* = SPI_I2SCFGR_CHLEN_Msk
  SPI_I2SCFGR_DATLEN_Pos* = (1)
  SPI_I2SCFGR_DATLEN_Msk* = (0x00000003 shl SPI_I2SCFGR_DATLEN_Pos) ## !< 0x00000006
  SPI_I2SCFGR_DATLEN* = SPI_I2SCFGR_DATLEN_Msk
  SPI_I2SCFGR_DATLEN_0* = (0x00000001 shl SPI_I2SCFGR_DATLEN_Pos) ## !< 0x00000002
  SPI_I2SCFGR_DATLEN_1* = (0x00000002 shl SPI_I2SCFGR_DATLEN_Pos) ## !< 0x00000004
  SPI_I2SCFGR_CKPOL_Pos* = (3)
  SPI_I2SCFGR_CKPOL_Msk* = (0x00000001 shl SPI_I2SCFGR_CKPOL_Pos) ## !< 0x00000008
  SPI_I2SCFGR_CKPOL* = SPI_I2SCFGR_CKPOL_Msk
  SPI_I2SCFGR_I2SSTD_Pos* = (4)
  SPI_I2SCFGR_I2SSTD_Msk* = (0x00000003 shl SPI_I2SCFGR_I2SSTD_Pos) ## !< 0x00000030
  SPI_I2SCFGR_I2SSTD* = SPI_I2SCFGR_I2SSTD_Msk
  SPI_I2SCFGR_I2SSTD_0* = (0x00000001 shl SPI_I2SCFGR_I2SSTD_Pos) ## !< 0x00000010
  SPI_I2SCFGR_I2SSTD_1* = (0x00000002 shl SPI_I2SCFGR_I2SSTD_Pos) ## !< 0x00000020
  SPI_I2SCFGR_PCMSYNC_Pos* = (7)
  SPI_I2SCFGR_PCMSYNC_Msk* = (0x00000001 shl SPI_I2SCFGR_PCMSYNC_Pos) ## !< 0x00000080
  SPI_I2SCFGR_PCMSYNC* = SPI_I2SCFGR_PCMSYNC_Msk
  SPI_I2SCFGR_I2SCFG_Pos* = (8)
  SPI_I2SCFGR_I2SCFG_Msk* = (0x00000003 shl SPI_I2SCFGR_I2SCFG_Pos) ## !< 0x00000300
  SPI_I2SCFGR_I2SCFG* = SPI_I2SCFGR_I2SCFG_Msk
  SPI_I2SCFGR_I2SCFG_0* = (0x00000001 shl SPI_I2SCFGR_I2SCFG_Pos) ## !< 0x00000100
  SPI_I2SCFGR_I2SCFG_1* = (0x00000002 shl SPI_I2SCFGR_I2SCFG_Pos) ## !< 0x00000200
  SPI_I2SCFGR_I2SE_Pos* = (10)
  SPI_I2SCFGR_I2SE_Msk* = (0x00000001 shl SPI_I2SCFGR_I2SE_Pos) ## !< 0x00000400
  SPI_I2SCFGR_I2SE* = SPI_I2SCFGR_I2SE_Msk
  SPI_I2SCFGR_I2SMOD_Pos* = (11)
  SPI_I2SCFGR_I2SMOD_Msk* = (0x00000001 shl SPI_I2SCFGR_I2SMOD_Pos) ## !< 0x00000800
  SPI_I2SCFGR_I2SMOD* = SPI_I2SCFGR_I2SMOD_Msk

## *****************  Bit definition for SPI_I2SPR register  ******************

const
  SPI_I2SPR_I2SDIV_Pos* = (0)
  SPI_I2SPR_I2SDIV_Msk* = (0x000000FF shl SPI_I2SPR_I2SDIV_Pos) ## !< 0x000000FF
  SPI_I2SPR_I2SDIV* = SPI_I2SPR_I2SDIV_Msk
  SPI_I2SPR_ODD_Pos* = (8)
  SPI_I2SPR_ODD_Msk* = (0x00000001 shl SPI_I2SPR_ODD_Pos) ## !< 0x00000100
  SPI_I2SPR_ODD* = SPI_I2SPR_ODD_Msk
  SPI_I2SPR_MCKOE_Pos* = (9)
  SPI_I2SPR_MCKOE_Msk* = (0x00000001 shl SPI_I2SPR_MCKOE_Pos) ## !< 0x00000200
  SPI_I2SPR_MCKOE* = SPI_I2SPR_MCKOE_Msk

## ****************************************************************************
##
##                        System Configuration (SYSCFG)
##
## ****************************************************************************
## ****************  Bit definition for SYSCFG_MEMRMP register  ***************

const
  SYSCFG_MEMRMP_MEM_MODE_Pos* = (0)
  SYSCFG_MEMRMP_MEM_MODE_Msk* = (0x00000003 shl SYSCFG_MEMRMP_MEM_MODE_Pos) ## !< 0x00000003
  SYSCFG_MEMRMP_MEM_MODE* = SYSCFG_MEMRMP_MEM_MODE_Msk
  SYSCFG_MEMRMP_MEM_MODE_0* = (0x00000001 shl SYSCFG_MEMRMP_MEM_MODE_Pos) ## !< 0x00000001
  SYSCFG_MEMRMP_MEM_MODE_1* = (0x00000002 shl SYSCFG_MEMRMP_MEM_MODE_Pos) ## !< 0x00000002
  SYSCFG_MEMRMP_BOOT_MODE_Pos* = (8)
  SYSCFG_MEMRMP_BOOT_MODE_Msk* = (0x00000003 shl SYSCFG_MEMRMP_BOOT_MODE_Pos) ## !< 0x00000300
  SYSCFG_MEMRMP_BOOT_MODE* = SYSCFG_MEMRMP_BOOT_MODE_Msk
  SYSCFG_MEMRMP_BOOT_MODE_0* = (0x00000001 shl SYSCFG_MEMRMP_BOOT_MODE_Pos) ## !< 0x00000100
  SYSCFG_MEMRMP_BOOT_MODE_1* = (0x00000002 shl SYSCFG_MEMRMP_BOOT_MODE_Pos) ## !< 0x00000200

## ****************  Bit definition for SYSCFG_PMC register  ******************

const
  SYSCFG_PMC_USB_PU_Pos* = (0)
  SYSCFG_PMC_USB_PU_Msk* = (0x00000001 shl SYSCFG_PMC_USB_PU_Pos) ## !< 0x00000001
  SYSCFG_PMC_USB_PU* = SYSCFG_PMC_USB_PU_Msk
  SYSCFG_PMC_LCD_CAPA_Pos* = (1)
  SYSCFG_PMC_LCD_CAPA_Msk* = (0x0000001F shl SYSCFG_PMC_LCD_CAPA_Pos) ## !< 0x0000003E
  SYSCFG_PMC_LCD_CAPA* = SYSCFG_PMC_LCD_CAPA_Msk
  SYSCFG_PMC_LCD_CAPA_0* = (0x00000001 shl SYSCFG_PMC_LCD_CAPA_Pos) ## !< 0x00000002
  SYSCFG_PMC_LCD_CAPA_1* = (0x00000002 shl SYSCFG_PMC_LCD_CAPA_Pos) ## !< 0x00000004
  SYSCFG_PMC_LCD_CAPA_2* = (0x00000004 shl SYSCFG_PMC_LCD_CAPA_Pos) ## !< 0x00000008
  SYSCFG_PMC_LCD_CAPA_3* = (0x00000008 shl SYSCFG_PMC_LCD_CAPA_Pos) ## !< 0x00000010
  SYSCFG_PMC_LCD_CAPA_4* = (0x00000010 shl SYSCFG_PMC_LCD_CAPA_Pos) ## !< 0x00000020

## ****************  Bit definition for SYSCFG_EXTICR1 register  **************

const
  SYSCFG_EXTICR1_EXTI0_Pos* = (0)
  SYSCFG_EXTICR1_EXTI0_Msk* = (0x0000000F shl SYSCFG_EXTICR1_EXTI0_Pos) ## !< 0x0000000F
  SYSCFG_EXTICR1_EXTI0* = SYSCFG_EXTICR1_EXTI0_Msk
  SYSCFG_EXTICR1_EXTI1_Pos* = (4)
  SYSCFG_EXTICR1_EXTI1_Msk* = (0x0000000F shl SYSCFG_EXTICR1_EXTI1_Pos) ## !< 0x000000F0
  SYSCFG_EXTICR1_EXTI1* = SYSCFG_EXTICR1_EXTI1_Msk
  SYSCFG_EXTICR1_EXTI2_Pos* = (8)
  SYSCFG_EXTICR1_EXTI2_Msk* = (0x0000000F shl SYSCFG_EXTICR1_EXTI2_Pos) ## !< 0x00000F00
  SYSCFG_EXTICR1_EXTI2* = SYSCFG_EXTICR1_EXTI2_Msk
  SYSCFG_EXTICR1_EXTI3_Pos* = (12)
  SYSCFG_EXTICR1_EXTI3_Msk* = (0x0000000F shl SYSCFG_EXTICR1_EXTI3_Pos) ## !< 0x0000F000
  SYSCFG_EXTICR1_EXTI3* = SYSCFG_EXTICR1_EXTI3_Msk

## *
##  @brief  EXTI0 configuration
##

const
  SYSCFG_EXTICR1_EXTI0_PA* = (0x00000000) ## !< PA[0] pin
  SYSCFG_EXTICR1_EXTI0_PB* = (0x00000001) ## !< PB[0] pin
  SYSCFG_EXTICR1_EXTI0_PC* = (0x00000002) ## !< PC[0] pin
  SYSCFG_EXTICR1_EXTI0_PD* = (0x00000003) ## !< PD[0] pin
  SYSCFG_EXTICR1_EXTI0_PE* = (0x00000004) ## !< PE[0] pin
  SYSCFG_EXTICR1_EXTI0_PH* = (0x00000005) ## !< PH[0] pin
  SYSCFG_EXTICR1_EXTI0_PF* = (0x00000006) ## !< PF[0] pin
  SYSCFG_EXTICR1_EXTI0_PG* = (0x00000007) ## !< PG[0] pin

## *
##  @brief  EXTI1 configuration
##

const
  SYSCFG_EXTICR1_EXTI1_PA* = (0x00000000) ## !< PA[1] pin
  SYSCFG_EXTICR1_EXTI1_PB* = (0x00000010) ## !< PB[1] pin
  SYSCFG_EXTICR1_EXTI1_PC* = (0x00000020) ## !< PC[1] pin
  SYSCFG_EXTICR1_EXTI1_PD* = (0x00000030) ## !< PD[1] pin
  SYSCFG_EXTICR1_EXTI1_PE* = (0x00000040) ## !< PE[1] pin
  SYSCFG_EXTICR1_EXTI1_PH* = (0x00000050) ## !< PH[1] pin
  SYSCFG_EXTICR1_EXTI1_PF* = (0x00000060) ## !< PF[1] pin
  SYSCFG_EXTICR1_EXTI1_PG* = (0x00000070) ## !< PG[1] pin

## *
##  @brief  EXTI2 configuration
##

const
  SYSCFG_EXTICR1_EXTI2_PA* = (0x00000000) ## !< PA[2] pin
  SYSCFG_EXTICR1_EXTI2_PB* = (0x00000100) ## !< PB[2] pin
  SYSCFG_EXTICR1_EXTI2_PC* = (0x00000200) ## !< PC[2] pin
  SYSCFG_EXTICR1_EXTI2_PD* = (0x00000300) ## !< PD[2] pin
  SYSCFG_EXTICR1_EXTI2_PE* = (0x00000400) ## !< PE[2] pin
  SYSCFG_EXTICR1_EXTI2_PH* = (0x00000500) ## !< PH[2] pin
  SYSCFG_EXTICR1_EXTI2_PF* = (0x00000600) ## !< PF[2] pin
  SYSCFG_EXTICR1_EXTI2_PG* = (0x00000700) ## !< PG[2] pin

## *
##  @brief  EXTI3 configuration
##

const
  SYSCFG_EXTICR1_EXTI3_PA* = (0x00000000) ## !< PA[3] pin
  SYSCFG_EXTICR1_EXTI3_PB* = (0x00001000) ## !< PB[3] pin
  SYSCFG_EXTICR1_EXTI3_PC* = (0x00002000) ## !< PC[3] pin
  SYSCFG_EXTICR1_EXTI3_PD* = (0x00003000) ## !< PD[3] pin
  SYSCFG_EXTICR1_EXTI3_PE* = (0x00004000) ## !< PE[3] pin
  SYSCFG_EXTICR1_EXTI3_PF* = (0x00003000) ## !< PF[3] pin
  SYSCFG_EXTICR1_EXTI3_PG* = (0x00004000) ## !< PG[3] pin

## ****************  Bit definition for SYSCFG_EXTICR2 register  ****************

const
  SYSCFG_EXTICR2_EXTI4_Pos* = (0)
  SYSCFG_EXTICR2_EXTI4_Msk* = (0x0000000F shl SYSCFG_EXTICR2_EXTI4_Pos) ## !< 0x0000000F
  SYSCFG_EXTICR2_EXTI4* = SYSCFG_EXTICR2_EXTI4_Msk
  SYSCFG_EXTICR2_EXTI5_Pos* = (4)
  SYSCFG_EXTICR2_EXTI5_Msk* = (0x0000000F shl SYSCFG_EXTICR2_EXTI5_Pos) ## !< 0x000000F0
  SYSCFG_EXTICR2_EXTI5* = SYSCFG_EXTICR2_EXTI5_Msk
  SYSCFG_EXTICR2_EXTI6_Pos* = (8)
  SYSCFG_EXTICR2_EXTI6_Msk* = (0x0000000F shl SYSCFG_EXTICR2_EXTI6_Pos) ## !< 0x00000F00
  SYSCFG_EXTICR2_EXTI6* = SYSCFG_EXTICR2_EXTI6_Msk
  SYSCFG_EXTICR2_EXTI7_Pos* = (12)
  SYSCFG_EXTICR2_EXTI7_Msk* = (0x0000000F shl SYSCFG_EXTICR2_EXTI7_Pos) ## !< 0x0000F000
  SYSCFG_EXTICR2_EXTI7* = SYSCFG_EXTICR2_EXTI7_Msk

## *
##  @brief  EXTI4 configuration
##

const
  SYSCFG_EXTICR2_EXTI4_PA* = (0x00000000) ## !< PA[4] pin
  SYSCFG_EXTICR2_EXTI4_PB* = (0x00000001) ## !< PB[4] pin
  SYSCFG_EXTICR2_EXTI4_PC* = (0x00000002) ## !< PC[4] pin
  SYSCFG_EXTICR2_EXTI4_PD* = (0x00000003) ## !< PD[4] pin
  SYSCFG_EXTICR2_EXTI4_PE* = (0x00000004) ## !< PE[4] pin
  SYSCFG_EXTICR2_EXTI4_PF* = (0x00000006) ## !< PF[4] pin
  SYSCFG_EXTICR2_EXTI4_PG* = (0x00000007) ## !< PG[4] pin

## *
##  @brief  EXTI5 configuration
##

const
  SYSCFG_EXTICR2_EXTI5_PA* = (0x00000000) ## !< PA[5] pin
  SYSCFG_EXTICR2_EXTI5_PB* = (0x00000010) ## !< PB[5] pin
  SYSCFG_EXTICR2_EXTI5_PC* = (0x00000020) ## !< PC[5] pin
  SYSCFG_EXTICR2_EXTI5_PD* = (0x00000030) ## !< PD[5] pin
  SYSCFG_EXTICR2_EXTI5_PE* = (0x00000040) ## !< PE[5] pin
  SYSCFG_EXTICR2_EXTI5_PF* = (0x00000060) ## !< PF[5] pin
  SYSCFG_EXTICR2_EXTI5_PG* = (0x00000070) ## !< PG[5] pin

## *
##  @brief  EXTI6 configuration
##

const
  SYSCFG_EXTICR2_EXTI6_PA* = (0x00000000) ## !< PA[6] pin
  SYSCFG_EXTICR2_EXTI6_PB* = (0x00000100) ## !< PB[6] pin
  SYSCFG_EXTICR2_EXTI6_PC* = (0x00000200) ## !< PC[6] pin
  SYSCFG_EXTICR2_EXTI6_PD* = (0x00000300) ## !< PD[6] pin
  SYSCFG_EXTICR2_EXTI6_PE* = (0x00000400) ## !< PE[6] pin
  SYSCFG_EXTICR2_EXTI6_PF* = (0x00000600) ## !< PF[6] pin
  SYSCFG_EXTICR2_EXTI6_PG* = (0x00000700) ## !< PG[6] pin

## *
##  @brief  EXTI7 configuration
##

const
  SYSCFG_EXTICR2_EXTI7_PA* = (0x00000000) ## !< PA[7] pin
  SYSCFG_EXTICR2_EXTI7_PB* = (0x00001000) ## !< PB[7] pin
  SYSCFG_EXTICR2_EXTI7_PC* = (0x00002000) ## !< PC[7] pin
  SYSCFG_EXTICR2_EXTI7_PD* = (0x00003000) ## !< PD[7] pin
  SYSCFG_EXTICR2_EXTI7_PE* = (0x00004000) ## !< PE[7] pin
  SYSCFG_EXTICR2_EXTI7_PF* = (0x00006000) ## !< PF[7] pin
  SYSCFG_EXTICR2_EXTI7_PG* = (0x00007000) ## !< PG[7] pin

## ****************  Bit definition for SYSCFG_EXTICR3 register  ****************

const
  SYSCFG_EXTICR3_EXTI8_Pos* = (0)
  SYSCFG_EXTICR3_EXTI8_Msk* = (0x0000000F shl SYSCFG_EXTICR3_EXTI8_Pos) ## !< 0x0000000F
  SYSCFG_EXTICR3_EXTI8* = SYSCFG_EXTICR3_EXTI8_Msk
  SYSCFG_EXTICR3_EXTI9_Pos* = (4)
  SYSCFG_EXTICR3_EXTI9_Msk* = (0x0000000F shl SYSCFG_EXTICR3_EXTI9_Pos) ## !< 0x000000F0
  SYSCFG_EXTICR3_EXTI9* = SYSCFG_EXTICR3_EXTI9_Msk
  SYSCFG_EXTICR3_EXTI10_Pos* = (8)
  SYSCFG_EXTICR3_EXTI10_Msk* = (0x0000000F shl SYSCFG_EXTICR3_EXTI10_Pos) ## !< 0x00000F00
  SYSCFG_EXTICR3_EXTI10* = SYSCFG_EXTICR3_EXTI10_Msk
  SYSCFG_EXTICR3_EXTI11_Pos* = (12)
  SYSCFG_EXTICR3_EXTI11_Msk* = (0x0000000F shl SYSCFG_EXTICR3_EXTI11_Pos) ## !< 0x0000F000
  SYSCFG_EXTICR3_EXTI11* = SYSCFG_EXTICR3_EXTI11_Msk

## *
##  @brief  EXTI8 configuration
##

const
  SYSCFG_EXTICR3_EXTI8_PA* = (0x00000000) ## !< PA[8] pin
  SYSCFG_EXTICR3_EXTI8_PB* = (0x00000001) ## !< PB[8] pin
  SYSCFG_EXTICR3_EXTI8_PC* = (0x00000002) ## !< PC[8] pin
  SYSCFG_EXTICR3_EXTI8_PD* = (0x00000003) ## !< PD[8] pin
  SYSCFG_EXTICR3_EXTI8_PE* = (0x00000004) ## !< PE[8] pin
  SYSCFG_EXTICR3_EXTI8_PF* = (0x00000006) ## !< PF[8] pin
  SYSCFG_EXTICR3_EXTI8_PG* = (0x00000007) ## !< PG[8] pin

## *
##  @brief  EXTI9 configuration
##

const
  SYSCFG_EXTICR3_EXTI9_PA* = (0x00000000) ## !< PA[9] pin
  SYSCFG_EXTICR3_EXTI9_PB* = (0x00000010) ## !< PB[9] pin
  SYSCFG_EXTICR3_EXTI9_PC* = (0x00000020) ## !< PC[9] pin
  SYSCFG_EXTICR3_EXTI9_PD* = (0x00000030) ## !< PD[9] pin
  SYSCFG_EXTICR3_EXTI9_PE* = (0x00000040) ## !< PE[9] pin
  SYSCFG_EXTICR3_EXTI9_PF* = (0x00000060) ## !< PF[9] pin
  SYSCFG_EXTICR3_EXTI9_PG* = (0x00000070) ## !< PG[9] pin

## *
##  @brief  EXTI10 configuration
##

const
  SYSCFG_EXTICR3_EXTI10_PA* = (0x00000000) ## !< PA[10] pin
  SYSCFG_EXTICR3_EXTI10_PB* = (0x00000100) ## !< PB[10] pin
  SYSCFG_EXTICR3_EXTI10_PC* = (0x00000200) ## !< PC[10] pin
  SYSCFG_EXTICR3_EXTI10_PD* = (0x00000300) ## !< PD[10] pin
  SYSCFG_EXTICR3_EXTI10_PE* = (0x00000400) ## !< PE[10] pin
  SYSCFG_EXTICR3_EXTI10_PF* = (0x00000600) ## !< PF[10] pin
  SYSCFG_EXTICR3_EXTI10_PG* = (0x00000700) ## !< PG[10] pin

## *
##  @brief  EXTI11 configuration
##

const
  SYSCFG_EXTICR3_EXTI11_PA* = (0x00000000) ## !< PA[11] pin
  SYSCFG_EXTICR3_EXTI11_PB* = (0x00001000) ## !< PB[11] pin
  SYSCFG_EXTICR3_EXTI11_PC* = (0x00002000) ## !< PC[11] pin
  SYSCFG_EXTICR3_EXTI11_PD* = (0x00003000) ## !< PD[11] pin
  SYSCFG_EXTICR3_EXTI11_PE* = (0x00004000) ## !< PE[11] pin
  SYSCFG_EXTICR3_EXTI11_PF* = (0x00006000) ## !< PF[11] pin
  SYSCFG_EXTICR3_EXTI11_PG* = (0x00007000) ## !< PG[11] pin

## ****************  Bit definition for SYSCFG_EXTICR4 register  ****************

const
  SYSCFG_EXTICR4_EXTI12_Pos* = (0)
  SYSCFG_EXTICR4_EXTI12_Msk* = (0x0000000F shl SYSCFG_EXTICR4_EXTI12_Pos) ## !< 0x0000000F
  SYSCFG_EXTICR4_EXTI12* = SYSCFG_EXTICR4_EXTI12_Msk
  SYSCFG_EXTICR4_EXTI13_Pos* = (4)
  SYSCFG_EXTICR4_EXTI13_Msk* = (0x0000000F shl SYSCFG_EXTICR4_EXTI13_Pos) ## !< 0x000000F0
  SYSCFG_EXTICR4_EXTI13* = SYSCFG_EXTICR4_EXTI13_Msk
  SYSCFG_EXTICR4_EXTI14_Pos* = (8)
  SYSCFG_EXTICR4_EXTI14_Msk* = (0x0000000F shl SYSCFG_EXTICR4_EXTI14_Pos) ## !< 0x00000F00
  SYSCFG_EXTICR4_EXTI14* = SYSCFG_EXTICR4_EXTI14_Msk
  SYSCFG_EXTICR4_EXTI15_Pos* = (12)
  SYSCFG_EXTICR4_EXTI15_Msk* = (0x0000000F shl SYSCFG_EXTICR4_EXTI15_Pos) ## !< 0x0000F000
  SYSCFG_EXTICR4_EXTI15* = SYSCFG_EXTICR4_EXTI15_Msk

## *
##  @brief  EXTI12 configuration
##

const
  SYSCFG_EXTICR4_EXTI12_PA* = (0x00000000) ## !< PA[12] pin
  SYSCFG_EXTICR4_EXTI12_PB* = (0x00000001) ## !< PB[12] pin
  SYSCFG_EXTICR4_EXTI12_PC* = (0x00000002) ## !< PC[12] pin
  SYSCFG_EXTICR4_EXTI12_PD* = (0x00000003) ## !< PD[12] pin
  SYSCFG_EXTICR4_EXTI12_PE* = (0x00000004) ## !< PE[12] pin
  SYSCFG_EXTICR4_EXTI12_PF* = (0x00000006) ## !< PF[12] pin
  SYSCFG_EXTICR4_EXTI12_PG* = (0x00000007) ## !< PG[12] pin

## *
##  @brief  EXTI13 configuration
##

const
  SYSCFG_EXTICR4_EXTI13_PA* = (0x00000000) ## !< PA[13] pin
  SYSCFG_EXTICR4_EXTI13_PB* = (0x00000010) ## !< PB[13] pin
  SYSCFG_EXTICR4_EXTI13_PC* = (0x00000020) ## !< PC[13] pin
  SYSCFG_EXTICR4_EXTI13_PD* = (0x00000030) ## !< PD[13] pin
  SYSCFG_EXTICR4_EXTI13_PE* = (0x00000040) ## !< PE[13] pin
  SYSCFG_EXTICR4_EXTI13_PF* = (0x00000060) ## !< PF[13] pin
  SYSCFG_EXTICR4_EXTI13_PG* = (0x00000070) ## !< PG[13] pin

## *
##  @brief  EXTI14 configuration
##

const
  SYSCFG_EXTICR4_EXTI14_PA* = (0x00000000) ## !< PA[14] pin
  SYSCFG_EXTICR4_EXTI14_PB* = (0x00000100) ## !< PB[14] pin
  SYSCFG_EXTICR4_EXTI14_PC* = (0x00000200) ## !< PC[14] pin
  SYSCFG_EXTICR4_EXTI14_PD* = (0x00000300) ## !< PD[14] pin
  SYSCFG_EXTICR4_EXTI14_PE* = (0x00000400) ## !< PE[14] pin
  SYSCFG_EXTICR4_EXTI14_PF* = (0x00000600) ## !< PF[14] pin
  SYSCFG_EXTICR4_EXTI14_PG* = (0x00000700) ## !< PG[14] pin

## *
##  @brief  EXTI15 configuration
##

const
  SYSCFG_EXTICR4_EXTI15_PA* = (0x00000000) ## !< PA[15] pin
  SYSCFG_EXTICR4_EXTI15_PB* = (0x00001000) ## !< PB[15] pin
  SYSCFG_EXTICR4_EXTI15_PC* = (0x00002000) ## !< PC[15] pin
  SYSCFG_EXTICR4_EXTI15_PD* = (0x00003000) ## !< PD[15] pin
  SYSCFG_EXTICR4_EXTI15_PE* = (0x00004000) ## !< PE[15] pin
  SYSCFG_EXTICR4_EXTI15_PF* = (0x00006000) ## !< PF[15] pin
  SYSCFG_EXTICR4_EXTI15_PG* = (0x00007000) ## !< PG[15] pin

## ****************************************************************************
##
##                        Routing Interface (RI)
##
## ****************************************************************************
## *******************  Bit definition for RI_ICR register  *******************

const
  RI_ICR_IC1OS_Pos* = (0)
  RI_ICR_IC1OS_Msk* = (0x0000000F shl RI_ICR_IC1OS_Pos) ## !< 0x0000000F
  RI_ICR_IC1OS* = RI_ICR_IC1OS_Msk
  RI_ICR_IC1OS_0* = (0x00000001 shl RI_ICR_IC1OS_Pos) ## !< 0x00000001
  RI_ICR_IC1OS_1* = (0x00000002 shl RI_ICR_IC1OS_Pos) ## !< 0x00000002
  RI_ICR_IC1OS_2* = (0x00000004 shl RI_ICR_IC1OS_Pos) ## !< 0x00000004
  RI_ICR_IC1OS_3* = (0x00000008 shl RI_ICR_IC1OS_Pos) ## !< 0x00000008
  RI_ICR_IC2OS_Pos* = (4)
  RI_ICR_IC2OS_Msk* = (0x0000000F shl RI_ICR_IC2OS_Pos) ## !< 0x000000F0
  RI_ICR_IC2OS* = RI_ICR_IC2OS_Msk
  RI_ICR_IC2OS_0* = (0x00000001 shl RI_ICR_IC2OS_Pos) ## !< 0x00000010
  RI_ICR_IC2OS_1* = (0x00000002 shl RI_ICR_IC2OS_Pos) ## !< 0x00000020
  RI_ICR_IC2OS_2* = (0x00000004 shl RI_ICR_IC2OS_Pos) ## !< 0x00000040
  RI_ICR_IC2OS_3* = (0x00000008 shl RI_ICR_IC2OS_Pos) ## !< 0x00000080
  RI_ICR_IC3OS_Pos* = (8)
  RI_ICR_IC3OS_Msk* = (0x0000000F shl RI_ICR_IC3OS_Pos) ## !< 0x00000F00
  RI_ICR_IC3OS* = RI_ICR_IC3OS_Msk
  RI_ICR_IC3OS_0* = (0x00000001 shl RI_ICR_IC3OS_Pos) ## !< 0x00000100
  RI_ICR_IC3OS_1* = (0x00000002 shl RI_ICR_IC3OS_Pos) ## !< 0x00000200
  RI_ICR_IC3OS_2* = (0x00000004 shl RI_ICR_IC3OS_Pos) ## !< 0x00000400
  RI_ICR_IC3OS_3* = (0x00000008 shl RI_ICR_IC3OS_Pos) ## !< 0x00000800
  RI_ICR_IC4OS_Pos* = (12)
  RI_ICR_IC4OS_Msk* = (0x0000000F shl RI_ICR_IC4OS_Pos) ## !< 0x0000F000
  RI_ICR_IC4OS* = RI_ICR_IC4OS_Msk
  RI_ICR_IC4OS_0* = (0x00000001 shl RI_ICR_IC4OS_Pos) ## !< 0x00001000
  RI_ICR_IC4OS_1* = (0x00000002 shl RI_ICR_IC4OS_Pos) ## !< 0x00002000
  RI_ICR_IC4OS_2* = (0x00000004 shl RI_ICR_IC4OS_Pos) ## !< 0x00004000
  RI_ICR_IC4OS_3* = (0x00000008 shl RI_ICR_IC4OS_Pos) ## !< 0x00008000
  RI_ICR_TIM_Pos* = (16)
  RI_ICR_TIM_Msk* = (0x00000003 shl RI_ICR_TIM_Pos) ## !< 0x00030000
  RI_ICR_TIM* = RI_ICR_TIM_Msk
  RI_ICR_TIM_0* = (0x00000001 shl RI_ICR_TIM_Pos) ## !< 0x00010000
  RI_ICR_TIM_1* = (0x00000002 shl RI_ICR_TIM_Pos) ## !< 0x00020000
  RI_ICR_IC1_Pos* = (18)
  RI_ICR_IC1_Msk* = (0x00000001 shl RI_ICR_IC1_Pos) ## !< 0x00040000
  RI_ICR_IC1* = RI_ICR_IC1_Msk
  RI_ICR_IC2_Pos* = (19)
  RI_ICR_IC2_Msk* = (0x00000001 shl RI_ICR_IC2_Pos) ## !< 0x00080000
  RI_ICR_IC2* = RI_ICR_IC2_Msk
  RI_ICR_IC3_Pos* = (20)
  RI_ICR_IC3_Msk* = (0x00000001 shl RI_ICR_IC3_Pos) ## !< 0x00100000
  RI_ICR_IC3* = RI_ICR_IC3_Msk
  RI_ICR_IC4_Pos* = (21)
  RI_ICR_IC4_Msk* = (0x00000001 shl RI_ICR_IC4_Pos) ## !< 0x00200000
  RI_ICR_IC4* = RI_ICR_IC4_Msk

## *******************  Bit definition for RI_ASCR1 register  *******************

const
  RI_ASCR1_CH_Pos* = (0)
  RI_ASCR1_CH_Msk* = (0x7BFDFFFF shl RI_ASCR1_CH_Pos) ## !< 0x7BFDFFFF
  RI_ASCR1_CH* = RI_ASCR1_CH_Msk
  RI_ASCR1_CH_0* = (0x00000001) ## !< Bit 0
  RI_ASCR1_CH_1* = (0x00000002) ## !< Bit 1
  RI_ASCR1_CH_2* = (0x00000004) ## !< Bit 2
  RI_ASCR1_CH_3* = (0x00000008) ## !< Bit 3
  RI_ASCR1_CH_4* = (0x00000010) ## !< Bit 4
  RI_ASCR1_CH_5* = (0x00000020) ## !< Bit 5
  RI_ASCR1_CH_6* = (0x00000040) ## !< Bit 6
  RI_ASCR1_CH_7* = (0x00000080) ## !< Bit 7
  RI_ASCR1_CH_8* = (0x00000100) ## !< Bit 8
  RI_ASCR1_CH_9* = (0x00000200) ## !< Bit 9
  RI_ASCR1_CH_10* = (0x00000400) ## !< Bit 10
  RI_ASCR1_CH_11* = (0x00000800) ## !< Bit 11
  RI_ASCR1_CH_12* = (0x00001000) ## !< Bit 12
  RI_ASCR1_CH_13* = (0x00002000) ## !< Bit 13
  RI_ASCR1_CH_14* = (0x00004000) ## !< Bit 14
  RI_ASCR1_CH_15* = (0x00008000) ## !< Bit 15
  RI_ASCR1_CH_31* = (0x00010000) ## !< Bit 16
  RI_ASCR1_CH_18* = (0x00040000) ## !< Bit 18
  RI_ASCR1_CH_19* = (0x00080000) ## !< Bit 19
  RI_ASCR1_CH_20* = (0x00100000) ## !< Bit 20
  RI_ASCR1_CH_21* = (0x00200000) ## !< Bit 21
  RI_ASCR1_CH_22* = (0x00400000) ## !< Bit 22
  RI_ASCR1_CH_23* = (0x00800000) ## !< Bit 23
  RI_ASCR1_CH_24* = (0x01000000) ## !< Bit 24
  RI_ASCR1_CH_25* = (0x02000000) ## !< Bit 25
  RI_ASCR1_VCOMP_Pos* = (26)
  RI_ASCR1_VCOMP_Msk* = (0x00000001 shl RI_ASCR1_VCOMP_Pos) ## !< 0x04000000
  RI_ASCR1_VCOMP* = RI_ASCR1_VCOMP_Msk
  RI_ASCR1_CH_27* = (0x08000000) ## !< Bit 27
  RI_ASCR1_CH_28* = (0x10000000) ## !< Bit 28
  RI_ASCR1_CH_29* = (0x20000000) ## !< Bit 29
  RI_ASCR1_CH_30* = (0x40000000) ## !< Bit 30
  RI_ASCR1_SCM_Pos* = (31)
  RI_ASCR1_SCM_Msk* = (0x00000001 shl RI_ASCR1_SCM_Pos) ## !< 0x80000000
  RI_ASCR1_SCM* = RI_ASCR1_SCM_Msk

## *******************  Bit definition for RI_ASCR2 register  *******************

const
  RI_ASCR2_GR10_1* = (0x00000001) ## !< GR10-1 selection bit
  RI_ASCR2_GR10_2* = (0x00000002) ## !< GR10-2 selection bit
  RI_ASCR2_GR10_3* = (0x00000004) ## !< GR10-3 selection bit
  RI_ASCR2_GR10_4* = (0x00000008) ## !< GR10-4 selection bit
  RI_ASCR2_GR6_Pos* = (4)
  RI_ASCR2_GR6_Msk* = (0x01800003 shl RI_ASCR2_GR6_Pos) ## !< 0x18000030
  RI_ASCR2_GR6* = RI_ASCR2_GR6_Msk
  RI_ASCR2_GR6_1* = (0x00000001 shl RI_ASCR2_GR6_Pos) ## !< 0x00000010
  RI_ASCR2_GR6_2* = (0x00000002 shl RI_ASCR2_GR6_Pos) ## !< 0x00000020
  RI_ASCR2_GR6_3* = (0x00800000 shl RI_ASCR2_GR6_Pos) ## !< 0x08000000
  RI_ASCR2_GR6_4* = (0x01000000 shl RI_ASCR2_GR6_Pos) ## !< 0x10000000
  RI_ASCR2_GR5_1* = (0x00000040) ## !< GR5-1 selection bit
  RI_ASCR2_GR5_2* = (0x00000080) ## !< GR5-2 selection bit
  RI_ASCR2_GR5_3* = (0x00000100) ## !< GR5-3 selection bit
  RI_ASCR2_GR4_1* = (0x00000200) ## !< GR4-1 selection bit
  RI_ASCR2_GR4_2* = (0x00000400) ## !< GR4-2 selection bit
  RI_ASCR2_GR4_3* = (0x00000800) ## !< GR4-3 selection bit
  RI_ASCR2_GR4_4* = (0x00008000) ## !< GR4-4 selection bit
  RI_ASCR2_CH0b_Pos* = (16)
  RI_ASCR2_CH0b_Msk* = (0x00000001 shl RI_ASCR2_CH0b_Pos) ## !< 0x00010000
  RI_ASCR2_CH0b* = RI_ASCR2_CH0b_Msk
  RI_ASCR2_CH1b_Pos* = (17)
  RI_ASCR2_CH1b_Msk* = (0x00000001 shl RI_ASCR2_CH1b_Pos) ## !< 0x00020000
  RI_ASCR2_CH1b* = RI_ASCR2_CH1b_Msk
  RI_ASCR2_CH2b_Pos* = (18)
  RI_ASCR2_CH2b_Msk* = (0x00000001 shl RI_ASCR2_CH2b_Pos) ## !< 0x00040000
  RI_ASCR2_CH2b* = RI_ASCR2_CH2b_Msk
  RI_ASCR2_CH3b_Pos* = (19)
  RI_ASCR2_CH3b_Msk* = (0x00000001 shl RI_ASCR2_CH3b_Pos) ## !< 0x00080000
  RI_ASCR2_CH3b* = RI_ASCR2_CH3b_Msk
  RI_ASCR2_CH6b_Pos* = (20)
  RI_ASCR2_CH6b_Msk* = (0x00000001 shl RI_ASCR2_CH6b_Pos) ## !< 0x00100000
  RI_ASCR2_CH6b* = RI_ASCR2_CH6b_Msk
  RI_ASCR2_CH7b_Pos* = (21)
  RI_ASCR2_CH7b_Msk* = (0x00000001 shl RI_ASCR2_CH7b_Pos) ## !< 0x00200000
  RI_ASCR2_CH7b* = RI_ASCR2_CH7b_Msk
  RI_ASCR2_CH8b_Pos* = (22)
  RI_ASCR2_CH8b_Msk* = (0x00000001 shl RI_ASCR2_CH8b_Pos) ## !< 0x00400000
  RI_ASCR2_CH8b* = RI_ASCR2_CH8b_Msk
  RI_ASCR2_CH9b_Pos* = (23)
  RI_ASCR2_CH9b_Msk* = (0x00000001 shl RI_ASCR2_CH9b_Pos) ## !< 0x00800000
  RI_ASCR2_CH9b* = RI_ASCR2_CH9b_Msk
  RI_ASCR2_CH10b_Pos* = (24)
  RI_ASCR2_CH10b_Msk* = (0x00000001 shl RI_ASCR2_CH10b_Pos) ## !< 0x01000000
  RI_ASCR2_CH10b* = RI_ASCR2_CH10b_Msk
  RI_ASCR2_CH11b_Pos* = (25)
  RI_ASCR2_CH11b_Msk* = (0x00000001 shl RI_ASCR2_CH11b_Pos) ## !< 0x02000000
  RI_ASCR2_CH11b* = RI_ASCR2_CH11b_Msk
  RI_ASCR2_CH12b_Pos* = (26)
  RI_ASCR2_CH12b_Msk* = (0x00000001 shl RI_ASCR2_CH12b_Pos) ## !< 0x04000000
  RI_ASCR2_CH12b* = RI_ASCR2_CH12b_Msk

## *******************  Bit definition for RI_HYSCR1 register  *******************

const
  RI_HYSCR1_PA_Pos* = (0)
  RI_HYSCR1_PA_Msk* = (0x0000FFFF shl RI_HYSCR1_PA_Pos) ## !< 0x0000FFFF
  RI_HYSCR1_PA* = RI_HYSCR1_PA_Msk
  RI_HYSCR1_PA_0* = (0x00000001 shl RI_HYSCR1_PA_Pos) ## !< 0x00000001
  RI_HYSCR1_PA_1* = (0x00000002 shl RI_HYSCR1_PA_Pos) ## !< 0x00000002
  RI_HYSCR1_PA_2* = (0x00000004 shl RI_HYSCR1_PA_Pos) ## !< 0x00000004
  RI_HYSCR1_PA_3* = (0x00000008 shl RI_HYSCR1_PA_Pos) ## !< 0x00000008
  RI_HYSCR1_PA_4* = (0x00000010 shl RI_HYSCR1_PA_Pos) ## !< 0x00000010
  RI_HYSCR1_PA_5* = (0x00000020 shl RI_HYSCR1_PA_Pos) ## !< 0x00000020
  RI_HYSCR1_PA_6* = (0x00000040 shl RI_HYSCR1_PA_Pos) ## !< 0x00000040
  RI_HYSCR1_PA_7* = (0x00000080 shl RI_HYSCR1_PA_Pos) ## !< 0x00000080
  RI_HYSCR1_PA_8* = (0x00000100 shl RI_HYSCR1_PA_Pos) ## !< 0x00000100
  RI_HYSCR1_PA_9* = (0x00000200 shl RI_HYSCR1_PA_Pos) ## !< 0x00000200
  RI_HYSCR1_PA_10* = (0x00000400 shl RI_HYSCR1_PA_Pos) ## !< 0x00000400
  RI_HYSCR1_PA_11* = (0x00000800 shl RI_HYSCR1_PA_Pos) ## !< 0x00000800
  RI_HYSCR1_PA_12* = (0x00001000 shl RI_HYSCR1_PA_Pos) ## !< 0x00001000
  RI_HYSCR1_PA_13* = (0x00002000 shl RI_HYSCR1_PA_Pos) ## !< 0x00002000
  RI_HYSCR1_PA_14* = (0x00004000 shl RI_HYSCR1_PA_Pos) ## !< 0x00004000
  RI_HYSCR1_PA_15* = (0x00008000 shl RI_HYSCR1_PA_Pos) ## !< 0x00008000
  RI_HYSCR1_PB_Pos* = (16)
  RI_HYSCR1_PB_Msk* = (0x0000FFFF shl RI_HYSCR1_PB_Pos) ## !< 0xFFFF0000
  RI_HYSCR1_PB* = RI_HYSCR1_PB_Msk
  RI_HYSCR1_PB_0* = (0x00000001 shl RI_HYSCR1_PB_Pos) ## !< 0x00010000
  RI_HYSCR1_PB_1* = (0x00000002 shl RI_HYSCR1_PB_Pos) ## !< 0x00020000
  RI_HYSCR1_PB_2* = (0x00000004 shl RI_HYSCR1_PB_Pos) ## !< 0x00040000
  RI_HYSCR1_PB_3* = (0x00000008 shl RI_HYSCR1_PB_Pos) ## !< 0x00080000
  RI_HYSCR1_PB_4* = (0x00000010 shl RI_HYSCR1_PB_Pos) ## !< 0x00100000
  RI_HYSCR1_PB_5* = (0x00000020 shl RI_HYSCR1_PB_Pos) ## !< 0x00200000
  RI_HYSCR1_PB_6* = (0x00000040 shl RI_HYSCR1_PB_Pos) ## !< 0x00400000
  RI_HYSCR1_PB_7* = (0x00000080 shl RI_HYSCR1_PB_Pos) ## !< 0x00800000
  RI_HYSCR1_PB_8* = (0x00000100 shl RI_HYSCR1_PB_Pos) ## !< 0x01000000
  RI_HYSCR1_PB_9* = (0x00000200 shl RI_HYSCR1_PB_Pos) ## !< 0x02000000
  RI_HYSCR1_PB_10* = (0x00000400 shl RI_HYSCR1_PB_Pos) ## !< 0x04000000
  RI_HYSCR1_PB_11* = (0x00000800 shl RI_HYSCR1_PB_Pos) ## !< 0x08000000
  RI_HYSCR1_PB_12* = (0x00001000 shl RI_HYSCR1_PB_Pos) ## !< 0x10000000
  RI_HYSCR1_PB_13* = (0x00002000 shl RI_HYSCR1_PB_Pos) ## !< 0x20000000
  RI_HYSCR1_PB_14* = (0x00004000 shl RI_HYSCR1_PB_Pos) ## !< 0x40000000
  RI_HYSCR1_PB_15* = (0x00008000 shl RI_HYSCR1_PB_Pos) ## !< 0x80000000

## *******************  Bit definition for RI_HYSCR2 register  *******************

const
  RI_HYSCR2_PC_Pos* = (0)
  RI_HYSCR2_PC_Msk* = (0x0000FFFF shl RI_HYSCR2_PC_Pos) ## !< 0x0000FFFF
  RI_HYSCR2_PC* = RI_HYSCR2_PC_Msk
  RI_HYSCR2_PC_0* = (0x00000001 shl RI_HYSCR2_PC_Pos) ## !< 0x00000001
  RI_HYSCR2_PC_1* = (0x00000002 shl RI_HYSCR2_PC_Pos) ## !< 0x00000002
  RI_HYSCR2_PC_2* = (0x00000004 shl RI_HYSCR2_PC_Pos) ## !< 0x00000004
  RI_HYSCR2_PC_3* = (0x00000008 shl RI_HYSCR2_PC_Pos) ## !< 0x00000008
  RI_HYSCR2_PC_4* = (0x00000010 shl RI_HYSCR2_PC_Pos) ## !< 0x00000010
  RI_HYSCR2_PC_5* = (0x00000020 shl RI_HYSCR2_PC_Pos) ## !< 0x00000020
  RI_HYSCR2_PC_6* = (0x00000040 shl RI_HYSCR2_PC_Pos) ## !< 0x00000040
  RI_HYSCR2_PC_7* = (0x00000080 shl RI_HYSCR2_PC_Pos) ## !< 0x00000080
  RI_HYSCR2_PC_8* = (0x00000100 shl RI_HYSCR2_PC_Pos) ## !< 0x00000100
  RI_HYSCR2_PC_9* = (0x00000200 shl RI_HYSCR2_PC_Pos) ## !< 0x00000200
  RI_HYSCR2_PC_10* = (0x00000400 shl RI_HYSCR2_PC_Pos) ## !< 0x00000400
  RI_HYSCR2_PC_11* = (0x00000800 shl RI_HYSCR2_PC_Pos) ## !< 0x00000800
  RI_HYSCR2_PC_12* = (0x00001000 shl RI_HYSCR2_PC_Pos) ## !< 0x00001000
  RI_HYSCR2_PC_13* = (0x00002000 shl RI_HYSCR2_PC_Pos) ## !< 0x00002000
  RI_HYSCR2_PC_14* = (0x00004000 shl RI_HYSCR2_PC_Pos) ## !< 0x00004000
  RI_HYSCR2_PC_15* = (0x00008000 shl RI_HYSCR2_PC_Pos) ## !< 0x00008000
  RI_HYSCR2_PD_Pos* = (16)
  RI_HYSCR2_PD_Msk* = (0x0000FFFF shl RI_HYSCR2_PD_Pos) ## !< 0xFFFF0000
  RI_HYSCR2_PD* = RI_HYSCR2_PD_Msk
  RI_HYSCR2_PD_0* = (0x00000001 shl RI_HYSCR2_PD_Pos) ## !< 0x00010000
  RI_HYSCR2_PD_1* = (0x00000002 shl RI_HYSCR2_PD_Pos) ## !< 0x00020000
  RI_HYSCR2_PD_2* = (0x00000004 shl RI_HYSCR2_PD_Pos) ## !< 0x00040000
  RI_HYSCR2_PD_3* = (0x00000008 shl RI_HYSCR2_PD_Pos) ## !< 0x00080000
  RI_HYSCR2_PD_4* = (0x00000010 shl RI_HYSCR2_PD_Pos) ## !< 0x00100000
  RI_HYSCR2_PD_5* = (0x00000020 shl RI_HYSCR2_PD_Pos) ## !< 0x00200000
  RI_HYSCR2_PD_6* = (0x00000040 shl RI_HYSCR2_PD_Pos) ## !< 0x00400000
  RI_HYSCR2_PD_7* = (0x00000080 shl RI_HYSCR2_PD_Pos) ## !< 0x00800000
  RI_HYSCR2_PD_8* = (0x00000100 shl RI_HYSCR2_PD_Pos) ## !< 0x01000000
  RI_HYSCR2_PD_9* = (0x00000200 shl RI_HYSCR2_PD_Pos) ## !< 0x02000000
  RI_HYSCR2_PD_10* = (0x00000400 shl RI_HYSCR2_PD_Pos) ## !< 0x04000000
  RI_HYSCR2_PD_11* = (0x00000800 shl RI_HYSCR2_PD_Pos) ## !< 0x08000000
  RI_HYSCR2_PD_12* = (0x00001000 shl RI_HYSCR2_PD_Pos) ## !< 0x10000000
  RI_HYSCR2_PD_13* = (0x00002000 shl RI_HYSCR2_PD_Pos) ## !< 0x20000000
  RI_HYSCR2_PD_14* = (0x00004000 shl RI_HYSCR2_PD_Pos) ## !< 0x40000000
  RI_HYSCR2_PD_15* = (0x00008000 shl RI_HYSCR2_PD_Pos) ## !< 0x80000000

## *******************  Bit definition for RI_HYSCR3 register  *******************

const
  RI_HYSCR3_PE_Pos* = (0)
  RI_HYSCR3_PE_Msk* = (0x0000FFFF shl RI_HYSCR3_PE_Pos) ## !< 0x0000FFFF
  RI_HYSCR3_PE* = RI_HYSCR3_PE_Msk
  RI_HYSCR3_PE_0* = (0x00000001 shl RI_HYSCR3_PE_Pos) ## !< 0x00000001
  RI_HYSCR3_PE_1* = (0x00000002 shl RI_HYSCR3_PE_Pos) ## !< 0x00000002
  RI_HYSCR3_PE_2* = (0x00000004 shl RI_HYSCR3_PE_Pos) ## !< 0x00000004
  RI_HYSCR3_PE_3* = (0x00000008 shl RI_HYSCR3_PE_Pos) ## !< 0x00000008
  RI_HYSCR3_PE_4* = (0x00000010 shl RI_HYSCR3_PE_Pos) ## !< 0x00000010
  RI_HYSCR3_PE_5* = (0x00000020 shl RI_HYSCR3_PE_Pos) ## !< 0x00000020
  RI_HYSCR3_PE_6* = (0x00000040 shl RI_HYSCR3_PE_Pos) ## !< 0x00000040
  RI_HYSCR3_PE_7* = (0x00000080 shl RI_HYSCR3_PE_Pos) ## !< 0x00000080
  RI_HYSCR3_PE_8* = (0x00000100 shl RI_HYSCR3_PE_Pos) ## !< 0x00000100
  RI_HYSCR3_PE_9* = (0x00000200 shl RI_HYSCR3_PE_Pos) ## !< 0x00000200
  RI_HYSCR3_PE_10* = (0x00000400 shl RI_HYSCR3_PE_Pos) ## !< 0x00000400
  RI_HYSCR3_PE_11* = (0x00000800 shl RI_HYSCR3_PE_Pos) ## !< 0x00000800
  RI_HYSCR3_PE_12* = (0x00001000 shl RI_HYSCR3_PE_Pos) ## !< 0x00001000
  RI_HYSCR3_PE_13* = (0x00002000 shl RI_HYSCR3_PE_Pos) ## !< 0x00002000
  RI_HYSCR3_PE_14* = (0x00004000 shl RI_HYSCR3_PE_Pos) ## !< 0x00004000
  RI_HYSCR3_PE_15* = (0x00008000 shl RI_HYSCR3_PE_Pos) ## !< 0x00008000
  RI_HYSCR3_PF_Pos* = (16)
  RI_HYSCR3_PF_Msk* = (0x0000FFFF shl RI_HYSCR3_PF_Pos) ## !< 0xFFFF0000
  RI_HYSCR3_PF* = RI_HYSCR3_PF_Msk
  RI_HYSCR3_PF_0* = (0x00000001 shl RI_HYSCR3_PF_Pos) ## !< 0x00010000
  RI_HYSCR3_PF_1* = (0x00000002 shl RI_HYSCR3_PF_Pos) ## !< 0x00020000
  RI_HYSCR3_PF_2* = (0x00000004 shl RI_HYSCR3_PF_Pos) ## !< 0x00040000
  RI_HYSCR3_PF_3* = (0x00000008 shl RI_HYSCR3_PF_Pos) ## !< 0x00080000
  RI_HYSCR3_PF_4* = (0x00000010 shl RI_HYSCR3_PF_Pos) ## !< 0x00100000
  RI_HYSCR3_PF_5* = (0x00000020 shl RI_HYSCR3_PF_Pos) ## !< 0x00200000
  RI_HYSCR3_PF_6* = (0x00000040 shl RI_HYSCR3_PF_Pos) ## !< 0x00400000
  RI_HYSCR3_PF_7* = (0x00000080 shl RI_HYSCR3_PF_Pos) ## !< 0x00800000
  RI_HYSCR3_PF_8* = (0x00000100 shl RI_HYSCR3_PF_Pos) ## !< 0x01000000
  RI_HYSCR3_PF_9* = (0x00000200 shl RI_HYSCR3_PF_Pos) ## !< 0x02000000
  RI_HYSCR3_PF_10* = (0x00000400 shl RI_HYSCR3_PF_Pos) ## !< 0x04000000
  RI_HYSCR3_PF_11* = (0x00000800 shl RI_HYSCR3_PF_Pos) ## !< 0x08000000
  RI_HYSCR3_PF_12* = (0x00001000 shl RI_HYSCR3_PF_Pos) ## !< 0x10000000
  RI_HYSCR3_PF_13* = (0x00002000 shl RI_HYSCR3_PF_Pos) ## !< 0x20000000
  RI_HYSCR3_PF_14* = (0x00004000 shl RI_HYSCR3_PF_Pos) ## !< 0x40000000
  RI_HYSCR3_PF_15* = (0x00008000 shl RI_HYSCR3_PF_Pos) ## !< 0x80000000

## *******************  Bit definition for RI_HYSCR4 register  *******************

const
  RI_HYSCR4_PG_Pos* = (0)
  RI_HYSCR4_PG_Msk* = (0x0000FFFF shl RI_HYSCR4_PG_Pos) ## !< 0x0000FFFF
  RI_HYSCR4_PG* = RI_HYSCR4_PG_Msk
  RI_HYSCR4_PG_0* = (0x00000001 shl RI_HYSCR4_PG_Pos) ## !< 0x00000001
  RI_HYSCR4_PG_1* = (0x00000002 shl RI_HYSCR4_PG_Pos) ## !< 0x00000002
  RI_HYSCR4_PG_2* = (0x00000004 shl RI_HYSCR4_PG_Pos) ## !< 0x00000004
  RI_HYSCR4_PG_3* = (0x00000008 shl RI_HYSCR4_PG_Pos) ## !< 0x00000008
  RI_HYSCR4_PG_4* = (0x00000010 shl RI_HYSCR4_PG_Pos) ## !< 0x00000010
  RI_HYSCR4_PG_5* = (0x00000020 shl RI_HYSCR4_PG_Pos) ## !< 0x00000020
  RI_HYSCR4_PG_6* = (0x00000040 shl RI_HYSCR4_PG_Pos) ## !< 0x00000040
  RI_HYSCR4_PG_7* = (0x00000080 shl RI_HYSCR4_PG_Pos) ## !< 0x00000080
  RI_HYSCR4_PG_8* = (0x00000100 shl RI_HYSCR4_PG_Pos) ## !< 0x00000100
  RI_HYSCR4_PG_9* = (0x00000200 shl RI_HYSCR4_PG_Pos) ## !< 0x00000200
  RI_HYSCR4_PG_10* = (0x00000400 shl RI_HYSCR4_PG_Pos) ## !< 0x00000400
  RI_HYSCR4_PG_11* = (0x00000800 shl RI_HYSCR4_PG_Pos) ## !< 0x00000800
  RI_HYSCR4_PG_12* = (0x00001000 shl RI_HYSCR4_PG_Pos) ## !< 0x00001000
  RI_HYSCR4_PG_13* = (0x00002000 shl RI_HYSCR4_PG_Pos) ## !< 0x00002000
  RI_HYSCR4_PG_14* = (0x00004000 shl RI_HYSCR4_PG_Pos) ## !< 0x00004000
  RI_HYSCR4_PG_15* = (0x00008000 shl RI_HYSCR4_PG_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_ASMR1 register  *******************

const
  RI_ASMR1_PA_Pos* = (0)
  RI_ASMR1_PA_Msk* = (0x0000FFFF shl RI_ASMR1_PA_Pos) ## !< 0x0000FFFF
  RI_ASMR1_PA* = RI_ASMR1_PA_Msk
  RI_ASMR1_PA_0* = (0x00000001 shl RI_ASMR1_PA_Pos) ## !< 0x00000001
  RI_ASMR1_PA_1* = (0x00000002 shl RI_ASMR1_PA_Pos) ## !< 0x00000002
  RI_ASMR1_PA_2* = (0x00000004 shl RI_ASMR1_PA_Pos) ## !< 0x00000004
  RI_ASMR1_PA_3* = (0x00000008 shl RI_ASMR1_PA_Pos) ## !< 0x00000008
  RI_ASMR1_PA_4* = (0x00000010 shl RI_ASMR1_PA_Pos) ## !< 0x00000010
  RI_ASMR1_PA_5* = (0x00000020 shl RI_ASMR1_PA_Pos) ## !< 0x00000020
  RI_ASMR1_PA_6* = (0x00000040 shl RI_ASMR1_PA_Pos) ## !< 0x00000040
  RI_ASMR1_PA_7* = (0x00000080 shl RI_ASMR1_PA_Pos) ## !< 0x00000080
  RI_ASMR1_PA_8* = (0x00000100 shl RI_ASMR1_PA_Pos) ## !< 0x00000100
  RI_ASMR1_PA_9* = (0x00000200 shl RI_ASMR1_PA_Pos) ## !< 0x00000200
  RI_ASMR1_PA_10* = (0x00000400 shl RI_ASMR1_PA_Pos) ## !< 0x00000400
  RI_ASMR1_PA_11* = (0x00000800 shl RI_ASMR1_PA_Pos) ## !< 0x00000800
  RI_ASMR1_PA_12* = (0x00001000 shl RI_ASMR1_PA_Pos) ## !< 0x00001000
  RI_ASMR1_PA_13* = (0x00002000 shl RI_ASMR1_PA_Pos) ## !< 0x00002000
  RI_ASMR1_PA_14* = (0x00004000 shl RI_ASMR1_PA_Pos) ## !< 0x00004000
  RI_ASMR1_PA_15* = (0x00008000 shl RI_ASMR1_PA_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_CMR1 register  *******************

const
  RI_CMR1_PA_Pos* = (0)
  RI_CMR1_PA_Msk* = (0x0000FFFF shl RI_CMR1_PA_Pos) ## !< 0x0000FFFF
  RI_CMR1_PA* = RI_CMR1_PA_Msk
  RI_CMR1_PA_0* = (0x00000001 shl RI_CMR1_PA_Pos) ## !< 0x00000001
  RI_CMR1_PA_1* = (0x00000002 shl RI_CMR1_PA_Pos) ## !< 0x00000002
  RI_CMR1_PA_2* = (0x00000004 shl RI_CMR1_PA_Pos) ## !< 0x00000004
  RI_CMR1_PA_3* = (0x00000008 shl RI_CMR1_PA_Pos) ## !< 0x00000008
  RI_CMR1_PA_4* = (0x00000010 shl RI_CMR1_PA_Pos) ## !< 0x00000010
  RI_CMR1_PA_5* = (0x00000020 shl RI_CMR1_PA_Pos) ## !< 0x00000020
  RI_CMR1_PA_6* = (0x00000040 shl RI_CMR1_PA_Pos) ## !< 0x00000040
  RI_CMR1_PA_7* = (0x00000080 shl RI_CMR1_PA_Pos) ## !< 0x00000080
  RI_CMR1_PA_8* = (0x00000100 shl RI_CMR1_PA_Pos) ## !< 0x00000100
  RI_CMR1_PA_9* = (0x00000200 shl RI_CMR1_PA_Pos) ## !< 0x00000200
  RI_CMR1_PA_10* = (0x00000400 shl RI_CMR1_PA_Pos) ## !< 0x00000400
  RI_CMR1_PA_11* = (0x00000800 shl RI_CMR1_PA_Pos) ## !< 0x00000800
  RI_CMR1_PA_12* = (0x00001000 shl RI_CMR1_PA_Pos) ## !< 0x00001000
  RI_CMR1_PA_13* = (0x00002000 shl RI_CMR1_PA_Pos) ## !< 0x00002000
  RI_CMR1_PA_14* = (0x00004000 shl RI_CMR1_PA_Pos) ## !< 0x00004000
  RI_CMR1_PA_15* = (0x00008000 shl RI_CMR1_PA_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_CICR1 register  *******************

const
  RI_CICR1_PA_Pos* = (0)
  RI_CICR1_PA_Msk* = (0x0000FFFF shl RI_CICR1_PA_Pos) ## !< 0x0000FFFF
  RI_CICR1_PA* = RI_CICR1_PA_Msk
  RI_CICR1_PA_0* = (0x00000001 shl RI_CICR1_PA_Pos) ## !< 0x00000001
  RI_CICR1_PA_1* = (0x00000002 shl RI_CICR1_PA_Pos) ## !< 0x00000002
  RI_CICR1_PA_2* = (0x00000004 shl RI_CICR1_PA_Pos) ## !< 0x00000004
  RI_CICR1_PA_3* = (0x00000008 shl RI_CICR1_PA_Pos) ## !< 0x00000008
  RI_CICR1_PA_4* = (0x00000010 shl RI_CICR1_PA_Pos) ## !< 0x00000010
  RI_CICR1_PA_5* = (0x00000020 shl RI_CICR1_PA_Pos) ## !< 0x00000020
  RI_CICR1_PA_6* = (0x00000040 shl RI_CICR1_PA_Pos) ## !< 0x00000040
  RI_CICR1_PA_7* = (0x00000080 shl RI_CICR1_PA_Pos) ## !< 0x00000080
  RI_CICR1_PA_8* = (0x00000100 shl RI_CICR1_PA_Pos) ## !< 0x00000100
  RI_CICR1_PA_9* = (0x00000200 shl RI_CICR1_PA_Pos) ## !< 0x00000200
  RI_CICR1_PA_10* = (0x00000400 shl RI_CICR1_PA_Pos) ## !< 0x00000400
  RI_CICR1_PA_11* = (0x00000800 shl RI_CICR1_PA_Pos) ## !< 0x00000800
  RI_CICR1_PA_12* = (0x00001000 shl RI_CICR1_PA_Pos) ## !< 0x00001000
  RI_CICR1_PA_13* = (0x00002000 shl RI_CICR1_PA_Pos) ## !< 0x00002000
  RI_CICR1_PA_14* = (0x00004000 shl RI_CICR1_PA_Pos) ## !< 0x00004000
  RI_CICR1_PA_15* = (0x00008000 shl RI_CICR1_PA_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_ASMR2 register  *******************

const
  RI_ASMR2_PB_Pos* = (0)
  RI_ASMR2_PB_Msk* = (0x0000FFFF shl RI_ASMR2_PB_Pos) ## !< 0x0000FFFF
  RI_ASMR2_PB* = RI_ASMR2_PB_Msk
  RI_ASMR2_PB_0* = (0x00000001 shl RI_ASMR2_PB_Pos) ## !< 0x00000001
  RI_ASMR2_PB_1* = (0x00000002 shl RI_ASMR2_PB_Pos) ## !< 0x00000002
  RI_ASMR2_PB_2* = (0x00000004 shl RI_ASMR2_PB_Pos) ## !< 0x00000004
  RI_ASMR2_PB_3* = (0x00000008 shl RI_ASMR2_PB_Pos) ## !< 0x00000008
  RI_ASMR2_PB_4* = (0x00000010 shl RI_ASMR2_PB_Pos) ## !< 0x00000010
  RI_ASMR2_PB_5* = (0x00000020 shl RI_ASMR2_PB_Pos) ## !< 0x00000020
  RI_ASMR2_PB_6* = (0x00000040 shl RI_ASMR2_PB_Pos) ## !< 0x00000040
  RI_ASMR2_PB_7* = (0x00000080 shl RI_ASMR2_PB_Pos) ## !< 0x00000080
  RI_ASMR2_PB_8* = (0x00000100 shl RI_ASMR2_PB_Pos) ## !< 0x00000100
  RI_ASMR2_PB_9* = (0x00000200 shl RI_ASMR2_PB_Pos) ## !< 0x00000200
  RI_ASMR2_PB_10* = (0x00000400 shl RI_ASMR2_PB_Pos) ## !< 0x00000400
  RI_ASMR2_PB_11* = (0x00000800 shl RI_ASMR2_PB_Pos) ## !< 0x00000800
  RI_ASMR2_PB_12* = (0x00001000 shl RI_ASMR2_PB_Pos) ## !< 0x00001000
  RI_ASMR2_PB_13* = (0x00002000 shl RI_ASMR2_PB_Pos) ## !< 0x00002000
  RI_ASMR2_PB_14* = (0x00004000 shl RI_ASMR2_PB_Pos) ## !< 0x00004000
  RI_ASMR2_PB_15* = (0x00008000 shl RI_ASMR2_PB_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_CMR2 register  *******************

const
  RI_CMR2_PB_Pos* = (0)
  RI_CMR2_PB_Msk* = (0x0000FFFF shl RI_CMR2_PB_Pos) ## !< 0x0000FFFF
  RI_CMR2_PB* = RI_CMR2_PB_Msk
  RI_CMR2_PB_0* = (0x00000001 shl RI_CMR2_PB_Pos) ## !< 0x00000001
  RI_CMR2_PB_1* = (0x00000002 shl RI_CMR2_PB_Pos) ## !< 0x00000002
  RI_CMR2_PB_2* = (0x00000004 shl RI_CMR2_PB_Pos) ## !< 0x00000004
  RI_CMR2_PB_3* = (0x00000008 shl RI_CMR2_PB_Pos) ## !< 0x00000008
  RI_CMR2_PB_4* = (0x00000010 shl RI_CMR2_PB_Pos) ## !< 0x00000010
  RI_CMR2_PB_5* = (0x00000020 shl RI_CMR2_PB_Pos) ## !< 0x00000020
  RI_CMR2_PB_6* = (0x00000040 shl RI_CMR2_PB_Pos) ## !< 0x00000040
  RI_CMR2_PB_7* = (0x00000080 shl RI_CMR2_PB_Pos) ## !< 0x00000080
  RI_CMR2_PB_8* = (0x00000100 shl RI_CMR2_PB_Pos) ## !< 0x00000100
  RI_CMR2_PB_9* = (0x00000200 shl RI_CMR2_PB_Pos) ## !< 0x00000200
  RI_CMR2_PB_10* = (0x00000400 shl RI_CMR2_PB_Pos) ## !< 0x00000400
  RI_CMR2_PB_11* = (0x00000800 shl RI_CMR2_PB_Pos) ## !< 0x00000800
  RI_CMR2_PB_12* = (0x00001000 shl RI_CMR2_PB_Pos) ## !< 0x00001000
  RI_CMR2_PB_13* = (0x00002000 shl RI_CMR2_PB_Pos) ## !< 0x00002000
  RI_CMR2_PB_14* = (0x00004000 shl RI_CMR2_PB_Pos) ## !< 0x00004000
  RI_CMR2_PB_15* = (0x00008000 shl RI_CMR2_PB_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_CICR2 register  *******************

const
  RI_CICR2_PB_Pos* = (0)
  RI_CICR2_PB_Msk* = (0x0000FFFF shl RI_CICR2_PB_Pos) ## !< 0x0000FFFF
  RI_CICR2_PB* = RI_CICR2_PB_Msk
  RI_CICR2_PB_0* = (0x00000001 shl RI_CICR2_PB_Pos) ## !< 0x00000001
  RI_CICR2_PB_1* = (0x00000002 shl RI_CICR2_PB_Pos) ## !< 0x00000002
  RI_CICR2_PB_2* = (0x00000004 shl RI_CICR2_PB_Pos) ## !< 0x00000004
  RI_CICR2_PB_3* = (0x00000008 shl RI_CICR2_PB_Pos) ## !< 0x00000008
  RI_CICR2_PB_4* = (0x00000010 shl RI_CICR2_PB_Pos) ## !< 0x00000010
  RI_CICR2_PB_5* = (0x00000020 shl RI_CICR2_PB_Pos) ## !< 0x00000020
  RI_CICR2_PB_6* = (0x00000040 shl RI_CICR2_PB_Pos) ## !< 0x00000040
  RI_CICR2_PB_7* = (0x00000080 shl RI_CICR2_PB_Pos) ## !< 0x00000080
  RI_CICR2_PB_8* = (0x00000100 shl RI_CICR2_PB_Pos) ## !< 0x00000100
  RI_CICR2_PB_9* = (0x00000200 shl RI_CICR2_PB_Pos) ## !< 0x00000200
  RI_CICR2_PB_10* = (0x00000400 shl RI_CICR2_PB_Pos) ## !< 0x00000400
  RI_CICR2_PB_11* = (0x00000800 shl RI_CICR2_PB_Pos) ## !< 0x00000800
  RI_CICR2_PB_12* = (0x00001000 shl RI_CICR2_PB_Pos) ## !< 0x00001000
  RI_CICR2_PB_13* = (0x00002000 shl RI_CICR2_PB_Pos) ## !< 0x00002000
  RI_CICR2_PB_14* = (0x00004000 shl RI_CICR2_PB_Pos) ## !< 0x00004000
  RI_CICR2_PB_15* = (0x00008000 shl RI_CICR2_PB_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_ASMR3 register  *******************

const
  RI_ASMR3_PC_Pos* = (0)
  RI_ASMR3_PC_Msk* = (0x0000FFFF shl RI_ASMR3_PC_Pos) ## !< 0x0000FFFF
  RI_ASMR3_PC* = RI_ASMR3_PC_Msk
  RI_ASMR3_PC_0* = (0x00000001 shl RI_ASMR3_PC_Pos) ## !< 0x00000001
  RI_ASMR3_PC_1* = (0x00000002 shl RI_ASMR3_PC_Pos) ## !< 0x00000002
  RI_ASMR3_PC_2* = (0x00000004 shl RI_ASMR3_PC_Pos) ## !< 0x00000004
  RI_ASMR3_PC_3* = (0x00000008 shl RI_ASMR3_PC_Pos) ## !< 0x00000008
  RI_ASMR3_PC_4* = (0x00000010 shl RI_ASMR3_PC_Pos) ## !< 0x00000010
  RI_ASMR3_PC_5* = (0x00000020 shl RI_ASMR3_PC_Pos) ## !< 0x00000020
  RI_ASMR3_PC_6* = (0x00000040 shl RI_ASMR3_PC_Pos) ## !< 0x00000040
  RI_ASMR3_PC_7* = (0x00000080 shl RI_ASMR3_PC_Pos) ## !< 0x00000080
  RI_ASMR3_PC_8* = (0x00000100 shl RI_ASMR3_PC_Pos) ## !< 0x00000100
  RI_ASMR3_PC_9* = (0x00000200 shl RI_ASMR3_PC_Pos) ## !< 0x00000200
  RI_ASMR3_PC_10* = (0x00000400 shl RI_ASMR3_PC_Pos) ## !< 0x00000400
  RI_ASMR3_PC_11* = (0x00000800 shl RI_ASMR3_PC_Pos) ## !< 0x00000800
  RI_ASMR3_PC_12* = (0x00001000 shl RI_ASMR3_PC_Pos) ## !< 0x00001000
  RI_ASMR3_PC_13* = (0x00002000 shl RI_ASMR3_PC_Pos) ## !< 0x00002000
  RI_ASMR3_PC_14* = (0x00004000 shl RI_ASMR3_PC_Pos) ## !< 0x00004000
  RI_ASMR3_PC_15* = (0x00008000 shl RI_ASMR3_PC_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_CMR3 register  *******************

const
  RI_CMR3_PC_Pos* = (0)
  RI_CMR3_PC_Msk* = (0x0000FFFF shl RI_CMR3_PC_Pos) ## !< 0x0000FFFF
  RI_CMR3_PC* = RI_CMR3_PC_Msk
  RI_CMR3_PC_0* = (0x00000001 shl RI_CMR3_PC_Pos) ## !< 0x00000001
  RI_CMR3_PC_1* = (0x00000002 shl RI_CMR3_PC_Pos) ## !< 0x00000002
  RI_CMR3_PC_2* = (0x00000004 shl RI_CMR3_PC_Pos) ## !< 0x00000004
  RI_CMR3_PC_3* = (0x00000008 shl RI_CMR3_PC_Pos) ## !< 0x00000008
  RI_CMR3_PC_4* = (0x00000010 shl RI_CMR3_PC_Pos) ## !< 0x00000010
  RI_CMR3_PC_5* = (0x00000020 shl RI_CMR3_PC_Pos) ## !< 0x00000020
  RI_CMR3_PC_6* = (0x00000040 shl RI_CMR3_PC_Pos) ## !< 0x00000040
  RI_CMR3_PC_7* = (0x00000080 shl RI_CMR3_PC_Pos) ## !< 0x00000080
  RI_CMR3_PC_8* = (0x00000100 shl RI_CMR3_PC_Pos) ## !< 0x00000100
  RI_CMR3_PC_9* = (0x00000200 shl RI_CMR3_PC_Pos) ## !< 0x00000200
  RI_CMR3_PC_10* = (0x00000400 shl RI_CMR3_PC_Pos) ## !< 0x00000400
  RI_CMR3_PC_11* = (0x00000800 shl RI_CMR3_PC_Pos) ## !< 0x00000800
  RI_CMR3_PC_12* = (0x00001000 shl RI_CMR3_PC_Pos) ## !< 0x00001000
  RI_CMR3_PC_13* = (0x00002000 shl RI_CMR3_PC_Pos) ## !< 0x00002000
  RI_CMR3_PC_14* = (0x00004000 shl RI_CMR3_PC_Pos) ## !< 0x00004000
  RI_CMR3_PC_15* = (0x00008000 shl RI_CMR3_PC_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_CICR3 register  *******************

const
  RI_CICR3_PC_Pos* = (0)
  RI_CICR3_PC_Msk* = (0x0000FFFF shl RI_CICR3_PC_Pos) ## !< 0x0000FFFF
  RI_CICR3_PC* = RI_CICR3_PC_Msk
  RI_CICR3_PC_0* = (0x00000001 shl RI_CICR3_PC_Pos) ## !< 0x00000001
  RI_CICR3_PC_1* = (0x00000002 shl RI_CICR3_PC_Pos) ## !< 0x00000002
  RI_CICR3_PC_2* = (0x00000004 shl RI_CICR3_PC_Pos) ## !< 0x00000004
  RI_CICR3_PC_3* = (0x00000008 shl RI_CICR3_PC_Pos) ## !< 0x00000008
  RI_CICR3_PC_4* = (0x00000010 shl RI_CICR3_PC_Pos) ## !< 0x00000010
  RI_CICR3_PC_5* = (0x00000020 shl RI_CICR3_PC_Pos) ## !< 0x00000020
  RI_CICR3_PC_6* = (0x00000040 shl RI_CICR3_PC_Pos) ## !< 0x00000040
  RI_CICR3_PC_7* = (0x00000080 shl RI_CICR3_PC_Pos) ## !< 0x00000080
  RI_CICR3_PC_8* = (0x00000100 shl RI_CICR3_PC_Pos) ## !< 0x00000100
  RI_CICR3_PC_9* = (0x00000200 shl RI_CICR3_PC_Pos) ## !< 0x00000200
  RI_CICR3_PC_10* = (0x00000400 shl RI_CICR3_PC_Pos) ## !< 0x00000400
  RI_CICR3_PC_11* = (0x00000800 shl RI_CICR3_PC_Pos) ## !< 0x00000800
  RI_CICR3_PC_12* = (0x00001000 shl RI_CICR3_PC_Pos) ## !< 0x00001000
  RI_CICR3_PC_13* = (0x00002000 shl RI_CICR3_PC_Pos) ## !< 0x00002000
  RI_CICR3_PC_14* = (0x00004000 shl RI_CICR3_PC_Pos) ## !< 0x00004000
  RI_CICR3_PC_15* = (0x00008000 shl RI_CICR3_PC_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_ASMR4 register  *******************

const
  RI_ASMR4_PF_Pos* = (0)
  RI_ASMR4_PF_Msk* = (0x0000FFFF shl RI_ASMR4_PF_Pos) ## !< 0x0000FFFF
  RI_ASMR4_PF* = RI_ASMR4_PF_Msk
  RI_ASMR4_PF_0* = (0x00000001 shl RI_ASMR4_PF_Pos) ## !< 0x00000001
  RI_ASMR4_PF_1* = (0x00000002 shl RI_ASMR4_PF_Pos) ## !< 0x00000002
  RI_ASMR4_PF_2* = (0x00000004 shl RI_ASMR4_PF_Pos) ## !< 0x00000004
  RI_ASMR4_PF_3* = (0x00000008 shl RI_ASMR4_PF_Pos) ## !< 0x00000008
  RI_ASMR4_PF_4* = (0x00000010 shl RI_ASMR4_PF_Pos) ## !< 0x00000010
  RI_ASMR4_PF_5* = (0x00000020 shl RI_ASMR4_PF_Pos) ## !< 0x00000020
  RI_ASMR4_PF_6* = (0x00000040 shl RI_ASMR4_PF_Pos) ## !< 0x00000040
  RI_ASMR4_PF_7* = (0x00000080 shl RI_ASMR4_PF_Pos) ## !< 0x00000080
  RI_ASMR4_PF_8* = (0x00000100 shl RI_ASMR4_PF_Pos) ## !< 0x00000100
  RI_ASMR4_PF_9* = (0x00000200 shl RI_ASMR4_PF_Pos) ## !< 0x00000200
  RI_ASMR4_PF_10* = (0x00000400 shl RI_ASMR4_PF_Pos) ## !< 0x00000400
  RI_ASMR4_PF_11* = (0x00000800 shl RI_ASMR4_PF_Pos) ## !< 0x00000800
  RI_ASMR4_PF_12* = (0x00001000 shl RI_ASMR4_PF_Pos) ## !< 0x00001000
  RI_ASMR4_PF_13* = (0x00002000 shl RI_ASMR4_PF_Pos) ## !< 0x00002000
  RI_ASMR4_PF_14* = (0x00004000 shl RI_ASMR4_PF_Pos) ## !< 0x00004000
  RI_ASMR4_PF_15* = (0x00008000 shl RI_ASMR4_PF_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_CMR4 register  *******************

const
  RI_CMR4_PF_Pos* = (0)
  RI_CMR4_PF_Msk* = (0x0000FFFF shl RI_CMR4_PF_Pos) ## !< 0x0000FFFF
  RI_CMR4_PF* = RI_CMR4_PF_Msk
  RI_CMR4_PF_0* = (0x00000001 shl RI_CMR4_PF_Pos) ## !< 0x00000001
  RI_CMR4_PF_1* = (0x00000002 shl RI_CMR4_PF_Pos) ## !< 0x00000002
  RI_CMR4_PF_2* = (0x00000004 shl RI_CMR4_PF_Pos) ## !< 0x00000004
  RI_CMR4_PF_3* = (0x00000008 shl RI_CMR4_PF_Pos) ## !< 0x00000008
  RI_CMR4_PF_4* = (0x00000010 shl RI_CMR4_PF_Pos) ## !< 0x00000010
  RI_CMR4_PF_5* = (0x00000020 shl RI_CMR4_PF_Pos) ## !< 0x00000020
  RI_CMR4_PF_6* = (0x00000040 shl RI_CMR4_PF_Pos) ## !< 0x00000040
  RI_CMR4_PF_7* = (0x00000080 shl RI_CMR4_PF_Pos) ## !< 0x00000080
  RI_CMR4_PF_8* = (0x00000100 shl RI_CMR4_PF_Pos) ## !< 0x00000100
  RI_CMR4_PF_9* = (0x00000200 shl RI_CMR4_PF_Pos) ## !< 0x00000200
  RI_CMR4_PF_10* = (0x00000400 shl RI_CMR4_PF_Pos) ## !< 0x00000400
  RI_CMR4_PF_11* = (0x00000800 shl RI_CMR4_PF_Pos) ## !< 0x00000800
  RI_CMR4_PF_12* = (0x00001000 shl RI_CMR4_PF_Pos) ## !< 0x00001000
  RI_CMR4_PF_13* = (0x00002000 shl RI_CMR4_PF_Pos) ## !< 0x00002000
  RI_CMR4_PF_14* = (0x00004000 shl RI_CMR4_PF_Pos) ## !< 0x00004000
  RI_CMR4_PF_15* = (0x00008000 shl RI_CMR4_PF_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_CICR4 register  *******************

const
  RI_CICR4_PF_Pos* = (0)
  RI_CICR4_PF_Msk* = (0x0000FFFF shl RI_CICR4_PF_Pos) ## !< 0x0000FFFF
  RI_CICR4_PF* = RI_CICR4_PF_Msk
  RI_CICR4_PF_0* = (0x00000001 shl RI_CICR4_PF_Pos) ## !< 0x00000001
  RI_CICR4_PF_1* = (0x00000002 shl RI_CICR4_PF_Pos) ## !< 0x00000002
  RI_CICR4_PF_2* = (0x00000004 shl RI_CICR4_PF_Pos) ## !< 0x00000004
  RI_CICR4_PF_3* = (0x00000008 shl RI_CICR4_PF_Pos) ## !< 0x00000008
  RI_CICR4_PF_4* = (0x00000010 shl RI_CICR4_PF_Pos) ## !< 0x00000010
  RI_CICR4_PF_5* = (0x00000020 shl RI_CICR4_PF_Pos) ## !< 0x00000020
  RI_CICR4_PF_6* = (0x00000040 shl RI_CICR4_PF_Pos) ## !< 0x00000040
  RI_CICR4_PF_7* = (0x00000080 shl RI_CICR4_PF_Pos) ## !< 0x00000080
  RI_CICR4_PF_8* = (0x00000100 shl RI_CICR4_PF_Pos) ## !< 0x00000100
  RI_CICR4_PF_9* = (0x00000200 shl RI_CICR4_PF_Pos) ## !< 0x00000200
  RI_CICR4_PF_10* = (0x00000400 shl RI_CICR4_PF_Pos) ## !< 0x00000400
  RI_CICR4_PF_11* = (0x00000800 shl RI_CICR4_PF_Pos) ## !< 0x00000800
  RI_CICR4_PF_12* = (0x00001000 shl RI_CICR4_PF_Pos) ## !< 0x00001000
  RI_CICR4_PF_13* = (0x00002000 shl RI_CICR4_PF_Pos) ## !< 0x00002000
  RI_CICR4_PF_14* = (0x00004000 shl RI_CICR4_PF_Pos) ## !< 0x00004000
  RI_CICR4_PF_15* = (0x00008000 shl RI_CICR4_PF_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_ASMR5 register  *******************

const
  RI_ASMR5_PG_Pos* = (0)
  RI_ASMR5_PG_Msk* = (0x0000FFFF shl RI_ASMR5_PG_Pos) ## !< 0x0000FFFF
  RI_ASMR5_PG* = RI_ASMR5_PG_Msk
  RI_ASMR5_PG_0* = (0x00000001 shl RI_ASMR5_PG_Pos) ## !< 0x00000001
  RI_ASMR5_PG_1* = (0x00000002 shl RI_ASMR5_PG_Pos) ## !< 0x00000002
  RI_ASMR5_PG_2* = (0x00000004 shl RI_ASMR5_PG_Pos) ## !< 0x00000004
  RI_ASMR5_PG_3* = (0x00000008 shl RI_ASMR5_PG_Pos) ## !< 0x00000008
  RI_ASMR5_PG_4* = (0x00000010 shl RI_ASMR5_PG_Pos) ## !< 0x00000010
  RI_ASMR5_PG_5* = (0x00000020 shl RI_ASMR5_PG_Pos) ## !< 0x00000020
  RI_ASMR5_PG_6* = (0x00000040 shl RI_ASMR5_PG_Pos) ## !< 0x00000040
  RI_ASMR5_PG_7* = (0x00000080 shl RI_ASMR5_PG_Pos) ## !< 0x00000080
  RI_ASMR5_PG_8* = (0x00000100 shl RI_ASMR5_PG_Pos) ## !< 0x00000100
  RI_ASMR5_PG_9* = (0x00000200 shl RI_ASMR5_PG_Pos) ## !< 0x00000200
  RI_ASMR5_PG_10* = (0x00000400 shl RI_ASMR5_PG_Pos) ## !< 0x00000400
  RI_ASMR5_PG_11* = (0x00000800 shl RI_ASMR5_PG_Pos) ## !< 0x00000800
  RI_ASMR5_PG_12* = (0x00001000 shl RI_ASMR5_PG_Pos) ## !< 0x00001000
  RI_ASMR5_PG_13* = (0x00002000 shl RI_ASMR5_PG_Pos) ## !< 0x00002000
  RI_ASMR5_PG_14* = (0x00004000 shl RI_ASMR5_PG_Pos) ## !< 0x00004000
  RI_ASMR5_PG_15* = (0x00008000 shl RI_ASMR5_PG_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_CMR5 register  *******************

const
  RI_CMR5_PG_Pos* = (0)
  RI_CMR5_PG_Msk* = (0x0000FFFF shl RI_CMR5_PG_Pos) ## !< 0x0000FFFF
  RI_CMR5_PG* = RI_CMR5_PG_Msk
  RI_CMR5_PG_0* = (0x00000001 shl RI_CMR5_PG_Pos) ## !< 0x00000001
  RI_CMR5_PG_1* = (0x00000002 shl RI_CMR5_PG_Pos) ## !< 0x00000002
  RI_CMR5_PG_2* = (0x00000004 shl RI_CMR5_PG_Pos) ## !< 0x00000004
  RI_CMR5_PG_3* = (0x00000008 shl RI_CMR5_PG_Pos) ## !< 0x00000008
  RI_CMR5_PG_4* = (0x00000010 shl RI_CMR5_PG_Pos) ## !< 0x00000010
  RI_CMR5_PG_5* = (0x00000020 shl RI_CMR5_PG_Pos) ## !< 0x00000020
  RI_CMR5_PG_6* = (0x00000040 shl RI_CMR5_PG_Pos) ## !< 0x00000040
  RI_CMR5_PG_7* = (0x00000080 shl RI_CMR5_PG_Pos) ## !< 0x00000080
  RI_CMR5_PG_8* = (0x00000100 shl RI_CMR5_PG_Pos) ## !< 0x00000100
  RI_CMR5_PG_9* = (0x00000200 shl RI_CMR5_PG_Pos) ## !< 0x00000200
  RI_CMR5_PG_10* = (0x00000400 shl RI_CMR5_PG_Pos) ## !< 0x00000400
  RI_CMR5_PG_11* = (0x00000800 shl RI_CMR5_PG_Pos) ## !< 0x00000800
  RI_CMR5_PG_12* = (0x00001000 shl RI_CMR5_PG_Pos) ## !< 0x00001000
  RI_CMR5_PG_13* = (0x00002000 shl RI_CMR5_PG_Pos) ## !< 0x00002000
  RI_CMR5_PG_14* = (0x00004000 shl RI_CMR5_PG_Pos) ## !< 0x00004000
  RI_CMR5_PG_15* = (0x00008000 shl RI_CMR5_PG_Pos) ## !< 0x00008000

## *******************  Bit definition for RI_CICR5 register  *******************

const
  RI_CICR5_PG_Pos* = (0)
  RI_CICR5_PG_Msk* = (0x0000FFFF shl RI_CICR5_PG_Pos) ## !< 0x0000FFFF
  RI_CICR5_PG* = RI_CICR5_PG_Msk
  RI_CICR5_PG_0* = (0x00000001 shl RI_CICR5_PG_Pos) ## !< 0x00000001
  RI_CICR5_PG_1* = (0x00000002 shl RI_CICR5_PG_Pos) ## !< 0x00000002
  RI_CICR5_PG_2* = (0x00000004 shl RI_CICR5_PG_Pos) ## !< 0x00000004
  RI_CICR5_PG_3* = (0x00000008 shl RI_CICR5_PG_Pos) ## !< 0x00000008
  RI_CICR5_PG_4* = (0x00000010 shl RI_CICR5_PG_Pos) ## !< 0x00000010
  RI_CICR5_PG_5* = (0x00000020 shl RI_CICR5_PG_Pos) ## !< 0x00000020
  RI_CICR5_PG_6* = (0x00000040 shl RI_CICR5_PG_Pos) ## !< 0x00000040
  RI_CICR5_PG_7* = (0x00000080 shl RI_CICR5_PG_Pos) ## !< 0x00000080
  RI_CICR5_PG_8* = (0x00000100 shl RI_CICR5_PG_Pos) ## !< 0x00000100
  RI_CICR5_PG_9* = (0x00000200 shl RI_CICR5_PG_Pos) ## !< 0x00000200
  RI_CICR5_PG_10* = (0x00000400 shl RI_CICR5_PG_Pos) ## !< 0x00000400
  RI_CICR5_PG_11* = (0x00000800 shl RI_CICR5_PG_Pos) ## !< 0x00000800
  RI_CICR5_PG_12* = (0x00001000 shl RI_CICR5_PG_Pos) ## !< 0x00001000
  RI_CICR5_PG_13* = (0x00002000 shl RI_CICR5_PG_Pos) ## !< 0x00002000
  RI_CICR5_PG_14* = (0x00004000 shl RI_CICR5_PG_Pos) ## !< 0x00004000
  RI_CICR5_PG_15* = (0x00008000 shl RI_CICR5_PG_Pos) ## !< 0x00008000

## ****************************************************************************
##
##                                Timers (TIM)
##
## ****************************************************************************
## ******************  Bit definition for TIM_CR1 register  *******************

const
  TIM_CR1_CEN_Pos* = (0)
  TIM_CR1_CEN_Msk* = (0x00000001 shl TIM_CR1_CEN_Pos) ## !< 0x00000001
  TIM_CR1_CEN* = TIM_CR1_CEN_Msk
  TIM_CR1_UDIS_Pos* = (1)
  TIM_CR1_UDIS_Msk* = (0x00000001 shl TIM_CR1_UDIS_Pos) ## !< 0x00000002
  TIM_CR1_UDIS* = TIM_CR1_UDIS_Msk
  TIM_CR1_URS_Pos* = (2)
  TIM_CR1_URS_Msk* = (0x00000001 shl TIM_CR1_URS_Pos) ## !< 0x00000004
  TIM_CR1_URS* = TIM_CR1_URS_Msk
  TIM_CR1_OPM_Pos* = (3)
  TIM_CR1_OPM_Msk* = (0x00000001 shl TIM_CR1_OPM_Pos) ## !< 0x00000008
  TIM_CR1_OPM* = TIM_CR1_OPM_Msk
  TIM_CR1_DIR_Pos* = (4)
  TIM_CR1_DIR_Msk* = (0x00000001 shl TIM_CR1_DIR_Pos) ## !< 0x00000010
  TIM_CR1_DIR* = TIM_CR1_DIR_Msk
  TIM_CR1_CMS_Pos* = (5)
  TIM_CR1_CMS_Msk* = (0x00000003 shl TIM_CR1_CMS_Pos) ## !< 0x00000060
  TIM_CR1_CMS* = TIM_CR1_CMS_Msk
  TIM_CR1_CMS_0* = (0x00000001 shl TIM_CR1_CMS_Pos) ## !< 0x00000020
  TIM_CR1_CMS_1* = (0x00000002 shl TIM_CR1_CMS_Pos) ## !< 0x00000040
  TIM_CR1_ARPE_Pos* = (7)
  TIM_CR1_ARPE_Msk* = (0x00000001 shl TIM_CR1_ARPE_Pos) ## !< 0x00000080
  TIM_CR1_ARPE* = TIM_CR1_ARPE_Msk
  TIM_CR1_CKD_Pos* = (8)
  TIM_CR1_CKD_Msk* = (0x00000003 shl TIM_CR1_CKD_Pos) ## !< 0x00000300
  TIM_CR1_CKD* = TIM_CR1_CKD_Msk
  TIM_CR1_CKD_0* = (0x00000001 shl TIM_CR1_CKD_Pos) ## !< 0x00000100
  TIM_CR1_CKD_1* = (0x00000002 shl TIM_CR1_CKD_Pos) ## !< 0x00000200

## ******************  Bit definition for TIM_CR2 register  *******************

const
  TIM_CR2_CCDS_Pos* = (3)
  TIM_CR2_CCDS_Msk* = (0x00000001 shl TIM_CR2_CCDS_Pos) ## !< 0x00000008
  TIM_CR2_CCDS* = TIM_CR2_CCDS_Msk
  TIM_CR2_MMS_Pos* = (4)
  TIM_CR2_MMS_Msk* = (0x00000007 shl TIM_CR2_MMS_Pos) ## !< 0x00000070
  TIM_CR2_MMS* = TIM_CR2_MMS_Msk
  TIM_CR2_MMS_0* = (0x00000001 shl TIM_CR2_MMS_Pos) ## !< 0x00000010
  TIM_CR2_MMS_1* = (0x00000002 shl TIM_CR2_MMS_Pos) ## !< 0x00000020
  TIM_CR2_MMS_2* = (0x00000004 shl TIM_CR2_MMS_Pos) ## !< 0x00000040
  TIM_CR2_TI1S_Pos* = (7)
  TIM_CR2_TI1S_Msk* = (0x00000001 shl TIM_CR2_TI1S_Pos) ## !< 0x00000080
  TIM_CR2_TI1S* = TIM_CR2_TI1S_Msk

## ******************  Bit definition for TIM_SMCR register  ******************

const
  TIM_SMCR_SMS_Pos* = (0)
  TIM_SMCR_SMS_Msk* = (0x00000007 shl TIM_SMCR_SMS_Pos) ## !< 0x00000007
  TIM_SMCR_SMS* = TIM_SMCR_SMS_Msk
  TIM_SMCR_SMS_0* = (0x00000001 shl TIM_SMCR_SMS_Pos) ## !< 0x00000001
  TIM_SMCR_SMS_1* = (0x00000002 shl TIM_SMCR_SMS_Pos) ## !< 0x00000002
  TIM_SMCR_SMS_2* = (0x00000004 shl TIM_SMCR_SMS_Pos) ## !< 0x00000004
  TIM_SMCR_OCCS_Pos* = (3)
  TIM_SMCR_OCCS_Msk* = (0x00000001 shl TIM_SMCR_OCCS_Pos) ## !< 0x00000008
  TIM_SMCR_OCCS* = TIM_SMCR_OCCS_Msk
  TIM_SMCR_TS_Pos* = (4)
  TIM_SMCR_TS_Msk* = (0x00000007 shl TIM_SMCR_TS_Pos) ## !< 0x00000070
  TIM_SMCR_TS* = TIM_SMCR_TS_Msk
  TIM_SMCR_TS_0* = (0x00000001 shl TIM_SMCR_TS_Pos) ## !< 0x00000010
  TIM_SMCR_TS_1* = (0x00000002 shl TIM_SMCR_TS_Pos) ## !< 0x00000020
  TIM_SMCR_TS_2* = (0x00000004 shl TIM_SMCR_TS_Pos) ## !< 0x00000040
  TIM_SMCR_MSM_Pos* = (7)
  TIM_SMCR_MSM_Msk* = (0x00000001 shl TIM_SMCR_MSM_Pos) ## !< 0x00000080
  TIM_SMCR_MSM* = TIM_SMCR_MSM_Msk
  TIM_SMCR_ETF_Pos* = (8)
  TIM_SMCR_ETF_Msk* = (0x0000000F shl TIM_SMCR_ETF_Pos) ## !< 0x00000F00
  TIM_SMCR_ETF* = TIM_SMCR_ETF_Msk
  TIM_SMCR_ETF_0* = (0x00000001 shl TIM_SMCR_ETF_Pos) ## !< 0x00000100
  TIM_SMCR_ETF_1* = (0x00000002 shl TIM_SMCR_ETF_Pos) ## !< 0x00000200
  TIM_SMCR_ETF_2* = (0x00000004 shl TIM_SMCR_ETF_Pos) ## !< 0x00000400
  TIM_SMCR_ETF_3* = (0x00000008 shl TIM_SMCR_ETF_Pos) ## !< 0x00000800
  TIM_SMCR_ETPS_Pos* = (12)
  TIM_SMCR_ETPS_Msk* = (0x00000003 shl TIM_SMCR_ETPS_Pos) ## !< 0x00003000
  TIM_SMCR_ETPS* = TIM_SMCR_ETPS_Msk
  TIM_SMCR_ETPS_0* = (0x00000001 shl TIM_SMCR_ETPS_Pos) ## !< 0x00001000
  TIM_SMCR_ETPS_1* = (0x00000002 shl TIM_SMCR_ETPS_Pos) ## !< 0x00002000
  TIM_SMCR_ECE_Pos* = (14)
  TIM_SMCR_ECE_Msk* = (0x00000001 shl TIM_SMCR_ECE_Pos) ## !< 0x00004000
  TIM_SMCR_ECE* = TIM_SMCR_ECE_Msk
  TIM_SMCR_ETP_Pos* = (15)
  TIM_SMCR_ETP_Msk* = (0x00000001 shl TIM_SMCR_ETP_Pos) ## !< 0x00008000
  TIM_SMCR_ETP* = TIM_SMCR_ETP_Msk

## ******************  Bit definition for TIM_DIER register  ******************

const
  TIM_DIER_UIE_Pos* = (0)
  TIM_DIER_UIE_Msk* = (0x00000001 shl TIM_DIER_UIE_Pos) ## !< 0x00000001
  TIM_DIER_UIE* = TIM_DIER_UIE_Msk
  TIM_DIER_CC1IE_Pos* = (1)
  TIM_DIER_CC1IE_Msk* = (0x00000001 shl TIM_DIER_CC1IE_Pos) ## !< 0x00000002
  TIM_DIER_CC1IE* = TIM_DIER_CC1IE_Msk
  TIM_DIER_CC2IE_Pos* = (2)
  TIM_DIER_CC2IE_Msk* = (0x00000001 shl TIM_DIER_CC2IE_Pos) ## !< 0x00000004
  TIM_DIER_CC2IE* = TIM_DIER_CC2IE_Msk
  TIM_DIER_CC3IE_Pos* = (3)
  TIM_DIER_CC3IE_Msk* = (0x00000001 shl TIM_DIER_CC3IE_Pos) ## !< 0x00000008
  TIM_DIER_CC3IE* = TIM_DIER_CC3IE_Msk
  TIM_DIER_CC4IE_Pos* = (4)
  TIM_DIER_CC4IE_Msk* = (0x00000001 shl TIM_DIER_CC4IE_Pos) ## !< 0x00000010
  TIM_DIER_CC4IE* = TIM_DIER_CC4IE_Msk
  TIM_DIER_TIE_Pos* = (6)
  TIM_DIER_TIE_Msk* = (0x00000001 shl TIM_DIER_TIE_Pos) ## !< 0x00000040
  TIM_DIER_TIE* = TIM_DIER_TIE_Msk
  TIM_DIER_UDE_Pos* = (8)
  TIM_DIER_UDE_Msk* = (0x00000001 shl TIM_DIER_UDE_Pos) ## !< 0x00000100
  TIM_DIER_UDE* = TIM_DIER_UDE_Msk
  TIM_DIER_CC1DE_Pos* = (9)
  TIM_DIER_CC1DE_Msk* = (0x00000001 shl TIM_DIER_CC1DE_Pos) ## !< 0x00000200
  TIM_DIER_CC1DE* = TIM_DIER_CC1DE_Msk
  TIM_DIER_CC2DE_Pos* = (10)
  TIM_DIER_CC2DE_Msk* = (0x00000001 shl TIM_DIER_CC2DE_Pos) ## !< 0x00000400
  TIM_DIER_CC2DE* = TIM_DIER_CC2DE_Msk
  TIM_DIER_CC3DE_Pos* = (11)
  TIM_DIER_CC3DE_Msk* = (0x00000001 shl TIM_DIER_CC3DE_Pos) ## !< 0x00000800
  TIM_DIER_CC3DE* = TIM_DIER_CC3DE_Msk
  TIM_DIER_CC4DE_Pos* = (12)
  TIM_DIER_CC4DE_Msk* = (0x00000001 shl TIM_DIER_CC4DE_Pos) ## !< 0x00001000
  TIM_DIER_CC4DE* = TIM_DIER_CC4DE_Msk
  TIM_DIER_COMDE* = 0x00002000'u32 ## !<COM DMA request enable
  TIM_DIER_TDE_Pos* = (14)
  TIM_DIER_TDE_Msk* = (0x00000001 shl TIM_DIER_TDE_Pos) ## !< 0x00004000
  TIM_DIER_TDE* = TIM_DIER_TDE_Msk

## *******************  Bit definition for TIM_SR register  *******************

const
  TIM_SR_UIF_Pos* = (0)
  TIM_SR_UIF_Msk* = (0x00000001 shl TIM_SR_UIF_Pos) ## !< 0x00000001
  TIM_SR_UIF* = TIM_SR_UIF_Msk
  TIM_SR_CC1IF_Pos* = (1)
  TIM_SR_CC1IF_Msk* = (0x00000001 shl TIM_SR_CC1IF_Pos) ## !< 0x00000002
  TIM_SR_CC1IF* = TIM_SR_CC1IF_Msk
  TIM_SR_CC2IF_Pos* = (2)
  TIM_SR_CC2IF_Msk* = (0x00000001 shl TIM_SR_CC2IF_Pos) ## !< 0x00000004
  TIM_SR_CC2IF* = TIM_SR_CC2IF_Msk
  TIM_SR_CC3IF_Pos* = (3)
  TIM_SR_CC3IF_Msk* = (0x00000001 shl TIM_SR_CC3IF_Pos) ## !< 0x00000008
  TIM_SR_CC3IF* = TIM_SR_CC3IF_Msk
  TIM_SR_CC4IF_Pos* = (4)
  TIM_SR_CC4IF_Msk* = (0x00000001 shl TIM_SR_CC4IF_Pos) ## !< 0x00000010
  TIM_SR_CC4IF* = TIM_SR_CC4IF_Msk
  TIM_SR_TIF_Pos* = (6)
  TIM_SR_TIF_Msk* = (0x00000001 shl TIM_SR_TIF_Pos) ## !< 0x00000040
  TIM_SR_TIF* = TIM_SR_TIF_Msk
  TIM_SR_CC1OF_Pos* = (9)
  TIM_SR_CC1OF_Msk* = (0x00000001 shl TIM_SR_CC1OF_Pos) ## !< 0x00000200
  TIM_SR_CC1OF* = TIM_SR_CC1OF_Msk
  TIM_SR_CC2OF_Pos* = (10)
  TIM_SR_CC2OF_Msk* = (0x00000001 shl TIM_SR_CC2OF_Pos) ## !< 0x00000400
  TIM_SR_CC2OF* = TIM_SR_CC2OF_Msk
  TIM_SR_CC3OF_Pos* = (11)
  TIM_SR_CC3OF_Msk* = (0x00000001 shl TIM_SR_CC3OF_Pos) ## !< 0x00000800
  TIM_SR_CC3OF* = TIM_SR_CC3OF_Msk
  TIM_SR_CC4OF_Pos* = (12)
  TIM_SR_CC4OF_Msk* = (0x00000001 shl TIM_SR_CC4OF_Pos) ## !< 0x00001000
  TIM_SR_CC4OF* = TIM_SR_CC4OF_Msk

## ******************  Bit definition for TIM_EGR register  *******************

const
  TIM_EGR_UG_Pos* = (0)
  TIM_EGR_UG_Msk* = (0x00000001 shl TIM_EGR_UG_Pos) ## !< 0x00000001
  TIM_EGR_UG* = TIM_EGR_UG_Msk
  TIM_EGR_CC1G_Pos* = (1)
  TIM_EGR_CC1G_Msk* = (0x00000001 shl TIM_EGR_CC1G_Pos) ## !< 0x00000002
  TIM_EGR_CC1G* = TIM_EGR_CC1G_Msk
  TIM_EGR_CC2G_Pos* = (2)
  TIM_EGR_CC2G_Msk* = (0x00000001 shl TIM_EGR_CC2G_Pos) ## !< 0x00000004
  TIM_EGR_CC2G* = TIM_EGR_CC2G_Msk
  TIM_EGR_CC3G_Pos* = (3)
  TIM_EGR_CC3G_Msk* = (0x00000001 shl TIM_EGR_CC3G_Pos) ## !< 0x00000008
  TIM_EGR_CC3G* = TIM_EGR_CC3G_Msk
  TIM_EGR_CC4G_Pos* = (4)
  TIM_EGR_CC4G_Msk* = (0x00000001 shl TIM_EGR_CC4G_Pos) ## !< 0x00000010
  TIM_EGR_CC4G* = TIM_EGR_CC4G_Msk
  TIM_EGR_TG_Pos* = (6)
  TIM_EGR_TG_Msk* = (0x00000001 shl TIM_EGR_TG_Pos) ## !< 0x00000040
  TIM_EGR_TG* = TIM_EGR_TG_Msk

## *****************  Bit definition for TIM_CCMR1 register  ******************

const
  TIM_CCMR1_CC1S_Pos* = (0)
  TIM_CCMR1_CC1S_Msk* = (0x00000003 shl TIM_CCMR1_CC1S_Pos) ## !< 0x00000003
  TIM_CCMR1_CC1S* = TIM_CCMR1_CC1S_Msk
  TIM_CCMR1_CC1S_0* = (0x00000001 shl TIM_CCMR1_CC1S_Pos) ## !< 0x00000001
  TIM_CCMR1_CC1S_1* = (0x00000002 shl TIM_CCMR1_CC1S_Pos) ## !< 0x00000002
  TIM_CCMR1_OC1FE_Pos* = (2)
  TIM_CCMR1_OC1FE_Msk* = (0x00000001 shl TIM_CCMR1_OC1FE_Pos) ## !< 0x00000004
  TIM_CCMR1_OC1FE* = TIM_CCMR1_OC1FE_Msk
  TIM_CCMR1_OC1PE_Pos* = (3)
  TIM_CCMR1_OC1PE_Msk* = (0x00000001 shl TIM_CCMR1_OC1PE_Pos) ## !< 0x00000008
  TIM_CCMR1_OC1PE* = TIM_CCMR1_OC1PE_Msk
  TIM_CCMR1_OC1M_Pos* = (4)
  TIM_CCMR1_OC1M_Msk* = (0x00000007 shl TIM_CCMR1_OC1M_Pos) ## !< 0x00000070
  TIM_CCMR1_OC1M* = TIM_CCMR1_OC1M_Msk
  TIM_CCMR1_OC1M_0* = (0x00000001 shl TIM_CCMR1_OC1M_Pos) ## !< 0x00000010
  TIM_CCMR1_OC1M_1* = (0x00000002 shl TIM_CCMR1_OC1M_Pos) ## !< 0x00000020
  TIM_CCMR1_OC1M_2* = (0x00000004 shl TIM_CCMR1_OC1M_Pos) ## !< 0x00000040
  TIM_CCMR1_OC1CE_Pos* = (7)
  TIM_CCMR1_OC1CE_Msk* = (0x00000001 shl TIM_CCMR1_OC1CE_Pos) ## !< 0x00000080
  TIM_CCMR1_OC1CE* = TIM_CCMR1_OC1CE_Msk
  TIM_CCMR1_CC2S_Pos* = (8)
  TIM_CCMR1_CC2S_Msk* = (0x00000003 shl TIM_CCMR1_CC2S_Pos) ## !< 0x00000300
  TIM_CCMR1_CC2S* = TIM_CCMR1_CC2S_Msk
  TIM_CCMR1_CC2S_0* = (0x00000001 shl TIM_CCMR1_CC2S_Pos) ## !< 0x00000100
  TIM_CCMR1_CC2S_1* = (0x00000002 shl TIM_CCMR1_CC2S_Pos) ## !< 0x00000200
  TIM_CCMR1_OC2FE_Pos* = (10)
  TIM_CCMR1_OC2FE_Msk* = (0x00000001 shl TIM_CCMR1_OC2FE_Pos) ## !< 0x00000400
  TIM_CCMR1_OC2FE* = TIM_CCMR1_OC2FE_Msk
  TIM_CCMR1_OC2PE_Pos* = (11)
  TIM_CCMR1_OC2PE_Msk* = (0x00000001 shl TIM_CCMR1_OC2PE_Pos) ## !< 0x00000800
  TIM_CCMR1_OC2PE* = TIM_CCMR1_OC2PE_Msk
  TIM_CCMR1_OC2M_Pos* = (12)
  TIM_CCMR1_OC2M_Msk* = (0x00000007 shl TIM_CCMR1_OC2M_Pos) ## !< 0x00007000
  TIM_CCMR1_OC2M* = TIM_CCMR1_OC2M_Msk
  TIM_CCMR1_OC2M_0* = (0x00000001 shl TIM_CCMR1_OC2M_Pos) ## !< 0x00001000
  TIM_CCMR1_OC2M_1* = (0x00000002 shl TIM_CCMR1_OC2M_Pos) ## !< 0x00002000
  TIM_CCMR1_OC2M_2* = (0x00000004 shl TIM_CCMR1_OC2M_Pos) ## !< 0x00004000
  TIM_CCMR1_OC2CE_Pos* = (15)
  TIM_CCMR1_OC2CE_Msk* = (0x00000001 shl TIM_CCMR1_OC2CE_Pos) ## !< 0x00008000
  TIM_CCMR1_OC2CE* = TIM_CCMR1_OC2CE_Msk

## ----------------------------------------------------------------------------

const
  TIM_CCMR1_IC1PSC_Pos* = (2)
  TIM_CCMR1_IC1PSC_Msk* = (0x00000003 shl TIM_CCMR1_IC1PSC_Pos) ## !< 0x0000000C
  TIM_CCMR1_IC1PSC* = TIM_CCMR1_IC1PSC_Msk
  TIM_CCMR1_IC1PSC_0* = (0x00000001 shl TIM_CCMR1_IC1PSC_Pos) ## !< 0x00000004
  TIM_CCMR1_IC1PSC_1* = (0x00000002 shl TIM_CCMR1_IC1PSC_Pos) ## !< 0x00000008
  TIM_CCMR1_IC1F_Pos* = (4)
  TIM_CCMR1_IC1F_Msk* = (0x0000000F shl TIM_CCMR1_IC1F_Pos) ## !< 0x000000F0
  TIM_CCMR1_IC1F* = TIM_CCMR1_IC1F_Msk
  TIM_CCMR1_IC1F_0* = (0x00000001 shl TIM_CCMR1_IC1F_Pos) ## !< 0x00000010
  TIM_CCMR1_IC1F_1* = (0x00000002 shl TIM_CCMR1_IC1F_Pos) ## !< 0x00000020
  TIM_CCMR1_IC1F_2* = (0x00000004 shl TIM_CCMR1_IC1F_Pos) ## !< 0x00000040
  TIM_CCMR1_IC1F_3* = (0x00000008 shl TIM_CCMR1_IC1F_Pos) ## !< 0x00000080
  TIM_CCMR1_IC2PSC_Pos* = (10)
  TIM_CCMR1_IC2PSC_Msk* = (0x00000003 shl TIM_CCMR1_IC2PSC_Pos) ## !< 0x00000C00
  TIM_CCMR1_IC2PSC* = TIM_CCMR1_IC2PSC_Msk
  TIM_CCMR1_IC2PSC_0* = (0x00000001 shl TIM_CCMR1_IC2PSC_Pos) ## !< 0x00000400
  TIM_CCMR1_IC2PSC_1* = (0x00000002 shl TIM_CCMR1_IC2PSC_Pos) ## !< 0x00000800
  TIM_CCMR1_IC2F_Pos* = (12)
  TIM_CCMR1_IC2F_Msk* = (0x0000000F shl TIM_CCMR1_IC2F_Pos) ## !< 0x0000F000
  TIM_CCMR1_IC2F* = TIM_CCMR1_IC2F_Msk
  TIM_CCMR1_IC2F_0* = (0x00000001 shl TIM_CCMR1_IC2F_Pos) ## !< 0x00001000
  TIM_CCMR1_IC2F_1* = (0x00000002 shl TIM_CCMR1_IC2F_Pos) ## !< 0x00002000
  TIM_CCMR1_IC2F_2* = (0x00000004 shl TIM_CCMR1_IC2F_Pos) ## !< 0x00004000
  TIM_CCMR1_IC2F_3* = (0x00000008 shl TIM_CCMR1_IC2F_Pos) ## !< 0x00008000

## *****************  Bit definition for TIM_CCMR2 register  ******************

const
  TIM_CCMR2_CC3S_Pos* = (0)
  TIM_CCMR2_CC3S_Msk* = (0x00000003 shl TIM_CCMR2_CC3S_Pos) ## !< 0x00000003
  TIM_CCMR2_CC3S* = TIM_CCMR2_CC3S_Msk
  TIM_CCMR2_CC3S_0* = (0x00000001 shl TIM_CCMR2_CC3S_Pos) ## !< 0x00000001
  TIM_CCMR2_CC3S_1* = (0x00000002 shl TIM_CCMR2_CC3S_Pos) ## !< 0x00000002
  TIM_CCMR2_OC3FE_Pos* = (2)
  TIM_CCMR2_OC3FE_Msk* = (0x00000001 shl TIM_CCMR2_OC3FE_Pos) ## !< 0x00000004
  TIM_CCMR2_OC3FE* = TIM_CCMR2_OC3FE_Msk
  TIM_CCMR2_OC3PE_Pos* = (3)
  TIM_CCMR2_OC3PE_Msk* = (0x00000001 shl TIM_CCMR2_OC3PE_Pos) ## !< 0x00000008
  TIM_CCMR2_OC3PE* = TIM_CCMR2_OC3PE_Msk
  TIM_CCMR2_OC3M_Pos* = (4)
  TIM_CCMR2_OC3M_Msk* = (0x00000007 shl TIM_CCMR2_OC3M_Pos) ## !< 0x00000070
  TIM_CCMR2_OC3M* = TIM_CCMR2_OC3M_Msk
  TIM_CCMR2_OC3M_0* = (0x00000001 shl TIM_CCMR2_OC3M_Pos) ## !< 0x00000010
  TIM_CCMR2_OC3M_1* = (0x00000002 shl TIM_CCMR2_OC3M_Pos) ## !< 0x00000020
  TIM_CCMR2_OC3M_2* = (0x00000004 shl TIM_CCMR2_OC3M_Pos) ## !< 0x00000040
  TIM_CCMR2_OC3CE_Pos* = (7)
  TIM_CCMR2_OC3CE_Msk* = (0x00000001 shl TIM_CCMR2_OC3CE_Pos) ## !< 0x00000080
  TIM_CCMR2_OC3CE* = TIM_CCMR2_OC3CE_Msk
  TIM_CCMR2_CC4S_Pos* = (8)
  TIM_CCMR2_CC4S_Msk* = (0x00000003 shl TIM_CCMR2_CC4S_Pos) ## !< 0x00000300
  TIM_CCMR2_CC4S* = TIM_CCMR2_CC4S_Msk
  TIM_CCMR2_CC4S_0* = (0x00000001 shl TIM_CCMR2_CC4S_Pos) ## !< 0x00000100
  TIM_CCMR2_CC4S_1* = (0x00000002 shl TIM_CCMR2_CC4S_Pos) ## !< 0x00000200
  TIM_CCMR2_OC4FE_Pos* = (10)
  TIM_CCMR2_OC4FE_Msk* = (0x00000001 shl TIM_CCMR2_OC4FE_Pos) ## !< 0x00000400
  TIM_CCMR2_OC4FE* = TIM_CCMR2_OC4FE_Msk
  TIM_CCMR2_OC4PE_Pos* = (11)
  TIM_CCMR2_OC4PE_Msk* = (0x00000001 shl TIM_CCMR2_OC4PE_Pos) ## !< 0x00000800
  TIM_CCMR2_OC4PE* = TIM_CCMR2_OC4PE_Msk
  TIM_CCMR2_OC4M_Pos* = (12)
  TIM_CCMR2_OC4M_Msk* = (0x00000007 shl TIM_CCMR2_OC4M_Pos) ## !< 0x00007000
  TIM_CCMR2_OC4M* = TIM_CCMR2_OC4M_Msk
  TIM_CCMR2_OC4M_0* = (0x00000001 shl TIM_CCMR2_OC4M_Pos) ## !< 0x00001000
  TIM_CCMR2_OC4M_1* = (0x00000002 shl TIM_CCMR2_OC4M_Pos) ## !< 0x00002000
  TIM_CCMR2_OC4M_2* = (0x00000004 shl TIM_CCMR2_OC4M_Pos) ## !< 0x00004000
  TIM_CCMR2_OC4CE_Pos* = (15)
  TIM_CCMR2_OC4CE_Msk* = (0x00000001 shl TIM_CCMR2_OC4CE_Pos) ## !< 0x00008000
  TIM_CCMR2_OC4CE* = TIM_CCMR2_OC4CE_Msk

## ----------------------------------------------------------------------------

const
  TIM_CCMR2_IC3PSC_Pos* = (2)
  TIM_CCMR2_IC3PSC_Msk* = (0x00000003 shl TIM_CCMR2_IC3PSC_Pos) ## !< 0x0000000C
  TIM_CCMR2_IC3PSC* = TIM_CCMR2_IC3PSC_Msk
  TIM_CCMR2_IC3PSC_0* = (0x00000001 shl TIM_CCMR2_IC3PSC_Pos) ## !< 0x00000004
  TIM_CCMR2_IC3PSC_1* = (0x00000002 shl TIM_CCMR2_IC3PSC_Pos) ## !< 0x00000008
  TIM_CCMR2_IC3F_Pos* = (4)
  TIM_CCMR2_IC3F_Msk* = (0x0000000F shl TIM_CCMR2_IC3F_Pos) ## !< 0x000000F0
  TIM_CCMR2_IC3F* = TIM_CCMR2_IC3F_Msk
  TIM_CCMR2_IC3F_0* = (0x00000001 shl TIM_CCMR2_IC3F_Pos) ## !< 0x00000010
  TIM_CCMR2_IC3F_1* = (0x00000002 shl TIM_CCMR2_IC3F_Pos) ## !< 0x00000020
  TIM_CCMR2_IC3F_2* = (0x00000004 shl TIM_CCMR2_IC3F_Pos) ## !< 0x00000040
  TIM_CCMR2_IC3F_3* = (0x00000008 shl TIM_CCMR2_IC3F_Pos) ## !< 0x00000080
  TIM_CCMR2_IC4PSC_Pos* = (10)
  TIM_CCMR2_IC4PSC_Msk* = (0x00000003 shl TIM_CCMR2_IC4PSC_Pos) ## !< 0x00000C00
  TIM_CCMR2_IC4PSC* = TIM_CCMR2_IC4PSC_Msk
  TIM_CCMR2_IC4PSC_0* = (0x00000001 shl TIM_CCMR2_IC4PSC_Pos) ## !< 0x00000400
  TIM_CCMR2_IC4PSC_1* = (0x00000002 shl TIM_CCMR2_IC4PSC_Pos) ## !< 0x00000800
  TIM_CCMR2_IC4F_Pos* = (12)
  TIM_CCMR2_IC4F_Msk* = (0x0000000F shl TIM_CCMR2_IC4F_Pos) ## !< 0x0000F000
  TIM_CCMR2_IC4F* = TIM_CCMR2_IC4F_Msk
  TIM_CCMR2_IC4F_0* = (0x00000001 shl TIM_CCMR2_IC4F_Pos) ## !< 0x00001000
  TIM_CCMR2_IC4F_1* = (0x00000002 shl TIM_CCMR2_IC4F_Pos) ## !< 0x00002000
  TIM_CCMR2_IC4F_2* = (0x00000004 shl TIM_CCMR2_IC4F_Pos) ## !< 0x00004000
  TIM_CCMR2_IC4F_3* = (0x00000008 shl TIM_CCMR2_IC4F_Pos) ## !< 0x00008000

## ******************  Bit definition for TIM_CCER register  ******************

const
  TIM_CCER_CC1E_Pos* = (0)
  TIM_CCER_CC1E_Msk* = (0x00000001 shl TIM_CCER_CC1E_Pos) ## !< 0x00000001
  TIM_CCER_CC1E* = TIM_CCER_CC1E_Msk
  TIM_CCER_CC1P_Pos* = (1)
  TIM_CCER_CC1P_Msk* = (0x00000001 shl TIM_CCER_CC1P_Pos) ## !< 0x00000002
  TIM_CCER_CC1P* = TIM_CCER_CC1P_Msk
  TIM_CCER_CC1NP_Pos* = (3)
  TIM_CCER_CC1NP_Msk* = (0x00000001 shl TIM_CCER_CC1NP_Pos) ## !< 0x00000008
  TIM_CCER_CC1NP* = TIM_CCER_CC1NP_Msk
  TIM_CCER_CC2E_Pos* = (4)
  TIM_CCER_CC2E_Msk* = (0x00000001 shl TIM_CCER_CC2E_Pos) ## !< 0x00000010
  TIM_CCER_CC2E* = TIM_CCER_CC2E_Msk
  TIM_CCER_CC2P_Pos* = (5)
  TIM_CCER_CC2P_Msk* = (0x00000001 shl TIM_CCER_CC2P_Pos) ## !< 0x00000020
  TIM_CCER_CC2P* = TIM_CCER_CC2P_Msk
  TIM_CCER_CC2NP_Pos* = (7)
  TIM_CCER_CC2NP_Msk* = (0x00000001 shl TIM_CCER_CC2NP_Pos) ## !< 0x00000080
  TIM_CCER_CC2NP* = TIM_CCER_CC2NP_Msk
  TIM_CCER_CC3E_Pos* = (8)
  TIM_CCER_CC3E_Msk* = (0x00000001 shl TIM_CCER_CC3E_Pos) ## !< 0x00000100
  TIM_CCER_CC3E* = TIM_CCER_CC3E_Msk
  TIM_CCER_CC3P_Pos* = (9)
  TIM_CCER_CC3P_Msk* = (0x00000001 shl TIM_CCER_CC3P_Pos) ## !< 0x00000200
  TIM_CCER_CC3P* = TIM_CCER_CC3P_Msk
  TIM_CCER_CC3NP_Pos* = (11)
  TIM_CCER_CC3NP_Msk* = (0x00000001 shl TIM_CCER_CC3NP_Pos) ## !< 0x00000800
  TIM_CCER_CC3NP* = TIM_CCER_CC3NP_Msk
  TIM_CCER_CC4E_Pos* = (12)
  TIM_CCER_CC4E_Msk* = (0x00000001 shl TIM_CCER_CC4E_Pos) ## !< 0x00001000
  TIM_CCER_CC4E* = TIM_CCER_CC4E_Msk
  TIM_CCER_CC4P_Pos* = (13)
  TIM_CCER_CC4P_Msk* = (0x00000001 shl TIM_CCER_CC4P_Pos) ## !< 0x00002000
  TIM_CCER_CC4P* = TIM_CCER_CC4P_Msk
  TIM_CCER_CC4NP_Pos* = (15)
  TIM_CCER_CC4NP_Msk* = (0x00000001 shl TIM_CCER_CC4NP_Pos) ## !< 0x00008000
  TIM_CCER_CC4NP* = TIM_CCER_CC4NP_Msk

## ******************  Bit definition for TIM_CNT register  *******************

const
  TIM_CNT_CNT_Pos* = (0)
  TIM_CNT_CNT_Msk* = (0xFFFFFFFF shl TIM_CNT_CNT_Pos) ## !< 0xFFFFFFFF
  TIM_CNT_CNT* = TIM_CNT_CNT_Msk

## ******************  Bit definition for TIM_PSC register  *******************

const
  TIM_PSC_PSC_Pos* = (0)
  TIM_PSC_PSC_Msk* = (0x0000FFFF shl TIM_PSC_PSC_Pos) ## !< 0x0000FFFF
  TIM_PSC_PSC* = TIM_PSC_PSC_Msk

## ******************  Bit definition for TIM_ARR register  *******************

const
  TIM_ARR_ARR_Pos* = (0)
  TIM_ARR_ARR_Msk* = (0xFFFFFFFF shl TIM_ARR_ARR_Pos) ## !< 0xFFFFFFFF
  TIM_ARR_ARR* = TIM_ARR_ARR_Msk

## ******************  Bit definition for TIM_CCR1 register  ******************

const
  TIM_CCR1_CCR1_Pos* = (0)
  TIM_CCR1_CCR1_Msk* = (0x0000FFFF shl TIM_CCR1_CCR1_Pos) ## !< 0x0000FFFF
  TIM_CCR1_CCR1* = TIM_CCR1_CCR1_Msk

## ******************  Bit definition for TIM_CCR2 register  ******************

const
  TIM_CCR2_CCR2_Pos* = (0)
  TIM_CCR2_CCR2_Msk* = (0x0000FFFF shl TIM_CCR2_CCR2_Pos) ## !< 0x0000FFFF
  TIM_CCR2_CCR2* = TIM_CCR2_CCR2_Msk

## ******************  Bit definition for TIM_CCR3 register  ******************

const
  TIM_CCR3_CCR3_Pos* = (0)
  TIM_CCR3_CCR3_Msk* = (0x0000FFFF shl TIM_CCR3_CCR3_Pos) ## !< 0x0000FFFF
  TIM_CCR3_CCR3* = TIM_CCR3_CCR3_Msk

## ******************  Bit definition for TIM_CCR4 register  ******************

const
  TIM_CCR4_CCR4_Pos* = (0)
  TIM_CCR4_CCR4_Msk* = (0x0000FFFF shl TIM_CCR4_CCR4_Pos) ## !< 0x0000FFFF
  TIM_CCR4_CCR4* = TIM_CCR4_CCR4_Msk

## ******************  Bit definition for TIM_DCR register  *******************

const
  TIM_DCR_DBA_Pos* = (0)
  TIM_DCR_DBA_Msk* = (0x0000001F shl TIM_DCR_DBA_Pos) ## !< 0x0000001F
  TIM_DCR_DBA* = TIM_DCR_DBA_Msk
  TIM_DCR_DBA_0* = (0x00000001 shl TIM_DCR_DBA_Pos) ## !< 0x00000001
  TIM_DCR_DBA_1* = (0x00000002 shl TIM_DCR_DBA_Pos) ## !< 0x00000002
  TIM_DCR_DBA_2* = (0x00000004 shl TIM_DCR_DBA_Pos) ## !< 0x00000004
  TIM_DCR_DBA_3* = (0x00000008 shl TIM_DCR_DBA_Pos) ## !< 0x00000008
  TIM_DCR_DBA_4* = (0x00000010 shl TIM_DCR_DBA_Pos) ## !< 0x00000010
  TIM_DCR_DBL_Pos* = (8)
  TIM_DCR_DBL_Msk* = (0x0000001F shl TIM_DCR_DBL_Pos) ## !< 0x00001F00
  TIM_DCR_DBL* = TIM_DCR_DBL_Msk
  TIM_DCR_DBL_0* = (0x00000001 shl TIM_DCR_DBL_Pos) ## !< 0x00000100
  TIM_DCR_DBL_1* = (0x00000002 shl TIM_DCR_DBL_Pos) ## !< 0x00000200
  TIM_DCR_DBL_2* = (0x00000004 shl TIM_DCR_DBL_Pos) ## !< 0x00000400
  TIM_DCR_DBL_3* = (0x00000008 shl TIM_DCR_DBL_Pos) ## !< 0x00000800
  TIM_DCR_DBL_4* = (0x00000010 shl TIM_DCR_DBL_Pos) ## !< 0x00001000

## ******************  Bit definition for TIM_DMAR register  ******************

const
  TIM_DMAR_DMAB_Pos* = (0)
  TIM_DMAR_DMAB_Msk* = (0x0000FFFF shl TIM_DMAR_DMAB_Pos) ## !< 0x0000FFFF
  TIM_DMAR_DMAB* = TIM_DMAR_DMAB_Msk

## ******************  Bit definition for TIM_OR register  ********************

const
  TIM_OR_TI1RMP_Pos* = (0)
  TIM_OR_TI1RMP_Msk* = (0x00000003 shl TIM_OR_TI1RMP_Pos) ## !< 0x00000003
  TIM_OR_TI1RMP* = TIM_OR_TI1RMP_Msk
  TIM_OR_TI1RMP_0* = (0x00000001 shl TIM_OR_TI1RMP_Pos) ## !< 0x00000001
  TIM_OR_TI1RMP_1* = (0x00000002 shl TIM_OR_TI1RMP_Pos) ## !< 0x00000002
  TIM_OR_ETR_RMP_Pos* = (2)
  TIM_OR_ETR_RMP_Msk* = (0x00000001 shl TIM_OR_ETR_RMP_Pos) ## !< 0x00000004
  TIM_OR_ETR_RMP* = TIM_OR_ETR_RMP_Msk
  TIM_OR_TI1_RMP_RI_Pos* = (3)
  TIM_OR_TI1_RMP_RI_Msk* = (0x00000001 shl TIM_OR_TI1_RMP_RI_Pos) ## !< 0x00000008
  TIM_OR_TI1_RMP_RI* = TIM_OR_TI1_RMP_RI_Msk

## ----------------------------------------------------------------------------

const
  TIM9_OR_ITR1_RMP_Pos* = (2)
  TIM9_OR_ITR1_RMP_Msk* = (0x00000001 shl TIM9_OR_ITR1_RMP_Pos) ## !< 0x00000004
  TIM9_OR_ITR1_RMP* = TIM9_OR_ITR1_RMP_Msk

## ----------------------------------------------------------------------------

const
  TIM2_OR_ITR1_RMP_Pos* = (0)
  TIM2_OR_ITR1_RMP_Msk* = (0x00000001 shl TIM2_OR_ITR1_RMP_Pos) ## !< 0x00000001
  TIM2_OR_ITR1_RMP* = TIM2_OR_ITR1_RMP_Msk

## ----------------------------------------------------------------------------

const
  TIM3_OR_ITR2_RMP_Pos* = (0)
  TIM3_OR_ITR2_RMP_Msk* = (0x00000001 shl TIM3_OR_ITR2_RMP_Pos) ## !< 0x00000001
  TIM3_OR_ITR2_RMP* = TIM3_OR_ITR2_RMP_Msk

## ----------------------------------------------------------------------------
## ****************************************************************************
##
##       Universal Synchronous Asynchronous Receiver Transmitter (USART)
##
## ****************************************************************************
## ******************  Bit definition for USART_SR register  ******************

const
  USART_SR_PE_Pos* = (0)
  USART_SR_PE_Msk* = (0x00000001 shl USART_SR_PE_Pos) ## !< 0x00000001
  USART_SR_PE* = USART_SR_PE_Msk
  USART_SR_FE_Pos* = (1)
  USART_SR_FE_Msk* = (0x00000001 shl USART_SR_FE_Pos) ## !< 0x00000002
  USART_SR_FE* = USART_SR_FE_Msk
  USART_SR_NE_Pos* = (2)
  USART_SR_NE_Msk* = (0x00000001 shl USART_SR_NE_Pos) ## !< 0x00000004
  USART_SR_NE* = USART_SR_NE_Msk
  USART_SR_ORE_Pos* = (3)
  USART_SR_ORE_Msk* = (0x00000001 shl USART_SR_ORE_Pos) ## !< 0x00000008
  USART_SR_ORE* = USART_SR_ORE_Msk
  USART_SR_IDLE_Pos* = (4)
  USART_SR_IDLE_Msk* = (0x00000001 shl USART_SR_IDLE_Pos) ## !< 0x00000010
  USART_SR_IDLE* = USART_SR_IDLE_Msk
  USART_SR_RXNE_Pos* = (5)
  USART_SR_RXNE_Msk* = (0x00000001 shl USART_SR_RXNE_Pos) ## !< 0x00000020
  USART_SR_RXNE* = USART_SR_RXNE_Msk
  USART_SR_TC_Pos* = (6)
  USART_SR_TC_Msk* = (0x00000001 shl USART_SR_TC_Pos) ## !< 0x00000040
  USART_SR_TC* = USART_SR_TC_Msk
  USART_SR_TXE_Pos* = (7)
  USART_SR_TXE_Msk* = (0x00000001 shl USART_SR_TXE_Pos) ## !< 0x00000080
  USART_SR_TXE* = USART_SR_TXE_Msk
  USART_SR_LBD_Pos* = (8)
  USART_SR_LBD_Msk* = (0x00000001 shl USART_SR_LBD_Pos) ## !< 0x00000100
  USART_SR_LBD* = USART_SR_LBD_Msk
  USART_SR_CTS_Pos* = (9)
  USART_SR_CTS_Msk* = (0x00000001 shl USART_SR_CTS_Pos) ## !< 0x00000200
  USART_SR_CTS* = USART_SR_CTS_Msk

## ******************  Bit definition for USART_DR register  ******************

const
  USART_DR_DR_Pos* = (0)
  USART_DR_DR_Msk* = (0x000001FF shl USART_DR_DR_Pos) ## !< 0x000001FF
  USART_DR_DR* = USART_DR_DR_Msk

## *****************  Bit definition for USART_BRR register  ******************

const
  USART_BRR_DIV_FRACTION_Pos* = (0)
  USART_BRR_DIV_FRACTION_Msk* = (0x0000000F shl USART_BRR_DIV_FRACTION_Pos) ## !< 0x0000000F
  USART_BRR_DIV_FRACTION* = USART_BRR_DIV_FRACTION_Msk
  USART_BRR_DIV_MANTISSA_Pos* = (4)
  USART_BRR_DIV_MANTISSA_Msk* = (0x00000FFF shl USART_BRR_DIV_MANTISSA_Pos) ## !< 0x0000FFF0
  USART_BRR_DIV_MANTISSA* = USART_BRR_DIV_MANTISSA_Msk

## *****************  Bit definition for USART_CR1 register  ******************

const
  USART_CR1_SBK_Pos* = (0)
  USART_CR1_SBK_Msk* = (0x00000001 shl USART_CR1_SBK_Pos) ## !< 0x00000001
  USART_CR1_SBK* = USART_CR1_SBK_Msk
  USART_CR1_RWU_Pos* = (1)
  USART_CR1_RWU_Msk* = (0x00000001 shl USART_CR1_RWU_Pos) ## !< 0x00000002
  USART_CR1_RWU* = USART_CR1_RWU_Msk
  USART_CR1_RE_Pos* = (2)
  USART_CR1_RE_Msk* = (0x00000001 shl USART_CR1_RE_Pos) ## !< 0x00000004
  USART_CR1_RE* = USART_CR1_RE_Msk
  USART_CR1_TE_Pos* = (3)
  USART_CR1_TE_Msk* = (0x00000001 shl USART_CR1_TE_Pos) ## !< 0x00000008
  USART_CR1_TE* = USART_CR1_TE_Msk
  USART_CR1_IDLEIE_Pos* = (4)
  USART_CR1_IDLEIE_Msk* = (0x00000001 shl USART_CR1_IDLEIE_Pos) ## !< 0x00000010
  USART_CR1_IDLEIE* = USART_CR1_IDLEIE_Msk
  USART_CR1_RXNEIE_Pos* = (5)
  USART_CR1_RXNEIE_Msk* = (0x00000001 shl USART_CR1_RXNEIE_Pos) ## !< 0x00000020
  USART_CR1_RXNEIE* = USART_CR1_RXNEIE_Msk
  USART_CR1_TCIE_Pos* = (6)
  USART_CR1_TCIE_Msk* = (0x00000001 shl USART_CR1_TCIE_Pos) ## !< 0x00000040
  USART_CR1_TCIE* = USART_CR1_TCIE_Msk
  USART_CR1_TXEIE_Pos* = (7)
  USART_CR1_TXEIE_Msk* = (0x00000001 shl USART_CR1_TXEIE_Pos) ## !< 0x00000080
  USART_CR1_TXEIE* = USART_CR1_TXEIE_Msk
  USART_CR1_PEIE_Pos* = (8)
  USART_CR1_PEIE_Msk* = (0x00000001 shl USART_CR1_PEIE_Pos) ## !< 0x00000100
  USART_CR1_PEIE* = USART_CR1_PEIE_Msk
  USART_CR1_PS_Pos* = (9)
  USART_CR1_PS_Msk* = (0x00000001 shl USART_CR1_PS_Pos) ## !< 0x00000200
  USART_CR1_PS* = USART_CR1_PS_Msk
  USART_CR1_PCE_Pos* = (10)
  USART_CR1_PCE_Msk* = (0x00000001 shl USART_CR1_PCE_Pos) ## !< 0x00000400
  USART_CR1_PCE* = USART_CR1_PCE_Msk
  USART_CR1_WAKE_Pos* = (11)
  USART_CR1_WAKE_Msk* = (0x00000001 shl USART_CR1_WAKE_Pos) ## !< 0x00000800
  USART_CR1_WAKE* = USART_CR1_WAKE_Msk
  USART_CR1_M_Pos* = (12)
  USART_CR1_M_Msk* = (0x00000001 shl USART_CR1_M_Pos) ## !< 0x00001000
  USART_CR1_M* = USART_CR1_M_Msk
  USART_CR1_UE_Pos* = (13)
  USART_CR1_UE_Msk* = (0x00000001 shl USART_CR1_UE_Pos) ## !< 0x00002000
  USART_CR1_UE* = USART_CR1_UE_Msk
  USART_CR1_OVER8_Pos* = (15)
  USART_CR1_OVER8_Msk* = (0x00000001 shl USART_CR1_OVER8_Pos) ## !< 0x00008000
  USART_CR1_OVER8* = USART_CR1_OVER8_Msk

## *****************  Bit definition for USART_CR2 register  ******************

const
  USART_CR2_ADD_Pos* = (0)
  USART_CR2_ADD_Msk* = (0x0000000F shl USART_CR2_ADD_Pos) ## !< 0x0000000F
  USART_CR2_ADD* = USART_CR2_ADD_Msk
  USART_CR2_LBDL_Pos* = (5)
  USART_CR2_LBDL_Msk* = (0x00000001 shl USART_CR2_LBDL_Pos) ## !< 0x00000020
  USART_CR2_LBDL* = USART_CR2_LBDL_Msk
  USART_CR2_LBDIE_Pos* = (6)
  USART_CR2_LBDIE_Msk* = (0x00000001 shl USART_CR2_LBDIE_Pos) ## !< 0x00000040
  USART_CR2_LBDIE* = USART_CR2_LBDIE_Msk
  USART_CR2_LBCL_Pos* = (8)
  USART_CR2_LBCL_Msk* = (0x00000001 shl USART_CR2_LBCL_Pos) ## !< 0x00000100
  USART_CR2_LBCL* = USART_CR2_LBCL_Msk
  USART_CR2_CPHA_Pos* = (9)
  USART_CR2_CPHA_Msk* = (0x00000001 shl USART_CR2_CPHA_Pos) ## !< 0x00000200
  USART_CR2_CPHA* = USART_CR2_CPHA_Msk
  USART_CR2_CPOL_Pos* = (10)
  USART_CR2_CPOL_Msk* = (0x00000001 shl USART_CR2_CPOL_Pos) ## !< 0x00000400
  USART_CR2_CPOL* = USART_CR2_CPOL_Msk
  USART_CR2_CLKEN_Pos* = (11)
  USART_CR2_CLKEN_Msk* = (0x00000001 shl USART_CR2_CLKEN_Pos) ## !< 0x00000800
  USART_CR2_CLKEN* = USART_CR2_CLKEN_Msk
  USART_CR2_STOP_Pos* = (12)
  USART_CR2_STOP_Msk* = (0x00000003 shl USART_CR2_STOP_Pos) ## !< 0x00003000
  USART_CR2_STOP* = USART_CR2_STOP_Msk
  USART_CR2_STOP_0* = (0x00000001 shl USART_CR2_STOP_Pos) ## !< 0x00001000
  USART_CR2_STOP_1* = (0x00000002 shl USART_CR2_STOP_Pos) ## !< 0x00002000
  USART_CR2_LINEN_Pos* = (14)
  USART_CR2_LINEN_Msk* = (0x00000001 shl USART_CR2_LINEN_Pos) ## !< 0x00004000
  USART_CR2_LINEN* = USART_CR2_LINEN_Msk

## *****************  Bit definition for USART_CR3 register  ******************

const
  USART_CR3_EIE_Pos* = (0)
  USART_CR3_EIE_Msk* = (0x00000001 shl USART_CR3_EIE_Pos) ## !< 0x00000001
  USART_CR3_EIE* = USART_CR3_EIE_Msk
  USART_CR3_IREN_Pos* = (1)
  USART_CR3_IREN_Msk* = (0x00000001 shl USART_CR3_IREN_Pos) ## !< 0x00000002
  USART_CR3_IREN* = USART_CR3_IREN_Msk
  USART_CR3_IRLP_Pos* = (2)
  USART_CR3_IRLP_Msk* = (0x00000001 shl USART_CR3_IRLP_Pos) ## !< 0x00000004
  USART_CR3_IRLP* = USART_CR3_IRLP_Msk
  USART_CR3_HDSEL_Pos* = (3)
  USART_CR3_HDSEL_Msk* = (0x00000001 shl USART_CR3_HDSEL_Pos) ## !< 0x00000008
  USART_CR3_HDSEL* = USART_CR3_HDSEL_Msk
  USART_CR3_NACK_Pos* = (4)
  USART_CR3_NACK_Msk* = (0x00000001 shl USART_CR3_NACK_Pos) ## !< 0x00000010
  USART_CR3_NACK* = USART_CR3_NACK_Msk
  USART_CR3_SCEN_Pos* = (5)
  USART_CR3_SCEN_Msk* = (0x00000001 shl USART_CR3_SCEN_Pos) ## !< 0x00000020
  USART_CR3_SCEN* = USART_CR3_SCEN_Msk
  USART_CR3_DMAR_Pos* = (6)
  USART_CR3_DMAR_Msk* = (0x00000001 shl USART_CR3_DMAR_Pos) ## !< 0x00000040
  USART_CR3_DMAR* = USART_CR3_DMAR_Msk
  USART_CR3_DMAT_Pos* = (7)
  USART_CR3_DMAT_Msk* = (0x00000001 shl USART_CR3_DMAT_Pos) ## !< 0x00000080
  USART_CR3_DMAT* = USART_CR3_DMAT_Msk
  USART_CR3_RTSE_Pos* = (8)
  USART_CR3_RTSE_Msk* = (0x00000001 shl USART_CR3_RTSE_Pos) ## !< 0x00000100
  USART_CR3_RTSE* = USART_CR3_RTSE_Msk
  USART_CR3_CTSE_Pos* = (9)
  USART_CR3_CTSE_Msk* = (0x00000001 shl USART_CR3_CTSE_Pos) ## !< 0x00000200
  USART_CR3_CTSE* = USART_CR3_CTSE_Msk
  USART_CR3_CTSIE_Pos* = (10)
  USART_CR3_CTSIE_Msk* = (0x00000001 shl USART_CR3_CTSIE_Pos) ## !< 0x00000400
  USART_CR3_CTSIE* = USART_CR3_CTSIE_Msk
  USART_CR3_ONEBIT_Pos* = (11)
  USART_CR3_ONEBIT_Msk* = (0x00000001 shl USART_CR3_ONEBIT_Pos) ## !< 0x00000800
  USART_CR3_ONEBIT* = USART_CR3_ONEBIT_Msk

## *****************  Bit definition for USART_GTPR register  *****************

const
  USART_GTPR_PSC_Pos* = (0)
  USART_GTPR_PSC_Msk* = (0x000000FF shl USART_GTPR_PSC_Pos) ## !< 0x000000FF
  USART_GTPR_PSC* = USART_GTPR_PSC_Msk
  USART_GTPR_PSC_0* = (0x00000001 shl USART_GTPR_PSC_Pos) ## !< 0x00000001
  USART_GTPR_PSC_1* = (0x00000002 shl USART_GTPR_PSC_Pos) ## !< 0x00000002
  USART_GTPR_PSC_2* = (0x00000004 shl USART_GTPR_PSC_Pos) ## !< 0x00000004
  USART_GTPR_PSC_3* = (0x00000008 shl USART_GTPR_PSC_Pos) ## !< 0x00000008
  USART_GTPR_PSC_4* = (0x00000010 shl USART_GTPR_PSC_Pos) ## !< 0x00000010
  USART_GTPR_PSC_5* = (0x00000020 shl USART_GTPR_PSC_Pos) ## !< 0x00000020
  USART_GTPR_PSC_6* = (0x00000040 shl USART_GTPR_PSC_Pos) ## !< 0x00000040
  USART_GTPR_PSC_7* = (0x00000080 shl USART_GTPR_PSC_Pos) ## !< 0x00000080
  USART_GTPR_GT_Pos* = (8)
  USART_GTPR_GT_Msk* = (0x000000FF shl USART_GTPR_GT_Pos) ## !< 0x0000FF00
  USART_GTPR_GT* = USART_GTPR_GT_Msk

## ****************************************************************************
##
##                      Universal Serial Bus (USB)
##
## ****************************************************************************
## !<Endpoint-specific registers

const
  USB_EP0R* = USB_BASE
  USB_EP1R* = (USB_BASE + 0x00000004) ## !< endpoint 1 register address
  USB_EP2R* = (USB_BASE + 0x00000008) ## !< endpoint 2 register address
  USB_EP3R* = (USB_BASE + 0x0000000C) ## !< endpoint 3 register address
  USB_EP4R* = (USB_BASE + 0x00000010) ## !< endpoint 4 register address
  USB_EP5R* = (USB_BASE + 0x00000014) ## !< endpoint 5 register address
  USB_EP6R* = (USB_BASE + 0x00000018) ## !< endpoint 6 register address
  USB_EP7R* = (USB_BASE + 0x0000001C) ## !< endpoint 7 register address

##  bit positions

const
  USB_EP_CTR_RX_Pos* = (15)
  USB_EP_CTR_RX_Msk* = (0x00000001 shl USB_EP_CTR_RX_Pos) ## !< 0x00008000
  USB_EP_CTR_RX* = USB_EP_CTR_RX_Msk
  USB_EP_DTOG_RX_Pos* = (14)
  USB_EP_DTOG_RX_Msk* = (0x00000001 shl USB_EP_DTOG_RX_Pos) ## !< 0x00004000
  USB_EP_DTOG_RX* = USB_EP_DTOG_RX_Msk
  USB_EPRX_STAT_Pos* = (12)
  USB_EPRX_STAT_Msk* = (0x00000003 shl USB_EPRX_STAT_Pos) ## !< 0x00003000
  USB_EPRX_STAT* = USB_EPRX_STAT_Msk
  USB_EP_SETUP_Pos* = (11)
  USB_EP_SETUP_Msk* = (0x00000001 shl USB_EP_SETUP_Pos) ## !< 0x00000800
  USB_EP_SETUP* = USB_EP_SETUP_Msk
  USB_EP_T_FIELD_Pos* = (9)
  USB_EP_T_FIELD_Msk* = (0x00000003 shl USB_EP_T_FIELD_Pos) ## !< 0x00000600
  USB_EP_T_FIELD* = USB_EP_T_FIELD_Msk
  USB_EP_KIND_Pos* = (8)
  USB_EP_KIND_Msk* = (0x00000001 shl USB_EP_KIND_Pos) ## !< 0x00000100
  USB_EP_KIND* = USB_EP_KIND_Msk
  USB_EP_CTR_TX_Pos* = (7)
  USB_EP_CTR_TX_Msk* = (0x00000001 shl USB_EP_CTR_TX_Pos) ## !< 0x00000080
  USB_EP_CTR_TX* = USB_EP_CTR_TX_Msk
  USB_EP_DTOG_TX_Pos* = (6)
  USB_EP_DTOG_TX_Msk* = (0x00000001 shl USB_EP_DTOG_TX_Pos) ## !< 0x00000040
  USB_EP_DTOG_TX* = USB_EP_DTOG_TX_Msk
  USB_EPTX_STAT_Pos* = (4)
  USB_EPTX_STAT_Msk* = (0x00000003 shl USB_EPTX_STAT_Pos) ## !< 0x00000030
  USB_EPTX_STAT* = USB_EPTX_STAT_Msk
  USB_EPADDR_FIELD_Pos* = (0)
  USB_EPADDR_FIELD_Msk* = (0x0000000F shl USB_EPADDR_FIELD_Pos) ## !< 0x0000000F
  USB_EPADDR_FIELD* = USB_EPADDR_FIELD_Msk

##  EndPoint REGister MASK (no toggle fields)

const
  USB_EPREG_MASK* = (USB_EP_CTR_RX or USB_EP_SETUP or USB_EP_T_FIELD or USB_EP_KIND or
      USB_EP_CTR_TX or USB_EPADDR_FIELD)

## !< EP_TYPE[1:0] EndPoint TYPE

const
  USB_EP_TYPE_MASK_Pos* = (9)
  USB_EP_TYPE_MASK_Msk* = (0x00000003 shl USB_EP_TYPE_MASK_Pos) ## !< 0x00000600
  USB_EP_TYPE_MASK* = USB_EP_TYPE_MASK_Msk
  USB_EP_BULK* = (0x00000000)   ## !< EndPoint BULK
  USB_EP_CONTROL* = (0x00000200) ## !< EndPoint CONTROL
  USB_EP_ISOCHRONOUS* = (0x00000400) ## !< EndPoint ISOCHRONOUS
  USB_EP_INTERRUPT* = (0x00000600) ## !< EndPoint INTERRUPT
  USB_EP_T_MASK* = (not USB_EP_T_FIELD and USB_EPREG_MASK)
  USB_EPKIND_MASK* = (not USB_EP_KIND and USB_EPREG_MASK) ## !< EP_KIND EndPoint KIND

## !< STAT_TX[1:0] STATus for TX transfer

const
  USB_EP_TX_DIS* = (0x00000000) ## !< EndPoint TX DISabled
  USB_EP_TX_STALL* = (0x00000010) ## !< EndPoint TX STALLed
  USB_EP_TX_NAK* = (0x00000020) ## !< EndPoint TX NAKed
  USB_EP_TX_VALID* = (0x00000030) ## !< EndPoint TX VALID
  USB_EPTX_DTOG1* = (0x00000010) ## !< EndPoint TX Data TOGgle bit1
  USB_EPTX_DTOG2* = (0x00000020) ## !< EndPoint TX Data TOGgle bit2
  USB_EPTX_DTOGMASK* = (USB_EPTX_STAT or USB_EPREG_MASK)

## !< STAT_RX[1:0] STATus for RX transfer

const
  USB_EP_RX_DIS* = (0x00000000) ## !< EndPoint RX DISabled
  USB_EP_RX_STALL* = (0x00001000) ## !< EndPoint RX STALLed
  USB_EP_RX_NAK* = (0x00002000) ## !< EndPoint RX NAKed
  USB_EP_RX_VALID* = (0x00003000) ## !< EndPoint RX VALID
  USB_EPRX_DTOG1* = (0x00001000) ## !< EndPoint RX Data TOGgle bit1
  USB_EPRX_DTOG2* = (0x00002000) ## !< EndPoint RX Data TOGgle bit1
  USB_EPRX_DTOGMASK* = (USB_EPRX_STAT or USB_EPREG_MASK)

## ******************  Bit definition for USB_EP0R register  ******************

const
  USB_EP0R_EA_Pos* = (0)
  USB_EP0R_EA_Msk* = (0x0000000F shl USB_EP0R_EA_Pos) ## !< 0x0000000F
  USB_EP0R_EA* = USB_EP0R_EA_Msk
  USB_EP0R_STAT_TX_Pos* = (4)
  USB_EP0R_STAT_TX_Msk* = (0x00000003 shl USB_EP0R_STAT_TX_Pos) ## !< 0x00000030
  USB_EP0R_STAT_TX* = USB_EP0R_STAT_TX_Msk
  USB_EP0R_STAT_TX_0* = (0x00000001 shl USB_EP0R_STAT_TX_Pos) ## !< 0x00000010
  USB_EP0R_STAT_TX_1* = (0x00000002 shl USB_EP0R_STAT_TX_Pos) ## !< 0x00000020
  USB_EP0R_DTOG_TX_Pos* = (6)
  USB_EP0R_DTOG_TX_Msk* = (0x00000001 shl USB_EP0R_DTOG_TX_Pos) ## !< 0x00000040
  USB_EP0R_DTOG_TX* = USB_EP0R_DTOG_TX_Msk
  USB_EP0R_CTR_TX_Pos* = (7)
  USB_EP0R_CTR_TX_Msk* = (0x00000001 shl USB_EP0R_CTR_TX_Pos) ## !< 0x00000080
  USB_EP0R_CTR_TX* = USB_EP0R_CTR_TX_Msk
  USB_EP0R_EP_KIND_Pos* = (8)
  USB_EP0R_EP_KIND_Msk* = (0x00000001 shl USB_EP0R_EP_KIND_Pos) ## !< 0x00000100
  USB_EP0R_EP_KIND* = USB_EP0R_EP_KIND_Msk
  USB_EP0R_EP_TYPE_Pos* = (9)
  USB_EP0R_EP_TYPE_Msk* = (0x00000003 shl USB_EP0R_EP_TYPE_Pos) ## !< 0x00000600
  USB_EP0R_EP_TYPE* = USB_EP0R_EP_TYPE_Msk
  USB_EP0R_EP_TYPE_0* = (0x00000001 shl USB_EP0R_EP_TYPE_Pos) ## !< 0x00000200
  USB_EP0R_EP_TYPE_1* = (0x00000002 shl USB_EP0R_EP_TYPE_Pos) ## !< 0x00000400
  USB_EP0R_SETUP_Pos* = (11)
  USB_EP0R_SETUP_Msk* = (0x00000001 shl USB_EP0R_SETUP_Pos) ## !< 0x00000800
  USB_EP0R_SETUP* = USB_EP0R_SETUP_Msk
  USB_EP0R_STAT_RX_Pos* = (12)
  USB_EP0R_STAT_RX_Msk* = (0x00000003 shl USB_EP0R_STAT_RX_Pos) ## !< 0x00003000
  USB_EP0R_STAT_RX* = USB_EP0R_STAT_RX_Msk
  USB_EP0R_STAT_RX_0* = (0x00000001 shl USB_EP0R_STAT_RX_Pos) ## !< 0x00001000
  USB_EP0R_STAT_RX_1* = (0x00000002 shl USB_EP0R_STAT_RX_Pos) ## !< 0x00002000
  USB_EP0R_DTOG_RX_Pos* = (14)
  USB_EP0R_DTOG_RX_Msk* = (0x00000001 shl USB_EP0R_DTOG_RX_Pos) ## !< 0x00004000
  USB_EP0R_DTOG_RX* = USB_EP0R_DTOG_RX_Msk
  USB_EP0R_CTR_RX_Pos* = (15)
  USB_EP0R_CTR_RX_Msk* = (0x00000001 shl USB_EP0R_CTR_RX_Pos) ## !< 0x00008000
  USB_EP0R_CTR_RX* = USB_EP0R_CTR_RX_Msk

## ******************  Bit definition for USB_EP1R register  ******************

const
  USB_EP1R_EA_Pos* = (0)
  USB_EP1R_EA_Msk* = (0x0000000F shl USB_EP1R_EA_Pos) ## !< 0x0000000F
  USB_EP1R_EA* = USB_EP1R_EA_Msk
  USB_EP1R_STAT_TX_Pos* = (4)
  USB_EP1R_STAT_TX_Msk* = (0x00000003 shl USB_EP1R_STAT_TX_Pos) ## !< 0x00000030
  USB_EP1R_STAT_TX* = USB_EP1R_STAT_TX_Msk
  USB_EP1R_STAT_TX_0* = (0x00000001 shl USB_EP1R_STAT_TX_Pos) ## !< 0x00000010
  USB_EP1R_STAT_TX_1* = (0x00000002 shl USB_EP1R_STAT_TX_Pos) ## !< 0x00000020
  USB_EP1R_DTOG_TX_Pos* = (6)
  USB_EP1R_DTOG_TX_Msk* = (0x00000001 shl USB_EP1R_DTOG_TX_Pos) ## !< 0x00000040
  USB_EP1R_DTOG_TX* = USB_EP1R_DTOG_TX_Msk
  USB_EP1R_CTR_TX_Pos* = (7)
  USB_EP1R_CTR_TX_Msk* = (0x00000001 shl USB_EP1R_CTR_TX_Pos) ## !< 0x00000080
  USB_EP1R_CTR_TX* = USB_EP1R_CTR_TX_Msk
  USB_EP1R_EP_KIND_Pos* = (8)
  USB_EP1R_EP_KIND_Msk* = (0x00000001 shl USB_EP1R_EP_KIND_Pos) ## !< 0x00000100
  USB_EP1R_EP_KIND* = USB_EP1R_EP_KIND_Msk
  USB_EP1R_EP_TYPE_Pos* = (9)
  USB_EP1R_EP_TYPE_Msk* = (0x00000003 shl USB_EP1R_EP_TYPE_Pos) ## !< 0x00000600
  USB_EP1R_EP_TYPE* = USB_EP1R_EP_TYPE_Msk
  USB_EP1R_EP_TYPE_0* = (0x00000001 shl USB_EP1R_EP_TYPE_Pos) ## !< 0x00000200
  USB_EP1R_EP_TYPE_1* = (0x00000002 shl USB_EP1R_EP_TYPE_Pos) ## !< 0x00000400
  USB_EP1R_SETUP_Pos* = (11)
  USB_EP1R_SETUP_Msk* = (0x00000001 shl USB_EP1R_SETUP_Pos) ## !< 0x00000800
  USB_EP1R_SETUP* = USB_EP1R_SETUP_Msk
  USB_EP1R_STAT_RX_Pos* = (12)
  USB_EP1R_STAT_RX_Msk* = (0x00000003 shl USB_EP1R_STAT_RX_Pos) ## !< 0x00003000
  USB_EP1R_STAT_RX* = USB_EP1R_STAT_RX_Msk
  USB_EP1R_STAT_RX_0* = (0x00000001 shl USB_EP1R_STAT_RX_Pos) ## !< 0x00001000
  USB_EP1R_STAT_RX_1* = (0x00000002 shl USB_EP1R_STAT_RX_Pos) ## !< 0x00002000
  USB_EP1R_DTOG_RX_Pos* = (14)
  USB_EP1R_DTOG_RX_Msk* = (0x00000001 shl USB_EP1R_DTOG_RX_Pos) ## !< 0x00004000
  USB_EP1R_DTOG_RX* = USB_EP1R_DTOG_RX_Msk
  USB_EP1R_CTR_RX_Pos* = (15)
  USB_EP1R_CTR_RX_Msk* = (0x00000001 shl USB_EP1R_CTR_RX_Pos) ## !< 0x00008000
  USB_EP1R_CTR_RX* = USB_EP1R_CTR_RX_Msk

## ******************  Bit definition for USB_EP2R register  ******************

const
  USB_EP2R_EA_Pos* = (0)
  USB_EP2R_EA_Msk* = (0x0000000F shl USB_EP2R_EA_Pos) ## !< 0x0000000F
  USB_EP2R_EA* = USB_EP2R_EA_Msk
  USB_EP2R_STAT_TX_Pos* = (4)
  USB_EP2R_STAT_TX_Msk* = (0x00000003 shl USB_EP2R_STAT_TX_Pos) ## !< 0x00000030
  USB_EP2R_STAT_TX* = USB_EP2R_STAT_TX_Msk
  USB_EP2R_STAT_TX_0* = (0x00000001 shl USB_EP2R_STAT_TX_Pos) ## !< 0x00000010
  USB_EP2R_STAT_TX_1* = (0x00000002 shl USB_EP2R_STAT_TX_Pos) ## !< 0x00000020
  USB_EP2R_DTOG_TX_Pos* = (6)
  USB_EP2R_DTOG_TX_Msk* = (0x00000001 shl USB_EP2R_DTOG_TX_Pos) ## !< 0x00000040
  USB_EP2R_DTOG_TX* = USB_EP2R_DTOG_TX_Msk
  USB_EP2R_CTR_TX_Pos* = (7)
  USB_EP2R_CTR_TX_Msk* = (0x00000001 shl USB_EP2R_CTR_TX_Pos) ## !< 0x00000080
  USB_EP2R_CTR_TX* = USB_EP2R_CTR_TX_Msk
  USB_EP2R_EP_KIND_Pos* = (8)
  USB_EP2R_EP_KIND_Msk* = (0x00000001 shl USB_EP2R_EP_KIND_Pos) ## !< 0x00000100
  USB_EP2R_EP_KIND* = USB_EP2R_EP_KIND_Msk
  USB_EP2R_EP_TYPE_Pos* = (9)
  USB_EP2R_EP_TYPE_Msk* = (0x00000003 shl USB_EP2R_EP_TYPE_Pos) ## !< 0x00000600
  USB_EP2R_EP_TYPE* = USB_EP2R_EP_TYPE_Msk
  USB_EP2R_EP_TYPE_0* = (0x00000001 shl USB_EP2R_EP_TYPE_Pos) ## !< 0x00000200
  USB_EP2R_EP_TYPE_1* = (0x00000002 shl USB_EP2R_EP_TYPE_Pos) ## !< 0x00000400
  USB_EP2R_SETUP_Pos* = (11)
  USB_EP2R_SETUP_Msk* = (0x00000001 shl USB_EP2R_SETUP_Pos) ## !< 0x00000800
  USB_EP2R_SETUP* = USB_EP2R_SETUP_Msk
  USB_EP2R_STAT_RX_Pos* = (12)
  USB_EP2R_STAT_RX_Msk* = (0x00000003 shl USB_EP2R_STAT_RX_Pos) ## !< 0x00003000
  USB_EP2R_STAT_RX* = USB_EP2R_STAT_RX_Msk
  USB_EP2R_STAT_RX_0* = (0x00000001 shl USB_EP2R_STAT_RX_Pos) ## !< 0x00001000
  USB_EP2R_STAT_RX_1* = (0x00000002 shl USB_EP2R_STAT_RX_Pos) ## !< 0x00002000
  USB_EP2R_DTOG_RX_Pos* = (14)
  USB_EP2R_DTOG_RX_Msk* = (0x00000001 shl USB_EP2R_DTOG_RX_Pos) ## !< 0x00004000
  USB_EP2R_DTOG_RX* = USB_EP2R_DTOG_RX_Msk
  USB_EP2R_CTR_RX_Pos* = (15)
  USB_EP2R_CTR_RX_Msk* = (0x00000001 shl USB_EP2R_CTR_RX_Pos) ## !< 0x00008000
  USB_EP2R_CTR_RX* = USB_EP2R_CTR_RX_Msk

## ******************  Bit definition for USB_EP3R register  ******************

const
  USB_EP3R_EA_Pos* = (0)
  USB_EP3R_EA_Msk* = (0x0000000F shl USB_EP3R_EA_Pos) ## !< 0x0000000F
  USB_EP3R_EA* = USB_EP3R_EA_Msk
  USB_EP3R_STAT_TX_Pos* = (4)
  USB_EP3R_STAT_TX_Msk* = (0x00000003 shl USB_EP3R_STAT_TX_Pos) ## !< 0x00000030
  USB_EP3R_STAT_TX* = USB_EP3R_STAT_TX_Msk
  USB_EP3R_STAT_TX_0* = (0x00000001 shl USB_EP3R_STAT_TX_Pos) ## !< 0x00000010
  USB_EP3R_STAT_TX_1* = (0x00000002 shl USB_EP3R_STAT_TX_Pos) ## !< 0x00000020
  USB_EP3R_DTOG_TX_Pos* = (6)
  USB_EP3R_DTOG_TX_Msk* = (0x00000001 shl USB_EP3R_DTOG_TX_Pos) ## !< 0x00000040
  USB_EP3R_DTOG_TX* = USB_EP3R_DTOG_TX_Msk
  USB_EP3R_CTR_TX_Pos* = (7)
  USB_EP3R_CTR_TX_Msk* = (0x00000001 shl USB_EP3R_CTR_TX_Pos) ## !< 0x00000080
  USB_EP3R_CTR_TX* = USB_EP3R_CTR_TX_Msk
  USB_EP3R_EP_KIND_Pos* = (8)
  USB_EP3R_EP_KIND_Msk* = (0x00000001 shl USB_EP3R_EP_KIND_Pos) ## !< 0x00000100
  USB_EP3R_EP_KIND* = USB_EP3R_EP_KIND_Msk
  USB_EP3R_EP_TYPE_Pos* = (9)
  USB_EP3R_EP_TYPE_Msk* = (0x00000003 shl USB_EP3R_EP_TYPE_Pos) ## !< 0x00000600
  USB_EP3R_EP_TYPE* = USB_EP3R_EP_TYPE_Msk
  USB_EP3R_EP_TYPE_0* = (0x00000001 shl USB_EP3R_EP_TYPE_Pos) ## !< 0x00000200
  USB_EP3R_EP_TYPE_1* = (0x00000002 shl USB_EP3R_EP_TYPE_Pos) ## !< 0x00000400
  USB_EP3R_SETUP_Pos* = (11)
  USB_EP3R_SETUP_Msk* = (0x00000001 shl USB_EP3R_SETUP_Pos) ## !< 0x00000800
  USB_EP3R_SETUP* = USB_EP3R_SETUP_Msk
  USB_EP3R_STAT_RX_Pos* = (12)
  USB_EP3R_STAT_RX_Msk* = (0x00000003 shl USB_EP3R_STAT_RX_Pos) ## !< 0x00003000
  USB_EP3R_STAT_RX* = USB_EP3R_STAT_RX_Msk
  USB_EP3R_STAT_RX_0* = (0x00000001 shl USB_EP3R_STAT_RX_Pos) ## !< 0x00001000
  USB_EP3R_STAT_RX_1* = (0x00000002 shl USB_EP3R_STAT_RX_Pos) ## !< 0x00002000
  USB_EP3R_DTOG_RX_Pos* = (14)
  USB_EP3R_DTOG_RX_Msk* = (0x00000001 shl USB_EP3R_DTOG_RX_Pos) ## !< 0x00004000
  USB_EP3R_DTOG_RX* = USB_EP3R_DTOG_RX_Msk
  USB_EP3R_CTR_RX_Pos* = (15)
  USB_EP3R_CTR_RX_Msk* = (0x00000001 shl USB_EP3R_CTR_RX_Pos) ## !< 0x00008000
  USB_EP3R_CTR_RX* = USB_EP3R_CTR_RX_Msk

## ******************  Bit definition for USB_EP4R register  ******************

const
  USB_EP4R_EA_Pos* = (0)
  USB_EP4R_EA_Msk* = (0x0000000F shl USB_EP4R_EA_Pos) ## !< 0x0000000F
  USB_EP4R_EA* = USB_EP4R_EA_Msk
  USB_EP4R_STAT_TX_Pos* = (4)
  USB_EP4R_STAT_TX_Msk* = (0x00000003 shl USB_EP4R_STAT_TX_Pos) ## !< 0x00000030
  USB_EP4R_STAT_TX* = USB_EP4R_STAT_TX_Msk
  USB_EP4R_STAT_TX_0* = (0x00000001 shl USB_EP4R_STAT_TX_Pos) ## !< 0x00000010
  USB_EP4R_STAT_TX_1* = (0x00000002 shl USB_EP4R_STAT_TX_Pos) ## !< 0x00000020
  USB_EP4R_DTOG_TX_Pos* = (6)
  USB_EP4R_DTOG_TX_Msk* = (0x00000001 shl USB_EP4R_DTOG_TX_Pos) ## !< 0x00000040
  USB_EP4R_DTOG_TX* = USB_EP4R_DTOG_TX_Msk
  USB_EP4R_CTR_TX_Pos* = (7)
  USB_EP4R_CTR_TX_Msk* = (0x00000001 shl USB_EP4R_CTR_TX_Pos) ## !< 0x00000080
  USB_EP4R_CTR_TX* = USB_EP4R_CTR_TX_Msk
  USB_EP4R_EP_KIND_Pos* = (8)
  USB_EP4R_EP_KIND_Msk* = (0x00000001 shl USB_EP4R_EP_KIND_Pos) ## !< 0x00000100
  USB_EP4R_EP_KIND* = USB_EP4R_EP_KIND_Msk
  USB_EP4R_EP_TYPE_Pos* = (9)
  USB_EP4R_EP_TYPE_Msk* = (0x00000003 shl USB_EP4R_EP_TYPE_Pos) ## !< 0x00000600
  USB_EP4R_EP_TYPE* = USB_EP4R_EP_TYPE_Msk
  USB_EP4R_EP_TYPE_0* = (0x00000001 shl USB_EP4R_EP_TYPE_Pos) ## !< 0x00000200
  USB_EP4R_EP_TYPE_1* = (0x00000002 shl USB_EP4R_EP_TYPE_Pos) ## !< 0x00000400
  USB_EP4R_SETUP_Pos* = (11)
  USB_EP4R_SETUP_Msk* = (0x00000001 shl USB_EP4R_SETUP_Pos) ## !< 0x00000800
  USB_EP4R_SETUP* = USB_EP4R_SETUP_Msk
  USB_EP4R_STAT_RX_Pos* = (12)
  USB_EP4R_STAT_RX_Msk* = (0x00000003 shl USB_EP4R_STAT_RX_Pos) ## !< 0x00003000
  USB_EP4R_STAT_RX* = USB_EP4R_STAT_RX_Msk
  USB_EP4R_STAT_RX_0* = (0x00000001 shl USB_EP4R_STAT_RX_Pos) ## !< 0x00001000
  USB_EP4R_STAT_RX_1* = (0x00000002 shl USB_EP4R_STAT_RX_Pos) ## !< 0x00002000
  USB_EP4R_DTOG_RX_Pos* = (14)
  USB_EP4R_DTOG_RX_Msk* = (0x00000001 shl USB_EP4R_DTOG_RX_Pos) ## !< 0x00004000
  USB_EP4R_DTOG_RX* = USB_EP4R_DTOG_RX_Msk
  USB_EP4R_CTR_RX_Pos* = (15)
  USB_EP4R_CTR_RX_Msk* = (0x00000001 shl USB_EP4R_CTR_RX_Pos) ## !< 0x00008000
  USB_EP4R_CTR_RX* = USB_EP4R_CTR_RX_Msk

## ******************  Bit definition for USB_EP5R register  ******************

const
  USB_EP5R_EA_Pos* = (0)
  USB_EP5R_EA_Msk* = (0x0000000F shl USB_EP5R_EA_Pos) ## !< 0x0000000F
  USB_EP5R_EA* = USB_EP5R_EA_Msk
  USB_EP5R_STAT_TX_Pos* = (4)
  USB_EP5R_STAT_TX_Msk* = (0x00000003 shl USB_EP5R_STAT_TX_Pos) ## !< 0x00000030
  USB_EP5R_STAT_TX* = USB_EP5R_STAT_TX_Msk
  USB_EP5R_STAT_TX_0* = (0x00000001 shl USB_EP5R_STAT_TX_Pos) ## !< 0x00000010
  USB_EP5R_STAT_TX_1* = (0x00000002 shl USB_EP5R_STAT_TX_Pos) ## !< 0x00000020
  USB_EP5R_DTOG_TX_Pos* = (6)
  USB_EP5R_DTOG_TX_Msk* = (0x00000001 shl USB_EP5R_DTOG_TX_Pos) ## !< 0x00000040
  USB_EP5R_DTOG_TX* = USB_EP5R_DTOG_TX_Msk
  USB_EP5R_CTR_TX_Pos* = (7)
  USB_EP5R_CTR_TX_Msk* = (0x00000001 shl USB_EP5R_CTR_TX_Pos) ## !< 0x00000080
  USB_EP5R_CTR_TX* = USB_EP5R_CTR_TX_Msk
  USB_EP5R_EP_KIND_Pos* = (8)
  USB_EP5R_EP_KIND_Msk* = (0x00000001 shl USB_EP5R_EP_KIND_Pos) ## !< 0x00000100
  USB_EP5R_EP_KIND* = USB_EP5R_EP_KIND_Msk
  USB_EP5R_EP_TYPE_Pos* = (9)
  USB_EP5R_EP_TYPE_Msk* = (0x00000003 shl USB_EP5R_EP_TYPE_Pos) ## !< 0x00000600
  USB_EP5R_EP_TYPE* = USB_EP5R_EP_TYPE_Msk
  USB_EP5R_EP_TYPE_0* = (0x00000001 shl USB_EP5R_EP_TYPE_Pos) ## !< 0x00000200
  USB_EP5R_EP_TYPE_1* = (0x00000002 shl USB_EP5R_EP_TYPE_Pos) ## !< 0x00000400
  USB_EP5R_SETUP_Pos* = (11)
  USB_EP5R_SETUP_Msk* = (0x00000001 shl USB_EP5R_SETUP_Pos) ## !< 0x00000800
  USB_EP5R_SETUP* = USB_EP5R_SETUP_Msk
  USB_EP5R_STAT_RX_Pos* = (12)
  USB_EP5R_STAT_RX_Msk* = (0x00000003 shl USB_EP5R_STAT_RX_Pos) ## !< 0x00003000
  USB_EP5R_STAT_RX* = USB_EP5R_STAT_RX_Msk
  USB_EP5R_STAT_RX_0* = (0x00000001 shl USB_EP5R_STAT_RX_Pos) ## !< 0x00001000
  USB_EP5R_STAT_RX_1* = (0x00000002 shl USB_EP5R_STAT_RX_Pos) ## !< 0x00002000
  USB_EP5R_DTOG_RX_Pos* = (14)
  USB_EP5R_DTOG_RX_Msk* = (0x00000001 shl USB_EP5R_DTOG_RX_Pos) ## !< 0x00004000
  USB_EP5R_DTOG_RX* = USB_EP5R_DTOG_RX_Msk
  USB_EP5R_CTR_RX_Pos* = (15)
  USB_EP5R_CTR_RX_Msk* = (0x00000001 shl USB_EP5R_CTR_RX_Pos) ## !< 0x00008000
  USB_EP5R_CTR_RX* = USB_EP5R_CTR_RX_Msk

## ******************  Bit definition for USB_EP6R register  ******************

const
  USB_EP6R_EA_Pos* = (0)
  USB_EP6R_EA_Msk* = (0x0000000F shl USB_EP6R_EA_Pos) ## !< 0x0000000F
  USB_EP6R_EA* = USB_EP6R_EA_Msk
  USB_EP6R_STAT_TX_Pos* = (4)
  USB_EP6R_STAT_TX_Msk* = (0x00000003 shl USB_EP6R_STAT_TX_Pos) ## !< 0x00000030
  USB_EP6R_STAT_TX* = USB_EP6R_STAT_TX_Msk
  USB_EP6R_STAT_TX_0* = (0x00000001 shl USB_EP6R_STAT_TX_Pos) ## !< 0x00000010
  USB_EP6R_STAT_TX_1* = (0x00000002 shl USB_EP6R_STAT_TX_Pos) ## !< 0x00000020
  USB_EP6R_DTOG_TX_Pos* = (6)
  USB_EP6R_DTOG_TX_Msk* = (0x00000001 shl USB_EP6R_DTOG_TX_Pos) ## !< 0x00000040
  USB_EP6R_DTOG_TX* = USB_EP6R_DTOG_TX_Msk
  USB_EP6R_CTR_TX_Pos* = (7)
  USB_EP6R_CTR_TX_Msk* = (0x00000001 shl USB_EP6R_CTR_TX_Pos) ## !< 0x00000080
  USB_EP6R_CTR_TX* = USB_EP6R_CTR_TX_Msk
  USB_EP6R_EP_KIND_Pos* = (8)
  USB_EP6R_EP_KIND_Msk* = (0x00000001 shl USB_EP6R_EP_KIND_Pos) ## !< 0x00000100
  USB_EP6R_EP_KIND* = USB_EP6R_EP_KIND_Msk
  USB_EP6R_EP_TYPE_Pos* = (9)
  USB_EP6R_EP_TYPE_Msk* = (0x00000003 shl USB_EP6R_EP_TYPE_Pos) ## !< 0x00000600
  USB_EP6R_EP_TYPE* = USB_EP6R_EP_TYPE_Msk
  USB_EP6R_EP_TYPE_0* = (0x00000001 shl USB_EP6R_EP_TYPE_Pos) ## !< 0x00000200
  USB_EP6R_EP_TYPE_1* = (0x00000002 shl USB_EP6R_EP_TYPE_Pos) ## !< 0x00000400
  USB_EP6R_SETUP_Pos* = (11)
  USB_EP6R_SETUP_Msk* = (0x00000001 shl USB_EP6R_SETUP_Pos) ## !< 0x00000800
  USB_EP6R_SETUP* = USB_EP6R_SETUP_Msk
  USB_EP6R_STAT_RX_Pos* = (12)
  USB_EP6R_STAT_RX_Msk* = (0x00000003 shl USB_EP6R_STAT_RX_Pos) ## !< 0x00003000
  USB_EP6R_STAT_RX* = USB_EP6R_STAT_RX_Msk
  USB_EP6R_STAT_RX_0* = (0x00000001 shl USB_EP6R_STAT_RX_Pos) ## !< 0x00001000
  USB_EP6R_STAT_RX_1* = (0x00000002 shl USB_EP6R_STAT_RX_Pos) ## !< 0x00002000
  USB_EP6R_DTOG_RX_Pos* = (14)
  USB_EP6R_DTOG_RX_Msk* = (0x00000001 shl USB_EP6R_DTOG_RX_Pos) ## !< 0x00004000
  USB_EP6R_DTOG_RX* = USB_EP6R_DTOG_RX_Msk
  USB_EP6R_CTR_RX_Pos* = (15)
  USB_EP6R_CTR_RX_Msk* = (0x00000001 shl USB_EP6R_CTR_RX_Pos) ## !< 0x00008000
  USB_EP6R_CTR_RX* = USB_EP6R_CTR_RX_Msk

## ******************  Bit definition for USB_EP7R register  ******************

const
  USB_EP7R_EA_Pos* = (0)
  USB_EP7R_EA_Msk* = (0x0000000F shl USB_EP7R_EA_Pos) ## !< 0x0000000F
  USB_EP7R_EA* = USB_EP7R_EA_Msk
  USB_EP7R_STAT_TX_Pos* = (4)
  USB_EP7R_STAT_TX_Msk* = (0x00000003 shl USB_EP7R_STAT_TX_Pos) ## !< 0x00000030
  USB_EP7R_STAT_TX* = USB_EP7R_STAT_TX_Msk
  USB_EP7R_STAT_TX_0* = (0x00000001 shl USB_EP7R_STAT_TX_Pos) ## !< 0x00000010
  USB_EP7R_STAT_TX_1* = (0x00000002 shl USB_EP7R_STAT_TX_Pos) ## !< 0x00000020
  USB_EP7R_DTOG_TX_Pos* = (6)
  USB_EP7R_DTOG_TX_Msk* = (0x00000001 shl USB_EP7R_DTOG_TX_Pos) ## !< 0x00000040
  USB_EP7R_DTOG_TX* = USB_EP7R_DTOG_TX_Msk
  USB_EP7R_CTR_TX_Pos* = (7)
  USB_EP7R_CTR_TX_Msk* = (0x00000001 shl USB_EP7R_CTR_TX_Pos) ## !< 0x00000080
  USB_EP7R_CTR_TX* = USB_EP7R_CTR_TX_Msk
  USB_EP7R_EP_KIND_Pos* = (8)
  USB_EP7R_EP_KIND_Msk* = (0x00000001 shl USB_EP7R_EP_KIND_Pos) ## !< 0x00000100
  USB_EP7R_EP_KIND* = USB_EP7R_EP_KIND_Msk
  USB_EP7R_EP_TYPE_Pos* = (9)
  USB_EP7R_EP_TYPE_Msk* = (0x00000003 shl USB_EP7R_EP_TYPE_Pos) ## !< 0x00000600
  USB_EP7R_EP_TYPE* = USB_EP7R_EP_TYPE_Msk
  USB_EP7R_EP_TYPE_0* = (0x00000001 shl USB_EP7R_EP_TYPE_Pos) ## !< 0x00000200
  USB_EP7R_EP_TYPE_1* = (0x00000002 shl USB_EP7R_EP_TYPE_Pos) ## !< 0x00000400
  USB_EP7R_SETUP_Pos* = (11)
  USB_EP7R_SETUP_Msk* = (0x00000001 shl USB_EP7R_SETUP_Pos) ## !< 0x00000800
  USB_EP7R_SETUP* = USB_EP7R_SETUP_Msk
  USB_EP7R_STAT_RX_Pos* = (12)
  USB_EP7R_STAT_RX_Msk* = (0x00000003 shl USB_EP7R_STAT_RX_Pos) ## !< 0x00003000
  USB_EP7R_STAT_RX* = USB_EP7R_STAT_RX_Msk
  USB_EP7R_STAT_RX_0* = (0x00000001 shl USB_EP7R_STAT_RX_Pos) ## !< 0x00001000
  USB_EP7R_STAT_RX_1* = (0x00000002 shl USB_EP7R_STAT_RX_Pos) ## !< 0x00002000
  USB_EP7R_DTOG_RX_Pos* = (14)
  USB_EP7R_DTOG_RX_Msk* = (0x00000001 shl USB_EP7R_DTOG_RX_Pos) ## !< 0x00004000
  USB_EP7R_DTOG_RX* = USB_EP7R_DTOG_RX_Msk
  USB_EP7R_CTR_RX_Pos* = (15)
  USB_EP7R_CTR_RX_Msk* = (0x00000001 shl USB_EP7R_CTR_RX_Pos) ## !< 0x00008000
  USB_EP7R_CTR_RX* = USB_EP7R_CTR_RX_Msk

## !<Common registers

const
  USB_CNTR* = (USB_BASE + 0x00000040) ## !< Control register
  USB_ISTR* = (USB_BASE + 0x00000044) ## !< Interrupt status register
  USB_FNR* = (USB_BASE + 0x00000048) ## !< Frame number register
  USB_DADDR* = (USB_BASE + 0x0000004C) ## !< Device address register
  USB_BTABLE* = (USB_BASE + 0x00000050) ## !< Buffer Table address register

## ******************  Bit definition for USB_CNTR register  ******************

const
  USB_CNTR_FRES_Pos* = (0)
  USB_CNTR_FRES_Msk* = (0x00000001 shl USB_CNTR_FRES_Pos) ## !< 0x00000001
  USB_CNTR_FRES* = USB_CNTR_FRES_Msk
  USB_CNTR_PDWN_Pos* = (1)
  USB_CNTR_PDWN_Msk* = (0x00000001 shl USB_CNTR_PDWN_Pos) ## !< 0x00000002
  USB_CNTR_PDWN* = USB_CNTR_PDWN_Msk
  USB_CNTR_LPMODE_Pos* = (2)
  USB_CNTR_LPMODE_Msk* = (0x00000001 shl USB_CNTR_LPMODE_Pos) ## !< 0x00000004
  USB_CNTR_LPMODE* = USB_CNTR_LPMODE_Msk
  USB_CNTR_FSUSP_Pos* = (3)
  USB_CNTR_FSUSP_Msk* = (0x00000001 shl USB_CNTR_FSUSP_Pos) ## !< 0x00000008
  USB_CNTR_FSUSP* = USB_CNTR_FSUSP_Msk
  USB_CNTR_RESUME_Pos* = (4)
  USB_CNTR_RESUME_Msk* = (0x00000001 shl USB_CNTR_RESUME_Pos) ## !< 0x00000010
  USB_CNTR_RESUME* = USB_CNTR_RESUME_Msk
  USB_CNTR_ESOFM_Pos* = (8)
  USB_CNTR_ESOFM_Msk* = (0x00000001 shl USB_CNTR_ESOFM_Pos) ## !< 0x00000100
  USB_CNTR_ESOFM* = USB_CNTR_ESOFM_Msk
  USB_CNTR_SOFM_Pos* = (9)
  USB_CNTR_SOFM_Msk* = (0x00000001 shl USB_CNTR_SOFM_Pos) ## !< 0x00000200
  USB_CNTR_SOFM* = USB_CNTR_SOFM_Msk
  USB_CNTR_RESETM_Pos* = (10)
  USB_CNTR_RESETM_Msk* = (0x00000001 shl USB_CNTR_RESETM_Pos) ## !< 0x00000400
  USB_CNTR_RESETM* = USB_CNTR_RESETM_Msk
  USB_CNTR_SUSPM_Pos* = (11)
  USB_CNTR_SUSPM_Msk* = (0x00000001 shl USB_CNTR_SUSPM_Pos) ## !< 0x00000800
  USB_CNTR_SUSPM* = USB_CNTR_SUSPM_Msk
  USB_CNTR_WKUPM_Pos* = (12)
  USB_CNTR_WKUPM_Msk* = (0x00000001 shl USB_CNTR_WKUPM_Pos) ## !< 0x00001000
  USB_CNTR_WKUPM* = USB_CNTR_WKUPM_Msk
  USB_CNTR_ERRM_Pos* = (13)
  USB_CNTR_ERRM_Msk* = (0x00000001 shl USB_CNTR_ERRM_Pos) ## !< 0x00002000
  USB_CNTR_ERRM* = USB_CNTR_ERRM_Msk
  USB_CNTR_PMAOVRM_Pos* = (14)
  USB_CNTR_PMAOVRM_Msk* = (0x00000001 shl USB_CNTR_PMAOVRM_Pos) ## !< 0x00004000
  USB_CNTR_PMAOVRM* = USB_CNTR_PMAOVRM_Msk
  USB_CNTR_CTRM_Pos* = (15)
  USB_CNTR_CTRM_Msk* = (0x00000001 shl USB_CNTR_CTRM_Pos) ## !< 0x00008000
  USB_CNTR_CTRM* = USB_CNTR_CTRM_Msk

## ******************  Bit definition for USB_ISTR register  ******************

const
  USB_ISTR_EP_ID_Pos* = (0)
  USB_ISTR_EP_ID_Msk* = (0x0000000F shl USB_ISTR_EP_ID_Pos) ## !< 0x0000000F
  USB_ISTR_EP_ID* = USB_ISTR_EP_ID_Msk
  USB_ISTR_DIR_Pos* = (4)
  USB_ISTR_DIR_Msk* = (0x00000001 shl USB_ISTR_DIR_Pos) ## !< 0x00000010
  USB_ISTR_DIR* = USB_ISTR_DIR_Msk
  USB_ISTR_ESOF_Pos* = (8)
  USB_ISTR_ESOF_Msk* = (0x00000001 shl USB_ISTR_ESOF_Pos) ## !< 0x00000100
  USB_ISTR_ESOF* = USB_ISTR_ESOF_Msk
  USB_ISTR_SOF_Pos* = (9)
  USB_ISTR_SOF_Msk* = (0x00000001 shl USB_ISTR_SOF_Pos) ## !< 0x00000200
  USB_ISTR_SOF* = USB_ISTR_SOF_Msk
  USB_ISTR_RESET_Pos* = (10)
  USB_ISTR_RESET_Msk* = (0x00000001 shl USB_ISTR_RESET_Pos) ## !< 0x00000400
  USB_ISTR_RESET* = USB_ISTR_RESET_Msk
  USB_ISTR_SUSP_Pos* = (11)
  USB_ISTR_SUSP_Msk* = (0x00000001 shl USB_ISTR_SUSP_Pos) ## !< 0x00000800
  USB_ISTR_SUSP* = USB_ISTR_SUSP_Msk
  USB_ISTR_WKUP_Pos* = (12)
  USB_ISTR_WKUP_Msk* = (0x00000001 shl USB_ISTR_WKUP_Pos) ## !< 0x00001000
  USB_ISTR_WKUP* = USB_ISTR_WKUP_Msk
  USB_ISTR_ERR_Pos* = (13)
  USB_ISTR_ERR_Msk* = (0x00000001 shl USB_ISTR_ERR_Pos) ## !< 0x00002000
  USB_ISTR_ERR* = USB_ISTR_ERR_Msk
  USB_ISTR_PMAOVR_Pos* = (14)
  USB_ISTR_PMAOVR_Msk* = (0x00000001 shl USB_ISTR_PMAOVR_Pos) ## !< 0x00004000
  USB_ISTR_PMAOVR* = USB_ISTR_PMAOVR_Msk
  USB_ISTR_CTR_Pos* = (15)
  USB_ISTR_CTR_Msk* = (0x00000001 shl USB_ISTR_CTR_Pos) ## !< 0x00008000
  USB_ISTR_CTR* = USB_ISTR_CTR_Msk
  USB_CLR_CTR* = (not USB_ISTR_CTR) ## !< clear Correct TRansfer bit
  USB_CLR_PMAOVRM* = (not USB_ISTR_PMAOVR) ## !< clear DMA OVeR/underrun bit
  USB_CLR_ERR* = (not USB_ISTR_ERR) ## !< clear ERRor bit
  USB_CLR_WKUP* = (not USB_ISTR_WKUP) ## !< clear WaKe UP bit
  USB_CLR_SUSP* = (not USB_ISTR_SUSP) ## !< clear SUSPend bit
  USB_CLR_RESET* = (not USB_ISTR_RESET) ## !< clear RESET bit
  USB_CLR_SOF* = (not USB_ISTR_SOF) ## !< clear Start Of Frame bit
  USB_CLR_ESOF* = (not USB_ISTR_ESOF) ## !< clear Expected Start Of Frame bit

## ******************  Bit definition for USB_FNR register  *******************

const
  USB_FNR_FN_Pos* = (0)
  USB_FNR_FN_Msk* = (0x000007FF shl USB_FNR_FN_Pos) ## !< 0x000007FF
  USB_FNR_FN* = USB_FNR_FN_Msk
  USB_FNR_LSOF_Pos* = (11)
  USB_FNR_LSOF_Msk* = (0x00000003 shl USB_FNR_LSOF_Pos) ## !< 0x00001800
  USB_FNR_LSOF* = USB_FNR_LSOF_Msk
  USB_FNR_LCK_Pos* = (13)
  USB_FNR_LCK_Msk* = (0x00000001 shl USB_FNR_LCK_Pos) ## !< 0x00002000
  USB_FNR_LCK* = USB_FNR_LCK_Msk
  USB_FNR_RXDM_Pos* = (14)
  USB_FNR_RXDM_Msk* = (0x00000001 shl USB_FNR_RXDM_Pos) ## !< 0x00004000
  USB_FNR_RXDM* = USB_FNR_RXDM_Msk
  USB_FNR_RXDP_Pos* = (15)
  USB_FNR_RXDP_Msk* = (0x00000001 shl USB_FNR_RXDP_Pos) ## !< 0x00008000
  USB_FNR_RXDP* = USB_FNR_RXDP_Msk

## *****************  Bit definition for USB_DADDR register  ******************

const
  USB_DADDR_ADD_Pos* = (0)
  USB_DADDR_ADD_Msk* = (0x0000007F shl USB_DADDR_ADD_Pos) ## !< 0x0000007F
  USB_DADDR_ADD* = USB_DADDR_ADD_Msk
  USB_DADDR_ADD0_Pos* = (0)
  USB_DADDR_ADD0_Msk* = (0x00000001 shl USB_DADDR_ADD0_Pos) ## !< 0x00000001
  USB_DADDR_ADD0* = USB_DADDR_ADD0_Msk
  USB_DADDR_ADD1_Pos* = (1)
  USB_DADDR_ADD1_Msk* = (0x00000001 shl USB_DADDR_ADD1_Pos) ## !< 0x00000002
  USB_DADDR_ADD1* = USB_DADDR_ADD1_Msk
  USB_DADDR_ADD2_Pos* = (2)
  USB_DADDR_ADD2_Msk* = (0x00000001 shl USB_DADDR_ADD2_Pos) ## !< 0x00000004
  USB_DADDR_ADD2* = USB_DADDR_ADD2_Msk
  USB_DADDR_ADD3_Pos* = (3)
  USB_DADDR_ADD3_Msk* = (0x00000001 shl USB_DADDR_ADD3_Pos) ## !< 0x00000008
  USB_DADDR_ADD3* = USB_DADDR_ADD3_Msk
  USB_DADDR_ADD4_Pos* = (4)
  USB_DADDR_ADD4_Msk* = (0x00000001 shl USB_DADDR_ADD4_Pos) ## !< 0x00000010
  USB_DADDR_ADD4* = USB_DADDR_ADD4_Msk
  USB_DADDR_ADD5_Pos* = (5)
  USB_DADDR_ADD5_Msk* = (0x00000001 shl USB_DADDR_ADD5_Pos) ## !< 0x00000020
  USB_DADDR_ADD5* = USB_DADDR_ADD5_Msk
  USB_DADDR_ADD6_Pos* = (6)
  USB_DADDR_ADD6_Msk* = (0x00000001 shl USB_DADDR_ADD6_Pos) ## !< 0x00000040
  USB_DADDR_ADD6* = USB_DADDR_ADD6_Msk
  USB_DADDR_EF_Pos* = (7)
  USB_DADDR_EF_Msk* = (0x00000001 shl USB_DADDR_EF_Pos) ## !< 0x00000080
  USB_DADDR_EF* = USB_DADDR_EF_Msk

## *****************  Bit definition for USB_BTABLE register  *****************

const
  USB_BTABLE_BTABLE_Pos* = (3)
  USB_BTABLE_BTABLE_Msk* = (0x00001FFF shl USB_BTABLE_BTABLE_Pos) ## !< 0x0000FFF8
  USB_BTABLE_BTABLE* = USB_BTABLE_BTABLE_Msk

## !< Buffer descriptor table
## ****************  Bit definition for USB_ADDR0_TX register  ****************

const
  USB_ADDR0_TX_ADDR0_TX_Pos* = (1)
  USB_ADDR0_TX_ADDR0_TX_Msk* = (0x00007FFF shl USB_ADDR0_TX_ADDR0_TX_Pos) ## !< 0x0000FFFE
  USB_ADDR0_TX_ADDR0_TX* = USB_ADDR0_TX_ADDR0_TX_Msk

## ****************  Bit definition for USB_ADDR1_TX register  ****************

const
  USB_ADDR1_TX_ADDR1_TX_Pos* = (1)
  USB_ADDR1_TX_ADDR1_TX_Msk* = (0x00007FFF shl USB_ADDR1_TX_ADDR1_TX_Pos) ## !< 0x0000FFFE
  USB_ADDR1_TX_ADDR1_TX* = USB_ADDR1_TX_ADDR1_TX_Msk

## ****************  Bit definition for USB_ADDR2_TX register  ****************

const
  USB_ADDR2_TX_ADDR2_TX_Pos* = (1)
  USB_ADDR2_TX_ADDR2_TX_Msk* = (0x00007FFF shl USB_ADDR2_TX_ADDR2_TX_Pos) ## !< 0x0000FFFE
  USB_ADDR2_TX_ADDR2_TX* = USB_ADDR2_TX_ADDR2_TX_Msk

## ****************  Bit definition for USB_ADDR3_TX register  ****************

const
  USB_ADDR3_TX_ADDR3_TX_Pos* = (1)
  USB_ADDR3_TX_ADDR3_TX_Msk* = (0x00007FFF shl USB_ADDR3_TX_ADDR3_TX_Pos) ## !< 0x0000FFFE
  USB_ADDR3_TX_ADDR3_TX* = USB_ADDR3_TX_ADDR3_TX_Msk

## ****************  Bit definition for USB_ADDR4_TX register  ****************

const
  USB_ADDR4_TX_ADDR4_TX_Pos* = (1)
  USB_ADDR4_TX_ADDR4_TX_Msk* = (0x00007FFF shl USB_ADDR4_TX_ADDR4_TX_Pos) ## !< 0x0000FFFE
  USB_ADDR4_TX_ADDR4_TX* = USB_ADDR4_TX_ADDR4_TX_Msk

## ****************  Bit definition for USB_ADDR5_TX register  ****************

const
  USB_ADDR5_TX_ADDR5_TX_Pos* = (1)
  USB_ADDR5_TX_ADDR5_TX_Msk* = (0x00007FFF shl USB_ADDR5_TX_ADDR5_TX_Pos) ## !< 0x0000FFFE
  USB_ADDR5_TX_ADDR5_TX* = USB_ADDR5_TX_ADDR5_TX_Msk

## ****************  Bit definition for USB_ADDR6_TX register  ****************

const
  USB_ADDR6_TX_ADDR6_TX_Pos* = (1)
  USB_ADDR6_TX_ADDR6_TX_Msk* = (0x00007FFF shl USB_ADDR6_TX_ADDR6_TX_Pos) ## !< 0x0000FFFE
  USB_ADDR6_TX_ADDR6_TX* = USB_ADDR6_TX_ADDR6_TX_Msk

## ****************  Bit definition for USB_ADDR7_TX register  ****************

const
  USB_ADDR7_TX_ADDR7_TX_Pos* = (1)
  USB_ADDR7_TX_ADDR7_TX_Msk* = (0x00007FFF shl USB_ADDR7_TX_ADDR7_TX_Pos) ## !< 0x0000FFFE
  USB_ADDR7_TX_ADDR7_TX* = USB_ADDR7_TX_ADDR7_TX_Msk

## ----------------------------------------------------------------------------
## ****************  Bit definition for USB_COUNT0_TX register  ***************

const
  USB_COUNT0_TX_COUNT0_TX_Pos* = (0)
  USB_COUNT0_TX_COUNT0_TX_Msk* = (0x000003FF shl USB_COUNT0_TX_COUNT0_TX_Pos) ## !< 0x000003FF
  USB_COUNT0_TX_COUNT0_TX* = USB_COUNT0_TX_COUNT0_TX_Msk

## ****************  Bit definition for USB_COUNT1_TX register  ***************

const
  USB_COUNT1_TX_COUNT1_TX_Pos* = (0)
  USB_COUNT1_TX_COUNT1_TX_Msk* = (0x000003FF shl USB_COUNT1_TX_COUNT1_TX_Pos) ## !< 0x000003FF
  USB_COUNT1_TX_COUNT1_TX* = USB_COUNT1_TX_COUNT1_TX_Msk

## ****************  Bit definition for USB_COUNT2_TX register  ***************

const
  USB_COUNT2_TX_COUNT2_TX_Pos* = (0)
  USB_COUNT2_TX_COUNT2_TX_Msk* = (0x000003FF shl USB_COUNT2_TX_COUNT2_TX_Pos) ## !< 0x000003FF
  USB_COUNT2_TX_COUNT2_TX* = USB_COUNT2_TX_COUNT2_TX_Msk

## ****************  Bit definition for USB_COUNT3_TX register  ***************

const
  USB_COUNT3_TX_COUNT3_TX_Pos* = (0)
  USB_COUNT3_TX_COUNT3_TX_Msk* = (0x000003FF shl USB_COUNT3_TX_COUNT3_TX_Pos) ## !< 0x000003FF
  USB_COUNT3_TX_COUNT3_TX* = USB_COUNT3_TX_COUNT3_TX_Msk

## ****************  Bit definition for USB_COUNT4_TX register  ***************

const
  USB_COUNT4_TX_COUNT4_TX_Pos* = (0)
  USB_COUNT4_TX_COUNT4_TX_Msk* = (0x000003FF shl USB_COUNT4_TX_COUNT4_TX_Pos) ## !< 0x000003FF
  USB_COUNT4_TX_COUNT4_TX* = USB_COUNT4_TX_COUNT4_TX_Msk

## ****************  Bit definition for USB_COUNT5_TX register  ***************

const
  USB_COUNT5_TX_COUNT5_TX_Pos* = (0)
  USB_COUNT5_TX_COUNT5_TX_Msk* = (0x000003FF shl USB_COUNT5_TX_COUNT5_TX_Pos) ## !< 0x000003FF
  USB_COUNT5_TX_COUNT5_TX* = USB_COUNT5_TX_COUNT5_TX_Msk

## ****************  Bit definition for USB_COUNT6_TX register  ***************

const
  USB_COUNT6_TX_COUNT6_TX_Pos* = (0)
  USB_COUNT6_TX_COUNT6_TX_Msk* = (0x000003FF shl USB_COUNT6_TX_COUNT6_TX_Pos) ## !< 0x000003FF
  USB_COUNT6_TX_COUNT6_TX* = USB_COUNT6_TX_COUNT6_TX_Msk

## ****************  Bit definition for USB_COUNT7_TX register  ***************

const
  USB_COUNT7_TX_COUNT7_TX_Pos* = (0)
  USB_COUNT7_TX_COUNT7_TX_Msk* = (0x000003FF shl USB_COUNT7_TX_COUNT7_TX_Pos) ## !< 0x000003FF
  USB_COUNT7_TX_COUNT7_TX* = USB_COUNT7_TX_COUNT7_TX_Msk

## ----------------------------------------------------------------------------
## ***************  Bit definition for USB_COUNT0_TX_0 register  **************

const
  USB_COUNT0_TX_0_COUNT0_TX_0* = (0x000003FF) ## !< Transmission Byte Count 0 (low)

## ***************  Bit definition for USB_COUNT0_TX_1 register  **************

const
  USB_COUNT0_TX_1_COUNT0_TX_1* = (0x03FF0000) ## !< Transmission Byte Count 0 (high)

## ***************  Bit definition for USB_COUNT1_TX_0 register  **************

const
  USB_COUNT1_TX_0_COUNT1_TX_0* = (0x000003FF) ## !< Transmission Byte Count 1 (low)

## ***************  Bit definition for USB_COUNT1_TX_1 register  **************

const
  USB_COUNT1_TX_1_COUNT1_TX_1* = (0x03FF0000) ## !< Transmission Byte Count 1 (high)

## ***************  Bit definition for USB_COUNT2_TX_0 register  **************

const
  USB_COUNT2_TX_0_COUNT2_TX_0* = (0x000003FF) ## !< Transmission Byte Count 2 (low)

## ***************  Bit definition for USB_COUNT2_TX_1 register  **************

const
  USB_COUNT2_TX_1_COUNT2_TX_1* = (0x03FF0000) ## !< Transmission Byte Count 2 (high)

## ***************  Bit definition for USB_COUNT3_TX_0 register  **************

const
  USB_COUNT3_TX_0_COUNT3_TX_0* = (0x00000000000003FF'i64).uint32 ## !< Transmission Byte Count 3 (low)

## ***************  Bit definition for USB_COUNT3_TX_1 register  **************

const
  USB_COUNT3_TX_1_COUNT3_TX_1* = (0x0000000003FF0000'i64).uint32 ## !< Transmission Byte Count 3 (high)

## ***************  Bit definition for USB_COUNT4_TX_0 register  **************

const
  USB_COUNT4_TX_0_COUNT4_TX_0* = (0x000003FF) ## !< Transmission Byte Count 4 (low)

## ***************  Bit definition for USB_COUNT4_TX_1 register  **************

const
  USB_COUNT4_TX_1_COUNT4_TX_1* = (0x03FF0000) ## !< Transmission Byte Count 4 (high)

## ***************  Bit definition for USB_COUNT5_TX_0 register  **************

const
  USB_COUNT5_TX_0_COUNT5_TX_0* = (0x000003FF) ## !< Transmission Byte Count 5 (low)

## ***************  Bit definition for USB_COUNT5_TX_1 register  **************

const
  USB_COUNT5_TX_1_COUNT5_TX_1* = (0x03FF0000) ## !< Transmission Byte Count 5 (high)

## ***************  Bit definition for USB_COUNT6_TX_0 register  **************

const
  USB_COUNT6_TX_0_COUNT6_TX_0* = (0x000003FF) ## !< Transmission Byte Count 6 (low)

## ***************  Bit definition for USB_COUNT6_TX_1 register  **************

const
  USB_COUNT6_TX_1_COUNT6_TX_1* = (0x03FF0000) ## !< Transmission Byte Count 6 (high)

## ***************  Bit definition for USB_COUNT7_TX_0 register  **************

const
  USB_COUNT7_TX_0_COUNT7_TX_0* = (0x000003FF) ## !< Transmission Byte Count 7 (low)

## ***************  Bit definition for USB_COUNT7_TX_1 register  **************

const
  USB_COUNT7_TX_1_COUNT7_TX_1* = (0x03FF0000) ## !< Transmission Byte Count 7 (high)

## ----------------------------------------------------------------------------
## ****************  Bit definition for USB_ADDR0_RX register  ****************

const
  USB_ADDR0_RX_ADDR0_RX_Pos* = (1)
  USB_ADDR0_RX_ADDR0_RX_Msk* = (0x00007FFF shl USB_ADDR0_RX_ADDR0_RX_Pos) ## !< 0x0000FFFE
  USB_ADDR0_RX_ADDR0_RX* = USB_ADDR0_RX_ADDR0_RX_Msk

## ****************  Bit definition for USB_ADDR1_RX register  ****************

const
  USB_ADDR1_RX_ADDR1_RX_Pos* = (1)
  USB_ADDR1_RX_ADDR1_RX_Msk* = (0x00007FFF shl USB_ADDR1_RX_ADDR1_RX_Pos) ## !< 0x0000FFFE
  USB_ADDR1_RX_ADDR1_RX* = USB_ADDR1_RX_ADDR1_RX_Msk

## ****************  Bit definition for USB_ADDR2_RX register  ****************

const
  USB_ADDR2_RX_ADDR2_RX_Pos* = (1)
  USB_ADDR2_RX_ADDR2_RX_Msk* = (0x00007FFF shl USB_ADDR2_RX_ADDR2_RX_Pos) ## !< 0x0000FFFE
  USB_ADDR2_RX_ADDR2_RX* = USB_ADDR2_RX_ADDR2_RX_Msk

## ****************  Bit definition for USB_ADDR3_RX register  ****************

const
  USB_ADDR3_RX_ADDR3_RX_Pos* = (1)
  USB_ADDR3_RX_ADDR3_RX_Msk* = (0x00007FFF shl USB_ADDR3_RX_ADDR3_RX_Pos) ## !< 0x0000FFFE
  USB_ADDR3_RX_ADDR3_RX* = USB_ADDR3_RX_ADDR3_RX_Msk

## ****************  Bit definition for USB_ADDR4_RX register  ****************

const
  USB_ADDR4_RX_ADDR4_RX_Pos* = (1)
  USB_ADDR4_RX_ADDR4_RX_Msk* = (0x00007FFF shl USB_ADDR4_RX_ADDR4_RX_Pos) ## !< 0x0000FFFE
  USB_ADDR4_RX_ADDR4_RX* = USB_ADDR4_RX_ADDR4_RX_Msk

## ****************  Bit definition for USB_ADDR5_RX register  ****************

const
  USB_ADDR5_RX_ADDR5_RX_Pos* = (1)
  USB_ADDR5_RX_ADDR5_RX_Msk* = (0x00007FFF shl USB_ADDR5_RX_ADDR5_RX_Pos) ## !< 0x0000FFFE
  USB_ADDR5_RX_ADDR5_RX* = USB_ADDR5_RX_ADDR5_RX_Msk

## ****************  Bit definition for USB_ADDR6_RX register  ****************

const
  USB_ADDR6_RX_ADDR6_RX_Pos* = (1)
  USB_ADDR6_RX_ADDR6_RX_Msk* = (0x00007FFF shl USB_ADDR6_RX_ADDR6_RX_Pos) ## !< 0x0000FFFE
  USB_ADDR6_RX_ADDR6_RX* = USB_ADDR6_RX_ADDR6_RX_Msk

## ****************  Bit definition for USB_ADDR7_RX register  ****************

const
  USB_ADDR7_RX_ADDR7_RX_Pos* = (1)
  USB_ADDR7_RX_ADDR7_RX_Msk* = (0x00007FFF shl USB_ADDR7_RX_ADDR7_RX_Pos) ## !< 0x0000FFFE
  USB_ADDR7_RX_ADDR7_RX* = USB_ADDR7_RX_ADDR7_RX_Msk

## ----------------------------------------------------------------------------
## ****************  Bit definition for USB_COUNT0_RX register  ***************

const
  USB_COUNT0_RX_COUNT0_RX_Pos* = (0)
  USB_COUNT0_RX_COUNT0_RX_Msk* = (0x000003FF shl USB_COUNT0_RX_COUNT0_RX_Pos) ## !< 0x000003FF
  USB_COUNT0_RX_COUNT0_RX* = USB_COUNT0_RX_COUNT0_RX_Msk
  USB_COUNT0_RX_NUM_BLOCK_Pos* = (10)
  USB_COUNT0_RX_NUM_BLOCK_Msk* = (0x0000001F shl USB_COUNT0_RX_NUM_BLOCK_Pos) ## !< 0x00007C00
  USB_COUNT0_RX_NUM_BLOCK* = USB_COUNT0_RX_NUM_BLOCK_Msk
  USB_COUNT0_RX_NUM_BLOCK_0* = (0x00000001 shl USB_COUNT0_RX_NUM_BLOCK_Pos) ## !< 0x00000400
  USB_COUNT0_RX_NUM_BLOCK_1* = (0x00000002 shl USB_COUNT0_RX_NUM_BLOCK_Pos) ## !< 0x00000800
  USB_COUNT0_RX_NUM_BLOCK_2* = (0x00000004 shl USB_COUNT0_RX_NUM_BLOCK_Pos) ## !< 0x00001000
  USB_COUNT0_RX_NUM_BLOCK_3* = (0x00000008 shl USB_COUNT0_RX_NUM_BLOCK_Pos) ## !< 0x00002000
  USB_COUNT0_RX_NUM_BLOCK_4* = (0x00000010 shl USB_COUNT0_RX_NUM_BLOCK_Pos) ## !< 0x00004000
  USB_COUNT0_RX_BLSIZE_Pos* = (15)
  USB_COUNT0_RX_BLSIZE_Msk* = (0x00000001 shl USB_COUNT0_RX_BLSIZE_Pos) ## !< 0x00008000
  USB_COUNT0_RX_BLSIZE* = USB_COUNT0_RX_BLSIZE_Msk

## ****************  Bit definition for USB_COUNT1_RX register  ***************

const
  USB_COUNT1_RX_COUNT1_RX_Pos* = (0)
  USB_COUNT1_RX_COUNT1_RX_Msk* = (0x000003FF shl USB_COUNT1_RX_COUNT1_RX_Pos) ## !< 0x000003FF
  USB_COUNT1_RX_COUNT1_RX* = USB_COUNT1_RX_COUNT1_RX_Msk
  USB_COUNT1_RX_NUM_BLOCK_Pos* = (10)
  USB_COUNT1_RX_NUM_BLOCK_Msk* = (0x0000001F shl USB_COUNT1_RX_NUM_BLOCK_Pos) ## !< 0x00007C00
  USB_COUNT1_RX_NUM_BLOCK* = USB_COUNT1_RX_NUM_BLOCK_Msk
  USB_COUNT1_RX_NUM_BLOCK_0* = (0x00000001 shl USB_COUNT1_RX_NUM_BLOCK_Pos) ## !< 0x00000400
  USB_COUNT1_RX_NUM_BLOCK_1* = (0x00000002 shl USB_COUNT1_RX_NUM_BLOCK_Pos) ## !< 0x00000800
  USB_COUNT1_RX_NUM_BLOCK_2* = (0x00000004 shl USB_COUNT1_RX_NUM_BLOCK_Pos) ## !< 0x00001000
  USB_COUNT1_RX_NUM_BLOCK_3* = (0x00000008 shl USB_COUNT1_RX_NUM_BLOCK_Pos) ## !< 0x00002000
  USB_COUNT1_RX_NUM_BLOCK_4* = (0x00000010 shl USB_COUNT1_RX_NUM_BLOCK_Pos) ## !< 0x00004000
  USB_COUNT1_RX_BLSIZE_Pos* = (15)
  USB_COUNT1_RX_BLSIZE_Msk* = (0x00000001 shl USB_COUNT1_RX_BLSIZE_Pos) ## !< 0x00008000
  USB_COUNT1_RX_BLSIZE* = USB_COUNT1_RX_BLSIZE_Msk

## ****************  Bit definition for USB_COUNT2_RX register  ***************

const
  USB_COUNT2_RX_COUNT2_RX_Pos* = (0)
  USB_COUNT2_RX_COUNT2_RX_Msk* = (0x000003FF shl USB_COUNT2_RX_COUNT2_RX_Pos) ## !< 0x000003FF
  USB_COUNT2_RX_COUNT2_RX* = USB_COUNT2_RX_COUNT2_RX_Msk
  USB_COUNT2_RX_NUM_BLOCK_Pos* = (10)
  USB_COUNT2_RX_NUM_BLOCK_Msk* = (0x0000001F shl USB_COUNT2_RX_NUM_BLOCK_Pos) ## !< 0x00007C00
  USB_COUNT2_RX_NUM_BLOCK* = USB_COUNT2_RX_NUM_BLOCK_Msk
  USB_COUNT2_RX_NUM_BLOCK_0* = (0x00000001 shl USB_COUNT2_RX_NUM_BLOCK_Pos) ## !< 0x00000400
  USB_COUNT2_RX_NUM_BLOCK_1* = (0x00000002 shl USB_COUNT2_RX_NUM_BLOCK_Pos) ## !< 0x00000800
  USB_COUNT2_RX_NUM_BLOCK_2* = (0x00000004 shl USB_COUNT2_RX_NUM_BLOCK_Pos) ## !< 0x00001000
  USB_COUNT2_RX_NUM_BLOCK_3* = (0x00000008 shl USB_COUNT2_RX_NUM_BLOCK_Pos) ## !< 0x00002000
  USB_COUNT2_RX_NUM_BLOCK_4* = (0x00000010 shl USB_COUNT2_RX_NUM_BLOCK_Pos) ## !< 0x00004000
  USB_COUNT2_RX_BLSIZE_Pos* = (15)
  USB_COUNT2_RX_BLSIZE_Msk* = (0x00000001 shl USB_COUNT2_RX_BLSIZE_Pos) ## !< 0x00008000
  USB_COUNT2_RX_BLSIZE* = USB_COUNT2_RX_BLSIZE_Msk

## ****************  Bit definition for USB_COUNT3_RX register  ***************

const
  USB_COUNT3_RX_COUNT3_RX_Pos* = (0)
  USB_COUNT3_RX_COUNT3_RX_Msk* = (0x000003FF shl USB_COUNT3_RX_COUNT3_RX_Pos) ## !< 0x000003FF
  USB_COUNT3_RX_COUNT3_RX* = USB_COUNT3_RX_COUNT3_RX_Msk
  USB_COUNT3_RX_NUM_BLOCK_Pos* = (10)
  USB_COUNT3_RX_NUM_BLOCK_Msk* = (0x0000001F shl USB_COUNT3_RX_NUM_BLOCK_Pos) ## !< 0x00007C00
  USB_COUNT3_RX_NUM_BLOCK* = USB_COUNT3_RX_NUM_BLOCK_Msk
  USB_COUNT3_RX_NUM_BLOCK_0* = (0x00000001 shl USB_COUNT3_RX_NUM_BLOCK_Pos) ## !< 0x00000400
  USB_COUNT3_RX_NUM_BLOCK_1* = (0x00000002 shl USB_COUNT3_RX_NUM_BLOCK_Pos) ## !< 0x00000800
  USB_COUNT3_RX_NUM_BLOCK_2* = (0x00000004 shl USB_COUNT3_RX_NUM_BLOCK_Pos) ## !< 0x00001000
  USB_COUNT3_RX_NUM_BLOCK_3* = (0x00000008 shl USB_COUNT3_RX_NUM_BLOCK_Pos) ## !< 0x00002000
  USB_COUNT3_RX_NUM_BLOCK_4* = (0x00000010 shl USB_COUNT3_RX_NUM_BLOCK_Pos) ## !< 0x00004000
  USB_COUNT3_RX_BLSIZE_Pos* = (15)
  USB_COUNT3_RX_BLSIZE_Msk* = (0x00000001 shl USB_COUNT3_RX_BLSIZE_Pos) ## !< 0x00008000
  USB_COUNT3_RX_BLSIZE* = USB_COUNT3_RX_BLSIZE_Msk

## ****************  Bit definition for USB_COUNT4_RX register  ***************

const
  USB_COUNT4_RX_COUNT4_RX_Pos* = (0)
  USB_COUNT4_RX_COUNT4_RX_Msk* = (0x000003FF shl USB_COUNT4_RX_COUNT4_RX_Pos) ## !< 0x000003FF
  USB_COUNT4_RX_COUNT4_RX* = USB_COUNT4_RX_COUNT4_RX_Msk
  USB_COUNT4_RX_NUM_BLOCK_Pos* = (10)
  USB_COUNT4_RX_NUM_BLOCK_Msk* = (0x0000001F shl USB_COUNT4_RX_NUM_BLOCK_Pos) ## !< 0x00007C00
  USB_COUNT4_RX_NUM_BLOCK* = USB_COUNT4_RX_NUM_BLOCK_Msk
  USB_COUNT4_RX_NUM_BLOCK_0* = (0x00000001 shl USB_COUNT4_RX_NUM_BLOCK_Pos) ## !< 0x00000400
  USB_COUNT4_RX_NUM_BLOCK_1* = (0x00000002 shl USB_COUNT4_RX_NUM_BLOCK_Pos) ## !< 0x00000800
  USB_COUNT4_RX_NUM_BLOCK_2* = (0x00000004 shl USB_COUNT4_RX_NUM_BLOCK_Pos) ## !< 0x00001000
  USB_COUNT4_RX_NUM_BLOCK_3* = (0x00000008 shl USB_COUNT4_RX_NUM_BLOCK_Pos) ## !< 0x00002000
  USB_COUNT4_RX_NUM_BLOCK_4* = (0x00000010 shl USB_COUNT4_RX_NUM_BLOCK_Pos) ## !< 0x00004000
  USB_COUNT4_RX_BLSIZE_Pos* = (15)
  USB_COUNT4_RX_BLSIZE_Msk* = (0x00000001 shl USB_COUNT4_RX_BLSIZE_Pos) ## !< 0x00008000
  USB_COUNT4_RX_BLSIZE* = USB_COUNT4_RX_BLSIZE_Msk

## ****************  Bit definition for USB_COUNT5_RX register  ***************

const
  USB_COUNT5_RX_COUNT5_RX_Pos* = (0)
  USB_COUNT5_RX_COUNT5_RX_Msk* = (0x000003FF shl USB_COUNT5_RX_COUNT5_RX_Pos) ## !< 0x000003FF
  USB_COUNT5_RX_COUNT5_RX* = USB_COUNT5_RX_COUNT5_RX_Msk
  USB_COUNT5_RX_NUM_BLOCK_Pos* = (10)
  USB_COUNT5_RX_NUM_BLOCK_Msk* = (0x0000001F shl USB_COUNT5_RX_NUM_BLOCK_Pos) ## !< 0x00007C00
  USB_COUNT5_RX_NUM_BLOCK* = USB_COUNT5_RX_NUM_BLOCK_Msk
  USB_COUNT5_RX_NUM_BLOCK_0* = (0x00000001 shl USB_COUNT5_RX_NUM_BLOCK_Pos) ## !< 0x00000400
  USB_COUNT5_RX_NUM_BLOCK_1* = (0x00000002 shl USB_COUNT5_RX_NUM_BLOCK_Pos) ## !< 0x00000800
  USB_COUNT5_RX_NUM_BLOCK_2* = (0x00000004 shl USB_COUNT5_RX_NUM_BLOCK_Pos) ## !< 0x00001000
  USB_COUNT5_RX_NUM_BLOCK_3* = (0x00000008 shl USB_COUNT5_RX_NUM_BLOCK_Pos) ## !< 0x00002000
  USB_COUNT5_RX_NUM_BLOCK_4* = (0x00000010 shl USB_COUNT5_RX_NUM_BLOCK_Pos) ## !< 0x00004000
  USB_COUNT5_RX_BLSIZE_Pos* = (15)
  USB_COUNT5_RX_BLSIZE_Msk* = (0x00000001 shl USB_COUNT5_RX_BLSIZE_Pos) ## !< 0x00008000
  USB_COUNT5_RX_BLSIZE* = USB_COUNT5_RX_BLSIZE_Msk

## ****************  Bit definition for USB_COUNT6_RX register  ***************

const
  USB_COUNT6_RX_COUNT6_RX_Pos* = (0)
  USB_COUNT6_RX_COUNT6_RX_Msk* = (0x000003FF shl USB_COUNT6_RX_COUNT6_RX_Pos) ## !< 0x000003FF
  USB_COUNT6_RX_COUNT6_RX* = USB_COUNT6_RX_COUNT6_RX_Msk
  USB_COUNT6_RX_NUM_BLOCK_Pos* = (10)
  USB_COUNT6_RX_NUM_BLOCK_Msk* = (0x0000001F shl USB_COUNT6_RX_NUM_BLOCK_Pos) ## !< 0x00007C00
  USB_COUNT6_RX_NUM_BLOCK* = USB_COUNT6_RX_NUM_BLOCK_Msk
  USB_COUNT6_RX_NUM_BLOCK_0* = (0x00000001 shl USB_COUNT6_RX_NUM_BLOCK_Pos) ## !< 0x00000400
  USB_COUNT6_RX_NUM_BLOCK_1* = (0x00000002 shl USB_COUNT6_RX_NUM_BLOCK_Pos) ## !< 0x00000800
  USB_COUNT6_RX_NUM_BLOCK_2* = (0x00000004 shl USB_COUNT6_RX_NUM_BLOCK_Pos) ## !< 0x00001000
  USB_COUNT6_RX_NUM_BLOCK_3* = (0x00000008 shl USB_COUNT6_RX_NUM_BLOCK_Pos) ## !< 0x00002000
  USB_COUNT6_RX_NUM_BLOCK_4* = (0x00000010 shl USB_COUNT6_RX_NUM_BLOCK_Pos) ## !< 0x00004000
  USB_COUNT6_RX_BLSIZE_Pos* = (15)
  USB_COUNT6_RX_BLSIZE_Msk* = (0x00000001 shl USB_COUNT6_RX_BLSIZE_Pos) ## !< 0x00008000
  USB_COUNT6_RX_BLSIZE* = USB_COUNT6_RX_BLSIZE_Msk

## ****************  Bit definition for USB_COUNT7_RX register  ***************

const
  USB_COUNT7_RX_COUNT7_RX_Pos* = (0)
  USB_COUNT7_RX_COUNT7_RX_Msk* = (0x000003FF shl USB_COUNT7_RX_COUNT7_RX_Pos) ## !< 0x000003FF
  USB_COUNT7_RX_COUNT7_RX* = USB_COUNT7_RX_COUNT7_RX_Msk
  USB_COUNT7_RX_NUM_BLOCK_Pos* = (10)
  USB_COUNT7_RX_NUM_BLOCK_Msk* = (0x0000001F shl USB_COUNT7_RX_NUM_BLOCK_Pos) ## !< 0x00007C00
  USB_COUNT7_RX_NUM_BLOCK* = USB_COUNT7_RX_NUM_BLOCK_Msk
  USB_COUNT7_RX_NUM_BLOCK_0* = (0x00000001 shl USB_COUNT7_RX_NUM_BLOCK_Pos) ## !< 0x00000400
  USB_COUNT7_RX_NUM_BLOCK_1* = (0x00000002 shl USB_COUNT7_RX_NUM_BLOCK_Pos) ## !< 0x00000800
  USB_COUNT7_RX_NUM_BLOCK_2* = (0x00000004 shl USB_COUNT7_RX_NUM_BLOCK_Pos) ## !< 0x00001000
  USB_COUNT7_RX_NUM_BLOCK_3* = (0x00000008 shl USB_COUNT7_RX_NUM_BLOCK_Pos) ## !< 0x00002000
  USB_COUNT7_RX_NUM_BLOCK_4* = (0x00000010 shl USB_COUNT7_RX_NUM_BLOCK_Pos) ## !< 0x00004000
  USB_COUNT7_RX_BLSIZE_Pos* = (15)
  USB_COUNT7_RX_BLSIZE_Msk* = (0x00000001 shl USB_COUNT7_RX_BLSIZE_Pos) ## !< 0x00008000
  USB_COUNT7_RX_BLSIZE* = USB_COUNT7_RX_BLSIZE_Msk

## ----------------------------------------------------------------------------
## ***************  Bit definition for USB_COUNT0_RX_0 register  **************

const
  USB_COUNT0_RX_0_COUNT0_RX_0* = (0x000003FF) ## !< Reception Byte Count (low)
  USB_COUNT0_RX_0_NUM_BLOCK_0* = (0x00007C00) ## !< NUM_BLOCK_0[4:0] bits (Number of blocks) (low)
  USB_COUNT0_RX_0_NUM_BLOCK_0_0* = (0x00000400) ## !< Bit 0
  USB_COUNT0_RX_0_NUM_BLOCK_0_1* = (0x00000800) ## !< Bit 1
  USB_COUNT0_RX_0_NUM_BLOCK_0_2* = (0x00001000) ## !< Bit 2
  USB_COUNT0_RX_0_NUM_BLOCK_0_3* = (0x00002000) ## !< Bit 3
  USB_COUNT0_RX_0_NUM_BLOCK_0_4* = (0x00004000) ## !< Bit 4
  USB_COUNT0_RX_0_BLSIZE_0* = (0x00008000) ## !< BLock SIZE (low)

## ***************  Bit definition for USB_COUNT0_RX_1 register  **************

const
  USB_COUNT0_RX_1_COUNT0_RX_1* = (0x03FF0000) ## !< Reception Byte Count (high)
  USB_COUNT0_RX_1_NUM_BLOCK_1* = (0x7C000000) ## !< NUM_BLOCK_1[4:0] bits (Number of blocks) (high)
  USB_COUNT0_RX_1_NUM_BLOCK_1_0* = (0x04000000) ## !< Bit 1
  USB_COUNT0_RX_1_NUM_BLOCK_1_1* = (0x08000000) ## !< Bit 1
  USB_COUNT0_RX_1_NUM_BLOCK_1_2* = (0x10000000) ## !< Bit 2
  USB_COUNT0_RX_1_NUM_BLOCK_1_3* = (0x20000000) ## !< Bit 3
  USB_COUNT0_RX_1_NUM_BLOCK_1_4* = (0x40000000) ## !< Bit 4
  USB_COUNT0_RX_1_BLSIZE_1* = (0x80000000) ## !< BLock SIZE (high)

## ***************  Bit definition for USB_COUNT1_RX_0 register  **************

const
  USB_COUNT1_RX_0_COUNT1_RX_0* = (0x000003FF) ## !< Reception Byte Count (low)
  USB_COUNT1_RX_0_NUM_BLOCK_0* = (0x00007C00) ## !< NUM_BLOCK_0[4:0] bits (Number of blocks) (low)
  USB_COUNT1_RX_0_NUM_BLOCK_0_0* = (0x00000400) ## !< Bit 0
  USB_COUNT1_RX_0_NUM_BLOCK_0_1* = (0x00000800) ## !< Bit 1
  USB_COUNT1_RX_0_NUM_BLOCK_0_2* = (0x00001000) ## !< Bit 2
  USB_COUNT1_RX_0_NUM_BLOCK_0_3* = (0x00002000) ## !< Bit 3
  USB_COUNT1_RX_0_NUM_BLOCK_0_4* = (0x00004000) ## !< Bit 4
  USB_COUNT1_RX_0_BLSIZE_0* = (0x00008000) ## !< BLock SIZE (low)

## ***************  Bit definition for USB_COUNT1_RX_1 register  **************

const
  USB_COUNT1_RX_1_COUNT1_RX_1* = (0x03FF0000) ## !< Reception Byte Count (high)
  USB_COUNT1_RX_1_NUM_BLOCK_1* = (0x7C000000) ## !< NUM_BLOCK_1[4:0] bits (Number of blocks) (high)
  USB_COUNT1_RX_1_NUM_BLOCK_1_0* = (0x04000000) ## !< Bit 0
  USB_COUNT1_RX_1_NUM_BLOCK_1_1* = (0x08000000) ## !< Bit 1
  USB_COUNT1_RX_1_NUM_BLOCK_1_2* = (0x10000000) ## !< Bit 2
  USB_COUNT1_RX_1_NUM_BLOCK_1_3* = (0x20000000) ## !< Bit 3
  USB_COUNT1_RX_1_NUM_BLOCK_1_4* = (0x40000000) ## !< Bit 4
  USB_COUNT1_RX_1_BLSIZE_1* = (0x80000000) ## !< BLock SIZE (high)

## ***************  Bit definition for USB_COUNT2_RX_0 register  **************

const
  USB_COUNT2_RX_0_COUNT2_RX_0* = (0x000003FF) ## !< Reception Byte Count (low)
  USB_COUNT2_RX_0_NUM_BLOCK_0* = (0x00007C00) ## !< NUM_BLOCK_0[4:0] bits (Number of blocks) (low)
  USB_COUNT2_RX_0_NUM_BLOCK_0_0* = (0x00000400) ## !< Bit 0
  USB_COUNT2_RX_0_NUM_BLOCK_0_1* = (0x00000800) ## !< Bit 1
  USB_COUNT2_RX_0_NUM_BLOCK_0_2* = (0x00001000) ## !< Bit 2
  USB_COUNT2_RX_0_NUM_BLOCK_0_3* = (0x00002000) ## !< Bit 3
  USB_COUNT2_RX_0_NUM_BLOCK_0_4* = (0x00004000) ## !< Bit 4
  USB_COUNT2_RX_0_BLSIZE_0* = (0x00008000) ## !< BLock SIZE (low)

## ***************  Bit definition for USB_COUNT2_RX_1 register  **************

const
  USB_COUNT2_RX_1_COUNT2_RX_1* = (0x03FF0000) ## !< Reception Byte Count (high)
  USB_COUNT2_RX_1_NUM_BLOCK_1* = (0x7C000000) ## !< NUM_BLOCK_1[4:0] bits (Number of blocks) (high)
  USB_COUNT2_RX_1_NUM_BLOCK_1_0* = (0x04000000) ## !< Bit 0
  USB_COUNT2_RX_1_NUM_BLOCK_1_1* = (0x08000000) ## !< Bit 1
  USB_COUNT2_RX_1_NUM_BLOCK_1_2* = (0x10000000) ## !< Bit 2
  USB_COUNT2_RX_1_NUM_BLOCK_1_3* = (0x20000000) ## !< Bit 3
  USB_COUNT2_RX_1_NUM_BLOCK_1_4* = (0x40000000) ## !< Bit 4
  USB_COUNT2_RX_1_BLSIZE_1* = (0x80000000) ## !< BLock SIZE (high)

## ***************  Bit definition for USB_COUNT3_RX_0 register  **************

const
  USB_COUNT3_RX_0_COUNT3_RX_0* = (0x000003FF) ## !< Reception Byte Count (low)
  USB_COUNT3_RX_0_NUM_BLOCK_0* = (0x00007C00) ## !< NUM_BLOCK_0[4:0] bits (Number of blocks) (low)
  USB_COUNT3_RX_0_NUM_BLOCK_0_0* = (0x00000400) ## !< Bit 0
  USB_COUNT3_RX_0_NUM_BLOCK_0_1* = (0x00000800) ## !< Bit 1
  USB_COUNT3_RX_0_NUM_BLOCK_0_2* = (0x00001000) ## !< Bit 2
  USB_COUNT3_RX_0_NUM_BLOCK_0_3* = (0x00002000) ## !< Bit 3
  USB_COUNT3_RX_0_NUM_BLOCK_0_4* = (0x00004000) ## !< Bit 4
  USB_COUNT3_RX_0_BLSIZE_0* = (0x00008000) ## !< BLock SIZE (low)

## ***************  Bit definition for USB_COUNT3_RX_1 register  **************

const
  USB_COUNT3_RX_1_COUNT3_RX_1* = (0x03FF0000) ## !< Reception Byte Count (high)
  USB_COUNT3_RX_1_NUM_BLOCK_1* = (0x7C000000) ## !< NUM_BLOCK_1[4:0] bits (Number of blocks) (high)
  USB_COUNT3_RX_1_NUM_BLOCK_1_0* = (0x04000000) ## !< Bit 0
  USB_COUNT3_RX_1_NUM_BLOCK_1_1* = (0x08000000) ## !< Bit 1
  USB_COUNT3_RX_1_NUM_BLOCK_1_2* = (0x10000000) ## !< Bit 2
  USB_COUNT3_RX_1_NUM_BLOCK_1_3* = (0x20000000) ## !< Bit 3
  USB_COUNT3_RX_1_NUM_BLOCK_1_4* = (0x40000000) ## !< Bit 4
  USB_COUNT3_RX_1_BLSIZE_1* = (0x80000000) ## !< BLock SIZE (high)

## ***************  Bit definition for USB_COUNT4_RX_0 register  **************

const
  USB_COUNT4_RX_0_COUNT4_RX_0* = (0x000003FF) ## !< Reception Byte Count (low)
  USB_COUNT4_RX_0_NUM_BLOCK_0* = (0x00007C00) ## !< NUM_BLOCK_0[4:0] bits (Number of blocks) (low)
  USB_COUNT4_RX_0_NUM_BLOCK_0_0* = (0x00000400) ## !< Bit 0
  USB_COUNT4_RX_0_NUM_BLOCK_0_1* = (0x00000800) ## !< Bit 1
  USB_COUNT4_RX_0_NUM_BLOCK_0_2* = (0x00001000) ## !< Bit 2
  USB_COUNT4_RX_0_NUM_BLOCK_0_3* = (0x00002000) ## !< Bit 3
  USB_COUNT4_RX_0_NUM_BLOCK_0_4* = (0x00004000) ## !< Bit 4
  USB_COUNT4_RX_0_BLSIZE_0* = (0x00008000) ## !< BLock SIZE (low)

## ***************  Bit definition for USB_COUNT4_RX_1 register  **************

const
  USB_COUNT4_RX_1_COUNT4_RX_1* = (0x03FF0000) ## !< Reception Byte Count (high)
  USB_COUNT4_RX_1_NUM_BLOCK_1* = (0x7C000000) ## !< NUM_BLOCK_1[4:0] bits (Number of blocks) (high)
  USB_COUNT4_RX_1_NUM_BLOCK_1_0* = (0x04000000) ## !< Bit 0
  USB_COUNT4_RX_1_NUM_BLOCK_1_1* = (0x08000000) ## !< Bit 1
  USB_COUNT4_RX_1_NUM_BLOCK_1_2* = (0x10000000) ## !< Bit 2
  USB_COUNT4_RX_1_NUM_BLOCK_1_3* = (0x20000000) ## !< Bit 3
  USB_COUNT4_RX_1_NUM_BLOCK_1_4* = (0x40000000) ## !< Bit 4
  USB_COUNT4_RX_1_BLSIZE_1* = (0x80000000) ## !< BLock SIZE (high)

## ***************  Bit definition for USB_COUNT5_RX_0 register  **************

const
  USB_COUNT5_RX_0_COUNT5_RX_0* = (0x000003FF) ## !< Reception Byte Count (low)
  USB_COUNT5_RX_0_NUM_BLOCK_0* = (0x00007C00) ## !< NUM_BLOCK_0[4:0] bits (Number of blocks) (low)
  USB_COUNT5_RX_0_NUM_BLOCK_0_0* = (0x00000400) ## !< Bit 0
  USB_COUNT5_RX_0_NUM_BLOCK_0_1* = (0x00000800) ## !< Bit 1
  USB_COUNT5_RX_0_NUM_BLOCK_0_2* = (0x00001000) ## !< Bit 2
  USB_COUNT5_RX_0_NUM_BLOCK_0_3* = (0x00002000) ## !< Bit 3
  USB_COUNT5_RX_0_NUM_BLOCK_0_4* = (0x00004000) ## !< Bit 4
  USB_COUNT5_RX_0_BLSIZE_0* = (0x00008000) ## !< BLock SIZE (low)

## ***************  Bit definition for USB_COUNT5_RX_1 register  **************

const
  USB_COUNT5_RX_1_COUNT5_RX_1* = (0x03FF0000) ## !< Reception Byte Count (high)
  USB_COUNT5_RX_1_NUM_BLOCK_1* = (0x7C000000) ## !< NUM_BLOCK_1[4:0] bits (Number of blocks) (high)
  USB_COUNT5_RX_1_NUM_BLOCK_1_0* = (0x04000000) ## !< Bit 0
  USB_COUNT5_RX_1_NUM_BLOCK_1_1* = (0x08000000) ## !< Bit 1
  USB_COUNT5_RX_1_NUM_BLOCK_1_2* = (0x10000000) ## !< Bit 2
  USB_COUNT5_RX_1_NUM_BLOCK_1_3* = (0x20000000) ## !< Bit 3
  USB_COUNT5_RX_1_NUM_BLOCK_1_4* = (0x40000000) ## !< Bit 4
  USB_COUNT5_RX_1_BLSIZE_1* = (0x80000000) ## !< BLock SIZE (high)

## **************  Bit definition for USB_COUNT6_RX_0  register  **************

const
  USB_COUNT6_RX_0_COUNT6_RX_0* = (0x000003FF) ## !< Reception Byte Count (low)
  USB_COUNT6_RX_0_NUM_BLOCK_0* = (0x00007C00) ## !< NUM_BLOCK_0[4:0] bits (Number of blocks) (low)
  USB_COUNT6_RX_0_NUM_BLOCK_0_0* = (0x00000400) ## !< Bit 0
  USB_COUNT6_RX_0_NUM_BLOCK_0_1* = (0x00000800) ## !< Bit 1
  USB_COUNT6_RX_0_NUM_BLOCK_0_2* = (0x00001000) ## !< Bit 2
  USB_COUNT6_RX_0_NUM_BLOCK_0_3* = (0x00002000) ## !< Bit 3
  USB_COUNT6_RX_0_NUM_BLOCK_0_4* = (0x00004000) ## !< Bit 4
  USB_COUNT6_RX_0_BLSIZE_0* = (0x00008000) ## !< BLock SIZE (low)

## ***************  Bit definition for USB_COUNT6_RX_1 register  **************

const
  USB_COUNT6_RX_1_COUNT6_RX_1* = (0x03FF0000) ## !< Reception Byte Count (high)
  USB_COUNT6_RX_1_NUM_BLOCK_1* = (0x7C000000) ## !< NUM_BLOCK_1[4:0] bits (Number of blocks) (high)
  USB_COUNT6_RX_1_NUM_BLOCK_1_0* = (0x04000000) ## !< Bit 0
  USB_COUNT6_RX_1_NUM_BLOCK_1_1* = (0x08000000) ## !< Bit 1
  USB_COUNT6_RX_1_NUM_BLOCK_1_2* = (0x10000000) ## !< Bit 2
  USB_COUNT6_RX_1_NUM_BLOCK_1_3* = (0x20000000) ## !< Bit 3
  USB_COUNT6_RX_1_NUM_BLOCK_1_4* = (0x40000000) ## !< Bit 4
  USB_COUNT6_RX_1_BLSIZE_1* = (0x80000000) ## !< BLock SIZE (high)

## **************  Bit definition for USB_COUNT7_RX_0 register  ***************

const
  USB_COUNT7_RX_0_COUNT7_RX_0* = (0x000003FF) ## !< Reception Byte Count (low)
  USB_COUNT7_RX_0_NUM_BLOCK_0* = (0x00007C00) ## !< NUM_BLOCK_0[4:0] bits (Number of blocks) (low)
  USB_COUNT7_RX_0_NUM_BLOCK_0_0* = (0x00000400) ## !< Bit 0
  USB_COUNT7_RX_0_NUM_BLOCK_0_1* = (0x00000800) ## !< Bit 1
  USB_COUNT7_RX_0_NUM_BLOCK_0_2* = (0x00001000) ## !< Bit 2
  USB_COUNT7_RX_0_NUM_BLOCK_0_3* = (0x00002000) ## !< Bit 3
  USB_COUNT7_RX_0_NUM_BLOCK_0_4* = (0x00004000) ## !< Bit 4
  USB_COUNT7_RX_0_BLSIZE_0* = (0x00008000) ## !< BLock SIZE (low)

## **************  Bit definition for USB_COUNT7_RX_1 register  ***************

const
  USB_COUNT7_RX_1_COUNT7_RX_1* = (0x03FF0000) ## !< Reception Byte Count (high)
  USB_COUNT7_RX_1_NUM_BLOCK_1* = (0x7C000000) ## !< NUM_BLOCK_1[4:0] bits (Number of blocks) (high)
  USB_COUNT7_RX_1_NUM_BLOCK_1_0* = (0x04000000) ## !< Bit 0
  USB_COUNT7_RX_1_NUM_BLOCK_1_1* = (0x08000000) ## !< Bit 1
  USB_COUNT7_RX_1_NUM_BLOCK_1_2* = (0x10000000) ## !< Bit 2
  USB_COUNT7_RX_1_NUM_BLOCK_1_3* = (0x20000000) ## !< Bit 3
  USB_COUNT7_RX_1_NUM_BLOCK_1_4* = (0x40000000) ## !< Bit 4
  USB_COUNT7_RX_1_BLSIZE_1* = (0x80000000) ## !< BLock SIZE (high)

## ****************************************************************************
##
##                          Window WATCHDOG (WWDG)
##
## ****************************************************************************
## ******************  Bit definition for WWDG_CR register  *******************

const
  WWDG_CR_T_Pos* = (0)
  WWDG_CR_T_Msk* = (0x0000007F shl WWDG_CR_T_Pos) ## !< 0x0000007F
  WWDG_CR_T* = WWDG_CR_T_Msk
  WWDG_CR_T_0* = (0x00000001 shl WWDG_CR_T_Pos) ## !< 0x00000001
  WWDG_CR_T_1* = (0x00000002 shl WWDG_CR_T_Pos) ## !< 0x00000002
  WWDG_CR_T_2* = (0x00000004 shl WWDG_CR_T_Pos) ## !< 0x00000004
  WWDG_CR_T_3* = (0x00000008 shl WWDG_CR_T_Pos) ## !< 0x00000008
  WWDG_CR_T_4* = (0x00000010 shl WWDG_CR_T_Pos) ## !< 0x00000010
  WWDG_CR_T_5* = (0x00000020 shl WWDG_CR_T_Pos) ## !< 0x00000020
  WWDG_CR_T_6* = (0x00000040 shl WWDG_CR_T_Pos) ## !< 0x00000040

##  Legacy defines

const
  WWDG_CR_T0x* = WWDG_CR_T_0
  WWDG_CR_T1x* = WWDG_CR_T_1
  WWDG_CR_T2x* = WWDG_CR_T_2
  WWDG_CR_T3x* = WWDG_CR_T_3
  WWDG_CR_T4x* = WWDG_CR_T_4
  WWDG_CR_T5x* = WWDG_CR_T_5
  WWDG_CR_T6x* = WWDG_CR_T_6
  WWDG_CR_WDGA_Pos* = (7)
  WWDG_CR_WDGA_Msk* = (0x00000001 shl WWDG_CR_WDGA_Pos) ## !< 0x00000080
  WWDG_CR_WDGA* = WWDG_CR_WDGA_Msk

## ******************  Bit definition for WWDG_CFR register  ******************

const
  WWDG_CFR_W_Pos* = (0)
  WWDG_CFR_W_Msk* = (0x0000007F shl WWDG_CFR_W_Pos) ## !< 0x0000007F
  WWDG_CFR_W* = WWDG_CFR_W_Msk
  WWDG_CFR_W_0* = (0x00000001 shl WWDG_CFR_W_Pos) ## !< 0x00000001
  WWDG_CFR_W_1* = (0x00000002 shl WWDG_CFR_W_Pos) ## !< 0x00000002
  WWDG_CFR_W_2* = (0x00000004 shl WWDG_CFR_W_Pos) ## !< 0x00000004
  WWDG_CFR_W_3* = (0x00000008 shl WWDG_CFR_W_Pos) ## !< 0x00000008
  WWDG_CFR_W_4* = (0x00000010 shl WWDG_CFR_W_Pos) ## !< 0x00000010
  WWDG_CFR_W_5* = (0x00000020 shl WWDG_CFR_W_Pos) ## !< 0x00000020
  WWDG_CFR_W_6* = (0x00000040 shl WWDG_CFR_W_Pos) ## !< 0x00000040

##  Legacy defines

const
  WWDG_CFR_W0x* = WWDG_CFR_W_0
  WWDG_CFR_W1x* = WWDG_CFR_W_1
  WWDG_CFR_W2x* = WWDG_CFR_W_2
  WWDG_CFR_W3x* = WWDG_CFR_W_3
  WWDG_CFR_W4x* = WWDG_CFR_W_4
  WWDG_CFR_W5x* = WWDG_CFR_W_5
  WWDG_CFR_W6x* = WWDG_CFR_W_6
  WWDG_CFR_WDGTB_Pos* = (7)
  WWDG_CFR_WDGTB_Msk* = (0x00000003 shl WWDG_CFR_WDGTB_Pos) ## !< 0x00000180
  WWDG_CFR_WDGTB* = WWDG_CFR_WDGTB_Msk
  WWDG_CFR_WDGTB_0* = (0x00000001 shl WWDG_CFR_WDGTB_Pos) ## !< 0x00000080
  WWDG_CFR_WDGTB_1* = (0x00000002 shl WWDG_CFR_WDGTB_Pos) ## !< 0x00000100

##  Legacy defines

const
  WWDG_CFR_WDGTB0x* = WWDG_CFR_WDGTB_0
  WWDG_CFR_WDGTB1x* = WWDG_CFR_WDGTB_1
  WWDG_CFR_EWI_Pos* = (9)
  WWDG_CFR_EWI_Msk* = (0x00000001 shl WWDG_CFR_EWI_Pos) ## !< 0x00000200
  WWDG_CFR_EWI* = WWDG_CFR_EWI_Msk

## ******************  Bit definition for WWDG_SR register  *******************

const
  WWDG_SR_EWIF_Pos* = (0)
  WWDG_SR_EWIF_Msk* = (0x00000001 shl WWDG_SR_EWIF_Pos) ## !< 0x00000001
  WWDG_SR_EWIF* = WWDG_SR_EWIF_Msk

## ****************************************************************************
##
##                         SystemTick (SysTick)
##
## ****************************************************************************
## ****************  Bit definition for SysTick_CTRL register  ****************

const
  SysTick_CTRL_ENABLE* = (0x00000001) ## !< Counter enable
  SysTick_CTRL_TICKINT* = (0x00000002) ## !< Counting down to 0 pends the SysTick handler
  SysTick_CTRL_CLKSOURCE* = (0x00000004) ## !< Clock source
  SysTick_CTRL_COUNTFLAG* = (0x00010000) ## !< Count Flag

## ****************  Bit definition for SysTick_LOAD register  ****************

const
  SysTick_LOAD_RELOAD* = (0x00FFFFFF) ## !< Value to load into the SysTick Current Value Register when the counter reaches 0

## ****************  Bit definition for SysTick_VAL register  *****************

const
  SysTick_VAL_CURRENT* = (0x00FFFFFF) ## !< Current value at the time the register is accessed

## ****************  Bit definition for SysTick_CALIB register  ***************

const
  SysTick_CALIB_TENMS* = (0x00FFFFFF) ## !< Reload value to use for 10ms timing
  SysTick_CALIB_SKEW* = (0x40000000) ## !< Calibration value is not exactly 10 ms
  SysTick_CALIB_NOREF* = (0x80000000) ## !< The reference clock is not provided

## ****************************************************************************
##
##                Nested Vectored Interrupt Controller (NVIC)
##
## ****************************************************************************
## *****************  Bit definition for NVIC_ISER register  ******************

const
  NVIC_ISER_SETENA_Pos* = (0)
  NVIC_ISER_SETENA_Msk* = (0xFFFFFFFF shl NVIC_ISER_SETENA_Pos) ## !< 0xFFFFFFFF
  NVIC_ISER_SETENA* = NVIC_ISER_SETENA_Msk
  NVIC_ISER_SETENA_0* = (0x00000001 shl NVIC_ISER_SETENA_Pos) ## !< 0x00000001
  NVIC_ISER_SETENA_1* = (0x00000002 shl NVIC_ISER_SETENA_Pos) ## !< 0x00000002
  NVIC_ISER_SETENA_2* = (0x00000004 shl NVIC_ISER_SETENA_Pos) ## !< 0x00000004
  NVIC_ISER_SETENA_3* = (0x00000008 shl NVIC_ISER_SETENA_Pos) ## !< 0x00000008
  NVIC_ISER_SETENA_4* = (0x00000010 shl NVIC_ISER_SETENA_Pos) ## !< 0x00000010
  NVIC_ISER_SETENA_5* = (0x00000020 shl NVIC_ISER_SETENA_Pos) ## !< 0x00000020
  NVIC_ISER_SETENA_6* = (0x00000040 shl NVIC_ISER_SETENA_Pos) ## !< 0x00000040
  NVIC_ISER_SETENA_7* = (0x00000080 shl NVIC_ISER_SETENA_Pos) ## !< 0x00000080
  NVIC_ISER_SETENA_8* = (0x00000100 shl NVIC_ISER_SETENA_Pos) ## !< 0x00000100
  NVIC_ISER_SETENA_9* = (0x00000200 shl NVIC_ISER_SETENA_Pos) ## !< 0x00000200
  NVIC_ISER_SETENA_10* = (0x00000400 shl NVIC_ISER_SETENA_Pos) ## !< 0x00000400
  NVIC_ISER_SETENA_11* = (0x00000800 shl NVIC_ISER_SETENA_Pos) ## !< 0x00000800
  NVIC_ISER_SETENA_12* = (0x00001000 shl NVIC_ISER_SETENA_Pos) ## !< 0x00001000
  NVIC_ISER_SETENA_13* = (0x00002000 shl NVIC_ISER_SETENA_Pos) ## !< 0x00002000
  NVIC_ISER_SETENA_14* = (0x00004000 shl NVIC_ISER_SETENA_Pos) ## !< 0x00004000
  NVIC_ISER_SETENA_15* = (0x00008000 shl NVIC_ISER_SETENA_Pos) ## !< 0x00008000
  NVIC_ISER_SETENA_16* = (0x00010000 shl NVIC_ISER_SETENA_Pos) ## !< 0x00010000
  NVIC_ISER_SETENA_17* = (0x00020000 shl NVIC_ISER_SETENA_Pos) ## !< 0x00020000
  NVIC_ISER_SETENA_18* = (0x00040000 shl NVIC_ISER_SETENA_Pos) ## !< 0x00040000
  NVIC_ISER_SETENA_19* = (0x00080000 shl NVIC_ISER_SETENA_Pos) ## !< 0x00080000
  NVIC_ISER_SETENA_20* = (0x00100000 shl NVIC_ISER_SETENA_Pos) ## !< 0x00100000
  NVIC_ISER_SETENA_21* = (0x00200000 shl NVIC_ISER_SETENA_Pos) ## !< 0x00200000
  NVIC_ISER_SETENA_22* = (0x00400000 shl NVIC_ISER_SETENA_Pos) ## !< 0x00400000
  NVIC_ISER_SETENA_23* = (0x00800000 shl NVIC_ISER_SETENA_Pos) ## !< 0x00800000
  NVIC_ISER_SETENA_24* = (0x01000000 shl NVIC_ISER_SETENA_Pos) ## !< 0x01000000
  NVIC_ISER_SETENA_25* = (0x02000000 shl NVIC_ISER_SETENA_Pos) ## !< 0x02000000
  NVIC_ISER_SETENA_26* = (0x04000000 shl NVIC_ISER_SETENA_Pos) ## !< 0x04000000
  NVIC_ISER_SETENA_27* = (0x08000000 shl NVIC_ISER_SETENA_Pos) ## !< 0x08000000
  NVIC_ISER_SETENA_28* = (0x10000000 shl NVIC_ISER_SETENA_Pos) ## !< 0x10000000
  NVIC_ISER_SETENA_29* = (0x20000000 shl NVIC_ISER_SETENA_Pos) ## !< 0x20000000
  NVIC_ISER_SETENA_30* = (0x40000000 shl NVIC_ISER_SETENA_Pos) ## !< 0x40000000
  NVIC_ISER_SETENA_31* = (0x80000000 shl NVIC_ISER_SETENA_Pos) ## !< 0x80000000

## *****************  Bit definition for NVIC_ICER register  ******************

const
  NVIC_ICER_CLRENA_Pos* = (0)
  NVIC_ICER_CLRENA_Msk* = (0xFFFFFFFF shl NVIC_ICER_CLRENA_Pos) ## !< 0xFFFFFFFF
  NVIC_ICER_CLRENA* = NVIC_ICER_CLRENA_Msk
  NVIC_ICER_CLRENA_0* = (0x00000001 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00000001
  NVIC_ICER_CLRENA_1* = (0x00000002 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00000002
  NVIC_ICER_CLRENA_2* = (0x00000004 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00000004
  NVIC_ICER_CLRENA_3* = (0x00000008 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00000008
  NVIC_ICER_CLRENA_4* = (0x00000010 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00000010
  NVIC_ICER_CLRENA_5* = (0x00000020 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00000020
  NVIC_ICER_CLRENA_6* = (0x00000040 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00000040
  NVIC_ICER_CLRENA_7* = (0x00000080 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00000080
  NVIC_ICER_CLRENA_8* = (0x00000100 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00000100
  NVIC_ICER_CLRENA_9* = (0x00000200 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00000200
  NVIC_ICER_CLRENA_10* = (0x00000400 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00000400
  NVIC_ICER_CLRENA_11* = (0x00000800 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00000800
  NVIC_ICER_CLRENA_12* = (0x00001000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00001000
  NVIC_ICER_CLRENA_13* = (0x00002000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00002000
  NVIC_ICER_CLRENA_14* = (0x00004000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00004000
  NVIC_ICER_CLRENA_15* = (0x00008000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00008000
  NVIC_ICER_CLRENA_16* = (0x00010000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00010000
  NVIC_ICER_CLRENA_17* = (0x00020000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00020000
  NVIC_ICER_CLRENA_18* = (0x00040000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00040000
  NVIC_ICER_CLRENA_19* = (0x00080000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00080000
  NVIC_ICER_CLRENA_20* = (0x00100000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00100000
  NVIC_ICER_CLRENA_21* = (0x00200000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00200000
  NVIC_ICER_CLRENA_22* = (0x00400000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00400000
  NVIC_ICER_CLRENA_23* = (0x00800000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x00800000
  NVIC_ICER_CLRENA_24* = (0x01000000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x01000000
  NVIC_ICER_CLRENA_25* = (0x02000000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x02000000
  NVIC_ICER_CLRENA_26* = (0x04000000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x04000000
  NVIC_ICER_CLRENA_27* = (0x08000000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x08000000
  NVIC_ICER_CLRENA_28* = (0x10000000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x10000000
  NVIC_ICER_CLRENA_29* = (0x20000000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x20000000
  NVIC_ICER_CLRENA_30* = (0x40000000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x40000000
  NVIC_ICER_CLRENA_31* = (0x80000000 shl NVIC_ICER_CLRENA_Pos) ## !< 0x80000000

## *****************  Bit definition for NVIC_ISPR register  ******************

const
  NVIC_ISPR_SETPEND_Pos* = (0)
  NVIC_ISPR_SETPEND_Msk* = (0xFFFFFFFF shl NVIC_ISPR_SETPEND_Pos) ## !< 0xFFFFFFFF
  NVIC_ISPR_SETPEND* = NVIC_ISPR_SETPEND_Msk
  NVIC_ISPR_SETPEND_0* = (0x00000001 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00000001
  NVIC_ISPR_SETPEND_1* = (0x00000002 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00000002
  NVIC_ISPR_SETPEND_2* = (0x00000004 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00000004
  NVIC_ISPR_SETPEND_3* = (0x00000008 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00000008
  NVIC_ISPR_SETPEND_4* = (0x00000010 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00000010
  NVIC_ISPR_SETPEND_5* = (0x00000020 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00000020
  NVIC_ISPR_SETPEND_6* = (0x00000040 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00000040
  NVIC_ISPR_SETPEND_7* = (0x00000080 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00000080
  NVIC_ISPR_SETPEND_8* = (0x00000100 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00000100
  NVIC_ISPR_SETPEND_9* = (0x00000200 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00000200
  NVIC_ISPR_SETPEND_10* = (0x00000400 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00000400
  NVIC_ISPR_SETPEND_11* = (0x00000800 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00000800
  NVIC_ISPR_SETPEND_12* = (0x00001000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00001000
  NVIC_ISPR_SETPEND_13* = (0x00002000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00002000
  NVIC_ISPR_SETPEND_14* = (0x00004000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00004000
  NVIC_ISPR_SETPEND_15* = (0x00008000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00008000
  NVIC_ISPR_SETPEND_16* = (0x00010000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00010000
  NVIC_ISPR_SETPEND_17* = (0x00020000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00020000
  NVIC_ISPR_SETPEND_18* = (0x00040000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00040000
  NVIC_ISPR_SETPEND_19* = (0x00080000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00080000
  NVIC_ISPR_SETPEND_20* = (0x00100000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00100000
  NVIC_ISPR_SETPEND_21* = (0x00200000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00200000
  NVIC_ISPR_SETPEND_22* = (0x00400000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00400000
  NVIC_ISPR_SETPEND_23* = (0x00800000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x00800000
  NVIC_ISPR_SETPEND_24* = (0x01000000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x01000000
  NVIC_ISPR_SETPEND_25* = (0x02000000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x02000000
  NVIC_ISPR_SETPEND_26* = (0x04000000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x04000000
  NVIC_ISPR_SETPEND_27* = (0x08000000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x08000000
  NVIC_ISPR_SETPEND_28* = (0x10000000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x10000000
  NVIC_ISPR_SETPEND_29* = (0x20000000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x20000000
  NVIC_ISPR_SETPEND_30* = (0x40000000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x40000000
  NVIC_ISPR_SETPEND_31* = (0x80000000 shl NVIC_ISPR_SETPEND_Pos) ## !< 0x80000000

## *****************  Bit definition for NVIC_ICPR register  ******************

const
  NVIC_ICPR_CLRPEND_Pos* = (0)
  NVIC_ICPR_CLRPEND_Msk* = (0xFFFFFFFF shl NVIC_ICPR_CLRPEND_Pos) ## !< 0xFFFFFFFF
  NVIC_ICPR_CLRPEND* = NVIC_ICPR_CLRPEND_Msk
  NVIC_ICPR_CLRPEND_0* = (0x00000001 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00000001
  NVIC_ICPR_CLRPEND_1* = (0x00000002 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00000002
  NVIC_ICPR_CLRPEND_2* = (0x00000004 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00000004
  NVIC_ICPR_CLRPEND_3* = (0x00000008 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00000008
  NVIC_ICPR_CLRPEND_4* = (0x00000010 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00000010
  NVIC_ICPR_CLRPEND_5* = (0x00000020 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00000020
  NVIC_ICPR_CLRPEND_6* = (0x00000040 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00000040
  NVIC_ICPR_CLRPEND_7* = (0x00000080 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00000080
  NVIC_ICPR_CLRPEND_8* = (0x00000100 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00000100
  NVIC_ICPR_CLRPEND_9* = (0x00000200 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00000200
  NVIC_ICPR_CLRPEND_10* = (0x00000400 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00000400
  NVIC_ICPR_CLRPEND_11* = (0x00000800 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00000800
  NVIC_ICPR_CLRPEND_12* = (0x00001000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00001000
  NVIC_ICPR_CLRPEND_13* = (0x00002000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00002000
  NVIC_ICPR_CLRPEND_14* = (0x00004000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00004000
  NVIC_ICPR_CLRPEND_15* = (0x00008000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00008000
  NVIC_ICPR_CLRPEND_16* = (0x00010000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00010000
  NVIC_ICPR_CLRPEND_17* = (0x00020000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00020000
  NVIC_ICPR_CLRPEND_18* = (0x00040000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00040000
  NVIC_ICPR_CLRPEND_19* = (0x00080000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00080000
  NVIC_ICPR_CLRPEND_20* = (0x00100000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00100000
  NVIC_ICPR_CLRPEND_21* = (0x00200000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00200000
  NVIC_ICPR_CLRPEND_22* = (0x00400000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00400000
  NVIC_ICPR_CLRPEND_23* = (0x00800000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x00800000
  NVIC_ICPR_CLRPEND_24* = (0x01000000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x01000000
  NVIC_ICPR_CLRPEND_25* = (0x02000000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x02000000
  NVIC_ICPR_CLRPEND_26* = (0x04000000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x04000000
  NVIC_ICPR_CLRPEND_27* = (0x08000000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x08000000
  NVIC_ICPR_CLRPEND_28* = (0x10000000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x10000000
  NVIC_ICPR_CLRPEND_29* = (0x20000000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x20000000
  NVIC_ICPR_CLRPEND_30* = (0x40000000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x40000000
  NVIC_ICPR_CLRPEND_31* = (0x80000000 shl NVIC_ICPR_CLRPEND_Pos) ## !< 0x80000000

## *****************  Bit definition for NVIC_IABR register  ******************

const
  NVIC_IABR_ACTIVE_Pos* = (0)
  NVIC_IABR_ACTIVE_Msk* = (0xFFFFFFFF shl NVIC_IABR_ACTIVE_Pos) ## !< 0xFFFFFFFF
  NVIC_IABR_ACTIVE* = NVIC_IABR_ACTIVE_Msk
  NVIC_IABR_ACTIVE_0* = (0x00000001 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00000001
  NVIC_IABR_ACTIVE_1* = (0x00000002 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00000002
  NVIC_IABR_ACTIVE_2* = (0x00000004 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00000004
  NVIC_IABR_ACTIVE_3* = (0x00000008 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00000008
  NVIC_IABR_ACTIVE_4* = (0x00000010 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00000010
  NVIC_IABR_ACTIVE_5* = (0x00000020 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00000020
  NVIC_IABR_ACTIVE_6* = (0x00000040 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00000040
  NVIC_IABR_ACTIVE_7* = (0x00000080 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00000080
  NVIC_IABR_ACTIVE_8* = (0x00000100 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00000100
  NVIC_IABR_ACTIVE_9* = (0x00000200 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00000200
  NVIC_IABR_ACTIVE_10* = (0x00000400 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00000400
  NVIC_IABR_ACTIVE_11* = (0x00000800 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00000800
  NVIC_IABR_ACTIVE_12* = (0x00001000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00001000
  NVIC_IABR_ACTIVE_13* = (0x00002000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00002000
  NVIC_IABR_ACTIVE_14* = (0x00004000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00004000
  NVIC_IABR_ACTIVE_15* = (0x00008000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00008000
  NVIC_IABR_ACTIVE_16* = (0x00010000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00010000
  NVIC_IABR_ACTIVE_17* = (0x00020000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00020000
  NVIC_IABR_ACTIVE_18* = (0x00040000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00040000
  NVIC_IABR_ACTIVE_19* = (0x00080000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00080000
  NVIC_IABR_ACTIVE_20* = (0x00100000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00100000
  NVIC_IABR_ACTIVE_21* = (0x00200000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00200000
  NVIC_IABR_ACTIVE_22* = (0x00400000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00400000
  NVIC_IABR_ACTIVE_23* = (0x00800000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x00800000
  NVIC_IABR_ACTIVE_24* = (0x01000000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x01000000
  NVIC_IABR_ACTIVE_25* = (0x02000000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x02000000
  NVIC_IABR_ACTIVE_26* = (0x04000000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x04000000
  NVIC_IABR_ACTIVE_27* = (0x08000000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x08000000
  NVIC_IABR_ACTIVE_28* = (0x10000000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x10000000
  NVIC_IABR_ACTIVE_29* = (0x20000000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x20000000
  NVIC_IABR_ACTIVE_30* = (0x40000000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x40000000
  NVIC_IABR_ACTIVE_31* = (0x80000000 shl NVIC_IABR_ACTIVE_Pos) ## !< 0x80000000

## *****************  Bit definition for NVIC_PRI0 register  ******************

const
  NVIC_IPR0_PRI_0* = (0x000000FF) ## !< Priority of interrupt 0
  NVIC_IPR0_PRI_1* = (0x0000FF00) ## !< Priority of interrupt 1
  NVIC_IPR0_PRI_2* = (0x00FF0000) ## !< Priority of interrupt 2
  NVIC_IPR0_PRI_3* = (0xFF000000) ## !< Priority of interrupt 3

## *****************  Bit definition for NVIC_PRI1 register  ******************

const
  NVIC_IPR1_PRI_4* = (0x000000FF) ## !< Priority of interrupt 4
  NVIC_IPR1_PRI_5* = (0x0000FF00) ## !< Priority of interrupt 5
  NVIC_IPR1_PRI_6* = (0x00FF0000) ## !< Priority of interrupt 6
  NVIC_IPR1_PRI_7* = (0xFF000000) ## !< Priority of interrupt 7

## *****************  Bit definition for NVIC_PRI2 register  ******************

const
  NVIC_IPR2_PRI_8* = (0x000000FF) ## !< Priority of interrupt 8
  NVIC_IPR2_PRI_9* = (0x0000FF00) ## !< Priority of interrupt 9
  NVIC_IPR2_PRI_10* = (0x00FF0000) ## !< Priority of interrupt 10
  NVIC_IPR2_PRI_11* = (0xFF000000) ## !< Priority of interrupt 11

## *****************  Bit definition for NVIC_PRI3 register  ******************

const
  NVIC_IPR3_PRI_12* = (0x000000FF) ## !< Priority of interrupt 12
  NVIC_IPR3_PRI_13* = (0x0000FF00) ## !< Priority of interrupt 13
  NVIC_IPR3_PRI_14* = (0x00FF0000) ## !< Priority of interrupt 14
  NVIC_IPR3_PRI_15* = (0xFF000000) ## !< Priority of interrupt 15

## *****************  Bit definition for NVIC_PRI4 register  ******************

const
  NVIC_IPR4_PRI_16* = (0x000000FF) ## !< Priority of interrupt 16
  NVIC_IPR4_PRI_17* = (0x0000FF00) ## !< Priority of interrupt 17
  NVIC_IPR4_PRI_18* = (0x00FF0000) ## !< Priority of interrupt 18
  NVIC_IPR4_PRI_19* = (0xFF000000) ## !< Priority of interrupt 19

## *****************  Bit definition for NVIC_PRI5 register  ******************

const
  NVIC_IPR5_PRI_20* = (0x000000FF) ## !< Priority of interrupt 20
  NVIC_IPR5_PRI_21* = (0x0000FF00) ## !< Priority of interrupt 21
  NVIC_IPR5_PRI_22* = (0x00FF0000) ## !< Priority of interrupt 22
  NVIC_IPR5_PRI_23* = (0xFF000000) ## !< Priority of interrupt 23

## *****************  Bit definition for NVIC_PRI6 register  ******************

const
  NVIC_IPR6_PRI_24* = (0x000000FF) ## !< Priority of interrupt 24
  NVIC_IPR6_PRI_25* = (0x0000FF00) ## !< Priority of interrupt 25
  NVIC_IPR6_PRI_26* = (0x00FF0000) ## !< Priority of interrupt 26
  NVIC_IPR6_PRI_27* = (0xFF000000) ## !< Priority of interrupt 27

## *****************  Bit definition for NVIC_PRI7 register  ******************

const
  NVIC_IPR7_PRI_28* = (0x000000FF) ## !< Priority of interrupt 28
  NVIC_IPR7_PRI_29* = (0x0000FF00) ## !< Priority of interrupt 29
  NVIC_IPR7_PRI_30* = (0x00FF0000) ## !< Priority of interrupt 30
  NVIC_IPR7_PRI_31* = (0xFF000000) ## !< Priority of interrupt 31

## *****************  Bit definition for SCB_CPUID register  ******************

const
  SCB_CPUID_REVISION* = (0x0000000F) ## !< Implementation defined revision number
  SCB_CPUID_PARTNO* = (0x0000FFF0) ## !< Number of processor within serie
  SCB_CPUID_Constant* = (0x000F0000) ## !< Reads as 0x0F
  SCB_CPUID_VARIANT* = (0x00F00000) ## !< Implementation defined variant number
  SCB_CPUID_IMPLEMENTER* = (0xFF000000) ## !< Implementer code. ARM is 0x41

## ******************  Bit definition for SCB_ICSR register  ******************

const
  SCB_ICSR_VECTACTIVE* = (0x000001FF) ## !< Active ISR number field
  SCB_ICSR_RETTOBASE* = (0x00000800) ## !< All active exceptions minus the IPSR_current_exception yields the empty set
  SCB_ICSR_VECTPENDING* = (0x003FF000) ## !< Pending ISR number field
  SCB_ICSR_ISRPENDING* = (0x00400000) ## !< Interrupt pending flag
  SCB_ICSR_ISRPREEMPT* = (0x00800000) ## !< It indicates that a pending interrupt becomes active in the next running cycle
  SCB_ICSR_PENDSTCLR* = (0x02000000) ## !< Clear pending SysTick bit
  SCB_ICSR_PENDSTSET* = (0x04000000) ## !< Set pending SysTick bit
  SCB_ICSR_PENDSVCLR* = (0x08000000) ## !< Clear pending pendSV bit
  SCB_ICSR_PENDSVSET* = (0x10000000) ## !< Set pending pendSV bit
  SCB_ICSR_NMIPENDSET* = (0x80000000) ## !< Set pending NMI bit

## ******************  Bit definition for SCB_VTOR register  ******************

const
  SCB_VTOR_TBLOFF* = (0x1FFFFF80) ## !< Vector table base offset field
  SCB_VTOR_TBLBASE* = (0x20000000) ## !< Table base in code(0) or RAM(1)

## !<*****************  Bit definition for SCB_AIRCR register  ******************

const
  SCB_AIRCR_VECTRESET* = (0x00000001) ## !< System Reset bit
  SCB_AIRCR_VECTCLRACTIVE* = (0x00000002) ## !< Clear active vector bit
  SCB_AIRCR_SYSRESETREQ* = (0x00000004) ## !< Requests chip control logic to generate a reset
  SCB_AIRCR_PRIGROUP* = (0x00000700) ## !< PRIGROUP[2:0] bits (Priority group)
  SCB_AIRCR_PRIGROUP_0* = (0x00000100) ## !< Bit 0
  SCB_AIRCR_PRIGROUP_1* = (0x00000200) ## !< Bit 1
  SCB_AIRCR_PRIGROUP_2* = (0x00000400) ## !< Bit 2

##  prority group configuration

const
  SCB_AIRCR_PRIGROUP0x* = (0x00000000) ## !< Priority group=0 (7 bits of pre-emption priority, 1 bit of subpriority)
  SCB_AIRCR_PRIGROUP1x* = (0x00000100) ## !< Priority group=1 (6 bits of pre-emption priority, 2 bits of subpriority)
  SCB_AIRCR_PRIGROUP2x* = (0x00000200) ## !< Priority group=2 (5 bits of pre-emption priority, 3 bits of subpriority)
  SCB_AIRCR_PRIGROUP3* = (0x00000300) ## !< Priority group=3 (4 bits of pre-emption priority, 4 bits of subpriority)
  SCB_AIRCR_PRIGROUP4* = (0x00000400) ## !< Priority group=4 (3 bits of pre-emption priority, 5 bits of subpriority)
  SCB_AIRCR_PRIGROUP5* = (0x00000500) ## !< Priority group=5 (2 bits of pre-emption priority, 6 bits of subpriority)
  SCB_AIRCR_PRIGROUP6* = (0x00000600) ## !< Priority group=6 (1 bit of pre-emption priority, 7 bits of subpriority)
  SCB_AIRCR_PRIGROUP7* = (0x00000700) ## !< Priority group=7 (no pre-emption priority, 8 bits of subpriority)
  SCB_AIRCR_ENDIANESS* = (0x00008000) ## !< Data endianness bit
  SCB_AIRCR_VECTKEY* = (0xFFFF0000) ## !< Register key (VECTKEY) - Reads as 0xFA05 (VECTKEYSTAT)

## ******************  Bit definition for SCB_SCR register  *******************

const
  SCB_SCR_SLEEPONEXIT* = (0x00000002) ## !< Sleep on exit bit
  SCB_SCR_SLEEPDEEP* = (0x00000004) ## !< Sleep deep bit
  SCB_SCR_SEVONPEND* = (0x00000010) ## !< Wake up from WFE

## *******************  Bit definition for SCB_CCR register  ******************

const
  SCB_CCR_NONBASETHRDENA* = (0x00000001) ## !< Thread mode can be entered from any level in Handler mode by controlled return value
  SCB_CCR_USERSETMPEND* = (0x00000002) ## !< Enables user code to write the Software Trigger Interrupt register to trigger (pend) a Main exception
  SCB_CCR_UNALIGN_TRP* = (0x00000008) ## !< Trap for unaligned access
  SCB_CCR_DIV_0_TRP* = (0x00000010) ## !< Trap on Divide by 0
  SCB_CCR_BFHFNMIGN* = (0x00000100) ## !< Handlers running at priority -1 and -2
  SCB_CCR_STKALIGN* = (0x00000200) ## !< On exception entry, the SP used prior to the exception is adjusted to be 8-byte aligned

## ******************  Bit definition for SCB_SHPR register *******************

const
  SCB_SHPR_PRI_N_Pos* = (0)
  SCB_SHPR_PRI_N_Msk* = (0x000000FF shl SCB_SHPR_PRI_N_Pos) ## !< 0x000000FF
  SCB_SHPR_PRI_N* = SCB_SHPR_PRI_N_Msk
  SCB_SHPR_PRI_N1_Pos* = (8)
  SCB_SHPR_PRI_N1_Msk* = (0x000000FF shl SCB_SHPR_PRI_N1_Pos) ## !< 0x0000FF00
  SCB_SHPR_PRI_N1* = SCB_SHPR_PRI_N1_Msk
  SCB_SHPR_PRI_N2_Pos* = (16)
  SCB_SHPR_PRI_N2_Msk* = (0x000000FF shl SCB_SHPR_PRI_N2_Pos) ## !< 0x00FF0000
  SCB_SHPR_PRI_N2* = SCB_SHPR_PRI_N2_Msk
  SCB_SHPR_PRI_N3_Pos* = (24)
  SCB_SHPR_PRI_N3_Msk* = (0x000000FF shl SCB_SHPR_PRI_N3_Pos) ## !< 0xFF000000
  SCB_SHPR_PRI_N3* = SCB_SHPR_PRI_N3_Msk

## *****************  Bit definition for SCB_SHCSR register  ******************

const
  SCB_SHCSR_MEMFAULTACT* = (0x00000001) ## !< MemManage is active
  SCB_SHCSR_BUSFAULTACT* = (0x00000002) ## !< BusFault is active
  SCB_SHCSR_USGFAULTACT* = (0x00000008) ## !< UsageFault is active
  SCB_SHCSR_SVCALLACT* = (0x00000080) ## !< SVCall is active
  SCB_SHCSR_MONITORACT* = (0x00000100) ## !< Monitor is active
  SCB_SHCSR_PENDSVACT* = (0x00000400) ## !< PendSV is active
  SCB_SHCSR_SYSTICKACT* = (0x00000800) ## !< SysTick is active
  SCB_SHCSR_USGFAULTPENDED* = (0x00001000) ## !< Usage Fault is pended
  SCB_SHCSR_MEMFAULTPENDED* = (0x00002000) ## !< MemManage is pended
  SCB_SHCSR_BUSFAULTPENDED* = (0x00004000) ## !< Bus Fault is pended
  SCB_SHCSR_SVCALLPENDED* = (0x00008000) ## !< SVCall is pended
  SCB_SHCSR_MEMFAULTENA* = (0x00010000) ## !< MemManage enable
  SCB_SHCSR_BUSFAULTENA* = (0x00020000) ## !< Bus Fault enable
  SCB_SHCSR_USGFAULTENA* = (0x00040000) ## !< UsageFault enable

## ******************  Bit definition for SCB_CFSR register  ******************
## !< MFSR

include "reg_utils"
include "core_cm3"
const
  SCB_CFSR_IACCVIOL_Pos* = (SCB_SHCSR_MEMFAULTACT_Pos + 0) ## !< SCB CFSR (MMFSR): IACCVIOL Position
  SCB_CFSR_IACCVIOL_Msk* = (1)  ## !< SCB CFSR (MMFSR): IACCVIOL Mask
  SCB_CFSR_IACCVIOL* = SCB_CFSR_IACCVIOL_Msk
  SCB_CFSR_DACCVIOL_Pos* = (SCB_SHCSR_MEMFAULTACT_Pos + 1) ## !< SCB CFSR (MMFSR): DACCVIOL Position
  SCB_CFSR_DACCVIOL_Msk* = (1 shl SCB_CFSR_DACCVIOL_Pos) ## !< SCB CFSR (MMFSR): DACCVIOL Mask
  SCB_CFSR_DACCVIOL* = SCB_CFSR_DACCVIOL_Msk
  SCB_CFSR_MUNSTKERR_Pos* = (SCB_SHCSR_MEMFAULTACT_Pos + 3) ## !< SCB CFSR (MMFSR): MUNSTKERR Position
  SCB_CFSR_MUNSTKERR_Msk* = (1 shl SCB_CFSR_MUNSTKERR_Pos) ## !< SCB CFSR (MMFSR): MUNSTKERR Mask
  SCB_CFSR_MUNSTKERR* = SCB_CFSR_MUNSTKERR_Msk
  SCB_CFSR_MSTKERR_Pos* = (SCB_SHCSR_MEMFAULTACT_Pos + 4) ## !< SCB CFSR (MMFSR): MSTKERR Position
  SCB_CFSR_MSTKERR_Msk* = (1 shl SCB_CFSR_MSTKERR_Pos) ## !< SCB CFSR (MMFSR): MSTKERR Mask
  SCB_CFSR_MSTKERR* = SCB_CFSR_MSTKERR_Msk
  SCB_CFSR_MMARVALID_Pos* = (SCB_SHCSR_MEMFAULTACT_Pos + 7) ## !< SCB CFSR (MMFSR): MMARVALID Position
  SCB_CFSR_MMARVALID_Msk* = (1 shl SCB_CFSR_MMARVALID_Pos) ## !< SCB CFSR (MMFSR): MMARVALID Mask
  SCB_CFSR_MMARVALID* = SCB_CFSR_MMARVALID_Msk

## !< BFSR

const
  SCB_CFSR_IBUSERR_Pos* = (SCB_CFSR_BUSFAULTSR_Pos + 0) ## !< SCB CFSR (BFSR): IBUSERR Position
  SCB_CFSR_IBUSERR_Msk* = (1 shl SCB_CFSR_IBUSERR_Pos) ## !< SCB CFSR (BFSR): IBUSERR Mask
  SCB_CFSR_IBUSERR* = SCB_CFSR_IBUSERR_Msk
  SCB_CFSR_PRECISERR_Pos* = (SCB_CFSR_BUSFAULTSR_Pos + 1) ## !< SCB CFSR (BFSR): PRECISERR Position
  SCB_CFSR_PRECISERR_Msk* = (1 shl SCB_CFSR_PRECISERR_Pos) ## !< SCB CFSR (BFSR): PRECISERR Mask
  SCB_CFSR_PRECISERR* = SCB_CFSR_PRECISERR_Msk
  SCB_CFSR_IMPRECISERR_Pos* = (SCB_CFSR_BUSFAULTSR_Pos + 2) ## !< SCB CFSR (BFSR): IMPRECISERR Position
  SCB_CFSR_IMPRECISERR_Msk* = (1 shl SCB_CFSR_IMPRECISERR_Pos) ## !< SCB CFSR (BFSR): IMPRECISERR Mask
  SCB_CFSR_IMPRECISERR* = SCB_CFSR_IMPRECISERR_Msk
  SCB_CFSR_UNSTKERR_Pos* = (SCB_CFSR_BUSFAULTSR_Pos + 3) ## !< SCB CFSR (BFSR): UNSTKERR Position
  SCB_CFSR_UNSTKERR_Msk* = (1 shl SCB_CFSR_UNSTKERR_Pos) ## !< SCB CFSR (BFSR): UNSTKERR Mask
  SCB_CFSR_UNSTKERR* = SCB_CFSR_UNSTKERR_Msk
  SCB_CFSR_STKERR_Pos* = (SCB_CFSR_BUSFAULTSR_Pos + 4) ## !< SCB CFSR (BFSR): STKERR Position
  SCB_CFSR_STKERR_Msk* = (1 shl SCB_CFSR_STKERR_Pos) ## !< SCB CFSR (BFSR): STKERR Mask
  SCB_CFSR_STKERR* = SCB_CFSR_STKERR_Msk
  SCB_CFSR_BFARVALID_Pos* = (SCB_CFSR_BUSFAULTSR_Pos + 7) ## !< SCB CFSR (BFSR): BFARVALID Position
  SCB_CFSR_BFARVALID_Msk* = (1 shl SCB_CFSR_BFARVALID_Pos) ## !< SCB CFSR (BFSR): BFARVALID Mask
  SCB_CFSR_BFARVALID* = SCB_CFSR_BFARVALID_Msk

## !< UFSR

const
  SCB_CFSR_UNDEFINSTR_Pos* = (SCB_CFSR_USGFAULTSR_Pos + 0) ## !< SCB CFSR (UFSR): UNDEFINSTR Position
  SCB_CFSR_UNDEFINSTR_Msk* = (1 shl SCB_CFSR_UNDEFINSTR_Pos) ## !< SCB CFSR (UFSR): UNDEFINSTR Mask
  SCB_CFSR_UNDEFINSTR* = SCB_CFSR_UNDEFINSTR_Msk
  SCB_CFSR_INVSTATE_Pos* = (SCB_CFSR_USGFAULTSR_Pos + 1) ## !< SCB CFSR (UFSR): INVSTATE Position
  SCB_CFSR_INVSTATE_Msk* = (1 shl SCB_CFSR_INVSTATE_Pos) ## !< SCB CFSR (UFSR): INVSTATE Mask
  SCB_CFSR_INVSTATE* = SCB_CFSR_INVSTATE_Msk
  SCB_CFSR_INVPC_Pos* = (SCB_CFSR_USGFAULTSR_Pos + 2) ## !< SCB CFSR (UFSR): INVPC Position
  SCB_CFSR_INVPC_Msk* = (1 shl SCB_CFSR_INVPC_Pos) ## !< SCB CFSR (UFSR): INVPC Mask
  SCB_CFSR_INVPC* = SCB_CFSR_INVPC_Msk
  SCB_CFSR_NOCP_Pos* = (SCB_CFSR_USGFAULTSR_Pos + 3) ## !< SCB CFSR (UFSR): NOCP Position
  SCB_CFSR_NOCP_Msk* = (1 shl SCB_CFSR_NOCP_Pos) ## !< SCB CFSR (UFSR): NOCP Mask
  SCB_CFSR_NOCP* = SCB_CFSR_NOCP_Msk
  SCB_CFSR_UNALIGNED_Pos* = (SCB_CFSR_USGFAULTSR_Pos + 8) ## !< SCB CFSR (UFSR): UNALIGNED Position
  SCB_CFSR_UNALIGNED_Msk* = (1 shl SCB_CFSR_UNALIGNED_Pos) ## !< SCB CFSR (UFSR): UNALIGNED Mask
  SCB_CFSR_UNALIGNED* = SCB_CFSR_UNALIGNED_Msk
  SCB_CFSR_DIVBYZERO_Pos* = (SCB_CFSR_USGFAULTSR_Pos + 9) ## !< SCB CFSR (UFSR): DIVBYZERO Position
  SCB_CFSR_DIVBYZERO_Msk* = (1 shl SCB_CFSR_DIVBYZERO_Pos) ## !< SCB CFSR (UFSR): DIVBYZERO Mask
  SCB_CFSR_DIVBYZERO* = SCB_CFSR_DIVBYZERO_Msk

## ******************  Bit definition for SCB_HFSR register  ******************

const
  SCB_HFSR_VECTTBL* = (0x00000002) ## !< Fault occures because of vector table read on exception processing
  SCB_HFSR_FORCED* = (0x40000000) ## !< Hard Fault activated when a configurable Fault was received and cannot activate
  SCB_HFSR_DEBUGEVT* = (0x80000000) ## !< Fault related to debug

## ******************  Bit definition for SCB_DFSR register  ******************

const
  SCB_DFSR_HALTED* = (0x00000001) ## !< Halt request flag
  SCB_DFSR_BKPT* = (0x00000002) ## !< BKPT flag
  SCB_DFSR_DWTTRAP* = (0x00000004) ## !< Data Watchpoint and Trace (DWT) flag
  SCB_DFSR_VCATCH* = (0x00000008) ## !< Vector catch flag
  SCB_DFSR_EXTERNAL* = (0x00000010) ## !< External debug request flag

## ******************  Bit definition for SCB_MMFAR register  *****************

const
  SCB_MMFAR_ADDRESS_Pos* = (0)
  SCB_MMFAR_ADDRESS_Msk* = (0xFFFFFFFF shl SCB_MMFAR_ADDRESS_Pos) ## !< 0xFFFFFFFF
  SCB_MMFAR_ADDRESS* = SCB_MMFAR_ADDRESS_Msk

## ******************  Bit definition for SCB_BFAR register  ******************

const
  SCB_BFAR_ADDRESS_Pos* = (0)
  SCB_BFAR_ADDRESS_Msk* = (0xFFFFFFFF shl SCB_BFAR_ADDRESS_Pos) ## !< 0xFFFFFFFF
  SCB_BFAR_ADDRESS* = SCB_BFAR_ADDRESS_Msk

## ******************  Bit definition for SCB_afsr register  ******************

const
  SCB_AFSR_IMPDEF_Pos* = (0)
  SCB_AFSR_IMPDEF_Msk* = (0xFFFFFFFF shl SCB_AFSR_IMPDEF_Pos) ## !< 0xFFFFFFFF
  SCB_AFSR_IMPDEF* = SCB_AFSR_IMPDEF_Msk

## *
##  @}
##
## *
##  @}
##
## * @addtogroup Exported_macro
##  @{
##
## ***************************** ADC Instances ********************************

template IS_ADC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == ADC1)

template IS_ADC_COMMON_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == ADC1_COMMON)

## ******************************* COMP Instances *****************************

template IS_COMP_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == COMP1) or ((INSTANCE) == COMP2))

template IS_COMP_COMMON_INSTANCE*(COMMON_INSTANCE: untyped): untyped =
  ((COMMON_INSTANCE) == COMP12_COMMON)

## ***************************** CRC Instances ********************************

template IS_CRC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == CRC)

## ***************************** DAC Instances ********************************

template IS_DAC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == DAC)

## ***************************** DMA Instances ********************************

template IS_DMA_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == DMA1_Channel1) or ((INSTANCE) == DMA1_Channel2) or
      ((INSTANCE) == DMA1_Channel3) or ((INSTANCE) == DMA1_Channel4) or
      ((INSTANCE) == DMA1_Channel5) or ((INSTANCE) == DMA1_Channel6) or
      ((INSTANCE) == DMA1_Channel7) or ((INSTANCE) == DMA2_Channel1) or
      ((INSTANCE) == DMA2_Channel2) or ((INSTANCE) == DMA2_Channel3) or
      ((INSTANCE) == DMA2_Channel4) or ((INSTANCE) == DMA2_Channel5))

## ****************************** GPIO Instances ******************************

template IS_GPIO_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == GPIOA) or ((INSTANCE) == GPIOB) or ((INSTANCE) == GPIOC) or
      ((INSTANCE) == GPIOD) or ((INSTANCE) == GPIOE) or ((INSTANCE) == GPIOF) or
      ((INSTANCE) == GPIOG) or ((INSTANCE) == GPIOH))

## *************************** GPIO Alternate Function Instances **************

template IS_GPIO_AF_INSTANCE*(INSTANCE: untyped): untyped =
  IS_GPIO_ALL_INSTANCE(INSTANCE)

## *************************** GPIO Lock Instances ****************************
##  On L1, all GPIO Bank support the Lock mechanism

template IS_GPIO_LOCK_INSTANCE*(INSTANCE: untyped): untyped =
  IS_GPIO_ALL_INSTANCE(INSTANCE)

## ******************************* I2C Instances ******************************

template IS_I2C_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == I2C1) or ((INSTANCE) == I2C2))

## ***************************** SMBUS Instances ******************************

template IS_SMBUS_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  IS_I2C_ALL_INSTANCE(INSTANCE)

## ******************************* I2S Instances ******************************

template IS_I2S_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == SPI2) or ((INSTANCE) == SPI3))

## ***************************** IWDG Instances *******************************

template IS_IWDG_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == IWDG)

## ***************************** OPAMP Instances ******************************

template IS_OPAMP_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == OPAMP1) or ((INSTANCE) == OPAMP2))

template IS_OPAMP_COMMON_INSTANCE*(COMMON_INSTANCE: untyped): untyped =
  ((COMMON_INSTANCE) == OPAMP12_COMMON)

## ***************************** RTC Instances ********************************

template IS_RTC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == RTC)

## ******************************* SPI Instances ******************************

template IS_SPI_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == SPI1) or ((INSTANCE) == SPI2) or ((INSTANCE) == SPI3))

## ***************************** TIM Instances ********************************

template IS_TIM_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5) or ((INSTANCE) == TIM6) or ((INSTANCE) == TIM7) or
      ((INSTANCE) == TIM9) or ((INSTANCE) == TIM10) or ((INSTANCE) == TIM11))

template IS_TIM_CC1_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9) or ((INSTANCE) == TIM10) or
      ((INSTANCE) == TIM11))

template IS_TIM_CC2_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9))

template IS_TIM_CC3_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5))

template IS_TIM_CC4_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5))

template IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9))

template IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9) or ((INSTANCE) == TIM10) or
      ((INSTANCE) == TIM11))

template IS_TIM_CLOCKSOURCE_TIX_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9))

template IS_TIM_CLOCKSOURCE_ITRX_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9))

template IS_TIM_OCXREF_CLEAR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4))

template IS_TIM_XOR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5))

template IS_TIM_ETR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5))

template IS_TIM_MASTER_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5) or ((INSTANCE) == TIM6) or ((INSTANCE) == TIM7) or
      ((INSTANCE) == TIM9))

template IS_TIM_SLAVE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9))

template IS_TIM_32B_COUNTER_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == TIM5)

template IS_TIM_DMABURST_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5))

template IS_TIM_CCX_INSTANCE*(INSTANCE, CHANNEL: untyped): untyped =
  ((((INSTANCE) == TIM2) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3) or ((CHANNEL) == TIM_CHANNEL_4))) or
      (((INSTANCE) == TIM3) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3) or ((CHANNEL) == TIM_CHANNEL_4))) or
      (((INSTANCE) == TIM4) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3) or ((CHANNEL) == TIM_CHANNEL_4))) or
      (((INSTANCE) == TIM5) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3) or ((CHANNEL) == TIM_CHANNEL_4))) or
      (((INSTANCE) == TIM9) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2))) or
      (((INSTANCE) == TIM10) and (((CHANNEL) == TIM_CHANNEL_1))) or
      (((INSTANCE) == TIM11) and (((CHANNEL) == TIM_CHANNEL_1))))

template IS_TIM_CLOCK_DIVISION_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9) or ((INSTANCE) == TIM10) or
      ((INSTANCE) == TIM11))

template IS_TIM_DMA_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5) or ((INSTANCE) == TIM6) or ((INSTANCE) == TIM7))

template IS_TIM_DMA_CC_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5))

template IS_TIM_COUNTER_MODE_SELECT_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9))

template IS_TIM_ENCODER_INTERFACE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM4) or
      ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9))

template IS_TIM_REMAP_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM9) or
      ((INSTANCE) == TIM10) or ((INSTANCE) == TIM11))

## ******************* USART Instances : Synchronous mode *********************

template IS_USART_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3))

## ******************* UART Instances : Asynchronous mode *********************

template IS_UART_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3) or
      ((INSTANCE) == UART4) or ((INSTANCE) == UART5))

## ******************* UART Instances : Half-Duplex mode *********************

template IS_UART_HALFDUPLEX_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3) or
      ((INSTANCE) == UART4) or ((INSTANCE) == UART5))

## ******************* UART Instances : LIN mode *********************

template IS_UART_LIN_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3) or
      ((INSTANCE) == UART4) or ((INSTANCE) == UART5))

## ***************** UART Instances : Hardware Flow control *******************

template IS_UART_HWFLOW_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3))

## ******************** UART Instances : Smard card mode **********************

template IS_SMARTCARD_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3))

## ********************** UART Instances : IRDA mode **************************

template IS_IRDA_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3) or
      ((INSTANCE) == UART4) or ((INSTANCE) == UART5))

## **************** UART Instances : Multi-Processor mode *********************

template IS_UART_MULTIPROCESSOR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3) or
      ((INSTANCE) == UART4) or ((INSTANCE) == UART5))

## ***************************** WWDG Instances *******************************

template IS_WWDG_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == WWDG)

## ***************************** LCD Instances *******************************

template IS_LCD_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == LCD)

## ***************************** USB Instances *******************************

template IS_USB_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == USB)

## *
##  @}
##
## ****************************************************************************
##   For a painless codes migration between the STM32L1xx device product
##   lines, the aliases defined below are put in place to overcome the
##   differences in the interrupt handlers and IRQn definitions.
##   No need to update developed interrupt code when moving across
##   product lines within the same STM32L1 Family
## ****************************************************************************
##  Aliases for __IRQn
##  Aliases for __IRQHandler
## *
##  @}
##
## *
##  @}
##

## *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE***
