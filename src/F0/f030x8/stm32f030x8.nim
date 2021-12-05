# *
# *****************************************************************************
#  @file    stm32f030x8.h
#  @author  MCD Application Team
#  @brief   CMSIS Cortex-M0 Device Peripheral Access Layer Header File.
#           This file contains all the peripheral register's definitions, bits
#           definitions and memory mapping for STM32F0xx devices.
##
#           This file contains:
#            - Data structures and the address mapping for all peripherals
#            - Peripheral's registers declarations and bits definition
#            - Macros to access peripheral’s registers hardware
##
# *****************************************************************************
#  @attention
##
#  <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
##
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#    1. Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#    2. Redistributions in binary form must reproduce the above copyright notice,
#       this list of conditions and the following disclaimer in the documentation
#       and/or other materials provided with the distribution.
#    3. Neither the name of STMicroelectronics nor the names of its contributors
#       may be used to endorse or promote products derived from this software
#       without specific prior written permission.
##
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
# *****************************************************************************
##
# * @addtogroup CMSIS
#  @{
##
# * @addtogroup stm32f030x8
#  @{
##

# * @addtogroup Configuration_section_for_CMSIS
#  @{
##
# *
#  @brief Configuration of the Cortex-M0 Processor and Core Peripherals
##

const
  CM0_REV* = 0
  MPU_PRESENT* = 0
  NVIC_PRIO_BITS* = 2
  Vendor_SysTickConfig* = 0

# *
#  @}
##
# * @addtogroup Peripheral_interrupt_number_definition
#  @{
##
# *
#  @brief STM32F0xx Interrupt Number Definition, according to the selected device
#         in @ref Library_configuration_section
##
# !< Interrupt Number Definition

type                          # *****  Cortex-M0 Processor Exceptions Numbers *************************************************************
  IRQn_Type* = enum
    NonMaskableInt_IRQn = -14,  # !< 2 Non Maskable Interrupt
    HardFault_IRQn = -13,       # !< 3 Cortex-M0 Hard Fault Interrupt
    SVC_IRQn = -5,              # !< 11 Cortex-M0 SV Call Interrupt
    PendSV_IRQn = -2,           # !< 14 Cortex-M0 Pend SV Interrupt
    SysTick_IRQn = -1,          # !< 15 Cortex-M0 System Tick Interrupt
                    # *****  STM32F0 specific Interrupt Numbers *****************************************************************
    WWDG_IRQn = 0,              # !< Window WatchDog Interrupt
    RTC_IRQn = 2,               # !< RTC Interrupt through EXTI Lines 17, 19 and 20
    FLASH_IRQn = 3,             # !< FLASH global Interrupt
    RCC_IRQn = 4,               # !< RCC global Interrupt
    EXTI0_1_IRQn = 5,           # !< EXTI Line 0 and 1 Interrupt
    EXTI2_3_IRQn = 6,           # !< EXTI Line 2 and 3 Interrupt
    EXTI4_15_IRQn = 7,          # !< EXTI Line 4 to 15 Interrupt
    DMA1_Channel1_IRQn = 9,     # !< DMA1 Channel 1 Interrupt
    DMA1_Channel2_3_IRQn = 10,  # !< DMA1 Channel 2 and Channel 3 Interrupt
    DMA1_Channel4_5_IRQn = 11,  # !< DMA1 Channel 4 and Channel 5 Interrupt
    ADC1_IRQn = 12,             # !< ADC1 Interrupt
    TIM1_BRK_UP_TRG_COM_IRQn = 13, # !< TIM1 Break, Update, Trigger and Commutation Interrupt
    TIM1_CC_IRQn = 14,          # !< TIM1 Capture Compare Interrupt
    TIM3_IRQn = 16,             # !< TIM3 global Interrupt
    TIM6_IRQn = 17,             # !< TIM6 global Interrupt
    TIM14_IRQn = 19,            # !< TIM14 global Interrupt
    TIM15_IRQn = 20,            # !< TIM15 global Interrupt
    TIM16_IRQn = 21,            # !< TIM16 global Interrupt
    TIM17_IRQn = 22,            # !< TIM17 global Interrupt
    I2C1_IRQn = 23,             # !< I2C1 Event Interrupt
    I2C2_IRQn = 24,             # !< I2C2 Event Interrupt
    SPI1_IRQn = 25,             # !< SPI1 global Interrupt
    SPI2_IRQn = 26,             # !< SPI2 global Interrupt
    USART1_IRQn = 27,           # !< USART1 global Interrupt
    USART2_IRQn = 28


# *
#  @}
##

#import core_cm0, system_stm32f0xx

#  STM32F0xx System Header
# * @addtogroup Peripheral_registers_structures
#  @{
##
# *
#  @brief Analog to Digital Converter
##

type
  ADC_TypeDef* {.bycopy.} = object
    ISR*: uint32             # !< ADC interrupt and status register,             Address offset: 0x00
    IER*: uint32             # !< ADC interrupt enable register,                 Address offset: 0x04
    CR*: uint32              # !< ADC control register,                          Address offset: 0x08
    CFGR1*: uint32           # !< ADC configuration register 1,                  Address offset: 0x0C
    CFGR2*: uint32           # !< ADC configuration register 2,                  Address offset: 0x10
    SMPR*: uint32            # !< ADC sampling time register,                    Address offset: 0x14
    RESERVED1*: uint32       # !< Reserved,                                                      0x18
    RESERVED2*: uint32       # !< Reserved,                                                      0x1C
    TR*: uint32              # !< ADC analog watchdog 1 threshold register,      Address offset: 0x20
    RESERVED3*: uint32       # !< Reserved,                                                      0x24
    CHSELR*: uint32          # !< ADC group regular sequencer register,          Address offset: 0x28
    RESERVED4*: array[5, uint32] # !< Reserved,                                                      0x2C
    DR*: uint32              # !< ADC group regular data register,               Address offset: 0x40

  ADC_Common_TypeDef* {.bycopy.} = object
    CCR*: uint32             # !< ADC common configuration register,             Address offset: ADC1 base address + 0x308


# *
#  @brief CRC calculation unit
##

type
  CRC_TypeDef* {.bycopy.} = object
    DR*: uint32              # !< CRC Data register,                           Address offset: 0x00
    IDR*: uint8              # !< CRC Independent data register,               Address offset: 0x04
    RESERVED0*: uint8        # !< Reserved,                                                    0x05
    RESERVED1*: uint16       # !< Reserved,                                                    0x06
    CR*: uint32              # !< CRC Control register,                        Address offset: 0x08
    RESERVED2*: uint32       # !< Reserved,                                                    0x0C
    INIT*: uint32            # !< Initial CRC value register,                  Address offset: 0x10
    RESERVED3*: uint32       # !< Reserved,                                                    0x14


# *
#  @brief Debug MCU
##

type
  DBGMCU_TypeDef* {.bycopy.} = object
    IDCODE*: uint32          # !< MCU device ID code,                          Address offset: 0x00
    CR*: uint32              # !< Debug MCU configuration register,            Address offset: 0x04
    APB1FZ*: uint32          # !< Debug MCU APB1 freeze register,              Address offset: 0x08
    APB2FZ*: uint32          # !< Debug MCU APB2 freeze register,              Address offset: 0x0C


# *
#  @brief DMA Controller
##

type
  DMA_Channel_TypeDef* {.bycopy.} = object
    CCR*: uint32             # !< DMA channel x configuration register
    CNDTR*: uint32           # !< DMA channel x number of data register
    CPAR*: uint32            # !< DMA channel x peripheral address register
    CMAR*: uint32            # !< DMA channel x memory address register

  DMA_TypeDef* {.bycopy.} = object
    ISR*: uint32             # !< DMA interrupt status register,               Address offset: 0x00
    IFCR*: uint32            # !< DMA interrupt flag clear register,           Address offset: 0x04


# *
#  @brief External Interrupt/Event Controller
##

type
  EXTI_TypeDef* {.bycopy.} = object
    IMR*: uint32             # !<EXTI Interrupt mask register,                 Address offset: 0x00
    EMR*: uint32             # !<EXTI Event mask register,                     Address offset: 0x04
    RTSR*: uint32            # !<EXTI Rising trigger selection register ,      Address offset: 0x08
    FTSR*: uint32            # !<EXTI Falling trigger selection register,      Address offset: 0x0C
    SWIER*: uint32           # !<EXTI Software interrupt event register,       Address offset: 0x10
    PR*: uint32              # !<EXTI Pending register,                        Address offset: 0x14


# *
#  @brief FLASH Registers
##

type
  FLASH_TypeDef* {.bycopy.} = object
    ACR*: uint32             # !<FLASH access control register,                 Address offset: 0x00
    KEYR*: uint32            # !<FLASH key register,                            Address offset: 0x04
    OPTKEYR*: uint32         # !<FLASH OPT key register,                        Address offset: 0x08
    SR*: uint32              # !<FLASH status register,                         Address offset: 0x0C
    CR*: uint32              # !<FLASH control register,                        Address offset: 0x10
    AR*: uint32              # !<FLASH address register,                        Address offset: 0x14
    RESERVED*: uint32        # !< Reserved,                                                     0x18
    OBR*: uint32             # !<FLASH option bytes register,                   Address offset: 0x1C
    WRPR*: uint32            # !<FLASH option bytes register,                   Address offset: 0x20


# *
#  @brief Option Bytes Registers
##

type
  OB_TypeDef* {.bycopy.} = object
    RDP*: uint16             # !< FLASH option byte Read protection,             Address offset: 0x00
    USER*: uint16            # !< FLASH option byte user options,                Address offset: 0x02
    DATA0*: uint16           # !< User data byte 0 (stored in FLASH_OBR[23:16]), Address offset: 0x04
    DATA1*: uint16           # !< User data byte 1 (stored in FLASH_OBR[31:24]), Address offset: 0x06
    WRP0*: uint16            # !< FLASH option byte write protection 0,          Address offset: 0x08
    WRP1*: uint16            # !< FLASH option byte write protection 1,          Address offset: 0x0A


# *
#  @brief General Purpose I/O
##

type
  GPIO_TypeDef* {.bycopy.} = object
    MODER*: uint32           # !< GPIO port mode register,                     Address offset: 0x00
    OTYPER*: uint32          # !< GPIO port output type register,              Address offset: 0x04
    OSPEEDR*: uint32         # !< GPIO port output speed register,             Address offset: 0x08
    PUPDR*: uint32           # !< GPIO port pull-up/pull-down register,        Address offset: 0x0C
    IDR*: uint32             # !< GPIO port input data register,               Address offset: 0x10
    ODR*: uint32             # !< GPIO port output data register,              Address offset: 0x14
    BSRR*: uint32            # !< GPIO port bit set/reset register,      Address offset: 0x1A
    LCKR*: uint32            # !< GPIO port configuration lock register,       Address offset: 0x1C
    AFR*: array[2, uint32]    # !< GPIO alternate function low register,  Address offset: 0x20-0x24
    BRR*: uint32             # !< GPIO bit reset register,                     Address offset: 0x28


# *
#  @brief SysTem Configuration
##

type
  SYSCFG_TypeDef* {.bycopy.} = object
    CFGR1*: uint32           # !< SYSCFG configuration register 1,                           Address offset: 0x00
    RESERVED*: uint32        # !< Reserved,                                                                  0x04
    EXTICR*: array[4, uint32] # !< SYSCFG external interrupt configuration register,     Address offset: 0x14-0x08
    CFGR2*: uint32           # !< SYSCFG configuration register 2,                           Address offset: 0x18


# *
#  @brief Inter-integrated Circuit Interface
##

type
  I2C_TypeDef* {.bycopy.} = object
    CR1*: uint32             # !< I2C Control register 1,                      Address offset: 0x00
    CR2*: uint32             # !< I2C Control register 2,                      Address offset: 0x04
    OAR1*: uint32            # !< I2C Own address 1 register,        Address offset: 0x08
    OAR2*: uint32            # !< I2C Own address 2 register,        Address offset: 0x0C
    TIMINGR*: uint32         # !< I2C Timing register,               Address offset: 0x10
    TIMEOUTR*: uint32        # !< I2C Timeout register,              Address offset: 0x14
    ISR*: uint32             # !< I2C Interrupt and status register, Address offset: 0x18
    ICR*: uint32             # !< I2C Interrupt clear register,      Address offset: 0x1C
    PECR*: uint32            # !< I2C PEC register,                  Address offset: 0x20
    RXDR*: uint32            # !< I2C Receive data register,         Address offset: 0x24
    TXDR*: uint32            # !< I2C Transmit data register,        Address offset: 0x28


# *
#  @brief Independent WATCHDOG
##

type
  IWDG_TypeDef* {.bycopy.} = object
    KR*: uint32              # !< IWDG Key register,       Address offset: 0x00
    PR*: uint32              # !< IWDG Prescaler register, Address offset: 0x04
    RLR*: uint32             # !< IWDG Reload register,    Address offset: 0x08
    SR*: uint32              # !< IWDG Status register,    Address offset: 0x0C
    WINR*: uint32            # !< IWDG Window register,    Address offset: 0x10


# *
#  @brief Power Control
##

type
  PWR_TypeDef* {.bycopy.} = object
    CR*: uint32              # !< PWR power control register,                          Address offset: 0x00
    CSR*: uint32             # !< PWR power control/status register,                   Address offset: 0x04


# *
#  @brief Reset and Clock Control
##

type
  RCC_TypeDef* {.bycopy.} = object
    CR*: uint32              # !< RCC clock control register,                                   Address offset: 0x00
    CFGR*: uint32            # !< RCC clock configuration register,                            Address offset: 0x04
    CIR*: uint32             # !< RCC clock interrupt register,                                Address offset: 0x08
    APB2RSTR*: uint32        # !< RCC APB2 peripheral reset register,                          Address offset: 0x0C
    APB1RSTR*: uint32        # !< RCC APB1 peripheral reset register,                          Address offset: 0x10
    AHBENR*: uint32          # !< RCC AHB peripheral clock register,                           Address offset: 0x14
    APB2ENR*: uint32         # !< RCC APB2 peripheral clock enable register,                   Address offset: 0x18
    APB1ENR*: uint32         # !< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C
    BDCR*: uint32            # !< RCC Backup domain control register,                          Address offset: 0x20
    CSR*: uint32             # !< RCC clock control & status register,                         Address offset: 0x24
    AHBRSTR*: uint32         # !< RCC AHB peripheral reset register,                           Address offset: 0x28
    CFGR2*: uint32           # !< RCC clock configuration register 2,                          Address offset: 0x2C
    CFGR3*: uint32           # !< RCC clock configuration register 3,                          Address offset: 0x30
    CR2*: uint32             # !< RCC clock control register 2,                                Address offset: 0x34


# *
#  @brief Real-Time Clock
##

type
  RTC_TypeDef* {.bycopy.} = object
    TR*: uint32              # !< RTC time register,                                         Address offset: 0x00
    DR*: uint32              # !< RTC date register,                                         Address offset: 0x04
    CR*: uint32              # !< RTC control register,                                      Address offset: 0x08
    ISR*: uint32             # !< RTC initialization and status register,                    Address offset: 0x0C
    PRER*: uint32            # !< RTC prescaler register,                                    Address offset: 0x10
    RESERVED1*: uint32       # !< Reserved,                                                  Address offset: 0x14
    RESERVED2*: uint32       # !< Reserved,                                                  Address offset: 0x18
    ALRMAR*: uint32          # !< RTC alarm A register,                                      Address offset: 0x1C
    RESERVED3*: uint32       # !< Reserved,                                                  Address offset: 0x20
    WPR*: uint32             # !< RTC write protection register,                             Address offset: 0x24
    SSR*: uint32             # !< RTC sub second register,                                   Address offset: 0x28
    SHIFTR*: uint32          # !< RTC shift control register,                                Address offset: 0x2C
    TSTR*: uint32            # !< RTC time stamp time register,                              Address offset: 0x30
    TSDR*: uint32            # !< RTC time stamp date register,                              Address offset: 0x34
    TSSSR*: uint32           # !< RTC time-stamp sub second register,                        Address offset: 0x38
    CALR*: uint32            # !< RTC calibration register,                                  Address offset: 0x3C
    TAFCR*: uint32           # !< RTC tamper and alternate function configuration register,  Address offset: 0x40
    ALRMASSR*: uint32        # !< RTC alarm A sub second register,                           Address offset: 0x44


# *
#  @brief Serial Peripheral Interface
##

type
  SPI_TypeDef* {.bycopy.} = object
    CR1*: uint32             # !< SPI Control register 1 (not used in I2S mode),      Address offset: 0x00
    CR2*: uint32             # !< SPI Control register 2,                             Address offset: 0x04
    SR*: uint32              # !< SPI Status register,                                Address offset: 0x08
    DR*: uint32              # !< SPI data register,                                  Address offset: 0x0C
    CRCPR*: uint32           # !< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10
    RXCRCR*: uint32          # !< SPI Rx CRC register (not used in I2S mode),         Address offset: 0x14
    TXCRCR*: uint32          # !< SPI Tx CRC register (not used in I2S mode),         Address offset: 0x18
    I2SCFGR*: uint32         # !< SPI_I2S configuration register,                     Address offset: 0x1C


# *
#  @brief TIM
##

type
  TIM_TypeDef* {.bycopy.} = object
    CR1*: uint32             # !< TIM control register 1,              Address offset: 0x00
    CR2*: uint32             # !< TIM control register 2,              Address offset: 0x04
    SMCR*: uint32            # !< TIM slave Mode Control register,     Address offset: 0x08
    DIER*: uint32            # !< TIM DMA/interrupt enable register,   Address offset: 0x0C
    SR*: uint32              # !< TIM status register,                 Address offset: 0x10
    EGR*: uint32             # !< TIM event generation register,       Address offset: 0x14
    CCMR1*: uint32           # !< TIM capture/compare mode register 1, Address offset: 0x18
    CCMR2*: uint32           # !< TIM capture/compare mode register 2, Address offset: 0x1C
    CCER*: uint32            # !< TIM capture/compare enable register, Address offset: 0x20
    CNT*: uint32             # !< TIM counter register,                Address offset: 0x24
    PSC*: uint32             # !< TIM prescaler register,              Address offset: 0x28
    ARR*: uint32             # !< TIM auto-reload register,            Address offset: 0x2C
    RCR*: uint32             # !< TIM  repetition counter register,            Address offset: 0x30
    CCR1*: uint32            # !< TIM capture/compare register 1,      Address offset: 0x34
    CCR2*: uint32            # !< TIM capture/compare register 2,      Address offset: 0x38
    CCR3*: uint32            # !< TIM capture/compare register 3,      Address offset: 0x3C
    CCR4*: uint32            # !< TIM capture/compare register 4,      Address offset: 0x40
    BDTR*: uint32            # !< TIM break and dead-time register,            Address offset: 0x44
    DCR*: uint32             # !< TIM DMA control register,            Address offset: 0x48
    DMAR*: uint32            # !< TIM DMA address for full transfer register,  Address offset: 0x4C
    OR*: uint32              # !< TIM option register,                 Address offset: 0x50


# *
#  @brief Universal Synchronous Asynchronous Receiver Transmitter
##

type
  USART_TypeDef* {.bycopy.} = object
    CR1*: uint32             # !< USART Control register 1,                 Address offset: 0x00
    CR2*: uint32             # !< USART Control register 2,                 Address offset: 0x04
    CR3*: uint32             # !< USART Control register 3,                 Address offset: 0x08
    BRR*: uint32             # !< USART Baud rate register,                 Address offset: 0x0C
    GTPR*: uint32            # !< USART Guard time and prescaler register,  Address offset: 0x10
    RTOR*: uint32            # !< USART Receiver Time Out register,         Address offset: 0x14
    RQR*: uint32             # !< USART Request register,                   Address offset: 0x18
    ISR*: uint32             # !< USART Interrupt and status register,      Address offset: 0x1C
    ICR*: uint32             # !< USART Interrupt flag Clear register,      Address offset: 0x20
    RDR*: uint16             # !< USART Receive Data register,              Address offset: 0x24
    RESERVED1*: uint16       # !< Reserved, 0x26
    TDR*: uint16             # !< USART Transmit Data register,             Address offset: 0x28
    RESERVED2*: uint16       # !< Reserved, 0x2A


# *
#  @brief Window WATCHDOG
##

type
  WWDG_TypeDef* {.bycopy.} = object
    CR*: uint32              # !< WWDG Control register,       Address offset: 0x00
    CFR*: uint32             # !< WWDG Configuration register, Address offset: 0x04
    SR*: uint32              # !< WWDG Status register,        Address offset: 0x08


# *
#  @}
##
# * @addtogroup Peripheral_memory_map
#  @{
##

const
  FLASH_BASE* = (cast[uint32](0x08000000)) # !< FLASH base address in the alias region
  FLASH_BANK1_END* = (cast[uint32](0x0800FFFF)) # !< FLASH END address of bank1
  SRAM_BASE* = (cast[uint32](0x20000000)) # !< SRAM base address in the alias region
  PERIPH_BASE* = (cast[uint32](0x40000000)) # !< Peripheral base address in the alias region

# !< Peripheral memory map

const
  APBPERIPH_BASE* = PERIPH_BASE
  AHBPERIPH_BASE* = (PERIPH_BASE + 0x00020000)
  AHB2PERIPH_BASE* = (PERIPH_BASE + 0x08000000)

# !< APB peripherals

const
  TIM3_BASE* = (APBPERIPH_BASE + 0x00000400)
  TIM6_BASE* = (APBPERIPH_BASE + 0x00001000)
  TIM14_BASE* = (APBPERIPH_BASE + 0x00002000)
  RTC_BASE* = (APBPERIPH_BASE + 0x00002800)
  WWDG_BASE* = (APBPERIPH_BASE + 0x00002C00)
  IWDG_BASE* = (APBPERIPH_BASE + 0x00003000)
  SPI2_BASE* = (APBPERIPH_BASE + 0x00003800)
  USART2_BASE* = (APBPERIPH_BASE + 0x00004400)
  I2C1_BASE* = (APBPERIPH_BASE + 0x00005400)
  I2C2_BASE* = (APBPERIPH_BASE + 0x00005800)
  PWR_BASE* = (APBPERIPH_BASE + 0x00007000)
  SYSCFG_BASE* = (APBPERIPH_BASE + 0x00010000)
  EXTI_BASE* = (APBPERIPH_BASE + 0x00010400)
  ADC1_BASE* = (APBPERIPH_BASE + 0x00012400)
  ADC_BASE* = (APBPERIPH_BASE + 0x00012708)
  TIM1_BASE* = (APBPERIPH_BASE + 0x00012C00)
  SPI1_BASE* = (APBPERIPH_BASE + 0x00013000)
  USART1_BASE* = (APBPERIPH_BASE + 0x00013800)
  TIM15_BASE* = (APBPERIPH_BASE + 0x00014000)
  TIM16_BASE* = (APBPERIPH_BASE + 0x00014400)
  TIM17_BASE* = (APBPERIPH_BASE + 0x00014800)
  DBGMCU_BASE* = (APBPERIPH_BASE + 0x00015800)

# !< AHB peripherals

const
  DMA1_BASE* = (AHBPERIPH_BASE + 0x00000000)
  DMA1_Channel1_BASE* = (DMA1_BASE + 0x00000008)
  DMA1_Channel2_BASE* = (DMA1_BASE + 0x0000001C)
  DMA1_Channel3_BASE* = (DMA1_BASE + 0x00000030)
  DMA1_Channel4_BASE* = (DMA1_BASE + 0x00000044)
  DMA1_Channel5_BASE* = (DMA1_BASE + 0x00000058)
  RCC_BASE* = (AHBPERIPH_BASE + 0x00001000)
  FLASH_R_BASE* = (AHBPERIPH_BASE + 0x00002000) # !< FLASH registers base address
  OB_BASE* = (cast[uint32](0x1FFFF800)) # !< FLASH Option Bytes base address
  FLASHSIZE_BASE* = (cast[uint32](0x1FFFF7CC)) # !< FLASH Size register base address
  UID_BASE* = (cast[uint32](0x1FFFF7AC)) # !< Unique device ID register base address
  CRC_BASE* = (AHBPERIPH_BASE + 0x00003000)

# !< AHB2 peripherals

const
  GPIOA_BASE* = (AHB2PERIPH_BASE + 0x00000000)
  GPIOB_BASE* = (AHB2PERIPH_BASE + 0x00000400)
  GPIOC_BASE* = (AHB2PERIPH_BASE + 0x00000800)
  GPIOD_BASE* = (AHB2PERIPH_BASE + 0x00000C00)
  GPIOF_BASE* = (AHB2PERIPH_BASE + 0x00001400)

# *
#  @}
##
# * @addtogroup Peripheral_declaration
#  @{
##

const
  TIM3* = (cast[ptr TIM_TypeDef](TIM3_BASE))
  TIM6* = (cast[ptr TIM_TypeDef](TIM6_BASE))
  TIM14* = (cast[ptr TIM_TypeDef](TIM14_BASE))
  RTC* = (cast[ptr RTC_TypeDef](RTC_BASE))
  WWDG* = (cast[ptr WWDG_TypeDef](WWDG_BASE))
  IWDG* = (cast[ptr IWDG_TypeDef](IWDG_BASE))
  USART2* = (cast[ptr USART_TypeDef](USART2_BASE))
  I2C1* = (cast[ptr I2C_TypeDef](I2C1_BASE))
  I2C2* = (cast[ptr I2C_TypeDef](I2C2_BASE))
  PWR* = (cast[ptr PWR_TypeDef](PWR_BASE))
  SYSCFG* = (cast[ptr SYSCFG_TypeDef](SYSCFG_BASE))
  EXTI* = (cast[ptr EXTI_TypeDef](EXTI_BASE))
  ADC1* = (cast[ptr ADC_TypeDef](ADC1_BASE))
  ADC1_COMMON* = (cast[ptr ADC_Common_TypeDef](ADC_BASE))
  ADC* = (cast[ptr ADC_Common_TypeDef](ADC_BASE)) #  Kept for legacy purpose
  TIM1* = (cast[ptr TIM_TypeDef](TIM1_BASE))
  SPI1* = (cast[ptr SPI_TypeDef](SPI1_BASE))
  SPI2* = (cast[ptr SPI_TypeDef](SPI2_BASE))
  USART1* = (cast[ptr USART_TypeDef](USART1_BASE))
  TIM15* = (cast[ptr TIM_TypeDef](TIM15_BASE))
  TIM16* = (cast[ptr TIM_TypeDef](TIM16_BASE))
  TIM17* = (cast[ptr TIM_TypeDef](TIM17_BASE))
  DBGMCU* = (cast[ptr DBGMCU_TypeDef](DBGMCU_BASE))
  DMA1* = (cast[ptr DMA_TypeDef](DMA1_BASE))
  DMA1_Channel1* = (cast[ptr DMA_Channel_TypeDef](DMA1_Channel1_BASE))
  DMA1_Channel2* = (cast[ptr DMA_Channel_TypeDef](DMA1_Channel2_BASE))
  DMA1_Channel3* = (cast[ptr DMA_Channel_TypeDef](DMA1_Channel3_BASE))
  DMA1_Channel4* = (cast[ptr DMA_Channel_TypeDef](DMA1_Channel4_BASE))
  DMA1_Channel5* = (cast[ptr DMA_Channel_TypeDef](DMA1_Channel5_BASE))
  FLASH* = (cast[ptr FLASH_TypeDef](FLASH_R_BASE))
  OB* = (cast[ptr OB_TypeDef](OB_BASE))
  RCC* = (cast[ptr RCC_TypeDef](RCC_BASE))
  CRC* = (cast[ptr CRC_TypeDef](CRC_BASE))
  GPIOA* = (cast[ptr GPIO_TypeDef](GPIOA_BASE))
  GPIOB* = (cast[ptr GPIO_TypeDef](GPIOB_BASE))
  GPIOC* = (cast[ptr GPIO_TypeDef](GPIOC_BASE))
  GPIOD* = (cast[ptr GPIO_TypeDef](GPIOD_BASE))
  GPIOF* = (cast[ptr GPIO_TypeDef](GPIOF_BASE))

# *
#  @}
##
# * @addtogroup Exported_constants
#  @{
##
# * @addtogroup Peripheral_Registers_Bits_Definition
#  @{
##
# ****************************************************************************
#                          Peripheral Registers Bits Definition
# ****************************************************************************
# ****************************************************************************
##
#                       Analog to Digital Converter (ADC)
##
# ****************************************************************************
##
#  @brief Specific device feature definitions (not present on all devices in the STM32F0 serie)
##
#  Note: No specific macro feature on this device
# *******************  Bits definition for ADC_ISR register  *****************

const
  ADC_ISR_ADRDY_Pos* = (0)
  ADC_ISR_ADRDY_Msk* = (0x00000001 shl ADC_ISR_ADRDY_Pos) # !< 0x00000001
  ADC_ISR_ADRDY* = ADC_ISR_ADRDY_Msk
  ADC_ISR_EOSMP_Pos* = (1)
  ADC_ISR_EOSMP_Msk* = (0x00000001 shl ADC_ISR_EOSMP_Pos) # !< 0x00000002
  ADC_ISR_EOSMP* = ADC_ISR_EOSMP_Msk
  ADC_ISR_EOC_Pos* = (2)
  ADC_ISR_EOC_Msk* = (0x00000001 shl ADC_ISR_EOC_Pos) # !< 0x00000004
  ADC_ISR_EOC* = ADC_ISR_EOC_Msk
  ADC_ISR_EOS_Pos* = (3)
  ADC_ISR_EOS_Msk* = (0x00000001 shl ADC_ISR_EOS_Pos) # !< 0x00000008
  ADC_ISR_EOS* = ADC_ISR_EOS_Msk
  ADC_ISR_OVR_Pos* = (4)
  ADC_ISR_OVR_Msk* = (0x00000001 shl ADC_ISR_OVR_Pos) # !< 0x00000010
  ADC_ISR_OVR* = ADC_ISR_OVR_Msk
  ADC_ISR_AWD1_Pos* = (7)
  ADC_ISR_AWD1_Msk* = (0x00000001 shl ADC_ISR_AWD1_Pos) # !< 0x00000080
  ADC_ISR_AWD1* = ADC_ISR_AWD1_Msk

#  Legacy defines

const
  ADC_ISR_AWD* = (ADC_ISR_AWD1)
  ADC_ISR_EOSEQ* = (ADC_ISR_EOS)

# *******************  Bits definition for ADC_IER register  *****************

const
  ADC_IER_ADRDYIE_Pos* = (0)
  ADC_IER_ADRDYIE_Msk* = (0x00000001 shl ADC_IER_ADRDYIE_Pos) # !< 0x00000001
  ADC_IER_ADRDYIE* = ADC_IER_ADRDYIE_Msk
  ADC_IER_EOSMPIE_Pos* = (1)
  ADC_IER_EOSMPIE_Msk* = (0x00000001 shl ADC_IER_EOSMPIE_Pos) # !< 0x00000002
  ADC_IER_EOSMPIE* = ADC_IER_EOSMPIE_Msk
  ADC_IER_EOCIE_Pos* = (2)
  ADC_IER_EOCIE_Msk* = (0x00000001 shl ADC_IER_EOCIE_Pos) # !< 0x00000004
  ADC_IER_EOCIE* = ADC_IER_EOCIE_Msk
  ADC_IER_EOSIE_Pos* = (3)
  ADC_IER_EOSIE_Msk* = (0x00000001 shl ADC_IER_EOSIE_Pos) # !< 0x00000008
  ADC_IER_EOSIE* = ADC_IER_EOSIE_Msk
  ADC_IER_OVRIE_Pos* = (4)
  ADC_IER_OVRIE_Msk* = (0x00000001 shl ADC_IER_OVRIE_Pos) # !< 0x00000010
  ADC_IER_OVRIE* = ADC_IER_OVRIE_Msk
  ADC_IER_AWD1IE_Pos* = (7)
  ADC_IER_AWD1IE_Msk* = (0x00000001 shl ADC_IER_AWD1IE_Pos) # !< 0x00000080
  ADC_IER_AWD1IE* = ADC_IER_AWD1IE_Msk

#  Legacy defines

const
  ADC_IER_AWDIE* = (ADC_IER_AWD1IE)
  ADC_IER_EOSEQIE* = (ADC_IER_EOSIE)

# *******************  Bits definition for ADC_CR register  ******************

const
  ADC_CR_ADEN_Pos* = (0)
  ADC_CR_ADEN_Msk* = (0x00000001 shl ADC_CR_ADEN_Pos) # !< 0x00000001
  ADC_CR_ADEN* = ADC_CR_ADEN_Msk
  ADC_CR_ADDIS_Pos* = (1)
  ADC_CR_ADDIS_Msk* = (0x00000001 shl ADC_CR_ADDIS_Pos) # !< 0x00000002
  ADC_CR_ADDIS* = ADC_CR_ADDIS_Msk
  ADC_CR_ADSTART_Pos* = (2)
  ADC_CR_ADSTART_Msk* = (0x00000001 shl ADC_CR_ADSTART_Pos) # !< 0x00000004
  ADC_CR_ADSTART* = ADC_CR_ADSTART_Msk
  ADC_CR_ADSTP_Pos* = (4)
  ADC_CR_ADSTP_Msk* = (0x00000001 shl ADC_CR_ADSTP_Pos) # !< 0x00000010
  ADC_CR_ADSTP* = ADC_CR_ADSTP_Msk
  ADC_CR_ADCAL_Pos* = (31)
  ADC_CR_ADCAL_Msk* = (0x00000001 shl ADC_CR_ADCAL_Pos) # !< 0x80000000
  ADC_CR_ADCAL* = ADC_CR_ADCAL_Msk

# ******************  Bits definition for ADC_CFGR1 register  ****************

const
  ADC_CFGR1_DMAEN_Pos* = (0)
  ADC_CFGR1_DMAEN_Msk* = (0x00000001 shl ADC_CFGR1_DMAEN_Pos) # !< 0x00000001
  ADC_CFGR1_DMAEN* = ADC_CFGR1_DMAEN_Msk
  ADC_CFGR1_DMACFG_Pos* = (1)
  ADC_CFGR1_DMACFG_Msk* = (0x00000001 shl ADC_CFGR1_DMACFG_Pos) # !< 0x00000002
  ADC_CFGR1_DMACFG* = ADC_CFGR1_DMACFG_Msk
  ADC_CFGR1_SCANDIR_Pos* = (2)
  ADC_CFGR1_SCANDIR_Msk* = (0x00000001 shl ADC_CFGR1_SCANDIR_Pos) # !< 0x00000004
  ADC_CFGR1_SCANDIR* = ADC_CFGR1_SCANDIR_Msk
  ADC_CFGR1_RES_Pos* = (3)
  ADC_CFGR1_RES_Msk* = (0x00000003 shl ADC_CFGR1_RES_Pos) # !< 0x00000018
  ADC_CFGR1_RES* = ADC_CFGR1_RES_Msk
  ADC_CFGR1_RES_0* = (0x00000001 shl ADC_CFGR1_RES_Pos) # !< 0x00000008
  ADC_CFGR1_RES_1* = (0x00000002 shl ADC_CFGR1_RES_Pos) # !< 0x00000010
  ADC_CFGR1_ALIGN_Pos* = (5)
  ADC_CFGR1_ALIGN_Msk* = (0x00000001 shl ADC_CFGR1_ALIGN_Pos) # !< 0x00000020
  ADC_CFGR1_ALIGN* = ADC_CFGR1_ALIGN_Msk
  ADC_CFGR1_EXTSEL_Pos* = (6)
  ADC_CFGR1_EXTSEL_Msk* = (0x00000007 shl ADC_CFGR1_EXTSEL_Pos) # !< 0x000001C0
  ADC_CFGR1_EXTSEL* = ADC_CFGR1_EXTSEL_Msk
  ADC_CFGR1_EXTSEL_0* = (0x00000001 shl ADC_CFGR1_EXTSEL_Pos) # !< 0x00000040
  ADC_CFGR1_EXTSEL_1* = (0x00000002 shl ADC_CFGR1_EXTSEL_Pos) # !< 0x00000080
  ADC_CFGR1_EXTSEL_2* = (0x00000004 shl ADC_CFGR1_EXTSEL_Pos) # !< 0x00000100
  ADC_CFGR1_EXTEN_Pos* = (10)
  ADC_CFGR1_EXTEN_Msk* = (0x00000003 shl ADC_CFGR1_EXTEN_Pos) # !< 0x00000C00
  ADC_CFGR1_EXTEN* = ADC_CFGR1_EXTEN_Msk
  ADC_CFGR1_EXTEN_0* = (0x00000001 shl ADC_CFGR1_EXTEN_Pos) # !< 0x00000400
  ADC_CFGR1_EXTEN_1* = (0x00000002 shl ADC_CFGR1_EXTEN_Pos) # !< 0x00000800
  ADC_CFGR1_OVRMOD_Pos* = (12)
  ADC_CFGR1_OVRMOD_Msk* = (0x00000001 shl ADC_CFGR1_OVRMOD_Pos) # !< 0x00001000
  ADC_CFGR1_OVRMOD* = ADC_CFGR1_OVRMOD_Msk
  ADC_CFGR1_CONT_Pos* = (13)
  ADC_CFGR1_CONT_Msk* = (0x00000001 shl ADC_CFGR1_CONT_Pos) # !< 0x00002000
  ADC_CFGR1_CONT* = ADC_CFGR1_CONT_Msk
  ADC_CFGR1_WAIT_Pos* = (14)
  ADC_CFGR1_WAIT_Msk* = (0x00000001 shl ADC_CFGR1_WAIT_Pos) # !< 0x00004000
  ADC_CFGR1_WAIT* = ADC_CFGR1_WAIT_Msk
  ADC_CFGR1_AUTOFF_Pos* = (15)
  ADC_CFGR1_AUTOFF_Msk* = (0x00000001 shl ADC_CFGR1_AUTOFF_Pos) # !< 0x00008000
  ADC_CFGR1_AUTOFF* = ADC_CFGR1_AUTOFF_Msk
  ADC_CFGR1_DISCEN_Pos* = (16)
  ADC_CFGR1_DISCEN_Msk* = (0x00000001 shl ADC_CFGR1_DISCEN_Pos) # !< 0x00010000
  ADC_CFGR1_DISCEN* = ADC_CFGR1_DISCEN_Msk
  ADC_CFGR1_AWD1SGL_Pos* = (22)
  ADC_CFGR1_AWD1SGL_Msk* = (0x00000001 shl ADC_CFGR1_AWD1SGL_Pos) # !< 0x00400000
  ADC_CFGR1_AWD1SGL* = ADC_CFGR1_AWD1SGL_Msk
  ADC_CFGR1_AWD1EN_Pos* = (23)
  ADC_CFGR1_AWD1EN_Msk* = (0x00000001 shl ADC_CFGR1_AWD1EN_Pos) # !< 0x00800000
  ADC_CFGR1_AWD1EN* = ADC_CFGR1_AWD1EN_Msk
  ADC_CFGR1_AWD1CH_Pos* = (26)
  ADC_CFGR1_AWD1CH_Msk* = (0x0000001F shl ADC_CFGR1_AWD1CH_Pos) # !< 0x7C000000
  ADC_CFGR1_AWD1CH* = ADC_CFGR1_AWD1CH_Msk
  ADC_CFGR1_AWD1CH_0* = (0x00000001 shl ADC_CFGR1_AWD1CH_Pos) # !< 0x04000000
  ADC_CFGR1_AWD1CH_1* = (0x00000002 shl ADC_CFGR1_AWD1CH_Pos) # !< 0x08000000
  ADC_CFGR1_AWD1CH_2* = (0x00000004 shl ADC_CFGR1_AWD1CH_Pos) # !< 0x10000000
  ADC_CFGR1_AWD1CH_3* = (0x00000008 shl ADC_CFGR1_AWD1CH_Pos) # !< 0x20000000
  ADC_CFGR1_AWD1CH_4* = (0x00000010 shl ADC_CFGR1_AWD1CH_Pos) # !< 0x40000000

#  Legacy defines

const
  ADC_CFGR1_AUTDLY* = (ADC_CFGR1_WAIT)
  ADC_CFGR1_AWDSGL* = (ADC_CFGR1_AWD1SGL)
  ADC_CFGR1_AWDEN* = (ADC_CFGR1_AWD1EN)
  ADC_CFGR1_AWDCH* = (ADC_CFGR1_AWD1CH)
  ADC_CFGR1_AWDCH_0* = (ADC_CFGR1_AWD1CH_0)
  ADC_CFGR1_AWDCH_1* = (ADC_CFGR1_AWD1CH_1)
  ADC_CFGR1_AWDCH_2* = (ADC_CFGR1_AWD1CH_2)
  ADC_CFGR1_AWDCH_3* = (ADC_CFGR1_AWD1CH_3)
  ADC_CFGR1_AWDCH_4* = (ADC_CFGR1_AWD1CH_4)

# ******************  Bits definition for ADC_CFGR2 register  ****************

const
  ADC_CFGR2_CKMODE_Pos* = (30)
  ADC_CFGR2_CKMODE_Msk* = (0x00000003 shl ADC_CFGR2_CKMODE_Pos) # !< 0xC0000000
  ADC_CFGR2_CKMODE* = ADC_CFGR2_CKMODE_Msk
  ADC_CFGR2_CKMODE_1* = (0x00000002 shl ADC_CFGR2_CKMODE_Pos) # !< 0x80000000
  ADC_CFGR2_CKMODE_0* = (0x00000001 shl ADC_CFGR2_CKMODE_Pos) # !< 0x40000000

#  Legacy defines

const
  ADC_CFGR2_JITOFFDIV4* = (ADC_CFGR2_CKMODE_1) # !< ADC clocked by PCLK div4
  ADC_CFGR2_JITOFFDIV2* = (ADC_CFGR2_CKMODE_0) # !< ADC clocked by PCLK div2

# *****************  Bit definition for ADC_SMPR register  *******************

const
  ADC_SMPR_SMP_Pos* = (0)
  ADC_SMPR_SMP_Msk* = (0x00000007 shl ADC_SMPR_SMP_Pos) # !< 0x00000007
  ADC_SMPR_SMP* = ADC_SMPR_SMP_Msk
  ADC_SMPR_SMP_0* = (0x00000001 shl ADC_SMPR_SMP_Pos) # !< 0x00000001
  ADC_SMPR_SMP_1* = (0x00000002 shl ADC_SMPR_SMP_Pos) # !< 0x00000002
  ADC_SMPR_SMP_2* = (0x00000004 shl ADC_SMPR_SMP_Pos) # !< 0x00000004

#  Legacy defines

const
  ADC_SMPR1_SMPR* = (ADC_SMPR_SMP) # !< SMP[2:0] bits (Sampling time selection)
  ADC_SMPR1_SMPR_0* = (ADC_SMPR_SMP_0) # !< bit 0
  ADC_SMPR1_SMPR_1* = (ADC_SMPR_SMP_1) # !< bit 1
  ADC_SMPR1_SMPR_2* = (ADC_SMPR_SMP_2) # !< bit 2

# ******************  Bit definition for ADC_TR register  *******************

const
  ADC_TR1_LT1_Pos* = (0)
  ADC_TR1_LT1_Msk* = (0x00000FFF shl ADC_TR1_LT1_Pos) # !< 0x00000FFF
  ADC_TR1_LT1* = ADC_TR1_LT1_Msk
  ADC_TR1_LT1_0* = (0x00000001 shl ADC_TR1_LT1_Pos) # !< 0x00000001
  ADC_TR1_LT1_1* = (0x00000002 shl ADC_TR1_LT1_Pos) # !< 0x00000002
  ADC_TR1_LT1_2* = (0x00000004 shl ADC_TR1_LT1_Pos) # !< 0x00000004
  ADC_TR1_LT1_3* = (0x00000008 shl ADC_TR1_LT1_Pos) # !< 0x00000008
  ADC_TR1_LT1_4* = (0x00000010 shl ADC_TR1_LT1_Pos) # !< 0x00000010
  ADC_TR1_LT1_5* = (0x00000020 shl ADC_TR1_LT1_Pos) # !< 0x00000020
  ADC_TR1_LT1_6* = (0x00000040 shl ADC_TR1_LT1_Pos) # !< 0x00000040
  ADC_TR1_LT1_7* = (0x00000080 shl ADC_TR1_LT1_Pos) # !< 0x00000080
  ADC_TR1_LT1_8* = (0x00000100 shl ADC_TR1_LT1_Pos) # !< 0x00000100
  ADC_TR1_LT1_9* = (0x00000200 shl ADC_TR1_LT1_Pos) # !< 0x00000200
  ADC_TR1_LT1_10* = (0x00000400 shl ADC_TR1_LT1_Pos) # !< 0x00000400
  ADC_TR1_LT1_11* = (0x00000800 shl ADC_TR1_LT1_Pos) # !< 0x00000800
  ADC_TR1_HT1_Pos* = (16)
  ADC_TR1_HT1_Msk* = (0x00000FFF shl ADC_TR1_HT1_Pos) # !< 0x0FFF0000
  ADC_TR1_HT1* = ADC_TR1_HT1_Msk
  ADC_TR1_HT1_0* = (0x00000001 shl ADC_TR1_HT1_Pos) # !< 0x00010000
  ADC_TR1_HT1_1* = (0x00000002 shl ADC_TR1_HT1_Pos) # !< 0x00020000
  ADC_TR1_HT1_2* = (0x00000004 shl ADC_TR1_HT1_Pos) # !< 0x00040000
  ADC_TR1_HT1_3* = (0x00000008 shl ADC_TR1_HT1_Pos) # !< 0x00080000
  ADC_TR1_HT1_4* = (0x00000010 shl ADC_TR1_HT1_Pos) # !< 0x00100000
  ADC_TR1_HT1_5* = (0x00000020 shl ADC_TR1_HT1_Pos) # !< 0x00200000
  ADC_TR1_HT1_6* = (0x00000040 shl ADC_TR1_HT1_Pos) # !< 0x00400000
  ADC_TR1_HT1_7* = (0x00000080 shl ADC_TR1_HT1_Pos) # !< 0x00800000
  ADC_TR1_HT1_8* = (0x00000100 shl ADC_TR1_HT1_Pos) # !< 0x01000000
  ADC_TR1_HT1_9* = (0x00000200 shl ADC_TR1_HT1_Pos) # !< 0x02000000
  ADC_TR1_HT1_10* = (0x00000400 shl ADC_TR1_HT1_Pos) # !< 0x04000000
  ADC_TR1_HT1_11* = (0x00000800 shl ADC_TR1_HT1_Pos) # !< 0x08000000

#  Legacy defines

const
  ADC_TR_HT* = (ADC_TR1_HT1)
  ADC_TR_LT* = (ADC_TR1_LT1)
  ADC_HTR_HT* = (ADC_TR1_HT1)
  ADC_LTR_LT* = (ADC_TR1_LT1)

# *****************  Bit definition for ADC_CHSELR register  *****************

const
  ADC_CHSELR_CHSEL_Pos* = (0)
  ADC_CHSELR_CHSEL_Msk* = (0x0007FFFF shl ADC_CHSELR_CHSEL_Pos) # !< 0x0007FFFF
  ADC_CHSELR_CHSEL* = ADC_CHSELR_CHSEL_Msk
  ADC_CHSELR_CHSEL18_Pos* = (18)
  ADC_CHSELR_CHSEL18_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL18_Pos) # !< 0x00040000
  ADC_CHSELR_CHSEL18* = ADC_CHSELR_CHSEL18_Msk
  ADC_CHSELR_CHSEL17_Pos* = (17)
  ADC_CHSELR_CHSEL17_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL17_Pos) # !< 0x00020000
  ADC_CHSELR_CHSEL17* = ADC_CHSELR_CHSEL17_Msk
  ADC_CHSELR_CHSEL16_Pos* = (16)
  ADC_CHSELR_CHSEL16_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL16_Pos) # !< 0x00010000
  ADC_CHSELR_CHSEL16* = ADC_CHSELR_CHSEL16_Msk
  ADC_CHSELR_CHSEL15_Pos* = (15)
  ADC_CHSELR_CHSEL15_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL15_Pos) # !< 0x00008000
  ADC_CHSELR_CHSEL15* = ADC_CHSELR_CHSEL15_Msk
  ADC_CHSELR_CHSEL14_Pos* = (14)
  ADC_CHSELR_CHSEL14_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL14_Pos) # !< 0x00004000
  ADC_CHSELR_CHSEL14* = ADC_CHSELR_CHSEL14_Msk
  ADC_CHSELR_CHSEL13_Pos* = (13)
  ADC_CHSELR_CHSEL13_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL13_Pos) # !< 0x00002000
  ADC_CHSELR_CHSEL13* = ADC_CHSELR_CHSEL13_Msk
  ADC_CHSELR_CHSEL12_Pos* = (12)
  ADC_CHSELR_CHSEL12_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL12_Pos) # !< 0x00001000
  ADC_CHSELR_CHSEL12* = ADC_CHSELR_CHSEL12_Msk
  ADC_CHSELR_CHSEL11_Pos* = (11)
  ADC_CHSELR_CHSEL11_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL11_Pos) # !< 0x00000800
  ADC_CHSELR_CHSEL11* = ADC_CHSELR_CHSEL11_Msk
  ADC_CHSELR_CHSEL10_Pos* = (10)
  ADC_CHSELR_CHSEL10_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL10_Pos) # !< 0x00000400
  ADC_CHSELR_CHSEL10* = ADC_CHSELR_CHSEL10_Msk
  ADC_CHSELR_CHSEL9_Pos* = (9)
  ADC_CHSELR_CHSEL9_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL9_Pos) # !< 0x00000200
  ADC_CHSELR_CHSEL9* = ADC_CHSELR_CHSEL9_Msk
  ADC_CHSELR_CHSEL8_Pos* = (8)
  ADC_CHSELR_CHSEL8_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL8_Pos) # !< 0x00000100
  ADC_CHSELR_CHSEL8* = ADC_CHSELR_CHSEL8_Msk
  ADC_CHSELR_CHSEL7_Pos* = (7)
  ADC_CHSELR_CHSEL7_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL7_Pos) # !< 0x00000080
  ADC_CHSELR_CHSEL7* = ADC_CHSELR_CHSEL7_Msk
  ADC_CHSELR_CHSEL6_Pos* = (6)
  ADC_CHSELR_CHSEL6_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL6_Pos) # !< 0x00000040
  ADC_CHSELR_CHSEL6* = ADC_CHSELR_CHSEL6_Msk
  ADC_CHSELR_CHSEL5_Pos* = (5)
  ADC_CHSELR_CHSEL5_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL5_Pos) # !< 0x00000020
  ADC_CHSELR_CHSEL5* = ADC_CHSELR_CHSEL5_Msk
  ADC_CHSELR_CHSEL4_Pos* = (4)
  ADC_CHSELR_CHSEL4_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL4_Pos) # !< 0x00000010
  ADC_CHSELR_CHSEL4* = ADC_CHSELR_CHSEL4_Msk
  ADC_CHSELR_CHSEL3_Pos* = (3)
  ADC_CHSELR_CHSEL3_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL3_Pos) # !< 0x00000008
  ADC_CHSELR_CHSEL3* = ADC_CHSELR_CHSEL3_Msk
  ADC_CHSELR_CHSEL2_Pos* = (2)
  ADC_CHSELR_CHSEL2_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL2_Pos) # !< 0x00000004
  ADC_CHSELR_CHSEL2* = ADC_CHSELR_CHSEL2_Msk
  ADC_CHSELR_CHSEL1_Pos* = (1)
  ADC_CHSELR_CHSEL1_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL1_Pos) # !< 0x00000002
  ADC_CHSELR_CHSEL1* = ADC_CHSELR_CHSEL1_Msk
  ADC_CHSELR_CHSEL0_Pos* = (0)
  ADC_CHSELR_CHSEL0_Msk* = (0x00000001 shl ADC_CHSELR_CHSEL0_Pos) # !< 0x00000001
  ADC_CHSELR_CHSEL0* = ADC_CHSELR_CHSEL0_Msk

# *******************  Bit definition for ADC_DR register  *******************

const
  ADC_DR_DATA_Pos* = (0)
  ADC_DR_DATA_Msk* = (0x0000FFFF shl ADC_DR_DATA_Pos) # !< 0x0000FFFF
  ADC_DR_DATA* = ADC_DR_DATA_Msk
  ADC_DR_DATA_0* = (0x00000001 shl ADC_DR_DATA_Pos) # !< 0x00000001
  ADC_DR_DATA_1* = (0x00000002 shl ADC_DR_DATA_Pos) # !< 0x00000002
  ADC_DR_DATA_2* = (0x00000004 shl ADC_DR_DATA_Pos) # !< 0x00000004
  ADC_DR_DATA_3* = (0x00000008 shl ADC_DR_DATA_Pos) # !< 0x00000008
  ADC_DR_DATA_4* = (0x00000010 shl ADC_DR_DATA_Pos) # !< 0x00000010
  ADC_DR_DATA_5* = (0x00000020 shl ADC_DR_DATA_Pos) # !< 0x00000020
  ADC_DR_DATA_6* = (0x00000040 shl ADC_DR_DATA_Pos) # !< 0x00000040
  ADC_DR_DATA_7* = (0x00000080 shl ADC_DR_DATA_Pos) # !< 0x00000080
  ADC_DR_DATA_8* = (0x00000100 shl ADC_DR_DATA_Pos) # !< 0x00000100
  ADC_DR_DATA_9* = (0x00000200 shl ADC_DR_DATA_Pos) # !< 0x00000200
  ADC_DR_DATA_10* = (0x00000400 shl ADC_DR_DATA_Pos) # !< 0x00000400
  ADC_DR_DATA_11* = (0x00000800 shl ADC_DR_DATA_Pos) # !< 0x00000800
  ADC_DR_DATA_12* = (0x00001000 shl ADC_DR_DATA_Pos) # !< 0x00001000
  ADC_DR_DATA_13* = (0x00002000 shl ADC_DR_DATA_Pos) # !< 0x00002000
  ADC_DR_DATA_14* = (0x00004000 shl ADC_DR_DATA_Pos) # !< 0x00004000
  ADC_DR_DATA_15* = (0x00008000 shl ADC_DR_DATA_Pos) # !< 0x00008000

# ************************  ADC Common registers  ****************************
# ******************  Bit definition for ADC_CCR register  *******************

const
  ADC_CCR_VREFEN_Pos* = (22)
  ADC_CCR_VREFEN_Msk* = (0x00000001 shl ADC_CCR_VREFEN_Pos) # !< 0x00400000
  ADC_CCR_VREFEN* = ADC_CCR_VREFEN_Msk
  ADC_CCR_TSEN_Pos* = (23)
  ADC_CCR_TSEN_Msk* = (0x00000001 shl ADC_CCR_TSEN_Pos) # !< 0x00800000
  ADC_CCR_TSEN* = ADC_CCR_TSEN_Msk

# ****************************************************************************
##
#                        CRC calculation unit (CRC)
##
# ****************************************************************************
# ******************  Bit definition for CRC_DR register  ********************

const
  CRC_DR_DR_Pos* = (0)
  CRC_DR_DR_Msk* = (0xFFFFFFFF shl CRC_DR_DR_Pos) # !< 0xFFFFFFFF
  CRC_DR_DR* = CRC_DR_DR_Msk

# ******************  Bit definition for CRC_IDR register  *******************

const
  CRC_IDR_IDR* = (0x000000FF.uint8) # !< General-purpose 8-bit data register bits

# *******************  Bit definition for CRC_CR register  *******************

const
  CRC_CR_RESET_Pos* = (0)
  CRC_CR_RESET_Msk* = (0x00000001 shl CRC_CR_RESET_Pos) # !< 0x00000001
  CRC_CR_RESET* = CRC_CR_RESET_Msk
  CRC_CR_REV_IN_Pos* = (5)
  CRC_CR_REV_IN_Msk* = (0x00000003 shl CRC_CR_REV_IN_Pos) # !< 0x00000060
  CRC_CR_REV_IN* = CRC_CR_REV_IN_Msk
  CRC_CR_REV_IN_0* = (0x00000001 shl CRC_CR_REV_IN_Pos) # !< 0x00000020
  CRC_CR_REV_IN_1* = (0x00000002 shl CRC_CR_REV_IN_Pos) # !< 0x00000040
  CRC_CR_REV_OUT_Pos* = (7)
  CRC_CR_REV_OUT_Msk* = (0x00000001 shl CRC_CR_REV_OUT_Pos) # !< 0x00000080
  CRC_CR_REV_OUT* = CRC_CR_REV_OUT_Msk

# ******************  Bit definition for CRC_INIT register  ******************

const
  CRC_INIT_INIT_Pos* = (0)
  CRC_INIT_INIT_Msk* = (0xFFFFFFFF shl CRC_INIT_INIT_Pos) # !< 0xFFFFFFFF
  CRC_INIT_INIT* = CRC_INIT_INIT_Msk

# ****************************************************************************
##
#                            Debug MCU (DBGMCU)
##
# ****************************************************************************
# ***************  Bit definition for DBGMCU_IDCODE register  ****************

const
  DBGMCU_IDCODE_DEV_ID_Pos* = (0)
  DBGMCU_IDCODE_DEV_ID_Msk* = (0x00000FFF shl DBGMCU_IDCODE_DEV_ID_Pos) # !< 0x00000FFF
  DBGMCU_IDCODE_DEV_ID* = DBGMCU_IDCODE_DEV_ID_Msk
  DBGMCU_IDCODE_REV_ID_Pos* = (16)
  DBGMCU_IDCODE_REV_ID_Msk* = (0x0000FFFF shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0xFFFF0000
  DBGMCU_IDCODE_REV_ID* = DBGMCU_IDCODE_REV_ID_Msk
  DBGMCU_IDCODE_REV_ID_0* = (0x00000001 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x00010000
  DBGMCU_IDCODE_REV_ID_1* = (0x00000002 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x00020000
  DBGMCU_IDCODE_REV_ID_2* = (0x00000004 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x00040000
  DBGMCU_IDCODE_REV_ID_3* = (0x00000008 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x00080000
  DBGMCU_IDCODE_REV_ID_4* = (0x00000010 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x00100000
  DBGMCU_IDCODE_REV_ID_5* = (0x00000020 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x00200000
  DBGMCU_IDCODE_REV_ID_6* = (0x00000040 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x00400000
  DBGMCU_IDCODE_REV_ID_7* = (0x00000080 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x00800000
  DBGMCU_IDCODE_REV_ID_8* = (0x00000100 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x01000000
  DBGMCU_IDCODE_REV_ID_9* = (0x00000200 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x02000000
  DBGMCU_IDCODE_REV_ID_10* = (0x00000400 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x04000000
  DBGMCU_IDCODE_REV_ID_11* = (0x00000800 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x08000000
  DBGMCU_IDCODE_REV_ID_12* = (0x00001000 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x10000000
  DBGMCU_IDCODE_REV_ID_13* = (0x00002000 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x20000000
  DBGMCU_IDCODE_REV_ID_14* = (0x00004000 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x40000000
  DBGMCU_IDCODE_REV_ID_15* = (0x00008000 shl DBGMCU_IDCODE_REV_ID_Pos) # !< 0x80000000

# *****************  Bit definition for DBGMCU_CR register  ******************

const
  DBGMCU_CR_DBG_STOP_Pos* = (1)
  DBGMCU_CR_DBG_STOP_Msk* = (0x00000001 shl DBGMCU_CR_DBG_STOP_Pos) # !< 0x00000002
  DBGMCU_CR_DBG_STOP* = DBGMCU_CR_DBG_STOP_Msk
  DBGMCU_CR_DBG_STANDBY_Pos* = (2)
  DBGMCU_CR_DBG_STANDBY_Msk* = (0x00000001 shl DBGMCU_CR_DBG_STANDBY_Pos) # !< 0x00000004
  DBGMCU_CR_DBG_STANDBY* = DBGMCU_CR_DBG_STANDBY_Msk

# *****************  Bit definition for DBGMCU_APB1_FZ register  *************

const
  DBGMCU_APB1_FZ_DBG_TIM3_STOP_Pos* = (1)
  DBGMCU_APB1_FZ_DBG_TIM3_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_TIM3_STOP_Pos) # !< 0x00000002
  DBGMCU_APB1_FZ_DBG_TIM3_STOP* = DBGMCU_APB1_FZ_DBG_TIM3_STOP_Msk
  DBGMCU_APB1_FZ_DBG_TIM6_STOP_Pos* = (4)
  DBGMCU_APB1_FZ_DBG_TIM6_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_TIM6_STOP_Pos) # !< 0x00000010
  DBGMCU_APB1_FZ_DBG_TIM6_STOP* = DBGMCU_APB1_FZ_DBG_TIM6_STOP_Msk
  DBGMCU_APB1_FZ_DBG_TIM14_STOP_Pos* = (8)
  DBGMCU_APB1_FZ_DBG_TIM14_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_TIM14_STOP_Pos) # !< 0x00000100
  DBGMCU_APB1_FZ_DBG_TIM14_STOP* = DBGMCU_APB1_FZ_DBG_TIM14_STOP_Msk
  DBGMCU_APB1_FZ_DBG_RTC_STOP_Pos* = (10)
  DBGMCU_APB1_FZ_DBG_RTC_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_RTC_STOP_Pos) # !< 0x00000400
  DBGMCU_APB1_FZ_DBG_RTC_STOP* = DBGMCU_APB1_FZ_DBG_RTC_STOP_Msk
  DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos* = (11)
  DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos) # !< 0x00000800
  DBGMCU_APB1_FZ_DBG_WWDG_STOP* = DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk
  DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos* = (12)
  DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos) # !< 0x00001000
  DBGMCU_APB1_FZ_DBG_IWDG_STOP* = DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk
  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos* = (21)
  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos) # !< 0x00200000
  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT* = DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk

# *****************  Bit definition for DBGMCU_APB2_FZ register  *************

const
  DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos* = (11)
  DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos) # !< 0x00000800
  DBGMCU_APB2_FZ_DBG_TIM1_STOP* = DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk
  DBGMCU_APB2_FZ_DBG_TIM15_STOP_Pos* = (16)
  DBGMCU_APB2_FZ_DBG_TIM15_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM15_STOP_Pos) # !< 0x00010000
  DBGMCU_APB2_FZ_DBG_TIM15_STOP* = DBGMCU_APB2_FZ_DBG_TIM15_STOP_Msk
  DBGMCU_APB2_FZ_DBG_TIM16_STOP_Pos* = (17)
  DBGMCU_APB2_FZ_DBG_TIM16_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM16_STOP_Pos) # !< 0x00020000
  DBGMCU_APB2_FZ_DBG_TIM16_STOP* = DBGMCU_APB2_FZ_DBG_TIM16_STOP_Msk
  DBGMCU_APB2_FZ_DBG_TIM17_STOP_Pos* = (18)
  DBGMCU_APB2_FZ_DBG_TIM17_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM17_STOP_Pos) # !< 0x00040000
  DBGMCU_APB2_FZ_DBG_TIM17_STOP* = DBGMCU_APB2_FZ_DBG_TIM17_STOP_Msk

# ****************************************************************************
##
#                            DMA Controller (DMA)
##
# ****************************************************************************
# ******************  Bit definition for DMA_ISR register  *******************

const
  DMA_ISR_GIF1_Pos* = (0)
  DMA_ISR_GIF1_Msk* = (0x00000001 shl DMA_ISR_GIF1_Pos) # !< 0x00000001
  DMA_ISR_GIF1* = DMA_ISR_GIF1_Msk
  DMA_ISR_TCIF1_Pos* = (1)
  DMA_ISR_TCIF1_Msk* = (0x00000001 shl DMA_ISR_TCIF1_Pos) # !< 0x00000002
  DMA_ISR_TCIF1* = DMA_ISR_TCIF1_Msk
  DMA_ISR_HTIF1_Pos* = (2)
  DMA_ISR_HTIF1_Msk* = (0x00000001 shl DMA_ISR_HTIF1_Pos) # !< 0x00000004
  DMA_ISR_HTIF1* = DMA_ISR_HTIF1_Msk
  DMA_ISR_TEIF1_Pos* = (3)
  DMA_ISR_TEIF1_Msk* = (0x00000001 shl DMA_ISR_TEIF1_Pos) # !< 0x00000008
  DMA_ISR_TEIF1* = DMA_ISR_TEIF1_Msk
  DMA_ISR_GIF2_Pos* = (4)
  DMA_ISR_GIF2_Msk* = (0x00000001 shl DMA_ISR_GIF2_Pos) # !< 0x00000010
  DMA_ISR_GIF2* = DMA_ISR_GIF2_Msk
  DMA_ISR_TCIF2_Pos* = (5)
  DMA_ISR_TCIF2_Msk* = (0x00000001 shl DMA_ISR_TCIF2_Pos) # !< 0x00000020
  DMA_ISR_TCIF2* = DMA_ISR_TCIF2_Msk
  DMA_ISR_HTIF2_Pos* = (6)
  DMA_ISR_HTIF2_Msk* = (0x00000001 shl DMA_ISR_HTIF2_Pos) # !< 0x00000040
  DMA_ISR_HTIF2* = DMA_ISR_HTIF2_Msk
  DMA_ISR_TEIF2_Pos* = (7)
  DMA_ISR_TEIF2_Msk* = (0x00000001 shl DMA_ISR_TEIF2_Pos) # !< 0x00000080
  DMA_ISR_TEIF2* = DMA_ISR_TEIF2_Msk
  DMA_ISR_GIF3_Pos* = (8)
  DMA_ISR_GIF3_Msk* = (0x00000001 shl DMA_ISR_GIF3_Pos) # !< 0x00000100
  DMA_ISR_GIF3* = DMA_ISR_GIF3_Msk
  DMA_ISR_TCIF3_Pos* = (9)
  DMA_ISR_TCIF3_Msk* = (0x00000001 shl DMA_ISR_TCIF3_Pos) # !< 0x00000200
  DMA_ISR_TCIF3* = DMA_ISR_TCIF3_Msk
  DMA_ISR_HTIF3_Pos* = (10)
  DMA_ISR_HTIF3_Msk* = (0x00000001 shl DMA_ISR_HTIF3_Pos) # !< 0x00000400
  DMA_ISR_HTIF3* = DMA_ISR_HTIF3_Msk
  DMA_ISR_TEIF3_Pos* = (11)
  DMA_ISR_TEIF3_Msk* = (0x00000001 shl DMA_ISR_TEIF3_Pos) # !< 0x00000800
  DMA_ISR_TEIF3* = DMA_ISR_TEIF3_Msk
  DMA_ISR_GIF4_Pos* = (12)
  DMA_ISR_GIF4_Msk* = (0x00000001 shl DMA_ISR_GIF4_Pos) # !< 0x00001000
  DMA_ISR_GIF4* = DMA_ISR_GIF4_Msk
  DMA_ISR_TCIF4_Pos* = (13)
  DMA_ISR_TCIF4_Msk* = (0x00000001 shl DMA_ISR_TCIF4_Pos) # !< 0x00002000
  DMA_ISR_TCIF4* = DMA_ISR_TCIF4_Msk
  DMA_ISR_HTIF4_Pos* = (14)
  DMA_ISR_HTIF4_Msk* = (0x00000001 shl DMA_ISR_HTIF4_Pos) # !< 0x00004000
  DMA_ISR_HTIF4* = DMA_ISR_HTIF4_Msk
  DMA_ISR_TEIF4_Pos* = (15)
  DMA_ISR_TEIF4_Msk* = (0x00000001 shl DMA_ISR_TEIF4_Pos) # !< 0x00008000
  DMA_ISR_TEIF4* = DMA_ISR_TEIF4_Msk
  DMA_ISR_GIF5_Pos* = (16)
  DMA_ISR_GIF5_Msk* = (0x00000001 shl DMA_ISR_GIF5_Pos) # !< 0x00010000
  DMA_ISR_GIF5* = DMA_ISR_GIF5_Msk
  DMA_ISR_TCIF5_Pos* = (17)
  DMA_ISR_TCIF5_Msk* = (0x00000001 shl DMA_ISR_TCIF5_Pos) # !< 0x00020000
  DMA_ISR_TCIF5* = DMA_ISR_TCIF5_Msk
  DMA_ISR_HTIF5_Pos* = (18)
  DMA_ISR_HTIF5_Msk* = (0x00000001 shl DMA_ISR_HTIF5_Pos) # !< 0x00040000
  DMA_ISR_HTIF5* = DMA_ISR_HTIF5_Msk
  DMA_ISR_TEIF5_Pos* = (19)
  DMA_ISR_TEIF5_Msk* = (0x00000001 shl DMA_ISR_TEIF5_Pos) # !< 0x00080000
  DMA_ISR_TEIF5* = DMA_ISR_TEIF5_Msk

# ******************  Bit definition for DMA_IFCR register  ******************

const
  DMA_IFCR_CGIF1_Pos* = (0)
  DMA_IFCR_CGIF1_Msk* = (0x00000001 shl DMA_IFCR_CGIF1_Pos) # !< 0x00000001
  DMA_IFCR_CGIF1* = DMA_IFCR_CGIF1_Msk
  DMA_IFCR_CTCIF1_Pos* = (1)
  DMA_IFCR_CTCIF1_Msk* = (0x00000001 shl DMA_IFCR_CTCIF1_Pos) # !< 0x00000002
  DMA_IFCR_CTCIF1* = DMA_IFCR_CTCIF1_Msk
  DMA_IFCR_CHTIF1_Pos* = (2)
  DMA_IFCR_CHTIF1_Msk* = (0x00000001 shl DMA_IFCR_CHTIF1_Pos) # !< 0x00000004
  DMA_IFCR_CHTIF1* = DMA_IFCR_CHTIF1_Msk
  DMA_IFCR_CTEIF1_Pos* = (3)
  DMA_IFCR_CTEIF1_Msk* = (0x00000001 shl DMA_IFCR_CTEIF1_Pos) # !< 0x00000008
  DMA_IFCR_CTEIF1* = DMA_IFCR_CTEIF1_Msk
  DMA_IFCR_CGIF2_Pos* = (4)
  DMA_IFCR_CGIF2_Msk* = (0x00000001 shl DMA_IFCR_CGIF2_Pos) # !< 0x00000010
  DMA_IFCR_CGIF2* = DMA_IFCR_CGIF2_Msk
  DMA_IFCR_CTCIF2_Pos* = (5)
  DMA_IFCR_CTCIF2_Msk* = (0x00000001 shl DMA_IFCR_CTCIF2_Pos) # !< 0x00000020
  DMA_IFCR_CTCIF2* = DMA_IFCR_CTCIF2_Msk
  DMA_IFCR_CHTIF2_Pos* = (6)
  DMA_IFCR_CHTIF2_Msk* = (0x00000001 shl DMA_IFCR_CHTIF2_Pos) # !< 0x00000040
  DMA_IFCR_CHTIF2* = DMA_IFCR_CHTIF2_Msk
  DMA_IFCR_CTEIF2_Pos* = (7)
  DMA_IFCR_CTEIF2_Msk* = (0x00000001 shl DMA_IFCR_CTEIF2_Pos) # !< 0x00000080
  DMA_IFCR_CTEIF2* = DMA_IFCR_CTEIF2_Msk
  DMA_IFCR_CGIF3_Pos* = (8)
  DMA_IFCR_CGIF3_Msk* = (0x00000001 shl DMA_IFCR_CGIF3_Pos) # !< 0x00000100
  DMA_IFCR_CGIF3* = DMA_IFCR_CGIF3_Msk
  DMA_IFCR_CTCIF3_Pos* = (9)
  DMA_IFCR_CTCIF3_Msk* = (0x00000001 shl DMA_IFCR_CTCIF3_Pos) # !< 0x00000200
  DMA_IFCR_CTCIF3* = DMA_IFCR_CTCIF3_Msk
  DMA_IFCR_CHTIF3_Pos* = (10)
  DMA_IFCR_CHTIF3_Msk* = (0x00000001 shl DMA_IFCR_CHTIF3_Pos) # !< 0x00000400
  DMA_IFCR_CHTIF3* = DMA_IFCR_CHTIF3_Msk
  DMA_IFCR_CTEIF3_Pos* = (11)
  DMA_IFCR_CTEIF3_Msk* = (0x00000001 shl DMA_IFCR_CTEIF3_Pos) # !< 0x00000800
  DMA_IFCR_CTEIF3* = DMA_IFCR_CTEIF3_Msk
  DMA_IFCR_CGIF4_Pos* = (12)
  DMA_IFCR_CGIF4_Msk* = (0x00000001 shl DMA_IFCR_CGIF4_Pos) # !< 0x00001000
  DMA_IFCR_CGIF4* = DMA_IFCR_CGIF4_Msk
  DMA_IFCR_CTCIF4_Pos* = (13)
  DMA_IFCR_CTCIF4_Msk* = (0x00000001 shl DMA_IFCR_CTCIF4_Pos) # !< 0x00002000
  DMA_IFCR_CTCIF4* = DMA_IFCR_CTCIF4_Msk
  DMA_IFCR_CHTIF4_Pos* = (14)
  DMA_IFCR_CHTIF4_Msk* = (0x00000001 shl DMA_IFCR_CHTIF4_Pos) # !< 0x00004000
  DMA_IFCR_CHTIF4* = DMA_IFCR_CHTIF4_Msk
  DMA_IFCR_CTEIF4_Pos* = (15)
  DMA_IFCR_CTEIF4_Msk* = (0x00000001 shl DMA_IFCR_CTEIF4_Pos) # !< 0x00008000
  DMA_IFCR_CTEIF4* = DMA_IFCR_CTEIF4_Msk
  DMA_IFCR_CGIF5_Pos* = (16)
  DMA_IFCR_CGIF5_Msk* = (0x00000001 shl DMA_IFCR_CGIF5_Pos) # !< 0x00010000
  DMA_IFCR_CGIF5* = DMA_IFCR_CGIF5_Msk
  DMA_IFCR_CTCIF5_Pos* = (17)
  DMA_IFCR_CTCIF5_Msk* = (0x00000001 shl DMA_IFCR_CTCIF5_Pos) # !< 0x00020000
  DMA_IFCR_CTCIF5* = DMA_IFCR_CTCIF5_Msk
  DMA_IFCR_CHTIF5_Pos* = (18)
  DMA_IFCR_CHTIF5_Msk* = (0x00000001 shl DMA_IFCR_CHTIF5_Pos) # !< 0x00040000
  DMA_IFCR_CHTIF5* = DMA_IFCR_CHTIF5_Msk
  DMA_IFCR_CTEIF5_Pos* = (19)
  DMA_IFCR_CTEIF5_Msk* = (0x00000001 shl DMA_IFCR_CTEIF5_Pos) # !< 0x00080000
  DMA_IFCR_CTEIF5* = DMA_IFCR_CTEIF5_Msk

# ******************  Bit definition for DMA_CCR register  *******************

const
  DMA_CCR_EN_Pos* = (0)
  DMA_CCR_EN_Msk* = (0x00000001 shl DMA_CCR_EN_Pos) # !< 0x00000001
  DMA_CCR_EN* = DMA_CCR_EN_Msk
  DMA_CCR_TCIE_Pos* = (1)
  DMA_CCR_TCIE_Msk* = (0x00000001 shl DMA_CCR_TCIE_Pos) # !< 0x00000002
  DMA_CCR_TCIE* = DMA_CCR_TCIE_Msk
  DMA_CCR_HTIE_Pos* = (2)
  DMA_CCR_HTIE_Msk* = (0x00000001 shl DMA_CCR_HTIE_Pos) # !< 0x00000004
  DMA_CCR_HTIE* = DMA_CCR_HTIE_Msk
  DMA_CCR_TEIE_Pos* = (3)
  DMA_CCR_TEIE_Msk* = (0x00000001 shl DMA_CCR_TEIE_Pos) # !< 0x00000008
  DMA_CCR_TEIE* = DMA_CCR_TEIE_Msk
  DMA_CCR_DIR_Pos* = (4)
  DMA_CCR_DIR_Msk* = (0x00000001 shl DMA_CCR_DIR_Pos) # !< 0x00000010
  DMA_CCR_DIR* = DMA_CCR_DIR_Msk
  DMA_CCR_CIRC_Pos* = (5)
  DMA_CCR_CIRC_Msk* = (0x00000001 shl DMA_CCR_CIRC_Pos) # !< 0x00000020
  DMA_CCR_CIRC* = DMA_CCR_CIRC_Msk
  DMA_CCR_PINC_Pos* = (6)
  DMA_CCR_PINC_Msk* = (0x00000001 shl DMA_CCR_PINC_Pos) # !< 0x00000040
  DMA_CCR_PINC* = DMA_CCR_PINC_Msk
  DMA_CCR_MINC_Pos* = (7)
  DMA_CCR_MINC_Msk* = (0x00000001 shl DMA_CCR_MINC_Pos) # !< 0x00000080
  DMA_CCR_MINC* = DMA_CCR_MINC_Msk
  DMA_CCR_PSIZE_Pos* = (8)
  DMA_CCR_PSIZE_Msk* = (0x00000003 shl DMA_CCR_PSIZE_Pos) # !< 0x00000300
  DMA_CCR_PSIZE* = DMA_CCR_PSIZE_Msk
  DMA_CCR_PSIZE_0* = (0x00000001 shl DMA_CCR_PSIZE_Pos) # !< 0x00000100
  DMA_CCR_PSIZE_1* = (0x00000002 shl DMA_CCR_PSIZE_Pos) # !< 0x00000200
  DMA_CCR_MSIZE_Pos* = (10)
  DMA_CCR_MSIZE_Msk* = (0x00000003 shl DMA_CCR_MSIZE_Pos) # !< 0x00000C00
  DMA_CCR_MSIZE* = DMA_CCR_MSIZE_Msk
  DMA_CCR_MSIZE_0* = (0x00000001 shl DMA_CCR_MSIZE_Pos) # !< 0x00000400
  DMA_CCR_MSIZE_1* = (0x00000002 shl DMA_CCR_MSIZE_Pos) # !< 0x00000800
  DMA_CCR_PL_Pos* = (12)
  DMA_CCR_PL_Msk* = (0x00000003 shl DMA_CCR_PL_Pos) # !< 0x00003000
  DMA_CCR_PL* = DMA_CCR_PL_Msk
  DMA_CCR_PL_0* = (0x00000001 shl DMA_CCR_PL_Pos) # !< 0x00001000
  DMA_CCR_PL_1* = (0x00000002 shl DMA_CCR_PL_Pos) # !< 0x00002000
  DMA_CCR_MEM2MEM_Pos* = (14)
  DMA_CCR_MEM2MEM_Msk* = (0x00000001 shl DMA_CCR_MEM2MEM_Pos) # !< 0x00004000
  DMA_CCR_MEM2MEM* = DMA_CCR_MEM2MEM_Msk

# *****************  Bit definition for DMA_CNDTR register  ******************

const
  DMA_CNDTR_NDT_Pos* = (0)
  DMA_CNDTR_NDT_Msk* = (0x0000FFFF shl DMA_CNDTR_NDT_Pos) # !< 0x0000FFFF
  DMA_CNDTR_NDT* = DMA_CNDTR_NDT_Msk

# *****************  Bit definition for DMA_CPAR register  *******************

const
  DMA_CPAR_PA_Pos* = (0)
  DMA_CPAR_PA_Msk* = (0xFFFFFFFF shl DMA_CPAR_PA_Pos) # !< 0xFFFFFFFF
  DMA_CPAR_PA* = DMA_CPAR_PA_Msk

# *****************  Bit definition for DMA_CMAR register  *******************

const
  DMA_CMAR_MA_Pos* = (0)
  DMA_CMAR_MA_Msk* = (0xFFFFFFFF shl DMA_CMAR_MA_Pos) # !< 0xFFFFFFFF
  DMA_CMAR_MA* = DMA_CMAR_MA_Msk

# ****************************************************************************
##
#                  External Interrupt/Event Controller (EXTI)
##
# ****************************************************************************
# ******************  Bit definition for EXTI_IMR register  ******************

const
  EXTI_IMR_MR0_Pos* = (0)
  EXTI_IMR_MR0_Msk* = (0x00000001 shl EXTI_IMR_MR0_Pos) # !< 0x00000001
  EXTI_IMR_MR0* = EXTI_IMR_MR0_Msk
  EXTI_IMR_MR1_Pos* = (1)
  EXTI_IMR_MR1_Msk* = (0x00000001 shl EXTI_IMR_MR1_Pos) # !< 0x00000002
  EXTI_IMR_MR1* = EXTI_IMR_MR1_Msk
  EXTI_IMR_MR2_Pos* = (2)
  EXTI_IMR_MR2_Msk* = (0x00000001 shl EXTI_IMR_MR2_Pos) # !< 0x00000004
  EXTI_IMR_MR2* = EXTI_IMR_MR2_Msk
  EXTI_IMR_MR3_Pos* = (3)
  EXTI_IMR_MR3_Msk* = (0x00000001 shl EXTI_IMR_MR3_Pos) # !< 0x00000008
  EXTI_IMR_MR3* = EXTI_IMR_MR3_Msk
  EXTI_IMR_MR4_Pos* = (4)
  EXTI_IMR_MR4_Msk* = (0x00000001 shl EXTI_IMR_MR4_Pos) # !< 0x00000010
  EXTI_IMR_MR4* = EXTI_IMR_MR4_Msk
  EXTI_IMR_MR5_Pos* = (5)
  EXTI_IMR_MR5_Msk* = (0x00000001 shl EXTI_IMR_MR5_Pos) # !< 0x00000020
  EXTI_IMR_MR5* = EXTI_IMR_MR5_Msk
  EXTI_IMR_MR6_Pos* = (6)
  EXTI_IMR_MR6_Msk* = (0x00000001 shl EXTI_IMR_MR6_Pos) # !< 0x00000040
  EXTI_IMR_MR6* = EXTI_IMR_MR6_Msk
  EXTI_IMR_MR7_Pos* = (7)
  EXTI_IMR_MR7_Msk* = (0x00000001 shl EXTI_IMR_MR7_Pos) # !< 0x00000080
  EXTI_IMR_MR7* = EXTI_IMR_MR7_Msk
  EXTI_IMR_MR8_Pos* = (8)
  EXTI_IMR_MR8_Msk* = (0x00000001 shl EXTI_IMR_MR8_Pos) # !< 0x00000100
  EXTI_IMR_MR8* = EXTI_IMR_MR8_Msk
  EXTI_IMR_MR9_Pos* = (9)
  EXTI_IMR_MR9_Msk* = (0x00000001 shl EXTI_IMR_MR9_Pos) # !< 0x00000200
  EXTI_IMR_MR9* = EXTI_IMR_MR9_Msk
  EXTI_IMR_MR10_Pos* = (10)
  EXTI_IMR_MR10_Msk* = (0x00000001 shl EXTI_IMR_MR10_Pos) # !< 0x00000400
  EXTI_IMR_MR10* = EXTI_IMR_MR10_Msk
  EXTI_IMR_MR11_Pos* = (11)
  EXTI_IMR_MR11_Msk* = (0x00000001 shl EXTI_IMR_MR11_Pos) # !< 0x00000800
  EXTI_IMR_MR11* = EXTI_IMR_MR11_Msk
  EXTI_IMR_MR12_Pos* = (12)
  EXTI_IMR_MR12_Msk* = (0x00000001 shl EXTI_IMR_MR12_Pos) # !< 0x00001000
  EXTI_IMR_MR12* = EXTI_IMR_MR12_Msk
  EXTI_IMR_MR13_Pos* = (13)
  EXTI_IMR_MR13_Msk* = (0x00000001 shl EXTI_IMR_MR13_Pos) # !< 0x00002000
  EXTI_IMR_MR13* = EXTI_IMR_MR13_Msk
  EXTI_IMR_MR14_Pos* = (14)
  EXTI_IMR_MR14_Msk* = (0x00000001 shl EXTI_IMR_MR14_Pos) # !< 0x00004000
  EXTI_IMR_MR14* = EXTI_IMR_MR14_Msk
  EXTI_IMR_MR15_Pos* = (15)
  EXTI_IMR_MR15_Msk* = (0x00000001 shl EXTI_IMR_MR15_Pos) # !< 0x00008000
  EXTI_IMR_MR15* = EXTI_IMR_MR15_Msk
  EXTI_IMR_MR17_Pos* = (17)
  EXTI_IMR_MR17_Msk* = (0x00000001 shl EXTI_IMR_MR17_Pos) # !< 0x00020000
  EXTI_IMR_MR17* = EXTI_IMR_MR17_Msk
  EXTI_IMR_MR18_Pos* = (18)
  EXTI_IMR_MR18_Msk* = (0x00000001 shl EXTI_IMR_MR18_Pos) # !< 0x00040000
  EXTI_IMR_MR18* = EXTI_IMR_MR18_Msk
  EXTI_IMR_MR19_Pos* = (19)
  EXTI_IMR_MR19_Msk* = (0x00000001 shl EXTI_IMR_MR19_Pos) # !< 0x00080000
  EXTI_IMR_MR19* = EXTI_IMR_MR19_Msk
  EXTI_IMR_MR23_Pos* = (23)
  EXTI_IMR_MR23_Msk* = (0x00000001 shl EXTI_IMR_MR23_Pos) # !< 0x00800000
  EXTI_IMR_MR23* = EXTI_IMR_MR23_Msk

#  References Defines

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
  EXTI_IMR_IM17* = EXTI_IMR_MR17
  EXTI_IMR_IM18* = EXTI_IMR_MR18
  EXTI_IMR_IM19* = EXTI_IMR_MR19
  EXTI_IMR_IM23* = EXTI_IMR_MR23
  EXTI_IMR_IM_Pos* = (0)
  EXTI_IMR_IM_Msk* = (0x008EFFFF shl EXTI_IMR_IM_Pos) # !< 0x008EFFFF
  EXTI_IMR_IM* = EXTI_IMR_IM_Msk

# *****************  Bit definition for EXTI_EMR register  *******************

const
  EXTI_EMR_MR0_Pos* = (0)
  EXTI_EMR_MR0_Msk* = (0x00000001 shl EXTI_EMR_MR0_Pos) # !< 0x00000001
  EXTI_EMR_MR0* = EXTI_EMR_MR0_Msk
  EXTI_EMR_MR1_Pos* = (1)
  EXTI_EMR_MR1_Msk* = (0x00000001 shl EXTI_EMR_MR1_Pos) # !< 0x00000002
  EXTI_EMR_MR1* = EXTI_EMR_MR1_Msk
  EXTI_EMR_MR2_Pos* = (2)
  EXTI_EMR_MR2_Msk* = (0x00000001 shl EXTI_EMR_MR2_Pos) # !< 0x00000004
  EXTI_EMR_MR2* = EXTI_EMR_MR2_Msk
  EXTI_EMR_MR3_Pos* = (3)
  EXTI_EMR_MR3_Msk* = (0x00000001 shl EXTI_EMR_MR3_Pos) # !< 0x00000008
  EXTI_EMR_MR3* = EXTI_EMR_MR3_Msk
  EXTI_EMR_MR4_Pos* = (4)
  EXTI_EMR_MR4_Msk* = (0x00000001 shl EXTI_EMR_MR4_Pos) # !< 0x00000010
  EXTI_EMR_MR4* = EXTI_EMR_MR4_Msk
  EXTI_EMR_MR5_Pos* = (5)
  EXTI_EMR_MR5_Msk* = (0x00000001 shl EXTI_EMR_MR5_Pos) # !< 0x00000020
  EXTI_EMR_MR5* = EXTI_EMR_MR5_Msk
  EXTI_EMR_MR6_Pos* = (6)
  EXTI_EMR_MR6_Msk* = (0x00000001 shl EXTI_EMR_MR6_Pos) # !< 0x00000040
  EXTI_EMR_MR6* = EXTI_EMR_MR6_Msk
  EXTI_EMR_MR7_Pos* = (7)
  EXTI_EMR_MR7_Msk* = (0x00000001 shl EXTI_EMR_MR7_Pos) # !< 0x00000080
  EXTI_EMR_MR7* = EXTI_EMR_MR7_Msk
  EXTI_EMR_MR8_Pos* = (8)
  EXTI_EMR_MR8_Msk* = (0x00000001 shl EXTI_EMR_MR8_Pos) # !< 0x00000100
  EXTI_EMR_MR8* = EXTI_EMR_MR8_Msk
  EXTI_EMR_MR9_Pos* = (9)
  EXTI_EMR_MR9_Msk* = (0x00000001 shl EXTI_EMR_MR9_Pos) # !< 0x00000200
  EXTI_EMR_MR9* = EXTI_EMR_MR9_Msk
  EXTI_EMR_MR10_Pos* = (10)
  EXTI_EMR_MR10_Msk* = (0x00000001 shl EXTI_EMR_MR10_Pos) # !< 0x00000400
  EXTI_EMR_MR10* = EXTI_EMR_MR10_Msk
  EXTI_EMR_MR11_Pos* = (11)
  EXTI_EMR_MR11_Msk* = (0x00000001 shl EXTI_EMR_MR11_Pos) # !< 0x00000800
  EXTI_EMR_MR11* = EXTI_EMR_MR11_Msk
  EXTI_EMR_MR12_Pos* = (12)
  EXTI_EMR_MR12_Msk* = (0x00000001 shl EXTI_EMR_MR12_Pos) # !< 0x00001000
  EXTI_EMR_MR12* = EXTI_EMR_MR12_Msk
  EXTI_EMR_MR13_Pos* = (13)
  EXTI_EMR_MR13_Msk* = (0x00000001 shl EXTI_EMR_MR13_Pos) # !< 0x00002000
  EXTI_EMR_MR13* = EXTI_EMR_MR13_Msk
  EXTI_EMR_MR14_Pos* = (14)
  EXTI_EMR_MR14_Msk* = (0x00000001 shl EXTI_EMR_MR14_Pos) # !< 0x00004000
  EXTI_EMR_MR14* = EXTI_EMR_MR14_Msk
  EXTI_EMR_MR15_Pos* = (15)
  EXTI_EMR_MR15_Msk* = (0x00000001 shl EXTI_EMR_MR15_Pos) # !< 0x00008000
  EXTI_EMR_MR15* = EXTI_EMR_MR15_Msk
  EXTI_EMR_MR17_Pos* = (17)
  EXTI_EMR_MR17_Msk* = (0x00000001 shl EXTI_EMR_MR17_Pos) # !< 0x00020000
  EXTI_EMR_MR17* = EXTI_EMR_MR17_Msk
  EXTI_EMR_MR18_Pos* = (18)
  EXTI_EMR_MR18_Msk* = (0x00000001 shl EXTI_EMR_MR18_Pos) # !< 0x00040000
  EXTI_EMR_MR18* = EXTI_EMR_MR18_Msk
  EXTI_EMR_MR19_Pos* = (19)
  EXTI_EMR_MR19_Msk* = (0x00000001 shl EXTI_EMR_MR19_Pos) # !< 0x00080000
  EXTI_EMR_MR19* = EXTI_EMR_MR19_Msk
  EXTI_EMR_MR23_Pos* = (23)
  EXTI_EMR_MR23_Msk* = (0x00000001 shl EXTI_EMR_MR23_Pos) # !< 0x00800000
  EXTI_EMR_MR23* = EXTI_EMR_MR23_Msk

#  References Defines

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
  EXTI_EMR_EM17* = EXTI_EMR_MR17
  EXTI_EMR_EM18* = EXTI_EMR_MR18
  EXTI_EMR_EM19* = EXTI_EMR_MR19
  EXTI_EMR_EM23* = EXTI_EMR_MR23

# ******************  Bit definition for EXTI_RTSR register  *****************

const
  EXTI_RTSR_TR0_Pos* = (0)
  EXTI_RTSR_TR0_Msk* = (0x00000001 shl EXTI_RTSR_TR0_Pos) # !< 0x00000001
  EXTI_RTSR_TR0* = EXTI_RTSR_TR0_Msk
  EXTI_RTSR_TR1_Pos* = (1)
  EXTI_RTSR_TR1_Msk* = (0x00000001 shl EXTI_RTSR_TR1_Pos) # !< 0x00000002
  EXTI_RTSR_TR1* = EXTI_RTSR_TR1_Msk
  EXTI_RTSR_TR2_Pos* = (2)
  EXTI_RTSR_TR2_Msk* = (0x00000001 shl EXTI_RTSR_TR2_Pos) # !< 0x00000004
  EXTI_RTSR_TR2* = EXTI_RTSR_TR2_Msk
  EXTI_RTSR_TR3_Pos* = (3)
  EXTI_RTSR_TR3_Msk* = (0x00000001 shl EXTI_RTSR_TR3_Pos) # !< 0x00000008
  EXTI_RTSR_TR3* = EXTI_RTSR_TR3_Msk
  EXTI_RTSR_TR4_Pos* = (4)
  EXTI_RTSR_TR4_Msk* = (0x00000001 shl EXTI_RTSR_TR4_Pos) # !< 0x00000010
  EXTI_RTSR_TR4* = EXTI_RTSR_TR4_Msk
  EXTI_RTSR_TR5_Pos* = (5)
  EXTI_RTSR_TR5_Msk* = (0x00000001 shl EXTI_RTSR_TR5_Pos) # !< 0x00000020
  EXTI_RTSR_TR5* = EXTI_RTSR_TR5_Msk
  EXTI_RTSR_TR6_Pos* = (6)
  EXTI_RTSR_TR6_Msk* = (0x00000001 shl EXTI_RTSR_TR6_Pos) # !< 0x00000040
  EXTI_RTSR_TR6* = EXTI_RTSR_TR6_Msk
  EXTI_RTSR_TR7_Pos* = (7)
  EXTI_RTSR_TR7_Msk* = (0x00000001 shl EXTI_RTSR_TR7_Pos) # !< 0x00000080
  EXTI_RTSR_TR7* = EXTI_RTSR_TR7_Msk
  EXTI_RTSR_TR8_Pos* = (8)
  EXTI_RTSR_TR8_Msk* = (0x00000001 shl EXTI_RTSR_TR8_Pos) # !< 0x00000100
  EXTI_RTSR_TR8* = EXTI_RTSR_TR8_Msk
  EXTI_RTSR_TR9_Pos* = (9)
  EXTI_RTSR_TR9_Msk* = (0x00000001 shl EXTI_RTSR_TR9_Pos) # !< 0x00000200
  EXTI_RTSR_TR9* = EXTI_RTSR_TR9_Msk
  EXTI_RTSR_TR10_Pos* = (10)
  EXTI_RTSR_TR10_Msk* = (0x00000001 shl EXTI_RTSR_TR10_Pos) # !< 0x00000400
  EXTI_RTSR_TR10* = EXTI_RTSR_TR10_Msk
  EXTI_RTSR_TR11_Pos* = (11)
  EXTI_RTSR_TR11_Msk* = (0x00000001 shl EXTI_RTSR_TR11_Pos) # !< 0x00000800
  EXTI_RTSR_TR11* = EXTI_RTSR_TR11_Msk
  EXTI_RTSR_TR12_Pos* = (12)
  EXTI_RTSR_TR12_Msk* = (0x00000001 shl EXTI_RTSR_TR12_Pos) # !< 0x00001000
  EXTI_RTSR_TR12* = EXTI_RTSR_TR12_Msk
  EXTI_RTSR_TR13_Pos* = (13)
  EXTI_RTSR_TR13_Msk* = (0x00000001 shl EXTI_RTSR_TR13_Pos) # !< 0x00002000
  EXTI_RTSR_TR13* = EXTI_RTSR_TR13_Msk
  EXTI_RTSR_TR14_Pos* = (14)
  EXTI_RTSR_TR14_Msk* = (0x00000001 shl EXTI_RTSR_TR14_Pos) # !< 0x00004000
  EXTI_RTSR_TR14* = EXTI_RTSR_TR14_Msk
  EXTI_RTSR_TR15_Pos* = (15)
  EXTI_RTSR_TR15_Msk* = (0x00000001 shl EXTI_RTSR_TR15_Pos) # !< 0x00008000
  EXTI_RTSR_TR15* = EXTI_RTSR_TR15_Msk
  EXTI_RTSR_TR16_Pos* = (16)
  EXTI_RTSR_TR16_Msk* = (0x00000001 shl EXTI_RTSR_TR16_Pos) # !< 0x00010000
  EXTI_RTSR_TR16* = EXTI_RTSR_TR16_Msk
  EXTI_RTSR_TR17_Pos* = (17)
  EXTI_RTSR_TR17_Msk* = (0x00000001 shl EXTI_RTSR_TR17_Pos) # !< 0x00020000
  EXTI_RTSR_TR17* = EXTI_RTSR_TR17_Msk
  EXTI_RTSR_TR19_Pos* = (19)
  EXTI_RTSR_TR19_Msk* = (0x00000001 shl EXTI_RTSR_TR19_Pos) # !< 0x00080000
  EXTI_RTSR_TR19* = EXTI_RTSR_TR19_Msk

#  References Defines

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
  EXTI_RTSR_RT19* = EXTI_RTSR_TR19

# ******************  Bit definition for EXTI_FTSR register ******************

const
  EXTI_FTSR_TR0_Pos* = (0)
  EXTI_FTSR_TR0_Msk* = (0x00000001 shl EXTI_FTSR_TR0_Pos) # !< 0x00000001
  EXTI_FTSR_TR0* = EXTI_FTSR_TR0_Msk
  EXTI_FTSR_TR1_Pos* = (1)
  EXTI_FTSR_TR1_Msk* = (0x00000001 shl EXTI_FTSR_TR1_Pos) # !< 0x00000002
  EXTI_FTSR_TR1* = EXTI_FTSR_TR1_Msk
  EXTI_FTSR_TR2_Pos* = (2)
  EXTI_FTSR_TR2_Msk* = (0x00000001 shl EXTI_FTSR_TR2_Pos) # !< 0x00000004
  EXTI_FTSR_TR2* = EXTI_FTSR_TR2_Msk
  EXTI_FTSR_TR3_Pos* = (3)
  EXTI_FTSR_TR3_Msk* = (0x00000001 shl EXTI_FTSR_TR3_Pos) # !< 0x00000008
  EXTI_FTSR_TR3* = EXTI_FTSR_TR3_Msk
  EXTI_FTSR_TR4_Pos* = (4)
  EXTI_FTSR_TR4_Msk* = (0x00000001 shl EXTI_FTSR_TR4_Pos) # !< 0x00000010
  EXTI_FTSR_TR4* = EXTI_FTSR_TR4_Msk
  EXTI_FTSR_TR5_Pos* = (5)
  EXTI_FTSR_TR5_Msk* = (0x00000001 shl EXTI_FTSR_TR5_Pos) # !< 0x00000020
  EXTI_FTSR_TR5* = EXTI_FTSR_TR5_Msk
  EXTI_FTSR_TR6_Pos* = (6)
  EXTI_FTSR_TR6_Msk* = (0x00000001 shl EXTI_FTSR_TR6_Pos) # !< 0x00000040
  EXTI_FTSR_TR6* = EXTI_FTSR_TR6_Msk
  EXTI_FTSR_TR7_Pos* = (7)
  EXTI_FTSR_TR7_Msk* = (0x00000001 shl EXTI_FTSR_TR7_Pos) # !< 0x00000080
  EXTI_FTSR_TR7* = EXTI_FTSR_TR7_Msk
  EXTI_FTSR_TR8_Pos* = (8)
  EXTI_FTSR_TR8_Msk* = (0x00000001 shl EXTI_FTSR_TR8_Pos) # !< 0x00000100
  EXTI_FTSR_TR8* = EXTI_FTSR_TR8_Msk
  EXTI_FTSR_TR9_Pos* = (9)
  EXTI_FTSR_TR9_Msk* = (0x00000001 shl EXTI_FTSR_TR9_Pos) # !< 0x00000200
  EXTI_FTSR_TR9* = EXTI_FTSR_TR9_Msk
  EXTI_FTSR_TR10_Pos* = (10)
  EXTI_FTSR_TR10_Msk* = (0x00000001 shl EXTI_FTSR_TR10_Pos) # !< 0x00000400
  EXTI_FTSR_TR10* = EXTI_FTSR_TR10_Msk
  EXTI_FTSR_TR11_Pos* = (11)
  EXTI_FTSR_TR11_Msk* = (0x00000001 shl EXTI_FTSR_TR11_Pos) # !< 0x00000800
  EXTI_FTSR_TR11* = EXTI_FTSR_TR11_Msk
  EXTI_FTSR_TR12_Pos* = (12)
  EXTI_FTSR_TR12_Msk* = (0x00000001 shl EXTI_FTSR_TR12_Pos) # !< 0x00001000
  EXTI_FTSR_TR12* = EXTI_FTSR_TR12_Msk
  EXTI_FTSR_TR13_Pos* = (13)
  EXTI_FTSR_TR13_Msk* = (0x00000001 shl EXTI_FTSR_TR13_Pos) # !< 0x00002000
  EXTI_FTSR_TR13* = EXTI_FTSR_TR13_Msk
  EXTI_FTSR_TR14_Pos* = (14)
  EXTI_FTSR_TR14_Msk* = (0x00000001 shl EXTI_FTSR_TR14_Pos) # !< 0x00004000
  EXTI_FTSR_TR14* = EXTI_FTSR_TR14_Msk
  EXTI_FTSR_TR15_Pos* = (15)
  EXTI_FTSR_TR15_Msk* = (0x00000001 shl EXTI_FTSR_TR15_Pos) # !< 0x00008000
  EXTI_FTSR_TR15* = EXTI_FTSR_TR15_Msk
  EXTI_FTSR_TR16_Pos* = (16)
  EXTI_FTSR_TR16_Msk* = (0x00000001 shl EXTI_FTSR_TR16_Pos) # !< 0x00010000
  EXTI_FTSR_TR16* = EXTI_FTSR_TR16_Msk
  EXTI_FTSR_TR17_Pos* = (17)
  EXTI_FTSR_TR17_Msk* = (0x00000001 shl EXTI_FTSR_TR17_Pos) # !< 0x00020000
  EXTI_FTSR_TR17* = EXTI_FTSR_TR17_Msk
  EXTI_FTSR_TR19_Pos* = (19)
  EXTI_FTSR_TR19_Msk* = (0x00000001 shl EXTI_FTSR_TR19_Pos) # !< 0x00080000
  EXTI_FTSR_TR19* = EXTI_FTSR_TR19_Msk

#  References Defines

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
  EXTI_FTSR_FT19* = EXTI_FTSR_TR19

# ****************** Bit definition for EXTI_SWIER register ******************

const
  EXTI_SWIER_SWIER0_Pos* = (0)
  EXTI_SWIER_SWIER0_Msk* = (0x00000001 shl EXTI_SWIER_SWIER0_Pos) # !< 0x00000001
  EXTI_SWIER_SWIER0* = EXTI_SWIER_SWIER0_Msk
  EXTI_SWIER_SWIER1_Pos* = (1)
  EXTI_SWIER_SWIER1_Msk* = (0x00000001 shl EXTI_SWIER_SWIER1_Pos) # !< 0x00000002
  EXTI_SWIER_SWIER1* = EXTI_SWIER_SWIER1_Msk
  EXTI_SWIER_SWIER2_Pos* = (2)
  EXTI_SWIER_SWIER2_Msk* = (0x00000001 shl EXTI_SWIER_SWIER2_Pos) # !< 0x00000004
  EXTI_SWIER_SWIER2* = EXTI_SWIER_SWIER2_Msk
  EXTI_SWIER_SWIER3_Pos* = (3)
  EXTI_SWIER_SWIER3_Msk* = (0x00000001 shl EXTI_SWIER_SWIER3_Pos) # !< 0x00000008
  EXTI_SWIER_SWIER3* = EXTI_SWIER_SWIER3_Msk
  EXTI_SWIER_SWIER4_Pos* = (4)
  EXTI_SWIER_SWIER4_Msk* = (0x00000001 shl EXTI_SWIER_SWIER4_Pos) # !< 0x00000010
  EXTI_SWIER_SWIER4* = EXTI_SWIER_SWIER4_Msk
  EXTI_SWIER_SWIER5_Pos* = (5)
  EXTI_SWIER_SWIER5_Msk* = (0x00000001 shl EXTI_SWIER_SWIER5_Pos) # !< 0x00000020
  EXTI_SWIER_SWIER5* = EXTI_SWIER_SWIER5_Msk
  EXTI_SWIER_SWIER6_Pos* = (6)
  EXTI_SWIER_SWIER6_Msk* = (0x00000001 shl EXTI_SWIER_SWIER6_Pos) # !< 0x00000040
  EXTI_SWIER_SWIER6* = EXTI_SWIER_SWIER6_Msk
  EXTI_SWIER_SWIER7_Pos* = (7)
  EXTI_SWIER_SWIER7_Msk* = (0x00000001 shl EXTI_SWIER_SWIER7_Pos) # !< 0x00000080
  EXTI_SWIER_SWIER7* = EXTI_SWIER_SWIER7_Msk
  EXTI_SWIER_SWIER8_Pos* = (8)
  EXTI_SWIER_SWIER8_Msk* = (0x00000001 shl EXTI_SWIER_SWIER8_Pos) # !< 0x00000100
  EXTI_SWIER_SWIER8* = EXTI_SWIER_SWIER8_Msk
  EXTI_SWIER_SWIER9_Pos* = (9)
  EXTI_SWIER_SWIER9_Msk* = (0x00000001 shl EXTI_SWIER_SWIER9_Pos) # !< 0x00000200
  EXTI_SWIER_SWIER9* = EXTI_SWIER_SWIER9_Msk
  EXTI_SWIER_SWIER10_Pos* = (10)
  EXTI_SWIER_SWIER10_Msk* = (0x00000001 shl EXTI_SWIER_SWIER10_Pos) # !< 0x00000400
  EXTI_SWIER_SWIER10* = EXTI_SWIER_SWIER10_Msk
  EXTI_SWIER_SWIER11_Pos* = (11)
  EXTI_SWIER_SWIER11_Msk* = (0x00000001 shl EXTI_SWIER_SWIER11_Pos) # !< 0x00000800
  EXTI_SWIER_SWIER11* = EXTI_SWIER_SWIER11_Msk
  EXTI_SWIER_SWIER12_Pos* = (12)
  EXTI_SWIER_SWIER12_Msk* = (0x00000001 shl EXTI_SWIER_SWIER12_Pos) # !< 0x00001000
  EXTI_SWIER_SWIER12* = EXTI_SWIER_SWIER12_Msk
  EXTI_SWIER_SWIER13_Pos* = (13)
  EXTI_SWIER_SWIER13_Msk* = (0x00000001 shl EXTI_SWIER_SWIER13_Pos) # !< 0x00002000
  EXTI_SWIER_SWIER13* = EXTI_SWIER_SWIER13_Msk
  EXTI_SWIER_SWIER14_Pos* = (14)
  EXTI_SWIER_SWIER14_Msk* = (0x00000001 shl EXTI_SWIER_SWIER14_Pos) # !< 0x00004000
  EXTI_SWIER_SWIER14* = EXTI_SWIER_SWIER14_Msk
  EXTI_SWIER_SWIER15_Pos* = (15)
  EXTI_SWIER_SWIER15_Msk* = (0x00000001 shl EXTI_SWIER_SWIER15_Pos) # !< 0x00008000
  EXTI_SWIER_SWIER15* = EXTI_SWIER_SWIER15_Msk
  EXTI_SWIER_SWIER16_Pos* = (16)
  EXTI_SWIER_SWIER16_Msk* = (0x00000001 shl EXTI_SWIER_SWIER16_Pos) # !< 0x00010000
  EXTI_SWIER_SWIER16* = EXTI_SWIER_SWIER16_Msk
  EXTI_SWIER_SWIER17_Pos* = (17)
  EXTI_SWIER_SWIER17_Msk* = (0x00000001 shl EXTI_SWIER_SWIER17_Pos) # !< 0x00020000
  EXTI_SWIER_SWIER17* = EXTI_SWIER_SWIER17_Msk
  EXTI_SWIER_SWIER19_Pos* = (19)
  EXTI_SWIER_SWIER19_Msk* = (0x00000001 shl EXTI_SWIER_SWIER19_Pos) # !< 0x00080000
  EXTI_SWIER_SWIER19* = EXTI_SWIER_SWIER19_Msk

#  References Defines

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
  EXTI_SWIER_SWI19* = EXTI_SWIER_SWIER19

# *****************  Bit definition for EXTI_PR register  ********************

const
  EXTI_PR_PR0_Pos* = (0)
  EXTI_PR_PR0_Msk* = (0x00000001 shl EXTI_PR_PR0_Pos) # !< 0x00000001
  EXTI_PR_PR0* = EXTI_PR_PR0_Msk
  EXTI_PR_PR1_Pos* = (1)
  EXTI_PR_PR1_Msk* = (0x00000001 shl EXTI_PR_PR1_Pos) # !< 0x00000002
  EXTI_PR_PR1* = EXTI_PR_PR1_Msk
  EXTI_PR_PR2_Pos* = (2)
  EXTI_PR_PR2_Msk* = (0x00000001 shl EXTI_PR_PR2_Pos) # !< 0x00000004
  EXTI_PR_PR2* = EXTI_PR_PR2_Msk
  EXTI_PR_PR3_Pos* = (3)
  EXTI_PR_PR3_Msk* = (0x00000001 shl EXTI_PR_PR3_Pos) # !< 0x00000008
  EXTI_PR_PR3* = EXTI_PR_PR3_Msk
  EXTI_PR_PR4_Pos* = (4)
  EXTI_PR_PR4_Msk* = (0x00000001 shl EXTI_PR_PR4_Pos) # !< 0x00000010
  EXTI_PR_PR4* = EXTI_PR_PR4_Msk
  EXTI_PR_PR5_Pos* = (5)
  EXTI_PR_PR5_Msk* = (0x00000001 shl EXTI_PR_PR5_Pos) # !< 0x00000020
  EXTI_PR_PR5* = EXTI_PR_PR5_Msk
  EXTI_PR_PR6_Pos* = (6)
  EXTI_PR_PR6_Msk* = (0x00000001 shl EXTI_PR_PR6_Pos) # !< 0x00000040
  EXTI_PR_PR6* = EXTI_PR_PR6_Msk
  EXTI_PR_PR7_Pos* = (7)
  EXTI_PR_PR7_Msk* = (0x00000001 shl EXTI_PR_PR7_Pos) # !< 0x00000080
  EXTI_PR_PR7* = EXTI_PR_PR7_Msk
  EXTI_PR_PR8_Pos* = (8)
  EXTI_PR_PR8_Msk* = (0x00000001 shl EXTI_PR_PR8_Pos) # !< 0x00000100
  EXTI_PR_PR8* = EXTI_PR_PR8_Msk
  EXTI_PR_PR9_Pos* = (9)
  EXTI_PR_PR9_Msk* = (0x00000001 shl EXTI_PR_PR9_Pos) # !< 0x00000200
  EXTI_PR_PR9* = EXTI_PR_PR9_Msk
  EXTI_PR_PR10_Pos* = (10)
  EXTI_PR_PR10_Msk* = (0x00000001 shl EXTI_PR_PR10_Pos) # !< 0x00000400
  EXTI_PR_PR10* = EXTI_PR_PR10_Msk
  EXTI_PR_PR11_Pos* = (11)
  EXTI_PR_PR11_Msk* = (0x00000001 shl EXTI_PR_PR11_Pos) # !< 0x00000800
  EXTI_PR_PR11* = EXTI_PR_PR11_Msk
  EXTI_PR_PR12_Pos* = (12)
  EXTI_PR_PR12_Msk* = (0x00000001 shl EXTI_PR_PR12_Pos) # !< 0x00001000
  EXTI_PR_PR12* = EXTI_PR_PR12_Msk
  EXTI_PR_PR13_Pos* = (13)
  EXTI_PR_PR13_Msk* = (0x00000001 shl EXTI_PR_PR13_Pos) # !< 0x00002000
  EXTI_PR_PR13* = EXTI_PR_PR13_Msk
  EXTI_PR_PR14_Pos* = (14)
  EXTI_PR_PR14_Msk* = (0x00000001 shl EXTI_PR_PR14_Pos) # !< 0x00004000
  EXTI_PR_PR14* = EXTI_PR_PR14_Msk
  EXTI_PR_PR15_Pos* = (15)
  EXTI_PR_PR15_Msk* = (0x00000001 shl EXTI_PR_PR15_Pos) # !< 0x00008000
  EXTI_PR_PR15* = EXTI_PR_PR15_Msk
  EXTI_PR_PR16_Pos* = (16)
  EXTI_PR_PR16_Msk* = (0x00000001 shl EXTI_PR_PR16_Pos) # !< 0x00010000
  EXTI_PR_PR16* = EXTI_PR_PR16_Msk
  EXTI_PR_PR17_Pos* = (17)
  EXTI_PR_PR17_Msk* = (0x00000001 shl EXTI_PR_PR17_Pos) # !< 0x00020000
  EXTI_PR_PR17* = EXTI_PR_PR17_Msk
  EXTI_PR_PR19_Pos* = (19)
  EXTI_PR_PR19_Msk* = (0x00000001 shl EXTI_PR_PR19_Pos) # !< 0x00080000
  EXTI_PR_PR19* = EXTI_PR_PR19_Msk

#  References Defines

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
  EXTI_PR_PIF19* = EXTI_PR_PR19

# ****************************************************************************
##
#                       FLASH and Option Bytes Registers
##
# ****************************************************************************
# ******************  Bit definition for FLASH_ACR register  *****************

const
  FLASH_ACR_LATENCY_Pos* = (0)
  FLASH_ACR_LATENCY_Msk* = (0x00000001 shl FLASH_ACR_LATENCY_Pos) # !< 0x00000001
  FLASH_ACR_LATENCY* = FLASH_ACR_LATENCY_Msk
  FLASH_ACR_PRFTBE_Pos* = (4)
  FLASH_ACR_PRFTBE_Msk* = (0x00000001 shl FLASH_ACR_PRFTBE_Pos) # !< 0x00000010
  FLASH_ACR_PRFTBE* = FLASH_ACR_PRFTBE_Msk
  FLASH_ACR_PRFTBS_Pos* = (5)
  FLASH_ACR_PRFTBS_Msk* = (0x00000001 shl FLASH_ACR_PRFTBS_Pos) # !< 0x00000020
  FLASH_ACR_PRFTBS* = FLASH_ACR_PRFTBS_Msk

# *****************  Bit definition for FLASH_KEYR register  *****************

const
  FLASH_KEYR_FKEYR_Pos* = (0)
  FLASH_KEYR_FKEYR_Msk* = (0xFFFFFFFF shl FLASH_KEYR_FKEYR_Pos) # !< 0xFFFFFFFF
  FLASH_KEYR_FKEYR* = FLASH_KEYR_FKEYR_Msk

# ****************  Bit definition for FLASH_OPTKEYR register  ***************

const
  FLASH_OPTKEYR_OPTKEYR_Pos* = (0)
  FLASH_OPTKEYR_OPTKEYR_Msk* = (0xFFFFFFFF shl FLASH_OPTKEYR_OPTKEYR_Pos) # !< 0xFFFFFFFF
  FLASH_OPTKEYR_OPTKEYR* = FLASH_OPTKEYR_OPTKEYR_Msk

# *****************  FLASH Keys  *********************************************

const
  FLASH_KEY1_Pos* = (0)
  FLASH_KEY1_Msk* = (0x45670123 shl FLASH_KEY1_Pos) # !< 0x45670123
  FLASH_KEY1* = FLASH_KEY1_Msk
  FLASH_KEY2_Pos* = (0)
  FLASH_KEY2_Msk* = (0xCDEF89AB shl FLASH_KEY2_Pos) # !< 0xCDEF89AB
  FLASH_KEY2* = FLASH_KEY2_Msk
  FLASH_OPTKEY1_Pos* = (0)
  FLASH_OPTKEY1_Msk* = (0x45670123 shl FLASH_OPTKEY1_Pos) # !< 0x45670123
  FLASH_OPTKEY1* = FLASH_OPTKEY1_Msk
  FLASH_OPTKEY2_Pos* = (0)
  FLASH_OPTKEY2_Msk* = (0xCDEF89AB shl FLASH_OPTKEY2_Pos) # !< 0xCDEF89AB
  FLASH_OPTKEY2* = FLASH_OPTKEY2_Msk

# *****************  Bit definition for FLASH_SR register  ******************

const
  FLASH_SR_BSY_Pos* = (0)
  FLASH_SR_BSY_Msk* = (0x00000001 shl FLASH_SR_BSY_Pos) # !< 0x00000001
  FLASH_SR_BSY* = FLASH_SR_BSY_Msk
  FLASH_SR_PGERR_Pos* = (2)
  FLASH_SR_PGERR_Msk* = (0x00000001 shl FLASH_SR_PGERR_Pos) # !< 0x00000004
  FLASH_SR_PGERR* = FLASH_SR_PGERR_Msk
  FLASH_SR_WRPRTERR_Pos* = (4)
  FLASH_SR_WRPRTERR_Msk* = (0x00000001 shl FLASH_SR_WRPRTERR_Pos) # !< 0x00000010
  FLASH_SR_WRPRTERR* = FLASH_SR_WRPRTERR_Msk
  FLASH_SR_EOP_Pos* = (5)
  FLASH_SR_EOP_Msk* = (0x00000001 shl FLASH_SR_EOP_Pos) # !< 0x00000020
  FLASH_SR_EOP* = FLASH_SR_EOP_Msk
  FLASH_SR_WRPERR* = FLASH_SR_WRPRTERR

# ******************  Bit definition for FLASH_CR register  ******************

const
  FLASH_CR_PG_Pos* = (0)
  FLASH_CR_PG_Msk* = (0x00000001 shl FLASH_CR_PG_Pos) # !< 0x00000001
  FLASH_CR_PG* = FLASH_CR_PG_Msk
  FLASH_CR_PER_Pos* = (1)
  FLASH_CR_PER_Msk* = (0x00000001 shl FLASH_CR_PER_Pos) # !< 0x00000002
  FLASH_CR_PER* = FLASH_CR_PER_Msk
  FLASH_CR_MER_Pos* = (2)
  FLASH_CR_MER_Msk* = (0x00000001 shl FLASH_CR_MER_Pos) # !< 0x00000004
  FLASH_CR_MER* = FLASH_CR_MER_Msk
  FLASH_CR_OPTPG_Pos* = (4)
  FLASH_CR_OPTPG_Msk* = (0x00000001 shl FLASH_CR_OPTPG_Pos) # !< 0x00000010
  FLASH_CR_OPTPG* = FLASH_CR_OPTPG_Msk
  FLASH_CR_OPTER_Pos* = (5)
  FLASH_CR_OPTER_Msk* = (0x00000001 shl FLASH_CR_OPTER_Pos) # !< 0x00000020
  FLASH_CR_OPTER* = FLASH_CR_OPTER_Msk
  FLASH_CR_STRT_Pos* = (6)
  FLASH_CR_STRT_Msk* = (0x00000001 shl FLASH_CR_STRT_Pos) # !< 0x00000040
  FLASH_CR_STRT* = FLASH_CR_STRT_Msk
  FLASH_CR_LOCK_Pos* = (7)
  FLASH_CR_LOCK_Msk* = (0x00000001 shl FLASH_CR_LOCK_Pos) # !< 0x00000080
  FLASH_CR_LOCK* = FLASH_CR_LOCK_Msk
  FLASH_CR_OPTWRE_Pos* = (9)
  FLASH_CR_OPTWRE_Msk* = (0x00000001 shl FLASH_CR_OPTWRE_Pos) # !< 0x00000200
  FLASH_CR_OPTWRE* = FLASH_CR_OPTWRE_Msk
  FLASH_CR_ERRIE_Pos* = (10)
  FLASH_CR_ERRIE_Msk* = (0x00000001 shl FLASH_CR_ERRIE_Pos) # !< 0x00000400
  FLASH_CR_ERRIE* = FLASH_CR_ERRIE_Msk
  FLASH_CR_EOPIE_Pos* = (12)
  FLASH_CR_EOPIE_Msk* = (0x00000001 shl FLASH_CR_EOPIE_Pos) # !< 0x00001000
  FLASH_CR_EOPIE* = FLASH_CR_EOPIE_Msk
  FLASH_CR_OBL_LAUNCH_Pos* = (13)
  FLASH_CR_OBL_LAUNCH_Msk* = (0x00000001 shl FLASH_CR_OBL_LAUNCH_Pos) # !< 0x00002000
  FLASH_CR_OBL_LAUNCH* = FLASH_CR_OBL_LAUNCH_Msk

# ******************  Bit definition for FLASH_AR register  ******************

const
  FLASH_AR_FAR_Pos* = (0)
  FLASH_AR_FAR_Msk* = (0xFFFFFFFF shl FLASH_AR_FAR_Pos) # !< 0xFFFFFFFF
  FLASH_AR_FAR* = FLASH_AR_FAR_Msk

# *****************  Bit definition for FLASH_OBR register  ******************

const
  FLASH_OBR_OPTERR_Pos* = (0)
  FLASH_OBR_OPTERR_Msk* = (0x00000001 shl FLASH_OBR_OPTERR_Pos) # !< 0x00000001
  FLASH_OBR_OPTERR* = FLASH_OBR_OPTERR_Msk
  FLASH_OBR_RDPRT1_Pos* = (1)
  FLASH_OBR_RDPRT1_Msk* = (0x00000001 shl FLASH_OBR_RDPRT1_Pos) # !< 0x00000002
  FLASH_OBR_RDPRT1* = FLASH_OBR_RDPRT1_Msk
  FLASH_OBR_RDPRT2_Pos* = (2)
  FLASH_OBR_RDPRT2_Msk* = (0x00000001 shl FLASH_OBR_RDPRT2_Pos) # !< 0x00000004
  FLASH_OBR_RDPRT2* = FLASH_OBR_RDPRT2_Msk
  FLASH_OBR_USER_Pos* = (8)
  FLASH_OBR_USER_Msk* = (0x00000077 shl FLASH_OBR_USER_Pos) # !< 0x00007700
  FLASH_OBR_USER* = FLASH_OBR_USER_Msk
  FLASH_OBR_IWDG_SW_Pos* = (8)
  FLASH_OBR_IWDG_SW_Msk* = (0x00000001 shl FLASH_OBR_IWDG_SW_Pos) # !< 0x00000100
  FLASH_OBR_IWDG_SW* = FLASH_OBR_IWDG_SW_Msk
  FLASH_OBR_nRST_STOP_Pos* = (9)
  FLASH_OBR_nRST_STOP_Msk* = (0x00000001 shl FLASH_OBR_nRST_STOP_Pos) # !< 0x00000200
  FLASH_OBR_nRST_STOP* = FLASH_OBR_nRST_STOP_Msk
  FLASH_OBR_nRST_STDBY_Pos* = (10)
  FLASH_OBR_nRST_STDBY_Msk* = (0x00000001 shl FLASH_OBR_nRST_STDBY_Pos) # !< 0x00000400
  FLASH_OBR_nRST_STDBY* = FLASH_OBR_nRST_STDBY_Msk
  FLASH_OBR_nBOOT1_Pos* = (12)
  FLASH_OBR_nBOOT1_Msk* = (0x00000001 shl FLASH_OBR_nBOOT1_Pos) # !< 0x00001000
  FLASH_OBR_nBOOT1* = FLASH_OBR_nBOOT1_Msk
  FLASH_OBR_VDDA_MONITOR_Pos* = (13)
  FLASH_OBR_VDDA_MONITOR_Msk* = (0x00000001 shl FLASH_OBR_VDDA_MONITOR_Pos) # !< 0x00002000
  FLASH_OBR_VDDA_MONITOR* = FLASH_OBR_VDDA_MONITOR_Msk
  FLASH_OBR_RAM_PARITY_CHECK_Pos* = (14)
  FLASH_OBR_RAM_PARITY_CHECK_Msk* = (0x00000001 shl
      FLASH_OBR_RAM_PARITY_CHECK_Pos) # !< 0x00004000
  FLASH_OBR_RAM_PARITY_CHECK* = FLASH_OBR_RAM_PARITY_CHECK_Msk
  FLASH_OBR_DATA0_Pos* = (16)
  FLASH_OBR_DATA0_Msk* = (0x000000FF shl FLASH_OBR_DATA0_Pos) # !< 0x00FF0000
  FLASH_OBR_DATA0* = FLASH_OBR_DATA0_Msk
  FLASH_OBR_DATA1_Pos* = (24)
  FLASH_OBR_DATA1_Msk* = (0x000000FF shl FLASH_OBR_DATA1_Pos) # !< 0xFF000000
  FLASH_OBR_DATA1* = FLASH_OBR_DATA1_Msk

#  Old BOOT1 bit definition, maintained for legacy purpose

const
  FLASH_OBR_BOOT1* = FLASH_OBR_nBOOT1

#  Old OBR_VDDA bit definition, maintained for legacy purpose

const
  FLASH_OBR_VDDA_ANALOG* = FLASH_OBR_VDDA_MONITOR

# *****************  Bit definition for FLASH_WRPR register  *****************

const
  FLASH_WRPR_WRP_Pos* = (0)
  FLASH_WRPR_WRP_Msk* = (0x0000FFFF shl FLASH_WRPR_WRP_Pos) # !< 0x0000FFFF
  FLASH_WRPR_WRP* = FLASH_WRPR_WRP_Msk

# ----------------------------------------------------------------------------
# *****************  Bit definition for OB_RDP register  *********************

const
  OB_RDP_RDP_Pos* = (0)
  OB_RDP_RDP_Msk* = (0x000000FF shl OB_RDP_RDP_Pos) # !< 0x000000FF
  OB_RDP_RDP* = OB_RDP_RDP_Msk
  OB_RDP_nRDP_Pos* = (8)
  OB_RDP_nRDP_Msk* = (0x000000FF shl OB_RDP_nRDP_Pos) # !< 0x0000FF00
  OB_RDP_nRDP* = OB_RDP_nRDP_Msk

# *****************  Bit definition for OB_USER register  ********************

const
  OB_USER_USER_Pos* = (16)
  OB_USER_USER_Msk* = (0x000000FF shl OB_USER_USER_Pos) # !< 0x00FF0000
  OB_USER_USER* = OB_USER_USER_Msk
  OB_USER_nUSER_Pos* = (24)
  OB_USER_nUSER_Msk* = (0x000000FF shl OB_USER_nUSER_Pos) # !< 0xFF000000
  OB_USER_nUSER* = OB_USER_nUSER_Msk

# *****************  Bit definition for OB_WRP0 register  ********************

const
  OB_WRP0_WRP0_Pos* = (0)
  OB_WRP0_WRP0_Msk* = (0x000000FF shl OB_WRP0_WRP0_Pos) # !< 0x000000FF
  OB_WRP0_WRP0* = OB_WRP0_WRP0_Msk
  OB_WRP0_nWRP0_Pos* = (8)
  OB_WRP0_nWRP0_Msk* = (0x000000FF shl OB_WRP0_nWRP0_Pos) # !< 0x0000FF00
  OB_WRP0_nWRP0* = OB_WRP0_nWRP0_Msk

# *****************  Bit definition for OB_WRP1 register  ********************

const
  OB_WRP1_WRP1_Pos* = (16)
  OB_WRP1_WRP1_Msk* = (0x000000FF shl OB_WRP1_WRP1_Pos) # !< 0x00FF0000
  OB_WRP1_WRP1* = OB_WRP1_WRP1_Msk
  OB_WRP1_nWRP1_Pos* = (24)
  OB_WRP1_nWRP1_Msk* = (0x000000FF shl OB_WRP1_nWRP1_Pos) # !< 0xFF000000
  OB_WRP1_nWRP1* = OB_WRP1_nWRP1_Msk

# ****************************************************************************
##
#                        General Purpose IOs (GPIO)
##
# ****************************************************************************
# ******************  Bit definition for GPIO_MODER register  ****************

const
  GPIO_MODER_MODER0_Pos* = (0)
  GPIO_MODER_MODER0_Msk* = (0x00000003 shl GPIO_MODER_MODER0_Pos) # !< 0x00000003
  GPIO_MODER_MODER0* = GPIO_MODER_MODER0_Msk
  GPIO_MODER_MODER0_0* = (0x00000001 shl GPIO_MODER_MODER0_Pos) # !< 0x00000001
  GPIO_MODER_MODER0_1* = (0x00000002 shl GPIO_MODER_MODER0_Pos) # !< 0x00000002
  GPIO_MODER_MODER1_Pos* = (2)
  GPIO_MODER_MODER1_Msk* = (0x00000003 shl GPIO_MODER_MODER1_Pos) # !< 0x0000000C
  GPIO_MODER_MODER1* = GPIO_MODER_MODER1_Msk
  GPIO_MODER_MODER1_0* = (0x00000001 shl GPIO_MODER_MODER1_Pos) # !< 0x00000004
  GPIO_MODER_MODER1_1* = (0x00000002 shl GPIO_MODER_MODER1_Pos) # !< 0x00000008
  GPIO_MODER_MODER2_Pos* = (4)
  GPIO_MODER_MODER2_Msk* = (0x00000003 shl GPIO_MODER_MODER2_Pos) # !< 0x00000030
  GPIO_MODER_MODER2* = GPIO_MODER_MODER2_Msk
  GPIO_MODER_MODER2_0* = (0x00000001 shl GPIO_MODER_MODER2_Pos) # !< 0x00000010
  GPIO_MODER_MODER2_1* = (0x00000002 shl GPIO_MODER_MODER2_Pos) # !< 0x00000020
  GPIO_MODER_MODER3_Pos* = (6)
  GPIO_MODER_MODER3_Msk* = (0x00000003 shl GPIO_MODER_MODER3_Pos) # !< 0x000000C0
  GPIO_MODER_MODER3* = GPIO_MODER_MODER3_Msk
  GPIO_MODER_MODER3_0* = (0x00000001 shl GPIO_MODER_MODER3_Pos) # !< 0x00000040
  GPIO_MODER_MODER3_1* = (0x00000002 shl GPIO_MODER_MODER3_Pos) # !< 0x00000080
  GPIO_MODER_MODER4_Pos* = (8)
  GPIO_MODER_MODER4_Msk* = (0x00000003 shl GPIO_MODER_MODER4_Pos) # !< 0x00000300
  GPIO_MODER_MODER4* = GPIO_MODER_MODER4_Msk
  GPIO_MODER_MODER4_0* = (0x00000001 shl GPIO_MODER_MODER4_Pos) # !< 0x00000100
  GPIO_MODER_MODER4_1* = (0x00000002 shl GPIO_MODER_MODER4_Pos) # !< 0x00000200
  GPIO_MODER_MODER5_Pos* = (10)
  GPIO_MODER_MODER5_Msk* = (0x00000003 shl GPIO_MODER_MODER5_Pos) # !< 0x00000C00
  GPIO_MODER_MODER5* = GPIO_MODER_MODER5_Msk
  GPIO_MODER_MODER5_0* = (0x00000001 shl GPIO_MODER_MODER5_Pos) # !< 0x00000400
  GPIO_MODER_MODER5_1* = (0x00000002 shl GPIO_MODER_MODER5_Pos) # !< 0x00000800
  GPIO_MODER_MODER6_Pos* = (12)
  GPIO_MODER_MODER6_Msk* = (0x00000003 shl GPIO_MODER_MODER6_Pos) # !< 0x00003000
  GPIO_MODER_MODER6* = GPIO_MODER_MODER6_Msk
  GPIO_MODER_MODER6_0* = (0x00000001 shl GPIO_MODER_MODER6_Pos) # !< 0x00001000
  GPIO_MODER_MODER6_1* = (0x00000002 shl GPIO_MODER_MODER6_Pos) # !< 0x00002000
  GPIO_MODER_MODER7_Pos* = (14)
  GPIO_MODER_MODER7_Msk* = (0x00000003 shl GPIO_MODER_MODER7_Pos) # !< 0x0000C000
  GPIO_MODER_MODER7* = GPIO_MODER_MODER7_Msk
  GPIO_MODER_MODER7_0* = (0x00000001 shl GPIO_MODER_MODER7_Pos) # !< 0x00004000
  GPIO_MODER_MODER7_1* = (0x00000002 shl GPIO_MODER_MODER7_Pos) # !< 0x00008000
  GPIO_MODER_MODER8_Pos* = (16)
  GPIO_MODER_MODER8_Msk* = (0x00000003 shl GPIO_MODER_MODER8_Pos) # !< 0x00030000
  GPIO_MODER_MODER8* = GPIO_MODER_MODER8_Msk
  GPIO_MODER_MODER8_0* = (0x00000001 shl GPIO_MODER_MODER8_Pos) # !< 0x00010000
  GPIO_MODER_MODER8_1* = (0x00000002 shl GPIO_MODER_MODER8_Pos) # !< 0x00020000
  GPIO_MODER_MODER9_Pos* = (18)
  GPIO_MODER_MODER9_Msk* = (0x00000003 shl GPIO_MODER_MODER9_Pos) # !< 0x000C0000
  GPIO_MODER_MODER9* = GPIO_MODER_MODER9_Msk
  GPIO_MODER_MODER9_0* = (0x00000001 shl GPIO_MODER_MODER9_Pos) # !< 0x00040000
  GPIO_MODER_MODER9_1* = (0x00000002 shl GPIO_MODER_MODER9_Pos) # !< 0x00080000
  GPIO_MODER_MODER10_Pos* = (20)
  GPIO_MODER_MODER10_Msk* = (0x00000003 shl GPIO_MODER_MODER10_Pos) # !< 0x00300000
#GPIO_MODER_MODER10* = GPIO_MODER_MODER10_Msk
  GPIO_MODER_MODER10_0* = (0x00000001 shl GPIO_MODER_MODER10_Pos) # !< 0x00100000
  GPIO_MODER_MODER10_1* = (0x00000002 shl GPIO_MODER_MODER10_Pos) # !< 0x00200000
  GPIO_MODER_MODER11_Pos* = (22)
  GPIO_MODER_MODER11_Msk* = (0x00000003 shl GPIO_MODER_MODER11_Pos) # !< 0x00C00000
#GPIO_MODER_MODER11* = GPIO_MODER_MODER11_Msk
  GPIO_MODER_MODER11_0* = (0x00000001 shl GPIO_MODER_MODER11_Pos) # !< 0x00400000
  GPIO_MODER_MODER11_1* = (0x00000002 shl GPIO_MODER_MODER11_Pos) # !< 0x00800000
  GPIO_MODER_MODER12_Pos* = (24)
  GPIO_MODER_MODER12_Msk* = (0x00000003 shl GPIO_MODER_MODER12_Pos) # !< 0x03000000
  GPIO_MODER_MODER12* = GPIO_MODER_MODER12_Msk
  GPIO_MODER_MODER12_0* = (0x00000001 shl GPIO_MODER_MODER12_Pos) # !< 0x01000000
  GPIO_MODER_MODER12_1* = (0x00000002 shl GPIO_MODER_MODER12_Pos) # !< 0x02000000
  GPIO_MODER_MODER13_Pos* = (26)
  GPIO_MODER_MODER13_Msk* = (0x00000003 shl GPIO_MODER_MODER13_Pos) # !< 0x0C000000
  GPIO_MODER_MODER13* = GPIO_MODER_MODER13_Msk
  GPIO_MODER_MODER13_0* = (0x00000001 shl GPIO_MODER_MODER13_Pos) # !< 0x04000000
  GPIO_MODER_MODER13_1* = (0x00000002 shl GPIO_MODER_MODER13_Pos) # !< 0x08000000
  GPIO_MODER_MODER14_Pos* = (28)
  GPIO_MODER_MODER14_Msk* = (0x00000003 shl GPIO_MODER_MODER14_Pos) # !< 0x30000000
  GPIO_MODER_MODER14* = GPIO_MODER_MODER14_Msk
  GPIO_MODER_MODER14_0* = (0x00000001 shl GPIO_MODER_MODER14_Pos) # !< 0x10000000
  GPIO_MODER_MODER14_1* = (0x00000002 shl GPIO_MODER_MODER14_Pos) # !< 0x20000000
  GPIO_MODER_MODER15_Pos* = (30)
  GPIO_MODER_MODER15_Msk* = (0x00000003 shl GPIO_MODER_MODER15_Pos) # !< 0xC0000000
  GPIO_MODER_MODER15* = GPIO_MODER_MODER15_Msk
  GPIO_MODER_MODER15_0* = (0x00000001 shl GPIO_MODER_MODER15_Pos) # !< 0x40000000
  GPIO_MODER_MODER15_1* = (0x00000002 shl GPIO_MODER_MODER15_Pos) # !< 0x80000000

# *****************  Bit definition for GPIO_OTYPER register  ****************

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

# ***************  Bit definition for GPIO_OSPEEDR register  *****************

const
  GPIO_OSPEEDR_OSPEEDR0_Pos* = (0)
  GPIO_OSPEEDR_OSPEEDR0_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR0_Pos) # !< 0x00000003
  GPIO_OSPEEDR_OSPEEDR0* = GPIO_OSPEEDR_OSPEEDR0_Msk
  GPIO_OSPEEDR_OSPEEDR0_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR0_Pos) # !< 0x00000001
  GPIO_OSPEEDR_OSPEEDR0_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR0_Pos) # !< 0x00000002
  GPIO_OSPEEDR_OSPEEDR1_Pos* = (2)
  GPIO_OSPEEDR_OSPEEDR1_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR1_Pos) # !< 0x0000000C
  GPIO_OSPEEDR_OSPEEDR1* = GPIO_OSPEEDR_OSPEEDR1_Msk
  GPIO_OSPEEDR_OSPEEDR1_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR1_Pos) # !< 0x00000004
  GPIO_OSPEEDR_OSPEEDR1_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR1_Pos) # !< 0x00000008
  GPIO_OSPEEDR_OSPEEDR2_Pos* = (4)
  GPIO_OSPEEDR_OSPEEDR2_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR2_Pos) # !< 0x00000030
  GPIO_OSPEEDR_OSPEEDR2* = GPIO_OSPEEDR_OSPEEDR2_Msk
  GPIO_OSPEEDR_OSPEEDR2_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR2_Pos) # !< 0x00000010
  GPIO_OSPEEDR_OSPEEDR2_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR2_Pos) # !< 0x00000020
  GPIO_OSPEEDR_OSPEEDR3_Pos* = (6)
  GPIO_OSPEEDR_OSPEEDR3_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR3_Pos) # !< 0x000000C0
  GPIO_OSPEEDR_OSPEEDR3* = GPIO_OSPEEDR_OSPEEDR3_Msk
  GPIO_OSPEEDR_OSPEEDR3_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR3_Pos) # !< 0x00000040
  GPIO_OSPEEDR_OSPEEDR3_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR3_Pos) # !< 0x00000080
  GPIO_OSPEEDR_OSPEEDR4_Pos* = (8)
  GPIO_OSPEEDR_OSPEEDR4_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR4_Pos) # !< 0x00000300
  GPIO_OSPEEDR_OSPEEDR4* = GPIO_OSPEEDR_OSPEEDR4_Msk
  GPIO_OSPEEDR_OSPEEDR4_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR4_Pos) # !< 0x00000100
  GPIO_OSPEEDR_OSPEEDR4_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR4_Pos) # !< 0x00000200
  GPIO_OSPEEDR_OSPEEDR5_Pos* = (10)
  GPIO_OSPEEDR_OSPEEDR5_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR5_Pos) # !< 0x00000C00
  GPIO_OSPEEDR_OSPEEDR5* = GPIO_OSPEEDR_OSPEEDR5_Msk
  GPIO_OSPEEDR_OSPEEDR5_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR5_Pos) # !< 0x00000400
  GPIO_OSPEEDR_OSPEEDR5_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR5_Pos) # !< 0x00000800
  GPIO_OSPEEDR_OSPEEDR6_Pos* = (12)
  GPIO_OSPEEDR_OSPEEDR6_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR6_Pos) # !< 0x00003000
  GPIO_OSPEEDR_OSPEEDR6* = GPIO_OSPEEDR_OSPEEDR6_Msk
  GPIO_OSPEEDR_OSPEEDR6_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR6_Pos) # !< 0x00001000
  GPIO_OSPEEDR_OSPEEDR6_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR6_Pos) # !< 0x00002000
  GPIO_OSPEEDR_OSPEEDR7_Pos* = (14)
  GPIO_OSPEEDR_OSPEEDR7_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR7_Pos) # !< 0x0000C000
  GPIO_OSPEEDR_OSPEEDR7* = GPIO_OSPEEDR_OSPEEDR7_Msk
  GPIO_OSPEEDR_OSPEEDR7_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR7_Pos) # !< 0x00004000
  GPIO_OSPEEDR_OSPEEDR7_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR7_Pos) # !< 0x00008000
  GPIO_OSPEEDR_OSPEEDR8_Pos* = (16)
  GPIO_OSPEEDR_OSPEEDR8_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR8_Pos) # !< 0x00030000
  GPIO_OSPEEDR_OSPEEDR8* = GPIO_OSPEEDR_OSPEEDR8_Msk
  GPIO_OSPEEDR_OSPEEDR8_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR8_Pos) # !< 0x00010000
  GPIO_OSPEEDR_OSPEEDR8_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR8_Pos) # !< 0x00020000
  GPIO_OSPEEDR_OSPEEDR9_Pos* = (18)
  GPIO_OSPEEDR_OSPEEDR9_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR9_Pos) # !< 0x000C0000
  GPIO_OSPEEDR_OSPEEDR9* = GPIO_OSPEEDR_OSPEEDR9_Msk
  GPIO_OSPEEDR_OSPEEDR9_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR9_Pos) # !< 0x00040000
  GPIO_OSPEEDR_OSPEEDR9_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR9_Pos) # !< 0x00080000
  GPIO_OSPEEDR_OSPEEDR10_Pos* = (20)
  GPIO_OSPEEDR_OSPEEDR10_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR10_Pos) # !< 0x00300000
#GPIO_OSPEEDR_OSPEEDR10* = GPIO_OSPEEDR_OSPEEDR10_Msk
  GPIO_OSPEEDR_OSPEEDR10_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR10_Pos) # !< 0x00100000
  GPIO_OSPEEDR_OSPEEDR10_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR10_Pos) # !< 0x00200000
  GPIO_OSPEEDR_OSPEEDR11_Pos* = (22)
  GPIO_OSPEEDR_OSPEEDR11_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR11_Pos) # !< 0x00C00000
#GPIO_OSPEEDR_OSPEEDR11* = GPIO_OSPEEDR_OSPEEDR11_Msk
  GPIO_OSPEEDR_OSPEEDR11_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR11_Pos) # !< 0x00400000
  GPIO_OSPEEDR_OSPEEDR11_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR11_Pos) # !< 0x00800000
  GPIO_OSPEEDR_OSPEEDR12_Pos* = (24)
  GPIO_OSPEEDR_OSPEEDR12_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR12_Pos) # !< 0x03000000
  GPIO_OSPEEDR_OSPEEDR12* = GPIO_OSPEEDR_OSPEEDR12_Msk
  GPIO_OSPEEDR_OSPEEDR12_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR12_Pos) # !< 0x01000000
  GPIO_OSPEEDR_OSPEEDR12_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR12_Pos) # !< 0x02000000
  GPIO_OSPEEDR_OSPEEDR13_Pos* = (26)
  GPIO_OSPEEDR_OSPEEDR13_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR13_Pos) # !< 0x0C000000
  GPIO_OSPEEDR_OSPEEDR13* = GPIO_OSPEEDR_OSPEEDR13_Msk
  GPIO_OSPEEDR_OSPEEDR13_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR13_Pos) # !< 0x04000000
  GPIO_OSPEEDR_OSPEEDR13_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR13_Pos) # !< 0x08000000
  GPIO_OSPEEDR_OSPEEDR14_Pos* = (28)
  GPIO_OSPEEDR_OSPEEDR14_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR14_Pos) # !< 0x30000000
  GPIO_OSPEEDR_OSPEEDR14* = GPIO_OSPEEDR_OSPEEDR14_Msk
  GPIO_OSPEEDR_OSPEEDR14_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR14_Pos) # !< 0x10000000
  GPIO_OSPEEDR_OSPEEDR14_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR14_Pos) # !< 0x20000000
  GPIO_OSPEEDR_OSPEEDR15_Pos* = (30)
  GPIO_OSPEEDR_OSPEEDR15_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEEDR15_Pos) # !< 0xC0000000
  GPIO_OSPEEDR_OSPEEDR15* = GPIO_OSPEEDR_OSPEEDR15_Msk
  GPIO_OSPEEDR_OSPEEDR15_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEEDR15_Pos) # !< 0x40000000
  GPIO_OSPEEDR_OSPEEDR15_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEEDR15_Pos) # !< 0x80000000

#  Old Bit definition for GPIO_OSPEEDR register maintained for legacy purpose

const
  GPIO_OSPEEDER_OSPEEDR0* = GPIO_OSPEEDR_OSPEEDR0
  GPIO_OSPEEDER_OSPEEDR0_0* = GPIO_OSPEEDR_OSPEEDR0_0
  GPIO_OSPEEDER_OSPEEDR0_1* = GPIO_OSPEEDR_OSPEEDR0_1
  GPIO_OSPEEDER_OSPEEDR1* = GPIO_OSPEEDR_OSPEEDR1
  GPIO_OSPEEDER_OSPEEDR1_0* = GPIO_OSPEEDR_OSPEEDR1_0
  GPIO_OSPEEDER_OSPEEDR1_1* = GPIO_OSPEEDR_OSPEEDR1_1
  GPIO_OSPEEDER_OSPEEDR2* = GPIO_OSPEEDR_OSPEEDR2
  GPIO_OSPEEDER_OSPEEDR2_0* = GPIO_OSPEEDR_OSPEEDR2_0
  GPIO_OSPEEDER_OSPEEDR2_1* = GPIO_OSPEEDR_OSPEEDR2_1
  GPIO_OSPEEDER_OSPEEDR3* = GPIO_OSPEEDR_OSPEEDR3
  GPIO_OSPEEDER_OSPEEDR3_0* = GPIO_OSPEEDR_OSPEEDR3_0
  GPIO_OSPEEDER_OSPEEDR3_1* = GPIO_OSPEEDR_OSPEEDR3_1
  GPIO_OSPEEDER_OSPEEDR4* = GPIO_OSPEEDR_OSPEEDR4
  GPIO_OSPEEDER_OSPEEDR4_0* = GPIO_OSPEEDR_OSPEEDR4_0
  GPIO_OSPEEDER_OSPEEDR4_1* = GPIO_OSPEEDR_OSPEEDR4_1
  GPIO_OSPEEDER_OSPEEDR5* = GPIO_OSPEEDR_OSPEEDR5
  GPIO_OSPEEDER_OSPEEDR5_0* = GPIO_OSPEEDR_OSPEEDR5_0
  GPIO_OSPEEDER_OSPEEDR5_1* = GPIO_OSPEEDR_OSPEEDR5_1
  GPIO_OSPEEDER_OSPEEDR6* = GPIO_OSPEEDR_OSPEEDR6
  GPIO_OSPEEDER_OSPEEDR6_0* = GPIO_OSPEEDR_OSPEEDR6_0
  GPIO_OSPEEDER_OSPEEDR6_1* = GPIO_OSPEEDR_OSPEEDR6_1
  GPIO_OSPEEDER_OSPEEDR7* = GPIO_OSPEEDR_OSPEEDR7
  GPIO_OSPEEDER_OSPEEDR7_0* = GPIO_OSPEEDR_OSPEEDR7_0
  GPIO_OSPEEDER_OSPEEDR7_1* = GPIO_OSPEEDR_OSPEEDR7_1
  GPIO_OSPEEDER_OSPEEDR8* = GPIO_OSPEEDR_OSPEEDR8
  GPIO_OSPEEDER_OSPEEDR8_0* = GPIO_OSPEEDR_OSPEEDR8_0
  GPIO_OSPEEDER_OSPEEDR8_1* = GPIO_OSPEEDR_OSPEEDR8_1
  GPIO_OSPEEDER_OSPEEDR9* = GPIO_OSPEEDR_OSPEEDR9
  GPIO_OSPEEDER_OSPEEDR9_0* = GPIO_OSPEEDR_OSPEEDR9_0
  GPIO_OSPEEDER_OSPEEDR9_1* = GPIO_OSPEEDR_OSPEEDR9_1
#GPIO_OSPEEDER_OSPEEDR10* = GPIO_OSPEEDR_OSPEEDR10
  GPIO_OSPEEDER_OSPEEDR10_0* = GPIO_OSPEEDR_OSPEEDR10_0
  GPIO_OSPEEDER_OSPEEDR10_1* = GPIO_OSPEEDR_OSPEEDR10_1
#GPIO_OSPEEDER_OSPEEDR11* = GPIO_OSPEEDR_OSPEEDR11
  GPIO_OSPEEDER_OSPEEDR11_0* = GPIO_OSPEEDR_OSPEEDR11_0
  GPIO_OSPEEDER_OSPEEDR11_1* = GPIO_OSPEEDR_OSPEEDR11_1
  GPIO_OSPEEDER_OSPEEDR12* = GPIO_OSPEEDR_OSPEEDR12
  GPIO_OSPEEDER_OSPEEDR12_0* = GPIO_OSPEEDR_OSPEEDR12_0
  GPIO_OSPEEDER_OSPEEDR12_1* = GPIO_OSPEEDR_OSPEEDR12_1
  GPIO_OSPEEDER_OSPEEDR13* = GPIO_OSPEEDR_OSPEEDR13
  GPIO_OSPEEDER_OSPEEDR13_0* = GPIO_OSPEEDR_OSPEEDR13_0
  GPIO_OSPEEDER_OSPEEDR13_1* = GPIO_OSPEEDR_OSPEEDR13_1
  GPIO_OSPEEDER_OSPEEDR14* = GPIO_OSPEEDR_OSPEEDR14
  GPIO_OSPEEDER_OSPEEDR14_0* = GPIO_OSPEEDR_OSPEEDR14_0
  GPIO_OSPEEDER_OSPEEDR14_1* = GPIO_OSPEEDR_OSPEEDR14_1
  GPIO_OSPEEDER_OSPEEDR15* = GPIO_OSPEEDR_OSPEEDR15
  GPIO_OSPEEDER_OSPEEDR15_0* = GPIO_OSPEEDR_OSPEEDR15_0
  GPIO_OSPEEDER_OSPEEDR15_1* = GPIO_OSPEEDR_OSPEEDR15_1

# ******************  Bit definition for GPIO_PUPDR register *****************

const
  GPIO_PUPDR_PUPDR0_Pos* = (0)
  GPIO_PUPDR_PUPDR0_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR0_Pos) # !< 0x00000003
  GPIO_PUPDR_PUPDR0* = GPIO_PUPDR_PUPDR0_Msk
  GPIO_PUPDR_PUPDR0_0* = (0x00000001 shl GPIO_PUPDR_PUPDR0_Pos) # !< 0x00000001
  GPIO_PUPDR_PUPDR0_1* = (0x00000002 shl GPIO_PUPDR_PUPDR0_Pos) # !< 0x00000002
  GPIO_PUPDR_PUPDR1_Pos* = (2)
  GPIO_PUPDR_PUPDR1_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR1_Pos) # !< 0x0000000C
  GPIO_PUPDR_PUPDR1* = GPIO_PUPDR_PUPDR1_Msk
  GPIO_PUPDR_PUPDR1_0* = (0x00000001 shl GPIO_PUPDR_PUPDR1_Pos) # !< 0x00000004
  GPIO_PUPDR_PUPDR1_1* = (0x00000002 shl GPIO_PUPDR_PUPDR1_Pos) # !< 0x00000008
  GPIO_PUPDR_PUPDR2_Pos* = (4)
  GPIO_PUPDR_PUPDR2_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR2_Pos) # !< 0x00000030
  GPIO_PUPDR_PUPDR2* = GPIO_PUPDR_PUPDR2_Msk
  GPIO_PUPDR_PUPDR2_0* = (0x00000001 shl GPIO_PUPDR_PUPDR2_Pos) # !< 0x00000010
  GPIO_PUPDR_PUPDR2_1* = (0x00000002 shl GPIO_PUPDR_PUPDR2_Pos) # !< 0x00000020
  GPIO_PUPDR_PUPDR3_Pos* = (6)
  GPIO_PUPDR_PUPDR3_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR3_Pos) # !< 0x000000C0
  GPIO_PUPDR_PUPDR3* = GPIO_PUPDR_PUPDR3_Msk
  GPIO_PUPDR_PUPDR3_0* = (0x00000001 shl GPIO_PUPDR_PUPDR3_Pos) # !< 0x00000040
  GPIO_PUPDR_PUPDR3_1* = (0x00000002 shl GPIO_PUPDR_PUPDR3_Pos) # !< 0x00000080
  GPIO_PUPDR_PUPDR4_Pos* = (8)
  GPIO_PUPDR_PUPDR4_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR4_Pos) # !< 0x00000300
  GPIO_PUPDR_PUPDR4* = GPIO_PUPDR_PUPDR4_Msk
  GPIO_PUPDR_PUPDR4_0* = (0x00000001 shl GPIO_PUPDR_PUPDR4_Pos) # !< 0x00000100
  GPIO_PUPDR_PUPDR4_1* = (0x00000002 shl GPIO_PUPDR_PUPDR4_Pos) # !< 0x00000200
  GPIO_PUPDR_PUPDR5_Pos* = (10)
  GPIO_PUPDR_PUPDR5_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR5_Pos) # !< 0x00000C00
  GPIO_PUPDR_PUPDR5* = GPIO_PUPDR_PUPDR5_Msk
  GPIO_PUPDR_PUPDR5_0* = (0x00000001 shl GPIO_PUPDR_PUPDR5_Pos) # !< 0x00000400
  GPIO_PUPDR_PUPDR5_1* = (0x00000002 shl GPIO_PUPDR_PUPDR5_Pos) # !< 0x00000800
  GPIO_PUPDR_PUPDR6_Pos* = (12)
  GPIO_PUPDR_PUPDR6_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR6_Pos) # !< 0x00003000
  GPIO_PUPDR_PUPDR6* = GPIO_PUPDR_PUPDR6_Msk
  GPIO_PUPDR_PUPDR6_0* = (0x00000001 shl GPIO_PUPDR_PUPDR6_Pos) # !< 0x00001000
  GPIO_PUPDR_PUPDR6_1* = (0x00000002 shl GPIO_PUPDR_PUPDR6_Pos) # !< 0x00002000
  GPIO_PUPDR_PUPDR7_Pos* = (14)
  GPIO_PUPDR_PUPDR7_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR7_Pos) # !< 0x0000C000
  GPIO_PUPDR_PUPDR7* = GPIO_PUPDR_PUPDR7_Msk
  GPIO_PUPDR_PUPDR7_0* = (0x00000001 shl GPIO_PUPDR_PUPDR7_Pos) # !< 0x00004000
  GPIO_PUPDR_PUPDR7_1* = (0x00000002 shl GPIO_PUPDR_PUPDR7_Pos) # !< 0x00008000
  GPIO_PUPDR_PUPDR8_Pos* = (16)
  GPIO_PUPDR_PUPDR8_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR8_Pos) # !< 0x00030000
  GPIO_PUPDR_PUPDR8* = GPIO_PUPDR_PUPDR8_Msk
  GPIO_PUPDR_PUPDR8_0* = (0x00000001 shl GPIO_PUPDR_PUPDR8_Pos) # !< 0x00010000
  GPIO_PUPDR_PUPDR8_1* = (0x00000002 shl GPIO_PUPDR_PUPDR8_Pos) # !< 0x00020000
  GPIO_PUPDR_PUPDR9_Pos* = (18)
  GPIO_PUPDR_PUPDR9_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR9_Pos) # !< 0x000C0000
  GPIO_PUPDR_PUPDR9* = GPIO_PUPDR_PUPDR9_Msk
  GPIO_PUPDR_PUPDR9_0* = (0x00000001 shl GPIO_PUPDR_PUPDR9_Pos) # !< 0x00040000
  GPIO_PUPDR_PUPDR9_1* = (0x00000002 shl GPIO_PUPDR_PUPDR9_Pos) # !< 0x00080000
  GPIO_PUPDR_PUPDR10_Pos* = (20)
  GPIO_PUPDR_PUPDR10_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR10_Pos) # !< 0x00300000
#GPIO_PUPDR_PUPDR10* = GPIO_PUPDR_PUPDR10_Msk
  GPIO_PUPDR_PUPDR10_0* = (0x00000001 shl GPIO_PUPDR_PUPDR10_Pos) # !< 0x00100000
  GPIO_PUPDR_PUPDR10_1* = (0x00000002 shl GPIO_PUPDR_PUPDR10_Pos) # !< 0x00200000
  GPIO_PUPDR_PUPDR11_Pos* = (22)
  GPIO_PUPDR_PUPDR11_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR11_Pos) # !< 0x00C00000
  #GPIO_PUPDR_PUPDR11* = GPIO_PUPDR_PUPDR11_Msk
  GPIO_PUPDR_PUPDR11_0* = (0x00000001 shl GPIO_PUPDR_PUPDR11_Pos) # !< 0x00400000
  GPIO_PUPDR_PUPDR11_1* = (0x00000002 shl GPIO_PUPDR_PUPDR11_Pos) # !< 0x00800000
  GPIO_PUPDR_PUPDR12_Pos* = (24)
  GPIO_PUPDR_PUPDR12_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR12_Pos) # !< 0x03000000
  GPIO_PUPDR_PUPDR12* = GPIO_PUPDR_PUPDR12_Msk
  GPIO_PUPDR_PUPDR12_0* = (0x00000001 shl GPIO_PUPDR_PUPDR12_Pos) # !< 0x01000000
  GPIO_PUPDR_PUPDR12_1* = (0x00000002 shl GPIO_PUPDR_PUPDR12_Pos) # !< 0x02000000
  GPIO_PUPDR_PUPDR13_Pos* = (26)
  GPIO_PUPDR_PUPDR13_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR13_Pos) # !< 0x0C000000
  GPIO_PUPDR_PUPDR13* = GPIO_PUPDR_PUPDR13_Msk
  GPIO_PUPDR_PUPDR13_0* = (0x00000001 shl GPIO_PUPDR_PUPDR13_Pos) # !< 0x04000000
  GPIO_PUPDR_PUPDR13_1* = (0x00000002 shl GPIO_PUPDR_PUPDR13_Pos) # !< 0x08000000
  GPIO_PUPDR_PUPDR14_Pos* = (28)
  GPIO_PUPDR_PUPDR14_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR14_Pos) # !< 0x30000000
#GPIO_PUPDR_PUPDR14* = GPIO_PUPDR_PUPDR14_Msk
  GPIO_PUPDR_PUPDR14_0* = (0x00000001 shl GPIO_PUPDR_PUPDR14_Pos) # !< 0x10000000
  GPIO_PUPDR_PUPDR14_1* = (0x00000002 shl GPIO_PUPDR_PUPDR14_Pos) # !< 0x20000000
  GPIO_PUPDR_PUPDR15_Pos* = (30)
  GPIO_PUPDR_PUPDR15_Msk* = (0x00000003 shl GPIO_PUPDR_PUPDR15_Pos) # !< 0xC0000000
  GPIO_PUPDR_PUPDR15* = GPIO_PUPDR_PUPDR15_Msk
  GPIO_PUPDR_PUPDR15_0* = (0x00000001 shl GPIO_PUPDR_PUPDR15_Pos) # !< 0x40000000
  GPIO_PUPDR_PUPDR15_1* = (0x00000002 shl GPIO_PUPDR_PUPDR15_Pos) # !< 0x80000000

# ******************  Bit definition for GPIO_IDR register  ******************

const
  GPIO_IDR_0* = (0x00000001)
  GPIO_IDR_1* = (0x00000002)
  GPIO_IDR_2* = (0x00000004)
  GPIO_IDR_3* = (0x00000008)
  GPIO_IDR_4* = (0x00000010)
  GPIO_IDR_5* = (0x00000020)
  GPIO_IDR_6* = (0x00000040)
  GPIO_IDR_7* = (0x00000080)
  GPIO_IDR_8* = (0x00000100)
  GPIO_IDR_9* = (0x00000200)
  GPIO_IDR_10* = (0x00000400)
  GPIO_IDR_11* = (0x00000800)
  GPIO_IDR_12* = (0x00001000)
  GPIO_IDR_13* = (0x00002000)
  GPIO_IDR_14* = (0x00004000)
  GPIO_IDR_15* = (0x00008000)

# *****************  Bit definition for GPIO_ODR register  *******************

const
  GPIO_ODR_0* = (0x00000001)
  GPIO_ODR_1* = (0x00000002)
  GPIO_ODR_2* = (0x00000004)
  GPIO_ODR_3* = (0x00000008)
  GPIO_ODR_4* = (0x00000010)
  GPIO_ODR_5* = (0x00000020)
  GPIO_ODR_6* = (0x00000040)
  GPIO_ODR_7* = (0x00000080)
  GPIO_ODR_8* = (0x00000100)
  GPIO_ODR_9* = (0x00000200)
  GPIO_ODR_10* = (0x00000400)
  GPIO_ODR_11* = (0x00000800)
  GPIO_ODR_12* = (0x00001000)
  GPIO_ODR_13* = (0x00002000)
  GPIO_ODR_14* = (0x00004000)
  GPIO_ODR_15* = (0x00008000)

# ***************** Bit definition for GPIO_BSRR register  *******************

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

# ***************** Bit definition for GPIO_LCKR register  *******************

const
  GPIO_LCKR_LCK0_Pos* = (0)
  GPIO_LCKR_LCK0_Msk* = (0x00000001 shl GPIO_LCKR_LCK0_Pos) # !< 0x00000001
  GPIO_LCKR_LCK0* = GPIO_LCKR_LCK0_Msk
  GPIO_LCKR_LCK1_Pos* = (1)
  GPIO_LCKR_LCK1_Msk* = (0x00000001 shl GPIO_LCKR_LCK1_Pos) # !< 0x00000002
  GPIO_LCKR_LCK1* = GPIO_LCKR_LCK1_Msk
  GPIO_LCKR_LCK2_Pos* = (2)
  GPIO_LCKR_LCK2_Msk* = (0x00000001 shl GPIO_LCKR_LCK2_Pos) # !< 0x00000004
  GPIO_LCKR_LCK2* = GPIO_LCKR_LCK2_Msk
  GPIO_LCKR_LCK3_Pos* = (3)
  GPIO_LCKR_LCK3_Msk* = (0x00000001 shl GPIO_LCKR_LCK3_Pos) # !< 0x00000008
  GPIO_LCKR_LCK3* = GPIO_LCKR_LCK3_Msk
  GPIO_LCKR_LCK4_Pos* = (4)
  GPIO_LCKR_LCK4_Msk* = (0x00000001 shl GPIO_LCKR_LCK4_Pos) # !< 0x00000010
  GPIO_LCKR_LCK4* = GPIO_LCKR_LCK4_Msk
  GPIO_LCKR_LCK5_Pos* = (5)
  GPIO_LCKR_LCK5_Msk* = (0x00000001 shl GPIO_LCKR_LCK5_Pos) # !< 0x00000020
  GPIO_LCKR_LCK5* = GPIO_LCKR_LCK5_Msk
  GPIO_LCKR_LCK6_Pos* = (6)
  GPIO_LCKR_LCK6_Msk* = (0x00000001 shl GPIO_LCKR_LCK6_Pos) # !< 0x00000040
  GPIO_LCKR_LCK6* = GPIO_LCKR_LCK6_Msk
  GPIO_LCKR_LCK7_Pos* = (7)
  GPIO_LCKR_LCK7_Msk* = (0x00000001 shl GPIO_LCKR_LCK7_Pos) # !< 0x00000080
  GPIO_LCKR_LCK7* = GPIO_LCKR_LCK7_Msk
  GPIO_LCKR_LCK8_Pos* = (8)
  GPIO_LCKR_LCK8_Msk* = (0x00000001 shl GPIO_LCKR_LCK8_Pos) # !< 0x00000100
  GPIO_LCKR_LCK8* = GPIO_LCKR_LCK8_Msk
  GPIO_LCKR_LCK9_Pos* = (9)
  GPIO_LCKR_LCK9_Msk* = (0x00000001 shl GPIO_LCKR_LCK9_Pos) # !< 0x00000200
  GPIO_LCKR_LCK9* = GPIO_LCKR_LCK9_Msk
  GPIO_LCKR_LCK10_Pos* = (10)
  GPIO_LCKR_LCK10_Msk* = (0x00000001 shl GPIO_LCKR_LCK10_Pos) # !< 0x00000400
  GPIO_LCKR_LCK10* = GPIO_LCKR_LCK10_Msk
  GPIO_LCKR_LCK11_Pos* = (11)
  GPIO_LCKR_LCK11_Msk* = (0x00000001 shl GPIO_LCKR_LCK11_Pos) # !< 0x00000800
  GPIO_LCKR_LCK11* = GPIO_LCKR_LCK11_Msk
  GPIO_LCKR_LCK12_Pos* = (12)
  GPIO_LCKR_LCK12_Msk* = (0x00000001 shl GPIO_LCKR_LCK12_Pos) # !< 0x00001000
  GPIO_LCKR_LCK12* = GPIO_LCKR_LCK12_Msk
  GPIO_LCKR_LCK13_Pos* = (13)
  GPIO_LCKR_LCK13_Msk* = (0x00000001 shl GPIO_LCKR_LCK13_Pos) # !< 0x00002000
  GPIO_LCKR_LCK13* = GPIO_LCKR_LCK13_Msk
  GPIO_LCKR_LCK14_Pos* = (14)
  GPIO_LCKR_LCK14_Msk* = (0x00000001 shl GPIO_LCKR_LCK14_Pos) # !< 0x00004000
  GPIO_LCKR_LCK14* = GPIO_LCKR_LCK14_Msk
  GPIO_LCKR_LCK15_Pos* = (15)
  GPIO_LCKR_LCK15_Msk* = (0x00000001 shl GPIO_LCKR_LCK15_Pos) # !< 0x00008000
  GPIO_LCKR_LCK15* = GPIO_LCKR_LCK15_Msk
  GPIO_LCKR_LCKK_Pos* = (16)
  GPIO_LCKR_LCKK_Msk* = (0x00000001 shl GPIO_LCKR_LCKK_Pos) # !< 0x00010000
  GPIO_LCKR_LCKK* = GPIO_LCKR_LCKK_Msk

# ***************** Bit definition for GPIO_AFRL register  *******************

const
  GPIO_AFRL_AFSEL0_Pos* = (0)
  GPIO_AFRL_AFSEL0_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL0_Pos) # !< 0x0000000F
  GPIO_AFRL_AFSEL0* = GPIO_AFRL_AFSEL0_Msk
  GPIO_AFRL_AFSEL1_Pos* = (4)
  GPIO_AFRL_AFSEL1_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL1_Pos) # !< 0x000000F0
  GPIO_AFRL_AFSEL1* = GPIO_AFRL_AFSEL1_Msk
  GPIO_AFRL_AFSEL2_Pos* = (8)
  GPIO_AFRL_AFSEL2_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL2_Pos) # !< 0x00000F00
  GPIO_AFRL_AFSEL2* = GPIO_AFRL_AFSEL2_Msk
  GPIO_AFRL_AFSEL3_Pos* = (12)
  GPIO_AFRL_AFSEL3_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL3_Pos) # !< 0x0000F000
  GPIO_AFRL_AFSEL3* = GPIO_AFRL_AFSEL3_Msk
  GPIO_AFRL_AFSEL4_Pos* = (16)
  GPIO_AFRL_AFSEL4_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL4_Pos) # !< 0x000F0000
  GPIO_AFRL_AFSEL4* = GPIO_AFRL_AFSEL4_Msk
  GPIO_AFRL_AFSEL5_Pos* = (20)
  GPIO_AFRL_AFSEL5_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL5_Pos) # !< 0x00F00000
  GPIO_AFRL_AFSEL5* = GPIO_AFRL_AFSEL5_Msk
  GPIO_AFRL_AFSEL6_Pos* = (24)
  GPIO_AFRL_AFSEL6_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL6_Pos) # !< 0x0F000000
  GPIO_AFRL_AFSEL6* = GPIO_AFRL_AFSEL6_Msk
  GPIO_AFRL_AFSEL7_Pos* = (28)
  GPIO_AFRL_AFSEL7_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL7_Pos) # !< 0xF0000000
  GPIO_AFRL_AFSEL7* = GPIO_AFRL_AFSEL7_Msk

#  Legacy aliases

const
  GPIO_AFRL_AFRL0_Pos* = GPIO_AFRL_AFSEL0_Pos
  GPIO_AFRL_AFRL0_Msk* = GPIO_AFRL_AFSEL0_Msk
  GPIO_AFRL_AFRL0* = GPIO_AFRL_AFSEL0
  GPIO_AFRL_AFRL1_Pos* = GPIO_AFRL_AFSEL1_Pos
  GPIO_AFRL_AFRL1_Msk* = GPIO_AFRL_AFSEL1_Msk
  GPIO_AFRL_AFRL1* = GPIO_AFRL_AFSEL1
  GPIO_AFRL_AFRL2_Pos* = GPIO_AFRL_AFSEL2_Pos
  GPIO_AFRL_AFRL2_Msk* = GPIO_AFRL_AFSEL2_Msk
  GPIO_AFRL_AFRL2* = GPIO_AFRL_AFSEL2
  GPIO_AFRL_AFRL3_Pos* = GPIO_AFRL_AFSEL3_Pos
  GPIO_AFRL_AFRL3_Msk* = GPIO_AFRL_AFSEL3_Msk
  GPIO_AFRL_AFRL3* = GPIO_AFRL_AFSEL3
  GPIO_AFRL_AFRL4_Pos* = GPIO_AFRL_AFSEL4_Pos
  GPIO_AFRL_AFRL4_Msk* = GPIO_AFRL_AFSEL4_Msk
  GPIO_AFRL_AFRL4* = GPIO_AFRL_AFSEL4
  GPIO_AFRL_AFRL5_Pos* = GPIO_AFRL_AFSEL5_Pos
  GPIO_AFRL_AFRL5_Msk* = GPIO_AFRL_AFSEL5_Msk
  GPIO_AFRL_AFRL5* = GPIO_AFRL_AFSEL5
  GPIO_AFRL_AFRL6_Pos* = GPIO_AFRL_AFSEL6_Pos
  GPIO_AFRL_AFRL6_Msk* = GPIO_AFRL_AFSEL6_Msk
  GPIO_AFRL_AFRL6* = GPIO_AFRL_AFSEL6
  GPIO_AFRL_AFRL7_Pos* = GPIO_AFRL_AFSEL7_Pos
  GPIO_AFRL_AFRL7_Msk* = GPIO_AFRL_AFSEL7_Msk
  GPIO_AFRL_AFRL7* = GPIO_AFRL_AFSEL7

# ***************** Bit definition for GPIO_AFRH register  *******************

const
  GPIO_AFRH_AFSEL8_Pos* = (0)
  GPIO_AFRH_AFSEL8_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL8_Pos) # !< 0x0000000F
  GPIO_AFRH_AFSEL8* = GPIO_AFRH_AFSEL8_Msk
  GPIO_AFRH_AFSEL9_Pos* = (4)
  GPIO_AFRH_AFSEL9_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL9_Pos) # !< 0x000000F0
  GPIO_AFRH_AFSEL9* = GPIO_AFRH_AFSEL9_Msk
  GPIO_AFRH_AFSEL10_Pos* = (8)
  GPIO_AFRH_AFSEL10_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL10_Pos) # !< 0x00000F00
  GPIO_AFRH_AFSEL10* = GPIO_AFRH_AFSEL10_Msk
  GPIO_AFRH_AFSEL11_Pos* = (12)
  GPIO_AFRH_AFSEL11_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL11_Pos) # !< 0x0000F000
  GPIO_AFRH_AFSEL11* = GPIO_AFRH_AFSEL11_Msk
  GPIO_AFRH_AFSEL12_Pos* = (16)
  GPIO_AFRH_AFSEL12_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL12_Pos) # !< 0x000F0000
  GPIO_AFRH_AFSEL12* = GPIO_AFRH_AFSEL12_Msk
  GPIO_AFRH_AFSEL13_Pos* = (20)
  GPIO_AFRH_AFSEL13_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL13_Pos) # !< 0x00F00000
  GPIO_AFRH_AFSEL13* = GPIO_AFRH_AFSEL13_Msk
  GPIO_AFRH_AFSEL14_Pos* = (24)
  GPIO_AFRH_AFSEL14_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL14_Pos) # !< 0x0F000000
  GPIO_AFRH_AFSEL14* = GPIO_AFRH_AFSEL14_Msk
  GPIO_AFRH_AFSEL15_Pos* = (28)
  GPIO_AFRH_AFSEL15_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL15_Pos) # !< 0xF0000000
  GPIO_AFRH_AFSEL15* = GPIO_AFRH_AFSEL15_Msk

#  Legacy aliases

const
  GPIO_AFRH_AFRH0_Pos* = GPIO_AFRH_AFSEL8_Pos
  GPIO_AFRH_AFRH0_Msk* = GPIO_AFRH_AFSEL8_Msk
  GPIO_AFRH_AFRH0* = GPIO_AFRH_AFSEL8
  GPIO_AFRH_AFRH1_Pos* = GPIO_AFRH_AFSEL9_Pos
  GPIO_AFRH_AFRH1_Msk* = GPIO_AFRH_AFSEL9_Msk
  GPIO_AFRH_AFRH1* = GPIO_AFRH_AFSEL9
  GPIO_AFRH_AFRH2_Pos* = GPIO_AFRH_AFSEL10_Pos
  GPIO_AFRH_AFRH2_Msk* = GPIO_AFRH_AFSEL10_Msk
  GPIO_AFRH_AFRH2* = GPIO_AFRH_AFSEL10
  GPIO_AFRH_AFRH3_Pos* = GPIO_AFRH_AFSEL11_Pos
  GPIO_AFRH_AFRH3_Msk* = GPIO_AFRH_AFSEL11_Msk
  GPIO_AFRH_AFRH3* = GPIO_AFRH_AFSEL11
  GPIO_AFRH_AFRH4_Pos* = GPIO_AFRH_AFSEL12_Pos
  GPIO_AFRH_AFRH4_Msk* = GPIO_AFRH_AFSEL12_Msk
  GPIO_AFRH_AFRH4* = GPIO_AFRH_AFSEL12
  GPIO_AFRH_AFRH5_Pos* = GPIO_AFRH_AFSEL13_Pos
  GPIO_AFRH_AFRH5_Msk* = GPIO_AFRH_AFSEL13_Msk
  GPIO_AFRH_AFRH5* = GPIO_AFRH_AFSEL13
  GPIO_AFRH_AFRH6_Pos* = GPIO_AFRH_AFSEL14_Pos
  GPIO_AFRH_AFRH6_Msk* = GPIO_AFRH_AFSEL14_Msk
  GPIO_AFRH_AFRH6* = GPIO_AFRH_AFSEL14
  GPIO_AFRH_AFRH7_Pos* = GPIO_AFRH_AFSEL15_Pos
  GPIO_AFRH_AFRH7_Msk* = GPIO_AFRH_AFSEL15_Msk
  GPIO_AFRH_AFRH7* = GPIO_AFRH_AFSEL15

# ***************** Bit definition for GPIO_BRR register  ********************

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

# ****************************************************************************
##
#                    Inter-integrated Circuit Interface (I2C)
##
# ****************************************************************************
# ******************  Bit definition for I2C_CR1 register  ******************

const
  I2C_CR1_PE_Pos* = (0)
  I2C_CR1_PE_Msk* = (0x00000001 shl I2C_CR1_PE_Pos) # !< 0x00000001
  I2C_CR1_PE* = I2C_CR1_PE_Msk
  I2C_CR1_TXIE_Pos* = (1)
  I2C_CR1_TXIE_Msk* = (0x00000001 shl I2C_CR1_TXIE_Pos) # !< 0x00000002
  I2C_CR1_TXIE* = I2C_CR1_TXIE_Msk
  I2C_CR1_RXIE_Pos* = (2)
  I2C_CR1_RXIE_Msk* = (0x00000001 shl I2C_CR1_RXIE_Pos) # !< 0x00000004
  I2C_CR1_RXIE* = I2C_CR1_RXIE_Msk
  I2C_CR1_ADDRIE_Pos* = (3)
  I2C_CR1_ADDRIE_Msk* = (0x00000001 shl I2C_CR1_ADDRIE_Pos) # !< 0x00000008
  I2C_CR1_ADDRIE* = I2C_CR1_ADDRIE_Msk
  I2C_CR1_NACKIE_Pos* = (4)
  I2C_CR1_NACKIE_Msk* = (0x00000001 shl I2C_CR1_NACKIE_Pos) # !< 0x00000010
  I2C_CR1_NACKIE* = I2C_CR1_NACKIE_Msk
  I2C_CR1_STOPIE_Pos* = (5)
  I2C_CR1_STOPIE_Msk* = (0x00000001 shl I2C_CR1_STOPIE_Pos) # !< 0x00000020
  I2C_CR1_STOPIE* = I2C_CR1_STOPIE_Msk
  I2C_CR1_TCIE_Pos* = (6)
  I2C_CR1_TCIE_Msk* = (0x00000001 shl I2C_CR1_TCIE_Pos) # !< 0x00000040
  I2C_CR1_TCIE* = I2C_CR1_TCIE_Msk
  I2C_CR1_ERRIE_Pos* = (7)
  I2C_CR1_ERRIE_Msk* = (0x00000001 shl I2C_CR1_ERRIE_Pos) # !< 0x00000080
  I2C_CR1_ERRIE* = I2C_CR1_ERRIE_Msk
  I2C_CR1_DNF_Pos* = (8)
  I2C_CR1_DNF_Msk* = (0x0000000F shl I2C_CR1_DNF_Pos) # !< 0x00000F00
  I2C_CR1_DNF* = I2C_CR1_DNF_Msk
  I2C_CR1_ANFOFF_Pos* = (12)
  I2C_CR1_ANFOFF_Msk* = (0x00000001 shl I2C_CR1_ANFOFF_Pos) # !< 0x00001000
  I2C_CR1_ANFOFF* = I2C_CR1_ANFOFF_Msk
  I2C_CR1_SWRST_Pos* = (13)
  I2C_CR1_SWRST_Msk* = (0x00000001 shl I2C_CR1_SWRST_Pos) # !< 0x00002000
  I2C_CR1_SWRST* = I2C_CR1_SWRST_Msk
  I2C_CR1_TXDMAEN_Pos* = (14)
  I2C_CR1_TXDMAEN_Msk* = (0x00000001 shl I2C_CR1_TXDMAEN_Pos) # !< 0x00004000
  I2C_CR1_TXDMAEN* = I2C_CR1_TXDMAEN_Msk
  I2C_CR1_RXDMAEN_Pos* = (15)
  I2C_CR1_RXDMAEN_Msk* = (0x00000001 shl I2C_CR1_RXDMAEN_Pos) # !< 0x00008000
  I2C_CR1_RXDMAEN* = I2C_CR1_RXDMAEN_Msk
  I2C_CR1_SBC_Pos* = (16)
  I2C_CR1_SBC_Msk* = (0x00000001 shl I2C_CR1_SBC_Pos) # !< 0x00010000
  I2C_CR1_SBC* = I2C_CR1_SBC_Msk
  I2C_CR1_NOSTRETCH_Pos* = (17)
  I2C_CR1_NOSTRETCH_Msk* = (0x00000001 shl I2C_CR1_NOSTRETCH_Pos) # !< 0x00020000
  I2C_CR1_NOSTRETCH* = I2C_CR1_NOSTRETCH_Msk
  I2C_CR1_GCEN_Pos* = (19)
  I2C_CR1_GCEN_Msk* = (0x00000001 shl I2C_CR1_GCEN_Pos) # !< 0x00080000
  I2C_CR1_GCEN* = I2C_CR1_GCEN_Msk
  I2C_CR1_SMBHEN_Pos* = (20)
  I2C_CR1_SMBHEN_Msk* = (0x00000001 shl I2C_CR1_SMBHEN_Pos) # !< 0x00100000
  I2C_CR1_SMBHEN* = I2C_CR1_SMBHEN_Msk
  I2C_CR1_SMBDEN_Pos* = (21)
  I2C_CR1_SMBDEN_Msk* = (0x00000001 shl I2C_CR1_SMBDEN_Pos) # !< 0x00200000
  I2C_CR1_SMBDEN* = I2C_CR1_SMBDEN_Msk
  I2C_CR1_ALERTEN_Pos* = (22)
  I2C_CR1_ALERTEN_Msk* = (0x00000001 shl I2C_CR1_ALERTEN_Pos) # !< 0x00400000
  I2C_CR1_ALERTEN* = I2C_CR1_ALERTEN_Msk
  I2C_CR1_PECEN_Pos* = (23)
  I2C_CR1_PECEN_Msk* = (0x00000001 shl I2C_CR1_PECEN_Pos) # !< 0x00800000
  I2C_CR1_PECEN* = I2C_CR1_PECEN_Msk

# *****************  Bit definition for I2C_CR2 register  *******************

const
  I2C_CR2_SADD_Pos* = (0)
  I2C_CR2_SADD_Msk* = (0x000003FF shl I2C_CR2_SADD_Pos) # !< 0x000003FF
  I2C_CR2_SADD* = I2C_CR2_SADD_Msk
  I2C_CR2_RD_WRN_Pos* = (10)
  I2C_CR2_RD_WRN_Msk* = (0x00000001 shl I2C_CR2_RD_WRN_Pos) # !< 0x00000400
  I2C_CR2_RD_WRN* = I2C_CR2_RD_WRN_Msk
  I2C_CR2_ADD10_Pos* = (11)
  I2C_CR2_ADD10_Msk* = (0x00000001 shl I2C_CR2_ADD10_Pos) # !< 0x00000800
  I2C_CR2_ADD10* = I2C_CR2_ADD10_Msk
  I2C_CR2_HEAD10R_Pos* = (12)
  I2C_CR2_HEAD10R_Msk* = (0x00000001 shl I2C_CR2_HEAD10R_Pos) # !< 0x00001000
  I2C_CR2_HEAD10R* = I2C_CR2_HEAD10R_Msk
  I2C_CR2_START_Pos* = (13)
  I2C_CR2_START_Msk* = (0x00000001 shl I2C_CR2_START_Pos) # !< 0x00002000
  I2C_CR2_START* = I2C_CR2_START_Msk
  I2C_CR2_STOP_Pos* = (14)
  I2C_CR2_STOP_Msk* = (0x00000001 shl I2C_CR2_STOP_Pos) # !< 0x00004000
  I2C_CR2_STOP* = I2C_CR2_STOP_Msk
  I2C_CR2_NACK_Pos* = (15)
  I2C_CR2_NACK_Msk* = (0x00000001 shl I2C_CR2_NACK_Pos) # !< 0x00008000
  I2C_CR2_NACK* = I2C_CR2_NACK_Msk
  I2C_CR2_NBYTES_Pos* = (16)
  I2C_CR2_NBYTES_Msk* = (0x000000FF shl I2C_CR2_NBYTES_Pos) # !< 0x00FF0000
  I2C_CR2_NBYTES* = I2C_CR2_NBYTES_Msk
  I2C_CR2_RELOAD_Pos* = (24)
  I2C_CR2_RELOAD_Msk* = (0x00000001 shl I2C_CR2_RELOAD_Pos) # !< 0x01000000
  I2C_CR2_RELOAD* = I2C_CR2_RELOAD_Msk
  I2C_CR2_AUTOEND_Pos* = (25)
  I2C_CR2_AUTOEND_Msk* = (0x00000001 shl I2C_CR2_AUTOEND_Pos) # !< 0x02000000
  I2C_CR2_AUTOEND* = I2C_CR2_AUTOEND_Msk
  I2C_CR2_PECBYTE_Pos* = (26)
  I2C_CR2_PECBYTE_Msk* = (0x00000001 shl I2C_CR2_PECBYTE_Pos) # !< 0x04000000
  I2C_CR2_PECBYTE* = I2C_CR2_PECBYTE_Msk

# ******************  Bit definition for I2C_OAR1 register  *****************

const
  I2C_OAR1_OA1_Pos* = (0)
  I2C_OAR1_OA1_Msk* = (0x000003FF shl I2C_OAR1_OA1_Pos) # !< 0x000003FF
  I2C_OAR1_OA1* = I2C_OAR1_OA1_Msk
  I2C_OAR1_OA1MODE_Pos* = (10)
  I2C_OAR1_OA1MODE_Msk* = (0x00000001 shl I2C_OAR1_OA1MODE_Pos) # !< 0x00000400
  I2C_OAR1_OA1MODE* = I2C_OAR1_OA1MODE_Msk
  I2C_OAR1_OA1EN_Pos* = (15)
  I2C_OAR1_OA1EN_Msk* = (0x00000001 shl I2C_OAR1_OA1EN_Pos) # !< 0x00008000
  I2C_OAR1_OA1EN* = I2C_OAR1_OA1EN_Msk

# ******************  Bit definition for I2C_OAR2 register  *****************

const
  I2C_OAR2_OA2_Pos* = (1)
  I2C_OAR2_OA2_Msk* = (0x0000007F shl I2C_OAR2_OA2_Pos) # !< 0x000000FE
  I2C_OAR2_OA2* = I2C_OAR2_OA2_Msk
  I2C_OAR2_OA2MSK_Pos* = (8)
  I2C_OAR2_OA2MSK_Msk* = (0x00000007 shl I2C_OAR2_OA2MSK_Pos) # !< 0x00000700
#I2C_OAR2_OA2MSK* = I2C_OAR2_OA2MSK_Msk
  I2C_OAR2_OA2NOMASK* = (0x00000000) # !< No mask
  I2C_OAR2_OA2MASK01_Pos* = (8)
  I2C_OAR2_OA2MASK01_Msk* = (0x00000001 shl I2C_OAR2_OA2MASK01_Pos) # !< 0x00000100
  I2C_OAR2_OA2MASK01* = I2C_OAR2_OA2MASK01_Msk
  I2C_OAR2_OA2MASK02_Pos* = (9)
  I2C_OAR2_OA2MASK02_Msk* = (0x00000001 shl I2C_OAR2_OA2MASK02_Pos) # !< 0x00000200
  I2C_OAR2_OA2MASK02* = I2C_OAR2_OA2MASK02_Msk
  I2C_OAR2_OA2MASK03_Pos* = (8)
  I2C_OAR2_OA2MASK03_Msk* = (0x00000003 shl I2C_OAR2_OA2MASK03_Pos) # !< 0x00000300
  I2C_OAR2_OA2MASK03* = I2C_OAR2_OA2MASK03_Msk
  I2C_OAR2_OA2MASK04_Pos* = (10)
  I2C_OAR2_OA2MASK04_Msk* = (0x00000001 shl I2C_OAR2_OA2MASK04_Pos) # !< 0x00000400
  I2C_OAR2_OA2MASK04* = I2C_OAR2_OA2MASK04_Msk
  I2C_OAR2_OA2MASK05_Pos* = (8)
  I2C_OAR2_OA2MASK05_Msk* = (0x00000005 shl I2C_OAR2_OA2MASK05_Pos) # !< 0x00000500
  I2C_OAR2_OA2MASK05* = I2C_OAR2_OA2MASK05_Msk
  I2C_OAR2_OA2MASK06_Pos* = (9)
  I2C_OAR2_OA2MASK06_Msk* = (0x00000003 shl I2C_OAR2_OA2MASK06_Pos) # !< 0x00000600
  I2C_OAR2_OA2MASK06* = I2C_OAR2_OA2MASK06_Msk
  I2C_OAR2_OA2MASK07_Pos* = (8)
  I2C_OAR2_OA2MASK07_Msk* = (0x00000007 shl I2C_OAR2_OA2MASK07_Pos) # !< 0x00000700
  I2C_OAR2_OA2MASK07* = I2C_OAR2_OA2MASK07_Msk
  I2C_OAR2_OA2EN_Pos* = (15)
  I2C_OAR2_OA2EN_Msk* = (0x00000001 shl I2C_OAR2_OA2EN_Pos) # !< 0x00008000
  I2C_OAR2_OA2EN* = I2C_OAR2_OA2EN_Msk

# ******************  Bit definition for I2C_TIMINGR register ***************

const
  I2C_TIMINGR_SCLL_Pos* = (0)
  I2C_TIMINGR_SCLL_Msk* = (0x000000FF shl I2C_TIMINGR_SCLL_Pos) # !< 0x000000FF
  I2C_TIMINGR_SCLL* = I2C_TIMINGR_SCLL_Msk
  I2C_TIMINGR_SCLH_Pos* = (8)
  I2C_TIMINGR_SCLH_Msk* = (0x000000FF shl I2C_TIMINGR_SCLH_Pos) # !< 0x0000FF00
  I2C_TIMINGR_SCLH* = I2C_TIMINGR_SCLH_Msk
  I2C_TIMINGR_SDADEL_Pos* = (16)
  I2C_TIMINGR_SDADEL_Msk* = (0x0000000F shl I2C_TIMINGR_SDADEL_Pos) # !< 0x000F0000
  I2C_TIMINGR_SDADEL* = I2C_TIMINGR_SDADEL_Msk
  I2C_TIMINGR_SCLDEL_Pos* = (20)
  I2C_TIMINGR_SCLDEL_Msk* = (0x0000000F shl I2C_TIMINGR_SCLDEL_Pos) # !< 0x00F00000
  I2C_TIMINGR_SCLDEL* = I2C_TIMINGR_SCLDEL_Msk
  I2C_TIMINGR_PRESC_Pos* = (28)
  I2C_TIMINGR_PRESC_Msk* = (0x0000000F shl I2C_TIMINGR_PRESC_Pos) # !< 0xF0000000
  I2C_TIMINGR_PRESC* = I2C_TIMINGR_PRESC_Msk

# ****************** Bit definition for I2C_TIMEOUTR register ***************

const
  I2C_TIMEOUTR_TIMEOUTA_Pos* = (0)
  I2C_TIMEOUTR_TIMEOUTA_Msk* = (0x00000FFF shl I2C_TIMEOUTR_TIMEOUTA_Pos) # !< 0x00000FFF
  I2C_TIMEOUTR_TIMEOUTA* = I2C_TIMEOUTR_TIMEOUTA_Msk
  I2C_TIMEOUTR_TIDLE_Pos* = (12)
  I2C_TIMEOUTR_TIDLE_Msk* = (0x00000001 shl I2C_TIMEOUTR_TIDLE_Pos) # !< 0x00001000
  I2C_TIMEOUTR_TIDLE* = I2C_TIMEOUTR_TIDLE_Msk
  I2C_TIMEOUTR_TIMOUTEN_Pos* = (15)
  I2C_TIMEOUTR_TIMOUTEN_Msk* = (0x00000001 shl I2C_TIMEOUTR_TIMOUTEN_Pos) # !< 0x00008000
  I2C_TIMEOUTR_TIMOUTEN* = I2C_TIMEOUTR_TIMOUTEN_Msk
  I2C_TIMEOUTR_TIMEOUTB_Pos* = (16)
  I2C_TIMEOUTR_TIMEOUTB_Msk* = (0x00000FFF shl I2C_TIMEOUTR_TIMEOUTB_Pos) # !< 0x0FFF0000
  I2C_TIMEOUTR_TIMEOUTB* = I2C_TIMEOUTR_TIMEOUTB_Msk
  I2C_TIMEOUTR_TEXTEN_Pos* = (31)
  I2C_TIMEOUTR_TEXTEN_Msk* = (0x00000001 shl I2C_TIMEOUTR_TEXTEN_Pos) # !< 0x80000000
  I2C_TIMEOUTR_TEXTEN* = I2C_TIMEOUTR_TEXTEN_Msk

# *****************  Bit definition for I2C_ISR register  *******************

const
  I2C_ISR_TXE_Pos* = (0)
  I2C_ISR_TXE_Msk* = (0x00000001 shl I2C_ISR_TXE_Pos) # !< 0x00000001
  I2C_ISR_TXE* = I2C_ISR_TXE_Msk
  I2C_ISR_TXIS_Pos* = (1)
  I2C_ISR_TXIS_Msk* = (0x00000001 shl I2C_ISR_TXIS_Pos) # !< 0x00000002
  I2C_ISR_TXIS* = I2C_ISR_TXIS_Msk
  I2C_ISR_RXNE_Pos* = (2)
  I2C_ISR_RXNE_Msk* = (0x00000001 shl I2C_ISR_RXNE_Pos) # !< 0x00000004
  I2C_ISR_RXNE* = I2C_ISR_RXNE_Msk
  I2C_ISR_ADDR_Pos* = (3)
  I2C_ISR_ADDR_Msk* = (0x00000001 shl I2C_ISR_ADDR_Pos) # !< 0x00000008
  I2C_ISR_ADDR* = I2C_ISR_ADDR_Msk
  I2C_ISR_NACKF_Pos* = (4)
  I2C_ISR_NACKF_Msk* = (0x00000001 shl I2C_ISR_NACKF_Pos) # !< 0x00000010
  I2C_ISR_NACKF* = I2C_ISR_NACKF_Msk
  I2C_ISR_STOPF_Pos* = (5)
  I2C_ISR_STOPF_Msk* = (0x00000001 shl I2C_ISR_STOPF_Pos) # !< 0x00000020
  I2C_ISR_STOPF* = I2C_ISR_STOPF_Msk
  I2C_ISR_TC_Pos* = (6)
  I2C_ISR_TC_Msk* = (0x00000001 shl I2C_ISR_TC_Pos) # !< 0x00000040
  I2C_ISR_TC* = I2C_ISR_TC_Msk
  I2C_ISR_TCR_Pos* = (7)
  I2C_ISR_TCR_Msk* = (0x00000001 shl I2C_ISR_TCR_Pos) # !< 0x00000080
  I2C_ISR_TCR* = I2C_ISR_TCR_Msk
  I2C_ISR_BERR_Pos* = (8)
  I2C_ISR_BERR_Msk* = (0x00000001 shl I2C_ISR_BERR_Pos) # !< 0x00000100
  I2C_ISR_BERR* = I2C_ISR_BERR_Msk
  I2C_ISR_ARLO_Pos* = (9)
  I2C_ISR_ARLO_Msk* = (0x00000001 shl I2C_ISR_ARLO_Pos) # !< 0x00000200
  I2C_ISR_ARLO* = I2C_ISR_ARLO_Msk
  I2C_ISR_OVR_Pos* = (10)
  I2C_ISR_OVR_Msk* = (0x00000001 shl I2C_ISR_OVR_Pos) # !< 0x00000400
  I2C_ISR_OVR* = I2C_ISR_OVR_Msk
  I2C_ISR_PECERR_Pos* = (11)
  I2C_ISR_PECERR_Msk* = (0x00000001 shl I2C_ISR_PECERR_Pos) # !< 0x00000800
  I2C_ISR_PECERR* = I2C_ISR_PECERR_Msk
  I2C_ISR_TIMEOUT_Pos* = (12)
  I2C_ISR_TIMEOUT_Msk* = (0x00000001 shl I2C_ISR_TIMEOUT_Pos) # !< 0x00001000
  I2C_ISR_TIMEOUT* = I2C_ISR_TIMEOUT_Msk
  I2C_ISR_ALERT_Pos* = (13)
  I2C_ISR_ALERT_Msk* = (0x00000001 shl I2C_ISR_ALERT_Pos) # !< 0x00002000
  I2C_ISR_ALERT* = I2C_ISR_ALERT_Msk
  I2C_ISR_BUSY_Pos* = (15)
  I2C_ISR_BUSY_Msk* = (0x00000001 shl I2C_ISR_BUSY_Pos) # !< 0x00008000
  I2C_ISR_BUSY* = I2C_ISR_BUSY_Msk
  I2C_ISR_DIR_Pos* = (16)
  I2C_ISR_DIR_Msk* = (0x00000001 shl I2C_ISR_DIR_Pos) # !< 0x00010000
  I2C_ISR_DIR* = I2C_ISR_DIR_Msk
  I2C_ISR_ADDCODE_Pos* = (17)
  I2C_ISR_ADDCODE_Msk* = (0x0000007F shl I2C_ISR_ADDCODE_Pos) # !< 0x00FE0000
  I2C_ISR_ADDCODE* = I2C_ISR_ADDCODE_Msk

# *****************  Bit definition for I2C_ICR register  *******************

const
  I2C_ICR_ADDRCF_Pos* = (3)
  I2C_ICR_ADDRCF_Msk* = (0x00000001 shl I2C_ICR_ADDRCF_Pos) # !< 0x00000008
  I2C_ICR_ADDRCF* = I2C_ICR_ADDRCF_Msk
  I2C_ICR_NACKCF_Pos* = (4)
  I2C_ICR_NACKCF_Msk* = (0x00000001 shl I2C_ICR_NACKCF_Pos) # !< 0x00000010
  I2C_ICR_NACKCF* = I2C_ICR_NACKCF_Msk
  I2C_ICR_STOPCF_Pos* = (5)
  I2C_ICR_STOPCF_Msk* = (0x00000001 shl I2C_ICR_STOPCF_Pos) # !< 0x00000020
  I2C_ICR_STOPCF* = I2C_ICR_STOPCF_Msk
  I2C_ICR_BERRCF_Pos* = (8)
  I2C_ICR_BERRCF_Msk* = (0x00000001 shl I2C_ICR_BERRCF_Pos) # !< 0x00000100
  I2C_ICR_BERRCF* = I2C_ICR_BERRCF_Msk
  I2C_ICR_ARLOCF_Pos* = (9)
  I2C_ICR_ARLOCF_Msk* = (0x00000001 shl I2C_ICR_ARLOCF_Pos) # !< 0x00000200
  I2C_ICR_ARLOCF* = I2C_ICR_ARLOCF_Msk
  I2C_ICR_OVRCF_Pos* = (10)
  I2C_ICR_OVRCF_Msk* = (0x00000001 shl I2C_ICR_OVRCF_Pos) # !< 0x00000400
  I2C_ICR_OVRCF* = I2C_ICR_OVRCF_Msk
  I2C_ICR_PECCF_Pos* = (11)
  I2C_ICR_PECCF_Msk* = (0x00000001 shl I2C_ICR_PECCF_Pos) # !< 0x00000800
  I2C_ICR_PECCF* = I2C_ICR_PECCF_Msk
  I2C_ICR_TIMOUTCF_Pos* = (12)
  I2C_ICR_TIMOUTCF_Msk* = (0x00000001 shl I2C_ICR_TIMOUTCF_Pos) # !< 0x00001000
  I2C_ICR_TIMOUTCF* = I2C_ICR_TIMOUTCF_Msk
  I2C_ICR_ALERTCF_Pos* = (13)
  I2C_ICR_ALERTCF_Msk* = (0x00000001 shl I2C_ICR_ALERTCF_Pos) # !< 0x00002000
  I2C_ICR_ALERTCF* = I2C_ICR_ALERTCF_Msk

# *****************  Bit definition for I2C_PECR register  ******************

const
  I2C_PECR_PEC_Pos* = (0)
  I2C_PECR_PEC_Msk* = (0x000000FF shl I2C_PECR_PEC_Pos) # !< 0x000000FF
  I2C_PECR_PEC* = I2C_PECR_PEC_Msk

# *****************  Bit definition for I2C_RXDR register  ********************

const
  I2C_RXDR_RXDATA_Pos* = (0)
  I2C_RXDR_RXDATA_Msk* = (0x000000FF shl I2C_RXDR_RXDATA_Pos) # !< 0x000000FF
  I2C_RXDR_RXDATA* = I2C_RXDR_RXDATA_Msk

# *****************  Bit definition for I2C_TXDR register  ******************

const
  I2C_TXDR_TXDATA_Pos* = (0)
  I2C_TXDR_TXDATA_Msk* = (0x000000FF shl I2C_TXDR_TXDATA_Pos) # !< 0x000000FF
  I2C_TXDR_TXDATA* = I2C_TXDR_TXDATA_Msk

# ***************************************************************************
##
#                         Independent WATCHDOG (IWDG)
##
# ***************************************************************************
# ******************  Bit definition for IWDG_KR register  ******************

const
  IWDG_KR_KEY_Pos* = (0)
  IWDG_KR_KEY_Msk* = (0x0000FFFF shl IWDG_KR_KEY_Pos) # !< 0x0000FFFF
  IWDG_KR_KEY* = IWDG_KR_KEY_Msk

# ******************  Bit definition for IWDG_PR register  ******************

const
  IWDG_PR_PR_Pos* = (0)
  IWDG_PR_PR_Msk* = (0x00000007 shl IWDG_PR_PR_Pos) # !< 0x00000007
  IWDG_PR_PR* = IWDG_PR_PR_Msk
  IWDG_PR_PR_0* = (0x00000001 shl IWDG_PR_PR_Pos) # !< 0x01
  IWDG_PR_PR_1* = (0x00000002 shl IWDG_PR_PR_Pos) # !< 0x02
  IWDG_PR_PR_2* = (0x00000004 shl IWDG_PR_PR_Pos) # !< 0x04

# ******************  Bit definition for IWDG_RLR register  *****************

const
  IWDG_RLR_RL_Pos* = (0)
  IWDG_RLR_RL_Msk* = (0x00000FFF shl IWDG_RLR_RL_Pos) # !< 0x00000FFF
  IWDG_RLR_RL* = IWDG_RLR_RL_Msk

# ******************  Bit definition for IWDG_SR register  ******************

const
  IWDG_SR_PVU_Pos* = (0)
  IWDG_SR_PVU_Msk* = (0x00000001 shl IWDG_SR_PVU_Pos) # !< 0x00000001
  IWDG_SR_PVU* = IWDG_SR_PVU_Msk
  IWDG_SR_RVU_Pos* = (1)
  IWDG_SR_RVU_Msk* = (0x00000001 shl IWDG_SR_RVU_Pos) # !< 0x00000002
  IWDG_SR_RVU* = IWDG_SR_RVU_Msk
  IWDG_SR_WVU_Pos* = (2)
  IWDG_SR_WVU_Msk* = (0x00000001 shl IWDG_SR_WVU_Pos) # !< 0x00000004
  IWDG_SR_WVU* = IWDG_SR_WVU_Msk

# ******************  Bit definition for IWDG_KR register  ******************

const
  IWDG_WINR_WIN_Pos* = (0)
  IWDG_WINR_WIN_Msk* = (0x00000FFF shl IWDG_WINR_WIN_Pos) # !< 0x00000FFF
  IWDG_WINR_WIN* = IWDG_WINR_WIN_Msk

# ***************************************************************************
##
#                           Power Control (PWR)
##
# ***************************************************************************
#  Note: No specific macro feature on this device
# *******************  Bit definition for PWR_CR register  ******************

const
  PWR_CR_LPDS_Pos* = (0)
  PWR_CR_LPDS_Msk* = (0x00000001 shl PWR_CR_LPDS_Pos) # !< 0x00000001
  PWR_CR_LPDS* = PWR_CR_LPDS_Msk
  PWR_CR_PDDS_Pos* = (1)
  PWR_CR_PDDS_Msk* = (0x00000001 shl PWR_CR_PDDS_Pos) # !< 0x00000002
  PWR_CR_PDDS* = PWR_CR_PDDS_Msk
  PWR_CR_CWUF_Pos* = (2)
  PWR_CR_CWUF_Msk* = (0x00000001 shl PWR_CR_CWUF_Pos) # !< 0x00000004
  PWR_CR_CWUF* = PWR_CR_CWUF_Msk
  PWR_CR_CSBF_Pos* = (3)
  PWR_CR_CSBF_Msk* = (0x00000001 shl PWR_CR_CSBF_Pos) # !< 0x00000008
  PWR_CR_CSBF* = PWR_CR_CSBF_Msk
  PWR_CR_DBP_Pos* = (8)
  PWR_CR_DBP_Msk* = (0x00000001 shl PWR_CR_DBP_Pos) # !< 0x00000100
  PWR_CR_DBP* = PWR_CR_DBP_Msk

# ******************  Bit definition for PWR_CSR register  ******************

const
  PWR_CSR_WUF_Pos* = (0)
  PWR_CSR_WUF_Msk* = (0x00000001 shl PWR_CSR_WUF_Pos) # !< 0x00000001
  PWR_CSR_WUF* = PWR_CSR_WUF_Msk
  PWR_CSR_SBF_Pos* = (1)
  PWR_CSR_SBF_Msk* = (0x00000001 shl PWR_CSR_SBF_Pos) # !< 0x00000002
  PWR_CSR_SBF* = PWR_CSR_SBF_Msk
  PWR_CSR_EWUP1_Pos* = (8)
  PWR_CSR_EWUP1_Msk* = (0x00000001 shl PWR_CSR_EWUP1_Pos) # !< 0x00000100
  PWR_CSR_EWUP1* = PWR_CSR_EWUP1_Msk
  PWR_CSR_EWUP2_Pos* = (9)
  PWR_CSR_EWUP2_Msk* = (0x00000001 shl PWR_CSR_EWUP2_Pos) # !< 0x00000200
  PWR_CSR_EWUP2* = PWR_CSR_EWUP2_Msk

# ***************************************************************************
##
#                          Reset and Clock Control
##
# ***************************************************************************
##
#  @brief Specific device feature definitions  (not present on all devices in the STM32F0 serie)
##
# *******************  Bit definition for RCC_CR register  ******************

const
  RCC_CR_HSION_Pos* = (0)
  RCC_CR_HSION_Msk* = (0x00000001 shl RCC_CR_HSION_Pos) # !< 0x00000001
  RCC_CR_HSION* = RCC_CR_HSION_Msk
  RCC_CR_HSIRDY_Pos* = (1)
  RCC_CR_HSIRDY_Msk* = (0x00000001 shl RCC_CR_HSIRDY_Pos) # !< 0x00000002
  RCC_CR_HSIRDY* = RCC_CR_HSIRDY_Msk
  RCC_CR_HSITRIM_Pos* = (3)
  RCC_CR_HSITRIM_Msk* = (0x0000001F shl RCC_CR_HSITRIM_Pos) # !< 0x000000F8
  RCC_CR_HSITRIM* = RCC_CR_HSITRIM_Msk
  RCC_CR_HSITRIM_0* = (0x00000001 shl RCC_CR_HSITRIM_Pos) # !< 0x00000008
  RCC_CR_HSITRIM_1* = (0x00000002 shl RCC_CR_HSITRIM_Pos) # !< 0x00000010
  RCC_CR_HSITRIM_2* = (0x00000004 shl RCC_CR_HSITRIM_Pos) # !< 0x00000020
  RCC_CR_HSITRIM_3* = (0x00000008 shl RCC_CR_HSITRIM_Pos) # !< 0x00000040
  RCC_CR_HSITRIM_4* = (0x00000010 shl RCC_CR_HSITRIM_Pos) # !< 0x00000080
  RCC_CR_HSICAL_Pos* = (8)
  RCC_CR_HSICAL_Msk* = (0x000000FF shl RCC_CR_HSICAL_Pos) # !< 0x0000FF00
  RCC_CR_HSICAL* = RCC_CR_HSICAL_Msk
  RCC_CR_HSICAL_0* = (0x00000001 shl RCC_CR_HSICAL_Pos) # !< 0x00000100
  RCC_CR_HSICAL_1* = (0x00000002 shl RCC_CR_HSICAL_Pos) # !< 0x00000200
  RCC_CR_HSICAL_2* = (0x00000004 shl RCC_CR_HSICAL_Pos) # !< 0x00000400
  RCC_CR_HSICAL_3* = (0x00000008 shl RCC_CR_HSICAL_Pos) # !< 0x00000800
  RCC_CR_HSICAL_4* = (0x00000010 shl RCC_CR_HSICAL_Pos) # !< 0x00001000
  RCC_CR_HSICAL_5* = (0x00000020 shl RCC_CR_HSICAL_Pos) # !< 0x00002000
  RCC_CR_HSICAL_6* = (0x00000040 shl RCC_CR_HSICAL_Pos) # !< 0x00004000
  RCC_CR_HSICAL_7* = (0x00000080 shl RCC_CR_HSICAL_Pos) # !< 0x00008000
  RCC_CR_HSEON_Pos* = (16)
  RCC_CR_HSEON_Msk* = (0x00000001 shl RCC_CR_HSEON_Pos) # !< 0x00010000
  RCC_CR_HSEON* = RCC_CR_HSEON_Msk
  RCC_CR_HSERDY_Pos* = (17)
  RCC_CR_HSERDY_Msk* = (0x00000001 shl RCC_CR_HSERDY_Pos) # !< 0x00020000
  RCC_CR_HSERDY* = RCC_CR_HSERDY_Msk
  RCC_CR_HSEBYP_Pos* = (18)
  RCC_CR_HSEBYP_Msk* = (0x00000001 shl RCC_CR_HSEBYP_Pos) # !< 0x00040000
  RCC_CR_HSEBYP* = RCC_CR_HSEBYP_Msk
  RCC_CR_CSSON_Pos* = (19)
  RCC_CR_CSSON_Msk* = (0x00000001 shl RCC_CR_CSSON_Pos) # !< 0x00080000
  RCC_CR_CSSON* = RCC_CR_CSSON_Msk
  RCC_CR_PLLON_Pos* = (24)
  RCC_CR_PLLON_Msk* = (0x00000001 shl RCC_CR_PLLON_Pos) # !< 0x01000000
  RCC_CR_PLLON* = RCC_CR_PLLON_Msk
  RCC_CR_PLLRDY_Pos* = (25)
  RCC_CR_PLLRDY_Msk* = (0x00000001 shl RCC_CR_PLLRDY_Pos) # !< 0x02000000
  RCC_CR_PLLRDY* = RCC_CR_PLLRDY_Msk

# *******************  Bit definition for RCC_CFGR register  ****************
# !< SW configuration

const
  RCC_CFGR_SW_Pos* = (0)
  RCC_CFGR_SW_Msk* = (0x00000003 shl RCC_CFGR_SW_Pos) # !< 0x00000003
  RCC_CFGR_SW* = RCC_CFGR_SW_Msk
  RCC_CFGR_SW_0* = (0x00000001 shl RCC_CFGR_SW_Pos) # !< 0x00000001
  RCC_CFGR_SW_1* = (0x00000002 shl RCC_CFGR_SW_Pos) # !< 0x00000002
  RCC_CFGR_SW_HSI* = (0x00000000) # !< HSI selected as system clock
  RCC_CFGR_SW_HSE* = (0x00000001) # !< HSE selected as system clock
  RCC_CFGR_SW_PLL* = (0x00000002) # !< PLL selected as system clock

# !< SWS configuration

const
  RCC_CFGR_SWS_Pos* = (2)
  RCC_CFGR_SWS_Msk* = (0x00000003 shl RCC_CFGR_SWS_Pos) # !< 0x0000000C
  RCC_CFGR_SWS* = RCC_CFGR_SWS_Msk
  RCC_CFGR_SWS_0* = (0x00000001 shl RCC_CFGR_SWS_Pos) # !< 0x00000004
  RCC_CFGR_SWS_1* = (0x00000002 shl RCC_CFGR_SWS_Pos) # !< 0x00000008
  RCC_CFGR_SWS_HSI* = (0x00000000) # !< HSI oscillator used as system clock
  RCC_CFGR_SWS_HSE* = (0x00000004) # !< HSE oscillator used as system clock
  RCC_CFGR_SWS_PLL* = (0x00000008) # !< PLL used as system clock

# !< HPRE configuration

const
  RCC_CFGR_HPRE_Pos* = (4)
  RCC_CFGR_HPRE_Msk* = (0x0000000F shl RCC_CFGR_HPRE_Pos) # !< 0x000000F0
  RCC_CFGR_HPRE* = RCC_CFGR_HPRE_Msk
  RCC_CFGR_HPRE_0* = (0x00000001 shl RCC_CFGR_HPRE_Pos) # !< 0x00000010
  RCC_CFGR_HPRE_1* = (0x00000002 shl RCC_CFGR_HPRE_Pos) # !< 0x00000020
  RCC_CFGR_HPRE_2* = (0x00000004 shl RCC_CFGR_HPRE_Pos) # !< 0x00000040
  RCC_CFGR_HPRE_3* = (0x00000008 shl RCC_CFGR_HPRE_Pos) # !< 0x00000080
  RCC_CFGR_HPRE_DIV1* = (0x00000000) # !< SYSCLK not divided
  RCC_CFGR_HPRE_DIV2* = (0x00000080) # !< SYSCLK divided by 2
  RCC_CFGR_HPRE_DIV4* = (0x00000090) # !< SYSCLK divided by 4
  RCC_CFGR_HPRE_DIV8* = (0x000000A0) # !< SYSCLK divided by 8
  RCC_CFGR_HPRE_DIV16* = (0x000000B0) # !< SYSCLK divided by 16
  RCC_CFGR_HPRE_DIV64* = (0x000000C0) # !< SYSCLK divided by 64
  RCC_CFGR_HPRE_DIV128* = (0x000000D0) # !< SYSCLK divided by 128
  RCC_CFGR_HPRE_DIV256* = (0x000000E0) # !< SYSCLK divided by 256
  RCC_CFGR_HPRE_DIV512* = (0x000000F0) # !< SYSCLK divided by 512

# !< PPRE configuration

const
  RCC_CFGR_PPRE_Pos* = (8)
  RCC_CFGR_PPRE_Msk* = (0x00000007 shl RCC_CFGR_PPRE_Pos) # !< 0x00000700
  RCC_CFGR_PPRE* = RCC_CFGR_PPRE_Msk
  RCC_CFGR_PPRE_0* = (0x00000001 shl RCC_CFGR_PPRE_Pos) # !< 0x00000100
  RCC_CFGR_PPRE_1* = (0x00000002 shl RCC_CFGR_PPRE_Pos) # !< 0x00000200
  RCC_CFGR_PPRE_2* = (0x00000004 shl RCC_CFGR_PPRE_Pos) # !< 0x00000400
  RCC_CFGR_PPRE_DIV1* = (0x00000000) # !< HCLK not divided
  RCC_CFGR_PPRE_DIV2_Pos* = (10)
  RCC_CFGR_PPRE_DIV2_Msk* = (0x00000001 shl RCC_CFGR_PPRE_DIV2_Pos) # !< 0x00000400
  RCC_CFGR_PPRE_DIV2* = RCC_CFGR_PPRE_DIV2_Msk
  RCC_CFGR_PPRE_DIV4_Pos* = (8)
  RCC_CFGR_PPRE_DIV4_Msk* = (0x00000005 shl RCC_CFGR_PPRE_DIV4_Pos) # !< 0x00000500
  RCC_CFGR_PPRE_DIV4* = RCC_CFGR_PPRE_DIV4_Msk
  RCC_CFGR_PPRE_DIV8_Pos* = (9)
  RCC_CFGR_PPRE_DIV8_Msk* = (0x00000003 shl RCC_CFGR_PPRE_DIV8_Pos) # !< 0x00000600
  RCC_CFGR_PPRE_DIV8* = RCC_CFGR_PPRE_DIV8_Msk
  RCC_CFGR_PPRE_DIV16_Pos* = (8)
  RCC_CFGR_PPRE_DIV16_Msk* = (0x00000007 shl RCC_CFGR_PPRE_DIV16_Pos) # !< 0x00000700
  RCC_CFGR_PPRE_DIV16* = RCC_CFGR_PPRE_DIV16_Msk

# !< ADCPPRE configuration

const
  RCC_CFGR_ADCPRE_Pos* = (14)
  RCC_CFGR_ADCPRE_Msk* = (0x00000001 shl RCC_CFGR_ADCPRE_Pos) # !< 0x00004000
  RCC_CFGR_ADCPRE* = RCC_CFGR_ADCPRE_Msk
  RCC_CFGR_ADCPRE_DIV2* = (0x00000000) # !< PCLK divided by 2
  RCC_CFGR_ADCPRE_DIV4* = (0x00004000) # !< PCLK divided by 4
  RCC_CFGR_PLLSRC_Pos* = (16)
  RCC_CFGR_PLLSRC_Msk* = (0x00000001 shl RCC_CFGR_PLLSRC_Pos) # !< 0x00010000
  RCC_CFGR_PLLSRC* = RCC_CFGR_PLLSRC_Msk
  RCC_CFGR_PLLSRC_HSI_DIV2* = (0x00000000) # !< HSI clock divided by 2 selected as PLL entry clock source
  RCC_CFGR_PLLSRC_HSE_PREDIV* = (0x00010000) # !< HSE/PREDIV clock selected as PLL entry clock source
  RCC_CFGR_PLLXTPRE_Pos* = (17)
  RCC_CFGR_PLLXTPRE_Msk* = (0x00000001 shl RCC_CFGR_PLLXTPRE_Pos) # !< 0x00020000
  RCC_CFGR_PLLXTPRE* = RCC_CFGR_PLLXTPRE_Msk
  RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV1* = (0x00000000) # !< HSE/PREDIV clock not divided for PLL entry
  RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV2* = (0x00020000) # !< HSE/PREDIV clock divided by 2 for PLL entry

# !< PLLMUL configuration

const
  RCC_CFGR_PLLMUL_Pos* = (18)
  RCC_CFGR_PLLMUL_Msk* = (0x0000000F shl RCC_CFGR_PLLMUL_Pos) # !< 0x003C0000
  RCC_CFGR_PLLMUL* = RCC_CFGR_PLLMUL_Msk
  RCC_CFGR_PLLMUL_0* = (0x00000001 shl RCC_CFGR_PLLMUL_Pos) # !< 0x00040000
  RCC_CFGR_PLLMUL_1* = (0x00000002 shl RCC_CFGR_PLLMUL_Pos) # !< 0x00080000
  RCC_CFGR_PLLMUL_2* = (0x00000004 shl RCC_CFGR_PLLMUL_Pos) # !< 0x00100000
  RCC_CFGR_PLLMUL_3* = (0x00000008 shl RCC_CFGR_PLLMUL_Pos) # !< 0x00200000
#RCC_CFGR_PLLMUL2* = (0x00000000) # !< PLL input clock*2
#RCC_CFGR_PLLMUL3* = (0x00040000) # !< PLL input clock*3
  RCC_CFGR_PLLMUL4* = (0x00080000) # !< PLL input clock*4
  RCC_CFGR_PLLMUL5* = (0x000C0000) # !< PLL input clock*5
  RCC_CFGR_PLLMUL6* = (0x00100000) # !< PLL input clock*6
  RCC_CFGR_PLLMUL7* = (0x00140000) # !< PLL input clock*7
  RCC_CFGR_PLLMUL8* = (0x00180000) # !< PLL input clock*8
  RCC_CFGR_PLLMUL9* = (0x001C0000) # !< PLL input clock*9
  RCC_CFGR_PLLMUL10* = (0x00200000) # !< PLL input clock10
  RCC_CFGR_PLLMUL11* = (0x00240000) # !< PLL input clock*11
  RCC_CFGR_PLLMUL12* = (0x00280000) # !< PLL input clock*12
  RCC_CFGR_PLLMUL13* = (0x002C0000) # !< PLL input clock*13
  RCC_CFGR_PLLMUL14* = (0x00300000) # !< PLL input clock*14
  RCC_CFGR_PLLMUL15* = (0x00340000) # !< PLL input clock*15
  RCC_CFGR_PLLMUL16* = (0x00380000) # !< PLL input clock*16

# !< MCO configuration

const
  RCC_CFGR_MCO_Pos* = (24)
  RCC_CFGR_MCO_Msk* = (0x0000000F shl RCC_CFGR_MCO_Pos) # !< 0x0F000000
  RCC_CFGR_MCO* = RCC_CFGR_MCO_Msk
  RCC_CFGR_MCO_0* = (0x00000001 shl RCC_CFGR_MCO_Pos) # !< 0x01000000
  RCC_CFGR_MCO_1* = (0x00000002 shl RCC_CFGR_MCO_Pos) # !< 0x02000000
  RCC_CFGR_MCO_2* = (0x00000004 shl RCC_CFGR_MCO_Pos) # !< 0x04000000
  RCC_CFGR_MCO_NOCLOCK* = (0x00000000) # !< No clock
  RCC_CFGR_MCO_HSI14* = (0x01000000) # !< HSI14 clock selected as MCO source
  RCC_CFGR_MCO_LSI* = (0x02000000) # !< LSI clock selected as MCO source
  RCC_CFGR_MCO_LSE* = (0x03000000) # !< LSE clock selected as MCO source
  RCC_CFGR_MCO_SYSCLK* = (0x04000000) # !< System clock selected as MCO source
  RCC_CFGR_MCO_HSI* = (0x05000000) # !< HSI clock selected as MCO source
  RCC_CFGR_MCO_HSE* = (0x06000000) # !< HSE clock selected as MCO source
  RCC_CFGR_MCO_PLL* = (0x07000000) # !< PLL clock divided by 2 selected as MCO source

#  Reference defines

const
  RCC_CFGR_MCOSEL* = RCC_CFGR_MCO
  RCC_CFGR_MCOSEL_0* = RCC_CFGR_MCO_0
  RCC_CFGR_MCOSEL_1* = RCC_CFGR_MCO_1
  RCC_CFGR_MCOSEL_2* = RCC_CFGR_MCO_2
  RCC_CFGR_MCOSEL_NOCLOCK* = RCC_CFGR_MCO_NOCLOCK
  RCC_CFGR_MCOSEL_HSI14* = RCC_CFGR_MCO_HSI14
  RCC_CFGR_MCOSEL_LSI* = RCC_CFGR_MCO_LSI
  RCC_CFGR_MCOSEL_LSE* = RCC_CFGR_MCO_LSE
  RCC_CFGR_MCOSEL_SYSCLK* = RCC_CFGR_MCO_SYSCLK
  RCC_CFGR_MCOSEL_HSI* = RCC_CFGR_MCO_HSI
  RCC_CFGR_MCOSEL_HSE* = RCC_CFGR_MCO_HSE
  RCC_CFGR_MCOSEL_PLL_DIV2* = RCC_CFGR_MCO_PLL

# !<******************  Bit definition for RCC_CIR register  ****************

const
  RCC_CIR_LSIRDYF_Pos* = (0)
  RCC_CIR_LSIRDYF_Msk* = (0x00000001 shl RCC_CIR_LSIRDYF_Pos) # !< 0x00000001
  RCC_CIR_LSIRDYF* = RCC_CIR_LSIRDYF_Msk
  RCC_CIR_LSERDYF_Pos* = (1)
  RCC_CIR_LSERDYF_Msk* = (0x00000001 shl RCC_CIR_LSERDYF_Pos) # !< 0x00000002
  RCC_CIR_LSERDYF* = RCC_CIR_LSERDYF_Msk
  RCC_CIR_HSIRDYF_Pos* = (2)
  RCC_CIR_HSIRDYF_Msk* = (0x00000001 shl RCC_CIR_HSIRDYF_Pos) # !< 0x00000004
  RCC_CIR_HSIRDYF* = RCC_CIR_HSIRDYF_Msk
  RCC_CIR_HSERDYF_Pos* = (3)
  RCC_CIR_HSERDYF_Msk* = (0x00000001 shl RCC_CIR_HSERDYF_Pos) # !< 0x00000008
  RCC_CIR_HSERDYF* = RCC_CIR_HSERDYF_Msk
  RCC_CIR_PLLRDYF_Pos* = (4)
  RCC_CIR_PLLRDYF_Msk* = (0x00000001 shl RCC_CIR_PLLRDYF_Pos) # !< 0x00000010
  RCC_CIR_PLLRDYF* = RCC_CIR_PLLRDYF_Msk
  RCC_CIR_HSI14RDYF_Pos* = (5)
  RCC_CIR_HSI14RDYF_Msk* = (0x00000001 shl RCC_CIR_HSI14RDYF_Pos) # !< 0x00000020
  RCC_CIR_HSI14RDYF* = RCC_CIR_HSI14RDYF_Msk
  RCC_CIR_CSSF_Pos* = (7)
  RCC_CIR_CSSF_Msk* = (0x00000001 shl RCC_CIR_CSSF_Pos) # !< 0x00000080
  RCC_CIR_CSSF* = RCC_CIR_CSSF_Msk
  RCC_CIR_LSIRDYIE_Pos* = (8)
  RCC_CIR_LSIRDYIE_Msk* = (0x00000001 shl RCC_CIR_LSIRDYIE_Pos) # !< 0x00000100
  RCC_CIR_LSIRDYIE* = RCC_CIR_LSIRDYIE_Msk
  RCC_CIR_LSERDYIE_Pos* = (9)
  RCC_CIR_LSERDYIE_Msk* = (0x00000001 shl RCC_CIR_LSERDYIE_Pos) # !< 0x00000200
  RCC_CIR_LSERDYIE* = RCC_CIR_LSERDYIE_Msk
  RCC_CIR_HSIRDYIE_Pos* = (10)
  RCC_CIR_HSIRDYIE_Msk* = (0x00000001 shl RCC_CIR_HSIRDYIE_Pos) # !< 0x00000400
  RCC_CIR_HSIRDYIE* = RCC_CIR_HSIRDYIE_Msk
  RCC_CIR_HSERDYIE_Pos* = (11)
  RCC_CIR_HSERDYIE_Msk* = (0x00000001 shl RCC_CIR_HSERDYIE_Pos) # !< 0x00000800
  RCC_CIR_HSERDYIE* = RCC_CIR_HSERDYIE_Msk
  RCC_CIR_PLLRDYIE_Pos* = (12)
  RCC_CIR_PLLRDYIE_Msk* = (0x00000001 shl RCC_CIR_PLLRDYIE_Pos) # !< 0x00001000
  RCC_CIR_PLLRDYIE* = RCC_CIR_PLLRDYIE_Msk
  RCC_CIR_HSI14RDYIE_Pos* = (13)
  RCC_CIR_HSI14RDYIE_Msk* = (0x00000001 shl RCC_CIR_HSI14RDYIE_Pos) # !< 0x00002000
  RCC_CIR_HSI14RDYIE* = RCC_CIR_HSI14RDYIE_Msk
  RCC_CIR_LSIRDYC_Pos* = (16)
  RCC_CIR_LSIRDYC_Msk* = (0x00000001 shl RCC_CIR_LSIRDYC_Pos) # !< 0x00010000
  RCC_CIR_LSIRDYC* = RCC_CIR_LSIRDYC_Msk
  RCC_CIR_LSERDYC_Pos* = (17)
  RCC_CIR_LSERDYC_Msk* = (0x00000001 shl RCC_CIR_LSERDYC_Pos) # !< 0x00020000
  RCC_CIR_LSERDYC* = RCC_CIR_LSERDYC_Msk
  RCC_CIR_HSIRDYC_Pos* = (18)
  RCC_CIR_HSIRDYC_Msk* = (0x00000001 shl RCC_CIR_HSIRDYC_Pos) # !< 0x00040000
  RCC_CIR_HSIRDYC* = RCC_CIR_HSIRDYC_Msk
  RCC_CIR_HSERDYC_Pos* = (19)
  RCC_CIR_HSERDYC_Msk* = (0x00000001 shl RCC_CIR_HSERDYC_Pos) # !< 0x00080000
  RCC_CIR_HSERDYC* = RCC_CIR_HSERDYC_Msk
  RCC_CIR_PLLRDYC_Pos* = (20)
  RCC_CIR_PLLRDYC_Msk* = (0x00000001 shl RCC_CIR_PLLRDYC_Pos) # !< 0x00100000
  RCC_CIR_PLLRDYC* = RCC_CIR_PLLRDYC_Msk
  RCC_CIR_HSI14RDYC_Pos* = (21)
  RCC_CIR_HSI14RDYC_Msk* = (0x00000001 shl RCC_CIR_HSI14RDYC_Pos) # !< 0x00200000
  RCC_CIR_HSI14RDYC* = RCC_CIR_HSI14RDYC_Msk
  RCC_CIR_CSSC_Pos* = (23)
  RCC_CIR_CSSC_Msk* = (0x00000001 shl RCC_CIR_CSSC_Pos) # !< 0x00800000
  RCC_CIR_CSSC* = RCC_CIR_CSSC_Msk

# ****************  Bit definition for RCC_APB2RSTR register  ***************

const
  RCC_APB2RSTR_SYSCFGRST_Pos* = (0)
  RCC_APB2RSTR_SYSCFGRST_Msk* = (0x00000001 shl RCC_APB2RSTR_SYSCFGRST_Pos) # !< 0x00000001
  RCC_APB2RSTR_SYSCFGRST* = RCC_APB2RSTR_SYSCFGRST_Msk
  RCC_APB2RSTR_ADCRST_Pos* = (9)
  RCC_APB2RSTR_ADCRST_Msk* = (0x00000001 shl RCC_APB2RSTR_ADCRST_Pos) # !< 0x00000200
  RCC_APB2RSTR_ADCRST* = RCC_APB2RSTR_ADCRST_Msk
  RCC_APB2RSTR_TIM1RST_Pos* = (11)
  RCC_APB2RSTR_TIM1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM1RST_Pos) # !< 0x00000800
  RCC_APB2RSTR_TIM1RST* = RCC_APB2RSTR_TIM1RST_Msk
  RCC_APB2RSTR_SPI1RST_Pos* = (12)
  RCC_APB2RSTR_SPI1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_SPI1RST_Pos) # !< 0x00001000
  RCC_APB2RSTR_SPI1RST* = RCC_APB2RSTR_SPI1RST_Msk
  RCC_APB2RSTR_USART1RST_Pos* = (14)
  RCC_APB2RSTR_USART1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_USART1RST_Pos) # !< 0x00004000
  RCC_APB2RSTR_USART1RST* = RCC_APB2RSTR_USART1RST_Msk
  RCC_APB2RSTR_TIM15RST_Pos* = (16)
  RCC_APB2RSTR_TIM15RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM15RST_Pos) # !< 0x00010000
  RCC_APB2RSTR_TIM15RST* = RCC_APB2RSTR_TIM15RST_Msk
  RCC_APB2RSTR_TIM16RST_Pos* = (17)
  RCC_APB2RSTR_TIM16RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM16RST_Pos) # !< 0x00020000
  RCC_APB2RSTR_TIM16RST* = RCC_APB2RSTR_TIM16RST_Msk
  RCC_APB2RSTR_TIM17RST_Pos* = (18)
  RCC_APB2RSTR_TIM17RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM17RST_Pos) # !< 0x00040000
  RCC_APB2RSTR_TIM17RST* = RCC_APB2RSTR_TIM17RST_Msk
  RCC_APB2RSTR_DBGMCURST_Pos* = (22)
  RCC_APB2RSTR_DBGMCURST_Msk* = (0x00000001 shl RCC_APB2RSTR_DBGMCURST_Pos) # !< 0x00400000
  RCC_APB2RSTR_DBGMCURST* = RCC_APB2RSTR_DBGMCURST_Msk

# !< Old ADC1 reset bit definition maintained for legacy purpose

const
  RCC_APB2RSTR_ADC1RST* = RCC_APB2RSTR_ADCRST

# ****************  Bit definition for RCC_APB1RSTR register  ***************

const
  RCC_APB1RSTR_TIM3RST_Pos* = (1)
  RCC_APB1RSTR_TIM3RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM3RST_Pos) # !< 0x00000002
  RCC_APB1RSTR_TIM3RST* = RCC_APB1RSTR_TIM3RST_Msk
  RCC_APB1RSTR_TIM6RST_Pos* = (4)
  RCC_APB1RSTR_TIM6RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM6RST_Pos) # !< 0x00000010
  RCC_APB1RSTR_TIM6RST* = RCC_APB1RSTR_TIM6RST_Msk
  RCC_APB1RSTR_TIM14RST_Pos* = (8)
  RCC_APB1RSTR_TIM14RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM14RST_Pos) # !< 0x00000100
  RCC_APB1RSTR_TIM14RST* = RCC_APB1RSTR_TIM14RST_Msk
  RCC_APB1RSTR_WWDGRST_Pos* = (11)
  RCC_APB1RSTR_WWDGRST_Msk* = (0x00000001 shl RCC_APB1RSTR_WWDGRST_Pos) # !< 0x00000800
  RCC_APB1RSTR_WWDGRST* = RCC_APB1RSTR_WWDGRST_Msk
  RCC_APB1RSTR_SPI2RST_Pos* = (14)
  RCC_APB1RSTR_SPI2RST_Msk* = (0x00000001 shl RCC_APB1RSTR_SPI2RST_Pos) # !< 0x00004000
  RCC_APB1RSTR_SPI2RST* = RCC_APB1RSTR_SPI2RST_Msk
  RCC_APB1RSTR_USART2RST_Pos* = (17)
  RCC_APB1RSTR_USART2RST_Msk* = (0x00000001 shl RCC_APB1RSTR_USART2RST_Pos) # !< 0x00020000
  RCC_APB1RSTR_USART2RST* = RCC_APB1RSTR_USART2RST_Msk
  RCC_APB1RSTR_I2C1RST_Pos* = (21)
  RCC_APB1RSTR_I2C1RST_Msk* = (0x00000001 shl RCC_APB1RSTR_I2C1RST_Pos) # !< 0x00200000
  RCC_APB1RSTR_I2C1RST* = RCC_APB1RSTR_I2C1RST_Msk
  RCC_APB1RSTR_I2C2RST_Pos* = (22)
  RCC_APB1RSTR_I2C2RST_Msk* = (0x00000001 shl RCC_APB1RSTR_I2C2RST_Pos) # !< 0x00400000
  RCC_APB1RSTR_I2C2RST* = RCC_APB1RSTR_I2C2RST_Msk
  RCC_APB1RSTR_PWRRST_Pos* = (28)
  RCC_APB1RSTR_PWRRST_Msk* = (0x00000001 shl RCC_APB1RSTR_PWRRST_Pos) # !< 0x10000000
  RCC_APB1RSTR_PWRRST* = RCC_APB1RSTR_PWRRST_Msk

# *****************  Bit definition for RCC_AHBENR register  ****************

const
  RCC_AHBENR_DMAEN_Pos* = (0)
  RCC_AHBENR_DMAEN_Msk* = (0x00000001 shl RCC_AHBENR_DMAEN_Pos) # !< 0x00000001
  RCC_AHBENR_DMAEN* = RCC_AHBENR_DMAEN_Msk
  RCC_AHBENR_SRAMEN_Pos* = (2)
  RCC_AHBENR_SRAMEN_Msk* = (0x00000001 shl RCC_AHBENR_SRAMEN_Pos) # !< 0x00000004
  RCC_AHBENR_SRAMEN* = RCC_AHBENR_SRAMEN_Msk
  RCC_AHBENR_FLITFEN_Pos* = (4)
  RCC_AHBENR_FLITFEN_Msk* = (0x00000001 shl RCC_AHBENR_FLITFEN_Pos) # !< 0x00000010
  RCC_AHBENR_FLITFEN* = RCC_AHBENR_FLITFEN_Msk
  RCC_AHBENR_CRCEN_Pos* = (6)
  RCC_AHBENR_CRCEN_Msk* = (0x00000001 shl RCC_AHBENR_CRCEN_Pos) # !< 0x00000040
  RCC_AHBENR_CRCEN* = RCC_AHBENR_CRCEN_Msk
  RCC_AHBENR_GPIOAEN_Pos* = (17)
  RCC_AHBENR_GPIOAEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOAEN_Pos) # !< 0x00020000
  RCC_AHBENR_GPIOAEN* = RCC_AHBENR_GPIOAEN_Msk
  RCC_AHBENR_GPIOBEN_Pos* = (18)
  RCC_AHBENR_GPIOBEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOBEN_Pos) # !< 0x00040000
  RCC_AHBENR_GPIOBEN* = RCC_AHBENR_GPIOBEN_Msk
  RCC_AHBENR_GPIOCEN_Pos* = (19)
  RCC_AHBENR_GPIOCEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOCEN_Pos) # !< 0x00080000
  RCC_AHBENR_GPIOCEN* = RCC_AHBENR_GPIOCEN_Msk
  RCC_AHBENR_GPIODEN_Pos* = (20)
  RCC_AHBENR_GPIODEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIODEN_Pos) # !< 0x00100000
  RCC_AHBENR_GPIODEN* = RCC_AHBENR_GPIODEN_Msk
  RCC_AHBENR_GPIOFEN_Pos* = (22)
  RCC_AHBENR_GPIOFEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOFEN_Pos) # !< 0x00400000
  RCC_AHBENR_GPIOFEN* = RCC_AHBENR_GPIOFEN_Msk

#  Old Bit definition maintained for legacy purpose

const
  RCC_AHBENR_DMA1EN* = RCC_AHBENR_DMAEN
#RCC_AHBENR_TSEN* = RCC_AHBENR_TSCEN

# ****************  Bit definition for RCC_APB2ENR register  ****************

const
  RCC_APB2ENR_SYSCFGCOMPEN_Pos* = (0)
  RCC_APB2ENR_SYSCFGCOMPEN_Msk* = (0x00000001 shl RCC_APB2ENR_SYSCFGCOMPEN_Pos) # !< 0x00000001
  RCC_APB2ENR_SYSCFGCOMPEN* = RCC_APB2ENR_SYSCFGCOMPEN_Msk
  RCC_APB2ENR_ADCEN_Pos* = (9)
  RCC_APB2ENR_ADCEN_Msk* = (0x00000001 shl RCC_APB2ENR_ADCEN_Pos) # !< 0x00000200
  RCC_APB2ENR_ADCEN* = RCC_APB2ENR_ADCEN_Msk
  RCC_APB2ENR_TIM1EN_Pos* = (11)
  RCC_APB2ENR_TIM1EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM1EN_Pos) # !< 0x00000800
  RCC_APB2ENR_TIM1EN* = RCC_APB2ENR_TIM1EN_Msk
  RCC_APB2ENR_SPI1EN_Pos* = (12)
  RCC_APB2ENR_SPI1EN_Msk* = (0x00000001 shl RCC_APB2ENR_SPI1EN_Pos) # !< 0x00001000
  RCC_APB2ENR_SPI1EN* = RCC_APB2ENR_SPI1EN_Msk
  RCC_APB2ENR_USART1EN_Pos* = (14)
  RCC_APB2ENR_USART1EN_Msk* = (0x00000001 shl RCC_APB2ENR_USART1EN_Pos) # !< 0x00004000
  RCC_APB2ENR_USART1EN* = RCC_APB2ENR_USART1EN_Msk
  RCC_APB2ENR_TIM15EN_Pos* = (16)
  RCC_APB2ENR_TIM15EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM15EN_Pos) # !< 0x00010000
  RCC_APB2ENR_TIM15EN* = RCC_APB2ENR_TIM15EN_Msk
  RCC_APB2ENR_TIM16EN_Pos* = (17)
  RCC_APB2ENR_TIM16EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM16EN_Pos) # !< 0x00020000
  RCC_APB2ENR_TIM16EN* = RCC_APB2ENR_TIM16EN_Msk
  RCC_APB2ENR_TIM17EN_Pos* = (18)
  RCC_APB2ENR_TIM17EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM17EN_Pos) # !< 0x00040000
  RCC_APB2ENR_TIM17EN* = RCC_APB2ENR_TIM17EN_Msk
  RCC_APB2ENR_DBGMCUEN_Pos* = (22)
  RCC_APB2ENR_DBGMCUEN_Msk* = (0x00000001 shl RCC_APB2ENR_DBGMCUEN_Pos) # !< 0x00400000
  RCC_APB2ENR_DBGMCUEN* = RCC_APB2ENR_DBGMCUEN_Msk

#  Old Bit definition maintained for legacy purpose

const
  RCC_APB2ENR_SYSCFGEN* = RCC_APB2ENR_SYSCFGCOMPEN
  RCC_APB2ENR_ADC1EN* = RCC_APB2ENR_ADCEN

# ****************  Bit definition for RCC_APB1ENR register  ****************

const
  RCC_APB1ENR_TIM3EN_Pos* = (1)
  RCC_APB1ENR_TIM3EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM3EN_Pos) # !< 0x00000002
  RCC_APB1ENR_TIM3EN* = RCC_APB1ENR_TIM3EN_Msk
  RCC_APB1ENR_TIM6EN_Pos* = (4)
  RCC_APB1ENR_TIM6EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM6EN_Pos) # !< 0x00000010
  RCC_APB1ENR_TIM6EN* = RCC_APB1ENR_TIM6EN_Msk
  RCC_APB1ENR_TIM14EN_Pos* = (8)
  RCC_APB1ENR_TIM14EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM14EN_Pos) # !< 0x00000100
  RCC_APB1ENR_TIM14EN* = RCC_APB1ENR_TIM14EN_Msk
  RCC_APB1ENR_WWDGEN_Pos* = (11)
  RCC_APB1ENR_WWDGEN_Msk* = (0x00000001 shl RCC_APB1ENR_WWDGEN_Pos) # !< 0x00000800
  RCC_APB1ENR_WWDGEN* = RCC_APB1ENR_WWDGEN_Msk
  RCC_APB1ENR_SPI2EN_Pos* = (14)
  RCC_APB1ENR_SPI2EN_Msk* = (0x00000001 shl RCC_APB1ENR_SPI2EN_Pos) # !< 0x00004000
  RCC_APB1ENR_SPI2EN* = RCC_APB1ENR_SPI2EN_Msk
  RCC_APB1ENR_USART2EN_Pos* = (17)
  RCC_APB1ENR_USART2EN_Msk* = (0x00000001 shl RCC_APB1ENR_USART2EN_Pos) # !< 0x00020000
  RCC_APB1ENR_USART2EN* = RCC_APB1ENR_USART2EN_Msk
  RCC_APB1ENR_I2C1EN_Pos* = (21)
  RCC_APB1ENR_I2C1EN_Msk* = (0x00000001 shl RCC_APB1ENR_I2C1EN_Pos) # !< 0x00200000
  RCC_APB1ENR_I2C1EN* = RCC_APB1ENR_I2C1EN_Msk
  RCC_APB1ENR_I2C2EN_Pos* = (22)
  RCC_APB1ENR_I2C2EN_Msk* = (0x00000001 shl RCC_APB1ENR_I2C2EN_Pos) # !< 0x00400000
  RCC_APB1ENR_I2C2EN* = RCC_APB1ENR_I2C2EN_Msk
  RCC_APB1ENR_PWREN_Pos* = (28)
  RCC_APB1ENR_PWREN_Msk* = (0x00000001 shl RCC_APB1ENR_PWREN_Pos) # !< 0x10000000
  RCC_APB1ENR_PWREN* = RCC_APB1ENR_PWREN_Msk

# ******************  Bit definition for RCC_BDCR register  *****************

const
  RCC_BDCR_LSEON_Pos* = (0)
  RCC_BDCR_LSEON_Msk* = (0x00000001 shl RCC_BDCR_LSEON_Pos) # !< 0x00000001
  RCC_BDCR_LSEON* = RCC_BDCR_LSEON_Msk
  RCC_BDCR_LSERDY_Pos* = (1)
  RCC_BDCR_LSERDY_Msk* = (0x00000001 shl RCC_BDCR_LSERDY_Pos) # !< 0x00000002
  RCC_BDCR_LSERDY* = RCC_BDCR_LSERDY_Msk
  RCC_BDCR_LSEBYP_Pos* = (2)
  RCC_BDCR_LSEBYP_Msk* = (0x00000001 shl RCC_BDCR_LSEBYP_Pos) # !< 0x00000004
  RCC_BDCR_LSEBYP* = RCC_BDCR_LSEBYP_Msk
  RCC_BDCR_LSEDRV_Pos* = (3)
  RCC_BDCR_LSEDRV_Msk* = (0x00000003 shl RCC_BDCR_LSEDRV_Pos) # !< 0x00000018
  RCC_BDCR_LSEDRV* = RCC_BDCR_LSEDRV_Msk
  RCC_BDCR_LSEDRV_0* = (0x00000001 shl RCC_BDCR_LSEDRV_Pos) # !< 0x00000008
  RCC_BDCR_LSEDRV_1* = (0x00000002 shl RCC_BDCR_LSEDRV_Pos) # !< 0x00000010
  RCC_BDCR_RTCSEL_Pos* = (8)
  RCC_BDCR_RTCSEL_Msk* = (0x00000003 shl RCC_BDCR_RTCSEL_Pos) # !< 0x00000300
  RCC_BDCR_RTCSEL* = RCC_BDCR_RTCSEL_Msk
  RCC_BDCR_RTCSEL_0* = (0x00000001 shl RCC_BDCR_RTCSEL_Pos) # !< 0x00000100
  RCC_BDCR_RTCSEL_1* = (0x00000002 shl RCC_BDCR_RTCSEL_Pos) # !< 0x00000200

# !< RTC configuration

const
  RCC_BDCR_RTCSEL_NOCLOCK* = (0x00000000) # !< No clock
  RCC_BDCR_RTCSEL_LSE* = (0x00000100) # !< LSE oscillator clock used as RTC clock
  RCC_BDCR_RTCSEL_LSI* = (0x00000200) # !< LSI oscillator clock used as RTC clock
  RCC_BDCR_RTCSEL_HSE* = (0x00000300) # !< HSE oscillator clock divided by 128 used as RTC clock
  RCC_BDCR_RTCEN_Pos* = (15)
  RCC_BDCR_RTCEN_Msk* = (0x00000001 shl RCC_BDCR_RTCEN_Pos) # !< 0x00008000
  RCC_BDCR_RTCEN* = RCC_BDCR_RTCEN_Msk
  RCC_BDCR_BDRST_Pos* = (16)
  RCC_BDCR_BDRST_Msk* = (0x00000001 shl RCC_BDCR_BDRST_Pos) # !< 0x00010000
  RCC_BDCR_BDRST* = RCC_BDCR_BDRST_Msk

# ******************  Bit definition for RCC_CSR register  ******************

const
  RCC_CSR_LSION_Pos* = (0)
  RCC_CSR_LSION_Msk* = (0x00000001 shl RCC_CSR_LSION_Pos) # !< 0x00000001
  RCC_CSR_LSION* = RCC_CSR_LSION_Msk
  RCC_CSR_LSIRDY_Pos* = (1)
  RCC_CSR_LSIRDY_Msk* = (0x00000001 shl RCC_CSR_LSIRDY_Pos) # !< 0x00000002
  RCC_CSR_LSIRDY* = RCC_CSR_LSIRDY_Msk
  RCC_CSR_V18PWRRSTF_Pos* = (23)
  RCC_CSR_V18PWRRSTF_Msk* = (0x00000001 shl RCC_CSR_V18PWRRSTF_Pos) # !< 0x00800000
  RCC_CSR_V18PWRRSTF* = RCC_CSR_V18PWRRSTF_Msk
  RCC_CSR_RMVF_Pos* = (24)
  RCC_CSR_RMVF_Msk* = (0x00000001 shl RCC_CSR_RMVF_Pos) # !< 0x01000000
  RCC_CSR_RMVF* = RCC_CSR_RMVF_Msk
  RCC_CSR_OBLRSTF_Pos* = (25)
  RCC_CSR_OBLRSTF_Msk* = (0x00000001 shl RCC_CSR_OBLRSTF_Pos) # !< 0x02000000
  RCC_CSR_OBLRSTF* = RCC_CSR_OBLRSTF_Msk
  RCC_CSR_PINRSTF_Pos* = (26)
  RCC_CSR_PINRSTF_Msk* = (0x00000001 shl RCC_CSR_PINRSTF_Pos) # !< 0x04000000
  RCC_CSR_PINRSTF* = RCC_CSR_PINRSTF_Msk
  RCC_CSR_PORRSTF_Pos* = (27)
  RCC_CSR_PORRSTF_Msk* = (0x00000001 shl RCC_CSR_PORRSTF_Pos) # !< 0x08000000
  RCC_CSR_PORRSTF* = RCC_CSR_PORRSTF_Msk
  RCC_CSR_SFTRSTF_Pos* = (28)
  RCC_CSR_SFTRSTF_Msk* = (0x00000001 shl RCC_CSR_SFTRSTF_Pos) # !< 0x10000000
  RCC_CSR_SFTRSTF* = RCC_CSR_SFTRSTF_Msk
  RCC_CSR_IWDGRSTF_Pos* = (29)
  RCC_CSR_IWDGRSTF_Msk* = (0x00000001 shl RCC_CSR_IWDGRSTF_Pos) # !< 0x20000000
  RCC_CSR_IWDGRSTF* = RCC_CSR_IWDGRSTF_Msk
  RCC_CSR_WWDGRSTF_Pos* = (30)
  RCC_CSR_WWDGRSTF_Msk* = (0x00000001 shl RCC_CSR_WWDGRSTF_Pos) # !< 0x40000000
  RCC_CSR_WWDGRSTF* = RCC_CSR_WWDGRSTF_Msk
  RCC_CSR_LPWRRSTF_Pos* = (31)
  RCC_CSR_LPWRRSTF_Msk* = (0x00000001 shl RCC_CSR_LPWRRSTF_Pos) # !< 0x80000000
  RCC_CSR_LPWRRSTF* = RCC_CSR_LPWRRSTF_Msk

#  Old Bit definition maintained for legacy purpose

const
  RCC_CSR_OBL* = RCC_CSR_OBLRSTF

# ******************  Bit definition for RCC_AHBRSTR register  **************

const
  RCC_AHBRSTR_GPIOARST_Pos* = (17)
  RCC_AHBRSTR_GPIOARST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOARST_Pos) # !< 0x00020000
  RCC_AHBRSTR_GPIOARST* = RCC_AHBRSTR_GPIOARST_Msk
  RCC_AHBRSTR_GPIOBRST_Pos* = (18)
  RCC_AHBRSTR_GPIOBRST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOBRST_Pos) # !< 0x00040000
  RCC_AHBRSTR_GPIOBRST* = RCC_AHBRSTR_GPIOBRST_Msk
  RCC_AHBRSTR_GPIOCRST_Pos* = (19)
  RCC_AHBRSTR_GPIOCRST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOCRST_Pos) # !< 0x00080000
  RCC_AHBRSTR_GPIOCRST* = RCC_AHBRSTR_GPIOCRST_Msk
  RCC_AHBRSTR_GPIODRST_Pos* = (20)
  RCC_AHBRSTR_GPIODRST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIODRST_Pos) # !< 0x00100000
  RCC_AHBRSTR_GPIODRST* = RCC_AHBRSTR_GPIODRST_Msk
  RCC_AHBRSTR_GPIOFRST_Pos* = (22)
  RCC_AHBRSTR_GPIOFRST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOFRST_Pos) # !< 0x00400000
  RCC_AHBRSTR_GPIOFRST* = RCC_AHBRSTR_GPIOFRST_Msk

# ******************  Bit definition for RCC_CFGR2 register  ****************
# !< PREDIV configuration

const
  RCC_CFGR2_PREDIV_Pos* = (0)
  RCC_CFGR2_PREDIV_Msk* = (0x0000000F shl RCC_CFGR2_PREDIV_Pos) # !< 0x0000000F
  RCC_CFGR2_PREDIV* = RCC_CFGR2_PREDIV_Msk
  RCC_CFGR2_PREDIV_0* = (0x00000001 shl RCC_CFGR2_PREDIV_Pos) # !< 0x00000001
  RCC_CFGR2_PREDIV_1* = (0x00000002 shl RCC_CFGR2_PREDIV_Pos) # !< 0x00000002
  RCC_CFGR2_PREDIV_2* = (0x00000004 shl RCC_CFGR2_PREDIV_Pos) # !< 0x00000004
  RCC_CFGR2_PREDIV_3* = (0x00000008 shl RCC_CFGR2_PREDIV_Pos) # !< 0x00000008
  RCC_CFGR2_PREDIV_DIV1* = (0x00000000) # !< PREDIV input clock not divided
  RCC_CFGR2_PREDIV_DIV2* = (0x00000001) # !< PREDIV input clock divided by 2
  RCC_CFGR2_PREDIV_DIV3* = (0x00000002) # !< PREDIV input clock divided by 3
  RCC_CFGR2_PREDIV_DIV4* = (0x00000003) # !< PREDIV input clock divided by 4
  RCC_CFGR2_PREDIV_DIV5* = (0x00000004) # !< PREDIV input clock divided by 5
  RCC_CFGR2_PREDIV_DIV6* = (0x00000005) # !< PREDIV input clock divided by 6
  RCC_CFGR2_PREDIV_DIV7* = (0x00000006) # !< PREDIV input clock divided by 7
  RCC_CFGR2_PREDIV_DIV8* = (0x00000007) # !< PREDIV input clock divided by 8
  RCC_CFGR2_PREDIV_DIV9* = (0x00000008) # !< PREDIV input clock divided by 9
  RCC_CFGR2_PREDIV_DIV10* = (0x00000009) # !< PREDIV input clock divided by 10
  RCC_CFGR2_PREDIV_DIV11* = (0x0000000A) # !< PREDIV input clock divided by 11
  RCC_CFGR2_PREDIV_DIV12* = (0x0000000B) # !< PREDIV input clock divided by 12
  RCC_CFGR2_PREDIV_DIV13* = (0x0000000C) # !< PREDIV input clock divided by 13
  RCC_CFGR2_PREDIV_DIV14* = (0x0000000D) # !< PREDIV input clock divided by 14
  RCC_CFGR2_PREDIV_DIV15* = (0x0000000E) # !< PREDIV input clock divided by 15
  RCC_CFGR2_PREDIV_DIV16* = (0x0000000F) # !< PREDIV input clock divided by 16

# ******************  Bit definition for RCC_CFGR3 register  ****************
# !< USART1 Clock source selection

const
  RCC_CFGR3_USART1SW_Pos* = (0)
  RCC_CFGR3_USART1SW_Msk* = (0x00000003 shl RCC_CFGR3_USART1SW_Pos) # !< 0x00000003
  RCC_CFGR3_USART1SW* = RCC_CFGR3_USART1SW_Msk
  RCC_CFGR3_USART1SW_0* = (0x00000001 shl RCC_CFGR3_USART1SW_Pos) # !< 0x00000001
  RCC_CFGR3_USART1SW_1* = (0x00000002 shl RCC_CFGR3_USART1SW_Pos) # !< 0x00000002
  RCC_CFGR3_USART1SW_PCLK* = (0x00000000) # !< PCLK clock used as USART1 clock source
  RCC_CFGR3_USART1SW_SYSCLK* = (0x00000001) # !< System clock selected as USART1 clock source
  RCC_CFGR3_USART1SW_LSE* = (0x00000002) # !< LSE oscillator clock used as USART1 clock source
  RCC_CFGR3_USART1SW_HSI* = (0x00000003) # !< HSI oscillator clock used as USART1 clock source

# !< I2C1 Clock source selection

const
  RCC_CFGR3_I2C1SW_Pos* = (4)
  RCC_CFGR3_I2C1SW_Msk* = (0x00000001 shl RCC_CFGR3_I2C1SW_Pos) # !< 0x00000010
  RCC_CFGR3_I2C1SW* = RCC_CFGR3_I2C1SW_Msk
  RCC_CFGR3_I2C1SW_HSI* = (0x00000000) # !< HSI oscillator clock used as I2C1 clock source
  RCC_CFGR3_I2C1SW_SYSCLK_Pos* = (4)
  RCC_CFGR3_I2C1SW_SYSCLK_Msk* = (0x00000001 shl RCC_CFGR3_I2C1SW_SYSCLK_Pos) # !< 0x00000010
  RCC_CFGR3_I2C1SW_SYSCLK* = RCC_CFGR3_I2C1SW_SYSCLK_Msk

# ******************  Bit definition for RCC_CR2 register  ******************

const
  RCC_CR2_HSI14ON_Pos* = (0)
  RCC_CR2_HSI14ON_Msk* = (0x00000001 shl RCC_CR2_HSI14ON_Pos) # !< 0x00000001
  RCC_CR2_HSI14ON* = RCC_CR2_HSI14ON_Msk
  RCC_CR2_HSI14RDY_Pos* = (1)
  RCC_CR2_HSI14RDY_Msk* = (0x00000001 shl RCC_CR2_HSI14RDY_Pos) # !< 0x00000002
  RCC_CR2_HSI14RDY* = RCC_CR2_HSI14RDY_Msk
  RCC_CR2_HSI14DIS_Pos* = (2)
  RCC_CR2_HSI14DIS_Msk* = (0x00000001 shl RCC_CR2_HSI14DIS_Pos) # !< 0x00000004
  RCC_CR2_HSI14DIS* = RCC_CR2_HSI14DIS_Msk
  RCC_CR2_HSI14TRIM_Pos* = (3)
  RCC_CR2_HSI14TRIM_Msk* = (0x0000001F shl RCC_CR2_HSI14TRIM_Pos) # !< 0x000000F8
  RCC_CR2_HSI14TRIM* = RCC_CR2_HSI14TRIM_Msk
  RCC_CR2_HSI14CAL_Pos* = (8)
  RCC_CR2_HSI14CAL_Msk* = (0x000000FF shl RCC_CR2_HSI14CAL_Pos) # !< 0x0000FF00
  RCC_CR2_HSI14CAL* = RCC_CR2_HSI14CAL_Msk

# ***************************************************************************
##
#                            Real-Time Clock (RTC)
##
# ***************************************************************************
##
#  @brief Specific device feature definitions  (not present on all devices in the STM32F0 serie)
##

const
  RTC_TAMPER1_SUPPORT* = true   # !< TAMPER 1 feature support
  RTC_TAMPER2_SUPPORT* = true   # !< TAMPER 2 feature support

# *******************  Bits definition for RTC_TR register  *****************

const
  RTC_TR_PM_Pos* = (22)
  RTC_TR_PM_Msk* = (0x00000001 shl RTC_TR_PM_Pos) # !< 0x00400000
  RTC_TR_PM* = RTC_TR_PM_Msk
  RTC_TR_HT_Pos* = (20)
  RTC_TR_HT_Msk* = (0x00000003 shl RTC_TR_HT_Pos) # !< 0x00300000
  RTC_TR_HT* = RTC_TR_HT_Msk
  RTC_TR_HT_0* = (0x00000001 shl RTC_TR_HT_Pos) # !< 0x00100000
  RTC_TR_HT_1* = (0x00000002 shl RTC_TR_HT_Pos) # !< 0x00200000
  RTC_TR_HU_Pos* = (16)
  RTC_TR_HU_Msk* = (0x0000000F shl RTC_TR_HU_Pos) # !< 0x000F0000
  RTC_TR_HU* = RTC_TR_HU_Msk
  RTC_TR_HU_0* = (0x00000001 shl RTC_TR_HU_Pos) # !< 0x00010000
  RTC_TR_HU_1* = (0x00000002 shl RTC_TR_HU_Pos) # !< 0x00020000
  RTC_TR_HU_2* = (0x00000004 shl RTC_TR_HU_Pos) # !< 0x00040000
  RTC_TR_HU_3* = (0x00000008 shl RTC_TR_HU_Pos) # !< 0x00080000
  RTC_TR_MNT_Pos* = (12)
  RTC_TR_MNT_Msk* = (0x00000007 shl RTC_TR_MNT_Pos) # !< 0x00007000
  RTC_TR_MNT* = RTC_TR_MNT_Msk
  RTC_TR_MNT_0* = (0x00000001 shl RTC_TR_MNT_Pos) # !< 0x00001000
  RTC_TR_MNT_1* = (0x00000002 shl RTC_TR_MNT_Pos) # !< 0x00002000
  RTC_TR_MNT_2* = (0x00000004 shl RTC_TR_MNT_Pos) # !< 0x00004000
  RTC_TR_MNU_Pos* = (8)
  RTC_TR_MNU_Msk* = (0x0000000F shl RTC_TR_MNU_Pos) # !< 0x00000F00
  RTC_TR_MNU* = RTC_TR_MNU_Msk
  RTC_TR_MNU_0* = (0x00000001 shl RTC_TR_MNU_Pos) # !< 0x00000100
  RTC_TR_MNU_1* = (0x00000002 shl RTC_TR_MNU_Pos) # !< 0x00000200
  RTC_TR_MNU_2* = (0x00000004 shl RTC_TR_MNU_Pos) # !< 0x00000400
  RTC_TR_MNU_3* = (0x00000008 shl RTC_TR_MNU_Pos) # !< 0x00000800
  RTC_TR_ST_Pos* = (4)
  RTC_TR_ST_Msk* = (0x00000007 shl RTC_TR_ST_Pos) # !< 0x00000070
  RTC_TR_ST* = RTC_TR_ST_Msk
  RTC_TR_ST_0* = (0x00000001 shl RTC_TR_ST_Pos) # !< 0x00000010
  RTC_TR_ST_1* = (0x00000002 shl RTC_TR_ST_Pos) # !< 0x00000020
  RTC_TR_ST_2* = (0x00000004 shl RTC_TR_ST_Pos) # !< 0x00000040
  RTC_TR_SU_Pos* = (0)
  RTC_TR_SU_Msk* = (0x0000000F shl RTC_TR_SU_Pos) # !< 0x0000000F
  RTC_TR_SU* = RTC_TR_SU_Msk
  RTC_TR_SU_0* = (0x00000001 shl RTC_TR_SU_Pos) # !< 0x00000001
  RTC_TR_SU_1* = (0x00000002 shl RTC_TR_SU_Pos) # !< 0x00000002
  RTC_TR_SU_2* = (0x00000004 shl RTC_TR_SU_Pos) # !< 0x00000004
  RTC_TR_SU_3* = (0x00000008 shl RTC_TR_SU_Pos) # !< 0x00000008

# *******************  Bits definition for RTC_DR register  *****************

const
  RTC_DR_YT_Pos* = (20)
  RTC_DR_YT_Msk* = (0x0000000F shl RTC_DR_YT_Pos) # !< 0x00F00000
  RTC_DR_YT* = RTC_DR_YT_Msk
  RTC_DR_YT_0* = (0x00000001 shl RTC_DR_YT_Pos) # !< 0x00100000
  RTC_DR_YT_1* = (0x00000002 shl RTC_DR_YT_Pos) # !< 0x00200000
  RTC_DR_YT_2* = (0x00000004 shl RTC_DR_YT_Pos) # !< 0x00400000
  RTC_DR_YT_3* = (0x00000008 shl RTC_DR_YT_Pos) # !< 0x00800000
  RTC_DR_YU_Pos* = (16)
  RTC_DR_YU_Msk* = (0x0000000F shl RTC_DR_YU_Pos) # !< 0x000F0000
  RTC_DR_YU* = RTC_DR_YU_Msk
  RTC_DR_YU_0* = (0x00000001 shl RTC_DR_YU_Pos) # !< 0x00010000
  RTC_DR_YU_1* = (0x00000002 shl RTC_DR_YU_Pos) # !< 0x00020000
  RTC_DR_YU_2* = (0x00000004 shl RTC_DR_YU_Pos) # !< 0x00040000
  RTC_DR_YU_3* = (0x00000008 shl RTC_DR_YU_Pos) # !< 0x00080000
  RTC_DR_WDU_Pos* = (13)
  RTC_DR_WDU_Msk* = (0x00000007 shl RTC_DR_WDU_Pos) # !< 0x0000E000
  RTC_DR_WDU* = RTC_DR_WDU_Msk
  RTC_DR_WDU_0* = (0x00000001 shl RTC_DR_WDU_Pos) # !< 0x00002000
  RTC_DR_WDU_1* = (0x00000002 shl RTC_DR_WDU_Pos) # !< 0x00004000
  RTC_DR_WDU_2* = (0x00000004 shl RTC_DR_WDU_Pos) # !< 0x00008000
  RTC_DR_MT_Pos* = (12)
  RTC_DR_MT_Msk* = (0x00000001 shl RTC_DR_MT_Pos) # !< 0x00001000
  RTC_DR_MT* = RTC_DR_MT_Msk
  RTC_DR_MU_Pos* = (8)
  RTC_DR_MU_Msk* = (0x0000000F shl RTC_DR_MU_Pos) # !< 0x00000F00
  RTC_DR_MU* = RTC_DR_MU_Msk
  RTC_DR_MU_0* = (0x00000001 shl RTC_DR_MU_Pos) # !< 0x00000100
  RTC_DR_MU_1* = (0x00000002 shl RTC_DR_MU_Pos) # !< 0x00000200
  RTC_DR_MU_2* = (0x00000004 shl RTC_DR_MU_Pos) # !< 0x00000400
  RTC_DR_MU_3* = (0x00000008 shl RTC_DR_MU_Pos) # !< 0x00000800
  RTC_DR_DT_Pos* = (4)
  RTC_DR_DT_Msk* = (0x00000003 shl RTC_DR_DT_Pos) # !< 0x00000030
  RTC_DR_DT* = RTC_DR_DT_Msk
  RTC_DR_DT_0* = (0x00000001 shl RTC_DR_DT_Pos) # !< 0x00000010
  RTC_DR_DT_1* = (0x00000002 shl RTC_DR_DT_Pos) # !< 0x00000020
  RTC_DR_DU_Pos* = (0)
  RTC_DR_DU_Msk* = (0x0000000F shl RTC_DR_DU_Pos) # !< 0x0000000F
  RTC_DR_DU* = RTC_DR_DU_Msk
  RTC_DR_DU_0* = (0x00000001 shl RTC_DR_DU_Pos) # !< 0x00000001
  RTC_DR_DU_1* = (0x00000002 shl RTC_DR_DU_Pos) # !< 0x00000002
  RTC_DR_DU_2* = (0x00000004 shl RTC_DR_DU_Pos) # !< 0x00000004
  RTC_DR_DU_3* = (0x00000008 shl RTC_DR_DU_Pos) # !< 0x00000008

# *******************  Bits definition for RTC_CR register  *****************

const
  RTC_CR_COE_Pos* = (23)
  RTC_CR_COE_Msk* = (0x00000001 shl RTC_CR_COE_Pos) # !< 0x00800000
  RTC_CR_COE* = RTC_CR_COE_Msk
  RTC_CR_OSEL_Pos* = (21)
  RTC_CR_OSEL_Msk* = (0x00000003 shl RTC_CR_OSEL_Pos) # !< 0x00600000
  RTC_CR_OSEL* = RTC_CR_OSEL_Msk
  RTC_CR_OSEL_0* = (0x00000001 shl RTC_CR_OSEL_Pos) # !< 0x00200000
  RTC_CR_OSEL_1* = (0x00000002 shl RTC_CR_OSEL_Pos) # !< 0x00400000
  RTC_CR_POL_Pos* = (20)
  RTC_CR_POL_Msk* = (0x00000001 shl RTC_CR_POL_Pos) # !< 0x00100000
  RTC_CR_POL* = RTC_CR_POL_Msk
  RTC_CR_COSEL_Pos* = (19)
  RTC_CR_COSEL_Msk* = (0x00000001 shl RTC_CR_COSEL_Pos) # !< 0x00080000
  RTC_CR_COSEL* = RTC_CR_COSEL_Msk
  RTC_CR_BKP_Pos* = (18)
  RTC_CR_BKP_Msk* = (0x00000001 shl RTC_CR_BKP_Pos) # !< 0x00040000
  RTC_CR_BKP* = RTC_CR_BKP_Msk
  RTC_CR_SUB1H_Pos* = (17)
  RTC_CR_SUB1H_Msk* = (0x00000001 shl RTC_CR_SUB1H_Pos) # !< 0x00020000
  RTC_CR_SUB1H* = RTC_CR_SUB1H_Msk
  RTC_CR_ADD1H_Pos* = (16)
  RTC_CR_ADD1H_Msk* = (0x00000001 shl RTC_CR_ADD1H_Pos) # !< 0x00010000
  RTC_CR_ADD1H* = RTC_CR_ADD1H_Msk
  RTC_CR_TSIE_Pos* = (15)
  RTC_CR_TSIE_Msk* = (0x00000001 shl RTC_CR_TSIE_Pos) # !< 0x00008000
  RTC_CR_TSIE* = RTC_CR_TSIE_Msk
  RTC_CR_ALRAIE_Pos* = (12)
  RTC_CR_ALRAIE_Msk* = (0x00000001 shl RTC_CR_ALRAIE_Pos) # !< 0x00001000
  RTC_CR_ALRAIE* = RTC_CR_ALRAIE_Msk
  RTC_CR_TSE_Pos* = (11)
  RTC_CR_TSE_Msk* = (0x00000001 shl RTC_CR_TSE_Pos) # !< 0x00000800
  RTC_CR_TSE* = RTC_CR_TSE_Msk
  RTC_CR_ALRAE_Pos* = (8)
  RTC_CR_ALRAE_Msk* = (0x00000001 shl RTC_CR_ALRAE_Pos) # !< 0x00000100
  RTC_CR_ALRAE* = RTC_CR_ALRAE_Msk
  RTC_CR_FMT_Pos* = (6)
  RTC_CR_FMT_Msk* = (0x00000001 shl RTC_CR_FMT_Pos) # !< 0x00000040
  RTC_CR_FMT* = RTC_CR_FMT_Msk
  RTC_CR_BYPSHAD_Pos* = (5)
  RTC_CR_BYPSHAD_Msk* = (0x00000001 shl RTC_CR_BYPSHAD_Pos) # !< 0x00000020
  RTC_CR_BYPSHAD* = RTC_CR_BYPSHAD_Msk
  RTC_CR_REFCKON_Pos* = (4)
  RTC_CR_REFCKON_Msk* = (0x00000001 shl RTC_CR_REFCKON_Pos) # !< 0x00000010
  RTC_CR_REFCKON* = RTC_CR_REFCKON_Msk
  RTC_CR_TSEDGE_Pos* = (3)
  RTC_CR_TSEDGE_Msk* = (0x00000001 shl RTC_CR_TSEDGE_Pos) # !< 0x00000008
  RTC_CR_TSEDGE* = RTC_CR_TSEDGE_Msk

#  Legacy defines

const
  RTC_CR_BCK_Pos* = RTC_CR_BKP_Pos
  RTC_CR_BCK_Msk* = RTC_CR_BKP_Msk
  RTC_CR_BCK* = RTC_CR_BKP

# *******************  Bits definition for RTC_ISR register  ****************

const
  RTC_ISR_RECALPF_Pos* = (16)
  RTC_ISR_RECALPF_Msk* = (0x00000001 shl RTC_ISR_RECALPF_Pos) # !< 0x00010000
  RTC_ISR_RECALPF* = RTC_ISR_RECALPF_Msk
  RTC_ISR_TAMP2F_Pos* = (14)
  RTC_ISR_TAMP2F_Msk* = (0x00000001 shl RTC_ISR_TAMP2F_Pos) # !< 0x00004000
  RTC_ISR_TAMP2F* = RTC_ISR_TAMP2F_Msk
  RTC_ISR_TAMP1F_Pos* = (13)
  RTC_ISR_TAMP1F_Msk* = (0x00000001 shl RTC_ISR_TAMP1F_Pos) # !< 0x00002000
  RTC_ISR_TAMP1F* = RTC_ISR_TAMP1F_Msk
  RTC_ISR_TSOVF_Pos* = (12)
  RTC_ISR_TSOVF_Msk* = (0x00000001 shl RTC_ISR_TSOVF_Pos) # !< 0x00001000
  RTC_ISR_TSOVF* = RTC_ISR_TSOVF_Msk
  RTC_ISR_TSF_Pos* = (11)
  RTC_ISR_TSF_Msk* = (0x00000001 shl RTC_ISR_TSF_Pos) # !< 0x00000800
  RTC_ISR_TSF* = RTC_ISR_TSF_Msk
  RTC_ISR_ALRAF_Pos* = (8)
  RTC_ISR_ALRAF_Msk* = (0x00000001 shl RTC_ISR_ALRAF_Pos) # !< 0x00000100
  RTC_ISR_ALRAF* = RTC_ISR_ALRAF_Msk
  RTC_ISR_INIT_Pos* = (7)
  RTC_ISR_INIT_Msk* = (0x00000001 shl RTC_ISR_INIT_Pos) # !< 0x00000080
  RTC_ISR_INIT* = RTC_ISR_INIT_Msk
  RTC_ISR_INITF_Pos* = (6)
  RTC_ISR_INITF_Msk* = (0x00000001 shl RTC_ISR_INITF_Pos) # !< 0x00000040
  RTC_ISR_INITF* = RTC_ISR_INITF_Msk
  RTC_ISR_RSF_Pos* = (5)
  RTC_ISR_RSF_Msk* = (0x00000001 shl RTC_ISR_RSF_Pos) # !< 0x00000020
  RTC_ISR_RSF* = RTC_ISR_RSF_Msk
  RTC_ISR_INITS_Pos* = (4)
  RTC_ISR_INITS_Msk* = (0x00000001 shl RTC_ISR_INITS_Pos) # !< 0x00000010
  RTC_ISR_INITS* = RTC_ISR_INITS_Msk
  RTC_ISR_SHPF_Pos* = (3)
  RTC_ISR_SHPF_Msk* = (0x00000001 shl RTC_ISR_SHPF_Pos) # !< 0x00000008
  RTC_ISR_SHPF* = RTC_ISR_SHPF_Msk
  RTC_ISR_ALRAWF_Pos* = (0)
  RTC_ISR_ALRAWF_Msk* = (0x00000001 shl RTC_ISR_ALRAWF_Pos) # !< 0x00000001
  RTC_ISR_ALRAWF* = RTC_ISR_ALRAWF_Msk

# *******************  Bits definition for RTC_PRER register  ***************

const
  RTC_PRER_PREDIV_A_Pos* = (16)
  RTC_PRER_PREDIV_A_Msk* = (0x0000007F shl RTC_PRER_PREDIV_A_Pos) # !< 0x007F0000
  RTC_PRER_PREDIV_A* = RTC_PRER_PREDIV_A_Msk
  RTC_PRER_PREDIV_S_Pos* = (0)
  RTC_PRER_PREDIV_S_Msk* = (0x00007FFF shl RTC_PRER_PREDIV_S_Pos) # !< 0x00007FFF
  RTC_PRER_PREDIV_S* = RTC_PRER_PREDIV_S_Msk

# *******************  Bits definition for RTC_ALRMAR register  *************

const
  RTC_ALRMAR_MSK4_Pos* = (31)
  RTC_ALRMAR_MSK4_Msk* = (0x00000001 shl RTC_ALRMAR_MSK4_Pos) # !< 0x80000000
  RTC_ALRMAR_MSK4* = RTC_ALRMAR_MSK4_Msk
  RTC_ALRMAR_WDSEL_Pos* = (30)
  RTC_ALRMAR_WDSEL_Msk* = (0x00000001 shl RTC_ALRMAR_WDSEL_Pos) # !< 0x40000000
  RTC_ALRMAR_WDSEL* = RTC_ALRMAR_WDSEL_Msk
  RTC_ALRMAR_DT_Pos* = (28)
  RTC_ALRMAR_DT_Msk* = (0x00000003 shl RTC_ALRMAR_DT_Pos) # !< 0x30000000
  RTC_ALRMAR_DT* = RTC_ALRMAR_DT_Msk
  RTC_ALRMAR_DT_0* = (0x00000001 shl RTC_ALRMAR_DT_Pos) # !< 0x10000000
  RTC_ALRMAR_DT_1* = (0x00000002 shl RTC_ALRMAR_DT_Pos) # !< 0x20000000
  RTC_ALRMAR_DU_Pos* = (24)
  RTC_ALRMAR_DU_Msk* = (0x0000000F shl RTC_ALRMAR_DU_Pos) # !< 0x0F000000
  RTC_ALRMAR_DU* = RTC_ALRMAR_DU_Msk
  RTC_ALRMAR_DU_0* = (0x00000001 shl RTC_ALRMAR_DU_Pos) # !< 0x01000000
  RTC_ALRMAR_DU_1* = (0x00000002 shl RTC_ALRMAR_DU_Pos) # !< 0x02000000
  RTC_ALRMAR_DU_2* = (0x00000004 shl RTC_ALRMAR_DU_Pos) # !< 0x04000000
  RTC_ALRMAR_DU_3* = (0x00000008 shl RTC_ALRMAR_DU_Pos) # !< 0x08000000
  RTC_ALRMAR_MSK3_Pos* = (23)
  RTC_ALRMAR_MSK3_Msk* = (0x00000001 shl RTC_ALRMAR_MSK3_Pos) # !< 0x00800000
  RTC_ALRMAR_MSK3* = RTC_ALRMAR_MSK3_Msk
  RTC_ALRMAR_PM_Pos* = (22)
  RTC_ALRMAR_PM_Msk* = (0x00000001 shl RTC_ALRMAR_PM_Pos) # !< 0x00400000
  RTC_ALRMAR_PM* = RTC_ALRMAR_PM_Msk
  RTC_ALRMAR_HT_Pos* = (20)
  RTC_ALRMAR_HT_Msk* = (0x00000003 shl RTC_ALRMAR_HT_Pos) # !< 0x00300000
  RTC_ALRMAR_HT* = RTC_ALRMAR_HT_Msk
  RTC_ALRMAR_HT_0* = (0x00000001 shl RTC_ALRMAR_HT_Pos) # !< 0x00100000
  RTC_ALRMAR_HT_1* = (0x00000002 shl RTC_ALRMAR_HT_Pos) # !< 0x00200000
  RTC_ALRMAR_HU_Pos* = (16)
  RTC_ALRMAR_HU_Msk* = (0x0000000F shl RTC_ALRMAR_HU_Pos) # !< 0x000F0000
  RTC_ALRMAR_HU* = RTC_ALRMAR_HU_Msk
  RTC_ALRMAR_HU_0* = (0x00000001 shl RTC_ALRMAR_HU_Pos) # !< 0x00010000
  RTC_ALRMAR_HU_1* = (0x00000002 shl RTC_ALRMAR_HU_Pos) # !< 0x00020000
  RTC_ALRMAR_HU_2* = (0x00000004 shl RTC_ALRMAR_HU_Pos) # !< 0x00040000
  RTC_ALRMAR_HU_3* = (0x00000008 shl RTC_ALRMAR_HU_Pos) # !< 0x00080000
  RTC_ALRMAR_MSK2_Pos* = (15)
  RTC_ALRMAR_MSK2_Msk* = (0x00000001 shl RTC_ALRMAR_MSK2_Pos) # !< 0x00008000
  RTC_ALRMAR_MSK2* = RTC_ALRMAR_MSK2_Msk
  RTC_ALRMAR_MNT_Pos* = (12)
  RTC_ALRMAR_MNT_Msk* = (0x00000007 shl RTC_ALRMAR_MNT_Pos) # !< 0x00007000
  RTC_ALRMAR_MNT* = RTC_ALRMAR_MNT_Msk
  RTC_ALRMAR_MNT_0* = (0x00000001 shl RTC_ALRMAR_MNT_Pos) # !< 0x00001000
  RTC_ALRMAR_MNT_1* = (0x00000002 shl RTC_ALRMAR_MNT_Pos) # !< 0x00002000
  RTC_ALRMAR_MNT_2* = (0x00000004 shl RTC_ALRMAR_MNT_Pos) # !< 0x00004000
  RTC_ALRMAR_MNU_Pos* = (8)
  RTC_ALRMAR_MNU_Msk* = (0x0000000F shl RTC_ALRMAR_MNU_Pos) # !< 0x00000F00
  RTC_ALRMAR_MNU* = RTC_ALRMAR_MNU_Msk
  RTC_ALRMAR_MNU_0* = (0x00000001 shl RTC_ALRMAR_MNU_Pos) # !< 0x00000100
  RTC_ALRMAR_MNU_1* = (0x00000002 shl RTC_ALRMAR_MNU_Pos) # !< 0x00000200
  RTC_ALRMAR_MNU_2* = (0x00000004 shl RTC_ALRMAR_MNU_Pos) # !< 0x00000400
  RTC_ALRMAR_MNU_3* = (0x00000008 shl RTC_ALRMAR_MNU_Pos) # !< 0x00000800
  RTC_ALRMAR_MSK1_Pos* = (7)
  RTC_ALRMAR_MSK1_Msk* = (0x00000001 shl RTC_ALRMAR_MSK1_Pos) # !< 0x00000080
  RTC_ALRMAR_MSK1* = RTC_ALRMAR_MSK1_Msk
  RTC_ALRMAR_ST_Pos* = (4)
  RTC_ALRMAR_ST_Msk* = (0x00000007 shl RTC_ALRMAR_ST_Pos) # !< 0x00000070
  RTC_ALRMAR_ST* = RTC_ALRMAR_ST_Msk
  RTC_ALRMAR_ST_0* = (0x00000001 shl RTC_ALRMAR_ST_Pos) # !< 0x00000010
  RTC_ALRMAR_ST_1* = (0x00000002 shl RTC_ALRMAR_ST_Pos) # !< 0x00000020
  RTC_ALRMAR_ST_2* = (0x00000004 shl RTC_ALRMAR_ST_Pos) # !< 0x00000040
  RTC_ALRMAR_SU_Pos* = (0)
  RTC_ALRMAR_SU_Msk* = (0x0000000F shl RTC_ALRMAR_SU_Pos) # !< 0x0000000F
  RTC_ALRMAR_SU* = RTC_ALRMAR_SU_Msk
  RTC_ALRMAR_SU_0* = (0x00000001 shl RTC_ALRMAR_SU_Pos) # !< 0x00000001
  RTC_ALRMAR_SU_1* = (0x00000002 shl RTC_ALRMAR_SU_Pos) # !< 0x00000002
  RTC_ALRMAR_SU_2* = (0x00000004 shl RTC_ALRMAR_SU_Pos) # !< 0x00000004
  RTC_ALRMAR_SU_3* = (0x00000008 shl RTC_ALRMAR_SU_Pos) # !< 0x00000008

# *******************  Bits definition for RTC_WPR register  ****************

const
  RTC_WPR_KEY_Pos* = (0)
  RTC_WPR_KEY_Msk* = (0x000000FF shl RTC_WPR_KEY_Pos) # !< 0x000000FF
  RTC_WPR_KEY* = RTC_WPR_KEY_Msk

# *******************  Bits definition for RTC_SSR register  ****************

const
  RTC_SSR_SS_Pos* = (0)
  RTC_SSR_SS_Msk* = (0x0000FFFF shl RTC_SSR_SS_Pos) # !< 0x0000FFFF
  RTC_SSR_SS* = RTC_SSR_SS_Msk

# *******************  Bits definition for RTC_SHIFTR register  *************

const
  RTC_SHIFTR_SUBFS_Pos* = (0)
  RTC_SHIFTR_SUBFS_Msk* = (0x00007FFF shl RTC_SHIFTR_SUBFS_Pos) # !< 0x00007FFF
  RTC_SHIFTR_SUBFS* = RTC_SHIFTR_SUBFS_Msk
  RTC_SHIFTR_ADD1S_Pos* = (31)
  RTC_SHIFTR_ADD1S_Msk* = (0x00000001 shl RTC_SHIFTR_ADD1S_Pos) # !< 0x80000000
  RTC_SHIFTR_ADD1S* = RTC_SHIFTR_ADD1S_Msk

# *******************  Bits definition for RTC_TSTR register  ***************

const
  RTC_TSTR_PM_Pos* = (22)
  RTC_TSTR_PM_Msk* = (0x00000001 shl RTC_TSTR_PM_Pos) # !< 0x00400000
  RTC_TSTR_PM* = RTC_TSTR_PM_Msk
  RTC_TSTR_HT_Pos* = (20)
  RTC_TSTR_HT_Msk* = (0x00000003 shl RTC_TSTR_HT_Pos) # !< 0x00300000
  RTC_TSTR_HT* = RTC_TSTR_HT_Msk
  RTC_TSTR_HT_0* = (0x00000001 shl RTC_TSTR_HT_Pos) # !< 0x00100000
  RTC_TSTR_HT_1* = (0x00000002 shl RTC_TSTR_HT_Pos) # !< 0x00200000
  RTC_TSTR_HU_Pos* = (16)
  RTC_TSTR_HU_Msk* = (0x0000000F shl RTC_TSTR_HU_Pos) # !< 0x000F0000
  RTC_TSTR_HU* = RTC_TSTR_HU_Msk
  RTC_TSTR_HU_0* = (0x00000001 shl RTC_TSTR_HU_Pos) # !< 0x00010000
  RTC_TSTR_HU_1* = (0x00000002 shl RTC_TSTR_HU_Pos) # !< 0x00020000
  RTC_TSTR_HU_2* = (0x00000004 shl RTC_TSTR_HU_Pos) # !< 0x00040000
  RTC_TSTR_HU_3* = (0x00000008 shl RTC_TSTR_HU_Pos) # !< 0x00080000
  RTC_TSTR_MNT_Pos* = (12)
  RTC_TSTR_MNT_Msk* = (0x00000007 shl RTC_TSTR_MNT_Pos) # !< 0x00007000
  RTC_TSTR_MNT* = RTC_TSTR_MNT_Msk
  RTC_TSTR_MNT_0* = (0x00000001 shl RTC_TSTR_MNT_Pos) # !< 0x00001000
  RTC_TSTR_MNT_1* = (0x00000002 shl RTC_TSTR_MNT_Pos) # !< 0x00002000
  RTC_TSTR_MNT_2* = (0x00000004 shl RTC_TSTR_MNT_Pos) # !< 0x00004000
  RTC_TSTR_MNU_Pos* = (8)
  RTC_TSTR_MNU_Msk* = (0x0000000F shl RTC_TSTR_MNU_Pos) # !< 0x00000F00
  RTC_TSTR_MNU* = RTC_TSTR_MNU_Msk
  RTC_TSTR_MNU_0* = (0x00000001 shl RTC_TSTR_MNU_Pos) # !< 0x00000100
  RTC_TSTR_MNU_1* = (0x00000002 shl RTC_TSTR_MNU_Pos) # !< 0x00000200
  RTC_TSTR_MNU_2* = (0x00000004 shl RTC_TSTR_MNU_Pos) # !< 0x00000400
  RTC_TSTR_MNU_3* = (0x00000008 shl RTC_TSTR_MNU_Pos) # !< 0x00000800
  RTC_TSTR_ST_Pos* = (4)
  RTC_TSTR_ST_Msk* = (0x00000007 shl RTC_TSTR_ST_Pos) # !< 0x00000070
  RTC_TSTR_ST* = RTC_TSTR_ST_Msk
  RTC_TSTR_ST_0* = (0x00000001 shl RTC_TSTR_ST_Pos) # !< 0x00000010
  RTC_TSTR_ST_1* = (0x00000002 shl RTC_TSTR_ST_Pos) # !< 0x00000020
  RTC_TSTR_ST_2* = (0x00000004 shl RTC_TSTR_ST_Pos) # !< 0x00000040
  RTC_TSTR_SU_Pos* = (0)
  RTC_TSTR_SU_Msk* = (0x0000000F shl RTC_TSTR_SU_Pos) # !< 0x0000000F
  RTC_TSTR_SU* = RTC_TSTR_SU_Msk
  RTC_TSTR_SU_0* = (0x00000001 shl RTC_TSTR_SU_Pos) # !< 0x00000001
  RTC_TSTR_SU_1* = (0x00000002 shl RTC_TSTR_SU_Pos) # !< 0x00000002
  RTC_TSTR_SU_2* = (0x00000004 shl RTC_TSTR_SU_Pos) # !< 0x00000004
  RTC_TSTR_SU_3* = (0x00000008 shl RTC_TSTR_SU_Pos) # !< 0x00000008

# *******************  Bits definition for RTC_TSDR register  ***************

const
  RTC_TSDR_WDU_Pos* = (13)
  RTC_TSDR_WDU_Msk* = (0x00000007 shl RTC_TSDR_WDU_Pos) # !< 0x0000E000
  RTC_TSDR_WDU* = RTC_TSDR_WDU_Msk
  RTC_TSDR_WDU_0* = (0x00000001 shl RTC_TSDR_WDU_Pos) # !< 0x00002000
  RTC_TSDR_WDU_1* = (0x00000002 shl RTC_TSDR_WDU_Pos) # !< 0x00004000
  RTC_TSDR_WDU_2* = (0x00000004 shl RTC_TSDR_WDU_Pos) # !< 0x00008000
  RTC_TSDR_MT_Pos* = (12)
  RTC_TSDR_MT_Msk* = (0x00000001 shl RTC_TSDR_MT_Pos) # !< 0x00001000
  RTC_TSDR_MT* = RTC_TSDR_MT_Msk
  RTC_TSDR_MU_Pos* = (8)
  RTC_TSDR_MU_Msk* = (0x0000000F shl RTC_TSDR_MU_Pos) # !< 0x00000F00
  RTC_TSDR_MU* = RTC_TSDR_MU_Msk
  RTC_TSDR_MU_0* = (0x00000001 shl RTC_TSDR_MU_Pos) # !< 0x00000100
  RTC_TSDR_MU_1* = (0x00000002 shl RTC_TSDR_MU_Pos) # !< 0x00000200
  RTC_TSDR_MU_2* = (0x00000004 shl RTC_TSDR_MU_Pos) # !< 0x00000400
  RTC_TSDR_MU_3* = (0x00000008 shl RTC_TSDR_MU_Pos) # !< 0x00000800
  RTC_TSDR_DT_Pos* = (4)
  RTC_TSDR_DT_Msk* = (0x00000003 shl RTC_TSDR_DT_Pos) # !< 0x00000030
  RTC_TSDR_DT* = RTC_TSDR_DT_Msk
  RTC_TSDR_DT_0* = (0x00000001 shl RTC_TSDR_DT_Pos) # !< 0x00000010
  RTC_TSDR_DT_1* = (0x00000002 shl RTC_TSDR_DT_Pos) # !< 0x00000020
  RTC_TSDR_DU_Pos* = (0)
  RTC_TSDR_DU_Msk* = (0x0000000F shl RTC_TSDR_DU_Pos) # !< 0x0000000F
  RTC_TSDR_DU* = RTC_TSDR_DU_Msk
  RTC_TSDR_DU_0* = (0x00000001 shl RTC_TSDR_DU_Pos) # !< 0x00000001
  RTC_TSDR_DU_1* = (0x00000002 shl RTC_TSDR_DU_Pos) # !< 0x00000002
  RTC_TSDR_DU_2* = (0x00000004 shl RTC_TSDR_DU_Pos) # !< 0x00000004
  RTC_TSDR_DU_3* = (0x00000008 shl RTC_TSDR_DU_Pos) # !< 0x00000008

# *******************  Bits definition for RTC_TSSSR register  **************

const
  RTC_TSSSR_SS_Pos* = (0)
  RTC_TSSSR_SS_Msk* = (0x0000FFFF shl RTC_TSSSR_SS_Pos) # !< 0x0000FFFF
  RTC_TSSSR_SS* = RTC_TSSSR_SS_Msk

# *******************  Bits definition for RTC_CALR register  ***************

const
  RTC_CALR_CALP_Pos* = (15)
  RTC_CALR_CALP_Msk* = (0x00000001 shl RTC_CALR_CALP_Pos) # !< 0x00008000
  RTC_CALR_CALP* = RTC_CALR_CALP_Msk
  RTC_CALR_CALW8_Pos* = (14)
  RTC_CALR_CALW8_Msk* = (0x00000001 shl RTC_CALR_CALW8_Pos) # !< 0x00004000
  RTC_CALR_CALW8* = RTC_CALR_CALW8_Msk
  RTC_CALR_CALW16_Pos* = (13)
  RTC_CALR_CALW16_Msk* = (0x00000001 shl RTC_CALR_CALW16_Pos) # !< 0x00002000
  RTC_CALR_CALW16* = RTC_CALR_CALW16_Msk
  RTC_CALR_CALM_Pos* = (0)
  RTC_CALR_CALM_Msk* = (0x000001FF shl RTC_CALR_CALM_Pos) # !< 0x000001FF
  RTC_CALR_CALM* = RTC_CALR_CALM_Msk
  RTC_CALR_CALM_0* = (0x00000001 shl RTC_CALR_CALM_Pos) # !< 0x00000001
  RTC_CALR_CALM_1* = (0x00000002 shl RTC_CALR_CALM_Pos) # !< 0x00000002
  RTC_CALR_CALM_2* = (0x00000004 shl RTC_CALR_CALM_Pos) # !< 0x00000004
  RTC_CALR_CALM_3* = (0x00000008 shl RTC_CALR_CALM_Pos) # !< 0x00000008
  RTC_CALR_CALM_4* = (0x00000010 shl RTC_CALR_CALM_Pos) # !< 0x00000010
  RTC_CALR_CALM_5* = (0x00000020 shl RTC_CALR_CALM_Pos) # !< 0x00000020
  RTC_CALR_CALM_6* = (0x00000040 shl RTC_CALR_CALM_Pos) # !< 0x00000040
  RTC_CALR_CALM_7* = (0x00000080 shl RTC_CALR_CALM_Pos) # !< 0x00000080
  RTC_CALR_CALM_8* = (0x00000100 shl RTC_CALR_CALM_Pos) # !< 0x00000100

# *******************  Bits definition for RTC_TAFCR register  **************

const
  RTC_TAFCR_PC15MODE_Pos* = (23)
  RTC_TAFCR_PC15MODE_Msk* = (0x00000001 shl RTC_TAFCR_PC15MODE_Pos) # !< 0x00800000
  RTC_TAFCR_PC15MODE* = RTC_TAFCR_PC15MODE_Msk
  RTC_TAFCR_PC15VALUE_Pos* = (22)
  RTC_TAFCR_PC15VALUE_Msk* = (0x00000001 shl RTC_TAFCR_PC15VALUE_Pos) # !< 0x00400000
  RTC_TAFCR_PC15VALUE* = RTC_TAFCR_PC15VALUE_Msk
  RTC_TAFCR_PC14MODE_Pos* = (21)
  RTC_TAFCR_PC14MODE_Msk* = (0x00000001 shl RTC_TAFCR_PC14MODE_Pos) # !< 0x00200000
  RTC_TAFCR_PC14MODE* = RTC_TAFCR_PC14MODE_Msk
  RTC_TAFCR_PC14VALUE_Pos* = (20)
  RTC_TAFCR_PC14VALUE_Msk* = (0x00000001 shl RTC_TAFCR_PC14VALUE_Pos) # !< 0x00100000
  RTC_TAFCR_PC14VALUE* = RTC_TAFCR_PC14VALUE_Msk
  RTC_TAFCR_PC13MODE_Pos* = (19)
  RTC_TAFCR_PC13MODE_Msk* = (0x00000001 shl RTC_TAFCR_PC13MODE_Pos) # !< 0x00080000
  RTC_TAFCR_PC13MODE* = RTC_TAFCR_PC13MODE_Msk
  RTC_TAFCR_PC13VALUE_Pos* = (18)
  RTC_TAFCR_PC13VALUE_Msk* = (0x00000001 shl RTC_TAFCR_PC13VALUE_Pos) # !< 0x00040000
  RTC_TAFCR_PC13VALUE* = RTC_TAFCR_PC13VALUE_Msk
  RTC_TAFCR_TAMPPUDIS_Pos* = (15)
  RTC_TAFCR_TAMPPUDIS_Msk* = (0x00000001 shl RTC_TAFCR_TAMPPUDIS_Pos) # !< 0x00008000
  RTC_TAFCR_TAMPPUDIS* = RTC_TAFCR_TAMPPUDIS_Msk
  RTC_TAFCR_TAMPPRCH_Pos* = (13)
  RTC_TAFCR_TAMPPRCH_Msk* = (0x00000003 shl RTC_TAFCR_TAMPPRCH_Pos) # !< 0x00006000
  RTC_TAFCR_TAMPPRCH* = RTC_TAFCR_TAMPPRCH_Msk
  RTC_TAFCR_TAMPPRCH_0* = (0x00000001 shl RTC_TAFCR_TAMPPRCH_Pos) # !< 0x00002000
  RTC_TAFCR_TAMPPRCH_1* = (0x00000002 shl RTC_TAFCR_TAMPPRCH_Pos) # !< 0x00004000
  RTC_TAFCR_TAMPFLT_Pos* = (11)
  RTC_TAFCR_TAMPFLT_Msk* = (0x00000003 shl RTC_TAFCR_TAMPFLT_Pos) # !< 0x00001800
  RTC_TAFCR_TAMPFLT* = RTC_TAFCR_TAMPFLT_Msk
  RTC_TAFCR_TAMPFLT_0* = (0x00000001 shl RTC_TAFCR_TAMPFLT_Pos) # !< 0x00000800
  RTC_TAFCR_TAMPFLT_1* = (0x00000002 shl RTC_TAFCR_TAMPFLT_Pos) # !< 0x00001000
  RTC_TAFCR_TAMPFREQ_Pos* = (8)
  RTC_TAFCR_TAMPFREQ_Msk* = (0x00000007 shl RTC_TAFCR_TAMPFREQ_Pos) # !< 0x00000700
  RTC_TAFCR_TAMPFREQ* = RTC_TAFCR_TAMPFREQ_Msk
  RTC_TAFCR_TAMPFREQ_0* = (0x00000001 shl RTC_TAFCR_TAMPFREQ_Pos) # !< 0x00000100
  RTC_TAFCR_TAMPFREQ_1* = (0x00000002 shl RTC_TAFCR_TAMPFREQ_Pos) # !< 0x00000200
  RTC_TAFCR_TAMPFREQ_2* = (0x00000004 shl RTC_TAFCR_TAMPFREQ_Pos) # !< 0x00000400
  RTC_TAFCR_TAMPTS_Pos* = (7)
  RTC_TAFCR_TAMPTS_Msk* = (0x00000001 shl RTC_TAFCR_TAMPTS_Pos) # !< 0x00000080
  RTC_TAFCR_TAMPTS* = RTC_TAFCR_TAMPTS_Msk
  RTC_TAFCR_TAMP2TRG_Pos* = (4)
  RTC_TAFCR_TAMP2TRG_Msk* = (0x00000001 shl RTC_TAFCR_TAMP2TRG_Pos) # !< 0x00000010
  RTC_TAFCR_TAMP2TRG* = RTC_TAFCR_TAMP2TRG_Msk
  RTC_TAFCR_TAMP2E_Pos* = (3)
  RTC_TAFCR_TAMP2E_Msk* = (0x00000001 shl RTC_TAFCR_TAMP2E_Pos) # !< 0x00000008
  RTC_TAFCR_TAMP2E* = RTC_TAFCR_TAMP2E_Msk
  RTC_TAFCR_TAMPIE_Pos* = (2)
  RTC_TAFCR_TAMPIE_Msk* = (0x00000001 shl RTC_TAFCR_TAMPIE_Pos) # !< 0x00000004
  RTC_TAFCR_TAMPIE* = RTC_TAFCR_TAMPIE_Msk
  RTC_TAFCR_TAMP1TRG_Pos* = (1)
  RTC_TAFCR_TAMP1TRG_Msk* = (0x00000001 shl RTC_TAFCR_TAMP1TRG_Pos) # !< 0x00000002
  RTC_TAFCR_TAMP1TRG* = RTC_TAFCR_TAMP1TRG_Msk
  RTC_TAFCR_TAMP1E_Pos* = (0)
  RTC_TAFCR_TAMP1E_Msk* = (0x00000001 shl RTC_TAFCR_TAMP1E_Pos) # !< 0x00000001
  RTC_TAFCR_TAMP1E* = RTC_TAFCR_TAMP1E_Msk

#  Reference defines

const
  RTC_TAFCR_ALARMOUTTYPE* = RTC_TAFCR_PC13VALUE

# *******************  Bits definition for RTC_ALRMASSR register  ***********

const
  RTC_ALRMASSR_MASKSS_Pos* = (24)
  RTC_ALRMASSR_MASKSS_Msk* = (0x0000000F shl RTC_ALRMASSR_MASKSS_Pos) # !< 0x0F000000
  RTC_ALRMASSR_MASKSS* = RTC_ALRMASSR_MASKSS_Msk
  RTC_ALRMASSR_MASKSS_0* = (0x00000001 shl RTC_ALRMASSR_MASKSS_Pos) # !< 0x01000000
  RTC_ALRMASSR_MASKSS_1* = (0x00000002 shl RTC_ALRMASSR_MASKSS_Pos) # !< 0x02000000
  RTC_ALRMASSR_MASKSS_2* = (0x00000004 shl RTC_ALRMASSR_MASKSS_Pos) # !< 0x04000000
  RTC_ALRMASSR_MASKSS_3* = (0x00000008 shl RTC_ALRMASSR_MASKSS_Pos) # !< 0x08000000
  RTC_ALRMASSR_SS_Pos* = (0)
  RTC_ALRMASSR_SS_Msk* = (0x00007FFF shl RTC_ALRMASSR_SS_Pos) # !< 0x00007FFF
  RTC_ALRMASSR_SS* = RTC_ALRMASSR_SS_Msk

# ***************************************************************************
##
#                         Serial Peripheral Interface (SPI)
##
# ***************************************************************************
##
#  @brief Specific device feature definitions (not present on all devices in the STM32F0 serie)
##
#  Note: No specific macro feature on this device
# ******************  Bit definition for SPI_CR1 register  ******************

const
  SPI_CR1_CPHA_Pos* = (0)
  SPI_CR1_CPHA_Msk* = (0x00000001 shl SPI_CR1_CPHA_Pos) # !< 0x00000001
  SPI_CR1_CPHA* = SPI_CR1_CPHA_Msk
  SPI_CR1_CPOL_Pos* = (1)
  SPI_CR1_CPOL_Msk* = (0x00000001 shl SPI_CR1_CPOL_Pos) # !< 0x00000002
  SPI_CR1_CPOL* = SPI_CR1_CPOL_Msk
  SPI_CR1_MSTR_Pos* = (2)
  SPI_CR1_MSTR_Msk* = (0x00000001 shl SPI_CR1_MSTR_Pos) # !< 0x00000004
  SPI_CR1_MSTR* = SPI_CR1_MSTR_Msk
  SPI_CR1_BR_Pos* = (3)
  SPI_CR1_BR_Msk* = (0x00000007 shl SPI_CR1_BR_Pos) # !< 0x00000038
  SPI_CR1_BR* = SPI_CR1_BR_Msk
  SPI_CR1_BR_0* = (0x00000001 shl SPI_CR1_BR_Pos) # !< 0x00000008
  SPI_CR1_BR_1* = (0x00000002 shl SPI_CR1_BR_Pos) # !< 0x00000010
  SPI_CR1_BR_2* = (0x00000004 shl SPI_CR1_BR_Pos) # !< 0x00000020
  SPI_CR1_SPE_Pos* = (6)
  SPI_CR1_SPE_Msk* = (0x00000001 shl SPI_CR1_SPE_Pos) # !< 0x00000040
  SPI_CR1_SPE* = SPI_CR1_SPE_Msk
  SPI_CR1_LSBFIRST_Pos* = (7)
  SPI_CR1_LSBFIRST_Msk* = (0x00000001 shl SPI_CR1_LSBFIRST_Pos) # !< 0x00000080
  SPI_CR1_LSBFIRST* = SPI_CR1_LSBFIRST_Msk
  SPI_CR1_SSI_Pos* = (8)
  SPI_CR1_SSI_Msk* = (0x00000001 shl SPI_CR1_SSI_Pos) # !< 0x00000100
  SPI_CR1_SSI* = SPI_CR1_SSI_Msk
  SPI_CR1_SSM_Pos* = (9)
  SPI_CR1_SSM_Msk* = (0x00000001 shl SPI_CR1_SSM_Pos) # !< 0x00000200
  SPI_CR1_SSM* = SPI_CR1_SSM_Msk
  SPI_CR1_RXONLY_Pos* = (10)
  SPI_CR1_RXONLY_Msk* = (0x00000001 shl SPI_CR1_RXONLY_Pos) # !< 0x00000400
  SPI_CR1_RXONLY* = SPI_CR1_RXONLY_Msk
  SPI_CR1_CRCL_Pos* = (11)
  SPI_CR1_CRCL_Msk* = (0x00000001 shl SPI_CR1_CRCL_Pos) # !< 0x00000800
  SPI_CR1_CRCL* = SPI_CR1_CRCL_Msk
  SPI_CR1_CRCNEXT_Pos* = (12)
  SPI_CR1_CRCNEXT_Msk* = (0x00000001 shl SPI_CR1_CRCNEXT_Pos) # !< 0x00001000
  SPI_CR1_CRCNEXT* = SPI_CR1_CRCNEXT_Msk
  SPI_CR1_CRCEN_Pos* = (13)
  SPI_CR1_CRCEN_Msk* = (0x00000001 shl SPI_CR1_CRCEN_Pos) # !< 0x00002000
  SPI_CR1_CRCEN* = SPI_CR1_CRCEN_Msk
  SPI_CR1_BIDIOE_Pos* = (14)
  SPI_CR1_BIDIOE_Msk* = (0x00000001 shl SPI_CR1_BIDIOE_Pos) # !< 0x00004000
  SPI_CR1_BIDIOE* = SPI_CR1_BIDIOE_Msk
  SPI_CR1_BIDIMODE_Pos* = (15)
  SPI_CR1_BIDIMODE_Msk* = (0x00000001 shl SPI_CR1_BIDIMODE_Pos) # !< 0x00008000
  SPI_CR1_BIDIMODE* = SPI_CR1_BIDIMODE_Msk

# ******************  Bit definition for SPI_CR2 register  ******************

const
  SPI_CR2_RXDMAEN_Pos* = (0)
  SPI_CR2_RXDMAEN_Msk* = (0x00000001 shl SPI_CR2_RXDMAEN_Pos) # !< 0x00000001
  SPI_CR2_RXDMAEN* = SPI_CR2_RXDMAEN_Msk
  SPI_CR2_TXDMAEN_Pos* = (1)
  SPI_CR2_TXDMAEN_Msk* = (0x00000001 shl SPI_CR2_TXDMAEN_Pos) # !< 0x00000002
  SPI_CR2_TXDMAEN* = SPI_CR2_TXDMAEN_Msk
  SPI_CR2_SSOE_Pos* = (2)
  SPI_CR2_SSOE_Msk* = (0x00000001 shl SPI_CR2_SSOE_Pos) # !< 0x00000004
  SPI_CR2_SSOE* = SPI_CR2_SSOE_Msk
  SPI_CR2_NSSP_Pos* = (3)
  SPI_CR2_NSSP_Msk* = (0x00000001 shl SPI_CR2_NSSP_Pos) # !< 0x00000008
  SPI_CR2_NSSP* = SPI_CR2_NSSP_Msk
  SPI_CR2_FRF_Pos* = (4)
  SPI_CR2_FRF_Msk* = (0x00000001 shl SPI_CR2_FRF_Pos) # !< 0x00000010
  SPI_CR2_FRF* = SPI_CR2_FRF_Msk
  SPI_CR2_ERRIE_Pos* = (5)
  SPI_CR2_ERRIE_Msk* = (0x00000001 shl SPI_CR2_ERRIE_Pos) # !< 0x00000020
  SPI_CR2_ERRIE* = SPI_CR2_ERRIE_Msk
  SPI_CR2_RXNEIE_Pos* = (6)
  SPI_CR2_RXNEIE_Msk* = (0x00000001 shl SPI_CR2_RXNEIE_Pos) # !< 0x00000040
  SPI_CR2_RXNEIE* = SPI_CR2_RXNEIE_Msk
  SPI_CR2_TXEIE_Pos* = (7)
  SPI_CR2_TXEIE_Msk* = (0x00000001 shl SPI_CR2_TXEIE_Pos) # !< 0x00000080
  SPI_CR2_TXEIE* = SPI_CR2_TXEIE_Msk
  SPI_CR2_DS_Pos* = (8)
  SPI_CR2_DS_Msk* = (0x0000000F shl SPI_CR2_DS_Pos) # !< 0x00000F00
  SPI_CR2_DS* = SPI_CR2_DS_Msk
  SPI_CR2_DS_0* = (0x00000001 shl SPI_CR2_DS_Pos) # !< 0x00000100
  SPI_CR2_DS_1* = (0x00000002 shl SPI_CR2_DS_Pos) # !< 0x00000200
  SPI_CR2_DS_2* = (0x00000004 shl SPI_CR2_DS_Pos) # !< 0x00000400
  SPI_CR2_DS_3* = (0x00000008 shl SPI_CR2_DS_Pos) # !< 0x00000800
  SPI_CR2_FRXTH_Pos* = (12)
  SPI_CR2_FRXTH_Msk* = (0x00000001 shl SPI_CR2_FRXTH_Pos) # !< 0x00001000
  SPI_CR2_FRXTH* = SPI_CR2_FRXTH_Msk
  SPI_CR2_LDMARX_Pos* = (13)
  SPI_CR2_LDMARX_Msk* = (0x00000001 shl SPI_CR2_LDMARX_Pos) # !< 0x00002000
  SPI_CR2_LDMARX* = SPI_CR2_LDMARX_Msk
  SPI_CR2_LDMATX_Pos* = (14)
  SPI_CR2_LDMATX_Msk* = (0x00000001 shl SPI_CR2_LDMATX_Pos) # !< 0x00004000
  SPI_CR2_LDMATX* = SPI_CR2_LDMATX_Msk

# *******************  Bit definition for SPI_SR register  ******************

const
  SPI_SR_RXNE_Pos* = (0)
  SPI_SR_RXNE_Msk* = (0x00000001 shl SPI_SR_RXNE_Pos) # !< 0x00000001
  SPI_SR_RXNE* = SPI_SR_RXNE_Msk
  SPI_SR_TXE_Pos* = (1)
  SPI_SR_TXE_Msk* = (0x00000001 shl SPI_SR_TXE_Pos) # !< 0x00000002
  SPI_SR_TXE* = SPI_SR_TXE_Msk
  SPI_SR_CRCERR_Pos* = (4)
  SPI_SR_CRCERR_Msk* = (0x00000001 shl SPI_SR_CRCERR_Pos) # !< 0x00000010
  SPI_SR_CRCERR* = SPI_SR_CRCERR_Msk
  SPI_SR_MODF_Pos* = (5)
  SPI_SR_MODF_Msk* = (0x00000001 shl SPI_SR_MODF_Pos) # !< 0x00000020
  SPI_SR_MODF* = SPI_SR_MODF_Msk
  SPI_SR_OVR_Pos* = (6)
  SPI_SR_OVR_Msk* = (0x00000001 shl SPI_SR_OVR_Pos) # !< 0x00000040
  SPI_SR_OVR* = SPI_SR_OVR_Msk
  SPI_SR_BSY_Pos* = (7)
  SPI_SR_BSY_Msk* = (0x00000001 shl SPI_SR_BSY_Pos) # !< 0x00000080
  SPI_SR_BSY* = SPI_SR_BSY_Msk
  SPI_SR_FRE_Pos* = (8)
  SPI_SR_FRE_Msk* = (0x00000001 shl SPI_SR_FRE_Pos) # !< 0x00000100
  SPI_SR_FRE* = SPI_SR_FRE_Msk
  SPI_SR_FRLVL_Pos* = (9)
  SPI_SR_FRLVL_Msk* = (0x00000003 shl SPI_SR_FRLVL_Pos) # !< 0x00000600
  SPI_SR_FRLVL* = SPI_SR_FRLVL_Msk
  SPI_SR_FRLVL_0* = (0x00000001 shl SPI_SR_FRLVL_Pos) # !< 0x00000200
  SPI_SR_FRLVL_1* = (0x00000002 shl SPI_SR_FRLVL_Pos) # !< 0x00000400
  SPI_SR_FTLVL_Pos* = (11)
  SPI_SR_FTLVL_Msk* = (0x00000003 shl SPI_SR_FTLVL_Pos) # !< 0x00001800
  SPI_SR_FTLVL* = SPI_SR_FTLVL_Msk
  SPI_SR_FTLVL_0* = (0x00000001 shl SPI_SR_FTLVL_Pos) # !< 0x00000800
  SPI_SR_FTLVL_1* = (0x00000002 shl SPI_SR_FTLVL_Pos) # !< 0x00001000

# *******************  Bit definition for SPI_DR register  ******************

const
  SPI_DR_DR_Pos* = (0)
  SPI_DR_DR_Msk* = (0xFFFFFFFF shl SPI_DR_DR_Pos) # !< 0xFFFFFFFF
  SPI_DR_DR* = SPI_DR_DR_Msk

# ******************  Bit definition for SPI_CRCPR register  ****************

const
  SPI_CRCPR_CRCPOLY_Pos* = (0)
  SPI_CRCPR_CRCPOLY_Msk* = (0xFFFFFFFF shl SPI_CRCPR_CRCPOLY_Pos) # !< 0xFFFFFFFF
  SPI_CRCPR_CRCPOLY* = SPI_CRCPR_CRCPOLY_Msk

# *****************  Bit definition for SPI_RXCRCR register  ****************

const
  SPI_RXCRCR_RXCRC_Pos* = (0)
  SPI_RXCRCR_RXCRC_Msk* = (0xFFFFFFFF shl SPI_RXCRCR_RXCRC_Pos) # !< 0xFFFFFFFF
  SPI_RXCRCR_RXCRC* = SPI_RXCRCR_RXCRC_Msk

# *****************  Bit definition for SPI_TXCRCR register  ****************

const
  SPI_TXCRCR_TXCRC_Pos* = (0)
  SPI_TXCRCR_TXCRC_Msk* = (0xFFFFFFFF shl SPI_TXCRCR_TXCRC_Pos) # !< 0xFFFFFFFF
  SPI_TXCRCR_TXCRC* = SPI_TXCRCR_TXCRC_Msk

# *****************  Bit definition for SPI_I2SCFGR register  ***************

const
  SPI_I2SCFGR_I2SMOD_Pos* = (11)
  SPI_I2SCFGR_I2SMOD_Msk* = (0x00000001 shl SPI_I2SCFGR_I2SMOD_Pos) # !< 0x00000800
  SPI_I2SCFGR_I2SMOD* = SPI_I2SCFGR_I2SMOD_Msk

# ***************************************************************************
##
#                        System Configuration (SYSCFG)
##
# ***************************************************************************
# ****************  Bit definition for SYSCFG_CFGR1 register  ***************

const
  SYSCFG_CFGR1_MEM_MODE_Pos* = (0)
  SYSCFG_CFGR1_MEM_MODE_Msk* = (0x00000003 shl SYSCFG_CFGR1_MEM_MODE_Pos) # !< 0x00000003
  SYSCFG_CFGR1_MEM_MODE* = SYSCFG_CFGR1_MEM_MODE_Msk
  SYSCFG_CFGR1_MEM_MODE_0* = (0x00000001 shl SYSCFG_CFGR1_MEM_MODE_Pos) # !< 0x00000001
  SYSCFG_CFGR1_MEM_MODE_1* = (0x00000002 shl SYSCFG_CFGR1_MEM_MODE_Pos) # !< 0x00000002
  SYSCFG_CFGR1_DMA_RMP_Pos* = (8)
  SYSCFG_CFGR1_DMA_RMP_Msk* = (0x0000001F shl SYSCFG_CFGR1_DMA_RMP_Pos) # !< 0x00001F00
  SYSCFG_CFGR1_DMA_RMP* = SYSCFG_CFGR1_DMA_RMP_Msk
  SYSCFG_CFGR1_ADC_DMA_RMP_Pos* = (8)
  SYSCFG_CFGR1_ADC_DMA_RMP_Msk* = (0x00000001 shl SYSCFG_CFGR1_ADC_DMA_RMP_Pos) # !< 0x00000100
  SYSCFG_CFGR1_ADC_DMA_RMP* = SYSCFG_CFGR1_ADC_DMA_RMP_Msk
  SYSCFG_CFGR1_USART1TX_DMA_RMP_Pos* = (9)
  SYSCFG_CFGR1_USART1TX_DMA_RMP_Msk* = (
    0x00000001 shl SYSCFG_CFGR1_USART1TX_DMA_RMP_Pos) # !< 0x00000200
  SYSCFG_CFGR1_USART1TX_DMA_RMP* = SYSCFG_CFGR1_USART1TX_DMA_RMP_Msk
  SYSCFG_CFGR1_USART1RX_DMA_RMP_Pos* = (10)
  SYSCFG_CFGR1_USART1RX_DMA_RMP_Msk* = (
    0x00000001 shl SYSCFG_CFGR1_USART1RX_DMA_RMP_Pos) # !< 0x00000400
  SYSCFG_CFGR1_USART1RX_DMA_RMP* = SYSCFG_CFGR1_USART1RX_DMA_RMP_Msk
  SYSCFG_CFGR1_TIM16_DMA_RMP_Pos* = (11)
  SYSCFG_CFGR1_TIM16_DMA_RMP_Msk* = (0x00000001 shl
      SYSCFG_CFGR1_TIM16_DMA_RMP_Pos) # !< 0x00000800
  SYSCFG_CFGR1_TIM16_DMA_RMP* = SYSCFG_CFGR1_TIM16_DMA_RMP_Msk
  SYSCFG_CFGR1_TIM17_DMA_RMP_Pos* = (12)
  SYSCFG_CFGR1_TIM17_DMA_RMP_Msk* = (0x00000001 shl
      SYSCFG_CFGR1_TIM17_DMA_RMP_Pos) # !< 0x00001000
  SYSCFG_CFGR1_TIM17_DMA_RMP* = SYSCFG_CFGR1_TIM17_DMA_RMP_Msk
  SYSCFG_CFGR1_I2C_FMP_PB6_Pos* = (16)
  SYSCFG_CFGR1_I2C_FMP_PB6_Msk* = (0x00000001 shl SYSCFG_CFGR1_I2C_FMP_PB6_Pos) # !< 0x00010000
  SYSCFG_CFGR1_I2C_FMP_PB6* = SYSCFG_CFGR1_I2C_FMP_PB6_Msk
  SYSCFG_CFGR1_I2C_FMP_PB7_Pos* = (17)
  SYSCFG_CFGR1_I2C_FMP_PB7_Msk* = (0x00000001 shl SYSCFG_CFGR1_I2C_FMP_PB7_Pos) # !< 0x00020000
  SYSCFG_CFGR1_I2C_FMP_PB7* = SYSCFG_CFGR1_I2C_FMP_PB7_Msk
  SYSCFG_CFGR1_I2C_FMP_PB8_Pos* = (18)
  SYSCFG_CFGR1_I2C_FMP_PB8_Msk* = (0x00000001 shl SYSCFG_CFGR1_I2C_FMP_PB8_Pos) # !< 0x00040000
  SYSCFG_CFGR1_I2C_FMP_PB8* = SYSCFG_CFGR1_I2C_FMP_PB8_Msk
  SYSCFG_CFGR1_I2C_FMP_PB9_Pos* = (19)
  SYSCFG_CFGR1_I2C_FMP_PB9_Msk* = (0x00000001 shl SYSCFG_CFGR1_I2C_FMP_PB9_Pos) # !< 0x00080000
  SYSCFG_CFGR1_I2C_FMP_PB9* = SYSCFG_CFGR1_I2C_FMP_PB9_Msk

# ****************  Bit definition for SYSCFG_EXTICR1 register  *************

const
  SYSCFG_EXTICR1_EXTI0_Pos* = (0)
  SYSCFG_EXTICR1_EXTI0_Msk* = (0x0000000F shl SYSCFG_EXTICR1_EXTI0_Pos) # !< 0x0000000F
  SYSCFG_EXTICR1_EXTI0* = SYSCFG_EXTICR1_EXTI0_Msk
  SYSCFG_EXTICR1_EXTI1_Pos* = (4)
  SYSCFG_EXTICR1_EXTI1_Msk* = (0x0000000F shl SYSCFG_EXTICR1_EXTI1_Pos) # !< 0x000000F0
  SYSCFG_EXTICR1_EXTI1* = SYSCFG_EXTICR1_EXTI1_Msk
  SYSCFG_EXTICR1_EXTI2_Pos* = (8)
  SYSCFG_EXTICR1_EXTI2_Msk* = (0x0000000F shl SYSCFG_EXTICR1_EXTI2_Pos) # !< 0x00000F00
  SYSCFG_EXTICR1_EXTI2* = SYSCFG_EXTICR1_EXTI2_Msk
  SYSCFG_EXTICR1_EXTI3_Pos* = (12)
  SYSCFG_EXTICR1_EXTI3_Msk* = (0x0000000F shl SYSCFG_EXTICR1_EXTI3_Pos) # !< 0x0000F000
  SYSCFG_EXTICR1_EXTI3* = SYSCFG_EXTICR1_EXTI3_Msk

# *
#  @brief  EXTI0 configuration
##

const
  SYSCFG_EXTICR1_EXTI0_PA* = (0x00000000) # !< PA[0] pin
  SYSCFG_EXTICR1_EXTI0_PB* = (0x00000001) # !< PB[0] pin
  SYSCFG_EXTICR1_EXTI0_PC* = (0x00000002) # !< PC[0] pin
  SYSCFG_EXTICR1_EXTI0_PD* = (0x00000003) # !< PD[0] pin
  SYSCFG_EXTICR1_EXTI0_PF* = (0x00000005) # !< PF[0] pin

# *
#  @brief  EXTI1 configuration
##

const
  SYSCFG_EXTICR1_EXTI1_PA* = (0x00000000) # !< PA[1] pin
  SYSCFG_EXTICR1_EXTI1_PB* = (0x00000010) # !< PB[1] pin
  SYSCFG_EXTICR1_EXTI1_PC* = (0x00000020) # !< PC[1] pin
  SYSCFG_EXTICR1_EXTI1_PD* = (0x00000030) # !< PD[1] pin
  SYSCFG_EXTICR1_EXTI1_PF* = (0x00000050) # !< PF[1] pin

# *
#  @brief  EXTI2 configuration
##

const
  SYSCFG_EXTICR1_EXTI2_PA* = (0x00000000) # !< PA[2] pin
  SYSCFG_EXTICR1_EXTI2_PB* = (0x00000100) # !< PB[2] pin
  SYSCFG_EXTICR1_EXTI2_PC* = (0x00000200) # !< PC[2] pin
  SYSCFG_EXTICR1_EXTI2_PD* = (0x00000300) # !< PD[2] pin
  SYSCFG_EXTICR1_EXTI2_PF* = (0x00000500) # !< PF[2] pin

# *
#  @brief  EXTI3 configuration
##

const
  SYSCFG_EXTICR1_EXTI3_PA* = (0x00000000) # !< PA[3] pin
  SYSCFG_EXTICR1_EXTI3_PB* = (0x00001000) # !< PB[3] pin
  SYSCFG_EXTICR1_EXTI3_PC* = (0x00002000) # !< PC[3] pin
  SYSCFG_EXTICR1_EXTI3_PD* = (0x00003000) # !< PD[3] pin
  SYSCFG_EXTICR1_EXTI3_PF* = (0x00005000) # !< PF[3] pin

# ****************  Bit definition for SYSCFG_EXTICR2 register  *************

const
  SYSCFG_EXTICR2_EXTI4_Pos* = (0)
  SYSCFG_EXTICR2_EXTI4_Msk* = (0x0000000F shl SYSCFG_EXTICR2_EXTI4_Pos) # !< 0x0000000F
  SYSCFG_EXTICR2_EXTI4* = SYSCFG_EXTICR2_EXTI4_Msk
  SYSCFG_EXTICR2_EXTI5_Pos* = (4)
  SYSCFG_EXTICR2_EXTI5_Msk* = (0x0000000F shl SYSCFG_EXTICR2_EXTI5_Pos) # !< 0x000000F0
  SYSCFG_EXTICR2_EXTI5* = SYSCFG_EXTICR2_EXTI5_Msk
  SYSCFG_EXTICR2_EXTI6_Pos* = (8)
  SYSCFG_EXTICR2_EXTI6_Msk* = (0x0000000F shl SYSCFG_EXTICR2_EXTI6_Pos) # !< 0x00000F00
  SYSCFG_EXTICR2_EXTI6* = SYSCFG_EXTICR2_EXTI6_Msk
  SYSCFG_EXTICR2_EXTI7_Pos* = (12)
  SYSCFG_EXTICR2_EXTI7_Msk* = (0x0000000F shl SYSCFG_EXTICR2_EXTI7_Pos) # !< 0x0000F000
  SYSCFG_EXTICR2_EXTI7* = SYSCFG_EXTICR2_EXTI7_Msk

# *
#  @brief  EXTI4 configuration
##

const
  SYSCFG_EXTICR2_EXTI4_PA* = (0x00000000) # !< PA[4] pin
  SYSCFG_EXTICR2_EXTI4_PB* = (0x00000001) # !< PB[4] pin
  SYSCFG_EXTICR2_EXTI4_PC* = (0x00000002) # !< PC[4] pin
  SYSCFG_EXTICR2_EXTI4_PD* = (0x00000003) # !< PD[4] pin
  SYSCFG_EXTICR2_EXTI4_PF* = (0x00000005) # !< PF[4] pin

# *
#  @brief  EXTI5 configuration
##

const
  SYSCFG_EXTICR2_EXTI5_PA* = (0x00000000) # !< PA[5] pin
  SYSCFG_EXTICR2_EXTI5_PB* = (0x00000010) # !< PB[5] pin
  SYSCFG_EXTICR2_EXTI5_PC* = (0x00000020) # !< PC[5] pin
  SYSCFG_EXTICR2_EXTI5_PD* = (0x00000030) # !< PD[5] pin
  SYSCFG_EXTICR2_EXTI5_PF* = (0x00000050) # !< PF[5] pin

# *
#  @brief  EXTI6 configuration
##

const
  SYSCFG_EXTICR2_EXTI6_PA* = (0x00000000) # !< PA[6] pin
  SYSCFG_EXTICR2_EXTI6_PB* = (0x00000100) # !< PB[6] pin
  SYSCFG_EXTICR2_EXTI6_PC* = (0x00000200) # !< PC[6] pin
  SYSCFG_EXTICR2_EXTI6_PD* = (0x00000300) # !< PD[6] pin
  SYSCFG_EXTICR2_EXTI6_PF* = (0x00000500) # !< PF[6] pin

# *
#  @brief  EXTI7 configuration
##

const
  SYSCFG_EXTICR2_EXTI7_PA* = (0x00000000) # !< PA[7] pin
  SYSCFG_EXTICR2_EXTI7_PB* = (0x00001000) # !< PB[7] pin
  SYSCFG_EXTICR2_EXTI7_PC* = (0x00002000) # !< PC[7] pin
  SYSCFG_EXTICR2_EXTI7_PD* = (0x00003000) # !< PD[7] pin
  SYSCFG_EXTICR2_EXTI7_PF* = (0x00005000) # !< PF[7] pin

# ****************  Bit definition for SYSCFG_EXTICR3 register  *************

const
  SYSCFG_EXTICR3_EXTI8_Pos* = (0)
  SYSCFG_EXTICR3_EXTI8_Msk* = (0x0000000F shl SYSCFG_EXTICR3_EXTI8_Pos) # !< 0x0000000F
  SYSCFG_EXTICR3_EXTI8* = SYSCFG_EXTICR3_EXTI8_Msk
  SYSCFG_EXTICR3_EXTI9_Pos* = (4)
  SYSCFG_EXTICR3_EXTI9_Msk* = (0x0000000F shl SYSCFG_EXTICR3_EXTI9_Pos) # !< 0x000000F0
  SYSCFG_EXTICR3_EXTI9* = SYSCFG_EXTICR3_EXTI9_Msk
  SYSCFG_EXTICR3_EXTI10_Pos* = (8)
  SYSCFG_EXTICR3_EXTI10_Msk* = (0x0000000F shl SYSCFG_EXTICR3_EXTI10_Pos) # !< 0x00000F00
  SYSCFG_EXTICR3_EXTI10* = SYSCFG_EXTICR3_EXTI10_Msk
  SYSCFG_EXTICR3_EXTI11_Pos* = (12)
  SYSCFG_EXTICR3_EXTI11_Msk* = (0x0000000F shl SYSCFG_EXTICR3_EXTI11_Pos) # !< 0x0000F000
  SYSCFG_EXTICR3_EXTI11* = SYSCFG_EXTICR3_EXTI11_Msk

# *
#  @brief  EXTI8 configuration
##

const
  SYSCFG_EXTICR3_EXTI8_PA* = (0x00000000) # !< PA[8] pin
  SYSCFG_EXTICR3_EXTI8_PB* = (0x00000001) # !< PB[8] pin
  SYSCFG_EXTICR3_EXTI8_PC* = (0x00000002) # !< PC[8] pin
  SYSCFG_EXTICR3_EXTI8_PD* = (0x00000003) # !< PD[8] pin
  SYSCFG_EXTICR3_EXTI8_PF* = (0x00000005) # !< PF[8] pin

# *
#  @brief  EXTI9 configuration
##

const
  SYSCFG_EXTICR3_EXTI9_PA* = (0x00000000) # !< PA[9] pin
  SYSCFG_EXTICR3_EXTI9_PB* = (0x00000010) # !< PB[9] pin
  SYSCFG_EXTICR3_EXTI9_PC* = (0x00000020) # !< PC[9] pin
  SYSCFG_EXTICR3_EXTI9_PD* = (0x00000030) # !< PD[9] pin
  SYSCFG_EXTICR3_EXTI9_PF* = (0x00000050) # !< PF[9] pin

# *
#  @brief  EXTI10 configuration
##

const
  SYSCFG_EXTICR3_EXTI10_PA* = (0x00000000) # !< PA[10] pin
  SYSCFG_EXTICR3_EXTI10_PB* = (0x00000100) # !< PB[10] pin
  SYSCFG_EXTICR3_EXTI10_PC* = (0x00000200) # !< PC[10] pin
  SYSCFG_EXTICR3_EXTI10_PD* = (0x00000300) # !< PD[10] pin
  SYSCFG_EXTICR3_EXTI10_PF* = (0x00000500) # !< PF[10] pin

# *
#  @brief  EXTI11 configuration
##

const
  SYSCFG_EXTICR3_EXTI11_PA* = (0x00000000) # !< PA[11] pin
  SYSCFG_EXTICR3_EXTI11_PB* = (0x00001000) # !< PB[11] pin
  SYSCFG_EXTICR3_EXTI11_PC* = (0x00002000) # !< PC[11] pin
  SYSCFG_EXTICR3_EXTI11_PD* = (0x00003000) # !< PD[11] pin
  SYSCFG_EXTICR3_EXTI11_PF* = (0x00005000) # !< PF[11] pin

# ****************  Bit definition for SYSCFG_EXTICR4 register  *************

const
  SYSCFG_EXTICR4_EXTI12_Pos* = (0)
  SYSCFG_EXTICR4_EXTI12_Msk* = (0x0000000F shl SYSCFG_EXTICR4_EXTI12_Pos) # !< 0x0000000F
  SYSCFG_EXTICR4_EXTI12* = SYSCFG_EXTICR4_EXTI12_Msk
  SYSCFG_EXTICR4_EXTI13_Pos* = (4)
  SYSCFG_EXTICR4_EXTI13_Msk* = (0x0000000F shl SYSCFG_EXTICR4_EXTI13_Pos) # !< 0x000000F0
  SYSCFG_EXTICR4_EXTI13* = SYSCFG_EXTICR4_EXTI13_Msk
  SYSCFG_EXTICR4_EXTI14_Pos* = (8)
  SYSCFG_EXTICR4_EXTI14_Msk* = (0x0000000F shl SYSCFG_EXTICR4_EXTI14_Pos) # !< 0x00000F00
  SYSCFG_EXTICR4_EXTI14* = SYSCFG_EXTICR4_EXTI14_Msk
  SYSCFG_EXTICR4_EXTI15_Pos* = (12)
  SYSCFG_EXTICR4_EXTI15_Msk* = (0x0000000F shl SYSCFG_EXTICR4_EXTI15_Pos) # !< 0x0000F000
  SYSCFG_EXTICR4_EXTI15* = SYSCFG_EXTICR4_EXTI15_Msk

# *
#  @brief  EXTI12 configuration
##

const
  SYSCFG_EXTICR4_EXTI12_PA* = (0x00000000) # !< PA[12] pin
  SYSCFG_EXTICR4_EXTI12_PB* = (0x00000001) # !< PB[12] pin
  SYSCFG_EXTICR4_EXTI12_PC* = (0x00000002) # !< PC[12] pin
  SYSCFG_EXTICR4_EXTI12_PD* = (0x00000003) # !< PD[12] pin
  SYSCFG_EXTICR4_EXTI12_PF* = (0x00000005) # !< PF[12] pin

# *
#  @brief  EXTI13 configuration
##

const
  SYSCFG_EXTICR4_EXTI13_PA* = (0x00000000) # !< PA[13] pin
  SYSCFG_EXTICR4_EXTI13_PB* = (0x00000010) # !< PB[13] pin
  SYSCFG_EXTICR4_EXTI13_PC* = (0x00000020) # !< PC[13] pin
  SYSCFG_EXTICR4_EXTI13_PD* = (0x00000030) # !< PD[13] pin
  SYSCFG_EXTICR4_EXTI13_PF* = (0x00000050) # !< PF[13] pin

# *
#  @brief  EXTI14 configuration
##

const
  SYSCFG_EXTICR4_EXTI14_PA* = (0x00000000) # !< PA[14] pin
  SYSCFG_EXTICR4_EXTI14_PB* = (0x00000100) # !< PB[14] pin
  SYSCFG_EXTICR4_EXTI14_PC* = (0x00000200) # !< PC[14] pin
  SYSCFG_EXTICR4_EXTI14_PD* = (0x00000300) # !< PD[14] pin
  SYSCFG_EXTICR4_EXTI14_PF* = (0x00000500) # !< PF[14] pin

# *
#  @brief  EXTI15 configuration
##

const
  SYSCFG_EXTICR4_EXTI15_PA* = (0x00000000) # !< PA[15] pin
  SYSCFG_EXTICR4_EXTI15_PB* = (0x00001000) # !< PB[15] pin
  SYSCFG_EXTICR4_EXTI15_PC* = (0x00002000) # !< PC[15] pin
  SYSCFG_EXTICR4_EXTI15_PD* = (0x00003000) # !< PD[15] pin
  SYSCFG_EXTICR4_EXTI15_PF* = (0x00005000) # !< PF[15] pin

# ****************  Bit definition for SYSCFG_CFGR2 register  ***************

const
  SYSCFG_CFGR2_LOCKUP_LOCK_Pos* = (0)
  SYSCFG_CFGR2_LOCKUP_LOCK_Msk* = (0x00000001 shl SYSCFG_CFGR2_LOCKUP_LOCK_Pos) # !< 0x00000001
  SYSCFG_CFGR2_LOCKUP_LOCK* = SYSCFG_CFGR2_LOCKUP_LOCK_Msk
  SYSCFG_CFGR2_SRAM_PARITY_LOCK_Pos* = (1)
  SYSCFG_CFGR2_SRAM_PARITY_LOCK_Msk* = (
    0x00000001 shl SYSCFG_CFGR2_SRAM_PARITY_LOCK_Pos) # !< 0x00000002
  SYSCFG_CFGR2_SRAM_PARITY_LOCK* = SYSCFG_CFGR2_SRAM_PARITY_LOCK_Msk
  SYSCFG_CFGR2_SRAM_PEF_Pos* = (8)
  SYSCFG_CFGR2_SRAM_PEF_Msk* = (0x00000001 shl SYSCFG_CFGR2_SRAM_PEF_Pos) # !< 0x00000100
  SYSCFG_CFGR2_SRAM_PEF* = SYSCFG_CFGR2_SRAM_PEF_Msk
  SYSCFG_CFGR2_SRAM_PE* = SYSCFG_CFGR2_SRAM_PEF

# ***************************************************************************
##
#                                Timers (TIM)
##
# ***************************************************************************
# ******************  Bit definition for TIM_CR1 register  ******************

const
  TIM_CR1_CEN_Pos* = (0)
  TIM_CR1_CEN_Msk* = (0x00000001 shl TIM_CR1_CEN_Pos) # !< 0x00000001
  TIM_CR1_CEN* = TIM_CR1_CEN_Msk
  TIM_CR1_UDIS_Pos* = (1)
  TIM_CR1_UDIS_Msk* = (0x00000001 shl TIM_CR1_UDIS_Pos) # !< 0x00000002
  TIM_CR1_UDIS* = TIM_CR1_UDIS_Msk
  TIM_CR1_URS_Pos* = (2)
  TIM_CR1_URS_Msk* = (0x00000001 shl TIM_CR1_URS_Pos) # !< 0x00000004
  TIM_CR1_URS* = TIM_CR1_URS_Msk
  TIM_CR1_OPM_Pos* = (3)
  TIM_CR1_OPM_Msk* = (0x00000001 shl TIM_CR1_OPM_Pos) # !< 0x00000008
  TIM_CR1_OPM* = TIM_CR1_OPM_Msk
  TIM_CR1_DIR_Pos* = (4)
  TIM_CR1_DIR_Msk* = (0x00000001 shl TIM_CR1_DIR_Pos) # !< 0x00000010
  TIM_CR1_DIR* = TIM_CR1_DIR_Msk
  TIM_CR1_CMS_Pos* = (5)
  TIM_CR1_CMS_Msk* = (0x00000003 shl TIM_CR1_CMS_Pos) # !< 0x00000060
  TIM_CR1_CMS* = TIM_CR1_CMS_Msk
  TIM_CR1_CMS_0* = (0x00000001 shl TIM_CR1_CMS_Pos) # !< 0x00000020
  TIM_CR1_CMS_1* = (0x00000002 shl TIM_CR1_CMS_Pos) # !< 0x00000040
  TIM_CR1_ARPE_Pos* = (7)
  TIM_CR1_ARPE_Msk* = (0x00000001 shl TIM_CR1_ARPE_Pos) # !< 0x00000080
  TIM_CR1_ARPE* = TIM_CR1_ARPE_Msk
  TIM_CR1_CKD_Pos* = (8)
  TIM_CR1_CKD_Msk* = (0x00000003 shl TIM_CR1_CKD_Pos) # !< 0x00000300
  TIM_CR1_CKD* = TIM_CR1_CKD_Msk
  TIM_CR1_CKD_0* = (0x00000001 shl TIM_CR1_CKD_Pos) # !< 0x00000100
  TIM_CR1_CKD_1* = (0x00000002 shl TIM_CR1_CKD_Pos) # !< 0x00000200

# ******************  Bit definition for TIM_CR2 register  ******************

const
  TIM_CR2_CCPC_Pos* = (0)
  TIM_CR2_CCPC_Msk* = (0x00000001 shl TIM_CR2_CCPC_Pos) # !< 0x00000001
  TIM_CR2_CCPC* = TIM_CR2_CCPC_Msk
  TIM_CR2_CCUS_Pos* = (2)
  TIM_CR2_CCUS_Msk* = (0x00000001 shl TIM_CR2_CCUS_Pos) # !< 0x00000004
  TIM_CR2_CCUS* = TIM_CR2_CCUS_Msk
  TIM_CR2_CCDS_Pos* = (3)
  TIM_CR2_CCDS_Msk* = (0x00000001 shl TIM_CR2_CCDS_Pos) # !< 0x00000008
  TIM_CR2_CCDS* = TIM_CR2_CCDS_Msk
  TIM_CR2_MMS_Pos* = (4)
  TIM_CR2_MMS_Msk* = (0x00000007 shl TIM_CR2_MMS_Pos) # !< 0x00000070
  TIM_CR2_MMS* = TIM_CR2_MMS_Msk
  TIM_CR2_MMS_0* = (0x00000001 shl TIM_CR2_MMS_Pos) # !< 0x00000010
  TIM_CR2_MMS_1* = (0x00000002 shl TIM_CR2_MMS_Pos) # !< 0x00000020
  TIM_CR2_MMS_2* = (0x00000004 shl TIM_CR2_MMS_Pos) # !< 0x00000040
  TIM_CR2_TI1S_Pos* = (7)
  TIM_CR2_TI1S_Msk* = (0x00000001 shl TIM_CR2_TI1S_Pos) # !< 0x00000080
  TIM_CR2_TI1S* = TIM_CR2_TI1S_Msk
  TIM_CR2_OIS1_Pos* = (8)
  TIM_CR2_OIS1_Msk* = (0x00000001 shl TIM_CR2_OIS1_Pos) # !< 0x00000100
  TIM_CR2_OIS1* = TIM_CR2_OIS1_Msk
  TIM_CR2_OIS1N_Pos* = (9)
  TIM_CR2_OIS1N_Msk* = (0x00000001 shl TIM_CR2_OIS1N_Pos) # !< 0x00000200
  TIM_CR2_OIS1N* = TIM_CR2_OIS1N_Msk
  TIM_CR2_OIS2_Pos* = (10)
  TIM_CR2_OIS2_Msk* = (0x00000001 shl TIM_CR2_OIS2_Pos) # !< 0x00000400
  TIM_CR2_OIS2* = TIM_CR2_OIS2_Msk
  TIM_CR2_OIS2N_Pos* = (11)
  TIM_CR2_OIS2N_Msk* = (0x00000001 shl TIM_CR2_OIS2N_Pos) # !< 0x00000800
  TIM_CR2_OIS2N* = TIM_CR2_OIS2N_Msk
  TIM_CR2_OIS3_Pos* = (12)
  TIM_CR2_OIS3_Msk* = (0x00000001 shl TIM_CR2_OIS3_Pos) # !< 0x00001000
  TIM_CR2_OIS3* = TIM_CR2_OIS3_Msk
  TIM_CR2_OIS3N_Pos* = (13)
  TIM_CR2_OIS3N_Msk* = (0x00000001 shl TIM_CR2_OIS3N_Pos) # !< 0x00002000
  TIM_CR2_OIS3N* = TIM_CR2_OIS3N_Msk
  TIM_CR2_OIS4_Pos* = (14)
  TIM_CR2_OIS4_Msk* = (0x00000001 shl TIM_CR2_OIS4_Pos) # !< 0x00004000
  TIM_CR2_OIS4* = TIM_CR2_OIS4_Msk

# ******************  Bit definition for TIM_SMCR register  *****************

const
  TIM_SMCR_SMS_Pos* = (0)
  TIM_SMCR_SMS_Msk* = (0x00000007 shl TIM_SMCR_SMS_Pos) # !< 0x00000007
  TIM_SMCR_SMS* = TIM_SMCR_SMS_Msk
  TIM_SMCR_SMS_0* = (0x00000001 shl TIM_SMCR_SMS_Pos) # !< 0x00000001
  TIM_SMCR_SMS_1* = (0x00000002 shl TIM_SMCR_SMS_Pos) # !< 0x00000002
  TIM_SMCR_SMS_2* = (0x00000004 shl TIM_SMCR_SMS_Pos) # !< 0x00000004
  TIM_SMCR_OCCS_Pos* = (3)
  TIM_SMCR_OCCS_Msk* = (0x00000001 shl TIM_SMCR_OCCS_Pos) # !< 0x00000008
  TIM_SMCR_OCCS* = TIM_SMCR_OCCS_Msk
  TIM_SMCR_TS_Pos* = (4)
  TIM_SMCR_TS_Msk* = (0x00000007 shl TIM_SMCR_TS_Pos) # !< 0x00000070
  TIM_SMCR_TS* = TIM_SMCR_TS_Msk
  TIM_SMCR_TS_0* = (0x00000001 shl TIM_SMCR_TS_Pos) # !< 0x00000010
  TIM_SMCR_TS_1* = (0x00000002 shl TIM_SMCR_TS_Pos) # !< 0x00000020
  TIM_SMCR_TS_2* = (0x00000004 shl TIM_SMCR_TS_Pos) # !< 0x00000040
  TIM_SMCR_MSM_Pos* = (7)
  TIM_SMCR_MSM_Msk* = (0x00000001 shl TIM_SMCR_MSM_Pos) # !< 0x00000080
  TIM_SMCR_MSM* = TIM_SMCR_MSM_Msk
  TIM_SMCR_ETF_Pos* = (8)
  TIM_SMCR_ETF_Msk* = (0x0000000F shl TIM_SMCR_ETF_Pos) # !< 0x00000F00
  TIM_SMCR_ETF* = TIM_SMCR_ETF_Msk
  TIM_SMCR_ETF_0* = (0x00000001 shl TIM_SMCR_ETF_Pos) # !< 0x00000100
  TIM_SMCR_ETF_1* = (0x00000002 shl TIM_SMCR_ETF_Pos) # !< 0x00000200
  TIM_SMCR_ETF_2* = (0x00000004 shl TIM_SMCR_ETF_Pos) # !< 0x00000400
  TIM_SMCR_ETF_3* = (0x00000008 shl TIM_SMCR_ETF_Pos) # !< 0x00000800
  TIM_SMCR_ETPS_Pos* = (12)
  TIM_SMCR_ETPS_Msk* = (0x00000003 shl TIM_SMCR_ETPS_Pos) # !< 0x00003000
  TIM_SMCR_ETPS* = TIM_SMCR_ETPS_Msk
  TIM_SMCR_ETPS_0* = (0x00000001 shl TIM_SMCR_ETPS_Pos) # !< 0x00001000
  TIM_SMCR_ETPS_1* = (0x00000002 shl TIM_SMCR_ETPS_Pos) # !< 0x00002000
  TIM_SMCR_ECE_Pos* = (14)
  TIM_SMCR_ECE_Msk* = (0x00000001 shl TIM_SMCR_ECE_Pos) # !< 0x00004000
  TIM_SMCR_ECE* = TIM_SMCR_ECE_Msk
  TIM_SMCR_ETP_Pos* = (15)
  TIM_SMCR_ETP_Msk* = (0x00000001 shl TIM_SMCR_ETP_Pos) # !< 0x00008000
  TIM_SMCR_ETP* = TIM_SMCR_ETP_Msk

# ******************  Bit definition for TIM_DIER register  *****************

const
  TIM_DIER_UIE_Pos* = (0)
  TIM_DIER_UIE_Msk* = (0x00000001 shl TIM_DIER_UIE_Pos) # !< 0x00000001
  TIM_DIER_UIE* = TIM_DIER_UIE_Msk
  TIM_DIER_CC1IE_Pos* = (1)
  TIM_DIER_CC1IE_Msk* = (0x00000001 shl TIM_DIER_CC1IE_Pos) # !< 0x00000002
  TIM_DIER_CC1IE* = TIM_DIER_CC1IE_Msk
  TIM_DIER_CC2IE_Pos* = (2)
  TIM_DIER_CC2IE_Msk* = (0x00000001 shl TIM_DIER_CC2IE_Pos) # !< 0x00000004
  TIM_DIER_CC2IE* = TIM_DIER_CC2IE_Msk
  TIM_DIER_CC3IE_Pos* = (3)
  TIM_DIER_CC3IE_Msk* = (0x00000001 shl TIM_DIER_CC3IE_Pos) # !< 0x00000008
  TIM_DIER_CC3IE* = TIM_DIER_CC3IE_Msk
  TIM_DIER_CC4IE_Pos* = (4)
  TIM_DIER_CC4IE_Msk* = (0x00000001 shl TIM_DIER_CC4IE_Pos) # !< 0x00000010
  TIM_DIER_CC4IE* = TIM_DIER_CC4IE_Msk
  TIM_DIER_COMIE_Pos* = (5)
  TIM_DIER_COMIE_Msk* = (0x00000001 shl TIM_DIER_COMIE_Pos) # !< 0x00000020
  TIM_DIER_COMIE* = TIM_DIER_COMIE_Msk
  TIM_DIER_TIE_Pos* = (6)
  TIM_DIER_TIE_Msk* = (0x00000001 shl TIM_DIER_TIE_Pos) # !< 0x00000040
  TIM_DIER_TIE* = TIM_DIER_TIE_Msk
  TIM_DIER_BIE_Pos* = (7)
  TIM_DIER_BIE_Msk* = (0x00000001 shl TIM_DIER_BIE_Pos) # !< 0x00000080
  TIM_DIER_BIE* = TIM_DIER_BIE_Msk
  TIM_DIER_UDE_Pos* = (8)
  TIM_DIER_UDE_Msk* = (0x00000001 shl TIM_DIER_UDE_Pos) # !< 0x00000100
  TIM_DIER_UDE* = TIM_DIER_UDE_Msk
  TIM_DIER_CC1DE_Pos* = (9)
  TIM_DIER_CC1DE_Msk* = (0x00000001 shl TIM_DIER_CC1DE_Pos) # !< 0x00000200
  TIM_DIER_CC1DE* = TIM_DIER_CC1DE_Msk
  TIM_DIER_CC2DE_Pos* = (10)
  TIM_DIER_CC2DE_Msk* = (0x00000001 shl TIM_DIER_CC2DE_Pos) # !< 0x00000400
  TIM_DIER_CC2DE* = TIM_DIER_CC2DE_Msk
  TIM_DIER_CC3DE_Pos* = (11)
  TIM_DIER_CC3DE_Msk* = (0x00000001 shl TIM_DIER_CC3DE_Pos) # !< 0x00000800
  TIM_DIER_CC3DE* = TIM_DIER_CC3DE_Msk
  TIM_DIER_CC4DE_Pos* = (12)
  TIM_DIER_CC4DE_Msk* = (0x00000001 shl TIM_DIER_CC4DE_Pos) # !< 0x00001000
  TIM_DIER_CC4DE* = TIM_DIER_CC4DE_Msk
  TIM_DIER_COMDE_Pos* = (13)
  TIM_DIER_COMDE_Msk* = (0x00000001 shl TIM_DIER_COMDE_Pos) # !< 0x00002000
  TIM_DIER_COMDE* = TIM_DIER_COMDE_Msk
  TIM_DIER_TDE_Pos* = (14)
  TIM_DIER_TDE_Msk* = (0x00000001 shl TIM_DIER_TDE_Pos) # !< 0x00004000
  TIM_DIER_TDE* = TIM_DIER_TDE_Msk

# *******************  Bit definition for TIM_SR register  ******************

const
  TIM_SR_UIF_Pos* = (0)
  TIM_SR_UIF_Msk* = (0x00000001 shl TIM_SR_UIF_Pos) # !< 0x00000001
  TIM_SR_UIF* = TIM_SR_UIF_Msk
  TIM_SR_CC1IF_Pos* = (1)
  TIM_SR_CC1IF_Msk* = (0x00000001 shl TIM_SR_CC1IF_Pos) # !< 0x00000002
  TIM_SR_CC1IF* = TIM_SR_CC1IF_Msk
  TIM_SR_CC2IF_Pos* = (2)
  TIM_SR_CC2IF_Msk* = (0x00000001 shl TIM_SR_CC2IF_Pos) # !< 0x00000004
  TIM_SR_CC2IF* = TIM_SR_CC2IF_Msk
  TIM_SR_CC3IF_Pos* = (3)
  TIM_SR_CC3IF_Msk* = (0x00000001 shl TIM_SR_CC3IF_Pos) # !< 0x00000008
  TIM_SR_CC3IF* = TIM_SR_CC3IF_Msk
  TIM_SR_CC4IF_Pos* = (4)
  TIM_SR_CC4IF_Msk* = (0x00000001 shl TIM_SR_CC4IF_Pos) # !< 0x00000010
  TIM_SR_CC4IF* = TIM_SR_CC4IF_Msk
  TIM_SR_COMIF_Pos* = (5)
  TIM_SR_COMIF_Msk* = (0x00000001 shl TIM_SR_COMIF_Pos) # !< 0x00000020
  TIM_SR_COMIF* = TIM_SR_COMIF_Msk
  TIM_SR_TIF_Pos* = (6)
  TIM_SR_TIF_Msk* = (0x00000001 shl TIM_SR_TIF_Pos) # !< 0x00000040
  TIM_SR_TIF* = TIM_SR_TIF_Msk
  TIM_SR_BIF_Pos* = (7)
  TIM_SR_BIF_Msk* = (0x00000001 shl TIM_SR_BIF_Pos) # !< 0x00000080
  TIM_SR_BIF* = TIM_SR_BIF_Msk
  TIM_SR_CC1OF_Pos* = (9)
  TIM_SR_CC1OF_Msk* = (0x00000001 shl TIM_SR_CC1OF_Pos) # !< 0x00000200
  TIM_SR_CC1OF* = TIM_SR_CC1OF_Msk
  TIM_SR_CC2OF_Pos* = (10)
  TIM_SR_CC2OF_Msk* = (0x00000001 shl TIM_SR_CC2OF_Pos) # !< 0x00000400
  TIM_SR_CC2OF* = TIM_SR_CC2OF_Msk
  TIM_SR_CC3OF_Pos* = (11)
  TIM_SR_CC3OF_Msk* = (0x00000001 shl TIM_SR_CC3OF_Pos) # !< 0x00000800
  TIM_SR_CC3OF* = TIM_SR_CC3OF_Msk
  TIM_SR_CC4OF_Pos* = (12)
  TIM_SR_CC4OF_Msk* = (0x00000001 shl TIM_SR_CC4OF_Pos) # !< 0x00001000
  TIM_SR_CC4OF* = TIM_SR_CC4OF_Msk

# ******************  Bit definition for TIM_EGR register  ******************

const
  TIM_EGR_UG_Pos* = (0)
  TIM_EGR_UG_Msk* = (0x00000001 shl TIM_EGR_UG_Pos) # !< 0x00000001
  TIM_EGR_UG* = TIM_EGR_UG_Msk
  TIM_EGR_CC1G_Pos* = (1)
  TIM_EGR_CC1G_Msk* = (0x00000001 shl TIM_EGR_CC1G_Pos) # !< 0x00000002
  TIM_EGR_CC1G* = TIM_EGR_CC1G_Msk
  TIM_EGR_CC2G_Pos* = (2)
  TIM_EGR_CC2G_Msk* = (0x00000001 shl TIM_EGR_CC2G_Pos) # !< 0x00000004
  TIM_EGR_CC2G* = TIM_EGR_CC2G_Msk
  TIM_EGR_CC3G_Pos* = (3)
  TIM_EGR_CC3G_Msk* = (0x00000001 shl TIM_EGR_CC3G_Pos) # !< 0x00000008
  TIM_EGR_CC3G* = TIM_EGR_CC3G_Msk
  TIM_EGR_CC4G_Pos* = (4)
  TIM_EGR_CC4G_Msk* = (0x00000001 shl TIM_EGR_CC4G_Pos) # !< 0x00000010
  TIM_EGR_CC4G* = TIM_EGR_CC4G_Msk
  TIM_EGR_COMG_Pos* = (5)
  TIM_EGR_COMG_Msk* = (0x00000001 shl TIM_EGR_COMG_Pos) # !< 0x00000020
  TIM_EGR_COMG* = TIM_EGR_COMG_Msk
  TIM_EGR_TG_Pos* = (6)
  TIM_EGR_TG_Msk* = (0x00000001 shl TIM_EGR_TG_Pos) # !< 0x00000040
  TIM_EGR_TG* = TIM_EGR_TG_Msk
  TIM_EGR_BG_Pos* = (7)
  TIM_EGR_BG_Msk* = (0x00000001 shl TIM_EGR_BG_Pos) # !< 0x00000080
  TIM_EGR_BG* = TIM_EGR_BG_Msk

# *****************  Bit definition for TIM_CCMR1 register  *****************

const
  TIM_CCMR1_CC1S_Pos* = (0)
  TIM_CCMR1_CC1S_Msk* = (0x00000003 shl TIM_CCMR1_CC1S_Pos) # !< 0x00000003
  TIM_CCMR1_CC1S* = TIM_CCMR1_CC1S_Msk
  TIM_CCMR1_CC1S_0* = (0x00000001 shl TIM_CCMR1_CC1S_Pos) # !< 0x00000001
  TIM_CCMR1_CC1S_1* = (0x00000002 shl TIM_CCMR1_CC1S_Pos) # !< 0x00000002
  TIM_CCMR1_OC1FE_Pos* = (2)
  TIM_CCMR1_OC1FE_Msk* = (0x00000001 shl TIM_CCMR1_OC1FE_Pos) # !< 0x00000004
  TIM_CCMR1_OC1FE* = TIM_CCMR1_OC1FE_Msk
  TIM_CCMR1_OC1PE_Pos* = (3)
  TIM_CCMR1_OC1PE_Msk* = (0x00000001 shl TIM_CCMR1_OC1PE_Pos) # !< 0x00000008
  TIM_CCMR1_OC1PE* = TIM_CCMR1_OC1PE_Msk
  TIM_CCMR1_OC1M_Pos* = (4)
  TIM_CCMR1_OC1M_Msk* = (0x00000007 shl TIM_CCMR1_OC1M_Pos) # !< 0x00000070
  TIM_CCMR1_OC1M* = TIM_CCMR1_OC1M_Msk
  TIM_CCMR1_OC1M_0* = (0x00000001 shl TIM_CCMR1_OC1M_Pos) # !< 0x00000010
  TIM_CCMR1_OC1M_1* = (0x00000002 shl TIM_CCMR1_OC1M_Pos) # !< 0x00000020
  TIM_CCMR1_OC1M_2* = (0x00000004 shl TIM_CCMR1_OC1M_Pos) # !< 0x00000040
  TIM_CCMR1_OC1CE_Pos* = (7)
  TIM_CCMR1_OC1CE_Msk* = (0x00000001 shl TIM_CCMR1_OC1CE_Pos) # !< 0x00000080
  TIM_CCMR1_OC1CE* = TIM_CCMR1_OC1CE_Msk
  TIM_CCMR1_CC2S_Pos* = (8)
  TIM_CCMR1_CC2S_Msk* = (0x00000003 shl TIM_CCMR1_CC2S_Pos) # !< 0x00000300
  TIM_CCMR1_CC2S* = TIM_CCMR1_CC2S_Msk
  TIM_CCMR1_CC2S_0* = (0x00000001 shl TIM_CCMR1_CC2S_Pos) # !< 0x00000100
  TIM_CCMR1_CC2S_1* = (0x00000002 shl TIM_CCMR1_CC2S_Pos) # !< 0x00000200
  TIM_CCMR1_OC2FE_Pos* = (10)
  TIM_CCMR1_OC2FE_Msk* = (0x00000001 shl TIM_CCMR1_OC2FE_Pos) # !< 0x00000400
  TIM_CCMR1_OC2FE* = TIM_CCMR1_OC2FE_Msk
  TIM_CCMR1_OC2PE_Pos* = (11)
  TIM_CCMR1_OC2PE_Msk* = (0x00000001 shl TIM_CCMR1_OC2PE_Pos) # !< 0x00000800
  TIM_CCMR1_OC2PE* = TIM_CCMR1_OC2PE_Msk
  TIM_CCMR1_OC2M_Pos* = (12)
  TIM_CCMR1_OC2M_Msk* = (0x00000007 shl TIM_CCMR1_OC2M_Pos) # !< 0x00007000
  TIM_CCMR1_OC2M* = TIM_CCMR1_OC2M_Msk
  TIM_CCMR1_OC2M_0* = (0x00000001 shl TIM_CCMR1_OC2M_Pos) # !< 0x00001000
  TIM_CCMR1_OC2M_1* = (0x00000002 shl TIM_CCMR1_OC2M_Pos) # !< 0x00002000
  TIM_CCMR1_OC2M_2* = (0x00000004 shl TIM_CCMR1_OC2M_Pos) # !< 0x00004000
  TIM_CCMR1_OC2CE_Pos* = (15)
  TIM_CCMR1_OC2CE_Msk* = (0x00000001 shl TIM_CCMR1_OC2CE_Pos) # !< 0x00008000
  TIM_CCMR1_OC2CE* = TIM_CCMR1_OC2CE_Msk

# ---------------------------------------------------------------------------

const
  TIM_CCMR1_IC1PSC_Pos* = (2)
  TIM_CCMR1_IC1PSC_Msk* = (0x00000003 shl TIM_CCMR1_IC1PSC_Pos) # !< 0x0000000C
  TIM_CCMR1_IC1PSC* = TIM_CCMR1_IC1PSC_Msk
  TIM_CCMR1_IC1PSC_0* = (0x00000001 shl TIM_CCMR1_IC1PSC_Pos) # !< 0x00000004
  TIM_CCMR1_IC1PSC_1* = (0x00000002 shl TIM_CCMR1_IC1PSC_Pos) # !< 0x00000008
  TIM_CCMR1_IC1F_Pos* = (4)
  TIM_CCMR1_IC1F_Msk* = (0x0000000F shl TIM_CCMR1_IC1F_Pos) # !< 0x000000F0
  TIM_CCMR1_IC1F* = TIM_CCMR1_IC1F_Msk
  TIM_CCMR1_IC1F_0* = (0x00000001 shl TIM_CCMR1_IC1F_Pos) # !< 0x00000010
  TIM_CCMR1_IC1F_1* = (0x00000002 shl TIM_CCMR1_IC1F_Pos) # !< 0x00000020
  TIM_CCMR1_IC1F_2* = (0x00000004 shl TIM_CCMR1_IC1F_Pos) # !< 0x00000040
  TIM_CCMR1_IC1F_3* = (0x00000008 shl TIM_CCMR1_IC1F_Pos) # !< 0x00000080
  TIM_CCMR1_IC2PSC_Pos* = (10)
  TIM_CCMR1_IC2PSC_Msk* = (0x00000003 shl TIM_CCMR1_IC2PSC_Pos) # !< 0x00000C00
  TIM_CCMR1_IC2PSC* = TIM_CCMR1_IC2PSC_Msk
  TIM_CCMR1_IC2PSC_0* = (0x00000001 shl TIM_CCMR1_IC2PSC_Pos) # !< 0x00000400
  TIM_CCMR1_IC2PSC_1* = (0x00000002 shl TIM_CCMR1_IC2PSC_Pos) # !< 0x00000800
  TIM_CCMR1_IC2F_Pos* = (12)
  TIM_CCMR1_IC2F_Msk* = (0x0000000F shl TIM_CCMR1_IC2F_Pos) # !< 0x0000F000
  TIM_CCMR1_IC2F* = TIM_CCMR1_IC2F_Msk
  TIM_CCMR1_IC2F_0* = (0x00000001 shl TIM_CCMR1_IC2F_Pos) # !< 0x00001000
  TIM_CCMR1_IC2F_1* = (0x00000002 shl TIM_CCMR1_IC2F_Pos) # !< 0x00002000
  TIM_CCMR1_IC2F_2* = (0x00000004 shl TIM_CCMR1_IC2F_Pos) # !< 0x00004000
  TIM_CCMR1_IC2F_3* = (0x00000008 shl TIM_CCMR1_IC2F_Pos) # !< 0x00008000

# *****************  Bit definition for TIM_CCMR2 register  *****************

const
  TIM_CCMR2_CC3S_Pos* = (0)
  TIM_CCMR2_CC3S_Msk* = (0x00000003 shl TIM_CCMR2_CC3S_Pos) # !< 0x00000003
  TIM_CCMR2_CC3S* = TIM_CCMR2_CC3S_Msk
  TIM_CCMR2_CC3S_0* = (0x00000001 shl TIM_CCMR2_CC3S_Pos) # !< 0x00000001
  TIM_CCMR2_CC3S_1* = (0x00000002 shl TIM_CCMR2_CC3S_Pos) # !< 0x00000002
  TIM_CCMR2_OC3FE_Pos* = (2)
  TIM_CCMR2_OC3FE_Msk* = (0x00000001 shl TIM_CCMR2_OC3FE_Pos) # !< 0x00000004
  TIM_CCMR2_OC3FE* = TIM_CCMR2_OC3FE_Msk
  TIM_CCMR2_OC3PE_Pos* = (3)
  TIM_CCMR2_OC3PE_Msk* = (0x00000001 shl TIM_CCMR2_OC3PE_Pos) # !< 0x00000008
  TIM_CCMR2_OC3PE* = TIM_CCMR2_OC3PE_Msk
  TIM_CCMR2_OC3M_Pos* = (4)
  TIM_CCMR2_OC3M_Msk* = (0x00000007 shl TIM_CCMR2_OC3M_Pos) # !< 0x00000070
  TIM_CCMR2_OC3M* = TIM_CCMR2_OC3M_Msk
  TIM_CCMR2_OC3M_0* = (0x00000001 shl TIM_CCMR2_OC3M_Pos) # !< 0x00000010
  TIM_CCMR2_OC3M_1* = (0x00000002 shl TIM_CCMR2_OC3M_Pos) # !< 0x00000020
  TIM_CCMR2_OC3M_2* = (0x00000004 shl TIM_CCMR2_OC3M_Pos) # !< 0x00000040
  TIM_CCMR2_OC3CE_Pos* = (7)
  TIM_CCMR2_OC3CE_Msk* = (0x00000001 shl TIM_CCMR2_OC3CE_Pos) # !< 0x00000080
  TIM_CCMR2_OC3CE* = TIM_CCMR2_OC3CE_Msk
  TIM_CCMR2_CC4S_Pos* = (8)
  TIM_CCMR2_CC4S_Msk* = (0x00000003 shl TIM_CCMR2_CC4S_Pos) # !< 0x00000300
  TIM_CCMR2_CC4S* = TIM_CCMR2_CC4S_Msk
  TIM_CCMR2_CC4S_0* = (0x00000001 shl TIM_CCMR2_CC4S_Pos) # !< 0x00000100
  TIM_CCMR2_CC4S_1* = (0x00000002 shl TIM_CCMR2_CC4S_Pos) # !< 0x00000200
  TIM_CCMR2_OC4FE_Pos* = (10)
  TIM_CCMR2_OC4FE_Msk* = (0x00000001 shl TIM_CCMR2_OC4FE_Pos) # !< 0x00000400
  TIM_CCMR2_OC4FE* = TIM_CCMR2_OC4FE_Msk
  TIM_CCMR2_OC4PE_Pos* = (11)
  TIM_CCMR2_OC4PE_Msk* = (0x00000001 shl TIM_CCMR2_OC4PE_Pos) # !< 0x00000800
  TIM_CCMR2_OC4PE* = TIM_CCMR2_OC4PE_Msk
  TIM_CCMR2_OC4M_Pos* = (12)
  TIM_CCMR2_OC4M_Msk* = (0x00000007 shl TIM_CCMR2_OC4M_Pos) # !< 0x00007000
  TIM_CCMR2_OC4M* = TIM_CCMR2_OC4M_Msk
  TIM_CCMR2_OC4M_0* = (0x00000001 shl TIM_CCMR2_OC4M_Pos) # !< 0x00001000
  TIM_CCMR2_OC4M_1* = (0x00000002 shl TIM_CCMR2_OC4M_Pos) # !< 0x00002000
  TIM_CCMR2_OC4M_2* = (0x00000004 shl TIM_CCMR2_OC4M_Pos) # !< 0x00004000
  TIM_CCMR2_OC4CE_Pos* = (15)
  TIM_CCMR2_OC4CE_Msk* = (0x00000001 shl TIM_CCMR2_OC4CE_Pos) # !< 0x00008000
  TIM_CCMR2_OC4CE* = TIM_CCMR2_OC4CE_Msk

# ---------------------------------------------------------------------------

const
  TIM_CCMR2_IC3PSC_Pos* = (2)
  TIM_CCMR2_IC3PSC_Msk* = (0x00000003 shl TIM_CCMR2_IC3PSC_Pos) # !< 0x0000000C
  TIM_CCMR2_IC3PSC* = TIM_CCMR2_IC3PSC_Msk
  TIM_CCMR2_IC3PSC_0* = (0x00000001 shl TIM_CCMR2_IC3PSC_Pos) # !< 0x00000004
  TIM_CCMR2_IC3PSC_1* = (0x00000002 shl TIM_CCMR2_IC3PSC_Pos) # !< 0x00000008
  TIM_CCMR2_IC3F_Pos* = (4)
  TIM_CCMR2_IC3F_Msk* = (0x0000000F shl TIM_CCMR2_IC3F_Pos) # !< 0x000000F0
  TIM_CCMR2_IC3F* = TIM_CCMR2_IC3F_Msk
  TIM_CCMR2_IC3F_0* = (0x00000001 shl TIM_CCMR2_IC3F_Pos) # !< 0x00000010
  TIM_CCMR2_IC3F_1* = (0x00000002 shl TIM_CCMR2_IC3F_Pos) # !< 0x00000020
  TIM_CCMR2_IC3F_2* = (0x00000004 shl TIM_CCMR2_IC3F_Pos) # !< 0x00000040
  TIM_CCMR2_IC3F_3* = (0x00000008 shl TIM_CCMR2_IC3F_Pos) # !< 0x00000080
  TIM_CCMR2_IC4PSC_Pos* = (10)
  TIM_CCMR2_IC4PSC_Msk* = (0x00000003 shl TIM_CCMR2_IC4PSC_Pos) # !< 0x00000C00
  TIM_CCMR2_IC4PSC* = TIM_CCMR2_IC4PSC_Msk
  TIM_CCMR2_IC4PSC_0* = (0x00000001 shl TIM_CCMR2_IC4PSC_Pos) # !< 0x00000400
  TIM_CCMR2_IC4PSC_1* = (0x00000002 shl TIM_CCMR2_IC4PSC_Pos) # !< 0x00000800
  TIM_CCMR2_IC4F_Pos* = (12)
  TIM_CCMR2_IC4F_Msk* = (0x0000000F shl TIM_CCMR2_IC4F_Pos) # !< 0x0000F000
  TIM_CCMR2_IC4F* = TIM_CCMR2_IC4F_Msk
  TIM_CCMR2_IC4F_0* = (0x00000001 shl TIM_CCMR2_IC4F_Pos) # !< 0x00001000
  TIM_CCMR2_IC4F_1* = (0x00000002 shl TIM_CCMR2_IC4F_Pos) # !< 0x00002000
  TIM_CCMR2_IC4F_2* = (0x00000004 shl TIM_CCMR2_IC4F_Pos) # !< 0x00004000
  TIM_CCMR2_IC4F_3* = (0x00000008 shl TIM_CCMR2_IC4F_Pos) # !< 0x00008000

# ******************  Bit definition for TIM_CCER register  *****************

const
  TIM_CCER_CC1E_Pos* = (0)
  TIM_CCER_CC1E_Msk* = (0x00000001 shl TIM_CCER_CC1E_Pos) # !< 0x00000001
  TIM_CCER_CC1E* = TIM_CCER_CC1E_Msk
  TIM_CCER_CC1P_Pos* = (1)
  TIM_CCER_CC1P_Msk* = (0x00000001 shl TIM_CCER_CC1P_Pos) # !< 0x00000002
  TIM_CCER_CC1P* = TIM_CCER_CC1P_Msk
  TIM_CCER_CC1NE_Pos* = (2)
  TIM_CCER_CC1NE_Msk* = (0x00000001 shl TIM_CCER_CC1NE_Pos) # !< 0x00000004
  TIM_CCER_CC1NE* = TIM_CCER_CC1NE_Msk
  TIM_CCER_CC1NP_Pos* = (3)
  TIM_CCER_CC1NP_Msk* = (0x00000001 shl TIM_CCER_CC1NP_Pos) # !< 0x00000008
  TIM_CCER_CC1NP* = TIM_CCER_CC1NP_Msk
  TIM_CCER_CC2E_Pos* = (4)
  TIM_CCER_CC2E_Msk* = (0x00000001 shl TIM_CCER_CC2E_Pos) # !< 0x00000010
  TIM_CCER_CC2E* = TIM_CCER_CC2E_Msk
  TIM_CCER_CC2P_Pos* = (5)
  TIM_CCER_CC2P_Msk* = (0x00000001 shl TIM_CCER_CC2P_Pos) # !< 0x00000020
  TIM_CCER_CC2P* = TIM_CCER_CC2P_Msk
  TIM_CCER_CC2NE_Pos* = (6)
  TIM_CCER_CC2NE_Msk* = (0x00000001 shl TIM_CCER_CC2NE_Pos) # !< 0x00000040
  TIM_CCER_CC2NE* = TIM_CCER_CC2NE_Msk
  TIM_CCER_CC2NP_Pos* = (7)
  TIM_CCER_CC2NP_Msk* = (0x00000001 shl TIM_CCER_CC2NP_Pos) # !< 0x00000080
  TIM_CCER_CC2NP* = TIM_CCER_CC2NP_Msk
  TIM_CCER_CC3E_Pos* = (8)
  TIM_CCER_CC3E_Msk* = (0x00000001 shl TIM_CCER_CC3E_Pos) # !< 0x00000100
  TIM_CCER_CC3E* = TIM_CCER_CC3E_Msk
  TIM_CCER_CC3P_Pos* = (9)
  TIM_CCER_CC3P_Msk* = (0x00000001 shl TIM_CCER_CC3P_Pos) # !< 0x00000200
  TIM_CCER_CC3P* = TIM_CCER_CC3P_Msk
  TIM_CCER_CC3NE_Pos* = (10)
  TIM_CCER_CC3NE_Msk* = (0x00000001 shl TIM_CCER_CC3NE_Pos) # !< 0x00000400
  TIM_CCER_CC3NE* = TIM_CCER_CC3NE_Msk
  TIM_CCER_CC3NP_Pos* = (11)
  TIM_CCER_CC3NP_Msk* = (0x00000001 shl TIM_CCER_CC3NP_Pos) # !< 0x00000800
  TIM_CCER_CC3NP* = TIM_CCER_CC3NP_Msk
  TIM_CCER_CC4E_Pos* = (12)
  TIM_CCER_CC4E_Msk* = (0x00000001 shl TIM_CCER_CC4E_Pos) # !< 0x00001000
  TIM_CCER_CC4E* = TIM_CCER_CC4E_Msk
  TIM_CCER_CC4P_Pos* = (13)
  TIM_CCER_CC4P_Msk* = (0x00000001 shl TIM_CCER_CC4P_Pos) # !< 0x00002000
  TIM_CCER_CC4P* = TIM_CCER_CC4P_Msk
  TIM_CCER_CC4NP_Pos* = (15)
  TIM_CCER_CC4NP_Msk* = (0x00000001 shl TIM_CCER_CC4NP_Pos) # !< 0x00008000
  TIM_CCER_CC4NP* = TIM_CCER_CC4NP_Msk

# ******************  Bit definition for TIM_CNT register  ******************

const
  TIM_CNT_CNT_Pos* = (0)
  TIM_CNT_CNT_Msk* = (0xFFFFFFFF shl TIM_CNT_CNT_Pos) # !< 0xFFFFFFFF
  TIM_CNT_CNT* = TIM_CNT_CNT_Msk

# ******************  Bit definition for TIM_PSC register  ******************

const
  TIM_PSC_PSC_Pos* = (0)
  TIM_PSC_PSC_Msk* = (0x0000FFFF shl TIM_PSC_PSC_Pos) # !< 0x0000FFFF
  TIM_PSC_PSC* = TIM_PSC_PSC_Msk

# ******************  Bit definition for TIM_ARR register  ******************

const
  TIM_ARR_ARR_Pos* = (0)
  TIM_ARR_ARR_Msk* = (0xFFFFFFFF shl TIM_ARR_ARR_Pos) # !< 0xFFFFFFFF
  TIM_ARR_ARR* = TIM_ARR_ARR_Msk

# ******************  Bit definition for TIM_RCR register  ******************

const
  TIM_RCR_REP_Pos* = (0)
  TIM_RCR_REP_Msk* = (0x000000FF shl TIM_RCR_REP_Pos) # !< 0x000000FF
  TIM_RCR_REP* = TIM_RCR_REP_Msk

# ******************  Bit definition for TIM_CCR1 register  *****************

const
  TIM_CCR1_CCR1_Pos* = (0)
  TIM_CCR1_CCR1_Msk* = (0x0000FFFF shl TIM_CCR1_CCR1_Pos) # !< 0x0000FFFF
  TIM_CCR1_CCR1* = TIM_CCR1_CCR1_Msk

# ******************  Bit definition for TIM_CCR2 register  *****************

const
  TIM_CCR2_CCR2_Pos* = (0)
  TIM_CCR2_CCR2_Msk* = (0x0000FFFF shl TIM_CCR2_CCR2_Pos) # !< 0x0000FFFF
  TIM_CCR2_CCR2* = TIM_CCR2_CCR2_Msk

# ******************  Bit definition for TIM_CCR3 register  *****************

const
  TIM_CCR3_CCR3_Pos* = (0)
  TIM_CCR3_CCR3_Msk* = (0x0000FFFF shl TIM_CCR3_CCR3_Pos) # !< 0x0000FFFF
  TIM_CCR3_CCR3* = TIM_CCR3_CCR3_Msk

# ******************  Bit definition for TIM_CCR4 register  *****************

const
  TIM_CCR4_CCR4_Pos* = (0)
  TIM_CCR4_CCR4_Msk* = (0x0000FFFF shl TIM_CCR4_CCR4_Pos) # !< 0x0000FFFF
  TIM_CCR4_CCR4* = TIM_CCR4_CCR4_Msk

# ******************  Bit definition for TIM_BDTR register  *****************

const
  TIM_BDTR_DTG_Pos* = (0)
  TIM_BDTR_DTG_Msk* = (0x000000FF shl TIM_BDTR_DTG_Pos) # !< 0x000000FF
  TIM_BDTR_DTG* = TIM_BDTR_DTG_Msk
  TIM_BDTR_DTG_0* = (0x00000001 shl TIM_BDTR_DTG_Pos) # !< 0x00000001
  TIM_BDTR_DTG_1* = (0x00000002 shl TIM_BDTR_DTG_Pos) # !< 0x00000002
  TIM_BDTR_DTG_2* = (0x00000004 shl TIM_BDTR_DTG_Pos) # !< 0x00000004
  TIM_BDTR_DTG_3* = (0x00000008 shl TIM_BDTR_DTG_Pos) # !< 0x00000008
  TIM_BDTR_DTG_4* = (0x00000010 shl TIM_BDTR_DTG_Pos) # !< 0x00000010
  TIM_BDTR_DTG_5* = (0x00000020 shl TIM_BDTR_DTG_Pos) # !< 0x00000020
  TIM_BDTR_DTG_6* = (0x00000040 shl TIM_BDTR_DTG_Pos) # !< 0x00000040
  TIM_BDTR_DTG_7* = (0x00000080 shl TIM_BDTR_DTG_Pos) # !< 0x00000080
  TIM_BDTR_LOCK_Pos* = (8)
  TIM_BDTR_LOCK_Msk* = (0x00000003 shl TIM_BDTR_LOCK_Pos) # !< 0x00000300
  TIM_BDTR_LOCK* = TIM_BDTR_LOCK_Msk
  TIM_BDTR_LOCK_0* = (0x00000001 shl TIM_BDTR_LOCK_Pos) # !< 0x00000100
  TIM_BDTR_LOCK_1* = (0x00000002 shl TIM_BDTR_LOCK_Pos) # !< 0x00000200
  TIM_BDTR_OSSI_Pos* = (10)
  TIM_BDTR_OSSI_Msk* = (0x00000001 shl TIM_BDTR_OSSI_Pos) # !< 0x00000400
  TIM_BDTR_OSSI* = TIM_BDTR_OSSI_Msk
  TIM_BDTR_OSSR_Pos* = (11)
  TIM_BDTR_OSSR_Msk* = (0x00000001 shl TIM_BDTR_OSSR_Pos) # !< 0x00000800
  TIM_BDTR_OSSR* = TIM_BDTR_OSSR_Msk
  TIM_BDTR_BKE_Pos* = (12)
  TIM_BDTR_BKE_Msk* = (0x00000001 shl TIM_BDTR_BKE_Pos) # !< 0x00001000
  TIM_BDTR_BKE* = TIM_BDTR_BKE_Msk
  TIM_BDTR_BKP_Pos* = (13)
  TIM_BDTR_BKP_Msk* = (0x00000001 shl TIM_BDTR_BKP_Pos) # !< 0x00002000
  TIM_BDTR_BKP* = TIM_BDTR_BKP_Msk
  TIM_BDTR_AOE_Pos* = (14)
  TIM_BDTR_AOE_Msk* = (0x00000001 shl TIM_BDTR_AOE_Pos) # !< 0x00004000
  TIM_BDTR_AOE* = TIM_BDTR_AOE_Msk
  TIM_BDTR_MOE_Pos* = (15)
  TIM_BDTR_MOE_Msk* = (0x00000001 shl TIM_BDTR_MOE_Pos) # !< 0x00008000
  TIM_BDTR_MOE* = TIM_BDTR_MOE_Msk

# ******************  Bit definition for TIM_DCR register  ******************

const
  TIM_DCR_DBA_Pos* = (0)
  TIM_DCR_DBA_Msk* = (0x0000001F shl TIM_DCR_DBA_Pos) # !< 0x0000001F
  TIM_DCR_DBA* = TIM_DCR_DBA_Msk
  TIM_DCR_DBA_0* = (0x00000001 shl TIM_DCR_DBA_Pos) # !< 0x00000001
  TIM_DCR_DBA_1* = (0x00000002 shl TIM_DCR_DBA_Pos) # !< 0x00000002
  TIM_DCR_DBA_2* = (0x00000004 shl TIM_DCR_DBA_Pos) # !< 0x00000004
  TIM_DCR_DBA_3* = (0x00000008 shl TIM_DCR_DBA_Pos) # !< 0x00000008
  TIM_DCR_DBA_4* = (0x00000010 shl TIM_DCR_DBA_Pos) # !< 0x00000010
  TIM_DCR_DBL_Pos* = (8)
  TIM_DCR_DBL_Msk* = (0x0000001F shl TIM_DCR_DBL_Pos) # !< 0x00001F00
  TIM_DCR_DBL* = TIM_DCR_DBL_Msk
  TIM_DCR_DBL_0* = (0x00000001 shl TIM_DCR_DBL_Pos) # !< 0x00000100
  TIM_DCR_DBL_1* = (0x00000002 shl TIM_DCR_DBL_Pos) # !< 0x00000200
  TIM_DCR_DBL_2* = (0x00000004 shl TIM_DCR_DBL_Pos) # !< 0x00000400
  TIM_DCR_DBL_3* = (0x00000008 shl TIM_DCR_DBL_Pos) # !< 0x00000800
  TIM_DCR_DBL_4* = (0x00000010 shl TIM_DCR_DBL_Pos) # !< 0x00001000

# ******************  Bit definition for TIM_DMAR register  *****************

const
  TIM_DMAR_DMAB_Pos* = (0)
  TIM_DMAR_DMAB_Msk* = (0x0000FFFF shl TIM_DMAR_DMAB_Pos) # !< 0x0000FFFF
  TIM_DMAR_DMAB* = TIM_DMAR_DMAB_Msk

# ******************  Bit definition for TIM14_OR register  *******************

const
  TIM14_OR_TI1_RMP_Pos* = (0)
  TIM14_OR_TI1_RMP_Msk* = (0x00000003 shl TIM14_OR_TI1_RMP_Pos) # !< 0x00000003
  TIM14_OR_TI1_RMP* = TIM14_OR_TI1_RMP_Msk
  TIM14_OR_TI1_RMP_0* = (0x00000001 shl TIM14_OR_TI1_RMP_Pos) # !< 0x00000001
  TIM14_OR_TI1_RMP_1* = (0x00000002 shl TIM14_OR_TI1_RMP_Pos) # !< 0x00000002

# ****************************************************************************
##
#       Universal Synchronous Asynchronous Receiver Transmitter (USART)
##
# ****************************************************************************
# *****************  Bit definition for USART_CR1 register  ******************

const
  USART_CR1_UE_Pos* = (0)
  USART_CR1_UE_Msk* = (0x00000001 shl USART_CR1_UE_Pos) # !< 0x00000001
  USART_CR1_UE* = USART_CR1_UE_Msk
  USART_CR1_RE_Pos* = (2)
  USART_CR1_RE_Msk* = (0x00000001 shl USART_CR1_RE_Pos) # !< 0x00000004
  USART_CR1_RE* = USART_CR1_RE_Msk
  USART_CR1_TE_Pos* = (3)
  USART_CR1_TE_Msk* = (0x00000001 shl USART_CR1_TE_Pos) # !< 0x00000008
  USART_CR1_TE* = USART_CR1_TE_Msk
  USART_CR1_IDLEIE_Pos* = (4)
  USART_CR1_IDLEIE_Msk* = (0x00000001 shl USART_CR1_IDLEIE_Pos) # !< 0x00000010
  USART_CR1_IDLEIE* = USART_CR1_IDLEIE_Msk
  USART_CR1_RXNEIE_Pos* = (5)
  USART_CR1_RXNEIE_Msk* = (0x00000001 shl USART_CR1_RXNEIE_Pos) # !< 0x00000020
  USART_CR1_RXNEIE* = USART_CR1_RXNEIE_Msk
  USART_CR1_TCIE_Pos* = (6)
  USART_CR1_TCIE_Msk* = (0x00000001 shl USART_CR1_TCIE_Pos) # !< 0x00000040
  USART_CR1_TCIE* = USART_CR1_TCIE_Msk
  USART_CR1_TXEIE_Pos* = (7)
  USART_CR1_TXEIE_Msk* = (0x00000001 shl USART_CR1_TXEIE_Pos) # !< 0x00000080
  USART_CR1_TXEIE* = USART_CR1_TXEIE_Msk
  USART_CR1_PEIE_Pos* = (8)
  USART_CR1_PEIE_Msk* = (0x00000001 shl USART_CR1_PEIE_Pos) # !< 0x00000100
  USART_CR1_PEIE* = USART_CR1_PEIE_Msk
  USART_CR1_PS_Pos* = (9)
  USART_CR1_PS_Msk* = (0x00000001 shl USART_CR1_PS_Pos) # !< 0x00000200
  USART_CR1_PS* = USART_CR1_PS_Msk
  USART_CR1_PCE_Pos* = (10)
  USART_CR1_PCE_Msk* = (0x00000001 shl USART_CR1_PCE_Pos) # !< 0x00000400
  USART_CR1_PCE* = USART_CR1_PCE_Msk
  USART_CR1_WAKE_Pos* = (11)
  USART_CR1_WAKE_Msk* = (0x00000001 shl USART_CR1_WAKE_Pos) # !< 0x00000800
  USART_CR1_WAKE* = USART_CR1_WAKE_Msk
  USART_CR1_M_Pos* = (12)
  USART_CR1_M_Msk* = (0x00000001 shl USART_CR1_M_Pos) # !< 0x00001000
  USART_CR1_M* = USART_CR1_M_Msk
  USART_CR1_MME_Pos* = (13)
  USART_CR1_MME_Msk* = (0x00000001 shl USART_CR1_MME_Pos) # !< 0x00002000
  USART_CR1_MME* = USART_CR1_MME_Msk
  USART_CR1_CMIE_Pos* = (14)
  USART_CR1_CMIE_Msk* = (0x00000001 shl USART_CR1_CMIE_Pos) # !< 0x00004000
  USART_CR1_CMIE* = USART_CR1_CMIE_Msk
  USART_CR1_OVER8_Pos* = (15)
  USART_CR1_OVER8_Msk* = (0x00000001 shl USART_CR1_OVER8_Pos) # !< 0x00008000
  USART_CR1_OVER8* = USART_CR1_OVER8_Msk
  USART_CR1_DEDT_Pos* = (16)
  USART_CR1_DEDT_Msk* = (0x0000001F shl USART_CR1_DEDT_Pos) # !< 0x001F0000
  USART_CR1_DEDT* = USART_CR1_DEDT_Msk
  USART_CR1_DEDT_0* = (0x00000001 shl USART_CR1_DEDT_Pos) # !< 0x00010000
  USART_CR1_DEDT_1* = (0x00000002 shl USART_CR1_DEDT_Pos) # !< 0x00020000
  USART_CR1_DEDT_2* = (0x00000004 shl USART_CR1_DEDT_Pos) # !< 0x00040000
  USART_CR1_DEDT_3* = (0x00000008 shl USART_CR1_DEDT_Pos) # !< 0x00080000
  USART_CR1_DEDT_4* = (0x00000010 shl USART_CR1_DEDT_Pos) # !< 0x00100000
  USART_CR1_DEAT_Pos* = (21)
  USART_CR1_DEAT_Msk* = (0x0000001F shl USART_CR1_DEAT_Pos) # !< 0x03E00000
  USART_CR1_DEAT* = USART_CR1_DEAT_Msk
  USART_CR1_DEAT_0* = (0x00000001 shl USART_CR1_DEAT_Pos) # !< 0x00200000
  USART_CR1_DEAT_1* = (0x00000002 shl USART_CR1_DEAT_Pos) # !< 0x00400000
  USART_CR1_DEAT_2* = (0x00000004 shl USART_CR1_DEAT_Pos) # !< 0x00800000
  USART_CR1_DEAT_3* = (0x00000008 shl USART_CR1_DEAT_Pos) # !< 0x01000000
  USART_CR1_DEAT_4* = (0x00000010 shl USART_CR1_DEAT_Pos) # !< 0x02000000
  USART_CR1_RTOIE_Pos* = (26)
  USART_CR1_RTOIE_Msk* = (0x00000001 shl USART_CR1_RTOIE_Pos) # !< 0x04000000
  USART_CR1_RTOIE* = USART_CR1_RTOIE_Msk
  USART_CR1_EOBIE_Pos* = (27)
  USART_CR1_EOBIE_Msk* = (0x00000001 shl USART_CR1_EOBIE_Pos) # !< 0x08000000
  USART_CR1_EOBIE* = USART_CR1_EOBIE_Msk

# *****************  Bit definition for USART_CR2 register  ******************

const
  USART_CR2_ADDM7_Pos* = (4)
  USART_CR2_ADDM7_Msk* = (0x00000001 shl USART_CR2_ADDM7_Pos) # !< 0x00000010
  USART_CR2_ADDM7* = USART_CR2_ADDM7_Msk
  USART_CR2_LBCL_Pos* = (8)
  USART_CR2_LBCL_Msk* = (0x00000001 shl USART_CR2_LBCL_Pos) # !< 0x00000100
  USART_CR2_LBCL* = USART_CR2_LBCL_Msk
  USART_CR2_CPHA_Pos* = (9)
  USART_CR2_CPHA_Msk* = (0x00000001 shl USART_CR2_CPHA_Pos) # !< 0x00000200
  USART_CR2_CPHA* = USART_CR2_CPHA_Msk
  USART_CR2_CPOL_Pos* = (10)
  USART_CR2_CPOL_Msk* = (0x00000001 shl USART_CR2_CPOL_Pos) # !< 0x00000400
  USART_CR2_CPOL* = USART_CR2_CPOL_Msk
  USART_CR2_CLKEN_Pos* = (11)
  USART_CR2_CLKEN_Msk* = (0x00000001 shl USART_CR2_CLKEN_Pos) # !< 0x00000800
  USART_CR2_CLKEN* = USART_CR2_CLKEN_Msk
  USART_CR2_STOP_Pos* = (12)
  USART_CR2_STOP_Msk* = (0x00000003 shl USART_CR2_STOP_Pos) # !< 0x00003000
  USART_CR2_STOP* = USART_CR2_STOP_Msk
  USART_CR2_STOP_0* = (0x00000001 shl USART_CR2_STOP_Pos) # !< 0x00001000
  USART_CR2_STOP_1* = (0x00000002 shl USART_CR2_STOP_Pos) # !< 0x00002000
  USART_CR2_SWAP_Pos* = (15)
  USART_CR2_SWAP_Msk* = (0x00000001 shl USART_CR2_SWAP_Pos) # !< 0x00008000
  USART_CR2_SWAP* = USART_CR2_SWAP_Msk
  USART_CR2_RXINV_Pos* = (16)
  USART_CR2_RXINV_Msk* = (0x00000001 shl USART_CR2_RXINV_Pos) # !< 0x00010000
  USART_CR2_RXINV* = USART_CR2_RXINV_Msk
  USART_CR2_TXINV_Pos* = (17)
  USART_CR2_TXINV_Msk* = (0x00000001 shl USART_CR2_TXINV_Pos) # !< 0x00020000
  USART_CR2_TXINV* = USART_CR2_TXINV_Msk
  USART_CR2_DATAINV_Pos* = (18)
  USART_CR2_DATAINV_Msk* = (0x00000001 shl USART_CR2_DATAINV_Pos) # !< 0x00040000
  USART_CR2_DATAINV* = USART_CR2_DATAINV_Msk
  USART_CR2_MSBFIRST_Pos* = (19)
  USART_CR2_MSBFIRST_Msk* = (0x00000001 shl USART_CR2_MSBFIRST_Pos) # !< 0x00080000
  USART_CR2_MSBFIRST* = USART_CR2_MSBFIRST_Msk
  USART_CR2_ABREN_Pos* = (20)
  USART_CR2_ABREN_Msk* = (0x00000001 shl USART_CR2_ABREN_Pos) # !< 0x00100000
  USART_CR2_ABREN* = USART_CR2_ABREN_Msk
  USART_CR2_ABRMODE_Pos* = (21)
  USART_CR2_ABRMODE_Msk* = (0x00000003 shl USART_CR2_ABRMODE_Pos) # !< 0x00600000
  USART_CR2_ABRMODE* = USART_CR2_ABRMODE_Msk
  USART_CR2_ABRMODE_0* = (0x00000001 shl USART_CR2_ABRMODE_Pos) # !< 0x00200000
  USART_CR2_ABRMODE_1* = (0x00000002 shl USART_CR2_ABRMODE_Pos) # !< 0x00400000
  USART_CR2_RTOEN_Pos* = (23)
  USART_CR2_RTOEN_Msk* = (0x00000001 shl USART_CR2_RTOEN_Pos) # !< 0x00800000
  USART_CR2_RTOEN* = USART_CR2_RTOEN_Msk
  USART_CR2_ADD_Pos* = (24)
  USART_CR2_ADD_Msk* = (0x000000FF shl USART_CR2_ADD_Pos) # !< 0xFF000000
  USART_CR2_ADD* = USART_CR2_ADD_Msk

# *****************  Bit definition for USART_CR3 register  ******************

const
  USART_CR3_EIE_Pos* = (0)
  USART_CR3_EIE_Msk* = (0x00000001 shl USART_CR3_EIE_Pos) # !< 0x00000001
  USART_CR3_EIE* = USART_CR3_EIE_Msk
  USART_CR3_HDSEL_Pos* = (3)
  USART_CR3_HDSEL_Msk* = (0x00000001 shl USART_CR3_HDSEL_Pos) # !< 0x00000008
  USART_CR3_HDSEL* = USART_CR3_HDSEL_Msk
  USART_CR3_DMAR_Pos* = (6)
  USART_CR3_DMAR_Msk* = (0x00000001 shl USART_CR3_DMAR_Pos) # !< 0x00000040
  USART_CR3_DMAR* = USART_CR3_DMAR_Msk
  USART_CR3_DMAT_Pos* = (7)
  USART_CR3_DMAT_Msk* = (0x00000001 shl USART_CR3_DMAT_Pos) # !< 0x00000080
  USART_CR3_DMAT* = USART_CR3_DMAT_Msk
  USART_CR3_RTSE_Pos* = (8)
  USART_CR3_RTSE_Msk* = (0x00000001 shl USART_CR3_RTSE_Pos) # !< 0x00000100
  USART_CR3_RTSE* = USART_CR3_RTSE_Msk
  USART_CR3_CTSE_Pos* = (9)
  USART_CR3_CTSE_Msk* = (0x00000001 shl USART_CR3_CTSE_Pos) # !< 0x00000200
  USART_CR3_CTSE* = USART_CR3_CTSE_Msk
  USART_CR3_CTSIE_Pos* = (10)
  USART_CR3_CTSIE_Msk* = (0x00000001 shl USART_CR3_CTSIE_Pos) # !< 0x00000400
  USART_CR3_CTSIE* = USART_CR3_CTSIE_Msk
  USART_CR3_ONEBIT_Pos* = (11)
  USART_CR3_ONEBIT_Msk* = (0x00000001 shl USART_CR3_ONEBIT_Pos) # !< 0x00000800
  USART_CR3_ONEBIT* = USART_CR3_ONEBIT_Msk
  USART_CR3_OVRDIS_Pos* = (12)
  USART_CR3_OVRDIS_Msk* = (0x00000001 shl USART_CR3_OVRDIS_Pos) # !< 0x00001000
  USART_CR3_OVRDIS* = USART_CR3_OVRDIS_Msk
  USART_CR3_DDRE_Pos* = (13)
  USART_CR3_DDRE_Msk* = (0x00000001 shl USART_CR3_DDRE_Pos) # !< 0x00002000
  USART_CR3_DDRE* = USART_CR3_DDRE_Msk
  USART_CR3_DEM_Pos* = (14)
  USART_CR3_DEM_Msk* = (0x00000001 shl USART_CR3_DEM_Pos) # !< 0x00004000
  USART_CR3_DEM* = USART_CR3_DEM_Msk
  USART_CR3_DEP_Pos* = (15)
  USART_CR3_DEP_Msk* = (0x00000001 shl USART_CR3_DEP_Pos) # !< 0x00008000
  USART_CR3_DEP* = USART_CR3_DEP_Msk

# *****************  Bit definition for USART_BRR register  ******************

const
  USART_BRR_DIV_FRACTION_Pos* = (0)
  USART_BRR_DIV_FRACTION_Msk* = (0x0000000F shl USART_BRR_DIV_FRACTION_Pos) # !< 0x0000000F
  USART_BRR_DIV_FRACTION* = USART_BRR_DIV_FRACTION_Msk
  USART_BRR_DIV_MANTISSA_Pos* = (4)
  USART_BRR_DIV_MANTISSA_Msk* = (0x00000FFF shl USART_BRR_DIV_MANTISSA_Pos) # !< 0x0000FFF0
  USART_BRR_DIV_MANTISSA* = USART_BRR_DIV_MANTISSA_Msk

# *****************  Bit definition for USART_GTPR register  *****************

const
  USART_GTPR_PSC_Pos* = (0)
  USART_GTPR_PSC_Msk* = (0x000000FF shl USART_GTPR_PSC_Pos) # !< 0x000000FF
  USART_GTPR_PSC* = USART_GTPR_PSC_Msk
  USART_GTPR_GT_Pos* = (8)
  USART_GTPR_GT_Msk* = (0x000000FF shl USART_GTPR_GT_Pos) # !< 0x0000FF00
  USART_GTPR_GT* = USART_GTPR_GT_Msk

# ******************  Bit definition for USART_RTOR register  ****************

const
  USART_RTOR_RTO_Pos* = (0)
  USART_RTOR_RTO_Msk* = (0x00FFFFFF shl USART_RTOR_RTO_Pos) # !< 0x00FFFFFF
  USART_RTOR_RTO* = USART_RTOR_RTO_Msk
  USART_RTOR_BLEN_Pos* = (24)
  USART_RTOR_BLEN_Msk* = (0x000000FF shl USART_RTOR_BLEN_Pos) # !< 0xFF000000
  USART_RTOR_BLEN* = USART_RTOR_BLEN_Msk

# ******************  Bit definition for USART_RQR register  *****************

const
  USART_RQR_ABRRQ_Pos* = (0)
  USART_RQR_ABRRQ_Msk* = (0x00000001 shl USART_RQR_ABRRQ_Pos) # !< 0x00000001
  USART_RQR_ABRRQ* = USART_RQR_ABRRQ_Msk
  USART_RQR_SBKRQ_Pos* = (1)
  USART_RQR_SBKRQ_Msk* = (0x00000001 shl USART_RQR_SBKRQ_Pos) # !< 0x00000002
  USART_RQR_SBKRQ* = USART_RQR_SBKRQ_Msk
  USART_RQR_MMRQ_Pos* = (2)
  USART_RQR_MMRQ_Msk* = (0x00000001 shl USART_RQR_MMRQ_Pos) # !< 0x00000004
  USART_RQR_MMRQ* = USART_RQR_MMRQ_Msk
  USART_RQR_RXFRQ_Pos* = (3)
  USART_RQR_RXFRQ_Msk* = (0x00000001 shl USART_RQR_RXFRQ_Pos) # !< 0x00000008
  USART_RQR_RXFRQ* = USART_RQR_RXFRQ_Msk

# ******************  Bit definition for USART_ISR register  *****************

const
  USART_ISR_PE_Pos* = (0)
  USART_ISR_PE_Msk* = (0x00000001 shl USART_ISR_PE_Pos) # !< 0x00000001
  USART_ISR_PE* = USART_ISR_PE_Msk
  USART_ISR_FE_Pos* = (1)
  USART_ISR_FE_Msk* = (0x00000001 shl USART_ISR_FE_Pos) # !< 0x00000002
  USART_ISR_FE* = USART_ISR_FE_Msk
  USART_ISR_NE_Pos* = (2)
  USART_ISR_NE_Msk* = (0x00000001 shl USART_ISR_NE_Pos) # !< 0x00000004
  USART_ISR_NE* = USART_ISR_NE_Msk
  USART_ISR_ORE_Pos* = (3)
  USART_ISR_ORE_Msk* = (0x00000001 shl USART_ISR_ORE_Pos) # !< 0x00000008
  USART_ISR_ORE* = USART_ISR_ORE_Msk
  USART_ISR_IDLE_Pos* = (4)
  USART_ISR_IDLE_Msk* = (0x00000001 shl USART_ISR_IDLE_Pos) # !< 0x00000010
  USART_ISR_IDLE* = USART_ISR_IDLE_Msk
  USART_ISR_RXNE_Pos* = (5)
  USART_ISR_RXNE_Msk* = (0x00000001 shl USART_ISR_RXNE_Pos) # !< 0x00000020
  USART_ISR_RXNE* = USART_ISR_RXNE_Msk
  USART_ISR_TC_Pos* = (6)
  USART_ISR_TC_Msk* = (0x00000001 shl USART_ISR_TC_Pos) # !< 0x00000040
  USART_ISR_TC* = USART_ISR_TC_Msk
  USART_ISR_TXE_Pos* = (7)
  USART_ISR_TXE_Msk* = (0x00000001 shl USART_ISR_TXE_Pos) # !< 0x00000080
  USART_ISR_TXE* = USART_ISR_TXE_Msk
  USART_ISR_CTSIF_Pos* = (9)
  USART_ISR_CTSIF_Msk* = (0x00000001 shl USART_ISR_CTSIF_Pos) # !< 0x00000200
  USART_ISR_CTSIF* = USART_ISR_CTSIF_Msk
  USART_ISR_CTS_Pos* = (10)
  USART_ISR_CTS_Msk* = (0x00000001 shl USART_ISR_CTS_Pos) # !< 0x00000400
  USART_ISR_CTS* = USART_ISR_CTS_Msk
  USART_ISR_RTOF_Pos* = (11)
  USART_ISR_RTOF_Msk* = (0x00000001 shl USART_ISR_RTOF_Pos) # !< 0x00000800
  USART_ISR_RTOF* = USART_ISR_RTOF_Msk
  USART_ISR_ABRE_Pos* = (14)
  USART_ISR_ABRE_Msk* = (0x00000001 shl USART_ISR_ABRE_Pos) # !< 0x00004000
  USART_ISR_ABRE* = USART_ISR_ABRE_Msk
  USART_ISR_ABRF_Pos* = (15)
  USART_ISR_ABRF_Msk* = (0x00000001 shl USART_ISR_ABRF_Pos) # !< 0x00008000
  USART_ISR_ABRF* = USART_ISR_ABRF_Msk
  USART_ISR_BUSY_Pos* = (16)
  USART_ISR_BUSY_Msk* = (0x00000001 shl USART_ISR_BUSY_Pos) # !< 0x00010000
  USART_ISR_BUSY* = USART_ISR_BUSY_Msk
  USART_ISR_CMF_Pos* = (17)
  USART_ISR_CMF_Msk* = (0x00000001 shl USART_ISR_CMF_Pos) # !< 0x00020000
  USART_ISR_CMF* = USART_ISR_CMF_Msk
  USART_ISR_SBKF_Pos* = (18)
  USART_ISR_SBKF_Msk* = (0x00000001 shl USART_ISR_SBKF_Pos) # !< 0x00040000
  USART_ISR_SBKF* = USART_ISR_SBKF_Msk
  USART_ISR_RWU_Pos* = (19)
  USART_ISR_RWU_Msk* = (0x00000001 shl USART_ISR_RWU_Pos) # !< 0x00080000
  USART_ISR_RWU* = USART_ISR_RWU_Msk
  USART_ISR_TEACK_Pos* = (21)
  USART_ISR_TEACK_Msk* = (0x00000001 shl USART_ISR_TEACK_Pos) # !< 0x00200000
  USART_ISR_TEACK* = USART_ISR_TEACK_Msk
  USART_ISR_REACK_Pos* = (22)
  USART_ISR_REACK_Msk* = (0x00000001 shl USART_ISR_REACK_Pos) # !< 0x00400000
  USART_ISR_REACK* = USART_ISR_REACK_Msk

# ******************  Bit definition for USART_ICR register  *****************

const
  USART_ICR_PECF_Pos* = (0)
  USART_ICR_PECF_Msk* = (0x00000001 shl USART_ICR_PECF_Pos) # !< 0x00000001
  USART_ICR_PECF* = USART_ICR_PECF_Msk
  USART_ICR_FECF_Pos* = (1)
  USART_ICR_FECF_Msk* = (0x00000001 shl USART_ICR_FECF_Pos) # !< 0x00000002
  USART_ICR_FECF* = USART_ICR_FECF_Msk
  USART_ICR_NCF_Pos* = (2)
  USART_ICR_NCF_Msk* = (0x00000001 shl USART_ICR_NCF_Pos) # !< 0x00000004
  USART_ICR_NCF* = USART_ICR_NCF_Msk
  USART_ICR_ORECF_Pos* = (3)
  USART_ICR_ORECF_Msk* = (0x00000001 shl USART_ICR_ORECF_Pos) # !< 0x00000008
  USART_ICR_ORECF* = USART_ICR_ORECF_Msk
  USART_ICR_IDLECF_Pos* = (4)
  USART_ICR_IDLECF_Msk* = (0x00000001 shl USART_ICR_IDLECF_Pos) # !< 0x00000010
  USART_ICR_IDLECF* = USART_ICR_IDLECF_Msk
  USART_ICR_TCCF_Pos* = (6)
  USART_ICR_TCCF_Msk* = (0x00000001 shl USART_ICR_TCCF_Pos) # !< 0x00000040
  USART_ICR_TCCF* = USART_ICR_TCCF_Msk
  USART_ICR_CTSCF_Pos* = (9)
  USART_ICR_CTSCF_Msk* = (0x00000001 shl USART_ICR_CTSCF_Pos) # !< 0x00000200
  USART_ICR_CTSCF* = USART_ICR_CTSCF_Msk
  USART_ICR_RTOCF_Pos* = (11)
  USART_ICR_RTOCF_Msk* = (0x00000001 shl USART_ICR_RTOCF_Pos) # !< 0x00000800
  USART_ICR_RTOCF* = USART_ICR_RTOCF_Msk
  USART_ICR_CMCF_Pos* = (17)
  USART_ICR_CMCF_Msk* = (0x00000001 shl USART_ICR_CMCF_Pos) # !< 0x00020000
  USART_ICR_CMCF* = USART_ICR_CMCF_Msk

# ******************  Bit definition for USART_RDR register  *****************

const
  USART_RDR_RDR* = (0x000001FF).uint16 # !< RDR[8:0] bits (Receive Data value)

# ******************  Bit definition for USART_TDR register  *****************

const
  USART_TDR_TDR* = (0x000001FF).uint16 # !< TDR[8:0] bits (Transmit Data value)

# ****************************************************************************
##
#                          Window WATCHDOG (WWDG)
##
# ****************************************************************************
# ******************  Bit definition for WWDG_CR register  *******************

const
  WWDG_CR_T_Pos* = (0)
  WWDG_CR_T_Msk* = (0x0000007F shl WWDG_CR_T_Pos) # !< 0x0000007F
  WWDG_CR_T* = WWDG_CR_T_Msk
  WWDG_CR_T_0* = (0x00000001 shl WWDG_CR_T_Pos) # !< 0x00000001
  WWDG_CR_T_1* = (0x00000002 shl WWDG_CR_T_Pos) # !< 0x00000002
  WWDG_CR_T_2* = (0x00000004 shl WWDG_CR_T_Pos) # !< 0x00000004
  WWDG_CR_T_3* = (0x00000008 shl WWDG_CR_T_Pos) # !< 0x00000008
  WWDG_CR_T_4* = (0x00000010 shl WWDG_CR_T_Pos) # !< 0x00000010
  WWDG_CR_T_5* = (0x00000020 shl WWDG_CR_T_Pos) # !< 0x00000020
  WWDG_CR_T_6* = (0x00000040 shl WWDG_CR_T_Pos) # !< 0x00000040

#  Legacy defines

const
#WWDG_CR_T0* = WWDG_CR_T_0
#  WWDG_CR_T1* = WWDG_CR_T_1
#  WWDG_CR_T2* = WWDG_CR_T_2
#  WWDG_CR_T3* = WWDG_CR_T_3
#  WWDG_CR_T4* = WWDG_CR_T_4
#  WWDG_CR_T5* = WWDG_CR_T_5
#  WWDG_CR_T6* = WWDG_CR_T_6
  WWDG_CR_WDGA_Pos* = (7)
  WWDG_CR_WDGA_Msk* = (0x00000001 shl WWDG_CR_WDGA_Pos) # !< 0x00000080
  WWDG_CR_WDGA* = WWDG_CR_WDGA_Msk

# ******************  Bit definition for WWDG_CFR register  ******************

const
  WWDG_CFR_W_Pos* = (0)
  WWDG_CFR_W_Msk* = (0x0000007F shl WWDG_CFR_W_Pos) # !< 0x0000007F
  WWDG_CFR_W* = WWDG_CFR_W_Msk
  WWDG_CFR_W_0* = (0x00000001 shl WWDG_CFR_W_Pos) # !< 0x00000001
  WWDG_CFR_W_1* = (0x00000002 shl WWDG_CFR_W_Pos) # !< 0x00000002
  WWDG_CFR_W_2* = (0x00000004 shl WWDG_CFR_W_Pos) # !< 0x00000004
  WWDG_CFR_W_3* = (0x00000008 shl WWDG_CFR_W_Pos) # !< 0x00000008
  WWDG_CFR_W_4* = (0x00000010 shl WWDG_CFR_W_Pos) # !< 0x00000010
  WWDG_CFR_W_5* = (0x00000020 shl WWDG_CFR_W_Pos) # !< 0x00000020
  WWDG_CFR_W_6* = (0x00000040 shl WWDG_CFR_W_Pos) # !< 0x00000040

#  Legacy defines

const
#   WWDG_CFR_W0* = WWDG_CFR_W_0
#   WWDG_CFR_W1* = WWDG_CFR_W_1
#   WWDG_CFR_W2* = WWDG_CFR_W_2
#   WWDG_CFR_W3* = WWDG_CFR_W_3
#   WWDG_CFR_W4* = WWDG_CFR_W_4
#   WWDG_CFR_W5* = WWDG_CFR_W_5
#   WWDG_CFR_W6* = WWDG_CFR_W_6
  WWDG_CFR_WDGTB_Pos* = (7)
  WWDG_CFR_WDGTB_Msk* = (0x00000003 shl WWDG_CFR_WDGTB_Pos) # !< 0x00000180
  WWDG_CFR_WDGTB* = WWDG_CFR_WDGTB_Msk
  WWDG_CFR_WDGTB_0* = (0x00000001 shl WWDG_CFR_WDGTB_Pos) # !< 0x00000080
  WWDG_CFR_WDGTB_1* = (0x00000002 shl WWDG_CFR_WDGTB_Pos) # !< 0x00000100

#  Legacy defines

const
#WWDG_CFR_WDGTB0* = WWDG_CFR_WDGTB_0
#WWDG_CFR_WDGTB1* = WWDG_CFR_WDGTB_1
  WWDG_CFR_EWI_Pos* = (9)
  WWDG_CFR_EWI_Msk* = (0x00000001 shl WWDG_CFR_EWI_Pos) # !< 0x00000200
  WWDG_CFR_EWI* = WWDG_CFR_EWI_Msk

# ******************  Bit definition for WWDG_SR register  *******************

const
  WWDG_SR_EWIF_Pos* = (0)
  WWDG_SR_EWIF_Msk* = (0x00000001 shl WWDG_SR_EWIF_Pos) # !< 0x00000001
  WWDG_SR_EWIF* = WWDG_SR_EWIF_Msk

# *
#  @}
##
# *
#  @}
##
# * @addtogroup Exported_macro
#  @{
##
# ***************************** ADC Instances ********************************

template IS_ADC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == ADC1)

template IS_ADC_COMMON_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == ADC)

# ***************************** CRC Instances ********************************

template IS_CRC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == CRC)

# ****************************** DMA Instances *******************************

template IS_DMA_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == DMA1_Channel1) or ((INSTANCE) == DMA1_Channel2) or
      ((INSTANCE) == DMA1_Channel3) or ((INSTANCE) == DMA1_Channel4) or
      ((INSTANCE) == DMA1_Channel5))

# ***************************** GPIO Instances *******************************

template IS_GPIO_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == GPIOA) or ((INSTANCE) == GPIOB) or ((INSTANCE) == GPIOC) or
      ((INSTANCE) == GPIOD) or ((INSTANCE) == GPIOF))

# *************************** GPIO Alternate Function Instances **************

template IS_GPIO_AF_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == GPIOA) or ((INSTANCE) == GPIOB))

# ***************************** GPIO Lock Instances **************************

template IS_GPIO_LOCK_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == GPIOA) or ((INSTANCE) == GPIOB))

# ***************************** I2C Instances ********************************

template IS_I2C_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == I2C1) or ((INSTANCE) == I2C2))

# ***************************** IWDG Instances *******************************

template IS_IWDG_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == IWDG)

# ***************************** RTC Instances ********************************

template IS_RTC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == RTC)

# ***************************** SMBUS Instances ********************************

template IS_SMBUS_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == I2C1)

# ***************************** SPI Instances ********************************

template IS_SPI_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == SPI1) or ((INSTANCE) == SPI2))

# ***************************** TIM Instances ********************************

template IS_TIM_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM6) or
      ((INSTANCE) == TIM14) or ((INSTANCE) == TIM15) or ((INSTANCE) == TIM16) or
      ((INSTANCE) == TIM17))

template IS_TIM_CC1_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM14) or
      ((INSTANCE) == TIM15) or ((INSTANCE) == TIM16) or ((INSTANCE) == TIM17))

template IS_TIM_CC2_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM15))

template IS_TIM_CC3_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3))

template IS_TIM_CC4_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3))

template IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3))

template IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3))

template IS_TIM_CLOCKSOURCE_TIX_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM15))

template IS_TIM_CLOCKSOURCE_ITRX_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM15))

template IS_TIM_OCXREF_CLEAR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3))

template IS_TIM_ENCODER_INTERFACE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3))

template IS_TIM_HALL_INTERFACE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1))

template IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1))

template IS_TIM_XOR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3))

template IS_TIM_MASTER_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM15))

template IS_TIM_SLAVE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM15))

template IS_TIM_32B_COUNTER_INSTANCE*(INSTANCE: untyped): untyped =
  (0)

template IS_TIM_DMABURST_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM15) or
      ((INSTANCE) == TIM16) or ((INSTANCE) == TIM17))

template IS_TIM_BREAK_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM15) or ((INSTANCE) == TIM16) or
      ((INSTANCE) == TIM17))

template IS_TIM_CCX_INSTANCE*(INSTANCE, CHANNEL: untyped): untyped =
  ((((INSTANCE) == TIM1) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3) or ((CHANNEL) == TIM_CHANNEL_4))) or
      (((INSTANCE) == TIM3) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3) or ((CHANNEL) == TIM_CHANNEL_4))) or
      (((INSTANCE) == TIM14) and (((CHANNEL) == TIM_CHANNEL_1))) or
      (((INSTANCE) == TIM15) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2))) or
      (((INSTANCE) == TIM16) and (((CHANNEL) == TIM_CHANNEL_1))) or
      (((INSTANCE) == TIM17) and (((CHANNEL) == TIM_CHANNEL_1))))

template IS_TIM_CCXN_INSTANCE*(INSTANCE, CHANNEL: untyped): untyped =
  ((((INSTANCE) == TIM1) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3))) or
      (((INSTANCE) == TIM15) and ((CHANNEL) == TIM_CHANNEL_1)) or
      (((INSTANCE) == TIM16) and ((CHANNEL) == TIM_CHANNEL_1)) or
      (((INSTANCE) == TIM17) and ((CHANNEL) == TIM_CHANNEL_1)))

template IS_TIM_COUNTER_MODE_SELECT_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3))

template IS_TIM_REPETITION_COUNTER_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM15) or ((INSTANCE) == TIM16) or
      ((INSTANCE) == TIM17))

template IS_TIM_CLOCK_DIVISION_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM14) or
      ((INSTANCE) == TIM15) or ((INSTANCE) == TIM16) or ((INSTANCE) == TIM17))

template IS_TIM_DMA_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM6) or
      ((INSTANCE) == TIM15) or ((INSTANCE) == TIM16) or ((INSTANCE) == TIM17))

template IS_TIM_DMA_CC_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM3) or ((INSTANCE) == TIM15) or
      ((INSTANCE) == TIM16) or ((INSTANCE) == TIM17))

template IS_TIM_COMMUTATION_EVENT_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM15) or ((INSTANCE) == TIM16) or
      ((INSTANCE) == TIM17))

template IS_TIM_REMAP_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == TIM14)

template IS_TIM_ADVANCED_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == TIM1)

# ******************* USART Instances : Synchronous mode *********************

template IS_USART_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2))

# ******************* USART Instances : auto Baud rate detection *************

template IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == USART1)

# ******************* UART Instances : Asynchronous mode *********************

template IS_UART_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2))

# ******************* UART Instances : Half-Duplex mode *********************

template IS_UART_HALFDUPLEX_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2))

# ***************** UART Instances : Hardware Flow control *******************

template IS_UART_HWFLOW_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2))

# ***************** UART Instances : Driver enable detection *******************

template IS_UART_DRIVER_ENABLE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2))

# ***************************** WWDG Instances *******************************

template IS_WWDG_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == WWDG)

# *
#  @}
##
# ****************************************************************************
#   For a painless codes migration between the STM32F0xx device product
#   lines, the aliases defined below are put in place to overcome the
#   differences in the interrupt handlers and IRQn definitions.
#   No need to update developed interrupt code when moving across
#   product lines within the same STM32F0 Family
# ****************************************************************************
#  Aliases for IRQn

const
  ADC1_COMP_IRQn* = ADC1_IRQn
  DMA1_Ch1_IRQn* = DMA1_Channel1_IRQn
  DMA1_Ch2_3_DMA2_Ch1_2_IRQn* = DMA1_Channel2_3_IRQn
  DMA1_Channel4_5_6_7_IRQn* = DMA1_Channel4_5_IRQn
  DMA1_Ch4_7_DMA2_Ch3_5_IRQn* = DMA1_Channel4_5_IRQn
  RCC_CRS_IRQn* = RCC_IRQn
  TIM6_DAC_IRQn* = TIM6_IRQn

#  Aliases for IRQHandler
when false:
  const
    ADC1_COMP_IRQHandler* = ADC1_IRQHandler
    DMA1_Ch1_IRQHandler* = DMA1_Channel1_IRQHandler
    DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler* = DMA1_Channel2_3_IRQHandler
    DMA1_Channel4_5_6_7_IRQHandler* = DMA1_Channel4_5_IRQHandler
    DMA1_Ch4_7_DMA2_Ch3_5_IRQHandler* = DMA1_Channel4_5_IRQHandler
    RCC_CRS_IRQHandler* = RCC_IRQHandler
    TIM6_DAC_IRQHandler* = TIM6_IRQHandler

# *
#  @}
##
# *
#  @}
##
# *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE***
include "reg_utils"
include "core_cm0"

