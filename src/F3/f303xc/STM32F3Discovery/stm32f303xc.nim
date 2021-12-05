## *
## *****************************************************************************
##  @file    stm32f303xc.h
##  @author  MCD Application Team
##  @brief   CMSIS STM32F303xC Devices Peripheral Access Layer Header File.
##
##           This file contains:
##            - Data structures and the address mapping for all peripherals
##            - Peripheral's registers declarations and bits definition
##            - Macros to access peripherals registers hardware
##
## *****************************************************************************
##  @attention
##
##  <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
## * @addtogroup CMSIS_Device
##  @{
##
## * @addtogroup stm32f303xc
##  @{
##

## * @addtogroup Configuration_section_for_CMSIS
##  @{
##
## *
##  @brief Configuration of the Cortex-M4 Processor and Core Peripherals
##

const
  CM4_REV* = 0x00000001
  MPU_PRESENT* = 1
  NVIC_PRIO_BITS* = 4
  Vendor_SysTickConfig* = 0
  FPU_PRESENT* = 1

## *
##  @}
##
## * @addtogroup Peripheral_interrupt_number_definition
##  @{
##
## *
##  @brief STM32F303xC devices Interrupt Number Definition, according to the selected device
##         in @ref Library_configuration_section
##

type                          ## *****  Cortex-M4 Processor Exceptions Numbers ***************************************************************
  IRQn_Type* = enum
    NonMaskableInt_IRQn = -14,  ## !< 2 Non Maskable Interrupt
    HardFault_IRQn = -13,       ## !< 3 Cortex-M4 Hard Fault Interrupt
    MemoryManagement_IRQn = -12, ## !< 4 Cortex-M4 Memory Management Interrupt
    BusFault_IRQn = -11,        ## !< 5 Cortex-M4 Bus Fault Interrupt
    UsageFault_IRQn = -10,      ## !< 6 Cortex-M4 Usage Fault Interrupt
    SVCall_IRQn = -5,           ## !< 11 Cortex-M4 SV Call Interrupt
    DebugMonitor_IRQn = -4,     ## !< 12 Cortex-M4 Debug Monitor Interrupt
    PendSV_IRQn = -2,           ## !< 14 Cortex-M4 Pend SV Interrupt
    SysTick_IRQn = -1,          ## !< 15 Cortex-M4 System Tick Interrupt
                    ## *****  STM32 specific Interrupt Numbers *********************************************************************
    WWDG_IRQn = 0,              ## !< Window WatchDog Interrupt
    PVD_IRQn = 1,               ## !< PVD through EXTI Line detection Interrupt
    TAMP_STAMP_IRQn = 2,        ## !< Tamper and TimeStamp interrupts through the EXTI line 19
    RTC_WKUP_IRQn = 3,          ## !< RTC Wakeup interrupt through the EXTI line 20
    FLASH_IRQn = 4,             ## !< FLASH global Interrupt
    RCC_IRQn = 5,               ## !< RCC global Interrupt
    EXTI0_IRQn = 6,             ## !< EXTI Line0 Interrupt
    EXTI1_IRQn = 7,             ## !< EXTI Line1 Interrupt
    EXTI2_TSC_IRQn = 8,         ## !< EXTI Line2 Interrupt and Touch Sense Controller Interrupt
    EXTI3_IRQn = 9,             ## !< EXTI Line3 Interrupt
    EXTI4_IRQn = 10,            ## !< EXTI Line4 Interrupt
    DMA1_Channel1_IRQn = 11,    ## !< DMA1 Channel 1 Interrupt
    DMA1_Channel2_IRQn = 12,    ## !< DMA1 Channel 2 Interrupt
    DMA1_Channel3_IRQn = 13,    ## !< DMA1 Channel 3 Interrupt
    DMA1_Channel4_IRQn = 14,    ## !< DMA1 Channel 4 Interrupt
    DMA1_Channel5_IRQn = 15,    ## !< DMA1 Channel 5 Interrupt
    DMA1_Channel6_IRQn = 16,    ## !< DMA1 Channel 6 Interrupt
    DMA1_Channel7_IRQn = 17,    ## !< DMA1 Channel 7 Interrupt
    ADC1_2_IRQn = 18,           ## !< ADC1 & ADC2 Interrupts
    USB_HP_CAN_TX_IRQn = 19,    ## !< USB Device High Priority or CAN TX Interrupts
    USB_LP_CAN_RX0_IRQn = 20,   ## !< USB Device Low Priority or CAN RX0 Interrupts
    CAN_RX1_IRQn = 21,          ## !< CAN RX1 Interrupt
    CAN_SCE_IRQn = 22,          ## !< CAN SCE Interrupt
    EXTI9_5_IRQn = 23,          ## !< External Line[9:5] Interrupts
    TIM1_BRK_TIM15_IRQn = 24,   ## !< TIM1 Break and TIM15 Interrupts
    TIM1_UP_TIM16_IRQn = 25,    ## !< TIM1 Update and TIM16 Interrupts
    TIM1_TRG_COM_TIM17_IRQn = 26, ## !< TIM1 Trigger and Commutation and TIM17 Interrupt
    TIM1_CC_IRQn = 27,          ## !< TIM1 Capture Compare Interrupt
    TIM2_IRQn = 28,             ## !< TIM2 global Interrupt
    TIM3_IRQn = 29,             ## !< TIM3 global Interrupt
    TIM4_IRQn = 30,             ## !< TIM4 global Interrupt
    I2C1_EV_IRQn = 31,          ## !< I2C1 Event Interrupt & EXTI Line23 Interrupt (I2C1 wakeup)
    I2C1_ER_IRQn = 32,          ## !< I2C1 Error Interrupt
    I2C2_EV_IRQn = 33,          ## !< I2C2 Event Interrupt & EXTI Line24 Interrupt (I2C2 wakeup)
    I2C2_ER_IRQn = 34,          ## !< I2C2 Error Interrupt
    SPI1_IRQn = 35,             ## !< SPI1 global Interrupt
    SPI2_IRQn = 36,             ## !< SPI2 global Interrupt
    USART1_IRQn = 37,           ## !< USART1 global Interrupt & EXTI Line25 Interrupt (USART1 wakeup)
    USART2_IRQn = 38,           ## !< USART2 global Interrupt & EXTI Line26 Interrupt (USART2 wakeup)
    USART3_IRQn = 39,           ## !< USART3 global Interrupt & EXTI Line28 Interrupt (USART3 wakeup)
    EXTI15_10_IRQn = 40,        ## !< External Line[15:10] Interrupts
    RTC_Alarm_IRQn = 41,        ## !< RTC Alarm (A and B) through EXTI Line 17 Interrupt
    USBWakeUp_IRQn = 42,        ## !< USB Wakeup Interrupt
    TIM8_BRK_IRQn = 43,         ## !< TIM8 Break Interrupt
    TIM8_UP_IRQn = 44,          ## !< TIM8 Update Interrupt
    TIM8_TRG_COM_IRQn = 45,     ## !< TIM8 Trigger and Commutation Interrupt
    TIM8_CC_IRQn = 46,          ## !< TIM8 Capture Compare Interrupt
    ADC3_IRQn = 47,             ## !< ADC3 global Interrupt
    SPI3_IRQn = 51,             ## !< SPI3 global Interrupt
    UART4_IRQn = 52,            ## !< UART4 global Interrupt & EXTI Line34 Interrupt (UART4 wakeup)
    UART5_IRQn = 53,            ## !< UART5 global Interrupt & EXTI Line35 Interrupt (UART5 wakeup)
    TIM6_DAC_IRQn = 54,         ## !< TIM6 global and DAC underrun error Interrupt
    TIM7_IRQn = 55,             ## !< TIM7 global Interrupt
    DMA2_Channel1_IRQn = 56,    ## !< DMA2 Channel 1 global Interrupt
    DMA2_Channel2_IRQn = 57,    ## !< DMA2 Channel 2 global Interrupt
    DMA2_Channel3_IRQn = 58,    ## !< DMA2 Channel 3 global Interrupt
    DMA2_Channel4_IRQn = 59,    ## !< DMA2 Channel 4 global Interrupt
    DMA2_Channel5_IRQn = 60,    ## !< DMA2 Channel 5 global Interrupt
    ADC4_IRQn = 61,             ## !< ADC4  global Interrupt
    COMP1_2_3_IRQn = 64,        ## !< COMP1, COMP2 and COMP3 global Interrupt via EXTI Line21, 22 and 29
    COMP4_5_6_IRQn = 65,        ## !< COMP4, COMP5 and COMP6 global Interrupt via EXTI Line30, 31 and 32
    COMP7_IRQn = 66,            ## !< COMP7 global Interrupt via EXTI Line33
    USB_HP_IRQn = 74,           ## !< USB High Priority global Interrupt
    USB_LP_IRQn = 75,           ## !< USB Low Priority global Interrupt
    USBWakeUp_RMP_IRQn = 76,    ## !< USB Wakeup Interrupt remap
    FPU_IRQn = 81               ## !< Floating point Interrupt


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
    ISR*: uint32               ## !< ADC Interrupt and Status Register,                 Address offset: 0x00
    IER*: uint32               ## !< ADC Interrupt Enable Register,                     Address offset: 0x04
    CR*: uint32                ## !< ADC control register,                              Address offset: 0x08
    CFGR*: uint32              ## !< ADC Configuration register,                        Address offset: 0x0C
    RESERVED0*: uint32         ## !< Reserved, 0x010
    SMPR1*: uint32             ## !< ADC sample time register 1,                        Address offset: 0x14
    SMPR2*: uint32             ## !< ADC sample time register 2,                        Address offset: 0x18
    RESERVED1*: uint32         ## !< Reserved, 0x01C
    TR1*: uint32               ## !< ADC watchdog threshold register 1,                 Address offset: 0x20
    TR2*: uint32               ## !< ADC watchdog threshold register 2,                 Address offset: 0x24
    TR3*: uint32               ## !< ADC watchdog threshold register 3,                 Address offset: 0x28
    RESERVED2*: uint32         ## !< Reserved, 0x02C
    SQR1*: uint32              ## !< ADC regular sequence register 1,                   Address offset: 0x30
    SQR2*: uint32              ## !< ADC regular sequence register 2,                   Address offset: 0x34
    SQR3*: uint32              ## !< ADC regular sequence register 3,                   Address offset: 0x38
    SQR4*: uint32              ## !< ADC regular sequence register 4,                   Address offset: 0x3C
    DR*: uint32                ## !< ADC regular data register,                         Address offset: 0x40
    RESERVED3*: uint32         ## !< Reserved, 0x044
    RESERVED4*: uint32         ## !< Reserved, 0x048
    JSQR*: uint32              ## !< ADC injected sequence register,                    Address offset: 0x4C
    RESERVED5*: array[4, uint32] ## !< Reserved, 0x050 - 0x05C
    OFR1*: uint32              ## !< ADC offset register 1,                             Address offset: 0x60
    OFR2*: uint32              ## !< ADC offset register 2,                             Address offset: 0x64
    OFR3*: uint32              ## !< ADC offset register 3,                             Address offset: 0x68
    OFR4*: uint32              ## !< ADC offset register 4,                             Address offset: 0x6C
    RESERVED6*: array[4, uint32] ## !< Reserved, 0x070 - 0x07C
    JDR1*: uint32              ## !< ADC injected data register 1,                      Address offset: 0x80
    JDR2*: uint32              ## !< ADC injected data register 2,                      Address offset: 0x84
    JDR3*: uint32              ## !< ADC injected data register 3,                      Address offset: 0x88
    JDR4*: uint32              ## !< ADC injected data register 4,                      Address offset: 0x8C
    RESERVED7*: array[4, uint32] ## !< Reserved, 0x090 - 0x09C
    AWD2CR*: uint32            ## !< ADC  Analog Watchdog 2 Configuration Register,     Address offset: 0xA0
    AWD3CR*: uint32            ## !< ADC  Analog Watchdog 3 Configuration Register,     Address offset: 0xA4
    RESERVED8*: uint32         ## !< Reserved, 0x0A8
    RESERVED9*: uint32         ## !< Reserved, 0x0AC
    DIFSEL*: uint32            ## !< ADC  Differential Mode Selection Register,         Address offset: 0xB0
    CALFACT*: uint32           ## !< ADC  Calibration Factors,                          Address offset: 0xB4

  ADC_Common_TypeDef* {.bycopy.} = object
    CSR*: uint32               ## !< ADC Common status register,                  Address offset: ADC1/3 base address + 0x300
    RESERVED*: uint32          ## !< Reserved, ADC1/3 base address + 0x304
    CCR*: uint32               ## !< ADC common control register,                 Address offset: ADC1/3 base address + 0x308
    CDR*: uint32               ## !< ADC common regular data register for dual
               ##                                      AND triple modes,                            Address offset: ADC1/3 base address + 0x30C


## *
##  @brief Controller Area Network TxMailBox
##

type
  CAN_TxMailBox_TypeDef* {.bycopy.} = object
    TIR*: uint32               ## !< CAN TX mailbox identifier register
    TDTR*: uint32              ## !< CAN mailbox data length control and time stamp register
    TDLR*: uint32              ## !< CAN mailbox data low register
    TDHR*: uint32              ## !< CAN mailbox data high register


## *
##  @brief Controller Area Network FIFOMailBox
##

type
  CAN_FIFOMailBox_TypeDef* {.bycopy.} = object
    RIR*: uint32               ## !< CAN receive FIFO mailbox identifier register
    RDTR*: uint32              ## !< CAN receive FIFO mailbox data length control and time stamp register
    RDLR*: uint32              ## !< CAN receive FIFO mailbox data low register
    RDHR*: uint32              ## !< CAN receive FIFO mailbox data high register


## *
##  @brief Controller Area Network FilterRegister
##

type
  CAN_FilterRegister_TypeDef* {.bycopy.} = object
    FR1*: uint32               ## !< CAN Filter bank register 1
    FR2*: uint32               ## !< CAN Filter bank register 1


## *
##  @brief Controller Area Network
##

type
  CAN_TypeDef* {.bycopy.} = object
    MCR*: uint32               ## !< CAN master control register,         Address offset: 0x00
    MSR*: uint32               ## !< CAN master status register,          Address offset: 0x04
    TSR*: uint32               ## !< CAN transmit status register,        Address offset: 0x08
    RF0R*: uint32              ## !< CAN receive FIFO 0 register,         Address offset: 0x0C
    RF1R*: uint32              ## !< CAN receive FIFO 1 register,         Address offset: 0x10
    IER*: uint32               ## !< CAN interrupt enable register,       Address offset: 0x14
    ESR*: uint32               ## !< CAN error status register,           Address offset: 0x18
    BTR*: uint32               ## !< CAN bit timing register,             Address offset: 0x1C
    RESERVED0*: array[88, uint32] ## !< Reserved, 0x020 - 0x17F
    sTxMailBox*: array[3, CAN_TxMailBox_TypeDef] ## !< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC
    sFIFOMailBox*: array[2, CAN_FIFOMailBox_TypeDef] ## !< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC
    RESERVED1*: array[12, uint32] ## !< Reserved, 0x1D0 - 0x1FF
    FMR*: uint32               ## !< CAN filter master register,          Address offset: 0x200
    FM1R*: uint32              ## !< CAN filter mode register,            Address offset: 0x204
    RESERVED2*: uint32         ## !< Reserved, 0x208
    FS1R*: uint32              ## !< CAN filter scale register,           Address offset: 0x20C
    RESERVED3*: uint32         ## !< Reserved, 0x210
    FFA1R*: uint32             ## !< CAN filter FIFO assignment register, Address offset: 0x214
    RESERVED4*: uint32         ## !< Reserved, 0x218
    FA1R*: uint32              ## !< CAN filter activation register,      Address offset: 0x21C
    RESERVED5*: array[8, uint32] ## !< Reserved, 0x220-0x23F
    sFilterRegister*: array[28, CAN_FilterRegister_TypeDef] ## !< CAN Filter Register,                 Address offset: 0x240-0x31C


## *
##  @brief Analog Comparators
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
    RESERVED0*: uint8          ## !< Reserved,                                                    0x05
    RESERVED1*: uint16         ## !< Reserved,                                                    0x06
    CR*: uint32                ## !< CRC Control register,                        Address offset: 0x08
    RESERVED2*: uint32         ## !< Reserved,                                                    0x0C
    INIT*: uint32              ## !< Initial CRC value register,                  Address offset: 0x10
    POL*: uint32               ## !< CRC polynomial register,                     Address offset: 0x14


## *
##  @brief Digital to Analog Converter
##

type
  DAC_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< DAC control register,                                    Address offset: 0x00
    SWTRIGR*: uint32           ## !< DAC software trigger register,                           Address offset: 0x04
    DHR12R1*: uint32           ## !< DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08
    DHR12L1*: uint32           ## !< DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C
    DHR8R1*: uint32            ## !< DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10
    DHR12R2*: uint32           ## !< DAC channel2 12-bit right aligned data holding register, Address offset: 0x14
    DHR12L2*: uint32           ## !< DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18
    DHR8R2*: uint32            ## !< DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C
    DHR12RD*: uint32           ## !< Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20
    DHR12LD*: uint32           ## !< DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24
    DHR8RD*: uint32            ## !< DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28
    DOR1*: uint32              ## !< DAC channel1 data output register,                       Address offset: 0x2C
    DOR2*: uint32              ## !< DAC channel2 data output register,                       Address offset: 0x30
    SR*: uint32                ## !< DAC status register,                                     Address offset: 0x34


## *
##  @brief Debug MCU
##

type
  DBGMCU_TypeDef* {.bycopy.} = object
    IDCODE*: uint32            ## !< MCU device ID code,               Address offset: 0x00
    CR*: uint32                ## !< Debug MCU configuration register, Address offset: 0x04
    APB1FZ*: uint32            ## !< Debug MCU APB1 freeze register,   Address offset: 0x08
    APB2FZ*: uint32            ## !< Debug MCU APB2 freeze register,   Address offset: 0x0C


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
    ISR*: uint32               ## !< DMA interrupt status register,                            Address offset: 0x00
    IFCR*: uint32              ## !< DMA interrupt flag clear register,                        Address offset: 0x04


## *
##  @brief External Interrupt/Event Controller
##

type
  EXTI_TypeDef* {.bycopy.} = object
    IMR*: uint32               ## !<EXTI Interrupt mask register,                             Address offset: 0x00
    EMR*: uint32               ## !<EXTI Event mask register,                                 Address offset: 0x04
    RTSR*: uint32              ## !<EXTI Rising trigger selection register ,                  Address offset: 0x08
    FTSR*: uint32              ## !<EXTI Falling trigger selection register,                  Address offset: 0x0C
    SWIER*: uint32             ## !<EXTI Software interrupt event register,                   Address offset: 0x10
    PR*: uint32                ## !<EXTI Pending register,                                    Address offset: 0x14
    RESERVED1*: uint32         ## !< Reserved, 0x18
    RESERVED2*: uint32         ## !< Reserved, 0x1C
    IMR2*: uint32              ## !< EXTI Interrupt mask register,                            Address offset: 0x20
    EMR2*: uint32              ## !< EXTI Event mask register,                                Address offset: 0x24
    RTSR2*: uint32             ## !< EXTI Rising trigger selection register,                  Address offset: 0x28
    FTSR2*: uint32             ## !< EXTI Falling trigger selection register,                 Address offset: 0x2C
    SWIER2*: uint32            ## !< EXTI Software interrupt event register,                  Address offset: 0x30
    PR2*: uint32               ## !< EXTI Pending register,                                   Address offset: 0x34


## *
##  @brief FLASH Registers
##

type
  FLASH_TypeDef* {.bycopy.} = object
    ACR*: uint32               ## !< FLASH access control register,              Address offset: 0x00
    KEYR*: uint32              ## !< FLASH key register,                         Address offset: 0x04
    OPTKEYR*: uint32           ## !< FLASH option key register,                  Address offset: 0x08
    SR*: uint32                ## !< FLASH status register,                      Address offset: 0x0C
    CR*: uint32                ## !< FLASH control register,                     Address offset: 0x10
    AR*: uint32                ## !< FLASH address register,                     Address offset: 0x14
    RESERVED*: uint32          ## !< Reserved, 0x18
    OBR*: uint32               ## !< FLASH Option byte register,                 Address offset: 0x1C
    WRPR*: uint32              ## !< FLASH Write register,                       Address offset: 0x20


## *
##  @brief Option Bytes Registers
##

type
  OB_TypeDef* {.bycopy.} = object
    RDP*: uint16               ## !<FLASH option byte Read protection,             Address offset: 0x00
    USER*: uint16              ## !<FLASH option byte user options,                Address offset: 0x02
    RESERVED0*: uint16         ## !< Reserved,                                                     0x04
    RESERVED1*: uint16         ## !< Reserved,                                                     0x06
    WRP0*: uint16              ## !<FLASH option byte write protection 0,          Address offset: 0x08
    WRP1*: uint16              ## !<FLASH option byte write protection 1,          Address offset: 0x0C
    WRP2*: uint16              ## !<FLASH option byte write protection 2,          Address offset: 0x10
    WRP3*: uint16              ## !<FLASH option byte write protection 3,          Address offset: 0x12


## *
##  @brief General Purpose I/O
##

type
  GPIO_TypeDef* {.bycopy.} = object
    MODER*: uint32             ## !< GPIO port mode register,               Address offset: 0x00
    OTYPER*: uint32            ## !< GPIO port output type register,        Address offset: 0x04
    OSPEEDR*: uint32           ## !< GPIO port output speed register,       Address offset: 0x08
    PUPDR*: uint32             ## !< GPIO port pull-up/pull-down register,  Address offset: 0x0C
    IDR*: uint32               ## !< GPIO port input data register,         Address offset: 0x10
    ODR*: uint32               ## !< GPIO port output data register,        Address offset: 0x14
    BSRR*: uint32              ## !< GPIO port bit set/reset register,      Address offset: 0x1A
    LCKR*: uint32              ## !< GPIO port configuration lock register, Address offset: 0x1C
    AFR*: array[2, uint32]      ## !< GPIO alternate function registers,     Address offset: 0x20-0x24
    BRR*: uint32               ## !< GPIO bit reset register,               Address offset: 0x28


## *
##  @brief Operational Amplifier (OPAMP)
##

type
  OPAMP_TypeDef* {.bycopy.} = object
    CSR*: uint32               ## !< OPAMP control and status register,            Address offset: 0x00


## *
##  @brief System configuration controller
##

type
  SYSCFG_TypeDef* {.bycopy.} = object
    CFGR1*: uint32             ## !< SYSCFG configuration register 1,                      Address offset: 0x00
    RCR*: uint32               ## !< SYSCFG CCM SRAM protection register,               Address offset: 0x04
    EXTICR*: array[4, uint32]   ## !< SYSCFG external interrupt configuration registers, Address offset: 0x14-0x08
    CFGR2*: uint32             ## !< SYSCFG configuration register 2,                      Address offset: 0x18


## *
##  @brief Inter-integrated Circuit Interface
##

type
  I2C_TypeDef* {.bycopy.} = object
    CR1*: uint32               ## !< I2C Control register 1,            Address offset: 0x00
    CR2*: uint32               ## !< I2C Control register 2,            Address offset: 0x04
    OAR1*: uint32              ## !< I2C Own address 1 register,        Address offset: 0x08
    OAR2*: uint32              ## !< I2C Own address 2 register,        Address offset: 0x0C
    TIMINGR*: uint32           ## !< I2C Timing register,               Address offset: 0x10
    TIMEOUTR*: uint32          ## !< I2C Timeout register,              Address offset: 0x14
    ISR*: uint32               ## !< I2C Interrupt and status register, Address offset: 0x18
    ICR*: uint32               ## !< I2C Interrupt clear register,      Address offset: 0x1C
    PECR*: uint32              ## !< I2C PEC register,                  Address offset: 0x20
    RXDR*: uint32              ## !< I2C Receive data register,         Address offset: 0x24
    TXDR*: uint32              ## !< I2C Transmit data register,        Address offset: 0x28


## *
##  @brief Independent WATCHDOG
##

type
  IWDG_TypeDef* {.bycopy.} = object
    KR*: uint32                ## !< IWDG Key register,       Address offset: 0x00
    PR*: uint32                ## !< IWDG Prescaler register, Address offset: 0x04
    RLR*: uint32               ## !< IWDG Reload register,    Address offset: 0x08
    SR*: uint32                ## !< IWDG Status register,    Address offset: 0x0C
    WINR*: uint32              ## !< IWDG Window register,    Address offset: 0x10


## *
##  @brief Power Control
##

type
  PWR_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< PWR power control register,        Address offset: 0x00
    CSR*: uint32               ## !< PWR power control/status register, Address offset: 0x04


## *
##  @brief Reset and Clock Control
##

type
  RCC_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< RCC clock control register,                                  Address offset: 0x00
    CFGR*: uint32              ## !< RCC clock configuration register,                            Address offset: 0x04
    CIR*: uint32               ## !< RCC clock interrupt register,                                Address offset: 0x08
    APB2RSTR*: uint32          ## !< RCC APB2 peripheral reset register,                          Address offset: 0x0C
    APB1RSTR*: uint32          ## !< RCC APB1 peripheral reset register,                          Address offset: 0x10
    AHBENR*: uint32            ## !< RCC AHB peripheral clock register,                           Address offset: 0x14
    APB2ENR*: uint32           ## !< RCC APB2 peripheral clock enable register,                   Address offset: 0x18
    APB1ENR*: uint32           ## !< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C
    BDCR*: uint32              ## !< RCC Backup domain control register,                          Address offset: 0x20
    CSR*: uint32               ## !< RCC clock control & status register,                         Address offset: 0x24
    AHBRSTR*: uint32           ## !< RCC AHB peripheral reset register,                           Address offset: 0x28
    CFGR2*: uint32             ## !< RCC clock configuration register 2,                          Address offset: 0x2C
    CFGR3*: uint32             ## !< RCC clock configuration register 3,                          Address offset: 0x30


## *
##  @brief Real-Time Clock
##

type
  RTC_TypeDef* {.bycopy.} = object
    TR*: uint32                ## !< RTC time register,                                        Address offset: 0x00
    DR*: uint32                ## !< RTC date register,                                        Address offset: 0x04
    CR*: uint32                ## !< RTC control register,                                     Address offset: 0x08
    ISR*: uint32               ## !< RTC initialization and status register,                   Address offset: 0x0C
    PRER*: uint32              ## !< RTC prescaler register,                                   Address offset: 0x10
    WUTR*: uint32              ## !< RTC wakeup timer register,                                Address offset: 0x14
    RESERVED0*: uint32         ## !< Reserved, 0x18
    ALRMAR*: uint32            ## !< RTC alarm A register,                                     Address offset: 0x1C
    ALRMBR*: uint32            ## !< RTC alarm B register,                                     Address offset: 0x20
    WPR*: uint32               ## !< RTC write protection register,                            Address offset: 0x24
    SSR*: uint32               ## !< RTC sub second register,                                  Address offset: 0x28
    SHIFTR*: uint32            ## !< RTC shift control register,                               Address offset: 0x2C
    TSTR*: uint32              ## !< RTC time stamp time register,                             Address offset: 0x30
    TSDR*: uint32              ## !< RTC time stamp date register,                             Address offset: 0x34
    TSSSR*: uint32             ## !< RTC time-stamp sub second register,                       Address offset: 0x38
    CALR*: uint32              ## !< RTC calibration register,                                 Address offset: 0x3C
    TAFCR*: uint32             ## !< RTC tamper and alternate function configuration register, Address offset: 0x40
    ALRMASSR*: uint32          ## !< RTC alarm A sub second register,                          Address offset: 0x44
    ALRMBSSR*: uint32          ## !< RTC alarm B sub second register,                          Address offset: 0x48
    RESERVED7*: uint32         ## !< Reserved, 0x4C
    BKP0R*: uint32             ## !< RTC backup register 0,                                    Address offset: 0x50
    BKP1R*: uint32             ## !< RTC backup register 1,                                    Address offset: 0x54
    BKP2R*: uint32             ## !< RTC backup register 2,                                    Address offset: 0x58
    BKP3R*: uint32             ## !< RTC backup register 3,                                    Address offset: 0x5C
    BKP4R*: uint32             ## !< RTC backup register 4,                                    Address offset: 0x60
    BKP5R*: uint32             ## !< RTC backup register 5,                                    Address offset: 0x64
    BKP6R*: uint32             ## !< RTC backup register 6,                                    Address offset: 0x68
    BKP7R*: uint32             ## !< RTC backup register 7,                                    Address offset: 0x6C
    BKP8R*: uint32             ## !< RTC backup register 8,                                    Address offset: 0x70
    BKP9R*: uint32             ## !< RTC backup register 9,                                    Address offset: 0x74
    BKP10R*: uint32            ## !< RTC backup register 10,                                   Address offset: 0x78
    BKP11R*: uint32            ## !< RTC backup register 11,                                   Address offset: 0x7C
    BKP12R*: uint32            ## !< RTC backup register 12,                                   Address offset: 0x80
    BKP13R*: uint32            ## !< RTC backup register 13,                                   Address offset: 0x84
    BKP14R*: uint32            ## !< RTC backup register 14,                                   Address offset: 0x88
    BKP15R*: uint32            ## !< RTC backup register 15,                                   Address offset: 0x8C


## *
##  @brief Serial Peripheral Interface
##

type
  SPI_TypeDef* {.bycopy.} = object
    CR1*: uint32               ## !< SPI Control register 1,                              Address offset: 0x00
    CR2*: uint32               ## !< SPI Control register 2,                              Address offset: 0x04
    SR*: uint32                ## !< SPI Status register,                                 Address offset: 0x08
    DR*: uint32                ## !< SPI data register,                                   Address offset: 0x0C
    CRCPR*: uint32             ## !< SPI CRC polynomial register,                         Address offset: 0x10
    RXCRCR*: uint32            ## !< SPI Rx CRC register,                                 Address offset: 0x14
    TXCRCR*: uint32            ## !< SPI Tx CRC register,                                 Address offset: 0x18
    I2SCFGR*: uint32           ## !< SPI_I2S configuration register,                      Address offset: 0x1C
    I2SPR*: uint32             ## !< SPI_I2S prescaler register,                          Address offset: 0x20


## *
##  @brief TIM
##

type
  TIM_TypeDef* {.bycopy.} = object
    CR1*: uint32               ## !< TIM control register 1,              Address offset: 0x00
    CR2*: uint32               ## !< TIM control register 2,              Address offset: 0x04
    SMCR*: uint32              ## !< TIM slave mode control register,     Address offset: 0x08
    DIER*: uint32              ## !< TIM DMA/interrupt enable register,   Address offset: 0x0C
    SR*: uint32                ## !< TIM status register,                 Address offset: 0x10
    EGR*: uint32               ## !< TIM event generation register,       Address offset: 0x14
    CCMR1*: uint32             ## !< TIM capture/compare mode register 1, Address offset: 0x18
    CCMR2*: uint32             ## !< TIM capture/compare mode register 2, Address offset: 0x1C
    CCER*: uint32              ## !< TIM capture/compare enable register, Address offset: 0x20
    CNT*: uint32               ## !< TIM counter register,                Address offset: 0x24
    PSC*: uint32               ## !< TIM prescaler,                       Address offset: 0x28
    ARR*: uint32               ## !< TIM auto-reload register,            Address offset: 0x2C
    RCR*: uint32               ## !< TIM repetition counter register,     Address offset: 0x30
    CCR1*: uint32              ## !< TIM capture/compare register 1,      Address offset: 0x34
    CCR2*: uint32              ## !< TIM capture/compare register 2,      Address offset: 0x38
    CCR3*: uint32              ## !< TIM capture/compare register 3,      Address offset: 0x3C
    CCR4*: uint32              ## !< TIM capture/compare register 4,      Address offset: 0x40
    BDTR*: uint32              ## !< TIM break and dead-time register,    Address offset: 0x44
    DCR*: uint32               ## !< TIM DMA control register,            Address offset: 0x48
    DMAR*: uint32              ## !< TIM DMA address for full transfer,   Address offset: 0x4C
    OR*: uint32                ## !< TIM option register,                 Address offset: 0x50
    CCMR3*: uint32             ## !< TIM capture/compare mode register 3, Address offset: 0x54
    CCR5*: uint32              ## !< TIM capture/compare register5,       Address offset: 0x58
    CCR6*: uint32              ## !< TIM capture/compare register 4,      Address offset: 0x5C


## *
##  @brief Touch Sensing Controller (TSC)
##

type
  TSC_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< TSC control register,                                     Address offset: 0x00
    IER*: uint32               ## !< TSC interrupt enable register,                            Address offset: 0x04
    ICR*: uint32               ## !< TSC interrupt clear register,                             Address offset: 0x08
    ISR*: uint32               ## !< TSC interrupt status register,                            Address offset: 0x0C
    IOHCR*: uint32             ## !< TSC I/O hysteresis control register,                      Address offset: 0x10
    RESERVED1*: uint32         ## !< Reserved,                                                 Address offset: 0x14
    IOASCR*: uint32            ## !< TSC I/O analog switch control register,                   Address offset: 0x18
    RESERVED2*: uint32         ## !< Reserved,                                                 Address offset: 0x1C
    IOSCR*: uint32             ## !< TSC I/O sampling control register,                        Address offset: 0x20
    RESERVED3*: uint32         ## !< Reserved,                                                 Address offset: 0x24
    IOCCR*: uint32             ## !< TSC I/O channel control register,                         Address offset: 0x28
    RESERVED4*: uint32         ## !< Reserved,                                                 Address offset: 0x2C
    IOGCSR*: uint32            ## !< TSC I/O group control status register,                    Address offset: 0x30
    IOGXCR*: array[8, uint32]   ## !< TSC I/O group x counter register,                         Address offset: 0x34-50


## *
##  @brief Universal Synchronous Asynchronous Receiver Transmitter
##

type
  USART_TypeDef* {.bycopy.} = object
    CR1*: uint32               ## !< USART Control register 1,                 Address offset: 0x00
    CR2*: uint32               ## !< USART Control register 2,                 Address offset: 0x04
    CR3*: uint32               ## !< USART Control register 3,                 Address offset: 0x08
    BRR*: uint32               ## !< USART Baud rate register,                 Address offset: 0x0C
    GTPR*: uint32              ## !< USART Guard time and prescaler register,  Address offset: 0x10
    RTOR*: uint32              ## !< USART Receiver Time Out register,         Address offset: 0x14
    RQR*: uint32               ## !< USART Request register,                   Address offset: 0x18
    ISR*: uint32               ## !< USART Interrupt and status register,      Address offset: 0x1C
    ICR*: uint32               ## !< USART Interrupt flag Clear register,      Address offset: 0x20
    RDR*: uint16               ## !< USART Receive Data register,              Address offset: 0x24
    RESERVED1*: uint16         ## !< Reserved, 0x26
    TDR*: uint16               ## !< USART Transmit Data register,             Address offset: 0x28
    RESERVED2*: uint16         ## !< Reserved, 0x2A


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


## * @addtogroup Peripheral_memory_map
##  @{
##

const
  FLASH_BASE* = (cast[uint32](0x08000000)) ## !< FLASH base address in the alias region
  CCMDATARAM_BASE* = (cast[uint32](0x10000000)) ## !< CCM(core coupled memory) data RAM base address in the alias region
  SRAM_BASE* = (cast[uint32](0x20000000)) ## !< SRAM base address in the alias region
  PERIPH_BASE* = (cast[uint32](0x40000000)) ## !< Peripheral base address in the alias region
  SRAM_BB_BASE* = (cast[uint32](0x22000000)) ## !< SRAM base address in the bit-band region
  PERIPH_BB_BASE* = (cast[uint32](0x42000000)) ## !< Peripheral base address in the bit-band region

## !< Peripheral memory map

const
  APB1PERIPH_BASE* = PERIPH_BASE
  APB2PERIPH_BASE* = (PERIPH_BASE + 0x00010000)
  AHB1PERIPH_BASE* = (PERIPH_BASE + 0x00020000)
  AHB2PERIPH_BASE* = (PERIPH_BASE + 0x08000000)
  AHB3PERIPH_BASE* = (PERIPH_BASE + 0x10000000)

## !< APB1 peripherals

const
  TIM2_BASE* = (APB1PERIPH_BASE + 0x00000000)
  TIM3_BASE* = (APB1PERIPH_BASE + 0x00000400)
  TIM4_BASE* = (APB1PERIPH_BASE + 0x00000800)
  TIM6_BASE* = (APB1PERIPH_BASE + 0x00001000)
  TIM7_BASE* = (APB1PERIPH_BASE + 0x00001400)
  RTC_BASE* = (APB1PERIPH_BASE + 0x00002800)
  WWDG_BASE* = (APB1PERIPH_BASE + 0x00002C00)
  IWDG_BASE* = (APB1PERIPH_BASE + 0x00003000)
  I2S2ext_BASE* = (APB1PERIPH_BASE + 0x00003400)
  SPI2_BASE* = (APB1PERIPH_BASE + 0x00003800)
  SPI3_BASE* = (APB1PERIPH_BASE + 0x00003C00)
  I2S3ext_BASE* = (APB1PERIPH_BASE + 0x00004000)
  USART2_BASE* = (APB1PERIPH_BASE + 0x00004400)
  USART3_BASE* = (APB1PERIPH_BASE + 0x00004800)
  UART4_BASE* = (APB1PERIPH_BASE + 0x00004C00)
  UART5_BASE* = (APB1PERIPH_BASE + 0x00005000)
  I2C1_BASE* = (APB1PERIPH_BASE + 0x00005400)
  I2C2_BASE* = (APB1PERIPH_BASE + 0x00005800)
  USB_BASE* = (APB1PERIPH_BASE + 0x00005C00) ## !< USB_IP Peripheral Registers base address
  USB_PMAADDR* = (APB1PERIPH_BASE + 0x00006000) ## !< USB_IP Packet Memory Area base address
  CAN_BASE* = (APB1PERIPH_BASE + 0x00006400)
  PWR_BASE* = (APB1PERIPH_BASE + 0x00007000)
  DAC1_BASE* = (APB1PERIPH_BASE + 0x00007400)
  DAC_BASE* = DAC1_BASE

## !< APB2 peripherals

const
  SYSCFG_BASE* = (APB2PERIPH_BASE + 0x00000000)
  COMP1_BASE* = (APB2PERIPH_BASE + 0x0000001C)
  COMP2_BASE* = (APB2PERIPH_BASE + 0x00000020)
  COMP3_BASE* = (APB2PERIPH_BASE + 0x00000024)
  COMP4_BASE* = (APB2PERIPH_BASE + 0x00000028)
  COMP5_BASE* = (APB2PERIPH_BASE + 0x0000002C)
  COMP6_BASE* = (APB2PERIPH_BASE + 0x00000030)
  COMP7_BASE* = (APB2PERIPH_BASE + 0x00000034)
  COMP_BASE* = COMP1_BASE
  OPAMP1_BASE* = (APB2PERIPH_BASE + 0x00000038)
  OPAMP2_BASE* = (APB2PERIPH_BASE + 0x0000003C)
  OPAMP3_BASE* = (APB2PERIPH_BASE + 0x00000040)
  OPAMP4_BASE* = (APB2PERIPH_BASE + 0x00000044)
  OPAMP_BASE* = OPAMP1_BASE
  EXTI_BASE* = (APB2PERIPH_BASE + 0x00000400)
  TIM1_BASE* = (APB2PERIPH_BASE + 0x00002C00)
  SPI1_BASE* = (APB2PERIPH_BASE + 0x00003000)
  TIM8_BASE* = (APB2PERIPH_BASE + 0x00003400)
  USART1_BASE* = (APB2PERIPH_BASE + 0x00003800)
  TIM15_BASE* = (APB2PERIPH_BASE + 0x00004000)
  TIM16_BASE* = (APB2PERIPH_BASE + 0x00004400)
  TIM17_BASE* = (APB2PERIPH_BASE + 0x00004800)

## !< AHB1 peripherals

const
  DMA1_BASE* = (AHB1PERIPH_BASE + 0x00000000)
  DMA1_Channel1_BASE* = (AHB1PERIPH_BASE + 0x00000008)
  DMA1_Channel2_BASE* = (AHB1PERIPH_BASE + 0x0000001C)
  DMA1_Channel3_BASE* = (AHB1PERIPH_BASE + 0x00000030)
  DMA1_Channel4_BASE* = (AHB1PERIPH_BASE + 0x00000044)
  DMA1_Channel5_BASE* = (AHB1PERIPH_BASE + 0x00000058)
  DMA1_Channel6_BASE* = (AHB1PERIPH_BASE + 0x0000006C)
  DMA1_Channel7_BASE* = (AHB1PERIPH_BASE + 0x00000080)
  DMA2_BASE* = (AHB1PERIPH_BASE + 0x00000400)
  DMA2_Channel1_BASE* = (AHB1PERIPH_BASE + 0x00000408)
  DMA2_Channel2_BASE* = (AHB1PERIPH_BASE + 0x0000041C)
  DMA2_Channel3_BASE* = (AHB1PERIPH_BASE + 0x00000430)
  DMA2_Channel4_BASE* = (AHB1PERIPH_BASE + 0x00000444)
  DMA2_Channel5_BASE* = (AHB1PERIPH_BASE + 0x00000458)
  RCC_BASE* = (AHB1PERIPH_BASE + 0x00001000)
  FLASH_R_BASE* = (AHB1PERIPH_BASE + 0x00002000) ## !< Flash registers base address
  OB_BASE* = (cast[uint32](0x1FFFF800)) ## !< Flash Option Bytes base address
  FLASHSIZE_BASE* = (cast[uint32](0x1FFFF7CC)) ## !< FLASH Size register base address
  UID_BASE* = (cast[uint32](0x1FFFF7AC)) ## !< Unique device ID register base address
  CRC_BASE* = (AHB1PERIPH_BASE + 0x00003000)
  TSC_BASE* = (AHB1PERIPH_BASE + 0x00004000)

## !< AHB2 peripherals

const
  GPIOA_BASE* = (AHB2PERIPH_BASE + 0x00000000)
  GPIOB_BASE* = (AHB2PERIPH_BASE + 0x00000400)
  GPIOC_BASE* = (AHB2PERIPH_BASE + 0x00000800)
  GPIOD_BASE* = (AHB2PERIPH_BASE + 0x00000C00)
  GPIOE_BASE* = (AHB2PERIPH_BASE + 0x00001000)
  GPIOF_BASE* = (AHB2PERIPH_BASE + 0x00001400)

## !< AHB3 peripherals

const
  ADC1_BASE* = (AHB3PERIPH_BASE + 0x00000000)
  ADC2_BASE* = (AHB3PERIPH_BASE + 0x00000100)
  ADC1_2_COMMON_BASE* = (AHB3PERIPH_BASE + 0x00000300)
  ADC3_BASE* = (AHB3PERIPH_BASE + 0x00000400)
  ADC4_BASE* = (AHB3PERIPH_BASE + 0x00000500)
  ADC3_4_COMMON_BASE* = (AHB3PERIPH_BASE + 0x00000700)
  DBGMCU_BASE* = (cast[uint32](0xE0042000'u32)) ## !< Debug MCU registers base address

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
  TIM6* = (cast[ptr TIM_TypeDef](TIM6_BASE))
  TIM7* = (cast[ptr TIM_TypeDef](TIM7_BASE))
  RTC* = (cast[ptr RTC_TypeDef](RTC_BASE))
  WWDG* = (cast[ptr WWDG_TypeDef](WWDG_BASE))
  IWDG* = (cast[ptr IWDG_TypeDef](IWDG_BASE))
  I2S2ext* = (cast[ptr SPI_TypeDef](I2S2ext_BASE))
  SPI2* = (cast[ptr SPI_TypeDef](SPI2_BASE))
  SPI3* = (cast[ptr SPI_TypeDef](SPI3_BASE))
  I2S3ext* = (cast[ptr SPI_TypeDef](I2S3ext_BASE))
  USART2* = (cast[ptr USART_TypeDef](USART2_BASE))
  USART3* = (cast[ptr USART_TypeDef](USART3_BASE))
  UART4* = (cast[ptr USART_TypeDef](UART4_BASE))
  UART5* = (cast[ptr USART_TypeDef](UART5_BASE))
  I2C1* = (cast[ptr I2C_TypeDef](I2C1_BASE))
  I2C2* = (cast[ptr I2C_TypeDef](I2C2_BASE))
  CAN* = (cast[ptr CAN_TypeDef](CAN_BASE))
  PWR* = (cast[ptr PWR_TypeDef](PWR_BASE))
  DAC* = (cast[ptr DAC_TypeDef](DAC_BASE))
  DAC1* = (cast[ptr DAC_TypeDef](DAC1_BASE))
  COMP1* = (cast[ptr COMP_TypeDef](COMP1_BASE))
  COMP2* = (cast[ptr COMP_TypeDef](COMP2_BASE))
  COMP12_COMMON* = (cast[ptr COMP_Common_TypeDef](COMP2_BASE))
  COMP3* = (cast[ptr COMP_TypeDef](COMP3_BASE))
  COMP4* = (cast[ptr COMP_TypeDef](COMP4_BASE))
  COMP34_COMMON* = (cast[ptr COMP_Common_TypeDef](COMP4_BASE))
  COMP5* = (cast[ptr COMP_TypeDef](COMP5_BASE))
  COMP6* = (cast[ptr COMP_TypeDef](COMP6_BASE))
  COMP56_COMMON* = (cast[ptr COMP_Common_TypeDef](COMP6_BASE))
  COMP7* = (cast[ptr COMP_TypeDef](COMP7_BASE))

##  Legacy define

const
  COMP* = (cast[ptr COMP_TypeDef](COMP_BASE))
  OPAMP1* = (cast[ptr OPAMP_TypeDef](OPAMP1_BASE))
  OPAMP* = (cast[ptr OPAMP_TypeDef](OPAMP_BASE))
  OPAMP2* = (cast[ptr OPAMP_TypeDef](OPAMP2_BASE))
  OPAMP3* = (cast[ptr OPAMP_TypeDef](OPAMP3_BASE))
  OPAMP4* = (cast[ptr OPAMP_TypeDef](OPAMP4_BASE))
  SYSCFG* = (cast[ptr SYSCFG_TypeDef](SYSCFG_BASE))
  EXTI* = (cast[ptr EXTI_TypeDef](EXTI_BASE))
  TIM1* = (cast[ptr TIM_TypeDef](TIM1_BASE))
  SPI1* = (cast[ptr SPI_TypeDef](SPI1_BASE))
  TIM8* = (cast[ptr TIM_TypeDef](TIM8_BASE))
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
  DMA1_Channel6* = (cast[ptr DMA_Channel_TypeDef](DMA1_Channel6_BASE))
  DMA1_Channel7* = (cast[ptr DMA_Channel_TypeDef](DMA1_Channel7_BASE))
  DMA2* = (cast[ptr DMA_TypeDef](DMA2_BASE))
  DMA2_Channel1* = (cast[ptr DMA_Channel_TypeDef](DMA2_Channel1_BASE))
  DMA2_Channel2* = (cast[ptr DMA_Channel_TypeDef](DMA2_Channel2_BASE))
  DMA2_Channel3* = (cast[ptr DMA_Channel_TypeDef](DMA2_Channel3_BASE))
  DMA2_Channel4* = (cast[ptr DMA_Channel_TypeDef](DMA2_Channel4_BASE))
  DMA2_Channel5* = (cast[ptr DMA_Channel_TypeDef](DMA2_Channel5_BASE))
  RCC* = (cast[ptr RCC_TypeDef](RCC_BASE))
  FLASH* = (cast[ptr FLASH_TypeDef](FLASH_R_BASE))
  OB* = (cast[ptr OB_TypeDef](OB_BASE))
  CRC* = (cast[ptr CRC_TypeDef](CRC_BASE))
  TSC* = (cast[ptr TSC_TypeDef](TSC_BASE))
  GPIOA* = (cast[ptr GPIO_TypeDef](GPIOA_BASE))
  GPIOB* = (cast[ptr GPIO_TypeDef](GPIOB_BASE))
  GPIOC* = (cast[ptr GPIO_TypeDef](GPIOC_BASE))
  GPIOD* = (cast[ptr GPIO_TypeDef](GPIOD_BASE))
  GPIOE* = (cast[ptr GPIO_TypeDef](GPIOE_BASE))
  GPIOF* = (cast[ptr GPIO_TypeDef](GPIOF_BASE))
  ADC1* = (cast[ptr ADC_TypeDef](ADC1_BASE))
  ADC2* = (cast[ptr ADC_TypeDef](ADC2_BASE))
  ADC3* = (cast[ptr ADC_TypeDef](ADC3_BASE))
  ADC4* = (cast[ptr ADC_TypeDef](ADC4_BASE))
  ADC12_COMMON* = (cast[ptr ADC_Common_TypeDef](ADC1_2_COMMON_BASE))
  ADC34_COMMON* = (cast[ptr ADC_Common_TypeDef](ADC3_4_COMMON_BASE))

##  Legacy defines

const
  ADC1_2_COMMONx* = ADC12_COMMON
  ADC3_4_COMMONx* = ADC34_COMMON
  USB* = (cast[ptr USB_TypeDef](USB_BASE))

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
##                          Peripheral Registers_Bits_Definition
## ****************************************************************************
## ****************************************************************************
##
##                         Analog to Digital Converter SAR (ADC)
##
## ****************************************************************************

const
  ADC5_V1_1* = true             ## !< ADC IP version

##
##  @brief Specific device feature definitions (not present on all devices in the STM32F3 serie)
##

const
  ADC_MULTIMODE_SUPPORT* = true ## !< ADC feature available only on specific devices: multimode available on devices with several ADC instances

## *******************  Bit definition for ADC_ISR register  *******************

const
  ADC_ISR_ADRDY_Pos* = (0)
  ADC_ISR_ADRDY_Msk* = (0x00000001 shl ADC_ISR_ADRDY_Pos) ## !< 0x00000001
  ADC_ISR_ADRDY* = ADC_ISR_ADRDY_Msk
  ADC_ISR_EOSMP_Pos* = (1)
  ADC_ISR_EOSMP_Msk* = (0x00000001 shl ADC_ISR_EOSMP_Pos) ## !< 0x00000002
  ADC_ISR_EOSMP* = ADC_ISR_EOSMP_Msk
  ADC_ISR_EOC_Pos* = (2)
  ADC_ISR_EOC_Msk* = (0x00000001 shl ADC_ISR_EOC_Pos) ## !< 0x00000004
  ADC_ISR_EOC* = ADC_ISR_EOC_Msk
  ADC_ISR_EOS_Pos* = (3)
  ADC_ISR_EOS_Msk* = (0x00000001 shl ADC_ISR_EOS_Pos) ## !< 0x00000008
  ADC_ISR_EOS* = ADC_ISR_EOS_Msk
  ADC_ISR_OVR_Pos* = (4)
  ADC_ISR_OVR_Msk* = (0x00000001 shl ADC_ISR_OVR_Pos) ## !< 0x00000010
  ADC_ISR_OVR* = ADC_ISR_OVR_Msk
  ADC_ISR_JEOC_Pos* = (5)
  ADC_ISR_JEOC_Msk* = (0x00000001 shl ADC_ISR_JEOC_Pos) ## !< 0x00000020
  ADC_ISR_JEOC* = ADC_ISR_JEOC_Msk
  ADC_ISR_JEOS_Pos* = (6)
  ADC_ISR_JEOS_Msk* = (0x00000001 shl ADC_ISR_JEOS_Pos) ## !< 0x00000040
  ADC_ISR_JEOS* = ADC_ISR_JEOS_Msk
  ADC_ISR_AWD1_Pos* = (7)
  ADC_ISR_AWD1_Msk* = (0x00000001 shl ADC_ISR_AWD1_Pos) ## !< 0x00000080
  ADC_ISR_AWD1* = ADC_ISR_AWD1_Msk
  ADC_ISR_AWD2_Pos* = (8)
  ADC_ISR_AWD2_Msk* = (0x00000001 shl ADC_ISR_AWD2_Pos) ## !< 0x00000100
  ADC_ISR_AWD2* = ADC_ISR_AWD2_Msk
  ADC_ISR_AWD3_Pos* = (9)
  ADC_ISR_AWD3_Msk* = (0x00000001 shl ADC_ISR_AWD3_Pos) ## !< 0x00000200
  ADC_ISR_AWD3* = ADC_ISR_AWD3_Msk
  ADC_ISR_JQOVF_Pos* = (10)
  ADC_ISR_JQOVF_Msk* = (0x00000001 shl ADC_ISR_JQOVF_Pos) ## !< 0x00000400
  ADC_ISR_JQOVF* = ADC_ISR_JQOVF_Msk

##  Legacy defines

const
  ADC_ISR_ADRD* = (ADC_ISR_ADRDY)

## *******************  Bit definition for ADC_IER register  *******************

const
  ADC_IER_ADRDYIE_Pos* = (0)
  ADC_IER_ADRDYIE_Msk* = (0x00000001 shl ADC_IER_ADRDYIE_Pos) ## !< 0x00000001
  ADC_IER_ADRDYIE* = ADC_IER_ADRDYIE_Msk
  ADC_IER_EOSMPIE_Pos* = (1)
  ADC_IER_EOSMPIE_Msk* = (0x00000001 shl ADC_IER_EOSMPIE_Pos) ## !< 0x00000002
  ADC_IER_EOSMPIE* = ADC_IER_EOSMPIE_Msk
  ADC_IER_EOCIE_Pos* = (2)
  ADC_IER_EOCIE_Msk* = (0x00000001 shl ADC_IER_EOCIE_Pos) ## !< 0x00000004
  ADC_IER_EOCIE* = ADC_IER_EOCIE_Msk
  ADC_IER_EOSIE_Pos* = (3)
  ADC_IER_EOSIE_Msk* = (0x00000001 shl ADC_IER_EOSIE_Pos) ## !< 0x00000008
  ADC_IER_EOSIE* = ADC_IER_EOSIE_Msk
  ADC_IER_OVRIE_Pos* = (4)
  ADC_IER_OVRIE_Msk* = (0x00000001 shl ADC_IER_OVRIE_Pos) ## !< 0x00000010
  ADC_IER_OVRIE* = ADC_IER_OVRIE_Msk
  ADC_IER_JEOCIE_Pos* = (5)
  ADC_IER_JEOCIE_Msk* = (0x00000001 shl ADC_IER_JEOCIE_Pos) ## !< 0x00000020
  ADC_IER_JEOCIE* = ADC_IER_JEOCIE_Msk
  ADC_IER_JEOSIE_Pos* = (6)
  ADC_IER_JEOSIE_Msk* = (0x00000001 shl ADC_IER_JEOSIE_Pos) ## !< 0x00000040
  ADC_IER_JEOSIE* = ADC_IER_JEOSIE_Msk
  ADC_IER_AWD1IE_Pos* = (7)
  ADC_IER_AWD1IE_Msk* = (0x00000001 shl ADC_IER_AWD1IE_Pos) ## !< 0x00000080
  ADC_IER_AWD1IE* = ADC_IER_AWD1IE_Msk
  ADC_IER_AWD2IE_Pos* = (8)
  ADC_IER_AWD2IE_Msk* = (0x00000001 shl ADC_IER_AWD2IE_Pos) ## !< 0x00000100
  ADC_IER_AWD2IE* = ADC_IER_AWD2IE_Msk
  ADC_IER_AWD3IE_Pos* = (9)
  ADC_IER_AWD3IE_Msk* = (0x00000001 shl ADC_IER_AWD3IE_Pos) ## !< 0x00000200
  ADC_IER_AWD3IE* = ADC_IER_AWD3IE_Msk
  ADC_IER_JQOVFIE_Pos* = (10)
  ADC_IER_JQOVFIE_Msk* = (0x00000001 shl ADC_IER_JQOVFIE_Pos) ## !< 0x00000400
  ADC_IER_JQOVFIE* = ADC_IER_JQOVFIE_Msk

##  Legacy defines

const
  ADC_IER_RDY* = (ADC_IER_ADRDYIE)
  ADC_IER_EOSMP* = (ADC_IER_EOSMPIE)
  ADC_IER_EOC* = (ADC_IER_EOCIE)
  ADC_IER_EOS* = (ADC_IER_EOSIE)
  ADC_IER_OVR* = (ADC_IER_OVRIE)
  ADC_IER_JEOC* = (ADC_IER_JEOCIE)
  ADC_IER_JEOS* = (ADC_IER_JEOSIE)
  ADC_IER_AWD1* = (ADC_IER_AWD1IE)
  ADC_IER_AWD2* = (ADC_IER_AWD2IE)
  ADC_IER_AWD3* = (ADC_IER_AWD3IE)
  ADC_IER_JQOVF* = (ADC_IER_JQOVFIE)

## *******************  Bit definition for ADC_CR register  *******************

const
  ADC_CR_ADEN_Pos* = (0)
  ADC_CR_ADEN_Msk* = (0x00000001 shl ADC_CR_ADEN_Pos) ## !< 0x00000001
  ADC_CR_ADEN* = ADC_CR_ADEN_Msk
  ADC_CR_ADDIS_Pos* = (1)
  ADC_CR_ADDIS_Msk* = (0x00000001 shl ADC_CR_ADDIS_Pos) ## !< 0x00000002
  ADC_CR_ADDIS* = ADC_CR_ADDIS_Msk
  ADC_CR_ADSTART_Pos* = (2)
  ADC_CR_ADSTART_Msk* = (0x00000001 shl ADC_CR_ADSTART_Pos) ## !< 0x00000004
  ADC_CR_ADSTART* = ADC_CR_ADSTART_Msk
  ADC_CR_JADSTART_Pos* = (3)
  ADC_CR_JADSTART_Msk* = (0x00000001 shl ADC_CR_JADSTART_Pos) ## !< 0x00000008
  ADC_CR_JADSTART* = ADC_CR_JADSTART_Msk
  ADC_CR_ADSTP_Pos* = (4)
  ADC_CR_ADSTP_Msk* = (0x00000001 shl ADC_CR_ADSTP_Pos) ## !< 0x00000010
  ADC_CR_ADSTP* = ADC_CR_ADSTP_Msk
  ADC_CR_JADSTP_Pos* = (5)
  ADC_CR_JADSTP_Msk* = (0x00000001 shl ADC_CR_JADSTP_Pos) ## !< 0x00000020
  ADC_CR_JADSTP* = ADC_CR_JADSTP_Msk
  ADC_CR_ADVREGEN_Pos* = (28)
  ADC_CR_ADVREGEN_Msk* = (0x00000003 shl ADC_CR_ADVREGEN_Pos) ## !< 0x30000000
  ADC_CR_ADVREGEN* = ADC_CR_ADVREGEN_Msk
  ADC_CR_ADVREGEN_0* = (0x00000001 shl ADC_CR_ADVREGEN_Pos) ## !< 0x10000000
  ADC_CR_ADVREGEN_1* = (0x00000002 shl ADC_CR_ADVREGEN_Pos) ## !< 0x20000000
  ADC_CR_ADCALDIF_Pos* = (30)
  ADC_CR_ADCALDIF_Msk* = (0x00000001 shl ADC_CR_ADCALDIF_Pos) ## !< 0x40000000
  ADC_CR_ADCALDIF* = ADC_CR_ADCALDIF_Msk
  ADC_CR_ADCAL_Pos* = (31)
  ADC_CR_ADCAL_Msk* = (0x00000001 shl ADC_CR_ADCAL_Pos) ## !< 0x80000000
  ADC_CR_ADCAL* = ADC_CR_ADCAL_Msk

## *******************  Bit definition for ADC_CFGR register  *****************

const
  ADC_CFGR_DMAEN_Pos* = (0)
  ADC_CFGR_DMAEN_Msk* = (0x00000001 shl ADC_CFGR_DMAEN_Pos) ## !< 0x00000001
  ADC_CFGR_DMAEN* = ADC_CFGR_DMAEN_Msk
  ADC_CFGR_DMACFG_Pos* = (1)
  ADC_CFGR_DMACFG_Msk* = (0x00000001 shl ADC_CFGR_DMACFG_Pos) ## !< 0x00000002
  ADC_CFGR_DMACFG* = ADC_CFGR_DMACFG_Msk
  ADC_CFGR_RES_Pos* = (3)
  ADC_CFGR_RES_Msk* = (0x00000003 shl ADC_CFGR_RES_Pos) ## !< 0x00000018
  ADC_CFGR_RES* = ADC_CFGR_RES_Msk
  ADC_CFGR_RES_0* = (0x00000001 shl ADC_CFGR_RES_Pos) ## !< 0x00000008
  ADC_CFGR_RES_1* = (0x00000002 shl ADC_CFGR_RES_Pos) ## !< 0x00000010
  ADC_CFGR_ALIGN_Pos* = (5)
  ADC_CFGR_ALIGN_Msk* = (0x00000001 shl ADC_CFGR_ALIGN_Pos) ## !< 0x00000020
  ADC_CFGR_ALIGN* = ADC_CFGR_ALIGN_Msk
  ADC_CFGR_EXTSEL_Pos* = (6)
  ADC_CFGR_EXTSEL_Msk* = (0x0000000F shl ADC_CFGR_EXTSEL_Pos) ## !< 0x000003C0
  ADC_CFGR_EXTSEL* = ADC_CFGR_EXTSEL_Msk
  ADC_CFGR_EXTSEL_0* = (0x00000001 shl ADC_CFGR_EXTSEL_Pos) ## !< 0x00000040
  ADC_CFGR_EXTSEL_1* = (0x00000002 shl ADC_CFGR_EXTSEL_Pos) ## !< 0x00000080
  ADC_CFGR_EXTSEL_2* = (0x00000004 shl ADC_CFGR_EXTSEL_Pos) ## !< 0x00000100
  ADC_CFGR_EXTSEL_3* = (0x00000008 shl ADC_CFGR_EXTSEL_Pos) ## !< 0x00000200
  ADC_CFGR_EXTEN_Pos* = (10)
  ADC_CFGR_EXTEN_Msk* = (0x00000003 shl ADC_CFGR_EXTEN_Pos) ## !< 0x00000C00
  ADC_CFGR_EXTEN* = ADC_CFGR_EXTEN_Msk
  ADC_CFGR_EXTEN_0* = (0x00000001 shl ADC_CFGR_EXTEN_Pos) ## !< 0x00000400
  ADC_CFGR_EXTEN_1* = (0x00000002 shl ADC_CFGR_EXTEN_Pos) ## !< 0x00000800
  ADC_CFGR_OVRMOD_Pos* = (12)
  ADC_CFGR_OVRMOD_Msk* = (0x00000001 shl ADC_CFGR_OVRMOD_Pos) ## !< 0x00001000
  ADC_CFGR_OVRMOD* = ADC_CFGR_OVRMOD_Msk
  ADC_CFGR_CONT_Pos* = (13)
  ADC_CFGR_CONT_Msk* = (0x00000001 shl ADC_CFGR_CONT_Pos) ## !< 0x00002000
  ADC_CFGR_CONT* = ADC_CFGR_CONT_Msk
  ADC_CFGR_AUTDLY_Pos* = (14)
  ADC_CFGR_AUTDLY_Msk* = (0x00000001 shl ADC_CFGR_AUTDLY_Pos) ## !< 0x00004000
  ADC_CFGR_AUTDLY* = ADC_CFGR_AUTDLY_Msk
  ADC_CFGR_DISCEN_Pos* = (16)
  ADC_CFGR_DISCEN_Msk* = (0x00000001 shl ADC_CFGR_DISCEN_Pos) ## !< 0x00010000
  ADC_CFGR_DISCEN* = ADC_CFGR_DISCEN_Msk
  ADC_CFGR_DISCNUM_Pos* = (17)
  ADC_CFGR_DISCNUM_Msk* = (0x00000007 shl ADC_CFGR_DISCNUM_Pos) ## !< 0x000E0000
  ADC_CFGR_DISCNUM* = ADC_CFGR_DISCNUM_Msk
  ADC_CFGR_DISCNUM_0* = (0x00000001 shl ADC_CFGR_DISCNUM_Pos) ## !< 0x00020000
  ADC_CFGR_DISCNUM_1* = (0x00000002 shl ADC_CFGR_DISCNUM_Pos) ## !< 0x00040000
  ADC_CFGR_DISCNUM_2* = (0x00000004 shl ADC_CFGR_DISCNUM_Pos) ## !< 0x00080000
  ADC_CFGR_JDISCEN_Pos* = (20)
  ADC_CFGR_JDISCEN_Msk* = (0x00000001 shl ADC_CFGR_JDISCEN_Pos) ## !< 0x00100000
  ADC_CFGR_JDISCEN* = ADC_CFGR_JDISCEN_Msk
  ADC_CFGR_JQM_Pos* = (21)
  ADC_CFGR_JQM_Msk* = (0x00000001 shl ADC_CFGR_JQM_Pos) ## !< 0x00200000
  ADC_CFGR_JQM* = ADC_CFGR_JQM_Msk
  ADC_CFGR_AWD1SGL_Pos* = (22)
  ADC_CFGR_AWD1SGL_Msk* = (0x00000001 shl ADC_CFGR_AWD1SGL_Pos) ## !< 0x00400000
  ADC_CFGR_AWD1SGL* = ADC_CFGR_AWD1SGL_Msk
  ADC_CFGR_AWD1EN_Pos* = (23)
  ADC_CFGR_AWD1EN_Msk* = (0x00000001 shl ADC_CFGR_AWD1EN_Pos) ## !< 0x00800000
  ADC_CFGR_AWD1EN* = ADC_CFGR_AWD1EN_Msk
  ADC_CFGR_JAWD1EN_Pos* = (24)
  ADC_CFGR_JAWD1EN_Msk* = (0x00000001 shl ADC_CFGR_JAWD1EN_Pos) ## !< 0x01000000
  ADC_CFGR_JAWD1EN* = ADC_CFGR_JAWD1EN_Msk
  ADC_CFGR_JAUTO_Pos* = (25)
  ADC_CFGR_JAUTO_Msk* = (0x00000001 shl ADC_CFGR_JAUTO_Pos) ## !< 0x02000000
  ADC_CFGR_JAUTO* = ADC_CFGR_JAUTO_Msk
  ADC_CFGR_AWD1CH_Pos* = (26)
  ADC_CFGR_AWD1CH_Msk* = (0x0000001F shl ADC_CFGR_AWD1CH_Pos) ## !< 0x7C000000
  ADC_CFGR_AWD1CH* = ADC_CFGR_AWD1CH_Msk
  ADC_CFGR_AWD1CH_0* = (0x00000001 shl ADC_CFGR_AWD1CH_Pos) ## !< 0x04000000
  ADC_CFGR_AWD1CH_1* = (0x00000002 shl ADC_CFGR_AWD1CH_Pos) ## !< 0x08000000
  ADC_CFGR_AWD1CH_2* = (0x00000004 shl ADC_CFGR_AWD1CH_Pos) ## !< 0x10000000
  ADC_CFGR_AWD1CH_3* = (0x00000008 shl ADC_CFGR_AWD1CH_Pos) ## !< 0x20000000
  ADC_CFGR_AWD1CH_4* = (0x00000010 shl ADC_CFGR_AWD1CH_Pos) ## !< 0x40000000

##  Legacy defines

const
  ADC_CFGR_AUTOFF_Pos* = (15)
  ADC_CFGR_AUTOFF_Msk* = (0x00000001 shl ADC_CFGR_AUTOFF_Pos) ## !< 0x00008000
  ADC_CFGR_AUTOFF* = ADC_CFGR_AUTOFF_Msk

## *******************  Bit definition for ADC_SMPR1 register  ****************

const
  ADC_SMPR1_SMP0_Pos* = (0)
  ADC_SMPR1_SMP0_Msk* = (0x00000007 shl ADC_SMPR1_SMP0_Pos) ## !< 0x00000007
  ADC_SMPR1_SMP0* = ADC_SMPR1_SMP0_Msk
  ADC_SMPR1_SMP0_0* = (0x00000001 shl ADC_SMPR1_SMP0_Pos) ## !< 0x00000001
  ADC_SMPR1_SMP0_1* = (0x00000002 shl ADC_SMPR1_SMP0_Pos) ## !< 0x00000002
  ADC_SMPR1_SMP0_2* = (0x00000004 shl ADC_SMPR1_SMP0_Pos) ## !< 0x00000004
  ADC_SMPR1_SMP1_Pos* = (3)
  ADC_SMPR1_SMP1_Msk* = (0x00000007 shl ADC_SMPR1_SMP1_Pos) ## !< 0x00000038
  ADC_SMPR1_SMP1* = ADC_SMPR1_SMP1_Msk
  ADC_SMPR1_SMP1_0* = (0x00000001 shl ADC_SMPR1_SMP1_Pos) ## !< 0x00000008
  ADC_SMPR1_SMP1_1* = (0x00000002 shl ADC_SMPR1_SMP1_Pos) ## !< 0x00000010
  ADC_SMPR1_SMP1_2* = (0x00000004 shl ADC_SMPR1_SMP1_Pos) ## !< 0x00000020
  ADC_SMPR1_SMP2_Pos* = (6)
  ADC_SMPR1_SMP2_Msk* = (0x00000007 shl ADC_SMPR1_SMP2_Pos) ## !< 0x000001C0
  ADC_SMPR1_SMP2* = ADC_SMPR1_SMP2_Msk
  ADC_SMPR1_SMP2_0* = (0x00000001 shl ADC_SMPR1_SMP2_Pos) ## !< 0x00000040
  ADC_SMPR1_SMP2_1* = (0x00000002 shl ADC_SMPR1_SMP2_Pos) ## !< 0x00000080
  ADC_SMPR1_SMP2_2* = (0x00000004 shl ADC_SMPR1_SMP2_Pos) ## !< 0x00000100
  ADC_SMPR1_SMP3_Pos* = (9)
  ADC_SMPR1_SMP3_Msk* = (0x00000007 shl ADC_SMPR1_SMP3_Pos) ## !< 0x00000E00
  ADC_SMPR1_SMP3* = ADC_SMPR1_SMP3_Msk
  ADC_SMPR1_SMP3_0* = (0x00000001 shl ADC_SMPR1_SMP3_Pos) ## !< 0x00000200
  ADC_SMPR1_SMP3_1* = (0x00000002 shl ADC_SMPR1_SMP3_Pos) ## !< 0x00000400
  ADC_SMPR1_SMP3_2* = (0x00000004 shl ADC_SMPR1_SMP3_Pos) ## !< 0x00000800
  ADC_SMPR1_SMP4_Pos* = (12)
  ADC_SMPR1_SMP4_Msk* = (0x00000007 shl ADC_SMPR1_SMP4_Pos) ## !< 0x00007000
  ADC_SMPR1_SMP4* = ADC_SMPR1_SMP4_Msk
  ADC_SMPR1_SMP4_0* = (0x00000001 shl ADC_SMPR1_SMP4_Pos) ## !< 0x00001000
  ADC_SMPR1_SMP4_1* = (0x00000002 shl ADC_SMPR1_SMP4_Pos) ## !< 0x00002000
  ADC_SMPR1_SMP4_2* = (0x00000004 shl ADC_SMPR1_SMP4_Pos) ## !< 0x00004000
  ADC_SMPR1_SMP5_Pos* = (15)
  ADC_SMPR1_SMP5_Msk* = (0x00000007 shl ADC_SMPR1_SMP5_Pos) ## !< 0x00038000
  ADC_SMPR1_SMP5* = ADC_SMPR1_SMP5_Msk
  ADC_SMPR1_SMP5_0* = (0x00000001 shl ADC_SMPR1_SMP5_Pos) ## !< 0x00008000
  ADC_SMPR1_SMP5_1* = (0x00000002 shl ADC_SMPR1_SMP5_Pos) ## !< 0x00010000
  ADC_SMPR1_SMP5_2* = (0x00000004 shl ADC_SMPR1_SMP5_Pos) ## !< 0x00020000
  ADC_SMPR1_SMP6_Pos* = (18)
  ADC_SMPR1_SMP6_Msk* = (0x00000007 shl ADC_SMPR1_SMP6_Pos) ## !< 0x001C0000
  ADC_SMPR1_SMP6* = ADC_SMPR1_SMP6_Msk
  ADC_SMPR1_SMP6_0* = (0x00000001 shl ADC_SMPR1_SMP6_Pos) ## !< 0x00040000
  ADC_SMPR1_SMP6_1* = (0x00000002 shl ADC_SMPR1_SMP6_Pos) ## !< 0x00080000
  ADC_SMPR1_SMP6_2* = (0x00000004 shl ADC_SMPR1_SMP6_Pos) ## !< 0x00100000
  ADC_SMPR1_SMP7_Pos* = (21)
  ADC_SMPR1_SMP7_Msk* = (0x00000007 shl ADC_SMPR1_SMP7_Pos) ## !< 0x00E00000
  ADC_SMPR1_SMP7* = ADC_SMPR1_SMP7_Msk
  ADC_SMPR1_SMP7_0* = (0x00000001 shl ADC_SMPR1_SMP7_Pos) ## !< 0x00200000
  ADC_SMPR1_SMP7_1* = (0x00000002 shl ADC_SMPR1_SMP7_Pos) ## !< 0x00400000
  ADC_SMPR1_SMP7_2* = (0x00000004 shl ADC_SMPR1_SMP7_Pos) ## !< 0x00800000
  ADC_SMPR1_SMP8_Pos* = (24)
  ADC_SMPR1_SMP8_Msk* = (0x00000007 shl ADC_SMPR1_SMP8_Pos) ## !< 0x07000000
  ADC_SMPR1_SMP8* = ADC_SMPR1_SMP8_Msk
  ADC_SMPR1_SMP8_0* = (0x00000001 shl ADC_SMPR1_SMP8_Pos) ## !< 0x01000000
  ADC_SMPR1_SMP8_1* = (0x00000002 shl ADC_SMPR1_SMP8_Pos) ## !< 0x02000000
  ADC_SMPR1_SMP8_2* = (0x00000004 shl ADC_SMPR1_SMP8_Pos) ## !< 0x04000000
  ADC_SMPR1_SMP9_Pos* = (27)
  ADC_SMPR1_SMP9_Msk* = (0x00000007 shl ADC_SMPR1_SMP9_Pos) ## !< 0x38000000
  ADC_SMPR1_SMP9* = ADC_SMPR1_SMP9_Msk
  ADC_SMPR1_SMP9_0* = (0x00000001 shl ADC_SMPR1_SMP9_Pos) ## !< 0x08000000
  ADC_SMPR1_SMP9_1* = (0x00000002 shl ADC_SMPR1_SMP9_Pos) ## !< 0x10000000
  ADC_SMPR1_SMP9_2* = (0x00000004 shl ADC_SMPR1_SMP9_Pos) ## !< 0x20000000

## *******************  Bit definition for ADC_SMPR2 register  ****************

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

## *******************  Bit definition for ADC_TR1 register  ******************

const
  ADC_TR1_LT1_Pos* = (0)
  ADC_TR1_LT1_Msk* = (0x00000FFF shl ADC_TR1_LT1_Pos) ## !< 0x00000FFF
  ADC_TR1_LT1* = ADC_TR1_LT1_Msk
  ADC_TR1_LT1_0* = (0x00000001 shl ADC_TR1_LT1_Pos) ## !< 0x00000001
  ADC_TR1_LT1_1* = (0x00000002 shl ADC_TR1_LT1_Pos) ## !< 0x00000002
  ADC_TR1_LT1_2* = (0x00000004 shl ADC_TR1_LT1_Pos) ## !< 0x00000004
  ADC_TR1_LT1_3* = (0x00000008 shl ADC_TR1_LT1_Pos) ## !< 0x00000008
  ADC_TR1_LT1_4* = (0x00000010 shl ADC_TR1_LT1_Pos) ## !< 0x00000010
  ADC_TR1_LT1_5* = (0x00000020 shl ADC_TR1_LT1_Pos) ## !< 0x00000020
  ADC_TR1_LT1_6* = (0x00000040 shl ADC_TR1_LT1_Pos) ## !< 0x00000040
  ADC_TR1_LT1_7* = (0x00000080 shl ADC_TR1_LT1_Pos) ## !< 0x00000080
  ADC_TR1_LT1_8* = (0x00000100 shl ADC_TR1_LT1_Pos) ## !< 0x00000100
  ADC_TR1_LT1_9* = (0x00000200 shl ADC_TR1_LT1_Pos) ## !< 0x00000200
  ADC_TR1_LT1_10* = (0x00000400 shl ADC_TR1_LT1_Pos) ## !< 0x00000400
  ADC_TR1_LT1_11* = (0x00000800 shl ADC_TR1_LT1_Pos) ## !< 0x00000800
  ADC_TR1_HT1_Pos* = (16)
  ADC_TR1_HT1_Msk* = (0x00000FFF shl ADC_TR1_HT1_Pos) ## !< 0x0FFF0000
  ADC_TR1_HT1* = ADC_TR1_HT1_Msk
  ADC_TR1_HT1_0* = (0x00000001 shl ADC_TR1_HT1_Pos) ## !< 0x00010000
  ADC_TR1_HT1_1* = (0x00000002 shl ADC_TR1_HT1_Pos) ## !< 0x00020000
  ADC_TR1_HT1_2* = (0x00000004 shl ADC_TR1_HT1_Pos) ## !< 0x00040000
  ADC_TR1_HT1_3* = (0x00000008 shl ADC_TR1_HT1_Pos) ## !< 0x00080000
  ADC_TR1_HT1_4* = (0x00000010 shl ADC_TR1_HT1_Pos) ## !< 0x00100000
  ADC_TR1_HT1_5* = (0x00000020 shl ADC_TR1_HT1_Pos) ## !< 0x00200000
  ADC_TR1_HT1_6* = (0x00000040 shl ADC_TR1_HT1_Pos) ## !< 0x00400000
  ADC_TR1_HT1_7* = (0x00000080 shl ADC_TR1_HT1_Pos) ## !< 0x00800000
  ADC_TR1_HT1_8* = (0x00000100 shl ADC_TR1_HT1_Pos) ## !< 0x01000000
  ADC_TR1_HT1_9* = (0x00000200 shl ADC_TR1_HT1_Pos) ## !< 0x02000000
  ADC_TR1_HT1_10* = (0x00000400 shl ADC_TR1_HT1_Pos) ## !< 0x04000000
  ADC_TR1_HT1_11* = (0x00000800 shl ADC_TR1_HT1_Pos) ## !< 0x08000000

## *******************  Bit definition for ADC_TR2 register  ******************

const
  ADC_TR2_LT2_Pos* = (0)
  ADC_TR2_LT2_Msk* = (0x000000FF shl ADC_TR2_LT2_Pos) ## !< 0x000000FF
  ADC_TR2_LT2* = ADC_TR2_LT2_Msk
  ADC_TR2_LT2_0* = (0x00000001 shl ADC_TR2_LT2_Pos) ## !< 0x00000001
  ADC_TR2_LT2_1* = (0x00000002 shl ADC_TR2_LT2_Pos) ## !< 0x00000002
  ADC_TR2_LT2_2* = (0x00000004 shl ADC_TR2_LT2_Pos) ## !< 0x00000004
  ADC_TR2_LT2_3* = (0x00000008 shl ADC_TR2_LT2_Pos) ## !< 0x00000008
  ADC_TR2_LT2_4* = (0x00000010 shl ADC_TR2_LT2_Pos) ## !< 0x00000010
  ADC_TR2_LT2_5* = (0x00000020 shl ADC_TR2_LT2_Pos) ## !< 0x00000020
  ADC_TR2_LT2_6* = (0x00000040 shl ADC_TR2_LT2_Pos) ## !< 0x00000040
  ADC_TR2_LT2_7* = (0x00000080 shl ADC_TR2_LT2_Pos) ## !< 0x00000080
  ADC_TR2_HT2_Pos* = (16)
  ADC_TR2_HT2_Msk* = (0x000000FF shl ADC_TR2_HT2_Pos) ## !< 0x00FF0000
  ADC_TR2_HT2* = ADC_TR2_HT2_Msk
  ADC_TR2_HT2_0* = (0x00000001 shl ADC_TR2_HT2_Pos) ## !< 0x00010000
  ADC_TR2_HT2_1* = (0x00000002 shl ADC_TR2_HT2_Pos) ## !< 0x00020000
  ADC_TR2_HT2_2* = (0x00000004 shl ADC_TR2_HT2_Pos) ## !< 0x00040000
  ADC_TR2_HT2_3* = (0x00000008 shl ADC_TR2_HT2_Pos) ## !< 0x00080000
  ADC_TR2_HT2_4* = (0x00000010 shl ADC_TR2_HT2_Pos) ## !< 0x00100000
  ADC_TR2_HT2_5* = (0x00000020 shl ADC_TR2_HT2_Pos) ## !< 0x00200000
  ADC_TR2_HT2_6* = (0x00000040 shl ADC_TR2_HT2_Pos) ## !< 0x00400000
  ADC_TR2_HT2_7* = (0x00000080 shl ADC_TR2_HT2_Pos) ## !< 0x00800000

## *******************  Bit definition for ADC_TR3 register  ******************

const
  ADC_TR3_LT3_Pos* = (0)
  ADC_TR3_LT3_Msk* = (0x000000FF shl ADC_TR3_LT3_Pos) ## !< 0x000000FF
  ADC_TR3_LT3* = ADC_TR3_LT3_Msk
  ADC_TR3_LT3_0* = (0x00000001 shl ADC_TR3_LT3_Pos) ## !< 0x00000001
  ADC_TR3_LT3_1* = (0x00000002 shl ADC_TR3_LT3_Pos) ## !< 0x00000002
  ADC_TR3_LT3_2* = (0x00000004 shl ADC_TR3_LT3_Pos) ## !< 0x00000004
  ADC_TR3_LT3_3* = (0x00000008 shl ADC_TR3_LT3_Pos) ## !< 0x00000008
  ADC_TR3_LT3_4* = (0x00000010 shl ADC_TR3_LT3_Pos) ## !< 0x00000010
  ADC_TR3_LT3_5* = (0x00000020 shl ADC_TR3_LT3_Pos) ## !< 0x00000020
  ADC_TR3_LT3_6* = (0x00000040 shl ADC_TR3_LT3_Pos) ## !< 0x00000040
  ADC_TR3_LT3_7* = (0x00000080 shl ADC_TR3_LT3_Pos) ## !< 0x00000080
  ADC_TR3_HT3_Pos* = (16)
  ADC_TR3_HT3_Msk* = (0x000000FF shl ADC_TR3_HT3_Pos) ## !< 0x00FF0000
  ADC_TR3_HT3* = ADC_TR3_HT3_Msk
  ADC_TR3_HT3_0* = (0x00000001 shl ADC_TR3_HT3_Pos) ## !< 0x00010000
  ADC_TR3_HT3_1* = (0x00000002 shl ADC_TR3_HT3_Pos) ## !< 0x00020000
  ADC_TR3_HT3_2* = (0x00000004 shl ADC_TR3_HT3_Pos) ## !< 0x00040000
  ADC_TR3_HT3_3* = (0x00000008 shl ADC_TR3_HT3_Pos) ## !< 0x00080000
  ADC_TR3_HT3_4* = (0x00000010 shl ADC_TR3_HT3_Pos) ## !< 0x00100000
  ADC_TR3_HT3_5* = (0x00000020 shl ADC_TR3_HT3_Pos) ## !< 0x00200000
  ADC_TR3_HT3_6* = (0x00000040 shl ADC_TR3_HT3_Pos) ## !< 0x00400000
  ADC_TR3_HT3_7* = (0x00000080 shl ADC_TR3_HT3_Pos) ## !< 0x00800000

## *******************  Bit definition for ADC_SQR1 register  *****************

const
  ADC_SQR1_L_Pos* = (0)
  ADC_SQR1_L_Msk* = (0x0000000F shl ADC_SQR1_L_Pos) ## !< 0x0000000F
  ADC_SQR1_L* = ADC_SQR1_L_Msk
  ADC_SQR1_L_0* = (0x00000001 shl ADC_SQR1_L_Pos) ## !< 0x00000001
  ADC_SQR1_L_1* = (0x00000002 shl ADC_SQR1_L_Pos) ## !< 0x00000002
  ADC_SQR1_L_2* = (0x00000004 shl ADC_SQR1_L_Pos) ## !< 0x00000004
  ADC_SQR1_L_3* = (0x00000008 shl ADC_SQR1_L_Pos) ## !< 0x00000008
  ADC_SQR1_SQ1_Pos* = (6)
  ADC_SQR1_SQ1_Msk* = (0x0000001F shl ADC_SQR1_SQ1_Pos) ## !< 0x000007C0
  ADC_SQR1_SQ1* = ADC_SQR1_SQ1_Msk
  ADC_SQR1_SQ1_0* = (0x00000001 shl ADC_SQR1_SQ1_Pos) ## !< 0x00000040
  ADC_SQR1_SQ1_1* = (0x00000002 shl ADC_SQR1_SQ1_Pos) ## !< 0x00000080
  ADC_SQR1_SQ1_2* = (0x00000004 shl ADC_SQR1_SQ1_Pos) ## !< 0x00000100
  ADC_SQR1_SQ1_3* = (0x00000008 shl ADC_SQR1_SQ1_Pos) ## !< 0x00000200
  ADC_SQR1_SQ1_4* = (0x00000010 shl ADC_SQR1_SQ1_Pos) ## !< 0x00000400
  ADC_SQR1_SQ2_Pos* = (12)
  ADC_SQR1_SQ2_Msk* = (0x0000001F shl ADC_SQR1_SQ2_Pos) ## !< 0x0001F000
  ADC_SQR1_SQ2* = ADC_SQR1_SQ2_Msk
  ADC_SQR1_SQ2_0* = (0x00000001 shl ADC_SQR1_SQ2_Pos) ## !< 0x00001000
  ADC_SQR1_SQ2_1* = (0x00000002 shl ADC_SQR1_SQ2_Pos) ## !< 0x00002000
  ADC_SQR1_SQ2_2* = (0x00000004 shl ADC_SQR1_SQ2_Pos) ## !< 0x00004000
  ADC_SQR1_SQ2_3* = (0x00000008 shl ADC_SQR1_SQ2_Pos) ## !< 0x00008000
  ADC_SQR1_SQ2_4* = (0x00000010 shl ADC_SQR1_SQ2_Pos) ## !< 0x00010000
  ADC_SQR1_SQ3_Pos* = (18)
  ADC_SQR1_SQ3_Msk* = (0x0000001F shl ADC_SQR1_SQ3_Pos) ## !< 0x007C0000
  ADC_SQR1_SQ3* = ADC_SQR1_SQ3_Msk
  ADC_SQR1_SQ3_0* = (0x00000001 shl ADC_SQR1_SQ3_Pos) ## !< 0x00040000
  ADC_SQR1_SQ3_1* = (0x00000002 shl ADC_SQR1_SQ3_Pos) ## !< 0x00080000
  ADC_SQR1_SQ3_2* = (0x00000004 shl ADC_SQR1_SQ3_Pos) ## !< 0x00100000
  ADC_SQR1_SQ3_3* = (0x00000008 shl ADC_SQR1_SQ3_Pos) ## !< 0x00200000
  ADC_SQR1_SQ3_4* = (0x00000010 shl ADC_SQR1_SQ3_Pos) ## !< 0x00400000
  ADC_SQR1_SQ4_Pos* = (24)
  ADC_SQR1_SQ4_Msk* = (0x0000001F shl ADC_SQR1_SQ4_Pos) ## !< 0x1F000000
  ADC_SQR1_SQ4* = ADC_SQR1_SQ4_Msk
  ADC_SQR1_SQ4_0* = (0x00000001 shl ADC_SQR1_SQ4_Pos) ## !< 0x01000000
  ADC_SQR1_SQ4_1* = (0x00000002 shl ADC_SQR1_SQ4_Pos) ## !< 0x02000000
  ADC_SQR1_SQ4_2* = (0x00000004 shl ADC_SQR1_SQ4_Pos) ## !< 0x04000000
  ADC_SQR1_SQ4_3* = (0x00000008 shl ADC_SQR1_SQ4_Pos) ## !< 0x08000000
  ADC_SQR1_SQ4_4* = (0x00000010 shl ADC_SQR1_SQ4_Pos) ## !< 0x10000000

## *******************  Bit definition for ADC_SQR2 register  *****************

const
  ADC_SQR2_SQ5_Pos* = (0)
  ADC_SQR2_SQ5_Msk* = (0x0000001F shl ADC_SQR2_SQ5_Pos) ## !< 0x0000001F
  ADC_SQR2_SQ5* = ADC_SQR2_SQ5_Msk
  ADC_SQR2_SQ5_0* = (0x00000001 shl ADC_SQR2_SQ5_Pos) ## !< 0x00000001
  ADC_SQR2_SQ5_1* = (0x00000002 shl ADC_SQR2_SQ5_Pos) ## !< 0x00000002
  ADC_SQR2_SQ5_2* = (0x00000004 shl ADC_SQR2_SQ5_Pos) ## !< 0x00000004
  ADC_SQR2_SQ5_3* = (0x00000008 shl ADC_SQR2_SQ5_Pos) ## !< 0x00000008
  ADC_SQR2_SQ5_4* = (0x00000010 shl ADC_SQR2_SQ5_Pos) ## !< 0x00000010
  ADC_SQR2_SQ6_Pos* = (6)
  ADC_SQR2_SQ6_Msk* = (0x0000001F shl ADC_SQR2_SQ6_Pos) ## !< 0x000007C0
  ADC_SQR2_SQ6* = ADC_SQR2_SQ6_Msk
  ADC_SQR2_SQ6_0* = (0x00000001 shl ADC_SQR2_SQ6_Pos) ## !< 0x00000040
  ADC_SQR2_SQ6_1* = (0x00000002 shl ADC_SQR2_SQ6_Pos) ## !< 0x00000080
  ADC_SQR2_SQ6_2* = (0x00000004 shl ADC_SQR2_SQ6_Pos) ## !< 0x00000100
  ADC_SQR2_SQ6_3* = (0x00000008 shl ADC_SQR2_SQ6_Pos) ## !< 0x00000200
  ADC_SQR2_SQ6_4* = (0x00000010 shl ADC_SQR2_SQ6_Pos) ## !< 0x00000400
  ADC_SQR2_SQ7_Pos* = (12)
  ADC_SQR2_SQ7_Msk* = (0x0000001F shl ADC_SQR2_SQ7_Pos) ## !< 0x0001F000
  ADC_SQR2_SQ7* = ADC_SQR2_SQ7_Msk
  ADC_SQR2_SQ7_0* = (0x00000001 shl ADC_SQR2_SQ7_Pos) ## !< 0x00001000
  ADC_SQR2_SQ7_1* = (0x00000002 shl ADC_SQR2_SQ7_Pos) ## !< 0x00002000
  ADC_SQR2_SQ7_2* = (0x00000004 shl ADC_SQR2_SQ7_Pos) ## !< 0x00004000
  ADC_SQR2_SQ7_3* = (0x00000008 shl ADC_SQR2_SQ7_Pos) ## !< 0x00008000
  ADC_SQR2_SQ7_4* = (0x00000010 shl ADC_SQR2_SQ7_Pos) ## !< 0x00010000
  ADC_SQR2_SQ8_Pos* = (18)
  ADC_SQR2_SQ8_Msk* = (0x0000001F shl ADC_SQR2_SQ8_Pos) ## !< 0x007C0000
  ADC_SQR2_SQ8* = ADC_SQR2_SQ8_Msk
  ADC_SQR2_SQ8_0* = (0x00000001 shl ADC_SQR2_SQ8_Pos) ## !< 0x00040000
  ADC_SQR2_SQ8_1* = (0x00000002 shl ADC_SQR2_SQ8_Pos) ## !< 0x00080000
  ADC_SQR2_SQ8_2* = (0x00000004 shl ADC_SQR2_SQ8_Pos) ## !< 0x00100000
  ADC_SQR2_SQ8_3* = (0x00000008 shl ADC_SQR2_SQ8_Pos) ## !< 0x00200000
  ADC_SQR2_SQ8_4* = (0x00000010 shl ADC_SQR2_SQ8_Pos) ## !< 0x00400000
  ADC_SQR2_SQ9_Pos* = (24)
  ADC_SQR2_SQ9_Msk* = (0x0000001F shl ADC_SQR2_SQ9_Pos) ## !< 0x1F000000
  ADC_SQR2_SQ9* = ADC_SQR2_SQ9_Msk
  ADC_SQR2_SQ9_0* = (0x00000001 shl ADC_SQR2_SQ9_Pos) ## !< 0x01000000
  ADC_SQR2_SQ9_1* = (0x00000002 shl ADC_SQR2_SQ9_Pos) ## !< 0x02000000
  ADC_SQR2_SQ9_2* = (0x00000004 shl ADC_SQR2_SQ9_Pos) ## !< 0x04000000
  ADC_SQR2_SQ9_3* = (0x00000008 shl ADC_SQR2_SQ9_Pos) ## !< 0x08000000
  ADC_SQR2_SQ9_4* = (0x00000010 shl ADC_SQR2_SQ9_Pos) ## !< 0x10000000

## *******************  Bit definition for ADC_SQR3 register  *****************

const
  ADC_SQR3_SQ10_Pos* = (0)
  ADC_SQR3_SQ10_Msk* = (0x0000001F shl ADC_SQR3_SQ10_Pos) ## !< 0x0000001F
  ADC_SQR3_SQ10* = ADC_SQR3_SQ10_Msk
  ADC_SQR3_SQ10_0* = (0x00000001 shl ADC_SQR3_SQ10_Pos) ## !< 0x00000001
  ADC_SQR3_SQ10_1* = (0x00000002 shl ADC_SQR3_SQ10_Pos) ## !< 0x00000002
  ADC_SQR3_SQ10_2* = (0x00000004 shl ADC_SQR3_SQ10_Pos) ## !< 0x00000004
  ADC_SQR3_SQ10_3* = (0x00000008 shl ADC_SQR3_SQ10_Pos) ## !< 0x00000008
  ADC_SQR3_SQ10_4* = (0x00000010 shl ADC_SQR3_SQ10_Pos) ## !< 0x00000010
  ADC_SQR3_SQ11_Pos* = (6)
  ADC_SQR3_SQ11_Msk* = (0x0000001F shl ADC_SQR3_SQ11_Pos) ## !< 0x000007C0
  ADC_SQR3_SQ11* = ADC_SQR3_SQ11_Msk
  ADC_SQR3_SQ11_0* = (0x00000001 shl ADC_SQR3_SQ11_Pos) ## !< 0x00000040
  ADC_SQR3_SQ11_1* = (0x00000002 shl ADC_SQR3_SQ11_Pos) ## !< 0x00000080
  ADC_SQR3_SQ11_2* = (0x00000004 shl ADC_SQR3_SQ11_Pos) ## !< 0x00000100
  ADC_SQR3_SQ11_3* = (0x00000008 shl ADC_SQR3_SQ11_Pos) ## !< 0x00000200
  ADC_SQR3_SQ11_4* = (0x00000010 shl ADC_SQR3_SQ11_Pos) ## !< 0x00000400
  ADC_SQR3_SQ12_Pos* = (12)
  ADC_SQR3_SQ12_Msk* = (0x0000001F shl ADC_SQR3_SQ12_Pos) ## !< 0x0001F000
  ADC_SQR3_SQ12* = ADC_SQR3_SQ12_Msk
  ADC_SQR3_SQ12_0* = (0x00000001 shl ADC_SQR3_SQ12_Pos) ## !< 0x00001000
  ADC_SQR3_SQ12_1* = (0x00000002 shl ADC_SQR3_SQ12_Pos) ## !< 0x00002000
  ADC_SQR3_SQ12_2* = (0x00000004 shl ADC_SQR3_SQ12_Pos) ## !< 0x00004000
  ADC_SQR3_SQ12_3* = (0x00000008 shl ADC_SQR3_SQ12_Pos) ## !< 0x00008000
  ADC_SQR3_SQ12_4* = (0x00000010 shl ADC_SQR3_SQ12_Pos) ## !< 0x00010000
  ADC_SQR3_SQ13_Pos* = (18)
  ADC_SQR3_SQ13_Msk* = (0x0000001F shl ADC_SQR3_SQ13_Pos) ## !< 0x007C0000
  ADC_SQR3_SQ13* = ADC_SQR3_SQ13_Msk
  ADC_SQR3_SQ13_0* = (0x00000001 shl ADC_SQR3_SQ13_Pos) ## !< 0x00040000
  ADC_SQR3_SQ13_1* = (0x00000002 shl ADC_SQR3_SQ13_Pos) ## !< 0x00080000
  ADC_SQR3_SQ13_2* = (0x00000004 shl ADC_SQR3_SQ13_Pos) ## !< 0x00100000
  ADC_SQR3_SQ13_3* = (0x00000008 shl ADC_SQR3_SQ13_Pos) ## !< 0x00200000
  ADC_SQR3_SQ13_4* = (0x00000010 shl ADC_SQR3_SQ13_Pos) ## !< 0x00400000
  ADC_SQR3_SQ14_Pos* = (24)
  ADC_SQR3_SQ14_Msk* = (0x0000001F shl ADC_SQR3_SQ14_Pos) ## !< 0x1F000000
  ADC_SQR3_SQ14* = ADC_SQR3_SQ14_Msk
  ADC_SQR3_SQ14_0* = (0x00000001 shl ADC_SQR3_SQ14_Pos) ## !< 0x01000000
  ADC_SQR3_SQ14_1* = (0x00000002 shl ADC_SQR3_SQ14_Pos) ## !< 0x02000000
  ADC_SQR3_SQ14_2* = (0x00000004 shl ADC_SQR3_SQ14_Pos) ## !< 0x04000000
  ADC_SQR3_SQ14_3* = (0x00000008 shl ADC_SQR3_SQ14_Pos) ## !< 0x08000000
  ADC_SQR3_SQ14_4* = (0x00000010 shl ADC_SQR3_SQ14_Pos) ## !< 0x10000000

## *******************  Bit definition for ADC_SQR4 register  *****************

const
  ADC_SQR4_SQ15_Pos* = (0)
  ADC_SQR4_SQ15_Msk* = (0x0000001F shl ADC_SQR4_SQ15_Pos) ## !< 0x0000001F
  ADC_SQR4_SQ15* = ADC_SQR4_SQ15_Msk
  ADC_SQR4_SQ15_0* = (0x00000001 shl ADC_SQR4_SQ15_Pos) ## !< 0x00000001
  ADC_SQR4_SQ15_1* = (0x00000002 shl ADC_SQR4_SQ15_Pos) ## !< 0x00000002
  ADC_SQR4_SQ15_2* = (0x00000004 shl ADC_SQR4_SQ15_Pos) ## !< 0x00000004
  ADC_SQR4_SQ15_3* = (0x00000008 shl ADC_SQR4_SQ15_Pos) ## !< 0x00000008
  ADC_SQR4_SQ15_4* = (0x00000010 shl ADC_SQR4_SQ15_Pos) ## !< 0x00000010
  ADC_SQR4_SQ16_Pos* = (6)
  ADC_SQR4_SQ16_Msk* = (0x0000001F shl ADC_SQR4_SQ16_Pos) ## !< 0x000007C0
  ADC_SQR4_SQ16* = ADC_SQR4_SQ16_Msk
  ADC_SQR4_SQ16_0* = (0x00000001 shl ADC_SQR4_SQ16_Pos) ## !< 0x00000040
  ADC_SQR4_SQ16_1* = (0x00000002 shl ADC_SQR4_SQ16_Pos) ## !< 0x00000080
  ADC_SQR4_SQ16_2* = (0x00000004 shl ADC_SQR4_SQ16_Pos) ## !< 0x00000100
  ADC_SQR4_SQ16_3* = (0x00000008 shl ADC_SQR4_SQ16_Pos) ## !< 0x00000200
  ADC_SQR4_SQ16_4* = (0x00000010 shl ADC_SQR4_SQ16_Pos) ## !< 0x00000400

## *******************  Bit definition for ADC_DR register  *******************

const
  ADC_DR_RDATA_Pos* = (0)
  ADC_DR_RDATA_Msk* = (0x0000FFFF shl ADC_DR_RDATA_Pos) ## !< 0x0000FFFF
  ADC_DR_RDATA* = ADC_DR_RDATA_Msk
  ADC_DR_RDATA_0* = (0x00000001 shl ADC_DR_RDATA_Pos) ## !< 0x00000001
  ADC_DR_RDATA_1* = (0x00000002 shl ADC_DR_RDATA_Pos) ## !< 0x00000002
  ADC_DR_RDATA_2* = (0x00000004 shl ADC_DR_RDATA_Pos) ## !< 0x00000004
  ADC_DR_RDATA_3* = (0x00000008 shl ADC_DR_RDATA_Pos) ## !< 0x00000008
  ADC_DR_RDATA_4* = (0x00000010 shl ADC_DR_RDATA_Pos) ## !< 0x00000010
  ADC_DR_RDATA_5* = (0x00000020 shl ADC_DR_RDATA_Pos) ## !< 0x00000020
  ADC_DR_RDATA_6* = (0x00000040 shl ADC_DR_RDATA_Pos) ## !< 0x00000040
  ADC_DR_RDATA_7* = (0x00000080 shl ADC_DR_RDATA_Pos) ## !< 0x00000080
  ADC_DR_RDATA_8* = (0x00000100 shl ADC_DR_RDATA_Pos) ## !< 0x00000100
  ADC_DR_RDATA_9* = (0x00000200 shl ADC_DR_RDATA_Pos) ## !< 0x00000200
  ADC_DR_RDATA_10* = (0x00000400 shl ADC_DR_RDATA_Pos) ## !< 0x00000400
  ADC_DR_RDATA_11* = (0x00000800 shl ADC_DR_RDATA_Pos) ## !< 0x00000800
  ADC_DR_RDATA_12* = (0x00001000 shl ADC_DR_RDATA_Pos) ## !< 0x00001000
  ADC_DR_RDATA_13* = (0x00002000 shl ADC_DR_RDATA_Pos) ## !< 0x00002000
  ADC_DR_RDATA_14* = (0x00004000 shl ADC_DR_RDATA_Pos) ## !< 0x00004000
  ADC_DR_RDATA_15* = (0x00008000 shl ADC_DR_RDATA_Pos) ## !< 0x00008000

## *******************  Bit definition for ADC_JSQR register  *****************

const
  ADC_JSQR_JL_Pos* = (0)
  ADC_JSQR_JL_Msk* = (0x00000003 shl ADC_JSQR_JL_Pos) ## !< 0x00000003
  ADC_JSQR_JL* = ADC_JSQR_JL_Msk
  ADC_JSQR_JL_0* = (0x00000001 shl ADC_JSQR_JL_Pos) ## !< 0x00000001
  ADC_JSQR_JL_1* = (0x00000002 shl ADC_JSQR_JL_Pos) ## !< 0x00000002
  ADC_JSQR_JEXTSEL_Pos* = (2)
  ADC_JSQR_JEXTSEL_Msk* = (0x0000000F shl ADC_JSQR_JEXTSEL_Pos) ## !< 0x0000003C
  ADC_JSQR_JEXTSEL* = ADC_JSQR_JEXTSEL_Msk
  ADC_JSQR_JEXTSEL_0* = (0x00000001 shl ADC_JSQR_JEXTSEL_Pos) ## !< 0x00000004
  ADC_JSQR_JEXTSEL_1* = (0x00000002 shl ADC_JSQR_JEXTSEL_Pos) ## !< 0x00000008
  ADC_JSQR_JEXTSEL_2* = (0x00000004 shl ADC_JSQR_JEXTSEL_Pos) ## !< 0x00000010
  ADC_JSQR_JEXTSEL_3* = (0x00000008 shl ADC_JSQR_JEXTSEL_Pos) ## !< 0x00000020
  ADC_JSQR_JEXTEN_Pos* = (6)
  ADC_JSQR_JEXTEN_Msk* = (0x00000003 shl ADC_JSQR_JEXTEN_Pos) ## !< 0x000000C0
  ADC_JSQR_JEXTEN* = ADC_JSQR_JEXTEN_Msk
  ADC_JSQR_JEXTEN_0* = (0x00000001 shl ADC_JSQR_JEXTEN_Pos) ## !< 0x00000040
  ADC_JSQR_JEXTEN_1* = (0x00000002 shl ADC_JSQR_JEXTEN_Pos) ## !< 0x00000080
  ADC_JSQR_JSQ1_Pos* = (8)
  ADC_JSQR_JSQ1_Msk* = (0x0000001F shl ADC_JSQR_JSQ1_Pos) ## !< 0x00001F00
  ADC_JSQR_JSQ1* = ADC_JSQR_JSQ1_Msk
  ADC_JSQR_JSQ1_0* = (0x00000001 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00000100
  ADC_JSQR_JSQ1_1* = (0x00000002 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00000200
  ADC_JSQR_JSQ1_2* = (0x00000004 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00000400
  ADC_JSQR_JSQ1_3* = (0x00000008 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00000800
  ADC_JSQR_JSQ1_4* = (0x00000010 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00001000
  ADC_JSQR_JSQ2_Pos* = (14)
  ADC_JSQR_JSQ2_Msk* = (0x0000001F shl ADC_JSQR_JSQ2_Pos) ## !< 0x0007C000
  ADC_JSQR_JSQ2* = ADC_JSQR_JSQ2_Msk
  ADC_JSQR_JSQ2_0* = (0x00000001 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00004000
  ADC_JSQR_JSQ2_1* = (0x00000002 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00008000
  ADC_JSQR_JSQ2_2* = (0x00000004 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00010000
  ADC_JSQR_JSQ2_3* = (0x00000008 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00020000
  ADC_JSQR_JSQ2_4* = (0x00000010 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00040000
  ADC_JSQR_JSQ3_Pos* = (20)
  ADC_JSQR_JSQ3_Msk* = (0x0000001F shl ADC_JSQR_JSQ3_Pos) ## !< 0x01F00000
  ADC_JSQR_JSQ3* = ADC_JSQR_JSQ3_Msk
  ADC_JSQR_JSQ3_0* = (0x00000001 shl ADC_JSQR_JSQ3_Pos) ## !< 0x00100000
  ADC_JSQR_JSQ3_1* = (0x00000002 shl ADC_JSQR_JSQ3_Pos) ## !< 0x00200000
  ADC_JSQR_JSQ3_2* = (0x00000004 shl ADC_JSQR_JSQ3_Pos) ## !< 0x00400000
  ADC_JSQR_JSQ3_3* = (0x00000008 shl ADC_JSQR_JSQ3_Pos) ## !< 0x00800000
  ADC_JSQR_JSQ3_4* = (0x00000010 shl ADC_JSQR_JSQ3_Pos) ## !< 0x01000000
  ADC_JSQR_JSQ4_Pos* = (26)
  ADC_JSQR_JSQ4_Msk* = (0x0000001F shl ADC_JSQR_JSQ4_Pos) ## !< 0x7C000000
  ADC_JSQR_JSQ4* = ADC_JSQR_JSQ4_Msk
  ADC_JSQR_JSQ4_0* = (0x00000001 shl ADC_JSQR_JSQ4_Pos) ## !< 0x04000000
  ADC_JSQR_JSQ4_1* = (0x00000002 shl ADC_JSQR_JSQ4_Pos) ## !< 0x08000000
  ADC_JSQR_JSQ4_2* = (0x00000004 shl ADC_JSQR_JSQ4_Pos) ## !< 0x10000000
  ADC_JSQR_JSQ4_3* = (0x00000008 shl ADC_JSQR_JSQ4_Pos) ## !< 0x20000000
  ADC_JSQR_JSQ4_4* = (0x00000010 shl ADC_JSQR_JSQ4_Pos) ## !< 0x40000000

## *******************  Bit definition for ADC_OFR1 register  *****************

const
  ADC_OFR1_OFFSET1_Pos* = (0)
  ADC_OFR1_OFFSET1_Msk* = (0x00000FFF shl ADC_OFR1_OFFSET1_Pos) ## !< 0x00000FFF
  ADC_OFR1_OFFSET1* = ADC_OFR1_OFFSET1_Msk
  ADC_OFR1_OFFSET1_0* = (0x00000001 shl ADC_OFR1_OFFSET1_Pos) ## !< 0x00000001
  ADC_OFR1_OFFSET1_1* = (0x00000002 shl ADC_OFR1_OFFSET1_Pos) ## !< 0x00000002
  ADC_OFR1_OFFSET1_2* = (0x00000004 shl ADC_OFR1_OFFSET1_Pos) ## !< 0x00000004
  ADC_OFR1_OFFSET1_3* = (0x00000008 shl ADC_OFR1_OFFSET1_Pos) ## !< 0x00000008
  ADC_OFR1_OFFSET1_4* = (0x00000010 shl ADC_OFR1_OFFSET1_Pos) ## !< 0x00000010
  ADC_OFR1_OFFSET1_5* = (0x00000020 shl ADC_OFR1_OFFSET1_Pos) ## !< 0x00000020
  ADC_OFR1_OFFSET1_6* = (0x00000040 shl ADC_OFR1_OFFSET1_Pos) ## !< 0x00000040
  ADC_OFR1_OFFSET1_7* = (0x00000080 shl ADC_OFR1_OFFSET1_Pos) ## !< 0x00000080
  ADC_OFR1_OFFSET1_8* = (0x00000100 shl ADC_OFR1_OFFSET1_Pos) ## !< 0x00000100
  ADC_OFR1_OFFSET1_9* = (0x00000200 shl ADC_OFR1_OFFSET1_Pos) ## !< 0x00000200
  ADC_OFR1_OFFSET1_10* = (0x00000400 shl ADC_OFR1_OFFSET1_Pos) ## !< 0x00000400
  ADC_OFR1_OFFSET1_11* = (0x00000800 shl ADC_OFR1_OFFSET1_Pos) ## !< 0x00000800
  ADC_OFR1_OFFSET1_CH_Pos* = (26)
  ADC_OFR1_OFFSET1_CH_Msk* = (0x0000001F shl ADC_OFR1_OFFSET1_CH_Pos) ## !< 0x7C000000
  ADC_OFR1_OFFSET1_CH* = ADC_OFR1_OFFSET1_CH_Msk
  ADC_OFR1_OFFSET1_CH_0* = (0x00000001 shl ADC_OFR1_OFFSET1_CH_Pos) ## !< 0x04000000
  ADC_OFR1_OFFSET1_CH_1* = (0x00000002 shl ADC_OFR1_OFFSET1_CH_Pos) ## !< 0x08000000
  ADC_OFR1_OFFSET1_CH_2* = (0x00000004 shl ADC_OFR1_OFFSET1_CH_Pos) ## !< 0x10000000
  ADC_OFR1_OFFSET1_CH_3* = (0x00000008 shl ADC_OFR1_OFFSET1_CH_Pos) ## !< 0x20000000
  ADC_OFR1_OFFSET1_CH_4* = (0x00000010 shl ADC_OFR1_OFFSET1_CH_Pos) ## !< 0x40000000
  ADC_OFR1_OFFSET1_EN_Pos* = (31)
  ADC_OFR1_OFFSET1_EN_Msk* = (0x00000001 shl ADC_OFR1_OFFSET1_EN_Pos) ## !< 0x80000000
  ADC_OFR1_OFFSET1_EN* = ADC_OFR1_OFFSET1_EN_Msk

## *******************  Bit definition for ADC_OFR2 register  *****************

const
  ADC_OFR2_OFFSET2_Pos* = (0)
  ADC_OFR2_OFFSET2_Msk* = (0x00000FFF shl ADC_OFR2_OFFSET2_Pos) ## !< 0x00000FFF
  ADC_OFR2_OFFSET2* = ADC_OFR2_OFFSET2_Msk
  ADC_OFR2_OFFSET2_0* = (0x00000001 shl ADC_OFR2_OFFSET2_Pos) ## !< 0x00000001
  ADC_OFR2_OFFSET2_1* = (0x00000002 shl ADC_OFR2_OFFSET2_Pos) ## !< 0x00000002
  ADC_OFR2_OFFSET2_2* = (0x00000004 shl ADC_OFR2_OFFSET2_Pos) ## !< 0x00000004
  ADC_OFR2_OFFSET2_3* = (0x00000008 shl ADC_OFR2_OFFSET2_Pos) ## !< 0x00000008
  ADC_OFR2_OFFSET2_4* = (0x00000010 shl ADC_OFR2_OFFSET2_Pos) ## !< 0x00000010
  ADC_OFR2_OFFSET2_5* = (0x00000020 shl ADC_OFR2_OFFSET2_Pos) ## !< 0x00000020
  ADC_OFR2_OFFSET2_6* = (0x00000040 shl ADC_OFR2_OFFSET2_Pos) ## !< 0x00000040
  ADC_OFR2_OFFSET2_7* = (0x00000080 shl ADC_OFR2_OFFSET2_Pos) ## !< 0x00000080
  ADC_OFR2_OFFSET2_8* = (0x00000100 shl ADC_OFR2_OFFSET2_Pos) ## !< 0x00000100
  ADC_OFR2_OFFSET2_9* = (0x00000200 shl ADC_OFR2_OFFSET2_Pos) ## !< 0x00000200
  ADC_OFR2_OFFSET2_10* = (0x00000400 shl ADC_OFR2_OFFSET2_Pos) ## !< 0x00000400
  ADC_OFR2_OFFSET2_11* = (0x00000800 shl ADC_OFR2_OFFSET2_Pos) ## !< 0x00000800
  ADC_OFR2_OFFSET2_CH_Pos* = (26)
  ADC_OFR2_OFFSET2_CH_Msk* = (0x0000001F shl ADC_OFR2_OFFSET2_CH_Pos) ## !< 0x7C000000
  ADC_OFR2_OFFSET2_CH* = ADC_OFR2_OFFSET2_CH_Msk
  ADC_OFR2_OFFSET2_CH_0* = (0x00000001 shl ADC_OFR2_OFFSET2_CH_Pos) ## !< 0x04000000
  ADC_OFR2_OFFSET2_CH_1* = (0x00000002 shl ADC_OFR2_OFFSET2_CH_Pos) ## !< 0x08000000
  ADC_OFR2_OFFSET2_CH_2* = (0x00000004 shl ADC_OFR2_OFFSET2_CH_Pos) ## !< 0x10000000
  ADC_OFR2_OFFSET2_CH_3* = (0x00000008 shl ADC_OFR2_OFFSET2_CH_Pos) ## !< 0x20000000
  ADC_OFR2_OFFSET2_CH_4* = (0x00000010 shl ADC_OFR2_OFFSET2_CH_Pos) ## !< 0x40000000
  ADC_OFR2_OFFSET2_EN_Pos* = (31)
  ADC_OFR2_OFFSET2_EN_Msk* = (0x00000001 shl ADC_OFR2_OFFSET2_EN_Pos) ## !< 0x80000000
  ADC_OFR2_OFFSET2_EN* = ADC_OFR2_OFFSET2_EN_Msk

## *******************  Bit definition for ADC_OFR3 register  *****************

const
  ADC_OFR3_OFFSET3_Pos* = (0)
  ADC_OFR3_OFFSET3_Msk* = (0x00000FFF shl ADC_OFR3_OFFSET3_Pos) ## !< 0x00000FFF
  ADC_OFR3_OFFSET3* = ADC_OFR3_OFFSET3_Msk
  ADC_OFR3_OFFSET3_0* = (0x00000001 shl ADC_OFR3_OFFSET3_Pos) ## !< 0x00000001
  ADC_OFR3_OFFSET3_1* = (0x00000002 shl ADC_OFR3_OFFSET3_Pos) ## !< 0x00000002
  ADC_OFR3_OFFSET3_2* = (0x00000004 shl ADC_OFR3_OFFSET3_Pos) ## !< 0x00000004
  ADC_OFR3_OFFSET3_3* = (0x00000008 shl ADC_OFR3_OFFSET3_Pos) ## !< 0x00000008
  ADC_OFR3_OFFSET3_4* = (0x00000010 shl ADC_OFR3_OFFSET3_Pos) ## !< 0x00000010
  ADC_OFR3_OFFSET3_5* = (0x00000020 shl ADC_OFR3_OFFSET3_Pos) ## !< 0x00000020
  ADC_OFR3_OFFSET3_6* = (0x00000040 shl ADC_OFR3_OFFSET3_Pos) ## !< 0x00000040
  ADC_OFR3_OFFSET3_7* = (0x00000080 shl ADC_OFR3_OFFSET3_Pos) ## !< 0x00000080
  ADC_OFR3_OFFSET3_8* = (0x00000100 shl ADC_OFR3_OFFSET3_Pos) ## !< 0x00000100
  ADC_OFR3_OFFSET3_9* = (0x00000200 shl ADC_OFR3_OFFSET3_Pos) ## !< 0x00000200
  ADC_OFR3_OFFSET3_10* = (0x00000400 shl ADC_OFR3_OFFSET3_Pos) ## !< 0x00000400
  ADC_OFR3_OFFSET3_11* = (0x00000800 shl ADC_OFR3_OFFSET3_Pos) ## !< 0x00000800
  ADC_OFR3_OFFSET3_CH_Pos* = (26)
  ADC_OFR3_OFFSET3_CH_Msk* = (0x0000001F shl ADC_OFR3_OFFSET3_CH_Pos) ## !< 0x7C000000
  ADC_OFR3_OFFSET3_CH* = ADC_OFR3_OFFSET3_CH_Msk
  ADC_OFR3_OFFSET3_CH_0* = (0x00000001 shl ADC_OFR3_OFFSET3_CH_Pos) ## !< 0x04000000
  ADC_OFR3_OFFSET3_CH_1* = (0x00000002 shl ADC_OFR3_OFFSET3_CH_Pos) ## !< 0x08000000
  ADC_OFR3_OFFSET3_CH_2* = (0x00000004 shl ADC_OFR3_OFFSET3_CH_Pos) ## !< 0x10000000
  ADC_OFR3_OFFSET3_CH_3* = (0x00000008 shl ADC_OFR3_OFFSET3_CH_Pos) ## !< 0x20000000
  ADC_OFR3_OFFSET3_CH_4* = (0x00000010 shl ADC_OFR3_OFFSET3_CH_Pos) ## !< 0x40000000
  ADC_OFR3_OFFSET3_EN_Pos* = (31)
  ADC_OFR3_OFFSET3_EN_Msk* = (0x00000001 shl ADC_OFR3_OFFSET3_EN_Pos) ## !< 0x80000000
  ADC_OFR3_OFFSET3_EN* = ADC_OFR3_OFFSET3_EN_Msk

## *******************  Bit definition for ADC_OFR4 register  *****************

const
  ADC_OFR4_OFFSET4_Pos* = (0)
  ADC_OFR4_OFFSET4_Msk* = (0x00000FFF shl ADC_OFR4_OFFSET4_Pos) ## !< 0x00000FFF
  ADC_OFR4_OFFSET4* = ADC_OFR4_OFFSET4_Msk
  ADC_OFR4_OFFSET4_0* = (0x00000001 shl ADC_OFR4_OFFSET4_Pos) ## !< 0x00000001
  ADC_OFR4_OFFSET4_1* = (0x00000002 shl ADC_OFR4_OFFSET4_Pos) ## !< 0x00000002
  ADC_OFR4_OFFSET4_2* = (0x00000004 shl ADC_OFR4_OFFSET4_Pos) ## !< 0x00000004
  ADC_OFR4_OFFSET4_3* = (0x00000008 shl ADC_OFR4_OFFSET4_Pos) ## !< 0x00000008
  ADC_OFR4_OFFSET4_4* = (0x00000010 shl ADC_OFR4_OFFSET4_Pos) ## !< 0x00000010
  ADC_OFR4_OFFSET4_5* = (0x00000020 shl ADC_OFR4_OFFSET4_Pos) ## !< 0x00000020
  ADC_OFR4_OFFSET4_6* = (0x00000040 shl ADC_OFR4_OFFSET4_Pos) ## !< 0x00000040
  ADC_OFR4_OFFSET4_7* = (0x00000080 shl ADC_OFR4_OFFSET4_Pos) ## !< 0x00000080
  ADC_OFR4_OFFSET4_8* = (0x00000100 shl ADC_OFR4_OFFSET4_Pos) ## !< 0x00000100
  ADC_OFR4_OFFSET4_9* = (0x00000200 shl ADC_OFR4_OFFSET4_Pos) ## !< 0x00000200
  ADC_OFR4_OFFSET4_10* = (0x00000400 shl ADC_OFR4_OFFSET4_Pos) ## !< 0x00000400
  ADC_OFR4_OFFSET4_11* = (0x00000800 shl ADC_OFR4_OFFSET4_Pos) ## !< 0x00000800
  ADC_OFR4_OFFSET4_CH_Pos* = (26)
  ADC_OFR4_OFFSET4_CH_Msk* = (0x0000001F shl ADC_OFR4_OFFSET4_CH_Pos) ## !< 0x7C000000
  ADC_OFR4_OFFSET4_CH* = ADC_OFR4_OFFSET4_CH_Msk
  ADC_OFR4_OFFSET4_CH_0* = (0x00000001 shl ADC_OFR4_OFFSET4_CH_Pos) ## !< 0x04000000
  ADC_OFR4_OFFSET4_CH_1* = (0x00000002 shl ADC_OFR4_OFFSET4_CH_Pos) ## !< 0x08000000
  ADC_OFR4_OFFSET4_CH_2* = (0x00000004 shl ADC_OFR4_OFFSET4_CH_Pos) ## !< 0x10000000
  ADC_OFR4_OFFSET4_CH_3* = (0x00000008 shl ADC_OFR4_OFFSET4_CH_Pos) ## !< 0x20000000
  ADC_OFR4_OFFSET4_CH_4* = (0x00000010 shl ADC_OFR4_OFFSET4_CH_Pos) ## !< 0x40000000
  ADC_OFR4_OFFSET4_EN_Pos* = (31)
  ADC_OFR4_OFFSET4_EN_Msk* = (0x00000001 shl ADC_OFR4_OFFSET4_EN_Pos) ## !< 0x80000000
  ADC_OFR4_OFFSET4_EN* = ADC_OFR4_OFFSET4_EN_Msk

## *******************  Bit definition for ADC_JDR1 register  *****************

const
  ADC_JDR1_JDATA_Pos* = (0)
  ADC_JDR1_JDATA_Msk* = (0x0000FFFF shl ADC_JDR1_JDATA_Pos) ## !< 0x0000FFFF
  ADC_JDR1_JDATA* = ADC_JDR1_JDATA_Msk
  ADC_JDR1_JDATA_0* = (0x00000001 shl ADC_JDR1_JDATA_Pos) ## !< 0x00000001
  ADC_JDR1_JDATA_1* = (0x00000002 shl ADC_JDR1_JDATA_Pos) ## !< 0x00000002
  ADC_JDR1_JDATA_2* = (0x00000004 shl ADC_JDR1_JDATA_Pos) ## !< 0x00000004
  ADC_JDR1_JDATA_3* = (0x00000008 shl ADC_JDR1_JDATA_Pos) ## !< 0x00000008
  ADC_JDR1_JDATA_4* = (0x00000010 shl ADC_JDR1_JDATA_Pos) ## !< 0x00000010
  ADC_JDR1_JDATA_5* = (0x00000020 shl ADC_JDR1_JDATA_Pos) ## !< 0x00000020
  ADC_JDR1_JDATA_6* = (0x00000040 shl ADC_JDR1_JDATA_Pos) ## !< 0x00000040
  ADC_JDR1_JDATA_7* = (0x00000080 shl ADC_JDR1_JDATA_Pos) ## !< 0x00000080
  ADC_JDR1_JDATA_8* = (0x00000100 shl ADC_JDR1_JDATA_Pos) ## !< 0x00000100
  ADC_JDR1_JDATA_9* = (0x00000200 shl ADC_JDR1_JDATA_Pos) ## !< 0x00000200
  ADC_JDR1_JDATA_10* = (0x00000400 shl ADC_JDR1_JDATA_Pos) ## !< 0x00000400
  ADC_JDR1_JDATA_11* = (0x00000800 shl ADC_JDR1_JDATA_Pos) ## !< 0x00000800
  ADC_JDR1_JDATA_12* = (0x00001000 shl ADC_JDR1_JDATA_Pos) ## !< 0x00001000
  ADC_JDR1_JDATA_13* = (0x00002000 shl ADC_JDR1_JDATA_Pos) ## !< 0x00002000
  ADC_JDR1_JDATA_14* = (0x00004000 shl ADC_JDR1_JDATA_Pos) ## !< 0x00004000
  ADC_JDR1_JDATA_15* = (0x00008000 shl ADC_JDR1_JDATA_Pos) ## !< 0x00008000

## *******************  Bit definition for ADC_JDR2 register  *****************

const
  ADC_JDR2_JDATA_Pos* = (0)
  ADC_JDR2_JDATA_Msk* = (0x0000FFFF shl ADC_JDR2_JDATA_Pos) ## !< 0x0000FFFF
  ADC_JDR2_JDATA* = ADC_JDR2_JDATA_Msk
  ADC_JDR2_JDATA_0* = (0x00000001 shl ADC_JDR2_JDATA_Pos) ## !< 0x00000001
  ADC_JDR2_JDATA_1* = (0x00000002 shl ADC_JDR2_JDATA_Pos) ## !< 0x00000002
  ADC_JDR2_JDATA_2* = (0x00000004 shl ADC_JDR2_JDATA_Pos) ## !< 0x00000004
  ADC_JDR2_JDATA_3* = (0x00000008 shl ADC_JDR2_JDATA_Pos) ## !< 0x00000008
  ADC_JDR2_JDATA_4* = (0x00000010 shl ADC_JDR2_JDATA_Pos) ## !< 0x00000010
  ADC_JDR2_JDATA_5* = (0x00000020 shl ADC_JDR2_JDATA_Pos) ## !< 0x00000020
  ADC_JDR2_JDATA_6* = (0x00000040 shl ADC_JDR2_JDATA_Pos) ## !< 0x00000040
  ADC_JDR2_JDATA_7* = (0x00000080 shl ADC_JDR2_JDATA_Pos) ## !< 0x00000080
  ADC_JDR2_JDATA_8* = (0x00000100 shl ADC_JDR2_JDATA_Pos) ## !< 0x00000100
  ADC_JDR2_JDATA_9* = (0x00000200 shl ADC_JDR2_JDATA_Pos) ## !< 0x00000200
  ADC_JDR2_JDATA_10* = (0x00000400 shl ADC_JDR2_JDATA_Pos) ## !< 0x00000400
  ADC_JDR2_JDATA_11* = (0x00000800 shl ADC_JDR2_JDATA_Pos) ## !< 0x00000800
  ADC_JDR2_JDATA_12* = (0x00001000 shl ADC_JDR2_JDATA_Pos) ## !< 0x00001000
  ADC_JDR2_JDATA_13* = (0x00002000 shl ADC_JDR2_JDATA_Pos) ## !< 0x00002000
  ADC_JDR2_JDATA_14* = (0x00004000 shl ADC_JDR2_JDATA_Pos) ## !< 0x00004000
  ADC_JDR2_JDATA_15* = (0x00008000 shl ADC_JDR2_JDATA_Pos) ## !< 0x00008000

## *******************  Bit definition for ADC_JDR3 register  *****************

const
  ADC_JDR3_JDATA_Pos* = (0)
  ADC_JDR3_JDATA_Msk* = (0x0000FFFF shl ADC_JDR3_JDATA_Pos) ## !< 0x0000FFFF
  ADC_JDR3_JDATA* = ADC_JDR3_JDATA_Msk
  ADC_JDR3_JDATA_0* = (0x00000001 shl ADC_JDR3_JDATA_Pos) ## !< 0x00000001
  ADC_JDR3_JDATA_1* = (0x00000002 shl ADC_JDR3_JDATA_Pos) ## !< 0x00000002
  ADC_JDR3_JDATA_2* = (0x00000004 shl ADC_JDR3_JDATA_Pos) ## !< 0x00000004
  ADC_JDR3_JDATA_3* = (0x00000008 shl ADC_JDR3_JDATA_Pos) ## !< 0x00000008
  ADC_JDR3_JDATA_4* = (0x00000010 shl ADC_JDR3_JDATA_Pos) ## !< 0x00000010
  ADC_JDR3_JDATA_5* = (0x00000020 shl ADC_JDR3_JDATA_Pos) ## !< 0x00000020
  ADC_JDR3_JDATA_6* = (0x00000040 shl ADC_JDR3_JDATA_Pos) ## !< 0x00000040
  ADC_JDR3_JDATA_7* = (0x00000080 shl ADC_JDR3_JDATA_Pos) ## !< 0x00000080
  ADC_JDR3_JDATA_8* = (0x00000100 shl ADC_JDR3_JDATA_Pos) ## !< 0x00000100
  ADC_JDR3_JDATA_9* = (0x00000200 shl ADC_JDR3_JDATA_Pos) ## !< 0x00000200
  ADC_JDR3_JDATA_10* = (0x00000400 shl ADC_JDR3_JDATA_Pos) ## !< 0x00000400
  ADC_JDR3_JDATA_11* = (0x00000800 shl ADC_JDR3_JDATA_Pos) ## !< 0x00000800
  ADC_JDR3_JDATA_12* = (0x00001000 shl ADC_JDR3_JDATA_Pos) ## !< 0x00001000
  ADC_JDR3_JDATA_13* = (0x00002000 shl ADC_JDR3_JDATA_Pos) ## !< 0x00002000
  ADC_JDR3_JDATA_14* = (0x00004000 shl ADC_JDR3_JDATA_Pos) ## !< 0x00004000
  ADC_JDR3_JDATA_15* = (0x00008000 shl ADC_JDR3_JDATA_Pos) ## !< 0x00008000

## *******************  Bit definition for ADC_JDR4 register  *****************

const
  ADC_JDR4_JDATA_Pos* = (0)
  ADC_JDR4_JDATA_Msk* = (0x0000FFFF shl ADC_JDR4_JDATA_Pos) ## !< 0x0000FFFF
  ADC_JDR4_JDATA* = ADC_JDR4_JDATA_Msk
  ADC_JDR4_JDATA_0* = (0x00000001 shl ADC_JDR4_JDATA_Pos) ## !< 0x00000001
  ADC_JDR4_JDATA_1* = (0x00000002 shl ADC_JDR4_JDATA_Pos) ## !< 0x00000002
  ADC_JDR4_JDATA_2* = (0x00000004 shl ADC_JDR4_JDATA_Pos) ## !< 0x00000004
  ADC_JDR4_JDATA_3* = (0x00000008 shl ADC_JDR4_JDATA_Pos) ## !< 0x00000008
  ADC_JDR4_JDATA_4* = (0x00000010 shl ADC_JDR4_JDATA_Pos) ## !< 0x00000010
  ADC_JDR4_JDATA_5* = (0x00000020 shl ADC_JDR4_JDATA_Pos) ## !< 0x00000020
  ADC_JDR4_JDATA_6* = (0x00000040 shl ADC_JDR4_JDATA_Pos) ## !< 0x00000040
  ADC_JDR4_JDATA_7* = (0x00000080 shl ADC_JDR4_JDATA_Pos) ## !< 0x00000080
  ADC_JDR4_JDATA_8* = (0x00000100 shl ADC_JDR4_JDATA_Pos) ## !< 0x00000100
  ADC_JDR4_JDATA_9* = (0x00000200 shl ADC_JDR4_JDATA_Pos) ## !< 0x00000200
  ADC_JDR4_JDATA_10* = (0x00000400 shl ADC_JDR4_JDATA_Pos) ## !< 0x00000400
  ADC_JDR4_JDATA_11* = (0x00000800 shl ADC_JDR4_JDATA_Pos) ## !< 0x00000800
  ADC_JDR4_JDATA_12* = (0x00001000 shl ADC_JDR4_JDATA_Pos) ## !< 0x00001000
  ADC_JDR4_JDATA_13* = (0x00002000 shl ADC_JDR4_JDATA_Pos) ## !< 0x00002000
  ADC_JDR4_JDATA_14* = (0x00004000 shl ADC_JDR4_JDATA_Pos) ## !< 0x00004000
  ADC_JDR4_JDATA_15* = (0x00008000 shl ADC_JDR4_JDATA_Pos) ## !< 0x00008000

## *******************  Bit definition for ADC_AWD2CR register  ***************

const
  ADC_AWD2CR_AWD2CH_Pos* = (0)
  ADC_AWD2CR_AWD2CH_Msk* = (0x0007FFFF shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x0007FFFF
  ADC_AWD2CR_AWD2CH* = ADC_AWD2CR_AWD2CH_Msk
  ADC_AWD2CR_AWD2CH_0* = (0x00000001 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00000001
  ADC_AWD2CR_AWD2CH_1* = (0x00000002 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00000002
  ADC_AWD2CR_AWD2CH_2* = (0x00000004 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00000004
  ADC_AWD2CR_AWD2CH_3* = (0x00000008 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00000008
  ADC_AWD2CR_AWD2CH_4* = (0x00000010 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00000010
  ADC_AWD2CR_AWD2CH_5* = (0x00000020 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00000020
  ADC_AWD2CR_AWD2CH_6* = (0x00000040 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00000040
  ADC_AWD2CR_AWD2CH_7* = (0x00000080 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00000080
  ADC_AWD2CR_AWD2CH_8* = (0x00000100 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00000100
  ADC_AWD2CR_AWD2CH_9* = (0x00000200 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00000200
  ADC_AWD2CR_AWD2CH_10* = (0x00000400 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00000400
  ADC_AWD2CR_AWD2CH_11* = (0x00000800 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00000800
  ADC_AWD2CR_AWD2CH_12* = (0x00001000 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00001000
  ADC_AWD2CR_AWD2CH_13* = (0x00002000 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00002000
  ADC_AWD2CR_AWD2CH_14* = (0x00004000 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00004000
  ADC_AWD2CR_AWD2CH_15* = (0x00008000 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00008000
  ADC_AWD2CR_AWD2CH_16* = (0x00010000 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00010000
  ADC_AWD2CR_AWD2CH_17* = (0x00020000 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00020000
  ADC_AWD2CR_AWD2CH_18* = (0x00040000 shl ADC_AWD2CR_AWD2CH_Pos) ## !< 0x00040000

## *******************  Bit definition for ADC_AWD3CR register  ***************

const
  ADC_AWD3CR_AWD3CH_Pos* = (0)
  ADC_AWD3CR_AWD3CH_Msk* = (0x0007FFFF shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x0007FFFF
  ADC_AWD3CR_AWD3CH* = ADC_AWD3CR_AWD3CH_Msk
  ADC_AWD3CR_AWD3CH_0* = (0x00000001 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00000001
  ADC_AWD3CR_AWD3CH_1* = (0x00000002 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00000002
  ADC_AWD3CR_AWD3CH_2* = (0x00000004 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00000004
  ADC_AWD3CR_AWD3CH_3* = (0x00000008 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00000008
  ADC_AWD3CR_AWD3CH_4* = (0x00000010 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00000010
  ADC_AWD3CR_AWD3CH_5* = (0x00000020 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00000020
  ADC_AWD3CR_AWD3CH_6* = (0x00000040 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00000040
  ADC_AWD3CR_AWD3CH_7* = (0x00000080 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00000080
  ADC_AWD3CR_AWD3CH_8* = (0x00000100 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00000100
  ADC_AWD3CR_AWD3CH_9* = (0x00000200 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00000200
  ADC_AWD3CR_AWD3CH_10* = (0x00000400 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00000400
  ADC_AWD3CR_AWD3CH_11* = (0x00000800 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00000800
  ADC_AWD3CR_AWD3CH_12* = (0x00001000 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00001000
  ADC_AWD3CR_AWD3CH_13* = (0x00002000 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00002000
  ADC_AWD3CR_AWD3CH_14* = (0x00004000 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00004000
  ADC_AWD3CR_AWD3CH_15* = (0x00008000 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00008000
  ADC_AWD3CR_AWD3CH_16* = (0x00010000 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00010000
  ADC_AWD3CR_AWD3CH_17* = (0x00020000 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00020000
  ADC_AWD3CR_AWD3CH_18* = (0x00040000 shl ADC_AWD3CR_AWD3CH_Pos) ## !< 0x00040000

## *******************  Bit definition for ADC_DIFSEL register  ***************

const
  ADC_DIFSEL_DIFSEL_Pos* = (0)
  ADC_DIFSEL_DIFSEL_Msk* = (0x0007FFFF shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x0007FFFF
  ADC_DIFSEL_DIFSEL* = ADC_DIFSEL_DIFSEL_Msk
  ADC_DIFSEL_DIFSEL_0* = (0x00000001 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00000001
  ADC_DIFSEL_DIFSEL_1* = (0x00000002 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00000002
  ADC_DIFSEL_DIFSEL_2* = (0x00000004 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00000004
  ADC_DIFSEL_DIFSEL_3* = (0x00000008 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00000008
  ADC_DIFSEL_DIFSEL_4* = (0x00000010 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00000010
  ADC_DIFSEL_DIFSEL_5* = (0x00000020 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00000020
  ADC_DIFSEL_DIFSEL_6* = (0x00000040 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00000040
  ADC_DIFSEL_DIFSEL_7* = (0x00000080 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00000080
  ADC_DIFSEL_DIFSEL_8* = (0x00000100 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00000100
  ADC_DIFSEL_DIFSEL_9* = (0x00000200 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00000200
  ADC_DIFSEL_DIFSEL_10* = (0x00000400 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00000400
  ADC_DIFSEL_DIFSEL_11* = (0x00000800 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00000800
  ADC_DIFSEL_DIFSEL_12* = (0x00001000 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00001000
  ADC_DIFSEL_DIFSEL_13* = (0x00002000 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00002000
  ADC_DIFSEL_DIFSEL_14* = (0x00004000 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00004000
  ADC_DIFSEL_DIFSEL_15* = (0x00008000 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00008000
  ADC_DIFSEL_DIFSEL_16* = (0x00010000 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00010000
  ADC_DIFSEL_DIFSEL_17* = (0x00020000 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00020000
  ADC_DIFSEL_DIFSEL_18* = (0x00040000 shl ADC_DIFSEL_DIFSEL_Pos) ## !< 0x00040000

## *******************  Bit definition for ADC_CALFACT register  **************

const
  ADC_CALFACT_CALFACT_S_Pos* = (0)
  ADC_CALFACT_CALFACT_S_Msk* = (0x0000007F shl ADC_CALFACT_CALFACT_S_Pos) ## !< 0x0000007F
  ADC_CALFACT_CALFACT_S* = ADC_CALFACT_CALFACT_S_Msk
  ADC_CALFACT_CALFACT_S_0* = (0x00000001 shl ADC_CALFACT_CALFACT_S_Pos) ## !< 0x00000001
  ADC_CALFACT_CALFACT_S_1* = (0x00000002 shl ADC_CALFACT_CALFACT_S_Pos) ## !< 0x00000002
  ADC_CALFACT_CALFACT_S_2* = (0x00000004 shl ADC_CALFACT_CALFACT_S_Pos) ## !< 0x00000004
  ADC_CALFACT_CALFACT_S_3* = (0x00000008 shl ADC_CALFACT_CALFACT_S_Pos) ## !< 0x00000008
  ADC_CALFACT_CALFACT_S_4* = (0x00000010 shl ADC_CALFACT_CALFACT_S_Pos) ## !< 0x00000010
  ADC_CALFACT_CALFACT_S_5* = (0x00000020 shl ADC_CALFACT_CALFACT_S_Pos) ## !< 0x00000020
  ADC_CALFACT_CALFACT_S_6* = (0x00000040 shl ADC_CALFACT_CALFACT_S_Pos) ## !< 0x00000040
  ADC_CALFACT_CALFACT_D_Pos* = (16)
  ADC_CALFACT_CALFACT_D_Msk* = (0x0000007F shl ADC_CALFACT_CALFACT_D_Pos) ## !< 0x007F0000
  ADC_CALFACT_CALFACT_D* = ADC_CALFACT_CALFACT_D_Msk
  ADC_CALFACT_CALFACT_D_0* = (0x00000001 shl ADC_CALFACT_CALFACT_D_Pos) ## !< 0x00010000
  ADC_CALFACT_CALFACT_D_1* = (0x00000002 shl ADC_CALFACT_CALFACT_D_Pos) ## !< 0x00020000
  ADC_CALFACT_CALFACT_D_2* = (0x00000004 shl ADC_CALFACT_CALFACT_D_Pos) ## !< 0x00040000
  ADC_CALFACT_CALFACT_D_3* = (0x00000008 shl ADC_CALFACT_CALFACT_D_Pos) ## !< 0x00080000
  ADC_CALFACT_CALFACT_D_4* = (0x00000010 shl ADC_CALFACT_CALFACT_D_Pos) ## !< 0x00100000
  ADC_CALFACT_CALFACT_D_5* = (0x00000020 shl ADC_CALFACT_CALFACT_D_Pos) ## !< 0x00200000
  ADC_CALFACT_CALFACT_D_6* = (0x00000040 shl ADC_CALFACT_CALFACT_D_Pos) ## !< 0x00400000

## ************************  ADC Common registers  ****************************
## **************  Bit definition for ADC12_COMMON_CSR register  **************

const
  ADC12_CSR_ADRDY_MST_Pos* = (0)
  ADC12_CSR_ADRDY_MST_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_MST_Pos) ## !< 0x00000001
  ADC12_CSR_ADRDY_MST* = ADC12_CSR_ADRDY_MST_Msk
  ADC12_CSR_ADRDY_EOSMP_MST_Pos* = (1)
  ADC12_CSR_ADRDY_EOSMP_MST_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_EOSMP_MST_Pos) ## !< 0x00000002
  ADC12_CSR_ADRDY_EOSMP_MST* = ADC12_CSR_ADRDY_EOSMP_MST_Msk
  ADC12_CSR_ADRDY_EOC_MST_Pos* = (2)
  ADC12_CSR_ADRDY_EOC_MST_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_EOC_MST_Pos) ## !< 0x00000004
  ADC12_CSR_ADRDY_EOC_MST* = ADC12_CSR_ADRDY_EOC_MST_Msk
  ADC12_CSR_ADRDY_EOS_MST_Pos* = (3)
  ADC12_CSR_ADRDY_EOS_MST_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_EOS_MST_Pos) ## !< 0x00000008
  ADC12_CSR_ADRDY_EOS_MST* = ADC12_CSR_ADRDY_EOS_MST_Msk
  ADC12_CSR_ADRDY_OVR_MST_Pos* = (4)
  ADC12_CSR_ADRDY_OVR_MST_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_OVR_MST_Pos) ## !< 0x00000010
  ADC12_CSR_ADRDY_OVR_MST* = ADC12_CSR_ADRDY_OVR_MST_Msk
  ADC12_CSR_ADRDY_JEOC_MST_Pos* = (5)
  ADC12_CSR_ADRDY_JEOC_MST_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_JEOC_MST_Pos) ## !< 0x00000020
  ADC12_CSR_ADRDY_JEOC_MST* = ADC12_CSR_ADRDY_JEOC_MST_Msk
  ADC12_CSR_ADRDY_JEOS_MST_Pos* = (6)
  ADC12_CSR_ADRDY_JEOS_MST_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_JEOS_MST_Pos) ## !< 0x00000040
  ADC12_CSR_ADRDY_JEOS_MST* = ADC12_CSR_ADRDY_JEOS_MST_Msk
  ADC12_CSR_AWD1_MST_Pos* = (7)
  ADC12_CSR_AWD1_MST_Msk* = (0x00000001 shl ADC12_CSR_AWD1_MST_Pos) ## !< 0x00000080
  ADC12_CSR_AWD1_MST* = ADC12_CSR_AWD1_MST_Msk
  ADC12_CSR_AWD2_MST_Pos* = (8)
  ADC12_CSR_AWD2_MST_Msk* = (0x00000001 shl ADC12_CSR_AWD2_MST_Pos) ## !< 0x00000100
  ADC12_CSR_AWD2_MST* = ADC12_CSR_AWD2_MST_Msk
  ADC12_CSR_AWD3_MST_Pos* = (9)
  ADC12_CSR_AWD3_MST_Msk* = (0x00000001 shl ADC12_CSR_AWD3_MST_Pos) ## !< 0x00000200
  ADC12_CSR_AWD3_MST* = ADC12_CSR_AWD3_MST_Msk
  ADC12_CSR_JQOVF_MST_Pos* = (10)
  ADC12_CSR_JQOVF_MST_Msk* = (0x00000001 shl ADC12_CSR_JQOVF_MST_Pos) ## !< 0x00000400
  ADC12_CSR_JQOVF_MST* = ADC12_CSR_JQOVF_MST_Msk
  ADC12_CSR_ADRDY_SLV_Pos* = (16)
  ADC12_CSR_ADRDY_SLV_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_SLV_Pos) ## !< 0x00010000
  ADC12_CSR_ADRDY_SLV* = ADC12_CSR_ADRDY_SLV_Msk
  ADC12_CSR_ADRDY_EOSMP_SLV_Pos* = (17)
  ADC12_CSR_ADRDY_EOSMP_SLV_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_EOSMP_SLV_Pos) ## !< 0x00020000
  ADC12_CSR_ADRDY_EOSMP_SLV* = ADC12_CSR_ADRDY_EOSMP_SLV_Msk
  ADC12_CSR_ADRDY_EOC_SLV_Pos* = (18)
  ADC12_CSR_ADRDY_EOC_SLV_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_EOC_SLV_Pos) ## !< 0x00040000
  ADC12_CSR_ADRDY_EOC_SLV* = ADC12_CSR_ADRDY_EOC_SLV_Msk
  ADC12_CSR_ADRDY_EOS_SLV_Pos* = (19)
  ADC12_CSR_ADRDY_EOS_SLV_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_EOS_SLV_Pos) ## !< 0x00080000
  ADC12_CSR_ADRDY_EOS_SLV* = ADC12_CSR_ADRDY_EOS_SLV_Msk
  ADC12_CSR_ADRDY_OVR_SLV_Pos* = (20)
  ADC12_CSR_ADRDY_OVR_SLV_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_OVR_SLV_Pos) ## !< 0x00100000
  ADC12_CSR_ADRDY_OVR_SLV* = ADC12_CSR_ADRDY_OVR_SLV_Msk
  ADC12_CSR_ADRDY_JEOC_SLV_Pos* = (21)
  ADC12_CSR_ADRDY_JEOC_SLV_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_JEOC_SLV_Pos) ## !< 0x00200000
  ADC12_CSR_ADRDY_JEOC_SLV* = ADC12_CSR_ADRDY_JEOC_SLV_Msk
  ADC12_CSR_ADRDY_JEOS_SLV_Pos* = (22)
  ADC12_CSR_ADRDY_JEOS_SLV_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_JEOS_SLV_Pos) ## !< 0x00400000
  ADC12_CSR_ADRDY_JEOS_SLV* = ADC12_CSR_ADRDY_JEOS_SLV_Msk
  ADC12_CSR_AWD1_SLV_Pos* = (23)
  ADC12_CSR_AWD1_SLV_Msk* = (0x00000001 shl ADC12_CSR_AWD1_SLV_Pos) ## !< 0x00800000
  ADC12_CSR_AWD1_SLV* = ADC12_CSR_AWD1_SLV_Msk
  ADC12_CSR_AWD2_SLV_Pos* = (24)
  ADC12_CSR_AWD2_SLV_Msk* = (0x00000001 shl ADC12_CSR_AWD2_SLV_Pos) ## !< 0x01000000
  ADC12_CSR_AWD2_SLV* = ADC12_CSR_AWD2_SLV_Msk
  ADC12_CSR_AWD3_SLV_Pos* = (25)
  ADC12_CSR_AWD3_SLV_Msk* = (0x00000001 shl ADC12_CSR_AWD3_SLV_Pos) ## !< 0x02000000
  ADC12_CSR_AWD3_SLV* = ADC12_CSR_AWD3_SLV_Msk
  ADC12_CSR_JQOVF_SLV_Pos* = (26)
  ADC12_CSR_JQOVF_SLV_Msk* = (0x00000001 shl ADC12_CSR_JQOVF_SLV_Pos) ## !< 0x04000000
  ADC12_CSR_JQOVF_SLV* = ADC12_CSR_JQOVF_SLV_Msk

## **************  Bit definition for ADC34_COMMON_CSR register  **************

const
  ADC34_CSR_ADRDY_MST_Pos* = (0)
  ADC34_CSR_ADRDY_MST_Msk* = (0x00000001 shl ADC34_CSR_ADRDY_MST_Pos) ## !< 0x00000001
  ADC34_CSR_ADRDY_MST* = ADC34_CSR_ADRDY_MST_Msk
  ADC34_CSR_ADRDY_EOSMP_MST_Pos* = (1)
  ADC34_CSR_ADRDY_EOSMP_MST_Msk* = (0x00000001 shl ADC34_CSR_ADRDY_EOSMP_MST_Pos) ## !< 0x00000002
  ADC34_CSR_ADRDY_EOSMP_MST* = ADC34_CSR_ADRDY_EOSMP_MST_Msk
  ADC34_CSR_ADRDY_EOC_MST_Pos* = (2)
  ADC34_CSR_ADRDY_EOC_MST_Msk* = (0x00000001 shl ADC34_CSR_ADRDY_EOC_MST_Pos) ## !< 0x00000004
  ADC34_CSR_ADRDY_EOC_MST* = ADC34_CSR_ADRDY_EOC_MST_Msk
  ADC34_CSR_ADRDY_EOS_MST_Pos* = (3)
  ADC34_CSR_ADRDY_EOS_MST_Msk* = (0x00000001 shl ADC34_CSR_ADRDY_EOS_MST_Pos) ## !< 0x00000008
  ADC34_CSR_ADRDY_EOS_MST* = ADC34_CSR_ADRDY_EOS_MST_Msk
  ADC34_CSR_ADRDY_OVR_MST_Pos* = (4)
  ADC34_CSR_ADRDY_OVR_MST_Msk* = (0x00000001 shl ADC34_CSR_ADRDY_OVR_MST_Pos) ## !< 0x00000010
  ADC34_CSR_ADRDY_OVR_MST* = ADC34_CSR_ADRDY_OVR_MST_Msk
  ADC34_CSR_ADRDY_JEOC_MST_Pos* = (5)
  ADC34_CSR_ADRDY_JEOC_MST_Msk* = (0x00000001 shl ADC34_CSR_ADRDY_JEOC_MST_Pos) ## !< 0x00000020
  ADC34_CSR_ADRDY_JEOC_MST* = ADC34_CSR_ADRDY_JEOC_MST_Msk
  ADC34_CSR_ADRDY_JEOS_MST_Pos* = (6)
  ADC34_CSR_ADRDY_JEOS_MST_Msk* = (0x00000001 shl ADC34_CSR_ADRDY_JEOS_MST_Pos) ## !< 0x00000040
  ADC34_CSR_ADRDY_JEOS_MST* = ADC34_CSR_ADRDY_JEOS_MST_Msk
  ADC34_CSR_AWD1_MST_Pos* = (7)
  ADC34_CSR_AWD1_MST_Msk* = (0x00000001 shl ADC34_CSR_AWD1_MST_Pos) ## !< 0x00000080
  ADC34_CSR_AWD1_MST* = ADC34_CSR_AWD1_MST_Msk
  ADC34_CSR_AWD2_MST_Pos* = (8)
  ADC34_CSR_AWD2_MST_Msk* = (0x00000001 shl ADC34_CSR_AWD2_MST_Pos) ## !< 0x00000100
  ADC34_CSR_AWD2_MST* = ADC34_CSR_AWD2_MST_Msk
  ADC34_CSR_AWD3_MST_Pos* = (9)
  ADC34_CSR_AWD3_MST_Msk* = (0x00000001 shl ADC34_CSR_AWD3_MST_Pos) ## !< 0x00000200
  ADC34_CSR_AWD3_MST* = ADC34_CSR_AWD3_MST_Msk
  ADC34_CSR_JQOVF_MST_Pos* = (10)
  ADC34_CSR_JQOVF_MST_Msk* = (0x00000001 shl ADC34_CSR_JQOVF_MST_Pos) ## !< 0x00000400
  ADC34_CSR_JQOVF_MST* = ADC34_CSR_JQOVF_MST_Msk
  ADC34_CSR_ADRDY_SLV_Pos* = (16)
  ADC34_CSR_ADRDY_SLV_Msk* = (0x00000001 shl ADC34_CSR_ADRDY_SLV_Pos) ## !< 0x00010000
  ADC34_CSR_ADRDY_SLV* = ADC34_CSR_ADRDY_SLV_Msk
  ADC34_CSR_ADRDY_EOSMP_SLV_Pos* = (17)
  ADC34_CSR_ADRDY_EOSMP_SLV_Msk* = (0x00000001 shl ADC34_CSR_ADRDY_EOSMP_SLV_Pos) ## !< 0x00020000
  ADC34_CSR_ADRDY_EOSMP_SLV* = ADC34_CSR_ADRDY_EOSMP_SLV_Msk
  ADC34_CSR_ADRDY_EOC_SLV_Pos* = (18)
  ADC34_CSR_ADRDY_EOC_SLV_Msk* = (0x00000001 shl ADC34_CSR_ADRDY_EOC_SLV_Pos) ## !< 0x00040000
  ADC34_CSR_ADRDY_EOC_SLV* = ADC34_CSR_ADRDY_EOC_SLV_Msk
  ADC34_CSR_ADRDY_EOS_SLV_Pos* = (19)
  ADC34_CSR_ADRDY_EOS_SLV_Msk* = (0x00000001 shl ADC34_CSR_ADRDY_EOS_SLV_Pos) ## !< 0x00080000
  ADC34_CSR_ADRDY_EOS_SLV* = ADC34_CSR_ADRDY_EOS_SLV_Msk
#ADC12_CSR_ADRDY_OVR_SLV_Pos* = (20)
#ADC12_CSR_ADRDY_OVR_SLV_Msk* = (0x00000001 shl ADC12_CSR_ADRDY_OVR_SLV_Pos) ## !< 0x00100000
#ADC12_CSR_ADRDY_OVR_SLV* = ADC12_CSR_ADRDY_OVR_SLV_Msk
  ADC34_CSR_ADRDY_JEOC_SLV_Pos* = (21)
  ADC34_CSR_ADRDY_JEOC_SLV_Msk* = (0x00000001 shl ADC34_CSR_ADRDY_JEOC_SLV_Pos) ## !< 0x00200000
  ADC34_CSR_ADRDY_JEOC_SLV* = ADC34_CSR_ADRDY_JEOC_SLV_Msk
  ADC34_CSR_ADRDY_JEOS_SLV_Pos* = (22)
  ADC34_CSR_ADRDY_JEOS_SLV_Msk* = (0x00000001 shl ADC34_CSR_ADRDY_JEOS_SLV_Pos) ## !< 0x00400000
  ADC34_CSR_ADRDY_JEOS_SLV* = ADC34_CSR_ADRDY_JEOS_SLV_Msk
  ADC34_CSR_AWD1_SLV_Pos* = (23)
  ADC34_CSR_AWD1_SLV_Msk* = (0x00000001 shl ADC34_CSR_AWD1_SLV_Pos) ## !< 0x00800000
  ADC34_CSR_AWD1_SLV* = ADC34_CSR_AWD1_SLV_Msk
  ADC34_CSR_AWD2_SLV_Pos* = (24)
  ADC34_CSR_AWD2_SLV_Msk* = (0x00000001 shl ADC34_CSR_AWD2_SLV_Pos) ## !< 0x01000000
  ADC34_CSR_AWD2_SLV* = ADC34_CSR_AWD2_SLV_Msk
  ADC34_CSR_AWD3_SLV_Pos* = (25)
  ADC34_CSR_AWD3_SLV_Msk* = (0x00000001 shl ADC34_CSR_AWD3_SLV_Pos) ## !< 0x02000000
  ADC34_CSR_AWD3_SLV* = ADC34_CSR_AWD3_SLV_Msk
  ADC34_CSR_JQOVF_SLV_Pos* = (26)
  ADC34_CSR_JQOVF_SLV_Msk* = (0x00000001 shl ADC34_CSR_JQOVF_SLV_Pos) ## !< 0x04000000
  ADC34_CSR_JQOVF_SLV* = ADC34_CSR_JQOVF_SLV_Msk

## **************  Bit definition for ADC12_COMMON_CCR register  **************

const
  ADC12_CCR_MULTI_Pos* = (0)
  ADC12_CCR_MULTI_Msk* = (0x0000001F shl ADC12_CCR_MULTI_Pos) ## !< 0x0000001F
  ADC12_CCR_MULTI* = ADC12_CCR_MULTI_Msk
  ADC12_CCR_MULTI_0* = (0x00000001 shl ADC12_CCR_MULTI_Pos) ## !< 0x00000001
  ADC12_CCR_MULTI_1* = (0x00000002 shl ADC12_CCR_MULTI_Pos) ## !< 0x00000002
  ADC12_CCR_MULTI_2* = (0x00000004 shl ADC12_CCR_MULTI_Pos) ## !< 0x00000004
  ADC12_CCR_MULTI_3* = (0x00000008 shl ADC12_CCR_MULTI_Pos) ## !< 0x00000008
  ADC12_CCR_MULTI_4* = (0x00000010 shl ADC12_CCR_MULTI_Pos) ## !< 0x00000010
  ADC12_CCR_DELAY_Pos* = (8)
  ADC12_CCR_DELAY_Msk* = (0x0000000F shl ADC12_CCR_DELAY_Pos) ## !< 0x00000F00
  ADC12_CCR_DELAY* = ADC12_CCR_DELAY_Msk
  ADC12_CCR_DELAY_0* = (0x00000001 shl ADC12_CCR_DELAY_Pos) ## !< 0x00000100
  ADC12_CCR_DELAY_1* = (0x00000002 shl ADC12_CCR_DELAY_Pos) ## !< 0x00000200
  ADC12_CCR_DELAY_2* = (0x00000004 shl ADC12_CCR_DELAY_Pos) ## !< 0x00000400
  ADC12_CCR_DELAY_3* = (0x00000008 shl ADC12_CCR_DELAY_Pos) ## !< 0x00000800
  ADC12_CCR_DMACFG_Pos* = (13)
  ADC12_CCR_DMACFG_Msk* = (0x00000001 shl ADC12_CCR_DMACFG_Pos) ## !< 0x00002000
  ADC12_CCR_DMACFG* = ADC12_CCR_DMACFG_Msk
  ADC12_CCR_MDMA_Pos* = (14)
  ADC12_CCR_MDMA_Msk* = (0x00000003 shl ADC12_CCR_MDMA_Pos) ## !< 0x0000C000
  ADC12_CCR_MDMA* = ADC12_CCR_MDMA_Msk
  ADC12_CCR_MDMA_0* = (0x00000001 shl ADC12_CCR_MDMA_Pos) ## !< 0x00004000
  ADC12_CCR_MDMA_1* = (0x00000002 shl ADC12_CCR_MDMA_Pos) ## !< 0x00008000
  ADC12_CCR_CKMODE_Pos* = (16)
  ADC12_CCR_CKMODE_Msk* = (0x00000003 shl ADC12_CCR_CKMODE_Pos) ## !< 0x00030000
  ADC12_CCR_CKMODE* = ADC12_CCR_CKMODE_Msk
  ADC12_CCR_CKMODE_0* = (0x00000001 shl ADC12_CCR_CKMODE_Pos) ## !< 0x00010000
  ADC12_CCR_CKMODE_1* = (0x00000002 shl ADC12_CCR_CKMODE_Pos) ## !< 0x00020000
  ADC12_CCR_VREFEN_Pos* = (22)
  ADC12_CCR_VREFEN_Msk* = (0x00000001 shl ADC12_CCR_VREFEN_Pos) ## !< 0x00400000
  ADC12_CCR_VREFEN* = ADC12_CCR_VREFEN_Msk
  ADC12_CCR_TSEN_Pos* = (23)
  ADC12_CCR_TSEN_Msk* = (0x00000001 shl ADC12_CCR_TSEN_Pos) ## !< 0x00800000
  ADC12_CCR_TSEN* = ADC12_CCR_TSEN_Msk
  ADC12_CCR_VBATEN_Pos* = (24)
  ADC12_CCR_VBATEN_Msk* = (0x00000001 shl ADC12_CCR_VBATEN_Pos) ## !< 0x01000000
  ADC12_CCR_VBATEN* = ADC12_CCR_VBATEN_Msk

## **************  Bit definition for ADC34_COMMON_CCR register  **************

const
  ADC34_CCR_MULTI_Pos* = (0)
  ADC34_CCR_MULTI_Msk* = (0x0000001F shl ADC34_CCR_MULTI_Pos) ## !< 0x0000001F
  ADC34_CCR_MULTI* = ADC34_CCR_MULTI_Msk
  ADC34_CCR_MULTI_0* = (0x00000001 shl ADC34_CCR_MULTI_Pos) ## !< 0x00000001
  ADC34_CCR_MULTI_1* = (0x00000002 shl ADC34_CCR_MULTI_Pos) ## !< 0x00000002
  ADC34_CCR_MULTI_2* = (0x00000004 shl ADC34_CCR_MULTI_Pos) ## !< 0x00000004
  ADC34_CCR_MULTI_3* = (0x00000008 shl ADC34_CCR_MULTI_Pos) ## !< 0x00000008
  ADC34_CCR_MULTI_4* = (0x00000010 shl ADC34_CCR_MULTI_Pos) ## !< 0x00000010
  ADC34_CCR_DELAY_Pos* = (8)
  ADC34_CCR_DELAY_Msk* = (0x0000000F shl ADC34_CCR_DELAY_Pos) ## !< 0x00000F00
  ADC34_CCR_DELAY* = ADC34_CCR_DELAY_Msk
  ADC34_CCR_DELAY_0* = (0x00000001 shl ADC34_CCR_DELAY_Pos) ## !< 0x00000100
  ADC34_CCR_DELAY_1* = (0x00000002 shl ADC34_CCR_DELAY_Pos) ## !< 0x00000200
  ADC34_CCR_DELAY_2* = (0x00000004 shl ADC34_CCR_DELAY_Pos) ## !< 0x00000400
  ADC34_CCR_DELAY_3* = (0x00000008 shl ADC34_CCR_DELAY_Pos) ## !< 0x00000800
  ADC34_CCR_DMACFG_Pos* = (13)
  ADC34_CCR_DMACFG_Msk* = (0x00000001 shl ADC34_CCR_DMACFG_Pos) ## !< 0x00002000
  ADC34_CCR_DMACFG* = ADC34_CCR_DMACFG_Msk
  ADC34_CCR_MDMA_Pos* = (14)
  ADC34_CCR_MDMA_Msk* = (0x00000003 shl ADC34_CCR_MDMA_Pos) ## !< 0x0000C000
  ADC34_CCR_MDMA* = ADC34_CCR_MDMA_Msk
  ADC34_CCR_MDMA_0* = (0x00000001 shl ADC34_CCR_MDMA_Pos) ## !< 0x00004000
  ADC34_CCR_MDMA_1* = (0x00000002 shl ADC34_CCR_MDMA_Pos) ## !< 0x00008000
  ADC34_CCR_CKMODE_Pos* = (16)
  ADC34_CCR_CKMODE_Msk* = (0x00000003 shl ADC34_CCR_CKMODE_Pos) ## !< 0x00030000
  ADC34_CCR_CKMODE* = ADC34_CCR_CKMODE_Msk
  ADC34_CCR_CKMODE_0* = (0x00000001 shl ADC34_CCR_CKMODE_Pos) ## !< 0x00010000
  ADC34_CCR_CKMODE_1* = (0x00000002 shl ADC34_CCR_CKMODE_Pos) ## !< 0x00020000
  ADC34_CCR_VREFEN_Pos* = (22)
  ADC34_CCR_VREFEN_Msk* = (0x00000001 shl ADC34_CCR_VREFEN_Pos) ## !< 0x00400000
  ADC34_CCR_VREFEN* = ADC34_CCR_VREFEN_Msk
  ADC34_CCR_TSEN_Pos* = (23)
  ADC34_CCR_TSEN_Msk* = (0x00000001 shl ADC34_CCR_TSEN_Pos) ## !< 0x00800000
  ADC34_CCR_TSEN* = ADC34_CCR_TSEN_Msk
  ADC34_CCR_VBATEN_Pos* = (24)
  ADC34_CCR_VBATEN_Msk* = (0x00000001 shl ADC34_CCR_VBATEN_Pos) ## !< 0x01000000
  ADC34_CCR_VBATEN* = ADC34_CCR_VBATEN_Msk

## **************  Bit definition for ADC12_COMMON_CDR register  **************

const
  ADC12_CDR_RDATA_MST_Pos* = (0)
  ADC12_CDR_RDATA_MST_Msk* = (0x0000FFFF shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x0000FFFF
  ADC12_CDR_RDATA_MST* = ADC12_CDR_RDATA_MST_Msk
  ADC12_CDR_RDATA_MST_0* = (0x00000001 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00000001
  ADC12_CDR_RDATA_MST_1* = (0x00000002 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00000002
  ADC12_CDR_RDATA_MST_2* = (0x00000004 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00000004
  ADC12_CDR_RDATA_MST_3* = (0x00000008 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00000008
  ADC12_CDR_RDATA_MST_4* = (0x00000010 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00000010
  ADC12_CDR_RDATA_MST_5* = (0x00000020 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00000020
  ADC12_CDR_RDATA_MST_6* = (0x00000040 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00000040
  ADC12_CDR_RDATA_MST_7* = (0x00000080 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00000080
  ADC12_CDR_RDATA_MST_8* = (0x00000100 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00000100
  ADC12_CDR_RDATA_MST_9* = (0x00000200 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00000200
  ADC12_CDR_RDATA_MST_10* = (0x00000400 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00000400
  ADC12_CDR_RDATA_MST_11* = (0x00000800 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00000800
  ADC12_CDR_RDATA_MST_12* = (0x00001000 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00001000
  ADC12_CDR_RDATA_MST_13* = (0x00002000 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00002000
  ADC12_CDR_RDATA_MST_14* = (0x00004000 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00004000
  ADC12_CDR_RDATA_MST_15* = (0x00008000 shl ADC12_CDR_RDATA_MST_Pos) ## !< 0x00008000
  ADC12_CDR_RDATA_SLV_Pos* = (16)
  ADC12_CDR_RDATA_SLV_Msk* = (0x0000FFFF shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0xFFFF0000
  ADC12_CDR_RDATA_SLV* = ADC12_CDR_RDATA_SLV_Msk
  ADC12_CDR_RDATA_SLV_0* = (0x00000001 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x00010000
  ADC12_CDR_RDATA_SLV_1* = (0x00000002 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x00020000
  ADC12_CDR_RDATA_SLV_2* = (0x00000004 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x00040000
  ADC12_CDR_RDATA_SLV_3* = (0x00000008 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x00080000
  ADC12_CDR_RDATA_SLV_4* = (0x00000010 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x00100000
  ADC12_CDR_RDATA_SLV_5* = (0x00000020 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x00200000
  ADC12_CDR_RDATA_SLV_6* = (0x00000040 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x00400000
  ADC12_CDR_RDATA_SLV_7* = (0x00000080 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x00800000
  ADC12_CDR_RDATA_SLV_8* = (0x00000100 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x01000000
  ADC12_CDR_RDATA_SLV_9* = (0x00000200 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x02000000
  ADC12_CDR_RDATA_SLV_10* = (0x00000400 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x04000000
  ADC12_CDR_RDATA_SLV_11* = (0x00000800 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x08000000
  ADC12_CDR_RDATA_SLV_12* = (0x00001000 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x10000000
  ADC12_CDR_RDATA_SLV_13* = (0x00002000 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x20000000
  ADC12_CDR_RDATA_SLV_14* = (0x00004000 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x40000000
  ADC12_CDR_RDATA_SLV_15* = (0x00008000 shl ADC12_CDR_RDATA_SLV_Pos) ## !< 0x80000000

## **************  Bit definition for ADC34_COMMON_CDR register  **************

const
  ADC34_CDR_RDATA_MST_Pos* = (0)
  ADC34_CDR_RDATA_MST_Msk* = (0x0000FFFF shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x0000FFFF
  ADC34_CDR_RDATA_MST* = ADC34_CDR_RDATA_MST_Msk
  ADC34_CDR_RDATA_MST_0* = (0x00000001 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00000001
  ADC34_CDR_RDATA_MST_1* = (0x00000002 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00000002
  ADC34_CDR_RDATA_MST_2* = (0x00000004 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00000004
  ADC34_CDR_RDATA_MST_3* = (0x00000008 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00000008
  ADC34_CDR_RDATA_MST_4* = (0x00000010 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00000010
  ADC34_CDR_RDATA_MST_5* = (0x00000020 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00000020
  ADC34_CDR_RDATA_MST_6* = (0x00000040 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00000040
  ADC34_CDR_RDATA_MST_7* = (0x00000080 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00000080
  ADC34_CDR_RDATA_MST_8* = (0x00000100 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00000100
  ADC34_CDR_RDATA_MST_9* = (0x00000200 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00000200
  ADC34_CDR_RDATA_MST_10* = (0x00000400 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00000400
  ADC34_CDR_RDATA_MST_11* = (0x00000800 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00000800
  ADC34_CDR_RDATA_MST_12* = (0x00001000 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00001000
  ADC34_CDR_RDATA_MST_13* = (0x00002000 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00002000
  ADC34_CDR_RDATA_MST_14* = (0x00004000 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00004000
  ADC34_CDR_RDATA_MST_15* = (0x00008000 shl ADC34_CDR_RDATA_MST_Pos) ## !< 0x00008000
  ADC34_CDR_RDATA_SLV_Pos* = (16)
  ADC34_CDR_RDATA_SLV_Msk* = (0x0000FFFF shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0xFFFF0000
  ADC34_CDR_RDATA_SLV* = ADC34_CDR_RDATA_SLV_Msk
  ADC34_CDR_RDATA_SLV_0* = (0x00000001 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x00010000
  ADC34_CDR_RDATA_SLV_1* = (0x00000002 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x00020000
  ADC34_CDR_RDATA_SLV_2* = (0x00000004 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x00040000
  ADC34_CDR_RDATA_SLV_3* = (0x00000008 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x00080000
  ADC34_CDR_RDATA_SLV_4* = (0x00000010 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x00100000
  ADC34_CDR_RDATA_SLV_5* = (0x00000020 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x00200000
  ADC34_CDR_RDATA_SLV_6* = (0x00000040 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x00400000
  ADC34_CDR_RDATA_SLV_7* = (0x00000080 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x00800000
  ADC34_CDR_RDATA_SLV_8* = (0x00000100 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x01000000
  ADC34_CDR_RDATA_SLV_9* = (0x00000200 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x02000000
  ADC34_CDR_RDATA_SLV_10* = (0x00000400 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x04000000
  ADC34_CDR_RDATA_SLV_11* = (0x00000800 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x08000000
  ADC34_CDR_RDATA_SLV_12* = (0x00001000 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x10000000
  ADC34_CDR_RDATA_SLV_13* = (0x00002000 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x20000000
  ADC34_CDR_RDATA_SLV_14* = (0x00004000 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x40000000
  ADC34_CDR_RDATA_SLV_15* = (0x00008000 shl ADC34_CDR_RDATA_SLV_Pos) ## !< 0x80000000

## *******************  Bit definition for ADC_CSR register  ******************

const
  ADC_CSR_ADRDY_MST_Pos* = (0)
  ADC_CSR_ADRDY_MST_Msk* = (0x00000001 shl ADC_CSR_ADRDY_MST_Pos) ## !< 0x00000001
  ADC_CSR_ADRDY_MST* = ADC_CSR_ADRDY_MST_Msk
  ADC_CSR_EOSMP_MST_Pos* = (1)
  ADC_CSR_EOSMP_MST_Msk* = (0x00000001 shl ADC_CSR_EOSMP_MST_Pos) ## !< 0x00000002
  ADC_CSR_EOSMP_MST* = ADC_CSR_EOSMP_MST_Msk
  ADC_CSR_EOC_MST_Pos* = (2)
  ADC_CSR_EOC_MST_Msk* = (0x00000001 shl ADC_CSR_EOC_MST_Pos) ## !< 0x00000004
  ADC_CSR_EOC_MST* = ADC_CSR_EOC_MST_Msk
  ADC_CSR_EOS_MST_Pos* = (3)
  ADC_CSR_EOS_MST_Msk* = (0x00000001 shl ADC_CSR_EOS_MST_Pos) ## !< 0x00000008
  ADC_CSR_EOS_MST* = ADC_CSR_EOS_MST_Msk
  ADC_CSR_OVR_MST_Pos* = (4)
  ADC_CSR_OVR_MST_Msk* = (0x00000001 shl ADC_CSR_OVR_MST_Pos) ## !< 0x00000010
  ADC_CSR_OVR_MST* = ADC_CSR_OVR_MST_Msk
  ADC_CSR_JEOC_MST_Pos* = (5)
  ADC_CSR_JEOC_MST_Msk* = (0x00000001 shl ADC_CSR_JEOC_MST_Pos) ## !< 0x00000020
  ADC_CSR_JEOC_MST* = ADC_CSR_JEOC_MST_Msk
  ADC_CSR_JEOS_MST_Pos* = (6)
  ADC_CSR_JEOS_MST_Msk* = (0x00000001 shl ADC_CSR_JEOS_MST_Pos) ## !< 0x00000040
  ADC_CSR_JEOS_MST* = ADC_CSR_JEOS_MST_Msk
  ADC_CSR_AWD1_MST_Pos* = (7)
  ADC_CSR_AWD1_MST_Msk* = (0x00000001 shl ADC_CSR_AWD1_MST_Pos) ## !< 0x00000080
  ADC_CSR_AWD1_MST* = ADC_CSR_AWD1_MST_Msk
  ADC_CSR_AWD2_MST_Pos* = (8)
  ADC_CSR_AWD2_MST_Msk* = (0x00000001 shl ADC_CSR_AWD2_MST_Pos) ## !< 0x00000100
  ADC_CSR_AWD2_MST* = ADC_CSR_AWD2_MST_Msk
  ADC_CSR_AWD3_MST_Pos* = (9)
  ADC_CSR_AWD3_MST_Msk* = (0x00000001 shl ADC_CSR_AWD3_MST_Pos) ## !< 0x00000200
  ADC_CSR_AWD3_MST* = ADC_CSR_AWD3_MST_Msk
  ADC_CSR_JQOVF_MST_Pos* = (10)
  ADC_CSR_JQOVF_MST_Msk* = (0x00000001 shl ADC_CSR_JQOVF_MST_Pos) ## !< 0x00000400
  ADC_CSR_JQOVF_MST* = ADC_CSR_JQOVF_MST_Msk
  ADC_CSR_ADRDY_SLV_Pos* = (16)
  ADC_CSR_ADRDY_SLV_Msk* = (0x00000001 shl ADC_CSR_ADRDY_SLV_Pos) ## !< 0x00010000
  ADC_CSR_ADRDY_SLV* = ADC_CSR_ADRDY_SLV_Msk
  ADC_CSR_EOSMP_SLV_Pos* = (17)
  ADC_CSR_EOSMP_SLV_Msk* = (0x00000001 shl ADC_CSR_EOSMP_SLV_Pos) ## !< 0x00020000
  ADC_CSR_EOSMP_SLV* = ADC_CSR_EOSMP_SLV_Msk
  ADC_CSR_EOC_SLV_Pos* = (18)
  ADC_CSR_EOC_SLV_Msk* = (0x00000001 shl ADC_CSR_EOC_SLV_Pos) ## !< 0x00040000
  ADC_CSR_EOC_SLV* = ADC_CSR_EOC_SLV_Msk
  ADC_CSR_EOS_SLV_Pos* = (19)
  ADC_CSR_EOS_SLV_Msk* = (0x00000001 shl ADC_CSR_EOS_SLV_Pos) ## !< 0x00080000
  ADC_CSR_EOS_SLV* = ADC_CSR_EOS_SLV_Msk
  ADC_CSR_OVR_SLV_Pos* = (20)
  ADC_CSR_OVR_SLV_Msk* = (0x00000001 shl ADC_CSR_OVR_SLV_Pos) ## !< 0x00100000
  ADC_CSR_OVR_SLV* = ADC_CSR_OVR_SLV_Msk
  ADC_CSR_JEOC_SLV_Pos* = (21)
  ADC_CSR_JEOC_SLV_Msk* = (0x00000001 shl ADC_CSR_JEOC_SLV_Pos) ## !< 0x00200000
  ADC_CSR_JEOC_SLV* = ADC_CSR_JEOC_SLV_Msk
  ADC_CSR_JEOS_SLV_Pos* = (22)
  ADC_CSR_JEOS_SLV_Msk* = (0x00000001 shl ADC_CSR_JEOS_SLV_Pos) ## !< 0x00400000
  ADC_CSR_JEOS_SLV* = ADC_CSR_JEOS_SLV_Msk
  ADC_CSR_AWD1_SLV_Pos* = (23)
  ADC_CSR_AWD1_SLV_Msk* = (0x00000001 shl ADC_CSR_AWD1_SLV_Pos) ## !< 0x00800000
  ADC_CSR_AWD1_SLV* = ADC_CSR_AWD1_SLV_Msk
  ADC_CSR_AWD2_SLV_Pos* = (24)
  ADC_CSR_AWD2_SLV_Msk* = (0x00000001 shl ADC_CSR_AWD2_SLV_Pos) ## !< 0x01000000
  ADC_CSR_AWD2_SLV* = ADC_CSR_AWD2_SLV_Msk
  ADC_CSR_AWD3_SLV_Pos* = (25)
  ADC_CSR_AWD3_SLV_Msk* = (0x00000001 shl ADC_CSR_AWD3_SLV_Pos) ## !< 0x02000000
  ADC_CSR_AWD3_SLV* = ADC_CSR_AWD3_SLV_Msk
  ADC_CSR_JQOVF_SLV_Pos* = (26)
  ADC_CSR_JQOVF_SLV_Msk* = (0x00000001 shl ADC_CSR_JQOVF_SLV_Pos) ## !< 0x04000000
  ADC_CSR_JQOVF_SLV* = ADC_CSR_JQOVF_SLV_Msk

##  Legacy defines

const
  ADC_CSR_ADRDY_EOSMP_MST* = ADC_CSR_EOSMP_MST
  ADC_CSR_ADRDY_EOC_MST* = ADC_CSR_EOC_MST
  ADC_CSR_ADRDY_EOS_MST* = ADC_CSR_EOS_MST
  ADC_CSR_ADRDY_OVR_MST* = ADC_CSR_OVR_MST
  ADC_CSR_ADRDY_JEOC_MST* = ADC_CSR_JEOC_MST
  ADC_CSR_ADRDY_JEOS_MST* = ADC_CSR_JEOS_MST
  ADC_CSR_ADRDY_EOSMP_SLV* = ADC_CSR_EOSMP_SLV
  ADC_CSR_ADRDY_EOC_SLV* = ADC_CSR_EOC_SLV
  ADC_CSR_ADRDY_EOS_SLV* = ADC_CSR_EOS_SLV
  ADC_CSR_ADRDY_OVR_SLV* = ADC_CSR_OVR_SLV
  ADC_CSR_ADRDY_JEOC_SLV* = ADC_CSR_JEOC_SLV
  ADC_CSR_ADRDY_JEOS_SLV* = ADC_CSR_JEOS_SLV

## *******************  Bit definition for ADC_CCR register  ******************

const
  ADC_CCR_DUAL_Pos* = (0)
  ADC_CCR_DUAL_Msk* = (0x0000001F shl ADC_CCR_DUAL_Pos) ## !< 0x0000001F
  ADC_CCR_DUAL* = ADC_CCR_DUAL_Msk
  ADC_CCR_DUAL_0* = (0x00000001 shl ADC_CCR_DUAL_Pos) ## !< 0x00000001
  ADC_CCR_DUAL_1* = (0x00000002 shl ADC_CCR_DUAL_Pos) ## !< 0x00000002
  ADC_CCR_DUAL_2* = (0x00000004 shl ADC_CCR_DUAL_Pos) ## !< 0x00000004
  ADC_CCR_DUAL_3* = (0x00000008 shl ADC_CCR_DUAL_Pos) ## !< 0x00000008
  ADC_CCR_DUAL_4* = (0x00000010 shl ADC_CCR_DUAL_Pos) ## !< 0x00000010
  ADC_CCR_DELAY_Pos* = (8)
  ADC_CCR_DELAY_Msk* = (0x0000000F shl ADC_CCR_DELAY_Pos) ## !< 0x00000F00
  ADC_CCR_DELAY* = ADC_CCR_DELAY_Msk
  ADC_CCR_DELAY_0* = (0x00000001 shl ADC_CCR_DELAY_Pos) ## !< 0x00000100
  ADC_CCR_DELAY_1* = (0x00000002 shl ADC_CCR_DELAY_Pos) ## !< 0x00000200
  ADC_CCR_DELAY_2* = (0x00000004 shl ADC_CCR_DELAY_Pos) ## !< 0x00000400
  ADC_CCR_DELAY_3* = (0x00000008 shl ADC_CCR_DELAY_Pos) ## !< 0x00000800
  ADC_CCR_DMACFG_Pos* = (13)
  ADC_CCR_DMACFG_Msk* = (0x00000001 shl ADC_CCR_DMACFG_Pos) ## !< 0x00002000
  ADC_CCR_DMACFG* = ADC_CCR_DMACFG_Msk
  ADC_CCR_MDMA_Pos* = (14)
  ADC_CCR_MDMA_Msk* = (0x00000003 shl ADC_CCR_MDMA_Pos) ## !< 0x0000C000
  ADC_CCR_MDMA* = ADC_CCR_MDMA_Msk
  ADC_CCR_MDMA_0* = (0x00000001 shl ADC_CCR_MDMA_Pos) ## !< 0x00004000
  ADC_CCR_MDMA_1* = (0x00000002 shl ADC_CCR_MDMA_Pos) ## !< 0x00008000
  ADC_CCR_CKMODE_Pos* = (16)
  ADC_CCR_CKMODE_Msk* = (0x00000003 shl ADC_CCR_CKMODE_Pos) ## !< 0x00030000
  ADC_CCR_CKMODE* = ADC_CCR_CKMODE_Msk
  ADC_CCR_CKMODE_0* = (0x00000001 shl ADC_CCR_CKMODE_Pos) ## !< 0x00010000
  ADC_CCR_CKMODE_1* = (0x00000002 shl ADC_CCR_CKMODE_Pos) ## !< 0x00020000
  ADC_CCR_VREFEN_Pos* = (22)
  ADC_CCR_VREFEN_Msk* = (0x00000001 shl ADC_CCR_VREFEN_Pos) ## !< 0x00400000
  ADC_CCR_VREFEN* = ADC_CCR_VREFEN_Msk
  ADC_CCR_TSEN_Pos* = (23)
  ADC_CCR_TSEN_Msk* = (0x00000001 shl ADC_CCR_TSEN_Pos) ## !< 0x00800000
  ADC_CCR_TSEN* = ADC_CCR_TSEN_Msk
  ADC_CCR_VBATEN_Pos* = (24)
  ADC_CCR_VBATEN_Msk* = (0x00000001 shl ADC_CCR_VBATEN_Pos) ## !< 0x01000000
  ADC_CCR_VBATEN* = ADC_CCR_VBATEN_Msk

##  Legacy defines

const
  ADC_CCR_MULTI* = (ADC_CCR_DUAL)
  ADC_CCR_MULTI_0* = (ADC_CCR_DUAL_0)
  ADC_CCR_MULTI_1* = (ADC_CCR_DUAL_1)
  ADC_CCR_MULTI_2* = (ADC_CCR_DUAL_2)
  ADC_CCR_MULTI_3* = (ADC_CCR_DUAL_3)
  ADC_CCR_MULTI_4* = (ADC_CCR_DUAL_4)

## *******************  Bit definition for ADC_CDR register  ******************

const
  ADC_CDR_RDATA_MST_Pos* = (0)
  ADC_CDR_RDATA_MST_Msk* = (0x0000FFFF shl ADC_CDR_RDATA_MST_Pos) ## !< 0x0000FFFF
  ADC_CDR_RDATA_MST* = ADC_CDR_RDATA_MST_Msk
  ADC_CDR_RDATA_MST_0* = (0x00000001 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00000001
  ADC_CDR_RDATA_MST_1* = (0x00000002 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00000002
  ADC_CDR_RDATA_MST_2* = (0x00000004 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00000004
  ADC_CDR_RDATA_MST_3* = (0x00000008 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00000008
  ADC_CDR_RDATA_MST_4* = (0x00000010 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00000010
  ADC_CDR_RDATA_MST_5* = (0x00000020 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00000020
  ADC_CDR_RDATA_MST_6* = (0x00000040 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00000040
  ADC_CDR_RDATA_MST_7* = (0x00000080 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00000080
  ADC_CDR_RDATA_MST_8* = (0x00000100 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00000100
  ADC_CDR_RDATA_MST_9* = (0x00000200 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00000200
  ADC_CDR_RDATA_MST_10* = (0x00000400 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00000400
  ADC_CDR_RDATA_MST_11* = (0x00000800 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00000800
  ADC_CDR_RDATA_MST_12* = (0x00001000 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00001000
  ADC_CDR_RDATA_MST_13* = (0x00002000 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00002000
  ADC_CDR_RDATA_MST_14* = (0x00004000 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00004000
  ADC_CDR_RDATA_MST_15* = (0x00008000 shl ADC_CDR_RDATA_MST_Pos) ## !< 0x00008000
  ADC_CDR_RDATA_SLV_Pos* = (16)
  ADC_CDR_RDATA_SLV_Msk* = (0x0000FFFF shl ADC_CDR_RDATA_SLV_Pos) ## !< 0xFFFF0000
  ADC_CDR_RDATA_SLV* = ADC_CDR_RDATA_SLV_Msk
  ADC_CDR_RDATA_SLV_0* = (0x00000001 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x00010000
  ADC_CDR_RDATA_SLV_1* = (0x00000002 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x00020000
  ADC_CDR_RDATA_SLV_2* = (0x00000004 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x00040000
  ADC_CDR_RDATA_SLV_3* = (0x00000008 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x00080000
  ADC_CDR_RDATA_SLV_4* = (0x00000010 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x00100000
  ADC_CDR_RDATA_SLV_5* = (0x00000020 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x00200000
  ADC_CDR_RDATA_SLV_6* = (0x00000040 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x00400000
  ADC_CDR_RDATA_SLV_7* = (0x00000080 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x00800000
  ADC_CDR_RDATA_SLV_8* = (0x00000100 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x01000000
  ADC_CDR_RDATA_SLV_9* = (0x00000200 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x02000000
  ADC_CDR_RDATA_SLV_10* = (0x00000400 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x04000000
  ADC_CDR_RDATA_SLV_11* = (0x00000800 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x08000000
  ADC_CDR_RDATA_SLV_12* = (0x00001000 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x10000000
  ADC_CDR_RDATA_SLV_13* = (0x00002000 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x20000000
  ADC_CDR_RDATA_SLV_14* = (0x00004000 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x40000000
  ADC_CDR_RDATA_SLV_15* = (0x00008000 shl ADC_CDR_RDATA_SLV_Pos) ## !< 0x80000000

## ****************************************************************************
##
##                       Analog Comparators (COMP)
##
## ****************************************************************************

const
  COMP_V1_3_0_0* = true         ## !< Comparator IP version

## *********************  Bit definition for COMP1_CSR register  **************

const
  COMP1_CSR_COMP1EN_Pos* = (0)
  COMP1_CSR_COMP1EN_Msk* = (0x00000001 shl COMP1_CSR_COMP1EN_Pos) ## !< 0x00000001
  COMP1_CSR_COMP1EN* = COMP1_CSR_COMP1EN_Msk
  COMP1_CSR_COMP1SW1_Pos* = (1)
  COMP1_CSR_COMP1SW1_Msk* = (0x00000001 shl COMP1_CSR_COMP1SW1_Pos) ## !< 0x00000002
  COMP1_CSR_COMP1SW1* = COMP1_CSR_COMP1SW1_Msk

##  Legacy defines

const
  COMP_CSR_COMP1SW1* = COMP1_CSR_COMP1SW1
  COMP1_CSR_COMP1MODE_Pos* = (2)
  COMP1_CSR_COMP1MODE_Msk* = (0x00000003 shl COMP1_CSR_COMP1MODE_Pos) ## !< 0x0000000C
  COMP1_CSR_COMP1MODE* = COMP1_CSR_COMP1MODE_Msk
  COMP1_CSR_COMP1MODE_0* = (0x00000001 shl COMP1_CSR_COMP1MODE_Pos) ## !< 0x00000004
  COMP1_CSR_COMP1MODE_1* = (0x00000002 shl COMP1_CSR_COMP1MODE_Pos) ## !< 0x00000008
  COMP1_CSR_COMP1INSEL_Pos* = (4)
  COMP1_CSR_COMP1INSEL_Msk* = (0x00000007 shl COMP1_CSR_COMP1INSEL_Pos) ## !< 0x00000070
  COMP1_CSR_COMP1INSEL* = COMP1_CSR_COMP1INSEL_Msk
  COMP1_CSR_COMP1INSEL_0* = (0x00000001 shl COMP1_CSR_COMP1INSEL_Pos) ## !< 0x00000010
  COMP1_CSR_COMP1INSEL_1* = (0x00000002 shl COMP1_CSR_COMP1INSEL_Pos) ## !< 0x00000020
  COMP1_CSR_COMP1INSEL_2* = (0x00000004 shl COMP1_CSR_COMP1INSEL_Pos) ## !< 0x00000040
  COMP1_CSR_COMP1OUTSEL_Pos* = (10)
  COMP1_CSR_COMP1OUTSEL_Msk* = (0x0000000F shl COMP1_CSR_COMP1OUTSEL_Pos) ## !< 0x00003C00
  COMP1_CSR_COMP1OUTSEL* = COMP1_CSR_COMP1OUTSEL_Msk
  COMP1_CSR_COMP1OUTSEL_0* = (0x00000001 shl COMP1_CSR_COMP1OUTSEL_Pos) ## !< 0x00000400
  COMP1_CSR_COMP1OUTSEL_1* = (0x00000002 shl COMP1_CSR_COMP1OUTSEL_Pos) ## !< 0x00000800
  COMP1_CSR_COMP1OUTSEL_2* = (0x00000004 shl COMP1_CSR_COMP1OUTSEL_Pos) ## !< 0x00001000
  COMP1_CSR_COMP1OUTSEL_3* = (0x00000008 shl COMP1_CSR_COMP1OUTSEL_Pos) ## !< 0x00002000
  COMP1_CSR_COMP1POL_Pos* = (15)
  COMP1_CSR_COMP1POL_Msk* = (0x00000001 shl COMP1_CSR_COMP1POL_Pos) ## !< 0x00008000
  COMP1_CSR_COMP1POL* = COMP1_CSR_COMP1POL_Msk
  COMP1_CSR_COMP1HYST_Pos* = (16)
  COMP1_CSR_COMP1HYST_Msk* = (0x00000003 shl COMP1_CSR_COMP1HYST_Pos) ## !< 0x00030000
  COMP1_CSR_COMP1HYST* = COMP1_CSR_COMP1HYST_Msk
  COMP1_CSR_COMP1HYST_0* = (0x00000001 shl COMP1_CSR_COMP1HYST_Pos) ## !< 0x00010000
  COMP1_CSR_COMP1HYST_1* = (0x00000002 shl COMP1_CSR_COMP1HYST_Pos) ## !< 0x00020000
  COMP1_CSR_COMP1BLANKING_Pos* = (18)
  COMP1_CSR_COMP1BLANKING_Msk* = (0x00000003 shl COMP1_CSR_COMP1BLANKING_Pos) ## !< 0x000C0000
  COMP1_CSR_COMP1BLANKING* = COMP1_CSR_COMP1BLANKING_Msk
  COMP1_CSR_COMP1BLANKING_0* = (0x00000001 shl COMP1_CSR_COMP1BLANKING_Pos) ## !< 0x00040000
  COMP1_CSR_COMP1BLANKING_1* = (0x00000002 shl COMP1_CSR_COMP1BLANKING_Pos) ## !< 0x00080000
  COMP1_CSR_COMP1BLANKING_2* = (0x00000004 shl COMP1_CSR_COMP1BLANKING_Pos) ## !< 0x00100000
  COMP1_CSR_COMP1OUT_Pos* = (30)
  COMP1_CSR_COMP1OUT_Msk* = (0x00000001 shl COMP1_CSR_COMP1OUT_Pos) ## !< 0x40000000
  COMP1_CSR_COMP1OUT* = COMP1_CSR_COMP1OUT_Msk
  COMP1_CSR_COMP1LOCK_Pos* = (31)
  COMP1_CSR_COMP1LOCK_Msk* = (0x00000001 shl COMP1_CSR_COMP1LOCK_Pos) ## !< 0x80000000
  COMP1_CSR_COMP1LOCK* = COMP1_CSR_COMP1LOCK_Msk

## *********************  Bit definition for COMP2_CSR register  **************

const
  COMP2_CSR_COMP2EN_Pos* = (0)
  COMP2_CSR_COMP2EN_Msk* = (0x00000001 shl COMP2_CSR_COMP2EN_Pos) ## !< 0x00000001
  COMP2_CSR_COMP2EN* = COMP2_CSR_COMP2EN_Msk
  COMP2_CSR_COMP2MODE_Pos* = (2)
  COMP2_CSR_COMP2MODE_Msk* = (0x00000003 shl COMP2_CSR_COMP2MODE_Pos) ## !< 0x0000000C
  COMP2_CSR_COMP2MODE* = COMP2_CSR_COMP2MODE_Msk
  COMP2_CSR_COMP2MODE_0* = (0x00000001 shl COMP2_CSR_COMP2MODE_Pos) ## !< 0x00000004
  COMP2_CSR_COMP2MODE_1* = (0x00000002 shl COMP2_CSR_COMP2MODE_Pos) ## !< 0x00000008
  COMP2_CSR_COMP2INSEL_Pos* = (4)
  COMP2_CSR_COMP2INSEL_Msk* = (0x00000007 shl COMP2_CSR_COMP2INSEL_Pos) ## !< 0x00000070
  COMP2_CSR_COMP2INSEL* = COMP2_CSR_COMP2INSEL_Msk
  COMP2_CSR_COMP2INSEL_0* = (0x00000010) ## !< COMP2 inverting input select bit 0
  COMP2_CSR_COMP2INSEL_1* = (0x00000020) ## !< COMP2 inverting input select bit 1
  COMP2_CSR_COMP2INSEL_2* = (0x00000040) ## !< COMP2 inverting input select bit 2
  COMP2_CSR_COMP2NONINSEL_Pos* = (7)
  COMP2_CSR_COMP2NONINSEL_Msk* = (0x00000001 shl COMP2_CSR_COMP2NONINSEL_Pos) ## !< 0x00000080
  COMP2_CSR_COMP2NONINSEL* = COMP2_CSR_COMP2NONINSEL_Msk
  COMP2_CSR_COMP2WNDWEN_Pos* = (9)
  COMP2_CSR_COMP2WNDWEN_Msk* = (0x00000001 shl COMP2_CSR_COMP2WNDWEN_Pos) ## !< 0x00000200
  COMP2_CSR_COMP2WNDWEN* = COMP2_CSR_COMP2WNDWEN_Msk
  COMP2_CSR_COMP2OUTSEL_Pos* = (10)
  COMP2_CSR_COMP2OUTSEL_Msk* = (0x0000000F shl COMP2_CSR_COMP2OUTSEL_Pos) ## !< 0x00003C00
  COMP2_CSR_COMP2OUTSEL* = COMP2_CSR_COMP2OUTSEL_Msk
  COMP2_CSR_COMP2OUTSEL_0* = (0x00000001 shl COMP2_CSR_COMP2OUTSEL_Pos) ## !< 0x00000400
  COMP2_CSR_COMP2OUTSEL_1* = (0x00000002 shl COMP2_CSR_COMP2OUTSEL_Pos) ## !< 0x00000800
  COMP2_CSR_COMP2OUTSEL_2* = (0x00000004 shl COMP2_CSR_COMP2OUTSEL_Pos) ## !< 0x00001000
  COMP2_CSR_COMP2OUTSEL_3* = (0x00000008 shl COMP2_CSR_COMP2OUTSEL_Pos) ## !< 0x00002000
  COMP2_CSR_COMP2POL_Pos* = (15)
  COMP2_CSR_COMP2POL_Msk* = (0x00000001 shl COMP2_CSR_COMP2POL_Pos) ## !< 0x00008000
  COMP2_CSR_COMP2POL* = COMP2_CSR_COMP2POL_Msk
  COMP2_CSR_COMP2HYST_Pos* = (16)
  COMP2_CSR_COMP2HYST_Msk* = (0x00000003 shl COMP2_CSR_COMP2HYST_Pos) ## !< 0x00030000
  COMP2_CSR_COMP2HYST* = COMP2_CSR_COMP2HYST_Msk
  COMP2_CSR_COMP2HYST_0* = (0x00000001 shl COMP2_CSR_COMP2HYST_Pos) ## !< 0x00010000
  COMP2_CSR_COMP2HYST_1* = (0x00000002 shl COMP2_CSR_COMP2HYST_Pos) ## !< 0x00020000
  COMP2_CSR_COMP2BLANKING_Pos* = (18)
  COMP2_CSR_COMP2BLANKING_Msk* = (0x00000003 shl COMP2_CSR_COMP2BLANKING_Pos) ## !< 0x000C0000
  COMP2_CSR_COMP2BLANKING* = COMP2_CSR_COMP2BLANKING_Msk
  COMP2_CSR_COMP2BLANKING_0* = (0x00000001 shl COMP2_CSR_COMP2BLANKING_Pos) ## !< 0x00040000
  COMP2_CSR_COMP2BLANKING_1* = (0x00000002 shl COMP2_CSR_COMP2BLANKING_Pos) ## !< 0x00080000
  COMP2_CSR_COMP2BLANKING_2* = (0x00000004 shl COMP2_CSR_COMP2BLANKING_Pos) ## !< 0x00100000
  COMP2_CSR_COMP2OUT_Pos* = (30)
  COMP2_CSR_COMP2OUT_Msk* = (0x00000001 shl COMP2_CSR_COMP2OUT_Pos) ## !< 0x40000000
  COMP2_CSR_COMP2OUT* = COMP2_CSR_COMP2OUT_Msk
  COMP2_CSR_COMP2LOCK_Pos* = (31)
  COMP2_CSR_COMP2LOCK_Msk* = (0x00000001 shl COMP2_CSR_COMP2LOCK_Pos) ## !< 0x80000000
  COMP2_CSR_COMP2LOCK* = COMP2_CSR_COMP2LOCK_Msk

## *********************  Bit definition for COMP3_CSR register  **************

const
  COMP3_CSR_COMP3EN_Pos* = (0)
  COMP3_CSR_COMP3EN_Msk* = (0x00000001 shl COMP3_CSR_COMP3EN_Pos) ## !< 0x00000001
  COMP3_CSR_COMP3EN* = COMP3_CSR_COMP3EN_Msk
  COMP3_CSR_COMP3MODE_Pos* = (2)
  COMP3_CSR_COMP3MODE_Msk* = (0x00000003 shl COMP3_CSR_COMP3MODE_Pos) ## !< 0x0000000C
  COMP3_CSR_COMP3MODE* = COMP3_CSR_COMP3MODE_Msk
  COMP3_CSR_COMP3MODE_0* = (0x00000001 shl COMP3_CSR_COMP3MODE_Pos) ## !< 0x00000004
  COMP3_CSR_COMP3MODE_1* = (0x00000002 shl COMP3_CSR_COMP3MODE_Pos) ## !< 0x00000008
  COMP3_CSR_COMP3INSEL_Pos* = (4)
  COMP3_CSR_COMP3INSEL_Msk* = (0x00000007 shl COMP3_CSR_COMP3INSEL_Pos) ## !< 0x00000070
  COMP3_CSR_COMP3INSEL* = COMP3_CSR_COMP3INSEL_Msk
  COMP3_CSR_COMP3INSEL_0* = (0x00000001 shl COMP3_CSR_COMP3INSEL_Pos) ## !< 0x00000010
  COMP3_CSR_COMP3INSEL_1* = (0x00000002 shl COMP3_CSR_COMP3INSEL_Pos) ## !< 0x00000020
  COMP3_CSR_COMP3INSEL_2* = (0x00000004 shl COMP3_CSR_COMP3INSEL_Pos) ## !< 0x00000040
  COMP3_CSR_COMP3NONINSEL_Pos* = (7)
  COMP3_CSR_COMP3NONINSEL_Msk* = (0x00000001 shl COMP3_CSR_COMP3NONINSEL_Pos) ## !< 0x00000080
  COMP3_CSR_COMP3NONINSEL* = COMP3_CSR_COMP3NONINSEL_Msk
  COMP3_CSR_COMP3OUTSEL_Pos* = (10)
  COMP3_CSR_COMP3OUTSEL_Msk* = (0x0000000F shl COMP3_CSR_COMP3OUTSEL_Pos) ## !< 0x00003C00
  COMP3_CSR_COMP3OUTSEL* = COMP3_CSR_COMP3OUTSEL_Msk
  COMP3_CSR_COMP3OUTSEL_0* = (0x00000001 shl COMP3_CSR_COMP3OUTSEL_Pos) ## !< 0x00000400
  COMP3_CSR_COMP3OUTSEL_1* = (0x00000002 shl COMP3_CSR_COMP3OUTSEL_Pos) ## !< 0x00000800
  COMP3_CSR_COMP3OUTSEL_2* = (0x00000004 shl COMP3_CSR_COMP3OUTSEL_Pos) ## !< 0x00001000
  COMP3_CSR_COMP3OUTSEL_3* = (0x00000008 shl COMP3_CSR_COMP3OUTSEL_Pos) ## !< 0x00002000
  COMP3_CSR_COMP3POL_Pos* = (15)
  COMP3_CSR_COMP3POL_Msk* = (0x00000001 shl COMP3_CSR_COMP3POL_Pos) ## !< 0x00008000
  COMP3_CSR_COMP3POL* = COMP3_CSR_COMP3POL_Msk
  COMP3_CSR_COMP3HYST_Pos* = (16)
  COMP3_CSR_COMP3HYST_Msk* = (0x00000003 shl COMP3_CSR_COMP3HYST_Pos) ## !< 0x00030000
  COMP3_CSR_COMP3HYST* = COMP3_CSR_COMP3HYST_Msk
  COMP3_CSR_COMP3HYST_0* = (0x00000001 shl COMP3_CSR_COMP3HYST_Pos) ## !< 0x00010000
  COMP3_CSR_COMP3HYST_1* = (0x00000002 shl COMP3_CSR_COMP3HYST_Pos) ## !< 0x00020000
  COMP3_CSR_COMP3BLANKING_Pos* = (18)
  COMP3_CSR_COMP3BLANKING_Msk* = (0x00000003 shl COMP3_CSR_COMP3BLANKING_Pos) ## !< 0x000C0000
  COMP3_CSR_COMP3BLANKING* = COMP3_CSR_COMP3BLANKING_Msk
  COMP3_CSR_COMP3BLANKING_0* = (0x00000001 shl COMP3_CSR_COMP3BLANKING_Pos) ## !< 0x00040000
  COMP3_CSR_COMP3BLANKING_1* = (0x00000002 shl COMP3_CSR_COMP3BLANKING_Pos) ## !< 0x00080000
  COMP3_CSR_COMP3BLANKING_2* = (0x00000004 shl COMP3_CSR_COMP3BLANKING_Pos) ## !< 0x00100000
  COMP3_CSR_COMP3OUT_Pos* = (30)
  COMP3_CSR_COMP3OUT_Msk* = (0x00000001 shl COMP3_CSR_COMP3OUT_Pos) ## !< 0x40000000
  COMP3_CSR_COMP3OUT* = COMP3_CSR_COMP3OUT_Msk
  COMP3_CSR_COMP3LOCK_Pos* = (31)
  COMP3_CSR_COMP3LOCK_Msk* = (0x00000001 shl COMP3_CSR_COMP3LOCK_Pos) ## !< 0x80000000
  COMP3_CSR_COMP3LOCK* = COMP3_CSR_COMP3LOCK_Msk

## *********************  Bit definition for COMP4_CSR register  **************

const
  COMP4_CSR_COMP4EN_Pos* = (0)
  COMP4_CSR_COMP4EN_Msk* = (0x00000001 shl COMP4_CSR_COMP4EN_Pos) ## !< 0x00000001
  COMP4_CSR_COMP4EN* = COMP4_CSR_COMP4EN_Msk
  COMP4_CSR_COMP4MODE_Pos* = (2)
  COMP4_CSR_COMP4MODE_Msk* = (0x00000003 shl COMP4_CSR_COMP4MODE_Pos) ## !< 0x0000000C
  COMP4_CSR_COMP4MODE* = COMP4_CSR_COMP4MODE_Msk
  COMP4_CSR_COMP4MODE_0* = (0x00000001 shl COMP4_CSR_COMP4MODE_Pos) ## !< 0x00000004
  COMP4_CSR_COMP4MODE_1* = (0x00000002 shl COMP4_CSR_COMP4MODE_Pos) ## !< 0x00000008
  COMP4_CSR_COMP4INSEL_Pos* = (4)
  COMP4_CSR_COMP4INSEL_Msk* = (0x00000007 shl COMP4_CSR_COMP4INSEL_Pos) ## !< 0x00000070
  COMP4_CSR_COMP4INSEL* = COMP4_CSR_COMP4INSEL_Msk
  COMP4_CSR_COMP4INSEL_0* = (0x00000010) ## !< COMP4 inverting input select bit 0
  COMP4_CSR_COMP4INSEL_1* = (0x00000020) ## !< COMP4 inverting input select bit 1
  COMP4_CSR_COMP4INSEL_2* = (0x00000040) ## !< COMP4 inverting input select bit 2
  COMP4_CSR_COMP4NONINSEL_Pos* = (7)
  COMP4_CSR_COMP4NONINSEL_Msk* = (0x00000001 shl COMP4_CSR_COMP4NONINSEL_Pos) ## !< 0x00000080
  COMP4_CSR_COMP4NONINSEL* = COMP4_CSR_COMP4NONINSEL_Msk
  COMP4_CSR_COMP4WNDWEN_Pos* = (9)
  COMP4_CSR_COMP4WNDWEN_Msk* = (0x00000001 shl COMP4_CSR_COMP4WNDWEN_Pos) ## !< 0x00000200
  COMP4_CSR_COMP4WNDWEN* = COMP4_CSR_COMP4WNDWEN_Msk
  COMP4_CSR_COMP4OUTSEL_Pos* = (10)
  COMP4_CSR_COMP4OUTSEL_Msk* = (0x0000000F shl COMP4_CSR_COMP4OUTSEL_Pos) ## !< 0x00003C00
  COMP4_CSR_COMP4OUTSEL* = COMP4_CSR_COMP4OUTSEL_Msk
  COMP4_CSR_COMP4OUTSEL_0* = (0x00000001 shl COMP4_CSR_COMP4OUTSEL_Pos) ## !< 0x00000400
  COMP4_CSR_COMP4OUTSEL_1* = (0x00000002 shl COMP4_CSR_COMP4OUTSEL_Pos) ## !< 0x00000800
  COMP4_CSR_COMP4OUTSEL_2* = (0x00000004 shl COMP4_CSR_COMP4OUTSEL_Pos) ## !< 0x00001000
  COMP4_CSR_COMP4OUTSEL_3* = (0x00000008 shl COMP4_CSR_COMP4OUTSEL_Pos) ## !< 0x00002000
  COMP4_CSR_COMP4POL_Pos* = (15)
  COMP4_CSR_COMP4POL_Msk* = (0x00000001 shl COMP4_CSR_COMP4POL_Pos) ## !< 0x00008000
  COMP4_CSR_COMP4POL* = COMP4_CSR_COMP4POL_Msk
  COMP4_CSR_COMP4HYST_Pos* = (16)
  COMP4_CSR_COMP4HYST_Msk* = (0x00000003 shl COMP4_CSR_COMP4HYST_Pos) ## !< 0x00030000
  COMP4_CSR_COMP4HYST* = COMP4_CSR_COMP4HYST_Msk
  COMP4_CSR_COMP4HYST_0* = (0x00000001 shl COMP4_CSR_COMP4HYST_Pos) ## !< 0x00010000
  COMP4_CSR_COMP4HYST_1* = (0x00000002 shl COMP4_CSR_COMP4HYST_Pos) ## !< 0x00020000
  COMP4_CSR_COMP4BLANKING_Pos* = (18)
  COMP4_CSR_COMP4BLANKING_Msk* = (0x00000003 shl COMP4_CSR_COMP4BLANKING_Pos) ## !< 0x000C0000
  COMP4_CSR_COMP4BLANKING* = COMP4_CSR_COMP4BLANKING_Msk
  COMP4_CSR_COMP4BLANKING_0* = (0x00000001 shl COMP4_CSR_COMP4BLANKING_Pos) ## !< 0x00040000
  COMP4_CSR_COMP4BLANKING_1* = (0x00000002 shl COMP4_CSR_COMP4BLANKING_Pos) ## !< 0x00080000
  COMP4_CSR_COMP4BLANKING_2* = (0x00000004 shl COMP4_CSR_COMP4BLANKING_Pos) ## !< 0x00100000
  COMP4_CSR_COMP4OUT_Pos* = (30)
  COMP4_CSR_COMP4OUT_Msk* = (0x00000001 shl COMP4_CSR_COMP4OUT_Pos) ## !< 0x40000000
  COMP4_CSR_COMP4OUT* = COMP4_CSR_COMP4OUT_Msk
  COMP4_CSR_COMP4LOCK_Pos* = (31)
  COMP4_CSR_COMP4LOCK_Msk* = (0x00000001 shl COMP4_CSR_COMP4LOCK_Pos) ## !< 0x80000000
  COMP4_CSR_COMP4LOCK* = COMP4_CSR_COMP4LOCK_Msk

## *********************  Bit definition for COMP5_CSR register  **************

const
  COMP5_CSR_COMP5EN_Pos* = (0)
  COMP5_CSR_COMP5EN_Msk* = (0x00000001 shl COMP5_CSR_COMP5EN_Pos) ## !< 0x00000001
  COMP5_CSR_COMP5EN* = COMP5_CSR_COMP5EN_Msk
  COMP5_CSR_COMP5MODE_Pos* = (2)
  COMP5_CSR_COMP5MODE_Msk* = (0x00000003 shl COMP5_CSR_COMP5MODE_Pos) ## !< 0x0000000C
  COMP5_CSR_COMP5MODE* = COMP5_CSR_COMP5MODE_Msk
  COMP5_CSR_COMP5MODE_0* = (0x00000001 shl COMP5_CSR_COMP5MODE_Pos) ## !< 0x00000004
  COMP5_CSR_COMP5MODE_1* = (0x00000002 shl COMP5_CSR_COMP5MODE_Pos) ## !< 0x00000008
  COMP5_CSR_COMP5INSEL_Pos* = (4)
  COMP5_CSR_COMP5INSEL_Msk* = (0x00000007 shl COMP5_CSR_COMP5INSEL_Pos) ## !< 0x00000070
  COMP5_CSR_COMP5INSEL* = COMP5_CSR_COMP5INSEL_Msk
  COMP5_CSR_COMP5INSEL_0* = (0x00000001 shl COMP5_CSR_COMP5INSEL_Pos) ## !< 0x00000010
  COMP5_CSR_COMP5INSEL_1* = (0x00000002 shl COMP5_CSR_COMP5INSEL_Pos) ## !< 0x00000020
  COMP5_CSR_COMP5INSEL_2* = (0x00000004 shl COMP5_CSR_COMP5INSEL_Pos) ## !< 0x00000040
  COMP5_CSR_COMP5NONINSEL_Pos* = (7)
  COMP5_CSR_COMP5NONINSEL_Msk* = (0x00000001 shl COMP5_CSR_COMP5NONINSEL_Pos) ## !< 0x00000080
  COMP5_CSR_COMP5NONINSEL* = COMP5_CSR_COMP5NONINSEL_Msk
  COMP5_CSR_COMP5OUTSEL_Pos* = (10)
  COMP5_CSR_COMP5OUTSEL_Msk* = (0x0000000F shl COMP5_CSR_COMP5OUTSEL_Pos) ## !< 0x00003C00
  COMP5_CSR_COMP5OUTSEL* = COMP5_CSR_COMP5OUTSEL_Msk
  COMP5_CSR_COMP5OUTSEL_0* = (0x00000001 shl COMP5_CSR_COMP5OUTSEL_Pos) ## !< 0x00000400
  COMP5_CSR_COMP5OUTSEL_1* = (0x00000002 shl COMP5_CSR_COMP5OUTSEL_Pos) ## !< 0x00000800
  COMP5_CSR_COMP5OUTSEL_2* = (0x00000004 shl COMP5_CSR_COMP5OUTSEL_Pos) ## !< 0x00001000
  COMP5_CSR_COMP5OUTSEL_3* = (0x00000008 shl COMP5_CSR_COMP5OUTSEL_Pos) ## !< 0x00002000
  COMP5_CSR_COMP5POL_Pos* = (15)
  COMP5_CSR_COMP5POL_Msk* = (0x00000001 shl COMP5_CSR_COMP5POL_Pos) ## !< 0x00008000
  COMP5_CSR_COMP5POL* = COMP5_CSR_COMP5POL_Msk
  COMP5_CSR_COMP5HYST_Pos* = (16)
  COMP5_CSR_COMP5HYST_Msk* = (0x00000003 shl COMP5_CSR_COMP5HYST_Pos) ## !< 0x00030000
  COMP5_CSR_COMP5HYST* = COMP5_CSR_COMP5HYST_Msk
  COMP5_CSR_COMP5HYST_0* = (0x00000001 shl COMP5_CSR_COMP5HYST_Pos) ## !< 0x00010000
  COMP5_CSR_COMP5HYST_1* = (0x00000002 shl COMP5_CSR_COMP5HYST_Pos) ## !< 0x00020000
  COMP5_CSR_COMP5BLANKING_Pos* = (18)
  COMP5_CSR_COMP5BLANKING_Msk* = (0x00000003 shl COMP5_CSR_COMP5BLANKING_Pos) ## !< 0x000C0000
  COMP5_CSR_COMP5BLANKING* = COMP5_CSR_COMP5BLANKING_Msk
  COMP5_CSR_COMP5BLANKING_0* = (0x00000001 shl COMP5_CSR_COMP5BLANKING_Pos) ## !< 0x00040000
  COMP5_CSR_COMP5BLANKING_1* = (0x00000002 shl COMP5_CSR_COMP5BLANKING_Pos) ## !< 0x00080000
  COMP5_CSR_COMP5BLANKING_2* = (0x00000004 shl COMP5_CSR_COMP5BLANKING_Pos) ## !< 0x00100000
  COMP5_CSR_COMP5OUT_Pos* = (30)
  COMP5_CSR_COMP5OUT_Msk* = (0x00000001 shl COMP5_CSR_COMP5OUT_Pos) ## !< 0x40000000
  COMP5_CSR_COMP5OUT* = COMP5_CSR_COMP5OUT_Msk
  COMP5_CSR_COMP5LOCK_Pos* = (31)
  COMP5_CSR_COMP5LOCK_Msk* = (0x00000001 shl COMP5_CSR_COMP5LOCK_Pos) ## !< 0x80000000
  COMP5_CSR_COMP5LOCK* = COMP5_CSR_COMP5LOCK_Msk

## *********************  Bit definition for COMP6_CSR register  **************

const
  COMP6_CSR_COMP6EN_Pos* = (0)
  COMP6_CSR_COMP6EN_Msk* = (0x00000001 shl COMP6_CSR_COMP6EN_Pos) ## !< 0x00000001
  COMP6_CSR_COMP6EN* = COMP6_CSR_COMP6EN_Msk
  COMP6_CSR_COMP6MODE_Pos* = (2)
  COMP6_CSR_COMP6MODE_Msk* = (0x00000003 shl COMP6_CSR_COMP6MODE_Pos) ## !< 0x0000000C
  COMP6_CSR_COMP6MODE* = COMP6_CSR_COMP6MODE_Msk
  COMP6_CSR_COMP6MODE_0* = (0x00000001 shl COMP6_CSR_COMP6MODE_Pos) ## !< 0x00000004
  COMP6_CSR_COMP6MODE_1* = (0x00000002 shl COMP6_CSR_COMP6MODE_Pos) ## !< 0x00000008
  COMP6_CSR_COMP6INSEL_Pos* = (4)
  COMP6_CSR_COMP6INSEL_Msk* = (0x00000007 shl COMP6_CSR_COMP6INSEL_Pos) ## !< 0x00000070
  COMP6_CSR_COMP6INSEL* = COMP6_CSR_COMP6INSEL_Msk
  COMP6_CSR_COMP6INSEL_0* = (0x00000010) ## !< COMP6 inverting input select bit 0
  COMP6_CSR_COMP6INSEL_1* = (0x00000020) ## !< COMP6 inverting input select bit 1
  COMP6_CSR_COMP6INSEL_2* = (0x00000040) ## !< COMP6 inverting input select bit 2
  COMP6_CSR_COMP6NONINSEL_Pos* = (7)
  COMP6_CSR_COMP6NONINSEL_Msk* = (0x00000001 shl COMP6_CSR_COMP6NONINSEL_Pos) ## !< 0x00000080
  COMP6_CSR_COMP6NONINSEL* = COMP6_CSR_COMP6NONINSEL_Msk
  COMP6_CSR_COMP6WNDWEN_Pos* = (9)
  COMP6_CSR_COMP6WNDWEN_Msk* = (0x00000001 shl COMP6_CSR_COMP6WNDWEN_Pos) ## !< 0x00000200
  COMP6_CSR_COMP6WNDWEN* = COMP6_CSR_COMP6WNDWEN_Msk
  COMP6_CSR_COMP6OUTSEL_Pos* = (10)
  COMP6_CSR_COMP6OUTSEL_Msk* = (0x0000000F shl COMP6_CSR_COMP6OUTSEL_Pos) ## !< 0x00003C00
  COMP6_CSR_COMP6OUTSEL* = COMP6_CSR_COMP6OUTSEL_Msk
  COMP6_CSR_COMP6OUTSEL_0* = (0x00000001 shl COMP6_CSR_COMP6OUTSEL_Pos) ## !< 0x00000400
  COMP6_CSR_COMP6OUTSEL_1* = (0x00000002 shl COMP6_CSR_COMP6OUTSEL_Pos) ## !< 0x00000800
  COMP6_CSR_COMP6OUTSEL_2* = (0x00000004 shl COMP6_CSR_COMP6OUTSEL_Pos) ## !< 0x00001000
  COMP6_CSR_COMP6OUTSEL_3* = (0x00000008 shl COMP6_CSR_COMP6OUTSEL_Pos) ## !< 0x00002000
  COMP6_CSR_COMP6POL_Pos* = (15)
  COMP6_CSR_COMP6POL_Msk* = (0x00000001 shl COMP6_CSR_COMP6POL_Pos) ## !< 0x00008000
  COMP6_CSR_COMP6POL* = COMP6_CSR_COMP6POL_Msk
  COMP6_CSR_COMP6HYST_Pos* = (16)
  COMP6_CSR_COMP6HYST_Msk* = (0x00000003 shl COMP6_CSR_COMP6HYST_Pos) ## !< 0x00030000
  COMP6_CSR_COMP6HYST* = COMP6_CSR_COMP6HYST_Msk
  COMP6_CSR_COMP6HYST_0* = (0x00000001 shl COMP6_CSR_COMP6HYST_Pos) ## !< 0x00010000
  COMP6_CSR_COMP6HYST_1* = (0x00000002 shl COMP6_CSR_COMP6HYST_Pos) ## !< 0x00020000
  COMP6_CSR_COMP6BLANKING_Pos* = (18)
  COMP6_CSR_COMP6BLANKING_Msk* = (0x00000003 shl COMP6_CSR_COMP6BLANKING_Pos) ## !< 0x000C0000
  COMP6_CSR_COMP6BLANKING* = COMP6_CSR_COMP6BLANKING_Msk
  COMP6_CSR_COMP6BLANKING_0* = (0x00000001 shl COMP6_CSR_COMP6BLANKING_Pos) ## !< 0x00040000
  COMP6_CSR_COMP6BLANKING_1* = (0x00000002 shl COMP6_CSR_COMP6BLANKING_Pos) ## !< 0x00080000
  COMP6_CSR_COMP6BLANKING_2* = (0x00000004 shl COMP6_CSR_COMP6BLANKING_Pos) ## !< 0x00100000
  COMP6_CSR_COMP6OUT_Pos* = (30)
  COMP6_CSR_COMP6OUT_Msk* = (0x00000001 shl COMP6_CSR_COMP6OUT_Pos) ## !< 0x40000000
  COMP6_CSR_COMP6OUT* = COMP6_CSR_COMP6OUT_Msk
  COMP6_CSR_COMP6LOCK_Pos* = (31)
  COMP6_CSR_COMP6LOCK_Msk* = (0x00000001 shl COMP6_CSR_COMP6LOCK_Pos) ## !< 0x80000000
  COMP6_CSR_COMP6LOCK* = COMP6_CSR_COMP6LOCK_Msk

## *********************  Bit definition for COMP7_CSR register  **************

const
  COMP7_CSR_COMP7EN_Pos* = (0)
  COMP7_CSR_COMP7EN_Msk* = (0x00000001 shl COMP7_CSR_COMP7EN_Pos) ## !< 0x00000001
  COMP7_CSR_COMP7EN* = COMP7_CSR_COMP7EN_Msk
  COMP7_CSR_COMP7MODE_Pos* = (2)
  COMP7_CSR_COMP7MODE_Msk* = (0x00000003 shl COMP7_CSR_COMP7MODE_Pos) ## !< 0x0000000C
  COMP7_CSR_COMP7MODE* = COMP7_CSR_COMP7MODE_Msk
  COMP7_CSR_COMP7MODE_0* = (0x00000001 shl COMP7_CSR_COMP7MODE_Pos) ## !< 0x00000004
  COMP7_CSR_COMP7MODE_1* = (0x00000002 shl COMP7_CSR_COMP7MODE_Pos) ## !< 0x00000008
  COMP7_CSR_COMP7INSEL_Pos* = (4)
  COMP7_CSR_COMP7INSEL_Msk* = (0x00000007 shl COMP7_CSR_COMP7INSEL_Pos) ## !< 0x00000070
  COMP7_CSR_COMP7INSEL* = COMP7_CSR_COMP7INSEL_Msk
  COMP7_CSR_COMP7INSEL_0* = (0x00000001 shl COMP7_CSR_COMP7INSEL_Pos) ## !< 0x00000010
  COMP7_CSR_COMP7INSEL_1* = (0x00000002 shl COMP7_CSR_COMP7INSEL_Pos) ## !< 0x00000020
  COMP7_CSR_COMP7INSEL_2* = (0x00000004 shl COMP7_CSR_COMP7INSEL_Pos) ## !< 0x00000040
  COMP7_CSR_COMP7NONINSEL_Pos* = (7)
  COMP7_CSR_COMP7NONINSEL_Msk* = (0x00000001 shl COMP7_CSR_COMP7NONINSEL_Pos) ## !< 0x00000080
  COMP7_CSR_COMP7NONINSEL* = COMP7_CSR_COMP7NONINSEL_Msk
  COMP7_CSR_COMP7OUTSEL_Pos* = (10)
  COMP7_CSR_COMP7OUTSEL_Msk* = (0x0000000F shl COMP7_CSR_COMP7OUTSEL_Pos) ## !< 0x00003C00
  COMP7_CSR_COMP7OUTSEL* = COMP7_CSR_COMP7OUTSEL_Msk
  COMP7_CSR_COMP7OUTSEL_0* = (0x00000001 shl COMP7_CSR_COMP7OUTSEL_Pos) ## !< 0x00000400
  COMP7_CSR_COMP7OUTSEL_1* = (0x00000002 shl COMP7_CSR_COMP7OUTSEL_Pos) ## !< 0x00000800
  COMP7_CSR_COMP7OUTSEL_2* = (0x00000004 shl COMP7_CSR_COMP7OUTSEL_Pos) ## !< 0x00001000
  COMP7_CSR_COMP7OUTSEL_3* = (0x00000008 shl COMP7_CSR_COMP7OUTSEL_Pos) ## !< 0x00002000
  COMP7_CSR_COMP7POL_Pos* = (15)
  COMP7_CSR_COMP7POL_Msk* = (0x00000001 shl COMP7_CSR_COMP7POL_Pos) ## !< 0x00008000
  COMP7_CSR_COMP7POL* = COMP7_CSR_COMP7POL_Msk
  COMP7_CSR_COMP7HYST_Pos* = (16)
  COMP7_CSR_COMP7HYST_Msk* = (0x00000003 shl COMP7_CSR_COMP7HYST_Pos) ## !< 0x00030000
  COMP7_CSR_COMP7HYST* = COMP7_CSR_COMP7HYST_Msk
  COMP7_CSR_COMP7HYST_0* = (0x00000001 shl COMP7_CSR_COMP7HYST_Pos) ## !< 0x00010000
  COMP7_CSR_COMP7HYST_1* = (0x00000002 shl COMP7_CSR_COMP7HYST_Pos) ## !< 0x00020000
  COMP7_CSR_COMP7BLANKING_Pos* = (18)
  COMP7_CSR_COMP7BLANKING_Msk* = (0x00000003 shl COMP7_CSR_COMP7BLANKING_Pos) ## !< 0x000C0000
  COMP7_CSR_COMP7BLANKING* = COMP7_CSR_COMP7BLANKING_Msk
  COMP7_CSR_COMP7BLANKING_0* = (0x00000001 shl COMP7_CSR_COMP7BLANKING_Pos) ## !< 0x00040000
  COMP7_CSR_COMP7BLANKING_1* = (0x00000002 shl COMP7_CSR_COMP7BLANKING_Pos) ## !< 0x00080000
  COMP7_CSR_COMP7BLANKING_2* = (0x00000004 shl COMP7_CSR_COMP7BLANKING_Pos) ## !< 0x00100000
  COMP7_CSR_COMP7OUT_Pos* = (30)
  COMP7_CSR_COMP7OUT_Msk* = (0x00000001 shl COMP7_CSR_COMP7OUT_Pos) ## !< 0x40000000
  COMP7_CSR_COMP7OUT* = COMP7_CSR_COMP7OUT_Msk
  COMP7_CSR_COMP7LOCK_Pos* = (31)
  COMP7_CSR_COMP7LOCK_Msk* = (0x00000001 shl COMP7_CSR_COMP7LOCK_Pos) ## !< 0x80000000
  COMP7_CSR_COMP7LOCK* = COMP7_CSR_COMP7LOCK_Msk

## *********************  Bit definition for COMP_CSR register  ***************

const
  COMP_CSR_COMPxEN_Pos* = (0)
  COMP_CSR_COMPxEN_Msk* = (0x00000001 shl COMP_CSR_COMPxEN_Pos) ## !< 0x00000001
  COMP_CSR_COMPxEN* = COMP_CSR_COMPxEN_Msk
  COMP_CSR_COMPxSW1_Pos* = (1)
  COMP_CSR_COMPxSW1_Msk* = (0x00000001 shl COMP_CSR_COMPxSW1_Pos) ## !< 0x00000002
  COMP_CSR_COMPxSW1* = COMP_CSR_COMPxSW1_Msk
  COMP_CSR_COMPxMODE_Pos* = (2)
  COMP_CSR_COMPxMODE_Msk* = (0x00000003 shl COMP_CSR_COMPxMODE_Pos) ## !< 0x0000000C
  COMP_CSR_COMPxMODE* = COMP_CSR_COMPxMODE_Msk
  COMP_CSR_COMPxMODE_0* = (0x00000001 shl COMP_CSR_COMPxMODE_Pos) ## !< 0x00000004
  COMP_CSR_COMPxMODE_1* = (0x00000002 shl COMP_CSR_COMPxMODE_Pos) ## !< 0x00000008
  COMP_CSR_COMPxINSEL_Pos* = (4)
  COMP_CSR_COMPxINSEL_Msk* = (0x00000007 shl COMP_CSR_COMPxINSEL_Pos) ## !< 0x00000070
  COMP_CSR_COMPxINSEL* = COMP_CSR_COMPxINSEL_Msk
  COMP_CSR_COMPxINSEL_0* = (0x00000010) ## !< COMPx inverting input select bit 0
  COMP_CSR_COMPxINSEL_1* = (0x00000020) ## !< COMPx inverting input select bit 1
  COMP_CSR_COMPxINSEL_2* = (0x00000040) ## !< COMPx inverting input select bit 2
  COMP_CSR_COMPxNONINSEL_Pos* = (7)
  COMP_CSR_COMPxNONINSEL_Msk* = (0x00000001 shl COMP_CSR_COMPxNONINSEL_Pos) ## !< 0x00000080
  COMP_CSR_COMPxNONINSEL* = COMP_CSR_COMPxNONINSEL_Msk
  COMP_CSR_COMPxWNDWEN_Pos* = (9)
  COMP_CSR_COMPxWNDWEN_Msk* = (0x00000001 shl COMP_CSR_COMPxWNDWEN_Pos) ## !< 0x00000200
  COMP_CSR_COMPxWNDWEN* = COMP_CSR_COMPxWNDWEN_Msk
  COMP_CSR_COMPxOUTSEL_Pos* = (10)
  COMP_CSR_COMPxOUTSEL_Msk* = (0x0000000F shl COMP_CSR_COMPxOUTSEL_Pos) ## !< 0x00003C00
  COMP_CSR_COMPxOUTSEL* = COMP_CSR_COMPxOUTSEL_Msk
  COMP_CSR_COMPxOUTSEL_0* = (0x00000001 shl COMP_CSR_COMPxOUTSEL_Pos) ## !< 0x00000400
  COMP_CSR_COMPxOUTSEL_1* = (0x00000002 shl COMP_CSR_COMPxOUTSEL_Pos) ## !< 0x00000800
  COMP_CSR_COMPxOUTSEL_2* = (0x00000004 shl COMP_CSR_COMPxOUTSEL_Pos) ## !< 0x00001000
  COMP_CSR_COMPxOUTSEL_3* = (0x00000008 shl COMP_CSR_COMPxOUTSEL_Pos) ## !< 0x00002000
  COMP_CSR_COMPxPOL_Pos* = (15)
  COMP_CSR_COMPxPOL_Msk* = (0x00000001 shl COMP_CSR_COMPxPOL_Pos) ## !< 0x00008000
  COMP_CSR_COMPxPOL* = COMP_CSR_COMPxPOL_Msk
  COMP_CSR_COMPxHYST_Pos* = (16)
  COMP_CSR_COMPxHYST_Msk* = (0x00000003 shl COMP_CSR_COMPxHYST_Pos) ## !< 0x00030000
  COMP_CSR_COMPxHYST* = COMP_CSR_COMPxHYST_Msk
  COMP_CSR_COMPxHYST_0* = (0x00000001 shl COMP_CSR_COMPxHYST_Pos) ## !< 0x00010000
  COMP_CSR_COMPxHYST_1* = (0x00000002 shl COMP_CSR_COMPxHYST_Pos) ## !< 0x00020000
  COMP_CSR_COMPxBLANKING_Pos* = (18)
  COMP_CSR_COMPxBLANKING_Msk* = (0x00000003 shl COMP_CSR_COMPxBLANKING_Pos) ## !< 0x000C0000
  COMP_CSR_COMPxBLANKING* = COMP_CSR_COMPxBLANKING_Msk
  COMP_CSR_COMPxBLANKING_0* = (0x00000001 shl COMP_CSR_COMPxBLANKING_Pos) ## !< 0x00040000
  COMP_CSR_COMPxBLANKING_1* = (0x00000002 shl COMP_CSR_COMPxBLANKING_Pos) ## !< 0x00080000
  COMP_CSR_COMPxBLANKING_2* = (0x00000004 shl COMP_CSR_COMPxBLANKING_Pos) ## !< 0x00100000
  COMP_CSR_COMPxOUT_Pos* = (30)
  COMP_CSR_COMPxOUT_Msk* = (0x00000001 shl COMP_CSR_COMPxOUT_Pos) ## !< 0x40000000
  COMP_CSR_COMPxOUT* = COMP_CSR_COMPxOUT_Msk
  COMP_CSR_COMPxLOCK_Pos* = (31)
  COMP_CSR_COMPxLOCK_Msk* = (0x00000001 shl COMP_CSR_COMPxLOCK_Pos) ## !< 0x80000000
  COMP_CSR_COMPxLOCK* = COMP_CSR_COMPxLOCK_Msk

## ****************************************************************************
##
##                      Operational Amplifier (OPAMP)
##
## ****************************************************************************
## ********************  Bit definition for OPAMP1_CSR register  **************

const
  OPAMP1_CSR_OPAMP1EN_Pos* = (0)
  OPAMP1_CSR_OPAMP1EN_Msk* = (0x00000001 shl OPAMP1_CSR_OPAMP1EN_Pos) ## !< 0x00000001
  OPAMP1_CSR_OPAMP1EN* = OPAMP1_CSR_OPAMP1EN_Msk
  OPAMP1_CSR_FORCEVP_Pos* = (1)
  OPAMP1_CSR_FORCEVP_Msk* = (0x00000001 shl OPAMP1_CSR_FORCEVP_Pos) ## !< 0x00000002
  OPAMP1_CSR_FORCEVP* = OPAMP1_CSR_FORCEVP_Msk
  OPAMP1_CSR_VPSEL_Pos* = (2)
  OPAMP1_CSR_VPSEL_Msk* = (0x00000003 shl OPAMP1_CSR_VPSEL_Pos) ## !< 0x0000000C
  OPAMP1_CSR_VPSEL* = OPAMP1_CSR_VPSEL_Msk
  OPAMP1_CSR_VPSEL_0* = (0x00000001 shl OPAMP1_CSR_VPSEL_Pos) ## !< 0x00000004
  OPAMP1_CSR_VPSEL_1* = (0x00000002 shl OPAMP1_CSR_VPSEL_Pos) ## !< 0x00000008
  OPAMP1_CSR_VMSEL_Pos* = (5)
  OPAMP1_CSR_VMSEL_Msk* = (0x00000003 shl OPAMP1_CSR_VMSEL_Pos) ## !< 0x00000060
  OPAMP1_CSR_VMSEL* = OPAMP1_CSR_VMSEL_Msk
  OPAMP1_CSR_VMSEL_0* = (0x00000001 shl OPAMP1_CSR_VMSEL_Pos) ## !< 0x00000020
  OPAMP1_CSR_VMSEL_1* = (0x00000002 shl OPAMP1_CSR_VMSEL_Pos) ## !< 0x00000040
  OPAMP1_CSR_TCMEN_Pos* = (7)
  OPAMP1_CSR_TCMEN_Msk* = (0x00000001 shl OPAMP1_CSR_TCMEN_Pos) ## !< 0x00000080
  OPAMP1_CSR_TCMEN* = OPAMP1_CSR_TCMEN_Msk
  OPAMP1_CSR_VMSSEL_Pos* = (8)
  OPAMP1_CSR_VMSSEL_Msk* = (0x00000001 shl OPAMP1_CSR_VMSSEL_Pos) ## !< 0x00000100
  OPAMP1_CSR_VMSSEL* = OPAMP1_CSR_VMSSEL_Msk
  OPAMP1_CSR_VPSSEL_Pos* = (9)
  OPAMP1_CSR_VPSSEL_Msk* = (0x00000003 shl OPAMP1_CSR_VPSSEL_Pos) ## !< 0x00000600
  OPAMP1_CSR_VPSSEL* = OPAMP1_CSR_VPSSEL_Msk
  OPAMP1_CSR_VPSSEL_0* = (0x00000001 shl OPAMP1_CSR_VPSSEL_Pos) ## !< 0x00000200
  OPAMP1_CSR_VPSSEL_1* = (0x00000002 shl OPAMP1_CSR_VPSSEL_Pos) ## !< 0x00000400
  OPAMP1_CSR_CALON_Pos* = (11)
  OPAMP1_CSR_CALON_Msk* = (0x00000001 shl OPAMP1_CSR_CALON_Pos) ## !< 0x00000800
  OPAMP1_CSR_CALON* = OPAMP1_CSR_CALON_Msk
  OPAMP1_CSR_CALSEL_Pos* = (12)
  OPAMP1_CSR_CALSEL_Msk* = (0x00000003 shl OPAMP1_CSR_CALSEL_Pos) ## !< 0x00003000
  OPAMP1_CSR_CALSEL* = OPAMP1_CSR_CALSEL_Msk
  OPAMP1_CSR_CALSEL_0* = (0x00000001 shl OPAMP1_CSR_CALSEL_Pos) ## !< 0x00001000
  OPAMP1_CSR_CALSEL_1* = (0x00000002 shl OPAMP1_CSR_CALSEL_Pos) ## !< 0x00002000
  OPAMP1_CSR_PGGAIN_Pos* = (14)
  OPAMP1_CSR_PGGAIN_Msk* = (0x0000000F shl OPAMP1_CSR_PGGAIN_Pos) ## !< 0x0003C000
  OPAMP1_CSR_PGGAIN* = OPAMP1_CSR_PGGAIN_Msk
  OPAMP1_CSR_PGGAIN_0* = (0x00000001 shl OPAMP1_CSR_PGGAIN_Pos) ## !< 0x00004000
  OPAMP1_CSR_PGGAIN_1* = (0x00000002 shl OPAMP1_CSR_PGGAIN_Pos) ## !< 0x00008000
  OPAMP1_CSR_PGGAIN_2* = (0x00000004 shl OPAMP1_CSR_PGGAIN_Pos) ## !< 0x00010000
  OPAMP1_CSR_PGGAIN_3* = (0x00000008 shl OPAMP1_CSR_PGGAIN_Pos) ## !< 0x00020000
  OPAMP1_CSR_USERTRIM_Pos* = (18)
  OPAMP1_CSR_USERTRIM_Msk* = (0x00000001 shl OPAMP1_CSR_USERTRIM_Pos) ## !< 0x00040000
  OPAMP1_CSR_USERTRIM* = OPAMP1_CSR_USERTRIM_Msk
  OPAMP1_CSR_TRIMOFFSETP_Pos* = (19)
  OPAMP1_CSR_TRIMOFFSETP_Msk* = (0x0000001F shl OPAMP1_CSR_TRIMOFFSETP_Pos) ## !< 0x00F80000
  OPAMP1_CSR_TRIMOFFSETP* = OPAMP1_CSR_TRIMOFFSETP_Msk
  OPAMP1_CSR_TRIMOFFSETN_Pos* = (24)
  OPAMP1_CSR_TRIMOFFSETN_Msk* = (0x0000001F shl OPAMP1_CSR_TRIMOFFSETN_Pos) ## !< 0x1F000000
  OPAMP1_CSR_TRIMOFFSETN* = OPAMP1_CSR_TRIMOFFSETN_Msk
  OPAMP1_CSR_TSTREF_Pos* = (29)
  OPAMP1_CSR_TSTREF_Msk* = (0x00000001 shl OPAMP1_CSR_TSTREF_Pos) ## !< 0x20000000
  OPAMP1_CSR_TSTREF* = OPAMP1_CSR_TSTREF_Msk
  OPAMP1_CSR_OUTCAL_Pos* = (30)
  OPAMP1_CSR_OUTCAL_Msk* = (0x00000001 shl OPAMP1_CSR_OUTCAL_Pos) ## !< 0x40000000
  OPAMP1_CSR_OUTCAL* = OPAMP1_CSR_OUTCAL_Msk
  OPAMP1_CSR_LOCK_Pos* = (31)
  OPAMP1_CSR_LOCK_Msk* = (0x00000001 shl OPAMP1_CSR_LOCK_Pos) ## !< 0x80000000
  OPAMP1_CSR_LOCK* = OPAMP1_CSR_LOCK_Msk

## ********************  Bit definition for OPAMP2_CSR register  **************

const
  OPAMP2_CSR_OPAMP2EN_Pos* = (0)
  OPAMP2_CSR_OPAMP2EN_Msk* = (0x00000001 shl OPAMP2_CSR_OPAMP2EN_Pos) ## !< 0x00000001
  OPAMP2_CSR_OPAMP2EN* = OPAMP2_CSR_OPAMP2EN_Msk
  OPAMP2_CSR_FORCEVP_Pos* = (1)
  OPAMP2_CSR_FORCEVP_Msk* = (0x00000001 shl OPAMP2_CSR_FORCEVP_Pos) ## !< 0x00000002
  OPAMP2_CSR_FORCEVP* = OPAMP2_CSR_FORCEVP_Msk
  OPAMP2_CSR_VPSEL_Pos* = (2)
  OPAMP2_CSR_VPSEL_Msk* = (0x00000003 shl OPAMP2_CSR_VPSEL_Pos) ## !< 0x0000000C
  OPAMP2_CSR_VPSEL* = OPAMP2_CSR_VPSEL_Msk
  OPAMP2_CSR_VPSEL_0* = (0x00000001 shl OPAMP2_CSR_VPSEL_Pos) ## !< 0x00000004
  OPAMP2_CSR_VPSEL_1* = (0x00000002 shl OPAMP2_CSR_VPSEL_Pos) ## !< 0x00000008
  OPAMP2_CSR_VMSEL_Pos* = (5)
  OPAMP2_CSR_VMSEL_Msk* = (0x00000003 shl OPAMP2_CSR_VMSEL_Pos) ## !< 0x00000060
  OPAMP2_CSR_VMSEL* = OPAMP2_CSR_VMSEL_Msk
  OPAMP2_CSR_VMSEL_0* = (0x00000001 shl OPAMP2_CSR_VMSEL_Pos) ## !< 0x00000020
  OPAMP2_CSR_VMSEL_1* = (0x00000002 shl OPAMP2_CSR_VMSEL_Pos) ## !< 0x00000040
  OPAMP2_CSR_TCMEN_Pos* = (7)
  OPAMP2_CSR_TCMEN_Msk* = (0x00000001 shl OPAMP2_CSR_TCMEN_Pos) ## !< 0x00000080
  OPAMP2_CSR_TCMEN* = OPAMP2_CSR_TCMEN_Msk
  OPAMP2_CSR_VMSSEL_Pos* = (8)
  OPAMP2_CSR_VMSSEL_Msk* = (0x00000001 shl OPAMP2_CSR_VMSSEL_Pos) ## !< 0x00000100
  OPAMP2_CSR_VMSSEL* = OPAMP2_CSR_VMSSEL_Msk
  OPAMP2_CSR_VPSSEL_Pos* = (9)
  OPAMP2_CSR_VPSSEL_Msk* = (0x00000003 shl OPAMP2_CSR_VPSSEL_Pos) ## !< 0x00000600
  OPAMP2_CSR_VPSSEL* = OPAMP2_CSR_VPSSEL_Msk
  OPAMP2_CSR_VPSSEL_0* = (0x00000001 shl OPAMP2_CSR_VPSSEL_Pos) ## !< 0x00000200
  OPAMP2_CSR_VPSSEL_1* = (0x00000002 shl OPAMP2_CSR_VPSSEL_Pos) ## !< 0x00000400
  OPAMP2_CSR_CALON_Pos* = (11)
  OPAMP2_CSR_CALON_Msk* = (0x00000001 shl OPAMP2_CSR_CALON_Pos) ## !< 0x00000800
  OPAMP2_CSR_CALON* = OPAMP2_CSR_CALON_Msk
  OPAMP2_CSR_CALSEL_Pos* = (12)
  OPAMP2_CSR_CALSEL_Msk* = (0x00000003 shl OPAMP2_CSR_CALSEL_Pos) ## !< 0x00003000
  OPAMP2_CSR_CALSEL* = OPAMP2_CSR_CALSEL_Msk
  OPAMP2_CSR_CALSEL_0* = (0x00000001 shl OPAMP2_CSR_CALSEL_Pos) ## !< 0x00001000
  OPAMP2_CSR_CALSEL_1* = (0x00000002 shl OPAMP2_CSR_CALSEL_Pos) ## !< 0x00002000
  OPAMP2_CSR_PGGAIN_Pos* = (14)
  OPAMP2_CSR_PGGAIN_Msk* = (0x0000000F shl OPAMP2_CSR_PGGAIN_Pos) ## !< 0x0003C000
  OPAMP2_CSR_PGGAIN* = OPAMP2_CSR_PGGAIN_Msk
  OPAMP2_CSR_PGGAIN_0* = (0x00000001 shl OPAMP2_CSR_PGGAIN_Pos) ## !< 0x00004000
  OPAMP2_CSR_PGGAIN_1* = (0x00000002 shl OPAMP2_CSR_PGGAIN_Pos) ## !< 0x00008000
  OPAMP2_CSR_PGGAIN_2* = (0x00000004 shl OPAMP2_CSR_PGGAIN_Pos) ## !< 0x00010000
  OPAMP2_CSR_PGGAIN_3* = (0x00000008 shl OPAMP2_CSR_PGGAIN_Pos) ## !< 0x00020000
  OPAMP2_CSR_USERTRIM_Pos* = (18)
  OPAMP2_CSR_USERTRIM_Msk* = (0x00000001 shl OPAMP2_CSR_USERTRIM_Pos) ## !< 0x00040000
  OPAMP2_CSR_USERTRIM* = OPAMP2_CSR_USERTRIM_Msk
  OPAMP2_CSR_TRIMOFFSETP_Pos* = (19)
  OPAMP2_CSR_TRIMOFFSETP_Msk* = (0x0000001F shl OPAMP2_CSR_TRIMOFFSETP_Pos) ## !< 0x00F80000
  OPAMP2_CSR_TRIMOFFSETP* = OPAMP2_CSR_TRIMOFFSETP_Msk
  OPAMP2_CSR_TRIMOFFSETN_Pos* = (24)
  OPAMP2_CSR_TRIMOFFSETN_Msk* = (0x0000001F shl OPAMP2_CSR_TRIMOFFSETN_Pos) ## !< 0x1F000000
  OPAMP2_CSR_TRIMOFFSETN* = OPAMP2_CSR_TRIMOFFSETN_Msk
  OPAMP2_CSR_TSTREF_Pos* = (29)
  OPAMP2_CSR_TSTREF_Msk* = (0x00000001 shl OPAMP2_CSR_TSTREF_Pos) ## !< 0x20000000
  OPAMP2_CSR_TSTREF* = OPAMP2_CSR_TSTREF_Msk
  OPAMP2_CSR_OUTCAL_Pos* = (30)
  OPAMP2_CSR_OUTCAL_Msk* = (0x00000001 shl OPAMP2_CSR_OUTCAL_Pos) ## !< 0x40000000
  OPAMP2_CSR_OUTCAL* = OPAMP2_CSR_OUTCAL_Msk
  OPAMP2_CSR_LOCK_Pos* = (31)
  OPAMP2_CSR_LOCK_Msk* = (0x00000001 shl OPAMP2_CSR_LOCK_Pos) ## !< 0x80000000
  OPAMP2_CSR_LOCK* = OPAMP2_CSR_LOCK_Msk

## ********************  Bit definition for OPAMP3_CSR register  **************

const
  OPAMP3_CSR_OPAMP3EN_Pos* = (0)
  OPAMP3_CSR_OPAMP3EN_Msk* = (0x00000001 shl OPAMP3_CSR_OPAMP3EN_Pos) ## !< 0x00000001
  OPAMP3_CSR_OPAMP3EN* = OPAMP3_CSR_OPAMP3EN_Msk
  OPAMP3_CSR_FORCEVP_Pos* = (1)
  OPAMP3_CSR_FORCEVP_Msk* = (0x00000001 shl OPAMP3_CSR_FORCEVP_Pos) ## !< 0x00000002
  OPAMP3_CSR_FORCEVP* = OPAMP3_CSR_FORCEVP_Msk
  OPAMP3_CSR_VPSEL_Pos* = (2)
  OPAMP3_CSR_VPSEL_Msk* = (0x00000003 shl OPAMP3_CSR_VPSEL_Pos) ## !< 0x0000000C
  OPAMP3_CSR_VPSEL* = OPAMP3_CSR_VPSEL_Msk
  OPAMP3_CSR_VPSEL_0* = (0x00000001 shl OPAMP3_CSR_VPSEL_Pos) ## !< 0x00000004
  OPAMP3_CSR_VPSEL_1* = (0x00000002 shl OPAMP3_CSR_VPSEL_Pos) ## !< 0x00000008
  OPAMP3_CSR_VMSEL_Pos* = (5)
  OPAMP3_CSR_VMSEL_Msk* = (0x00000003 shl OPAMP3_CSR_VMSEL_Pos) ## !< 0x00000060
  OPAMP3_CSR_VMSEL* = OPAMP3_CSR_VMSEL_Msk
  OPAMP3_CSR_VMSEL_0* = (0x00000001 shl OPAMP3_CSR_VMSEL_Pos) ## !< 0x00000020
  OPAMP3_CSR_VMSEL_1* = (0x00000002 shl OPAMP3_CSR_VMSEL_Pos) ## !< 0x00000040
  OPAMP3_CSR_TCMEN_Pos* = (7)
  OPAMP3_CSR_TCMEN_Msk* = (0x00000001 shl OPAMP3_CSR_TCMEN_Pos) ## !< 0x00000080
  OPAMP3_CSR_TCMEN* = OPAMP3_CSR_TCMEN_Msk
  OPAMP3_CSR_VMSSEL_Pos* = (8)
  OPAMP3_CSR_VMSSEL_Msk* = (0x00000001 shl OPAMP3_CSR_VMSSEL_Pos) ## !< 0x00000100
  OPAMP3_CSR_VMSSEL* = OPAMP3_CSR_VMSSEL_Msk
  OPAMP3_CSR_VPSSEL_Pos* = (9)
  OPAMP3_CSR_VPSSEL_Msk* = (0x00000003 shl OPAMP3_CSR_VPSSEL_Pos) ## !< 0x00000600
  OPAMP3_CSR_VPSSEL* = OPAMP3_CSR_VPSSEL_Msk
  OPAMP3_CSR_VPSSEL_0* = (0x00000001 shl OPAMP3_CSR_VPSSEL_Pos) ## !< 0x00000200
  OPAMP3_CSR_VPSSEL_1* = (0x00000002 shl OPAMP3_CSR_VPSSEL_Pos) ## !< 0x00000400
  OPAMP3_CSR_CALON_Pos* = (11)
  OPAMP3_CSR_CALON_Msk* = (0x00000001 shl OPAMP3_CSR_CALON_Pos) ## !< 0x00000800
  OPAMP3_CSR_CALON* = OPAMP3_CSR_CALON_Msk
  OPAMP3_CSR_CALSEL_Pos* = (12)
  OPAMP3_CSR_CALSEL_Msk* = (0x00000003 shl OPAMP3_CSR_CALSEL_Pos) ## !< 0x00003000
  OPAMP3_CSR_CALSEL* = OPAMP3_CSR_CALSEL_Msk
  OPAMP3_CSR_CALSEL_0* = (0x00000001 shl OPAMP3_CSR_CALSEL_Pos) ## !< 0x00001000
  OPAMP3_CSR_CALSEL_1* = (0x00000002 shl OPAMP3_CSR_CALSEL_Pos) ## !< 0x00002000
  OPAMP3_CSR_PGGAIN_Pos* = (14)
  OPAMP3_CSR_PGGAIN_Msk* = (0x0000000F shl OPAMP3_CSR_PGGAIN_Pos) ## !< 0x0003C000
  OPAMP3_CSR_PGGAIN* = OPAMP3_CSR_PGGAIN_Msk
  OPAMP3_CSR_PGGAIN_0* = (0x00000001 shl OPAMP3_CSR_PGGAIN_Pos) ## !< 0x00004000
  OPAMP3_CSR_PGGAIN_1* = (0x00000002 shl OPAMP3_CSR_PGGAIN_Pos) ## !< 0x00008000
  OPAMP3_CSR_PGGAIN_2* = (0x00000004 shl OPAMP3_CSR_PGGAIN_Pos) ## !< 0x00010000
  OPAMP3_CSR_PGGAIN_3* = (0x00000008 shl OPAMP3_CSR_PGGAIN_Pos) ## !< 0x00020000
  OPAMP3_CSR_USERTRIM_Pos* = (18)
  OPAMP3_CSR_USERTRIM_Msk* = (0x00000001 shl OPAMP3_CSR_USERTRIM_Pos) ## !< 0x00040000
  OPAMP3_CSR_USERTRIM* = OPAMP3_CSR_USERTRIM_Msk
  OPAMP3_CSR_TRIMOFFSETP_Pos* = (19)
  OPAMP3_CSR_TRIMOFFSETP_Msk* = (0x0000001F shl OPAMP3_CSR_TRIMOFFSETP_Pos) ## !< 0x00F80000
  OPAMP3_CSR_TRIMOFFSETP* = OPAMP3_CSR_TRIMOFFSETP_Msk
  OPAMP3_CSR_TRIMOFFSETN_Pos* = (24)
  OPAMP3_CSR_TRIMOFFSETN_Msk* = (0x0000001F shl OPAMP3_CSR_TRIMOFFSETN_Pos) ## !< 0x1F000000
  OPAMP3_CSR_TRIMOFFSETN* = OPAMP3_CSR_TRIMOFFSETN_Msk
  OPAMP3_CSR_TSTREF_Pos* = (29)
  OPAMP3_CSR_TSTREF_Msk* = (0x00000001 shl OPAMP3_CSR_TSTREF_Pos) ## !< 0x20000000
  OPAMP3_CSR_TSTREF* = OPAMP3_CSR_TSTREF_Msk
  OPAMP3_CSR_OUTCAL_Pos* = (30)
  OPAMP3_CSR_OUTCAL_Msk* = (0x00000001 shl OPAMP3_CSR_OUTCAL_Pos) ## !< 0x40000000
  OPAMP3_CSR_OUTCAL* = OPAMP3_CSR_OUTCAL_Msk
  OPAMP3_CSR_LOCK_Pos* = (31)
  OPAMP3_CSR_LOCK_Msk* = (0x00000001 shl OPAMP3_CSR_LOCK_Pos) ## !< 0x80000000
  OPAMP3_CSR_LOCK* = OPAMP3_CSR_LOCK_Msk

## ********************  Bit definition for OPAMP4_CSR register  **************

const
  OPAMP4_CSR_OPAMP4EN_Pos* = (0)
  OPAMP4_CSR_OPAMP4EN_Msk* = (0x00000001 shl OPAMP4_CSR_OPAMP4EN_Pos) ## !< 0x00000001
  OPAMP4_CSR_OPAMP4EN* = OPAMP4_CSR_OPAMP4EN_Msk
  OPAMP4_CSR_FORCEVP_Pos* = (1)
  OPAMP4_CSR_FORCEVP_Msk* = (0x00000001 shl OPAMP4_CSR_FORCEVP_Pos) ## !< 0x00000002
  OPAMP4_CSR_FORCEVP* = OPAMP4_CSR_FORCEVP_Msk
  OPAMP4_CSR_VPSEL_Pos* = (2)
  OPAMP4_CSR_VPSEL_Msk* = (0x00000003 shl OPAMP4_CSR_VPSEL_Pos) ## !< 0x0000000C
  OPAMP4_CSR_VPSEL* = OPAMP4_CSR_VPSEL_Msk
  OPAMP4_CSR_VPSEL_0* = (0x00000001 shl OPAMP4_CSR_VPSEL_Pos) ## !< 0x00000004
  OPAMP4_CSR_VPSEL_1* = (0x00000002 shl OPAMP4_CSR_VPSEL_Pos) ## !< 0x00000008
  OPAMP4_CSR_VMSEL_Pos* = (5)
  OPAMP4_CSR_VMSEL_Msk* = (0x00000003 shl OPAMP4_CSR_VMSEL_Pos) ## !< 0x00000060
  OPAMP4_CSR_VMSEL* = OPAMP4_CSR_VMSEL_Msk
  OPAMP4_CSR_VMSEL_0* = (0x00000001 shl OPAMP4_CSR_VMSEL_Pos) ## !< 0x00000020
  OPAMP4_CSR_VMSEL_1* = (0x00000002 shl OPAMP4_CSR_VMSEL_Pos) ## !< 0x00000040
  OPAMP4_CSR_TCMEN_Pos* = (7)
  OPAMP4_CSR_TCMEN_Msk* = (0x00000001 shl OPAMP4_CSR_TCMEN_Pos) ## !< 0x00000080
  OPAMP4_CSR_TCMEN* = OPAMP4_CSR_TCMEN_Msk
  OPAMP4_CSR_VMSSEL_Pos* = (8)
  OPAMP4_CSR_VMSSEL_Msk* = (0x00000001 shl OPAMP4_CSR_VMSSEL_Pos) ## !< 0x00000100
  OPAMP4_CSR_VMSSEL* = OPAMP4_CSR_VMSSEL_Msk
  OPAMP4_CSR_VPSSEL_Pos* = (9)
  OPAMP4_CSR_VPSSEL_Msk* = (0x00000003 shl OPAMP4_CSR_VPSSEL_Pos) ## !< 0x00000600
  OPAMP4_CSR_VPSSEL* = OPAMP4_CSR_VPSSEL_Msk
  OPAMP4_CSR_VPSSEL_0* = (0x00000001 shl OPAMP4_CSR_VPSSEL_Pos) ## !< 0x00000200
  OPAMP4_CSR_VPSSEL_1* = (0x00000002 shl OPAMP4_CSR_VPSSEL_Pos) ## !< 0x00000400
  OPAMP4_CSR_CALON_Pos* = (11)
  OPAMP4_CSR_CALON_Msk* = (0x00000001 shl OPAMP4_CSR_CALON_Pos) ## !< 0x00000800
  OPAMP4_CSR_CALON* = OPAMP4_CSR_CALON_Msk
  OPAMP4_CSR_CALSEL_Pos* = (12)
  OPAMP4_CSR_CALSEL_Msk* = (0x00000003 shl OPAMP4_CSR_CALSEL_Pos) ## !< 0x00003000
  OPAMP4_CSR_CALSEL* = OPAMP4_CSR_CALSEL_Msk
  OPAMP4_CSR_CALSEL_0* = (0x00000001 shl OPAMP4_CSR_CALSEL_Pos) ## !< 0x00001000
  OPAMP4_CSR_CALSEL_1* = (0x00000002 shl OPAMP4_CSR_CALSEL_Pos) ## !< 0x00002000
  OPAMP4_CSR_PGGAIN_Pos* = (14)
  OPAMP4_CSR_PGGAIN_Msk* = (0x0000000F shl OPAMP4_CSR_PGGAIN_Pos) ## !< 0x0003C000
  OPAMP4_CSR_PGGAIN* = OPAMP4_CSR_PGGAIN_Msk
  OPAMP4_CSR_PGGAIN_0* = (0x00000001 shl OPAMP4_CSR_PGGAIN_Pos) ## !< 0x00004000
  OPAMP4_CSR_PGGAIN_1* = (0x00000002 shl OPAMP4_CSR_PGGAIN_Pos) ## !< 0x00008000
  OPAMP4_CSR_PGGAIN_2* = (0x00000004 shl OPAMP4_CSR_PGGAIN_Pos) ## !< 0x00010000
  OPAMP4_CSR_PGGAIN_3* = (0x00000008 shl OPAMP4_CSR_PGGAIN_Pos) ## !< 0x00020000
  OPAMP4_CSR_USERTRIM_Pos* = (18)
  OPAMP4_CSR_USERTRIM_Msk* = (0x00000001 shl OPAMP4_CSR_USERTRIM_Pos) ## !< 0x00040000
  OPAMP4_CSR_USERTRIM* = OPAMP4_CSR_USERTRIM_Msk
  OPAMP4_CSR_TRIMOFFSETP_Pos* = (19)
  OPAMP4_CSR_TRIMOFFSETP_Msk* = (0x0000001F shl OPAMP4_CSR_TRIMOFFSETP_Pos) ## !< 0x00F80000
  OPAMP4_CSR_TRIMOFFSETP* = OPAMP4_CSR_TRIMOFFSETP_Msk
  OPAMP4_CSR_TRIMOFFSETN_Pos* = (24)
  OPAMP4_CSR_TRIMOFFSETN_Msk* = (0x0000001F shl OPAMP4_CSR_TRIMOFFSETN_Pos) ## !< 0x1F000000
  OPAMP4_CSR_TRIMOFFSETN* = OPAMP4_CSR_TRIMOFFSETN_Msk
  OPAMP4_CSR_TSTREF_Pos* = (29)
  OPAMP4_CSR_TSTREF_Msk* = (0x00000001 shl OPAMP4_CSR_TSTREF_Pos) ## !< 0x20000000
  OPAMP4_CSR_TSTREF* = OPAMP4_CSR_TSTREF_Msk
  OPAMP4_CSR_OUTCAL_Pos* = (30)
  OPAMP4_CSR_OUTCAL_Msk* = (0x00000001 shl OPAMP4_CSR_OUTCAL_Pos) ## !< 0x40000000
  OPAMP4_CSR_OUTCAL* = OPAMP4_CSR_OUTCAL_Msk
  OPAMP4_CSR_LOCK_Pos* = (31)
  OPAMP4_CSR_LOCK_Msk* = (0x00000001 shl OPAMP4_CSR_LOCK_Pos) ## !< 0x80000000
  OPAMP4_CSR_LOCK* = OPAMP4_CSR_LOCK_Msk

## ********************  Bit definition for OPAMPx_CSR register  **************

const
  OPAMP_CSR_OPAMPxEN_Pos* = (0)
  OPAMP_CSR_OPAMPxEN_Msk* = (0x00000001 shl OPAMP_CSR_OPAMPxEN_Pos) ## !< 0x00000001
  OPAMP_CSR_OPAMPxEN* = OPAMP_CSR_OPAMPxEN_Msk
  OPAMP_CSR_FORCEVP_Pos* = (1)
  OPAMP_CSR_FORCEVP_Msk* = (0x00000001 shl OPAMP_CSR_FORCEVP_Pos) ## !< 0x00000002
  OPAMP_CSR_FORCEVP* = OPAMP_CSR_FORCEVP_Msk
  OPAMP_CSR_VPSEL_Pos* = (2)
  OPAMP_CSR_VPSEL_Msk* = (0x00000003 shl OPAMP_CSR_VPSEL_Pos) ## !< 0x0000000C
  OPAMP_CSR_VPSEL* = OPAMP_CSR_VPSEL_Msk
  OPAMP_CSR_VPSEL_0* = (0x00000001 shl OPAMP_CSR_VPSEL_Pos) ## !< 0x00000004
  OPAMP_CSR_VPSEL_1* = (0x00000002 shl OPAMP_CSR_VPSEL_Pos) ## !< 0x00000008
  OPAMP_CSR_VMSEL_Pos* = (5)
  OPAMP_CSR_VMSEL_Msk* = (0x00000003 shl OPAMP_CSR_VMSEL_Pos) ## !< 0x00000060
  OPAMP_CSR_VMSEL* = OPAMP_CSR_VMSEL_Msk
  OPAMP_CSR_VMSEL_0* = (0x00000001 shl OPAMP_CSR_VMSEL_Pos) ## !< 0x00000020
  OPAMP_CSR_VMSEL_1* = (0x00000002 shl OPAMP_CSR_VMSEL_Pos) ## !< 0x00000040
  OPAMP_CSR_TCMEN_Pos* = (7)
  OPAMP_CSR_TCMEN_Msk* = (0x00000001 shl OPAMP_CSR_TCMEN_Pos) ## !< 0x00000080
  OPAMP_CSR_TCMEN* = OPAMP_CSR_TCMEN_Msk
  OPAMP_CSR_VMSSEL_Pos* = (8)
  OPAMP_CSR_VMSSEL_Msk* = (0x00000001 shl OPAMP_CSR_VMSSEL_Pos) ## !< 0x00000100
  OPAMP_CSR_VMSSEL* = OPAMP_CSR_VMSSEL_Msk
  OPAMP_CSR_VPSSEL_Pos* = (9)
  OPAMP_CSR_VPSSEL_Msk* = (0x00000003 shl OPAMP_CSR_VPSSEL_Pos) ## !< 0x00000600
  OPAMP_CSR_VPSSEL* = OPAMP_CSR_VPSSEL_Msk
  OPAMP_CSR_VPSSEL_0* = (0x00000001 shl OPAMP_CSR_VPSSEL_Pos) ## !< 0x00000200
  OPAMP_CSR_VPSSEL_1* = (0x00000002 shl OPAMP_CSR_VPSSEL_Pos) ## !< 0x00000400
  OPAMP_CSR_CALON_Pos* = (11)
  OPAMP_CSR_CALON_Msk* = (0x00000001 shl OPAMP_CSR_CALON_Pos) ## !< 0x00000800
  OPAMP_CSR_CALON* = OPAMP_CSR_CALON_Msk
  OPAMP_CSR_CALSEL_Pos* = (12)
  OPAMP_CSR_CALSEL_Msk* = (0x00000003 shl OPAMP_CSR_CALSEL_Pos) ## !< 0x00003000
  OPAMP_CSR_CALSEL* = OPAMP_CSR_CALSEL_Msk
  OPAMP_CSR_CALSEL_0* = (0x00000001 shl OPAMP_CSR_CALSEL_Pos) ## !< 0x00001000
  OPAMP_CSR_CALSEL_1* = (0x00000002 shl OPAMP_CSR_CALSEL_Pos) ## !< 0x00002000
  OPAMP_CSR_PGGAIN_Pos* = (14)
  OPAMP_CSR_PGGAIN_Msk* = (0x0000000F shl OPAMP_CSR_PGGAIN_Pos) ## !< 0x0003C000
  OPAMP_CSR_PGGAIN* = OPAMP_CSR_PGGAIN_Msk
  OPAMP_CSR_PGGAIN_0* = (0x00000001 shl OPAMP_CSR_PGGAIN_Pos) ## !< 0x00004000
  OPAMP_CSR_PGGAIN_1* = (0x00000002 shl OPAMP_CSR_PGGAIN_Pos) ## !< 0x00008000
  OPAMP_CSR_PGGAIN_2* = (0x00000004 shl OPAMP_CSR_PGGAIN_Pos) ## !< 0x00010000
  OPAMP_CSR_PGGAIN_3* = (0x00000008 shl OPAMP_CSR_PGGAIN_Pos) ## !< 0x00020000
  OPAMP_CSR_USERTRIM_Pos* = (18)
  OPAMP_CSR_USERTRIM_Msk* = (0x00000001 shl OPAMP_CSR_USERTRIM_Pos) ## !< 0x00040000
  OPAMP_CSR_USERTRIM* = OPAMP_CSR_USERTRIM_Msk
  OPAMP_CSR_TRIMOFFSETP_Pos* = (19)
  OPAMP_CSR_TRIMOFFSETP_Msk* = (0x0000001F shl OPAMP_CSR_TRIMOFFSETP_Pos) ## !< 0x00F80000
  OPAMP_CSR_TRIMOFFSETP* = OPAMP_CSR_TRIMOFFSETP_Msk
  OPAMP_CSR_TRIMOFFSETN_Pos* = (24)
  OPAMP_CSR_TRIMOFFSETN_Msk* = (0x0000001F shl OPAMP_CSR_TRIMOFFSETN_Pos) ## !< 0x1F000000
  OPAMP_CSR_TRIMOFFSETN* = OPAMP_CSR_TRIMOFFSETN_Msk
  OPAMP_CSR_TSTREF_Pos* = (29)
  OPAMP_CSR_TSTREF_Msk* = (0x00000001 shl OPAMP_CSR_TSTREF_Pos) ## !< 0x20000000
  OPAMP_CSR_TSTREF* = OPAMP_CSR_TSTREF_Msk
  OPAMP_CSR_OUTCAL_Pos* = (30)
  OPAMP_CSR_OUTCAL_Msk* = (0x00000001 shl OPAMP_CSR_OUTCAL_Pos) ## !< 0x40000000
  OPAMP_CSR_OUTCAL* = OPAMP_CSR_OUTCAL_Msk
  OPAMP_CSR_LOCK_Pos* = (31)
  OPAMP_CSR_LOCK_Msk* = (0x00000001 shl OPAMP_CSR_LOCK_Pos) ## !< 0x80000000
  OPAMP_CSR_LOCK* = OPAMP_CSR_LOCK_Msk

## ****************************************************************************
##
##                    Controller Area Network (CAN )
##
## ****************************************************************************
## ******************  Bit definition for CAN_MCR register  *******************

const
  CAN_MCR_INRQ_Pos* = (0)
  CAN_MCR_INRQ_Msk* = (0x00000001 shl CAN_MCR_INRQ_Pos) ## !< 0x00000001
  CAN_MCR_INRQ* = CAN_MCR_INRQ_Msk
  CAN_MCR_SLEEP_Pos* = (1)
  CAN_MCR_SLEEP_Msk* = (0x00000001 shl CAN_MCR_SLEEP_Pos) ## !< 0x00000002
  CAN_MCR_SLEEP* = CAN_MCR_SLEEP_Msk
  CAN_MCR_TXFP_Pos* = (2)
  CAN_MCR_TXFP_Msk* = (0x00000001 shl CAN_MCR_TXFP_Pos) ## !< 0x00000004
  CAN_MCR_TXFP* = CAN_MCR_TXFP_Msk
  CAN_MCR_RFLM_Pos* = (3)
  CAN_MCR_RFLM_Msk* = (0x00000001 shl CAN_MCR_RFLM_Pos) ## !< 0x00000008
  CAN_MCR_RFLM* = CAN_MCR_RFLM_Msk
  CAN_MCR_NART_Pos* = (4)
  CAN_MCR_NART_Msk* = (0x00000001 shl CAN_MCR_NART_Pos) ## !< 0x00000010
  CAN_MCR_NART* = CAN_MCR_NART_Msk
  CAN_MCR_AWUM_Pos* = (5)
  CAN_MCR_AWUM_Msk* = (0x00000001 shl CAN_MCR_AWUM_Pos) ## !< 0x00000020
  CAN_MCR_AWUM* = CAN_MCR_AWUM_Msk
  CAN_MCR_ABOM_Pos* = (6)
  CAN_MCR_ABOM_Msk* = (0x00000001 shl CAN_MCR_ABOM_Pos) ## !< 0x00000040
  CAN_MCR_ABOM* = CAN_MCR_ABOM_Msk
  CAN_MCR_TTCM_Pos* = (7)
  CAN_MCR_TTCM_Msk* = (0x00000001 shl CAN_MCR_TTCM_Pos) ## !< 0x00000080
  CAN_MCR_TTCM* = CAN_MCR_TTCM_Msk
  CAN_MCR_RESET_Pos* = (15)
  CAN_MCR_RESET_Msk* = (0x00000001 shl CAN_MCR_RESET_Pos) ## !< 0x00008000
  CAN_MCR_RESET* = CAN_MCR_RESET_Msk

## ******************  Bit definition for CAN_MSR register  *******************

const
  CAN_MSR_INAK_Pos* = (0)
  CAN_MSR_INAK_Msk* = (0x00000001 shl CAN_MSR_INAK_Pos) ## !< 0x00000001
  CAN_MSR_INAK* = CAN_MSR_INAK_Msk
  CAN_MSR_SLAK_Pos* = (1)
  CAN_MSR_SLAK_Msk* = (0x00000001 shl CAN_MSR_SLAK_Pos) ## !< 0x00000002
  CAN_MSR_SLAK* = CAN_MSR_SLAK_Msk
  CAN_MSR_ERRI_Pos* = (2)
  CAN_MSR_ERRI_Msk* = (0x00000001 shl CAN_MSR_ERRI_Pos) ## !< 0x00000004
  CAN_MSR_ERRI* = CAN_MSR_ERRI_Msk
  CAN_MSR_WKUI_Pos* = (3)
  CAN_MSR_WKUI_Msk* = (0x00000001 shl CAN_MSR_WKUI_Pos) ## !< 0x00000008
  CAN_MSR_WKUI* = CAN_MSR_WKUI_Msk
  CAN_MSR_SLAKI_Pos* = (4)
  CAN_MSR_SLAKI_Msk* = (0x00000001 shl CAN_MSR_SLAKI_Pos) ## !< 0x00000010
  CAN_MSR_SLAKI* = CAN_MSR_SLAKI_Msk
  CAN_MSR_TXM_Pos* = (8)
  CAN_MSR_TXM_Msk* = (0x00000001 shl CAN_MSR_TXM_Pos) ## !< 0x00000100
  CAN_MSR_TXM* = CAN_MSR_TXM_Msk
  CAN_MSR_RXM_Pos* = (9)
  CAN_MSR_RXM_Msk* = (0x00000001 shl CAN_MSR_RXM_Pos) ## !< 0x00000200
  CAN_MSR_RXM* = CAN_MSR_RXM_Msk
  CAN_MSR_SAMP_Pos* = (10)
  CAN_MSR_SAMP_Msk* = (0x00000001 shl CAN_MSR_SAMP_Pos) ## !< 0x00000400
  CAN_MSR_SAMP* = CAN_MSR_SAMP_Msk
  CAN_MSR_RX_Pos* = (11)
  CAN_MSR_RX_Msk* = (0x00000001 shl CAN_MSR_RX_Pos) ## !< 0x00000800
  CAN_MSR_RX* = CAN_MSR_RX_Msk

## ******************  Bit definition for CAN_TSR register  *******************

const
  CAN_TSR_RQCP0_Pos* = (0)
  CAN_TSR_RQCP0_Msk* = (0x00000001 shl CAN_TSR_RQCP0_Pos) ## !< 0x00000001
  CAN_TSR_RQCP0* = CAN_TSR_RQCP0_Msk
  CAN_TSR_TXOK0_Pos* = (1)
  CAN_TSR_TXOK0_Msk* = (0x00000001 shl CAN_TSR_TXOK0_Pos) ## !< 0x00000002
  CAN_TSR_TXOK0* = CAN_TSR_TXOK0_Msk
  CAN_TSR_ALST0_Pos* = (2)
  CAN_TSR_ALST0_Msk* = (0x00000001 shl CAN_TSR_ALST0_Pos) ## !< 0x00000004
  CAN_TSR_ALST0* = CAN_TSR_ALST0_Msk
  CAN_TSR_TERR0_Pos* = (3)
  CAN_TSR_TERR0_Msk* = (0x00000001 shl CAN_TSR_TERR0_Pos) ## !< 0x00000008
  CAN_TSR_TERR0* = CAN_TSR_TERR0_Msk
  CAN_TSR_ABRQ0_Pos* = (7)
  CAN_TSR_ABRQ0_Msk* = (0x00000001 shl CAN_TSR_ABRQ0_Pos) ## !< 0x00000080
  CAN_TSR_ABRQ0* = CAN_TSR_ABRQ0_Msk
  CAN_TSR_RQCP1_Pos* = (8)
  CAN_TSR_RQCP1_Msk* = (0x00000001 shl CAN_TSR_RQCP1_Pos) ## !< 0x00000100
  CAN_TSR_RQCP1* = CAN_TSR_RQCP1_Msk
  CAN_TSR_TXOK1_Pos* = (9)
  CAN_TSR_TXOK1_Msk* = (0x00000001 shl CAN_TSR_TXOK1_Pos) ## !< 0x00000200
  CAN_TSR_TXOK1* = CAN_TSR_TXOK1_Msk
  CAN_TSR_ALST1_Pos* = (10)
  CAN_TSR_ALST1_Msk* = (0x00000001 shl CAN_TSR_ALST1_Pos) ## !< 0x00000400
  CAN_TSR_ALST1* = CAN_TSR_ALST1_Msk
  CAN_TSR_TERR1_Pos* = (11)
  CAN_TSR_TERR1_Msk* = (0x00000001 shl CAN_TSR_TERR1_Pos) ## !< 0x00000800
  CAN_TSR_TERR1* = CAN_TSR_TERR1_Msk
  CAN_TSR_ABRQ1_Pos* = (15)
  CAN_TSR_ABRQ1_Msk* = (0x00000001 shl CAN_TSR_ABRQ1_Pos) ## !< 0x00008000
  CAN_TSR_ABRQ1* = CAN_TSR_ABRQ1_Msk
  CAN_TSR_RQCP2_Pos* = (16)
  CAN_TSR_RQCP2_Msk* = (0x00000001 shl CAN_TSR_RQCP2_Pos) ## !< 0x00010000
  CAN_TSR_RQCP2* = CAN_TSR_RQCP2_Msk
  CAN_TSR_TXOK2_Pos* = (17)
  CAN_TSR_TXOK2_Msk* = (0x00000001 shl CAN_TSR_TXOK2_Pos) ## !< 0x00020000
  CAN_TSR_TXOK2* = CAN_TSR_TXOK2_Msk
  CAN_TSR_ALST2_Pos* = (18)
  CAN_TSR_ALST2_Msk* = (0x00000001 shl CAN_TSR_ALST2_Pos) ## !< 0x00040000
  CAN_TSR_ALST2* = CAN_TSR_ALST2_Msk
  CAN_TSR_TERR2_Pos* = (19)
  CAN_TSR_TERR2_Msk* = (0x00000001 shl CAN_TSR_TERR2_Pos) ## !< 0x00080000
  CAN_TSR_TERR2* = CAN_TSR_TERR2_Msk
  CAN_TSR_ABRQ2_Pos* = (23)
  CAN_TSR_ABRQ2_Msk* = (0x00000001 shl CAN_TSR_ABRQ2_Pos) ## !< 0x00800000
  CAN_TSR_ABRQ2* = CAN_TSR_ABRQ2_Msk
  CAN_TSR_CODE_Pos* = (24)
  CAN_TSR_CODE_Msk* = (0x00000003 shl CAN_TSR_CODE_Pos) ## !< 0x03000000
  CAN_TSR_CODE* = CAN_TSR_CODE_Msk
  CAN_TSR_TME_Pos* = (26)
  CAN_TSR_TME_Msk* = (0x00000007 shl CAN_TSR_TME_Pos) ## !< 0x1C000000
  CAN_TSR_TME* = CAN_TSR_TME_Msk
  CAN_TSR_TME0_Pos* = (26)
  CAN_TSR_TME0_Msk* = (0x00000001 shl CAN_TSR_TME0_Pos) ## !< 0x04000000
  CAN_TSR_TME0* = CAN_TSR_TME0_Msk
  CAN_TSR_TME1_Pos* = (27)
  CAN_TSR_TME1_Msk* = (0x00000001 shl CAN_TSR_TME1_Pos) ## !< 0x08000000
  CAN_TSR_TME1* = CAN_TSR_TME1_Msk
  CAN_TSR_TME2_Pos* = (28)
  CAN_TSR_TME2_Msk* = (0x00000001 shl CAN_TSR_TME2_Pos) ## !< 0x10000000
  CAN_TSR_TME2* = CAN_TSR_TME2_Msk
  CAN_TSR_LOW_Pos* = (29)
  CAN_TSR_LOW_Msk* = (0x00000007 shl CAN_TSR_LOW_Pos) ## !< 0xE0000000
  CAN_TSR_LOW* = CAN_TSR_LOW_Msk
  CAN_TSR_LOW0_Pos* = (29)
  CAN_TSR_LOW0_Msk* = (0x00000001 shl CAN_TSR_LOW0_Pos) ## !< 0x20000000
  CAN_TSR_LOW0* = CAN_TSR_LOW0_Msk
  CAN_TSR_LOW1_Pos* = (30)
  CAN_TSR_LOW1_Msk* = (0x00000001 shl CAN_TSR_LOW1_Pos) ## !< 0x40000000
  CAN_TSR_LOW1* = CAN_TSR_LOW1_Msk
  CAN_TSR_LOW2_Pos* = (31)
  CAN_TSR_LOW2_Msk* = (0x00000001 shl CAN_TSR_LOW2_Pos) ## !< 0x80000000
  CAN_TSR_LOW2* = CAN_TSR_LOW2_Msk

## ******************  Bit definition for CAN_RF0R register  ******************

const
  CAN_RF0R_FMP0_Pos* = (0)
  CAN_RF0R_FMP0_Msk* = (0x00000003 shl CAN_RF0R_FMP0_Pos) ## !< 0x00000003
  CAN_RF0R_FMP0* = CAN_RF0R_FMP0_Msk
  CAN_RF0R_FULL0_Pos* = (3)
  CAN_RF0R_FULL0_Msk* = (0x00000001 shl CAN_RF0R_FULL0_Pos) ## !< 0x00000008
  CAN_RF0R_FULL0* = CAN_RF0R_FULL0_Msk
  CAN_RF0R_FOVR0_Pos* = (4)
  CAN_RF0R_FOVR0_Msk* = (0x00000001 shl CAN_RF0R_FOVR0_Pos) ## !< 0x00000010
  CAN_RF0R_FOVR0* = CAN_RF0R_FOVR0_Msk
  CAN_RF0R_RFOM0_Pos* = (5)
  CAN_RF0R_RFOM0_Msk* = (0x00000001 shl CAN_RF0R_RFOM0_Pos) ## !< 0x00000020
  CAN_RF0R_RFOM0* = CAN_RF0R_RFOM0_Msk

## ******************  Bit definition for CAN_RF1R register  ******************

const
  CAN_RF1R_FMP1_Pos* = (0)
  CAN_RF1R_FMP1_Msk* = (0x00000003 shl CAN_RF1R_FMP1_Pos) ## !< 0x00000003
  CAN_RF1R_FMP1* = CAN_RF1R_FMP1_Msk
  CAN_RF1R_FULL1_Pos* = (3)
  CAN_RF1R_FULL1_Msk* = (0x00000001 shl CAN_RF1R_FULL1_Pos) ## !< 0x00000008
  CAN_RF1R_FULL1* = CAN_RF1R_FULL1_Msk
  CAN_RF1R_FOVR1_Pos* = (4)
  CAN_RF1R_FOVR1_Msk* = (0x00000001 shl CAN_RF1R_FOVR1_Pos) ## !< 0x00000010
  CAN_RF1R_FOVR1* = CAN_RF1R_FOVR1_Msk
  CAN_RF1R_RFOM1_Pos* = (5)
  CAN_RF1R_RFOM1_Msk* = (0x00000001 shl CAN_RF1R_RFOM1_Pos) ## !< 0x00000020
  CAN_RF1R_RFOM1* = CAN_RF1R_RFOM1_Msk

## *******************  Bit definition for CAN_IER register  ******************

const
  CAN_IER_TMEIE_Pos* = (0)
  CAN_IER_TMEIE_Msk* = (0x00000001 shl CAN_IER_TMEIE_Pos) ## !< 0x00000001
  CAN_IER_TMEIE* = CAN_IER_TMEIE_Msk
  CAN_IER_FMPIE0_Pos* = (1)
  CAN_IER_FMPIE0_Msk* = (0x00000001 shl CAN_IER_FMPIE0_Pos) ## !< 0x00000002
  CAN_IER_FMPIE0* = CAN_IER_FMPIE0_Msk
  CAN_IER_FFIE0_Pos* = (2)
  CAN_IER_FFIE0_Msk* = (0x00000001 shl CAN_IER_FFIE0_Pos) ## !< 0x00000004
  CAN_IER_FFIE0* = CAN_IER_FFIE0_Msk
  CAN_IER_FOVIE0_Pos* = (3)
  CAN_IER_FOVIE0_Msk* = (0x00000001 shl CAN_IER_FOVIE0_Pos) ## !< 0x00000008
  CAN_IER_FOVIE0* = CAN_IER_FOVIE0_Msk
  CAN_IER_FMPIE1_Pos* = (4)
  CAN_IER_FMPIE1_Msk* = (0x00000001 shl CAN_IER_FMPIE1_Pos) ## !< 0x00000010
  CAN_IER_FMPIE1* = CAN_IER_FMPIE1_Msk
  CAN_IER_FFIE1_Pos* = (5)
  CAN_IER_FFIE1_Msk* = (0x00000001 shl CAN_IER_FFIE1_Pos) ## !< 0x00000020
  CAN_IER_FFIE1* = CAN_IER_FFIE1_Msk
  CAN_IER_FOVIE1_Pos* = (6)
  CAN_IER_FOVIE1_Msk* = (0x00000001 shl CAN_IER_FOVIE1_Pos) ## !< 0x00000040
  CAN_IER_FOVIE1* = CAN_IER_FOVIE1_Msk
  CAN_IER_EWGIE_Pos* = (8)
  CAN_IER_EWGIE_Msk* = (0x00000001 shl CAN_IER_EWGIE_Pos) ## !< 0x00000100
  CAN_IER_EWGIE* = CAN_IER_EWGIE_Msk
  CAN_IER_EPVIE_Pos* = (9)
  CAN_IER_EPVIE_Msk* = (0x00000001 shl CAN_IER_EPVIE_Pos) ## !< 0x00000200
  CAN_IER_EPVIE* = CAN_IER_EPVIE_Msk
  CAN_IER_BOFIE_Pos* = (10)
  CAN_IER_BOFIE_Msk* = (0x00000001 shl CAN_IER_BOFIE_Pos) ## !< 0x00000400
  CAN_IER_BOFIE* = CAN_IER_BOFIE_Msk
  CAN_IER_LECIE_Pos* = (11)
  CAN_IER_LECIE_Msk* = (0x00000001 shl CAN_IER_LECIE_Pos) ## !< 0x00000800
  CAN_IER_LECIE* = CAN_IER_LECIE_Msk
  CAN_IER_ERRIE_Pos* = (15)
  CAN_IER_ERRIE_Msk* = (0x00000001 shl CAN_IER_ERRIE_Pos) ## !< 0x00008000
  CAN_IER_ERRIE* = CAN_IER_ERRIE_Msk
  CAN_IER_WKUIE_Pos* = (16)
  CAN_IER_WKUIE_Msk* = (0x00000001 shl CAN_IER_WKUIE_Pos) ## !< 0x00010000
  CAN_IER_WKUIE* = CAN_IER_WKUIE_Msk
  CAN_IER_SLKIE_Pos* = (17)
  CAN_IER_SLKIE_Msk* = (0x00000001 shl CAN_IER_SLKIE_Pos) ## !< 0x00020000
  CAN_IER_SLKIE* = CAN_IER_SLKIE_Msk

## *******************  Bit definition for CAN_ESR register  ******************

const
  CAN_ESR_EWGF_Pos* = (0)
  CAN_ESR_EWGF_Msk* = (0x00000001 shl CAN_ESR_EWGF_Pos) ## !< 0x00000001
  CAN_ESR_EWGF* = CAN_ESR_EWGF_Msk
  CAN_ESR_EPVF_Pos* = (1)
  CAN_ESR_EPVF_Msk* = (0x00000001 shl CAN_ESR_EPVF_Pos) ## !< 0x00000002
  CAN_ESR_EPVF* = CAN_ESR_EPVF_Msk
  CAN_ESR_BOFF_Pos* = (2)
  CAN_ESR_BOFF_Msk* = (0x00000001 shl CAN_ESR_BOFF_Pos) ## !< 0x00000004
  CAN_ESR_BOFF* = CAN_ESR_BOFF_Msk
  CAN_ESR_LEC_Pos* = (4)
  CAN_ESR_LEC_Msk* = (0x00000007 shl CAN_ESR_LEC_Pos) ## !< 0x00000070
  CAN_ESR_LEC* = CAN_ESR_LEC_Msk
  CAN_ESR_LEC_0* = (0x00000001 shl CAN_ESR_LEC_Pos) ## !< 0x00000010
  CAN_ESR_LEC_1* = (0x00000002 shl CAN_ESR_LEC_Pos) ## !< 0x00000020
  CAN_ESR_LEC_2* = (0x00000004 shl CAN_ESR_LEC_Pos) ## !< 0x00000040
  CAN_ESR_TEC_Pos* = (16)
  CAN_ESR_TEC_Msk* = (0x000000FF shl CAN_ESR_TEC_Pos) ## !< 0x00FF0000
  CAN_ESR_TEC* = CAN_ESR_TEC_Msk
  CAN_ESR_REC_Pos* = (24)
  CAN_ESR_REC_Msk* = (0x000000FF shl CAN_ESR_REC_Pos) ## !< 0xFF000000
  CAN_ESR_REC* = CAN_ESR_REC_Msk

## ******************  Bit definition for CAN_BTR register  *******************

const
  CAN_BTR_BRP_Pos* = (0)
  CAN_BTR_BRP_Msk* = (0x000003FF shl CAN_BTR_BRP_Pos) ## !< 0x000003FF
  CAN_BTR_BRP* = CAN_BTR_BRP_Msk
  CAN_BTR_TS1_Pos* = (16)
  CAN_BTR_TS1_Msk* = (0x0000000F shl CAN_BTR_TS1_Pos) ## !< 0x000F0000
  CAN_BTR_TS1* = CAN_BTR_TS1_Msk
  CAN_BTR_TS1_0* = (0x00000001 shl CAN_BTR_TS1_Pos) ## !< 0x00010000
  CAN_BTR_TS1_1* = (0x00000002 shl CAN_BTR_TS1_Pos) ## !< 0x00020000
  CAN_BTR_TS1_2* = (0x00000004 shl CAN_BTR_TS1_Pos) ## !< 0x00040000
  CAN_BTR_TS1_3* = (0x00000008 shl CAN_BTR_TS1_Pos) ## !< 0x00080000
  CAN_BTR_TS2_Pos* = (20)
  CAN_BTR_TS2_Msk* = (0x00000007 shl CAN_BTR_TS2_Pos) ## !< 0x00700000
  CAN_BTR_TS2* = CAN_BTR_TS2_Msk
  CAN_BTR_TS2_0* = (0x00000001 shl CAN_BTR_TS2_Pos) ## !< 0x00100000
  CAN_BTR_TS2_1* = (0x00000002 shl CAN_BTR_TS2_Pos) ## !< 0x00200000
  CAN_BTR_TS2_2* = (0x00000004 shl CAN_BTR_TS2_Pos) ## !< 0x00400000
  CAN_BTR_SJW_Pos* = (24)
  CAN_BTR_SJW_Msk* = (0x00000003 shl CAN_BTR_SJW_Pos) ## !< 0x03000000
  CAN_BTR_SJW* = CAN_BTR_SJW_Msk
  CAN_BTR_SJW_0* = (0x00000001 shl CAN_BTR_SJW_Pos) ## !< 0x01000000
  CAN_BTR_SJW_1* = (0x00000002 shl CAN_BTR_SJW_Pos) ## !< 0x02000000
  CAN_BTR_LBKM_Pos* = (30)
  CAN_BTR_LBKM_Msk* = (0x00000001 shl CAN_BTR_LBKM_Pos) ## !< 0x40000000
  CAN_BTR_LBKM* = CAN_BTR_LBKM_Msk
  CAN_BTR_SILM_Pos* = (31)
  CAN_BTR_SILM_Msk* = (0x00000001 shl CAN_BTR_SILM_Pos) ## !< 0x80000000
  CAN_BTR_SILM* = CAN_BTR_SILM_Msk

## !<Mailbox registers
## *****************  Bit definition for CAN_TI0R register  *******************

const
  CAN_TI0R_TXRQ_Pos* = (0)
  CAN_TI0R_TXRQ_Msk* = (0x00000001 shl CAN_TI0R_TXRQ_Pos) ## !< 0x00000001
  CAN_TI0R_TXRQ* = CAN_TI0R_TXRQ_Msk
  CAN_TI0R_RTR_Pos* = (1)
  CAN_TI0R_RTR_Msk* = (0x00000001 shl CAN_TI0R_RTR_Pos) ## !< 0x00000002
  CAN_TI0R_RTR* = CAN_TI0R_RTR_Msk
  CAN_TI0R_IDE_Pos* = (2)
  CAN_TI0R_IDE_Msk* = (0x00000001 shl CAN_TI0R_IDE_Pos) ## !< 0x00000004
  CAN_TI0R_IDE* = CAN_TI0R_IDE_Msk
  CAN_TI0R_EXID_Pos* = (3)
  CAN_TI0R_EXID_Msk* = (0x0003FFFF shl CAN_TI0R_EXID_Pos) ## !< 0x001FFFF8
  CAN_TI0R_EXID* = CAN_TI0R_EXID_Msk
  CAN_TI0R_STID_Pos* = (21)
  CAN_TI0R_STID_Msk* = (0x000007FF shl CAN_TI0R_STID_Pos) ## !< 0xFFE00000
  CAN_TI0R_STID* = CAN_TI0R_STID_Msk

## *****************  Bit definition for CAN_TDT0R register  ******************

const
  CAN_TDT0R_DLC_Pos* = (0)
  CAN_TDT0R_DLC_Msk* = (0x0000000F shl CAN_TDT0R_DLC_Pos) ## !< 0x0000000F
  CAN_TDT0R_DLC* = CAN_TDT0R_DLC_Msk
  CAN_TDT0R_TGT_Pos* = (8)
  CAN_TDT0R_TGT_Msk* = (0x00000001 shl CAN_TDT0R_TGT_Pos) ## !< 0x00000100
  CAN_TDT0R_TGT* = CAN_TDT0R_TGT_Msk
  CAN_TDT0R_TIME_Pos* = (16)
  CAN_TDT0R_TIME_Msk* = (0x0000FFFF shl CAN_TDT0R_TIME_Pos) ## !< 0xFFFF0000
  CAN_TDT0R_TIME* = CAN_TDT0R_TIME_Msk

## *****************  Bit definition for CAN_TDL0R register  ******************

const
  CAN_TDL0R_DATA0_Pos* = (0)
  CAN_TDL0R_DATA0_Msk* = (0x000000FF shl CAN_TDL0R_DATA0_Pos) ## !< 0x000000FF
  CAN_TDL0R_DATA0* = CAN_TDL0R_DATA0_Msk
  CAN_TDL0R_DATA1_Pos* = (8)
  CAN_TDL0R_DATA1_Msk* = (0x000000FF shl CAN_TDL0R_DATA1_Pos) ## !< 0x0000FF00
  CAN_TDL0R_DATA1* = CAN_TDL0R_DATA1_Msk
  CAN_TDL0R_DATA2_Pos* = (16)
  CAN_TDL0R_DATA2_Msk* = (0x000000FF shl CAN_TDL0R_DATA2_Pos) ## !< 0x00FF0000
  CAN_TDL0R_DATA2* = CAN_TDL0R_DATA2_Msk
  CAN_TDL0R_DATA3_Pos* = (24)
  CAN_TDL0R_DATA3_Msk* = (0x000000FF shl CAN_TDL0R_DATA3_Pos) ## !< 0xFF000000
  CAN_TDL0R_DATA3* = CAN_TDL0R_DATA3_Msk

## *****************  Bit definition for CAN_TDH0R register  ******************

const
  CAN_TDH0R_DATA4_Pos* = (0)
  CAN_TDH0R_DATA4_Msk* = (0x000000FF shl CAN_TDH0R_DATA4_Pos) ## !< 0x000000FF
  CAN_TDH0R_DATA4* = CAN_TDH0R_DATA4_Msk
  CAN_TDH0R_DATA5_Pos* = (8)
  CAN_TDH0R_DATA5_Msk* = (0x000000FF shl CAN_TDH0R_DATA5_Pos) ## !< 0x0000FF00
  CAN_TDH0R_DATA5* = CAN_TDH0R_DATA5_Msk
  CAN_TDH0R_DATA6_Pos* = (16)
  CAN_TDH0R_DATA6_Msk* = (0x000000FF shl CAN_TDH0R_DATA6_Pos) ## !< 0x00FF0000
  CAN_TDH0R_DATA6* = CAN_TDH0R_DATA6_Msk
  CAN_TDH0R_DATA7_Pos* = (24)
  CAN_TDH0R_DATA7_Msk* = (0x000000FF shl CAN_TDH0R_DATA7_Pos) ## !< 0xFF000000
  CAN_TDH0R_DATA7* = CAN_TDH0R_DATA7_Msk

## ******************  Bit definition for CAN_TI1R register  ******************

const
  CAN_TI1R_TXRQ_Pos* = (0)
  CAN_TI1R_TXRQ_Msk* = (0x00000001 shl CAN_TI1R_TXRQ_Pos) ## !< 0x00000001
  CAN_TI1R_TXRQ* = CAN_TI1R_TXRQ_Msk
  CAN_TI1R_RTR_Pos* = (1)
  CAN_TI1R_RTR_Msk* = (0x00000001 shl CAN_TI1R_RTR_Pos) ## !< 0x00000002
  CAN_TI1R_RTR* = CAN_TI1R_RTR_Msk
  CAN_TI1R_IDE_Pos* = (2)
  CAN_TI1R_IDE_Msk* = (0x00000001 shl CAN_TI1R_IDE_Pos) ## !< 0x00000004
  CAN_TI1R_IDE* = CAN_TI1R_IDE_Msk
  CAN_TI1R_EXID_Pos* = (3)
  CAN_TI1R_EXID_Msk* = (0x0003FFFF shl CAN_TI1R_EXID_Pos) ## !< 0x001FFFF8
  CAN_TI1R_EXID* = CAN_TI1R_EXID_Msk
  CAN_TI1R_STID_Pos* = (21)
  CAN_TI1R_STID_Msk* = (0x000007FF shl CAN_TI1R_STID_Pos) ## !< 0xFFE00000
  CAN_TI1R_STID* = CAN_TI1R_STID_Msk

## ******************  Bit definition for CAN_TDT1R register  *****************

const
  CAN_TDT1R_DLC_Pos* = (0)
  CAN_TDT1R_DLC_Msk* = (0x0000000F shl CAN_TDT1R_DLC_Pos) ## !< 0x0000000F
  CAN_TDT1R_DLC* = CAN_TDT1R_DLC_Msk
  CAN_TDT1R_TGT_Pos* = (8)
  CAN_TDT1R_TGT_Msk* = (0x00000001 shl CAN_TDT1R_TGT_Pos) ## !< 0x00000100
  CAN_TDT1R_TGT* = CAN_TDT1R_TGT_Msk
  CAN_TDT1R_TIME_Pos* = (16)
  CAN_TDT1R_TIME_Msk* = (0x0000FFFF shl CAN_TDT1R_TIME_Pos) ## !< 0xFFFF0000
  CAN_TDT1R_TIME* = CAN_TDT1R_TIME_Msk

## ******************  Bit definition for CAN_TDL1R register  *****************

const
  CAN_TDL1R_DATA0_Pos* = (0)
  CAN_TDL1R_DATA0_Msk* = (0x000000FF shl CAN_TDL1R_DATA0_Pos) ## !< 0x000000FF
  CAN_TDL1R_DATA0* = CAN_TDL1R_DATA0_Msk
  CAN_TDL1R_DATA1_Pos* = (8)
  CAN_TDL1R_DATA1_Msk* = (0x000000FF shl CAN_TDL1R_DATA1_Pos) ## !< 0x0000FF00
  CAN_TDL1R_DATA1* = CAN_TDL1R_DATA1_Msk
  CAN_TDL1R_DATA2_Pos* = (16)
  CAN_TDL1R_DATA2_Msk* = (0x000000FF shl CAN_TDL1R_DATA2_Pos) ## !< 0x00FF0000
  CAN_TDL1R_DATA2* = CAN_TDL1R_DATA2_Msk
  CAN_TDL1R_DATA3_Pos* = (24)
  CAN_TDL1R_DATA3_Msk* = (0x000000FF shl CAN_TDL1R_DATA3_Pos) ## !< 0xFF000000
  CAN_TDL1R_DATA3* = CAN_TDL1R_DATA3_Msk

## ******************  Bit definition for CAN_TDH1R register  *****************

const
  CAN_TDH1R_DATA4_Pos* = (0)
  CAN_TDH1R_DATA4_Msk* = (0x000000FF shl CAN_TDH1R_DATA4_Pos) ## !< 0x000000FF
  CAN_TDH1R_DATA4* = CAN_TDH1R_DATA4_Msk
  CAN_TDH1R_DATA5_Pos* = (8)
  CAN_TDH1R_DATA5_Msk* = (0x000000FF shl CAN_TDH1R_DATA5_Pos) ## !< 0x0000FF00
  CAN_TDH1R_DATA5* = CAN_TDH1R_DATA5_Msk
  CAN_TDH1R_DATA6_Pos* = (16)
  CAN_TDH1R_DATA6_Msk* = (0x000000FF shl CAN_TDH1R_DATA6_Pos) ## !< 0x00FF0000
  CAN_TDH1R_DATA6* = CAN_TDH1R_DATA6_Msk
  CAN_TDH1R_DATA7_Pos* = (24)
  CAN_TDH1R_DATA7_Msk* = (0x000000FF shl CAN_TDH1R_DATA7_Pos) ## !< 0xFF000000
  CAN_TDH1R_DATA7* = CAN_TDH1R_DATA7_Msk

## ******************  Bit definition for CAN_TI2R register  ******************

const
  CAN_TI2R_TXRQ_Pos* = (0)
  CAN_TI2R_TXRQ_Msk* = (0x00000001 shl CAN_TI2R_TXRQ_Pos) ## !< 0x00000001
  CAN_TI2R_TXRQ* = CAN_TI2R_TXRQ_Msk
  CAN_TI2R_RTR_Pos* = (1)
  CAN_TI2R_RTR_Msk* = (0x00000001 shl CAN_TI2R_RTR_Pos) ## !< 0x00000002
  CAN_TI2R_RTR* = CAN_TI2R_RTR_Msk
  CAN_TI2R_IDE_Pos* = (2)
  CAN_TI2R_IDE_Msk* = (0x00000001 shl CAN_TI2R_IDE_Pos) ## !< 0x00000004
  CAN_TI2R_IDE* = CAN_TI2R_IDE_Msk
  CAN_TI2R_EXID_Pos* = (3)
  CAN_TI2R_EXID_Msk* = (0x0003FFFF shl CAN_TI2R_EXID_Pos) ## !< 0x001FFFF8
  CAN_TI2R_EXID* = CAN_TI2R_EXID_Msk
  CAN_TI2R_STID_Pos* = (21)
  CAN_TI2R_STID_Msk* = (0x000007FF shl CAN_TI2R_STID_Pos) ## !< 0xFFE00000
  CAN_TI2R_STID* = CAN_TI2R_STID_Msk

## ******************  Bit definition for CAN_TDT2R register  *****************

const
  CAN_TDT2R_DLC_Pos* = (0)
  CAN_TDT2R_DLC_Msk* = (0x0000000F shl CAN_TDT2R_DLC_Pos) ## !< 0x0000000F
  CAN_TDT2R_DLC* = CAN_TDT2R_DLC_Msk
  CAN_TDT2R_TGT_Pos* = (8)
  CAN_TDT2R_TGT_Msk* = (0x00000001 shl CAN_TDT2R_TGT_Pos) ## !< 0x00000100
  CAN_TDT2R_TGT* = CAN_TDT2R_TGT_Msk
  CAN_TDT2R_TIME_Pos* = (16)
  CAN_TDT2R_TIME_Msk* = (0x0000FFFF shl CAN_TDT2R_TIME_Pos) ## !< 0xFFFF0000
  CAN_TDT2R_TIME* = CAN_TDT2R_TIME_Msk

## ******************  Bit definition for CAN_TDL2R register  *****************

const
  CAN_TDL2R_DATA0_Pos* = (0)
  CAN_TDL2R_DATA0_Msk* = (0x000000FF shl CAN_TDL2R_DATA0_Pos) ## !< 0x000000FF
  CAN_TDL2R_DATA0* = CAN_TDL2R_DATA0_Msk
  CAN_TDL2R_DATA1_Pos* = (8)
  CAN_TDL2R_DATA1_Msk* = (0x000000FF shl CAN_TDL2R_DATA1_Pos) ## !< 0x0000FF00
  CAN_TDL2R_DATA1* = CAN_TDL2R_DATA1_Msk
  CAN_TDL2R_DATA2_Pos* = (16)
  CAN_TDL2R_DATA2_Msk* = (0x000000FF shl CAN_TDL2R_DATA2_Pos) ## !< 0x00FF0000
  CAN_TDL2R_DATA2* = CAN_TDL2R_DATA2_Msk
  CAN_TDL2R_DATA3_Pos* = (24)
  CAN_TDL2R_DATA3_Msk* = (0x000000FF shl CAN_TDL2R_DATA3_Pos) ## !< 0xFF000000
  CAN_TDL2R_DATA3* = CAN_TDL2R_DATA3_Msk

## ******************  Bit definition for CAN_TDH2R register  *****************

const
  CAN_TDH2R_DATA4_Pos* = (0)
  CAN_TDH2R_DATA4_Msk* = (0x000000FF shl CAN_TDH2R_DATA4_Pos) ## !< 0x000000FF
  CAN_TDH2R_DATA4* = CAN_TDH2R_DATA4_Msk
  CAN_TDH2R_DATA5_Pos* = (8)
  CAN_TDH2R_DATA5_Msk* = (0x000000FF shl CAN_TDH2R_DATA5_Pos) ## !< 0x0000FF00
  CAN_TDH2R_DATA5* = CAN_TDH2R_DATA5_Msk
  CAN_TDH2R_DATA6_Pos* = (16)
  CAN_TDH2R_DATA6_Msk* = (0x000000FF shl CAN_TDH2R_DATA6_Pos) ## !< 0x00FF0000
  CAN_TDH2R_DATA6* = CAN_TDH2R_DATA6_Msk
  CAN_TDH2R_DATA7_Pos* = (24)
  CAN_TDH2R_DATA7_Msk* = (0x000000FF shl CAN_TDH2R_DATA7_Pos) ## !< 0xFF000000
  CAN_TDH2R_DATA7* = CAN_TDH2R_DATA7_Msk

## ******************  Bit definition for CAN_RI0R register  ******************

const
  CAN_RI0R_RTR_Pos* = (1)
  CAN_RI0R_RTR_Msk* = (0x00000001 shl CAN_RI0R_RTR_Pos) ## !< 0x00000002
  CAN_RI0R_RTR* = CAN_RI0R_RTR_Msk
  CAN_RI0R_IDE_Pos* = (2)
  CAN_RI0R_IDE_Msk* = (0x00000001 shl CAN_RI0R_IDE_Pos) ## !< 0x00000004
  CAN_RI0R_IDE* = CAN_RI0R_IDE_Msk
  CAN_RI0R_EXID_Pos* = (3)
  CAN_RI0R_EXID_Msk* = (0x0003FFFF shl CAN_RI0R_EXID_Pos) ## !< 0x001FFFF8
  CAN_RI0R_EXID* = CAN_RI0R_EXID_Msk
  CAN_RI0R_STID_Pos* = (21)
  CAN_RI0R_STID_Msk* = (0x000007FF shl CAN_RI0R_STID_Pos) ## !< 0xFFE00000
  CAN_RI0R_STID* = CAN_RI0R_STID_Msk

## ******************  Bit definition for CAN_RDT0R register  *****************

const
  CAN_RDT0R_DLC_Pos* = (0)
  CAN_RDT0R_DLC_Msk* = (0x0000000F shl CAN_RDT0R_DLC_Pos) ## !< 0x0000000F
  CAN_RDT0R_DLC* = CAN_RDT0R_DLC_Msk
  CAN_RDT0R_FMI_Pos* = (8)
  CAN_RDT0R_FMI_Msk* = (0x000000FF shl CAN_RDT0R_FMI_Pos) ## !< 0x0000FF00
  CAN_RDT0R_FMI* = CAN_RDT0R_FMI_Msk
  CAN_RDT0R_TIME_Pos* = (16)
  CAN_RDT0R_TIME_Msk* = (0x0000FFFF shl CAN_RDT0R_TIME_Pos) ## !< 0xFFFF0000
  CAN_RDT0R_TIME* = CAN_RDT0R_TIME_Msk

## ******************  Bit definition for CAN_RDL0R register  *****************

const
  CAN_RDL0R_DATA0_Pos* = (0)
  CAN_RDL0R_DATA0_Msk* = (0x000000FF shl CAN_RDL0R_DATA0_Pos) ## !< 0x000000FF
  CAN_RDL0R_DATA0* = CAN_RDL0R_DATA0_Msk
  CAN_RDL0R_DATA1_Pos* = (8)
  CAN_RDL0R_DATA1_Msk* = (0x000000FF shl CAN_RDL0R_DATA1_Pos) ## !< 0x0000FF00
  CAN_RDL0R_DATA1* = CAN_RDL0R_DATA1_Msk
  CAN_RDL0R_DATA2_Pos* = (16)
  CAN_RDL0R_DATA2_Msk* = (0x000000FF shl CAN_RDL0R_DATA2_Pos) ## !< 0x00FF0000
  CAN_RDL0R_DATA2* = CAN_RDL0R_DATA2_Msk
  CAN_RDL0R_DATA3_Pos* = (24)
  CAN_RDL0R_DATA3_Msk* = (0x000000FF shl CAN_RDL0R_DATA3_Pos) ## !< 0xFF000000
  CAN_RDL0R_DATA3* = CAN_RDL0R_DATA3_Msk

## ******************  Bit definition for CAN_RDH0R register  *****************

const
  CAN_RDH0R_DATA4_Pos* = (0)
  CAN_RDH0R_DATA4_Msk* = (0x000000FF shl CAN_RDH0R_DATA4_Pos) ## !< 0x000000FF
  CAN_RDH0R_DATA4* = CAN_RDH0R_DATA4_Msk
  CAN_RDH0R_DATA5_Pos* = (8)
  CAN_RDH0R_DATA5_Msk* = (0x000000FF shl CAN_RDH0R_DATA5_Pos) ## !< 0x0000FF00
  CAN_RDH0R_DATA5* = CAN_RDH0R_DATA5_Msk
  CAN_RDH0R_DATA6_Pos* = (16)
  CAN_RDH0R_DATA6_Msk* = (0x000000FF shl CAN_RDH0R_DATA6_Pos) ## !< 0x00FF0000
  CAN_RDH0R_DATA6* = CAN_RDH0R_DATA6_Msk
  CAN_RDH0R_DATA7_Pos* = (24)
  CAN_RDH0R_DATA7_Msk* = (0x000000FF shl CAN_RDH0R_DATA7_Pos) ## !< 0xFF000000
  CAN_RDH0R_DATA7* = CAN_RDH0R_DATA7_Msk

## ******************  Bit definition for CAN_RI1R register  ******************

const
  CAN_RI1R_RTR_Pos* = (1)
  CAN_RI1R_RTR_Msk* = (0x00000001 shl CAN_RI1R_RTR_Pos) ## !< 0x00000002
  CAN_RI1R_RTR* = CAN_RI1R_RTR_Msk
  CAN_RI1R_IDE_Pos* = (2)
  CAN_RI1R_IDE_Msk* = (0x00000001 shl CAN_RI1R_IDE_Pos) ## !< 0x00000004
  CAN_RI1R_IDE* = CAN_RI1R_IDE_Msk
  CAN_RI1R_EXID_Pos* = (3)
  CAN_RI1R_EXID_Msk* = (0x0003FFFF shl CAN_RI1R_EXID_Pos) ## !< 0x001FFFF8
  CAN_RI1R_EXID* = CAN_RI1R_EXID_Msk
  CAN_RI1R_STID_Pos* = (21)
  CAN_RI1R_STID_Msk* = (0x000007FF shl CAN_RI1R_STID_Pos) ## !< 0xFFE00000
  CAN_RI1R_STID* = CAN_RI1R_STID_Msk

## ******************  Bit definition for CAN_RDT1R register  *****************

const
  CAN_RDT1R_DLC_Pos* = (0)
  CAN_RDT1R_DLC_Msk* = (0x0000000F shl CAN_RDT1R_DLC_Pos) ## !< 0x0000000F
  CAN_RDT1R_DLC* = CAN_RDT1R_DLC_Msk
  CAN_RDT1R_FMI_Pos* = (8)
  CAN_RDT1R_FMI_Msk* = (0x000000FF shl CAN_RDT1R_FMI_Pos) ## !< 0x0000FF00
  CAN_RDT1R_FMI* = CAN_RDT1R_FMI_Msk
  CAN_RDT1R_TIME_Pos* = (16)
  CAN_RDT1R_TIME_Msk* = (0x0000FFFF shl CAN_RDT1R_TIME_Pos) ## !< 0xFFFF0000
  CAN_RDT1R_TIME* = CAN_RDT1R_TIME_Msk

## ******************  Bit definition for CAN_RDL1R register  *****************

const
  CAN_RDL1R_DATA0_Pos* = (0)
  CAN_RDL1R_DATA0_Msk* = (0x000000FF shl CAN_RDL1R_DATA0_Pos) ## !< 0x000000FF
  CAN_RDL1R_DATA0* = CAN_RDL1R_DATA0_Msk
  CAN_RDL1R_DATA1_Pos* = (8)
  CAN_RDL1R_DATA1_Msk* = (0x000000FF shl CAN_RDL1R_DATA1_Pos) ## !< 0x0000FF00
  CAN_RDL1R_DATA1* = CAN_RDL1R_DATA1_Msk
  CAN_RDL1R_DATA2_Pos* = (16)
  CAN_RDL1R_DATA2_Msk* = (0x000000FF shl CAN_RDL1R_DATA2_Pos) ## !< 0x00FF0000
  CAN_RDL1R_DATA2* = CAN_RDL1R_DATA2_Msk
  CAN_RDL1R_DATA3_Pos* = (24)
  CAN_RDL1R_DATA3_Msk* = (0x000000FF shl CAN_RDL1R_DATA3_Pos) ## !< 0xFF000000
  CAN_RDL1R_DATA3* = CAN_RDL1R_DATA3_Msk

## ******************  Bit definition for CAN_RDH1R register  *****************

const
  CAN_RDH1R_DATA4_Pos* = (0)
  CAN_RDH1R_DATA4_Msk* = (0x000000FF shl CAN_RDH1R_DATA4_Pos) ## !< 0x000000FF
  CAN_RDH1R_DATA4* = CAN_RDH1R_DATA4_Msk
  CAN_RDH1R_DATA5_Pos* = (8)
  CAN_RDH1R_DATA5_Msk* = (0x000000FF shl CAN_RDH1R_DATA5_Pos) ## !< 0x0000FF00
  CAN_RDH1R_DATA5* = CAN_RDH1R_DATA5_Msk
  CAN_RDH1R_DATA6_Pos* = (16)
  CAN_RDH1R_DATA6_Msk* = (0x000000FF shl CAN_RDH1R_DATA6_Pos) ## !< 0x00FF0000
  CAN_RDH1R_DATA6* = CAN_RDH1R_DATA6_Msk
  CAN_RDH1R_DATA7_Pos* = (24)
  CAN_RDH1R_DATA7_Msk* = (0x000000FF shl CAN_RDH1R_DATA7_Pos) ## !< 0xFF000000
  CAN_RDH1R_DATA7* = CAN_RDH1R_DATA7_Msk

## !<CAN filter registers
## ******************  Bit definition for CAN_FMR register  *******************

const
  CAN_FMR_FINIT_Pos* = (0)
  CAN_FMR_FINIT_Msk* = (0x00000001 shl CAN_FMR_FINIT_Pos) ## !< 0x00000001
  CAN_FMR_FINIT* = CAN_FMR_FINIT_Msk

## ******************  Bit definition for CAN_FM1R register  ******************

const
  CAN_FM1R_FBM_Pos* = (0)
  CAN_FM1R_FBM_Msk* = (0x00003FFF shl CAN_FM1R_FBM_Pos) ## !< 0x00003FFF
  CAN_FM1R_FBM* = CAN_FM1R_FBM_Msk
  CAN_FM1R_FBM0_Pos* = (0)
  CAN_FM1R_FBM0_Msk* = (0x00000001 shl CAN_FM1R_FBM0_Pos) ## !< 0x00000001
  CAN_FM1R_FBM0* = CAN_FM1R_FBM0_Msk
  CAN_FM1R_FBM1_Pos* = (1)
  CAN_FM1R_FBM1_Msk* = (0x00000001 shl CAN_FM1R_FBM1_Pos) ## !< 0x00000002
  CAN_FM1R_FBM1* = CAN_FM1R_FBM1_Msk
  CAN_FM1R_FBM2_Pos* = (2)
  CAN_FM1R_FBM2_Msk* = (0x00000001 shl CAN_FM1R_FBM2_Pos) ## !< 0x00000004
  CAN_FM1R_FBM2* = CAN_FM1R_FBM2_Msk
  CAN_FM1R_FBM3_Pos* = (3)
  CAN_FM1R_FBM3_Msk* = (0x00000001 shl CAN_FM1R_FBM3_Pos) ## !< 0x00000008
  CAN_FM1R_FBM3* = CAN_FM1R_FBM3_Msk
  CAN_FM1R_FBM4_Pos* = (4)
  CAN_FM1R_FBM4_Msk* = (0x00000001 shl CAN_FM1R_FBM4_Pos) ## !< 0x00000010
  CAN_FM1R_FBM4* = CAN_FM1R_FBM4_Msk
  CAN_FM1R_FBM5_Pos* = (5)
  CAN_FM1R_FBM5_Msk* = (0x00000001 shl CAN_FM1R_FBM5_Pos) ## !< 0x00000020
  CAN_FM1R_FBM5* = CAN_FM1R_FBM5_Msk
  CAN_FM1R_FBM6_Pos* = (6)
  CAN_FM1R_FBM6_Msk* = (0x00000001 shl CAN_FM1R_FBM6_Pos) ## !< 0x00000040
  CAN_FM1R_FBM6* = CAN_FM1R_FBM6_Msk
  CAN_FM1R_FBM7_Pos* = (7)
  CAN_FM1R_FBM7_Msk* = (0x00000001 shl CAN_FM1R_FBM7_Pos) ## !< 0x00000080
  CAN_FM1R_FBM7* = CAN_FM1R_FBM7_Msk
  CAN_FM1R_FBM8_Pos* = (8)
  CAN_FM1R_FBM8_Msk* = (0x00000001 shl CAN_FM1R_FBM8_Pos) ## !< 0x00000100
  CAN_FM1R_FBM8* = CAN_FM1R_FBM8_Msk
  CAN_FM1R_FBM9_Pos* = (9)
  CAN_FM1R_FBM9_Msk* = (0x00000001 shl CAN_FM1R_FBM9_Pos) ## !< 0x00000200
  CAN_FM1R_FBM9* = CAN_FM1R_FBM9_Msk
  CAN_FM1R_FBM10_Pos* = (10)
  CAN_FM1R_FBM10_Msk* = (0x00000001 shl CAN_FM1R_FBM10_Pos) ## !< 0x00000400
  CAN_FM1R_FBM10* = CAN_FM1R_FBM10_Msk
  CAN_FM1R_FBM11_Pos* = (11)
  CAN_FM1R_FBM11_Msk* = (0x00000001 shl CAN_FM1R_FBM11_Pos) ## !< 0x00000800
  CAN_FM1R_FBM11* = CAN_FM1R_FBM11_Msk
  CAN_FM1R_FBM12_Pos* = (12)
  CAN_FM1R_FBM12_Msk* = (0x00000001 shl CAN_FM1R_FBM12_Pos) ## !< 0x00001000
  CAN_FM1R_FBM12* = CAN_FM1R_FBM12_Msk
  CAN_FM1R_FBM13_Pos* = (13)
  CAN_FM1R_FBM13_Msk* = (0x00000001 shl CAN_FM1R_FBM13_Pos) ## !< 0x00002000
  CAN_FM1R_FBM13* = CAN_FM1R_FBM13_Msk

## ******************  Bit definition for CAN_FS1R register  ******************

const
  CAN_FS1R_FSC_Pos* = (0)
  CAN_FS1R_FSC_Msk* = (0x00003FFF shl CAN_FS1R_FSC_Pos) ## !< 0x00003FFF
  CAN_FS1R_FSC* = CAN_FS1R_FSC_Msk
  CAN_FS1R_FSC0_Pos* = (0)
  CAN_FS1R_FSC0_Msk* = (0x00000001 shl CAN_FS1R_FSC0_Pos) ## !< 0x00000001
  CAN_FS1R_FSC0* = CAN_FS1R_FSC0_Msk
  CAN_FS1R_FSC1_Pos* = (1)
  CAN_FS1R_FSC1_Msk* = (0x00000001 shl CAN_FS1R_FSC1_Pos) ## !< 0x00000002
  CAN_FS1R_FSC1* = CAN_FS1R_FSC1_Msk
  CAN_FS1R_FSC2_Pos* = (2)
  CAN_FS1R_FSC2_Msk* = (0x00000001 shl CAN_FS1R_FSC2_Pos) ## !< 0x00000004
  CAN_FS1R_FSC2* = CAN_FS1R_FSC2_Msk
  CAN_FS1R_FSC3_Pos* = (3)
  CAN_FS1R_FSC3_Msk* = (0x00000001 shl CAN_FS1R_FSC3_Pos) ## !< 0x00000008
  CAN_FS1R_FSC3* = CAN_FS1R_FSC3_Msk
  CAN_FS1R_FSC4_Pos* = (4)
  CAN_FS1R_FSC4_Msk* = (0x00000001 shl CAN_FS1R_FSC4_Pos) ## !< 0x00000010
  CAN_FS1R_FSC4* = CAN_FS1R_FSC4_Msk
  CAN_FS1R_FSC5_Pos* = (5)
  CAN_FS1R_FSC5_Msk* = (0x00000001 shl CAN_FS1R_FSC5_Pos) ## !< 0x00000020
  CAN_FS1R_FSC5* = CAN_FS1R_FSC5_Msk
  CAN_FS1R_FSC6_Pos* = (6)
  CAN_FS1R_FSC6_Msk* = (0x00000001 shl CAN_FS1R_FSC6_Pos) ## !< 0x00000040
  CAN_FS1R_FSC6* = CAN_FS1R_FSC6_Msk
  CAN_FS1R_FSC7_Pos* = (7)
  CAN_FS1R_FSC7_Msk* = (0x00000001 shl CAN_FS1R_FSC7_Pos) ## !< 0x00000080
  CAN_FS1R_FSC7* = CAN_FS1R_FSC7_Msk
  CAN_FS1R_FSC8_Pos* = (8)
  CAN_FS1R_FSC8_Msk* = (0x00000001 shl CAN_FS1R_FSC8_Pos) ## !< 0x00000100
  CAN_FS1R_FSC8* = CAN_FS1R_FSC8_Msk
  CAN_FS1R_FSC9_Pos* = (9)
  CAN_FS1R_FSC9_Msk* = (0x00000001 shl CAN_FS1R_FSC9_Pos) ## !< 0x00000200
  CAN_FS1R_FSC9* = CAN_FS1R_FSC9_Msk
  CAN_FS1R_FSC10_Pos* = (10)
  CAN_FS1R_FSC10_Msk* = (0x00000001 shl CAN_FS1R_FSC10_Pos) ## !< 0x00000400
  CAN_FS1R_FSC10* = CAN_FS1R_FSC10_Msk
  CAN_FS1R_FSC11_Pos* = (11)
  CAN_FS1R_FSC11_Msk* = (0x00000001 shl CAN_FS1R_FSC11_Pos) ## !< 0x00000800
  CAN_FS1R_FSC11* = CAN_FS1R_FSC11_Msk
  CAN_FS1R_FSC12_Pos* = (12)
  CAN_FS1R_FSC12_Msk* = (0x00000001 shl CAN_FS1R_FSC12_Pos) ## !< 0x00001000
  CAN_FS1R_FSC12* = CAN_FS1R_FSC12_Msk
  CAN_FS1R_FSC13_Pos* = (13)
  CAN_FS1R_FSC13_Msk* = (0x00000001 shl CAN_FS1R_FSC13_Pos) ## !< 0x00002000
  CAN_FS1R_FSC13* = CAN_FS1R_FSC13_Msk

## *****************  Bit definition for CAN_FFA1R register  ******************

const
  CAN_FFA1R_FFA_Pos* = (0)
  CAN_FFA1R_FFA_Msk* = (0x00003FFF shl CAN_FFA1R_FFA_Pos) ## !< 0x00003FFF
  CAN_FFA1R_FFA* = CAN_FFA1R_FFA_Msk
  CAN_FFA1R_FFA0_Pos* = (0)
  CAN_FFA1R_FFA0_Msk* = (0x00000001 shl CAN_FFA1R_FFA0_Pos) ## !< 0x00000001
  CAN_FFA1R_FFA0* = CAN_FFA1R_FFA0_Msk
  CAN_FFA1R_FFA1_Pos* = (1)
  CAN_FFA1R_FFA1_Msk* = (0x00000001 shl CAN_FFA1R_FFA1_Pos) ## !< 0x00000002
  CAN_FFA1R_FFA1* = CAN_FFA1R_FFA1_Msk
  CAN_FFA1R_FFA2_Pos* = (2)
  CAN_FFA1R_FFA2_Msk* = (0x00000001 shl CAN_FFA1R_FFA2_Pos) ## !< 0x00000004
  CAN_FFA1R_FFA2* = CAN_FFA1R_FFA2_Msk
  CAN_FFA1R_FFA3_Pos* = (3)
  CAN_FFA1R_FFA3_Msk* = (0x00000001 shl CAN_FFA1R_FFA3_Pos) ## !< 0x00000008
  CAN_FFA1R_FFA3* = CAN_FFA1R_FFA3_Msk
  CAN_FFA1R_FFA4_Pos* = (4)
  CAN_FFA1R_FFA4_Msk* = (0x00000001 shl CAN_FFA1R_FFA4_Pos) ## !< 0x00000010
  CAN_FFA1R_FFA4* = CAN_FFA1R_FFA4_Msk
  CAN_FFA1R_FFA5_Pos* = (5)
  CAN_FFA1R_FFA5_Msk* = (0x00000001 shl CAN_FFA1R_FFA5_Pos) ## !< 0x00000020
  CAN_FFA1R_FFA5* = CAN_FFA1R_FFA5_Msk
  CAN_FFA1R_FFA6_Pos* = (6)
  CAN_FFA1R_FFA6_Msk* = (0x00000001 shl CAN_FFA1R_FFA6_Pos) ## !< 0x00000040
  CAN_FFA1R_FFA6* = CAN_FFA1R_FFA6_Msk
  CAN_FFA1R_FFA7_Pos* = (7)
  CAN_FFA1R_FFA7_Msk* = (0x00000001 shl CAN_FFA1R_FFA7_Pos) ## !< 0x00000080
  CAN_FFA1R_FFA7* = CAN_FFA1R_FFA7_Msk
  CAN_FFA1R_FFA8_Pos* = (8)
  CAN_FFA1R_FFA8_Msk* = (0x00000001 shl CAN_FFA1R_FFA8_Pos) ## !< 0x00000100
  CAN_FFA1R_FFA8* = CAN_FFA1R_FFA8_Msk
  CAN_FFA1R_FFA9_Pos* = (9)
  CAN_FFA1R_FFA9_Msk* = (0x00000001 shl CAN_FFA1R_FFA9_Pos) ## !< 0x00000200
  CAN_FFA1R_FFA9* = CAN_FFA1R_FFA9_Msk
  CAN_FFA1R_FFA10_Pos* = (10)
  CAN_FFA1R_FFA10_Msk* = (0x00000001 shl CAN_FFA1R_FFA10_Pos) ## !< 0x00000400
  CAN_FFA1R_FFA10* = CAN_FFA1R_FFA10_Msk
  CAN_FFA1R_FFA11_Pos* = (11)
  CAN_FFA1R_FFA11_Msk* = (0x00000001 shl CAN_FFA1R_FFA11_Pos) ## !< 0x00000800
  CAN_FFA1R_FFA11* = CAN_FFA1R_FFA11_Msk
  CAN_FFA1R_FFA12_Pos* = (12)
  CAN_FFA1R_FFA12_Msk* = (0x00000001 shl CAN_FFA1R_FFA12_Pos) ## !< 0x00001000
  CAN_FFA1R_FFA12* = CAN_FFA1R_FFA12_Msk
  CAN_FFA1R_FFA13_Pos* = (13)
  CAN_FFA1R_FFA13_Msk* = (0x00000001 shl CAN_FFA1R_FFA13_Pos) ## !< 0x00002000
  CAN_FFA1R_FFA13* = CAN_FFA1R_FFA13_Msk

## ******************  Bit definition for CAN_FA1R register  ******************

const
  CAN_FA1R_FACT_Pos* = (0)
  CAN_FA1R_FACT_Msk* = (0x00003FFF shl CAN_FA1R_FACT_Pos) ## !< 0x00003FFF
  CAN_FA1R_FACT* = CAN_FA1R_FACT_Msk
  CAN_FA1R_FACT0_Pos* = (0)
  CAN_FA1R_FACT0_Msk* = (0x00000001 shl CAN_FA1R_FACT0_Pos) ## !< 0x00000001
  CAN_FA1R_FACT0* = CAN_FA1R_FACT0_Msk
  CAN_FA1R_FACT1_Pos* = (1)
  CAN_FA1R_FACT1_Msk* = (0x00000001 shl CAN_FA1R_FACT1_Pos) ## !< 0x00000002
  CAN_FA1R_FACT1* = CAN_FA1R_FACT1_Msk
  CAN_FA1R_FACT2_Pos* = (2)
  CAN_FA1R_FACT2_Msk* = (0x00000001 shl CAN_FA1R_FACT2_Pos) ## !< 0x00000004
  CAN_FA1R_FACT2* = CAN_FA1R_FACT2_Msk
  CAN_FA1R_FACT3_Pos* = (3)
  CAN_FA1R_FACT3_Msk* = (0x00000001 shl CAN_FA1R_FACT3_Pos) ## !< 0x00000008
  CAN_FA1R_FACT3* = CAN_FA1R_FACT3_Msk
  CAN_FA1R_FACT4_Pos* = (4)
  CAN_FA1R_FACT4_Msk* = (0x00000001 shl CAN_FA1R_FACT4_Pos) ## !< 0x00000010
  CAN_FA1R_FACT4* = CAN_FA1R_FACT4_Msk
  CAN_FA1R_FACT5_Pos* = (5)
  CAN_FA1R_FACT5_Msk* = (0x00000001 shl CAN_FA1R_FACT5_Pos) ## !< 0x00000020
  CAN_FA1R_FACT5* = CAN_FA1R_FACT5_Msk
  CAN_FA1R_FACT6_Pos* = (6)
  CAN_FA1R_FACT6_Msk* = (0x00000001 shl CAN_FA1R_FACT6_Pos) ## !< 0x00000040
  CAN_FA1R_FACT6* = CAN_FA1R_FACT6_Msk
  CAN_FA1R_FACT7_Pos* = (7)
  CAN_FA1R_FACT7_Msk* = (0x00000001 shl CAN_FA1R_FACT7_Pos) ## !< 0x00000080
  CAN_FA1R_FACT7* = CAN_FA1R_FACT7_Msk
  CAN_FA1R_FACT8_Pos* = (8)
  CAN_FA1R_FACT8_Msk* = (0x00000001 shl CAN_FA1R_FACT8_Pos) ## !< 0x00000100
  CAN_FA1R_FACT8* = CAN_FA1R_FACT8_Msk
  CAN_FA1R_FACT9_Pos* = (9)
  CAN_FA1R_FACT9_Msk* = (0x00000001 shl CAN_FA1R_FACT9_Pos) ## !< 0x00000200
  CAN_FA1R_FACT9* = CAN_FA1R_FACT9_Msk
  CAN_FA1R_FACT10_Pos* = (10)
  CAN_FA1R_FACT10_Msk* = (0x00000001 shl CAN_FA1R_FACT10_Pos) ## !< 0x00000400
  CAN_FA1R_FACT10* = CAN_FA1R_FACT10_Msk
  CAN_FA1R_FACT11_Pos* = (11)
  CAN_FA1R_FACT11_Msk* = (0x00000001 shl CAN_FA1R_FACT11_Pos) ## !< 0x00000800
  CAN_FA1R_FACT11* = CAN_FA1R_FACT11_Msk
  CAN_FA1R_FACT12_Pos* = (12)
  CAN_FA1R_FACT12_Msk* = (0x00000001 shl CAN_FA1R_FACT12_Pos) ## !< 0x00001000
  CAN_FA1R_FACT12* = CAN_FA1R_FACT12_Msk
  CAN_FA1R_FACT13_Pos* = (13)
  CAN_FA1R_FACT13_Msk* = (0x00000001 shl CAN_FA1R_FACT13_Pos) ## !< 0x00002000
  CAN_FA1R_FACT13* = CAN_FA1R_FACT13_Msk

## ******************  Bit definition for CAN_F0R1 register  ******************

const
  CAN_F0R1_FB0_Pos* = (0)
  CAN_F0R1_FB0_Msk* = (0x00000001 shl CAN_F0R1_FB0_Pos) ## !< 0x00000001
  CAN_F0R1_FB0* = CAN_F0R1_FB0_Msk
  CAN_F0R1_FB1_Pos* = (1)
  CAN_F0R1_FB1_Msk* = (0x00000001 shl CAN_F0R1_FB1_Pos) ## !< 0x00000002
  CAN_F0R1_FB1* = CAN_F0R1_FB1_Msk
  CAN_F0R1_FB2_Pos* = (2)
  CAN_F0R1_FB2_Msk* = (0x00000001 shl CAN_F0R1_FB2_Pos) ## !< 0x00000004
  CAN_F0R1_FB2* = CAN_F0R1_FB2_Msk
  CAN_F0R1_FB3_Pos* = (3)
  CAN_F0R1_FB3_Msk* = (0x00000001 shl CAN_F0R1_FB3_Pos) ## !< 0x00000008
  CAN_F0R1_FB3* = CAN_F0R1_FB3_Msk
  CAN_F0R1_FB4_Pos* = (4)
  CAN_F0R1_FB4_Msk* = (0x00000001 shl CAN_F0R1_FB4_Pos) ## !< 0x00000010
  CAN_F0R1_FB4* = CAN_F0R1_FB4_Msk
  CAN_F0R1_FB5_Pos* = (5)
  CAN_F0R1_FB5_Msk* = (0x00000001 shl CAN_F0R1_FB5_Pos) ## !< 0x00000020
  CAN_F0R1_FB5* = CAN_F0R1_FB5_Msk
  CAN_F0R1_FB6_Pos* = (6)
  CAN_F0R1_FB6_Msk* = (0x00000001 shl CAN_F0R1_FB6_Pos) ## !< 0x00000040
  CAN_F0R1_FB6* = CAN_F0R1_FB6_Msk
  CAN_F0R1_FB7_Pos* = (7)
  CAN_F0R1_FB7_Msk* = (0x00000001 shl CAN_F0R1_FB7_Pos) ## !< 0x00000080
  CAN_F0R1_FB7* = CAN_F0R1_FB7_Msk
  CAN_F0R1_FB8_Pos* = (8)
  CAN_F0R1_FB8_Msk* = (0x00000001 shl CAN_F0R1_FB8_Pos) ## !< 0x00000100
  CAN_F0R1_FB8* = CAN_F0R1_FB8_Msk
  CAN_F0R1_FB9_Pos* = (9)
  CAN_F0R1_FB9_Msk* = (0x00000001 shl CAN_F0R1_FB9_Pos) ## !< 0x00000200
  CAN_F0R1_FB9* = CAN_F0R1_FB9_Msk
  CAN_F0R1_FB10_Pos* = (10)
  CAN_F0R1_FB10_Msk* = (0x00000001 shl CAN_F0R1_FB10_Pos) ## !< 0x00000400
  CAN_F0R1_FB10* = CAN_F0R1_FB10_Msk
  CAN_F0R1_FB11_Pos* = (11)
  CAN_F0R1_FB11_Msk* = (0x00000001 shl CAN_F0R1_FB11_Pos) ## !< 0x00000800
  CAN_F0R1_FB11* = CAN_F0R1_FB11_Msk
  CAN_F0R1_FB12_Pos* = (12)
  CAN_F0R1_FB12_Msk* = (0x00000001 shl CAN_F0R1_FB12_Pos) ## !< 0x00001000
  CAN_F0R1_FB12* = CAN_F0R1_FB12_Msk
  CAN_F0R1_FB13_Pos* = (13)
  CAN_F0R1_FB13_Msk* = (0x00000001 shl CAN_F0R1_FB13_Pos) ## !< 0x00002000
  CAN_F0R1_FB13* = CAN_F0R1_FB13_Msk
  CAN_F0R1_FB14_Pos* = (14)
  CAN_F0R1_FB14_Msk* = (0x00000001 shl CAN_F0R1_FB14_Pos) ## !< 0x00004000
  CAN_F0R1_FB14* = CAN_F0R1_FB14_Msk
  CAN_F0R1_FB15_Pos* = (15)
  CAN_F0R1_FB15_Msk* = (0x00000001 shl CAN_F0R1_FB15_Pos) ## !< 0x00008000
  CAN_F0R1_FB15* = CAN_F0R1_FB15_Msk
  CAN_F0R1_FB16_Pos* = (16)
  CAN_F0R1_FB16_Msk* = (0x00000001 shl CAN_F0R1_FB16_Pos) ## !< 0x00010000
  CAN_F0R1_FB16* = CAN_F0R1_FB16_Msk
  CAN_F0R1_FB17_Pos* = (17)
  CAN_F0R1_FB17_Msk* = (0x00000001 shl CAN_F0R1_FB17_Pos) ## !< 0x00020000
  CAN_F0R1_FB17* = CAN_F0R1_FB17_Msk
  CAN_F0R1_FB18_Pos* = (18)
  CAN_F0R1_FB18_Msk* = (0x00000001 shl CAN_F0R1_FB18_Pos) ## !< 0x00040000
  CAN_F0R1_FB18* = CAN_F0R1_FB18_Msk
  CAN_F0R1_FB19_Pos* = (19)
  CAN_F0R1_FB19_Msk* = (0x00000001 shl CAN_F0R1_FB19_Pos) ## !< 0x00080000
  CAN_F0R1_FB19* = CAN_F0R1_FB19_Msk
  CAN_F0R1_FB20_Pos* = (20)
  CAN_F0R1_FB20_Msk* = (0x00000001 shl CAN_F0R1_FB20_Pos) ## !< 0x00100000
  CAN_F0R1_FB20* = CAN_F0R1_FB20_Msk
  CAN_F0R1_FB21_Pos* = (21)
  CAN_F0R1_FB21_Msk* = (0x00000001 shl CAN_F0R1_FB21_Pos) ## !< 0x00200000
  CAN_F0R1_FB21* = CAN_F0R1_FB21_Msk
  CAN_F0R1_FB22_Pos* = (22)
  CAN_F0R1_FB22_Msk* = (0x00000001 shl CAN_F0R1_FB22_Pos) ## !< 0x00400000
  CAN_F0R1_FB22* = CAN_F0R1_FB22_Msk
  CAN_F0R1_FB23_Pos* = (23)
  CAN_F0R1_FB23_Msk* = (0x00000001 shl CAN_F0R1_FB23_Pos) ## !< 0x00800000
  CAN_F0R1_FB23* = CAN_F0R1_FB23_Msk
  CAN_F0R1_FB24_Pos* = (24)
  CAN_F0R1_FB24_Msk* = (0x00000001 shl CAN_F0R1_FB24_Pos) ## !< 0x01000000
  CAN_F0R1_FB24* = CAN_F0R1_FB24_Msk
  CAN_F0R1_FB25_Pos* = (25)
  CAN_F0R1_FB25_Msk* = (0x00000001 shl CAN_F0R1_FB25_Pos) ## !< 0x02000000
  CAN_F0R1_FB25* = CAN_F0R1_FB25_Msk
  CAN_F0R1_FB26_Pos* = (26)
  CAN_F0R1_FB26_Msk* = (0x00000001 shl CAN_F0R1_FB26_Pos) ## !< 0x04000000
  CAN_F0R1_FB26* = CAN_F0R1_FB26_Msk
  CAN_F0R1_FB27_Pos* = (27)
  CAN_F0R1_FB27_Msk* = (0x00000001 shl CAN_F0R1_FB27_Pos) ## !< 0x08000000
  CAN_F0R1_FB27* = CAN_F0R1_FB27_Msk
  CAN_F0R1_FB28_Pos* = (28)
  CAN_F0R1_FB28_Msk* = (0x00000001 shl CAN_F0R1_FB28_Pos) ## !< 0x10000000
  CAN_F0R1_FB28* = CAN_F0R1_FB28_Msk
  CAN_F0R1_FB29_Pos* = (29)
  CAN_F0R1_FB29_Msk* = (0x00000001 shl CAN_F0R1_FB29_Pos) ## !< 0x20000000
  CAN_F0R1_FB29* = CAN_F0R1_FB29_Msk
  CAN_F0R1_FB30_Pos* = (30)
  CAN_F0R1_FB30_Msk* = (0x00000001 shl CAN_F0R1_FB30_Pos) ## !< 0x40000000
  CAN_F0R1_FB30* = CAN_F0R1_FB30_Msk
  CAN_F0R1_FB31_Pos* = (31)
  CAN_F0R1_FB31_Msk* = (0x00000001 shl CAN_F0R1_FB31_Pos) ## !< 0x80000000
  CAN_F0R1_FB31* = CAN_F0R1_FB31_Msk

## ******************  Bit definition for CAN_F1R1 register  ******************

const
  CAN_F1R1_FB0_Pos* = (0)
  CAN_F1R1_FB0_Msk* = (0x00000001 shl CAN_F1R1_FB0_Pos) ## !< 0x00000001
  CAN_F1R1_FB0* = CAN_F1R1_FB0_Msk
  CAN_F1R1_FB1_Pos* = (1)
  CAN_F1R1_FB1_Msk* = (0x00000001 shl CAN_F1R1_FB1_Pos) ## !< 0x00000002
  CAN_F1R1_FB1* = CAN_F1R1_FB1_Msk
  CAN_F1R1_FB2_Pos* = (2)
  CAN_F1R1_FB2_Msk* = (0x00000001 shl CAN_F1R1_FB2_Pos) ## !< 0x00000004
  CAN_F1R1_FB2* = CAN_F1R1_FB2_Msk
  CAN_F1R1_FB3_Pos* = (3)
  CAN_F1R1_FB3_Msk* = (0x00000001 shl CAN_F1R1_FB3_Pos) ## !< 0x00000008
  CAN_F1R1_FB3* = CAN_F1R1_FB3_Msk
  CAN_F1R1_FB4_Pos* = (4)
  CAN_F1R1_FB4_Msk* = (0x00000001 shl CAN_F1R1_FB4_Pos) ## !< 0x00000010
  CAN_F1R1_FB4* = CAN_F1R1_FB4_Msk
  CAN_F1R1_FB5_Pos* = (5)
  CAN_F1R1_FB5_Msk* = (0x00000001 shl CAN_F1R1_FB5_Pos) ## !< 0x00000020
  CAN_F1R1_FB5* = CAN_F1R1_FB5_Msk
  CAN_F1R1_FB6_Pos* = (6)
  CAN_F1R1_FB6_Msk* = (0x00000001 shl CAN_F1R1_FB6_Pos) ## !< 0x00000040
  CAN_F1R1_FB6* = CAN_F1R1_FB6_Msk
  CAN_F1R1_FB7_Pos* = (7)
  CAN_F1R1_FB7_Msk* = (0x00000001 shl CAN_F1R1_FB7_Pos) ## !< 0x00000080
  CAN_F1R1_FB7* = CAN_F1R1_FB7_Msk
  CAN_F1R1_FB8_Pos* = (8)
  CAN_F1R1_FB8_Msk* = (0x00000001 shl CAN_F1R1_FB8_Pos) ## !< 0x00000100
  CAN_F1R1_FB8* = CAN_F1R1_FB8_Msk
  CAN_F1R1_FB9_Pos* = (9)
  CAN_F1R1_FB9_Msk* = (0x00000001 shl CAN_F1R1_FB9_Pos) ## !< 0x00000200
  CAN_F1R1_FB9* = CAN_F1R1_FB9_Msk
  CAN_F1R1_FB10_Pos* = (10)
  CAN_F1R1_FB10_Msk* = (0x00000001 shl CAN_F1R1_FB10_Pos) ## !< 0x00000400
  CAN_F1R1_FB10* = CAN_F1R1_FB10_Msk
  CAN_F1R1_FB11_Pos* = (11)
  CAN_F1R1_FB11_Msk* = (0x00000001 shl CAN_F1R1_FB11_Pos) ## !< 0x00000800
  CAN_F1R1_FB11* = CAN_F1R1_FB11_Msk
  CAN_F1R1_FB12_Pos* = (12)
  CAN_F1R1_FB12_Msk* = (0x00000001 shl CAN_F1R1_FB12_Pos) ## !< 0x00001000
  CAN_F1R1_FB12* = CAN_F1R1_FB12_Msk
  CAN_F1R1_FB13_Pos* = (13)
  CAN_F1R1_FB13_Msk* = (0x00000001 shl CAN_F1R1_FB13_Pos) ## !< 0x00002000
  CAN_F1R1_FB13* = CAN_F1R1_FB13_Msk
  CAN_F1R1_FB14_Pos* = (14)
  CAN_F1R1_FB14_Msk* = (0x00000001 shl CAN_F1R1_FB14_Pos) ## !< 0x00004000
  CAN_F1R1_FB14* = CAN_F1R1_FB14_Msk
  CAN_F1R1_FB15_Pos* = (15)
  CAN_F1R1_FB15_Msk* = (0x00000001 shl CAN_F1R1_FB15_Pos) ## !< 0x00008000
  CAN_F1R1_FB15* = CAN_F1R1_FB15_Msk
  CAN_F1R1_FB16_Pos* = (16)
  CAN_F1R1_FB16_Msk* = (0x00000001 shl CAN_F1R1_FB16_Pos) ## !< 0x00010000
  CAN_F1R1_FB16* = CAN_F1R1_FB16_Msk
  CAN_F1R1_FB17_Pos* = (17)
  CAN_F1R1_FB17_Msk* = (0x00000001 shl CAN_F1R1_FB17_Pos) ## !< 0x00020000
  CAN_F1R1_FB17* = CAN_F1R1_FB17_Msk
  CAN_F1R1_FB18_Pos* = (18)
  CAN_F1R1_FB18_Msk* = (0x00000001 shl CAN_F1R1_FB18_Pos) ## !< 0x00040000
  CAN_F1R1_FB18* = CAN_F1R1_FB18_Msk
  CAN_F1R1_FB19_Pos* = (19)
  CAN_F1R1_FB19_Msk* = (0x00000001 shl CAN_F1R1_FB19_Pos) ## !< 0x00080000
  CAN_F1R1_FB19* = CAN_F1R1_FB19_Msk
  CAN_F1R1_FB20_Pos* = (20)
  CAN_F1R1_FB20_Msk* = (0x00000001 shl CAN_F1R1_FB20_Pos) ## !< 0x00100000
  CAN_F1R1_FB20* = CAN_F1R1_FB20_Msk
  CAN_F1R1_FB21_Pos* = (21)
  CAN_F1R1_FB21_Msk* = (0x00000001 shl CAN_F1R1_FB21_Pos) ## !< 0x00200000
  CAN_F1R1_FB21* = CAN_F1R1_FB21_Msk
  CAN_F1R1_FB22_Pos* = (22)
  CAN_F1R1_FB22_Msk* = (0x00000001 shl CAN_F1R1_FB22_Pos) ## !< 0x00400000
  CAN_F1R1_FB22* = CAN_F1R1_FB22_Msk
  CAN_F1R1_FB23_Pos* = (23)
  CAN_F1R1_FB23_Msk* = (0x00000001 shl CAN_F1R1_FB23_Pos) ## !< 0x00800000
  CAN_F1R1_FB23* = CAN_F1R1_FB23_Msk
  CAN_F1R1_FB24_Pos* = (24)
  CAN_F1R1_FB24_Msk* = (0x00000001 shl CAN_F1R1_FB24_Pos) ## !< 0x01000000
  CAN_F1R1_FB24* = CAN_F1R1_FB24_Msk
  CAN_F1R1_FB25_Pos* = (25)
  CAN_F1R1_FB25_Msk* = (0x00000001 shl CAN_F1R1_FB25_Pos) ## !< 0x02000000
  CAN_F1R1_FB25* = CAN_F1R1_FB25_Msk
  CAN_F1R1_FB26_Pos* = (26)
  CAN_F1R1_FB26_Msk* = (0x00000001 shl CAN_F1R1_FB26_Pos) ## !< 0x04000000
  CAN_F1R1_FB26* = CAN_F1R1_FB26_Msk
  CAN_F1R1_FB27_Pos* = (27)
  CAN_F1R1_FB27_Msk* = (0x00000001 shl CAN_F1R1_FB27_Pos) ## !< 0x08000000
  CAN_F1R1_FB27* = CAN_F1R1_FB27_Msk
  CAN_F1R1_FB28_Pos* = (28)
  CAN_F1R1_FB28_Msk* = (0x00000001 shl CAN_F1R1_FB28_Pos) ## !< 0x10000000
  CAN_F1R1_FB28* = CAN_F1R1_FB28_Msk
  CAN_F1R1_FB29_Pos* = (29)
  CAN_F1R1_FB29_Msk* = (0x00000001 shl CAN_F1R1_FB29_Pos) ## !< 0x20000000
  CAN_F1R1_FB29* = CAN_F1R1_FB29_Msk
  CAN_F1R1_FB30_Pos* = (30)
  CAN_F1R1_FB30_Msk* = (0x00000001 shl CAN_F1R1_FB30_Pos) ## !< 0x40000000
  CAN_F1R1_FB30* = CAN_F1R1_FB30_Msk
  CAN_F1R1_FB31_Pos* = (31)
  CAN_F1R1_FB31_Msk* = (0x00000001 shl CAN_F1R1_FB31_Pos) ## !< 0x80000000
  CAN_F1R1_FB31* = CAN_F1R1_FB31_Msk

## ******************  Bit definition for CAN_F2R1 register  ******************

const
  CAN_F2R1_FB0_Pos* = (0)
  CAN_F2R1_FB0_Msk* = (0x00000001 shl CAN_F2R1_FB0_Pos) ## !< 0x00000001
  CAN_F2R1_FB0* = CAN_F2R1_FB0_Msk
  CAN_F2R1_FB1_Pos* = (1)
  CAN_F2R1_FB1_Msk* = (0x00000001 shl CAN_F2R1_FB1_Pos) ## !< 0x00000002
  CAN_F2R1_FB1* = CAN_F2R1_FB1_Msk
  CAN_F2R1_FB2_Pos* = (2)
  CAN_F2R1_FB2_Msk* = (0x00000001 shl CAN_F2R1_FB2_Pos) ## !< 0x00000004
  CAN_F2R1_FB2* = CAN_F2R1_FB2_Msk
  CAN_F2R1_FB3_Pos* = (3)
  CAN_F2R1_FB3_Msk* = (0x00000001 shl CAN_F2R1_FB3_Pos) ## !< 0x00000008
  CAN_F2R1_FB3* = CAN_F2R1_FB3_Msk
  CAN_F2R1_FB4_Pos* = (4)
  CAN_F2R1_FB4_Msk* = (0x00000001 shl CAN_F2R1_FB4_Pos) ## !< 0x00000010
  CAN_F2R1_FB4* = CAN_F2R1_FB4_Msk
  CAN_F2R1_FB5_Pos* = (5)
  CAN_F2R1_FB5_Msk* = (0x00000001 shl CAN_F2R1_FB5_Pos) ## !< 0x00000020
  CAN_F2R1_FB5* = CAN_F2R1_FB5_Msk
  CAN_F2R1_FB6_Pos* = (6)
  CAN_F2R1_FB6_Msk* = (0x00000001 shl CAN_F2R1_FB6_Pos) ## !< 0x00000040
  CAN_F2R1_FB6* = CAN_F2R1_FB6_Msk
  CAN_F2R1_FB7_Pos* = (7)
  CAN_F2R1_FB7_Msk* = (0x00000001 shl CAN_F2R1_FB7_Pos) ## !< 0x00000080
  CAN_F2R1_FB7* = CAN_F2R1_FB7_Msk
  CAN_F2R1_FB8_Pos* = (8)
  CAN_F2R1_FB8_Msk* = (0x00000001 shl CAN_F2R1_FB8_Pos) ## !< 0x00000100
  CAN_F2R1_FB8* = CAN_F2R1_FB8_Msk
  CAN_F2R1_FB9_Pos* = (9)
  CAN_F2R1_FB9_Msk* = (0x00000001 shl CAN_F2R1_FB9_Pos) ## !< 0x00000200
  CAN_F2R1_FB9* = CAN_F2R1_FB9_Msk
  CAN_F2R1_FB10_Pos* = (10)
  CAN_F2R1_FB10_Msk* = (0x00000001 shl CAN_F2R1_FB10_Pos) ## !< 0x00000400
  CAN_F2R1_FB10* = CAN_F2R1_FB10_Msk
  CAN_F2R1_FB11_Pos* = (11)
  CAN_F2R1_FB11_Msk* = (0x00000001 shl CAN_F2R1_FB11_Pos) ## !< 0x00000800
  CAN_F2R1_FB11* = CAN_F2R1_FB11_Msk
  CAN_F2R1_FB12_Pos* = (12)
  CAN_F2R1_FB12_Msk* = (0x00000001 shl CAN_F2R1_FB12_Pos) ## !< 0x00001000
  CAN_F2R1_FB12* = CAN_F2R1_FB12_Msk
  CAN_F2R1_FB13_Pos* = (13)
  CAN_F2R1_FB13_Msk* = (0x00000001 shl CAN_F2R1_FB13_Pos) ## !< 0x00002000
  CAN_F2R1_FB13* = CAN_F2R1_FB13_Msk
  CAN_F2R1_FB14_Pos* = (14)
  CAN_F2R1_FB14_Msk* = (0x00000001 shl CAN_F2R1_FB14_Pos) ## !< 0x00004000
  CAN_F2R1_FB14* = CAN_F2R1_FB14_Msk
  CAN_F2R1_FB15_Pos* = (15)
  CAN_F2R1_FB15_Msk* = (0x00000001 shl CAN_F2R1_FB15_Pos) ## !< 0x00008000
  CAN_F2R1_FB15* = CAN_F2R1_FB15_Msk
  CAN_F2R1_FB16_Pos* = (16)
  CAN_F2R1_FB16_Msk* = (0x00000001 shl CAN_F2R1_FB16_Pos) ## !< 0x00010000
  CAN_F2R1_FB16* = CAN_F2R1_FB16_Msk
  CAN_F2R1_FB17_Pos* = (17)
  CAN_F2R1_FB17_Msk* = (0x00000001 shl CAN_F2R1_FB17_Pos) ## !< 0x00020000
  CAN_F2R1_FB17* = CAN_F2R1_FB17_Msk
  CAN_F2R1_FB18_Pos* = (18)
  CAN_F2R1_FB18_Msk* = (0x00000001 shl CAN_F2R1_FB18_Pos) ## !< 0x00040000
  CAN_F2R1_FB18* = CAN_F2R1_FB18_Msk
  CAN_F2R1_FB19_Pos* = (19)
  CAN_F2R1_FB19_Msk* = (0x00000001 shl CAN_F2R1_FB19_Pos) ## !< 0x00080000
  CAN_F2R1_FB19* = CAN_F2R1_FB19_Msk
  CAN_F2R1_FB20_Pos* = (20)
  CAN_F2R1_FB20_Msk* = (0x00000001 shl CAN_F2R1_FB20_Pos) ## !< 0x00100000
  CAN_F2R1_FB20* = CAN_F2R1_FB20_Msk
  CAN_F2R1_FB21_Pos* = (21)
  CAN_F2R1_FB21_Msk* = (0x00000001 shl CAN_F2R1_FB21_Pos) ## !< 0x00200000
  CAN_F2R1_FB21* = CAN_F2R1_FB21_Msk
  CAN_F2R1_FB22_Pos* = (22)
  CAN_F2R1_FB22_Msk* = (0x00000001 shl CAN_F2R1_FB22_Pos) ## !< 0x00400000
  CAN_F2R1_FB22* = CAN_F2R1_FB22_Msk
  CAN_F2R1_FB23_Pos* = (23)
  CAN_F2R1_FB23_Msk* = (0x00000001 shl CAN_F2R1_FB23_Pos) ## !< 0x00800000
  CAN_F2R1_FB23* = CAN_F2R1_FB23_Msk
  CAN_F2R1_FB24_Pos* = (24)
  CAN_F2R1_FB24_Msk* = (0x00000001 shl CAN_F2R1_FB24_Pos) ## !< 0x01000000
  CAN_F2R1_FB24* = CAN_F2R1_FB24_Msk
  CAN_F2R1_FB25_Pos* = (25)
  CAN_F2R1_FB25_Msk* = (0x00000001 shl CAN_F2R1_FB25_Pos) ## !< 0x02000000
  CAN_F2R1_FB25* = CAN_F2R1_FB25_Msk
  CAN_F2R1_FB26_Pos* = (26)
  CAN_F2R1_FB26_Msk* = (0x00000001 shl CAN_F2R1_FB26_Pos) ## !< 0x04000000
  CAN_F2R1_FB26* = CAN_F2R1_FB26_Msk
  CAN_F2R1_FB27_Pos* = (27)
  CAN_F2R1_FB27_Msk* = (0x00000001 shl CAN_F2R1_FB27_Pos) ## !< 0x08000000
  CAN_F2R1_FB27* = CAN_F2R1_FB27_Msk
  CAN_F2R1_FB28_Pos* = (28)
  CAN_F2R1_FB28_Msk* = (0x00000001 shl CAN_F2R1_FB28_Pos) ## !< 0x10000000
  CAN_F2R1_FB28* = CAN_F2R1_FB28_Msk
  CAN_F2R1_FB29_Pos* = (29)
  CAN_F2R1_FB29_Msk* = (0x00000001 shl CAN_F2R1_FB29_Pos) ## !< 0x20000000
  CAN_F2R1_FB29* = CAN_F2R1_FB29_Msk
  CAN_F2R1_FB30_Pos* = (30)
  CAN_F2R1_FB30_Msk* = (0x00000001 shl CAN_F2R1_FB30_Pos) ## !< 0x40000000
  CAN_F2R1_FB30* = CAN_F2R1_FB30_Msk
  CAN_F2R1_FB31_Pos* = (31)
  CAN_F2R1_FB31_Msk* = (0x00000001 shl CAN_F2R1_FB31_Pos) ## !< 0x80000000
  CAN_F2R1_FB31* = CAN_F2R1_FB31_Msk

## ******************  Bit definition for CAN_F3R1 register  ******************

const
  CAN_F3R1_FB0_Pos* = (0)
  CAN_F3R1_FB0_Msk* = (0x00000001 shl CAN_F3R1_FB0_Pos) ## !< 0x00000001
  CAN_F3R1_FB0* = CAN_F3R1_FB0_Msk
  CAN_F3R1_FB1_Pos* = (1)
  CAN_F3R1_FB1_Msk* = (0x00000001 shl CAN_F3R1_FB1_Pos) ## !< 0x00000002
  CAN_F3R1_FB1* = CAN_F3R1_FB1_Msk
  CAN_F3R1_FB2_Pos* = (2)
  CAN_F3R1_FB2_Msk* = (0x00000001 shl CAN_F3R1_FB2_Pos) ## !< 0x00000004
  CAN_F3R1_FB2* = CAN_F3R1_FB2_Msk
  CAN_F3R1_FB3_Pos* = (3)
  CAN_F3R1_FB3_Msk* = (0x00000001 shl CAN_F3R1_FB3_Pos) ## !< 0x00000008
  CAN_F3R1_FB3* = CAN_F3R1_FB3_Msk
  CAN_F3R1_FB4_Pos* = (4)
  CAN_F3R1_FB4_Msk* = (0x00000001 shl CAN_F3R1_FB4_Pos) ## !< 0x00000010
  CAN_F3R1_FB4* = CAN_F3R1_FB4_Msk
  CAN_F3R1_FB5_Pos* = (5)
  CAN_F3R1_FB5_Msk* = (0x00000001 shl CAN_F3R1_FB5_Pos) ## !< 0x00000020
  CAN_F3R1_FB5* = CAN_F3R1_FB5_Msk
  CAN_F3R1_FB6_Pos* = (6)
  CAN_F3R1_FB6_Msk* = (0x00000001 shl CAN_F3R1_FB6_Pos) ## !< 0x00000040
  CAN_F3R1_FB6* = CAN_F3R1_FB6_Msk
  CAN_F3R1_FB7_Pos* = (7)
  CAN_F3R1_FB7_Msk* = (0x00000001 shl CAN_F3R1_FB7_Pos) ## !< 0x00000080
  CAN_F3R1_FB7* = CAN_F3R1_FB7_Msk
  CAN_F3R1_FB8_Pos* = (8)
  CAN_F3R1_FB8_Msk* = (0x00000001 shl CAN_F3R1_FB8_Pos) ## !< 0x00000100
  CAN_F3R1_FB8* = CAN_F3R1_FB8_Msk
  CAN_F3R1_FB9_Pos* = (9)
  CAN_F3R1_FB9_Msk* = (0x00000001 shl CAN_F3R1_FB9_Pos) ## !< 0x00000200
  CAN_F3R1_FB9* = CAN_F3R1_FB9_Msk
  CAN_F3R1_FB10_Pos* = (10)
  CAN_F3R1_FB10_Msk* = (0x00000001 shl CAN_F3R1_FB10_Pos) ## !< 0x00000400
  CAN_F3R1_FB10* = CAN_F3R1_FB10_Msk
  CAN_F3R1_FB11_Pos* = (11)
  CAN_F3R1_FB11_Msk* = (0x00000001 shl CAN_F3R1_FB11_Pos) ## !< 0x00000800
  CAN_F3R1_FB11* = CAN_F3R1_FB11_Msk
  CAN_F3R1_FB12_Pos* = (12)
  CAN_F3R1_FB12_Msk* = (0x00000001 shl CAN_F3R1_FB12_Pos) ## !< 0x00001000
  CAN_F3R1_FB12* = CAN_F3R1_FB12_Msk
  CAN_F3R1_FB13_Pos* = (13)
  CAN_F3R1_FB13_Msk* = (0x00000001 shl CAN_F3R1_FB13_Pos) ## !< 0x00002000
  CAN_F3R1_FB13* = CAN_F3R1_FB13_Msk
  CAN_F3R1_FB14_Pos* = (14)
  CAN_F3R1_FB14_Msk* = (0x00000001 shl CAN_F3R1_FB14_Pos) ## !< 0x00004000
  CAN_F3R1_FB14* = CAN_F3R1_FB14_Msk
  CAN_F3R1_FB15_Pos* = (15)
  CAN_F3R1_FB15_Msk* = (0x00000001 shl CAN_F3R1_FB15_Pos) ## !< 0x00008000
  CAN_F3R1_FB15* = CAN_F3R1_FB15_Msk
  CAN_F3R1_FB16_Pos* = (16)
  CAN_F3R1_FB16_Msk* = (0x00000001 shl CAN_F3R1_FB16_Pos) ## !< 0x00010000
  CAN_F3R1_FB16* = CAN_F3R1_FB16_Msk
  CAN_F3R1_FB17_Pos* = (17)
  CAN_F3R1_FB17_Msk* = (0x00000001 shl CAN_F3R1_FB17_Pos) ## !< 0x00020000
  CAN_F3R1_FB17* = CAN_F3R1_FB17_Msk
  CAN_F3R1_FB18_Pos* = (18)
  CAN_F3R1_FB18_Msk* = (0x00000001 shl CAN_F3R1_FB18_Pos) ## !< 0x00040000
  CAN_F3R1_FB18* = CAN_F3R1_FB18_Msk
  CAN_F3R1_FB19_Pos* = (19)
  CAN_F3R1_FB19_Msk* = (0x00000001 shl CAN_F3R1_FB19_Pos) ## !< 0x00080000
  CAN_F3R1_FB19* = CAN_F3R1_FB19_Msk
  CAN_F3R1_FB20_Pos* = (20)
  CAN_F3R1_FB20_Msk* = (0x00000001 shl CAN_F3R1_FB20_Pos) ## !< 0x00100000
  CAN_F3R1_FB20* = CAN_F3R1_FB20_Msk
  CAN_F3R1_FB21_Pos* = (21)
  CAN_F3R1_FB21_Msk* = (0x00000001 shl CAN_F3R1_FB21_Pos) ## !< 0x00200000
  CAN_F3R1_FB21* = CAN_F3R1_FB21_Msk
  CAN_F3R1_FB22_Pos* = (22)
  CAN_F3R1_FB22_Msk* = (0x00000001 shl CAN_F3R1_FB22_Pos) ## !< 0x00400000
  CAN_F3R1_FB22* = CAN_F3R1_FB22_Msk
  CAN_F3R1_FB23_Pos* = (23)
  CAN_F3R1_FB23_Msk* = (0x00000001 shl CAN_F3R1_FB23_Pos) ## !< 0x00800000
  CAN_F3R1_FB23* = CAN_F3R1_FB23_Msk
  CAN_F3R1_FB24_Pos* = (24)
  CAN_F3R1_FB24_Msk* = (0x00000001 shl CAN_F3R1_FB24_Pos) ## !< 0x01000000
  CAN_F3R1_FB24* = CAN_F3R1_FB24_Msk
  CAN_F3R1_FB25_Pos* = (25)
  CAN_F3R1_FB25_Msk* = (0x00000001 shl CAN_F3R1_FB25_Pos) ## !< 0x02000000
  CAN_F3R1_FB25* = CAN_F3R1_FB25_Msk
  CAN_F3R1_FB26_Pos* = (26)
  CAN_F3R1_FB26_Msk* = (0x00000001 shl CAN_F3R1_FB26_Pos) ## !< 0x04000000
  CAN_F3R1_FB26* = CAN_F3R1_FB26_Msk
  CAN_F3R1_FB27_Pos* = (27)
  CAN_F3R1_FB27_Msk* = (0x00000001 shl CAN_F3R1_FB27_Pos) ## !< 0x08000000
  CAN_F3R1_FB27* = CAN_F3R1_FB27_Msk
  CAN_F3R1_FB28_Pos* = (28)
  CAN_F3R1_FB28_Msk* = (0x00000001 shl CAN_F3R1_FB28_Pos) ## !< 0x10000000
  CAN_F3R1_FB28* = CAN_F3R1_FB28_Msk
  CAN_F3R1_FB29_Pos* = (29)
  CAN_F3R1_FB29_Msk* = (0x00000001 shl CAN_F3R1_FB29_Pos) ## !< 0x20000000
  CAN_F3R1_FB29* = CAN_F3R1_FB29_Msk
  CAN_F3R1_FB30_Pos* = (30)
  CAN_F3R1_FB30_Msk* = (0x00000001 shl CAN_F3R1_FB30_Pos) ## !< 0x40000000
  CAN_F3R1_FB30* = CAN_F3R1_FB30_Msk
  CAN_F3R1_FB31_Pos* = (31)
  CAN_F3R1_FB31_Msk* = (0x00000001 shl CAN_F3R1_FB31_Pos) ## !< 0x80000000
  CAN_F3R1_FB31* = CAN_F3R1_FB31_Msk

## ******************  Bit definition for CAN_F4R1 register  ******************

const
  CAN_F4R1_FB0_Pos* = (0)
  CAN_F4R1_FB0_Msk* = (0x00000001 shl CAN_F4R1_FB0_Pos) ## !< 0x00000001
  CAN_F4R1_FB0* = CAN_F4R1_FB0_Msk
  CAN_F4R1_FB1_Pos* = (1)
  CAN_F4R1_FB1_Msk* = (0x00000001 shl CAN_F4R1_FB1_Pos) ## !< 0x00000002
  CAN_F4R1_FB1* = CAN_F4R1_FB1_Msk
  CAN_F4R1_FB2_Pos* = (2)
  CAN_F4R1_FB2_Msk* = (0x00000001 shl CAN_F4R1_FB2_Pos) ## !< 0x00000004
  CAN_F4R1_FB2* = CAN_F4R1_FB2_Msk
  CAN_F4R1_FB3_Pos* = (3)
  CAN_F4R1_FB3_Msk* = (0x00000001 shl CAN_F4R1_FB3_Pos) ## !< 0x00000008
  CAN_F4R1_FB3* = CAN_F4R1_FB3_Msk
  CAN_F4R1_FB4_Pos* = (4)
  CAN_F4R1_FB4_Msk* = (0x00000001 shl CAN_F4R1_FB4_Pos) ## !< 0x00000010
  CAN_F4R1_FB4* = CAN_F4R1_FB4_Msk
  CAN_F4R1_FB5_Pos* = (5)
  CAN_F4R1_FB5_Msk* = (0x00000001 shl CAN_F4R1_FB5_Pos) ## !< 0x00000020
  CAN_F4R1_FB5* = CAN_F4R1_FB5_Msk
  CAN_F4R1_FB6_Pos* = (6)
  CAN_F4R1_FB6_Msk* = (0x00000001 shl CAN_F4R1_FB6_Pos) ## !< 0x00000040
  CAN_F4R1_FB6* = CAN_F4R1_FB6_Msk
  CAN_F4R1_FB7_Pos* = (7)
  CAN_F4R1_FB7_Msk* = (0x00000001 shl CAN_F4R1_FB7_Pos) ## !< 0x00000080
  CAN_F4R1_FB7* = CAN_F4R1_FB7_Msk
  CAN_F4R1_FB8_Pos* = (8)
  CAN_F4R1_FB8_Msk* = (0x00000001 shl CAN_F4R1_FB8_Pos) ## !< 0x00000100
  CAN_F4R1_FB8* = CAN_F4R1_FB8_Msk
  CAN_F4R1_FB9_Pos* = (9)
  CAN_F4R1_FB9_Msk* = (0x00000001 shl CAN_F4R1_FB9_Pos) ## !< 0x00000200
  CAN_F4R1_FB9* = CAN_F4R1_FB9_Msk
  CAN_F4R1_FB10_Pos* = (10)
  CAN_F4R1_FB10_Msk* = (0x00000001 shl CAN_F4R1_FB10_Pos) ## !< 0x00000400
  CAN_F4R1_FB10* = CAN_F4R1_FB10_Msk
  CAN_F4R1_FB11_Pos* = (11)
  CAN_F4R1_FB11_Msk* = (0x00000001 shl CAN_F4R1_FB11_Pos) ## !< 0x00000800
  CAN_F4R1_FB11* = CAN_F4R1_FB11_Msk
  CAN_F4R1_FB12_Pos* = (12)
  CAN_F4R1_FB12_Msk* = (0x00000001 shl CAN_F4R1_FB12_Pos) ## !< 0x00001000
  CAN_F4R1_FB12* = CAN_F4R1_FB12_Msk
  CAN_F4R1_FB13_Pos* = (13)
  CAN_F4R1_FB13_Msk* = (0x00000001 shl CAN_F4R1_FB13_Pos) ## !< 0x00002000
  CAN_F4R1_FB13* = CAN_F4R1_FB13_Msk
  CAN_F4R1_FB14_Pos* = (14)
  CAN_F4R1_FB14_Msk* = (0x00000001 shl CAN_F4R1_FB14_Pos) ## !< 0x00004000
  CAN_F4R1_FB14* = CAN_F4R1_FB14_Msk
  CAN_F4R1_FB15_Pos* = (15)
  CAN_F4R1_FB15_Msk* = (0x00000001 shl CAN_F4R1_FB15_Pos) ## !< 0x00008000
  CAN_F4R1_FB15* = CAN_F4R1_FB15_Msk
  CAN_F4R1_FB16_Pos* = (16)
  CAN_F4R1_FB16_Msk* = (0x00000001 shl CAN_F4R1_FB16_Pos) ## !< 0x00010000
  CAN_F4R1_FB16* = CAN_F4R1_FB16_Msk
  CAN_F4R1_FB17_Pos* = (17)
  CAN_F4R1_FB17_Msk* = (0x00000001 shl CAN_F4R1_FB17_Pos) ## !< 0x00020000
  CAN_F4R1_FB17* = CAN_F4R1_FB17_Msk
  CAN_F4R1_FB18_Pos* = (18)
  CAN_F4R1_FB18_Msk* = (0x00000001 shl CAN_F4R1_FB18_Pos) ## !< 0x00040000
  CAN_F4R1_FB18* = CAN_F4R1_FB18_Msk
  CAN_F4R1_FB19_Pos* = (19)
  CAN_F4R1_FB19_Msk* = (0x00000001 shl CAN_F4R1_FB19_Pos) ## !< 0x00080000
  CAN_F4R1_FB19* = CAN_F4R1_FB19_Msk
  CAN_F4R1_FB20_Pos* = (20)
  CAN_F4R1_FB20_Msk* = (0x00000001 shl CAN_F4R1_FB20_Pos) ## !< 0x00100000
  CAN_F4R1_FB20* = CAN_F4R1_FB20_Msk
  CAN_F4R1_FB21_Pos* = (21)
  CAN_F4R1_FB21_Msk* = (0x00000001 shl CAN_F4R1_FB21_Pos) ## !< 0x00200000
  CAN_F4R1_FB21* = CAN_F4R1_FB21_Msk
  CAN_F4R1_FB22_Pos* = (22)
  CAN_F4R1_FB22_Msk* = (0x00000001 shl CAN_F4R1_FB22_Pos) ## !< 0x00400000
  CAN_F4R1_FB22* = CAN_F4R1_FB22_Msk
  CAN_F4R1_FB23_Pos* = (23)
  CAN_F4R1_FB23_Msk* = (0x00000001 shl CAN_F4R1_FB23_Pos) ## !< 0x00800000
  CAN_F4R1_FB23* = CAN_F4R1_FB23_Msk
  CAN_F4R1_FB24_Pos* = (24)
  CAN_F4R1_FB24_Msk* = (0x00000001 shl CAN_F4R1_FB24_Pos) ## !< 0x01000000
  CAN_F4R1_FB24* = CAN_F4R1_FB24_Msk
  CAN_F4R1_FB25_Pos* = (25)
  CAN_F4R1_FB25_Msk* = (0x00000001 shl CAN_F4R1_FB25_Pos) ## !< 0x02000000
  CAN_F4R1_FB25* = CAN_F4R1_FB25_Msk
  CAN_F4R1_FB26_Pos* = (26)
  CAN_F4R1_FB26_Msk* = (0x00000001 shl CAN_F4R1_FB26_Pos) ## !< 0x04000000
  CAN_F4R1_FB26* = CAN_F4R1_FB26_Msk
  CAN_F4R1_FB27_Pos* = (27)
  CAN_F4R1_FB27_Msk* = (0x00000001 shl CAN_F4R1_FB27_Pos) ## !< 0x08000000
  CAN_F4R1_FB27* = CAN_F4R1_FB27_Msk
  CAN_F4R1_FB28_Pos* = (28)
  CAN_F4R1_FB28_Msk* = (0x00000001 shl CAN_F4R1_FB28_Pos) ## !< 0x10000000
  CAN_F4R1_FB28* = CAN_F4R1_FB28_Msk
  CAN_F4R1_FB29_Pos* = (29)
  CAN_F4R1_FB29_Msk* = (0x00000001 shl CAN_F4R1_FB29_Pos) ## !< 0x20000000
  CAN_F4R1_FB29* = CAN_F4R1_FB29_Msk
  CAN_F4R1_FB30_Pos* = (30)
  CAN_F4R1_FB30_Msk* = (0x00000001 shl CAN_F4R1_FB30_Pos) ## !< 0x40000000
  CAN_F4R1_FB30* = CAN_F4R1_FB30_Msk
  CAN_F4R1_FB31_Pos* = (31)
  CAN_F4R1_FB31_Msk* = (0x00000001 shl CAN_F4R1_FB31_Pos) ## !< 0x80000000
  CAN_F4R1_FB31* = CAN_F4R1_FB31_Msk

## ******************  Bit definition for CAN_F5R1 register  ******************

const
  CAN_F5R1_FB0_Pos* = (0)
  CAN_F5R1_FB0_Msk* = (0x00000001 shl CAN_F5R1_FB0_Pos) ## !< 0x00000001
  CAN_F5R1_FB0* = CAN_F5R1_FB0_Msk
  CAN_F5R1_FB1_Pos* = (1)
  CAN_F5R1_FB1_Msk* = (0x00000001 shl CAN_F5R1_FB1_Pos) ## !< 0x00000002
  CAN_F5R1_FB1* = CAN_F5R1_FB1_Msk
  CAN_F5R1_FB2_Pos* = (2)
  CAN_F5R1_FB2_Msk* = (0x00000001 shl CAN_F5R1_FB2_Pos) ## !< 0x00000004
  CAN_F5R1_FB2* = CAN_F5R1_FB2_Msk
  CAN_F5R1_FB3_Pos* = (3)
  CAN_F5R1_FB3_Msk* = (0x00000001 shl CAN_F5R1_FB3_Pos) ## !< 0x00000008
  CAN_F5R1_FB3* = CAN_F5R1_FB3_Msk
  CAN_F5R1_FB4_Pos* = (4)
  CAN_F5R1_FB4_Msk* = (0x00000001 shl CAN_F5R1_FB4_Pos) ## !< 0x00000010
  CAN_F5R1_FB4* = CAN_F5R1_FB4_Msk
  CAN_F5R1_FB5_Pos* = (5)
  CAN_F5R1_FB5_Msk* = (0x00000001 shl CAN_F5R1_FB5_Pos) ## !< 0x00000020
  CAN_F5R1_FB5* = CAN_F5R1_FB5_Msk
  CAN_F5R1_FB6_Pos* = (6)
  CAN_F5R1_FB6_Msk* = (0x00000001 shl CAN_F5R1_FB6_Pos) ## !< 0x00000040
  CAN_F5R1_FB6* = CAN_F5R1_FB6_Msk
  CAN_F5R1_FB7_Pos* = (7)
  CAN_F5R1_FB7_Msk* = (0x00000001 shl CAN_F5R1_FB7_Pos) ## !< 0x00000080
  CAN_F5R1_FB7* = CAN_F5R1_FB7_Msk
  CAN_F5R1_FB8_Pos* = (8)
  CAN_F5R1_FB8_Msk* = (0x00000001 shl CAN_F5R1_FB8_Pos) ## !< 0x00000100
  CAN_F5R1_FB8* = CAN_F5R1_FB8_Msk
  CAN_F5R1_FB9_Pos* = (9)
  CAN_F5R1_FB9_Msk* = (0x00000001 shl CAN_F5R1_FB9_Pos) ## !< 0x00000200
  CAN_F5R1_FB9* = CAN_F5R1_FB9_Msk
  CAN_F5R1_FB10_Pos* = (10)
  CAN_F5R1_FB10_Msk* = (0x00000001 shl CAN_F5R1_FB10_Pos) ## !< 0x00000400
  CAN_F5R1_FB10* = CAN_F5R1_FB10_Msk
  CAN_F5R1_FB11_Pos* = (11)
  CAN_F5R1_FB11_Msk* = (0x00000001 shl CAN_F5R1_FB11_Pos) ## !< 0x00000800
  CAN_F5R1_FB11* = CAN_F5R1_FB11_Msk
  CAN_F5R1_FB12_Pos* = (12)
  CAN_F5R1_FB12_Msk* = (0x00000001 shl CAN_F5R1_FB12_Pos) ## !< 0x00001000
  CAN_F5R1_FB12* = CAN_F5R1_FB12_Msk
  CAN_F5R1_FB13_Pos* = (13)
  CAN_F5R1_FB13_Msk* = (0x00000001 shl CAN_F5R1_FB13_Pos) ## !< 0x00002000
  CAN_F5R1_FB13* = CAN_F5R1_FB13_Msk
  CAN_F5R1_FB14_Pos* = (14)
  CAN_F5R1_FB14_Msk* = (0x00000001 shl CAN_F5R1_FB14_Pos) ## !< 0x00004000
  CAN_F5R1_FB14* = CAN_F5R1_FB14_Msk
  CAN_F5R1_FB15_Pos* = (15)
  CAN_F5R1_FB15_Msk* = (0x00000001 shl CAN_F5R1_FB15_Pos) ## !< 0x00008000
  CAN_F5R1_FB15* = CAN_F5R1_FB15_Msk
  CAN_F5R1_FB16_Pos* = (16)
  CAN_F5R1_FB16_Msk* = (0x00000001 shl CAN_F5R1_FB16_Pos) ## !< 0x00010000
  CAN_F5R1_FB16* = CAN_F5R1_FB16_Msk
  CAN_F5R1_FB17_Pos* = (17)
  CAN_F5R1_FB17_Msk* = (0x00000001 shl CAN_F5R1_FB17_Pos) ## !< 0x00020000
  CAN_F5R1_FB17* = CAN_F5R1_FB17_Msk
  CAN_F5R1_FB18_Pos* = (18)
  CAN_F5R1_FB18_Msk* = (0x00000001 shl CAN_F5R1_FB18_Pos) ## !< 0x00040000
  CAN_F5R1_FB18* = CAN_F5R1_FB18_Msk
  CAN_F5R1_FB19_Pos* = (19)
  CAN_F5R1_FB19_Msk* = (0x00000001 shl CAN_F5R1_FB19_Pos) ## !< 0x00080000
  CAN_F5R1_FB19* = CAN_F5R1_FB19_Msk
  CAN_F5R1_FB20_Pos* = (20)
  CAN_F5R1_FB20_Msk* = (0x00000001 shl CAN_F5R1_FB20_Pos) ## !< 0x00100000
  CAN_F5R1_FB20* = CAN_F5R1_FB20_Msk
  CAN_F5R1_FB21_Pos* = (21)
  CAN_F5R1_FB21_Msk* = (0x00000001 shl CAN_F5R1_FB21_Pos) ## !< 0x00200000
  CAN_F5R1_FB21* = CAN_F5R1_FB21_Msk
  CAN_F5R1_FB22_Pos* = (22)
  CAN_F5R1_FB22_Msk* = (0x00000001 shl CAN_F5R1_FB22_Pos) ## !< 0x00400000
  CAN_F5R1_FB22* = CAN_F5R1_FB22_Msk
  CAN_F5R1_FB23_Pos* = (23)
  CAN_F5R1_FB23_Msk* = (0x00000001 shl CAN_F5R1_FB23_Pos) ## !< 0x00800000
  CAN_F5R1_FB23* = CAN_F5R1_FB23_Msk
  CAN_F5R1_FB24_Pos* = (24)
  CAN_F5R1_FB24_Msk* = (0x00000001 shl CAN_F5R1_FB24_Pos) ## !< 0x01000000
  CAN_F5R1_FB24* = CAN_F5R1_FB24_Msk
  CAN_F5R1_FB25_Pos* = (25)
  CAN_F5R1_FB25_Msk* = (0x00000001 shl CAN_F5R1_FB25_Pos) ## !< 0x02000000
  CAN_F5R1_FB25* = CAN_F5R1_FB25_Msk
  CAN_F5R1_FB26_Pos* = (26)
  CAN_F5R1_FB26_Msk* = (0x00000001 shl CAN_F5R1_FB26_Pos) ## !< 0x04000000
  CAN_F5R1_FB26* = CAN_F5R1_FB26_Msk
  CAN_F5R1_FB27_Pos* = (27)
  CAN_F5R1_FB27_Msk* = (0x00000001 shl CAN_F5R1_FB27_Pos) ## !< 0x08000000
  CAN_F5R1_FB27* = CAN_F5R1_FB27_Msk
  CAN_F5R1_FB28_Pos* = (28)
  CAN_F5R1_FB28_Msk* = (0x00000001 shl CAN_F5R1_FB28_Pos) ## !< 0x10000000
  CAN_F5R1_FB28* = CAN_F5R1_FB28_Msk
  CAN_F5R1_FB29_Pos* = (29)
  CAN_F5R1_FB29_Msk* = (0x00000001 shl CAN_F5R1_FB29_Pos) ## !< 0x20000000
  CAN_F5R1_FB29* = CAN_F5R1_FB29_Msk
  CAN_F5R1_FB30_Pos* = (30)
  CAN_F5R1_FB30_Msk* = (0x00000001 shl CAN_F5R1_FB30_Pos) ## !< 0x40000000
  CAN_F5R1_FB30* = CAN_F5R1_FB30_Msk
  CAN_F5R1_FB31_Pos* = (31)
  CAN_F5R1_FB31_Msk* = (0x00000001 shl CAN_F5R1_FB31_Pos) ## !< 0x80000000
  CAN_F5R1_FB31* = CAN_F5R1_FB31_Msk

## ******************  Bit definition for CAN_F6R1 register  ******************

const
  CAN_F6R1_FB0_Pos* = (0)
  CAN_F6R1_FB0_Msk* = (0x00000001 shl CAN_F6R1_FB0_Pos) ## !< 0x00000001
  CAN_F6R1_FB0* = CAN_F6R1_FB0_Msk
  CAN_F6R1_FB1_Pos* = (1)
  CAN_F6R1_FB1_Msk* = (0x00000001 shl CAN_F6R1_FB1_Pos) ## !< 0x00000002
  CAN_F6R1_FB1* = CAN_F6R1_FB1_Msk
  CAN_F6R1_FB2_Pos* = (2)
  CAN_F6R1_FB2_Msk* = (0x00000001 shl CAN_F6R1_FB2_Pos) ## !< 0x00000004
  CAN_F6R1_FB2* = CAN_F6R1_FB2_Msk
  CAN_F6R1_FB3_Pos* = (3)
  CAN_F6R1_FB3_Msk* = (0x00000001 shl CAN_F6R1_FB3_Pos) ## !< 0x00000008
  CAN_F6R1_FB3* = CAN_F6R1_FB3_Msk
  CAN_F6R1_FB4_Pos* = (4)
  CAN_F6R1_FB4_Msk* = (0x00000001 shl CAN_F6R1_FB4_Pos) ## !< 0x00000010
  CAN_F6R1_FB4* = CAN_F6R1_FB4_Msk
  CAN_F6R1_FB5_Pos* = (5)
  CAN_F6R1_FB5_Msk* = (0x00000001 shl CAN_F6R1_FB5_Pos) ## !< 0x00000020
  CAN_F6R1_FB5* = CAN_F6R1_FB5_Msk
  CAN_F6R1_FB6_Pos* = (6)
  CAN_F6R1_FB6_Msk* = (0x00000001 shl CAN_F6R1_FB6_Pos) ## !< 0x00000040
  CAN_F6R1_FB6* = CAN_F6R1_FB6_Msk
  CAN_F6R1_FB7_Pos* = (7)
  CAN_F6R1_FB7_Msk* = (0x00000001 shl CAN_F6R1_FB7_Pos) ## !< 0x00000080
  CAN_F6R1_FB7* = CAN_F6R1_FB7_Msk
  CAN_F6R1_FB8_Pos* = (8)
  CAN_F6R1_FB8_Msk* = (0x00000001 shl CAN_F6R1_FB8_Pos) ## !< 0x00000100
  CAN_F6R1_FB8* = CAN_F6R1_FB8_Msk
  CAN_F6R1_FB9_Pos* = (9)
  CAN_F6R1_FB9_Msk* = (0x00000001 shl CAN_F6R1_FB9_Pos) ## !< 0x00000200
  CAN_F6R1_FB9* = CAN_F6R1_FB9_Msk
  CAN_F6R1_FB10_Pos* = (10)
  CAN_F6R1_FB10_Msk* = (0x00000001 shl CAN_F6R1_FB10_Pos) ## !< 0x00000400
  CAN_F6R1_FB10* = CAN_F6R1_FB10_Msk
  CAN_F6R1_FB11_Pos* = (11)
  CAN_F6R1_FB11_Msk* = (0x00000001 shl CAN_F6R1_FB11_Pos) ## !< 0x00000800
  CAN_F6R1_FB11* = CAN_F6R1_FB11_Msk
  CAN_F6R1_FB12_Pos* = (12)
  CAN_F6R1_FB12_Msk* = (0x00000001 shl CAN_F6R1_FB12_Pos) ## !< 0x00001000
  CAN_F6R1_FB12* = CAN_F6R1_FB12_Msk
  CAN_F6R1_FB13_Pos* = (13)
  CAN_F6R1_FB13_Msk* = (0x00000001 shl CAN_F6R1_FB13_Pos) ## !< 0x00002000
  CAN_F6R1_FB13* = CAN_F6R1_FB13_Msk
  CAN_F6R1_FB14_Pos* = (14)
  CAN_F6R1_FB14_Msk* = (0x00000001 shl CAN_F6R1_FB14_Pos) ## !< 0x00004000
  CAN_F6R1_FB14* = CAN_F6R1_FB14_Msk
  CAN_F6R1_FB15_Pos* = (15)
  CAN_F6R1_FB15_Msk* = (0x00000001 shl CAN_F6R1_FB15_Pos) ## !< 0x00008000
  CAN_F6R1_FB15* = CAN_F6R1_FB15_Msk
  CAN_F6R1_FB16_Pos* = (16)
  CAN_F6R1_FB16_Msk* = (0x00000001 shl CAN_F6R1_FB16_Pos) ## !< 0x00010000
  CAN_F6R1_FB16* = CAN_F6R1_FB16_Msk
  CAN_F6R1_FB17_Pos* = (17)
  CAN_F6R1_FB17_Msk* = (0x00000001 shl CAN_F6R1_FB17_Pos) ## !< 0x00020000
  CAN_F6R1_FB17* = CAN_F6R1_FB17_Msk
  CAN_F6R1_FB18_Pos* = (18)
  CAN_F6R1_FB18_Msk* = (0x00000001 shl CAN_F6R1_FB18_Pos) ## !< 0x00040000
  CAN_F6R1_FB18* = CAN_F6R1_FB18_Msk
  CAN_F6R1_FB19_Pos* = (19)
  CAN_F6R1_FB19_Msk* = (0x00000001 shl CAN_F6R1_FB19_Pos) ## !< 0x00080000
  CAN_F6R1_FB19* = CAN_F6R1_FB19_Msk
  CAN_F6R1_FB20_Pos* = (20)
  CAN_F6R1_FB20_Msk* = (0x00000001 shl CAN_F6R1_FB20_Pos) ## !< 0x00100000
  CAN_F6R1_FB20* = CAN_F6R1_FB20_Msk
  CAN_F6R1_FB21_Pos* = (21)
  CAN_F6R1_FB21_Msk* = (0x00000001 shl CAN_F6R1_FB21_Pos) ## !< 0x00200000
  CAN_F6R1_FB21* = CAN_F6R1_FB21_Msk
  CAN_F6R1_FB22_Pos* = (22)
  CAN_F6R1_FB22_Msk* = (0x00000001 shl CAN_F6R1_FB22_Pos) ## !< 0x00400000
  CAN_F6R1_FB22* = CAN_F6R1_FB22_Msk
  CAN_F6R1_FB23_Pos* = (23)
  CAN_F6R1_FB23_Msk* = (0x00000001 shl CAN_F6R1_FB23_Pos) ## !< 0x00800000
  CAN_F6R1_FB23* = CAN_F6R1_FB23_Msk
  CAN_F6R1_FB24_Pos* = (24)
  CAN_F6R1_FB24_Msk* = (0x00000001 shl CAN_F6R1_FB24_Pos) ## !< 0x01000000
  CAN_F6R1_FB24* = CAN_F6R1_FB24_Msk
  CAN_F6R1_FB25_Pos* = (25)
  CAN_F6R1_FB25_Msk* = (0x00000001 shl CAN_F6R1_FB25_Pos) ## !< 0x02000000
  CAN_F6R1_FB25* = CAN_F6R1_FB25_Msk
  CAN_F6R1_FB26_Pos* = (26)
  CAN_F6R1_FB26_Msk* = (0x00000001 shl CAN_F6R1_FB26_Pos) ## !< 0x04000000
  CAN_F6R1_FB26* = CAN_F6R1_FB26_Msk
  CAN_F6R1_FB27_Pos* = (27)
  CAN_F6R1_FB27_Msk* = (0x00000001 shl CAN_F6R1_FB27_Pos) ## !< 0x08000000
  CAN_F6R1_FB27* = CAN_F6R1_FB27_Msk
  CAN_F6R1_FB28_Pos* = (28)
  CAN_F6R1_FB28_Msk* = (0x00000001 shl CAN_F6R1_FB28_Pos) ## !< 0x10000000
  CAN_F6R1_FB28* = CAN_F6R1_FB28_Msk
  CAN_F6R1_FB29_Pos* = (29)
  CAN_F6R1_FB29_Msk* = (0x00000001 shl CAN_F6R1_FB29_Pos) ## !< 0x20000000
  CAN_F6R1_FB29* = CAN_F6R1_FB29_Msk
  CAN_F6R1_FB30_Pos* = (30)
  CAN_F6R1_FB30_Msk* = (0x00000001 shl CAN_F6R1_FB30_Pos) ## !< 0x40000000
  CAN_F6R1_FB30* = CAN_F6R1_FB30_Msk
  CAN_F6R1_FB31_Pos* = (31)
  CAN_F6R1_FB31_Msk* = (0x00000001 shl CAN_F6R1_FB31_Pos) ## !< 0x80000000
  CAN_F6R1_FB31* = CAN_F6R1_FB31_Msk

## ******************  Bit definition for CAN_F7R1 register  ******************

const
  CAN_F7R1_FB0_Pos* = (0)
  CAN_F7R1_FB0_Msk* = (0x00000001 shl CAN_F7R1_FB0_Pos) ## !< 0x00000001
  CAN_F7R1_FB0* = CAN_F7R1_FB0_Msk
  CAN_F7R1_FB1_Pos* = (1)
  CAN_F7R1_FB1_Msk* = (0x00000001 shl CAN_F7R1_FB1_Pos) ## !< 0x00000002
  CAN_F7R1_FB1* = CAN_F7R1_FB1_Msk
  CAN_F7R1_FB2_Pos* = (2)
  CAN_F7R1_FB2_Msk* = (0x00000001 shl CAN_F7R1_FB2_Pos) ## !< 0x00000004
  CAN_F7R1_FB2* = CAN_F7R1_FB2_Msk
  CAN_F7R1_FB3_Pos* = (3)
  CAN_F7R1_FB3_Msk* = (0x00000001 shl CAN_F7R1_FB3_Pos) ## !< 0x00000008
  CAN_F7R1_FB3* = CAN_F7R1_FB3_Msk
  CAN_F7R1_FB4_Pos* = (4)
  CAN_F7R1_FB4_Msk* = (0x00000001 shl CAN_F7R1_FB4_Pos) ## !< 0x00000010
  CAN_F7R1_FB4* = CAN_F7R1_FB4_Msk
  CAN_F7R1_FB5_Pos* = (5)
  CAN_F7R1_FB5_Msk* = (0x00000001 shl CAN_F7R1_FB5_Pos) ## !< 0x00000020
  CAN_F7R1_FB5* = CAN_F7R1_FB5_Msk
  CAN_F7R1_FB6_Pos* = (6)
  CAN_F7R1_FB6_Msk* = (0x00000001 shl CAN_F7R1_FB6_Pos) ## !< 0x00000040
  CAN_F7R1_FB6* = CAN_F7R1_FB6_Msk
  CAN_F7R1_FB7_Pos* = (7)
  CAN_F7R1_FB7_Msk* = (0x00000001 shl CAN_F7R1_FB7_Pos) ## !< 0x00000080
  CAN_F7R1_FB7* = CAN_F7R1_FB7_Msk
  CAN_F7R1_FB8_Pos* = (8)
  CAN_F7R1_FB8_Msk* = (0x00000001 shl CAN_F7R1_FB8_Pos) ## !< 0x00000100
  CAN_F7R1_FB8* = CAN_F7R1_FB8_Msk
  CAN_F7R1_FB9_Pos* = (9)
  CAN_F7R1_FB9_Msk* = (0x00000001 shl CAN_F7R1_FB9_Pos) ## !< 0x00000200
  CAN_F7R1_FB9* = CAN_F7R1_FB9_Msk
  CAN_F7R1_FB10_Pos* = (10)
  CAN_F7R1_FB10_Msk* = (0x00000001 shl CAN_F7R1_FB10_Pos) ## !< 0x00000400
  CAN_F7R1_FB10* = CAN_F7R1_FB10_Msk
  CAN_F7R1_FB11_Pos* = (11)
  CAN_F7R1_FB11_Msk* = (0x00000001 shl CAN_F7R1_FB11_Pos) ## !< 0x00000800
  CAN_F7R1_FB11* = CAN_F7R1_FB11_Msk
  CAN_F7R1_FB12_Pos* = (12)
  CAN_F7R1_FB12_Msk* = (0x00000001 shl CAN_F7R1_FB12_Pos) ## !< 0x00001000
  CAN_F7R1_FB12* = CAN_F7R1_FB12_Msk
  CAN_F7R1_FB13_Pos* = (13)
  CAN_F7R1_FB13_Msk* = (0x00000001 shl CAN_F7R1_FB13_Pos) ## !< 0x00002000
  CAN_F7R1_FB13* = CAN_F7R1_FB13_Msk
  CAN_F7R1_FB14_Pos* = (14)
  CAN_F7R1_FB14_Msk* = (0x00000001 shl CAN_F7R1_FB14_Pos) ## !< 0x00004000
  CAN_F7R1_FB14* = CAN_F7R1_FB14_Msk
  CAN_F7R1_FB15_Pos* = (15)
  CAN_F7R1_FB15_Msk* = (0x00000001 shl CAN_F7R1_FB15_Pos) ## !< 0x00008000
  CAN_F7R1_FB15* = CAN_F7R1_FB15_Msk
  CAN_F7R1_FB16_Pos* = (16)
  CAN_F7R1_FB16_Msk* = (0x00000001 shl CAN_F7R1_FB16_Pos) ## !< 0x00010000
  CAN_F7R1_FB16* = CAN_F7R1_FB16_Msk
  CAN_F7R1_FB17_Pos* = (17)
  CAN_F7R1_FB17_Msk* = (0x00000001 shl CAN_F7R1_FB17_Pos) ## !< 0x00020000
  CAN_F7R1_FB17* = CAN_F7R1_FB17_Msk
  CAN_F7R1_FB18_Pos* = (18)
  CAN_F7R1_FB18_Msk* = (0x00000001 shl CAN_F7R1_FB18_Pos) ## !< 0x00040000
  CAN_F7R1_FB18* = CAN_F7R1_FB18_Msk
  CAN_F7R1_FB19_Pos* = (19)
  CAN_F7R1_FB19_Msk* = (0x00000001 shl CAN_F7R1_FB19_Pos) ## !< 0x00080000
  CAN_F7R1_FB19* = CAN_F7R1_FB19_Msk
  CAN_F7R1_FB20_Pos* = (20)
  CAN_F7R1_FB20_Msk* = (0x00000001 shl CAN_F7R1_FB20_Pos) ## !< 0x00100000
  CAN_F7R1_FB20* = CAN_F7R1_FB20_Msk
  CAN_F7R1_FB21_Pos* = (21)
  CAN_F7R1_FB21_Msk* = (0x00000001 shl CAN_F7R1_FB21_Pos) ## !< 0x00200000
  CAN_F7R1_FB21* = CAN_F7R1_FB21_Msk
  CAN_F7R1_FB22_Pos* = (22)
  CAN_F7R1_FB22_Msk* = (0x00000001 shl CAN_F7R1_FB22_Pos) ## !< 0x00400000
  CAN_F7R1_FB22* = CAN_F7R1_FB22_Msk
  CAN_F7R1_FB23_Pos* = (23)
  CAN_F7R1_FB23_Msk* = (0x00000001 shl CAN_F7R1_FB23_Pos) ## !< 0x00800000
  CAN_F7R1_FB23* = CAN_F7R1_FB23_Msk
  CAN_F7R1_FB24_Pos* = (24)
  CAN_F7R1_FB24_Msk* = (0x00000001 shl CAN_F7R1_FB24_Pos) ## !< 0x01000000
  CAN_F7R1_FB24* = CAN_F7R1_FB24_Msk
  CAN_F7R1_FB25_Pos* = (25)
  CAN_F7R1_FB25_Msk* = (0x00000001 shl CAN_F7R1_FB25_Pos) ## !< 0x02000000
  CAN_F7R1_FB25* = CAN_F7R1_FB25_Msk
  CAN_F7R1_FB26_Pos* = (26)
  CAN_F7R1_FB26_Msk* = (0x00000001 shl CAN_F7R1_FB26_Pos) ## !< 0x04000000
  CAN_F7R1_FB26* = CAN_F7R1_FB26_Msk
  CAN_F7R1_FB27_Pos* = (27)
  CAN_F7R1_FB27_Msk* = (0x00000001 shl CAN_F7R1_FB27_Pos) ## !< 0x08000000
  CAN_F7R1_FB27* = CAN_F7R1_FB27_Msk
  CAN_F7R1_FB28_Pos* = (28)
  CAN_F7R1_FB28_Msk* = (0x00000001 shl CAN_F7R1_FB28_Pos) ## !< 0x10000000
  CAN_F7R1_FB28* = CAN_F7R1_FB28_Msk
  CAN_F7R1_FB29_Pos* = (29)
  CAN_F7R1_FB29_Msk* = (0x00000001 shl CAN_F7R1_FB29_Pos) ## !< 0x20000000
  CAN_F7R1_FB29* = CAN_F7R1_FB29_Msk
  CAN_F7R1_FB30_Pos* = (30)
  CAN_F7R1_FB30_Msk* = (0x00000001 shl CAN_F7R1_FB30_Pos) ## !< 0x40000000
  CAN_F7R1_FB30* = CAN_F7R1_FB30_Msk
  CAN_F7R1_FB31_Pos* = (31)
  CAN_F7R1_FB31_Msk* = (0x00000001 shl CAN_F7R1_FB31_Pos) ## !< 0x80000000
  CAN_F7R1_FB31* = CAN_F7R1_FB31_Msk

## ******************  Bit definition for CAN_F8R1 register  ******************

const
  CAN_F8R1_FB0_Pos* = (0)
  CAN_F8R1_FB0_Msk* = (0x00000001 shl CAN_F8R1_FB0_Pos) ## !< 0x00000001
  CAN_F8R1_FB0* = CAN_F8R1_FB0_Msk
  CAN_F8R1_FB1_Pos* = (1)
  CAN_F8R1_FB1_Msk* = (0x00000001 shl CAN_F8R1_FB1_Pos) ## !< 0x00000002
  CAN_F8R1_FB1* = CAN_F8R1_FB1_Msk
  CAN_F8R1_FB2_Pos* = (2)
  CAN_F8R1_FB2_Msk* = (0x00000001 shl CAN_F8R1_FB2_Pos) ## !< 0x00000004
  CAN_F8R1_FB2* = CAN_F8R1_FB2_Msk
  CAN_F8R1_FB3_Pos* = (3)
  CAN_F8R1_FB3_Msk* = (0x00000001 shl CAN_F8R1_FB3_Pos) ## !< 0x00000008
  CAN_F8R1_FB3* = CAN_F8R1_FB3_Msk
  CAN_F8R1_FB4_Pos* = (4)
  CAN_F8R1_FB4_Msk* = (0x00000001 shl CAN_F8R1_FB4_Pos) ## !< 0x00000010
  CAN_F8R1_FB4* = CAN_F8R1_FB4_Msk
  CAN_F8R1_FB5_Pos* = (5)
  CAN_F8R1_FB5_Msk* = (0x00000001 shl CAN_F8R1_FB5_Pos) ## !< 0x00000020
  CAN_F8R1_FB5* = CAN_F8R1_FB5_Msk
  CAN_F8R1_FB6_Pos* = (6)
  CAN_F8R1_FB6_Msk* = (0x00000001 shl CAN_F8R1_FB6_Pos) ## !< 0x00000040
  CAN_F8R1_FB6* = CAN_F8R1_FB6_Msk
  CAN_F8R1_FB7_Pos* = (7)
  CAN_F8R1_FB7_Msk* = (0x00000001 shl CAN_F8R1_FB7_Pos) ## !< 0x00000080
  CAN_F8R1_FB7* = CAN_F8R1_FB7_Msk
  CAN_F8R1_FB8_Pos* = (8)
  CAN_F8R1_FB8_Msk* = (0x00000001 shl CAN_F8R1_FB8_Pos) ## !< 0x00000100
  CAN_F8R1_FB8* = CAN_F8R1_FB8_Msk
  CAN_F8R1_FB9_Pos* = (9)
  CAN_F8R1_FB9_Msk* = (0x00000001 shl CAN_F8R1_FB9_Pos) ## !< 0x00000200
  CAN_F8R1_FB9* = CAN_F8R1_FB9_Msk
  CAN_F8R1_FB10_Pos* = (10)
  CAN_F8R1_FB10_Msk* = (0x00000001 shl CAN_F8R1_FB10_Pos) ## !< 0x00000400
  CAN_F8R1_FB10* = CAN_F8R1_FB10_Msk
  CAN_F8R1_FB11_Pos* = (11)
  CAN_F8R1_FB11_Msk* = (0x00000001 shl CAN_F8R1_FB11_Pos) ## !< 0x00000800
  CAN_F8R1_FB11* = CAN_F8R1_FB11_Msk
  CAN_F8R1_FB12_Pos* = (12)
  CAN_F8R1_FB12_Msk* = (0x00000001 shl CAN_F8R1_FB12_Pos) ## !< 0x00001000
  CAN_F8R1_FB12* = CAN_F8R1_FB12_Msk
  CAN_F8R1_FB13_Pos* = (13)
  CAN_F8R1_FB13_Msk* = (0x00000001 shl CAN_F8R1_FB13_Pos) ## !< 0x00002000
  CAN_F8R1_FB13* = CAN_F8R1_FB13_Msk
  CAN_F8R1_FB14_Pos* = (14)
  CAN_F8R1_FB14_Msk* = (0x00000001 shl CAN_F8R1_FB14_Pos) ## !< 0x00004000
  CAN_F8R1_FB14* = CAN_F8R1_FB14_Msk
  CAN_F8R1_FB15_Pos* = (15)
  CAN_F8R1_FB15_Msk* = (0x00000001 shl CAN_F8R1_FB15_Pos) ## !< 0x00008000
  CAN_F8R1_FB15* = CAN_F8R1_FB15_Msk
  CAN_F8R1_FB16_Pos* = (16)
  CAN_F8R1_FB16_Msk* = (0x00000001 shl CAN_F8R1_FB16_Pos) ## !< 0x00010000
  CAN_F8R1_FB16* = CAN_F8R1_FB16_Msk
  CAN_F8R1_FB17_Pos* = (17)
  CAN_F8R1_FB17_Msk* = (0x00000001 shl CAN_F8R1_FB17_Pos) ## !< 0x00020000
  CAN_F8R1_FB17* = CAN_F8R1_FB17_Msk
  CAN_F8R1_FB18_Pos* = (18)
  CAN_F8R1_FB18_Msk* = (0x00000001 shl CAN_F8R1_FB18_Pos) ## !< 0x00040000
  CAN_F8R1_FB18* = CAN_F8R1_FB18_Msk
  CAN_F8R1_FB19_Pos* = (19)
  CAN_F8R1_FB19_Msk* = (0x00000001 shl CAN_F8R1_FB19_Pos) ## !< 0x00080000
  CAN_F8R1_FB19* = CAN_F8R1_FB19_Msk
  CAN_F8R1_FB20_Pos* = (20)
  CAN_F8R1_FB20_Msk* = (0x00000001 shl CAN_F8R1_FB20_Pos) ## !< 0x00100000
  CAN_F8R1_FB20* = CAN_F8R1_FB20_Msk
  CAN_F8R1_FB21_Pos* = (21)
  CAN_F8R1_FB21_Msk* = (0x00000001 shl CAN_F8R1_FB21_Pos) ## !< 0x00200000
  CAN_F8R1_FB21* = CAN_F8R1_FB21_Msk
  CAN_F8R1_FB22_Pos* = (22)
  CAN_F8R1_FB22_Msk* = (0x00000001 shl CAN_F8R1_FB22_Pos) ## !< 0x00400000
  CAN_F8R1_FB22* = CAN_F8R1_FB22_Msk
  CAN_F8R1_FB23_Pos* = (23)
  CAN_F8R1_FB23_Msk* = (0x00000001 shl CAN_F8R1_FB23_Pos) ## !< 0x00800000
  CAN_F8R1_FB23* = CAN_F8R1_FB23_Msk
  CAN_F8R1_FB24_Pos* = (24)
  CAN_F8R1_FB24_Msk* = (0x00000001 shl CAN_F8R1_FB24_Pos) ## !< 0x01000000
  CAN_F8R1_FB24* = CAN_F8R1_FB24_Msk
  CAN_F8R1_FB25_Pos* = (25)
  CAN_F8R1_FB25_Msk* = (0x00000001 shl CAN_F8R1_FB25_Pos) ## !< 0x02000000
  CAN_F8R1_FB25* = CAN_F8R1_FB25_Msk
  CAN_F8R1_FB26_Pos* = (26)
  CAN_F8R1_FB26_Msk* = (0x00000001 shl CAN_F8R1_FB26_Pos) ## !< 0x04000000
  CAN_F8R1_FB26* = CAN_F8R1_FB26_Msk
  CAN_F8R1_FB27_Pos* = (27)
  CAN_F8R1_FB27_Msk* = (0x00000001 shl CAN_F8R1_FB27_Pos) ## !< 0x08000000
  CAN_F8R1_FB27* = CAN_F8R1_FB27_Msk
  CAN_F8R1_FB28_Pos* = (28)
  CAN_F8R1_FB28_Msk* = (0x00000001 shl CAN_F8R1_FB28_Pos) ## !< 0x10000000
  CAN_F8R1_FB28* = CAN_F8R1_FB28_Msk
  CAN_F8R1_FB29_Pos* = (29)
  CAN_F8R1_FB29_Msk* = (0x00000001 shl CAN_F8R1_FB29_Pos) ## !< 0x20000000
  CAN_F8R1_FB29* = CAN_F8R1_FB29_Msk
  CAN_F8R1_FB30_Pos* = (30)
  CAN_F8R1_FB30_Msk* = (0x00000001 shl CAN_F8R1_FB30_Pos) ## !< 0x40000000
  CAN_F8R1_FB30* = CAN_F8R1_FB30_Msk
  CAN_F8R1_FB31_Pos* = (31)
  CAN_F8R1_FB31_Msk* = (0x00000001 shl CAN_F8R1_FB31_Pos) ## !< 0x80000000
  CAN_F8R1_FB31* = CAN_F8R1_FB31_Msk

## ******************  Bit definition for CAN_F9R1 register  ******************

const
  CAN_F9R1_FB0_Pos* = (0)
  CAN_F9R1_FB0_Msk* = (0x00000001 shl CAN_F9R1_FB0_Pos) ## !< 0x00000001
  CAN_F9R1_FB0* = CAN_F9R1_FB0_Msk
  CAN_F9R1_FB1_Pos* = (1)
  CAN_F9R1_FB1_Msk* = (0x00000001 shl CAN_F9R1_FB1_Pos) ## !< 0x00000002
  CAN_F9R1_FB1* = CAN_F9R1_FB1_Msk
  CAN_F9R1_FB2_Pos* = (2)
  CAN_F9R1_FB2_Msk* = (0x00000001 shl CAN_F9R1_FB2_Pos) ## !< 0x00000004
  CAN_F9R1_FB2* = CAN_F9R1_FB2_Msk
  CAN_F9R1_FB3_Pos* = (3)
  CAN_F9R1_FB3_Msk* = (0x00000001 shl CAN_F9R1_FB3_Pos) ## !< 0x00000008
  CAN_F9R1_FB3* = CAN_F9R1_FB3_Msk
  CAN_F9R1_FB4_Pos* = (4)
  CAN_F9R1_FB4_Msk* = (0x00000001 shl CAN_F9R1_FB4_Pos) ## !< 0x00000010
  CAN_F9R1_FB4* = CAN_F9R1_FB4_Msk
  CAN_F9R1_FB5_Pos* = (5)
  CAN_F9R1_FB5_Msk* = (0x00000001 shl CAN_F9R1_FB5_Pos) ## !< 0x00000020
  CAN_F9R1_FB5* = CAN_F9R1_FB5_Msk
  CAN_F9R1_FB6_Pos* = (6)
  CAN_F9R1_FB6_Msk* = (0x00000001 shl CAN_F9R1_FB6_Pos) ## !< 0x00000040
  CAN_F9R1_FB6* = CAN_F9R1_FB6_Msk
  CAN_F9R1_FB7_Pos* = (7)
  CAN_F9R1_FB7_Msk* = (0x00000001 shl CAN_F9R1_FB7_Pos) ## !< 0x00000080
  CAN_F9R1_FB7* = CAN_F9R1_FB7_Msk
  CAN_F9R1_FB8_Pos* = (8)
  CAN_F9R1_FB8_Msk* = (0x00000001 shl CAN_F9R1_FB8_Pos) ## !< 0x00000100
  CAN_F9R1_FB8* = CAN_F9R1_FB8_Msk
  CAN_F9R1_FB9_Pos* = (9)
  CAN_F9R1_FB9_Msk* = (0x00000001 shl CAN_F9R1_FB9_Pos) ## !< 0x00000200
  CAN_F9R1_FB9* = CAN_F9R1_FB9_Msk
  CAN_F9R1_FB10_Pos* = (10)
  CAN_F9R1_FB10_Msk* = (0x00000001 shl CAN_F9R1_FB10_Pos) ## !< 0x00000400
  CAN_F9R1_FB10* = CAN_F9R1_FB10_Msk
  CAN_F9R1_FB11_Pos* = (11)
  CAN_F9R1_FB11_Msk* = (0x00000001 shl CAN_F9R1_FB11_Pos) ## !< 0x00000800
  CAN_F9R1_FB11* = CAN_F9R1_FB11_Msk
  CAN_F9R1_FB12_Pos* = (12)
  CAN_F9R1_FB12_Msk* = (0x00000001 shl CAN_F9R1_FB12_Pos) ## !< 0x00001000
  CAN_F9R1_FB12* = CAN_F9R1_FB12_Msk
  CAN_F9R1_FB13_Pos* = (13)
  CAN_F9R1_FB13_Msk* = (0x00000001 shl CAN_F9R1_FB13_Pos) ## !< 0x00002000
  CAN_F9R1_FB13* = CAN_F9R1_FB13_Msk
  CAN_F9R1_FB14_Pos* = (14)
  CAN_F9R1_FB14_Msk* = (0x00000001 shl CAN_F9R1_FB14_Pos) ## !< 0x00004000
  CAN_F9R1_FB14* = CAN_F9R1_FB14_Msk
  CAN_F9R1_FB15_Pos* = (15)
  CAN_F9R1_FB15_Msk* = (0x00000001 shl CAN_F9R1_FB15_Pos) ## !< 0x00008000
  CAN_F9R1_FB15* = CAN_F9R1_FB15_Msk
  CAN_F9R1_FB16_Pos* = (16)
  CAN_F9R1_FB16_Msk* = (0x00000001 shl CAN_F9R1_FB16_Pos) ## !< 0x00010000
  CAN_F9R1_FB16* = CAN_F9R1_FB16_Msk
  CAN_F9R1_FB17_Pos* = (17)
  CAN_F9R1_FB17_Msk* = (0x00000001 shl CAN_F9R1_FB17_Pos) ## !< 0x00020000
  CAN_F9R1_FB17* = CAN_F9R1_FB17_Msk
  CAN_F9R1_FB18_Pos* = (18)
  CAN_F9R1_FB18_Msk* = (0x00000001 shl CAN_F9R1_FB18_Pos) ## !< 0x00040000
  CAN_F9R1_FB18* = CAN_F9R1_FB18_Msk
  CAN_F9R1_FB19_Pos* = (19)
  CAN_F9R1_FB19_Msk* = (0x00000001 shl CAN_F9R1_FB19_Pos) ## !< 0x00080000
  CAN_F9R1_FB19* = CAN_F9R1_FB19_Msk
  CAN_F9R1_FB20_Pos* = (20)
  CAN_F9R1_FB20_Msk* = (0x00000001 shl CAN_F9R1_FB20_Pos) ## !< 0x00100000
  CAN_F9R1_FB20* = CAN_F9R1_FB20_Msk
  CAN_F9R1_FB21_Pos* = (21)
  CAN_F9R1_FB21_Msk* = (0x00000001 shl CAN_F9R1_FB21_Pos) ## !< 0x00200000
  CAN_F9R1_FB21* = CAN_F9R1_FB21_Msk
  CAN_F9R1_FB22_Pos* = (22)
  CAN_F9R1_FB22_Msk* = (0x00000001 shl CAN_F9R1_FB22_Pos) ## !< 0x00400000
  CAN_F9R1_FB22* = CAN_F9R1_FB22_Msk
  CAN_F9R1_FB23_Pos* = (23)
  CAN_F9R1_FB23_Msk* = (0x00000001 shl CAN_F9R1_FB23_Pos) ## !< 0x00800000
  CAN_F9R1_FB23* = CAN_F9R1_FB23_Msk
  CAN_F9R1_FB24_Pos* = (24)
  CAN_F9R1_FB24_Msk* = (0x00000001 shl CAN_F9R1_FB24_Pos) ## !< 0x01000000
  CAN_F9R1_FB24* = CAN_F9R1_FB24_Msk
  CAN_F9R1_FB25_Pos* = (25)
  CAN_F9R1_FB25_Msk* = (0x00000001 shl CAN_F9R1_FB25_Pos) ## !< 0x02000000
  CAN_F9R1_FB25* = CAN_F9R1_FB25_Msk
  CAN_F9R1_FB26_Pos* = (26)
  CAN_F9R1_FB26_Msk* = (0x00000001 shl CAN_F9R1_FB26_Pos) ## !< 0x04000000
  CAN_F9R1_FB26* = CAN_F9R1_FB26_Msk
  CAN_F9R1_FB27_Pos* = (27)
  CAN_F9R1_FB27_Msk* = (0x00000001 shl CAN_F9R1_FB27_Pos) ## !< 0x08000000
  CAN_F9R1_FB27* = CAN_F9R1_FB27_Msk
  CAN_F9R1_FB28_Pos* = (28)
  CAN_F9R1_FB28_Msk* = (0x00000001 shl CAN_F9R1_FB28_Pos) ## !< 0x10000000
  CAN_F9R1_FB28* = CAN_F9R1_FB28_Msk
  CAN_F9R1_FB29_Pos* = (29)
  CAN_F9R1_FB29_Msk* = (0x00000001 shl CAN_F9R1_FB29_Pos) ## !< 0x20000000
  CAN_F9R1_FB29* = CAN_F9R1_FB29_Msk
  CAN_F9R1_FB30_Pos* = (30)
  CAN_F9R1_FB30_Msk* = (0x00000001 shl CAN_F9R1_FB30_Pos) ## !< 0x40000000
  CAN_F9R1_FB30* = CAN_F9R1_FB30_Msk
  CAN_F9R1_FB31_Pos* = (31)
  CAN_F9R1_FB31_Msk* = (0x00000001 shl CAN_F9R1_FB31_Pos) ## !< 0x80000000
  CAN_F9R1_FB31* = CAN_F9R1_FB31_Msk

## ******************  Bit definition for CAN_F10R1 register  *****************

const
  CAN_F10R1_FB0_Pos* = (0)
  CAN_F10R1_FB0_Msk* = (0x00000001 shl CAN_F10R1_FB0_Pos) ## !< 0x00000001
  CAN_F10R1_FB0* = CAN_F10R1_FB0_Msk
  CAN_F10R1_FB1_Pos* = (1)
  CAN_F10R1_FB1_Msk* = (0x00000001 shl CAN_F10R1_FB1_Pos) ## !< 0x00000002
  CAN_F10R1_FB1* = CAN_F10R1_FB1_Msk
  CAN_F10R1_FB2_Pos* = (2)
  CAN_F10R1_FB2_Msk* = (0x00000001 shl CAN_F10R1_FB2_Pos) ## !< 0x00000004
  CAN_F10R1_FB2* = CAN_F10R1_FB2_Msk
  CAN_F10R1_FB3_Pos* = (3)
  CAN_F10R1_FB3_Msk* = (0x00000001 shl CAN_F10R1_FB3_Pos) ## !< 0x00000008
  CAN_F10R1_FB3* = CAN_F10R1_FB3_Msk
  CAN_F10R1_FB4_Pos* = (4)
  CAN_F10R1_FB4_Msk* = (0x00000001 shl CAN_F10R1_FB4_Pos) ## !< 0x00000010
  CAN_F10R1_FB4* = CAN_F10R1_FB4_Msk
  CAN_F10R1_FB5_Pos* = (5)
  CAN_F10R1_FB5_Msk* = (0x00000001 shl CAN_F10R1_FB5_Pos) ## !< 0x00000020
  CAN_F10R1_FB5* = CAN_F10R1_FB5_Msk
  CAN_F10R1_FB6_Pos* = (6)
  CAN_F10R1_FB6_Msk* = (0x00000001 shl CAN_F10R1_FB6_Pos) ## !< 0x00000040
  CAN_F10R1_FB6* = CAN_F10R1_FB6_Msk
  CAN_F10R1_FB7_Pos* = (7)
  CAN_F10R1_FB7_Msk* = (0x00000001 shl CAN_F10R1_FB7_Pos) ## !< 0x00000080
  CAN_F10R1_FB7* = CAN_F10R1_FB7_Msk
  CAN_F10R1_FB8_Pos* = (8)
  CAN_F10R1_FB8_Msk* = (0x00000001 shl CAN_F10R1_FB8_Pos) ## !< 0x00000100
  CAN_F10R1_FB8* = CAN_F10R1_FB8_Msk
  CAN_F10R1_FB9_Pos* = (9)
  CAN_F10R1_FB9_Msk* = (0x00000001 shl CAN_F10R1_FB9_Pos) ## !< 0x00000200
  CAN_F10R1_FB9* = CAN_F10R1_FB9_Msk
  CAN_F10R1_FB10_Pos* = (10)
  CAN_F10R1_FB10_Msk* = (0x00000001 shl CAN_F10R1_FB10_Pos) ## !< 0x00000400
  CAN_F10R1_FB10* = CAN_F10R1_FB10_Msk
  CAN_F10R1_FB11_Pos* = (11)
  CAN_F10R1_FB11_Msk* = (0x00000001 shl CAN_F10R1_FB11_Pos) ## !< 0x00000800
  CAN_F10R1_FB11* = CAN_F10R1_FB11_Msk
  CAN_F10R1_FB12_Pos* = (12)
  CAN_F10R1_FB12_Msk* = (0x00000001 shl CAN_F10R1_FB12_Pos) ## !< 0x00001000
  CAN_F10R1_FB12* = CAN_F10R1_FB12_Msk
  CAN_F10R1_FB13_Pos* = (13)
  CAN_F10R1_FB13_Msk* = (0x00000001 shl CAN_F10R1_FB13_Pos) ## !< 0x00002000
  CAN_F10R1_FB13* = CAN_F10R1_FB13_Msk
  CAN_F10R1_FB14_Pos* = (14)
  CAN_F10R1_FB14_Msk* = (0x00000001 shl CAN_F10R1_FB14_Pos) ## !< 0x00004000
  CAN_F10R1_FB14* = CAN_F10R1_FB14_Msk
  CAN_F10R1_FB15_Pos* = (15)
  CAN_F10R1_FB15_Msk* = (0x00000001 shl CAN_F10R1_FB15_Pos) ## !< 0x00008000
  CAN_F10R1_FB15* = CAN_F10R1_FB15_Msk
  CAN_F10R1_FB16_Pos* = (16)
  CAN_F10R1_FB16_Msk* = (0x00000001 shl CAN_F10R1_FB16_Pos) ## !< 0x00010000
  CAN_F10R1_FB16* = CAN_F10R1_FB16_Msk
  CAN_F10R1_FB17_Pos* = (17)
  CAN_F10R1_FB17_Msk* = (0x00000001 shl CAN_F10R1_FB17_Pos) ## !< 0x00020000
  CAN_F10R1_FB17* = CAN_F10R1_FB17_Msk
  CAN_F10R1_FB18_Pos* = (18)
  CAN_F10R1_FB18_Msk* = (0x00000001 shl CAN_F10R1_FB18_Pos) ## !< 0x00040000
  CAN_F10R1_FB18* = CAN_F10R1_FB18_Msk
  CAN_F10R1_FB19_Pos* = (19)
  CAN_F10R1_FB19_Msk* = (0x00000001 shl CAN_F10R1_FB19_Pos) ## !< 0x00080000
  CAN_F10R1_FB19* = CAN_F10R1_FB19_Msk
  CAN_F10R1_FB20_Pos* = (20)
  CAN_F10R1_FB20_Msk* = (0x00000001 shl CAN_F10R1_FB20_Pos) ## !< 0x00100000
  CAN_F10R1_FB20* = CAN_F10R1_FB20_Msk
  CAN_F10R1_FB21_Pos* = (21)
  CAN_F10R1_FB21_Msk* = (0x00000001 shl CAN_F10R1_FB21_Pos) ## !< 0x00200000
  CAN_F10R1_FB21* = CAN_F10R1_FB21_Msk
  CAN_F10R1_FB22_Pos* = (22)
  CAN_F10R1_FB22_Msk* = (0x00000001 shl CAN_F10R1_FB22_Pos) ## !< 0x00400000
  CAN_F10R1_FB22* = CAN_F10R1_FB22_Msk
  CAN_F10R1_FB23_Pos* = (23)
  CAN_F10R1_FB23_Msk* = (0x00000001 shl CAN_F10R1_FB23_Pos) ## !< 0x00800000
  CAN_F10R1_FB23* = CAN_F10R1_FB23_Msk
  CAN_F10R1_FB24_Pos* = (24)
  CAN_F10R1_FB24_Msk* = (0x00000001 shl CAN_F10R1_FB24_Pos) ## !< 0x01000000
  CAN_F10R1_FB24* = CAN_F10R1_FB24_Msk
  CAN_F10R1_FB25_Pos* = (25)
  CAN_F10R1_FB25_Msk* = (0x00000001 shl CAN_F10R1_FB25_Pos) ## !< 0x02000000
  CAN_F10R1_FB25* = CAN_F10R1_FB25_Msk
  CAN_F10R1_FB26_Pos* = (26)
  CAN_F10R1_FB26_Msk* = (0x00000001 shl CAN_F10R1_FB26_Pos) ## !< 0x04000000
  CAN_F10R1_FB26* = CAN_F10R1_FB26_Msk
  CAN_F10R1_FB27_Pos* = (27)
  CAN_F10R1_FB27_Msk* = (0x00000001 shl CAN_F10R1_FB27_Pos) ## !< 0x08000000
  CAN_F10R1_FB27* = CAN_F10R1_FB27_Msk
  CAN_F10R1_FB28_Pos* = (28)
  CAN_F10R1_FB28_Msk* = (0x00000001 shl CAN_F10R1_FB28_Pos) ## !< 0x10000000
  CAN_F10R1_FB28* = CAN_F10R1_FB28_Msk
  CAN_F10R1_FB29_Pos* = (29)
  CAN_F10R1_FB29_Msk* = (0x00000001 shl CAN_F10R1_FB29_Pos) ## !< 0x20000000
  CAN_F10R1_FB29* = CAN_F10R1_FB29_Msk
  CAN_F10R1_FB30_Pos* = (30)
  CAN_F10R1_FB30_Msk* = (0x00000001 shl CAN_F10R1_FB30_Pos) ## !< 0x40000000
  CAN_F10R1_FB30* = CAN_F10R1_FB30_Msk
  CAN_F10R1_FB31_Pos* = (31)
  CAN_F10R1_FB31_Msk* = (0x00000001 shl CAN_F10R1_FB31_Pos) ## !< 0x80000000
  CAN_F10R1_FB31* = CAN_F10R1_FB31_Msk

## ******************  Bit definition for CAN_F11R1 register  *****************

const
  CAN_F11R1_FB0_Pos* = (0)
  CAN_F11R1_FB0_Msk* = (0x00000001 shl CAN_F11R1_FB0_Pos) ## !< 0x00000001
  CAN_F11R1_FB0* = CAN_F11R1_FB0_Msk
  CAN_F11R1_FB1_Pos* = (1)
  CAN_F11R1_FB1_Msk* = (0x00000001 shl CAN_F11R1_FB1_Pos) ## !< 0x00000002
  CAN_F11R1_FB1* = CAN_F11R1_FB1_Msk
  CAN_F11R1_FB2_Pos* = (2)
  CAN_F11R1_FB2_Msk* = (0x00000001 shl CAN_F11R1_FB2_Pos) ## !< 0x00000004
  CAN_F11R1_FB2* = CAN_F11R1_FB2_Msk
  CAN_F11R1_FB3_Pos* = (3)
  CAN_F11R1_FB3_Msk* = (0x00000001 shl CAN_F11R1_FB3_Pos) ## !< 0x00000008
  CAN_F11R1_FB3* = CAN_F11R1_FB3_Msk
  CAN_F11R1_FB4_Pos* = (4)
  CAN_F11R1_FB4_Msk* = (0x00000001 shl CAN_F11R1_FB4_Pos) ## !< 0x00000010
  CAN_F11R1_FB4* = CAN_F11R1_FB4_Msk
  CAN_F11R1_FB5_Pos* = (5)
  CAN_F11R1_FB5_Msk* = (0x00000001 shl CAN_F11R1_FB5_Pos) ## !< 0x00000020
  CAN_F11R1_FB5* = CAN_F11R1_FB5_Msk
  CAN_F11R1_FB6_Pos* = (6)
  CAN_F11R1_FB6_Msk* = (0x00000001 shl CAN_F11R1_FB6_Pos) ## !< 0x00000040
  CAN_F11R1_FB6* = CAN_F11R1_FB6_Msk
  CAN_F11R1_FB7_Pos* = (7)
  CAN_F11R1_FB7_Msk* = (0x00000001 shl CAN_F11R1_FB7_Pos) ## !< 0x00000080
  CAN_F11R1_FB7* = CAN_F11R1_FB7_Msk
  CAN_F11R1_FB8_Pos* = (8)
  CAN_F11R1_FB8_Msk* = (0x00000001 shl CAN_F11R1_FB8_Pos) ## !< 0x00000100
  CAN_F11R1_FB8* = CAN_F11R1_FB8_Msk
  CAN_F11R1_FB9_Pos* = (9)
  CAN_F11R1_FB9_Msk* = (0x00000001 shl CAN_F11R1_FB9_Pos) ## !< 0x00000200
  CAN_F11R1_FB9* = CAN_F11R1_FB9_Msk
  CAN_F11R1_FB10_Pos* = (10)
  CAN_F11R1_FB10_Msk* = (0x00000001 shl CAN_F11R1_FB10_Pos) ## !< 0x00000400
  CAN_F11R1_FB10* = CAN_F11R1_FB10_Msk
  CAN_F11R1_FB11_Pos* = (11)
  CAN_F11R1_FB11_Msk* = (0x00000001 shl CAN_F11R1_FB11_Pos) ## !< 0x00000800
  CAN_F11R1_FB11* = CAN_F11R1_FB11_Msk
  CAN_F11R1_FB12_Pos* = (12)
  CAN_F11R1_FB12_Msk* = (0x00000001 shl CAN_F11R1_FB12_Pos) ## !< 0x00001000
  CAN_F11R1_FB12* = CAN_F11R1_FB12_Msk
  CAN_F11R1_FB13_Pos* = (13)
  CAN_F11R1_FB13_Msk* = (0x00000001 shl CAN_F11R1_FB13_Pos) ## !< 0x00002000
  CAN_F11R1_FB13* = CAN_F11R1_FB13_Msk
  CAN_F11R1_FB14_Pos* = (14)
  CAN_F11R1_FB14_Msk* = (0x00000001 shl CAN_F11R1_FB14_Pos) ## !< 0x00004000
  CAN_F11R1_FB14* = CAN_F11R1_FB14_Msk
  CAN_F11R1_FB15_Pos* = (15)
  CAN_F11R1_FB15_Msk* = (0x00000001 shl CAN_F11R1_FB15_Pos) ## !< 0x00008000
  CAN_F11R1_FB15* = CAN_F11R1_FB15_Msk
  CAN_F11R1_FB16_Pos* = (16)
  CAN_F11R1_FB16_Msk* = (0x00000001 shl CAN_F11R1_FB16_Pos) ## !< 0x00010000
  CAN_F11R1_FB16* = CAN_F11R1_FB16_Msk
  CAN_F11R1_FB17_Pos* = (17)
  CAN_F11R1_FB17_Msk* = (0x00000001 shl CAN_F11R1_FB17_Pos) ## !< 0x00020000
  CAN_F11R1_FB17* = CAN_F11R1_FB17_Msk
  CAN_F11R1_FB18_Pos* = (18)
  CAN_F11R1_FB18_Msk* = (0x00000001 shl CAN_F11R1_FB18_Pos) ## !< 0x00040000
  CAN_F11R1_FB18* = CAN_F11R1_FB18_Msk
  CAN_F11R1_FB19_Pos* = (19)
  CAN_F11R1_FB19_Msk* = (0x00000001 shl CAN_F11R1_FB19_Pos) ## !< 0x00080000
  CAN_F11R1_FB19* = CAN_F11R1_FB19_Msk
  CAN_F11R1_FB20_Pos* = (20)
  CAN_F11R1_FB20_Msk* = (0x00000001 shl CAN_F11R1_FB20_Pos) ## !< 0x00100000
  CAN_F11R1_FB20* = CAN_F11R1_FB20_Msk
  CAN_F11R1_FB21_Pos* = (21)
  CAN_F11R1_FB21_Msk* = (0x00000001 shl CAN_F11R1_FB21_Pos) ## !< 0x00200000
  CAN_F11R1_FB21* = CAN_F11R1_FB21_Msk
  CAN_F11R1_FB22_Pos* = (22)
  CAN_F11R1_FB22_Msk* = (0x00000001 shl CAN_F11R1_FB22_Pos) ## !< 0x00400000
  CAN_F11R1_FB22* = CAN_F11R1_FB22_Msk
  CAN_F11R1_FB23_Pos* = (23)
  CAN_F11R1_FB23_Msk* = (0x00000001 shl CAN_F11R1_FB23_Pos) ## !< 0x00800000
  CAN_F11R1_FB23* = CAN_F11R1_FB23_Msk
  CAN_F11R1_FB24_Pos* = (24)
  CAN_F11R1_FB24_Msk* = (0x00000001 shl CAN_F11R1_FB24_Pos) ## !< 0x01000000
  CAN_F11R1_FB24* = CAN_F11R1_FB24_Msk
  CAN_F11R1_FB25_Pos* = (25)
  CAN_F11R1_FB25_Msk* = (0x00000001 shl CAN_F11R1_FB25_Pos) ## !< 0x02000000
  CAN_F11R1_FB25* = CAN_F11R1_FB25_Msk
  CAN_F11R1_FB26_Pos* = (26)
  CAN_F11R1_FB26_Msk* = (0x00000001 shl CAN_F11R1_FB26_Pos) ## !< 0x04000000
  CAN_F11R1_FB26* = CAN_F11R1_FB26_Msk
  CAN_F11R1_FB27_Pos* = (27)
  CAN_F11R1_FB27_Msk* = (0x00000001 shl CAN_F11R1_FB27_Pos) ## !< 0x08000000
  CAN_F11R1_FB27* = CAN_F11R1_FB27_Msk
  CAN_F11R1_FB28_Pos* = (28)
  CAN_F11R1_FB28_Msk* = (0x00000001 shl CAN_F11R1_FB28_Pos) ## !< 0x10000000
  CAN_F11R1_FB28* = CAN_F11R1_FB28_Msk
  CAN_F11R1_FB29_Pos* = (29)
  CAN_F11R1_FB29_Msk* = (0x00000001 shl CAN_F11R1_FB29_Pos) ## !< 0x20000000
  CAN_F11R1_FB29* = CAN_F11R1_FB29_Msk
  CAN_F11R1_FB30_Pos* = (30)
  CAN_F11R1_FB30_Msk* = (0x00000001 shl CAN_F11R1_FB30_Pos) ## !< 0x40000000
  CAN_F11R1_FB30* = CAN_F11R1_FB30_Msk
  CAN_F11R1_FB31_Pos* = (31)
  CAN_F11R1_FB31_Msk* = (0x00000001 shl CAN_F11R1_FB31_Pos) ## !< 0x80000000
  CAN_F11R1_FB31* = CAN_F11R1_FB31_Msk

## ******************  Bit definition for CAN_F12R1 register  *****************

const
  CAN_F12R1_FB0_Pos* = (0)
  CAN_F12R1_FB0_Msk* = (0x00000001 shl CAN_F12R1_FB0_Pos) ## !< 0x00000001
  CAN_F12R1_FB0* = CAN_F12R1_FB0_Msk
  CAN_F12R1_FB1_Pos* = (1)
  CAN_F12R1_FB1_Msk* = (0x00000001 shl CAN_F12R1_FB1_Pos) ## !< 0x00000002
  CAN_F12R1_FB1* = CAN_F12R1_FB1_Msk
  CAN_F12R1_FB2_Pos* = (2)
  CAN_F12R1_FB2_Msk* = (0x00000001 shl CAN_F12R1_FB2_Pos) ## !< 0x00000004
  CAN_F12R1_FB2* = CAN_F12R1_FB2_Msk
  CAN_F12R1_FB3_Pos* = (3)
  CAN_F12R1_FB3_Msk* = (0x00000001 shl CAN_F12R1_FB3_Pos) ## !< 0x00000008
  CAN_F12R1_FB3* = CAN_F12R1_FB3_Msk
  CAN_F12R1_FB4_Pos* = (4)
  CAN_F12R1_FB4_Msk* = (0x00000001 shl CAN_F12R1_FB4_Pos) ## !< 0x00000010
  CAN_F12R1_FB4* = CAN_F12R1_FB4_Msk
  CAN_F12R1_FB5_Pos* = (5)
  CAN_F12R1_FB5_Msk* = (0x00000001 shl CAN_F12R1_FB5_Pos) ## !< 0x00000020
  CAN_F12R1_FB5* = CAN_F12R1_FB5_Msk
  CAN_F12R1_FB6_Pos* = (6)
  CAN_F12R1_FB6_Msk* = (0x00000001 shl CAN_F12R1_FB6_Pos) ## !< 0x00000040
  CAN_F12R1_FB6* = CAN_F12R1_FB6_Msk
  CAN_F12R1_FB7_Pos* = (7)
  CAN_F12R1_FB7_Msk* = (0x00000001 shl CAN_F12R1_FB7_Pos) ## !< 0x00000080
  CAN_F12R1_FB7* = CAN_F12R1_FB7_Msk
  CAN_F12R1_FB8_Pos* = (8)
  CAN_F12R1_FB8_Msk* = (0x00000001 shl CAN_F12R1_FB8_Pos) ## !< 0x00000100
  CAN_F12R1_FB8* = CAN_F12R1_FB8_Msk
  CAN_F12R1_FB9_Pos* = (9)
  CAN_F12R1_FB9_Msk* = (0x00000001 shl CAN_F12R1_FB9_Pos) ## !< 0x00000200
  CAN_F12R1_FB9* = CAN_F12R1_FB9_Msk
  CAN_F12R1_FB10_Pos* = (10)
  CAN_F12R1_FB10_Msk* = (0x00000001 shl CAN_F12R1_FB10_Pos) ## !< 0x00000400
  CAN_F12R1_FB10* = CAN_F12R1_FB10_Msk
  CAN_F12R1_FB11_Pos* = (11)
  CAN_F12R1_FB11_Msk* = (0x00000001 shl CAN_F12R1_FB11_Pos) ## !< 0x00000800
  CAN_F12R1_FB11* = CAN_F12R1_FB11_Msk
  CAN_F12R1_FB12_Pos* = (12)
  CAN_F12R1_FB12_Msk* = (0x00000001 shl CAN_F12R1_FB12_Pos) ## !< 0x00001000
  CAN_F12R1_FB12* = CAN_F12R1_FB12_Msk
  CAN_F12R1_FB13_Pos* = (13)
  CAN_F12R1_FB13_Msk* = (0x00000001 shl CAN_F12R1_FB13_Pos) ## !< 0x00002000
  CAN_F12R1_FB13* = CAN_F12R1_FB13_Msk
  CAN_F12R1_FB14_Pos* = (14)
  CAN_F12R1_FB14_Msk* = (0x00000001 shl CAN_F12R1_FB14_Pos) ## !< 0x00004000
  CAN_F12R1_FB14* = CAN_F12R1_FB14_Msk
  CAN_F12R1_FB15_Pos* = (15)
  CAN_F12R1_FB15_Msk* = (0x00000001 shl CAN_F12R1_FB15_Pos) ## !< 0x00008000
  CAN_F12R1_FB15* = CAN_F12R1_FB15_Msk
  CAN_F12R1_FB16_Pos* = (16)
  CAN_F12R1_FB16_Msk* = (0x00000001 shl CAN_F12R1_FB16_Pos) ## !< 0x00010000
  CAN_F12R1_FB16* = CAN_F12R1_FB16_Msk
  CAN_F12R1_FB17_Pos* = (17)
  CAN_F12R1_FB17_Msk* = (0x00000001 shl CAN_F12R1_FB17_Pos) ## !< 0x00020000
  CAN_F12R1_FB17* = CAN_F12R1_FB17_Msk
  CAN_F12R1_FB18_Pos* = (18)
  CAN_F12R1_FB18_Msk* = (0x00000001 shl CAN_F12R1_FB18_Pos) ## !< 0x00040000
  CAN_F12R1_FB18* = CAN_F12R1_FB18_Msk
  CAN_F12R1_FB19_Pos* = (19)
  CAN_F12R1_FB19_Msk* = (0x00000001 shl CAN_F12R1_FB19_Pos) ## !< 0x00080000
  CAN_F12R1_FB19* = CAN_F12R1_FB19_Msk
  CAN_F12R1_FB20_Pos* = (20)
  CAN_F12R1_FB20_Msk* = (0x00000001 shl CAN_F12R1_FB20_Pos) ## !< 0x00100000
  CAN_F12R1_FB20* = CAN_F12R1_FB20_Msk
  CAN_F12R1_FB21_Pos* = (21)
  CAN_F12R1_FB21_Msk* = (0x00000001 shl CAN_F12R1_FB21_Pos) ## !< 0x00200000
  CAN_F12R1_FB21* = CAN_F12R1_FB21_Msk
  CAN_F12R1_FB22_Pos* = (22)
  CAN_F12R1_FB22_Msk* = (0x00000001 shl CAN_F12R1_FB22_Pos) ## !< 0x00400000
  CAN_F12R1_FB22* = CAN_F12R1_FB22_Msk
  CAN_F12R1_FB23_Pos* = (23)
  CAN_F12R1_FB23_Msk* = (0x00000001 shl CAN_F12R1_FB23_Pos) ## !< 0x00800000
  CAN_F12R1_FB23* = CAN_F12R1_FB23_Msk
  CAN_F12R1_FB24_Pos* = (24)
  CAN_F12R1_FB24_Msk* = (0x00000001 shl CAN_F12R1_FB24_Pos) ## !< 0x01000000
  CAN_F12R1_FB24* = CAN_F12R1_FB24_Msk
  CAN_F12R1_FB25_Pos* = (25)
  CAN_F12R1_FB25_Msk* = (0x00000001 shl CAN_F12R1_FB25_Pos) ## !< 0x02000000
  CAN_F12R1_FB25* = CAN_F12R1_FB25_Msk
  CAN_F12R1_FB26_Pos* = (26)
  CAN_F12R1_FB26_Msk* = (0x00000001 shl CAN_F12R1_FB26_Pos) ## !< 0x04000000
  CAN_F12R1_FB26* = CAN_F12R1_FB26_Msk
  CAN_F12R1_FB27_Pos* = (27)
  CAN_F12R1_FB27_Msk* = (0x00000001 shl CAN_F12R1_FB27_Pos) ## !< 0x08000000
  CAN_F12R1_FB27* = CAN_F12R1_FB27_Msk
  CAN_F12R1_FB28_Pos* = (28)
  CAN_F12R1_FB28_Msk* = (0x00000001 shl CAN_F12R1_FB28_Pos) ## !< 0x10000000
  CAN_F12R1_FB28* = CAN_F12R1_FB28_Msk
  CAN_F12R1_FB29_Pos* = (29)
  CAN_F12R1_FB29_Msk* = (0x00000001 shl CAN_F12R1_FB29_Pos) ## !< 0x20000000
  CAN_F12R1_FB29* = CAN_F12R1_FB29_Msk
  CAN_F12R1_FB30_Pos* = (30)
  CAN_F12R1_FB30_Msk* = (0x00000001 shl CAN_F12R1_FB30_Pos) ## !< 0x40000000
  CAN_F12R1_FB30* = CAN_F12R1_FB30_Msk
  CAN_F12R1_FB31_Pos* = (31)
  CAN_F12R1_FB31_Msk* = (0x00000001 shl CAN_F12R1_FB31_Pos) ## !< 0x80000000
  CAN_F12R1_FB31* = CAN_F12R1_FB31_Msk

## ******************  Bit definition for CAN_F13R1 register  *****************

const
  CAN_F13R1_FB0_Pos* = (0)
  CAN_F13R1_FB0_Msk* = (0x00000001 shl CAN_F13R1_FB0_Pos) ## !< 0x00000001
  CAN_F13R1_FB0* = CAN_F13R1_FB0_Msk
  CAN_F13R1_FB1_Pos* = (1)
  CAN_F13R1_FB1_Msk* = (0x00000001 shl CAN_F13R1_FB1_Pos) ## !< 0x00000002
  CAN_F13R1_FB1* = CAN_F13R1_FB1_Msk
  CAN_F13R1_FB2_Pos* = (2)
  CAN_F13R1_FB2_Msk* = (0x00000001 shl CAN_F13R1_FB2_Pos) ## !< 0x00000004
  CAN_F13R1_FB2* = CAN_F13R1_FB2_Msk
  CAN_F13R1_FB3_Pos* = (3)
  CAN_F13R1_FB3_Msk* = (0x00000001 shl CAN_F13R1_FB3_Pos) ## !< 0x00000008
  CAN_F13R1_FB3* = CAN_F13R1_FB3_Msk
  CAN_F13R1_FB4_Pos* = (4)
  CAN_F13R1_FB4_Msk* = (0x00000001 shl CAN_F13R1_FB4_Pos) ## !< 0x00000010
  CAN_F13R1_FB4* = CAN_F13R1_FB4_Msk
  CAN_F13R1_FB5_Pos* = (5)
  CAN_F13R1_FB5_Msk* = (0x00000001 shl CAN_F13R1_FB5_Pos) ## !< 0x00000020
  CAN_F13R1_FB5* = CAN_F13R1_FB5_Msk
  CAN_F13R1_FB6_Pos* = (6)
  CAN_F13R1_FB6_Msk* = (0x00000001 shl CAN_F13R1_FB6_Pos) ## !< 0x00000040
  CAN_F13R1_FB6* = CAN_F13R1_FB6_Msk
  CAN_F13R1_FB7_Pos* = (7)
  CAN_F13R1_FB7_Msk* = (0x00000001 shl CAN_F13R1_FB7_Pos) ## !< 0x00000080
  CAN_F13R1_FB7* = CAN_F13R1_FB7_Msk
  CAN_F13R1_FB8_Pos* = (8)
  CAN_F13R1_FB8_Msk* = (0x00000001 shl CAN_F13R1_FB8_Pos) ## !< 0x00000100
  CAN_F13R1_FB8* = CAN_F13R1_FB8_Msk
  CAN_F13R1_FB9_Pos* = (9)
  CAN_F13R1_FB9_Msk* = (0x00000001 shl CAN_F13R1_FB9_Pos) ## !< 0x00000200
  CAN_F13R1_FB9* = CAN_F13R1_FB9_Msk
  CAN_F13R1_FB10_Pos* = (10)
  CAN_F13R1_FB10_Msk* = (0x00000001 shl CAN_F13R1_FB10_Pos) ## !< 0x00000400
  CAN_F13R1_FB10* = CAN_F13R1_FB10_Msk
  CAN_F13R1_FB11_Pos* = (11)
  CAN_F13R1_FB11_Msk* = (0x00000001 shl CAN_F13R1_FB11_Pos) ## !< 0x00000800
  CAN_F13R1_FB11* = CAN_F13R1_FB11_Msk
  CAN_F13R1_FB12_Pos* = (12)
  CAN_F13R1_FB12_Msk* = (0x00000001 shl CAN_F13R1_FB12_Pos) ## !< 0x00001000
  CAN_F13R1_FB12* = CAN_F13R1_FB12_Msk
  CAN_F13R1_FB13_Pos* = (13)
  CAN_F13R1_FB13_Msk* = (0x00000001 shl CAN_F13R1_FB13_Pos) ## !< 0x00002000
  CAN_F13R1_FB13* = CAN_F13R1_FB13_Msk
  CAN_F13R1_FB14_Pos* = (14)
  CAN_F13R1_FB14_Msk* = (0x00000001 shl CAN_F13R1_FB14_Pos) ## !< 0x00004000
  CAN_F13R1_FB14* = CAN_F13R1_FB14_Msk
  CAN_F13R1_FB15_Pos* = (15)
  CAN_F13R1_FB15_Msk* = (0x00000001 shl CAN_F13R1_FB15_Pos) ## !< 0x00008000
  CAN_F13R1_FB15* = CAN_F13R1_FB15_Msk
  CAN_F13R1_FB16_Pos* = (16)
  CAN_F13R1_FB16_Msk* = (0x00000001 shl CAN_F13R1_FB16_Pos) ## !< 0x00010000
  CAN_F13R1_FB16* = CAN_F13R1_FB16_Msk
  CAN_F13R1_FB17_Pos* = (17)
  CAN_F13R1_FB17_Msk* = (0x00000001 shl CAN_F13R1_FB17_Pos) ## !< 0x00020000
  CAN_F13R1_FB17* = CAN_F13R1_FB17_Msk
  CAN_F13R1_FB18_Pos* = (18)
  CAN_F13R1_FB18_Msk* = (0x00000001 shl CAN_F13R1_FB18_Pos) ## !< 0x00040000
  CAN_F13R1_FB18* = CAN_F13R1_FB18_Msk
  CAN_F13R1_FB19_Pos* = (19)
  CAN_F13R1_FB19_Msk* = (0x00000001 shl CAN_F13R1_FB19_Pos) ## !< 0x00080000
  CAN_F13R1_FB19* = CAN_F13R1_FB19_Msk
  CAN_F13R1_FB20_Pos* = (20)
  CAN_F13R1_FB20_Msk* = (0x00000001 shl CAN_F13R1_FB20_Pos) ## !< 0x00100000
  CAN_F13R1_FB20* = CAN_F13R1_FB20_Msk
  CAN_F13R1_FB21_Pos* = (21)
  CAN_F13R1_FB21_Msk* = (0x00000001 shl CAN_F13R1_FB21_Pos) ## !< 0x00200000
  CAN_F13R1_FB21* = CAN_F13R1_FB21_Msk
  CAN_F13R1_FB22_Pos* = (22)
  CAN_F13R1_FB22_Msk* = (0x00000001 shl CAN_F13R1_FB22_Pos) ## !< 0x00400000
  CAN_F13R1_FB22* = CAN_F13R1_FB22_Msk
  CAN_F13R1_FB23_Pos* = (23)
  CAN_F13R1_FB23_Msk* = (0x00000001 shl CAN_F13R1_FB23_Pos) ## !< 0x00800000
  CAN_F13R1_FB23* = CAN_F13R1_FB23_Msk
  CAN_F13R1_FB24_Pos* = (24)
  CAN_F13R1_FB24_Msk* = (0x00000001 shl CAN_F13R1_FB24_Pos) ## !< 0x01000000
  CAN_F13R1_FB24* = CAN_F13R1_FB24_Msk
  CAN_F13R1_FB25_Pos* = (25)
  CAN_F13R1_FB25_Msk* = (0x00000001 shl CAN_F13R1_FB25_Pos) ## !< 0x02000000
  CAN_F13R1_FB25* = CAN_F13R1_FB25_Msk
  CAN_F13R1_FB26_Pos* = (26)
  CAN_F13R1_FB26_Msk* = (0x00000001 shl CAN_F13R1_FB26_Pos) ## !< 0x04000000
  CAN_F13R1_FB26* = CAN_F13R1_FB26_Msk
  CAN_F13R1_FB27_Pos* = (27)
  CAN_F13R1_FB27_Msk* = (0x00000001 shl CAN_F13R1_FB27_Pos) ## !< 0x08000000
  CAN_F13R1_FB27* = CAN_F13R1_FB27_Msk
  CAN_F13R1_FB28_Pos* = (28)
  CAN_F13R1_FB28_Msk* = (0x00000001 shl CAN_F13R1_FB28_Pos) ## !< 0x10000000
  CAN_F13R1_FB28* = CAN_F13R1_FB28_Msk
  CAN_F13R1_FB29_Pos* = (29)
  CAN_F13R1_FB29_Msk* = (0x00000001 shl CAN_F13R1_FB29_Pos) ## !< 0x20000000
  CAN_F13R1_FB29* = CAN_F13R1_FB29_Msk
  CAN_F13R1_FB30_Pos* = (30)
  CAN_F13R1_FB30_Msk* = (0x00000001 shl CAN_F13R1_FB30_Pos) ## !< 0x40000000
  CAN_F13R1_FB30* = CAN_F13R1_FB30_Msk
  CAN_F13R1_FB31_Pos* = (31)
  CAN_F13R1_FB31_Msk* = (0x00000001 shl CAN_F13R1_FB31_Pos) ## !< 0x80000000
  CAN_F13R1_FB31* = CAN_F13R1_FB31_Msk

## ******************  Bit definition for CAN_F0R2 register  ******************

const
  CAN_F0R2_FB0_Pos* = (0)
  CAN_F0R2_FB0_Msk* = (0x00000001 shl CAN_F0R2_FB0_Pos) ## !< 0x00000001
  CAN_F0R2_FB0* = CAN_F0R2_FB0_Msk
  CAN_F0R2_FB1_Pos* = (1)
  CAN_F0R2_FB1_Msk* = (0x00000001 shl CAN_F0R2_FB1_Pos) ## !< 0x00000002
  CAN_F0R2_FB1* = CAN_F0R2_FB1_Msk
  CAN_F0R2_FB2_Pos* = (2)
  CAN_F0R2_FB2_Msk* = (0x00000001 shl CAN_F0R2_FB2_Pos) ## !< 0x00000004
  CAN_F0R2_FB2* = CAN_F0R2_FB2_Msk
  CAN_F0R2_FB3_Pos* = (3)
  CAN_F0R2_FB3_Msk* = (0x00000001 shl CAN_F0R2_FB3_Pos) ## !< 0x00000008
  CAN_F0R2_FB3* = CAN_F0R2_FB3_Msk
  CAN_F0R2_FB4_Pos* = (4)
  CAN_F0R2_FB4_Msk* = (0x00000001 shl CAN_F0R2_FB4_Pos) ## !< 0x00000010
  CAN_F0R2_FB4* = CAN_F0R2_FB4_Msk
  CAN_F0R2_FB5_Pos* = (5)
  CAN_F0R2_FB5_Msk* = (0x00000001 shl CAN_F0R2_FB5_Pos) ## !< 0x00000020
  CAN_F0R2_FB5* = CAN_F0R2_FB5_Msk
  CAN_F0R2_FB6_Pos* = (6)
  CAN_F0R2_FB6_Msk* = (0x00000001 shl CAN_F0R2_FB6_Pos) ## !< 0x00000040
  CAN_F0R2_FB6* = CAN_F0R2_FB6_Msk
  CAN_F0R2_FB7_Pos* = (7)
  CAN_F0R2_FB7_Msk* = (0x00000001 shl CAN_F0R2_FB7_Pos) ## !< 0x00000080
  CAN_F0R2_FB7* = CAN_F0R2_FB7_Msk
  CAN_F0R2_FB8_Pos* = (8)
  CAN_F0R2_FB8_Msk* = (0x00000001 shl CAN_F0R2_FB8_Pos) ## !< 0x00000100
  CAN_F0R2_FB8* = CAN_F0R2_FB8_Msk
  CAN_F0R2_FB9_Pos* = (9)
  CAN_F0R2_FB9_Msk* = (0x00000001 shl CAN_F0R2_FB9_Pos) ## !< 0x00000200
  CAN_F0R2_FB9* = CAN_F0R2_FB9_Msk
  CAN_F0R2_FB10_Pos* = (10)
  CAN_F0R2_FB10_Msk* = (0x00000001 shl CAN_F0R2_FB10_Pos) ## !< 0x00000400
  CAN_F0R2_FB10* = CAN_F0R2_FB10_Msk
  CAN_F0R2_FB11_Pos* = (11)
  CAN_F0R2_FB11_Msk* = (0x00000001 shl CAN_F0R2_FB11_Pos) ## !< 0x00000800
  CAN_F0R2_FB11* = CAN_F0R2_FB11_Msk
  CAN_F0R2_FB12_Pos* = (12)
  CAN_F0R2_FB12_Msk* = (0x00000001 shl CAN_F0R2_FB12_Pos) ## !< 0x00001000
  CAN_F0R2_FB12* = CAN_F0R2_FB12_Msk
  CAN_F0R2_FB13_Pos* = (13)
  CAN_F0R2_FB13_Msk* = (0x00000001 shl CAN_F0R2_FB13_Pos) ## !< 0x00002000
  CAN_F0R2_FB13* = CAN_F0R2_FB13_Msk
  CAN_F0R2_FB14_Pos* = (14)
  CAN_F0R2_FB14_Msk* = (0x00000001 shl CAN_F0R2_FB14_Pos) ## !< 0x00004000
  CAN_F0R2_FB14* = CAN_F0R2_FB14_Msk
  CAN_F0R2_FB15_Pos* = (15)
  CAN_F0R2_FB15_Msk* = (0x00000001 shl CAN_F0R2_FB15_Pos) ## !< 0x00008000
  CAN_F0R2_FB15* = CAN_F0R2_FB15_Msk
  CAN_F0R2_FB16_Pos* = (16)
  CAN_F0R2_FB16_Msk* = (0x00000001 shl CAN_F0R2_FB16_Pos) ## !< 0x00010000
  CAN_F0R2_FB16* = CAN_F0R2_FB16_Msk
  CAN_F0R2_FB17_Pos* = (17)
  CAN_F0R2_FB17_Msk* = (0x00000001 shl CAN_F0R2_FB17_Pos) ## !< 0x00020000
  CAN_F0R2_FB17* = CAN_F0R2_FB17_Msk
  CAN_F0R2_FB18_Pos* = (18)
  CAN_F0R2_FB18_Msk* = (0x00000001 shl CAN_F0R2_FB18_Pos) ## !< 0x00040000
  CAN_F0R2_FB18* = CAN_F0R2_FB18_Msk
  CAN_F0R2_FB19_Pos* = (19)
  CAN_F0R2_FB19_Msk* = (0x00000001 shl CAN_F0R2_FB19_Pos) ## !< 0x00080000
  CAN_F0R2_FB19* = CAN_F0R2_FB19_Msk
  CAN_F0R2_FB20_Pos* = (20)
  CAN_F0R2_FB20_Msk* = (0x00000001 shl CAN_F0R2_FB20_Pos) ## !< 0x00100000
  CAN_F0R2_FB20* = CAN_F0R2_FB20_Msk
  CAN_F0R2_FB21_Pos* = (21)
  CAN_F0R2_FB21_Msk* = (0x00000001 shl CAN_F0R2_FB21_Pos) ## !< 0x00200000
  CAN_F0R2_FB21* = CAN_F0R2_FB21_Msk
  CAN_F0R2_FB22_Pos* = (22)
  CAN_F0R2_FB22_Msk* = (0x00000001 shl CAN_F0R2_FB22_Pos) ## !< 0x00400000
  CAN_F0R2_FB22* = CAN_F0R2_FB22_Msk
  CAN_F0R2_FB23_Pos* = (23)
  CAN_F0R2_FB23_Msk* = (0x00000001 shl CAN_F0R2_FB23_Pos) ## !< 0x00800000
  CAN_F0R2_FB23* = CAN_F0R2_FB23_Msk
  CAN_F0R2_FB24_Pos* = (24)
  CAN_F0R2_FB24_Msk* = (0x00000001 shl CAN_F0R2_FB24_Pos) ## !< 0x01000000
  CAN_F0R2_FB24* = CAN_F0R2_FB24_Msk
  CAN_F0R2_FB25_Pos* = (25)
  CAN_F0R2_FB25_Msk* = (0x00000001 shl CAN_F0R2_FB25_Pos) ## !< 0x02000000
  CAN_F0R2_FB25* = CAN_F0R2_FB25_Msk
  CAN_F0R2_FB26_Pos* = (26)
  CAN_F0R2_FB26_Msk* = (0x00000001 shl CAN_F0R2_FB26_Pos) ## !< 0x04000000
  CAN_F0R2_FB26* = CAN_F0R2_FB26_Msk
  CAN_F0R2_FB27_Pos* = (27)
  CAN_F0R2_FB27_Msk* = (0x00000001 shl CAN_F0R2_FB27_Pos) ## !< 0x08000000
  CAN_F0R2_FB27* = CAN_F0R2_FB27_Msk
  CAN_F0R2_FB28_Pos* = (28)
  CAN_F0R2_FB28_Msk* = (0x00000001 shl CAN_F0R2_FB28_Pos) ## !< 0x10000000
  CAN_F0R2_FB28* = CAN_F0R2_FB28_Msk
  CAN_F0R2_FB29_Pos* = (29)
  CAN_F0R2_FB29_Msk* = (0x00000001 shl CAN_F0R2_FB29_Pos) ## !< 0x20000000
  CAN_F0R2_FB29* = CAN_F0R2_FB29_Msk
  CAN_F0R2_FB30_Pos* = (30)
  CAN_F0R2_FB30_Msk* = (0x00000001 shl CAN_F0R2_FB30_Pos) ## !< 0x40000000
  CAN_F0R2_FB30* = CAN_F0R2_FB30_Msk
  CAN_F0R2_FB31_Pos* = (31)
  CAN_F0R2_FB31_Msk* = (0x00000001 shl CAN_F0R2_FB31_Pos) ## !< 0x80000000
  CAN_F0R2_FB31* = CAN_F0R2_FB31_Msk

## ******************  Bit definition for CAN_F1R2 register  ******************

const
  CAN_F1R2_FB0_Pos* = (0)
  CAN_F1R2_FB0_Msk* = (0x00000001 shl CAN_F1R2_FB0_Pos) ## !< 0x00000001
  CAN_F1R2_FB0* = CAN_F1R2_FB0_Msk
  CAN_F1R2_FB1_Pos* = (1)
  CAN_F1R2_FB1_Msk* = (0x00000001 shl CAN_F1R2_FB1_Pos) ## !< 0x00000002
  CAN_F1R2_FB1* = CAN_F1R2_FB1_Msk
  CAN_F1R2_FB2_Pos* = (2)
  CAN_F1R2_FB2_Msk* = (0x00000001 shl CAN_F1R2_FB2_Pos) ## !< 0x00000004
  CAN_F1R2_FB2* = CAN_F1R2_FB2_Msk
  CAN_F1R2_FB3_Pos* = (3)
  CAN_F1R2_FB3_Msk* = (0x00000001 shl CAN_F1R2_FB3_Pos) ## !< 0x00000008
  CAN_F1R2_FB3* = CAN_F1R2_FB3_Msk
  CAN_F1R2_FB4_Pos* = (4)
  CAN_F1R2_FB4_Msk* = (0x00000001 shl CAN_F1R2_FB4_Pos) ## !< 0x00000010
  CAN_F1R2_FB4* = CAN_F1R2_FB4_Msk
  CAN_F1R2_FB5_Pos* = (5)
  CAN_F1R2_FB5_Msk* = (0x00000001 shl CAN_F1R2_FB5_Pos) ## !< 0x00000020
  CAN_F1R2_FB5* = CAN_F1R2_FB5_Msk
  CAN_F1R2_FB6_Pos* = (6)
  CAN_F1R2_FB6_Msk* = (0x00000001 shl CAN_F1R2_FB6_Pos) ## !< 0x00000040
  CAN_F1R2_FB6* = CAN_F1R2_FB6_Msk
  CAN_F1R2_FB7_Pos* = (7)
  CAN_F1R2_FB7_Msk* = (0x00000001 shl CAN_F1R2_FB7_Pos) ## !< 0x00000080
  CAN_F1R2_FB7* = CAN_F1R2_FB7_Msk
  CAN_F1R2_FB8_Pos* = (8)
  CAN_F1R2_FB8_Msk* = (0x00000001 shl CAN_F1R2_FB8_Pos) ## !< 0x00000100
  CAN_F1R2_FB8* = CAN_F1R2_FB8_Msk
  CAN_F1R2_FB9_Pos* = (9)
  CAN_F1R2_FB9_Msk* = (0x00000001 shl CAN_F1R2_FB9_Pos) ## !< 0x00000200
  CAN_F1R2_FB9* = CAN_F1R2_FB9_Msk
  CAN_F1R2_FB10_Pos* = (10)
  CAN_F1R2_FB10_Msk* = (0x00000001 shl CAN_F1R2_FB10_Pos) ## !< 0x00000400
  CAN_F1R2_FB10* = CAN_F1R2_FB10_Msk
  CAN_F1R2_FB11_Pos* = (11)
  CAN_F1R2_FB11_Msk* = (0x00000001 shl CAN_F1R2_FB11_Pos) ## !< 0x00000800
  CAN_F1R2_FB11* = CAN_F1R2_FB11_Msk
  CAN_F1R2_FB12_Pos* = (12)
  CAN_F1R2_FB12_Msk* = (0x00000001 shl CAN_F1R2_FB12_Pos) ## !< 0x00001000
  CAN_F1R2_FB12* = CAN_F1R2_FB12_Msk
  CAN_F1R2_FB13_Pos* = (13)
  CAN_F1R2_FB13_Msk* = (0x00000001 shl CAN_F1R2_FB13_Pos) ## !< 0x00002000
  CAN_F1R2_FB13* = CAN_F1R2_FB13_Msk
  CAN_F1R2_FB14_Pos* = (14)
  CAN_F1R2_FB14_Msk* = (0x00000001 shl CAN_F1R2_FB14_Pos) ## !< 0x00004000
  CAN_F1R2_FB14* = CAN_F1R2_FB14_Msk
  CAN_F1R2_FB15_Pos* = (15)
  CAN_F1R2_FB15_Msk* = (0x00000001 shl CAN_F1R2_FB15_Pos) ## !< 0x00008000
  CAN_F1R2_FB15* = CAN_F1R2_FB15_Msk
  CAN_F1R2_FB16_Pos* = (16)
  CAN_F1R2_FB16_Msk* = (0x00000001 shl CAN_F1R2_FB16_Pos) ## !< 0x00010000
  CAN_F1R2_FB16* = CAN_F1R2_FB16_Msk
  CAN_F1R2_FB17_Pos* = (17)
  CAN_F1R2_FB17_Msk* = (0x00000001 shl CAN_F1R2_FB17_Pos) ## !< 0x00020000
  CAN_F1R2_FB17* = CAN_F1R2_FB17_Msk
  CAN_F1R2_FB18_Pos* = (18)
  CAN_F1R2_FB18_Msk* = (0x00000001 shl CAN_F1R2_FB18_Pos) ## !< 0x00040000
  CAN_F1R2_FB18* = CAN_F1R2_FB18_Msk
  CAN_F1R2_FB19_Pos* = (19)
  CAN_F1R2_FB19_Msk* = (0x00000001 shl CAN_F1R2_FB19_Pos) ## !< 0x00080000
  CAN_F1R2_FB19* = CAN_F1R2_FB19_Msk
  CAN_F1R2_FB20_Pos* = (20)
  CAN_F1R2_FB20_Msk* = (0x00000001 shl CAN_F1R2_FB20_Pos) ## !< 0x00100000
  CAN_F1R2_FB20* = CAN_F1R2_FB20_Msk
  CAN_F1R2_FB21_Pos* = (21)
  CAN_F1R2_FB21_Msk* = (0x00000001 shl CAN_F1R2_FB21_Pos) ## !< 0x00200000
  CAN_F1R2_FB21* = CAN_F1R2_FB21_Msk
  CAN_F1R2_FB22_Pos* = (22)
  CAN_F1R2_FB22_Msk* = (0x00000001 shl CAN_F1R2_FB22_Pos) ## !< 0x00400000
  CAN_F1R2_FB22* = CAN_F1R2_FB22_Msk
  CAN_F1R2_FB23_Pos* = (23)
  CAN_F1R2_FB23_Msk* = (0x00000001 shl CAN_F1R2_FB23_Pos) ## !< 0x00800000
  CAN_F1R2_FB23* = CAN_F1R2_FB23_Msk
  CAN_F1R2_FB24_Pos* = (24)
  CAN_F1R2_FB24_Msk* = (0x00000001 shl CAN_F1R2_FB24_Pos) ## !< 0x01000000
  CAN_F1R2_FB24* = CAN_F1R2_FB24_Msk
  CAN_F1R2_FB25_Pos* = (25)
  CAN_F1R2_FB25_Msk* = (0x00000001 shl CAN_F1R2_FB25_Pos) ## !< 0x02000000
  CAN_F1R2_FB25* = CAN_F1R2_FB25_Msk
  CAN_F1R2_FB26_Pos* = (26)
  CAN_F1R2_FB26_Msk* = (0x00000001 shl CAN_F1R2_FB26_Pos) ## !< 0x04000000
  CAN_F1R2_FB26* = CAN_F1R2_FB26_Msk
  CAN_F1R2_FB27_Pos* = (27)
  CAN_F1R2_FB27_Msk* = (0x00000001 shl CAN_F1R2_FB27_Pos) ## !< 0x08000000
  CAN_F1R2_FB27* = CAN_F1R2_FB27_Msk
  CAN_F1R2_FB28_Pos* = (28)
  CAN_F1R2_FB28_Msk* = (0x00000001 shl CAN_F1R2_FB28_Pos) ## !< 0x10000000
  CAN_F1R2_FB28* = CAN_F1R2_FB28_Msk
  CAN_F1R2_FB29_Pos* = (29)
  CAN_F1R2_FB29_Msk* = (0x00000001 shl CAN_F1R2_FB29_Pos) ## !< 0x20000000
  CAN_F1R2_FB29* = CAN_F1R2_FB29_Msk
  CAN_F1R2_FB30_Pos* = (30)
  CAN_F1R2_FB30_Msk* = (0x00000001 shl CAN_F1R2_FB30_Pos) ## !< 0x40000000
  CAN_F1R2_FB30* = CAN_F1R2_FB30_Msk
  CAN_F1R2_FB31_Pos* = (31)
  CAN_F1R2_FB31_Msk* = (0x00000001 shl CAN_F1R2_FB31_Pos) ## !< 0x80000000
  CAN_F1R2_FB31* = CAN_F1R2_FB31_Msk

## ******************  Bit definition for CAN_F2R2 register  ******************

const
  CAN_F2R2_FB0_Pos* = (0)
  CAN_F2R2_FB0_Msk* = (0x00000001 shl CAN_F2R2_FB0_Pos) ## !< 0x00000001
  CAN_F2R2_FB0* = CAN_F2R2_FB0_Msk
  CAN_F2R2_FB1_Pos* = (1)
  CAN_F2R2_FB1_Msk* = (0x00000001 shl CAN_F2R2_FB1_Pos) ## !< 0x00000002
  CAN_F2R2_FB1* = CAN_F2R2_FB1_Msk
  CAN_F2R2_FB2_Pos* = (2)
  CAN_F2R2_FB2_Msk* = (0x00000001 shl CAN_F2R2_FB2_Pos) ## !< 0x00000004
  CAN_F2R2_FB2* = CAN_F2R2_FB2_Msk
  CAN_F2R2_FB3_Pos* = (3)
  CAN_F2R2_FB3_Msk* = (0x00000001 shl CAN_F2R2_FB3_Pos) ## !< 0x00000008
  CAN_F2R2_FB3* = CAN_F2R2_FB3_Msk
  CAN_F2R2_FB4_Pos* = (4)
  CAN_F2R2_FB4_Msk* = (0x00000001 shl CAN_F2R2_FB4_Pos) ## !< 0x00000010
  CAN_F2R2_FB4* = CAN_F2R2_FB4_Msk
  CAN_F2R2_FB5_Pos* = (5)
  CAN_F2R2_FB5_Msk* = (0x00000001 shl CAN_F2R2_FB5_Pos) ## !< 0x00000020
  CAN_F2R2_FB5* = CAN_F2R2_FB5_Msk
  CAN_F2R2_FB6_Pos* = (6)
  CAN_F2R2_FB6_Msk* = (0x00000001 shl CAN_F2R2_FB6_Pos) ## !< 0x00000040
  CAN_F2R2_FB6* = CAN_F2R2_FB6_Msk
  CAN_F2R2_FB7_Pos* = (7)
  CAN_F2R2_FB7_Msk* = (0x00000001 shl CAN_F2R2_FB7_Pos) ## !< 0x00000080
  CAN_F2R2_FB7* = CAN_F2R2_FB7_Msk
  CAN_F2R2_FB8_Pos* = (8)
  CAN_F2R2_FB8_Msk* = (0x00000001 shl CAN_F2R2_FB8_Pos) ## !< 0x00000100
  CAN_F2R2_FB8* = CAN_F2R2_FB8_Msk
  CAN_F2R2_FB9_Pos* = (9)
  CAN_F2R2_FB9_Msk* = (0x00000001 shl CAN_F2R2_FB9_Pos) ## !< 0x00000200
  CAN_F2R2_FB9* = CAN_F2R2_FB9_Msk
  CAN_F2R2_FB10_Pos* = (10)
  CAN_F2R2_FB10_Msk* = (0x00000001 shl CAN_F2R2_FB10_Pos) ## !< 0x00000400
  CAN_F2R2_FB10* = CAN_F2R2_FB10_Msk
  CAN_F2R2_FB11_Pos* = (11)
  CAN_F2R2_FB11_Msk* = (0x00000001 shl CAN_F2R2_FB11_Pos) ## !< 0x00000800
  CAN_F2R2_FB11* = CAN_F2R2_FB11_Msk
  CAN_F2R2_FB12_Pos* = (12)
  CAN_F2R2_FB12_Msk* = (0x00000001 shl CAN_F2R2_FB12_Pos) ## !< 0x00001000
  CAN_F2R2_FB12* = CAN_F2R2_FB12_Msk
  CAN_F2R2_FB13_Pos* = (13)
  CAN_F2R2_FB13_Msk* = (0x00000001 shl CAN_F2R2_FB13_Pos) ## !< 0x00002000
  CAN_F2R2_FB13* = CAN_F2R2_FB13_Msk
  CAN_F2R2_FB14_Pos* = (14)
  CAN_F2R2_FB14_Msk* = (0x00000001 shl CAN_F2R2_FB14_Pos) ## !< 0x00004000
  CAN_F2R2_FB14* = CAN_F2R2_FB14_Msk
  CAN_F2R2_FB15_Pos* = (15)
  CAN_F2R2_FB15_Msk* = (0x00000001 shl CAN_F2R2_FB15_Pos) ## !< 0x00008000
  CAN_F2R2_FB15* = CAN_F2R2_FB15_Msk
  CAN_F2R2_FB16_Pos* = (16)
  CAN_F2R2_FB16_Msk* = (0x00000001 shl CAN_F2R2_FB16_Pos) ## !< 0x00010000
  CAN_F2R2_FB16* = CAN_F2R2_FB16_Msk
  CAN_F2R2_FB17_Pos* = (17)
  CAN_F2R2_FB17_Msk* = (0x00000001 shl CAN_F2R2_FB17_Pos) ## !< 0x00020000
  CAN_F2R2_FB17* = CAN_F2R2_FB17_Msk
  CAN_F2R2_FB18_Pos* = (18)
  CAN_F2R2_FB18_Msk* = (0x00000001 shl CAN_F2R2_FB18_Pos) ## !< 0x00040000
  CAN_F2R2_FB18* = CAN_F2R2_FB18_Msk
  CAN_F2R2_FB19_Pos* = (19)
  CAN_F2R2_FB19_Msk* = (0x00000001 shl CAN_F2R2_FB19_Pos) ## !< 0x00080000
  CAN_F2R2_FB19* = CAN_F2R2_FB19_Msk
  CAN_F2R2_FB20_Pos* = (20)
  CAN_F2R2_FB20_Msk* = (0x00000001 shl CAN_F2R2_FB20_Pos) ## !< 0x00100000
  CAN_F2R2_FB20* = CAN_F2R2_FB20_Msk
  CAN_F2R2_FB21_Pos* = (21)
  CAN_F2R2_FB21_Msk* = (0x00000001 shl CAN_F2R2_FB21_Pos) ## !< 0x00200000
  CAN_F2R2_FB21* = CAN_F2R2_FB21_Msk
  CAN_F2R2_FB22_Pos* = (22)
  CAN_F2R2_FB22_Msk* = (0x00000001 shl CAN_F2R2_FB22_Pos) ## !< 0x00400000
  CAN_F2R2_FB22* = CAN_F2R2_FB22_Msk
  CAN_F2R2_FB23_Pos* = (23)
  CAN_F2R2_FB23_Msk* = (0x00000001 shl CAN_F2R2_FB23_Pos) ## !< 0x00800000
  CAN_F2R2_FB23* = CAN_F2R2_FB23_Msk
  CAN_F2R2_FB24_Pos* = (24)
  CAN_F2R2_FB24_Msk* = (0x00000001 shl CAN_F2R2_FB24_Pos) ## !< 0x01000000
  CAN_F2R2_FB24* = CAN_F2R2_FB24_Msk
  CAN_F2R2_FB25_Pos* = (25)
  CAN_F2R2_FB25_Msk* = (0x00000001 shl CAN_F2R2_FB25_Pos) ## !< 0x02000000
  CAN_F2R2_FB25* = CAN_F2R2_FB25_Msk
  CAN_F2R2_FB26_Pos* = (26)
  CAN_F2R2_FB26_Msk* = (0x00000001 shl CAN_F2R2_FB26_Pos) ## !< 0x04000000
  CAN_F2R2_FB26* = CAN_F2R2_FB26_Msk
  CAN_F2R2_FB27_Pos* = (27)
  CAN_F2R2_FB27_Msk* = (0x00000001 shl CAN_F2R2_FB27_Pos) ## !< 0x08000000
  CAN_F2R2_FB27* = CAN_F2R2_FB27_Msk
  CAN_F2R2_FB28_Pos* = (28)
  CAN_F2R2_FB28_Msk* = (0x00000001 shl CAN_F2R2_FB28_Pos) ## !< 0x10000000
  CAN_F2R2_FB28* = CAN_F2R2_FB28_Msk
  CAN_F2R2_FB29_Pos* = (29)
  CAN_F2R2_FB29_Msk* = (0x00000001 shl CAN_F2R2_FB29_Pos) ## !< 0x20000000
  CAN_F2R2_FB29* = CAN_F2R2_FB29_Msk
  CAN_F2R2_FB30_Pos* = (30)
  CAN_F2R2_FB30_Msk* = (0x00000001 shl CAN_F2R2_FB30_Pos) ## !< 0x40000000
  CAN_F2R2_FB30* = CAN_F2R2_FB30_Msk
  CAN_F2R2_FB31_Pos* = (31)
  CAN_F2R2_FB31_Msk* = (0x00000001 shl CAN_F2R2_FB31_Pos) ## !< 0x80000000
  CAN_F2R2_FB31* = CAN_F2R2_FB31_Msk

## ******************  Bit definition for CAN_F3R2 register  ******************

const
  CAN_F3R2_FB0_Pos* = (0)
  CAN_F3R2_FB0_Msk* = (0x00000001 shl CAN_F3R2_FB0_Pos) ## !< 0x00000001
  CAN_F3R2_FB0* = CAN_F3R2_FB0_Msk
  CAN_F3R2_FB1_Pos* = (1)
  CAN_F3R2_FB1_Msk* = (0x00000001 shl CAN_F3R2_FB1_Pos) ## !< 0x00000002
  CAN_F3R2_FB1* = CAN_F3R2_FB1_Msk
  CAN_F3R2_FB2_Pos* = (2)
  CAN_F3R2_FB2_Msk* = (0x00000001 shl CAN_F3R2_FB2_Pos) ## !< 0x00000004
  CAN_F3R2_FB2* = CAN_F3R2_FB2_Msk
  CAN_F3R2_FB3_Pos* = (3)
  CAN_F3R2_FB3_Msk* = (0x00000001 shl CAN_F3R2_FB3_Pos) ## !< 0x00000008
  CAN_F3R2_FB3* = CAN_F3R2_FB3_Msk
  CAN_F3R2_FB4_Pos* = (4)
  CAN_F3R2_FB4_Msk* = (0x00000001 shl CAN_F3R2_FB4_Pos) ## !< 0x00000010
  CAN_F3R2_FB4* = CAN_F3R2_FB4_Msk
  CAN_F3R2_FB5_Pos* = (5)
  CAN_F3R2_FB5_Msk* = (0x00000001 shl CAN_F3R2_FB5_Pos) ## !< 0x00000020
  CAN_F3R2_FB5* = CAN_F3R2_FB5_Msk
  CAN_F3R2_FB6_Pos* = (6)
  CAN_F3R2_FB6_Msk* = (0x00000001 shl CAN_F3R2_FB6_Pos) ## !< 0x00000040
  CAN_F3R2_FB6* = CAN_F3R2_FB6_Msk
  CAN_F3R2_FB7_Pos* = (7)
  CAN_F3R2_FB7_Msk* = (0x00000001 shl CAN_F3R2_FB7_Pos) ## !< 0x00000080
  CAN_F3R2_FB7* = CAN_F3R2_FB7_Msk
  CAN_F3R2_FB8_Pos* = (8)
  CAN_F3R2_FB8_Msk* = (0x00000001 shl CAN_F3R2_FB8_Pos) ## !< 0x00000100
  CAN_F3R2_FB8* = CAN_F3R2_FB8_Msk
  CAN_F3R2_FB9_Pos* = (9)
  CAN_F3R2_FB9_Msk* = (0x00000001 shl CAN_F3R2_FB9_Pos) ## !< 0x00000200
  CAN_F3R2_FB9* = CAN_F3R2_FB9_Msk
  CAN_F3R2_FB10_Pos* = (10)
  CAN_F3R2_FB10_Msk* = (0x00000001 shl CAN_F3R2_FB10_Pos) ## !< 0x00000400
  CAN_F3R2_FB10* = CAN_F3R2_FB10_Msk
  CAN_F3R2_FB11_Pos* = (11)
  CAN_F3R2_FB11_Msk* = (0x00000001 shl CAN_F3R2_FB11_Pos) ## !< 0x00000800
  CAN_F3R2_FB11* = CAN_F3R2_FB11_Msk
  CAN_F3R2_FB12_Pos* = (12)
  CAN_F3R2_FB12_Msk* = (0x00000001 shl CAN_F3R2_FB12_Pos) ## !< 0x00001000
  CAN_F3R2_FB12* = CAN_F3R2_FB12_Msk
  CAN_F3R2_FB13_Pos* = (13)
  CAN_F3R2_FB13_Msk* = (0x00000001 shl CAN_F3R2_FB13_Pos) ## !< 0x00002000
  CAN_F3R2_FB13* = CAN_F3R2_FB13_Msk
  CAN_F3R2_FB14_Pos* = (14)
  CAN_F3R2_FB14_Msk* = (0x00000001 shl CAN_F3R2_FB14_Pos) ## !< 0x00004000
  CAN_F3R2_FB14* = CAN_F3R2_FB14_Msk
  CAN_F3R2_FB15_Pos* = (15)
  CAN_F3R2_FB15_Msk* = (0x00000001 shl CAN_F3R2_FB15_Pos) ## !< 0x00008000
  CAN_F3R2_FB15* = CAN_F3R2_FB15_Msk
  CAN_F3R2_FB16_Pos* = (16)
  CAN_F3R2_FB16_Msk* = (0x00000001 shl CAN_F3R2_FB16_Pos) ## !< 0x00010000
  CAN_F3R2_FB16* = CAN_F3R2_FB16_Msk
  CAN_F3R2_FB17_Pos* = (17)
  CAN_F3R2_FB17_Msk* = (0x00000001 shl CAN_F3R2_FB17_Pos) ## !< 0x00020000
  CAN_F3R2_FB17* = CAN_F3R2_FB17_Msk
  CAN_F3R2_FB18_Pos* = (18)
  CAN_F3R2_FB18_Msk* = (0x00000001 shl CAN_F3R2_FB18_Pos) ## !< 0x00040000
  CAN_F3R2_FB18* = CAN_F3R2_FB18_Msk
  CAN_F3R2_FB19_Pos* = (19)
  CAN_F3R2_FB19_Msk* = (0x00000001 shl CAN_F3R2_FB19_Pos) ## !< 0x00080000
  CAN_F3R2_FB19* = CAN_F3R2_FB19_Msk
  CAN_F3R2_FB20_Pos* = (20)
  CAN_F3R2_FB20_Msk* = (0x00000001 shl CAN_F3R2_FB20_Pos) ## !< 0x00100000
  CAN_F3R2_FB20* = CAN_F3R2_FB20_Msk
  CAN_F3R2_FB21_Pos* = (21)
  CAN_F3R2_FB21_Msk* = (0x00000001 shl CAN_F3R2_FB21_Pos) ## !< 0x00200000
  CAN_F3R2_FB21* = CAN_F3R2_FB21_Msk
  CAN_F3R2_FB22_Pos* = (22)
  CAN_F3R2_FB22_Msk* = (0x00000001 shl CAN_F3R2_FB22_Pos) ## !< 0x00400000
  CAN_F3R2_FB22* = CAN_F3R2_FB22_Msk
  CAN_F3R2_FB23_Pos* = (23)
  CAN_F3R2_FB23_Msk* = (0x00000001 shl CAN_F3R2_FB23_Pos) ## !< 0x00800000
  CAN_F3R2_FB23* = CAN_F3R2_FB23_Msk
  CAN_F3R2_FB24_Pos* = (24)
  CAN_F3R2_FB24_Msk* = (0x00000001 shl CAN_F3R2_FB24_Pos) ## !< 0x01000000
  CAN_F3R2_FB24* = CAN_F3R2_FB24_Msk
  CAN_F3R2_FB25_Pos* = (25)
  CAN_F3R2_FB25_Msk* = (0x00000001 shl CAN_F3R2_FB25_Pos) ## !< 0x02000000
  CAN_F3R2_FB25* = CAN_F3R2_FB25_Msk
  CAN_F3R2_FB26_Pos* = (26)
  CAN_F3R2_FB26_Msk* = (0x00000001 shl CAN_F3R2_FB26_Pos) ## !< 0x04000000
  CAN_F3R2_FB26* = CAN_F3R2_FB26_Msk
  CAN_F3R2_FB27_Pos* = (27)
  CAN_F3R2_FB27_Msk* = (0x00000001 shl CAN_F3R2_FB27_Pos) ## !< 0x08000000
  CAN_F3R2_FB27* = CAN_F3R2_FB27_Msk
  CAN_F3R2_FB28_Pos* = (28)
  CAN_F3R2_FB28_Msk* = (0x00000001 shl CAN_F3R2_FB28_Pos) ## !< 0x10000000
  CAN_F3R2_FB28* = CAN_F3R2_FB28_Msk
  CAN_F3R2_FB29_Pos* = (29)
  CAN_F3R2_FB29_Msk* = (0x00000001 shl CAN_F3R2_FB29_Pos) ## !< 0x20000000
  CAN_F3R2_FB29* = CAN_F3R2_FB29_Msk
  CAN_F3R2_FB30_Pos* = (30)
  CAN_F3R2_FB30_Msk* = (0x00000001 shl CAN_F3R2_FB30_Pos) ## !< 0x40000000
  CAN_F3R2_FB30* = CAN_F3R2_FB30_Msk
  CAN_F3R2_FB31_Pos* = (31)
  CAN_F3R2_FB31_Msk* = (0x00000001 shl CAN_F3R2_FB31_Pos) ## !< 0x80000000
  CAN_F3R2_FB31* = CAN_F3R2_FB31_Msk

## ******************  Bit definition for CAN_F4R2 register  ******************

const
  CAN_F4R2_FB0_Pos* = (0)
  CAN_F4R2_FB0_Msk* = (0x00000001 shl CAN_F4R2_FB0_Pos) ## !< 0x00000001
  CAN_F4R2_FB0* = CAN_F4R2_FB0_Msk
  CAN_F4R2_FB1_Pos* = (1)
  CAN_F4R2_FB1_Msk* = (0x00000001 shl CAN_F4R2_FB1_Pos) ## !< 0x00000002
  CAN_F4R2_FB1* = CAN_F4R2_FB1_Msk
  CAN_F4R2_FB2_Pos* = (2)
  CAN_F4R2_FB2_Msk* = (0x00000001 shl CAN_F4R2_FB2_Pos) ## !< 0x00000004
  CAN_F4R2_FB2* = CAN_F4R2_FB2_Msk
  CAN_F4R2_FB3_Pos* = (3)
  CAN_F4R2_FB3_Msk* = (0x00000001 shl CAN_F4R2_FB3_Pos) ## !< 0x00000008
  CAN_F4R2_FB3* = CAN_F4R2_FB3_Msk
  CAN_F4R2_FB4_Pos* = (4)
  CAN_F4R2_FB4_Msk* = (0x00000001 shl CAN_F4R2_FB4_Pos) ## !< 0x00000010
  CAN_F4R2_FB4* = CAN_F4R2_FB4_Msk
  CAN_F4R2_FB5_Pos* = (5)
  CAN_F4R2_FB5_Msk* = (0x00000001 shl CAN_F4R2_FB5_Pos) ## !< 0x00000020
  CAN_F4R2_FB5* = CAN_F4R2_FB5_Msk
  CAN_F4R2_FB6_Pos* = (6)
  CAN_F4R2_FB6_Msk* = (0x00000001 shl CAN_F4R2_FB6_Pos) ## !< 0x00000040
  CAN_F4R2_FB6* = CAN_F4R2_FB6_Msk
  CAN_F4R2_FB7_Pos* = (7)
  CAN_F4R2_FB7_Msk* = (0x00000001 shl CAN_F4R2_FB7_Pos) ## !< 0x00000080
  CAN_F4R2_FB7* = CAN_F4R2_FB7_Msk
  CAN_F4R2_FB8_Pos* = (8)
  CAN_F4R2_FB8_Msk* = (0x00000001 shl CAN_F4R2_FB8_Pos) ## !< 0x00000100
  CAN_F4R2_FB8* = CAN_F4R2_FB8_Msk
  CAN_F4R2_FB9_Pos* = (9)
  CAN_F4R2_FB9_Msk* = (0x00000001 shl CAN_F4R2_FB9_Pos) ## !< 0x00000200
  CAN_F4R2_FB9* = CAN_F4R2_FB9_Msk
  CAN_F4R2_FB10_Pos* = (10)
  CAN_F4R2_FB10_Msk* = (0x00000001 shl CAN_F4R2_FB10_Pos) ## !< 0x00000400
  CAN_F4R2_FB10* = CAN_F4R2_FB10_Msk
  CAN_F4R2_FB11_Pos* = (11)
  CAN_F4R2_FB11_Msk* = (0x00000001 shl CAN_F4R2_FB11_Pos) ## !< 0x00000800
  CAN_F4R2_FB11* = CAN_F4R2_FB11_Msk
  CAN_F4R2_FB12_Pos* = (12)
  CAN_F4R2_FB12_Msk* = (0x00000001 shl CAN_F4R2_FB12_Pos) ## !< 0x00001000
  CAN_F4R2_FB12* = CAN_F4R2_FB12_Msk
  CAN_F4R2_FB13_Pos* = (13)
  CAN_F4R2_FB13_Msk* = (0x00000001 shl CAN_F4R2_FB13_Pos) ## !< 0x00002000
  CAN_F4R2_FB13* = CAN_F4R2_FB13_Msk
  CAN_F4R2_FB14_Pos* = (14)
  CAN_F4R2_FB14_Msk* = (0x00000001 shl CAN_F4R2_FB14_Pos) ## !< 0x00004000
  CAN_F4R2_FB14* = CAN_F4R2_FB14_Msk
  CAN_F4R2_FB15_Pos* = (15)
  CAN_F4R2_FB15_Msk* = (0x00000001 shl CAN_F4R2_FB15_Pos) ## !< 0x00008000
  CAN_F4R2_FB15* = CAN_F4R2_FB15_Msk
  CAN_F4R2_FB16_Pos* = (16)
  CAN_F4R2_FB16_Msk* = (0x00000001 shl CAN_F4R2_FB16_Pos) ## !< 0x00010000
  CAN_F4R2_FB16* = CAN_F4R2_FB16_Msk
  CAN_F4R2_FB17_Pos* = (17)
  CAN_F4R2_FB17_Msk* = (0x00000001 shl CAN_F4R2_FB17_Pos) ## !< 0x00020000
  CAN_F4R2_FB17* = CAN_F4R2_FB17_Msk
  CAN_F4R2_FB18_Pos* = (18)
  CAN_F4R2_FB18_Msk* = (0x00000001 shl CAN_F4R2_FB18_Pos) ## !< 0x00040000
  CAN_F4R2_FB18* = CAN_F4R2_FB18_Msk
  CAN_F4R2_FB19_Pos* = (19)
  CAN_F4R2_FB19_Msk* = (0x00000001 shl CAN_F4R2_FB19_Pos) ## !< 0x00080000
  CAN_F4R2_FB19* = CAN_F4R2_FB19_Msk
  CAN_F4R2_FB20_Pos* = (20)
  CAN_F4R2_FB20_Msk* = (0x00000001 shl CAN_F4R2_FB20_Pos) ## !< 0x00100000
  CAN_F4R2_FB20* = CAN_F4R2_FB20_Msk
  CAN_F4R2_FB21_Pos* = (21)
  CAN_F4R2_FB21_Msk* = (0x00000001 shl CAN_F4R2_FB21_Pos) ## !< 0x00200000
  CAN_F4R2_FB21* = CAN_F4R2_FB21_Msk
  CAN_F4R2_FB22_Pos* = (22)
  CAN_F4R2_FB22_Msk* = (0x00000001 shl CAN_F4R2_FB22_Pos) ## !< 0x00400000
  CAN_F4R2_FB22* = CAN_F4R2_FB22_Msk
  CAN_F4R2_FB23_Pos* = (23)
  CAN_F4R2_FB23_Msk* = (0x00000001 shl CAN_F4R2_FB23_Pos) ## !< 0x00800000
  CAN_F4R2_FB23* = CAN_F4R2_FB23_Msk
  CAN_F4R2_FB24_Pos* = (24)
  CAN_F4R2_FB24_Msk* = (0x00000001 shl CAN_F4R2_FB24_Pos) ## !< 0x01000000
  CAN_F4R2_FB24* = CAN_F4R2_FB24_Msk
  CAN_F4R2_FB25_Pos* = (25)
  CAN_F4R2_FB25_Msk* = (0x00000001 shl CAN_F4R2_FB25_Pos) ## !< 0x02000000
  CAN_F4R2_FB25* = CAN_F4R2_FB25_Msk
  CAN_F4R2_FB26_Pos* = (26)
  CAN_F4R2_FB26_Msk* = (0x00000001 shl CAN_F4R2_FB26_Pos) ## !< 0x04000000
  CAN_F4R2_FB26* = CAN_F4R2_FB26_Msk
  CAN_F4R2_FB27_Pos* = (27)
  CAN_F4R2_FB27_Msk* = (0x00000001 shl CAN_F4R2_FB27_Pos) ## !< 0x08000000
  CAN_F4R2_FB27* = CAN_F4R2_FB27_Msk
  CAN_F4R2_FB28_Pos* = (28)
  CAN_F4R2_FB28_Msk* = (0x00000001 shl CAN_F4R2_FB28_Pos) ## !< 0x10000000
  CAN_F4R2_FB28* = CAN_F4R2_FB28_Msk
  CAN_F4R2_FB29_Pos* = (29)
  CAN_F4R2_FB29_Msk* = (0x00000001 shl CAN_F4R2_FB29_Pos) ## !< 0x20000000
  CAN_F4R2_FB29* = CAN_F4R2_FB29_Msk
  CAN_F4R2_FB30_Pos* = (30)
  CAN_F4R2_FB30_Msk* = (0x00000001 shl CAN_F4R2_FB30_Pos) ## !< 0x40000000
  CAN_F4R2_FB30* = CAN_F4R2_FB30_Msk
  CAN_F4R2_FB31_Pos* = (31)
  CAN_F4R2_FB31_Msk* = (0x00000001 shl CAN_F4R2_FB31_Pos) ## !< 0x80000000
  CAN_F4R2_FB31* = CAN_F4R2_FB31_Msk

## ******************  Bit definition for CAN_F5R2 register  ******************

const
  CAN_F5R2_FB0_Pos* = (0)
  CAN_F5R2_FB0_Msk* = (0x00000001 shl CAN_F5R2_FB0_Pos) ## !< 0x00000001
  CAN_F5R2_FB0* = CAN_F5R2_FB0_Msk
  CAN_F5R2_FB1_Pos* = (1)
  CAN_F5R2_FB1_Msk* = (0x00000001 shl CAN_F5R2_FB1_Pos) ## !< 0x00000002
  CAN_F5R2_FB1* = CAN_F5R2_FB1_Msk
  CAN_F5R2_FB2_Pos* = (2)
  CAN_F5R2_FB2_Msk* = (0x00000001 shl CAN_F5R2_FB2_Pos) ## !< 0x00000004
  CAN_F5R2_FB2* = CAN_F5R2_FB2_Msk
  CAN_F5R2_FB3_Pos* = (3)
  CAN_F5R2_FB3_Msk* = (0x00000001 shl CAN_F5R2_FB3_Pos) ## !< 0x00000008
  CAN_F5R2_FB3* = CAN_F5R2_FB3_Msk
  CAN_F5R2_FB4_Pos* = (4)
  CAN_F5R2_FB4_Msk* = (0x00000001 shl CAN_F5R2_FB4_Pos) ## !< 0x00000010
  CAN_F5R2_FB4* = CAN_F5R2_FB4_Msk
  CAN_F5R2_FB5_Pos* = (5)
  CAN_F5R2_FB5_Msk* = (0x00000001 shl CAN_F5R2_FB5_Pos) ## !< 0x00000020
  CAN_F5R2_FB5* = CAN_F5R2_FB5_Msk
  CAN_F5R2_FB6_Pos* = (6)
  CAN_F5R2_FB6_Msk* = (0x00000001 shl CAN_F5R2_FB6_Pos) ## !< 0x00000040
  CAN_F5R2_FB6* = CAN_F5R2_FB6_Msk
  CAN_F5R2_FB7_Pos* = (7)
  CAN_F5R2_FB7_Msk* = (0x00000001 shl CAN_F5R2_FB7_Pos) ## !< 0x00000080
  CAN_F5R2_FB7* = CAN_F5R2_FB7_Msk
  CAN_F5R2_FB8_Pos* = (8)
  CAN_F5R2_FB8_Msk* = (0x00000001 shl CAN_F5R2_FB8_Pos) ## !< 0x00000100
  CAN_F5R2_FB8* = CAN_F5R2_FB8_Msk
  CAN_F5R2_FB9_Pos* = (9)
  CAN_F5R2_FB9_Msk* = (0x00000001 shl CAN_F5R2_FB9_Pos) ## !< 0x00000200
  CAN_F5R2_FB9* = CAN_F5R2_FB9_Msk
  CAN_F5R2_FB10_Pos* = (10)
  CAN_F5R2_FB10_Msk* = (0x00000001 shl CAN_F5R2_FB10_Pos) ## !< 0x00000400
  CAN_F5R2_FB10* = CAN_F5R2_FB10_Msk
  CAN_F5R2_FB11_Pos* = (11)
  CAN_F5R2_FB11_Msk* = (0x00000001 shl CAN_F5R2_FB11_Pos) ## !< 0x00000800
  CAN_F5R2_FB11* = CAN_F5R2_FB11_Msk
  CAN_F5R2_FB12_Pos* = (12)
  CAN_F5R2_FB12_Msk* = (0x00000001 shl CAN_F5R2_FB12_Pos) ## !< 0x00001000
  CAN_F5R2_FB12* = CAN_F5R2_FB12_Msk
  CAN_F5R2_FB13_Pos* = (13)
  CAN_F5R2_FB13_Msk* = (0x00000001 shl CAN_F5R2_FB13_Pos) ## !< 0x00002000
  CAN_F5R2_FB13* = CAN_F5R2_FB13_Msk
  CAN_F5R2_FB14_Pos* = (14)
  CAN_F5R2_FB14_Msk* = (0x00000001 shl CAN_F5R2_FB14_Pos) ## !< 0x00004000
  CAN_F5R2_FB14* = CAN_F5R2_FB14_Msk
  CAN_F5R2_FB15_Pos* = (15)
  CAN_F5R2_FB15_Msk* = (0x00000001 shl CAN_F5R2_FB15_Pos) ## !< 0x00008000
  CAN_F5R2_FB15* = CAN_F5R2_FB15_Msk
  CAN_F5R2_FB16_Pos* = (16)
  CAN_F5R2_FB16_Msk* = (0x00000001 shl CAN_F5R2_FB16_Pos) ## !< 0x00010000
  CAN_F5R2_FB16* = CAN_F5R2_FB16_Msk
  CAN_F5R2_FB17_Pos* = (17)
  CAN_F5R2_FB17_Msk* = (0x00000001 shl CAN_F5R2_FB17_Pos) ## !< 0x00020000
  CAN_F5R2_FB17* = CAN_F5R2_FB17_Msk
  CAN_F5R2_FB18_Pos* = (18)
  CAN_F5R2_FB18_Msk* = (0x00000001 shl CAN_F5R2_FB18_Pos) ## !< 0x00040000
  CAN_F5R2_FB18* = CAN_F5R2_FB18_Msk
  CAN_F5R2_FB19_Pos* = (19)
  CAN_F5R2_FB19_Msk* = (0x00000001 shl CAN_F5R2_FB19_Pos) ## !< 0x00080000
  CAN_F5R2_FB19* = CAN_F5R2_FB19_Msk
  CAN_F5R2_FB20_Pos* = (20)
  CAN_F5R2_FB20_Msk* = (0x00000001 shl CAN_F5R2_FB20_Pos) ## !< 0x00100000
  CAN_F5R2_FB20* = CAN_F5R2_FB20_Msk
  CAN_F5R2_FB21_Pos* = (21)
  CAN_F5R2_FB21_Msk* = (0x00000001 shl CAN_F5R2_FB21_Pos) ## !< 0x00200000
  CAN_F5R2_FB21* = CAN_F5R2_FB21_Msk
  CAN_F5R2_FB22_Pos* = (22)
  CAN_F5R2_FB22_Msk* = (0x00000001 shl CAN_F5R2_FB22_Pos) ## !< 0x00400000
  CAN_F5R2_FB22* = CAN_F5R2_FB22_Msk
  CAN_F5R2_FB23_Pos* = (23)
  CAN_F5R2_FB23_Msk* = (0x00000001 shl CAN_F5R2_FB23_Pos) ## !< 0x00800000
  CAN_F5R2_FB23* = CAN_F5R2_FB23_Msk
  CAN_F5R2_FB24_Pos* = (24)
  CAN_F5R2_FB24_Msk* = (0x00000001 shl CAN_F5R2_FB24_Pos) ## !< 0x01000000
  CAN_F5R2_FB24* = CAN_F5R2_FB24_Msk
  CAN_F5R2_FB25_Pos* = (25)
  CAN_F5R2_FB25_Msk* = (0x00000001 shl CAN_F5R2_FB25_Pos) ## !< 0x02000000
  CAN_F5R2_FB25* = CAN_F5R2_FB25_Msk
  CAN_F5R2_FB26_Pos* = (26)
  CAN_F5R2_FB26_Msk* = (0x00000001 shl CAN_F5R2_FB26_Pos) ## !< 0x04000000
  CAN_F5R2_FB26* = CAN_F5R2_FB26_Msk
  CAN_F5R2_FB27_Pos* = (27)
  CAN_F5R2_FB27_Msk* = (0x00000001 shl CAN_F5R2_FB27_Pos) ## !< 0x08000000
  CAN_F5R2_FB27* = CAN_F5R2_FB27_Msk
  CAN_F5R2_FB28_Pos* = (28)
  CAN_F5R2_FB28_Msk* = (0x00000001 shl CAN_F5R2_FB28_Pos) ## !< 0x10000000
  CAN_F5R2_FB28* = CAN_F5R2_FB28_Msk
  CAN_F5R2_FB29_Pos* = (29)
  CAN_F5R2_FB29_Msk* = (0x00000001 shl CAN_F5R2_FB29_Pos) ## !< 0x20000000
  CAN_F5R2_FB29* = CAN_F5R2_FB29_Msk
  CAN_F5R2_FB30_Pos* = (30)
  CAN_F5R2_FB30_Msk* = (0x00000001 shl CAN_F5R2_FB30_Pos) ## !< 0x40000000
  CAN_F5R2_FB30* = CAN_F5R2_FB30_Msk
  CAN_F5R2_FB31_Pos* = (31)
  CAN_F5R2_FB31_Msk* = (0x00000001 shl CAN_F5R2_FB31_Pos) ## !< 0x80000000
  CAN_F5R2_FB31* = CAN_F5R2_FB31_Msk

## ******************  Bit definition for CAN_F6R2 register  ******************

const
  CAN_F6R2_FB0_Pos* = (0)
  CAN_F6R2_FB0_Msk* = (0x00000001 shl CAN_F6R2_FB0_Pos) ## !< 0x00000001
  CAN_F6R2_FB0* = CAN_F6R2_FB0_Msk
  CAN_F6R2_FB1_Pos* = (1)
  CAN_F6R2_FB1_Msk* = (0x00000001 shl CAN_F6R2_FB1_Pos) ## !< 0x00000002
  CAN_F6R2_FB1* = CAN_F6R2_FB1_Msk
  CAN_F6R2_FB2_Pos* = (2)
  CAN_F6R2_FB2_Msk* = (0x00000001 shl CAN_F6R2_FB2_Pos) ## !< 0x00000004
  CAN_F6R2_FB2* = CAN_F6R2_FB2_Msk
  CAN_F6R2_FB3_Pos* = (3)
  CAN_F6R2_FB3_Msk* = (0x00000001 shl CAN_F6R2_FB3_Pos) ## !< 0x00000008
  CAN_F6R2_FB3* = CAN_F6R2_FB3_Msk
  CAN_F6R2_FB4_Pos* = (4)
  CAN_F6R2_FB4_Msk* = (0x00000001 shl CAN_F6R2_FB4_Pos) ## !< 0x00000010
  CAN_F6R2_FB4* = CAN_F6R2_FB4_Msk
  CAN_F6R2_FB5_Pos* = (5)
  CAN_F6R2_FB5_Msk* = (0x00000001 shl CAN_F6R2_FB5_Pos) ## !< 0x00000020
  CAN_F6R2_FB5* = CAN_F6R2_FB5_Msk
  CAN_F6R2_FB6_Pos* = (6)
  CAN_F6R2_FB6_Msk* = (0x00000001 shl CAN_F6R2_FB6_Pos) ## !< 0x00000040
  CAN_F6R2_FB6* = CAN_F6R2_FB6_Msk
  CAN_F6R2_FB7_Pos* = (7)
  CAN_F6R2_FB7_Msk* = (0x00000001 shl CAN_F6R2_FB7_Pos) ## !< 0x00000080
  CAN_F6R2_FB7* = CAN_F6R2_FB7_Msk
  CAN_F6R2_FB8_Pos* = (8)
  CAN_F6R2_FB8_Msk* = (0x00000001 shl CAN_F6R2_FB8_Pos) ## !< 0x00000100
  CAN_F6R2_FB8* = CAN_F6R2_FB8_Msk
  CAN_F6R2_FB9_Pos* = (9)
  CAN_F6R2_FB9_Msk* = (0x00000001 shl CAN_F6R2_FB9_Pos) ## !< 0x00000200
  CAN_F6R2_FB9* = CAN_F6R2_FB9_Msk
  CAN_F6R2_FB10_Pos* = (10)
  CAN_F6R2_FB10_Msk* = (0x00000001 shl CAN_F6R2_FB10_Pos) ## !< 0x00000400
  CAN_F6R2_FB10* = CAN_F6R2_FB10_Msk
  CAN_F6R2_FB11_Pos* = (11)
  CAN_F6R2_FB11_Msk* = (0x00000001 shl CAN_F6R2_FB11_Pos) ## !< 0x00000800
  CAN_F6R2_FB11* = CAN_F6R2_FB11_Msk
  CAN_F6R2_FB12_Pos* = (12)
  CAN_F6R2_FB12_Msk* = (0x00000001 shl CAN_F6R2_FB12_Pos) ## !< 0x00001000
  CAN_F6R2_FB12* = CAN_F6R2_FB12_Msk
  CAN_F6R2_FB13_Pos* = (13)
  CAN_F6R2_FB13_Msk* = (0x00000001 shl CAN_F6R2_FB13_Pos) ## !< 0x00002000
  CAN_F6R2_FB13* = CAN_F6R2_FB13_Msk
  CAN_F6R2_FB14_Pos* = (14)
  CAN_F6R2_FB14_Msk* = (0x00000001 shl CAN_F6R2_FB14_Pos) ## !< 0x00004000
  CAN_F6R2_FB14* = CAN_F6R2_FB14_Msk
  CAN_F6R2_FB15_Pos* = (15)
  CAN_F6R2_FB15_Msk* = (0x00000001 shl CAN_F6R2_FB15_Pos) ## !< 0x00008000
  CAN_F6R2_FB15* = CAN_F6R2_FB15_Msk
  CAN_F6R2_FB16_Pos* = (16)
  CAN_F6R2_FB16_Msk* = (0x00000001 shl CAN_F6R2_FB16_Pos) ## !< 0x00010000
  CAN_F6R2_FB16* = CAN_F6R2_FB16_Msk
  CAN_F6R2_FB17_Pos* = (17)
  CAN_F6R2_FB17_Msk* = (0x00000001 shl CAN_F6R2_FB17_Pos) ## !< 0x00020000
  CAN_F6R2_FB17* = CAN_F6R2_FB17_Msk
  CAN_F6R2_FB18_Pos* = (18)
  CAN_F6R2_FB18_Msk* = (0x00000001 shl CAN_F6R2_FB18_Pos) ## !< 0x00040000
  CAN_F6R2_FB18* = CAN_F6R2_FB18_Msk
  CAN_F6R2_FB19_Pos* = (19)
  CAN_F6R2_FB19_Msk* = (0x00000001 shl CAN_F6R2_FB19_Pos) ## !< 0x00080000
  CAN_F6R2_FB19* = CAN_F6R2_FB19_Msk
  CAN_F6R2_FB20_Pos* = (20)
  CAN_F6R2_FB20_Msk* = (0x00000001 shl CAN_F6R2_FB20_Pos) ## !< 0x00100000
  CAN_F6R2_FB20* = CAN_F6R2_FB20_Msk
  CAN_F6R2_FB21_Pos* = (21)
  CAN_F6R2_FB21_Msk* = (0x00000001 shl CAN_F6R2_FB21_Pos) ## !< 0x00200000
  CAN_F6R2_FB21* = CAN_F6R2_FB21_Msk
  CAN_F6R2_FB22_Pos* = (22)
  CAN_F6R2_FB22_Msk* = (0x00000001 shl CAN_F6R2_FB22_Pos) ## !< 0x00400000
  CAN_F6R2_FB22* = CAN_F6R2_FB22_Msk
  CAN_F6R2_FB23_Pos* = (23)
  CAN_F6R2_FB23_Msk* = (0x00000001 shl CAN_F6R2_FB23_Pos) ## !< 0x00800000
  CAN_F6R2_FB23* = CAN_F6R2_FB23_Msk
  CAN_F6R2_FB24_Pos* = (24)
  CAN_F6R2_FB24_Msk* = (0x00000001 shl CAN_F6R2_FB24_Pos) ## !< 0x01000000
  CAN_F6R2_FB24* = CAN_F6R2_FB24_Msk
  CAN_F6R2_FB25_Pos* = (25)
  CAN_F6R2_FB25_Msk* = (0x00000001 shl CAN_F6R2_FB25_Pos) ## !< 0x02000000
  CAN_F6R2_FB25* = CAN_F6R2_FB25_Msk
  CAN_F6R2_FB26_Pos* = (26)
  CAN_F6R2_FB26_Msk* = (0x00000001 shl CAN_F6R2_FB26_Pos) ## !< 0x04000000
  CAN_F6R2_FB26* = CAN_F6R2_FB26_Msk
  CAN_F6R2_FB27_Pos* = (27)
  CAN_F6R2_FB27_Msk* = (0x00000001 shl CAN_F6R2_FB27_Pos) ## !< 0x08000000
  CAN_F6R2_FB27* = CAN_F6R2_FB27_Msk
  CAN_F6R2_FB28_Pos* = (28)
  CAN_F6R2_FB28_Msk* = (0x00000001 shl CAN_F6R2_FB28_Pos) ## !< 0x10000000
  CAN_F6R2_FB28* = CAN_F6R2_FB28_Msk
  CAN_F6R2_FB29_Pos* = (29)
  CAN_F6R2_FB29_Msk* = (0x00000001 shl CAN_F6R2_FB29_Pos) ## !< 0x20000000
  CAN_F6R2_FB29* = CAN_F6R2_FB29_Msk
  CAN_F6R2_FB30_Pos* = (30)
  CAN_F6R2_FB30_Msk* = (0x00000001 shl CAN_F6R2_FB30_Pos) ## !< 0x40000000
  CAN_F6R2_FB30* = CAN_F6R2_FB30_Msk
  CAN_F6R2_FB31_Pos* = (31)
  CAN_F6R2_FB31_Msk* = (0x00000001 shl CAN_F6R2_FB31_Pos) ## !< 0x80000000
  CAN_F6R2_FB31* = CAN_F6R2_FB31_Msk

## ******************  Bit definition for CAN_F7R2 register  ******************

const
  CAN_F7R2_FB0_Pos* = (0)
  CAN_F7R2_FB0_Msk* = (0x00000001 shl CAN_F7R2_FB0_Pos) ## !< 0x00000001
  CAN_F7R2_FB0* = CAN_F7R2_FB0_Msk
  CAN_F7R2_FB1_Pos* = (1)
  CAN_F7R2_FB1_Msk* = (0x00000001 shl CAN_F7R2_FB1_Pos) ## !< 0x00000002
  CAN_F7R2_FB1* = CAN_F7R2_FB1_Msk
  CAN_F7R2_FB2_Pos* = (2)
  CAN_F7R2_FB2_Msk* = (0x00000001 shl CAN_F7R2_FB2_Pos) ## !< 0x00000004
  CAN_F7R2_FB2* = CAN_F7R2_FB2_Msk
  CAN_F7R2_FB3_Pos* = (3)
  CAN_F7R2_FB3_Msk* = (0x00000001 shl CAN_F7R2_FB3_Pos) ## !< 0x00000008
  CAN_F7R2_FB3* = CAN_F7R2_FB3_Msk
  CAN_F7R2_FB4_Pos* = (4)
  CAN_F7R2_FB4_Msk* = (0x00000001 shl CAN_F7R2_FB4_Pos) ## !< 0x00000010
  CAN_F7R2_FB4* = CAN_F7R2_FB4_Msk
  CAN_F7R2_FB5_Pos* = (5)
  CAN_F7R2_FB5_Msk* = (0x00000001 shl CAN_F7R2_FB5_Pos) ## !< 0x00000020
  CAN_F7R2_FB5* = CAN_F7R2_FB5_Msk
  CAN_F7R2_FB6_Pos* = (6)
  CAN_F7R2_FB6_Msk* = (0x00000001 shl CAN_F7R2_FB6_Pos) ## !< 0x00000040
  CAN_F7R2_FB6* = CAN_F7R2_FB6_Msk
  CAN_F7R2_FB7_Pos* = (7)
  CAN_F7R2_FB7_Msk* = (0x00000001 shl CAN_F7R2_FB7_Pos) ## !< 0x00000080
  CAN_F7R2_FB7* = CAN_F7R2_FB7_Msk
  CAN_F7R2_FB8_Pos* = (8)
  CAN_F7R2_FB8_Msk* = (0x00000001 shl CAN_F7R2_FB8_Pos) ## !< 0x00000100
  CAN_F7R2_FB8* = CAN_F7R2_FB8_Msk
  CAN_F7R2_FB9_Pos* = (9)
  CAN_F7R2_FB9_Msk* = (0x00000001 shl CAN_F7R2_FB9_Pos) ## !< 0x00000200
  CAN_F7R2_FB9* = CAN_F7R2_FB9_Msk
  CAN_F7R2_FB10_Pos* = (10)
  CAN_F7R2_FB10_Msk* = (0x00000001 shl CAN_F7R2_FB10_Pos) ## !< 0x00000400
  CAN_F7R2_FB10* = CAN_F7R2_FB10_Msk
  CAN_F7R2_FB11_Pos* = (11)
  CAN_F7R2_FB11_Msk* = (0x00000001 shl CAN_F7R2_FB11_Pos) ## !< 0x00000800
  CAN_F7R2_FB11* = CAN_F7R2_FB11_Msk
  CAN_F7R2_FB12_Pos* = (12)
  CAN_F7R2_FB12_Msk* = (0x00000001 shl CAN_F7R2_FB12_Pos) ## !< 0x00001000
  CAN_F7R2_FB12* = CAN_F7R2_FB12_Msk
  CAN_F7R2_FB13_Pos* = (13)
  CAN_F7R2_FB13_Msk* = (0x00000001 shl CAN_F7R2_FB13_Pos) ## !< 0x00002000
  CAN_F7R2_FB13* = CAN_F7R2_FB13_Msk
  CAN_F7R2_FB14_Pos* = (14)
  CAN_F7R2_FB14_Msk* = (0x00000001 shl CAN_F7R2_FB14_Pos) ## !< 0x00004000
  CAN_F7R2_FB14* = CAN_F7R2_FB14_Msk
  CAN_F7R2_FB15_Pos* = (15)
  CAN_F7R2_FB15_Msk* = (0x00000001 shl CAN_F7R2_FB15_Pos) ## !< 0x00008000
  CAN_F7R2_FB15* = CAN_F7R2_FB15_Msk
  CAN_F7R2_FB16_Pos* = (16)
  CAN_F7R2_FB16_Msk* = (0x00000001 shl CAN_F7R2_FB16_Pos) ## !< 0x00010000
  CAN_F7R2_FB16* = CAN_F7R2_FB16_Msk
  CAN_F7R2_FB17_Pos* = (17)
  CAN_F7R2_FB17_Msk* = (0x00000001 shl CAN_F7R2_FB17_Pos) ## !< 0x00020000
  CAN_F7R2_FB17* = CAN_F7R2_FB17_Msk
  CAN_F7R2_FB18_Pos* = (18)
  CAN_F7R2_FB18_Msk* = (0x00000001 shl CAN_F7R2_FB18_Pos) ## !< 0x00040000
  CAN_F7R2_FB18* = CAN_F7R2_FB18_Msk
  CAN_F7R2_FB19_Pos* = (19)
  CAN_F7R2_FB19_Msk* = (0x00000001 shl CAN_F7R2_FB19_Pos) ## !< 0x00080000
  CAN_F7R2_FB19* = CAN_F7R2_FB19_Msk
  CAN_F7R2_FB20_Pos* = (20)
  CAN_F7R2_FB20_Msk* = (0x00000001 shl CAN_F7R2_FB20_Pos) ## !< 0x00100000
  CAN_F7R2_FB20* = CAN_F7R2_FB20_Msk
  CAN_F7R2_FB21_Pos* = (21)
  CAN_F7R2_FB21_Msk* = (0x00000001 shl CAN_F7R2_FB21_Pos) ## !< 0x00200000
  CAN_F7R2_FB21* = CAN_F7R2_FB21_Msk
  CAN_F7R2_FB22_Pos* = (22)
  CAN_F7R2_FB22_Msk* = (0x00000001 shl CAN_F7R2_FB22_Pos) ## !< 0x00400000
  CAN_F7R2_FB22* = CAN_F7R2_FB22_Msk
  CAN_F7R2_FB23_Pos* = (23)
  CAN_F7R2_FB23_Msk* = (0x00000001 shl CAN_F7R2_FB23_Pos) ## !< 0x00800000
  CAN_F7R2_FB23* = CAN_F7R2_FB23_Msk
  CAN_F7R2_FB24_Pos* = (24)
  CAN_F7R2_FB24_Msk* = (0x00000001 shl CAN_F7R2_FB24_Pos) ## !< 0x01000000
  CAN_F7R2_FB24* = CAN_F7R2_FB24_Msk
  CAN_F7R2_FB25_Pos* = (25)
  CAN_F7R2_FB25_Msk* = (0x00000001 shl CAN_F7R2_FB25_Pos) ## !< 0x02000000
  CAN_F7R2_FB25* = CAN_F7R2_FB25_Msk
  CAN_F7R2_FB26_Pos* = (26)
  CAN_F7R2_FB26_Msk* = (0x00000001 shl CAN_F7R2_FB26_Pos) ## !< 0x04000000
  CAN_F7R2_FB26* = CAN_F7R2_FB26_Msk
  CAN_F7R2_FB27_Pos* = (27)
  CAN_F7R2_FB27_Msk* = (0x00000001 shl CAN_F7R2_FB27_Pos) ## !< 0x08000000
  CAN_F7R2_FB27* = CAN_F7R2_FB27_Msk
  CAN_F7R2_FB28_Pos* = (28)
  CAN_F7R2_FB28_Msk* = (0x00000001 shl CAN_F7R2_FB28_Pos) ## !< 0x10000000
  CAN_F7R2_FB28* = CAN_F7R2_FB28_Msk
  CAN_F7R2_FB29_Pos* = (29)
  CAN_F7R2_FB29_Msk* = (0x00000001 shl CAN_F7R2_FB29_Pos) ## !< 0x20000000
  CAN_F7R2_FB29* = CAN_F7R2_FB29_Msk
  CAN_F7R2_FB30_Pos* = (30)
  CAN_F7R2_FB30_Msk* = (0x00000001 shl CAN_F7R2_FB30_Pos) ## !< 0x40000000
  CAN_F7R2_FB30* = CAN_F7R2_FB30_Msk
  CAN_F7R2_FB31_Pos* = (31)
  CAN_F7R2_FB31_Msk* = (0x00000001 shl CAN_F7R2_FB31_Pos) ## !< 0x80000000
  CAN_F7R2_FB31* = CAN_F7R2_FB31_Msk

## ******************  Bit definition for CAN_F8R2 register  ******************

const
  CAN_F8R2_FB0_Pos* = (0)
  CAN_F8R2_FB0_Msk* = (0x00000001 shl CAN_F8R2_FB0_Pos) ## !< 0x00000001
  CAN_F8R2_FB0* = CAN_F8R2_FB0_Msk
  CAN_F8R2_FB1_Pos* = (1)
  CAN_F8R2_FB1_Msk* = (0x00000001 shl CAN_F8R2_FB1_Pos) ## !< 0x00000002
  CAN_F8R2_FB1* = CAN_F8R2_FB1_Msk
  CAN_F8R2_FB2_Pos* = (2)
  CAN_F8R2_FB2_Msk* = (0x00000001 shl CAN_F8R2_FB2_Pos) ## !< 0x00000004
  CAN_F8R2_FB2* = CAN_F8R2_FB2_Msk
  CAN_F8R2_FB3_Pos* = (3)
  CAN_F8R2_FB3_Msk* = (0x00000001 shl CAN_F8R2_FB3_Pos) ## !< 0x00000008
  CAN_F8R2_FB3* = CAN_F8R2_FB3_Msk
  CAN_F8R2_FB4_Pos* = (4)
  CAN_F8R2_FB4_Msk* = (0x00000001 shl CAN_F8R2_FB4_Pos) ## !< 0x00000010
  CAN_F8R2_FB4* = CAN_F8R2_FB4_Msk
  CAN_F8R2_FB5_Pos* = (5)
  CAN_F8R2_FB5_Msk* = (0x00000001 shl CAN_F8R2_FB5_Pos) ## !< 0x00000020
  CAN_F8R2_FB5* = CAN_F8R2_FB5_Msk
  CAN_F8R2_FB6_Pos* = (6)
  CAN_F8R2_FB6_Msk* = (0x00000001 shl CAN_F8R2_FB6_Pos) ## !< 0x00000040
  CAN_F8R2_FB6* = CAN_F8R2_FB6_Msk
  CAN_F8R2_FB7_Pos* = (7)
  CAN_F8R2_FB7_Msk* = (0x00000001 shl CAN_F8R2_FB7_Pos) ## !< 0x00000080
  CAN_F8R2_FB7* = CAN_F8R2_FB7_Msk
  CAN_F8R2_FB8_Pos* = (8)
  CAN_F8R2_FB8_Msk* = (0x00000001 shl CAN_F8R2_FB8_Pos) ## !< 0x00000100
  CAN_F8R2_FB8* = CAN_F8R2_FB8_Msk
  CAN_F8R2_FB9_Pos* = (9)
  CAN_F8R2_FB9_Msk* = (0x00000001 shl CAN_F8R2_FB9_Pos) ## !< 0x00000200
  CAN_F8R2_FB9* = CAN_F8R2_FB9_Msk
  CAN_F8R2_FB10_Pos* = (10)
  CAN_F8R2_FB10_Msk* = (0x00000001 shl CAN_F8R2_FB10_Pos) ## !< 0x00000400
  CAN_F8R2_FB10* = CAN_F8R2_FB10_Msk
  CAN_F8R2_FB11_Pos* = (11)
  CAN_F8R2_FB11_Msk* = (0x00000001 shl CAN_F8R2_FB11_Pos) ## !< 0x00000800
  CAN_F8R2_FB11* = CAN_F8R2_FB11_Msk
  CAN_F8R2_FB12_Pos* = (12)
  CAN_F8R2_FB12_Msk* = (0x00000001 shl CAN_F8R2_FB12_Pos) ## !< 0x00001000
  CAN_F8R2_FB12* = CAN_F8R2_FB12_Msk
  CAN_F8R2_FB13_Pos* = (13)
  CAN_F8R2_FB13_Msk* = (0x00000001 shl CAN_F8R2_FB13_Pos) ## !< 0x00002000
  CAN_F8R2_FB13* = CAN_F8R2_FB13_Msk
  CAN_F8R2_FB14_Pos* = (14)
  CAN_F8R2_FB14_Msk* = (0x00000001 shl CAN_F8R2_FB14_Pos) ## !< 0x00004000
  CAN_F8R2_FB14* = CAN_F8R2_FB14_Msk
  CAN_F8R2_FB15_Pos* = (15)
  CAN_F8R2_FB15_Msk* = (0x00000001 shl CAN_F8R2_FB15_Pos) ## !< 0x00008000
  CAN_F8R2_FB15* = CAN_F8R2_FB15_Msk
  CAN_F8R2_FB16_Pos* = (16)
  CAN_F8R2_FB16_Msk* = (0x00000001 shl CAN_F8R2_FB16_Pos) ## !< 0x00010000
  CAN_F8R2_FB16* = CAN_F8R2_FB16_Msk
  CAN_F8R2_FB17_Pos* = (17)
  CAN_F8R2_FB17_Msk* = (0x00000001 shl CAN_F8R2_FB17_Pos) ## !< 0x00020000
  CAN_F8R2_FB17* = CAN_F8R2_FB17_Msk
  CAN_F8R2_FB18_Pos* = (18)
  CAN_F8R2_FB18_Msk* = (0x00000001 shl CAN_F8R2_FB18_Pos) ## !< 0x00040000
  CAN_F8R2_FB18* = CAN_F8R2_FB18_Msk
  CAN_F8R2_FB19_Pos* = (19)
  CAN_F8R2_FB19_Msk* = (0x00000001 shl CAN_F8R2_FB19_Pos) ## !< 0x00080000
  CAN_F8R2_FB19* = CAN_F8R2_FB19_Msk
  CAN_F8R2_FB20_Pos* = (20)
  CAN_F8R2_FB20_Msk* = (0x00000001 shl CAN_F8R2_FB20_Pos) ## !< 0x00100000
  CAN_F8R2_FB20* = CAN_F8R2_FB20_Msk
  CAN_F8R2_FB21_Pos* = (21)
  CAN_F8R2_FB21_Msk* = (0x00000001 shl CAN_F8R2_FB21_Pos) ## !< 0x00200000
  CAN_F8R2_FB21* = CAN_F8R2_FB21_Msk
  CAN_F8R2_FB22_Pos* = (22)
  CAN_F8R2_FB22_Msk* = (0x00000001 shl CAN_F8R2_FB22_Pos) ## !< 0x00400000
  CAN_F8R2_FB22* = CAN_F8R2_FB22_Msk
  CAN_F8R2_FB23_Pos* = (23)
  CAN_F8R2_FB23_Msk* = (0x00000001 shl CAN_F8R2_FB23_Pos) ## !< 0x00800000
  CAN_F8R2_FB23* = CAN_F8R2_FB23_Msk
  CAN_F8R2_FB24_Pos* = (24)
  CAN_F8R2_FB24_Msk* = (0x00000001 shl CAN_F8R2_FB24_Pos) ## !< 0x01000000
  CAN_F8R2_FB24* = CAN_F8R2_FB24_Msk
  CAN_F8R2_FB25_Pos* = (25)
  CAN_F8R2_FB25_Msk* = (0x00000001 shl CAN_F8R2_FB25_Pos) ## !< 0x02000000
  CAN_F8R2_FB25* = CAN_F8R2_FB25_Msk
  CAN_F8R2_FB26_Pos* = (26)
  CAN_F8R2_FB26_Msk* = (0x00000001 shl CAN_F8R2_FB26_Pos) ## !< 0x04000000
  CAN_F8R2_FB26* = CAN_F8R2_FB26_Msk
  CAN_F8R2_FB27_Pos* = (27)
  CAN_F8R2_FB27_Msk* = (0x00000001 shl CAN_F8R2_FB27_Pos) ## !< 0x08000000
  CAN_F8R2_FB27* = CAN_F8R2_FB27_Msk
  CAN_F8R2_FB28_Pos* = (28)
  CAN_F8R2_FB28_Msk* = (0x00000001 shl CAN_F8R2_FB28_Pos) ## !< 0x10000000
  CAN_F8R2_FB28* = CAN_F8R2_FB28_Msk
  CAN_F8R2_FB29_Pos* = (29)
  CAN_F8R2_FB29_Msk* = (0x00000001 shl CAN_F8R2_FB29_Pos) ## !< 0x20000000
  CAN_F8R2_FB29* = CAN_F8R2_FB29_Msk
  CAN_F8R2_FB30_Pos* = (30)
  CAN_F8R2_FB30_Msk* = (0x00000001 shl CAN_F8R2_FB30_Pos) ## !< 0x40000000
  CAN_F8R2_FB30* = CAN_F8R2_FB30_Msk
  CAN_F8R2_FB31_Pos* = (31)
  CAN_F8R2_FB31_Msk* = (0x00000001 shl CAN_F8R2_FB31_Pos) ## !< 0x80000000
  CAN_F8R2_FB31* = CAN_F8R2_FB31_Msk

## ******************  Bit definition for CAN_F9R2 register  ******************

const
  CAN_F9R2_FB0_Pos* = (0)
  CAN_F9R2_FB0_Msk* = (0x00000001 shl CAN_F9R2_FB0_Pos) ## !< 0x00000001
  CAN_F9R2_FB0* = CAN_F9R2_FB0_Msk
  CAN_F9R2_FB1_Pos* = (1)
  CAN_F9R2_FB1_Msk* = (0x00000001 shl CAN_F9R2_FB1_Pos) ## !< 0x00000002
  CAN_F9R2_FB1* = CAN_F9R2_FB1_Msk
  CAN_F9R2_FB2_Pos* = (2)
  CAN_F9R2_FB2_Msk* = (0x00000001 shl CAN_F9R2_FB2_Pos) ## !< 0x00000004
  CAN_F9R2_FB2* = CAN_F9R2_FB2_Msk
  CAN_F9R2_FB3_Pos* = (3)
  CAN_F9R2_FB3_Msk* = (0x00000001 shl CAN_F9R2_FB3_Pos) ## !< 0x00000008
  CAN_F9R2_FB3* = CAN_F9R2_FB3_Msk
  CAN_F9R2_FB4_Pos* = (4)
  CAN_F9R2_FB4_Msk* = (0x00000001 shl CAN_F9R2_FB4_Pos) ## !< 0x00000010
  CAN_F9R2_FB4* = CAN_F9R2_FB4_Msk
  CAN_F9R2_FB5_Pos* = (5)
  CAN_F9R2_FB5_Msk* = (0x00000001 shl CAN_F9R2_FB5_Pos) ## !< 0x00000020
  CAN_F9R2_FB5* = CAN_F9R2_FB5_Msk
  CAN_F9R2_FB6_Pos* = (6)
  CAN_F9R2_FB6_Msk* = (0x00000001 shl CAN_F9R2_FB6_Pos) ## !< 0x00000040
  CAN_F9R2_FB6* = CAN_F9R2_FB6_Msk
  CAN_F9R2_FB7_Pos* = (7)
  CAN_F9R2_FB7_Msk* = (0x00000001 shl CAN_F9R2_FB7_Pos) ## !< 0x00000080
  CAN_F9R2_FB7* = CAN_F9R2_FB7_Msk
  CAN_F9R2_FB8_Pos* = (8)
  CAN_F9R2_FB8_Msk* = (0x00000001 shl CAN_F9R2_FB8_Pos) ## !< 0x00000100
  CAN_F9R2_FB8* = CAN_F9R2_FB8_Msk
  CAN_F9R2_FB9_Pos* = (9)
  CAN_F9R2_FB9_Msk* = (0x00000001 shl CAN_F9R2_FB9_Pos) ## !< 0x00000200
  CAN_F9R2_FB9* = CAN_F9R2_FB9_Msk
  CAN_F9R2_FB10_Pos* = (10)
  CAN_F9R2_FB10_Msk* = (0x00000001 shl CAN_F9R2_FB10_Pos) ## !< 0x00000400
  CAN_F9R2_FB10* = CAN_F9R2_FB10_Msk
  CAN_F9R2_FB11_Pos* = (11)
  CAN_F9R2_FB11_Msk* = (0x00000001 shl CAN_F9R2_FB11_Pos) ## !< 0x00000800
  CAN_F9R2_FB11* = CAN_F9R2_FB11_Msk
  CAN_F9R2_FB12_Pos* = (12)
  CAN_F9R2_FB12_Msk* = (0x00000001 shl CAN_F9R2_FB12_Pos) ## !< 0x00001000
  CAN_F9R2_FB12* = CAN_F9R2_FB12_Msk
  CAN_F9R2_FB13_Pos* = (13)
  CAN_F9R2_FB13_Msk* = (0x00000001 shl CAN_F9R2_FB13_Pos) ## !< 0x00002000
  CAN_F9R2_FB13* = CAN_F9R2_FB13_Msk
  CAN_F9R2_FB14_Pos* = (14)
  CAN_F9R2_FB14_Msk* = (0x00000001 shl CAN_F9R2_FB14_Pos) ## !< 0x00004000
  CAN_F9R2_FB14* = CAN_F9R2_FB14_Msk
  CAN_F9R2_FB15_Pos* = (15)
  CAN_F9R2_FB15_Msk* = (0x00000001 shl CAN_F9R2_FB15_Pos) ## !< 0x00008000
  CAN_F9R2_FB15* = CAN_F9R2_FB15_Msk
  CAN_F9R2_FB16_Pos* = (16)
  CAN_F9R2_FB16_Msk* = (0x00000001 shl CAN_F9R2_FB16_Pos) ## !< 0x00010000
  CAN_F9R2_FB16* = CAN_F9R2_FB16_Msk
  CAN_F9R2_FB17_Pos* = (17)
  CAN_F9R2_FB17_Msk* = (0x00000001 shl CAN_F9R2_FB17_Pos) ## !< 0x00020000
  CAN_F9R2_FB17* = CAN_F9R2_FB17_Msk
  CAN_F9R2_FB18_Pos* = (18)
  CAN_F9R2_FB18_Msk* = (0x00000001 shl CAN_F9R2_FB18_Pos) ## !< 0x00040000
  CAN_F9R2_FB18* = CAN_F9R2_FB18_Msk
  CAN_F9R2_FB19_Pos* = (19)
  CAN_F9R2_FB19_Msk* = (0x00000001 shl CAN_F9R2_FB19_Pos) ## !< 0x00080000
  CAN_F9R2_FB19* = CAN_F9R2_FB19_Msk
  CAN_F9R2_FB20_Pos* = (20)
  CAN_F9R2_FB20_Msk* = (0x00000001 shl CAN_F9R2_FB20_Pos) ## !< 0x00100000
  CAN_F9R2_FB20* = CAN_F9R2_FB20_Msk
  CAN_F9R2_FB21_Pos* = (21)
  CAN_F9R2_FB21_Msk* = (0x00000001 shl CAN_F9R2_FB21_Pos) ## !< 0x00200000
  CAN_F9R2_FB21* = CAN_F9R2_FB21_Msk
  CAN_F9R2_FB22_Pos* = (22)
  CAN_F9R2_FB22_Msk* = (0x00000001 shl CAN_F9R2_FB22_Pos) ## !< 0x00400000
  CAN_F9R2_FB22* = CAN_F9R2_FB22_Msk
  CAN_F9R2_FB23_Pos* = (23)
  CAN_F9R2_FB23_Msk* = (0x00000001 shl CAN_F9R2_FB23_Pos) ## !< 0x00800000
  CAN_F9R2_FB23* = CAN_F9R2_FB23_Msk
  CAN_F9R2_FB24_Pos* = (24)
  CAN_F9R2_FB24_Msk* = (0x00000001 shl CAN_F9R2_FB24_Pos) ## !< 0x01000000
  CAN_F9R2_FB24* = CAN_F9R2_FB24_Msk
  CAN_F9R2_FB25_Pos* = (25)
  CAN_F9R2_FB25_Msk* = (0x00000001 shl CAN_F9R2_FB25_Pos) ## !< 0x02000000
  CAN_F9R2_FB25* = CAN_F9R2_FB25_Msk
  CAN_F9R2_FB26_Pos* = (26)
  CAN_F9R2_FB26_Msk* = (0x00000001 shl CAN_F9R2_FB26_Pos) ## !< 0x04000000
  CAN_F9R2_FB26* = CAN_F9R2_FB26_Msk
  CAN_F9R2_FB27_Pos* = (27)
  CAN_F9R2_FB27_Msk* = (0x00000001 shl CAN_F9R2_FB27_Pos) ## !< 0x08000000
  CAN_F9R2_FB27* = CAN_F9R2_FB27_Msk
  CAN_F9R2_FB28_Pos* = (28)
  CAN_F9R2_FB28_Msk* = (0x00000001 shl CAN_F9R2_FB28_Pos) ## !< 0x10000000
  CAN_F9R2_FB28* = CAN_F9R2_FB28_Msk
  CAN_F9R2_FB29_Pos* = (29)
  CAN_F9R2_FB29_Msk* = (0x00000001 shl CAN_F9R2_FB29_Pos) ## !< 0x20000000
  CAN_F9R2_FB29* = CAN_F9R2_FB29_Msk
  CAN_F9R2_FB30_Pos* = (30)
  CAN_F9R2_FB30_Msk* = (0x00000001 shl CAN_F9R2_FB30_Pos) ## !< 0x40000000
  CAN_F9R2_FB30* = CAN_F9R2_FB30_Msk
  CAN_F9R2_FB31_Pos* = (31)
  CAN_F9R2_FB31_Msk* = (0x00000001 shl CAN_F9R2_FB31_Pos) ## !< 0x80000000
  CAN_F9R2_FB31* = CAN_F9R2_FB31_Msk

## ******************  Bit definition for CAN_F10R2 register  *****************

const
  CAN_F10R2_FB0_Pos* = (0)
  CAN_F10R2_FB0_Msk* = (0x00000001 shl CAN_F10R2_FB0_Pos) ## !< 0x00000001
  CAN_F10R2_FB0* = CAN_F10R2_FB0_Msk
  CAN_F10R2_FB1_Pos* = (1)
  CAN_F10R2_FB1_Msk* = (0x00000001 shl CAN_F10R2_FB1_Pos) ## !< 0x00000002
  CAN_F10R2_FB1* = CAN_F10R2_FB1_Msk
  CAN_F10R2_FB2_Pos* = (2)
  CAN_F10R2_FB2_Msk* = (0x00000001 shl CAN_F10R2_FB2_Pos) ## !< 0x00000004
  CAN_F10R2_FB2* = CAN_F10R2_FB2_Msk
  CAN_F10R2_FB3_Pos* = (3)
  CAN_F10R2_FB3_Msk* = (0x00000001 shl CAN_F10R2_FB3_Pos) ## !< 0x00000008
  CAN_F10R2_FB3* = CAN_F10R2_FB3_Msk
  CAN_F10R2_FB4_Pos* = (4)
  CAN_F10R2_FB4_Msk* = (0x00000001 shl CAN_F10R2_FB4_Pos) ## !< 0x00000010
  CAN_F10R2_FB4* = CAN_F10R2_FB4_Msk
  CAN_F10R2_FB5_Pos* = (5)
  CAN_F10R2_FB5_Msk* = (0x00000001 shl CAN_F10R2_FB5_Pos) ## !< 0x00000020
  CAN_F10R2_FB5* = CAN_F10R2_FB5_Msk
  CAN_F10R2_FB6_Pos* = (6)
  CAN_F10R2_FB6_Msk* = (0x00000001 shl CAN_F10R2_FB6_Pos) ## !< 0x00000040
  CAN_F10R2_FB6* = CAN_F10R2_FB6_Msk
  CAN_F10R2_FB7_Pos* = (7)
  CAN_F10R2_FB7_Msk* = (0x00000001 shl CAN_F10R2_FB7_Pos) ## !< 0x00000080
  CAN_F10R2_FB7* = CAN_F10R2_FB7_Msk
  CAN_F10R2_FB8_Pos* = (8)
  CAN_F10R2_FB8_Msk* = (0x00000001 shl CAN_F10R2_FB8_Pos) ## !< 0x00000100
  CAN_F10R2_FB8* = CAN_F10R2_FB8_Msk
  CAN_F10R2_FB9_Pos* = (9)
  CAN_F10R2_FB9_Msk* = (0x00000001 shl CAN_F10R2_FB9_Pos) ## !< 0x00000200
  CAN_F10R2_FB9* = CAN_F10R2_FB9_Msk
  CAN_F10R2_FB10_Pos* = (10)
  CAN_F10R2_FB10_Msk* = (0x00000001 shl CAN_F10R2_FB10_Pos) ## !< 0x00000400
  CAN_F10R2_FB10* = CAN_F10R2_FB10_Msk
  CAN_F10R2_FB11_Pos* = (11)
  CAN_F10R2_FB11_Msk* = (0x00000001 shl CAN_F10R2_FB11_Pos) ## !< 0x00000800
  CAN_F10R2_FB11* = CAN_F10R2_FB11_Msk
  CAN_F10R2_FB12_Pos* = (12)
  CAN_F10R2_FB12_Msk* = (0x00000001 shl CAN_F10R2_FB12_Pos) ## !< 0x00001000
  CAN_F10R2_FB12* = CAN_F10R2_FB12_Msk
  CAN_F10R2_FB13_Pos* = (13)
  CAN_F10R2_FB13_Msk* = (0x00000001 shl CAN_F10R2_FB13_Pos) ## !< 0x00002000
  CAN_F10R2_FB13* = CAN_F10R2_FB13_Msk
  CAN_F10R2_FB14_Pos* = (14)
  CAN_F10R2_FB14_Msk* = (0x00000001 shl CAN_F10R2_FB14_Pos) ## !< 0x00004000
  CAN_F10R2_FB14* = CAN_F10R2_FB14_Msk
  CAN_F10R2_FB15_Pos* = (15)
  CAN_F10R2_FB15_Msk* = (0x00000001 shl CAN_F10R2_FB15_Pos) ## !< 0x00008000
  CAN_F10R2_FB15* = CAN_F10R2_FB15_Msk
  CAN_F10R2_FB16_Pos* = (16)
  CAN_F10R2_FB16_Msk* = (0x00000001 shl CAN_F10R2_FB16_Pos) ## !< 0x00010000
  CAN_F10R2_FB16* = CAN_F10R2_FB16_Msk
  CAN_F10R2_FB17_Pos* = (17)
  CAN_F10R2_FB17_Msk* = (0x00000001 shl CAN_F10R2_FB17_Pos) ## !< 0x00020000
  CAN_F10R2_FB17* = CAN_F10R2_FB17_Msk
  CAN_F10R2_FB18_Pos* = (18)
  CAN_F10R2_FB18_Msk* = (0x00000001 shl CAN_F10R2_FB18_Pos) ## !< 0x00040000
  CAN_F10R2_FB18* = CAN_F10R2_FB18_Msk
  CAN_F10R2_FB19_Pos* = (19)
  CAN_F10R2_FB19_Msk* = (0x00000001 shl CAN_F10R2_FB19_Pos) ## !< 0x00080000
  CAN_F10R2_FB19* = CAN_F10R2_FB19_Msk
  CAN_F10R2_FB20_Pos* = (20)
  CAN_F10R2_FB20_Msk* = (0x00000001 shl CAN_F10R2_FB20_Pos) ## !< 0x00100000
  CAN_F10R2_FB20* = CAN_F10R2_FB20_Msk
  CAN_F10R2_FB21_Pos* = (21)
  CAN_F10R2_FB21_Msk* = (0x00000001 shl CAN_F10R2_FB21_Pos) ## !< 0x00200000
  CAN_F10R2_FB21* = CAN_F10R2_FB21_Msk
  CAN_F10R2_FB22_Pos* = (22)
  CAN_F10R2_FB22_Msk* = (0x00000001 shl CAN_F10R2_FB22_Pos) ## !< 0x00400000
  CAN_F10R2_FB22* = CAN_F10R2_FB22_Msk
  CAN_F10R2_FB23_Pos* = (23)
  CAN_F10R2_FB23_Msk* = (0x00000001 shl CAN_F10R2_FB23_Pos) ## !< 0x00800000
  CAN_F10R2_FB23* = CAN_F10R2_FB23_Msk
  CAN_F10R2_FB24_Pos* = (24)
  CAN_F10R2_FB24_Msk* = (0x00000001 shl CAN_F10R2_FB24_Pos) ## !< 0x01000000
  CAN_F10R2_FB24* = CAN_F10R2_FB24_Msk
  CAN_F10R2_FB25_Pos* = (25)
  CAN_F10R2_FB25_Msk* = (0x00000001 shl CAN_F10R2_FB25_Pos) ## !< 0x02000000
  CAN_F10R2_FB25* = CAN_F10R2_FB25_Msk
  CAN_F10R2_FB26_Pos* = (26)
  CAN_F10R2_FB26_Msk* = (0x00000001 shl CAN_F10R2_FB26_Pos) ## !< 0x04000000
  CAN_F10R2_FB26* = CAN_F10R2_FB26_Msk
  CAN_F10R2_FB27_Pos* = (27)
  CAN_F10R2_FB27_Msk* = (0x00000001 shl CAN_F10R2_FB27_Pos) ## !< 0x08000000
  CAN_F10R2_FB27* = CAN_F10R2_FB27_Msk
  CAN_F10R2_FB28_Pos* = (28)
  CAN_F10R2_FB28_Msk* = (0x00000001 shl CAN_F10R2_FB28_Pos) ## !< 0x10000000
  CAN_F10R2_FB28* = CAN_F10R2_FB28_Msk
  CAN_F10R2_FB29_Pos* = (29)
  CAN_F10R2_FB29_Msk* = (0x00000001 shl CAN_F10R2_FB29_Pos) ## !< 0x20000000
  CAN_F10R2_FB29* = CAN_F10R2_FB29_Msk
  CAN_F10R2_FB30_Pos* = (30)
  CAN_F10R2_FB30_Msk* = (0x00000001 shl CAN_F10R2_FB30_Pos) ## !< 0x40000000
  CAN_F10R2_FB30* = CAN_F10R2_FB30_Msk
  CAN_F10R2_FB31_Pos* = (31)
  CAN_F10R2_FB31_Msk* = (0x00000001 shl CAN_F10R2_FB31_Pos) ## !< 0x80000000
  CAN_F10R2_FB31* = CAN_F10R2_FB31_Msk

## ******************  Bit definition for CAN_F11R2 register  *****************

const
  CAN_F11R2_FB0_Pos* = (0)
  CAN_F11R2_FB0_Msk* = (0x00000001 shl CAN_F11R2_FB0_Pos) ## !< 0x00000001
  CAN_F11R2_FB0* = CAN_F11R2_FB0_Msk
  CAN_F11R2_FB1_Pos* = (1)
  CAN_F11R2_FB1_Msk* = (0x00000001 shl CAN_F11R2_FB1_Pos) ## !< 0x00000002
  CAN_F11R2_FB1* = CAN_F11R2_FB1_Msk
  CAN_F11R2_FB2_Pos* = (2)
  CAN_F11R2_FB2_Msk* = (0x00000001 shl CAN_F11R2_FB2_Pos) ## !< 0x00000004
  CAN_F11R2_FB2* = CAN_F11R2_FB2_Msk
  CAN_F11R2_FB3_Pos* = (3)
  CAN_F11R2_FB3_Msk* = (0x00000001 shl CAN_F11R2_FB3_Pos) ## !< 0x00000008
  CAN_F11R2_FB3* = CAN_F11R2_FB3_Msk
  CAN_F11R2_FB4_Pos* = (4)
  CAN_F11R2_FB4_Msk* = (0x00000001 shl CAN_F11R2_FB4_Pos) ## !< 0x00000010
  CAN_F11R2_FB4* = CAN_F11R2_FB4_Msk
  CAN_F11R2_FB5_Pos* = (5)
  CAN_F11R2_FB5_Msk* = (0x00000001 shl CAN_F11R2_FB5_Pos) ## !< 0x00000020
  CAN_F11R2_FB5* = CAN_F11R2_FB5_Msk
  CAN_F11R2_FB6_Pos* = (6)
  CAN_F11R2_FB6_Msk* = (0x00000001 shl CAN_F11R2_FB6_Pos) ## !< 0x00000040
  CAN_F11R2_FB6* = CAN_F11R2_FB6_Msk
  CAN_F11R2_FB7_Pos* = (7)
  CAN_F11R2_FB7_Msk* = (0x00000001 shl CAN_F11R2_FB7_Pos) ## !< 0x00000080
  CAN_F11R2_FB7* = CAN_F11R2_FB7_Msk
  CAN_F11R2_FB8_Pos* = (8)
  CAN_F11R2_FB8_Msk* = (0x00000001 shl CAN_F11R2_FB8_Pos) ## !< 0x00000100
  CAN_F11R2_FB8* = CAN_F11R2_FB8_Msk
  CAN_F11R2_FB9_Pos* = (9)
  CAN_F11R2_FB9_Msk* = (0x00000001 shl CAN_F11R2_FB9_Pos) ## !< 0x00000200
  CAN_F11R2_FB9* = CAN_F11R2_FB9_Msk
  CAN_F11R2_FB10_Pos* = (10)
  CAN_F11R2_FB10_Msk* = (0x00000001 shl CAN_F11R2_FB10_Pos) ## !< 0x00000400
  CAN_F11R2_FB10* = CAN_F11R2_FB10_Msk
  CAN_F11R2_FB11_Pos* = (11)
  CAN_F11R2_FB11_Msk* = (0x00000001 shl CAN_F11R2_FB11_Pos) ## !< 0x00000800
  CAN_F11R2_FB11* = CAN_F11R2_FB11_Msk
  CAN_F11R2_FB12_Pos* = (12)
  CAN_F11R2_FB12_Msk* = (0x00000001 shl CAN_F11R2_FB12_Pos) ## !< 0x00001000
  CAN_F11R2_FB12* = CAN_F11R2_FB12_Msk
  CAN_F11R2_FB13_Pos* = (13)
  CAN_F11R2_FB13_Msk* = (0x00000001 shl CAN_F11R2_FB13_Pos) ## !< 0x00002000
  CAN_F11R2_FB13* = CAN_F11R2_FB13_Msk
  CAN_F11R2_FB14_Pos* = (14)
  CAN_F11R2_FB14_Msk* = (0x00000001 shl CAN_F11R2_FB14_Pos) ## !< 0x00004000
  CAN_F11R2_FB14* = CAN_F11R2_FB14_Msk
  CAN_F11R2_FB15_Pos* = (15)
  CAN_F11R2_FB15_Msk* = (0x00000001 shl CAN_F11R2_FB15_Pos) ## !< 0x00008000
  CAN_F11R2_FB15* = CAN_F11R2_FB15_Msk
  CAN_F11R2_FB16_Pos* = (16)
  CAN_F11R2_FB16_Msk* = (0x00000001 shl CAN_F11R2_FB16_Pos) ## !< 0x00010000
  CAN_F11R2_FB16* = CAN_F11R2_FB16_Msk
  CAN_F11R2_FB17_Pos* = (17)
  CAN_F11R2_FB17_Msk* = (0x00000001 shl CAN_F11R2_FB17_Pos) ## !< 0x00020000
  CAN_F11R2_FB17* = CAN_F11R2_FB17_Msk
  CAN_F11R2_FB18_Pos* = (18)
  CAN_F11R2_FB18_Msk* = (0x00000001 shl CAN_F11R2_FB18_Pos) ## !< 0x00040000
  CAN_F11R2_FB18* = CAN_F11R2_FB18_Msk
  CAN_F11R2_FB19_Pos* = (19)
  CAN_F11R2_FB19_Msk* = (0x00000001 shl CAN_F11R2_FB19_Pos) ## !< 0x00080000
  CAN_F11R2_FB19* = CAN_F11R2_FB19_Msk
  CAN_F11R2_FB20_Pos* = (20)
  CAN_F11R2_FB20_Msk* = (0x00000001 shl CAN_F11R2_FB20_Pos) ## !< 0x00100000
  CAN_F11R2_FB20* = CAN_F11R2_FB20_Msk
  CAN_F11R2_FB21_Pos* = (21)
  CAN_F11R2_FB21_Msk* = (0x00000001 shl CAN_F11R2_FB21_Pos) ## !< 0x00200000
  CAN_F11R2_FB21* = CAN_F11R2_FB21_Msk
  CAN_F11R2_FB22_Pos* = (22)
  CAN_F11R2_FB22_Msk* = (0x00000001 shl CAN_F11R2_FB22_Pos) ## !< 0x00400000
  CAN_F11R2_FB22* = CAN_F11R2_FB22_Msk
  CAN_F11R2_FB23_Pos* = (23)
  CAN_F11R2_FB23_Msk* = (0x00000001 shl CAN_F11R2_FB23_Pos) ## !< 0x00800000
  CAN_F11R2_FB23* = CAN_F11R2_FB23_Msk
  CAN_F11R2_FB24_Pos* = (24)
  CAN_F11R2_FB24_Msk* = (0x00000001 shl CAN_F11R2_FB24_Pos) ## !< 0x01000000
  CAN_F11R2_FB24* = CAN_F11R2_FB24_Msk
  CAN_F11R2_FB25_Pos* = (25)
  CAN_F11R2_FB25_Msk* = (0x00000001 shl CAN_F11R2_FB25_Pos) ## !< 0x02000000
  CAN_F11R2_FB25* = CAN_F11R2_FB25_Msk
  CAN_F11R2_FB26_Pos* = (26)
  CAN_F11R2_FB26_Msk* = (0x00000001 shl CAN_F11R2_FB26_Pos) ## !< 0x04000000
  CAN_F11R2_FB26* = CAN_F11R2_FB26_Msk
  CAN_F11R2_FB27_Pos* = (27)
  CAN_F11R2_FB27_Msk* = (0x00000001 shl CAN_F11R2_FB27_Pos) ## !< 0x08000000
  CAN_F11R2_FB27* = CAN_F11R2_FB27_Msk
  CAN_F11R2_FB28_Pos* = (28)
  CAN_F11R2_FB28_Msk* = (0x00000001 shl CAN_F11R2_FB28_Pos) ## !< 0x10000000
  CAN_F11R2_FB28* = CAN_F11R2_FB28_Msk
  CAN_F11R2_FB29_Pos* = (29)
  CAN_F11R2_FB29_Msk* = (0x00000001 shl CAN_F11R2_FB29_Pos) ## !< 0x20000000
  CAN_F11R2_FB29* = CAN_F11R2_FB29_Msk
  CAN_F11R2_FB30_Pos* = (30)
  CAN_F11R2_FB30_Msk* = (0x00000001 shl CAN_F11R2_FB30_Pos) ## !< 0x40000000
  CAN_F11R2_FB30* = CAN_F11R2_FB30_Msk
  CAN_F11R2_FB31_Pos* = (31)
  CAN_F11R2_FB31_Msk* = (0x00000001 shl CAN_F11R2_FB31_Pos) ## !< 0x80000000
  CAN_F11R2_FB31* = CAN_F11R2_FB31_Msk

## ******************  Bit definition for CAN_F12R2 register  *****************

const
  CAN_F12R2_FB0_Pos* = (0)
  CAN_F12R2_FB0_Msk* = (0x00000001 shl CAN_F12R2_FB0_Pos) ## !< 0x00000001
  CAN_F12R2_FB0* = CAN_F12R2_FB0_Msk
  CAN_F12R2_FB1_Pos* = (1)
  CAN_F12R2_FB1_Msk* = (0x00000001 shl CAN_F12R2_FB1_Pos) ## !< 0x00000002
  CAN_F12R2_FB1* = CAN_F12R2_FB1_Msk
  CAN_F12R2_FB2_Pos* = (2)
  CAN_F12R2_FB2_Msk* = (0x00000001 shl CAN_F12R2_FB2_Pos) ## !< 0x00000004
  CAN_F12R2_FB2* = CAN_F12R2_FB2_Msk
  CAN_F12R2_FB3_Pos* = (3)
  CAN_F12R2_FB3_Msk* = (0x00000001 shl CAN_F12R2_FB3_Pos) ## !< 0x00000008
  CAN_F12R2_FB3* = CAN_F12R2_FB3_Msk
  CAN_F12R2_FB4_Pos* = (4)
  CAN_F12R2_FB4_Msk* = (0x00000001 shl CAN_F12R2_FB4_Pos) ## !< 0x00000010
  CAN_F12R2_FB4* = CAN_F12R2_FB4_Msk
  CAN_F12R2_FB5_Pos* = (5)
  CAN_F12R2_FB5_Msk* = (0x00000001 shl CAN_F12R2_FB5_Pos) ## !< 0x00000020
  CAN_F12R2_FB5* = CAN_F12R2_FB5_Msk
  CAN_F12R2_FB6_Pos* = (6)
  CAN_F12R2_FB6_Msk* = (0x00000001 shl CAN_F12R2_FB6_Pos) ## !< 0x00000040
  CAN_F12R2_FB6* = CAN_F12R2_FB6_Msk
  CAN_F12R2_FB7_Pos* = (7)
  CAN_F12R2_FB7_Msk* = (0x00000001 shl CAN_F12R2_FB7_Pos) ## !< 0x00000080
  CAN_F12R2_FB7* = CAN_F12R2_FB7_Msk
  CAN_F12R2_FB8_Pos* = (8)
  CAN_F12R2_FB8_Msk* = (0x00000001 shl CAN_F12R2_FB8_Pos) ## !< 0x00000100
  CAN_F12R2_FB8* = CAN_F12R2_FB8_Msk
  CAN_F12R2_FB9_Pos* = (9)
  CAN_F12R2_FB9_Msk* = (0x00000001 shl CAN_F12R2_FB9_Pos) ## !< 0x00000200
  CAN_F12R2_FB9* = CAN_F12R2_FB9_Msk
  CAN_F12R2_FB10_Pos* = (10)
  CAN_F12R2_FB10_Msk* = (0x00000001 shl CAN_F12R2_FB10_Pos) ## !< 0x00000400
  CAN_F12R2_FB10* = CAN_F12R2_FB10_Msk
  CAN_F12R2_FB11_Pos* = (11)
  CAN_F12R2_FB11_Msk* = (0x00000001 shl CAN_F12R2_FB11_Pos) ## !< 0x00000800
  CAN_F12R2_FB11* = CAN_F12R2_FB11_Msk
  CAN_F12R2_FB12_Pos* = (12)
  CAN_F12R2_FB12_Msk* = (0x00000001 shl CAN_F12R2_FB12_Pos) ## !< 0x00001000
  CAN_F12R2_FB12* = CAN_F12R2_FB12_Msk
  CAN_F12R2_FB13_Pos* = (13)
  CAN_F12R2_FB13_Msk* = (0x00000001 shl CAN_F12R2_FB13_Pos) ## !< 0x00002000
  CAN_F12R2_FB13* = CAN_F12R2_FB13_Msk
  CAN_F12R2_FB14_Pos* = (14)
  CAN_F12R2_FB14_Msk* = (0x00000001 shl CAN_F12R2_FB14_Pos) ## !< 0x00004000
  CAN_F12R2_FB14* = CAN_F12R2_FB14_Msk
  CAN_F12R2_FB15_Pos* = (15)
  CAN_F12R2_FB15_Msk* = (0x00000001 shl CAN_F12R2_FB15_Pos) ## !< 0x00008000
  CAN_F12R2_FB15* = CAN_F12R2_FB15_Msk
  CAN_F12R2_FB16_Pos* = (16)
  CAN_F12R2_FB16_Msk* = (0x00000001 shl CAN_F12R2_FB16_Pos) ## !< 0x00010000
  CAN_F12R2_FB16* = CAN_F12R2_FB16_Msk
  CAN_F12R2_FB17_Pos* = (17)
  CAN_F12R2_FB17_Msk* = (0x00000001 shl CAN_F12R2_FB17_Pos) ## !< 0x00020000
  CAN_F12R2_FB17* = CAN_F12R2_FB17_Msk
  CAN_F12R2_FB18_Pos* = (18)
  CAN_F12R2_FB18_Msk* = (0x00000001 shl CAN_F12R2_FB18_Pos) ## !< 0x00040000
  CAN_F12R2_FB18* = CAN_F12R2_FB18_Msk
  CAN_F12R2_FB19_Pos* = (19)
  CAN_F12R2_FB19_Msk* = (0x00000001 shl CAN_F12R2_FB19_Pos) ## !< 0x00080000
  CAN_F12R2_FB19* = CAN_F12R2_FB19_Msk
  CAN_F12R2_FB20_Pos* = (20)
  CAN_F12R2_FB20_Msk* = (0x00000001 shl CAN_F12R2_FB20_Pos) ## !< 0x00100000
  CAN_F12R2_FB20* = CAN_F12R2_FB20_Msk
  CAN_F12R2_FB21_Pos* = (21)
  CAN_F12R2_FB21_Msk* = (0x00000001 shl CAN_F12R2_FB21_Pos) ## !< 0x00200000
  CAN_F12R2_FB21* = CAN_F12R2_FB21_Msk
  CAN_F12R2_FB22_Pos* = (22)
  CAN_F12R2_FB22_Msk* = (0x00000001 shl CAN_F12R2_FB22_Pos) ## !< 0x00400000
  CAN_F12R2_FB22* = CAN_F12R2_FB22_Msk
  CAN_F12R2_FB23_Pos* = (23)
  CAN_F12R2_FB23_Msk* = (0x00000001 shl CAN_F12R2_FB23_Pos) ## !< 0x00800000
  CAN_F12R2_FB23* = CAN_F12R2_FB23_Msk
  CAN_F12R2_FB24_Pos* = (24)
  CAN_F12R2_FB24_Msk* = (0x00000001 shl CAN_F12R2_FB24_Pos) ## !< 0x01000000
  CAN_F12R2_FB24* = CAN_F12R2_FB24_Msk
  CAN_F12R2_FB25_Pos* = (25)
  CAN_F12R2_FB25_Msk* = (0x00000001 shl CAN_F12R2_FB25_Pos) ## !< 0x02000000
  CAN_F12R2_FB25* = CAN_F12R2_FB25_Msk
  CAN_F12R2_FB26_Pos* = (26)
  CAN_F12R2_FB26_Msk* = (0x00000001 shl CAN_F12R2_FB26_Pos) ## !< 0x04000000
  CAN_F12R2_FB26* = CAN_F12R2_FB26_Msk
  CAN_F12R2_FB27_Pos* = (27)
  CAN_F12R2_FB27_Msk* = (0x00000001 shl CAN_F12R2_FB27_Pos) ## !< 0x08000000
  CAN_F12R2_FB27* = CAN_F12R2_FB27_Msk
  CAN_F12R2_FB28_Pos* = (28)
  CAN_F12R2_FB28_Msk* = (0x00000001 shl CAN_F12R2_FB28_Pos) ## !< 0x10000000
  CAN_F12R2_FB28* = CAN_F12R2_FB28_Msk
  CAN_F12R2_FB29_Pos* = (29)
  CAN_F12R2_FB29_Msk* = (0x00000001 shl CAN_F12R2_FB29_Pos) ## !< 0x20000000
  CAN_F12R2_FB29* = CAN_F12R2_FB29_Msk
  CAN_F12R2_FB30_Pos* = (30)
  CAN_F12R2_FB30_Msk* = (0x00000001 shl CAN_F12R2_FB30_Pos) ## !< 0x40000000
  CAN_F12R2_FB30* = CAN_F12R2_FB30_Msk
  CAN_F12R2_FB31_Pos* = (31)
  CAN_F12R2_FB31_Msk* = (0x00000001 shl CAN_F12R2_FB31_Pos) ## !< 0x80000000
  CAN_F12R2_FB31* = CAN_F12R2_FB31_Msk

## ******************  Bit definition for CAN_F13R2 register  *****************

const
  CAN_F13R2_FB0_Pos* = (0)
  CAN_F13R2_FB0_Msk* = (0x00000001 shl CAN_F13R2_FB0_Pos) ## !< 0x00000001
  CAN_F13R2_FB0* = CAN_F13R2_FB0_Msk
  CAN_F13R2_FB1_Pos* = (1)
  CAN_F13R2_FB1_Msk* = (0x00000001 shl CAN_F13R2_FB1_Pos) ## !< 0x00000002
  CAN_F13R2_FB1* = CAN_F13R2_FB1_Msk
  CAN_F13R2_FB2_Pos* = (2)
  CAN_F13R2_FB2_Msk* = (0x00000001 shl CAN_F13R2_FB2_Pos) ## !< 0x00000004
  CAN_F13R2_FB2* = CAN_F13R2_FB2_Msk
  CAN_F13R2_FB3_Pos* = (3)
  CAN_F13R2_FB3_Msk* = (0x00000001 shl CAN_F13R2_FB3_Pos) ## !< 0x00000008
  CAN_F13R2_FB3* = CAN_F13R2_FB3_Msk
  CAN_F13R2_FB4_Pos* = (4)
  CAN_F13R2_FB4_Msk* = (0x00000001 shl CAN_F13R2_FB4_Pos) ## !< 0x00000010
  CAN_F13R2_FB4* = CAN_F13R2_FB4_Msk
  CAN_F13R2_FB5_Pos* = (5)
  CAN_F13R2_FB5_Msk* = (0x00000001 shl CAN_F13R2_FB5_Pos) ## !< 0x00000020
  CAN_F13R2_FB5* = CAN_F13R2_FB5_Msk
  CAN_F13R2_FB6_Pos* = (6)
  CAN_F13R2_FB6_Msk* = (0x00000001 shl CAN_F13R2_FB6_Pos) ## !< 0x00000040
  CAN_F13R2_FB6* = CAN_F13R2_FB6_Msk
  CAN_F13R2_FB7_Pos* = (7)
  CAN_F13R2_FB7_Msk* = (0x00000001 shl CAN_F13R2_FB7_Pos) ## !< 0x00000080
  CAN_F13R2_FB7* = CAN_F13R2_FB7_Msk
  CAN_F13R2_FB8_Pos* = (8)
  CAN_F13R2_FB8_Msk* = (0x00000001 shl CAN_F13R2_FB8_Pos) ## !< 0x00000100
  CAN_F13R2_FB8* = CAN_F13R2_FB8_Msk
  CAN_F13R2_FB9_Pos* = (9)
  CAN_F13R2_FB9_Msk* = (0x00000001 shl CAN_F13R2_FB9_Pos) ## !< 0x00000200
  CAN_F13R2_FB9* = CAN_F13R2_FB9_Msk
  CAN_F13R2_FB10_Pos* = (10)
  CAN_F13R2_FB10_Msk* = (0x00000001 shl CAN_F13R2_FB10_Pos) ## !< 0x00000400
  CAN_F13R2_FB10* = CAN_F13R2_FB10_Msk
  CAN_F13R2_FB11_Pos* = (11)
  CAN_F13R2_FB11_Msk* = (0x00000001 shl CAN_F13R2_FB11_Pos) ## !< 0x00000800
  CAN_F13R2_FB11* = CAN_F13R2_FB11_Msk
  CAN_F13R2_FB12_Pos* = (12)
  CAN_F13R2_FB12_Msk* = (0x00000001 shl CAN_F13R2_FB12_Pos) ## !< 0x00001000
  CAN_F13R2_FB12* = CAN_F13R2_FB12_Msk
  CAN_F13R2_FB13_Pos* = (13)
  CAN_F13R2_FB13_Msk* = (0x00000001 shl CAN_F13R2_FB13_Pos) ## !< 0x00002000
  CAN_F13R2_FB13* = CAN_F13R2_FB13_Msk
  CAN_F13R2_FB14_Pos* = (14)
  CAN_F13R2_FB14_Msk* = (0x00000001 shl CAN_F13R2_FB14_Pos) ## !< 0x00004000
  CAN_F13R2_FB14* = CAN_F13R2_FB14_Msk
  CAN_F13R2_FB15_Pos* = (15)
  CAN_F13R2_FB15_Msk* = (0x00000001 shl CAN_F13R2_FB15_Pos) ## !< 0x00008000
  CAN_F13R2_FB15* = CAN_F13R2_FB15_Msk
  CAN_F13R2_FB16_Pos* = (16)
  CAN_F13R2_FB16_Msk* = (0x00000001 shl CAN_F13R2_FB16_Pos) ## !< 0x00010000
  CAN_F13R2_FB16* = CAN_F13R2_FB16_Msk
  CAN_F13R2_FB17_Pos* = (17)
  CAN_F13R2_FB17_Msk* = (0x00000001 shl CAN_F13R2_FB17_Pos) ## !< 0x00020000
  CAN_F13R2_FB17* = CAN_F13R2_FB17_Msk
  CAN_F13R2_FB18_Pos* = (18)
  CAN_F13R2_FB18_Msk* = (0x00000001 shl CAN_F13R2_FB18_Pos) ## !< 0x00040000
  CAN_F13R2_FB18* = CAN_F13R2_FB18_Msk
  CAN_F13R2_FB19_Pos* = (19)
  CAN_F13R2_FB19_Msk* = (0x00000001 shl CAN_F13R2_FB19_Pos) ## !< 0x00080000
  CAN_F13R2_FB19* = CAN_F13R2_FB19_Msk
  CAN_F13R2_FB20_Pos* = (20)
  CAN_F13R2_FB20_Msk* = (0x00000001 shl CAN_F13R2_FB20_Pos) ## !< 0x00100000
  CAN_F13R2_FB20* = CAN_F13R2_FB20_Msk
  CAN_F13R2_FB21_Pos* = (21)
  CAN_F13R2_FB21_Msk* = (0x00000001 shl CAN_F13R2_FB21_Pos) ## !< 0x00200000
  CAN_F13R2_FB21* = CAN_F13R2_FB21_Msk
  CAN_F13R2_FB22_Pos* = (22)
  CAN_F13R2_FB22_Msk* = (0x00000001 shl CAN_F13R2_FB22_Pos) ## !< 0x00400000
  CAN_F13R2_FB22* = CAN_F13R2_FB22_Msk
  CAN_F13R2_FB23_Pos* = (23)
  CAN_F13R2_FB23_Msk* = (0x00000001 shl CAN_F13R2_FB23_Pos) ## !< 0x00800000
  CAN_F13R2_FB23* = CAN_F13R2_FB23_Msk
  CAN_F13R2_FB24_Pos* = (24)
  CAN_F13R2_FB24_Msk* = (0x00000001 shl CAN_F13R2_FB24_Pos) ## !< 0x01000000
  CAN_F13R2_FB24* = CAN_F13R2_FB24_Msk
  CAN_F13R2_FB25_Pos* = (25)
  CAN_F13R2_FB25_Msk* = (0x00000001 shl CAN_F13R2_FB25_Pos) ## !< 0x02000000
  CAN_F13R2_FB25* = CAN_F13R2_FB25_Msk
  CAN_F13R2_FB26_Pos* = (26)
  CAN_F13R2_FB26_Msk* = (0x00000001 shl CAN_F13R2_FB26_Pos) ## !< 0x04000000
  CAN_F13R2_FB26* = CAN_F13R2_FB26_Msk
  CAN_F13R2_FB27_Pos* = (27)
  CAN_F13R2_FB27_Msk* = (0x00000001 shl CAN_F13R2_FB27_Pos) ## !< 0x08000000
  CAN_F13R2_FB27* = CAN_F13R2_FB27_Msk
  CAN_F13R2_FB28_Pos* = (28)
  CAN_F13R2_FB28_Msk* = (0x00000001 shl CAN_F13R2_FB28_Pos) ## !< 0x10000000
  CAN_F13R2_FB28* = CAN_F13R2_FB28_Msk
  CAN_F13R2_FB29_Pos* = (29)
  CAN_F13R2_FB29_Msk* = (0x00000001 shl CAN_F13R2_FB29_Pos) ## !< 0x20000000
  CAN_F13R2_FB29* = CAN_F13R2_FB29_Msk
  CAN_F13R2_FB30_Pos* = (30)
  CAN_F13R2_FB30_Msk* = (0x00000001 shl CAN_F13R2_FB30_Pos) ## !< 0x40000000
  CAN_F13R2_FB30* = CAN_F13R2_FB30_Msk
  CAN_F13R2_FB31_Pos* = (31)
  CAN_F13R2_FB31_Msk* = (0x00000001 shl CAN_F13R2_FB31_Pos) ## !< 0x80000000
  CAN_F13R2_FB31* = CAN_F13R2_FB31_Msk

## ****************************************************************************
##
##                      CRC calculation unit (CRC)
##
## ****************************************************************************
## ******************  Bit definition for CRC_DR register  ********************

const
  CRC_DR_DR_Pos* = (0)
  CRC_DR_DR_Msk* = (0xFFFFFFFF shl CRC_DR_DR_Pos) ## !< 0xFFFFFFFF
  CRC_DR_DR* = CRC_DR_DR_Msk

## ******************  Bit definition for CRC_IDR register  *******************

const
  CRC_IDR_IDR* = (cast[uint8](0x000000FF'u8)) ## !< General-purpose 8-bit data register bits

## *******************  Bit definition for CRC_CR register  *******************

const
  CRC_CR_RESET_Pos* = (0)
  CRC_CR_RESET_Msk* = (0x00000001 shl CRC_CR_RESET_Pos) ## !< 0x00000001
  CRC_CR_RESET* = CRC_CR_RESET_Msk
  CRC_CR_POLYSIZE_Pos* = (3)
  CRC_CR_POLYSIZE_Msk* = (0x00000003 shl CRC_CR_POLYSIZE_Pos) ## !< 0x00000018
  CRC_CR_POLYSIZE* = CRC_CR_POLYSIZE_Msk
  CRC_CR_POLYSIZE_0* = (0x00000001 shl CRC_CR_POLYSIZE_Pos) ## !< 0x00000008
  CRC_CR_POLYSIZE_1* = (0x00000002 shl CRC_CR_POLYSIZE_Pos) ## !< 0x00000010
  CRC_CR_REV_IN_Pos* = (5)
  CRC_CR_REV_IN_Msk* = (0x00000003 shl CRC_CR_REV_IN_Pos) ## !< 0x00000060
  CRC_CR_REV_IN* = CRC_CR_REV_IN_Msk
  CRC_CR_REV_IN_0* = (0x00000001 shl CRC_CR_REV_IN_Pos) ## !< 0x00000020
  CRC_CR_REV_IN_1* = (0x00000002 shl CRC_CR_REV_IN_Pos) ## !< 0x00000040
  CRC_CR_REV_OUT_Pos* = (7)
  CRC_CR_REV_OUT_Msk* = (0x00000001 shl CRC_CR_REV_OUT_Pos) ## !< 0x00000080
  CRC_CR_REV_OUT* = CRC_CR_REV_OUT_Msk

## ******************  Bit definition for CRC_INIT register  ******************

const
  CRC_INIT_INIT_Pos* = (0)
  CRC_INIT_INIT_Msk* = (0xFFFFFFFF shl CRC_INIT_INIT_Pos) ## !< 0xFFFFFFFF
  CRC_INIT_INIT* = CRC_INIT_INIT_Msk

## ******************  Bit definition for CRC_POL register  *******************

const
  CRC_POL_POL_Pos* = (0)
  CRC_POL_POL_Msk* = (0xFFFFFFFF shl CRC_POL_POL_Pos) ## !< 0xFFFFFFFF
  CRC_POL_POL* = CRC_POL_POL_Msk

## ****************************************************************************
##
##                  Digital to Analog Converter (DAC)
##
## ****************************************************************************
##
##  @brief Specific device feature definitions (not present on all devices in the STM32F3 serie)
##

const
  DAC_CHANNEL2_SUPPORT* = true  ## !< DAC feature available only on specific devices: DAC channel 2 available (may not be available on all DAC instances DACx)

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
##                                  Debug MCU (DBGMCU)
##
## ****************************************************************************
## *******************  Bit definition for DBGMCU_IDCODE register  ************

const
  DBGMCU_IDCODE_DEV_ID_Pos* = (0)
  DBGMCU_IDCODE_DEV_ID_Msk* = (0x00000FFF shl DBGMCU_IDCODE_DEV_ID_Pos) ## !< 0x00000FFF
  DBGMCU_IDCODE_DEV_ID* = DBGMCU_IDCODE_DEV_ID_Msk
  DBGMCU_IDCODE_REV_ID_Pos* = (16)
  DBGMCU_IDCODE_REV_ID_Msk* = (0x0000FFFF shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0xFFFF0000
  DBGMCU_IDCODE_REV_ID* = DBGMCU_IDCODE_REV_ID_Msk

## *******************  Bit definition for DBGMCU_CR register  ****************

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

## *******************  Bit definition for DBGMCU_APB1_FZ register  ***********

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
  DBGMCU_APB1_FZ_DBG_CAN_STOP_Pos* = (25)
  DBGMCU_APB1_FZ_DBG_CAN_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_CAN_STOP_Pos) ## !< 0x02000000
  DBGMCU_APB1_FZ_DBG_CAN_STOP* = DBGMCU_APB1_FZ_DBG_CAN_STOP_Msk

## *******************  Bit definition for DBGMCU_APB2_FZ register  ***********

const
  DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos* = (0)
  DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos) ## !< 0x00000001
  DBGMCU_APB2_FZ_DBG_TIM1_STOP* = DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk
  DBGMCU_APB2_FZ_DBG_TIM8_STOP_Pos* = (1)
  DBGMCU_APB2_FZ_DBG_TIM8_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM8_STOP_Pos) ## !< 0x00000002
  DBGMCU_APB2_FZ_DBG_TIM8_STOP* = DBGMCU_APB2_FZ_DBG_TIM8_STOP_Msk
  DBGMCU_APB2_FZ_DBG_TIM15_STOP_Pos* = (2)
  DBGMCU_APB2_FZ_DBG_TIM15_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM15_STOP_Pos) ## !< 0x00000004
  DBGMCU_APB2_FZ_DBG_TIM15_STOP* = DBGMCU_APB2_FZ_DBG_TIM15_STOP_Msk
  DBGMCU_APB2_FZ_DBG_TIM16_STOP_Pos* = (3)
  DBGMCU_APB2_FZ_DBG_TIM16_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM16_STOP_Pos) ## !< 0x00000008
  DBGMCU_APB2_FZ_DBG_TIM16_STOP* = DBGMCU_APB2_FZ_DBG_TIM16_STOP_Msk
  DBGMCU_APB2_FZ_DBG_TIM17_STOP_Pos* = (4)
  DBGMCU_APB2_FZ_DBG_TIM17_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM17_STOP_Pos) ## !< 0x00000010
  DBGMCU_APB2_FZ_DBG_TIM17_STOP* = DBGMCU_APB2_FZ_DBG_TIM17_STOP_Msk

## ****************************************************************************
##
##                              DMA Controller (DMA)
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

## ******************  Bit definition for DMA_CCR register  *******************

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

## *****************  Bit definition for DMA_CNDTR register  ******************

const
  DMA_CNDTR_NDT_Pos* = (0)
  DMA_CNDTR_NDT_Msk* = (0x0000FFFF shl DMA_CNDTR_NDT_Pos) ## !< 0x0000FFFF
  DMA_CNDTR_NDT* = DMA_CNDTR_NDT_Msk

## *****************  Bit definition for DMA_CPAR register  *******************

const
  DMA_CPAR_PA_Pos* = (0)
  DMA_CPAR_PA_Msk* = (0xFFFFFFFF shl DMA_CPAR_PA_Pos) ## !< 0xFFFFFFFF
  DMA_CPAR_PA* = DMA_CPAR_PA_Msk

## *****************  Bit definition for DMA_CMAR register  *******************

const
  DMA_CMAR_MA_Pos* = (0)
  DMA_CMAR_MA_Msk* = (0xFFFFFFFF shl DMA_CMAR_MA_Pos) ## !< 0xFFFFFFFF
  DMA_CMAR_MA* = DMA_CMAR_MA_Msk

## ****************************************************************************
##
##                     External Interrupt/Event Controller (EXTI)
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
  EXTI_IMR_MR24_Pos* = (24)
  EXTI_IMR_MR24_Msk* = (0x00000001 shl EXTI_IMR_MR24_Pos) ## !< 0x01000000
  EXTI_IMR_MR24* = EXTI_IMR_MR24_Msk
  EXTI_IMR_MR25_Pos* = (25)
  EXTI_IMR_MR25_Msk* = (0x00000001 shl EXTI_IMR_MR25_Pos) ## !< 0x02000000
  EXTI_IMR_MR25* = EXTI_IMR_MR25_Msk
  EXTI_IMR_MR26_Pos* = (26)
  EXTI_IMR_MR26_Msk* = (0x00000001 shl EXTI_IMR_MR26_Pos) ## !< 0x04000000
  EXTI_IMR_MR26* = EXTI_IMR_MR26_Msk
  EXTI_IMR_MR28_Pos* = (28)
  EXTI_IMR_MR28_Msk* = (0x00000001 shl EXTI_IMR_MR28_Pos) ## !< 0x10000000
  EXTI_IMR_MR28* = EXTI_IMR_MR28_Msk
  EXTI_IMR_MR29_Pos* = (29)
  EXTI_IMR_MR29_Msk* = (0x00000001 shl EXTI_IMR_MR29_Pos) ## !< 0x20000000
  EXTI_IMR_MR29* = EXTI_IMR_MR29_Msk
  EXTI_IMR_MR30_Pos* = (30)
  EXTI_IMR_MR30_Msk* = (0x00000001 shl EXTI_IMR_MR30_Pos) ## !< 0x40000000
  EXTI_IMR_MR30* = EXTI_IMR_MR30_Msk
  EXTI_IMR_MR31_Pos* = (31)
  EXTI_IMR_MR31_Msk* = (0x00000001 shl EXTI_IMR_MR31_Pos) ## !< 0x80000000
  EXTI_IMR_MR31* = EXTI_IMR_MR31_Msk

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
  EXTI_IMR_IM23* = EXTI_IMR_MR23
  EXTI_IMR_IM24* = EXTI_IMR_MR24
  EXTI_IMR_IM25* = EXTI_IMR_MR25
  EXTI_IMR_IM26* = EXTI_IMR_MR26

when defined(EXTI_IMR_MR27):
  const
    EXTI_IMR_IM27* = EXTI_IMR_MR27
const
  EXTI_IMR_IM28* = EXTI_IMR_MR28
  EXTI_IMR_IM29* = EXTI_IMR_MR29
  EXTI_IMR_IM30* = EXTI_IMR_MR30
  EXTI_IMR_IM31* = EXTI_IMR_MR31
  EXTI_IMR_IM_Pos* = (0)
  EXTI_IMR_IM_Msk* = (0xFFFFFFFF shl EXTI_IMR_IM_Pos) ## !< 0xFFFFFFFF
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
  EXTI_EMR_MR24_Pos* = (24)
  EXTI_EMR_MR24_Msk* = (0x00000001 shl EXTI_EMR_MR24_Pos) ## !< 0x01000000
  EXTI_EMR_MR24* = EXTI_EMR_MR24_Msk
  EXTI_EMR_MR25_Pos* = (25)
  EXTI_EMR_MR25_Msk* = (0x00000001 shl EXTI_EMR_MR25_Pos) ## !< 0x02000000
  EXTI_EMR_MR25* = EXTI_EMR_MR25_Msk
  EXTI_EMR_MR26_Pos* = (26)
  EXTI_EMR_MR26_Msk* = (0x00000001 shl EXTI_EMR_MR26_Pos) ## !< 0x04000000
  EXTI_EMR_MR26* = EXTI_EMR_MR26_Msk
  EXTI_EMR_MR28_Pos* = (28)
  EXTI_EMR_MR28_Msk* = (0x00000001 shl EXTI_EMR_MR28_Pos) ## !< 0x10000000
  EXTI_EMR_MR28* = EXTI_EMR_MR28_Msk
  EXTI_EMR_MR29_Pos* = (29)
  EXTI_EMR_MR29_Msk* = (0x00000001 shl EXTI_EMR_MR29_Pos) ## !< 0x20000000
  EXTI_EMR_MR29* = EXTI_EMR_MR29_Msk
  EXTI_EMR_MR30_Pos* = (30)
  EXTI_EMR_MR30_Msk* = (0x00000001 shl EXTI_EMR_MR30_Pos) ## !< 0x40000000
  EXTI_EMR_MR30* = EXTI_EMR_MR30_Msk
  EXTI_EMR_MR31_Pos* = (31)
  EXTI_EMR_MR31_Msk* = (0x00000001 shl EXTI_EMR_MR31_Pos) ## !< 0x80000000
  EXTI_EMR_MR31* = EXTI_EMR_MR31_Msk

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
  EXTI_EMR_EM24* = EXTI_EMR_MR24
  EXTI_EMR_EM25* = EXTI_EMR_MR25
  EXTI_EMR_EM26* = EXTI_EMR_MR26

when defined(EXTI_EMR_MR27):
  const
    EXTI_EMR_EM27* = EXTI_EMR_MR27
const
  EXTI_EMR_EM28* = EXTI_EMR_MR28
  EXTI_EMR_EM29* = EXTI_EMR_MR29
  EXTI_EMR_EM30* = EXTI_EMR_MR30
  EXTI_EMR_EM31* = EXTI_EMR_MR31

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
  EXTI_RTSR_TR29_Pos* = (29)
  EXTI_RTSR_TR29_Msk* = (0x00000001 shl EXTI_RTSR_TR29_Pos) ## !< 0x20000000
  EXTI_RTSR_TR29* = EXTI_RTSR_TR29_Msk
  EXTI_RTSR_TR30_Pos* = (30)
  EXTI_RTSR_TR30_Msk* = (0x00000001 shl EXTI_RTSR_TR30_Pos) ## !< 0x40000000
  EXTI_RTSR_TR30* = EXTI_RTSR_TR30_Msk
  EXTI_RTSR_TR31_Pos* = (31)
  EXTI_RTSR_TR31_Msk* = (0x00000001 shl EXTI_RTSR_TR31_Pos) ## !< 0x80000000
  EXTI_RTSR_TR31* = EXTI_RTSR_TR31_Msk

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

when defined(EXTI_RTSR_TR23):
  const
    EXTI_RTSR_RT23* = EXTI_RTSR_TR23
when defined(EXTI_RTSR_TR24):
  const
    EXTI_RTSR_RT24* = EXTI_RTSR_TR24
when defined(EXTI_RTSR_TR25):
  const
    EXTI_RTSR_RT25* = EXTI_RTSR_TR25
when defined(EXTI_RTSR_TR26):
  const
    EXTI_RTSR_RT26* = EXTI_RTSR_TR26
when defined(EXTI_RTSR_TR27):
  const
    EXTI_RTSR_RT27* = EXTI_RTSR_TR27
when defined(EXTI_RTSR_TR28):
  const
    EXTI_RTSR_RT28* = EXTI_RTSR_TR28
const
  EXTI_RTSR_RT29* = EXTI_RTSR_TR29
  EXTI_RTSR_RT30* = EXTI_RTSR_TR30
  EXTI_RTSR_RT31* = EXTI_RTSR_TR31

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
  EXTI_FTSR_TR29_Pos* = (29)
  EXTI_FTSR_TR29_Msk* = (0x00000001 shl EXTI_FTSR_TR29_Pos) ## !< 0x20000000
  EXTI_FTSR_TR29* = EXTI_FTSR_TR29_Msk
  EXTI_FTSR_TR30_Pos* = (30)
  EXTI_FTSR_TR30_Msk* = (0x00000001 shl EXTI_FTSR_TR30_Pos) ## !< 0x40000000
  EXTI_FTSR_TR30* = EXTI_FTSR_TR30_Msk
  EXTI_FTSR_TR31_Pos* = (31)
  EXTI_FTSR_TR31_Msk* = (0x00000001 shl EXTI_FTSR_TR31_Pos) ## !< 0x80000000
  EXTI_FTSR_TR31* = EXTI_FTSR_TR31_Msk

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

when defined(EXTI_FTSR_TR23):
  const
    EXTI_FTSR_FT23* = EXTI_FTSR_TR23
when defined(EXTI_FTSR_TR24):
  const
    EXTI_FTSR_FT24* = EXTI_FTSR_TR24
when defined(EXTI_FTSR_TR25):
  const
    EXTI_FTSR_FT25* = EXTI_FTSR_TR25
when defined(EXTI_FTSR_TR26):
  const
    EXTI_FTSR_FT26* = EXTI_FTSR_TR26
when defined(EXTI_FTSR_TR27):
  const
    EXTI_FTSR_FT27* = EXTI_FTSR_TR27
when defined(EXTI_FTSR_TR28):
  const
    EXTI_FTSR_FT28* = EXTI_FTSR_TR28
const
  EXTI_FTSR_FT29* = EXTI_FTSR_TR29
  EXTI_FTSR_FT30* = EXTI_FTSR_TR30
  EXTI_FTSR_FT31* = EXTI_FTSR_TR31

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
  EXTI_SWIER_SWIER29_Pos* = (29)
  EXTI_SWIER_SWIER29_Msk* = (0x00000001 shl EXTI_SWIER_SWIER29_Pos) ## !< 0x20000000
  EXTI_SWIER_SWIER29* = EXTI_SWIER_SWIER29_Msk
  EXTI_SWIER_SWIER30_Pos* = (30)
  EXTI_SWIER_SWIER30_Msk* = (0x00000001 shl EXTI_SWIER_SWIER30_Pos) ## !< 0x40000000
  EXTI_SWIER_SWIER30* = EXTI_SWIER_SWIER30_Msk
  EXTI_SWIER_SWIER31_Pos* = (31)
  EXTI_SWIER_SWIER31_Msk* = (0x00000001 shl EXTI_SWIER_SWIER31_Pos) ## !< 0x80000000
  EXTI_SWIER_SWIER31* = EXTI_SWIER_SWIER31_Msk

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

when defined(EXTI_SWIER_SWIER23):
  const
    EXTI_SWIER_SWI23* = EXTI_SWIER_SWIER23
when defined(EXTI_SWIER_SWIER24):
  const
    EXTI_SWIER_SWI24* = EXTI_SWIER_SWIER24
when defined(EXTI_SWIER_SWIER25):
  const
    EXTI_SWIER_SWI25* = EXTI_SWIER_SWIER25
when defined(EXTI_SWIER_SWIER26):
  const
    EXTI_SWIER_SWI26* = EXTI_SWIER_SWIER26
when defined(EXTI_SWIER_SWIER27):
  const
    EXTI_SWIER_SWI27* = EXTI_SWIER_SWIER27
when defined(EXTI_SWIER_SWIER28):
  const
    EXTI_SWIER_SWI28* = EXTI_SWIER_SWIER28
const
  EXTI_SWIER_SWI29* = EXTI_SWIER_SWIER29
  EXTI_SWIER_SWI30* = EXTI_SWIER_SWIER30
  EXTI_SWIER_SWI31* = EXTI_SWIER_SWIER31

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
  EXTI_PR_PR29_Pos* = (29)
  EXTI_PR_PR29_Msk* = (0x00000001 shl EXTI_PR_PR29_Pos) ## !< 0x20000000
  EXTI_PR_PR29* = EXTI_PR_PR29_Msk
  EXTI_PR_PR30_Pos* = (30)
  EXTI_PR_PR30_Msk* = (0x00000001 shl EXTI_PR_PR30_Pos) ## !< 0x40000000
  EXTI_PR_PR30* = EXTI_PR_PR30_Msk
  EXTI_PR_PR31_Pos* = (31)
  EXTI_PR_PR31_Msk* = (0x00000001 shl EXTI_PR_PR31_Pos) ## !< 0x80000000
  EXTI_PR_PR31* = EXTI_PR_PR31_Msk

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

when defined(EXTI_PR_PR23):
  const
    EXTI_PR_PIF23* = EXTI_PR_PR23
when defined(EXTI_PR_PR24):
  const
    EXTI_PR_PIF24* = EXTI_PR_PR24
when defined(EXTI_PR_PR25):
  const
    EXTI_PR_PIF25* = EXTI_PR_PR25
when defined(EXTI_PR_PR26):
  const
    EXTI_PR_PIF26* = EXTI_PR_PR26
when defined(EXTI_PR_PR27):
  const
    EXTI_PR_PIF27* = EXTI_PR_PR27
when defined(EXTI_PR_PR28):
  const
    EXTI_PR_PIF28* = EXTI_PR_PR28
const
  EXTI_PR_PIF29* = EXTI_PR_PR29
  EXTI_PR_PIF30* = EXTI_PR_PR30
  EXTI_PR_PIF31* = EXTI_PR_PR31
  EXTI_32_63_SUPPORT* = true    ##  EXTI support more than 32 lines

## ******************  Bit definition for EXTI_IMR2 register  *****************

const
  EXTI_IMR2_MR32_Pos* = (0)
  EXTI_IMR2_MR32_Msk* = (0x00000001 shl EXTI_IMR2_MR32_Pos) ## !< 0x00000001
  EXTI_IMR2_MR32* = EXTI_IMR2_MR32_Msk
  EXTI_IMR2_MR33_Pos* = (1)
  EXTI_IMR2_MR33_Msk* = (0x00000001 shl EXTI_IMR2_MR33_Pos) ## !< 0x00000002
  EXTI_IMR2_MR33* = EXTI_IMR2_MR33_Msk
  EXTI_IMR2_MR34_Pos* = (2)
  EXTI_IMR2_MR34_Msk* = (0x00000001 shl EXTI_IMR2_MR34_Pos) ## !< 0x00000004
  EXTI_IMR2_MR34* = EXTI_IMR2_MR34_Msk
  EXTI_IMR2_MR35_Pos* = (3)
  EXTI_IMR2_MR35_Msk* = (0x00000001 shl EXTI_IMR2_MR35_Pos) ## !< 0x00000008
  EXTI_IMR2_MR35* = EXTI_IMR2_MR35_Msk

##  References Defines

const
  EXTI_IMR2_IM32* = EXTI_IMR2_MR32
  EXTI_IMR2_IM33* = EXTI_IMR2_MR33
  EXTI_IMR2_IM34* = EXTI_IMR2_MR34
  EXTI_IMR2_IM35* = EXTI_IMR2_MR35
  EXTI_IMR2_IM_Pos* = (0)
  EXTI_IMR2_IM_Msk* = (0x0000000F shl EXTI_IMR2_IM_Pos) ## !< 0x0000000F
  EXTI_IMR2_IM* = EXTI_IMR2_IM_Msk

## ******************  Bit definition for EXTI_EMR2 ***************************

const
  EXTI_EMR2_MR32_Pos* = (0)
  EXTI_EMR2_MR32_Msk* = (0x00000001 shl EXTI_EMR2_MR32_Pos) ## !< 0x00000001
  EXTI_EMR2_MR32* = EXTI_EMR2_MR32_Msk
  EXTI_EMR2_MR33_Pos* = (1)
  EXTI_EMR2_MR33_Msk* = (0x00000001 shl EXTI_EMR2_MR33_Pos) ## !< 0x00000002
  EXTI_EMR2_MR33* = EXTI_EMR2_MR33_Msk
  EXTI_EMR2_MR34_Pos* = (2)
  EXTI_EMR2_MR34_Msk* = (0x00000001 shl EXTI_EMR2_MR34_Pos) ## !< 0x00000004
  EXTI_EMR2_MR34* = EXTI_EMR2_MR34_Msk
  EXTI_EMR2_MR35_Pos* = (3)
  EXTI_EMR2_MR35_Msk* = (0x00000001 shl EXTI_EMR2_MR35_Pos) ## !< 0x00000008
  EXTI_EMR2_MR35* = EXTI_EMR2_MR35_Msk

##  References Defines

const
  EXTI_EMR2_EM32* = EXTI_EMR2_MR32
  EXTI_EMR2_EM33* = EXTI_EMR2_MR33
  EXTI_EMR2_EM34* = EXTI_EMR2_MR34
  EXTI_EMR2_EM35* = EXTI_EMR2_MR35
  EXTI_EMR2_EM_Pos* = (0)
  EXTI_EMR2_EM_Msk* = (0x0000000F shl EXTI_EMR2_EM_Pos) ## !< 0x0000000F
  EXTI_EMR2_EM* = EXTI_EMR2_EM_Msk

## *****************  Bit definition for EXTI_RTSR2 register *******************

const
  EXTI_RTSR2_TR32_Pos* = (0)
  EXTI_RTSR2_TR32_Msk* = (0x00000001 shl EXTI_RTSR2_TR32_Pos) ## !< 0x00000001
  EXTI_RTSR2_TR32* = EXTI_RTSR2_TR32_Msk
  EXTI_RTSR2_TR33_Pos* = (1)
  EXTI_RTSR2_TR33_Msk* = (0x00000001 shl EXTI_RTSR2_TR33_Pos) ## !< 0x00000002
  EXTI_RTSR2_TR33* = EXTI_RTSR2_TR33_Msk

##  References Defines

const
  EXTI_RTSR2_RT32* = EXTI_RTSR2_TR32
  EXTI_RTSR2_RT33* = EXTI_RTSR2_TR33

when defined(EXTI_RTSR2_TR34):
  const
    EXTI_RTSR2_RT34* = EXTI_RTSR2_TR34
when defined(EXTI_RTSR2_TR35):
  const
    EXTI_RTSR2_RT35* = EXTI_RTSR2_TR35
## *****************  Bit definition for EXTI_FTSR2 register  *****************

const
  EXTI_FTSR2_TR32_Pos* = (0)
  EXTI_FTSR2_TR32_Msk* = (0x00000001 shl EXTI_FTSR2_TR32_Pos) ## !< 0x00000001
  EXTI_FTSR2_TR32* = EXTI_FTSR2_TR32_Msk
  EXTI_FTSR2_TR33_Pos* = (1)
  EXTI_FTSR2_TR33_Msk* = (0x00000001 shl EXTI_FTSR2_TR33_Pos) ## !< 0x00000002
  EXTI_FTSR2_TR33* = EXTI_FTSR2_TR33_Msk

##  References Defines

const
  EXTI_FTSR2_FT32* = EXTI_FTSR2_TR32
  EXTI_FTSR2_FT33* = EXTI_FTSR2_TR33

when defined(EXTI_FTSR2_TR34):
  const
    EXTI_FTSR2_FT34* = EXTI_FTSR2_TR34
when defined(EXTI_FTSR2_TR35):
  const
    EXTI_FTSR2_FT35* = EXTI_FTSR2_TR35
## *****************  Bit definition for EXTI_SWIER2 register  ****************

const
  EXTI_SWIER2_SWIER32_Pos* = (0)
  EXTI_SWIER2_SWIER32_Msk* = (0x00000001 shl EXTI_SWIER2_SWIER32_Pos) ## !< 0x00000001
  EXTI_SWIER2_SWIER32* = EXTI_SWIER2_SWIER32_Msk
  EXTI_SWIER2_SWIER33_Pos* = (1)
  EXTI_SWIER2_SWIER33_Msk* = (0x00000001 shl EXTI_SWIER2_SWIER33_Pos) ## !< 0x00000002
  EXTI_SWIER2_SWIER33* = EXTI_SWIER2_SWIER33_Msk

##  References Defines

const
  EXTI_SWIER2_SWI32* = EXTI_SWIER2_SWIER32
  EXTI_SWIER2_SWI33* = EXTI_SWIER2_SWIER33

when defined(EXTI_SWIER2_SWIER34):
  const
    EXTI_SWIER2_SWI34* = EXTI_SWIER2_SWIER34
when defined(EXTI_SWIER2_SWIER35):
  const
    EXTI_SWIER2_SWI35* = EXTI_SWIER2_SWIER35
## ******************  Bit definition for EXTI_PR2 register  ******************

const
  EXTI_PR2_PR32_Pos* = (0)
  EXTI_PR2_PR32_Msk* = (0x00000001 shl EXTI_PR2_PR32_Pos) ## !< 0x00000001
  EXTI_PR2_PR32* = EXTI_PR2_PR32_Msk
  EXTI_PR2_PR33_Pos* = (1)
  EXTI_PR2_PR33_Msk* = (0x00000001 shl EXTI_PR2_PR33_Pos) ## !< 0x00000002
  EXTI_PR2_PR33* = EXTI_PR2_PR33_Msk

##  References Defines

const
  EXTI_PR2_PIF32* = EXTI_PR2_PR32
  EXTI_PR2_PIF33* = EXTI_PR2_PR33

when defined(EXTI_PR2_PR34):
  const
    EXTI_PR2_PIF34* = EXTI_PR2_PR34
when defined(EXTI_PR2_PR35):
  const
    EXTI_PR2_PIF35* = EXTI_PR2_PR35
## ****************************************************************************
##
##                                     FLASH
##
## ****************************************************************************
## ******************  Bit definition for FLASH_ACR register  *****************

const
  FLASH_ACR_LATENCY_Pos* = (0)
  FLASH_ACR_LATENCY_Msk* = (0x00000007 shl FLASH_ACR_LATENCY_Pos) ## !< 0x00000007
  FLASH_ACR_LATENCY* = FLASH_ACR_LATENCY_Msk
  FLASH_ACR_LATENCY_0* = (0x00000001 shl FLASH_ACR_LATENCY_Pos) ## !< 0x00000001
  FLASH_ACR_LATENCY_1* = (0x00000002 shl FLASH_ACR_LATENCY_Pos) ## !< 0x00000002
  FLASH_ACR_LATENCY_2* = (0x00000004 shl FLASH_ACR_LATENCY_Pos) ## !< 0x00000004
  FLASH_ACR_HLFCYA_Pos* = (3)
  FLASH_ACR_HLFCYA_Msk* = (0x00000001 shl FLASH_ACR_HLFCYA_Pos) ## !< 0x00000008
  FLASH_ACR_HLFCYA* = FLASH_ACR_HLFCYA_Msk
  FLASH_ACR_PRFTBE_Pos* = (4)
  FLASH_ACR_PRFTBE_Msk* = (0x00000001 shl FLASH_ACR_PRFTBE_Pos) ## !< 0x00000010
  FLASH_ACR_PRFTBE* = FLASH_ACR_PRFTBE_Msk
  FLASH_ACR_PRFTBS_Pos* = (5)
  FLASH_ACR_PRFTBS_Msk* = (0x00000001 shl FLASH_ACR_PRFTBS_Pos) ## !< 0x00000020
  FLASH_ACR_PRFTBS* = FLASH_ACR_PRFTBS_Msk

## *****************  Bit definition for FLASH_KEYR register  *****************

const
  FLASH_KEYR_FKEYR_Pos* = (0)
  FLASH_KEYR_FKEYR_Msk* = (0xFFFFFFFF shl FLASH_KEYR_FKEYR_Pos) ## !< 0xFFFFFFFF
  FLASH_KEYR_FKEYR* = FLASH_KEYR_FKEYR_Msk
  RDP_KEY_Pos* = (0)
  RDP_KEY_Msk* = (0x000000A5 shl RDP_KEY_Pos) ## !< 0x000000A5
  RDP_KEY* = RDP_KEY_Msk
  FLASH_KEY1_Pos* = (0)
  FLASH_KEY1_Msk* = (0x45670123 shl FLASH_KEY1_Pos) ## !< 0x45670123
  FLASH_KEY1* = FLASH_KEY1_Msk
  FLASH_KEY2_Pos* = (0)
  FLASH_KEY2_Msk* = (0xCDEF89AB shl FLASH_KEY2_Pos) ## !< 0xCDEF89AB
  FLASH_KEY2* = FLASH_KEY2_Msk

## ****************  Bit definition for FLASH_OPTKEYR register  ***************

const
  FLASH_OPTKEYR_OPTKEYR_Pos* = (0)
  FLASH_OPTKEYR_OPTKEYR_Msk* = (0xFFFFFFFF shl FLASH_OPTKEYR_OPTKEYR_Pos) ## !< 0xFFFFFFFF
  FLASH_OPTKEYR_OPTKEYR* = FLASH_OPTKEYR_OPTKEYR_Msk
  FLASH_OPTKEY1* = FLASH_KEY1
  FLASH_OPTKEY2* = FLASH_KEY2

## *****************  Bit definition for FLASH_SR register  ******************

const
  FLASH_SR_BSY_Pos* = (0)
  FLASH_SR_BSY_Msk* = (0x00000001 shl FLASH_SR_BSY_Pos) ## !< 0x00000001
  FLASH_SR_BSY* = FLASH_SR_BSY_Msk
  FLASH_SR_PGERR_Pos* = (2)
  FLASH_SR_PGERR_Msk* = (0x00000001 shl FLASH_SR_PGERR_Pos) ## !< 0x00000004
  FLASH_SR_PGERR* = FLASH_SR_PGERR_Msk
  FLASH_SR_WRPERR_Pos* = (4)
  FLASH_SR_WRPERR_Msk* = (0x00000001 shl FLASH_SR_WRPERR_Pos) ## !< 0x00000010
  FLASH_SR_WRPERR* = FLASH_SR_WRPERR_Msk
  FLASH_SR_EOP_Pos* = (5)
  FLASH_SR_EOP_Msk* = (0x00000001 shl FLASH_SR_EOP_Pos) ## !< 0x00000020
  FLASH_SR_EOP* = FLASH_SR_EOP_Msk

## ******************  Bit definition for FLASH_CR register  ******************

const
  FLASH_CR_PG_Pos* = (0)
  FLASH_CR_PG_Msk* = (0x00000001 shl FLASH_CR_PG_Pos) ## !< 0x00000001
  FLASH_CR_PG* = FLASH_CR_PG_Msk
  FLASH_CR_PER_Pos* = (1)
  FLASH_CR_PER_Msk* = (0x00000001 shl FLASH_CR_PER_Pos) ## !< 0x00000002
  FLASH_CR_PER* = FLASH_CR_PER_Msk
  FLASH_CR_MER_Pos* = (2)
  FLASH_CR_MER_Msk* = (0x00000001 shl FLASH_CR_MER_Pos) ## !< 0x00000004
  FLASH_CR_MER* = FLASH_CR_MER_Msk
  FLASH_CR_OPTPG_Pos* = (4)
  FLASH_CR_OPTPG_Msk* = (0x00000001 shl FLASH_CR_OPTPG_Pos) ## !< 0x00000010
  FLASH_CR_OPTPG* = FLASH_CR_OPTPG_Msk
  FLASH_CR_OPTER_Pos* = (5)
  FLASH_CR_OPTER_Msk* = (0x00000001 shl FLASH_CR_OPTER_Pos) ## !< 0x00000020
  FLASH_CR_OPTER* = FLASH_CR_OPTER_Msk
  FLASH_CR_STRT_Pos* = (6)
  FLASH_CR_STRT_Msk* = (0x00000001 shl FLASH_CR_STRT_Pos) ## !< 0x00000040
  FLASH_CR_STRT* = FLASH_CR_STRT_Msk
  FLASH_CR_LOCK_Pos* = (7)
  FLASH_CR_LOCK_Msk* = (0x00000001 shl FLASH_CR_LOCK_Pos) ## !< 0x00000080
  FLASH_CR_LOCK* = FLASH_CR_LOCK_Msk
  FLASH_CR_OPTWRE_Pos* = (9)
  FLASH_CR_OPTWRE_Msk* = (0x00000001 shl FLASH_CR_OPTWRE_Pos) ## !< 0x00000200
  FLASH_CR_OPTWRE* = FLASH_CR_OPTWRE_Msk
  FLASH_CR_ERRIE_Pos* = (10)
  FLASH_CR_ERRIE_Msk* = (0x00000001 shl FLASH_CR_ERRIE_Pos) ## !< 0x00000400
  FLASH_CR_ERRIE* = FLASH_CR_ERRIE_Msk
  FLASH_CR_EOPIE_Pos* = (12)
  FLASH_CR_EOPIE_Msk* = (0x00000001 shl FLASH_CR_EOPIE_Pos) ## !< 0x00001000
  FLASH_CR_EOPIE* = FLASH_CR_EOPIE_Msk
  FLASH_CR_OBL_LAUNCH_Pos* = (13)
  FLASH_CR_OBL_LAUNCH_Msk* = (0x00000001 shl FLASH_CR_OBL_LAUNCH_Pos) ## !< 0x00002000
  FLASH_CR_OBL_LAUNCH* = FLASH_CR_OBL_LAUNCH_Msk

## ******************  Bit definition for FLASH_AR register  ******************

const
  FLASH_AR_FAR_Pos* = (0)
  FLASH_AR_FAR_Msk* = (0xFFFFFFFF shl FLASH_AR_FAR_Pos) ## !< 0xFFFFFFFF
  FLASH_AR_FAR* = FLASH_AR_FAR_Msk

## *****************  Bit definition for FLASH_OBR register  ******************

const
  FLASH_OBR_OPTERR_Pos* = (0)
  FLASH_OBR_OPTERR_Msk* = (0x00000001 shl FLASH_OBR_OPTERR_Pos) ## !< 0x00000001
  FLASH_OBR_OPTERR* = FLASH_OBR_OPTERR_Msk
  FLASH_OBR_RDPRT_Pos* = (1)
  FLASH_OBR_RDPRT_Msk* = (0x00000003 shl FLASH_OBR_RDPRT_Pos) ## !< 0x00000006
  FLASH_OBR_RDPRT* = FLASH_OBR_RDPRT_Msk
  FLASH_OBR_RDPRT_1* = (0x00000001 shl FLASH_OBR_RDPRT_Pos) ## !< 0x00000002
  FLASH_OBR_RDPRT_2* = (0x00000003 shl FLASH_OBR_RDPRT_Pos) ## !< 0x00000006
  FLASH_OBR_USER_Pos* = (8)
  FLASH_OBR_USER_Msk* = (0x00000077 shl FLASH_OBR_USER_Pos) ## !< 0x00007700
  FLASH_OBR_USER* = FLASH_OBR_USER_Msk
  FLASH_OBR_IWDG_SW_Pos* = (8)
  FLASH_OBR_IWDG_SW_Msk* = (0x00000001 shl FLASH_OBR_IWDG_SW_Pos) ## !< 0x00000100
  FLASH_OBR_IWDG_SW* = FLASH_OBR_IWDG_SW_Msk
  FLASH_OBR_nRST_STOP_Pos* = (9)
  FLASH_OBR_nRST_STOP_Msk* = (0x00000001 shl FLASH_OBR_nRST_STOP_Pos) ## !< 0x00000200
  FLASH_OBR_nRST_STOP* = FLASH_OBR_nRST_STOP_Msk
  FLASH_OBR_nRST_STDBY_Pos* = (10)
  FLASH_OBR_nRST_STDBY_Msk* = (0x00000001 shl FLASH_OBR_nRST_STDBY_Pos) ## !< 0x00000400
  FLASH_OBR_nRST_STDBY* = FLASH_OBR_nRST_STDBY_Msk
  FLASH_OBR_nBOOT1_Pos* = (12)
  FLASH_OBR_nBOOT1_Msk* = (0x00000001 shl FLASH_OBR_nBOOT1_Pos) ## !< 0x00001000
  FLASH_OBR_nBOOT1* = FLASH_OBR_nBOOT1_Msk
  FLASH_OBR_VDDA_MONITOR_Pos* = (13)
  FLASH_OBR_VDDA_MONITOR_Msk* = (0x00000001 shl FLASH_OBR_VDDA_MONITOR_Pos) ## !< 0x00002000
  FLASH_OBR_VDDA_MONITOR* = FLASH_OBR_VDDA_MONITOR_Msk
  FLASH_OBR_SRAM_PE_Pos* = (14)
  FLASH_OBR_SRAM_PE_Msk* = (0x00000001 shl FLASH_OBR_SRAM_PE_Pos) ## !< 0x00004000
  FLASH_OBR_SRAM_PE* = FLASH_OBR_SRAM_PE_Msk
  FLASH_OBR_DATA0_Pos* = (16)
  FLASH_OBR_DATA0_Msk* = (0x000000FF shl FLASH_OBR_DATA0_Pos) ## !< 0x00FF0000
  FLASH_OBR_DATA0* = FLASH_OBR_DATA0_Msk
  FLASH_OBR_DATA1_Pos* = (24)
  FLASH_OBR_DATA1_Msk* = (0x000000FF shl FLASH_OBR_DATA1_Pos) ## !< 0xFF000000
  FLASH_OBR_DATA1* = FLASH_OBR_DATA1_Msk

##  Legacy defines

const
  FLASH_OBR_WDG_SW* = FLASH_OBR_IWDG_SW

## *****************  Bit definition for FLASH_WRPR register  *****************

const
  FLASH_WRPR_WRP_Pos* = (0)
  FLASH_WRPR_WRP_Msk* = (0xFFFFFFFF shl FLASH_WRPR_WRP_Pos) ## !< 0xFFFFFFFF
  FLASH_WRPR_WRP* = FLASH_WRPR_WRP_Msk

## ----------------------------------------------------------------------------
## *****************  Bit definition for OB_RDP register  *********************

const
  OB_RDP_RDP_Pos* = (0)
  OB_RDP_RDP_Msk* = (0x000000FF shl OB_RDP_RDP_Pos) ## !< 0x000000FF
  OB_RDP_RDP* = OB_RDP_RDP_Msk
  OB_RDP_nRDP_Pos* = (8)
  OB_RDP_nRDP_Msk* = (0x000000FF shl OB_RDP_nRDP_Pos) ## !< 0x0000FF00
  OB_RDP_nRDP* = OB_RDP_nRDP_Msk

## *****************  Bit definition for OB_USER register  ********************

const
  OB_USER_USER_Pos* = (16)
  OB_USER_USER_Msk* = (0x000000FF shl OB_USER_USER_Pos) ## !< 0x00FF0000
  OB_USER_USER* = OB_USER_USER_Msk
  OB_USER_nUSER_Pos* = (24)
  OB_USER_nUSER_Msk* = (0x000000FF shl OB_USER_nUSER_Pos) ## !< 0xFF000000
  OB_USER_nUSER* = OB_USER_nUSER_Msk

## *****************  Bit definition for FLASH_WRP0 register  *****************

const
  OB_WRP0_WRP0_Pos* = (0)
  OB_WRP0_WRP0_Msk* = (0x000000FF shl OB_WRP0_WRP0_Pos) ## !< 0x000000FF
  OB_WRP0_WRP0* = OB_WRP0_WRP0_Msk
  OB_WRP0_nWRP0_Pos* = (8)
  OB_WRP0_nWRP0_Msk* = (0x000000FF shl OB_WRP0_nWRP0_Pos) ## !< 0x0000FF00
  OB_WRP0_nWRP0* = OB_WRP0_nWRP0_Msk

## *****************  Bit definition for FLASH_WRP1 register  *****************

const
  OB_WRP1_WRP1_Pos* = (16)
  OB_WRP1_WRP1_Msk* = (0x000000FF shl OB_WRP1_WRP1_Pos) ## !< 0x00FF0000
  OB_WRP1_WRP1* = OB_WRP1_WRP1_Msk
  OB_WRP1_nWRP1_Pos* = (24)
  OB_WRP1_nWRP1_Msk* = (0x000000FF shl OB_WRP1_nWRP1_Pos) ## !< 0xFF000000
  OB_WRP1_nWRP1* = OB_WRP1_nWRP1_Msk

## *****************  Bit definition for FLASH_WRP2 register  *****************

const
  OB_WRP2_WRP2_Pos* = (0)
  OB_WRP2_WRP2_Msk* = (0x000000FF shl OB_WRP2_WRP2_Pos) ## !< 0x000000FF
  OB_WRP2_WRP2* = OB_WRP2_WRP2_Msk
  OB_WRP2_nWRP2_Pos* = (8)
  OB_WRP2_nWRP2_Msk* = (0x000000FF shl OB_WRP2_nWRP2_Pos) ## !< 0x0000FF00
  OB_WRP2_nWRP2* = OB_WRP2_nWRP2_Msk

## *****************  Bit definition for FLASH_WRP3 register  *****************

const
  OB_WRP3_WRP3_Pos* = (16)
  OB_WRP3_WRP3_Msk* = (0x000000FF shl OB_WRP3_WRP3_Pos) ## !< 0x00FF0000
  OB_WRP3_WRP3* = OB_WRP3_WRP3_Msk
  OB_WRP3_nWRP3_Pos* = (24)
  OB_WRP3_nWRP3_Msk* = (0x000000FF shl OB_WRP3_nWRP3_Pos) ## !< 0xFF000000
  OB_WRP3_nWRP3* = OB_WRP3_nWRP3_Msk

## ****************************************************************************
##
##                             General Purpose I/O (GPIO)
##
## ****************************************************************************
## ******************  Bit definition for GPIO_MODER register  ****************

const
  GPIO_MODER_MODER0_Pos* = (0)
  GPIO_MODER_MODER0_Msk* = (0x00000003 shl GPIO_MODER_MODER0_Pos) ## !< 0x00000003
  GPIO_MODER_MODER0* = GPIO_MODER_MODER0_Msk
  GPIO_MODER_MODER0_0* = (0x00000001 shl GPIO_MODER_MODER0_Pos) ## !< 0x00000001
  GPIO_MODER_MODER0_1* = (0x00000002 shl GPIO_MODER_MODER0_Pos) ## !< 0x00000002
  GPIO_MODER_MODER1_Pos* = (2)
  GPIO_MODER_MODER1_Msk* = (0x00000003 shl GPIO_MODER_MODER1_Pos) ## !< 0x0000000C
  GPIO_MODER_MODER1* = GPIO_MODER_MODER1_Msk
  GPIO_MODER_MODER1_0* = (0x00000001 shl GPIO_MODER_MODER1_Pos) ## !< 0x00000004
  GPIO_MODER_MODER1_1* = (0x00000002 shl GPIO_MODER_MODER1_Pos) ## !< 0x00000008
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
  GPIO_MODER_MODER10x* = GPIO_MODER_MODER10_Msk
  GPIO_MODER_MODER10_0* = (0x00000001 shl GPIO_MODER_MODER10_Pos) ## !< 0x00100000
  GPIO_MODER_MODER10_1* = (0x00000002 shl GPIO_MODER_MODER10_Pos) ## !< 0x00200000
  GPIO_MODER_MODER11_Pos* = (22)
  GPIO_MODER_MODER11_Msk* = (0x00000003 shl GPIO_MODER_MODER11_Pos) ## !< 0x00C00000
  GPIO_MODER_MODER11x* = GPIO_MODER_MODER11_Msk
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

## *****************  Bit definition for GPIO_OTYPER register  ****************

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

## ***************  Bit definition for GPIO_OSPEEDR register  *****************

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

## ******************  Bit definition for GPIO_PUPDR register *****************

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

## ******************  Bit definition for GPIO_IDR register  ******************

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

## *****************  Bit definition for GPIO_ODR register  *******************

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

## ***************** Bit definition for GPIO_BSRR register  *******************

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
  GPIO_AFRL_AFRL0_Pos* = (0)
  GPIO_AFRL_AFRL0_Msk* = (0x0000000F shl GPIO_AFRL_AFRL0_Pos) ## !< 0x0000000F
  GPIO_AFRL_AFRL0* = GPIO_AFRL_AFRL0_Msk
  GPIO_AFRL_AFRL1_Pos* = (4)
  GPIO_AFRL_AFRL1_Msk* = (0x0000000F shl GPIO_AFRL_AFRL1_Pos) ## !< 0x000000F0
  GPIO_AFRL_AFRL1* = GPIO_AFRL_AFRL1_Msk
  GPIO_AFRL_AFRL2_Pos* = (8)
  GPIO_AFRL_AFRL2_Msk* = (0x0000000F shl GPIO_AFRL_AFRL2_Pos) ## !< 0x00000F00
  GPIO_AFRL_AFRL2* = GPIO_AFRL_AFRL2_Msk
  GPIO_AFRL_AFRL3_Pos* = (12)
  GPIO_AFRL_AFRL3_Msk* = (0x0000000F shl GPIO_AFRL_AFRL3_Pos) ## !< 0x0000F000
  GPIO_AFRL_AFRL3* = GPIO_AFRL_AFRL3_Msk
  GPIO_AFRL_AFRL4_Pos* = (16)
  GPIO_AFRL_AFRL4_Msk* = (0x0000000F shl GPIO_AFRL_AFRL4_Pos) ## !< 0x000F0000
  GPIO_AFRL_AFRL4* = GPIO_AFRL_AFRL4_Msk
  GPIO_AFRL_AFRL5_Pos* = (20)
  GPIO_AFRL_AFRL5_Msk* = (0x0000000F shl GPIO_AFRL_AFRL5_Pos) ## !< 0x00F00000
  GPIO_AFRL_AFRL5* = GPIO_AFRL_AFRL5_Msk
  GPIO_AFRL_AFRL6_Pos* = (24)
  GPIO_AFRL_AFRL6_Msk* = (0x0000000F shl GPIO_AFRL_AFRL6_Pos) ## !< 0x0F000000
  GPIO_AFRL_AFRL6* = GPIO_AFRL_AFRL6_Msk
  GPIO_AFRL_AFRL7_Pos* = (28)
  GPIO_AFRL_AFRL7_Msk* = (0x0000000F shl GPIO_AFRL_AFRL7_Pos) ## !< 0xF0000000
  GPIO_AFRL_AFRL7* = GPIO_AFRL_AFRL7_Msk

## ***************** Bit definition for GPIO_AFRH register  *******************

const
  GPIO_AFRH_AFRH0_Pos* = (0)
  GPIO_AFRH_AFRH0_Msk* = (0x0000000F shl GPIO_AFRH_AFRH0_Pos) ## !< 0x0000000F
  GPIO_AFRH_AFRH0* = GPIO_AFRH_AFRH0_Msk
  GPIO_AFRH_AFRH1_Pos* = (4)
  GPIO_AFRH_AFRH1_Msk* = (0x0000000F shl GPIO_AFRH_AFRH1_Pos) ## !< 0x000000F0
  GPIO_AFRH_AFRH1* = GPIO_AFRH_AFRH1_Msk
  GPIO_AFRH_AFRH2_Pos* = (8)
  GPIO_AFRH_AFRH2_Msk* = (0x0000000F shl GPIO_AFRH_AFRH2_Pos) ## !< 0x00000F00
  GPIO_AFRH_AFRH2* = GPIO_AFRH_AFRH2_Msk
  GPIO_AFRH_AFRH3_Pos* = (12)
  GPIO_AFRH_AFRH3_Msk* = (0x0000000F shl GPIO_AFRH_AFRH3_Pos) ## !< 0x0000F000
  GPIO_AFRH_AFRH3* = GPIO_AFRH_AFRH3_Msk
  GPIO_AFRH_AFRH4_Pos* = (16)
  GPIO_AFRH_AFRH4_Msk* = (0x0000000F shl GPIO_AFRH_AFRH4_Pos) ## !< 0x000F0000
  GPIO_AFRH_AFRH4* = GPIO_AFRH_AFRH4_Msk
  GPIO_AFRH_AFRH5_Pos* = (20)
  GPIO_AFRH_AFRH5_Msk* = (0x0000000F shl GPIO_AFRH_AFRH5_Pos) ## !< 0x00F00000
  GPIO_AFRH_AFRH5* = GPIO_AFRH_AFRH5_Msk
  GPIO_AFRH_AFRH6_Pos* = (24)
  GPIO_AFRH_AFRH6_Msk* = (0x0000000F shl GPIO_AFRH_AFRH6_Pos) ## !< 0x0F000000
  GPIO_AFRH_AFRH6* = GPIO_AFRH_AFRH6_Msk
  GPIO_AFRH_AFRH7_Pos* = (28)
  GPIO_AFRH_AFRH7_Msk* = (0x0000000F shl GPIO_AFRH_AFRH7_Pos) ## !< 0xF0000000
  GPIO_AFRH_AFRH7* = GPIO_AFRH_AFRH7_Msk

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
##                       Inter-integrated Circuit Interface (I2C)
##
## ****************************************************************************
## ******************  Bit definition for I2C_CR1 register  ******************

const
  I2C_CR1_PE_Pos* = (0)
  I2C_CR1_PE_Msk* = (0x00000001 shl I2C_CR1_PE_Pos) ## !< 0x00000001
  I2C_CR1_PE* = I2C_CR1_PE_Msk
  I2C_CR1_TXIE_Pos* = (1)
  I2C_CR1_TXIE_Msk* = (0x00000001 shl I2C_CR1_TXIE_Pos) ## !< 0x00000002
  I2C_CR1_TXIE* = I2C_CR1_TXIE_Msk
  I2C_CR1_RXIE_Pos* = (2)
  I2C_CR1_RXIE_Msk* = (0x00000001 shl I2C_CR1_RXIE_Pos) ## !< 0x00000004
  I2C_CR1_RXIE* = I2C_CR1_RXIE_Msk
  I2C_CR1_ADDRIE_Pos* = (3)
  I2C_CR1_ADDRIE_Msk* = (0x00000001 shl I2C_CR1_ADDRIE_Pos) ## !< 0x00000008
  I2C_CR1_ADDRIE* = I2C_CR1_ADDRIE_Msk
  I2C_CR1_NACKIE_Pos* = (4)
  I2C_CR1_NACKIE_Msk* = (0x00000001 shl I2C_CR1_NACKIE_Pos) ## !< 0x00000010
  I2C_CR1_NACKIE* = I2C_CR1_NACKIE_Msk
  I2C_CR1_STOPIE_Pos* = (5)
  I2C_CR1_STOPIE_Msk* = (0x00000001 shl I2C_CR1_STOPIE_Pos) ## !< 0x00000020
  I2C_CR1_STOPIE* = I2C_CR1_STOPIE_Msk
  I2C_CR1_TCIE_Pos* = (6)
  I2C_CR1_TCIE_Msk* = (0x00000001 shl I2C_CR1_TCIE_Pos) ## !< 0x00000040
  I2C_CR1_TCIE* = I2C_CR1_TCIE_Msk
  I2C_CR1_ERRIE_Pos* = (7)
  I2C_CR1_ERRIE_Msk* = (0x00000001 shl I2C_CR1_ERRIE_Pos) ## !< 0x00000080
  I2C_CR1_ERRIE* = I2C_CR1_ERRIE_Msk
  I2C_CR1_DNF_Pos* = (8)
  I2C_CR1_DNF_Msk* = (0x0000000F shl I2C_CR1_DNF_Pos) ## !< 0x00000F00
  I2C_CR1_DNF* = I2C_CR1_DNF_Msk
  I2C_CR1_ANFOFF_Pos* = (12)
  I2C_CR1_ANFOFF_Msk* = (0x00000001 shl I2C_CR1_ANFOFF_Pos) ## !< 0x00001000
  I2C_CR1_ANFOFF* = I2C_CR1_ANFOFF_Msk
  I2C_CR1_SWRST_Pos* = (13)
  I2C_CR1_SWRST_Msk* = (0x00000001 shl I2C_CR1_SWRST_Pos) ## !< 0x00002000
  I2C_CR1_SWRST* = I2C_CR1_SWRST_Msk
  I2C_CR1_TXDMAEN_Pos* = (14)
  I2C_CR1_TXDMAEN_Msk* = (0x00000001 shl I2C_CR1_TXDMAEN_Pos) ## !< 0x00004000
  I2C_CR1_TXDMAEN* = I2C_CR1_TXDMAEN_Msk
  I2C_CR1_RXDMAEN_Pos* = (15)
  I2C_CR1_RXDMAEN_Msk* = (0x00000001 shl I2C_CR1_RXDMAEN_Pos) ## !< 0x00008000
  I2C_CR1_RXDMAEN* = I2C_CR1_RXDMAEN_Msk
  I2C_CR1_SBC_Pos* = (16)
  I2C_CR1_SBC_Msk* = (0x00000001 shl I2C_CR1_SBC_Pos) ## !< 0x00010000
  I2C_CR1_SBC* = I2C_CR1_SBC_Msk
  I2C_CR1_NOSTRETCH_Pos* = (17)
  I2C_CR1_NOSTRETCH_Msk* = (0x00000001 shl I2C_CR1_NOSTRETCH_Pos) ## !< 0x00020000
  I2C_CR1_NOSTRETCH* = I2C_CR1_NOSTRETCH_Msk
  I2C_CR1_WUPEN_Pos* = (18)
  I2C_CR1_WUPEN_Msk* = (0x00000001 shl I2C_CR1_WUPEN_Pos) ## !< 0x00040000
  I2C_CR1_WUPEN* = I2C_CR1_WUPEN_Msk
  I2C_CR1_GCEN_Pos* = (19)
  I2C_CR1_GCEN_Msk* = (0x00000001 shl I2C_CR1_GCEN_Pos) ## !< 0x00080000
  I2C_CR1_GCEN* = I2C_CR1_GCEN_Msk
  I2C_CR1_SMBHEN_Pos* = (20)
  I2C_CR1_SMBHEN_Msk* = (0x00000001 shl I2C_CR1_SMBHEN_Pos) ## !< 0x00100000
  I2C_CR1_SMBHEN* = I2C_CR1_SMBHEN_Msk
  I2C_CR1_SMBDEN_Pos* = (21)
  I2C_CR1_SMBDEN_Msk* = (0x00000001 shl I2C_CR1_SMBDEN_Pos) ## !< 0x00200000
  I2C_CR1_SMBDEN* = I2C_CR1_SMBDEN_Msk
  I2C_CR1_ALERTEN_Pos* = (22)
  I2C_CR1_ALERTEN_Msk* = (0x00000001 shl I2C_CR1_ALERTEN_Pos) ## !< 0x00400000
  I2C_CR1_ALERTEN* = I2C_CR1_ALERTEN_Msk
  I2C_CR1_PECEN_Pos* = (23)
  I2C_CR1_PECEN_Msk* = (0x00000001 shl I2C_CR1_PECEN_Pos) ## !< 0x00800000
  I2C_CR1_PECEN* = I2C_CR1_PECEN_Msk

##  Legacy defines

const
  I2C_CR1_DFN* = I2C_CR1_DNF

## *****************  Bit definition for I2C_CR2 register  *******************

const
  I2C_CR2_SADD_Pos* = (0)
  I2C_CR2_SADD_Msk* = (0x000003FF shl I2C_CR2_SADD_Pos) ## !< 0x000003FF
  I2C_CR2_SADD* = I2C_CR2_SADD_Msk
  I2C_CR2_RD_WRN_Pos* = (10)
  I2C_CR2_RD_WRN_Msk* = (0x00000001 shl I2C_CR2_RD_WRN_Pos) ## !< 0x00000400
  I2C_CR2_RD_WRN* = I2C_CR2_RD_WRN_Msk
  I2C_CR2_ADD10_Pos* = (11)
  I2C_CR2_ADD10_Msk* = (0x00000001 shl I2C_CR2_ADD10_Pos) ## !< 0x00000800
  I2C_CR2_ADD10* = I2C_CR2_ADD10_Msk
  I2C_CR2_HEAD10R_Pos* = (12)
  I2C_CR2_HEAD10R_Msk* = (0x00000001 shl I2C_CR2_HEAD10R_Pos) ## !< 0x00001000
  I2C_CR2_HEAD10R* = I2C_CR2_HEAD10R_Msk
  I2C_CR2_START_Pos* = (13)
  I2C_CR2_START_Msk* = (0x00000001 shl I2C_CR2_START_Pos) ## !< 0x00002000
  I2C_CR2_START* = I2C_CR2_START_Msk
  I2C_CR2_STOP_Pos* = (14)
  I2C_CR2_STOP_Msk* = (0x00000001 shl I2C_CR2_STOP_Pos) ## !< 0x00004000
  I2C_CR2_STOP* = I2C_CR2_STOP_Msk
  I2C_CR2_NACK_Pos* = (15)
  I2C_CR2_NACK_Msk* = (0x00000001 shl I2C_CR2_NACK_Pos) ## !< 0x00008000
  I2C_CR2_NACK* = I2C_CR2_NACK_Msk
  I2C_CR2_NBYTES_Pos* = (16)
  I2C_CR2_NBYTES_Msk* = (0x000000FF shl I2C_CR2_NBYTES_Pos) ## !< 0x00FF0000
  I2C_CR2_NBYTES* = I2C_CR2_NBYTES_Msk
  I2C_CR2_RELOAD_Pos* = (24)
  I2C_CR2_RELOAD_Msk* = (0x00000001 shl I2C_CR2_RELOAD_Pos) ## !< 0x01000000
  I2C_CR2_RELOAD* = I2C_CR2_RELOAD_Msk
  I2C_CR2_AUTOEND_Pos* = (25)
  I2C_CR2_AUTOEND_Msk* = (0x00000001 shl I2C_CR2_AUTOEND_Pos) ## !< 0x02000000
  I2C_CR2_AUTOEND* = I2C_CR2_AUTOEND_Msk
  I2C_CR2_PECBYTE_Pos* = (26)
  I2C_CR2_PECBYTE_Msk* = (0x00000001 shl I2C_CR2_PECBYTE_Pos) ## !< 0x04000000
  I2C_CR2_PECBYTE* = I2C_CR2_PECBYTE_Msk

## ******************  Bit definition for I2C_OAR1 register  *****************

const
  I2C_OAR1_OA1_Pos* = (0)
  I2C_OAR1_OA1_Msk* = (0x000003FF shl I2C_OAR1_OA1_Pos) ## !< 0x000003FF
  I2C_OAR1_OA1* = I2C_OAR1_OA1_Msk
  I2C_OAR1_OA1MODE_Pos* = (10)
  I2C_OAR1_OA1MODE_Msk* = (0x00000001 shl I2C_OAR1_OA1MODE_Pos) ## !< 0x00000400
  I2C_OAR1_OA1MODE* = I2C_OAR1_OA1MODE_Msk
  I2C_OAR1_OA1EN_Pos* = (15)
  I2C_OAR1_OA1EN_Msk* = (0x00000001 shl I2C_OAR1_OA1EN_Pos) ## !< 0x00008000
  I2C_OAR1_OA1EN* = I2C_OAR1_OA1EN_Msk

## ******************  Bit definition for I2C_OAR2 register  ******************

const
  I2C_OAR2_OA2_Pos* = (1)
  I2C_OAR2_OA2_Msk* = (0x0000007F shl I2C_OAR2_OA2_Pos) ## !< 0x000000FE
  I2C_OAR2_OA2* = I2C_OAR2_OA2_Msk
  I2C_OAR2_OA2MSK_Pos* = (8)
  I2C_OAR2_OA2MSK_Msk* = (0x00000007 shl I2C_OAR2_OA2MSK_Pos) ## !< 0x00000700
  I2C_OAR2_OA2MSKx* = I2C_OAR2_OA2MSK_Msk
  I2C_OAR2_OA2NOMASK* = (0x00000000) ## !< No mask
  I2C_OAR2_OA2MASK01_Pos* = (8)
  I2C_OAR2_OA2MASK01_Msk* = (0x00000001 shl I2C_OAR2_OA2MASK01_Pos) ## !< 0x00000100
  I2C_OAR2_OA2MASK01* = I2C_OAR2_OA2MASK01_Msk
  I2C_OAR2_OA2MASK02_Pos* = (9)
  I2C_OAR2_OA2MASK02_Msk* = (0x00000001 shl I2C_OAR2_OA2MASK02_Pos) ## !< 0x00000200
  I2C_OAR2_OA2MASK02* = I2C_OAR2_OA2MASK02_Msk
  I2C_OAR2_OA2MASK03_Pos* = (8)
  I2C_OAR2_OA2MASK03_Msk* = (0x00000003 shl I2C_OAR2_OA2MASK03_Pos) ## !< 0x00000300
  I2C_OAR2_OA2MASK03* = I2C_OAR2_OA2MASK03_Msk
  I2C_OAR2_OA2MASK04_Pos* = (10)
  I2C_OAR2_OA2MASK04_Msk* = (0x00000001 shl I2C_OAR2_OA2MASK04_Pos) ## !< 0x00000400
  I2C_OAR2_OA2MASK04* = I2C_OAR2_OA2MASK04_Msk
  I2C_OAR2_OA2MASK05_Pos* = (8)
  I2C_OAR2_OA2MASK05_Msk* = (0x00000005 shl I2C_OAR2_OA2MASK05_Pos) ## !< 0x00000500
  I2C_OAR2_OA2MASK05* = I2C_OAR2_OA2MASK05_Msk
  I2C_OAR2_OA2MASK06_Pos* = (9)
  I2C_OAR2_OA2MASK06_Msk* = (0x00000003 shl I2C_OAR2_OA2MASK06_Pos) ## !< 0x00000600
  I2C_OAR2_OA2MASK06* = I2C_OAR2_OA2MASK06_Msk
  I2C_OAR2_OA2MASK07_Pos* = (8)
  I2C_OAR2_OA2MASK07_Msk* = (0x00000007 shl I2C_OAR2_OA2MASK07_Pos) ## !< 0x00000700
  I2C_OAR2_OA2MASK07* = I2C_OAR2_OA2MASK07_Msk
  I2C_OAR2_OA2EN_Pos* = (15)
  I2C_OAR2_OA2EN_Msk* = (0x00000001 shl I2C_OAR2_OA2EN_Pos) ## !< 0x00008000
  I2C_OAR2_OA2EN* = I2C_OAR2_OA2EN_Msk

## ******************  Bit definition for I2C_TIMINGR register ****************

const
  I2C_TIMINGR_SCLL_Pos* = (0)
  I2C_TIMINGR_SCLL_Msk* = (0x000000FF shl I2C_TIMINGR_SCLL_Pos) ## !< 0x000000FF
  I2C_TIMINGR_SCLL* = I2C_TIMINGR_SCLL_Msk
  I2C_TIMINGR_SCLH_Pos* = (8)
  I2C_TIMINGR_SCLH_Msk* = (0x000000FF shl I2C_TIMINGR_SCLH_Pos) ## !< 0x0000FF00
  I2C_TIMINGR_SCLH* = I2C_TIMINGR_SCLH_Msk
  I2C_TIMINGR_SDADEL_Pos* = (16)
  I2C_TIMINGR_SDADEL_Msk* = (0x0000000F shl I2C_TIMINGR_SDADEL_Pos) ## !< 0x000F0000
  I2C_TIMINGR_SDADEL* = I2C_TIMINGR_SDADEL_Msk
  I2C_TIMINGR_SCLDEL_Pos* = (20)
  I2C_TIMINGR_SCLDEL_Msk* = (0x0000000F shl I2C_TIMINGR_SCLDEL_Pos) ## !< 0x00F00000
  I2C_TIMINGR_SCLDEL* = I2C_TIMINGR_SCLDEL_Msk
  I2C_TIMINGR_PRESC_Pos* = (28)
  I2C_TIMINGR_PRESC_Msk* = (0x0000000F shl I2C_TIMINGR_PRESC_Pos) ## !< 0xF0000000
  I2C_TIMINGR_PRESC* = I2C_TIMINGR_PRESC_Msk

## ****************** Bit definition for I2C_TIMEOUTR register ****************

const
  I2C_TIMEOUTR_TIMEOUTA_Pos* = (0)
  I2C_TIMEOUTR_TIMEOUTA_Msk* = (0x00000FFF shl I2C_TIMEOUTR_TIMEOUTA_Pos) ## !< 0x00000FFF
  I2C_TIMEOUTR_TIMEOUTA* = I2C_TIMEOUTR_TIMEOUTA_Msk
  I2C_TIMEOUTR_TIDLE_Pos* = (12)
  I2C_TIMEOUTR_TIDLE_Msk* = (0x00000001 shl I2C_TIMEOUTR_TIDLE_Pos) ## !< 0x00001000
  I2C_TIMEOUTR_TIDLE* = I2C_TIMEOUTR_TIDLE_Msk
  I2C_TIMEOUTR_TIMOUTEN_Pos* = (15)
  I2C_TIMEOUTR_TIMOUTEN_Msk* = (0x00000001 shl I2C_TIMEOUTR_TIMOUTEN_Pos) ## !< 0x00008000
  I2C_TIMEOUTR_TIMOUTEN* = I2C_TIMEOUTR_TIMOUTEN_Msk
  I2C_TIMEOUTR_TIMEOUTB_Pos* = (16)
  I2C_TIMEOUTR_TIMEOUTB_Msk* = (0x00000FFF shl I2C_TIMEOUTR_TIMEOUTB_Pos) ## !< 0x0FFF0000
  I2C_TIMEOUTR_TIMEOUTB* = I2C_TIMEOUTR_TIMEOUTB_Msk
  I2C_TIMEOUTR_TEXTEN_Pos* = (31)
  I2C_TIMEOUTR_TEXTEN_Msk* = (0x00000001 shl I2C_TIMEOUTR_TEXTEN_Pos) ## !< 0x80000000
  I2C_TIMEOUTR_TEXTEN* = I2C_TIMEOUTR_TEXTEN_Msk

## *****************  Bit definition for I2C_ISR register  ********************

const
  I2C_ISR_TXE_Pos* = (0)
  I2C_ISR_TXE_Msk* = (0x00000001 shl I2C_ISR_TXE_Pos) ## !< 0x00000001
  I2C_ISR_TXE* = I2C_ISR_TXE_Msk
  I2C_ISR_TXIS_Pos* = (1)
  I2C_ISR_TXIS_Msk* = (0x00000001 shl I2C_ISR_TXIS_Pos) ## !< 0x00000002
  I2C_ISR_TXIS* = I2C_ISR_TXIS_Msk
  I2C_ISR_RXNE_Pos* = (2)
  I2C_ISR_RXNE_Msk* = (0x00000001 shl I2C_ISR_RXNE_Pos) ## !< 0x00000004
  I2C_ISR_RXNE* = I2C_ISR_RXNE_Msk
  I2C_ISR_ADDR_Pos* = (3)
  I2C_ISR_ADDR_Msk* = (0x00000001 shl I2C_ISR_ADDR_Pos) ## !< 0x00000008
  I2C_ISR_ADDR* = I2C_ISR_ADDR_Msk
  I2C_ISR_NACKF_Pos* = (4)
  I2C_ISR_NACKF_Msk* = (0x00000001 shl I2C_ISR_NACKF_Pos) ## !< 0x00000010
  I2C_ISR_NACKF* = I2C_ISR_NACKF_Msk
  I2C_ISR_STOPF_Pos* = (5)
  I2C_ISR_STOPF_Msk* = (0x00000001 shl I2C_ISR_STOPF_Pos) ## !< 0x00000020
  I2C_ISR_STOPF* = I2C_ISR_STOPF_Msk
  I2C_ISR_TC_Pos* = (6)
  I2C_ISR_TC_Msk* = (0x00000001 shl I2C_ISR_TC_Pos) ## !< 0x00000040
  I2C_ISR_TC* = I2C_ISR_TC_Msk
  I2C_ISR_TCR_Pos* = (7)
  I2C_ISR_TCR_Msk* = (0x00000001 shl I2C_ISR_TCR_Pos) ## !< 0x00000080
  I2C_ISR_TCR* = I2C_ISR_TCR_Msk
  I2C_ISR_BERR_Pos* = (8)
  I2C_ISR_BERR_Msk* = (0x00000001 shl I2C_ISR_BERR_Pos) ## !< 0x00000100
  I2C_ISR_BERR* = I2C_ISR_BERR_Msk
  I2C_ISR_ARLO_Pos* = (9)
  I2C_ISR_ARLO_Msk* = (0x00000001 shl I2C_ISR_ARLO_Pos) ## !< 0x00000200
  I2C_ISR_ARLO* = I2C_ISR_ARLO_Msk
  I2C_ISR_OVR_Pos* = (10)
  I2C_ISR_OVR_Msk* = (0x00000001 shl I2C_ISR_OVR_Pos) ## !< 0x00000400
  I2C_ISR_OVR* = I2C_ISR_OVR_Msk
  I2C_ISR_PECERR_Pos* = (11)
  I2C_ISR_PECERR_Msk* = (0x00000001 shl I2C_ISR_PECERR_Pos) ## !< 0x00000800
  I2C_ISR_PECERR* = I2C_ISR_PECERR_Msk
  I2C_ISR_TIMEOUT_Pos* = (12)
  I2C_ISR_TIMEOUT_Msk* = (0x00000001 shl I2C_ISR_TIMEOUT_Pos) ## !< 0x00001000
  I2C_ISR_TIMEOUT* = I2C_ISR_TIMEOUT_Msk
  I2C_ISR_ALERT_Pos* = (13)
  I2C_ISR_ALERT_Msk* = (0x00000001 shl I2C_ISR_ALERT_Pos) ## !< 0x00002000
  I2C_ISR_ALERT* = I2C_ISR_ALERT_Msk
  I2C_ISR_BUSY_Pos* = (15)
  I2C_ISR_BUSY_Msk* = (0x00000001 shl I2C_ISR_BUSY_Pos) ## !< 0x00008000
  I2C_ISR_BUSY* = I2C_ISR_BUSY_Msk
  I2C_ISR_DIR_Pos* = (16)
  I2C_ISR_DIR_Msk* = (0x00000001 shl I2C_ISR_DIR_Pos) ## !< 0x00010000
  I2C_ISR_DIR* = I2C_ISR_DIR_Msk
  I2C_ISR_ADDCODE_Pos* = (17)
  I2C_ISR_ADDCODE_Msk* = (0x0000007F shl I2C_ISR_ADDCODE_Pos) ## !< 0x00FE0000
  I2C_ISR_ADDCODE* = I2C_ISR_ADDCODE_Msk

## *****************  Bit definition for I2C_ICR register  ********************

const
  I2C_ICR_ADDRCF_Pos* = (3)
  I2C_ICR_ADDRCF_Msk* = (0x00000001 shl I2C_ICR_ADDRCF_Pos) ## !< 0x00000008
  I2C_ICR_ADDRCF* = I2C_ICR_ADDRCF_Msk
  I2C_ICR_NACKCF_Pos* = (4)
  I2C_ICR_NACKCF_Msk* = (0x00000001 shl I2C_ICR_NACKCF_Pos) ## !< 0x00000010
  I2C_ICR_NACKCF* = I2C_ICR_NACKCF_Msk
  I2C_ICR_STOPCF_Pos* = (5)
  I2C_ICR_STOPCF_Msk* = (0x00000001 shl I2C_ICR_STOPCF_Pos) ## !< 0x00000020
  I2C_ICR_STOPCF* = I2C_ICR_STOPCF_Msk
  I2C_ICR_BERRCF_Pos* = (8)
  I2C_ICR_BERRCF_Msk* = (0x00000001 shl I2C_ICR_BERRCF_Pos) ## !< 0x00000100
  I2C_ICR_BERRCF* = I2C_ICR_BERRCF_Msk
  I2C_ICR_ARLOCF_Pos* = (9)
  I2C_ICR_ARLOCF_Msk* = (0x00000001 shl I2C_ICR_ARLOCF_Pos) ## !< 0x00000200
  I2C_ICR_ARLOCF* = I2C_ICR_ARLOCF_Msk
  I2C_ICR_OVRCF_Pos* = (10)
  I2C_ICR_OVRCF_Msk* = (0x00000001 shl I2C_ICR_OVRCF_Pos) ## !< 0x00000400
  I2C_ICR_OVRCF* = I2C_ICR_OVRCF_Msk
  I2C_ICR_PECCF_Pos* = (11)
  I2C_ICR_PECCF_Msk* = (0x00000001 shl I2C_ICR_PECCF_Pos) ## !< 0x00000800
  I2C_ICR_PECCF* = I2C_ICR_PECCF_Msk
  I2C_ICR_TIMOUTCF_Pos* = (12)
  I2C_ICR_TIMOUTCF_Msk* = (0x00000001 shl I2C_ICR_TIMOUTCF_Pos) ## !< 0x00001000
  I2C_ICR_TIMOUTCF* = I2C_ICR_TIMOUTCF_Msk
  I2C_ICR_ALERTCF_Pos* = (13)
  I2C_ICR_ALERTCF_Msk* = (0x00000001 shl I2C_ICR_ALERTCF_Pos) ## !< 0x00002000
  I2C_ICR_ALERTCF* = I2C_ICR_ALERTCF_Msk

## *****************  Bit definition for I2C_PECR register  *******************

const
  I2C_PECR_PEC_Pos* = (0)
  I2C_PECR_PEC_Msk* = (0x000000FF shl I2C_PECR_PEC_Pos) ## !< 0x000000FF
  I2C_PECR_PEC* = I2C_PECR_PEC_Msk

## *****************  Bit definition for I2C_RXDR register  ********************

const
  I2C_RXDR_RXDATA_Pos* = (0)
  I2C_RXDR_RXDATA_Msk* = (0x000000FF shl I2C_RXDR_RXDATA_Pos) ## !< 0x000000FF
  I2C_RXDR_RXDATA* = I2C_RXDR_RXDATA_Msk

## *****************  Bit definition for I2C_TXDR register  ********************

const
  I2C_TXDR_TXDATA_Pos* = (0)
  I2C_TXDR_TXDATA_Msk* = (0x000000FF shl I2C_TXDR_TXDATA_Pos) ## !< 0x000000FF
  I2C_TXDR_TXDATA* = I2C_TXDR_TXDATA_Msk

## ****************************************************************************
##
##                            Independent WATCHDOG (IWDG)
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
  IWDG_SR_WVU_Pos* = (2)
  IWDG_SR_WVU_Msk* = (0x00000001 shl IWDG_SR_WVU_Pos) ## !< 0x00000004
  IWDG_SR_WVU* = IWDG_SR_WVU_Msk

## ******************  Bit definition for IWDG_KR register  *******************

const
  IWDG_WINR_WIN_Pos* = (0)
  IWDG_WINR_WIN_Msk* = (0x00000FFF shl IWDG_WINR_WIN_Pos) ## !< 0x00000FFF
  IWDG_WINR_WIN* = IWDG_WINR_WIN_Msk

## ****************************************************************************
##
##                              Power Control
##
## ****************************************************************************

const
  PWR_PVD_SUPPORT* = true       ## !< PWR feature available only on specific devices: Power Voltage Detection feature

## *******************  Bit definition for PWR_CR register  *******************

const
  PWR_CR_LPDS_Pos* = (0)
  PWR_CR_LPDS_Msk* = (0x00000001 shl PWR_CR_LPDS_Pos) ## !< 0x00000001
  PWR_CR_LPDS* = PWR_CR_LPDS_Msk
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
##                          Reset and Clock Control
##
## ****************************************************************************
## *******************  Bit definition for RCC_CR register  *******************

const
  RCC_CR_HSION_Pos* = (0)
  RCC_CR_HSION_Msk* = (0x00000001 shl RCC_CR_HSION_Pos) ## !< 0x00000001
  RCC_CR_HSION* = RCC_CR_HSION_Msk
  RCC_CR_HSIRDY_Pos* = (1)
  RCC_CR_HSIRDY_Msk* = (0x00000001 shl RCC_CR_HSIRDY_Pos) ## !< 0x00000002
  RCC_CR_HSIRDY* = RCC_CR_HSIRDY_Msk
  RCC_CR_HSITRIM_Pos* = (3)
  RCC_CR_HSITRIM_Msk* = (0x0000001F shl RCC_CR_HSITRIM_Pos) ## !< 0x000000F8
  RCC_CR_HSITRIM* = RCC_CR_HSITRIM_Msk
  RCC_CR_HSITRIM_0* = (0x00000001 shl RCC_CR_HSITRIM_Pos) ## !< 0x00000008
  RCC_CR_HSITRIM_1* = (0x00000002 shl RCC_CR_HSITRIM_Pos) ## !< 0x00000010
  RCC_CR_HSITRIM_2* = (0x00000004 shl RCC_CR_HSITRIM_Pos) ## !< 0x00000020
  RCC_CR_HSITRIM_3* = (0x00000008 shl RCC_CR_HSITRIM_Pos) ## !< 0x00000040
  RCC_CR_HSITRIM_4* = (0x00000010 shl RCC_CR_HSITRIM_Pos) ## !< 0x00000080
  RCC_CR_HSICAL_Pos* = (8)
  RCC_CR_HSICAL_Msk* = (0x000000FF shl RCC_CR_HSICAL_Pos) ## !< 0x0000FF00
  RCC_CR_HSICAL* = RCC_CR_HSICAL_Msk
  RCC_CR_HSICAL_0* = (0x00000001 shl RCC_CR_HSICAL_Pos) ## !< 0x00000100
  RCC_CR_HSICAL_1* = (0x00000002 shl RCC_CR_HSICAL_Pos) ## !< 0x00000200
  RCC_CR_HSICAL_2* = (0x00000004 shl RCC_CR_HSICAL_Pos) ## !< 0x00000400
  RCC_CR_HSICAL_3* = (0x00000008 shl RCC_CR_HSICAL_Pos) ## !< 0x00000800
  RCC_CR_HSICAL_4* = (0x00000010 shl RCC_CR_HSICAL_Pos) ## !< 0x00001000
  RCC_CR_HSICAL_5* = (0x00000020 shl RCC_CR_HSICAL_Pos) ## !< 0x00002000
  RCC_CR_HSICAL_6* = (0x00000040 shl RCC_CR_HSICAL_Pos) ## !< 0x00004000
  RCC_CR_HSICAL_7* = (0x00000080 shl RCC_CR_HSICAL_Pos) ## !< 0x00008000
  RCC_CR_HSEON_Pos* = (16)
  RCC_CR_HSEON_Msk* = (0x00000001 shl RCC_CR_HSEON_Pos) ## !< 0x00010000
  RCC_CR_HSEON* = RCC_CR_HSEON_Msk
  RCC_CR_HSERDY_Pos* = (17)
  RCC_CR_HSERDY_Msk* = (0x00000001 shl RCC_CR_HSERDY_Pos) ## !< 0x00020000
  RCC_CR_HSERDY* = RCC_CR_HSERDY_Msk
  RCC_CR_HSEBYP_Pos* = (18)
  RCC_CR_HSEBYP_Msk* = (0x00000001 shl RCC_CR_HSEBYP_Pos) ## !< 0x00040000
  RCC_CR_HSEBYP* = RCC_CR_HSEBYP_Msk
  RCC_CR_CSSON_Pos* = (19)
  RCC_CR_CSSON_Msk* = (0x00000001 shl RCC_CR_CSSON_Pos) ## !< 0x00080000
  RCC_CR_CSSON* = RCC_CR_CSSON_Msk
  RCC_CR_PLLON_Pos* = (24)
  RCC_CR_PLLON_Msk* = (0x00000001 shl RCC_CR_PLLON_Pos) ## !< 0x01000000
  RCC_CR_PLLON* = RCC_CR_PLLON_Msk
  RCC_CR_PLLRDY_Pos* = (25)
  RCC_CR_PLLRDY_Msk* = (0x00000001 shl RCC_CR_PLLRDY_Pos) ## !< 0x02000000
  RCC_CR_PLLRDY* = RCC_CR_PLLRDY_Msk

## *******************  Bit definition for RCC_CFGR register  *****************
## !< SW configuration

const
  RCC_CFGR_SW_Pos* = (0)
  RCC_CFGR_SW_Msk* = (0x00000003 shl RCC_CFGR_SW_Pos) ## !< 0x00000003
  RCC_CFGR_SW* = RCC_CFGR_SW_Msk
  RCC_CFGR_SW_0* = (0x00000001 shl RCC_CFGR_SW_Pos) ## !< 0x00000001
  RCC_CFGR_SW_1* = (0x00000002 shl RCC_CFGR_SW_Pos) ## !< 0x00000002
  RCC_CFGR_SW_HSI* = (0x00000000) ## !< HSI selected as system clock
  RCC_CFGR_SW_HSE* = (0x00000001) ## !< HSE selected as system clock
  RCC_CFGR_SW_PLL* = (0x00000002) ## !< PLL selected as system clock

## !< SWS configuration

const
  RCC_CFGR_SWS_Pos* = (2)
  RCC_CFGR_SWS_Msk* = (0x00000003 shl RCC_CFGR_SWS_Pos) ## !< 0x0000000C
  RCC_CFGR_SWS* = RCC_CFGR_SWS_Msk
  RCC_CFGR_SWS_0* = (0x00000001 shl RCC_CFGR_SWS_Pos) ## !< 0x00000004
  RCC_CFGR_SWS_1* = (0x00000002 shl RCC_CFGR_SWS_Pos) ## !< 0x00000008
  RCC_CFGR_SWS_HSI* = (0x00000000) ## !< HSI oscillator used as system clock
  RCC_CFGR_SWS_HSE* = (0x00000004) ## !< HSE oscillator used as system clock
  RCC_CFGR_SWS_PLL* = (0x00000008) ## !< PLL used as system clock

## !< HPRE configuration

const
  RCC_CFGR_HPRE_Pos* = (4)
  RCC_CFGR_HPRE_Msk* = (0x0000000F shl RCC_CFGR_HPRE_Pos) ## !< 0x000000F0
  RCC_CFGR_HPRE* = RCC_CFGR_HPRE_Msk
  RCC_CFGR_HPRE_0* = (0x00000001 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000010
  RCC_CFGR_HPRE_1* = (0x00000002 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000020
  RCC_CFGR_HPRE_2* = (0x00000004 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000040
  RCC_CFGR_HPRE_3* = (0x00000008 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000080
  RCC_CFGR_HPRE_DIV1* = (0x00000000) ## !< SYSCLK not divided
  RCC_CFGR_HPRE_DIV2* = (0x00000080) ## !< SYSCLK divided by 2
  RCC_CFGR_HPRE_DIV4* = (0x00000090) ## !< SYSCLK divided by 4
  RCC_CFGR_HPRE_DIV8* = (0x000000A0) ## !< SYSCLK divided by 8
  RCC_CFGR_HPRE_DIV16* = (0x000000B0) ## !< SYSCLK divided by 16
  RCC_CFGR_HPRE_DIV64* = (0x000000C0) ## !< SYSCLK divided by 64
  RCC_CFGR_HPRE_DIV128* = (0x000000D0) ## !< SYSCLK divided by 128
  RCC_CFGR_HPRE_DIV256* = (0x000000E0) ## !< SYSCLK divided by 256
  RCC_CFGR_HPRE_DIV512* = (0x000000F0) ## !< SYSCLK divided by 512

## !< PPRE1 configuration

const
  RCC_CFGR_PPRE1_Pos* = (8)
  RCC_CFGR_PPRE1_Msk* = (0x00000007 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00000700
  RCC_CFGR_PPRE1* = RCC_CFGR_PPRE1_Msk
  RCC_CFGR_PPRE1_0* = (0x00000001 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00000100
  RCC_CFGR_PPRE1_1* = (0x00000002 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00000200
  RCC_CFGR_PPRE1_2* = (0x00000004 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00000400
  RCC_CFGR_PPRE1_DIV1* = (0x00000000) ## !< HCLK not divided
  RCC_CFGR_PPRE1_DIV2* = (0x00000400) ## !< HCLK divided by 2
  RCC_CFGR_PPRE1_DIV4* = (0x00000500) ## !< HCLK divided by 4
  RCC_CFGR_PPRE1_DIV8* = (0x00000600) ## !< HCLK divided by 8
  RCC_CFGR_PPRE1_DIV16* = (0x00000700) ## !< HCLK divided by 16

## !< PPRE2 configuration

const
  RCC_CFGR_PPRE2_Pos* = (11)
  RCC_CFGR_PPRE2_Msk* = (0x00000007 shl RCC_CFGR_PPRE2_Pos) ## !< 0x00003800
  RCC_CFGR_PPRE2* = RCC_CFGR_PPRE2_Msk
  RCC_CFGR_PPRE2_0* = (0x00000001 shl RCC_CFGR_PPRE2_Pos) ## !< 0x00000800
  RCC_CFGR_PPRE2_1* = (0x00000002 shl RCC_CFGR_PPRE2_Pos) ## !< 0x00001000
  RCC_CFGR_PPRE2_2* = (0x00000004 shl RCC_CFGR_PPRE2_Pos) ## !< 0x00002000
  RCC_CFGR_PPRE2_DIV1* = (0x00000000) ## !< HCLK not divided
  RCC_CFGR_PPRE2_DIV2* = (0x00002000) ## !< HCLK divided by 2
  RCC_CFGR_PPRE2_DIV4* = (0x00002800) ## !< HCLK divided by 4
  RCC_CFGR_PPRE2_DIV8* = (0x00003000) ## !< HCLK divided by 8
  RCC_CFGR_PPRE2_DIV16* = (0x00003800) ## !< HCLK divided by 16
  RCC_CFGR_PLLSRC_Pos* = (16)
  RCC_CFGR_PLLSRC_Msk* = (0x00000001 shl RCC_CFGR_PLLSRC_Pos) ## !< 0x00010000
  RCC_CFGR_PLLSRC* = RCC_CFGR_PLLSRC_Msk
  RCC_CFGR_PLLSRC_HSI_DIV2* = (0x00000000) ## !< HSI clock divided by 2 selected as PLL entry clock source
  RCC_CFGR_PLLSRC_HSE_PREDIV* = (0x00010000) ## !< HSE/PREDIV clock selected as PLL entry clock source
  RCC_CFGR_PLLXTPRE_Pos* = (17)
  RCC_CFGR_PLLXTPRE_Msk* = (0x00000001 shl RCC_CFGR_PLLXTPRE_Pos) ## !< 0x00020000
  RCC_CFGR_PLLXTPRE* = RCC_CFGR_PLLXTPRE_Msk
  RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV1* = (0x00000000) ## !< HSE/PREDIV clock not divided for PLL entry
  RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV2* = (0x00020000) ## !< HSE/PREDIV clock divided by 2 for PLL entry

## !< PLLMUL configuration

const
  RCC_CFGR_PLLMUL_Pos* = (18)
  RCC_CFGR_PLLMUL_Msk* = (0x0000000F shl RCC_CFGR_PLLMUL_Pos) ## !< 0x003C0000
  RCC_CFGR_PLLMUL* = RCC_CFGR_PLLMUL_Msk
  RCC_CFGR_PLLMUL_0* = (0x00000001 shl RCC_CFGR_PLLMUL_Pos) ## !< 0x00040000
  RCC_CFGR_PLLMUL_1* = (0x00000002 shl RCC_CFGR_PLLMUL_Pos) ## !< 0x00080000
  RCC_CFGR_PLLMUL_2* = (0x00000004 shl RCC_CFGR_PLLMUL_Pos) ## !< 0x00100000
  RCC_CFGR_PLLMUL_3* = (0x00000008 shl RCC_CFGR_PLLMUL_Pos) ## !< 0x00200000
  RCC_CFGR_PLLMUL2x* = (0x00000000) ## !< PLL input clock*2
  RCC_CFGR_PLLMUL3x* = (0x00040000) ## !< PLL input clock*3
  RCC_CFGR_PLLMUL4* = (0x00080000) ## !< PLL input clock*4
  RCC_CFGR_PLLMUL5* = (0x000C0000) ## !< PLL input clock*5
  RCC_CFGR_PLLMUL6* = (0x00100000) ## !< PLL input clock*6
  RCC_CFGR_PLLMUL7* = (0x00140000) ## !< PLL input clock*7
  RCC_CFGR_PLLMUL8* = (0x00180000) ## !< PLL input clock*8
  RCC_CFGR_PLLMUL9* = (0x001C0000) ## !< PLL input clock*9
  RCC_CFGR_PLLMUL10* = (0x00200000) ## !< PLL input clock10
  RCC_CFGR_PLLMUL11* = (0x00240000) ## !< PLL input clock*11
  RCC_CFGR_PLLMUL12* = (0x00280000) ## !< PLL input clock*12
  RCC_CFGR_PLLMUL13* = (0x002C0000) ## !< PLL input clock*13
  RCC_CFGR_PLLMUL14* = (0x00300000) ## !< PLL input clock*14
  RCC_CFGR_PLLMUL15* = (0x00340000) ## !< PLL input clock*15
  RCC_CFGR_PLLMUL16* = (0x00380000) ## !< PLL input clock*16

## !< USB configuration

const
  RCC_CFGR_USBPRE_Pos* = (22)
  RCC_CFGR_USBPRE_Msk* = (0x00000001 shl RCC_CFGR_USBPRE_Pos) ## !< 0x00400000
  RCC_CFGR_USBPRE* = RCC_CFGR_USBPRE_Msk
  RCC_CFGR_USBPRE_DIV1_5* = (0x00000000) ## !< USB prescaler is PLL clock divided by 1.5
  RCC_CFGR_USBPRE_DIV1* = (0x00400000) ## !< USB prescaler is PLL clock divided by 1

## !< I2S configuration

const
  RCC_CFGR_I2SSRC_Pos* = (23)
  RCC_CFGR_I2SSRC_Msk* = (0x00000001 shl RCC_CFGR_I2SSRC_Pos) ## !< 0x00800000
  RCC_CFGR_I2SSRC* = RCC_CFGR_I2SSRC_Msk
  RCC_CFGR_I2SSRC_SYSCLK* = (0x00000000) ## !< System clock selected as I2S clock source
  RCC_CFGR_I2SSRC_EXT* = (0x00800000) ## !< External clock selected as I2S clock source

## !< MCO configuration

const
  RCC_CFGR_MCO_Pos* = (24)
  RCC_CFGR_MCO_Msk* = (0x00000007 shl RCC_CFGR_MCO_Pos) ## !< 0x07000000
  RCC_CFGR_MCO* = RCC_CFGR_MCO_Msk
  RCC_CFGR_MCO_0* = (0x00000001 shl RCC_CFGR_MCO_Pos) ## !< 0x01000000
  RCC_CFGR_MCO_1* = (0x00000002 shl RCC_CFGR_MCO_Pos) ## !< 0x02000000
  RCC_CFGR_MCO_2* = (0x00000004 shl RCC_CFGR_MCO_Pos) ## !< 0x04000000
  RCC_CFGR_MCO_NOCLOCK* = (0x00000000) ## !< No clock
  RCC_CFGR_MCO_LSI* = (0x02000000) ## !< LSI clock selected as MCO source
  RCC_CFGR_MCO_LSE* = (0x03000000) ## !< LSE clock selected as MCO source
  RCC_CFGR_MCO_SYSCLK* = (0x04000000) ## !< System clock selected as MCO source
  RCC_CFGR_MCO_HSI* = (0x05000000) ## !< HSI clock selected as MCO source
  RCC_CFGR_MCO_HSE* = (0x06000000) ## !< HSE clock selected as MCO source
  RCC_CFGR_MCO_PLL* = (0x07000000) ## !< PLL clock divided by 2 selected as MCO source
  RCC_CFGR_MCOF_Pos* = (28)
  RCC_CFGR_MCOF_Msk* = (0x00000001 shl RCC_CFGR_MCOF_Pos) ## !< 0x10000000
  RCC_CFGR_MCOF* = RCC_CFGR_MCOF_Msk

##  Reference defines

const
  RCC_CFGR_MCOSEL* = RCC_CFGR_MCO
  RCC_CFGR_MCOSEL_0* = RCC_CFGR_MCO_0
  RCC_CFGR_MCOSEL_1* = RCC_CFGR_MCO_1
  RCC_CFGR_MCOSEL_2* = RCC_CFGR_MCO_2
  RCC_CFGR_MCOSEL_NOCLOCK* = RCC_CFGR_MCO_NOCLOCK
  RCC_CFGR_MCOSEL_LSI* = RCC_CFGR_MCO_LSI
  RCC_CFGR_MCOSEL_LSE* = RCC_CFGR_MCO_LSE
  RCC_CFGR_MCOSEL_SYSCLK* = RCC_CFGR_MCO_SYSCLK
  RCC_CFGR_MCOSEL_HSI* = RCC_CFGR_MCO_HSI
  RCC_CFGR_MCOSEL_HSE* = RCC_CFGR_MCO_HSE
  RCC_CFGR_MCOSEL_PLL_DIV2* = RCC_CFGR_MCO_PLL

## ********************  Bit definition for RCC_CIR register  *******************

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
  RCC_CIR_CSSC_Pos* = (23)
  RCC_CIR_CSSC_Msk* = (0x00000001 shl RCC_CIR_CSSC_Pos) ## !< 0x00800000
  RCC_CIR_CSSC* = RCC_CIR_CSSC_Msk

## *****************  Bit definition for RCC_APB2RSTR register  ****************

const
  RCC_APB2RSTR_SYSCFGRST_Pos* = (0)
  RCC_APB2RSTR_SYSCFGRST_Msk* = (0x00000001 shl RCC_APB2RSTR_SYSCFGRST_Pos) ## !< 0x00000001
  RCC_APB2RSTR_SYSCFGRST* = RCC_APB2RSTR_SYSCFGRST_Msk
  RCC_APB2RSTR_TIM1RST_Pos* = (11)
  RCC_APB2RSTR_TIM1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM1RST_Pos) ## !< 0x00000800
  RCC_APB2RSTR_TIM1RST* = RCC_APB2RSTR_TIM1RST_Msk
  RCC_APB2RSTR_SPI1RST_Pos* = (12)
  RCC_APB2RSTR_SPI1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_SPI1RST_Pos) ## !< 0x00001000
  RCC_APB2RSTR_SPI1RST* = RCC_APB2RSTR_SPI1RST_Msk
  RCC_APB2RSTR_TIM8RST_Pos* = (13)
  RCC_APB2RSTR_TIM8RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM8RST_Pos) ## !< 0x00002000
  RCC_APB2RSTR_TIM8RST* = RCC_APB2RSTR_TIM8RST_Msk
  RCC_APB2RSTR_USART1RST_Pos* = (14)
  RCC_APB2RSTR_USART1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_USART1RST_Pos) ## !< 0x00004000
  RCC_APB2RSTR_USART1RST* = RCC_APB2RSTR_USART1RST_Msk
  RCC_APB2RSTR_TIM15RST_Pos* = (16)
  RCC_APB2RSTR_TIM15RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM15RST_Pos) ## !< 0x00010000
  RCC_APB2RSTR_TIM15RST* = RCC_APB2RSTR_TIM15RST_Msk
  RCC_APB2RSTR_TIM16RST_Pos* = (17)
  RCC_APB2RSTR_TIM16RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM16RST_Pos) ## !< 0x00020000
  RCC_APB2RSTR_TIM16RST* = RCC_APB2RSTR_TIM16RST_Msk
  RCC_APB2RSTR_TIM17RST_Pos* = (18)
  RCC_APB2RSTR_TIM17RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM17RST_Pos) ## !< 0x00040000
  RCC_APB2RSTR_TIM17RST* = RCC_APB2RSTR_TIM17RST_Msk

## *****************  Bit definition for RCC_APB1RSTR register  *****************

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
  RCC_APB1RSTR_TIM6RST_Pos* = (4)
  RCC_APB1RSTR_TIM6RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM6RST_Pos) ## !< 0x00000010
  RCC_APB1RSTR_TIM6RST* = RCC_APB1RSTR_TIM6RST_Msk
  RCC_APB1RSTR_TIM7RST_Pos* = (5)
  RCC_APB1RSTR_TIM7RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM7RST_Pos) ## !< 0x00000020
  RCC_APB1RSTR_TIM7RST* = RCC_APB1RSTR_TIM7RST_Msk
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
  RCC_APB1RSTR_CANRST_Pos* = (25)
  RCC_APB1RSTR_CANRST_Msk* = (0x00000001 shl RCC_APB1RSTR_CANRST_Pos) ## !< 0x02000000
  RCC_APB1RSTR_CANRST* = RCC_APB1RSTR_CANRST_Msk
  RCC_APB1RSTR_PWRRST_Pos* = (28)
  RCC_APB1RSTR_PWRRST_Msk* = (0x00000001 shl RCC_APB1RSTR_PWRRST_Pos) ## !< 0x10000000
  RCC_APB1RSTR_PWRRST* = RCC_APB1RSTR_PWRRST_Msk
  RCC_APB1RSTR_DAC1RST_Pos* = (29)
  RCC_APB1RSTR_DAC1RST_Msk* = (0x00000001 shl RCC_APB1RSTR_DAC1RST_Pos) ## !< 0x20000000
  RCC_APB1RSTR_DAC1RST* = RCC_APB1RSTR_DAC1RST_Msk

## *****************  Bit definition for RCC_AHBENR register  *****************

const
  RCC_AHBENR_DMA1EN_Pos* = (0)
  RCC_AHBENR_DMA1EN_Msk* = (0x00000001 shl RCC_AHBENR_DMA1EN_Pos) ## !< 0x00000001
  RCC_AHBENR_DMA1EN* = RCC_AHBENR_DMA1EN_Msk
  RCC_AHBENR_DMA2EN_Pos* = (1)
  RCC_AHBENR_DMA2EN_Msk* = (0x00000001 shl RCC_AHBENR_DMA2EN_Pos) ## !< 0x00000002
  RCC_AHBENR_DMA2EN* = RCC_AHBENR_DMA2EN_Msk
  RCC_AHBENR_SRAMEN_Pos* = (2)
  RCC_AHBENR_SRAMEN_Msk* = (0x00000001 shl RCC_AHBENR_SRAMEN_Pos) ## !< 0x00000004
  RCC_AHBENR_SRAMEN* = RCC_AHBENR_SRAMEN_Msk
  RCC_AHBENR_FLITFEN_Pos* = (4)
  RCC_AHBENR_FLITFEN_Msk* = (0x00000001 shl RCC_AHBENR_FLITFEN_Pos) ## !< 0x00000010
  RCC_AHBENR_FLITFEN* = RCC_AHBENR_FLITFEN_Msk
  RCC_AHBENR_CRCEN_Pos* = (6)
  RCC_AHBENR_CRCEN_Msk* = (0x00000001 shl RCC_AHBENR_CRCEN_Pos) ## !< 0x00000040
  RCC_AHBENR_CRCEN* = RCC_AHBENR_CRCEN_Msk
  RCC_AHBENR_GPIOAEN_Pos* = (17)
  RCC_AHBENR_GPIOAEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOAEN_Pos) ## !< 0x00020000
  RCC_AHBENR_GPIOAEN* = RCC_AHBENR_GPIOAEN_Msk
  RCC_AHBENR_GPIOBEN_Pos* = (18)
  RCC_AHBENR_GPIOBEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOBEN_Pos) ## !< 0x00040000
  RCC_AHBENR_GPIOBEN* = RCC_AHBENR_GPIOBEN_Msk
  RCC_AHBENR_GPIOCEN_Pos* = (19)
  RCC_AHBENR_GPIOCEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOCEN_Pos) ## !< 0x00080000
  RCC_AHBENR_GPIOCEN* = RCC_AHBENR_GPIOCEN_Msk
  RCC_AHBENR_GPIODEN_Pos* = (20)
  RCC_AHBENR_GPIODEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIODEN_Pos) ## !< 0x00100000
  RCC_AHBENR_GPIODEN* = RCC_AHBENR_GPIODEN_Msk
  RCC_AHBENR_GPIOEEN_Pos* = (21)
  RCC_AHBENR_GPIOEEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOEEN_Pos) ## !< 0x00200000
  RCC_AHBENR_GPIOEEN* = RCC_AHBENR_GPIOEEN_Msk
  RCC_AHBENR_GPIOFEN_Pos* = (22)
  RCC_AHBENR_GPIOFEN_Msk* = (0x00000001 shl RCC_AHBENR_GPIOFEN_Pos) ## !< 0x00400000
  RCC_AHBENR_GPIOFEN* = RCC_AHBENR_GPIOFEN_Msk
  RCC_AHBENR_TSCEN_Pos* = (24)
  RCC_AHBENR_TSCEN_Msk* = (0x00000001 shl RCC_AHBENR_TSCEN_Pos) ## !< 0x01000000
  RCC_AHBENR_TSCEN* = RCC_AHBENR_TSCEN_Msk
  RCC_AHBENR_ADC12EN_Pos* = (28)
  RCC_AHBENR_ADC12EN_Msk* = (0x00000001 shl RCC_AHBENR_ADC12EN_Pos) ## !< 0x10000000
  RCC_AHBENR_ADC12EN* = RCC_AHBENR_ADC12EN_Msk
  RCC_AHBENR_ADC34EN_Pos* = (29)
  RCC_AHBENR_ADC34EN_Msk* = (0x00000001 shl RCC_AHBENR_ADC34EN_Pos) ## !< 0x20000000
  RCC_AHBENR_ADC34EN* = RCC_AHBENR_ADC34EN_Msk

## ****************  Bit definition for RCC_APB2ENR register  *****************

const
  RCC_APB2ENR_SYSCFGEN_Pos* = (0)
  RCC_APB2ENR_SYSCFGEN_Msk* = (0x00000001 shl RCC_APB2ENR_SYSCFGEN_Pos) ## !< 0x00000001
  RCC_APB2ENR_SYSCFGEN* = RCC_APB2ENR_SYSCFGEN_Msk
  RCC_APB2ENR_TIM1EN_Pos* = (11)
  RCC_APB2ENR_TIM1EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM1EN_Pos) ## !< 0x00000800
  RCC_APB2ENR_TIM1EN* = RCC_APB2ENR_TIM1EN_Msk
  RCC_APB2ENR_SPI1EN_Pos* = (12)
  RCC_APB2ENR_SPI1EN_Msk* = (0x00000001 shl RCC_APB2ENR_SPI1EN_Pos) ## !< 0x00001000
  RCC_APB2ENR_SPI1EN* = RCC_APB2ENR_SPI1EN_Msk
  RCC_APB2ENR_TIM8EN_Pos* = (13)
  RCC_APB2ENR_TIM8EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM8EN_Pos) ## !< 0x00002000
  RCC_APB2ENR_TIM8EN* = RCC_APB2ENR_TIM8EN_Msk
  RCC_APB2ENR_USART1EN_Pos* = (14)
  RCC_APB2ENR_USART1EN_Msk* = (0x00000001 shl RCC_APB2ENR_USART1EN_Pos) ## !< 0x00004000
  RCC_APB2ENR_USART1EN* = RCC_APB2ENR_USART1EN_Msk
  RCC_APB2ENR_TIM15EN_Pos* = (16)
  RCC_APB2ENR_TIM15EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM15EN_Pos) ## !< 0x00010000
  RCC_APB2ENR_TIM15EN* = RCC_APB2ENR_TIM15EN_Msk
  RCC_APB2ENR_TIM16EN_Pos* = (17)
  RCC_APB2ENR_TIM16EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM16EN_Pos) ## !< 0x00020000
  RCC_APB2ENR_TIM16EN* = RCC_APB2ENR_TIM16EN_Msk
  RCC_APB2ENR_TIM17EN_Pos* = (18)
  RCC_APB2ENR_TIM17EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM17EN_Pos) ## !< 0x00040000
  RCC_APB2ENR_TIM17EN* = RCC_APB2ENR_TIM17EN_Msk

## *****************  Bit definition for RCC_APB1ENR register  *****************

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
  RCC_APB1ENR_TIM6EN_Pos* = (4)
  RCC_APB1ENR_TIM6EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM6EN_Pos) ## !< 0x00000010
  RCC_APB1ENR_TIM6EN* = RCC_APB1ENR_TIM6EN_Msk
  RCC_APB1ENR_TIM7EN_Pos* = (5)
  RCC_APB1ENR_TIM7EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM7EN_Pos) ## !< 0x00000020
  RCC_APB1ENR_TIM7EN* = RCC_APB1ENR_TIM7EN_Msk
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
  RCC_APB1ENR_CANEN_Pos* = (25)
  RCC_APB1ENR_CANEN_Msk* = (0x00000001 shl RCC_APB1ENR_CANEN_Pos) ## !< 0x02000000
  RCC_APB1ENR_CANEN* = RCC_APB1ENR_CANEN_Msk
  RCC_APB1ENR_PWREN_Pos* = (28)
  RCC_APB1ENR_PWREN_Msk* = (0x00000001 shl RCC_APB1ENR_PWREN_Pos) ## !< 0x10000000
  RCC_APB1ENR_PWREN* = RCC_APB1ENR_PWREN_Msk
  RCC_APB1ENR_DAC1EN_Pos* = (29)
  RCC_APB1ENR_DAC1EN_Msk* = (0x00000001 shl RCC_APB1ENR_DAC1EN_Pos) ## !< 0x20000000
  RCC_APB1ENR_DAC1EN* = RCC_APB1ENR_DAC1EN_Msk

## *******************  Bit definition for RCC_BDCR register  *****************

const
  RCC_BDCR_LSE_Pos* = (0)
  RCC_BDCR_LSE_Msk* = (0x00000007 shl RCC_BDCR_LSE_Pos) ## !< 0x00000007
  RCC_BDCR_LSE* = RCC_BDCR_LSE_Msk
  RCC_BDCR_LSEON_Pos* = (0)
  RCC_BDCR_LSEON_Msk* = (0x00000001 shl RCC_BDCR_LSEON_Pos) ## !< 0x00000001
  RCC_BDCR_LSEON* = RCC_BDCR_LSEON_Msk
  RCC_BDCR_LSERDY_Pos* = (1)
  RCC_BDCR_LSERDY_Msk* = (0x00000001 shl RCC_BDCR_LSERDY_Pos) ## !< 0x00000002
  RCC_BDCR_LSERDY* = RCC_BDCR_LSERDY_Msk
  RCC_BDCR_LSEBYP_Pos* = (2)
  RCC_BDCR_LSEBYP_Msk* = (0x00000001 shl RCC_BDCR_LSEBYP_Pos) ## !< 0x00000004
  RCC_BDCR_LSEBYP* = RCC_BDCR_LSEBYP_Msk
  RCC_BDCR_LSEDRV_Pos* = (3)
  RCC_BDCR_LSEDRV_Msk* = (0x00000003 shl RCC_BDCR_LSEDRV_Pos) ## !< 0x00000018
  RCC_BDCR_LSEDRV* = RCC_BDCR_LSEDRV_Msk
  RCC_BDCR_LSEDRV_0* = (0x00000001 shl RCC_BDCR_LSEDRV_Pos) ## !< 0x00000008
  RCC_BDCR_LSEDRV_1* = (0x00000002 shl RCC_BDCR_LSEDRV_Pos) ## !< 0x00000010
  RCC_BDCR_RTCSEL_Pos* = (8)
  RCC_BDCR_RTCSEL_Msk* = (0x00000003 shl RCC_BDCR_RTCSEL_Pos) ## !< 0x00000300
  RCC_BDCR_RTCSEL* = RCC_BDCR_RTCSEL_Msk
  RCC_BDCR_RTCSEL_0* = (0x00000001 shl RCC_BDCR_RTCSEL_Pos) ## !< 0x00000100
  RCC_BDCR_RTCSEL_1* = (0x00000002 shl RCC_BDCR_RTCSEL_Pos) ## !< 0x00000200

## !< RTC configuration

const
  RCC_BDCR_RTCSEL_NOCLOCK* = (0x00000000) ## !< No clock
  RCC_BDCR_RTCSEL_LSE* = (0x00000100) ## !< LSE oscillator clock used as RTC clock
  RCC_BDCR_RTCSEL_LSI* = (0x00000200) ## !< LSI oscillator clock used as RTC clock
  RCC_BDCR_RTCSEL_HSE* = (0x00000300) ## !< HSE oscillator clock divided by 32 used as RTC clock
  RCC_BDCR_RTCEN_Pos* = (15)
  RCC_BDCR_RTCEN_Msk* = (0x00000001 shl RCC_BDCR_RTCEN_Pos) ## !< 0x00008000
  RCC_BDCR_RTCEN* = RCC_BDCR_RTCEN_Msk
  RCC_BDCR_BDRST_Pos* = (16)
  RCC_BDCR_BDRST_Msk* = (0x00000001 shl RCC_BDCR_BDRST_Pos) ## !< 0x00010000
  RCC_BDCR_BDRST* = RCC_BDCR_BDRST_Msk

## *******************  Bit definition for RCC_CSR register  ******************

const
  RCC_CSR_LSION_Pos* = (0)
  RCC_CSR_LSION_Msk* = (0x00000001 shl RCC_CSR_LSION_Pos) ## !< 0x00000001
  RCC_CSR_LSION* = RCC_CSR_LSION_Msk
  RCC_CSR_LSIRDY_Pos* = (1)
  RCC_CSR_LSIRDY_Msk* = (0x00000001 shl RCC_CSR_LSIRDY_Pos) ## !< 0x00000002
  RCC_CSR_LSIRDY* = RCC_CSR_LSIRDY_Msk
  RCC_CSR_V18PWRRSTF_Pos* = (23)
  RCC_CSR_V18PWRRSTF_Msk* = (0x00000001 shl RCC_CSR_V18PWRRSTF_Pos) ## !< 0x00800000
  RCC_CSR_V18PWRRSTF* = RCC_CSR_V18PWRRSTF_Msk
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

##  Legacy defines

const
  RCC_CSR_VREGRSTF* = RCC_CSR_V18PWRRSTF

## ******************  Bit definition for RCC_AHBRSTR register  ***************

const
  RCC_AHBRSTR_GPIOARST_Pos* = (17)
  RCC_AHBRSTR_GPIOARST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOARST_Pos) ## !< 0x00020000
  RCC_AHBRSTR_GPIOARST* = RCC_AHBRSTR_GPIOARST_Msk
  RCC_AHBRSTR_GPIOBRST_Pos* = (18)
  RCC_AHBRSTR_GPIOBRST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOBRST_Pos) ## !< 0x00040000
  RCC_AHBRSTR_GPIOBRST* = RCC_AHBRSTR_GPIOBRST_Msk
  RCC_AHBRSTR_GPIOCRST_Pos* = (19)
  RCC_AHBRSTR_GPIOCRST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOCRST_Pos) ## !< 0x00080000
  RCC_AHBRSTR_GPIOCRST* = RCC_AHBRSTR_GPIOCRST_Msk
  RCC_AHBRSTR_GPIODRST_Pos* = (20)
  RCC_AHBRSTR_GPIODRST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIODRST_Pos) ## !< 0x00100000
  RCC_AHBRSTR_GPIODRST* = RCC_AHBRSTR_GPIODRST_Msk
  RCC_AHBRSTR_GPIOERST_Pos* = (21)
  RCC_AHBRSTR_GPIOERST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOERST_Pos) ## !< 0x00200000
  RCC_AHBRSTR_GPIOERST* = RCC_AHBRSTR_GPIOERST_Msk
  RCC_AHBRSTR_GPIOFRST_Pos* = (22)
  RCC_AHBRSTR_GPIOFRST_Msk* = (0x00000001 shl RCC_AHBRSTR_GPIOFRST_Pos) ## !< 0x00400000
  RCC_AHBRSTR_GPIOFRST* = RCC_AHBRSTR_GPIOFRST_Msk
  RCC_AHBRSTR_TSCRST_Pos* = (24)
  RCC_AHBRSTR_TSCRST_Msk* = (0x00000001 shl RCC_AHBRSTR_TSCRST_Pos) ## !< 0x01000000
  RCC_AHBRSTR_TSCRST* = RCC_AHBRSTR_TSCRST_Msk
  RCC_AHBRSTR_ADC12RST_Pos* = (28)
  RCC_AHBRSTR_ADC12RST_Msk* = (0x00000001 shl RCC_AHBRSTR_ADC12RST_Pos) ## !< 0x10000000
  RCC_AHBRSTR_ADC12RST* = RCC_AHBRSTR_ADC12RST_Msk
  RCC_AHBRSTR_ADC34RST_Pos* = (29)
  RCC_AHBRSTR_ADC34RST_Msk* = (0x00000001 shl RCC_AHBRSTR_ADC34RST_Pos) ## !< 0x20000000
  RCC_AHBRSTR_ADC34RST* = RCC_AHBRSTR_ADC34RST_Msk

## ******************  Bit definition for RCC_CFGR2 register  *****************
## !< PREDIV configuration

const
  RCC_CFGR2_PREDIV_Pos* = (0)
  RCC_CFGR2_PREDIV_Msk* = (0x0000000F shl RCC_CFGR2_PREDIV_Pos) ## !< 0x0000000F
  RCC_CFGR2_PREDIV* = RCC_CFGR2_PREDIV_Msk
  RCC_CFGR2_PREDIV_0* = (0x00000001 shl RCC_CFGR2_PREDIV_Pos) ## !< 0x00000001
  RCC_CFGR2_PREDIV_1* = (0x00000002 shl RCC_CFGR2_PREDIV_Pos) ## !< 0x00000002
  RCC_CFGR2_PREDIV_2* = (0x00000004 shl RCC_CFGR2_PREDIV_Pos) ## !< 0x00000004
  RCC_CFGR2_PREDIV_3* = (0x00000008 shl RCC_CFGR2_PREDIV_Pos) ## !< 0x00000008
  RCC_CFGR2_PREDIV_DIV1* = (0x00000000) ## !< PREDIV input clock not divided
  RCC_CFGR2_PREDIV_DIV2* = (0x00000001) ## !< PREDIV input clock divided by 2
  RCC_CFGR2_PREDIV_DIV3* = (0x00000002) ## !< PREDIV input clock divided by 3
  RCC_CFGR2_PREDIV_DIV4* = (0x00000003) ## !< PREDIV input clock divided by 4
  RCC_CFGR2_PREDIV_DIV5* = (0x00000004) ## !< PREDIV input clock divided by 5
  RCC_CFGR2_PREDIV_DIV6* = (0x00000005) ## !< PREDIV input clock divided by 6
  RCC_CFGR2_PREDIV_DIV7* = (0x00000006) ## !< PREDIV input clock divided by 7
  RCC_CFGR2_PREDIV_DIV8* = (0x00000007) ## !< PREDIV input clock divided by 8
  RCC_CFGR2_PREDIV_DIV9* = (0x00000008) ## !< PREDIV input clock divided by 9
  RCC_CFGR2_PREDIV_DIV10* = (0x00000009) ## !< PREDIV input clock divided by 10
  RCC_CFGR2_PREDIV_DIV11* = (0x0000000A) ## !< PREDIV input clock divided by 11
  RCC_CFGR2_PREDIV_DIV12* = (0x0000000B) ## !< PREDIV input clock divided by 12
  RCC_CFGR2_PREDIV_DIV13* = (0x0000000C) ## !< PREDIV input clock divided by 13
  RCC_CFGR2_PREDIV_DIV14* = (0x0000000D) ## !< PREDIV input clock divided by 14
  RCC_CFGR2_PREDIV_DIV15* = (0x0000000E) ## !< PREDIV input clock divided by 15
  RCC_CFGR2_PREDIV_DIV16* = (0x0000000F) ## !< PREDIV input clock divided by 16

## !< ADCPRE12 configuration

const
  RCC_CFGR2_ADCPRE12_Pos* = (4)
  RCC_CFGR2_ADCPRE12_Msk* = (0x0000001F shl RCC_CFGR2_ADCPRE12_Pos) ## !< 0x000001F0
  RCC_CFGR2_ADCPRE12* = RCC_CFGR2_ADCPRE12_Msk
  RCC_CFGR2_ADCPRE12_0* = (0x00000001 shl RCC_CFGR2_ADCPRE12_Pos) ## !< 0x00000010
  RCC_CFGR2_ADCPRE12_1* = (0x00000002 shl RCC_CFGR2_ADCPRE12_Pos) ## !< 0x00000020
  RCC_CFGR2_ADCPRE12_2* = (0x00000004 shl RCC_CFGR2_ADCPRE12_Pos) ## !< 0x00000040
  RCC_CFGR2_ADCPRE12_3* = (0x00000008 shl RCC_CFGR2_ADCPRE12_Pos) ## !< 0x00000080
  RCC_CFGR2_ADCPRE12_4* = (0x00000010 shl RCC_CFGR2_ADCPRE12_Pos) ## !< 0x00000100
  RCC_CFGR2_ADCPRE12_NO* = (0x00000000) ## !< ADC12 clock disabled, ADC12 can use AHB clock
  RCC_CFGR2_ADCPRE12_DIV1* = (0x00000100) ## !< ADC12 PLL clock divided by 1
  RCC_CFGR2_ADCPRE12_DIV2* = (0x00000110) ## !< ADC12 PLL clock divided by 2
  RCC_CFGR2_ADCPRE12_DIV4* = (0x00000120) ## !< ADC12 PLL clock divided by 4
  RCC_CFGR2_ADCPRE12_DIV6* = (0x00000130) ## !< ADC12 PLL clock divided by 6
  RCC_CFGR2_ADCPRE12_DIV8* = (0x00000140) ## !< ADC12 PLL clock divided by 8
  RCC_CFGR2_ADCPRE12_DIV10* = (0x00000150) ## !< ADC12 PLL clock divided by 10
  RCC_CFGR2_ADCPRE12_DIV12* = (0x00000160) ## !< ADC12 PLL clock divided by 12
  RCC_CFGR2_ADCPRE12_DIV16* = (0x00000170) ## !< ADC12 PLL clock divided by 16
  RCC_CFGR2_ADCPRE12_DIV32* = (0x00000180) ## !< ADC12 PLL clock divided by 32
  RCC_CFGR2_ADCPRE12_DIV64* = (0x00000190) ## !< ADC12 PLL clock divided by 64
  RCC_CFGR2_ADCPRE12_DIV128* = (0x000001A0) ## !< ADC12 PLL clock divided by 128
  RCC_CFGR2_ADCPRE12_DIV256* = (0x000001B0) ## !< ADC12 PLL clock divided by 256

## !< ADCPRE34 configuration

const
  RCC_CFGR2_ADCPRE34_Pos* = (9)
  RCC_CFGR2_ADCPRE34_Msk* = (0x0000001F shl RCC_CFGR2_ADCPRE34_Pos) ## !< 0x00003E00
  RCC_CFGR2_ADCPRE34* = RCC_CFGR2_ADCPRE34_Msk
  RCC_CFGR2_ADCPRE34_0* = (0x00000001 shl RCC_CFGR2_ADCPRE34_Pos) ## !< 0x00000200
  RCC_CFGR2_ADCPRE34_1* = (0x00000002 shl RCC_CFGR2_ADCPRE34_Pos) ## !< 0x00000400
  RCC_CFGR2_ADCPRE34_2* = (0x00000004 shl RCC_CFGR2_ADCPRE34_Pos) ## !< 0x00000800
  RCC_CFGR2_ADCPRE34_3* = (0x00000008 shl RCC_CFGR2_ADCPRE34_Pos) ## !< 0x00001000
  RCC_CFGR2_ADCPRE34_4* = (0x00000010 shl RCC_CFGR2_ADCPRE34_Pos) ## !< 0x00002000
  RCC_CFGR2_ADCPRE34_NO* = (0x00000000) ## !< ADC34 clock disabled, ADC34 can use AHB clock
  RCC_CFGR2_ADCPRE34_DIV1* = (0x00002000) ## !< ADC34 PLL clock divided by 1
  RCC_CFGR2_ADCPRE34_DIV2* = (0x00002200) ## !< ADC34 PLL clock divided by 2
  RCC_CFGR2_ADCPRE34_DIV4* = (0x00002400) ## !< ADC34 PLL clock divided by 4
  RCC_CFGR2_ADCPRE34_DIV6* = (0x00002600) ## !< ADC34 PLL clock divided by 6
  RCC_CFGR2_ADCPRE34_DIV8* = (0x00002800) ## !< ADC34 PLL clock divided by 8
  RCC_CFGR2_ADCPRE34_DIV10* = (0x00002A00) ## !< ADC34 PLL clock divided by 10
  RCC_CFGR2_ADCPRE34_DIV12* = (0x00002C00) ## !< ADC34 PLL clock divided by 12
  RCC_CFGR2_ADCPRE34_DIV16* = (0x00002E00) ## !< ADC34 PLL clock divided by 16
  RCC_CFGR2_ADCPRE34_DIV32* = (0x00003000) ## !< ADC34 PLL clock divided by 32
  RCC_CFGR2_ADCPRE34_DIV64* = (0x00003200) ## !< ADC34 PLL clock divided by 64
  RCC_CFGR2_ADCPRE34_DIV128* = (0x00003400) ## !< ADC34 PLL clock divided by 128
  RCC_CFGR2_ADCPRE34_DIV256* = (0x00003600) ## !< ADC34 PLL clock divided by 256

## ******************  Bit definition for RCC_CFGR3 register  *****************

const
  RCC_CFGR3_USART1SW_Pos* = (0)
  RCC_CFGR3_USART1SW_Msk* = (0x00000003 shl RCC_CFGR3_USART1SW_Pos) ## !< 0x00000003
  RCC_CFGR3_USART1SW* = RCC_CFGR3_USART1SW_Msk
  RCC_CFGR3_USART1SW_0* = (0x00000001 shl RCC_CFGR3_USART1SW_Pos) ## !< 0x00000001
  RCC_CFGR3_USART1SW_1* = (0x00000002 shl RCC_CFGR3_USART1SW_Pos) ## !< 0x00000002
  RCC_CFGR3_USART1SW_PCLK2* = (0x00000000) ## !< PCLK2 clock used as USART1 clock source
  RCC_CFGR3_USART1SW_SYSCLK* = (0x00000001) ## !< System clock selected as USART1 clock source
  RCC_CFGR3_USART1SW_LSE* = (0x00000002) ## !< LSE oscillator clock used as USART1 clock source
  RCC_CFGR3_USART1SW_HSI* = (0x00000003) ## !< HSI oscillator clock used as USART1 clock source

##  Legacy defines

const
  RCC_CFGR3_USART1SW_PCLK* = RCC_CFGR3_USART1SW_PCLK2
  RCC_CFGR3_I2CSW_Pos* = (4)
  RCC_CFGR3_I2CSW_Msk* = (0x00000003 shl RCC_CFGR3_I2CSW_Pos) ## !< 0x00000030
  RCC_CFGR3_I2CSW* = RCC_CFGR3_I2CSW_Msk
  RCC_CFGR3_I2C1SW_Pos* = (4)
  RCC_CFGR3_I2C1SW_Msk* = (0x00000001 shl RCC_CFGR3_I2C1SW_Pos) ## !< 0x00000010
  RCC_CFGR3_I2C1SW* = RCC_CFGR3_I2C1SW_Msk
  RCC_CFGR3_I2C2SW_Pos* = (5)
  RCC_CFGR3_I2C2SW_Msk* = (0x00000001 shl RCC_CFGR3_I2C2SW_Pos) ## !< 0x00000020
  RCC_CFGR3_I2C2SW* = RCC_CFGR3_I2C2SW_Msk
  RCC_CFGR3_I2C1SW_HSI* = (0x00000000) ## !< HSI oscillator clock used as I2C1 clock source
  RCC_CFGR3_I2C1SW_SYSCLK_Pos* = (4)
  RCC_CFGR3_I2C1SW_SYSCLK_Msk* = (0x00000001 shl RCC_CFGR3_I2C1SW_SYSCLK_Pos) ## !< 0x00000010
  RCC_CFGR3_I2C1SW_SYSCLK* = RCC_CFGR3_I2C1SW_SYSCLK_Msk
  RCC_CFGR3_I2C2SW_HSI* = (0x00000000) ## !< HSI oscillator clock used as I2C2 clock source
  RCC_CFGR3_I2C2SW_SYSCLK_Pos* = (5)
  RCC_CFGR3_I2C2SW_SYSCLK_Msk* = (0x00000001 shl RCC_CFGR3_I2C2SW_SYSCLK_Pos) ## !< 0x00000020
  RCC_CFGR3_I2C2SW_SYSCLK* = RCC_CFGR3_I2C2SW_SYSCLK_Msk
  RCC_CFGR3_TIMSW_Pos* = (8)
  RCC_CFGR3_TIMSW_Msk* = (0x00000003 shl RCC_CFGR3_TIMSW_Pos) ## !< 0x00000300
  RCC_CFGR3_TIMSW* = RCC_CFGR3_TIMSW_Msk
  RCC_CFGR3_TIM1SW_Pos* = (8)
  RCC_CFGR3_TIM1SW_Msk* = (0x00000001 shl RCC_CFGR3_TIM1SW_Pos) ## !< 0x00000100
  RCC_CFGR3_TIM1SW* = RCC_CFGR3_TIM1SW_Msk
  RCC_CFGR3_TIM8SW_Pos* = (9)
  RCC_CFGR3_TIM8SW_Msk* = (0x00000001 shl RCC_CFGR3_TIM8SW_Pos) ## !< 0x00000200
  RCC_CFGR3_TIM8SW* = RCC_CFGR3_TIM8SW_Msk
  RCC_CFGR3_TIM1SW_PCLK2* = (0x00000000) ## !< PCLK2 used as TIM1 clock source
  RCC_CFGR3_TIM1SW_PLL_Pos* = (8)
  RCC_CFGR3_TIM1SW_PLL_Msk* = (0x00000001 shl RCC_CFGR3_TIM1SW_PLL_Pos) ## !< 0x00000100
  RCC_CFGR3_TIM1SW_PLL* = RCC_CFGR3_TIM1SW_PLL_Msk
  RCC_CFGR3_TIM8SW_PCLK2* = (0x00000000) ## !< PCLK2 used as TIM8 clock source
  RCC_CFGR3_TIM8SW_PLL_Pos* = (9)
  RCC_CFGR3_TIM8SW_PLL_Msk* = (0x00000001 shl RCC_CFGR3_TIM8SW_PLL_Pos) ## !< 0x00000200
  RCC_CFGR3_TIM8SW_PLL* = RCC_CFGR3_TIM8SW_PLL_Msk
  RCC_CFGR3_USART2SW_Pos* = (16)
  RCC_CFGR3_USART2SW_Msk* = (0x00000003 shl RCC_CFGR3_USART2SW_Pos) ## !< 0x00030000
  RCC_CFGR3_USART2SW* = RCC_CFGR3_USART2SW_Msk
  RCC_CFGR3_USART2SW_0* = (0x00000001 shl RCC_CFGR3_USART2SW_Pos) ## !< 0x00010000
  RCC_CFGR3_USART2SW_1* = (0x00000002 shl RCC_CFGR3_USART2SW_Pos) ## !< 0x00020000
  RCC_CFGR3_USART2SW_PCLK* = (0x00000000) ## !< PCLK1 clock used as USART2 clock source
  RCC_CFGR3_USART2SW_SYSCLK* = (0x00010000) ## !< System clock selected as USART2 clock source
  RCC_CFGR3_USART2SW_LSE* = (0x00020000) ## !< LSE oscillator clock used as USART2 clock source
  RCC_CFGR3_USART2SW_HSI* = (0x00030000) ## !< HSI oscillator clock used as USART2 clock source
  RCC_CFGR3_USART3SW_Pos* = (18)
  RCC_CFGR3_USART3SW_Msk* = (0x00000003 shl RCC_CFGR3_USART3SW_Pos) ## !< 0x000C0000
  RCC_CFGR3_USART3SW* = RCC_CFGR3_USART3SW_Msk
  RCC_CFGR3_USART3SW_0* = (0x00000001 shl RCC_CFGR3_USART3SW_Pos) ## !< 0x00040000
  RCC_CFGR3_USART3SW_1* = (0x00000002 shl RCC_CFGR3_USART3SW_Pos) ## !< 0x00080000
  RCC_CFGR3_USART3SW_PCLK* = (0x00000000) ## !< PCLK1 clock used as USART3 clock source
  RCC_CFGR3_USART3SW_SYSCLK* = (0x00040000) ## !< System clock selected as USART3 clock source
  RCC_CFGR3_USART3SW_LSE* = (0x00080000) ## !< LSE oscillator clock used as USART3 clock source
  RCC_CFGR3_USART3SW_HSI* = (0x000C0000) ## !< HSI oscillator clock used as USART3 clock source
  RCC_CFGR3_UART4SW_Pos* = (20)
  RCC_CFGR3_UART4SW_Msk* = (0x00000003 shl RCC_CFGR3_UART4SW_Pos) ## !< 0x00300000
  RCC_CFGR3_UART4SW* = RCC_CFGR3_UART4SW_Msk
  RCC_CFGR3_UART4SW_0* = (0x00000001 shl RCC_CFGR3_UART4SW_Pos) ## !< 0x00100000
  RCC_CFGR3_UART4SW_1* = (0x00000002 shl RCC_CFGR3_UART4SW_Pos) ## !< 0x00200000
  RCC_CFGR3_UART4SW_PCLK* = (0x00000000) ## !< PCLK1 clock used as UART4 clock source
  RCC_CFGR3_UART4SW_SYSCLK* = (0x00100000) ## !< System clock selected as UART4 clock source
  RCC_CFGR3_UART4SW_LSE* = (0x00200000) ## !< LSE oscillator clock used as UART4 clock source
  RCC_CFGR3_UART4SW_HSI* = (0x00300000) ## !< HSI oscillator clock used as UART4 clock source
  RCC_CFGR3_UART5SW_Pos* = (22)
  RCC_CFGR3_UART5SW_Msk* = (0x00000003 shl RCC_CFGR3_UART5SW_Pos) ## !< 0x00C00000
  RCC_CFGR3_UART5SW* = RCC_CFGR3_UART5SW_Msk
  RCC_CFGR3_UART5SW_0* = (0x00000001 shl RCC_CFGR3_UART5SW_Pos) ## !< 0x00400000
  RCC_CFGR3_UART5SW_1* = (0x00000002 shl RCC_CFGR3_UART5SW_Pos) ## !< 0x00800000
  RCC_CFGR3_UART5SW_PCLK* = (0x00000000) ## !< PCLK1 clock used as UART5 clock source
  RCC_CFGR3_UART5SW_SYSCLK* = (0x00400000) ## !< System clock selected as UART5 clock source
  RCC_CFGR3_UART5SW_LSE* = (0x00800000) ## !< LSE oscillator clock used as UART5 clock source
  RCC_CFGR3_UART5SW_HSI* = (0x00C00000) ## !< HSI oscillator clock used as UART5 clock source

##  Legacy defines

const
  RCC_CFGR3_TIM1SW_HCLK* = RCC_CFGR3_TIM1SW_PCLK2
  RCC_CFGR3_TIM8SW_HCLK* = RCC_CFGR3_TIM8SW_PCLK2

## ****************************************************************************
##
##                            Real-Time Clock (RTC)
##
## ****************************************************************************
##
##  @brief Specific device feature definitions  (not present on all devices in the STM32F3 serie)
##

const
  RTC_TAMPER1_SUPPORT* = true   ## !< TAMPER 1 feature support
  RTC_TAMPER2_SUPPORT* = true   ## !< TAMPER 2 feature support
  RTC_TAMPER3_SUPPORT* = true   ## !< TAMPER 3 feature support
  RTC_BACKUP_SUPPORT* = true    ## !< BACKUP register feature support
  RTC_WAKEUP_SUPPORT* = true    ## !< WAKEUP feature support

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
  RTC_TAFCR_PC15MODE_Pos* = (23)
  RTC_TAFCR_PC15MODE_Msk* = (0x00000001 shl RTC_TAFCR_PC15MODE_Pos) ## !< 0x00800000
  RTC_TAFCR_PC15MODE* = RTC_TAFCR_PC15MODE_Msk
  RTC_TAFCR_PC15VALUE_Pos* = (22)
  RTC_TAFCR_PC15VALUE_Msk* = (0x00000001 shl RTC_TAFCR_PC15VALUE_Pos) ## !< 0x00400000
  RTC_TAFCR_PC15VALUE* = RTC_TAFCR_PC15VALUE_Msk
  RTC_TAFCR_PC14MODE_Pos* = (21)
  RTC_TAFCR_PC14MODE_Msk* = (0x00000001 shl RTC_TAFCR_PC14MODE_Pos) ## !< 0x00200000
  RTC_TAFCR_PC14MODE* = RTC_TAFCR_PC14MODE_Msk
  RTC_TAFCR_PC14VALUE_Pos* = (20)
  RTC_TAFCR_PC14VALUE_Msk* = (0x00000001 shl RTC_TAFCR_PC14VALUE_Pos) ## !< 0x00100000
  RTC_TAFCR_PC14VALUE* = RTC_TAFCR_PC14VALUE_Msk
  RTC_TAFCR_PC13MODE_Pos* = (19)
  RTC_TAFCR_PC13MODE_Msk* = (0x00000001 shl RTC_TAFCR_PC13MODE_Pos) ## !< 0x00080000
  RTC_TAFCR_PC13MODE* = RTC_TAFCR_PC13MODE_Msk
  RTC_TAFCR_PC13VALUE_Pos* = (18)
  RTC_TAFCR_PC13VALUE_Msk* = (0x00000001 shl RTC_TAFCR_PC13VALUE_Pos) ## !< 0x00040000
  RTC_TAFCR_PC13VALUE* = RTC_TAFCR_PC13VALUE_Msk
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

##  Reference defines

const
  RTC_TAFCR_ALARMOUTTYPE* = RTC_TAFCR_PC13VALUE

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

## ******************* Number of backup registers *****************************

const
  RTC_BKP_NUMBER* = 16

## ****************************************************************************
##
##                         Serial Peripheral Interface (SPI)
##
## ****************************************************************************
##
##  @brief Specific device feature definitions (not present on all devices in the STM32F3 serie)
##

const
  SPI_I2S_SUPPORT* = true       ## !< I2S support
  SPI_I2S_FULLDUPLEX_SUPPORT* = true ## !< I2S Full-Duplex support

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
  SPI_CR1_CRCL_Pos* = (11)
  SPI_CR1_CRCL_Msk* = (0x00000001 shl SPI_CR1_CRCL_Pos) ## !< 0x00000800
  SPI_CR1_CRCL* = SPI_CR1_CRCL_Msk
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
  SPI_CR2_NSSP_Pos* = (3)
  SPI_CR2_NSSP_Msk* = (0x00000001 shl SPI_CR2_NSSP_Pos) ## !< 0x00000008
  SPI_CR2_NSSP* = SPI_CR2_NSSP_Msk
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
  SPI_CR2_DS_Pos* = (8)
  SPI_CR2_DS_Msk* = (0x0000000F shl SPI_CR2_DS_Pos) ## !< 0x00000F00
  SPI_CR2_DS* = SPI_CR2_DS_Msk
  SPI_CR2_DS_0* = (0x00000001 shl SPI_CR2_DS_Pos) ## !< 0x00000100
  SPI_CR2_DS_1* = (0x00000002 shl SPI_CR2_DS_Pos) ## !< 0x00000200
  SPI_CR2_DS_2* = (0x00000004 shl SPI_CR2_DS_Pos) ## !< 0x00000400
  SPI_CR2_DS_3* = (0x00000008 shl SPI_CR2_DS_Pos) ## !< 0x00000800
  SPI_CR2_FRXTH_Pos* = (12)
  SPI_CR2_FRXTH_Msk* = (0x00000001 shl SPI_CR2_FRXTH_Pos) ## !< 0x00001000
  SPI_CR2_FRXTH* = SPI_CR2_FRXTH_Msk
  SPI_CR2_LDMARX_Pos* = (13)
  SPI_CR2_LDMARX_Msk* = (0x00000001 shl SPI_CR2_LDMARX_Pos) ## !< 0x00002000
  SPI_CR2_LDMARX* = SPI_CR2_LDMARX_Msk
  SPI_CR2_LDMATX_Pos* = (14)
  SPI_CR2_LDMATX_Msk* = (0x00000001 shl SPI_CR2_LDMATX_Pos) ## !< 0x00004000
  SPI_CR2_LDMATX* = SPI_CR2_LDMATX_Msk

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
  SPI_SR_FRLVL_Pos* = (9)
  SPI_SR_FRLVL_Msk* = (0x00000003 shl SPI_SR_FRLVL_Pos) ## !< 0x00000600
  SPI_SR_FRLVL* = SPI_SR_FRLVL_Msk
  SPI_SR_FRLVL_0* = (0x00000001 shl SPI_SR_FRLVL_Pos) ## !< 0x00000200
  SPI_SR_FRLVL_1* = (0x00000002 shl SPI_SR_FRLVL_Pos) ## !< 0x00000400
  SPI_SR_FTLVL_Pos* = (11)
  SPI_SR_FTLVL_Msk* = (0x00000003 shl SPI_SR_FTLVL_Pos) ## !< 0x00001800
  SPI_SR_FTLVL* = SPI_SR_FTLVL_Msk
  SPI_SR_FTLVL_0* = (0x00000001 shl SPI_SR_FTLVL_Pos) ## !< 0x00000800
  SPI_SR_FTLVL_1* = (0x00000002 shl SPI_SR_FTLVL_Pos) ## !< 0x00001000

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
##                         System Configuration(SYSCFG)
##
## ****************************************************************************
## ****************  Bit definition for SYSCFG_CFGR1 register  ***************

const
  SYSCFG_CFGR1_MEM_MODE_Pos* = (0)
  SYSCFG_CFGR1_MEM_MODE_Msk* = (0x00000003 shl SYSCFG_CFGR1_MEM_MODE_Pos) ## !< 0x00000003
  SYSCFG_CFGR1_MEM_MODE* = SYSCFG_CFGR1_MEM_MODE_Msk
  SYSCFG_CFGR1_MEM_MODE_0* = (0x00000001) ## !< Bit 0
  SYSCFG_CFGR1_MEM_MODE_1* = (0x00000002) ## !< Bit 1
  SYSCFG_CFGR1_USB_IT_RMP_Pos* = (5)
  SYSCFG_CFGR1_USB_IT_RMP_Msk* = (0x00000001 shl SYSCFG_CFGR1_USB_IT_RMP_Pos) ## !< 0x00000020
  SYSCFG_CFGR1_USB_IT_RMP* = SYSCFG_CFGR1_USB_IT_RMP_Msk
  SYSCFG_CFGR1_TIM1_ITR3_RMP_Pos* = (6)
  SYSCFG_CFGR1_TIM1_ITR3_RMP_Msk* = (0x00000001 shl
      SYSCFG_CFGR1_TIM1_ITR3_RMP_Pos) ## !< 0x00000040
  SYSCFG_CFGR1_TIM1_ITR3_RMP* = SYSCFG_CFGR1_TIM1_ITR3_RMP_Msk
  SYSCFG_CFGR1_DAC1_TRIG1_RMP_Pos* = (7)
  SYSCFG_CFGR1_DAC1_TRIG1_RMP_Msk* = (
    0x00000001 shl SYSCFG_CFGR1_DAC1_TRIG1_RMP_Pos) ## !< 0x00000080
  SYSCFG_CFGR1_DAC1_TRIG1_RMP* = SYSCFG_CFGR1_DAC1_TRIG1_RMP_Msk
  SYSCFG_CFGR1_DMA_RMP_Pos* = (8)
  SYSCFG_CFGR1_DMA_RMP_Msk* = (0x00000079 shl SYSCFG_CFGR1_DMA_RMP_Pos) ## !< 0x00007900
  SYSCFG_CFGR1_DMA_RMP* = SYSCFG_CFGR1_DMA_RMP_Msk
  SYSCFG_CFGR1_ADC24_DMA_RMP_Pos* = (8)
  SYSCFG_CFGR1_ADC24_DMA_RMP_Msk* = (0x00000001 shl
      SYSCFG_CFGR1_ADC24_DMA_RMP_Pos) ## !< 0x00000100
  SYSCFG_CFGR1_ADC24_DMA_RMP* = SYSCFG_CFGR1_ADC24_DMA_RMP_Msk
  SYSCFG_CFGR1_TIM16_DMA_RMP_Pos* = (11)
  SYSCFG_CFGR1_TIM16_DMA_RMP_Msk* = (0x00000001 shl
      SYSCFG_CFGR1_TIM16_DMA_RMP_Pos) ## !< 0x00000800
  SYSCFG_CFGR1_TIM16_DMA_RMP* = SYSCFG_CFGR1_TIM16_DMA_RMP_Msk
  SYSCFG_CFGR1_TIM17_DMA_RMP_Pos* = (12)
  SYSCFG_CFGR1_TIM17_DMA_RMP_Msk* = (0x00000001 shl
      SYSCFG_CFGR1_TIM17_DMA_RMP_Pos) ## !< 0x00001000
  SYSCFG_CFGR1_TIM17_DMA_RMP* = SYSCFG_CFGR1_TIM17_DMA_RMP_Msk
  SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP_Pos* = (13)
  SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP_Msk* = (
    0x00000001 shl SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP_Pos) ## !< 0x00002000
  SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP* = SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP_Msk
  SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP_Pos* = (14)
  SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP_Msk* = (
    0x00000001 shl SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP_Pos) ## !< 0x00004000
  SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP* = SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP_Msk
  SYSCFG_CFGR1_I2C_PB6_FMP_Pos* = (16)
  SYSCFG_CFGR1_I2C_PB6_FMP_Msk* = (0x00000001 shl SYSCFG_CFGR1_I2C_PB6_FMP_Pos) ## !< 0x00010000
  SYSCFG_CFGR1_I2C_PB6_FMP* = SYSCFG_CFGR1_I2C_PB6_FMP_Msk
  SYSCFG_CFGR1_I2C_PB7_FMP_Pos* = (17)
  SYSCFG_CFGR1_I2C_PB7_FMP_Msk* = (0x00000001 shl SYSCFG_CFGR1_I2C_PB7_FMP_Pos) ## !< 0x00020000
  SYSCFG_CFGR1_I2C_PB7_FMP* = SYSCFG_CFGR1_I2C_PB7_FMP_Msk
  SYSCFG_CFGR1_I2C_PB8_FMP_Pos* = (18)
  SYSCFG_CFGR1_I2C_PB8_FMP_Msk* = (0x00000001 shl SYSCFG_CFGR1_I2C_PB8_FMP_Pos) ## !< 0x00040000
  SYSCFG_CFGR1_I2C_PB8_FMP* = SYSCFG_CFGR1_I2C_PB8_FMP_Msk
  SYSCFG_CFGR1_I2C_PB9_FMP_Pos* = (19)
  SYSCFG_CFGR1_I2C_PB9_FMP_Msk* = (0x00000001 shl SYSCFG_CFGR1_I2C_PB9_FMP_Pos) ## !< 0x00080000
  SYSCFG_CFGR1_I2C_PB9_FMP* = SYSCFG_CFGR1_I2C_PB9_FMP_Msk
  SYSCFG_CFGR1_I2C1_FMP_Pos* = (20)
  SYSCFG_CFGR1_I2C1_FMP_Msk* = (0x00000001 shl SYSCFG_CFGR1_I2C1_FMP_Pos) ## !< 0x00100000
  SYSCFG_CFGR1_I2C1_FMP* = SYSCFG_CFGR1_I2C1_FMP_Msk
  SYSCFG_CFGR1_I2C2_FMP_Pos* = (21)
  SYSCFG_CFGR1_I2C2_FMP_Msk* = (0x00000001 shl SYSCFG_CFGR1_I2C2_FMP_Pos) ## !< 0x00200000
  SYSCFG_CFGR1_I2C2_FMP* = SYSCFG_CFGR1_I2C2_FMP_Msk
  SYSCFG_CFGR1_ENCODER_MODE_Pos* = (22)
  SYSCFG_CFGR1_ENCODER_MODE_Msk* = (0x00000003 shl SYSCFG_CFGR1_ENCODER_MODE_Pos) ## !< 0x00C00000
  SYSCFG_CFGR1_ENCODER_MODE* = SYSCFG_CFGR1_ENCODER_MODE_Msk
  SYSCFG_CFGR1_ENCODER_MODE_0* = (0x00000001 shl SYSCFG_CFGR1_ENCODER_MODE_Pos) ## !< 0x00400000
  SYSCFG_CFGR1_ENCODER_MODE_1* = (0x00000002 shl SYSCFG_CFGR1_ENCODER_MODE_Pos) ## !< 0x00800000
  SYSCFG_CFGR1_ENCODER_MODE_TIM2_Pos* = (22)
  SYSCFG_CFGR1_ENCODER_MODE_TIM2_Msk* = (
    0x00000001 shl SYSCFG_CFGR1_ENCODER_MODE_TIM2_Pos) ## !< 0x00400000
  SYSCFG_CFGR1_ENCODER_MODE_TIM2* = SYSCFG_CFGR1_ENCODER_MODE_TIM2_Msk
  SYSCFG_CFGR1_ENCODER_MODE_TIM3_Pos* = (23)
  SYSCFG_CFGR1_ENCODER_MODE_TIM3_Msk* = (
    0x00000001 shl SYSCFG_CFGR1_ENCODER_MODE_TIM3_Pos) ## !< 0x00800000
  SYSCFG_CFGR1_ENCODER_MODE_TIM3* = SYSCFG_CFGR1_ENCODER_MODE_TIM3_Msk
  SYSCFG_CFGR1_ENCODER_MODE_TIM4_Pos* = (22)
  SYSCFG_CFGR1_ENCODER_MODE_TIM4_Msk* = (
    0x00000003 shl SYSCFG_CFGR1_ENCODER_MODE_TIM4_Pos) ## !< 0x00C00000
  SYSCFG_CFGR1_ENCODER_MODE_TIM4* = SYSCFG_CFGR1_ENCODER_MODE_TIM4_Msk
  SYSCFG_CFGR1_FPU_IE_Pos* = (26)
  SYSCFG_CFGR1_FPU_IE_Msk* = (0x0000003F shl SYSCFG_CFGR1_FPU_IE_Pos) ## !< 0xFC000000
  SYSCFG_CFGR1_FPU_IE* = SYSCFG_CFGR1_FPU_IE_Msk
  SYSCFG_CFGR1_FPU_IE_0* = (0x00000001 shl SYSCFG_CFGR1_FPU_IE_Pos) ## !< 0x04000000
  SYSCFG_CFGR1_FPU_IE_1* = (0x00000002 shl SYSCFG_CFGR1_FPU_IE_Pos) ## !< 0x08000000
  SYSCFG_CFGR1_FPU_IE_2* = (0x00000004 shl SYSCFG_CFGR1_FPU_IE_Pos) ## !< 0x10000000
  SYSCFG_CFGR1_FPU_IE_3* = (0x00000008 shl SYSCFG_CFGR1_FPU_IE_Pos) ## !< 0x20000000
  SYSCFG_CFGR1_FPU_IE_4* = (0x00000010 shl SYSCFG_CFGR1_FPU_IE_Pos) ## !< 0x40000000
  SYSCFG_CFGR1_FPU_IE_5* = (0x00000020 shl SYSCFG_CFGR1_FPU_IE_Pos) ## !< 0x80000000

## ****************  Bit definition for SYSCFG_RCR register  ******************

const
  SYSCFG_RCR_PAGE0_Pos* = (0)
  SYSCFG_RCR_PAGE0_Msk* = (0x00000001 shl SYSCFG_RCR_PAGE0_Pos) ## !< 0x00000001
  SYSCFG_RCR_PAGE0* = SYSCFG_RCR_PAGE0_Msk
  SYSCFG_RCR_PAGE1_Pos* = (1)
  SYSCFG_RCR_PAGE1_Msk* = (0x00000001 shl SYSCFG_RCR_PAGE1_Pos) ## !< 0x00000002
  SYSCFG_RCR_PAGE1* = SYSCFG_RCR_PAGE1_Msk
  SYSCFG_RCR_PAGE2_Pos* = (2)
  SYSCFG_RCR_PAGE2_Msk* = (0x00000001 shl SYSCFG_RCR_PAGE2_Pos) ## !< 0x00000004
  SYSCFG_RCR_PAGE2* = SYSCFG_RCR_PAGE2_Msk
  SYSCFG_RCR_PAGE3_Pos* = (3)
  SYSCFG_RCR_PAGE3_Msk* = (0x00000001 shl SYSCFG_RCR_PAGE3_Pos) ## !< 0x00000008
  SYSCFG_RCR_PAGE3* = SYSCFG_RCR_PAGE3_Msk
  SYSCFG_RCR_PAGE4_Pos* = (4)
  SYSCFG_RCR_PAGE4_Msk* = (0x00000001 shl SYSCFG_RCR_PAGE4_Pos) ## !< 0x00000010
  SYSCFG_RCR_PAGE4* = SYSCFG_RCR_PAGE4_Msk
  SYSCFG_RCR_PAGE5_Pos* = (5)
  SYSCFG_RCR_PAGE5_Msk* = (0x00000001 shl SYSCFG_RCR_PAGE5_Pos) ## !< 0x00000020
  SYSCFG_RCR_PAGE5* = SYSCFG_RCR_PAGE5_Msk
  SYSCFG_RCR_PAGE6_Pos* = (6)
  SYSCFG_RCR_PAGE6_Msk* = (0x00000001 shl SYSCFG_RCR_PAGE6_Pos) ## !< 0x00000040
  SYSCFG_RCR_PAGE6* = SYSCFG_RCR_PAGE6_Msk
  SYSCFG_RCR_PAGE7_Pos* = (7)
  SYSCFG_RCR_PAGE7_Msk* = (0x00000001 shl SYSCFG_RCR_PAGE7_Pos) ## !< 0x00000080
  SYSCFG_RCR_PAGE7* = SYSCFG_RCR_PAGE7_Msk

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

## !<*
##  @brief  EXTI0 configuration
##

const
  SYSCFG_EXTICR1_EXTI0_PA* = (0x00000000) ## !< PA[0] pin
  SYSCFG_EXTICR1_EXTI0_PB* = (0x00000001) ## !< PB[0] pin
  SYSCFG_EXTICR1_EXTI0_PC* = (0x00000002) ## !< PC[0] pin
  SYSCFG_EXTICR1_EXTI0_PD* = (0x00000003) ## !< PD[0] pin
  SYSCFG_EXTICR1_EXTI0_PE* = (0x00000004) ## !< PE[0] pin
  SYSCFG_EXTICR1_EXTI0_PF* = (0x00000005) ## !< PF[0] pin

## !<*
##  @brief  EXTI1 configuration
##

const
  SYSCFG_EXTICR1_EXTI1_PA* = (0x00000000) ## !< PA[1] pin
  SYSCFG_EXTICR1_EXTI1_PB* = (0x00000010) ## !< PB[1] pin
  SYSCFG_EXTICR1_EXTI1_PC* = (0x00000020) ## !< PC[1] pin
  SYSCFG_EXTICR1_EXTI1_PD* = (0x00000030) ## !< PD[1] pin
  SYSCFG_EXTICR1_EXTI1_PE* = (0x00000040) ## !< PE[1] pin
  SYSCFG_EXTICR1_EXTI1_PF* = (0x00000050) ## !< PF[1] pin

## !<*
##  @brief  EXTI2 configuration
##

const
  SYSCFG_EXTICR1_EXTI2_PA* = (0x00000000) ## !< PA[2] pin
  SYSCFG_EXTICR1_EXTI2_PB* = (0x00000100) ## !< PB[2] pin
  SYSCFG_EXTICR1_EXTI2_PC* = (0x00000200) ## !< PC[2] pin
  SYSCFG_EXTICR1_EXTI2_PD* = (0x00000300) ## !< PD[2] pin
  SYSCFG_EXTICR1_EXTI2_PE* = (0x00000400) ## !< PE[2] pin
  SYSCFG_EXTICR1_EXTI2_PF* = (0x00000500) ## !< PF[2] pin

## !<*
##  @brief  EXTI3 configuration
##

const
  SYSCFG_EXTICR1_EXTI3_PA* = (0x00000000) ## !< PA[3] pin
  SYSCFG_EXTICR1_EXTI3_PB* = (0x00001000) ## !< PB[3] pin
  SYSCFG_EXTICR1_EXTI3_PC* = (0x00002000) ## !< PC[3] pin
  SYSCFG_EXTICR1_EXTI3_PD* = (0x00003000) ## !< PD[3] pin
  SYSCFG_EXTICR1_EXTI3_PE* = (0x00004000) ## !< PE[3] pin

## ****************  Bit definition for SYSCFG_EXTICR2 register  **************

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

## !<*
##  @brief  EXTI4 configuration
##

const
  SYSCFG_EXTICR2_EXTI4_PA* = (0x00000000) ## !< PA[4] pin
  SYSCFG_EXTICR2_EXTI4_PB* = (0x00000001) ## !< PB[4] pin
  SYSCFG_EXTICR2_EXTI4_PC* = (0x00000002) ## !< PC[4] pin
  SYSCFG_EXTICR2_EXTI4_PD* = (0x00000003) ## !< PD[4] pin
  SYSCFG_EXTICR2_EXTI4_PE* = (0x00000004) ## !< PE[4] pin
  SYSCFG_EXTICR2_EXTI4_PF* = (0x00000005) ## !< PF[4] pin

## !<*
##  @brief  EXTI5 configuration
##

const
  SYSCFG_EXTICR2_EXTI5_PA* = (0x00000000) ## !< PA[5] pin
  SYSCFG_EXTICR2_EXTI5_PB* = (0x00000010) ## !< PB[5] pin
  SYSCFG_EXTICR2_EXTI5_PC* = (0x00000020) ## !< PC[5] pin
  SYSCFG_EXTICR2_EXTI5_PD* = (0x00000030) ## !< PD[5] pin
  SYSCFG_EXTICR2_EXTI5_PE* = (0x00000040) ## !< PE[5] pin
  SYSCFG_EXTICR2_EXTI5_PF* = (0x00000050) ## !< PF[5] pin

## !<*
##  @brief  EXTI6 configuration
##

const
  SYSCFG_EXTICR2_EXTI6_PA* = (0x00000000) ## !< PA[6] pin
  SYSCFG_EXTICR2_EXTI6_PB* = (0x00000100) ## !< PB[6] pin
  SYSCFG_EXTICR2_EXTI6_PC* = (0x00000200) ## !< PC[6] pin
  SYSCFG_EXTICR2_EXTI6_PD* = (0x00000300) ## !< PD[6] pin
  SYSCFG_EXTICR2_EXTI6_PE* = (0x00000400) ## !< PE[6] pin
  SYSCFG_EXTICR2_EXTI6_PF* = (0x00000500) ## !< PF[6] pin

## !<*
##  @brief  EXTI7 configuration
##

const
  SYSCFG_EXTICR2_EXTI7_PA* = (0x00000000) ## !< PA[7] pin
  SYSCFG_EXTICR2_EXTI7_PB* = (0x00001000) ## !< PB[7] pin
  SYSCFG_EXTICR2_EXTI7_PC* = (0x00002000) ## !< PC[7] pin
  SYSCFG_EXTICR2_EXTI7_PD* = (0x00003000) ## !< PD[7] pin
  SYSCFG_EXTICR2_EXTI7_PE* = (0x00004000) ## !< PE[7] pin

## ****************  Bit definition for SYSCFG_EXTICR3 register  **************

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

## !<*
##  @brief  EXTI8 configuration
##

const
  SYSCFG_EXTICR3_EXTI8_PA* = (0x00000000) ## !< PA[8] pin
  SYSCFG_EXTICR3_EXTI8_PB* = (0x00000001) ## !< PB[8] pin
  SYSCFG_EXTICR3_EXTI8_PC* = (0x00000002) ## !< PC[8] pin
  SYSCFG_EXTICR3_EXTI8_PD* = (0x00000003) ## !< PD[8] pin
  SYSCFG_EXTICR3_EXTI8_PE* = (0x00000004) ## !< PE[8] pin

## !<*
##  @brief  EXTI9 configuration
##

const
  SYSCFG_EXTICR3_EXTI9_PA* = (0x00000000) ## !< PA[9] pin
  SYSCFG_EXTICR3_EXTI9_PB* = (0x00000010) ## !< PB[9] pin
  SYSCFG_EXTICR3_EXTI9_PC* = (0x00000020) ## !< PC[9] pin
  SYSCFG_EXTICR3_EXTI9_PD* = (0x00000030) ## !< PD[9] pin
  SYSCFG_EXTICR3_EXTI9_PE* = (0x00000040) ## !< PE[9] pin
  SYSCFG_EXTICR3_EXTI9_PF* = (0x00000050) ## !< PF[9] pin

## !<*
##  @brief  EXTI10 configuration
##

const
  SYSCFG_EXTICR3_EXTI10_PA* = (0x00000000) ## !< PA[10] pin
  SYSCFG_EXTICR3_EXTI10_PB* = (0x00000100) ## !< PB[10] pin
  SYSCFG_EXTICR3_EXTI10_PC* = (0x00000200) ## !< PC[10] pin
  SYSCFG_EXTICR3_EXTI10_PD* = (0x00000300) ## !< PD[10] pin
  SYSCFG_EXTICR3_EXTI10_PE* = (0x00000400) ## !< PE[10] pin
  SYSCFG_EXTICR3_EXTI10_PF* = (0x00000500) ## !< PF[10] pin

## !<*
##  @brief  EXTI11 configuration
##

const
  SYSCFG_EXTICR3_EXTI11_PA* = (0x00000000) ## !< PA[11] pin
  SYSCFG_EXTICR3_EXTI11_PB* = (0x00001000) ## !< PB[11] pin
  SYSCFG_EXTICR3_EXTI11_PC* = (0x00002000) ## !< PC[11] pin
  SYSCFG_EXTICR3_EXTI11_PD* = (0x00003000) ## !< PD[11] pin
  SYSCFG_EXTICR3_EXTI11_PE* = (0x00004000) ## !< PE[11] pin

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

## !<*
##  @brief  EXTI12 configuration
##

const
  SYSCFG_EXTICR4_EXTI12_PA* = (0x00000000) ## !< PA[12] pin
  SYSCFG_EXTICR4_EXTI12_PB* = (0x00000001) ## !< PB[12] pin
  SYSCFG_EXTICR4_EXTI12_PC* = (0x00000002) ## !< PC[12] pin
  SYSCFG_EXTICR4_EXTI12_PD* = (0x00000003) ## !< PD[12] pin
  SYSCFG_EXTICR4_EXTI12_PE* = (0x00000004) ## !< PE[12] pin

## !<*
##  @brief  EXTI13 configuration
##

const
  SYSCFG_EXTICR4_EXTI13_PA* = (0x00000000) ## !< PA[13] pin
  SYSCFG_EXTICR4_EXTI13_PB* = (0x00000010) ## !< PB[13] pin
  SYSCFG_EXTICR4_EXTI13_PC* = (0x00000020) ## !< PC[13] pin
  SYSCFG_EXTICR4_EXTI13_PD* = (0x00000030) ## !< PD[13] pin
  SYSCFG_EXTICR4_EXTI13_PE* = (0x00000040) ## !< PE[13] pin

## !<*
##  @brief  EXTI14 configuration
##

const
  SYSCFG_EXTICR4_EXTI14_PA* = (0x00000000) ## !< PA[14] pin
  SYSCFG_EXTICR4_EXTI14_PB* = (0x00000100) ## !< PB[14] pin
  SYSCFG_EXTICR4_EXTI14_PC* = (0x00000200) ## !< PC[14] pin
  SYSCFG_EXTICR4_EXTI14_PD* = (0x00000300) ## !< PD[14] pin
  SYSCFG_EXTICR4_EXTI14_PE* = (0x00000400) ## !< PE[14] pin

## !<*
##  @brief  EXTI15 configuration
##

const
  SYSCFG_EXTICR4_EXTI15_PA* = (0x00000000) ## !< PA[15] pin
  SYSCFG_EXTICR4_EXTI15_PB* = (0x00001000) ## !< PB[15] pin
  SYSCFG_EXTICR4_EXTI15_PC* = (0x00002000) ## !< PC[15] pin
  SYSCFG_EXTICR4_EXTI15_PD* = (0x00003000) ## !< PD[15] pin
  SYSCFG_EXTICR4_EXTI15_PE* = (0x00004000) ## !< PE[15] pin

## ****************  Bit definition for SYSCFG_CFGR2 register  ***************

const
  SYSCFG_CFGR2_LOCKUP_LOCK_Pos* = (0)
  SYSCFG_CFGR2_LOCKUP_LOCK_Msk* = (0x00000001 shl SYSCFG_CFGR2_LOCKUP_LOCK_Pos) ## !< 0x00000001
  SYSCFG_CFGR2_LOCKUP_LOCK* = SYSCFG_CFGR2_LOCKUP_LOCK_Msk
  SYSCFG_CFGR2_SRAM_PARITY_LOCK_Pos* = (1)
  SYSCFG_CFGR2_SRAM_PARITY_LOCK_Msk* = (
    0x00000001 shl SYSCFG_CFGR2_SRAM_PARITY_LOCK_Pos) ## !< 0x00000002
  SYSCFG_CFGR2_SRAM_PARITY_LOCK* = SYSCFG_CFGR2_SRAM_PARITY_LOCK_Msk
  SYSCFG_CFGR2_PVD_LOCK_Pos* = (2)
  SYSCFG_CFGR2_PVD_LOCK_Msk* = (0x00000001 shl SYSCFG_CFGR2_PVD_LOCK_Pos) ## !< 0x00000004
  SYSCFG_CFGR2_PVD_LOCK* = SYSCFG_CFGR2_PVD_LOCK_Msk
  SYSCFG_CFGR2_BYP_ADDR_PAR_Pos* = (4)
  SYSCFG_CFGR2_BYP_ADDR_PAR_Msk* = (0x00000001 shl SYSCFG_CFGR2_BYP_ADDR_PAR_Pos) ## !< 0x00000010
  SYSCFG_CFGR2_BYP_ADDR_PAR* = SYSCFG_CFGR2_BYP_ADDR_PAR_Msk
  SYSCFG_CFGR2_SRAM_PE_Pos* = (8)
  SYSCFG_CFGR2_SRAM_PE_Msk* = (0x00000001 shl SYSCFG_CFGR2_SRAM_PE_Pos) ## !< 0x00000100
  SYSCFG_CFGR2_SRAM_PE* = SYSCFG_CFGR2_SRAM_PE_Msk

## ****************************************************************************
##
##                                     TIM
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
  TIM_CR1_UIFREMAP_Pos* = (11)
  TIM_CR1_UIFREMAP_Msk* = (0x00000001 shl TIM_CR1_UIFREMAP_Pos) ## !< 0x00000800
  TIM_CR1_UIFREMAP* = TIM_CR1_UIFREMAP_Msk

## ******************  Bit definition for TIM_CR2 register  *******************

const
  TIM_CR2_CCPC_Pos* = (0)
  TIM_CR2_CCPC_Msk* = (0x00000001 shl TIM_CR2_CCPC_Pos) ## !< 0x00000001
  TIM_CR2_CCPC* = TIM_CR2_CCPC_Msk
  TIM_CR2_CCUS_Pos* = (2)
  TIM_CR2_CCUS_Msk* = (0x00000001 shl TIM_CR2_CCUS_Pos) ## !< 0x00000004
  TIM_CR2_CCUS* = TIM_CR2_CCUS_Msk
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
  TIM_CR2_OIS1_Pos* = (8)
  TIM_CR2_OIS1_Msk* = (0x00000001 shl TIM_CR2_OIS1_Pos) ## !< 0x00000100
  TIM_CR2_OIS1* = TIM_CR2_OIS1_Msk
  TIM_CR2_OIS1N_Pos* = (9)
  TIM_CR2_OIS1N_Msk* = (0x00000001 shl TIM_CR2_OIS1N_Pos) ## !< 0x00000200
  TIM_CR2_OIS1N* = TIM_CR2_OIS1N_Msk
  TIM_CR2_OIS2_Pos* = (10)
  TIM_CR2_OIS2_Msk* = (0x00000001 shl TIM_CR2_OIS2_Pos) ## !< 0x00000400
  TIM_CR2_OIS2* = TIM_CR2_OIS2_Msk
  TIM_CR2_OIS2N_Pos* = (11)
  TIM_CR2_OIS2N_Msk* = (0x00000001 shl TIM_CR2_OIS2N_Pos) ## !< 0x00000800
  TIM_CR2_OIS2N* = TIM_CR2_OIS2N_Msk
  TIM_CR2_OIS3_Pos* = (12)
  TIM_CR2_OIS3_Msk* = (0x00000001 shl TIM_CR2_OIS3_Pos) ## !< 0x00001000
  TIM_CR2_OIS3* = TIM_CR2_OIS3_Msk
  TIM_CR2_OIS3N_Pos* = (13)
  TIM_CR2_OIS3N_Msk* = (0x00000001 shl TIM_CR2_OIS3N_Pos) ## !< 0x00002000
  TIM_CR2_OIS3N* = TIM_CR2_OIS3N_Msk
  TIM_CR2_OIS4_Pos* = (14)
  TIM_CR2_OIS4_Msk* = (0x00000001 shl TIM_CR2_OIS4_Pos) ## !< 0x00004000
  TIM_CR2_OIS4* = TIM_CR2_OIS4_Msk
  TIM_CR2_OIS5_Pos* = (16)
  TIM_CR2_OIS5_Msk* = (0x00000001 shl TIM_CR2_OIS5_Pos) ## !< 0x00010000
  TIM_CR2_OIS5* = TIM_CR2_OIS5_Msk
  TIM_CR2_OIS6_Pos* = (18)
  TIM_CR2_OIS6_Msk* = (0x00000001 shl TIM_CR2_OIS6_Pos) ## !< 0x00040000
  TIM_CR2_OIS6* = TIM_CR2_OIS6_Msk
  TIM_CR2_MMS2_Pos* = (20)
  TIM_CR2_MMS2_Msk* = (0x0000000F shl TIM_CR2_MMS2_Pos) ## !< 0x00F00000
  TIM_CR2_MMS2x* = TIM_CR2_MMS2_Msk
  TIM_CR2_MMS2_0* = (0x00000001 shl TIM_CR2_MMS2_Pos) ## !< 0x00100000
  TIM_CR2_MMS2_1* = (0x00000002 shl TIM_CR2_MMS2_Pos) ## !< 0x00200000
  TIM_CR2_MMS2_2* = (0x00000004 shl TIM_CR2_MMS2_Pos) ## !< 0x00400000
  TIM_CR2_MMS2_3* = (0x00000008 shl TIM_CR2_MMS2_Pos) ## !< 0x00800000

## ******************  Bit definition for TIM_SMCR register  ******************

const
  TIM_SMCR_SMS_Pos* = (0)
  TIM_SMCR_SMS_Msk* = (0x00010007 shl TIM_SMCR_SMS_Pos) ## !< 0x00010007
  TIM_SMCR_SMS* = TIM_SMCR_SMS_Msk
  TIM_SMCR_SMS_0* = (0x00000001) ## !<Bit 0
  TIM_SMCR_SMS_1* = (0x00000002) ## !<Bit 1
  TIM_SMCR_SMS_2* = (0x00000004) ## !<Bit 2
  TIM_SMCR_SMS_3* = (0x00010000) ## !<Bit 3
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
  TIM_DIER_COMIE_Pos* = (5)
  TIM_DIER_COMIE_Msk* = (0x00000001 shl TIM_DIER_COMIE_Pos) ## !< 0x00000020
  TIM_DIER_COMIE* = TIM_DIER_COMIE_Msk
  TIM_DIER_TIE_Pos* = (6)
  TIM_DIER_TIE_Msk* = (0x00000001 shl TIM_DIER_TIE_Pos) ## !< 0x00000040
  TIM_DIER_TIE* = TIM_DIER_TIE_Msk
  TIM_DIER_BIE_Pos* = (7)
  TIM_DIER_BIE_Msk* = (0x00000001 shl TIM_DIER_BIE_Pos) ## !< 0x00000080
  TIM_DIER_BIE* = TIM_DIER_BIE_Msk
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
  TIM_DIER_COMDE_Pos* = (13)
  TIM_DIER_COMDE_Msk* = (0x00000001 shl TIM_DIER_COMDE_Pos) ## !< 0x00002000
  TIM_DIER_COMDE* = TIM_DIER_COMDE_Msk
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
  TIM_SR_COMIF_Pos* = (5)
  TIM_SR_COMIF_Msk* = (0x00000001 shl TIM_SR_COMIF_Pos) ## !< 0x00000020
  TIM_SR_COMIF* = TIM_SR_COMIF_Msk
  TIM_SR_TIF_Pos* = (6)
  TIM_SR_TIF_Msk* = (0x00000001 shl TIM_SR_TIF_Pos) ## !< 0x00000040
  TIM_SR_TIF* = TIM_SR_TIF_Msk
  TIM_SR_BIF_Pos* = (7)
  TIM_SR_BIF_Msk* = (0x00000001 shl TIM_SR_BIF_Pos) ## !< 0x00000080
  TIM_SR_BIF* = TIM_SR_BIF_Msk
  TIM_SR_B2IF_Pos* = (8)
  TIM_SR_B2IF_Msk* = (0x00000001 shl TIM_SR_B2IF_Pos) ## !< 0x00000100
  TIM_SR_B2IF* = TIM_SR_B2IF_Msk
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
  TIM_SR_CC5IF_Pos* = (16)
  TIM_SR_CC5IF_Msk* = (0x00000001 shl TIM_SR_CC5IF_Pos) ## !< 0x00010000
  TIM_SR_CC5IF* = TIM_SR_CC5IF_Msk
  TIM_SR_CC6IF_Pos* = (17)
  TIM_SR_CC6IF_Msk* = (0x00000001 shl TIM_SR_CC6IF_Pos) ## !< 0x00020000
  TIM_SR_CC6IF* = TIM_SR_CC6IF_Msk

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
  TIM_EGR_COMG_Pos* = (5)
  TIM_EGR_COMG_Msk* = (0x00000001 shl TIM_EGR_COMG_Pos) ## !< 0x00000020
  TIM_EGR_COMG* = TIM_EGR_COMG_Msk
  TIM_EGR_TG_Pos* = (6)
  TIM_EGR_TG_Msk* = (0x00000001 shl TIM_EGR_TG_Pos) ## !< 0x00000040
  TIM_EGR_TG* = TIM_EGR_TG_Msk
  TIM_EGR_BG_Pos* = (7)
  TIM_EGR_BG_Msk* = (0x00000001 shl TIM_EGR_BG_Pos) ## !< 0x00000080
  TIM_EGR_BG* = TIM_EGR_BG_Msk
  TIM_EGR_B2G_Pos* = (8)
  TIM_EGR_B2G_Msk* = (0x00000001 shl TIM_EGR_B2G_Pos) ## !< 0x00000100
  TIM_EGR_B2G* = TIM_EGR_B2G_Msk

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
  TIM_CCMR1_OC1M_Msk* = (0x00001007 shl TIM_CCMR1_OC1M_Pos) ## !< 0x00010070
  TIM_CCMR1_OC1M* = TIM_CCMR1_OC1M_Msk
  TIM_CCMR1_OC1M_0* = (0x00000010) ## !<Bit 0
  TIM_CCMR1_OC1M_1* = (0x00000020) ## !<Bit 1
  TIM_CCMR1_OC1M_2* = (0x00000040) ## !<Bit 2
  TIM_CCMR1_OC1M_3* = (0x00010000) ## !<Bit 3
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
  TIM_CCMR1_OC2M_Msk* = (0x00001007 shl TIM_CCMR1_OC2M_Pos) ## !< 0x01007000
  TIM_CCMR1_OC2M* = TIM_CCMR1_OC2M_Msk
  TIM_CCMR1_OC2M_0* = (0x00001000) ## !<Bit 0
  TIM_CCMR1_OC2M_1* = (0x00002000) ## !<Bit 1
  TIM_CCMR1_OC2M_2* = (0x00004000) ## !<Bit 2
  TIM_CCMR1_OC2M_3* = (0x01000000) ## !<Bit 3
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
  TIM_CCMR2_OC3M_Msk* = (0x00001007 shl TIM_CCMR2_OC3M_Pos) ## !< 0x00010070
  TIM_CCMR2_OC3M* = TIM_CCMR2_OC3M_Msk
  TIM_CCMR2_OC3M_0* = (0x00000010) ## !<Bit 0
  TIM_CCMR2_OC3M_1* = (0x00000020) ## !<Bit 1
  TIM_CCMR2_OC3M_2* = (0x00000040) ## !<Bit 2
  TIM_CCMR2_OC3M_3* = (0x00010000) ## !<Bit 3
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
  TIM_CCMR2_OC4M_Msk* = (0x00001007 shl TIM_CCMR2_OC4M_Pos) ## !< 0x01007000
  TIM_CCMR2_OC4M* = TIM_CCMR2_OC4M_Msk
  TIM_CCMR2_OC4M_0* = (0x00001000) ## !<Bit 0
  TIM_CCMR2_OC4M_1* = (0x00002000) ## !<Bit 1
  TIM_CCMR2_OC4M_2* = (0x00004000) ## !<Bit 2
  TIM_CCMR2_OC4M_3* = (0x01000000) ## !<Bit 3
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
  TIM_CCER_CC1NE_Pos* = (2)
  TIM_CCER_CC1NE_Msk* = (0x00000001 shl TIM_CCER_CC1NE_Pos) ## !< 0x00000004
  TIM_CCER_CC1NE* = TIM_CCER_CC1NE_Msk
  TIM_CCER_CC1NP_Pos* = (3)
  TIM_CCER_CC1NP_Msk* = (0x00000001 shl TIM_CCER_CC1NP_Pos) ## !< 0x00000008
  TIM_CCER_CC1NP* = TIM_CCER_CC1NP_Msk
  TIM_CCER_CC2E_Pos* = (4)
  TIM_CCER_CC2E_Msk* = (0x00000001 shl TIM_CCER_CC2E_Pos) ## !< 0x00000010
  TIM_CCER_CC2E* = TIM_CCER_CC2E_Msk
  TIM_CCER_CC2P_Pos* = (5)
  TIM_CCER_CC2P_Msk* = (0x00000001 shl TIM_CCER_CC2P_Pos) ## !< 0x00000020
  TIM_CCER_CC2P* = TIM_CCER_CC2P_Msk
  TIM_CCER_CC2NE_Pos* = (6)
  TIM_CCER_CC2NE_Msk* = (0x00000001 shl TIM_CCER_CC2NE_Pos) ## !< 0x00000040
  TIM_CCER_CC2NE* = TIM_CCER_CC2NE_Msk
  TIM_CCER_CC2NP_Pos* = (7)
  TIM_CCER_CC2NP_Msk* = (0x00000001 shl TIM_CCER_CC2NP_Pos) ## !< 0x00000080
  TIM_CCER_CC2NP* = TIM_CCER_CC2NP_Msk
  TIM_CCER_CC3E_Pos* = (8)
  TIM_CCER_CC3E_Msk* = (0x00000001 shl TIM_CCER_CC3E_Pos) ## !< 0x00000100
  TIM_CCER_CC3E* = TIM_CCER_CC3E_Msk
  TIM_CCER_CC3P_Pos* = (9)
  TIM_CCER_CC3P_Msk* = (0x00000001 shl TIM_CCER_CC3P_Pos) ## !< 0x00000200
  TIM_CCER_CC3P* = TIM_CCER_CC3P_Msk
  TIM_CCER_CC3NE_Pos* = (10)
  TIM_CCER_CC3NE_Msk* = (0x00000001 shl TIM_CCER_CC3NE_Pos) ## !< 0x00000400
  TIM_CCER_CC3NE* = TIM_CCER_CC3NE_Msk
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
  TIM_CCER_CC5E_Pos* = (16)
  TIM_CCER_CC5E_Msk* = (0x00000001 shl TIM_CCER_CC5E_Pos) ## !< 0x00010000
  TIM_CCER_CC5E* = TIM_CCER_CC5E_Msk
  TIM_CCER_CC5P_Pos* = (17)
  TIM_CCER_CC5P_Msk* = (0x00000001 shl TIM_CCER_CC5P_Pos) ## !< 0x00020000
  TIM_CCER_CC5P* = TIM_CCER_CC5P_Msk
  TIM_CCER_CC6E_Pos* = (20)
  TIM_CCER_CC6E_Msk* = (0x00000001 shl TIM_CCER_CC6E_Pos) ## !< 0x00100000
  TIM_CCER_CC6E* = TIM_CCER_CC6E_Msk
  TIM_CCER_CC6P_Pos* = (21)
  TIM_CCER_CC6P_Msk* = (0x00000001 shl TIM_CCER_CC6P_Pos) ## !< 0x00200000
  TIM_CCER_CC6P* = TIM_CCER_CC6P_Msk

## ******************  Bit definition for TIM_CNT register  *******************

const
  TIM_CNT_CNT_Pos* = (0)
  TIM_CNT_CNT_Msk* = (0xFFFFFFFF shl TIM_CNT_CNT_Pos) ## !< 0xFFFFFFFF
  TIM_CNT_CNT* = TIM_CNT_CNT_Msk
  TIM_CNT_UIFCPY_Pos* = (31)
  TIM_CNT_UIFCPY_Msk* = (0x00000001 shl TIM_CNT_UIFCPY_Pos) ## !< 0x80000000
  TIM_CNT_UIFCPY* = TIM_CNT_UIFCPY_Msk

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

## ******************  Bit definition for TIM_RCR register  *******************

const
  TIM_RCR_REP_Pos* = (0)
  TIM_RCR_REP_Msk* = (0x0000FFFF shl TIM_RCR_REP_Pos) ## !< 0x0000FFFF
  TIM_RCR_REP* = TIM_RCR_REP_Msk

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

## ******************  Bit definition for TIM_CCR5 register  ******************

const
  TIM_CCR5_CCR5_Pos* = (0)
  TIM_CCR5_CCR5_Msk* = (0xFFFFFFFF shl TIM_CCR5_CCR5_Pos) ## !< 0xFFFFFFFF
  TIM_CCR5_CCR5* = TIM_CCR5_CCR5_Msk
  TIM_CCR5_GC5C1_Pos* = (29)
  TIM_CCR5_GC5C1_Msk* = (0x00000001 shl TIM_CCR5_GC5C1_Pos) ## !< 0x20000000
  TIM_CCR5_GC5C1* = TIM_CCR5_GC5C1_Msk
  TIM_CCR5_GC5C2_Pos* = (30)
  TIM_CCR5_GC5C2_Msk* = (0x00000001 shl TIM_CCR5_GC5C2_Pos) ## !< 0x40000000
  TIM_CCR5_GC5C2* = TIM_CCR5_GC5C2_Msk
  TIM_CCR5_GC5C3_Pos* = (31)
  TIM_CCR5_GC5C3_Msk* = (0x00000001 shl TIM_CCR5_GC5C3_Pos) ## !< 0x80000000
  TIM_CCR5_GC5C3* = TIM_CCR5_GC5C3_Msk

## ******************  Bit definition for TIM_CCR6 register  ******************

const
  TIM_CCR6_CCR6_Pos* = (0)
  TIM_CCR6_CCR6_Msk* = (0x0000FFFF shl TIM_CCR6_CCR6_Pos) ## !< 0x0000FFFF
  TIM_CCR6_CCR6* = TIM_CCR6_CCR6_Msk

## ******************  Bit definition for TIM_BDTR register  ******************

const
  TIM_BDTR_DTG_Pos* = (0)
  TIM_BDTR_DTG_Msk* = (0x000000FF shl TIM_BDTR_DTG_Pos) ## !< 0x000000FF
  TIM_BDTR_DTG* = TIM_BDTR_DTG_Msk
  TIM_BDTR_DTG_0* = (0x00000001 shl TIM_BDTR_DTG_Pos) ## !< 0x00000001
  TIM_BDTR_DTG_1* = (0x00000002 shl TIM_BDTR_DTG_Pos) ## !< 0x00000002
  TIM_BDTR_DTG_2* = (0x00000004 shl TIM_BDTR_DTG_Pos) ## !< 0x00000004
  TIM_BDTR_DTG_3* = (0x00000008 shl TIM_BDTR_DTG_Pos) ## !< 0x00000008
  TIM_BDTR_DTG_4* = (0x00000010 shl TIM_BDTR_DTG_Pos) ## !< 0x00000010
  TIM_BDTR_DTG_5* = (0x00000020 shl TIM_BDTR_DTG_Pos) ## !< 0x00000020
  TIM_BDTR_DTG_6* = (0x00000040 shl TIM_BDTR_DTG_Pos) ## !< 0x00000040
  TIM_BDTR_DTG_7* = (0x00000080 shl TIM_BDTR_DTG_Pos) ## !< 0x00000080
  TIM_BDTR_LOCK_Pos* = (8)
  TIM_BDTR_LOCK_Msk* = (0x00000003 shl TIM_BDTR_LOCK_Pos) ## !< 0x00000300
  TIM_BDTR_LOCK* = TIM_BDTR_LOCK_Msk
  TIM_BDTR_LOCK_0* = (0x00000001 shl TIM_BDTR_LOCK_Pos) ## !< 0x00000100
  TIM_BDTR_LOCK_1* = (0x00000002 shl TIM_BDTR_LOCK_Pos) ## !< 0x00000200
  TIM_BDTR_OSSI_Pos* = (10)
  TIM_BDTR_OSSI_Msk* = (0x00000001 shl TIM_BDTR_OSSI_Pos) ## !< 0x00000400
  TIM_BDTR_OSSI* = TIM_BDTR_OSSI_Msk
  TIM_BDTR_OSSR_Pos* = (11)
  TIM_BDTR_OSSR_Msk* = (0x00000001 shl TIM_BDTR_OSSR_Pos) ## !< 0x00000800
  TIM_BDTR_OSSR* = TIM_BDTR_OSSR_Msk
  TIM_BDTR_BKE_Pos* = (12)
  TIM_BDTR_BKE_Msk* = (0x00000001 shl TIM_BDTR_BKE_Pos) ## !< 0x00001000
  TIM_BDTR_BKE* = TIM_BDTR_BKE_Msk
  TIM_BDTR_BKP_Pos* = (13)
  TIM_BDTR_BKP_Msk* = (0x00000001 shl TIM_BDTR_BKP_Pos) ## !< 0x00002000
  TIM_BDTR_BKP* = TIM_BDTR_BKP_Msk
  TIM_BDTR_AOE_Pos* = (14)
  TIM_BDTR_AOE_Msk* = (0x00000001 shl TIM_BDTR_AOE_Pos) ## !< 0x00004000
  TIM_BDTR_AOE* = TIM_BDTR_AOE_Msk
  TIM_BDTR_MOE_Pos* = (15)
  TIM_BDTR_MOE_Msk* = (0x00000001 shl TIM_BDTR_MOE_Pos) ## !< 0x00008000
  TIM_BDTR_MOE* = TIM_BDTR_MOE_Msk
  TIM_BDTR_BKF_Pos* = (16)
  TIM_BDTR_BKF_Msk* = (0x0000000F shl TIM_BDTR_BKF_Pos) ## !< 0x000F0000
  TIM_BDTR_BKF* = TIM_BDTR_BKF_Msk
  TIM_BDTR_BK2F_Pos* = (20)
  TIM_BDTR_BK2F_Msk* = (0x0000000F shl TIM_BDTR_BK2F_Pos) ## !< 0x00F00000
  TIM_BDTR_BK2F* = TIM_BDTR_BK2F_Msk
  TIM_BDTR_BK2E_Pos* = (24)
  TIM_BDTR_BK2E_Msk* = (0x00000001 shl TIM_BDTR_BK2E_Pos) ## !< 0x01000000
  TIM_BDTR_BK2E* = TIM_BDTR_BK2E_Msk
  TIM_BDTR_BK2P_Pos* = (25)
  TIM_BDTR_BK2P_Msk* = (0x00000001 shl TIM_BDTR_BK2P_Pos) ## !< 0x02000000
  TIM_BDTR_BK2P* = TIM_BDTR_BK2P_Msk

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

## ******************  Bit definition for TIM16_OR register  ********************

const
  TIM16_OR_TI1_RMP_Pos* = (0)
  TIM16_OR_TI1_RMP_Msk* = (0x00000003 shl TIM16_OR_TI1_RMP_Pos) ## !< 0x00000003
  TIM16_OR_TI1_RMP* = TIM16_OR_TI1_RMP_Msk
  TIM16_OR_TI1_RMP_0* = (0x00000001 shl TIM16_OR_TI1_RMP_Pos) ## !< 0x00000001
  TIM16_OR_TI1_RMP_1* = (0x00000002 shl TIM16_OR_TI1_RMP_Pos) ## !< 0x00000002

## ******************  Bit definition for TIM1_OR register  ********************

const
  TIM1_OR_ETR_RMP_Pos* = (0)
  TIM1_OR_ETR_RMP_Msk* = (0x0000000F shl TIM1_OR_ETR_RMP_Pos) ## !< 0x0000000F
  TIM1_OR_ETR_RMP* = TIM1_OR_ETR_RMP_Msk
  TIM1_OR_ETR_RMP_0* = (0x00000001 shl TIM1_OR_ETR_RMP_Pos) ## !< 0x00000001
  TIM1_OR_ETR_RMP_1* = (0x00000002 shl TIM1_OR_ETR_RMP_Pos) ## !< 0x00000002
  TIM1_OR_ETR_RMP_2* = (0x00000004 shl TIM1_OR_ETR_RMP_Pos) ## !< 0x00000004
  TIM1_OR_ETR_RMP_3* = (0x00000008 shl TIM1_OR_ETR_RMP_Pos) ## !< 0x00000008

## ******************  Bit definition for TIM8_OR register  ********************

const
  TIM8_OR_ETR_RMP_Pos* = (0)
  TIM8_OR_ETR_RMP_Msk* = (0x0000000F shl TIM8_OR_ETR_RMP_Pos) ## !< 0x0000000F
  TIM8_OR_ETR_RMP* = TIM8_OR_ETR_RMP_Msk
  TIM8_OR_ETR_RMP_0* = (0x00000001 shl TIM8_OR_ETR_RMP_Pos) ## !< 0x00000001
  TIM8_OR_ETR_RMP_1* = (0x00000002 shl TIM8_OR_ETR_RMP_Pos) ## !< 0x00000002
  TIM8_OR_ETR_RMP_2* = (0x00000004 shl TIM8_OR_ETR_RMP_Pos) ## !< 0x00000004
  TIM8_OR_ETR_RMP_3* = (0x00000008 shl TIM8_OR_ETR_RMP_Pos) ## !< 0x00000008

## *****************  Bit definition for TIM_CCMR3 register  ******************

const
  TIM_CCMR3_OC5FE_Pos* = (2)
  TIM_CCMR3_OC5FE_Msk* = (0x00000001 shl TIM_CCMR3_OC5FE_Pos) ## !< 0x00000004
  TIM_CCMR3_OC5FE* = TIM_CCMR3_OC5FE_Msk
  TIM_CCMR3_OC5PE_Pos* = (3)
  TIM_CCMR3_OC5PE_Msk* = (0x00000001 shl TIM_CCMR3_OC5PE_Pos) ## !< 0x00000008
  TIM_CCMR3_OC5PE* = TIM_CCMR3_OC5PE_Msk
  TIM_CCMR3_OC5M_Pos* = (4)
  TIM_CCMR3_OC5M_Msk* = (0x00001007 shl TIM_CCMR3_OC5M_Pos) ## !< 0x00010070
  TIM_CCMR3_OC5M* = TIM_CCMR3_OC5M_Msk
  TIM_CCMR3_OC5M_0* = (0x00000001 shl TIM_CCMR3_OC5M_Pos) ## !< 0x00000010
  TIM_CCMR3_OC5M_1* = (0x00000002 shl TIM_CCMR3_OC5M_Pos) ## !< 0x00000020
  TIM_CCMR3_OC5M_2* = (0x00000004 shl TIM_CCMR3_OC5M_Pos) ## !< 0x00000040
  TIM_CCMR3_OC5M_3* = (0x00001000 shl TIM_CCMR3_OC5M_Pos) ## !< 0x00010000
  TIM_CCMR3_OC5CE_Pos* = (7)
  TIM_CCMR3_OC5CE_Msk* = (0x00000001 shl TIM_CCMR3_OC5CE_Pos) ## !< 0x00000080
  TIM_CCMR3_OC5CE* = TIM_CCMR3_OC5CE_Msk
  TIM_CCMR3_OC6FE_Pos* = (10)
  TIM_CCMR3_OC6FE_Msk* = (0x00000001 shl TIM_CCMR3_OC6FE_Pos) ## !< 0x00000400
  TIM_CCMR3_OC6FE* = TIM_CCMR3_OC6FE_Msk
  TIM_CCMR3_OC6PE_Pos* = (11)
  TIM_CCMR3_OC6PE_Msk* = (0x00000001 shl TIM_CCMR3_OC6PE_Pos) ## !< 0x00000800
  TIM_CCMR3_OC6PE* = TIM_CCMR3_OC6PE_Msk
  TIM_CCMR3_OC6M_Pos* = (12)
  TIM_CCMR3_OC6M_Msk* = (0x00001007 shl TIM_CCMR3_OC6M_Pos) ## !< 0x01007000
  TIM_CCMR3_OC6M* = TIM_CCMR3_OC6M_Msk
  TIM_CCMR3_OC6M_0* = (0x00000001 shl TIM_CCMR3_OC6M_Pos) ## !< 0x00001000
  TIM_CCMR3_OC6M_1* = (0x00000002 shl TIM_CCMR3_OC6M_Pos) ## !< 0x00002000
  TIM_CCMR3_OC6M_2* = (0x00000004 shl TIM_CCMR3_OC6M_Pos) ## !< 0x00004000
  TIM_CCMR3_OC6M_3* = (0x00001000 shl TIM_CCMR3_OC6M_Pos) ## !< 0x01000000
  TIM_CCMR3_OC6CE_Pos* = (15)
  TIM_CCMR3_OC6CE_Msk* = (0x00000001 shl TIM_CCMR3_OC6CE_Pos) ## !< 0x00008000
  TIM_CCMR3_OC6CE* = TIM_CCMR3_OC6CE_Msk

## ****************************************************************************
##
##                           Touch Sensing Controller (TSC)
##
## ****************************************************************************
## ******************  Bit definition for TSC_CR register  ********************

const
  TSC_CR_TSCE_Pos* = (0)
  TSC_CR_TSCE_Msk* = (0x00000001 shl TSC_CR_TSCE_Pos) ## !< 0x00000001
  TSC_CR_TSCE* = TSC_CR_TSCE_Msk
  TSC_CR_START_Pos* = (1)
  TSC_CR_START_Msk* = (0x00000001 shl TSC_CR_START_Pos) ## !< 0x00000002
  TSC_CR_START* = TSC_CR_START_Msk
  TSC_CR_AM_Pos* = (2)
  TSC_CR_AM_Msk* = (0x00000001 shl TSC_CR_AM_Pos) ## !< 0x00000004
  TSC_CR_AM* = TSC_CR_AM_Msk
  TSC_CR_SYNCPOL_Pos* = (3)
  TSC_CR_SYNCPOL_Msk* = (0x00000001 shl TSC_CR_SYNCPOL_Pos) ## !< 0x00000008
  TSC_CR_SYNCPOL* = TSC_CR_SYNCPOL_Msk
  TSC_CR_IODEF_Pos* = (4)
  TSC_CR_IODEF_Msk* = (0x00000001 shl TSC_CR_IODEF_Pos) ## !< 0x00000010
  TSC_CR_IODEF* = TSC_CR_IODEF_Msk
  TSC_CR_MCV_Pos* = (5)
  TSC_CR_MCV_Msk* = (0x00000007 shl TSC_CR_MCV_Pos) ## !< 0x000000E0
  TSC_CR_MCV* = TSC_CR_MCV_Msk
  TSC_CR_MCV_0* = (0x00000001 shl TSC_CR_MCV_Pos) ## !< 0x00000020
  TSC_CR_MCV_1* = (0x00000002 shl TSC_CR_MCV_Pos) ## !< 0x00000040
  TSC_CR_MCV_2* = (0x00000004 shl TSC_CR_MCV_Pos) ## !< 0x00000080
  TSC_CR_PGPSC_Pos* = (12)
  TSC_CR_PGPSC_Msk* = (0x00000007 shl TSC_CR_PGPSC_Pos) ## !< 0x00007000
  TSC_CR_PGPSC* = TSC_CR_PGPSC_Msk
  TSC_CR_PGPSC_0* = (0x00000001 shl TSC_CR_PGPSC_Pos) ## !< 0x00001000
  TSC_CR_PGPSC_1* = (0x00000002 shl TSC_CR_PGPSC_Pos) ## !< 0x00002000
  TSC_CR_PGPSC_2* = (0x00000004 shl TSC_CR_PGPSC_Pos) ## !< 0x00004000
  TSC_CR_SSPSC_Pos* = (15)
  TSC_CR_SSPSC_Msk* = (0x00000001 shl TSC_CR_SSPSC_Pos) ## !< 0x00008000
  TSC_CR_SSPSC* = TSC_CR_SSPSC_Msk
  TSC_CR_SSE_Pos* = (16)
  TSC_CR_SSE_Msk* = (0x00000001 shl TSC_CR_SSE_Pos) ## !< 0x00010000
  TSC_CR_SSE* = TSC_CR_SSE_Msk
  TSC_CR_SSD_Pos* = (17)
  TSC_CR_SSD_Msk* = (0x0000007F shl TSC_CR_SSD_Pos) ## !< 0x00FE0000
  TSC_CR_SSD* = TSC_CR_SSD_Msk
  TSC_CR_SSD_0* = (0x00000001 shl TSC_CR_SSD_Pos) ## !< 0x00020000
  TSC_CR_SSD_1* = (0x00000002 shl TSC_CR_SSD_Pos) ## !< 0x00040000
  TSC_CR_SSD_2* = (0x00000004 shl TSC_CR_SSD_Pos) ## !< 0x00080000
  TSC_CR_SSD_3* = (0x00000008 shl TSC_CR_SSD_Pos) ## !< 0x00100000
  TSC_CR_SSD_4* = (0x00000010 shl TSC_CR_SSD_Pos) ## !< 0x00200000
  TSC_CR_SSD_5* = (0x00000020 shl TSC_CR_SSD_Pos) ## !< 0x00400000
  TSC_CR_SSD_6* = (0x00000040 shl TSC_CR_SSD_Pos) ## !< 0x00800000
  TSC_CR_CTPL_Pos* = (24)
  TSC_CR_CTPL_Msk* = (0x0000000F shl TSC_CR_CTPL_Pos) ## !< 0x0F000000
  TSC_CR_CTPL* = TSC_CR_CTPL_Msk
  TSC_CR_CTPL_0* = (0x00000001 shl TSC_CR_CTPL_Pos) ## !< 0x01000000
  TSC_CR_CTPL_1* = (0x00000002 shl TSC_CR_CTPL_Pos) ## !< 0x02000000
  TSC_CR_CTPL_2* = (0x00000004 shl TSC_CR_CTPL_Pos) ## !< 0x04000000
  TSC_CR_CTPL_3* = (0x00000008 shl TSC_CR_CTPL_Pos) ## !< 0x08000000
  TSC_CR_CTPH_Pos* = (28)
  TSC_CR_CTPH_Msk* = (0x0000000F shl TSC_CR_CTPH_Pos) ## !< 0xF0000000
  TSC_CR_CTPH* = TSC_CR_CTPH_Msk
  TSC_CR_CTPH_0* = (0x00000001 shl TSC_CR_CTPH_Pos) ## !< 0x10000000
  TSC_CR_CTPH_1* = (0x00000002 shl TSC_CR_CTPH_Pos) ## !< 0x20000000
  TSC_CR_CTPH_2* = (0x00000004 shl TSC_CR_CTPH_Pos) ## !< 0x40000000
  TSC_CR_CTPH_3* = (0x00000008 shl TSC_CR_CTPH_Pos) ## !< 0x80000000

## ******************  Bit definition for TSC_IER register  *******************

const
  TSC_IER_EOAIE_Pos* = (0)
  TSC_IER_EOAIE_Msk* = (0x00000001 shl TSC_IER_EOAIE_Pos) ## !< 0x00000001
  TSC_IER_EOAIE* = TSC_IER_EOAIE_Msk
  TSC_IER_MCEIE_Pos* = (1)
  TSC_IER_MCEIE_Msk* = (0x00000001 shl TSC_IER_MCEIE_Pos) ## !< 0x00000002
  TSC_IER_MCEIE* = TSC_IER_MCEIE_Msk

## ******************  Bit definition for TSC_ICR register  *******************

const
  TSC_ICR_EOAIC_Pos* = (0)
  TSC_ICR_EOAIC_Msk* = (0x00000001 shl TSC_ICR_EOAIC_Pos) ## !< 0x00000001
  TSC_ICR_EOAIC* = TSC_ICR_EOAIC_Msk
  TSC_ICR_MCEIC_Pos* = (1)
  TSC_ICR_MCEIC_Msk* = (0x00000001 shl TSC_ICR_MCEIC_Pos) ## !< 0x00000002
  TSC_ICR_MCEIC* = TSC_ICR_MCEIC_Msk

## ******************  Bit definition for TSC_ISR register  *******************

const
  TSC_ISR_EOAF_Pos* = (0)
  TSC_ISR_EOAF_Msk* = (0x00000001 shl TSC_ISR_EOAF_Pos) ## !< 0x00000001
  TSC_ISR_EOAF* = TSC_ISR_EOAF_Msk
  TSC_ISR_MCEF_Pos* = (1)
  TSC_ISR_MCEF_Msk* = (0x00000001 shl TSC_ISR_MCEF_Pos) ## !< 0x00000002
  TSC_ISR_MCEF* = TSC_ISR_MCEF_Msk

## ******************  Bit definition for TSC_IOHCR register  *****************

const
  TSC_IOHCR_G1_IO1_Pos* = (0)
  TSC_IOHCR_G1_IO1_Msk* = (0x00000001 shl TSC_IOHCR_G1_IO1_Pos) ## !< 0x00000001
  TSC_IOHCR_G1_IO1* = TSC_IOHCR_G1_IO1_Msk
  TSC_IOHCR_G1_IO2_Pos* = (1)
  TSC_IOHCR_G1_IO2_Msk* = (0x00000001 shl TSC_IOHCR_G1_IO2_Pos) ## !< 0x00000002
  TSC_IOHCR_G1_IO2* = TSC_IOHCR_G1_IO2_Msk
  TSC_IOHCR_G1_IO3_Pos* = (2)
  TSC_IOHCR_G1_IO3_Msk* = (0x00000001 shl TSC_IOHCR_G1_IO3_Pos) ## !< 0x00000004
  TSC_IOHCR_G1_IO3* = TSC_IOHCR_G1_IO3_Msk
  TSC_IOHCR_G1_IO4_Pos* = (3)
  TSC_IOHCR_G1_IO4_Msk* = (0x00000001 shl TSC_IOHCR_G1_IO4_Pos) ## !< 0x00000008
  TSC_IOHCR_G1_IO4* = TSC_IOHCR_G1_IO4_Msk
  TSC_IOHCR_G2_IO1_Pos* = (4)
  TSC_IOHCR_G2_IO1_Msk* = (0x00000001 shl TSC_IOHCR_G2_IO1_Pos) ## !< 0x00000010
  TSC_IOHCR_G2_IO1* = TSC_IOHCR_G2_IO1_Msk
  TSC_IOHCR_G2_IO2_Pos* = (5)
  TSC_IOHCR_G2_IO2_Msk* = (0x00000001 shl TSC_IOHCR_G2_IO2_Pos) ## !< 0x00000020
  TSC_IOHCR_G2_IO2* = TSC_IOHCR_G2_IO2_Msk
  TSC_IOHCR_G2_IO3_Pos* = (6)
  TSC_IOHCR_G2_IO3_Msk* = (0x00000001 shl TSC_IOHCR_G2_IO3_Pos) ## !< 0x00000040
  TSC_IOHCR_G2_IO3* = TSC_IOHCR_G2_IO3_Msk
  TSC_IOHCR_G2_IO4_Pos* = (7)
  TSC_IOHCR_G2_IO4_Msk* = (0x00000001 shl TSC_IOHCR_G2_IO4_Pos) ## !< 0x00000080
  TSC_IOHCR_G2_IO4* = TSC_IOHCR_G2_IO4_Msk
  TSC_IOHCR_G3_IO1_Pos* = (8)
  TSC_IOHCR_G3_IO1_Msk* = (0x00000001 shl TSC_IOHCR_G3_IO1_Pos) ## !< 0x00000100
  TSC_IOHCR_G3_IO1* = TSC_IOHCR_G3_IO1_Msk
  TSC_IOHCR_G3_IO2_Pos* = (9)
  TSC_IOHCR_G3_IO2_Msk* = (0x00000001 shl TSC_IOHCR_G3_IO2_Pos) ## !< 0x00000200
  TSC_IOHCR_G3_IO2* = TSC_IOHCR_G3_IO2_Msk
  TSC_IOHCR_G3_IO3_Pos* = (10)
  TSC_IOHCR_G3_IO3_Msk* = (0x00000001 shl TSC_IOHCR_G3_IO3_Pos) ## !< 0x00000400
  TSC_IOHCR_G3_IO3* = TSC_IOHCR_G3_IO3_Msk
  TSC_IOHCR_G3_IO4_Pos* = (11)
  TSC_IOHCR_G3_IO4_Msk* = (0x00000001 shl TSC_IOHCR_G3_IO4_Pos) ## !< 0x00000800
  TSC_IOHCR_G3_IO4* = TSC_IOHCR_G3_IO4_Msk
  TSC_IOHCR_G4_IO1_Pos* = (12)
  TSC_IOHCR_G4_IO1_Msk* = (0x00000001 shl TSC_IOHCR_G4_IO1_Pos) ## !< 0x00001000
  TSC_IOHCR_G4_IO1* = TSC_IOHCR_G4_IO1_Msk
  TSC_IOHCR_G4_IO2_Pos* = (13)
  TSC_IOHCR_G4_IO2_Msk* = (0x00000001 shl TSC_IOHCR_G4_IO2_Pos) ## !< 0x00002000
  TSC_IOHCR_G4_IO2* = TSC_IOHCR_G4_IO2_Msk
  TSC_IOHCR_G4_IO3_Pos* = (14)
  TSC_IOHCR_G4_IO3_Msk* = (0x00000001 shl TSC_IOHCR_G4_IO3_Pos) ## !< 0x00004000
  TSC_IOHCR_G4_IO3* = TSC_IOHCR_G4_IO3_Msk
  TSC_IOHCR_G4_IO4_Pos* = (15)
  TSC_IOHCR_G4_IO4_Msk* = (0x00000001 shl TSC_IOHCR_G4_IO4_Pos) ## !< 0x00008000
  TSC_IOHCR_G4_IO4* = TSC_IOHCR_G4_IO4_Msk
  TSC_IOHCR_G5_IO1_Pos* = (16)
  TSC_IOHCR_G5_IO1_Msk* = (0x00000001 shl TSC_IOHCR_G5_IO1_Pos) ## !< 0x00010000
  TSC_IOHCR_G5_IO1* = TSC_IOHCR_G5_IO1_Msk
  TSC_IOHCR_G5_IO2_Pos* = (17)
  TSC_IOHCR_G5_IO2_Msk* = (0x00000001 shl TSC_IOHCR_G5_IO2_Pos) ## !< 0x00020000
  TSC_IOHCR_G5_IO2* = TSC_IOHCR_G5_IO2_Msk
  TSC_IOHCR_G5_IO3_Pos* = (18)
  TSC_IOHCR_G5_IO3_Msk* = (0x00000001 shl TSC_IOHCR_G5_IO3_Pos) ## !< 0x00040000
  TSC_IOHCR_G5_IO3* = TSC_IOHCR_G5_IO3_Msk
  TSC_IOHCR_G5_IO4_Pos* = (19)
  TSC_IOHCR_G5_IO4_Msk* = (0x00000001 shl TSC_IOHCR_G5_IO4_Pos) ## !< 0x00080000
  TSC_IOHCR_G5_IO4* = TSC_IOHCR_G5_IO4_Msk
  TSC_IOHCR_G6_IO1_Pos* = (20)
  TSC_IOHCR_G6_IO1_Msk* = (0x00000001 shl TSC_IOHCR_G6_IO1_Pos) ## !< 0x00100000
  TSC_IOHCR_G6_IO1* = TSC_IOHCR_G6_IO1_Msk
  TSC_IOHCR_G6_IO2_Pos* = (21)
  TSC_IOHCR_G6_IO2_Msk* = (0x00000001 shl TSC_IOHCR_G6_IO2_Pos) ## !< 0x00200000
  TSC_IOHCR_G6_IO2* = TSC_IOHCR_G6_IO2_Msk
  TSC_IOHCR_G6_IO3_Pos* = (22)
  TSC_IOHCR_G6_IO3_Msk* = (0x00000001 shl TSC_IOHCR_G6_IO3_Pos) ## !< 0x00400000
  TSC_IOHCR_G6_IO3* = TSC_IOHCR_G6_IO3_Msk
  TSC_IOHCR_G6_IO4_Pos* = (23)
  TSC_IOHCR_G6_IO4_Msk* = (0x00000001 shl TSC_IOHCR_G6_IO4_Pos) ## !< 0x00800000
  TSC_IOHCR_G6_IO4* = TSC_IOHCR_G6_IO4_Msk
  TSC_IOHCR_G7_IO1_Pos* = (24)
  TSC_IOHCR_G7_IO1_Msk* = (0x00000001 shl TSC_IOHCR_G7_IO1_Pos) ## !< 0x01000000
  TSC_IOHCR_G7_IO1* = TSC_IOHCR_G7_IO1_Msk
  TSC_IOHCR_G7_IO2_Pos* = (25)
  TSC_IOHCR_G7_IO2_Msk* = (0x00000001 shl TSC_IOHCR_G7_IO2_Pos) ## !< 0x02000000
  TSC_IOHCR_G7_IO2* = TSC_IOHCR_G7_IO2_Msk
  TSC_IOHCR_G7_IO3_Pos* = (26)
  TSC_IOHCR_G7_IO3_Msk* = (0x00000001 shl TSC_IOHCR_G7_IO3_Pos) ## !< 0x04000000
  TSC_IOHCR_G7_IO3* = TSC_IOHCR_G7_IO3_Msk
  TSC_IOHCR_G7_IO4_Pos* = (27)
  TSC_IOHCR_G7_IO4_Msk* = (0x00000001 shl TSC_IOHCR_G7_IO4_Pos) ## !< 0x08000000
  TSC_IOHCR_G7_IO4* = TSC_IOHCR_G7_IO4_Msk
  TSC_IOHCR_G8_IO1_Pos* = (28)
  TSC_IOHCR_G8_IO1_Msk* = (0x00000001 shl TSC_IOHCR_G8_IO1_Pos) ## !< 0x10000000
  TSC_IOHCR_G8_IO1* = TSC_IOHCR_G8_IO1_Msk
  TSC_IOHCR_G8_IO2_Pos* = (29)
  TSC_IOHCR_G8_IO2_Msk* = (0x00000001 shl TSC_IOHCR_G8_IO2_Pos) ## !< 0x20000000
  TSC_IOHCR_G8_IO2* = TSC_IOHCR_G8_IO2_Msk
  TSC_IOHCR_G8_IO3_Pos* = (30)
  TSC_IOHCR_G8_IO3_Msk* = (0x00000001 shl TSC_IOHCR_G8_IO3_Pos) ## !< 0x40000000
  TSC_IOHCR_G8_IO3* = TSC_IOHCR_G8_IO3_Msk
  TSC_IOHCR_G8_IO4_Pos* = (31)
  TSC_IOHCR_G8_IO4_Msk* = (0x00000001 shl TSC_IOHCR_G8_IO4_Pos) ## !< 0x80000000
  TSC_IOHCR_G8_IO4* = TSC_IOHCR_G8_IO4_Msk

## ******************  Bit definition for TSC_IOASCR register  ****************

const
  TSC_IOASCR_G1_IO1_Pos* = (0)
  TSC_IOASCR_G1_IO1_Msk* = (0x00000001 shl TSC_IOASCR_G1_IO1_Pos) ## !< 0x00000001
  TSC_IOASCR_G1_IO1* = TSC_IOASCR_G1_IO1_Msk
  TSC_IOASCR_G1_IO2_Pos* = (1)
  TSC_IOASCR_G1_IO2_Msk* = (0x00000001 shl TSC_IOASCR_G1_IO2_Pos) ## !< 0x00000002
  TSC_IOASCR_G1_IO2* = TSC_IOASCR_G1_IO2_Msk
  TSC_IOASCR_G1_IO3_Pos* = (2)
  TSC_IOASCR_G1_IO3_Msk* = (0x00000001 shl TSC_IOASCR_G1_IO3_Pos) ## !< 0x00000004
  TSC_IOASCR_G1_IO3* = TSC_IOASCR_G1_IO3_Msk
  TSC_IOASCR_G1_IO4_Pos* = (3)
  TSC_IOASCR_G1_IO4_Msk* = (0x00000001 shl TSC_IOASCR_G1_IO4_Pos) ## !< 0x00000008
  TSC_IOASCR_G1_IO4* = TSC_IOASCR_G1_IO4_Msk
  TSC_IOASCR_G2_IO1_Pos* = (4)
  TSC_IOASCR_G2_IO1_Msk* = (0x00000001 shl TSC_IOASCR_G2_IO1_Pos) ## !< 0x00000010
  TSC_IOASCR_G2_IO1* = TSC_IOASCR_G2_IO1_Msk
  TSC_IOASCR_G2_IO2_Pos* = (5)
  TSC_IOASCR_G2_IO2_Msk* = (0x00000001 shl TSC_IOASCR_G2_IO2_Pos) ## !< 0x00000020
  TSC_IOASCR_G2_IO2* = TSC_IOASCR_G2_IO2_Msk
  TSC_IOASCR_G2_IO3_Pos* = (6)
  TSC_IOASCR_G2_IO3_Msk* = (0x00000001 shl TSC_IOASCR_G2_IO3_Pos) ## !< 0x00000040
  TSC_IOASCR_G2_IO3* = TSC_IOASCR_G2_IO3_Msk
  TSC_IOASCR_G2_IO4_Pos* = (7)
  TSC_IOASCR_G2_IO4_Msk* = (0x00000001 shl TSC_IOASCR_G2_IO4_Pos) ## !< 0x00000080
  TSC_IOASCR_G2_IO4* = TSC_IOASCR_G2_IO4_Msk
  TSC_IOASCR_G3_IO1_Pos* = (8)
  TSC_IOASCR_G3_IO1_Msk* = (0x00000001 shl TSC_IOASCR_G3_IO1_Pos) ## !< 0x00000100
  TSC_IOASCR_G3_IO1* = TSC_IOASCR_G3_IO1_Msk
  TSC_IOASCR_G3_IO2_Pos* = (9)
  TSC_IOASCR_G3_IO2_Msk* = (0x00000001 shl TSC_IOASCR_G3_IO2_Pos) ## !< 0x00000200
  TSC_IOASCR_G3_IO2* = TSC_IOASCR_G3_IO2_Msk
  TSC_IOASCR_G3_IO3_Pos* = (10)
  TSC_IOASCR_G3_IO3_Msk* = (0x00000001 shl TSC_IOASCR_G3_IO3_Pos) ## !< 0x00000400
  TSC_IOASCR_G3_IO3* = TSC_IOASCR_G3_IO3_Msk
  TSC_IOASCR_G3_IO4_Pos* = (11)
  TSC_IOASCR_G3_IO4_Msk* = (0x00000001 shl TSC_IOASCR_G3_IO4_Pos) ## !< 0x00000800
  TSC_IOASCR_G3_IO4* = TSC_IOASCR_G3_IO4_Msk
  TSC_IOASCR_G4_IO1_Pos* = (12)
  TSC_IOASCR_G4_IO1_Msk* = (0x00000001 shl TSC_IOASCR_G4_IO1_Pos) ## !< 0x00001000
  TSC_IOASCR_G4_IO1* = TSC_IOASCR_G4_IO1_Msk
  TSC_IOASCR_G4_IO2_Pos* = (13)
  TSC_IOASCR_G4_IO2_Msk* = (0x00000001 shl TSC_IOASCR_G4_IO2_Pos) ## !< 0x00002000
  TSC_IOASCR_G4_IO2* = TSC_IOASCR_G4_IO2_Msk
  TSC_IOASCR_G4_IO3_Pos* = (14)
  TSC_IOASCR_G4_IO3_Msk* = (0x00000001 shl TSC_IOASCR_G4_IO3_Pos) ## !< 0x00004000
  TSC_IOASCR_G4_IO3* = TSC_IOASCR_G4_IO3_Msk
  TSC_IOASCR_G4_IO4_Pos* = (15)
  TSC_IOASCR_G4_IO4_Msk* = (0x00000001 shl TSC_IOASCR_G4_IO4_Pos) ## !< 0x00008000
  TSC_IOASCR_G4_IO4* = TSC_IOASCR_G4_IO4_Msk
  TSC_IOASCR_G5_IO1_Pos* = (16)
  TSC_IOASCR_G5_IO1_Msk* = (0x00000001 shl TSC_IOASCR_G5_IO1_Pos) ## !< 0x00010000
  TSC_IOASCR_G5_IO1* = TSC_IOASCR_G5_IO1_Msk
  TSC_IOASCR_G5_IO2_Pos* = (17)
  TSC_IOASCR_G5_IO2_Msk* = (0x00000001 shl TSC_IOASCR_G5_IO2_Pos) ## !< 0x00020000
  TSC_IOASCR_G5_IO2* = TSC_IOASCR_G5_IO2_Msk
  TSC_IOASCR_G5_IO3_Pos* = (18)
  TSC_IOASCR_G5_IO3_Msk* = (0x00000001 shl TSC_IOASCR_G5_IO3_Pos) ## !< 0x00040000
  TSC_IOASCR_G5_IO3* = TSC_IOASCR_G5_IO3_Msk
  TSC_IOASCR_G5_IO4_Pos* = (19)
  TSC_IOASCR_G5_IO4_Msk* = (0x00000001 shl TSC_IOASCR_G5_IO4_Pos) ## !< 0x00080000
  TSC_IOASCR_G5_IO4* = TSC_IOASCR_G5_IO4_Msk
  TSC_IOASCR_G6_IO1_Pos* = (20)
  TSC_IOASCR_G6_IO1_Msk* = (0x00000001 shl TSC_IOASCR_G6_IO1_Pos) ## !< 0x00100000
  TSC_IOASCR_G6_IO1* = TSC_IOASCR_G6_IO1_Msk
  TSC_IOASCR_G6_IO2_Pos* = (21)
  TSC_IOASCR_G6_IO2_Msk* = (0x00000001 shl TSC_IOASCR_G6_IO2_Pos) ## !< 0x00200000
  TSC_IOASCR_G6_IO2* = TSC_IOASCR_G6_IO2_Msk
  TSC_IOASCR_G6_IO3_Pos* = (22)
  TSC_IOASCR_G6_IO3_Msk* = (0x00000001 shl TSC_IOASCR_G6_IO3_Pos) ## !< 0x00400000
  TSC_IOASCR_G6_IO3* = TSC_IOASCR_G6_IO3_Msk
  TSC_IOASCR_G6_IO4_Pos* = (23)
  TSC_IOASCR_G6_IO4_Msk* = (0x00000001 shl TSC_IOASCR_G6_IO4_Pos) ## !< 0x00800000
  TSC_IOASCR_G6_IO4* = TSC_IOASCR_G6_IO4_Msk
  TSC_IOASCR_G7_IO1_Pos* = (24)
  TSC_IOASCR_G7_IO1_Msk* = (0x00000001 shl TSC_IOASCR_G7_IO1_Pos) ## !< 0x01000000
  TSC_IOASCR_G7_IO1* = TSC_IOASCR_G7_IO1_Msk
  TSC_IOASCR_G7_IO2_Pos* = (25)
  TSC_IOASCR_G7_IO2_Msk* = (0x00000001 shl TSC_IOASCR_G7_IO2_Pos) ## !< 0x02000000
  TSC_IOASCR_G7_IO2* = TSC_IOASCR_G7_IO2_Msk
  TSC_IOASCR_G7_IO3_Pos* = (26)
  TSC_IOASCR_G7_IO3_Msk* = (0x00000001 shl TSC_IOASCR_G7_IO3_Pos) ## !< 0x04000000
  TSC_IOASCR_G7_IO3* = TSC_IOASCR_G7_IO3_Msk
  TSC_IOASCR_G7_IO4_Pos* = (27)
  TSC_IOASCR_G7_IO4_Msk* = (0x00000001 shl TSC_IOASCR_G7_IO4_Pos) ## !< 0x08000000
  TSC_IOASCR_G7_IO4* = TSC_IOASCR_G7_IO4_Msk
  TSC_IOASCR_G8_IO1_Pos* = (28)
  TSC_IOASCR_G8_IO1_Msk* = (0x00000001 shl TSC_IOASCR_G8_IO1_Pos) ## !< 0x10000000
  TSC_IOASCR_G8_IO1* = TSC_IOASCR_G8_IO1_Msk
  TSC_IOASCR_G8_IO2_Pos* = (29)
  TSC_IOASCR_G8_IO2_Msk* = (0x00000001 shl TSC_IOASCR_G8_IO2_Pos) ## !< 0x20000000
  TSC_IOASCR_G8_IO2* = TSC_IOASCR_G8_IO2_Msk
  TSC_IOASCR_G8_IO3_Pos* = (30)
  TSC_IOASCR_G8_IO3_Msk* = (0x00000001 shl TSC_IOASCR_G8_IO3_Pos) ## !< 0x40000000
  TSC_IOASCR_G8_IO3* = TSC_IOASCR_G8_IO3_Msk
  TSC_IOASCR_G8_IO4_Pos* = (31)
  TSC_IOASCR_G8_IO4_Msk* = (0x00000001 shl TSC_IOASCR_G8_IO4_Pos) ## !< 0x80000000
  TSC_IOASCR_G8_IO4* = TSC_IOASCR_G8_IO4_Msk

## ******************  Bit definition for TSC_IOSCR register  *****************

const
  TSC_IOSCR_G1_IO1_Pos* = (0)
  TSC_IOSCR_G1_IO1_Msk* = (0x00000001 shl TSC_IOSCR_G1_IO1_Pos) ## !< 0x00000001
  TSC_IOSCR_G1_IO1* = TSC_IOSCR_G1_IO1_Msk
  TSC_IOSCR_G1_IO2_Pos* = (1)
  TSC_IOSCR_G1_IO2_Msk* = (0x00000001 shl TSC_IOSCR_G1_IO2_Pos) ## !< 0x00000002
  TSC_IOSCR_G1_IO2* = TSC_IOSCR_G1_IO2_Msk
  TSC_IOSCR_G1_IO3_Pos* = (2)
  TSC_IOSCR_G1_IO3_Msk* = (0x00000001 shl TSC_IOSCR_G1_IO3_Pos) ## !< 0x00000004
  TSC_IOSCR_G1_IO3* = TSC_IOSCR_G1_IO3_Msk
  TSC_IOSCR_G1_IO4_Pos* = (3)
  TSC_IOSCR_G1_IO4_Msk* = (0x00000001 shl TSC_IOSCR_G1_IO4_Pos) ## !< 0x00000008
  TSC_IOSCR_G1_IO4* = TSC_IOSCR_G1_IO4_Msk
  TSC_IOSCR_G2_IO1_Pos* = (4)
  TSC_IOSCR_G2_IO1_Msk* = (0x00000001 shl TSC_IOSCR_G2_IO1_Pos) ## !< 0x00000010
  TSC_IOSCR_G2_IO1* = TSC_IOSCR_G2_IO1_Msk
  TSC_IOSCR_G2_IO2_Pos* = (5)
  TSC_IOSCR_G2_IO2_Msk* = (0x00000001 shl TSC_IOSCR_G2_IO2_Pos) ## !< 0x00000020
  TSC_IOSCR_G2_IO2* = TSC_IOSCR_G2_IO2_Msk
  TSC_IOSCR_G2_IO3_Pos* = (6)
  TSC_IOSCR_G2_IO3_Msk* = (0x00000001 shl TSC_IOSCR_G2_IO3_Pos) ## !< 0x00000040
  TSC_IOSCR_G2_IO3* = TSC_IOSCR_G2_IO3_Msk
  TSC_IOSCR_G2_IO4_Pos* = (7)
  TSC_IOSCR_G2_IO4_Msk* = (0x00000001 shl TSC_IOSCR_G2_IO4_Pos) ## !< 0x00000080
  TSC_IOSCR_G2_IO4* = TSC_IOSCR_G2_IO4_Msk
  TSC_IOSCR_G3_IO1_Pos* = (8)
  TSC_IOSCR_G3_IO1_Msk* = (0x00000001 shl TSC_IOSCR_G3_IO1_Pos) ## !< 0x00000100
  TSC_IOSCR_G3_IO1* = TSC_IOSCR_G3_IO1_Msk
  TSC_IOSCR_G3_IO2_Pos* = (9)
  TSC_IOSCR_G3_IO2_Msk* = (0x00000001 shl TSC_IOSCR_G3_IO2_Pos) ## !< 0x00000200
  TSC_IOSCR_G3_IO2* = TSC_IOSCR_G3_IO2_Msk
  TSC_IOSCR_G3_IO3_Pos* = (10)
  TSC_IOSCR_G3_IO3_Msk* = (0x00000001 shl TSC_IOSCR_G3_IO3_Pos) ## !< 0x00000400
  TSC_IOSCR_G3_IO3* = TSC_IOSCR_G3_IO3_Msk
  TSC_IOSCR_G3_IO4_Pos* = (11)
  TSC_IOSCR_G3_IO4_Msk* = (0x00000001 shl TSC_IOSCR_G3_IO4_Pos) ## !< 0x00000800
  TSC_IOSCR_G3_IO4* = TSC_IOSCR_G3_IO4_Msk
  TSC_IOSCR_G4_IO1_Pos* = (12)
  TSC_IOSCR_G4_IO1_Msk* = (0x00000001 shl TSC_IOSCR_G4_IO1_Pos) ## !< 0x00001000
  TSC_IOSCR_G4_IO1* = TSC_IOSCR_G4_IO1_Msk
  TSC_IOSCR_G4_IO2_Pos* = (13)
  TSC_IOSCR_G4_IO2_Msk* = (0x00000001 shl TSC_IOSCR_G4_IO2_Pos) ## !< 0x00002000
  TSC_IOSCR_G4_IO2* = TSC_IOSCR_G4_IO2_Msk
  TSC_IOSCR_G4_IO3_Pos* = (14)
  TSC_IOSCR_G4_IO3_Msk* = (0x00000001 shl TSC_IOSCR_G4_IO3_Pos) ## !< 0x00004000
  TSC_IOSCR_G4_IO3* = TSC_IOSCR_G4_IO3_Msk
  TSC_IOSCR_G4_IO4_Pos* = (15)
  TSC_IOSCR_G4_IO4_Msk* = (0x00000001 shl TSC_IOSCR_G4_IO4_Pos) ## !< 0x00008000
  TSC_IOSCR_G4_IO4* = TSC_IOSCR_G4_IO4_Msk
  TSC_IOSCR_G5_IO1_Pos* = (16)
  TSC_IOSCR_G5_IO1_Msk* = (0x00000001 shl TSC_IOSCR_G5_IO1_Pos) ## !< 0x00010000
  TSC_IOSCR_G5_IO1* = TSC_IOSCR_G5_IO1_Msk
  TSC_IOSCR_G5_IO2_Pos* = (17)
  TSC_IOSCR_G5_IO2_Msk* = (0x00000001 shl TSC_IOSCR_G5_IO2_Pos) ## !< 0x00020000
  TSC_IOSCR_G5_IO2* = TSC_IOSCR_G5_IO2_Msk
  TSC_IOSCR_G5_IO3_Pos* = (18)
  TSC_IOSCR_G5_IO3_Msk* = (0x00000001 shl TSC_IOSCR_G5_IO3_Pos) ## !< 0x00040000
  TSC_IOSCR_G5_IO3* = TSC_IOSCR_G5_IO3_Msk
  TSC_IOSCR_G5_IO4_Pos* = (19)
  TSC_IOSCR_G5_IO4_Msk* = (0x00000001 shl TSC_IOSCR_G5_IO4_Pos) ## !< 0x00080000
  TSC_IOSCR_G5_IO4* = TSC_IOSCR_G5_IO4_Msk
  TSC_IOSCR_G6_IO1_Pos* = (20)
  TSC_IOSCR_G6_IO1_Msk* = (0x00000001 shl TSC_IOSCR_G6_IO1_Pos) ## !< 0x00100000
  TSC_IOSCR_G6_IO1* = TSC_IOSCR_G6_IO1_Msk
  TSC_IOSCR_G6_IO2_Pos* = (21)
  TSC_IOSCR_G6_IO2_Msk* = (0x00000001 shl TSC_IOSCR_G6_IO2_Pos) ## !< 0x00200000
  TSC_IOSCR_G6_IO2* = TSC_IOSCR_G6_IO2_Msk
  TSC_IOSCR_G6_IO3_Pos* = (22)
  TSC_IOSCR_G6_IO3_Msk* = (0x00000001 shl TSC_IOSCR_G6_IO3_Pos) ## !< 0x00400000
  TSC_IOSCR_G6_IO3* = TSC_IOSCR_G6_IO3_Msk
  TSC_IOSCR_G6_IO4_Pos* = (23)
  TSC_IOSCR_G6_IO4_Msk* = (0x00000001 shl TSC_IOSCR_G6_IO4_Pos) ## !< 0x00800000
  TSC_IOSCR_G6_IO4* = TSC_IOSCR_G6_IO4_Msk
  TSC_IOSCR_G7_IO1_Pos* = (24)
  TSC_IOSCR_G7_IO1_Msk* = (0x00000001 shl TSC_IOSCR_G7_IO1_Pos) ## !< 0x01000000
  TSC_IOSCR_G7_IO1* = TSC_IOSCR_G7_IO1_Msk
  TSC_IOSCR_G7_IO2_Pos* = (25)
  TSC_IOSCR_G7_IO2_Msk* = (0x00000001 shl TSC_IOSCR_G7_IO2_Pos) ## !< 0x02000000
  TSC_IOSCR_G7_IO2* = TSC_IOSCR_G7_IO2_Msk
  TSC_IOSCR_G7_IO3_Pos* = (26)
  TSC_IOSCR_G7_IO3_Msk* = (0x00000001 shl TSC_IOSCR_G7_IO3_Pos) ## !< 0x04000000
  TSC_IOSCR_G7_IO3* = TSC_IOSCR_G7_IO3_Msk
  TSC_IOSCR_G7_IO4_Pos* = (27)
  TSC_IOSCR_G7_IO4_Msk* = (0x00000001 shl TSC_IOSCR_G7_IO4_Pos) ## !< 0x08000000
  TSC_IOSCR_G7_IO4* = TSC_IOSCR_G7_IO4_Msk
  TSC_IOSCR_G8_IO1_Pos* = (28)
  TSC_IOSCR_G8_IO1_Msk* = (0x00000001 shl TSC_IOSCR_G8_IO1_Pos) ## !< 0x10000000
  TSC_IOSCR_G8_IO1* = TSC_IOSCR_G8_IO1_Msk
  TSC_IOSCR_G8_IO2_Pos* = (29)
  TSC_IOSCR_G8_IO2_Msk* = (0x00000001 shl TSC_IOSCR_G8_IO2_Pos) ## !< 0x20000000
  TSC_IOSCR_G8_IO2* = TSC_IOSCR_G8_IO2_Msk
  TSC_IOSCR_G8_IO3_Pos* = (30)
  TSC_IOSCR_G8_IO3_Msk* = (0x00000001 shl TSC_IOSCR_G8_IO3_Pos) ## !< 0x40000000
  TSC_IOSCR_G8_IO3* = TSC_IOSCR_G8_IO3_Msk
  TSC_IOSCR_G8_IO4_Pos* = (31)
  TSC_IOSCR_G8_IO4_Msk* = (0x00000001 shl TSC_IOSCR_G8_IO4_Pos) ## !< 0x80000000
  TSC_IOSCR_G8_IO4* = TSC_IOSCR_G8_IO4_Msk

## ******************  Bit definition for TSC_IOCCR register  *****************

const
  TSC_IOCCR_G1_IO1_Pos* = (0)
  TSC_IOCCR_G1_IO1_Msk* = (0x00000001 shl TSC_IOCCR_G1_IO1_Pos) ## !< 0x00000001
  TSC_IOCCR_G1_IO1* = TSC_IOCCR_G1_IO1_Msk
  TSC_IOCCR_G1_IO2_Pos* = (1)
  TSC_IOCCR_G1_IO2_Msk* = (0x00000001 shl TSC_IOCCR_G1_IO2_Pos) ## !< 0x00000002
  TSC_IOCCR_G1_IO2* = TSC_IOCCR_G1_IO2_Msk
  TSC_IOCCR_G1_IO3_Pos* = (2)
  TSC_IOCCR_G1_IO3_Msk* = (0x00000001 shl TSC_IOCCR_G1_IO3_Pos) ## !< 0x00000004
  TSC_IOCCR_G1_IO3* = TSC_IOCCR_G1_IO3_Msk
  TSC_IOCCR_G1_IO4_Pos* = (3)
  TSC_IOCCR_G1_IO4_Msk* = (0x00000001 shl TSC_IOCCR_G1_IO4_Pos) ## !< 0x00000008
  TSC_IOCCR_G1_IO4* = TSC_IOCCR_G1_IO4_Msk
  TSC_IOCCR_G2_IO1_Pos* = (4)
  TSC_IOCCR_G2_IO1_Msk* = (0x00000001 shl TSC_IOCCR_G2_IO1_Pos) ## !< 0x00000010
  TSC_IOCCR_G2_IO1* = TSC_IOCCR_G2_IO1_Msk
  TSC_IOCCR_G2_IO2_Pos* = (5)
  TSC_IOCCR_G2_IO2_Msk* = (0x00000001 shl TSC_IOCCR_G2_IO2_Pos) ## !< 0x00000020
  TSC_IOCCR_G2_IO2* = TSC_IOCCR_G2_IO2_Msk
  TSC_IOCCR_G2_IO3_Pos* = (6)
  TSC_IOCCR_G2_IO3_Msk* = (0x00000001 shl TSC_IOCCR_G2_IO3_Pos) ## !< 0x00000040
  TSC_IOCCR_G2_IO3* = TSC_IOCCR_G2_IO3_Msk
  TSC_IOCCR_G2_IO4_Pos* = (7)
  TSC_IOCCR_G2_IO4_Msk* = (0x00000001 shl TSC_IOCCR_G2_IO4_Pos) ## !< 0x00000080
  TSC_IOCCR_G2_IO4* = TSC_IOCCR_G2_IO4_Msk
  TSC_IOCCR_G3_IO1_Pos* = (8)
  TSC_IOCCR_G3_IO1_Msk* = (0x00000001 shl TSC_IOCCR_G3_IO1_Pos) ## !< 0x00000100
  TSC_IOCCR_G3_IO1* = TSC_IOCCR_G3_IO1_Msk
  TSC_IOCCR_G3_IO2_Pos* = (9)
  TSC_IOCCR_G3_IO2_Msk* = (0x00000001 shl TSC_IOCCR_G3_IO2_Pos) ## !< 0x00000200
  TSC_IOCCR_G3_IO2* = TSC_IOCCR_G3_IO2_Msk
  TSC_IOCCR_G3_IO3_Pos* = (10)
  TSC_IOCCR_G3_IO3_Msk* = (0x00000001 shl TSC_IOCCR_G3_IO3_Pos) ## !< 0x00000400
  TSC_IOCCR_G3_IO3* = TSC_IOCCR_G3_IO3_Msk
  TSC_IOCCR_G3_IO4_Pos* = (11)
  TSC_IOCCR_G3_IO4_Msk* = (0x00000001 shl TSC_IOCCR_G3_IO4_Pos) ## !< 0x00000800
  TSC_IOCCR_G3_IO4* = TSC_IOCCR_G3_IO4_Msk
  TSC_IOCCR_G4_IO1_Pos* = (12)
  TSC_IOCCR_G4_IO1_Msk* = (0x00000001 shl TSC_IOCCR_G4_IO1_Pos) ## !< 0x00001000
  TSC_IOCCR_G4_IO1* = TSC_IOCCR_G4_IO1_Msk
  TSC_IOCCR_G4_IO2_Pos* = (13)
  TSC_IOCCR_G4_IO2_Msk* = (0x00000001 shl TSC_IOCCR_G4_IO2_Pos) ## !< 0x00002000
  TSC_IOCCR_G4_IO2* = TSC_IOCCR_G4_IO2_Msk
  TSC_IOCCR_G4_IO3_Pos* = (14)
  TSC_IOCCR_G4_IO3_Msk* = (0x00000001 shl TSC_IOCCR_G4_IO3_Pos) ## !< 0x00004000
  TSC_IOCCR_G4_IO3* = TSC_IOCCR_G4_IO3_Msk
  TSC_IOCCR_G4_IO4_Pos* = (15)
  TSC_IOCCR_G4_IO4_Msk* = (0x00000001 shl TSC_IOCCR_G4_IO4_Pos) ## !< 0x00008000
  TSC_IOCCR_G4_IO4* = TSC_IOCCR_G4_IO4_Msk
  TSC_IOCCR_G5_IO1_Pos* = (16)
  TSC_IOCCR_G5_IO1_Msk* = (0x00000001 shl TSC_IOCCR_G5_IO1_Pos) ## !< 0x00010000
  TSC_IOCCR_G5_IO1* = TSC_IOCCR_G5_IO1_Msk
  TSC_IOCCR_G5_IO2_Pos* = (17)
  TSC_IOCCR_G5_IO2_Msk* = (0x00000001 shl TSC_IOCCR_G5_IO2_Pos) ## !< 0x00020000
  TSC_IOCCR_G5_IO2* = TSC_IOCCR_G5_IO2_Msk
  TSC_IOCCR_G5_IO3_Pos* = (18)
  TSC_IOCCR_G5_IO3_Msk* = (0x00000001 shl TSC_IOCCR_G5_IO3_Pos) ## !< 0x00040000
  TSC_IOCCR_G5_IO3* = TSC_IOCCR_G5_IO3_Msk
  TSC_IOCCR_G5_IO4_Pos* = (19)
  TSC_IOCCR_G5_IO4_Msk* = (0x00000001 shl TSC_IOCCR_G5_IO4_Pos) ## !< 0x00080000
  TSC_IOCCR_G5_IO4* = TSC_IOCCR_G5_IO4_Msk
  TSC_IOCCR_G6_IO1_Pos* = (20)
  TSC_IOCCR_G6_IO1_Msk* = (0x00000001 shl TSC_IOCCR_G6_IO1_Pos) ## !< 0x00100000
  TSC_IOCCR_G6_IO1* = TSC_IOCCR_G6_IO1_Msk
  TSC_IOCCR_G6_IO2_Pos* = (21)
  TSC_IOCCR_G6_IO2_Msk* = (0x00000001 shl TSC_IOCCR_G6_IO2_Pos) ## !< 0x00200000
  TSC_IOCCR_G6_IO2* = TSC_IOCCR_G6_IO2_Msk
  TSC_IOCCR_G6_IO3_Pos* = (22)
  TSC_IOCCR_G6_IO3_Msk* = (0x00000001 shl TSC_IOCCR_G6_IO3_Pos) ## !< 0x00400000
  TSC_IOCCR_G6_IO3* = TSC_IOCCR_G6_IO3_Msk
  TSC_IOCCR_G6_IO4_Pos* = (23)
  TSC_IOCCR_G6_IO4_Msk* = (0x00000001 shl TSC_IOCCR_G6_IO4_Pos) ## !< 0x00800000
  TSC_IOCCR_G6_IO4* = TSC_IOCCR_G6_IO4_Msk
  TSC_IOCCR_G7_IO1_Pos* = (24)
  TSC_IOCCR_G7_IO1_Msk* = (0x00000001 shl TSC_IOCCR_G7_IO1_Pos) ## !< 0x01000000
  TSC_IOCCR_G7_IO1* = TSC_IOCCR_G7_IO1_Msk
  TSC_IOCCR_G7_IO2_Pos* = (25)
  TSC_IOCCR_G7_IO2_Msk* = (0x00000001 shl TSC_IOCCR_G7_IO2_Pos) ## !< 0x02000000
  TSC_IOCCR_G7_IO2* = TSC_IOCCR_G7_IO2_Msk
  TSC_IOCCR_G7_IO3_Pos* = (26)
  TSC_IOCCR_G7_IO3_Msk* = (0x00000001 shl TSC_IOCCR_G7_IO3_Pos) ## !< 0x04000000
  TSC_IOCCR_G7_IO3* = TSC_IOCCR_G7_IO3_Msk
  TSC_IOCCR_G7_IO4_Pos* = (27)
  TSC_IOCCR_G7_IO4_Msk* = (0x00000001 shl TSC_IOCCR_G7_IO4_Pos) ## !< 0x08000000
  TSC_IOCCR_G7_IO4* = TSC_IOCCR_G7_IO4_Msk
  TSC_IOCCR_G8_IO1_Pos* = (28)
  TSC_IOCCR_G8_IO1_Msk* = (0x00000001 shl TSC_IOCCR_G8_IO1_Pos) ## !< 0x10000000
  TSC_IOCCR_G8_IO1* = TSC_IOCCR_G8_IO1_Msk
  TSC_IOCCR_G8_IO2_Pos* = (29)
  TSC_IOCCR_G8_IO2_Msk* = (0x00000001 shl TSC_IOCCR_G8_IO2_Pos) ## !< 0x20000000
  TSC_IOCCR_G8_IO2* = TSC_IOCCR_G8_IO2_Msk
  TSC_IOCCR_G8_IO3_Pos* = (30)
  TSC_IOCCR_G8_IO3_Msk* = (0x00000001 shl TSC_IOCCR_G8_IO3_Pos) ## !< 0x40000000
  TSC_IOCCR_G8_IO3* = TSC_IOCCR_G8_IO3_Msk
  TSC_IOCCR_G8_IO4_Pos* = (31)
  TSC_IOCCR_G8_IO4_Msk* = (0x00000001 shl TSC_IOCCR_G8_IO4_Pos) ## !< 0x80000000
  TSC_IOCCR_G8_IO4* = TSC_IOCCR_G8_IO4_Msk

## ******************  Bit definition for TSC_IOGCSR register  ****************

const
  TSC_IOGCSR_G1E_Pos* = (0)
  TSC_IOGCSR_G1E_Msk* = (0x00000001 shl TSC_IOGCSR_G1E_Pos) ## !< 0x00000001
  TSC_IOGCSR_G1E* = TSC_IOGCSR_G1E_Msk
  TSC_IOGCSR_G2E_Pos* = (1)
  TSC_IOGCSR_G2E_Msk* = (0x00000001 shl TSC_IOGCSR_G2E_Pos) ## !< 0x00000002
  TSC_IOGCSR_G2E* = TSC_IOGCSR_G2E_Msk
  TSC_IOGCSR_G3E_Pos* = (2)
  TSC_IOGCSR_G3E_Msk* = (0x00000001 shl TSC_IOGCSR_G3E_Pos) ## !< 0x00000004
  TSC_IOGCSR_G3E* = TSC_IOGCSR_G3E_Msk
  TSC_IOGCSR_G4E_Pos* = (3)
  TSC_IOGCSR_G4E_Msk* = (0x00000001 shl TSC_IOGCSR_G4E_Pos) ## !< 0x00000008
  TSC_IOGCSR_G4E* = TSC_IOGCSR_G4E_Msk
  TSC_IOGCSR_G5E_Pos* = (4)
  TSC_IOGCSR_G5E_Msk* = (0x00000001 shl TSC_IOGCSR_G5E_Pos) ## !< 0x00000010
  TSC_IOGCSR_G5E* = TSC_IOGCSR_G5E_Msk
  TSC_IOGCSR_G6E_Pos* = (5)
  TSC_IOGCSR_G6E_Msk* = (0x00000001 shl TSC_IOGCSR_G6E_Pos) ## !< 0x00000020
  TSC_IOGCSR_G6E* = TSC_IOGCSR_G6E_Msk
  TSC_IOGCSR_G7E_Pos* = (6)
  TSC_IOGCSR_G7E_Msk* = (0x00000001 shl TSC_IOGCSR_G7E_Pos) ## !< 0x00000040
  TSC_IOGCSR_G7E* = TSC_IOGCSR_G7E_Msk
  TSC_IOGCSR_G8E_Pos* = (7)
  TSC_IOGCSR_G8E_Msk* = (0x00000001 shl TSC_IOGCSR_G8E_Pos) ## !< 0x00000080
  TSC_IOGCSR_G8E* = TSC_IOGCSR_G8E_Msk
  TSC_IOGCSR_G1S_Pos* = (16)
  TSC_IOGCSR_G1S_Msk* = (0x00000001 shl TSC_IOGCSR_G1S_Pos) ## !< 0x00010000
  TSC_IOGCSR_G1S* = TSC_IOGCSR_G1S_Msk
  TSC_IOGCSR_G2S_Pos* = (17)
  TSC_IOGCSR_G2S_Msk* = (0x00000001 shl TSC_IOGCSR_G2S_Pos) ## !< 0x00020000
  TSC_IOGCSR_G2S* = TSC_IOGCSR_G2S_Msk
  TSC_IOGCSR_G3S_Pos* = (18)
  TSC_IOGCSR_G3S_Msk* = (0x00000001 shl TSC_IOGCSR_G3S_Pos) ## !< 0x00040000
  TSC_IOGCSR_G3S* = TSC_IOGCSR_G3S_Msk
  TSC_IOGCSR_G4S_Pos* = (19)
  TSC_IOGCSR_G4S_Msk* = (0x00000001 shl TSC_IOGCSR_G4S_Pos) ## !< 0x00080000
  TSC_IOGCSR_G4S* = TSC_IOGCSR_G4S_Msk
  TSC_IOGCSR_G5S_Pos* = (20)
  TSC_IOGCSR_G5S_Msk* = (0x00000001 shl TSC_IOGCSR_G5S_Pos) ## !< 0x00100000
  TSC_IOGCSR_G5S* = TSC_IOGCSR_G5S_Msk
  TSC_IOGCSR_G6S_Pos* = (21)
  TSC_IOGCSR_G6S_Msk* = (0x00000001 shl TSC_IOGCSR_G6S_Pos) ## !< 0x00200000
  TSC_IOGCSR_G6S* = TSC_IOGCSR_G6S_Msk
  TSC_IOGCSR_G7S_Pos* = (22)
  TSC_IOGCSR_G7S_Msk* = (0x00000001 shl TSC_IOGCSR_G7S_Pos) ## !< 0x00400000
  TSC_IOGCSR_G7S* = TSC_IOGCSR_G7S_Msk
  TSC_IOGCSR_G8S_Pos* = (23)
  TSC_IOGCSR_G8S_Msk* = (0x00000001 shl TSC_IOGCSR_G8S_Pos) ## !< 0x00800000
  TSC_IOGCSR_G8S* = TSC_IOGCSR_G8S_Msk

## ******************  Bit definition for TSC_IOGXCR register  ****************

const
  TSC_IOGXCR_CNT_Pos* = (0)
  TSC_IOGXCR_CNT_Msk* = (0x00003FFF shl TSC_IOGXCR_CNT_Pos) ## !< 0x00003FFF
  TSC_IOGXCR_CNT* = TSC_IOGXCR_CNT_Msk

## ****************************************************************************
##
##       Universal Synchronous Asynchronous Receiver Transmitter (USART)
##
## ****************************************************************************
## *****************  Bit definition for USART_CR1 register  ******************

const
  USART_CR1_UE_Pos* = (0)
  USART_CR1_UE_Msk* = (0x00000001 shl USART_CR1_UE_Pos) ## !< 0x00000001
  USART_CR1_UE* = USART_CR1_UE_Msk
  USART_CR1_UESM_Pos* = (1)
  USART_CR1_UESM_Msk* = (0x00000001 shl USART_CR1_UESM_Pos) ## !< 0x00000002
  USART_CR1_UESM* = USART_CR1_UESM_Msk
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
  USART_CR1_M0_Pos* = (12)
  USART_CR1_M0_Msk* = (0x00000001 shl USART_CR1_M0_Pos) ## !< 0x00001000
  USART_CR1_M0* = USART_CR1_M0_Msk
  USART_CR1_MME_Pos* = (13)
  USART_CR1_MME_Msk* = (0x00000001 shl USART_CR1_MME_Pos) ## !< 0x00002000
  USART_CR1_MME* = USART_CR1_MME_Msk
  USART_CR1_CMIE_Pos* = (14)
  USART_CR1_CMIE_Msk* = (0x00000001 shl USART_CR1_CMIE_Pos) ## !< 0x00004000
  USART_CR1_CMIE* = USART_CR1_CMIE_Msk
  USART_CR1_OVER8_Pos* = (15)
  USART_CR1_OVER8_Msk* = (0x00000001 shl USART_CR1_OVER8_Pos) ## !< 0x00008000
  USART_CR1_OVER8* = USART_CR1_OVER8_Msk
  USART_CR1_DEDT_Pos* = (16)
  USART_CR1_DEDT_Msk* = (0x0000001F shl USART_CR1_DEDT_Pos) ## !< 0x001F0000
  USART_CR1_DEDT* = USART_CR1_DEDT_Msk
  USART_CR1_DEDT_0* = (0x00000001 shl USART_CR1_DEDT_Pos) ## !< 0x00010000
  USART_CR1_DEDT_1* = (0x00000002 shl USART_CR1_DEDT_Pos) ## !< 0x00020000
  USART_CR1_DEDT_2* = (0x00000004 shl USART_CR1_DEDT_Pos) ## !< 0x00040000
  USART_CR1_DEDT_3* = (0x00000008 shl USART_CR1_DEDT_Pos) ## !< 0x00080000
  USART_CR1_DEDT_4* = (0x00000010 shl USART_CR1_DEDT_Pos) ## !< 0x00100000
  USART_CR1_DEAT_Pos* = (21)
  USART_CR1_DEAT_Msk* = (0x0000001F shl USART_CR1_DEAT_Pos) ## !< 0x03E00000
  USART_CR1_DEAT* = USART_CR1_DEAT_Msk
  USART_CR1_DEAT_0* = (0x00000001 shl USART_CR1_DEAT_Pos) ## !< 0x00200000
  USART_CR1_DEAT_1* = (0x00000002 shl USART_CR1_DEAT_Pos) ## !< 0x00400000
  USART_CR1_DEAT_2* = (0x00000004 shl USART_CR1_DEAT_Pos) ## !< 0x00800000
  USART_CR1_DEAT_3* = (0x00000008 shl USART_CR1_DEAT_Pos) ## !< 0x01000000
  USART_CR1_DEAT_4* = (0x00000010 shl USART_CR1_DEAT_Pos) ## !< 0x02000000
  USART_CR1_RTOIE_Pos* = (26)
  USART_CR1_RTOIE_Msk* = (0x00000001 shl USART_CR1_RTOIE_Pos) ## !< 0x04000000
  USART_CR1_RTOIE* = USART_CR1_RTOIE_Msk
  USART_CR1_EOBIE_Pos* = (27)
  USART_CR1_EOBIE_Msk* = (0x00000001 shl USART_CR1_EOBIE_Pos) ## !< 0x08000000
  USART_CR1_EOBIE* = USART_CR1_EOBIE_Msk

## *****************  Bit definition for USART_CR2 register  ******************

const
  USART_CR2_ADDM7_Pos* = (4)
  USART_CR2_ADDM7_Msk* = (0x00000001 shl USART_CR2_ADDM7_Pos) ## !< 0x00000010
  USART_CR2_ADDM7* = USART_CR2_ADDM7_Msk
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
  USART_CR2_SWAP_Pos* = (15)
  USART_CR2_SWAP_Msk* = (0x00000001 shl USART_CR2_SWAP_Pos) ## !< 0x00008000
  USART_CR2_SWAP* = USART_CR2_SWAP_Msk
  USART_CR2_RXINV_Pos* = (16)
  USART_CR2_RXINV_Msk* = (0x00000001 shl USART_CR2_RXINV_Pos) ## !< 0x00010000
  USART_CR2_RXINV* = USART_CR2_RXINV_Msk
  USART_CR2_TXINV_Pos* = (17)
  USART_CR2_TXINV_Msk* = (0x00000001 shl USART_CR2_TXINV_Pos) ## !< 0x00020000
  USART_CR2_TXINV* = USART_CR2_TXINV_Msk
  USART_CR2_DATAINV_Pos* = (18)
  USART_CR2_DATAINV_Msk* = (0x00000001 shl USART_CR2_DATAINV_Pos) ## !< 0x00040000
  USART_CR2_DATAINV* = USART_CR2_DATAINV_Msk
  USART_CR2_MSBFIRST_Pos* = (19)
  USART_CR2_MSBFIRST_Msk* = (0x00000001 shl USART_CR2_MSBFIRST_Pos) ## !< 0x00080000
  USART_CR2_MSBFIRST* = USART_CR2_MSBFIRST_Msk
  USART_CR2_ABREN_Pos* = (20)
  USART_CR2_ABREN_Msk* = (0x00000001 shl USART_CR2_ABREN_Pos) ## !< 0x00100000
  USART_CR2_ABREN* = USART_CR2_ABREN_Msk
  USART_CR2_ABRMODE_Pos* = (21)
  USART_CR2_ABRMODE_Msk* = (0x00000003 shl USART_CR2_ABRMODE_Pos) ## !< 0x00600000
  USART_CR2_ABRMODE* = USART_CR2_ABRMODE_Msk
  USART_CR2_ABRMODE_0* = (0x00000001 shl USART_CR2_ABRMODE_Pos) ## !< 0x00200000
  USART_CR2_ABRMODE_1* = (0x00000002 shl USART_CR2_ABRMODE_Pos) ## !< 0x00400000
  USART_CR2_RTOEN_Pos* = (23)
  USART_CR2_RTOEN_Msk* = (0x00000001 shl USART_CR2_RTOEN_Pos) ## !< 0x00800000
  USART_CR2_RTOEN* = USART_CR2_RTOEN_Msk
  USART_CR2_ADD_Pos* = (24)
  USART_CR2_ADD_Msk* = (0x000000FF shl USART_CR2_ADD_Pos) ## !< 0xFF000000
  USART_CR2_ADD* = USART_CR2_ADD_Msk

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
  USART_CR3_OVRDIS_Pos* = (12)
  USART_CR3_OVRDIS_Msk* = (0x00000001 shl USART_CR3_OVRDIS_Pos) ## !< 0x00001000
  USART_CR3_OVRDIS* = USART_CR3_OVRDIS_Msk
  USART_CR3_DDRE_Pos* = (13)
  USART_CR3_DDRE_Msk* = (0x00000001 shl USART_CR3_DDRE_Pos) ## !< 0x00002000
  USART_CR3_DDRE* = USART_CR3_DDRE_Msk
  USART_CR3_DEM_Pos* = (14)
  USART_CR3_DEM_Msk* = (0x00000001 shl USART_CR3_DEM_Pos) ## !< 0x00004000
  USART_CR3_DEM* = USART_CR3_DEM_Msk
  USART_CR3_DEP_Pos* = (15)
  USART_CR3_DEP_Msk* = (0x00000001 shl USART_CR3_DEP_Pos) ## !< 0x00008000
  USART_CR3_DEP* = USART_CR3_DEP_Msk
  USART_CR3_SCARCNT_Pos* = (17)
  USART_CR3_SCARCNT_Msk* = (0x00000007 shl USART_CR3_SCARCNT_Pos) ## !< 0x000E0000
  USART_CR3_SCARCNT* = USART_CR3_SCARCNT_Msk
  USART_CR3_SCARCNT_0* = (0x00000001 shl USART_CR3_SCARCNT_Pos) ## !< 0x00020000
  USART_CR3_SCARCNT_1* = (0x00000002 shl USART_CR3_SCARCNT_Pos) ## !< 0x00040000
  USART_CR3_SCARCNT_2* = (0x00000004 shl USART_CR3_SCARCNT_Pos) ## !< 0x00080000
  USART_CR3_WUS_Pos* = (20)
  USART_CR3_WUS_Msk* = (0x00000003 shl USART_CR3_WUS_Pos) ## !< 0x00300000
  USART_CR3_WUS* = USART_CR3_WUS_Msk
  USART_CR3_WUS_0* = (0x00000001 shl USART_CR3_WUS_Pos) ## !< 0x00100000
  USART_CR3_WUS_1* = (0x00000002 shl USART_CR3_WUS_Pos) ## !< 0x00200000
  USART_CR3_WUFIE_Pos* = (22)
  USART_CR3_WUFIE_Msk* = (0x00000001 shl USART_CR3_WUFIE_Pos) ## !< 0x00400000
  USART_CR3_WUFIE* = USART_CR3_WUFIE_Msk

## *****************  Bit definition for USART_BRR register  ******************

const
  USART_BRR_DIV_FRACTION_Pos* = (0)
  USART_BRR_DIV_FRACTION_Msk* = (0x0000000F shl USART_BRR_DIV_FRACTION_Pos) ## !< 0x0000000F
  USART_BRR_DIV_FRACTION* = USART_BRR_DIV_FRACTION_Msk
  USART_BRR_DIV_MANTISSA_Pos* = (4)
  USART_BRR_DIV_MANTISSA_Msk* = (0x00000FFF shl USART_BRR_DIV_MANTISSA_Pos) ## !< 0x0000FFF0
  USART_BRR_DIV_MANTISSA* = USART_BRR_DIV_MANTISSA_Msk

## *****************  Bit definition for USART_GTPR register  *****************

const
  USART_GTPR_PSC_Pos* = (0)
  USART_GTPR_PSC_Msk* = (0x000000FF shl USART_GTPR_PSC_Pos) ## !< 0x000000FF
  USART_GTPR_PSC* = USART_GTPR_PSC_Msk
  USART_GTPR_GT_Pos* = (8)
  USART_GTPR_GT_Msk* = (0x000000FF shl USART_GTPR_GT_Pos) ## !< 0x0000FF00
  USART_GTPR_GT* = USART_GTPR_GT_Msk

## ******************  Bit definition for USART_RTOR register  ****************

const
  USART_RTOR_RTO_Pos* = (0)
  USART_RTOR_RTO_Msk* = (0x00FFFFFF shl USART_RTOR_RTO_Pos) ## !< 0x00FFFFFF
  USART_RTOR_RTO* = USART_RTOR_RTO_Msk
  USART_RTOR_BLEN_Pos* = (24)
  USART_RTOR_BLEN_Msk* = (0x000000FF shl USART_RTOR_BLEN_Pos) ## !< 0xFF000000
  USART_RTOR_BLEN* = USART_RTOR_BLEN_Msk

## ******************  Bit definition for USART_RQR register  *****************

const
  USART_RQR_ABRRQ_Pos* = (0)
  USART_RQR_ABRRQ_Msk* = (0x00000001 shl USART_RQR_ABRRQ_Pos) ## !< 0x00000001
  USART_RQR_ABRRQ* = USART_RQR_ABRRQ_Msk
  USART_RQR_SBKRQ_Pos* = (1)
  USART_RQR_SBKRQ_Msk* = (0x00000001 shl USART_RQR_SBKRQ_Pos) ## !< 0x00000002
  USART_RQR_SBKRQ* = USART_RQR_SBKRQ_Msk
  USART_RQR_MMRQ_Pos* = (2)
  USART_RQR_MMRQ_Msk* = (0x00000001 shl USART_RQR_MMRQ_Pos) ## !< 0x00000004
  USART_RQR_MMRQ* = USART_RQR_MMRQ_Msk
  USART_RQR_RXFRQ_Pos* = (3)
  USART_RQR_RXFRQ_Msk* = (0x00000001 shl USART_RQR_RXFRQ_Pos) ## !< 0x00000008
  USART_RQR_RXFRQ* = USART_RQR_RXFRQ_Msk
  USART_RQR_TXFRQ_Pos* = (4)
  USART_RQR_TXFRQ_Msk* = (0x00000001 shl USART_RQR_TXFRQ_Pos) ## !< 0x00000010
  USART_RQR_TXFRQ* = USART_RQR_TXFRQ_Msk

## ******************  Bit definition for USART_ISR register  *****************

const
  USART_ISR_PE_Pos* = (0)
  USART_ISR_PE_Msk* = (0x00000001 shl USART_ISR_PE_Pos) ## !< 0x00000001
  USART_ISR_PE* = USART_ISR_PE_Msk
  USART_ISR_FE_Pos* = (1)
  USART_ISR_FE_Msk* = (0x00000001 shl USART_ISR_FE_Pos) ## !< 0x00000002
  USART_ISR_FE* = USART_ISR_FE_Msk
  USART_ISR_NE_Pos* = (2)
  USART_ISR_NE_Msk* = (0x00000001 shl USART_ISR_NE_Pos) ## !< 0x00000004
  USART_ISR_NE* = USART_ISR_NE_Msk
  USART_ISR_ORE_Pos* = (3)
  USART_ISR_ORE_Msk* = (0x00000001 shl USART_ISR_ORE_Pos) ## !< 0x00000008
  USART_ISR_ORE* = USART_ISR_ORE_Msk
  USART_ISR_IDLE_Pos* = (4)
  USART_ISR_IDLE_Msk* = (0x00000001 shl USART_ISR_IDLE_Pos) ## !< 0x00000010
  USART_ISR_IDLE* = USART_ISR_IDLE_Msk
  USART_ISR_RXNE_Pos* = (5)
  USART_ISR_RXNE_Msk* = (0x00000001 shl USART_ISR_RXNE_Pos) ## !< 0x00000020
  USART_ISR_RXNE* = USART_ISR_RXNE_Msk
  USART_ISR_TC_Pos* = (6)
  USART_ISR_TC_Msk* = (0x00000001 shl USART_ISR_TC_Pos) ## !< 0x00000040
  USART_ISR_TC* = USART_ISR_TC_Msk
  USART_ISR_TXE_Pos* = (7)
  USART_ISR_TXE_Msk* = (0x00000001 shl USART_ISR_TXE_Pos) ## !< 0x00000080
  USART_ISR_TXE* = USART_ISR_TXE_Msk
  USART_ISR_LBDF_Pos* = (8)
  USART_ISR_LBDF_Msk* = (0x00000001 shl USART_ISR_LBDF_Pos) ## !< 0x00000100
  USART_ISR_LBDF* = USART_ISR_LBDF_Msk
  USART_ISR_CTSIF_Pos* = (9)
  USART_ISR_CTSIF_Msk* = (0x00000001 shl USART_ISR_CTSIF_Pos) ## !< 0x00000200
  USART_ISR_CTSIF* = USART_ISR_CTSIF_Msk
  USART_ISR_CTS_Pos* = (10)
  USART_ISR_CTS_Msk* = (0x00000001 shl USART_ISR_CTS_Pos) ## !< 0x00000400
  USART_ISR_CTS* = USART_ISR_CTS_Msk
  USART_ISR_RTOF_Pos* = (11)
  USART_ISR_RTOF_Msk* = (0x00000001 shl USART_ISR_RTOF_Pos) ## !< 0x00000800
  USART_ISR_RTOF* = USART_ISR_RTOF_Msk
  USART_ISR_EOBF_Pos* = (12)
  USART_ISR_EOBF_Msk* = (0x00000001 shl USART_ISR_EOBF_Pos) ## !< 0x00001000
  USART_ISR_EOBF* = USART_ISR_EOBF_Msk
  USART_ISR_ABRE_Pos* = (14)
  USART_ISR_ABRE_Msk* = (0x00000001 shl USART_ISR_ABRE_Pos) ## !< 0x00004000
  USART_ISR_ABRE* = USART_ISR_ABRE_Msk
  USART_ISR_ABRF_Pos* = (15)
  USART_ISR_ABRF_Msk* = (0x00000001 shl USART_ISR_ABRF_Pos) ## !< 0x00008000
  USART_ISR_ABRF* = USART_ISR_ABRF_Msk
  USART_ISR_BUSY_Pos* = (16)
  USART_ISR_BUSY_Msk* = (0x00000001 shl USART_ISR_BUSY_Pos) ## !< 0x00010000
  USART_ISR_BUSY* = USART_ISR_BUSY_Msk
  USART_ISR_CMF_Pos* = (17)
  USART_ISR_CMF_Msk* = (0x00000001 shl USART_ISR_CMF_Pos) ## !< 0x00020000
  USART_ISR_CMF* = USART_ISR_CMF_Msk
  USART_ISR_SBKF_Pos* = (18)
  USART_ISR_SBKF_Msk* = (0x00000001 shl USART_ISR_SBKF_Pos) ## !< 0x00040000
  USART_ISR_SBKF* = USART_ISR_SBKF_Msk
  USART_ISR_RWU_Pos* = (19)
  USART_ISR_RWU_Msk* = (0x00000001 shl USART_ISR_RWU_Pos) ## !< 0x00080000
  USART_ISR_RWU* = USART_ISR_RWU_Msk
  USART_ISR_WUF_Pos* = (20)
  USART_ISR_WUF_Msk* = (0x00000001 shl USART_ISR_WUF_Pos) ## !< 0x00100000
  USART_ISR_WUF* = USART_ISR_WUF_Msk
  USART_ISR_TEACK_Pos* = (21)
  USART_ISR_TEACK_Msk* = (0x00000001 shl USART_ISR_TEACK_Pos) ## !< 0x00200000
  USART_ISR_TEACK* = USART_ISR_TEACK_Msk
  USART_ISR_REACK_Pos* = (22)
  USART_ISR_REACK_Msk* = (0x00000001 shl USART_ISR_REACK_Pos) ## !< 0x00400000
  USART_ISR_REACK* = USART_ISR_REACK_Msk

## ******************  Bit definition for USART_ICR register  *****************

const
  USART_ICR_PECF_Pos* = (0)
  USART_ICR_PECF_Msk* = (0x00000001 shl USART_ICR_PECF_Pos) ## !< 0x00000001
  USART_ICR_PECF* = USART_ICR_PECF_Msk
  USART_ICR_FECF_Pos* = (1)
  USART_ICR_FECF_Msk* = (0x00000001 shl USART_ICR_FECF_Pos) ## !< 0x00000002
  USART_ICR_FECF* = USART_ICR_FECF_Msk
  USART_ICR_NCF_Pos* = (2)
  USART_ICR_NCF_Msk* = (0x00000001 shl USART_ICR_NCF_Pos) ## !< 0x00000004
  USART_ICR_NCF* = USART_ICR_NCF_Msk
  USART_ICR_ORECF_Pos* = (3)
  USART_ICR_ORECF_Msk* = (0x00000001 shl USART_ICR_ORECF_Pos) ## !< 0x00000008
  USART_ICR_ORECF* = USART_ICR_ORECF_Msk
  USART_ICR_IDLECF_Pos* = (4)
  USART_ICR_IDLECF_Msk* = (0x00000001 shl USART_ICR_IDLECF_Pos) ## !< 0x00000010
  USART_ICR_IDLECF* = USART_ICR_IDLECF_Msk
  USART_ICR_TCCF_Pos* = (6)
  USART_ICR_TCCF_Msk* = (0x00000001 shl USART_ICR_TCCF_Pos) ## !< 0x00000040
  USART_ICR_TCCF* = USART_ICR_TCCF_Msk
  USART_ICR_LBDCF_Pos* = (8)
  USART_ICR_LBDCF_Msk* = (0x00000001 shl USART_ICR_LBDCF_Pos) ## !< 0x00000100
  USART_ICR_LBDCF* = USART_ICR_LBDCF_Msk
  USART_ICR_CTSCF_Pos* = (9)
  USART_ICR_CTSCF_Msk* = (0x00000001 shl USART_ICR_CTSCF_Pos) ## !< 0x00000200
  USART_ICR_CTSCF* = USART_ICR_CTSCF_Msk
  USART_ICR_RTOCF_Pos* = (11)
  USART_ICR_RTOCF_Msk* = (0x00000001 shl USART_ICR_RTOCF_Pos) ## !< 0x00000800
  USART_ICR_RTOCF* = USART_ICR_RTOCF_Msk
  USART_ICR_EOBCF_Pos* = (12)
  USART_ICR_EOBCF_Msk* = (0x00000001 shl USART_ICR_EOBCF_Pos) ## !< 0x00001000
  USART_ICR_EOBCF* = USART_ICR_EOBCF_Msk
  USART_ICR_CMCF_Pos* = (17)
  USART_ICR_CMCF_Msk* = (0x00000001 shl USART_ICR_CMCF_Pos) ## !< 0x00020000
  USART_ICR_CMCF* = USART_ICR_CMCF_Msk
  USART_ICR_WUCF_Pos* = (20)
  USART_ICR_WUCF_Msk* = (0x00000001 shl USART_ICR_WUCF_Pos) ## !< 0x00100000
  USART_ICR_WUCF* = USART_ICR_WUCF_Msk

## ******************  Bit definition for USART_RDR register  *****************

const
  USART_RDR_RDR_Pos* = (0)
  USART_RDR_RDR_Msk* = (0x000001FF shl USART_RDR_RDR_Pos) ## !< 0x000001FF
  USART_RDR_RDR* = USART_RDR_RDR_Msk

## ******************  Bit definition for USART_TDR register  *****************

const
  USART_TDR_TDR_Pos* = (0)
  USART_TDR_TDR_Msk* = (0x000001FF shl USART_TDR_TDR_Pos) ## !< 0x000001FF
  USART_TDR_TDR* = USART_TDR_TDR_Msk

## ****************************************************************************
##
##                          USB Device General registers
##
## ****************************************************************************

const
  USB_CNTR* = (USB_BASE + 0x00000040) ## !< Control register
  USB_ISTR* = (USB_BASE + 0x00000044) ## !< Interrupt status register
  USB_FNR* = (USB_BASE + 0x00000048) ## !< Frame number register
  USB_DADDR* = (USB_BASE + 0x0000004C) ## !< Device address register
  USB_BTABLE* = (USB_BASE + 0x00000050) ## !< Buffer Table address register

## ***************************  ISTR interrupt events  ************************

const
  USB_ISTR_CTR* = (cast[uint16](0x00008000'u16)) ## !< Correct TRansfer (clear-only bit)
  USB_ISTR_PMAOVR* = (cast[uint16](0x00004000'u16)) ## !< DMA OVeR/underrun (clear-only bit)
  USB_ISTR_ERR* = (cast[uint16](0x00002000'u16)) ## !< ERRor (clear-only bit)
  USB_ISTR_WKUP* = (cast[uint16](0x00001000'u16)) ## !< WaKe UP (clear-only bit)
  USB_ISTR_SUSP* = (cast[uint16](0x00000800'u16)) ## !< SUSPend (clear-only bit)
  USB_ISTR_RESET* = (cast[uint16](0x00000400'u16)) ## !< RESET (clear-only bit)
  USB_ISTR_SOF* = (cast[uint16](0x00000200'u16)) ## !< Start Of Frame (clear-only bit)
  USB_ISTR_ESOF* = (cast[uint16](0x00000100'u16)) ## !< Expected Start Of Frame (clear-only bit)
  USB_ISTR_DIR* = (cast[uint16](0x00000010'u16)) ## !< DIRection of transaction (read-only bit)
  USB_ISTR_EP_ID* = (cast[uint16](0x0000000F'u16)) ## !< EndPoint IDentifier (read-only bit)

##  Legacy defines

const
  USB_ISTR_PMAOVRM* = USB_ISTR_PMAOVR
  USB_CLR_CTR* = (not USB_ISTR_CTR) ## !< clear Correct TRansfer bit
  USB_CLR_PMAOVR* = (not USB_ISTR_PMAOVR) ## !< clear DMA OVeR/underrun bit
  USB_CLR_ERR* = (not USB_ISTR_ERR) ## !< clear ERRor bit
  USB_CLR_WKUP* = (not USB_ISTR_WKUP) ## !< clear WaKe UP bit
  USB_CLR_SUSP* = (not USB_ISTR_SUSP) ## !< clear SUSPend bit
  USB_CLR_RESET* = (not USB_ISTR_RESET) ## !< clear RESET bit
  USB_CLR_SOF* = (not USB_ISTR_SOF) ## !< clear Start Of Frame bit
  USB_CLR_ESOF* = (not USB_ISTR_ESOF) ## !< clear Expected Start Of Frame bit

##  Legacy defines

const
  USB_CLR_PMAOVRM* = USB_CLR_PMAOVR

## ************************  CNTR control register bits definitions  **********

const
  USB_CNTR_CTRM* =   (cast[uint16](0x00008000'u16)) ## !< Correct TRansfer Mask
  USB_CNTR_PMAOVR* = (cast[uint16](0x00004000'u16)) ## !< DMA OVeR/underrun Mask
  USB_CNTR_ERRM* =   (cast[uint16](0x00002000'u16)) ## !< ERRor Mask
  USB_CNTR_WKUPM* =  (cast[uint16](0x00001000'u16)) ## !< WaKe UP Mask
  USB_CNTR_SUSPM* =  (cast[uint16](0x00000800'u16)) ## !< SUSPend Mask
  USB_CNTR_RESETM* = (cast[uint16](0x00000400'u16)) ## !< RESET Mask
  USB_CNTR_SOFM* =   (cast[uint16](0x00000200'u16)) ## !< Start Of Frame Mask
  USB_CNTR_ESOFM* =  (cast[uint16](0x00000100'u16)) ## !< Expected Start Of Frame Mask
  USB_CNTR_RESUME* = (cast[uint16](0x00000010'u16)) ## !< RESUME request
  USB_CNTR_FSUSP* =  (cast[uint16](0x00000008'u16)) ## !< Force SUSPend
  USB_CNTR_LPMODE* = (cast[uint16](0x00000004'u16)) ## !< Low-power MODE
  USB_CNTR_PDWN* =   (cast[uint16](0x00000002'u16)) ## !< Power DoWN
  USB_CNTR_FRES* =   (cast[uint16](0x00000001'u16)) ## !< Force USB RESet

##  Legacy defines

const
  USB_CNTR_PMAOVRM* = USB_CNTR_PMAOVR
  USB_CNTR_LP_MODEx* = USB_CNTR_LPMODE

## *******************  FNR Frame Number Register bit definitions   ***********

const
  USB_FNR_RXDP* = (cast[uint16](0x00008000'u16)) ## !< status of D+ data line
  USB_FNR_RXDM* = (cast[uint16](0x00004000'u16)) ## !< status of D- data line
  USB_FNR_LCK* =  (cast[uint16](0x00002000'u16)) ## !< LoCKed
  USB_FNR_LSOF* = (cast[uint16](0x00001800'u16)) ## !< Lost SOF
  USB_FNR_FN* =   (cast[uint16](0x000007FF'u16)) ## !< Frame Number

## *******************  DADDR Device ADDRess bit definitions    ***************

const
  USB_DADDR_EF* =  (cast[uint8](0x00000080'u8)) ## !< USB device address Enable Function
  USB_DADDR_ADD* = (cast[uint8](0x0000007F'u8)) ## !< USB device address

## *****************************  Endpoint register    ************************

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
  USB_EP_CTR_RX* =    (cast[uint16](0x00008000'u16)) ## !<  EndPoint Correct TRansfer RX
  USB_EP_DTOG_RX* =   (cast[uint16](0x00004000'u16)) ## !<  EndPoint Data TOGGLE RX
  USB_EPRX_STAT* =    (cast[uint16](0x00003000'u16)) ## !<  EndPoint RX STATus bit field
  USB_EP_SETUP* =     (cast[uint16](0x00000800'u16)) ## !<  EndPoint SETUP
  USB_EP_T_FIELD* =   (cast[uint16](0x00000600'u16)) ## !<  EndPoint TYPE
  USB_EP_KIND* =      (cast[uint16](0x00000100'u16)) ## !<  EndPoint KIND
  USB_EP_CTR_TX* =    (cast[uint16](0x00000080'u16)) ## !<  EndPoint Correct TRansfer TX
  USB_EP_DTOG_TX* =   (cast[uint16](0x00000040'u16)) ## !<  EndPoint Data TOGGLE TX
  USB_EPTX_STAT* =    (cast[uint16](0x00000030'u16)) ## !<  EndPoint TX STATus bit field
  USB_EPADDR_FIELD* = (cast[uint16](0x0000000F'u16)) ## !<  EndPoint ADDRess FIELD

##  EndPoint REGister MASK (no toggle fields)

const
  USB_EPREG_MASK* = (USB_EP_CTR_RX or USB_EP_SETUP or USB_EP_T_FIELD or USB_EP_KIND or
      USB_EP_CTR_TX or USB_EPADDR_FIELD)

## !< EP_TYPE[1:0] EndPoint TYPE

const
  USB_EP_TYPE_MASK* =   (cast[uint16](0x00000600'u16)) ## !< EndPoint TYPE Mask
  USB_EP_BULK* =        (cast[uint16](0x00000000'u16)) ## !< EndPoint BULK
  USB_EP_CONTROL* =     (cast[uint16](0x00000200'u16)) ## !< EndPoint CONTROL
  USB_EP_ISOCHRONOUS* = (cast[uint16](0x00000400'u16)) ## !< EndPoint ISOCHRONOUS
  USB_EP_INTERRUPT* =   (cast[uint16](0x00000600'u16)) ## !< EndPoint INTERRUPT
  USB_EP_T_MASK* = ((uint16)(not USB_EP_T_FIELD) and USB_EPREG_MASK)
  USB_EPKIND_MASK* = ((uint16)(not USB_EP_KIND) and USB_EPREG_MASK) ## !< EP_KIND EndPoint KIND

## !< STAT_TX[1:0] STATus for TX transfer

const
  USB_EP_TX_DIS* =   (cast[uint16](0x00000000'u16)) ## !< EndPoint TX DISabled
  USB_EP_TX_STALL* = (cast[uint16](0x00000010'u16)) ## !< EndPoint TX STALLed
  USB_EP_TX_NAK* =   (cast[uint16](0x00000020'u16)) ## !< EndPoint TX NAKed
  USB_EP_TX_VALID* = (cast[uint16](0x00000030'u16)) ## !< EndPoint TX VALID
  USB_EPTX_DTOG1* =  (cast[uint16](0x00000010'u16)) ## !< EndPoint TX Data TOGgle bit1
  USB_EPTX_DTOG2* =  (cast[uint16](0x00000020'u16)) ## !< EndPoint TX Data TOGgle bit2
  USB_EPTX_DTOGMASK* = (USB_EPTX_STAT or USB_EPREG_MASK)

## !< STAT_RX[1:0] STATus for RX transfer

const
  USB_EP_RX_DIS* =   (cast[uint16](0x00000000'u16)) ## !< EndPoint RX DISabled
  USB_EP_RX_STALL* = (cast[uint16](0x00001000'u16)) ## !< EndPoint RX STALLed
  USB_EP_RX_NAK* =   (cast[uint16](0x00002000'u16)) ## !< EndPoint RX NAKed
  USB_EP_RX_VALID* = (cast[uint16](0x00003000'u16)) ## !< EndPoint RX VALID
  USB_EPRX_DTOG1* =  (cast[uint16](0x00001000'u16)) ## !< EndPoint RX Data TOGgle bit1
  USB_EPRX_DTOG2* =  (cast[uint16](0x00002000'u16)) ## !< EndPoint RX Data TOGgle bit1
  USB_EPRX_DTOGMASK* = (USB_EPRX_STAT or USB_EPREG_MASK)

## ****************************************************************************
##
##                             Window WATCHDOG
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

## *
##  @}
##
## *
##  @}
##
## * @addtogroup Exported_macros
##  @{
##
## ***************************** ADC Instances ********************************

template IS_ADC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == ADC1) or ((INSTANCE) == ADC2) or ((INSTANCE) == ADC3) or
      ((INSTANCE) == ADC4))

template IS_ADC_MULTIMODE_MASTER_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == ADC1) or ((INSTANCE) == ADC3))

template IS_ADC_COMMON_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == ADC12_COMMON) or ((INSTANCE) == ADC34_COMMON))

## ***************************** CAN Instances ********************************

template IS_CAN_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == CAN)

## ***************************** COMP Instances *******************************

template IS_COMP_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == COMP1) or ((INSTANCE) == COMP2) or ((INSTANCE) == COMP3) or
      ((INSTANCE) == COMP4) or ((INSTANCE) == COMP5) or ((INSTANCE) == COMP6) or
      ((INSTANCE) == COMP7))

template IS_COMP_COMMON_INSTANCE*(COMMON_INSTANCE: untyped): untyped =
  (((COMMON_INSTANCE) == COMP12_COMMON) or ((COMMON_INSTANCE) == COMP34_COMMON) or
      ((COMMON_INSTANCE) == COMP56_COMMON))

## ******************* COMP Instances with switch on DAC1 Channel1 output *****

template IS_COMP_DAC1SWITCH_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == COMP1)

## ******************* COMP Instances with window mode capability *************

template IS_COMP_WINDOWMODE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == COMP2) or ((INSTANCE) == COMP4) or ((INSTANCE) == COMP6))

## ***************************** CRC Instances ********************************

template IS_CRC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == CRC)

## ***************************** DAC Instances ********************************

template IS_DAC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == DAC1)

template IS_DAC_CHANNEL_INSTANCE*(INSTANCE, CHANNEL: untyped): untyped =
  ((((INSTANCE) == DAC1) and
      (((CHANNEL) == DAC_CHANNEL_1) or ((CHANNEL) == DAC_CHANNEL_2))))

## ***************************** DMA Instances ********************************

template IS_DMA_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == DMA1_Channel1) or ((INSTANCE) == DMA1_Channel2) or
      ((INSTANCE) == DMA1_Channel3) or ((INSTANCE) == DMA1_Channel4) or
      ((INSTANCE) == DMA1_Channel5) or ((INSTANCE) == DMA1_Channel6) or
      ((INSTANCE) == DMA1_Channel7) or ((INSTANCE) == DMA2_Channel1) or
      ((INSTANCE) == DMA2_Channel2) or ((INSTANCE) == DMA2_Channel3) or
      ((INSTANCE) == DMA2_Channel4) or ((INSTANCE) == DMA2_Channel5))

## ***************************** GPIO Instances *******************************

template IS_GPIO_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == GPIOA) or ((INSTANCE) == GPIOB) or ((INSTANCE) == GPIOC) or
      ((INSTANCE) == GPIOD) or ((INSTANCE) == GPIOE) or ((INSTANCE) == GPIOF))

template IS_GPIO_AF_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == GPIOA) or ((INSTANCE) == GPIOB) or ((INSTANCE) == GPIOC) or
      ((INSTANCE) == GPIOD) or ((INSTANCE) == GPIOE) or ((INSTANCE) == GPIOF))

template IS_GPIO_LOCK_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == GPIOA) or ((INSTANCE) == GPIOB) or ((INSTANCE) == GPIOD))

## ***************************** I2C Instances ********************************

template IS_I2C_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == I2C1) or ((INSTANCE) == I2C2))

## ***************** I2C Instances : wakeup capability from stop modes ********

template IS_I2C_WAKEUP_FROMSTOP_INSTANCE*(INSTANCE: untyped): untyped =
  IS_I2C_ALL_INSTANCE(INSTANCE)

## ***************************** I2S Instances ********************************

template IS_I2S_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == SPI2) or ((INSTANCE) == SPI3))

template IS_I2S_EXT_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == I2S2ext) or ((INSTANCE) == I2S3ext))

## ***************************** OPAMP Instances ******************************

template IS_OPAMP_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == OPAMP1) or ((INSTANCE) == OPAMP2) or ((INSTANCE) == OPAMP3) or
      ((INSTANCE) == OPAMP4))

## ***************************** IWDG Instances *******************************

template IS_IWDG_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == IWDG)

## ***************************** RTC Instances ********************************

template IS_RTC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == RTC)

## ***************************** SMBUS Instances ******************************

template IS_SMBUS_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == I2C1) or ((INSTANCE) == I2C2))

## ***************************** SPI Instances ********************************

template IS_SPI_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == SPI1) or ((INSTANCE) == SPI2) or ((INSTANCE) == SPI3))

## ****************** TIM Instances : All supported instances *****************

template IS_TIM_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM6) or ((INSTANCE) == TIM7) or
      ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15) or ((INSTANCE) == TIM16) or
      ((INSTANCE) == TIM17))

## ****************** TIM Instances : at least 1 capture/compare channel ******

template IS_TIM_CC1_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15) or
      ((INSTANCE) == TIM16) or ((INSTANCE) == TIM17))

## ***************** TIM Instances : at least 2 capture/compare channels ******

template IS_TIM_CC2_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15))

## ***************** TIM Instances : at least 3 capture/compare channels ******

template IS_TIM_CC3_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : at least 4 capture/compare channels ******

template IS_TIM_CC4_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : at least 5 capture/compare channels ******

template IS_TIM_CC5_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : at least 6 capture/compare channels ******

template IS_TIM_CC6_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8))

## ************************* TIM Instances : Advanced-control timers **********
## ***************** TIM Instances : Advanced timer instances ******************

template IS_TIM_ADVANCED_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : supporting clock selection ***************

template IS_TIM_CLOCK_SELECT_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15))

## ***************** TIM Instances : supporting external clock mode 1 for ETRF input

template IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : supporting external clock mode 2 *********

template IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : supporting external clock mode 1 for TIX inputs

template IS_TIM_CLOCKSOURCE_TIX_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15))

## ***************** TIM Instances : supporting internal trigger inputs(ITRX) ******

template IS_TIM_CLOCKSOURCE_ITRX_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15))

## ***************** TIM Instances : supporting OCxREF clear ******************

template IS_TIM_OCXREF_CLEAR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : supporting encoder interface *************

template IS_TIM_ENCODER_INTERFACE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : supporting Hall interface ****************

template IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8))

## *************** TIM Instances : external trigger input available ***********

template IS_TIM_ETR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : supporting input XOR function ************

template IS_TIM_XOR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15))

## ***************** TIM Instances : supporting master mode *******************

template IS_TIM_MASTER_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM6) or ((INSTANCE) == TIM7) or
      ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15))

## ***************** TIM Instances : supporting slave mode ********************

template IS_TIM_SLAVE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15))

## ***************** TIM Instances : supporting synchronization ***************

template IS_TIM_SYNCHRO_INSTANCE*(INSTANCE: untyped): untyped =
  IS_TIM_MASTER_INSTANCE(INSTANCE)

## ***************** TIM Instances : supporting 32 bits counter ***************

template IS_TIM_32B_COUNTER_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == TIM2)

## ***************** TIM Instances : supporting DMA burst *********************

template IS_TIM_DMABURST_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15) or
      ((INSTANCE) == TIM16) or ((INSTANCE) == TIM17))

## ***************** TIM Instances : supporting the break function ************

template IS_TIM_BREAK_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15) or
      ((INSTANCE) == TIM16) or ((INSTANCE) == TIM17))

## ***************** TIM Instances : supporting input/output channel(s) *******

template IS_TIM_CCX_INSTANCE*(INSTANCE, CHANNEL: untyped): untyped =
  ((((INSTANCE) == TIM1) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3) or ((CHANNEL) == TIM_CHANNEL_4) or
      ((CHANNEL) == TIM_CHANNEL_5) or ((CHANNEL) == TIM_CHANNEL_6))) or
      (((INSTANCE) == TIM2) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3) or ((CHANNEL) == TIM_CHANNEL_4))) or
      (((INSTANCE) == TIM3) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3) or ((CHANNEL) == TIM_CHANNEL_4))) or
      (((INSTANCE) == TIM4) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3) or ((CHANNEL) == TIM_CHANNEL_4))) or
      (((INSTANCE) == TIM8) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3) or ((CHANNEL) == TIM_CHANNEL_4) or
      ((CHANNEL) == TIM_CHANNEL_5) or ((CHANNEL) == TIM_CHANNEL_6))) or
      (((INSTANCE) == TIM15) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2))) or
      (((INSTANCE) == TIM16) and (((CHANNEL) == TIM_CHANNEL_1))) or
      (((INSTANCE) == TIM17) and (((CHANNEL) == TIM_CHANNEL_1))))

## ***************** TIM Instances : supporting complementary output(s) *******

template IS_TIM_CCXN_INSTANCE*(INSTANCE, CHANNEL: untyped): untyped =
  ((((INSTANCE) == TIM1) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3))) or
      (((INSTANCE) == TIM8) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3))) or
      (((INSTANCE) == TIM15) and ((CHANNEL) == TIM_CHANNEL_1)) or
      (((INSTANCE) == TIM16) and ((CHANNEL) == TIM_CHANNEL_1)) or
      (((INSTANCE) == TIM17) and ((CHANNEL) == TIM_CHANNEL_1)))

## ***************** TIM Instances : supporting counting mode selection *******

template IS_TIM_COUNTER_MODE_SELECT_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : supporting repetition counter ************

template IS_TIM_REPETITION_COUNTER_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15) or
      ((INSTANCE) == TIM16) or ((INSTANCE) == TIM17))

## ***************** TIM Instances : supporting clock division ****************

template IS_TIM_CLOCK_DIVISION_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15) or
      ((INSTANCE) == TIM16) or ((INSTANCE) == TIM17))

## ***************** TIM Instances : supporting 2 break inputs ****************

template IS_TIM_BKIN2_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : supporting ADC triggering through TRGO2 **

template IS_TIM_TRGO2_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : supporting DMA generation on Update events

template IS_TIM_DMA_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM6) or ((INSTANCE) == TIM7) or
      ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15) or ((INSTANCE) == TIM16) or
      ((INSTANCE) == TIM17))

## ***************** TIM Instances : supporting DMA generation on Capture/Compare events

template IS_TIM_DMA_CC_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15) or
      ((INSTANCE) == TIM16) or ((INSTANCE) == TIM17))

## ***************** TIM Instances : supporting commutation event generation **

template IS_TIM_COMMUTATION_EVENT_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM15) or
      ((INSTANCE) == TIM16) or ((INSTANCE) == TIM17))

## ***************** TIM Instances : supporting remapping capability **********

template IS_TIM_REMAP_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM16))

## ***************** TIM Instances : supporting combined 3-phase PWM mode *****

template IS_TIM_COMBINED3PHASEPWM_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8))

## ***************************** TSC Instances ********************************

template IS_TSC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == TSC)

## ******************* USART Instances : Synchronous mode *********************

template IS_USART_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3))

## ***************** USART Instances : Auto Baud Rate detection ***************

template IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE*(INSTANCE: untyped): untyped =
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

## ******************* UART Instances : Wake-up from Stop mode *********************

template IS_UART_WAKEUP_FROMSTOP_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3) or
      ((INSTANCE) == UART4) or ((INSTANCE) == UART5))

## ***************** UART Instances : Hardware Flow control *******************

template IS_UART_HWFLOW_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3))

## ***************** UART Instances : Auto Baud Rate detection ****************

template IS_UART_AUTOBAUDRATE_DETECTION_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3))

## ***************** UART Instances : Driver Enable ***************************

template IS_UART_DRIVER_ENABLE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3))

## ******************** UART Instances : Smard card mode **********************

template IS_SMARTCARD_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3))

## ********************** UART Instances : IRDA mode **************************

template IS_IRDA_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3) or
      ((INSTANCE) == UART4) or ((INSTANCE) == UART5))

## ******************* UART Instances : Support of continuous communication using DMA ***

template IS_UART_DMA_INSTANCE*(INSTANCE: untyped): untyped =
  (1)

## ***************************** USB Instances ********************************

template IS_USB_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == USB)

## ***************************** WWDG Instances *******************************

template IS_WWDG_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == WWDG)

## *
##  @}
##
## ****************************************************************************
##   For a painless codes migration between the STM32F3xx device product
##   lines, the aliases defined below are put in place to overcome the
##   differences in the interrupt handlers and IRQn definitions.
##   No need to update developed interrupt code when moving across
##   product lines within the same STM32F3 Family
## ****************************************************************************
##  Aliases for __IRQn

const
  ADC1_IRQn* = ADC1_2_IRQn
  SDADC1_IRQn* = ADC4_IRQn
  COMP_IRQn* = COMP1_2_3_IRQn
  COMP2_IRQn* = COMP1_2_3_IRQn
  COMP1_2_IRQn* = COMP1_2_3_IRQn
  COMP4_6_IRQn* = COMP4_5_6_IRQn
  TIM15_IRQn* = TIM1_BRK_TIM15_IRQn
  TIM18_DAC2_IRQn* = TIM1_CC_IRQn
  TIM17_IRQn* = TIM1_TRG_COM_TIM17_IRQn
  TIM16_IRQn* = TIM1_UP_TIM16_IRQn
  TIM6_DAC1_IRQn* = TIM6_DAC_IRQn
  TIM7_DAC2_IRQn* = TIM7_IRQn
  TIM12_IRQn* = TIM8_BRK_IRQn
  TIM14_IRQn* = TIM8_TRG_COM_IRQn
  TIM13_IRQn* = TIM8_UP_IRQn
  CEC_IRQn* = USBWakeUp_IRQn
#USBWakeUp_IRQn* = USBWakeUp_RMP_IRQn
  CAN_TX_IRQn* = USB_HP_CAN_TX_IRQn
  CAN_RX0_IRQn* = USB_LP_CAN_RX0_IRQn

##  Aliases for __IRQHandler
when false:
    const
      ADC1_IRQHandler* = ADC1_2_IRQHandler
      SDADC1_IRQHandler* = ADC4_IRQHandler
      COMP_IRQHandler* = COMP1_2_3_IRQHandler
      COMP2_IRQHandler* = COMP1_2_3_IRQHandler
      COMP1_2_IRQHandler* = COMP1_2_3_IRQHandler
      COMP4_6_IRQHandler* = COMP4_5_6_IRQHandler
      TIM15_IRQHandler* = TIM1_BRK_TIM15_IRQHandler
      TIM18_DAC2_IRQHandler* = TIM1_CC_IRQHandler
      TIM17_IRQHandler* = TIM1_TRG_COM_TIM17_IRQHandler
      TIM16_IRQHandler* = TIM1_UP_TIM16_IRQHandler
      TIM6_DAC1_IRQHandler* = TIM6_DAC_IRQHandler
      TIM7_DAC2_IRQHandler* = TIM7_IRQHandler
      TIM12_IRQHandler* = TIM8_BRK_IRQHandler
      TIM14_IRQHandler* = TIM8_TRG_COM_IRQHandler
      TIM13_IRQHandler* = TIM8_UP_IRQHandler
      CEC_IRQHandler* = USBWakeUp_IRQHandler
      USBWakeUp_IRQHandler* = USBWakeUp_RMP_IRQHandler
      CAN_TX_IRQHandler* = USB_HP_CAN_TX_IRQHandler
      CAN_RX0_IRQHandler* = USB_LP_CAN_RX0_IRQHandler

## *
##  @}
##
## *
##  @}
##
## *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE***
include "reg_utils"
include "core_cm4"
