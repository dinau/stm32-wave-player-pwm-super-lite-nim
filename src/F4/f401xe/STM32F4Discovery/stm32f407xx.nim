## *
## *****************************************************************************
##  @file    stm32f407xx.h
##  @author  MCD Application Team
##  @brief   CMSIS STM32F407xx Device Peripheral Access Layer Header File.
##
##           This file contains:
##            - Data structures and the address mapping for all peripherals
##            - peripherals registers declarations and bits definition
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
## * @addtogroup CMSIS_Device
##  @{
##
## * @addtogroup stm32f407xx
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
##  @brief STM32F4XX Interrupt Number Definition, according to the selected device
##         in @ref Library_configuration_section
##

type                          ## *****  Cortex-M4 Processor Exceptions Numbers ***************************************************************
  IRQn_Type* = enum
    NonMaskableInt_IRQn = -14,  ## !< 2 Non Maskable Interrupt
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
    TAMP_STAMP_IRQn = 2,        ## !< Tamper and TimeStamp interrupts through the EXTI line
    RTC_WKUP_IRQn = 3,          ## !< RTC Wakeup interrupt through the EXTI line
    FLASH_IRQn = 4,             ## !< FLASH global Interrupt
    RCC_IRQn = 5,               ## !< RCC global Interrupt
    EXTI0_IRQn = 6,             ## !< EXTI Line0 Interrupt
    EXTI1_IRQn = 7,             ## !< EXTI Line1 Interrupt
    EXTI2_IRQn = 8,             ## !< EXTI Line2 Interrupt
    EXTI3_IRQn = 9,             ## !< EXTI Line3 Interrupt
    EXTI4_IRQn = 10,            ## !< EXTI Line4 Interrupt
    DMA1_Stream0_IRQn = 11,     ## !< DMA1 Stream 0 global Interrupt
    DMA1_Stream1_IRQn = 12,     ## !< DMA1 Stream 1 global Interrupt
    DMA1_Stream2_IRQn = 13,     ## !< DMA1 Stream 2 global Interrupt
    DMA1_Stream3_IRQn = 14,     ## !< DMA1 Stream 3 global Interrupt
    DMA1_Stream4_IRQn = 15,     ## !< DMA1 Stream 4 global Interrupt
    DMA1_Stream5_IRQn = 16,     ## !< DMA1 Stream 5 global Interrupt
    DMA1_Stream6_IRQn = 17,     ## !< DMA1 Stream 6 global Interrupt
    ADC_IRQn = 18,              ## !< ADC1, ADC2 and ADC3 global Interrupts
    CAN1_TX_IRQn = 19,          ## !< CAN1 TX Interrupt
    CAN1_RX0_IRQn = 20,         ## !< CAN1 RX0 Interrupt
    CAN1_RX1_IRQn = 21,         ## !< CAN1 RX1 Interrupt
    CAN1_SCE_IRQn = 22,         ## !< CAN1 SCE Interrupt
    EXTI9_5_IRQn = 23,          ## !< External Line[9:5] Interrupts
    TIM1_BRK_TIM9_IRQn = 24,    ## !< TIM1 Break interrupt and TIM9 global interrupt
    TIM1_UP_TIM10_IRQn = 25,    ## !< TIM1 Update Interrupt and TIM10 global interrupt
    TIM1_TRG_COM_TIM11_IRQn = 26, ## !< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
    TIM1_CC_IRQn = 27,          ## !< TIM1 Capture Compare Interrupt
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
    RTC_Alarm_IRQn = 41,        ## !< RTC Alarm (A and B) through EXTI Line Interrupt
    OTG_FS_WKUP_IRQn = 42,      ## !< USB OTG FS Wakeup through EXTI line interrupt
    TIM8_BRK_TIM12_IRQn = 43,   ## !< TIM8 Break Interrupt and TIM12 global interrupt
    TIM8_UP_TIM13_IRQn = 44,    ## !< TIM8 Update Interrupt and TIM13 global interrupt
    TIM8_TRG_COM_TIM14_IRQn = 45, ## !< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt
    TIM8_CC_IRQn = 46,          ## !< TIM8 Capture Compare global interrupt
    DMA1_Stream7_IRQn = 47,     ## !< DMA1 Stream7 Interrupt
    FSMC_IRQn = 48,             ## !< FSMC global Interrupt
    SDIO_IRQn = 49,             ## !< SDIO global Interrupt
    TIM5_IRQn = 50,             ## !< TIM5 global Interrupt
    SPI3_IRQn = 51,             ## !< SPI3 global Interrupt
    UART4_IRQn = 52,            ## !< UART4 global Interrupt
    UART5_IRQn = 53,            ## !< UART5 global Interrupt
    TIM6_DAC_IRQn = 54,         ## !< TIM6 global and DAC1&2 underrun error  interrupts
    TIM7_IRQn = 55,             ## !< TIM7 global interrupt
    DMA2_Stream0_IRQn = 56,     ## !< DMA2 Stream 0 global Interrupt
    DMA2_Stream1_IRQn = 57,     ## !< DMA2 Stream 1 global Interrupt
    DMA2_Stream2_IRQn = 58,     ## !< DMA2 Stream 2 global Interrupt
    DMA2_Stream3_IRQn = 59,     ## !< DMA2 Stream 3 global Interrupt
    DMA2_Stream4_IRQn = 60,     ## !< DMA2 Stream 4 global Interrupt
    ETH_IRQn = 61,              ## !< Ethernet global Interrupt
    ETH_WKUP_IRQn = 62,         ## !< Ethernet Wakeup through EXTI line Interrupt
    CAN2_TX_IRQn = 63,          ## !< CAN2 TX Interrupt
    CAN2_RX0_IRQn = 64,         ## !< CAN2 RX0 Interrupt
    CAN2_RX1_IRQn = 65,         ## !< CAN2 RX1 Interrupt
    CAN2_SCE_IRQn = 66,         ## !< CAN2 SCE Interrupt
    OTG_FS_IRQn = 67,           ## !< USB OTG FS global Interrupt
    DMA2_Stream5_IRQn = 68,     ## !< DMA2 Stream 5 global interrupt
    DMA2_Stream6_IRQn = 69,     ## !< DMA2 Stream 6 global interrupt
    DMA2_Stream7_IRQn = 70,     ## !< DMA2 Stream 7 global interrupt
    USART6_IRQn = 71,           ## !< USART6 global interrupt
    I2C3_EV_IRQn = 72,          ## !< I2C3 event interrupt
    I2C3_ER_IRQn = 73,          ## !< I2C3 error interrupt
    OTG_HS_EP1_OUT_IRQn = 74,   ## !< USB OTG HS End Point 1 Out global interrupt
    OTG_HS_EP1_IN_IRQn = 75,    ## !< USB OTG HS End Point 1 In global interrupt
    OTG_HS_WKUP_IRQn = 76,      ## !< USB OTG HS Wakeup through EXTI interrupt
    OTG_HS_IRQn = 77,           ## !< USB OTG HS global interrupt
    DCMI_IRQn = 78,             ## !< DCMI global interrupt
    RNG_IRQn = 80,              ## !< RNG global Interrupt
    FPU_IRQn = 81


##  Legacy define

const
  HASH_RNG_IRQn* = RNG_IRQn

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
    JOFR1*: uint32             ## !< ADC injected channel data offset register 1, Address offset: 0x14
    JOFR2*: uint32             ## !< ADC injected channel data offset register 2, Address offset: 0x18
    JOFR3*: uint32             ## !< ADC injected channel data offset register 3, Address offset: 0x1C
    JOFR4*: uint32             ## !< ADC injected channel data offset register 4, Address offset: 0x20
    HTR*: uint32               ## !< ADC watchdog higher threshold register,      Address offset: 0x24
    LTR*: uint32               ## !< ADC watchdog lower threshold register,       Address offset: 0x28
    SQR1*: uint32              ## !< ADC regular sequence register 1,             Address offset: 0x2C
    SQR2*: uint32              ## !< ADC regular sequence register 2,             Address offset: 0x30
    SQR3*: uint32              ## !< ADC regular sequence register 3,             Address offset: 0x34
    JSQR*: uint32              ## !< ADC injected sequence register,              Address offset: 0x38
    JDR1*: uint32              ## !< ADC injected data register 1,                Address offset: 0x3C
    JDR2*: uint32              ## !< ADC injected data register 2,                Address offset: 0x40
    JDR3*: uint32              ## !< ADC injected data register 3,                Address offset: 0x44
    JDR4*: uint32              ## !< ADC injected data register 4,                Address offset: 0x48
    DR*: uint32                ## !< ADC regular data register,                   Address offset: 0x4C

  ADC_Common_TypeDef* {.bycopy.} = object
    CSR*: uint32               ## !< ADC Common status register,                  Address offset: ADC1 base address + 0x300
    CCR*: uint32               ## !< ADC common control register,                 Address offset: ADC1 base address + 0x304
    CDR*: uint32               ## !< ADC common regular data register for dual
               ##                              AND triple modes,                            Address offset: ADC1 base address + 0x308


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
##  @brief CRC calculation unit
##

type
  CRC_TypeDef* {.bycopy.} = object
    DR*: uint32                ## !< CRC Data register,             Address offset: 0x00
    IDR*: uint8                ## !< CRC Independent data register, Address offset: 0x04
    RESERVED0*: uint8          ## !< Reserved, 0x05
    RESERVED1*: uint16         ## !< Reserved, 0x06
    CR*: uint32                ## !< CRC Control register,          Address offset: 0x08


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
##  @brief DCMI
##

type
  DCMI_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< DCMI control register 1,                       Address offset: 0x00
    SR*: uint32                ## !< DCMI status register,                          Address offset: 0x04
    RISR*: uint32              ## !< DCMI raw interrupt status register,            Address offset: 0x08
    IER*: uint32               ## !< DCMI interrupt enable register,                Address offset: 0x0C
    MISR*: uint32              ## !< DCMI masked interrupt status register,         Address offset: 0x10
    ICR*: uint32               ## !< DCMI interrupt clear register,                 Address offset: 0x14
    ESCR*: uint32              ## !< DCMI embedded synchronization code register,   Address offset: 0x18
    ESUR*: uint32              ## !< DCMI embedded synchronization unmask register, Address offset: 0x1C
    CWSTRTR*: uint32           ## !< DCMI crop window start,                        Address offset: 0x20
    CWSIZER*: uint32           ## !< DCMI crop window size,                         Address offset: 0x24
    DR*: uint32                ## !< DCMI data register,                            Address offset: 0x28


## *
##  @brief DMA Controller
##

type
  DMA_Stream_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< DMA stream x configuration register
    NDTR*: uint32              ## !< DMA stream x number of data register
    PAR*: uint32               ## !< DMA stream x peripheral address register
    M0AR*: uint32              ## !< DMA stream x memory 0 address register
    M1AR*: uint32              ## !< DMA stream x memory 1 address register
    FCR*: uint32               ## !< DMA stream x FIFO control register

  DMA_TypeDef* {.bycopy.} = object
    LISR*: uint32              ## !< DMA low interrupt status register,      Address offset: 0x00
    HISR*: uint32              ## !< DMA high interrupt status register,     Address offset: 0x04
    LIFCR*: uint32             ## !< DMA low interrupt flag clear register,  Address offset: 0x08
    HIFCR*: uint32             ## !< DMA high interrupt flag clear register, Address offset: 0x0C


## *
##  @brief Ethernet MAC
##

type
  ETH_TypeDef* {.bycopy.} = object
    MACCR*: uint32
    MACFFR*: uint32
    MACHTHR*: uint32
    MACHTLR*: uint32
    MACMIIAR*: uint32
    MACMIIDR*: uint32
    MACFCR*: uint32
    MACVLANTR*: uint32         ##     8
    RESERVED0*: array[2, uint32]
    MACRWUFFR*: uint32         ##    11
    MACPMTCSR*: uint32
    RESERVED1*: uint32
    MACDBGR*: uint32
    MACSR*: uint32             ##    15
    MACIMR*: uint32
    MACA0HR*: uint32
    MACA0LR*: uint32
    MACA1HR*: uint32
    MACA1LR*: uint32
    MACA2HR*: uint32
    MACA2LR*: uint32
    MACA3HR*: uint32
    MACA3LR*: uint32           ##    24
    RESERVED2*: array[40, uint32]
    MMCCR*: uint32             ##    65
    MMCRIR*: uint32
    MMCTIR*: uint32
    MMCRIMR*: uint32
    MMCTIMR*: uint32           ##    69
    RESERVED3*: array[14, uint32]
    MMCTGFSCCR*: uint32        ##    84
    MMCTGFMSCCR*: uint32
    RESERVED4*: array[5, uint32]
    MMCTGFCR*: uint32
    RESERVED5*: array[10, uint32]
    MMCRFCECR*: uint32
    MMCRFAECR*: uint32
    RESERVED6*: array[10, uint32]
    MMCRGUFCR*: uint32
    RESERVED7*: array[334, uint32]
    PTPTSCR*: uint32
    PTPSSIR*: uint32
    PTPTSHR*: uint32
    PTPTSLR*: uint32
    PTPTSHUR*: uint32
    PTPTSLUR*: uint32
    PTPTSAR*: uint32
    PTPTTHR*: uint32
    PTPTTLR*: uint32
    RESERVED8*: uint32
    PTPTSSR*: uint32
    RESERVED9*: array[565, uint32]
    DMABMR*: uint32
    DMATPDR*: uint32
    DMARPDR*: uint32
    DMARDLAR*: uint32
    DMATDLAR*: uint32
    DMASR*: uint32
    DMAOMR*: uint32
    DMAIER*: uint32
    DMAMFBOCR*: uint32
    DMARSWTR*: uint32
    RESERVED10*: array[8, uint32]
    DMACHTDR*: uint32
    DMACHRDR*: uint32
    DMACHTBAR*: uint32
    DMACHRBAR*: uint32


## *
##  @brief External Interrupt/Event Controller
##

type
  EXTI_TypeDef* {.bycopy.} = object
    IMR*: uint32               ## !< EXTI Interrupt mask register,            Address offset: 0x00
    EMR*: uint32               ## !< EXTI Event mask register,                Address offset: 0x04
    RTSR*: uint32              ## !< EXTI Rising trigger selection register,  Address offset: 0x08
    FTSR*: uint32              ## !< EXTI Falling trigger selection register, Address offset: 0x0C
    SWIER*: uint32             ## !< EXTI Software interrupt event register,  Address offset: 0x10
    PR*: uint32                ## !< EXTI Pending register,                   Address offset: 0x14


## *
##  @brief FLASH Registers
##

type
  FLASH_TypeDef* {.bycopy.} = object
    ACR*: uint32               ## !< FLASH access control register,   Address offset: 0x00
    KEYR*: uint32              ## !< FLASH key register,              Address offset: 0x04
    OPTKEYR*: uint32           ## !< FLASH option key register,       Address offset: 0x08
    SR*: uint32                ## !< FLASH status register,           Address offset: 0x0C
    CR*: uint32                ## !< FLASH control register,          Address offset: 0x10
    OPTCR*: uint32             ## !< FLASH option control register ,  Address offset: 0x14
    OPTCR1*: uint32            ## !< FLASH option control register 1, Address offset: 0x18


## *
##  @brief Flexible Static Memory Controller
##

type
  FSMC_Bank1_TypeDef* {.bycopy.} = object
    BTCR*: array[8, uint32]     ## !< NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C


## *
##  @brief Flexible Static Memory Controller Bank1E
##

type
  FSMC_Bank1E_TypeDef* {.bycopy.} = object
    BWTR*: array[7, uint32]     ## !< NOR/PSRAM write timing registers, Address offset: 0x104-0x11C


## *
##  @brief Flexible Static Memory Controller Bank2
##

type
  FSMC_Bank2_3_TypeDef* {.bycopy.} = object
    PCR2*: uint32              ## !< NAND Flash control register 2,                       Address offset: 0x60
    SR2*: uint32               ## !< NAND Flash FIFO status and interrupt register 2,     Address offset: 0x64
    PMEM2*: uint32             ## !< NAND Flash Common memory space timing register 2,    Address offset: 0x68
    PATT2*: uint32             ## !< NAND Flash Attribute memory space timing register 2, Address offset: 0x6C
    RESERVED0*: uint32         ## !< Reserved, 0x70
    ECCR2*: uint32             ## !< NAND Flash ECC result registers 2,                   Address offset: 0x74
    RESERVED1*: uint32         ## !< Reserved, 0x78
    RESERVED2*: uint32         ## !< Reserved, 0x7C
    PCR3*: uint32              ## !< NAND Flash control register 3,                       Address offset: 0x80
    SR3*: uint32               ## !< NAND Flash FIFO status and interrupt register 3,     Address offset: 0x84
    PMEM3*: uint32             ## !< NAND Flash Common memory space timing register 3,    Address offset: 0x88
    PATT3*: uint32             ## !< NAND Flash Attribute memory space timing register 3, Address offset: 0x8C
    RESERVED3*: uint32         ## !< Reserved, 0x90
    ECCR3*: uint32             ## !< NAND Flash ECC result registers 3,                   Address offset: 0x94


## *
##  @brief Flexible Static Memory Controller Bank4
##

type
  FSMC_Bank4_TypeDef* {.bycopy.} = object
    PCR4*: uint32              ## !< PC Card  control register 4,                       Address offset: 0xA0
    SR4*: uint32               ## !< PC Card  FIFO status and interrupt register 4,     Address offset: 0xA4
    PMEM4*: uint32             ## !< PC Card  Common memory space timing register 4,    Address offset: 0xA8
    PATT4*: uint32             ## !< PC Card  Attribute memory space timing register 4, Address offset: 0xAC
    PIO4*: uint32              ## !< PC Card  I/O space timing register 4,              Address offset: 0xB0


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
    BSRR*: uint32              ## !< GPIO port bit set/reset register,      Address offset: 0x18
    LCKR*: uint32              ## !< GPIO port configuration lock register, Address offset: 0x1C
    AFR*: array[2, uint32]      ## !< GPIO alternate function registers,     Address offset: 0x20-0x24


## *
##  @brief System configuration controller
##

type
  SYSCFG_TypeDef* {.bycopy.} = object
    MEMRMP*: uint32            ## !< SYSCFG memory remap register,                      Address offset: 0x00
    PMC*: uint32               ## !< SYSCFG peripheral mode configuration register,     Address offset: 0x04
    EXTICR*: array[4, uint32]   ## !< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14
    RESERVED*: array[2, uint32] ## !< Reserved, 0x18-0x1C
    CMPCR*: uint32             ## !< SYSCFG Compensation cell control register,         Address offset: 0x20


## *
##  @brief Inter-integrated Circuit Interface
##

type
  I2C_TypeDef* {.bycopy.} = object
    CR1*: uint32               ## !< I2C Control register 1,     Address offset: 0x00
    CR2*: uint32               ## !< I2C Control register 2,     Address offset: 0x04
    OAR1*: uint32              ## !< I2C Own address register 1, Address offset: 0x08
    OAR2*: uint32              ## !< I2C Own address register 2, Address offset: 0x0C
    DR*: uint32                ## !< I2C Data register,          Address offset: 0x10
    SR1*: uint32               ## !< I2C Status register 1,      Address offset: 0x14
    SR2*: uint32               ## !< I2C Status register 2,      Address offset: 0x18
    CCR*: uint32               ## !< I2C Clock control register, Address offset: 0x1C
    TRISE*: uint32             ## !< I2C TRISE register,         Address offset: 0x20


## *
##  @brief Independent WATCHDOG
##

type
  IWDG_TypeDef* {.bycopy.} = object
    KR*: uint32                ## !< IWDG Key register,       Address offset: 0x00
    PR*: uint32                ## !< IWDG Prescaler register, Address offset: 0x04
    RLR*: uint32               ## !< IWDG Reload register,    Address offset: 0x08
    SR*: uint32                ## !< IWDG Status register,    Address offset: 0x0C


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
    PLLCFGR*: uint32           ## !< RCC PLL configuration register,                              Address offset: 0x04
    CFGR*: uint32              ## !< RCC clock configuration register,                            Address offset: 0x08
    CIR*: uint32               ## !< RCC clock interrupt register,                                Address offset: 0x0C
    AHB1RSTR*: uint32          ## !< RCC AHB1 peripheral reset register,                          Address offset: 0x10
    AHB2RSTR*: uint32          ## !< RCC AHB2 peripheral reset register,                          Address offset: 0x14
    AHB3RSTR*: uint32          ## !< RCC AHB3 peripheral reset register,                          Address offset: 0x18
    RESERVED0*: uint32         ## !< Reserved, 0x1C
    APB1RSTR*: uint32          ## !< RCC APB1 peripheral reset register,                          Address offset: 0x20
    APB2RSTR*: uint32          ## !< RCC APB2 peripheral reset register,                          Address offset: 0x24
    RESERVED1*: array[2, uint32] ## !< Reserved, 0x28-0x2C
    AHB1ENR*: uint32           ## !< RCC AHB1 peripheral clock register,                          Address offset: 0x30
    AHB2ENR*: uint32           ## !< RCC AHB2 peripheral clock register,                          Address offset: 0x34
    AHB3ENR*: uint32           ## !< RCC AHB3 peripheral clock register,                          Address offset: 0x38
    RESERVED2*: uint32         ## !< Reserved, 0x3C
    APB1ENR*: uint32           ## !< RCC APB1 peripheral clock enable register,                   Address offset: 0x40
    APB2ENR*: uint32           ## !< RCC APB2 peripheral clock enable register,                   Address offset: 0x44
    RESERVED3*: array[2, uint32] ## !< Reserved, 0x48-0x4C
    AHB1LPENR*: uint32         ## !< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50
    AHB2LPENR*: uint32         ## !< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54
    AHB3LPENR*: uint32         ## !< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58
    RESERVED4*: uint32         ## !< Reserved, 0x5C
    APB1LPENR*: uint32         ## !< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60
    APB2LPENR*: uint32         ## !< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64
    RESERVED5*: array[2, uint32] ## !< Reserved, 0x68-0x6C
    BDCR*: uint32              ## !< RCC Backup domain control register,                          Address offset: 0x70
    CSR*: uint32               ## !< RCC clock control & status register,                         Address offset: 0x74
    RESERVED6*: array[2, uint32] ## !< Reserved, 0x78-0x7C
    SSCGR*: uint32             ## !< RCC spread spectrum clock generation register,               Address offset: 0x80
    PLLI2SCFGR*: uint32        ## !< RCC PLLI2S configuration register,                           Address offset: 0x84


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
    CALIBR*: uint32            ## !< RTC calibration register,                                 Address offset: 0x18
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
    BKP0R*: uint32             ## !< RTC backup register 1,                                    Address offset: 0x50
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
    BKP16R*: uint32            ## !< RTC backup register 16,                                   Address offset: 0x90
    BKP17R*: uint32            ## !< RTC backup register 17,                                   Address offset: 0x94
    BKP18R*: uint32            ## !< RTC backup register 18,                                   Address offset: 0x98
    BKP19R*: uint32            ## !< RTC backup register 19,                                   Address offset: 0x9C


## *
##  @brief SD host Interface
##

type
  SDIO_TypeDef* {.bycopy.} = object
    POWER*: uint32             ## !< SDIO power control register,    Address offset: 0x00
    CLKCR*: uint32             ## !< SDI clock control register,     Address offset: 0x04
    ARG*: uint32               ## !< SDIO argument register,         Address offset: 0x08
    CMD*: uint32               ## !< SDIO command register,          Address offset: 0x0C
    RESPCMD*: uint32           ## !< SDIO command response register, Address offset: 0x10
    RESP1*: uint32             ## !< SDIO response 1 register,       Address offset: 0x14
    RESP2*: uint32             ## !< SDIO response 2 register,       Address offset: 0x18
    RESP3*: uint32             ## !< SDIO response 3 register,       Address offset: 0x1C
    RESP4*: uint32             ## !< SDIO response 4 register,       Address offset: 0x20
    DTIMER*: uint32            ## !< SDIO data timer register,       Address offset: 0x24
    DLEN*: uint32              ## !< SDIO data length register,      Address offset: 0x28
    DCTRL*: uint32             ## !< SDIO data control register,     Address offset: 0x2C
    DCOUNT*: uint32            ## !< SDIO data counter register,     Address offset: 0x30
    STA*: uint32               ## !< SDIO status register,           Address offset: 0x34
    ICR*: uint32               ## !< SDIO interrupt clear register,  Address offset: 0x38
    MASK*: uint32              ## !< SDIO mask register,             Address offset: 0x3C
    RESERVED0*: array[2, uint32] ## !< Reserved, 0x40-0x44
    FIFOCNT*: uint32           ## !< SDIO FIFO counter register,     Address offset: 0x48
    RESERVED1*: array[13, uint32] ## !< Reserved, 0x4C-0x7C
    FIFO*: uint32              ## !< SDIO data FIFO register,        Address offset: 0x80


## *
##  @brief Serial Peripheral Interface
##

type
  SPI_TypeDef* {.bycopy.} = object
    CR1*: uint32               ## !< SPI control register 1 (not used in I2S mode),      Address offset: 0x00
    CR2*: uint32               ## !< SPI control register 2,                             Address offset: 0x04
    SR*: uint32                ## !< SPI status register,                                Address offset: 0x08
    DR*: uint32                ## !< SPI data register,                                  Address offset: 0x0C
    CRCPR*: uint32             ## !< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10
    RXCRCR*: uint32            ## !< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14
    TXCRCR*: uint32            ## !< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18
    I2SCFGR*: uint32           ## !< SPI_I2S configuration register,                     Address offset: 0x1C
    I2SPR*: uint32             ## !< SPI_I2S prescaler register,                         Address offset: 0x20


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
##  @brief Window WATCHDOG
##

type
  WWDG_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< WWDG Control register,       Address offset: 0x00
    CFR*: uint32               ## !< WWDG Configuration register, Address offset: 0x04
    SR*: uint32                ## !< WWDG Status register,        Address offset: 0x08


## *
##  @brief RNG
##

type
  RNG_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< RNG control register, Address offset: 0x00
    SR*: uint32                ## !< RNG status register,  Address offset: 0x04
    DR*: uint32                ## !< RNG data register,    Address offset: 0x08


## *
##  @brief USB_OTG_Core_Registers
##

type
  USB_OTG_GlobalTypeDef* {.bycopy.} = object
    GOTGCTL*: uint32           ## !< USB_OTG Control and Status Register          000h
    GOTGINT*: uint32           ## !< USB_OTG Interrupt Register                   004h
    GAHBCFG*: uint32           ## !< Core AHB Configuration Register              008h
    GUSBCFG*: uint32           ## !< Core USB Configuration Register              00Ch
    GRSTCTL*: uint32           ## !< Core Reset Register                          010h
    GINTSTS*: uint32           ## !< Core Interrupt Register                      014h
    GINTMSK*: uint32           ## !< Core Interrupt Mask Register                 018h
    GRXSTSR*: uint32           ## !< Receive Sts Q Read Register                  01Ch
    GRXSTSP*: uint32           ## !< Receive Sts Q Read & POP Register            020h
    GRXFSIZ*: uint32           ## !< Receive FIFO Size Register                   024h
    DIEPTXF0_HNPTXFSIZ*: uint32 ## !< EP0 / Non Periodic Tx FIFO Size Register     028h
    HNPTXSTS*: uint32          ## !< Non Periodic Tx FIFO/Queue Sts reg           02Ch
    Reserved30*: array[2, uint32] ## !< Reserved                                     030h
    GCCFG*: uint32             ## !< General Purpose IO Register                  038h
    CID*: uint32               ## !< User ID Register                             03Ch
    Reserved40*: array[48, uint32] ## !< Reserved                                0x40-0xFF
    HPTXFSIZ*: uint32          ## !< Host Periodic Tx FIFO Size Reg               100h
    DIEPTXF*: array[0x0000000F, uint32] ## !< dev Periodic Transmit FIFO


## *
##  @brief USB_OTG_device_Registers
##

type
  USB_OTG_DeviceTypeDef* {.bycopy.} = object
    DCFG*: uint32              ## !< dev Configuration Register   800h
    DCTL*: uint32              ## !< dev Control Register         804h
    DSTS*: uint32              ## !< dev Status Register (RO)     808h
    Reserved0C*: uint32        ## !< Reserved                     80Ch
    DIEPMSK*: uint32           ## !< dev IN Endpoint Mask         810h
    DOEPMSK*: uint32           ## !< dev OUT Endpoint Mask        814h
    DAINT*: uint32             ## !< dev All Endpoints Itr Reg    818h
    DAINTMSK*: uint32          ## !< dev All Endpoints Itr Mask   81Ch
    Reserved20*: uint32        ## !< Reserved                     820h
    Reserved9*: uint32         ## !< Reserved                     824h
    DVBUSDIS*: uint32          ## !< dev VBUS discharge Register  828h
    DVBUSPULSE*: uint32        ## !< dev VBUS Pulse Register      82Ch
    DTHRCTL*: uint32           ## !< dev threshold                830h
    DIEPEMPMSK*: uint32        ## !< dev empty msk                834h
    DEACHINT*: uint32          ## !< dedicated EP interrupt       838h
    DEACHMSK*: uint32          ## !< dedicated EP msk             83Ch
    Reserved40*: uint32        ## !< dedicated EP mask            840h
    DINEP1MSK*: uint32         ## !< dedicated EP mask            844h
    Reserved44*: array[15, uint32] ## !< Reserved                 844-87Ch
    DOUTEP1MSK*: uint32        ## !< dedicated EP msk             884h


## *
##  @brief USB_OTG_IN_Endpoint-Specific_Register
##

type
  USB_OTG_INEndpointTypeDef* {.bycopy.} = object
    DIEPCTL*: uint32           ## !< dev IN Endpoint Control Reg    900h + (ep_num * 20h) + 00h
    Reserved04*: uint32        ## !< Reserved                       900h + (ep_num * 20h) + 04h
    DIEPINT*: uint32           ## !< dev IN Endpoint Itr Reg        900h + (ep_num * 20h) + 08h
    Reserved0C*: uint32        ## !< Reserved                       900h + (ep_num * 20h) + 0Ch
    DIEPTSIZ*: uint32          ## !< IN Endpoint Txfer Size         900h + (ep_num * 20h) + 10h
    DIEPDMA*: uint32           ## !< IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h
    DTXFSTS*: uint32           ## !< IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h
    Reserved18*: uint32        ## !< Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch


## *
##  @brief USB_OTG_OUT_Endpoint-Specific_Registers
##

type
  USB_OTG_OUTEndpointTypeDef* {.bycopy.} = object
    DOEPCTL*: uint32           ## !< dev OUT Endpoint Control Reg           B00h + (ep_num * 20h) + 00h
    Reserved04*: uint32        ## !< Reserved                               B00h + (ep_num * 20h) + 04h
    DOEPINT*: uint32           ## !< dev OUT Endpoint Itr Reg               B00h + (ep_num * 20h) + 08h
    Reserved0C*: uint32        ## !< Reserved                               B00h + (ep_num * 20h) + 0Ch
    DOEPTSIZ*: uint32          ## !< dev OUT Endpoint Txfer Size            B00h + (ep_num * 20h) + 10h
    DOEPDMA*: uint32           ## !< dev OUT Endpoint DMA Address           B00h + (ep_num * 20h) + 14h
    Reserved18*: array[2, uint32] ## !< Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch


## *
##  @brief USB_OTG_Host_Mode_Register_Structures
##

type
  USB_OTG_HostTypeDef* {.bycopy.} = object
    HCFG*: uint32              ## !< Host Configuration Register          400h
    HFIR*: uint32              ## !< Host Frame Interval Register         404h
    HFNUM*: uint32             ## !< Host Frame Nbr/Frame Remaining       408h
    Reserved40C*: uint32       ## !< Reserved                             40Ch
    HPTXSTS*: uint32           ## !< Host Periodic Tx FIFO/ Queue Status  410h
    HAINT*: uint32             ## !< Host All Channels Interrupt Register 414h
    HAINTMSK*: uint32          ## !< Host All Channels Interrupt Mask     418h


## *
##  @brief USB_OTG_Host_Channel_Specific_Registers
##

type
  USB_OTG_HostChannelTypeDef* {.bycopy.} = object
    HCCHAR*: uint32            ## !< Host Channel Characteristics Register    500h
    HCSPLT*: uint32            ## !< Host Channel Split Control Register      504h
    HCINT*: uint32             ## !< Host Channel Interrupt Register          508h
    HCINTMSK*: uint32          ## !< Host Channel Interrupt Mask Register     50Ch
    HCTSIZ*: uint32            ## !< Host Channel Transfer Size Register      510h
    HCDMA*: uint32             ## !< Host Channel DMA Address Register        514h
    Reserved*: array[2, uint32] ## !< Reserved


## *
##  @}
##
## * @addtogroup Peripheral_memory_map
##  @{
##

const
  FLASH_BASE* = 0x08000000
  CCMDATARAM_BASE* = 0x10000000
  SRAM1_BASE* = 0x20000000
  SRAM2_BASE* = 0x2001C000
  PERIPH_BASE* = 0x40000000
  BKPSRAM_BASE* = 0x40024000
  FSMC_R_BASE* = 0xA0000000
  SRAM1_BB_BASE* = 0x22000000
  SRAM2_BB_BASE* = 0x22380000
  PERIPH_BB_BASE* = 0x42000000
  BKPSRAM_BB_BASE* = 0x42480000
  FLASH_END* = 0x080FFFFF
  FLASH_OTP_BASE* = 0x1FFF7800
  FLASH_OTP_END* = 0x1FFF7A0F
  CCMDATARAM_END* = 0x1000FFFF

##  Legacy defines

const
  SRAM_BASE* = SRAM1_BASE
  SRAM_BB_BASE* = SRAM1_BB_BASE

## !< Peripheral memory map

const
  APB1PERIPH_BASE* = PERIPH_BASE
  APB2PERIPH_BASE* = (PERIPH_BASE + 0x00010000)
  AHB1PERIPH_BASE* = (PERIPH_BASE + 0x00020000)
  AHB2PERIPH_BASE* = (PERIPH_BASE + 0x10000000)

## !< APB1 peripherals

const
  TIM2_BASE* = (APB1PERIPH_BASE + 0x00000000)
  TIM3_BASE* = (APB1PERIPH_BASE + 0x00000400)
  TIM4_BASE* = (APB1PERIPH_BASE + 0x00000800)
  TIM5_BASE* = (APB1PERIPH_BASE + 0x00000C00)
  TIM6_BASE* = (APB1PERIPH_BASE + 0x00001000)
  TIM7_BASE* = (APB1PERIPH_BASE + 0x00001400)
  TIM12_BASE* = (APB1PERIPH_BASE + 0x00001800)
  TIM13_BASE* = (APB1PERIPH_BASE + 0x00001C00)
  TIM14_BASE* = (APB1PERIPH_BASE + 0x00002000)
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
  I2C3_BASE* = (APB1PERIPH_BASE + 0x00005C00)
  CAN1_BASE* = (APB1PERIPH_BASE + 0x00006400)
  CAN2_BASE* = (APB1PERIPH_BASE + 0x00006800)
  PWR_BASE* = (APB1PERIPH_BASE + 0x00007000)
  DAC_BASE* = (APB1PERIPH_BASE + 0x00007400)

## !< APB2 peripherals

const
  TIM1_BASE* = (APB2PERIPH_BASE + 0x00000000)
  TIM8_BASE* = (APB2PERIPH_BASE + 0x00000400)
  USART1_BASE* = (APB2PERIPH_BASE + 0x00001000)
  USART6_BASE* = (APB2PERIPH_BASE + 0x00001400)
  ADC1_BASE* = (APB2PERIPH_BASE + 0x00002000)
  ADC2_BASE* = (APB2PERIPH_BASE + 0x00002100)
  ADC3_BASE* = (APB2PERIPH_BASE + 0x00002200)
  ADC123_COMMON_BASE* = (APB2PERIPH_BASE + 0x00002300)

##  Legacy define

const
  ADC_BASE* = ADC123_COMMON_BASE
  SDIO_BASE* = (APB2PERIPH_BASE + 0x00002C00)
  SPI1_BASE* = (APB2PERIPH_BASE + 0x00003000)
  SYSCFG_BASE* = (APB2PERIPH_BASE + 0x00003800)
  EXTI_BASE* = (APB2PERIPH_BASE + 0x00003C00)
  TIM9_BASE* = (APB2PERIPH_BASE + 0x00004000)
  TIM10_BASE* = (APB2PERIPH_BASE + 0x00004400)
  TIM11_BASE* = (APB2PERIPH_BASE + 0x00004800)

## !< AHB1 peripherals

const
  GPIOA_BASE* = (AHB1PERIPH_BASE + 0x00000000)
  GPIOB_BASE* = (AHB1PERIPH_BASE + 0x00000400)
  GPIOC_BASE* = (AHB1PERIPH_BASE + 0x00000800)
  GPIOD_BASE* = (AHB1PERIPH_BASE + 0x00000C00)
  GPIOE_BASE* = (AHB1PERIPH_BASE + 0x00001000)
  GPIOF_BASE* = (AHB1PERIPH_BASE + 0x00001400)
  GPIOG_BASE* = (AHB1PERIPH_BASE + 0x00001800)
  GPIOH_BASE* = (AHB1PERIPH_BASE + 0x00001C00)
  GPIOI_BASE* = (AHB1PERIPH_BASE + 0x00002000)
  CRC_BASE* = (AHB1PERIPH_BASE + 0x00003000)
  RCC_BASE* = (AHB1PERIPH_BASE + 0x00003800)
  FLASH_R_BASE* = (AHB1PERIPH_BASE + 0x00003C00)
  DMA1_BASE* = (AHB1PERIPH_BASE + 0x00006000)
  DMA1_Stream0_BASE* = (DMA1_BASE + 0x00000010)
  DMA1_Stream1_BASE* = (DMA1_BASE + 0x00000028)
  DMA1_Stream2_BASE* = (DMA1_BASE + 0x00000040)
  DMA1_Stream3_BASE* = (DMA1_BASE + 0x00000058)
  DMA1_Stream4_BASE* = (DMA1_BASE + 0x00000070)
  DMA1_Stream5_BASE* = (DMA1_BASE + 0x00000088)
  DMA1_Stream6_BASE* = (DMA1_BASE + 0x000000A0)
  DMA1_Stream7_BASE* = (DMA1_BASE + 0x000000B8)
  DMA2_BASE* = (AHB1PERIPH_BASE + 0x00006400)
  DMA2_Stream0_BASE* = (DMA2_BASE + 0x00000010)
  DMA2_Stream1_BASE* = (DMA2_BASE + 0x00000028)
  DMA2_Stream2_BASE* = (DMA2_BASE + 0x00000040)
  DMA2_Stream3_BASE* = (DMA2_BASE + 0x00000058)
  DMA2_Stream4_BASE* = (DMA2_BASE + 0x00000070)
  DMA2_Stream5_BASE* = (DMA2_BASE + 0x00000088)
  DMA2_Stream6_BASE* = (DMA2_BASE + 0x000000A0)
  DMA2_Stream7_BASE* = (DMA2_BASE + 0x000000B8)
  ETH_BASE* = (AHB1PERIPH_BASE + 0x00008000)
  ETH_MAC_BASE* = (ETH_BASE)
  ETH_MMC_BASE* = (ETH_BASE + 0x00000100)
  ETH_PTP_BASE* = (ETH_BASE + 0x00000700)
  ETH_DMA_BASE* = (ETH_BASE + 0x00001000)

## !< AHB2 peripherals

const
  DCMI_BASE* = (AHB2PERIPH_BASE + 0x00050000)
  RNG_BASE* = (AHB2PERIPH_BASE + 0x00060800)

## !< FSMC Bankx registers base address

const
  FSMC_Bank1_R_BASE* = (FSMC_R_BASE + 0x00000000)
  FSMC_Bank1E_R_BASE* = (FSMC_R_BASE + 0x00000104)
  FSMC_Bank2_3_R_BASE* = (FSMC_R_BASE + 0x00000060)
  FSMC_Bank4_R_BASE* = (FSMC_R_BASE + 0x000000A0)

## !< Debug MCU registers base address

const
  DBGMCU_BASE* = 0xE0042000

## !< USB registers base address

const
  USB_OTG_HS_PERIPH_BASE* = 0x40040000
  USB_OTG_FS_PERIPH_BASE* = 0x50000000
  USB_OTG_GLOBAL_BASE* = 0x00000000
  USB_OTG_DEVICE_BASE* = 0x00000800
  USB_OTG_IN_ENDPOINT_BASE* = 0x00000900
  USB_OTG_OUT_ENDPOINT_BASE* = 0x00000B00
  USB_OTG_EP_REG_SIZE* = 0x00000020
  USB_OTG_HOST_BASE* = 0x00000400
  USB_OTG_HOST_PORT_BASE* = 0x00000440
  USB_OTG_HOST_CHANNEL_BASE* = 0x00000500
  USB_OTG_HOST_CHANNEL_SIZE* = 0x00000020
  USB_OTG_PCGCCTL_BASE* = 0x00000E00
  USB_OTG_FIFO_BASE* = 0x00001000
  USB_OTG_FIFO_SIZE* = 0x00001000
  UID_BASE* = 0x1FFF7A10
  FLASHSIZE_BASE* = 0x1FFF7A22
  PACKAGE_BASE* = 0x1FFF7BF0

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
  TIM12* = (cast[ptr TIM_TypeDef](TIM12_BASE))
  TIM13* = (cast[ptr TIM_TypeDef](TIM13_BASE))
  TIM14* = (cast[ptr TIM_TypeDef](TIM14_BASE))
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
  I2C3* = (cast[ptr I2C_TypeDef](I2C3_BASE))
  CAN1* = (cast[ptr CAN_TypeDef](CAN1_BASE))
  CAN2* = (cast[ptr CAN_TypeDef](CAN2_BASE))
  PWR* = (cast[ptr PWR_TypeDef](PWR_BASE))
  DAC1* = (cast[ptr DAC_TypeDef](DAC_BASE))
  DAC* = (cast[ptr DAC_TypeDef](DAC_BASE)) ##  Kept for legacy purpose
  TIM1* = (cast[ptr TIM_TypeDef](TIM1_BASE))
  TIM8* = (cast[ptr TIM_TypeDef](TIM8_BASE))
  USART1* = (cast[ptr USART_TypeDef](USART1_BASE))
  USART6* = (cast[ptr USART_TypeDef](USART6_BASE))
  ADC1* = (cast[ptr ADC_TypeDef](ADC1_BASE))
  ADC2* = (cast[ptr ADC_TypeDef](ADC2_BASE))
  ADC3* = (cast[ptr ADC_TypeDef](ADC3_BASE))
  ADC123_COMMON* = (cast[ptr ADC_Common_TypeDef](ADC123_COMMON_BASE))

##  Legacy define

const
  ADC* = ADC123_COMMON
  SDIO* = (cast[ptr SDIO_TypeDef](SDIO_BASE))
  SPI1* = (cast[ptr SPI_TypeDef](SPI1_BASE))
  SYSCFG* = (cast[ptr SYSCFG_TypeDef](SYSCFG_BASE))
  EXTI* = (cast[ptr EXTI_TypeDef](EXTI_BASE))
  TIM9* = (cast[ptr TIM_TypeDef](TIM9_BASE))
  TIM10* = (cast[ptr TIM_TypeDef](TIM10_BASE))
  TIM11* = (cast[ptr TIM_TypeDef](TIM11_BASE))
  GPIOA* = (cast[ptr GPIO_TypeDef](GPIOA_BASE))
  GPIOB* = (cast[ptr GPIO_TypeDef](GPIOB_BASE))
  GPIOC* = (cast[ptr GPIO_TypeDef](GPIOC_BASE))
  GPIOD* = (cast[ptr GPIO_TypeDef](GPIOD_BASE))
  GPIOE* = (cast[ptr GPIO_TypeDef](GPIOE_BASE))
  GPIOF* = (cast[ptr GPIO_TypeDef](GPIOF_BASE))
  GPIOG* = (cast[ptr GPIO_TypeDef](GPIOG_BASE))
  GPIOH* = (cast[ptr GPIO_TypeDef](GPIOH_BASE))
  GPIOI* = (cast[ptr GPIO_TypeDef](GPIOI_BASE))
  CRC* = (cast[ptr CRC_TypeDef](CRC_BASE))
  RCC* = (cast[ptr RCC_TypeDef](RCC_BASE))
  FLASH* = (cast[ptr FLASH_TypeDef](FLASH_R_BASE))
  DMA1* = (cast[ptr DMA_TypeDef](DMA1_BASE))
  DMA1_Stream0* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream0_BASE))
  DMA1_Stream1* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream1_BASE))
  DMA1_Stream2* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream2_BASE))
  DMA1_Stream3* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream3_BASE))
  DMA1_Stream4* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream4_BASE))
  DMA1_Stream5* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream5_BASE))
  DMA1_Stream6* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream6_BASE))
  DMA1_Stream7* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream7_BASE))
  DMA2* = (cast[ptr DMA_TypeDef](DMA2_BASE))
  DMA2_Stream0* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream0_BASE))
  DMA2_Stream1* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream1_BASE))
  DMA2_Stream2* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream2_BASE))
  DMA2_Stream3* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream3_BASE))
  DMA2_Stream4* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream4_BASE))
  DMA2_Stream5* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream5_BASE))
  DMA2_Stream6* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream6_BASE))
  DMA2_Stream7* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream7_BASE))
  ETH* = (cast[ptr ETH_TypeDef](ETH_BASE))
  DCMI* = (cast[ptr DCMI_TypeDef](DCMI_BASE))
  RNG* = (cast[ptr RNG_TypeDef](RNG_BASE))
  FSMC_Bank1* = (cast[ptr FSMC_Bank1_TypeDef](FSMC_Bank1_R_BASE))
  FSMC_Bank1E* = (cast[ptr FSMC_Bank1E_TypeDef](FSMC_Bank1E_R_BASE))
  FSMC_Bank2_3* = (cast[ptr FSMC_Bank2_3_TypeDef](FSMC_Bank2_3_R_BASE))
  FSMC_Bank4* = (cast[ptr FSMC_Bank4_TypeDef](FSMC_Bank4_R_BASE))
  DBGMCU* = (cast[ptr DBGMCU_TypeDef](DBGMCU_BASE))
  USB_OTG_FS* = (cast[ptr USB_OTG_GlobalTypeDef](USB_OTG_FS_PERIPH_BASE))
  USB_OTG_HS* = (cast[ptr USB_OTG_GlobalTypeDef](USB_OTG_HS_PERIPH_BASE))

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
##                         Analog to Digital Converter
##
## ****************************************************************************
##
##  @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
##

const
  ADC_MULTIMODE_SUPPORT* = true ## !<ADC Multimode feature available on specific devices

## *******************  Bit definition for ADC_SR register  *******************

const
  ADC_SR_AWD_Pos* = (0)
  ADC_SR_AWD_Msk* = (0x00000001 shl ADC_SR_AWD_Pos) ## !< 0x00000001
  ADC_SR_AWD* = ADC_SR_AWD_Msk
  ADC_SR_EOC_Pos* = (1)
  ADC_SR_EOC_Msk* = (0x00000001 shl ADC_SR_EOC_Pos) ## !< 0x00000002
  ADC_SR_EOC* = ADC_SR_EOC_Msk
  ADC_SR_JEOC_Pos* = (2)
  ADC_SR_JEOC_Msk* = (0x00000001 shl ADC_SR_JEOC_Pos) ## !< 0x00000004
  ADC_SR_JEOC* = ADC_SR_JEOC_Msk
  ADC_SR_JSTRT_Pos* = (3)
  ADC_SR_JSTRT_Msk* = (0x00000001 shl ADC_SR_JSTRT_Pos) ## !< 0x00000008
  ADC_SR_JSTRT* = ADC_SR_JSTRT_Msk
  ADC_SR_STRT_Pos* = (4)
  ADC_SR_STRT_Msk* = (0x00000001 shl ADC_SR_STRT_Pos) ## !< 0x00000010
  ADC_SR_STRT* = ADC_SR_STRT_Msk
  ADC_SR_OVR_Pos* = (5)
  ADC_SR_OVR_Msk* = (0x00000001 shl ADC_SR_OVR_Pos) ## !< 0x00000020
  ADC_SR_OVR* = ADC_SR_OVR_Msk

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
  ADC_CR1_EOCIE_Pos* = (5)
  ADC_CR1_EOCIE_Msk* = (0x00000001 shl ADC_CR1_EOCIE_Pos) ## !< 0x00000020
  ADC_CR1_EOCIE* = ADC_CR1_EOCIE_Msk
  ADC_CR1_AWDIE_Pos* = (6)
  ADC_CR1_AWDIE_Msk* = (0x00000001 shl ADC_CR1_AWDIE_Pos) ## !< 0x00000040
  ADC_CR1_AWDIE* = ADC_CR1_AWDIE_Msk
  ADC_CR1_JEOCIE_Pos* = (7)
  ADC_CR1_JEOCIE_Msk* = (0x00000001 shl ADC_CR1_JEOCIE_Pos) ## !< 0x00000080
  ADC_CR1_JEOCIE* = ADC_CR1_JEOCIE_Msk
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

## ******************  Bit definition for ADC_CR2 register  *******************

const
  ADC_CR2_ADON_Pos* = (0)
  ADC_CR2_ADON_Msk* = (0x00000001 shl ADC_CR2_ADON_Pos) ## !< 0x00000001
  ADC_CR2_ADON* = ADC_CR2_ADON_Msk
  ADC_CR2_CONT_Pos* = (1)
  ADC_CR2_CONT_Msk* = (0x00000001 shl ADC_CR2_CONT_Pos) ## !< 0x00000002
  ADC_CR2_CONT* = ADC_CR2_CONT_Msk
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
  ADC_SMPR1_SMP10_Pos* = (0)
  ADC_SMPR1_SMP10_Msk* = (0x00000007 shl ADC_SMPR1_SMP10_Pos) ## !< 0x00000007
  ADC_SMPR1_SMP10* = ADC_SMPR1_SMP10_Msk
  ADC_SMPR1_SMP10_0* = (0x00000001 shl ADC_SMPR1_SMP10_Pos) ## !< 0x00000001
  ADC_SMPR1_SMP10_1* = (0x00000002 shl ADC_SMPR1_SMP10_Pos) ## !< 0x00000002
  ADC_SMPR1_SMP10_2* = (0x00000004 shl ADC_SMPR1_SMP10_Pos) ## !< 0x00000004
  ADC_SMPR1_SMP11_Pos* = (3)
  ADC_SMPR1_SMP11_Msk* = (0x00000007 shl ADC_SMPR1_SMP11_Pos) ## !< 0x00000038
  ADC_SMPR1_SMP11* = ADC_SMPR1_SMP11_Msk
  ADC_SMPR1_SMP11_0* = (0x00000001 shl ADC_SMPR1_SMP11_Pos) ## !< 0x00000008
  ADC_SMPR1_SMP11_1* = (0x00000002 shl ADC_SMPR1_SMP11_Pos) ## !< 0x00000010
  ADC_SMPR1_SMP11_2* = (0x00000004 shl ADC_SMPR1_SMP11_Pos) ## !< 0x00000020
  ADC_SMPR1_SMP12_Pos* = (6)
  ADC_SMPR1_SMP12_Msk* = (0x00000007 shl ADC_SMPR1_SMP12_Pos) ## !< 0x000001C0
  ADC_SMPR1_SMP12* = ADC_SMPR1_SMP12_Msk
  ADC_SMPR1_SMP12_0* = (0x00000001 shl ADC_SMPR1_SMP12_Pos) ## !< 0x00000040
  ADC_SMPR1_SMP12_1* = (0x00000002 shl ADC_SMPR1_SMP12_Pos) ## !< 0x00000080
  ADC_SMPR1_SMP12_2* = (0x00000004 shl ADC_SMPR1_SMP12_Pos) ## !< 0x00000100
  ADC_SMPR1_SMP13_Pos* = (9)
  ADC_SMPR1_SMP13_Msk* = (0x00000007 shl ADC_SMPR1_SMP13_Pos) ## !< 0x00000E00
  ADC_SMPR1_SMP13* = ADC_SMPR1_SMP13_Msk
  ADC_SMPR1_SMP13_0* = (0x00000001 shl ADC_SMPR1_SMP13_Pos) ## !< 0x00000200
  ADC_SMPR1_SMP13_1* = (0x00000002 shl ADC_SMPR1_SMP13_Pos) ## !< 0x00000400
  ADC_SMPR1_SMP13_2* = (0x00000004 shl ADC_SMPR1_SMP13_Pos) ## !< 0x00000800
  ADC_SMPR1_SMP14_Pos* = (12)
  ADC_SMPR1_SMP14_Msk* = (0x00000007 shl ADC_SMPR1_SMP14_Pos) ## !< 0x00007000
  ADC_SMPR1_SMP14* = ADC_SMPR1_SMP14_Msk
  ADC_SMPR1_SMP14_0* = (0x00000001 shl ADC_SMPR1_SMP14_Pos) ## !< 0x00001000
  ADC_SMPR1_SMP14_1* = (0x00000002 shl ADC_SMPR1_SMP14_Pos) ## !< 0x00002000
  ADC_SMPR1_SMP14_2* = (0x00000004 shl ADC_SMPR1_SMP14_Pos) ## !< 0x00004000
  ADC_SMPR1_SMP15_Pos* = (15)
  ADC_SMPR1_SMP15_Msk* = (0x00000007 shl ADC_SMPR1_SMP15_Pos) ## !< 0x00038000
  ADC_SMPR1_SMP15* = ADC_SMPR1_SMP15_Msk
  ADC_SMPR1_SMP15_0* = (0x00000001 shl ADC_SMPR1_SMP15_Pos) ## !< 0x00008000
  ADC_SMPR1_SMP15_1* = (0x00000002 shl ADC_SMPR1_SMP15_Pos) ## !< 0x00010000
  ADC_SMPR1_SMP15_2* = (0x00000004 shl ADC_SMPR1_SMP15_Pos) ## !< 0x00020000
  ADC_SMPR1_SMP16_Pos* = (18)
  ADC_SMPR1_SMP16_Msk* = (0x00000007 shl ADC_SMPR1_SMP16_Pos) ## !< 0x001C0000
  ADC_SMPR1_SMP16* = ADC_SMPR1_SMP16_Msk
  ADC_SMPR1_SMP16_0* = (0x00000001 shl ADC_SMPR1_SMP16_Pos) ## !< 0x00040000
  ADC_SMPR1_SMP16_1* = (0x00000002 shl ADC_SMPR1_SMP16_Pos) ## !< 0x00080000
  ADC_SMPR1_SMP16_2* = (0x00000004 shl ADC_SMPR1_SMP16_Pos) ## !< 0x00100000
  ADC_SMPR1_SMP17_Pos* = (21)
  ADC_SMPR1_SMP17_Msk* = (0x00000007 shl ADC_SMPR1_SMP17_Pos) ## !< 0x00E00000
  ADC_SMPR1_SMP17* = ADC_SMPR1_SMP17_Msk
  ADC_SMPR1_SMP17_0* = (0x00000001 shl ADC_SMPR1_SMP17_Pos) ## !< 0x00200000
  ADC_SMPR1_SMP17_1* = (0x00000002 shl ADC_SMPR1_SMP17_Pos) ## !< 0x00400000
  ADC_SMPR1_SMP17_2* = (0x00000004 shl ADC_SMPR1_SMP17_Pos) ## !< 0x00800000
  ADC_SMPR1_SMP18_Pos* = (24)
  ADC_SMPR1_SMP18_Msk* = (0x00000007 shl ADC_SMPR1_SMP18_Pos) ## !< 0x07000000
  ADC_SMPR1_SMP18* = ADC_SMPR1_SMP18_Msk
  ADC_SMPR1_SMP18_0* = (0x00000001 shl ADC_SMPR1_SMP18_Pos) ## !< 0x01000000
  ADC_SMPR1_SMP18_1* = (0x00000002 shl ADC_SMPR1_SMP18_Pos) ## !< 0x02000000
  ADC_SMPR1_SMP18_2* = (0x00000004 shl ADC_SMPR1_SMP18_Pos) ## !< 0x04000000

## *****************  Bit definition for ADC_SMPR2 register  ******************

const
  ADC_SMPR2_SMP0_Pos* = (0)
  ADC_SMPR2_SMP0_Msk* = (0x00000007 shl ADC_SMPR2_SMP0_Pos) ## !< 0x00000007
  ADC_SMPR2_SMP0* = ADC_SMPR2_SMP0_Msk
  ADC_SMPR2_SMP0_0* = (0x00000001 shl ADC_SMPR2_SMP0_Pos) ## !< 0x00000001
  ADC_SMPR2_SMP0_1* = (0x00000002 shl ADC_SMPR2_SMP0_Pos) ## !< 0x00000002
  ADC_SMPR2_SMP0_2* = (0x00000004 shl ADC_SMPR2_SMP0_Pos) ## !< 0x00000004
  ADC_SMPR2_SMP1_Pos* = (3)
  ADC_SMPR2_SMP1_Msk* = (0x00000007 shl ADC_SMPR2_SMP1_Pos) ## !< 0x00000038
  ADC_SMPR2_SMP1* = ADC_SMPR2_SMP1_Msk
  ADC_SMPR2_SMP1_0* = (0x00000001 shl ADC_SMPR2_SMP1_Pos) ## !< 0x00000008
  ADC_SMPR2_SMP1_1* = (0x00000002 shl ADC_SMPR2_SMP1_Pos) ## !< 0x00000010
  ADC_SMPR2_SMP1_2* = (0x00000004 shl ADC_SMPR2_SMP1_Pos) ## !< 0x00000020
  ADC_SMPR2_SMP2_Pos* = (6)
  ADC_SMPR2_SMP2_Msk* = (0x00000007 shl ADC_SMPR2_SMP2_Pos) ## !< 0x000001C0
  ADC_SMPR2_SMP2* = ADC_SMPR2_SMP2_Msk
  ADC_SMPR2_SMP2_0* = (0x00000001 shl ADC_SMPR2_SMP2_Pos) ## !< 0x00000040
  ADC_SMPR2_SMP2_1* = (0x00000002 shl ADC_SMPR2_SMP2_Pos) ## !< 0x00000080
  ADC_SMPR2_SMP2_2* = (0x00000004 shl ADC_SMPR2_SMP2_Pos) ## !< 0x00000100
  ADC_SMPR2_SMP3_Pos* = (9)
  ADC_SMPR2_SMP3_Msk* = (0x00000007 shl ADC_SMPR2_SMP3_Pos) ## !< 0x00000E00
  ADC_SMPR2_SMP3* = ADC_SMPR2_SMP3_Msk
  ADC_SMPR2_SMP3_0* = (0x00000001 shl ADC_SMPR2_SMP3_Pos) ## !< 0x00000200
  ADC_SMPR2_SMP3_1* = (0x00000002 shl ADC_SMPR2_SMP3_Pos) ## !< 0x00000400
  ADC_SMPR2_SMP3_2* = (0x00000004 shl ADC_SMPR2_SMP3_Pos) ## !< 0x00000800
  ADC_SMPR2_SMP4_Pos* = (12)
  ADC_SMPR2_SMP4_Msk* = (0x00000007 shl ADC_SMPR2_SMP4_Pos) ## !< 0x00007000
  ADC_SMPR2_SMP4* = ADC_SMPR2_SMP4_Msk
  ADC_SMPR2_SMP4_0* = (0x00000001 shl ADC_SMPR2_SMP4_Pos) ## !< 0x00001000
  ADC_SMPR2_SMP4_1* = (0x00000002 shl ADC_SMPR2_SMP4_Pos) ## !< 0x00002000
  ADC_SMPR2_SMP4_2* = (0x00000004 shl ADC_SMPR2_SMP4_Pos) ## !< 0x00004000
  ADC_SMPR2_SMP5_Pos* = (15)
  ADC_SMPR2_SMP5_Msk* = (0x00000007 shl ADC_SMPR2_SMP5_Pos) ## !< 0x00038000
  ADC_SMPR2_SMP5* = ADC_SMPR2_SMP5_Msk
  ADC_SMPR2_SMP5_0* = (0x00000001 shl ADC_SMPR2_SMP5_Pos) ## !< 0x00008000
  ADC_SMPR2_SMP5_1* = (0x00000002 shl ADC_SMPR2_SMP5_Pos) ## !< 0x00010000
  ADC_SMPR2_SMP5_2* = (0x00000004 shl ADC_SMPR2_SMP5_Pos) ## !< 0x00020000
  ADC_SMPR2_SMP6_Pos* = (18)
  ADC_SMPR2_SMP6_Msk* = (0x00000007 shl ADC_SMPR2_SMP6_Pos) ## !< 0x001C0000
  ADC_SMPR2_SMP6* = ADC_SMPR2_SMP6_Msk
  ADC_SMPR2_SMP6_0* = (0x00000001 shl ADC_SMPR2_SMP6_Pos) ## !< 0x00040000
  ADC_SMPR2_SMP6_1* = (0x00000002 shl ADC_SMPR2_SMP6_Pos) ## !< 0x00080000
  ADC_SMPR2_SMP6_2* = (0x00000004 shl ADC_SMPR2_SMP6_Pos) ## !< 0x00100000
  ADC_SMPR2_SMP7_Pos* = (21)
  ADC_SMPR2_SMP7_Msk* = (0x00000007 shl ADC_SMPR2_SMP7_Pos) ## !< 0x00E00000
  ADC_SMPR2_SMP7* = ADC_SMPR2_SMP7_Msk
  ADC_SMPR2_SMP7_0* = (0x00000001 shl ADC_SMPR2_SMP7_Pos) ## !< 0x00200000
  ADC_SMPR2_SMP7_1* = (0x00000002 shl ADC_SMPR2_SMP7_Pos) ## !< 0x00400000
  ADC_SMPR2_SMP7_2* = (0x00000004 shl ADC_SMPR2_SMP7_Pos) ## !< 0x00800000
  ADC_SMPR2_SMP8_Pos* = (24)
  ADC_SMPR2_SMP8_Msk* = (0x00000007 shl ADC_SMPR2_SMP8_Pos) ## !< 0x07000000
  ADC_SMPR2_SMP8* = ADC_SMPR2_SMP8_Msk
  ADC_SMPR2_SMP8_0* = (0x00000001 shl ADC_SMPR2_SMP8_Pos) ## !< 0x01000000
  ADC_SMPR2_SMP8_1* = (0x00000002 shl ADC_SMPR2_SMP8_Pos) ## !< 0x02000000
  ADC_SMPR2_SMP8_2* = (0x00000004 shl ADC_SMPR2_SMP8_Pos) ## !< 0x04000000
  ADC_SMPR2_SMP9_Pos* = (27)
  ADC_SMPR2_SMP9_Msk* = (0x00000007 shl ADC_SMPR2_SMP9_Pos) ## !< 0x38000000
  ADC_SMPR2_SMP9* = ADC_SMPR2_SMP9_Msk
  ADC_SMPR2_SMP9_0* = (0x00000001 shl ADC_SMPR2_SMP9_Pos) ## !< 0x08000000
  ADC_SMPR2_SMP9_1* = (0x00000002 shl ADC_SMPR2_SMP9_Pos) ## !< 0x10000000
  ADC_SMPR2_SMP9_2* = (0x00000004 shl ADC_SMPR2_SMP9_Pos) ## !< 0x20000000

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
  ADC_SQR1_SQ13_Pos* = (0)
  ADC_SQR1_SQ13_Msk* = (0x0000001F shl ADC_SQR1_SQ13_Pos) ## !< 0x0000001F
  ADC_SQR1_SQ13* = ADC_SQR1_SQ13_Msk
  ADC_SQR1_SQ13_0* = (0x00000001 shl ADC_SQR1_SQ13_Pos) ## !< 0x00000001
  ADC_SQR1_SQ13_1* = (0x00000002 shl ADC_SQR1_SQ13_Pos) ## !< 0x00000002
  ADC_SQR1_SQ13_2* = (0x00000004 shl ADC_SQR1_SQ13_Pos) ## !< 0x00000004
  ADC_SQR1_SQ13_3* = (0x00000008 shl ADC_SQR1_SQ13_Pos) ## !< 0x00000008
  ADC_SQR1_SQ13_4* = (0x00000010 shl ADC_SQR1_SQ13_Pos) ## !< 0x00000010
  ADC_SQR1_SQ14_Pos* = (5)
  ADC_SQR1_SQ14_Msk* = (0x0000001F shl ADC_SQR1_SQ14_Pos) ## !< 0x000003E0
  ADC_SQR1_SQ14* = ADC_SQR1_SQ14_Msk
  ADC_SQR1_SQ14_0* = (0x00000001 shl ADC_SQR1_SQ14_Pos) ## !< 0x00000020
  ADC_SQR1_SQ14_1* = (0x00000002 shl ADC_SQR1_SQ14_Pos) ## !< 0x00000040
  ADC_SQR1_SQ14_2* = (0x00000004 shl ADC_SQR1_SQ14_Pos) ## !< 0x00000080
  ADC_SQR1_SQ14_3* = (0x00000008 shl ADC_SQR1_SQ14_Pos) ## !< 0x00000100
  ADC_SQR1_SQ14_4* = (0x00000010 shl ADC_SQR1_SQ14_Pos) ## !< 0x00000200
  ADC_SQR1_SQ15_Pos* = (10)
  ADC_SQR1_SQ15_Msk* = (0x0000001F shl ADC_SQR1_SQ15_Pos) ## !< 0x00007C00
  ADC_SQR1_SQ15* = ADC_SQR1_SQ15_Msk
  ADC_SQR1_SQ15_0* = (0x00000001 shl ADC_SQR1_SQ15_Pos) ## !< 0x00000400
  ADC_SQR1_SQ15_1* = (0x00000002 shl ADC_SQR1_SQ15_Pos) ## !< 0x00000800
  ADC_SQR1_SQ15_2* = (0x00000004 shl ADC_SQR1_SQ15_Pos) ## !< 0x00001000
  ADC_SQR1_SQ15_3* = (0x00000008 shl ADC_SQR1_SQ15_Pos) ## !< 0x00002000
  ADC_SQR1_SQ15_4* = (0x00000010 shl ADC_SQR1_SQ15_Pos) ## !< 0x00004000
  ADC_SQR1_SQ16_Pos* = (15)
  ADC_SQR1_SQ16_Msk* = (0x0000001F shl ADC_SQR1_SQ16_Pos) ## !< 0x000F8000
  ADC_SQR1_SQ16* = ADC_SQR1_SQ16_Msk
  ADC_SQR1_SQ16_0* = (0x00000001 shl ADC_SQR1_SQ16_Pos) ## !< 0x00008000
  ADC_SQR1_SQ16_1* = (0x00000002 shl ADC_SQR1_SQ16_Pos) ## !< 0x00010000
  ADC_SQR1_SQ16_2* = (0x00000004 shl ADC_SQR1_SQ16_Pos) ## !< 0x00020000
  ADC_SQR1_SQ16_3* = (0x00000008 shl ADC_SQR1_SQ16_Pos) ## !< 0x00040000
  ADC_SQR1_SQ16_4* = (0x00000010 shl ADC_SQR1_SQ16_Pos) ## !< 0x00080000
  ADC_SQR1_L_Pos* = (20)
  ADC_SQR1_L_Msk* = (0x0000000F shl ADC_SQR1_L_Pos) ## !< 0x00F00000
  ADC_SQR1_L* = ADC_SQR1_L_Msk
  ADC_SQR1_L_0* = (0x00000001 shl ADC_SQR1_L_Pos) ## !< 0x00100000
  ADC_SQR1_L_1* = (0x00000002 shl ADC_SQR1_L_Pos) ## !< 0x00200000
  ADC_SQR1_L_2* = (0x00000004 shl ADC_SQR1_L_Pos) ## !< 0x00400000
  ADC_SQR1_L_3* = (0x00000008 shl ADC_SQR1_L_Pos) ## !< 0x00800000

## ******************  Bit definition for ADC_SQR2 register  ******************

const
  ADC_SQR2_SQ7_Pos* = (0)
  ADC_SQR2_SQ7_Msk* = (0x0000001F shl ADC_SQR2_SQ7_Pos) ## !< 0x0000001F
  ADC_SQR2_SQ7* = ADC_SQR2_SQ7_Msk
  ADC_SQR2_SQ7_0* = (0x00000001 shl ADC_SQR2_SQ7_Pos) ## !< 0x00000001
  ADC_SQR2_SQ7_1* = (0x00000002 shl ADC_SQR2_SQ7_Pos) ## !< 0x00000002
  ADC_SQR2_SQ7_2* = (0x00000004 shl ADC_SQR2_SQ7_Pos) ## !< 0x00000004
  ADC_SQR2_SQ7_3* = (0x00000008 shl ADC_SQR2_SQ7_Pos) ## !< 0x00000008
  ADC_SQR2_SQ7_4* = (0x00000010 shl ADC_SQR2_SQ7_Pos) ## !< 0x00000010
  ADC_SQR2_SQ8_Pos* = (5)
  ADC_SQR2_SQ8_Msk* = (0x0000001F shl ADC_SQR2_SQ8_Pos) ## !< 0x000003E0
  ADC_SQR2_SQ8* = ADC_SQR2_SQ8_Msk
  ADC_SQR2_SQ8_0* = (0x00000001 shl ADC_SQR2_SQ8_Pos) ## !< 0x00000020
  ADC_SQR2_SQ8_1* = (0x00000002 shl ADC_SQR2_SQ8_Pos) ## !< 0x00000040
  ADC_SQR2_SQ8_2* = (0x00000004 shl ADC_SQR2_SQ8_Pos) ## !< 0x00000080
  ADC_SQR2_SQ8_3* = (0x00000008 shl ADC_SQR2_SQ8_Pos) ## !< 0x00000100
  ADC_SQR2_SQ8_4* = (0x00000010 shl ADC_SQR2_SQ8_Pos) ## !< 0x00000200
  ADC_SQR2_SQ9_Pos* = (10)
  ADC_SQR2_SQ9_Msk* = (0x0000001F shl ADC_SQR2_SQ9_Pos) ## !< 0x00007C00
  ADC_SQR2_SQ9* = ADC_SQR2_SQ9_Msk
  ADC_SQR2_SQ9_0* = (0x00000001 shl ADC_SQR2_SQ9_Pos) ## !< 0x00000400
  ADC_SQR2_SQ9_1* = (0x00000002 shl ADC_SQR2_SQ9_Pos) ## !< 0x00000800
  ADC_SQR2_SQ9_2* = (0x00000004 shl ADC_SQR2_SQ9_Pos) ## !< 0x00001000
  ADC_SQR2_SQ9_3* = (0x00000008 shl ADC_SQR2_SQ9_Pos) ## !< 0x00002000
  ADC_SQR2_SQ9_4* = (0x00000010 shl ADC_SQR2_SQ9_Pos) ## !< 0x00004000
  ADC_SQR2_SQ10_Pos* = (15)
  ADC_SQR2_SQ10_Msk* = (0x0000001F shl ADC_SQR2_SQ10_Pos) ## !< 0x000F8000
  ADC_SQR2_SQ10* = ADC_SQR2_SQ10_Msk
  ADC_SQR2_SQ10_0* = (0x00000001 shl ADC_SQR2_SQ10_Pos) ## !< 0x00008000
  ADC_SQR2_SQ10_1* = (0x00000002 shl ADC_SQR2_SQ10_Pos) ## !< 0x00010000
  ADC_SQR2_SQ10_2* = (0x00000004 shl ADC_SQR2_SQ10_Pos) ## !< 0x00020000
  ADC_SQR2_SQ10_3* = (0x00000008 shl ADC_SQR2_SQ10_Pos) ## !< 0x00040000
  ADC_SQR2_SQ10_4* = (0x00000010 shl ADC_SQR2_SQ10_Pos) ## !< 0x00080000
  ADC_SQR2_SQ11_Pos* = (20)
  ADC_SQR2_SQ11_Msk* = (0x0000001F shl ADC_SQR2_SQ11_Pos) ## !< 0x01F00000
  ADC_SQR2_SQ11* = ADC_SQR2_SQ11_Msk
  ADC_SQR2_SQ11_0* = (0x00000001 shl ADC_SQR2_SQ11_Pos) ## !< 0x00100000
  ADC_SQR2_SQ11_1* = (0x00000002 shl ADC_SQR2_SQ11_Pos) ## !< 0x00200000
  ADC_SQR2_SQ11_2* = (0x00000004 shl ADC_SQR2_SQ11_Pos) ## !< 0x00400000
  ADC_SQR2_SQ11_3* = (0x00000008 shl ADC_SQR2_SQ11_Pos) ## !< 0x00800000
  ADC_SQR2_SQ11_4* = (0x00000010 shl ADC_SQR2_SQ11_Pos) ## !< 0x01000000
  ADC_SQR2_SQ12_Pos* = (25)
  ADC_SQR2_SQ12_Msk* = (0x0000001F shl ADC_SQR2_SQ12_Pos) ## !< 0x3E000000
  ADC_SQR2_SQ12* = ADC_SQR2_SQ12_Msk
  ADC_SQR2_SQ12_0* = (0x00000001 shl ADC_SQR2_SQ12_Pos) ## !< 0x02000000
  ADC_SQR2_SQ12_1* = (0x00000002 shl ADC_SQR2_SQ12_Pos) ## !< 0x04000000
  ADC_SQR2_SQ12_2* = (0x00000004 shl ADC_SQR2_SQ12_Pos) ## !< 0x08000000
  ADC_SQR2_SQ12_3* = (0x00000008 shl ADC_SQR2_SQ12_Pos) ## !< 0x10000000
  ADC_SQR2_SQ12_4* = (0x00000010 shl ADC_SQR2_SQ12_Pos) ## !< 0x20000000

## ******************  Bit definition for ADC_SQR3 register  ******************

const
  ADC_SQR3_SQ1_Pos* = (0)
  ADC_SQR3_SQ1_Msk* = (0x0000001F shl ADC_SQR3_SQ1_Pos) ## !< 0x0000001F
  ADC_SQR3_SQ1* = ADC_SQR3_SQ1_Msk
  ADC_SQR3_SQ1_0* = (0x00000001 shl ADC_SQR3_SQ1_Pos) ## !< 0x00000001
  ADC_SQR3_SQ1_1* = (0x00000002 shl ADC_SQR3_SQ1_Pos) ## !< 0x00000002
  ADC_SQR3_SQ1_2* = (0x00000004 shl ADC_SQR3_SQ1_Pos) ## !< 0x00000004
  ADC_SQR3_SQ1_3* = (0x00000008 shl ADC_SQR3_SQ1_Pos) ## !< 0x00000008
  ADC_SQR3_SQ1_4* = (0x00000010 shl ADC_SQR3_SQ1_Pos) ## !< 0x00000010
  ADC_SQR3_SQ2_Pos* = (5)
  ADC_SQR3_SQ2_Msk* = (0x0000001F shl ADC_SQR3_SQ2_Pos) ## !< 0x000003E0
  ADC_SQR3_SQ2* = ADC_SQR3_SQ2_Msk
  ADC_SQR3_SQ2_0* = (0x00000001 shl ADC_SQR3_SQ2_Pos) ## !< 0x00000020
  ADC_SQR3_SQ2_1* = (0x00000002 shl ADC_SQR3_SQ2_Pos) ## !< 0x00000040
  ADC_SQR3_SQ2_2* = (0x00000004 shl ADC_SQR3_SQ2_Pos) ## !< 0x00000080
  ADC_SQR3_SQ2_3* = (0x00000008 shl ADC_SQR3_SQ2_Pos) ## !< 0x00000100
  ADC_SQR3_SQ2_4* = (0x00000010 shl ADC_SQR3_SQ2_Pos) ## !< 0x00000200
  ADC_SQR3_SQ3_Pos* = (10)
  ADC_SQR3_SQ3_Msk* = (0x0000001F shl ADC_SQR3_SQ3_Pos) ## !< 0x00007C00
  ADC_SQR3_SQ3* = ADC_SQR3_SQ3_Msk
  ADC_SQR3_SQ3_0* = (0x00000001 shl ADC_SQR3_SQ3_Pos) ## !< 0x00000400
  ADC_SQR3_SQ3_1* = (0x00000002 shl ADC_SQR3_SQ3_Pos) ## !< 0x00000800
  ADC_SQR3_SQ3_2* = (0x00000004 shl ADC_SQR3_SQ3_Pos) ## !< 0x00001000
  ADC_SQR3_SQ3_3* = (0x00000008 shl ADC_SQR3_SQ3_Pos) ## !< 0x00002000
  ADC_SQR3_SQ3_4* = (0x00000010 shl ADC_SQR3_SQ3_Pos) ## !< 0x00004000
  ADC_SQR3_SQ4_Pos* = (15)
  ADC_SQR3_SQ4_Msk* = (0x0000001F shl ADC_SQR3_SQ4_Pos) ## !< 0x000F8000
  ADC_SQR3_SQ4* = ADC_SQR3_SQ4_Msk
  ADC_SQR3_SQ4_0* = (0x00000001 shl ADC_SQR3_SQ4_Pos) ## !< 0x00008000
  ADC_SQR3_SQ4_1* = (0x00000002 shl ADC_SQR3_SQ4_Pos) ## !< 0x00010000
  ADC_SQR3_SQ4_2* = (0x00000004 shl ADC_SQR3_SQ4_Pos) ## !< 0x00020000
  ADC_SQR3_SQ4_3* = (0x00000008 shl ADC_SQR3_SQ4_Pos) ## !< 0x00040000
  ADC_SQR3_SQ4_4* = (0x00000010 shl ADC_SQR3_SQ4_Pos) ## !< 0x00080000
  ADC_SQR3_SQ5_Pos* = (20)
  ADC_SQR3_SQ5_Msk* = (0x0000001F shl ADC_SQR3_SQ5_Pos) ## !< 0x01F00000
  ADC_SQR3_SQ5* = ADC_SQR3_SQ5_Msk
  ADC_SQR3_SQ5_0* = (0x00000001 shl ADC_SQR3_SQ5_Pos) ## !< 0x00100000
  ADC_SQR3_SQ5_1* = (0x00000002 shl ADC_SQR3_SQ5_Pos) ## !< 0x00200000
  ADC_SQR3_SQ5_2* = (0x00000004 shl ADC_SQR3_SQ5_Pos) ## !< 0x00400000
  ADC_SQR3_SQ5_3* = (0x00000008 shl ADC_SQR3_SQ5_Pos) ## !< 0x00800000
  ADC_SQR3_SQ5_4* = (0x00000010 shl ADC_SQR3_SQ5_Pos) ## !< 0x01000000
  ADC_SQR3_SQ6_Pos* = (25)
  ADC_SQR3_SQ6_Msk* = (0x0000001F shl ADC_SQR3_SQ6_Pos) ## !< 0x3E000000
  ADC_SQR3_SQ6* = ADC_SQR3_SQ6_Msk
  ADC_SQR3_SQ6_0* = (0x00000001 shl ADC_SQR3_SQ6_Pos) ## !< 0x02000000
  ADC_SQR3_SQ6_1* = (0x00000002 shl ADC_SQR3_SQ6_Pos) ## !< 0x04000000
  ADC_SQR3_SQ6_2* = (0x00000004 shl ADC_SQR3_SQ6_Pos) ## !< 0x08000000
  ADC_SQR3_SQ6_3* = (0x00000008 shl ADC_SQR3_SQ6_Pos) ## !< 0x10000000
  ADC_SQR3_SQ6_4* = (0x00000010 shl ADC_SQR3_SQ6_Pos) ## !< 0x20000000

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
  ADC_DR_ADC2DATA_Pos* = (16)
  ADC_DR_ADC2DATA_Msk* = (0x0000FFFF shl ADC_DR_ADC2DATA_Pos) ## !< 0xFFFF0000
  ADC_DR_ADC2DATA* = ADC_DR_ADC2DATA_Msk

## ******************  Bit definition for ADC_CSR register  *******************

const
  ADC_CSR_AWD1_Pos* = (0)
  ADC_CSR_AWD1_Msk* = (0x00000001 shl ADC_CSR_AWD1_Pos) ## !< 0x00000001
  ADC_CSR_AWD1* = ADC_CSR_AWD1_Msk
  ADC_CSR_EOC1_Pos* = (1)
  ADC_CSR_EOC1_Msk* = (0x00000001 shl ADC_CSR_EOC1_Pos) ## !< 0x00000002
  ADC_CSR_EOC1* = ADC_CSR_EOC1_Msk
  ADC_CSR_JEOC1_Pos* = (2)
  ADC_CSR_JEOC1_Msk* = (0x00000001 shl ADC_CSR_JEOC1_Pos) ## !< 0x00000004
  ADC_CSR_JEOC1* = ADC_CSR_JEOC1_Msk
  ADC_CSR_JSTRT1_Pos* = (3)
  ADC_CSR_JSTRT1_Msk* = (0x00000001 shl ADC_CSR_JSTRT1_Pos) ## !< 0x00000008
  ADC_CSR_JSTRT1* = ADC_CSR_JSTRT1_Msk
  ADC_CSR_STRT1_Pos* = (4)
  ADC_CSR_STRT1_Msk* = (0x00000001 shl ADC_CSR_STRT1_Pos) ## !< 0x00000010
  ADC_CSR_STRT1* = ADC_CSR_STRT1_Msk
  ADC_CSR_OVR1_Pos* = (5)
  ADC_CSR_OVR1_Msk* = (0x00000001 shl ADC_CSR_OVR1_Pos) ## !< 0x00000020
  ADC_CSR_OVR1* = ADC_CSR_OVR1_Msk
  ADC_CSR_AWD2_Pos* = (8)
  ADC_CSR_AWD2_Msk* = (0x00000001 shl ADC_CSR_AWD2_Pos) ## !< 0x00000100
  ADC_CSR_AWD2* = ADC_CSR_AWD2_Msk
  ADC_CSR_EOC2_Pos* = (9)
  ADC_CSR_EOC2_Msk* = (0x00000001 shl ADC_CSR_EOC2_Pos) ## !< 0x00000200
  ADC_CSR_EOC2* = ADC_CSR_EOC2_Msk
  ADC_CSR_JEOC2_Pos* = (10)
  ADC_CSR_JEOC2_Msk* = (0x00000001 shl ADC_CSR_JEOC2_Pos) ## !< 0x00000400
  ADC_CSR_JEOC2* = ADC_CSR_JEOC2_Msk
  ADC_CSR_JSTRT2_Pos* = (11)
  ADC_CSR_JSTRT2_Msk* = (0x00000001 shl ADC_CSR_JSTRT2_Pos) ## !< 0x00000800
  ADC_CSR_JSTRT2* = ADC_CSR_JSTRT2_Msk
  ADC_CSR_STRT2_Pos* = (12)
  ADC_CSR_STRT2_Msk* = (0x00000001 shl ADC_CSR_STRT2_Pos) ## !< 0x00001000
  ADC_CSR_STRT2* = ADC_CSR_STRT2_Msk
  ADC_CSR_OVR2_Pos* = (13)
  ADC_CSR_OVR2_Msk* = (0x00000001 shl ADC_CSR_OVR2_Pos) ## !< 0x00002000
  ADC_CSR_OVR2* = ADC_CSR_OVR2_Msk
  ADC_CSR_AWD3_Pos* = (16)
  ADC_CSR_AWD3_Msk* = (0x00000001 shl ADC_CSR_AWD3_Pos) ## !< 0x00010000
  ADC_CSR_AWD3* = ADC_CSR_AWD3_Msk
  ADC_CSR_EOC3_Pos* = (17)
  ADC_CSR_EOC3_Msk* = (0x00000001 shl ADC_CSR_EOC3_Pos) ## !< 0x00020000
  ADC_CSR_EOC3* = ADC_CSR_EOC3_Msk
  ADC_CSR_JEOC3_Pos* = (18)
  ADC_CSR_JEOC3_Msk* = (0x00000001 shl ADC_CSR_JEOC3_Pos) ## !< 0x00040000
  ADC_CSR_JEOC3* = ADC_CSR_JEOC3_Msk
  ADC_CSR_JSTRT3_Pos* = (19)
  ADC_CSR_JSTRT3_Msk* = (0x00000001 shl ADC_CSR_JSTRT3_Pos) ## !< 0x00080000
  ADC_CSR_JSTRT3* = ADC_CSR_JSTRT3_Msk
  ADC_CSR_STRT3_Pos* = (20)
  ADC_CSR_STRT3_Msk* = (0x00000001 shl ADC_CSR_STRT3_Pos) ## !< 0x00100000
  ADC_CSR_STRT3* = ADC_CSR_STRT3_Msk
  ADC_CSR_OVR3_Pos* = (21)
  ADC_CSR_OVR3_Msk* = (0x00000001 shl ADC_CSR_OVR3_Pos) ## !< 0x00200000
  ADC_CSR_OVR3* = ADC_CSR_OVR3_Msk

##  Legacy defines

const
  ADC_CSR_DOVR1* = ADC_CSR_OVR1
  ADC_CSR_DOVR2* = ADC_CSR_OVR2
  ADC_CSR_DOVR3* = ADC_CSR_OVR3

## ******************  Bit definition for ADC_CCR register  *******************

const
  ADC_CCR_MULTI_Pos* = (0)
  ADC_CCR_MULTI_Msk* = (0x0000001F shl ADC_CCR_MULTI_Pos) ## !< 0x0000001F
  ADC_CCR_MULTI* = ADC_CCR_MULTI_Msk
  ADC_CCR_MULTI_0* = (0x00000001 shl ADC_CCR_MULTI_Pos) ## !< 0x00000001
  ADC_CCR_MULTI_1* = (0x00000002 shl ADC_CCR_MULTI_Pos) ## !< 0x00000002
  ADC_CCR_MULTI_2* = (0x00000004 shl ADC_CCR_MULTI_Pos) ## !< 0x00000004
  ADC_CCR_MULTI_3* = (0x00000008 shl ADC_CCR_MULTI_Pos) ## !< 0x00000008
  ADC_CCR_MULTI_4* = (0x00000010 shl ADC_CCR_MULTI_Pos) ## !< 0x00000010
  ADC_CCR_DELAY_Pos* = (8)
  ADC_CCR_DELAY_Msk* = (0x0000000F shl ADC_CCR_DELAY_Pos) ## !< 0x00000F00
  ADC_CCR_DELAY* = ADC_CCR_DELAY_Msk
  ADC_CCR_DELAY_0* = (0x00000001 shl ADC_CCR_DELAY_Pos) ## !< 0x00000100
  ADC_CCR_DELAY_1* = (0x00000002 shl ADC_CCR_DELAY_Pos) ## !< 0x00000200
  ADC_CCR_DELAY_2* = (0x00000004 shl ADC_CCR_DELAY_Pos) ## !< 0x00000400
  ADC_CCR_DELAY_3* = (0x00000008 shl ADC_CCR_DELAY_Pos) ## !< 0x00000800
  ADC_CCR_DDS_Pos* = (13)
  ADC_CCR_DDS_Msk* = (0x00000001 shl ADC_CCR_DDS_Pos) ## !< 0x00002000
  ADC_CCR_DDS* = ADC_CCR_DDS_Msk
  ADC_CCR_DMA_Pos* = (14)
  ADC_CCR_DMA_Msk* = (0x00000003 shl ADC_CCR_DMA_Pos) ## !< 0x0000C000
  ADC_CCR_DMA* = ADC_CCR_DMA_Msk
  ADC_CCR_DMA_0* = (0x00000001 shl ADC_CCR_DMA_Pos) ## !< 0x00004000
  ADC_CCR_DMA_1* = (0x00000002 shl ADC_CCR_DMA_Pos) ## !< 0x00008000
  ADC_CCR_ADCPRE_Pos* = (16)
  ADC_CCR_ADCPRE_Msk* = (0x00000003 shl ADC_CCR_ADCPRE_Pos) ## !< 0x00030000
  ADC_CCR_ADCPRE* = ADC_CCR_ADCPRE_Msk
  ADC_CCR_ADCPRE_0* = (0x00000001 shl ADC_CCR_ADCPRE_Pos) ## !< 0x00010000
  ADC_CCR_ADCPRE_1* = (0x00000002 shl ADC_CCR_ADCPRE_Pos) ## !< 0x00020000
  ADC_CCR_VBATE_Pos* = (22)
  ADC_CCR_VBATE_Msk* = (0x00000001 shl ADC_CCR_VBATE_Pos) ## !< 0x00400000
  ADC_CCR_VBATE* = ADC_CCR_VBATE_Msk
  ADC_CCR_TSVREFE_Pos* = (23)
  ADC_CCR_TSVREFE_Msk* = (0x00000001 shl ADC_CCR_TSVREFE_Pos) ## !< 0x00800000
  ADC_CCR_TSVREFE* = ADC_CCR_TSVREFE_Msk

## ******************  Bit definition for ADC_CDR register  *******************

const
  ADC_CDR_DATA1_Pos* = (0)
  ADC_CDR_DATA1_Msk* = (0x0000FFFF shl ADC_CDR_DATA1_Pos) ## !< 0x0000FFFF
  ADC_CDR_DATA1* = ADC_CDR_DATA1_Msk
  ADC_CDR_DATA2_Pos* = (16)
  ADC_CDR_DATA2_Msk* = (0x0000FFFF shl ADC_CDR_DATA2_Pos) ## !< 0xFFFF0000
  ADC_CDR_DATA2* = ADC_CDR_DATA2_Msk

##  Legacy defines

const
  ADC_CDR_RDATA_MST* = ADC_CDR_DATA1
  ADC_CDR_RDATA_SLV* = ADC_CDR_DATA2

## ****************************************************************************
##
##                          Controller Area Network
##
## ****************************************************************************
## !<CAN control and status registers
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
  CAN_MCR_DBF_Pos* = (16)
  CAN_MCR_DBF_Msk* = (0x00000001 shl CAN_MCR_DBF_Pos) ## !< 0x00010000
  CAN_MCR_DBF* = CAN_MCR_DBF_Msk

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
#CAN_IER_EWGIE_Pos* = (8)

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
  CAN_FMR_CAN2SB_Pos* = (8)
  CAN_FMR_CAN2SB_Msk* = (0x0000003F shl CAN_FMR_CAN2SB_Pos) ## !< 0x00003F00
  CAN_FMR_CAN2SB* = CAN_FMR_CAN2SB_Msk

## ******************  Bit definition for CAN_FM1R register  ******************

const
  CAN_FM1R_FBM_Pos* = (0)
  CAN_FM1R_FBM_Msk* = (0x0FFFFFFF shl CAN_FM1R_FBM_Pos) ## !< 0x0FFFFFFF
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
  CAN_FM1R_FBM14_Pos* = (14)
  CAN_FM1R_FBM14_Msk* = (0x00000001 shl CAN_FM1R_FBM14_Pos) ## !< 0x00004000
  CAN_FM1R_FBM14* = CAN_FM1R_FBM14_Msk
  CAN_FM1R_FBM15_Pos* = (15)
  CAN_FM1R_FBM15_Msk* = (0x00000001 shl CAN_FM1R_FBM15_Pos) ## !< 0x00008000
  CAN_FM1R_FBM15* = CAN_FM1R_FBM15_Msk
  CAN_FM1R_FBM16_Pos* = (16)
  CAN_FM1R_FBM16_Msk* = (0x00000001 shl CAN_FM1R_FBM16_Pos) ## !< 0x00010000
  CAN_FM1R_FBM16* = CAN_FM1R_FBM16_Msk
  CAN_FM1R_FBM17_Pos* = (17)
  CAN_FM1R_FBM17_Msk* = (0x00000001 shl CAN_FM1R_FBM17_Pos) ## !< 0x00020000
  CAN_FM1R_FBM17* = CAN_FM1R_FBM17_Msk
  CAN_FM1R_FBM18_Pos* = (18)
  CAN_FM1R_FBM18_Msk* = (0x00000001 shl CAN_FM1R_FBM18_Pos) ## !< 0x00040000
  CAN_FM1R_FBM18* = CAN_FM1R_FBM18_Msk
  CAN_FM1R_FBM19_Pos* = (19)
  CAN_FM1R_FBM19_Msk* = (0x00000001 shl CAN_FM1R_FBM19_Pos) ## !< 0x00080000
  CAN_FM1R_FBM19* = CAN_FM1R_FBM19_Msk
  CAN_FM1R_FBM20_Pos* = (20)
  CAN_FM1R_FBM20_Msk* = (0x00000001 shl CAN_FM1R_FBM20_Pos) ## !< 0x00100000
  CAN_FM1R_FBM20* = CAN_FM1R_FBM20_Msk
  CAN_FM1R_FBM21_Pos* = (21)
  CAN_FM1R_FBM21_Msk* = (0x00000001 shl CAN_FM1R_FBM21_Pos) ## !< 0x00200000
  CAN_FM1R_FBM21* = CAN_FM1R_FBM21_Msk
  CAN_FM1R_FBM22_Pos* = (22)
  CAN_FM1R_FBM22_Msk* = (0x00000001 shl CAN_FM1R_FBM22_Pos) ## !< 0x00400000
  CAN_FM1R_FBM22* = CAN_FM1R_FBM22_Msk
  CAN_FM1R_FBM23_Pos* = (23)
  CAN_FM1R_FBM23_Msk* = (0x00000001 shl CAN_FM1R_FBM23_Pos) ## !< 0x00800000
  CAN_FM1R_FBM23* = CAN_FM1R_FBM23_Msk
  CAN_FM1R_FBM24_Pos* = (24)
  CAN_FM1R_FBM24_Msk* = (0x00000001 shl CAN_FM1R_FBM24_Pos) ## !< 0x01000000
  CAN_FM1R_FBM24* = CAN_FM1R_FBM24_Msk
  CAN_FM1R_FBM25_Pos* = (25)
  CAN_FM1R_FBM25_Msk* = (0x00000001 shl CAN_FM1R_FBM25_Pos) ## !< 0x02000000
  CAN_FM1R_FBM25* = CAN_FM1R_FBM25_Msk
  CAN_FM1R_FBM26_Pos* = (26)
  CAN_FM1R_FBM26_Msk* = (0x00000001 shl CAN_FM1R_FBM26_Pos) ## !< 0x04000000
  CAN_FM1R_FBM26* = CAN_FM1R_FBM26_Msk
  CAN_FM1R_FBM27_Pos* = (27)
  CAN_FM1R_FBM27_Msk* = (0x00000001 shl CAN_FM1R_FBM27_Pos) ## !< 0x08000000
  CAN_FM1R_FBM27* = CAN_FM1R_FBM27_Msk

## ******************  Bit definition for CAN_FS1R register  ******************

const
  CAN_FS1R_FSC_Pos* = (0)
  CAN_FS1R_FSC_Msk* = (0x0FFFFFFF shl CAN_FS1R_FSC_Pos) ## !< 0x0FFFFFFF
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
  CAN_FS1R_FSC14_Pos* = (14)
  CAN_FS1R_FSC14_Msk* = (0x00000001 shl CAN_FS1R_FSC14_Pos) ## !< 0x00004000
  CAN_FS1R_FSC14* = CAN_FS1R_FSC14_Msk
  CAN_FS1R_FSC15_Pos* = (15)
  CAN_FS1R_FSC15_Msk* = (0x00000001 shl CAN_FS1R_FSC15_Pos) ## !< 0x00008000
  CAN_FS1R_FSC15* = CAN_FS1R_FSC15_Msk
  CAN_FS1R_FSC16_Pos* = (16)
  CAN_FS1R_FSC16_Msk* = (0x00000001 shl CAN_FS1R_FSC16_Pos) ## !< 0x00010000
  CAN_FS1R_FSC16* = CAN_FS1R_FSC16_Msk
  CAN_FS1R_FSC17_Pos* = (17)
  CAN_FS1R_FSC17_Msk* = (0x00000001 shl CAN_FS1R_FSC17_Pos) ## !< 0x00020000
  CAN_FS1R_FSC17* = CAN_FS1R_FSC17_Msk
  CAN_FS1R_FSC18_Pos* = (18)
  CAN_FS1R_FSC18_Msk* = (0x00000001 shl CAN_FS1R_FSC18_Pos) ## !< 0x00040000
  CAN_FS1R_FSC18* = CAN_FS1R_FSC18_Msk
  CAN_FS1R_FSC19_Pos* = (19)
  CAN_FS1R_FSC19_Msk* = (0x00000001 shl CAN_FS1R_FSC19_Pos) ## !< 0x00080000
  CAN_FS1R_FSC19* = CAN_FS1R_FSC19_Msk
  CAN_FS1R_FSC20_Pos* = (20)
  CAN_FS1R_FSC20_Msk* = (0x00000001 shl CAN_FS1R_FSC20_Pos) ## !< 0x00100000
  CAN_FS1R_FSC20* = CAN_FS1R_FSC20_Msk
  CAN_FS1R_FSC21_Pos* = (21)
  CAN_FS1R_FSC21_Msk* = (0x00000001 shl CAN_FS1R_FSC21_Pos) ## !< 0x00200000
  CAN_FS1R_FSC21* = CAN_FS1R_FSC21_Msk
  CAN_FS1R_FSC22_Pos* = (22)
  CAN_FS1R_FSC22_Msk* = (0x00000001 shl CAN_FS1R_FSC22_Pos) ## !< 0x00400000
  CAN_FS1R_FSC22* = CAN_FS1R_FSC22_Msk
  CAN_FS1R_FSC23_Pos* = (23)
  CAN_FS1R_FSC23_Msk* = (0x00000001 shl CAN_FS1R_FSC23_Pos) ## !< 0x00800000
  CAN_FS1R_FSC23* = CAN_FS1R_FSC23_Msk
  CAN_FS1R_FSC24_Pos* = (24)
  CAN_FS1R_FSC24_Msk* = (0x00000001 shl CAN_FS1R_FSC24_Pos) ## !< 0x01000000
  CAN_FS1R_FSC24* = CAN_FS1R_FSC24_Msk
  CAN_FS1R_FSC25_Pos* = (25)
  CAN_FS1R_FSC25_Msk* = (0x00000001 shl CAN_FS1R_FSC25_Pos) ## !< 0x02000000
  CAN_FS1R_FSC25* = CAN_FS1R_FSC25_Msk
  CAN_FS1R_FSC26_Pos* = (26)
  CAN_FS1R_FSC26_Msk* = (0x00000001 shl CAN_FS1R_FSC26_Pos) ## !< 0x04000000
  CAN_FS1R_FSC26* = CAN_FS1R_FSC26_Msk
  CAN_FS1R_FSC27_Pos* = (27)
  CAN_FS1R_FSC27_Msk* = (0x00000001 shl CAN_FS1R_FSC27_Pos) ## !< 0x08000000
  CAN_FS1R_FSC27* = CAN_FS1R_FSC27_Msk

## *****************  Bit definition for CAN_FFA1R register  ******************

const
  CAN_FFA1R_FFA_Pos* = (0)
  CAN_FFA1R_FFA_Msk* = (0x0FFFFFFF shl CAN_FFA1R_FFA_Pos) ## !< 0x0FFFFFFF
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
  CAN_FFA1R_FFA14_Pos* = (14)
  CAN_FFA1R_FFA14_Msk* = (0x00000001 shl CAN_FFA1R_FFA14_Pos) ## !< 0x00004000
  CAN_FFA1R_FFA14* = CAN_FFA1R_FFA14_Msk
  CAN_FFA1R_FFA15_Pos* = (15)
  CAN_FFA1R_FFA15_Msk* = (0x00000001 shl CAN_FFA1R_FFA15_Pos) ## !< 0x00008000
  CAN_FFA1R_FFA15* = CAN_FFA1R_FFA15_Msk
  CAN_FFA1R_FFA16_Pos* = (16)
  CAN_FFA1R_FFA16_Msk* = (0x00000001 shl CAN_FFA1R_FFA16_Pos) ## !< 0x00010000
  CAN_FFA1R_FFA16* = CAN_FFA1R_FFA16_Msk
  CAN_FFA1R_FFA17_Pos* = (17)
  CAN_FFA1R_FFA17_Msk* = (0x00000001 shl CAN_FFA1R_FFA17_Pos) ## !< 0x00020000
  CAN_FFA1R_FFA17* = CAN_FFA1R_FFA17_Msk
  CAN_FFA1R_FFA18_Pos* = (18)
  CAN_FFA1R_FFA18_Msk* = (0x00000001 shl CAN_FFA1R_FFA18_Pos) ## !< 0x00040000
  CAN_FFA1R_FFA18* = CAN_FFA1R_FFA18_Msk
  CAN_FFA1R_FFA19_Pos* = (19)
  CAN_FFA1R_FFA19_Msk* = (0x00000001 shl CAN_FFA1R_FFA19_Pos) ## !< 0x00080000
  CAN_FFA1R_FFA19* = CAN_FFA1R_FFA19_Msk
  CAN_FFA1R_FFA20_Pos* = (20)
  CAN_FFA1R_FFA20_Msk* = (0x00000001 shl CAN_FFA1R_FFA20_Pos) ## !< 0x00100000
  CAN_FFA1R_FFA20* = CAN_FFA1R_FFA20_Msk
  CAN_FFA1R_FFA21_Pos* = (21)
  CAN_FFA1R_FFA21_Msk* = (0x00000001 shl CAN_FFA1R_FFA21_Pos) ## !< 0x00200000
  CAN_FFA1R_FFA21* = CAN_FFA1R_FFA21_Msk
  CAN_FFA1R_FFA22_Pos* = (22)
  CAN_FFA1R_FFA22_Msk* = (0x00000001 shl CAN_FFA1R_FFA22_Pos) ## !< 0x00400000
  CAN_FFA1R_FFA22* = CAN_FFA1R_FFA22_Msk
  CAN_FFA1R_FFA23_Pos* = (23)
  CAN_FFA1R_FFA23_Msk* = (0x00000001 shl CAN_FFA1R_FFA23_Pos) ## !< 0x00800000
  CAN_FFA1R_FFA23* = CAN_FFA1R_FFA23_Msk
  CAN_FFA1R_FFA24_Pos* = (24)
  CAN_FFA1R_FFA24_Msk* = (0x00000001 shl CAN_FFA1R_FFA24_Pos) ## !< 0x01000000
  CAN_FFA1R_FFA24* = CAN_FFA1R_FFA24_Msk
  CAN_FFA1R_FFA25_Pos* = (25)
  CAN_FFA1R_FFA25_Msk* = (0x00000001 shl CAN_FFA1R_FFA25_Pos) ## !< 0x02000000
  CAN_FFA1R_FFA25* = CAN_FFA1R_FFA25_Msk
  CAN_FFA1R_FFA26_Pos* = (26)
  CAN_FFA1R_FFA26_Msk* = (0x00000001 shl CAN_FFA1R_FFA26_Pos) ## !< 0x04000000
  CAN_FFA1R_FFA26* = CAN_FFA1R_FFA26_Msk
  CAN_FFA1R_FFA27_Pos* = (27)
  CAN_FFA1R_FFA27_Msk* = (0x00000001 shl CAN_FFA1R_FFA27_Pos) ## !< 0x08000000
  CAN_FFA1R_FFA27* = CAN_FFA1R_FFA27_Msk

## ******************  Bit definition for CAN_FA1R register  ******************

const
  CAN_FA1R_FACT_Pos* = (0)
  CAN_FA1R_FACT_Msk* = (0x0FFFFFFF shl CAN_FA1R_FACT_Pos) ## !< 0x0FFFFFFF
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
  CAN_FA1R_FACT14_Pos* = (14)
  CAN_FA1R_FACT14_Msk* = (0x00000001 shl CAN_FA1R_FACT14_Pos) ## !< 0x00004000
  CAN_FA1R_FACT14* = CAN_FA1R_FACT14_Msk
  CAN_FA1R_FACT15_Pos* = (15)
  CAN_FA1R_FACT15_Msk* = (0x00000001 shl CAN_FA1R_FACT15_Pos) ## !< 0x00008000
  CAN_FA1R_FACT15* = CAN_FA1R_FACT15_Msk
  CAN_FA1R_FACT16_Pos* = (16)
  CAN_FA1R_FACT16_Msk* = (0x00000001 shl CAN_FA1R_FACT16_Pos) ## !< 0x00010000
  CAN_FA1R_FACT16* = CAN_FA1R_FACT16_Msk
  CAN_FA1R_FACT17_Pos* = (17)
  CAN_FA1R_FACT17_Msk* = (0x00000001 shl CAN_FA1R_FACT17_Pos) ## !< 0x00020000
  CAN_FA1R_FACT17* = CAN_FA1R_FACT17_Msk
  CAN_FA1R_FACT18_Pos* = (18)
  CAN_FA1R_FACT18_Msk* = (0x00000001 shl CAN_FA1R_FACT18_Pos) ## !< 0x00040000
  CAN_FA1R_FACT18* = CAN_FA1R_FACT18_Msk
  CAN_FA1R_FACT19_Pos* = (19)
  CAN_FA1R_FACT19_Msk* = (0x00000001 shl CAN_FA1R_FACT19_Pos) ## !< 0x00080000
  CAN_FA1R_FACT19* = CAN_FA1R_FACT19_Msk
  CAN_FA1R_FACT20_Pos* = (20)
  CAN_FA1R_FACT20_Msk* = (0x00000001 shl CAN_FA1R_FACT20_Pos) ## !< 0x00100000
  CAN_FA1R_FACT20* = CAN_FA1R_FACT20_Msk
  CAN_FA1R_FACT21_Pos* = (21)
  CAN_FA1R_FACT21_Msk* = (0x00000001 shl CAN_FA1R_FACT21_Pos) ## !< 0x00200000
  CAN_FA1R_FACT21* = CAN_FA1R_FACT21_Msk
  CAN_FA1R_FACT22_Pos* = (22)
  CAN_FA1R_FACT22_Msk* = (0x00000001 shl CAN_FA1R_FACT22_Pos) ## !< 0x00400000
  CAN_FA1R_FACT22* = CAN_FA1R_FACT22_Msk
  CAN_FA1R_FACT23_Pos* = (23)
  CAN_FA1R_FACT23_Msk* = (0x00000001 shl CAN_FA1R_FACT23_Pos) ## !< 0x00800000
  CAN_FA1R_FACT23* = CAN_FA1R_FACT23_Msk
  CAN_FA1R_FACT24_Pos* = (24)
  CAN_FA1R_FACT24_Msk* = (0x00000001 shl CAN_FA1R_FACT24_Pos) ## !< 0x01000000
  CAN_FA1R_FACT24* = CAN_FA1R_FACT24_Msk
  CAN_FA1R_FACT25_Pos* = (25)
  CAN_FA1R_FACT25_Msk* = (0x00000001 shl CAN_FA1R_FACT25_Pos) ## !< 0x02000000
  CAN_FA1R_FACT25* = CAN_FA1R_FACT25_Msk
  CAN_FA1R_FACT26_Pos* = (26)
  CAN_FA1R_FACT26_Msk* = (0x00000001 shl CAN_FA1R_FACT26_Pos) ## !< 0x04000000
  CAN_FA1R_FACT26* = CAN_FA1R_FACT26_Msk
  CAN_FA1R_FACT27_Pos* = (27)
  CAN_FA1R_FACT27_Msk* = (0x00000001 shl CAN_FA1R_FACT27_Pos) ## !< 0x08000000
  CAN_FA1R_FACT27* = CAN_FA1R_FACT27_Msk

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
##                           CRC calculation unit
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
##                       Digital to Analog Converter
##
## ****************************************************************************
##
##  @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
##

const
  DAC_CHANNEL2_SUPPORT* = true  ## !< DAC feature available only on specific devices: availability of DAC channel 2

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
##                                     DCMI
##
## ****************************************************************************
## *******************  Bits definition for DCMI_CR register  *****************

const
  DCMI_CR_CAPTURE_Pos* = (0)
  DCMI_CR_CAPTURE_Msk* = (0x00000001 shl DCMI_CR_CAPTURE_Pos) ## !< 0x00000001
  DCMI_CR_CAPTURE* = DCMI_CR_CAPTURE_Msk
  DCMI_CR_CM_Pos* = (1)
  DCMI_CR_CM_Msk* = (0x00000001 shl DCMI_CR_CM_Pos) ## !< 0x00000002
  DCMI_CR_CM* = DCMI_CR_CM_Msk
  DCMI_CR_CROP_Pos* = (2)
  DCMI_CR_CROP_Msk* = (0x00000001 shl DCMI_CR_CROP_Pos) ## !< 0x00000004
  DCMI_CR_CROP* = DCMI_CR_CROP_Msk
  DCMI_CR_JPEG_Pos* = (3)
  DCMI_CR_JPEG_Msk* = (0x00000001 shl DCMI_CR_JPEG_Pos) ## !< 0x00000008
  DCMI_CR_JPEG* = DCMI_CR_JPEG_Msk
  DCMI_CR_ESS_Pos* = (4)
  DCMI_CR_ESS_Msk* = (0x00000001 shl DCMI_CR_ESS_Pos) ## !< 0x00000010
  DCMI_CR_ESS* = DCMI_CR_ESS_Msk
  DCMI_CR_PCKPOL_Pos* = (5)
  DCMI_CR_PCKPOL_Msk* = (0x00000001 shl DCMI_CR_PCKPOL_Pos) ## !< 0x00000020
  DCMI_CR_PCKPOL* = DCMI_CR_PCKPOL_Msk
  DCMI_CR_HSPOL_Pos* = (6)
  DCMI_CR_HSPOL_Msk* = (0x00000001 shl DCMI_CR_HSPOL_Pos) ## !< 0x00000040
  DCMI_CR_HSPOL* = DCMI_CR_HSPOL_Msk
  DCMI_CR_VSPOL_Pos* = (7)
  DCMI_CR_VSPOL_Msk* = (0x00000001 shl DCMI_CR_VSPOL_Pos) ## !< 0x00000080
  DCMI_CR_VSPOL* = DCMI_CR_VSPOL_Msk
  DCMI_CR_FCRC_0* = 0x00000100
  DCMI_CR_FCRC_1* = 0x00000200
  DCMI_CR_EDM_0* = 0x00000400
  DCMI_CR_EDM_1* = 0x00000800
  DCMI_CR_CRE_Pos* = (12)
  DCMI_CR_CRE_Msk* = (0x00000001 shl DCMI_CR_CRE_Pos) ## !< 0x00001000
  DCMI_CR_CRE* = DCMI_CR_CRE_Msk
  DCMI_CR_ENABLE_Pos* = (14)
  DCMI_CR_ENABLE_Msk* = (0x00000001 shl DCMI_CR_ENABLE_Pos) ## !< 0x00004000
  DCMI_CR_ENABLE* = DCMI_CR_ENABLE_Msk

## *******************  Bits definition for DCMI_SR register  *****************

const
  DCMI_SR_HSYNC_Pos* = (0)
  DCMI_SR_HSYNC_Msk* = (0x00000001 shl DCMI_SR_HSYNC_Pos) ## !< 0x00000001
  DCMI_SR_HSYNC* = DCMI_SR_HSYNC_Msk
  DCMI_SR_VSYNC_Pos* = (1)
  DCMI_SR_VSYNC_Msk* = (0x00000001 shl DCMI_SR_VSYNC_Pos) ## !< 0x00000002
  DCMI_SR_VSYNC* = DCMI_SR_VSYNC_Msk
  DCMI_SR_FNE_Pos* = (2)
  DCMI_SR_FNE_Msk* = (0x00000001 shl DCMI_SR_FNE_Pos) ## !< 0x00000004
  DCMI_SR_FNE* = DCMI_SR_FNE_Msk

## *******************  Bits definition for DCMI_RIS register  ****************

const
  DCMI_RIS_FRAME_RIS_Pos* = (0)
  DCMI_RIS_FRAME_RIS_Msk* = (0x00000001 shl DCMI_RIS_FRAME_RIS_Pos) ## !< 0x00000001
  DCMI_RIS_FRAME_RIS* = DCMI_RIS_FRAME_RIS_Msk
  DCMI_RIS_OVR_RIS_Pos* = (1)
  DCMI_RIS_OVR_RIS_Msk* = (0x00000001 shl DCMI_RIS_OVR_RIS_Pos) ## !< 0x00000002
  DCMI_RIS_OVR_RIS* = DCMI_RIS_OVR_RIS_Msk
  DCMI_RIS_ERR_RIS_Pos* = (2)
  DCMI_RIS_ERR_RIS_Msk* = (0x00000001 shl DCMI_RIS_ERR_RIS_Pos) ## !< 0x00000004
  DCMI_RIS_ERR_RIS* = DCMI_RIS_ERR_RIS_Msk
  DCMI_RIS_VSYNC_RIS_Pos* = (3)
  DCMI_RIS_VSYNC_RIS_Msk* = (0x00000001 shl DCMI_RIS_VSYNC_RIS_Pos) ## !< 0x00000008
  DCMI_RIS_VSYNC_RIS* = DCMI_RIS_VSYNC_RIS_Msk
  DCMI_RIS_LINE_RIS_Pos* = (4)
  DCMI_RIS_LINE_RIS_Msk* = (0x00000001 shl DCMI_RIS_LINE_RIS_Pos) ## !< 0x00000010
  DCMI_RIS_LINE_RIS* = DCMI_RIS_LINE_RIS_Msk

##  Legacy defines

const
  DCMI_RISR_FRAME_RIS* = DCMI_RIS_FRAME_RIS
  DCMI_RISR_OVR_RIS* = DCMI_RIS_OVR_RIS
  DCMI_RISR_ERR_RIS* = DCMI_RIS_ERR_RIS
  DCMI_RISR_VSYNC_RIS* = DCMI_RIS_VSYNC_RIS
  DCMI_RISR_LINE_RIS* = DCMI_RIS_LINE_RIS
  DCMI_RISR_OVF_RIS* = DCMI_RIS_OVR_RIS

## *******************  Bits definition for DCMI_IER register  ****************

const
  DCMI_IER_FRAME_IE_Pos* = (0)
  DCMI_IER_FRAME_IE_Msk* = (0x00000001 shl DCMI_IER_FRAME_IE_Pos) ## !< 0x00000001
  DCMI_IER_FRAME_IE* = DCMI_IER_FRAME_IE_Msk
  DCMI_IER_OVR_IE_Pos* = (1)
  DCMI_IER_OVR_IE_Msk* = (0x00000001 shl DCMI_IER_OVR_IE_Pos) ## !< 0x00000002
  DCMI_IER_OVR_IE* = DCMI_IER_OVR_IE_Msk
  DCMI_IER_ERR_IE_Pos* = (2)
  DCMI_IER_ERR_IE_Msk* = (0x00000001 shl DCMI_IER_ERR_IE_Pos) ## !< 0x00000004
  DCMI_IER_ERR_IE* = DCMI_IER_ERR_IE_Msk
  DCMI_IER_VSYNC_IE_Pos* = (3)
  DCMI_IER_VSYNC_IE_Msk* = (0x00000001 shl DCMI_IER_VSYNC_IE_Pos) ## !< 0x00000008
  DCMI_IER_VSYNC_IE* = DCMI_IER_VSYNC_IE_Msk
  DCMI_IER_LINE_IE_Pos* = (4)
  DCMI_IER_LINE_IE_Msk* = (0x00000001 shl DCMI_IER_LINE_IE_Pos) ## !< 0x00000010
  DCMI_IER_LINE_IE* = DCMI_IER_LINE_IE_Msk

##  Legacy defines

const
  DCMI_IER_OVF_IE* = DCMI_IER_OVR_IE

## *******************  Bits definition for DCMI_MIS register  ****************

const
  DCMI_MIS_FRAME_MIS_Pos* = (0)
  DCMI_MIS_FRAME_MIS_Msk* = (0x00000001 shl DCMI_MIS_FRAME_MIS_Pos) ## !< 0x00000001
  DCMI_MIS_FRAME_MIS* = DCMI_MIS_FRAME_MIS_Msk
  DCMI_MIS_OVR_MIS_Pos* = (1)
  DCMI_MIS_OVR_MIS_Msk* = (0x00000001 shl DCMI_MIS_OVR_MIS_Pos) ## !< 0x00000002
  DCMI_MIS_OVR_MIS* = DCMI_MIS_OVR_MIS_Msk
  DCMI_MIS_ERR_MIS_Pos* = (2)
  DCMI_MIS_ERR_MIS_Msk* = (0x00000001 shl DCMI_MIS_ERR_MIS_Pos) ## !< 0x00000004
  DCMI_MIS_ERR_MIS* = DCMI_MIS_ERR_MIS_Msk
  DCMI_MIS_VSYNC_MIS_Pos* = (3)
  DCMI_MIS_VSYNC_MIS_Msk* = (0x00000001 shl DCMI_MIS_VSYNC_MIS_Pos) ## !< 0x00000008
  DCMI_MIS_VSYNC_MIS* = DCMI_MIS_VSYNC_MIS_Msk
  DCMI_MIS_LINE_MIS_Pos* = (4)
  DCMI_MIS_LINE_MIS_Msk* = (0x00000001 shl DCMI_MIS_LINE_MIS_Pos) ## !< 0x00000010
  DCMI_MIS_LINE_MIS* = DCMI_MIS_LINE_MIS_Msk

##  Legacy defines

const
  DCMI_MISR_FRAME_MIS* = DCMI_MIS_FRAME_MIS
  DCMI_MISR_OVF_MIS* = DCMI_MIS_OVR_MIS
  DCMI_MISR_ERR_MIS* = DCMI_MIS_ERR_MIS
  DCMI_MISR_VSYNC_MIS* = DCMI_MIS_VSYNC_MIS
  DCMI_MISR_LINE_MIS* = DCMI_MIS_LINE_MIS

## *******************  Bits definition for DCMI_ICR register  ****************

const
  DCMI_ICR_FRAME_ISC_Pos* = (0)
  DCMI_ICR_FRAME_ISC_Msk* = (0x00000001 shl DCMI_ICR_FRAME_ISC_Pos) ## !< 0x00000001
  DCMI_ICR_FRAME_ISC* = DCMI_ICR_FRAME_ISC_Msk
  DCMI_ICR_OVR_ISC_Pos* = (1)
  DCMI_ICR_OVR_ISC_Msk* = (0x00000001 shl DCMI_ICR_OVR_ISC_Pos) ## !< 0x00000002
  DCMI_ICR_OVR_ISC* = DCMI_ICR_OVR_ISC_Msk
  DCMI_ICR_ERR_ISC_Pos* = (2)
  DCMI_ICR_ERR_ISC_Msk* = (0x00000001 shl DCMI_ICR_ERR_ISC_Pos) ## !< 0x00000004
  DCMI_ICR_ERR_ISC* = DCMI_ICR_ERR_ISC_Msk
  DCMI_ICR_VSYNC_ISC_Pos* = (3)
  DCMI_ICR_VSYNC_ISC_Msk* = (0x00000001 shl DCMI_ICR_VSYNC_ISC_Pos) ## !< 0x00000008
  DCMI_ICR_VSYNC_ISC* = DCMI_ICR_VSYNC_ISC_Msk
  DCMI_ICR_LINE_ISC_Pos* = (4)
  DCMI_ICR_LINE_ISC_Msk* = (0x00000001 shl DCMI_ICR_LINE_ISC_Pos) ## !< 0x00000010
  DCMI_ICR_LINE_ISC* = DCMI_ICR_LINE_ISC_Msk

##  Legacy defines

const
  DCMI_ICR_OVF_ISC* = DCMI_ICR_OVR_ISC

## *******************  Bits definition for DCMI_ESCR register  *****************

const
  DCMI_ESCR_FSC_Pos* = (0)
  DCMI_ESCR_FSC_Msk* = (0x000000FF shl DCMI_ESCR_FSC_Pos) ## !< 0x000000FF
  DCMI_ESCR_FSC* = DCMI_ESCR_FSC_Msk
  DCMI_ESCR_LSC_Pos* = (8)
  DCMI_ESCR_LSC_Msk* = (0x000000FF shl DCMI_ESCR_LSC_Pos) ## !< 0x0000FF00
  DCMI_ESCR_LSC* = DCMI_ESCR_LSC_Msk
  DCMI_ESCR_LEC_Pos* = (16)
  DCMI_ESCR_LEC_Msk* = (0x000000FF shl DCMI_ESCR_LEC_Pos) ## !< 0x00FF0000
  DCMI_ESCR_LEC* = DCMI_ESCR_LEC_Msk
  DCMI_ESCR_FEC_Pos* = (24)
  DCMI_ESCR_FEC_Msk* = (0x000000FF shl DCMI_ESCR_FEC_Pos) ## !< 0xFF000000
  DCMI_ESCR_FEC* = DCMI_ESCR_FEC_Msk

## *******************  Bits definition for DCMI_ESUR register  *****************

const
  DCMI_ESUR_FSU_Pos* = (0)
  DCMI_ESUR_FSU_Msk* = (0x000000FF shl DCMI_ESUR_FSU_Pos) ## !< 0x000000FF
  DCMI_ESUR_FSU* = DCMI_ESUR_FSU_Msk
  DCMI_ESUR_LSU_Pos* = (8)
  DCMI_ESUR_LSU_Msk* = (0x000000FF shl DCMI_ESUR_LSU_Pos) ## !< 0x0000FF00
  DCMI_ESUR_LSU* = DCMI_ESUR_LSU_Msk
  DCMI_ESUR_LEU_Pos* = (16)
  DCMI_ESUR_LEU_Msk* = (0x000000FF shl DCMI_ESUR_LEU_Pos) ## !< 0x00FF0000
  DCMI_ESUR_LEU* = DCMI_ESUR_LEU_Msk
  DCMI_ESUR_FEU_Pos* = (24)
  DCMI_ESUR_FEU_Msk* = (0x000000FF shl DCMI_ESUR_FEU_Pos) ## !< 0xFF000000
  DCMI_ESUR_FEU* = DCMI_ESUR_FEU_Msk

## *******************  Bits definition for DCMI_CWSTRT register  *****************

const
  DCMI_CWSTRT_HOFFCNT_Pos* = (0)
  DCMI_CWSTRT_HOFFCNT_Msk* = (0x00003FFF shl DCMI_CWSTRT_HOFFCNT_Pos) ## !< 0x00003FFF
  DCMI_CWSTRT_HOFFCNT* = DCMI_CWSTRT_HOFFCNT_Msk
  DCMI_CWSTRT_VST_Pos* = (16)
  DCMI_CWSTRT_VST_Msk* = (0x00001FFF shl DCMI_CWSTRT_VST_Pos) ## !< 0x1FFF0000
  DCMI_CWSTRT_VST* = DCMI_CWSTRT_VST_Msk

## *******************  Bits definition for DCMI_CWSIZE register  *****************

const
  DCMI_CWSIZE_CAPCNT_Pos* = (0)
  DCMI_CWSIZE_CAPCNT_Msk* = (0x00003FFF shl DCMI_CWSIZE_CAPCNT_Pos) ## !< 0x00003FFF
  DCMI_CWSIZE_CAPCNT* = DCMI_CWSIZE_CAPCNT_Msk
  DCMI_CWSIZE_VLINE_Pos* = (16)
  DCMI_CWSIZE_VLINE_Msk* = (0x00003FFF shl DCMI_CWSIZE_VLINE_Pos) ## !< 0x3FFF0000
  DCMI_CWSIZE_VLINE* = DCMI_CWSIZE_VLINE_Msk

## *******************  Bits definition for DCMI_DR register  ********************

const
  DCMI_DR_BYTE0_Pos* = (0)
  DCMI_DR_BYTE0_Msk* = (0x000000FF shl DCMI_DR_BYTE0_Pos) ## !< 0x000000FF
  DCMI_DR_BYTE0* = DCMI_DR_BYTE0_Msk
  DCMI_DR_BYTE1_Pos* = (8)
  DCMI_DR_BYTE1_Msk* = (0x000000FF shl DCMI_DR_BYTE1_Pos) ## !< 0x0000FF00
  DCMI_DR_BYTE1* = DCMI_DR_BYTE1_Msk
  DCMI_DR_BYTE2_Pos* = (16)
  DCMI_DR_BYTE2_Msk* = (0x000000FF shl DCMI_DR_BYTE2_Pos) ## !< 0x00FF0000
  DCMI_DR_BYTE2* = DCMI_DR_BYTE2_Msk
  DCMI_DR_BYTE3_Pos* = (24)
  DCMI_DR_BYTE3_Msk* = (0x000000FF shl DCMI_DR_BYTE3_Pos) ## !< 0xFF000000
  DCMI_DR_BYTE3* = DCMI_DR_BYTE3_Msk

## ****************************************************************************
##
##                              DMA Controller
##
## ****************************************************************************
## *******************  Bits definition for DMA_SxCR register  ****************

const
  DMA_SxCR_CHSEL_Pos* = (25)
  DMA_SxCR_CHSEL_Msk* = (0x00000007 shl DMA_SxCR_CHSEL_Pos) ## !< 0x0E000000
  DMA_SxCR_CHSEL* = DMA_SxCR_CHSEL_Msk
  DMA_SxCR_CHSEL_0* = 0x02000000
  DMA_SxCR_CHSEL_1* = 0x04000000
  DMA_SxCR_CHSEL_2* = 0x08000000
  DMA_SxCR_MBURST_Pos* = (23)
  DMA_SxCR_MBURST_Msk* = (0x00000003 shl DMA_SxCR_MBURST_Pos) ## !< 0x01800000
  DMA_SxCR_MBURST* = DMA_SxCR_MBURST_Msk
  DMA_SxCR_MBURST_0* = (0x00000001 shl DMA_SxCR_MBURST_Pos) ## !< 0x00800000
  DMA_SxCR_MBURST_1* = (0x00000002 shl DMA_SxCR_MBURST_Pos) ## !< 0x01000000
  DMA_SxCR_PBURST_Pos* = (21)
  DMA_SxCR_PBURST_Msk* = (0x00000003 shl DMA_SxCR_PBURST_Pos) ## !< 0x00600000
  DMA_SxCR_PBURST* = DMA_SxCR_PBURST_Msk
  DMA_SxCR_PBURST_0* = (0x00000001 shl DMA_SxCR_PBURST_Pos) ## !< 0x00200000
  DMA_SxCR_PBURST_1* = (0x00000002 shl DMA_SxCR_PBURST_Pos) ## !< 0x00400000
  DMA_SxCR_CT_Pos* = (19)
  DMA_SxCR_CT_Msk* = (0x00000001 shl DMA_SxCR_CT_Pos) ## !< 0x00080000
  DMA_SxCR_CT* = DMA_SxCR_CT_Msk
  DMA_SxCR_DBM_Pos* = (18)
  DMA_SxCR_DBM_Msk* = (0x00000001 shl DMA_SxCR_DBM_Pos) ## !< 0x00040000
  DMA_SxCR_DBM* = DMA_SxCR_DBM_Msk
  DMA_SxCR_PL_Pos* = (16)
  DMA_SxCR_PL_Msk* = (0x00000003 shl DMA_SxCR_PL_Pos) ## !< 0x00030000
  DMA_SxCR_PL* = DMA_SxCR_PL_Msk
  DMA_SxCR_PL_0* = (0x00000001 shl DMA_SxCR_PL_Pos) ## !< 0x00010000
  DMA_SxCR_PL_1* = (0x00000002 shl DMA_SxCR_PL_Pos) ## !< 0x00020000
  DMA_SxCR_PINCOS_Pos* = (15)
  DMA_SxCR_PINCOS_Msk* = (0x00000001 shl DMA_SxCR_PINCOS_Pos) ## !< 0x00008000
  DMA_SxCR_PINCOS* = DMA_SxCR_PINCOS_Msk
  DMA_SxCR_MSIZE_Pos* = (13)
  DMA_SxCR_MSIZE_Msk* = (0x00000003 shl DMA_SxCR_MSIZE_Pos) ## !< 0x00006000
  DMA_SxCR_MSIZE* = DMA_SxCR_MSIZE_Msk
  DMA_SxCR_MSIZE_0* = (0x00000001 shl DMA_SxCR_MSIZE_Pos) ## !< 0x00002000
  DMA_SxCR_MSIZE_1* = (0x00000002 shl DMA_SxCR_MSIZE_Pos) ## !< 0x00004000
  DMA_SxCR_PSIZE_Pos* = (11)
  DMA_SxCR_PSIZE_Msk* = (0x00000003 shl DMA_SxCR_PSIZE_Pos) ## !< 0x00001800
  DMA_SxCR_PSIZE* = DMA_SxCR_PSIZE_Msk
  DMA_SxCR_PSIZE_0* = (0x00000001 shl DMA_SxCR_PSIZE_Pos) ## !< 0x00000800
  DMA_SxCR_PSIZE_1* = (0x00000002 shl DMA_SxCR_PSIZE_Pos) ## !< 0x00001000
  DMA_SxCR_MINC_Pos* = (10)
  DMA_SxCR_MINC_Msk* = (0x00000001 shl DMA_SxCR_MINC_Pos) ## !< 0x00000400
  DMA_SxCR_MINC* = DMA_SxCR_MINC_Msk
  DMA_SxCR_PINC_Pos* = (9)
  DMA_SxCR_PINC_Msk* = (0x00000001 shl DMA_SxCR_PINC_Pos) ## !< 0x00000200
  DMA_SxCR_PINC* = DMA_SxCR_PINC_Msk
  DMA_SxCR_CIRC_Pos* = (8)
  DMA_SxCR_CIRC_Msk* = (0x00000001 shl DMA_SxCR_CIRC_Pos) ## !< 0x00000100
  DMA_SxCR_CIRC* = DMA_SxCR_CIRC_Msk
  DMA_SxCR_DIR_Pos* = (6)
  DMA_SxCR_DIR_Msk* = (0x00000003 shl DMA_SxCR_DIR_Pos) ## !< 0x000000C0
  DMA_SxCR_DIR* = DMA_SxCR_DIR_Msk
  DMA_SxCR_DIR_0* = (0x00000001 shl DMA_SxCR_DIR_Pos) ## !< 0x00000040
  DMA_SxCR_DIR_1* = (0x00000002 shl DMA_SxCR_DIR_Pos) ## !< 0x00000080
  DMA_SxCR_PFCTRL_Pos* = (5)
  DMA_SxCR_PFCTRL_Msk* = (0x00000001 shl DMA_SxCR_PFCTRL_Pos) ## !< 0x00000020
  DMA_SxCR_PFCTRL* = DMA_SxCR_PFCTRL_Msk
  DMA_SxCR_TCIE_Pos* = (4)
  DMA_SxCR_TCIE_Msk* = (0x00000001 shl DMA_SxCR_TCIE_Pos) ## !< 0x00000010
  DMA_SxCR_TCIE* = DMA_SxCR_TCIE_Msk
  DMA_SxCR_HTIE_Pos* = (3)
  DMA_SxCR_HTIE_Msk* = (0x00000001 shl DMA_SxCR_HTIE_Pos) ## !< 0x00000008
  DMA_SxCR_HTIE* = DMA_SxCR_HTIE_Msk
  DMA_SxCR_TEIE_Pos* = (2)
  DMA_SxCR_TEIE_Msk* = (0x00000001 shl DMA_SxCR_TEIE_Pos) ## !< 0x00000004
  DMA_SxCR_TEIE* = DMA_SxCR_TEIE_Msk
  DMA_SxCR_DMEIE_Pos* = (1)
  DMA_SxCR_DMEIE_Msk* = (0x00000001 shl DMA_SxCR_DMEIE_Pos) ## !< 0x00000002
  DMA_SxCR_DMEIE* = DMA_SxCR_DMEIE_Msk
  DMA_SxCR_EN_Pos* = (0)
  DMA_SxCR_EN_Msk* = (0x00000001 shl DMA_SxCR_EN_Pos) ## !< 0x00000001
  DMA_SxCR_EN* = DMA_SxCR_EN_Msk

##  Legacy defines

const
  DMA_SxCR_ACK_Pos* = (20)
  DMA_SxCR_ACK_Msk* = (0x00000001 shl DMA_SxCR_ACK_Pos) ## !< 0x00100000
  DMA_SxCR_ACK* = DMA_SxCR_ACK_Msk

## *******************  Bits definition for DMA_SxCNDTR register  *************

const
  DMA_SxNDT_Pos* = (0)
  DMA_SxNDT_Msk* = (0x0000FFFF shl DMA_SxNDT_Pos) ## !< 0x0000FFFF
  DMA_SxNDT* = DMA_SxNDT_Msk
  DMA_SxNDT_0* = (0x00000001 shl DMA_SxNDT_Pos) ## !< 0x00000001
  DMA_SxNDT_1* = (0x00000002 shl DMA_SxNDT_Pos) ## !< 0x00000002
  DMA_SxNDT_2* = (0x00000004 shl DMA_SxNDT_Pos) ## !< 0x00000004
  DMA_SxNDT_3* = (0x00000008 shl DMA_SxNDT_Pos) ## !< 0x00000008
  DMA_SxNDT_4* = (0x00000010 shl DMA_SxNDT_Pos) ## !< 0x00000010
  DMA_SxNDT_5* = (0x00000020 shl DMA_SxNDT_Pos) ## !< 0x00000020
  DMA_SxNDT_6* = (0x00000040 shl DMA_SxNDT_Pos) ## !< 0x00000040
  DMA_SxNDT_7* = (0x00000080 shl DMA_SxNDT_Pos) ## !< 0x00000080
  DMA_SxNDT_8* = (0x00000100 shl DMA_SxNDT_Pos) ## !< 0x00000100
  DMA_SxNDT_9* = (0x00000200 shl DMA_SxNDT_Pos) ## !< 0x00000200
  DMA_SxNDT_10* = (0x00000400 shl DMA_SxNDT_Pos) ## !< 0x00000400
  DMA_SxNDT_11* = (0x00000800 shl DMA_SxNDT_Pos) ## !< 0x00000800
  DMA_SxNDT_12* = (0x00001000 shl DMA_SxNDT_Pos) ## !< 0x00001000
  DMA_SxNDT_13* = (0x00002000 shl DMA_SxNDT_Pos) ## !< 0x00002000
  DMA_SxNDT_14* = (0x00004000 shl DMA_SxNDT_Pos) ## !< 0x00004000
  DMA_SxNDT_15* = (0x00008000 shl DMA_SxNDT_Pos) ## !< 0x00008000

## *******************  Bits definition for DMA_SxFCR register  ***************

const
  DMA_SxFCR_FEIE_Pos* = (7)
  DMA_SxFCR_FEIE_Msk* = (0x00000001 shl DMA_SxFCR_FEIE_Pos) ## !< 0x00000080
  DMA_SxFCR_FEIE* = DMA_SxFCR_FEIE_Msk
  DMA_SxFCR_FS_Pos* = (3)
  DMA_SxFCR_FS_Msk* = (0x00000007 shl DMA_SxFCR_FS_Pos) ## !< 0x00000038
  DMA_SxFCR_FS* = DMA_SxFCR_FS_Msk
  DMA_SxFCR_FS_0* = (0x00000001 shl DMA_SxFCR_FS_Pos) ## !< 0x00000008
  DMA_SxFCR_FS_1* = (0x00000002 shl DMA_SxFCR_FS_Pos) ## !< 0x00000010
  DMA_SxFCR_FS_2* = (0x00000004 shl DMA_SxFCR_FS_Pos) ## !< 0x00000020
  DMA_SxFCR_DMDIS_Pos* = (2)
  DMA_SxFCR_DMDIS_Msk* = (0x00000001 shl DMA_SxFCR_DMDIS_Pos) ## !< 0x00000004
  DMA_SxFCR_DMDIS* = DMA_SxFCR_DMDIS_Msk
  DMA_SxFCR_FTH_Pos* = (0)
  DMA_SxFCR_FTH_Msk* = (0x00000003 shl DMA_SxFCR_FTH_Pos) ## !< 0x00000003
  DMA_SxFCR_FTH* = DMA_SxFCR_FTH_Msk
  DMA_SxFCR_FTH_0* = (0x00000001 shl DMA_SxFCR_FTH_Pos) ## !< 0x00000001
  DMA_SxFCR_FTH_1* = (0x00000002 shl DMA_SxFCR_FTH_Pos) ## !< 0x00000002

## *******************  Bits definition for DMA_LISR register  ****************

const
  DMA_LISR_TCIF3_Pos* = (27)
  DMA_LISR_TCIF3_Msk* = (0x00000001 shl DMA_LISR_TCIF3_Pos) ## !< 0x08000000
  DMA_LISR_TCIF3* = DMA_LISR_TCIF3_Msk
  DMA_LISR_HTIF3_Pos* = (26)
  DMA_LISR_HTIF3_Msk* = (0x00000001 shl DMA_LISR_HTIF3_Pos) ## !< 0x04000000
  DMA_LISR_HTIF3* = DMA_LISR_HTIF3_Msk
  DMA_LISR_TEIF3_Pos* = (25)
  DMA_LISR_TEIF3_Msk* = (0x00000001 shl DMA_LISR_TEIF3_Pos) ## !< 0x02000000
  DMA_LISR_TEIF3* = DMA_LISR_TEIF3_Msk
  DMA_LISR_DMEIF3_Pos* = (24)
  DMA_LISR_DMEIF3_Msk* = (0x00000001 shl DMA_LISR_DMEIF3_Pos) ## !< 0x01000000
  DMA_LISR_DMEIF3* = DMA_LISR_DMEIF3_Msk
  DMA_LISR_FEIF3_Pos* = (22)
  DMA_LISR_FEIF3_Msk* = (0x00000001 shl DMA_LISR_FEIF3_Pos) ## !< 0x00400000
  DMA_LISR_FEIF3* = DMA_LISR_FEIF3_Msk
  DMA_LISR_TCIF2_Pos* = (21)
  DMA_LISR_TCIF2_Msk* = (0x00000001 shl DMA_LISR_TCIF2_Pos) ## !< 0x00200000
  DMA_LISR_TCIF2* = DMA_LISR_TCIF2_Msk
  DMA_LISR_HTIF2_Pos* = (20)
  DMA_LISR_HTIF2_Msk* = (0x00000001 shl DMA_LISR_HTIF2_Pos) ## !< 0x00100000
  DMA_LISR_HTIF2* = DMA_LISR_HTIF2_Msk
  DMA_LISR_TEIF2_Pos* = (19)
  DMA_LISR_TEIF2_Msk* = (0x00000001 shl DMA_LISR_TEIF2_Pos) ## !< 0x00080000
  DMA_LISR_TEIF2* = DMA_LISR_TEIF2_Msk
  DMA_LISR_DMEIF2_Pos* = (18)
  DMA_LISR_DMEIF2_Msk* = (0x00000001 shl DMA_LISR_DMEIF2_Pos) ## !< 0x00040000
  DMA_LISR_DMEIF2* = DMA_LISR_DMEIF2_Msk
  DMA_LISR_FEIF2_Pos* = (16)
  DMA_LISR_FEIF2_Msk* = (0x00000001 shl DMA_LISR_FEIF2_Pos) ## !< 0x00010000
  DMA_LISR_FEIF2* = DMA_LISR_FEIF2_Msk
  DMA_LISR_TCIF1_Pos* = (11)
  DMA_LISR_TCIF1_Msk* = (0x00000001 shl DMA_LISR_TCIF1_Pos) ## !< 0x00000800
  DMA_LISR_TCIF1* = DMA_LISR_TCIF1_Msk
  DMA_LISR_HTIF1_Pos* = (10)
  DMA_LISR_HTIF1_Msk* = (0x00000001 shl DMA_LISR_HTIF1_Pos) ## !< 0x00000400
  DMA_LISR_HTIF1* = DMA_LISR_HTIF1_Msk
  DMA_LISR_TEIF1_Pos* = (9)
  DMA_LISR_TEIF1_Msk* = (0x00000001 shl DMA_LISR_TEIF1_Pos) ## !< 0x00000200
  DMA_LISR_TEIF1* = DMA_LISR_TEIF1_Msk
  DMA_LISR_DMEIF1_Pos* = (8)
  DMA_LISR_DMEIF1_Msk* = (0x00000001 shl DMA_LISR_DMEIF1_Pos) ## !< 0x00000100
  DMA_LISR_DMEIF1* = DMA_LISR_DMEIF1_Msk
  DMA_LISR_FEIF1_Pos* = (6)
  DMA_LISR_FEIF1_Msk* = (0x00000001 shl DMA_LISR_FEIF1_Pos) ## !< 0x00000040
  DMA_LISR_FEIF1* = DMA_LISR_FEIF1_Msk
  DMA_LISR_TCIF0_Pos* = (5)
  DMA_LISR_TCIF0_Msk* = (0x00000001 shl DMA_LISR_TCIF0_Pos) ## !< 0x00000020
  DMA_LISR_TCIF0* = DMA_LISR_TCIF0_Msk
  DMA_LISR_HTIF0_Pos* = (4)
  DMA_LISR_HTIF0_Msk* = (0x00000001 shl DMA_LISR_HTIF0_Pos) ## !< 0x00000010
  DMA_LISR_HTIF0* = DMA_LISR_HTIF0_Msk
  DMA_LISR_TEIF0_Pos* = (3)
  DMA_LISR_TEIF0_Msk* = (0x00000001 shl DMA_LISR_TEIF0_Pos) ## !< 0x00000008
  DMA_LISR_TEIF0* = DMA_LISR_TEIF0_Msk
  DMA_LISR_DMEIF0_Pos* = (2)
  DMA_LISR_DMEIF0_Msk* = (0x00000001 shl DMA_LISR_DMEIF0_Pos) ## !< 0x00000004
  DMA_LISR_DMEIF0* = DMA_LISR_DMEIF0_Msk
  DMA_LISR_FEIF0_Pos* = (0)
  DMA_LISR_FEIF0_Msk* = (0x00000001 shl DMA_LISR_FEIF0_Pos) ## !< 0x00000001
  DMA_LISR_FEIF0* = DMA_LISR_FEIF0_Msk

## *******************  Bits definition for DMA_HISR register  ****************

const
  DMA_HISR_TCIF7_Pos* = (27)
  DMA_HISR_TCIF7_Msk* = (0x00000001 shl DMA_HISR_TCIF7_Pos) ## !< 0x08000000
  DMA_HISR_TCIF7* = DMA_HISR_TCIF7_Msk
  DMA_HISR_HTIF7_Pos* = (26)
  DMA_HISR_HTIF7_Msk* = (0x00000001 shl DMA_HISR_HTIF7_Pos) ## !< 0x04000000
  DMA_HISR_HTIF7* = DMA_HISR_HTIF7_Msk
  DMA_HISR_TEIF7_Pos* = (25)
  DMA_HISR_TEIF7_Msk* = (0x00000001 shl DMA_HISR_TEIF7_Pos) ## !< 0x02000000
  DMA_HISR_TEIF7* = DMA_HISR_TEIF7_Msk
  DMA_HISR_DMEIF7_Pos* = (24)
  DMA_HISR_DMEIF7_Msk* = (0x00000001 shl DMA_HISR_DMEIF7_Pos) ## !< 0x01000000
  DMA_HISR_DMEIF7* = DMA_HISR_DMEIF7_Msk
  DMA_HISR_FEIF7_Pos* = (22)
  DMA_HISR_FEIF7_Msk* = (0x00000001 shl DMA_HISR_FEIF7_Pos) ## !< 0x00400000
  DMA_HISR_FEIF7* = DMA_HISR_FEIF7_Msk
  DMA_HISR_TCIF6_Pos* = (21)
  DMA_HISR_TCIF6_Msk* = (0x00000001 shl DMA_HISR_TCIF6_Pos) ## !< 0x00200000
  DMA_HISR_TCIF6* = DMA_HISR_TCIF6_Msk
  DMA_HISR_HTIF6_Pos* = (20)
  DMA_HISR_HTIF6_Msk* = (0x00000001 shl DMA_HISR_HTIF6_Pos) ## !< 0x00100000
  DMA_HISR_HTIF6* = DMA_HISR_HTIF6_Msk
  DMA_HISR_TEIF6_Pos* = (19)
  DMA_HISR_TEIF6_Msk* = (0x00000001 shl DMA_HISR_TEIF6_Pos) ## !< 0x00080000
  DMA_HISR_TEIF6* = DMA_HISR_TEIF6_Msk
  DMA_HISR_DMEIF6_Pos* = (18)
  DMA_HISR_DMEIF6_Msk* = (0x00000001 shl DMA_HISR_DMEIF6_Pos) ## !< 0x00040000
  DMA_HISR_DMEIF6* = DMA_HISR_DMEIF6_Msk
  DMA_HISR_FEIF6_Pos* = (16)
  DMA_HISR_FEIF6_Msk* = (0x00000001 shl DMA_HISR_FEIF6_Pos) ## !< 0x00010000
  DMA_HISR_FEIF6* = DMA_HISR_FEIF6_Msk
  DMA_HISR_TCIF5_Pos* = (11)
  DMA_HISR_TCIF5_Msk* = (0x00000001 shl DMA_HISR_TCIF5_Pos) ## !< 0x00000800
  DMA_HISR_TCIF5* = DMA_HISR_TCIF5_Msk
  DMA_HISR_HTIF5_Pos* = (10)
  DMA_HISR_HTIF5_Msk* = (0x00000001 shl DMA_HISR_HTIF5_Pos) ## !< 0x00000400
  DMA_HISR_HTIF5* = DMA_HISR_HTIF5_Msk
  DMA_HISR_TEIF5_Pos* = (9)
  DMA_HISR_TEIF5_Msk* = (0x00000001 shl DMA_HISR_TEIF5_Pos) ## !< 0x00000200
  DMA_HISR_TEIF5* = DMA_HISR_TEIF5_Msk
  DMA_HISR_DMEIF5_Pos* = (8)
  DMA_HISR_DMEIF5_Msk* = (0x00000001 shl DMA_HISR_DMEIF5_Pos) ## !< 0x00000100
  DMA_HISR_DMEIF5* = DMA_HISR_DMEIF5_Msk
  DMA_HISR_FEIF5_Pos* = (6)
  DMA_HISR_FEIF5_Msk* = (0x00000001 shl DMA_HISR_FEIF5_Pos) ## !< 0x00000040
  DMA_HISR_FEIF5* = DMA_HISR_FEIF5_Msk
  DMA_HISR_TCIF4_Pos* = (5)
  DMA_HISR_TCIF4_Msk* = (0x00000001 shl DMA_HISR_TCIF4_Pos) ## !< 0x00000020
  DMA_HISR_TCIF4* = DMA_HISR_TCIF4_Msk
  DMA_HISR_HTIF4_Pos* = (4)
  DMA_HISR_HTIF4_Msk* = (0x00000001 shl DMA_HISR_HTIF4_Pos) ## !< 0x00000010
  DMA_HISR_HTIF4* = DMA_HISR_HTIF4_Msk
  DMA_HISR_TEIF4_Pos* = (3)
  DMA_HISR_TEIF4_Msk* = (0x00000001 shl DMA_HISR_TEIF4_Pos) ## !< 0x00000008
  DMA_HISR_TEIF4* = DMA_HISR_TEIF4_Msk
  DMA_HISR_DMEIF4_Pos* = (2)
  DMA_HISR_DMEIF4_Msk* = (0x00000001 shl DMA_HISR_DMEIF4_Pos) ## !< 0x00000004
  DMA_HISR_DMEIF4* = DMA_HISR_DMEIF4_Msk
  DMA_HISR_FEIF4_Pos* = (0)
  DMA_HISR_FEIF4_Msk* = (0x00000001 shl DMA_HISR_FEIF4_Pos) ## !< 0x00000001
  DMA_HISR_FEIF4* = DMA_HISR_FEIF4_Msk

## *******************  Bits definition for DMA_LIFCR register  ***************

const
  DMA_LIFCR_CTCIF3_Pos* = (27)
  DMA_LIFCR_CTCIF3_Msk* = (0x00000001 shl DMA_LIFCR_CTCIF3_Pos) ## !< 0x08000000
  DMA_LIFCR_CTCIF3* = DMA_LIFCR_CTCIF3_Msk
  DMA_LIFCR_CHTIF3_Pos* = (26)
  DMA_LIFCR_CHTIF3_Msk* = (0x00000001 shl DMA_LIFCR_CHTIF3_Pos) ## !< 0x04000000
  DMA_LIFCR_CHTIF3* = DMA_LIFCR_CHTIF3_Msk
  DMA_LIFCR_CTEIF3_Pos* = (25)
  DMA_LIFCR_CTEIF3_Msk* = (0x00000001 shl DMA_LIFCR_CTEIF3_Pos) ## !< 0x02000000
  DMA_LIFCR_CTEIF3* = DMA_LIFCR_CTEIF3_Msk
  DMA_LIFCR_CDMEIF3_Pos* = (24)
  DMA_LIFCR_CDMEIF3_Msk* = (0x00000001 shl DMA_LIFCR_CDMEIF3_Pos) ## !< 0x01000000
  DMA_LIFCR_CDMEIF3* = DMA_LIFCR_CDMEIF3_Msk
  DMA_LIFCR_CFEIF3_Pos* = (22)
  DMA_LIFCR_CFEIF3_Msk* = (0x00000001 shl DMA_LIFCR_CFEIF3_Pos) ## !< 0x00400000
  DMA_LIFCR_CFEIF3* = DMA_LIFCR_CFEIF3_Msk
  DMA_LIFCR_CTCIF2_Pos* = (21)
  DMA_LIFCR_CTCIF2_Msk* = (0x00000001 shl DMA_LIFCR_CTCIF2_Pos) ## !< 0x00200000
  DMA_LIFCR_CTCIF2* = DMA_LIFCR_CTCIF2_Msk
  DMA_LIFCR_CHTIF2_Pos* = (20)
  DMA_LIFCR_CHTIF2_Msk* = (0x00000001 shl DMA_LIFCR_CHTIF2_Pos) ## !< 0x00100000
  DMA_LIFCR_CHTIF2* = DMA_LIFCR_CHTIF2_Msk
  DMA_LIFCR_CTEIF2_Pos* = (19)
  DMA_LIFCR_CTEIF2_Msk* = (0x00000001 shl DMA_LIFCR_CTEIF2_Pos) ## !< 0x00080000
  DMA_LIFCR_CTEIF2* = DMA_LIFCR_CTEIF2_Msk
  DMA_LIFCR_CDMEIF2_Pos* = (18)
  DMA_LIFCR_CDMEIF2_Msk* = (0x00000001 shl DMA_LIFCR_CDMEIF2_Pos) ## !< 0x00040000
  DMA_LIFCR_CDMEIF2* = DMA_LIFCR_CDMEIF2_Msk
  DMA_LIFCR_CFEIF2_Pos* = (16)
  DMA_LIFCR_CFEIF2_Msk* = (0x00000001 shl DMA_LIFCR_CFEIF2_Pos) ## !< 0x00010000
  DMA_LIFCR_CFEIF2* = DMA_LIFCR_CFEIF2_Msk
  DMA_LIFCR_CTCIF1_Pos* = (11)
  DMA_LIFCR_CTCIF1_Msk* = (0x00000001 shl DMA_LIFCR_CTCIF1_Pos) ## !< 0x00000800
  DMA_LIFCR_CTCIF1* = DMA_LIFCR_CTCIF1_Msk
  DMA_LIFCR_CHTIF1_Pos* = (10)
  DMA_LIFCR_CHTIF1_Msk* = (0x00000001 shl DMA_LIFCR_CHTIF1_Pos) ## !< 0x00000400
  DMA_LIFCR_CHTIF1* = DMA_LIFCR_CHTIF1_Msk
  DMA_LIFCR_CTEIF1_Pos* = (9)
  DMA_LIFCR_CTEIF1_Msk* = (0x00000001 shl DMA_LIFCR_CTEIF1_Pos) ## !< 0x00000200
  DMA_LIFCR_CTEIF1* = DMA_LIFCR_CTEIF1_Msk
  DMA_LIFCR_CDMEIF1_Pos* = (8)
  DMA_LIFCR_CDMEIF1_Msk* = (0x00000001 shl DMA_LIFCR_CDMEIF1_Pos) ## !< 0x00000100
  DMA_LIFCR_CDMEIF1* = DMA_LIFCR_CDMEIF1_Msk
  DMA_LIFCR_CFEIF1_Pos* = (6)
  DMA_LIFCR_CFEIF1_Msk* = (0x00000001 shl DMA_LIFCR_CFEIF1_Pos) ## !< 0x00000040
  DMA_LIFCR_CFEIF1* = DMA_LIFCR_CFEIF1_Msk
  DMA_LIFCR_CTCIF0_Pos* = (5)
  DMA_LIFCR_CTCIF0_Msk* = (0x00000001 shl DMA_LIFCR_CTCIF0_Pos) ## !< 0x00000020
  DMA_LIFCR_CTCIF0* = DMA_LIFCR_CTCIF0_Msk
  DMA_LIFCR_CHTIF0_Pos* = (4)
  DMA_LIFCR_CHTIF0_Msk* = (0x00000001 shl DMA_LIFCR_CHTIF0_Pos) ## !< 0x00000010
  DMA_LIFCR_CHTIF0* = DMA_LIFCR_CHTIF0_Msk
  DMA_LIFCR_CTEIF0_Pos* = (3)
  DMA_LIFCR_CTEIF0_Msk* = (0x00000001 shl DMA_LIFCR_CTEIF0_Pos) ## !< 0x00000008
  DMA_LIFCR_CTEIF0* = DMA_LIFCR_CTEIF0_Msk
  DMA_LIFCR_CDMEIF0_Pos* = (2)
  DMA_LIFCR_CDMEIF0_Msk* = (0x00000001 shl DMA_LIFCR_CDMEIF0_Pos) ## !< 0x00000004
  DMA_LIFCR_CDMEIF0* = DMA_LIFCR_CDMEIF0_Msk
  DMA_LIFCR_CFEIF0_Pos* = (0)
  DMA_LIFCR_CFEIF0_Msk* = (0x00000001 shl DMA_LIFCR_CFEIF0_Pos) ## !< 0x00000001
  DMA_LIFCR_CFEIF0* = DMA_LIFCR_CFEIF0_Msk

## *******************  Bits definition for DMA_HIFCR  register  ***************

const
  DMA_HIFCR_CTCIF7_Pos* = (27)
  DMA_HIFCR_CTCIF7_Msk* = (0x00000001 shl DMA_HIFCR_CTCIF7_Pos) ## !< 0x08000000
  DMA_HIFCR_CTCIF7* = DMA_HIFCR_CTCIF7_Msk
  DMA_HIFCR_CHTIF7_Pos* = (26)
  DMA_HIFCR_CHTIF7_Msk* = (0x00000001 shl DMA_HIFCR_CHTIF7_Pos) ## !< 0x04000000
  DMA_HIFCR_CHTIF7* = DMA_HIFCR_CHTIF7_Msk
  DMA_HIFCR_CTEIF7_Pos* = (25)
  DMA_HIFCR_CTEIF7_Msk* = (0x00000001 shl DMA_HIFCR_CTEIF7_Pos) ## !< 0x02000000
  DMA_HIFCR_CTEIF7* = DMA_HIFCR_CTEIF7_Msk
  DMA_HIFCR_CDMEIF7_Pos* = (24)
  DMA_HIFCR_CDMEIF7_Msk* = (0x00000001 shl DMA_HIFCR_CDMEIF7_Pos) ## !< 0x01000000
  DMA_HIFCR_CDMEIF7* = DMA_HIFCR_CDMEIF7_Msk
  DMA_HIFCR_CFEIF7_Pos* = (22)
  DMA_HIFCR_CFEIF7_Msk* = (0x00000001 shl DMA_HIFCR_CFEIF7_Pos) ## !< 0x00400000
  DMA_HIFCR_CFEIF7* = DMA_HIFCR_CFEIF7_Msk
  DMA_HIFCR_CTCIF6_Pos* = (21)
  DMA_HIFCR_CTCIF6_Msk* = (0x00000001 shl DMA_HIFCR_CTCIF6_Pos) ## !< 0x00200000
  DMA_HIFCR_CTCIF6* = DMA_HIFCR_CTCIF6_Msk
  DMA_HIFCR_CHTIF6_Pos* = (20)
  DMA_HIFCR_CHTIF6_Msk* = (0x00000001 shl DMA_HIFCR_CHTIF6_Pos) ## !< 0x00100000
  DMA_HIFCR_CHTIF6* = DMA_HIFCR_CHTIF6_Msk
  DMA_HIFCR_CTEIF6_Pos* = (19)
  DMA_HIFCR_CTEIF6_Msk* = (0x00000001 shl DMA_HIFCR_CTEIF6_Pos) ## !< 0x00080000
  DMA_HIFCR_CTEIF6* = DMA_HIFCR_CTEIF6_Msk
  DMA_HIFCR_CDMEIF6_Pos* = (18)
  DMA_HIFCR_CDMEIF6_Msk* = (0x00000001 shl DMA_HIFCR_CDMEIF6_Pos) ## !< 0x00040000
  DMA_HIFCR_CDMEIF6* = DMA_HIFCR_CDMEIF6_Msk
  DMA_HIFCR_CFEIF6_Pos* = (16)
  DMA_HIFCR_CFEIF6_Msk* = (0x00000001 shl DMA_HIFCR_CFEIF6_Pos) ## !< 0x00010000
  DMA_HIFCR_CFEIF6* = DMA_HIFCR_CFEIF6_Msk
  DMA_HIFCR_CTCIF5_Pos* = (11)
  DMA_HIFCR_CTCIF5_Msk* = (0x00000001 shl DMA_HIFCR_CTCIF5_Pos) ## !< 0x00000800
  DMA_HIFCR_CTCIF5* = DMA_HIFCR_CTCIF5_Msk
  DMA_HIFCR_CHTIF5_Pos* = (10)
  DMA_HIFCR_CHTIF5_Msk* = (0x00000001 shl DMA_HIFCR_CHTIF5_Pos) ## !< 0x00000400
  DMA_HIFCR_CHTIF5* = DMA_HIFCR_CHTIF5_Msk
  DMA_HIFCR_CTEIF5_Pos* = (9)
  DMA_HIFCR_CTEIF5_Msk* = (0x00000001 shl DMA_HIFCR_CTEIF5_Pos) ## !< 0x00000200
  DMA_HIFCR_CTEIF5* = DMA_HIFCR_CTEIF5_Msk
  DMA_HIFCR_CDMEIF5_Pos* = (8)
  DMA_HIFCR_CDMEIF5_Msk* = (0x00000001 shl DMA_HIFCR_CDMEIF5_Pos) ## !< 0x00000100
  DMA_HIFCR_CDMEIF5* = DMA_HIFCR_CDMEIF5_Msk
  DMA_HIFCR_CFEIF5_Pos* = (6)
  DMA_HIFCR_CFEIF5_Msk* = (0x00000001 shl DMA_HIFCR_CFEIF5_Pos) ## !< 0x00000040
  DMA_HIFCR_CFEIF5* = DMA_HIFCR_CFEIF5_Msk
  DMA_HIFCR_CTCIF4_Pos* = (5)
  DMA_HIFCR_CTCIF4_Msk* = (0x00000001 shl DMA_HIFCR_CTCIF4_Pos) ## !< 0x00000020
  DMA_HIFCR_CTCIF4* = DMA_HIFCR_CTCIF4_Msk
  DMA_HIFCR_CHTIF4_Pos* = (4)
  DMA_HIFCR_CHTIF4_Msk* = (0x00000001 shl DMA_HIFCR_CHTIF4_Pos) ## !< 0x00000010
  DMA_HIFCR_CHTIF4* = DMA_HIFCR_CHTIF4_Msk
  DMA_HIFCR_CTEIF4_Pos* = (3)
  DMA_HIFCR_CTEIF4_Msk* = (0x00000001 shl DMA_HIFCR_CTEIF4_Pos) ## !< 0x00000008
  DMA_HIFCR_CTEIF4* = DMA_HIFCR_CTEIF4_Msk
  DMA_HIFCR_CDMEIF4_Pos* = (2)
  DMA_HIFCR_CDMEIF4_Msk* = (0x00000001 shl DMA_HIFCR_CDMEIF4_Pos) ## !< 0x00000004
  DMA_HIFCR_CDMEIF4* = DMA_HIFCR_CDMEIF4_Msk
  DMA_HIFCR_CFEIF4_Pos* = (0)
  DMA_HIFCR_CFEIF4_Msk* = (0x00000001 shl DMA_HIFCR_CFEIF4_Pos) ## !< 0x00000001
  DMA_HIFCR_CFEIF4* = DMA_HIFCR_CFEIF4_Msk

## *****************  Bit definition for DMA_SxPAR register  *******************

const
  DMA_SxPAR_PA_Pos* = (0)
  DMA_SxPAR_PA_Msk* = (0xFFFFFFFF shl DMA_SxPAR_PA_Pos) ## !< 0xFFFFFFFF
  DMA_SxPAR_PA* = DMA_SxPAR_PA_Msk

## *****************  Bit definition for DMA_SxM0AR register  *******************

const
  DMA_SxM0AR_M0A_Pos* = (0)
  DMA_SxM0AR_M0A_Msk* = (0xFFFFFFFF shl DMA_SxM0AR_M0A_Pos) ## !< 0xFFFFFFFF
  DMA_SxM0AR_M0A* = DMA_SxM0AR_M0A_Msk

## *****************  Bit definition for DMA_SxM1AR register  *******************

const
  DMA_SxM1AR_M1A_Pos* = (0)
  DMA_SxM1AR_M1A_Msk* = (0xFFFFFFFF shl DMA_SxM1AR_M1A_Pos) ## !< 0xFFFFFFFF
  DMA_SxM1AR_M1A* = DMA_SxM1AR_M1A_Msk

## ****************************************************************************
##
##                     External Interrupt/Event Controller
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

##  Reference Defines

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
  EXTI_IMR_IM_Pos* = (0)
  EXTI_IMR_IM_Msk* = (0x007FFFFF shl EXTI_IMR_IM_Pos) ## !< 0x007FFFFF
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

##  Reference Defines

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

## ****************************************************************************
##
##                                     FLASH
##
## ****************************************************************************
## ******************  Bits definition for FLASH_ACR register  ****************

const
  FLASH_ACR_LATENCY_Pos* = (0)
  FLASH_ACR_LATENCY_Msk* = (0x0000000F shl FLASH_ACR_LATENCY_Pos) ## !< 0x0000000F
  FLASH_ACR_LATENCY* = FLASH_ACR_LATENCY_Msk
  FLASH_ACR_LATENCY_0WS* = 0x00000000
  FLASH_ACR_LATENCY_1WS* = 0x00000001
  FLASH_ACR_LATENCY_2WS* = 0x00000002
  FLASH_ACR_LATENCY_3WS* = 0x00000003
  FLASH_ACR_LATENCY_4WS* = 0x00000004
  FLASH_ACR_LATENCY_5WS* = 0x00000005
  FLASH_ACR_LATENCY_6WS* = 0x00000006
  FLASH_ACR_LATENCY_7WS* = 0x00000007
  FLASH_ACR_PRFTEN_Pos* = (8)
  FLASH_ACR_PRFTEN_Msk* = (0x00000001 shl FLASH_ACR_PRFTEN_Pos) ## !< 0x00000100
  FLASH_ACR_PRFTEN* = FLASH_ACR_PRFTEN_Msk
  FLASH_ACR_ICEN_Pos* = (9)
  FLASH_ACR_ICEN_Msk* = (0x00000001 shl FLASH_ACR_ICEN_Pos) ## !< 0x00000200
  FLASH_ACR_ICEN* = FLASH_ACR_ICEN_Msk
  FLASH_ACR_DCEN_Pos* = (10)
  FLASH_ACR_DCEN_Msk* = (0x00000001 shl FLASH_ACR_DCEN_Pos) ## !< 0x00000400
  FLASH_ACR_DCEN* = FLASH_ACR_DCEN_Msk
  FLASH_ACR_ICRST_Pos* = (11)
  FLASH_ACR_ICRST_Msk* = (0x00000001 shl FLASH_ACR_ICRST_Pos) ## !< 0x00000800
  FLASH_ACR_ICRST* = FLASH_ACR_ICRST_Msk
  FLASH_ACR_DCRST_Pos* = (12)
  FLASH_ACR_DCRST_Msk* = (0x00000001 shl FLASH_ACR_DCRST_Pos) ## !< 0x00001000
  FLASH_ACR_DCRST* = FLASH_ACR_DCRST_Msk
  FLASH_ACR_BYTE0_ADDRESS_Pos* = (10)
  FLASH_ACR_BYTE0_ADDRESS_Msk* = (0x0010008F shl FLASH_ACR_BYTE0_ADDRESS_Pos) ## !< 0x40023C00
  FLASH_ACR_BYTE0_ADDRESS* = FLASH_ACR_BYTE0_ADDRESS_Msk
  FLASH_ACR_BYTE2_ADDRESS_Pos* = (0)
  FLASH_ACR_BYTE2_ADDRESS_Msk* = (0x40023C03 shl FLASH_ACR_BYTE2_ADDRESS_Pos) ## !< 0x40023C03
  FLASH_ACR_BYTE2_ADDRESS* = FLASH_ACR_BYTE2_ADDRESS_Msk

## ******************  Bits definition for FLASH_SR register  *****************

const
  FLASH_SR_EOP_Pos* = (0)
  FLASH_SR_EOP_Msk* = (0x00000001 shl FLASH_SR_EOP_Pos) ## !< 0x00000001
  FLASH_SR_EOP* = FLASH_SR_EOP_Msk
  FLASH_SR_SOP_Pos* = (1)
  FLASH_SR_SOP_Msk* = (0x00000001 shl FLASH_SR_SOP_Pos) ## !< 0x00000002
  FLASH_SR_SOP* = FLASH_SR_SOP_Msk
  FLASH_SR_WRPERR_Pos* = (4)
  FLASH_SR_WRPERR_Msk* = (0x00000001 shl FLASH_SR_WRPERR_Pos) ## !< 0x00000010
  FLASH_SR_WRPERR* = FLASH_SR_WRPERR_Msk
  FLASH_SR_PGAERR_Pos* = (5)
  FLASH_SR_PGAERR_Msk* = (0x00000001 shl FLASH_SR_PGAERR_Pos) ## !< 0x00000020
  FLASH_SR_PGAERR* = FLASH_SR_PGAERR_Msk
  FLASH_SR_PGPERR_Pos* = (6)
  FLASH_SR_PGPERR_Msk* = (0x00000001 shl FLASH_SR_PGPERR_Pos) ## !< 0x00000040
  FLASH_SR_PGPERR* = FLASH_SR_PGPERR_Msk
  FLASH_SR_PGSERR_Pos* = (7)
  FLASH_SR_PGSERR_Msk* = (0x00000001 shl FLASH_SR_PGSERR_Pos) ## !< 0x00000080
  FLASH_SR_PGSERR* = FLASH_SR_PGSERR_Msk
  FLASH_SR_BSY_Pos* = (16)
  FLASH_SR_BSY_Msk* = (0x00000001 shl FLASH_SR_BSY_Pos) ## !< 0x00010000
  FLASH_SR_BSY* = FLASH_SR_BSY_Msk

## ******************  Bits definition for FLASH_CR register  *****************

const
  FLASH_CR_PG_Pos* = (0)
  FLASH_CR_PG_Msk* = (0x00000001 shl FLASH_CR_PG_Pos) ## !< 0x00000001
  FLASH_CR_PG* = FLASH_CR_PG_Msk
  FLASH_CR_SER_Pos* = (1)
  FLASH_CR_SER_Msk* = (0x00000001 shl FLASH_CR_SER_Pos) ## !< 0x00000002
  FLASH_CR_SER* = FLASH_CR_SER_Msk
  FLASH_CR_MER_Pos* = (2)
  FLASH_CR_MER_Msk* = (0x00000001 shl FLASH_CR_MER_Pos) ## !< 0x00000004
  FLASH_CR_MER* = FLASH_CR_MER_Msk
  FLASH_CR_SNB_Pos* = (3)
  FLASH_CR_SNB_Msk* = (0x0000001F shl FLASH_CR_SNB_Pos) ## !< 0x000000F8
  FLASH_CR_SNB* = FLASH_CR_SNB_Msk
  FLASH_CR_SNB_0* = (0x00000001 shl FLASH_CR_SNB_Pos) ## !< 0x00000008
  FLASH_CR_SNB_1* = (0x00000002 shl FLASH_CR_SNB_Pos) ## !< 0x00000010
  FLASH_CR_SNB_2* = (0x00000004 shl FLASH_CR_SNB_Pos) ## !< 0x00000020
  FLASH_CR_SNB_3* = (0x00000008 shl FLASH_CR_SNB_Pos) ## !< 0x00000040
  FLASH_CR_SNB_4* = (0x00000010 shl FLASH_CR_SNB_Pos) ## !< 0x00000080
  FLASH_CR_PSIZE_Pos* = (8)
  FLASH_CR_PSIZE_Msk* = (0x00000003 shl FLASH_CR_PSIZE_Pos) ## !< 0x00000300
  FLASH_CR_PSIZE* = FLASH_CR_PSIZE_Msk
  FLASH_CR_PSIZE_0* = (0x00000001 shl FLASH_CR_PSIZE_Pos) ## !< 0x00000100
  FLASH_CR_PSIZE_1* = (0x00000002 shl FLASH_CR_PSIZE_Pos) ## !< 0x00000200
  FLASH_CR_STRT_Pos* = (16)
  FLASH_CR_STRT_Msk* = (0x00000001 shl FLASH_CR_STRT_Pos) ## !< 0x00010000
  FLASH_CR_STRT* = FLASH_CR_STRT_Msk
  FLASH_CR_EOPIE_Pos* = (24)
  FLASH_CR_EOPIE_Msk* = (0x00000001 shl FLASH_CR_EOPIE_Pos) ## !< 0x01000000
  FLASH_CR_EOPIE* = FLASH_CR_EOPIE_Msk
  FLASH_CR_LOCK_Pos* = (31)
  FLASH_CR_LOCK_Msk* = (0x00000001 shl FLASH_CR_LOCK_Pos) ## !< 0x80000000
  FLASH_CR_LOCK* = FLASH_CR_LOCK_Msk

## ******************  Bits definition for FLASH_OPTCR register  **************

const
  FLASH_OPTCR_OPTLOCK_Pos* = (0)
  FLASH_OPTCR_OPTLOCK_Msk* = (0x00000001 shl FLASH_OPTCR_OPTLOCK_Pos) ## !< 0x00000001
  FLASH_OPTCR_OPTLOCK* = FLASH_OPTCR_OPTLOCK_Msk
  FLASH_OPTCR_OPTSTRT_Pos* = (1)
  FLASH_OPTCR_OPTSTRT_Msk* = (0x00000001 shl FLASH_OPTCR_OPTSTRT_Pos) ## !< 0x00000002
  FLASH_OPTCR_OPTSTRT* = FLASH_OPTCR_OPTSTRT_Msk
  FLASH_OPTCR_BOR_LEV_0* = 0x00000004
  FLASH_OPTCR_BOR_LEV_1* = 0x00000008
  FLASH_OPTCR_BOR_LEV_Pos* = (2)
  FLASH_OPTCR_BOR_LEV_Msk* = (0x00000003 shl FLASH_OPTCR_BOR_LEV_Pos) ## !< 0x0000000C
  FLASH_OPTCR_BOR_LEV* = FLASH_OPTCR_BOR_LEV_Msk
  FLASH_OPTCR_WDG_SW_Pos* = (5)
  FLASH_OPTCR_WDG_SW_Msk* = (0x00000001 shl FLASH_OPTCR_WDG_SW_Pos) ## !< 0x00000020
  FLASH_OPTCR_WDG_SW* = FLASH_OPTCR_WDG_SW_Msk
  FLASH_OPTCR_nRST_STOP_Pos* = (6)
  FLASH_OPTCR_nRST_STOP_Msk* = (0x00000001 shl FLASH_OPTCR_nRST_STOP_Pos) ## !< 0x00000040
  FLASH_OPTCR_nRST_STOP* = FLASH_OPTCR_nRST_STOP_Msk
  FLASH_OPTCR_nRST_STDBY_Pos* = (7)
  FLASH_OPTCR_nRST_STDBY_Msk* = (0x00000001 shl FLASH_OPTCR_nRST_STDBY_Pos) ## !< 0x00000080
  FLASH_OPTCR_nRST_STDBY* = FLASH_OPTCR_nRST_STDBY_Msk
  FLASH_OPTCR_RDP_Pos* = (8)
  FLASH_OPTCR_RDP_Msk* = (0x000000FF shl FLASH_OPTCR_RDP_Pos) ## !< 0x0000FF00
  FLASH_OPTCR_RDP* = FLASH_OPTCR_RDP_Msk
  FLASH_OPTCR_RDP_0* = (0x00000001 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00000100
  FLASH_OPTCR_RDP_1* = (0x00000002 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00000200
  FLASH_OPTCR_RDP_2* = (0x00000004 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00000400
  FLASH_OPTCR_RDP_3* = (0x00000008 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00000800
  FLASH_OPTCR_RDP_4* = (0x00000010 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00001000
  FLASH_OPTCR_RDP_5* = (0x00000020 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00002000
  FLASH_OPTCR_RDP_6* = (0x00000040 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00004000
  FLASH_OPTCR_RDP_7* = (0x00000080 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00008000
  FLASH_OPTCR_nWRP_Pos* = (16)
  FLASH_OPTCR_nWRP_Msk* = (0x00000FFF shl FLASH_OPTCR_nWRP_Pos) ## !< 0x0FFF0000
  FLASH_OPTCR_nWRP* = FLASH_OPTCR_nWRP_Msk
  FLASH_OPTCR_nWRP_0* = 0x00010000
  FLASH_OPTCR_nWRP_1* = 0x00020000
  FLASH_OPTCR_nWRP_2* = 0x00040000
  FLASH_OPTCR_nWRP_3* = 0x00080000
  FLASH_OPTCR_nWRP_4* = 0x00100000
  FLASH_OPTCR_nWRP_5* = 0x00200000
  FLASH_OPTCR_nWRP_6* = 0x00400000
  FLASH_OPTCR_nWRP_7* = 0x00800000
  FLASH_OPTCR_nWRP_8* = 0x01000000
  FLASH_OPTCR_nWRP_9* = 0x02000000
  FLASH_OPTCR_nWRP_10* = 0x04000000
  FLASH_OPTCR_nWRP_11* = 0x08000000

## *****************  Bits definition for FLASH_OPTCR1 register  **************

const
  FLASH_OPTCR1_nWRP_Pos* = (16)
  FLASH_OPTCR1_nWRP_Msk* = (0x00000FFF shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x0FFF0000
  FLASH_OPTCR1_nWRP* = FLASH_OPTCR1_nWRP_Msk
  FLASH_OPTCR1_nWRP_0* = (0x00000001 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00010000
  FLASH_OPTCR1_nWRP_1* = (0x00000002 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00020000
  FLASH_OPTCR1_nWRP_2* = (0x00000004 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00040000
  FLASH_OPTCR1_nWRP_3* = (0x00000008 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00080000
  FLASH_OPTCR1_nWRP_4* = (0x00000010 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00100000
  FLASH_OPTCR1_nWRP_5* = (0x00000020 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00200000
  FLASH_OPTCR1_nWRP_6* = (0x00000040 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00400000
  FLASH_OPTCR1_nWRP_7* = (0x00000080 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00800000
  FLASH_OPTCR1_nWRP_8* = (0x00000100 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x01000000
  FLASH_OPTCR1_nWRP_9* = (0x00000200 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x02000000
  FLASH_OPTCR1_nWRP_10* = (0x00000400 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x04000000
  FLASH_OPTCR1_nWRP_11* = (0x00000800 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x08000000

## ****************************************************************************
##
##                    Flexible Static Memory Controller
##
## ****************************************************************************
## *****************  Bit definition for FSMC_BCR1 register  ******************

const
  FSMC_BCR1_MBKEN_Pos* = (0)
  FSMC_BCR1_MBKEN_Msk* = (0x00000001 shl FSMC_BCR1_MBKEN_Pos) ## !< 0x00000001
  FSMC_BCR1_MBKEN* = FSMC_BCR1_MBKEN_Msk
  FSMC_BCR1_MUXEN_Pos* = (1)
  FSMC_BCR1_MUXEN_Msk* = (0x00000001 shl FSMC_BCR1_MUXEN_Pos) ## !< 0x00000002
  FSMC_BCR1_MUXEN* = FSMC_BCR1_MUXEN_Msk
  FSMC_BCR1_MTYP_Pos* = (2)
  FSMC_BCR1_MTYP_Msk* = (0x00000003 shl FSMC_BCR1_MTYP_Pos) ## !< 0x0000000C
  FSMC_BCR1_MTYP* = FSMC_BCR1_MTYP_Msk
  FSMC_BCR1_MTYP_0* = (0x00000001 shl FSMC_BCR1_MTYP_Pos) ## !< 0x00000004
  FSMC_BCR1_MTYP_1* = (0x00000002 shl FSMC_BCR1_MTYP_Pos) ## !< 0x00000008
  FSMC_BCR1_MWID_Pos* = (4)
  FSMC_BCR1_MWID_Msk* = (0x00000003 shl FSMC_BCR1_MWID_Pos) ## !< 0x00000030
  FSMC_BCR1_MWID* = FSMC_BCR1_MWID_Msk
  FSMC_BCR1_MWID_0* = (0x00000001 shl FSMC_BCR1_MWID_Pos) ## !< 0x00000010
  FSMC_BCR1_MWID_1* = (0x00000002 shl FSMC_BCR1_MWID_Pos) ## !< 0x00000020
  FSMC_BCR1_FACCEN_Pos* = (6)
  FSMC_BCR1_FACCEN_Msk* = (0x00000001 shl FSMC_BCR1_FACCEN_Pos) ## !< 0x00000040
  FSMC_BCR1_FACCEN* = FSMC_BCR1_FACCEN_Msk
  FSMC_BCR1_BURSTEN_Pos* = (8)
  FSMC_BCR1_BURSTEN_Msk* = (0x00000001 shl FSMC_BCR1_BURSTEN_Pos) ## !< 0x00000100
  FSMC_BCR1_BURSTEN* = FSMC_BCR1_BURSTEN_Msk
  FSMC_BCR1_WAITPOL_Pos* = (9)
  FSMC_BCR1_WAITPOL_Msk* = (0x00000001 shl FSMC_BCR1_WAITPOL_Pos) ## !< 0x00000200
  FSMC_BCR1_WAITPOL* = FSMC_BCR1_WAITPOL_Msk
  FSMC_BCR1_WRAPMOD_Pos* = (10)
  FSMC_BCR1_WRAPMOD_Msk* = (0x00000001 shl FSMC_BCR1_WRAPMOD_Pos) ## !< 0x00000400
  FSMC_BCR1_WRAPMOD* = FSMC_BCR1_WRAPMOD_Msk
  FSMC_BCR1_WAITCFG_Pos* = (11)
  FSMC_BCR1_WAITCFG_Msk* = (0x00000001 shl FSMC_BCR1_WAITCFG_Pos) ## !< 0x00000800
  FSMC_BCR1_WAITCFG* = FSMC_BCR1_WAITCFG_Msk
  FSMC_BCR1_WREN_Pos* = (12)
  FSMC_BCR1_WREN_Msk* = (0x00000001 shl FSMC_BCR1_WREN_Pos) ## !< 0x00001000
  FSMC_BCR1_WREN* = FSMC_BCR1_WREN_Msk
  FSMC_BCR1_WAITEN_Pos* = (13)
  FSMC_BCR1_WAITEN_Msk* = (0x00000001 shl FSMC_BCR1_WAITEN_Pos) ## !< 0x00002000
  FSMC_BCR1_WAITEN* = FSMC_BCR1_WAITEN_Msk
  FSMC_BCR1_EXTMOD_Pos* = (14)
  FSMC_BCR1_EXTMOD_Msk* = (0x00000001 shl FSMC_BCR1_EXTMOD_Pos) ## !< 0x00004000
  FSMC_BCR1_EXTMOD* = FSMC_BCR1_EXTMOD_Msk
  FSMC_BCR1_ASYNCWAIT_Pos* = (15)
  FSMC_BCR1_ASYNCWAIT_Msk* = (0x00000001 shl FSMC_BCR1_ASYNCWAIT_Pos) ## !< 0x00008000
  FSMC_BCR1_ASYNCWAIT* = FSMC_BCR1_ASYNCWAIT_Msk
  FSMC_BCR1_CPSIZE_Pos* = (16)
  FSMC_BCR1_CPSIZE_Msk* = (0x00000007 shl FSMC_BCR1_CPSIZE_Pos) ## !< 0x00070000
  FSMC_BCR1_CPSIZE* = FSMC_BCR1_CPSIZE_Msk
  FSMC_BCR1_CPSIZE_0* = (0x00000001 shl FSMC_BCR1_CPSIZE_Pos) ## !< 0x00010000
  FSMC_BCR1_CPSIZE_1* = (0x00000002 shl FSMC_BCR1_CPSIZE_Pos) ## !< 0x00020000
  FSMC_BCR1_CPSIZE_2* = (0x00000004 shl FSMC_BCR1_CPSIZE_Pos) ## !< 0x00040000
  FSMC_BCR1_CBURSTRW_Pos* = (19)
  FSMC_BCR1_CBURSTRW_Msk* = (0x00000001 shl FSMC_BCR1_CBURSTRW_Pos) ## !< 0x00080000
  FSMC_BCR1_CBURSTRW* = FSMC_BCR1_CBURSTRW_Msk

## *****************  Bit definition for FSMC_BCR2 register  ******************

const
  FSMC_BCR2_MBKEN_Pos* = (0)
  FSMC_BCR2_MBKEN_Msk* = (0x00000001 shl FSMC_BCR2_MBKEN_Pos) ## !< 0x00000001
  FSMC_BCR2_MBKEN* = FSMC_BCR2_MBKEN_Msk
  FSMC_BCR2_MUXEN_Pos* = (1)
  FSMC_BCR2_MUXEN_Msk* = (0x00000001 shl FSMC_BCR2_MUXEN_Pos) ## !< 0x00000002
  FSMC_BCR2_MUXEN* = FSMC_BCR2_MUXEN_Msk
  FSMC_BCR2_MTYP_Pos* = (2)
  FSMC_BCR2_MTYP_Msk* = (0x00000003 shl FSMC_BCR2_MTYP_Pos) ## !< 0x0000000C
  FSMC_BCR2_MTYP* = FSMC_BCR2_MTYP_Msk
  FSMC_BCR2_MTYP_0* = (0x00000001 shl FSMC_BCR2_MTYP_Pos) ## !< 0x00000004
  FSMC_BCR2_MTYP_1* = (0x00000002 shl FSMC_BCR2_MTYP_Pos) ## !< 0x00000008
  FSMC_BCR2_MWID_Pos* = (4)
  FSMC_BCR2_MWID_Msk* = (0x00000003 shl FSMC_BCR2_MWID_Pos) ## !< 0x00000030
  FSMC_BCR2_MWID* = FSMC_BCR2_MWID_Msk
  FSMC_BCR2_MWID_0* = (0x00000001 shl FSMC_BCR2_MWID_Pos) ## !< 0x00000010
  FSMC_BCR2_MWID_1* = (0x00000002 shl FSMC_BCR2_MWID_Pos) ## !< 0x00000020
  FSMC_BCR2_FACCEN_Pos* = (6)
  FSMC_BCR2_FACCEN_Msk* = (0x00000001 shl FSMC_BCR2_FACCEN_Pos) ## !< 0x00000040
  FSMC_BCR2_FACCEN* = FSMC_BCR2_FACCEN_Msk
  FSMC_BCR2_BURSTEN_Pos* = (8)
  FSMC_BCR2_BURSTEN_Msk* = (0x00000001 shl FSMC_BCR2_BURSTEN_Pos) ## !< 0x00000100
  FSMC_BCR2_BURSTEN* = FSMC_BCR2_BURSTEN_Msk
  FSMC_BCR2_WAITPOL_Pos* = (9)
  FSMC_BCR2_WAITPOL_Msk* = (0x00000001 shl FSMC_BCR2_WAITPOL_Pos) ## !< 0x00000200
  FSMC_BCR2_WAITPOL* = FSMC_BCR2_WAITPOL_Msk
  FSMC_BCR2_WRAPMOD_Pos* = (10)
  FSMC_BCR2_WRAPMOD_Msk* = (0x00000001 shl FSMC_BCR2_WRAPMOD_Pos) ## !< 0x00000400
  FSMC_BCR2_WRAPMOD* = FSMC_BCR2_WRAPMOD_Msk
  FSMC_BCR2_WAITCFG_Pos* = (11)
  FSMC_BCR2_WAITCFG_Msk* = (0x00000001 shl FSMC_BCR2_WAITCFG_Pos) ## !< 0x00000800
  FSMC_BCR2_WAITCFG* = FSMC_BCR2_WAITCFG_Msk
  FSMC_BCR2_WREN_Pos* = (12)
  FSMC_BCR2_WREN_Msk* = (0x00000001 shl FSMC_BCR2_WREN_Pos) ## !< 0x00001000
  FSMC_BCR2_WREN* = FSMC_BCR2_WREN_Msk
  FSMC_BCR2_WAITEN_Pos* = (13)
  FSMC_BCR2_WAITEN_Msk* = (0x00000001 shl FSMC_BCR2_WAITEN_Pos) ## !< 0x00002000
  FSMC_BCR2_WAITEN* = FSMC_BCR2_WAITEN_Msk
  FSMC_BCR2_EXTMOD_Pos* = (14)
  FSMC_BCR2_EXTMOD_Msk* = (0x00000001 shl FSMC_BCR2_EXTMOD_Pos) ## !< 0x00004000
  FSMC_BCR2_EXTMOD* = FSMC_BCR2_EXTMOD_Msk
  FSMC_BCR2_ASYNCWAIT_Pos* = (15)
  FSMC_BCR2_ASYNCWAIT_Msk* = (0x00000001 shl FSMC_BCR2_ASYNCWAIT_Pos) ## !< 0x00008000
  FSMC_BCR2_ASYNCWAIT* = FSMC_BCR2_ASYNCWAIT_Msk
  FSMC_BCR2_CPSIZE_Pos* = (16)
  FSMC_BCR2_CPSIZE_Msk* = (0x00000007 shl FSMC_BCR2_CPSIZE_Pos) ## !< 0x00070000
  FSMC_BCR2_CPSIZE* = FSMC_BCR2_CPSIZE_Msk
  FSMC_BCR2_CPSIZE_0* = (0x00000001 shl FSMC_BCR2_CPSIZE_Pos) ## !< 0x00010000
  FSMC_BCR2_CPSIZE_1* = (0x00000002 shl FSMC_BCR2_CPSIZE_Pos) ## !< 0x00020000
  FSMC_BCR2_CPSIZE_2* = (0x00000004 shl FSMC_BCR2_CPSIZE_Pos) ## !< 0x00040000
  FSMC_BCR2_CBURSTRW_Pos* = (19)
  FSMC_BCR2_CBURSTRW_Msk* = (0x00000001 shl FSMC_BCR2_CBURSTRW_Pos) ## !< 0x00080000
  FSMC_BCR2_CBURSTRW* = FSMC_BCR2_CBURSTRW_Msk

## *****************  Bit definition for FSMC_BCR3 register  ******************

const
  FSMC_BCR3_MBKEN_Pos* = (0)
  FSMC_BCR3_MBKEN_Msk* = (0x00000001 shl FSMC_BCR3_MBKEN_Pos) ## !< 0x00000001
  FSMC_BCR3_MBKEN* = FSMC_BCR3_MBKEN_Msk
  FSMC_BCR3_MUXEN_Pos* = (1)
  FSMC_BCR3_MUXEN_Msk* = (0x00000001 shl FSMC_BCR3_MUXEN_Pos) ## !< 0x00000002
  FSMC_BCR3_MUXEN* = FSMC_BCR3_MUXEN_Msk
  FSMC_BCR3_MTYP_Pos* = (2)
  FSMC_BCR3_MTYP_Msk* = (0x00000003 shl FSMC_BCR3_MTYP_Pos) ## !< 0x0000000C
  FSMC_BCR3_MTYP* = FSMC_BCR3_MTYP_Msk
  FSMC_BCR3_MTYP_0* = (0x00000001 shl FSMC_BCR3_MTYP_Pos) ## !< 0x00000004
  FSMC_BCR3_MTYP_1* = (0x00000002 shl FSMC_BCR3_MTYP_Pos) ## !< 0x00000008
  FSMC_BCR3_MWID_Pos* = (4)
  FSMC_BCR3_MWID_Msk* = (0x00000003 shl FSMC_BCR3_MWID_Pos) ## !< 0x00000030
  FSMC_BCR3_MWID* = FSMC_BCR3_MWID_Msk
  FSMC_BCR3_MWID_0* = (0x00000001 shl FSMC_BCR3_MWID_Pos) ## !< 0x00000010
  FSMC_BCR3_MWID_1* = (0x00000002 shl FSMC_BCR3_MWID_Pos) ## !< 0x00000020
  FSMC_BCR3_FACCEN_Pos* = (6)
  FSMC_BCR3_FACCEN_Msk* = (0x00000001 shl FSMC_BCR3_FACCEN_Pos) ## !< 0x00000040
  FSMC_BCR3_FACCEN* = FSMC_BCR3_FACCEN_Msk
  FSMC_BCR3_BURSTEN_Pos* = (8)
  FSMC_BCR3_BURSTEN_Msk* = (0x00000001 shl FSMC_BCR3_BURSTEN_Pos) ## !< 0x00000100
  FSMC_BCR3_BURSTEN* = FSMC_BCR3_BURSTEN_Msk
  FSMC_BCR3_WAITPOL_Pos* = (9)
  FSMC_BCR3_WAITPOL_Msk* = (0x00000001 shl FSMC_BCR3_WAITPOL_Pos) ## !< 0x00000200
  FSMC_BCR3_WAITPOL* = FSMC_BCR3_WAITPOL_Msk
  FSMC_BCR3_WRAPMOD_Pos* = (10)
  FSMC_BCR3_WRAPMOD_Msk* = (0x00000001 shl FSMC_BCR3_WRAPMOD_Pos) ## !< 0x00000400
  FSMC_BCR3_WRAPMOD* = FSMC_BCR3_WRAPMOD_Msk
  FSMC_BCR3_WAITCFG_Pos* = (11)
  FSMC_BCR3_WAITCFG_Msk* = (0x00000001 shl FSMC_BCR3_WAITCFG_Pos) ## !< 0x00000800
  FSMC_BCR3_WAITCFG* = FSMC_BCR3_WAITCFG_Msk
  FSMC_BCR3_WREN_Pos* = (12)
  FSMC_BCR3_WREN_Msk* = (0x00000001 shl FSMC_BCR3_WREN_Pos) ## !< 0x00001000
  FSMC_BCR3_WREN* = FSMC_BCR3_WREN_Msk
  FSMC_BCR3_WAITEN_Pos* = (13)
  FSMC_BCR3_WAITEN_Msk* = (0x00000001 shl FSMC_BCR3_WAITEN_Pos) ## !< 0x00002000
  FSMC_BCR3_WAITEN* = FSMC_BCR3_WAITEN_Msk
  FSMC_BCR3_EXTMOD_Pos* = (14)
  FSMC_BCR3_EXTMOD_Msk* = (0x00000001 shl FSMC_BCR3_EXTMOD_Pos) ## !< 0x00004000
  FSMC_BCR3_EXTMOD* = FSMC_BCR3_EXTMOD_Msk
  FSMC_BCR3_ASYNCWAIT_Pos* = (15)
  FSMC_BCR3_ASYNCWAIT_Msk* = (0x00000001 shl FSMC_BCR3_ASYNCWAIT_Pos) ## !< 0x00008000
  FSMC_BCR3_ASYNCWAIT* = FSMC_BCR3_ASYNCWAIT_Msk
  FSMC_BCR3_CPSIZE_Pos* = (16)
  FSMC_BCR3_CPSIZE_Msk* = (0x00000007 shl FSMC_BCR3_CPSIZE_Pos) ## !< 0x00070000
  FSMC_BCR3_CPSIZE* = FSMC_BCR3_CPSIZE_Msk
  FSMC_BCR3_CPSIZE_0* = (0x00000001 shl FSMC_BCR3_CPSIZE_Pos) ## !< 0x00010000
  FSMC_BCR3_CPSIZE_1* = (0x00000002 shl FSMC_BCR3_CPSIZE_Pos) ## !< 0x00020000
  FSMC_BCR3_CPSIZE_2* = (0x00000004 shl FSMC_BCR3_CPSIZE_Pos) ## !< 0x00040000
  FSMC_BCR3_CBURSTRW_Pos* = (19)
  FSMC_BCR3_CBURSTRW_Msk* = (0x00000001 shl FSMC_BCR3_CBURSTRW_Pos) ## !< 0x00080000
  FSMC_BCR3_CBURSTRW* = FSMC_BCR3_CBURSTRW_Msk

## *****************  Bit definition for FSMC_BCR4 register  ******************

const
  FSMC_BCR4_MBKEN_Pos* = (0)
  FSMC_BCR4_MBKEN_Msk* = (0x00000001 shl FSMC_BCR4_MBKEN_Pos) ## !< 0x00000001
  FSMC_BCR4_MBKEN* = FSMC_BCR4_MBKEN_Msk
  FSMC_BCR4_MUXEN_Pos* = (1)
  FSMC_BCR4_MUXEN_Msk* = (0x00000001 shl FSMC_BCR4_MUXEN_Pos) ## !< 0x00000002
  FSMC_BCR4_MUXEN* = FSMC_BCR4_MUXEN_Msk
  FSMC_BCR4_MTYP_Pos* = (2)
  FSMC_BCR4_MTYP_Msk* = (0x00000003 shl FSMC_BCR4_MTYP_Pos) ## !< 0x0000000C
  FSMC_BCR4_MTYP* = FSMC_BCR4_MTYP_Msk
  FSMC_BCR4_MTYP_0* = (0x00000001 shl FSMC_BCR4_MTYP_Pos) ## !< 0x00000004
  FSMC_BCR4_MTYP_1* = (0x00000002 shl FSMC_BCR4_MTYP_Pos) ## !< 0x00000008
  FSMC_BCR4_MWID_Pos* = (4)
  FSMC_BCR4_MWID_Msk* = (0x00000003 shl FSMC_BCR4_MWID_Pos) ## !< 0x00000030
  FSMC_BCR4_MWID* = FSMC_BCR4_MWID_Msk
  FSMC_BCR4_MWID_0* = (0x00000001 shl FSMC_BCR4_MWID_Pos) ## !< 0x00000010
  FSMC_BCR4_MWID_1* = (0x00000002 shl FSMC_BCR4_MWID_Pos) ## !< 0x00000020
  FSMC_BCR4_FACCEN_Pos* = (6)
  FSMC_BCR4_FACCEN_Msk* = (0x00000001 shl FSMC_BCR4_FACCEN_Pos) ## !< 0x00000040
  FSMC_BCR4_FACCEN* = FSMC_BCR4_FACCEN_Msk
  FSMC_BCR4_BURSTEN_Pos* = (8)
  FSMC_BCR4_BURSTEN_Msk* = (0x00000001 shl FSMC_BCR4_BURSTEN_Pos) ## !< 0x00000100
  FSMC_BCR4_BURSTEN* = FSMC_BCR4_BURSTEN_Msk
  FSMC_BCR4_WAITPOL_Pos* = (9)
  FSMC_BCR4_WAITPOL_Msk* = (0x00000001 shl FSMC_BCR4_WAITPOL_Pos) ## !< 0x00000200
  FSMC_BCR4_WAITPOL* = FSMC_BCR4_WAITPOL_Msk
  FSMC_BCR4_WRAPMOD_Pos* = (10)
  FSMC_BCR4_WRAPMOD_Msk* = (0x00000001 shl FSMC_BCR4_WRAPMOD_Pos) ## !< 0x00000400
  FSMC_BCR4_WRAPMOD* = FSMC_BCR4_WRAPMOD_Msk
  FSMC_BCR4_WAITCFG_Pos* = (11)
  FSMC_BCR4_WAITCFG_Msk* = (0x00000001 shl FSMC_BCR4_WAITCFG_Pos) ## !< 0x00000800
  FSMC_BCR4_WAITCFG* = FSMC_BCR4_WAITCFG_Msk
  FSMC_BCR4_WREN_Pos* = (12)
  FSMC_BCR4_WREN_Msk* = (0x00000001 shl FSMC_BCR4_WREN_Pos) ## !< 0x00001000
  FSMC_BCR4_WREN* = FSMC_BCR4_WREN_Msk
  FSMC_BCR4_WAITEN_Pos* = (13)
  FSMC_BCR4_WAITEN_Msk* = (0x00000001 shl FSMC_BCR4_WAITEN_Pos) ## !< 0x00002000
  FSMC_BCR4_WAITEN* = FSMC_BCR4_WAITEN_Msk
  FSMC_BCR4_EXTMOD_Pos* = (14)
  FSMC_BCR4_EXTMOD_Msk* = (0x00000001 shl FSMC_BCR4_EXTMOD_Pos) ## !< 0x00004000
  FSMC_BCR4_EXTMOD* = FSMC_BCR4_EXTMOD_Msk
  FSMC_BCR4_ASYNCWAIT_Pos* = (15)
  FSMC_BCR4_ASYNCWAIT_Msk* = (0x00000001 shl FSMC_BCR4_ASYNCWAIT_Pos) ## !< 0x00008000
  FSMC_BCR4_ASYNCWAIT* = FSMC_BCR4_ASYNCWAIT_Msk
  FSMC_BCR4_CPSIZE_Pos* = (16)
  FSMC_BCR4_CPSIZE_Msk* = (0x00000007 shl FSMC_BCR4_CPSIZE_Pos) ## !< 0x00070000
  FSMC_BCR4_CPSIZE* = FSMC_BCR4_CPSIZE_Msk
  FSMC_BCR4_CPSIZE_0* = (0x00000001 shl FSMC_BCR4_CPSIZE_Pos) ## !< 0x00010000
  FSMC_BCR4_CPSIZE_1* = (0x00000002 shl FSMC_BCR4_CPSIZE_Pos) ## !< 0x00020000
  FSMC_BCR4_CPSIZE_2* = (0x00000004 shl FSMC_BCR4_CPSIZE_Pos) ## !< 0x00040000
  FSMC_BCR4_CBURSTRW_Pos* = (19)
  FSMC_BCR4_CBURSTRW_Msk* = (0x00000001 shl FSMC_BCR4_CBURSTRW_Pos) ## !< 0x00080000
  FSMC_BCR4_CBURSTRW* = FSMC_BCR4_CBURSTRW_Msk

## *****************  Bit definition for FSMC_BTR1 register  *****************

const
  FSMC_BTR1_ADDSET_Pos* = (0)
  FSMC_BTR1_ADDSET_Msk* = (0x0000000F shl FSMC_BTR1_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BTR1_ADDSET* = FSMC_BTR1_ADDSET_Msk
  FSMC_BTR1_ADDSET_0* = (0x00000001 shl FSMC_BTR1_ADDSET_Pos) ## !< 0x00000001
  FSMC_BTR1_ADDSET_1* = (0x00000002 shl FSMC_BTR1_ADDSET_Pos) ## !< 0x00000002
  FSMC_BTR1_ADDSET_2* = (0x00000004 shl FSMC_BTR1_ADDSET_Pos) ## !< 0x00000004
  FSMC_BTR1_ADDSET_3* = (0x00000008 shl FSMC_BTR1_ADDSET_Pos) ## !< 0x00000008
  FSMC_BTR1_ADDHLD_Pos* = (4)
  FSMC_BTR1_ADDHLD_Msk* = (0x0000000F shl FSMC_BTR1_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BTR1_ADDHLD* = FSMC_BTR1_ADDHLD_Msk
  FSMC_BTR1_ADDHLD_0* = (0x00000001 shl FSMC_BTR1_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BTR1_ADDHLD_1* = (0x00000002 shl FSMC_BTR1_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BTR1_ADDHLD_2* = (0x00000004 shl FSMC_BTR1_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BTR1_ADDHLD_3* = (0x00000008 shl FSMC_BTR1_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BTR1_DATAST_Pos* = (8)
  FSMC_BTR1_DATAST_Msk* = (0x000000FF shl FSMC_BTR1_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BTR1_DATAST* = FSMC_BTR1_DATAST_Msk
  FSMC_BTR1_DATAST_0* = (0x00000001 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00000100
  FSMC_BTR1_DATAST_1* = (0x00000002 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00000200
  FSMC_BTR1_DATAST_2* = (0x00000004 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00000400
  FSMC_BTR1_DATAST_3* = (0x00000008 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00000800
  FSMC_BTR1_DATAST_4* = (0x00000010 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00001000
  FSMC_BTR1_DATAST_5* = (0x00000020 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00002000
  FSMC_BTR1_DATAST_6* = (0x00000040 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00004000
  FSMC_BTR1_DATAST_7* = (0x00000080 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00008000
  FSMC_BTR1_BUSTURN_Pos* = (16)
  FSMC_BTR1_BUSTURN_Msk* = (0x0000000F shl FSMC_BTR1_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BTR1_BUSTURN* = FSMC_BTR1_BUSTURN_Msk
  FSMC_BTR1_BUSTURN_0* = (0x00000001 shl FSMC_BTR1_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BTR1_BUSTURN_1* = (0x00000002 shl FSMC_BTR1_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BTR1_BUSTURN_2* = (0x00000004 shl FSMC_BTR1_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BTR1_BUSTURN_3* = (0x00000008 shl FSMC_BTR1_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BTR1_CLKDIV_Pos* = (20)
  FSMC_BTR1_CLKDIV_Msk* = (0x0000000F shl FSMC_BTR1_CLKDIV_Pos) ## !< 0x00F00000
  FSMC_BTR1_CLKDIV* = FSMC_BTR1_CLKDIV_Msk
  FSMC_BTR1_CLKDIV_0* = (0x00000001 shl FSMC_BTR1_CLKDIV_Pos) ## !< 0x00100000
  FSMC_BTR1_CLKDIV_1* = (0x00000002 shl FSMC_BTR1_CLKDIV_Pos) ## !< 0x00200000
  FSMC_BTR1_CLKDIV_2* = (0x00000004 shl FSMC_BTR1_CLKDIV_Pos) ## !< 0x00400000
  FSMC_BTR1_CLKDIV_3* = (0x00000008 shl FSMC_BTR1_CLKDIV_Pos) ## !< 0x00800000
  FSMC_BTR1_DATLAT_Pos* = (24)
  FSMC_BTR1_DATLAT_Msk* = (0x0000000F shl FSMC_BTR1_DATLAT_Pos) ## !< 0x0F000000
  FSMC_BTR1_DATLAT* = FSMC_BTR1_DATLAT_Msk
  FSMC_BTR1_DATLAT_0* = (0x00000001 shl FSMC_BTR1_DATLAT_Pos) ## !< 0x01000000
  FSMC_BTR1_DATLAT_1* = (0x00000002 shl FSMC_BTR1_DATLAT_Pos) ## !< 0x02000000
  FSMC_BTR1_DATLAT_2* = (0x00000004 shl FSMC_BTR1_DATLAT_Pos) ## !< 0x04000000
  FSMC_BTR1_DATLAT_3* = (0x00000008 shl FSMC_BTR1_DATLAT_Pos) ## !< 0x08000000
  FSMC_BTR1_ACCMOD_Pos* = (28)
  FSMC_BTR1_ACCMOD_Msk* = (0x00000003 shl FSMC_BTR1_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BTR1_ACCMOD* = FSMC_BTR1_ACCMOD_Msk
  FSMC_BTR1_ACCMOD_0* = (0x00000001 shl FSMC_BTR1_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BTR1_ACCMOD_1* = (0x00000002 shl FSMC_BTR1_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FSMC_BTR2 register  ******************

const
  FSMC_BTR2_ADDSET_Pos* = (0)
  FSMC_BTR2_ADDSET_Msk* = (0x0000000F shl FSMC_BTR2_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BTR2_ADDSET* = FSMC_BTR2_ADDSET_Msk
  FSMC_BTR2_ADDSET_0* = (0x00000001 shl FSMC_BTR2_ADDSET_Pos) ## !< 0x00000001
  FSMC_BTR2_ADDSET_1* = (0x00000002 shl FSMC_BTR2_ADDSET_Pos) ## !< 0x00000002
  FSMC_BTR2_ADDSET_2* = (0x00000004 shl FSMC_BTR2_ADDSET_Pos) ## !< 0x00000004
  FSMC_BTR2_ADDSET_3* = (0x00000008 shl FSMC_BTR2_ADDSET_Pos) ## !< 0x00000008
  FSMC_BTR2_ADDHLD_Pos* = (4)
  FSMC_BTR2_ADDHLD_Msk* = (0x0000000F shl FSMC_BTR2_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BTR2_ADDHLD* = FSMC_BTR2_ADDHLD_Msk
  FSMC_BTR2_ADDHLD_0* = (0x00000001 shl FSMC_BTR2_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BTR2_ADDHLD_1* = (0x00000002 shl FSMC_BTR2_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BTR2_ADDHLD_2* = (0x00000004 shl FSMC_BTR2_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BTR2_ADDHLD_3* = (0x00000008 shl FSMC_BTR2_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BTR2_DATAST_Pos* = (8)
  FSMC_BTR2_DATAST_Msk* = (0x000000FF shl FSMC_BTR2_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BTR2_DATAST* = FSMC_BTR2_DATAST_Msk
  FSMC_BTR2_DATAST_0* = (0x00000001 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00000100
  FSMC_BTR2_DATAST_1* = (0x00000002 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00000200
  FSMC_BTR2_DATAST_2* = (0x00000004 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00000400
  FSMC_BTR2_DATAST_3* = (0x00000008 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00000800
  FSMC_BTR2_DATAST_4* = (0x00000010 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00001000
  FSMC_BTR2_DATAST_5* = (0x00000020 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00002000
  FSMC_BTR2_DATAST_6* = (0x00000040 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00004000
  FSMC_BTR2_DATAST_7* = (0x00000080 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00008000
  FSMC_BTR2_BUSTURN_Pos* = (16)
  FSMC_BTR2_BUSTURN_Msk* = (0x0000000F shl FSMC_BTR2_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BTR2_BUSTURN* = FSMC_BTR2_BUSTURN_Msk
  FSMC_BTR2_BUSTURN_0* = (0x00000001 shl FSMC_BTR2_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BTR2_BUSTURN_1* = (0x00000002 shl FSMC_BTR2_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BTR2_BUSTURN_2* = (0x00000004 shl FSMC_BTR2_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BTR2_BUSTURN_3* = (0x00000008 shl FSMC_BTR2_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BTR2_CLKDIV_Pos* = (20)
  FSMC_BTR2_CLKDIV_Msk* = (0x0000000F shl FSMC_BTR2_CLKDIV_Pos) ## !< 0x00F00000
  FSMC_BTR2_CLKDIV* = FSMC_BTR2_CLKDIV_Msk
  FSMC_BTR2_CLKDIV_0* = (0x00000001 shl FSMC_BTR2_CLKDIV_Pos) ## !< 0x00100000
  FSMC_BTR2_CLKDIV_1* = (0x00000002 shl FSMC_BTR2_CLKDIV_Pos) ## !< 0x00200000
  FSMC_BTR2_CLKDIV_2* = (0x00000004 shl FSMC_BTR2_CLKDIV_Pos) ## !< 0x00400000
  FSMC_BTR2_CLKDIV_3* = (0x00000008 shl FSMC_BTR2_CLKDIV_Pos) ## !< 0x00800000
  FSMC_BTR2_DATLAT_Pos* = (24)
  FSMC_BTR2_DATLAT_Msk* = (0x0000000F shl FSMC_BTR2_DATLAT_Pos) ## !< 0x0F000000
  FSMC_BTR2_DATLAT* = FSMC_BTR2_DATLAT_Msk
  FSMC_BTR2_DATLAT_0* = (0x00000001 shl FSMC_BTR2_DATLAT_Pos) ## !< 0x01000000
  FSMC_BTR2_DATLAT_1* = (0x00000002 shl FSMC_BTR2_DATLAT_Pos) ## !< 0x02000000
  FSMC_BTR2_DATLAT_2* = (0x00000004 shl FSMC_BTR2_DATLAT_Pos) ## !< 0x04000000
  FSMC_BTR2_DATLAT_3* = (0x00000008 shl FSMC_BTR2_DATLAT_Pos) ## !< 0x08000000
  FSMC_BTR2_ACCMOD_Pos* = (28)
  FSMC_BTR2_ACCMOD_Msk* = (0x00000003 shl FSMC_BTR2_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BTR2_ACCMOD* = FSMC_BTR2_ACCMOD_Msk
  FSMC_BTR2_ACCMOD_0* = (0x00000001 shl FSMC_BTR2_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BTR2_ACCMOD_1* = (0x00000002 shl FSMC_BTR2_ACCMOD_Pos) ## !< 0x20000000

## ******************  Bit definition for FSMC_BTR3 register  ******************

const
  FSMC_BTR3_ADDSET_Pos* = (0)
  FSMC_BTR3_ADDSET_Msk* = (0x0000000F shl FSMC_BTR3_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BTR3_ADDSET* = FSMC_BTR3_ADDSET_Msk
  FSMC_BTR3_ADDSET_0* = (0x00000001 shl FSMC_BTR3_ADDSET_Pos) ## !< 0x00000001
  FSMC_BTR3_ADDSET_1* = (0x00000002 shl FSMC_BTR3_ADDSET_Pos) ## !< 0x00000002
  FSMC_BTR3_ADDSET_2* = (0x00000004 shl FSMC_BTR3_ADDSET_Pos) ## !< 0x00000004
  FSMC_BTR3_ADDSET_3* = (0x00000008 shl FSMC_BTR3_ADDSET_Pos) ## !< 0x00000008
  FSMC_BTR3_ADDHLD_Pos* = (4)
  FSMC_BTR3_ADDHLD_Msk* = (0x0000000F shl FSMC_BTR3_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BTR3_ADDHLD* = FSMC_BTR3_ADDHLD_Msk
  FSMC_BTR3_ADDHLD_0* = (0x00000001 shl FSMC_BTR3_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BTR3_ADDHLD_1* = (0x00000002 shl FSMC_BTR3_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BTR3_ADDHLD_2* = (0x00000004 shl FSMC_BTR3_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BTR3_ADDHLD_3* = (0x00000008 shl FSMC_BTR3_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BTR3_DATAST_Pos* = (8)
  FSMC_BTR3_DATAST_Msk* = (0x000000FF shl FSMC_BTR3_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BTR3_DATAST* = FSMC_BTR3_DATAST_Msk
  FSMC_BTR3_DATAST_0* = (0x00000001 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00000100
  FSMC_BTR3_DATAST_1* = (0x00000002 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00000200
  FSMC_BTR3_DATAST_2* = (0x00000004 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00000400
  FSMC_BTR3_DATAST_3* = (0x00000008 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00000800
  FSMC_BTR3_DATAST_4* = (0x00000010 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00001000
  FSMC_BTR3_DATAST_5* = (0x00000020 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00002000
  FSMC_BTR3_DATAST_6* = (0x00000040 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00004000
  FSMC_BTR3_DATAST_7* = (0x00000080 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00008000
  FSMC_BTR3_BUSTURN_Pos* = (16)
  FSMC_BTR3_BUSTURN_Msk* = (0x0000000F shl FSMC_BTR3_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BTR3_BUSTURN* = FSMC_BTR3_BUSTURN_Msk
  FSMC_BTR3_BUSTURN_0* = (0x00000001 shl FSMC_BTR3_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BTR3_BUSTURN_1* = (0x00000002 shl FSMC_BTR3_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BTR3_BUSTURN_2* = (0x00000004 shl FSMC_BTR3_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BTR3_BUSTURN_3* = (0x00000008 shl FSMC_BTR3_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BTR3_CLKDIV_Pos* = (20)
  FSMC_BTR3_CLKDIV_Msk* = (0x0000000F shl FSMC_BTR3_CLKDIV_Pos) ## !< 0x00F00000
  FSMC_BTR3_CLKDIV* = FSMC_BTR3_CLKDIV_Msk
  FSMC_BTR3_CLKDIV_0* = (0x00000001 shl FSMC_BTR3_CLKDIV_Pos) ## !< 0x00100000
  FSMC_BTR3_CLKDIV_1* = (0x00000002 shl FSMC_BTR3_CLKDIV_Pos) ## !< 0x00200000
  FSMC_BTR3_CLKDIV_2* = (0x00000004 shl FSMC_BTR3_CLKDIV_Pos) ## !< 0x00400000
  FSMC_BTR3_CLKDIV_3* = (0x00000008 shl FSMC_BTR3_CLKDIV_Pos) ## !< 0x00800000
  FSMC_BTR3_DATLAT_Pos* = (24)
  FSMC_BTR3_DATLAT_Msk* = (0x0000000F shl FSMC_BTR3_DATLAT_Pos) ## !< 0x0F000000
  FSMC_BTR3_DATLAT* = FSMC_BTR3_DATLAT_Msk
  FSMC_BTR3_DATLAT_0* = (0x00000001 shl FSMC_BTR3_DATLAT_Pos) ## !< 0x01000000
  FSMC_BTR3_DATLAT_1* = (0x00000002 shl FSMC_BTR3_DATLAT_Pos) ## !< 0x02000000
  FSMC_BTR3_DATLAT_2* = (0x00000004 shl FSMC_BTR3_DATLAT_Pos) ## !< 0x04000000
  FSMC_BTR3_DATLAT_3* = (0x00000008 shl FSMC_BTR3_DATLAT_Pos) ## !< 0x08000000
  FSMC_BTR3_ACCMOD_Pos* = (28)
  FSMC_BTR3_ACCMOD_Msk* = (0x00000003 shl FSMC_BTR3_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BTR3_ACCMOD* = FSMC_BTR3_ACCMOD_Msk
  FSMC_BTR3_ACCMOD_0* = (0x00000001 shl FSMC_BTR3_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BTR3_ACCMOD_1* = (0x00000002 shl FSMC_BTR3_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FSMC_BTR4 register  ******************

const
  FSMC_BTR4_ADDSET_Pos* = (0)
  FSMC_BTR4_ADDSET_Msk* = (0x0000000F shl FSMC_BTR4_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BTR4_ADDSET* = FSMC_BTR4_ADDSET_Msk
  FSMC_BTR4_ADDSET_0* = (0x00000001 shl FSMC_BTR4_ADDSET_Pos) ## !< 0x00000001
  FSMC_BTR4_ADDSET_1* = (0x00000002 shl FSMC_BTR4_ADDSET_Pos) ## !< 0x00000002
  FSMC_BTR4_ADDSET_2* = (0x00000004 shl FSMC_BTR4_ADDSET_Pos) ## !< 0x00000004
  FSMC_BTR4_ADDSET_3* = (0x00000008 shl FSMC_BTR4_ADDSET_Pos) ## !< 0x00000008
  FSMC_BTR4_ADDHLD_Pos* = (4)
  FSMC_BTR4_ADDHLD_Msk* = (0x0000000F shl FSMC_BTR4_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BTR4_ADDHLD* = FSMC_BTR4_ADDHLD_Msk
  FSMC_BTR4_ADDHLD_0* = (0x00000001 shl FSMC_BTR4_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BTR4_ADDHLD_1* = (0x00000002 shl FSMC_BTR4_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BTR4_ADDHLD_2* = (0x00000004 shl FSMC_BTR4_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BTR4_ADDHLD_3* = (0x00000008 shl FSMC_BTR4_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BTR4_DATAST_Pos* = (8)
  FSMC_BTR4_DATAST_Msk* = (0x000000FF shl FSMC_BTR4_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BTR4_DATAST* = FSMC_BTR4_DATAST_Msk
  FSMC_BTR4_DATAST_0* = (0x00000001 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00000100
  FSMC_BTR4_DATAST_1* = (0x00000002 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00000200
  FSMC_BTR4_DATAST_2* = (0x00000004 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00000400
  FSMC_BTR4_DATAST_3* = (0x00000008 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00000800
  FSMC_BTR4_DATAST_4* = (0x00000010 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00001000
  FSMC_BTR4_DATAST_5* = (0x00000020 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00002000
  FSMC_BTR4_DATAST_6* = (0x00000040 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00004000
  FSMC_BTR4_DATAST_7* = (0x00000080 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00008000
  FSMC_BTR4_BUSTURN_Pos* = (16)
  FSMC_BTR4_BUSTURN_Msk* = (0x0000000F shl FSMC_BTR4_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BTR4_BUSTURN* = FSMC_BTR4_BUSTURN_Msk
  FSMC_BTR4_BUSTURN_0* = (0x00000001 shl FSMC_BTR4_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BTR4_BUSTURN_1* = (0x00000002 shl FSMC_BTR4_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BTR4_BUSTURN_2* = (0x00000004 shl FSMC_BTR4_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BTR4_BUSTURN_3* = (0x00000008 shl FSMC_BTR4_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BTR4_CLKDIV_Pos* = (20)
  FSMC_BTR4_CLKDIV_Msk* = (0x0000000F shl FSMC_BTR4_CLKDIV_Pos) ## !< 0x00F00000
  FSMC_BTR4_CLKDIV* = FSMC_BTR4_CLKDIV_Msk
  FSMC_BTR4_CLKDIV_0* = (0x00000001 shl FSMC_BTR4_CLKDIV_Pos) ## !< 0x00100000
  FSMC_BTR4_CLKDIV_1* = (0x00000002 shl FSMC_BTR4_CLKDIV_Pos) ## !< 0x00200000
  FSMC_BTR4_CLKDIV_2* = (0x00000004 shl FSMC_BTR4_CLKDIV_Pos) ## !< 0x00400000
  FSMC_BTR4_CLKDIV_3* = (0x00000008 shl FSMC_BTR4_CLKDIV_Pos) ## !< 0x00800000
  FSMC_BTR4_DATLAT_Pos* = (24)
  FSMC_BTR4_DATLAT_Msk* = (0x0000000F shl FSMC_BTR4_DATLAT_Pos) ## !< 0x0F000000
  FSMC_BTR4_DATLAT* = FSMC_BTR4_DATLAT_Msk
  FSMC_BTR4_DATLAT_0* = (0x00000001 shl FSMC_BTR4_DATLAT_Pos) ## !< 0x01000000
  FSMC_BTR4_DATLAT_1* = (0x00000002 shl FSMC_BTR4_DATLAT_Pos) ## !< 0x02000000
  FSMC_BTR4_DATLAT_2* = (0x00000004 shl FSMC_BTR4_DATLAT_Pos) ## !< 0x04000000
  FSMC_BTR4_DATLAT_3* = (0x00000008 shl FSMC_BTR4_DATLAT_Pos) ## !< 0x08000000
  FSMC_BTR4_ACCMOD_Pos* = (28)
  FSMC_BTR4_ACCMOD_Msk* = (0x00000003 shl FSMC_BTR4_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BTR4_ACCMOD* = FSMC_BTR4_ACCMOD_Msk
  FSMC_BTR4_ACCMOD_0* = (0x00000001 shl FSMC_BTR4_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BTR4_ACCMOD_1* = (0x00000002 shl FSMC_BTR4_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FSMC_BWTR1 register  *****************

const
  FSMC_BWTR1_ADDSET_Pos* = (0)
  FSMC_BWTR1_ADDSET_Msk* = (0x0000000F shl FSMC_BWTR1_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BWTR1_ADDSET* = FSMC_BWTR1_ADDSET_Msk
  FSMC_BWTR1_ADDSET_0* = (0x00000001 shl FSMC_BWTR1_ADDSET_Pos) ## !< 0x00000001
  FSMC_BWTR1_ADDSET_1* = (0x00000002 shl FSMC_BWTR1_ADDSET_Pos) ## !< 0x00000002
  FSMC_BWTR1_ADDSET_2* = (0x00000004 shl FSMC_BWTR1_ADDSET_Pos) ## !< 0x00000004
  FSMC_BWTR1_ADDSET_3* = (0x00000008 shl FSMC_BWTR1_ADDSET_Pos) ## !< 0x00000008
  FSMC_BWTR1_ADDHLD_Pos* = (4)
  FSMC_BWTR1_ADDHLD_Msk* = (0x0000000F shl FSMC_BWTR1_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BWTR1_ADDHLD* = FSMC_BWTR1_ADDHLD_Msk
  FSMC_BWTR1_ADDHLD_0* = (0x00000001 shl FSMC_BWTR1_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BWTR1_ADDHLD_1* = (0x00000002 shl FSMC_BWTR1_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BWTR1_ADDHLD_2* = (0x00000004 shl FSMC_BWTR1_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BWTR1_ADDHLD_3* = (0x00000008 shl FSMC_BWTR1_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BWTR1_DATAST_Pos* = (8)
  FSMC_BWTR1_DATAST_Msk* = (0x000000FF shl FSMC_BWTR1_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BWTR1_DATAST* = FSMC_BWTR1_DATAST_Msk
  FSMC_BWTR1_DATAST_0* = (0x00000001 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00000100
  FSMC_BWTR1_DATAST_1* = (0x00000002 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00000200
  FSMC_BWTR1_DATAST_2* = (0x00000004 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00000400
  FSMC_BWTR1_DATAST_3* = (0x00000008 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00000800
  FSMC_BWTR1_DATAST_4* = (0x00000010 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00001000
  FSMC_BWTR1_DATAST_5* = (0x00000020 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00002000
  FSMC_BWTR1_DATAST_6* = (0x00000040 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00004000
  FSMC_BWTR1_DATAST_7* = (0x00000080 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00008000
  FSMC_BWTR1_BUSTURN_Pos* = (16)
  FSMC_BWTR1_BUSTURN_Msk* = (0x0000000F shl FSMC_BWTR1_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BWTR1_BUSTURN* = FSMC_BWTR1_BUSTURN_Msk
  FSMC_BWTR1_BUSTURN_0* = (0x00000001 shl FSMC_BWTR1_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BWTR1_BUSTURN_1* = (0x00000002 shl FSMC_BWTR1_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BWTR1_BUSTURN_2* = (0x00000004 shl FSMC_BWTR1_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BWTR1_BUSTURN_3* = (0x00000008 shl FSMC_BWTR1_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BWTR1_ACCMOD_Pos* = (28)
  FSMC_BWTR1_ACCMOD_Msk* = (0x00000003 shl FSMC_BWTR1_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BWTR1_ACCMOD* = FSMC_BWTR1_ACCMOD_Msk
  FSMC_BWTR1_ACCMOD_0* = (0x00000001 shl FSMC_BWTR1_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BWTR1_ACCMOD_1* = (0x00000002 shl FSMC_BWTR1_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FSMC_BWTR2 register  *****************

const
  FSMC_BWTR2_ADDSET_Pos* = (0)
  FSMC_BWTR2_ADDSET_Msk* = (0x0000000F shl FSMC_BWTR2_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BWTR2_ADDSET* = FSMC_BWTR2_ADDSET_Msk
  FSMC_BWTR2_ADDSET_0* = (0x00000001 shl FSMC_BWTR2_ADDSET_Pos) ## !< 0x00000001
  FSMC_BWTR2_ADDSET_1* = (0x00000002 shl FSMC_BWTR2_ADDSET_Pos) ## !< 0x00000002
  FSMC_BWTR2_ADDSET_2* = (0x00000004 shl FSMC_BWTR2_ADDSET_Pos) ## !< 0x00000004
  FSMC_BWTR2_ADDSET_3* = (0x00000008 shl FSMC_BWTR2_ADDSET_Pos) ## !< 0x00000008
  FSMC_BWTR2_ADDHLD_Pos* = (4)
  FSMC_BWTR2_ADDHLD_Msk* = (0x0000000F shl FSMC_BWTR2_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BWTR2_ADDHLD* = FSMC_BWTR2_ADDHLD_Msk
  FSMC_BWTR2_ADDHLD_0* = (0x00000001 shl FSMC_BWTR2_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BWTR2_ADDHLD_1* = (0x00000002 shl FSMC_BWTR2_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BWTR2_ADDHLD_2* = (0x00000004 shl FSMC_BWTR2_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BWTR2_ADDHLD_3* = (0x00000008 shl FSMC_BWTR2_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BWTR2_DATAST_Pos* = (8)
  FSMC_BWTR2_DATAST_Msk* = (0x000000FF shl FSMC_BWTR2_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BWTR2_DATAST* = FSMC_BWTR2_DATAST_Msk
  FSMC_BWTR2_DATAST_0* = (0x00000001 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00000100
  FSMC_BWTR2_DATAST_1* = (0x00000002 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00000200
  FSMC_BWTR2_DATAST_2* = (0x00000004 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00000400
  FSMC_BWTR2_DATAST_3* = (0x00000008 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00000800
  FSMC_BWTR2_DATAST_4* = (0x00000010 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00001000
  FSMC_BWTR2_DATAST_5* = (0x00000020 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00002000
  FSMC_BWTR2_DATAST_6* = (0x00000040 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00004000
  FSMC_BWTR2_DATAST_7* = (0x00000080 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00008000
  FSMC_BWTR2_BUSTURN_Pos* = (16)
  FSMC_BWTR2_BUSTURN_Msk* = (0x0000000F shl FSMC_BWTR2_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BWTR2_BUSTURN* = FSMC_BWTR2_BUSTURN_Msk
  FSMC_BWTR2_BUSTURN_0* = (0x00000001 shl FSMC_BWTR2_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BWTR2_BUSTURN_1* = (0x00000002 shl FSMC_BWTR2_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BWTR2_BUSTURN_2* = (0x00000004 shl FSMC_BWTR2_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BWTR2_BUSTURN_3* = (0x00000008 shl FSMC_BWTR2_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BWTR2_ACCMOD_Pos* = (28)
  FSMC_BWTR2_ACCMOD_Msk* = (0x00000003 shl FSMC_BWTR2_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BWTR2_ACCMOD* = FSMC_BWTR2_ACCMOD_Msk
  FSMC_BWTR2_ACCMOD_0* = (0x00000001 shl FSMC_BWTR2_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BWTR2_ACCMOD_1* = (0x00000002 shl FSMC_BWTR2_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FSMC_BWTR3 register  *****************

const
  FSMC_BWTR3_ADDSET_Pos* = (0)
  FSMC_BWTR3_ADDSET_Msk* = (0x0000000F shl FSMC_BWTR3_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BWTR3_ADDSET* = FSMC_BWTR3_ADDSET_Msk
  FSMC_BWTR3_ADDSET_0* = (0x00000001 shl FSMC_BWTR3_ADDSET_Pos) ## !< 0x00000001
  FSMC_BWTR3_ADDSET_1* = (0x00000002 shl FSMC_BWTR3_ADDSET_Pos) ## !< 0x00000002
  FSMC_BWTR3_ADDSET_2* = (0x00000004 shl FSMC_BWTR3_ADDSET_Pos) ## !< 0x00000004
  FSMC_BWTR3_ADDSET_3* = (0x00000008 shl FSMC_BWTR3_ADDSET_Pos) ## !< 0x00000008
  FSMC_BWTR3_ADDHLD_Pos* = (4)
  FSMC_BWTR3_ADDHLD_Msk* = (0x0000000F shl FSMC_BWTR3_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BWTR3_ADDHLD* = FSMC_BWTR3_ADDHLD_Msk
  FSMC_BWTR3_ADDHLD_0* = (0x00000001 shl FSMC_BWTR3_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BWTR3_ADDHLD_1* = (0x00000002 shl FSMC_BWTR3_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BWTR3_ADDHLD_2* = (0x00000004 shl FSMC_BWTR3_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BWTR3_ADDHLD_3* = (0x00000008 shl FSMC_BWTR3_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BWTR3_DATAST_Pos* = (8)
  FSMC_BWTR3_DATAST_Msk* = (0x000000FF shl FSMC_BWTR3_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BWTR3_DATAST* = FSMC_BWTR3_DATAST_Msk
  FSMC_BWTR3_DATAST_0* = (0x00000001 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00000100
  FSMC_BWTR3_DATAST_1* = (0x00000002 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00000200
  FSMC_BWTR3_DATAST_2* = (0x00000004 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00000400
  FSMC_BWTR3_DATAST_3* = (0x00000008 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00000800
  FSMC_BWTR3_DATAST_4* = (0x00000010 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00001000
  FSMC_BWTR3_DATAST_5* = (0x00000020 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00002000
  FSMC_BWTR3_DATAST_6* = (0x00000040 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00004000
  FSMC_BWTR3_DATAST_7* = (0x00000080 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00008000
  FSMC_BWTR3_BUSTURN_Pos* = (16)
  FSMC_BWTR3_BUSTURN_Msk* = (0x0000000F shl FSMC_BWTR3_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BWTR3_BUSTURN* = FSMC_BWTR3_BUSTURN_Msk
  FSMC_BWTR3_BUSTURN_0* = (0x00000001 shl FSMC_BWTR3_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BWTR3_BUSTURN_1* = (0x00000002 shl FSMC_BWTR3_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BWTR3_BUSTURN_2* = (0x00000004 shl FSMC_BWTR3_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BWTR3_BUSTURN_3* = (0x00000008 shl FSMC_BWTR3_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BWTR3_ACCMOD_Pos* = (28)
  FSMC_BWTR3_ACCMOD_Msk* = (0x00000003 shl FSMC_BWTR3_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BWTR3_ACCMOD* = FSMC_BWTR3_ACCMOD_Msk
  FSMC_BWTR3_ACCMOD_0* = (0x00000001 shl FSMC_BWTR3_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BWTR3_ACCMOD_1* = (0x00000002 shl FSMC_BWTR3_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FSMC_BWTR4 register  *****************

const
  FSMC_BWTR4_ADDSET_Pos* = (0)
  FSMC_BWTR4_ADDSET_Msk* = (0x0000000F shl FSMC_BWTR4_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BWTR4_ADDSET* = FSMC_BWTR4_ADDSET_Msk
  FSMC_BWTR4_ADDSET_0* = (0x00000001 shl FSMC_BWTR4_ADDSET_Pos) ## !< 0x00000001
  FSMC_BWTR4_ADDSET_1* = (0x00000002 shl FSMC_BWTR4_ADDSET_Pos) ## !< 0x00000002
  FSMC_BWTR4_ADDSET_2* = (0x00000004 shl FSMC_BWTR4_ADDSET_Pos) ## !< 0x00000004
  FSMC_BWTR4_ADDSET_3* = (0x00000008 shl FSMC_BWTR4_ADDSET_Pos) ## !< 0x00000008
  FSMC_BWTR4_ADDHLD_Pos* = (4)
  FSMC_BWTR4_ADDHLD_Msk* = (0x0000000F shl FSMC_BWTR4_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BWTR4_ADDHLD* = FSMC_BWTR4_ADDHLD_Msk
  FSMC_BWTR4_ADDHLD_0* = (0x00000001 shl FSMC_BWTR4_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BWTR4_ADDHLD_1* = (0x00000002 shl FSMC_BWTR4_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BWTR4_ADDHLD_2* = (0x00000004 shl FSMC_BWTR4_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BWTR4_ADDHLD_3* = (0x00000008 shl FSMC_BWTR4_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BWTR4_DATAST_Pos* = (8)
  FSMC_BWTR4_DATAST_Msk* = (0x000000FF shl FSMC_BWTR4_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BWTR4_DATAST* = FSMC_BWTR4_DATAST_Msk
  FSMC_BWTR4_DATAST_0* = 0x00000100
  FSMC_BWTR4_DATAST_1* = 0x00000200
  FSMC_BWTR4_DATAST_2* = 0x00000400
  FSMC_BWTR4_DATAST_3* = 0x00000800
  FSMC_BWTR4_DATAST_4* = 0x00001000
  FSMC_BWTR4_DATAST_5* = 0x00002000
  FSMC_BWTR4_DATAST_6* = 0x00004000
  FSMC_BWTR4_DATAST_7* = 0x00008000
  FSMC_BWTR4_BUSTURN_Pos* = (16)
  FSMC_BWTR4_BUSTURN_Msk* = (0x0000000F shl FSMC_BWTR4_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BWTR4_BUSTURN* = FSMC_BWTR4_BUSTURN_Msk
  FSMC_BWTR4_BUSTURN_0* = (0x00000001 shl FSMC_BWTR4_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BWTR4_BUSTURN_1* = (0x00000002 shl FSMC_BWTR4_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BWTR4_BUSTURN_2* = (0x00000004 shl FSMC_BWTR4_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BWTR4_BUSTURN_3* = (0x00000008 shl FSMC_BWTR4_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BWTR4_ACCMOD_Pos* = (28)
  FSMC_BWTR4_ACCMOD_Msk* = (0x00000003 shl FSMC_BWTR4_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BWTR4_ACCMOD* = FSMC_BWTR4_ACCMOD_Msk
  FSMC_BWTR4_ACCMOD_0* = (0x00000001 shl FSMC_BWTR4_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BWTR4_ACCMOD_1* = (0x00000002 shl FSMC_BWTR4_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FSMC_PCR2 register  ******************

const
  FSMC_PCR2_PWAITEN_Pos* = (1)
  FSMC_PCR2_PWAITEN_Msk* = (0x00000001 shl FSMC_PCR2_PWAITEN_Pos) ## !< 0x00000002
  FSMC_PCR2_PWAITEN* = FSMC_PCR2_PWAITEN_Msk
  FSMC_PCR2_PBKEN_Pos* = (2)
  FSMC_PCR2_PBKEN_Msk* = (0x00000001 shl FSMC_PCR2_PBKEN_Pos) ## !< 0x00000004
  FSMC_PCR2_PBKEN* = FSMC_PCR2_PBKEN_Msk
  FSMC_PCR2_PTYP_Pos* = (3)
  FSMC_PCR2_PTYP_Msk* = (0x00000001 shl FSMC_PCR2_PTYP_Pos) ## !< 0x00000008
  FSMC_PCR2_PTYP* = FSMC_PCR2_PTYP_Msk
  FSMC_PCR2_PWID_Pos* = (4)
  FSMC_PCR2_PWID_Msk* = (0x00000003 shl FSMC_PCR2_PWID_Pos) ## !< 0x00000030
  FSMC_PCR2_PWID* = FSMC_PCR2_PWID_Msk
  FSMC_PCR2_PWID_0* = (0x00000001 shl FSMC_PCR2_PWID_Pos) ## !< 0x00000010
  FSMC_PCR2_PWID_1* = (0x00000002 shl FSMC_PCR2_PWID_Pos) ## !< 0x00000020
  FSMC_PCR2_ECCEN_Pos* = (6)
  FSMC_PCR2_ECCEN_Msk* = (0x00000001 shl FSMC_PCR2_ECCEN_Pos) ## !< 0x00000040
  FSMC_PCR2_ECCEN* = FSMC_PCR2_ECCEN_Msk
  FSMC_PCR2_TCLR_Pos* = (9)
  FSMC_PCR2_TCLR_Msk* = (0x0000000F shl FSMC_PCR2_TCLR_Pos) ## !< 0x00001E00
  FSMC_PCR2_TCLR* = FSMC_PCR2_TCLR_Msk
  FSMC_PCR2_TCLR_0* = (0x00000001 shl FSMC_PCR2_TCLR_Pos) ## !< 0x00000200
  FSMC_PCR2_TCLR_1* = (0x00000002 shl FSMC_PCR2_TCLR_Pos) ## !< 0x00000400
  FSMC_PCR2_TCLR_2* = (0x00000004 shl FSMC_PCR2_TCLR_Pos) ## !< 0x00000800
  FSMC_PCR2_TCLR_3* = (0x00000008 shl FSMC_PCR2_TCLR_Pos) ## !< 0x00001000
  FSMC_PCR2_TAR_Pos* = (13)
  FSMC_PCR2_TAR_Msk* = (0x0000000F shl FSMC_PCR2_TAR_Pos) ## !< 0x0001E000
  FSMC_PCR2_TAR* = FSMC_PCR2_TAR_Msk
  FSMC_PCR2_TAR_0* = (0x00000001 shl FSMC_PCR2_TAR_Pos) ## !< 0x00002000
  FSMC_PCR2_TAR_1* = (0x00000002 shl FSMC_PCR2_TAR_Pos) ## !< 0x00004000
  FSMC_PCR2_TAR_2* = (0x00000004 shl FSMC_PCR2_TAR_Pos) ## !< 0x00008000
  FSMC_PCR2_TAR_3* = (0x00000008 shl FSMC_PCR2_TAR_Pos) ## !< 0x00010000
  FSMC_PCR2_ECCPS_Pos* = (17)
  FSMC_PCR2_ECCPS_Msk* = (0x00000007 shl FSMC_PCR2_ECCPS_Pos) ## !< 0x000E0000
  FSMC_PCR2_ECCPS* = FSMC_PCR2_ECCPS_Msk
  FSMC_PCR2_ECCPS_0* = (0x00000001 shl FSMC_PCR2_ECCPS_Pos) ## !< 0x00020000
  FSMC_PCR2_ECCPS_1* = (0x00000002 shl FSMC_PCR2_ECCPS_Pos) ## !< 0x00040000
  FSMC_PCR2_ECCPS_2* = (0x00000004 shl FSMC_PCR2_ECCPS_Pos) ## !< 0x00080000

## *****************  Bit definition for FSMC_PCR3 register  ******************

const
  FSMC_PCR3_PWAITEN_Pos* = (1)
  FSMC_PCR3_PWAITEN_Msk* = (0x00000001 shl FSMC_PCR3_PWAITEN_Pos) ## !< 0x00000002
  FSMC_PCR3_PWAITEN* = FSMC_PCR3_PWAITEN_Msk
  FSMC_PCR3_PBKEN_Pos* = (2)
  FSMC_PCR3_PBKEN_Msk* = (0x00000001 shl FSMC_PCR3_PBKEN_Pos) ## !< 0x00000004
  FSMC_PCR3_PBKEN* = FSMC_PCR3_PBKEN_Msk
  FSMC_PCR3_PTYP_Pos* = (3)
  FSMC_PCR3_PTYP_Msk* = (0x00000001 shl FSMC_PCR3_PTYP_Pos) ## !< 0x00000008
  FSMC_PCR3_PTYP* = FSMC_PCR3_PTYP_Msk
  FSMC_PCR3_PWID_Pos* = (4)
  FSMC_PCR3_PWID_Msk* = (0x00000003 shl FSMC_PCR3_PWID_Pos) ## !< 0x00000030
  FSMC_PCR3_PWID* = FSMC_PCR3_PWID_Msk
  FSMC_PCR3_PWID_0* = (0x00000001 shl FSMC_PCR3_PWID_Pos) ## !< 0x00000010
  FSMC_PCR3_PWID_1* = (0x00000002 shl FSMC_PCR3_PWID_Pos) ## !< 0x00000020
  FSMC_PCR3_ECCEN_Pos* = (6)
  FSMC_PCR3_ECCEN_Msk* = (0x00000001 shl FSMC_PCR3_ECCEN_Pos) ## !< 0x00000040
  FSMC_PCR3_ECCEN* = FSMC_PCR3_ECCEN_Msk
  FSMC_PCR3_TCLR_Pos* = (9)
  FSMC_PCR3_TCLR_Msk* = (0x0000000F shl FSMC_PCR3_TCLR_Pos) ## !< 0x00001E00
  FSMC_PCR3_TCLR* = FSMC_PCR3_TCLR_Msk
  FSMC_PCR3_TCLR_0* = (0x00000001 shl FSMC_PCR3_TCLR_Pos) ## !< 0x00000200
  FSMC_PCR3_TCLR_1* = (0x00000002 shl FSMC_PCR3_TCLR_Pos) ## !< 0x00000400
  FSMC_PCR3_TCLR_2* = (0x00000004 shl FSMC_PCR3_TCLR_Pos) ## !< 0x00000800
  FSMC_PCR3_TCLR_3* = (0x00000008 shl FSMC_PCR3_TCLR_Pos) ## !< 0x00001000
  FSMC_PCR3_TAR_Pos* = (13)
  FSMC_PCR3_TAR_Msk* = (0x0000000F shl FSMC_PCR3_TAR_Pos) ## !< 0x0001E000
  FSMC_PCR3_TAR* = FSMC_PCR3_TAR_Msk
  FSMC_PCR3_TAR_0* = (0x00000001 shl FSMC_PCR3_TAR_Pos) ## !< 0x00002000
  FSMC_PCR3_TAR_1* = (0x00000002 shl FSMC_PCR3_TAR_Pos) ## !< 0x00004000
  FSMC_PCR3_TAR_2* = (0x00000004 shl FSMC_PCR3_TAR_Pos) ## !< 0x00008000
  FSMC_PCR3_TAR_3* = (0x00000008 shl FSMC_PCR3_TAR_Pos) ## !< 0x00010000
  FSMC_PCR3_ECCPS_Pos* = (17)
  FSMC_PCR3_ECCPS_Msk* = (0x00000007 shl FSMC_PCR3_ECCPS_Pos) ## !< 0x000E0000
  FSMC_PCR3_ECCPS* = FSMC_PCR3_ECCPS_Msk
  FSMC_PCR3_ECCPS_0* = (0x00000001 shl FSMC_PCR3_ECCPS_Pos) ## !< 0x00020000
  FSMC_PCR3_ECCPS_1* = (0x00000002 shl FSMC_PCR3_ECCPS_Pos) ## !< 0x00040000
  FSMC_PCR3_ECCPS_2* = (0x00000004 shl FSMC_PCR3_ECCPS_Pos) ## !< 0x00080000

## *****************  Bit definition for FSMC_PCR4 register  ******************

const
  FSMC_PCR4_PWAITEN_Pos* = (1)
  FSMC_PCR4_PWAITEN_Msk* = (0x00000001 shl FSMC_PCR4_PWAITEN_Pos) ## !< 0x00000002
  FSMC_PCR4_PWAITEN* = FSMC_PCR4_PWAITEN_Msk
  FSMC_PCR4_PBKEN_Pos* = (2)
  FSMC_PCR4_PBKEN_Msk* = (0x00000001 shl FSMC_PCR4_PBKEN_Pos) ## !< 0x00000004
  FSMC_PCR4_PBKEN* = FSMC_PCR4_PBKEN_Msk
  FSMC_PCR4_PTYP_Pos* = (3)
  FSMC_PCR4_PTYP_Msk* = (0x00000001 shl FSMC_PCR4_PTYP_Pos) ## !< 0x00000008
  FSMC_PCR4_PTYP* = FSMC_PCR4_PTYP_Msk
  FSMC_PCR4_PWID_Pos* = (4)
  FSMC_PCR4_PWID_Msk* = (0x00000003 shl FSMC_PCR4_PWID_Pos) ## !< 0x00000030
  FSMC_PCR4_PWID* = FSMC_PCR4_PWID_Msk
  FSMC_PCR4_PWID_0* = (0x00000001 shl FSMC_PCR4_PWID_Pos) ## !< 0x00000010
  FSMC_PCR4_PWID_1* = (0x00000002 shl FSMC_PCR4_PWID_Pos) ## !< 0x00000020
  FSMC_PCR4_ECCEN_Pos* = (6)
  FSMC_PCR4_ECCEN_Msk* = (0x00000001 shl FSMC_PCR4_ECCEN_Pos) ## !< 0x00000040
  FSMC_PCR4_ECCEN* = FSMC_PCR4_ECCEN_Msk
  FSMC_PCR4_TCLR_Pos* = (9)
  FSMC_PCR4_TCLR_Msk* = (0x0000000F shl FSMC_PCR4_TCLR_Pos) ## !< 0x00001E00
  FSMC_PCR4_TCLR* = FSMC_PCR4_TCLR_Msk
  FSMC_PCR4_TCLR_0* = (0x00000001 shl FSMC_PCR4_TCLR_Pos) ## !< 0x00000200
  FSMC_PCR4_TCLR_1* = (0x00000002 shl FSMC_PCR4_TCLR_Pos) ## !< 0x00000400
  FSMC_PCR4_TCLR_2* = (0x00000004 shl FSMC_PCR4_TCLR_Pos) ## !< 0x00000800
  FSMC_PCR4_TCLR_3* = (0x00000008 shl FSMC_PCR4_TCLR_Pos) ## !< 0x00001000
  FSMC_PCR4_TAR_Pos* = (13)
  FSMC_PCR4_TAR_Msk* = (0x0000000F shl FSMC_PCR4_TAR_Pos) ## !< 0x0001E000
  FSMC_PCR4_TAR* = FSMC_PCR4_TAR_Msk
  FSMC_PCR4_TAR_0* = (0x00000001 shl FSMC_PCR4_TAR_Pos) ## !< 0x00002000
  FSMC_PCR4_TAR_1* = (0x00000002 shl FSMC_PCR4_TAR_Pos) ## !< 0x00004000
  FSMC_PCR4_TAR_2* = (0x00000004 shl FSMC_PCR4_TAR_Pos) ## !< 0x00008000
  FSMC_PCR4_TAR_3* = (0x00000008 shl FSMC_PCR4_TAR_Pos) ## !< 0x00010000
  FSMC_PCR4_ECCPS_Pos* = (17)
  FSMC_PCR4_ECCPS_Msk* = (0x00000007 shl FSMC_PCR4_ECCPS_Pos) ## !< 0x000E0000
  FSMC_PCR4_ECCPS* = FSMC_PCR4_ECCPS_Msk
  FSMC_PCR4_ECCPS_0* = (0x00000001 shl FSMC_PCR4_ECCPS_Pos) ## !< 0x00020000
  FSMC_PCR4_ECCPS_1* = (0x00000002 shl FSMC_PCR4_ECCPS_Pos) ## !< 0x00040000
  FSMC_PCR4_ECCPS_2* = (0x00000004 shl FSMC_PCR4_ECCPS_Pos) ## !< 0x00080000

## ******************  Bit definition for FSMC_SR2 register  ******************

const
  FSMC_SR2_IRS_Pos* = (0)
  FSMC_SR2_IRS_Msk* = (0x00000001 shl FSMC_SR2_IRS_Pos) ## !< 0x00000001
  FSMC_SR2_IRS* = FSMC_SR2_IRS_Msk
  FSMC_SR2_ILS_Pos* = (1)
  FSMC_SR2_ILS_Msk* = (0x00000001 shl FSMC_SR2_ILS_Pos) ## !< 0x00000002
  FSMC_SR2_ILS* = FSMC_SR2_ILS_Msk
  FSMC_SR2_IFS_Pos* = (2)
  FSMC_SR2_IFS_Msk* = (0x00000001 shl FSMC_SR2_IFS_Pos) ## !< 0x00000004
  FSMC_SR2_IFS* = FSMC_SR2_IFS_Msk
  FSMC_SR2_IREN_Pos* = (3)
  FSMC_SR2_IREN_Msk* = (0x00000001 shl FSMC_SR2_IREN_Pos) ## !< 0x00000008
  FSMC_SR2_IREN* = FSMC_SR2_IREN_Msk
  FSMC_SR2_ILEN_Pos* = (4)
  FSMC_SR2_ILEN_Msk* = (0x00000001 shl FSMC_SR2_ILEN_Pos) ## !< 0x00000010
  FSMC_SR2_ILEN* = FSMC_SR2_ILEN_Msk
  FSMC_SR2_IFEN_Pos* = (5)
  FSMC_SR2_IFEN_Msk* = (0x00000001 shl FSMC_SR2_IFEN_Pos) ## !< 0x00000020
  FSMC_SR2_IFEN* = FSMC_SR2_IFEN_Msk
  FSMC_SR2_FEMPT_Pos* = (6)
  FSMC_SR2_FEMPT_Msk* = (0x00000001 shl FSMC_SR2_FEMPT_Pos) ## !< 0x00000040
  FSMC_SR2_FEMPT* = FSMC_SR2_FEMPT_Msk

## ******************  Bit definition for FSMC_SR3 register  ******************

const
  FSMC_SR3_IRS_Pos* = (0)
  FSMC_SR3_IRS_Msk* = (0x00000001 shl FSMC_SR3_IRS_Pos) ## !< 0x00000001
  FSMC_SR3_IRS* = FSMC_SR3_IRS_Msk
  FSMC_SR3_ILS_Pos* = (1)
  FSMC_SR3_ILS_Msk* = (0x00000001 shl FSMC_SR3_ILS_Pos) ## !< 0x00000002
  FSMC_SR3_ILS* = FSMC_SR3_ILS_Msk
  FSMC_SR3_IFS_Pos* = (2)
  FSMC_SR3_IFS_Msk* = (0x00000001 shl FSMC_SR3_IFS_Pos) ## !< 0x00000004
  FSMC_SR3_IFS* = FSMC_SR3_IFS_Msk
  FSMC_SR3_IREN_Pos* = (3)
  FSMC_SR3_IREN_Msk* = (0x00000001 shl FSMC_SR3_IREN_Pos) ## !< 0x00000008
  FSMC_SR3_IREN* = FSMC_SR3_IREN_Msk
  FSMC_SR3_ILEN_Pos* = (4)
  FSMC_SR3_ILEN_Msk* = (0x00000001 shl FSMC_SR3_ILEN_Pos) ## !< 0x00000010
  FSMC_SR3_ILEN* = FSMC_SR3_ILEN_Msk
  FSMC_SR3_IFEN_Pos* = (5)
  FSMC_SR3_IFEN_Msk* = (0x00000001 shl FSMC_SR3_IFEN_Pos) ## !< 0x00000020
  FSMC_SR3_IFEN* = FSMC_SR3_IFEN_Msk
  FSMC_SR3_FEMPT_Pos* = (6)
  FSMC_SR3_FEMPT_Msk* = (0x00000001 shl FSMC_SR3_FEMPT_Pos) ## !< 0x00000040
  FSMC_SR3_FEMPT* = FSMC_SR3_FEMPT_Msk

## ******************  Bit definition for FSMC_SR4 register  ******************

const
  FSMC_SR4_IRS_Pos* = (0)
  FSMC_SR4_IRS_Msk* = (0x00000001 shl FSMC_SR4_IRS_Pos) ## !< 0x00000001
  FSMC_SR4_IRS* = FSMC_SR4_IRS_Msk
  FSMC_SR4_ILS_Pos* = (1)
  FSMC_SR4_ILS_Msk* = (0x00000001 shl FSMC_SR4_ILS_Pos) ## !< 0x00000002
  FSMC_SR4_ILS* = FSMC_SR4_ILS_Msk
  FSMC_SR4_IFS_Pos* = (2)
  FSMC_SR4_IFS_Msk* = (0x00000001 shl FSMC_SR4_IFS_Pos) ## !< 0x00000004
  FSMC_SR4_IFS* = FSMC_SR4_IFS_Msk
  FSMC_SR4_IREN_Pos* = (3)
  FSMC_SR4_IREN_Msk* = (0x00000001 shl FSMC_SR4_IREN_Pos) ## !< 0x00000008
  FSMC_SR4_IREN* = FSMC_SR4_IREN_Msk
  FSMC_SR4_ILEN_Pos* = (4)
  FSMC_SR4_ILEN_Msk* = (0x00000001 shl FSMC_SR4_ILEN_Pos) ## !< 0x00000010
  FSMC_SR4_ILEN* = FSMC_SR4_ILEN_Msk
  FSMC_SR4_IFEN_Pos* = (5)
  FSMC_SR4_IFEN_Msk* = (0x00000001 shl FSMC_SR4_IFEN_Pos) ## !< 0x00000020
  FSMC_SR4_IFEN* = FSMC_SR4_IFEN_Msk
  FSMC_SR4_FEMPT_Pos* = (6)
  FSMC_SR4_FEMPT_Msk* = (0x00000001 shl FSMC_SR4_FEMPT_Pos) ## !< 0x00000040
  FSMC_SR4_FEMPT* = FSMC_SR4_FEMPT_Msk

## *****************  Bit definition for FSMC_PMEM2 register  *****************

const
  FSMC_PMEM2_MEMSET2_Pos* = (0)
  FSMC_PMEM2_MEMSET2_Msk* = (0x000000FF shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x000000FF
  FSMC_PMEM2_MEMSET2* = FSMC_PMEM2_MEMSET2_Msk
  FSMC_PMEM2_MEMSET2_0* = (0x00000001 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000001
  FSMC_PMEM2_MEMSET2_1* = (0x00000002 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000002
  FSMC_PMEM2_MEMSET2_2* = (0x00000004 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000004
  FSMC_PMEM2_MEMSET2_3* = (0x00000008 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000008
  FSMC_PMEM2_MEMSET2_4* = (0x00000010 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000010
  FSMC_PMEM2_MEMSET2_5* = (0x00000020 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000020
  FSMC_PMEM2_MEMSET2_6* = (0x00000040 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000040
  FSMC_PMEM2_MEMSET2_7* = (0x00000080 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000080
  FSMC_PMEM2_MEMWAIT2_Pos* = (8)
  FSMC_PMEM2_MEMWAIT2_Msk* = (0x000000FF shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x0000FF00
  FSMC_PMEM2_MEMWAIT2* = FSMC_PMEM2_MEMWAIT2_Msk
  FSMC_PMEM2_MEMWAIT2_0* = (0x00000001 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00000100
  FSMC_PMEM2_MEMWAIT2_1* = (0x00000002 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00000200
  FSMC_PMEM2_MEMWAIT2_2* = (0x00000004 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00000400
  FSMC_PMEM2_MEMWAIT2_3* = (0x00000008 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00000800
  FSMC_PMEM2_MEMWAIT2_4* = (0x00000010 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00001000
  FSMC_PMEM2_MEMWAIT2_5* = (0x00000020 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00002000
  FSMC_PMEM2_MEMWAIT2_6* = (0x00000040 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00004000
  FSMC_PMEM2_MEMWAIT2_7* = (0x00000080 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00008000
  FSMC_PMEM2_MEMHOLD2_Pos* = (16)
  FSMC_PMEM2_MEMHOLD2_Msk* = (0x000000FF shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00FF0000
  FSMC_PMEM2_MEMHOLD2* = FSMC_PMEM2_MEMHOLD2_Msk
  FSMC_PMEM2_MEMHOLD2_0* = (0x00000001 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00010000
  FSMC_PMEM2_MEMHOLD2_1* = (0x00000002 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00020000
  FSMC_PMEM2_MEMHOLD2_2* = (0x00000004 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00040000
  FSMC_PMEM2_MEMHOLD2_3* = (0x00000008 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00080000
  FSMC_PMEM2_MEMHOLD2_4* = (0x00000010 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00100000
  FSMC_PMEM2_MEMHOLD2_5* = (0x00000020 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00200000
  FSMC_PMEM2_MEMHOLD2_6* = (0x00000040 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00400000
  FSMC_PMEM2_MEMHOLD2_7* = (0x00000080 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00800000
  FSMC_PMEM2_MEMHIZ2_Pos* = (24)
  FSMC_PMEM2_MEMHIZ2_Msk* = (0x000000FF shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0xFF000000
  FSMC_PMEM2_MEMHIZ2* = FSMC_PMEM2_MEMHIZ2_Msk
  FSMC_PMEM2_MEMHIZ2_0* = (0x00000001 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x01000000
  FSMC_PMEM2_MEMHIZ2_1* = (0x00000002 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x02000000
  FSMC_PMEM2_MEMHIZ2_2* = (0x00000004 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x04000000
  FSMC_PMEM2_MEMHIZ2_3* = (0x00000008 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x08000000
  FSMC_PMEM2_MEMHIZ2_4* = (0x00000010 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x10000000
  FSMC_PMEM2_MEMHIZ2_5* = (0x00000020 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x20000000
  FSMC_PMEM2_MEMHIZ2_6* = (0x00000040 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x40000000
  FSMC_PMEM2_MEMHIZ2_7* = (0x00000080 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x80000000

## *****************  Bit definition for FSMC_PMEM3 register  *****************

const
  FSMC_PMEM3_MEMSET3_Pos* = (0)
  FSMC_PMEM3_MEMSET3_Msk* = (0x000000FF shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x000000FF
  FSMC_PMEM3_MEMSET3* = FSMC_PMEM3_MEMSET3_Msk
  FSMC_PMEM3_MEMSET3_0* = (0x00000001 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000001
  FSMC_PMEM3_MEMSET3_1* = (0x00000002 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000002
  FSMC_PMEM3_MEMSET3_2* = (0x00000004 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000004
  FSMC_PMEM3_MEMSET3_3* = (0x00000008 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000008
  FSMC_PMEM3_MEMSET3_4* = (0x00000010 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000010
  FSMC_PMEM3_MEMSET3_5* = (0x00000020 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000020
  FSMC_PMEM3_MEMSET3_6* = (0x00000040 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000040
  FSMC_PMEM3_MEMSET3_7* = (0x00000080 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000080
  FSMC_PMEM3_MEMWAIT3_Pos* = (8)
  FSMC_PMEM3_MEMWAIT3_Msk* = (0x000000FF shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x0000FF00
  FSMC_PMEM3_MEMWAIT3* = FSMC_PMEM3_MEMWAIT3_Msk
  FSMC_PMEM3_MEMWAIT3_0* = (0x00000001 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00000100
  FSMC_PMEM3_MEMWAIT3_1* = (0x00000002 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00000200
  FSMC_PMEM3_MEMWAIT3_2* = (0x00000004 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00000400
  FSMC_PMEM3_MEMWAIT3_3* = (0x00000008 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00000800
  FSMC_PMEM3_MEMWAIT3_4* = (0x00000010 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00001000
  FSMC_PMEM3_MEMWAIT3_5* = (0x00000020 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00002000
  FSMC_PMEM3_MEMWAIT3_6* = (0x00000040 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00004000
  FSMC_PMEM3_MEMWAIT3_7* = (0x00000080 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00008000
  FSMC_PMEM3_MEMHOLD3_Pos* = (16)
  FSMC_PMEM3_MEMHOLD3_Msk* = (0x000000FF shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00FF0000
  FSMC_PMEM3_MEMHOLD3* = FSMC_PMEM3_MEMHOLD3_Msk
  FSMC_PMEM3_MEMHOLD3_0* = (0x00000001 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00010000
  FSMC_PMEM3_MEMHOLD3_1* = (0x00000002 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00020000
  FSMC_PMEM3_MEMHOLD3_2* = (0x00000004 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00040000
  FSMC_PMEM3_MEMHOLD3_3* = (0x00000008 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00080000
  FSMC_PMEM3_MEMHOLD3_4* = (0x00000010 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00100000
  FSMC_PMEM3_MEMHOLD3_5* = (0x00000020 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00200000
  FSMC_PMEM3_MEMHOLD3_6* = (0x00000040 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00400000
  FSMC_PMEM3_MEMHOLD3_7* = (0x00000080 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00800000
  FSMC_PMEM3_MEMHIZ3_Pos* = (24)
  FSMC_PMEM3_MEMHIZ3_Msk* = (0x000000FF shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0xFF000000
  FSMC_PMEM3_MEMHIZ3* = FSMC_PMEM3_MEMHIZ3_Msk
  FSMC_PMEM3_MEMHIZ3_0* = (0x00000001 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x01000000
  FSMC_PMEM3_MEMHIZ3_1* = (0x00000002 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x02000000
  FSMC_PMEM3_MEMHIZ3_2* = (0x00000004 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x04000000
  FSMC_PMEM3_MEMHIZ3_3* = (0x00000008 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x08000000
  FSMC_PMEM3_MEMHIZ3_4* = (0x00000010 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x10000000
  FSMC_PMEM3_MEMHIZ3_5* = (0x00000020 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x20000000
  FSMC_PMEM3_MEMHIZ3_6* = (0x00000040 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x40000000
  FSMC_PMEM3_MEMHIZ3_7* = (0x00000080 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x80000000

## *****************  Bit definition for FSMC_PMEM4 register  *****************

const
  FSMC_PMEM4_MEMSET4_Pos* = (0)
  FSMC_PMEM4_MEMSET4_Msk* = (0x000000FF shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x000000FF
  FSMC_PMEM4_MEMSET4* = FSMC_PMEM4_MEMSET4_Msk
  FSMC_PMEM4_MEMSET4_0* = (0x00000001 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000001
  FSMC_PMEM4_MEMSET4_1* = (0x00000002 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000002
  FSMC_PMEM4_MEMSET4_2* = (0x00000004 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000004
  FSMC_PMEM4_MEMSET4_3* = (0x00000008 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000008
  FSMC_PMEM4_MEMSET4_4* = (0x00000010 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000010
  FSMC_PMEM4_MEMSET4_5* = (0x00000020 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000020
  FSMC_PMEM4_MEMSET4_6* = (0x00000040 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000040
  FSMC_PMEM4_MEMSET4_7* = (0x00000080 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000080
  FSMC_PMEM4_MEMWAIT4_Pos* = (8)
  FSMC_PMEM4_MEMWAIT4_Msk* = (0x000000FF shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x0000FF00
  FSMC_PMEM4_MEMWAIT4* = FSMC_PMEM4_MEMWAIT4_Msk
  FSMC_PMEM4_MEMWAIT4_0* = (0x00000001 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00000100
  FSMC_PMEM4_MEMWAIT4_1* = (0x00000002 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00000200
  FSMC_PMEM4_MEMWAIT4_2* = (0x00000004 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00000400
  FSMC_PMEM4_MEMWAIT4_3* = (0x00000008 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00000800
  FSMC_PMEM4_MEMWAIT4_4* = (0x00000010 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00001000
  FSMC_PMEM4_MEMWAIT4_5* = (0x00000020 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00002000
  FSMC_PMEM4_MEMWAIT4_6* = (0x00000040 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00004000
  FSMC_PMEM4_MEMWAIT4_7* = (0x00000080 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00008000
  FSMC_PMEM4_MEMHOLD4_Pos* = (16)
  FSMC_PMEM4_MEMHOLD4_Msk* = (0x000000FF shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00FF0000
  FSMC_PMEM4_MEMHOLD4* = FSMC_PMEM4_MEMHOLD4_Msk
  FSMC_PMEM4_MEMHOLD4_0* = (0x00000001 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00010000
  FSMC_PMEM4_MEMHOLD4_1* = (0x00000002 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00020000
  FSMC_PMEM4_MEMHOLD4_2* = (0x00000004 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00040000
  FSMC_PMEM4_MEMHOLD4_3* = (0x00000008 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00080000
  FSMC_PMEM4_MEMHOLD4_4* = (0x00000010 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00100000
  FSMC_PMEM4_MEMHOLD4_5* = (0x00000020 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00200000
  FSMC_PMEM4_MEMHOLD4_6* = (0x00000040 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00400000
  FSMC_PMEM4_MEMHOLD4_7* = (0x00000080 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00800000
  FSMC_PMEM4_MEMHIZ4_Pos* = (24)
  FSMC_PMEM4_MEMHIZ4_Msk* = (0x000000FF shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0xFF000000
  FSMC_PMEM4_MEMHIZ4* = FSMC_PMEM4_MEMHIZ4_Msk
  FSMC_PMEM4_MEMHIZ4_0* = (0x00000001 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x01000000
  FSMC_PMEM4_MEMHIZ4_1* = (0x00000002 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x02000000
  FSMC_PMEM4_MEMHIZ4_2* = (0x00000004 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x04000000
  FSMC_PMEM4_MEMHIZ4_3* = (0x00000008 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x08000000
  FSMC_PMEM4_MEMHIZ4_4* = (0x00000010 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x10000000
  FSMC_PMEM4_MEMHIZ4_5* = (0x00000020 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x20000000
  FSMC_PMEM4_MEMHIZ4_6* = (0x00000040 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x40000000
  FSMC_PMEM4_MEMHIZ4_7* = (0x00000080 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x80000000

## *****************  Bit definition for FSMC_PATT2 register  *****************

const
  FSMC_PATT2_ATTSET2_Pos* = (0)
  FSMC_PATT2_ATTSET2_Msk* = (0x000000FF shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x000000FF
  FSMC_PATT2_ATTSET2* = FSMC_PATT2_ATTSET2_Msk
  FSMC_PATT2_ATTSET2_0* = (0x00000001 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000001
  FSMC_PATT2_ATTSET2_1* = (0x00000002 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000002
  FSMC_PATT2_ATTSET2_2* = (0x00000004 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000004
  FSMC_PATT2_ATTSET2_3* = (0x00000008 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000008
  FSMC_PATT2_ATTSET2_4* = (0x00000010 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000010
  FSMC_PATT2_ATTSET2_5* = (0x00000020 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000020
  FSMC_PATT2_ATTSET2_6* = (0x00000040 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000040
  FSMC_PATT2_ATTSET2_7* = (0x00000080 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000080
  FSMC_PATT2_ATTWAIT2_Pos* = (8)
  FSMC_PATT2_ATTWAIT2_Msk* = (0x000000FF shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x0000FF00
  FSMC_PATT2_ATTWAIT2* = FSMC_PATT2_ATTWAIT2_Msk
  FSMC_PATT2_ATTWAIT2_0* = (0x00000001 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00000100
  FSMC_PATT2_ATTWAIT2_1* = (0x00000002 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00000200
  FSMC_PATT2_ATTWAIT2_2* = (0x00000004 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00000400
  FSMC_PATT2_ATTWAIT2_3* = (0x00000008 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00000800
  FSMC_PATT2_ATTWAIT2_4* = (0x00000010 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00001000
  FSMC_PATT2_ATTWAIT2_5* = (0x00000020 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00002000
  FSMC_PATT2_ATTWAIT2_6* = (0x00000040 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00004000
  FSMC_PATT2_ATTWAIT2_7* = (0x00000080 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00008000
  FSMC_PATT2_ATTHOLD2_Pos* = (16)
  FSMC_PATT2_ATTHOLD2_Msk* = (0x000000FF shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00FF0000
  FSMC_PATT2_ATTHOLD2* = FSMC_PATT2_ATTHOLD2_Msk
  FSMC_PATT2_ATTHOLD2_0* = (0x00000001 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00010000
  FSMC_PATT2_ATTHOLD2_1* = (0x00000002 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00020000
  FSMC_PATT2_ATTHOLD2_2* = (0x00000004 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00040000
  FSMC_PATT2_ATTHOLD2_3* = (0x00000008 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00080000
  FSMC_PATT2_ATTHOLD2_4* = (0x00000010 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00100000
  FSMC_PATT2_ATTHOLD2_5* = (0x00000020 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00200000
  FSMC_PATT2_ATTHOLD2_6* = (0x00000040 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00400000
  FSMC_PATT2_ATTHOLD2_7* = (0x00000080 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00800000
  FSMC_PATT2_ATTHIZ2_Pos* = (24)
  FSMC_PATT2_ATTHIZ2_Msk* = (0x000000FF shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0xFF000000
  FSMC_PATT2_ATTHIZ2* = FSMC_PATT2_ATTHIZ2_Msk
  FSMC_PATT2_ATTHIZ2_0* = (0x00000001 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x01000000
  FSMC_PATT2_ATTHIZ2_1* = (0x00000002 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x02000000
  FSMC_PATT2_ATTHIZ2_2* = (0x00000004 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x04000000
  FSMC_PATT2_ATTHIZ2_3* = (0x00000008 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x08000000
  FSMC_PATT2_ATTHIZ2_4* = (0x00000010 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x10000000
  FSMC_PATT2_ATTHIZ2_5* = (0x00000020 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x20000000
  FSMC_PATT2_ATTHIZ2_6* = (0x00000040 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x40000000
  FSMC_PATT2_ATTHIZ2_7* = (0x00000080 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x80000000

## *****************  Bit definition for FSMC_PATT3 register  *****************

const
  FSMC_PATT3_ATTSET3_Pos* = (0)
  FSMC_PATT3_ATTSET3_Msk* = (0x000000FF shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x000000FF
  FSMC_PATT3_ATTSET3* = FSMC_PATT3_ATTSET3_Msk
  FSMC_PATT3_ATTSET3_0* = (0x00000001 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000001
  FSMC_PATT3_ATTSET3_1* = (0x00000002 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000002
  FSMC_PATT3_ATTSET3_2* = (0x00000004 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000004
  FSMC_PATT3_ATTSET3_3* = (0x00000008 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000008
  FSMC_PATT3_ATTSET3_4* = (0x00000010 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000010
  FSMC_PATT3_ATTSET3_5* = (0x00000020 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000020
  FSMC_PATT3_ATTSET3_6* = (0x00000040 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000040
  FSMC_PATT3_ATTSET3_7* = (0x00000080 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000080
  FSMC_PATT3_ATTWAIT3_Pos* = (8)
  FSMC_PATT3_ATTWAIT3_Msk* = (0x000000FF shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x0000FF00
  FSMC_PATT3_ATTWAIT3* = FSMC_PATT3_ATTWAIT3_Msk
  FSMC_PATT3_ATTWAIT3_0* = (0x00000001 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00000100
  FSMC_PATT3_ATTWAIT3_1* = (0x00000002 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00000200
  FSMC_PATT3_ATTWAIT3_2* = (0x00000004 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00000400
  FSMC_PATT3_ATTWAIT3_3* = (0x00000008 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00000800
  FSMC_PATT3_ATTWAIT3_4* = (0x00000010 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00001000
  FSMC_PATT3_ATTWAIT3_5* = (0x00000020 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00002000
  FSMC_PATT3_ATTWAIT3_6* = (0x00000040 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00004000
  FSMC_PATT3_ATTWAIT3_7* = (0x00000080 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00008000
  FSMC_PATT3_ATTHOLD3_Pos* = (16)
  FSMC_PATT3_ATTHOLD3_Msk* = (0x000000FF shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00FF0000
  FSMC_PATT3_ATTHOLD3* = FSMC_PATT3_ATTHOLD3_Msk
  FSMC_PATT3_ATTHOLD3_0* = (0x00000001 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00010000
  FSMC_PATT3_ATTHOLD3_1* = (0x00000002 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00020000
  FSMC_PATT3_ATTHOLD3_2* = (0x00000004 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00040000
  FSMC_PATT3_ATTHOLD3_3* = (0x00000008 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00080000
  FSMC_PATT3_ATTHOLD3_4* = (0x00000010 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00100000
  FSMC_PATT3_ATTHOLD3_5* = (0x00000020 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00200000
  FSMC_PATT3_ATTHOLD3_6* = (0x00000040 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00400000
  FSMC_PATT3_ATTHOLD3_7* = (0x00000080 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00800000
  FSMC_PATT3_ATTHIZ3_Pos* = (24)
  FSMC_PATT3_ATTHIZ3_Msk* = (0x000000FF shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0xFF000000
  FSMC_PATT3_ATTHIZ3* = FSMC_PATT3_ATTHIZ3_Msk
  FSMC_PATT3_ATTHIZ3_0* = (0x00000001 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x01000000
  FSMC_PATT3_ATTHIZ3_1* = (0x00000002 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x02000000
  FSMC_PATT3_ATTHIZ3_2* = (0x00000004 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x04000000
  FSMC_PATT3_ATTHIZ3_3* = (0x00000008 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x08000000
  FSMC_PATT3_ATTHIZ3_4* = (0x00000010 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x10000000
  FSMC_PATT3_ATTHIZ3_5* = (0x00000020 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x20000000
  FSMC_PATT3_ATTHIZ3_6* = (0x00000040 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x40000000
  FSMC_PATT3_ATTHIZ3_7* = (0x00000080 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x80000000

## *****************  Bit definition for FSMC_PATT4 register  *****************

const
  FSMC_PATT4_ATTSET4_Pos* = (0)
  FSMC_PATT4_ATTSET4_Msk* = (0x000000FF shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x000000FF
  FSMC_PATT4_ATTSET4* = FSMC_PATT4_ATTSET4_Msk
  FSMC_PATT4_ATTSET4_0* = (0x00000001 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000001
  FSMC_PATT4_ATTSET4_1* = (0x00000002 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000002
  FSMC_PATT4_ATTSET4_2* = (0x00000004 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000004
  FSMC_PATT4_ATTSET4_3* = (0x00000008 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000008
  FSMC_PATT4_ATTSET4_4* = (0x00000010 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000010
  FSMC_PATT4_ATTSET4_5* = (0x00000020 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000020
  FSMC_PATT4_ATTSET4_6* = (0x00000040 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000040
  FSMC_PATT4_ATTSET4_7* = (0x00000080 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000080
  FSMC_PATT4_ATTWAIT4_Pos* = (8)
  FSMC_PATT4_ATTWAIT4_Msk* = (0x000000FF shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x0000FF00
  FSMC_PATT4_ATTWAIT4* = FSMC_PATT4_ATTWAIT4_Msk
  FSMC_PATT4_ATTWAIT4_0* = (0x00000001 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00000100
  FSMC_PATT4_ATTWAIT4_1* = (0x00000002 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00000200
  FSMC_PATT4_ATTWAIT4_2* = (0x00000004 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00000400
  FSMC_PATT4_ATTWAIT4_3* = (0x00000008 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00000800
  FSMC_PATT4_ATTWAIT4_4* = (0x00000010 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00001000
  FSMC_PATT4_ATTWAIT4_5* = (0x00000020 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00002000
  FSMC_PATT4_ATTWAIT4_6* = (0x00000040 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00004000
  FSMC_PATT4_ATTWAIT4_7* = (0x00000080 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00008000
  FSMC_PATT4_ATTHOLD4_Pos* = (16)
  FSMC_PATT4_ATTHOLD4_Msk* = (0x000000FF shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00FF0000
  FSMC_PATT4_ATTHOLD4* = FSMC_PATT4_ATTHOLD4_Msk
  FSMC_PATT4_ATTHOLD4_0* = (0x00000001 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00010000
  FSMC_PATT4_ATTHOLD4_1* = (0x00000002 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00020000
  FSMC_PATT4_ATTHOLD4_2* = (0x00000004 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00040000
  FSMC_PATT4_ATTHOLD4_3* = (0x00000008 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00080000
  FSMC_PATT4_ATTHOLD4_4* = (0x00000010 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00100000
  FSMC_PATT4_ATTHOLD4_5* = (0x00000020 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00200000
  FSMC_PATT4_ATTHOLD4_6* = (0x00000040 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00400000
  FSMC_PATT4_ATTHOLD4_7* = (0x00000080 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00800000
  FSMC_PATT4_ATTHIZ4_Pos* = (24)
  FSMC_PATT4_ATTHIZ4_Msk* = (0x000000FF shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0xFF000000
  FSMC_PATT4_ATTHIZ4* = FSMC_PATT4_ATTHIZ4_Msk
  FSMC_PATT4_ATTHIZ4_0* = (0x00000001 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x01000000
  FSMC_PATT4_ATTHIZ4_1* = (0x00000002 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x02000000
  FSMC_PATT4_ATTHIZ4_2* = (0x00000004 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x04000000
  FSMC_PATT4_ATTHIZ4_3* = (0x00000008 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x08000000
  FSMC_PATT4_ATTHIZ4_4* = (0x00000010 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x10000000
  FSMC_PATT4_ATTHIZ4_5* = (0x00000020 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x20000000
  FSMC_PATT4_ATTHIZ4_6* = (0x00000040 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x40000000
  FSMC_PATT4_ATTHIZ4_7* = (0x00000080 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x80000000

## *****************  Bit definition for FSMC_PIO4 register  ******************

const
  FSMC_PIO4_IOSET4_Pos* = (0)
  FSMC_PIO4_IOSET4_Msk* = (0x000000FF shl FSMC_PIO4_IOSET4_Pos) ## !< 0x000000FF
  FSMC_PIO4_IOSET4* = FSMC_PIO4_IOSET4_Msk
  FSMC_PIO4_IOSET4_0* = (0x00000001 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000001
  FSMC_PIO4_IOSET4_1* = (0x00000002 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000002
  FSMC_PIO4_IOSET4_2* = (0x00000004 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000004
  FSMC_PIO4_IOSET4_3* = (0x00000008 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000008
  FSMC_PIO4_IOSET4_4* = (0x00000010 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000010
  FSMC_PIO4_IOSET4_5* = (0x00000020 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000020
  FSMC_PIO4_IOSET4_6* = (0x00000040 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000040
  FSMC_PIO4_IOSET4_7* = (0x00000080 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000080
  FSMC_PIO4_IOWAIT4_Pos* = (8)
  FSMC_PIO4_IOWAIT4_Msk* = (0x000000FF shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x0000FF00
  FSMC_PIO4_IOWAIT4* = FSMC_PIO4_IOWAIT4_Msk
  FSMC_PIO4_IOWAIT4_0* = (0x00000001 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00000100
  FSMC_PIO4_IOWAIT4_1* = (0x00000002 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00000200
  FSMC_PIO4_IOWAIT4_2* = (0x00000004 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00000400
  FSMC_PIO4_IOWAIT4_3* = (0x00000008 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00000800
  FSMC_PIO4_IOWAIT4_4* = (0x00000010 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00001000
  FSMC_PIO4_IOWAIT4_5* = (0x00000020 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00002000
  FSMC_PIO4_IOWAIT4_6* = (0x00000040 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00004000
  FSMC_PIO4_IOWAIT4_7* = (0x00000080 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00008000
  FSMC_PIO4_IOHOLD4_Pos* = (16)
  FSMC_PIO4_IOHOLD4_Msk* = (0x000000FF shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00FF0000
  FSMC_PIO4_IOHOLD4* = FSMC_PIO4_IOHOLD4_Msk
  FSMC_PIO4_IOHOLD4_0* = (0x00000001 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00010000
  FSMC_PIO4_IOHOLD4_1* = (0x00000002 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00020000
  FSMC_PIO4_IOHOLD4_2* = (0x00000004 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00040000
  FSMC_PIO4_IOHOLD4_3* = (0x00000008 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00080000
  FSMC_PIO4_IOHOLD4_4* = (0x00000010 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00100000
  FSMC_PIO4_IOHOLD4_5* = (0x00000020 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00200000
  FSMC_PIO4_IOHOLD4_6* = (0x00000040 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00400000
  FSMC_PIO4_IOHOLD4_7* = (0x00000080 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00800000
  FSMC_PIO4_IOHIZ4_Pos* = (24)
  FSMC_PIO4_IOHIZ4_Msk* = (0x000000FF shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0xFF000000
  FSMC_PIO4_IOHIZ4* = FSMC_PIO4_IOHIZ4_Msk
  FSMC_PIO4_IOHIZ4_0* = (0x00000001 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x01000000
  FSMC_PIO4_IOHIZ4_1* = (0x00000002 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x02000000
  FSMC_PIO4_IOHIZ4_2* = (0x00000004 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x04000000
  FSMC_PIO4_IOHIZ4_3* = (0x00000008 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x08000000
  FSMC_PIO4_IOHIZ4_4* = (0x00000010 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x10000000
  FSMC_PIO4_IOHIZ4_5* = (0x00000020 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x20000000
  FSMC_PIO4_IOHIZ4_6* = (0x00000040 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x40000000
  FSMC_PIO4_IOHIZ4_7* = (0x00000080 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x80000000

## *****************  Bit definition for FSMC_ECCR2 register  *****************

const
  FSMC_ECCR2_ECC2_Pos* = (0)
  FSMC_ECCR2_ECC2_Msk* = (0xFFFFFFFF shl FSMC_ECCR2_ECC2_Pos) ## !< 0xFFFFFFFF
  FSMC_ECCR2_ECC2* = FSMC_ECCR2_ECC2_Msk

## *****************  Bit definition for FSMC_ECCR3 register  *****************

const
  FSMC_ECCR3_ECC3_Pos* = (0)
  FSMC_ECCR3_ECC3_Msk* = (0xFFFFFFFF shl FSMC_ECCR3_ECC3_Pos) ## !< 0xFFFFFFFF
  FSMC_ECCR3_ECC3* = FSMC_ECCR3_ECC3_Msk

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

##  Legacy defines

const
  GPIO_MODER_MODE0_Pos* = GPIO_MODER_MODER0_Pos
  GPIO_MODER_MODE0_Msk* = GPIO_MODER_MODER0_Msk
  GPIO_MODER_MODE0* = GPIO_MODER_MODER0
  GPIO_MODER_MODE0_0* = GPIO_MODER_MODER0_0
  GPIO_MODER_MODE0_1* = GPIO_MODER_MODER0_1
  GPIO_MODER_MODE1_Pos* = GPIO_MODER_MODER1_Pos
  GPIO_MODER_MODE1_Msk* = GPIO_MODER_MODER1_Msk
  GPIO_MODER_MODE1* = GPIO_MODER_MODER1
  GPIO_MODER_MODE1_0* = GPIO_MODER_MODER1_0
  GPIO_MODER_MODE1_1* = GPIO_MODER_MODER1_1
  GPIO_MODER_MODE2_Pos* = GPIO_MODER_MODER2_PoS
  GPIO_MODER_MODE2_Msk* = GPIO_MODER_MODER2_Msk
  GPIO_MODER_MODE2* = GPIO_MODER_MODER2
  GPIO_MODER_MODE2_0* = GPIO_MODER_MODER2_0
  GPIO_MODER_MODE2_1* = GPIO_MODER_MODER2_1
  GPIO_MODER_MODE3_Pos* = GPIO_MODER_MODER3_Pos
  GPIO_MODER_MODE3_Msk* = GPIO_MODER_MODER3_Msk
  GPIO_MODER_MODE3* = GPIO_MODER_MODER3
  GPIO_MODER_MODE3_0* = GPIO_MODER_MODER3_0
  GPIO_MODER_MODE3_1* = GPIO_MODER_MODER3_1
  GPIO_MODER_MODE4_Pos* = GPIO_MODER_MODER4_Pos
  GPIO_MODER_MODE4_Msk* = GPIO_MODER_MODER4_Msk
  GPIO_MODER_MODE4* = GPIO_MODER_MODER4
  GPIO_MODER_MODE4_0* = GPIO_MODER_MODER4_0
  GPIO_MODER_MODE4_1* = GPIO_MODER_MODER4_1
  GPIO_MODER_MODE5_Pos* = GPIO_MODER_MODER5_Pos
  GPIO_MODER_MODE5_Msk* = GPIO_MODER_MODER5_Msk
  GPIO_MODER_MODE5* = GPIO_MODER_MODER5
  GPIO_MODER_MODE5_0* = GPIO_MODER_MODER5_0
  GPIO_MODER_MODE5_1* = GPIO_MODER_MODER5_1
  GPIO_MODER_MODE6_Pos* = GPIO_MODER_MODER6_Pos
  GPIO_MODER_MODE6_Msk* = GPIO_MODER_MODER6_Msk
  GPIO_MODER_MODE6* = GPIO_MODER_MODER6
  GPIO_MODER_MODE6_0* = GPIO_MODER_MODER6_0
  GPIO_MODER_MODE6_1* = GPIO_MODER_MODER6_1
  GPIO_MODER_MODE7_Pos* = GPIO_MODER_MODER7_Pos
  GPIO_MODER_MODE7_Msk* = GPIO_MODER_MODER7_Msk
  GPIO_MODER_MODE7* = GPIO_MODER_MODER7
  GPIO_MODER_MODE7_0* = GPIO_MODER_MODER7_0
  GPIO_MODER_MODE7_1* = GPIO_MODER_MODER7_1
  GPIO_MODER_MODE8_Pos* = GPIO_MODER_MODER8_Pos
  GPIO_MODER_MODE8_Msk* = GPIO_MODER_MODER2_Msk
  GPIO_MODER_MODE8* = GPIO_MODER_MODER8
  GPIO_MODER_MODE8_0* = GPIO_MODER_MODER8_0
  GPIO_MODER_MODE8_1* = GPIO_MODER_MODER8_1
  GPIO_MODER_MODE9_Pos* = GPIO_MODER_MODER9_Pos
  GPIO_MODER_MODE9_Msk* = GPIO_MODER_MODER9_Msk
  GPIO_MODER_MODE9* = GPIO_MODER_MODER9
  GPIO_MODER_MODE9_0* = GPIO_MODER_MODER9_0
  GPIO_MODER_MODE9_1* = GPIO_MODER_MODER9_1
  GPIO_MODER_MODE10_Pos* = GPIO_MODER_MODER10_Pos
  GPIO_MODER_MODE10_Msk* = GPIO_MODER_MODER10_Msk
  GPIO_MODER_MODE10x* = GPIO_MODER_MODER10
  GPIO_MODER_MODE10_0* = GPIO_MODER_MODER10_0
  GPIO_MODER_MODE10_1* = GPIO_MODER_MODER10_1
  GPIO_MODER_MODE11_Pos* = GPIO_MODER_MODER11_Pos
  GPIO_MODER_MODE11_Msk* = GPIO_MODER_MODER11_Msk
  GPIO_MODER_MODE11x* = GPIO_MODER_MODER11
  GPIO_MODER_MODE11_0* = GPIO_MODER_MODER11_0
  GPIO_MODER_MODE11_1* = GPIO_MODER_MODER11_1
  GPIO_MODER_MODE12_Pos* = GPIO_MODER_MODER12_Pos
  GPIO_MODER_MODE12_Msk* = GPIO_MODER_MODER12_Msk
  GPIO_MODER_MODE12* = GPIO_MODER_MODER12
  GPIO_MODER_MODE12_0* = GPIO_MODER_MODER12_0
  GPIO_MODER_MODE12_1* = GPIO_MODER_MODER12_1
  GPIO_MODER_MODE13_Pos* = GPIO_MODER_MODER13_Pos
  GPIO_MODER_MODE13_Msk* = GPIO_MODER_MODER13_Msk
  GPIO_MODER_MODE13* = GPIO_MODER_MODER13
  GPIO_MODER_MODE13_0* = GPIO_MODER_MODER13_0
  GPIO_MODER_MODE13_1* = GPIO_MODER_MODER13_1
  GPIO_MODER_MODE14_Pos* = GPIO_MODER_MODER14_Pos
  GPIO_MODER_MODE14_Msk* = GPIO_MODER_MODER14_Msk
  GPIO_MODER_MODE14* = GPIO_MODER_MODER14
  GPIO_MODER_MODE14_0* = GPIO_MODER_MODER14_0
  GPIO_MODER_MODE14_1* = GPIO_MODER_MODER14_1
  GPIO_MODER_MODE15_Pos* = GPIO_MODER_MODER15_Pos
  GPIO_MODER_MODE15_Msk* = GPIO_MODER_MODER15_Msk
  GPIO_MODER_MODE15* = GPIO_MODER_MODER15
  GPIO_MODER_MODE15_0* = GPIO_MODER_MODER15_0
  GPIO_MODER_MODE15_1* = GPIO_MODER_MODER15_1

## *****************  Bits definition for GPIO_OTYPER register  ***************

const
  GPIO_OTYPER_OT0_Pos* = (0)
  GPIO_OTYPER_OT0_Msk* = (0x00000001 shl GPIO_OTYPER_OT0_Pos) ## !< 0x00000001
  GPIO_OTYPER_OT0* = GPIO_OTYPER_OT0_Msk
  GPIO_OTYPER_OT1_Pos* = (1)
  GPIO_OTYPER_OT1_Msk* = (0x00000001 shl GPIO_OTYPER_OT1_Pos) ## !< 0x00000002
  GPIO_OTYPER_OT1* = GPIO_OTYPER_OT1_Msk
  GPIO_OTYPER_OT2_Pos* = (2)
  GPIO_OTYPER_OT2_Msk* = (0x00000001 shl GPIO_OTYPER_OT2_Pos) ## !< 0x00000004
  GPIO_OTYPER_OT2* = GPIO_OTYPER_OT2_Msk
  GPIO_OTYPER_OT3_Pos* = (3)
  GPIO_OTYPER_OT3_Msk* = (0x00000001 shl GPIO_OTYPER_OT3_Pos) ## !< 0x00000008
  GPIO_OTYPER_OT3* = GPIO_OTYPER_OT3_Msk
  GPIO_OTYPER_OT4_Pos* = (4)
  GPIO_OTYPER_OT4_Msk* = (0x00000001 shl GPIO_OTYPER_OT4_Pos) ## !< 0x00000010
  GPIO_OTYPER_OT4* = GPIO_OTYPER_OT4_Msk
  GPIO_OTYPER_OT5_Pos* = (5)
  GPIO_OTYPER_OT5_Msk* = (0x00000001 shl GPIO_OTYPER_OT5_Pos) ## !< 0x00000020
  GPIO_OTYPER_OT5* = GPIO_OTYPER_OT5_Msk
  GPIO_OTYPER_OT6_Pos* = (6)
  GPIO_OTYPER_OT6_Msk* = (0x00000001 shl GPIO_OTYPER_OT6_Pos) ## !< 0x00000040
  GPIO_OTYPER_OT6* = GPIO_OTYPER_OT6_Msk
  GPIO_OTYPER_OT7_Pos* = (7)
  GPIO_OTYPER_OT7_Msk* = (0x00000001 shl GPIO_OTYPER_OT7_Pos) ## !< 0x00000080
  GPIO_OTYPER_OT7* = GPIO_OTYPER_OT7_Msk
  GPIO_OTYPER_OT8_Pos* = (8)
  GPIO_OTYPER_OT8_Msk* = (0x00000001 shl GPIO_OTYPER_OT8_Pos) ## !< 0x00000100
  GPIO_OTYPER_OT8* = GPIO_OTYPER_OT8_Msk
  GPIO_OTYPER_OT9_Pos* = (9)
  GPIO_OTYPER_OT9_Msk* = (0x00000001 shl GPIO_OTYPER_OT9_Pos) ## !< 0x00000200
  GPIO_OTYPER_OT9* = GPIO_OTYPER_OT9_Msk
  GPIO_OTYPER_OT10_Pos* = (10)
  GPIO_OTYPER_OT10_Msk* = (0x00000001 shl GPIO_OTYPER_OT10_Pos) ## !< 0x00000400
  GPIO_OTYPER_OT10* = GPIO_OTYPER_OT10_Msk
  GPIO_OTYPER_OT11_Pos* = (11)
  GPIO_OTYPER_OT11_Msk* = (0x00000001 shl GPIO_OTYPER_OT11_Pos) ## !< 0x00000800
  GPIO_OTYPER_OT11* = GPIO_OTYPER_OT11_Msk
  GPIO_OTYPER_OT12_Pos* = (12)
  GPIO_OTYPER_OT12_Msk* = (0x00000001 shl GPIO_OTYPER_OT12_Pos) ## !< 0x00001000
  GPIO_OTYPER_OT12* = GPIO_OTYPER_OT12_Msk
  GPIO_OTYPER_OT13_Pos* = (13)
  GPIO_OTYPER_OT13_Msk* = (0x00000001 shl GPIO_OTYPER_OT13_Pos) ## !< 0x00002000
  GPIO_OTYPER_OT13* = GPIO_OTYPER_OT13_Msk
  GPIO_OTYPER_OT14_Pos* = (14)
  GPIO_OTYPER_OT14_Msk* = (0x00000001 shl GPIO_OTYPER_OT14_Pos) ## !< 0x00004000
  GPIO_OTYPER_OT14* = GPIO_OTYPER_OT14_Msk
  GPIO_OTYPER_OT15_Pos* = (15)
  GPIO_OTYPER_OT15_Msk* = (0x00000001 shl GPIO_OTYPER_OT15_Pos) ## !< 0x00008000
  GPIO_OTYPER_OT15* = GPIO_OTYPER_OT15_Msk

##  Legacy defines

const
  GPIO_OTYPER_OT_0x* = GPIO_OTYPER_OT0
  GPIO_OTYPER_OT_1x* = GPIO_OTYPER_OT1
  GPIO_OTYPER_OT_2x* = GPIO_OTYPER_OT2
  GPIO_OTYPER_OT_3x* = GPIO_OTYPER_OT3
  GPIO_OTYPER_OT_4x* = GPIO_OTYPER_OT4
  GPIO_OTYPER_OT_5x* = GPIO_OTYPER_OT5
  GPIO_OTYPER_OT_6x* = GPIO_OTYPER_OT6
  GPIO_OTYPER_OT_7x* = GPIO_OTYPER_OT7
  GPIO_OTYPER_OT_8x* = GPIO_OTYPER_OT8
  GPIO_OTYPER_OT_9x* = GPIO_OTYPER_OT9
  GPIO_OTYPER_OT_10x* = GPIO_OTYPER_OT10
  GPIO_OTYPER_OT_11x* = GPIO_OTYPER_OT11
  GPIO_OTYPER_OT_12x* = GPIO_OTYPER_OT12
  GPIO_OTYPER_OT_13x* = GPIO_OTYPER_OT13
  GPIO_OTYPER_OT_14x* = GPIO_OTYPER_OT14
  GPIO_OTYPER_OT_15x* = GPIO_OTYPER_OT15

## *****************  Bits definition for GPIO_OSPEEDR register  **************

const
  GPIO_OSPEEDR_OSPEED0_Pos* = (0)
  GPIO_OSPEEDR_OSPEED0_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED0_Pos) ## !< 0x00000003
  GPIO_OSPEEDR_OSPEED0* = GPIO_OSPEEDR_OSPEED0_Msk
  GPIO_OSPEEDR_OSPEED0_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED0_Pos) ## !< 0x00000001
  GPIO_OSPEEDR_OSPEED0_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED0_Pos) ## !< 0x00000002
  GPIO_OSPEEDR_OSPEED1_Pos* = (2)
  GPIO_OSPEEDR_OSPEED1_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED1_Pos) ## !< 0x0000000C
  GPIO_OSPEEDR_OSPEED1* = GPIO_OSPEEDR_OSPEED1_Msk
  GPIO_OSPEEDR_OSPEED1_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED1_Pos) ## !< 0x00000004
  GPIO_OSPEEDR_OSPEED1_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED1_Pos) ## !< 0x00000008
  GPIO_OSPEEDR_OSPEED2_Pos* = (4)
  GPIO_OSPEEDR_OSPEED2_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED2_Pos) ## !< 0x00000030
  GPIO_OSPEEDR_OSPEED2* = GPIO_OSPEEDR_OSPEED2_Msk
  GPIO_OSPEEDR_OSPEED2_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED2_Pos) ## !< 0x00000010
  GPIO_OSPEEDR_OSPEED2_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED2_Pos) ## !< 0x00000020
  GPIO_OSPEEDR_OSPEED3_Pos* = (6)
  GPIO_OSPEEDR_OSPEED3_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED3_Pos) ## !< 0x000000C0
  GPIO_OSPEEDR_OSPEED3* = GPIO_OSPEEDR_OSPEED3_Msk
  GPIO_OSPEEDR_OSPEED3_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED3_Pos) ## !< 0x00000040
  GPIO_OSPEEDR_OSPEED3_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED3_Pos) ## !< 0x00000080
  GPIO_OSPEEDR_OSPEED4_Pos* = (8)
  GPIO_OSPEEDR_OSPEED4_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED4_Pos) ## !< 0x00000300
  GPIO_OSPEEDR_OSPEED4* = GPIO_OSPEEDR_OSPEED4_Msk
  GPIO_OSPEEDR_OSPEED4_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED4_Pos) ## !< 0x00000100
  GPIO_OSPEEDR_OSPEED4_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED4_Pos) ## !< 0x00000200
  GPIO_OSPEEDR_OSPEED5_Pos* = (10)
  GPIO_OSPEEDR_OSPEED5_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED5_Pos) ## !< 0x00000C00
  GPIO_OSPEEDR_OSPEED5* = GPIO_OSPEEDR_OSPEED5_Msk
  GPIO_OSPEEDR_OSPEED5_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED5_Pos) ## !< 0x00000400
  GPIO_OSPEEDR_OSPEED5_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED5_Pos) ## !< 0x00000800
  GPIO_OSPEEDR_OSPEED6_Pos* = (12)
  GPIO_OSPEEDR_OSPEED6_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED6_Pos) ## !< 0x00003000
  GPIO_OSPEEDR_OSPEED6* = GPIO_OSPEEDR_OSPEED6_Msk
  GPIO_OSPEEDR_OSPEED6_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED6_Pos) ## !< 0x00001000
  GPIO_OSPEEDR_OSPEED6_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED6_Pos) ## !< 0x00002000
  GPIO_OSPEEDR_OSPEED7_Pos* = (14)
  GPIO_OSPEEDR_OSPEED7_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED7_Pos) ## !< 0x0000C000
  GPIO_OSPEEDR_OSPEED7* = GPIO_OSPEEDR_OSPEED7_Msk
  GPIO_OSPEEDR_OSPEED7_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED7_Pos) ## !< 0x00004000
  GPIO_OSPEEDR_OSPEED7_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED7_Pos) ## !< 0x00008000
  GPIO_OSPEEDR_OSPEED8_Pos* = (16)
  GPIO_OSPEEDR_OSPEED8_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED8_Pos) ## !< 0x00030000
  GPIO_OSPEEDR_OSPEED8* = GPIO_OSPEEDR_OSPEED8_Msk
  GPIO_OSPEEDR_OSPEED8_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED8_Pos) ## !< 0x00010000
  GPIO_OSPEEDR_OSPEED8_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED8_Pos) ## !< 0x00020000
  GPIO_OSPEEDR_OSPEED9_Pos* = (18)
  GPIO_OSPEEDR_OSPEED9_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED9_Pos) ## !< 0x000C0000
  GPIO_OSPEEDR_OSPEED9* = GPIO_OSPEEDR_OSPEED9_Msk
  GPIO_OSPEEDR_OSPEED9_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED9_Pos) ## !< 0x00040000
  GPIO_OSPEEDR_OSPEED9_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED9_Pos) ## !< 0x00080000
  GPIO_OSPEEDR_OSPEED10_Pos* = (20)
  GPIO_OSPEEDR_OSPEED10_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED10_Pos) ## !< 0x00300000
  GPIO_OSPEEDR_OSPEED10x* = GPIO_OSPEEDR_OSPEED10_Msk
  GPIO_OSPEEDR_OSPEED10_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED10_Pos) ## !< 0x00100000
  GPIO_OSPEEDR_OSPEED10_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED10_Pos) ## !< 0x00200000
  GPIO_OSPEEDR_OSPEED11_Pos* = (22)
  GPIO_OSPEEDR_OSPEED11_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED11_Pos) ## !< 0x00C00000
  GPIO_OSPEEDR_OSPEED11x* = GPIO_OSPEEDR_OSPEED11_Msk
  GPIO_OSPEEDR_OSPEED11_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED11_Pos) ## !< 0x00400000
  GPIO_OSPEEDR_OSPEED11_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED11_Pos) ## !< 0x00800000
  GPIO_OSPEEDR_OSPEED12_Pos* = (24)
  GPIO_OSPEEDR_OSPEED12_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED12_Pos) ## !< 0x03000000
  GPIO_OSPEEDR_OSPEED12* = GPIO_OSPEEDR_OSPEED12_Msk
  GPIO_OSPEEDR_OSPEED12_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED12_Pos) ## !< 0x01000000
  GPIO_OSPEEDR_OSPEED12_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED12_Pos) ## !< 0x02000000
  GPIO_OSPEEDR_OSPEED13_Pos* = (26)
  GPIO_OSPEEDR_OSPEED13_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED13_Pos) ## !< 0x0C000000
  GPIO_OSPEEDR_OSPEED13* = GPIO_OSPEEDR_OSPEED13_Msk
  GPIO_OSPEEDR_OSPEED13_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED13_Pos) ## !< 0x04000000
  GPIO_OSPEEDR_OSPEED13_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED13_Pos) ## !< 0x08000000
  GPIO_OSPEEDR_OSPEED14_Pos* = (28)
  GPIO_OSPEEDR_OSPEED14_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED14_Pos) ## !< 0x30000000
  GPIO_OSPEEDR_OSPEED14* = GPIO_OSPEEDR_OSPEED14_Msk
  GPIO_OSPEEDR_OSPEED14_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED14_Pos) ## !< 0x10000000
  GPIO_OSPEEDR_OSPEED14_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED14_Pos) ## !< 0x20000000
  GPIO_OSPEEDR_OSPEED15_Pos* = (30)
  GPIO_OSPEEDR_OSPEED15_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED15_Pos) ## !< 0xC0000000
  GPIO_OSPEEDR_OSPEED15* = GPIO_OSPEEDR_OSPEED15_Msk
  GPIO_OSPEEDR_OSPEED15_0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED15_Pos) ## !< 0x40000000
  GPIO_OSPEEDR_OSPEED15_1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED15_Pos) ## !< 0x80000000

##  Legacy defines

const
  GPIO_OSPEEDER_OSPEEDR0* = GPIO_OSPEEDR_OSPEED0
  GPIO_OSPEEDER_OSPEEDR0_0* = GPIO_OSPEEDR_OSPEED0_0
  GPIO_OSPEEDER_OSPEEDR0_1* = GPIO_OSPEEDR_OSPEED0_1
  GPIO_OSPEEDER_OSPEEDR1* = GPIO_OSPEEDR_OSPEED1
  GPIO_OSPEEDER_OSPEEDR1_0* = GPIO_OSPEEDR_OSPEED1_0
  GPIO_OSPEEDER_OSPEEDR1_1* = GPIO_OSPEEDR_OSPEED1_1
  GPIO_OSPEEDER_OSPEEDR2* = GPIO_OSPEEDR_OSPEED2
  GPIO_OSPEEDER_OSPEEDR2_0* = GPIO_OSPEEDR_OSPEED2_0
  GPIO_OSPEEDER_OSPEEDR2_1* = GPIO_OSPEEDR_OSPEED2_1
  GPIO_OSPEEDER_OSPEEDR3* = GPIO_OSPEEDR_OSPEED3
  GPIO_OSPEEDER_OSPEEDR3_0* = GPIO_OSPEEDR_OSPEED3_0
  GPIO_OSPEEDER_OSPEEDR3_1* = GPIO_OSPEEDR_OSPEED3_1
  GPIO_OSPEEDER_OSPEEDR4* = GPIO_OSPEEDR_OSPEED4
  GPIO_OSPEEDER_OSPEEDR4_0* = GPIO_OSPEEDR_OSPEED4_0
  GPIO_OSPEEDER_OSPEEDR4_1* = GPIO_OSPEEDR_OSPEED4_1
  GPIO_OSPEEDER_OSPEEDR5* = GPIO_OSPEEDR_OSPEED5
  GPIO_OSPEEDER_OSPEEDR5_0* = GPIO_OSPEEDR_OSPEED5_0
  GPIO_OSPEEDER_OSPEEDR5_1* = GPIO_OSPEEDR_OSPEED5_1
  GPIO_OSPEEDER_OSPEEDR6* = GPIO_OSPEEDR_OSPEED6
  GPIO_OSPEEDER_OSPEEDR6_0* = GPIO_OSPEEDR_OSPEED6_0
  GPIO_OSPEEDER_OSPEEDR6_1* = GPIO_OSPEEDR_OSPEED6_1
  GPIO_OSPEEDER_OSPEEDR7* = GPIO_OSPEEDR_OSPEED7
  GPIO_OSPEEDER_OSPEEDR7_0* = GPIO_OSPEEDR_OSPEED7_0
  GPIO_OSPEEDER_OSPEEDR7_1* = GPIO_OSPEEDR_OSPEED7_1
  GPIO_OSPEEDER_OSPEEDR8* = GPIO_OSPEEDR_OSPEED8
  GPIO_OSPEEDER_OSPEEDR8_0* = GPIO_OSPEEDR_OSPEED8_0
  GPIO_OSPEEDER_OSPEEDR8_1* = GPIO_OSPEEDR_OSPEED8_1
  GPIO_OSPEEDER_OSPEEDR9* = GPIO_OSPEEDR_OSPEED9
  GPIO_OSPEEDER_OSPEEDR9_0* = GPIO_OSPEEDR_OSPEED9_0
  GPIO_OSPEEDER_OSPEEDR9_1* = GPIO_OSPEEDR_OSPEED9_1
  GPIO_OSPEEDER_OSPEEDR10x* = GPIO_OSPEEDR_OSPEED10
  GPIO_OSPEEDER_OSPEEDR10_0* = GPIO_OSPEEDR_OSPEED10_0
  GPIO_OSPEEDER_OSPEEDR10_1* = GPIO_OSPEEDR_OSPEED10_1
  GPIO_OSPEEDER_OSPEEDR11x* = GPIO_OSPEEDR_OSPEED11
  GPIO_OSPEEDER_OSPEEDR11_0* = GPIO_OSPEEDR_OSPEED11_0
  GPIO_OSPEEDER_OSPEEDR11_1* = GPIO_OSPEEDR_OSPEED11_1
  GPIO_OSPEEDER_OSPEEDR12* = GPIO_OSPEEDR_OSPEED12
  GPIO_OSPEEDER_OSPEEDR12_0* = GPIO_OSPEEDR_OSPEED12_0
  GPIO_OSPEEDER_OSPEEDR12_1* = GPIO_OSPEEDR_OSPEED12_1
  GPIO_OSPEEDER_OSPEEDR13* = GPIO_OSPEEDR_OSPEED13
  GPIO_OSPEEDER_OSPEEDR13_0* = GPIO_OSPEEDR_OSPEED13_0
  GPIO_OSPEEDER_OSPEEDR13_1* = GPIO_OSPEEDR_OSPEED13_1
  GPIO_OSPEEDER_OSPEEDR14* = GPIO_OSPEEDR_OSPEED14
  GPIO_OSPEEDER_OSPEEDR14_0* = GPIO_OSPEEDR_OSPEED14_0
  GPIO_OSPEEDER_OSPEEDR14_1* = GPIO_OSPEEDR_OSPEED14_1
  GPIO_OSPEEDER_OSPEEDR15* = GPIO_OSPEEDR_OSPEED15
  GPIO_OSPEEDER_OSPEEDR15_0* = GPIO_OSPEEDR_OSPEED15_0
  GPIO_OSPEEDER_OSPEEDR15_1* = GPIO_OSPEEDR_OSPEED15_1

## *****************  Bits definition for GPIO_PUPDR register  ****************

const
  GPIO_PUPDR_PUPD0_Pos* = (0)
  GPIO_PUPDR_PUPD0_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD0_Pos) ## !< 0x00000003
  GPIO_PUPDR_PUPD0* = GPIO_PUPDR_PUPD0_Msk
  GPIO_PUPDR_PUPD0_0* = (0x00000001 shl GPIO_PUPDR_PUPD0_Pos) ## !< 0x00000001
  GPIO_PUPDR_PUPD0_1* = (0x00000002 shl GPIO_PUPDR_PUPD0_Pos) ## !< 0x00000002
  GPIO_PUPDR_PUPD1_Pos* = (2)
  GPIO_PUPDR_PUPD1_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD1_Pos) ## !< 0x0000000C
  GPIO_PUPDR_PUPD1* = GPIO_PUPDR_PUPD1_Msk
  GPIO_PUPDR_PUPD1_0* = (0x00000001 shl GPIO_PUPDR_PUPD1_Pos) ## !< 0x00000004
  GPIO_PUPDR_PUPD1_1* = (0x00000002 shl GPIO_PUPDR_PUPD1_Pos) ## !< 0x00000008
  GPIO_PUPDR_PUPD2_Pos* = (4)
  GPIO_PUPDR_PUPD2_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD2_Pos) ## !< 0x00000030
  GPIO_PUPDR_PUPD2* = GPIO_PUPDR_PUPD2_Msk
  GPIO_PUPDR_PUPD2_0* = (0x00000001 shl GPIO_PUPDR_PUPD2_Pos) ## !< 0x00000010
  GPIO_PUPDR_PUPD2_1* = (0x00000002 shl GPIO_PUPDR_PUPD2_Pos) ## !< 0x00000020
  GPIO_PUPDR_PUPD3_Pos* = (6)
  GPIO_PUPDR_PUPD3_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD3_Pos) ## !< 0x000000C0
  GPIO_PUPDR_PUPD3* = GPIO_PUPDR_PUPD3_Msk
  GPIO_PUPDR_PUPD3_0* = (0x00000001 shl GPIO_PUPDR_PUPD3_Pos) ## !< 0x00000040
  GPIO_PUPDR_PUPD3_1* = (0x00000002 shl GPIO_PUPDR_PUPD3_Pos) ## !< 0x00000080
  GPIO_PUPDR_PUPD4_Pos* = (8)
  GPIO_PUPDR_PUPD4_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD4_Pos) ## !< 0x00000300
  GPIO_PUPDR_PUPD4* = GPIO_PUPDR_PUPD4_Msk
  GPIO_PUPDR_PUPD4_0* = (0x00000001 shl GPIO_PUPDR_PUPD4_Pos) ## !< 0x00000100
  GPIO_PUPDR_PUPD4_1* = (0x00000002 shl GPIO_PUPDR_PUPD4_Pos) ## !< 0x00000200
  GPIO_PUPDR_PUPD5_Pos* = (10)
  GPIO_PUPDR_PUPD5_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD5_Pos) ## !< 0x00000C00
  GPIO_PUPDR_PUPD5* = GPIO_PUPDR_PUPD5_Msk
  GPIO_PUPDR_PUPD5_0* = (0x00000001 shl GPIO_PUPDR_PUPD5_Pos) ## !< 0x00000400
  GPIO_PUPDR_PUPD5_1* = (0x00000002 shl GPIO_PUPDR_PUPD5_Pos) ## !< 0x00000800
  GPIO_PUPDR_PUPD6_Pos* = (12)
  GPIO_PUPDR_PUPD6_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD6_Pos) ## !< 0x00003000
  GPIO_PUPDR_PUPD6* = GPIO_PUPDR_PUPD6_Msk
  GPIO_PUPDR_PUPD6_0* = (0x00000001 shl GPIO_PUPDR_PUPD6_Pos) ## !< 0x00001000
  GPIO_PUPDR_PUPD6_1* = (0x00000002 shl GPIO_PUPDR_PUPD6_Pos) ## !< 0x00002000
  GPIO_PUPDR_PUPD7_Pos* = (14)
  GPIO_PUPDR_PUPD7_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD7_Pos) ## !< 0x0000C000
  GPIO_PUPDR_PUPD7* = GPIO_PUPDR_PUPD7_Msk
  GPIO_PUPDR_PUPD7_0* = (0x00000001 shl GPIO_PUPDR_PUPD7_Pos) ## !< 0x00004000
  GPIO_PUPDR_PUPD7_1* = (0x00000002 shl GPIO_PUPDR_PUPD7_Pos) ## !< 0x00008000
  GPIO_PUPDR_PUPD8_Pos* = (16)
  GPIO_PUPDR_PUPD8_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD8_Pos) ## !< 0x00030000
  GPIO_PUPDR_PUPD8* = GPIO_PUPDR_PUPD8_Msk
  GPIO_PUPDR_PUPD8_0* = (0x00000001 shl GPIO_PUPDR_PUPD8_Pos) ## !< 0x00010000
  GPIO_PUPDR_PUPD8_1* = (0x00000002 shl GPIO_PUPDR_PUPD8_Pos) ## !< 0x00020000
  GPIO_PUPDR_PUPD9_Pos* = (18)
  GPIO_PUPDR_PUPD9_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD9_Pos) ## !< 0x000C0000
  GPIO_PUPDR_PUPD9* = GPIO_PUPDR_PUPD9_Msk
  GPIO_PUPDR_PUPD9_0* = (0x00000001 shl GPIO_PUPDR_PUPD9_Pos) ## !< 0x00040000
  GPIO_PUPDR_PUPD9_1* = (0x00000002 shl GPIO_PUPDR_PUPD9_Pos) ## !< 0x00080000
  GPIO_PUPDR_PUPD10_Pos* = (20)
  GPIO_PUPDR_PUPD10_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD10_Pos) ## !< 0x00300000
  GPIO_PUPDR_PUPD10x* = GPIO_PUPDR_PUPD10_Msk
  GPIO_PUPDR_PUPD10_0* = (0x00000001 shl GPIO_PUPDR_PUPD10_Pos) ## !< 0x00100000
  GPIO_PUPDR_PUPD10_1* = (0x00000002 shl GPIO_PUPDR_PUPD10_Pos) ## !< 0x00200000
  GPIO_PUPDR_PUPD11_Pos* = (22)
  GPIO_PUPDR_PUPD11_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD11_Pos) ## !< 0x00C00000
  GPIO_PUPDR_PUPD11x* = GPIO_PUPDR_PUPD11_Msk
  GPIO_PUPDR_PUPD11_0* = (0x00000001 shl GPIO_PUPDR_PUPD11_Pos) ## !< 0x00400000
  GPIO_PUPDR_PUPD11_1* = (0x00000002 shl GPIO_PUPDR_PUPD11_Pos) ## !< 0x00800000
  GPIO_PUPDR_PUPD12_Pos* = (24)
  GPIO_PUPDR_PUPD12_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD12_Pos) ## !< 0x03000000
  GPIO_PUPDR_PUPD12* = GPIO_PUPDR_PUPD12_Msk
  GPIO_PUPDR_PUPD12_0* = (0x00000001 shl GPIO_PUPDR_PUPD12_Pos) ## !< 0x01000000
  GPIO_PUPDR_PUPD12_1* = (0x00000002 shl GPIO_PUPDR_PUPD12_Pos) ## !< 0x02000000
  GPIO_PUPDR_PUPD13_Pos* = (26)
  GPIO_PUPDR_PUPD13_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD13_Pos) ## !< 0x0C000000
  GPIO_PUPDR_PUPD13* = GPIO_PUPDR_PUPD13_Msk
  GPIO_PUPDR_PUPD13_0* = (0x00000001 shl GPIO_PUPDR_PUPD13_Pos) ## !< 0x04000000
  GPIO_PUPDR_PUPD13_1* = (0x00000002 shl GPIO_PUPDR_PUPD13_Pos) ## !< 0x08000000
  GPIO_PUPDR_PUPD14_Pos* = (28)
  GPIO_PUPDR_PUPD14_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD14_Pos) ## !< 0x30000000
  GPIO_PUPDR_PUPD14* = GPIO_PUPDR_PUPD14_Msk
  GPIO_PUPDR_PUPD14_0* = (0x00000001 shl GPIO_PUPDR_PUPD14_Pos) ## !< 0x10000000
  GPIO_PUPDR_PUPD14_1* = (0x00000002 shl GPIO_PUPDR_PUPD14_Pos) ## !< 0x20000000
  GPIO_PUPDR_PUPD15_Pos* = (30)
  GPIO_PUPDR_PUPD15_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD15_Pos) ## !< 0xC0000000
  GPIO_PUPDR_PUPD15* = GPIO_PUPDR_PUPD15_Msk
  GPIO_PUPDR_PUPD15_0* = (0x00000001 shl GPIO_PUPDR_PUPD15_Pos) ## !< 0x40000000
  GPIO_PUPDR_PUPD15_1* = (0x00000002 shl GPIO_PUPDR_PUPD15_Pos) ## !< 0x80000000

##  Legacy defines

const
  GPIO_PUPDR_PUPDR0* = GPIO_PUPDR_PUPD0
  GPIO_PUPDR_PUPDR0_0* = GPIO_PUPDR_PUPD0_0
  GPIO_PUPDR_PUPDR0_1* = GPIO_PUPDR_PUPD0_1
  GPIO_PUPDR_PUPDR1* = GPIO_PUPDR_PUPD1
  GPIO_PUPDR_PUPDR1_0* = GPIO_PUPDR_PUPD1_0
  GPIO_PUPDR_PUPDR1_1* = GPIO_PUPDR_PUPD1_1
  GPIO_PUPDR_PUPDR2* = GPIO_PUPDR_PUPD2
  GPIO_PUPDR_PUPDR2_0* = GPIO_PUPDR_PUPD2_0
  GPIO_PUPDR_PUPDR2_1* = GPIO_PUPDR_PUPD2_1
  GPIO_PUPDR_PUPDR3* = GPIO_PUPDR_PUPD3
  GPIO_PUPDR_PUPDR3_0* = GPIO_PUPDR_PUPD3_0
  GPIO_PUPDR_PUPDR3_1* = GPIO_PUPDR_PUPD3_1
  GPIO_PUPDR_PUPDR4* = GPIO_PUPDR_PUPD4
  GPIO_PUPDR_PUPDR4_0* = GPIO_PUPDR_PUPD4_0
  GPIO_PUPDR_PUPDR4_1* = GPIO_PUPDR_PUPD4_1
  GPIO_PUPDR_PUPDR5* = GPIO_PUPDR_PUPD5
  GPIO_PUPDR_PUPDR5_0* = GPIO_PUPDR_PUPD5_0
  GPIO_PUPDR_PUPDR5_1* = GPIO_PUPDR_PUPD5_1
  GPIO_PUPDR_PUPDR6* = GPIO_PUPDR_PUPD6
  GPIO_PUPDR_PUPDR6_0* = GPIO_PUPDR_PUPD6_0
  GPIO_PUPDR_PUPDR6_1* = GPIO_PUPDR_PUPD6_1
  GPIO_PUPDR_PUPDR7* = GPIO_PUPDR_PUPD7
  GPIO_PUPDR_PUPDR7_0* = GPIO_PUPDR_PUPD7_0
  GPIO_PUPDR_PUPDR7_1* = GPIO_PUPDR_PUPD7_1
  GPIO_PUPDR_PUPDR8* = GPIO_PUPDR_PUPD8
  GPIO_PUPDR_PUPDR8_0* = GPIO_PUPDR_PUPD8_0
  GPIO_PUPDR_PUPDR8_1* = GPIO_PUPDR_PUPD8_1
  GPIO_PUPDR_PUPDR9* = GPIO_PUPDR_PUPD9
  GPIO_PUPDR_PUPDR9_0* = GPIO_PUPDR_PUPD9_0
  GPIO_PUPDR_PUPDR9_1* = GPIO_PUPDR_PUPD9_1
  GPIO_PUPDR_PUPDR10x* = GPIO_PUPDR_PUPD10
  GPIO_PUPDR_PUPDR10_0* = GPIO_PUPDR_PUPD10_0
  GPIO_PUPDR_PUPDR10_1* = GPIO_PUPDR_PUPD10_1
  GPIO_PUPDR_PUPDR11x* = GPIO_PUPDR_PUPD11
  GPIO_PUPDR_PUPDR11_0* = GPIO_PUPDR_PUPD11_0
  GPIO_PUPDR_PUPDR11_1* = GPIO_PUPDR_PUPD11_1
  GPIO_PUPDR_PUPDR12* = GPIO_PUPDR_PUPD12
  GPIO_PUPDR_PUPDR12_0* = GPIO_PUPDR_PUPD12_0
  GPIO_PUPDR_PUPDR12_1* = GPIO_PUPDR_PUPD12_1
  GPIO_PUPDR_PUPDR13* = GPIO_PUPDR_PUPD13
  GPIO_PUPDR_PUPDR13_0* = GPIO_PUPDR_PUPD13_0
  GPIO_PUPDR_PUPDR13_1* = GPIO_PUPDR_PUPD13_1
  GPIO_PUPDR_PUPDR14* = GPIO_PUPDR_PUPD14
  GPIO_PUPDR_PUPDR14_0* = GPIO_PUPDR_PUPD14_0
  GPIO_PUPDR_PUPDR14_1* = GPIO_PUPDR_PUPD14_1
  GPIO_PUPDR_PUPDR15* = GPIO_PUPDR_PUPD15
  GPIO_PUPDR_PUPDR15_0* = GPIO_PUPDR_PUPD15_0
  GPIO_PUPDR_PUPDR15_1* = GPIO_PUPDR_PUPD15_1

## *****************  Bits definition for GPIO_IDR register  ******************

const
  GPIO_IDR_ID0_Pos* = (0)
  GPIO_IDR_ID0_Msk* = (0x00000001 shl GPIO_IDR_ID0_Pos) ## !< 0x00000001
  GPIO_IDR_ID0* = GPIO_IDR_ID0_Msk
  GPIO_IDR_ID1_Pos* = (1)
  GPIO_IDR_ID1_Msk* = (0x00000001 shl GPIO_IDR_ID1_Pos) ## !< 0x00000002
  GPIO_IDR_ID1* = GPIO_IDR_ID1_Msk
  GPIO_IDR_ID2_Pos* = (2)
  GPIO_IDR_ID2_Msk* = (0x00000001 shl GPIO_IDR_ID2_Pos) ## !< 0x00000004
  GPIO_IDR_ID2* = GPIO_IDR_ID2_Msk
  GPIO_IDR_ID3_Pos* = (3)
  GPIO_IDR_ID3_Msk* = (0x00000001 shl GPIO_IDR_ID3_Pos) ## !< 0x00000008
  GPIO_IDR_ID3* = GPIO_IDR_ID3_Msk
  GPIO_IDR_ID4_Pos* = (4)
  GPIO_IDR_ID4_Msk* = (0x00000001 shl GPIO_IDR_ID4_Pos) ## !< 0x00000010
  GPIO_IDR_ID4* = GPIO_IDR_ID4_Msk
  GPIO_IDR_ID5_Pos* = (5)
  GPIO_IDR_ID5_Msk* = (0x00000001 shl GPIO_IDR_ID5_Pos) ## !< 0x00000020
  GPIO_IDR_ID5* = GPIO_IDR_ID5_Msk
  GPIO_IDR_ID6_Pos* = (6)
  GPIO_IDR_ID6_Msk* = (0x00000001 shl GPIO_IDR_ID6_Pos) ## !< 0x00000040
  GPIO_IDR_ID6* = GPIO_IDR_ID6_Msk
  GPIO_IDR_ID7_Pos* = (7)
  GPIO_IDR_ID7_Msk* = (0x00000001 shl GPIO_IDR_ID7_Pos) ## !< 0x00000080
  GPIO_IDR_ID7* = GPIO_IDR_ID7_Msk
  GPIO_IDR_ID8_Pos* = (8)
  GPIO_IDR_ID8_Msk* = (0x00000001 shl GPIO_IDR_ID8_Pos) ## !< 0x00000100
  GPIO_IDR_ID8* = GPIO_IDR_ID8_Msk
  GPIO_IDR_ID9_Pos* = (9)
  GPIO_IDR_ID9_Msk* = (0x00000001 shl GPIO_IDR_ID9_Pos) ## !< 0x00000200
  GPIO_IDR_ID9* = GPIO_IDR_ID9_Msk
  GPIO_IDR_ID10_Pos* = (10)
  GPIO_IDR_ID10_Msk* = (0x00000001 shl GPIO_IDR_ID10_Pos) ## !< 0x00000400
  GPIO_IDR_ID10* = GPIO_IDR_ID10_Msk
  GPIO_IDR_ID11_Pos* = (11)
  GPIO_IDR_ID11_Msk* = (0x00000001 shl GPIO_IDR_ID11_Pos) ## !< 0x00000800
  GPIO_IDR_ID11* = GPIO_IDR_ID11_Msk
  GPIO_IDR_ID12_Pos* = (12)
  GPIO_IDR_ID12_Msk* = (0x00000001 shl GPIO_IDR_ID12_Pos) ## !< 0x00001000
  GPIO_IDR_ID12* = GPIO_IDR_ID12_Msk
  GPIO_IDR_ID13_Pos* = (13)
  GPIO_IDR_ID13_Msk* = (0x00000001 shl GPIO_IDR_ID13_Pos) ## !< 0x00002000
  GPIO_IDR_ID13* = GPIO_IDR_ID13_Msk
  GPIO_IDR_ID14_Pos* = (14)
  GPIO_IDR_ID14_Msk* = (0x00000001 shl GPIO_IDR_ID14_Pos) ## !< 0x00004000
  GPIO_IDR_ID14* = GPIO_IDR_ID14_Msk
  GPIO_IDR_ID15_Pos* = (15)
  GPIO_IDR_ID15_Msk* = (0x00000001 shl GPIO_IDR_ID15_Pos) ## !< 0x00008000
  GPIO_IDR_ID15* = GPIO_IDR_ID15_Msk

##  Legacy defines

const
  GPIO_IDR_IDR_0* = GPIO_IDR_ID0
  GPIO_IDR_IDR_1* = GPIO_IDR_ID1
  GPIO_IDR_IDR_2* = GPIO_IDR_ID2
  GPIO_IDR_IDR_3* = GPIO_IDR_ID3
  GPIO_IDR_IDR_4* = GPIO_IDR_ID4
  GPIO_IDR_IDR_5* = GPIO_IDR_ID5
  GPIO_IDR_IDR_6* = GPIO_IDR_ID6
  GPIO_IDR_IDR_7* = GPIO_IDR_ID7
  GPIO_IDR_IDR_8* = GPIO_IDR_ID8
  GPIO_IDR_IDR_9* = GPIO_IDR_ID9
  GPIO_IDR_IDR_10* = GPIO_IDR_ID10
  GPIO_IDR_IDR_11* = GPIO_IDR_ID11
  GPIO_IDR_IDR_12* = GPIO_IDR_ID12
  GPIO_IDR_IDR_13* = GPIO_IDR_ID13
  GPIO_IDR_IDR_14* = GPIO_IDR_ID14
  GPIO_IDR_IDR_15* = GPIO_IDR_ID15

## *****************  Bits definition for GPIO_ODR register  ******************

const
  GPIO_ODR_OD0_Pos* = (0)
  GPIO_ODR_OD0_Msk* = (0x00000001 shl GPIO_ODR_OD0_Pos) ## !< 0x00000001
  GPIO_ODR_OD0* = GPIO_ODR_OD0_Msk
  GPIO_ODR_OD1_Pos* = (1)
  GPIO_ODR_OD1_Msk* = (0x00000001 shl GPIO_ODR_OD1_Pos) ## !< 0x00000002
  GPIO_ODR_OD1* = GPIO_ODR_OD1_Msk
  GPIO_ODR_OD2_Pos* = (2)
  GPIO_ODR_OD2_Msk* = (0x00000001 shl GPIO_ODR_OD2_Pos) ## !< 0x00000004
  GPIO_ODR_OD2* = GPIO_ODR_OD2_Msk
  GPIO_ODR_OD3_Pos* = (3)
  GPIO_ODR_OD3_Msk* = (0x00000001 shl GPIO_ODR_OD3_Pos) ## !< 0x00000008
  GPIO_ODR_OD3* = GPIO_ODR_OD3_Msk
  GPIO_ODR_OD4_Pos* = (4)
  GPIO_ODR_OD4_Msk* = (0x00000001 shl GPIO_ODR_OD4_Pos) ## !< 0x00000010
  GPIO_ODR_OD4* = GPIO_ODR_OD4_Msk
  GPIO_ODR_OD5_Pos* = (5)
  GPIO_ODR_OD5_Msk* = (0x00000001 shl GPIO_ODR_OD5_Pos) ## !< 0x00000020
  GPIO_ODR_OD5* = GPIO_ODR_OD5_Msk
  GPIO_ODR_OD6_Pos* = (6)
  GPIO_ODR_OD6_Msk* = (0x00000001 shl GPIO_ODR_OD6_Pos) ## !< 0x00000040
  GPIO_ODR_OD6* = GPIO_ODR_OD6_Msk
  GPIO_ODR_OD7_Pos* = (7)
  GPIO_ODR_OD7_Msk* = (0x00000001 shl GPIO_ODR_OD7_Pos) ## !< 0x00000080
  GPIO_ODR_OD7* = GPIO_ODR_OD7_Msk
  GPIO_ODR_OD8_Pos* = (8)
  GPIO_ODR_OD8_Msk* = (0x00000001 shl GPIO_ODR_OD8_Pos) ## !< 0x00000100
  GPIO_ODR_OD8* = GPIO_ODR_OD8_Msk
  GPIO_ODR_OD9_Pos* = (9)
  GPIO_ODR_OD9_Msk* = (0x00000001 shl GPIO_ODR_OD9_Pos) ## !< 0x00000200
  GPIO_ODR_OD9* = GPIO_ODR_OD9_Msk
  GPIO_ODR_OD10_Pos* = (10)
  GPIO_ODR_OD10_Msk* = (0x00000001 shl GPIO_ODR_OD10_Pos) ## !< 0x00000400
  GPIO_ODR_OD10* = GPIO_ODR_OD10_Msk
  GPIO_ODR_OD11_Pos* = (11)
  GPIO_ODR_OD11_Msk* = (0x00000001 shl GPIO_ODR_OD11_Pos) ## !< 0x00000800
  GPIO_ODR_OD11* = GPIO_ODR_OD11_Msk
  GPIO_ODR_OD12_Pos* = (12)
  GPIO_ODR_OD12_Msk* = (0x00000001 shl GPIO_ODR_OD12_Pos) ## !< 0x00001000
  GPIO_ODR_OD12* = GPIO_ODR_OD12_Msk
  GPIO_ODR_OD13_Pos* = (13)
  GPIO_ODR_OD13_Msk* = (0x00000001 shl GPIO_ODR_OD13_Pos) ## !< 0x00002000
  GPIO_ODR_OD13* = GPIO_ODR_OD13_Msk
  GPIO_ODR_OD14_Pos* = (14)
  GPIO_ODR_OD14_Msk* = (0x00000001 shl GPIO_ODR_OD14_Pos) ## !< 0x00004000
  GPIO_ODR_OD14* = GPIO_ODR_OD14_Msk
  GPIO_ODR_OD15_Pos* = (15)
  GPIO_ODR_OD15_Msk* = (0x00000001 shl GPIO_ODR_OD15_Pos) ## !< 0x00008000
  GPIO_ODR_OD15* = GPIO_ODR_OD15_Msk

##  Legacy defines

const
  GPIO_ODR_ODR_0* = GPIO_ODR_OD0
  GPIO_ODR_ODR_1* = GPIO_ODR_OD1
  GPIO_ODR_ODR_2* = GPIO_ODR_OD2
  GPIO_ODR_ODR_3* = GPIO_ODR_OD3
  GPIO_ODR_ODR_4* = GPIO_ODR_OD4
  GPIO_ODR_ODR_5* = GPIO_ODR_OD5
  GPIO_ODR_ODR_6* = GPIO_ODR_OD6
  GPIO_ODR_ODR_7* = GPIO_ODR_OD7
  GPIO_ODR_ODR_8* = GPIO_ODR_OD8
  GPIO_ODR_ODR_9* = GPIO_ODR_OD9
  GPIO_ODR_ODR_10* = GPIO_ODR_OD10
  GPIO_ODR_ODR_11* = GPIO_ODR_OD11
  GPIO_ODR_ODR_12* = GPIO_ODR_OD12
  GPIO_ODR_ODR_13* = GPIO_ODR_OD13
  GPIO_ODR_ODR_14* = GPIO_ODR_OD14
  GPIO_ODR_ODR_15* = GPIO_ODR_OD15

## *****************  Bits definition for GPIO_BSRR register  *****************

const
  GPIO_BSRR_BS0_Pos* = (0)
  GPIO_BSRR_BS0_Msk* = (0x00000001 shl GPIO_BSRR_BS0_Pos) ## !< 0x00000001
  GPIO_BSRR_BS0* = GPIO_BSRR_BS0_Msk
  GPIO_BSRR_BS1_Pos* = (1)
  GPIO_BSRR_BS1_Msk* = (0x00000001 shl GPIO_BSRR_BS1_Pos) ## !< 0x00000002
  GPIO_BSRR_BS1* = GPIO_BSRR_BS1_Msk
  GPIO_BSRR_BS2_Pos* = (2)
  GPIO_BSRR_BS2_Msk* = (0x00000001 shl GPIO_BSRR_BS2_Pos) ## !< 0x00000004
  GPIO_BSRR_BS2* = GPIO_BSRR_BS2_Msk
  GPIO_BSRR_BS3_Pos* = (3)
  GPIO_BSRR_BS3_Msk* = (0x00000001 shl GPIO_BSRR_BS3_Pos) ## !< 0x00000008
  GPIO_BSRR_BS3* = GPIO_BSRR_BS3_Msk
  GPIO_BSRR_BS4_Pos* = (4)
  GPIO_BSRR_BS4_Msk* = (0x00000001 shl GPIO_BSRR_BS4_Pos) ## !< 0x00000010
  GPIO_BSRR_BS4* = GPIO_BSRR_BS4_Msk
  GPIO_BSRR_BS5_Pos* = (5)
  GPIO_BSRR_BS5_Msk* = (0x00000001 shl GPIO_BSRR_BS5_Pos) ## !< 0x00000020
  GPIO_BSRR_BS5* = GPIO_BSRR_BS5_Msk
  GPIO_BSRR_BS6_Pos* = (6)
  GPIO_BSRR_BS6_Msk* = (0x00000001 shl GPIO_BSRR_BS6_Pos) ## !< 0x00000040
  GPIO_BSRR_BS6* = GPIO_BSRR_BS6_Msk
  GPIO_BSRR_BS7_Pos* = (7)
  GPIO_BSRR_BS7_Msk* = (0x00000001 shl GPIO_BSRR_BS7_Pos) ## !< 0x00000080
  GPIO_BSRR_BS7* = GPIO_BSRR_BS7_Msk
  GPIO_BSRR_BS8_Pos* = (8)
  GPIO_BSRR_BS8_Msk* = (0x00000001 shl GPIO_BSRR_BS8_Pos) ## !< 0x00000100
  GPIO_BSRR_BS8* = GPIO_BSRR_BS8_Msk
  GPIO_BSRR_BS9_Pos* = (9)
  GPIO_BSRR_BS9_Msk* = (0x00000001 shl GPIO_BSRR_BS9_Pos) ## !< 0x00000200
  GPIO_BSRR_BS9* = GPIO_BSRR_BS9_Msk
  GPIO_BSRR_BS10_Pos* = (10)
  GPIO_BSRR_BS10_Msk* = (0x00000001 shl GPIO_BSRR_BS10_Pos) ## !< 0x00000400
  GPIO_BSRR_BS10* = GPIO_BSRR_BS10_Msk
  GPIO_BSRR_BS11_Pos* = (11)
  GPIO_BSRR_BS11_Msk* = (0x00000001 shl GPIO_BSRR_BS11_Pos) ## !< 0x00000800
  GPIO_BSRR_BS11* = GPIO_BSRR_BS11_Msk
  GPIO_BSRR_BS12_Pos* = (12)
  GPIO_BSRR_BS12_Msk* = (0x00000001 shl GPIO_BSRR_BS12_Pos) ## !< 0x00001000
  GPIO_BSRR_BS12* = GPIO_BSRR_BS12_Msk
  GPIO_BSRR_BS13_Pos* = (13)
  GPIO_BSRR_BS13_Msk* = (0x00000001 shl GPIO_BSRR_BS13_Pos) ## !< 0x00002000
  GPIO_BSRR_BS13* = GPIO_BSRR_BS13_Msk
  GPIO_BSRR_BS14_Pos* = (14)
  GPIO_BSRR_BS14_Msk* = (0x00000001 shl GPIO_BSRR_BS14_Pos) ## !< 0x00004000
  GPIO_BSRR_BS14* = GPIO_BSRR_BS14_Msk
  GPIO_BSRR_BS15_Pos* = (15)
  GPIO_BSRR_BS15_Msk* = (0x00000001 shl GPIO_BSRR_BS15_Pos) ## !< 0x00008000
  GPIO_BSRR_BS15* = GPIO_BSRR_BS15_Msk
  GPIO_BSRR_BR0_Pos* = (16)
  GPIO_BSRR_BR0_Msk* = (0x00000001 shl GPIO_BSRR_BR0_Pos) ## !< 0x00010000
  GPIO_BSRR_BR0* = GPIO_BSRR_BR0_Msk
  GPIO_BSRR_BR1_Pos* = (17)
  GPIO_BSRR_BR1_Msk* = (0x00000001 shl GPIO_BSRR_BR1_Pos) ## !< 0x00020000
  GPIO_BSRR_BR1* = GPIO_BSRR_BR1_Msk
  GPIO_BSRR_BR2_Pos* = (18)
  GPIO_BSRR_BR2_Msk* = (0x00000001 shl GPIO_BSRR_BR2_Pos) ## !< 0x00040000
  GPIO_BSRR_BR2* = GPIO_BSRR_BR2_Msk
  GPIO_BSRR_BR3_Pos* = (19)
  GPIO_BSRR_BR3_Msk* = (0x00000001 shl GPIO_BSRR_BR3_Pos) ## !< 0x00080000
  GPIO_BSRR_BR3* = GPIO_BSRR_BR3_Msk
  GPIO_BSRR_BR4_Pos* = (20)
  GPIO_BSRR_BR4_Msk* = (0x00000001 shl GPIO_BSRR_BR4_Pos) ## !< 0x00100000
  GPIO_BSRR_BR4* = GPIO_BSRR_BR4_Msk
  GPIO_BSRR_BR5_Pos* = (21)
  GPIO_BSRR_BR5_Msk* = (0x00000001 shl GPIO_BSRR_BR5_Pos) ## !< 0x00200000
  GPIO_BSRR_BR5* = GPIO_BSRR_BR5_Msk
  GPIO_BSRR_BR6_Pos* = (22)
  GPIO_BSRR_BR6_Msk* = (0x00000001 shl GPIO_BSRR_BR6_Pos) ## !< 0x00400000
  GPIO_BSRR_BR6* = GPIO_BSRR_BR6_Msk
  GPIO_BSRR_BR7_Pos* = (23)
  GPIO_BSRR_BR7_Msk* = (0x00000001 shl GPIO_BSRR_BR7_Pos) ## !< 0x00800000
  GPIO_BSRR_BR7* = GPIO_BSRR_BR7_Msk
  GPIO_BSRR_BR8_Pos* = (24)
  GPIO_BSRR_BR8_Msk* = (0x00000001 shl GPIO_BSRR_BR8_Pos) ## !< 0x01000000
  GPIO_BSRR_BR8* = GPIO_BSRR_BR8_Msk
  GPIO_BSRR_BR9_Pos* = (25)
  GPIO_BSRR_BR9_Msk* = (0x00000001 shl GPIO_BSRR_BR9_Pos) ## !< 0x02000000
  GPIO_BSRR_BR9* = GPIO_BSRR_BR9_Msk
  GPIO_BSRR_BR10_Pos* = (26)
  GPIO_BSRR_BR10_Msk* = (0x00000001 shl GPIO_BSRR_BR10_Pos) ## !< 0x04000000
  GPIO_BSRR_BR10* = GPIO_BSRR_BR10_Msk
  GPIO_BSRR_BR11_Pos* = (27)
  GPIO_BSRR_BR11_Msk* = (0x00000001 shl GPIO_BSRR_BR11_Pos) ## !< 0x08000000
  GPIO_BSRR_BR11* = GPIO_BSRR_BR11_Msk
  GPIO_BSRR_BR12_Pos* = (28)
  GPIO_BSRR_BR12_Msk* = (0x00000001 shl GPIO_BSRR_BR12_Pos) ## !< 0x10000000
  GPIO_BSRR_BR12* = GPIO_BSRR_BR12_Msk
  GPIO_BSRR_BR13_Pos* = (29)
  GPIO_BSRR_BR13_Msk* = (0x00000001 shl GPIO_BSRR_BR13_Pos) ## !< 0x20000000
  GPIO_BSRR_BR13* = GPIO_BSRR_BR13_Msk
  GPIO_BSRR_BR14_Pos* = (30)
  GPIO_BSRR_BR14_Msk* = (0x00000001 shl GPIO_BSRR_BR14_Pos) ## !< 0x40000000
  GPIO_BSRR_BR14* = GPIO_BSRR_BR14_Msk
  GPIO_BSRR_BR15_Pos* = (31)
  GPIO_BSRR_BR15_Msk* = (0x00000001 shl GPIO_BSRR_BR15_Pos) ## !< 0x80000000
  GPIO_BSRR_BR15* = GPIO_BSRR_BR15_Msk

##  Legacy defines

const
  GPIO_BSRR_BS_0x* = GPIO_BSRR_BS0
  GPIO_BSRR_BS_1x* = GPIO_BSRR_BS1
  GPIO_BSRR_BS_2x* = GPIO_BSRR_BS2
  GPIO_BSRR_BS_3x* = GPIO_BSRR_BS3
  GPIO_BSRR_BS_4x* = GPIO_BSRR_BS4
  GPIO_BSRR_BS_5x* = GPIO_BSRR_BS5
  GPIO_BSRR_BS_6x* = GPIO_BSRR_BS6
  GPIO_BSRR_BS_7x* = GPIO_BSRR_BS7
  GPIO_BSRR_BS_8x* = GPIO_BSRR_BS8
  GPIO_BSRR_BS_9x* = GPIO_BSRR_BS9
  GPIO_BSRR_BS_10x* = GPIO_BSRR_BS10
  GPIO_BSRR_BS_11x* = GPIO_BSRR_BS11
  GPIO_BSRR_BS_12x* = GPIO_BSRR_BS12
  GPIO_BSRR_BS_13x* = GPIO_BSRR_BS13
  GPIO_BSRR_BS_14x* = GPIO_BSRR_BS14
  GPIO_BSRR_BS_15x* = GPIO_BSRR_BS15
  GPIO_BSRR_BR_0x* = GPIO_BSRR_BR0
  GPIO_BSRR_BR_1x* = GPIO_BSRR_BR1
  GPIO_BSRR_BR_2x* = GPIO_BSRR_BR2
  GPIO_BSRR_BR_3x* = GPIO_BSRR_BR3
  GPIO_BSRR_BR_4x* = GPIO_BSRR_BR4
  GPIO_BSRR_BR_5x* = GPIO_BSRR_BR5
  GPIO_BSRR_BR_6x* = GPIO_BSRR_BR6
  GPIO_BSRR_BR_7x* = GPIO_BSRR_BR7
  GPIO_BSRR_BR_8x* = GPIO_BSRR_BR8
  GPIO_BSRR_BR_9x* = GPIO_BSRR_BR9
  GPIO_BSRR_BR_10x* = GPIO_BSRR_BR10
  GPIO_BSRR_BR_11x* = GPIO_BSRR_BR11
  GPIO_BSRR_BR_12x* = GPIO_BSRR_BR12
  GPIO_BSRR_BR_13x* = GPIO_BSRR_BR13
  GPIO_BSRR_BR_14x* = GPIO_BSRR_BR14
  GPIO_BSRR_BR_15x* = GPIO_BSRR_BR15
  GPIO_BRR_BR0* = GPIO_BSRR_BR0
  GPIO_BRR_BR0_Pos* = GPIO_BSRR_BR0_Pos
  GPIO_BRR_BR0_Msk* = GPIO_BSRR_BR0_Msk
  GPIO_BRR_BR1* = GPIO_BSRR_BR1
  GPIO_BRR_BR1_Pos* = GPIO_BSRR_BR1_Pos
  GPIO_BRR_BR1_Msk* = GPIO_BSRR_BR1_Msk
  GPIO_BRR_BR2* = GPIO_BSRR_BR2
  GPIO_BRR_BR2_Pos* = GPIO_BSRR_BR2_Pos
  GPIO_BRR_BR2_Msk* = GPIO_BSRR_BR2_Msk
  GPIO_BRR_BR3* = GPIO_BSRR_BR3
  GPIO_BRR_BR3_Pos* = GPIO_BSRR_BR3_Pos
  GPIO_BRR_BR3_Msk* = GPIO_BSRR_BR3_Msk
  GPIO_BRR_BR4* = GPIO_BSRR_BR4
  GPIO_BRR_BR4_Pos* = GPIO_BSRR_BR4_Pos
  GPIO_BRR_BR4_Msk* = GPIO_BSRR_BR4_Msk
  GPIO_BRR_BR5* = GPIO_BSRR_BR5
  GPIO_BRR_BR5_Pos* = GPIO_BSRR_BR5_Pos
  GPIO_BRR_BR5_Msk* = GPIO_BSRR_BR5_Msk
  GPIO_BRR_BR6* = GPIO_BSRR_BR6
  GPIO_BRR_BR6_Pos* = GPIO_BSRR_BR6_Pos
  GPIO_BRR_BR6_Msk* = GPIO_BSRR_BR6_Msk
  GPIO_BRR_BR7* = GPIO_BSRR_BR7
  GPIO_BRR_BR7_Pos* = GPIO_BSRR_BR7_Pos
  GPIO_BRR_BR7_Msk* = GPIO_BSRR_BR7_Msk
  GPIO_BRR_BR8* = GPIO_BSRR_BR8
  GPIO_BRR_BR8_Pos* = GPIO_BSRR_BR8_Pos
  GPIO_BRR_BR8_Msk* = GPIO_BSRR_BR8_Msk
  GPIO_BRR_BR9* = GPIO_BSRR_BR9
  GPIO_BRR_BR9_Pos* = GPIO_BSRR_BR9_Pos
  GPIO_BRR_BR9_Msk* = GPIO_BSRR_BR9_Msk
  GPIO_BRR_BR10* = GPIO_BSRR_BR10
  GPIO_BRR_BR10_Pos* = GPIO_BSRR_BR10_Pos
  GPIO_BRR_BR10_Msk* = GPIO_BSRR_BR10_Msk
  GPIO_BRR_BR11* = GPIO_BSRR_BR11
  GPIO_BRR_BR11_Pos* = GPIO_BSRR_BR11_Pos
  GPIO_BRR_BR11_Msk* = GPIO_BSRR_BR11_Msk
  GPIO_BRR_BR12* = GPIO_BSRR_BR12
  GPIO_BRR_BR12_Pos* = GPIO_BSRR_BR12_Pos
  GPIO_BRR_BR12_Msk* = GPIO_BSRR_BR12_Msk
  GPIO_BRR_BR13* = GPIO_BSRR_BR13
  GPIO_BRR_BR13_Pos* = GPIO_BSRR_BR13_Pos
  GPIO_BRR_BR13_Msk* = GPIO_BSRR_BR13_Msk
  GPIO_BRR_BR14* = GPIO_BSRR_BR14
  GPIO_BRR_BR14_Pos* = GPIO_BSRR_BR14_Pos
  GPIO_BRR_BR14_Msk* = GPIO_BSRR_BR14_Msk
  GPIO_BRR_BR15* = GPIO_BSRR_BR15
  GPIO_BRR_BR15_Pos* = GPIO_BSRR_BR15_Pos
  GPIO_BRR_BR15_Msk* = GPIO_BSRR_BR15_Msk

## ***************** Bit definition for GPIO_LCKR register ********************

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

## ***************** Bit definition for GPIO_AFRL register ********************

const
  GPIO_AFRL_AFSEL0_Pos* = (0)
  GPIO_AFRL_AFSEL0_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL0_Pos) ## !< 0x0000000F
  GPIO_AFRL_AFSEL0* = GPIO_AFRL_AFSEL0_Msk
  GPIO_AFRL_AFSEL0_0* = (0x00000001 shl GPIO_AFRL_AFSEL0_Pos) ## !< 0x00000001
  GPIO_AFRL_AFSEL0_1* = (0x00000002 shl GPIO_AFRL_AFSEL0_Pos) ## !< 0x00000002
  GPIO_AFRL_AFSEL0_2* = (0x00000004 shl GPIO_AFRL_AFSEL0_Pos) ## !< 0x00000004
  GPIO_AFRL_AFSEL0_3* = (0x00000008 shl GPIO_AFRL_AFSEL0_Pos) ## !< 0x00000008
  GPIO_AFRL_AFSEL1_Pos* = (4)
  GPIO_AFRL_AFSEL1_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL1_Pos) ## !< 0x000000F0
  GPIO_AFRL_AFSEL1* = GPIO_AFRL_AFSEL1_Msk
  GPIO_AFRL_AFSEL1_0* = (0x00000001 shl GPIO_AFRL_AFSEL1_Pos) ## !< 0x00000010
  GPIO_AFRL_AFSEL1_1* = (0x00000002 shl GPIO_AFRL_AFSEL1_Pos) ## !< 0x00000020
  GPIO_AFRL_AFSEL1_2* = (0x00000004 shl GPIO_AFRL_AFSEL1_Pos) ## !< 0x00000040
  GPIO_AFRL_AFSEL1_3* = (0x00000008 shl GPIO_AFRL_AFSEL1_Pos) ## !< 0x00000080
  GPIO_AFRL_AFSEL2_Pos* = (8)
  GPIO_AFRL_AFSEL2_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL2_Pos) ## !< 0x00000F00
  GPIO_AFRL_AFSEL2* = GPIO_AFRL_AFSEL2_Msk
  GPIO_AFRL_AFSEL2_0* = (0x00000001 shl GPIO_AFRL_AFSEL2_Pos) ## !< 0x00000100
  GPIO_AFRL_AFSEL2_1* = (0x00000002 shl GPIO_AFRL_AFSEL2_Pos) ## !< 0x00000200
  GPIO_AFRL_AFSEL2_2* = (0x00000004 shl GPIO_AFRL_AFSEL2_Pos) ## !< 0x00000400
  GPIO_AFRL_AFSEL2_3* = (0x00000008 shl GPIO_AFRL_AFSEL2_Pos) ## !< 0x00000800
  GPIO_AFRL_AFSEL3_Pos* = (12)
  GPIO_AFRL_AFSEL3_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL3_Pos) ## !< 0x0000F000
  GPIO_AFRL_AFSEL3* = GPIO_AFRL_AFSEL3_Msk
  GPIO_AFRL_AFSEL3_0* = (0x00000001 shl GPIO_AFRL_AFSEL3_Pos) ## !< 0x00001000
  GPIO_AFRL_AFSEL3_1* = (0x00000002 shl GPIO_AFRL_AFSEL3_Pos) ## !< 0x00002000
  GPIO_AFRL_AFSEL3_2* = (0x00000004 shl GPIO_AFRL_AFSEL3_Pos) ## !< 0x00004000
  GPIO_AFRL_AFSEL3_3* = (0x00000008 shl GPIO_AFRL_AFSEL3_Pos) ## !< 0x00008000
  GPIO_AFRL_AFSEL4_Pos* = (16)
  GPIO_AFRL_AFSEL4_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL4_Pos) ## !< 0x000F0000
  GPIO_AFRL_AFSEL4* = GPIO_AFRL_AFSEL4_Msk
  GPIO_AFRL_AFSEL4_0* = (0x00000001 shl GPIO_AFRL_AFSEL4_Pos) ## !< 0x00010000
  GPIO_AFRL_AFSEL4_1* = (0x00000002 shl GPIO_AFRL_AFSEL4_Pos) ## !< 0x00020000
  GPIO_AFRL_AFSEL4_2* = (0x00000004 shl GPIO_AFRL_AFSEL4_Pos) ## !< 0x00040000
  GPIO_AFRL_AFSEL4_3* = (0x00000008 shl GPIO_AFRL_AFSEL4_Pos) ## !< 0x00080000
  GPIO_AFRL_AFSEL5_Pos* = (20)
  GPIO_AFRL_AFSEL5_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL5_Pos) ## !< 0x00F00000
  GPIO_AFRL_AFSEL5* = GPIO_AFRL_AFSEL5_Msk
  GPIO_AFRL_AFSEL5_0* = (0x00000001 shl GPIO_AFRL_AFSEL5_Pos) ## !< 0x00100000
  GPIO_AFRL_AFSEL5_1* = (0x00000002 shl GPIO_AFRL_AFSEL5_Pos) ## !< 0x00200000
  GPIO_AFRL_AFSEL5_2* = (0x00000004 shl GPIO_AFRL_AFSEL5_Pos) ## !< 0x00400000
  GPIO_AFRL_AFSEL5_3* = (0x00000008 shl GPIO_AFRL_AFSEL5_Pos) ## !< 0x00800000
  GPIO_AFRL_AFSEL6_Pos* = (24)
  GPIO_AFRL_AFSEL6_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL6_Pos) ## !< 0x0F000000
  GPIO_AFRL_AFSEL6* = GPIO_AFRL_AFSEL6_Msk
  GPIO_AFRL_AFSEL6_0* = (0x00000001 shl GPIO_AFRL_AFSEL6_Pos) ## !< 0x01000000
  GPIO_AFRL_AFSEL6_1* = (0x00000002 shl GPIO_AFRL_AFSEL6_Pos) ## !< 0x02000000
  GPIO_AFRL_AFSEL6_2* = (0x00000004 shl GPIO_AFRL_AFSEL6_Pos) ## !< 0x04000000
  GPIO_AFRL_AFSEL6_3* = (0x00000008 shl GPIO_AFRL_AFSEL6_Pos) ## !< 0x08000000
  GPIO_AFRL_AFSEL7_Pos* = (28)
  GPIO_AFRL_AFSEL7_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL7_Pos) ## !< 0xF0000000
  GPIO_AFRL_AFSEL7* = GPIO_AFRL_AFSEL7_Msk
  GPIO_AFRL_AFSEL7_0* = (0x00000001 shl GPIO_AFRL_AFSEL7_Pos) ## !< 0x10000000
  GPIO_AFRL_AFSEL7_1* = (0x00000002 shl GPIO_AFRL_AFSEL7_Pos) ## !< 0x20000000
  GPIO_AFRL_AFSEL7_2* = (0x00000004 shl GPIO_AFRL_AFSEL7_Pos) ## !< 0x40000000
  GPIO_AFRL_AFSEL7_3* = (0x00000008 shl GPIO_AFRL_AFSEL7_Pos) ## !< 0x80000000

##  Legacy defines

const
  GPIO_AFRL_AFRL0* = GPIO_AFRL_AFSEL0
  GPIO_AFRL_AFRL0_0* = GPIO_AFRL_AFSEL0_0
  GPIO_AFRL_AFRL0_1* = GPIO_AFRL_AFSEL0_1
  GPIO_AFRL_AFRL0_2* = GPIO_AFRL_AFSEL0_2
  GPIO_AFRL_AFRL0_3* = GPIO_AFRL_AFSEL0_3
  GPIO_AFRL_AFRL1* = GPIO_AFRL_AFSEL1
  GPIO_AFRL_AFRL1_0* = GPIO_AFRL_AFSEL1_0
  GPIO_AFRL_AFRL1_1* = GPIO_AFRL_AFSEL1_1
  GPIO_AFRL_AFRL1_2* = GPIO_AFRL_AFSEL1_2
  GPIO_AFRL_AFRL1_3* = GPIO_AFRL_AFSEL1_3
  GPIO_AFRL_AFRL2* = GPIO_AFRL_AFSEL2
  GPIO_AFRL_AFRL2_0* = GPIO_AFRL_AFSEL2_0
  GPIO_AFRL_AFRL2_1* = GPIO_AFRL_AFSEL2_1
  GPIO_AFRL_AFRL2_2* = GPIO_AFRL_AFSEL2_2
  GPIO_AFRL_AFRL2_3* = GPIO_AFRL_AFSEL2_3
  GPIO_AFRL_AFRL3* = GPIO_AFRL_AFSEL3
  GPIO_AFRL_AFRL3_0* = GPIO_AFRL_AFSEL3_0
  GPIO_AFRL_AFRL3_1* = GPIO_AFRL_AFSEL3_1
  GPIO_AFRL_AFRL3_2* = GPIO_AFRL_AFSEL3_2
  GPIO_AFRL_AFRL3_3* = GPIO_AFRL_AFSEL3_3
  GPIO_AFRL_AFRL4* = GPIO_AFRL_AFSEL4
  GPIO_AFRL_AFRL4_0* = GPIO_AFRL_AFSEL4_0
  GPIO_AFRL_AFRL4_1* = GPIO_AFRL_AFSEL4_1
  GPIO_AFRL_AFRL4_2* = GPIO_AFRL_AFSEL4_2
  GPIO_AFRL_AFRL4_3* = GPIO_AFRL_AFSEL4_3
  GPIO_AFRL_AFRL5* = GPIO_AFRL_AFSEL5
  GPIO_AFRL_AFRL5_0* = GPIO_AFRL_AFSEL5_0
  GPIO_AFRL_AFRL5_1* = GPIO_AFRL_AFSEL5_1
  GPIO_AFRL_AFRL5_2* = GPIO_AFRL_AFSEL5_2
  GPIO_AFRL_AFRL5_3* = GPIO_AFRL_AFSEL5_3
  GPIO_AFRL_AFRL6* = GPIO_AFRL_AFSEL6
  GPIO_AFRL_AFRL6_0* = GPIO_AFRL_AFSEL6_0
  GPIO_AFRL_AFRL6_1* = GPIO_AFRL_AFSEL6_1
  GPIO_AFRL_AFRL6_2* = GPIO_AFRL_AFSEL6_2
  GPIO_AFRL_AFRL6_3* = GPIO_AFRL_AFSEL6_3
  GPIO_AFRL_AFRL7* = GPIO_AFRL_AFSEL7
  GPIO_AFRL_AFRL7_0* = GPIO_AFRL_AFSEL7_0
  GPIO_AFRL_AFRL7_1* = GPIO_AFRL_AFSEL7_1
  GPIO_AFRL_AFRL7_2* = GPIO_AFRL_AFSEL7_2
  GPIO_AFRL_AFRL7_3* = GPIO_AFRL_AFSEL7_3

## ***************** Bit definition for GPIO_AFRH register ********************

const
  GPIO_AFRH_AFSEL8_Pos* = (0)
  GPIO_AFRH_AFSEL8_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL8_Pos) ## !< 0x0000000F
  GPIO_AFRH_AFSEL8* = GPIO_AFRH_AFSEL8_Msk
  GPIO_AFRH_AFSEL8_0* = (0x00000001 shl GPIO_AFRH_AFSEL8_Pos) ## !< 0x00000001
  GPIO_AFRH_AFSEL8_1* = (0x00000002 shl GPIO_AFRH_AFSEL8_Pos) ## !< 0x00000002
  GPIO_AFRH_AFSEL8_2* = (0x00000004 shl GPIO_AFRH_AFSEL8_Pos) ## !< 0x00000004
  GPIO_AFRH_AFSEL8_3* = (0x00000008 shl GPIO_AFRH_AFSEL8_Pos) ## !< 0x00000008
  GPIO_AFRH_AFSEL9_Pos* = (4)
  GPIO_AFRH_AFSEL9_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL9_Pos) ## !< 0x000000F0
  GPIO_AFRH_AFSEL9* = GPIO_AFRH_AFSEL9_Msk
  GPIO_AFRH_AFSEL9_0* = (0x00000001 shl GPIO_AFRH_AFSEL9_Pos) ## !< 0x00000010
  GPIO_AFRH_AFSEL9_1* = (0x00000002 shl GPIO_AFRH_AFSEL9_Pos) ## !< 0x00000020
  GPIO_AFRH_AFSEL9_2* = (0x00000004 shl GPIO_AFRH_AFSEL9_Pos) ## !< 0x00000040
  GPIO_AFRH_AFSEL9_3* = (0x00000008 shl GPIO_AFRH_AFSEL9_Pos) ## !< 0x00000080
  GPIO_AFRH_AFSEL10_Pos* = (8)
  GPIO_AFRH_AFSEL10_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL10_Pos) ## !< 0x00000F00
  GPIO_AFRH_AFSEL10* = GPIO_AFRH_AFSEL10_Msk
  GPIO_AFRH_AFSEL10_0* = (0x00000001 shl GPIO_AFRH_AFSEL10_Pos) ## !< 0x00000100
  GPIO_AFRH_AFSEL10_1* = (0x00000002 shl GPIO_AFRH_AFSEL10_Pos) ## !< 0x00000200
  GPIO_AFRH_AFSEL10_2* = (0x00000004 shl GPIO_AFRH_AFSEL10_Pos) ## !< 0x00000400
  GPIO_AFRH_AFSEL10_3* = (0x00000008 shl GPIO_AFRH_AFSEL10_Pos) ## !< 0x00000800
  GPIO_AFRH_AFSEL11_Pos* = (12)
  GPIO_AFRH_AFSEL11_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL11_Pos) ## !< 0x0000F000
  GPIO_AFRH_AFSEL11* = GPIO_AFRH_AFSEL11_Msk
  GPIO_AFRH_AFSEL11_0* = (0x00000001 shl GPIO_AFRH_AFSEL11_Pos) ## !< 0x00001000
  GPIO_AFRH_AFSEL11_1* = (0x00000002 shl GPIO_AFRH_AFSEL11_Pos) ## !< 0x00002000
  GPIO_AFRH_AFSEL11_2* = (0x00000004 shl GPIO_AFRH_AFSEL11_Pos) ## !< 0x00004000
  GPIO_AFRH_AFSEL11_3* = (0x00000008 shl GPIO_AFRH_AFSEL11_Pos) ## !< 0x00008000
  GPIO_AFRH_AFSEL12_Pos* = (16)
  GPIO_AFRH_AFSEL12_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL12_Pos) ## !< 0x000F0000
  GPIO_AFRH_AFSEL12* = GPIO_AFRH_AFSEL12_Msk
  GPIO_AFRH_AFSEL12_0* = (0x00000001 shl GPIO_AFRH_AFSEL12_Pos) ## !< 0x00010000
  GPIO_AFRH_AFSEL12_1* = (0x00000002 shl GPIO_AFRH_AFSEL12_Pos) ## !< 0x00020000
  GPIO_AFRH_AFSEL12_2* = (0x00000004 shl GPIO_AFRH_AFSEL12_Pos) ## !< 0x00040000
  GPIO_AFRH_AFSEL12_3* = (0x00000008 shl GPIO_AFRH_AFSEL12_Pos) ## !< 0x00080000
  GPIO_AFRH_AFSEL13_Pos* = (20)
  GPIO_AFRH_AFSEL13_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL13_Pos) ## !< 0x00F00000
  GPIO_AFRH_AFSEL13* = GPIO_AFRH_AFSEL13_Msk
  GPIO_AFRH_AFSEL13_0* = (0x00000001 shl GPIO_AFRH_AFSEL13_Pos) ## !< 0x00100000
  GPIO_AFRH_AFSEL13_1* = (0x00000002 shl GPIO_AFRH_AFSEL13_Pos) ## !< 0x00200000
  GPIO_AFRH_AFSEL13_2* = (0x00000004 shl GPIO_AFRH_AFSEL13_Pos) ## !< 0x00400000
  GPIO_AFRH_AFSEL13_3* = (0x00000008 shl GPIO_AFRH_AFSEL13_Pos) ## !< 0x00800000
  GPIO_AFRH_AFSEL14_Pos* = (24)
  GPIO_AFRH_AFSEL14_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL14_Pos) ## !< 0x0F000000
  GPIO_AFRH_AFSEL14* = GPIO_AFRH_AFSEL14_Msk
  GPIO_AFRH_AFSEL14_0* = (0x00000001 shl GPIO_AFRH_AFSEL14_Pos) ## !< 0x01000000
  GPIO_AFRH_AFSEL14_1* = (0x00000002 shl GPIO_AFRH_AFSEL14_Pos) ## !< 0x02000000
  GPIO_AFRH_AFSEL14_2* = (0x00000004 shl GPIO_AFRH_AFSEL14_Pos) ## !< 0x04000000
  GPIO_AFRH_AFSEL14_3* = (0x00000008 shl GPIO_AFRH_AFSEL14_Pos) ## !< 0x08000000
  GPIO_AFRH_AFSEL15_Pos* = (28)
  GPIO_AFRH_AFSEL15_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL15_Pos) ## !< 0xF0000000
  GPIO_AFRH_AFSEL15* = GPIO_AFRH_AFSEL15_Msk
  GPIO_AFRH_AFSEL15_0* = (0x00000001 shl GPIO_AFRH_AFSEL15_Pos) ## !< 0x10000000
  GPIO_AFRH_AFSEL15_1* = (0x00000002 shl GPIO_AFRH_AFSEL15_Pos) ## !< 0x20000000
  GPIO_AFRH_AFSEL15_2* = (0x00000004 shl GPIO_AFRH_AFSEL15_Pos) ## !< 0x40000000
  GPIO_AFRH_AFSEL15_3* = (0x00000008 shl GPIO_AFRH_AFSEL15_Pos) ## !< 0x80000000

##  Legacy defines

const
  GPIO_AFRH_AFRH0* = GPIO_AFRH_AFSEL8
  GPIO_AFRH_AFRH0_0* = GPIO_AFRH_AFSEL8_0
  GPIO_AFRH_AFRH0_1* = GPIO_AFRH_AFSEL8_1
  GPIO_AFRH_AFRH0_2* = GPIO_AFRH_AFSEL8_2
  GPIO_AFRH_AFRH0_3* = GPIO_AFRH_AFSEL8_3
  GPIO_AFRH_AFRH1* = GPIO_AFRH_AFSEL9
  GPIO_AFRH_AFRH1_0* = GPIO_AFRH_AFSEL9_0
  GPIO_AFRH_AFRH1_1* = GPIO_AFRH_AFSEL9_1
  GPIO_AFRH_AFRH1_2* = GPIO_AFRH_AFSEL9_2
  GPIO_AFRH_AFRH1_3* = GPIO_AFRH_AFSEL9_3
  GPIO_AFRH_AFRH2* = GPIO_AFRH_AFSEL10
  GPIO_AFRH_AFRH2_0* = GPIO_AFRH_AFSEL10_0
  GPIO_AFRH_AFRH2_1* = GPIO_AFRH_AFSEL10_1
  GPIO_AFRH_AFRH2_2* = GPIO_AFRH_AFSEL10_2
  GPIO_AFRH_AFRH2_3* = GPIO_AFRH_AFSEL10_3
  GPIO_AFRH_AFRH3* = GPIO_AFRH_AFSEL11
  GPIO_AFRH_AFRH3_0* = GPIO_AFRH_AFSEL11_0
  GPIO_AFRH_AFRH3_1* = GPIO_AFRH_AFSEL11_1
  GPIO_AFRH_AFRH3_2* = GPIO_AFRH_AFSEL11_2
  GPIO_AFRH_AFRH3_3* = GPIO_AFRH_AFSEL11_3
  GPIO_AFRH_AFRH4* = GPIO_AFRH_AFSEL12
  GPIO_AFRH_AFRH4_0* = GPIO_AFRH_AFSEL12_0
  GPIO_AFRH_AFRH4_1* = GPIO_AFRH_AFSEL12_1
  GPIO_AFRH_AFRH4_2* = GPIO_AFRH_AFSEL12_2
  GPIO_AFRH_AFRH4_3* = GPIO_AFRH_AFSEL12_3
  GPIO_AFRH_AFRH5* = GPIO_AFRH_AFSEL13
  GPIO_AFRH_AFRH5_0* = GPIO_AFRH_AFSEL13_0
  GPIO_AFRH_AFRH5_1* = GPIO_AFRH_AFSEL13_1
  GPIO_AFRH_AFRH5_2* = GPIO_AFRH_AFSEL13_2
  GPIO_AFRH_AFRH5_3* = GPIO_AFRH_AFSEL13_3
  GPIO_AFRH_AFRH6* = GPIO_AFRH_AFSEL14
  GPIO_AFRH_AFRH6_0* = GPIO_AFRH_AFSEL14_0
  GPIO_AFRH_AFRH6_1* = GPIO_AFRH_AFSEL14_1
  GPIO_AFRH_AFRH6_2* = GPIO_AFRH_AFSEL14_2
  GPIO_AFRH_AFRH6_3* = GPIO_AFRH_AFSEL14_3
  GPIO_AFRH_AFRH7* = GPIO_AFRH_AFSEL15
  GPIO_AFRH_AFRH7_0* = GPIO_AFRH_AFSEL15_0
  GPIO_AFRH_AFRH7_1* = GPIO_AFRH_AFSEL15_1
  GPIO_AFRH_AFRH7_2* = GPIO_AFRH_AFSEL15_2
  GPIO_AFRH_AFRH7_3* = GPIO_AFRH_AFSEL15_3

## ****************************************************************************
##
##                       Inter-integrated Circuit Interface
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
  I2C_OAR1_ADD1_7* = 0x000000FE
  I2C_OAR1_ADD8_9* = 0x00000300
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
##                            Independent WATCHDOG
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
  IWDG_PR_PR_0* = (0x00000001 shl IWDG_PR_PR_Pos) ## !< 0x01
  IWDG_PR_PR_1* = (0x00000002 shl IWDG_PR_PR_Pos) ## !< 0x02
  IWDG_PR_PR_2* = (0x00000004 shl IWDG_PR_PR_Pos) ## !< 0x04

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
##                              Power Control
##
## ****************************************************************************
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
  PWR_CR_PLS_LEV0* = 0x00000000
  PWR_CR_PLS_LEV1* = 0x00000020
  PWR_CR_PLS_LEV2* = 0x00000040
  PWR_CR_PLS_LEV3* = 0x00000060
  PWR_CR_PLS_LEV4* = 0x00000080
  PWR_CR_PLS_LEV5* = 0x000000A0
  PWR_CR_PLS_LEV6* = 0x000000C0
  PWR_CR_PLS_LEV7* = 0x000000E0
  PWR_CR_DBP_Pos* = (8)
  PWR_CR_DBP_Msk* = (0x00000001 shl PWR_CR_DBP_Pos) ## !< 0x00000100
  PWR_CR_DBP* = PWR_CR_DBP_Msk
  PWR_CR_FPDS_Pos* = (9)
  PWR_CR_FPDS_Msk* = (0x00000001 shl PWR_CR_FPDS_Pos) ## !< 0x00000200
  PWR_CR_FPDS* = PWR_CR_FPDS_Msk
  PWR_CR_VOS_Pos* = (14)
  PWR_CR_VOS_Msk* = (0x00000001 shl PWR_CR_VOS_Pos) ## !< 0x00004000
  PWR_CR_VOS* = PWR_CR_VOS_Msk

##  Legacy define

const
  PWR_CR_PMODE* = PWR_CR_VOS

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
  PWR_CSR_BRR_Pos* = (3)
  PWR_CSR_BRR_Msk* = (0x00000001 shl PWR_CSR_BRR_Pos) ## !< 0x00000008
  PWR_CSR_BRR* = PWR_CSR_BRR_Msk
  PWR_CSR_EWUP_Pos* = (8)
  PWR_CSR_EWUP_Msk* = (0x00000001 shl PWR_CSR_EWUP_Pos) ## !< 0x00000100
  PWR_CSR_EWUP* = PWR_CSR_EWUP_Msk
  PWR_CSR_BRE_Pos* = (9)
  PWR_CSR_BRE_Msk* = (0x00000001 shl PWR_CSR_BRE_Pos) ## !< 0x00000200
  PWR_CSR_BRE* = PWR_CSR_BRE_Msk
  PWR_CSR_VOSRDY_Pos* = (14)
  PWR_CSR_VOSRDY_Msk* = (0x00000001 shl PWR_CSR_VOSRDY_Pos) ## !< 0x00004000
  PWR_CSR_VOSRDY* = PWR_CSR_VOSRDY_Msk

##  Legacy define

const
  PWR_CSR_REGRDY* = PWR_CSR_VOSRDY

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

##
##  @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
##

const
  RCC_PLLI2S_SUPPORT* = true    ## !< Support PLLI2S oscillator
  RCC_CR_PLLI2SON_Pos* = (26)
  RCC_CR_PLLI2SON_Msk* = (0x00000001 shl RCC_CR_PLLI2SON_Pos) ## !< 0x04000000
  RCC_CR_PLLI2SON* = RCC_CR_PLLI2SON_Msk
  RCC_CR_PLLI2SRDY_Pos* = (27)
  RCC_CR_PLLI2SRDY_Msk* = (0x00000001 shl RCC_CR_PLLI2SRDY_Pos) ## !< 0x08000000
  RCC_CR_PLLI2SRDY* = RCC_CR_PLLI2SRDY_Msk

## *******************  Bit definition for RCC_PLLCFGR register  **************

const
  RCC_PLLCFGR_PLLM_Pos* = (0)
  RCC_PLLCFGR_PLLM_Msk* = (0x0000003F shl RCC_PLLCFGR_PLLM_Pos) ## !< 0x0000003F
  RCC_PLLCFGR_PLLM* = RCC_PLLCFGR_PLLM_Msk
  RCC_PLLCFGR_PLLM_0* = (0x00000001 shl RCC_PLLCFGR_PLLM_Pos) ## !< 0x00000001
  RCC_PLLCFGR_PLLM_1* = (0x00000002 shl RCC_PLLCFGR_PLLM_Pos) ## !< 0x00000002
  RCC_PLLCFGR_PLLM_2* = (0x00000004 shl RCC_PLLCFGR_PLLM_Pos) ## !< 0x00000004
  RCC_PLLCFGR_PLLM_3* = (0x00000008 shl RCC_PLLCFGR_PLLM_Pos) ## !< 0x00000008
  RCC_PLLCFGR_PLLM_4* = (0x00000010 shl RCC_PLLCFGR_PLLM_Pos) ## !< 0x00000010
  RCC_PLLCFGR_PLLM_5* = (0x00000020 shl RCC_PLLCFGR_PLLM_Pos) ## !< 0x00000020
  RCC_PLLCFGR_PLLN_Pos* = (6)
  RCC_PLLCFGR_PLLN_Msk* = (0x000001FF shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00007FC0
  RCC_PLLCFGR_PLLN* = RCC_PLLCFGR_PLLN_Msk
  RCC_PLLCFGR_PLLN_0* = (0x00000001 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00000040
  RCC_PLLCFGR_PLLN_1* = (0x00000002 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00000080
  RCC_PLLCFGR_PLLN_2* = (0x00000004 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00000100
  RCC_PLLCFGR_PLLN_3* = (0x00000008 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00000200
  RCC_PLLCFGR_PLLN_4* = (0x00000010 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00000400
  RCC_PLLCFGR_PLLN_5* = (0x00000020 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00000800
  RCC_PLLCFGR_PLLN_6* = (0x00000040 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00001000
  RCC_PLLCFGR_PLLN_7* = (0x00000080 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00002000
  RCC_PLLCFGR_PLLN_8* = (0x00000100 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00004000
  RCC_PLLCFGR_PLLP_Pos* = (16)
  RCC_PLLCFGR_PLLP_Msk* = (0x00000003 shl RCC_PLLCFGR_PLLP_Pos) ## !< 0x00030000
  RCC_PLLCFGR_PLLP* = RCC_PLLCFGR_PLLP_Msk
  RCC_PLLCFGR_PLLP_0* = (0x00000001 shl RCC_PLLCFGR_PLLP_Pos) ## !< 0x00010000
  RCC_PLLCFGR_PLLP_1* = (0x00000002 shl RCC_PLLCFGR_PLLP_Pos) ## !< 0x00020000
  RCC_PLLCFGR_PLLSRC_Pos* = (22)
  RCC_PLLCFGR_PLLSRC_Msk* = (0x00000001 shl RCC_PLLCFGR_PLLSRC_Pos) ## !< 0x00400000
  RCC_PLLCFGR_PLLSRC* = RCC_PLLCFGR_PLLSRC_Msk
  RCC_PLLCFGR_PLLSRC_HSE_Pos* = (22)
  RCC_PLLCFGR_PLLSRC_HSE_Msk* = (0x00000001 shl RCC_PLLCFGR_PLLSRC_HSE_Pos) ## !< 0x00400000
  RCC_PLLCFGR_PLLSRC_HSE* = RCC_PLLCFGR_PLLSRC_HSE_Msk
  RCC_PLLCFGR_PLLSRC_HSI* = 0x00000000
  RCC_PLLCFGR_PLLQ_Pos* = (24)
  RCC_PLLCFGR_PLLQ_Msk* = (0x0000000F shl RCC_PLLCFGR_PLLQ_Pos) ## !< 0x0F000000
  RCC_PLLCFGR_PLLQ* = RCC_PLLCFGR_PLLQ_Msk
  RCC_PLLCFGR_PLLQ_0* = (0x00000001 shl RCC_PLLCFGR_PLLQ_Pos) ## !< 0x01000000
  RCC_PLLCFGR_PLLQ_1* = (0x00000002 shl RCC_PLLCFGR_PLLQ_Pos) ## !< 0x02000000
  RCC_PLLCFGR_PLLQ_2* = (0x00000004 shl RCC_PLLCFGR_PLLQ_Pos) ## !< 0x04000000
  RCC_PLLCFGR_PLLQ_3* = (0x00000008 shl RCC_PLLCFGR_PLLQ_Pos) ## !< 0x08000000

## *******************  Bit definition for RCC_CFGR register  *****************
## !< SW configuration

const
  RCC_CFGR_SW_Pos* = (0)
  RCC_CFGR_SW_Msk* = (0x00000003 shl RCC_CFGR_SW_Pos) ## !< 0x00000003
  RCC_CFGR_SW* = RCC_CFGR_SW_Msk
  RCC_CFGR_SW_0* = (0x00000001 shl RCC_CFGR_SW_Pos) ## !< 0x00000001
  RCC_CFGR_SW_1* = (0x00000002 shl RCC_CFGR_SW_Pos) ## !< 0x00000002
  RCC_CFGR_SW_HSI* = 0x00000000
  RCC_CFGR_SW_HSE* = 0x00000001
  RCC_CFGR_SW_PLL* = 0x00000002

## !< SWS configuration

const
  RCC_CFGR_SWS_Pos* = (2)
  RCC_CFGR_SWS_Msk* = (0x00000003 shl RCC_CFGR_SWS_Pos) ## !< 0x0000000C
  RCC_CFGR_SWS* = RCC_CFGR_SWS_Msk
  RCC_CFGR_SWS_0* = (0x00000001 shl RCC_CFGR_SWS_Pos) ## !< 0x00000004
  RCC_CFGR_SWS_1* = (0x00000002 shl RCC_CFGR_SWS_Pos) ## !< 0x00000008
  RCC_CFGR_SWS_HSI* = 0x00000000
  RCC_CFGR_SWS_HSE* = 0x00000004
  RCC_CFGR_SWS_PLL* = 0x00000008

## !< HPRE configuration

const
  RCC_CFGR_HPRE_Pos* = (4)
  RCC_CFGR_HPRE_Msk* = (0x0000000F shl RCC_CFGR_HPRE_Pos) ## !< 0x000000F0
  RCC_CFGR_HPRE* = RCC_CFGR_HPRE_Msk
  RCC_CFGR_HPRE_0* = (0x00000001 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000010
  RCC_CFGR_HPRE_1* = (0x00000002 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000020
  RCC_CFGR_HPRE_2* = (0x00000004 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000040
  RCC_CFGR_HPRE_3* = (0x00000008 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000080
  RCC_CFGR_HPRE_DIV1* = 0x00000000
  RCC_CFGR_HPRE_DIV2* = 0x00000080
  RCC_CFGR_HPRE_DIV4* = 0x00000090
  RCC_CFGR_HPRE_DIV8* = 0x000000A0
  RCC_CFGR_HPRE_DIV16* = 0x000000B0
  RCC_CFGR_HPRE_DIV64* = 0x000000C0
  RCC_CFGR_HPRE_DIV128* = 0x000000D0
  RCC_CFGR_HPRE_DIV256* = 0x000000E0
  RCC_CFGR_HPRE_DIV512* = 0x000000F0

## !< PPRE1 configuration

const
  RCC_CFGR_PPRE1_Pos* = (10)
  RCC_CFGR_PPRE1_Msk* = (0x00000007 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00001C00
  RCC_CFGR_PPRE1* = RCC_CFGR_PPRE1_Msk
  RCC_CFGR_PPRE1_0* = (0x00000001 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00000400
  RCC_CFGR_PPRE1_1* = (0x00000002 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00000800
  RCC_CFGR_PPRE1_2* = (0x00000004 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00001000
  RCC_CFGR_PPRE1_DIV1* = 0x00000000
  RCC_CFGR_PPRE1_DIV2* = 0x00001000
  RCC_CFGR_PPRE1_DIV4* = 0x00001400
  RCC_CFGR_PPRE1_DIV8* = 0x00001800
  RCC_CFGR_PPRE1_DIV16* = 0x00001C00

## !< PPRE2 configuration

const
  RCC_CFGR_PPRE2_Pos* = (13)
  RCC_CFGR_PPRE2_Msk* = (0x00000007 shl RCC_CFGR_PPRE2_Pos) ## !< 0x0000E000
  RCC_CFGR_PPRE2* = RCC_CFGR_PPRE2_Msk
  RCC_CFGR_PPRE2_0* = (0x00000001 shl RCC_CFGR_PPRE2_Pos) ## !< 0x00002000
  RCC_CFGR_PPRE2_1* = (0x00000002 shl RCC_CFGR_PPRE2_Pos) ## !< 0x00004000
  RCC_CFGR_PPRE2_2* = (0x00000004 shl RCC_CFGR_PPRE2_Pos) ## !< 0x00008000
  RCC_CFGR_PPRE2_DIV1* = 0x00000000
  RCC_CFGR_PPRE2_DIV2* = 0x00008000
  RCC_CFGR_PPRE2_DIV4* = 0x0000A000
  RCC_CFGR_PPRE2_DIV8* = 0x0000C000
  RCC_CFGR_PPRE2_DIV16* = 0x0000E000

## !< RTCPRE configuration

const
  RCC_CFGR_RTCPRE_Pos* = (16)
  RCC_CFGR_RTCPRE_Msk* = (0x0000001F shl RCC_CFGR_RTCPRE_Pos) ## !< 0x001F0000
  RCC_CFGR_RTCPRE* = RCC_CFGR_RTCPRE_Msk
  RCC_CFGR_RTCPRE_0* = (0x00000001 shl RCC_CFGR_RTCPRE_Pos) ## !< 0x00010000
  RCC_CFGR_RTCPRE_1* = (0x00000002 shl RCC_CFGR_RTCPRE_Pos) ## !< 0x00020000
  RCC_CFGR_RTCPRE_2* = (0x00000004 shl RCC_CFGR_RTCPRE_Pos) ## !< 0x00040000
  RCC_CFGR_RTCPRE_3* = (0x00000008 shl RCC_CFGR_RTCPRE_Pos) ## !< 0x00080000
  RCC_CFGR_RTCPRE_4* = (0x00000010 shl RCC_CFGR_RTCPRE_Pos) ## !< 0x00100000

## !< MCO1 configuration

const
  RCC_CFGR_MCO1_Pos* = (21)
  RCC_CFGR_MCO1_Msk* = (0x00000003 shl RCC_CFGR_MCO1_Pos) ## !< 0x00600000
  RCC_CFGR_MCO1* = RCC_CFGR_MCO1_Msk
  RCC_CFGR_MCO1_0* = (0x00000001 shl RCC_CFGR_MCO1_Pos) ## !< 0x00200000
  RCC_CFGR_MCO1_1* = (0x00000002 shl RCC_CFGR_MCO1_Pos) ## !< 0x00400000
  RCC_CFGR_I2SSRC_Pos* = (23)
  RCC_CFGR_I2SSRC_Msk* = (0x00000001 shl RCC_CFGR_I2SSRC_Pos) ## !< 0x00800000
  RCC_CFGR_I2SSRC* = RCC_CFGR_I2SSRC_Msk
  RCC_CFGR_MCO1PRE_Pos* = (24)
  RCC_CFGR_MCO1PRE_Msk* = (0x00000007 shl RCC_CFGR_MCO1PRE_Pos) ## !< 0x07000000
  RCC_CFGR_MCO1PRE* = RCC_CFGR_MCO1PRE_Msk
  RCC_CFGR_MCO1PRE_0* = (0x00000001 shl RCC_CFGR_MCO1PRE_Pos) ## !< 0x01000000
  RCC_CFGR_MCO1PRE_1* = (0x00000002 shl RCC_CFGR_MCO1PRE_Pos) ## !< 0x02000000
  RCC_CFGR_MCO1PRE_2* = (0x00000004 shl RCC_CFGR_MCO1PRE_Pos) ## !< 0x04000000
  RCC_CFGR_MCO2PRE_Pos* = (27)
  RCC_CFGR_MCO2PRE_Msk* = (0x00000007 shl RCC_CFGR_MCO2PRE_Pos) ## !< 0x38000000
  RCC_CFGR_MCO2PRE* = RCC_CFGR_MCO2PRE_Msk
  RCC_CFGR_MCO2PRE_0* = (0x00000001 shl RCC_CFGR_MCO2PRE_Pos) ## !< 0x08000000
  RCC_CFGR_MCO2PRE_1* = (0x00000002 shl RCC_CFGR_MCO2PRE_Pos) ## !< 0x10000000
  RCC_CFGR_MCO2PRE_2* = (0x00000004 shl RCC_CFGR_MCO2PRE_Pos) ## !< 0x20000000
  RCC_CFGR_MCO2_Pos* = (30)
  RCC_CFGR_MCO2_Msk* = (0x00000003 shl RCC_CFGR_MCO2_Pos) ## !< 0xC0000000
  RCC_CFGR_MCO2* = RCC_CFGR_MCO2_Msk
  RCC_CFGR_MCO2_0* = (0x00000001 shl RCC_CFGR_MCO2_Pos) ## !< 0x40000000
  RCC_CFGR_MCO2_1* = (0x00000002 shl RCC_CFGR_MCO2_Pos) ## !< 0x80000000

## *******************  Bit definition for RCC_CIR register  ******************

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
  RCC_CIR_PLLI2SRDYF_Pos* = (5)
  RCC_CIR_PLLI2SRDYF_Msk* = (0x00000001 shl RCC_CIR_PLLI2SRDYF_Pos) ## !< 0x00000020
  RCC_CIR_PLLI2SRDYF* = RCC_CIR_PLLI2SRDYF_Msk
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
  RCC_CIR_PLLI2SRDYIE_Pos* = (13)
  RCC_CIR_PLLI2SRDYIE_Msk* = (0x00000001 shl RCC_CIR_PLLI2SRDYIE_Pos) ## !< 0x00002000
  RCC_CIR_PLLI2SRDYIE* = RCC_CIR_PLLI2SRDYIE_Msk
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
  RCC_CIR_PLLI2SRDYC_Pos* = (21)
  RCC_CIR_PLLI2SRDYC_Msk* = (0x00000001 shl RCC_CIR_PLLI2SRDYC_Pos) ## !< 0x00200000
  RCC_CIR_PLLI2SRDYC* = RCC_CIR_PLLI2SRDYC_Msk
  RCC_CIR_CSSC_Pos* = (23)
  RCC_CIR_CSSC_Msk* = (0x00000001 shl RCC_CIR_CSSC_Pos) ## !< 0x00800000
  RCC_CIR_CSSC* = RCC_CIR_CSSC_Msk

## *******************  Bit definition for RCC_AHB1RSTR register  *************

const
  RCC_AHB1RSTR_GPIOARST_Pos* = (0)
  RCC_AHB1RSTR_GPIOARST_Msk* = (0x00000001 shl RCC_AHB1RSTR_GPIOARST_Pos) ## !< 0x00000001
  RCC_AHB1RSTR_GPIOARST* = RCC_AHB1RSTR_GPIOARST_Msk
  RCC_AHB1RSTR_GPIOBRST_Pos* = (1)
  RCC_AHB1RSTR_GPIOBRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_GPIOBRST_Pos) ## !< 0x00000002
  RCC_AHB1RSTR_GPIOBRST* = RCC_AHB1RSTR_GPIOBRST_Msk
  RCC_AHB1RSTR_GPIOCRST_Pos* = (2)
  RCC_AHB1RSTR_GPIOCRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_GPIOCRST_Pos) ## !< 0x00000004
  RCC_AHB1RSTR_GPIOCRST* = RCC_AHB1RSTR_GPIOCRST_Msk
  RCC_AHB1RSTR_GPIODRST_Pos* = (3)
  RCC_AHB1RSTR_GPIODRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_GPIODRST_Pos) ## !< 0x00000008
  RCC_AHB1RSTR_GPIODRST* = RCC_AHB1RSTR_GPIODRST_Msk
  RCC_AHB1RSTR_GPIOERST_Pos* = (4)
  RCC_AHB1RSTR_GPIOERST_Msk* = (0x00000001 shl RCC_AHB1RSTR_GPIOERST_Pos) ## !< 0x00000010
  RCC_AHB1RSTR_GPIOERST* = RCC_AHB1RSTR_GPIOERST_Msk
  RCC_AHB1RSTR_GPIOFRST_Pos* = (5)
  RCC_AHB1RSTR_GPIOFRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_GPIOFRST_Pos) ## !< 0x00000020
  RCC_AHB1RSTR_GPIOFRST* = RCC_AHB1RSTR_GPIOFRST_Msk
  RCC_AHB1RSTR_GPIOGRST_Pos* = (6)
  RCC_AHB1RSTR_GPIOGRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_GPIOGRST_Pos) ## !< 0x00000040
  RCC_AHB1RSTR_GPIOGRST* = RCC_AHB1RSTR_GPIOGRST_Msk
  RCC_AHB1RSTR_GPIOHRST_Pos* = (7)
  RCC_AHB1RSTR_GPIOHRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_GPIOHRST_Pos) ## !< 0x00000080
  RCC_AHB1RSTR_GPIOHRST* = RCC_AHB1RSTR_GPIOHRST_Msk
  RCC_AHB1RSTR_GPIOIRST_Pos* = (8)
  RCC_AHB1RSTR_GPIOIRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_GPIOIRST_Pos) ## !< 0x00000100
  RCC_AHB1RSTR_GPIOIRST* = RCC_AHB1RSTR_GPIOIRST_Msk
  RCC_AHB1RSTR_CRCRST_Pos* = (12)
  RCC_AHB1RSTR_CRCRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_CRCRST_Pos) ## !< 0x00001000
  RCC_AHB1RSTR_CRCRST* = RCC_AHB1RSTR_CRCRST_Msk
  RCC_AHB1RSTR_DMA1RST_Pos* = (21)
  RCC_AHB1RSTR_DMA1RST_Msk* = (0x00000001 shl RCC_AHB1RSTR_DMA1RST_Pos) ## !< 0x00200000
  RCC_AHB1RSTR_DMA1RST* = RCC_AHB1RSTR_DMA1RST_Msk
  RCC_AHB1RSTR_DMA2RST_Pos* = (22)
  RCC_AHB1RSTR_DMA2RST_Msk* = (0x00000001 shl RCC_AHB1RSTR_DMA2RST_Pos) ## !< 0x00400000
  RCC_AHB1RSTR_DMA2RST* = RCC_AHB1RSTR_DMA2RST_Msk
  RCC_AHB1RSTR_ETHMACRST_Pos* = (25)
  RCC_AHB1RSTR_ETHMACRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_ETHMACRST_Pos) ## !< 0x02000000
  RCC_AHB1RSTR_ETHMACRST* = RCC_AHB1RSTR_ETHMACRST_Msk
  RCC_AHB1RSTR_OTGHRST_Pos* = (29)
  RCC_AHB1RSTR_OTGHRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_OTGHRST_Pos) ## !< 0x20000000
  RCC_AHB1RSTR_OTGHRST* = RCC_AHB1RSTR_OTGHRST_Msk

## *******************  Bit definition for RCC_AHB2RSTR register  *************

const
  RCC_AHB2RSTR_DCMIRST_Pos* = (0)
  RCC_AHB2RSTR_DCMIRST_Msk* = (0x00000001 shl RCC_AHB2RSTR_DCMIRST_Pos) ## !< 0x00000001
  RCC_AHB2RSTR_DCMIRST* = RCC_AHB2RSTR_DCMIRST_Msk
  RCC_AHB2RSTR_RNGRST_Pos* = (6)
  RCC_AHB2RSTR_RNGRST_Msk* = (0x00000001 shl RCC_AHB2RSTR_RNGRST_Pos) ## !< 0x00000040
  RCC_AHB2RSTR_RNGRST* = RCC_AHB2RSTR_RNGRST_Msk
  RCC_AHB2RSTR_OTGFSRST_Pos* = (7)
  RCC_AHB2RSTR_OTGFSRST_Msk* = (0x00000001 shl RCC_AHB2RSTR_OTGFSRST_Pos) ## !< 0x00000080
  RCC_AHB2RSTR_OTGFSRST* = RCC_AHB2RSTR_OTGFSRST_Msk

## *******************  Bit definition for RCC_AHB3RSTR register  *************

const
  RCC_AHB3RSTR_FSMCRST_Pos* = (0)
  RCC_AHB3RSTR_FSMCRST_Msk* = (0x00000001 shl RCC_AHB3RSTR_FSMCRST_Pos) ## !< 0x00000001
  RCC_AHB3RSTR_FSMCRST* = RCC_AHB3RSTR_FSMCRST_Msk

## *******************  Bit definition for RCC_APB1RSTR register  *************

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
  RCC_APB1RSTR_TIM12RST_Pos* = (6)
  RCC_APB1RSTR_TIM12RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM12RST_Pos) ## !< 0x00000040
  RCC_APB1RSTR_TIM12RST* = RCC_APB1RSTR_TIM12RST_Msk
  RCC_APB1RSTR_TIM13RST_Pos* = (7)
  RCC_APB1RSTR_TIM13RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM13RST_Pos) ## !< 0x00000080
  RCC_APB1RSTR_TIM13RST* = RCC_APB1RSTR_TIM13RST_Msk
  RCC_APB1RSTR_TIM14RST_Pos* = (8)
  RCC_APB1RSTR_TIM14RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM14RST_Pos) ## !< 0x00000100
  RCC_APB1RSTR_TIM14RST* = RCC_APB1RSTR_TIM14RST_Msk
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
  RCC_APB1RSTR_I2C3RST_Pos* = (23)
  RCC_APB1RSTR_I2C3RST_Msk* = (0x00000001 shl RCC_APB1RSTR_I2C3RST_Pos) ## !< 0x00800000
  RCC_APB1RSTR_I2C3RST* = RCC_APB1RSTR_I2C3RST_Msk
  RCC_APB1RSTR_CAN1RST_Pos* = (25)
  RCC_APB1RSTR_CAN1RST_Msk* = (0x00000001 shl RCC_APB1RSTR_CAN1RST_Pos) ## !< 0x02000000
  RCC_APB1RSTR_CAN1RST* = RCC_APB1RSTR_CAN1RST_Msk
  RCC_APB1RSTR_CAN2RST_Pos* = (26)
  RCC_APB1RSTR_CAN2RST_Msk* = (0x00000001 shl RCC_APB1RSTR_CAN2RST_Pos) ## !< 0x04000000
  RCC_APB1RSTR_CAN2RST* = RCC_APB1RSTR_CAN2RST_Msk
  RCC_APB1RSTR_PWRRST_Pos* = (28)
  RCC_APB1RSTR_PWRRST_Msk* = (0x00000001 shl RCC_APB1RSTR_PWRRST_Pos) ## !< 0x10000000
  RCC_APB1RSTR_PWRRST* = RCC_APB1RSTR_PWRRST_Msk
  RCC_APB1RSTR_DACRST_Pos* = (29)
  RCC_APB1RSTR_DACRST_Msk* = (0x00000001 shl RCC_APB1RSTR_DACRST_Pos) ## !< 0x20000000
  RCC_APB1RSTR_DACRST* = RCC_APB1RSTR_DACRST_Msk

## *******************  Bit definition for RCC_APB2RSTR register  *************

const
  RCC_APB2RSTR_TIM1RST_Pos* = (0)
  RCC_APB2RSTR_TIM1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM1RST_Pos) ## !< 0x00000001
  RCC_APB2RSTR_TIM1RST* = RCC_APB2RSTR_TIM1RST_Msk
  RCC_APB2RSTR_TIM8RST_Pos* = (1)
  RCC_APB2RSTR_TIM8RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM8RST_Pos) ## !< 0x00000002
  RCC_APB2RSTR_TIM8RST* = RCC_APB2RSTR_TIM8RST_Msk
  RCC_APB2RSTR_USART1RST_Pos* = (4)
  RCC_APB2RSTR_USART1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_USART1RST_Pos) ## !< 0x00000010
  RCC_APB2RSTR_USART1RST* = RCC_APB2RSTR_USART1RST_Msk
  RCC_APB2RSTR_USART6RST_Pos* = (5)
  RCC_APB2RSTR_USART6RST_Msk* = (0x00000001 shl RCC_APB2RSTR_USART6RST_Pos) ## !< 0x00000020
  RCC_APB2RSTR_USART6RST* = RCC_APB2RSTR_USART6RST_Msk
  RCC_APB2RSTR_ADCRST_Pos* = (8)
  RCC_APB2RSTR_ADCRST_Msk* = (0x00000001 shl RCC_APB2RSTR_ADCRST_Pos) ## !< 0x00000100
  RCC_APB2RSTR_ADCRST* = RCC_APB2RSTR_ADCRST_Msk
  RCC_APB2RSTR_SDIORST_Pos* = (11)
  RCC_APB2RSTR_SDIORST_Msk* = (0x00000001 shl RCC_APB2RSTR_SDIORST_Pos) ## !< 0x00000800
  RCC_APB2RSTR_SDIORST* = RCC_APB2RSTR_SDIORST_Msk
  RCC_APB2RSTR_SPI1RST_Pos* = (12)
  RCC_APB2RSTR_SPI1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_SPI1RST_Pos) ## !< 0x00001000
  RCC_APB2RSTR_SPI1RST* = RCC_APB2RSTR_SPI1RST_Msk
  RCC_APB2RSTR_SYSCFGRST_Pos* = (14)
  RCC_APB2RSTR_SYSCFGRST_Msk* = (0x00000001 shl RCC_APB2RSTR_SYSCFGRST_Pos) ## !< 0x00004000
  RCC_APB2RSTR_SYSCFGRST* = RCC_APB2RSTR_SYSCFGRST_Msk
  RCC_APB2RSTR_TIM9RST_Pos* = (16)
  RCC_APB2RSTR_TIM9RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM9RST_Pos) ## !< 0x00010000
  RCC_APB2RSTR_TIM9RST* = RCC_APB2RSTR_TIM9RST_Msk
  RCC_APB2RSTR_TIM10RST_Pos* = (17)
  RCC_APB2RSTR_TIM10RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM10RST_Pos) ## !< 0x00020000
  RCC_APB2RSTR_TIM10RST* = RCC_APB2RSTR_TIM10RST_Msk
  RCC_APB2RSTR_TIM11RST_Pos* = (18)
  RCC_APB2RSTR_TIM11RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM11RST_Pos) ## !< 0x00040000
  RCC_APB2RSTR_TIM11RST* = RCC_APB2RSTR_TIM11RST_Msk

##  Old SPI1RST bit definition, maintained for legacy purpose

const
  RCC_APB2RSTR_SPI1* = RCC_APB2RSTR_SPI1RST

## *******************  Bit definition for RCC_AHB1ENR register  **************

const
  RCC_AHB1ENR_GPIOAEN_Pos* = (0)
  RCC_AHB1ENR_GPIOAEN_Msk* = (0x00000001 shl RCC_AHB1ENR_GPIOAEN_Pos) ## !< 0x00000001
  RCC_AHB1ENR_GPIOAEN* = RCC_AHB1ENR_GPIOAEN_Msk
  RCC_AHB1ENR_GPIOBEN_Pos* = (1)
  RCC_AHB1ENR_GPIOBEN_Msk* = (0x00000001 shl RCC_AHB1ENR_GPIOBEN_Pos) ## !< 0x00000002
  RCC_AHB1ENR_GPIOBEN* = RCC_AHB1ENR_GPIOBEN_Msk
  RCC_AHB1ENR_GPIOCEN_Pos* = (2)
  RCC_AHB1ENR_GPIOCEN_Msk* = (0x00000001 shl RCC_AHB1ENR_GPIOCEN_Pos) ## !< 0x00000004
  RCC_AHB1ENR_GPIOCEN* = RCC_AHB1ENR_GPIOCEN_Msk
  RCC_AHB1ENR_GPIODEN_Pos* = (3)
  RCC_AHB1ENR_GPIODEN_Msk* = (0x00000001 shl RCC_AHB1ENR_GPIODEN_Pos) ## !< 0x00000008
  RCC_AHB1ENR_GPIODEN* = RCC_AHB1ENR_GPIODEN_Msk
  RCC_AHB1ENR_GPIOEEN_Pos* = (4)
  RCC_AHB1ENR_GPIOEEN_Msk* = (0x00000001 shl RCC_AHB1ENR_GPIOEEN_Pos) ## !< 0x00000010
  RCC_AHB1ENR_GPIOEEN* = RCC_AHB1ENR_GPIOEEN_Msk
  RCC_AHB1ENR_GPIOFEN_Pos* = (5)
  RCC_AHB1ENR_GPIOFEN_Msk* = (0x00000001 shl RCC_AHB1ENR_GPIOFEN_Pos) ## !< 0x00000020
  RCC_AHB1ENR_GPIOFEN* = RCC_AHB1ENR_GPIOFEN_Msk
  RCC_AHB1ENR_GPIOGEN_Pos* = (6)
  RCC_AHB1ENR_GPIOGEN_Msk* = (0x00000001 shl RCC_AHB1ENR_GPIOGEN_Pos) ## !< 0x00000040
  RCC_AHB1ENR_GPIOGEN* = RCC_AHB1ENR_GPIOGEN_Msk
  RCC_AHB1ENR_GPIOHEN_Pos* = (7)
  RCC_AHB1ENR_GPIOHEN_Msk* = (0x00000001 shl RCC_AHB1ENR_GPIOHEN_Pos) ## !< 0x00000080
  RCC_AHB1ENR_GPIOHEN* = RCC_AHB1ENR_GPIOHEN_Msk
  RCC_AHB1ENR_GPIOIEN_Pos* = (8)
  RCC_AHB1ENR_GPIOIEN_Msk* = (0x00000001 shl RCC_AHB1ENR_GPIOIEN_Pos) ## !< 0x00000100
  RCC_AHB1ENR_GPIOIEN* = RCC_AHB1ENR_GPIOIEN_Msk
  RCC_AHB1ENR_CRCEN_Pos* = (12)
  RCC_AHB1ENR_CRCEN_Msk* = (0x00000001 shl RCC_AHB1ENR_CRCEN_Pos) ## !< 0x00001000
  RCC_AHB1ENR_CRCEN* = RCC_AHB1ENR_CRCEN_Msk
  RCC_AHB1ENR_BKPSRAMEN_Pos* = (18)
  RCC_AHB1ENR_BKPSRAMEN_Msk* = (0x00000001 shl RCC_AHB1ENR_BKPSRAMEN_Pos) ## !< 0x00040000
  RCC_AHB1ENR_BKPSRAMEN* = RCC_AHB1ENR_BKPSRAMEN_Msk
  RCC_AHB1ENR_CCMDATARAMEN_Pos* = (20)
  RCC_AHB1ENR_CCMDATARAMEN_Msk* = (0x00000001 shl RCC_AHB1ENR_CCMDATARAMEN_Pos) ## !< 0x00100000
  RCC_AHB1ENR_CCMDATARAMEN* = RCC_AHB1ENR_CCMDATARAMEN_Msk
  RCC_AHB1ENR_DMA1EN_Pos* = (21)
  RCC_AHB1ENR_DMA1EN_Msk* = (0x00000001 shl RCC_AHB1ENR_DMA1EN_Pos) ## !< 0x00200000
  RCC_AHB1ENR_DMA1EN* = RCC_AHB1ENR_DMA1EN_Msk
  RCC_AHB1ENR_DMA2EN_Pos* = (22)
  RCC_AHB1ENR_DMA2EN_Msk* = (0x00000001 shl RCC_AHB1ENR_DMA2EN_Pos) ## !< 0x00400000
  RCC_AHB1ENR_DMA2EN* = RCC_AHB1ENR_DMA2EN_Msk
  RCC_AHB1ENR_ETHMACEN_Pos* = (25)
  RCC_AHB1ENR_ETHMACEN_Msk* = (0x00000001 shl RCC_AHB1ENR_ETHMACEN_Pos) ## !< 0x02000000
  RCC_AHB1ENR_ETHMACEN* = RCC_AHB1ENR_ETHMACEN_Msk
  RCC_AHB1ENR_ETHMACTXEN_Pos* = (26)
  RCC_AHB1ENR_ETHMACTXEN_Msk* = (0x00000001 shl RCC_AHB1ENR_ETHMACTXEN_Pos) ## !< 0x04000000
  RCC_AHB1ENR_ETHMACTXEN* = RCC_AHB1ENR_ETHMACTXEN_Msk
  RCC_AHB1ENR_ETHMACRXEN_Pos* = (27)
  RCC_AHB1ENR_ETHMACRXEN_Msk* = (0x00000001 shl RCC_AHB1ENR_ETHMACRXEN_Pos) ## !< 0x08000000
  RCC_AHB1ENR_ETHMACRXEN* = RCC_AHB1ENR_ETHMACRXEN_Msk
  RCC_AHB1ENR_ETHMACPTPEN_Pos* = (28)
  RCC_AHB1ENR_ETHMACPTPEN_Msk* = (0x00000001 shl RCC_AHB1ENR_ETHMACPTPEN_Pos) ## !< 0x10000000
  RCC_AHB1ENR_ETHMACPTPEN* = RCC_AHB1ENR_ETHMACPTPEN_Msk
  RCC_AHB1ENR_OTGHSEN_Pos* = (29)
  RCC_AHB1ENR_OTGHSEN_Msk* = (0x00000001 shl RCC_AHB1ENR_OTGHSEN_Pos) ## !< 0x20000000
  RCC_AHB1ENR_OTGHSEN* = RCC_AHB1ENR_OTGHSEN_Msk
  RCC_AHB1ENR_OTGHSULPIEN_Pos* = (30)
  RCC_AHB1ENR_OTGHSULPIEN_Msk* = (0x00000001 shl RCC_AHB1ENR_OTGHSULPIEN_Pos) ## !< 0x40000000
  RCC_AHB1ENR_OTGHSULPIEN* = RCC_AHB1ENR_OTGHSULPIEN_Msk

## *******************  Bit definition for RCC_AHB2ENR register  **************
##
##  @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
##

const
  RCC_AHB2_SUPPORT* = true      ## !< AHB2 Bus is supported
  RCC_AHB2ENR_DCMIEN_Pos* = (0)
  RCC_AHB2ENR_DCMIEN_Msk* = (0x00000001 shl RCC_AHB2ENR_DCMIEN_Pos) ## !< 0x00000001
  RCC_AHB2ENR_DCMIEN* = RCC_AHB2ENR_DCMIEN_Msk
  RCC_AHB2ENR_RNGEN_Pos* = (6)
  RCC_AHB2ENR_RNGEN_Msk* = (0x00000001 shl RCC_AHB2ENR_RNGEN_Pos) ## !< 0x00000040
  RCC_AHB2ENR_RNGEN* = RCC_AHB2ENR_RNGEN_Msk
  RCC_AHB2ENR_OTGFSEN_Pos* = (7)
  RCC_AHB2ENR_OTGFSEN_Msk* = (0x00000001 shl RCC_AHB2ENR_OTGFSEN_Pos) ## !< 0x00000080
  RCC_AHB2ENR_OTGFSEN* = RCC_AHB2ENR_OTGFSEN_Msk

## *******************  Bit definition for RCC_AHB3ENR register  **************
##
##  @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
##

const
  RCC_AHB3_SUPPORT* = true      ## !< AHB3 Bus is supported
  RCC_AHB3ENR_FSMCEN_Pos* = (0)
  RCC_AHB3ENR_FSMCEN_Msk* = (0x00000001 shl RCC_AHB3ENR_FSMCEN_Pos) ## !< 0x00000001
  RCC_AHB3ENR_FSMCEN* = RCC_AHB3ENR_FSMCEN_Msk

## *******************  Bit definition for RCC_APB1ENR register  **************

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
  RCC_APB1ENR_TIM12EN_Pos* = (6)
  RCC_APB1ENR_TIM12EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM12EN_Pos) ## !< 0x00000040
  RCC_APB1ENR_TIM12EN* = RCC_APB1ENR_TIM12EN_Msk
  RCC_APB1ENR_TIM13EN_Pos* = (7)
  RCC_APB1ENR_TIM13EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM13EN_Pos) ## !< 0x00000080
  RCC_APB1ENR_TIM13EN* = RCC_APB1ENR_TIM13EN_Msk
  RCC_APB1ENR_TIM14EN_Pos* = (8)
  RCC_APB1ENR_TIM14EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM14EN_Pos) ## !< 0x00000100
  RCC_APB1ENR_TIM14EN* = RCC_APB1ENR_TIM14EN_Msk
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
  RCC_APB1ENR_I2C3EN_Pos* = (23)
  RCC_APB1ENR_I2C3EN_Msk* = (0x00000001 shl RCC_APB1ENR_I2C3EN_Pos) ## !< 0x00800000
  RCC_APB1ENR_I2C3EN* = RCC_APB1ENR_I2C3EN_Msk
  RCC_APB1ENR_CAN1EN_Pos* = (25)
  RCC_APB1ENR_CAN1EN_Msk* = (0x00000001 shl RCC_APB1ENR_CAN1EN_Pos) ## !< 0x02000000
  RCC_APB1ENR_CAN1EN* = RCC_APB1ENR_CAN1EN_Msk
  RCC_APB1ENR_CAN2EN_Pos* = (26)
  RCC_APB1ENR_CAN2EN_Msk* = (0x00000001 shl RCC_APB1ENR_CAN2EN_Pos) ## !< 0x04000000
  RCC_APB1ENR_CAN2EN* = RCC_APB1ENR_CAN2EN_Msk
  RCC_APB1ENR_PWREN_Pos* = (28)
  RCC_APB1ENR_PWREN_Msk* = (0x00000001 shl RCC_APB1ENR_PWREN_Pos) ## !< 0x10000000
  RCC_APB1ENR_PWREN* = RCC_APB1ENR_PWREN_Msk
  RCC_APB1ENR_DACEN_Pos* = (29)
  RCC_APB1ENR_DACEN_Msk* = (0x00000001 shl RCC_APB1ENR_DACEN_Pos) ## !< 0x20000000
  RCC_APB1ENR_DACEN* = RCC_APB1ENR_DACEN_Msk

## *******************  Bit definition for RCC_APB2ENR register  **************

const
  RCC_APB2ENR_TIM1EN_Pos* = (0)
  RCC_APB2ENR_TIM1EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM1EN_Pos) ## !< 0x00000001
  RCC_APB2ENR_TIM1EN* = RCC_APB2ENR_TIM1EN_Msk
  RCC_APB2ENR_TIM8EN_Pos* = (1)
  RCC_APB2ENR_TIM8EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM8EN_Pos) ## !< 0x00000002
  RCC_APB2ENR_TIM8EN* = RCC_APB2ENR_TIM8EN_Msk
  RCC_APB2ENR_USART1EN_Pos* = (4)
  RCC_APB2ENR_USART1EN_Msk* = (0x00000001 shl RCC_APB2ENR_USART1EN_Pos) ## !< 0x00000010
  RCC_APB2ENR_USART1EN* = RCC_APB2ENR_USART1EN_Msk
  RCC_APB2ENR_USART6EN_Pos* = (5)
  RCC_APB2ENR_USART6EN_Msk* = (0x00000001 shl RCC_APB2ENR_USART6EN_Pos) ## !< 0x00000020
  RCC_APB2ENR_USART6EN* = RCC_APB2ENR_USART6EN_Msk
  RCC_APB2ENR_ADC1EN_Pos* = (8)
  RCC_APB2ENR_ADC1EN_Msk* = (0x00000001 shl RCC_APB2ENR_ADC1EN_Pos) ## !< 0x00000100
  RCC_APB2ENR_ADC1EN* = RCC_APB2ENR_ADC1EN_Msk
  RCC_APB2ENR_ADC2EN_Pos* = (9)
  RCC_APB2ENR_ADC2EN_Msk* = (0x00000001 shl RCC_APB2ENR_ADC2EN_Pos) ## !< 0x00000200
  RCC_APB2ENR_ADC2EN* = RCC_APB2ENR_ADC2EN_Msk
  RCC_APB2ENR_ADC3EN_Pos* = (10)
  RCC_APB2ENR_ADC3EN_Msk* = (0x00000001 shl RCC_APB2ENR_ADC3EN_Pos) ## !< 0x00000400
  RCC_APB2ENR_ADC3EN* = RCC_APB2ENR_ADC3EN_Msk
  RCC_APB2ENR_SDIOEN_Pos* = (11)
  RCC_APB2ENR_SDIOEN_Msk* = (0x00000001 shl RCC_APB2ENR_SDIOEN_Pos) ## !< 0x00000800
  RCC_APB2ENR_SDIOEN* = RCC_APB2ENR_SDIOEN_Msk
  RCC_APB2ENR_SPI1EN_Pos* = (12)
  RCC_APB2ENR_SPI1EN_Msk* = (0x00000001 shl RCC_APB2ENR_SPI1EN_Pos) ## !< 0x00001000
  RCC_APB2ENR_SPI1EN* = RCC_APB2ENR_SPI1EN_Msk
  RCC_APB2ENR_SYSCFGEN_Pos* = (14)
  RCC_APB2ENR_SYSCFGEN_Msk* = (0x00000001 shl RCC_APB2ENR_SYSCFGEN_Pos) ## !< 0x00004000
  RCC_APB2ENR_SYSCFGEN* = RCC_APB2ENR_SYSCFGEN_Msk
  RCC_APB2ENR_TIM9EN_Pos* = (16)
  RCC_APB2ENR_TIM9EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM9EN_Pos) ## !< 0x00010000
  RCC_APB2ENR_TIM9EN* = RCC_APB2ENR_TIM9EN_Msk
  RCC_APB2ENR_TIM10EN_Pos* = (17)
  RCC_APB2ENR_TIM10EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM10EN_Pos) ## !< 0x00020000
  RCC_APB2ENR_TIM10EN* = RCC_APB2ENR_TIM10EN_Msk
  RCC_APB2ENR_TIM11EN_Pos* = (18)
  RCC_APB2ENR_TIM11EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM11EN_Pos) ## !< 0x00040000
  RCC_APB2ENR_TIM11EN* = RCC_APB2ENR_TIM11EN_Msk

## *******************  Bit definition for RCC_AHB1LPENR register  ************

const
  RCC_AHB1LPENR_GPIOALPEN_Pos* = (0)
  RCC_AHB1LPENR_GPIOALPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_GPIOALPEN_Pos) ## !< 0x00000001
  RCC_AHB1LPENR_GPIOALPEN* = RCC_AHB1LPENR_GPIOALPEN_Msk
  RCC_AHB1LPENR_GPIOBLPEN_Pos* = (1)
  RCC_AHB1LPENR_GPIOBLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_GPIOBLPEN_Pos) ## !< 0x00000002
  RCC_AHB1LPENR_GPIOBLPEN* = RCC_AHB1LPENR_GPIOBLPEN_Msk
  RCC_AHB1LPENR_GPIOCLPEN_Pos* = (2)
  RCC_AHB1LPENR_GPIOCLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_GPIOCLPEN_Pos) ## !< 0x00000004
  RCC_AHB1LPENR_GPIOCLPEN* = RCC_AHB1LPENR_GPIOCLPEN_Msk
  RCC_AHB1LPENR_GPIODLPEN_Pos* = (3)
  RCC_AHB1LPENR_GPIODLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_GPIODLPEN_Pos) ## !< 0x00000008
  RCC_AHB1LPENR_GPIODLPEN* = RCC_AHB1LPENR_GPIODLPEN_Msk
  RCC_AHB1LPENR_GPIOELPEN_Pos* = (4)
  RCC_AHB1LPENR_GPIOELPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_GPIOELPEN_Pos) ## !< 0x00000010
  RCC_AHB1LPENR_GPIOELPEN* = RCC_AHB1LPENR_GPIOELPEN_Msk
  RCC_AHB1LPENR_GPIOFLPEN_Pos* = (5)
  RCC_AHB1LPENR_GPIOFLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_GPIOFLPEN_Pos) ## !< 0x00000020
  RCC_AHB1LPENR_GPIOFLPEN* = RCC_AHB1LPENR_GPIOFLPEN_Msk
  RCC_AHB1LPENR_GPIOGLPEN_Pos* = (6)
  RCC_AHB1LPENR_GPIOGLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_GPIOGLPEN_Pos) ## !< 0x00000040
  RCC_AHB1LPENR_GPIOGLPEN* = RCC_AHB1LPENR_GPIOGLPEN_Msk
  RCC_AHB1LPENR_GPIOHLPEN_Pos* = (7)
  RCC_AHB1LPENR_GPIOHLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_GPIOHLPEN_Pos) ## !< 0x00000080
  RCC_AHB1LPENR_GPIOHLPEN* = RCC_AHB1LPENR_GPIOHLPEN_Msk
  RCC_AHB1LPENR_GPIOILPEN_Pos* = (8)
  RCC_AHB1LPENR_GPIOILPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_GPIOILPEN_Pos) ## !< 0x00000100
  RCC_AHB1LPENR_GPIOILPEN* = RCC_AHB1LPENR_GPIOILPEN_Msk
  RCC_AHB1LPENR_CRCLPEN_Pos* = (12)
  RCC_AHB1LPENR_CRCLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_CRCLPEN_Pos) ## !< 0x00001000
  RCC_AHB1LPENR_CRCLPEN* = RCC_AHB1LPENR_CRCLPEN_Msk
  RCC_AHB1LPENR_FLITFLPEN_Pos* = (15)
  RCC_AHB1LPENR_FLITFLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_FLITFLPEN_Pos) ## !< 0x00008000
  RCC_AHB1LPENR_FLITFLPEN* = RCC_AHB1LPENR_FLITFLPEN_Msk
  RCC_AHB1LPENR_SRAM1LPEN_Pos* = (16)
  RCC_AHB1LPENR_SRAM1LPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_SRAM1LPEN_Pos) ## !< 0x00010000
  RCC_AHB1LPENR_SRAM1LPEN* = RCC_AHB1LPENR_SRAM1LPEN_Msk
  RCC_AHB1LPENR_SRAM2LPEN_Pos* = (17)
  RCC_AHB1LPENR_SRAM2LPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_SRAM2LPEN_Pos) ## !< 0x00020000
  RCC_AHB1LPENR_SRAM2LPEN* = RCC_AHB1LPENR_SRAM2LPEN_Msk
  RCC_AHB1LPENR_BKPSRAMLPEN_Pos* = (18)
  RCC_AHB1LPENR_BKPSRAMLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_BKPSRAMLPEN_Pos) ## !< 0x00040000
  RCC_AHB1LPENR_BKPSRAMLPEN* = RCC_AHB1LPENR_BKPSRAMLPEN_Msk
  RCC_AHB1LPENR_DMA1LPEN_Pos* = (21)
  RCC_AHB1LPENR_DMA1LPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_DMA1LPEN_Pos) ## !< 0x00200000
  RCC_AHB1LPENR_DMA1LPEN* = RCC_AHB1LPENR_DMA1LPEN_Msk
  RCC_AHB1LPENR_DMA2LPEN_Pos* = (22)
  RCC_AHB1LPENR_DMA2LPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_DMA2LPEN_Pos) ## !< 0x00400000
  RCC_AHB1LPENR_DMA2LPEN* = RCC_AHB1LPENR_DMA2LPEN_Msk
  RCC_AHB1LPENR_ETHMACLPEN_Pos* = (25)
  RCC_AHB1LPENR_ETHMACLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_ETHMACLPEN_Pos) ## !< 0x02000000
  RCC_AHB1LPENR_ETHMACLPEN* = RCC_AHB1LPENR_ETHMACLPEN_Msk
  RCC_AHB1LPENR_ETHMACTXLPEN_Pos* = (26)
  RCC_AHB1LPENR_ETHMACTXLPEN_Msk* = (0x00000001 shl
      RCC_AHB1LPENR_ETHMACTXLPEN_Pos) ## !< 0x04000000
  RCC_AHB1LPENR_ETHMACTXLPEN* = RCC_AHB1LPENR_ETHMACTXLPEN_Msk
  RCC_AHB1LPENR_ETHMACRXLPEN_Pos* = (27)
  RCC_AHB1LPENR_ETHMACRXLPEN_Msk* = (0x00000001 shl
      RCC_AHB1LPENR_ETHMACRXLPEN_Pos) ## !< 0x08000000
  RCC_AHB1LPENR_ETHMACRXLPEN* = RCC_AHB1LPENR_ETHMACRXLPEN_Msk
  RCC_AHB1LPENR_ETHMACPTPLPEN_Pos* = (28)
  RCC_AHB1LPENR_ETHMACPTPLPEN_Msk* = (
    0x00000001 shl RCC_AHB1LPENR_ETHMACPTPLPEN_Pos) ## !< 0x10000000
  RCC_AHB1LPENR_ETHMACPTPLPEN* = RCC_AHB1LPENR_ETHMACPTPLPEN_Msk
  RCC_AHB1LPENR_OTGHSLPEN_Pos* = (29)
  RCC_AHB1LPENR_OTGHSLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_OTGHSLPEN_Pos) ## !< 0x20000000
  RCC_AHB1LPENR_OTGHSLPEN* = RCC_AHB1LPENR_OTGHSLPEN_Msk
  RCC_AHB1LPENR_OTGHSULPILPEN_Pos* = (30)
  RCC_AHB1LPENR_OTGHSULPILPEN_Msk* = (
    0x00000001 shl RCC_AHB1LPENR_OTGHSULPILPEN_Pos) ## !< 0x40000000
  RCC_AHB1LPENR_OTGHSULPILPEN* = RCC_AHB1LPENR_OTGHSULPILPEN_Msk

## *******************  Bit definition for RCC_AHB2LPENR register  ************

const
  RCC_AHB2LPENR_DCMILPEN_Pos* = (0)
  RCC_AHB2LPENR_DCMILPEN_Msk* = (0x00000001 shl RCC_AHB2LPENR_DCMILPEN_Pos) ## !< 0x00000001
  RCC_AHB2LPENR_DCMILPEN* = RCC_AHB2LPENR_DCMILPEN_Msk
  RCC_AHB2LPENR_RNGLPEN_Pos* = (6)
  RCC_AHB2LPENR_RNGLPEN_Msk* = (0x00000001 shl RCC_AHB2LPENR_RNGLPEN_Pos) ## !< 0x00000040
  RCC_AHB2LPENR_RNGLPEN* = RCC_AHB2LPENR_RNGLPEN_Msk
  RCC_AHB2LPENR_OTGFSLPEN_Pos* = (7)
  RCC_AHB2LPENR_OTGFSLPEN_Msk* = (0x00000001 shl RCC_AHB2LPENR_OTGFSLPEN_Pos) ## !< 0x00000080
  RCC_AHB2LPENR_OTGFSLPEN* = RCC_AHB2LPENR_OTGFSLPEN_Msk

## *******************  Bit definition for RCC_AHB3LPENR register  ************

const
  RCC_AHB3LPENR_FSMCLPEN_Pos* = (0)
  RCC_AHB3LPENR_FSMCLPEN_Msk* = (0x00000001 shl RCC_AHB3LPENR_FSMCLPEN_Pos) ## !< 0x00000001
  RCC_AHB3LPENR_FSMCLPEN* = RCC_AHB3LPENR_FSMCLPEN_Msk

## *******************  Bit definition for RCC_APB1LPENR register  ************

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
  RCC_APB1LPENR_TIM12LPEN_Pos* = (6)
  RCC_APB1LPENR_TIM12LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_TIM12LPEN_Pos) ## !< 0x00000040
  RCC_APB1LPENR_TIM12LPEN* = RCC_APB1LPENR_TIM12LPEN_Msk
  RCC_APB1LPENR_TIM13LPEN_Pos* = (7)
  RCC_APB1LPENR_TIM13LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_TIM13LPEN_Pos) ## !< 0x00000080
  RCC_APB1LPENR_TIM13LPEN* = RCC_APB1LPENR_TIM13LPEN_Msk
  RCC_APB1LPENR_TIM14LPEN_Pos* = (8)
  RCC_APB1LPENR_TIM14LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_TIM14LPEN_Pos) ## !< 0x00000100
  RCC_APB1LPENR_TIM14LPEN* = RCC_APB1LPENR_TIM14LPEN_Msk
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
  RCC_APB1LPENR_I2C3LPEN_Pos* = (23)
  RCC_APB1LPENR_I2C3LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_I2C3LPEN_Pos) ## !< 0x00800000
  RCC_APB1LPENR_I2C3LPEN* = RCC_APB1LPENR_I2C3LPEN_Msk
  RCC_APB1LPENR_CAN1LPEN_Pos* = (25)
  RCC_APB1LPENR_CAN1LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_CAN1LPEN_Pos) ## !< 0x02000000
  RCC_APB1LPENR_CAN1LPEN* = RCC_APB1LPENR_CAN1LPEN_Msk
  RCC_APB1LPENR_CAN2LPEN_Pos* = (26)
  RCC_APB1LPENR_CAN2LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_CAN2LPEN_Pos) ## !< 0x04000000
  RCC_APB1LPENR_CAN2LPEN* = RCC_APB1LPENR_CAN2LPEN_Msk
  RCC_APB1LPENR_PWRLPEN_Pos* = (28)
  RCC_APB1LPENR_PWRLPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_PWRLPEN_Pos) ## !< 0x10000000
  RCC_APB1LPENR_PWRLPEN* = RCC_APB1LPENR_PWRLPEN_Msk
  RCC_APB1LPENR_DACLPEN_Pos* = (29)
  RCC_APB1LPENR_DACLPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_DACLPEN_Pos) ## !< 0x20000000
  RCC_APB1LPENR_DACLPEN* = RCC_APB1LPENR_DACLPEN_Msk

## *******************  Bit definition for RCC_APB2LPENR register  ************

const
  RCC_APB2LPENR_TIM1LPEN_Pos* = (0)
  RCC_APB2LPENR_TIM1LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_TIM1LPEN_Pos) ## !< 0x00000001
  RCC_APB2LPENR_TIM1LPEN* = RCC_APB2LPENR_TIM1LPEN_Msk
  RCC_APB2LPENR_TIM8LPEN_Pos* = (1)
  RCC_APB2LPENR_TIM8LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_TIM8LPEN_Pos) ## !< 0x00000002
  RCC_APB2LPENR_TIM8LPEN* = RCC_APB2LPENR_TIM8LPEN_Msk
  RCC_APB2LPENR_USART1LPEN_Pos* = (4)
  RCC_APB2LPENR_USART1LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_USART1LPEN_Pos) ## !< 0x00000010
  RCC_APB2LPENR_USART1LPEN* = RCC_APB2LPENR_USART1LPEN_Msk
  RCC_APB2LPENR_USART6LPEN_Pos* = (5)
  RCC_APB2LPENR_USART6LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_USART6LPEN_Pos) ## !< 0x00000020
  RCC_APB2LPENR_USART6LPEN* = RCC_APB2LPENR_USART6LPEN_Msk
  RCC_APB2LPENR_ADC1LPEN_Pos* = (8)
  RCC_APB2LPENR_ADC1LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_ADC1LPEN_Pos) ## !< 0x00000100
  RCC_APB2LPENR_ADC1LPEN* = RCC_APB2LPENR_ADC1LPEN_Msk
  RCC_APB2LPENR_ADC2LPEN_Pos* = (9)
  RCC_APB2LPENR_ADC2LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_ADC2LPEN_Pos) ## !< 0x00000200
  RCC_APB2LPENR_ADC2LPEN* = RCC_APB2LPENR_ADC2LPEN_Msk
  RCC_APB2LPENR_ADC3LPEN_Pos* = (10)
  RCC_APB2LPENR_ADC3LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_ADC3LPEN_Pos) ## !< 0x00000400
  RCC_APB2LPENR_ADC3LPEN* = RCC_APB2LPENR_ADC3LPEN_Msk
  RCC_APB2LPENR_SDIOLPEN_Pos* = (11)
  RCC_APB2LPENR_SDIOLPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_SDIOLPEN_Pos) ## !< 0x00000800
  RCC_APB2LPENR_SDIOLPEN* = RCC_APB2LPENR_SDIOLPEN_Msk
  RCC_APB2LPENR_SPI1LPEN_Pos* = (12)
  RCC_APB2LPENR_SPI1LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_SPI1LPEN_Pos) ## !< 0x00001000
  RCC_APB2LPENR_SPI1LPEN* = RCC_APB2LPENR_SPI1LPEN_Msk
  RCC_APB2LPENR_SYSCFGLPEN_Pos* = (14)
  RCC_APB2LPENR_SYSCFGLPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_SYSCFGLPEN_Pos) ## !< 0x00004000
  RCC_APB2LPENR_SYSCFGLPEN* = RCC_APB2LPENR_SYSCFGLPEN_Msk
  RCC_APB2LPENR_TIM9LPEN_Pos* = (16)
  RCC_APB2LPENR_TIM9LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_TIM9LPEN_Pos) ## !< 0x00010000
  RCC_APB2LPENR_TIM9LPEN* = RCC_APB2LPENR_TIM9LPEN_Msk
  RCC_APB2LPENR_TIM10LPEN_Pos* = (17)
  RCC_APB2LPENR_TIM10LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_TIM10LPEN_Pos) ## !< 0x00020000
  RCC_APB2LPENR_TIM10LPEN* = RCC_APB2LPENR_TIM10LPEN_Msk
  RCC_APB2LPENR_TIM11LPEN_Pos* = (18)
  RCC_APB2LPENR_TIM11LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_TIM11LPEN_Pos) ## !< 0x00040000
  RCC_APB2LPENR_TIM11LPEN* = RCC_APB2LPENR_TIM11LPEN_Msk

## *******************  Bit definition for RCC_BDCR register  *****************

const
  RCC_BDCR_LSEON_Pos* = (0)
  RCC_BDCR_LSEON_Msk* = (0x00000001 shl RCC_BDCR_LSEON_Pos) ## !< 0x00000001
  RCC_BDCR_LSEON* = RCC_BDCR_LSEON_Msk
  RCC_BDCR_LSERDY_Pos* = (1)
  RCC_BDCR_LSERDY_Msk* = (0x00000001 shl RCC_BDCR_LSERDY_Pos) ## !< 0x00000002
  RCC_BDCR_LSERDY* = RCC_BDCR_LSERDY_Msk
  RCC_BDCR_LSEBYP_Pos* = (2)
  RCC_BDCR_LSEBYP_Msk* = (0x00000001 shl RCC_BDCR_LSEBYP_Pos) ## !< 0x00000004
  RCC_BDCR_LSEBYP* = RCC_BDCR_LSEBYP_Msk
  RCC_BDCR_RTCSEL_Pos* = (8)
  RCC_BDCR_RTCSEL_Msk* = (0x00000003 shl RCC_BDCR_RTCSEL_Pos) ## !< 0x00000300
  RCC_BDCR_RTCSEL* = RCC_BDCR_RTCSEL_Msk
  RCC_BDCR_RTCSEL_0* = (0x00000001 shl RCC_BDCR_RTCSEL_Pos) ## !< 0x00000100
  RCC_BDCR_RTCSEL_1* = (0x00000002 shl RCC_BDCR_RTCSEL_Pos) ## !< 0x00000200
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
  RCC_CSR_RMVF_Pos* = (24)
  RCC_CSR_RMVF_Msk* = (0x00000001 shl RCC_CSR_RMVF_Pos) ## !< 0x01000000
  RCC_CSR_RMVF* = RCC_CSR_RMVF_Msk
  RCC_CSR_BORRSTF_Pos* = (25)
  RCC_CSR_BORRSTF_Msk* = (0x00000001 shl RCC_CSR_BORRSTF_Pos) ## !< 0x02000000
  RCC_CSR_BORRSTF* = RCC_CSR_BORRSTF_Msk
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
  RCC_CSR_PADRSTF* = RCC_CSR_PINRSTF
  RCC_CSR_WDGRSTF* = RCC_CSR_IWDGRSTF

## *******************  Bit definition for RCC_SSCGR register  ****************

const
  RCC_SSCGR_MODPER_Pos* = (0)
  RCC_SSCGR_MODPER_Msk* = (0x00001FFF shl RCC_SSCGR_MODPER_Pos) ## !< 0x00001FFF
  RCC_SSCGR_MODPER* = RCC_SSCGR_MODPER_Msk
  RCC_SSCGR_INCSTEP_Pos* = (13)
  RCC_SSCGR_INCSTEP_Msk* = (0x00007FFF shl RCC_SSCGR_INCSTEP_Pos) ## !< 0x0FFFE000
  RCC_SSCGR_INCSTEP* = RCC_SSCGR_INCSTEP_Msk
  RCC_SSCGR_SPREADSEL_Pos* = (30)
  RCC_SSCGR_SPREADSEL_Msk* = (0x00000001 shl RCC_SSCGR_SPREADSEL_Pos) ## !< 0x40000000
  RCC_SSCGR_SPREADSEL* = RCC_SSCGR_SPREADSEL_Msk
  RCC_SSCGR_SSCGEN_Pos* = (31)
  RCC_SSCGR_SSCGEN_Msk* = (0x00000001 shl RCC_SSCGR_SSCGEN_Pos) ## !< 0x80000000
  RCC_SSCGR_SSCGEN* = RCC_SSCGR_SSCGEN_Msk

## *******************  Bit definition for RCC_PLLI2SCFGR register  ***********

const
  RCC_PLLI2SCFGR_PLLI2SN_Pos* = (6)
  RCC_PLLI2SCFGR_PLLI2SN_Msk* = (0x000001FF shl RCC_PLLI2SCFGR_PLLI2SN_Pos) ## !< 0x00007FC0
  RCC_PLLI2SCFGR_PLLI2SN* = RCC_PLLI2SCFGR_PLLI2SN_Msk
  RCC_PLLI2SCFGR_PLLI2SN_0* = (0x00000001 shl RCC_PLLI2SCFGR_PLLI2SN_Pos) ## !< 0x00000040
  RCC_PLLI2SCFGR_PLLI2SN_1* = (0x00000002 shl RCC_PLLI2SCFGR_PLLI2SN_Pos) ## !< 0x00000080
  RCC_PLLI2SCFGR_PLLI2SN_2* = (0x00000004 shl RCC_PLLI2SCFGR_PLLI2SN_Pos) ## !< 0x00000100
  RCC_PLLI2SCFGR_PLLI2SN_3* = (0x00000008 shl RCC_PLLI2SCFGR_PLLI2SN_Pos) ## !< 0x00000200
  RCC_PLLI2SCFGR_PLLI2SN_4* = (0x00000010 shl RCC_PLLI2SCFGR_PLLI2SN_Pos) ## !< 0x00000400
  RCC_PLLI2SCFGR_PLLI2SN_5* = (0x00000020 shl RCC_PLLI2SCFGR_PLLI2SN_Pos) ## !< 0x00000800
  RCC_PLLI2SCFGR_PLLI2SN_6* = (0x00000040 shl RCC_PLLI2SCFGR_PLLI2SN_Pos) ## !< 0x00001000
  RCC_PLLI2SCFGR_PLLI2SN_7* = (0x00000080 shl RCC_PLLI2SCFGR_PLLI2SN_Pos) ## !< 0x00002000
  RCC_PLLI2SCFGR_PLLI2SN_8* = (0x00000100 shl RCC_PLLI2SCFGR_PLLI2SN_Pos) ## !< 0x00004000
  RCC_PLLI2SCFGR_PLLI2SR_Pos* = (28)
  RCC_PLLI2SCFGR_PLLI2SR_Msk* = (0x00000007 shl RCC_PLLI2SCFGR_PLLI2SR_Pos) ## !< 0x70000000
  RCC_PLLI2SCFGR_PLLI2SR* = RCC_PLLI2SCFGR_PLLI2SR_Msk
  RCC_PLLI2SCFGR_PLLI2SR_0* = (0x00000001 shl RCC_PLLI2SCFGR_PLLI2SR_Pos) ## !< 0x10000000
  RCC_PLLI2SCFGR_PLLI2SR_1* = (0x00000002 shl RCC_PLLI2SCFGR_PLLI2SR_Pos) ## !< 0x20000000
  RCC_PLLI2SCFGR_PLLI2SR_2* = (0x00000004 shl RCC_PLLI2SCFGR_PLLI2SR_Pos) ## !< 0x40000000

## ****************************************************************************
##
##                                     RNG
##
## ****************************************************************************
## *******************  Bits definition for RNG_CR register  ******************

const
  RNG_CR_RNGEN_Pos* = (2)
  RNG_CR_RNGEN_Msk* = (0x00000001 shl RNG_CR_RNGEN_Pos) ## !< 0x00000004
  RNG_CR_RNGEN* = RNG_CR_RNGEN_Msk
  RNG_CR_IE_Pos* = (3)
  RNG_CR_IE_Msk* = (0x00000001 shl RNG_CR_IE_Pos) ## !< 0x00000008
  RNG_CR_IE* = RNG_CR_IE_Msk

## *******************  Bits definition for RNG_SR register  ******************

const
  RNG_SR_DRDY_Pos* = (0)
  RNG_SR_DRDY_Msk* = (0x00000001 shl RNG_SR_DRDY_Pos) ## !< 0x00000001
  RNG_SR_DRDY* = RNG_SR_DRDY_Msk
  RNG_SR_CECS_Pos* = (1)
  RNG_SR_CECS_Msk* = (0x00000001 shl RNG_SR_CECS_Pos) ## !< 0x00000002
  RNG_SR_CECS* = RNG_SR_CECS_Msk
  RNG_SR_SECS_Pos* = (2)
  RNG_SR_SECS_Msk* = (0x00000001 shl RNG_SR_SECS_Pos) ## !< 0x00000004
  RNG_SR_SECS* = RNG_SR_SECS_Msk
  RNG_SR_CEIS_Pos* = (5)
  RNG_SR_CEIS_Msk* = (0x00000001 shl RNG_SR_CEIS_Pos) ## !< 0x00000020
  RNG_SR_CEIS* = RNG_SR_CEIS_Msk
  RNG_SR_SEIS_Pos* = (6)
  RNG_SR_SEIS_Msk* = (0x00000001 shl RNG_SR_SEIS_Pos) ## !< 0x00000040
  RNG_SR_SEIS* = RNG_SR_SEIS_Msk

## ****************************************************************************
##
##                            Real-Time Clock (RTC)
##
## ****************************************************************************
##
##  @brief Specific device feature definitions  (not present on all devices in the STM32F4 serie)
##

const
  RTC_TAMPER2_SUPPORT* = true   ## !< TAMPER 2 feature support
  RTC_AF2_SUPPORT* = true       ## !< RTC Alternate Function 2 mapping support

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
  RTC_CR_BCK* = RTC_CR_BKP

## *******************  Bits definition for RTC_ISR register  *****************

const
  RTC_ISR_RECALPF_Pos* = (16)
  RTC_ISR_RECALPF_Msk* = (0x00000001 shl RTC_ISR_RECALPF_Pos) ## !< 0x00010000
  RTC_ISR_RECALPF* = RTC_ISR_RECALPF_Msk
  RTC_ISR_TAMP1F_Pos* = (13)
  RTC_ISR_TAMP1F_Msk* = (0x00000001 shl RTC_ISR_TAMP1F_Pos) ## !< 0x00002000
  RTC_ISR_TAMP1F* = RTC_ISR_TAMP1F_Msk
  RTC_ISR_TAMP2F_Pos* = (14)
  RTC_ISR_TAMP2F_Msk* = (0x00000001 shl RTC_ISR_TAMP2F_Pos) ## !< 0x00004000
  RTC_ISR_TAMP2F* = RTC_ISR_TAMP2F_Msk
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
  RTC_TAFCR_TSINSEL_Pos* = (17)
  RTC_TAFCR_TSINSEL_Msk* = (0x00000001 shl RTC_TAFCR_TSINSEL_Pos) ## !< 0x00020000
  RTC_TAFCR_TSINSEL* = RTC_TAFCR_TSINSEL_Msk
  RTC_TAFCR_TAMP1INSEL_Pos* = (16)
  RTC_TAFCR_TAMP1INSEL_Msk* = (0x00000001 shl RTC_TAFCR_TAMP1INSEL_Pos) ## !< 0x00010000
  RTC_TAFCR_TAMP1INSEL* = RTC_TAFCR_TAMP1INSEL_Msk
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

##  Legacy defines

const
  RTC_TAFCR_TAMPINSEL* = RTC_TAFCR_TAMP1INSEL

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

## ******************* Number of backup registers *****************************

const
  RTC_BKP_NUMBER* = 0x0000000000000014'i64

## ****************************************************************************
##
##                           SD host Interface
##
## ****************************************************************************
## *****************  Bit definition for SDIO_POWER register  *****************

const
  SDIO_POWER_PWRCTRL_Pos* = (0)
  SDIO_POWER_PWRCTRL_Msk* = (0x00000003 shl SDIO_POWER_PWRCTRL_Pos) ## !< 0x00000003
  SDIO_POWER_PWRCTRL* = SDIO_POWER_PWRCTRL_Msk
  SDIO_POWER_PWRCTRL_0* = (0x00000001 shl SDIO_POWER_PWRCTRL_Pos) ## !< 0x01
  SDIO_POWER_PWRCTRL_1* = (0x00000002 shl SDIO_POWER_PWRCTRL_Pos) ## !< 0x02

## *****************  Bit definition for SDIO_CLKCR register  *****************

const
  SDIO_CLKCR_CLKDIV_Pos* = (0)
  SDIO_CLKCR_CLKDIV_Msk* = (0x000000FF shl SDIO_CLKCR_CLKDIV_Pos) ## !< 0x000000FF
  SDIO_CLKCR_CLKDIV* = SDIO_CLKCR_CLKDIV_Msk
  SDIO_CLKCR_CLKEN_Pos* = (8)
  SDIO_CLKCR_CLKEN_Msk* = (0x00000001 shl SDIO_CLKCR_CLKEN_Pos) ## !< 0x00000100
  SDIO_CLKCR_CLKEN* = SDIO_CLKCR_CLKEN_Msk
  SDIO_CLKCR_PWRSAV_Pos* = (9)
  SDIO_CLKCR_PWRSAV_Msk* = (0x00000001 shl SDIO_CLKCR_PWRSAV_Pos) ## !< 0x00000200
  SDIO_CLKCR_PWRSAV* = SDIO_CLKCR_PWRSAV_Msk
  SDIO_CLKCR_BYPASS_Pos* = (10)
  SDIO_CLKCR_BYPASS_Msk* = (0x00000001 shl SDIO_CLKCR_BYPASS_Pos) ## !< 0x00000400
  SDIO_CLKCR_BYPASS* = SDIO_CLKCR_BYPASS_Msk
  SDIO_CLKCR_WIDBUS_Pos* = (11)
  SDIO_CLKCR_WIDBUS_Msk* = (0x00000003 shl SDIO_CLKCR_WIDBUS_Pos) ## !< 0x00001800
  SDIO_CLKCR_WIDBUS* = SDIO_CLKCR_WIDBUS_Msk
  SDIO_CLKCR_WIDBUS_0* = (0x00000001 shl SDIO_CLKCR_WIDBUS_Pos) ## !< 0x0800
  SDIO_CLKCR_WIDBUS_1* = (0x00000002 shl SDIO_CLKCR_WIDBUS_Pos) ## !< 0x1000
  SDIO_CLKCR_NEGEDGE_Pos* = (13)
  SDIO_CLKCR_NEGEDGE_Msk* = (0x00000001 shl SDIO_CLKCR_NEGEDGE_Pos) ## !< 0x00002000
  SDIO_CLKCR_NEGEDGE* = SDIO_CLKCR_NEGEDGE_Msk
  SDIO_CLKCR_HWFC_EN_Pos* = (14)
  SDIO_CLKCR_HWFC_EN_Msk* = (0x00000001 shl SDIO_CLKCR_HWFC_EN_Pos) ## !< 0x00004000
  SDIO_CLKCR_HWFC_EN* = SDIO_CLKCR_HWFC_EN_Msk

## ******************  Bit definition for SDIO_ARG register  ******************

const
  SDIO_ARG_CMDARG_Pos* = (0)
  SDIO_ARG_CMDARG_Msk* = (0xFFFFFFFF shl SDIO_ARG_CMDARG_Pos) ## !< 0xFFFFFFFF
  SDIO_ARG_CMDARG* = SDIO_ARG_CMDARG_Msk

## ******************  Bit definition for SDIO_CMD register  ******************

const
  SDIO_CMD_CMDINDEX_Pos* = (0)
  SDIO_CMD_CMDINDEX_Msk* = (0x0000003F shl SDIO_CMD_CMDINDEX_Pos) ## !< 0x0000003F
  SDIO_CMD_CMDINDEX* = SDIO_CMD_CMDINDEX_Msk
  SDIO_CMD_WAITRESP_Pos* = (6)
  SDIO_CMD_WAITRESP_Msk* = (0x00000003 shl SDIO_CMD_WAITRESP_Pos) ## !< 0x000000C0
  SDIO_CMD_WAITRESP* = SDIO_CMD_WAITRESP_Msk
  SDIO_CMD_WAITRESP_0* = (0x00000001 shl SDIO_CMD_WAITRESP_Pos) ## !< 0x0040
  SDIO_CMD_WAITRESP_1* = (0x00000002 shl SDIO_CMD_WAITRESP_Pos) ## !< 0x0080
  SDIO_CMD_WAITINT_Pos* = (8)
  SDIO_CMD_WAITINT_Msk* = (0x00000001 shl SDIO_CMD_WAITINT_Pos) ## !< 0x00000100
  SDIO_CMD_WAITINT* = SDIO_CMD_WAITINT_Msk
  SDIO_CMD_WAITPEND_Pos* = (9)
  SDIO_CMD_WAITPEND_Msk* = (0x00000001 shl SDIO_CMD_WAITPEND_Pos) ## !< 0x00000200
  SDIO_CMD_WAITPEND* = SDIO_CMD_WAITPEND_Msk
  SDIO_CMD_CPSMEN_Pos* = (10)
  SDIO_CMD_CPSMEN_Msk* = (0x00000001 shl SDIO_CMD_CPSMEN_Pos) ## !< 0x00000400
  SDIO_CMD_CPSMEN* = SDIO_CMD_CPSMEN_Msk
  SDIO_CMD_SDIOSUSPEND_Pos* = (11)
  SDIO_CMD_SDIOSUSPEND_Msk* = (0x00000001 shl SDIO_CMD_SDIOSUSPEND_Pos) ## !< 0x00000800
  SDIO_CMD_SDIOSUSPEND* = SDIO_CMD_SDIOSUSPEND_Msk
  SDIO_CMD_ENCMDCOMPL_Pos* = (12)
  SDIO_CMD_ENCMDCOMPL_Msk* = (0x00000001 shl SDIO_CMD_ENCMDCOMPL_Pos) ## !< 0x00001000
  SDIO_CMD_ENCMDCOMPL* = SDIO_CMD_ENCMDCOMPL_Msk
  SDIO_CMD_NIEN_Pos* = (13)
  SDIO_CMD_NIEN_Msk* = (0x00000001 shl SDIO_CMD_NIEN_Pos) ## !< 0x00002000
  SDIO_CMD_NIEN* = SDIO_CMD_NIEN_Msk
  SDIO_CMD_CEATACMD_Pos* = (14)
  SDIO_CMD_CEATACMD_Msk* = (0x00000001 shl SDIO_CMD_CEATACMD_Pos) ## !< 0x00004000
  SDIO_CMD_CEATACMD* = SDIO_CMD_CEATACMD_Msk

## ****************  Bit definition for SDIO_RESPCMD register  ****************

const
  SDIO_RESPCMD_RESPCMD_Pos* = (0)
  SDIO_RESPCMD_RESPCMD_Msk* = (0x0000003F shl SDIO_RESPCMD_RESPCMD_Pos) ## !< 0x0000003F
  SDIO_RESPCMD_RESPCMD* = SDIO_RESPCMD_RESPCMD_Msk

## *****************  Bit definition for SDIO_RESP0 register  *****************

const
  SDIO_RESP0_CARDSTATUS0_Pos* = (0)
  SDIO_RESP0_CARDSTATUS0_Msk* = (0xFFFFFFFF shl SDIO_RESP0_CARDSTATUS0_Pos) ## !< 0xFFFFFFFF
  SDIO_RESP0_CARDSTATUS0* = SDIO_RESP0_CARDSTATUS0_Msk

## *****************  Bit definition for SDIO_RESP1 register  *****************

const
  SDIO_RESP1_CARDSTATUS1_Pos* = (0)
  SDIO_RESP1_CARDSTATUS1_Msk* = (0xFFFFFFFF shl SDIO_RESP1_CARDSTATUS1_Pos) ## !< 0xFFFFFFFF
  SDIO_RESP1_CARDSTATUS1* = SDIO_RESP1_CARDSTATUS1_Msk

## *****************  Bit definition for SDIO_RESP2 register  *****************

const
  SDIO_RESP2_CARDSTATUS2_Pos* = (0)
  SDIO_RESP2_CARDSTATUS2_Msk* = (0xFFFFFFFF shl SDIO_RESP2_CARDSTATUS2_Pos) ## !< 0xFFFFFFFF
  SDIO_RESP2_CARDSTATUS2* = SDIO_RESP2_CARDSTATUS2_Msk

## *****************  Bit definition for SDIO_RESP3 register  *****************

const
  SDIO_RESP3_CARDSTATUS3_Pos* = (0)
  SDIO_RESP3_CARDSTATUS3_Msk* = (0xFFFFFFFF shl SDIO_RESP3_CARDSTATUS3_Pos) ## !< 0xFFFFFFFF
  SDIO_RESP3_CARDSTATUS3* = SDIO_RESP3_CARDSTATUS3_Msk

## *****************  Bit definition for SDIO_RESP4 register  *****************

const
  SDIO_RESP4_CARDSTATUS4_Pos* = (0)
  SDIO_RESP4_CARDSTATUS4_Msk* = (0xFFFFFFFF shl SDIO_RESP4_CARDSTATUS4_Pos) ## !< 0xFFFFFFFF
  SDIO_RESP4_CARDSTATUS4* = SDIO_RESP4_CARDSTATUS4_Msk

## *****************  Bit definition for SDIO_DTIMER register  ****************

const
  SDIO_DTIMER_DATATIME_Pos* = (0)
  SDIO_DTIMER_DATATIME_Msk* = (0xFFFFFFFF shl SDIO_DTIMER_DATATIME_Pos) ## !< 0xFFFFFFFF
  SDIO_DTIMER_DATATIME* = SDIO_DTIMER_DATATIME_Msk

## *****************  Bit definition for SDIO_DLEN register  ******************

const
  SDIO_DLEN_DATALENGTH_Pos* = (0)
  SDIO_DLEN_DATALENGTH_Msk* = (0x01FFFFFF shl SDIO_DLEN_DATALENGTH_Pos) ## !< 0x01FFFFFF
  SDIO_DLEN_DATALENGTH* = SDIO_DLEN_DATALENGTH_Msk

## *****************  Bit definition for SDIO_DCTRL register  *****************

const
  SDIO_DCTRL_DTEN_Pos* = (0)
  SDIO_DCTRL_DTEN_Msk* = (0x00000001 shl SDIO_DCTRL_DTEN_Pos) ## !< 0x00000001
  SDIO_DCTRL_DTEN* = SDIO_DCTRL_DTEN_Msk
  SDIO_DCTRL_DTDIR_Pos* = (1)
  SDIO_DCTRL_DTDIR_Msk* = (0x00000001 shl SDIO_DCTRL_DTDIR_Pos) ## !< 0x00000002
  SDIO_DCTRL_DTDIR* = SDIO_DCTRL_DTDIR_Msk
  SDIO_DCTRL_DTMODE_Pos* = (2)
  SDIO_DCTRL_DTMODE_Msk* = (0x00000001 shl SDIO_DCTRL_DTMODE_Pos) ## !< 0x00000004
  SDIO_DCTRL_DTMODE* = SDIO_DCTRL_DTMODE_Msk
  SDIO_DCTRL_DMAEN_Pos* = (3)
  SDIO_DCTRL_DMAEN_Msk* = (0x00000001 shl SDIO_DCTRL_DMAEN_Pos) ## !< 0x00000008
  SDIO_DCTRL_DMAEN* = SDIO_DCTRL_DMAEN_Msk
  SDIO_DCTRL_DBLOCKSIZE_Pos* = (4)
  SDIO_DCTRL_DBLOCKSIZE_Msk* = (0x0000000F shl SDIO_DCTRL_DBLOCKSIZE_Pos) ## !< 0x000000F0
  SDIO_DCTRL_DBLOCKSIZE* = SDIO_DCTRL_DBLOCKSIZE_Msk
  SDIO_DCTRL_DBLOCKSIZE_0* = (0x00000001 shl SDIO_DCTRL_DBLOCKSIZE_Pos) ## !< 0x0010
  SDIO_DCTRL_DBLOCKSIZE_1* = (0x00000002 shl SDIO_DCTRL_DBLOCKSIZE_Pos) ## !< 0x0020
  SDIO_DCTRL_DBLOCKSIZE_2* = (0x00000004 shl SDIO_DCTRL_DBLOCKSIZE_Pos) ## !< 0x0040
  SDIO_DCTRL_DBLOCKSIZE_3* = (0x00000008 shl SDIO_DCTRL_DBLOCKSIZE_Pos) ## !< 0x0080
  SDIO_DCTRL_RWSTART_Pos* = (8)
  SDIO_DCTRL_RWSTART_Msk* = (0x00000001 shl SDIO_DCTRL_RWSTART_Pos) ## !< 0x00000100
  SDIO_DCTRL_RWSTART* = SDIO_DCTRL_RWSTART_Msk
  SDIO_DCTRL_RWSTOP_Pos* = (9)
  SDIO_DCTRL_RWSTOP_Msk* = (0x00000001 shl SDIO_DCTRL_RWSTOP_Pos) ## !< 0x00000200
  SDIO_DCTRL_RWSTOP* = SDIO_DCTRL_RWSTOP_Msk
  SDIO_DCTRL_RWMOD_Pos* = (10)
  SDIO_DCTRL_RWMOD_Msk* = (0x00000001 shl SDIO_DCTRL_RWMOD_Pos) ## !< 0x00000400
  SDIO_DCTRL_RWMOD* = SDIO_DCTRL_RWMOD_Msk
  SDIO_DCTRL_SDIOEN_Pos* = (11)
  SDIO_DCTRL_SDIOEN_Msk* = (0x00000001 shl SDIO_DCTRL_SDIOEN_Pos) ## !< 0x00000800
  SDIO_DCTRL_SDIOEN* = SDIO_DCTRL_SDIOEN_Msk

## *****************  Bit definition for SDIO_DCOUNT register  ****************

const
  SDIO_DCOUNT_DATACOUNT_Pos* = (0)
  SDIO_DCOUNT_DATACOUNT_Msk* = (0x01FFFFFF shl SDIO_DCOUNT_DATACOUNT_Pos) ## !< 0x01FFFFFF
  SDIO_DCOUNT_DATACOUNT* = SDIO_DCOUNT_DATACOUNT_Msk

## *****************  Bit definition for SDIO_STA register  *******************

const
  SDIO_STA_CCRCFAIL_Pos* = (0)
  SDIO_STA_CCRCFAIL_Msk* = (0x00000001 shl SDIO_STA_CCRCFAIL_Pos) ## !< 0x00000001
  SDIO_STA_CCRCFAIL* = SDIO_STA_CCRCFAIL_Msk
  SDIO_STA_DCRCFAIL_Pos* = (1)
  SDIO_STA_DCRCFAIL_Msk* = (0x00000001 shl SDIO_STA_DCRCFAIL_Pos) ## !< 0x00000002
  SDIO_STA_DCRCFAIL* = SDIO_STA_DCRCFAIL_Msk
  SDIO_STA_CTIMEOUT_Pos* = (2)
  SDIO_STA_CTIMEOUT_Msk* = (0x00000001 shl SDIO_STA_CTIMEOUT_Pos) ## !< 0x00000004
  SDIO_STA_CTIMEOUT* = SDIO_STA_CTIMEOUT_Msk
  SDIO_STA_DTIMEOUT_Pos* = (3)
  SDIO_STA_DTIMEOUT_Msk* = (0x00000001 shl SDIO_STA_DTIMEOUT_Pos) ## !< 0x00000008
  SDIO_STA_DTIMEOUT* = SDIO_STA_DTIMEOUT_Msk
  SDIO_STA_TXUNDERR_Pos* = (4)
  SDIO_STA_TXUNDERR_Msk* = (0x00000001 shl SDIO_STA_TXUNDERR_Pos) ## !< 0x00000010
  SDIO_STA_TXUNDERR* = SDIO_STA_TXUNDERR_Msk
  SDIO_STA_RXOVERR_Pos* = (5)
  SDIO_STA_RXOVERR_Msk* = (0x00000001 shl SDIO_STA_RXOVERR_Pos) ## !< 0x00000020
  SDIO_STA_RXOVERR* = SDIO_STA_RXOVERR_Msk
  SDIO_STA_CMDREND_Pos* = (6)
  SDIO_STA_CMDREND_Msk* = (0x00000001 shl SDIO_STA_CMDREND_Pos) ## !< 0x00000040
  SDIO_STA_CMDREND* = SDIO_STA_CMDREND_Msk
  SDIO_STA_CMDSENT_Pos* = (7)
  SDIO_STA_CMDSENT_Msk* = (0x00000001 shl SDIO_STA_CMDSENT_Pos) ## !< 0x00000080
  SDIO_STA_CMDSENT* = SDIO_STA_CMDSENT_Msk
  SDIO_STA_DATAEND_Pos* = (8)
  SDIO_STA_DATAEND_Msk* = (0x00000001 shl SDIO_STA_DATAEND_Pos) ## !< 0x00000100
  SDIO_STA_DATAEND* = SDIO_STA_DATAEND_Msk
  SDIO_STA_STBITERR_Pos* = (9)
  SDIO_STA_STBITERR_Msk* = (0x00000001 shl SDIO_STA_STBITERR_Pos) ## !< 0x00000200
  SDIO_STA_STBITERR* = SDIO_STA_STBITERR_Msk
  SDIO_STA_DBCKEND_Pos* = (10)
  SDIO_STA_DBCKEND_Msk* = (0x00000001 shl SDIO_STA_DBCKEND_Pos) ## !< 0x00000400
  SDIO_STA_DBCKEND* = SDIO_STA_DBCKEND_Msk
  SDIO_STA_CMDACT_Pos* = (11)
  SDIO_STA_CMDACT_Msk* = (0x00000001 shl SDIO_STA_CMDACT_Pos) ## !< 0x00000800
  SDIO_STA_CMDACT* = SDIO_STA_CMDACT_Msk
  SDIO_STA_TXACT_Pos* = (12)
  SDIO_STA_TXACT_Msk* = (0x00000001 shl SDIO_STA_TXACT_Pos) ## !< 0x00001000
  SDIO_STA_TXACT* = SDIO_STA_TXACT_Msk
  SDIO_STA_RXACT_Pos* = (13)
  SDIO_STA_RXACT_Msk* = (0x00000001 shl SDIO_STA_RXACT_Pos) ## !< 0x00002000
  SDIO_STA_RXACT* = SDIO_STA_RXACT_Msk
  SDIO_STA_TXFIFOHE_Pos* = (14)
  SDIO_STA_TXFIFOHE_Msk* = (0x00000001 shl SDIO_STA_TXFIFOHE_Pos) ## !< 0x00004000
  SDIO_STA_TXFIFOHE* = SDIO_STA_TXFIFOHE_Msk
  SDIO_STA_RXFIFOHF_Pos* = (15)
  SDIO_STA_RXFIFOHF_Msk* = (0x00000001 shl SDIO_STA_RXFIFOHF_Pos) ## !< 0x00008000
  SDIO_STA_RXFIFOHF* = SDIO_STA_RXFIFOHF_Msk
  SDIO_STA_TXFIFOF_Pos* = (16)
  SDIO_STA_TXFIFOF_Msk* = (0x00000001 shl SDIO_STA_TXFIFOF_Pos) ## !< 0x00010000
  SDIO_STA_TXFIFOF* = SDIO_STA_TXFIFOF_Msk
  SDIO_STA_RXFIFOF_Pos* = (17)
  SDIO_STA_RXFIFOF_Msk* = (0x00000001 shl SDIO_STA_RXFIFOF_Pos) ## !< 0x00020000
  SDIO_STA_RXFIFOF* = SDIO_STA_RXFIFOF_Msk
  SDIO_STA_TXFIFOE_Pos* = (18)
  SDIO_STA_TXFIFOE_Msk* = (0x00000001 shl SDIO_STA_TXFIFOE_Pos) ## !< 0x00040000
  SDIO_STA_TXFIFOE* = SDIO_STA_TXFIFOE_Msk
  SDIO_STA_RXFIFOE_Pos* = (19)
  SDIO_STA_RXFIFOE_Msk* = (0x00000001 shl SDIO_STA_RXFIFOE_Pos) ## !< 0x00080000
  SDIO_STA_RXFIFOE* = SDIO_STA_RXFIFOE_Msk
  SDIO_STA_TXDAVL_Pos* = (20)
  SDIO_STA_TXDAVL_Msk* = (0x00000001 shl SDIO_STA_TXDAVL_Pos) ## !< 0x00100000
  SDIO_STA_TXDAVL* = SDIO_STA_TXDAVL_Msk
  SDIO_STA_RXDAVL_Pos* = (21)
  SDIO_STA_RXDAVL_Msk* = (0x00000001 shl SDIO_STA_RXDAVL_Pos) ## !< 0x00200000
  SDIO_STA_RXDAVL* = SDIO_STA_RXDAVL_Msk
  SDIO_STA_SDIOIT_Pos* = (22)
  SDIO_STA_SDIOIT_Msk* = (0x00000001 shl SDIO_STA_SDIOIT_Pos) ## !< 0x00400000
  SDIO_STA_SDIOIT* = SDIO_STA_SDIOIT_Msk
  SDIO_STA_CEATAEND_Pos* = (23)
  SDIO_STA_CEATAEND_Msk* = (0x00000001 shl SDIO_STA_CEATAEND_Pos) ## !< 0x00800000
  SDIO_STA_CEATAEND* = SDIO_STA_CEATAEND_Msk

## ******************  Bit definition for SDIO_ICR register  ******************

const
  SDIO_ICR_CCRCFAILC_Pos* = (0)
  SDIO_ICR_CCRCFAILC_Msk* = (0x00000001 shl SDIO_ICR_CCRCFAILC_Pos) ## !< 0x00000001
  SDIO_ICR_CCRCFAILC* = SDIO_ICR_CCRCFAILC_Msk
  SDIO_ICR_DCRCFAILC_Pos* = (1)
  SDIO_ICR_DCRCFAILC_Msk* = (0x00000001 shl SDIO_ICR_DCRCFAILC_Pos) ## !< 0x00000002
  SDIO_ICR_DCRCFAILC* = SDIO_ICR_DCRCFAILC_Msk
  SDIO_ICR_CTIMEOUTC_Pos* = (2)
  SDIO_ICR_CTIMEOUTC_Msk* = (0x00000001 shl SDIO_ICR_CTIMEOUTC_Pos) ## !< 0x00000004
  SDIO_ICR_CTIMEOUTC* = SDIO_ICR_CTIMEOUTC_Msk
  SDIO_ICR_DTIMEOUTC_Pos* = (3)
  SDIO_ICR_DTIMEOUTC_Msk* = (0x00000001 shl SDIO_ICR_DTIMEOUTC_Pos) ## !< 0x00000008
  SDIO_ICR_DTIMEOUTC* = SDIO_ICR_DTIMEOUTC_Msk
  SDIO_ICR_TXUNDERRC_Pos* = (4)
  SDIO_ICR_TXUNDERRC_Msk* = (0x00000001 shl SDIO_ICR_TXUNDERRC_Pos) ## !< 0x00000010
  SDIO_ICR_TXUNDERRC* = SDIO_ICR_TXUNDERRC_Msk
  SDIO_ICR_RXOVERRC_Pos* = (5)
  SDIO_ICR_RXOVERRC_Msk* = (0x00000001 shl SDIO_ICR_RXOVERRC_Pos) ## !< 0x00000020
  SDIO_ICR_RXOVERRC* = SDIO_ICR_RXOVERRC_Msk
  SDIO_ICR_CMDRENDC_Pos* = (6)
  SDIO_ICR_CMDRENDC_Msk* = (0x00000001 shl SDIO_ICR_CMDRENDC_Pos) ## !< 0x00000040
  SDIO_ICR_CMDRENDC* = SDIO_ICR_CMDRENDC_Msk
  SDIO_ICR_CMDSENTC_Pos* = (7)
  SDIO_ICR_CMDSENTC_Msk* = (0x00000001 shl SDIO_ICR_CMDSENTC_Pos) ## !< 0x00000080
  SDIO_ICR_CMDSENTC* = SDIO_ICR_CMDSENTC_Msk
  SDIO_ICR_DATAENDC_Pos* = (8)
  SDIO_ICR_DATAENDC_Msk* = (0x00000001 shl SDIO_ICR_DATAENDC_Pos) ## !< 0x00000100
  SDIO_ICR_DATAENDC* = SDIO_ICR_DATAENDC_Msk
  SDIO_ICR_STBITERRC_Pos* = (9)
  SDIO_ICR_STBITERRC_Msk* = (0x00000001 shl SDIO_ICR_STBITERRC_Pos) ## !< 0x00000200
  SDIO_ICR_STBITERRC* = SDIO_ICR_STBITERRC_Msk
  SDIO_ICR_DBCKENDC_Pos* = (10)
  SDIO_ICR_DBCKENDC_Msk* = (0x00000001 shl SDIO_ICR_DBCKENDC_Pos) ## !< 0x00000400
  SDIO_ICR_DBCKENDC* = SDIO_ICR_DBCKENDC_Msk
  SDIO_ICR_SDIOITC_Pos* = (22)
  SDIO_ICR_SDIOITC_Msk* = (0x00000001 shl SDIO_ICR_SDIOITC_Pos) ## !< 0x00400000
  SDIO_ICR_SDIOITC* = SDIO_ICR_SDIOITC_Msk
  SDIO_ICR_CEATAENDC_Pos* = (23)
  SDIO_ICR_CEATAENDC_Msk* = (0x00000001 shl SDIO_ICR_CEATAENDC_Pos) ## !< 0x00800000
  SDIO_ICR_CEATAENDC* = SDIO_ICR_CEATAENDC_Msk

## *****************  Bit definition for SDIO_MASK register  ******************

const
  SDIO_MASK_CCRCFAILIE_Pos* = (0)
  SDIO_MASK_CCRCFAILIE_Msk* = (0x00000001 shl SDIO_MASK_CCRCFAILIE_Pos) ## !< 0x00000001
  SDIO_MASK_CCRCFAILIE* = SDIO_MASK_CCRCFAILIE_Msk
  SDIO_MASK_DCRCFAILIE_Pos* = (1)
  SDIO_MASK_DCRCFAILIE_Msk* = (0x00000001 shl SDIO_MASK_DCRCFAILIE_Pos) ## !< 0x00000002
  SDIO_MASK_DCRCFAILIE* = SDIO_MASK_DCRCFAILIE_Msk
  SDIO_MASK_CTIMEOUTIE_Pos* = (2)
  SDIO_MASK_CTIMEOUTIE_Msk* = (0x00000001 shl SDIO_MASK_CTIMEOUTIE_Pos) ## !< 0x00000004
  SDIO_MASK_CTIMEOUTIE* = SDIO_MASK_CTIMEOUTIE_Msk
  SDIO_MASK_DTIMEOUTIE_Pos* = (3)
  SDIO_MASK_DTIMEOUTIE_Msk* = (0x00000001 shl SDIO_MASK_DTIMEOUTIE_Pos) ## !< 0x00000008
  SDIO_MASK_DTIMEOUTIE* = SDIO_MASK_DTIMEOUTIE_Msk
  SDIO_MASK_TXUNDERRIE_Pos* = (4)
  SDIO_MASK_TXUNDERRIE_Msk* = (0x00000001 shl SDIO_MASK_TXUNDERRIE_Pos) ## !< 0x00000010
  SDIO_MASK_TXUNDERRIE* = SDIO_MASK_TXUNDERRIE_Msk
  SDIO_MASK_RXOVERRIE_Pos* = (5)
  SDIO_MASK_RXOVERRIE_Msk* = (0x00000001 shl SDIO_MASK_RXOVERRIE_Pos) ## !< 0x00000020
  SDIO_MASK_RXOVERRIE* = SDIO_MASK_RXOVERRIE_Msk
  SDIO_MASK_CMDRENDIE_Pos* = (6)
  SDIO_MASK_CMDRENDIE_Msk* = (0x00000001 shl SDIO_MASK_CMDRENDIE_Pos) ## !< 0x00000040
  SDIO_MASK_CMDRENDIE* = SDIO_MASK_CMDRENDIE_Msk
  SDIO_MASK_CMDSENTIE_Pos* = (7)
  SDIO_MASK_CMDSENTIE_Msk* = (0x00000001 shl SDIO_MASK_CMDSENTIE_Pos) ## !< 0x00000080
  SDIO_MASK_CMDSENTIE* = SDIO_MASK_CMDSENTIE_Msk
  SDIO_MASK_DATAENDIE_Pos* = (8)
  SDIO_MASK_DATAENDIE_Msk* = (0x00000001 shl SDIO_MASK_DATAENDIE_Pos) ## !< 0x00000100
  SDIO_MASK_DATAENDIE* = SDIO_MASK_DATAENDIE_Msk
  SDIO_MASK_STBITERRIE_Pos* = (9)
  SDIO_MASK_STBITERRIE_Msk* = (0x00000001 shl SDIO_MASK_STBITERRIE_Pos) ## !< 0x00000200
  SDIO_MASK_STBITERRIE* = SDIO_MASK_STBITERRIE_Msk
  SDIO_MASK_DBCKENDIE_Pos* = (10)
  SDIO_MASK_DBCKENDIE_Msk* = (0x00000001 shl SDIO_MASK_DBCKENDIE_Pos) ## !< 0x00000400
  SDIO_MASK_DBCKENDIE* = SDIO_MASK_DBCKENDIE_Msk
  SDIO_MASK_CMDACTIE_Pos* = (11)
  SDIO_MASK_CMDACTIE_Msk* = (0x00000001 shl SDIO_MASK_CMDACTIE_Pos) ## !< 0x00000800
  SDIO_MASK_CMDACTIE* = SDIO_MASK_CMDACTIE_Msk
  SDIO_MASK_TXACTIE_Pos* = (12)
  SDIO_MASK_TXACTIE_Msk* = (0x00000001 shl SDIO_MASK_TXACTIE_Pos) ## !< 0x00001000
  SDIO_MASK_TXACTIE* = SDIO_MASK_TXACTIE_Msk
  SDIO_MASK_RXACTIE_Pos* = (13)
  SDIO_MASK_RXACTIE_Msk* = (0x00000001 shl SDIO_MASK_RXACTIE_Pos) ## !< 0x00002000
  SDIO_MASK_RXACTIE* = SDIO_MASK_RXACTIE_Msk
  SDIO_MASK_TXFIFOHEIE_Pos* = (14)
  SDIO_MASK_TXFIFOHEIE_Msk* = (0x00000001 shl SDIO_MASK_TXFIFOHEIE_Pos) ## !< 0x00004000
  SDIO_MASK_TXFIFOHEIE* = SDIO_MASK_TXFIFOHEIE_Msk
  SDIO_MASK_RXFIFOHFIE_Pos* = (15)
  SDIO_MASK_RXFIFOHFIE_Msk* = (0x00000001 shl SDIO_MASK_RXFIFOHFIE_Pos) ## !< 0x00008000
  SDIO_MASK_RXFIFOHFIE* = SDIO_MASK_RXFIFOHFIE_Msk
  SDIO_MASK_TXFIFOFIE_Pos* = (16)
  SDIO_MASK_TXFIFOFIE_Msk* = (0x00000001 shl SDIO_MASK_TXFIFOFIE_Pos) ## !< 0x00010000
  SDIO_MASK_TXFIFOFIE* = SDIO_MASK_TXFIFOFIE_Msk
  SDIO_MASK_RXFIFOFIE_Pos* = (17)
  SDIO_MASK_RXFIFOFIE_Msk* = (0x00000001 shl SDIO_MASK_RXFIFOFIE_Pos) ## !< 0x00020000
  SDIO_MASK_RXFIFOFIE* = SDIO_MASK_RXFIFOFIE_Msk
  SDIO_MASK_TXFIFOEIE_Pos* = (18)
  SDIO_MASK_TXFIFOEIE_Msk* = (0x00000001 shl SDIO_MASK_TXFIFOEIE_Pos) ## !< 0x00040000
  SDIO_MASK_TXFIFOEIE* = SDIO_MASK_TXFIFOEIE_Msk
  SDIO_MASK_RXFIFOEIE_Pos* = (19)
  SDIO_MASK_RXFIFOEIE_Msk* = (0x00000001 shl SDIO_MASK_RXFIFOEIE_Pos) ## !< 0x00080000
  SDIO_MASK_RXFIFOEIE* = SDIO_MASK_RXFIFOEIE_Msk
  SDIO_MASK_TXDAVLIE_Pos* = (20)
  SDIO_MASK_TXDAVLIE_Msk* = (0x00000001 shl SDIO_MASK_TXDAVLIE_Pos) ## !< 0x00100000
  SDIO_MASK_TXDAVLIE* = SDIO_MASK_TXDAVLIE_Msk
  SDIO_MASK_RXDAVLIE_Pos* = (21)
  SDIO_MASK_RXDAVLIE_Msk* = (0x00000001 shl SDIO_MASK_RXDAVLIE_Pos) ## !< 0x00200000
  SDIO_MASK_RXDAVLIE* = SDIO_MASK_RXDAVLIE_Msk
  SDIO_MASK_SDIOITIE_Pos* = (22)
  SDIO_MASK_SDIOITIE_Msk* = (0x00000001 shl SDIO_MASK_SDIOITIE_Pos) ## !< 0x00400000
  SDIO_MASK_SDIOITIE* = SDIO_MASK_SDIOITIE_Msk
  SDIO_MASK_CEATAENDIE_Pos* = (23)
  SDIO_MASK_CEATAENDIE_Msk* = (0x00000001 shl SDIO_MASK_CEATAENDIE_Pos) ## !< 0x00800000
  SDIO_MASK_CEATAENDIE* = SDIO_MASK_CEATAENDIE_Msk

## ****************  Bit definition for SDIO_FIFOCNT register  ****************

const
  SDIO_FIFOCNT_FIFOCOUNT_Pos* = (0)
  SDIO_FIFOCNT_FIFOCOUNT_Msk* = (0x00FFFFFF shl SDIO_FIFOCNT_FIFOCOUNT_Pos) ## !< 0x00FFFFFF
  SDIO_FIFOCNT_FIFOCOUNT* = SDIO_FIFOCNT_FIFOCOUNT_Msk

## *****************  Bit definition for SDIO_FIFO register  ******************

const
  SDIO_FIFO_FIFODATA_Pos* = (0)
  SDIO_FIFO_FIFODATA_Msk* = (0xFFFFFFFF shl SDIO_FIFO_FIFODATA_Pos) ## !< 0xFFFFFFFF
  SDIO_FIFO_FIFODATA* = SDIO_FIFO_FIFODATA_Msk

## ****************************************************************************
##
##                         Serial Peripheral Interface
##
## ****************************************************************************

const
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
##                                  SYSCFG
##
## ****************************************************************************
## *****************  Bit definition for SYSCFG_MEMRMP register  **************

const
  SYSCFG_MEMRMP_MEM_MODE_Pos* = (0)
  SYSCFG_MEMRMP_MEM_MODE_Msk* = (0x00000003 shl SYSCFG_MEMRMP_MEM_MODE_Pos) ## !< 0x00000003
  SYSCFG_MEMRMP_MEM_MODE* = SYSCFG_MEMRMP_MEM_MODE_Msk
  SYSCFG_MEMRMP_MEM_MODE_0* = (0x00000001 shl SYSCFG_MEMRMP_MEM_MODE_Pos) ## !< 0x00000001
  SYSCFG_MEMRMP_MEM_MODE_1* = (0x00000002 shl SYSCFG_MEMRMP_MEM_MODE_Pos) ## !< 0x00000002

## *****************  Bit definition for SYSCFG_PMC register  *****************

const
  SYSCFG_PMC_MII_RMII_SEL_Pos* = (23)
  SYSCFG_PMC_MII_RMII_SEL_Msk* = (0x00000001 shl SYSCFG_PMC_MII_RMII_SEL_Pos) ## !< 0x00800000
  SYSCFG_PMC_MII_RMII_SEL* = SYSCFG_PMC_MII_RMII_SEL_Msk

##  Old MII_RMII_SEL bit definition, maintained for legacy purpose

const
  SYSCFG_PMC_MII_RMII* = SYSCFG_PMC_MII_RMII_SEL

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
##  @brief   EXTI0 configuration
##

const
  SYSCFG_EXTICR1_EXTI0_PA* = 0x00000000
  SYSCFG_EXTICR1_EXTI0_PB* = 0x00000001
  SYSCFG_EXTICR1_EXTI0_PC* = 0x00000002
  SYSCFG_EXTICR1_EXTI0_PD* = 0x00000003
  SYSCFG_EXTICR1_EXTI0_PE* = 0x00000004
  SYSCFG_EXTICR1_EXTI0_PF* = 0x00000005
  SYSCFG_EXTICR1_EXTI0_PG* = 0x00000006
  SYSCFG_EXTICR1_EXTI0_PH* = 0x00000007
  SYSCFG_EXTICR1_EXTI0_PI* = 0x00000008

## *
##  @brief   EXTI1 configuration
##

const
  SYSCFG_EXTICR1_EXTI1_PA* = 0x00000000
  SYSCFG_EXTICR1_EXTI1_PB* = 0x00000010
  SYSCFG_EXTICR1_EXTI1_PC* = 0x00000020
  SYSCFG_EXTICR1_EXTI1_PD* = 0x00000030
  SYSCFG_EXTICR1_EXTI1_PE* = 0x00000040
  SYSCFG_EXTICR1_EXTI1_PF* = 0x00000050
  SYSCFG_EXTICR1_EXTI1_PG* = 0x00000060
  SYSCFG_EXTICR1_EXTI1_PH* = 0x00000070
  SYSCFG_EXTICR1_EXTI1_PI* = 0x00000080

## *
##  @brief   EXTI2 configuration
##

const
  SYSCFG_EXTICR1_EXTI2_PA* = 0x00000000
  SYSCFG_EXTICR1_EXTI2_PB* = 0x00000100
  SYSCFG_EXTICR1_EXTI2_PC* = 0x00000200
  SYSCFG_EXTICR1_EXTI2_PD* = 0x00000300
  SYSCFG_EXTICR1_EXTI2_PE* = 0x00000400
  SYSCFG_EXTICR1_EXTI2_PF* = 0x00000500
  SYSCFG_EXTICR1_EXTI2_PG* = 0x00000600
  SYSCFG_EXTICR1_EXTI2_PH* = 0x00000700
  SYSCFG_EXTICR1_EXTI2_PI* = 0x00000800

## *
##  @brief   EXTI3 configuration
##

const
  SYSCFG_EXTICR1_EXTI3_PA* = 0x00000000
  SYSCFG_EXTICR1_EXTI3_PB* = 0x00001000
  SYSCFG_EXTICR1_EXTI3_PC* = 0x00002000
  SYSCFG_EXTICR1_EXTI3_PD* = 0x00003000
  SYSCFG_EXTICR1_EXTI3_PE* = 0x00004000
  SYSCFG_EXTICR1_EXTI3_PF* = 0x00005000
  SYSCFG_EXTICR1_EXTI3_PG* = 0x00006000
  SYSCFG_EXTICR1_EXTI3_PH* = 0x00007000
  SYSCFG_EXTICR1_EXTI3_PI* = 0x00008000

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

## *
##  @brief   EXTI4 configuration
##

const
  SYSCFG_EXTICR2_EXTI4_PA* = 0x00000000
  SYSCFG_EXTICR2_EXTI4_PB* = 0x00000001
  SYSCFG_EXTICR2_EXTI4_PC* = 0x00000002
  SYSCFG_EXTICR2_EXTI4_PD* = 0x00000003
  SYSCFG_EXTICR2_EXTI4_PE* = 0x00000004
  SYSCFG_EXTICR2_EXTI4_PF* = 0x00000005
  SYSCFG_EXTICR2_EXTI4_PG* = 0x00000006
  SYSCFG_EXTICR2_EXTI4_PH* = 0x00000007
  SYSCFG_EXTICR2_EXTI4_PI* = 0x00000008

## *
##  @brief   EXTI5 configuration
##

const
  SYSCFG_EXTICR2_EXTI5_PA* = 0x00000000
  SYSCFG_EXTICR2_EXTI5_PB* = 0x00000010
  SYSCFG_EXTICR2_EXTI5_PC* = 0x00000020
  SYSCFG_EXTICR2_EXTI5_PD* = 0x00000030
  SYSCFG_EXTICR2_EXTI5_PE* = 0x00000040
  SYSCFG_EXTICR2_EXTI5_PF* = 0x00000050
  SYSCFG_EXTICR2_EXTI5_PG* = 0x00000060
  SYSCFG_EXTICR2_EXTI5_PH* = 0x00000070
  SYSCFG_EXTICR2_EXTI5_PI* = 0x00000080

## *
##  @brief   EXTI6 configuration
##

const
  SYSCFG_EXTICR2_EXTI6_PA* = 0x00000000
  SYSCFG_EXTICR2_EXTI6_PB* = 0x00000100
  SYSCFG_EXTICR2_EXTI6_PC* = 0x00000200
  SYSCFG_EXTICR2_EXTI6_PD* = 0x00000300
  SYSCFG_EXTICR2_EXTI6_PE* = 0x00000400
  SYSCFG_EXTICR2_EXTI6_PF* = 0x00000500
  SYSCFG_EXTICR2_EXTI6_PG* = 0x00000600
  SYSCFG_EXTICR2_EXTI6_PH* = 0x00000700
  SYSCFG_EXTICR2_EXTI6_PI* = 0x00000800

## *
##  @brief   EXTI7 configuration
##

const
  SYSCFG_EXTICR2_EXTI7_PA* = 0x00000000
  SYSCFG_EXTICR2_EXTI7_PB* = 0x00001000
  SYSCFG_EXTICR2_EXTI7_PC* = 0x00002000
  SYSCFG_EXTICR2_EXTI7_PD* = 0x00003000
  SYSCFG_EXTICR2_EXTI7_PE* = 0x00004000
  SYSCFG_EXTICR2_EXTI7_PF* = 0x00005000
  SYSCFG_EXTICR2_EXTI7_PG* = 0x00006000
  SYSCFG_EXTICR2_EXTI7_PH* = 0x00007000
  SYSCFG_EXTICR2_EXTI7_PI* = 0x00008000

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

## *
##  @brief   EXTI8 configuration
##

const
  SYSCFG_EXTICR3_EXTI8_PA* = 0x00000000
  SYSCFG_EXTICR3_EXTI8_PB* = 0x00000001
  SYSCFG_EXTICR3_EXTI8_PC* = 0x00000002
  SYSCFG_EXTICR3_EXTI8_PD* = 0x00000003
  SYSCFG_EXTICR3_EXTI8_PE* = 0x00000004
  SYSCFG_EXTICR3_EXTI8_PF* = 0x00000005
  SYSCFG_EXTICR3_EXTI8_PG* = 0x00000006
  SYSCFG_EXTICR3_EXTI8_PH* = 0x00000007
  SYSCFG_EXTICR3_EXTI8_PI* = 0x00000008

## *
##  @brief   EXTI9 configuration
##

const
  SYSCFG_EXTICR3_EXTI9_PA* = 0x00000000
  SYSCFG_EXTICR3_EXTI9_PB* = 0x00000010
  SYSCFG_EXTICR3_EXTI9_PC* = 0x00000020
  SYSCFG_EXTICR3_EXTI9_PD* = 0x00000030
  SYSCFG_EXTICR3_EXTI9_PE* = 0x00000040
  SYSCFG_EXTICR3_EXTI9_PF* = 0x00000050
  SYSCFG_EXTICR3_EXTI9_PG* = 0x00000060
  SYSCFG_EXTICR3_EXTI9_PH* = 0x00000070
  SYSCFG_EXTICR3_EXTI9_PI* = 0x00000080

## *
##  @brief   EXTI10 configuration
##

const
  SYSCFG_EXTICR3_EXTI10_PA* = 0x00000000
  SYSCFG_EXTICR3_EXTI10_PB* = 0x00000100
  SYSCFG_EXTICR3_EXTI10_PC* = 0x00000200
  SYSCFG_EXTICR3_EXTI10_PD* = 0x00000300
  SYSCFG_EXTICR3_EXTI10_PE* = 0x00000400
  SYSCFG_EXTICR3_EXTI10_PF* = 0x00000500
  SYSCFG_EXTICR3_EXTI10_PG* = 0x00000600
  SYSCFG_EXTICR3_EXTI10_PH* = 0x00000700
  SYSCFG_EXTICR3_EXTI10_PI* = 0x00000800

## *
##  @brief   EXTI11 configuration
##

const
  SYSCFG_EXTICR3_EXTI11_PA* = 0x00000000
  SYSCFG_EXTICR3_EXTI11_PB* = 0x00001000
  SYSCFG_EXTICR3_EXTI11_PC* = 0x00002000
  SYSCFG_EXTICR3_EXTI11_PD* = 0x00003000
  SYSCFG_EXTICR3_EXTI11_PE* = 0x00004000
  SYSCFG_EXTICR3_EXTI11_PF* = 0x00005000
  SYSCFG_EXTICR3_EXTI11_PG* = 0x00006000
  SYSCFG_EXTICR3_EXTI11_PH* = 0x00007000
  SYSCFG_EXTICR3_EXTI11_PI* = 0x00008000

## ****************  Bit definition for SYSCFG_EXTICR4 register  **************

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
##  @brief   EXTI12 configuration
##

const
  SYSCFG_EXTICR4_EXTI12_PA* = 0x00000000
  SYSCFG_EXTICR4_EXTI12_PB* = 0x00000001
  SYSCFG_EXTICR4_EXTI12_PC* = 0x00000002
  SYSCFG_EXTICR4_EXTI12_PD* = 0x00000003
  SYSCFG_EXTICR4_EXTI12_PE* = 0x00000004
  SYSCFG_EXTICR4_EXTI12_PF* = 0x00000005
  SYSCFG_EXTICR4_EXTI12_PG* = 0x00000006
  SYSCFG_EXTICR4_EXTI12_PH* = 0x00000007

## *
##  @brief   EXTI13 configuration
##

const
  SYSCFG_EXTICR4_EXTI13_PA* = 0x00000000
  SYSCFG_EXTICR4_EXTI13_PB* = 0x00000010
  SYSCFG_EXTICR4_EXTI13_PC* = 0x00000020
  SYSCFG_EXTICR4_EXTI13_PD* = 0x00000030
  SYSCFG_EXTICR4_EXTI13_PE* = 0x00000040
  SYSCFG_EXTICR4_EXTI13_PF* = 0x00000050
  SYSCFG_EXTICR4_EXTI13_PG* = 0x00000060
  SYSCFG_EXTICR4_EXTI13_PH* = 0x00000070

## *
##  @brief   EXTI14 configuration
##

const
  SYSCFG_EXTICR4_EXTI14_PA* = 0x00000000
  SYSCFG_EXTICR4_EXTI14_PB* = 0x00000100
  SYSCFG_EXTICR4_EXTI14_PC* = 0x00000200
  SYSCFG_EXTICR4_EXTI14_PD* = 0x00000300
  SYSCFG_EXTICR4_EXTI14_PE* = 0x00000400
  SYSCFG_EXTICR4_EXTI14_PF* = 0x00000500
  SYSCFG_EXTICR4_EXTI14_PG* = 0x00000600
  SYSCFG_EXTICR4_EXTI14_PH* = 0x00000700

## *
##  @brief   EXTI15 configuration
##

const
  SYSCFG_EXTICR4_EXTI15_PA* = 0x00000000
  SYSCFG_EXTICR4_EXTI15_PB* = 0x00001000
  SYSCFG_EXTICR4_EXTI15_PC* = 0x00002000
  SYSCFG_EXTICR4_EXTI15_PD* = 0x00003000
  SYSCFG_EXTICR4_EXTI15_PE* = 0x00004000
  SYSCFG_EXTICR4_EXTI15_PF* = 0x00005000
  SYSCFG_EXTICR4_EXTI15_PG* = 0x00006000
  SYSCFG_EXTICR4_EXTI15_PH* = 0x00007000

## *****************  Bit definition for SYSCFG_CMPCR register  ***************

const
  SYSCFG_CMPCR_CMP_PD_Pos* = (0)
  SYSCFG_CMPCR_CMP_PD_Msk* = (0x00000001 shl SYSCFG_CMPCR_CMP_PD_Pos) ## !< 0x00000001
  SYSCFG_CMPCR_CMP_PD* = SYSCFG_CMPCR_CMP_PD_Msk
  SYSCFG_CMPCR_READY_Pos* = (8)
  SYSCFG_CMPCR_READY_Msk* = (0x00000001 shl SYSCFG_CMPCR_READY_Pos) ## !< 0x00000100
  SYSCFG_CMPCR_READY* = SYSCFG_CMPCR_READY_Msk

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
  TIM_CR1_CMS_0* = (0x00000001 shl TIM_CR1_CMS_Pos) ## !< 0x0020
  TIM_CR1_CMS_1* = (0x00000002 shl TIM_CR1_CMS_Pos) ## !< 0x0040
  TIM_CR1_ARPE_Pos* = (7)
  TIM_CR1_ARPE_Msk* = (0x00000001 shl TIM_CR1_ARPE_Pos) ## !< 0x00000080
  TIM_CR1_ARPE* = TIM_CR1_ARPE_Msk
  TIM_CR1_CKD_Pos* = (8)
  TIM_CR1_CKD_Msk* = (0x00000003 shl TIM_CR1_CKD_Pos) ## !< 0x00000300
  TIM_CR1_CKD* = TIM_CR1_CKD_Msk
  TIM_CR1_CKD_0* = (0x00000001 shl TIM_CR1_CKD_Pos) ## !< 0x0100
  TIM_CR1_CKD_1* = (0x00000002 shl TIM_CR1_CKD_Pos) ## !< 0x0200

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
  TIM_CR2_MMS_0* = (0x00000001 shl TIM_CR2_MMS_Pos) ## !< 0x0010
  TIM_CR2_MMS_1* = (0x00000002 shl TIM_CR2_MMS_Pos) ## !< 0x0020
  TIM_CR2_MMS_2* = (0x00000004 shl TIM_CR2_MMS_Pos) ## !< 0x0040
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

## ******************  Bit definition for TIM_SMCR register  ******************

const
  TIM_SMCR_SMS_Pos* = (0)
  TIM_SMCR_SMS_Msk* = (0x00000007 shl TIM_SMCR_SMS_Pos) ## !< 0x00000007
  TIM_SMCR_SMS* = TIM_SMCR_SMS_Msk
  TIM_SMCR_SMS_0* = (0x00000001 shl TIM_SMCR_SMS_Pos) ## !< 0x0001
  TIM_SMCR_SMS_1* = (0x00000002 shl TIM_SMCR_SMS_Pos) ## !< 0x0002
  TIM_SMCR_SMS_2* = (0x00000004 shl TIM_SMCR_SMS_Pos) ## !< 0x0004
  TIM_SMCR_TS_Pos* = (4)
  TIM_SMCR_TS_Msk* = (0x00000007 shl TIM_SMCR_TS_Pos) ## !< 0x00000070
  TIM_SMCR_TS* = TIM_SMCR_TS_Msk
  TIM_SMCR_TS_0* = (0x00000001 shl TIM_SMCR_TS_Pos) ## !< 0x0010
  TIM_SMCR_TS_1* = (0x00000002 shl TIM_SMCR_TS_Pos) ## !< 0x0020
  TIM_SMCR_TS_2* = (0x00000004 shl TIM_SMCR_TS_Pos) ## !< 0x0040
  TIM_SMCR_MSM_Pos* = (7)
  TIM_SMCR_MSM_Msk* = (0x00000001 shl TIM_SMCR_MSM_Pos) ## !< 0x00000080
  TIM_SMCR_MSM* = TIM_SMCR_MSM_Msk
  TIM_SMCR_ETF_Pos* = (8)
  TIM_SMCR_ETF_Msk* = (0x0000000F shl TIM_SMCR_ETF_Pos) ## !< 0x00000F00
  TIM_SMCR_ETF* = TIM_SMCR_ETF_Msk
  TIM_SMCR_ETF_0* = (0x00000001 shl TIM_SMCR_ETF_Pos) ## !< 0x0100
  TIM_SMCR_ETF_1* = (0x00000002 shl TIM_SMCR_ETF_Pos) ## !< 0x0200
  TIM_SMCR_ETF_2* = (0x00000004 shl TIM_SMCR_ETF_Pos) ## !< 0x0400
  TIM_SMCR_ETF_3* = (0x00000008 shl TIM_SMCR_ETF_Pos) ## !< 0x0800
  TIM_SMCR_ETPS_Pos* = (12)
  TIM_SMCR_ETPS_Msk* = (0x00000003 shl TIM_SMCR_ETPS_Pos) ## !< 0x00003000
  TIM_SMCR_ETPS* = TIM_SMCR_ETPS_Msk
  TIM_SMCR_ETPS_0* = (0x00000001 shl TIM_SMCR_ETPS_Pos) ## !< 0x1000
  TIM_SMCR_ETPS_1* = (0x00000002 shl TIM_SMCR_ETPS_Pos) ## !< 0x2000
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
  TIM_EGR_COMG_Pos* = (5)
  TIM_EGR_COMG_Msk* = (0x00000001 shl TIM_EGR_COMG_Pos) ## !< 0x00000020
  TIM_EGR_COMG* = TIM_EGR_COMG_Msk
  TIM_EGR_TG_Pos* = (6)
  TIM_EGR_TG_Msk* = (0x00000001 shl TIM_EGR_TG_Pos) ## !< 0x00000040
  TIM_EGR_TG* = TIM_EGR_TG_Msk
  TIM_EGR_BG_Pos* = (7)
  TIM_EGR_BG_Msk* = (0x00000001 shl TIM_EGR_BG_Pos) ## !< 0x00000080
  TIM_EGR_BG* = TIM_EGR_BG_Msk

## *****************  Bit definition for TIM_CCMR1 register  ******************

const
  TIM_CCMR1_CC1S_Pos* = (0)
  TIM_CCMR1_CC1S_Msk* = (0x00000003 shl TIM_CCMR1_CC1S_Pos) ## !< 0x00000003
  TIM_CCMR1_CC1S* = TIM_CCMR1_CC1S_Msk
  TIM_CCMR1_CC1S_0* = (0x00000001 shl TIM_CCMR1_CC1S_Pos) ## !< 0x0001
  TIM_CCMR1_CC1S_1* = (0x00000002 shl TIM_CCMR1_CC1S_Pos) ## !< 0x0002
  TIM_CCMR1_OC1FE_Pos* = (2)
  TIM_CCMR1_OC1FE_Msk* = (0x00000001 shl TIM_CCMR1_OC1FE_Pos) ## !< 0x00000004
  TIM_CCMR1_OC1FE* = TIM_CCMR1_OC1FE_Msk
  TIM_CCMR1_OC1PE_Pos* = (3)
  TIM_CCMR1_OC1PE_Msk* = (0x00000001 shl TIM_CCMR1_OC1PE_Pos) ## !< 0x00000008
  TIM_CCMR1_OC1PE* = TIM_CCMR1_OC1PE_Msk
  TIM_CCMR1_OC1M_Pos* = (4)
  TIM_CCMR1_OC1M_Msk* = (0x00000007 shl TIM_CCMR1_OC1M_Pos) ## !< 0x00000070
  TIM_CCMR1_OC1M* = TIM_CCMR1_OC1M_Msk
  TIM_CCMR1_OC1M_0* = (0x00000001 shl TIM_CCMR1_OC1M_Pos) ## !< 0x0010
  TIM_CCMR1_OC1M_1* = (0x00000002 shl TIM_CCMR1_OC1M_Pos) ## !< 0x0020
  TIM_CCMR1_OC1M_2* = (0x00000004 shl TIM_CCMR1_OC1M_Pos) ## !< 0x0040
  TIM_CCMR1_OC1CE_Pos* = (7)
  TIM_CCMR1_OC1CE_Msk* = (0x00000001 shl TIM_CCMR1_OC1CE_Pos) ## !< 0x00000080
  TIM_CCMR1_OC1CE* = TIM_CCMR1_OC1CE_Msk
  TIM_CCMR1_CC2S_Pos* = (8)
  TIM_CCMR1_CC2S_Msk* = (0x00000003 shl TIM_CCMR1_CC2S_Pos) ## !< 0x00000300
  TIM_CCMR1_CC2S* = TIM_CCMR1_CC2S_Msk
  TIM_CCMR1_CC2S_0* = (0x00000001 shl TIM_CCMR1_CC2S_Pos) ## !< 0x0100
  TIM_CCMR1_CC2S_1* = (0x00000002 shl TIM_CCMR1_CC2S_Pos) ## !< 0x0200
  TIM_CCMR1_OC2FE_Pos* = (10)
  TIM_CCMR1_OC2FE_Msk* = (0x00000001 shl TIM_CCMR1_OC2FE_Pos) ## !< 0x00000400
  TIM_CCMR1_OC2FE* = TIM_CCMR1_OC2FE_Msk
  TIM_CCMR1_OC2PE_Pos* = (11)
  TIM_CCMR1_OC2PE_Msk* = (0x00000001 shl TIM_CCMR1_OC2PE_Pos) ## !< 0x00000800
  TIM_CCMR1_OC2PE* = TIM_CCMR1_OC2PE_Msk
  TIM_CCMR1_OC2M_Pos* = (12)
  TIM_CCMR1_OC2M_Msk* = (0x00000007 shl TIM_CCMR1_OC2M_Pos) ## !< 0x00007000
  TIM_CCMR1_OC2M* = TIM_CCMR1_OC2M_Msk
  TIM_CCMR1_OC2M_0* = (0x00000001 shl TIM_CCMR1_OC2M_Pos) ## !< 0x1000
  TIM_CCMR1_OC2M_1* = (0x00000002 shl TIM_CCMR1_OC2M_Pos) ## !< 0x2000
  TIM_CCMR1_OC2M_2* = (0x00000004 shl TIM_CCMR1_OC2M_Pos) ## !< 0x4000
  TIM_CCMR1_OC2CE_Pos* = (15)
  TIM_CCMR1_OC2CE_Msk* = (0x00000001 shl TIM_CCMR1_OC2CE_Pos) ## !< 0x00008000
  TIM_CCMR1_OC2CE* = TIM_CCMR1_OC2CE_Msk

## ----------------------------------------------------------------------------

const
  TIM_CCMR1_IC1PSC_Pos* = (2)
  TIM_CCMR1_IC1PSC_Msk* = (0x00000003 shl TIM_CCMR1_IC1PSC_Pos) ## !< 0x0000000C
  TIM_CCMR1_IC1PSC* = TIM_CCMR1_IC1PSC_Msk
  TIM_CCMR1_IC1PSC_0* = (0x00000001 shl TIM_CCMR1_IC1PSC_Pos) ## !< 0x0004
  TIM_CCMR1_IC1PSC_1* = (0x00000002 shl TIM_CCMR1_IC1PSC_Pos) ## !< 0x0008
  TIM_CCMR1_IC1F_Pos* = (4)
  TIM_CCMR1_IC1F_Msk* = (0x0000000F shl TIM_CCMR1_IC1F_Pos) ## !< 0x000000F0
  TIM_CCMR1_IC1F* = TIM_CCMR1_IC1F_Msk
  TIM_CCMR1_IC1F_0* = (0x00000001 shl TIM_CCMR1_IC1F_Pos) ## !< 0x0010
  TIM_CCMR1_IC1F_1* = (0x00000002 shl TIM_CCMR1_IC1F_Pos) ## !< 0x0020
  TIM_CCMR1_IC1F_2* = (0x00000004 shl TIM_CCMR1_IC1F_Pos) ## !< 0x0040
  TIM_CCMR1_IC1F_3* = (0x00000008 shl TIM_CCMR1_IC1F_Pos) ## !< 0x0080
  TIM_CCMR1_IC2PSC_Pos* = (10)
  TIM_CCMR1_IC2PSC_Msk* = (0x00000003 shl TIM_CCMR1_IC2PSC_Pos) ## !< 0x00000C00
  TIM_CCMR1_IC2PSC* = TIM_CCMR1_IC2PSC_Msk
  TIM_CCMR1_IC2PSC_0* = (0x00000001 shl TIM_CCMR1_IC2PSC_Pos) ## !< 0x0400
  TIM_CCMR1_IC2PSC_1* = (0x00000002 shl TIM_CCMR1_IC2PSC_Pos) ## !< 0x0800
  TIM_CCMR1_IC2F_Pos* = (12)
  TIM_CCMR1_IC2F_Msk* = (0x0000000F shl TIM_CCMR1_IC2F_Pos) ## !< 0x0000F000
  TIM_CCMR1_IC2F* = TIM_CCMR1_IC2F_Msk
  TIM_CCMR1_IC2F_0* = (0x00000001 shl TIM_CCMR1_IC2F_Pos) ## !< 0x1000
  TIM_CCMR1_IC2F_1* = (0x00000002 shl TIM_CCMR1_IC2F_Pos) ## !< 0x2000
  TIM_CCMR1_IC2F_2* = (0x00000004 shl TIM_CCMR1_IC2F_Pos) ## !< 0x4000
  TIM_CCMR1_IC2F_3* = (0x00000008 shl TIM_CCMR1_IC2F_Pos) ## !< 0x8000

## *****************  Bit definition for TIM_CCMR2 register  ******************

const
  TIM_CCMR2_CC3S_Pos* = (0)
  TIM_CCMR2_CC3S_Msk* = (0x00000003 shl TIM_CCMR2_CC3S_Pos) ## !< 0x00000003
  TIM_CCMR2_CC3S* = TIM_CCMR2_CC3S_Msk
  TIM_CCMR2_CC3S_0* = (0x00000001 shl TIM_CCMR2_CC3S_Pos) ## !< 0x0001
  TIM_CCMR2_CC3S_1* = (0x00000002 shl TIM_CCMR2_CC3S_Pos) ## !< 0x0002
  TIM_CCMR2_OC3FE_Pos* = (2)
  TIM_CCMR2_OC3FE_Msk* = (0x00000001 shl TIM_CCMR2_OC3FE_Pos) ## !< 0x00000004
  TIM_CCMR2_OC3FE* = TIM_CCMR2_OC3FE_Msk
  TIM_CCMR2_OC3PE_Pos* = (3)
  TIM_CCMR2_OC3PE_Msk* = (0x00000001 shl TIM_CCMR2_OC3PE_Pos) ## !< 0x00000008
  TIM_CCMR2_OC3PE* = TIM_CCMR2_OC3PE_Msk
  TIM_CCMR2_OC3M_Pos* = (4)
  TIM_CCMR2_OC3M_Msk* = (0x00000007 shl TIM_CCMR2_OC3M_Pos) ## !< 0x00000070
  TIM_CCMR2_OC3M* = TIM_CCMR2_OC3M_Msk
  TIM_CCMR2_OC3M_0* = (0x00000001 shl TIM_CCMR2_OC3M_Pos) ## !< 0x0010
  TIM_CCMR2_OC3M_1* = (0x00000002 shl TIM_CCMR2_OC3M_Pos) ## !< 0x0020
  TIM_CCMR2_OC3M_2* = (0x00000004 shl TIM_CCMR2_OC3M_Pos) ## !< 0x0040
  TIM_CCMR2_OC3CE_Pos* = (7)
  TIM_CCMR2_OC3CE_Msk* = (0x00000001 shl TIM_CCMR2_OC3CE_Pos) ## !< 0x00000080
  TIM_CCMR2_OC3CE* = TIM_CCMR2_OC3CE_Msk
  TIM_CCMR2_CC4S_Pos* = (8)
  TIM_CCMR2_CC4S_Msk* = (0x00000003 shl TIM_CCMR2_CC4S_Pos) ## !< 0x00000300
  TIM_CCMR2_CC4S* = TIM_CCMR2_CC4S_Msk
  TIM_CCMR2_CC4S_0* = (0x00000001 shl TIM_CCMR2_CC4S_Pos) ## !< 0x0100
  TIM_CCMR2_CC4S_1* = (0x00000002 shl TIM_CCMR2_CC4S_Pos) ## !< 0x0200
  TIM_CCMR2_OC4FE_Pos* = (10)
  TIM_CCMR2_OC4FE_Msk* = (0x00000001 shl TIM_CCMR2_OC4FE_Pos) ## !< 0x00000400
  TIM_CCMR2_OC4FE* = TIM_CCMR2_OC4FE_Msk
  TIM_CCMR2_OC4PE_Pos* = (11)
  TIM_CCMR2_OC4PE_Msk* = (0x00000001 shl TIM_CCMR2_OC4PE_Pos) ## !< 0x00000800
  TIM_CCMR2_OC4PE* = TIM_CCMR2_OC4PE_Msk
  TIM_CCMR2_OC4M_Pos* = (12)
  TIM_CCMR2_OC4M_Msk* = (0x00000007 shl TIM_CCMR2_OC4M_Pos) ## !< 0x00007000
  TIM_CCMR2_OC4M* = TIM_CCMR2_OC4M_Msk
  TIM_CCMR2_OC4M_0* = (0x00000001 shl TIM_CCMR2_OC4M_Pos) ## !< 0x1000
  TIM_CCMR2_OC4M_1* = (0x00000002 shl TIM_CCMR2_OC4M_Pos) ## !< 0x2000
  TIM_CCMR2_OC4M_2* = (0x00000004 shl TIM_CCMR2_OC4M_Pos) ## !< 0x4000
  TIM_CCMR2_OC4CE_Pos* = (15)
  TIM_CCMR2_OC4CE_Msk* = (0x00000001 shl TIM_CCMR2_OC4CE_Pos) ## !< 0x00008000
  TIM_CCMR2_OC4CE* = TIM_CCMR2_OC4CE_Msk

## ----------------------------------------------------------------------------

const
  TIM_CCMR2_IC3PSC_Pos* = (2)
  TIM_CCMR2_IC3PSC_Msk* = (0x00000003 shl TIM_CCMR2_IC3PSC_Pos) ## !< 0x0000000C
  TIM_CCMR2_IC3PSC* = TIM_CCMR2_IC3PSC_Msk
  TIM_CCMR2_IC3PSC_0* = (0x00000001 shl TIM_CCMR2_IC3PSC_Pos) ## !< 0x0004
  TIM_CCMR2_IC3PSC_1* = (0x00000002 shl TIM_CCMR2_IC3PSC_Pos) ## !< 0x0008
  TIM_CCMR2_IC3F_Pos* = (4)
  TIM_CCMR2_IC3F_Msk* = (0x0000000F shl TIM_CCMR2_IC3F_Pos) ## !< 0x000000F0
  TIM_CCMR2_IC3F* = TIM_CCMR2_IC3F_Msk
  TIM_CCMR2_IC3F_0* = (0x00000001 shl TIM_CCMR2_IC3F_Pos) ## !< 0x0010
  TIM_CCMR2_IC3F_1* = (0x00000002 shl TIM_CCMR2_IC3F_Pos) ## !< 0x0020
  TIM_CCMR2_IC3F_2* = (0x00000004 shl TIM_CCMR2_IC3F_Pos) ## !< 0x0040
  TIM_CCMR2_IC3F_3* = (0x00000008 shl TIM_CCMR2_IC3F_Pos) ## !< 0x0080
  TIM_CCMR2_IC4PSC_Pos* = (10)
  TIM_CCMR2_IC4PSC_Msk* = (0x00000003 shl TIM_CCMR2_IC4PSC_Pos) ## !< 0x00000C00
  TIM_CCMR2_IC4PSC* = TIM_CCMR2_IC4PSC_Msk
  TIM_CCMR2_IC4PSC_0* = (0x00000001 shl TIM_CCMR2_IC4PSC_Pos) ## !< 0x0400
  TIM_CCMR2_IC4PSC_1* = (0x00000002 shl TIM_CCMR2_IC4PSC_Pos) ## !< 0x0800
  TIM_CCMR2_IC4F_Pos* = (12)
  TIM_CCMR2_IC4F_Msk* = (0x0000000F shl TIM_CCMR2_IC4F_Pos) ## !< 0x0000F000
  TIM_CCMR2_IC4F* = TIM_CCMR2_IC4F_Msk
  TIM_CCMR2_IC4F_0* = (0x00000001 shl TIM_CCMR2_IC4F_Pos) ## !< 0x1000
  TIM_CCMR2_IC4F_1* = (0x00000002 shl TIM_CCMR2_IC4F_Pos) ## !< 0x2000
  TIM_CCMR2_IC4F_2* = (0x00000004 shl TIM_CCMR2_IC4F_Pos) ## !< 0x4000
  TIM_CCMR2_IC4F_3* = (0x00000008 shl TIM_CCMR2_IC4F_Pos) ## !< 0x8000

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

## ******************  Bit definition for TIM_RCR register  *******************

const
  TIM_RCR_REP_Pos* = (0)
  TIM_RCR_REP_Msk* = (0x000000FF shl TIM_RCR_REP_Pos) ## !< 0x000000FF
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

## ******************  Bit definition for TIM_BDTR register  ******************

const
  TIM_BDTR_DTG_Pos* = (0)
  TIM_BDTR_DTG_Msk* = (0x000000FF shl TIM_BDTR_DTG_Pos) ## !< 0x000000FF
  TIM_BDTR_DTG* = TIM_BDTR_DTG_Msk
  TIM_BDTR_DTG_0* = (0x00000001 shl TIM_BDTR_DTG_Pos) ## !< 0x0001
  TIM_BDTR_DTG_1* = (0x00000002 shl TIM_BDTR_DTG_Pos) ## !< 0x0002
  TIM_BDTR_DTG_2* = (0x00000004 shl TIM_BDTR_DTG_Pos) ## !< 0x0004
  TIM_BDTR_DTG_3* = (0x00000008 shl TIM_BDTR_DTG_Pos) ## !< 0x0008
  TIM_BDTR_DTG_4* = (0x00000010 shl TIM_BDTR_DTG_Pos) ## !< 0x0010
  TIM_BDTR_DTG_5* = (0x00000020 shl TIM_BDTR_DTG_Pos) ## !< 0x0020
  TIM_BDTR_DTG_6* = (0x00000040 shl TIM_BDTR_DTG_Pos) ## !< 0x0040
  TIM_BDTR_DTG_7* = (0x00000080 shl TIM_BDTR_DTG_Pos) ## !< 0x0080
  TIM_BDTR_LOCK_Pos* = (8)
  TIM_BDTR_LOCK_Msk* = (0x00000003 shl TIM_BDTR_LOCK_Pos) ## !< 0x00000300
  TIM_BDTR_LOCK* = TIM_BDTR_LOCK_Msk
  TIM_BDTR_LOCK_0* = (0x00000001 shl TIM_BDTR_LOCK_Pos) ## !< 0x0100
  TIM_BDTR_LOCK_1* = (0x00000002 shl TIM_BDTR_LOCK_Pos) ## !< 0x0200
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

## ******************  Bit definition for TIM_DCR register  *******************

const
  TIM_DCR_DBA_Pos* = (0)
  TIM_DCR_DBA_Msk* = (0x0000001F shl TIM_DCR_DBA_Pos) ## !< 0x0000001F
  TIM_DCR_DBA* = TIM_DCR_DBA_Msk
  TIM_DCR_DBA_0* = (0x00000001 shl TIM_DCR_DBA_Pos) ## !< 0x0001
  TIM_DCR_DBA_1* = (0x00000002 shl TIM_DCR_DBA_Pos) ## !< 0x0002
  TIM_DCR_DBA_2* = (0x00000004 shl TIM_DCR_DBA_Pos) ## !< 0x0004
  TIM_DCR_DBA_3* = (0x00000008 shl TIM_DCR_DBA_Pos) ## !< 0x0008
  TIM_DCR_DBA_4* = (0x00000010 shl TIM_DCR_DBA_Pos) ## !< 0x0010
  TIM_DCR_DBL_Pos* = (8)
  TIM_DCR_DBL_Msk* = (0x0000001F shl TIM_DCR_DBL_Pos) ## !< 0x00001F00
  TIM_DCR_DBL* = TIM_DCR_DBL_Msk
  TIM_DCR_DBL_0* = (0x00000001 shl TIM_DCR_DBL_Pos) ## !< 0x0100
  TIM_DCR_DBL_1* = (0x00000002 shl TIM_DCR_DBL_Pos) ## !< 0x0200
  TIM_DCR_DBL_2* = (0x00000004 shl TIM_DCR_DBL_Pos) ## !< 0x0400
  TIM_DCR_DBL_3* = (0x00000008 shl TIM_DCR_DBL_Pos) ## !< 0x0800
  TIM_DCR_DBL_4* = (0x00000010 shl TIM_DCR_DBL_Pos) ## !< 0x1000

## ******************  Bit definition for TIM_DMAR register  ******************

const
  TIM_DMAR_DMAB_Pos* = (0)
  TIM_DMAR_DMAB_Msk* = (0x0000FFFF shl TIM_DMAR_DMAB_Pos) ## !< 0x0000FFFF
  TIM_DMAR_DMAB* = TIM_DMAR_DMAB_Msk

## ******************  Bit definition for TIM_OR register  ********************

const
  TIM_OR_TI1_RMP_Pos* = (0)
  TIM_OR_TI1_RMP_Msk* = (0x00000003 shl TIM_OR_TI1_RMP_Pos) ## !< 0x00000003
  TIM_OR_TI1_RMP* = TIM_OR_TI1_RMP_Msk
  TIM_OR_TI1_RMP_0* = (0x00000001 shl TIM_OR_TI1_RMP_Pos) ## !< 0x00000001
  TIM_OR_TI1_RMP_1* = (0x00000002 shl TIM_OR_TI1_RMP_Pos) ## !< 0x00000002
  TIM_OR_TI4_RMP_Pos* = (6)
  TIM_OR_TI4_RMP_Msk* = (0x00000003 shl TIM_OR_TI4_RMP_Pos) ## !< 0x000000C0
  TIM_OR_TI4_RMP* = TIM_OR_TI4_RMP_Msk
  TIM_OR_TI4_RMP_0* = (0x00000001 shl TIM_OR_TI4_RMP_Pos) ## !< 0x0040
  TIM_OR_TI4_RMP_1* = (0x00000002 shl TIM_OR_TI4_RMP_Pos) ## !< 0x0080
  TIM_OR_ITR1_RMP_Pos* = (10)
  TIM_OR_ITR1_RMP_Msk* = (0x00000003 shl TIM_OR_ITR1_RMP_Pos) ## !< 0x00000C00
  TIM_OR_ITR1_RMP* = TIM_OR_ITR1_RMP_Msk
  TIM_OR_ITR1_RMP_0* = (0x00000001 shl TIM_OR_ITR1_RMP_Pos) ## !< 0x0400
  TIM_OR_ITR1_RMP_1* = (0x00000002 shl TIM_OR_ITR1_RMP_Pos) ## !< 0x0800

## ****************************************************************************
##
##          Universal Synchronous Asynchronous Receiver Transmitter
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
  USART_BRR_DIV_Fraction_Pos* = (0)
  USART_BRR_DIV_Fraction_Msk* = (0x0000000F shl USART_BRR_DIV_Fraction_Pos) ## !< 0x0000000F
  USART_BRR_DIV_Fraction* = USART_BRR_DIV_Fraction_Msk
  USART_BRR_DIV_Mantissa_Pos* = (4)
  USART_BRR_DIV_Mantissa_Msk* = (0x00000FFF shl USART_BRR_DIV_Mantissa_Pos) ## !< 0x0000FFF0
  USART_BRR_DIV_Mantissa* = USART_BRR_DIV_Mantissa_Msk

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
  USART_CR2_STOP_0* = (0x00000001 shl USART_CR2_STOP_Pos) ## !< 0x1000
  USART_CR2_STOP_1* = (0x00000002 shl USART_CR2_STOP_Pos) ## !< 0x2000
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
  USART_GTPR_PSC_0* = (0x00000001 shl USART_GTPR_PSC_Pos) ## !< 0x0001
  USART_GTPR_PSC_1* = (0x00000002 shl USART_GTPR_PSC_Pos) ## !< 0x0002
  USART_GTPR_PSC_2* = (0x00000004 shl USART_GTPR_PSC_Pos) ## !< 0x0004
  USART_GTPR_PSC_3* = (0x00000008 shl USART_GTPR_PSC_Pos) ## !< 0x0008
  USART_GTPR_PSC_4* = (0x00000010 shl USART_GTPR_PSC_Pos) ## !< 0x0010
  USART_GTPR_PSC_5* = (0x00000020 shl USART_GTPR_PSC_Pos) ## !< 0x0020
  USART_GTPR_PSC_6* = (0x00000040 shl USART_GTPR_PSC_Pos) ## !< 0x0040
  USART_GTPR_PSC_7* = (0x00000080 shl USART_GTPR_PSC_Pos) ## !< 0x0080
  USART_GTPR_GT_Pos* = (8)
  USART_GTPR_GT_Msk* = (0x000000FF shl USART_GTPR_GT_Pos) ## !< 0x0000FF00
  USART_GTPR_GT* = USART_GTPR_GT_Msk

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
  WWDG_CR_T_0* = (0x00000001 shl WWDG_CR_T_Pos) ## !< 0x01
  WWDG_CR_T_1* = (0x00000002 shl WWDG_CR_T_Pos) ## !< 0x02
  WWDG_CR_T_2* = (0x00000004 shl WWDG_CR_T_Pos) ## !< 0x04
  WWDG_CR_T_3* = (0x00000008 shl WWDG_CR_T_Pos) ## !< 0x08
  WWDG_CR_T_4* = (0x00000010 shl WWDG_CR_T_Pos) ## !< 0x10
  WWDG_CR_T_5* = (0x00000020 shl WWDG_CR_T_Pos) ## !< 0x20
  WWDG_CR_T_6* = (0x00000040 shl WWDG_CR_T_Pos) ## !< 0x40

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
  WWDG_CFR_W_0* = (0x00000001 shl WWDG_CFR_W_Pos) ## !< 0x0001
  WWDG_CFR_W_1* = (0x00000002 shl WWDG_CFR_W_Pos) ## !< 0x0002
  WWDG_CFR_W_2* = (0x00000004 shl WWDG_CFR_W_Pos) ## !< 0x0004
  WWDG_CFR_W_3* = (0x00000008 shl WWDG_CFR_W_Pos) ## !< 0x0008
  WWDG_CFR_W_4* = (0x00000010 shl WWDG_CFR_W_Pos) ## !< 0x0010
  WWDG_CFR_W_5* = (0x00000020 shl WWDG_CFR_W_Pos) ## !< 0x0020
  WWDG_CFR_W_6* = (0x00000040 shl WWDG_CFR_W_Pos) ## !< 0x0040

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
  WWDG_CFR_WDGTB_0* = (0x00000001 shl WWDG_CFR_WDGTB_Pos) ## !< 0x0080
  WWDG_CFR_WDGTB_1* = (0x00000002 shl WWDG_CFR_WDGTB_Pos) ## !< 0x0100

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
##                                 DBG
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
  DBGMCU_APB1_FZ_DBG_TIM12_STOP_Pos* = (6)
  DBGMCU_APB1_FZ_DBG_TIM12_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_TIM12_STOP_Pos) ## !< 0x00000040
  DBGMCU_APB1_FZ_DBG_TIM12_STOP* = DBGMCU_APB1_FZ_DBG_TIM12_STOP_Msk
  DBGMCU_APB1_FZ_DBG_TIM13_STOP_Pos* = (7)
  DBGMCU_APB1_FZ_DBG_TIM13_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_TIM13_STOP_Pos) ## !< 0x00000080
  DBGMCU_APB1_FZ_DBG_TIM13_STOP* = DBGMCU_APB1_FZ_DBG_TIM13_STOP_Msk
  DBGMCU_APB1_FZ_DBG_TIM14_STOP_Pos* = (8)
  DBGMCU_APB1_FZ_DBG_TIM14_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_TIM14_STOP_Pos) ## !< 0x00000100
  DBGMCU_APB1_FZ_DBG_TIM14_STOP* = DBGMCU_APB1_FZ_DBG_TIM14_STOP_Msk
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
  DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Pos* = (23)
  DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Pos) ## !< 0x00800000
  DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT* = DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Msk
  DBGMCU_APB1_FZ_DBG_CAN1_STOP_Pos* = (25)
  DBGMCU_APB1_FZ_DBG_CAN1_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_CAN1_STOP_Pos) ## !< 0x02000000
  DBGMCU_APB1_FZ_DBG_CAN1_STOP* = DBGMCU_APB1_FZ_DBG_CAN1_STOP_Msk
  DBGMCU_APB1_FZ_DBG_CAN2_STOP_Pos* = (26)
  DBGMCU_APB1_FZ_DBG_CAN2_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_CAN2_STOP_Pos) ## !< 0x04000000
  DBGMCU_APB1_FZ_DBG_CAN2_STOP* = DBGMCU_APB1_FZ_DBG_CAN2_STOP_Msk

##  Old IWDGSTOP bit definition, maintained for legacy purpose

const
  DBGMCU_APB1_FZ_DBG_IWDEG_STOP* = DBGMCU_APB1_FZ_DBG_IWDG_STOP

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
  DBGMCU_APB2_FZ_DBG_TIM9_STOP_Pos* = (16)
  DBGMCU_APB2_FZ_DBG_TIM9_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM9_STOP_Pos) ## !< 0x00010000
  DBGMCU_APB2_FZ_DBG_TIM9_STOP* = DBGMCU_APB2_FZ_DBG_TIM9_STOP_Msk
  DBGMCU_APB2_FZ_DBG_TIM10_STOP_Pos* = (17)
  DBGMCU_APB2_FZ_DBG_TIM10_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM10_STOP_Pos) ## !< 0x00020000
  DBGMCU_APB2_FZ_DBG_TIM10_STOP* = DBGMCU_APB2_FZ_DBG_TIM10_STOP_Msk
  DBGMCU_APB2_FZ_DBG_TIM11_STOP_Pos* = (18)
  DBGMCU_APB2_FZ_DBG_TIM11_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM11_STOP_Pos) ## !< 0x00040000
  DBGMCU_APB2_FZ_DBG_TIM11_STOP* = DBGMCU_APB2_FZ_DBG_TIM11_STOP_Msk

## ****************************************************************************
##
##                 Ethernet MAC Registers bits definitions
##
## ****************************************************************************
##  Bit definition for Ethernet MAC Control Register register

const
  ETH_MACCR_WD_Pos* = (23)
  ETH_MACCR_WD_Msk* = (0x00000001 shl ETH_MACCR_WD_Pos) ## !< 0x00800000
  ETH_MACCR_WD* = ETH_MACCR_WD_Msk
  ETH_MACCR_JD_Pos* = (22)
  ETH_MACCR_JD_Msk* = (0x00000001 shl ETH_MACCR_JD_Pos) ## !< 0x00400000
  ETH_MACCR_JD* = ETH_MACCR_JD_Msk
  ETH_MACCR_IFG_Pos* = (17)
  ETH_MACCR_IFG_Msk* = (0x00000007 shl ETH_MACCR_IFG_Pos) ## !< 0x000E0000
  ETH_MACCR_IFG* = ETH_MACCR_IFG_Msk
  ETH_MACCR_IFG_96Bit* = 0x00000000
  ETH_MACCR_IFG_88Bit* = 0x00020000
  ETH_MACCR_IFG_80Bit* = 0x00040000
  ETH_MACCR_IFG_72Bit* = 0x00060000
  ETH_MACCR_IFG_64Bit* = 0x00080000
  ETH_MACCR_IFG_56Bit* = 0x000A0000
  ETH_MACCR_IFG_48Bit* = 0x000C0000
  ETH_MACCR_IFG_40Bit* = 0x000E0000
  ETH_MACCR_CSD_Pos* = (16)
  ETH_MACCR_CSD_Msk* = (0x00000001 shl ETH_MACCR_CSD_Pos) ## !< 0x00010000
  ETH_MACCR_CSD* = ETH_MACCR_CSD_Msk
  ETH_MACCR_FES_Pos* = (14)
  ETH_MACCR_FES_Msk* = (0x00000001 shl ETH_MACCR_FES_Pos) ## !< 0x00004000
  ETH_MACCR_FES* = ETH_MACCR_FES_Msk
  ETH_MACCR_ROD_Pos* = (13)
  ETH_MACCR_ROD_Msk* = (0x00000001 shl ETH_MACCR_ROD_Pos) ## !< 0x00002000
  ETH_MACCR_ROD* = ETH_MACCR_ROD_Msk
  ETH_MACCR_LM_Pos* = (12)
  ETH_MACCR_LM_Msk* = (0x00000001 shl ETH_MACCR_LM_Pos) ## !< 0x00001000
  ETH_MACCR_LM* = ETH_MACCR_LM_Msk
  ETH_MACCR_DM_Pos* = (11)
  ETH_MACCR_DM_Msk* = (0x00000001 shl ETH_MACCR_DM_Pos) ## !< 0x00000800
  ETH_MACCR_DM* = ETH_MACCR_DM_Msk
  ETH_MACCR_IPCO_Pos* = (10)
  ETH_MACCR_IPCO_Msk* = (0x00000001 shl ETH_MACCR_IPCO_Pos) ## !< 0x00000400
  ETH_MACCR_IPCO* = ETH_MACCR_IPCO_Msk
  ETH_MACCR_RD_Pos* = (9)
  ETH_MACCR_RD_Msk* = (0x00000001 shl ETH_MACCR_RD_Pos) ## !< 0x00000200
  ETH_MACCR_RD* = ETH_MACCR_RD_Msk
  ETH_MACCR_APCS_Pos* = (7)
  ETH_MACCR_APCS_Msk* = (0x00000001 shl ETH_MACCR_APCS_Pos) ## !< 0x00000080
  ETH_MACCR_APCS* = ETH_MACCR_APCS_Msk
  ETH_MACCR_BL_Pos* = (5)
  ETH_MACCR_BL_Msk* = (0x00000003 shl ETH_MACCR_BL_Pos) ## !< 0x00000060
  ETH_MACCR_BL* = ETH_MACCR_BL_Msk
  ETH_MACCR_BL_10* = 0x00000000
  ETH_MACCR_BL_8* = 0x00000020
  ETH_MACCR_BL_4* = 0x00000040
  ETH_MACCR_BL_1* = 0x00000060
  ETH_MACCR_DC_Pos* = (4)
  ETH_MACCR_DC_Msk* = (0x00000001 shl ETH_MACCR_DC_Pos) ## !< 0x00000010
  ETH_MACCR_DC* = ETH_MACCR_DC_Msk
  ETH_MACCR_TE_Pos* = (3)
  ETH_MACCR_TE_Msk* = (0x00000001 shl ETH_MACCR_TE_Pos) ## !< 0x00000008
  ETH_MACCR_TE* = ETH_MACCR_TE_Msk
  ETH_MACCR_RE_Pos* = (2)
  ETH_MACCR_RE_Msk* = (0x00000001 shl ETH_MACCR_RE_Pos) ## !< 0x00000004
  ETH_MACCR_RE* = ETH_MACCR_RE_Msk

##  Bit definition for Ethernet MAC Frame Filter Register

const
  ETH_MACFFR_RA_Pos* = (31)
  ETH_MACFFR_RA_Msk* = (0x00000001 shl ETH_MACFFR_RA_Pos) ## !< 0x80000000
  ETH_MACFFR_RA* = ETH_MACFFR_RA_Msk
  ETH_MACFFR_HPF_Pos* = (10)
  ETH_MACFFR_HPF_Msk* = (0x00000001 shl ETH_MACFFR_HPF_Pos) ## !< 0x00000400
  ETH_MACFFR_HPF* = ETH_MACFFR_HPF_Msk
  ETH_MACFFR_SAF_Pos* = (9)
  ETH_MACFFR_SAF_Msk* = (0x00000001 shl ETH_MACFFR_SAF_Pos) ## !< 0x00000200
  ETH_MACFFR_SAF* = ETH_MACFFR_SAF_Msk
  ETH_MACFFR_SAIF_Pos* = (8)
  ETH_MACFFR_SAIF_Msk* = (0x00000001 shl ETH_MACFFR_SAIF_Pos) ## !< 0x00000100
  ETH_MACFFR_SAIF* = ETH_MACFFR_SAIF_Msk
  ETH_MACFFR_PCF_Pos* = (6)
  ETH_MACFFR_PCF_Msk* = (0x00000003 shl ETH_MACFFR_PCF_Pos) ## !< 0x000000C0
  ETH_MACFFR_PCF* = ETH_MACFFR_PCF_Msk
  ETH_MACFFR_PCF_BlockAll_Pos* = (6)
  ETH_MACFFR_PCF_BlockAll_Msk* = (0x00000001 shl ETH_MACFFR_PCF_BlockAll_Pos) ## !< 0x00000040
  ETH_MACFFR_PCF_BlockAll* = ETH_MACFFR_PCF_BlockAll_Msk
  ETH_MACFFR_PCF_ForwardAll_Pos* = (7)
  ETH_MACFFR_PCF_ForwardAll_Msk* = (0x00000001 shl ETH_MACFFR_PCF_ForwardAll_Pos) ## !< 0x00000080
  ETH_MACFFR_PCF_ForwardAll* = ETH_MACFFR_PCF_ForwardAll_Msk
  ETH_MACFFR_PCF_ForwardPassedAddrFilter_Pos* = (6)
  ETH_MACFFR_PCF_ForwardPassedAddrFilter_Msk* = (
    0x00000003 shl ETH_MACFFR_PCF_ForwardPassedAddrFilter_Pos) ## !< 0x000000C0
  ETH_MACFFR_PCF_ForwardPassedAddrFilter* = ETH_MACFFR_PCF_ForwardPassedAddrFilter_Msk
  ETH_MACFFR_BFD_Pos* = (5)
  ETH_MACFFR_BFD_Msk* = (0x00000001 shl ETH_MACFFR_BFD_Pos) ## !< 0x00000020
  ETH_MACFFR_BFD* = ETH_MACFFR_BFD_Msk
  ETH_MACFFR_PAM_Pos* = (4)
  ETH_MACFFR_PAM_Msk* = (0x00000001 shl ETH_MACFFR_PAM_Pos) ## !< 0x00000010
  ETH_MACFFR_PAM* = ETH_MACFFR_PAM_Msk
  ETH_MACFFR_DAIF_Pos* = (3)
  ETH_MACFFR_DAIF_Msk* = (0x00000001 shl ETH_MACFFR_DAIF_Pos) ## !< 0x00000008
  ETH_MACFFR_DAIF* = ETH_MACFFR_DAIF_Msk
  ETH_MACFFR_HM_Pos* = (2)
  ETH_MACFFR_HM_Msk* = (0x00000001 shl ETH_MACFFR_HM_Pos) ## !< 0x00000004
  ETH_MACFFR_HM* = ETH_MACFFR_HM_Msk
  ETH_MACFFR_HU_Pos* = (1)
  ETH_MACFFR_HU_Msk* = (0x00000001 shl ETH_MACFFR_HU_Pos) ## !< 0x00000002
  ETH_MACFFR_HU* = ETH_MACFFR_HU_Msk
  ETH_MACFFR_PM_Pos* = (0)
  ETH_MACFFR_PM_Msk* = (0x00000001 shl ETH_MACFFR_PM_Pos) ## !< 0x00000001
  ETH_MACFFR_PM* = ETH_MACFFR_PM_Msk

##  Bit definition for Ethernet MAC Hash Table High Register

const
  ETH_MACHTHR_HTH_Pos* = (0)
  ETH_MACHTHR_HTH_Msk* = (0xFFFFFFFF shl ETH_MACHTHR_HTH_Pos) ## !< 0xFFFFFFFF
  ETH_MACHTHR_HTH* = ETH_MACHTHR_HTH_Msk

##  Bit definition for Ethernet MAC Hash Table Low Register

const
  ETH_MACHTLR_HTL_Pos* = (0)
  ETH_MACHTLR_HTL_Msk* = (0xFFFFFFFF shl ETH_MACHTLR_HTL_Pos) ## !< 0xFFFFFFFF
  ETH_MACHTLR_HTL* = ETH_MACHTLR_HTL_Msk

##  Bit definition for Ethernet MAC MII Address Register

const
  ETH_MACMIIAR_PA_Pos* = (11)
  ETH_MACMIIAR_PA_Msk* = (0x0000001F shl ETH_MACMIIAR_PA_Pos) ## !< 0x0000F800
  ETH_MACMIIAR_PA* = ETH_MACMIIAR_PA_Msk
  ETH_MACMIIAR_MR_Pos* = (6)
  ETH_MACMIIAR_MR_Msk* = (0x0000001F shl ETH_MACMIIAR_MR_Pos) ## !< 0x000007C0
  ETH_MACMIIAR_MR* = ETH_MACMIIAR_MR_Msk
  ETH_MACMIIAR_CR_Pos* = (2)
  ETH_MACMIIAR_CR_Msk* = (0x00000007 shl ETH_MACMIIAR_CR_Pos) ## !< 0x0000001C
  ETH_MACMIIAR_CR* = ETH_MACMIIAR_CR_Msk
  ETH_MACMIIAR_CR_Div42* = 0x00000000
  ETH_MACMIIAR_CR_Div62_Pos* = (2)
  ETH_MACMIIAR_CR_Div62_Msk* = (0x00000001 shl ETH_MACMIIAR_CR_Div62_Pos) ## !< 0x00000004
  ETH_MACMIIAR_CR_Div62* = ETH_MACMIIAR_CR_Div62_Msk
  ETH_MACMIIAR_CR_Div16_Pos* = (3)
  ETH_MACMIIAR_CR_Div16_Msk* = (0x00000001 shl ETH_MACMIIAR_CR_Div16_Pos) ## !< 0x00000008
  ETH_MACMIIAR_CR_Div16* = ETH_MACMIIAR_CR_Div16_Msk
  ETH_MACMIIAR_CR_Div26_Pos* = (2)
  ETH_MACMIIAR_CR_Div26_Msk* = (0x00000003 shl ETH_MACMIIAR_CR_Div26_Pos) ## !< 0x0000000C
  ETH_MACMIIAR_CR_Div26* = ETH_MACMIIAR_CR_Div26_Msk
  ETH_MACMIIAR_CR_Div102_Pos* = (4)
  ETH_MACMIIAR_CR_Div102_Msk* = (0x00000001 shl ETH_MACMIIAR_CR_Div102_Pos) ## !< 0x00000010
  ETH_MACMIIAR_CR_Div102* = ETH_MACMIIAR_CR_Div102_Msk
  ETH_MACMIIAR_MW_Pos* = (1)
  ETH_MACMIIAR_MW_Msk* = (0x00000001 shl ETH_MACMIIAR_MW_Pos) ## !< 0x00000002
  ETH_MACMIIAR_MW* = ETH_MACMIIAR_MW_Msk
  ETH_MACMIIAR_MB_Pos* = (0)
  ETH_MACMIIAR_MB_Msk* = (0x00000001 shl ETH_MACMIIAR_MB_Pos) ## !< 0x00000001
  ETH_MACMIIAR_MB* = ETH_MACMIIAR_MB_Msk

##  Bit definition for Ethernet MAC MII Data Register

const
  ETH_MACMIIDR_MD_Pos* = (0)
  ETH_MACMIIDR_MD_Msk* = (0x0000FFFF shl ETH_MACMIIDR_MD_Pos) ## !< 0x0000FFFF
  ETH_MACMIIDR_MD* = ETH_MACMIIDR_MD_Msk

##  Bit definition for Ethernet MAC Flow Control Register

const
  ETH_MACFCR_PT_Pos* = (16)
  ETH_MACFCR_PT_Msk* = (0x0000FFFF shl ETH_MACFCR_PT_Pos) ## !< 0xFFFF0000
  ETH_MACFCR_PT* = ETH_MACFCR_PT_Msk
  ETH_MACFCR_ZQPD_Pos* = (7)
  ETH_MACFCR_ZQPD_Msk* = (0x00000001 shl ETH_MACFCR_ZQPD_Pos) ## !< 0x00000080
  ETH_MACFCR_ZQPD* = ETH_MACFCR_ZQPD_Msk
  ETH_MACFCR_PLT_Pos* = (4)
  ETH_MACFCR_PLT_Msk* = (0x00000003 shl ETH_MACFCR_PLT_Pos) ## !< 0x00000030
  ETH_MACFCR_PLT* = ETH_MACFCR_PLT_Msk
  ETH_MACFCR_PLT_Minus4* = 0x00000000
  ETH_MACFCR_PLT_Minus28_Pos* = (4)
  ETH_MACFCR_PLT_Minus28_Msk* = (0x00000001 shl ETH_MACFCR_PLT_Minus28_Pos) ## !< 0x00000010
  ETH_MACFCR_PLT_Minus28* = ETH_MACFCR_PLT_Minus28_Msk
  ETH_MACFCR_PLT_Minus144_Pos* = (5)
  ETH_MACFCR_PLT_Minus144_Msk* = (0x00000001 shl ETH_MACFCR_PLT_Minus144_Pos) ## !< 0x00000020
  ETH_MACFCR_PLT_Minus144* = ETH_MACFCR_PLT_Minus144_Msk
  ETH_MACFCR_PLT_Minus256_Pos* = (4)
  ETH_MACFCR_PLT_Minus256_Msk* = (0x00000003 shl ETH_MACFCR_PLT_Minus256_Pos) ## !< 0x00000030
  ETH_MACFCR_PLT_Minus256* = ETH_MACFCR_PLT_Minus256_Msk
  ETH_MACFCR_UPFD_Pos* = (3)
  ETH_MACFCR_UPFD_Msk* = (0x00000001 shl ETH_MACFCR_UPFD_Pos) ## !< 0x00000008
  ETH_MACFCR_UPFD* = ETH_MACFCR_UPFD_Msk
  ETH_MACFCR_RFCE_Pos* = (2)
  ETH_MACFCR_RFCE_Msk* = (0x00000001 shl ETH_MACFCR_RFCE_Pos) ## !< 0x00000004
  ETH_MACFCR_RFCE* = ETH_MACFCR_RFCE_Msk
  ETH_MACFCR_TFCE_Pos* = (1)
  ETH_MACFCR_TFCE_Msk* = (0x00000001 shl ETH_MACFCR_TFCE_Pos) ## !< 0x00000002
  ETH_MACFCR_TFCE* = ETH_MACFCR_TFCE_Msk
  ETH_MACFCR_FCBBPA_Pos* = (0)
  ETH_MACFCR_FCBBPA_Msk* = (0x00000001 shl ETH_MACFCR_FCBBPA_Pos) ## !< 0x00000001
  ETH_MACFCR_FCBBPA* = ETH_MACFCR_FCBBPA_Msk

##  Bit definition for Ethernet MAC VLAN Tag Register

const
  ETH_MACVLANTR_VLANTC_Pos* = (16)
  ETH_MACVLANTR_VLANTC_Msk* = (0x00000001 shl ETH_MACVLANTR_VLANTC_Pos) ## !< 0x00010000
  ETH_MACVLANTR_VLANTC* = ETH_MACVLANTR_VLANTC_Msk
  ETH_MACVLANTR_VLANTI_Pos* = (0)
  ETH_MACVLANTR_VLANTI_Msk* = (0x0000FFFF shl ETH_MACVLANTR_VLANTI_Pos) ## !< 0x0000FFFF
  ETH_MACVLANTR_VLANTI* = ETH_MACVLANTR_VLANTI_Msk

##  Bit definition for Ethernet MAC Remote Wake-UpFrame Filter Register

const
  ETH_MACRWUFFR_D_Pos* = (0)
  ETH_MACRWUFFR_D_Msk* = (0xFFFFFFFF shl ETH_MACRWUFFR_D_Pos) ## !< 0xFFFFFFFF
  ETH_MACRWUFFR_D* = ETH_MACRWUFFR_D_Msk

##  Eight sequential Writes to this address (offset 0x28) will write all Wake-UpFrame Filter Registers.
##    Eight sequential Reads from this address (offset 0x28) will read all Wake-UpFrame Filter Registers.
##  Wake-UpFrame Filter Reg0 : Filter 0 Byte Mask
##    Wake-UpFrame Filter Reg1 : Filter 1 Byte Mask
##    Wake-UpFrame Filter Reg2 : Filter 2 Byte Mask
##    Wake-UpFrame Filter Reg3 : Filter 3 Byte Mask
##    Wake-UpFrame Filter Reg4 : RSVD - Filter3 Command - RSVD - Filter2 Command -
##                               RSVD - Filter1 Command - RSVD - Filter0 Command
##    Wake-UpFrame Filter Re5 : Filter3 Offset - Filter2 Offset - Filter1 Offset - Filter0 Offset
##    Wake-UpFrame Filter Re6 : Filter1 CRC16 - Filter0 CRC16
##    Wake-UpFrame Filter Re7 : Filter3 CRC16 - Filter2 CRC16
##  Bit definition for Ethernet MAC PMT Control and Status Register

const
  ETH_MACPMTCSR_WFFRPR_Pos* = (31)
  ETH_MACPMTCSR_WFFRPR_Msk* = (0x00000001 shl ETH_MACPMTCSR_WFFRPR_Pos) ## !< 0x80000000
  ETH_MACPMTCSR_WFFRPR* = ETH_MACPMTCSR_WFFRPR_Msk
  ETH_MACPMTCSR_GU_Pos* = (9)
  ETH_MACPMTCSR_GU_Msk* = (0x00000001 shl ETH_MACPMTCSR_GU_Pos) ## !< 0x00000200
  ETH_MACPMTCSR_GU* = ETH_MACPMTCSR_GU_Msk
  ETH_MACPMTCSR_WFR_Pos* = (6)
  ETH_MACPMTCSR_WFR_Msk* = (0x00000001 shl ETH_MACPMTCSR_WFR_Pos) ## !< 0x00000040
  ETH_MACPMTCSR_WFR* = ETH_MACPMTCSR_WFR_Msk
  ETH_MACPMTCSR_MPR_Pos* = (5)
  ETH_MACPMTCSR_MPR_Msk* = (0x00000001 shl ETH_MACPMTCSR_MPR_Pos) ## !< 0x00000020
  ETH_MACPMTCSR_MPR* = ETH_MACPMTCSR_MPR_Msk
  ETH_MACPMTCSR_WFE_Pos* = (2)
  ETH_MACPMTCSR_WFE_Msk* = (0x00000001 shl ETH_MACPMTCSR_WFE_Pos) ## !< 0x00000004
  ETH_MACPMTCSR_WFE* = ETH_MACPMTCSR_WFE_Msk
  ETH_MACPMTCSR_MPE_Pos* = (1)
  ETH_MACPMTCSR_MPE_Msk* = (0x00000001 shl ETH_MACPMTCSR_MPE_Pos) ## !< 0x00000002
  ETH_MACPMTCSR_MPE* = ETH_MACPMTCSR_MPE_Msk
  ETH_MACPMTCSR_PD_Pos* = (0)
  ETH_MACPMTCSR_PD_Msk* = (0x00000001 shl ETH_MACPMTCSR_PD_Pos) ## !< 0x00000001
  ETH_MACPMTCSR_PD* = ETH_MACPMTCSR_PD_Msk

##  Bit definition for Ethernet MAC debug Register

const
  ETH_MACDBGR_TFF_Pos* = (25)
  ETH_MACDBGR_TFF_Msk* = (0x00000001 shl ETH_MACDBGR_TFF_Pos) ## !< 0x02000000
  ETH_MACDBGR_TFF* = ETH_MACDBGR_TFF_Msk
  ETH_MACDBGR_TFNE_Pos* = (24)
  ETH_MACDBGR_TFNE_Msk* = (0x00000001 shl ETH_MACDBGR_TFNE_Pos) ## !< 0x01000000
  ETH_MACDBGR_TFNE* = ETH_MACDBGR_TFNE_Msk
  ETH_MACDBGR_TFWA_Pos* = (22)
  ETH_MACDBGR_TFWA_Msk* = (0x00000001 shl ETH_MACDBGR_TFWA_Pos) ## !< 0x00400000
  ETH_MACDBGR_TFWA* = ETH_MACDBGR_TFWA_Msk
  ETH_MACDBGR_TFRS_Pos* = (20)
  ETH_MACDBGR_TFRS_Msk* = (0x00000003 shl ETH_MACDBGR_TFRS_Pos) ## !< 0x00300000
  ETH_MACDBGR_TFRS* = ETH_MACDBGR_TFRS_Msk
  ETH_MACDBGR_TFRS_WRITING_Pos* = (20)
  ETH_MACDBGR_TFRS_WRITING_Msk* = (0x00000003 shl ETH_MACDBGR_TFRS_WRITING_Pos) ## !< 0x00300000
  ETH_MACDBGR_TFRS_WRITING* = ETH_MACDBGR_TFRS_WRITING_Msk
  ETH_MACDBGR_TFRS_WAITING_Pos* = (21)
  ETH_MACDBGR_TFRS_WAITING_Msk* = (0x00000001 shl ETH_MACDBGR_TFRS_WAITING_Pos) ## !< 0x00200000
  ETH_MACDBGR_TFRS_WAITING* = ETH_MACDBGR_TFRS_WAITING_Msk
  ETH_MACDBGR_TFRS_READ_Pos* = (20)
  ETH_MACDBGR_TFRS_READ_Msk* = (0x00000001 shl ETH_MACDBGR_TFRS_READ_Pos) ## !< 0x00100000
  ETH_MACDBGR_TFRS_READ* = ETH_MACDBGR_TFRS_READ_Msk
  ETH_MACDBGR_TFRS_IDLE* = 0x00000000
  ETH_MACDBGR_MTP_Pos* = (19)
  ETH_MACDBGR_MTP_Msk* = (0x00000001 shl ETH_MACDBGR_MTP_Pos) ## !< 0x00080000
  ETH_MACDBGR_MTP* = ETH_MACDBGR_MTP_Msk
  ETH_MACDBGR_MTFCS_Pos* = (17)
  ETH_MACDBGR_MTFCS_Msk* = (0x00000003 shl ETH_MACDBGR_MTFCS_Pos) ## !< 0x00060000
  ETH_MACDBGR_MTFCS* = ETH_MACDBGR_MTFCS_Msk
  ETH_MACDBGR_MTFCS_TRANSFERRING_Pos* = (17)
  ETH_MACDBGR_MTFCS_TRANSFERRING_Msk* = (
    0x00000003 shl ETH_MACDBGR_MTFCS_TRANSFERRING_Pos) ## !< 0x00060000
  ETH_MACDBGR_MTFCS_TRANSFERRING* = ETH_MACDBGR_MTFCS_TRANSFERRING_Msk
  ETH_MACDBGR_MTFCS_GENERATINGPCF_Pos* = (18)
  ETH_MACDBGR_MTFCS_GENERATINGPCF_Msk* = (
    0x00000001 shl ETH_MACDBGR_MTFCS_GENERATINGPCF_Pos) ## !< 0x00040000
  ETH_MACDBGR_MTFCS_GENERATINGPCF* = ETH_MACDBGR_MTFCS_GENERATINGPCF_Msk
  ETH_MACDBGR_MTFCS_WAITING_Pos* = (17)
  ETH_MACDBGR_MTFCS_WAITING_Msk* = (0x00000001 shl ETH_MACDBGR_MTFCS_WAITING_Pos) ## !< 0x00020000
  ETH_MACDBGR_MTFCS_WAITING* = ETH_MACDBGR_MTFCS_WAITING_Msk
  ETH_MACDBGR_MTFCS_IDLE* = 0x00000000
  ETH_MACDBGR_MMTEA_Pos* = (16)
  ETH_MACDBGR_MMTEA_Msk* = (0x00000001 shl ETH_MACDBGR_MMTEA_Pos) ## !< 0x00010000
  ETH_MACDBGR_MMTEA* = ETH_MACDBGR_MMTEA_Msk
  ETH_MACDBGR_RFFL_Pos* = (8)
  ETH_MACDBGR_RFFL_Msk* = (0x00000003 shl ETH_MACDBGR_RFFL_Pos) ## !< 0x00000300
  ETH_MACDBGR_RFFL* = ETH_MACDBGR_RFFL_Msk
  ETH_MACDBGR_RFFL_FULL_Pos* = (8)
  ETH_MACDBGR_RFFL_FULL_Msk* = (0x00000003 shl ETH_MACDBGR_RFFL_FULL_Pos) ## !< 0x00000300
  ETH_MACDBGR_RFFL_FULL* = ETH_MACDBGR_RFFL_FULL_Msk
  ETH_MACDBGR_RFFL_ABOVEFCT_Pos* = (9)
  ETH_MACDBGR_RFFL_ABOVEFCT_Msk* = (0x00000001 shl ETH_MACDBGR_RFFL_ABOVEFCT_Pos) ## !< 0x00000200
  ETH_MACDBGR_RFFL_ABOVEFCT* = ETH_MACDBGR_RFFL_ABOVEFCT_Msk
  ETH_MACDBGR_RFFL_BELOWFCT_Pos* = (8)
  ETH_MACDBGR_RFFL_BELOWFCT_Msk* = (0x00000001 shl ETH_MACDBGR_RFFL_BELOWFCT_Pos) ## !< 0x00000100
  ETH_MACDBGR_RFFL_BELOWFCT* = ETH_MACDBGR_RFFL_BELOWFCT_Msk
  ETH_MACDBGR_RFFL_EMPTY* = 0x00000000
  ETH_MACDBGR_RFRCS_Pos* = (5)
  ETH_MACDBGR_RFRCS_Msk* = (0x00000003 shl ETH_MACDBGR_RFRCS_Pos) ## !< 0x00000060
  ETH_MACDBGR_RFRCS* = ETH_MACDBGR_RFRCS_Msk
  ETH_MACDBGR_RFRCS_FLUSHING_Pos* = (5)
  ETH_MACDBGR_RFRCS_FLUSHING_Msk* = (0x00000003 shl
      ETH_MACDBGR_RFRCS_FLUSHING_Pos) ## !< 0x00000060
  ETH_MACDBGR_RFRCS_FLUSHING* = ETH_MACDBGR_RFRCS_FLUSHING_Msk
  ETH_MACDBGR_RFRCS_STATUSREADING_Pos* = (6)
  ETH_MACDBGR_RFRCS_STATUSREADING_Msk* = (
    0x00000001 shl ETH_MACDBGR_RFRCS_STATUSREADING_Pos) ## !< 0x00000040
  ETH_MACDBGR_RFRCS_STATUSREADING* = ETH_MACDBGR_RFRCS_STATUSREADING_Msk
  ETH_MACDBGR_RFRCS_DATAREADING_Pos* = (5)
  ETH_MACDBGR_RFRCS_DATAREADING_Msk* = (
    0x00000001 shl ETH_MACDBGR_RFRCS_DATAREADING_Pos) ## !< 0x00000020
  ETH_MACDBGR_RFRCS_DATAREADING* = ETH_MACDBGR_RFRCS_DATAREADING_Msk
  ETH_MACDBGR_RFRCS_IDLE* = 0x00000000
  ETH_MACDBGR_RFWRA_Pos* = (4)
  ETH_MACDBGR_RFWRA_Msk* = (0x00000001 shl ETH_MACDBGR_RFWRA_Pos) ## !< 0x00000010
  ETH_MACDBGR_RFWRA* = ETH_MACDBGR_RFWRA_Msk
  ETH_MACDBGR_MSFRWCS_Pos* = (1)
  ETH_MACDBGR_MSFRWCS_Msk* = (0x00000003 shl ETH_MACDBGR_MSFRWCS_Pos) ## !< 0x00000006
  ETH_MACDBGR_MSFRWCS* = ETH_MACDBGR_MSFRWCS_Msk
  ETH_MACDBGR_MSFRWCS_1* = (0x00000002 shl ETH_MACDBGR_MSFRWCS_Pos) ## !< 0x00000004
  ETH_MACDBGR_MSFRWCS_0* = (0x00000001 shl ETH_MACDBGR_MSFRWCS_Pos) ## !< 0x00000002
  ETH_MACDBGR_MMRPEA_Pos* = (0)
  ETH_MACDBGR_MMRPEA_Msk* = (0x00000001 shl ETH_MACDBGR_MMRPEA_Pos) ## !< 0x00000001
  ETH_MACDBGR_MMRPEA* = ETH_MACDBGR_MMRPEA_Msk

##  Bit definition for Ethernet MAC Status Register

const
  ETH_MACSR_TSTS_Pos* = (9)
  ETH_MACSR_TSTS_Msk* = (0x00000001 shl ETH_MACSR_TSTS_Pos) ## !< 0x00000200
  ETH_MACSR_TSTS* = ETH_MACSR_TSTS_Msk
  ETH_MACSR_MMCTS_Pos* = (6)
  ETH_MACSR_MMCTS_Msk* = (0x00000001 shl ETH_MACSR_MMCTS_Pos) ## !< 0x00000040
  ETH_MACSR_MMCTS* = ETH_MACSR_MMCTS_Msk
  ETH_MACSR_MMMCRS_Pos* = (5)
  ETH_MACSR_MMMCRS_Msk* = (0x00000001 shl ETH_MACSR_MMMCRS_Pos) ## !< 0x00000020
  ETH_MACSR_MMMCRS* = ETH_MACSR_MMMCRS_Msk
  ETH_MACSR_MMCS_Pos* = (4)
  ETH_MACSR_MMCS_Msk* = (0x00000001 shl ETH_MACSR_MMCS_Pos) ## !< 0x00000010
  ETH_MACSR_MMCS* = ETH_MACSR_MMCS_Msk
  ETH_MACSR_PMTS_Pos* = (3)
  ETH_MACSR_PMTS_Msk* = (0x00000001 shl ETH_MACSR_PMTS_Pos) ## !< 0x00000008
  ETH_MACSR_PMTS* = ETH_MACSR_PMTS_Msk

##  Bit definition for Ethernet MAC Interrupt Mask Register

const
  ETH_MACIMR_TSTIM_Pos* = (9)
  ETH_MACIMR_TSTIM_Msk* = (0x00000001 shl ETH_MACIMR_TSTIM_Pos) ## !< 0x00000200
  ETH_MACIMR_TSTIM* = ETH_MACIMR_TSTIM_Msk
  ETH_MACIMR_PMTIM_Pos* = (3)
  ETH_MACIMR_PMTIM_Msk* = (0x00000001 shl ETH_MACIMR_PMTIM_Pos) ## !< 0x00000008
  ETH_MACIMR_PMTIM* = ETH_MACIMR_PMTIM_Msk

##  Bit definition for Ethernet MAC Address0 High Register

const
  ETH_MACA0HR_MACA0H_Pos* = (0)
  ETH_MACA0HR_MACA0H_Msk* = (0x0000FFFF shl ETH_MACA0HR_MACA0H_Pos) ## !< 0x0000FFFF
  ETH_MACA0HR_MACA0H* = ETH_MACA0HR_MACA0H_Msk

##  Bit definition for Ethernet MAC Address0 Low Register

const
  ETH_MACA0LR_MACA0L_Pos* = (0)
  ETH_MACA0LR_MACA0L_Msk* = (0xFFFFFFFF shl ETH_MACA0LR_MACA0L_Pos) ## !< 0xFFFFFFFF
  ETH_MACA0LR_MACA0L* = ETH_MACA0LR_MACA0L_Msk

##  Bit definition for Ethernet MAC Address1 High Register

const
  ETH_MACA1HR_AE_Pos* = (31)
  ETH_MACA1HR_AE_Msk* = (0x00000001 shl ETH_MACA1HR_AE_Pos) ## !< 0x80000000
  ETH_MACA1HR_AE* = ETH_MACA1HR_AE_Msk
  ETH_MACA1HR_SA_Pos* = (30)
  ETH_MACA1HR_SA_Msk* = (0x00000001 shl ETH_MACA1HR_SA_Pos) ## !< 0x40000000
  ETH_MACA1HR_SA* = ETH_MACA1HR_SA_Msk
  ETH_MACA1HR_MBC_Pos* = (24)
  ETH_MACA1HR_MBC_Msk* = (0x0000003F shl ETH_MACA1HR_MBC_Pos) ## !< 0x3F000000
  ETH_MACA1HR_MBC* = ETH_MACA1HR_MBC_Msk
  ETH_MACA1HR_MBC_HBits15_8* = 0x20000000
  ETH_MACA1HR_MBC_HBits7_0* = 0x10000000
  ETH_MACA1HR_MBC_LBits31_24* = 0x08000000
  ETH_MACA1HR_MBC_LBits23_16* = 0x04000000
  ETH_MACA1HR_MBC_LBits15_8* = 0x02000000
  ETH_MACA1HR_MBC_LBits7_0* = 0x01000000
  ETH_MACA1HR_MACA1H_Pos* = (0)
  ETH_MACA1HR_MACA1H_Msk* = (0x0000FFFF shl ETH_MACA1HR_MACA1H_Pos) ## !< 0x0000FFFF
  ETH_MACA1HR_MACA1H* = ETH_MACA1HR_MACA1H_Msk

##  Bit definition for Ethernet MAC Address1 Low Register

const
  ETH_MACA1LR_MACA1L_Pos* = (0)
  ETH_MACA1LR_MACA1L_Msk* = (0xFFFFFFFF shl ETH_MACA1LR_MACA1L_Pos) ## !< 0xFFFFFFFF
  ETH_MACA1LR_MACA1L* = ETH_MACA1LR_MACA1L_Msk

##  Bit definition for Ethernet MAC Address2 High Register

const
  ETH_MACA2HR_AE_Pos* = (31)
  ETH_MACA2HR_AE_Msk* = (0x00000001 shl ETH_MACA2HR_AE_Pos) ## !< 0x80000000
  ETH_MACA2HR_AE* = ETH_MACA2HR_AE_Msk
  ETH_MACA2HR_SA_Pos* = (30)
  ETH_MACA2HR_SA_Msk* = (0x00000001 shl ETH_MACA2HR_SA_Pos) ## !< 0x40000000
  ETH_MACA2HR_SA* = ETH_MACA2HR_SA_Msk
  ETH_MACA2HR_MBC_Pos* = (24)
  ETH_MACA2HR_MBC_Msk* = (0x0000003F shl ETH_MACA2HR_MBC_Pos) ## !< 0x3F000000
  ETH_MACA2HR_MBC* = ETH_MACA2HR_MBC_Msk
  ETH_MACA2HR_MBC_HBits15_8* = 0x20000000
  ETH_MACA2HR_MBC_HBits7_0* = 0x10000000
  ETH_MACA2HR_MBC_LBits31_24* = 0x08000000
  ETH_MACA2HR_MBC_LBits23_16* = 0x04000000
  ETH_MACA2HR_MBC_LBits15_8* = 0x02000000
  ETH_MACA2HR_MBC_LBits7_0* = 0x01000000
  ETH_MACA2HR_MACA2H_Pos* = (0)
  ETH_MACA2HR_MACA2H_Msk* = (0x0000FFFF shl ETH_MACA2HR_MACA2H_Pos) ## !< 0x0000FFFF
  ETH_MACA2HR_MACA2H* = ETH_MACA2HR_MACA2H_Msk

##  Bit definition for Ethernet MAC Address2 Low Register

const
  ETH_MACA2LR_MACA2L_Pos* = (0)
  ETH_MACA2LR_MACA2L_Msk* = (0xFFFFFFFF shl ETH_MACA2LR_MACA2L_Pos) ## !< 0xFFFFFFFF
  ETH_MACA2LR_MACA2L* = ETH_MACA2LR_MACA2L_Msk

##  Bit definition for Ethernet MAC Address3 High Register

const
  ETH_MACA3HR_AE_Pos* = (31)
  ETH_MACA3HR_AE_Msk* = (0x00000001 shl ETH_MACA3HR_AE_Pos) ## !< 0x80000000
  ETH_MACA3HR_AE* = ETH_MACA3HR_AE_Msk
  ETH_MACA3HR_SA_Pos* = (30)
  ETH_MACA3HR_SA_Msk* = (0x00000001 shl ETH_MACA3HR_SA_Pos) ## !< 0x40000000
  ETH_MACA3HR_SA* = ETH_MACA3HR_SA_Msk
  ETH_MACA3HR_MBC_Pos* = (24)
  ETH_MACA3HR_MBC_Msk* = (0x0000003F shl ETH_MACA3HR_MBC_Pos) ## !< 0x3F000000
  ETH_MACA3HR_MBC* = ETH_MACA3HR_MBC_Msk
  ETH_MACA3HR_MBC_HBits15_8* = 0x20000000
  ETH_MACA3HR_MBC_HBits7_0* = 0x10000000
  ETH_MACA3HR_MBC_LBits31_24* = 0x08000000
  ETH_MACA3HR_MBC_LBits23_16* = 0x04000000
  ETH_MACA3HR_MBC_LBits15_8* = 0x02000000
  ETH_MACA3HR_MBC_LBits7_0* = 0x01000000
  ETH_MACA3HR_MACA3H_Pos* = (0)
  ETH_MACA3HR_MACA3H_Msk* = (0x0000FFFF shl ETH_MACA3HR_MACA3H_Pos) ## !< 0x0000FFFF
  ETH_MACA3HR_MACA3H* = ETH_MACA3HR_MACA3H_Msk

##  Bit definition for Ethernet MAC Address3 Low Register

const
  ETH_MACA3LR_MACA3L_Pos* = (0)
  ETH_MACA3LR_MACA3L_Msk* = (0xFFFFFFFF shl ETH_MACA3LR_MACA3L_Pos) ## !< 0xFFFFFFFF
  ETH_MACA3LR_MACA3L* = ETH_MACA3LR_MACA3L_Msk

## ****************************************************************************
##                 Ethernet MMC Registers bits definition
## ****************************************************************************
##  Bit definition for Ethernet MMC Contol Register

const
  ETH_MMCCR_MCFHP_Pos* = (5)
  ETH_MMCCR_MCFHP_Msk* = (0x00000001 shl ETH_MMCCR_MCFHP_Pos) ## !< 0x00000020
  ETH_MMCCR_MCFHP* = ETH_MMCCR_MCFHP_Msk
  ETH_MMCCR_MCP_Pos* = (4)
  ETH_MMCCR_MCP_Msk* = (0x00000001 shl ETH_MMCCR_MCP_Pos) ## !< 0x00000010
  ETH_MMCCR_MCP* = ETH_MMCCR_MCP_Msk
  ETH_MMCCR_MCF_Pos* = (3)
  ETH_MMCCR_MCF_Msk* = (0x00000001 shl ETH_MMCCR_MCF_Pos) ## !< 0x00000008
  ETH_MMCCR_MCF* = ETH_MMCCR_MCF_Msk
  ETH_MMCCR_ROR_Pos* = (2)
  ETH_MMCCR_ROR_Msk* = (0x00000001 shl ETH_MMCCR_ROR_Pos) ## !< 0x00000004
  ETH_MMCCR_ROR* = ETH_MMCCR_ROR_Msk
  ETH_MMCCR_CSR_Pos* = (1)
  ETH_MMCCR_CSR_Msk* = (0x00000001 shl ETH_MMCCR_CSR_Pos) ## !< 0x00000002
  ETH_MMCCR_CSR* = ETH_MMCCR_CSR_Msk
  ETH_MMCCR_CR_Pos* = (0)
  ETH_MMCCR_CR_Msk* = (0x00000001 shl ETH_MMCCR_CR_Pos) ## !< 0x00000001
  ETH_MMCCR_CR* = ETH_MMCCR_CR_Msk

##  Bit definition for Ethernet MMC Receive Interrupt Register

const
  ETH_MMCRIR_RGUFS_Pos* = (17)
  ETH_MMCRIR_RGUFS_Msk* = (0x00000001 shl ETH_MMCRIR_RGUFS_Pos) ## !< 0x00020000
  ETH_MMCRIR_RGUFS* = ETH_MMCRIR_RGUFS_Msk
  ETH_MMCRIR_RFAES_Pos* = (6)
  ETH_MMCRIR_RFAES_Msk* = (0x00000001 shl ETH_MMCRIR_RFAES_Pos) ## !< 0x00000040
  ETH_MMCRIR_RFAES* = ETH_MMCRIR_RFAES_Msk
  ETH_MMCRIR_RFCES_Pos* = (5)
  ETH_MMCRIR_RFCES_Msk* = (0x00000001 shl ETH_MMCRIR_RFCES_Pos) ## !< 0x00000020
  ETH_MMCRIR_RFCES* = ETH_MMCRIR_RFCES_Msk

##  Bit definition for Ethernet MMC Transmit Interrupt Register

const
  ETH_MMCTIR_TGFS_Pos* = (21)
  ETH_MMCTIR_TGFS_Msk* = (0x00000001 shl ETH_MMCTIR_TGFS_Pos) ## !< 0x00200000
  ETH_MMCTIR_TGFS* = ETH_MMCTIR_TGFS_Msk
  ETH_MMCTIR_TGFMSCS_Pos* = (15)
  ETH_MMCTIR_TGFMSCS_Msk* = (0x00000001 shl ETH_MMCTIR_TGFMSCS_Pos) ## !< 0x00008000
  ETH_MMCTIR_TGFMSCS* = ETH_MMCTIR_TGFMSCS_Msk
  ETH_MMCTIR_TGFSCS_Pos* = (14)
  ETH_MMCTIR_TGFSCS_Msk* = (0x00000001 shl ETH_MMCTIR_TGFSCS_Pos) ## !< 0x00004000
  ETH_MMCTIR_TGFSCS* = ETH_MMCTIR_TGFSCS_Msk

##  Bit definition for Ethernet MMC Receive Interrupt Mask Register

const
  ETH_MMCRIMR_RGUFM_Pos* = (17)
  ETH_MMCRIMR_RGUFM_Msk* = (0x00000001 shl ETH_MMCRIMR_RGUFM_Pos) ## !< 0x00020000
  ETH_MMCRIMR_RGUFM* = ETH_MMCRIMR_RGUFM_Msk
  ETH_MMCRIMR_RFAEM_Pos* = (6)
  ETH_MMCRIMR_RFAEM_Msk* = (0x00000001 shl ETH_MMCRIMR_RFAEM_Pos) ## !< 0x00000040
  ETH_MMCRIMR_RFAEM* = ETH_MMCRIMR_RFAEM_Msk
  ETH_MMCRIMR_RFCEM_Pos* = (5)
  ETH_MMCRIMR_RFCEM_Msk* = (0x00000001 shl ETH_MMCRIMR_RFCEM_Pos) ## !< 0x00000020
  ETH_MMCRIMR_RFCEM* = ETH_MMCRIMR_RFCEM_Msk

##  Bit definition for Ethernet MMC Transmit Interrupt Mask Register

const
  ETH_MMCTIMR_TGFM_Pos* = (21)
  ETH_MMCTIMR_TGFM_Msk* = (0x00000001 shl ETH_MMCTIMR_TGFM_Pos) ## !< 0x00200000
  ETH_MMCTIMR_TGFM* = ETH_MMCTIMR_TGFM_Msk
  ETH_MMCTIMR_TGFMSCM_Pos* = (15)
  ETH_MMCTIMR_TGFMSCM_Msk* = (0x00000001 shl ETH_MMCTIMR_TGFMSCM_Pos) ## !< 0x00008000
  ETH_MMCTIMR_TGFMSCM* = ETH_MMCTIMR_TGFMSCM_Msk
  ETH_MMCTIMR_TGFSCM_Pos* = (14)
  ETH_MMCTIMR_TGFSCM_Msk* = (0x00000001 shl ETH_MMCTIMR_TGFSCM_Pos) ## !< 0x00004000
  ETH_MMCTIMR_TGFSCM* = ETH_MMCTIMR_TGFSCM_Msk

##  Bit definition for Ethernet MMC Transmitted Good Frames after Single Collision Counter Register

const
  ETH_MMCTGFSCCR_TGFSCC_Pos* = (0)
  ETH_MMCTGFSCCR_TGFSCC_Msk* = (0xFFFFFFFF shl ETH_MMCTGFSCCR_TGFSCC_Pos) ## !< 0xFFFFFFFF
  ETH_MMCTGFSCCR_TGFSCC* = ETH_MMCTGFSCCR_TGFSCC_Msk

##  Bit definition for Ethernet MMC Transmitted Good Frames after More than a Single Collision Counter Register

const
  ETH_MMCTGFMSCCR_TGFMSCC_Pos* = (0)
  ETH_MMCTGFMSCCR_TGFMSCC_Msk* = (0xFFFFFFFF shl ETH_MMCTGFMSCCR_TGFMSCC_Pos) ## !< 0xFFFFFFFF
  ETH_MMCTGFMSCCR_TGFMSCC* = ETH_MMCTGFMSCCR_TGFMSCC_Msk

##  Bit definition for Ethernet MMC Transmitted Good Frames Counter Register

const
  ETH_MMCTGFCR_TGFC_Pos* = (0)
  ETH_MMCTGFCR_TGFC_Msk* = (0xFFFFFFFF shl ETH_MMCTGFCR_TGFC_Pos) ## !< 0xFFFFFFFF
  ETH_MMCTGFCR_TGFC* = ETH_MMCTGFCR_TGFC_Msk

##  Bit definition for Ethernet MMC Received Frames with CRC Error Counter Register

const
  ETH_MMCRFCECR_RFCEC_Pos* = (0)
  ETH_MMCRFCECR_RFCEC_Msk* = (0xFFFFFFFF shl ETH_MMCRFCECR_RFCEC_Pos) ## !< 0xFFFFFFFF
  ETH_MMCRFCECR_RFCEC* = ETH_MMCRFCECR_RFCEC_Msk

##  Bit definition for Ethernet MMC Received Frames with Alignement Error Counter Register

const
  ETH_MMCRFAECR_RFAEC_Pos* = (0)
  ETH_MMCRFAECR_RFAEC_Msk* = (0xFFFFFFFF shl ETH_MMCRFAECR_RFAEC_Pos) ## !< 0xFFFFFFFF
  ETH_MMCRFAECR_RFAEC* = ETH_MMCRFAECR_RFAEC_Msk

##  Bit definition for Ethernet MMC Received Good Unicast Frames Counter Register

const
  ETH_MMCRGUFCR_RGUFC_Pos* = (0)
  ETH_MMCRGUFCR_RGUFC_Msk* = (0xFFFFFFFF shl ETH_MMCRGUFCR_RGUFC_Pos) ## !< 0xFFFFFFFF
  ETH_MMCRGUFCR_RGUFC* = ETH_MMCRGUFCR_RGUFC_Msk

## ****************************************************************************
##                Ethernet PTP Registers bits definition
## ****************************************************************************
##  Bit definition for Ethernet PTP Time Stamp Contol Register

const
  ETH_PTPTSCR_TSCNT_Pos* = (16)
  ETH_PTPTSCR_TSCNT_Msk* = (0x00000003 shl ETH_PTPTSCR_TSCNT_Pos) ## !< 0x00030000
  ETH_PTPTSCR_TSCNT* = ETH_PTPTSCR_TSCNT_Msk
  ETH_PTPTSSR_TSSMRME_Pos* = (15)
  ETH_PTPTSSR_TSSMRME_Msk* = (0x00000001 shl ETH_PTPTSSR_TSSMRME_Pos) ## !< 0x00008000
  ETH_PTPTSSR_TSSMRME* = ETH_PTPTSSR_TSSMRME_Msk
  ETH_PTPTSSR_TSSEME_Pos* = (14)
  ETH_PTPTSSR_TSSEME_Msk* = (0x00000001 shl ETH_PTPTSSR_TSSEME_Pos) ## !< 0x00004000
  ETH_PTPTSSR_TSSEME* = ETH_PTPTSSR_TSSEME_Msk
  ETH_PTPTSSR_TSSIPV4FE_Pos* = (13)
  ETH_PTPTSSR_TSSIPV4FE_Msk* = (0x00000001 shl ETH_PTPTSSR_TSSIPV4FE_Pos) ## !< 0x00002000
  ETH_PTPTSSR_TSSIPV4FE* = ETH_PTPTSSR_TSSIPV4FE_Msk
  ETH_PTPTSSR_TSSIPV6FE_Pos* = (12)
  ETH_PTPTSSR_TSSIPV6FE_Msk* = (0x00000001 shl ETH_PTPTSSR_TSSIPV6FE_Pos) ## !< 0x00001000
  ETH_PTPTSSR_TSSIPV6FE* = ETH_PTPTSSR_TSSIPV6FE_Msk
  ETH_PTPTSSR_TSSPTPOEFE_Pos* = (11)
  ETH_PTPTSSR_TSSPTPOEFE_Msk* = (0x00000001 shl ETH_PTPTSSR_TSSPTPOEFE_Pos) ## !< 0x00000800
  ETH_PTPTSSR_TSSPTPOEFE* = ETH_PTPTSSR_TSSPTPOEFE_Msk
  ETH_PTPTSSR_TSPTPPSV2E_Pos* = (10)
  ETH_PTPTSSR_TSPTPPSV2E_Msk* = (0x00000001 shl ETH_PTPTSSR_TSPTPPSV2E_Pos) ## !< 0x00000400
  ETH_PTPTSSR_TSPTPPSV2E* = ETH_PTPTSSR_TSPTPPSV2E_Msk
  ETH_PTPTSSR_TSSSR_Pos* = (9)
  ETH_PTPTSSR_TSSSR_Msk* = (0x00000001 shl ETH_PTPTSSR_TSSSR_Pos) ## !< 0x00000200
  ETH_PTPTSSR_TSSSR* = ETH_PTPTSSR_TSSSR_Msk
  ETH_PTPTSSR_TSSARFE_Pos* = (8)
  ETH_PTPTSSR_TSSARFE_Msk* = (0x00000001 shl ETH_PTPTSSR_TSSARFE_Pos) ## !< 0x00000100
  ETH_PTPTSSR_TSSARFE* = ETH_PTPTSSR_TSSARFE_Msk
  ETH_PTPTSCR_TSARU_Pos* = (5)
  ETH_PTPTSCR_TSARU_Msk* = (0x00000001 shl ETH_PTPTSCR_TSARU_Pos) ## !< 0x00000020
  ETH_PTPTSCR_TSARU* = ETH_PTPTSCR_TSARU_Msk
  ETH_PTPTSCR_TSITE_Pos* = (4)
  ETH_PTPTSCR_TSITE_Msk* = (0x00000001 shl ETH_PTPTSCR_TSITE_Pos) ## !< 0x00000010
  ETH_PTPTSCR_TSITE* = ETH_PTPTSCR_TSITE_Msk
  ETH_PTPTSCR_TSSTU_Pos* = (3)
  ETH_PTPTSCR_TSSTU_Msk* = (0x00000001 shl ETH_PTPTSCR_TSSTU_Pos) ## !< 0x00000008
  ETH_PTPTSCR_TSSTU* = ETH_PTPTSCR_TSSTU_Msk
  ETH_PTPTSCR_TSSTI_Pos* = (2)
  ETH_PTPTSCR_TSSTI_Msk* = (0x00000001 shl ETH_PTPTSCR_TSSTI_Pos) ## !< 0x00000004
  ETH_PTPTSCR_TSSTI* = ETH_PTPTSCR_TSSTI_Msk
  ETH_PTPTSCR_TSFCU_Pos* = (1)
  ETH_PTPTSCR_TSFCU_Msk* = (0x00000001 shl ETH_PTPTSCR_TSFCU_Pos) ## !< 0x00000002
  ETH_PTPTSCR_TSFCU* = ETH_PTPTSCR_TSFCU_Msk
  ETH_PTPTSCR_TSE_Pos* = (0)
  ETH_PTPTSCR_TSE_Msk* = (0x00000001 shl ETH_PTPTSCR_TSE_Pos) ## !< 0x00000001
  ETH_PTPTSCR_TSE* = ETH_PTPTSCR_TSE_Msk

##  Bit definition for Ethernet PTP Sub-Second Increment Register

const
  ETH_PTPSSIR_STSSI_Pos* = (0)
  ETH_PTPSSIR_STSSI_Msk* = (0x000000FF shl ETH_PTPSSIR_STSSI_Pos) ## !< 0x000000FF
  ETH_PTPSSIR_STSSI* = ETH_PTPSSIR_STSSI_Msk

##  Bit definition for Ethernet PTP Time Stamp High Register

const
  ETH_PTPTSHR_STS_Pos* = (0)
  ETH_PTPTSHR_STS_Msk* = (0xFFFFFFFF shl ETH_PTPTSHR_STS_Pos) ## !< 0xFFFFFFFF
  ETH_PTPTSHR_STS* = ETH_PTPTSHR_STS_Msk

##  Bit definition for Ethernet PTP Time Stamp Low Register

const
  ETH_PTPTSLR_STPNS_Pos* = (31)
  ETH_PTPTSLR_STPNS_Msk* = (0x00000001 shl ETH_PTPTSLR_STPNS_Pos) ## !< 0x80000000
  ETH_PTPTSLR_STPNS* = ETH_PTPTSLR_STPNS_Msk
  ETH_PTPTSLR_STSS_Pos* = (0)
  ETH_PTPTSLR_STSS_Msk* = (0x7FFFFFFF shl ETH_PTPTSLR_STSS_Pos) ## !< 0x7FFFFFFF
  ETH_PTPTSLR_STSS* = ETH_PTPTSLR_STSS_Msk

##  Bit definition for Ethernet PTP Time Stamp High Update Register

const
  ETH_PTPTSHUR_TSUS_Pos* = (0)
  ETH_PTPTSHUR_TSUS_Msk* = (0xFFFFFFFF shl ETH_PTPTSHUR_TSUS_Pos) ## !< 0xFFFFFFFF
  ETH_PTPTSHUR_TSUS* = ETH_PTPTSHUR_TSUS_Msk

##  Bit definition for Ethernet PTP Time Stamp Low Update Register

const
  ETH_PTPTSLUR_TSUPNS_Pos* = (31)
  ETH_PTPTSLUR_TSUPNS_Msk* = (0x00000001 shl ETH_PTPTSLUR_TSUPNS_Pos) ## !< 0x80000000
  ETH_PTPTSLUR_TSUPNS* = ETH_PTPTSLUR_TSUPNS_Msk
  ETH_PTPTSLUR_TSUSS_Pos* = (0)
  ETH_PTPTSLUR_TSUSS_Msk* = (0x7FFFFFFF shl ETH_PTPTSLUR_TSUSS_Pos) ## !< 0x7FFFFFFF
  ETH_PTPTSLUR_TSUSS* = ETH_PTPTSLUR_TSUSS_Msk

##  Bit definition for Ethernet PTP Time Stamp Addend Register

const
  ETH_PTPTSAR_TSA_Pos* = (0)
  ETH_PTPTSAR_TSA_Msk* = (0xFFFFFFFF shl ETH_PTPTSAR_TSA_Pos) ## !< 0xFFFFFFFF
  ETH_PTPTSAR_TSA* = ETH_PTPTSAR_TSA_Msk

##  Bit definition for Ethernet PTP Target Time High Register

const
  ETH_PTPTTHR_TTSH_Pos* = (0)
  ETH_PTPTTHR_TTSH_Msk* = (0xFFFFFFFF shl ETH_PTPTTHR_TTSH_Pos) ## !< 0xFFFFFFFF
  ETH_PTPTTHR_TTSH* = ETH_PTPTTHR_TTSH_Msk

##  Bit definition for Ethernet PTP Target Time Low Register

const
  ETH_PTPTTLR_TTSL_Pos* = (0)
  ETH_PTPTTLR_TTSL_Msk* = (0xFFFFFFFF shl ETH_PTPTTLR_TTSL_Pos) ## !< 0xFFFFFFFF
  ETH_PTPTTLR_TTSL* = ETH_PTPTTLR_TTSL_Msk

##  Bit definition for Ethernet PTP Time Stamp Status Register

const
  ETH_PTPTSSR_TSTTR_Pos* = (5)
  ETH_PTPTSSR_TSTTR_Msk* = (0x00000001 shl ETH_PTPTSSR_TSTTR_Pos) ## !< 0x00000020
  ETH_PTPTSSR_TSTTR* = ETH_PTPTSSR_TSTTR_Msk
  ETH_PTPTSSR_TSSO_Pos* = (4)
  ETH_PTPTSSR_TSSO_Msk* = (0x00000001 shl ETH_PTPTSSR_TSSO_Pos) ## !< 0x00000010
  ETH_PTPTSSR_TSSO* = ETH_PTPTSSR_TSSO_Msk

## ****************************************************************************
##                  Ethernet DMA Registers bits definition
## ****************************************************************************
##  Bit definition for Ethernet DMA Bus Mode Register

const
  ETH_DMABMR_AAB_Pos* = (25)
  ETH_DMABMR_AAB_Msk* = (0x00000001 shl ETH_DMABMR_AAB_Pos) ## !< 0x02000000
  ETH_DMABMR_AAB* = ETH_DMABMR_AAB_Msk
  ETH_DMABMR_FPM_Pos* = (24)
  ETH_DMABMR_FPM_Msk* = (0x00000001 shl ETH_DMABMR_FPM_Pos) ## !< 0x01000000
  ETH_DMABMR_FPM* = ETH_DMABMR_FPM_Msk
  ETH_DMABMR_USP_Pos* = (23)
  ETH_DMABMR_USP_Msk* = (0x00000001 shl ETH_DMABMR_USP_Pos) ## !< 0x00800000
  ETH_DMABMR_USP* = ETH_DMABMR_USP_Msk
  ETH_DMABMR_RDP_Pos* = (17)
  ETH_DMABMR_RDP_Msk* = (0x0000003F shl ETH_DMABMR_RDP_Pos) ## !< 0x007E0000
  ETH_DMABMR_RDP* = ETH_DMABMR_RDP_Msk
  ETH_DMABMR_RDP_1Beat* = 0x00020000
  ETH_DMABMR_RDP_2Beat* = 0x00040000
  ETH_DMABMR_RDP_4Beat* = 0x00080000
  ETH_DMABMR_RDP_8Beat* = 0x00100000
  ETH_DMABMR_RDP_16Beat* = 0x00200000
  ETH_DMABMR_RDP_32Beat* = 0x00400000
  ETH_DMABMR_RDP_4xPBL_4Beat* = 0x01020000
  ETH_DMABMR_RDP_4xPBL_8Beat* = 0x01040000
  ETH_DMABMR_RDP_4xPBL_16Beat* = 0x01080000
  ETH_DMABMR_RDP_4xPBL_32Beat* = 0x01100000
  ETH_DMABMR_RDP_4xPBL_64Beat* = 0x01200000
  ETH_DMABMR_RDP_4xPBL_128Beat* = 0x01400000
  ETH_DMABMR_FB_Pos* = (16)
  ETH_DMABMR_FB_Msk* = (0x00000001 shl ETH_DMABMR_FB_Pos) ## !< 0x00010000
  ETH_DMABMR_FB* = ETH_DMABMR_FB_Msk
  ETH_DMABMR_RTPR_Pos* = (14)
  ETH_DMABMR_RTPR_Msk* = (0x00000003 shl ETH_DMABMR_RTPR_Pos) ## !< 0x0000C000
  ETH_DMABMR_RTPR* = ETH_DMABMR_RTPR_Msk
  ETH_DMABMR_RTPR_1_1* = 0x00000000
  ETH_DMABMR_RTPR_2_1* = 0x00004000
  ETH_DMABMR_RTPR_3_1* = 0x00008000
  ETH_DMABMR_RTPR_4_1* = 0x0000C000
  ETH_DMABMR_PBL_Pos* = (8)
  ETH_DMABMR_PBL_Msk* = (0x0000003F shl ETH_DMABMR_PBL_Pos) ## !< 0x00003F00
  ETH_DMABMR_PBL* = ETH_DMABMR_PBL_Msk
  ETH_DMABMR_PBL_1Beat* = 0x00000100
  ETH_DMABMR_PBL_2Beat* = 0x00000200
  ETH_DMABMR_PBL_4Beat* = 0x00000400
  ETH_DMABMR_PBL_8Beat* = 0x00000800
  ETH_DMABMR_PBL_16Beat* = 0x00001000
  ETH_DMABMR_PBL_32Beat* = 0x00002000
  ETH_DMABMR_PBL_4xPBL_4Beat* = 0x01000100
  ETH_DMABMR_PBL_4xPBL_8Beat* = 0x01000200
  ETH_DMABMR_PBL_4xPBL_16Beat* = 0x01000400
  ETH_DMABMR_PBL_4xPBL_32Beat* = 0x01000800
  ETH_DMABMR_PBL_4xPBL_64Beat* = 0x01001000
  ETH_DMABMR_PBL_4xPBL_128Beat* = 0x01002000
  ETH_DMABMR_EDE_Pos* = (7)
  ETH_DMABMR_EDE_Msk* = (0x00000001 shl ETH_DMABMR_EDE_Pos) ## !< 0x00000080
  ETH_DMABMR_EDE* = ETH_DMABMR_EDE_Msk
  ETH_DMABMR_DSL_Pos* = (2)
  ETH_DMABMR_DSL_Msk* = (0x0000001F shl ETH_DMABMR_DSL_Pos) ## !< 0x0000007C
  ETH_DMABMR_DSL* = ETH_DMABMR_DSL_Msk
  ETH_DMABMR_DA_Pos* = (1)
  ETH_DMABMR_DA_Msk* = (0x00000001 shl ETH_DMABMR_DA_Pos) ## !< 0x00000002
  ETH_DMABMR_DA* = ETH_DMABMR_DA_Msk
  ETH_DMABMR_SR_Pos* = (0)
  ETH_DMABMR_SR_Msk* = (0x00000001 shl ETH_DMABMR_SR_Pos) ## !< 0x00000001
  ETH_DMABMR_SR* = ETH_DMABMR_SR_Msk

##  Bit definition for Ethernet DMA Transmit Poll Demand Register

const
  ETH_DMATPDR_TPD_Pos* = (0)
  ETH_DMATPDR_TPD_Msk* = (0xFFFFFFFF shl ETH_DMATPDR_TPD_Pos) ## !< 0xFFFFFFFF
  ETH_DMATPDR_TPD* = ETH_DMATPDR_TPD_Msk

##  Bit definition for Ethernet DMA Receive Poll Demand Register

const
  ETH_DMARPDR_RPD_Pos* = (0)
  ETH_DMARPDR_RPD_Msk* = (0xFFFFFFFF shl ETH_DMARPDR_RPD_Pos) ## !< 0xFFFFFFFF
  ETH_DMARPDR_RPD* = ETH_DMARPDR_RPD_Msk

##  Bit definition for Ethernet DMA Receive Descriptor List Address Register

const
  ETH_DMARDLAR_SRL_Pos* = (0)
  ETH_DMARDLAR_SRL_Msk* = (0xFFFFFFFF shl ETH_DMARDLAR_SRL_Pos) ## !< 0xFFFFFFFF
  ETH_DMARDLAR_SRL* = ETH_DMARDLAR_SRL_Msk

##  Bit definition for Ethernet DMA Transmit Descriptor List Address Register

const
  ETH_DMATDLAR_STL_Pos* = (0)
  ETH_DMATDLAR_STL_Msk* = (0xFFFFFFFF shl ETH_DMATDLAR_STL_Pos) ## !< 0xFFFFFFFF
  ETH_DMATDLAR_STL* = ETH_DMATDLAR_STL_Msk

##  Bit definition for Ethernet DMA Status Register

const
  ETH_DMASR_TSTS_Pos* = (29)
  ETH_DMASR_TSTS_Msk* = (0x00000001 shl ETH_DMASR_TSTS_Pos) ## !< 0x20000000
  ETH_DMASR_TSTS* = ETH_DMASR_TSTS_Msk
  ETH_DMASR_PMTS_Pos* = (28)
  ETH_DMASR_PMTS_Msk* = (0x00000001 shl ETH_DMASR_PMTS_Pos) ## !< 0x10000000
  ETH_DMASR_PMTS* = ETH_DMASR_PMTS_Msk
  ETH_DMASR_MMCS_Pos* = (27)
  ETH_DMASR_MMCS_Msk* = (0x00000001 shl ETH_DMASR_MMCS_Pos) ## !< 0x08000000
  ETH_DMASR_MMCS* = ETH_DMASR_MMCS_Msk
  ETH_DMASR_EBS_Pos* = (23)
  ETH_DMASR_EBS_Msk* = (0x00000007 shl ETH_DMASR_EBS_Pos) ## !< 0x03800000
  ETH_DMASR_EBS* = ETH_DMASR_EBS_Msk

##  combination with EBS[2:0] for GetFlagStatus function

const
  ETH_DMASR_EBS_DescAccess_Pos* = (25)
  ETH_DMASR_EBS_DescAccess_Msk* = (0x00000001 shl ETH_DMASR_EBS_DescAccess_Pos) ## !< 0x02000000
  ETH_DMASR_EBS_DescAccess* = ETH_DMASR_EBS_DescAccess_Msk
  ETH_DMASR_EBS_ReadTransf_Pos* = (24)
  ETH_DMASR_EBS_ReadTransf_Msk* = (0x00000001 shl ETH_DMASR_EBS_ReadTransf_Pos) ## !< 0x01000000
  ETH_DMASR_EBS_ReadTransf* = ETH_DMASR_EBS_ReadTransf_Msk
  ETH_DMASR_EBS_DataTransfTx_Pos* = (23)
  ETH_DMASR_EBS_DataTransfTx_Msk* = (0x00000001 shl
      ETH_DMASR_EBS_DataTransfTx_Pos) ## !< 0x00800000
  ETH_DMASR_EBS_DataTransfTx* = ETH_DMASR_EBS_DataTransfTx_Msk
  ETH_DMASR_TPS_Pos* = (20)
  ETH_DMASR_TPS_Msk* = (0x00000007 shl ETH_DMASR_TPS_Pos) ## !< 0x00700000
  ETH_DMASR_TPS* = ETH_DMASR_TPS_Msk
  ETH_DMASR_TPS_Stopped* = 0x00000000
  ETH_DMASR_TPS_Fetching_Pos* = (20)
  ETH_DMASR_TPS_Fetching_Msk* = (0x00000001 shl ETH_DMASR_TPS_Fetching_Pos) ## !< 0x00100000
  ETH_DMASR_TPS_Fetching* = ETH_DMASR_TPS_Fetching_Msk
  ETH_DMASR_TPS_Waiting_Pos* = (21)
  ETH_DMASR_TPS_Waiting_Msk* = (0x00000001 shl ETH_DMASR_TPS_Waiting_Pos) ## !< 0x00200000
  ETH_DMASR_TPS_Waiting* = ETH_DMASR_TPS_Waiting_Msk
  ETH_DMASR_TPS_Reading_Pos* = (20)
  ETH_DMASR_TPS_Reading_Msk* = (0x00000003 shl ETH_DMASR_TPS_Reading_Pos) ## !< 0x00300000
  ETH_DMASR_TPS_Reading* = ETH_DMASR_TPS_Reading_Msk
  ETH_DMASR_TPS_Suspended_Pos* = (21)
  ETH_DMASR_TPS_Suspended_Msk* = (0x00000003 shl ETH_DMASR_TPS_Suspended_Pos) ## !< 0x00600000
  ETH_DMASR_TPS_Suspended* = ETH_DMASR_TPS_Suspended_Msk
  ETH_DMASR_TPS_Closing_Pos* = (20)
  ETH_DMASR_TPS_Closing_Msk* = (0x00000007 shl ETH_DMASR_TPS_Closing_Pos) ## !< 0x00700000
  ETH_DMASR_TPS_Closing* = ETH_DMASR_TPS_Closing_Msk
  ETH_DMASR_RPS_Pos* = (17)
  ETH_DMASR_RPS_Msk* = (0x00000007 shl ETH_DMASR_RPS_Pos) ## !< 0x000E0000
  ETH_DMASR_RPS* = ETH_DMASR_RPS_Msk
  ETH_DMASR_RPS_Stopped* = 0x00000000
  ETH_DMASR_RPS_Fetching_Pos* = (17)
  ETH_DMASR_RPS_Fetching_Msk* = (0x00000001 shl ETH_DMASR_RPS_Fetching_Pos) ## !< 0x00020000
  ETH_DMASR_RPS_Fetching* = ETH_DMASR_RPS_Fetching_Msk
  ETH_DMASR_RPS_Waiting_Pos* = (17)
  ETH_DMASR_RPS_Waiting_Msk* = (0x00000003 shl ETH_DMASR_RPS_Waiting_Pos) ## !< 0x00060000
  ETH_DMASR_RPS_Waiting* = ETH_DMASR_RPS_Waiting_Msk
  ETH_DMASR_RPS_Suspended_Pos* = (19)
  ETH_DMASR_RPS_Suspended_Msk* = (0x00000001 shl ETH_DMASR_RPS_Suspended_Pos) ## !< 0x00080000
  ETH_DMASR_RPS_Suspended* = ETH_DMASR_RPS_Suspended_Msk
  ETH_DMASR_RPS_Closing_Pos* = (17)
  ETH_DMASR_RPS_Closing_Msk* = (0x00000005 shl ETH_DMASR_RPS_Closing_Pos) ## !< 0x000A0000
  ETH_DMASR_RPS_Closing* = ETH_DMASR_RPS_Closing_Msk
  ETH_DMASR_RPS_Queuing_Pos* = (17)
  ETH_DMASR_RPS_Queuing_Msk* = (0x00000007 shl ETH_DMASR_RPS_Queuing_Pos) ## !< 0x000E0000
  ETH_DMASR_RPS_Queuing* = ETH_DMASR_RPS_Queuing_Msk
  ETH_DMASR_NIS_Pos* = (16)
  ETH_DMASR_NIS_Msk* = (0x00000001 shl ETH_DMASR_NIS_Pos) ## !< 0x00010000
  ETH_DMASR_NIS* = ETH_DMASR_NIS_Msk
  ETH_DMASR_AIS_Pos* = (15)
  ETH_DMASR_AIS_Msk* = (0x00000001 shl ETH_DMASR_AIS_Pos) ## !< 0x00008000
  ETH_DMASR_AIS* = ETH_DMASR_AIS_Msk
  ETH_DMASR_ERS_Pos* = (14)
  ETH_DMASR_ERS_Msk* = (0x00000001 shl ETH_DMASR_ERS_Pos) ## !< 0x00004000
  ETH_DMASR_ERS* = ETH_DMASR_ERS_Msk
  ETH_DMASR_FBES_Pos* = (13)
  ETH_DMASR_FBES_Msk* = (0x00000001 shl ETH_DMASR_FBES_Pos) ## !< 0x00002000
  ETH_DMASR_FBES* = ETH_DMASR_FBES_Msk
  ETH_DMASR_ETS_Pos* = (10)
  ETH_DMASR_ETS_Msk* = (0x00000001 shl ETH_DMASR_ETS_Pos) ## !< 0x00000400
  ETH_DMASR_ETS* = ETH_DMASR_ETS_Msk
  ETH_DMASR_RWTS_Pos* = (9)
  ETH_DMASR_RWTS_Msk* = (0x00000001 shl ETH_DMASR_RWTS_Pos) ## !< 0x00000200
  ETH_DMASR_RWTS* = ETH_DMASR_RWTS_Msk
  ETH_DMASR_RPSS_Pos* = (8)
  ETH_DMASR_RPSS_Msk* = (0x00000001 shl ETH_DMASR_RPSS_Pos) ## !< 0x00000100
  ETH_DMASR_RPSS* = ETH_DMASR_RPSS_Msk
  ETH_DMASR_RBUS_Pos* = (7)
  ETH_DMASR_RBUS_Msk* = (0x00000001 shl ETH_DMASR_RBUS_Pos) ## !< 0x00000080
  ETH_DMASR_RBUS* = ETH_DMASR_RBUS_Msk
  ETH_DMASR_RS_Pos* = (6)
  ETH_DMASR_RS_Msk* = (0x00000001 shl ETH_DMASR_RS_Pos) ## !< 0x00000040
  ETH_DMASR_RS* = ETH_DMASR_RS_Msk
  ETH_DMASR_TUS_Pos* = (5)
  ETH_DMASR_TUS_Msk* = (0x00000001 shl ETH_DMASR_TUS_Pos) ## !< 0x00000020
  ETH_DMASR_TUS* = ETH_DMASR_TUS_Msk
  ETH_DMASR_ROS_Pos* = (4)
  ETH_DMASR_ROS_Msk* = (0x00000001 shl ETH_DMASR_ROS_Pos) ## !< 0x00000010
  ETH_DMASR_ROS* = ETH_DMASR_ROS_Msk
  ETH_DMASR_TJTS_Pos* = (3)
  ETH_DMASR_TJTS_Msk* = (0x00000001 shl ETH_DMASR_TJTS_Pos) ## !< 0x00000008
  ETH_DMASR_TJTS* = ETH_DMASR_TJTS_Msk
  ETH_DMASR_TBUS_Pos* = (2)
  ETH_DMASR_TBUS_Msk* = (0x00000001 shl ETH_DMASR_TBUS_Pos) ## !< 0x00000004
  ETH_DMASR_TBUS* = ETH_DMASR_TBUS_Msk
  ETH_DMASR_TPSS_Pos* = (1)
  ETH_DMASR_TPSS_Msk* = (0x00000001 shl ETH_DMASR_TPSS_Pos) ## !< 0x00000002
  ETH_DMASR_TPSS* = ETH_DMASR_TPSS_Msk
  ETH_DMASR_TS_Pos* = (0)
  ETH_DMASR_TS_Msk* = (0x00000001 shl ETH_DMASR_TS_Pos) ## !< 0x00000001
  ETH_DMASR_TS* = ETH_DMASR_TS_Msk

##  Bit definition for Ethernet DMA Operation Mode Register

const
  ETH_DMAOMR_DTCEFD_Pos* = (26)
  ETH_DMAOMR_DTCEFD_Msk* = (0x00000001 shl ETH_DMAOMR_DTCEFD_Pos) ## !< 0x04000000
  ETH_DMAOMR_DTCEFD* = ETH_DMAOMR_DTCEFD_Msk
  ETH_DMAOMR_RSF_Pos* = (25)
  ETH_DMAOMR_RSF_Msk* = (0x00000001 shl ETH_DMAOMR_RSF_Pos) ## !< 0x02000000
  ETH_DMAOMR_RSF* = ETH_DMAOMR_RSF_Msk
  ETH_DMAOMR_DFRF_Pos* = (24)
  ETH_DMAOMR_DFRF_Msk* = (0x00000001 shl ETH_DMAOMR_DFRF_Pos) ## !< 0x01000000
  ETH_DMAOMR_DFRF* = ETH_DMAOMR_DFRF_Msk
  ETH_DMAOMR_TSF_Pos* = (21)
  ETH_DMAOMR_TSF_Msk* = (0x00000001 shl ETH_DMAOMR_TSF_Pos) ## !< 0x00200000
  ETH_DMAOMR_TSF* = ETH_DMAOMR_TSF_Msk
  ETH_DMAOMR_FTF_Pos* = (20)
  ETH_DMAOMR_FTF_Msk* = (0x00000001 shl ETH_DMAOMR_FTF_Pos) ## !< 0x00100000
  ETH_DMAOMR_FTF* = ETH_DMAOMR_FTF_Msk
  ETH_DMAOMR_TTC_Pos* = (14)
  ETH_DMAOMR_TTC_Msk* = (0x00000007 shl ETH_DMAOMR_TTC_Pos) ## !< 0x0001C000
  ETH_DMAOMR_TTC* = ETH_DMAOMR_TTC_Msk
  ETH_DMAOMR_TTC_64Bytes* = 0x00000000
  ETH_DMAOMR_TTC_128Bytes* = 0x00004000
  ETH_DMAOMR_TTC_192Bytes* = 0x00008000
  ETH_DMAOMR_TTC_256Bytes* = 0x0000C000
  ETH_DMAOMR_TTC_40Bytes* = 0x00010000
  ETH_DMAOMR_TTC_32Bytes* = 0x00014000
  ETH_DMAOMR_TTC_24Bytes* = 0x00018000
  ETH_DMAOMR_TTC_16Bytes* = 0x0001C000
  ETH_DMAOMR_ST_Pos* = (13)
  ETH_DMAOMR_ST_Msk* = (0x00000001 shl ETH_DMAOMR_ST_Pos) ## !< 0x00002000
  ETH_DMAOMR_ST* = ETH_DMAOMR_ST_Msk
  ETH_DMAOMR_FEF_Pos* = (7)
  ETH_DMAOMR_FEF_Msk* = (0x00000001 shl ETH_DMAOMR_FEF_Pos) ## !< 0x00000080
  ETH_DMAOMR_FEF* = ETH_DMAOMR_FEF_Msk
  ETH_DMAOMR_FUGF_Pos* = (6)
  ETH_DMAOMR_FUGF_Msk* = (0x00000001 shl ETH_DMAOMR_FUGF_Pos) ## !< 0x00000040
  ETH_DMAOMR_FUGF* = ETH_DMAOMR_FUGF_Msk
  ETH_DMAOMR_RTC_Pos* = (3)
  ETH_DMAOMR_RTC_Msk* = (0x00000003 shl ETH_DMAOMR_RTC_Pos) ## !< 0x00000018
  ETH_DMAOMR_RTC* = ETH_DMAOMR_RTC_Msk
  ETH_DMAOMR_RTC_64Bytes* = 0x00000000
  ETH_DMAOMR_RTC_32Bytes* = 0x00000008
  ETH_DMAOMR_RTC_96Bytes* = 0x00000010
  ETH_DMAOMR_RTC_128Bytes* = 0x00000018
  ETH_DMAOMR_OSF_Pos* = (2)
  ETH_DMAOMR_OSF_Msk* = (0x00000001 shl ETH_DMAOMR_OSF_Pos) ## !< 0x00000004
  ETH_DMAOMR_OSF* = ETH_DMAOMR_OSF_Msk
  ETH_DMAOMR_SR_Pos* = (1)
  ETH_DMAOMR_SR_Msk* = (0x00000001 shl ETH_DMAOMR_SR_Pos) ## !< 0x00000002
  ETH_DMAOMR_SR* = ETH_DMAOMR_SR_Msk

##  Bit definition for Ethernet DMA Interrupt Enable Register

const
  ETH_DMAIER_NISE_Pos* = (16)
  ETH_DMAIER_NISE_Msk* = (0x00000001 shl ETH_DMAIER_NISE_Pos) ## !< 0x00010000
  ETH_DMAIER_NISE* = ETH_DMAIER_NISE_Msk
  ETH_DMAIER_AISE_Pos* = (15)
  ETH_DMAIER_AISE_Msk* = (0x00000001 shl ETH_DMAIER_AISE_Pos) ## !< 0x00008000
  ETH_DMAIER_AISE* = ETH_DMAIER_AISE_Msk
  ETH_DMAIER_ERIE_Pos* = (14)
  ETH_DMAIER_ERIE_Msk* = (0x00000001 shl ETH_DMAIER_ERIE_Pos) ## !< 0x00004000
  ETH_DMAIER_ERIE* = ETH_DMAIER_ERIE_Msk
  ETH_DMAIER_FBEIE_Pos* = (13)
  ETH_DMAIER_FBEIE_Msk* = (0x00000001 shl ETH_DMAIER_FBEIE_Pos) ## !< 0x00002000
  ETH_DMAIER_FBEIE* = ETH_DMAIER_FBEIE_Msk
  ETH_DMAIER_ETIE_Pos* = (10)
  ETH_DMAIER_ETIE_Msk* = (0x00000001 shl ETH_DMAIER_ETIE_Pos) ## !< 0x00000400
  ETH_DMAIER_ETIE* = ETH_DMAIER_ETIE_Msk
  ETH_DMAIER_RWTIE_Pos* = (9)
  ETH_DMAIER_RWTIE_Msk* = (0x00000001 shl ETH_DMAIER_RWTIE_Pos) ## !< 0x00000200
  ETH_DMAIER_RWTIE* = ETH_DMAIER_RWTIE_Msk
  ETH_DMAIER_RPSIE_Pos* = (8)
  ETH_DMAIER_RPSIE_Msk* = (0x00000001 shl ETH_DMAIER_RPSIE_Pos) ## !< 0x00000100
  ETH_DMAIER_RPSIE* = ETH_DMAIER_RPSIE_Msk
  ETH_DMAIER_RBUIE_Pos* = (7)
  ETH_DMAIER_RBUIE_Msk* = (0x00000001 shl ETH_DMAIER_RBUIE_Pos) ## !< 0x00000080
  ETH_DMAIER_RBUIE* = ETH_DMAIER_RBUIE_Msk
  ETH_DMAIER_RIE_Pos* = (6)
  ETH_DMAIER_RIE_Msk* = (0x00000001 shl ETH_DMAIER_RIE_Pos) ## !< 0x00000040
  ETH_DMAIER_RIE* = ETH_DMAIER_RIE_Msk
  ETH_DMAIER_TUIE_Pos* = (5)
  ETH_DMAIER_TUIE_Msk* = (0x00000001 shl ETH_DMAIER_TUIE_Pos) ## !< 0x00000020
  ETH_DMAIER_TUIE* = ETH_DMAIER_TUIE_Msk
  ETH_DMAIER_ROIE_Pos* = (4)
  ETH_DMAIER_ROIE_Msk* = (0x00000001 shl ETH_DMAIER_ROIE_Pos) ## !< 0x00000010
  ETH_DMAIER_ROIE* = ETH_DMAIER_ROIE_Msk
  ETH_DMAIER_TJTIE_Pos* = (3)
  ETH_DMAIER_TJTIE_Msk* = (0x00000001 shl ETH_DMAIER_TJTIE_Pos) ## !< 0x00000008
  ETH_DMAIER_TJTIE* = ETH_DMAIER_TJTIE_Msk
  ETH_DMAIER_TBUIE_Pos* = (2)
  ETH_DMAIER_TBUIE_Msk* = (0x00000001 shl ETH_DMAIER_TBUIE_Pos) ## !< 0x00000004
  ETH_DMAIER_TBUIE* = ETH_DMAIER_TBUIE_Msk
  ETH_DMAIER_TPSIE_Pos* = (1)
  ETH_DMAIER_TPSIE_Msk* = (0x00000001 shl ETH_DMAIER_TPSIE_Pos) ## !< 0x00000002
  ETH_DMAIER_TPSIE* = ETH_DMAIER_TPSIE_Msk
  ETH_DMAIER_TIE_Pos* = (0)
  ETH_DMAIER_TIE_Msk* = (0x00000001 shl ETH_DMAIER_TIE_Pos) ## !< 0x00000001
  ETH_DMAIER_TIE* = ETH_DMAIER_TIE_Msk

##  Bit definition for Ethernet DMA Missed Frame and Buffer Overflow Counter Register

const
  ETH_DMAMFBOCR_OFOC_Pos* = (28)
  ETH_DMAMFBOCR_OFOC_Msk* = (0x00000001 shl ETH_DMAMFBOCR_OFOC_Pos) ## !< 0x10000000
  ETH_DMAMFBOCR_OFOC* = ETH_DMAMFBOCR_OFOC_Msk
  ETH_DMAMFBOCR_MFA_Pos* = (17)
  ETH_DMAMFBOCR_MFA_Msk* = (0x000007FF shl ETH_DMAMFBOCR_MFA_Pos) ## !< 0x0FFE0000
  ETH_DMAMFBOCR_MFA* = ETH_DMAMFBOCR_MFA_Msk
  ETH_DMAMFBOCR_OMFC_Pos* = (16)
  ETH_DMAMFBOCR_OMFC_Msk* = (0x00000001 shl ETH_DMAMFBOCR_OMFC_Pos) ## !< 0x00010000
  ETH_DMAMFBOCR_OMFC* = ETH_DMAMFBOCR_OMFC_Msk
  ETH_DMAMFBOCR_MFC_Pos* = (0)
  ETH_DMAMFBOCR_MFC_Msk* = (0x0000FFFF shl ETH_DMAMFBOCR_MFC_Pos) ## !< 0x0000FFFF
  ETH_DMAMFBOCR_MFC* = ETH_DMAMFBOCR_MFC_Msk

##  Bit definition for Ethernet DMA Current Host Transmit Descriptor Register

const
  ETH_DMACHTDR_HTDAP_Pos* = (0)
  ETH_DMACHTDR_HTDAP_Msk* = (0xFFFFFFFF shl ETH_DMACHTDR_HTDAP_Pos) ## !< 0xFFFFFFFF
  ETH_DMACHTDR_HTDAP* = ETH_DMACHTDR_HTDAP_Msk

##  Bit definition for Ethernet DMA Current Host Receive Descriptor Register

const
  ETH_DMACHRDR_HRDAP_Pos* = (0)
  ETH_DMACHRDR_HRDAP_Msk* = (0xFFFFFFFF shl ETH_DMACHRDR_HRDAP_Pos) ## !< 0xFFFFFFFF
  ETH_DMACHRDR_HRDAP* = ETH_DMACHRDR_HRDAP_Msk

##  Bit definition for Ethernet DMA Current Host Transmit Buffer Address Register

const
  ETH_DMACHTBAR_HTBAP_Pos* = (0)
  ETH_DMACHTBAR_HTBAP_Msk* = (0xFFFFFFFF shl ETH_DMACHTBAR_HTBAP_Pos) ## !< 0xFFFFFFFF
  ETH_DMACHTBAR_HTBAP* = ETH_DMACHTBAR_HTBAP_Msk

##  Bit definition for Ethernet DMA Current Host Receive Buffer Address Register

const
  ETH_DMACHRBAR_HRBAP_Pos* = (0)
  ETH_DMACHRBAR_HRBAP_Msk* = (0xFFFFFFFF shl ETH_DMACHRBAR_HRBAP_Pos) ## !< 0xFFFFFFFF
  ETH_DMACHRBAR_HRBAP* = ETH_DMACHRBAR_HRBAP_Msk

## ****************************************************************************
##
##                                        USB_OTG
##
## ****************************************************************************
## *******************  Bit definition for USB_OTG_GOTGCTL register  **********

const
  USB_OTG_GOTGCTL_SRQSCS_Pos* = (0)
  USB_OTG_GOTGCTL_SRQSCS_Msk* = (0x00000001 shl USB_OTG_GOTGCTL_SRQSCS_Pos) ## !< 0x00000001
  USB_OTG_GOTGCTL_SRQSCS* = USB_OTG_GOTGCTL_SRQSCS_Msk
  USB_OTG_GOTGCTL_SRQ_Pos* = (1)
  USB_OTG_GOTGCTL_SRQ_Msk* = (0x00000001 shl USB_OTG_GOTGCTL_SRQ_Pos) ## !< 0x00000002
  USB_OTG_GOTGCTL_SRQ* = USB_OTG_GOTGCTL_SRQ_Msk
  USB_OTG_GOTGCTL_HNGSCS_Pos* = (8)
  USB_OTG_GOTGCTL_HNGSCS_Msk* = (0x00000001 shl USB_OTG_GOTGCTL_HNGSCS_Pos) ## !< 0x00000100
  USB_OTG_GOTGCTL_HNGSCS* = USB_OTG_GOTGCTL_HNGSCS_Msk
  USB_OTG_GOTGCTL_HNPRQ_Pos* = (9)
  USB_OTG_GOTGCTL_HNPRQ_Msk* = (0x00000001 shl USB_OTG_GOTGCTL_HNPRQ_Pos) ## !< 0x00000200
  USB_OTG_GOTGCTL_HNPRQ* = USB_OTG_GOTGCTL_HNPRQ_Msk
  USB_OTG_GOTGCTL_HSHNPEN_Pos* = (10)
  USB_OTG_GOTGCTL_HSHNPEN_Msk* = (0x00000001 shl USB_OTG_GOTGCTL_HSHNPEN_Pos) ## !< 0x00000400
  USB_OTG_GOTGCTL_HSHNPEN* = USB_OTG_GOTGCTL_HSHNPEN_Msk
  USB_OTG_GOTGCTL_DHNPEN_Pos* = (11)
  USB_OTG_GOTGCTL_DHNPEN_Msk* = (0x00000001 shl USB_OTG_GOTGCTL_DHNPEN_Pos) ## !< 0x00000800
  USB_OTG_GOTGCTL_DHNPEN* = USB_OTG_GOTGCTL_DHNPEN_Msk
  USB_OTG_GOTGCTL_CIDSTS_Pos* = (16)
  USB_OTG_GOTGCTL_CIDSTS_Msk* = (0x00000001 shl USB_OTG_GOTGCTL_CIDSTS_Pos) ## !< 0x00010000
  USB_OTG_GOTGCTL_CIDSTS* = USB_OTG_GOTGCTL_CIDSTS_Msk
  USB_OTG_GOTGCTL_DBCT_Pos* = (17)
  USB_OTG_GOTGCTL_DBCT_Msk* = (0x00000001 shl USB_OTG_GOTGCTL_DBCT_Pos) ## !< 0x00020000
  USB_OTG_GOTGCTL_DBCT* = USB_OTG_GOTGCTL_DBCT_Msk
  USB_OTG_GOTGCTL_ASVLD_Pos* = (18)
  USB_OTG_GOTGCTL_ASVLD_Msk* = (0x00000001 shl USB_OTG_GOTGCTL_ASVLD_Pos) ## !< 0x00040000
  USB_OTG_GOTGCTL_ASVLD* = USB_OTG_GOTGCTL_ASVLD_Msk
  USB_OTG_GOTGCTL_BSVLD_Pos* = (19)
  USB_OTG_GOTGCTL_BSVLD_Msk* = (0x00000001 shl USB_OTG_GOTGCTL_BSVLD_Pos) ## !< 0x00080000
  USB_OTG_GOTGCTL_BSVLD* = USB_OTG_GOTGCTL_BSVLD_Msk

## *******************  Bit definition forUSB_OTG_HCFG register  *******************

const
  USB_OTG_HCFG_FSLSPCS_Pos* = (0)
  USB_OTG_HCFG_FSLSPCS_Msk* = (0x00000003 shl USB_OTG_HCFG_FSLSPCS_Pos) ## !< 0x00000003
  USB_OTG_HCFG_FSLSPCS* = USB_OTG_HCFG_FSLSPCS_Msk
  USB_OTG_HCFG_FSLSPCS_0* = (0x00000001 shl USB_OTG_HCFG_FSLSPCS_Pos) ## !< 0x00000001
  USB_OTG_HCFG_FSLSPCS_1* = (0x00000002 shl USB_OTG_HCFG_FSLSPCS_Pos) ## !< 0x00000002
  USB_OTG_HCFG_FSLSS_Pos* = (2)
  USB_OTG_HCFG_FSLSS_Msk* = (0x00000001 shl USB_OTG_HCFG_FSLSS_Pos) ## !< 0x00000004
  USB_OTG_HCFG_FSLSS* = USB_OTG_HCFG_FSLSS_Msk

## *******************  Bit definition for USB_OTG_DCFG register  *******************

const
  USB_OTG_DCFG_DSPD_Pos* = (0)
  USB_OTG_DCFG_DSPD_Msk* = (0x00000003 shl USB_OTG_DCFG_DSPD_Pos) ## !< 0x00000003
  USB_OTG_DCFG_DSPD* = USB_OTG_DCFG_DSPD_Msk
  USB_OTG_DCFG_DSPD_0* = (0x00000001 shl USB_OTG_DCFG_DSPD_Pos) ## !< 0x00000001
  USB_OTG_DCFG_DSPD_1* = (0x00000002 shl USB_OTG_DCFG_DSPD_Pos) ## !< 0x00000002
  USB_OTG_DCFG_NZLSOHSK_Pos* = (2)
  USB_OTG_DCFG_NZLSOHSK_Msk* = (0x00000001 shl USB_OTG_DCFG_NZLSOHSK_Pos) ## !< 0x00000004
  USB_OTG_DCFG_NZLSOHSK* = USB_OTG_DCFG_NZLSOHSK_Msk
  USB_OTG_DCFG_DAD_Pos* = (4)
  USB_OTG_DCFG_DAD_Msk* = (0x0000007F shl USB_OTG_DCFG_DAD_Pos) ## !< 0x000007F0
  USB_OTG_DCFG_DAD* = USB_OTG_DCFG_DAD_Msk
  USB_OTG_DCFG_DAD_0* = (0x00000001 shl USB_OTG_DCFG_DAD_Pos) ## !< 0x00000010
  USB_OTG_DCFG_DAD_1* = (0x00000002 shl USB_OTG_DCFG_DAD_Pos) ## !< 0x00000020
  USB_OTG_DCFG_DAD_2* = (0x00000004 shl USB_OTG_DCFG_DAD_Pos) ## !< 0x00000040
  USB_OTG_DCFG_DAD_3* = (0x00000008 shl USB_OTG_DCFG_DAD_Pos) ## !< 0x00000080
  USB_OTG_DCFG_DAD_4* = (0x00000010 shl USB_OTG_DCFG_DAD_Pos) ## !< 0x00000100
  USB_OTG_DCFG_DAD_5* = (0x00000020 shl USB_OTG_DCFG_DAD_Pos) ## !< 0x00000200
  USB_OTG_DCFG_DAD_6* = (0x00000040 shl USB_OTG_DCFG_DAD_Pos) ## !< 0x00000400
  USB_OTG_DCFG_PFIVL_Pos* = (11)
  USB_OTG_DCFG_PFIVL_Msk* = (0x00000003 shl USB_OTG_DCFG_PFIVL_Pos) ## !< 0x00001800
  USB_OTG_DCFG_PFIVL* = USB_OTG_DCFG_PFIVL_Msk
  USB_OTG_DCFG_PFIVL_0* = (0x00000001 shl USB_OTG_DCFG_PFIVL_Pos) ## !< 0x00000800
  USB_OTG_DCFG_PFIVL_1* = (0x00000002 shl USB_OTG_DCFG_PFIVL_Pos) ## !< 0x00001000
  USB_OTG_DCFG_XCVRDLY_Pos* = (14)
  USB_OTG_DCFG_XCVRDLY_Msk* = (0x00000001 shl USB_OTG_DCFG_XCVRDLY_Pos) ## !< 0x00004000
  USB_OTG_DCFG_XCVRDLY* = USB_OTG_DCFG_XCVRDLY_Msk
  USB_OTG_DCFG_ERRATIM_Pos* = (15)
  USB_OTG_DCFG_ERRATIM_Msk* = (0x00000001 shl USB_OTG_DCFG_ERRATIM_Pos) ## !< 0x00008000
  USB_OTG_DCFG_ERRATIM* = USB_OTG_DCFG_ERRATIM_Msk
  USB_OTG_DCFG_PERSCHIVL_Pos* = (24)
  USB_OTG_DCFG_PERSCHIVL_Msk* = (0x00000003 shl USB_OTG_DCFG_PERSCHIVL_Pos) ## !< 0x03000000
  USB_OTG_DCFG_PERSCHIVL* = USB_OTG_DCFG_PERSCHIVL_Msk
  USB_OTG_DCFG_PERSCHIVL_0* = (0x00000001 shl USB_OTG_DCFG_PERSCHIVL_Pos) ## !< 0x01000000
  USB_OTG_DCFG_PERSCHIVL_1* = (0x00000002 shl USB_OTG_DCFG_PERSCHIVL_Pos) ## !< 0x02000000

## *******************  Bit definition for USB_OTG_PCGCR register  *******************

const
  USB_OTG_PCGCR_STPPCLK_Pos* = (0)
  USB_OTG_PCGCR_STPPCLK_Msk* = (0x00000001 shl USB_OTG_PCGCR_STPPCLK_Pos) ## !< 0x00000001
  USB_OTG_PCGCR_STPPCLK* = USB_OTG_PCGCR_STPPCLK_Msk
  USB_OTG_PCGCR_GATEHCLK_Pos* = (1)
  USB_OTG_PCGCR_GATEHCLK_Msk* = (0x00000001 shl USB_OTG_PCGCR_GATEHCLK_Pos) ## !< 0x00000002
  USB_OTG_PCGCR_GATEHCLK* = USB_OTG_PCGCR_GATEHCLK_Msk
  USB_OTG_PCGCR_PHYSUSP_Pos* = (4)
  USB_OTG_PCGCR_PHYSUSP_Msk* = (0x00000001 shl USB_OTG_PCGCR_PHYSUSP_Pos) ## !< 0x00000010
  USB_OTG_PCGCR_PHYSUSP* = USB_OTG_PCGCR_PHYSUSP_Msk

## *******************  Bit definition for USB_OTG_GOTGINT register  *******************

const
  USB_OTG_GOTGINT_SEDET_Pos* = (2)
  USB_OTG_GOTGINT_SEDET_Msk* = (0x00000001 shl USB_OTG_GOTGINT_SEDET_Pos) ## !< 0x00000004
  USB_OTG_GOTGINT_SEDET* = USB_OTG_GOTGINT_SEDET_Msk
  USB_OTG_GOTGINT_SRSSCHG_Pos* = (8)
  USB_OTG_GOTGINT_SRSSCHG_Msk* = (0x00000001 shl USB_OTG_GOTGINT_SRSSCHG_Pos) ## !< 0x00000100
  USB_OTG_GOTGINT_SRSSCHG* = USB_OTG_GOTGINT_SRSSCHG_Msk
  USB_OTG_GOTGINT_HNSSCHG_Pos* = (9)
  USB_OTG_GOTGINT_HNSSCHG_Msk* = (0x00000001 shl USB_OTG_GOTGINT_HNSSCHG_Pos) ## !< 0x00000200
  USB_OTG_GOTGINT_HNSSCHG* = USB_OTG_GOTGINT_HNSSCHG_Msk
  USB_OTG_GOTGINT_HNGDET_Pos* = (17)
  USB_OTG_GOTGINT_HNGDET_Msk* = (0x00000001 shl USB_OTG_GOTGINT_HNGDET_Pos) ## !< 0x00020000
  USB_OTG_GOTGINT_HNGDET* = USB_OTG_GOTGINT_HNGDET_Msk
  USB_OTG_GOTGINT_ADTOCHG_Pos* = (18)
  USB_OTG_GOTGINT_ADTOCHG_Msk* = (0x00000001 shl USB_OTG_GOTGINT_ADTOCHG_Pos) ## !< 0x00040000
  USB_OTG_GOTGINT_ADTOCHG* = USB_OTG_GOTGINT_ADTOCHG_Msk
  USB_OTG_GOTGINT_DBCDNE_Pos* = (19)
  USB_OTG_GOTGINT_DBCDNE_Msk* = (0x00000001 shl USB_OTG_GOTGINT_DBCDNE_Pos) ## !< 0x00080000
  USB_OTG_GOTGINT_DBCDNE* = USB_OTG_GOTGINT_DBCDNE_Msk

## *******************  Bit definition for USB_OTG_DCTL register  *******************

const
  USB_OTG_DCTL_RWUSIG_Pos* = (0)
  USB_OTG_DCTL_RWUSIG_Msk* = (0x00000001 shl USB_OTG_DCTL_RWUSIG_Pos) ## !< 0x00000001
  USB_OTG_DCTL_RWUSIG* = USB_OTG_DCTL_RWUSIG_Msk
  USB_OTG_DCTL_SDIS_Pos* = (1)
  USB_OTG_DCTL_SDIS_Msk* = (0x00000001 shl USB_OTG_DCTL_SDIS_Pos) ## !< 0x00000002
  USB_OTG_DCTL_SDIS* = USB_OTG_DCTL_SDIS_Msk
  USB_OTG_DCTL_GINSTS_Pos* = (2)
  USB_OTG_DCTL_GINSTS_Msk* = (0x00000001 shl USB_OTG_DCTL_GINSTS_Pos) ## !< 0x00000004
  USB_OTG_DCTL_GINSTS* = USB_OTG_DCTL_GINSTS_Msk
  USB_OTG_DCTL_GONSTS_Pos* = (3)
  USB_OTG_DCTL_GONSTS_Msk* = (0x00000001 shl USB_OTG_DCTL_GONSTS_Pos) ## !< 0x00000008
  USB_OTG_DCTL_GONSTS* = USB_OTG_DCTL_GONSTS_Msk
  USB_OTG_DCTL_TCTL_Pos* = (4)
  USB_OTG_DCTL_TCTL_Msk* = (0x00000007 shl USB_OTG_DCTL_TCTL_Pos) ## !< 0x00000070
  USB_OTG_DCTL_TCTL* = USB_OTG_DCTL_TCTL_Msk
  USB_OTG_DCTL_TCTL_0* = (0x00000001 shl USB_OTG_DCTL_TCTL_Pos) ## !< 0x00000010
  USB_OTG_DCTL_TCTL_1* = (0x00000002 shl USB_OTG_DCTL_TCTL_Pos) ## !< 0x00000020
  USB_OTG_DCTL_TCTL_2* = (0x00000004 shl USB_OTG_DCTL_TCTL_Pos) ## !< 0x00000040
  USB_OTG_DCTL_SGINAK_Pos* = (7)
  USB_OTG_DCTL_SGINAK_Msk* = (0x00000001 shl USB_OTG_DCTL_SGINAK_Pos) ## !< 0x00000080
  USB_OTG_DCTL_SGINAK* = USB_OTG_DCTL_SGINAK_Msk
  USB_OTG_DCTL_CGINAK_Pos* = (8)
  USB_OTG_DCTL_CGINAK_Msk* = (0x00000001 shl USB_OTG_DCTL_CGINAK_Pos) ## !< 0x00000100
  USB_OTG_DCTL_CGINAK* = USB_OTG_DCTL_CGINAK_Msk
  USB_OTG_DCTL_SGONAK_Pos* = (9)
  USB_OTG_DCTL_SGONAK_Msk* = (0x00000001 shl USB_OTG_DCTL_SGONAK_Pos) ## !< 0x00000200
  USB_OTG_DCTL_SGONAK* = USB_OTG_DCTL_SGONAK_Msk
  USB_OTG_DCTL_CGONAK_Pos* = (10)
  USB_OTG_DCTL_CGONAK_Msk* = (0x00000001 shl USB_OTG_DCTL_CGONAK_Pos) ## !< 0x00000400
  USB_OTG_DCTL_CGONAK* = USB_OTG_DCTL_CGONAK_Msk
  USB_OTG_DCTL_POPRGDNE_Pos* = (11)
  USB_OTG_DCTL_POPRGDNE_Msk* = (0x00000001 shl USB_OTG_DCTL_POPRGDNE_Pos) ## !< 0x00000800
  USB_OTG_DCTL_POPRGDNE* = USB_OTG_DCTL_POPRGDNE_Msk

## *******************  Bit definition for USB_OTG_HFIR register  *******************

const
  USB_OTG_HFIR_FRIVL_Pos* = (0)
  USB_OTG_HFIR_FRIVL_Msk* = (0x0000FFFF shl USB_OTG_HFIR_FRIVL_Pos) ## !< 0x0000FFFF
  USB_OTG_HFIR_FRIVL* = USB_OTG_HFIR_FRIVL_Msk

## *******************  Bit definition for USB_OTG_HFNUM register  *******************

const
  USB_OTG_HFNUM_FRNUM_Pos* = (0)
  USB_OTG_HFNUM_FRNUM_Msk* = (0x0000FFFF shl USB_OTG_HFNUM_FRNUM_Pos) ## !< 0x0000FFFF
  USB_OTG_HFNUM_FRNUM* = USB_OTG_HFNUM_FRNUM_Msk
  USB_OTG_HFNUM_FTREM_Pos* = (16)
  USB_OTG_HFNUM_FTREM_Msk* = (0x0000FFFF shl USB_OTG_HFNUM_FTREM_Pos) ## !< 0xFFFF0000
  USB_OTG_HFNUM_FTREM* = USB_OTG_HFNUM_FTREM_Msk

## *******************  Bit definition for USB_OTG_DSTS register  *******************

const
  USB_OTG_DSTS_SUSPSTS_Pos* = (0)
  USB_OTG_DSTS_SUSPSTS_Msk* = (0x00000001 shl USB_OTG_DSTS_SUSPSTS_Pos) ## !< 0x00000001
  USB_OTG_DSTS_SUSPSTS* = USB_OTG_DSTS_SUSPSTS_Msk
  USB_OTG_DSTS_ENUMSPD_Pos* = (1)
  USB_OTG_DSTS_ENUMSPD_Msk* = (0x00000003 shl USB_OTG_DSTS_ENUMSPD_Pos) ## !< 0x00000006
  USB_OTG_DSTS_ENUMSPD* = USB_OTG_DSTS_ENUMSPD_Msk
  USB_OTG_DSTS_ENUMSPD_0* = (0x00000001 shl USB_OTG_DSTS_ENUMSPD_Pos) ## !< 0x00000002
  USB_OTG_DSTS_ENUMSPD_1* = (0x00000002 shl USB_OTG_DSTS_ENUMSPD_Pos) ## !< 0x00000004
  USB_OTG_DSTS_EERR_Pos* = (3)
  USB_OTG_DSTS_EERR_Msk* = (0x00000001 shl USB_OTG_DSTS_EERR_Pos) ## !< 0x00000008
  USB_OTG_DSTS_EERR* = USB_OTG_DSTS_EERR_Msk
  USB_OTG_DSTS_FNSOF_Pos* = (8)
  USB_OTG_DSTS_FNSOF_Msk* = (0x00003FFF shl USB_OTG_DSTS_FNSOF_Pos) ## !< 0x003FFF00
  USB_OTG_DSTS_FNSOF* = USB_OTG_DSTS_FNSOF_Msk

## *******************  Bit definition for USB_OTG_GAHBCFG register  *******************

const
  USB_OTG_GAHBCFG_GINT_Pos* = (0)
  USB_OTG_GAHBCFG_GINT_Msk* = (0x00000001 shl USB_OTG_GAHBCFG_GINT_Pos) ## !< 0x00000001
  USB_OTG_GAHBCFG_GINT* = USB_OTG_GAHBCFG_GINT_Msk
  USB_OTG_GAHBCFG_HBSTLEN_Pos* = (1)
  USB_OTG_GAHBCFG_HBSTLEN_Msk* = (0x0000000F shl USB_OTG_GAHBCFG_HBSTLEN_Pos) ## !< 0x0000001E
  USB_OTG_GAHBCFG_HBSTLEN* = USB_OTG_GAHBCFG_HBSTLEN_Msk
  USB_OTG_GAHBCFG_HBSTLEN_0* = (0x00000000 shl USB_OTG_GAHBCFG_HBSTLEN_Pos) ## !< Single
  USB_OTG_GAHBCFG_HBSTLEN_1* = (0x00000001 shl USB_OTG_GAHBCFG_HBSTLEN_Pos) ## !< INCR
  USB_OTG_GAHBCFG_HBSTLEN_2* = (0x00000003 shl USB_OTG_GAHBCFG_HBSTLEN_Pos) ## !< INCR4
  USB_OTG_GAHBCFG_HBSTLEN_3* = (0x00000005 shl USB_OTG_GAHBCFG_HBSTLEN_Pos) ## !< INCR8
  USB_OTG_GAHBCFG_HBSTLEN_4* = (0x00000007 shl USB_OTG_GAHBCFG_HBSTLEN_Pos) ## !< INCR16
  USB_OTG_GAHBCFG_DMAEN_Pos* = (5)
  USB_OTG_GAHBCFG_DMAEN_Msk* = (0x00000001 shl USB_OTG_GAHBCFG_DMAEN_Pos) ## !< 0x00000020
  USB_OTG_GAHBCFG_DMAEN* = USB_OTG_GAHBCFG_DMAEN_Msk
  USB_OTG_GAHBCFG_TXFELVL_Pos* = (7)
  USB_OTG_GAHBCFG_TXFELVL_Msk* = (0x00000001 shl USB_OTG_GAHBCFG_TXFELVL_Pos) ## !< 0x00000080
  USB_OTG_GAHBCFG_TXFELVL* = USB_OTG_GAHBCFG_TXFELVL_Msk
  USB_OTG_GAHBCFG_PTXFELVL_Pos* = (8)
  USB_OTG_GAHBCFG_PTXFELVL_Msk* = (0x00000001 shl USB_OTG_GAHBCFG_PTXFELVL_Pos) ## !< 0x00000100
  USB_OTG_GAHBCFG_PTXFELVL* = USB_OTG_GAHBCFG_PTXFELVL_Msk

## *******************  Bit definition for USB_OTG_GUSBCFG register  *******************

const
  USB_OTG_GUSBCFG_TOCAL_Pos* = (0)
  USB_OTG_GUSBCFG_TOCAL_Msk* = (0x00000007 shl USB_OTG_GUSBCFG_TOCAL_Pos) ## !< 0x00000007
  USB_OTG_GUSBCFG_TOCAL* = USB_OTG_GUSBCFG_TOCAL_Msk
  USB_OTG_GUSBCFG_TOCAL_0* = (0x00000001 shl USB_OTG_GUSBCFG_TOCAL_Pos) ## !< 0x00000001
  USB_OTG_GUSBCFG_TOCAL_1* = (0x00000002 shl USB_OTG_GUSBCFG_TOCAL_Pos) ## !< 0x00000002
  USB_OTG_GUSBCFG_TOCAL_2* = (0x00000004 shl USB_OTG_GUSBCFG_TOCAL_Pos) ## !< 0x00000004
  USB_OTG_GUSBCFG_PHYSEL_Pos* = (6)
  USB_OTG_GUSBCFG_PHYSEL_Msk* = (0x00000001 shl USB_OTG_GUSBCFG_PHYSEL_Pos) ## !< 0x00000040
  USB_OTG_GUSBCFG_PHYSEL* = USB_OTG_GUSBCFG_PHYSEL_Msk
  USB_OTG_GUSBCFG_SRPCAP_Pos* = (8)
  USB_OTG_GUSBCFG_SRPCAP_Msk* = (0x00000001 shl USB_OTG_GUSBCFG_SRPCAP_Pos) ## !< 0x00000100
  USB_OTG_GUSBCFG_SRPCAP* = USB_OTG_GUSBCFG_SRPCAP_Msk
  USB_OTG_GUSBCFG_HNPCAP_Pos* = (9)
  USB_OTG_GUSBCFG_HNPCAP_Msk* = (0x00000001 shl USB_OTG_GUSBCFG_HNPCAP_Pos) ## !< 0x00000200
  USB_OTG_GUSBCFG_HNPCAP* = USB_OTG_GUSBCFG_HNPCAP_Msk
  USB_OTG_GUSBCFG_TRDT_Pos* = (10)
  USB_OTG_GUSBCFG_TRDT_Msk* = (0x0000000F shl USB_OTG_GUSBCFG_TRDT_Pos) ## !< 0x00003C00
  USB_OTG_GUSBCFG_TRDT* = USB_OTG_GUSBCFG_TRDT_Msk
  USB_OTG_GUSBCFG_TRDT_0* = (0x00000001 shl USB_OTG_GUSBCFG_TRDT_Pos) ## !< 0x00000400
  USB_OTG_GUSBCFG_TRDT_1* = (0x00000002 shl USB_OTG_GUSBCFG_TRDT_Pos) ## !< 0x00000800
  USB_OTG_GUSBCFG_TRDT_2* = (0x00000004 shl USB_OTG_GUSBCFG_TRDT_Pos) ## !< 0x00001000
  USB_OTG_GUSBCFG_TRDT_3* = (0x00000008 shl USB_OTG_GUSBCFG_TRDT_Pos) ## !< 0x00002000
  USB_OTG_GUSBCFG_PHYLPCS_Pos* = (15)
  USB_OTG_GUSBCFG_PHYLPCS_Msk* = (0x00000001 shl USB_OTG_GUSBCFG_PHYLPCS_Pos) ## !< 0x00008000
  USB_OTG_GUSBCFG_PHYLPCS* = USB_OTG_GUSBCFG_PHYLPCS_Msk
  USB_OTG_GUSBCFG_ULPIFSLS_Pos* = (17)
  USB_OTG_GUSBCFG_ULPIFSLS_Msk* = (0x00000001 shl USB_OTG_GUSBCFG_ULPIFSLS_Pos) ## !< 0x00020000
  USB_OTG_GUSBCFG_ULPIFSLS* = USB_OTG_GUSBCFG_ULPIFSLS_Msk
  USB_OTG_GUSBCFG_ULPIAR_Pos* = (18)
  USB_OTG_GUSBCFG_ULPIAR_Msk* = (0x00000001 shl USB_OTG_GUSBCFG_ULPIAR_Pos) ## !< 0x00040000
  USB_OTG_GUSBCFG_ULPIAR* = USB_OTG_GUSBCFG_ULPIAR_Msk
  USB_OTG_GUSBCFG_ULPICSM_Pos* = (19)
  USB_OTG_GUSBCFG_ULPICSM_Msk* = (0x00000001 shl USB_OTG_GUSBCFG_ULPICSM_Pos) ## !< 0x00080000
  USB_OTG_GUSBCFG_ULPICSM* = USB_OTG_GUSBCFG_ULPICSM_Msk
  USB_OTG_GUSBCFG_ULPIEVBUSD_Pos* = (20)
  USB_OTG_GUSBCFG_ULPIEVBUSD_Msk* = (0x00000001 shl
      USB_OTG_GUSBCFG_ULPIEVBUSD_Pos) ## !< 0x00100000
  USB_OTG_GUSBCFG_ULPIEVBUSD* = USB_OTG_GUSBCFG_ULPIEVBUSD_Msk
  USB_OTG_GUSBCFG_ULPIEVBUSI_Pos* = (21)
  USB_OTG_GUSBCFG_ULPIEVBUSI_Msk* = (0x00000001 shl
      USB_OTG_GUSBCFG_ULPIEVBUSI_Pos) ## !< 0x00200000
  USB_OTG_GUSBCFG_ULPIEVBUSI* = USB_OTG_GUSBCFG_ULPIEVBUSI_Msk
  USB_OTG_GUSBCFG_TSDPS_Pos* = (22)
  USB_OTG_GUSBCFG_TSDPS_Msk* = (0x00000001 shl USB_OTG_GUSBCFG_TSDPS_Pos) ## !< 0x00400000
  USB_OTG_GUSBCFG_TSDPS* = USB_OTG_GUSBCFG_TSDPS_Msk
  USB_OTG_GUSBCFG_PCCI_Pos* = (23)
  USB_OTG_GUSBCFG_PCCI_Msk* = (0x00000001 shl USB_OTG_GUSBCFG_PCCI_Pos) ## !< 0x00800000
  USB_OTG_GUSBCFG_PCCI* = USB_OTG_GUSBCFG_PCCI_Msk
  USB_OTG_GUSBCFG_PTCI_Pos* = (24)
  USB_OTG_GUSBCFG_PTCI_Msk* = (0x00000001 shl USB_OTG_GUSBCFG_PTCI_Pos) ## !< 0x01000000
  USB_OTG_GUSBCFG_PTCI* = USB_OTG_GUSBCFG_PTCI_Msk
  USB_OTG_GUSBCFG_ULPIIPD_Pos* = (25)
  USB_OTG_GUSBCFG_ULPIIPD_Msk* = (0x00000001 shl USB_OTG_GUSBCFG_ULPIIPD_Pos) ## !< 0x02000000
  USB_OTG_GUSBCFG_ULPIIPD* = USB_OTG_GUSBCFG_ULPIIPD_Msk
  USB_OTG_GUSBCFG_FHMOD_Pos* = (29)
  USB_OTG_GUSBCFG_FHMOD_Msk* = (0x00000001 shl USB_OTG_GUSBCFG_FHMOD_Pos) ## !< 0x20000000
  USB_OTG_GUSBCFG_FHMOD* = USB_OTG_GUSBCFG_FHMOD_Msk
  USB_OTG_GUSBCFG_FDMOD_Pos* = (30)
  USB_OTG_GUSBCFG_FDMOD_Msk* = (0x00000001 shl USB_OTG_GUSBCFG_FDMOD_Pos) ## !< 0x40000000
  USB_OTG_GUSBCFG_FDMOD* = USB_OTG_GUSBCFG_FDMOD_Msk
  USB_OTG_GUSBCFG_CTXPKT_Pos* = (31)
  USB_OTG_GUSBCFG_CTXPKT_Msk* = (0x00000001 shl USB_OTG_GUSBCFG_CTXPKT_Pos) ## !< 0x80000000
  USB_OTG_GUSBCFG_CTXPKT* = USB_OTG_GUSBCFG_CTXPKT_Msk

## *******************  Bit definition for USB_OTG_GRSTCTL register  *******************

const
  USB_OTG_GRSTCTL_CSRST_Pos* = (0)
  USB_OTG_GRSTCTL_CSRST_Msk* = (0x00000001 shl USB_OTG_GRSTCTL_CSRST_Pos) ## !< 0x00000001
  USB_OTG_GRSTCTL_CSRST* = USB_OTG_GRSTCTL_CSRST_Msk
  USB_OTG_GRSTCTL_HSRST_Pos* = (1)
  USB_OTG_GRSTCTL_HSRST_Msk* = (0x00000001 shl USB_OTG_GRSTCTL_HSRST_Pos) ## !< 0x00000002
  USB_OTG_GRSTCTL_HSRST* = USB_OTG_GRSTCTL_HSRST_Msk
  USB_OTG_GRSTCTL_FCRST_Pos* = (2)
  USB_OTG_GRSTCTL_FCRST_Msk* = (0x00000001 shl USB_OTG_GRSTCTL_FCRST_Pos) ## !< 0x00000004
  USB_OTG_GRSTCTL_FCRST* = USB_OTG_GRSTCTL_FCRST_Msk
  USB_OTG_GRSTCTL_RXFFLSH_Pos* = (4)
  USB_OTG_GRSTCTL_RXFFLSH_Msk* = (0x00000001 shl USB_OTG_GRSTCTL_RXFFLSH_Pos) ## !< 0x00000010
  USB_OTG_GRSTCTL_RXFFLSH* = USB_OTG_GRSTCTL_RXFFLSH_Msk
  USB_OTG_GRSTCTL_TXFFLSH_Pos* = (5)
  USB_OTG_GRSTCTL_TXFFLSH_Msk* = (0x00000001 shl USB_OTG_GRSTCTL_TXFFLSH_Pos) ## !< 0x00000020
  USB_OTG_GRSTCTL_TXFFLSH* = USB_OTG_GRSTCTL_TXFFLSH_Msk
  USB_OTG_GRSTCTL_TXFNUM_Pos* = (6)
  USB_OTG_GRSTCTL_TXFNUM_Msk* = (0x0000001F shl USB_OTG_GRSTCTL_TXFNUM_Pos) ## !< 0x000007C0
  USB_OTG_GRSTCTL_TXFNUM* = USB_OTG_GRSTCTL_TXFNUM_Msk
  USB_OTG_GRSTCTL_TXFNUM_0* = (0x00000001 shl USB_OTG_GRSTCTL_TXFNUM_Pos) ## !< 0x00000040
  USB_OTG_GRSTCTL_TXFNUM_1* = (0x00000002 shl USB_OTG_GRSTCTL_TXFNUM_Pos) ## !< 0x00000080
  USB_OTG_GRSTCTL_TXFNUM_2* = (0x00000004 shl USB_OTG_GRSTCTL_TXFNUM_Pos) ## !< 0x00000100
  USB_OTG_GRSTCTL_TXFNUM_3* = (0x00000008 shl USB_OTG_GRSTCTL_TXFNUM_Pos) ## !< 0x00000200
  USB_OTG_GRSTCTL_TXFNUM_4* = (0x00000010 shl USB_OTG_GRSTCTL_TXFNUM_Pos) ## !< 0x00000400
  USB_OTG_GRSTCTL_DMAREQ_Pos* = (30)
  USB_OTG_GRSTCTL_DMAREQ_Msk* = (0x00000001 shl USB_OTG_GRSTCTL_DMAREQ_Pos) ## !< 0x40000000
  USB_OTG_GRSTCTL_DMAREQ* = USB_OTG_GRSTCTL_DMAREQ_Msk
  USB_OTG_GRSTCTL_AHBIDL_Pos* = (31)
  USB_OTG_GRSTCTL_AHBIDL_Msk* = (0x00000001 shl USB_OTG_GRSTCTL_AHBIDL_Pos) ## !< 0x80000000
  USB_OTG_GRSTCTL_AHBIDL* = USB_OTG_GRSTCTL_AHBIDL_Msk

## *******************  Bit definition for USB_OTG_DIEPMSK register  *******************

const
  USB_OTG_DIEPMSK_XFRCM_Pos* = (0)
  USB_OTG_DIEPMSK_XFRCM_Msk* = (0x00000001 shl USB_OTG_DIEPMSK_XFRCM_Pos) ## !< 0x00000001
  USB_OTG_DIEPMSK_XFRCM* = USB_OTG_DIEPMSK_XFRCM_Msk
  USB_OTG_DIEPMSK_EPDM_Pos* = (1)
  USB_OTG_DIEPMSK_EPDM_Msk* = (0x00000001 shl USB_OTG_DIEPMSK_EPDM_Pos) ## !< 0x00000002
  USB_OTG_DIEPMSK_EPDM* = USB_OTG_DIEPMSK_EPDM_Msk
  USB_OTG_DIEPMSK_TOM_Pos* = (3)
  USB_OTG_DIEPMSK_TOM_Msk* = (0x00000001 shl USB_OTG_DIEPMSK_TOM_Pos) ## !< 0x00000008
  USB_OTG_DIEPMSK_TOM* = USB_OTG_DIEPMSK_TOM_Msk
  USB_OTG_DIEPMSK_ITTXFEMSK_Pos* = (4)
  USB_OTG_DIEPMSK_ITTXFEMSK_Msk* = (0x00000001 shl USB_OTG_DIEPMSK_ITTXFEMSK_Pos) ## !< 0x00000010
  USB_OTG_DIEPMSK_ITTXFEMSK* = USB_OTG_DIEPMSK_ITTXFEMSK_Msk
  USB_OTG_DIEPMSK_INEPNMM_Pos* = (5)
  USB_OTG_DIEPMSK_INEPNMM_Msk* = (0x00000001 shl USB_OTG_DIEPMSK_INEPNMM_Pos) ## !< 0x00000020
  USB_OTG_DIEPMSK_INEPNMM* = USB_OTG_DIEPMSK_INEPNMM_Msk
  USB_OTG_DIEPMSK_INEPNEM_Pos* = (6)
  USB_OTG_DIEPMSK_INEPNEM_Msk* = (0x00000001 shl USB_OTG_DIEPMSK_INEPNEM_Pos) ## !< 0x00000040
  USB_OTG_DIEPMSK_INEPNEM* = USB_OTG_DIEPMSK_INEPNEM_Msk
  USB_OTG_DIEPMSK_TXFURM_Pos* = (8)
  USB_OTG_DIEPMSK_TXFURM_Msk* = (0x00000001 shl USB_OTG_DIEPMSK_TXFURM_Pos) ## !< 0x00000100
  USB_OTG_DIEPMSK_TXFURM* = USB_OTG_DIEPMSK_TXFURM_Msk
  USB_OTG_DIEPMSK_BIM_Pos* = (9)
  USB_OTG_DIEPMSK_BIM_Msk* = (0x00000001 shl USB_OTG_DIEPMSK_BIM_Pos) ## !< 0x00000200
  USB_OTG_DIEPMSK_BIM* = USB_OTG_DIEPMSK_BIM_Msk

## *******************  Bit definition for USB_OTG_HPTXSTS register  *******************

const
  USB_OTG_HPTXSTS_PTXFSAVL_Pos* = (0)
  USB_OTG_HPTXSTS_PTXFSAVL_Msk* = (0x0000FFFF shl USB_OTG_HPTXSTS_PTXFSAVL_Pos) ## !< 0x0000FFFF
  USB_OTG_HPTXSTS_PTXFSAVL* = USB_OTG_HPTXSTS_PTXFSAVL_Msk
  USB_OTG_HPTXSTS_PTXQSAV_Pos* = (16)
  USB_OTG_HPTXSTS_PTXQSAV_Msk* = (0x000000FF shl USB_OTG_HPTXSTS_PTXQSAV_Pos) ## !< 0x00FF0000
  USB_OTG_HPTXSTS_PTXQSAV* = USB_OTG_HPTXSTS_PTXQSAV_Msk
  USB_OTG_HPTXSTS_PTXQSAV_0* = (0x00000001 shl USB_OTG_HPTXSTS_PTXQSAV_Pos) ## !< 0x00010000
  USB_OTG_HPTXSTS_PTXQSAV_1* = (0x00000002 shl USB_OTG_HPTXSTS_PTXQSAV_Pos) ## !< 0x00020000
  USB_OTG_HPTXSTS_PTXQSAV_2* = (0x00000004 shl USB_OTG_HPTXSTS_PTXQSAV_Pos) ## !< 0x00040000
  USB_OTG_HPTXSTS_PTXQSAV_3* = (0x00000008 shl USB_OTG_HPTXSTS_PTXQSAV_Pos) ## !< 0x00080000
  USB_OTG_HPTXSTS_PTXQSAV_4* = (0x00000010 shl USB_OTG_HPTXSTS_PTXQSAV_Pos) ## !< 0x00100000
  USB_OTG_HPTXSTS_PTXQSAV_5* = (0x00000020 shl USB_OTG_HPTXSTS_PTXQSAV_Pos) ## !< 0x00200000
  USB_OTG_HPTXSTS_PTXQSAV_6* = (0x00000040 shl USB_OTG_HPTXSTS_PTXQSAV_Pos) ## !< 0x00400000
  USB_OTG_HPTXSTS_PTXQSAV_7* = (0x00000080 shl USB_OTG_HPTXSTS_PTXQSAV_Pos) ## !< 0x00800000
  USB_OTG_HPTXSTS_PTXQTOP_Pos* = (24)
  USB_OTG_HPTXSTS_PTXQTOP_Msk* = (0x000000FF shl USB_OTG_HPTXSTS_PTXQTOP_Pos) ## !< 0xFF000000
  USB_OTG_HPTXSTS_PTXQTOP* = USB_OTG_HPTXSTS_PTXQTOP_Msk
  USB_OTG_HPTXSTS_PTXQTOP_0* = (0x00000001 shl USB_OTG_HPTXSTS_PTXQTOP_Pos) ## !< 0x01000000
  USB_OTG_HPTXSTS_PTXQTOP_1* = (0x00000002 shl USB_OTG_HPTXSTS_PTXQTOP_Pos) ## !< 0x02000000
  USB_OTG_HPTXSTS_PTXQTOP_2* = (0x00000004 shl USB_OTG_HPTXSTS_PTXQTOP_Pos) ## !< 0x04000000
  USB_OTG_HPTXSTS_PTXQTOP_3* = (0x00000008 shl USB_OTG_HPTXSTS_PTXQTOP_Pos) ## !< 0x08000000
  USB_OTG_HPTXSTS_PTXQTOP_4* = (0x00000010 shl USB_OTG_HPTXSTS_PTXQTOP_Pos) ## !< 0x10000000
  USB_OTG_HPTXSTS_PTXQTOP_5* = (0x00000020 shl USB_OTG_HPTXSTS_PTXQTOP_Pos) ## !< 0x20000000
  USB_OTG_HPTXSTS_PTXQTOP_6* = (0x00000040 shl USB_OTG_HPTXSTS_PTXQTOP_Pos) ## !< 0x40000000
  USB_OTG_HPTXSTS_PTXQTOP_7* = (0x00000080 shl USB_OTG_HPTXSTS_PTXQTOP_Pos) ## !< 0x80000000

## *******************  Bit definition for USB_OTG_HAINT register  *******************

const
  USB_OTG_HAINT_HAINT_Pos* = (0)
  USB_OTG_HAINT_HAINT_Msk* = (0x0000FFFF shl USB_OTG_HAINT_HAINT_Pos) ## !< 0x0000FFFF
  USB_OTG_HAINT_HAINT* = USB_OTG_HAINT_HAINT_Msk

## *******************  Bit definition for USB_OTG_DOEPMSK register  *******************

const
  USB_OTG_DOEPMSK_XFRCM_Pos* = (0)
  USB_OTG_DOEPMSK_XFRCM_Msk* = (0x00000001 shl USB_OTG_DOEPMSK_XFRCM_Pos) ## !< 0x00000001
  USB_OTG_DOEPMSK_XFRCM* = USB_OTG_DOEPMSK_XFRCM_Msk
  USB_OTG_DOEPMSK_EPDM_Pos* = (1)
  USB_OTG_DOEPMSK_EPDM_Msk* = (0x00000001 shl USB_OTG_DOEPMSK_EPDM_Pos) ## !< 0x00000002
  USB_OTG_DOEPMSK_EPDM* = USB_OTG_DOEPMSK_EPDM_Msk
  USB_OTG_DOEPMSK_AHBERRM_Pos* = (2)
  USB_OTG_DOEPMSK_AHBERRM_Msk* = (0x00000001 shl USB_OTG_DOEPMSK_AHBERRM_Pos) ## !< 0x00000004
  USB_OTG_DOEPMSK_AHBERRM* = USB_OTG_DOEPMSK_AHBERRM_Msk
  USB_OTG_DOEPMSK_STUPM_Pos* = (3)
  USB_OTG_DOEPMSK_STUPM_Msk* = (0x00000001 shl USB_OTG_DOEPMSK_STUPM_Pos) ## !< 0x00000008
  USB_OTG_DOEPMSK_STUPM* = USB_OTG_DOEPMSK_STUPM_Msk
  USB_OTG_DOEPMSK_OTEPDM_Pos* = (4)
  USB_OTG_DOEPMSK_OTEPDM_Msk* = (0x00000001 shl USB_OTG_DOEPMSK_OTEPDM_Pos) ## !< 0x00000010
  USB_OTG_DOEPMSK_OTEPDM* = USB_OTG_DOEPMSK_OTEPDM_Msk
  USB_OTG_DOEPMSK_OTEPSPRM_Pos* = (5)
  USB_OTG_DOEPMSK_OTEPSPRM_Msk* = (0x00000001 shl USB_OTG_DOEPMSK_OTEPSPRM_Pos) ## !< 0x00000020
  USB_OTG_DOEPMSK_OTEPSPRM* = USB_OTG_DOEPMSK_OTEPSPRM_Msk
  USB_OTG_DOEPMSK_B2BSTUP_Pos* = (6)
  USB_OTG_DOEPMSK_B2BSTUP_Msk* = (0x00000001 shl USB_OTG_DOEPMSK_B2BSTUP_Pos) ## !< 0x00000040
  USB_OTG_DOEPMSK_B2BSTUP* = USB_OTG_DOEPMSK_B2BSTUP_Msk
  USB_OTG_DOEPMSK_OPEM_Pos* = (8)
  USB_OTG_DOEPMSK_OPEM_Msk* = (0x00000001 shl USB_OTG_DOEPMSK_OPEM_Pos) ## !< 0x00000100
  USB_OTG_DOEPMSK_OPEM* = USB_OTG_DOEPMSK_OPEM_Msk
  USB_OTG_DOEPMSK_BOIM_Pos* = (9)
  USB_OTG_DOEPMSK_BOIM_Msk* = (0x00000001 shl USB_OTG_DOEPMSK_BOIM_Pos) ## !< 0x00000200
  USB_OTG_DOEPMSK_BOIM* = USB_OTG_DOEPMSK_BOIM_Msk
  USB_OTG_DOEPMSK_BERRM_Pos* = (12)
  USB_OTG_DOEPMSK_BERRM_Msk* = (0x00000001 shl USB_OTG_DOEPMSK_BERRM_Pos) ## !< 0x00001000
  USB_OTG_DOEPMSK_BERRM* = USB_OTG_DOEPMSK_BERRM_Msk
  USB_OTG_DOEPMSK_NAKM_Pos* = (13)
  USB_OTG_DOEPMSK_NAKM_Msk* = (0x00000001 shl USB_OTG_DOEPMSK_NAKM_Pos) ## !< 0x00002000
  USB_OTG_DOEPMSK_NAKM* = USB_OTG_DOEPMSK_NAKM_Msk
  USB_OTG_DOEPMSK_NYETM_Pos* = (14)
  USB_OTG_DOEPMSK_NYETM_Msk* = (0x00000001 shl USB_OTG_DOEPMSK_NYETM_Pos) ## !< 0x00004000
  USB_OTG_DOEPMSK_NYETM* = USB_OTG_DOEPMSK_NYETM_Msk

## *******************  Bit definition for USB_OTG_GINTSTS register  *******************

const
  USB_OTG_GINTSTS_CMOD_Pos* = (0)
  USB_OTG_GINTSTS_CMOD_Msk* = (0x00000001 shl USB_OTG_GINTSTS_CMOD_Pos) ## !< 0x00000001
  USB_OTG_GINTSTS_CMOD* = USB_OTG_GINTSTS_CMOD_Msk
  USB_OTG_GINTSTS_MMIS_Pos* = (1)
  USB_OTG_GINTSTS_MMIS_Msk* = (0x00000001 shl USB_OTG_GINTSTS_MMIS_Pos) ## !< 0x00000002
  USB_OTG_GINTSTS_MMIS* = USB_OTG_GINTSTS_MMIS_Msk
  USB_OTG_GINTSTS_OTGINT_Pos* = (2)
  USB_OTG_GINTSTS_OTGINT_Msk* = (0x00000001 shl USB_OTG_GINTSTS_OTGINT_Pos) ## !< 0x00000004
  USB_OTG_GINTSTS_OTGINT* = USB_OTG_GINTSTS_OTGINT_Msk
  USB_OTG_GINTSTS_SOF_Pos* = (3)
  USB_OTG_GINTSTS_SOF_Msk* = (0x00000001 shl USB_OTG_GINTSTS_SOF_Pos) ## !< 0x00000008
  USB_OTG_GINTSTS_SOF* = USB_OTG_GINTSTS_SOF_Msk
  USB_OTG_GINTSTS_RXFLVL_Pos* = (4)
  USB_OTG_GINTSTS_RXFLVL_Msk* = (0x00000001 shl USB_OTG_GINTSTS_RXFLVL_Pos) ## !< 0x00000010
  USB_OTG_GINTSTS_RXFLVL* = USB_OTG_GINTSTS_RXFLVL_Msk
  USB_OTG_GINTSTS_NPTXFE_Pos* = (5)
  USB_OTG_GINTSTS_NPTXFE_Msk* = (0x00000001 shl USB_OTG_GINTSTS_NPTXFE_Pos) ## !< 0x00000020
  USB_OTG_GINTSTS_NPTXFE* = USB_OTG_GINTSTS_NPTXFE_Msk
  USB_OTG_GINTSTS_GINAKEFF_Pos* = (6)
  USB_OTG_GINTSTS_GINAKEFF_Msk* = (0x00000001 shl USB_OTG_GINTSTS_GINAKEFF_Pos) ## !< 0x00000040
  USB_OTG_GINTSTS_GINAKEFF* = USB_OTG_GINTSTS_GINAKEFF_Msk
  USB_OTG_GINTSTS_BOUTNAKEFF_Pos* = (7)
  USB_OTG_GINTSTS_BOUTNAKEFF_Msk* = (0x00000001 shl
      USB_OTG_GINTSTS_BOUTNAKEFF_Pos) ## !< 0x00000080
  USB_OTG_GINTSTS_BOUTNAKEFF* = USB_OTG_GINTSTS_BOUTNAKEFF_Msk
  USB_OTG_GINTSTS_ESUSP_Pos* = (10)
  USB_OTG_GINTSTS_ESUSP_Msk* = (0x00000001 shl USB_OTG_GINTSTS_ESUSP_Pos) ## !< 0x00000400
  USB_OTG_GINTSTS_ESUSP* = USB_OTG_GINTSTS_ESUSP_Msk
  USB_OTG_GINTSTS_USBSUSP_Pos* = (11)
  USB_OTG_GINTSTS_USBSUSP_Msk* = (0x00000001 shl USB_OTG_GINTSTS_USBSUSP_Pos) ## !< 0x00000800
  USB_OTG_GINTSTS_USBSUSP* = USB_OTG_GINTSTS_USBSUSP_Msk
  USB_OTG_GINTSTS_USBRST_Pos* = (12)
  USB_OTG_GINTSTS_USBRST_Msk* = (0x00000001 shl USB_OTG_GINTSTS_USBRST_Pos) ## !< 0x00001000
  USB_OTG_GINTSTS_USBRST* = USB_OTG_GINTSTS_USBRST_Msk
  USB_OTG_GINTSTS_ENUMDNE_Pos* = (13)
  USB_OTG_GINTSTS_ENUMDNE_Msk* = (0x00000001 shl USB_OTG_GINTSTS_ENUMDNE_Pos) ## !< 0x00002000
  USB_OTG_GINTSTS_ENUMDNE* = USB_OTG_GINTSTS_ENUMDNE_Msk
  USB_OTG_GINTSTS_ISOODRP_Pos* = (14)
  USB_OTG_GINTSTS_ISOODRP_Msk* = (0x00000001 shl USB_OTG_GINTSTS_ISOODRP_Pos) ## !< 0x00004000
  USB_OTG_GINTSTS_ISOODRP* = USB_OTG_GINTSTS_ISOODRP_Msk
  USB_OTG_GINTSTS_EOPF_Pos* = (15)
  USB_OTG_GINTSTS_EOPF_Msk* = (0x00000001 shl USB_OTG_GINTSTS_EOPF_Pos) ## !< 0x00008000
  USB_OTG_GINTSTS_EOPF* = USB_OTG_GINTSTS_EOPF_Msk
  USB_OTG_GINTSTS_IEPINT_Pos* = (18)
  USB_OTG_GINTSTS_IEPINT_Msk* = (0x00000001 shl USB_OTG_GINTSTS_IEPINT_Pos) ## !< 0x00040000
  USB_OTG_GINTSTS_IEPINT* = USB_OTG_GINTSTS_IEPINT_Msk
  USB_OTG_GINTSTS_OEPINT_Pos* = (19)
  USB_OTG_GINTSTS_OEPINT_Msk* = (0x00000001 shl USB_OTG_GINTSTS_OEPINT_Pos) ## !< 0x00080000
  USB_OTG_GINTSTS_OEPINT* = USB_OTG_GINTSTS_OEPINT_Msk
  USB_OTG_GINTSTS_IISOIXFR_Pos* = (20)
  USB_OTG_GINTSTS_IISOIXFR_Msk* = (0x00000001 shl USB_OTG_GINTSTS_IISOIXFR_Pos) ## !< 0x00100000
  USB_OTG_GINTSTS_IISOIXFR* = USB_OTG_GINTSTS_IISOIXFR_Msk
  USB_OTG_GINTSTS_PXFR_INCOMPISOOUT_Pos* = (21)
  USB_OTG_GINTSTS_PXFR_INCOMPISOOUT_Msk* = (
    0x00000001 shl USB_OTG_GINTSTS_PXFR_INCOMPISOOUT_Pos) ## !< 0x00200000
  USB_OTG_GINTSTS_PXFR_INCOMPISOOUT* = USB_OTG_GINTSTS_PXFR_INCOMPISOOUT_Msk
  USB_OTG_GINTSTS_DATAFSUSP_Pos* = (22)
  USB_OTG_GINTSTS_DATAFSUSP_Msk* = (0x00000001 shl USB_OTG_GINTSTS_DATAFSUSP_Pos) ## !< 0x00400000
  USB_OTG_GINTSTS_DATAFSUSP* = USB_OTG_GINTSTS_DATAFSUSP_Msk
  USB_OTG_GINTSTS_HPRTINT_Pos* = (24)
  USB_OTG_GINTSTS_HPRTINT_Msk* = (0x00000001 shl USB_OTG_GINTSTS_HPRTINT_Pos) ## !< 0x01000000
  USB_OTG_GINTSTS_HPRTINT* = USB_OTG_GINTSTS_HPRTINT_Msk
  USB_OTG_GINTSTS_HCINT_Pos* = (25)
  USB_OTG_GINTSTS_HCINT_Msk* = (0x00000001 shl USB_OTG_GINTSTS_HCINT_Pos) ## !< 0x02000000
  USB_OTG_GINTSTS_HCINT* = USB_OTG_GINTSTS_HCINT_Msk
  USB_OTG_GINTSTS_PTXFE_Pos* = (26)
  USB_OTG_GINTSTS_PTXFE_Msk* = (0x00000001 shl USB_OTG_GINTSTS_PTXFE_Pos) ## !< 0x04000000
  USB_OTG_GINTSTS_PTXFE* = USB_OTG_GINTSTS_PTXFE_Msk
  USB_OTG_GINTSTS_CIDSCHG_Pos* = (28)
  USB_OTG_GINTSTS_CIDSCHG_Msk* = (0x00000001 shl USB_OTG_GINTSTS_CIDSCHG_Pos) ## !< 0x10000000
  USB_OTG_GINTSTS_CIDSCHG* = USB_OTG_GINTSTS_CIDSCHG_Msk
  USB_OTG_GINTSTS_DISCINT_Pos* = (29)
  USB_OTG_GINTSTS_DISCINT_Msk* = (0x00000001 shl USB_OTG_GINTSTS_DISCINT_Pos) ## !< 0x20000000
  USB_OTG_GINTSTS_DISCINT* = USB_OTG_GINTSTS_DISCINT_Msk
  USB_OTG_GINTSTS_SRQINT_Pos* = (30)
  USB_OTG_GINTSTS_SRQINT_Msk* = (0x00000001 shl USB_OTG_GINTSTS_SRQINT_Pos) ## !< 0x40000000
  USB_OTG_GINTSTS_SRQINT* = USB_OTG_GINTSTS_SRQINT_Msk
  USB_OTG_GINTSTS_WKUINT_Pos* = (31)
  USB_OTG_GINTSTS_WKUINT_Msk* = (0x00000001 shl USB_OTG_GINTSTS_WKUINT_Pos) ## !< 0x80000000
  USB_OTG_GINTSTS_WKUINT* = USB_OTG_GINTSTS_WKUINT_Msk

## *******************  Bit definition for USB_OTG_GINTMSK register  *******************

const
  USB_OTG_GINTMSK_MMISM_Pos* = (1)
  USB_OTG_GINTMSK_MMISM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_MMISM_Pos) ## !< 0x00000002
  USB_OTG_GINTMSK_MMISM* = USB_OTG_GINTMSK_MMISM_Msk
  USB_OTG_GINTMSK_OTGINT_Pos* = (2)
  USB_OTG_GINTMSK_OTGINT_Msk* = (0x00000001 shl USB_OTG_GINTMSK_OTGINT_Pos) ## !< 0x00000004
  USB_OTG_GINTMSK_OTGINT* = USB_OTG_GINTMSK_OTGINT_Msk
  USB_OTG_GINTMSK_SOFM_Pos* = (3)
  USB_OTG_GINTMSK_SOFM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_SOFM_Pos) ## !< 0x00000008
  USB_OTG_GINTMSK_SOFM* = USB_OTG_GINTMSK_SOFM_Msk
  USB_OTG_GINTMSK_RXFLVLM_Pos* = (4)
  USB_OTG_GINTMSK_RXFLVLM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_RXFLVLM_Pos) ## !< 0x00000010
  USB_OTG_GINTMSK_RXFLVLM* = USB_OTG_GINTMSK_RXFLVLM_Msk
  USB_OTG_GINTMSK_NPTXFEM_Pos* = (5)
  USB_OTG_GINTMSK_NPTXFEM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_NPTXFEM_Pos) ## !< 0x00000020
  USB_OTG_GINTMSK_NPTXFEM* = USB_OTG_GINTMSK_NPTXFEM_Msk
  USB_OTG_GINTMSK_GINAKEFFM_Pos* = (6)
  USB_OTG_GINTMSK_GINAKEFFM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_GINAKEFFM_Pos) ## !< 0x00000040
  USB_OTG_GINTMSK_GINAKEFFM* = USB_OTG_GINTMSK_GINAKEFFM_Msk
  USB_OTG_GINTMSK_GONAKEFFM_Pos* = (7)
  USB_OTG_GINTMSK_GONAKEFFM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_GONAKEFFM_Pos) ## !< 0x00000080
  USB_OTG_GINTMSK_GONAKEFFM* = USB_OTG_GINTMSK_GONAKEFFM_Msk
  USB_OTG_GINTMSK_ESUSPM_Pos* = (10)
  USB_OTG_GINTMSK_ESUSPM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_ESUSPM_Pos) ## !< 0x00000400
  USB_OTG_GINTMSK_ESUSPM* = USB_OTG_GINTMSK_ESUSPM_Msk
  USB_OTG_GINTMSK_USBSUSPM_Pos* = (11)
  USB_OTG_GINTMSK_USBSUSPM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_USBSUSPM_Pos) ## !< 0x00000800
  USB_OTG_GINTMSK_USBSUSPM* = USB_OTG_GINTMSK_USBSUSPM_Msk
  USB_OTG_GINTMSK_USBRST_Pos* = (12)
  USB_OTG_GINTMSK_USBRST_Msk* = (0x00000001 shl USB_OTG_GINTMSK_USBRST_Pos) ## !< 0x00001000
  USB_OTG_GINTMSK_USBRST* = USB_OTG_GINTMSK_USBRST_Msk
  USB_OTG_GINTMSK_ENUMDNEM_Pos* = (13)
  USB_OTG_GINTMSK_ENUMDNEM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_ENUMDNEM_Pos) ## !< 0x00002000
  USB_OTG_GINTMSK_ENUMDNEM* = USB_OTG_GINTMSK_ENUMDNEM_Msk
  USB_OTG_GINTMSK_ISOODRPM_Pos* = (14)
  USB_OTG_GINTMSK_ISOODRPM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_ISOODRPM_Pos) ## !< 0x00004000
  USB_OTG_GINTMSK_ISOODRPM* = USB_OTG_GINTMSK_ISOODRPM_Msk
  USB_OTG_GINTMSK_EOPFM_Pos* = (15)
  USB_OTG_GINTMSK_EOPFM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_EOPFM_Pos) ## !< 0x00008000
  USB_OTG_GINTMSK_EOPFM* = USB_OTG_GINTMSK_EOPFM_Msk
  USB_OTG_GINTMSK_EPMISM_Pos* = (17)
  USB_OTG_GINTMSK_EPMISM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_EPMISM_Pos) ## !< 0x00020000
  USB_OTG_GINTMSK_EPMISM* = USB_OTG_GINTMSK_EPMISM_Msk
  USB_OTG_GINTMSK_IEPINT_Pos* = (18)
  USB_OTG_GINTMSK_IEPINT_Msk* = (0x00000001 shl USB_OTG_GINTMSK_IEPINT_Pos) ## !< 0x00040000
  USB_OTG_GINTMSK_IEPINT* = USB_OTG_GINTMSK_IEPINT_Msk
  USB_OTG_GINTMSK_OEPINT_Pos* = (19)
  USB_OTG_GINTMSK_OEPINT_Msk* = (0x00000001 shl USB_OTG_GINTMSK_OEPINT_Pos) ## !< 0x00080000
  USB_OTG_GINTMSK_OEPINT* = USB_OTG_GINTMSK_OEPINT_Msk
  USB_OTG_GINTMSK_IISOIXFRM_Pos* = (20)
  USB_OTG_GINTMSK_IISOIXFRM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_IISOIXFRM_Pos) ## !< 0x00100000
  USB_OTG_GINTMSK_IISOIXFRM* = USB_OTG_GINTMSK_IISOIXFRM_Msk
  USB_OTG_GINTMSK_PXFRM_IISOOXFRM_Pos* = (21)
  USB_OTG_GINTMSK_PXFRM_IISOOXFRM_Msk* = (
    0x00000001 shl USB_OTG_GINTMSK_PXFRM_IISOOXFRM_Pos) ## !< 0x00200000
  USB_OTG_GINTMSK_PXFRM_IISOOXFRM* = USB_OTG_GINTMSK_PXFRM_IISOOXFRM_Msk
  USB_OTG_GINTMSK_FSUSPM_Pos* = (22)
  USB_OTG_GINTMSK_FSUSPM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_FSUSPM_Pos) ## !< 0x00400000
  USB_OTG_GINTMSK_FSUSPM* = USB_OTG_GINTMSK_FSUSPM_Msk
  USB_OTG_GINTMSK_PRTIM_Pos* = (24)
  USB_OTG_GINTMSK_PRTIM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_PRTIM_Pos) ## !< 0x01000000
  USB_OTG_GINTMSK_PRTIM* = USB_OTG_GINTMSK_PRTIM_Msk
  USB_OTG_GINTMSK_HCIM_Pos* = (25)
  USB_OTG_GINTMSK_HCIM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_HCIM_Pos) ## !< 0x02000000
  USB_OTG_GINTMSK_HCIM* = USB_OTG_GINTMSK_HCIM_Msk
  USB_OTG_GINTMSK_PTXFEM_Pos* = (26)
  USB_OTG_GINTMSK_PTXFEM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_PTXFEM_Pos) ## !< 0x04000000
  USB_OTG_GINTMSK_PTXFEM* = USB_OTG_GINTMSK_PTXFEM_Msk
  USB_OTG_GINTMSK_CIDSCHGM_Pos* = (28)
  USB_OTG_GINTMSK_CIDSCHGM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_CIDSCHGM_Pos) ## !< 0x10000000
  USB_OTG_GINTMSK_CIDSCHGM* = USB_OTG_GINTMSK_CIDSCHGM_Msk
  USB_OTG_GINTMSK_DISCINT_Pos* = (29)
  USB_OTG_GINTMSK_DISCINT_Msk* = (0x00000001 shl USB_OTG_GINTMSK_DISCINT_Pos) ## !< 0x20000000
  USB_OTG_GINTMSK_DISCINT* = USB_OTG_GINTMSK_DISCINT_Msk
  USB_OTG_GINTMSK_SRQIM_Pos* = (30)
  USB_OTG_GINTMSK_SRQIM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_SRQIM_Pos) ## !< 0x40000000
  USB_OTG_GINTMSK_SRQIM* = USB_OTG_GINTMSK_SRQIM_Msk
  USB_OTG_GINTMSK_WUIM_Pos* = (31)
  USB_OTG_GINTMSK_WUIM_Msk* = (0x00000001 shl USB_OTG_GINTMSK_WUIM_Pos) ## !< 0x80000000
  USB_OTG_GINTMSK_WUIM* = USB_OTG_GINTMSK_WUIM_Msk

## *******************  Bit definition for USB_OTG_DAINT register  *******************

const
  USB_OTG_DAINT_IEPINT_Pos* = (0)
  USB_OTG_DAINT_IEPINT_Msk* = (0x0000FFFF shl USB_OTG_DAINT_IEPINT_Pos) ## !< 0x0000FFFF
  USB_OTG_DAINT_IEPINT* = USB_OTG_DAINT_IEPINT_Msk
  USB_OTG_DAINT_OEPINT_Pos* = (16)
  USB_OTG_DAINT_OEPINT_Msk* = (0x0000FFFF shl USB_OTG_DAINT_OEPINT_Pos) ## !< 0xFFFF0000
  USB_OTG_DAINT_OEPINT* = USB_OTG_DAINT_OEPINT_Msk

## *******************  Bit definition for USB_OTG_HAINTMSK register  *******************

const
  USB_OTG_HAINTMSK_HAINTM_Pos* = (0)
  USB_OTG_HAINTMSK_HAINTM_Msk* = (0x0000FFFF shl USB_OTG_HAINTMSK_HAINTM_Pos) ## !< 0x0000FFFF
  USB_OTG_HAINTMSK_HAINTM* = USB_OTG_HAINTMSK_HAINTM_Msk

## *******************  Bit definition for USB_OTG_GRXSTSP register  *******************

const
  USB_OTG_GRXSTSP_EPNUM_Pos* = (0)
  USB_OTG_GRXSTSP_EPNUM_Msk* = (0x0000000F shl USB_OTG_GRXSTSP_EPNUM_Pos) ## !< 0x0000000F
  USB_OTG_GRXSTSP_EPNUM* = USB_OTG_GRXSTSP_EPNUM_Msk
  USB_OTG_GRXSTSP_BCNT_Pos* = (4)
  USB_OTG_GRXSTSP_BCNT_Msk* = (0x000007FF shl USB_OTG_GRXSTSP_BCNT_Pos) ## !< 0x00007FF0
  USB_OTG_GRXSTSP_BCNT* = USB_OTG_GRXSTSP_BCNT_Msk
  USB_OTG_GRXSTSP_DPID_Pos* = (15)
  USB_OTG_GRXSTSP_DPID_Msk* = (0x00000003 shl USB_OTG_GRXSTSP_DPID_Pos) ## !< 0x00018000
  USB_OTG_GRXSTSP_DPID* = USB_OTG_GRXSTSP_DPID_Msk
  USB_OTG_GRXSTSP_PKTSTS_Pos* = (17)
  USB_OTG_GRXSTSP_PKTSTS_Msk* = (0x0000000F shl USB_OTG_GRXSTSP_PKTSTS_Pos) ## !< 0x001E0000
  USB_OTG_GRXSTSP_PKTSTS* = USB_OTG_GRXSTSP_PKTSTS_Msk

## *******************  Bit definition for USB_OTG_DAINTMSK register  *******************

const
  USB_OTG_DAINTMSK_IEPM_Pos* = (0)
  USB_OTG_DAINTMSK_IEPM_Msk* = (0x0000FFFF shl USB_OTG_DAINTMSK_IEPM_Pos) ## !< 0x0000FFFF
  USB_OTG_DAINTMSK_IEPM* = USB_OTG_DAINTMSK_IEPM_Msk
  USB_OTG_DAINTMSK_OEPM_Pos* = (16)
  USB_OTG_DAINTMSK_OEPM_Msk* = (0x0000FFFF shl USB_OTG_DAINTMSK_OEPM_Pos) ## !< 0xFFFF0000
  USB_OTG_DAINTMSK_OEPM* = USB_OTG_DAINTMSK_OEPM_Msk

## *******************  Bit definition for USB_OTG_GRXFSIZ register  *******************

const
  USB_OTG_GRXFSIZ_RXFD_Pos* = (0)
  USB_OTG_GRXFSIZ_RXFD_Msk* = (0x0000FFFF shl USB_OTG_GRXFSIZ_RXFD_Pos) ## !< 0x0000FFFF
  USB_OTG_GRXFSIZ_RXFD* = USB_OTG_GRXFSIZ_RXFD_Msk

## *******************  Bit definition for USB_OTG_DVBUSDIS register  *******************

const
  USB_OTG_DVBUSDIS_VBUSDT_Pos* = (0)
  USB_OTG_DVBUSDIS_VBUSDT_Msk* = (0x0000FFFF shl USB_OTG_DVBUSDIS_VBUSDT_Pos) ## !< 0x0000FFFF
  USB_OTG_DVBUSDIS_VBUSDT* = USB_OTG_DVBUSDIS_VBUSDT_Msk

## *******************  Bit definition for OTG register  *******************

const
  USB_OTG_NPTXFSA_Pos* = (0)
  USB_OTG_NPTXFSA_Msk* = (0x0000FFFF shl USB_OTG_NPTXFSA_Pos) ## !< 0x0000FFFF
  USB_OTG_NPTXFSA* = USB_OTG_NPTXFSA_Msk
  USB_OTG_NPTXFD_Pos* = (16)
  USB_OTG_NPTXFD_Msk* = (0x0000FFFF shl USB_OTG_NPTXFD_Pos) ## !< 0xFFFF0000
  USB_OTG_NPTXFD* = USB_OTG_NPTXFD_Msk
  USB_OTG_TX0FSA_Pos* = (0)
  USB_OTG_TX0FSA_Msk* = (0x0000FFFF shl USB_OTG_TX0FSA_Pos) ## !< 0x0000FFFF
  USB_OTG_TX0FSA* = USB_OTG_TX0FSA_Msk
  USB_OTG_TX0FD_Pos* = (16)
  USB_OTG_TX0FD_Msk* = (0x0000FFFF shl USB_OTG_TX0FD_Pos) ## !< 0xFFFF0000
  USB_OTG_TX0FD* = USB_OTG_TX0FD_Msk

## *******************  Bit definition forUSB_OTG_DVBUSPULSE register  *******************

const
  USB_OTG_DVBUSPULSE_DVBUSP_Pos* = (0)
  USB_OTG_DVBUSPULSE_DVBUSP_Msk* = (0x00000FFF shl USB_OTG_DVBUSPULSE_DVBUSP_Pos) ## !< 0x00000FFF
  USB_OTG_DVBUSPULSE_DVBUSP* = USB_OTG_DVBUSPULSE_DVBUSP_Msk

## *******************  Bit definition for USB_OTG_GNPTXSTS register  *******************

const
  USB_OTG_GNPTXSTS_NPTXFSAV_Pos* = (0)
  USB_OTG_GNPTXSTS_NPTXFSAV_Msk* = (0x0000FFFF shl USB_OTG_GNPTXSTS_NPTXFSAV_Pos) ## !< 0x0000FFFF
  USB_OTG_GNPTXSTS_NPTXFSAV* = USB_OTG_GNPTXSTS_NPTXFSAV_Msk
  USB_OTG_GNPTXSTS_NPTQXSAV_Pos* = (16)
  USB_OTG_GNPTXSTS_NPTQXSAV_Msk* = (0x000000FF shl USB_OTG_GNPTXSTS_NPTQXSAV_Pos) ## !< 0x00FF0000
  USB_OTG_GNPTXSTS_NPTQXSAV* = USB_OTG_GNPTXSTS_NPTQXSAV_Msk
  USB_OTG_GNPTXSTS_NPTQXSAV_0* = (0x00000001 shl USB_OTG_GNPTXSTS_NPTQXSAV_Pos) ## !< 0x00010000
  USB_OTG_GNPTXSTS_NPTQXSAV_1* = (0x00000002 shl USB_OTG_GNPTXSTS_NPTQXSAV_Pos) ## !< 0x00020000
  USB_OTG_GNPTXSTS_NPTQXSAV_2* = (0x00000004 shl USB_OTG_GNPTXSTS_NPTQXSAV_Pos) ## !< 0x00040000
  USB_OTG_GNPTXSTS_NPTQXSAV_3* = (0x00000008 shl USB_OTG_GNPTXSTS_NPTQXSAV_Pos) ## !< 0x00080000
  USB_OTG_GNPTXSTS_NPTQXSAV_4* = (0x00000010 shl USB_OTG_GNPTXSTS_NPTQXSAV_Pos) ## !< 0x00100000
  USB_OTG_GNPTXSTS_NPTQXSAV_5* = (0x00000020 shl USB_OTG_GNPTXSTS_NPTQXSAV_Pos) ## !< 0x00200000
  USB_OTG_GNPTXSTS_NPTQXSAV_6* = (0x00000040 shl USB_OTG_GNPTXSTS_NPTQXSAV_Pos) ## !< 0x00400000
  USB_OTG_GNPTXSTS_NPTQXSAV_7* = (0x00000080 shl USB_OTG_GNPTXSTS_NPTQXSAV_Pos) ## !< 0x00800000
  USB_OTG_GNPTXSTS_NPTXQTOP_Pos* = (24)
  USB_OTG_GNPTXSTS_NPTXQTOP_Msk* = (0x0000007F shl USB_OTG_GNPTXSTS_NPTXQTOP_Pos) ## !< 0x7F000000
  USB_OTG_GNPTXSTS_NPTXQTOP* = USB_OTG_GNPTXSTS_NPTXQTOP_Msk
  USB_OTG_GNPTXSTS_NPTXQTOP_0* = (0x00000001 shl USB_OTG_GNPTXSTS_NPTXQTOP_Pos) ## !< 0x01000000
  USB_OTG_GNPTXSTS_NPTXQTOP_1* = (0x00000002 shl USB_OTG_GNPTXSTS_NPTXQTOP_Pos) ## !< 0x02000000
  USB_OTG_GNPTXSTS_NPTXQTOP_2* = (0x00000004 shl USB_OTG_GNPTXSTS_NPTXQTOP_Pos) ## !< 0x04000000
  USB_OTG_GNPTXSTS_NPTXQTOP_3* = (0x00000008 shl USB_OTG_GNPTXSTS_NPTXQTOP_Pos) ## !< 0x08000000
  USB_OTG_GNPTXSTS_NPTXQTOP_4* = (0x00000010 shl USB_OTG_GNPTXSTS_NPTXQTOP_Pos) ## !< 0x10000000
  USB_OTG_GNPTXSTS_NPTXQTOP_5* = (0x00000020 shl USB_OTG_GNPTXSTS_NPTXQTOP_Pos) ## !< 0x20000000
  USB_OTG_GNPTXSTS_NPTXQTOP_6* = (0x00000040 shl USB_OTG_GNPTXSTS_NPTXQTOP_Pos) ## !< 0x40000000

## *******************  Bit definition for USB_OTG_DTHRCTL register  *******************

const
  USB_OTG_DTHRCTL_NONISOTHREN_Pos* = (0)
  USB_OTG_DTHRCTL_NONISOTHREN_Msk* = (
    0x00000001 shl USB_OTG_DTHRCTL_NONISOTHREN_Pos) ## !< 0x00000001
  USB_OTG_DTHRCTL_NONISOTHREN* = USB_OTG_DTHRCTL_NONISOTHREN_Msk
  USB_OTG_DTHRCTL_ISOTHREN_Pos* = (1)
  USB_OTG_DTHRCTL_ISOTHREN_Msk* = (0x00000001 shl USB_OTG_DTHRCTL_ISOTHREN_Pos) ## !< 0x00000002
  USB_OTG_DTHRCTL_ISOTHREN* = USB_OTG_DTHRCTL_ISOTHREN_Msk
  USB_OTG_DTHRCTL_TXTHRLEN_Pos* = (2)
  USB_OTG_DTHRCTL_TXTHRLEN_Msk* = (0x000001FF shl USB_OTG_DTHRCTL_TXTHRLEN_Pos) ## !< 0x000007FC
  USB_OTG_DTHRCTL_TXTHRLEN* = USB_OTG_DTHRCTL_TXTHRLEN_Msk
  USB_OTG_DTHRCTL_TXTHRLEN_0* = (0x00000001 shl USB_OTG_DTHRCTL_TXTHRLEN_Pos) ## !< 0x00000004
  USB_OTG_DTHRCTL_TXTHRLEN_1* = (0x00000002 shl USB_OTG_DTHRCTL_TXTHRLEN_Pos) ## !< 0x00000008
  USB_OTG_DTHRCTL_TXTHRLEN_2* = (0x00000004 shl USB_OTG_DTHRCTL_TXTHRLEN_Pos) ## !< 0x00000010
  USB_OTG_DTHRCTL_TXTHRLEN_3* = (0x00000008 shl USB_OTG_DTHRCTL_TXTHRLEN_Pos) ## !< 0x00000020
  USB_OTG_DTHRCTL_TXTHRLEN_4* = (0x00000010 shl USB_OTG_DTHRCTL_TXTHRLEN_Pos) ## !< 0x00000040
  USB_OTG_DTHRCTL_TXTHRLEN_5* = (0x00000020 shl USB_OTG_DTHRCTL_TXTHRLEN_Pos) ## !< 0x00000080
  USB_OTG_DTHRCTL_TXTHRLEN_6* = (0x00000040 shl USB_OTG_DTHRCTL_TXTHRLEN_Pos) ## !< 0x00000100
  USB_OTG_DTHRCTL_TXTHRLEN_7* = (0x00000080 shl USB_OTG_DTHRCTL_TXTHRLEN_Pos) ## !< 0x00000200
  USB_OTG_DTHRCTL_TXTHRLEN_8* = (0x00000100 shl USB_OTG_DTHRCTL_TXTHRLEN_Pos) ## !< 0x00000400
  USB_OTG_DTHRCTL_RXTHREN_Pos* = (16)
  USB_OTG_DTHRCTL_RXTHREN_Msk* = (0x00000001 shl USB_OTG_DTHRCTL_RXTHREN_Pos) ## !< 0x00010000
  USB_OTG_DTHRCTL_RXTHREN* = USB_OTG_DTHRCTL_RXTHREN_Msk
  USB_OTG_DTHRCTL_RXTHRLEN_Pos* = (17)
  USB_OTG_DTHRCTL_RXTHRLEN_Msk* = (0x000001FF shl USB_OTG_DTHRCTL_RXTHRLEN_Pos) ## !< 0x03FE0000
  USB_OTG_DTHRCTL_RXTHRLEN* = USB_OTG_DTHRCTL_RXTHRLEN_Msk
  USB_OTG_DTHRCTL_RXTHRLEN_0* = (0x00000001 shl USB_OTG_DTHRCTL_RXTHRLEN_Pos) ## !< 0x00020000
  USB_OTG_DTHRCTL_RXTHRLEN_1* = (0x00000002 shl USB_OTG_DTHRCTL_RXTHRLEN_Pos) ## !< 0x00040000
  USB_OTG_DTHRCTL_RXTHRLEN_2* = (0x00000004 shl USB_OTG_DTHRCTL_RXTHRLEN_Pos) ## !< 0x00080000
  USB_OTG_DTHRCTL_RXTHRLEN_3* = (0x00000008 shl USB_OTG_DTHRCTL_RXTHRLEN_Pos) ## !< 0x00100000
  USB_OTG_DTHRCTL_RXTHRLEN_4* = (0x00000010 shl USB_OTG_DTHRCTL_RXTHRLEN_Pos) ## !< 0x00200000
  USB_OTG_DTHRCTL_RXTHRLEN_5* = (0x00000020 shl USB_OTG_DTHRCTL_RXTHRLEN_Pos) ## !< 0x00400000
  USB_OTG_DTHRCTL_RXTHRLEN_6* = (0x00000040 shl USB_OTG_DTHRCTL_RXTHRLEN_Pos) ## !< 0x00800000
  USB_OTG_DTHRCTL_RXTHRLEN_7* = (0x00000080 shl USB_OTG_DTHRCTL_RXTHRLEN_Pos) ## !< 0x01000000
  USB_OTG_DTHRCTL_RXTHRLEN_8* = (0x00000100 shl USB_OTG_DTHRCTL_RXTHRLEN_Pos) ## !< 0x02000000
  USB_OTG_DTHRCTL_ARPEN_Pos* = (27)
  USB_OTG_DTHRCTL_ARPEN_Msk* = (0x00000001 shl USB_OTG_DTHRCTL_ARPEN_Pos) ## !< 0x08000000
  USB_OTG_DTHRCTL_ARPEN* = USB_OTG_DTHRCTL_ARPEN_Msk

## *******************  Bit definition for USB_OTG_DIEPEMPMSK register  *******************

const
  USB_OTG_DIEPEMPMSK_INEPTXFEM_Pos* = (0)
  USB_OTG_DIEPEMPMSK_INEPTXFEM_Msk* = (
    0x0000FFFF shl USB_OTG_DIEPEMPMSK_INEPTXFEM_Pos) ## !< 0x0000FFFF
  USB_OTG_DIEPEMPMSK_INEPTXFEM* = USB_OTG_DIEPEMPMSK_INEPTXFEM_Msk

## *******************  Bit definition for USB_OTG_DEACHINT register  *******************

const
  USB_OTG_DEACHINT_IEP1INT_Pos* = (1)
  USB_OTG_DEACHINT_IEP1INT_Msk* = (0x00000001 shl USB_OTG_DEACHINT_IEP1INT_Pos) ## !< 0x00000002
  USB_OTG_DEACHINT_IEP1INT* = USB_OTG_DEACHINT_IEP1INT_Msk
  USB_OTG_DEACHINT_OEP1INT_Pos* = (17)
  USB_OTG_DEACHINT_OEP1INT_Msk* = (0x00000001 shl USB_OTG_DEACHINT_OEP1INT_Pos) ## !< 0x00020000
  USB_OTG_DEACHINT_OEP1INT* = USB_OTG_DEACHINT_OEP1INT_Msk

## *******************  Bit definition for USB_OTG_GCCFG register  *******************

const
  USB_OTG_GCCFG_PWRDWN_Pos* = (16)
  USB_OTG_GCCFG_PWRDWN_Msk* = (0x00000001 shl USB_OTG_GCCFG_PWRDWN_Pos) ## !< 0x00010000
  USB_OTG_GCCFG_PWRDWN* = USB_OTG_GCCFG_PWRDWN_Msk
  USB_OTG_GCCFG_I2CPADEN_Pos* = (17)
  USB_OTG_GCCFG_I2CPADEN_Msk* = (0x00000001 shl USB_OTG_GCCFG_I2CPADEN_Pos) ## !< 0x00020000
  USB_OTG_GCCFG_I2CPADEN* = USB_OTG_GCCFG_I2CPADEN_Msk
  USB_OTG_GCCFG_VBUSASEN_Pos* = (18)
  USB_OTG_GCCFG_VBUSASEN_Msk* = (0x00000001 shl USB_OTG_GCCFG_VBUSASEN_Pos) ## !< 0x00040000
  USB_OTG_GCCFG_VBUSASEN* = USB_OTG_GCCFG_VBUSASEN_Msk
  USB_OTG_GCCFG_VBUSBSEN_Pos* = (19)
  USB_OTG_GCCFG_VBUSBSEN_Msk* = (0x00000001 shl USB_OTG_GCCFG_VBUSBSEN_Pos) ## !< 0x00080000
  USB_OTG_GCCFG_VBUSBSEN* = USB_OTG_GCCFG_VBUSBSEN_Msk
  USB_OTG_GCCFG_SOFOUTEN_Pos* = (20)
  USB_OTG_GCCFG_SOFOUTEN_Msk* = (0x00000001 shl USB_OTG_GCCFG_SOFOUTEN_Pos) ## !< 0x00100000
  USB_OTG_GCCFG_SOFOUTEN* = USB_OTG_GCCFG_SOFOUTEN_Msk
  USB_OTG_GCCFG_NOVBUSSENS_Pos* = (21)
  USB_OTG_GCCFG_NOVBUSSENS_Msk* = (0x00000001 shl USB_OTG_GCCFG_NOVBUSSENS_Pos) ## !< 0x00200000
  USB_OTG_GCCFG_NOVBUSSENS* = USB_OTG_GCCFG_NOVBUSSENS_Msk

## *******************  Bit definition forUSB_OTG_DEACHINTMSK register  *******************

const
  USB_OTG_DEACHINTMSK_IEP1INTM_Pos* = (1)
  USB_OTG_DEACHINTMSK_IEP1INTM_Msk* = (
    0x00000001 shl USB_OTG_DEACHINTMSK_IEP1INTM_Pos) ## !< 0x00000002
  USB_OTG_DEACHINTMSK_IEP1INTM* = USB_OTG_DEACHINTMSK_IEP1INTM_Msk
  USB_OTG_DEACHINTMSK_OEP1INTM_Pos* = (17)
  USB_OTG_DEACHINTMSK_OEP1INTM_Msk* = (
    0x00000001 shl USB_OTG_DEACHINTMSK_OEP1INTM_Pos) ## !< 0x00020000
  USB_OTG_DEACHINTMSK_OEP1INTM* = USB_OTG_DEACHINTMSK_OEP1INTM_Msk

## *******************  Bit definition for USB_OTG_CID register  *******************

const
  USB_OTG_CID_PRODUCT_ID_Pos* = (0)
  USB_OTG_CID_PRODUCT_ID_Msk* = (0xFFFFFFFF shl USB_OTG_CID_PRODUCT_ID_Pos) ## !< 0xFFFFFFFF
  USB_OTG_CID_PRODUCT_ID* = USB_OTG_CID_PRODUCT_ID_Msk

## *******************  Bit definition for USB_OTG_DIEPEACHMSK1 register  *******************

const
  USB_OTG_DIEPEACHMSK1_XFRCM_Pos* = (0)
  USB_OTG_DIEPEACHMSK1_XFRCM_Msk* = (0x00000001 shl
      USB_OTG_DIEPEACHMSK1_XFRCM_Pos) ## !< 0x00000001
  USB_OTG_DIEPEACHMSK1_XFRCM* = USB_OTG_DIEPEACHMSK1_XFRCM_Msk
  USB_OTG_DIEPEACHMSK1_EPDM_Pos* = (1)
  USB_OTG_DIEPEACHMSK1_EPDM_Msk* = (0x00000001 shl USB_OTG_DIEPEACHMSK1_EPDM_Pos) ## !< 0x00000002
  USB_OTG_DIEPEACHMSK1_EPDM* = USB_OTG_DIEPEACHMSK1_EPDM_Msk
  USB_OTG_DIEPEACHMSK1_TOM_Pos* = (3)
  USB_OTG_DIEPEACHMSK1_TOM_Msk* = (0x00000001 shl USB_OTG_DIEPEACHMSK1_TOM_Pos) ## !< 0x00000008
  USB_OTG_DIEPEACHMSK1_TOM* = USB_OTG_DIEPEACHMSK1_TOM_Msk
  USB_OTG_DIEPEACHMSK1_ITTXFEMSK_Pos* = (4)
  USB_OTG_DIEPEACHMSK1_ITTXFEMSK_Msk* = (
    0x00000001 shl USB_OTG_DIEPEACHMSK1_ITTXFEMSK_Pos) ## !< 0x00000010
  USB_OTG_DIEPEACHMSK1_ITTXFEMSK* = USB_OTG_DIEPEACHMSK1_ITTXFEMSK_Msk
  USB_OTG_DIEPEACHMSK1_INEPNMM_Pos* = (5)
  USB_OTG_DIEPEACHMSK1_INEPNMM_Msk* = (
    0x00000001 shl USB_OTG_DIEPEACHMSK1_INEPNMM_Pos) ## !< 0x00000020
  USB_OTG_DIEPEACHMSK1_INEPNMM* = USB_OTG_DIEPEACHMSK1_INEPNMM_Msk
  USB_OTG_DIEPEACHMSK1_INEPNEM_Pos* = (6)
  USB_OTG_DIEPEACHMSK1_INEPNEM_Msk* = (
    0x00000001 shl USB_OTG_DIEPEACHMSK1_INEPNEM_Pos) ## !< 0x00000040
  USB_OTG_DIEPEACHMSK1_INEPNEM* = USB_OTG_DIEPEACHMSK1_INEPNEM_Msk
  USB_OTG_DIEPEACHMSK1_TXFURM_Pos* = (8)
  USB_OTG_DIEPEACHMSK1_TXFURM_Msk* = (
    0x00000001 shl USB_OTG_DIEPEACHMSK1_TXFURM_Pos) ## !< 0x00000100
  USB_OTG_DIEPEACHMSK1_TXFURM* = USB_OTG_DIEPEACHMSK1_TXFURM_Msk
  USB_OTG_DIEPEACHMSK1_BIM_Pos* = (9)
  USB_OTG_DIEPEACHMSK1_BIM_Msk* = (0x00000001 shl USB_OTG_DIEPEACHMSK1_BIM_Pos) ## !< 0x00000200
  USB_OTG_DIEPEACHMSK1_BIM* = USB_OTG_DIEPEACHMSK1_BIM_Msk
  USB_OTG_DIEPEACHMSK1_NAKM_Pos* = (13)
  USB_OTG_DIEPEACHMSK1_NAKM_Msk* = (0x00000001 shl USB_OTG_DIEPEACHMSK1_NAKM_Pos) ## !< 0x00002000
  USB_OTG_DIEPEACHMSK1_NAKM* = USB_OTG_DIEPEACHMSK1_NAKM_Msk

## *******************  Bit definition for USB_OTG_HPRT register  *******************

const
  USB_OTG_HPRT_PCSTS_Pos* = (0)
  USB_OTG_HPRT_PCSTS_Msk* = (0x00000001 shl USB_OTG_HPRT_PCSTS_Pos) ## !< 0x00000001
  USB_OTG_HPRT_PCSTS* = USB_OTG_HPRT_PCSTS_Msk
  USB_OTG_HPRT_PCDET_Pos* = (1)
  USB_OTG_HPRT_PCDET_Msk* = (0x00000001 shl USB_OTG_HPRT_PCDET_Pos) ## !< 0x00000002
  USB_OTG_HPRT_PCDET* = USB_OTG_HPRT_PCDET_Msk
  USB_OTG_HPRT_PENA_Pos* = (2)
  USB_OTG_HPRT_PENA_Msk* = (0x00000001 shl USB_OTG_HPRT_PENA_Pos) ## !< 0x00000004
  USB_OTG_HPRT_PENA* = USB_OTG_HPRT_PENA_Msk
  USB_OTG_HPRT_PENCHNG_Pos* = (3)
  USB_OTG_HPRT_PENCHNG_Msk* = (0x00000001 shl USB_OTG_HPRT_PENCHNG_Pos) ## !< 0x00000008
  USB_OTG_HPRT_PENCHNG* = USB_OTG_HPRT_PENCHNG_Msk
  USB_OTG_HPRT_POCA_Pos* = (4)
  USB_OTG_HPRT_POCA_Msk* = (0x00000001 shl USB_OTG_HPRT_POCA_Pos) ## !< 0x00000010
  USB_OTG_HPRT_POCA* = USB_OTG_HPRT_POCA_Msk
  USB_OTG_HPRT_POCCHNG_Pos* = (5)
  USB_OTG_HPRT_POCCHNG_Msk* = (0x00000001 shl USB_OTG_HPRT_POCCHNG_Pos) ## !< 0x00000020
  USB_OTG_HPRT_POCCHNG* = USB_OTG_HPRT_POCCHNG_Msk
  USB_OTG_HPRT_PRES_Pos* = (6)
  USB_OTG_HPRT_PRES_Msk* = (0x00000001 shl USB_OTG_HPRT_PRES_Pos) ## !< 0x00000040
  USB_OTG_HPRT_PRES* = USB_OTG_HPRT_PRES_Msk
  USB_OTG_HPRT_PSUSP_Pos* = (7)
  USB_OTG_HPRT_PSUSP_Msk* = (0x00000001 shl USB_OTG_HPRT_PSUSP_Pos) ## !< 0x00000080
  USB_OTG_HPRT_PSUSP* = USB_OTG_HPRT_PSUSP_Msk
  USB_OTG_HPRT_PRST_Pos* = (8)
  USB_OTG_HPRT_PRST_Msk* = (0x00000001 shl USB_OTG_HPRT_PRST_Pos) ## !< 0x00000100
  USB_OTG_HPRT_PRST* = USB_OTG_HPRT_PRST_Msk
  USB_OTG_HPRT_PLSTS_Pos* = (10)
  USB_OTG_HPRT_PLSTS_Msk* = (0x00000003 shl USB_OTG_HPRT_PLSTS_Pos) ## !< 0x00000C00
  USB_OTG_HPRT_PLSTS* = USB_OTG_HPRT_PLSTS_Msk
  USB_OTG_HPRT_PLSTS_0* = (0x00000001 shl USB_OTG_HPRT_PLSTS_Pos) ## !< 0x00000400
  USB_OTG_HPRT_PLSTS_1* = (0x00000002 shl USB_OTG_HPRT_PLSTS_Pos) ## !< 0x00000800
  USB_OTG_HPRT_PPWR_Pos* = (12)
  USB_OTG_HPRT_PPWR_Msk* = (0x00000001 shl USB_OTG_HPRT_PPWR_Pos) ## !< 0x00001000
  USB_OTG_HPRT_PPWR* = USB_OTG_HPRT_PPWR_Msk
  USB_OTG_HPRT_PTCTL_Pos* = (13)
  USB_OTG_HPRT_PTCTL_Msk* = (0x0000000F shl USB_OTG_HPRT_PTCTL_Pos) ## !< 0x0001E000
  USB_OTG_HPRT_PTCTL* = USB_OTG_HPRT_PTCTL_Msk
  USB_OTG_HPRT_PTCTL_0* = (0x00000001 shl USB_OTG_HPRT_PTCTL_Pos) ## !< 0x00002000
  USB_OTG_HPRT_PTCTL_1* = (0x00000002 shl USB_OTG_HPRT_PTCTL_Pos) ## !< 0x00004000
  USB_OTG_HPRT_PTCTL_2* = (0x00000004 shl USB_OTG_HPRT_PTCTL_Pos) ## !< 0x00008000
  USB_OTG_HPRT_PTCTL_3* = (0x00000008 shl USB_OTG_HPRT_PTCTL_Pos) ## !< 0x00010000
  USB_OTG_HPRT_PSPD_Pos* = (17)
  USB_OTG_HPRT_PSPD_Msk* = (0x00000003 shl USB_OTG_HPRT_PSPD_Pos) ## !< 0x00060000
  USB_OTG_HPRT_PSPD* = USB_OTG_HPRT_PSPD_Msk
  USB_OTG_HPRT_PSPD_0* = (0x00000001 shl USB_OTG_HPRT_PSPD_Pos) ## !< 0x00020000
  USB_OTG_HPRT_PSPD_1* = (0x00000002 shl USB_OTG_HPRT_PSPD_Pos) ## !< 0x00040000

## *******************  Bit definition for USB_OTG_DOEPEACHMSK1 register  *******************

const
  USB_OTG_DOEPEACHMSK1_XFRCM_Pos* = (0)
  USB_OTG_DOEPEACHMSK1_XFRCM_Msk* = (0x00000001 shl
      USB_OTG_DOEPEACHMSK1_XFRCM_Pos) ## !< 0x00000001
  USB_OTG_DOEPEACHMSK1_XFRCM* = USB_OTG_DOEPEACHMSK1_XFRCM_Msk
  USB_OTG_DOEPEACHMSK1_EPDM_Pos* = (1)
  USB_OTG_DOEPEACHMSK1_EPDM_Msk* = (0x00000001 shl USB_OTG_DOEPEACHMSK1_EPDM_Pos) ## !< 0x00000002
  USB_OTG_DOEPEACHMSK1_EPDM* = USB_OTG_DOEPEACHMSK1_EPDM_Msk
  USB_OTG_DOEPEACHMSK1_TOM_Pos* = (3)
  USB_OTG_DOEPEACHMSK1_TOM_Msk* = (0x00000001 shl USB_OTG_DOEPEACHMSK1_TOM_Pos) ## !< 0x00000008
  USB_OTG_DOEPEACHMSK1_TOM* = USB_OTG_DOEPEACHMSK1_TOM_Msk
  USB_OTG_DOEPEACHMSK1_ITTXFEMSK_Pos* = (4)
  USB_OTG_DOEPEACHMSK1_ITTXFEMSK_Msk* = (
    0x00000001 shl USB_OTG_DOEPEACHMSK1_ITTXFEMSK_Pos) ## !< 0x00000010
  USB_OTG_DOEPEACHMSK1_ITTXFEMSK* = USB_OTG_DOEPEACHMSK1_ITTXFEMSK_Msk
  USB_OTG_DOEPEACHMSK1_INEPNMM_Pos* = (5)
  USB_OTG_DOEPEACHMSK1_INEPNMM_Msk* = (
    0x00000001 shl USB_OTG_DOEPEACHMSK1_INEPNMM_Pos) ## !< 0x00000020
  USB_OTG_DOEPEACHMSK1_INEPNMM* = USB_OTG_DOEPEACHMSK1_INEPNMM_Msk
  USB_OTG_DOEPEACHMSK1_INEPNEM_Pos* = (6)
  USB_OTG_DOEPEACHMSK1_INEPNEM_Msk* = (
    0x00000001 shl USB_OTG_DOEPEACHMSK1_INEPNEM_Pos) ## !< 0x00000040
  USB_OTG_DOEPEACHMSK1_INEPNEM* = USB_OTG_DOEPEACHMSK1_INEPNEM_Msk
  USB_OTG_DOEPEACHMSK1_TXFURM_Pos* = (8)
  USB_OTG_DOEPEACHMSK1_TXFURM_Msk* = (
    0x00000001 shl USB_OTG_DOEPEACHMSK1_TXFURM_Pos) ## !< 0x00000100
  USB_OTG_DOEPEACHMSK1_TXFURM* = USB_OTG_DOEPEACHMSK1_TXFURM_Msk
  USB_OTG_DOEPEACHMSK1_BIM_Pos* = (9)
  USB_OTG_DOEPEACHMSK1_BIM_Msk* = (0x00000001 shl USB_OTG_DOEPEACHMSK1_BIM_Pos) ## !< 0x00000200
  USB_OTG_DOEPEACHMSK1_BIM* = USB_OTG_DOEPEACHMSK1_BIM_Msk
  USB_OTG_DOEPEACHMSK1_BERRM_Pos* = (12)
  USB_OTG_DOEPEACHMSK1_BERRM_Msk* = (0x00000001 shl
      USB_OTG_DOEPEACHMSK1_BERRM_Pos) ## !< 0x00001000
  USB_OTG_DOEPEACHMSK1_BERRM* = USB_OTG_DOEPEACHMSK1_BERRM_Msk
  USB_OTG_DOEPEACHMSK1_NAKM_Pos* = (13)
  USB_OTG_DOEPEACHMSK1_NAKM_Msk* = (0x00000001 shl USB_OTG_DOEPEACHMSK1_NAKM_Pos) ## !< 0x00002000
  USB_OTG_DOEPEACHMSK1_NAKM* = USB_OTG_DOEPEACHMSK1_NAKM_Msk
  USB_OTG_DOEPEACHMSK1_NYETM_Pos* = (14)
  USB_OTG_DOEPEACHMSK1_NYETM_Msk* = (0x00000001 shl
      USB_OTG_DOEPEACHMSK1_NYETM_Pos) ## !< 0x00004000
  USB_OTG_DOEPEACHMSK1_NYETM* = USB_OTG_DOEPEACHMSK1_NYETM_Msk

## *******************  Bit definition for USB_OTG_HPTXFSIZ register  *******************

const
  USB_OTG_HPTXFSIZ_PTXSA_Pos* = (0)
  USB_OTG_HPTXFSIZ_PTXSA_Msk* = (0x0000FFFF shl USB_OTG_HPTXFSIZ_PTXSA_Pos) ## !< 0x0000FFFF
  USB_OTG_HPTXFSIZ_PTXSA* = USB_OTG_HPTXFSIZ_PTXSA_Msk
  USB_OTG_HPTXFSIZ_PTXFD_Pos* = (16)
  USB_OTG_HPTXFSIZ_PTXFD_Msk* = (0x0000FFFF shl USB_OTG_HPTXFSIZ_PTXFD_Pos) ## !< 0xFFFF0000
  USB_OTG_HPTXFSIZ_PTXFD* = USB_OTG_HPTXFSIZ_PTXFD_Msk

## *******************  Bit definition for USB_OTG_DIEPCTL register  *******************

const
  USB_OTG_DIEPCTL_MPSIZ_Pos* = (0)
  USB_OTG_DIEPCTL_MPSIZ_Msk* = (0x000007FF shl USB_OTG_DIEPCTL_MPSIZ_Pos) ## !< 0x000007FF
  USB_OTG_DIEPCTL_MPSIZ* = USB_OTG_DIEPCTL_MPSIZ_Msk
  USB_OTG_DIEPCTL_USBAEP_Pos* = (15)
  USB_OTG_DIEPCTL_USBAEP_Msk* = (0x00000001 shl USB_OTG_DIEPCTL_USBAEP_Pos) ## !< 0x00008000
  USB_OTG_DIEPCTL_USBAEP* = USB_OTG_DIEPCTL_USBAEP_Msk
  USB_OTG_DIEPCTL_EONUM_DPID_Pos* = (16)
  USB_OTG_DIEPCTL_EONUM_DPID_Msk* = (0x00000001 shl
      USB_OTG_DIEPCTL_EONUM_DPID_Pos) ## !< 0x00010000
  USB_OTG_DIEPCTL_EONUM_DPID* = USB_OTG_DIEPCTL_EONUM_DPID_Msk
  USB_OTG_DIEPCTL_NAKSTS_Pos* = (17)
  USB_OTG_DIEPCTL_NAKSTS_Msk* = (0x00000001 shl USB_OTG_DIEPCTL_NAKSTS_Pos) ## !< 0x00020000
  USB_OTG_DIEPCTL_NAKSTS* = USB_OTG_DIEPCTL_NAKSTS_Msk
  USB_OTG_DIEPCTL_EPTYP_Pos* = (18)
  USB_OTG_DIEPCTL_EPTYP_Msk* = (0x00000003 shl USB_OTG_DIEPCTL_EPTYP_Pos) ## !< 0x000C0000
  USB_OTG_DIEPCTL_EPTYP* = USB_OTG_DIEPCTL_EPTYP_Msk
  USB_OTG_DIEPCTL_EPTYP_0* = (0x00000001 shl USB_OTG_DIEPCTL_EPTYP_Pos) ## !< 0x00040000
  USB_OTG_DIEPCTL_EPTYP_1* = (0x00000002 shl USB_OTG_DIEPCTL_EPTYP_Pos) ## !< 0x00080000
  USB_OTG_DIEPCTL_STALL_Pos* = (21)
  USB_OTG_DIEPCTL_STALL_Msk* = (0x00000001 shl USB_OTG_DIEPCTL_STALL_Pos) ## !< 0x00200000
  USB_OTG_DIEPCTL_STALL* = USB_OTG_DIEPCTL_STALL_Msk
  USB_OTG_DIEPCTL_TXFNUM_Pos* = (22)
  USB_OTG_DIEPCTL_TXFNUM_Msk* = (0x0000000F shl USB_OTG_DIEPCTL_TXFNUM_Pos) ## !< 0x03C00000
  USB_OTG_DIEPCTL_TXFNUM* = USB_OTG_DIEPCTL_TXFNUM_Msk
  USB_OTG_DIEPCTL_TXFNUM_0* = (0x00000001 shl USB_OTG_DIEPCTL_TXFNUM_Pos) ## !< 0x00400000
  USB_OTG_DIEPCTL_TXFNUM_1* = (0x00000002 shl USB_OTG_DIEPCTL_TXFNUM_Pos) ## !< 0x00800000
  USB_OTG_DIEPCTL_TXFNUM_2* = (0x00000004 shl USB_OTG_DIEPCTL_TXFNUM_Pos) ## !< 0x01000000
  USB_OTG_DIEPCTL_TXFNUM_3* = (0x00000008 shl USB_OTG_DIEPCTL_TXFNUM_Pos) ## !< 0x02000000
  USB_OTG_DIEPCTL_CNAK_Pos* = (26)
  USB_OTG_DIEPCTL_CNAK_Msk* = (0x00000001 shl USB_OTG_DIEPCTL_CNAK_Pos) ## !< 0x04000000
  USB_OTG_DIEPCTL_CNAK* = USB_OTG_DIEPCTL_CNAK_Msk
  USB_OTG_DIEPCTL_SNAK_Pos* = (27)
  USB_OTG_DIEPCTL_SNAK_Msk* = (0x00000001 shl USB_OTG_DIEPCTL_SNAK_Pos) ## !< 0x08000000
  USB_OTG_DIEPCTL_SNAK* = USB_OTG_DIEPCTL_SNAK_Msk
  USB_OTG_DIEPCTL_SD0PID_SEVNFRM_Pos* = (28)
  USB_OTG_DIEPCTL_SD0PID_SEVNFRM_Msk* = (
    0x00000001 shl USB_OTG_DIEPCTL_SD0PID_SEVNFRM_Pos) ## !< 0x10000000
  USB_OTG_DIEPCTL_SD0PID_SEVNFRM* = USB_OTG_DIEPCTL_SD0PID_SEVNFRM_Msk
  USB_OTG_DIEPCTL_SODDFRM_Pos* = (29)
  USB_OTG_DIEPCTL_SODDFRM_Msk* = (0x00000001 shl USB_OTG_DIEPCTL_SODDFRM_Pos) ## !< 0x20000000
  USB_OTG_DIEPCTL_SODDFRM* = USB_OTG_DIEPCTL_SODDFRM_Msk
  USB_OTG_DIEPCTL_EPDIS_Pos* = (30)
  USB_OTG_DIEPCTL_EPDIS_Msk* = (0x00000001 shl USB_OTG_DIEPCTL_EPDIS_Pos) ## !< 0x40000000
  USB_OTG_DIEPCTL_EPDIS* = USB_OTG_DIEPCTL_EPDIS_Msk
  USB_OTG_DIEPCTL_EPENA_Pos* = (31)
  USB_OTG_DIEPCTL_EPENA_Msk* = (0x00000001 shl USB_OTG_DIEPCTL_EPENA_Pos) ## !< 0x80000000
  USB_OTG_DIEPCTL_EPENA* = USB_OTG_DIEPCTL_EPENA_Msk

## *******************  Bit definition for USB_OTG_HCCHAR register  *******************

const
  USB_OTG_HCCHAR_MPSIZ_Pos* = (0)
  USB_OTG_HCCHAR_MPSIZ_Msk* = (0x000007FF shl USB_OTG_HCCHAR_MPSIZ_Pos) ## !< 0x000007FF
  USB_OTG_HCCHAR_MPSIZ* = USB_OTG_HCCHAR_MPSIZ_Msk
  USB_OTG_HCCHAR_EPNUM_Pos* = (11)
  USB_OTG_HCCHAR_EPNUM_Msk* = (0x0000000F shl USB_OTG_HCCHAR_EPNUM_Pos) ## !< 0x00007800
  USB_OTG_HCCHAR_EPNUM* = USB_OTG_HCCHAR_EPNUM_Msk
  USB_OTG_HCCHAR_EPNUM_0* = (0x00000001 shl USB_OTG_HCCHAR_EPNUM_Pos) ## !< 0x00000800
  USB_OTG_HCCHAR_EPNUM_1* = (0x00000002 shl USB_OTG_HCCHAR_EPNUM_Pos) ## !< 0x00001000
  USB_OTG_HCCHAR_EPNUM_2* = (0x00000004 shl USB_OTG_HCCHAR_EPNUM_Pos) ## !< 0x00002000
  USB_OTG_HCCHAR_EPNUM_3* = (0x00000008 shl USB_OTG_HCCHAR_EPNUM_Pos) ## !< 0x00004000
  USB_OTG_HCCHAR_EPDIR_Pos* = (15)
  USB_OTG_HCCHAR_EPDIR_Msk* = (0x00000001 shl USB_OTG_HCCHAR_EPDIR_Pos) ## !< 0x00008000
  USB_OTG_HCCHAR_EPDIR* = USB_OTG_HCCHAR_EPDIR_Msk
  USB_OTG_HCCHAR_LSDEV_Pos* = (17)
  USB_OTG_HCCHAR_LSDEV_Msk* = (0x00000001 shl USB_OTG_HCCHAR_LSDEV_Pos) ## !< 0x00020000
  USB_OTG_HCCHAR_LSDEV* = USB_OTG_HCCHAR_LSDEV_Msk
  USB_OTG_HCCHAR_EPTYP_Pos* = (18)
  USB_OTG_HCCHAR_EPTYP_Msk* = (0x00000003 shl USB_OTG_HCCHAR_EPTYP_Pos) ## !< 0x000C0000
  USB_OTG_HCCHAR_EPTYP* = USB_OTG_HCCHAR_EPTYP_Msk
  USB_OTG_HCCHAR_EPTYP_0* = (0x00000001 shl USB_OTG_HCCHAR_EPTYP_Pos) ## !< 0x00040000
  USB_OTG_HCCHAR_EPTYP_1* = (0x00000002 shl USB_OTG_HCCHAR_EPTYP_Pos) ## !< 0x00080000
  USB_OTG_HCCHAR_MC_Pos* = (20)
  USB_OTG_HCCHAR_MC_Msk* = (0x00000003 shl USB_OTG_HCCHAR_MC_Pos) ## !< 0x00300000
  USB_OTG_HCCHAR_MC* = USB_OTG_HCCHAR_MC_Msk
  USB_OTG_HCCHAR_MC_0* = (0x00000001 shl USB_OTG_HCCHAR_MC_Pos) ## !< 0x00100000
  USB_OTG_HCCHAR_MC_1* = (0x00000002 shl USB_OTG_HCCHAR_MC_Pos) ## !< 0x00200000
  USB_OTG_HCCHAR_DAD_Pos* = (22)
  USB_OTG_HCCHAR_DAD_Msk* = (0x0000007F shl USB_OTG_HCCHAR_DAD_Pos) ## !< 0x1FC00000
  USB_OTG_HCCHAR_DAD* = USB_OTG_HCCHAR_DAD_Msk
  USB_OTG_HCCHAR_DAD_0* = (0x00000001 shl USB_OTG_HCCHAR_DAD_Pos) ## !< 0x00400000
  USB_OTG_HCCHAR_DAD_1* = (0x00000002 shl USB_OTG_HCCHAR_DAD_Pos) ## !< 0x00800000
  USB_OTG_HCCHAR_DAD_2* = (0x00000004 shl USB_OTG_HCCHAR_DAD_Pos) ## !< 0x01000000
  USB_OTG_HCCHAR_DAD_3* = (0x00000008 shl USB_OTG_HCCHAR_DAD_Pos) ## !< 0x02000000
  USB_OTG_HCCHAR_DAD_4* = (0x00000010 shl USB_OTG_HCCHAR_DAD_Pos) ## !< 0x04000000
  USB_OTG_HCCHAR_DAD_5* = (0x00000020 shl USB_OTG_HCCHAR_DAD_Pos) ## !< 0x08000000
  USB_OTG_HCCHAR_DAD_6* = (0x00000040 shl USB_OTG_HCCHAR_DAD_Pos) ## !< 0x10000000
  USB_OTG_HCCHAR_ODDFRM_Pos* = (29)
  USB_OTG_HCCHAR_ODDFRM_Msk* = (0x00000001 shl USB_OTG_HCCHAR_ODDFRM_Pos) ## !< 0x20000000
  USB_OTG_HCCHAR_ODDFRM* = USB_OTG_HCCHAR_ODDFRM_Msk
  USB_OTG_HCCHAR_CHDIS_Pos* = (30)
  USB_OTG_HCCHAR_CHDIS_Msk* = (0x00000001 shl USB_OTG_HCCHAR_CHDIS_Pos) ## !< 0x40000000
  USB_OTG_HCCHAR_CHDIS* = USB_OTG_HCCHAR_CHDIS_Msk
  USB_OTG_HCCHAR_CHENA_Pos* = (31)
  USB_OTG_HCCHAR_CHENA_Msk* = (0x00000001 shl USB_OTG_HCCHAR_CHENA_Pos) ## !< 0x80000000
  USB_OTG_HCCHAR_CHENA* = USB_OTG_HCCHAR_CHENA_Msk

## *******************  Bit definition for USB_OTG_HCSPLT register  *******************

const
  USB_OTG_HCSPLT_PRTADDR_Pos* = (0)
  USB_OTG_HCSPLT_PRTADDR_Msk* = (0x0000007F shl USB_OTG_HCSPLT_PRTADDR_Pos) ## !< 0x0000007F
  USB_OTG_HCSPLT_PRTADDR* = USB_OTG_HCSPLT_PRTADDR_Msk
  USB_OTG_HCSPLT_PRTADDR_0* = (0x00000001 shl USB_OTG_HCSPLT_PRTADDR_Pos) ## !< 0x00000001
  USB_OTG_HCSPLT_PRTADDR_1* = (0x00000002 shl USB_OTG_HCSPLT_PRTADDR_Pos) ## !< 0x00000002
  USB_OTG_HCSPLT_PRTADDR_2* = (0x00000004 shl USB_OTG_HCSPLT_PRTADDR_Pos) ## !< 0x00000004
  USB_OTG_HCSPLT_PRTADDR_3* = (0x00000008 shl USB_OTG_HCSPLT_PRTADDR_Pos) ## !< 0x00000008
  USB_OTG_HCSPLT_PRTADDR_4* = (0x00000010 shl USB_OTG_HCSPLT_PRTADDR_Pos) ## !< 0x00000010
  USB_OTG_HCSPLT_PRTADDR_5* = (0x00000020 shl USB_OTG_HCSPLT_PRTADDR_Pos) ## !< 0x00000020
  USB_OTG_HCSPLT_PRTADDR_6* = (0x00000040 shl USB_OTG_HCSPLT_PRTADDR_Pos) ## !< 0x00000040
  USB_OTG_HCSPLT_HUBADDR_Pos* = (7)
  USB_OTG_HCSPLT_HUBADDR_Msk* = (0x0000007F shl USB_OTG_HCSPLT_HUBADDR_Pos) ## !< 0x00003F80
  USB_OTG_HCSPLT_HUBADDR* = USB_OTG_HCSPLT_HUBADDR_Msk
  USB_OTG_HCSPLT_HUBADDR_0* = (0x00000001 shl USB_OTG_HCSPLT_HUBADDR_Pos) ## !< 0x00000080
  USB_OTG_HCSPLT_HUBADDR_1* = (0x00000002 shl USB_OTG_HCSPLT_HUBADDR_Pos) ## !< 0x00000100
  USB_OTG_HCSPLT_HUBADDR_2* = (0x00000004 shl USB_OTG_HCSPLT_HUBADDR_Pos) ## !< 0x00000200
  USB_OTG_HCSPLT_HUBADDR_3* = (0x00000008 shl USB_OTG_HCSPLT_HUBADDR_Pos) ## !< 0x00000400
  USB_OTG_HCSPLT_HUBADDR_4* = (0x00000010 shl USB_OTG_HCSPLT_HUBADDR_Pos) ## !< 0x00000800
  USB_OTG_HCSPLT_HUBADDR_5* = (0x00000020 shl USB_OTG_HCSPLT_HUBADDR_Pos) ## !< 0x00001000
  USB_OTG_HCSPLT_HUBADDR_6* = (0x00000040 shl USB_OTG_HCSPLT_HUBADDR_Pos) ## !< 0x00002000
  USB_OTG_HCSPLT_XACTPOS_Pos* = (14)
  USB_OTG_HCSPLT_XACTPOS_Msk* = (0x00000003 shl USB_OTG_HCSPLT_XACTPOS_Pos) ## !< 0x0000C000
  USB_OTG_HCSPLT_XACTPOS* = USB_OTG_HCSPLT_XACTPOS_Msk
  USB_OTG_HCSPLT_XACTPOS_0* = (0x00000001 shl USB_OTG_HCSPLT_XACTPOS_Pos) ## !< 0x00004000
  USB_OTG_HCSPLT_XACTPOS_1* = (0x00000002 shl USB_OTG_HCSPLT_XACTPOS_Pos) ## !< 0x00008000
  USB_OTG_HCSPLT_COMPLSPLT_Pos* = (16)
  USB_OTG_HCSPLT_COMPLSPLT_Msk* = (0x00000001 shl USB_OTG_HCSPLT_COMPLSPLT_Pos) ## !< 0x00010000
  USB_OTG_HCSPLT_COMPLSPLT* = USB_OTG_HCSPLT_COMPLSPLT_Msk
  USB_OTG_HCSPLT_SPLITEN_Pos* = (31)
  USB_OTG_HCSPLT_SPLITEN_Msk* = (0x00000001 shl USB_OTG_HCSPLT_SPLITEN_Pos) ## !< 0x80000000
  USB_OTG_HCSPLT_SPLITEN* = USB_OTG_HCSPLT_SPLITEN_Msk

## *******************  Bit definition for USB_OTG_HCINT register  *******************

const
  USB_OTG_HCINT_XFRC_Pos* = (0)
  USB_OTG_HCINT_XFRC_Msk* = (0x00000001 shl USB_OTG_HCINT_XFRC_Pos) ## !< 0x00000001
  USB_OTG_HCINT_XFRC* = USB_OTG_HCINT_XFRC_Msk
  USB_OTG_HCINT_CHH_Pos* = (1)
  USB_OTG_HCINT_CHH_Msk* = (0x00000001 shl USB_OTG_HCINT_CHH_Pos) ## !< 0x00000002
  USB_OTG_HCINT_CHH* = USB_OTG_HCINT_CHH_Msk
  USB_OTG_HCINT_AHBERR_Pos* = (2)
  USB_OTG_HCINT_AHBERR_Msk* = (0x00000001 shl USB_OTG_HCINT_AHBERR_Pos) ## !< 0x00000004
  USB_OTG_HCINT_AHBERR* = USB_OTG_HCINT_AHBERR_Msk
  USB_OTG_HCINT_STALL_Pos* = (3)
  USB_OTG_HCINT_STALL_Msk* = (0x00000001 shl USB_OTG_HCINT_STALL_Pos) ## !< 0x00000008
  USB_OTG_HCINT_STALL* = USB_OTG_HCINT_STALL_Msk
  USB_OTG_HCINT_NAK_Pos* = (4)
  USB_OTG_HCINT_NAK_Msk* = (0x00000001 shl USB_OTG_HCINT_NAK_Pos) ## !< 0x00000010
  USB_OTG_HCINT_NAK* = USB_OTG_HCINT_NAK_Msk
  USB_OTG_HCINT_ACK_Pos* = (5)
  USB_OTG_HCINT_ACK_Msk* = (0x00000001 shl USB_OTG_HCINT_ACK_Pos) ## !< 0x00000020
  USB_OTG_HCINT_ACK* = USB_OTG_HCINT_ACK_Msk
  USB_OTG_HCINT_NYET_Pos* = (6)
  USB_OTG_HCINT_NYET_Msk* = (0x00000001 shl USB_OTG_HCINT_NYET_Pos) ## !< 0x00000040
  USB_OTG_HCINT_NYET* = USB_OTG_HCINT_NYET_Msk
  USB_OTG_HCINT_TXERR_Pos* = (7)
  USB_OTG_HCINT_TXERR_Msk* = (0x00000001 shl USB_OTG_HCINT_TXERR_Pos) ## !< 0x00000080
  USB_OTG_HCINT_TXERR* = USB_OTG_HCINT_TXERR_Msk
  USB_OTG_HCINT_BBERR_Pos* = (8)
  USB_OTG_HCINT_BBERR_Msk* = (0x00000001 shl USB_OTG_HCINT_BBERR_Pos) ## !< 0x00000100
  USB_OTG_HCINT_BBERR* = USB_OTG_HCINT_BBERR_Msk
  USB_OTG_HCINT_FRMOR_Pos* = (9)
  USB_OTG_HCINT_FRMOR_Msk* = (0x00000001 shl USB_OTG_HCINT_FRMOR_Pos) ## !< 0x00000200
  USB_OTG_HCINT_FRMOR* = USB_OTG_HCINT_FRMOR_Msk
  USB_OTG_HCINT_DTERR_Pos* = (10)
  USB_OTG_HCINT_DTERR_Msk* = (0x00000001 shl USB_OTG_HCINT_DTERR_Pos) ## !< 0x00000400
  USB_OTG_HCINT_DTERR* = USB_OTG_HCINT_DTERR_Msk

## *******************  Bit definition for USB_OTG_DIEPINT register  *******************

const
  USB_OTG_DIEPINT_XFRC_Pos* = (0)
  USB_OTG_DIEPINT_XFRC_Msk* = (0x00000001 shl USB_OTG_DIEPINT_XFRC_Pos) ## !< 0x00000001
  USB_OTG_DIEPINT_XFRC* = USB_OTG_DIEPINT_XFRC_Msk
  USB_OTG_DIEPINT_EPDISD_Pos* = (1)
  USB_OTG_DIEPINT_EPDISD_Msk* = (0x00000001 shl USB_OTG_DIEPINT_EPDISD_Pos) ## !< 0x00000002
  USB_OTG_DIEPINT_EPDISD* = USB_OTG_DIEPINT_EPDISD_Msk
  USB_OTG_DIEPINT_AHBERR_Pos* = (2)
  USB_OTG_DIEPINT_AHBERR_Msk* = (0x00000001 shl USB_OTG_DIEPINT_AHBERR_Pos) ## !< 0x00000004
  USB_OTG_DIEPINT_AHBERR* = USB_OTG_DIEPINT_AHBERR_Msk
  USB_OTG_DIEPINT_TOC_Pos* = (3)
  USB_OTG_DIEPINT_TOC_Msk* = (0x00000001 shl USB_OTG_DIEPINT_TOC_Pos) ## !< 0x00000008
  USB_OTG_DIEPINT_TOC* = USB_OTG_DIEPINT_TOC_Msk
  USB_OTG_DIEPINT_ITTXFE_Pos* = (4)
  USB_OTG_DIEPINT_ITTXFE_Msk* = (0x00000001 shl USB_OTG_DIEPINT_ITTXFE_Pos) ## !< 0x00000010
  USB_OTG_DIEPINT_ITTXFE* = USB_OTG_DIEPINT_ITTXFE_Msk
  USB_OTG_DIEPINT_INEPNM_Pos* = (5)
  USB_OTG_DIEPINT_INEPNM_Msk* = (0x00000001 shl USB_OTG_DIEPINT_INEPNM_Pos) ## !< 0x00000004
  USB_OTG_DIEPINT_INEPNM* = USB_OTG_DIEPINT_INEPNM_Msk
  USB_OTG_DIEPINT_INEPNE_Pos* = (6)
  USB_OTG_DIEPINT_INEPNE_Msk* = (0x00000001 shl USB_OTG_DIEPINT_INEPNE_Pos) ## !< 0x00000040
  USB_OTG_DIEPINT_INEPNE* = USB_OTG_DIEPINT_INEPNE_Msk
  USB_OTG_DIEPINT_TXFE_Pos* = (7)
  USB_OTG_DIEPINT_TXFE_Msk* = (0x00000001 shl USB_OTG_DIEPINT_TXFE_Pos) ## !< 0x00000080
  USB_OTG_DIEPINT_TXFE* = USB_OTG_DIEPINT_TXFE_Msk
  USB_OTG_DIEPINT_TXFIFOUDRN_Pos* = (8)
  USB_OTG_DIEPINT_TXFIFOUDRN_Msk* = (0x00000001 shl
      USB_OTG_DIEPINT_TXFIFOUDRN_Pos) ## !< 0x00000100
  USB_OTG_DIEPINT_TXFIFOUDRN* = USB_OTG_DIEPINT_TXFIFOUDRN_Msk
  USB_OTG_DIEPINT_BNA_Pos* = (9)
  USB_OTG_DIEPINT_BNA_Msk* = (0x00000001 shl USB_OTG_DIEPINT_BNA_Pos) ## !< 0x00000200
  USB_OTG_DIEPINT_BNA* = USB_OTG_DIEPINT_BNA_Msk
  USB_OTG_DIEPINT_PKTDRPSTS_Pos* = (11)
  USB_OTG_DIEPINT_PKTDRPSTS_Msk* = (0x00000001 shl USB_OTG_DIEPINT_PKTDRPSTS_Pos) ## !< 0x00000800
  USB_OTG_DIEPINT_PKTDRPSTS* = USB_OTG_DIEPINT_PKTDRPSTS_Msk
  USB_OTG_DIEPINT_BERR_Pos* = (12)
  USB_OTG_DIEPINT_BERR_Msk* = (0x00000001 shl USB_OTG_DIEPINT_BERR_Pos) ## !< 0x00001000
  USB_OTG_DIEPINT_BERR* = USB_OTG_DIEPINT_BERR_Msk
  USB_OTG_DIEPINT_NAK_Pos* = (13)
  USB_OTG_DIEPINT_NAK_Msk* = (0x00000001 shl USB_OTG_DIEPINT_NAK_Pos) ## !< 0x00002000
  USB_OTG_DIEPINT_NAK* = USB_OTG_DIEPINT_NAK_Msk

## *******************  Bit definition forUSB_OTG_HCINTMSK register  *******************

const
  USB_OTG_HCINTMSK_XFRCM_Pos* = (0)
  USB_OTG_HCINTMSK_XFRCM_Msk* = (0x00000001 shl USB_OTG_HCINTMSK_XFRCM_Pos) ## !< 0x00000001
  USB_OTG_HCINTMSK_XFRCM* = USB_OTG_HCINTMSK_XFRCM_Msk
  USB_OTG_HCINTMSK_CHHM_Pos* = (1)
  USB_OTG_HCINTMSK_CHHM_Msk* = (0x00000001 shl USB_OTG_HCINTMSK_CHHM_Pos) ## !< 0x00000002
  USB_OTG_HCINTMSK_CHHM* = USB_OTG_HCINTMSK_CHHM_Msk
  USB_OTG_HCINTMSK_AHBERR_Pos* = (2)
  USB_OTG_HCINTMSK_AHBERR_Msk* = (0x00000001 shl USB_OTG_HCINTMSK_AHBERR_Pos) ## !< 0x00000004
  USB_OTG_HCINTMSK_AHBERR* = USB_OTG_HCINTMSK_AHBERR_Msk
  USB_OTG_HCINTMSK_STALLM_Pos* = (3)
  USB_OTG_HCINTMSK_STALLM_Msk* = (0x00000001 shl USB_OTG_HCINTMSK_STALLM_Pos) ## !< 0x00000008
  USB_OTG_HCINTMSK_STALLM* = USB_OTG_HCINTMSK_STALLM_Msk
  USB_OTG_HCINTMSK_NAKM_Pos* = (4)
  USB_OTG_HCINTMSK_NAKM_Msk* = (0x00000001 shl USB_OTG_HCINTMSK_NAKM_Pos) ## !< 0x00000010
  USB_OTG_HCINTMSK_NAKM* = USB_OTG_HCINTMSK_NAKM_Msk
  USB_OTG_HCINTMSK_ACKM_Pos* = (5)
  USB_OTG_HCINTMSK_ACKM_Msk* = (0x00000001 shl USB_OTG_HCINTMSK_ACKM_Pos) ## !< 0x00000020
  USB_OTG_HCINTMSK_ACKM* = USB_OTG_HCINTMSK_ACKM_Msk
  USB_OTG_HCINTMSK_NYET_Pos* = (6)
  USB_OTG_HCINTMSK_NYET_Msk* = (0x00000001 shl USB_OTG_HCINTMSK_NYET_Pos) ## !< 0x00000040
  USB_OTG_HCINTMSK_NYET* = USB_OTG_HCINTMSK_NYET_Msk
  USB_OTG_HCINTMSK_TXERRM_Pos* = (7)
  USB_OTG_HCINTMSK_TXERRM_Msk* = (0x00000001 shl USB_OTG_HCINTMSK_TXERRM_Pos) ## !< 0x00000080
  USB_OTG_HCINTMSK_TXERRM* = USB_OTG_HCINTMSK_TXERRM_Msk
  USB_OTG_HCINTMSK_BBERRM_Pos* = (8)
  USB_OTG_HCINTMSK_BBERRM_Msk* = (0x00000001 shl USB_OTG_HCINTMSK_BBERRM_Pos) ## !< 0x00000100
  USB_OTG_HCINTMSK_BBERRM* = USB_OTG_HCINTMSK_BBERRM_Msk
  USB_OTG_HCINTMSK_FRMORM_Pos* = (9)
  USB_OTG_HCINTMSK_FRMORM_Msk* = (0x00000001 shl USB_OTG_HCINTMSK_FRMORM_Pos) ## !< 0x00000200
  USB_OTG_HCINTMSK_FRMORM* = USB_OTG_HCINTMSK_FRMORM_Msk
  USB_OTG_HCINTMSK_DTERRM_Pos* = (10)
  USB_OTG_HCINTMSK_DTERRM_Msk* = (0x00000001 shl USB_OTG_HCINTMSK_DTERRM_Pos) ## !< 0x00000400
  USB_OTG_HCINTMSK_DTERRM* = USB_OTG_HCINTMSK_DTERRM_Msk

## *******************  Bit definition for USB_OTG_DIEPTSIZ register  *******************

const
  USB_OTG_DIEPTSIZ_XFRSIZ_Pos* = (0)
  USB_OTG_DIEPTSIZ_XFRSIZ_Msk* = (0x0007FFFF shl USB_OTG_DIEPTSIZ_XFRSIZ_Pos) ## !< 0x0007FFFF
  USB_OTG_DIEPTSIZ_XFRSIZ* = USB_OTG_DIEPTSIZ_XFRSIZ_Msk
  USB_OTG_DIEPTSIZ_PKTCNT_Pos* = (19)
  USB_OTG_DIEPTSIZ_PKTCNT_Msk* = (0x000003FF shl USB_OTG_DIEPTSIZ_PKTCNT_Pos) ## !< 0x1FF80000
  USB_OTG_DIEPTSIZ_PKTCNT* = USB_OTG_DIEPTSIZ_PKTCNT_Msk
  USB_OTG_DIEPTSIZ_MULCNT_Pos* = (29)
  USB_OTG_DIEPTSIZ_MULCNT_Msk* = (0x00000003 shl USB_OTG_DIEPTSIZ_MULCNT_Pos) ## !< 0x60000000
  USB_OTG_DIEPTSIZ_MULCNT* = USB_OTG_DIEPTSIZ_MULCNT_Msk

## *******************  Bit definition for USB_OTG_HCTSIZ register  *******************

const
  USB_OTG_HCTSIZ_XFRSIZ_Pos* = (0)
  USB_OTG_HCTSIZ_XFRSIZ_Msk* = (0x0007FFFF shl USB_OTG_HCTSIZ_XFRSIZ_Pos) ## !< 0x0007FFFF
  USB_OTG_HCTSIZ_XFRSIZ* = USB_OTG_HCTSIZ_XFRSIZ_Msk
  USB_OTG_HCTSIZ_PKTCNT_Pos* = (19)
  USB_OTG_HCTSIZ_PKTCNT_Msk* = (0x000003FF shl USB_OTG_HCTSIZ_PKTCNT_Pos) ## !< 0x1FF80000
  USB_OTG_HCTSIZ_PKTCNT* = USB_OTG_HCTSIZ_PKTCNT_Msk
  USB_OTG_HCTSIZ_DOPING_Pos* = (31)
  USB_OTG_HCTSIZ_DOPING_Msk* = (0x00000001 shl USB_OTG_HCTSIZ_DOPING_Pos) ## !< 0x80000000
  USB_OTG_HCTSIZ_DOPING* = USB_OTG_HCTSIZ_DOPING_Msk
  USB_OTG_HCTSIZ_DPID_Pos* = (29)
  USB_OTG_HCTSIZ_DPID_Msk* = (0x00000003 shl USB_OTG_HCTSIZ_DPID_Pos) ## !< 0x60000000
  USB_OTG_HCTSIZ_DPID* = USB_OTG_HCTSIZ_DPID_Msk
  USB_OTG_HCTSIZ_DPID_0* = (0x00000001 shl USB_OTG_HCTSIZ_DPID_Pos) ## !< 0x20000000
  USB_OTG_HCTSIZ_DPID_1* = (0x00000002 shl USB_OTG_HCTSIZ_DPID_Pos) ## !< 0x40000000

## *******************  Bit definition for USB_OTG_DIEPDMA register  *******************

const
  USB_OTG_DIEPDMA_DMAADDR_Pos* = (0)
  USB_OTG_DIEPDMA_DMAADDR_Msk* = (0xFFFFFFFF shl USB_OTG_DIEPDMA_DMAADDR_Pos) ## !< 0xFFFFFFFF
  USB_OTG_DIEPDMA_DMAADDR* = USB_OTG_DIEPDMA_DMAADDR_Msk

## *******************  Bit definition for USB_OTG_HCDMA register  *******************

const
  USB_OTG_HCDMA_DMAADDR_Pos* = (0)
  USB_OTG_HCDMA_DMAADDR_Msk* = (0xFFFFFFFF shl USB_OTG_HCDMA_DMAADDR_Pos) ## !< 0xFFFFFFFF
  USB_OTG_HCDMA_DMAADDR* = USB_OTG_HCDMA_DMAADDR_Msk

## *******************  Bit definition for USB_OTG_DTXFSTS register  *******************

const
  USB_OTG_DTXFSTS_INEPTFSAV_Pos* = (0)
  USB_OTG_DTXFSTS_INEPTFSAV_Msk* = (0x0000FFFF shl USB_OTG_DTXFSTS_INEPTFSAV_Pos) ## !< 0x0000FFFF
  USB_OTG_DTXFSTS_INEPTFSAV* = USB_OTG_DTXFSTS_INEPTFSAV_Msk

## *******************  Bit definition for USB_OTG_DIEPTXF register  *******************

const
  USB_OTG_DIEPTXF_INEPTXSA_Pos* = (0)
  USB_OTG_DIEPTXF_INEPTXSA_Msk* = (0x0000FFFF shl USB_OTG_DIEPTXF_INEPTXSA_Pos) ## !< 0x0000FFFF
  USB_OTG_DIEPTXF_INEPTXSA* = USB_OTG_DIEPTXF_INEPTXSA_Msk
  USB_OTG_DIEPTXF_INEPTXFD_Pos* = (16)
  USB_OTG_DIEPTXF_INEPTXFD_Msk* = (0x0000FFFF shl USB_OTG_DIEPTXF_INEPTXFD_Pos) ## !< 0xFFFF0000
  USB_OTG_DIEPTXF_INEPTXFD* = USB_OTG_DIEPTXF_INEPTXFD_Msk

## *******************  Bit definition for USB_OTG_DOEPCTL register  *******************

const
  USB_OTG_DOEPCTL_MPSIZ_Pos* = (0)
  USB_OTG_DOEPCTL_MPSIZ_Msk* = (0x000007FF shl USB_OTG_DOEPCTL_MPSIZ_Pos) ## !< 0x000007FF
  USB_OTG_DOEPCTL_MPSIZ* = USB_OTG_DOEPCTL_MPSIZ_Msk
  USB_OTG_DOEPCTL_USBAEP_Pos* = (15)
  USB_OTG_DOEPCTL_USBAEP_Msk* = (0x00000001 shl USB_OTG_DOEPCTL_USBAEP_Pos) ## !< 0x00008000
  USB_OTG_DOEPCTL_USBAEP* = USB_OTG_DOEPCTL_USBAEP_Msk
  USB_OTG_DOEPCTL_NAKSTS_Pos* = (17)
  USB_OTG_DOEPCTL_NAKSTS_Msk* = (0x00000001 shl USB_OTG_DOEPCTL_NAKSTS_Pos) ## !< 0x00020000
  USB_OTG_DOEPCTL_NAKSTS* = USB_OTG_DOEPCTL_NAKSTS_Msk
  USB_OTG_DOEPCTL_SD0PID_SEVNFRM_Pos* = (28)
  USB_OTG_DOEPCTL_SD0PID_SEVNFRM_Msk* = (
    0x00000001 shl USB_OTG_DOEPCTL_SD0PID_SEVNFRM_Pos) ## !< 0x10000000
  USB_OTG_DOEPCTL_SD0PID_SEVNFRM* = USB_OTG_DOEPCTL_SD0PID_SEVNFRM_Msk
  USB_OTG_DOEPCTL_SODDFRM_Pos* = (29)
  USB_OTG_DOEPCTL_SODDFRM_Msk* = (0x00000001 shl USB_OTG_DOEPCTL_SODDFRM_Pos) ## !< 0x20000000
  USB_OTG_DOEPCTL_SODDFRM* = USB_OTG_DOEPCTL_SODDFRM_Msk
  USB_OTG_DOEPCTL_EPTYP_Pos* = (18)
  USB_OTG_DOEPCTL_EPTYP_Msk* = (0x00000003 shl USB_OTG_DOEPCTL_EPTYP_Pos) ## !< 0x000C0000
  USB_OTG_DOEPCTL_EPTYP* = USB_OTG_DOEPCTL_EPTYP_Msk
  USB_OTG_DOEPCTL_EPTYP_0* = (0x00000001 shl USB_OTG_DOEPCTL_EPTYP_Pos) ## !< 0x00040000
  USB_OTG_DOEPCTL_EPTYP_1* = (0x00000002 shl USB_OTG_DOEPCTL_EPTYP_Pos) ## !< 0x00080000
  USB_OTG_DOEPCTL_SNPM_Pos* = (20)
  USB_OTG_DOEPCTL_SNPM_Msk* = (0x00000001 shl USB_OTG_DOEPCTL_SNPM_Pos) ## !< 0x00100000
  USB_OTG_DOEPCTL_SNPM* = USB_OTG_DOEPCTL_SNPM_Msk
  USB_OTG_DOEPCTL_STALL_Pos* = (21)
  USB_OTG_DOEPCTL_STALL_Msk* = (0x00000001 shl USB_OTG_DOEPCTL_STALL_Pos) ## !< 0x00200000
  USB_OTG_DOEPCTL_STALL* = USB_OTG_DOEPCTL_STALL_Msk
  USB_OTG_DOEPCTL_CNAK_Pos* = (26)
  USB_OTG_DOEPCTL_CNAK_Msk* = (0x00000001 shl USB_OTG_DOEPCTL_CNAK_Pos) ## !< 0x04000000
  USB_OTG_DOEPCTL_CNAK* = USB_OTG_DOEPCTL_CNAK_Msk
  USB_OTG_DOEPCTL_SNAK_Pos* = (27)
  USB_OTG_DOEPCTL_SNAK_Msk* = (0x00000001 shl USB_OTG_DOEPCTL_SNAK_Pos) ## !< 0x08000000
  USB_OTG_DOEPCTL_SNAK* = USB_OTG_DOEPCTL_SNAK_Msk
  USB_OTG_DOEPCTL_EPDIS_Pos* = (30)
  USB_OTG_DOEPCTL_EPDIS_Msk* = (0x00000001 shl USB_OTG_DOEPCTL_EPDIS_Pos) ## !< 0x40000000
  USB_OTG_DOEPCTL_EPDIS* = USB_OTG_DOEPCTL_EPDIS_Msk
  USB_OTG_DOEPCTL_EPENA_Pos* = (31)
  USB_OTG_DOEPCTL_EPENA_Msk* = (0x00000001 shl USB_OTG_DOEPCTL_EPENA_Pos) ## !< 0x80000000
  USB_OTG_DOEPCTL_EPENA* = USB_OTG_DOEPCTL_EPENA_Msk

## *******************  Bit definition for USB_OTG_DOEPINT register  *******************

const
  USB_OTG_DOEPINT_XFRC_Pos* = (0)
  USB_OTG_DOEPINT_XFRC_Msk* = (0x00000001 shl USB_OTG_DOEPINT_XFRC_Pos) ## !< 0x00000001
  USB_OTG_DOEPINT_XFRC* = USB_OTG_DOEPINT_XFRC_Msk
  USB_OTG_DOEPINT_EPDISD_Pos* = (1)
  USB_OTG_DOEPINT_EPDISD_Msk* = (0x00000001 shl USB_OTG_DOEPINT_EPDISD_Pos) ## !< 0x00000002
  USB_OTG_DOEPINT_EPDISD* = USB_OTG_DOEPINT_EPDISD_Msk
  USB_OTG_DOEPINT_AHBERR_Pos* = (2)
  USB_OTG_DOEPINT_AHBERR_Msk* = (0x00000001 shl USB_OTG_DOEPINT_AHBERR_Pos) ## !< 0x00000004
  USB_OTG_DOEPINT_AHBERR* = USB_OTG_DOEPINT_AHBERR_Msk
  USB_OTG_DOEPINT_STUP_Pos* = (3)
  USB_OTG_DOEPINT_STUP_Msk* = (0x00000001 shl USB_OTG_DOEPINT_STUP_Pos) ## !< 0x00000008
  USB_OTG_DOEPINT_STUP* = USB_OTG_DOEPINT_STUP_Msk
  USB_OTG_DOEPINT_OTEPDIS_Pos* = (4)
  USB_OTG_DOEPINT_OTEPDIS_Msk* = (0x00000001 shl USB_OTG_DOEPINT_OTEPDIS_Pos) ## !< 0x00000010
  USB_OTG_DOEPINT_OTEPDIS* = USB_OTG_DOEPINT_OTEPDIS_Msk
  USB_OTG_DOEPINT_OTEPSPR_Pos* = (5)
  USB_OTG_DOEPINT_OTEPSPR_Msk* = (0x00000001 shl USB_OTG_DOEPINT_OTEPSPR_Pos) ## !< 0x00000020
  USB_OTG_DOEPINT_OTEPSPR* = USB_OTG_DOEPINT_OTEPSPR_Msk
  USB_OTG_DOEPINT_B2BSTUP_Pos* = (6)
  USB_OTG_DOEPINT_B2BSTUP_Msk* = (0x00000001 shl USB_OTG_DOEPINT_B2BSTUP_Pos) ## !< 0x00000040
  USB_OTG_DOEPINT_B2BSTUP* = USB_OTG_DOEPINT_B2BSTUP_Msk
  USB_OTG_DOEPINT_OUTPKTERR_Pos* = (8)
  USB_OTG_DOEPINT_OUTPKTERR_Msk* = (0x00000001 shl USB_OTG_DOEPINT_OUTPKTERR_Pos) ## !< 0x00000100
  USB_OTG_DOEPINT_OUTPKTERR* = USB_OTG_DOEPINT_OUTPKTERR_Msk
  USB_OTG_DOEPINT_NAK_Pos* = (13)
  USB_OTG_DOEPINT_NAK_Msk* = (0x00000001 shl USB_OTG_DOEPINT_NAK_Pos) ## !< 0x00002000
  USB_OTG_DOEPINT_NAK* = USB_OTG_DOEPINT_NAK_Msk
  USB_OTG_DOEPINT_NYET_Pos* = (14)
  USB_OTG_DOEPINT_NYET_Msk* = (0x00000001 shl USB_OTG_DOEPINT_NYET_Pos) ## !< 0x00004000
  USB_OTG_DOEPINT_NYET* = USB_OTG_DOEPINT_NYET_Msk
  USB_OTG_DOEPINT_STPKTRX_Pos* = (15)
  USB_OTG_DOEPINT_STPKTRX_Msk* = (0x00000001 shl USB_OTG_DOEPINT_STPKTRX_Pos) ## !< 0x00008000
  USB_OTG_DOEPINT_STPKTRX* = USB_OTG_DOEPINT_STPKTRX_Msk

## *******************  Bit definition for USB_OTG_DOEPTSIZ register  *******************

const
  USB_OTG_DOEPTSIZ_XFRSIZ_Pos* = (0)
  USB_OTG_DOEPTSIZ_XFRSIZ_Msk* = (0x0007FFFF shl USB_OTG_DOEPTSIZ_XFRSIZ_Pos) ## !< 0x0007FFFF
  USB_OTG_DOEPTSIZ_XFRSIZ* = USB_OTG_DOEPTSIZ_XFRSIZ_Msk
  USB_OTG_DOEPTSIZ_PKTCNT_Pos* = (19)
  USB_OTG_DOEPTSIZ_PKTCNT_Msk* = (0x000003FF shl USB_OTG_DOEPTSIZ_PKTCNT_Pos) ## !< 0x1FF80000
  USB_OTG_DOEPTSIZ_PKTCNT* = USB_OTG_DOEPTSIZ_PKTCNT_Msk
  USB_OTG_DOEPTSIZ_STUPCNT_Pos* = (29)
  USB_OTG_DOEPTSIZ_STUPCNT_Msk* = (0x00000003 shl USB_OTG_DOEPTSIZ_STUPCNT_Pos) ## !< 0x60000000
  USB_OTG_DOEPTSIZ_STUPCNT* = USB_OTG_DOEPTSIZ_STUPCNT_Msk
  USB_OTG_DOEPTSIZ_STUPCNT_0* = (0x00000001 shl USB_OTG_DOEPTSIZ_STUPCNT_Pos) ## !< 0x20000000
  USB_OTG_DOEPTSIZ_STUPCNT_1* = (0x00000002 shl USB_OTG_DOEPTSIZ_STUPCNT_Pos) ## !< 0x40000000

## *******************  Bit definition for PCGCCTL register  *******************

const
  USB_OTG_PCGCCTL_STOPCLK_Pos* = (0)
  USB_OTG_PCGCCTL_STOPCLK_Msk* = (0x00000001 shl USB_OTG_PCGCCTL_STOPCLK_Pos) ## !< 0x00000001
  USB_OTG_PCGCCTL_STOPCLK* = USB_OTG_PCGCCTL_STOPCLK_Msk
  USB_OTG_PCGCCTL_GATECLK_Pos* = (1)
  USB_OTG_PCGCCTL_GATECLK_Msk* = (0x00000001 shl USB_OTG_PCGCCTL_GATECLK_Pos) ## !< 0x00000002
  USB_OTG_PCGCCTL_GATECLK* = USB_OTG_PCGCCTL_GATECLK_Msk
  USB_OTG_PCGCCTL_PHYSUSP_Pos* = (4)
  USB_OTG_PCGCCTL_PHYSUSP_Msk* = (0x00000001 shl USB_OTG_PCGCCTL_PHYSUSP_Pos) ## !< 0x00000010
  USB_OTG_PCGCCTL_PHYSUSP* = USB_OTG_PCGCCTL_PHYSUSP_Msk

##  Legacy define
## *******************  Bit definition for OTG register  *******************

const
  USB_OTG_CHNUM_Pos* = (0)
  USB_OTG_CHNUM_Msk* = (0x0000000F shl USB_OTG_CHNUM_Pos) ## !< 0x0000000F
  USB_OTG_CHNUM* = USB_OTG_CHNUM_Msk
  USB_OTG_CHNUM_0* = (0x00000001 shl USB_OTG_CHNUM_Pos) ## !< 0x00000001
  USB_OTG_CHNUM_1* = (0x00000002 shl USB_OTG_CHNUM_Pos) ## !< 0x00000002
  USB_OTG_CHNUM_2* = (0x00000004 shl USB_OTG_CHNUM_Pos) ## !< 0x00000004
  USB_OTG_CHNUM_3* = (0x00000008 shl USB_OTG_CHNUM_Pos) ## !< 0x00000008
  USB_OTG_BCNT_Pos* = (4)
  USB_OTG_BCNT_Msk* = (0x000007FF shl USB_OTG_BCNT_Pos) ## !< 0x00007FF0
  USB_OTG_BCNT* = USB_OTG_BCNT_Msk
  USB_OTG_DPID_Pos* = (15)
  USB_OTG_DPID_Msk* = (0x00000003 shl USB_OTG_DPID_Pos) ## !< 0x00018000
  USB_OTG_DPID* = USB_OTG_DPID_Msk
  USB_OTG_DPID_0* = (0x00000001 shl USB_OTG_DPID_Pos) ## !< 0x00008000
  USB_OTG_DPID_1* = (0x00000002 shl USB_OTG_DPID_Pos) ## !< 0x00010000
  USB_OTG_PKTSTS_Pos* = (17)
  USB_OTG_PKTSTS_Msk* = (0x0000000F shl USB_OTG_PKTSTS_Pos) ## !< 0x001E0000
  USB_OTG_PKTSTS* = USB_OTG_PKTSTS_Msk
  USB_OTG_PKTSTS_0* = (0x00000001 shl USB_OTG_PKTSTS_Pos) ## !< 0x00020000
  USB_OTG_PKTSTS_1* = (0x00000002 shl USB_OTG_PKTSTS_Pos) ## !< 0x00040000
  USB_OTG_PKTSTS_2* = (0x00000004 shl USB_OTG_PKTSTS_Pos) ## !< 0x00080000
  USB_OTG_PKTSTS_3* = (0x00000008 shl USB_OTG_PKTSTS_Pos) ## !< 0x00100000
  USB_OTG_EPNUM_Pos* = (0)
  USB_OTG_EPNUM_Msk* = (0x0000000F shl USB_OTG_EPNUM_Pos) ## !< 0x0000000F
  USB_OTG_EPNUM* = USB_OTG_EPNUM_Msk
  USB_OTG_EPNUM_0* = (0x00000001 shl USB_OTG_EPNUM_Pos) ## !< 0x00000001
  USB_OTG_EPNUM_1* = (0x00000002 shl USB_OTG_EPNUM_Pos) ## !< 0x00000002
  USB_OTG_EPNUM_2* = (0x00000004 shl USB_OTG_EPNUM_Pos) ## !< 0x00000004
  USB_OTG_EPNUM_3* = (0x00000008 shl USB_OTG_EPNUM_Pos) ## !< 0x00000008
  USB_OTG_FRMNUM_Pos* = (21)
  USB_OTG_FRMNUM_Msk* = (0x0000000F shl USB_OTG_FRMNUM_Pos) ## !< 0x01E00000
  USB_OTG_FRMNUM* = USB_OTG_FRMNUM_Msk
  USB_OTG_FRMNUM_0* = (0x00000001 shl USB_OTG_FRMNUM_Pos) ## !< 0x00200000
  USB_OTG_FRMNUM_1* = (0x00000002 shl USB_OTG_FRMNUM_Pos) ## !< 0x00400000
  USB_OTG_FRMNUM_2* = (0x00000004 shl USB_OTG_FRMNUM_Pos) ## !< 0x00800000
  USB_OTG_FRMNUM_3* = (0x00000008 shl USB_OTG_FRMNUM_Pos) ## !< 0x01000000

## *
##  @}
##
## *
##  @}
##
## * @addtogroup Exported_macros
##  @{
##
## ****************************** ADC Instances *******************************

template IS_ADC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == ADC1) or ((INSTANCE) == ADC2) or ((INSTANCE) == ADC3))

template IS_ADC_MULTIMODE_MASTER_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == ADC1)

template IS_ADC_COMMON_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == ADC123_COMMON)

## ****************************** CAN Instances *******************************

template IS_CAN_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == CAN1) or ((INSTANCE) == CAN2))

## ****************************** CRC Instances *******************************

template IS_CRC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == CRC)

## ****************************** DAC Instances *******************************

template IS_DAC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == DAC1)

## ****************************** DCMI Instances ******************************

template IS_DCMI_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == DCMI)

## ******************************* DMA Instances ******************************

template IS_DMA_STREAM_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == DMA1_Stream0) or ((INSTANCE) == DMA1_Stream1) or
      ((INSTANCE) == DMA1_Stream2) or ((INSTANCE) == DMA1_Stream3) or
      ((INSTANCE) == DMA1_Stream4) or ((INSTANCE) == DMA1_Stream5) or
      ((INSTANCE) == DMA1_Stream6) or ((INSTANCE) == DMA1_Stream7) or
      ((INSTANCE) == DMA2_Stream0) or ((INSTANCE) == DMA2_Stream1) or
      ((INSTANCE) == DMA2_Stream2) or ((INSTANCE) == DMA2_Stream3) or
      ((INSTANCE) == DMA2_Stream4) or ((INSTANCE) == DMA2_Stream5) or
      ((INSTANCE) == DMA2_Stream6) or ((INSTANCE) == DMA2_Stream7))

## ****************************** GPIO Instances ******************************

template IS_GPIO_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == GPIOA) or ((INSTANCE) == GPIOB) or ((INSTANCE) == GPIOC) or
      ((INSTANCE) == GPIOD) or ((INSTANCE) == GPIOE) or ((INSTANCE) == GPIOF) or
      ((INSTANCE) == GPIOG) or ((INSTANCE) == GPIOH) or ((INSTANCE) == GPIOI))

## ******************************* I2C Instances ******************************

template IS_I2C_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == I2C1) or ((INSTANCE) == I2C2) or ((INSTANCE) == I2C3))

## ****************************** SMBUS Instances *****************************

#const
#  IS_SMBUS_ALL_INSTANCE* = IS_I2C_ALL_INSTANCE

## ******************************* I2S Instances ******************************

template IS_I2S_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == SPI2) or ((INSTANCE) == SPI3))

## ************************** I2S Extended Instances **************************

template IS_I2S_EXT_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == I2S2ext) or ((INSTANCE) == I2S3ext))

##  Legacy Defines

#const
#  IS_I2S_ALL_INSTANCE_EXT* = IS_I2S_EXT_ALL_INSTANCE

## ****************************** RNG Instances *******************************

template IS_RNG_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == RNG)

## ***************************** RTC Instances ********************************

template IS_RTC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == RTC)

## ******************************* SPI Instances ******************************

template IS_SPI_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == SPI1) or ((INSTANCE) == SPI2) or ((INSTANCE) == SPI3))

## ***************** TIM Instances : All supported instances ******************

template IS_TIM_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM6) or
      ((INSTANCE) == TIM7) or ((INSTANCE) == TIM8) or ((INSTANCE) == TIM9) or
      ((INSTANCE) == TIM10) or ((INSTANCE) == TIM11) or ((INSTANCE) == TIM12) or
      ((INSTANCE) == TIM13) or ((INSTANCE) == TIM14))

## ************ TIM Instances : at least 1 capture/compare channel ************

template IS_TIM_CC1_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8) or
      ((INSTANCE) == TIM9) or ((INSTANCE) == TIM10) or ((INSTANCE) == TIM11) or
      ((INSTANCE) == TIM12) or ((INSTANCE) == TIM13) or ((INSTANCE) == TIM14))

## *********** TIM Instances : at least 2 capture/compare channels ************

template IS_TIM_CC2_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8) or
      ((INSTANCE) == TIM9) or ((INSTANCE) == TIM12))

## *********** TIM Instances : at least 3 capture/compare channels ************

template IS_TIM_CC3_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8))

## *********** TIM Instances : at least 4 capture/compare channels ************

template IS_TIM_CC4_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8))

## ******************* TIM Instances : Advanced-control timers ****************

template IS_TIM_ADVANCED_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8))

## ****************** TIM Instances : Timer input XOR function ****************

template IS_TIM_XOR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : DMA requests generation (UDE) ************

template IS_TIM_DMA_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM6) or
      ((INSTANCE) == TIM7) or ((INSTANCE) == TIM8))

## *********** TIM Instances : DMA requests generation (CCxDE) ****************

template IS_TIM_DMA_CC_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8))

## *********** TIM Instances : DMA requests generation (COMDE) ****************

template IS_TIM_CCDMA_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8))

## ******************* TIM Instances : DMA burst feature **********************

template IS_TIM_DMABURST_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8))

## ***** TIM Instances : master mode available (TIMx_CR2.MMS available )*******

template IS_TIM_MASTER_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM6) or
      ((INSTANCE) == TIM7) or ((INSTANCE) == TIM8))

## ********** TIM Instances : Slave mode available (TIMx_SMCR available )******

template IS_TIM_SLAVE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8) or
      ((INSTANCE) == TIM9) or ((INSTANCE) == TIM12))

## ***************** TIM Instances : supporting synchronization ***************

template IS_TIM_SYNCHRO_INSTANCE*(INSTANCE: untyped): untyped =
  IS_TIM_MASTER_INSTANCE(INSTANCE)

## ********************* TIM Instances : 32 bit Counter ***********************

template IS_TIM_32B_COUNTER_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM5))

## **************** TIM Instances : external trigger input availabe ***********

template IS_TIM_ETR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : remapping capability *********************

template IS_TIM_REMAP_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM2) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM11))

## ****************** TIM Instances : output(s) available *********************

template IS_TIM_CCX_INSTANCE*(INSTANCE, CHANNEL: untyped): untyped =
  ((((INSTANCE) == TIM1) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3) or ((CHANNEL) == TIM_CHANNEL_4))) or
      (((INSTANCE) == TIM2) and
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
      (((INSTANCE) == TIM8) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3) or ((CHANNEL) == TIM_CHANNEL_4))) or
      (((INSTANCE) == TIM9) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2))) or
      (((INSTANCE) == TIM10) and (((CHANNEL) == TIM_CHANNEL_1))) or
      (((INSTANCE) == TIM11) and (((CHANNEL) == TIM_CHANNEL_1))) or
      (((INSTANCE) == TIM12) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2))) or
      (((INSTANCE) == TIM13) and (((CHANNEL) == TIM_CHANNEL_1))) or
      (((INSTANCE) == TIM14) and (((CHANNEL) == TIM_CHANNEL_1))))

## *********** TIM Instances : complementary output(s) available **************

template IS_TIM_CCXN_INSTANCE*(INSTANCE, CHANNEL: untyped): untyped =
  ((((INSTANCE) == TIM1) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3))) or
      (((INSTANCE) == TIM8) and
      (((CHANNEL) == TIM_CHANNEL_1) or ((CHANNEL) == TIM_CHANNEL_2) or
      ((CHANNEL) == TIM_CHANNEL_3))))

## ***************** TIM Instances : supporting counting mode selection *******

template IS_TIM_COUNTER_MODE_SELECT_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : supporting clock division ****************

template IS_TIM_CLOCK_DIVISION_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8) or
      ((INSTANCE) == TIM9) or ((INSTANCE) == TIM10) or ((INSTANCE) == TIM11) or
      ((INSTANCE) == TIM12) or ((INSTANCE) == TIM13) or ((INSTANCE) == TIM14))

## ***************** TIM Instances : supporting commutation event generation **

template IS_TIM_COMMUTATION_EVENT_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : supporting OCxREF clear ******************

template IS_TIM_OCXREF_CLEAR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8))

## ***** TIM Instances : supporting external clock mode 1 for ETRF input ******

template IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8) or
      ((INSTANCE) == TIM9) or ((INSTANCE) == TIM12))

## ***** TIM Instances : supporting external clock mode 2 for ETRF input ******

template IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8))

## ***** TIM Instances : supporting external clock mode 1 for TIX inputs *****

template IS_TIM_CLOCKSOURCE_TIX_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8) or
      ((INSTANCE) == TIM9) or ((INSTANCE) == TIM12))

## ********* TIM Instances : supporting internal trigger inputs(ITRX) ********

template IS_TIM_CLOCKSOURCE_ITRX_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8) or
      ((INSTANCE) == TIM9) or ((INSTANCE) == TIM12))

## ***************** TIM Instances : supporting repetition counter ************

template IS_TIM_REPETITION_COUNTER_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : supporting encoder interface *************

template IS_TIM_ENCODER_INTERFACE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8) or
      ((INSTANCE) == TIM9) or ((INSTANCE) == TIM12))

## ***************** TIM Instances : supporting Hall sensor interface *********

template IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM2) or ((INSTANCE) == TIM3) or
      ((INSTANCE) == TIM4) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM8))

## ***************** TIM Instances : supporting the break function ************

template IS_TIM_BREAK_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM8))

## ******************* USART Instances : Synchronous mode *********************

template IS_USART_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3) or
      ((INSTANCE) == USART6))

## ******************* UART Instances : Half-Duplex mode *********************

template IS_UART_HALFDUPLEX_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3) or
      ((INSTANCE) == UART4) or ((INSTANCE) == UART5) or ((INSTANCE) == USART6))

##  Legacy defines

#const
#  IS_UART_INSTANCE* = IS_UART_HALFDUPLEX_INSTANCE

## ***************** UART Instances : Hardware Flow control *******************

template IS_UART_HWFLOW_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3) or
      ((INSTANCE) == USART6))

## ******************* UART Instances : LIN mode *********************

#const
#  IS_UART_LIN_INSTANCE* = IS_UART_HALFDUPLEX_INSTANCE

## ******************** UART Instances : Smart card mode **********************

template IS_SMARTCARD_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3) or
      ((INSTANCE) == USART6))

## ********************** UART Instances : IRDA mode **************************

template IS_IRDA_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART3) or
      ((INSTANCE) == UART4) or ((INSTANCE) == UART5) or ((INSTANCE) == USART6))

## ********************** PCD Instances ***************************************

template IS_PCD_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USB_OTG_FS) or ((INSTANCE) == USB_OTG_HS))

## ********************** HCD Instances ***************************************

template IS_HCD_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USB_OTG_FS) or ((INSTANCE) == USB_OTG_HS))

## ***************************** SDIO Instances *******************************

template IS_SDIO_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == SDIO)

## ***************************** IWDG Instances *******************************

template IS_IWDG_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == IWDG)

## ***************************** WWDG Instances *******************************

template IS_WWDG_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == WWDG)

## ***************************** USB Exported Constants ***********************

const
  USB_OTG_FS_HOST_MAX_CHANNEL_NBR* = 8
  USB_OTG_FS_MAX_IN_ENDPOINTS* = 4
  USB_OTG_FS_MAX_OUT_ENDPOINTS* = 4
  USB_OTG_FS_TOTAL_FIFO_SIZE* = 1280

##
##  @brief Specific devices reset values definitions
##

const
  RCC_PLLCFGR_RST_VALUE* = 0x24003010
  RCC_PLLI2SCFGR_RST_VALUE* = 0x20003000
  RCC_MAX_FREQUENCY* = 168000000
  RCC_MAX_FREQUENCY_SCALE1* = RCC_MAX_FREQUENCY
  RCC_MAX_FREQUENCY_SCALE2* = 144000000
  RCC_PLLVCO_OUTPUT_MIN* = 100000000
  RCC_PLLVCO_INPUT_MIN* = 950000
  RCC_PLLVCO_INPUT_MAX* = 2100000
  RCC_PLLVCO_OUTPUT_MAX* = 432000000
  RCC_PLLN_MIN_VALUE* = 50
  RCC_PLLN_MAX_VALUE* = 432
  FLASH_SCALE1_LATENCY1_FREQ* = 30000000
  FLASH_SCALE1_LATENCY2_FREQ* = 60000000
  FLASH_SCALE1_LATENCY3_FREQ* = 90000000
  FLASH_SCALE1_LATENCY4_FREQ* = 120000000
  FLASH_SCALE1_LATENCY5_FREQ* = 150000000
  FLASH_SCALE2_LATENCY1_FREQ* = 30000000
  FLASH_SCALE2_LATENCY2_FREQ* = 60000000
  FLASH_SCALE2_LATENCY3_FREQ* = 90000000
  FLASH_SCALE2_LATENCY4_FREQ* = 12000000
  USB_OTG_HS_HOST_MAX_CHANNEL_NBR* = 12
  USB_OTG_HS_MAX_IN_ENDPOINTS* = 6
  USB_OTG_HS_MAX_OUT_ENDPOINTS* = 6
  USB_OTG_HS_TOTAL_FIFO_SIZE* = 4096

## ****************************************************************************
##   For a painless codes migration between the STM32F4xx device product
##   lines, the aliases defined below are put in place to overcome the
##   differences in the interrupt handlers and IRQn definitions.
##   No need to update developed interrupt code when moving across
##   product lines within the same STM32F4 Family
## ****************************************************************************
##  Aliases for __IRQn

const
  FMC_IRQn* = FSMC_IRQn

##  Aliases for __IRQHandler

#const FMC_IRQHandler* = FSMC_IRQHandler

## *
##  @}
##
## *
##  @}
##
## *
##  @}
##

## *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE***
include "reg_utils"
include "core_cm4"
