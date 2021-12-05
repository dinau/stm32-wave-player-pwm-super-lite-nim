# ************************************************************************
# *
#  @file     core_cm0.h
#  @brief    CMSIS Cortex-M0 Core Peripheral Access Layer Header File
#  @version  V4.30
#  @date     20. October 2015
# ****************************************************************************
#  Copyright (c) 2009 - 2015 ARM LIMITED
#
#    All rights reserved.
#    Redistribution and use in source and binary forms, with or without
#    modification, are permitted provided that the following conditions are met:
#    - Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    - Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    - Neither the name of ARM nor the names of its contributors may be used
#      to endorse or promote products derived from this software without
#      specific prior written permission.
#
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#    ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
#    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#    POSSIBILITY OF SUCH DAMAGE.
#    ---------------------------------------------------------------------------

# *
#   \page CMSIS_MISRA_Exceptions  MISRA-C:2004 Compliance Exceptions
#   CMSIS violates the following MISRA-C:2004 rules:
#
#    \li Required Rule 8.5, object/function definition in header file.<br>
#      Function definitions in header files are used to allow 'inlining'.
#
#    \li Required Rule 18.4, declaration of union type or object of union type: '{...}'.<br>
#      Unions are used for effective representation of core registers.
#
#    \li Advisory Rule 19.7, Function-like macro defined.<br>
#      Function-like macros are used to allow more efficient code.
#
# ******************************************************************************
#                  CMSIS definitions
# ****************************************************************************
# *
#   \ingroup Cortex_M0
#   @{
#
#   CMSIS CM0 definitions
const
  CM0_CMSIS_VERSION_MAIN* = (0x00000004) # !< [31:16] CMSIS HAL main version
  CM0_CMSIS_VERSION_SUB* = (0x0000001E) # !< [15:0]  CMSIS HAL sub version
  CM0_CMSIS_VERSION* = ((CM0_CMSIS_VERSION_MAIN shl 16) or CM0_CMSIS_VERSION_SUB) # !< CMSIS HAL version number
  CORTEX_M* = (0x00000000)      # !< Cortex-M Core

# #define FPU_USED       0U

when not defined(CMSIS_GENERIC):
  #  check device defines and use defaults
  #const
    #CM0_REV* = 0x00000000
    #NVIC_PRIO_BITS* = 2
    #Vendor_SysTickConfig* = 0
  #  IO definitions (access restrictions to peripheral registers)
  # *
  #     \defgroup CMSIS_glob_defs CMSIS Global Defines
  #
  #     <strong>IO Type Qualifiers</strong> are used
  #     \li to specify the access to peripheral variables.
  #     \li for automatic generation of peripheral register debug information.
  #
  #   #define   I     volatile const       /*!< Defines 'read only' permissions */
  # #define     O     volatile             /*!< Defines 'write only' permissions */
  # #define     IO    volatile
  #  following defines should be used for structure members
  # #define     IM     volatile const      /*! Defines 'read only' structure member permissions */
  # #define     OM     volatile            /*! Defines 'write only' structure member permissions */
  # #define     IOM    volatile            /*! Defines 'read / write' structure member permissions */
  # @} end of group Cortex_M0
  # ******************************************************************************
  #                  Register Abstraction
  #   Core Register contain:
  #   - Core Register
  #   - Core NVIC Register
  #   - Core SCB Register
  #   - Core SysTick Register
  # ****************************************************************************
  # *
  #   \defgroup CMSIS_core_register Defines and Type Definitions
  #   \brief Type definitions and defines for Cortex-M processor based devices.
  #
  # *
  #   \ingroup    CMSIS_core_register
  #   \defgroup   CMSIS_CORE  Status and Control Registers
  #   \brief      Core Register type definitions.
  #   @{
  #
  # *
  #   \brief  Union type to access the Application Program Status Register (APSR).
  #
  type
    INNER_C_STRUCT_core_cm0_146* {.bycopy.} = object
      reserved0* {.bitsize: 28.}: uint32 # !< bit:  0..27  Reserved
      V* {.bitsize: 1.}: uint32 # !< bit:     28  Overflow condition code flag
      C* {.bitsize: 1.}: uint32 # !< bit:     29  Carry condition code flag
      Z* {.bitsize: 1.}: uint32 # !< bit:     30  Zero condition code flag
      N* {.bitsize: 1.}: uint32 # !< bit:     31  Negative condition code flag

  type
    APSR_Type* {.bycopy.} = object
      b*: INNER_C_STRUCT_core_cm0_146 # !< Structure used for bit  access
      w*: uint32             # !< Type      used for word access

  #  APSR Register Definitions
  const
    APSR_N_Pos* = 31
    APSR_N_Msk* = (1 shl APSR_N_Pos) # !< APSR: N Mask
    APSR_Z_Pos* = 30
    APSR_Z_Msk* = (1 shl APSR_Z_Pos) # !< APSR: Z Mask
    APSR_C_Pos* = 29
    APSR_C_Msk* = (1 shl APSR_C_Pos) # !< APSR: C Mask
    APSR_V_Pos* = 28
    APSR_V_Msk* = (1 shl APSR_V_Pos) # !< APSR: V Mask
  # *
  #   \brief  Union type to access the Interrupt Program Status Register (IPSR).
  #
  type
    INNER_C_STRUCT_core_cm0_176* {.bycopy.} = object
      ISR* {.bitsize: 9.}: uint32 # !< bit:  0.. 8  Exception number
      reserved0* {.bitsize: 23.}: uint32 # !< bit:  9..31  Reserved

  type
    IPSR_Type* {.bycopy.} = object
      b*: INNER_C_STRUCT_core_cm0_176 # !< Structure used for bit  access
      w*: uint32             # !< Type      used for word access

  #  IPSR Register Definitions
  const
    IPSR_ISR_Pos* = 0
    IPSR_ISR_Msk* = (0x000001FF) # !< IPSR: ISR Mask
  # *
  #   \brief  Union type to access the Special-Purpose Program Status Registers (xPSR).
  #
  type
    INNER_C_STRUCT_core_cm0_194* {.bycopy.} = object
      ISR* {.bitsize: 9.}: uint32 # !< bit:  0.. 8  Exception number
      reserved0* {.bitsize: 15.}: uint32 # !< bit:  9..23  Reserved
      T* {.bitsize: 1.}: uint32 # !< bit:     24  Thumb bit        (read 0)
      reserved1* {.bitsize: 3.}: uint32 # !< bit: 25..27  Reserved
      V* {.bitsize: 1.}: uint32 # !< bit:     28  Overflow condition code flag
      C* {.bitsize: 1.}: uint32 # !< bit:     29  Carry condition code flag
      Z* {.bitsize: 1.}: uint32 # !< bit:     30  Zero condition code flag
      N* {.bitsize: 1.}: uint32 # !< bit:     31  Negative condition code flag

  type
    xPSR_Type* {.bycopy.} = object
      b*: INNER_C_STRUCT_core_cm0_194 # !< Structure used for bit  access
      w*: uint32             # !< Type      used for word access

  #  xPSR Register Definitions
  const
    xPSR_N_Pos* = 31
    xPSR_N_Msk* = (1 shl xPSR_N_Pos) # !< xPSR: N Mask
    xPSR_Z_Pos* = 30
    xPSR_Z_Msk* = (1 shl xPSR_Z_Pos) # !< xPSR: Z Mask
    xPSR_C_Pos* = 29
    xPSR_C_Msk* = (1 shl xPSR_C_Pos) # !< xPSR: C Mask
    xPSR_V_Pos* = 28
    xPSR_V_Msk* = (1 shl xPSR_V_Pos) # !< xPSR: V Mask
    xPSR_T_Pos* = 24
    xPSR_T_Msk* = (1 shl xPSR_T_Pos) # !< xPSR: T Mask
    xPSR_ISR_Pos* = 0
    xPSR_ISR_Msk* = (0x000001FF) # !< xPSR: ISR Mask
  # *
  #   \brief  Union type to access the Control Registers (CONTROL).
  #
  type
    INNER_C_STRUCT_core_cm0_233* {.bycopy.} = object
      reserved0* {.bitsize: 1.}: uint32 # !< bit:      0  Reserved
      SPSEL* {.bitsize: 1.}: uint32 # !< bit:      1  Stack to be used
      reserved1* {.bitsize: 30.}: uint32 # !< bit:  2..31  Reserved

  type
    CONTROL_Type* {.bycopy.} = object
      b*: INNER_C_STRUCT_core_cm0_233 # !< Structure used for bit  access
      w*: uint32             # !< Type      used for word access

  #  CONTROL Register Definitions
  const
    CONTROL_SPSEL_Pos* = 1
    CONTROL_SPSEL_Msk* = (1 shl CONTROL_SPSEL_Pos) # !< CONTROL: SPSEL Mask
  # @} end of group CMSIS_CORE
  # *
  #   \ingroup    CMSIS_core_register
  #   \defgroup   CMSIS_NVIC  Nested Vectored Interrupt Controller (NVIC)
  #   \brief      Type definitions for the NVIC Registers
  #   @{
  #
  # *
  #   \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
  #
  type
    NVIC_Type* {.bycopy.} = object
      ISER*: array[1, uint32] # !< Offset: 0x000 (R/W)  Interrupt Set Enable Register
      RESERVED0*: array[31, uint32]
      ICER*: array[1, uint32] # !< Offset: 0x080 (R/W)  Interrupt Clear Enable Register
      RSERVED1*: array[31, uint32]
      ISPR*: array[1, uint32] # !< Offset: 0x100 (R/W)  Interrupt Set Pending Register
      RESERVED2*: array[31, uint32]
      ICPR*: array[1, uint32] # !< Offset: 0x180 (R/W)  Interrupt Clear Pending Register
      RESERVED3*: array[31, uint32]
      RESERVED4*: array[64, uint32]
      IP*: array[8, uint32]   # !< Offset: 0x300 (R/W)  Interrupt Priority Register

  # @} end of group CMSIS_NVIC
  # *
  #   \ingroup  CMSIS_core_register
  #   \defgroup CMSIS_SCB     System Control Block (SCB)
  #   \brief    Type definitions for the System Control Block Registers
  #   @{
  #
  # *
  #   \brief  Structure type to access the System Control Block (SCB).
  #
  type
    SCB_Type* {.bycopy.} = object
      CPUID*: uint32         # !< Offset: 0x000 (R/ )  CPUID Base Register
      ICSR*: uint32          # !< Offset: 0x004 (R/W)  Interrupt Control and State Register
      RESERVED0*: uint32
      AIRCR*: uint32         # !< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register
      SCR*: uint32           # !< Offset: 0x010 (R/W)  System Control Register
      CCR*: uint32           # !< Offset: 0x014 (R/W)  Configuration Control Register
      RESERVED1*: uint32
      SHP*: array[2, uint32]  # !< Offset: 0x01C (R/W)  System Handlers Priority Registers. [0] is RESERVED
      SHCSR*: uint32         # !< Offset: 0x024 (R/W)  System Handler Control and State Register

  #  SCB CPUID Register Definitions
  const
    SCB_CPUID_IMPLEMENTER_Pos* = 24
    SCB_CPUID_IMPLEMENTER_Msk* = (0x000000FF shl SCB_CPUID_IMPLEMENTER_Pos) # !< SCB CPUID: IMPLEMENTER Mask
    SCB_CPUID_VARIANT_Pos* = 20
    SCB_CPUID_VARIANT_Msk* = (0x0000000F shl SCB_CPUID_VARIANT_Pos) # !< SCB CPUID: VARIANT Mask
    SCB_CPUID_ARCHITECTURE_Pos* = 16
    SCB_CPUID_ARCHITECTURE_Msk* = (0x0000000F shl SCB_CPUID_ARCHITECTURE_Pos) # !< SCB CPUID: ARCHITECTURE Mask
    SCB_CPUID_PARTNO_Pos* = 4
    SCB_CPUID_PARTNO_Msk* = (0x00000FFF shl SCB_CPUID_PARTNO_Pos) # !< SCB CPUID: PARTNO Mask
    SCB_CPUID_REVISION_Pos* = 0
    SCB_CPUID_REVISION_Msk* = (0x0000000F) # !< SCB CPUID: REVISION Mask
  #  SCB Interrupt Control State Register Definitions
  const
    SCB_ICSR_NMIPENDSET_Pos* = 31
    SCB_ICSR_NMIPENDSET_Msk* = (1 shl SCB_ICSR_NMIPENDSET_Pos) # !< SCB ICSR: NMIPENDSET Mask
    SCB_ICSR_PENDSVSET_Pos* = 28
    SCB_ICSR_PENDSVSET_Msk* = (1 shl SCB_ICSR_PENDSVSET_Pos) # !< SCB ICSR: PENDSVSET Mask
    SCB_ICSR_PENDSVCLR_Pos* = 27
    SCB_ICSR_PENDSVCLR_Msk* = (1 shl SCB_ICSR_PENDSVCLR_Pos) # !< SCB ICSR: PENDSVCLR Mask
    SCB_ICSR_PENDSTSET_Pos* = 26
    SCB_ICSR_PENDSTSET_Msk* = (1 shl SCB_ICSR_PENDSTSET_Pos) # !< SCB ICSR: PENDSTSET Mask
    SCB_ICSR_PENDSTCLR_Pos* = 25
    SCB_ICSR_PENDSTCLR_Msk* = (1 shl SCB_ICSR_PENDSTCLR_Pos) # !< SCB ICSR: PENDSTCLR Mask
    SCB_ICSR_ISRPREEMPT_Pos* = 23
    SCB_ICSR_ISRPREEMPT_Msk* = (1 shl SCB_ICSR_ISRPREEMPT_Pos) # !< SCB ICSR: ISRPREEMPT Mask
    SCB_ICSR_ISRPENDING_Pos* = 22
    SCB_ICSR_ISRPENDING_Msk* = (1 shl SCB_ICSR_ISRPENDING_Pos) # !< SCB ICSR: ISRPENDING Mask
    SCB_ICSR_VECTPENDING_Pos* = 12
    SCB_ICSR_VECTPENDING_Msk* = (0x000001FF shl SCB_ICSR_VECTPENDING_Pos) # !< SCB ICSR: VECTPENDING Mask
    SCB_ICSR_VECTACTIVE_Pos* = 0
    SCB_ICSR_VECTACTIVE_Msk* = (0x000001FF) # !< SCB ICSR: VECTACTIVE Mask
  #  SCB Application Interrupt and Reset Control Register Definitions
  const
    SCB_AIRCR_VECTKEY_Pos* = 16
    SCB_AIRCR_VECTKEY_Msk* = (0x0000FFFF shl SCB_AIRCR_VECTKEY_Pos) # !< SCB AIRCR: VECTKEY Mask
    SCB_AIRCR_VECTKEYSTAT_Pos* = 16
    SCB_AIRCR_VECTKEYSTAT_Msk* = (0x0000FFFF shl SCB_AIRCR_VECTKEYSTAT_Pos) # !< SCB AIRCR: VECTKEYSTAT Mask
    SCB_AIRCR_ENDIANESS_Pos* = 15
    SCB_AIRCR_ENDIANESS_Msk* = (1 shl SCB_AIRCR_ENDIANESS_Pos) # !< SCB AIRCR: ENDIANESS Mask
    SCB_AIRCR_SYSRESETREQ_Pos* = 2
    SCB_AIRCR_SYSRESETREQ_Msk* = (1 shl SCB_AIRCR_SYSRESETREQ_Pos) # !< SCB AIRCR: SYSRESETREQ Mask
    SCB_AIRCR_VECTCLRACTIVE_Pos* = 1
    SCB_AIRCR_VECTCLRACTIVE_Msk* = (1 shl SCB_AIRCR_VECTCLRACTIVE_Pos) # !< SCB AIRCR: VECTCLRACTIVE Mask
  #  SCB System Control Register Definitions
  const
    SCB_SCR_SEVONPEND_Pos* = 4
    SCB_SCR_SEVONPEND_Msk* = (1 shl SCB_SCR_SEVONPEND_Pos) # !< SCB SCR: SEVONPEND Mask
    SCB_SCR_SLEEPDEEP_Pos* = 2
    SCB_SCR_SLEEPDEEP_Msk* = (1 shl SCB_SCR_SLEEPDEEP_Pos) # !< SCB SCR: SLEEPDEEP Mask
    SCB_SCR_SLEEPONEXIT_Pos* = 1
    SCB_SCR_SLEEPONEXIT_Msk* = (1 shl SCB_SCR_SLEEPONEXIT_Pos) # !< SCB SCR: SLEEPONEXIT Mask
  #  SCB Configuration Control Register Definitions
  const
    SCB_CCR_STKALIGN_Pos* = 9
    SCB_CCR_STKALIGN_Msk* = (1 shl SCB_CCR_STKALIGN_Pos) # !< SCB CCR: STKALIGN Mask
    SCB_CCR_UNALIGN_TRP_Pos* = 3
    SCB_CCR_UNALIGN_TRP_Msk* = (1 shl SCB_CCR_UNALIGN_TRP_Pos) # !< SCB CCR: UNALIGN_TRP Mask
  #  SCB System Handler Control and State Register Definitions
  const
    SCB_SHCSR_SVCALLPENDED_Pos* = 15
    SCB_SHCSR_SVCALLPENDED_Msk* = (1 shl SCB_SHCSR_SVCALLPENDED_Pos) # !< SCB SHCSR: SVCALLPENDED Mask
  # @} end of group CMSIS_SCB
  # *
  #   \ingroup  CMSIS_core_register
  #   \defgroup CMSIS_SysTick     System Tick Timer (SysTick)
  #   \brief    Type definitions for the System Timer Registers.
  #   @{
  #
  # *
  #   \brief  Structure type to access the System Timer (SysTick).
  #
  type
    SysTick_Type* {.bycopy.} = object
      CTRL*: uint32          # !< Offset: 0x000 (R/W)  SysTick Control and Status Register
      LOAD*: uint32          # !< Offset: 0x004 (R/W)  SysTick Reload Value Register
      VAL*: uint32           # !< Offset: 0x008 (R/W)  SysTick Current Value Register
      CALIB*: uint32         # !< Offset: 0x00C (R/ )  SysTick Calibration Register

  #  SysTick Control / Status Register Definitions
  const
    SysTick_CTRL_COUNTFLAG_Pos* = 16
    SysTick_CTRL_COUNTFLAG_Msk* = (1 shl SysTick_CTRL_COUNTFLAG_Pos) # !< SysTick CTRL: COUNTFLAG Mask
    SysTick_CTRL_CLKSOURCE_Pos* = 2
    SysTick_CTRL_CLKSOURCE_Msk* = (1 shl SysTick_CTRL_CLKSOURCE_Pos) # !< SysTick CTRL: CLKSOURCE Mask
    SysTick_CTRL_TICKINT_Pos* = 1
    SysTick_CTRL_TICKINT_Msk* = (1 shl SysTick_CTRL_TICKINT_Pos) # !< SysTick CTRL: TICKINT Mask
    SysTick_CTRL_ENABLE_Pos* = 0
    SysTick_CTRL_ENABLE_Msk* = (1) # !< SysTick CTRL: ENABLE Mask
  #  SysTick Reload Register Definitions
  const
    SysTick_LOAD_RELOAD_Pos* = 0
    SysTick_LOAD_RELOAD_Msk* = (0x00FFFFFF) # !< SysTick LOAD: RELOAD Mask
  #  SysTick Current Register Definitions
  const
    SysTick_VAL_CURRENT_Pos* = 0
    SysTick_VAL_CURRENT_Msk* = (0x00FFFFFF) # !< SysTick VAL: CURRENT Mask
  #  SysTick Calibration Register Definitions
  const
    SysTick_CALIB_NOREF_Pos* = 31
    SysTick_CALIB_NOREF_Msk* = (1 shl SysTick_CALIB_NOREF_Pos) # !< SysTick CALIB: NOREF Mask
    SysTick_CALIB_SKEW_Pos* = 30
    SysTick_CALIB_SKEW_Msk* = (1 shl SysTick_CALIB_SKEW_Pos) # !< SysTick CALIB: SKEW Mask
    SysTick_CALIB_TENMS_Pos* = 0
    SysTick_CALIB_TENMS_Msk* = (0x00FFFFFF) # !< SysTick CALIB: TENMS Mask
  # @} end of group CMSIS_SysTick
  # *
  #   \ingroup  CMSIS_core_register
  #   \defgroup CMSIS_CoreDebug       Core Debug Registers (CoreDebug)
  #   \brief    Cortex-M0 Core Debug Registers (DCB registers, SHCSR, and DFSR) are only accessible over DAP and not via processor.
  #             Therefore they are not covered by the Cortex-M0 header file.
  #   @{
  #
  # @} end of group CMSIS_CoreDebug
  # *
  #   \ingroup    CMSIS_core_register
  #   \defgroup   CMSIS_core_bitfield     Core register bit field macros
  #   \brief      Macros for use with bit field definitions (xxx_Pos, xxx_Msk).
  #   @{
  #
  # *
  #   \brief   Mask and shift a bit field value for use in a register bit range.
  #   \param[in] field  Name of the register bit field.
  #   \param[in] value  Value of the bit field.
  #   \return           Masked and shifted value.
  #
  # #define _VAL2FLD(field, value)    ((value << field # _Pos) & field # _Msk)
  # *
  #   \brief     Mask and shift a register value to extract a bit filed value.
  #   \param[in] field  Name of the register bit field.
  #   \param[in] value  Value of register.
  #   \return           Masked and shifted bit field value.
  #
  # #define _FLD2VAL(field, value)    ((value & field # _Msk) >> field # _Pos)
  # @} end of group CMSIS_core_bitfield
  # *
  #   \ingroup    CMSIS_core_register
  #   \defgroup   CMSIS_core_base     Core Definitions
  #   \brief      Definitions for base addresses, unions, and structures.
  #   @{
  #
  #  Memory mapping of Cortex-M0 Hardware
  const
    SCS_BASE* = (0xE000E000)    # !< System Control Space Base Address
    SysTick_BASE* = (SCS_BASE + 0x00000010) # !< SysTick Base Address
    NVIC_BASE* = (SCS_BASE + 0x00000100) # !< NVIC Base Address
    SCB_BASE* = (SCS_BASE + 0x00000D00) # !< System Control Block Base Address
    SCB* = (cast[ptr SCB_Type](SCB_BASE)) # !< SCB configuration struct
    SysTick* = (cast[ptr SysTick_Type](SysTick_BASE)) # !< SysTick configuration struct
    NVIC* = (cast[ptr NVIC_Type](NVIC_BASE)) # !< NVIC configuration struct
  # @}
  # ******************************************************************************
  #                 Hardware Abstraction Layer
  #   Core Function Interface contains:
  #   - Core NVIC Functions
  #   - Core SysTick Functions
  #   - Core Register Access Functions
  # ****************************************************************************
  # *
  #   \defgroup CMSIS_Core_FunctionInterface Functions and Instructions Reference
  #
  #  #############   NVIC functions  ##################
  # *
  #   \ingroup  CMSIS_Core_FunctionInterface
  #   \defgroup CMSIS_Core_NVICFunctions NVIC Functions
  #   \brief    Functions that manage interrupts and exceptions via the NVIC.
  #   @{
  #
  #  Interrupt Priorities are WORD accessible only under ARMv6M
  #  The following MACROS handle generation of the register offset and byte masks
  template BIT_SHIFT*(IRQn: untyped): untyped =
    (((((uint32)((int32)(IRQn)))) and 0x00000003) * 8)

  template SHP_IDX*(IRQn: untyped): untyped =
    ((((((uint32)((int32)(IRQn))) and 0x0000000F) - 8) shr 2))

  template IP_IDX*(IRQn: untyped): untyped =
    ((((uint32)((int32)(IRQn))) shr 2))

  # *
  #   \brief   Enable External Interrupt
  #   \details Enables a device-specific interrupt in the NVIC interrupt controller.
  #   \param [in]      IRQn  External interrupt number. Value cannot be negative.
  #
  proc NVIC_EnableIRQ*(IRQn: IRQn_Type) {.inline.} =
    NVIC.ISER[0].st (uint32)(1 shl
        (((uint32)(cast[int32](IRQn))) and 0x0000001F))

  # *
  #   \brief   Disable External Interrupt
  #   \details Disables a device-specific interrupt in the NVIC interrupt controller.
  #   \param [in]      IRQn  External interrupt number. Value cannot be negative.
  #
  proc NVIC_DisableIRQ*(IRQn: IRQn_Type) {.inline.} =
    NVIC.ICER[0].st (uint32)(1 shl
        (((uint32)(cast[int32](IRQn))) and 0x0000001F))

  # *
  #   \brief   Get Pending Interrupt
  #   \details Reads the pending register in the NVIC and returns the pending bit for the specified interrupt.
  #   \param [in]      IRQn  Interrupt number.
  #   \return             0  Interrupt status is not pending.
  #   \return             1  Interrupt status is pending.
  #
  proc NVIC_GetPendingIRQ*(IRQn: IRQn_Type): uint32 {.inline.} =
    return (uint32)(if( ( NVIC.ISPR[0].ld and
            (uint32)(1 shl (((uint32)(cast[int32](IRQn))) and 0x0000001F))) != 0): 1 else: 0)

  # *
  #   \brief   Set Pending Interrupt
  #   \details Sets the pending bit of an external interrupt.
  #   \param [in]      IRQn  Interrupt number. Value cannot be negative.
  #
  proc NVIC_SetPendingIRQ*(IRQn: IRQn_Type) {.inline.} =
    NVIC.ISPR[0].st (uint32)(1 shl
        (((uint32)(cast[int32](IRQn))) and 0x0000001F))

  # *
  #   \brief   Clear Pending Interrupt
  #   \details Clears the pending bit of an external interrupt.
  #   \param [in]      IRQn  External interrupt number. Value cannot be negative.
  #
  proc NVIC_ClearPendingIRQ*(IRQn: IRQn_Type) {.inline.} =
    NVIC.ICPR[0].st (uint32)(1 shl
        (((uint32)(cast[int32](IRQn))) and 0x0000001F))

  # *
  #   \brief   Set Interrupt Priority
  #   \details Sets the priority of an interrupt.
  #   \note    The priority cannot be set for every core interrupt.
  #   \param [in]      IRQn  Interrupt number.
  #   \param [in]  priority  Priority to set.
  #
  proc NVIC_SetPriority*(IRQn: IRQn_Type; priority: uint32) {.inline.} =
    if (int32)(IRQn) < 0:
      SCB.SHP[SHP_IDX(IRQn)].st ((uint32)(
          SCB.SHP[SHP_IDX(IRQn)].ld and not (uint32)(0x000000FF shl BIT_SHIFT(IRQn))) or
          (((priority shl (8 - NVIC_PRIO_BITS)) and cast[uint32](0x000000FF)) shl
          BIT_SHIFT(IRQn)))
    else:
      NVIC.IP[IP_IDX(IRQn)].st ((uint32)(
          NVIC.IP[IP_IDX(IRQn)].ld and not (uint32)(0x000000FF shl BIT_SHIFT(IRQn))) or
          (((priority shl (8 - NVIC_PRIO_BITS)) and cast[uint32](0x000000FF)) shl
          BIT_SHIFT(IRQn)))

  # *
  #   \brief   Get Interrupt Priority
  #   \details Reads the priority of an interrupt.
  #            The interrupt number can be positive to specify an external (device specific) interrupt,
  #            or negative to specify an internal (core) interrupt.
  #   \param [in]   IRQn  Interrupt number.
  #   \return             Interrupt Priority.
  #                       Value is aligned automatically to the implemented priority bits of the microcontroller.
  #
  proc NVIC_GetPriority*(IRQn: IRQn_Type): uint32 {.inline.} =
    if (int32)(IRQn) < 0:
      return (uint32)(((SCB.SHP[SHP_IDX(IRQn)].ld shr BIT_SHIFT(IRQn)) and
          cast[uint32](0x000000FF)) shr (8 - NVIC_PRIO_BITS))
    else:
      return (uint32)(((NVIC.IP[IP_IDX(IRQn)].ld shr BIT_SHIFT(IRQn)) and
          cast[uint32](0x000000FF)) shr (8 - NVIC_PRIO_BITS))

  # *
  #   \brief   System Reset
  #   \details Initiates a system reset request to reset the MCU.
  #
  proc NVIC_SystemReset*() {.inline.} =
      DSB()
      #  Ensure all outstanding memory accesses included
      #                                                                        buffered write are completed before reset
      SCB.AIRCR = ((0x000005FA shl SCB_AIRCR_VECTKEY_Pos) or
          SCB_AIRCR_SYSRESETREQ_Msk)
      DSB()
      #  Ensure completion of memory access
      while true:
        NOP()

  # @} end of CMSIS_Core_NVICFunctions
  #  #################    SysTick function  ######################
  # *
  #   \ingroup  CMSIS_Core_FunctionInterface
  #   \defgroup CMSIS_Core_SysTickFunctions SysTick Functions
  #   \brief    Functions that configure the System.
  #   @{
  #
  when (Vendor_SysTickConfig == 0):
    # *
    #   \brief   System Tick Configuration
    #   \details Initializes the System Timer and its interrupt, and starts the System Tick Timer.
    #            Counter is in free running mode to generate periodic interrupts.
    #   \param [in]  ticks  Number of ticks between two interrupts.
    #   \return          0  Function succeeded.
    #   \return          1  Function failed.
    #   \note    When the variable <b>Vendor_SysTickConfig</b> is set to 1, then the
    #            function <b>SysTick_Config</b> is not included. In this case, the file <b><i>device</i>.h</b>
    #            must contain a vendor-specific implementation of this function.
    #
    proc SysTick_Config*(ticks: uint32): uint32 {.inline,discardable.} =
      if (ticks - 1) > SysTick_LOAD_RELOAD_Msk.uint32:
        return 1 #  Reload value impossible
      SysTick.LOAD.st  (uint32)(ticks - 1) #  set reload register
      NVIC_SetPriority(SysTick_IRQn, (1 shl NVIC_PRIO_BITS) - 1) #  set Priority for Systick Interrupt
      SysTick.VAL.st 0 #  Load the SysTick Counter Value
      SysTick.CTRL.st  (SysTick_CTRL_CLKSOURCE_Msk or SysTick_CTRL_TICKINT_Msk or
          SysTick_CTRL_ENABLE_Msk) #  Enable SysTick IRQ and SysTick Timer
      return 0 #  Function successful

  # @} end of CMSIS_Core_SysTickFunctions
