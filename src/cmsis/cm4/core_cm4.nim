# ************************************************************************
# *
#  @file     core_cm4.h
#  @brief    CMSIS Cortex-M4 Core Peripheral Access Layer Header File
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
#   \ingroup Cortex_M4
#   @{
#
#   CMSIS CM4 definitions

const
  CM4_CMSIS_VERSION_MAIN* = (0x00000004) # !< [31:16] CMSIS HAL main version
  CM4_CMSIS_VERSION_SUB* = (0x0000001E) # !< [15:0]  CMSIS HAL sub version
  CM4_CMSIS_VERSION* = ((CM4_CMSIS_VERSION_MAIN shl 16) or CM4_CMSIS_VERSION_SUB) # !< CMSIS HAL version number
  CORTEX_M* = (0x00000004)      # !< Cortex-M Core

# #if   defined ( __CC_ARM )
#   #define __ASM            __asm                                      /*!< asm keyword for ARM Compiler */
#   #define __INLINE         __inline                                   /*!< inline keyword for ARM Compiler */
#   #define static __inline  static __inline
#
# #elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#   #define __ASM            __asm                                      /*!< asm keyword for ARM Compiler */
#   #define __INLINE         __inline                                   /*!< inline keyword for ARM Compiler */
#   #define static __inline  static __inline
#
# #endif
# * __FPU_USED indicates whether an FPU is used or not.
#     For this, __FPU_PRESENT has to be checked prior to making use of FPU specific registers and functions.
#

when defined(GNUC):
  when defined(VFP_FP) and not defined(SOFTFP):
    when (FPU_PRESENT == 1):
      const
        FPU_USED* = 1
    else:
      const
        FPU_USED* = 0
  else:
    const
      FPU_USED* = 0
when not defined(CMSIS_GENERIC):
  type
    INNER_C_STRUCT_core_cm4_184* {.bycopy.} = object
      reserved0* {.bitsize: 16.}: uint32 # !< bit:  0..15  Reserved
      GE* {.bitsize: 4.}: uint32 # !< bit: 16..19  Greater than or Equal flags
      reserved1* {.bitsize: 7.}: uint32 # !< bit: 20..26  Reserved
      Q* {.bitsize: 1.}: uint32  # !< bit:     27  Saturation condition flag
      V* {.bitsize: 1.}: uint32  # !< bit:     28  Overflow condition code flag
      C* {.bitsize: 1.}: uint32  # !< bit:     29  Carry condition code flag
      Z* {.bitsize: 1.}: uint32  # !< bit:     30  Zero condition code flag
      N* {.bitsize: 1.}: uint32  # !< bit:     31  Negative condition code flag

  type
    APSR_Type* {.bycopy.} = object
      b*: INNER_C_STRUCT_core_cm4_184 # !< Structure used for bit  access
      w*: uint32               # !< Type      used for word access

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
    APSR_Q_Pos* = 27
    APSR_Q_Msk* = (1 shl APSR_Q_Pos) # !< APSR: Q Mask
    APSR_GE_Pos* = 16
    APSR_GE_Msk* = (0x0000000F shl APSR_GE_Pos) # !< APSR: GE Mask
  # *
  #   \brief  Union type to access the Interrupt Program Status Register (IPSR).
  #
  type
    INNER_C_STRUCT_core_cm4_223* {.bycopy.} = object
      ISR* {.bitsize: 9.}: uint32 # !< bit:  0.. 8  Exception number
      reserved0* {.bitsize: 23.}: uint32 # !< bit:  9..31  Reserved

  type
    IPSR_Type* {.bycopy.} = object
      b*: INNER_C_STRUCT_core_cm4_223 # !< Structure used for bit  access
      w*: uint32               # !< Type      used for word access

  #  IPSR Register Definitions
  const
    IPSR_ISR_Pos* = 0
    IPSR_ISR_Msk* = (0x000001FF) # !< IPSR: ISR Mask
  # *
  #   \brief  Union type to access the Special-Purpose Program Status Registers (xPSR).
  #
  type
    INNER_C_STRUCT_core_cm4_241* {.bycopy.} = object
      ISR* {.bitsize: 9.}: uint32 # !< bit:  0.. 8  Exception number
      reserved0* {.bitsize: 7.}: uint32 # !< bit:  9..15  Reserved
      GE* {.bitsize: 4.}: uint32 # !< bit: 16..19  Greater than or Equal flags
      reserved1* {.bitsize: 4.}: uint32 # !< bit: 20..23  Reserved
      T* {.bitsize: 1.}: uint32  # !< bit:     24  Thumb bit        (read 0)
      IT* {.bitsize: 2.}: uint32 # !< bit: 25..26  saved IT state   (read 0)
      Q* {.bitsize: 1.}: uint32  # !< bit:     27  Saturation condition flag
      V* {.bitsize: 1.}: uint32  # !< bit:     28  Overflow condition code flag
      C* {.bitsize: 1.}: uint32  # !< bit:     29  Carry condition code flag
      Z* {.bitsize: 1.}: uint32  # !< bit:     30  Zero condition code flag
      N* {.bitsize: 1.}: uint32  # !< bit:     31  Negative condition code flag

  type
    xPSR_Type* {.bycopy.} = object
      b*: INNER_C_STRUCT_core_cm4_241 # !< Structure used for bit  access
      w*: uint32               # !< Type      used for word access

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
    xPSR_Q_Pos* = 27
    xPSR_Q_Msk* = (1 shl xPSR_Q_Pos) # !< xPSR: Q Mask
    xPSR_IT_Pos* = 25
    xPSR_IT_Msk* = (3 shl xPSR_IT_Pos) # !< xPSR: IT Mask
    xPSR_T_Pos* = 24
    xPSR_T_Msk* = (1 shl xPSR_T_Pos) # !< xPSR: T Mask
    xPSR_GE_Pos* = 16
    xPSR_GE_Msk* = (0x0000000F shl xPSR_GE_Pos) # !< xPSR: GE Mask
    xPSR_ISR_Pos* = 0
    xPSR_ISR_Msk* = (0x000001FF) # !< xPSR: ISR Mask
  # *
  #   \brief  Union type to access the Control Registers (CONTROL).
  #
  type
    INNER_C_STRUCT_core_cm4_292* {.bycopy.} = object
      nPRIV* {.bitsize: 1.}: uint32 # !< bit:      0  Execution privilege in Thread mode
      SPSEL* {.bitsize: 1.}: uint32 # !< bit:      1  Stack to be used
      FPCA* {.bitsize: 1.}: uint32 # !< bit:      2  FP extension active flag
      reserved0* {.bitsize: 29.}: uint32 # !< bit:  3..31  Reserved

  type
    CONTROL_Type* {.bycopy.} = object
      b*: INNER_C_STRUCT_core_cm4_292 # !< Structure used for bit  access
      w*: uint32               # !< Type      used for word access

  #  CONTROL Register Definitions
  const
    CONTROL_FPCA_Pos* = 2
    CONTROL_FPCA_Msk* = (1 shl CONTROL_FPCA_Pos) # !< CONTROL: FPCA Mask
    CONTROL_SPSEL_Pos* = 1
    CONTROL_SPSEL_Msk* = (1 shl CONTROL_SPSEL_Pos) # !< CONTROL: SPSEL Mask
    CONTROL_nPRIV_Pos* = 0
    CONTROL_nPRIV_Msk* = (1)    # !< CONTROL: nPRIV Mask
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
      ISER*: array[8, uint32]   # !< Offset: 0x000 (R/W)  Interrupt Set Enable Register
      RESERVED0*: array[24, uint32]
      ICER*: array[8, uint32]   # !< Offset: 0x080 (R/W)  Interrupt Clear Enable Register
      RSERVED1*: array[24, uint32]
      ISPR*: array[8, uint32]   # !< Offset: 0x100 (R/W)  Interrupt Set Pending Register
      RESERVED2*: array[24, uint32]
      ICPR*: array[8, uint32]   # !< Offset: 0x180 (R/W)  Interrupt Clear Pending Register
      RESERVED3*: array[24, uint32]
      IABR*: array[8, uint32]   # !< Offset: 0x200 (R/W)  Interrupt Active bit Register
      RESERVED4*: array[56, uint32]
      IP*: array[240, uint8]    # !< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide)
      RESERVED5*: array[644, uint32]
      STIR*: uint32            # !< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register

  #  Software Triggered Interrupt Register Definitions
  const
    NVIC_STIR_INTID_Pos* = 0
    NVIC_STIR_INTID_Msk* = (0x000001FF) # !< STIR: INTLINESNUM Mask
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
      CPUID*: uint32           # !< Offset: 0x000 (R/ )  CPUID Base Register
      ICSR*: uint32            # !< Offset: 0x004 (R/W)  Interrupt Control and State Register
      VTOR*: uint32            # !< Offset: 0x008 (R/W)  Vector Table Offset Register
      AIRCR*: uint32           # !< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register
      SCR*: uint32             # !< Offset: 0x010 (R/W)  System Control Register
      CCR*: uint32             # !< Offset: 0x014 (R/W)  Configuration Control Register
      SHP*: array[12, uint8]    # !< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15)
      SHCSR*: uint32           # !< Offset: 0x024 (R/W)  System Handler Control and State Register
      CFSR*: uint32            # !< Offset: 0x028 (R/W)  Configurable Fault Status Register
      HFSR*: uint32            # !< Offset: 0x02C (R/W)  HardFault Status Register
      DFSR*: uint32            # !< Offset: 0x030 (R/W)  Debug Fault Status Register
      MMFAR*: uint32           # !< Offset: 0x034 (R/W)  MemManage Fault Address Register
      BFAR*: uint32            # !< Offset: 0x038 (R/W)  BusFault Address Register
      AFSR*: uint32            # !< Offset: 0x03C (R/W)  Auxiliary Fault Status Register
      PFR*: array[2, uint32]    # !< Offset: 0x040 (R/ )  Processor Feature Register
      DFR*: uint32             # !< Offset: 0x048 (R/ )  Debug Feature Register
      ADR*: uint32             # !< Offset: 0x04C (R/ )  Auxiliary Feature Register
      MMFR*: array[4, uint32]   # !< Offset: 0x050 (R/ )  Memory Model Feature Register
      ISAR*: array[5, uint32]   # !< Offset: 0x060 (R/ )  Instruction Set Attributes Register
      RESERVED0*: array[5, uint32]
      CPACR*: uint32           # !< Offset: 0x088 (R/W)  Coprocessor Access Control Register

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
    SCB_ICSR_RETTOBASE_Pos* = 11
    SCB_ICSR_RETTOBASE_Msk* = (1 shl SCB_ICSR_RETTOBASE_Pos) # !< SCB ICSR: RETTOBASE Mask
    SCB_ICSR_VECTACTIVE_Pos* = 0
    SCB_ICSR_VECTACTIVE_Msk* = (0x000001FF) # !< SCB ICSR: VECTACTIVE Mask
  #  SCB Vector Table Offset Register Definitions
  const
    SCB_VTOR_TBLOFF_Pos* = 7
    SCB_VTOR_TBLOFF_Msk* = (0x01FFFFFF shl SCB_VTOR_TBLOFF_Pos) # !< SCB VTOR: TBLOFF Mask
  #  SCB Application Interrupt and Reset Control Register Definitions
  const
    SCB_AIRCR_VECTKEY_Pos* = 16
    SCB_AIRCR_VECTKEY_Msk* = (0x0000FFFF shl SCB_AIRCR_VECTKEY_Pos) # !< SCB AIRCR: VECTKEY Mask
    SCB_AIRCR_VECTKEYSTAT_Pos* = 16
    SCB_AIRCR_VECTKEYSTAT_Msk* = (0x0000FFFF shl SCB_AIRCR_VECTKEYSTAT_Pos) # !< SCB AIRCR: VECTKEYSTAT Mask
    SCB_AIRCR_ENDIANESS_Pos* = 15
    SCB_AIRCR_ENDIANESS_Msk* = (1 shl SCB_AIRCR_ENDIANESS_Pos) # !< SCB AIRCR: ENDIANESS Mask
    SCB_AIRCR_PRIGROUP_Pos* = 8
    SCB_AIRCR_PRIGROUP_Msk* = (7 shl SCB_AIRCR_PRIGROUP_Pos) # !< SCB AIRCR: PRIGROUP Mask
    SCB_AIRCR_SYSRESETREQ_Pos* = 2
    SCB_AIRCR_SYSRESETREQ_Msk* = (1 shl SCB_AIRCR_SYSRESETREQ_Pos) # !< SCB AIRCR: SYSRESETREQ Mask
    SCB_AIRCR_VECTCLRACTIVE_Pos* = 1
    SCB_AIRCR_VECTCLRACTIVE_Msk* = (1 shl SCB_AIRCR_VECTCLRACTIVE_Pos) # !< SCB AIRCR: VECTCLRACTIVE Mask
    SCB_AIRCR_VECTRESET_Pos* = 0
    SCB_AIRCR_VECTRESET_Msk* = (1) # !< SCB AIRCR: VECTRESET Mask
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
    SCB_CCR_BFHFNMIGN_Pos* = 8
    SCB_CCR_BFHFNMIGN_Msk* = (1 shl SCB_CCR_BFHFNMIGN_Pos) # !< SCB CCR: BFHFNMIGN Mask
    SCB_CCR_DIV_0_TRP_Pos* = 4
    SCB_CCR_DIV_0_TRP_Msk* = (1 shl SCB_CCR_DIV_0_TRP_Pos) # !< SCB CCR: DIV_0_TRP Mask
    SCB_CCR_UNALIGN_TRP_Pos* = 3
    SCB_CCR_UNALIGN_TRP_Msk* = (1 shl SCB_CCR_UNALIGN_TRP_Pos) # !< SCB CCR: UNALIGN_TRP Mask
    SCB_CCR_USERSETMPEND_Pos* = 1
    SCB_CCR_USERSETMPEND_Msk* = (1 shl SCB_CCR_USERSETMPEND_Pos) # !< SCB CCR: USERSETMPEND Mask
    SCB_CCR_NONBASETHRDENA_Pos* = 0
    SCB_CCR_NONBASETHRDENA_Msk* = (1) # !< SCB CCR: NONBASETHRDENA Mask
  #  SCB System Handler Control and State Register Definitions
  const
    SCB_SHCSR_USGFAULTENA_Pos* = 18
    SCB_SHCSR_USGFAULTENA_Msk* = (1 shl SCB_SHCSR_USGFAULTENA_Pos) # !< SCB SHCSR: USGFAULTENA Mask
    SCB_SHCSR_BUSFAULTENA_Pos* = 17
    SCB_SHCSR_BUSFAULTENA_Msk* = (1 shl SCB_SHCSR_BUSFAULTENA_Pos) # !< SCB SHCSR: BUSFAULTENA Mask
    SCB_SHCSR_MEMFAULTENA_Pos* = 16
    SCB_SHCSR_MEMFAULTENA_Msk* = (1 shl SCB_SHCSR_MEMFAULTENA_Pos) # !< SCB SHCSR: MEMFAULTENA Mask
    SCB_SHCSR_SVCALLPENDED_Pos* = 15
    SCB_SHCSR_SVCALLPENDED_Msk* = (1 shl SCB_SHCSR_SVCALLPENDED_Pos) # !< SCB SHCSR: SVCALLPENDED Mask
    SCB_SHCSR_BUSFAULTPENDED_Pos* = 14
    SCB_SHCSR_BUSFAULTPENDED_Msk* = (1 shl SCB_SHCSR_BUSFAULTPENDED_Pos) # !< SCB SHCSR: BUSFAULTPENDED Mask
    SCB_SHCSR_MEMFAULTPENDED_Pos* = 13
    SCB_SHCSR_MEMFAULTPENDED_Msk* = (1 shl SCB_SHCSR_MEMFAULTPENDED_Pos) # !< SCB SHCSR: MEMFAULTPENDED Mask
    SCB_SHCSR_USGFAULTPENDED_Pos* = 12
    SCB_SHCSR_USGFAULTPENDED_Msk* = (1 shl SCB_SHCSR_USGFAULTPENDED_Pos) # !< SCB SHCSR: USGFAULTPENDED Mask
    SCB_SHCSR_SYSTICKACT_Pos* = 11
    SCB_SHCSR_SYSTICKACT_Msk* = (1 shl SCB_SHCSR_SYSTICKACT_Pos) # !< SCB SHCSR: SYSTICKACT Mask
    SCB_SHCSR_PENDSVACT_Pos* = 10
    SCB_SHCSR_PENDSVACT_Msk* = (1 shl SCB_SHCSR_PENDSVACT_Pos) # !< SCB SHCSR: PENDSVACT Mask
    SCB_SHCSR_MONITORACT_Pos* = 8
    SCB_SHCSR_MONITORACT_Msk* = (1 shl SCB_SHCSR_MONITORACT_Pos) # !< SCB SHCSR: MONITORACT Mask
    SCB_SHCSR_SVCALLACT_Pos* = 7
    SCB_SHCSR_SVCALLACT_Msk* = (1 shl SCB_SHCSR_SVCALLACT_Pos) # !< SCB SHCSR: SVCALLACT Mask
    SCB_SHCSR_USGFAULTACT_Pos* = 3
    SCB_SHCSR_USGFAULTACT_Msk* = (1 shl SCB_SHCSR_USGFAULTACT_Pos) # !< SCB SHCSR: USGFAULTACT Mask
    SCB_SHCSR_BUSFAULTACT_Pos* = 1
    SCB_SHCSR_BUSFAULTACT_Msk* = (1 shl SCB_SHCSR_BUSFAULTACT_Pos) # !< SCB SHCSR: BUSFAULTACT Mask
    SCB_SHCSR_MEMFAULTACT_Pos* = 0
    SCB_SHCSR_MEMFAULTACT_Msk* = (1) # !< SCB SHCSR: MEMFAULTACT Mask
  #  SCB Configurable Fault Status Register Definitions
  const
    SCB_CFSR_USGFAULTSR_Pos* = 16
    SCB_CFSR_USGFAULTSR_Msk* = (0x0000FFFF shl SCB_CFSR_USGFAULTSR_Pos) # !< SCB CFSR: Usage Fault Status Register Mask
    SCB_CFSR_BUSFAULTSR_Pos* = 8
    SCB_CFSR_BUSFAULTSR_Msk* = (0x000000FF shl SCB_CFSR_BUSFAULTSR_Pos) # !< SCB CFSR: Bus Fault Status Register Mask
    SCB_CFSR_MEMFAULTSR_Pos* = 0
    SCB_CFSR_MEMFAULTSR_Msk* = (0x000000FF) # !< SCB CFSR: Memory Manage Fault Status Register Mask
  #  SCB Hard Fault Status Register Definitions
  const
    SCB_HFSR_DEBUGEVT_Pos* = 31
    SCB_HFSR_DEBUGEVT_Msk* = (1 shl SCB_HFSR_DEBUGEVT_Pos) # !< SCB HFSR: DEBUGEVT Mask
    SCB_HFSR_FORCED_Pos* = 30
    SCB_HFSR_FORCED_Msk* = (1 shl SCB_HFSR_FORCED_Pos) # !< SCB HFSR: FORCED Mask
    SCB_HFSR_VECTTBL_Pos* = 1
    SCB_HFSR_VECTTBL_Msk* = (1 shl SCB_HFSR_VECTTBL_Pos) # !< SCB HFSR: VECTTBL Mask
  #  SCB Debug Fault Status Register Definitions
  const
    SCB_DFSR_EXTERNAL_Pos* = 4
    SCB_DFSR_EXTERNAL_Msk* = (1 shl SCB_DFSR_EXTERNAL_Pos) # !< SCB DFSR: EXTERNAL Mask
    SCB_DFSR_VCATCH_Pos* = 3
    SCB_DFSR_VCATCH_Msk* = (1 shl SCB_DFSR_VCATCH_Pos) # !< SCB DFSR: VCATCH Mask
    SCB_DFSR_DWTTRAP_Pos* = 2
    SCB_DFSR_DWTTRAP_Msk* = (1 shl SCB_DFSR_DWTTRAP_Pos) # !< SCB DFSR: DWTTRAP Mask
    SCB_DFSR_BKPT_Pos* = 1
    SCB_DFSR_BKPT_Msk* = (1 shl SCB_DFSR_BKPT_Pos) # !< SCB DFSR: BKPT Mask
    SCB_DFSR_HALTED_Pos* = 0
    SCB_DFSR_HALTED_Msk* = (1)  # !< SCB DFSR: HALTED Mask
  # @} end of group CMSIS_SCB
  # *
  #   \ingroup  CMSIS_core_register
  #   \defgroup CMSIS_SCnSCB System Controls not in SCB (SCnSCB)
  #   \brief    Type definitions for the System Control and ID Register not in the SCB
  #   @{
  #
  # *
  #   \brief  Structure type to access the System Control and ID Register not in the SCB.
  #
  type
    SCnSCB_Type* {.bycopy.} = object
      RESERVED0*: array[1, uint32]
      ICTR*: uint32            # !< Offset: 0x004 (R/ )  Interrupt Controller Type Register
      ACTLR*: uint32           # !< Offset: 0x008 (R/W)  Auxiliary Control Register

  #  Interrupt Controller Type Register Definitions
  const
    SCnSCB_ICTR_INTLINESNUM_Pos* = 0
    SCnSCB_ICTR_INTLINESNUM_Msk* = (0x0000000F) # !< ICTR: INTLINESNUM Mask
  #  Auxiliary Control Register Definitions
  const
    SCnSCB_ACTLR_DISOOFP_Pos* = 9
    SCnSCB_ACTLR_DISOOFP_Msk* = (1 shl SCnSCB_ACTLR_DISOOFP_Pos) # !< ACTLR: DISOOFP Mask
    SCnSCB_ACTLR_DISFPCA_Pos* = 8
    SCnSCB_ACTLR_DISFPCA_Msk* = (1 shl SCnSCB_ACTLR_DISFPCA_Pos) # !< ACTLR: DISFPCA Mask
    SCnSCB_ACTLR_DISFOLD_Pos* = 2
    SCnSCB_ACTLR_DISFOLD_Msk* = (1 shl SCnSCB_ACTLR_DISFOLD_Pos) # !< ACTLR: DISFOLD Mask
    SCnSCB_ACTLR_DISDEFWBUF_Pos* = 1
    SCnSCB_ACTLR_DISDEFWBUF_Msk* = (1 shl SCnSCB_ACTLR_DISDEFWBUF_Pos) # !< ACTLR: DISDEFWBUF Mask
    SCnSCB_ACTLR_DISMCYCINT_Pos* = 0
    SCnSCB_ACTLR_DISMCYCINT_Msk* = (1) # !< ACTLR: DISMCYCINT Mask
  # @} end of group CMSIS_SCnotSCB
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
      CTRL*: uint32            # !< Offset: 0x000 (R/W)  SysTick Control and Status Register
      LOAD*: uint32            # !< Offset: 0x004 (R/W)  SysTick Reload Value Register
      VAL*: uint32             # !< Offset: 0x008 (R/W)  SysTick Current Value Register
      CALIB*: uint32           # !< Offset: 0x00C (R/ )  SysTick Calibration Register

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
  #   \defgroup CMSIS_ITM     Instrumentation Trace Macrocell (ITM)
  #   \brief    Type definitions for the Instrumentation Trace Macrocell (ITM)
  #   @{
  #
  # *
  #   \brief  Structure type to access the Instrumentation Trace Macrocell Register (ITM).
  #
  type
    INNER_C_UNION_core_cm4_672* {.bycopy.} = object
      u8*: uint8               # !< Offset: 0x000 ( /W)  ITM Stimulus Port 8-bit
      u16*: uint16             # !< Offset: 0x000 ( /W)  ITM Stimulus Port 16-bit
      u32*: uint32             # !< Offset: 0x000 ( /W)  ITM Stimulus Port 32-bit

  type
    ITM_Type* {.bycopy.} = object
      PORT*: array[32, INNER_C_UNION_core_cm4_672] # !< Offset: 0x000 ( /W)  ITM Stimulus Port Registers
      RESERVED0*: array[864, uint32]
      TER*: uint32             # !< Offset: 0xE00 (R/W)  ITM Trace Enable Register
      RESERVED1*: array[15, uint32]
      TPR*: uint32             # !< Offset: 0xE40 (R/W)  ITM Trace Privilege Register
      RESERVED2*: array[15, uint32]
      TCR*: uint32             # !< Offset: 0xE80 (R/W)  ITM Trace Control Register
      RESERVED3*: array[29, uint32]
      IWR*: uint32             # !< Offset: 0xEF8 ( /W)  ITM Integration Write Register
      IRR*: uint32             # !< Offset: 0xEFC (R/ )  ITM Integration Read Register
      IMCR*: uint32            # !< Offset: 0xF00 (R/W)  ITM Integration Mode Control Register
      RESERVED4*: array[43, uint32]
      LAR*: uint32             # !< Offset: 0xFB0 ( /W)  ITM Lock Access Register
      LSR*: uint32             # !< Offset: 0xFB4 (R/ )  ITM Lock Status Register
      RESERVED5*: array[6, uint32]
      PID4*: uint32            # !< Offset: 0xFD0 (R/ )  ITM Peripheral Identification Register #4
      PID5*: uint32            # !< Offset: 0xFD4 (R/ )  ITM Peripheral Identification Register #5
      PID6*: uint32            # !< Offset: 0xFD8 (R/ )  ITM Peripheral Identification Register #6
      PID7*: uint32            # !< Offset: 0xFDC (R/ )  ITM Peripheral Identification Register #7
      PID0*: uint32            # !< Offset: 0xFE0 (R/ )  ITM Peripheral Identification Register #0
      PID1*: uint32            # !< Offset: 0xFE4 (R/ )  ITM Peripheral Identification Register #1
      PID2*: uint32            # !< Offset: 0xFE8 (R/ )  ITM Peripheral Identification Register #2
      PID3*: uint32            # !< Offset: 0xFEC (R/ )  ITM Peripheral Identification Register #3
      CID0*: uint32            # !< Offset: 0xFF0 (R/ )  ITM Component  Identification Register #0
      CID1*: uint32            # !< Offset: 0xFF4 (R/ )  ITM Component  Identification Register #1
      CID2*: uint32            # !< Offset: 0xFF8 (R/ )  ITM Component  Identification Register #2
      CID3*: uint32            # !< Offset: 0xFFC (R/ )  ITM Component  Identification Register #3

  #  ITM Trace Privilege Register Definitions
  const
    ITM_TPR_PRIVMASK_Pos* = 0
    ITM_TPR_PRIVMASK_Msk* = (0x0000000F) # !< ITM TPR: PRIVMASK Mask
  #  ITM Trace Control Register Definitions
  const
    ITM_TCR_BUSY_Pos* = 23
    ITM_TCR_BUSY_Msk* = (1 shl ITM_TCR_BUSY_Pos) # !< ITM TCR: BUSY Mask
    ITM_TCR_TraceBusID_Pos* = 16
    ITM_TCR_TraceBusID_Msk* = (0x0000007F shl ITM_TCR_TraceBusID_Pos) # !< ITM TCR: ATBID Mask
    ITM_TCR_GTSFREQ_Pos* = 10
    ITM_TCR_GTSFREQ_Msk* = (3 shl ITM_TCR_GTSFREQ_Pos) # !< ITM TCR: Global timestamp frequency Mask
    ITM_TCR_TSPrescale_Pos* = 8
    ITM_TCR_TSPrescale_Msk* = (3 shl ITM_TCR_TSPrescale_Pos) # !< ITM TCR: TSPrescale Mask
    ITM_TCR_SWOENA_Pos* = 4
    ITM_TCR_SWOENA_Msk* = (1 shl ITM_TCR_SWOENA_Pos) # !< ITM TCR: SWOENA Mask
    ITM_TCR_DWTENA_Pos* = 3
    ITM_TCR_DWTENA_Msk* = (1 shl ITM_TCR_DWTENA_Pos) # !< ITM TCR: DWTENA Mask
    ITM_TCR_SYNCENA_Pos* = 2
    ITM_TCR_SYNCENA_Msk* = (1 shl ITM_TCR_SYNCENA_Pos) # !< ITM TCR: SYNCENA Mask
    ITM_TCR_TSENA_Pos* = 1
    ITM_TCR_TSENA_Msk* = (1 shl ITM_TCR_TSENA_Pos) # !< ITM TCR: TSENA Mask
    ITM_TCR_ITMENA_Pos* = 0
    ITM_TCR_ITMENA_Msk* = (1)   # !< ITM TCR: ITM Enable bit Mask
  #  ITM Integration Write Register Definitions
  const
    ITM_IWR_ATVALIDM_Pos* = 0
    ITM_IWR_ATVALIDM_Msk* = (1) # !< ITM IWR: ATVALIDM Mask
  #  ITM Integration Read Register Definitions
  const
    ITM_IRR_ATREADYM_Pos* = 0
    ITM_IRR_ATREADYM_Msk* = (1) # !< ITM IRR: ATREADYM Mask
  #  ITM Integration Mode Control Register Definitions
  const
    ITM_IMCR_INTEGRATION_Pos* = 0
    ITM_IMCR_INTEGRATION_Msk* = (1) # !< ITM IMCR: INTEGRATION Mask
  #  ITM Lock Status Register Definitions
  const
    ITM_LSR_ByteAcc_Pos* = 2
    ITM_LSR_ByteAcc_Msk* = (1 shl ITM_LSR_ByteAcc_Pos) # !< ITM LSR: ByteAcc Mask
    ITM_LSR_Access_Pos* = 1
    ITM_LSR_Access_Msk* = (1 shl ITM_LSR_Access_Pos) # !< ITM LSR: Access Mask
    ITM_LSR_Present_Pos* = 0
    ITM_LSR_Present_Msk* = (1)  # !< ITM LSR: Present Mask
  # @}
  #  end of group CMSIS_ITM
  # *
  #   \ingroup  CMSIS_core_register
  #   \defgroup CMSIS_DWT     Data Watchpoint and Trace (DWT)
  #   \brief    Type definitions for the Data Watchpoint and Trace (DWT)
  #   @{
  #
  # *
  #   \brief  Structure type to access the Data Watchpoint and Trace Register (DWT).
  #
  type
    DWT_Type* {.bycopy.} = object
      CTRL*: uint32            # !< Offset: 0x000 (R/W)  Control Register
      CYCCNT*: uint32          # !< Offset: 0x004 (R/W)  Cycle Count Register
      CPICNT*: uint32          # !< Offset: 0x008 (R/W)  CPI Count Register
      EXCCNT*: uint32          # !< Offset: 0x00C (R/W)  Exception Overhead Count Register
      SLEEPCNT*: uint32        # !< Offset: 0x010 (R/W)  Sleep Count Register
      LSUCNT*: uint32          # !< Offset: 0x014 (R/W)  LSU Count Register
      FOLDCNT*: uint32         # !< Offset: 0x018 (R/W)  Folded-instruction Count Register
      PCSR*: uint32            # !< Offset: 0x01C (R/ )  Program Counter Sample Register
      COMP0*: uint32           # !< Offset: 0x020 (R/W)  Comparator Register 0
      MASK0*: uint32           # !< Offset: 0x024 (R/W)  Mask Register 0
      FUNCTION0*: uint32       # !< Offset: 0x028 (R/W)  Function Register 0
      RESERVED0*: array[1, uint32]
      COMP1*: uint32           # !< Offset: 0x030 (R/W)  Comparator Register 1
      MASK1*: uint32           # !< Offset: 0x034 (R/W)  Mask Register 1
      FUNCTION1*: uint32       # !< Offset: 0x038 (R/W)  Function Register 1
      RESERVED1*: array[1, uint32]
      COMP2*: uint32           # !< Offset: 0x040 (R/W)  Comparator Register 2
      MASK2*: uint32           # !< Offset: 0x044 (R/W)  Mask Register 2
      FUNCTION2*: uint32       # !< Offset: 0x048 (R/W)  Function Register 2
      RESERVED2*: array[1, uint32]
      COMP3*: uint32           # !< Offset: 0x050 (R/W)  Comparator Register 3
      MASK3*: uint32           # !< Offset: 0x054 (R/W)  Mask Register 3
      FUNCTION3*: uint32       # !< Offset: 0x058 (R/W)  Function Register 3

  #  DWT Control Register Definitions
  const
    DWT_CTRL_NUMCOMP_Pos* = 28
    DWT_CTRL_NUMCOMP_Msk* = (0x0000000F shl DWT_CTRL_NUMCOMP_Pos) # !< DWT CTRL: NUMCOMP Mask
    DWT_CTRL_NOTRCPKT_Pos* = 27
    DWT_CTRL_NOTRCPKT_Msk* = (0x00000001 shl DWT_CTRL_NOTRCPKT_Pos) # !< DWT CTRL: NOTRCPKT Mask
    DWT_CTRL_NOEXTTRIG_Pos* = 26
    DWT_CTRL_NOEXTTRIG_Msk* = (0x00000001 shl DWT_CTRL_NOEXTTRIG_Pos) # !< DWT CTRL: NOEXTTRIG Mask
    DWT_CTRL_NOCYCCNT_Pos* = 25
    DWT_CTRL_NOCYCCNT_Msk* = (0x00000001 shl DWT_CTRL_NOCYCCNT_Pos) # !< DWT CTRL: NOCYCCNT Mask
    DWT_CTRL_NOPRFCNT_Pos* = 24
    DWT_CTRL_NOPRFCNT_Msk* = (0x00000001 shl DWT_CTRL_NOPRFCNT_Pos) # !< DWT CTRL: NOPRFCNT Mask
    DWT_CTRL_CYCEVTENA_Pos* = 22
    DWT_CTRL_CYCEVTENA_Msk* = (0x00000001 shl DWT_CTRL_CYCEVTENA_Pos) # !< DWT CTRL: CYCEVTENA Mask
    DWT_CTRL_FOLDEVTENA_Pos* = 21
    DWT_CTRL_FOLDEVTENA_Msk* = (0x00000001 shl DWT_CTRL_FOLDEVTENA_Pos) # !< DWT CTRL: FOLDEVTENA Mask
    DWT_CTRL_LSUEVTENA_Pos* = 20
    DWT_CTRL_LSUEVTENA_Msk* = (0x00000001 shl DWT_CTRL_LSUEVTENA_Pos) # !< DWT CTRL: LSUEVTENA Mask
    DWT_CTRL_SLEEPEVTENA_Pos* = 19
    DWT_CTRL_SLEEPEVTENA_Msk* = (0x00000001 shl DWT_CTRL_SLEEPEVTENA_Pos) # !< DWT CTRL: SLEEPEVTENA Mask
    DWT_CTRL_EXCEVTENA_Pos* = 18
    DWT_CTRL_EXCEVTENA_Msk* = (0x00000001 shl DWT_CTRL_EXCEVTENA_Pos) # !< DWT CTRL: EXCEVTENA Mask
    DWT_CTRL_CPIEVTENA_Pos* = 17
    DWT_CTRL_CPIEVTENA_Msk* = (0x00000001 shl DWT_CTRL_CPIEVTENA_Pos) # !< DWT CTRL: CPIEVTENA Mask
    DWT_CTRL_EXCTRCENA_Pos* = 16
    DWT_CTRL_EXCTRCENA_Msk* = (0x00000001 shl DWT_CTRL_EXCTRCENA_Pos) # !< DWT CTRL: EXCTRCENA Mask
    DWT_CTRL_PCSAMPLENA_Pos* = 12
    DWT_CTRL_PCSAMPLENA_Msk* = (0x00000001 shl DWT_CTRL_PCSAMPLENA_Pos) # !< DWT CTRL: PCSAMPLENA Mask
    DWT_CTRL_SYNCTAP_Pos* = 10
    DWT_CTRL_SYNCTAP_Msk* = (0x00000003 shl DWT_CTRL_SYNCTAP_Pos) # !< DWT CTRL: SYNCTAP Mask
    DWT_CTRL_CYCTAP_Pos* = 9
    DWT_CTRL_CYCTAP_Msk* = (0x00000001 shl DWT_CTRL_CYCTAP_Pos) # !< DWT CTRL: CYCTAP Mask
    DWT_CTRL_POSTINIT_Pos* = 5
    DWT_CTRL_POSTINIT_Msk* = (0x0000000F shl DWT_CTRL_POSTINIT_Pos) # !< DWT CTRL: POSTINIT Mask
    DWT_CTRL_POSTPRESET_Pos* = 1
    DWT_CTRL_POSTPRESET_Msk* = (0x0000000F shl DWT_CTRL_POSTPRESET_Pos) # !< DWT CTRL: POSTPRESET Mask
    DWT_CTRL_CYCCNTENA_Pos* = 0
    DWT_CTRL_CYCCNTENA_Msk* = (0x00000001) # !< DWT CTRL: CYCCNTENA Mask
  #  DWT CPI Count Register Definitions
  const
    DWT_CPICNT_CPICNT_Pos* = 0
    DWT_CPICNT_CPICNT_Msk* = (0x000000FF) # !< DWT CPICNT: CPICNT Mask
  #  DWT Exception Overhead Count Register Definitions
  const
    DWT_EXCCNT_EXCCNT_Pos* = 0
    DWT_EXCCNT_EXCCNT_Msk* = (0x000000FF) # !< DWT EXCCNT: EXCCNT Mask
  #  DWT Sleep Count Register Definitions
  const
    DWT_SLEEPCNT_SLEEPCNT_Pos* = 0
    DWT_SLEEPCNT_SLEEPCNT_Msk* = (0x000000FF) # !< DWT SLEEPCNT: SLEEPCNT Mask
  #  DWT LSU Count Register Definitions
  const
    DWT_LSUCNT_LSUCNT_Pos* = 0
    DWT_LSUCNT_LSUCNT_Msk* = (0x000000FF) # !< DWT LSUCNT: LSUCNT Mask
  #  DWT Folded-instruction Count Register Definitions
  const
    DWT_FOLDCNT_FOLDCNT_Pos* = 0
    DWT_FOLDCNT_FOLDCNT_Msk* = (0x000000FF) # !< DWT FOLDCNT: FOLDCNT Mask
  #  DWT Comparator Mask Register Definitions
  const
    DWT_MASK_MASK_Pos* = 0
    DWT_MASK_MASK_Msk* = (0x0000001F) # !< DWT MASK: MASK Mask
  #  DWT Comparator Function Register Definitions
  const
    DWT_FUNCTION_MATCHED_Pos* = 24
    DWT_FUNCTION_MATCHED_Msk* = (0x00000001 shl DWT_FUNCTION_MATCHED_Pos) # !< DWT FUNCTION: MATCHED Mask
    DWT_FUNCTION_DATAVADDR1_Pos* = 16
    DWT_FUNCTION_DATAVADDR1_Msk* = (0x0000000F shl DWT_FUNCTION_DATAVADDR1_Pos) # !< DWT FUNCTION: DATAVADDR1 Mask
    DWT_FUNCTION_DATAVADDR0_Pos* = 12
    DWT_FUNCTION_DATAVADDR0_Msk* = (0x0000000F shl DWT_FUNCTION_DATAVADDR0_Pos) # !< DWT FUNCTION: DATAVADDR0 Mask
    DWT_FUNCTION_DATAVSIZE_Pos* = 10
    DWT_FUNCTION_DATAVSIZE_Msk* = (0x00000003 shl DWT_FUNCTION_DATAVSIZE_Pos) # !< DWT FUNCTION: DATAVSIZE Mask
    DWT_FUNCTION_LNK1ENA_Pos* = 9
    DWT_FUNCTION_LNK1ENA_Msk* = (0x00000001 shl DWT_FUNCTION_LNK1ENA_Pos) # !< DWT FUNCTION: LNK1ENA Mask
    DWT_FUNCTION_DATAVMATCH_Pos* = 8
    DWT_FUNCTION_DATAVMATCH_Msk* = (0x00000001 shl DWT_FUNCTION_DATAVMATCH_Pos) # !< DWT FUNCTION: DATAVMATCH Mask
    DWT_FUNCTION_CYCMATCH_Pos* = 7
    DWT_FUNCTION_CYCMATCH_Msk* = (0x00000001 shl DWT_FUNCTION_CYCMATCH_Pos) # !< DWT FUNCTION: CYCMATCH Mask
    DWT_FUNCTION_EMITRANGE_Pos* = 5
    DWT_FUNCTION_EMITRANGE_Msk* = (0x00000001 shl DWT_FUNCTION_EMITRANGE_Pos) # !< DWT FUNCTION: EMITRANGE Mask
    DWT_FUNCTION_FUNCTION_Pos* = 0
    DWT_FUNCTION_FUNCTION_Msk* = (0x0000000F) # !< DWT FUNCTION: FUNCTION Mask
  # @}
  #  end of group CMSIS_DWT
  # *
  #   \ingroup  CMSIS_core_register
  #   \defgroup CMSIS_TPI     Trace Port Interface (TPI)
  #   \brief    Type definitions for the Trace Port Interface (TPI)
  #   @{
  #
  # *
  #   \brief  Structure type to access the Trace Port Interface Register (TPI).
  #
  type
    TPI_Type* {.bycopy.} = object
      SSPSR*: uint32           # !< Offset: 0x000 (R/ )  Supported Parallel Port Size Register
      CSPSR*: uint32           # !< Offset: 0x004 (R/W)  Current Parallel Port Size Register
      RESERVED0*: array[2, uint32]
      ACPR*: uint32            # !< Offset: 0x010 (R/W)  Asynchronous Clock Prescaler Register
      RESERVED1*: array[55, uint32]
      SPPR*: uint32            # !< Offset: 0x0F0 (R/W)  Selected Pin Protocol Register
      RESERVED2*: array[131, uint32]
      FFSR*: uint32            # !< Offset: 0x300 (R/ )  Formatter and Flush Status Register
      FFCR*: uint32            # !< Offset: 0x304 (R/W)  Formatter and Flush Control Register
      FSCR*: uint32            # !< Offset: 0x308 (R/ )  Formatter Synchronization Counter Register
      RESERVED3*: array[759, uint32]
      TRIGGER*: uint32         # !< Offset: 0xEE8 (R/ )  TRIGGER
      FIFO0*: uint32           # !< Offset: 0xEEC (R/ )  Integration ETM Data
      ITATBCTR2*: uint32       # !< Offset: 0xEF0 (R/ )  ITATBCTR2
      RESERVED4*: array[1, uint32]
      ITATBCTR0*: uint32       # !< Offset: 0xEF8 (R/ )  ITATBCTR0
      FIFO1*: uint32           # !< Offset: 0xEFC (R/ )  Integration ITM Data
      ITCTRL*: uint32          # !< Offset: 0xF00 (R/W)  Integration Mode Control
      RESERVED5*: array[39, uint32]
      CLAIMSET*: uint32        # !< Offset: 0xFA0 (R/W)  Claim tag set
      CLAIMCLR*: uint32        # !< Offset: 0xFA4 (R/W)  Claim tag clear
      RESERVED7*: array[8, uint32]
      DEVID*: uint32           # !< Offset: 0xFC8 (R/ )  TPIU_DEVID
      DEVTYPE*: uint32         # !< Offset: 0xFCC (R/ )  TPIU_DEVTYPE

  #  TPI Asynchronous Clock Prescaler Register Definitions
  const
    TPI_ACPR_PRESCALER_Pos* = 0
    TPI_ACPR_PRESCALER_Msk* = (0x00001FFF) # !< TPI ACPR: PRESCALER Mask
  #  TPI Selected Pin Protocol Register Definitions
  const
    TPI_SPPR_TXMODE_Pos* = 0
    TPI_SPPR_TXMODE_Msk* = (0x00000003) # !< TPI SPPR: TXMODE Mask
  #  TPI Formatter and Flush Status Register Definitions
  const
    TPI_FFSR_FtNonStop_Pos* = 3
    TPI_FFSR_FtNonStop_Msk* = (0x00000001 shl TPI_FFSR_FtNonStop_Pos) # !< TPI FFSR: FtNonStop Mask
    TPI_FFSR_TCPresent_Pos* = 2
    TPI_FFSR_TCPresent_Msk* = (0x00000001 shl TPI_FFSR_TCPresent_Pos) # !< TPI FFSR: TCPresent Mask
    TPI_FFSR_FtStopped_Pos* = 1
    TPI_FFSR_FtStopped_Msk* = (0x00000001 shl TPI_FFSR_FtStopped_Pos) # !< TPI FFSR: FtStopped Mask
    TPI_FFSR_FlInProg_Pos* = 0
    TPI_FFSR_FlInProg_Msk* = (0x00000001) # !< TPI FFSR: FlInProg Mask
  #  TPI Formatter and Flush Control Register Definitions
  const
    TPI_FFCR_TrigIn_Pos* = 8
    TPI_FFCR_TrigIn_Msk* = (0x00000001 shl TPI_FFCR_TrigIn_Pos) # !< TPI FFCR: TrigIn Mask
    TPI_FFCR_EnFCont_Pos* = 1
    TPI_FFCR_EnFCont_Msk* = (0x00000001 shl TPI_FFCR_EnFCont_Pos) # !< TPI FFCR: EnFCont Mask
  #  TPI TRIGGER Register Definitions
  const
    TPI_TRIGGER_TRIGGER_Pos* = 0
    TPI_TRIGGER_TRIGGER_Msk* = (0x00000001) # !< TPI TRIGGER: TRIGGER Mask
  #  TPI Integration ETM Data Register Definitions (FIFO0)
  const
    TPI_FIFO0_ITM_ATVALID_Pos* = 29
    TPI_FIFO0_ITM_ATVALID_Msk* = (0x00000003 shl TPI_FIFO0_ITM_ATVALID_Pos) # !< TPI FIFO0: ITM_ATVALID Mask
    TPI_FIFO0_ITM_bytecount_Pos* = 27
    TPI_FIFO0_ITM_bytecount_Msk* = (0x00000003 shl TPI_FIFO0_ITM_bytecount_Pos) # !< TPI FIFO0: ITM_bytecount Mask
    TPI_FIFO0_ETM_ATVALID_Pos* = 26
    TPI_FIFO0_ETM_ATVALID_Msk* = (0x00000003 shl TPI_FIFO0_ETM_ATVALID_Pos) # !< TPI FIFO0: ETM_ATVALID Mask
    TPI_FIFO0_ETM_bytecount_Pos* = 24
    TPI_FIFO0_ETM_bytecount_Msk* = (0x00000003 shl TPI_FIFO0_ETM_bytecount_Pos) # !< TPI FIFO0: ETM_bytecount Mask
    TPI_FIFO0_ETM2_Pos* = 16
    TPI_FIFO0_ETM2_Msk* = (0x000000FF shl TPI_FIFO0_ETM2_Pos) # !< TPI FIFO0: ETM2 Mask
    TPI_FIFO0_ETM1_Pos* = 8
    TPI_FIFO0_ETM1_Msk* = (0x000000FF shl TPI_FIFO0_ETM1_Pos) # !< TPI FIFO0: ETM1 Mask
    TPI_FIFO0_ETM0_Pos* = 0
    TPI_FIFO0_ETM0_Msk* = (0x000000FF) # !< TPI FIFO0: ETM0 Mask
  #  TPI ITATBCTR2 Register Definitions
  const
    TPI_ITATBCTR2_ATREADY_Pos* = 0
    TPI_ITATBCTR2_ATREADY_Msk* = (0x00000001) # !< TPI ITATBCTR2: ATREADY Mask
  #  TPI Integration ITM Data Register Definitions (FIFO1)
  const
    TPI_FIFO1_ITM_ATVALID_Pos* = 29
    TPI_FIFO1_ITM_ATVALID_Msk* = (0x00000003 shl TPI_FIFO1_ITM_ATVALID_Pos) # !< TPI FIFO1: ITM_ATVALID Mask
    TPI_FIFO1_ITM_bytecount_Pos* = 27
    TPI_FIFO1_ITM_bytecount_Msk* = (0x00000003 shl TPI_FIFO1_ITM_bytecount_Pos) # !< TPI FIFO1: ITM_bytecount Mask
    TPI_FIFO1_ETM_ATVALID_Pos* = 26
    TPI_FIFO1_ETM_ATVALID_Msk* = (0x00000003 shl TPI_FIFO1_ETM_ATVALID_Pos) # !< TPI FIFO1: ETM_ATVALID Mask
    TPI_FIFO1_ETM_bytecount_Pos* = 24
    TPI_FIFO1_ETM_bytecount_Msk* = (0x00000003 shl TPI_FIFO1_ETM_bytecount_Pos) # !< TPI FIFO1: ETM_bytecount Mask
    TPI_FIFO1_ITM2_Pos* = 16
    TPI_FIFO1_ITM2_Msk* = (0x000000FF shl TPI_FIFO1_ITM2_Pos) # !< TPI FIFO1: ITM2 Mask
    TPI_FIFO1_ITM1_Pos* = 8
    TPI_FIFO1_ITM1_Msk* = (0x000000FF shl TPI_FIFO1_ITM1_Pos) # !< TPI FIFO1: ITM1 Mask
    TPI_FIFO1_ITM0_Pos* = 0
    TPI_FIFO1_ITM0_Msk* = (0x000000FF) # !< TPI FIFO1: ITM0 Mask
  #  TPI ITATBCTR0 Register Definitions
  const
    TPI_ITATBCTR0_ATREADY_Pos* = 0
    TPI_ITATBCTR0_ATREADY_Msk* = (0x00000001) # !< TPI ITATBCTR0: ATREADY Mask
  #  TPI Integration Mode Control Register Definitions
  const
    TPI_ITCTRL_Mode_Pos* = 0
    TPI_ITCTRL_Mode_Msk* = (0x00000001) # !< TPI ITCTRL: Mode Mask
  #  TPI DEVID Register Definitions
  const
    TPI_DEVID_NRZVALID_Pos* = 11
    TPI_DEVID_NRZVALID_Msk* = (0x00000001 shl TPI_DEVID_NRZVALID_Pos) # !< TPI DEVID: NRZVALID Mask
    TPI_DEVID_MANCVALID_Pos* = 10
    TPI_DEVID_MANCVALID_Msk* = (0x00000001 shl TPI_DEVID_MANCVALID_Pos) # !< TPI DEVID: MANCVALID Mask
    TPI_DEVID_PTINVALID_Pos* = 9
    TPI_DEVID_PTINVALID_Msk* = (0x00000001 shl TPI_DEVID_PTINVALID_Pos) # !< TPI DEVID: PTINVALID Mask
    TPI_DEVID_MinBufSz_Pos* = 6
    TPI_DEVID_MinBufSz_Msk* = (0x00000007 shl TPI_DEVID_MinBufSz_Pos) # !< TPI DEVID: MinBufSz Mask
    TPI_DEVID_AsynClkIn_Pos* = 5
    TPI_DEVID_AsynClkIn_Msk* = (0x00000001 shl TPI_DEVID_AsynClkIn_Pos) # !< TPI DEVID: AsynClkIn Mask
    TPI_DEVID_NrTraceInput_Pos* = 0
    TPI_DEVID_NrTraceInput_Msk* = (0x0000001F) # !< TPI DEVID: NrTraceInput Mask
  #  TPI DEVTYPE Register Definitions
  const
    TPI_DEVTYPE_MajorType_Pos* = 4
    TPI_DEVTYPE_MajorType_Msk* = (0x0000000F shl TPI_DEVTYPE_MajorType_Pos) # !< TPI DEVTYPE: MajorType Mask
    TPI_DEVTYPE_SubType_Pos* = 0
    TPI_DEVTYPE_SubType_Msk* = (0x0000000F) # !< TPI DEVTYPE: SubType Mask
  # @}
  #  end of group CMSIS_TPI
  when (MPU_PRESENT == 1):
    # *
    #   \ingroup  CMSIS_core_register
    #   \defgroup CMSIS_MPU     Memory Protection Unit (MPU)
    #   \brief    Type definitions for the Memory Protection Unit (MPU)
    #   @{
    #
    # *
    #   \brief  Structure type to access the Memory Protection Unit (MPU).
    #
    type
      MPU_Type* {.bycopy.} = object
        TYPE*: uint32          # !< Offset: 0x000 (R/ )  MPU Type Register
        CTRL*: uint32          # !< Offset: 0x004 (R/W)  MPU Control Register
        RNR*: uint32           # !< Offset: 0x008 (R/W)  MPU Region RNRber Register
        RBAR*: uint32          # !< Offset: 0x00C (R/W)  MPU Region Base Address Register
        RASR*: uint32          # !< Offset: 0x010 (R/W)  MPU Region Attribute and Size Register
        RBAR_A1*: uint32       # !< Offset: 0x014 (R/W)  MPU Alias 1 Region Base Address Register
        RASR_A1*: uint32       # !< Offset: 0x018 (R/W)  MPU Alias 1 Region Attribute and Size Register
        RBAR_A2*: uint32       # !< Offset: 0x01C (R/W)  MPU Alias 2 Region Base Address Register
        RASR_A2*: uint32       # !< Offset: 0x020 (R/W)  MPU Alias 2 Region Attribute and Size Register
        RBAR_A3*: uint32       # !< Offset: 0x024 (R/W)  MPU Alias 3 Region Base Address Register
        RASR_A3*: uint32       # !< Offset: 0x028 (R/W)  MPU Alias 3 Region Attribute and Size Register

    #  MPU Type Register Definitions
    const
      MPU_TYPE_IREGION_Pos* = 16
      MPU_TYPE_IREGION_Msk* = (0x000000FF shl MPU_TYPE_IREGION_Pos) # !< MPU TYPE: IREGION Mask
      MPU_TYPE_DREGION_Pos* = 8
      MPU_TYPE_DREGION_Msk* = (0x000000FF shl MPU_TYPE_DREGION_Pos) # !< MPU TYPE: DREGION Mask
      MPU_TYPE_SEPARATE_Pos* = 0
      MPU_TYPE_SEPARATE_Msk* = (1) # !< MPU TYPE: SEPARATE Mask
    #  MPU Control Register Definitions
    const
      MPU_CTRL_PRIVDEFENA_Pos* = 2
      MPU_CTRL_PRIVDEFENA_Msk* = (1 shl MPU_CTRL_PRIVDEFENA_Pos) # !< MPU CTRL: PRIVDEFENA Mask
      MPU_CTRL_HFNMIENA_Pos* = 1
      MPU_CTRL_HFNMIENA_Msk* = (1 shl MPU_CTRL_HFNMIENA_Pos) # !< MPU CTRL: HFNMIENA Mask
      MPU_CTRL_ENABLE_Pos* = 0
      MPU_CTRL_ENABLE_Msk* = (1) # !< MPU CTRL: ENABLE Mask
    #  MPU Region Number Register Definitions
    const
      MPU_RNR_REGION_Pos* = 0
      MPU_RNR_REGION_Msk* = (0x000000FF) # !< MPU RNR: REGION Mask
    #  MPU Region Base Address Register Definitions
    const
      MPU_RBAR_ADDR_Pos* = 5
      MPU_RBAR_ADDR_Msk* = (0x07FFFFFF shl MPU_RBAR_ADDR_Pos) # !< MPU RBAR: ADDR Mask
      MPU_RBAR_VALID_Pos* = 4
      MPU_RBAR_VALID_Msk* = (1 shl MPU_RBAR_VALID_Pos) # !< MPU RBAR: VALID Mask
      MPU_RBAR_REGION_Pos* = 0
      MPU_RBAR_REGION_Msk* = (0x0000000F) # !< MPU RBAR: REGION Mask
    #  MPU Region Attribute and Size Register Definitions
    const
      MPU_RASR_ATTRS_Pos* = 16
      MPU_RASR_ATTRS_Msk* = (0x0000FFFF shl MPU_RASR_ATTRS_Pos) # !< MPU RASR: MPU Region Attribute field Mask
      MPU_RASR_XN_Pos* = 28
      MPU_RASR_XN_Msk* = (1 shl MPU_RASR_XN_Pos) # !< MPU RASR: ATTRS.XN Mask
      MPU_RASR_AP_Pos* = 24
      MPU_RASR_AP_Msk* = (0x00000007 shl MPU_RASR_AP_Pos) # !< MPU RASR: ATTRS.AP Mask
      MPU_RASR_TEX_Pos* = 19
      MPU_RASR_TEX_Msk* = (0x00000007 shl MPU_RASR_TEX_Pos) # !< MPU RASR: ATTRS.TEX Mask
      MPU_RASR_S_Pos* = 18
      MPU_RASR_S_Msk* = (1 shl MPU_RASR_S_Pos) # !< MPU RASR: ATTRS.S Mask
      MPU_RASR_C_Pos* = 17
      MPU_RASR_C_Msk* = (1 shl MPU_RASR_C_Pos) # !< MPU RASR: ATTRS.C Mask
      MPU_RASR_B_Pos* = 16
      MPU_RASR_B_Msk* = (1 shl MPU_RASR_B_Pos) # !< MPU RASR: ATTRS.B Mask
      MPU_RASR_SRD_Pos* = 8
      MPU_RASR_SRD_Msk* = (0x000000FF shl MPU_RASR_SRD_Pos) # !< MPU RASR: Sub-Region Disable Mask
      MPU_RASR_SIZE_Pos* = 1
      MPU_RASR_SIZE_Msk* = (0x0000001F shl MPU_RASR_SIZE_Pos) # !< MPU RASR: Region Size Field Mask
      MPU_RASR_ENABLE_Pos* = 0
      MPU_RASR_ENABLE_Msk* = (1) # !< MPU RASR: Region enable bit Disable Mask
    # @} end of group CMSIS_MPU
  when (FPU_PRESENT == 1):
    # *
    #   \ingroup  CMSIS_core_register
    #   \defgroup CMSIS_FPU     Floating Point Unit (FPU)
    #   \brief    Type definitions for the Floating Point Unit (FPU)
    #   @{
    #
    # *
    #   \brief  Structure type to access the Floating Point Unit (FPU).
    #
    type
      FPU_Type* {.bycopy.} = object
        RESERVED0*: array[1, uint32]
        FPCCR*: uint32         # !< Offset: 0x004 (R/W)  Floating-Point Context Control Register
        FPCAR*: uint32         # !< Offset: 0x008 (R/W)  Floating-Point Context Address Register
        FPDSCR*: uint32        # !< Offset: 0x00C (R/W)  Floating-Point Default Status Control Register
        MVFR0*: uint32         # !< Offset: 0x010 (R/ )  Media and FP Feature Register 0
        MVFR1*: uint32         # !< Offset: 0x014 (R/ )  Media and FP Feature Register 1

    #  Floating-Point Context Control Register Definitions
    const
      FPU_FPCCR_ASPEN_Pos* = 31
      FPU_FPCCR_ASPEN_Msk* = (1 shl FPU_FPCCR_ASPEN_Pos) # !< FPCCR: ASPEN bit Mask
      FPU_FPCCR_LSPEN_Pos* = 30
      FPU_FPCCR_LSPEN_Msk* = (1 shl FPU_FPCCR_LSPEN_Pos) # !< FPCCR: LSPEN bit Mask
      FPU_FPCCR_MONRDY_Pos* = 8
      FPU_FPCCR_MONRDY_Msk* = (1 shl FPU_FPCCR_MONRDY_Pos) # !< FPCCR: MONRDY bit Mask
      FPU_FPCCR_BFRDY_Pos* = 6
      FPU_FPCCR_BFRDY_Msk* = (1 shl FPU_FPCCR_BFRDY_Pos) # !< FPCCR: BFRDY bit Mask
      FPU_FPCCR_MMRDY_Pos* = 5
      FPU_FPCCR_MMRDY_Msk* = (1 shl FPU_FPCCR_MMRDY_Pos) # !< FPCCR: MMRDY bit Mask
      FPU_FPCCR_HFRDY_Pos* = 4
      FPU_FPCCR_HFRDY_Msk* = (1 shl FPU_FPCCR_HFRDY_Pos) # !< FPCCR: HFRDY bit Mask
      FPU_FPCCR_THREAD_Pos* = 3
      FPU_FPCCR_THREAD_Msk* = (1 shl FPU_FPCCR_THREAD_Pos) # !< FPCCR: processor mode active bit Mask
      FPU_FPCCR_USER_Pos* = 1
      FPU_FPCCR_USER_Msk* = (1 shl FPU_FPCCR_USER_Pos) # !< FPCCR: privilege level bit Mask
      FPU_FPCCR_LSPACT_Pos* = 0
      FPU_FPCCR_LSPACT_Msk* = (1) # !< FPCCR: Lazy state preservation active bit Mask
    #  Floating-Point Context Address Register Definitions
    const
      FPU_FPCAR_ADDRESS_Pos* = 3
      FPU_FPCAR_ADDRESS_Msk* = (0x1FFFFFFF shl FPU_FPCAR_ADDRESS_Pos) # !< FPCAR: ADDRESS bit Mask
    #  Floating-Point Default Status Control Register Definitions
    const
      FPU_FPDSCR_AHP_Pos* = 26
      FPU_FPDSCR_AHP_Msk* = (1 shl FPU_FPDSCR_AHP_Pos) # !< FPDSCR: AHP bit Mask
      FPU_FPDSCR_DN_Pos* = 25
      FPU_FPDSCR_DN_Msk* = (1 shl FPU_FPDSCR_DN_Pos) # !< FPDSCR: DN bit Mask
      FPU_FPDSCR_FZ_Pos* = 24
      FPU_FPDSCR_FZ_Msk* = (1 shl FPU_FPDSCR_FZ_Pos) # !< FPDSCR: FZ bit Mask
      FPU_FPDSCR_RMode_Pos* = 22
      FPU_FPDSCR_RMode_Msk* = (3 shl FPU_FPDSCR_RMode_Pos) # !< FPDSCR: RMode bit Mask
    #  Media and FP Feature Register 0 Definitions
    const
      FPU_MVFR0_FP_rounding_modes_Pos* = 28
      FPU_MVFR0_FP_rounding_modes_Msk* = (
        0x0000000F shl FPU_MVFR0_FP_rounding_modes_Pos) # !< MVFR0: FP rounding modes bits Mask
      FPU_MVFR0_Short_vectors_Pos* = 24
      FPU_MVFR0_Short_vectors_Msk* = (0x0000000F shl FPU_MVFR0_Short_vectors_Pos) # !< MVFR0: Short vectors bits Mask
      FPU_MVFR0_Square_root_Pos* = 20
      FPU_MVFR0_Square_root_Msk* = (0x0000000F shl FPU_MVFR0_Square_root_Pos) # !< MVFR0: Square root bits Mask
      FPU_MVFR0_Divide_Pos* = 16
      FPU_MVFR0_Divide_Msk* = (0x0000000F shl FPU_MVFR0_Divide_Pos) # !< MVFR0: Divide bits Mask
      FPU_MVFR0_FP_excep_trapping_Pos* = 12
      FPU_MVFR0_FP_excep_trapping_Msk* = (
        0x0000000F shl FPU_MVFR0_FP_excep_trapping_Pos) # !< MVFR0: FP exception trapping bits Mask
      FPU_MVFR0_Double_precision_Pos* = 8
      FPU_MVFR0_Double_precision_Msk* = (
        0x0000000F shl FPU_MVFR0_Double_precision_Pos) # !< MVFR0: Double-precision bits Mask
      FPU_MVFR0_Single_precision_Pos* = 4
      FPU_MVFR0_Single_precision_Msk* = (
        0x0000000F shl FPU_MVFR0_Single_precision_Pos) # !< MVFR0: Single-precision bits Mask
      FPU_MVFR0_A_SIMD_registers_Pos* = 0
      FPU_MVFR0_A_SIMD_registers_Msk* = (0x0000000F) # !< MVFR0: A_SIMD registers bits Mask
    #  Media and FP Feature Register 1 Definitions
    const
      FPU_MVFR1_FP_fused_MAC_Pos* = 28
      FPU_MVFR1_FP_fused_MAC_Msk* = (0x0000000F shl FPU_MVFR1_FP_fused_MAC_Pos) # !< MVFR1: FP fused MAC bits Mask
      FPU_MVFR1_FP_HPFP_Pos* = 24
      FPU_MVFR1_FP_HPFP_Msk* = (0x0000000F shl FPU_MVFR1_FP_HPFP_Pos) # !< MVFR1: FP HPFP bits Mask
      FPU_MVFR1_D_NaN_mode_Pos* = 4
      FPU_MVFR1_D_NaN_mode_Msk* = (0x0000000F shl FPU_MVFR1_D_NaN_mode_Pos) # !< MVFR1: D_NaN mode bits Mask
      FPU_MVFR1_FtZ_mode_Pos* = 0
      FPU_MVFR1_FtZ_mode_Msk* = (0x0000000F) # !< MVFR1: FtZ mode bits Mask
    # @} end of group CMSIS_FPU
  # *
  #   \ingroup  CMSIS_core_register
  #   \defgroup CMSIS_CoreDebug       Core Debug Registers (CoreDebug)
  #   \brief    Type definitions for the Core Debug Registers
  #   @{
  #
  # *
  #   \brief  Structure type to access the Core Debug Register (CoreDebug).
  #
  type
    CoreDebug_Type* {.bycopy.} = object
      DHCSR*: uint32           # !< Offset: 0x000 (R/W)  Debug Halting Control and Status Register
      DCRSR*: uint32           # !< Offset: 0x004 ( /W)  Debug Core Register Selector Register
      DCRDR*: uint32           # !< Offset: 0x008 (R/W)  Debug Core Register Data Register
      DEMCR*: uint32           # !< Offset: 0x00C (R/W)  Debug Exception and Monitor Control Register

  #  Debug Halting Control and Status Register Definitions
  const
    CoreDebug_DHCSR_DBGKEY_Pos* = 16
    CoreDebug_DHCSR_DBGKEY_Msk* = (0x0000FFFF shl CoreDebug_DHCSR_DBGKEY_Pos) # !< CoreDebug DHCSR: DBGKEY Mask
    CoreDebug_DHCSR_S_RESET_ST_Pos* = 25
    CoreDebug_DHCSR_S_RESET_ST_Msk* = (1 shl CoreDebug_DHCSR_S_RESET_ST_Pos) # !< CoreDebug DHCSR: S_RESET_ST Mask
    CoreDebug_DHCSR_S_RETIRE_ST_Pos* = 24
    CoreDebug_DHCSR_S_RETIRE_ST_Msk* = (1 shl CoreDebug_DHCSR_S_RETIRE_ST_Pos) # !< CoreDebug DHCSR: S_RETIRE_ST Mask
    CoreDebug_DHCSR_S_LOCKUP_Pos* = 19
    CoreDebug_DHCSR_S_LOCKUP_Msk* = (1 shl CoreDebug_DHCSR_S_LOCKUP_Pos) # !< CoreDebug DHCSR: S_LOCKUP Mask
    CoreDebug_DHCSR_S_SLEEP_Pos* = 18
    CoreDebug_DHCSR_S_SLEEP_Msk* = (1 shl CoreDebug_DHCSR_S_SLEEP_Pos) # !< CoreDebug DHCSR: S_SLEEP Mask
    CoreDebug_DHCSR_S_HALT_Pos* = 17
    CoreDebug_DHCSR_S_HALT_Msk* = (1 shl CoreDebug_DHCSR_S_HALT_Pos) # !< CoreDebug DHCSR: S_HALT Mask
    CoreDebug_DHCSR_S_REGRDY_Pos* = 16
    CoreDebug_DHCSR_S_REGRDY_Msk* = (1 shl CoreDebug_DHCSR_S_REGRDY_Pos) # !< CoreDebug DHCSR: S_REGRDY Mask
    CoreDebug_DHCSR_C_SNAPSTALL_Pos* = 5
    CoreDebug_DHCSR_C_SNAPSTALL_Msk* = (1 shl CoreDebug_DHCSR_C_SNAPSTALL_Pos) # !< CoreDebug DHCSR: C_SNAPSTALL Mask
    CoreDebug_DHCSR_C_MASKINTS_Pos* = 3
    CoreDebug_DHCSR_C_MASKINTS_Msk* = (1 shl CoreDebug_DHCSR_C_MASKINTS_Pos) # !< CoreDebug DHCSR: C_MASKINTS Mask
    CoreDebug_DHCSR_C_STEP_Pos* = 2
    CoreDebug_DHCSR_C_STEP_Msk* = (1 shl CoreDebug_DHCSR_C_STEP_Pos) # !< CoreDebug DHCSR: C_STEP Mask
    CoreDebug_DHCSR_C_HALT_Pos* = 1
    CoreDebug_DHCSR_C_HALT_Msk* = (1 shl CoreDebug_DHCSR_C_HALT_Pos) # !< CoreDebug DHCSR: C_HALT Mask
    CoreDebug_DHCSR_C_DEBUGEN_Pos* = 0
    CoreDebug_DHCSR_C_DEBUGEN_Msk* = (1) # !< CoreDebug DHCSR: C_DEBUGEN Mask
  #  Debug Core Register Selector Register Definitions
  const
    CoreDebug_DCRSR_REGWnR_Pos* = 16
    CoreDebug_DCRSR_REGWnR_Msk* = (1 shl CoreDebug_DCRSR_REGWnR_Pos) # !< CoreDebug DCRSR: REGWnR Mask
    CoreDebug_DCRSR_REGSEL_Pos* = 0
    CoreDebug_DCRSR_REGSEL_Msk* = (0x0000001F) # !< CoreDebug DCRSR: REGSEL Mask
  #  Debug Exception and Monitor Control Register Definitions
  const
    CoreDebug_DEMCR_TRCENA_Pos* = 24
    CoreDebug_DEMCR_TRCENA_Msk* = (1 shl CoreDebug_DEMCR_TRCENA_Pos) # !< CoreDebug DEMCR: TRCENA Mask
    CoreDebug_DEMCR_MON_REQ_Pos* = 19
    CoreDebug_DEMCR_MON_REQ_Msk* = (1 shl CoreDebug_DEMCR_MON_REQ_Pos) # !< CoreDebug DEMCR: MON_REQ Mask
    CoreDebug_DEMCR_MON_STEP_Pos* = 18
    CoreDebug_DEMCR_MON_STEP_Msk* = (1 shl CoreDebug_DEMCR_MON_STEP_Pos) # !< CoreDebug DEMCR: MON_STEP Mask
    CoreDebug_DEMCR_MON_PEND_Pos* = 17
    CoreDebug_DEMCR_MON_PEND_Msk* = (1 shl CoreDebug_DEMCR_MON_PEND_Pos) # !< CoreDebug DEMCR: MON_PEND Mask
    CoreDebug_DEMCR_MON_EN_Pos* = 16
    CoreDebug_DEMCR_MON_EN_Msk* = (1 shl CoreDebug_DEMCR_MON_EN_Pos) # !< CoreDebug DEMCR: MON_EN Mask
    CoreDebug_DEMCR_VC_HARDERR_Pos* = 10
    CoreDebug_DEMCR_VC_HARDERR_Msk* = (1 shl CoreDebug_DEMCR_VC_HARDERR_Pos) # !< CoreDebug DEMCR: VC_HARDERR Mask
    CoreDebug_DEMCR_VC_INTERR_Pos* = 9
    CoreDebug_DEMCR_VC_INTERR_Msk* = (1 shl CoreDebug_DEMCR_VC_INTERR_Pos) # !< CoreDebug DEMCR: VC_INTERR Mask
    CoreDebug_DEMCR_VC_BUSERR_Pos* = 8
    CoreDebug_DEMCR_VC_BUSERR_Msk* = (1 shl CoreDebug_DEMCR_VC_BUSERR_Pos) # !< CoreDebug DEMCR: VC_BUSERR Mask
    CoreDebug_DEMCR_VC_STATERR_Pos* = 7
    CoreDebug_DEMCR_VC_STATERR_Msk* = (1 shl CoreDebug_DEMCR_VC_STATERR_Pos) # !< CoreDebug DEMCR: VC_STATERR Mask
    CoreDebug_DEMCR_VC_CHKERR_Pos* = 6
    CoreDebug_DEMCR_VC_CHKERR_Msk* = (1 shl CoreDebug_DEMCR_VC_CHKERR_Pos) # !< CoreDebug DEMCR: VC_CHKERR Mask
    CoreDebug_DEMCR_VC_NOCPERR_Pos* = 5
    CoreDebug_DEMCR_VC_NOCPERR_Msk* = (1 shl CoreDebug_DEMCR_VC_NOCPERR_Pos) # !< CoreDebug DEMCR: VC_NOCPERR Mask
    CoreDebug_DEMCR_VC_MMERR_Pos* = 4
    CoreDebug_DEMCR_VC_MMERR_Msk* = (1 shl CoreDebug_DEMCR_VC_MMERR_Pos) # !< CoreDebug DEMCR: VC_MMERR Mask
    CoreDebug_DEMCR_VC_CORERESET_Pos* = 0
    CoreDebug_DEMCR_VC_CORERESET_Msk* = (1) # !< CoreDebug DEMCR: VC_CORERESET Mask
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
  #  #define _VAL2FLD(field, value)    ((value << field # _Pos) & field # _Msk)
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
  #  Memory mapping of Cortex-M4 Hardware
  const
    SCS_BASE* = (0xE000E000)    # !< System Control Space Base Address
    ITM_BASE* = (0xE0000000)    # !< ITM Base Address
    DWT_BASE* = (0xE0001000)    # !< DWT Base Address
    TPI_BASE* = (0xE0040000)    # !< TPI Base Address
    CoreDebug_BASE* = (0xE000EDF0) # !< Core Debug Base Address
    SysTick_BASE* = (SCS_BASE + 0x00000010) # !< SysTick Base Address
    NVIC_BASE* = (SCS_BASE + 0x00000100) # !< NVIC Base Address
    SCB_BASE* = (SCS_BASE + 0x00000D00) # !< System Control Block Base Address
    SCnSCB* = (cast[ptr SCnSCB_Type](SCS_BASE)) # !< System control Register not in SCB
    SCB* = (cast[ptr SCB_Type](SCB_BASE)) # !< SCB configuration struct
    SysTick* = (cast[ptr SysTick_Type](SysTick_BASE)) # !< SysTick configuration struct
    NVIC* = (cast[ptr NVIC_Type](NVIC_BASE)) # !< NVIC configuration struct
    ITM* = (cast[ptr ITM_Type](ITM_BASE)) # !< ITM configuration struct
    DWT* = (cast[ptr DWT_Type](DWT_BASE)) # !< DWT configuration struct
    TPI* = (cast[ptr TPI_Type](TPI_BASE)) # !< TPI configuration struct
    CoreDebug* = (cast[ptr CoreDebug_Type](CoreDebug_BASE)) # !< Core Debug configuration struct
  when (MPU_PRESENT == 1):
    const
      MPU_BASE* = (SCS_BASE + 0x00000D90) # !< Memory Protection Unit
      MPU* = (cast[ptr MPU_Type](MPU_BASE)) # !< Memory Protection Unit
  when (FPU_PRESENT == 1):
    const
      FPU_BASE* = (SCS_BASE + 0x00000F30) # !< Floating Point Unit
      FPU* = (cast[ptr FPU_Type](FPU_BASE)) # !< Floating Point Unit
  # @}
  # ******************************************************************************
  #                 Hardware Abstraction Layer
  #   Core Function Interface contains:
  #   - Core NVIC Functions
  #   - Core SysTick Functions
  #   - Core Debug Functions
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
  # *
  #   \brief   Set Priority Grouping
  #   \details Sets the priority grouping field using the required unlock sequence.
  #            The parameter PriorityGroup is assigned to the field SCB->AIRCR [10:8] PRIGROUP field.
  #            Only values from 0..7 are used.
  #            In case of a conflict between priority grouping and available
  #            priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.
  #   \param [in]      PriorityGroup  Priority grouping field.
  #
  proc NVIC_SetPriorityGrouping*(PriorityGroup: uint32) {.inline.} =
    var reg_value: uint32
    var PriorityGroupTmp: uint32 = (PriorityGroup and cast[uint32](0x00000007))
    #  only values 0..7 are used
    reg_value = SCB.AIRCR
    #  read old register configuration
    reg_value = reg_value and
        not (cast[uint32](SCB_AIRCR_VECTKEY_Msk or SCB_AIRCR_PRIGROUP_Msk))
    #  clear bits to change
    reg_value = (reg_value or
        (cast[uint32](0x000005FA) shl SCB_AIRCR_VECTKEY_Pos) or
        (PriorityGroupTmp shl 8))
    #  Insert write key and priorty group
    SCB.AIRCR = reg_value

  # *
  #   \brief   Get Priority Grouping
  #   \details Reads the priority grouping field from the NVIC Interrupt Controller.
  #   \return                Priority grouping field (SCB->AIRCR [10:8] PRIGROUP field).
  #
  proc NVIC_GetPriorityGrouping*(): uint32 {.inline.} =
    return (uint32)((SCB.AIRCR and SCB_AIRCR_PRIGROUP_Msk) shr
        SCB_AIRCR_PRIGROUP_Pos)

  # *
  #   \brief   Enable External Interrupt
  #   \details Enables a device-specific interrupt in the NVIC interrupt controller.
  #   \param [in]      IRQn  External interrupt number. Value cannot be negative.
  #
  proc NVIC_EnableIRQ*(IRQn: IRQn_Type) {.inline.} =
    NVIC.ISER[((cast[int32](IRQn)) shr 5)].st (uint32)(
        1 shl ((cast[int32](IRQn)) and 0x0000001F))

  # *
  #   \brief   Disable External Interrupt
  #   \details Disables a device-specific interrupt in the NVIC interrupt controller.
  #   \param [in]      IRQn  External interrupt number. Value cannot be negative.
  #
  proc NVIC_DisableIRQ*(IRQn: IRQn_Type) {.inline.} =
    NVIC.ICER[((cast[int32](IRQn)) shr 5)].st (uint32)(
        1 shl ((cast[int32](IRQn)) and 0x0000001F))

  # *
  #   \brief   Get Pending Interrupt
  #   \details Reads the pending register in the NVIC and returns the pending bit for the specified interrupt.
  #   \param [in]      IRQn  Interrupt number.
  #   \return             0  Interrupt status is not pending.
  #   \return             1  Interrupt status is pending.
  #
  proc NVIC_GetPendingIRQ*(IRQn: IRQn_Type): uint32 {.inline.} =
    return
      if (
              NVIC.ISPR[((cast[uint32](IRQn)) shr 5)].ld and
              (1 shl ((cast[uint32](IRQn)) and 0x0000001F)).uint32
        ) != 0:
        1.uint32
      else:
        0.uint32

  # *
  #   \brief   Set Pending Interrupt
  #   \details Sets the pending bit of an external interrupt.
  #   \param [in]      IRQn  Interrupt number. Value cannot be negative.
  #
  proc NVIC_SetPendingIRQ*(IRQn: IRQn_Type) {.inline.} =
    NVIC.ISPR[((cast[int32](IRQn)) shr 5)].st (uint32)(
        1 shl ((cast[int32](IRQn)) and 0x0000001F))

  # *
  #   \brief   Clear Pending Interrupt
  #   \details Clears the pending bit of an external interrupt.
  #   \param [in]      IRQn  External interrupt number. Value cannot be negative.
  #
  proc NVIC_ClearPendingIRQ*(IRQn: IRQn_Type) {.inline.} =
    NVIC.ICPR[((cast[uint32](IRQn)) shr 5)].st (uint32)(
        1 shl ((cast[uint32](IRQn)) and 0x0000001F))

  # *
  #   \brief   Get Active Interrupt
  #   \details Reads the active register in NVIC and returns the active bit.
  #   \param [in]      IRQn  Interrupt number.
  #   \return             0  Interrupt status is not active.
  #   \return             1  Interrupt status is active.
  #
  proc NVIC_GetActive*(IRQn: IRQn_Type): uint32 {.inline.} =
    return (if ((NVIC.IABR[((cast[uint32](IRQn)) shr 5)].ld and
        (1 shl ((cast[uint32](IRQn)) and 0x0000001F)).uint32) != 0): 1 else: 0).uint32

  # *
  #   \brief   Set Interrupt Priority
  #   \details Sets the priority of an interrupt.
  #   \note    The priority cannot be set for every core interrupt.
  #   \param [in]      IRQn  Interrupt number.
  #   \param [in]  priority  Priority to set.
  #
  proc NVIC_SetPriority*(IRQn: IRQn_Type; priority: uint32) {.inline.} =
    if (int32)(IRQn) < 0:
      SCB.SHP[(((cast[uint32](IRQn)) and 0x0000000F.uint32) - 4).uint8].st  (uint8)(
          (priority shl (8 - NVIC_PRIO_BITS)) and cast[uint32](0x000000FF))
    else:
      NVIC.IP[(cast[uint32](IRQn))].st (uint8)(
          (priority shl (8 - NVIC_PRIO_BITS)) and cast[uint32](0x000000FF))

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
      return (cast[uint32](SCB.SHP[(((cast[uint32](IRQn)) and 0x0000000F) - 4).uint8].ld) shr
          (8 - NVIC_PRIO_BITS))
    else:
      return (cast[uint32](NVIC.IP[(cast[uint32](IRQn))].ld) shr
          (8 - NVIC_PRIO_BITS))

  # *
  #   \brief   Encode Priority
  #   \details Encodes the priority for an interrupt with the given priority group,
  #            preemptive priority value, and subpriority value.
  #            In case of a conflict between priority grouping and available
  #            priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.
  #   \param [in]     PriorityGroup  Used priority group.
  #   \param [in]   PreemptPriority  Preemptive priority value (starting from 0).
  #   \param [in]       SubPriority  Subpriority value (starting from 0).
  #   \return                        Encoded priority. Value can be used in the function \ref NVIC_SetPriority().
  #
  proc NVIC_EncodePriority*(PriorityGroup: uint32; PreemptPriority: uint32;
                           SubPriority: uint32): uint32 {.inline.} =
    var PriorityGroupTmp: uint32 = (PriorityGroup and cast[uint32](0x00000007))
    #  only values 0..7 are used
    var PreemptPriorityBits: uint32
    var SubPriorityBits: uint32
    PreemptPriorityBits = if ((7'u32 - PriorityGroupTmp) > (uint32)(NVIC_PRIO_BITS)): (
        uint32)(NVIC_PRIO_BITS) else: (uint32)(7'u32 - PriorityGroupTmp)
    SubPriorityBits = if ((PriorityGroupTmp + (uint32)(NVIC_PRIO_BITS)) <
        cast[uint32](7)): cast[uint32](0) else: (uint32)(
        (PriorityGroupTmp - 7) + (uint32)(NVIC_PRIO_BITS))
    return ((PreemptPriority and (uint32)((1 shl (PreemptPriorityBits)) - 1)) shl
        SubPriorityBits) or
        ((SubPriority and (uint32)((1 shl (SubPriorityBits)) - 1)))

  # *
  #   \brief   Decode Priority
  #   \details Decodes an interrupt priority value with a given priority group to
  #            preemptive priority value and subpriority value.
  #            In case of a conflict between priority grouping and available
  #            priority bits (__NVIC_PRIO_BITS) the smallest possible priority group is set.
  #   \param [in]         Priority   Priority value, which can be retrieved with the function \ref NVIC_GetPriority().
  #   \param [in]     PriorityGroup  Used priority group.
  #   \param [out] pPreemptPriority  Preemptive priority value (starting from 0).
  #   \param [out]     pSubPriority  Subpriority value (starting from 0).
  #
  proc NVIC_DecodePriority*(Priority: uint32; PriorityGroup: uint32;
                           pPreemptPriority: ptr uint32; pSubPriority: ptr uint32) {.
      inline.} =
    var PriorityGroupTmp: uint32 = (PriorityGroup and cast[uint32](0x00000007))
    #  only values 0..7 are used
    var PreemptPriorityBits: uint32
    var SubPriorityBits: uint32
    PreemptPriorityBits = if ((7'u32 - PriorityGroupTmp) > (uint32)(NVIC_PRIO_BITS)): (
        uint32)(NVIC_PRIO_BITS) else: (uint32)(7'u32 - PriorityGroupTmp)
    SubPriorityBits = if ((PriorityGroupTmp + (uint32)(NVIC_PRIO_BITS)) <
        cast[uint32](7)): cast[uint32](0) else: (uint32)(
        (PriorityGroupTmp - 7) + (uint32)(NVIC_PRIO_BITS))
    pPreemptPriority[].st ((Priority shr SubPriorityBits) and
        (uint32)((1 shl (PreemptPriorityBits)) - 1))
    pSubPriority[].st ((Priority) and (uint32)((1 shl (SubPriorityBits)) - 1))

  # *
  #   \brief   System Reset
  #   \details Initiates a system reset request to reset the MCU.
  #
  proc NVIC_SystemReset*() {.inline.} =
    DSB()
    #  Ensure all outstanding memory accesses included
    #                                                                        buffered write are completed before reset
    SCB.AIRCR = (
            (0x000005FA'u32 shl SCB_AIRCR_VECTKEY_Pos) or
            (SCB.AIRCR and SCB_AIRCR_PRIGROUP_Msk) or SCB_AIRCR_SYSRESETREQ_Msk).uint32
    #  Keep priority group unchanged
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
    #   \note    When the variable <b>__Vendor_SysTickConfig</b> is set to 1, then the
    #            function <b>SysTick_Config</b> is not included. In this case, the file <b><i>device</i>.h</b>
    #            must contain a vendor-specific implementation of this function.
    #
    proc SysTick_Config*(ticks: uint32): uint32 {.inline,discardable.} =
      if (ticks - 1).uint32 > SysTick_LOAD_RELOAD_Msk.uint32:
        return 1'u32 #  Reload value impossible
      SysTick.LOAD.st  (uint32)(ticks - 1) #  set reload register
      NVIC_SetPriority(SysTick_IRQn, (1 shl NVIC_PRIO_BITS) - 1) #  set Priority for Systick Interrupt
      SysTick.VAL.st 0 #  Load the SysTick Counter Value
      SysTick.CTRL.st  (SysTick_CTRL_CLKSOURCE_Msk or SysTick_CTRL_TICKINT_Msk or
          SysTick_CTRL_ENABLE_Msk) #  Enable SysTick IRQ and SysTick Timer
      return 0 #  Function successful

  # @} end of CMSIS_Core_SysTickFunctions
  #  ################## Debug In/Output function ######################
  # *
  #   \ingroup  CMSIS_Core_FunctionInterface
  #   \defgroup CMSIS_core_DebugFunctions ITM Functions
  #   \brief    Functions that access the ITM debug interface.
  #   @{
  #
  var ITM_RxBuffer*: int32
  # !< External variable to receive characters.
  const
    ITM_RXBUFFER_EMPTY* = 0x5AA55AA5
  # *
  #   \brief   ITM Send Character
  #   \details Transmits a character via the ITM channel 0, and
  #            \li Just returns when no debugger is connected that has booked the output.
  #            \li Is blocking when a debugger is connected, but the previous character sent has not been transmitted.
  #   \param [in]     ch  Character to transmit.
  #   \returns            Character to transmit.
  #
  proc ITM_SendChar*(ch: uint32): uint32 {.inline.} =
    if ((ITM.TCR and ITM_TCR_ITMENA_Msk) != 0) and ((ITM.TER and 1) != 0): #  ITM enabled
      while ITM.PORT[0].u32 == 0:
        NOP()
      ITM.PORT[0].u8 = cast[uint8](ch)
    return ch

  # *
  #   \brief   ITM Receive Character
  #   \details Inputs a character via the external variable \ref ITM_RxBuffer.
  #   \return             Received character.
  #   \return         -1  No character pending.
  #
  proc ITM_ReceiveChar*(): int32 {.inline.} =
    var ch: int32 = -1
    #  no character available
    if ITM_RxBuffer != ITM_RXBUFFER_EMPTY:
      ch = ITM_RxBuffer
      ITM_RxBuffer = ITM_RXBUFFER_EMPTY
      #  ready for next character
    return ch

  # *
  #   \brief   ITM Check Character
  #   \details Checks whether a character is pending for reading in the variable \ref ITM_RxBuffer.
  #   \return          0  No character available.
  #   \return          1  Character available.
  #
  proc ITM_CheckChar*(): int32 {.inline.} =
    if ITM_RxBuffer == ITM_RXBUFFER_EMPTY:
      return 0
      #  no character available
    else:
      return 1
      #     character available

  # @} end of CMSIS_core_DebugFunctions
