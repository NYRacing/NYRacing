/*
 * Code generated for Simulink model VehCtrlMdel240912_2018b.
 *
 * FILE    : VehCtrlMdel240912_2018b.c
 *
 * VERSION : 1.123
 *
 * DATE    : Sat Sep 14 22:02:59 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#include "VehCtrlMdel240912_2018b.h"
#include "VehCtrlMdel240912_2018b_private.h"

/* #include "myinclude.h" */

/* Named constants for Chart: '<S8>/Timer1' */
#define VehCtrlMdel240912_2018b_IN_Out ((uint8_T)2U)
#define VehCtrlMdel240912_20_IN_Trigger ((uint8_T)3U)
#define VehCtrlMdel240912_IN_InterState ((uint8_T)1U)

/* Named constants for Chart: '<S7>/Chart' */
#define VehCtrlMdel240912_2018b_IN_A   ((uint8_T)1U)
#define VehCtrlMdel240912_2018b_IN_B   ((uint8_T)2U)
#define VehCtrlMdel240912_2018b_IN_C   ((uint8_T)3U)
#define VehCtrlMdel240912_IN_DYC_Enable ((uint8_T)5U)
#define VehCtrlMdel240912__IN_InitState ((uint8_T)7U)
#define VehCtrlMdel24091_IN_TCSF_Enable ((uint8_T)9U)
#define VehCtrlMdel240_IN_DYC_Disenable ((uint8_T)4U)
#define VehCtrlMdel24_IN_TCSF_Disenable ((uint8_T)8U)
#define VehCtrlMdel24_IN_TCSR_Disenable ((uint8_T)10U)
#define VehCtrlMdel2_IN_F_TVD_TCS_STATE ((uint8_T)6U)

/* Named constants for Chart: '<S90>/Chart2' */
#define VehCtrlM_IN_MC_InverterOn_State ((uint8_T)8U)
#define VehCtrlMdel240912_2018_IN_Guard ((uint8_T)1U)
#define VehCtrlMdel240912_2018_IN_Ready ((uint8_T)5U)
#define VehCtrlMdel240912_2018_IN_Trans ((uint8_T)7U)
#define VehCtrlMdel240912_2018_IN_start ((uint8_T)10U)
#define VehCtrlMdel240912_2018b_IN_Init ((uint8_T)2U)
#define VehCtrlMdel240912_2018b_IN_OFF ((uint8_T)1U)
#define VehCtrlMdel240912_2018b_IN_ON  ((uint8_T)2U)
#define VehCtrlMdel240912_201_IN_AMKCAN ((uint8_T)1U)
#define VehCtrlMdel240912_20_IN_Standby ((uint8_T)6U)
#define VehCtrlMdel240912_IN_MC_DCready ((uint8_T)7U)
#define VehCtrlMdel240912_IN_SYSRDYCECK ((uint8_T)9U)
#define VehCtrlMdel240912_event_AMKDCON (3)
#define VehCtrlMdel240912_event_EbeepON (5)
#define VehCtrlMdel24091_IN_MCDCOncheck ((uint8_T)5U)
#define VehCtrlMdel24091_IN_MC_DCOnFail ((uint8_T)6U)
#define VehCtrlMdel24091_event_AMKCANON (1)
#define VehCtrlMdel24091_event_AMKDCOFF (2)
#define VehCtrlMdel24091_event_EbeepOFF (4)
#define VehCtrlMdel2409_IN_MCUReadyFail ((uint8_T)4U)
#define VehCtrlMdel2409_event_AMKCANOFF (0)
#define VehCtrlMdel240_IN_AMKDCOnFinish ((uint8_T)2U)
#define VehCtrlMdel240_IN_DCOnCheckPass ((uint8_T)3U)
#define VehCtrlMdel240_IN_InitStateBack ((uint8_T)3U)
#define VehCtrlMdel240_IN_WaitForEngine ((uint8_T)8U)
#define VehCtrlMdel240_event_InverterON (7)
#define VehCtrlMdel24_event_InverterOFF (6)
#define VehCtrlMdel2_IN_MCDCEnableState ((uint8_T)4U)
#define VehCtrlMdel2_event_MCDCEnableON (9)
#define VehCtrlMdel_event_MCDCEnableOFF (8)

/* Named constants for Chart: '<S151>/Timer' */
#define VehCtrlMdel240912_2018_IN_Out_n ((uint8_T)2U)
#define VehCtrlMdel240912__IN_Trigger_n ((uint8_T)3U)
#define VehCtrlMdel2409_IN_InterState_o ((uint8_T)1U)

EE_Data ecflashdataold[1024];
EE_Data ecflashdatanew[1024];

/* Exported block signals */
real_T Gear_Trs;                       /* '<S282>/Switch2' */
real_T Mode_Trs;                       /* '<S282>/Switch3' */
real_T Trq_CUT;                        /* '<S151>/Timer' */
real_T L12V_error;                     /* '<S127>/CAN Unpack' */
real_T alarm;                          /* '<S127>/CAN Unpack' */
real_T controller_ready;               /* '<S127>/CAN Unpack' */
real_T selfcheck;                      /* '<S127>/CAN Unpack' */
real_T RPM;                            /* '<S127>/CAN Unpack' */
real_T trq;                            /* '<S127>/CAN Unpack' */
real_T AC_current;                     /* '<S122>/CAN Unpack' */
real_T DC_current;                     /* '<S122>/CAN Unpack' */
real_T MCU_Temp;                       /* '<S122>/CAN Unpack' */
real_T motor_Temp;                     /* '<S122>/CAN Unpack' */
real_T StrWhlAngAliveRollCnt;          /* '<S144>/CAN Unpack1' */
real_T StrWhlAng;                      /* '<S144>/CAN Unpack1' */
real_T StrWhlAngV;                     /* '<S144>/CAN Unpack1' */
real_T IMU_Ax_Valid;                   /* '<S135>/CAN Unpack' */
real_T IMU_Ax_Value;                   /* '<S135>/CAN Unpack' */
real_T IMU_Ay_Valid;                   /* '<S135>/CAN Unpack' */
real_T IMU_Ay_Value;                   /* '<S135>/CAN Unpack' */
real_T IMU_DTC;                        /* '<S135>/CAN Unpack' */
real_T IMU_Yaw_Valid;                  /* '<S135>/CAN Unpack' */
real_T IMU_Yaw_Value;                  /* '<S135>/CAN Unpack' */
uint32_T Acc_vol;                      /* '<S151>/Add2' */
uint32_T Acc_vol2;                     /* '<S151>/Add3' */
uint32_T Acc_POS1;                     /* '<S151>/1-D Lookup Table' */
uint32_T Acc_POS2;                     /* '<S151>/1-D Lookup Table3' */
uint16_T F_BrkPrs;                     /* '<S151>/1-D Lookup Table1' */
boolean_T ignition;                    /* '<S101>/Logical Operator' */
boolean_T beeper_state;                /* '<S90>/Chart2' */

/* Block signals (default storage) */
B_VehCtrlMdel240912_2018b_T VehCtrlMdel240912_2018b_B;

/* Block states (default storage) */
DW_VehCtrlMdel240912_2018b_T VehCtrlMdel240912_2018b_DW;

/* Real-time model */
RT_MODEL_VehCtrlMdel240912_20_T VehCtrlMdel240912_2018b_M_;
RT_MODEL_VehCtrlMdel240912_20_T *const VehCtrlMdel240912_2018b_M =
  &VehCtrlMdel240912_2018b_M_;

/* Forward declaration for local functions */
static void VehC_enter_atomic_WaitForEngine(void);
static void VehCtrlMdel240912_2018b_VehStat(const boolean_T *ignition_d, const
  boolean_T *HVCUTOFF, const real_T *MCFL_bSystemReady, const real_T
  *MCFR_bSystemReady, const real_T *controller_ready_e, const boolean_T *Compare);
static void VehCtr_enter_atomic_MC_DCOnFail(void);
static void VehCtrlMdel_enter_atomic_AMKCAN(void);
static void rate_monotonic_scheduler(void);
void ISR_PIT_CH3(void)
{
  PIT_0.TIMER[3].TFLG.R = 1;
  ECUCoderModelBaseCounter++;
  rate_monotonic_scheduler();
}

void ec_flash_operation(void)
{
  uint32 i = 0;
  uint16 counter = 0;
  uint8 startkey[8] = { 0x01, 0x5A, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 };

  boolean_T kl15s = 1;
  N256K_BLOCK_SEL n256KBlockSelect;
  CONTEXT_DATA pgmCtxData;
  n256KBlockSelect.first256KBlockSelect = 0x00000000;
  n256KBlockSelect.second256KBlockSelect = 0x00000000;
  pgmCtxData.pReqCompletionFn = pFlashProgram;
  AfterRunFlags[1] = 1;
  ec_gpio_write(58,1);
  kl15s = ec_gpio_read(107);
  if (0 == kl15s && AfterRunFlags[0] == 0) {
    i= 1000000;
    while (i--) {
      ;
    }

    kl15s = ec_gpio_read(107);
    if (0 == kl15s && AfterRunFlags[0] == 0) {
      for (i=0; i<1024; i++) {
        if (ecflashdataold[i].U != ecflashdatanew[i].U) {
          counter++;
        }
      }

      if (counter > 0) {
        DisableInterrupts();
        App_FlashErase( &ssdConfig, 0, 0x00000000, 0x00000000, 0x00000001,
                       n256KBlockSelect );
        App_FlashProgram( &ssdConfig, 0, 0xFA0000, 8, (uint32)startkey,
                         &pgmCtxData );
        App_FlashProgram( &ssdConfig, 0, 0xFA0010, 4096, (uint32)ecflashdatanew,
                         &pgmCtxData );
        i= 1000;
        while (i--) {
          ;
        }
      }

      AfterRunFlags[1] = 0;
      ec_gpio_write(58,0);
      i= 60000;
      while (i--) {
        ;
      }
    }
  }
}

uint32_T look1_iu32n16bflftfIu32_binlc(uint32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex)
{
  real32_T uCast;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Clip'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  uCast = (real32_T)u0 * 1.52587891E-5F;
  if ((real32_T)u0 * 1.52587891E-5F < bp0[0U]) {
    iLeft = 0U;
    uCast = 0.0F;
  } else if (uCast < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (uCast < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    uCast = ((real32_T)u0 * 1.52587891E-5F - bp0[iLeft]) / (bp0[iLeft + 1U] -
      bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    uCast = 1.0F;
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  return (uint32_T)((table[iLeft + 1U] - table[iLeft]) * uCast * 65536.0F) +
    (uint32_T)(table[iLeft] * 65536.0F);
}

real_T look1_binlx(real_T u0, const real_T bp0[], const real_T table[], uint32_T
                   maxIndex)
{
  real_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

uint16_T look1_iu16bflftfIu16_binlc(uint16_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex)
{
  real32_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Clip'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 < bp0[0U]) {
    iLeft = 0U;
    frac = 0.0F;
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = ((real32_T)u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = 1.0F;
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  return (uint16_T)((uint32_T)(uint16_T)((table[iLeft + 1U] - table[iLeft]) *
    frac) + (uint16_T)table[iLeft]);
}

real32_T look2_iflf_binlx(real32_T u0, real32_T u1, const real32_T bp0[], const
  real32_T bp1[], const real32_T table[], const uint32_T maxIndex[], uint32_T
  stride)
{
  real32_T frac;
  uint32_T bpIndices[2];
  real32_T fractions[2];
  real32_T yL_1d;
  uint32_T iRght;
  uint32_T bpIdx;
  uint32_T iLeft;

  /* Column-major Lookup 2-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex[0U]]) {
    /* Binary Search */
    bpIdx = maxIndex[0U] >> 1U;
    iLeft = 0U;
    iRght = maxIndex[0U];
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex[0U] - 1U;
    frac = (u0 - bp0[maxIndex[0U] - 1U]) / (bp0[maxIndex[0U]] - bp0[maxIndex[0U]
      - 1U]);
  }

  fractions[0U] = frac;
  bpIndices[0U] = iLeft;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u1 <= bp1[0U]) {
    iLeft = 0U;
    frac = (u1 - bp1[0U]) / (bp1[1U] - bp1[0U]);
  } else if (u1 < bp1[maxIndex[1U]]) {
    /* Binary Search */
    bpIdx = maxIndex[1U] >> 1U;
    iLeft = 0U;
    iRght = maxIndex[1U];
    while (iRght - iLeft > 1U) {
      if (u1 < bp1[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u1 - bp1[iLeft]) / (bp1[iLeft + 1U] - bp1[iLeft]);
  } else {
    iLeft = maxIndex[1U] - 1U;
    frac = (u1 - bp1[maxIndex[1U] - 1U]) / (bp1[maxIndex[1U]] - bp1[maxIndex[1U]
      - 1U]);
  }

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  bpIdx = iLeft * stride + bpIndices[0U];
  yL_1d = (table[bpIdx + 1U] - table[bpIdx]) * fractions[0U] + table[bpIdx];
  bpIdx += stride;
  return (((table[bpIdx + 1U] - table[bpIdx]) * fractions[0U] + table[bpIdx]) -
          yL_1d) * frac + yL_1d;
}

real_T look2_binlx(real_T u0, real_T u1, const real_T bp0[], const real_T bp1[],
                   const real_T table[], const uint32_T maxIndex[], uint32_T
                   stride)
{
  real_T frac;
  uint32_T bpIndices[2];
  real_T fractions[2];
  real_T yL_1d;
  uint32_T iRght;
  uint32_T bpIdx;
  uint32_T iLeft;

  /* Column-major Lookup 2-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex[0U]]) {
    /* Binary Search */
    bpIdx = maxIndex[0U] >> 1U;
    iLeft = 0U;
    iRght = maxIndex[0U];
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex[0U] - 1U;
    frac = (u0 - bp0[maxIndex[0U] - 1U]) / (bp0[maxIndex[0U]] - bp0[maxIndex[0U]
      - 1U]);
  }

  fractions[0U] = frac;
  bpIndices[0U] = iLeft;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u1 <= bp1[0U]) {
    iLeft = 0U;
    frac = (u1 - bp1[0U]) / (bp1[1U] - bp1[0U]);
  } else if (u1 < bp1[maxIndex[1U]]) {
    /* Binary Search */
    bpIdx = maxIndex[1U] >> 1U;
    iLeft = 0U;
    iRght = maxIndex[1U];
    while (iRght - iLeft > 1U) {
      if (u1 < bp1[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u1 - bp1[iLeft]) / (bp1[iLeft + 1U] - bp1[iLeft]);
  } else {
    iLeft = maxIndex[1U] - 1U;
    frac = (u1 - bp1[maxIndex[1U] - 1U]) / (bp1[maxIndex[1U]] - bp1[maxIndex[1U]
      - 1U]);
  }

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  bpIdx = iLeft * stride + bpIndices[0U];
  yL_1d = (table[bpIdx + 1U] - table[bpIdx]) * fractions[0U] + table[bpIdx];
  bpIdx += stride;
  return (((table[bpIdx + 1U] - table[bpIdx]) * fractions[0U] + table[bpIdx]) -
          yL_1d) * frac + yL_1d;
}

real32_T look1_iflf_binlc(real32_T u0, const real32_T bp0[], const real32_T
  table[], uint32_T maxIndex)
{
  real32_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Clip'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = 0.0F;
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = 1.0F;
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

void ISR_FlexCAN_2_MB0(void)
{
  /* Call the system: <S290>/CCPReceive */
  {
    /* S-Function (ec5744_caninterruptslb1): '<S290>/ReceiveandTransmitInterrupt' */

    /* Output and update for function-call system: '<S290>/CCPReceive' */

    /* S-Function (ec5744_canreceiveslb): '<S306>/CANReceive' */

    /* Receive CAN message */
    {
      uint8 CAN2BUF0RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

      uint8 can2buf0looprx= 0;
      VehCtrlMdel240912_2018b_B.CANReceive_o3= 256;
      VehCtrlMdel240912_2018b_B.CANReceive_o5= 8;
      VehCtrlMdel240912_2018b_B.CANReceive_o2= ec_can_receive(2,0, CAN2BUF0RX);
      VehCtrlMdel240912_2018b_B.CANReceive_o4[0]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240912_2018b_B.CANReceive_o4[1]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240912_2018b_B.CANReceive_o4[2]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240912_2018b_B.CANReceive_o4[3]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240912_2018b_B.CANReceive_o4[4]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240912_2018b_B.CANReceive_o4[5]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240912_2018b_B.CANReceive_o4[6]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240912_2018b_B.CANReceive_o4[7]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
    }

    /* Nothing to do for system: <S306>/Nothing */

    /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S306>/CANReceive' */

    /* End of Outputs for S-Function (ec5744_caninterruptslb1): '<S290>/ReceiveandTransmitInterrupt' */
  }

  FLEXCAN(2).IFLAG1.B.BUF0I = 1;
                               /* Clear CAN interrupt flag by writing it to 1 */
}

/*
 * Set which subrates need to run this base step (base rate always runs).
 * This function must be called prior to calling the model step function
 * in order to "remember" which rates need to run this base step.  The
 * buffering of events allows for overlapping preemption.
 */
void VehCtrlMdel240912_2018b_SetEventsForThisBaseStep(boolean_T *eventFlags)
{
  /* Task runs when its counter is zero, computed via rtmStepTask macro */
  eventFlags[1] = ((boolean_T)rtmStepTask(VehCtrlMdel240912_2018b_M, 1));
  eventFlags[2] = ((boolean_T)rtmStepTask(VehCtrlMdel240912_2018b_M, 2));
  eventFlags[3] = ((boolean_T)rtmStepTask(VehCtrlMdel240912_2018b_M, 3));
  eventFlags[4] = ((boolean_T)rtmStepTask(VehCtrlMdel240912_2018b_M, 4));
  eventFlags[5] = ((boolean_T)rtmStepTask(VehCtrlMdel240912_2018b_M, 5));
  eventFlags[6] = ((boolean_T)rtmStepTask(VehCtrlMdel240912_2018b_M, 6));
}

/*
 *   This function updates active task flag for each subrate
 * and rate transition flags for tasks that exchange data.
 * The function assumes rate-monotonic multitasking scheduler.
 * The function must be called at model base rate so that
 * the generated code self-manages all its subrates and rate
 * transition flags.
 */
static void rate_monotonic_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[1])++;
  if ((VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[1]) > 1) {/* Sample time: [0.001s, 0.0s] */
    VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[1] = 0;
  }

  (VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[2])++;
  if ((VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[2]) > 9) {/* Sample time: [0.005s, 0.0s] */
    VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[2] = 0;
  }

  (VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[3])++;
  if ((VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[3]) > 19) {/* Sample time: [0.01s, 0.0s] */
    VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[3] = 0;
  }

  (VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[4])++;
  if ((VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[4]) > 99) {/* Sample time: [0.05s, 0.0s] */
    VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[4] = 0;
  }

  (VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[5])++;
  if ((VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[5]) > 199) {/* Sample time: [0.1s, 0.0s] */
    VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[5] = 0;
  }

  (VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[6])++;
  if ((VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[6]) > 999) {/* Sample time: [0.5s, 0.0s] */
    VehCtrlMdel240912_2018b_M->Timing.TaskCounters.TID[6] = 0;
  }
}

/*
 * Output and update for atomic system:
 *    '<S8>/Timer1'
 *    '<S8>/Timer2'
 *    '<S153>/Timer'
 *    '<S154>/Timer'
 *    '<S154>/Timer1'
 *    '<S154>/Timer2'
 *    '<S154>/Timer3'
 *    '<S208>/Timer'
 *    '<S208>/Timer1'
 *    '<S208>/Timer2'
 *    ...
 */
void VehCtrlMdel240912_20_Timer1(boolean_T rtu_Trigger, real32_T rtu_CountTime,
  real_T *rty_Exit, DW_Timer1_VehCtrlMdel240912_2_T *localDW)
{
  boolean_T sf_internal_predicateOutput;

  /* Chart: '<S8>/Timer1' */
  if (localDW->bitsForTID3.is_active_c5_VehCtrlMdel240912_ == 0U) {
    localDW->bitsForTID3.is_active_c5_VehCtrlMdel240912_ = 1U;
    localDW->bitsForTID3.is_c5_VehCtrlMdel240912_2018b = 3U;
    localDW->x += 0.01;
    *rty_Exit = 0.0;
  } else {
    switch (localDW->bitsForTID3.is_c5_VehCtrlMdel240912_2018b) {
     case VehCtrlMdel240912_IN_InterState:
      if (rtu_Trigger) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel240912_2018b = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      }
      break;

     case VehCtrlMdel240912_2018b_IN_Out:
      *rty_Exit = 1.0;
      if (!rtu_Trigger) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel240912_2018b = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      }
      break;

     default:
      /* case IN_Trigger: */
      *rty_Exit = 0.0;
      sf_internal_predicateOutput = (rtu_Trigger && (localDW->x < rtu_CountTime));
      if (sf_internal_predicateOutput) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel240912_2018b = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      } else if (localDW->x >= rtu_CountTime) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel240912_2018b = 2U;
        *rty_Exit = 1.0;
        localDW->x = 0.0;
      } else {
        sf_internal_predicateOutput = ((localDW->x < rtu_CountTime) &&
          (!rtu_Trigger));
        if (sf_internal_predicateOutput) {
          localDW->bitsForTID3.is_c5_VehCtrlMdel240912_2018b = 1U;
          localDW->x = 0.0;
        }
      }
      break;
    }
  }

  /* End of Chart: '<S8>/Timer1' */
}

/* Function for Chart: '<S90>/Chart2' */
static void VehC_enter_atomic_WaitForEngine(void)
{
  int32_T b_previousEvent;
  b_previousEvent = VehCtrlMdel240912_2018b_DW.sfEvent;
  VehCtrlMdel240912_2018b_DW.sfEvent = VehCtrlMdel2409_event_AMKCANOFF;
  if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_AMKCANenable != 0U) {
    switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKCANenable) {
     case VehCtrlMdel240912_2018b_IN_OFF:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel24091_event_AMKCANON)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKCANenable = 2U;
      }
      break;

     case VehCtrlMdel240912_2018b_IN_ON:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel2409_event_AMKCANOFF)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKCANenable = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240912_2018b_DW.sfEvent = VehCtrlMdel24091_event_AMKDCOFF;
  if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_AMKDCon != 0U) {
    switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCon) {
     case VehCtrlMdel240912_2018b_IN_OFF:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel240912_event_AMKDCON)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCon = 2U;
      }
      break;

     case VehCtrlMdel240912_2018b_IN_ON:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel24091_event_AMKDCOFF)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCon = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240912_2018b_DW.sfEvent = VehCtrlMdel_event_MCDCEnableOFF;
  if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_MCDCEnable != 0U) {
    switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MCDCEnable) {
     case VehCtrlMdel240912_2018b_IN_OFF:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel2_event_MCDCEnableON)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MCDCEnable = 2U;
      }
      break;

     case VehCtrlMdel240912_2018b_IN_ON:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel_event_MCDCEnableOFF)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MCDCEnable = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240912_2018b_DW.sfEvent = VehCtrlMdel24_event_InverterOFF;
  if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_MC_InverterOn != 0U) {
    switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MC_InverterOn) {
     case VehCtrlMdel240912_2018b_IN_OFF:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel240_event_InverterON)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MC_InverterOn = 2U;
      }
      break;

     case VehCtrlMdel240912_2018b_IN_ON:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel24_event_InverterOFF)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MC_InverterOn = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240912_2018b_DW.sfEvent = b_previousEvent;
}

/* Function for Chart: '<S90>/Chart2' */
static void VehCtrlMdel240912_2018b_VehStat(const boolean_T *ignition_d, const
  boolean_T *HVCUTOFF, const real_T *MCFL_bSystemReady, const real_T
  *MCFR_bSystemReady, const real_T *controller_ready_e, const boolean_T *Compare)
{
  boolean_T sf_internal_predicateOutput;
  int32_T b_previousEvent;
  switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat) {
   case VehCtrlMdel240912_2018_IN_Guard:
    if (VehCtrlMdel240912_2018b_DW.temporalCounter_i1 >= 25U) {
      b_previousEvent = VehCtrlMdel240912_2018b_DW.sfEvent;
      VehCtrlMdel240912_2018b_DW.sfEvent = VehCtrlMdel240912_event_EbeepON;
      if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_BeeperStat != 0U) {
        switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_BeeperStat) {
         case VehCtrlMdel240912_2018b_IN_OFF:
          if (VehCtrlMdel240912_2018b_DW.sfEvent ==
              VehCtrlMdel240912_event_EbeepON) {
            VehCtrlMdel240912_2018b_DW.bitsForTID3.is_BeeperStat = 2U;
          }
          break;

         case VehCtrlMdel240912_2018b_IN_ON:
          if (VehCtrlMdel240912_2018b_DW.sfEvent ==
              VehCtrlMdel24091_event_EbeepOFF) {
            VehCtrlMdel240912_2018b_DW.bitsForTID3.is_BeeperStat = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240912_2018b_DW.sfEvent = b_previousEvent;
      VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat = 7U;
      VehCtrlMdel240912_2018b_DW.temporalCounter_i1 = 0U;
    } else {
      sf_internal_predicateOutput = ((!*ignition_d) || (!*Compare) ||
        (!VehCtrlMdel240912_2018b_B.Compare_p));
      if (sf_internal_predicateOutput) {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat = 6U;
      } else {
        sf_internal_predicateOutput = ((!(*controller_ready_e != 0.0)) ||
          (!(*MCFL_bSystemReady != 0.0)) || (!(*MCFR_bSystemReady != 0.0)));
        if (sf_internal_predicateOutput) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat = 3U;
        }
      }
    }
    break;

   case VehCtrlMdel240912_2018b_IN_Init:
    VehCtrlMdel240912_2018b_B.errorReset = 1.0;
    if (VehCtrlMdel240912_2018b_DW.temporalCounter_i1 >= 100U) {
      VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat = 8U;
      VehC_enter_atomic_WaitForEngine();
    }
    break;

   case VehCtrlMdel240_IN_InitStateBack:
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat = 8U;
    VehC_enter_atomic_WaitForEngine();
    break;

   case VehCtrlMdel2409_IN_MCUReadyFail:
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat = 8U;
    VehC_enter_atomic_WaitForEngine();
    break;

   case VehCtrlMdel240912_2018_IN_Ready:
    sf_internal_predicateOutput = ((!(*controller_ready_e != 0.0)) ||
      (!(*MCFL_bSystemReady != 0.0)) || (!(*MCFR_bSystemReady != 0.0)));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat = 4U;
    } else {
      if (!*HVCUTOFF) {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat = 3U;
      }
    }
    break;

   case VehCtrlMdel240912_20_IN_Standby:
    sf_internal_predicateOutput = ((!(*MCFL_bSystemReady != 0.0)) ||
      (!(*MCFR_bSystemReady != 0.0)) || (!(*controller_ready_e != 0.0)));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat = 8U;
      VehC_enter_atomic_WaitForEngine();
    } else {
      sf_internal_predicateOutput = ((*ignition_d) && (*Compare) &&
        VehCtrlMdel240912_2018b_B.Compare_p && (*controller_ready_e != 0.0) && (*
        MCFL_bSystemReady != 0.0) && (*MCFR_bSystemReady != 0.0));
      if (sf_internal_predicateOutput) {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat = 1U;
        VehCtrlMdel240912_2018b_DW.temporalCounter_i1 = 0U;
      }
    }
    break;

   case VehCtrlMdel240912_2018_IN_Trans:
    if (VehCtrlMdel240912_2018b_DW.temporalCounter_i1 >= 250U) {
      b_previousEvent = VehCtrlMdel240912_2018b_DW.sfEvent;
      VehCtrlMdel240912_2018b_DW.sfEvent = VehCtrlMdel24091_event_EbeepOFF;
      if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_BeeperStat != 0U) {
        switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_BeeperStat) {
         case VehCtrlMdel240912_2018b_IN_OFF:
          if (VehCtrlMdel240912_2018b_DW.sfEvent ==
              VehCtrlMdel240912_event_EbeepON) {
            VehCtrlMdel240912_2018b_DW.bitsForTID3.is_BeeperStat = 2U;
          }
          break;

         case VehCtrlMdel240912_2018b_IN_ON:
          if (VehCtrlMdel240912_2018b_DW.sfEvent ==
              VehCtrlMdel24091_event_EbeepOFF) {
            VehCtrlMdel240912_2018b_DW.bitsForTID3.is_BeeperStat = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240912_2018b_DW.sfEvent = b_previousEvent;
      VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat = 5U;
    }
    break;

   case VehCtrlMdel240_IN_WaitForEngine:
    sf_internal_predicateOutput = ((*MCFL_bSystemReady != 0.0) &&
      (*MCFR_bSystemReady != 0.0) && (*controller_ready_e != 0.0));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat = 6U;
    }
    break;
  }
}

/* Function for Chart: '<S90>/Chart2' */
static void VehCtr_enter_atomic_MC_DCOnFail(void)
{
  int32_T b_previousEvent;
  b_previousEvent = VehCtrlMdel240912_2018b_DW.sfEvent;
  VehCtrlMdel240912_2018b_DW.sfEvent = VehCtrlMdel24091_event_AMKDCOFF;
  if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_AMKDCon != 0U) {
    switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCon) {
     case VehCtrlMdel240912_2018b_IN_OFF:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel240912_event_AMKDCON)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCon = 2U;
      }
      break;

     case VehCtrlMdel240912_2018b_IN_ON:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel24091_event_AMKDCOFF)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCon = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240912_2018b_DW.sfEvent = VehCtrlMdel_event_MCDCEnableOFF;
  if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_MCDCEnable != 0U) {
    switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MCDCEnable) {
     case VehCtrlMdel240912_2018b_IN_OFF:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel2_event_MCDCEnableON)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MCDCEnable = 2U;
      }
      break;

     case VehCtrlMdel240912_2018b_IN_ON:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel_event_MCDCEnableOFF)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MCDCEnable = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240912_2018b_DW.sfEvent = VehCtrlMdel24_event_InverterOFF;
  if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_MC_InverterOn != 0U) {
    switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MC_InverterOn) {
     case VehCtrlMdel240912_2018b_IN_OFF:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel240_event_InverterON)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MC_InverterOn = 2U;
      }
      break;

     case VehCtrlMdel240912_2018b_IN_ON:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel24_event_InverterOFF)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MC_InverterOn = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240912_2018b_DW.sfEvent = b_previousEvent;
}

/* Function for Chart: '<S90>/Chart2' */
static void VehCtrlMdel_enter_atomic_AMKCAN(void)
{
  int32_T b_previousEvent;
  b_previousEvent = VehCtrlMdel240912_2018b_DW.sfEvent;
  VehCtrlMdel240912_2018b_DW.sfEvent = VehCtrlMdel24091_event_AMKCANON;
  if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_AMKCANenable != 0U) {
    switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKCANenable) {
     case VehCtrlMdel240912_2018b_IN_OFF:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel24091_event_AMKCANON)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKCANenable = 2U;
      }
      break;

     case VehCtrlMdel240912_2018b_IN_ON:
      if (VehCtrlMdel240912_2018b_DW.sfEvent == VehCtrlMdel2409_event_AMKCANOFF)
      {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKCANenable = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240912_2018b_DW.sfEvent = b_previousEvent;
}

/* Model step function for TID0 */
void VehCtrlMdel240912_2018b_step0(void) /* Sample time: [0.0005s, 0.0s] */
{
  {                                    /* Sample time: [0.0005s, 0.0s] */
    rate_monotonic_scheduler();
  }
}

/* Model step function for TID1 */
void VehCtrlMdel240912_2018b_step1(void) /* Sample time: [0.001s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S290>/Function-Call Generator' incorporates:
   *  SubSystem: '<S290>/CCPBackground'
   */

  /* S-Function (ec5744_ccpslb): '<S305>/CCPBackground' */
  ccpBackground();
  Lin0_Background();

  /* End of Outputs for S-Function (fcncallgen): '<S290>/Function-Call Generator' */
}

/* Model step function for TID2 */
void VehCtrlMdel240912_2018b_step2(void) /* Sample time: [0.005s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S288>/5ms' incorporates:
   *  SubSystem: '<S288>/daq5ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S303>/CCPDAQ' */
  ccpDaq(0);

  /* End of Outputs for S-Function (fcncallgen): '<S288>/5ms' */
}

/* Model step function for TID3 */
void VehCtrlMdel240912_2018b_step3(void) /* Sample time: [0.01s, 0.0s] */
{
  int32_T d_previousEvent;
  real_T rtb_Gain5;
  real_T rtb_Yk1_l;
  real_T rtb_Gain4;
  real_T rtb_Switch2_on;
  real32_T rtb_Add;
  real32_T rtb_Brk_F;
  boolean_T rtb_LogicalOperator2;
  boolean_T rtb_LogicalOperator3;
  boolean_T rtb_Compare;
  real32_T rtb_APP_POS2;
  boolean_T rtb_Compare_am;
  boolean_T rtb_LowerRelop1_b;
  real_T elapseTime;
  real_T rtb_UkYk1;
  real_T rtb_Add2;
  real_T rtb_g_mpss1;
  real_T rtb_UkYk1_ix;
  real_T rtb_StrWhlAngV;
  real32_T rtb_Add4_j;
  real32_T rtb_Add7;
  real32_T rtb_Add6_p;
  real32_T rtb_Switch2_mn;
  real32_T rtb_Switch2_b0;
  real32_T rtb_CastToDouble;
  boolean_T rtb_Compare_in;
  boolean_T rtb_AND_l;
  boolean_T rtb_AND2;
  boolean_T rtb_Compare_o;
  real32_T rtb_VxIMU_est;
  real32_T rtb_Ax;
  real32_T rtb_UkYk1_ea;
  real32_T rtb_Add10;
  real_T rtb_deltafalllimit_iz;
  real_T rtb_deltafalllimit_i4;
  real32_T rtb_deltafalllimit_km;
  boolean_T rtb_UpperRelop_ir;
  real32_T rtb_MaxWhlSpd_mps_n;
  uint32_T rtb_Gain3_m;
  uint32_T rtb_Gain2;
  uint32_T FunctionCallSubsystem_ELAPS_T;
  real_T WhlSpdFL;
  real_T WhlSpdFR;
  real_T WhlSpdRR_mps;
  real_T WhlSpdRL_mps;
  real_T FLWhlStrAng;
  real32_T Acc_POS;
  int32_T Brk_F;
  boolean_T rtb_LogicalOperator_idx_0;

  /* S-Function (fcncallgen): '<S3>/10ms7' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* S-Function (ec5744_swislbu3): '<S101>/SwitchInput' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240912_2018b_B.Drive_ready= ec_gpio_read(92);

  /* Logic: '<S101>/Logical Operator' */
  ignition = !VehCtrlMdel240912_2018b_B.Drive_ready;

  /* S-Function (ec5744_swislbu3): '<S101>/SwitchInput1' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240912_2018b_B.AMKCLDCON_e= ec_gpio_read(99);

  /* S-Function (ec5744_swislbu3): '<S101>/SwitchInput2' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240912_2018b_B.HVCUTOFF_j= ec_gpio_read(45);

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms7' */

  /* S-Function (fcncallgen): '<S3>/10ms6' incorporates:
   *  SubSystem: '<S3>/EMRAXMCU_RECIEVE'
   */
  /* S-Function (ec5744_canreceiveslb): '<S98>/CANReceive1' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF1RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf1looprx= 0;
    VehCtrlMdel240912_2018b_B.CANReceive1_o3= 218089455;
    VehCtrlMdel240912_2018b_B.CANReceive1_o5= 8;
    VehCtrlMdel240912_2018b_B.CANReceive1_o2= ec_can_receive(0,1, CAN0BUF1RX);
    VehCtrlMdel240912_2018b_B.CANReceive1_o4[0]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4[1]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4[2]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4[3]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4[4]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4[5]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4[6]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4[7]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
  }

  /* Call the system: <S98>/MCU_pwr */

  /* Output and update for function-call system: '<S98>/MCU_pwr' */

  /* Outputs for Enabled SubSystem: '<S120>/MCU_VCUMeter1' incorporates:
   *  EnablePort: '<S122>/Enable'
   */
  if (VehCtrlMdel240912_2018b_B.CANReceive1_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S122>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Length = 8;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.ID =
        VehCtrlMdel240912_2018b_B.CANReceive1_o3;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Extended = 1;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4[0];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4[1];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4[2];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4[3];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4[4];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4[5];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4[6];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S122>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S122>/CAN Unpack' */
      if ((8 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Length) &&
          (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.ID != INVALID_CAN_ID) )
      {
        if ((218089455 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.ID) &&
            (1U == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Extended) ) {
          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 48
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 0.1
             *  offset                  = -1600.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[6]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[7]) <<
                      8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = (result * 0.1) + -1600.0;
                AC_current = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 32
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 0.1
             *  offset                  = -1600.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[5]) <<
                      8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = (result * 0.1) + -1600.0;
                DC_current = result;
              }
            }

            /* --------------- START Unpacking signal 2 ------------------
             *  startBit                = 0
             *  length                  = 8
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = -50.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[0]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result + -50.0;
                MCU_Temp = result;
              }
            }

            /* --------------- START Unpacking signal 3 ------------------
             *  startBit                = 8
             *  length                  = 8
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = -50.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[1]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result + -50.0;
                motor_Temp = result;
              }
            }

            /* --------------- START Unpacking signal 4 ------------------
             *  startBit                = 16
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 0.1
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_g.Data[3]) <<
                      8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.1;
                VehCtrlMdel240912_2018b_B.voltage = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S120>/MCU_VCUMeter1' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S98>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S98>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF0RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf0looprx= 0;
    VehCtrlMdel240912_2018b_B.CANReceive3_o3= 218089199;
    VehCtrlMdel240912_2018b_B.CANReceive3_o5= 8;
    VehCtrlMdel240912_2018b_B.CANReceive3_o2= ec_can_receive(0,0, CAN0BUF0RX);
    VehCtrlMdel240912_2018b_B.CANReceive3_o4[0]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4[1]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4[2]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4[3]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4[4]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4[5]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4[6]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4[7]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
  }

  /* Call the system: <S98>/MCU_state */

  /* Output and update for function-call system: '<S98>/MCU_state' */

  /* Outputs for Enabled SubSystem: '<S121>/MCU_state' incorporates:
   *  EnablePort: '<S127>/Enable'
   */
  if (VehCtrlMdel240912_2018b_B.CANReceive3_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S127>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Length = 8;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4.ID =
        VehCtrlMdel240912_2018b_B.CANReceive3_o3;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Extended = 1;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4[0];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4[1];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4[2];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4[3];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4[4];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4[5];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4[6];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S127>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S127>/CAN Unpack' */
      if ((8 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Length) &&
          (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.ID != INVALID_CAN_ID) ) {
        if ((218089199 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4.ID) && (1U
             == VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Extended) ) {
          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 45
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x20U)) >> 5);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                L12V_error = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 56
             *  length                  = 2
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[7]) &
                      (uint8_T)(0x3U));
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                alarm = result;
              }
            }

            /* --------------- START Unpacking signal 2 ------------------
             *  startBit                = 46
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x40U)) >> 6);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.low_VOL = result;
              }
            }

            /* --------------- START Unpacking signal 3 ------------------
             *  startBit                = 40
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x1U));
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                controller_ready = result;
              }
            }

            /* --------------- START Unpacking signal 4 ------------------
             *  startBit                = 48
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[6]) &
                      (uint8_T)(0x1U));
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCU_Temp_error = result;
              }
            }

            /* --------------- START Unpacking signal 5 ------------------
             *  startBit                = 32
             *  length                  = 8
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[4]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.Mode = result;
              }
            }

            /* --------------- START Unpacking signal 6 ------------------
             *  startBit                = 49
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[6]) &
                      (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.motorTemp_error = result;
              }
            }

            /* --------------- START Unpacking signal 7 ------------------
             *  startBit                = 43
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.overCurrent = result;
              }
            }

            /* --------------- START Unpacking signal 8 ------------------
             *  startBit                = 47
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x80U)) >> 7);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.overpower = result;
              }
            }

            /* --------------- START Unpacking signal 9 ------------------
             *  startBit                = 44
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x10U)) >> 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.overvol = result;
              }
            }

            /* --------------- START Unpacking signal 10 ------------------
             *  startBit                = 41
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.Precharge = result;
              }
            }

            /* --------------- START Unpacking signal 11 ------------------
             *  startBit                = 42
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.Reslove_error = result;
              }
            }

            /* --------------- START Unpacking signal 12 ------------------
             *  startBit                = 55
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[6]) &
                      (uint8_T)(0x80U)) >> 7);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                selfcheck = result;
              }
            }

            /* --------------- START Unpacking signal 13 ------------------
             *  startBit                = 0
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 0.5
             *  offset                  = -10000.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[0]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[1]) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = (result * 0.5) + -10000.0;
                RPM = result;
              }
            }

            /* --------------- START Unpacking signal 14 ------------------
             *  startBit                = 16
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4.Data[3]) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                trq = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S121>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S98>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms6' */

  /* S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S94>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF16RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf16looprx= 0;
    VehCtrlMdel240912_2018b_B.CANReceive3_o3_m= 1698;
    VehCtrlMdel240912_2018b_B.CANReceive3_o5_b= 8;
    VehCtrlMdel240912_2018b_B.CANReceive3_o2_m= ec_can_receive(0,16, CAN0BUF16RX);
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[0]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[1]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[2]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[3]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[4]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[5]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[6]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[7]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
  }

  /* Call the system: <S94>/ABS_BUS_state */

  /* Output and update for function-call system: '<S94>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S102>/IMU_state' incorporates:
   *  EnablePort: '<S103>/Enable'
   */
  if (VehCtrlMdel240912_2018b_B.CANReceive3_o2_m > 0) {
    /* S-Function (ecucoder_canunmessage): '<S103>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Length = 8;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.ID =
        VehCtrlMdel240912_2018b_B.CANReceive3_o3_m;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Extended = 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[0];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[1];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[2];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[3];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[4];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[5];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[6];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_lg[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S103>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S103>/CAN Unpack1' */
      if ((8 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Length) &&
          (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.ID != INVALID_CAN_ID) )
      {
        if ((1698 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.ID) && (0U ==
             VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Extended) ) {
          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 8
             *  length                  = 16
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = SIGNED
             *  factor                  = 0.014063
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                int16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);
                  int16_T* tempValuePtr = (int16_T*)&tempValue;

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[1]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[0]) <<
                      8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.014063;
                VehCtrlMdel240912_2018b_B.ABS_WS_FL = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 24
             *  length                  = 16
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = SIGNED
             *  factor                  = 0.014063
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                int16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);
                  int16_T* tempValuePtr = (int16_T*)&tempValue;

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[3]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[2]) <<
                      8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.014063;
                VehCtrlMdel240912_2018b_B.ABS_WS_FR = result;
              }
            }

            /* --------------- START Unpacking signal 2 ------------------
             *  startBit                = 40
             *  length                  = 16
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = SIGNED
             *  factor                  = 0.014063
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                int16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);
                  int16_T* tempValuePtr = (int16_T*)&tempValue;

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[5]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[4]) <<
                      8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.014063;
                VehCtrlMdel240912_2018b_B.ABS_WS_RL = result;
              }
            }

            /* --------------- START Unpacking signal 3 ------------------
             *  startBit                = 56
             *  length                  = 16
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = SIGNED
             *  factor                  = 0.014063
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                int16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);
                  int16_T* tempValuePtr = (int16_T*)&tempValue;

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[7]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_ja.Data[6]) <<
                      8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.014063;
                VehCtrlMdel240912_2018b_B.ABS_WS_RR = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S102>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S94>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms3' */

  /* S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S100>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF7RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf7looprx= 0;
    VehCtrlMdel240912_2018b_B.CANReceive3_o3_c= 330;
    VehCtrlMdel240912_2018b_B.CANReceive3_o5_d= 8;
    VehCtrlMdel240912_2018b_B.CANReceive3_o2_p= ec_can_receive(0,7, CAN0BUF7RX);
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[0]= CAN0BUF7RX[can0buf7looprx];
    can0buf7looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[1]= CAN0BUF7RX[can0buf7looprx];
    can0buf7looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[2]= CAN0BUF7RX[can0buf7looprx];
    can0buf7looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[3]= CAN0BUF7RX[can0buf7looprx];
    can0buf7looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[4]= CAN0BUF7RX[can0buf7looprx];
    can0buf7looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[5]= CAN0BUF7RX[can0buf7looprx];
    can0buf7looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[6]= CAN0BUF7RX[can0buf7looprx];
    can0buf7looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[7]= CAN0BUF7RX[can0buf7looprx];
    can0buf7looprx++;
  }

  /* Call the system: <S100>/StrWhSnis_state */

  /* Output and update for function-call system: '<S100>/StrWhSnis_state' */

  /* Outputs for Enabled SubSystem: '<S143>/IMU_state' incorporates:
   *  EnablePort: '<S144>/Enable'
   */
  if (VehCtrlMdel240912_2018b_B.CANReceive3_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S144>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Length = 8;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.ID =
        VehCtrlMdel240912_2018b_B.CANReceive3_o3_c;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Extended = 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[0];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[1];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[2];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[3];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[4];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[5];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[6];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_k[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S144>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S144>/CAN Unpack1' */
      if ((8 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Length) &&
          (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.ID != INVALID_CAN_ID) )
      {
        if ((330 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.ID) && (0U ==
             VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Extended) ) {
          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 0
             *  length                  = 8
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Data[0]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                StrWhlAngAliveRollCnt = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 32
             *  length                  = 16
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 0.1
             *  offset                  = -779.3
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Data[4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Data[3]) <<
                      8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = (result * 0.1) + -779.3;
                StrWhlAng = result;
              }
            }

            /* --------------- START Unpacking signal 2 ------------------
             *  startBit                = 16
             *  length                  = 8
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_l.Data[2]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                StrWhlAngV = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S143>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S100>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms4' */

  /* S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S104>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF1RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf1looprx= 0;
    VehCtrlMdel240912_2018b_B.CANReceive3_o3_e= 640;
    VehCtrlMdel240912_2018b_B.CANReceive3_o5_a= 8;
    VehCtrlMdel240912_2018b_B.CANReceive3_o2_l= ec_can_receive(1,1, CAN1BUF1RX);
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[0]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[1]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[2]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[3]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[4]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[5]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[6]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[7]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
  }

  /* Call the system: <S104>/AMKMCU_state */

  /* Output and update for function-call system: '<S104>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S106>/MCU_state' incorporates:
   *  EnablePort: '<S109>/Enable'
   */
  if (VehCtrlMdel240912_2018b_B.CANReceive3_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S109>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Length = 8;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.ID =
        VehCtrlMdel240912_2018b_B.CANReceive3_o3_e;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Extended = 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[0];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[1];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[2];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[3];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[4];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[5];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[6];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_l[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S109>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S109>/CAN Unpack' */
      if ((8 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Length) &&
          (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.ID != INVALID_CAN_ID) )
      {
        if ((640 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.ID) && (0U ==
             VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Extended) ) {
          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 32
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = SIGNED
             *  factor                  = 0.0098
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                int16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);
                  int16_T* tempValuePtr = (int16_T*)&tempValue;

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[5]) <<
                      8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.0098;
                VehCtrlMdel240912_2018b_B.MCFL_ActualTorque = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 48
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = SIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                int16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);
                  int16_T* tempValuePtr = (int16_T*)&tempValue;

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[6]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[7]) <<
                      8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFL_ActualVelocity_p = result;
              }
            }

            /* --------------- START Unpacking signal 2 ------------------
             *  startBit                = 16
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[3]) <<
                      8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFL_DCVoltage = result;
              }
            }

            /* --------------- START Unpacking signal 3 ------------------
             *  startBit                = 12
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x10U)) >> 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFL_bDCOn_l = result;
              }
            }

            /* --------------- START Unpacking signal 4 ------------------
             *  startBit                = 15
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x80U)) >> 7);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFL_bDerating = result;
              }
            }

            /* --------------- START Unpacking signal 5 ------------------
             *  startBit                = 9
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFL_bError = result;
              }
            }

            /* --------------- START Unpacking signal 6 ------------------
             *  startBit                = 14
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x40U)) >> 6);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFL_bInverterOn = result;
              }
            }

            /* --------------- START Unpacking signal 7 ------------------
             *  startBit                = 11
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFL_bQuitDCOn = result;
              }
            }

            /* --------------- START Unpacking signal 8 ------------------
             *  startBit                = 13
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x20U)) >> 5);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFL_bQuitInverterOn = result;
              }
            }

            /* --------------- START Unpacking signal 9 ------------------
             *  startBit                = 0
             *  length                  = 8
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[0]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFL_bReserve = result;
              }
            }

            /* --------------- START Unpacking signal 10 ------------------
             *  startBit                = 8
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x1U));
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFL_bSystemReady_c = result;
              }
            }

            /* --------------- START Unpacking signal 11 ------------------
             *  startBit                = 10
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFL_bWarn = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S106>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S104>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S104>/CANReceive1' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF2RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf2looprx= 0;
    VehCtrlMdel240912_2018b_B.CANReceive1_o3_n= 642;
    VehCtrlMdel240912_2018b_B.CANReceive1_o5_a= 8;
    VehCtrlMdel240912_2018b_B.CANReceive1_o2_l= ec_can_receive(1,2, CAN1BUF2RX);
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[0]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[1]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[2]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[3]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[4]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[5]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[6]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[7]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
  }

  /* Call the system: <S104>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S104>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S107>/MCU_state' incorporates:
   *  EnablePort: '<S110>/Enable'
   */
  if (VehCtrlMdel240912_2018b_B.CANReceive1_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S110>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_b.Length = 8;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_b.ID =
        VehCtrlMdel240912_2018b_B.CANReceive1_o3_n;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_b.Extended = 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[0];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[1];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[2];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[3];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[4];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[5];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[6];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_c[7];
      canunpackloop++;
    }
  }

  /* End of Outputs for SubSystem: '<S107>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S104>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S104>/CANReceive2' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF3RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf3looprx= 0;
    VehCtrlMdel240912_2018b_B.CANReceive2_o3= 644;
    VehCtrlMdel240912_2018b_B.CANReceive2_o5= 8;
    VehCtrlMdel240912_2018b_B.CANReceive2_o2= ec_can_receive(1,3, CAN1BUF3RX);
    VehCtrlMdel240912_2018b_B.CANReceive2_o4[0]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive2_o4[1]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive2_o4[2]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive2_o4[3]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive2_o4[4]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive2_o4[5]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive2_o4[6]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive2_o4[7]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
  }

  /* Call the system: <S104>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S104>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S108>/MCU_state' incorporates:
   *  EnablePort: '<S111>/Enable'
   */
  if (VehCtrlMdel240912_2018b_B.CANReceive2_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S111>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Length = 8;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.ID =
        VehCtrlMdel240912_2018b_B.CANReceive2_o3;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Extended = 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4[0];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4[1];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4[2];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4[3];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4[4];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4[5];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4[6];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S111>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S111>/CAN Unpack' */
      if ((6 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Length) &&
          (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.ID != INVALID_CAN_ID) )
      {
        if ((64 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.ID) && (0U ==
             VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Extended) ) {
          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 32
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 0.1
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Data[4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Data[5]) <<
                      8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.1;
                VehCtrlMdel240912_2018b_B.MCFL_TempIGBT = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 16
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 0.1
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Data[3]) <<
                      8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.1;
                VehCtrlMdel240912_2018b_B.MCFL_TempInverter_p = result;
              }
            }

            /* --------------- START Unpacking signal 2 ------------------
             *  startBit                = 0
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 0.1
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Data[0]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_c.Data[1]) <<
                      8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.1;
                VehCtrlMdel240912_2018b_B.MCFL_TempMotor = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S108>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S104>/CANReceive2' */

  /* S-Function (ec5744_canreceiveslb): '<S105>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF4RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf4looprx= 0;
    VehCtrlMdel240912_2018b_B.CANReceive3_o3_i= 641;
    VehCtrlMdel240912_2018b_B.CANReceive3_o5_an= 8;
    VehCtrlMdel240912_2018b_B.CANReceive3_o2_a= ec_can_receive(1,4, CAN1BUF4RX);
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[0]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[1]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[2]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[3]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[4]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[5]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[6]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[7]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
  }

  /* Call the system: <S105>/AMKMCU_state */

  /* Output and update for function-call system: '<S105>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S112>/MCU_state' incorporates:
   *  EnablePort: '<S115>/Enable'
   */
  if (VehCtrlMdel240912_2018b_B.CANReceive3_o2_a > 0) {
    /* S-Function (ecucoder_canunmessage): '<S115>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Length = 8;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.ID =
        VehCtrlMdel240912_2018b_B.CANReceive3_o3_i;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Extended = 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[0];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[1];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[2];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[3];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[4];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[5];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[6];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_g[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S115>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S115>/CAN Unpack' */
      if ((8 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Length) &&
          (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.ID != INVALID_CAN_ID) )
      {
        if ((644 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.ID) && (0U ==
             VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Extended) ) {
          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 32
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = SIGNED
             *  factor                  = 0.0098
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                int16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);
                  int16_T* tempValuePtr = (int16_T*)&tempValue;

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[5]) <<
                      8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.0098;
                VehCtrlMdel240912_2018b_B.MCFR_ActualTorque = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 16
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = SIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                int16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);
                  int16_T* tempValuePtr = (int16_T*)&tempValue;

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[3]) <<
                      8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFR_ActualVelocity_k = result;
              }
            }

            /* --------------- START Unpacking signal 2 ------------------
             *  startBit                = 48
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[6]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[7]) <<
                      8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFR_DCVoltage = result;
              }
            }

            /* --------------- START Unpacking signal 3 ------------------
             *  startBit                = 12
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x10U)) >> 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFR_bDCOn_j = result;
              }
            }

            /* --------------- START Unpacking signal 4 ------------------
             *  startBit                = 15
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x80U)) >> 7);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFR_bDerating = result;
              }
            }

            /* --------------- START Unpacking signal 5 ------------------
             *  startBit                = 9
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFR_bError = result;
              }
            }

            /* --------------- START Unpacking signal 6 ------------------
             *  startBit                = 14
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x40U)) >> 6);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFR_bInverterOn = result;
              }
            }

            /* --------------- START Unpacking signal 7 ------------------
             *  startBit                = 11
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFR_bQuitDCOn = result;
              }
            }

            /* --------------- START Unpacking signal 8 ------------------
             *  startBit                = 13
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x20U)) >> 5);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFR_bQuitInverterOn = result;
              }
            }

            /* --------------- START Unpacking signal 9 ------------------
             *  startBit                = 0
             *  length                  = 8
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[0]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFR_bReserve = result;
              }
            }

            /* --------------- START Unpacking signal 10 ------------------
             *  startBit                = 8
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x1U));
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFR_bSystemReady_j = result;
              }
            }

            /* --------------- START Unpacking signal 11 ------------------
             *  startBit                = 10
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFR_bWarn = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S112>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S105>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S105>/CANReceive1' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF5RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf5looprx= 0;
    VehCtrlMdel240912_2018b_B.CANReceive1_o3_h= 643;
    VehCtrlMdel240912_2018b_B.CANReceive1_o5_j= 8;
    VehCtrlMdel240912_2018b_B.CANReceive1_o2_o= ec_can_receive(1,5, CAN1BUF5RX);
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[0]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[1]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[2]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[3]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[4]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[5]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[6]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[7]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
  }

  /* Call the system: <S105>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S105>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S113>/MCU_state' incorporates:
   *  EnablePort: '<S116>/Enable'
   */
  if (VehCtrlMdel240912_2018b_B.CANReceive1_o2_o > 0) {
    /* S-Function (ecucoder_canunmessage): '<S116>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_p.Length = 8;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_p.ID =
        VehCtrlMdel240912_2018b_B.CANReceive1_o3_h;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_p.Extended = 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[0];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[1];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[2];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[3];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[4];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[5];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[6];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive1_o4_j[7];
      canunpackloop++;
    }
  }

  /* End of Outputs for SubSystem: '<S113>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S105>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S105>/CANReceive2' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF6RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf6looprx= 0;
    VehCtrlMdel240912_2018b_B.CANReceive2_o3_j= 387;
    VehCtrlMdel240912_2018b_B.CANReceive2_o5_e= 8;
    VehCtrlMdel240912_2018b_B.CANReceive2_o2_p= ec_can_receive(1,6, CAN1BUF6RX);
    VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[0]= CAN1BUF6RX[can1buf6looprx];
    can1buf6looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[1]= CAN1BUF6RX[can1buf6looprx];
    can1buf6looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[2]= CAN1BUF6RX[can1buf6looprx];
    can1buf6looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[3]= CAN1BUF6RX[can1buf6looprx];
    can1buf6looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[4]= CAN1BUF6RX[can1buf6looprx];
    can1buf6looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[5]= CAN1BUF6RX[can1buf6looprx];
    can1buf6looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[6]= CAN1BUF6RX[can1buf6looprx];
    can1buf6looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[7]= CAN1BUF6RX[can1buf6looprx];
    can1buf6looprx++;
  }

  /* Call the system: <S105>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S105>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S114>/MCU_state' incorporates:
   *  EnablePort: '<S117>/Enable'
   */
  if (VehCtrlMdel240912_2018b_B.CANReceive2_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S117>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Length = 8;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.ID =
        VehCtrlMdel240912_2018b_B.CANReceive2_o3_j;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Extended = 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[0];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[1];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[2];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[3];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[4];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[5];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[6];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive2_o4_k[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S117>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S117>/CAN Unpack' */
      if ((8 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Length) &&
          (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.ID != INVALID_CAN_ID) )
      {
        if ((640 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.ID) && (0U ==
             VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Extended) ) {
          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 32
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = SIGNED
             *  factor                  = 0.0098
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                int16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);
                  int16_T* tempValuePtr = (int16_T*)&tempValue;

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Data[4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Data[5]) <<
                      8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.0098;
                VehCtrlMdel240912_2018b_B.MCFR_TempIGBT = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 48
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = SIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                int16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);
                  int16_T* tempValuePtr = (int16_T*)&tempValue;

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Data[6]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Data[7]) <<
                      8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFR_TempInverter_i = result;
              }
            }

            /* --------------- START Unpacking signal 2 ------------------
             *  startBit                = 16
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_n.Data[3]) <<
                      8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240912_2018b_B.MCFR_TempMotor = result;
              }
            }

            /* --------------- START Unpacking signal 3 ------------------
             *  startBit                = 12
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            /*
             * Signal is not connected or connected to terminator.
             * No unpacking code generated.
             */

            /* --------------- START Unpacking signal 4 ------------------
             *  startBit                = 15
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            /*
             * Signal is not connected or connected to terminator.
             * No unpacking code generated.
             */

            /* --------------- START Unpacking signal 5 ------------------
             *  startBit                = 9
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            /*
             * Signal is not connected or connected to terminator.
             * No unpacking code generated.
             */

            /* --------------- START Unpacking signal 6 ------------------
             *  startBit                = 14
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            /*
             * Signal is not connected or connected to terminator.
             * No unpacking code generated.
             */

            /* --------------- START Unpacking signal 7 ------------------
             *  startBit                = 11
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            /*
             * Signal is not connected or connected to terminator.
             * No unpacking code generated.
             */

            /* --------------- START Unpacking signal 8 ------------------
             *  startBit                = 13
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            /*
             * Signal is not connected or connected to terminator.
             * No unpacking code generated.
             */

            /* --------------- START Unpacking signal 9 ------------------
             *  startBit                = 0
             *  length                  = 8
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            /*
             * Signal is not connected or connected to terminator.
             * No unpacking code generated.
             */

            /* --------------- START Unpacking signal 10 ------------------
             *  startBit                = 8
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            /*
             * Signal is not connected or connected to terminator.
             * No unpacking code generated.
             */

            /* --------------- START Unpacking signal 11 ------------------
             *  startBit                = 10
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            /*
             * Signal is not connected or connected to terminator.
             * No unpacking code generated.
             */
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S114>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S105>/CANReceive2' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms5' */

  /* S-Function (fcncallgen): '<S3>/10ms' incorporates:
   *  SubSystem: '<S3>/AccBrk_BUS'
   */
  /* S-Function (ec5744_asislbu3): '<S96>/Acc1' */

  /* Read the ADC conversion result of the analog signal */
  VehCtrlMdel240912_2018b_B.Acc1= adc_read_chan(0,11);

  /* S-Function (ec5744_asislbu3): '<S96>/Acc2' */

  /* Read the ADC conversion result of the analog signal */
  VehCtrlMdel240912_2018b_B.Acc2= adc_read_chan(1,2);

  /* S-Function (ec5744_asislbu3): '<S96>/Brk1' */

  /* Read the ADC conversion result of the analog signal */
  VehCtrlMdel240912_2018b_B.Brk1= adc_read_chan(1,0);

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms' */

  /* S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* S-Function (ec5744_canreceiveslb): '<S99>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF0RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf0looprx= 0;
    VehCtrlMdel240912_2018b_B.CANReceive3_o3_cz= 513;
    VehCtrlMdel240912_2018b_B.CANReceive3_o5_m= 8;
    VehCtrlMdel240912_2018b_B.CANReceive3_o2_ma= ec_can_receive(0,0, CAN0BUF0RX);
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[0]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[1]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[2]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[3]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[4]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[5]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[6]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[7]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
  }

  /* Call the system: <S99>/IMU_state */

  /* Output and update for function-call system: '<S99>/IMU_state' */

  /* Outputs for Enabled SubSystem: '<S134>/MCU_state' incorporates:
   *  EnablePort: '<S135>/Enable'
   */
  if (VehCtrlMdel240912_2018b_B.CANReceive3_o2_ma > 0) {
    /* S-Function (ecucoder_canunmessage): '<S135>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Length = 8;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.ID =
        VehCtrlMdel240912_2018b_B.CANReceive3_o3_cz;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Extended = 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[0];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[1];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[2];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[3];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[4];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[5];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[6];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_i[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S135>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S135>/CAN Unpack' */
      if ((8 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Length) &&
          (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.ID != INVALID_CAN_ID) )
      {
        if ((513 == VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.ID) && (0U ==
             VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Extended) ) {
          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 1
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[0]) &
                      (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                IMU_Ax_Valid = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 40
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 0.00015
             *  offset                  = -4.9152
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[5]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[6]) <<
                      8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = (result * 0.00015) + -4.9152;
                IMU_Ax_Value = result;
              }
            }

            /* --------------- START Unpacking signal 2 ------------------
             *  startBit                = 2
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[0]) &
                      (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                IMU_Ay_Valid = result;
              }
            }

            /* --------------- START Unpacking signal 3 ------------------
             *  startBit                = 24
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 0.00015
             *  offset                  = -4.9152
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[3]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[4]) <<
                      8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = (result * 0.00015) + -4.9152;
                IMU_Ay_Value = result;
              }
            }

            /* --------------- START Unpacking signal 4 ------------------
             *  startBit                = 4
             *  length                  = 4
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[0]) &
                      (uint8_T)(0xF0U)) >> 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                IMU_DTC = result;
              }
            }

            /* --------------- START Unpacking signal 5 ------------------
             *  startBit                = 3
             *  length                  = 1
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint8_T unpackedValue = 0;

                {
                  uint8_T tempValue = (uint8_T) (0);

                  {
                    tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[0]) &
                      (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                IMU_Yaw_Valid = result;
              }
            }

            /* --------------- START Unpacking signal 6 ------------------
             *  startBit                = 8
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 0.00571
             *  offset                  = -187.24
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[1]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240912_2018b_B.CANUnPackMessage4_h.Data[2]) <<
                      8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = (result * 0.00571) + -187.24;
                IMU_Yaw_Value = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S134>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S99>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms2' */

  /* S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S97>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF16RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf16looprx= 0;
    VehCtrlMdel240912_2018b_B.CANReceive3_o3_l= 1698;
    VehCtrlMdel240912_2018b_B.CANReceive3_o5_de= 8;
    VehCtrlMdel240912_2018b_B.CANReceive3_o2_k= ec_can_receive(0,16, CAN0BUF16RX);
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[0]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[1]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[2]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[3]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[4]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[5]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[6]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[7]= CAN0BUF16RX[can0buf16looprx];
    can0buf16looprx++;
  }

  /* Call the system: <S97>/ABS_BUS_state */

  /* Output and update for function-call system: '<S97>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S118>/IMU_state' incorporates:
   *  EnablePort: '<S119>/Enable'
   */
  if (VehCtrlMdel240912_2018b_B.CANReceive3_o2_k > 0) {
    /* S-Function (ecucoder_canunmessage): '<S119>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_i.Length = 8;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_i.ID =
        VehCtrlMdel240912_2018b_B.CANReceive3_o3_l;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_i.Extended = 0;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[0];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[1];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[2];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[3];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[4];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[5];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[6];
      canunpackloop++;
      VehCtrlMdel240912_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240912_2018b_B.CANReceive3_o4_p[7];
      canunpackloop++;
    }
  }

  /* End of Outputs for SubSystem: '<S118>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S97>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms1' */

  /* S-Function (fcncallgen): '<S4>/10ms' incorporates:
   *  SubSystem: '<S4>/Function-Call Subsystem'
   */
  if (VehCtrlMdel240912_2018b_DW.FunctionCallSubsystem_RESET_ELA) {
    FunctionCallSubsystem_ELAPS_T = 0U;
  } else {
    FunctionCallSubsystem_ELAPS_T = VehCtrlMdel240912_2018b_M->Timing.clockTick3
      - VehCtrlMdel240912_2018b_DW.FunctionCallSubsystem_PREV_T;
  }

  VehCtrlMdel240912_2018b_DW.FunctionCallSubsystem_PREV_T =
    VehCtrlMdel240912_2018b_M->Timing.clockTick3;
  VehCtrlMdel240912_2018b_DW.FunctionCallSubsystem_RESET_ELA = false;

  /* Gain: '<S151>/Gain' */
  rtb_Gain3_m = 45875U * VehCtrlMdel240912_2018b_B.Acc1;

  /* Gain: '<S151>/Gain1' incorporates:
   *  UnitDelay: '<S151>/Unit Delay'
   */
  rtb_Gain2 = 39322U * VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_k;

  /* Sum: '<S151>/Add2' */
  Acc_vol = (rtb_Gain2 >> 1) + rtb_Gain3_m;

  /* RelationalOperator: '<S155>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol <= 32768000U);

  /* RelationalOperator: '<S156>/Compare' */
  rtb_LogicalOperator3 = (Acc_vol >= 294912000U);

  /* Logic: '<S151>/Logical Operator' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_LogicalOperator3);

  /* Gain: '<S151>/Gain2' */
  rtb_Gain2 = 45875U * VehCtrlMdel240912_2018b_B.Acc2;

  /* UnitDelay: '<S151>/Unit Delay1' incorporates:
   *  UnitDelay: '<S151>/Unit Delay'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_k =
    VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_fm;

  /* Gain: '<S151>/Gain3' incorporates:
   *  UnitDelay: '<S151>/Unit Delay'
   */
  rtb_Gain3_m = 39322U * VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_k;

  /* Sum: '<S151>/Add3' */
  Acc_vol2 = (rtb_Gain3_m >> 1) + rtb_Gain2;

  /* RelationalOperator: '<S157>/Compare' */
  rtb_LogicalOperator3 = (Acc_vol2 <= 32768000U);

  /* RelationalOperator: '<S158>/Compare' */
  rtb_Compare = (Acc_vol2 >= 294912000U);

  /* Logic: '<S151>/Logical Operator1' */
  rtb_LogicalOperator3 = (rtb_LogicalOperator3 || rtb_Compare);

  /* Logic: '<S151>/Logical Operator2' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_LogicalOperator3);

  /* Lookup_n-D: '<S151>/1-D Lookup Table' */
  Acc_POS1 = look1_iu32n16bflftfIu32_binlc(Acc_vol,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable_bp01Data_h,
    VehCtrlMdel240912_2018b_ConstP.pooled63, 1U);

  /* DataTypeConversion: '<S151>/Data Type Conversion1' */
  rtb_Add = (real32_T)Acc_POS1 * 1.52587891E-5F;

  /* Lookup_n-D: '<S151>/1-D Lookup Table3' */
  Acc_POS2 = look1_iu32n16bflftfIu32_binlc(Acc_vol2,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable3_bp01Data,
    VehCtrlMdel240912_2018b_ConstP.pooled63, 1U);

  /* DataTypeConversion: '<S151>/Data Type Conversion4' */
  rtb_APP_POS2 = (real32_T)Acc_POS2 * 1.52587891E-5F;

  /* Sum: '<S151>/Add1' */
  rtb_Brk_F = rtb_Add - rtb_APP_POS2;

  /* Abs: '<S151>/Abs' */
  rtb_Brk_F = fabsf(rtb_Brk_F);

  /* RelationalOperator: '<S161>/Compare' incorporates:
   *  Constant: '<S161>/Constant'
   */
  rtb_Compare = (rtb_Brk_F > 10.0F);

  /* RelationalOperator: '<S159>/Compare' incorporates:
   *  Constant: '<S159>/Constant'
   */
  rtb_LogicalOperator3 = (rtb_Add > 100.0F);

  /* RelationalOperator: '<S160>/Compare' incorporates:
   *  Constant: '<S160>/Constant'
   */
  rtb_Compare_am = (rtb_APP_POS2 > 100.0F);

  /* Logic: '<S151>/Logical Operator3' */
  rtb_LogicalOperator3 = (rtb_LogicalOperator3 || rtb_Compare_am);

  /* RelationalOperator: '<S162>/Compare' incorporates:
   *  Constant: '<S162>/Constant'
   */
  rtb_Compare_am = (VehCtrlMdel240912_2018b_B.Brk1 <= 300);

  /* RelationalOperator: '<S163>/Compare' incorporates:
   *  Constant: '<S163>/Constant'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240912_2018b_B.Brk1 >= 4500);

  /* Logic: '<S151>/Logical Operator5' */
  rtb_Compare_am = (rtb_Compare_am || rtb_LowerRelop1_b);

  /* Logic: '<S151>/Logical Operator4' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator3 || rtb_Compare ||
    rtb_LogicalOperator2 || rtb_Compare_am);

  /* Chart: '<S151>/Timer' incorporates:
   *  Constant: '<S151>/Constant1'
   */
  if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_c3_VehCtrlMdel240912_ ==
      0U) {
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_c3_VehCtrlMdel240912_ = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c3_VehCtrlMdel240912_2018b = 3U;
    VehCtrlMdel240912_2018b_DW.x += 0.01;
    Trq_CUT = 0.0;
  } else {
    switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c3_VehCtrlMdel240912_2018b)
    {
     case VehCtrlMdel2409_IN_InterState_o:
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c3_VehCtrlMdel240912_2018b =
          3U;
        VehCtrlMdel240912_2018b_DW.x += 0.01;
        Trq_CUT = 0.0;
      }
      break;

     case VehCtrlMdel240912_2018_IN_Out_n:
      Trq_CUT = 1.0;
      if (!rtb_LogicalOperator2) {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c3_VehCtrlMdel240912_2018b =
          3U;
        VehCtrlMdel240912_2018b_DW.x += 0.01;
        Trq_CUT = 0.0;
      }
      break;

     default:
      /* case IN_Trigger: */
      Trq_CUT = 0.0;
      rtb_LogicalOperator3 = (rtb_LogicalOperator2 &&
        (VehCtrlMdel240912_2018b_DW.x < 0.10999999940395355));
      if (rtb_LogicalOperator3) {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c3_VehCtrlMdel240912_2018b =
          3U;
        VehCtrlMdel240912_2018b_DW.x += 0.01;
        Trq_CUT = 0.0;
      } else if (VehCtrlMdel240912_2018b_DW.x >= 0.10999999940395355) {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c3_VehCtrlMdel240912_2018b =
          2U;
        Trq_CUT = 1.0;
        VehCtrlMdel240912_2018b_DW.x = 0.0;
      } else {
        rtb_LogicalOperator3 = ((VehCtrlMdel240912_2018b_DW.x <
          0.10999999940395355) && (!rtb_LogicalOperator2));
        if (rtb_LogicalOperator3) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c3_VehCtrlMdel240912_2018b =
            1U;
          VehCtrlMdel240912_2018b_DW.x = 0.0;
        }
      }
      break;
    }
  }

  /* End of Chart: '<S151>/Timer' */

  /* UnitDelay: '<S182>/Delay Input2'
   *
   * Block description for '<S182>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE;

  /* SampleTimeMath: '<S182>/sample time'
   *
   * About '<S182>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S182>/delta rise limit' */
  rtb_Gain5 = 1200.0 * elapseTime;

  /* Sum: '<S182>/Difference Inputs1'
   *
   * Block description for '<S182>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = StrWhlAng - rtb_Yk1_l;

  /* RelationalOperator: '<S185>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain5);

  /* Switch: '<S185>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S182>/delta fall limit' */
    rtb_deltafalllimit_iz = -1200.0 * elapseTime;

    /* RelationalOperator: '<S185>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S185>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S185>/Switch' */
    rtb_Gain5 = rtb_UkYk1;
  }

  /* End of Switch: '<S185>/Switch2' */

  /* Sum: '<S182>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S182>/Delay Input2'
   *
   * Block description for '<S182>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S182>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE = rtb_Gain5 + rtb_Yk1_l;

  /* Abs: '<S153>/Abs' incorporates:
   *  UnitDelay: '<S182>/Delay Input2'
   *
   * Block description for '<S182>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain5 = fabs(VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE);

  /* RelationalOperator: '<S181>/Compare' incorporates:
   *  Constant: '<S181>/Constant'
   */
  rtb_LogicalOperator2 = (rtb_Gain5 > 120.0);

  /* Chart: '<S153>/Timer' incorporates:
   *  Constant: '<S153>/Constant5'
   */
  VehCtrlMdel240912_20_Timer1(rtb_LogicalOperator2, 0.11F,
    &VehCtrlMdel240912_2018b_B.Exit_on, &VehCtrlMdel240912_2018b_DW.sf_Timer_k);

  /* UnitDelay: '<S195>/Delay Input2'
   *
   * Block description for '<S195>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain5 = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_a;

  /* SampleTimeMath: '<S195>/sample time'
   *
   * About '<S195>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S195>/delta rise limit' */
  rtb_Yk1_l = 10.0 * elapseTime;

  /* Sum: '<S195>/Difference Inputs1'
   *
   * Block description for '<S195>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = VehCtrlMdel240912_2018b_B.ABS_WS_RL - rtb_Gain5;

  /* RelationalOperator: '<S203>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Yk1_l);

  /* Switch: '<S203>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S195>/delta fall limit' */
    rtb_Yk1_l = -10.0 * elapseTime;

    /* RelationalOperator: '<S203>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_UkYk1 < rtb_Yk1_l);

    /* Switch: '<S203>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_UkYk1 = rtb_Yk1_l;
    }

    /* End of Switch: '<S203>/Switch' */
    rtb_Yk1_l = rtb_UkYk1;
  }

  /* End of Switch: '<S203>/Switch2' */

  /* Saturate: '<S154>/Saturation' incorporates:
   *  Sum: '<S195>/Difference Inputs2'
   *  UnitDelay: '<S195>/Delay Input2'
   *
   * Block description for '<S195>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S195>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_a = rtb_Yk1_l + rtb_Gain5;
  if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_a > 30.0) {
    rtb_Gain5 = 30.0;
  } else if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_a < 0.0) {
    rtb_Gain5 = 0.0;
  } else {
    rtb_Gain5 = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_a;
  }

  /* End of Saturate: '<S154>/Saturation' */

  /* Gain: '<S154>/Gain' */
  rtb_Gain5 *= 0.27777777777777779;

  /* RelationalOperator: '<S187>/Compare' incorporates:
   *  Constant: '<S187>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain5 >= 0.0);

  /* RelationalOperator: '<S188>/Compare' incorporates:
   *  Constant: '<S188>/Constant'
   */
  rtb_Compare_am = (rtb_Gain5 < 40.0);

  /* Logic: '<S154>/OR' */
  rtb_LogicalOperator2 = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S154>/Timer' incorporates:
   *  Constant: '<S154>/Constant5'
   */
  VehCtrlMdel240912_20_Timer1(rtb_LogicalOperator2, 0.11F,
    &VehCtrlMdel240912_2018b_B.Exit_le, &VehCtrlMdel240912_2018b_DW.sf_Timer_b);

  /* UnitDelay: '<S196>/Delay Input2'
   *
   * Block description for '<S196>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_b;

  /* SampleTimeMath: '<S196>/sample time'
   *
   * About '<S196>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S196>/delta rise limit' */
  rtb_Gain4 = 10.0 * elapseTime;

  /* Sum: '<S196>/Difference Inputs1'
   *
   * Block description for '<S196>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = VehCtrlMdel240912_2018b_B.ABS_WS_RR - rtb_Yk1_l;

  /* RelationalOperator: '<S204>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain4);

  /* Switch: '<S204>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S196>/delta fall limit' */
    rtb_deltafalllimit_iz = -10.0 * elapseTime;

    /* RelationalOperator: '<S204>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S204>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S204>/Switch' */
    rtb_Gain4 = rtb_UkYk1;
  }

  /* End of Switch: '<S204>/Switch2' */

  /* Saturate: '<S154>/Saturation1' incorporates:
   *  Sum: '<S196>/Difference Inputs2'
   *  UnitDelay: '<S196>/Delay Input2'
   *
   * Block description for '<S196>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S196>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_b = rtb_Gain4 + rtb_Yk1_l;
  if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_b > 30.0) {
    rtb_Gain4 = 30.0;
  } else if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_b < 0.0) {
    rtb_Gain4 = 0.0;
  } else {
    rtb_Gain4 = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_b;
  }

  /* End of Saturate: '<S154>/Saturation1' */

  /* Gain: '<S154>/Gain3' */
  rtb_Gain4 *= 0.27777777777777779;

  /* RelationalOperator: '<S189>/Compare' incorporates:
   *  Constant: '<S189>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain4 >= 0.0);

  /* RelationalOperator: '<S190>/Compare' incorporates:
   *  Constant: '<S190>/Constant'
   */
  rtb_Compare_am = (rtb_Gain4 < 40.0);

  /* Logic: '<S154>/OR1' */
  rtb_LogicalOperator2 = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S154>/Timer1' incorporates:
   *  Constant: '<S154>/Constant1'
   */
  VehCtrlMdel240912_20_Timer1(rtb_LogicalOperator2, 0.11F,
    &VehCtrlMdel240912_2018b_B.Exit_i, &VehCtrlMdel240912_2018b_DW.sf_Timer1_n);

  /* UnitDelay: '<S197>/Delay Input2'
   *
   * Block description for '<S197>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_h;

  /* SampleTimeMath: '<S197>/sample time'
   *
   * About '<S197>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S197>/delta rise limit' */
  rtb_Switch2_on = 10.0 * elapseTime;

  /* Sum: '<S197>/Difference Inputs1'
   *
   * Block description for '<S197>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = VehCtrlMdel240912_2018b_B.MCFR_ActualVelocity_k - rtb_Yk1_l;

  /* RelationalOperator: '<S205>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Switch2_on);

  /* Switch: '<S205>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S197>/delta fall limit' */
    rtb_deltafalllimit_iz = -10.0 * elapseTime;

    /* RelationalOperator: '<S205>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S205>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S205>/Switch' */
    rtb_Switch2_on = rtb_UkYk1;
  }

  /* End of Switch: '<S205>/Switch2' */

  /* Saturate: '<S154>/Saturation2' incorporates:
   *  Sum: '<S197>/Difference Inputs2'
   *  UnitDelay: '<S197>/Delay Input2'
   *
   * Block description for '<S197>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S197>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_h = rtb_Switch2_on + rtb_Yk1_l;
  if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_h > 30.0) {
    rtb_Switch2_on = 30.0;
  } else if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_h < 0.0) {
    rtb_Switch2_on = 0.0;
  } else {
    rtb_Switch2_on = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_h;
  }

  /* End of Saturate: '<S154>/Saturation2' */

  /* Gain: '<S154>/Gain1' */
  rtb_Switch2_on *= 0.1341030088495575;

  /* RelationalOperator: '<S191>/Compare' incorporates:
   *  Constant: '<S191>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Switch2_on >= 0.0);

  /* RelationalOperator: '<S192>/Compare' incorporates:
   *  Constant: '<S192>/Constant'
   */
  rtb_Compare_am = (rtb_Switch2_on < 40.0);

  /* Logic: '<S154>/OR2' */
  rtb_LogicalOperator2 = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S154>/Timer2' incorporates:
   *  Constant: '<S154>/Constant4'
   */
  VehCtrlMdel240912_20_Timer1(rtb_LogicalOperator2, 0.11F,
    &VehCtrlMdel240912_2018b_B.Exit_o, &VehCtrlMdel240912_2018b_DW.sf_Timer2_l);

  /* UnitDelay: '<S198>/Delay Input2'
   *
   * Block description for '<S198>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_n;

  /* SampleTimeMath: '<S198>/sample time'
   *
   * About '<S198>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S198>/delta rise limit' */
  rtb_deltafalllimit_iz = 10.0 * elapseTime;

  /* Sum: '<S198>/Difference Inputs1'
   *
   * Block description for '<S198>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = VehCtrlMdel240912_2018b_B.MCFL_ActualVelocity_p - rtb_Yk1_l;

  /* RelationalOperator: '<S206>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_deltafalllimit_iz);

  /* Switch: '<S206>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S198>/delta fall limit' */
    rtb_deltafalllimit_iz = -10.0 * elapseTime;

    /* RelationalOperator: '<S206>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S206>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S206>/Switch' */
    rtb_deltafalllimit_iz = rtb_UkYk1;
  }

  /* End of Switch: '<S206>/Switch2' */

  /* Saturate: '<S154>/Saturation3' incorporates:
   *  Sum: '<S198>/Difference Inputs2'
   *  UnitDelay: '<S198>/Delay Input2'
   *
   * Block description for '<S198>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S198>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_n = rtb_deltafalllimit_iz +
    rtb_Yk1_l;
  if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_n > 30.0) {
    rtb_deltafalllimit_iz = 30.0;
  } else if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_n < 0.0) {
    rtb_deltafalllimit_iz = 0.0;
  } else {
    rtb_deltafalllimit_iz = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_n;
  }

  /* End of Saturate: '<S154>/Saturation3' */

  /* Gain: '<S154>/Gain2' */
  rtb_deltafalllimit_iz *= 0.1341030088495575;

  /* RelationalOperator: '<S193>/Compare' incorporates:
   *  Constant: '<S193>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_iz >= 0.0);

  /* RelationalOperator: '<S194>/Compare' incorporates:
   *  Constant: '<S194>/Constant'
   */
  rtb_Compare_am = (rtb_deltafalllimit_iz < 40.0);

  /* Logic: '<S154>/OR3' */
  rtb_LogicalOperator2 = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S154>/Timer3' incorporates:
   *  Constant: '<S154>/Constant8'
   */
  VehCtrlMdel240912_20_Timer1(rtb_LogicalOperator2, 0.11F,
    &VehCtrlMdel240912_2018b_B.Exit_h, &VehCtrlMdel240912_2018b_DW.sf_Timer3);

  /* SignalConversion generated from: '<S149>/Out1' */
  WhlSpdFL = rtb_deltafalllimit_iz;

  /* SignalConversion generated from: '<S149>/Out1' */
  WhlSpdFR = rtb_Switch2_on;

  /* SignalConversion generated from: '<S149>/Out1' */
  WhlSpdRR_mps = rtb_Gain4;

  /* SignalConversion generated from: '<S149>/Out1' */
  WhlSpdRL_mps = rtb_Gain5;

  /* Gain: '<S153>/Gain' incorporates:
   *  UnitDelay: '<S182>/Delay Input2'
   *
   * Block description for '<S182>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_deltafalllimit_iz = 0.7 * VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE;

  /* UnitDelay: '<S153>/Unit Delay' */
  rtb_Switch2_on = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE;

  /* Gain: '<S153>/Gain1' */
  rtb_Switch2_on *= 0.3;

  /* Sum: '<S153>/Add2' */
  rtb_deltafalllimit_iz += rtb_Switch2_on;

  /* Lookup_n-D: '<S153>/1-D Lookup Table' */
  rtb_Switch2_on = look1_binlx(rtb_deltafalllimit_iz,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable_bp01Data_o,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable_tableData_f, 23U);

  /* SignalConversion generated from: '<S149>/Out1' */
  FLWhlStrAng = rtb_Switch2_on;

  /* Lookup_n-D: '<S153>/1-D Lookup Table1' */
  rtb_Switch2_on = look1_binlx(rtb_deltafalllimit_iz,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable1_bp01Data,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable1_tableData, 23U);

  /* SignalConversion generated from: '<S149>/Out1' */
  rtb_UkYk1 = rtb_Switch2_on;

  /* SignalConversion generated from: '<S149>/Out1' */
  rtb_Yk1_l = rtb_deltafalllimit_iz;

  /* Sum: '<S151>/Add' */
  rtb_Add += rtb_APP_POS2;

  /* Product: '<S151>/Divide' incorporates:
   *  Constant: '<S151>/Constant'
   */
  rtb_Brk_F = (real32_T)(rtb_Add / 2.0);

  /* SignalConversion generated from: '<S149>/Out1' */
  Acc_POS = rtb_Brk_F;

  /* Lookup_n-D: '<S151>/1-D Lookup Table1' */
  F_BrkPrs = look1_iu16bflftfIu16_binlc(VehCtrlMdel240912_2018b_B.Brk1,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable1_bp01Data_f,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable1_tableData_m, 1U);

  /* DataTypeConversion: '<S151>/Data Type Conversion' */
  rtb_Brk_F = F_BrkPrs;

  /* SignalConversion generated from: '<S149>/Out1' */
  Brk_F = (int32_T)rtb_Brk_F;

  /* UnitDelay: '<S172>/Delay Input2'
   *
   * Block description for '<S172>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_deltafalllimit_iz = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_l;

  /* SampleTimeMath: '<S172>/sample time'
   *
   * About '<S172>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S172>/delta rise limit' incorporates:
   *  Constant: '<S171>/Constant'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* UnitDelay: '<S171>/Unit Delay' */
  rtb_Gain4 = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_g;

  /* Gain: '<S171>/Gain1' */
  rtb_Gain4 *= 0.3;

  /* Gain: '<S152>/g_mpss' incorporates:
   *  UnitDelay: '<S171>/Unit Delay'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_g = 9.8 * IMU_Ax_Value;

  /* Gain: '<S171>/Gain' incorporates:
   *  UnitDelay: '<S171>/Unit Delay'
   */
  rtb_Gain5 = 0.7 * VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_g;

  /* Sum: '<S171>/Add2' */
  rtb_Add2 = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S172>/Difference Inputs1'
   *
   * Block description for '<S172>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add2 -= rtb_deltafalllimit_iz;

  /* RelationalOperator: '<S178>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Add2 > rtb_Switch2_on);

  /* Switch: '<S178>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S172>/delta fall limit' incorporates:
     *  Constant: '<S171>/Constant1'
     */
    rtb_deltafalllimit_i4 = -5000.0 * elapseTime;

    /* RelationalOperator: '<S178>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Add2 < rtb_deltafalllimit_i4);

    /* Switch: '<S178>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Add2 = rtb_deltafalllimit_i4;
    }

    /* End of Switch: '<S178>/Switch' */
    rtb_Switch2_on = rtb_Add2;
  }

  /* End of Switch: '<S178>/Switch2' */

  /* Sum: '<S172>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S172>/Delay Input2'
   *
   * Block description for '<S172>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S172>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_l = rtb_Switch2_on +
    rtb_deltafalllimit_iz;

  /* RelationalOperator: '<S175>/LowerRelop1' incorporates:
   *  Constant: '<S171>/Constant6'
   *  UnitDelay: '<S172>/Delay Input2'
   *
   * Block description for '<S172>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_l > 1.5);

  /* Switch: '<S175>/Switch2' incorporates:
   *  Constant: '<S171>/Constant6'
   */
  if (rtb_LowerRelop1_b) {
    rtb_deltafalllimit_iz = 1.5;
  } else {
    /* RelationalOperator: '<S175>/UpperRelop' incorporates:
     *  Constant: '<S171>/Constant7'
     *  UnitDelay: '<S172>/Delay Input2'
     *
     * Block description for '<S172>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_LogicalOperator2 = (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_l <
      -1.5);

    /* Switch: '<S175>/Switch' incorporates:
     *  Constant: '<S171>/Constant7'
     *  UnitDelay: '<S172>/Delay Input2'
     *
     * Block description for '<S172>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_LogicalOperator2) {
      rtb_deltafalllimit_iz = -1.5;
    } else {
      rtb_deltafalllimit_iz = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_l;
    }

    /* End of Switch: '<S175>/Switch' */
  }

  /* End of Switch: '<S175>/Switch2' */

  /* SignalConversion generated from: '<S149>/Out1' */
  rtb_Add2 = rtb_deltafalllimit_iz;

  /* UnitDelay: '<S173>/Delay Input2'
   *
   * Block description for '<S173>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_deltafalllimit_iz = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_m;

  /* SampleTimeMath: '<S173>/sample time'
   *
   * About '<S173>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S173>/delta rise limit' incorporates:
   *  Constant: '<S171>/Constant2'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S152>/g_mpss1' */
  rtb_g_mpss1 = 9.8 * IMU_Ay_Value;

  /* Gain: '<S171>/Gain2' */
  rtb_Gain4 = 0.7 * rtb_g_mpss1;

  /* UnitDelay: '<S171>/Unit Delay1' */
  rtb_Gain5 = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE;

  /* Gain: '<S171>/Gain3' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S171>/Add1' */
  rtb_deltafalllimit_i4 = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S173>/Difference Inputs1'
   *
   * Block description for '<S173>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_i4 -= rtb_deltafalllimit_iz;

  /* RelationalOperator: '<S179>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_i4 > rtb_Switch2_on);

  /* Switch: '<S179>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S173>/delta fall limit' incorporates:
     *  Constant: '<S171>/Constant4'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S179>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_deltafalllimit_i4 < elapseTime);

    /* Switch: '<S179>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_deltafalllimit_i4 = elapseTime;
    }

    /* End of Switch: '<S179>/Switch' */
    rtb_Switch2_on = rtb_deltafalllimit_i4;
  }

  /* End of Switch: '<S179>/Switch2' */

  /* Sum: '<S173>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S173>/Delay Input2'
   *
   * Block description for '<S173>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S173>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_m = rtb_Switch2_on +
    rtb_deltafalllimit_iz;

  /* RelationalOperator: '<S176>/LowerRelop1' incorporates:
   *  Constant: '<S171>/Constant8'
   *  UnitDelay: '<S173>/Delay Input2'
   *
   * Block description for '<S173>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_m > 1.5);

  /* Switch: '<S176>/Switch2' incorporates:
   *  Constant: '<S171>/Constant8'
   */
  if (rtb_LowerRelop1_b) {
    rtb_deltafalllimit_iz = 1.5;
  } else {
    /* RelationalOperator: '<S176>/UpperRelop' incorporates:
     *  Constant: '<S171>/Constant9'
     *  UnitDelay: '<S173>/Delay Input2'
     *
     * Block description for '<S173>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_LogicalOperator2 = (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_m <
      -1.5);

    /* Switch: '<S176>/Switch' incorporates:
     *  Constant: '<S171>/Constant9'
     *  UnitDelay: '<S173>/Delay Input2'
     *
     * Block description for '<S173>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_LogicalOperator2) {
      rtb_deltafalllimit_iz = -1.5;
    } else {
      rtb_deltafalllimit_iz = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_m;
    }

    /* End of Switch: '<S176>/Switch' */
  }

  /* End of Switch: '<S176>/Switch2' */

  /* SignalConversion generated from: '<S149>/Out1' */
  rtb_deltafalllimit_i4 = rtb_deltafalllimit_iz;

  /* SampleTimeMath: '<S183>/sample time'
   *
   * About '<S183>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S183>/delta rise limit' */
  rtb_deltafalllimit_iz = 1200.0 * elapseTime;

  /* UnitDelay: '<S183>/Delay Input2'
   *
   * Block description for '<S183>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_on = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_j;

  /* Sum: '<S183>/Difference Inputs1'
   *
   * Block description for '<S183>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1_ix = StrWhlAngV - rtb_Switch2_on;

  /* RelationalOperator: '<S186>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1_ix > rtb_deltafalllimit_iz);

  /* Switch: '<S186>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S183>/delta fall limit' */
    rtb_deltafalllimit_iz = -1200.0 * elapseTime;

    /* RelationalOperator: '<S186>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_UkYk1_ix < rtb_deltafalllimit_iz);

    /* Switch: '<S186>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_UkYk1_ix = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S186>/Switch' */
    rtb_deltafalllimit_iz = rtb_UkYk1_ix;
  }

  /* End of Switch: '<S186>/Switch2' */

  /* Saturate: '<S153>/Saturation1' incorporates:
   *  Sum: '<S183>/Difference Inputs2'
   *  UnitDelay: '<S183>/Delay Input2'
   *
   * Block description for '<S183>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S183>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_j = rtb_deltafalllimit_iz +
    rtb_Switch2_on;
  if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_j > 1200.0) {
    rtb_StrWhlAngV = 1200.0;
  } else if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_j < 0.0) {
    rtb_StrWhlAngV = 0.0;
  } else {
    rtb_StrWhlAngV = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_j;
  }

  /* End of Saturate: '<S153>/Saturation1' */

  /* Gain: '<S153>/Gain2' */
  rtb_deltafalllimit_iz = 0.7 * rtb_StrWhlAngV;

  /* UnitDelay: '<S153>/Unit Delay1' */
  rtb_Switch2_on = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_a;

  /* Gain: '<S153>/Gain3' */
  rtb_Switch2_on *= 0.3;

  /* Sum: '<S153>/Add1' */
  rtb_deltafalllimit_iz += rtb_Switch2_on;

  /* SignalConversion generated from: '<S149>/Out1' */
  rtb_UkYk1_ix = rtb_deltafalllimit_iz;

  /* UnitDelay: '<S174>/Delay Input2'
   *
   * Block description for '<S174>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_deltafalllimit_iz = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_k;

  /* SampleTimeMath: '<S174>/sample time'
   *
   * About '<S174>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S174>/delta rise limit' incorporates:
   *  Constant: '<S171>/Constant3'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S171>/Gain4' */
  rtb_Gain4 = 0.7 * IMU_Yaw_Value;

  /* UnitDelay: '<S171>/Unit Delay2' */
  rtb_Gain5 = VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE;

  /* Gain: '<S171>/Gain5' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S171>/Add3' */
  rtb_Gain5 += rtb_Gain4;

  /* Sum: '<S174>/Difference Inputs1'
   *
   * Block description for '<S174>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 -= rtb_deltafalllimit_iz;

  /* RelationalOperator: '<S180>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Gain5 > rtb_Switch2_on);

  /* Switch: '<S180>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S174>/delta fall limit' incorporates:
     *  Constant: '<S171>/Constant5'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S180>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Gain5 < elapseTime);

    /* Switch: '<S180>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S180>/Switch' */
    rtb_Switch2_on = rtb_Gain5;
  }

  /* End of Switch: '<S180>/Switch2' */

  /* Saturate: '<S171>/Saturation2' incorporates:
   *  Sum: '<S174>/Difference Inputs2'
   *  UnitDelay: '<S174>/Delay Input2'
   *
   * Block description for '<S174>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S174>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_k = rtb_Switch2_on +
    rtb_deltafalllimit_iz;
  if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_k > 180.0) {
    rtb_deltafalllimit_iz = 180.0;
  } else if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_k < -180.0) {
    rtb_deltafalllimit_iz = -180.0;
  } else {
    rtb_deltafalllimit_iz = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_k;
  }

  /* End of Saturate: '<S171>/Saturation2' */

  /* SignalConversion generated from: '<S149>/Out1' */
  rtb_LogicalOperator2 = VehCtrlMdel240912_2018b_B.AMKCLDCON_e;

  /* SignalConversion generated from: '<S149>/Out1' */
  rtb_LogicalOperator3 = VehCtrlMdel240912_2018b_B.HVCUTOFF_j;

  /* Update for UnitDelay: '<S151>/Unit Delay' */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_k = VehCtrlMdel240912_2018b_B.Acc1;

  /* Update for UnitDelay: '<S151>/Unit Delay1' */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_fm =
    VehCtrlMdel240912_2018b_B.Acc2;

  /* Update for UnitDelay: '<S153>/Unit Delay' incorporates:
   *  UnitDelay: '<S182>/Delay Input2'
   *
   * Block description for '<S182>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE =
    VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE;

  /* Update for UnitDelay: '<S171>/Unit Delay1' */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE = rtb_g_mpss1;

  /* Update for UnitDelay: '<S153>/Unit Delay1' */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_a = rtb_StrWhlAngV;

  /* Update for UnitDelay: '<S171>/Unit Delay2' */
  VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE = IMU_Yaw_Value;

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms' */

  /* S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  if (VehCtrlMdel240912_2018b_DW.Subsystem_RESET_ELAPS_T) {
    FunctionCallSubsystem_ELAPS_T = 0U;
  } else {
    FunctionCallSubsystem_ELAPS_T = VehCtrlMdel240912_2018b_M->Timing.clockTick3
      - VehCtrlMdel240912_2018b_DW.Subsystem_PREV_T;
  }

  VehCtrlMdel240912_2018b_DW.Subsystem_PREV_T =
    VehCtrlMdel240912_2018b_M->Timing.clockTick3;
  VehCtrlMdel240912_2018b_DW.Subsystem_RESET_ELAPS_T = false;

  /* Gain: '<S150>/Gain5' */
  elapseTime = 10.0 * IMU_Ax_Valid;

  /* DataTypeConversion: '<S150>/Cast To Double' */
  rtb_CastToDouble = (real32_T)elapseTime;

  /* MinMax: '<S270>/Min3' incorporates:
   *  Gain: '<S224>/Gain'
   *  UnitDelay: '<S224>/Unit Delay'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_p *= 0.3F;

  /* UnitDelay: '<S228>/Delay Input2'
   *
   * Block description for '<S228>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_b0 = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_n2;

  /* SampleTimeMath: '<S228>/sample time'
   *
   * About '<S228>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S228>/delta rise limit' */
  rtb_Add4_j = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S150>/Cast To Double1' */
  rtb_Add7 = (real32_T)WhlSpdFL;

  /* DataTypeConversion: '<S150>/Cast To Double6' */
  rtb_Add6_p = (real32_T)FLWhlStrAng;

  /* Gain: '<S210>/Gain2' */
  rtb_Add6_p *= 0.0174532924F;

  /* Trigonometry: '<S210>/Asin' */
  rtb_Add6_p = cosf(rtb_Add6_p);

  /* Product: '<S210>/Product1' */
  rtb_Add7 *= rtb_Add6_p;

  /* DataTypeConversion: '<S150>/Cast To Double5' */
  rtb_Add6_p = (real32_T)rtb_deltafalllimit_iz;

  /* Gain: '<S210>/Gain4' */
  rtb_Add6_p *= 0.0174532924F;

  /* Product: '<S210>/Product3' */
  rtb_Switch2_mn = 0.6F * rtb_Add6_p;

  /* Sum: '<S210>/Add2' */
  rtb_Brk_F = rtb_Add7 - rtb_Switch2_mn;

  /* UnitDelay: '<S217>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_j;

  /* Sum: '<S217>/Add4' */
  rtb_Add7 = rtb_Brk_F - rtb_Add7;

  /* Product: '<S217>/Divide' incorporates:
   *  Constant: '<S217>/steptime'
   */
  rtb_Add = rtb_Add7 / 0.01F;

  /* RelationalOperator: '<S229>/LowerRelop1' incorporates:
   *  Constant: '<S224>/Constant1'
   */
  rtb_Compare_am = (rtb_Add > 100.0F);

  /* Switch: '<S229>/Switch2' incorporates:
   *  Constant: '<S224>/Constant1'
   */
  if (rtb_Compare_am) {
    rtb_Add = 100.0F;
  } else {
    /* RelationalOperator: '<S229>/UpperRelop' incorporates:
     *  Constant: '<S224>/Constant'
     */
    rtb_Compare = (rtb_Add < -100.0F);

    /* Switch: '<S229>/Switch' incorporates:
     *  Constant: '<S224>/Constant'
     */
    if (rtb_Compare) {
      rtb_Add = -100.0F;
    }

    /* End of Switch: '<S229>/Switch' */
  }

  /* End of Switch: '<S229>/Switch2' */

  /* Sum: '<S228>/Difference Inputs1'
   *
   * Block description for '<S228>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add -= rtb_Switch2_b0;

  /* RelationalOperator: '<S230>/LowerRelop1' */
  rtb_Compare_am = (rtb_Add > rtb_Add4_j);

  /* Switch: '<S230>/Switch2' */
  if (!rtb_Compare_am) {
    /* Product: '<S228>/delta fall limit' */
    rtb_APP_POS2 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S230>/UpperRelop' */
    rtb_Compare = (rtb_Add < rtb_APP_POS2);

    /* Switch: '<S230>/Switch' */
    if (rtb_Compare) {
      rtb_Add = rtb_APP_POS2;
    }

    /* End of Switch: '<S230>/Switch' */
    rtb_Add4_j = rtb_Add;
  }

  /* End of Switch: '<S230>/Switch2' */

  /* Sum: '<S228>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S228>/Delay Input2'
   *
   * Block description for '<S228>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S228>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_n2 = rtb_Add4_j + rtb_Switch2_b0;

  /* Gain: '<S224>/Gain1' incorporates:
   *  UnitDelay: '<S228>/Delay Input2'
   *
   * Block description for '<S228>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add7 = 0.7F * VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_n2;

  /* MinMax: '<S270>/Min3' incorporates:
   *  Abs: '<S217>/Abs'
   *  Sum: '<S217>/Add'
   *  Sum: '<S224>/Add'
   *  UnitDelay: '<S224>/Unit Delay'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_p += rtb_Add7;
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_p -= rtb_CastToDouble;
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_p = fabsf
    (VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_p);

  /* RelationalOperator: '<S220>/Compare' incorporates:
   *  Constant: '<S220>/Constant'
   *  UnitDelay: '<S224>/Unit Delay'
   */
  rtb_Compare_am = (VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_p <= 0.5F);

  /* UnitDelay: '<S225>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_pj;

  /* Gain: '<S225>/Gain' */
  rtb_Add7 *= 0.3F;

  /* UnitDelay: '<S231>/Delay Input2'
   *
   * Block description for '<S231>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add4_j = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_e;

  /* SampleTimeMath: '<S231>/sample time'
   *
   * About '<S231>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S231>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(100.0 * elapseTime);

  /* MinMax: '<S270>/Min3' incorporates:
   *  DataTypeConversion: '<S150>/Cast To Double2'
   *  UnitDelay: '<S224>/Unit Delay'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_p = (real32_T)WhlSpdFR;

  /* DataTypeConversion: '<S150>/Cast To Double7' */
  rtb_Add10 = (real32_T)rtb_UkYk1;

  /* Gain: '<S210>/Gain3' */
  rtb_Add10 *= 0.0174532924F;

  /* Trigonometry: '<S210>/Asin1' */
  rtb_Add10 = cosf(rtb_Add10);

  /* Product: '<S210>/Product2' incorporates:
   *  UnitDelay: '<S224>/Unit Delay'
   */
  rtb_Add10 *= VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_p;

  /* Sum: '<S210>/Add3' */
  rtb_Add = rtb_Switch2_mn + rtb_Add10;

  /* UnitDelay: '<S217>/Unit Delay1' */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_n;

  /* Sum: '<S217>/Add5' */
  rtb_Add10 = rtb_Add - rtb_Add10;

  /* Product: '<S217>/Divide1' incorporates:
   *  Constant: '<S217>/steptime1'
   */
  rtb_APP_POS2 = rtb_Add10 / 0.01F;

  /* RelationalOperator: '<S232>/LowerRelop1' incorporates:
   *  Constant: '<S225>/Constant1'
   */
  rtb_LowerRelop1_b = (rtb_APP_POS2 > 100.0F);

  /* Switch: '<S232>/Switch2' incorporates:
   *  Constant: '<S225>/Constant1'
   */
  if (rtb_LowerRelop1_b) {
    rtb_APP_POS2 = 100.0F;
  } else {
    /* RelationalOperator: '<S232>/UpperRelop' incorporates:
     *  Constant: '<S225>/Constant'
     */
    rtb_Compare = (rtb_APP_POS2 < -100.0F);

    /* Switch: '<S232>/Switch' incorporates:
     *  Constant: '<S225>/Constant'
     */
    if (rtb_Compare) {
      rtb_APP_POS2 = -100.0F;
    }

    /* End of Switch: '<S232>/Switch' */
  }

  /* End of Switch: '<S232>/Switch2' */

  /* Sum: '<S231>/Difference Inputs1'
   *
   * Block description for '<S231>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_APP_POS2 -= rtb_Add4_j;

  /* RelationalOperator: '<S233>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_APP_POS2 > rtb_Switch2_b0);

  /* Switch: '<S233>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S231>/delta fall limit' */
    rtb_deltafalllimit_km = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S233>/UpperRelop' */
    rtb_Compare = (rtb_APP_POS2 < rtb_deltafalllimit_km);

    /* Switch: '<S233>/Switch' */
    if (rtb_Compare) {
      rtb_APP_POS2 = rtb_deltafalllimit_km;
    }

    /* End of Switch: '<S233>/Switch' */
    rtb_Switch2_b0 = rtb_APP_POS2;
  }

  /* End of Switch: '<S233>/Switch2' */

  /* Sum: '<S231>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S231>/Delay Input2'
   *
   * Block description for '<S231>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S231>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_e = rtb_Switch2_b0 + rtb_Add4_j;

  /* Gain: '<S225>/Gain1' incorporates:
   *  UnitDelay: '<S231>/Delay Input2'
   *
   * Block description for '<S231>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = 0.7F * VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_e;

  /* Sum: '<S225>/Add' */
  rtb_Add7 += rtb_Add10;

  /* Sum: '<S217>/Add1' */
  rtb_Add7 -= rtb_CastToDouble;

  /* Abs: '<S217>/Abs1' */
  rtb_Add7 = fabsf(rtb_Add7);

  /* RelationalOperator: '<S221>/Compare' incorporates:
   *  Constant: '<S221>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Add7 <= 0.5F);

  /* UnitDelay: '<S226>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_a;

  /* Gain: '<S226>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S234>/Delay Input2'
   *
   * Block description for '<S234>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hk3;

  /* SampleTimeMath: '<S234>/sample time'
   *
   * About '<S234>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S234>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S150>/Cast To Double3' */
  rtb_Add4_j = (real32_T)WhlSpdRL_mps;

  /* Product: '<S210>/Product' */
  rtb_Add6_p *= 0.58F;

  /* Sum: '<S210>/Add' */
  rtb_APP_POS2 = rtb_Add4_j - rtb_Add6_p;

  /* UnitDelay: '<S217>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_l;

  /* Sum: '<S217>/Add6' */
  rtb_Add4_j = rtb_APP_POS2 - rtb_Add4_j;

  /* Product: '<S217>/Divide2' incorporates:
   *  Constant: '<S217>/steptime2'
   */
  rtb_deltafalllimit_km = rtb_Add4_j / 0.01F;

  /* RelationalOperator: '<S235>/LowerRelop1' incorporates:
   *  Constant: '<S226>/Constant1'
   */
  rtb_Compare_in = (rtb_deltafalllimit_km > 100.0F);

  /* Switch: '<S235>/Switch2' incorporates:
   *  Constant: '<S226>/Constant1'
   */
  if (rtb_Compare_in) {
    rtb_deltafalllimit_km = 100.0F;
  } else {
    /* RelationalOperator: '<S235>/UpperRelop' incorporates:
     *  Constant: '<S226>/Constant'
     */
    rtb_Compare = (rtb_deltafalllimit_km < -100.0F);

    /* Switch: '<S235>/Switch' incorporates:
     *  Constant: '<S226>/Constant'
     */
    if (rtb_Compare) {
      rtb_deltafalllimit_km = -100.0F;
    }

    /* End of Switch: '<S235>/Switch' */
  }

  /* End of Switch: '<S235>/Switch2' */

  /* Sum: '<S234>/Difference Inputs1'
   *
   * Block description for '<S234>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_km -= rtb_Switch2_mn;

  /* RelationalOperator: '<S236>/LowerRelop1' */
  rtb_Compare_in = (rtb_deltafalllimit_km > rtb_Add7);

  /* Switch: '<S236>/Switch2' */
  if (!rtb_Compare_in) {
    /* Product: '<S234>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S236>/UpperRelop' */
    rtb_Compare = (rtb_deltafalllimit_km < rtb_Add7);

    /* Switch: '<S236>/Switch' */
    if (rtb_Compare) {
      rtb_deltafalllimit_km = rtb_Add7;
    }

    /* End of Switch: '<S236>/Switch' */
    rtb_Add7 = rtb_deltafalllimit_km;
  }

  /* End of Switch: '<S236>/Switch2' */

  /* Sum: '<S234>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S234>/Delay Input2'
   *
   * Block description for '<S234>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S234>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hk3 = rtb_Add7 + rtb_Switch2_mn;

  /* Gain: '<S226>/Gain1' incorporates:
   *  UnitDelay: '<S234>/Delay Input2'
   *
   * Block description for '<S234>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hk3;

  /* Sum: '<S226>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S217>/Add2' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S217>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S222>/Compare' incorporates:
   *  Constant: '<S222>/Constant'
   */
  rtb_Compare_in = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S227>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_nc;

  /* Gain: '<S227>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S237>/Delay Input2'
   *
   * Block description for '<S237>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_c;

  /* SampleTimeMath: '<S237>/sample time'
   *
   * About '<S237>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S237>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S150>/Cast To Double4' */
  rtb_Add4_j = (real32_T)WhlSpdRR_mps;

  /* Sum: '<S210>/Add1' */
  rtb_deltafalllimit_km = rtb_Add6_p + rtb_Add4_j;

  /* UnitDelay: '<S217>/Unit Delay3' */
  rtb_Add6_p = VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE;

  /* Sum: '<S217>/Add7' */
  rtb_Add6_p = rtb_deltafalllimit_km - rtb_Add6_p;

  /* Product: '<S217>/Divide3' incorporates:
   *  Constant: '<S217>/steptime3'
   */
  rtb_Add6_p /= 0.01F;

  /* RelationalOperator: '<S238>/LowerRelop1' incorporates:
   *  Constant: '<S227>/Constant1'
   */
  rtb_Compare = (rtb_Add6_p > 100.0F);

  /* Switch: '<S238>/Switch2' incorporates:
   *  Constant: '<S227>/Constant1'
   */
  if (rtb_Compare) {
    rtb_Add6_p = 100.0F;
  } else {
    /* RelationalOperator: '<S238>/UpperRelop' incorporates:
     *  Constant: '<S227>/Constant'
     */
    rtb_Compare = (rtb_Add6_p < -100.0F);

    /* Switch: '<S238>/Switch' incorporates:
     *  Constant: '<S227>/Constant'
     */
    if (rtb_Compare) {
      rtb_Add6_p = -100.0F;
    }

    /* End of Switch: '<S238>/Switch' */
  }

  /* End of Switch: '<S238>/Switch2' */

  /* Sum: '<S237>/Difference Inputs1'
   *
   * Block description for '<S237>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add6_p -= rtb_Switch2_mn;

  /* RelationalOperator: '<S239>/LowerRelop1' */
  rtb_Compare = (rtb_Add6_p > rtb_Add7);

  /* Switch: '<S239>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S237>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S239>/UpperRelop' */
    rtb_Compare = (rtb_Add6_p < rtb_Add7);

    /* Switch: '<S239>/Switch' */
    if (rtb_Compare) {
      rtb_Add6_p = rtb_Add7;
    }

    /* End of Switch: '<S239>/Switch' */
    rtb_Add7 = rtb_Add6_p;
  }

  /* End of Switch: '<S239>/Switch2' */

  /* Sum: '<S237>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S237>/Delay Input2'
   *
   * Block description for '<S237>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S237>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_c = rtb_Add7 + rtb_Switch2_mn;

  /* Gain: '<S227>/Gain1' incorporates:
   *  UnitDelay: '<S237>/Delay Input2'
   *
   * Block description for '<S237>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_c;

  /* Sum: '<S227>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S217>/Add3' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S217>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S223>/Compare' incorporates:
   *  Constant: '<S223>/Constant'
   */
  rtb_Compare = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S244>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_l;

  /* Gain: '<S244>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S248>/Delay Input2'
   *
   * Block description for '<S248>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_i;

  /* SampleTimeMath: '<S248>/sample time'
   *
   * About '<S248>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S248>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S218>/Unit Delay4' */
  rtb_Add7 = VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_ik;

  /* UnitDelay: '<S218>/Unit Delay' */
  rtb_Add4_j = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_a0;

  /* Sum: '<S218>/Add4' */
  rtb_Add4_j = rtb_Brk_F - rtb_Add4_j;

  /* Product: '<S218>/Divide' incorporates:
   *  Constant: '<S218>/steptime'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S218>/Add' */
  VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_ik = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S218>/Add8' */
  rtb_Add7 = VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_ik - rtb_Add7;

  /* Product: '<S218>/Divide4' incorporates:
   *  Constant: '<S218>/steptime4'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S249>/LowerRelop1' incorporates:
   *  Constant: '<S244>/Constant1'
   */
  rtb_AND_l = (rtb_Add7 > 100.0F);

  /* Switch: '<S249>/Switch2' incorporates:
   *  Constant: '<S244>/Constant1'
   */
  if (rtb_AND_l) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S249>/UpperRelop' incorporates:
     *  Constant: '<S244>/Constant'
     */
    rtb_AND_l = (rtb_Add7 < -100.0F);

    /* Switch: '<S249>/Switch' incorporates:
     *  Constant: '<S244>/Constant'
     */
    if (rtb_AND_l) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S249>/Switch' */
  }

  /* End of Switch: '<S249>/Switch2' */

  /* Sum: '<S248>/Difference Inputs1'
   *
   * Block description for '<S248>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S250>/LowerRelop1' */
  rtb_AND_l = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S250>/Switch2' */
  if (!rtb_AND_l) {
    /* Product: '<S248>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S250>/UpperRelop' */
    rtb_AND_l = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S250>/Switch' */
    if (rtb_AND_l) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S250>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S250>/Switch2' */

  /* Sum: '<S248>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S248>/Delay Input2'
   *
   * Block description for '<S248>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S248>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_i = rtb_Add6_p + rtb_Switch2_mn;

  /* Gain: '<S244>/Gain1' incorporates:
   *  UnitDelay: '<S248>/Delay Input2'
   *
   * Block description for '<S248>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_i;

  /* Sum: '<S244>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S218>/Abs' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S240>/Compare' incorporates:
   *  Constant: '<S240>/Constant'
   */
  rtb_AND_l = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S245>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_ap;

  /* Gain: '<S245>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S251>/Delay Input2'
   *
   * Block description for '<S251>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_el;

  /* SampleTimeMath: '<S251>/sample time'
   *
   * About '<S251>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S251>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S218>/Unit Delay5' */
  rtb_Add7 = VehCtrlMdel240912_2018b_DW.UnitDelay5_DSTATE;

  /* UnitDelay: '<S218>/Unit Delay1' */
  rtb_Add4_j = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_am;

  /* Sum: '<S218>/Add5' */
  rtb_Add4_j = rtb_Add - rtb_Add4_j;

  /* Product: '<S218>/Divide1' incorporates:
   *  Constant: '<S218>/steptime1'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S218>/Add1' incorporates:
   *  UnitDelay: '<S218>/Unit Delay5'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay5_DSTATE = rtb_Add4_j - rtb_CastToDouble;

  /* Sum: '<S218>/Add10' incorporates:
   *  UnitDelay: '<S218>/Unit Delay5'
   */
  rtb_Add7 = VehCtrlMdel240912_2018b_DW.UnitDelay5_DSTATE - rtb_Add7;

  /* Product: '<S218>/Divide5' incorporates:
   *  Constant: '<S218>/steptime5'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S252>/LowerRelop1' incorporates:
   *  Constant: '<S245>/Constant1'
   */
  rtb_AND2 = (rtb_Add7 > 100.0F);

  /* Switch: '<S252>/Switch2' incorporates:
   *  Constant: '<S245>/Constant1'
   */
  if (rtb_AND2) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S252>/UpperRelop' incorporates:
     *  Constant: '<S245>/Constant'
     */
    rtb_AND2 = (rtb_Add7 < -100.0F);

    /* Switch: '<S252>/Switch' incorporates:
     *  Constant: '<S245>/Constant'
     */
    if (rtb_AND2) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S252>/Switch' */
  }

  /* End of Switch: '<S252>/Switch2' */

  /* Sum: '<S251>/Difference Inputs1'
   *
   * Block description for '<S251>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S253>/LowerRelop1' */
  rtb_AND2 = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S253>/Switch2' */
  if (!rtb_AND2) {
    /* Product: '<S251>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S253>/UpperRelop' */
    rtb_AND2 = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S253>/Switch' */
    if (rtb_AND2) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S253>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S253>/Switch2' */

  /* Sum: '<S251>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S251>/Delay Input2'
   *
   * Block description for '<S251>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S251>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_el = rtb_Add6_p + rtb_Switch2_mn;

  /* Gain: '<S245>/Gain1' incorporates:
   *  UnitDelay: '<S251>/Delay Input2'
   *
   * Block description for '<S251>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_el;

  /* Sum: '<S245>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S218>/Abs1' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S241>/Compare' incorporates:
   *  Constant: '<S241>/Constant'
   */
  rtb_AND2 = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S246>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_o;

  /* Gain: '<S246>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S254>/Delay Input2'
   *
   * Block description for '<S254>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_p;

  /* SampleTimeMath: '<S254>/sample time'
   *
   * About '<S254>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S254>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S218>/Unit Delay6' */
  rtb_Add7 = VehCtrlMdel240912_2018b_DW.UnitDelay6_DSTATE;

  /* UnitDelay: '<S218>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_c;

  /* Sum: '<S218>/Add6' */
  rtb_Add4_j = rtb_APP_POS2 - rtb_Add4_j;

  /* Product: '<S218>/Divide2' incorporates:
   *  Constant: '<S218>/steptime2'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S218>/Add2' incorporates:
   *  UnitDelay: '<S218>/Unit Delay6'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay6_DSTATE = rtb_Add4_j - rtb_CastToDouble;

  /* Sum: '<S218>/Add12' incorporates:
   *  UnitDelay: '<S218>/Unit Delay6'
   */
  rtb_Add7 = VehCtrlMdel240912_2018b_DW.UnitDelay6_DSTATE - rtb_Add7;

  /* Product: '<S218>/Divide6' incorporates:
   *  Constant: '<S218>/steptime6'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S255>/LowerRelop1' incorporates:
   *  Constant: '<S246>/Constant1'
   */
  rtb_Compare_o = (rtb_Add7 > 100.0F);

  /* Switch: '<S255>/Switch2' incorporates:
   *  Constant: '<S246>/Constant1'
   */
  if (rtb_Compare_o) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S255>/UpperRelop' incorporates:
     *  Constant: '<S246>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S255>/Switch' incorporates:
     *  Constant: '<S246>/Constant'
     */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S255>/Switch' */
  }

  /* End of Switch: '<S255>/Switch2' */

  /* Sum: '<S254>/Difference Inputs1'
   *
   * Block description for '<S254>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S256>/LowerRelop1' */
  rtb_Compare_o = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S256>/Switch2' */
  if (!rtb_Compare_o) {
    /* Product: '<S254>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S256>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S256>/Switch' */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S256>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S256>/Switch2' */

  /* Sum: '<S254>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S254>/Delay Input2'
   *
   * Block description for '<S254>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S254>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_p = rtb_Add6_p + rtb_Switch2_mn;

  /* Gain: '<S246>/Gain1' incorporates:
   *  UnitDelay: '<S254>/Delay Input2'
   *
   * Block description for '<S254>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_p;

  /* Sum: '<S246>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S218>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S242>/Compare' incorporates:
   *  Constant: '<S242>/Constant'
   */
  rtb_Compare_o = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S247>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_ah;

  /* Gain: '<S247>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S257>/Delay Input2'
   *
   * Block description for '<S257>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_mt;

  /* SampleTimeMath: '<S257>/sample time'
   *
   * About '<S257>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S257>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S218>/Unit Delay7' */
  rtb_Add7 = VehCtrlMdel240912_2018b_DW.UnitDelay7_DSTATE;

  /* UnitDelay: '<S218>/Unit Delay3' */
  rtb_Add4_j = VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_d;

  /* Sum: '<S218>/Add7' */
  rtb_Add4_j = rtb_deltafalllimit_km - rtb_Add4_j;

  /* Product: '<S218>/Divide3' incorporates:
   *  Constant: '<S218>/steptime3'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S218>/Add3' incorporates:
   *  UnitDelay: '<S218>/Unit Delay7'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay7_DSTATE = rtb_Add4_j - rtb_CastToDouble;

  /* Sum: '<S218>/Add14' incorporates:
   *  UnitDelay: '<S218>/Unit Delay7'
   */
  rtb_Add7 = VehCtrlMdel240912_2018b_DW.UnitDelay7_DSTATE - rtb_Add7;

  /* Product: '<S218>/Divide7' incorporates:
   *  Constant: '<S218>/steptime7'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S258>/LowerRelop1' incorporates:
   *  Constant: '<S247>/Constant1'
   */
  rtb_UpperRelop_ir = (rtb_Add7 > 100.0F);

  /* Switch: '<S258>/Switch2' incorporates:
   *  Constant: '<S247>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S258>/UpperRelop' incorporates:
     *  Constant: '<S247>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S258>/Switch' incorporates:
     *  Constant: '<S247>/Constant'
     */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S258>/Switch' */
  }

  /* End of Switch: '<S258>/Switch2' */

  /* Sum: '<S257>/Difference Inputs1'
   *
   * Block description for '<S257>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S259>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S259>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S257>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S259>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S259>/Switch' */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S259>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S259>/Switch2' */

  /* Sum: '<S257>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S257>/Delay Input2'
   *
   * Block description for '<S257>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S257>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_mt = rtb_Add6_p + rtb_Switch2_mn;

  /* Gain: '<S247>/Gain1' incorporates:
   *  UnitDelay: '<S257>/Delay Input2'
   *
   * Block description for '<S257>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_mt;

  /* Sum: '<S247>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S218>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S243>/Compare' incorporates:
   *  Constant: '<S243>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.8F);

  /* Logic: '<S208>/Logical Operator' */
  rtb_LogicalOperator_idx_0 = (rtb_Compare_am || rtb_AND_l);
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b || rtb_AND2);
  rtb_Compare_in = (rtb_Compare_in || rtb_Compare_o);
  rtb_Compare_am = (rtb_Compare || rtb_UpperRelop_ir);

  /* UnitDelay: '<S150>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_lh;

  /* Sum: '<S219>/Add' */
  rtb_Switch2_mn = rtb_Brk_F - rtb_Add10;

  /* Abs: '<S219>/Abs' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S260>/Compare' incorporates:
   *  Constant: '<S260>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S219>/AND3' */
  rtb_Compare_o = (rtb_UpperRelop_ir && (VehCtrlMdel240912_2018b_B.Exit_h != 0.0));

  /* Sum: '<S219>/Add1' */
  rtb_Switch2_mn = rtb_Add - rtb_Add10;

  /* Abs: '<S219>/Abs1' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S261>/Compare' incorporates:
   *  Constant: '<S261>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S219>/AND2' */
  rtb_AND2 = (rtb_UpperRelop_ir && (VehCtrlMdel240912_2018b_B.Exit_o != 0.0));

  /* Sum: '<S219>/Add2' */
  rtb_Switch2_mn = rtb_APP_POS2 - rtb_Add10;

  /* Abs: '<S219>/Abs2' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S262>/Compare' incorporates:
   *  Constant: '<S262>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S219>/AND' */
  rtb_AND_l = (rtb_UpperRelop_ir && (VehCtrlMdel240912_2018b_B.Exit_le != 0.0));

  /* Sum: '<S219>/Add3' */
  rtb_Add10 = rtb_deltafalllimit_km - rtb_Add10;

  /* Abs: '<S219>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S263>/Compare' incorporates:
   *  Constant: '<S263>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 2.0F);

  /* Logic: '<S219>/AND1' */
  rtb_Compare = (rtb_UpperRelop_ir && (VehCtrlMdel240912_2018b_B.Exit_i != 0.0));

  /* Logic: '<S208>/Logical Operator1' */
  rtb_UpperRelop_ir = (rtb_Compare_o && rtb_LogicalOperator_idx_0);
  rtb_LowerRelop1_b = (rtb_AND2 && rtb_LowerRelop1_b);
  rtb_Compare_in = (rtb_AND_l && rtb_Compare_in);
  rtb_Compare = (rtb_Compare && rtb_Compare_am);

  /* Chart: '<S208>/Timer' incorporates:
   *  Constant: '<S208>/Constant1'
   */
  VehCtrlMdel240912_20_Timer1(rtb_UpperRelop_ir, 0.11F,
    &VehCtrlMdel240912_2018b_B.Exit_c, &VehCtrlMdel240912_2018b_DW.sf_Timer_o);

  /* Chart: '<S208>/Timer1' incorporates:
   *  Constant: '<S208>/Constant2'
   */
  VehCtrlMdel240912_20_Timer1(rtb_LowerRelop1_b, 0.11F,
    &VehCtrlMdel240912_2018b_B.Exit_lh4, &VehCtrlMdel240912_2018b_DW.sf_Timer1_m);

  /* Chart: '<S208>/Timer2' incorporates:
   *  Constant: '<S208>/Constant3'
   */
  VehCtrlMdel240912_20_Timer1(rtb_Compare_in, 0.11F,
    &VehCtrlMdel240912_2018b_B.Exit_lh, &VehCtrlMdel240912_2018b_DW.sf_Timer2_g);

  /* Chart: '<S208>/Timer3' incorporates:
   *  Constant: '<S208>/Constant4'
   */
  VehCtrlMdel240912_20_Timer1(rtb_Compare, 0.11F,
    &VehCtrlMdel240912_2018b_B.Exit_a, &VehCtrlMdel240912_2018b_DW.sf_Timer3_i);

  /* Logic: '<S207>/Logical Operator' */
  rtb_UpperRelop_ir = ((VehCtrlMdel240912_2018b_B.Exit_c != 0.0) ||
                       (VehCtrlMdel240912_2018b_B.Exit_lh4 != 0.0) ||
                       (VehCtrlMdel240912_2018b_B.Exit_lh != 0.0) ||
                       (VehCtrlMdel240912_2018b_B.Exit_a != 0.0));

  /* Logic: '<S207>/Logical Operator1' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* UnitDelay: '<S207>/Unit Delay4' */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_m;

  /* Sum: '<S207>/Add1' */
  rtb_Add10 = Acc_POS - rtb_Add10;

  /* RelationalOperator: '<S211>/Compare' incorporates:
   *  Constant: '<S211>/Constant'
   */
  rtb_Compare_o = (rtb_Add10 > 0.1F);

  /* Logic: '<S207>/Logical Operator2' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir || rtb_Compare_o);

  /* Logic: '<S207>/AND' */
  rtb_Compare = ((IMU_Ax_Valid != 0.0) && rtb_UpperRelop_ir);

  /* UnitDelay: '<S207>/Unit Delay3' */
  rtb_UpperRelop_ir = VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_f;

  /* Logic: '<S207>/Logical Operator3' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S207>/Switch3' incorporates:
   *  UnitDelay: '<S207>/Unit Delay1'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S207>/Switch4' */
    if (!rtb_Compare) {
      /* Saturate: '<S20>/Saturation' incorporates:
       *  Constant: '<S207>/InitZORE'
       */
      VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k = 0.0F;
    }

    /* End of Switch: '<S207>/Switch4' */
    VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_o =
      VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Switch: '<S207>/Switch3' */

  /* UnitDelay: '<S209>/Unit Delay3' */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_p;

  /* Sum: '<S209>/Add5' incorporates:
   *  UnitDelay: '<S209>/Unit Delay1'
   */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_d - rtb_Add10;

  /* Product: '<S209>/Divide3' incorporates:
   *  Constant: '<S209>/steptime3'
   */
  rtb_Add10 /= 0.01F;

  /* UnitDelay: '<S209>/Unit Delay2' */
  rtb_Switch2_mn = VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_f;

  /* Sum: '<S209>/Add9' */
  rtb_Switch2_mn -= rtb_Add10;

  /* UnitDelay: '<S209>/Unit Delay4' */
  rtb_Add6_p = VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_mn;

  /* Sum: '<S209>/Add6' incorporates:
   *  Constant: '<S209>/steptime4'
   */
  rtb_Add6_p += 0.1F;

  /* Sum: '<S209>/Add8' incorporates:
   *  Constant: '<S209>/steptime6'
   */
  rtb_Add7 = rtb_Add6_p + 2.0F;

  /* Product: '<S209>/Divide5' */
  rtb_Add7 = 1.0F / rtb_Add7 * rtb_Add6_p;

  /* Logic: '<S209>/Logical Operator' */
  rtb_Compare_am = ((VehCtrlMdel240912_2018b_B.Exit_c != 0.0) ||
                    (VehCtrlMdel240912_2018b_B.Exit_lh4 != 0.0) ||
                    (VehCtrlMdel240912_2018b_B.Exit_lh != 0.0) ||
                    (VehCtrlMdel240912_2018b_B.Exit_a != 0.0));

  /* Switch: '<S209>/Switch13' incorporates:
   *  Constant: '<S209>/Constant10'
   */
  if (rtb_Compare_am) {
    rtb_Add4_j = rtb_Add7;
  } else {
    rtb_Add4_j = 1.0F;
  }

  /* End of Switch: '<S209>/Switch13' */

  /* Product: '<S209>/Divide6' */
  rtb_Switch2_mn *= rtb_Add4_j;

  /* Sum: '<S209>/Add10' */
  rtb_Ax = rtb_Switch2_mn + rtb_Add10;

  /* Switch: '<S207>/Switch1' */
  if (rtb_Compare) {
    /* Saturate: '<S207>/Saturation1' */
    if (VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_d > 200.0F) {
      VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_d = 200.0F;
    } else {
      if (VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_d < -10.0F) {
        VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_d = -10.0F;
      }
    }

    /* Product: '<S207>/Product' incorporates:
     *  Constant: '<S207>/steptime1'
     */
    rtb_Switch2_mn = rtb_Ax * 0.01F;

    /* Saturate: '<S207>/Saturation1' incorporates:
     *  Sum: '<S207>/Add'
     */
    VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_d += rtb_Switch2_mn;
  } else {
    /* Saturate: '<S207>/Saturation1' incorporates:
     *  Constant: '<S207>/Constant'
     *  UnitDelay: '<S207>/Unit Delay'
     */
    VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_d = 0.0F;
  }

  /* End of Switch: '<S207>/Switch1' */

  /* Saturate: '<S207>/Saturation' */
  if (VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_d > 200.0F) {
    rtb_Add10 = 200.0F;
  } else if (VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_d < -10.0F) {
    rtb_Add10 = -10.0F;
  } else {
    rtb_Add10 = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_d;
  }

  /* End of Saturate: '<S207>/Saturation' */

  /* Sum: '<S207>/Add3' incorporates:
   *  UnitDelay: '<S207>/Unit Delay1'
   */
  rtb_VxIMU_est = rtb_Add10 + VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_o;

  /* MinMax: '<S208>/Min1' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Brk_F, rtb_Add);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_APP_POS2);
  rtb_Add10 = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_km);

  /* Sum: '<S207>/Add2' */
  rtb_Add10 -= rtb_VxIMU_est;

  /* RelationalOperator: '<S212>/Compare' incorporates:
   *  Constant: '<S212>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.0F);

  /* Switch: '<S207>/Switch6' incorporates:
   *  Constant: '<S207>/Reset'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S207>/Add10' incorporates:
     *  Constant: '<S207>/Steptime'
     */
    rtb_Add10 = VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_i + 0.01F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S207>/Switch6' */

  /* MinMax: '<S207>/Min' incorporates:
   *  Constant: '<S207>/ResetDelay'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_i = fminf(rtb_Add10, 0.1F);

  /* RelationalOperator: '<S207>/Relational Operator9' incorporates:
   *  Constant: '<S207>/ResetDelay'
   *  UnitDelay: '<S207>/Unit Delay2'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_i >= 0.1F);

  /* RelationalOperator: '<S265>/Compare' incorporates:
   *  Constant: '<S265>/Constant'
   */
  rtb_Compare_in = (rtb_Ax < -0.5F);

  /* Chart: '<S209>/Timer2' incorporates:
   *  Constant: '<S209>/Constant15'
   */
  VehCtrlMdel240912_20_Timer1(rtb_Compare_in, 0.11F,
    &VehCtrlMdel240912_2018b_B.Exit, &VehCtrlMdel240912_2018b_DW.sf_Timer2_j);

  /* UnitDelay: '<S269>/Delay Input2'
   *
   * Block description for '<S269>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_g;

  /* SampleTimeMath: '<S269>/sample time'
   *
   * About '<S269>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S269>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(10.0 * elapseTime);

  /* Sum: '<S270>/Add3' */
  rtb_Add4_j = ((rtb_deltafalllimit_km + rtb_APP_POS2) + rtb_Add) + rtb_Brk_F;

  /* MinMax: '<S270>/Min4' */
  rtb_Switch2_b0 = fminf(rtb_Brk_F, rtb_Add);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_APP_POS2);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_km);

  /* MinMax: '<S270>/Min3' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Brk_F, rtb_Add);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_APP_POS2);
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_p = fmaxf(rtb_MaxWhlSpd_mps_n,
    rtb_deltafalllimit_km);

  /* Sum: '<S270>/Add4' incorporates:
   *  UnitDelay: '<S224>/Unit Delay'
   */
  rtb_Add4_j = (rtb_Add4_j - rtb_Switch2_b0) -
    VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_p;

  /* Gain: '<S270>/Gain1' */
  rtb_Add4_j *= 0.5F;

  /* Sum: '<S269>/Difference Inputs1'
   *
   * Block description for '<S269>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add4_j -= rtb_Add10;

  /* RelationalOperator: '<S278>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add4_j > rtb_Switch2_mn);

  /* Switch: '<S278>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S269>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-10.0 * elapseTime);

    /* RelationalOperator: '<S278>/UpperRelop' */
    rtb_Compare_in = (rtb_Add4_j < rtb_Switch2_mn);

    /* Switch: '<S278>/Switch' */
    if (rtb_Compare_in) {
      rtb_Add4_j = rtb_Switch2_mn;
    }

    /* End of Switch: '<S278>/Switch' */
    rtb_Switch2_mn = rtb_Add4_j;
  }

  /* End of Switch: '<S278>/Switch2' */

  /* Sum: '<S269>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S269>/Delay Input2'
   *
   * Block description for '<S269>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S269>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_g = rtb_Switch2_mn + rtb_Add10;

  /* RelationalOperator: '<S264>/Compare' incorporates:
   *  Constant: '<S264>/Constant'
   */
  rtb_Compare_in = (rtb_Ax > 0.5F);

  /* Chart: '<S209>/Timer1' incorporates:
   *  Constant: '<S209>/Constant14'
   */
  VehCtrlMdel240912_20_Timer1(rtb_Compare_in, 0.11F,
    &VehCtrlMdel240912_2018b_B.Exit_l, &VehCtrlMdel240912_2018b_DW.sf_Timer1_p);

  /* Logic: '<S209>/Logical Operator2' */
  rtb_UpperRelop_ir = !(VehCtrlMdel240912_2018b_B.Exit_l != 0.0);

  /* Switch: '<S209>/Switch6' incorporates:
   *  Switch: '<S209>/Switch4'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S209>/Switch5' incorporates:
     *  UnitDelay: '<S269>/Delay Input2'
     *
     * Block description for '<S269>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (VehCtrlMdel240912_2018b_B.Exit != 0.0) {
      /* Switch: '<S209>/Switch11' incorporates:
       *  Constant: '<S209>/Constant7'
       */
      if (VehCtrlMdel240912_2018b_B.Exit_a != 0.0) {
        rtb_Switch2_mn = rtb_deltafalllimit_km;
      } else {
        rtb_Switch2_mn = 0.0F;
      }

      /* End of Switch: '<S209>/Switch11' */

      /* Switch: '<S209>/Switch10' incorporates:
       *  Constant: '<S209>/Constant6'
       */
      if (VehCtrlMdel240912_2018b_B.Exit_lh != 0.0) {
        rtb_Add10 = rtb_APP_POS2;
      } else {
        rtb_Add10 = 0.0F;
      }

      /* End of Switch: '<S209>/Switch10' */

      /* Switch: '<S209>/Switch9' incorporates:
       *  Constant: '<S209>/Constant5'
       */
      if (VehCtrlMdel240912_2018b_B.Exit_lh4 != 0.0) {
        rtb_Add4_j = rtb_Add;
      } else {
        rtb_Add4_j = 0.0F;
      }

      /* End of Switch: '<S209>/Switch9' */

      /* Switch: '<S209>/Switch8' incorporates:
       *  Constant: '<S209>/Constant4'
       */
      if (VehCtrlMdel240912_2018b_B.Exit_c != 0.0) {
        rtb_Switch2_b0 = rtb_Brk_F;
      } else {
        rtb_Switch2_b0 = 0.0F;
      }

      /* End of Switch: '<S209>/Switch8' */

      /* MinMax: '<S209>/Min1' */
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Switch2_b0, rtb_Add4_j);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Add10);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Switch2_mn);
    } else {
      rtb_MaxWhlSpd_mps_n = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_g;
    }

    /* End of Switch: '<S209>/Switch5' */
  } else {
    if (VehCtrlMdel240912_2018b_B.Exit_a != 0.0) {
      /* Switch: '<S209>/Switch4' */
      rtb_Switch2_mn = rtb_deltafalllimit_km;
    } else {
      /* Switch: '<S209>/Switch4' incorporates:
       *  Constant: '<S209>/Constant3'
       */
      rtb_Switch2_mn = 9999.0F;
    }

    /* Switch: '<S209>/Switch3' incorporates:
     *  Constant: '<S209>/Constant2'
     */
    if (VehCtrlMdel240912_2018b_B.Exit_lh != 0.0) {
      rtb_Add10 = rtb_APP_POS2;
    } else {
      rtb_Add10 = 9999.0F;
    }

    /* End of Switch: '<S209>/Switch3' */

    /* Switch: '<S209>/Switch2' incorporates:
     *  Constant: '<S209>/Constant1'
     */
    if (VehCtrlMdel240912_2018b_B.Exit_lh4 != 0.0) {
      rtb_Add4_j = rtb_Add;
    } else {
      rtb_Add4_j = 9999.0F;
    }

    /* End of Switch: '<S209>/Switch2' */

    /* Switch: '<S209>/Switch1' incorporates:
     *  Constant: '<S209>/Constant'
     */
    if (VehCtrlMdel240912_2018b_B.Exit_c != 0.0) {
      rtb_Switch2_b0 = rtb_Brk_F;
    } else {
      rtb_Switch2_b0 = 9999.0F;
    }

    /* End of Switch: '<S209>/Switch1' */

    /* MinMax: '<S209>/Min2' */
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add4_j);
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add10);
    rtb_MaxWhlSpd_mps_n = fminf(rtb_Switch2_b0, rtb_Switch2_mn);
  }

  /* End of Switch: '<S209>/Switch6' */

  /* Logic: '<S209>/NOT3' */
  rtb_UpperRelop_ir = !rtb_Compare_am;

  /* Logic: '<S209>/Logical Operator3' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir && rtb_LowerRelop1_b);

  /* Logic: '<S209>/NOT4' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S209>/Switch7' incorporates:
   *  UnitDelay: '<S269>/Delay Input2'
   *
   * Block description for '<S269>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (rtb_UpperRelop_ir) {
    /* Logic: '<S209>/Logical Operator1' */
    rtb_LowerRelop1_b = (rtb_LowerRelop1_b || rtb_Compare_am);

    /* Switch: '<S209>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_VxIMU_est = rtb_MaxWhlSpd_mps_n;
    }

    /* End of Switch: '<S209>/Switch' */
  } else {
    rtb_VxIMU_est = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_g;
  }

  /* End of Switch: '<S209>/Switch7' */

  /* UnitDelay: '<S267>/Delay Input2'
   *
   * Block description for '<S267>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_af;

  /* Sum: '<S267>/Difference Inputs1'
   *
   * Block description for '<S267>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_VxIMU_est -= rtb_Add10;

  /* Switch: '<S209>/Switch12' incorporates:
   *  Constant: '<S209>/Constant8'
   *  Constant: '<S209>/Constant9'
   */
  if (rtb_Compare_am) {
    rtb_Switch2_mn = 0.1F;
  } else {
    rtb_Switch2_mn = 0.05F;
  }

  /* End of Switch: '<S209>/Switch12' */

  /* Sum: '<S209>/Add4' */
  rtb_Add4_j = rtb_Ax + rtb_Switch2_mn;

  /* SampleTimeMath: '<S267>/sample time'
   *
   * About '<S267>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S267>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S276>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_VxIMU_est > rtb_Switch2_b0);

  /* Sum: '<S209>/Add3' */
  rtb_Ax -= rtb_Switch2_mn;

  /* Switch: '<S276>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S267>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S276>/UpperRelop' */
    rtb_Compare_am = (rtb_VxIMU_est < rtb_Switch2_mn);

    /* Switch: '<S276>/Switch' */
    if (rtb_Compare_am) {
      rtb_VxIMU_est = rtb_Switch2_mn;
    }

    /* End of Switch: '<S276>/Switch' */
    rtb_Switch2_b0 = rtb_VxIMU_est;
  }

  /* End of Switch: '<S276>/Switch2' */

  /* Sum: '<S267>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S267>/Delay Input2'
   *
   * Block description for '<S267>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S267>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_af = rtb_Switch2_b0 + rtb_Add10;

  /* RelationalOperator: '<S274>/LowerRelop1' incorporates:
   *  Constant: '<S266>/Constant1'
   *  UnitDelay: '<S267>/Delay Input2'
   *
   * Block description for '<S267>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_af > 100.0F);

  /* Switch: '<S274>/Switch2' incorporates:
   *  Constant: '<S266>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_mn = 100.0F;
  } else {
    /* RelationalOperator: '<S274>/UpperRelop' incorporates:
     *  Constant: '<S266>/Constant'
     *  UnitDelay: '<S267>/Delay Input2'
     *
     * Block description for '<S267>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_Compare_am = (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_af < 0.0F);

    /* Switch: '<S274>/Switch' incorporates:
     *  Constant: '<S266>/Constant'
     *  UnitDelay: '<S267>/Delay Input2'
     *
     * Block description for '<S267>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_Compare_am) {
      rtb_Switch2_mn = 0.0F;
    } else {
      rtb_Switch2_mn = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_af;
    }

    /* End of Switch: '<S274>/Switch' */
  }

  /* End of Switch: '<S274>/Switch2' */

  /* UnitDelay: '<S273>/Delay Input2'
   *
   * Block description for '<S273>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_f;

  /* Sum: '<S273>/Difference Inputs1'
   *
   * Block description for '<S273>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_Switch2_mn - rtb_Add10;

  /* SampleTimeMath: '<S273>/sample time'
   *
   * About '<S273>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S273>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(15.0 * elapseTime);

  /* RelationalOperator: '<S275>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S275>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S273>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-15.0 * elapseTime);

    /* RelationalOperator: '<S275>/UpperRelop' */
    rtb_Compare_am = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S275>/Switch' */
    if (rtb_Compare_am) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S275>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S275>/Switch2' */

  /* Sum: '<S273>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S273>/Delay Input2'
   *
   * Block description for '<S273>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S273>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_f = rtb_Switch2_mn + rtb_Add10;

  /* UnitDelay: '<S266>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_ncs;

  /* Gain: '<S266>/Gain' */
  rtb_Add10 *= 0.0F;

  /* Saturate: '<S20>/Saturation' incorporates:
   *  Sum: '<S266>/Add'
   *  UnitDelay: '<S150>/Unit Delay1'
   *  UnitDelay: '<S273>/Delay Input2'
   *
   * Block description for '<S273>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k = rtb_Add10 +
    VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_f;

  /* SampleTimeMath: '<S268>/sample time'
   *
   * About '<S268>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* UnitDelay: '<S268>/Delay Input2'
   *
   * Block description for '<S268>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hu;

  /* Sum: '<S268>/Difference Inputs1'
   *
   * Block description for '<S268>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_MaxWhlSpd_mps_n - rtb_Add10;

  /* Product: '<S268>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S277>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S277>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S268>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S277>/UpperRelop' */
    rtb_Compare_am = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S277>/Switch' */
    if (rtb_Compare_am) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S277>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S277>/Switch2' */

  /* Sum: '<S268>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S268>/Delay Input2'
   *
   * Block description for '<S268>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S268>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hu = rtb_Switch2_mn + rtb_Add10;

  /* Sum: '<S209>/Add7' incorporates:
   *  Constant: '<S209>/steptime5'
   */
  rtb_Add7 = 1.0F - rtb_Add7;

  /* Product: '<S209>/Divide4' incorporates:
   *  UnitDelay: '<S209>/Unit Delay4'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_mn = rtb_Add7 * rtb_Add6_p;

  /* Update for MinMax: '<S270>/Min3' incorporates:
   *  UnitDelay: '<S224>/Unit Delay'
   *  UnitDelay: '<S228>/Delay Input2'
   *
   * Block description for '<S228>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_p =
    VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_n2;

  /* Update for UnitDelay: '<S217>/Unit Delay' */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_j = rtb_Brk_F;

  /* Update for UnitDelay: '<S225>/Unit Delay' incorporates:
   *  UnitDelay: '<S231>/Delay Input2'
   *
   * Block description for '<S231>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_pj =
    VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_e;

  /* Update for UnitDelay: '<S217>/Unit Delay1' */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_n = rtb_Add;

  /* Update for UnitDelay: '<S226>/Unit Delay' incorporates:
   *  UnitDelay: '<S234>/Delay Input2'
   *
   * Block description for '<S234>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_a =
    VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hk3;

  /* Update for UnitDelay: '<S217>/Unit Delay2' */
  VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_l = rtb_APP_POS2;

  /* Update for UnitDelay: '<S227>/Unit Delay' incorporates:
   *  UnitDelay: '<S237>/Delay Input2'
   *
   * Block description for '<S237>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_nc =
    VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_c;

  /* Update for UnitDelay: '<S217>/Unit Delay3' */
  VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE = rtb_deltafalllimit_km;

  /* Update for UnitDelay: '<S244>/Unit Delay' incorporates:
   *  UnitDelay: '<S248>/Delay Input2'
   *
   * Block description for '<S248>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_l =
    VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_i;

  /* Update for UnitDelay: '<S218>/Unit Delay' */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_a0 = rtb_Brk_F;

  /* Update for UnitDelay: '<S245>/Unit Delay' incorporates:
   *  UnitDelay: '<S251>/Delay Input2'
   *
   * Block description for '<S251>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_ap =
    VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_el;

  /* Update for UnitDelay: '<S218>/Unit Delay1' */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_am = rtb_Add;

  /* Update for UnitDelay: '<S246>/Unit Delay' incorporates:
   *  UnitDelay: '<S254>/Delay Input2'
   *
   * Block description for '<S254>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_o =
    VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_p;

  /* Update for UnitDelay: '<S218>/Unit Delay2' */
  VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_c = rtb_APP_POS2;

  /* Update for UnitDelay: '<S247>/Unit Delay' incorporates:
   *  UnitDelay: '<S257>/Delay Input2'
   *
   * Block description for '<S257>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_ah =
    VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_mt;

  /* Update for UnitDelay: '<S218>/Unit Delay3' */
  VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_d = rtb_deltafalllimit_km;

  /* Update for UnitDelay: '<S150>/Unit Delay' incorporates:
   *  UnitDelay: '<S150>/Unit Delay1'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_lh =
    VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k;

  /* Update for UnitDelay: '<S207>/Unit Delay4' */
  VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_m = Acc_POS;

  /* Update for UnitDelay: '<S207>/Unit Delay3' */
  VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_f = rtb_Compare;

  /* Update for UnitDelay: '<S209>/Unit Delay3' incorporates:
   *  UnitDelay: '<S209>/Unit Delay1'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_p =
    VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_d;

  /* Update for UnitDelay: '<S209>/Unit Delay1' incorporates:
   *  UnitDelay: '<S268>/Delay Input2'
   *
   * Block description for '<S268>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_d =
    VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hu;

  /* Update for UnitDelay: '<S209>/Unit Delay2' */
  VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_f = rtb_CastToDouble;

  /* Update for UnitDelay: '<S266>/Unit Delay' incorporates:
   *  UnitDelay: '<S273>/Delay Input2'
   *
   * Block description for '<S273>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_ncs =
    VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_f;

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms1' */

  /* S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* RelationalOperator: '<S92>/Compare' incorporates:
   *  Constant: '<S92>/Constant'
   */
  rtb_Compare = (Brk_F >= 450);

  /* RelationalOperator: '<S93>/Compare' incorporates:
   *  Constant: '<S93>/Constant'
   */
  VehCtrlMdel240912_2018b_B.Compare_p = (Acc_POS <= 50.0F);

  /* Chart: '<S90>/Chart2' */
  FunctionCallSubsystem_ELAPS_T = VehCtrlMdel240912_2018b_M->Timing.clockTick3 -
    VehCtrlMdel240912_2018b_DW.previousTicks;
  VehCtrlMdel240912_2018b_DW.previousTicks =
    VehCtrlMdel240912_2018b_M->Timing.clockTick3;
  if (VehCtrlMdel240912_2018b_DW.temporalCounter_i1 +
      FunctionCallSubsystem_ELAPS_T <= 255U) {
    VehCtrlMdel240912_2018b_DW.temporalCounter_i1 = (uint8_T)
      (VehCtrlMdel240912_2018b_DW.temporalCounter_i1 +
       FunctionCallSubsystem_ELAPS_T);
  } else {
    VehCtrlMdel240912_2018b_DW.temporalCounter_i1 = MAX_uint8_T;
  }

  if (VehCtrlMdel240912_2018b_DW.temporalCounter_i2 +
      FunctionCallSubsystem_ELAPS_T <= 1023U) {
    VehCtrlMdel240912_2018b_DW.temporalCounter_i2 = (uint16_T)
      (VehCtrlMdel240912_2018b_DW.temporalCounter_i2 +
       FunctionCallSubsystem_ELAPS_T);
  } else {
    VehCtrlMdel240912_2018b_DW.temporalCounter_i2 = 1023U;
  }

  VehCtrlMdel240912_2018b_DW.sfEvent = -1;
  if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_c1_VehCtrlMdel240912_ ==
      0U) {
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_c1_VehCtrlMdel240912_ = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_VehStat = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat = 2U;
    VehCtrlMdel240912_2018b_DW.temporalCounter_i1 = 0U;
    VehCtrlMdel240912_2018b_B.errorReset = 1.0;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_BeeperStat = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_BeeperStat = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_Output = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_AMKDCready = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCready = 10U;
    VehCtrlMdel240912_2018b_DW.temporalCounter_i2 = 0U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_AMKCANenable = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKCANenable = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_AMKDCon = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCon = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_MCDCEnable = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MCDCEnable = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_MC_InverterOn = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MC_InverterOn = 1U;
  } else {
    if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_VehStat != 0U) {
      VehCtrlMdel240912_2018b_VehStat(&ignition, &rtb_LogicalOperator3,
        &VehCtrlMdel240912_2018b_B.MCFL_bSystemReady_c,
        &VehCtrlMdel240912_2018b_B.MCFR_bSystemReady_j, &controller_ready,
        &rtb_Compare);
    }

    if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_BeeperStat != 0U) {
      switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_BeeperStat) {
       case VehCtrlMdel240912_2018b_IN_OFF:
        if (VehCtrlMdel240912_2018b_DW.sfEvent ==
            VehCtrlMdel240912_event_EbeepON) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_BeeperStat = 2U;
        }
        break;

       case VehCtrlMdel240912_2018b_IN_ON:
        if (VehCtrlMdel240912_2018b_DW.sfEvent ==
            VehCtrlMdel24091_event_EbeepOFF) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_BeeperStat = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_Output != 0U) {
      VehCtrlMdel240912_2018b_B.VehReady =
        (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_VehStat ==
         VehCtrlMdel240912_2018_IN_Ready);
      beeper_state = (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_BeeperStat ==
                      VehCtrlMdel240912_2018b_IN_ON);
      VehCtrlMdel240912_2018b_B.MCFL_DCOn_setpoints =
        (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCon ==
         VehCtrlMdel240912_2018b_IN_ON);
      VehCtrlMdel240912_2018b_B.MCFL_DCEnable =
        (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MCDCEnable ==
         VehCtrlMdel240912_2018b_IN_ON);
      VehCtrlMdel240912_2018b_B.MCFL_InverterOn =
        (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MC_InverterOn ==
         VehCtrlMdel240912_2018b_IN_ON);
    }

    if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_AMKDCready != 0U) {
      switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCready) {
       case VehCtrlMdel240912_201_IN_AMKCAN:
        if (rtb_LogicalOperator2) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCready = 7U;
          VehCtrlMdel240912_2018b_DW.temporalCounter_i2 = 0U;
        }
        break;

       case VehCtrlMdel240_IN_AMKDCOnFinish:
       case VehCtrlMdel24091_IN_MC_DCOnFail:
        break;

       case VehCtrlMdel240_IN_DCOnCheckPass:
        rtb_LogicalOperator3 = ((VehCtrlMdel240912_2018b_DW.temporalCounter_i2 >=
          10U) && (VehCtrlMdel240912_2018b_B.MCFL_bDCOn_l != 0.0) &&
          (VehCtrlMdel240912_2018b_B.MCFR_bDCOn_j != 0.0));
        if (rtb_LogicalOperator3) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCready = 4U;
          VehCtrlMdel240912_2018b_DW.temporalCounter_i2 = 0U;
          d_previousEvent = VehCtrlMdel240912_2018b_DW.sfEvent;
          VehCtrlMdel240912_2018b_DW.sfEvent = VehCtrlMdel2_event_MCDCEnableON;
          if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_MCDCEnable != 0U)
          {
            switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MCDCEnable) {
             case VehCtrlMdel240912_2018b_IN_OFF:
              if (VehCtrlMdel240912_2018b_DW.sfEvent ==
                  VehCtrlMdel2_event_MCDCEnableON) {
                VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MCDCEnable = 2U;
              }
              break;

             case VehCtrlMdel240912_2018b_IN_ON:
              if (VehCtrlMdel240912_2018b_DW.sfEvent ==
                  VehCtrlMdel_event_MCDCEnableOFF) {
                VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MCDCEnable = 1U;
              }
              break;
            }
          }

          VehCtrlMdel240912_2018b_DW.sfEvent = d_previousEvent;
        }
        break;

       case VehCtrlMdel2_IN_MCDCEnableState:
        if (VehCtrlMdel240912_2018b_DW.temporalCounter_i2 >= 10U) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCready = 8U;
          d_previousEvent = VehCtrlMdel240912_2018b_DW.sfEvent;
          VehCtrlMdel240912_2018b_DW.sfEvent = VehCtrlMdel240_event_InverterON;
          if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_MC_InverterOn !=
              0U) {
            switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MC_InverterOn) {
             case VehCtrlMdel240912_2018b_IN_OFF:
              if (VehCtrlMdel240912_2018b_DW.sfEvent ==
                  VehCtrlMdel240_event_InverterON) {
                VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MC_InverterOn = 2U;
              }
              break;

             case VehCtrlMdel240912_2018b_IN_ON:
              if (VehCtrlMdel240912_2018b_DW.sfEvent ==
                  VehCtrlMdel24_event_InverterOFF) {
                VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MC_InverterOn = 1U;
              }
              break;
            }
          }

          VehCtrlMdel240912_2018b_DW.sfEvent = d_previousEvent;
        }
        break;

       case VehCtrlMdel24091_IN_MCDCOncheck:
        if (VehCtrlMdel240912_2018b_DW.temporalCounter_i2 >= 10U) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCready = 3U;
          VehCtrlMdel240912_2018b_DW.temporalCounter_i2 = 0U;
        } else {
          rtb_LogicalOperator3 = ((!(VehCtrlMdel240912_2018b_B.MCFR_bDCOn_j !=
            0.0)) || (!(VehCtrlMdel240912_2018b_B.MCFL_bDCOn_l != 0.0)));
          if (rtb_LogicalOperator3) {
            VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCready = 6U;
            VehCtr_enter_atomic_MC_DCOnFail();
          }
        }
        break;

       case VehCtrlMdel240912_IN_MC_DCready:
        if (!rtb_LogicalOperator2) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCready = 1U;
          VehCtrlMdel_enter_atomic_AMKCAN();
        } else {
          if (VehCtrlMdel240912_2018b_DW.temporalCounter_i2 >= 10U) {
            d_previousEvent = VehCtrlMdel240912_2018b_DW.sfEvent;
            VehCtrlMdel240912_2018b_DW.sfEvent = VehCtrlMdel240912_event_AMKDCON;
            if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_AMKDCon != 0U)
            {
              switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCon) {
               case VehCtrlMdel240912_2018b_IN_OFF:
                if (VehCtrlMdel240912_2018b_DW.sfEvent ==
                    VehCtrlMdel240912_event_AMKDCON) {
                  VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCon = 2U;
                }
                break;

               case VehCtrlMdel240912_2018b_IN_ON:
                if (VehCtrlMdel240912_2018b_DW.sfEvent ==
                    VehCtrlMdel24091_event_AMKDCOFF) {
                  VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCon = 1U;
                }
                break;
              }
            }

            VehCtrlMdel240912_2018b_DW.sfEvent = d_previousEvent;
            VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCready = 5U;
            VehCtrlMdel240912_2018b_DW.temporalCounter_i2 = 0U;
          }
        }
        break;

       case VehCtrlM_IN_MC_InverterOn_State:
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCready = 9U;
        break;

       case VehCtrlMdel240912_IN_SYSRDYCECK:
        rtb_LogicalOperator3 = ((VehCtrlMdel240912_2018b_B.MCFL_bSystemReady_c
          != 0.0) && (VehCtrlMdel240912_2018b_B.MCFR_bSystemReady_j != 0.0));
        if (rtb_LogicalOperator3) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCready = 2U;
        } else {
          rtb_LogicalOperator3 =
            ((!(VehCtrlMdel240912_2018b_B.MCFL_bSystemReady_c != 0.0)) ||
             (!(VehCtrlMdel240912_2018b_B.MCFR_bSystemReady_j != 0.0)));
          if (rtb_LogicalOperator3) {
            VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCready = 6U;
            VehCtr_enter_atomic_MC_DCOnFail();
          }
        }
        break;

       case VehCtrlMdel240912_2018_IN_start:
        if (VehCtrlMdel240912_2018b_DW.temporalCounter_i2 >= 1000U) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCready = 1U;
          VehCtrlMdel_enter_atomic_AMKCAN();
        }
        break;
      }
    }

    if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_AMKCANenable != 0U) {
      switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKCANenable) {
       case VehCtrlMdel240912_2018b_IN_OFF:
        if (VehCtrlMdel240912_2018b_DW.sfEvent ==
            VehCtrlMdel24091_event_AMKCANON) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKCANenable = 2U;
        }
        break;

       case VehCtrlMdel240912_2018b_IN_ON:
        if (VehCtrlMdel240912_2018b_DW.sfEvent ==
            VehCtrlMdel2409_event_AMKCANOFF) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKCANenable = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_AMKDCon != 0U) {
      switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCon) {
       case VehCtrlMdel240912_2018b_IN_OFF:
        if (VehCtrlMdel240912_2018b_DW.sfEvent ==
            VehCtrlMdel240912_event_AMKDCON) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCon = 2U;
        }
        break;

       case VehCtrlMdel240912_2018b_IN_ON:
        if (VehCtrlMdel240912_2018b_DW.sfEvent ==
            VehCtrlMdel24091_event_AMKDCOFF) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_AMKDCon = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_MCDCEnable != 0U) {
      switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MCDCEnable) {
       case VehCtrlMdel240912_2018b_IN_OFF:
        if (VehCtrlMdel240912_2018b_DW.sfEvent ==
            VehCtrlMdel2_event_MCDCEnableON) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MCDCEnable = 2U;
        }
        break;

       case VehCtrlMdel240912_2018b_IN_ON:
        if (VehCtrlMdel240912_2018b_DW.sfEvent ==
            VehCtrlMdel_event_MCDCEnableOFF) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MCDCEnable = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_MC_InverterOn != 0U) {
      switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MC_InverterOn) {
       case VehCtrlMdel240912_2018b_IN_OFF:
        if (VehCtrlMdel240912_2018b_DW.sfEvent ==
            VehCtrlMdel240_event_InverterON) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MC_InverterOn = 2U;
        }
        break;

       case VehCtrlMdel240912_2018b_IN_ON:
        if (VehCtrlMdel240912_2018b_DW.sfEvent ==
            VehCtrlMdel24_event_InverterOFF) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_MC_InverterOn = 1U;
        }
        break;
      }
    }
  }

  /* End of Chart: '<S90>/Chart2' */
  /* End of Outputs for S-Function (fcncallgen): '<S2>/10ms' */

  /* S-Function (fcncallgen): '<S5>/10ms1' incorporates:
   *  SubSystem: '<S5>/Beeper'
   */
  /* S-Function (ec5744_pdsslbu3): '<S279>/PowerDriverSwitch(HS)' */

  /* Set level beeper_state for the specified power driver switch */
  ec_gpio_write(83,beeper_state);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms1' */

  /* S-Function (fcncallgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/PwrTrainTempPrtct'
   */
  /* MinMax: '<S8>/Max' */
  elapseTime = fmax(VehCtrlMdel240912_2018b_B.MCFL_TempInverter_p,
                    VehCtrlMdel240912_2018b_B.MCFR_TempInverter_i);

  /* RelationalOperator: '<S83>/Compare' incorporates:
   *  Constant: '<S83>/Constant'
   */
  rtb_LogicalOperator2 = (elapseTime > 40.0);

  /* RelationalOperator: '<S84>/Compare' incorporates:
   *  Constant: '<S84>/Constant'
   */
  rtb_LogicalOperator3 = (elapseTime > 45.0);

  /* Logic: '<S8>/NOT' */
  rtb_Compare = !rtb_LogicalOperator3;

  /* Logic: '<S8>/AND' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 && rtb_Compare);

  /* Switch: '<S8>/Switch' incorporates:
   *  Constant: '<S8>/Constant'
   */
  if (rtb_LogicalOperator2) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table1' */
    WhlSpdRR_mps = look1_binlx(elapseTime,
      VehCtrlMdel240912_2018b_ConstP.pooled18,
      VehCtrlMdel240912_2018b_ConstP.pooled17, 7U);
  } else {
    WhlSpdRR_mps = 0.0;
  }

  /* End of Switch: '<S8>/Switch' */

  /* SignalConversion generated from: '<S8>/Out1' */
  WhlSpdFL = WhlSpdRR_mps;

  /* Chart: '<S8>/Timer1' incorporates:
   *  Constant: '<S8>/Constant2'
   */
  VehCtrlMdel240912_20_Timer1(rtb_LogicalOperator3, 0.11F,
    &VehCtrlMdel240912_2018b_B.Exit_g, &VehCtrlMdel240912_2018b_DW.sf_Timer1);

  /* RelationalOperator: '<S86>/Compare' incorporates:
   *  Constant: '<S86>/Constant'
   */
  rtb_LogicalOperator2 = (MCU_Temp > 50.0);

  /* Chart: '<S8>/Timer2' incorporates:
   *  Constant: '<S8>/Constant3'
   */
  VehCtrlMdel240912_20_Timer1(rtb_LogicalOperator2, 0.11F,
    &VehCtrlMdel240912_2018b_B.Exit_d, &VehCtrlMdel240912_2018b_DW.sf_Timer2);

  /* Logic: '<S8>/NOT1' */
  rtb_Compare = !rtb_LogicalOperator2;

  /* RelationalOperator: '<S85>/Compare' incorporates:
   *  Constant: '<S85>/Constant'
   */
  rtb_LogicalOperator2 = (MCU_Temp > 45.0);

  /* Logic: '<S8>/AND1' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 && rtb_Compare);

  /* MinMax: '<S8>/Max1' */
  rtb_Switch2_on = fmax(MCU_Temp, motor_Temp);

  /* Switch: '<S8>/Switch1' incorporates:
   *  Constant: '<S8>/Constant1'
   */
  if (rtb_LogicalOperator2) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table3' */
    WhlSpdRR_mps = look1_binlx(rtb_Switch2_on,
      VehCtrlMdel240912_2018b_ConstP.pooled18,
      VehCtrlMdel240912_2018b_ConstP.pooled17, 7U);
  } else {
    WhlSpdRR_mps = 0.0;
  }

  /* End of Switch: '<S8>/Switch1' */

  /* SignalConversion generated from: '<S8>/Out1' */
  WhlSpdFR = WhlSpdRR_mps;

  /* Lookup_n-D: '<S8>/2-D Lookup Table2' */
  WhlSpdRR_mps = look1_binlx(rtb_Switch2_on,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable2_bp01Data,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable2_tableData, 6U);

  /* DataTypeConversion: '<S8>/Cast To Single' */
  rtb_CastToDouble = (real32_T)WhlSpdRR_mps;

  /* Gain: '<S8>/Gain' */
  rtb_CastToDouble *= 10.0F;

  /* RelationalOperator: '<S87>/Compare' incorporates:
   *  Constant: '<S87>/Constant'
   */
  rtb_Compare = (elapseTime > 35.0);

  /* SignalConversion generated from: '<S8>/Out1' */
  VehCtrlMdel240912_2018b_B.aWaterPumpON = rtb_Compare;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/Function-Call Generator' */

  /* S-Function (fcncallgen): '<S1>/10ms1' incorporates:
   *  SubSystem: '<S1>/MoTrqReq'
   */
  if (VehCtrlMdel240912_2018b_DW.MoTrqReq_RESET_ELAPS_T) {
    FunctionCallSubsystem_ELAPS_T = 0U;
  } else {
    FunctionCallSubsystem_ELAPS_T = VehCtrlMdel240912_2018b_M->Timing.clockTick3
      - VehCtrlMdel240912_2018b_DW.MoTrqReq_PREV_T;
  }

  VehCtrlMdel240912_2018b_DW.MoTrqReq_PREV_T =
    VehCtrlMdel240912_2018b_M->Timing.clockTick3;
  VehCtrlMdel240912_2018b_DW.MoTrqReq_RESET_ELAPS_T = false;

  /* Lookup_n-D: '<S7>/2-D Lookup Table1' incorporates:
   *  UnitDelay: '<S150>/Unit Delay1'
   */
  rtb_Add6_p = look2_iflf_binlx(Acc_POS,
    VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable1_bp01Data_l,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable1_bp02Data,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable1_tableData_g,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable1_maxIndex, 11U);

  /* Gain: '<S10>/Gain4' */
  Acc_POS = 0.25F * rtb_Add6_p;

  /* SampleTimeMath: '<S32>/sample time'
   *
   * About '<S32>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S32>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant42'
   */
  rtb_g_mpss1 = 2000.0 * elapseTime;

  /* Lookup_n-D: '<S10>/2-D Lookup Table' */
  rtb_Gain4 = look2_binlx(rtb_Yk1_l, rtb_UkYk1_ix,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable_bp01Data,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable_bp02Data,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable_tableData,
    VehCtrlMdel240912_2018b_ConstP.uDLookupTable_maxIndex, 4U);

  /* UnitDelay: '<S35>/Delay Input2'
   *
   * Block description for '<S35>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add7 = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_d;

  /* SampleTimeMath: '<S35>/sample time'
   *
   * About '<S35>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_StrWhlAngV = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S35>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(1000.0 * rtb_StrWhlAngV);

  /* Gain: '<S10>/Gain21' */
  rtb_Switch2_on = 0.1020408163265306 * rtb_Add2;

  /* MATLAB Function: '<S10>/Wtarget' incorporates:
   *  Constant: '<S10>/Constant14'
   *  Constant: '<S10>/Constant15'
   *  Constant: '<S10>/Constant16'
   *  Constant: '<S10>/Constant17'
   *  Constant: '<S10>/Constant18'
   *  Constant: '<S10>/Constant19'
   *  UnitDelay: '<S150>/Unit Delay1'
   */
  rtb_Switch2_mn = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k * (real32_T)
    rtb_Yk1_l / (340.0F * VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k *
                 VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k * 15.5799866F /
                 460.0F / 440.0F / 1.592F / 2.0F + 1.592F);
  if (rtb_Switch2_mn < 0.0F) {
    rtb_Add10 = -1.0F;
  } else if (rtb_Switch2_mn > 0.0F) {
    rtb_Add10 = 1.0F;
  } else if (rtb_Switch2_mn == 0.0F) {
    rtb_Add10 = 0.0F;
  } else {
    rtb_Add10 = (rtNaNF);
  }

  rtb_Switch2_mn = fminf(fabsf(6.24750042F / (real32_T)rtb_Switch2_on), fabsf
    (rtb_Switch2_mn)) * rtb_Add10;

  /* End of MATLAB Function: '<S10>/Wtarget' */

  /* Sum: '<S35>/Difference Inputs1'
   *
   * Block description for '<S35>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_mn -= rtb_Add7;

  /* RelationalOperator: '<S50>/LowerRelop1' */
  rtb_LogicalOperator2 = (rtb_Switch2_mn > rtb_Switch2_b0);

  /* Switch: '<S50>/Switch2' */
  if (!rtb_LogicalOperator2) {
    /* Product: '<S35>/delta fall limit' */
    rtb_Add10 = (real32_T)(-1000.0 * rtb_StrWhlAngV);

    /* RelationalOperator: '<S50>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Switch2_mn < rtb_Add10);

    /* Switch: '<S50>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Switch2_mn = rtb_Add10;
    }

    /* End of Switch: '<S50>/Switch' */
    rtb_Switch2_b0 = rtb_Switch2_mn;
  }

  /* End of Switch: '<S50>/Switch2' */

  /* Sum: '<S35>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S35>/Delay Input2'
   *
   * Block description for '<S35>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S35>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_d = rtb_Switch2_b0 + rtb_Add7;

  /* Sum: '<S10>/Add' incorporates:
   *  UnitDelay: '<S35>/Delay Input2'
   *
   * Block description for '<S35>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain5 = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_d -
    rtb_deltafalllimit_iz;

  /* UnitDelay: '<S10>/Unit Delay3' */
  rtb_LogicalOperator2 = VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_l;

  /* Abs: '<S10>/Abs' */
  WhlSpdRL_mps = fabs(rtb_Gain5);

  /* RelationalOperator: '<S22>/Compare' incorporates:
   *  Constant: '<S22>/Constant'
   */
  rtb_AND2 = (WhlSpdRL_mps > 10.0);

  /* Abs: '<S10>/Abs1' */
  WhlSpdRL_mps = fabs(rtb_deltafalllimit_iz);

  /* RelationalOperator: '<S23>/Compare' incorporates:
   *  Constant: '<S23>/Constant'
   */
  rtb_AND_l = (WhlSpdRL_mps > 5.0);

  /* RelationalOperator: '<S24>/Compare' incorporates:
   *  Constant: '<S24>/Constant'
   *  UnitDelay: '<S150>/Unit Delay1'
   */
  rtb_Compare_in = (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k > 3.0F);

  /* UnitDelay: '<S7>/Unit Delay' */
  WhlSpdRL_mps = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_n;

  /* Logic: '<S10>/AND' */
  rtb_Compare = (rtb_AND2 && rtb_AND_l && rtb_Compare_in && (WhlSpdRL_mps != 0.0));

  /* Logic: '<S10>/Logical Operator4' */
  rtb_LogicalOperator2 = ((!rtb_LogicalOperator2) && (!rtb_Compare));

  /* Abs: '<S10>/Abs2' */
  WhlSpdRL_mps = fabs(rtb_Gain5);

  /* RelationalOperator: '<S25>/Compare' incorporates:
   *  Constant: '<S25>/Constant'
   */
  rtb_Compare = (WhlSpdRL_mps < 10.0);

  /* RelationalOperator: '<S26>/Compare' incorporates:
   *  Constant: '<S26>/Constant'
   */
  rtb_Compare_in = (rtb_Add2 < -5.0);

  /* Logic: '<S10>/OR' */
  rtb_Compare = (rtb_Compare || rtb_Compare_in);

  /* Switch: '<S10>/Switch6' incorporates:
   *  Constant: '<S10>/Reset'
   */
  if (rtb_Compare) {
    /* Sum: '<S10>/Add10' incorporates:
     *  Constant: '<S10>/Steptime'
     */
    rtb_Switch2_b0 = VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_n + 0.01F;
  } else {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Switch: '<S10>/Switch6' */

  /* MinMax: '<S10>/Min' incorporates:
   *  Constant: '<S10>/ResetDelay'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_n = fminf(rtb_Switch2_b0, 0.5F);

  /* RelationalOperator: '<S10>/Relational Operator9' incorporates:
   *  Constant: '<S10>/ResetDelay'
   *  UnitDelay: '<S10>/Unit Delay4'
   */
  rtb_Compare = (VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_n >= 0.5F);

  /* Logic: '<S10>/Logical Operator5' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_l = ((!rtb_LogicalOperator2) && (
    !rtb_Compare));

  /* Switch: '<S10>/Switch1' incorporates:
   *  Constant: '<S10>/Constant1'
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  if (!VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_l) {
    rtb_Gain5 = 0.0;
  }

  /* End of Switch: '<S10>/Switch1' */

  /* Product: '<S10>/Product3' */
  rtb_Gain4 *= rtb_Gain5;

  /* UnitDelay: '<S10>/Unit Delay2' */
  WhlSpdRL_mps = VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_d;

  /* Product: '<S10>/Product' */
  WhlSpdRL_mps *= rtb_Yk1_l;

  /* RelationalOperator: '<S21>/Compare' incorporates:
   *  Constant: '<S21>/Constant'
   */
  rtb_Compare = (WhlSpdRL_mps <= 0.0);

  /* UnitDelay: '<S10>/Unit Delay' */
  WhlSpdRL_mps = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_m;

  /* Switch: '<S10>/Switch' incorporates:
   *  Constant: '<S10>/Constant'
   */
  if (rtb_Compare) {
    WhlSpdRL_mps = 0.0;
  }

  /* End of Switch: '<S10>/Switch' */

  /* Product: '<S10>/Product4' */
  WhlSpdRR_mps = rtb_Gain5;

  /* UnitDelay: '<S10>/Unit Delay1' */
  rtb_Gain5 = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_p;

  /* Product: '<S10>/Product5' */
  rtb_Switch2_on = rtb_Gain5;

  /* Sum: '<S10>/Add2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_m = (WhlSpdRL_mps + WhlSpdRR_mps)
    - rtb_Switch2_on;

  /* MATLAB Function: '<S10>/MATLAB Function1' incorporates:
   *  Constant: '<S10>/Constant20'
   *  Constant: '<S10>/Constant21'
   *  Constant: '<S10>/Constant22'
   *  Constant: '<S10>/Constant23'
   *  Constant: '<S10>/Constant24'
   *  Constant: '<S10>/Constant25'
   *  UnitDelay: '<S150>/Unit Delay1'
   */
  rtb_Add7 = ((5.43088F / VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k *
               (-1.35294116F / VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k) -
               (-0.0458234884F / VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k /
                VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k - 1.0F) *
               -3.33390474F) * 105.0F * (real32_T)rtb_Yk1_l - -1.35294116F /
              VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k * 105.0F *
              (real32_T)rtb_UkYk1_ix) / (-0.0458234884F /
    VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k /
    VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k - 1.0F) / 100.0F;

  /* UnitDelay: '<S10>/Unit Delay6' */
  rtb_Compare = VehCtrlMdel240912_2018b_DW.UnitDelay6_DSTATE_i;

  /* Logic: '<S10>/Logical Operator3' */
  rtb_Compare = !rtb_Compare;

  /* Switch: '<S10>/Switch3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay5'
   */
  if (rtb_Compare) {
    /* Switch: '<S10>/Switch4' incorporates:
     *  Constant: '<S10>/InitZORE'
     *  UnitDelay: '<S10>/Unit Delay3'
     */
    if (!VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_l) {
      rtb_Add7 = 0.0F;
    }

    /* End of Switch: '<S10>/Switch4' */
    VehCtrlMdel240912_2018b_DW.UnitDelay5_DSTATE_b = rtb_Add7;
  }

  /* End of Switch: '<S10>/Switch3' */

  /* Gain: '<S10>/Gain18' incorporates:
   *  UnitDelay: '<S10>/Unit Delay5'
   */
  rtb_Switch2_b0 = 0.01F * VehCtrlMdel240912_2018b_DW.UnitDelay5_DSTATE_b;

  /* Sum: '<S10>/Add1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay'
   */
  rtb_Switch2_on = (rtb_Gain4 + VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_m) +
    rtb_Switch2_b0;

  /* Saturate: '<S10>/Saturation' */
  if (rtb_Switch2_on > 1000.0) {
    rtb_UkYk1_ix = 1000.0;
  } else if (rtb_Switch2_on < -1000.0) {
    rtb_UkYk1_ix = -1000.0;
  } else {
    rtb_UkYk1_ix = rtb_Switch2_on;
  }

  /* End of Saturate: '<S10>/Saturation' */

  /* Sum: '<S32>/Difference Inputs1' incorporates:
   *  UnitDelay: '<S32>/Delay Input2'
   *
   * Block description for '<S32>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S32>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = rtb_UkYk1_ix - VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_jk;

  /* RelationalOperator: '<S47>/LowerRelop1' */
  rtb_Compare = (WhlSpdRR_mps > rtb_g_mpss1);

  /* Switch: '<S47>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S32>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S47>/UpperRelop' */
    rtb_LogicalOperator2 = (WhlSpdRR_mps < elapseTime);

    /* Switch: '<S47>/Switch' */
    if (rtb_LogicalOperator2) {
      WhlSpdRR_mps = elapseTime;
    }

    /* End of Switch: '<S47>/Switch' */
    rtb_g_mpss1 = WhlSpdRR_mps;
  }

  /* End of Switch: '<S47>/Switch2' */

  /* Sum: '<S32>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S32>/Delay Input2'
   *
   * Block description for '<S32>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S32>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_jk += rtb_g_mpss1;

  /* Gain: '<S10>/Gain8' */
  WhlSpdRR_mps = 0.017453292519943295 * FLWhlStrAng;

  /* Trigonometry: '<S10>/Cos' */
  WhlSpdRR_mps = cos(WhlSpdRR_mps);

  /* Gain: '<S10>/Gain11' */
  WhlSpdRR_mps *= 1.2;

  /* Sum: '<S10>/Add6' incorporates:
   *  Constant: '<S10>/Constant27'
   */
  WhlSpdRL_mps = 90.0 - FLWhlStrAng;

  /* Gain: '<S10>/Gain9' */
  WhlSpdRL_mps *= 0.017453292519943295;

  /* Trigonometry: '<S10>/Cos1' */
  WhlSpdRL_mps = cos(WhlSpdRL_mps);

  /* Gain: '<S10>/Gain10' */
  WhlSpdRL_mps *= 1.522;

  /* Sum: '<S10>/Add7' */
  WhlSpdRR_mps += WhlSpdRL_mps;

  /* Product: '<S10>/Product1' incorporates:
   *  UnitDelay: '<S32>/Delay Input2'
   *
   * Block description for '<S32>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps *= VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_jk;

  /* Saturate: '<S10>/Saturation3' */
  if (WhlSpdRR_mps > 20.0) {
    WhlSpdRR_mps = 20.0;
  } else {
    if (WhlSpdRR_mps < -20.0) {
      WhlSpdRR_mps = -20.0;
    }
  }

  /* End of Saturate: '<S10>/Saturation3' */

  /* Sum: '<S10>/Add4' */
  FLWhlStrAng = Acc_POS + WhlSpdRR_mps;

  /* Lookup_n-D: '<S10>/228' */
  elapseTime = look1_binlx(RPM, VehCtrlMdel240912_2018b_ConstP.u28_bp01Data,
    VehCtrlMdel240912_2018b_ConstP.u28_tableData, 26U);

  /* Lookup_n-D: '<S10>/AMK' */
  WhlSpdRR_mps = look1_binlx(VehCtrlMdel240912_2018b_B.MCFL_ActualVelocity_p,
    VehCtrlMdel240912_2018b_ConstP.pooled14,
    VehCtrlMdel240912_2018b_ConstP.pooled13, 19U);

  /* Lookup_n-D: '<S10>/AMK1' */
  WhlSpdRL_mps = look1_binlx(VehCtrlMdel240912_2018b_B.MCFR_ActualVelocity_k,
    VehCtrlMdel240912_2018b_ConstP.pooled14,
    VehCtrlMdel240912_2018b_ConstP.pooled13, 19U);

  /* MATLAB Function: '<S10>/ÔØºÉ×ªÒÆ' incorporates:
   *  Constant: '<S10>/Constant10'
   *  Constant: '<S10>/Constant2'
   *  Constant: '<S10>/Constant3'
   *  Constant: '<S10>/Constant4'
   *  Constant: '<S10>/Constant5'
   *  Constant: '<S10>/Constant9'
   */
  rtb_Switch2_mn = (1666.0F - 340.0F * (real32_T)rtb_deltafalllimit_i4 * 0.29F /
                    1.2F) * 0.521984875F - 170.0F * (real32_T)rtb_Add2 * 0.29F /
    1.592F;
  rtb_Add10 = (340.0F * (real32_T)rtb_deltafalllimit_i4 * 0.29F / 1.2F + 1666.0F)
    * 0.521984875F - 170.0F * (real32_T)rtb_Add2 * 0.29F / 1.592F;
  rtb_Ax = (1666.0F - 340.0F * (real32_T)rtb_deltafalllimit_i4 * 0.29F / 1.2F) *
    0.521984875F + 170.0F * (real32_T)rtb_Add2 * 0.29F / 1.592F;
  rtb_Add4_j = (340.0F * (real32_T)rtb_deltafalllimit_i4 * 0.29F / 1.2F +
                1666.0F) * 0.521984875F + 170.0F * (real32_T)rtb_Add2 * 0.29F /
    1.592F;

  /* Gain: '<S10>/Gain3' */
  rtb_Gain5 = 0.1020408163265306 * rtb_deltafalllimit_i4;

  /* MATLAB Function: '<S10>/MATLAB Function' incorporates:
   *  Constant: '<S10>/Constant11'
   *  Constant: '<S10>/Constant12'
   *  Constant: '<S10>/Constant13'
   *  Constant: '<S10>/Constant26'
   */
  rtb_Add7 = rtb_Switch2_mn * 0.75F;
  rtb_Switch2_mn = rtb_Switch2_mn * (real32_T)rtb_Gain5 / 9.8F;
  rtb_Switch2_b0 = rtb_Add10 * 0.75F;
  rtb_MaxWhlSpd_mps_n = rtb_Add10 * (real32_T)rtb_Gain5 / 9.8F;
  rtb_Add10 = rtb_Ax * 0.75F;
  rtb_Ax = rtb_Ax * (real32_T)rtb_Gain5 / 9.8F;
  rtb_VxIMU_est = rtb_Add4_j * 0.75F;
  rtb_Add4_j = rtb_Add4_j * (real32_T)rtb_Gain5 / 9.8F;
  rtb_Switch2_b0 = fminf(sqrtf(rtb_Switch2_b0 * rtb_Switch2_b0 -
    rtb_MaxWhlSpd_mps_n * rtb_MaxWhlSpd_mps_n) * 0.2F / 11.4F, (real32_T)
    WhlSpdRL_mps);
  rtb_Add7 = fminf(sqrtf(rtb_Add7 * rtb_Add7 - rtb_Switch2_mn * rtb_Switch2_mn) *
                   0.2F / 11.4F, (real32_T)WhlSpdRR_mps);
  rtb_Add10 = fminf((sqrtf(rtb_VxIMU_est * rtb_VxIMU_est - rtb_Add4_j *
    rtb_Add4_j) + sqrtf(rtb_Add10 * rtb_Add10 - rtb_Ax * rtb_Ax)) / 2.0F * 0.2F /
                    3.4F, (real32_T)elapseTime);

  /* Gain: '<S10>/Gain' */
  rtb_Switch2_b0 *= 0.95F;

  /* Gain: '<S10>/Gain1' */
  rtb_Add7 *= 0.95F;

  /* MinMax: '<S10>/Min1' */
  rtb_Switch2_mn = fminf(rtb_Switch2_b0, rtb_Add7);

  /* Sum: '<S10>/Add14' */
  WhlSpdRR_mps = rtb_Switch2_mn - FLWhlStrAng;

  /* RelationalOperator: '<S10>/Relational Operator' incorporates:
   *  Constant: '<S10>/Constant37'
   */
  rtb_Compare_am = (WhlSpdRR_mps < 0.0);

  /* Gain: '<S10>/Gain2' */
  rtb_Switch2_mn = 0.95F * rtb_Add10;

  /* Gain: '<S10>/Gain5' */
  rtb_Add6_p *= 0.5F;

  /* Sum: '<S10>/Add15' */
  rtb_Add10 = rtb_Switch2_mn - rtb_Add6_p;

  /* RelationalOperator: '<S10>/Relational Operator1' incorporates:
   *  Constant: '<S10>/Constant38'
   */
  rtb_LowerRelop1_b = (rtb_Add10 < 0.0F);

  /* Logic: '<S10>/AND1' */
  rtb_Compare = (rtb_Compare_am && rtb_LowerRelop1_b);

  /* Logic: '<S10>/OR1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_Compare = (VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_l || rtb_Compare);

  /* Switch: '<S10>/Switch2' incorporates:
   *  Constant: '<S10>/Constant32'
   */
  if (rtb_Compare) {
    WhlSpdRR_mps = 0.0;
  } else {
    /* Logic: '<S10>/NOT' */
    rtb_LogicalOperator2 = !rtb_LowerRelop1_b;

    /* Switch: '<S10>/Switch5' incorporates:
     *  Constant: '<S10>/Constant33'
     */
    if (!rtb_LogicalOperator2) {
      WhlSpdRR_mps = 0.0;
    }

    /* End of Switch: '<S10>/Switch5' */
  }

  /* End of Switch: '<S10>/Switch2' */

  /* Saturate: '<S10>/Saturation1' */
  if (WhlSpdRR_mps > 100.0) {
    elapseTime = 100.0;
  } else if (WhlSpdRR_mps < 0.0) {
    elapseTime = 0.0;
  } else {
    elapseTime = WhlSpdRR_mps;
  }

  /* End of Saturate: '<S10>/Saturation1' */

  /* UnitDelay: '<S34>/Delay Input2'
   *
   * Block description for '<S34>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_n0;

  /* Sum: '<S34>/Difference Inputs1'
   *
   * Block description for '<S34>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 = elapseTime - WhlSpdRR_mps;

  /* SampleTimeMath: '<S34>/sample time'
   *
   * About '<S34>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S34>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant45'
   */
  WhlSpdRL_mps = 1000.0 * elapseTime;

  /* RelationalOperator: '<S49>/LowerRelop1' */
  rtb_Compare = (rtb_Gain5 > WhlSpdRL_mps);

  /* Switch: '<S49>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S34>/delta fall limit' */
    elapseTime *= -1000.0;

    /* RelationalOperator: '<S49>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Gain5 < elapseTime);

    /* Switch: '<S49>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S49>/Switch' */
    WhlSpdRL_mps = rtb_Gain5;
  }

  /* End of Switch: '<S49>/Switch2' */

  /* Sum: '<S34>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S34>/Delay Input2'
   *
   * Block description for '<S34>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S34>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_n0 = WhlSpdRL_mps + WhlSpdRR_mps;

  /* Gain: '<S10>/Gain19' incorporates:
   *  UnitDelay: '<S34>/Delay Input2'
   *
   * Block description for '<S34>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = -VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_n0;

  /* RelationalOperator: '<S38>/LowerRelop1' */
  rtb_Compare = (rtb_Add6_p > rtb_Switch2_mn);

  /* Switch: '<S38>/Switch2' */
  if (rtb_Compare) {
    rtb_Add6_p = rtb_Switch2_mn;
  } else {
    /* RelationalOperator: '<S38>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    rtb_LogicalOperator2 = (rtb_Add6_p < 0.0F);

    /* Switch: '<S38>/Switch' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    if (rtb_LogicalOperator2) {
      rtb_Add6_p = 0.0F;
    }

    /* End of Switch: '<S38>/Switch' */
  }

  /* End of Switch: '<S38>/Switch2' */

  /* Sum: '<S10>/Add13' */
  elapseTime = rtb_Add6_p + WhlSpdRR_mps;

  /* RelationalOperator: '<S41>/LowerRelop1' */
  rtb_Compare = (elapseTime > rtb_Switch2_mn);

  /* Switch: '<S41>/Switch2' */
  if (rtb_Compare) {
    elapseTime = rtb_Switch2_mn;
  } else {
    /* RelationalOperator: '<S41>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    rtb_LogicalOperator2 = (elapseTime < 0.0);

    /* Switch: '<S41>/Switch' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    if (rtb_LogicalOperator2) {
      elapseTime = 0.0;
    }

    /* End of Switch: '<S41>/Switch' */
  }

  /* End of Switch: '<S41>/Switch2' */

  /* UnitDelay: '<S31>/Delay Input2'
   *
   * Block description for '<S31>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hk;

  /* Sum: '<S31>/Difference Inputs1'
   *
   * Block description for '<S31>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 = elapseTime - WhlSpdRR_mps;

  /* SampleTimeMath: '<S31>/sample time'
   *
   * About '<S31>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S31>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  WhlSpdRL_mps = 2000.0 * elapseTime;

  /* RelationalOperator: '<S46>/LowerRelop1' */
  rtb_Compare = (rtb_Gain5 > WhlSpdRL_mps);

  /* Switch: '<S46>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S31>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S46>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Gain5 < elapseTime);

    /* Switch: '<S46>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S46>/Switch' */
    WhlSpdRL_mps = rtb_Gain5;
  }

  /* End of Switch: '<S46>/Switch2' */

  /* Sum: '<S31>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S31>/Delay Input2'
   *
   * Block description for '<S31>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S31>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hk = WhlSpdRL_mps + WhlSpdRR_mps;

  /* Sum: '<S17>/Add' incorporates:
   *  Constant: '<S17>/Constant3'
   */
  WhlSpdRR_mps = 1.0 - WhlSpdFR;

  /* Product: '<S17>/Product5' */
  WhlSpdRL_mps = 63.0 * WhlSpdRR_mps;

  /* Product: '<S17>/Product' */
  WhlSpdRR_mps = WhlSpdRL_mps * 9550.0;

  /* Sum: '<S17>/Add1' incorporates:
   *  Constant: '<S17>/RPM_min'
   */
  WhlSpdRL_mps = RPM + 10.0;

  /* MinMax: '<S17>/Max' incorporates:
   *  Constant: '<S17>/RPM_min1'
   */
  rtb_Gain5 = fmax(WhlSpdRL_mps, 1.0);

  /* Product: '<S17>/Divide' */
  WhlSpdRR_mps /= rtb_Gain5;

  /* MinMax: '<S7>/MinMax2' incorporates:
   *  UnitDelay: '<S31>/Delay Input2'
   *
   * Block description for '<S31>/Delay Input2':
   *
   *  Store in Global RAM
   */
  elapseTime = fmin(WhlSpdRR_mps,
                    VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hk);

  /* Saturate: '<S20>/Saturation' */
  if (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k > 40.0F) {
    rtb_Switch2_mn = 40.0F;
  } else if (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k < 0.0F) {
    rtb_Switch2_mn = 0.0F;
  } else {
    rtb_Switch2_mn = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k;
  }

  /* Lookup_n-D: '<S20>/VehSpd_SlipTarget_mps' */
  rtb_Add6_p = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240912_2018b_ConstP.pooled52,
    VehCtrlMdel240912_2018b_ConstP.pooled51, 3U);

  /* Sum: '<S20>/Add9' */
  rtb_Add6_p += rtb_Switch2_mn;

  /* Sum: '<S7>/Add1' */
  rtb_Ax = rtb_APP_POS2 + rtb_deltafalllimit_km;

  /* Gain: '<S7>/Gain2' */
  rtb_Ax *= 0.5F;

  /* Saturate: '<S20>/Saturation1' */
  if (rtb_Ax < 0.0F) {
    rtb_VxIMU_est = 0.0F;
  } else {
    rtb_VxIMU_est = rtb_Ax;
  }

  /* End of Saturate: '<S20>/Saturation1' */

  /* Sum: '<S20>/Add1' */
  rtb_APP_POS2 = rtb_Add6_p - rtb_VxIMU_est;

  /* UnitDelay: '<S77>/Unit Delay1' */
  rtb_Compare = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_e;

  /* RelationalOperator: '<S20>/Relational Operator7' incorporates:
   *  Constant: '<S20>/Cal_DeltaV_mps'
   */
  rtb_Compare_in = (rtb_APP_POS2 < 0.0F);

  /* Logic: '<S77>/Logical Operator4' */
  rtb_Compare = ((!rtb_Compare) && (!rtb_Compare_in));

  /* Logic: '<S20>/Logical Operator2' */
  rtb_Compare_in = !rtb_Compare_in;

  /* UnitDelay: '<S20>/Unit Delay4' */
  WhlSpdRR_mps = VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE;

  /* RelationalOperator: '<S20>/Relational Operator8' incorporates:
   *  Constant: '<S20>/Cal_DeltaV_mps1'
   */
  rtb_AND_l = (WhlSpdRR_mps > 235.0);

  /* Logic: '<S20>/Logical Operator1' */
  rtb_Compare_in = (rtb_Compare_in && rtb_AND_l);

  /* Switch: '<S78>/Switch6' incorporates:
   *  Constant: '<S78>/Reset'
   */
  if (rtb_Compare_in) {
    /* Sum: '<S78>/Add10' incorporates:
     *  Constant: '<S78>/Steptime'
     */
    rtb_Ax = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_i + 0.01F;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S78>/Switch6' */

  /* MinMax: '<S78>/Min' incorporates:
   *  Constant: '<S20>/ResetDelay'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_i = fminf(rtb_Ax, 0.1F);

  /* RelationalOperator: '<S78>/Relational Operator9' incorporates:
   *  Constant: '<S20>/ResetDelay'
   *  UnitDelay: '<S78>/Unit Delay1'
   */
  rtb_Compare_in = (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_i >= 0.1F);

  /* UnitDelay: '<S20>/Unit Delay3' */
  rtb_AND_l = VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_a;

  /* Logic: '<S20>/Logical Operator3' */
  rtb_Compare_in = (rtb_Compare_in || rtb_AND_l);

  /* Logic: '<S77>/Logical Operator5' incorporates:
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_e = ((!rtb_Compare) &&
    (!rtb_Compare_in));

  /* Switch: '<S20>/Switch6' incorporates:
   *  Constant: '<S20>/Verror_Reset'
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  if (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_e) {
    rtb_Ax = rtb_APP_POS2;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S20>/Switch6' */

  /* Product: '<S20>/Product' incorporates:
   *  Constant: '<S20>/P_Gain'
   */
  rtb_deltafalllimit_km = rtb_Ax * 40.0F;

  /* Sum: '<S20>/Add11' */
  WhlSpdRR_mps = elapseTime - rtb_deltafalllimit_km;

  /* UnitDelay: '<S20>/Unit Delay5' */
  rtb_Add6_p = VehCtrlMdel240912_2018b_DW.UnitDelay5_DSTATE_l;

  /* Product: '<S20>/Product2' */
  rtb_Add6_p *= rtb_APP_POS2;

  /* RelationalOperator: '<S72>/Compare' incorporates:
   *  Constant: '<S72>/Constant'
   */
  rtb_Compare = (rtb_Add6_p <= 0.0F);

  /* UnitDelay: '<S20>/Unit Delay' */
  rtb_Add6_p = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_b;

  /* Switch: '<S20>/Switch3' incorporates:
   *  Constant: '<S20>/Verror_Reset1'
   */
  if (rtb_Compare) {
    rtb_Add6_p = 0.0F;
  }

  /* End of Switch: '<S20>/Switch3' */

  /* Sum: '<S20>/Add2' */
  rtb_Add6_p += rtb_Ax;

  /* Saturate: '<S20>/Saturation2' */
  if (rtb_Add6_p > 400.0F) {
    VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_b = 400.0F;
  } else if (rtb_Add6_p < -100.0F) {
    VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_b = -100.0F;
  } else {
    VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_b = rtb_Add6_p;
  }

  /* End of Saturate: '<S20>/Saturation2' */

  /* RelationalOperator: '<S82>/Compare' incorporates:
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  rtb_LogicalOperator2 = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_e;

  /* UnitDelay: '<S76>/Delay Input1'
   *
   * Block description for '<S76>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_Compare = VehCtrlMdel240912_2018b_DW.DelayInput1_DSTATE;

  /* RelationalOperator: '<S76>/FixPt Relational Operator' */
  rtb_Compare = ((int32_T)rtb_LogicalOperator2 > (int32_T)rtb_Compare);

  /* Switch: '<S20>/Switch' incorporates:
   *  Constant: '<S20>/Integr_StartPoint'
   */
  if (rtb_Compare) {
    /* Sum: '<S20>/Add4' */
    WhlSpdRL_mps = elapseTime - VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_g;
  } else {
    WhlSpdRL_mps = 0.0;
  }

  /* End of Switch: '<S20>/Switch' */

  /* Lookup_n-D: '<S20>/VehicleStableTarget_mps' */
  rtb_Ax = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240912_2018b_ConstP.pooled52,
    VehCtrlMdel240912_2018b_ConstP.pooled59, 3U);

  /* Sum: '<S20>/Add5' */
  rtb_Ax += rtb_Switch2_mn;

  /* Sum: '<S20>/Add10' */
  rtb_Ax = rtb_VxIMU_est - rtb_Ax;

  /* RelationalOperator: '<S20>/Relational Operator' incorporates:
   *  Constant: '<S20>/Verror'
   */
  rtb_Compare = (rtb_Ax < 0.0F);

  /* Logic: '<S20>/Logical Operator4' incorporates:
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  rtb_Compare = (rtb_Compare && VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_e);

  /* Switch: '<S20>/Switch1' incorporates:
   *  Constant: '<S20>/Trq_IReset'
   *  Constant: '<S20>/Trq_I_FF'
   */
  if (rtb_Compare) {
    rtb_Ax = 20.0F;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S20>/Switch1' */

  /* Sum: '<S20>/Add6' incorporates:
   *  UnitDelay: '<S20>/Unit Delay'
   */
  rtb_Gain5 = (VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_b + WhlSpdRL_mps) +
    rtb_Ax;

  /* Product: '<S20>/Product1' */
  WhlSpdFR = rtb_Gain5 * 10.0;

  /* RelationalOperator: '<S79>/LowerRelop1' */
  rtb_Compare = (WhlSpdFR > WhlSpdRR_mps);

  /* Switch: '<S79>/Switch2' */
  if (!rtb_Compare) {
    /* Gain: '<S20>/Gain3' */
    rtb_Add6_p = -rtb_deltafalllimit_km;

    /* RelationalOperator: '<S79>/UpperRelop' */
    rtb_LogicalOperator3 = (WhlSpdFR < rtb_Add6_p);

    /* Switch: '<S79>/Switch' */
    if (rtb_LogicalOperator3) {
      WhlSpdFR = rtb_Add6_p;
    }

    /* End of Switch: '<S79>/Switch' */
    WhlSpdRR_mps = WhlSpdFR;
  }

  /* End of Switch: '<S79>/Switch2' */

  /* Sum: '<S20>/Add7' incorporates:
   *  UnitDelay: '<S20>/Unit Delay4'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE = rtb_deltafalllimit_km +
    WhlSpdRR_mps;

  /* Lookup_n-D: '<S20>/VehicleStableTarget_mps1' */
  rtb_Ax = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240912_2018b_ConstP.pooled52,
    VehCtrlMdel240912_2018b_ConstP.pooled59, 3U);

  /* Sum: '<S20>/Add13' */
  rtb_Switch2_mn += rtb_Ax;

  /* Sum: '<S20>/Add12' */
  rtb_Switch2_mn = rtb_VxIMU_est - rtb_Switch2_mn;

  /* RelationalOperator: '<S20>/Relational Operator1' incorporates:
   *  Constant: '<S20>/Verror1'
   */
  rtb_Compare = (rtb_Switch2_mn < 0.0F);

  /* RelationalOperator: '<S20>/Relational Operator2' incorporates:
   *  UnitDelay: '<S20>/Unit Delay4'
   */
  rtb_Compare_in = (VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE >= elapseTime);

  /* RelationalOperator: '<S74>/Compare' incorporates:
   *  Constant: '<S74>/Constant'
   *  UnitDelay: '<S20>/Unit Delay4'
   */
  rtb_AND_l = (VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE <= 0.01);

  /* Logic: '<S20>/OR' */
  rtb_Compare_in = (rtb_Compare_in || rtb_AND_l);

  /* Logic: '<S20>/Logical Operator5' incorporates:
   *  UnitDelay: '<S20>/Unit Delay3'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_a = (rtb_Compare &&
    rtb_Compare_in);

  /* Switch: '<S20>/Switch2' incorporates:
   *  Switch: '<S20>/Switch7'
   *  UnitDelay: '<S20>/Unit Delay3'
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  if (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_e) {
    /* RelationalOperator: '<S80>/LowerRelop1' incorporates:
     *  Constant: '<S20>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S20>/Unit Delay4'
     */
    rtb_LogicalOperator3 = (VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE > 235.0);

    /* Switch: '<S80>/Switch2' incorporates:
     *  Constant: '<S20>/TCS_TrqRequest_Max2'
     */
    if (rtb_LogicalOperator3) {
      VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_g = 235.0;
    } else {
      /* RelationalOperator: '<S80>/UpperRelop' incorporates:
       *  Constant: '<S20>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S20>/Unit Delay4'
       */
      rtb_LogicalOperator3 = (VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE < 0.0);

      /* Switch: '<S80>/Switch' incorporates:
       *  Constant: '<S20>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S20>/Unit Delay4'
       */
      if (rtb_LogicalOperator3) {
        VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_g = 0.0;
      } else {
        VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_g =
          VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE;
      }

      /* End of Switch: '<S80>/Switch' */
    }

    /* End of Switch: '<S80>/Switch2' */

    /* RelationalOperator: '<S81>/LowerRelop1' */
    rtb_LogicalOperator3 = (elapseTime >
      VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_g);

    /* Switch: '<S81>/Switch2' */
    if (!rtb_LogicalOperator3) {
      /* RelationalOperator: '<S81>/UpperRelop' incorporates:
       *  Constant: '<S20>/TCS_TrqRequest_Min1'
       */
      rtb_LogicalOperator3 = (elapseTime < 0.0);

      /* Switch: '<S81>/Switch' incorporates:
       *  Constant: '<S20>/TCS_TrqRequest_Min1'
       */
      if (rtb_LogicalOperator3) {
        VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_g = 0.0;
      } else {
        VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_g = elapseTime;
      }

      /* End of Switch: '<S81>/Switch' */
    }

    /* End of Switch: '<S81>/Switch2' */
  } else {
    if (VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_a) {
      /* Switch: '<S20>/Switch7' */
      VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_g = elapseTime;
    }
  }

  /* End of Switch: '<S20>/Switch2' */

  /* UnitDelay: '<S66>/Unit Delay1' */
  rtb_Compare = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_gl;

  /* Saturate: '<S19>/Saturation' */
  if (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k > 40.0F) {
    rtb_Ax = 40.0F;
  } else if (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k < 0.0F) {
    rtb_Ax = 0.0F;
  } else {
    rtb_Ax = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Saturate: '<S19>/Saturation' */

  /* Lookup_n-D: '<S19>/VehSpd_SlipTarget_mps' */
  rtb_Switch2_mn = look1_iflf_binlc(rtb_Ax,
    VehCtrlMdel240912_2018b_ConstP.pooled52,
    VehCtrlMdel240912_2018b_ConstP.pooled51, 3U);

  /* Sum: '<S19>/Add9' */
  rtb_Switch2_mn += rtb_Ax;

  /* Saturate: '<S19>/Saturation1' */
  if (rtb_Add > 50.0F) {
    rtb_Add6_p = 50.0F;
  } else if (rtb_Add < 0.0F) {
    rtb_Add6_p = 0.0F;
  } else {
    rtb_Add6_p = rtb_Add;
  }

  /* End of Saturate: '<S19>/Saturation1' */

  /* Sum: '<S19>/Add1' */
  rtb_Add = rtb_Switch2_mn - rtb_Add6_p;

  /* RelationalOperator: '<S19>/Relational Operator7' incorporates:
   *  Constant: '<S19>/Cal_DeltaV_mps'
   */
  rtb_Compare_in = (rtb_Add < 0.0F);

  /* Logic: '<S66>/Logical Operator4' */
  rtb_Compare = ((!rtb_Compare) && (!rtb_Compare_in));

  /* Logic: '<S19>/Logical Operator2' */
  rtb_Compare_in = !rtb_Compare_in;

  /* UnitDelay: '<S19>/Unit Delay4' */
  WhlSpdRR_mps = VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_i;

  /* RelationalOperator: '<S19>/Relational Operator8' incorporates:
   *  Constant: '<S19>/Cal_DeltaV_mps1'
   */
  rtb_AND_l = (WhlSpdRR_mps > 235.0);

  /* Logic: '<S19>/Logical Operator1' */
  rtb_Compare_in = (rtb_Compare_in && rtb_AND_l);

  /* Switch: '<S67>/Switch6' incorporates:
   *  Constant: '<S67>/Reset'
   */
  if (rtb_Compare_in) {
    /* Sum: '<S67>/Add10' incorporates:
     *  Constant: '<S67>/Steptime'
     */
    rtb_Switch2_mn = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_n5 + 0.01F;
  } else {
    rtb_Switch2_mn = 0.0F;
  }

  /* End of Switch: '<S67>/Switch6' */

  /* MinMax: '<S67>/Min' incorporates:
   *  Constant: '<S19>/ResetDelay'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_n5 = fminf(rtb_Switch2_mn, 0.1F);

  /* RelationalOperator: '<S67>/Relational Operator9' incorporates:
   *  Constant: '<S19>/ResetDelay'
   *  UnitDelay: '<S67>/Unit Delay1'
   */
  rtb_Compare_in = (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_n5 >= 0.1F);

  /* UnitDelay: '<S19>/Unit Delay3' */
  rtb_AND_l = VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_e;

  /* Logic: '<S19>/Logical Operator3' */
  rtb_Compare_in = (rtb_Compare_in || rtb_AND_l);

  /* Logic: '<S66>/Logical Operator5' incorporates:
   *  UnitDelay: '<S66>/Unit Delay1'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_gl = ((!rtb_Compare) &&
    (!rtb_Compare_in));

  /* UnitDelay: '<S57>/Unit Delay1' */
  rtb_Compare = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_dp;

  /* Saturate: '<S18>/Saturation' */
  if (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k > 40.0F) {
    rtb_Switch2_mn = 40.0F;
  } else if (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k < 0.0F) {
    rtb_Switch2_mn = 0.0F;
  } else {
    rtb_Switch2_mn = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Saturate: '<S18>/Saturation' */

  /* Lookup_n-D: '<S18>/VehSpd_SlipTarget_mps' */
  rtb_MaxWhlSpd_mps_n = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240912_2018b_ConstP.pooled52,
    VehCtrlMdel240912_2018b_ConstP.pooled51, 3U);

  /* Sum: '<S18>/Add9' */
  rtb_MaxWhlSpd_mps_n += rtb_Switch2_mn;

  /* Saturate: '<S18>/Saturation1' */
  if (rtb_Brk_F < 0.0F) {
    rtb_Brk_F = 0.0F;
  }

  /* End of Saturate: '<S18>/Saturation1' */

  /* Sum: '<S18>/Add1' */
  rtb_Add4_j = rtb_MaxWhlSpd_mps_n - rtb_Brk_F;

  /* RelationalOperator: '<S18>/Relational Operator7' incorporates:
   *  Constant: '<S18>/Cal_DeltaV_mps'
   */
  rtb_Compare_in = (rtb_Add4_j < 0.0F);

  /* Logic: '<S57>/Logical Operator4' */
  rtb_Compare = ((!rtb_Compare) && (!rtb_Compare_in));

  /* Logic: '<S18>/Logical Operator2' */
  rtb_Compare_in = !rtb_Compare_in;

  /* UnitDelay: '<S18>/Unit Delay4' */
  WhlSpdRR_mps = VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_l;

  /* RelationalOperator: '<S18>/Relational Operator8' incorporates:
   *  Constant: '<S18>/Cal_DeltaV_mps1'
   */
  rtb_AND_l = (WhlSpdRR_mps > 235.0);

  /* Logic: '<S18>/Logical Operator1' */
  rtb_Compare_in = (rtb_Compare_in && rtb_AND_l);

  /* Switch: '<S58>/Switch6' incorporates:
   *  Constant: '<S58>/Reset'
   */
  if (rtb_Compare_in) {
    /* Sum: '<S58>/Add10' incorporates:
     *  Constant: '<S58>/Steptime'
     */
    rtb_MaxWhlSpd_mps_n = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_h + 0.01F;
  } else {
    rtb_MaxWhlSpd_mps_n = 0.0F;
  }

  /* End of Switch: '<S58>/Switch6' */

  /* MinMax: '<S58>/Min' incorporates:
   *  Constant: '<S18>/ResetDelay'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_h = fminf(rtb_MaxWhlSpd_mps_n,
    0.1F);

  /* RelationalOperator: '<S58>/Relational Operator9' incorporates:
   *  Constant: '<S18>/ResetDelay'
   *  UnitDelay: '<S58>/Unit Delay1'
   */
  rtb_Compare_in = (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_h >= 0.1F);

  /* UnitDelay: '<S18>/Unit Delay3' */
  rtb_AND_l = VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_i;

  /* Logic: '<S18>/Logical Operator3' */
  rtb_Compare_in = (rtb_Compare_in || rtb_AND_l);

  /* Logic: '<S57>/Logical Operator5' incorporates:
   *  UnitDelay: '<S57>/Unit Delay1'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_dp = ((!rtb_Compare) &&
    (!rtb_Compare_in));

  /* Chart: '<S7>/Chart' */
  if (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_c7_VehCtrlMdel240912_ ==
      0U) {
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_active_c7_VehCtrlMdel240912_ = 1U;
    VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240912_2018b = 7U;
  } else {
    switch (VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240912_2018b)
    {
     case VehCtrlMdel240912_2018b_IN_A:
      VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240912_2018b = 1U;
      if (fabs(rtb_deltafalllimit_i4) > 0.5) {
        VehCtrlMdel240912_2018b_DW.a = atan(rtb_Add2 / rtb_deltafalllimit_i4) *
          180.0 / 3.1415926535897931;
      } else {
        VehCtrlMdel240912_2018b_DW.a = 0.0;
      }

      VehCtrlMdel240912_2018b_DW.b = rtb_Add2 * rtb_Add2 + rtb_deltafalllimit_i4
        * rtb_deltafalllimit_i4;
      VehCtrlMdel240912_2018b_DW.b = sqrt(VehCtrlMdel240912_2018b_DW.b);
      break;

     case VehCtrlMdel240912_2018b_IN_B:
      VehCtrlMdel240912_2018b_B.TCSR_Enable_OUT = 1.0;
      rtb_LogicalOperator3 = ((!(VehCtrlMdel240912_2018b_DW.DYC_flag != 0.0)) ||
        (rtb_deltafalllimit_iz > 30.0) || (VehCtrlMdel240912_2018b_DW.a > 30.0) ||
        (VehCtrlMdel240912_2018b_DW.a > -30.0) || (VehCtrlMdel240912_2018b_DW.b >
        3.0));
      if (rtb_LogicalOperator3) {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240912_2018b =
          6U;
      }
      break;

     case VehCtrlMdel240912_2018b_IN_C:
      VehCtrlMdel240912_2018b_B.TCSR_Enable_OUT = 0.0;
      rtb_LogicalOperator3 = ((VehCtrlMdel240912_2018b_DW.DYC_flag != 0.0) &&
        (rtb_deltafalllimit_iz <= 30.0) && (VehCtrlMdel240912_2018b_DW.a >= 30.0)
        && (VehCtrlMdel240912_2018b_DW.a <= -30.0) &&
        (VehCtrlMdel240912_2018b_DW.b <= 3.0));
      if (rtb_LogicalOperator3) {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240912_2018b =
          6U;
      }
      break;

     case VehCtrlMdel240_IN_DYC_Disenable:
      VehCtrlMdel240912_2018b_B.DYC_Enable_OUT = 0.0;
      break;

     case VehCtrlMdel240912_IN_DYC_Enable:
      VehCtrlMdel240912_2018b_B.DYC_Enable_OUT = 1.0;
      rtb_LogicalOperator3 = ((rtb_deltafalllimit_iz >= 50.0) ||
        (rtb_deltafalllimit_i4 >= 5.0) || (VehCtrlMdel240912_2018b_DW.b >= 5.0));
      if (rtb_LogicalOperator3) {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240912_2018b =
          7U;
      }
      break;

     case VehCtrlMdel2_IN_F_TVD_TCS_STATE:
      rtb_LogicalOperator3 = ((VehCtrlMdel240912_2018b_DW.DYC_flag != 0.0) &&
        (rtb_deltafalllimit_iz <= 30.0) && (VehCtrlMdel240912_2018b_DW.a >= 30.0)
        && (VehCtrlMdel240912_2018b_DW.a <= -30.0) &&
        (VehCtrlMdel240912_2018b_DW.b <= 3.0));
      if (rtb_LogicalOperator3) {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240912_2018b =
          2U;
        VehCtrlMdel240912_2018b_B.TCSR_Enable_OUT = 1.0;
      } else {
        rtb_LogicalOperator3 = ((!(VehCtrlMdel240912_2018b_DW.DYC_flag != 0.0)) ||
          (rtb_deltafalllimit_iz > 30.0) || (VehCtrlMdel240912_2018b_DW.a > 30.0)
          || (VehCtrlMdel240912_2018b_DW.a > -30.0) ||
          (VehCtrlMdel240912_2018b_DW.b > 3.0));
        if (rtb_LogicalOperator3) {
          VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240912_2018b =
            3U;
          VehCtrlMdel240912_2018b_B.TCSR_Enable_OUT = 0.0;
        }
      }
      break;

     case VehCtrlMdel240912__IN_InitState:
      VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240912_2018b = 4U;
      VehCtrlMdel240912_2018b_B.DYC_Enable_OUT = 0.0;
      VehCtrlMdel240912_2018b_DW.DYC_flag = 0.0;
      break;

     case VehCtrlMdel24_IN_TCSF_Disenable:
      VehCtrlMdel240912_2018b_B.TCSF_Enable_OUT = 0.0;
      break;

     case VehCtrlMdel24091_IN_TCSF_Enable:
      VehCtrlMdel240912_2018b_B.TCSF_Enable_OUT = 1.0;
      if (VehCtrlMdel240912_2018b_DW.b > 5.0) {
        VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240912_2018b =
          7U;
      }
      break;

     case VehCtrlMdel24_IN_TCSR_Disenable:
      VehCtrlMdel240912_2018b_B.TCSR_Enable_OUT = 0.0;
      break;

     default:
      /* case IN_TCSR_Enable: */
      VehCtrlMdel240912_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240912_2018b = 10U;
      VehCtrlMdel240912_2018b_B.TCSR_Enable_OUT = 0.0;
      break;
    }
  }

  /* End of Chart: '<S7>/Chart' */

  /* Logic: '<S7>/Logical Operator2' */
  rtb_Compare = !VehCtrlMdel240912_2018b_B.VehReady;

  /* Logic: '<S7>/Logical Operator3' */
  rtb_Compare_in = (rtb_Compare || (Trq_CUT != 0.0));

  /* Logic: '<S7>/Logical Operator1' */
  rtb_LogicalOperator3 = ((VehCtrlMdel240912_2018b_B.Exit_d != 0.0) ||
    rtb_Compare_in);

  /* Switch: '<S7>/Switch2' incorporates:
   *  Constant: '<S7>/Constant4'
   *  Switch: '<S7>/Switch5'
   */
  if (rtb_LogicalOperator3) {
    rtb_VxIMU_est = 0.0F;
  } else {
    if (VehCtrlMdel240912_2018b_B.TCSR_Enable_OUT != 0.0) {
      /* Abs: '<S20>/Abs' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_deltafalllimit_iz = fabs(rtb_Yk1_l);

      /* RelationalOperator: '<S73>/Compare' incorporates:
       *  Constant: '<S73>/Constant'
       *  Switch: '<S7>/Switch5'
       */
      rtb_Compare = (rtb_deltafalllimit_iz <= 20.0);

      /* RelationalOperator: '<S75>/Compare' incorporates:
       *  Constant: '<S75>/Constant'
       *  Switch: '<S7>/Switch5'
       */
      rtb_AND_l = (rtb_VxIMU_est > 0.0F);

      /* Logic: '<S20>/Logical Operator6' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_AND_l = (rtb_AND_l && rtb_Compare);

      /* Switch: '<S20>/Switch5' incorporates:
       *  Switch: '<S7>/Switch5'
       *  UnitDelay: '<S20>/Unit Delay2'
       */
      if (rtb_AND_l) {
        elapseTime = VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_g;
      }

      /* End of Switch: '<S20>/Switch5' */
    }

    /* Gain: '<S7>/Gain6' */
    rtb_deltafalllimit_iz = 3.8461538461538463 * elapseTime;

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefRear' */
    rtb_VxIMU_est = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240912_2018b_ConstP.pooled32,
      VehCtrlMdel240912_2018b_ConstP.BrakeCompensateCoefRear_tableDa, 1U);

    /* RelationalOperator: '<S14>/LowerRelop1' */
    rtb_Compare = (rtb_deltafalllimit_iz > rtb_VxIMU_est);

    /* Switch: '<S14>/Switch2' */
    if (!rtb_Compare) {
      /* RelationalOperator: '<S14>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      rtb_Compare = (rtb_deltafalllimit_iz < 0.0);

      /* Switch: '<S14>/Switch' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      if (rtb_Compare) {
        rtb_VxIMU_est = 0.0F;
      } else {
        rtb_VxIMU_est = (real32_T)rtb_deltafalllimit_iz;
      }

      /* End of Switch: '<S14>/Switch' */
    }

    /* End of Switch: '<S14>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch2' */

  /* UnitDelay: '<S11>/Delay Input2'
   *
   * Block description for '<S11>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_MaxWhlSpd_mps_n = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_cd;

  /* Sum: '<S11>/Difference Inputs1'
   *
   * Block description for '<S11>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1_ea = rtb_VxIMU_est - rtb_MaxWhlSpd_mps_n;

  /* SampleTimeMath: '<S11>/sample time'
   *
   * About '<S11>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S11>/delta rise limit' */
  rtb_VxIMU_est = (real32_T)(25000.0 * elapseTime);

  /* RelationalOperator: '<S51>/LowerRelop1' */
  rtb_Compare = (rtb_UkYk1_ea > rtb_VxIMU_est);

  /* Switch: '<S51>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S11>/delta fall limit' */
    rtb_VxIMU_est = (real32_T)(-25000.0 * elapseTime);

    /* RelationalOperator: '<S51>/UpperRelop' */
    rtb_Compare = (rtb_UkYk1_ea < rtb_VxIMU_est);

    /* Switch: '<S51>/Switch' */
    if (rtb_Compare) {
      rtb_UkYk1_ea = rtb_VxIMU_est;
    }

    /* End of Switch: '<S51>/Switch' */
    rtb_VxIMU_est = rtb_UkYk1_ea;
  }

  /* End of Switch: '<S51>/Switch2' */

  /* Saturate: '<S7>/Saturation1' incorporates:
   *  Sum: '<S11>/Difference Inputs2'
   *  UnitDelay: '<S11>/Delay Input2'
   *
   * Block description for '<S11>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S11>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_cd = rtb_VxIMU_est +
    rtb_MaxWhlSpd_mps_n;
  if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_cd > 1000.0F) {
    VehCtrlMdel240912_2018b_B.TrqR_cmd = 1000.0F;
  } else if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_cd < 0.0F) {
    VehCtrlMdel240912_2018b_B.TrqR_cmd = 0.0F;
  } else {
    VehCtrlMdel240912_2018b_B.TrqR_cmd =
      VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_cd;
  }

  /* End of Saturate: '<S7>/Saturation1' */

  /* Logic: '<S7>/Logical Operator4' */
  rtb_Compare = ((VehCtrlMdel240912_2018b_B.Exit_g != 0.0) || rtb_Compare_in ||
                 rtb_LogicalOperator3);

  /* UnitDelay: '<S29>/Delay Input2'
   *
   * Block description for '<S29>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_mb;

  /* SampleTimeMath: '<S29>/sample time'
   *
   * About '<S29>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S29>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  WhlSpdRL_mps = 2000.0 * elapseTime;

  /* RelationalOperator: '<S36>/LowerRelop1' */
  rtb_Compare_in = (FLWhlStrAng > rtb_Switch2_b0);

  /* Switch: '<S36>/Switch2' */
  if (rtb_Compare_in) {
    rtb_Gain5 = rtb_Switch2_b0;
  } else {
    /* RelationalOperator: '<S36>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    rtb_Compare_in = (FLWhlStrAng < 0.0);

    /* Switch: '<S36>/Switch' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    if (rtb_Compare_in) {
      FLWhlStrAng = 0.0;
    }

    /* End of Switch: '<S36>/Switch' */
    rtb_Gain5 = FLWhlStrAng;
  }

  /* End of Switch: '<S36>/Switch2' */

  /* UnitDelay: '<S33>/Delay Input2'
   *
   * Block description for '<S33>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_VxIMU_est = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_fo;

  /* SampleTimeMath: '<S33>/sample time'
   *
   * About '<S33>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_StrWhlAngV = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S33>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant34'
   */
  rtb_MaxWhlSpd_mps_n = (real32_T)(1000.0 * rtb_StrWhlAngV);

  /* Logic: '<S10>/AND2' */
  rtb_Compare_in = (rtb_Compare_am && rtb_LowerRelop1_b);

  /* Logic: '<S10>/OR2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_Compare_in = (VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_l ||
                    rtb_Compare_in);

  /* Switch: '<S10>/Switch7' incorporates:
   *  Constant: '<S10>/Constant39'
   */
  if (rtb_Compare_in) {
    rtb_Add10 = 0.0F;
  } else {
    /* Logic: '<S10>/NOT1' */
    rtb_Compare_am = !rtb_Compare_am;

    /* Switch: '<S10>/Switch8' incorporates:
     *  Constant: '<S10>/Constant40'
     */
    if (!rtb_Compare_am) {
      rtb_Add10 = 0.0F;
    }

    /* End of Switch: '<S10>/Switch8' */
  }

  /* End of Switch: '<S10>/Switch7' */

  /* Saturate: '<S10>/Saturation2' */
  if (rtb_Add10 > 100.0F) {
    rtb_Add10 = 100.0F;
  } else {
    if (rtb_Add10 < 0.0F) {
      rtb_Add10 = 0.0F;
    }
  }

  /* End of Saturate: '<S10>/Saturation2' */

  /* Sum: '<S33>/Difference Inputs1'
   *
   * Block description for '<S33>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add10 -= rtb_VxIMU_est;

  /* RelationalOperator: '<S48>/LowerRelop1' */
  rtb_Compare_in = (rtb_Add10 > rtb_MaxWhlSpd_mps_n);

  /* Switch: '<S48>/Switch2' */
  if (!rtb_Compare_in) {
    /* Product: '<S33>/delta fall limit' */
    rtb_MaxWhlSpd_mps_n = (real32_T)(-1000.0 * rtb_StrWhlAngV);

    /* RelationalOperator: '<S48>/UpperRelop' */
    rtb_Compare_am = (rtb_Add10 < rtb_MaxWhlSpd_mps_n);

    /* Switch: '<S48>/Switch' */
    if (rtb_Compare_am) {
      rtb_Add10 = rtb_MaxWhlSpd_mps_n;
    }

    /* End of Switch: '<S48>/Switch' */
    rtb_MaxWhlSpd_mps_n = rtb_Add10;
  }

  /* End of Switch: '<S48>/Switch2' */

  /* Sum: '<S33>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S33>/Delay Input2'
   *
   * Block description for '<S33>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S33>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_fo = rtb_MaxWhlSpd_mps_n +
    rtb_VxIMU_est;

  /* Gain: '<S10>/Gain20' incorporates:
   *  UnitDelay: '<S33>/Delay Input2'
   *
   * Block description for '<S33>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = -VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_fo;

  /* Sum: '<S10>/Add11' */
  rtb_deltafalllimit_iz = rtb_Gain5 + rtb_Add10;

  /* RelationalOperator: '<S39>/LowerRelop1' */
  rtb_Compare_in = (rtb_deltafalllimit_iz > rtb_Switch2_b0);

  /* Switch: '<S39>/Switch2' */
  if (rtb_Compare_in) {
    rtb_deltafalllimit_iz = rtb_Switch2_b0;
  } else {
    /* RelationalOperator: '<S39>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    rtb_Compare_am = (rtb_deltafalllimit_iz < 0.0);

    /* Switch: '<S39>/Switch' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    if (rtb_Compare_am) {
      rtb_deltafalllimit_iz = 0.0;
    }

    /* End of Switch: '<S39>/Switch' */
  }

  /* End of Switch: '<S39>/Switch2' */

  /* Sum: '<S29>/Difference Inputs1'
   *
   * Block description for '<S29>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_iz -= WhlSpdRR_mps;

  /* RelationalOperator: '<S44>/LowerRelop1' */
  rtb_Compare_in = (rtb_deltafalllimit_iz > WhlSpdRL_mps);

  /* Switch: '<S44>/Switch2' */
  if (!rtb_Compare_in) {
    /* Product: '<S29>/delta fall limit' */
    rtb_Add2 = -2000.0 * elapseTime;

    /* RelationalOperator: '<S44>/UpperRelop' */
    rtb_Compare_am = (rtb_deltafalllimit_iz < rtb_Add2);

    /* Switch: '<S44>/Switch' */
    if (rtb_Compare_am) {
      rtb_deltafalllimit_iz = rtb_Add2;
    }

    /* End of Switch: '<S44>/Switch' */
    WhlSpdRL_mps = rtb_deltafalllimit_iz;
  }

  /* End of Switch: '<S44>/Switch2' */

  /* Sum: '<S29>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S29>/Delay Input2'
   *
   * Block description for '<S29>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S29>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_mb = WhlSpdRL_mps + WhlSpdRR_mps;

  /* Sum: '<S17>/Add4' incorporates:
   *  Constant: '<S17>/Constant'
   */
  WhlSpdRR_mps = 1.0 - WhlSpdFL;

  /* Product: '<S17>/Product4' */
  WhlSpdRL_mps = 7.0 * WhlSpdRR_mps;

  /* Product: '<S17>/Product1' */
  rtb_Gain5 = WhlSpdRL_mps * 9550.0;

  /* Sum: '<S17>/Add2' incorporates:
   *  Constant: '<S17>/RPM_min2'
   */
  WhlSpdRL_mps = VehCtrlMdel240912_2018b_B.MCFR_ActualVelocity_k + 10.0;

  /* MinMax: '<S17>/Max1' incorporates:
   *  Constant: '<S17>/RPM_min3'
   */
  rtb_Gain4 = fmax(WhlSpdRL_mps, 1.0);

  /* Product: '<S17>/Divide1' */
  rtb_Gain5 /= rtb_Gain4;

  /* MinMax: '<S7>/MinMax' incorporates:
   *  UnitDelay: '<S29>/Delay Input2'
   *
   * Block description for '<S29>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_deltafalllimit_iz = fmin(rtb_Gain5,
    VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_mb);

  /* Switch: '<S19>/Switch6' incorporates:
   *  Constant: '<S19>/Verror_Reset'
   *  UnitDelay: '<S66>/Unit Delay1'
   */
  if (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_gl) {
    rtb_VxIMU_est = rtb_Add;
  } else {
    rtb_VxIMU_est = 0.0F;
  }

  /* End of Switch: '<S19>/Switch6' */

  /* Product: '<S19>/Product' incorporates:
   *  Constant: '<S19>/P_Gain'
   */
  rtb_Switch2_b0 = rtb_VxIMU_est * 40.0F;

  /* Sum: '<S19>/Add11' */
  WhlSpdRL_mps = rtb_deltafalllimit_iz - rtb_Switch2_b0;

  /* UnitDelay: '<S19>/Unit Delay5' */
  rtb_MaxWhlSpd_mps_n = VehCtrlMdel240912_2018b_DW.UnitDelay5_DSTATE_i;

  /* Product: '<S19>/Product2' */
  rtb_MaxWhlSpd_mps_n *= rtb_Add;

  /* RelationalOperator: '<S63>/Compare' incorporates:
   *  Constant: '<S63>/Constant'
   */
  rtb_Compare_in = (rtb_MaxWhlSpd_mps_n <= 0.0F);

  /* UnitDelay: '<S19>/Unit Delay' */
  rtb_MaxWhlSpd_mps_n = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_f;

  /* Switch: '<S19>/Switch3' incorporates:
   *  Constant: '<S19>/Verror_Reset1'
   */
  if (rtb_Compare_in) {
    rtb_MaxWhlSpd_mps_n = 0.0F;
  }

  /* End of Switch: '<S19>/Switch3' */

  /* Sum: '<S19>/Add2' */
  rtb_MaxWhlSpd_mps_n += rtb_VxIMU_est;

  /* Saturate: '<S19>/Saturation2' */
  if (rtb_MaxWhlSpd_mps_n > 400.0F) {
    VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_f = 400.0F;
  } else if (rtb_MaxWhlSpd_mps_n < -100.0F) {
    VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_f = -100.0F;
  } else {
    VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_f = rtb_MaxWhlSpd_mps_n;
  }

  /* End of Saturate: '<S19>/Saturation2' */

  /* RelationalOperator: '<S71>/Compare' incorporates:
   *  UnitDelay: '<S66>/Unit Delay1'
   */
  rtb_Compare_am = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_gl;

  /* UnitDelay: '<S65>/Delay Input1'
   *
   * Block description for '<S65>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_Compare_in = VehCtrlMdel240912_2018b_DW.DelayInput1_DSTATE_j;

  /* RelationalOperator: '<S65>/FixPt Relational Operator' */
  rtb_Compare_in = ((int32_T)rtb_Compare_am > (int32_T)rtb_Compare_in);

  /* Switch: '<S19>/Switch' incorporates:
   *  Constant: '<S19>/Integr_StartPoint'
   */
  if (rtb_Compare_in) {
    /* Sum: '<S19>/Add4' */
    rtb_Gain5 = rtb_deltafalllimit_iz -
      VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_f;
  } else {
    rtb_Gain5 = 0.0;
  }

  /* End of Switch: '<S19>/Switch' */

  /* Lookup_n-D: '<S19>/VehicleStableTarget_mps' */
  rtb_VxIMU_est = look1_iflf_binlc(rtb_Ax,
    VehCtrlMdel240912_2018b_ConstP.pooled52,
    VehCtrlMdel240912_2018b_ConstP.pooled59, 3U);

  /* Sum: '<S19>/Add5' */
  rtb_VxIMU_est += rtb_Ax;

  /* Sum: '<S19>/Add10' */
  rtb_VxIMU_est = rtb_Add6_p - rtb_VxIMU_est;

  /* RelationalOperator: '<S19>/Relational Operator' incorporates:
   *  Constant: '<S19>/Verror'
   */
  rtb_Compare_in = (rtb_VxIMU_est < 0.0F);

  /* Logic: '<S19>/Logical Operator4' incorporates:
   *  UnitDelay: '<S66>/Unit Delay1'
   */
  rtb_Compare_in = (rtb_Compare_in &&
                    VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_gl);

  /* Switch: '<S19>/Switch1' incorporates:
   *  Constant: '<S19>/Trq_IReset'
   *  Constant: '<S19>/Trq_I_FF'
   */
  if (rtb_Compare_in) {
    rtb_VxIMU_est = 20.0F;
  } else {
    rtb_VxIMU_est = 0.0F;
  }

  /* End of Switch: '<S19>/Switch1' */

  /* Sum: '<S19>/Add6' incorporates:
   *  UnitDelay: '<S19>/Unit Delay'
   */
  rtb_Gain4 = (VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_f + rtb_Gain5) +
    rtb_VxIMU_est;

  /* Product: '<S19>/Product1' */
  rtb_Add2 = rtb_Gain4 * 10.0;

  /* RelationalOperator: '<S68>/LowerRelop1' */
  rtb_Compare_in = (rtb_Add2 > WhlSpdRL_mps);

  /* Switch: '<S68>/Switch2' */
  if (!rtb_Compare_in) {
    /* Gain: '<S19>/Gain3' */
    rtb_VxIMU_est = -rtb_Switch2_b0;

    /* RelationalOperator: '<S68>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_Add2 < rtb_VxIMU_est);

    /* Switch: '<S68>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_Add2 = rtb_VxIMU_est;
    }

    /* End of Switch: '<S68>/Switch' */
    WhlSpdRL_mps = rtb_Add2;
  }

  /* End of Switch: '<S68>/Switch2' */

  /* Sum: '<S19>/Add7' incorporates:
   *  UnitDelay: '<S19>/Unit Delay4'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_i = rtb_Switch2_b0 + WhlSpdRL_mps;

  /* Lookup_n-D: '<S19>/VehicleStableTarget_mps1' */
  rtb_VxIMU_est = look1_iflf_binlc(rtb_Ax,
    VehCtrlMdel240912_2018b_ConstP.pooled52,
    VehCtrlMdel240912_2018b_ConstP.pooled59, 3U);

  /* Sum: '<S19>/Add13' */
  rtb_Ax += rtb_VxIMU_est;

  /* Sum: '<S19>/Add12' */
  rtb_Add6_p -= rtb_Ax;

  /* RelationalOperator: '<S19>/Relational Operator1' incorporates:
   *  Constant: '<S19>/Verror1'
   */
  rtb_Compare_in = (rtb_Add6_p < 0.0F);

  /* RelationalOperator: '<S19>/Relational Operator2' incorporates:
   *  UnitDelay: '<S19>/Unit Delay4'
   */
  rtb_AND_l = (VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_i >=
               rtb_deltafalllimit_iz);

  /* RelationalOperator: '<S64>/Compare' incorporates:
   *  Constant: '<S64>/Constant'
   *  UnitDelay: '<S19>/Unit Delay4'
   */
  rtb_AND2 = (VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_i <= 0.01);

  /* Logic: '<S19>/OR' */
  rtb_AND_l = (rtb_AND_l || rtb_AND2);

  /* Logic: '<S19>/Logical Operator5' incorporates:
   *  UnitDelay: '<S19>/Unit Delay3'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_e = (rtb_Compare_in && rtb_AND_l);

  /* Switch: '<S19>/Switch2' incorporates:
   *  Switch: '<S19>/Switch7'
   *  UnitDelay: '<S19>/Unit Delay3'
   *  UnitDelay: '<S66>/Unit Delay1'
   */
  if (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_gl) {
    /* RelationalOperator: '<S69>/LowerRelop1' incorporates:
     *  Constant: '<S19>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S19>/Unit Delay4'
     */
    rtb_LowerRelop1_b = (VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_i > 235.0);

    /* Switch: '<S69>/Switch2' incorporates:
     *  Constant: '<S19>/TCS_TrqRequest_Max2'
     */
    if (rtb_LowerRelop1_b) {
      VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_j = 235.0;
    } else {
      /* RelationalOperator: '<S69>/UpperRelop' incorporates:
       *  Constant: '<S19>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S19>/Unit Delay4'
       */
      rtb_LowerRelop1_b = (VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_i < 0.0);

      /* Switch: '<S69>/Switch' incorporates:
       *  Constant: '<S19>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S19>/Unit Delay4'
       */
      if (rtb_LowerRelop1_b) {
        VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_j = 0.0;
      } else {
        VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_j =
          VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_i;
      }

      /* End of Switch: '<S69>/Switch' */
    }

    /* End of Switch: '<S69>/Switch2' */

    /* RelationalOperator: '<S70>/LowerRelop1' */
    rtb_LowerRelop1_b = (rtb_deltafalllimit_iz >
                         VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_j);

    /* Switch: '<S70>/Switch2' */
    if (!rtb_LowerRelop1_b) {
      /* RelationalOperator: '<S70>/UpperRelop' incorporates:
       *  Constant: '<S19>/TCS_TrqRequest_Min1'
       */
      rtb_LowerRelop1_b = (rtb_deltafalllimit_iz < 0.0);

      /* Switch: '<S70>/Switch' incorporates:
       *  Constant: '<S19>/TCS_TrqRequest_Min1'
       */
      if (rtb_LowerRelop1_b) {
        VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_j = 0.0;
      } else {
        VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_j = rtb_deltafalllimit_iz;
      }

      /* End of Switch: '<S70>/Switch' */
    }

    /* End of Switch: '<S70>/Switch2' */
  } else {
    if (VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_e) {
      /* Switch: '<S19>/Switch7' */
      VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_j = rtb_deltafalllimit_iz;
    }
  }

  /* End of Switch: '<S19>/Switch2' */

  /* Switch: '<S7>/Switch3' incorporates:
   *  Constant: '<S7>/Constant6'
   *  Switch: '<S7>/Switch6'
   */
  if (rtb_Compare) {
    rtb_Add6_p = 0.0F;
  } else {
    if (VehCtrlMdel240912_2018b_B.TCSF_Enable_OUT != 0.0) {
      /* Switch: '<S7>/Switch6' incorporates:
       *  UnitDelay: '<S19>/Unit Delay2'
       */
      rtb_deltafalllimit_iz = VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_j;
    }

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefFront1' */
    rtb_Add6_p = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240912_2018b_ConstP.pooled32,
      VehCtrlMdel240912_2018b_ConstP.pooled31, 1U);

    /* RelationalOperator: '<S15>/LowerRelop1' */
    rtb_LowerRelop1_b = (rtb_deltafalllimit_iz > rtb_Add6_p);

    /* Switch: '<S15>/Switch2' */
    if (!rtb_LowerRelop1_b) {
      /* RelationalOperator: '<S15>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      rtb_LowerRelop1_b = (rtb_deltafalllimit_iz < 0.0);

      /* Switch: '<S15>/Switch' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      if (rtb_LowerRelop1_b) {
        rtb_Add6_p = 0.0F;
      } else {
        rtb_Add6_p = (real32_T)rtb_deltafalllimit_iz;
      }

      /* End of Switch: '<S15>/Switch' */
    }

    /* End of Switch: '<S15>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch3' */

  /* UnitDelay: '<S12>/Delay Input2'
   *
   * Block description for '<S12>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_VxIMU_est = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hn;

  /* Sum: '<S12>/Difference Inputs1'
   *
   * Block description for '<S12>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add6_p -= rtb_VxIMU_est;

  /* SampleTimeMath: '<S12>/sample time'
   *
   * About '<S12>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S12>/delta rise limit' */
  rtb_MaxWhlSpd_mps_n = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S52>/LowerRelop1' */
  rtb_Compare_in = (rtb_Add6_p > rtb_MaxWhlSpd_mps_n);

  /* Switch: '<S52>/Switch2' */
  if (!rtb_Compare_in) {
    /* Product: '<S12>/delta fall limit' */
    rtb_Ax = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S52>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_Add6_p < rtb_Ax);

    /* Switch: '<S52>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_Add6_p = rtb_Ax;
    }

    /* End of Switch: '<S52>/Switch' */
    rtb_MaxWhlSpd_mps_n = rtb_Add6_p;
  }

  /* End of Switch: '<S52>/Switch2' */

  /* Saturate: '<S7>/Saturation2' incorporates:
   *  Sum: '<S12>/Difference Inputs2'
   *  UnitDelay: '<S12>/Delay Input2'
   *
   * Block description for '<S12>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S12>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hn = rtb_MaxWhlSpd_mps_n +
    rtb_VxIMU_est;
  if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hn > 20.0F) {
    VehCtrlMdel240912_2018b_B.TrqFR_cmd = 20.0F;
  } else if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hn < 0.0F) {
    VehCtrlMdel240912_2018b_B.TrqFR_cmd = 0.0F;
  } else {
    VehCtrlMdel240912_2018b_B.TrqFR_cmd =
      VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_hn;
  }

  /* End of Saturate: '<S7>/Saturation2' */

  /* Product: '<S17>/Product3' */
  WhlSpdRL_mps = 7.0 * WhlSpdRR_mps;

  /* Product: '<S17>/Product2' */
  WhlSpdRR_mps = WhlSpdRL_mps * 9550.0;

  /* Sum: '<S17>/Add3' incorporates:
   *  Constant: '<S17>/RPM_min4'
   */
  WhlSpdRL_mps = VehCtrlMdel240912_2018b_B.MCFL_ActualVelocity_p + 10.0;

  /* MinMax: '<S17>/Max2' incorporates:
   *  Constant: '<S17>/RPM_min5'
   */
  rtb_Gain5 = fmax(WhlSpdRL_mps, 1.0);

  /* Product: '<S17>/Divide2' */
  WhlSpdRR_mps /= rtb_Gain5;

  /* UnitDelay: '<S30>/Delay Input2'
   *
   * Block description for '<S30>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRL_mps = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_o;

  /* SampleTimeMath: '<S30>/sample time'
   *
   * About '<S30>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S30>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  rtb_Gain5 = 2000.0 * elapseTime;

  /* Gain: '<S10>/Gain14' */
  rtb_Gain4 = 0.017453292519943295 * rtb_UkYk1;

  /* Trigonometry: '<S10>/Cos2' */
  rtb_Gain4 = cos(rtb_Gain4);

  /* Gain: '<S10>/Gain13' */
  rtb_Gain4 *= 1.2;

  /* Sum: '<S10>/Add8' incorporates:
   *  Constant: '<S10>/Constant28'
   */
  rtb_g_mpss1 = 90.0 - rtb_UkYk1;

  /* Gain: '<S10>/Gain15' */
  rtb_g_mpss1 *= 0.017453292519943295;

  /* Trigonometry: '<S10>/Cos3' */
  rtb_g_mpss1 = cos(rtb_g_mpss1);

  /* Gain: '<S10>/Gain12' */
  rtb_g_mpss1 *= 1.522;

  /* Sum: '<S10>/Add9' */
  rtb_Gain4 += rtb_g_mpss1;

  /* Product: '<S10>/Product2' incorporates:
   *  UnitDelay: '<S32>/Delay Input2'
   *
   * Block description for '<S32>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain4 *= VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_jk;

  /* Saturate: '<S10>/Saturation4' */
  if (rtb_Gain4 > 20.0) {
    rtb_Gain4 = 20.0;
  } else {
    if (rtb_Gain4 < -20.0) {
      rtb_Gain4 = -20.0;
    }
  }

  /* End of Saturate: '<S10>/Saturation4' */

  /* Sum: '<S10>/Add5' */
  rtb_Gain4 = Acc_POS - rtb_Gain4;

  /* RelationalOperator: '<S37>/LowerRelop1' */
  rtb_Compare_in = (rtb_Gain4 > rtb_Add7);

  /* Switch: '<S37>/Switch2' */
  if (rtb_Compare_in) {
    rtb_Gain4 = rtb_Add7;
  } else {
    /* RelationalOperator: '<S37>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    rtb_LowerRelop1_b = (rtb_Gain4 < 0.0);

    /* Switch: '<S37>/Switch' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    if (rtb_LowerRelop1_b) {
      rtb_Gain4 = 0.0;
    }

    /* End of Switch: '<S37>/Switch' */
  }

  /* End of Switch: '<S37>/Switch2' */

  /* Sum: '<S10>/Add12' */
  rtb_UkYk1 = rtb_Gain4 + rtb_Add10;

  /* RelationalOperator: '<S40>/LowerRelop1' */
  rtb_Compare_in = (rtb_UkYk1 > rtb_Add7);

  /* Switch: '<S40>/Switch2' */
  if (rtb_Compare_in) {
    rtb_UkYk1 = rtb_Add7;
  } else {
    /* RelationalOperator: '<S40>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    rtb_LowerRelop1_b = (rtb_UkYk1 < 0.0);

    /* Switch: '<S40>/Switch' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    if (rtb_LowerRelop1_b) {
      rtb_UkYk1 = 0.0;
    }

    /* End of Switch: '<S40>/Switch' */
  }

  /* End of Switch: '<S40>/Switch2' */

  /* Sum: '<S30>/Difference Inputs1'
   *
   * Block description for '<S30>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 -= WhlSpdRL_mps;

  /* RelationalOperator: '<S45>/LowerRelop1' */
  rtb_Compare_in = (rtb_UkYk1 > rtb_Gain5);

  /* Switch: '<S45>/Switch2' */
  if (!rtb_Compare_in) {
    /* Product: '<S30>/delta fall limit' */
    rtb_deltafalllimit_iz = -2000.0 * elapseTime;

    /* RelationalOperator: '<S45>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S45>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S45>/Switch' */
    rtb_Gain5 = rtb_UkYk1;
  }

  /* End of Switch: '<S45>/Switch2' */

  /* Sum: '<S30>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S30>/Delay Input2'
   *
   * Block description for '<S30>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S30>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_o = rtb_Gain5 + WhlSpdRL_mps;

  /* MinMax: '<S7>/MinMax1' incorporates:
   *  UnitDelay: '<S30>/Delay Input2'
   *
   * Block description for '<S30>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = fmin(WhlSpdRR_mps, VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_o);

  /* Switch: '<S18>/Switch6' incorporates:
   *  Constant: '<S18>/Verror_Reset'
   *  UnitDelay: '<S57>/Unit Delay1'
   */
  if (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_dp) {
    rtb_Add10 = rtb_Add4_j;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S18>/Switch6' */

  /* Product: '<S18>/Product' incorporates:
   *  Constant: '<S18>/P_Gain'
   */
  Acc_POS = rtb_Add10 * 40.0F;

  /* Sum: '<S18>/Add11' */
  WhlSpdRR_mps = rtb_UkYk1 - Acc_POS;

  /* UnitDelay: '<S18>/Unit Delay5' */
  rtb_VxIMU_est = VehCtrlMdel240912_2018b_DW.UnitDelay5_DSTATE_ip;

  /* Product: '<S18>/Product2' */
  rtb_VxIMU_est *= rtb_Add4_j;

  /* RelationalOperator: '<S54>/Compare' incorporates:
   *  Constant: '<S54>/Constant'
   */
  rtb_Compare_in = (rtb_VxIMU_est <= 0.0F);

  /* UnitDelay: '<S18>/Unit Delay' */
  rtb_VxIMU_est = VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_nr;

  /* Switch: '<S18>/Switch3' incorporates:
   *  Constant: '<S18>/Verror_Reset1'
   */
  if (rtb_Compare_in) {
    rtb_VxIMU_est = 0.0F;
  }

  /* End of Switch: '<S18>/Switch3' */

  /* Sum: '<S18>/Add2' */
  rtb_VxIMU_est += rtb_Add10;

  /* Saturate: '<S18>/Saturation2' */
  if (rtb_VxIMU_est > 400.0F) {
    VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_nr = 400.0F;
  } else if (rtb_VxIMU_est < -100.0F) {
    VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_nr = -100.0F;
  } else {
    VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_nr = rtb_VxIMU_est;
  }

  /* End of Saturate: '<S18>/Saturation2' */

  /* RelationalOperator: '<S62>/Compare' incorporates:
   *  UnitDelay: '<S57>/Unit Delay1'
   */
  rtb_LowerRelop1_b = VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_dp;

  /* UnitDelay: '<S56>/Delay Input1'
   *
   * Block description for '<S56>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_Compare_in = VehCtrlMdel240912_2018b_DW.DelayInput1_DSTATE_b;

  /* RelationalOperator: '<S56>/FixPt Relational Operator' */
  rtb_Compare_in = ((int32_T)rtb_LowerRelop1_b > (int32_T)rtb_Compare_in);

  /* Switch: '<S18>/Switch' incorporates:
   *  Constant: '<S18>/Integr_StartPoint'
   */
  if (rtb_Compare_in) {
    /* Sum: '<S18>/Add4' */
    WhlSpdRL_mps = rtb_UkYk1 - VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_g4;
  } else {
    WhlSpdRL_mps = 0.0;
  }

  /* End of Switch: '<S18>/Switch' */

  /* Lookup_n-D: '<S18>/VehicleStableTarget_mps' */
  rtb_Add10 = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240912_2018b_ConstP.pooled52,
    VehCtrlMdel240912_2018b_ConstP.pooled59, 3U);

  /* Sum: '<S18>/Add5' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S18>/Add10' */
  rtb_Add10 = rtb_Brk_F - rtb_Add10;

  /* RelationalOperator: '<S18>/Relational Operator' incorporates:
   *  Constant: '<S18>/Verror'
   */
  rtb_Compare_in = (rtb_Add10 < 0.0F);

  /* Logic: '<S18>/Logical Operator4' incorporates:
   *  UnitDelay: '<S57>/Unit Delay1'
   */
  rtb_Compare_in = (rtb_Compare_in &&
                    VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_dp);

  /* Switch: '<S18>/Switch1' incorporates:
   *  Constant: '<S18>/Trq_IReset'
   *  Constant: '<S18>/Trq_I_FF'
   */
  if (rtb_Compare_in) {
    rtb_Add10 = 20.0F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S18>/Switch1' */

  /* Sum: '<S18>/Add6' incorporates:
   *  UnitDelay: '<S18>/Unit Delay'
   */
  rtb_Gain5 = (VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_nr + WhlSpdRL_mps) +
    rtb_Add10;

  /* Product: '<S18>/Product1' */
  rtb_deltafalllimit_iz = rtb_Gain5 * 10.0;

  /* RelationalOperator: '<S59>/LowerRelop1' */
  rtb_Compare_in = (rtb_deltafalllimit_iz > WhlSpdRR_mps);

  /* Switch: '<S59>/Switch2' */
  if (!rtb_Compare_in) {
    /* Gain: '<S18>/Gain3' */
    rtb_Add6_p = -Acc_POS;

    /* RelationalOperator: '<S59>/UpperRelop' */
    rtb_Compare_in = (rtb_deltafalllimit_iz < rtb_Add6_p);

    /* Switch: '<S59>/Switch' */
    if (rtb_Compare_in) {
      rtb_deltafalllimit_iz = rtb_Add6_p;
    }

    /* End of Switch: '<S59>/Switch' */
    WhlSpdRR_mps = rtb_deltafalllimit_iz;
  }

  /* End of Switch: '<S59>/Switch2' */

  /* Sum: '<S18>/Add7' incorporates:
   *  UnitDelay: '<S18>/Unit Delay4'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_l = Acc_POS + WhlSpdRR_mps;

  /* Lookup_n-D: '<S18>/VehicleStableTarget_mps1' */
  rtb_Add10 = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240912_2018b_ConstP.pooled52,
    VehCtrlMdel240912_2018b_ConstP.pooled59, 3U);

  /* Sum: '<S18>/Add13' */
  rtb_Switch2_mn += rtb_Add10;

  /* Sum: '<S18>/Add12' */
  rtb_Brk_F -= rtb_Switch2_mn;

  /* RelationalOperator: '<S18>/Relational Operator1' incorporates:
   *  Constant: '<S18>/Verror1'
   */
  rtb_Compare_in = (rtb_Brk_F < 0.0F);

  /* RelationalOperator: '<S18>/Relational Operator2' incorporates:
   *  UnitDelay: '<S18>/Unit Delay4'
   */
  rtb_AND_l = (VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_l >= rtb_UkYk1);

  /* RelationalOperator: '<S55>/Compare' incorporates:
   *  Constant: '<S55>/Constant'
   *  UnitDelay: '<S18>/Unit Delay4'
   */
  rtb_AND2 = (VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_l <= 0.01);

  /* Logic: '<S18>/OR' */
  rtb_AND_l = (rtb_AND_l || rtb_AND2);

  /* Logic: '<S18>/Logical Operator5' */
  rtb_Compare_in = (rtb_Compare_in && rtb_AND_l);

  /* Switch: '<S18>/Switch2' incorporates:
   *  Switch: '<S18>/Switch7'
   *  UnitDelay: '<S57>/Unit Delay1'
   */
  if (VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_dp) {
    /* RelationalOperator: '<S60>/LowerRelop1' incorporates:
     *  Constant: '<S18>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S18>/Unit Delay4'
     */
    rtb_AND_l = (VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_l > 235.0);

    /* Switch: '<S60>/Switch2' incorporates:
     *  Constant: '<S18>/TCS_TrqRequest_Max2'
     */
    if (rtb_AND_l) {
      VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_b = 235.0;
    } else {
      /* RelationalOperator: '<S60>/UpperRelop' incorporates:
       *  Constant: '<S18>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S18>/Unit Delay4'
       */
      rtb_AND_l = (VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_l < 0.0);

      /* Switch: '<S60>/Switch' incorporates:
       *  Constant: '<S18>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S18>/Unit Delay4'
       */
      if (rtb_AND_l) {
        VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_b = 0.0;
      } else {
        VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_b =
          VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_l;
      }

      /* End of Switch: '<S60>/Switch' */
    }

    /* End of Switch: '<S60>/Switch2' */

    /* RelationalOperator: '<S61>/LowerRelop1' */
    rtb_AND_l = (rtb_UkYk1 > VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_b);

    /* Switch: '<S61>/Switch2' */
    if (!rtb_AND_l) {
      /* RelationalOperator: '<S61>/UpperRelop' incorporates:
       *  Constant: '<S18>/TCS_TrqRequest_Min1'
       */
      rtb_AND_l = (rtb_UkYk1 < 0.0);

      /* Switch: '<S61>/Switch' incorporates:
       *  Constant: '<S18>/TCS_TrqRequest_Min1'
       */
      if (rtb_AND_l) {
        VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_b = 0.0;
      } else {
        VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_b = rtb_UkYk1;
      }

      /* End of Switch: '<S61>/Switch' */
    }

    /* End of Switch: '<S61>/Switch2' */
  } else {
    if (rtb_Compare_in) {
      /* Switch: '<S18>/Switch7' */
      VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_b = rtb_UkYk1;
    }
  }

  /* End of Switch: '<S18>/Switch2' */

  /* Switch: '<S7>/Switch4' incorporates:
   *  Constant: '<S7>/Constant8'
   *  Switch: '<S7>/Switch7'
   */
  if (rtb_Compare) {
    rtb_Brk_F = 0.0F;
  } else {
    if (VehCtrlMdel240912_2018b_B.TCSF_Enable_OUT != 0.0) {
      /* Switch: '<S7>/Switch7' incorporates:
       *  UnitDelay: '<S18>/Unit Delay2'
       */
      rtb_UkYk1 = VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_b;
    }

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefFront2' */
    rtb_Brk_F = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240912_2018b_ConstP.pooled32,
      VehCtrlMdel240912_2018b_ConstP.pooled31, 1U);

    /* RelationalOperator: '<S16>/LowerRelop1' */
    rtb_Compare = (rtb_UkYk1 > rtb_Brk_F);

    /* Switch: '<S16>/Switch2' */
    if (!rtb_Compare) {
      /* RelationalOperator: '<S16>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      rtb_Compare = (rtb_UkYk1 < 0.0);

      /* Switch: '<S16>/Switch' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      if (rtb_Compare) {
        rtb_Brk_F = 0.0F;
      } else {
        rtb_Brk_F = (real32_T)rtb_UkYk1;
      }

      /* End of Switch: '<S16>/Switch' */
    }

    /* End of Switch: '<S16>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch4' */

  /* UnitDelay: '<S13>/Delay Input2'
   *
   * Block description for '<S13>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_ib;

  /* Sum: '<S13>/Difference Inputs1'
   *
   * Block description for '<S13>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Brk_F -= rtb_Add10;

  /* SampleTimeMath: '<S13>/sample time'
   *
   * About '<S13>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S13>/delta rise limit' */
  rtb_VxIMU_est = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S53>/LowerRelop1' */
  rtb_Compare = (rtb_Brk_F > rtb_VxIMU_est);

  /* Switch: '<S53>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S13>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S53>/UpperRelop' */
    rtb_Compare = (rtb_Brk_F < rtb_Add6_p);

    /* Switch: '<S53>/Switch' */
    if (rtb_Compare) {
      rtb_Brk_F = rtb_Add6_p;
    }

    /* End of Switch: '<S53>/Switch' */
    rtb_VxIMU_est = rtb_Brk_F;
  }

  /* End of Switch: '<S53>/Switch2' */

  /* Saturate: '<S7>/Saturation3' incorporates:
   *  Sum: '<S13>/Difference Inputs2'
   *  UnitDelay: '<S13>/Delay Input2'
   *
   * Block description for '<S13>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S13>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_ib = rtb_VxIMU_est + rtb_Add10;
  if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_ib > 20.0F) {
    VehCtrlMdel240912_2018b_B.TrqFL_cmd = 20.0F;
  } else if (VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_ib < 0.0F) {
    VehCtrlMdel240912_2018b_B.TrqFL_cmd = 0.0F;
  } else {
    VehCtrlMdel240912_2018b_B.TrqFL_cmd =
      VehCtrlMdel240912_2018b_DW.DelayInput2_DSTATE_ib;
  }

  /* End of Saturate: '<S7>/Saturation3' */

  /* Sum: '<S10>/Add3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay1'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_p = rtb_Switch2_on - rtb_UkYk1_ix;

  /* SampleTimeMath: '<S7>/Weighted Sample Time1'
   *
   * About '<S7>/Weighted Sample Time1':
   *  y = u * K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;
  rtb_Switch2_on = 420000.0 * elapseTime;

  /* Sum: '<S7>/Add2' */
  rtb_Switch2_on += RPM;

  /* Saturate: '<S7>/RPM_Saturation1' */
  if (rtb_Switch2_on > 5000.0) {
    rtb_Switch2_on = 5000.0;
  } else {
    if (rtb_Switch2_on < -50.0) {
      rtb_Switch2_on = -50.0;
    }
  }

  /* End of Saturate: '<S7>/RPM_Saturation1' */

  /* Update for UnitDelay: '<S7>/Unit Delay' */
  VehCtrlMdel240912_2018b_DW.UnitDelay_DSTATE_n =
    VehCtrlMdel240912_2018b_B.DYC_Enable_OUT;

  /* Update for UnitDelay: '<S10>/Unit Delay2' */
  VehCtrlMdel240912_2018b_DW.UnitDelay2_DSTATE_d = rtb_Yk1_l;

  /* Update for UnitDelay: '<S10>/Unit Delay6' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  VehCtrlMdel240912_2018b_DW.UnitDelay6_DSTATE_i =
    VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_l;

  /* Update for UnitDelay: '<S20>/Unit Delay5' */
  VehCtrlMdel240912_2018b_DW.UnitDelay5_DSTATE_l = rtb_APP_POS2;

  /* Update for UnitDelay: '<S20>/Unit Delay1' */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_g = rtb_deltafalllimit_km;

  /* Update for UnitDelay: '<S76>/Delay Input1'
   *
   * Block description for '<S76>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput1_DSTATE = rtb_LogicalOperator2;

  /* Update for UnitDelay: '<S18>/Unit Delay3' */
  VehCtrlMdel240912_2018b_DW.UnitDelay3_DSTATE_i = rtb_Compare_in;

  /* Update for UnitDelay: '<S19>/Unit Delay5' */
  VehCtrlMdel240912_2018b_DW.UnitDelay5_DSTATE_i = rtb_Add;

  /* Update for UnitDelay: '<S19>/Unit Delay1' */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_f = rtb_Switch2_b0;

  /* Update for UnitDelay: '<S65>/Delay Input1'
   *
   * Block description for '<S65>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput1_DSTATE_j = rtb_Compare_am;

  /* Update for UnitDelay: '<S18>/Unit Delay5' */
  VehCtrlMdel240912_2018b_DW.UnitDelay5_DSTATE_ip = rtb_Add4_j;

  /* Update for UnitDelay: '<S18>/Unit Delay1' */
  VehCtrlMdel240912_2018b_DW.UnitDelay1_DSTATE_g4 = Acc_POS;

  /* Update for UnitDelay: '<S56>/Delay Input1'
   *
   * Block description for '<S56>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240912_2018b_DW.DelayInput1_DSTATE_b = rtb_LowerRelop1_b;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/10ms1' */

  /* S-Function (fcncallgen): '<S5>/10ms2' incorporates:
   *  SubSystem: '<S5>/VCU2AMKMCUFL'
   */
  /* S-Function (scanpack): '<S280>/CAN Pack1' incorporates:
   *  Constant: '<S280>/Constant'
   *  Constant: '<S280>/Constant1'
   */
  /* S-Function (scanpack): '<S280>/CAN Pack1' */
  VehCtrlMdel240912_2018b_B.CANPack1_d.ID = 386U;
  VehCtrlMdel240912_2018b_B.CANPack1_d.Length = 8U;
  VehCtrlMdel240912_2018b_B.CANPack1_d.Extended = 0U;
  VehCtrlMdel240912_2018b_B.CANPack1_d.Remote = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_d.Data[0] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_d.Data[1] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_d.Data[2] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_d.Data[3] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_d.Data[4] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_d.Data[5] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_d.Data[6] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_d.Data[7] = 0;

  {
    /* --------------- START Packing signal 0 ------------------
     *  startBit                = 16
     *  length                  = 16
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = SIGNED
     *  factor                  = 0.0098
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real32_T outValue = 0;

      {
        real32_T result = VehCtrlMdel240912_2018b_B.TrqFL_cmd;

        /* no offset to apply */
        result = result * (1 / 0.0098F);

        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        int16_T packedValue;
        int32_T scaledValue;
        if (outValue > 2147483647.0) {
          scaledValue = 2147483647;
        } else if (outValue < -2147483648.0) {
          scaledValue = -2147483647 - 1;
        } else {
          scaledValue = (int32_T) outValue;
        }

        if (scaledValue > (int32_T) (32767)) {
          packedValue = 32767;
        } else if (scaledValue < (int32_T)((-(32767)-1))) {
          packedValue = (-(32767)-1);
        } else {
          packedValue = (int16_T) (scaledValue);
        }

        {
          uint16_T* tempValuePtr = (uint16_T*)&packedValue;
          uint16_T tempValue = *tempValuePtr;

          {
            VehCtrlMdel240912_2018b_B.CANPack1_d.Data[2] =
              VehCtrlMdel240912_2018b_B.CANPack1_d.Data[2] | (uint8_T)((uint16_T)
              (tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240912_2018b_B.CANPack1_d.Data[3] =
              VehCtrlMdel240912_2018b_B.CANPack1_d.Data[3] | (uint8_T)((uint16_T)
              ((uint16_T)(tempValue & (uint16_T)0xFF00U) >> 8));
          }
        }
      }
    }

    /* --------------- START Packing signal 1 ------------------
     *  startBit                = 48
     *  length                  = 16
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = SIGNED
     *  factor                  = 0.0098
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real64_T outValue = 0;

      {
        real64_T result = 0.0;

        /* no offset to apply */
        result = result * (1 / 0.0098);

        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        int16_T packedValue;
        int32_T scaledValue;
        if (outValue > 2147483647.0) {
          scaledValue = 2147483647;
        } else if (outValue < -2147483648.0) {
          scaledValue = -2147483647 - 1;
        } else {
          scaledValue = (int32_T) outValue;
        }

        if (scaledValue > (int32_T) (32767)) {
          packedValue = 32767;
        } else if (scaledValue < (int32_T)((-(32767)-1))) {
          packedValue = (-(32767)-1);
        } else {
          packedValue = (int16_T) (scaledValue);
        }

        {
          uint16_T* tempValuePtr = (uint16_T*)&packedValue;
          uint16_T tempValue = *tempValuePtr;

          {
            VehCtrlMdel240912_2018b_B.CANPack1_d.Data[6] =
              VehCtrlMdel240912_2018b_B.CANPack1_d.Data[6] | (uint8_T)((uint16_T)
              (tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240912_2018b_B.CANPack1_d.Data[7] =
              VehCtrlMdel240912_2018b_B.CANPack1_d.Data[7] | (uint8_T)((uint16_T)
              ((uint16_T)(tempValue & (uint16_T)0xFF00U) >> 8));
          }
        }
      }
    }

    /* --------------- START Packing signal 2 ------------------
     *  startBit                = 32
     *  length                  = 16
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = SIGNED
     *  factor                  = 0.0098
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real64_T outValue = 0;

      {
        real64_T result = 1000.0;

        /* no offset to apply */
        result = result * (1 / 0.0098);

        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        int16_T packedValue;
        int32_T scaledValue;
        if (outValue > 2147483647.0) {
          scaledValue = 2147483647;
        } else if (outValue < -2147483648.0) {
          scaledValue = -2147483647 - 1;
        } else {
          scaledValue = (int32_T) outValue;
        }

        if (scaledValue > (int32_T) (32767)) {
          packedValue = 32767;
        } else if (scaledValue < (int32_T)((-(32767)-1))) {
          packedValue = (-(32767)-1);
        } else {
          packedValue = (int16_T) (scaledValue);
        }

        {
          uint16_T* tempValuePtr = (uint16_T*)&packedValue;
          uint16_T tempValue = *tempValuePtr;

          {
            VehCtrlMdel240912_2018b_B.CANPack1_d.Data[4] =
              VehCtrlMdel240912_2018b_B.CANPack1_d.Data[4] | (uint8_T)((uint16_T)
              (tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240912_2018b_B.CANPack1_d.Data[5] =
              VehCtrlMdel240912_2018b_B.CANPack1_d.Data[5] | (uint8_T)((uint16_T)
              ((uint16_T)(tempValue & (uint16_T)0xFF00U) >> 8));
          }
        }
      }
    }

    /* --------------- START Packing signal 3 ------------------
     *  startBit                = 9
     *  length                  = 1
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T)
          (VehCtrlMdel240912_2018b_B.MCFL_DCOn_setpoints);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        if (packingValue > (boolean_T)(1)) {
          packedValue = (uint8_T) 1;
        } else if (packingValue < (boolean_T)(0)) {
          packedValue = (uint8_T) 0;
        } else {
          packedValue = (uint8_T) (packingValue);
        }

        {
          {
            VehCtrlMdel240912_2018b_B.CANPack1_d.Data[1] =
              VehCtrlMdel240912_2018b_B.CANPack1_d.Data[1] | (uint8_T)((uint8_T)
              ((uint8_T)(packedValue & (uint8_T)0x1U) << 1));
          }
        }
      }
    }

    /* --------------- START Packing signal 4 ------------------
     *  startBit                = 10
     *  length                  = 1
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (VehCtrlMdel240912_2018b_B.MCFL_DCEnable);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        if (packingValue > (boolean_T)(1)) {
          packedValue = (uint8_T) 1;
        } else if (packingValue < (boolean_T)(0)) {
          packedValue = (uint8_T) 0;
        } else {
          packedValue = (uint8_T) (packingValue);
        }

        {
          {
            VehCtrlMdel240912_2018b_B.CANPack1_d.Data[1] =
              VehCtrlMdel240912_2018b_B.CANPack1_d.Data[1] | (uint8_T)((uint8_T)
              ((uint8_T)(packedValue & (uint8_T)0x1U) << 2));
          }
        }
      }
    }

    /* --------------- START Packing signal 5 ------------------
     *  startBit                = 11
     *  length                  = 1
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real64_T outValue = 0;

      {
        real64_T result = VehCtrlMdel240912_2018b_B.errorReset;

        /* no scaling required */
        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        uint8_T packedValue;
        if (outValue > (real64_T)(1)) {
          packedValue = (uint8_T) 1;
        } else if (outValue < (real64_T)(0)) {
          packedValue = (uint8_T) 0;
        } else {
          packedValue = (uint8_T) (outValue);
        }

        {
          {
            VehCtrlMdel240912_2018b_B.CANPack1_d.Data[1] =
              VehCtrlMdel240912_2018b_B.CANPack1_d.Data[1] | (uint8_T)((uint8_T)
              ((uint8_T)(packedValue & (uint8_T)0x1U) << 3));
          }
        }
      }
    }

    /* --------------- START Packing signal 6 ------------------
     *  startBit                = 8
     *  length                  = 1
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (VehCtrlMdel240912_2018b_B.MCFL_InverterOn);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        if (packingValue > (boolean_T)(1)) {
          packedValue = (uint8_T) 1;
        } else if (packingValue < (boolean_T)(0)) {
          packedValue = (uint8_T) 0;
        } else {
          packedValue = (uint8_T) (packingValue);
        }

        {
          {
            VehCtrlMdel240912_2018b_B.CANPack1_d.Data[1] =
              VehCtrlMdel240912_2018b_B.CANPack1_d.Data[1] | (uint8_T)((uint8_T)
              (packedValue & (uint8_T)0x1U));
          }
        }
      }
    }

    /* --------------- START Packing signal 7 ------------------
     *  startBit                = 0
     *  length                  = 8
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/

    /* --------------- START Packing signal 8 ------------------
     *  startBit                = 12
     *  length                  = 4
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
  }

  /* S-Function (ecucoder_canmessage): '<S280>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    VehCtrlMdel240912_2018b_B.CANPackMessage_h[0]=
      VehCtrlMdel240912_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage_h[1]=
      VehCtrlMdel240912_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage_h[2]=
      VehCtrlMdel240912_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage_h[3]=
      VehCtrlMdel240912_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage_h[4]=
      VehCtrlMdel240912_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage_h[5]=
      VehCtrlMdel240912_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage_h[6]=
      VehCtrlMdel240912_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage_h[7]=
      VehCtrlMdel240912_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S280>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN1BUF8TX[8];
    uint8 can1buf8looptx= 0;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_h[0];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_h[1];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_h[2];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_h[3];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_h[4];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_h[5];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_h[6];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_h[7];
    can1buf8looptx++;
    VehCtrlMdel240912_2018b_B.CANTransmit_c= ec_can_transmit(1, 8, 1, 386U, 8,
      CAN1BUF8TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms2' */

  /* S-Function (fcncallgen): '<S5>/10ms4' incorporates:
   *  SubSystem: '<S5>/VCU2AMKMCUFR'
   */
  /* S-Function (scanpack): '<S281>/CAN Pack1' incorporates:
   *  Constant: '<S281>/Constant'
   *  Constant: '<S281>/Constant1'
   */
  /* S-Function (scanpack): '<S281>/CAN Pack1' */
  VehCtrlMdel240912_2018b_B.CANPack1.ID = 387U;
  VehCtrlMdel240912_2018b_B.CANPack1.Length = 8U;
  VehCtrlMdel240912_2018b_B.CANPack1.Extended = 0U;
  VehCtrlMdel240912_2018b_B.CANPack1.Remote = 0;
  VehCtrlMdel240912_2018b_B.CANPack1.Data[0] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1.Data[1] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1.Data[2] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1.Data[3] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1.Data[4] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1.Data[5] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1.Data[6] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1.Data[7] = 0;

  {
    /* --------------- START Packing signal 0 ------------------
     *  startBit                = 16
     *  length                  = 16
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = SIGNED
     *  factor                  = 0.0098
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real32_T outValue = 0;

      {
        real32_T result = VehCtrlMdel240912_2018b_B.TrqFR_cmd;

        /* no offset to apply */
        result = result * (1 / 0.0098F);

        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        int16_T packedValue;
        int32_T scaledValue;
        if (outValue > 2147483647.0) {
          scaledValue = 2147483647;
        } else if (outValue < -2147483648.0) {
          scaledValue = -2147483647 - 1;
        } else {
          scaledValue = (int32_T) outValue;
        }

        if (scaledValue > (int32_T) (32767)) {
          packedValue = 32767;
        } else if (scaledValue < (int32_T)((-(32767)-1))) {
          packedValue = (-(32767)-1);
        } else {
          packedValue = (int16_T) (scaledValue);
        }

        {
          uint16_T* tempValuePtr = (uint16_T*)&packedValue;
          uint16_T tempValue = *tempValuePtr;

          {
            VehCtrlMdel240912_2018b_B.CANPack1.Data[2] =
              VehCtrlMdel240912_2018b_B.CANPack1.Data[2] | (uint8_T)((uint16_T)
              (tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240912_2018b_B.CANPack1.Data[3] =
              VehCtrlMdel240912_2018b_B.CANPack1.Data[3] | (uint8_T)((uint16_T)
              ((uint16_T)(tempValue & (uint16_T)0xFF00U) >> 8));
          }
        }
      }
    }

    /* --------------- START Packing signal 1 ------------------
     *  startBit                = 48
     *  length                  = 16
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = SIGNED
     *  factor                  = 0.0098
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real64_T outValue = 0;

      {
        real64_T result = 0.0;

        /* no offset to apply */
        result = result * (1 / 0.0098);

        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        int16_T packedValue;
        int32_T scaledValue;
        if (outValue > 2147483647.0) {
          scaledValue = 2147483647;
        } else if (outValue < -2147483648.0) {
          scaledValue = -2147483647 - 1;
        } else {
          scaledValue = (int32_T) outValue;
        }

        if (scaledValue > (int32_T) (32767)) {
          packedValue = 32767;
        } else if (scaledValue < (int32_T)((-(32767)-1))) {
          packedValue = (-(32767)-1);
        } else {
          packedValue = (int16_T) (scaledValue);
        }

        {
          uint16_T* tempValuePtr = (uint16_T*)&packedValue;
          uint16_T tempValue = *tempValuePtr;

          {
            VehCtrlMdel240912_2018b_B.CANPack1.Data[6] =
              VehCtrlMdel240912_2018b_B.CANPack1.Data[6] | (uint8_T)((uint16_T)
              (tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240912_2018b_B.CANPack1.Data[7] =
              VehCtrlMdel240912_2018b_B.CANPack1.Data[7] | (uint8_T)((uint16_T)
              ((uint16_T)(tempValue & (uint16_T)0xFF00U) >> 8));
          }
        }
      }
    }

    /* --------------- START Packing signal 2 ------------------
     *  startBit                = 32
     *  length                  = 16
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = SIGNED
     *  factor                  = 0.0098
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real64_T outValue = 0;

      {
        real64_T result = 1000.0;

        /* no offset to apply */
        result = result * (1 / 0.0098);

        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        int16_T packedValue;
        int32_T scaledValue;
        if (outValue > 2147483647.0) {
          scaledValue = 2147483647;
        } else if (outValue < -2147483648.0) {
          scaledValue = -2147483647 - 1;
        } else {
          scaledValue = (int32_T) outValue;
        }

        if (scaledValue > (int32_T) (32767)) {
          packedValue = 32767;
        } else if (scaledValue < (int32_T)((-(32767)-1))) {
          packedValue = (-(32767)-1);
        } else {
          packedValue = (int16_T) (scaledValue);
        }

        {
          uint16_T* tempValuePtr = (uint16_T*)&packedValue;
          uint16_T tempValue = *tempValuePtr;

          {
            VehCtrlMdel240912_2018b_B.CANPack1.Data[4] =
              VehCtrlMdel240912_2018b_B.CANPack1.Data[4] | (uint8_T)((uint16_T)
              (tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240912_2018b_B.CANPack1.Data[5] =
              VehCtrlMdel240912_2018b_B.CANPack1.Data[5] | (uint8_T)((uint16_T)
              ((uint16_T)(tempValue & (uint16_T)0xFF00U) >> 8));
          }
        }
      }
    }

    /* --------------- START Packing signal 3 ------------------
     *  startBit                = 9
     *  length                  = 1
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T)
          (VehCtrlMdel240912_2018b_B.MCFL_DCOn_setpoints);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        if (packingValue > (boolean_T)(1)) {
          packedValue = (uint8_T) 1;
        } else if (packingValue < (boolean_T)(0)) {
          packedValue = (uint8_T) 0;
        } else {
          packedValue = (uint8_T) (packingValue);
        }

        {
          {
            VehCtrlMdel240912_2018b_B.CANPack1.Data[1] =
              VehCtrlMdel240912_2018b_B.CANPack1.Data[1] | (uint8_T)((uint8_T)
              ((uint8_T)(packedValue & (uint8_T)0x1U) << 1));
          }
        }
      }
    }

    /* --------------- START Packing signal 4 ------------------
     *  startBit                = 10
     *  length                  = 1
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (VehCtrlMdel240912_2018b_B.MCFL_DCEnable);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        if (packingValue > (boolean_T)(1)) {
          packedValue = (uint8_T) 1;
        } else if (packingValue < (boolean_T)(0)) {
          packedValue = (uint8_T) 0;
        } else {
          packedValue = (uint8_T) (packingValue);
        }

        {
          {
            VehCtrlMdel240912_2018b_B.CANPack1.Data[1] =
              VehCtrlMdel240912_2018b_B.CANPack1.Data[1] | (uint8_T)((uint8_T)
              ((uint8_T)(packedValue & (uint8_T)0x1U) << 2));
          }
        }
      }
    }

    /* --------------- START Packing signal 5 ------------------
     *  startBit                = 11
     *  length                  = 1
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real64_T outValue = 0;

      {
        real64_T result = VehCtrlMdel240912_2018b_B.errorReset;

        /* no scaling required */
        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        uint8_T packedValue;
        if (outValue > (real64_T)(1)) {
          packedValue = (uint8_T) 1;
        } else if (outValue < (real64_T)(0)) {
          packedValue = (uint8_T) 0;
        } else {
          packedValue = (uint8_T) (outValue);
        }

        {
          {
            VehCtrlMdel240912_2018b_B.CANPack1.Data[1] =
              VehCtrlMdel240912_2018b_B.CANPack1.Data[1] | (uint8_T)((uint8_T)
              ((uint8_T)(packedValue & (uint8_T)0x1U) << 3));
          }
        }
      }
    }

    /* --------------- START Packing signal 6 ------------------
     *  startBit                = 8
     *  length                  = 1
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (VehCtrlMdel240912_2018b_B.MCFL_InverterOn);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        if (packingValue > (boolean_T)(1)) {
          packedValue = (uint8_T) 1;
        } else if (packingValue < (boolean_T)(0)) {
          packedValue = (uint8_T) 0;
        } else {
          packedValue = (uint8_T) (packingValue);
        }

        {
          {
            VehCtrlMdel240912_2018b_B.CANPack1.Data[1] =
              VehCtrlMdel240912_2018b_B.CANPack1.Data[1] | (uint8_T)((uint8_T)
              (packedValue & (uint8_T)0x1U));
          }
        }
      }
    }

    /* --------------- START Packing signal 7 ------------------
     *  startBit                = 0
     *  length                  = 8
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/

    /* --------------- START Packing signal 8 ------------------
     *  startBit                = 12
     *  length                  = 4
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
  }

  /* S-Function (ecucoder_canmessage): '<S281>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    VehCtrlMdel240912_2018b_B.CANPackMessage[0]=
      VehCtrlMdel240912_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage[1]=
      VehCtrlMdel240912_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage[2]=
      VehCtrlMdel240912_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage[3]=
      VehCtrlMdel240912_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage[4]=
      VehCtrlMdel240912_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage[5]=
      VehCtrlMdel240912_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage[6]=
      VehCtrlMdel240912_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage[7]=
      VehCtrlMdel240912_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S281>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN1BUF8TX[8];
    uint8 can1buf8looptx= 0;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage[0];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage[1];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage[2];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage[3];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage[4];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage[5];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage[6];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage[7];
    can1buf8looptx++;
    VehCtrlMdel240912_2018b_B.CANTransmit_l= ec_can_transmit(1, 8, 1, 146927393U,
      8, CAN1BUF8TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms4' */

  /* S-Function (fcncallgen): '<S5>/10ms3' incorporates:
   *  SubSystem: '<S5>/VCU2EmraxMCU'
   */
  /* Switch: '<S282>/Switch2' incorporates:
   *  Constant: '<S282>/Constant13'
   *  Constant: '<S282>/Constant17'
   *  Constant: '<S282>/Constant19'
   *  Constant: '<S282>/Constant20'
   *  Switch: '<S282>/Switch3'
   */
  if (rtb_LogicalOperator3) {
    Gear_Trs = 0.0;
    Mode_Trs = 0.0;
  } else {
    Gear_Trs = 2.0;
    Mode_Trs = 2.0;
  }

  /* End of Switch: '<S282>/Switch2' */

  /* DataTypeConversion: '<S282>/Cast To Boolean4' */
  VehCtrlMdel240912_2018b_B.CastToBoolean4 = (uint16_T)Gear_Trs;

  /* DataTypeConversion: '<S282>/Cast To Boolean6' */
  VehCtrlMdel240912_2018b_B.CastToBoolean6 = (uint16_T)Mode_Trs;

  /* DataTypeConversion: '<S282>/Data Type Conversion2' */
  VehCtrlMdel240912_2018b_B.DataTypeConversion2 = (int32_T)floor(rtb_Switch2_on);

  /* S-Function (scanpack): '<S282>/CAN Pack1' */
  /* S-Function (scanpack): '<S282>/CAN Pack1' */
  VehCtrlMdel240912_2018b_B.CANPack1_a.ID = 146927393U;
  VehCtrlMdel240912_2018b_B.CANPack1_a.Length = 8U;
  VehCtrlMdel240912_2018b_B.CANPack1_a.Extended = 1U;
  VehCtrlMdel240912_2018b_B.CANPack1_a.Remote = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_a.Data[0] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_a.Data[1] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_a.Data[2] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_a.Data[3] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_a.Data[4] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_a.Data[5] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_a.Data[6] = 0;
  VehCtrlMdel240912_2018b_B.CANPack1_a.Data[7] = 0;

  {
    /* --------------- START Packing signal 0 ------------------
     *  startBit                = 32
     *  length                  = 8
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (VehCtrlMdel240912_2018b_B.CastToBoolean4);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        if (packingValue > (uint16_T)(255)) {
          packedValue = (uint8_T) 255;
        } else {
          packedValue = (uint8_T) (packingValue);
        }

        {
          {
            VehCtrlMdel240912_2018b_B.CANPack1_a.Data[4] =
              VehCtrlMdel240912_2018b_B.CANPack1_a.Data[4] | (uint8_T)
              (packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 1 ------------------
     *  startBit                = 40
     *  length                  = 2
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (VehCtrlMdel240912_2018b_B.CastToBoolean6);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        if (packingValue > (uint16_T)(3)) {
          packedValue = (uint8_T) 3;
        } else {
          packedValue = (uint8_T) (packingValue);
        }

        {
          {
            VehCtrlMdel240912_2018b_B.CANPack1_a.Data[5] =
              VehCtrlMdel240912_2018b_B.CANPack1_a.Data[5] | (uint8_T)((uint8_T)
              (packedValue & (uint8_T)0x3U));
          }
        }
      }
    }

    /* --------------- START Packing signal 2 ------------------
     *  startBit                = 0
     *  length                  = 16
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 0.5
     *  offset                  = -10000.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real32_T outValue = 0;

      {
        real32_T result = VehCtrlMdel240912_2018b_B.TrqR_cmd;

        /* full scaling operation */
        result = (result - -10000.0F) * (1 / 0.5F);

        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        uint16_T packedValue;
        if (outValue > (real32_T)(65535)) {
          packedValue = (uint16_T) 65535;
        } else if (outValue < (real32_T)(0)) {
          packedValue = (uint16_T) 0;
        } else {
          packedValue = (uint16_T) (outValue);
        }

        {
          {
            VehCtrlMdel240912_2018b_B.CANPack1_a.Data[0] =
              VehCtrlMdel240912_2018b_B.CANPack1_a.Data[0] | (uint8_T)((uint16_T)
              (packedValue & (uint16_T)0xFFU));
            VehCtrlMdel240912_2018b_B.CANPack1_a.Data[1] =
              VehCtrlMdel240912_2018b_B.CANPack1_a.Data[1] | (uint8_T)((uint16_T)
              ((uint16_T)(packedValue & (uint16_T)0xFF00U) >> 8));
          }
        }
      }
    }

    /* --------------- START Packing signal 3 ------------------
     *  startBit                = 16
     *  length                  = 16
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      int32_T packingValue = 0;

      {
        int32_T result = (int32_T)
          (VehCtrlMdel240912_2018b_B.DataTypeConversion2);

        /* no scaling required */
        packingValue = result;
      }

      if (packingValue < 0) {
        packingValue = 0;
      }

      {
        uint16_T packedValue;
        if (packingValue > (int32_T)(65535)) {
          packedValue = (uint16_T) 65535;
        } else if (packingValue < (int32_T)(0)) {
          packedValue = (uint16_T) 0;
        } else {
          packedValue = (uint16_T) (packingValue);
        }

        {
          {
            VehCtrlMdel240912_2018b_B.CANPack1_a.Data[2] =
              VehCtrlMdel240912_2018b_B.CANPack1_a.Data[2] | (uint8_T)((uint16_T)
              (packedValue & (uint16_T)0xFFU));
            VehCtrlMdel240912_2018b_B.CANPack1_a.Data[3] =
              VehCtrlMdel240912_2018b_B.CANPack1_a.Data[3] | (uint8_T)((uint16_T)
              ((uint16_T)(packedValue & (uint16_T)0xFF00U) >> 8));
          }
        }
      }
    }
  }

  /* S-Function (ecucoder_canmessage): '<S282>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    VehCtrlMdel240912_2018b_B.CANPackMessage_d[0]=
      VehCtrlMdel240912_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage_d[1]=
      VehCtrlMdel240912_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage_d[2]=
      VehCtrlMdel240912_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage_d[3]=
      VehCtrlMdel240912_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage_d[4]=
      VehCtrlMdel240912_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage_d[5]=
      VehCtrlMdel240912_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage_d[6]=
      VehCtrlMdel240912_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240912_2018b_B.CANPackMessage_d[7]=
      VehCtrlMdel240912_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S282>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN0BUF8TX[8];
    uint8 can0buf8looptx= 0;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_d[0];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_d[1];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_d[2];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_d[3];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_d[4];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_d[5];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_d[6];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240912_2018b_B.CANPackMessage_d[7];
    can0buf8looptx++;
    VehCtrlMdel240912_2018b_B.CANTransmit_k= ec_can_transmit(0, 8, 1, 146927393U,
      8, CAN0BUF8TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms3' */

  /* S-Function (fcncallgen): '<S5>/10ms6' incorporates:
   *  SubSystem: '<S5>/WP_OUTPUT'
   */
  /* DataTypeConversion: '<S283>/Cast To Single1' */
  VehCtrlMdel240912_2018b_B.CastToSingle1 = (uint16_T)rtb_CastToDouble;

  /* S-Function (ec5744_pdsslbu3): '<S283>/PowerDriverSwitch(HS)' */

  /* Set level VehCtrlMdel240912_2018b_B.aWaterPumpON for the specified power driver switch */
  ec_gpio_write(50,VehCtrlMdel240912_2018b_B.aWaterPumpON);
  ec_gpio_write(42,VehCtrlMdel240912_2018b_B.aWaterPumpON);

  /* S-Function (ec5744_pdpslbu3): '<S283>/PowerDriverPWM' incorporates:
   *  Constant: '<S283>/Constant'
   */

  /* Power driver PWM output for channel 6 */
  ec_pwm_output(6,((uint16_T)1000U),VehCtrlMdel240912_2018b_B.CastToSingle1);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms6' */

  /* S-Function (fcncallgen): '<S288>/10ms' incorporates:
   *  SubSystem: '<S288>/daq10ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S300>/CCPDAQ' */
  ccpDaq(1);

  /* End of Outputs for S-Function (fcncallgen): '<S288>/10ms' */

  /* Update absolute time */
  /* The "clockTick3" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick3" ensures timer will not overflow during the
   * application lifespan selected.
   */
  VehCtrlMdel240912_2018b_M->Timing.clockTick3++;
}

/* Model step function for TID4 */
void VehCtrlMdel240912_2018b_step4(void) /* Sample time: [0.05s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S288>/50ms' incorporates:
   *  SubSystem: '<S288>/daq50ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S302>/CCPDAQ' */
  ccpDaq(2);

  /* End of Outputs for S-Function (fcncallgen): '<S288>/50ms' */
}

/* Model step function for TID5 */
void VehCtrlMdel240912_2018b_step5(void) /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S287>/100MS' incorporates:
   *  SubSystem: '<S287>/Function-Call Subsystem'
   */
  /* S-Function (ec5744_canreceiveslb): '<S291>/CANReceive' */

  /* Receive CAN message */
  {
    uint8 CAN2BUF1RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can2buf1looprx= 0;
    VehCtrlMdel240912_2018b_B.CANReceive_o3_l= 278;
    VehCtrlMdel240912_2018b_B.CANReceive_o5_l= 8;
    VehCtrlMdel240912_2018b_B.CANReceive_o2_p= ec_can_receive(2,1, CAN2BUF1RX);
    VehCtrlMdel240912_2018b_B.CANReceive_o4_i[0]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive_o4_i[1]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive_o4_i[2]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive_o4_i[3]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive_o4_i[4]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive_o4_i[5]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive_o4_i[6]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240912_2018b_B.CANReceive_o4_i[7]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
  }

  /* Call the system: <S291>/Function-Call Subsystem */

  /* Output and update for function-call system: '<S291>/Function-Call Subsystem' */
  {
    uint8_T rtb_Add;
    uint8_T rtb_Compare;

    /* Outputs for Enabled SubSystem: '<S292>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S293>/Enable'
     */
    if (VehCtrlMdel240912_2018b_B.CANReceive_o2_p > 0) {
      /* RelationalOperator: '<S294>/Compare' incorporates:
       *  Constant: '<S294>/Constant'
       */
      rtb_Add = (uint8_T)(VehCtrlMdel240912_2018b_B.CANReceive_o4_i[0] == 83);

      /* RelationalOperator: '<S295>/Compare' incorporates:
       *  Constant: '<S295>/Constant'
       */
      rtb_Compare = (uint8_T)(VehCtrlMdel240912_2018b_B.CANReceive_o4_i[5] == 84);

      /* Sum: '<S293>/Add' */
      rtb_Add = (uint8_T)((uint32_T)rtb_Add + rtb_Compare);

      /* RelationalOperator: '<S296>/Compare' incorporates:
       *  Constant: '<S296>/Constant'
       */
      rtb_Compare = (uint8_T)(rtb_Add == 2);

      /* If: '<S293>/If' */
      if (rtb_Compare > 0) {
        /* Outputs for IfAction SubSystem: '<S293>/If Action Subsystem' incorporates:
         *  ActionPort: '<S297>/Action Port'
         */
        /* S-Function (ec5744_bootloaderslb): '<S297>/BootLoader' */
        {
          uint16 i= 0;
          N256K_BLOCK_SEL n256KBlockSelect;
          CONTEXT_DATA pgmCtxData;
          n256KBlockSelect.first256KBlockSelect = 0x00000080;
          n256KBlockSelect.second256KBlockSelect = 0x00000000;
          pgmCtxData.pReqCompletionFn = pFlashProgram;
          uint8 CAN2BUF9TX[]= { 11, 12, 12, 13 };

          uint8 returnCode1= 0;
          uint8 bootflag[]= { 1, 0, 0, 0, 0, 0, 0, 0 };

          DisableInterrupts();
          SIUL2.GPDO[22].R = 1;
          App_FlashErase( &ssdConfig, 0, 0x00000000, 0x00000000, 0x00000000,
                         n256KBlockSelect );
          App_FlashProgram( &ssdConfig, 0, 0x011FFFF0, 8, (uint32)bootflag,
                           &pgmCtxData );
          i= 1000;
          while (i--) {
            ;
          }

          ec_can_transmit(2, 9, 0, 593, 4, CAN2BUF9TX);
          i= 10000;
          while (i--) {
            ;
          }
        }

        /* S-Function (ec5744_cpuresetslb): '<S297>/CPUReset' */

        /* Perform a microcontroller reset */
        MC_ME.MCTL.R = 0X00005AF0;
        MC_ME.MCTL.R = 0X0000A50F;

        /* End of Outputs for SubSystem: '<S293>/If Action Subsystem' */
      } else {
        /* Outputs for IfAction SubSystem: '<S293>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S298>/Action Port'
         */
        /* S-Function (ec5744_cantransmitslb): '<S298>/CANTransmit' incorporates:
         *  Constant: '<S298>/Constant'
         */

        /*Transmit CAN message*/
        {
          uint8 CAN2BUF9TX[1];
          uint8 can2buf9looptx= 0;
          CAN2BUF9TX[can2buf9looptx]= ((uint8_T)1U);
          can2buf9looptx++;
          VehCtrlMdel240912_2018b_B.CANTransmit= ec_can_transmit(2, 9, 0, 593U,
            1, CAN2BUF9TX);
        }

        /* End of Outputs for SubSystem: '<S293>/If Action Subsystem1' */
      }

      /* End of If: '<S293>/If' */
    }

    /* End of Outputs for SubSystem: '<S292>/Enabled Subsystem' */
  }

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S291>/CANReceive' */
  /* End of Outputs for S-Function (fcncallgen): '<S287>/100MS' */

  /* S-Function (fcncallgen): '<S288>/100ms' incorporates:
   *  SubSystem: '<S288>/daq100ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S299>/CCPDAQ' */
  ccpDaq(3);

  /* End of Outputs for S-Function (fcncallgen): '<S288>/100ms' */
}

/* Model step function for TID6 */
void VehCtrlMdel240912_2018b_step6(void) /* Sample time: [0.5s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S288>/500ms' incorporates:
   *  SubSystem: '<S288>/daq500ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S301>/CCPDAQ' */
  ccpDaq(4);

  /* End of Outputs for S-Function (fcncallgen): '<S288>/500ms' */

  /* S-Function (fcncallgen): '<S289>/500ms' incorporates:
   *  SubSystem: '<S289>/EEPROMOperation'
   */

  /* S-Function (ec5744_eepromoslb): '<S304>/EEPROMOperatin' */
#if defined EC_EEPROM_ENABLE

  /* Operate the EEPROM module on the MPC5744 */
  ec_flash_operation();

#endif

  /* End of Outputs for S-Function (fcncallgen): '<S289>/500ms' */
}

/* Model step wrapper function for compatibility with a static main program */
void VehCtrlMdel240912_2018b_step(int_T tid)
{
  switch (tid) {
   case 0 :
    VehCtrlMdel240912_2018b_step0();
    break;

   case 1 :
    VehCtrlMdel240912_2018b_step1();
    break;

   case 2 :
    VehCtrlMdel240912_2018b_step2();
    break;

   case 3 :
    VehCtrlMdel240912_2018b_step3();
    break;

   case 4 :
    VehCtrlMdel240912_2018b_step4();
    break;

   case 5 :
    VehCtrlMdel240912_2018b_step5();
    break;

   case 6 :
    VehCtrlMdel240912_2018b_step6();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void VehCtrlMdel240912_2018b_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* Start for S-Function (fcncallgen): '<S3>/10ms6' incorporates:
   *  SubSystem: '<S3>/EMRAXMCU_RECIEVE'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S98>/CANReceive1' incorporates:
   *  SubSystem: '<S98>/MCU_pwr'
   */
  /* Start for function-call system: '<S98>/MCU_pwr' */

  /* Start for Enabled SubSystem: '<S120>/MCU_VCUMeter1' */

  /* Start for S-Function (scanunpack): '<S122>/CAN Unpack' */

  /*-----------S-Function Block: <S122>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S120>/MCU_VCUMeter1' */
  ec_buffer_init(0,1,1,218089455);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S98>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S98>/CANReceive3' incorporates:
   *  SubSystem: '<S98>/MCU_state'
   */
  /* Start for function-call system: '<S98>/MCU_state' */

  /* Start for Enabled SubSystem: '<S121>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S127>/CAN Unpack' */

  /*-----------S-Function Block: <S127>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S121>/MCU_state' */
  ec_buffer_init(0,0,1,218089199);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S98>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S94>/CANReceive3' incorporates:
   *  SubSystem: '<S94>/ABS_BUS_state'
   */
  /* Start for function-call system: '<S94>/ABS_BUS_state' */

  /* Start for Enabled SubSystem: '<S102>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S103>/CAN Unpack1' */

  /*-----------S-Function Block: <S103>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S102>/IMU_state' */
  ec_buffer_init(0,16,0,1698);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S94>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S100>/CANReceive3' incorporates:
   *  SubSystem: '<S100>/StrWhSnis_state'
   */
  /* Start for function-call system: '<S100>/StrWhSnis_state' */

  /* Start for Enabled SubSystem: '<S143>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S144>/CAN Unpack1' */

  /*-----------S-Function Block: <S144>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S143>/IMU_state' */
  ec_buffer_init(0,7,0,330);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S100>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms4' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S104>/CANReceive3' incorporates:
   *  SubSystem: '<S104>/AMKMCU_state'
   */
  /* Start for function-call system: '<S104>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S106>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S109>/CAN Unpack' */

  /*-----------S-Function Block: <S109>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S106>/MCU_state' */
  ec_buffer_init(1,1,0,640);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S104>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S104>/CANReceive1' */
  ec_buffer_init(1,2,0,642);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S104>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S104>/CANReceive2' incorporates:
   *  SubSystem: '<S104>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S104>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S108>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S111>/CAN Unpack' */

  /*-----------S-Function Block: <S111>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S108>/MCU_state' */
  ec_buffer_init(1,3,0,644);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S104>/CANReceive2' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S105>/CANReceive3' incorporates:
   *  SubSystem: '<S105>/AMKMCU_state'
   */
  /* Start for function-call system: '<S105>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S112>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S115>/CAN Unpack' */

  /*-----------S-Function Block: <S115>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S112>/MCU_state' */
  ec_buffer_init(1,4,0,641);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S105>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S105>/CANReceive1' */
  ec_buffer_init(1,5,0,643);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S105>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S105>/CANReceive2' incorporates:
   *  SubSystem: '<S105>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S105>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S114>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S117>/CAN Unpack' */

  /*-----------S-Function Block: <S117>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S114>/MCU_state' */
  ec_buffer_init(1,6,0,387);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S105>/CANReceive2' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms5' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S99>/CANReceive3' incorporates:
   *  SubSystem: '<S99>/IMU_state'
   */
  /* Start for function-call system: '<S99>/IMU_state' */

  /* Start for Enabled SubSystem: '<S134>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S135>/CAN Unpack' */

  /*-----------S-Function Block: <S135>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S134>/MCU_state' */
  ec_buffer_init(0,0,0,513);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S99>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms2' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S97>/CANReceive3' */
  ec_buffer_init(0,16,0,1698);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S97>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S280>/CANTransmit' */
  ec_buffer_init(1,8,1,386U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms2' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S281>/CANTransmit' */
  ec_buffer_init(1,8,1,146927393U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms4' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S282>/CANTransmit' */
  ec_buffer_init(0,8,1,146927393U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S5>/10ms6' incorporates:
   *  SubSystem: '<S5>/WP_OUTPUT'
   */
  /* Start for S-Function (ec5744_pdpslbu3): '<S283>/PowerDriverPWM' incorporates:
   *  Constant: '<S283>/Constant'
   */

  /* Initialize PWM output for channel 6 */
  SIUL2_MSCR42 = 0X02000003;           //PWM_A3

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S287>/100MS' incorporates:
   *  SubSystem: '<S287>/Function-Call Subsystem'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S291>/CANReceive' incorporates:
   *  SubSystem: '<S291>/Function-Call Subsystem'
   */
  /* Start for function-call system: '<S291>/Function-Call Subsystem' */

  /* Start for Enabled SubSystem: '<S292>/Enabled Subsystem' */
  /* Start for IfAction SubSystem: '<S293>/If Action Subsystem1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S298>/CANTransmit' incorporates:
   *  Constant: '<S298>/Constant'
   */
  ec_buffer_init(2,9,0,593U);

  /* End of Start for SubSystem: '<S293>/If Action Subsystem1' */
  /* End of Start for SubSystem: '<S292>/Enabled Subsystem' */
  ec_buffer_init(2,1,0,278);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S291>/CANReceive' */
  /* End of Start for S-Function (fcncallgen): '<S287>/100MS' */

  /* Start for S-Function (fcncallgen): '<S290>/Function-Call Generator' incorporates:
   *  SubSystem: '<S290>/CCPBackground'
   */
  /* Start for S-Function (ec5744_ccpslb): '<S305>/CCPBackground' */
  ccpInit();

  /* End of Start for S-Function (fcncallgen): '<S290>/Function-Call Generator' */

  /* Start for S-Function (ec5744_caninterruptslb1): '<S290>/ReceiveandTransmitInterrupt' incorporates:
   *  SubSystem: '<S290>/CCPReceive'
   */
  /* Start for function-call system: '<S290>/CCPReceive' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S306>/CANReceive' */
  ec_buffer_init(2,0,0,CCP_CRO_ID);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S306>/CANReceive' */
  ec_bufint_init(2,0);
  INTC_0.PSR[548].B.PRIN = 12;
  IntcIsrVectorTable[548] = (uint32_t)&ISR_FlexCAN_2_MB0;

  /* End of Start for S-Function (ec5744_caninterruptslb1): '<S290>/ReceiveandTransmitInterrupt' */

  /* Start for S-Function (ec5744_eeprombsbu3): '<Root>/EEPROMEnable' */
  Fls_Read(0xFA0010,ecflashdataold,4096);
  Fls_Read(0xFA0010,ecflashdatanew,4096);

  /* SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  /* InitializeConditions for UnitDelay: '<S209>/Unit Delay4' */
  VehCtrlMdel240912_2018b_DW.UnitDelay4_DSTATE_mn = 0.01F;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' */

  /* SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* SystemInitialize for Chart: '<S90>/Chart2' */
  VehCtrlMdel240912_2018b_DW.sfEvent = -1;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S4>/10ms' incorporates:
   *  SubSystem: '<S4>/Function-Call Subsystem'
   */
  VehCtrlMdel240912_2018b_DW.FunctionCallSubsystem_RESET_ELA = true;

  /* End of Enable for S-Function (fcncallgen): '<S4>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  VehCtrlMdel240912_2018b_DW.Subsystem_RESET_ELAPS_T = true;

  /* End of Enable for S-Function (fcncallgen): '<S4>/10ms1' */

  /* Enable for S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* Enable for Chart: '<S90>/Chart2' */
  VehCtrlMdel240912_2018b_DW.previousTicks =
    VehCtrlMdel240912_2018b_M->Timing.clockTick3;

  /* End of Enable for S-Function (fcncallgen): '<S2>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S1>/10ms1' incorporates:
   *  SubSystem: '<S1>/MoTrqReq'
   */
  VehCtrlMdel240912_2018b_DW.MoTrqReq_RESET_ELAPS_T = true;

  /* End of Enable for S-Function (fcncallgen): '<S1>/10ms1' */
}

/* File trailer for ECUCoder generated file VehCtrlMdel240912_2018b.c.
 *
 * [EOF]
 */
