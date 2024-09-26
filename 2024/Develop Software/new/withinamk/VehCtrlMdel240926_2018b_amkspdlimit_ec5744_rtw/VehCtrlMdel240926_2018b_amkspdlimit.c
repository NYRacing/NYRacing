/*
 * Code generated for Simulink model VehCtrlMdel240926_2018b_amkspdlimit.
 *
 * FILE    : VehCtrlMdel240926_2018b_amkspdlimit.c
 *
 * VERSION : 1.174
 *
 * DATE    : Thu Sep 26 14:50:31 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#include "VehCtrlMdel240926_2018b_amkspdlimit.h"
#include "VehCtrlMdel240926_2018b_amkspdlimit_private.h"

/* #include "myinclude.h" */

/* Named constants for Chart: '<S8>/Timer1' */
#define VehCtrlMdel240926_2018b__IN_Out ((uint8_T)2U)
#define VehCtrlMdel240926_20_IN_Trigger ((uint8_T)3U)
#define VehCtrlMdel240926_IN_InterState ((uint8_T)1U)

/* Named constants for Chart: '<S122>/Timer' */
#define VehCtrlMdel240926_2018_IN_Out_n ((uint8_T)2U)
#define VehCtrlMdel240926__IN_Trigger_c ((uint8_T)3U)
#define VehCtrlMdel2409_IN_InterState_n ((uint8_T)1U)

/* Named constants for Chart: '<S7>/Chart' */
#define VehCtrlMdel240926_2018b_am_IN_B ((uint8_T)1U)
#define VehCtrlMdel240926_2018b_am_IN_C ((uint8_T)2U)
#define VehCtrlMdel240926_IN_DYC_Enable ((uint8_T)2U)
#define VehCtrlMdel240926_IN_InitState1 ((uint8_T)1U)
#define VehCtrlMdel240926_IN_InitState2 ((uint8_T)1U)
#define VehCtrlMdel240926__IN_InitState ((uint8_T)3U)
#define VehCtrlMdel240_IN_DYC_Disenable ((uint8_T)1U)
#define VehCtrlMdel24_IN_TCSF_Disenable ((uint8_T)2U)
#define VehCtrlMdel24_IN_TCSR_Disenable ((uint8_T)2U)
#define VehCtrlMdel2_IN_F_TVD_TCS_STATE ((uint8_T)3U)

/* Named constants for Chart: '<S105>/Chart2' */
#define VehCtrlM_IN_MC_InverterOn_State ((uint8_T)7U)
#define VehCtrlMdel240926_2018_IN_Guard ((uint8_T)1U)
#define VehCtrlMdel240926_2018_IN_Ready ((uint8_T)5U)
#define VehCtrlMdel240926_2018_IN_Trans ((uint8_T)7U)
#define VehCtrlMdel240926_2018_IN_start ((uint8_T)9U)
#define VehCtrlMdel240926_2018b_IN_Init ((uint8_T)2U)
#define VehCtrlMdel240926_2018b__IN_OFF ((uint8_T)1U)
#define VehCtrlMdel240926_2018b_a_IN_ON ((uint8_T)2U)
#define VehCtrlMdel240926_201_IN_AMKCAN ((uint8_T)1U)
#define VehCtrlMdel240926_20_IN_Standby ((uint8_T)6U)
#define VehCtrlMdel240926_IN_MC_DCready ((uint8_T)6U)
#define VehCtrlMdel240926_IN_SYSRDYCECK ((uint8_T)8U)
#define VehCtrlMdel240926_event_AMKDCON (3)
#define VehCtrlMdel240926_event_EbeepON (5)
#define VehCtrlMdel24092_IN_MCDCOncheck ((uint8_T)5U)
#define VehCtrlMdel24092_event_AMKCANON (1)
#define VehCtrlMdel24092_event_AMKDCOFF (2)
#define VehCtrlMdel24092_event_EbeepOFF (4)
#define VehCtrlMdel24092_event_TorqueON (11)
#define VehCtrlMdel2409_IN_MCUReadyFail ((uint8_T)4U)
#define VehCtrlMdel2409_event_AMKCANOFF (0)
#define VehCtrlMdel2409_event_TorqueOFF (10)
#define VehCtrlMdel240_IN_AMKDCOnFinish ((uint8_T)2U)
#define VehCtrlMdel240_IN_DCOnCheckPass ((uint8_T)3U)
#define VehCtrlMdel240_IN_InitStateBack ((uint8_T)3U)
#define VehCtrlMdel240_IN_WaitForEngine ((uint8_T)8U)
#define VehCtrlMdel240_event_InverterON (7)
#define VehCtrlMdel24_event_InverterOFF (6)
#define VehCtrlMdel2_IN_MCDCEnableState ((uint8_T)4U)
#define VehCtrlMdel2_event_MCDCEnableON (9)
#define VehCtrlMdel_event_MCDCEnableOFF (8)

EE_Data ecflashdataold[1024];
EE_Data ecflashdatanew[1024];

/* Exported block signals */
real_T Gear_Trs;                       /* '<S336>/Switch2' */
real_T Mode_Trs;                       /* '<S336>/Switch3' */
real_T Trq_CUT;                        /* '<S201>/Timer' */
real_T ignition;                       /* '<S122>/Timer' */
real_T L12V_error;                     /* '<S179>/CAN Unpack' */
real_T alarm;                          /* '<S179>/CAN Unpack' */
real_T controller_ready;               /* '<S179>/CAN Unpack' */
real_T selfcheck;                      /* '<S179>/CAN Unpack' */
real_T RPM;                            /* '<S179>/CAN Unpack' */
real_T trq;                            /* '<S179>/CAN Unpack' */
real_T AC_current;                     /* '<S174>/CAN Unpack' */
real_T DC_current;                     /* '<S174>/CAN Unpack' */
real_T MCU_Temp;                       /* '<S174>/CAN Unpack' */
real_T motor_Temp;                     /* '<S174>/CAN Unpack' */
real_T MCFR_ActualVelocity;            /* '<S152>/CAN Unpack' */
real_T MCFR_DCVoltage;                 /* '<S152>/CAN Unpack' */
real_T MCFR_bDCOn;                     /* '<S152>/CAN Unpack' */
real_T MCFR_bError;                    /* '<S152>/CAN Unpack' */
real_T MCFR_bInverterOn;               /* '<S152>/CAN Unpack' */
real_T MCFR_bQuitInverterOn;           /* '<S152>/CAN Unpack' */
real_T MCFR_bSystemReady;              /* '<S152>/CAN Unpack' */
real_T MCFR_TempIGBT;                  /* '<S162>/CAN Unpack' */
real_T MCFR_TempInverter;              /* '<S162>/CAN Unpack' */
real_T MCFR_TempMotor;                 /* '<S162>/CAN Unpack' */
real_T MCFR_ErrorInfo;                 /* '<S160>/CAN Unpack' */
real_T MCFL_DCVoltage;                 /* '<S134>/CAN Unpack' */
real_T MCFL_bDCOn;                     /* '<S134>/CAN Unpack' */
real_T MCFL_bError;                    /* '<S134>/CAN Unpack' */
real_T MCFL_bInverterOn;               /* '<S134>/CAN Unpack' */
real_T MCFL_bQuitDCOn;                 /* '<S134>/CAN Unpack' */
real_T MCFL_bQuitInverterOn;           /* '<S134>/CAN Unpack' */
real_T MCFL_bSystemReady;              /* '<S134>/CAN Unpack' */
real_T MCFL_ActualVelocity;            /* '<S134>/Gain' */
real_T MCFL_TempIGBT;                  /* '<S145>/CAN Unpack' */
real_T MCFL_TempInverter;              /* '<S145>/CAN Unpack' */
real_T MCFL_TempMotor;                 /* '<S145>/CAN Unpack' */
real_T MCFL_ErrorInfo;                 /* '<S143>/CAN Unpack' */
real_T StrWhlAngAliveRollCnt;          /* '<S192>/CAN Unpack1' */
real_T StrWhlAng;                      /* '<S192>/CAN Unpack1' */
real_T StrWhlAngV;                     /* '<S192>/CAN Unpack1' */
real_T ABS_WS_FL;                      /* '<S124>/CAN Unpack1' */
real_T ABS_WS_FR;                      /* '<S124>/CAN Unpack1' */
real_T ABS_WS_RL;                      /* '<S124>/CAN Unpack1' */
real_T ABS_WS_RR;                      /* '<S124>/CAN Unpack1' */
real_T IMU_Ay_Value;                   /* '<S187>/CAN Unpack' */
real_T IMU_Ax_Value;                   /* '<S187>/CAN Unpack' */
real_T IMU_Yaw_Value;                  /* '<S187>/CAN Unpack' */
real_T EMRAX_Trq_CUT;                  /*  */
real_T AMK_Trq_CUT;                    /*  */
uint32_T Acc_vol2;                     /* '<S201>/Add3' */
uint32_T Acc_vol;                      /* '<S201>/Add2' */
uint32_T Acc_POS2;                     /* '<S201>/1-D Lookup Table3' */
real32_T Acc_POS;                      /* '<S201>/MATLAB Function' */
real32_T TrqR_cmd;                     /* '<S7>/Saturation1' */
real32_T TrqFR_cmd;                    /* '<S7>/Saturation2' */
real32_T TrqFL_cmd;                    /* '<S7>/Saturation3' */
uint16_T F_BrkPrs;                     /* '<S201>/1-D Lookup Table1' */
uint16_T Acc1;                         /* '<S117>/Acc3' */
uint16_T Acc2;                         /* '<S117>/Acc4' */
uint16_T Brk1;                         /* '<S117>/Brk1' */
uint16_T Brk2;                         /* '<S117>/Brk2' */
boolean_T HVCUTOFF;                    /* '<S122>/Constant' */
boolean_T KeyPressed;                  /* '<S105>/Cast To Boolean' */
boolean_T Brk;                         /* '<S107>/Compare' */
boolean_T ACC_Release;                 /* '<S108>/Compare' */
boolean_T beeper_state;                /* '<S105>/Chart2' */
boolean_T MCFL_DCOn_setpoints;         /* '<S105>/Chart2' */
boolean_T MCFR_DCEnable;               /* '<S105>/Chart2' */
boolean_T MCFR_InverterOn;             /* '<S105>/Chart2' */
boolean_T TroqueOn;                    /* '<S7>/Logical Operator6' */
boolean_T Trq_CUT_final;               /* '<S7>/Logical Operator4' */

/* Block signals (default storage) */
B_VehCtrlMdel240926_2018b_amk_T VehCtrlMdel240926_2018b_amksp_B;

/* Block states (default storage) */
DW_VehCtrlMdel240926_2018b_am_T VehCtrlMdel240926_2018b_amks_DW;

/* Real-time model */
RT_MODEL_VehCtrlMdel240926_20_T VehCtrlMdel240926_2018b_amks_M_;
RT_MODEL_VehCtrlMdel240926_20_T *const VehCtrlMdel240926_2018b_amks_M =
  &VehCtrlMdel240926_2018b_amks_M_;

/* Forward declaration for local functions */
static void VehC_enter_atomic_WaitForEngine(void);
static void VehCtrlMdel240926_2018b_VehStat(const real_T *MCFL_bQuitInverterOn_k,
  const real_T *MCFL_bSystemReady_i, const real_T *controller_ready_e, const
  real_T *MCFR_bQuitInverterOn_d, const real_T *MCFR_bSystemReady_g);
static void VehCtrlMdel_enter_atomic_AMKCAN(void);
static void VehCtrlMdel240926_20_AMKDCready(const real_T *MCFL_bDCOn_j, const
  real_T *MCFL_bQuitInverterOn_k, const real_T *MCFL_bSystemReady_i, const
  real_T *MCFR_bDCOn_n, const real_T *MCFR_bQuitInverterOn_d, const real_T
  *MCFR_bSystemReady_g);
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
  /* Call the system: <S344>/CCPReceive */
  {
    /* S-Function (ec5744_caninterruptslb1): '<S344>/ReceiveandTransmitInterrupt' */

    /* Output and update for function-call system: '<S344>/CCPReceive' */

    /* S-Function (ec5744_canreceiveslb): '<S360>/CANReceive' */

    /* Receive CAN message */
    {
      uint8 CAN2BUF0RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

      uint8 can2buf0looprx= 0;
      VehCtrlMdel240926_2018b_amksp_B.CANReceive_o3= 256;
      VehCtrlMdel240926_2018b_amksp_B.CANReceive_o5= 8;
      VehCtrlMdel240926_2018b_amksp_B.CANReceive_o2= ec_can_receive(2,0,
        CAN2BUF0RX);
      VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4[0]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4[1]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4[2]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4[3]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4[4]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4[5]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4[6]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4[7]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
    }

    /* Nothing to do for system: <S360>/Nothing */

    /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S360>/CANReceive' */

    /* End of Outputs for S-Function (ec5744_caninterruptslb1): '<S344>/ReceiveandTransmitInterrupt' */
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
void VehCtrlMdel240926_2018b_amkspdlimit_SetEventsForThisBaseStep(boolean_T
  *eventFlags)
{
  /* Task runs when its counter is zero, computed via rtmStepTask macro */
  eventFlags[1] = ((boolean_T)rtmStepTask(VehCtrlMdel240926_2018b_amks_M, 1));
  eventFlags[2] = ((boolean_T)rtmStepTask(VehCtrlMdel240926_2018b_amks_M, 2));
  eventFlags[3] = ((boolean_T)rtmStepTask(VehCtrlMdel240926_2018b_amks_M, 3));
  eventFlags[4] = ((boolean_T)rtmStepTask(VehCtrlMdel240926_2018b_amks_M, 4));
  eventFlags[5] = ((boolean_T)rtmStepTask(VehCtrlMdel240926_2018b_amks_M, 5));
  eventFlags[6] = ((boolean_T)rtmStepTask(VehCtrlMdel240926_2018b_amks_M, 6));
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
  (VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[1])++;
  if ((VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[1]) > 1) {/* Sample time: [0.001s, 0.0s] */
    VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[1] = 0;
  }

  (VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[2])++;
  if ((VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[2]) > 9) {/* Sample time: [0.005s, 0.0s] */
    VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[2] = 0;
  }

  (VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[3])++;
  if ((VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[3]) > 19) {/* Sample time: [0.01s, 0.0s] */
    VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[3] = 0;
  }

  (VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[4])++;
  if ((VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[4]) > 99) {/* Sample time: [0.05s, 0.0s] */
    VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[4] = 0;
  }

  (VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[5])++;
  if ((VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[5]) > 199) {/* Sample time: [0.1s, 0.0s] */
    VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[5] = 0;
  }

  (VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[6])++;
  if ((VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[6]) > 999) {/* Sample time: [0.5s, 0.0s] */
    VehCtrlMdel240926_2018b_amks_M->Timing.TaskCounters.TID[6] = 0;
  }
}

/*
 * Output and update for atomic system:
 *    '<S8>/Timer1'
 *    '<S8>/Timer2'
 *    '<S203>/Timer'
 *    '<S204>/Timer'
 *    '<S204>/Timer1'
 *    '<S204>/Timer2'
 *    '<S204>/Timer3'
 *    '<S262>/Timer'
 *    '<S262>/Timer1'
 *    '<S262>/Timer2'
 *    ...
 */
void VehCtrlMdel240926_20_Timer1(boolean_T rtu_Trigger, real32_T rtu_CountTime,
  real_T *rty_Exit, DW_Timer1_VehCtrlMdel240926_2_T *localDW)
{
  boolean_T sf_internal_predicateOutput;

  /* Chart: '<S8>/Timer1' */
  if (localDW->bitsForTID3.is_active_c5_VehCtrlMdel240926_ == 0U) {
    localDW->bitsForTID3.is_active_c5_VehCtrlMdel240926_ = 1U;
    localDW->bitsForTID3.is_c5_VehCtrlMdel240926_2018b_a = 3U;
    localDW->x += 0.01;
    *rty_Exit = 0.0;
  } else {
    switch (localDW->bitsForTID3.is_c5_VehCtrlMdel240926_2018b_a) {
     case VehCtrlMdel240926_IN_InterState:
      if (rtu_Trigger) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel240926_2018b_a = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      }
      break;

     case VehCtrlMdel240926_2018b__IN_Out:
      *rty_Exit = 1.0;
      if (!rtu_Trigger) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel240926_2018b_a = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      }
      break;

     default:
      /* case IN_Trigger: */
      *rty_Exit = 0.0;
      sf_internal_predicateOutput = (rtu_Trigger && (localDW->x < rtu_CountTime));
      if (sf_internal_predicateOutput) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel240926_2018b_a = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      } else if (localDW->x >= rtu_CountTime) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel240926_2018b_a = 2U;
        *rty_Exit = 1.0;
        localDW->x = 0.0;
      } else {
        sf_internal_predicateOutput = ((localDW->x < rtu_CountTime) &&
          (!rtu_Trigger));
        if (sf_internal_predicateOutput) {
          localDW->bitsForTID3.is_c5_VehCtrlMdel240926_2018b_a = 1U;
          localDW->x = 0.0;
        }
      }
      break;
    }
  }

  /* End of Chart: '<S8>/Timer1' */
}

/*
 * Output and update for atomic system:
 *    '<S122>/Timer'
 *    '<S201>/Timer'
 */
void VehCtrlMdel240926_201_Timer(boolean_T rtu_Trigger, real32_T rtu_CountTime,
  real_T *rty_Exit, DW_Timer_VehCtrlMdel240926_20_T *localDW)
{
  boolean_T sf_internal_predicateOutput;

  /* Chart: '<S122>/Timer' */
  if (localDW->bitsForTID3.is_active_c21_VehCtrlMdel240926 == 0U) {
    localDW->bitsForTID3.is_active_c21_VehCtrlMdel240926 = 1U;
    localDW->bitsForTID3.is_c21_VehCtrlMdel240926_2018b_ = 3U;
    localDW->x += 0.01;
    *rty_Exit = 0.0;
  } else {
    switch (localDW->bitsForTID3.is_c21_VehCtrlMdel240926_2018b_) {
     case VehCtrlMdel2409_IN_InterState_n:
      if (rtu_Trigger) {
        localDW->bitsForTID3.is_c21_VehCtrlMdel240926_2018b_ = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      }
      break;

     case VehCtrlMdel240926_2018_IN_Out_n:
      *rty_Exit = 1.0;
      if (!rtu_Trigger) {
        localDW->bitsForTID3.is_c21_VehCtrlMdel240926_2018b_ = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      }
      break;

     default:
      /* case IN_Trigger: */
      *rty_Exit = 0.0;
      sf_internal_predicateOutput = (rtu_Trigger && (localDW->x < rtu_CountTime));
      if (sf_internal_predicateOutput) {
        localDW->bitsForTID3.is_c21_VehCtrlMdel240926_2018b_ = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      } else if (localDW->x >= rtu_CountTime) {
        localDW->bitsForTID3.is_c21_VehCtrlMdel240926_2018b_ = 2U;
        *rty_Exit = 1.0;
        localDW->x = 0.0;
      } else {
        sf_internal_predicateOutput = ((localDW->x < rtu_CountTime) &&
          (!rtu_Trigger));
        if (sf_internal_predicateOutput) {
          localDW->bitsForTID3.is_c21_VehCtrlMdel240926_2018b_ = 1U;
          localDW->x = 0.0;
        }
      }
      break;
    }
  }

  /* End of Chart: '<S122>/Timer' */
}

/* Function for Chart: '<S105>/Chart2' */
static void VehC_enter_atomic_WaitForEngine(void)
{
  int32_T b_previousEvent;
  b_previousEvent = VehCtrlMdel240926_2018b_amks_DW.sfEvent;
  VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel2409_event_AMKCANOFF;
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_AMKCANenable != 0U)
  {
    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKCANenable) {
     case VehCtrlMdel240926_2018b__IN_OFF:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel24092_event_AMKCANON) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 2U;
      }
      break;

     case VehCtrlMdel240926_2018b_a_IN_ON:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel2409_event_AMKCANOFF) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel24092_event_AMKDCOFF;
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_AMKDCon != 0U) {
    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCon) {
     case VehCtrlMdel240926_2018b__IN_OFF:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel240926_event_AMKDCON) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCon = 2U;
      }
      break;

     case VehCtrlMdel240926_2018b_a_IN_ON:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel24092_event_AMKDCOFF) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCon = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel_event_MCDCEnableOFF;
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MCDCEnable != 0U) {
    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCDCEnable) {
     case VehCtrlMdel240926_2018b__IN_OFF:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel2_event_MCDCEnableON) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCDCEnable = 2U;
      }
      break;

     case VehCtrlMdel240926_2018b_a_IN_ON:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel_event_MCDCEnableOFF) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCDCEnable = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel24_event_InverterOFF;
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MC_InverterOn != 0U)
  {
    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_InverterOn) {
     case VehCtrlMdel240926_2018b__IN_OFF:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel240_event_InverterON) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_InverterOn = 2U;
      }
      break;

     case VehCtrlMdel240926_2018b_a_IN_ON:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel24_event_InverterOFF) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_InverterOn = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel2409_event_TorqueOFF;
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MC_TorqueCUT != 0U)
  {
    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT) {
     case VehCtrlMdel240926_2018b__IN_OFF:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel24092_event_TorqueON) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT = 2U;
      }
      break;

     case VehCtrlMdel240926_2018b_a_IN_ON:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel2409_event_TorqueOFF) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240926_2018b_amks_DW.sfEvent = b_previousEvent;
  VehCtrlMdel240926_2018b_amksp_B.errorReset = 0.0;
}

/* Function for Chart: '<S105>/Chart2' */
static void VehCtrlMdel240926_2018b_VehStat(const real_T *MCFL_bQuitInverterOn_k,
  const real_T *MCFL_bSystemReady_i, const real_T *controller_ready_e, const
  real_T *MCFR_bQuitInverterOn_d, const real_T *MCFR_bSystemReady_g)
{
  boolean_T sf_internal_predicateOutput;
  int32_T b_previousEvent;
  switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat) {
   case VehCtrlMdel240926_2018_IN_Guard:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 >= 25U) {
      b_previousEvent = VehCtrlMdel240926_2018b_amks_DW.sfEvent;
      VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel240926_event_EbeepON;
      if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_BeeperStat != 0U)
      {
        switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_BeeperStat) {
         case VehCtrlMdel240926_2018b__IN_OFF:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel240926_event_EbeepON) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_BeeperStat = 2U;
          }
          break;

         case VehCtrlMdel240926_2018b_a_IN_ON:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel24092_event_EbeepOFF) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_BeeperStat = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240926_2018b_amks_DW.sfEvent = b_previousEvent;
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 7U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
    } else {
      sf_internal_predicateOutput = ((!KeyPressed) || (!Brk) || (!ACC_Release));
      if (sf_internal_predicateOutput) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 6U;
      } else {
        sf_internal_predicateOutput = ((!(*controller_ready_e != 0.0)) ||
          (!(*MCFL_bSystemReady_i != 0.0)) || (!(*MCFR_bSystemReady_g != 0.0)));
        if (sf_internal_predicateOutput) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 3U;
        }
      }
    }
    break;

   case VehCtrlMdel240926_2018b_IN_Init:
    VehCtrlMdel240926_2018b_amksp_B.errorReset = 1.0;
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 >= 10U) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 8U;
      VehC_enter_atomic_WaitForEngine();
    }
    break;

   case VehCtrlMdel240_IN_InitStateBack:
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 8U;
    VehC_enter_atomic_WaitForEngine();
    break;

   case VehCtrlMdel2409_IN_MCUReadyFail:
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 8U;
    VehC_enter_atomic_WaitForEngine();
    break;

   case VehCtrlMdel240926_2018_IN_Ready:
    sf_internal_predicateOutput = ((!(*controller_ready_e != 0.0)) ||
      (!(*MCFL_bSystemReady_i != 0.0)) || (!(*MCFR_bSystemReady_g != 0.0)));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 4U;
    }
    break;

   case VehCtrlMdel240926_20_IN_Standby:
    sf_internal_predicateOutput = ((!(*MCFL_bSystemReady_i != 0.0)) ||
      (!(*MCFR_bSystemReady_g != 0.0)) || (!(*controller_ready_e != 0.0)));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 8U;
      VehC_enter_atomic_WaitForEngine();
    } else {
      sf_internal_predicateOutput = (KeyPressed && Brk && ACC_Release &&
        (*controller_ready_e != 0.0) && (*MCFL_bSystemReady_i != 0.0) &&
        (*MCFR_bSystemReady_g != 0.0) && (*MCFL_bQuitInverterOn_k != 0.0) &&
        (*MCFR_bQuitInverterOn_d != 0.0));
      if (sf_internal_predicateOutput) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 1U;
        VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
      }
    }
    break;

   case VehCtrlMdel240926_2018_IN_Trans:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 >= 250U) {
      b_previousEvent = VehCtrlMdel240926_2018b_amks_DW.sfEvent;
      VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel24092_event_EbeepOFF;
      if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_BeeperStat != 0U)
      {
        switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_BeeperStat) {
         case VehCtrlMdel240926_2018b__IN_OFF:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel240926_event_EbeepON) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_BeeperStat = 2U;
          }
          break;

         case VehCtrlMdel240926_2018b_a_IN_ON:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel24092_event_EbeepOFF) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_BeeperStat = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240926_2018b_amks_DW.sfEvent = b_previousEvent;
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 5U;
    }
    break;

   case VehCtrlMdel240_IN_WaitForEngine:
    VehCtrlMdel240926_2018b_amksp_B.errorReset = 0.0;
    sf_internal_predicateOutput = ((*MCFL_bSystemReady_i != 0.0) &&
      (*MCFR_bSystemReady_g != 0.0) && (*controller_ready_e != 0.0));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 6U;
    }
    break;
  }
}

/* Function for Chart: '<S105>/Chart2' */
static void VehCtrlMdel_enter_atomic_AMKCAN(void)
{
  int32_T b_previousEvent;
  b_previousEvent = VehCtrlMdel240926_2018b_amks_DW.sfEvent;
  VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel24092_event_AMKCANON;
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_AMKCANenable != 0U)
  {
    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKCANenable) {
     case VehCtrlMdel240926_2018b__IN_OFF:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel24092_event_AMKCANON) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 2U;
      }
      break;

     case VehCtrlMdel240926_2018b_a_IN_ON:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel2409_event_AMKCANOFF) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240926_2018b_amks_DW.sfEvent = b_previousEvent;
}

/* Function for Chart: '<S105>/Chart2' */
static void VehCtrlMdel240926_20_AMKDCready(const real_T *MCFL_bDCOn_j, const
  real_T *MCFL_bQuitInverterOn_k, const real_T *MCFL_bSystemReady_i, const
  real_T *MCFR_bDCOn_n, const real_T *MCFR_bQuitInverterOn_d, const real_T
  *MCFR_bSystemReady_g)
{
  boolean_T sf_internal_predicateOutput;
  int32_T e_previousEvent;
  switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready) {
   case VehCtrlMdel240926_201_IN_AMKCAN:
    sf_internal_predicateOutput = (KeyPressed && Brk && ACC_Release);
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 6U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
    }
    break;

   case VehCtrlMdel240_IN_AMKDCOnFinish:
    if (KeyPressed) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 1U;
      VehCtrlMdel_enter_atomic_AMKCAN();
    }
    break;

   case VehCtrlMdel240_IN_DCOnCheckPass:
    sf_internal_predicateOutput =
      ((VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 100U) &&
       (*MCFL_bDCOn_j != 0.0) && (*MCFR_bDCOn_n != 0.0));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 4U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
      e_previousEvent = VehCtrlMdel240926_2018b_amks_DW.sfEvent;
      VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel2_event_MCDCEnableON;
      if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MCDCEnable != 0U)
      {
        switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCDCEnable) {
         case VehCtrlMdel240926_2018b__IN_OFF:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel2_event_MCDCEnableON) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCDCEnable = 2U;
          }
          break;

         case VehCtrlMdel240926_2018b_a_IN_ON:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel_event_MCDCEnableOFF) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCDCEnable = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240926_2018b_amks_DW.sfEvent = e_previousEvent;
    }
    break;

   case VehCtrlMdel2_IN_MCDCEnableState:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 100U) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 7U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
      e_previousEvent = VehCtrlMdel240926_2018b_amks_DW.sfEvent;
      VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel240_event_InverterON;
      if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MC_InverterOn !=
          0U) {
        switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_InverterOn) {
         case VehCtrlMdel240926_2018b__IN_OFF:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel240_event_InverterON) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_InverterOn = 2U;
          }
          break;

         case VehCtrlMdel240926_2018b_a_IN_ON:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel24_event_InverterOFF) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_InverterOn = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240926_2018b_amks_DW.sfEvent = e_previousEvent;
    }
    break;

   case VehCtrlMdel24092_IN_MCDCOncheck:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 100U) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 3U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
    }
    break;

   case VehCtrlMdel240926_IN_MC_DCready:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 100U) {
      e_previousEvent = VehCtrlMdel240926_2018b_amks_DW.sfEvent;
      VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel240926_event_AMKDCON;
      if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_AMKDCon != 0U) {
        switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCon) {
         case VehCtrlMdel240926_2018b__IN_OFF:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel240926_event_AMKDCON) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCon = 2U;
          }
          break;

         case VehCtrlMdel240926_2018b_a_IN_ON:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel24092_event_AMKDCOFF) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCon = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240926_2018b_amks_DW.sfEvent = e_previousEvent;
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 5U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
    }
    break;

   case VehCtrlM_IN_MC_InverterOn_State:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 100U) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 8U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
    }
    break;

   case VehCtrlMdel240926_IN_SYSRDYCECK:
    sf_internal_predicateOutput =
      ((VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 800U) &&
       (*MCFL_bSystemReady_i != 0.0) && (*MCFR_bSystemReady_g != 0.0) &&
       (*MCFL_bQuitInverterOn_k != 0.0) && (*MCFR_bQuitInverterOn_d != 0.0));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 2U;
      e_previousEvent = VehCtrlMdel240926_2018b_amks_DW.sfEvent;
      VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel24092_event_TorqueON;
      if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MC_TorqueCUT !=
          0U) {
        switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT) {
         case VehCtrlMdel240926_2018b__IN_OFF:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel24092_event_TorqueON) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT = 2U;
          }
          break;

         case VehCtrlMdel240926_2018b_a_IN_ON:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel2409_event_TorqueOFF) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240926_2018b_amks_DW.sfEvent = e_previousEvent;
    }
    break;

   case VehCtrlMdel240926_2018_IN_start:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 500U) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 1U;
      VehCtrlMdel_enter_atomic_AMKCAN();
    }
    break;
  }
}

/* Model step function for TID0 */
void VehCtrlMdel240926_2018b_amkspdlimit_step0(void) /* Sample time: [0.0005s, 0.0s] */
{
  {                                    /* Sample time: [0.0005s, 0.0s] */
    rate_monotonic_scheduler();
  }
}

/* Model step function for TID1 */
void VehCtrlMdel240926_2018b_amkspdlimit_step1(void) /* Sample time: [0.001s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S344>/Function-Call Generator' incorporates:
   *  SubSystem: '<S344>/CCPBackground'
   */

  /* S-Function (ec5744_ccpslb): '<S359>/CCPBackground' */
  ccpBackground();
  Lin0_Background();

  /* End of Outputs for S-Function (fcncallgen): '<S344>/Function-Call Generator' */
}

/* Model step function for TID2 */
void VehCtrlMdel240926_2018b_amkspdlimit_step2(void) /* Sample time: [0.005s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S342>/5ms' incorporates:
   *  SubSystem: '<S342>/daq5ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S357>/CCPDAQ' */
  ccpDaq(0);

  /* End of Outputs for S-Function (fcncallgen): '<S342>/5ms' */
}

/* Model step function for TID3 */
void VehCtrlMdel240926_2018b_amkspdlimit_step3(void) /* Sample time: [0.01s, 0.0s] */
{
  boolean_T rtb_ignition;
  real_T rtb_Gain5;
  real_T rtb_Yk1_l;
  real_T rtb_Gain4;
  real_T rtb_Switch2_on;
  real32_T rtb_Add;
  real32_T rtb_Acc_POS;
  boolean_T rtb_LogicalOperator2;
  boolean_T rtb_Compare;
  boolean_T rtb_Compare_am;
  boolean_T rtb_LowerRelop1_b;
  real_T elapseTime;
  real_T rtb_UkYk1;
  real_T rtb_Add2;
  real_T rtb_g_mpss1;
  real_T rtb_Add1;
  real_T rtb_UkYk1_ix;
  real_T rtb_StrWhlAngV;
  real32_T rtb_Add4_j;
  real32_T rtb_Add7;
  real32_T rtb_Add6_p;
  real32_T rtb_Switch2_mn;
  real32_T rtb_Divide;
  real32_T rtb_Switch2_b0;
  real32_T rtb_CastToDouble;
  boolean_T rtb_AND2;
  boolean_T rtb_Compare_b;
  real32_T rtb_VxIMU_est;
  real32_T rtb_Ax;
  real32_T rtb_Saturation1_i;
  real32_T rtb_UkYk1_ea;
  real32_T rtb_Add10;
  real32_T rtb_Switch_jg;
  real_T rtb_deltafalllimit_iz;
  real32_T rtb_deltafalllimit_aw;
  real32_T rtb_deltafalllimit_i;
  boolean_T rtb_UpperRelop_ir;
  real32_T rtb_MaxWhlSpd_mps_n;
  uint32_T rtb_Gain1_h;
  uint32_T rtb_Gain;
  uint32_T FunctionCallSubsystem_ELAPS_T;
  int32_T Brk_F;
  real_T WhlSpdFL;
  real_T WhlSpdFR;
  real_T WhlSpdRR_mps;
  real_T WhlSpdRL_mps;
  real_T FLWhlStrAng;
  boolean_T rtb_LogicalOperator_idx_0;

  /* S-Function (fcncallgen): '<S3>/10ms7' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* SignalConversion generated from: '<S122>/Constant' */
  HVCUTOFF = true;

  /* S-Function (ec5744_swislbu3): '<S122>/SwitchInput' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240926_2018b_amksp_B.Drive_ready= ec_gpio_read(92);

  /* Logic: '<S122>/Logical Operator' */
  rtb_ignition = !VehCtrlMdel240926_2018b_amksp_B.Drive_ready;

  /* Chart: '<S122>/Timer' incorporates:
   *  Constant: '<S122>/Constant5'
   */
  VehCtrlMdel240926_201_Timer(rtb_ignition, 0.11F, &ignition,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer);

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms7' */

  /* S-Function (fcncallgen): '<S3>/10ms6' incorporates:
   *  SubSystem: '<S3>/EMRAXMCU_RECIEVE'
   */
  /* S-Function (ec5744_canreceiveslb): '<S119>/CANReceive1' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF1RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf1looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o3= 218089455;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o5= 8;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2= ec_can_receive(0,1,
      CAN0BUF1RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[0]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[1]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[2]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[3]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[4]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[5]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[6]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[7]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
  }

  /* Call the system: <S119>/MCU_pwr */

  /* Output and update for function-call system: '<S119>/MCU_pwr' */

  /* Outputs for Enabled SubSystem: '<S172>/MCU_VCUMeter1' incorporates:
   *  EnablePort: '<S174>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S174>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Length = 8;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.ID =
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o3;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Extended = 1;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[0];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[1];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[2];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[3];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[4];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[5];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[6];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S174>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S174>/CAN Unpack' */
      if ((8 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Length) &&
          (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.ID !=
           INVALID_CAN_ID) ) {
        if ((218089455 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.ID)
            && (1U ==
                VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Extended) )
        {
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data
                       [6]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data
                       [7]) << 8);
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data
                       [4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data
                       [5]) << 8);
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data
                       [0]);
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data
                       [1]);
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data
                       [2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_g.Data
                       [3]) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.1;
                VehCtrlMdel240926_2018b_amksp_B.voltage = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S172>/MCU_VCUMeter1' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S119>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S119>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF0RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf0looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3= 218089199;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o5= 8;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2= ec_can_receive(0,0,
      CAN0BUF0RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[0]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[1]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[2]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[3]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[4]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[5]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[6]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[7]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
  }

  /* Call the system: <S119>/MCU_state */

  /* Output and update for function-call system: '<S119>/MCU_state' */

  /* Outputs for Enabled SubSystem: '<S173>/MCU_state' incorporates:
   *  EnablePort: '<S179>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S179>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Length = 8;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.ID =
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Extended = 1;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[0];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[1];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[2];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[3];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[4];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[5];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[6];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S179>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S179>/CAN Unpack' */
      if ((8 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Length) &&
          (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.ID !=
           INVALID_CAN_ID) ) {
        if ((218089199 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.ID) &&
            (1U == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Extended) )
        {
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[5])
                      & (uint8_T)(0x20U)) >> 5);
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[7])
                      & (uint8_T)(0x3U));
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[5])
                      & (uint8_T)(0x40U)) >> 6);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.low_VOL = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[5])
                      & (uint8_T)(0x1U));
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[6])
                      & (uint8_T)(0x1U));
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.MCU_Temp_error = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[4]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.Mode = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[6])
                      & (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.motorTemp_error = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[5])
                      & (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.overCurrent = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[5])
                      & (uint8_T)(0x80U)) >> 7);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.overpower = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[5])
                      & (uint8_T)(0x10U)) >> 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.overvol = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[5])
                      & (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.Precharge = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[5])
                      & (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.Reslove_error = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[6])
                      & (uint8_T)(0x80U)) >> 7);
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[0]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[1])
                      << 8);
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4.Data[3])
                      << 8);
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

  /* End of Outputs for SubSystem: '<S173>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S119>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms6' */

  /* S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S115>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF16RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf16looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_m= 1698;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o5_b= 8;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_m= ec_can_receive(1,16,
      CAN1BUF16RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[0]=
      CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[1]=
      CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[2]=
      CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[3]=
      CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[4]=
      CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[5]=
      CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[6]=
      CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[7]=
      CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
  }

  /* Call the system: <S115>/ABS_BUS_state */

  /* Output and update for function-call system: '<S115>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S123>/IMU_state' incorporates:
   *  EnablePort: '<S124>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_m > 0) {
    /* S-Function (ecucoder_canunmessage): '<S124>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Length = 8;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.ID =
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_m;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Extended = 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[0];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[1];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[2];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[3];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[4];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[5];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[6];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S124>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S124>/CAN Unpack1' */
      if ((8 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Length) &&
          (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.ID !=
           INVALID_CAN_ID) ) {
        if ((1698 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.ID) &&
            (0U == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Extended)
            ) {
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[
                       1]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[
                       0]) << 8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.014063;
                ABS_WS_FL = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[
                       3]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[
                       2]) << 8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.014063;
                ABS_WS_FR = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[
                       5]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[
                       4]) << 8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.014063;
                ABS_WS_RL = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[
                       7]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_ja.Data[
                       6]) << 8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.014063;
                ABS_WS_RR = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S123>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S115>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms3' */

  /* S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF7RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf7looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_c= 330;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o5_d= 8;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_p= ec_can_receive(1,7,
      CAN1BUF7RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[0]=
      CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[1]=
      CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[2]=
      CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[3]=
      CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[4]=
      CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[5]=
      CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[6]=
      CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[7]=
      CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
  }

  /* Call the system: <S121>/StrWhSnis_state */

  /* Output and update for function-call system: '<S121>/StrWhSnis_state' */

  /* Outputs for Enabled SubSystem: '<S191>/IMU_state' incorporates:
   *  EnablePort: '<S192>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S192>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Length = 8;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.ID =
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_c;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Extended = 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[0];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[1];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[2];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[3];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[4];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[5];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[6];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S192>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S192>/CAN Unpack1' */
      if ((8 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Length) &&
          (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.ID !=
           INVALID_CAN_ID) ) {
        if ((330 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.ID) &&
            (0U == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Extended)
            ) {
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Data
                       [0]);
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Data
                       [4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Data
                       [3]) << 8);
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_l.Data
                       [2]);
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

  /* End of Outputs for SubSystem: '<S191>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms4' */

  /* S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S129>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF1RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf1looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_e= 640;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o5_a= 8;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_l= ec_can_receive(1,1,
      CAN1BUF1RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[0]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[1]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[2]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[3]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[4]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[5]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[6]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[7]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
  }

  /* Call the system: <S129>/AMKMCU_state */

  /* Output and update for function-call system: '<S129>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S131>/MCU_state' incorporates:
   *  EnablePort: '<S134>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S134>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Length = 8;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.ID =
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_e;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Extended = 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[0];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[1];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[2];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[3];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[4];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[5];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[6];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_l[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S134>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S134>/CAN Unpack' */
      if ((8 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Length) &&
          (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.ID !=
           INVALID_CAN_ID) ) {
        if ((640 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.ID) &&
            (0U == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Extended)
            ) {
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [5]) << 8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.0098;
                VehCtrlMdel240926_2018b_amksp_B.MCFL_ActualTorque = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [6]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [7]) << 8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.MCFL_ActualVelocity_p = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [3]) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFL_DCVoltage = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [1]) & (uint8_T)(0x10U)) >> 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFL_bDCOn = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [1]) & (uint8_T)(0x80U)) >> 7);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.MCFL_bDerating = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [1]) & (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFL_bError = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [1]) & (uint8_T)(0x40U)) >> 6);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFL_bInverterOn = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [1]) & (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFL_bQuitDCOn = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [1]) & (uint8_T)(0x20U)) >> 5);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFL_bQuitInverterOn = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [0]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.MCFL_bReserve = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [1]) & (uint8_T)(0x1U));
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFL_bSystemReady = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [1]) & (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.MCFL_bWarn = result;
              }
            }
          }
        }
      }
    }

    /* Gain: '<S134>/Gain' */
    MCFL_ActualVelocity = -VehCtrlMdel240926_2018b_amksp_B.MCFL_ActualVelocity_p;
  }

  /* End of Outputs for SubSystem: '<S131>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S129>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S129>/CANReceive1' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF2RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf2looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o3_n= 642;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o5_a= 8;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2_l= ec_can_receive(1,2,
      CAN1BUF2RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[0]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[1]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[2]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[3]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[4]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[5]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[6]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[7]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
  }

  /* Call the system: <S129>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S129>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S132>/MCU_state' incorporates:
   *  EnablePort: '<S143>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S143>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Length = 8;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.ID =
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o3_n;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Extended = 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[0];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[1];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[2];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[3];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[4];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[5];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[6];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_c[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S143>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S143>/CAN Unpack' */
      if ((8 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Length) &&
          (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.ID !=
           INVALID_CAN_ID) ) {
        if ((642 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.ID) &&
            (0U == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Extended)
            ) {
          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 0
             *  length                  = 32
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint32_T unpackedValue = 0;

                {
                  uint32_T tempValue = (uint32_T) (0);

                  {
                    tempValue = tempValue | (uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [0]);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [1]) << 8);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [2]) << 16);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [3]) << 24);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.MCFL_DiagnosticNum = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 32
             *  length                  = 32
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint32_T unpackedValue = 0;

                {
                  uint32_T tempValue = (uint32_T) (0);

                  {
                    tempValue = tempValue | (uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [4]);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [5]) << 8);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [6]) << 16);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [7]) << 24);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFL_ErrorInfo = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S132>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S129>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S129>/CANReceive2' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF3RX[6]= { 0, 0, 0, 0, 0, 0 };

    uint8 can1buf3looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o3= 644;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o5= 6;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o2= ec_can_receive(1,3,
      CAN1BUF3RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4[0]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4[1]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4[2]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4[3]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4[4]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4[5]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
  }

  /* Call the system: <S129>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S129>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S133>/MCU_state' incorporates:
   *  EnablePort: '<S145>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S145>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Length = 6;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.ID =
        VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o3;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Extended = 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4[0];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4[1];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4[2];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4[3];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4[4];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4[5];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S145>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S145>/CAN Unpack' */
      if ((6 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Length) &&
          (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.ID !=
           INVALID_CAN_ID) ) {
        if ((644 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.ID) &&
            (0U == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Extended)
            ) {
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Data
                       [4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Data
                       [5]) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.1;
                MCFL_TempIGBT = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Data
                       [2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Data
                       [3]) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.1;
                MCFL_TempInverter = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Data
                       [0]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_c.Data
                       [1]) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.1;
                MCFL_TempMotor = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S133>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S129>/CANReceive2' */

  /* S-Function (ec5744_canreceiveslb): '<S130>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF4RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf4looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_i= 641;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o5_an= 8;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_a= ec_can_receive(1,4,
      CAN1BUF4RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[0]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[1]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[2]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[3]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[4]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[5]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[6]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[7]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
  }

  /* Call the system: <S130>/AMKMCU_state */

  /* Output and update for function-call system: '<S130>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S149>/MCU_state' incorporates:
   *  EnablePort: '<S152>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_a > 0) {
    /* S-Function (ecucoder_canunmessage): '<S152>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Length = 8;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.ID =
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_i;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Extended = 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[0];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[1];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[2];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[3];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[4];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[5];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[6];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_g[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S152>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S152>/CAN Unpack' */
      if ((8 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Length) &&
          (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.ID !=
           INVALID_CAN_ID) ) {
        if ((641 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.ID) &&
            (0U == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Extended)
            ) {
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [5]) << 8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.0098;
                VehCtrlMdel240926_2018b_amksp_B.MCFR_ActualTorque = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [6]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [7]) << 8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFR_ActualVelocity = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [3]) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFR_DCVoltage = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [1]) & (uint8_T)(0x10U)) >> 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFR_bDCOn = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [1]) & (uint8_T)(0x80U)) >> 7);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.MCFR_bDerating = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [1]) & (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFR_bError = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [1]) & (uint8_T)(0x40U)) >> 6);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFR_bInverterOn = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [1]) & (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.MCFR_bQuitDCOn = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [1]) & (uint8_T)(0x20U)) >> 5);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFR_bQuitInverterOn = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [0]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.MCFR_bReserve = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [1]) & (uint8_T)(0x1U));
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFR_bSystemReady = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [1]) & (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.MCFR_bWarn = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S149>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S130>/CANReceive1' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF5RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf5looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o3_h= 643;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o5_j= 8;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2_o= ec_can_receive(1,5,
      CAN1BUF5RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[0]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[1]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[2]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[3]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[4]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[5]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[6]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[7]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
  }

  /* Call the system: <S130>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S130>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S150>/MCU_state' incorporates:
   *  EnablePort: '<S160>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2_o > 0) {
    /* S-Function (ecucoder_canunmessage): '<S160>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Length = 8;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.ID =
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o3_h;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Extended = 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[0];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[1];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[2];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[3];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[4];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[5];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[6];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o4_j[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S160>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S160>/CAN Unpack' */
      if ((8 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Length) &&
          (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.ID !=
           INVALID_CAN_ID) ) {
        if ((643 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.ID) &&
            (0U == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Extended)
            ) {
          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 0
             *  length                  = 32
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint32_T unpackedValue = 0;

                {
                  uint32_T tempValue = (uint32_T) (0);

                  {
                    tempValue = tempValue | (uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [0]);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [1]) << 8);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [2]) << 16);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [3]) << 24);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.MCFR_DiagnosticNum = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 32
             *  length                  = 32
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint32_T unpackedValue = 0;

                {
                  uint32_T tempValue = (uint32_T) (0);

                  {
                    tempValue = tempValue | (uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [4]);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [5]) << 8);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [6]) << 16);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [7]) << 24);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFR_ErrorInfo = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S150>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S130>/CANReceive2' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF0RX[6]= { 0, 0, 0, 0, 0, 0 };

    uint8 can1buf0looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o3_j= 645;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o5_e= 6;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o2_p= ec_can_receive(1,0,
      CAN1BUF0RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4_k[0]=
      CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4_k[1]=
      CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4_k[2]=
      CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4_k[3]=
      CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4_k[4]=
      CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4_k[5]=
      CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
  }

  /* Call the system: <S130>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S130>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S151>/MCU_state' incorporates:
   *  EnablePort: '<S162>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S162>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Length = 6;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.ID =
        VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o3_j;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Extended = 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4_k[0];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4_k[1];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4_k[2];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4_k[3];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4_k[4];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o4_k[5];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S162>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S162>/CAN Unpack' */
      if ((6 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Length) &&
          (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.ID !=
           INVALID_CAN_ID) ) {
        if ((645 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.ID) &&
            (0U == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Extended)
            ) {
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Data
                       [4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Data
                       [5]) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.1;
                MCFR_TempIGBT = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Data
                       [2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Data
                       [3]) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.1;
                MCFR_TempInverter = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Data
                       [0]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_n.Data
                       [1]) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.1;
                MCFR_TempMotor = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S151>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive2' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms5' */

  /* S-Function (fcncallgen): '<S3>/10ms' incorporates:
   *  SubSystem: '<S3>/AccBrk_BUS'
   */
  /* S-Function (ec5744_asislbu3): '<S117>/Acc3' */

  /* Read the ADC conversion result of the analog signal */
  Acc1= adc_read_chan(1,2);

  /* S-Function (ec5744_asislbu3): '<S117>/Acc4' */

  /* Read the ADC conversion result of the analog signal */
  Acc2= adc_read_chan(1,4);

  /* S-Function (ec5744_asislbu3): '<S117>/Brk1' */

  /* Read the ADC conversion result of the analog signal */
  Brk1= adc_read_chan(1,0);

  /* S-Function (ec5744_asislbu3): '<S117>/Brk2' */

  /* Read the ADC conversion result of the analog signal */
  Brk2= adc_read_chan(0,13);

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms' */

  /* S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* S-Function (ec5744_canreceiveslb): '<S120>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF17RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf17looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_cz= 513;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o5_m= 8;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_ma= ec_can_receive(1,17,
      CAN1BUF17RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[0]=
      CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[1]=
      CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[2]=
      CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[3]=
      CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[4]=
      CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[5]=
      CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[6]=
      CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[7]=
      CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
  }

  /* Call the system: <S120>/IMU_state */

  /* Output and update for function-call system: '<S120>/IMU_state' */

  /* Outputs for Enabled SubSystem: '<S186>/MCU_state' incorporates:
   *  EnablePort: '<S187>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_ma > 0) {
    /* S-Function (ecucoder_canunmessage): '<S187>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Length = 8;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.ID =
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_cz;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Extended = 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[0];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[1];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[2];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[3];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[4];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[5];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[6];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S187>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S187>/CAN Unpack' */
      if ((8 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Length) &&
          (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.ID !=
           INVALID_CAN_ID) ) {
        if ((513 == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.ID) &&
            (0U == VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Extended)
            ) {
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [0]) & (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.CANUnpack_o1 = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [5]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [6]) << 8);
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [0]) & (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.CANUnpack_o3 = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [3]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [4]) << 8);
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [0]) & (uint8_T)(0xF0U)) >> 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.CANUnpack_o5 = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [0]) & (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240926_2018b_amksp_B.CANUnpack_o6 = result;
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
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [1]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [2]) << 8);
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

  /* End of Outputs for SubSystem: '<S186>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S120>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms2' */

  /* S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S118>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF3RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf3looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_l= 408961267;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o5_de= 8;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_k= ec_can_receive(0,3,
      CAN0BUF3RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[0]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[1]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[2]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[3]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[4]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[5]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[6]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[7]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
  }

  /* Call the system: <S118>/ABS_BUS_state */

  /* Output and update for function-call system: '<S118>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S170>/IMU_state' incorporates:
   *  EnablePort: '<S171>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_k > 0) {
    /* S-Function (ecucoder_canunmessage): '<S171>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_i.Length = 8;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_i.ID =
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_l;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_i.Extended = 1;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[0];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[1];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[2];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[3];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[4];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[5];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[6];
      canunpackloop++;
      VehCtrlMdel240926_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_p[7];
      canunpackloop++;
    }
  }

  /* End of Outputs for SubSystem: '<S170>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S118>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms1' */

  /* S-Function (fcncallgen): '<S4>/10ms' incorporates:
   *  SubSystem: '<S4>/Function-Call Subsystem'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.FunctionCallSubsystem_RESET_ELA) {
    FunctionCallSubsystem_ELAPS_T = 0U;
  } else {
    FunctionCallSubsystem_ELAPS_T =
      VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3 -
      VehCtrlMdel240926_2018b_amks_DW.FunctionCallSubsystem_PREV_T;
  }

  VehCtrlMdel240926_2018b_amks_DW.FunctionCallSubsystem_PREV_T =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
  VehCtrlMdel240926_2018b_amks_DW.FunctionCallSubsystem_RESET_ELA = false;

  /* Lookup_n-D: '<S201>/1-D Lookup Table1' */
  F_BrkPrs = look1_iu16bflftfIu16_binlc(Brk1,
    VehCtrlMdel240926_2018b__ConstP.pooled67,
    VehCtrlMdel240926_2018b__ConstP.pooled67, 1U);

  /* DataTypeConversion: '<S201>/Data Type Conversion' */
  rtb_Add = F_BrkPrs;

  /* SignalConversion generated from: '<S199>/Out1' */
  Brk_F = (int32_T)rtb_Add;

  /* Gain: '<S201>/Gain2' */
  rtb_Gain1_h = 45875U * Acc2;

  /* Gain: '<S201>/Gain3' incorporates:
   *  UnitDelay: '<S201>/Unit Delay1'
   */
  rtb_Gain = 39322U * VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S201>/Add3' */
  Acc_vol2 = (rtb_Gain >> 1) + rtb_Gain1_h;

  /* RelationalOperator: '<S209>/Compare' */
  rtb_ignition = (Acc_vol2 <= 32768000U);

  /* RelationalOperator: '<S210>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol2 >= 294912000U);

  /* Logic: '<S201>/Logical Operator1' */
  rtb_ignition = (rtb_ignition || rtb_LogicalOperator2);

  /* Gain: '<S201>/Gain' */
  rtb_Gain = 45875U * Acc1;

  /* UnitDelay: '<S201>/Unit Delay' incorporates:
   *  UnitDelay: '<S201>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_k;

  /* Gain: '<S201>/Gain1' incorporates:
   *  UnitDelay: '<S201>/Unit Delay1'
   */
  rtb_Gain1_h = 39322U * VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S201>/Add2' */
  Acc_vol = (rtb_Gain1_h >> 1) + rtb_Gain;

  /* RelationalOperator: '<S205>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol <= 32768000U);

  /* RelationalOperator: '<S206>/Compare' */
  rtb_Compare = (Acc_vol >= 294912000U);

  /* Logic: '<S201>/Logical Operator' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_Compare);

  /* Logic: '<S201>/Logical Operator2' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_ignition);

  /* DataTypeConversion: '<S201>/Data Type Conversion2' */
  rtb_Add = (real32_T)Acc_vol * 1.52587891E-5F;

  /* MATLAB Function: '<S201>/MATLAB Function' */
  Acc_POS = fmaxf(fminf((rtb_Add - 3060.0F) * -0.4F, 100.0F), 0.0F);

  /* Lookup_n-D: '<S201>/1-D Lookup Table3' */
  Acc_POS2 = look1_iu32n16bflftfIu32_binlc(Acc_vol2,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable3_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable3_tableData, 1U);

  /* DataTypeConversion: '<S201>/Data Type Conversion4' */
  rtb_Add = (real32_T)Acc_POS2 * 1.52587891E-5F;

  /* Sum: '<S201>/Add1' */
  rtb_Acc_POS = Acc_POS - rtb_Add;

  /* Abs: '<S201>/Abs' */
  rtb_Acc_POS = fabsf(rtb_Acc_POS);

  /* RelationalOperator: '<S213>/Compare' incorporates:
   *  Constant: '<S213>/Constant'
   */
  rtb_Compare = (rtb_Acc_POS > 15.0F);

  /* RelationalOperator: '<S211>/Compare' incorporates:
   *  Constant: '<S211>/Constant'
   */
  rtb_ignition = (Acc_POS > 100.0F);

  /* RelationalOperator: '<S212>/Compare' incorporates:
   *  Constant: '<S212>/Constant'
   */
  rtb_Compare_am = (rtb_Add > 100.0F);

  /* Logic: '<S201>/Logical Operator3' */
  rtb_ignition = (rtb_ignition || rtb_Compare_am);

  /* RelationalOperator: '<S214>/Compare' incorporates:
   *  Constant: '<S214>/Constant'
   */
  rtb_Compare_am = (Brk1 <= 300);

  /* RelationalOperator: '<S215>/Compare' incorporates:
   *  Constant: '<S215>/Constant'
   */
  rtb_LowerRelop1_b = (Brk1 >= 4500);

  /* Logic: '<S201>/Logical Operator5' */
  rtb_Compare_am = (rtb_Compare_am || rtb_LowerRelop1_b);

  /* Logic: '<S201>/Logical Operator4' */
  rtb_ignition = (rtb_ignition || rtb_Compare || rtb_LogicalOperator2 ||
                  rtb_Compare_am);

  /* Chart: '<S201>/Timer' incorporates:
   *  Constant: '<S201>/Constant1'
   */
  VehCtrlMdel240926_201_Timer(rtb_ignition, 0.11F, &Trq_CUT,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_a);

  /* UnitDelay: '<S236>/Delay Input2'
   *
   * Block description for '<S236>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE;

  /* SampleTimeMath: '<S236>/sample time'
   *
   * About '<S236>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S236>/delta rise limit' */
  rtb_Gain5 = 1200.0 * elapseTime;

  /* Sum: '<S236>/Difference Inputs1'
   *
   * Block description for '<S236>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = StrWhlAng - rtb_Yk1_l;

  /* RelationalOperator: '<S239>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain5);

  /* Switch: '<S239>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S236>/delta fall limit' */
    rtb_deltafalllimit_iz = -1200.0 * elapseTime;

    /* RelationalOperator: '<S239>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S239>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S239>/Switch' */
    rtb_Gain5 = rtb_UkYk1;
  }

  /* End of Switch: '<S239>/Switch2' */

  /* Sum: '<S236>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S236>/Delay Input2'
   *
   * Block description for '<S236>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S236>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE = rtb_Gain5 + rtb_Yk1_l;

  /* Abs: '<S203>/Abs' incorporates:
   *  UnitDelay: '<S236>/Delay Input2'
   *
   * Block description for '<S236>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain5 = fabs(VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE);

  /* RelationalOperator: '<S235>/Compare' incorporates:
   *  Constant: '<S235>/Constant'
   */
  rtb_ignition = (rtb_Gain5 > 120.0);

  /* Chart: '<S203>/Timer' incorporates:
   *  Constant: '<S203>/Constant5'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_on,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_k);

  /* UnitDelay: '<S249>/Delay Input2'
   *
   * Block description for '<S249>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a;

  /* SampleTimeMath: '<S249>/sample time'
   *
   * About '<S249>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S249>/delta rise limit' */
  rtb_Yk1_l = 10.0 * elapseTime;

  /* Sum: '<S249>/Difference Inputs1'
   *
   * Block description for '<S249>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RL - rtb_Gain5;

  /* RelationalOperator: '<S257>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Yk1_l);

  /* Switch: '<S257>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S249>/delta fall limit' */
    rtb_Yk1_l = -10.0 * elapseTime;

    /* RelationalOperator: '<S257>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_Yk1_l);

    /* Switch: '<S257>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_Yk1_l;
    }

    /* End of Switch: '<S257>/Switch' */
    rtb_Yk1_l = rtb_UkYk1;
  }

  /* End of Switch: '<S257>/Switch2' */

  /* Saturate: '<S204>/Saturation' incorporates:
   *  Sum: '<S249>/Difference Inputs2'
   *  UnitDelay: '<S249>/Delay Input2'
   *
   * Block description for '<S249>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S249>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a = rtb_Yk1_l + rtb_Gain5;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a > 30.0) {
    rtb_Gain5 = 30.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a < 0.0) {
    rtb_Gain5 = 0.0;
  } else {
    rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a;
  }

  /* End of Saturate: '<S204>/Saturation' */

  /* Gain: '<S204>/Gain' */
  rtb_Gain5 *= 0.27777777777777779;

  /* RelationalOperator: '<S241>/Compare' incorporates:
   *  Constant: '<S241>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain5 >= 0.0);

  /* RelationalOperator: '<S242>/Compare' incorporates:
   *  Constant: '<S242>/Constant'
   */
  rtb_Compare_am = (rtb_Gain5 < 40.0);

  /* Logic: '<S204>/OR' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S204>/Timer' incorporates:
   *  Constant: '<S204>/Constant5'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_le,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_b);

  /* UnitDelay: '<S250>/Delay Input2'
   *
   * Block description for '<S250>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b;

  /* SampleTimeMath: '<S250>/sample time'
   *
   * About '<S250>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S250>/delta rise limit' */
  rtb_Gain4 = 10.0 * elapseTime;

  /* Sum: '<S250>/Difference Inputs1'
   *
   * Block description for '<S250>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RR - rtb_Yk1_l;

  /* RelationalOperator: '<S258>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain4);

  /* Switch: '<S258>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S250>/delta fall limit' */
    rtb_deltafalllimit_iz = -10.0 * elapseTime;

    /* RelationalOperator: '<S258>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S258>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S258>/Switch' */
    rtb_Gain4 = rtb_UkYk1;
  }

  /* End of Switch: '<S258>/Switch2' */

  /* Saturate: '<S204>/Saturation1' incorporates:
   *  Sum: '<S250>/Difference Inputs2'
   *  UnitDelay: '<S250>/Delay Input2'
   *
   * Block description for '<S250>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S250>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b = rtb_Gain4 + rtb_Yk1_l;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b > 30.0) {
    rtb_Gain4 = 30.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b < 0.0) {
    rtb_Gain4 = 0.0;
  } else {
    rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b;
  }

  /* End of Saturate: '<S204>/Saturation1' */

  /* Gain: '<S204>/Gain3' */
  rtb_Gain4 *= 0.27777777777777779;

  /* RelationalOperator: '<S243>/Compare' incorporates:
   *  Constant: '<S243>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain4 >= 0.0);

  /* RelationalOperator: '<S244>/Compare' incorporates:
   *  Constant: '<S244>/Constant'
   */
  rtb_Compare_am = (rtb_Gain4 < 40.0);

  /* Logic: '<S204>/OR1' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S204>/Timer1' incorporates:
   *  Constant: '<S204>/Constant1'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_i,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_n);

  /* UnitDelay: '<S251>/Delay Input2'
   *
   * Block description for '<S251>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h;

  /* SampleTimeMath: '<S251>/sample time'
   *
   * About '<S251>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S251>/delta rise limit' */
  rtb_Switch2_on = 10.0 * elapseTime;

  /* Sum: '<S251>/Difference Inputs1'
   *
   * Block description for '<S251>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = MCFR_ActualVelocity - rtb_Yk1_l;

  /* RelationalOperator: '<S259>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Switch2_on);

  /* Switch: '<S259>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S251>/delta fall limit' */
    rtb_deltafalllimit_iz = -10.0 * elapseTime;

    /* RelationalOperator: '<S259>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S259>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S259>/Switch' */
    rtb_Switch2_on = rtb_UkYk1;
  }

  /* End of Switch: '<S259>/Switch2' */

  /* Saturate: '<S204>/Saturation2' incorporates:
   *  Sum: '<S251>/Difference Inputs2'
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h = rtb_Switch2_on +
    rtb_Yk1_l;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h > 30.0) {
    rtb_Switch2_on = 30.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h < 0.0) {
    rtb_Switch2_on = 0.0;
  } else {
    rtb_Switch2_on = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h;
  }

  /* End of Saturate: '<S204>/Saturation2' */

  /* Gain: '<S204>/Gain1' */
  rtb_Switch2_on *= 0.1341030088495575;

  /* RelationalOperator: '<S245>/Compare' incorporates:
   *  Constant: '<S245>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Switch2_on >= 0.0);

  /* RelationalOperator: '<S246>/Compare' incorporates:
   *  Constant: '<S246>/Constant'
   */
  rtb_Compare_am = (rtb_Switch2_on < 40.0);

  /* Logic: '<S204>/OR2' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S204>/Timer2' incorporates:
   *  Constant: '<S204>/Constant4'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_o,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_l);

  /* UnitDelay: '<S252>/Delay Input2'
   *
   * Block description for '<S252>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n;

  /* SampleTimeMath: '<S252>/sample time'
   *
   * About '<S252>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S252>/delta rise limit' */
  rtb_UkYk1 = 10.0 * elapseTime;

  /* Sum: '<S252>/Difference Inputs1'
   *
   * Block description for '<S252>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_iz = MCFL_ActualVelocity - rtb_Yk1_l;

  /* RelationalOperator: '<S260>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_iz > rtb_UkYk1);

  /* Switch: '<S260>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S252>/delta fall limit' */
    rtb_UkYk1 = -10.0 * elapseTime;

    /* RelationalOperator: '<S260>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_iz < rtb_UkYk1);

    /* Switch: '<S260>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_iz = rtb_UkYk1;
    }

    /* End of Switch: '<S260>/Switch' */
    rtb_UkYk1 = rtb_deltafalllimit_iz;
  }

  /* End of Switch: '<S260>/Switch2' */

  /* Saturate: '<S204>/Saturation3' incorporates:
   *  Sum: '<S252>/Difference Inputs2'
   *  UnitDelay: '<S252>/Delay Input2'
   *
   * Block description for '<S252>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S252>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n = rtb_UkYk1 + rtb_Yk1_l;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n > 30.0) {
    rtb_UkYk1 = 30.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n < 0.0) {
    rtb_UkYk1 = 0.0;
  } else {
    rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n;
  }

  /* End of Saturate: '<S204>/Saturation3' */

  /* Gain: '<S204>/Gain2' */
  rtb_UkYk1 *= 0.1341030088495575;

  /* RelationalOperator: '<S247>/Compare' incorporates:
   *  Constant: '<S247>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_UkYk1 >= 0.0);

  /* RelationalOperator: '<S248>/Compare' incorporates:
   *  Constant: '<S248>/Constant'
   */
  rtb_Compare_am = (rtb_UkYk1 < 40.0);

  /* Logic: '<S204>/OR3' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S204>/Timer3' incorporates:
   *  Constant: '<S204>/Constant8'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_h,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer3);

  /* SignalConversion generated from: '<S199>/Out1' */
  WhlSpdFL = rtb_UkYk1;

  /* SignalConversion generated from: '<S199>/Out1' */
  WhlSpdFR = rtb_Switch2_on;

  /* SignalConversion generated from: '<S199>/Out1' */
  WhlSpdRR_mps = rtb_Gain4;

  /* SignalConversion generated from: '<S199>/Out1' */
  WhlSpdRL_mps = rtb_Gain5;

  /* Gain: '<S203>/Gain' incorporates:
   *  UnitDelay: '<S236>/Delay Input2'
   *
   * Block description for '<S236>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = 0.7 * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE;

  /* UnitDelay: '<S203>/Unit Delay' */
  rtb_Switch2_on = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE;

  /* Gain: '<S203>/Gain1' */
  rtb_Switch2_on *= 0.3;

  /* Sum: '<S203>/Add2' */
  rtb_UkYk1 += rtb_Switch2_on;

  /* Lookup_n-D: '<S203>/1-D Lookup Table' */
  rtb_Switch2_on = look1_binlx(rtb_UkYk1,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable_tableData, 23U);

  /* SignalConversion generated from: '<S199>/Out1' */
  FLWhlStrAng = rtb_Switch2_on;

  /* Lookup_n-D: '<S203>/1-D Lookup Table1' */
  rtb_Switch2_on = look1_binlx(rtb_UkYk1,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_tableData, 23U);

  /* SignalConversion generated from: '<S199>/Out1' */
  rtb_Yk1_l = rtb_Switch2_on;

  /* SignalConversion generated from: '<S199>/Out1' */
  rtb_deltafalllimit_iz = rtb_UkYk1;

  /* Sum: '<S201>/Add' */
  rtb_Add += Acc_POS;

  /* Product: '<S201>/Divide' incorporates:
   *  Constant: '<S201>/Constant'
   */
  rtb_Acc_POS = (real32_T)(rtb_Add / 2.0);

  /* UnitDelay: '<S226>/Delay Input2'
   *
   * Block description for '<S226>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l;

  /* SampleTimeMath: '<S226>/sample time'
   *
   * About '<S226>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S226>/delta rise limit' incorporates:
   *  Constant: '<S225>/Constant'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* UnitDelay: '<S225>/Unit Delay' */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_g;

  /* Gain: '<S225>/Gain1' */
  rtb_Gain4 *= 0.3;

  /* Gain: '<S202>/g_mpss' incorporates:
   *  UnitDelay: '<S225>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_g = 9.8 * IMU_Ay_Value;

  /* Gain: '<S225>/Gain' incorporates:
   *  UnitDelay: '<S225>/Unit Delay'
   */
  rtb_Gain5 = 0.7 * VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_g;

  /* Sum: '<S225>/Add2' */
  rtb_Add2 = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S226>/Difference Inputs1'
   *
   * Block description for '<S226>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add2 -= rtb_UkYk1;

  /* RelationalOperator: '<S232>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Add2 > rtb_Switch2_on);

  /* Switch: '<S232>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S226>/delta fall limit' incorporates:
     *  Constant: '<S225>/Constant1'
     */
    rtb_Switch2_on = -5000.0 * elapseTime;

    /* RelationalOperator: '<S232>/UpperRelop' */
    rtb_ignition = (rtb_Add2 < rtb_Switch2_on);

    /* Switch: '<S232>/Switch' */
    if (rtb_ignition) {
      rtb_Add2 = rtb_Switch2_on;
    }

    /* End of Switch: '<S232>/Switch' */
    rtb_Switch2_on = rtb_Add2;
  }

  /* End of Switch: '<S232>/Switch2' */

  /* Sum: '<S226>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S226>/Delay Input2'
   *
   * Block description for '<S226>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S226>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l = rtb_Switch2_on +
    rtb_UkYk1;

  /* RelationalOperator: '<S229>/LowerRelop1' incorporates:
   *  Constant: '<S225>/Constant6'
   *  UnitDelay: '<S226>/Delay Input2'
   *
   * Block description for '<S226>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l >
                       1.5);

  /* Switch: '<S229>/Switch2' incorporates:
   *  Constant: '<S225>/Constant6'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S229>/UpperRelop' incorporates:
     *  Constant: '<S225>/Constant7'
     *  UnitDelay: '<S226>/Delay Input2'
     *
     * Block description for '<S226>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_ignition = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l < -1.5);

    /* Switch: '<S229>/Switch' incorporates:
     *  Constant: '<S225>/Constant7'
     *  UnitDelay: '<S226>/Delay Input2'
     *
     * Block description for '<S226>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_ignition) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l;
    }

    /* End of Switch: '<S229>/Switch' */
  }

  /* End of Switch: '<S229>/Switch2' */

  /* SignalConversion generated from: '<S199>/Out1' */
  rtb_Add2 = rtb_UkYk1;

  /* UnitDelay: '<S227>/Delay Input2'
   *
   * Block description for '<S227>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m;

  /* SampleTimeMath: '<S227>/sample time'
   *
   * About '<S227>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S227>/delta rise limit' incorporates:
   *  Constant: '<S225>/Constant2'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S202>/g_mpss1' */
  rtb_g_mpss1 = 9.8 * IMU_Ax_Value;

  /* Gain: '<S225>/Gain2' */
  rtb_Gain4 = 0.7 * rtb_g_mpss1;

  /* UnitDelay: '<S225>/Unit Delay1' */
  rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE;

  /* Gain: '<S225>/Gain3' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S225>/Add1' */
  rtb_Add1 = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S227>/Difference Inputs1'
   *
   * Block description for '<S227>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add1 -= rtb_UkYk1;

  /* RelationalOperator: '<S233>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Add1 > rtb_Switch2_on);

  /* Switch: '<S233>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S227>/delta fall limit' incorporates:
     *  Constant: '<S225>/Constant4'
     */
    rtb_Switch2_on = -5000.0 * elapseTime;

    /* RelationalOperator: '<S233>/UpperRelop' */
    rtb_ignition = (rtb_Add1 < rtb_Switch2_on);

    /* Switch: '<S233>/Switch' */
    if (rtb_ignition) {
      rtb_Add1 = rtb_Switch2_on;
    }

    /* End of Switch: '<S233>/Switch' */
    rtb_Switch2_on = rtb_Add1;
  }

  /* End of Switch: '<S233>/Switch2' */

  /* Sum: '<S227>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S227>/Delay Input2'
   *
   * Block description for '<S227>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S227>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m = rtb_Switch2_on +
    rtb_UkYk1;

  /* RelationalOperator: '<S230>/LowerRelop1' incorporates:
   *  Constant: '<S225>/Constant8'
   *  UnitDelay: '<S227>/Delay Input2'
   *
   * Block description for '<S227>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m >
                       1.5);

  /* Switch: '<S230>/Switch2' incorporates:
   *  Constant: '<S225>/Constant8'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S230>/UpperRelop' incorporates:
     *  Constant: '<S225>/Constant9'
     *  UnitDelay: '<S227>/Delay Input2'
     *
     * Block description for '<S227>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_ignition = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m < -1.5);

    /* Switch: '<S230>/Switch' incorporates:
     *  Constant: '<S225>/Constant9'
     *  UnitDelay: '<S227>/Delay Input2'
     *
     * Block description for '<S227>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_ignition) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m;
    }

    /* End of Switch: '<S230>/Switch' */
  }

  /* End of Switch: '<S230>/Switch2' */

  /* SignalConversion generated from: '<S199>/Out1' */
  rtb_Add1 = rtb_UkYk1;

  /* SampleTimeMath: '<S237>/sample time'
   *
   * About '<S237>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S237>/delta rise limit' */
  rtb_UkYk1 = 1200.0 * elapseTime;

  /* UnitDelay: '<S237>/Delay Input2'
   *
   * Block description for '<S237>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_on = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j;

  /* Sum: '<S237>/Difference Inputs1'
   *
   * Block description for '<S237>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1_ix = StrWhlAngV - rtb_Switch2_on;

  /* RelationalOperator: '<S240>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1_ix > rtb_UkYk1);

  /* Switch: '<S240>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S237>/delta fall limit' */
    rtb_UkYk1 = -1200.0 * elapseTime;

    /* RelationalOperator: '<S240>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1_ix < rtb_UkYk1);

    /* Switch: '<S240>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1_ix = rtb_UkYk1;
    }

    /* End of Switch: '<S240>/Switch' */
    rtb_UkYk1 = rtb_UkYk1_ix;
  }

  /* End of Switch: '<S240>/Switch2' */

  /* Saturate: '<S203>/Saturation1' incorporates:
   *  Sum: '<S237>/Difference Inputs2'
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j = rtb_UkYk1 +
    rtb_Switch2_on;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j > 1200.0) {
    rtb_StrWhlAngV = 1200.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j < 0.0) {
    rtb_StrWhlAngV = 0.0;
  } else {
    rtb_StrWhlAngV = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j;
  }

  /* End of Saturate: '<S203>/Saturation1' */

  /* Gain: '<S203>/Gain2' */
  rtb_UkYk1 = 0.7 * rtb_StrWhlAngV;

  /* UnitDelay: '<S203>/Unit Delay1' */
  rtb_Switch2_on = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_a;

  /* Gain: '<S203>/Gain3' */
  rtb_Switch2_on *= 0.3;

  /* Sum: '<S203>/Add1' */
  rtb_UkYk1 += rtb_Switch2_on;

  /* SignalConversion generated from: '<S199>/Out1' */
  rtb_UkYk1_ix = rtb_UkYk1;

  /* UnitDelay: '<S228>/Delay Input2'
   *
   * Block description for '<S228>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k;

  /* SampleTimeMath: '<S228>/sample time'
   *
   * About '<S228>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S228>/delta rise limit' incorporates:
   *  Constant: '<S225>/Constant3'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S225>/Gain4' */
  rtb_Gain4 = 0.7 * IMU_Yaw_Value;

  /* UnitDelay: '<S225>/Unit Delay2' */
  rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE;

  /* Gain: '<S225>/Gain5' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S225>/Add3' */
  rtb_Gain5 += rtb_Gain4;

  /* Sum: '<S228>/Difference Inputs1'
   *
   * Block description for '<S228>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 -= rtb_UkYk1;

  /* RelationalOperator: '<S234>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Gain5 > rtb_Switch2_on);

  /* Switch: '<S234>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S228>/delta fall limit' incorporates:
     *  Constant: '<S225>/Constant5'
     */
    rtb_Switch2_on = -5000.0 * elapseTime;

    /* RelationalOperator: '<S234>/UpperRelop' */
    rtb_ignition = (rtb_Gain5 < rtb_Switch2_on);

    /* Switch: '<S234>/Switch' */
    if (rtb_ignition) {
      rtb_Gain5 = rtb_Switch2_on;
    }

    /* End of Switch: '<S234>/Switch' */
    rtb_Switch2_on = rtb_Gain5;
  }

  /* End of Switch: '<S234>/Switch2' */

  /* Saturate: '<S225>/Saturation2' incorporates:
   *  Sum: '<S228>/Difference Inputs2'
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k = rtb_Switch2_on +
    rtb_UkYk1;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k > 180.0) {
    rtb_UkYk1 = 180.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k < -180.0) {
    rtb_UkYk1 = -180.0;
  } else {
    rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k;
  }

  /* End of Saturate: '<S225>/Saturation2' */

  /* Update for UnitDelay: '<S201>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm = Acc2;

  /* Update for UnitDelay: '<S201>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_k = Acc1;

  /* Update for UnitDelay: '<S203>/Unit Delay' incorporates:
   *  UnitDelay: '<S236>/Delay Input2'
   *
   * Block description for '<S236>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE;

  /* Update for UnitDelay: '<S225>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE = rtb_g_mpss1;

  /* Update for UnitDelay: '<S203>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_a = rtb_StrWhlAngV;

  /* Update for UnitDelay: '<S225>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE = IMU_Yaw_Value;

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms' */

  /* S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.Subsystem_RESET_ELAPS_T) {
    FunctionCallSubsystem_ELAPS_T = 0U;
  } else {
    FunctionCallSubsystem_ELAPS_T =
      VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3 -
      VehCtrlMdel240926_2018b_amks_DW.Subsystem_PREV_T;
  }

  VehCtrlMdel240926_2018b_amks_DW.Subsystem_PREV_T =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
  VehCtrlMdel240926_2018b_amks_DW.Subsystem_RESET_ELAPS_T = false;

  /* Gain: '<S200>/Gain5' */
  rtb_Switch2_on = 10.0 * VehCtrlMdel240926_2018b_amksp_B.CANUnpack_o1;

  /* DataTypeConversion: '<S200>/Cast To Double' */
  rtb_CastToDouble = (real32_T)rtb_Switch2_on;

  /* MinMax: '<S324>/Min3' incorporates:
   *  Gain: '<S278>/Gain'
   *  UnitDelay: '<S278>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj *= 0.3F;

  /* UnitDelay: '<S282>/Delay Input2'
   *
   * Block description for '<S282>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* SampleTimeMath: '<S282>/sample time'
   *
   * About '<S282>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S282>/delta rise limit' */
  rtb_Add4_j = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S200>/Cast To Double1' */
  rtb_Add7 = (real32_T)WhlSpdFL;

  /* DataTypeConversion: '<S200>/Cast To Double6' */
  rtb_Add6_p = (real32_T)FLWhlStrAng;

  /* Gain: '<S264>/Gain2' */
  rtb_Add6_p *= 0.0174532924F;

  /* Trigonometry: '<S264>/Asin' */
  rtb_Add6_p = cosf(rtb_Add6_p);

  /* Product: '<S264>/Product1' */
  rtb_Add7 *= rtb_Add6_p;

  /* DataTypeConversion: '<S200>/Cast To Double5' */
  rtb_Add6_p = (real32_T)rtb_UkYk1;

  /* Gain: '<S264>/Gain4' */
  rtb_Add6_p *= 0.0174532924F;

  /* Product: '<S264>/Product3' */
  rtb_Switch2_mn = 0.6F * rtb_Add6_p;

  /* Sum: '<S264>/Add2' */
  rtb_Add = rtb_Add7 - rtb_Switch2_mn;

  /* UnitDelay: '<S271>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_j;

  /* Sum: '<S271>/Add4' */
  rtb_Add7 = rtb_Add - rtb_Add7;

  /* Product: '<S271>/Divide' incorporates:
   *  Constant: '<S271>/steptime'
   */
  rtb_Divide = rtb_Add7 / 0.01F;

  /* RelationalOperator: '<S283>/LowerRelop1' incorporates:
   *  Constant: '<S278>/Constant1'
   */
  rtb_LogicalOperator2 = (rtb_Divide > 100.0F);

  /* Switch: '<S283>/Switch2' incorporates:
   *  Constant: '<S278>/Constant1'
   */
  if (rtb_LogicalOperator2) {
    rtb_Divide = 100.0F;
  } else {
    /* RelationalOperator: '<S283>/UpperRelop' incorporates:
     *  Constant: '<S278>/Constant'
     */
    rtb_ignition = (rtb_Divide < -100.0F);

    /* Switch: '<S283>/Switch' incorporates:
     *  Constant: '<S278>/Constant'
     */
    if (rtb_ignition) {
      rtb_Divide = -100.0F;
    }

    /* End of Switch: '<S283>/Switch' */
  }

  /* End of Switch: '<S283>/Switch2' */

  /* Sum: '<S282>/Difference Inputs1'
   *
   * Block description for '<S282>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Divide -= rtb_Switch2_b0;

  /* RelationalOperator: '<S284>/LowerRelop1' */
  rtb_LogicalOperator2 = (rtb_Divide > rtb_Add4_j);

  /* Switch: '<S284>/Switch2' */
  if (!rtb_LogicalOperator2) {
    /* Product: '<S282>/delta fall limit' */
    rtb_deltafalllimit_aw = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S284>/UpperRelop' */
    rtb_ignition = (rtb_Divide < rtb_deltafalllimit_aw);

    /* Switch: '<S284>/Switch' */
    if (rtb_ignition) {
      rtb_Divide = rtb_deltafalllimit_aw;
    }

    /* End of Switch: '<S284>/Switch' */
    rtb_Add4_j = rtb_Divide;
  }

  /* End of Switch: '<S284>/Switch2' */

  /* Sum: '<S282>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S282>/Delay Input2'
   *
   * Block description for '<S282>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S282>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2 = rtb_Add4_j +
    rtb_Switch2_b0;

  /* Gain: '<S278>/Gain1' incorporates:
   *  UnitDelay: '<S282>/Delay Input2'
   *
   * Block description for '<S282>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add7 = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* MinMax: '<S324>/Min3' incorporates:
   *  Abs: '<S271>/Abs'
   *  Sum: '<S271>/Add'
   *  Sum: '<S278>/Add'
   *  UnitDelay: '<S278>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj += rtb_Add7;
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj -= rtb_CastToDouble;
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj = fabsf
    (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj);

  /* RelationalOperator: '<S274>/Compare' incorporates:
   *  Constant: '<S274>/Constant'
   *  UnitDelay: '<S278>/Unit Delay'
   */
  rtb_LogicalOperator2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj <=
    0.5F);

  /* UnitDelay: '<S279>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pjk;

  /* Gain: '<S279>/Gain' */
  rtb_Add7 *= 0.3F;

  /* UnitDelay: '<S285>/Delay Input2'
   *
   * Block description for '<S285>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* SampleTimeMath: '<S285>/sample time'
   *
   * About '<S285>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S285>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(100.0 * elapseTime);

  /* MinMax: '<S324>/Min3' incorporates:
   *  DataTypeConversion: '<S200>/Cast To Double2'
   *  UnitDelay: '<S278>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj = (real32_T)WhlSpdFR;

  /* DataTypeConversion: '<S200>/Cast To Double7' */
  rtb_Add10 = (real32_T)rtb_Yk1_l;

  /* Gain: '<S264>/Gain3' */
  rtb_Add10 *= 0.0174532924F;

  /* Trigonometry: '<S264>/Asin1' */
  rtb_Add10 = cosf(rtb_Add10);

  /* Product: '<S264>/Product2' incorporates:
   *  UnitDelay: '<S278>/Unit Delay'
   */
  rtb_Add10 *= VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj;

  /* Sum: '<S264>/Add3' */
  rtb_Divide = rtb_Switch2_mn + rtb_Add10;

  /* UnitDelay: '<S271>/Unit Delay1' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n;

  /* Sum: '<S271>/Add5' */
  rtb_Add10 = rtb_Divide - rtb_Add10;

  /* Product: '<S271>/Divide1' incorporates:
   *  Constant: '<S271>/steptime1'
   */
  rtb_deltafalllimit_aw = rtb_Add10 / 0.01F;

  /* RelationalOperator: '<S286>/LowerRelop1' incorporates:
   *  Constant: '<S279>/Constant1'
   */
  rtb_Compare = (rtb_deltafalllimit_aw > 100.0F);

  /* Switch: '<S286>/Switch2' incorporates:
   *  Constant: '<S279>/Constant1'
   */
  if (rtb_Compare) {
    rtb_deltafalllimit_aw = 100.0F;
  } else {
    /* RelationalOperator: '<S286>/UpperRelop' incorporates:
     *  Constant: '<S279>/Constant'
     */
    rtb_ignition = (rtb_deltafalllimit_aw < -100.0F);

    /* Switch: '<S286>/Switch' incorporates:
     *  Constant: '<S279>/Constant'
     */
    if (rtb_ignition) {
      rtb_deltafalllimit_aw = -100.0F;
    }

    /* End of Switch: '<S286>/Switch' */
  }

  /* End of Switch: '<S286>/Switch2' */

  /* Sum: '<S285>/Difference Inputs1'
   *
   * Block description for '<S285>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_aw -= rtb_Add4_j;

  /* RelationalOperator: '<S287>/LowerRelop1' */
  rtb_Compare = (rtb_deltafalllimit_aw > rtb_Switch2_b0);

  /* Switch: '<S287>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S285>/delta fall limit' */
    rtb_deltafalllimit_i = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S287>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_aw < rtb_deltafalllimit_i);

    /* Switch: '<S287>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_aw = rtb_deltafalllimit_i;
    }

    /* End of Switch: '<S287>/Switch' */
    rtb_Switch2_b0 = rtb_deltafalllimit_aw;
  }

  /* End of Switch: '<S287>/Switch2' */

  /* Sum: '<S285>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S285>/Delay Input2'
   *
   * Block description for '<S285>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S285>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e = rtb_Switch2_b0 +
    rtb_Add4_j;

  /* Gain: '<S279>/Gain1' incorporates:
   *  UnitDelay: '<S285>/Delay Input2'
   *
   * Block description for '<S285>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* Sum: '<S279>/Add' */
  rtb_Add7 += rtb_Add10;

  /* Sum: '<S271>/Add1' */
  rtb_Add7 -= rtb_CastToDouble;

  /* Abs: '<S271>/Abs1' */
  rtb_Add7 = fabsf(rtb_Add7);

  /* RelationalOperator: '<S275>/Compare' incorporates:
   *  Constant: '<S275>/Constant'
   */
  rtb_Compare = (rtb_Add7 <= 0.5F);

  /* UnitDelay: '<S280>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a;

  /* Gain: '<S280>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S288>/Delay Input2'
   *
   * Block description for '<S288>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk3;

  /* SampleTimeMath: '<S288>/sample time'
   *
   * About '<S288>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S288>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S200>/Cast To Double3' */
  rtb_Add4_j = (real32_T)WhlSpdRL_mps;

  /* Product: '<S264>/Product' */
  rtb_Add6_p *= 0.58F;

  /* Sum: '<S264>/Add' */
  rtb_deltafalllimit_aw = rtb_Add4_j - rtb_Add6_p;

  /* UnitDelay: '<S271>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_l;

  /* Sum: '<S271>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_aw - rtb_Add4_j;

  /* Product: '<S271>/Divide2' incorporates:
   *  Constant: '<S271>/steptime2'
   */
  rtb_deltafalllimit_i = rtb_Add4_j / 0.01F;

  /* RelationalOperator: '<S289>/LowerRelop1' incorporates:
   *  Constant: '<S280>/Constant1'
   */
  rtb_Compare_am = (rtb_deltafalllimit_i > 100.0F);

  /* Switch: '<S289>/Switch2' incorporates:
   *  Constant: '<S280>/Constant1'
   */
  if (rtb_Compare_am) {
    rtb_deltafalllimit_i = 100.0F;
  } else {
    /* RelationalOperator: '<S289>/UpperRelop' incorporates:
     *  Constant: '<S280>/Constant'
     */
    rtb_ignition = (rtb_deltafalllimit_i < -100.0F);

    /* Switch: '<S289>/Switch' incorporates:
     *  Constant: '<S280>/Constant'
     */
    if (rtb_ignition) {
      rtb_deltafalllimit_i = -100.0F;
    }

    /* End of Switch: '<S289>/Switch' */
  }

  /* End of Switch: '<S289>/Switch2' */

  /* Sum: '<S288>/Difference Inputs1'
   *
   * Block description for '<S288>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_i -= rtb_Switch2_mn;

  /* RelationalOperator: '<S290>/LowerRelop1' */
  rtb_Compare_am = (rtb_deltafalllimit_i > rtb_Add7);

  /* Switch: '<S290>/Switch2' */
  if (!rtb_Compare_am) {
    /* Product: '<S288>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S290>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_i < rtb_Add7);

    /* Switch: '<S290>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_i = rtb_Add7;
    }

    /* End of Switch: '<S290>/Switch' */
    rtb_Add7 = rtb_deltafalllimit_i;
  }

  /* End of Switch: '<S290>/Switch2' */

  /* Sum: '<S288>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S288>/Delay Input2'
   *
   * Block description for '<S288>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S288>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk3 = rtb_Add7 +
    rtb_Switch2_mn;

  /* Gain: '<S280>/Gain1' incorporates:
   *  UnitDelay: '<S288>/Delay Input2'
   *
   * Block description for '<S288>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk3;

  /* Sum: '<S280>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S271>/Add2' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S271>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S276>/Compare' incorporates:
   *  Constant: '<S276>/Constant'
   */
  rtb_Compare_am = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S281>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nc;

  /* Gain: '<S281>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S291>/Delay Input2'
   *
   * Block description for '<S291>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* SampleTimeMath: '<S291>/sample time'
   *
   * About '<S291>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S291>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S200>/Cast To Double4' */
  rtb_Add4_j = (real32_T)WhlSpdRR_mps;

  /* Sum: '<S264>/Add1' */
  rtb_deltafalllimit_i = rtb_Add6_p + rtb_Add4_j;

  /* UnitDelay: '<S271>/Unit Delay3' */
  rtb_Add6_p = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_h;

  /* Sum: '<S271>/Add7' */
  rtb_Add6_p = rtb_deltafalllimit_i - rtb_Add6_p;

  /* Product: '<S271>/Divide3' incorporates:
   *  Constant: '<S271>/steptime3'
   */
  rtb_Add6_p /= 0.01F;

  /* RelationalOperator: '<S292>/LowerRelop1' incorporates:
   *  Constant: '<S281>/Constant1'
   */
  rtb_ignition = (rtb_Add6_p > 100.0F);

  /* Switch: '<S292>/Switch2' incorporates:
   *  Constant: '<S281>/Constant1'
   */
  if (rtb_ignition) {
    rtb_Add6_p = 100.0F;
  } else {
    /* RelationalOperator: '<S292>/UpperRelop' incorporates:
     *  Constant: '<S281>/Constant'
     */
    rtb_ignition = (rtb_Add6_p < -100.0F);

    /* Switch: '<S292>/Switch' incorporates:
     *  Constant: '<S281>/Constant'
     */
    if (rtb_ignition) {
      rtb_Add6_p = -100.0F;
    }

    /* End of Switch: '<S292>/Switch' */
  }

  /* End of Switch: '<S292>/Switch2' */

  /* Sum: '<S291>/Difference Inputs1'
   *
   * Block description for '<S291>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add6_p -= rtb_Switch2_mn;

  /* RelationalOperator: '<S293>/LowerRelop1' */
  rtb_ignition = (rtb_Add6_p > rtb_Add7);

  /* Switch: '<S293>/Switch2' */
  if (!rtb_ignition) {
    /* Product: '<S291>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S293>/UpperRelop' */
    rtb_ignition = (rtb_Add6_p < rtb_Add7);

    /* Switch: '<S293>/Switch' */
    if (rtb_ignition) {
      rtb_Add6_p = rtb_Add7;
    }

    /* End of Switch: '<S293>/Switch' */
    rtb_Add7 = rtb_Add6_p;
  }

  /* End of Switch: '<S293>/Switch2' */

  /* Sum: '<S291>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S291>/Delay Input2'
   *
   * Block description for '<S291>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S291>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c = rtb_Add7 +
    rtb_Switch2_mn;

  /* Gain: '<S281>/Gain1' incorporates:
   *  UnitDelay: '<S291>/Delay Input2'
   *
   * Block description for '<S291>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* Sum: '<S281>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S271>/Add3' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S271>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S277>/Compare' incorporates:
   *  Constant: '<S277>/Constant'
   */
  rtb_ignition = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S298>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l;

  /* Gain: '<S298>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S302>/Delay Input2'
   *
   * Block description for '<S302>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* SampleTimeMath: '<S302>/sample time'
   *
   * About '<S302>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S302>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S272>/Unit Delay4' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_ik;

  /* UnitDelay: '<S272>/Unit Delay' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a0;

  /* Sum: '<S272>/Add4' */
  rtb_Add4_j = rtb_Add - rtb_Add4_j;

  /* Product: '<S272>/Divide' incorporates:
   *  Constant: '<S272>/steptime'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S272>/Add' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_ik = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S272>/Add8' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_ik - rtb_Add7;

  /* Product: '<S272>/Divide4' incorporates:
   *  Constant: '<S272>/steptime4'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S303>/LowerRelop1' incorporates:
   *  Constant: '<S298>/Constant1'
   */
  rtb_LowerRelop1_b = (rtb_Add7 > 100.0F);

  /* Switch: '<S303>/Switch2' incorporates:
   *  Constant: '<S298>/Constant1'
   */
  if (rtb_LowerRelop1_b) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S303>/UpperRelop' incorporates:
     *  Constant: '<S298>/Constant'
     */
    rtb_LowerRelop1_b = (rtb_Add7 < -100.0F);

    /* Switch: '<S303>/Switch' incorporates:
     *  Constant: '<S298>/Constant'
     */
    if (rtb_LowerRelop1_b) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S303>/Switch' */
  }

  /* End of Switch: '<S303>/Switch2' */

  /* Sum: '<S302>/Difference Inputs1'
   *
   * Block description for '<S302>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S304>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S304>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S302>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S304>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S304>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S304>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S304>/Switch2' */

  /* Sum: '<S302>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S302>/Delay Input2'
   *
   * Block description for '<S302>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S302>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i = rtb_Add6_p +
    rtb_Switch2_mn;

  /* Gain: '<S298>/Gain1' incorporates:
   *  UnitDelay: '<S302>/Delay Input2'
   *
   * Block description for '<S302>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* Sum: '<S298>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S272>/Abs' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S294>/Compare' incorporates:
   *  Constant: '<S294>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S299>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ap;

  /* Gain: '<S299>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S305>/Delay Input2'
   *
   * Block description for '<S305>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* SampleTimeMath: '<S305>/sample time'
   *
   * About '<S305>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S305>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S272>/Unit Delay5' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE;

  /* UnitDelay: '<S272>/Unit Delay1' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_am;

  /* Sum: '<S272>/Add5' */
  rtb_Add4_j = rtb_Divide - rtb_Add4_j;

  /* Product: '<S272>/Divide1' incorporates:
   *  Constant: '<S272>/steptime1'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S272>/Add1' incorporates:
   *  UnitDelay: '<S272>/Unit Delay5'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S272>/Add10' incorporates:
   *  UnitDelay: '<S272>/Unit Delay5'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE - rtb_Add7;

  /* Product: '<S272>/Divide5' incorporates:
   *  Constant: '<S272>/steptime5'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S306>/LowerRelop1' incorporates:
   *  Constant: '<S299>/Constant1'
   */
  rtb_AND2 = (rtb_Add7 > 100.0F);

  /* Switch: '<S306>/Switch2' incorporates:
   *  Constant: '<S299>/Constant1'
   */
  if (rtb_AND2) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S306>/UpperRelop' incorporates:
     *  Constant: '<S299>/Constant'
     */
    rtb_AND2 = (rtb_Add7 < -100.0F);

    /* Switch: '<S306>/Switch' incorporates:
     *  Constant: '<S299>/Constant'
     */
    if (rtb_AND2) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S306>/Switch' */
  }

  /* End of Switch: '<S306>/Switch2' */

  /* Sum: '<S305>/Difference Inputs1'
   *
   * Block description for '<S305>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S307>/LowerRelop1' */
  rtb_AND2 = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S307>/Switch2' */
  if (!rtb_AND2) {
    /* Product: '<S305>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S307>/UpperRelop' */
    rtb_AND2 = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S307>/Switch' */
    if (rtb_AND2) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S307>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S307>/Switch2' */

  /* Sum: '<S305>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S305>/Delay Input2'
   *
   * Block description for '<S305>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S305>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el = rtb_Add6_p +
    rtb_Switch2_mn;

  /* Gain: '<S299>/Gain1' incorporates:
   *  UnitDelay: '<S305>/Delay Input2'
   *
   * Block description for '<S305>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* Sum: '<S299>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S272>/Abs1' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S295>/Compare' incorporates:
   *  Constant: '<S295>/Constant'
   */
  rtb_AND2 = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S300>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o;

  /* Gain: '<S300>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S308>/Delay Input2'
   *
   * Block description for '<S308>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p;

  /* SampleTimeMath: '<S308>/sample time'
   *
   * About '<S308>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S308>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S272>/Unit Delay6' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE;

  /* UnitDelay: '<S272>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_c;

  /* Sum: '<S272>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_aw - rtb_Add4_j;

  /* Product: '<S272>/Divide2' incorporates:
   *  Constant: '<S272>/steptime2'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S272>/Add2' incorporates:
   *  UnitDelay: '<S272>/Unit Delay6'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S272>/Add12' incorporates:
   *  UnitDelay: '<S272>/Unit Delay6'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE - rtb_Add7;

  /* Product: '<S272>/Divide6' incorporates:
   *  Constant: '<S272>/steptime6'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S309>/LowerRelop1' incorporates:
   *  Constant: '<S300>/Constant1'
   */
  rtb_Compare_b = (rtb_Add7 > 100.0F);

  /* Switch: '<S309>/Switch2' incorporates:
   *  Constant: '<S300>/Constant1'
   */
  if (rtb_Compare_b) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S309>/UpperRelop' incorporates:
     *  Constant: '<S300>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S309>/Switch' incorporates:
     *  Constant: '<S300>/Constant'
     */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S309>/Switch' */
  }

  /* End of Switch: '<S309>/Switch2' */

  /* Sum: '<S308>/Difference Inputs1'
   *
   * Block description for '<S308>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S310>/LowerRelop1' */
  rtb_Compare_b = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S310>/Switch2' */
  if (!rtb_Compare_b) {
    /* Product: '<S308>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S310>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S310>/Switch' */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S310>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S310>/Switch2' */

  /* Sum: '<S308>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S308>/Delay Input2'
   *
   * Block description for '<S308>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S308>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p = rtb_Add6_p +
    rtb_Switch2_mn;

  /* Gain: '<S300>/Gain1' incorporates:
   *  UnitDelay: '<S308>/Delay Input2'
   *
   * Block description for '<S308>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p;

  /* Sum: '<S300>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S272>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S296>/Compare' incorporates:
   *  Constant: '<S296>/Constant'
   */
  rtb_Compare_b = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S301>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ah;

  /* Gain: '<S301>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S311>/Delay Input2'
   *
   * Block description for '<S311>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* SampleTimeMath: '<S311>/sample time'
   *
   * About '<S311>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S311>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S272>/Unit Delay7' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE;

  /* UnitDelay: '<S272>/Unit Delay3' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_d;

  /* Sum: '<S272>/Add7' */
  rtb_Add4_j = rtb_deltafalllimit_i - rtb_Add4_j;

  /* Product: '<S272>/Divide3' incorporates:
   *  Constant: '<S272>/steptime3'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S272>/Add3' incorporates:
   *  UnitDelay: '<S272>/Unit Delay7'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S272>/Add14' incorporates:
   *  UnitDelay: '<S272>/Unit Delay7'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE - rtb_Add7;

  /* Product: '<S272>/Divide7' incorporates:
   *  Constant: '<S272>/steptime7'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S312>/LowerRelop1' incorporates:
   *  Constant: '<S301>/Constant1'
   */
  rtb_UpperRelop_ir = (rtb_Add7 > 100.0F);

  /* Switch: '<S312>/Switch2' incorporates:
   *  Constant: '<S301>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S312>/UpperRelop' incorporates:
     *  Constant: '<S301>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S312>/Switch' incorporates:
     *  Constant: '<S301>/Constant'
     */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S312>/Switch' */
  }

  /* End of Switch: '<S312>/Switch2' */

  /* Sum: '<S311>/Difference Inputs1'
   *
   * Block description for '<S311>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S313>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S313>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S311>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S313>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S313>/Switch' */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S313>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S313>/Switch2' */

  /* Sum: '<S311>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S311>/Delay Input2'
   *
   * Block description for '<S311>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S311>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt = rtb_Add6_p +
    rtb_Switch2_mn;

  /* Gain: '<S301>/Gain1' incorporates:
   *  UnitDelay: '<S311>/Delay Input2'
   *
   * Block description for '<S311>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* Sum: '<S301>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S272>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S297>/Compare' incorporates:
   *  Constant: '<S297>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.8F);

  /* Logic: '<S262>/Logical Operator' */
  rtb_LogicalOperator_idx_0 = (rtb_LogicalOperator2 || rtb_LowerRelop1_b);
  rtb_Compare = (rtb_Compare || rtb_AND2);
  rtb_Compare_am = (rtb_Compare_am || rtb_Compare_b);
  rtb_LogicalOperator2 = (rtb_ignition || rtb_UpperRelop_ir);

  /* UnitDelay: '<S200>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_lh;

  /* Sum: '<S273>/Add' */
  rtb_Switch2_mn = rtb_Add - rtb_Add10;

  /* Abs: '<S273>/Abs' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S314>/Compare' incorporates:
   *  Constant: '<S314>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S273>/AND3' */
  rtb_Compare_b = (rtb_UpperRelop_ir && (VehCtrlMdel240926_2018b_amksp_B.Exit_h
    != 0.0));

  /* Sum: '<S273>/Add1' */
  rtb_Switch2_mn = rtb_Divide - rtb_Add10;

  /* Abs: '<S273>/Abs1' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S315>/Compare' incorporates:
   *  Constant: '<S315>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S273>/AND2' */
  rtb_AND2 = (rtb_UpperRelop_ir && (VehCtrlMdel240926_2018b_amksp_B.Exit_o !=
    0.0));

  /* Sum: '<S273>/Add2' */
  rtb_Switch2_mn = rtb_deltafalllimit_aw - rtb_Add10;

  /* Abs: '<S273>/Abs2' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S316>/Compare' incorporates:
   *  Constant: '<S316>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S273>/AND' */
  rtb_LowerRelop1_b = (rtb_UpperRelop_ir &&
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_le != 0.0));

  /* Sum: '<S273>/Add3' */
  rtb_Add10 = rtb_deltafalllimit_i - rtb_Add10;

  /* Abs: '<S273>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S317>/Compare' incorporates:
   *  Constant: '<S317>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 2.0F);

  /* Logic: '<S273>/AND1' */
  rtb_ignition = (rtb_UpperRelop_ir && (VehCtrlMdel240926_2018b_amksp_B.Exit_i
    != 0.0));

  /* Logic: '<S262>/Logical Operator1' */
  rtb_UpperRelop_ir = (rtb_Compare_b && rtb_LogicalOperator_idx_0);
  rtb_Compare = (rtb_AND2 && rtb_Compare);
  rtb_Compare_am = (rtb_LowerRelop1_b && rtb_Compare_am);
  rtb_ignition = (rtb_ignition && rtb_LogicalOperator2);

  /* Chart: '<S262>/Timer' incorporates:
   *  Constant: '<S262>/Constant1'
   */
  VehCtrlMdel240926_20_Timer1(rtb_UpperRelop_ir, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_c,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_o);

  /* Chart: '<S262>/Timer1' incorporates:
   *  Constant: '<S262>/Constant2'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_lh4,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_m);

  /* Chart: '<S262>/Timer2' incorporates:
   *  Constant: '<S262>/Constant3'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_lh,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_g);

  /* Chart: '<S262>/Timer3' incorporates:
   *  Constant: '<S262>/Constant4'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_a,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer3_i);

  /* Logic: '<S261>/Logical Operator' */
  rtb_UpperRelop_ir = ((VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0));

  /* Logic: '<S261>/Logical Operator1' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* UnitDelay: '<S261>/Unit Delay4' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_m;

  /* Sum: '<S261>/Add1' */
  rtb_Add10 = rtb_Acc_POS - rtb_Add10;

  /* RelationalOperator: '<S265>/Compare' incorporates:
   *  Constant: '<S265>/Constant'
   */
  rtb_Compare_b = (rtb_Add10 > 0.1F);

  /* Logic: '<S261>/Logical Operator2' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir || rtb_Compare_b);

  /* Logic: '<S261>/AND' */
  rtb_ignition = ((VehCtrlMdel240926_2018b_amksp_B.CANUnpack_o1 != 0.0) &&
                  rtb_UpperRelop_ir);

  /* UnitDelay: '<S261>/Unit Delay3' */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_f;

  /* Logic: '<S261>/Logical Operator3' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S261>/Switch3' incorporates:
   *  UnitDelay: '<S261>/Unit Delay1'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S261>/Switch4' */
    if (!rtb_ignition) {
      /* Saturate: '<S29>/Saturation' incorporates:
       *  Constant: '<S261>/InitZORE'
       */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k = 0.0F;
    }

    /* End of Switch: '<S261>/Switch4' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_o =
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Switch: '<S261>/Switch3' */

  /* UnitDelay: '<S263>/Unit Delay3' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_p;

  /* Sum: '<S263>/Add5' incorporates:
   *  UnitDelay: '<S263>/Unit Delay1'
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d - rtb_Add10;

  /* Product: '<S263>/Divide3' incorporates:
   *  Constant: '<S263>/steptime3'
   */
  rtb_Add10 /= 0.01F;

  /* UnitDelay: '<S263>/Unit Delay2' */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_f;

  /* Sum: '<S263>/Add9' */
  rtb_Switch2_mn -= rtb_Add10;

  /* UnitDelay: '<S263>/Unit Delay4' */
  rtb_Add6_p = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn;

  /* Sum: '<S263>/Add6' incorporates:
   *  Constant: '<S263>/steptime4'
   */
  rtb_Add6_p += 0.1F;

  /* Sum: '<S263>/Add8' incorporates:
   *  Constant: '<S263>/steptime6'
   */
  rtb_Add7 = rtb_Add6_p + 2.0F;

  /* Product: '<S263>/Divide5' */
  rtb_Add7 = 1.0F / rtb_Add7 * rtb_Add6_p;

  /* Logic: '<S263>/Logical Operator' */
  rtb_LogicalOperator2 = ((VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) ||
    (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) ||
    (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) ||
    (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0));

  /* Switch: '<S263>/Switch13' incorporates:
   *  Constant: '<S263>/Constant10'
   */
  if (rtb_LogicalOperator2) {
    rtb_Add4_j = rtb_Add7;
  } else {
    rtb_Add4_j = 1.0F;
  }

  /* End of Switch: '<S263>/Switch13' */

  /* Product: '<S263>/Divide6' */
  rtb_Switch2_mn *= rtb_Add4_j;

  /* Sum: '<S263>/Add10' */
  rtb_Ax = rtb_Switch2_mn + rtb_Add10;

  /* Switch: '<S261>/Switch1' */
  if (rtb_ignition) {
    /* Saturate: '<S261>/Saturation1' */
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d > 200.0F) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = 200.0F;
    } else {
      if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d < -10.0F) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = -10.0F;
      }
    }

    /* Product: '<S261>/Product' incorporates:
     *  Constant: '<S261>/steptime1'
     */
    rtb_Switch2_mn = rtb_Ax * 0.01F;

    /* Saturate: '<S261>/Saturation1' incorporates:
     *  Sum: '<S261>/Add'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d += rtb_Switch2_mn;
  } else {
    /* Saturate: '<S261>/Saturation1' incorporates:
     *  Constant: '<S261>/Constant'
     *  UnitDelay: '<S261>/Unit Delay'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = 0.0F;
  }

  /* End of Switch: '<S261>/Switch1' */

  /* Saturate: '<S261>/Saturation' */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d > 200.0F) {
    rtb_Add10 = 200.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d < -10.0F) {
    rtb_Add10 = -10.0F;
  } else {
    rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d;
  }

  /* End of Saturate: '<S261>/Saturation' */

  /* Sum: '<S261>/Add3' incorporates:
   *  UnitDelay: '<S261>/Unit Delay1'
   */
  rtb_VxIMU_est = rtb_Add10 +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_o;

  /* MinMax: '<S262>/Min1' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Add, rtb_Divide);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_aw);
  rtb_Add10 = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_i);

  /* Sum: '<S261>/Add2' */
  rtb_Add10 -= rtb_VxIMU_est;

  /* RelationalOperator: '<S266>/Compare' incorporates:
   *  Constant: '<S266>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.0F);

  /* Switch: '<S261>/Switch6' incorporates:
   *  Constant: '<S261>/Reset'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S261>/Add10' incorporates:
     *  Constant: '<S261>/Steptime'
     */
    rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i + 0.01F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S261>/Switch6' */

  /* MinMax: '<S261>/Min' incorporates:
   *  Constant: '<S261>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i = fminf(rtb_Add10, 0.1F);

  /* RelationalOperator: '<S261>/Relational Operator9' incorporates:
   *  Constant: '<S261>/ResetDelay'
   *  UnitDelay: '<S261>/Unit Delay2'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i >= 0.1F);

  /* RelationalOperator: '<S319>/Compare' incorporates:
   *  Constant: '<S319>/Constant'
   */
  rtb_Compare_am = (rtb_Ax < -0.5F);

  /* Chart: '<S263>/Timer2' incorporates:
   *  Constant: '<S263>/Constant15'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_j);

  /* UnitDelay: '<S323>/Delay Input2'
   *
   * Block description for '<S323>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;

  /* SampleTimeMath: '<S323>/sample time'
   *
   * About '<S323>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S323>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(10.0 * elapseTime);

  /* Sum: '<S324>/Add3' */
  rtb_Add4_j = ((rtb_deltafalllimit_i + rtb_deltafalllimit_aw) + rtb_Divide) +
    rtb_Add;

  /* MinMax: '<S324>/Min4' */
  rtb_Switch2_b0 = fminf(rtb_Add, rtb_Divide);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_aw);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_i);

  /* MinMax: '<S324>/Min3' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Add, rtb_Divide);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_aw);
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj = fmaxf
    (rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_i);

  /* Sum: '<S324>/Add4' incorporates:
   *  UnitDelay: '<S278>/Unit Delay'
   */
  rtb_Add4_j = (rtb_Add4_j - rtb_Switch2_b0) -
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj;

  /* Gain: '<S324>/Gain1' */
  rtb_Add4_j *= 0.5F;

  /* Sum: '<S323>/Difference Inputs1'
   *
   * Block description for '<S323>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add4_j -= rtb_Add10;

  /* RelationalOperator: '<S332>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add4_j > rtb_Switch2_mn);

  /* Switch: '<S332>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S323>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-10.0 * elapseTime);

    /* RelationalOperator: '<S332>/UpperRelop' */
    rtb_Compare_am = (rtb_Add4_j < rtb_Switch2_mn);

    /* Switch: '<S332>/Switch' */
    if (rtb_Compare_am) {
      rtb_Add4_j = rtb_Switch2_mn;
    }

    /* End of Switch: '<S332>/Switch' */
    rtb_Switch2_mn = rtb_Add4_j;
  }

  /* End of Switch: '<S332>/Switch2' */

  /* Sum: '<S323>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S323>/Delay Input2'
   *
   * Block description for '<S323>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S323>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g = rtb_Switch2_mn +
    rtb_Add10;

  /* RelationalOperator: '<S318>/Compare' incorporates:
   *  Constant: '<S318>/Constant'
   */
  rtb_Compare_am = (rtb_Ax > 0.5F);

  /* Chart: '<S263>/Timer1' incorporates:
   *  Constant: '<S263>/Constant14'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_l,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_p);

  /* Logic: '<S263>/Logical Operator2' */
  rtb_UpperRelop_ir = !(VehCtrlMdel240926_2018b_amksp_B.Exit_l != 0.0);

  /* Switch: '<S263>/Switch6' incorporates:
   *  Switch: '<S263>/Switch4'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S263>/Switch5' incorporates:
     *  UnitDelay: '<S323>/Delay Input2'
     *
     * Block description for '<S323>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit != 0.0) {
      /* Switch: '<S263>/Switch11' incorporates:
       *  Constant: '<S263>/Constant7'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0) {
        rtb_Switch2_mn = rtb_deltafalllimit_i;
      } else {
        rtb_Switch2_mn = 0.0F;
      }

      /* End of Switch: '<S263>/Switch11' */

      /* Switch: '<S263>/Switch10' incorporates:
       *  Constant: '<S263>/Constant6'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) {
        rtb_Add10 = rtb_deltafalllimit_aw;
      } else {
        rtb_Add10 = 0.0F;
      }

      /* End of Switch: '<S263>/Switch10' */

      /* Switch: '<S263>/Switch9' incorporates:
       *  Constant: '<S263>/Constant5'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) {
        rtb_Add4_j = rtb_Divide;
      } else {
        rtb_Add4_j = 0.0F;
      }

      /* End of Switch: '<S263>/Switch9' */

      /* Switch: '<S263>/Switch8' incorporates:
       *  Constant: '<S263>/Constant4'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) {
        rtb_Switch2_b0 = rtb_Add;
      } else {
        rtb_Switch2_b0 = 0.0F;
      }

      /* End of Switch: '<S263>/Switch8' */

      /* MinMax: '<S263>/Min1' */
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Switch2_b0, rtb_Add4_j);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Add10);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Switch2_mn);
    } else {
      rtb_MaxWhlSpd_mps_n = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;
    }

    /* End of Switch: '<S263>/Switch5' */
  } else {
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0) {
      /* Switch: '<S263>/Switch4' */
      rtb_Switch2_mn = rtb_deltafalllimit_i;
    } else {
      /* Switch: '<S263>/Switch4' incorporates:
       *  Constant: '<S263>/Constant3'
       */
      rtb_Switch2_mn = 9999.0F;
    }

    /* Switch: '<S263>/Switch3' incorporates:
     *  Constant: '<S263>/Constant2'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) {
      rtb_Add10 = rtb_deltafalllimit_aw;
    } else {
      rtb_Add10 = 9999.0F;
    }

    /* End of Switch: '<S263>/Switch3' */

    /* Switch: '<S263>/Switch2' incorporates:
     *  Constant: '<S263>/Constant1'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) {
      rtb_Add4_j = rtb_Divide;
    } else {
      rtb_Add4_j = 9999.0F;
    }

    /* End of Switch: '<S263>/Switch2' */

    /* Switch: '<S263>/Switch1' incorporates:
     *  Constant: '<S263>/Constant'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) {
      rtb_Switch2_b0 = rtb_Add;
    } else {
      rtb_Switch2_b0 = 9999.0F;
    }

    /* End of Switch: '<S263>/Switch1' */

    /* MinMax: '<S263>/Min2' */
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add4_j);
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add10);
    rtb_MaxWhlSpd_mps_n = fminf(rtb_Switch2_b0, rtb_Switch2_mn);
  }

  /* End of Switch: '<S263>/Switch6' */

  /* Logic: '<S263>/NOT3' */
  rtb_UpperRelop_ir = !rtb_LogicalOperator2;

  /* Logic: '<S263>/Logical Operator3' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir && rtb_Compare);

  /* Logic: '<S263>/NOT4' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S263>/Switch7' incorporates:
   *  UnitDelay: '<S323>/Delay Input2'
   *
   * Block description for '<S323>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (rtb_UpperRelop_ir) {
    /* Logic: '<S263>/Logical Operator1' */
    rtb_Compare = (rtb_Compare || rtb_LogicalOperator2);

    /* Switch: '<S263>/Switch' */
    if (rtb_Compare) {
      rtb_VxIMU_est = rtb_MaxWhlSpd_mps_n;
    }

    /* End of Switch: '<S263>/Switch' */
  } else {
    rtb_VxIMU_est = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;
  }

  /* End of Switch: '<S263>/Switch7' */

  /* UnitDelay: '<S321>/Delay Input2'
   *
   * Block description for '<S321>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_af;

  /* Sum: '<S321>/Difference Inputs1'
   *
   * Block description for '<S321>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_VxIMU_est -= rtb_Add10;

  /* Switch: '<S263>/Switch12' incorporates:
   *  Constant: '<S263>/Constant8'
   *  Constant: '<S263>/Constant9'
   */
  if (rtb_LogicalOperator2) {
    rtb_Switch2_mn = 0.1F;
  } else {
    rtb_Switch2_mn = 0.05F;
  }

  /* End of Switch: '<S263>/Switch12' */

  /* Sum: '<S263>/Add4' */
  rtb_Add4_j = rtb_Ax + rtb_Switch2_mn;

  /* SampleTimeMath: '<S321>/sample time'
   *
   * About '<S321>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S321>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S330>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_VxIMU_est > rtb_Switch2_b0);

  /* Sum: '<S263>/Add3' */
  rtb_Ax -= rtb_Switch2_mn;

  /* Switch: '<S330>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S321>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S330>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_VxIMU_est < rtb_Switch2_mn);

    /* Switch: '<S330>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_VxIMU_est = rtb_Switch2_mn;
    }

    /* End of Switch: '<S330>/Switch' */
    rtb_Switch2_b0 = rtb_VxIMU_est;
  }

  /* End of Switch: '<S330>/Switch2' */

  /* Sum: '<S321>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S321>/Delay Input2'
   *
   * Block description for '<S321>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S321>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_af = rtb_Switch2_b0 +
    rtb_Add10;

  /* RelationalOperator: '<S328>/LowerRelop1' incorporates:
   *  Constant: '<S320>/Constant1'
   *  UnitDelay: '<S321>/Delay Input2'
   *
   * Block description for '<S321>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_af >
                       100.0F);

  /* Switch: '<S328>/Switch2' incorporates:
   *  Constant: '<S320>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_mn = 100.0F;
  } else {
    /* RelationalOperator: '<S328>/UpperRelop' incorporates:
     *  Constant: '<S320>/Constant'
     *  UnitDelay: '<S321>/Delay Input2'
     *
     * Block description for '<S321>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_LogicalOperator2 =
      (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_af < 0.0F);

    /* Switch: '<S328>/Switch' incorporates:
     *  Constant: '<S320>/Constant'
     *  UnitDelay: '<S321>/Delay Input2'
     *
     * Block description for '<S321>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_LogicalOperator2) {
      rtb_Switch2_mn = 0.0F;
    } else {
      rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_af;
    }

    /* End of Switch: '<S328>/Switch' */
  }

  /* End of Switch: '<S328>/Switch2' */

  /* UnitDelay: '<S327>/Delay Input2'
   *
   * Block description for '<S327>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* Sum: '<S327>/Difference Inputs1'
   *
   * Block description for '<S327>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_Switch2_mn - rtb_Add10;

  /* SampleTimeMath: '<S327>/sample time'
   *
   * About '<S327>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S327>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(15.0 * elapseTime);

  /* RelationalOperator: '<S329>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S329>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S327>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-15.0 * elapseTime);

    /* RelationalOperator: '<S329>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S329>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S329>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S329>/Switch2' */

  /* Sum: '<S327>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S327>/Delay Input2'
   *
   * Block description for '<S327>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S327>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f = rtb_Switch2_mn +
    rtb_Add10;

  /* UnitDelay: '<S320>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ncs;

  /* Gain: '<S320>/Gain' */
  rtb_Add10 *= 0.0F;

  /* Saturate: '<S29>/Saturation' incorporates:
   *  Sum: '<S320>/Add'
   *  UnitDelay: '<S200>/Unit Delay1'
   *  UnitDelay: '<S327>/Delay Input2'
   *
   * Block description for '<S327>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k = rtb_Add10 +
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* SampleTimeMath: '<S322>/sample time'
   *
   * About '<S322>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* UnitDelay: '<S322>/Delay Input2'
   *
   * Block description for '<S322>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu;

  /* Sum: '<S322>/Difference Inputs1'
   *
   * Block description for '<S322>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_MaxWhlSpd_mps_n - rtb_Add10;

  /* Product: '<S322>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S331>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S331>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S322>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S331>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S331>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S331>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S331>/Switch2' */

  /* Sum: '<S322>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S322>/Delay Input2'
   *
   * Block description for '<S322>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S322>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu = rtb_Switch2_mn +
    rtb_Add10;

  /* Sum: '<S263>/Add7' incorporates:
   *  Constant: '<S263>/steptime5'
   */
  rtb_Add7 = 1.0F - rtb_Add7;

  /* Product: '<S263>/Divide4' incorporates:
   *  UnitDelay: '<S263>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn = rtb_Add7 * rtb_Add6_p;

  /* Update for MinMax: '<S324>/Min3' incorporates:
   *  UnitDelay: '<S278>/Unit Delay'
   *  UnitDelay: '<S282>/Delay Input2'
   *
   * Block description for '<S282>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* Update for UnitDelay: '<S271>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_j = rtb_Add;

  /* Update for UnitDelay: '<S279>/Unit Delay' incorporates:
   *  UnitDelay: '<S285>/Delay Input2'
   *
   * Block description for '<S285>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pjk =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* Update for UnitDelay: '<S271>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n = rtb_Divide;

  /* Update for UnitDelay: '<S280>/Unit Delay' incorporates:
   *  UnitDelay: '<S288>/Delay Input2'
   *
   * Block description for '<S288>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk3;

  /* Update for UnitDelay: '<S271>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_l = rtb_deltafalllimit_aw;

  /* Update for UnitDelay: '<S281>/Unit Delay' incorporates:
   *  UnitDelay: '<S291>/Delay Input2'
   *
   * Block description for '<S291>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nc =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* Update for UnitDelay: '<S271>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_h = rtb_deltafalllimit_i;

  /* Update for UnitDelay: '<S298>/Unit Delay' incorporates:
   *  UnitDelay: '<S302>/Delay Input2'
   *
   * Block description for '<S302>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* Update for UnitDelay: '<S272>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a0 = rtb_Add;

  /* Update for UnitDelay: '<S299>/Unit Delay' incorporates:
   *  UnitDelay: '<S305>/Delay Input2'
   *
   * Block description for '<S305>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ap =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* Update for UnitDelay: '<S272>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_am = rtb_Divide;

  /* Update for UnitDelay: '<S300>/Unit Delay' incorporates:
   *  UnitDelay: '<S308>/Delay Input2'
   *
   * Block description for '<S308>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p;

  /* Update for UnitDelay: '<S272>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_c = rtb_deltafalllimit_aw;

  /* Update for UnitDelay: '<S301>/Unit Delay' incorporates:
   *  UnitDelay: '<S311>/Delay Input2'
   *
   * Block description for '<S311>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ah =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* Update for UnitDelay: '<S272>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_d = rtb_deltafalllimit_i;

  /* Update for UnitDelay: '<S200>/Unit Delay' incorporates:
   *  UnitDelay: '<S200>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_lh =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k;

  /* Update for UnitDelay: '<S261>/Unit Delay4' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_m = rtb_Acc_POS;

  /* Update for UnitDelay: '<S261>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_f = rtb_ignition;

  /* Update for UnitDelay: '<S263>/Unit Delay3' incorporates:
   *  UnitDelay: '<S263>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_p =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d;

  /* Update for UnitDelay: '<S263>/Unit Delay1' incorporates:
   *  UnitDelay: '<S322>/Delay Input2'
   *
   * Block description for '<S322>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu;

  /* Update for UnitDelay: '<S263>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_f = rtb_CastToDouble;

  /* Update for UnitDelay: '<S320>/Unit Delay' incorporates:
   *  UnitDelay: '<S327>/Delay Input2'
   *
   * Block description for '<S327>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ncs =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms1' */

  /* S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* DataTypeConversion: '<S105>/Cast To Boolean' */
  KeyPressed = (ignition != 0.0);

  /* RelationalOperator: '<S107>/Compare' incorporates:
   *  Constant: '<S107>/Constant'
   */
  Brk = (Brk_F >= 450);

  /* RelationalOperator: '<S108>/Compare' incorporates:
   *  Constant: '<S108>/Constant'
   */
  ACC_Release = (rtb_Acc_POS <= 50.0F);

  /* Chart: '<S105>/Chart2' */
  FunctionCallSubsystem_ELAPS_T =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3 -
    VehCtrlMdel240926_2018b_amks_DW.previousTicks;
  VehCtrlMdel240926_2018b_amks_DW.previousTicks =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
  if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 +
      FunctionCallSubsystem_ELAPS_T <= 255U) {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = (uint8_T)
      (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 +
       FunctionCallSubsystem_ELAPS_T);
  } else {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = MAX_uint8_T;
  }

  if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 +
      FunctionCallSubsystem_ELAPS_T <= 1023U) {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = (uint16_T)
      (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 +
       FunctionCallSubsystem_ELAPS_T);
  } else {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 1023U;
  }

  VehCtrlMdel240926_2018b_amks_DW.sfEvent = -1;
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c1_VehCtrlMdel240926_
      == 0U) {
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c1_VehCtrlMdel240926_ =
      1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_VehStat = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 2U;
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
    VehCtrlMdel240926_2018b_amksp_B.errorReset = 1.0;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_BeeperStat = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_BeeperStat = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_AMKDCon = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCon = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MCDCEnable = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCDCEnable = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MC_TorqueCUT = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_AMKDCready = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 9U;
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_Output = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_AMKCANenable = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MC_InverterOn = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_InverterOn = 1U;
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_VehStat != 0U) {
      VehCtrlMdel240926_2018b_VehStat(&MCFL_bQuitInverterOn, &MCFL_bSystemReady,
        &controller_ready, &MCFR_bQuitInverterOn, &MCFR_bSystemReady);
    }

    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_BeeperStat != 0U)
    {
      switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_BeeperStat) {
       case VehCtrlMdel240926_2018b__IN_OFF:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel240926_event_EbeepON) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_BeeperStat = 2U;
        }
        break;

       case VehCtrlMdel240926_2018b_a_IN_ON:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel24092_event_EbeepOFF) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_BeeperStat = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_AMKDCon != 0U) {
      switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCon) {
       case VehCtrlMdel240926_2018b__IN_OFF:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel240926_event_AMKDCON) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCon = 2U;
        }
        break;

       case VehCtrlMdel240926_2018b_a_IN_ON:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel24092_event_AMKDCOFF) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCon = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MCDCEnable != 0U)
    {
      switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCDCEnable) {
       case VehCtrlMdel240926_2018b__IN_OFF:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel2_event_MCDCEnableON) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCDCEnable = 2U;
        }
        break;

       case VehCtrlMdel240926_2018b_a_IN_ON:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel_event_MCDCEnableOFF) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCDCEnable = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MC_TorqueCUT != 0U)
    {
      switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT) {
       case VehCtrlMdel240926_2018b__IN_OFF:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel24092_event_TorqueON) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT = 2U;
        }
        break;

       case VehCtrlMdel240926_2018b_a_IN_ON:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel2409_event_TorqueOFF) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_AMKDCready != 0U)
    {
      VehCtrlMdel240926_20_AMKDCready(&MCFL_bDCOn, &MCFL_bQuitInverterOn,
        &MCFL_bSystemReady, &MCFR_bDCOn, &MCFR_bQuitInverterOn,
        &MCFR_bSystemReady);
    }

    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_Output != 0U) {
      VehCtrlMdel240926_2018b_amksp_B.VehReady =
        (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat ==
         VehCtrlMdel240926_2018_IN_Ready);
      beeper_state = (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_BeeperStat ==
                      VehCtrlMdel240926_2018b_a_IN_ON);
      MCFL_DCOn_setpoints =
        (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCon ==
         VehCtrlMdel240926_2018b_a_IN_ON);
      VehCtrlMdel240926_2018b_amksp_B.MCFL_DCEnable =
        (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCDCEnable ==
         VehCtrlMdel240926_2018b_a_IN_ON);
      MCFR_DCEnable = (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCDCEnable
                       == VehCtrlMdel240926_2018b_a_IN_ON);
      VehCtrlMdel240926_2018b_amksp_B.MCFL_InverterOn =
        (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_InverterOn ==
         VehCtrlMdel240926_2018b_a_IN_ON);
      MCFR_InverterOn =
        (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_InverterOn ==
         VehCtrlMdel240926_2018b_a_IN_ON);
      VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueOn =
        (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT ==
         VehCtrlMdel240926_2018b_a_IN_ON);
      VehCtrlMdel240926_2018b_amksp_B.MCFR_TorqueOn =
        (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT ==
         VehCtrlMdel240926_2018b_a_IN_ON);
    }

    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_AMKCANenable != 0U)
    {
      switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKCANenable) {
       case VehCtrlMdel240926_2018b__IN_OFF:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel24092_event_AMKCANON) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 2U;
        }
        break;

       case VehCtrlMdel240926_2018b_a_IN_ON:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel2409_event_AMKCANOFF) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MC_InverterOn !=
        0U) {
      switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_InverterOn) {
       case VehCtrlMdel240926_2018b__IN_OFF:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel240_event_InverterON) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_InverterOn = 2U;
        }
        break;

       case VehCtrlMdel240926_2018b_a_IN_ON:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel24_event_InverterOFF) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MC_InverterOn = 1U;
        }
        break;
      }
    }
  }

  /* End of Chart: '<S105>/Chart2' */
  /* End of Outputs for S-Function (fcncallgen): '<S2>/10ms' */

  /* S-Function (fcncallgen): '<S5>/10ms1' incorporates:
   *  SubSystem: '<S5>/Beeper'
   */
  /* S-Function (ec5744_pdsslbu3): '<S333>/PowerDriverSwitch(HS)' */

  /* Set level beeper_state for the specified power driver switch */
  ec_gpio_write(83,beeper_state);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms1' */

  /* S-Function (fcncallgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/PwrTrainTempPrtct'
   */
  /* MinMax: '<S8>/Max' */
  rtb_Switch2_on = fmax(MCFL_TempInverter, MCFR_TempInverter);

  /* RelationalOperator: '<S98>/Compare' incorporates:
   *  Constant: '<S98>/Constant'
   */
  rtb_ignition = (rtb_Switch2_on > 40.0);

  /* RelationalOperator: '<S99>/Compare' incorporates:
   *  Constant: '<S99>/Constant'
   */
  rtb_LogicalOperator2 = (rtb_Switch2_on > 45.0);

  /* Logic: '<S8>/NOT' */
  rtb_Compare = !rtb_LogicalOperator2;

  /* Logic: '<S8>/AND' */
  rtb_ignition = (rtb_ignition && rtb_Compare);

  /* Switch: '<S8>/Switch' incorporates:
   *  Constant: '<S8>/Constant'
   */
  if (rtb_ignition) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table1' */
    elapseTime = look1_binlx(rtb_Switch2_on,
      VehCtrlMdel240926_2018b__ConstP.pooled19,
      VehCtrlMdel240926_2018b__ConstP.pooled18, 7U);
  } else {
    elapseTime = 0.0;
  }

  /* End of Switch: '<S8>/Switch' */

  /* SignalConversion generated from: '<S8>/Out1' */
  WhlSpdFR = elapseTime;

  /* Chart: '<S8>/Timer1' incorporates:
   *  Constant: '<S8>/Constant2'
   */
  VehCtrlMdel240926_20_Timer1(rtb_LogicalOperator2, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_g,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1);

  /* RelationalOperator: '<S101>/Compare' incorporates:
   *  Constant: '<S101>/Constant'
   */
  rtb_ignition = (MCU_Temp > 50.0);

  /* Chart: '<S8>/Timer2' incorporates:
   *  Constant: '<S8>/Constant3'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_d,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2);

  /* SignalConversion generated from: '<S8>/Out1' */
  EMRAX_Trq_CUT = VehCtrlMdel240926_2018b_amksp_B.Exit_d;

  /* Logic: '<S8>/NOT1' */
  rtb_Compare = !rtb_ignition;

  /* RelationalOperator: '<S100>/Compare' incorporates:
   *  Constant: '<S100>/Constant'
   */
  rtb_ignition = (MCU_Temp > 45.0);

  /* Logic: '<S8>/AND1' */
  rtb_ignition = (rtb_ignition && rtb_Compare);

  /* MinMax: '<S8>/Max1' */
  WhlSpdFL = fmax(MCU_Temp, motor_Temp);

  /* Switch: '<S8>/Switch1' incorporates:
   *  Constant: '<S8>/Constant1'
   */
  if (rtb_ignition) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table3' */
    elapseTime = look1_binlx(WhlSpdFL, VehCtrlMdel240926_2018b__ConstP.pooled19,
      VehCtrlMdel240926_2018b__ConstP.pooled18, 7U);
  } else {
    elapseTime = 0.0;
  }

  /* End of Switch: '<S8>/Switch1' */

  /* SignalConversion generated from: '<S8>/Out1' */
  WhlSpdRL_mps = elapseTime;

  /* Lookup_n-D: '<S8>/2-D Lookup Table2' */
  elapseTime = look1_binlx(WhlSpdFL,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable2_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable2_tableData, 6U);

  /* DataTypeConversion: '<S8>/Cast To Single' */
  rtb_CastToDouble = (real32_T)elapseTime;

  /* Gain: '<S8>/Gain' */
  rtb_CastToDouble *= 10.0F;

  /* SignalConversion generated from: '<S8>/Out1' */
  AMK_Trq_CUT = VehCtrlMdel240926_2018b_amksp_B.Exit_g;

  /* RelationalOperator: '<S102>/Compare' incorporates:
   *  Constant: '<S102>/Constant'
   */
  rtb_Compare = (rtb_Switch2_on > 35.0);

  /* SignalConversion generated from: '<S8>/Out1' */
  VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON = rtb_Compare;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/Function-Call Generator' */

  /* S-Function (fcncallgen): '<S1>/10ms1' incorporates:
   *  SubSystem: '<S1>/MoTrqReq'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.MoTrqReq_RESET_ELAPS_T) {
    FunctionCallSubsystem_ELAPS_T = 0U;
  } else {
    FunctionCallSubsystem_ELAPS_T =
      VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3 -
      VehCtrlMdel240926_2018b_amks_DW.MoTrqReq_PREV_T;
  }

  VehCtrlMdel240926_2018b_amks_DW.MoTrqReq_PREV_T =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
  VehCtrlMdel240926_2018b_amks_DW.MoTrqReq_RESET_ELAPS_T = false;

  /* Lookup_n-D: '<S7>/2-D Lookup Table1' incorporates:
   *  UnitDelay: '<S200>/Unit Delay1'
   */
  rtb_Add6_p = look2_iflf_binlx(rtb_Acc_POS,
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp01Data_l,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp02Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_tableData_g,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_maxIndex, 11U);

  /* Gain: '<S10>/Gain4' */
  rtb_Acc_POS = 0.1F * rtb_Add6_p;

  /* Gain: '<S10>/Gain24' */
  rtb_Acc_POS *= 0.5F;

  /* SampleTimeMath: '<S41>/sample time'
   *
   * About '<S41>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S41>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant42'
   */
  rtb_g_mpss1 = 2000.0 * elapseTime;

  /* UnitDelay: '<S44>/Delay Input2'
   *
   * Block description for '<S44>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_d;

  /* SampleTimeMath: '<S44>/sample time'
   *
   * About '<S44>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Gain4 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S44>/delta rise limit' */
  rtb_Ax = (real32_T)(1000.0 * rtb_Gain4);

  /* Gain: '<S10>/Gain21' */
  rtb_Switch2_on = 0.1020408163265306 * rtb_Add2;

  /* MATLAB Function: '<S10>/Wtarget' incorporates:
   *  Constant: '<S10>/Constant14'
   *  Constant: '<S10>/Constant15'
   *  Constant: '<S10>/Constant16'
   *  Constant: '<S10>/Constant17'
   *  Constant: '<S10>/Constant18'
   *  Constant: '<S10>/Constant19'
   *  UnitDelay: '<S200>/Unit Delay1'
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k *
    (real32_T)rtb_deltafalllimit_iz / (340.0F *
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k *
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k * 15.5799866F / 460.0F /
    440.0F / 1.592F / 2.0F + 1.592F);
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

  /* Sum: '<S44>/Difference Inputs1'
   *
   * Block description for '<S44>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_mn -= rtb_Add7;

  /* RelationalOperator: '<S59>/LowerRelop1' */
  rtb_ignition = (rtb_Switch2_mn > rtb_Ax);

  /* Switch: '<S59>/Switch2' */
  if (!rtb_ignition) {
    /* Product: '<S44>/delta fall limit' */
    rtb_Add10 = (real32_T)(-1000.0 * rtb_Gain4);

    /* RelationalOperator: '<S59>/UpperRelop' */
    rtb_ignition = (rtb_Switch2_mn < rtb_Add10);

    /* Switch: '<S59>/Switch' */
    if (rtb_ignition) {
      rtb_Switch2_mn = rtb_Add10;
    }

    /* End of Switch: '<S59>/Switch' */
    rtb_Ax = rtb_Switch2_mn;
  }

  /* End of Switch: '<S59>/Switch2' */

  /* Sum: '<S44>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S44>/Delay Input2'
   *
   * Block description for '<S44>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S44>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_d = rtb_Ax + rtb_Add7;

  /* Sum: '<S10>/Add' incorporates:
   *  UnitDelay: '<S44>/Delay Input2'
   *
   * Block description for '<S44>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdFL = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_d - rtb_UkYk1;

  /* UnitDelay: '<S10>/Unit Delay3' */
  rtb_ignition = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_l;

  /* Abs: '<S10>/Abs' */
  rtb_Gain4 = fabs(WhlSpdFL);

  /* RelationalOperator: '<S31>/Compare' incorporates:
   *  Constant: '<S31>/Constant'
   */
  rtb_LogicalOperator2 = (rtb_Gain4 > 10.0);

  /* Abs: '<S10>/Abs1' */
  rtb_Gain4 = fabs(rtb_UkYk1);

  /* RelationalOperator: '<S32>/Compare' incorporates:
   *  Constant: '<S32>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain4 > 5.0);

  /* RelationalOperator: '<S33>/Compare' incorporates:
   *  Constant: '<S33>/Constant'
   *  UnitDelay: '<S200>/Unit Delay1'
   */
  rtb_AND2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k > 3.0F);

  /* UnitDelay: '<S7>/Unit Delay' */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_n;

  /* Logic: '<S10>/AND' */
  rtb_UpperRelop_ir = (rtb_LogicalOperator2 && rtb_LowerRelop1_b && rtb_AND2 &&
                       (rtb_Gain4 != 0.0));

  /* Logic: '<S10>/Logical Operator4' */
  rtb_ignition = ((!rtb_ignition) && (!rtb_UpperRelop_ir));

  /* Abs: '<S10>/Abs2' */
  rtb_Gain4 = fabs(WhlSpdFL);

  /* RelationalOperator: '<S34>/Compare' incorporates:
   *  Constant: '<S34>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Gain4 < 10.0);

  /* RelationalOperator: '<S35>/Compare' incorporates:
   *  Constant: '<S35>/Constant'
   */
  rtb_AND2 = (rtb_Add2 < -5.0);

  /* Logic: '<S10>/OR' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir || rtb_AND2);

  /* Switch: '<S10>/Switch6' incorporates:
   *  Constant: '<S10>/Reset'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S10>/Add10' incorporates:
     *  Constant: '<S10>/Steptime'
     */
    rtb_Ax = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_n + 0.01F;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S10>/Switch6' */

  /* MinMax: '<S10>/Min' incorporates:
   *  Constant: '<S10>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_n = fminf(rtb_Ax, 0.5F);

  /* RelationalOperator: '<S10>/Relational Operator9' incorporates:
   *  Constant: '<S10>/ResetDelay'
   *  UnitDelay: '<S10>/Unit Delay4'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_n >=
                       0.5F);

  /* Logic: '<S10>/Logical Operator5' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_l = ((!rtb_ignition) &&
    (!rtb_UpperRelop_ir));

  /* Switch: '<S10>/Switch1' incorporates:
   *  Constant: '<S10>/Constant1'
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_l) {
    rtb_Gain4 = WhlSpdFL;
  } else {
    rtb_Gain4 = 0.0;
  }

  /* End of Switch: '<S10>/Switch1' */

  /* Product: '<S10>/Product3' */
  rtb_Gain5 = 2.0 * rtb_Gain4;

  /* Product: '<S10>/Product' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d *= WhlSpdFL;

  /* RelationalOperator: '<S30>/Compare' incorporates:
   *  Constant: '<S30>/Constant'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d <=
                       0.0);

  /* UnitDelay: '<S10>/Unit Delay' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m;

  /* Switch: '<S10>/Switch' incorporates:
   *  Constant: '<S10>/Constant'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (rtb_UpperRelop_ir) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = 0.0;
  }

  /* End of Switch: '<S10>/Switch' */

  /* Product: '<S10>/Product4' */
  WhlSpdRR_mps = rtb_Gain4;

  /* UnitDelay: '<S10>/Unit Delay1' */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_p;

  /* Product: '<S10>/Product5' */
  rtb_Switch2_on = rtb_Gain4;

  /* Sum: '<S10>/Add2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m =
    (VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d + WhlSpdRR_mps) -
    rtb_Switch2_on;

  /* MATLAB Function: '<S10>/MATLAB Function1' incorporates:
   *  Constant: '<S10>/Constant20'
   *  Constant: '<S10>/Constant21'
   *  Constant: '<S10>/Constant22'
   *  Constant: '<S10>/Constant23'
   *  Constant: '<S10>/Constant24'
   *  Constant: '<S10>/Constant25'
   *  UnitDelay: '<S200>/Unit Delay1'
   */
  rtb_Add7 = ((5.43088F / VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k *
               (-1.35294116F /
                VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k) -
               (-0.0458234884F /
                VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k /
                VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k - 1.0F) *
               -3.33390474F) * 105.0F * (real32_T)rtb_deltafalllimit_iz -
              -1.35294116F / VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k
              * 105.0F * (real32_T)rtb_UkYk1_ix) / (-0.0458234884F /
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k /
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k - 1.0F) / 100.0F;

  /* UnitDelay: '<S10>/Unit Delay6' */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE_i;

  /* Logic: '<S10>/Logical Operator3' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S10>/Switch3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay5'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S10>/Switch4' incorporates:
     *  Constant: '<S10>/InitZORE'
     *  UnitDelay: '<S10>/Unit Delay3'
     */
    if (!VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_l) {
      rtb_Add7 = 0.0F;
    }

    /* End of Switch: '<S10>/Switch4' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_b = rtb_Add7;
  }

  /* End of Switch: '<S10>/Switch3' */

  /* Gain: '<S10>/Gain18' incorporates:
   *  UnitDelay: '<S10>/Unit Delay5'
   */
  rtb_Ax = 0.01F * VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_b;

  /* Sum: '<S10>/Add1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay'
   */
  rtb_Switch2_on = (rtb_Gain5 +
                    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m) + rtb_Ax;

  /* Saturate: '<S10>/Saturation' */
  if (rtb_Switch2_on > 1000.0) {
    rtb_UkYk1_ix = 1000.0;
  } else if (rtb_Switch2_on < -1000.0) {
    rtb_UkYk1_ix = -1000.0;
  } else {
    rtb_UkYk1_ix = rtb_Switch2_on;
  }

  /* End of Saturate: '<S10>/Saturation' */

  /* Sum: '<S41>/Difference Inputs1' incorporates:
   *  UnitDelay: '<S41>/Delay Input2'
   *
   * Block description for '<S41>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S41>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = rtb_UkYk1_ix -
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_jk;

  /* RelationalOperator: '<S56>/LowerRelop1' */
  rtb_UpperRelop_ir = (WhlSpdRR_mps > rtb_g_mpss1);

  /* Switch: '<S56>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S41>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S56>/UpperRelop' */
    rtb_ignition = (WhlSpdRR_mps < elapseTime);

    /* Switch: '<S56>/Switch' */
    if (rtb_ignition) {
      WhlSpdRR_mps = elapseTime;
    }

    /* End of Switch: '<S56>/Switch' */
    rtb_g_mpss1 = WhlSpdRR_mps;
  }

  /* End of Switch: '<S56>/Switch2' */

  /* Sum: '<S41>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S41>/Delay Input2'
   *
   * Block description for '<S41>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S41>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_jk += rtb_g_mpss1;

  /* Gain: '<S10>/Gain8' */
  WhlSpdRR_mps = 0.017453292519943295 * FLWhlStrAng;

  /* Trigonometry: '<S10>/Cos' */
  WhlSpdRR_mps = cos(WhlSpdRR_mps);

  /* Gain: '<S10>/Gain11' */
  WhlSpdRR_mps *= 1.2;

  /* Sum: '<S10>/Add6' incorporates:
   *  Constant: '<S10>/Constant27'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = 90.0 - FLWhlStrAng;

  /* Gain: '<S10>/Gain9' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d *= 0.017453292519943295;

  /* Trigonometry: '<S10>/Cos1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = cos
    (VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d);

  /* Gain: '<S10>/Gain10' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d *= 1.522;

  /* Sum: '<S10>/Add7' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  WhlSpdRR_mps += VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d;

  /* Product: '<S10>/Product1' incorporates:
   *  UnitDelay: '<S41>/Delay Input2'
   *
   * Block description for '<S41>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps *= VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_jk;

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
  FLWhlStrAng = rtb_Acc_POS + WhlSpdRR_mps;

  /* Lookup_n-D: '<S10>/228' */
  elapseTime = look1_binlx(RPM, VehCtrlMdel240926_2018b__ConstP.u28_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.u28_tableData, 26U);

  /* Lookup_n-D: '<S10>/AMK' */
  WhlSpdRR_mps = look1_binlx(MCFL_ActualVelocity,
    VehCtrlMdel240926_2018b__ConstP.pooled15,
    VehCtrlMdel240926_2018b__ConstP.pooled14, 19U);

  /* Lookup_n-D: '<S10>/AMK1' */
  rtb_Gain5 = look1_binlx(MCFR_ActualVelocity,
    VehCtrlMdel240926_2018b__ConstP.pooled15,
    VehCtrlMdel240926_2018b__ConstP.pooled14, 19U);

  /* MATLAB Function: '<S10>/ÔØºÉ×ªÒÆ' incorporates:
   *  Constant: '<S10>/Constant10'
   *  Constant: '<S10>/Constant2'
   *  Constant: '<S10>/Constant3'
   *  Constant: '<S10>/Constant4'
   *  Constant: '<S10>/Constant5'
   *  Constant: '<S10>/Constant9'
   */
  rtb_Switch2_mn = (1666.0F - 340.0F * (real32_T)rtb_Add1 * 0.29F / 1.2F) *
    0.521984875F - 170.0F * (real32_T)rtb_Add2 * 0.29F / 1.592F;
  rtb_Add10 = (340.0F * (real32_T)rtb_Add1 * 0.29F / 1.2F + 1666.0F) *
    0.521984875F - 170.0F * (real32_T)rtb_Add2 * 0.29F / 1.592F;
  rtb_Ax = (1666.0F - 340.0F * (real32_T)rtb_Add1 * 0.29F / 1.2F) * 0.521984875F
    + 170.0F * (real32_T)rtb_Add2 * 0.29F / 1.592F;
  rtb_Add4_j = (340.0F * (real32_T)rtb_Add1 * 0.29F / 1.2F + 1666.0F) *
    0.521984875F + 170.0F * (real32_T)rtb_Add2 * 0.29F / 1.592F;

  /* Gain: '<S10>/Gain3' */
  rtb_g_mpss1 = 0.1020408163265306 * rtb_Add1;

  /* MATLAB Function: '<S10>/MATLAB Function' incorporates:
   *  Constant: '<S10>/Constant11'
   *  Constant: '<S10>/Constant12'
   *  Constant: '<S10>/Constant13'
   *  Constant: '<S10>/Constant26'
   */
  rtb_Add7 = rtb_Switch2_mn * 0.75F;
  rtb_Switch2_mn = rtb_Switch2_mn * (real32_T)rtb_g_mpss1 / 9.8F;
  rtb_Switch2_b0 = rtb_Add10 * 0.75F;
  rtb_MaxWhlSpd_mps_n = rtb_Add10 * (real32_T)rtb_g_mpss1 / 9.8F;
  rtb_Add10 = rtb_Ax * 0.75F;
  rtb_Ax = rtb_Ax * (real32_T)rtb_g_mpss1 / 9.8F;
  rtb_VxIMU_est = rtb_Add4_j * 0.75F;
  rtb_Add4_j = rtb_Add4_j * (real32_T)rtb_g_mpss1 / 9.8F;
  rtb_Switch2_b0 = fminf(sqrtf(rtb_Switch2_b0 * rtb_Switch2_b0 -
    rtb_MaxWhlSpd_mps_n * rtb_MaxWhlSpd_mps_n) * 0.2F / 11.4F, (real32_T)
    rtb_Gain5);
  rtb_Add7 = fminf(sqrtf(rtb_Add7 * rtb_Add7 - rtb_Switch2_mn * rtb_Switch2_mn) *
                   0.2F / 11.4F, (real32_T)WhlSpdRR_mps);
  rtb_Switch2_mn = fminf((sqrtf(rtb_VxIMU_est * rtb_VxIMU_est - rtb_Add4_j *
    rtb_Add4_j) + sqrtf(rtb_Add10 * rtb_Add10 - rtb_Ax * rtb_Ax)) / 2.0F * 0.2F /
    3.4F, (real32_T)elapseTime);

  /* Gain: '<S10>/Gain' */
  rtb_Ax = 0.95F * rtb_Switch2_b0;

  /* Gain: '<S10>/Gain1' */
  rtb_Add7 *= 0.95F;

  /* MinMax: '<S10>/Min1' */
  rtb_Add4_j = fminf(rtb_Ax, rtb_Add7);

  /* Sum: '<S10>/Add14' */
  WhlSpdRR_mps = rtb_Add4_j - FLWhlStrAng;

  /* RelationalOperator: '<S10>/Relational Operator' incorporates:
   *  Constant: '<S10>/Constant37'
   */
  rtb_LogicalOperator2 = (WhlSpdRR_mps < 0.0);

  /* Gain: '<S10>/Gain2' */
  rtb_Add4_j = 0.95F * rtb_Switch2_mn;

  /* Gain: '<S10>/Gain5' */
  rtb_Add6_p *= 0.8F;

  /* Sum: '<S10>/Add15' */
  rtb_Switch2_b0 = rtb_Add4_j - rtb_Add6_p;

  /* RelationalOperator: '<S10>/Relational Operator1' incorporates:
   *  Constant: '<S10>/Constant38'
   */
  rtb_Compare = (rtb_Switch2_b0 < 0.0F);

  /* Logic: '<S10>/AND1' */
  rtb_UpperRelop_ir = (rtb_LogicalOperator2 && rtb_Compare);

  /* Logic: '<S10>/OR1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_l ||
                       rtb_UpperRelop_ir);

  /* Switch: '<S10>/Switch2' incorporates:
   *  Constant: '<S10>/Constant32'
   */
  if (rtb_UpperRelop_ir) {
    WhlSpdRR_mps = 0.0;
  } else {
    /* Logic: '<S10>/NOT' */
    rtb_ignition = !rtb_Compare;

    /* Switch: '<S10>/Switch5' incorporates:
     *  Constant: '<S10>/Constant33'
     */
    if (!rtb_ignition) {
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

  /* UnitDelay: '<S43>/Delay Input2'
   *
   * Block description for '<S43>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n0;

  /* Sum: '<S43>/Difference Inputs1'
   *
   * Block description for '<S43>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 = elapseTime - WhlSpdRR_mps;

  /* SampleTimeMath: '<S43>/sample time'
   *
   * About '<S43>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S43>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant45'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = 1000.0 * elapseTime;

  /* RelationalOperator: '<S58>/LowerRelop1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_UpperRelop_ir = (rtb_Gain5 >
                       VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d);

  /* Switch: '<S58>/Switch2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S43>/delta fall limit' */
    elapseTime *= -1000.0;

    /* RelationalOperator: '<S58>/UpperRelop' */
    rtb_ignition = (rtb_Gain5 < elapseTime);

    /* Switch: '<S58>/Switch' */
    if (rtb_ignition) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S58>/Switch' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = rtb_Gain5;
  }

  /* End of Switch: '<S58>/Switch2' */

  /* Sum: '<S43>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S43>/Delay Input2'
   *
   * Block description for '<S43>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S43>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n0 =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d + WhlSpdRR_mps;

  /* Gain: '<S10>/Gain19' incorporates:
   *  UnitDelay: '<S43>/Delay Input2'
   *
   * Block description for '<S43>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = -VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n0;

  /* RelationalOperator: '<S47>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add6_p > rtb_Add4_j);

  /* Switch: '<S47>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Add6_p = rtb_Add4_j;
  } else {
    /* RelationalOperator: '<S47>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    rtb_ignition = (rtb_Add6_p < 0.0F);

    /* Switch: '<S47>/Switch' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    if (rtb_ignition) {
      rtb_Add6_p = 0.0F;
    }

    /* End of Switch: '<S47>/Switch' */
  }

  /* End of Switch: '<S47>/Switch2' */

  /* Sum: '<S10>/Add13' */
  elapseTime = rtb_Add6_p + WhlSpdRR_mps;

  /* RelationalOperator: '<S50>/LowerRelop1' */
  rtb_UpperRelop_ir = (elapseTime > rtb_Add4_j);

  /* Switch: '<S50>/Switch2' */
  if (rtb_UpperRelop_ir) {
    elapseTime = rtb_Add4_j;
  } else {
    /* RelationalOperator: '<S50>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    rtb_ignition = (elapseTime < 0.0);

    /* Switch: '<S50>/Switch' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    if (rtb_ignition) {
      elapseTime = 0.0;
    }

    /* End of Switch: '<S50>/Switch' */
  }

  /* End of Switch: '<S50>/Switch2' */

  /* UnitDelay: '<S40>/Delay Input2'
   *
   * Block description for '<S40>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* Sum: '<S40>/Difference Inputs1'
   *
   * Block description for '<S40>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 = elapseTime - WhlSpdRR_mps;

  /* SampleTimeMath: '<S40>/sample time'
   *
   * About '<S40>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S40>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = 2000.0 * elapseTime;

  /* RelationalOperator: '<S55>/LowerRelop1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_UpperRelop_ir = (rtb_Gain5 >
                       VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d);

  /* Switch: '<S55>/Switch2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S40>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S55>/UpperRelop' */
    rtb_ignition = (rtb_Gain5 < elapseTime);

    /* Switch: '<S55>/Switch' */
    if (rtb_ignition) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S55>/Switch' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = rtb_Gain5;
  }

  /* End of Switch: '<S55>/Switch2' */

  /* Sum: '<S40>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S40>/Delay Input2'
   *
   * Block description for '<S40>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S40>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d + WhlSpdRR_mps;

  /* Sum: '<S24>/Add' incorporates:
   *  Constant: '<S24>/Constant3'
   */
  WhlSpdRR_mps = 1.0 - WhlSpdRL_mps;

  /* Product: '<S24>/Product5' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = 63.0 * WhlSpdRR_mps;

  /* Product: '<S24>/Product' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  WhlSpdRR_mps = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d * 9550.0;

  /* Sum: '<S24>/Add1' incorporates:
   *  Constant: '<S24>/RPM_min'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = RPM + 10.0;

  /* MinMax: '<S24>/Max' incorporates:
   *  Constant: '<S24>/RPM_min1'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_Gain5 = fmax(VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d, 1.0);

  /* Product: '<S24>/Divide' */
  WhlSpdRR_mps /= rtb_Gain5;

  /* MinMax: '<S7>/MinMax2' incorporates:
   *  UnitDelay: '<S40>/Delay Input2'
   *
   * Block description for '<S40>/Delay Input2':
   *
   *  Store in Global RAM
   */
  elapseTime = fmin(WhlSpdRR_mps,
                    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk);

  /* Saturate: '<S29>/Saturation' */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k > 40.0F) {
    rtb_Add4_j = 40.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k < 0.0F) {
    rtb_Add4_j = 0.0F;
  } else {
    rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k;
  }

  /* Lookup_n-D: '<S29>/VehSpd_SlipTarget_mps' */
  rtb_Add6_p = look1_iflf_binlc(rtb_Add4_j,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled55, 3U);

  /* Sum: '<S29>/Add9' */
  rtb_Add6_p += rtb_Add4_j;

  /* Sum: '<S7>/Add1' */
  rtb_MaxWhlSpd_mps_n = rtb_deltafalllimit_aw + rtb_deltafalllimit_i;

  /* Gain: '<S7>/Gain2' */
  rtb_MaxWhlSpd_mps_n *= 0.5F;

  /* Saturate: '<S29>/Saturation1' */
  if (rtb_MaxWhlSpd_mps_n < 0.0F) {
    rtb_Saturation1_i = 0.0F;
  } else {
    rtb_Saturation1_i = rtb_MaxWhlSpd_mps_n;
  }

  /* End of Saturate: '<S29>/Saturation1' */

  /* Sum: '<S29>/Add1' */
  rtb_deltafalllimit_aw = rtb_Add6_p - rtb_Saturation1_i;

  /* UnitDelay: '<S92>/Unit Delay1' */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e;

  /* RelationalOperator: '<S29>/Relational Operator7' incorporates:
   *  Constant: '<S29>/Cal_DeltaV_mps'
   */
  rtb_AND2 = (rtb_deltafalllimit_aw < 0.0F);

  /* Logic: '<S92>/Logical Operator4' */
  rtb_UpperRelop_ir = ((!rtb_UpperRelop_ir) && (!rtb_AND2));

  /* Logic: '<S29>/Logical Operator2' */
  rtb_AND2 = !rtb_AND2;

  /* UnitDelay: '<S29>/Unit Delay4' */
  WhlSpdRR_mps = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE;

  /* RelationalOperator: '<S29>/Relational Operator8' incorporates:
   *  Constant: '<S29>/Cal_DeltaV_mps1'
   */
  rtb_LowerRelop1_b = (WhlSpdRR_mps > 235.0);

  /* Logic: '<S29>/Logical Operator1' */
  rtb_AND2 = (rtb_AND2 && rtb_LowerRelop1_b);

  /* Switch: '<S93>/Switch6' incorporates:
   *  Constant: '<S93>/Reset'
   */
  if (rtb_AND2) {
    /* Sum: '<S93>/Add10' incorporates:
     *  Constant: '<S93>/Steptime'
     */
    rtb_MaxWhlSpd_mps_n = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i +
      0.01F;
  } else {
    rtb_MaxWhlSpd_mps_n = 0.0F;
  }

  /* End of Switch: '<S93>/Switch6' */

  /* MinMax: '<S93>/Min' incorporates:
   *  Constant: '<S29>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i = fminf
    (rtb_MaxWhlSpd_mps_n, 0.1F);

  /* RelationalOperator: '<S93>/Relational Operator9' incorporates:
   *  Constant: '<S29>/ResetDelay'
   *  UnitDelay: '<S93>/Unit Delay1'
   */
  rtb_AND2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i >= 0.1F);

  /* UnitDelay: '<S29>/Unit Delay3' */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_a;

  /* Logic: '<S29>/Logical Operator3' */
  rtb_AND2 = (rtb_AND2 || rtb_LowerRelop1_b);

  /* Logic: '<S92>/Logical Operator5' incorporates:
   *  UnitDelay: '<S92>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e = ((!rtb_UpperRelop_ir) &&
    (!rtb_AND2));

  /* Switch: '<S29>/Switch6' incorporates:
   *  Constant: '<S29>/Verror_Reset'
   *  UnitDelay: '<S92>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e) {
    rtb_MaxWhlSpd_mps_n = rtb_deltafalllimit_aw;
  } else {
    rtb_MaxWhlSpd_mps_n = 0.0F;
  }

  /* End of Switch: '<S29>/Switch6' */

  /* Product: '<S29>/Product' incorporates:
   *  Constant: '<S29>/P_Gain'
   */
  rtb_deltafalllimit_i = rtb_MaxWhlSpd_mps_n * 40.0F;

  /* Sum: '<S29>/Add11' */
  WhlSpdRR_mps = elapseTime - rtb_deltafalllimit_i;

  /* UnitDelay: '<S29>/Unit Delay5' */
  rtb_Add6_p = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_l;

  /* Product: '<S29>/Product2' */
  rtb_Add6_p *= rtb_deltafalllimit_aw;

  /* RelationalOperator: '<S87>/Compare' incorporates:
   *  Constant: '<S87>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add6_p <= 0.0F);

  /* UnitDelay: '<S29>/Unit Delay' */
  rtb_Add6_p = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b;

  /* Switch: '<S29>/Switch3' incorporates:
   *  Constant: '<S29>/Verror_Reset1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add6_p = 0.0F;
  }

  /* End of Switch: '<S29>/Switch3' */

  /* Sum: '<S29>/Add2' */
  rtb_Add6_p += rtb_MaxWhlSpd_mps_n;

  /* Saturate: '<S29>/Saturation2' */
  if (rtb_Add6_p > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = 400.0F;
  } else if (rtb_Add6_p < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = rtb_Add6_p;
  }

  /* End of Saturate: '<S29>/Saturation2' */

  /* RelationalOperator: '<S97>/Compare' incorporates:
   *  UnitDelay: '<S92>/Unit Delay1'
   */
  rtb_ignition = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e;

  /* UnitDelay: '<S91>/Delay Input1'
   *
   * Block description for '<S91>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE;

  /* RelationalOperator: '<S91>/FixPt Relational Operator' */
  rtb_UpperRelop_ir = ((int32_T)rtb_ignition > (int32_T)rtb_UpperRelop_ir);

  /* Switch: '<S29>/Switch' incorporates:
   *  Constant: '<S29>/Integr_StartPoint'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S29>/Add4' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = elapseTime -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = 0.0;
  }

  /* End of Switch: '<S29>/Switch' */

  /* Lookup_n-D: '<S29>/VehicleStableTarget_mps' */
  rtb_MaxWhlSpd_mps_n = look1_iflf_binlc(rtb_Add4_j,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S29>/Add5' */
  rtb_MaxWhlSpd_mps_n += rtb_Add4_j;

  /* Sum: '<S29>/Add10' */
  rtb_MaxWhlSpd_mps_n = rtb_Saturation1_i - rtb_MaxWhlSpd_mps_n;

  /* RelationalOperator: '<S29>/Relational Operator' incorporates:
   *  Constant: '<S29>/Verror'
   */
  rtb_UpperRelop_ir = (rtb_MaxWhlSpd_mps_n < 0.0F);

  /* Logic: '<S29>/Logical Operator4' incorporates:
   *  UnitDelay: '<S92>/Unit Delay1'
   */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir &&
                       VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e);

  /* Switch: '<S29>/Switch1' incorporates:
   *  Constant: '<S29>/Trq_IReset'
   *  Constant: '<S29>/Trq_I_FF'
   */
  if (rtb_UpperRelop_ir) {
    rtb_MaxWhlSpd_mps_n = 20.0F;
  } else {
    rtb_MaxWhlSpd_mps_n = 0.0F;
  }

  /* End of Switch: '<S29>/Switch1' */

  /* Sum: '<S29>/Add6' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S29>/Unit Delay'
   */
  rtb_Gain5 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b +
               VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d) +
    rtb_MaxWhlSpd_mps_n;

  /* Product: '<S29>/Product1' */
  WhlSpdRL_mps = rtb_Gain5 * 10.0;

  /* RelationalOperator: '<S94>/LowerRelop1' */
  rtb_UpperRelop_ir = (WhlSpdRL_mps > WhlSpdRR_mps);

  /* Switch: '<S94>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Gain: '<S29>/Gain3' */
    rtb_Add6_p = -rtb_deltafalllimit_i;

    /* RelationalOperator: '<S94>/UpperRelop' */
    rtb_Compare_am = (WhlSpdRL_mps < rtb_Add6_p);

    /* Switch: '<S94>/Switch' */
    if (rtb_Compare_am) {
      WhlSpdRL_mps = rtb_Add6_p;
    }

    /* End of Switch: '<S94>/Switch' */
    WhlSpdRR_mps = WhlSpdRL_mps;
  }

  /* End of Switch: '<S94>/Switch2' */

  /* Sum: '<S29>/Add7' incorporates:
   *  UnitDelay: '<S29>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE = rtb_deltafalllimit_i +
    WhlSpdRR_mps;

  /* Lookup_n-D: '<S29>/VehicleStableTarget_mps1' */
  rtb_MaxWhlSpd_mps_n = look1_iflf_binlc(rtb_Add4_j,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S29>/Add13' */
  rtb_Add4_j += rtb_MaxWhlSpd_mps_n;

  /* Sum: '<S29>/Add12' */
  rtb_Add4_j = rtb_Saturation1_i - rtb_Add4_j;

  /* RelationalOperator: '<S29>/Relational Operator1' incorporates:
   *  Constant: '<S29>/Verror1'
   */
  rtb_UpperRelop_ir = (rtb_Add4_j < 0.0F);

  /* RelationalOperator: '<S29>/Relational Operator2' incorporates:
   *  UnitDelay: '<S29>/Unit Delay4'
   */
  rtb_AND2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE >= elapseTime);

  /* RelationalOperator: '<S89>/Compare' incorporates:
   *  Constant: '<S89>/Constant'
   *  UnitDelay: '<S29>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE <= 0.01);

  /* Logic: '<S29>/OR' */
  rtb_AND2 = (rtb_AND2 || rtb_LowerRelop1_b);

  /* Logic: '<S29>/Logical Operator5' incorporates:
   *  UnitDelay: '<S29>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_a = (rtb_UpperRelop_ir &&
    rtb_AND2);

  /* Switch: '<S29>/Switch2' incorporates:
   *  Switch: '<S29>/Switch7'
   *  UnitDelay: '<S29>/Unit Delay3'
   *  UnitDelay: '<S92>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e) {
    /* RelationalOperator: '<S95>/LowerRelop1' incorporates:
     *  Constant: '<S29>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S29>/Unit Delay4'
     */
    rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE > 235.0);

    /* Switch: '<S95>/Switch2' incorporates:
     *  Constant: '<S29>/TCS_TrqRequest_Max2'
     */
    if (rtb_Compare_am) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 235.0;
    } else {
      /* RelationalOperator: '<S95>/UpperRelop' incorporates:
       *  Constant: '<S29>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S29>/Unit Delay4'
       */
      rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE < 0.0);

      /* Switch: '<S95>/Switch' incorporates:
       *  Constant: '<S29>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S29>/Unit Delay4'
       */
      if (rtb_Compare_am) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE;
      }

      /* End of Switch: '<S95>/Switch' */
    }

    /* End of Switch: '<S95>/Switch2' */

    /* RelationalOperator: '<S96>/LowerRelop1' */
    rtb_Compare_am = (elapseTime >
                      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g);

    /* Switch: '<S96>/Switch2' */
    if (!rtb_Compare_am) {
      /* RelationalOperator: '<S96>/UpperRelop' incorporates:
       *  Constant: '<S29>/TCS_TrqRequest_Min1'
       */
      rtb_Compare_am = (elapseTime < 0.0);

      /* Switch: '<S96>/Switch' incorporates:
       *  Constant: '<S29>/TCS_TrqRequest_Min1'
       */
      if (rtb_Compare_am) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = elapseTime;
      }

      /* End of Switch: '<S96>/Switch' */
    }

    /* End of Switch: '<S96>/Switch2' */
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_a) {
      /* Switch: '<S29>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = elapseTime;
    }
  }

  /* End of Switch: '<S29>/Switch2' */

  /* UnitDelay: '<S81>/Unit Delay1' */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl;

  /* Saturate: '<S28>/Saturation' */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k > 40.0F) {
    rtb_MaxWhlSpd_mps_n = 40.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k < 0.0F) {
    rtb_MaxWhlSpd_mps_n = 0.0F;
  } else {
    rtb_MaxWhlSpd_mps_n = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Saturate: '<S28>/Saturation' */

  /* Lookup_n-D: '<S28>/VehSpd_SlipTarget_mps' */
  rtb_Add4_j = look1_iflf_binlc(rtb_MaxWhlSpd_mps_n,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled55, 3U);

  /* Sum: '<S28>/Add9' */
  rtb_Add4_j += rtb_MaxWhlSpd_mps_n;

  /* Saturate: '<S28>/Saturation1' */
  if (rtb_Divide > 50.0F) {
    rtb_Add6_p = 50.0F;
  } else if (rtb_Divide < 0.0F) {
    rtb_Add6_p = 0.0F;
  } else {
    rtb_Add6_p = rtb_Divide;
  }

  /* End of Saturate: '<S28>/Saturation1' */

  /* Sum: '<S28>/Add1' */
  rtb_Switch2_mn = rtb_Add4_j - rtb_Add6_p;

  /* RelationalOperator: '<S28>/Relational Operator7' incorporates:
   *  Constant: '<S28>/Cal_DeltaV_mps'
   */
  rtb_AND2 = (rtb_Switch2_mn < 0.0F);

  /* Logic: '<S81>/Logical Operator4' */
  rtb_UpperRelop_ir = ((!rtb_UpperRelop_ir) && (!rtb_AND2));

  /* Logic: '<S28>/Logical Operator2' */
  rtb_AND2 = !rtb_AND2;

  /* UnitDelay: '<S28>/Unit Delay4' */
  WhlSpdRR_mps = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i;

  /* RelationalOperator: '<S28>/Relational Operator8' incorporates:
   *  Constant: '<S28>/Cal_DeltaV_mps1'
   */
  rtb_LowerRelop1_b = (WhlSpdRR_mps > 235.0);

  /* Logic: '<S28>/Logical Operator1' */
  rtb_AND2 = (rtb_AND2 && rtb_LowerRelop1_b);

  /* Switch: '<S82>/Switch6' incorporates:
   *  Constant: '<S82>/Reset'
   */
  if (rtb_AND2) {
    /* Sum: '<S82>/Add10' incorporates:
     *  Constant: '<S82>/Steptime'
     */
    rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 + 0.01F;
  } else {
    rtb_Add4_j = 0.0F;
  }

  /* End of Switch: '<S82>/Switch6' */

  /* MinMax: '<S82>/Min' incorporates:
   *  Constant: '<S28>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 = fminf(rtb_Add4_j, 0.1F);

  /* RelationalOperator: '<S82>/Relational Operator9' incorporates:
   *  Constant: '<S28>/ResetDelay'
   *  UnitDelay: '<S82>/Unit Delay1'
   */
  rtb_AND2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 >= 0.1F);

  /* UnitDelay: '<S28>/Unit Delay3' */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e;

  /* Logic: '<S28>/Logical Operator3' */
  rtb_AND2 = (rtb_AND2 || rtb_LowerRelop1_b);

  /* Logic: '<S81>/Logical Operator5' incorporates:
   *  UnitDelay: '<S81>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl = ((!rtb_UpperRelop_ir) &&
    (!rtb_AND2));

  /* UnitDelay: '<S72>/Unit Delay1' */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp;

  /* Saturate: '<S27>/Saturation' */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k > 40.0F) {
    rtb_Add4_j = 40.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k < 0.0F) {
    rtb_Add4_j = 0.0F;
  } else {
    rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Saturate: '<S27>/Saturation' */

  /* Lookup_n-D: '<S27>/VehSpd_SlipTarget_mps' */
  rtb_Switch_jg = look1_iflf_binlc(rtb_Add4_j,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled55, 3U);

  /* Sum: '<S27>/Add9' */
  rtb_Switch_jg += rtb_Add4_j;

  /* Saturate: '<S27>/Saturation1' */
  if (rtb_Add < 0.0F) {
    rtb_VxIMU_est = 0.0F;
  } else {
    rtb_VxIMU_est = rtb_Add;
  }

  /* End of Saturate: '<S27>/Saturation1' */

  /* Sum: '<S27>/Add1' */
  rtb_Add10 = rtb_Switch_jg - rtb_VxIMU_est;

  /* RelationalOperator: '<S27>/Relational Operator7' incorporates:
   *  Constant: '<S27>/Cal_DeltaV_mps'
   */
  rtb_AND2 = (rtb_Add10 < 0.0F);

  /* Logic: '<S72>/Logical Operator4' */
  rtb_UpperRelop_ir = ((!rtb_UpperRelop_ir) && (!rtb_AND2));

  /* Logic: '<S27>/Logical Operator2' */
  rtb_AND2 = !rtb_AND2;

  /* UnitDelay: '<S27>/Unit Delay4' */
  WhlSpdRR_mps = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l;

  /* RelationalOperator: '<S27>/Relational Operator8' incorporates:
   *  Constant: '<S27>/Cal_DeltaV_mps1'
   */
  rtb_LowerRelop1_b = (WhlSpdRR_mps > 235.0);

  /* Logic: '<S27>/Logical Operator1' */
  rtb_AND2 = (rtb_AND2 && rtb_LowerRelop1_b);

  /* Switch: '<S73>/Switch6' incorporates:
   *  Constant: '<S73>/Reset'
   */
  if (rtb_AND2) {
    /* Sum: '<S73>/Add10' incorporates:
     *  Constant: '<S73>/Steptime'
     */
    rtb_Switch_jg = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h + 0.01F;
  } else {
    rtb_Switch_jg = 0.0F;
  }

  /* End of Switch: '<S73>/Switch6' */

  /* MinMax: '<S73>/Min' incorporates:
   *  Constant: '<S27>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h = fminf(rtb_Switch_jg,
    0.1F);

  /* RelationalOperator: '<S73>/Relational Operator9' incorporates:
   *  Constant: '<S27>/ResetDelay'
   *  UnitDelay: '<S73>/Unit Delay1'
   */
  rtb_AND2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h >= 0.1F);

  /* UnitDelay: '<S27>/Unit Delay3' */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i;

  /* Logic: '<S27>/Logical Operator3' */
  rtb_AND2 = (rtb_AND2 || rtb_LowerRelop1_b);

  /* Logic: '<S72>/Logical Operator5' incorporates:
   *  UnitDelay: '<S72>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp = ((!rtb_UpperRelop_ir) &&
    (!rtb_AND2));

  /* Chart: '<S7>/Chart' */
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c7_VehCtrlMdel240926_
      == 0U) {
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c7_VehCtrlMdel240926_ =
      1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_B = 3U;
    VehCtrlMdel240926_2018b_amks_DW.b = rtb_Add2 * rtb_Add2 + rtb_Add1 *
      rtb_Add1;
    VehCtrlMdel240926_2018b_amks_DW.b = sqrt(VehCtrlMdel240926_2018b_amks_DW.b);
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_C = 3U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_D = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_E = 1U;
  } else {
    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_B) {
     case VehCtrlMdel240_IN_DYC_Disenable:
      VehCtrlMdel240926_2018b_amksp_B.DYC_Enable_OUT = 0.0;
      break;

     case VehCtrlMdel240926_IN_DYC_Enable:
      VehCtrlMdel240926_2018b_amksp_B.DYC_Enable_OUT = 1.0;
      rtb_Compare_am = ((rtb_UkYk1 >= 50.0) || (rtb_Add1 >= 5.0) ||
                        (VehCtrlMdel240926_2018b_amks_DW.b >= 5.0));
      if (rtb_Compare_am) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_B = 3U;
      }
      break;

     default:
      /* case IN_InitState: */
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_B = 1U;
      VehCtrlMdel240926_2018b_amksp_B.DYC_Enable_OUT = 0.0;
      VehCtrlMdel240926_2018b_amks_DW.DYC_flag = 0.0;
      break;
    }

    if (fabs(rtb_Add1) > 0.5) {
      WhlSpdRR_mps = atan(rtb_Add2 / rtb_Add1) * 180.0 / 3.1415926535897931;
    } else {
      WhlSpdRR_mps = 0.0;
    }

    VehCtrlMdel240926_2018b_amks_DW.b = rtb_Add2 * rtb_Add2 + rtb_Add1 *
      rtb_Add1;
    VehCtrlMdel240926_2018b_amks_DW.b = sqrt(VehCtrlMdel240926_2018b_amks_DW.b);
    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_C) {
     case VehCtrlMdel240926_2018b_am_IN_B:
      rtb_Compare_am = ((!(VehCtrlMdel240926_2018b_amks_DW.DYC_flag != 0.0)) ||
                        (rtb_UkYk1 > 30.0) || (WhlSpdRR_mps > 30.0) ||
                        (WhlSpdRR_mps > -30.0) ||
                        (VehCtrlMdel240926_2018b_amks_DW.b > 3.0));
      if (rtb_Compare_am) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_C = 3U;
      }
      break;

     case VehCtrlMdel240926_2018b_am_IN_C:
      break;

     default:
      /* case IN_F_TVD_TCS_STATE: */
      rtb_Compare_am = ((!(VehCtrlMdel240926_2018b_amks_DW.DYC_flag != 0.0)) ||
                        (rtb_UkYk1 > 30.0) || (WhlSpdRR_mps > 30.0) ||
                        (WhlSpdRR_mps > -30.0) ||
                        (VehCtrlMdel240926_2018b_amks_DW.b > 3.0));
      if (rtb_Compare_am) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_C = 2U;
        VehCtrlMdel240926_2018b_amksp_B.TCSR_Enable_OUT = 0.0;
      }
      break;
    }

    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_D) {
     case VehCtrlMdel240926_IN_InitState2:
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_D = 2U;
      VehCtrlMdel240926_2018b_amksp_B.TCSR_Enable_OUT = 0.0;
      break;

     case VehCtrlMdel24_IN_TCSR_Disenable:
      break;

     default:
      /* case IN_TCSR_Enable: */
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_D = 2U;
      VehCtrlMdel240926_2018b_amksp_B.TCSR_Enable_OUT = 0.0;
      break;
    }

    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_E) {
     case VehCtrlMdel240926_IN_InitState1:
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_E = 2U;
      VehCtrlMdel240926_2018b_amksp_B.TCSF_Enable_OUT = 0.0;
      break;

     case VehCtrlMdel24_IN_TCSF_Disenable:
      VehCtrlMdel240926_2018b_amksp_B.TCSF_Enable_OUT = 0.0;
      break;

     default:
      /* case IN_TCSF_Enable: */
      VehCtrlMdel240926_2018b_amksp_B.TCSF_Enable_OUT = 1.0;
      if (VehCtrlMdel240926_2018b_amks_DW.b > 5.0) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_E = 1U;
      }
      break;
    }
  }

  /* End of Chart: '<S7>/Chart' */

  /* Switch: '<S7>/Switch5' */
  if (VehCtrlMdel240926_2018b_amksp_B.TCSR_Enable_OUT != 0.0) {
    /* Abs: '<S29>/Abs' */
    rtb_UkYk1 = fabs(rtb_deltafalllimit_iz);

    /* RelationalOperator: '<S88>/Compare' incorporates:
     *  Constant: '<S88>/Constant'
     */
    rtb_Compare_am = (rtb_UkYk1 <= 20.0);

    /* RelationalOperator: '<S90>/Compare' incorporates:
     *  Constant: '<S90>/Constant'
     */
    rtb_LowerRelop1_b = (rtb_Saturation1_i > 0.0F);

    /* Logic: '<S29>/Logical Operator6' */
    rtb_LowerRelop1_b = (rtb_LowerRelop1_b && rtb_Compare_am);

    /* Switch: '<S29>/Switch5' incorporates:
     *  UnitDelay: '<S29>/Unit Delay2'
     */
    if (rtb_LowerRelop1_b) {
      elapseTime = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g;
    }

    /* End of Switch: '<S29>/Switch5' */
    WhlSpdRR_mps = elapseTime;
  } else {
    WhlSpdRR_mps = elapseTime;
  }

  /* End of Switch: '<S7>/Switch5' */

  /* Gain: '<S7>/Gain6' */
  rtb_UkYk1 = 3.8461538461538463 * WhlSpdRR_mps;

  /* Lookup_n-D: '<S7>/BrakeCompensateCoefRear' */
  rtb_Switch_jg = look1_iflf_binlc((real32_T)Brk_F,
    VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefRear_bp01Dat,
    VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefRear_tableDa, 1U);

  /* RelationalOperator: '<S21>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1 > rtb_Switch_jg);

  /* Switch: '<S21>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* RelationalOperator: '<S21>/UpperRelop' incorporates:
     *  Constant: '<S7>/Constant5'
     */
    rtb_Compare_am = (rtb_UkYk1 < 0.0);

    /* Switch: '<S21>/Switch' incorporates:
     *  Constant: '<S7>/Constant5'
     */
    if (rtb_Compare_am) {
      rtb_Switch_jg = 0.0F;
    } else {
      rtb_Switch_jg = (real32_T)rtb_UkYk1;
    }

    /* End of Switch: '<S21>/Switch' */
  }

  /* End of Switch: '<S21>/Switch2' */

  /* Switch: '<S7>/Switch2' */
  rtb_Saturation1_i = rtb_Switch_jg;

  /* UnitDelay: '<S18>/Delay Input2'
   *
   * Block description for '<S18>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch_jg = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd;

  /* Sum: '<S18>/Difference Inputs1'
   *
   * Block description for '<S18>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1_ea = rtb_Saturation1_i - rtb_Switch_jg;

  /* SampleTimeMath: '<S18>/sample time'
   *
   * About '<S18>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S18>/delta rise limit' */
  rtb_Saturation1_i = (real32_T)(25000.0 * elapseTime);

  /* RelationalOperator: '<S60>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1_ea > rtb_Saturation1_i);

  /* Switch: '<S60>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S18>/delta fall limit' */
    rtb_Saturation1_i = (real32_T)(-25000.0 * elapseTime);

    /* RelationalOperator: '<S60>/UpperRelop' */
    rtb_Compare_am = (rtb_UkYk1_ea < rtb_Saturation1_i);

    /* Switch: '<S60>/Switch' */
    if (rtb_Compare_am) {
      rtb_UkYk1_ea = rtb_Saturation1_i;
    }

    /* End of Switch: '<S60>/Switch' */
    rtb_Saturation1_i = rtb_UkYk1_ea;
  }

  /* End of Switch: '<S60>/Switch2' */

  /* Saturate: '<S7>/Saturation1' incorporates:
   *  Sum: '<S18>/Difference Inputs2'
   *  UnitDelay: '<S18>/Delay Input2'
   *
   * Block description for '<S18>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S18>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd = rtb_Saturation1_i +
    rtb_Switch_jg;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd > 1000.0F) {
    TrqR_cmd = 1000.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd < 0.0F) {
    TrqR_cmd = 0.0F;
  } else {
    TrqR_cmd = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd;
  }

  /* End of Saturate: '<S7>/Saturation1' */

  /* UnitDelay: '<S38>/Delay Input2'
   *
   * Block description for '<S38>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mb;

  /* SampleTimeMath: '<S38>/sample time'
   *
   * About '<S38>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S38>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = 2000.0 * elapseTime;

  /* RelationalOperator: '<S45>/LowerRelop1' */
  rtb_UpperRelop_ir = (FLWhlStrAng > rtb_Ax);

  /* Switch: '<S45>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Gain5 = rtb_Ax;
  } else {
    /* RelationalOperator: '<S45>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    rtb_Compare_am = (FLWhlStrAng < 0.0);

    /* Switch: '<S45>/Switch' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    if (rtb_Compare_am) {
      FLWhlStrAng = 0.0;
    }

    /* End of Switch: '<S45>/Switch' */
    rtb_Gain5 = FLWhlStrAng;
  }

  /* End of Switch: '<S45>/Switch2' */

  /* UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Saturation1_i = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_fo;

  /* SampleTimeMath: '<S42>/sample time'
   *
   * About '<S42>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Gain4 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S42>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant34'
   */
  rtb_Switch_jg = (real32_T)(1000.0 * rtb_Gain4);

  /* Logic: '<S10>/AND2' */
  rtb_UpperRelop_ir = (rtb_LogicalOperator2 && rtb_Compare);

  /* Logic: '<S10>/OR2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_l ||
                       rtb_UpperRelop_ir);

  /* Switch: '<S10>/Switch7' incorporates:
   *  Constant: '<S10>/Constant39'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_b0 = 0.0F;
  } else {
    /* Logic: '<S10>/NOT1' */
    rtb_LogicalOperator2 = !rtb_LogicalOperator2;

    /* Switch: '<S10>/Switch8' incorporates:
     *  Constant: '<S10>/Constant40'
     */
    if (!rtb_LogicalOperator2) {
      rtb_Switch2_b0 = 0.0F;
    }

    /* End of Switch: '<S10>/Switch8' */
  }

  /* End of Switch: '<S10>/Switch7' */

  /* Saturate: '<S10>/Saturation2' */
  if (rtb_Switch2_b0 > 100.0F) {
    rtb_Switch2_b0 = 100.0F;
  } else {
    if (rtb_Switch2_b0 < 0.0F) {
      rtb_Switch2_b0 = 0.0F;
    }
  }

  /* End of Saturate: '<S10>/Saturation2' */

  /* Sum: '<S42>/Difference Inputs1'
   *
   * Block description for '<S42>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 -= rtb_Saturation1_i;

  /* RelationalOperator: '<S57>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch_jg);

  /* Switch: '<S57>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S42>/delta fall limit' */
    rtb_Switch_jg = (real32_T)(-1000.0 * rtb_Gain4);

    /* RelationalOperator: '<S57>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Switch2_b0 < rtb_Switch_jg);

    /* Switch: '<S57>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Switch2_b0 = rtb_Switch_jg;
    }

    /* End of Switch: '<S57>/Switch' */
    rtb_Switch_jg = rtb_Switch2_b0;
  }

  /* End of Switch: '<S57>/Switch2' */

  /* Sum: '<S42>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_fo = rtb_Switch_jg +
    rtb_Saturation1_i;

  /* Gain: '<S10>/Gain20' incorporates:
   *  UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_b0 = -VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_fo;

  /* Sum: '<S10>/Add11' */
  rtb_UkYk1 = rtb_Gain5 + rtb_Switch2_b0;

  /* RelationalOperator: '<S48>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1 > rtb_Ax);

  /* Switch: '<S48>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_UkYk1 = rtb_Ax;
  } else {
    /* RelationalOperator: '<S48>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    rtb_LogicalOperator2 = (rtb_UkYk1 < 0.0);

    /* Switch: '<S48>/Switch' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    if (rtb_LogicalOperator2) {
      rtb_UkYk1 = 0.0;
    }

    /* End of Switch: '<S48>/Switch' */
  }

  /* End of Switch: '<S48>/Switch2' */

  /* Sum: '<S38>/Difference Inputs1'
   *
   * Block description for '<S38>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 -= WhlSpdRR_mps;

  /* RelationalOperator: '<S53>/LowerRelop1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_UpperRelop_ir = (rtb_UkYk1 >
                       VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d);

  /* Switch: '<S53>/Switch2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S38>/delta fall limit' */
    rtb_deltafalllimit_iz = -2000.0 * elapseTime;

    /* RelationalOperator: '<S53>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S53>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S53>/Switch' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = rtb_UkYk1;
  }

  /* End of Switch: '<S53>/Switch2' */

  /* Sum: '<S38>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S38>/Delay Input2'
   *
   * Block description for '<S38>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S38>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mb =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d + WhlSpdRR_mps;

  /* Sum: '<S24>/Add4' incorporates:
   *  Constant: '<S24>/Constant'
   */
  WhlSpdRR_mps = 1.0 - WhlSpdFR;

  /* Product: '<S24>/Product4' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = 7.0 * WhlSpdRR_mps;

  /* Product: '<S24>/Product1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d * 9550.0;

  /* Sum: '<S24>/Add2' incorporates:
   *  Constant: '<S24>/RPM_min2'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = MCFR_ActualVelocity +
    10.0;

  /* MinMax: '<S24>/Max1' incorporates:
   *  Constant: '<S24>/RPM_min3'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_Gain4 = fmax(VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d, 1.0);

  /* Product: '<S24>/Divide1' */
  rtb_Gain5 /= rtb_Gain4;

  /* MinMax: '<S7>/MinMax' incorporates:
   *  UnitDelay: '<S38>/Delay Input2'
   *
   * Block description for '<S38>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_deltafalllimit_iz = fmin(rtb_Gain5,
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mb);

  /* Switch: '<S28>/Switch6' incorporates:
   *  Constant: '<S28>/Verror_Reset'
   *  UnitDelay: '<S81>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl) {
    rtb_Saturation1_i = rtb_Switch2_mn;
  } else {
    rtb_Saturation1_i = 0.0F;
  }

  /* End of Switch: '<S28>/Switch6' */

  /* Product: '<S28>/Product' incorporates:
   *  Constant: '<S28>/P_Gain'
   */
  rtb_Ax = rtb_Saturation1_i * 40.0F;

  /* Sum: '<S28>/Add11' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = rtb_deltafalllimit_iz -
    rtb_Ax;

  /* UnitDelay: '<S28>/Unit Delay5' */
  rtb_Switch_jg = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* Product: '<S28>/Product2' */
  rtb_Switch_jg *= rtb_Switch2_mn;

  /* RelationalOperator: '<S78>/Compare' incorporates:
   *  Constant: '<S78>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch_jg <= 0.0F);

  /* UnitDelay: '<S28>/Unit Delay' */
  rtb_Switch_jg = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f;

  /* Switch: '<S28>/Switch3' incorporates:
   *  Constant: '<S28>/Verror_Reset1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Switch_jg = 0.0F;
  }

  /* End of Switch: '<S28>/Switch3' */

  /* Sum: '<S28>/Add2' */
  rtb_Switch_jg += rtb_Saturation1_i;

  /* Saturate: '<S28>/Saturation2' */
  if (rtb_Switch_jg > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f = 400.0F;
  } else if (rtb_Switch_jg < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f = rtb_Switch_jg;
  }

  /* End of Saturate: '<S28>/Saturation2' */

  /* RelationalOperator: '<S86>/Compare' incorporates:
   *  UnitDelay: '<S81>/Unit Delay1'
   */
  rtb_LogicalOperator2 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl;

  /* UnitDelay: '<S80>/Delay Input1'
   *
   * Block description for '<S80>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_j;

  /* RelationalOperator: '<S80>/FixPt Relational Operator' */
  rtb_UpperRelop_ir = ((int32_T)rtb_LogicalOperator2 > (int32_T)
                       rtb_UpperRelop_ir);

  /* Switch: '<S28>/Switch' incorporates:
   *  Constant: '<S28>/Integr_StartPoint'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S28>/Add4' */
    rtb_Gain5 = rtb_deltafalllimit_iz -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_f;
  } else {
    rtb_Gain5 = 0.0;
  }

  /* End of Switch: '<S28>/Switch' */

  /* Lookup_n-D: '<S28>/VehicleStableTarget_mps' */
  rtb_Saturation1_i = look1_iflf_binlc(rtb_MaxWhlSpd_mps_n,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S28>/Add5' */
  rtb_Saturation1_i += rtb_MaxWhlSpd_mps_n;

  /* Sum: '<S28>/Add10' */
  rtb_Saturation1_i = rtb_Add6_p - rtb_Saturation1_i;

  /* RelationalOperator: '<S28>/Relational Operator' incorporates:
   *  Constant: '<S28>/Verror'
   */
  rtb_UpperRelop_ir = (rtb_Saturation1_i < 0.0F);

  /* Logic: '<S28>/Logical Operator4' incorporates:
   *  UnitDelay: '<S81>/Unit Delay1'
   */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir &&
                       VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl);

  /* Switch: '<S28>/Switch1' incorporates:
   *  Constant: '<S28>/Trq_IReset'
   *  Constant: '<S28>/Trq_I_FF'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Saturation1_i = 20.0F;
  } else {
    rtb_Saturation1_i = 0.0F;
  }

  /* End of Switch: '<S28>/Switch1' */

  /* Sum: '<S28>/Add6' incorporates:
   *  UnitDelay: '<S28>/Unit Delay'
   */
  rtb_Gain4 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f + rtb_Gain5) +
    rtb_Saturation1_i;

  /* Product: '<S28>/Product1' */
  rtb_UkYk1 = rtb_Gain4 * 10.0;

  /* RelationalOperator: '<S83>/LowerRelop1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_UpperRelop_ir = (rtb_UkYk1 >
                       VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d);

  /* Switch: '<S83>/Switch2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (!rtb_UpperRelop_ir) {
    /* Gain: '<S28>/Gain3' */
    rtb_Saturation1_i = -rtb_Ax;

    /* RelationalOperator: '<S83>/UpperRelop' */
    rtb_Compare = (rtb_UkYk1 < rtb_Saturation1_i);

    /* Switch: '<S83>/Switch' */
    if (rtb_Compare) {
      rtb_UkYk1 = rtb_Saturation1_i;
    }

    /* End of Switch: '<S83>/Switch' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = rtb_UkYk1;
  }

  /* End of Switch: '<S83>/Switch2' */

  /* Sum: '<S28>/Add7' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S28>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i = rtb_Ax +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d;

  /* Lookup_n-D: '<S28>/VehicleStableTarget_mps1' */
  rtb_Saturation1_i = look1_iflf_binlc(rtb_MaxWhlSpd_mps_n,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S28>/Add13' */
  rtb_MaxWhlSpd_mps_n += rtb_Saturation1_i;

  /* Sum: '<S28>/Add12' */
  rtb_Add6_p -= rtb_MaxWhlSpd_mps_n;

  /* RelationalOperator: '<S28>/Relational Operator1' incorporates:
   *  Constant: '<S28>/Verror1'
   */
  rtb_UpperRelop_ir = (rtb_Add6_p < 0.0F);

  /* RelationalOperator: '<S28>/Relational Operator2' incorporates:
   *  UnitDelay: '<S28>/Unit Delay4'
   */
  rtb_AND2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i >=
              rtb_deltafalllimit_iz);

  /* RelationalOperator: '<S79>/Compare' incorporates:
   *  Constant: '<S79>/Constant'
   *  UnitDelay: '<S28>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i <=
                       0.01);

  /* Logic: '<S28>/OR' */
  rtb_AND2 = (rtb_AND2 || rtb_LowerRelop1_b);

  /* Logic: '<S28>/Logical Operator5' incorporates:
   *  UnitDelay: '<S28>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e = (rtb_UpperRelop_ir &&
    rtb_AND2);

  /* Switch: '<S28>/Switch2' incorporates:
   *  Switch: '<S28>/Switch7'
   *  UnitDelay: '<S28>/Unit Delay3'
   *  UnitDelay: '<S81>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl) {
    /* RelationalOperator: '<S84>/LowerRelop1' incorporates:
     *  Constant: '<S28>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S28>/Unit Delay4'
     */
    rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i > 235.0);

    /* Switch: '<S84>/Switch2' incorporates:
     *  Constant: '<S28>/TCS_TrqRequest_Max2'
     */
    if (rtb_Compare) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j = 235.0;
    } else {
      /* RelationalOperator: '<S84>/UpperRelop' incorporates:
       *  Constant: '<S28>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S28>/Unit Delay4'
       */
      rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i < 0.0);

      /* Switch: '<S84>/Switch' incorporates:
       *  Constant: '<S28>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S28>/Unit Delay4'
       */
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i;
      }

      /* End of Switch: '<S84>/Switch' */
    }

    /* End of Switch: '<S84>/Switch2' */

    /* RelationalOperator: '<S85>/LowerRelop1' */
    rtb_Compare = (rtb_deltafalllimit_iz >
                   VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j);

    /* Switch: '<S85>/Switch2' */
    if (!rtb_Compare) {
      /* RelationalOperator: '<S85>/UpperRelop' incorporates:
       *  Constant: '<S28>/TCS_TrqRequest_Min1'
       */
      rtb_Compare = (rtb_deltafalllimit_iz < 0.0);

      /* Switch: '<S85>/Switch' incorporates:
       *  Constant: '<S28>/TCS_TrqRequest_Min1'
       */
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j =
          rtb_deltafalllimit_iz;
      }

      /* End of Switch: '<S85>/Switch' */
    }

    /* End of Switch: '<S85>/Switch2' */
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e) {
      /* Switch: '<S28>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j =
        rtb_deltafalllimit_iz;
    }
  }

  /* End of Switch: '<S28>/Switch2' */

  /* Sum: '<S7>/Add' incorporates:
   *  Constant: '<S7>/Constant1'
   *  UnitDelay: '<S200>/Unit Delay1'
   */
  rtb_Saturation1_i = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k + 5.0F;

  /* Sum: '<S25>/Add' */
  rtb_Saturation1_i -= rtb_Divide;

  /* RelationalOperator: '<S63>/Compare' incorporates:
   *  Constant: '<S63>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Saturation1_i < 0.0F);

  /* Switch: '<S25>/Switch' incorporates:
   *  Constant: '<S25>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_UkYk1 = rtb_Saturation1_i;
  } else {
    rtb_UkYk1 = 0.0;
  }

  /* End of Switch: '<S25>/Switch' */

  /* Gain: '<S25>/Gain1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = 5.0 * rtb_UkYk1;

  /* Logic: '<S25>/NOT' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* UnitDelay: '<S25>/Unit Delay' */
  rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p;

  /* Switch: '<S25>/Switch1' incorporates:
   *  Constant: '<S25>/Constant2'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Gain5 = 0.0;
  }

  /* End of Switch: '<S25>/Switch1' */

  /* Sum: '<S25>/Add2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S25>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d + rtb_Gain5;

  /* Logic: '<S7>/Logical Operator5' */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amksp_B.MCFR_TorqueOn &&
                       VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueOn);

  /* Logic: '<S7>/Logical Operator6' */
  TroqueOn = !rtb_UpperRelop_ir;

  /* Logic: '<S7>/Logical Operator2' */
  rtb_UpperRelop_ir = !VehCtrlMdel240926_2018b_amksp_B.VehReady;

  /* Logic: '<S7>/Logical Operator3' */
  rtb_AND2 = (TroqueOn || rtb_UpperRelop_ir || (Trq_CUT != 0.0));

  /* Logic: '<S7>/Logical Operator1' */
  rtb_Compare = ((EMRAX_Trq_CUT != 0.0) || rtb_AND2);

  /* Logic: '<S7>/Logical Operator4' */
  Trq_CUT_final = ((AMK_Trq_CUT != 0.0) || rtb_AND2 || rtb_Compare);

  /* Switch: '<S7>/Switch3' incorporates:
   *  Constant: '<S7>/Constant6'
   *  Switch: '<S7>/Switch6'
   */
  if (Trq_CUT_final) {
    rtb_Divide = 0.0F;
  } else {
    if (VehCtrlMdel240926_2018b_amksp_B.TCSF_Enable_OUT != 0.0) {
      /* Switch: '<S7>/Switch6' incorporates:
       *  UnitDelay: '<S28>/Unit Delay2'
       */
      rtb_deltafalllimit_iz =
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j;
    }

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefFront1' */
    rtb_Divide = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240926_2018b__ConstP.pooled34,
      VehCtrlMdel240926_2018b__ConstP.pooled33, 1U);

    /* RelationalOperator: '<S22>/LowerRelop1' */
    rtb_Compare_am = (rtb_deltafalllimit_iz > rtb_Divide);

    /* Switch: '<S22>/Switch2' */
    if (!rtb_Compare_am) {
      /* RelationalOperator: '<S22>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      rtb_Compare_am = (rtb_deltafalllimit_iz < 0.0);

      /* Switch: '<S22>/Switch' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      if (rtb_Compare_am) {
        rtb_Divide = 0.0F;
      } else {
        rtb_Divide = (real32_T)rtb_deltafalllimit_iz;
      }

      /* End of Switch: '<S22>/Switch' */
    }

    /* End of Switch: '<S22>/Switch2' */

    /* Sum: '<S25>/Add6' */
    rtb_deltafalllimit_iz = rtb_UkYk1 -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE;

    /* Gain: '<S25>/Gain3' */
    rtb_deltafalllimit_iz *= 0.0;

    /* Gain: '<S25>/Gain' */
    rtb_Add2 = 20.0 * rtb_UkYk1;

    /* Sum: '<S25>/Add1' incorporates:
     *  UnitDelay: '<S25>/Unit Delay'
     */
    rtb_Add2 = (rtb_Add2 + rtb_deltafalllimit_iz) +
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p;

    /* RelationalOperator: '<S65>/LowerRelop1' incorporates:
     *  Constant: '<S25>/Constant5'
     */
    rtb_Compare_am = (rtb_Add2 > 0.0);

    /* Switch: '<S65>/Switch2' incorporates:
     *  Constant: '<S25>/Constant5'
     */
    if (rtb_Compare_am) {
      rtb_Add2 = 0.0;
    } else {
      /* Gain: '<S25>/Gain4' */
      rtb_Add6_p = -rtb_Divide;

      /* RelationalOperator: '<S65>/UpperRelop' */
      rtb_Compare_am = (rtb_Add2 < rtb_Add6_p);

      /* Switch: '<S65>/Switch' */
      if (rtb_Compare_am) {
        rtb_Add2 = rtb_Add6_p;
      }

      /* End of Switch: '<S65>/Switch' */
    }

    /* End of Switch: '<S65>/Switch2' */

    /* Sum: '<S25>/Add4' */
    rtb_deltafalllimit_iz = rtb_Divide + rtb_Add2;

    /* RelationalOperator: '<S64>/LowerRelop1' */
    rtb_Compare_am = (rtb_Divide > rtb_deltafalllimit_iz);

    /* Switch: '<S64>/Switch2' */
    if (rtb_Compare_am) {
      rtb_Divide = (real32_T)rtb_deltafalllimit_iz;
    } else {
      /* RelationalOperator: '<S64>/UpperRelop' incorporates:
       *  Constant: '<S25>/Constant3'
       */
      rtb_Compare_am = (rtb_Divide < 0.0F);

      /* Switch: '<S64>/Switch' incorporates:
       *  Constant: '<S25>/Constant3'
       */
      if (rtb_Compare_am) {
        rtb_Divide = 0.0F;
      }

      /* End of Switch: '<S64>/Switch' */
    }

    /* End of Switch: '<S64>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch3' */

  /* UnitDelay: '<S19>/Delay Input2'
   *
   * Block description for '<S19>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Saturation1_i = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn;

  /* Sum: '<S19>/Difference Inputs1'
   *
   * Block description for '<S19>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Divide -= rtb_Saturation1_i;

  /* SampleTimeMath: '<S19>/sample time'
   *
   * About '<S19>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S19>/delta rise limit' */
  rtb_Switch_jg = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S61>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Divide > rtb_Switch_jg);

  /* Switch: '<S61>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S19>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S61>/UpperRelop' */
    rtb_Compare_am = (rtb_Divide < rtb_Add6_p);

    /* Switch: '<S61>/Switch' */
    if (rtb_Compare_am) {
      rtb_Divide = rtb_Add6_p;
    }

    /* End of Switch: '<S61>/Switch' */
    rtb_Switch_jg = rtb_Divide;
  }

  /* End of Switch: '<S61>/Switch2' */

  /* Saturate: '<S7>/Saturation2' incorporates:
   *  Sum: '<S19>/Difference Inputs2'
   *  UnitDelay: '<S19>/Delay Input2'
   *
   * Block description for '<S19>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S19>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn = rtb_Switch_jg +
    rtb_Saturation1_i;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn > 20.0F) {
    TrqFR_cmd = 20.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn < 0.0F) {
    TrqFR_cmd = 0.0F;
  } else {
    TrqFR_cmd = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn;
  }

  /* End of Saturate: '<S7>/Saturation2' */

  /* UnitDelay: '<S39>/Delay Input2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *
   * Block description for '<S39>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_o;

  /* SampleTimeMath: '<S39>/sample time'
   *
   * About '<S39>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S39>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  rtb_Gain5 = 2000.0 * elapseTime;

  /* Gain: '<S10>/Gain14' */
  rtb_Gain4 = 0.017453292519943295 * rtb_Yk1_l;

  /* Trigonometry: '<S10>/Cos2' */
  rtb_Gain4 = cos(rtb_Gain4);

  /* Gain: '<S10>/Gain13' */
  rtb_Gain4 *= 1.2;

  /* Sum: '<S10>/Add8' incorporates:
   *  Constant: '<S10>/Constant28'
   */
  rtb_g_mpss1 = 90.0 - rtb_Yk1_l;

  /* Gain: '<S10>/Gain15' */
  rtb_g_mpss1 *= 0.017453292519943295;

  /* Trigonometry: '<S10>/Cos3' */
  rtb_g_mpss1 = cos(rtb_g_mpss1);

  /* Gain: '<S10>/Gain12' */
  rtb_g_mpss1 *= 1.522;

  /* Sum: '<S10>/Add9' */
  rtb_Gain4 += rtb_g_mpss1;

  /* Product: '<S10>/Product2' incorporates:
   *  UnitDelay: '<S41>/Delay Input2'
   *
   * Block description for '<S41>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain4 *= VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_jk;

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
  rtb_Gain4 = rtb_Acc_POS - rtb_Gain4;

  /* RelationalOperator: '<S46>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Gain4 > rtb_Add7);

  /* Switch: '<S46>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Gain4 = rtb_Add7;
  } else {
    /* RelationalOperator: '<S46>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    rtb_Compare_am = (rtb_Gain4 < 0.0);

    /* Switch: '<S46>/Switch' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    if (rtb_Compare_am) {
      rtb_Gain4 = 0.0;
    }

    /* End of Switch: '<S46>/Switch' */
  }

  /* End of Switch: '<S46>/Switch2' */

  /* Sum: '<S10>/Add12' */
  rtb_Yk1_l = rtb_Gain4 + rtb_Switch2_b0;

  /* RelationalOperator: '<S49>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Yk1_l > rtb_Add7);

  /* Switch: '<S49>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Yk1_l = rtb_Add7;
  } else {
    /* RelationalOperator: '<S49>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    rtb_Compare_am = (rtb_Yk1_l < 0.0);

    /* Switch: '<S49>/Switch' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    if (rtb_Compare_am) {
      rtb_Yk1_l = 0.0;
    }

    /* End of Switch: '<S49>/Switch' */
  }

  /* End of Switch: '<S49>/Switch2' */

  /* Sum: '<S39>/Difference Inputs1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *
   * Block description for '<S39>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Yk1_l -= VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d;

  /* RelationalOperator: '<S54>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Yk1_l > rtb_Gain5);

  /* Switch: '<S54>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S39>/delta fall limit' */
    rtb_deltafalllimit_iz = -2000.0 * elapseTime;

    /* RelationalOperator: '<S54>/UpperRelop' */
    rtb_Compare_am = (rtb_Yk1_l < rtb_deltafalllimit_iz);

    /* Switch: '<S54>/Switch' */
    if (rtb_Compare_am) {
      rtb_Yk1_l = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S54>/Switch' */
    rtb_Gain5 = rtb_Yk1_l;
  }

  /* End of Switch: '<S54>/Switch2' */

  /* Sum: '<S39>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S39>/Delay Input2'
   *
   * Block description for '<S39>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S39>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_o = rtb_Gain5 +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d;

  /* Product: '<S24>/Product3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = 7.0 * WhlSpdRR_mps;

  /* Product: '<S24>/Product2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  WhlSpdRR_mps = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d * 9550.0;

  /* Sum: '<S24>/Add3' incorporates:
   *  Constant: '<S24>/RPM_min4'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = MCFL_ActualVelocity +
    10.0;

  /* MinMax: '<S24>/Max2' incorporates:
   *  Constant: '<S24>/RPM_min5'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_Gain5 = fmax(VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d, 1.0);

  /* Product: '<S24>/Divide2' */
  WhlSpdRR_mps /= rtb_Gain5;

  /* MinMax: '<S7>/MinMax1' incorporates:
   *  UnitDelay: '<S39>/Delay Input2'
   *
   * Block description for '<S39>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_deltafalllimit_iz = fmin(WhlSpdRR_mps,
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_o);

  /* Switch: '<S27>/Switch6' incorporates:
   *  Constant: '<S27>/Verror_Reset'
   *  UnitDelay: '<S72>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp) {
    rtb_Switch2_b0 = rtb_Add10;
  } else {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Switch: '<S27>/Switch6' */

  /* Product: '<S27>/Product' incorporates:
   *  Constant: '<S27>/P_Gain'
   */
  rtb_Divide = rtb_Switch2_b0 * 40.0F;

  /* Sum: '<S27>/Add11' */
  WhlSpdRR_mps = rtb_deltafalllimit_iz - rtb_Divide;

  /* UnitDelay: '<S27>/Unit Delay5' */
  rtb_Saturation1_i = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_ip;

  /* Product: '<S27>/Product2' */
  rtb_Saturation1_i *= rtb_Add10;

  /* RelationalOperator: '<S69>/Compare' incorporates:
   *  Constant: '<S69>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Saturation1_i <= 0.0F);

  /* UnitDelay: '<S27>/Unit Delay' */
  rtb_Saturation1_i = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr;

  /* Switch: '<S27>/Switch3' incorporates:
   *  Constant: '<S27>/Verror_Reset1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Saturation1_i = 0.0F;
  }

  /* End of Switch: '<S27>/Switch3' */

  /* Sum: '<S27>/Add2' */
  rtb_Saturation1_i += rtb_Switch2_b0;

  /* Saturate: '<S27>/Saturation2' */
  if (rtb_Saturation1_i > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = 400.0F;
  } else if (rtb_Saturation1_i < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = rtb_Saturation1_i;
  }

  /* End of Saturate: '<S27>/Saturation2' */

  /* RelationalOperator: '<S77>/Compare' incorporates:
   *  UnitDelay: '<S72>/Unit Delay1'
   */
  rtb_Compare_am = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp;

  /* UnitDelay: '<S71>/Delay Input1'
   *
   * Block description for '<S71>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_b;

  /* RelationalOperator: '<S71>/FixPt Relational Operator' */
  rtb_UpperRelop_ir = ((int32_T)rtb_Compare_am > (int32_T)rtb_UpperRelop_ir);

  /* Switch: '<S27>/Switch' incorporates:
   *  Constant: '<S27>/Integr_StartPoint'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S27>/Add4' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = rtb_deltafalllimit_iz
      - VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g4;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = 0.0;
  }

  /* End of Switch: '<S27>/Switch' */

  /* Lookup_n-D: '<S27>/VehicleStableTarget_mps' */
  rtb_Switch2_b0 = look1_iflf_binlc(rtb_Add4_j,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S27>/Add5' */
  rtb_Switch2_b0 += rtb_Add4_j;

  /* Sum: '<S27>/Add10' */
  rtb_Switch2_b0 = rtb_VxIMU_est - rtb_Switch2_b0;

  /* RelationalOperator: '<S27>/Relational Operator' incorporates:
   *  Constant: '<S27>/Verror'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 < 0.0F);

  /* Logic: '<S27>/Logical Operator4' incorporates:
   *  UnitDelay: '<S72>/Unit Delay1'
   */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir &&
                       VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp);

  /* Switch: '<S27>/Switch1' incorporates:
   *  Constant: '<S27>/Trq_IReset'
   *  Constant: '<S27>/Trq_I_FF'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_b0 = 20.0F;
  } else {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Switch: '<S27>/Switch1' */

  /* Sum: '<S27>/Add6' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S27>/Unit Delay'
   */
  rtb_Gain5 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr +
               VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d) +
    rtb_Switch2_b0;

  /* Product: '<S27>/Product1' */
  rtb_Yk1_l = rtb_Gain5 * 10.0;

  /* RelationalOperator: '<S74>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Yk1_l > WhlSpdRR_mps);

  /* Switch: '<S74>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Gain: '<S27>/Gain3' */
    rtb_Acc_POS = -rtb_Divide;

    /* RelationalOperator: '<S74>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_Yk1_l < rtb_Acc_POS);

    /* Switch: '<S74>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_Yk1_l = rtb_Acc_POS;
    }

    /* End of Switch: '<S74>/Switch' */
    WhlSpdRR_mps = rtb_Yk1_l;
  }

  /* End of Switch: '<S74>/Switch2' */

  /* Sum: '<S27>/Add7' incorporates:
   *  UnitDelay: '<S27>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l = rtb_Divide +
    WhlSpdRR_mps;

  /* Lookup_n-D: '<S27>/VehicleStableTarget_mps1' */
  rtb_Switch2_b0 = look1_iflf_binlc(rtb_Add4_j,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S27>/Add13' */
  rtb_Add4_j += rtb_Switch2_b0;

  /* Sum: '<S27>/Add12' */
  rtb_VxIMU_est -= rtb_Add4_j;

  /* RelationalOperator: '<S27>/Relational Operator1' incorporates:
   *  Constant: '<S27>/Verror1'
   */
  rtb_UpperRelop_ir = (rtb_VxIMU_est < 0.0F);

  /* RelationalOperator: '<S27>/Relational Operator2' incorporates:
   *  UnitDelay: '<S27>/Unit Delay4'
   */
  rtb_AND2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l >=
              rtb_deltafalllimit_iz);

  /* RelationalOperator: '<S70>/Compare' incorporates:
   *  Constant: '<S70>/Constant'
   *  UnitDelay: '<S27>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l <=
                       0.01);

  /* Logic: '<S27>/OR' */
  rtb_AND2 = (rtb_AND2 || rtb_LowerRelop1_b);

  /* Logic: '<S27>/Logical Operator5' */
  rtb_LowerRelop1_b = (rtb_UpperRelop_ir && rtb_AND2);

  /* Switch: '<S27>/Switch2' incorporates:
   *  Switch: '<S27>/Switch7'
   *  UnitDelay: '<S72>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp) {
    /* RelationalOperator: '<S75>/LowerRelop1' incorporates:
     *  Constant: '<S27>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S27>/Unit Delay4'
     */
    rtb_AND2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l > 235.0);

    /* Switch: '<S75>/Switch2' incorporates:
     *  Constant: '<S27>/TCS_TrqRequest_Max2'
     */
    if (rtb_AND2) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 235.0;
    } else {
      /* RelationalOperator: '<S75>/UpperRelop' incorporates:
       *  Constant: '<S27>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S27>/Unit Delay4'
       */
      rtb_AND2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l < 0.0);

      /* Switch: '<S75>/Switch' incorporates:
       *  Constant: '<S27>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S27>/Unit Delay4'
       */
      if (rtb_AND2) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l;
      }

      /* End of Switch: '<S75>/Switch' */
    }

    /* End of Switch: '<S75>/Switch2' */

    /* RelationalOperator: '<S76>/LowerRelop1' */
    rtb_AND2 = (rtb_deltafalllimit_iz >
                VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b);

    /* Switch: '<S76>/Switch2' */
    if (!rtb_AND2) {
      /* RelationalOperator: '<S76>/UpperRelop' incorporates:
       *  Constant: '<S27>/TCS_TrqRequest_Min1'
       */
      rtb_AND2 = (rtb_deltafalllimit_iz < 0.0);

      /* Switch: '<S76>/Switch' incorporates:
       *  Constant: '<S27>/TCS_TrqRequest_Min1'
       */
      if (rtb_AND2) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b =
          rtb_deltafalllimit_iz;
      }

      /* End of Switch: '<S76>/Switch' */
    }

    /* End of Switch: '<S76>/Switch2' */
  } else {
    if (rtb_LowerRelop1_b) {
      /* Switch: '<S27>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b =
        rtb_deltafalllimit_iz;
    }
  }

  /* End of Switch: '<S27>/Switch2' */

  /* Sum: '<S7>/Add3' incorporates:
   *  Constant: '<S7>/Constant2'
   *  UnitDelay: '<S200>/Unit Delay1'
   */
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k + 5.0F;

  /* Sum: '<S26>/Add' */
  rtb_Switch2_b0 -= rtb_Add;

  /* RelationalOperator: '<S66>/Compare' incorporates:
   *  Constant: '<S66>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 < 0.0F);

  /* Switch: '<S26>/Switch' incorporates:
   *  Constant: '<S26>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Yk1_l = rtb_Switch2_b0;
  } else {
    rtb_Yk1_l = 0.0;
  }

  /* End of Switch: '<S26>/Switch' */

  /* Gain: '<S26>/Gain1' */
  WhlSpdRR_mps = 5.0 * rtb_Yk1_l;

  /* Logic: '<S26>/NOT' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* UnitDelay: '<S26>/Unit Delay' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_gb;

  /* Switch: '<S26>/Switch1' incorporates:
   *  Constant: '<S26>/Constant2'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (rtb_UpperRelop_ir) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = 0.0;
  }

  /* End of Switch: '<S26>/Switch1' */

  /* Sum: '<S26>/Add2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S26>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_gb = WhlSpdRR_mps +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d;

  /* Switch: '<S7>/Switch4' incorporates:
   *  Constant: '<S7>/Constant8'
   *  Switch: '<S7>/Switch7'
   */
  if (Trq_CUT_final) {
    rtb_Add = 0.0F;
  } else {
    if (VehCtrlMdel240926_2018b_amksp_B.TCSF_Enable_OUT != 0.0) {
      /* Switch: '<S7>/Switch7' incorporates:
       *  UnitDelay: '<S27>/Unit Delay2'
       */
      rtb_deltafalllimit_iz =
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b;
    }

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefFront2' */
    rtb_Add = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240926_2018b__ConstP.pooled34,
      VehCtrlMdel240926_2018b__ConstP.pooled33, 1U);

    /* RelationalOperator: '<S23>/LowerRelop1' */
    rtb_AND2 = (rtb_deltafalllimit_iz > rtb_Add);

    /* Switch: '<S23>/Switch2' */
    if (!rtb_AND2) {
      /* RelationalOperator: '<S23>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      rtb_AND2 = (rtb_deltafalllimit_iz < 0.0);

      /* Switch: '<S23>/Switch' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      if (rtb_AND2) {
        rtb_Add = 0.0F;
      } else {
        rtb_Add = (real32_T)rtb_deltafalllimit_iz;
      }

      /* End of Switch: '<S23>/Switch' */
    }

    /* End of Switch: '<S23>/Switch2' */

    /* Sum: '<S26>/Add6' */
    rtb_deltafalllimit_iz = rtb_Yk1_l -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_k;

    /* Gain: '<S26>/Gain3' */
    rtb_deltafalllimit_iz *= 0.0;

    /* Gain: '<S26>/Gain' */
    rtb_Add2 = 20.0 * rtb_Yk1_l;

    /* Sum: '<S26>/Add1' incorporates:
     *  UnitDelay: '<S26>/Unit Delay'
     */
    rtb_Add2 = (rtb_Add2 + rtb_deltafalllimit_iz) +
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_gb;

    /* RelationalOperator: '<S68>/LowerRelop1' incorporates:
     *  Constant: '<S26>/Constant5'
     */
    rtb_AND2 = (rtb_Add2 > 0.0);

    /* Switch: '<S68>/Switch2' incorporates:
     *  Constant: '<S26>/Constant5'
     */
    if (rtb_AND2) {
      rtb_Add2 = 0.0;
    } else {
      /* Gain: '<S26>/Gain4' */
      rtb_Acc_POS = -rtb_Add;

      /* RelationalOperator: '<S68>/UpperRelop' */
      rtb_AND2 = (rtb_Add2 < rtb_Acc_POS);

      /* Switch: '<S68>/Switch' */
      if (rtb_AND2) {
        rtb_Add2 = rtb_Acc_POS;
      }

      /* End of Switch: '<S68>/Switch' */
    }

    /* End of Switch: '<S68>/Switch2' */

    /* Sum: '<S26>/Add4' */
    rtb_deltafalllimit_iz = rtb_Add + rtb_Add2;

    /* RelationalOperator: '<S67>/LowerRelop1' */
    rtb_AND2 = (rtb_Add > rtb_deltafalllimit_iz);

    /* Switch: '<S67>/Switch2' */
    if (rtb_AND2) {
      rtb_Add = (real32_T)rtb_deltafalllimit_iz;
    } else {
      /* RelationalOperator: '<S67>/UpperRelop' incorporates:
       *  Constant: '<S26>/Constant3'
       */
      rtb_AND2 = (rtb_Add < 0.0F);

      /* Switch: '<S67>/Switch' incorporates:
       *  Constant: '<S26>/Constant3'
       */
      if (rtb_AND2) {
        rtb_Add = 0.0F;
      }

      /* End of Switch: '<S67>/Switch' */
    }

    /* End of Switch: '<S67>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch4' */

  /* UnitDelay: '<S20>/Delay Input2'
   *
   * Block description for '<S20>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib;

  /* Sum: '<S20>/Difference Inputs1'
   *
   * Block description for '<S20>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add -= rtb_Switch2_b0;

  /* SampleTimeMath: '<S20>/sample time'
   *
   * About '<S20>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S20>/delta rise limit' */
  rtb_Saturation1_i = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S62>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add > rtb_Saturation1_i);

  /* Switch: '<S62>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S20>/delta fall limit' */
    rtb_Acc_POS = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S62>/UpperRelop' */
    rtb_AND2 = (rtb_Add < rtb_Acc_POS);

    /* Switch: '<S62>/Switch' */
    if (rtb_AND2) {
      rtb_Add = rtb_Acc_POS;
    }

    /* End of Switch: '<S62>/Switch' */
    rtb_Saturation1_i = rtb_Add;
  }

  /* End of Switch: '<S62>/Switch2' */

  /* Saturate: '<S7>/Saturation3' incorporates:
   *  Sum: '<S20>/Difference Inputs2'
   *  UnitDelay: '<S20>/Delay Input2'
   *
   * Block description for '<S20>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S20>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib = rtb_Saturation1_i +
    rtb_Switch2_b0;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib > 20.0F) {
    TrqFL_cmd = 20.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib < 0.0F) {
    TrqFL_cmd = 0.0F;
  } else {
    TrqFL_cmd = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib;
  }

  /* End of Saturate: '<S7>/Saturation3' */

  /* Sum: '<S10>/Add3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_p = rtb_Switch2_on -
    rtb_UkYk1_ix;

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
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_n =
    VehCtrlMdel240926_2018b_amksp_B.DYC_Enable_OUT;

  /* Update for UnitDelay: '<S10>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_d = WhlSpdFL;

  /* Update for UnitDelay: '<S10>/Unit Delay6' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE_i =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_l;

  /* Update for UnitDelay: '<S29>/Unit Delay5' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_l = rtb_deltafalllimit_aw;

  /* Update for UnitDelay: '<S29>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g = rtb_deltafalllimit_i;

  /* Update for UnitDelay: '<S91>/Delay Input1'
   *
   * Block description for '<S91>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE = rtb_ignition;

  /* Update for UnitDelay: '<S27>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i = rtb_LowerRelop1_b;

  /* Update for UnitDelay: '<S28>/Unit Delay5' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = rtb_Switch2_mn;

  /* Update for UnitDelay: '<S28>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_f = rtb_Ax;

  /* Update for UnitDelay: '<S80>/Delay Input1'
   *
   * Block description for '<S80>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_j = rtb_LogicalOperator2;

  /* Update for UnitDelay: '<S25>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE = rtb_UkYk1;

  /* Update for UnitDelay: '<S27>/Unit Delay5' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_ip = rtb_Add10;

  /* Update for UnitDelay: '<S27>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g4 = rtb_Divide;

  /* Update for UnitDelay: '<S71>/Delay Input1'
   *
   * Block description for '<S71>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_b = rtb_Compare_am;

  /* Update for UnitDelay: '<S26>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_k = rtb_Yk1_l;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/10ms1' */

  /* S-Function (fcncallgen): '<S5>/10ms2' incorporates:
   *  SubSystem: '<S5>/VCU2AMKMCUFL'
   */
  /* Switch: '<S334>/Switch' incorporates:
   *  Constant: '<S334>/Constant1'
   *  Constant: '<S334>/Constant2'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueOn) {
    VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueLimitP = 10.0;
  } else {
    VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueLimitP = 0.0;
  }

  /* End of Switch: '<S334>/Switch' */

  /* S-Function (scanpack): '<S334>/CAN Pack1' incorporates:
   *  Constant: '<S334>/Constant1'
   */
  /* S-Function (scanpack): '<S334>/CAN Pack1' */
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.ID = 386U;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Length = 8U;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Extended = 0U;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Remote = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[0] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[1] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[2] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[3] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[4] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[5] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[6] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[7] = 0;

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
        real32_T result = TrqFL_cmd;

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[2] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[2] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[3] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[3] | (uint8_T)
              ((uint16_T)((uint16_T)(tempValue & (uint16_T)0xFF00U) >> 8));
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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[6] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[6] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[7] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[7] | (uint8_T)
              ((uint16_T)((uint16_T)(tempValue & (uint16_T)0xFF00U) >> 8));
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
        real64_T result = VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueLimitP;

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[4] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[4] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[5] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[5] | (uint8_T)
              ((uint16_T)((uint16_T)(tempValue & (uint16_T)0xFF00U) >> 8));
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
        uint32_T result = (uint32_T) (MCFL_DCOn_setpoints);

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[1] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[1] | (uint8_T)
              ((uint8_T)((uint8_T)(packedValue & (uint8_T)0x1U) << 1));
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
        uint32_T result = (uint32_T)
          (VehCtrlMdel240926_2018b_amksp_B.MCFL_DCEnable);

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[1] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[1] | (uint8_T)
              ((uint8_T)((uint8_T)(packedValue & (uint8_T)0x1U) << 2));
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
        real64_T result = VehCtrlMdel240926_2018b_amksp_B.errorReset;

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[1] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[1] | (uint8_T)
              ((uint8_T)((uint8_T)(packedValue & (uint8_T)0x1U) << 3));
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
        uint32_T result = (uint32_T)
          (VehCtrlMdel240926_2018b_amksp_B.MCFL_InverterOn);

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[1] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[1] | (uint8_T)
              ((uint8_T)(packedValue & (uint8_T)0x1U));
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

  /* S-Function (ecucoder_canmessage): '<S334>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[0]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[1]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[2]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[3]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[4]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[5]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[6]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[7]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S334>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN1BUF8TX[8];
    uint8 can1buf8looptx= 0;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[0];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[1];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[2];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[3];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[4];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[5];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[6];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_h[7];
    can1buf8looptx++;
    VehCtrlMdel240926_2018b_amksp_B.CANTransmit_c= ec_can_transmit(1, 8, 0, 386U,
      8, CAN1BUF8TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms2' */

  /* S-Function (fcncallgen): '<S5>/10ms4' incorporates:
   *  SubSystem: '<S5>/VCU2AMKMCUFR'
   */
  /* Switch: '<S335>/Switch' incorporates:
   *  Constant: '<S335>/Constant'
   *  Constant: '<S335>/Constant1'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.MCFR_TorqueOn) {
    VehCtrlMdel240926_2018b_amksp_B.MCFR_TorqueLimitP = 10.0;
  } else {
    VehCtrlMdel240926_2018b_amksp_B.MCFR_TorqueLimitP = 0.0;
  }

  /* End of Switch: '<S335>/Switch' */

  /* S-Function (scanpack): '<S335>/CAN Pack1' incorporates:
   *  Constant: '<S335>/Constant'
   */
  /* S-Function (scanpack): '<S335>/CAN Pack1' */
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.ID = 387U;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.Length = 8U;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.Extended = 0U;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.Remote = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[0] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[1] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[2] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[3] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[4] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[5] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[6] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[7] = 0;

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
        real32_T result = TrqFR_cmd;

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[2] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[2] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[3] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[3] | (uint8_T)
              ((uint16_T)((uint16_T)(tempValue & (uint16_T)0xFF00U) >> 8));
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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[6] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[6] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[7] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[7] | (uint8_T)
              ((uint16_T)((uint16_T)(tempValue & (uint16_T)0xFF00U) >> 8));
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
        real64_T result = VehCtrlMdel240926_2018b_amksp_B.MCFR_TorqueLimitP;

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[4] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[4] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[5] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[5] | (uint8_T)
              ((uint16_T)((uint16_T)(tempValue & (uint16_T)0xFF00U) >> 8));
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
        uint32_T result = (uint32_T) (MCFL_DCOn_setpoints);

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[1] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[1] | (uint8_T)
              ((uint8_T)((uint8_T)(packedValue & (uint8_T)0x1U) << 1));
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
        uint32_T result = (uint32_T)
          (VehCtrlMdel240926_2018b_amksp_B.MCFL_DCEnable);

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[1] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[1] | (uint8_T)
              ((uint8_T)((uint8_T)(packedValue & (uint8_T)0x1U) << 2));
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
        real64_T result = VehCtrlMdel240926_2018b_amksp_B.errorReset;

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[1] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[1] | (uint8_T)
              ((uint8_T)((uint8_T)(packedValue & (uint8_T)0x1U) << 3));
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
        uint32_T result = (uint32_T)
          (VehCtrlMdel240926_2018b_amksp_B.MCFL_InverterOn);

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[1] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[1] | (uint8_T)
              ((uint8_T)(packedValue & (uint8_T)0x1U));
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

  /* S-Function (ecucoder_canmessage): '<S335>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[0]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[1]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[2]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[3]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[4]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[5]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[6]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[7]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S335>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN1BUF9TX[8];
    uint8 can1buf9looptx= 0;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[0];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[1];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[2];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[3];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[4];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[5];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[6];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[7];
    can1buf9looptx++;
    VehCtrlMdel240926_2018b_amksp_B.CANTransmit_l= ec_can_transmit(1, 9, 0, 387U,
      8, CAN1BUF9TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms4' */

  /* S-Function (fcncallgen): '<S5>/10ms3' incorporates:
   *  SubSystem: '<S5>/VCU2EmraxMCU'
   */
  /* Switch: '<S336>/Switch2' incorporates:
   *  Constant: '<S336>/Constant13'
   *  Constant: '<S336>/Constant17'
   *  Constant: '<S336>/Constant19'
   *  Constant: '<S336>/Constant20'
   *  Switch: '<S336>/Switch3'
   */
  if (rtb_Compare) {
    Gear_Trs = 0.0;
    Mode_Trs = 0.0;
  } else {
    Gear_Trs = 2.0;
    Mode_Trs = 2.0;
  }

  /* End of Switch: '<S336>/Switch2' */

  /* DataTypeConversion: '<S336>/Cast To Boolean4' */
  VehCtrlMdel240926_2018b_amksp_B.CastToBoolean4 = (uint16_T)Gear_Trs;

  /* DataTypeConversion: '<S336>/Cast To Boolean6' */
  VehCtrlMdel240926_2018b_amksp_B.CastToBoolean6 = (uint16_T)Mode_Trs;

  /* DataTypeConversion: '<S336>/Data Type Conversion2' */
  VehCtrlMdel240926_2018b_amksp_B.DataTypeConversion2 = (int32_T)floor
    (rtb_Switch2_on);

  /* S-Function (scanpack): '<S336>/CAN Pack1' */
  /* S-Function (scanpack): '<S336>/CAN Pack1' */
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.ID = 146927393U;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Length = 8U;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Extended = 1U;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Remote = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[0] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[1] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[2] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[3] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[4] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[5] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[6] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[7] = 0;

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
        uint32_T result = (uint32_T)
          (VehCtrlMdel240926_2018b_amksp_B.CastToBoolean4);

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[4] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[4] | (uint8_T)
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
        uint32_T result = (uint32_T)
          (VehCtrlMdel240926_2018b_amksp_B.CastToBoolean6);

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[5] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[5] | (uint8_T)
              ((uint8_T)(packedValue & (uint8_T)0x3U));
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
        real32_T result = TrqR_cmd;

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[0] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[0] | (uint8_T)
              ((uint16_T)(packedValue & (uint16_T)0xFFU));
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[1] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[1] | (uint8_T)
              ((uint16_T)((uint16_T)(packedValue & (uint16_T)0xFF00U) >> 8));
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
          (VehCtrlMdel240926_2018b_amksp_B.DataTypeConversion2);

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[2] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[2] | (uint8_T)
              ((uint16_T)(packedValue & (uint16_T)0xFFU));
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[3] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[3] | (uint8_T)
              ((uint16_T)((uint16_T)(packedValue & (uint16_T)0xFF00U) >> 8));
          }
        }
      }
    }
  }

  /* S-Function (ecucoder_canmessage): '<S336>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[0]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[1]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[2]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[3]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[4]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[5]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[6]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[7]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S336>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN0BUF8TX[8];
    uint8 can0buf8looptx= 0;
    CAN0BUF8TX[can0buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[0];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[1];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[2];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[3];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[4];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[5];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[6];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_d[7];
    can0buf8looptx++;
    VehCtrlMdel240926_2018b_amksp_B.CANTransmit_k= ec_can_transmit(0, 8, 1,
      146927393U, 8, CAN0BUF8TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms3' */

  /* S-Function (fcncallgen): '<S5>/10ms6' incorporates:
   *  SubSystem: '<S5>/WP_OUTPUT'
   */
  /* DataTypeConversion: '<S337>/Cast To Single1' */
  VehCtrlMdel240926_2018b_amksp_B.CastToSingle1 = (uint16_T)rtb_CastToDouble;

  /* S-Function (ec5744_pdsslbu3): '<S337>/PowerDriverSwitch(HS)' */

  /* Set level VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON for the specified power driver switch */
  ec_gpio_write(50,VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON);
  ec_gpio_write(42,VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON);

  /* S-Function (ec5744_pdpslbu3): '<S337>/PowerDriverPWM' incorporates:
   *  Constant: '<S337>/Constant'
   */

  /* Power driver PWM output for channel 6 */
  ec_pwm_output(6,((uint16_T)1000U),
                VehCtrlMdel240926_2018b_amksp_B.CastToSingle1);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms6' */

  /* S-Function (fcncallgen): '<S342>/10ms' incorporates:
   *  SubSystem: '<S342>/daq10ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S354>/CCPDAQ' */
  ccpDaq(1);

  /* End of Outputs for S-Function (fcncallgen): '<S342>/10ms' */

  /* Update absolute time */
  /* The "clockTick3" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick3" ensures timer will not overflow during the
   * application lifespan selected.
   */
  VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3++;
}

/* Model step function for TID4 */
void VehCtrlMdel240926_2018b_amkspdlimit_step4(void) /* Sample time: [0.05s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S342>/50ms' incorporates:
   *  SubSystem: '<S342>/daq50ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S356>/CCPDAQ' */
  ccpDaq(2);

  /* End of Outputs for S-Function (fcncallgen): '<S342>/50ms' */
}

/* Model step function for TID5 */
void VehCtrlMdel240926_2018b_amkspdlimit_step5(void) /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S341>/100MS' incorporates:
   *  SubSystem: '<S341>/Function-Call Subsystem'
   */
  /* S-Function (ec5744_canreceiveslb): '<S345>/CANReceive' */

  /* Receive CAN message */
  {
    uint8 CAN2BUF1RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can2buf1looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive_o3_l= 278;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive_o5_l= 8;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive_o2_p= ec_can_receive(2,1,
      CAN2BUF1RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[0]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[1]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[2]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[3]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[4]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[5]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[6]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[7]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
  }

  /* Call the system: <S345>/Function-Call Subsystem */

  /* Output and update for function-call system: '<S345>/Function-Call Subsystem' */
  {
    uint8_T rtb_Add;
    uint8_T rtb_Compare;

    /* Outputs for Enabled SubSystem: '<S346>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S347>/Enable'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.CANReceive_o2_p > 0) {
      /* RelationalOperator: '<S348>/Compare' incorporates:
       *  Constant: '<S348>/Constant'
       */
      rtb_Add = (uint8_T)(VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[0] ==
                          83);

      /* RelationalOperator: '<S349>/Compare' incorporates:
       *  Constant: '<S349>/Constant'
       */
      rtb_Compare = (uint8_T)(VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[5]
        == 84);

      /* Sum: '<S347>/Add' */
      rtb_Add = (uint8_T)((uint32_T)rtb_Add + rtb_Compare);

      /* RelationalOperator: '<S350>/Compare' incorporates:
       *  Constant: '<S350>/Constant'
       */
      rtb_Compare = (uint8_T)(rtb_Add == 2);

      /* If: '<S347>/If' */
      if (rtb_Compare > 0) {
        /* Outputs for IfAction SubSystem: '<S347>/If Action Subsystem' incorporates:
         *  ActionPort: '<S351>/Action Port'
         */
        /* S-Function (ec5744_bootloaderslb): '<S351>/BootLoader' */
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

        /* S-Function (ec5744_cpuresetslb): '<S351>/CPUReset' */

        /* Perform a microcontroller reset */
        MC_ME.MCTL.R = 0X00005AF0;
        MC_ME.MCTL.R = 0X0000A50F;

        /* End of Outputs for SubSystem: '<S347>/If Action Subsystem' */
      } else {
        /* Outputs for IfAction SubSystem: '<S347>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S352>/Action Port'
         */
        /* S-Function (ec5744_cantransmitslb): '<S352>/CANTransmit' incorporates:
         *  Constant: '<S352>/Constant'
         */

        /*Transmit CAN message*/
        {
          uint8 CAN2BUF9TX[1];
          uint8 can2buf9looptx= 0;
          CAN2BUF9TX[can2buf9looptx]= ((uint8_T)1U);
          can2buf9looptx++;
          VehCtrlMdel240926_2018b_amksp_B.CANTransmit= ec_can_transmit(2, 9, 0,
            593U, 1, CAN2BUF9TX);
        }

        /* End of Outputs for SubSystem: '<S347>/If Action Subsystem1' */
      }

      /* End of If: '<S347>/If' */
    }

    /* End of Outputs for SubSystem: '<S346>/Enabled Subsystem' */
  }

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S345>/CANReceive' */
  /* End of Outputs for S-Function (fcncallgen): '<S341>/100MS' */

  /* S-Function (fcncallgen): '<S342>/100ms' incorporates:
   *  SubSystem: '<S342>/daq100ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S353>/CCPDAQ' */
  ccpDaq(3);

  /* End of Outputs for S-Function (fcncallgen): '<S342>/100ms' */
}

/* Model step function for TID6 */
void VehCtrlMdel240926_2018b_amkspdlimit_step6(void) /* Sample time: [0.5s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S342>/500ms' incorporates:
   *  SubSystem: '<S342>/daq500ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S355>/CCPDAQ' */
  ccpDaq(4);

  /* End of Outputs for S-Function (fcncallgen): '<S342>/500ms' */

  /* S-Function (fcncallgen): '<S343>/500ms' incorporates:
   *  SubSystem: '<S343>/EEPROMOperation'
   */

  /* S-Function (ec5744_eepromoslb): '<S358>/EEPROMOperatin' */
#if defined EC_EEPROM_ENABLE

  /* Operate the EEPROM module on the MPC5744 */
  ec_flash_operation();

#endif

  /* End of Outputs for S-Function (fcncallgen): '<S343>/500ms' */
}

/* Model step wrapper function for compatibility with a static main program */
void VehCtrlMdel240926_2018b_amkspdlimit_step(int_T tid)
{
  switch (tid) {
   case 0 :
    VehCtrlMdel240926_2018b_amkspdlimit_step0();
    break;

   case 1 :
    VehCtrlMdel240926_2018b_amkspdlimit_step1();
    break;

   case 2 :
    VehCtrlMdel240926_2018b_amkspdlimit_step2();
    break;

   case 3 :
    VehCtrlMdel240926_2018b_amkspdlimit_step3();
    break;

   case 4 :
    VehCtrlMdel240926_2018b_amkspdlimit_step4();
    break;

   case 5 :
    VehCtrlMdel240926_2018b_amkspdlimit_step5();
    break;

   case 6 :
    VehCtrlMdel240926_2018b_amkspdlimit_step6();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void VehCtrlMdel240926_2018b_amkspdlimit_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* Start for S-Function (fcncallgen): '<S3>/10ms6' incorporates:
   *  SubSystem: '<S3>/EMRAXMCU_RECIEVE'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S119>/CANReceive1' incorporates:
   *  SubSystem: '<S119>/MCU_pwr'
   */
  /* Start for function-call system: '<S119>/MCU_pwr' */

  /* Start for Enabled SubSystem: '<S172>/MCU_VCUMeter1' */

  /* Start for S-Function (scanunpack): '<S174>/CAN Unpack' */

  /*-----------S-Function Block: <S174>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S172>/MCU_VCUMeter1' */
  ec_buffer_init(0,1,1,218089455);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S119>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S119>/CANReceive3' incorporates:
   *  SubSystem: '<S119>/MCU_state'
   */
  /* Start for function-call system: '<S119>/MCU_state' */

  /* Start for Enabled SubSystem: '<S173>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S179>/CAN Unpack' */

  /*-----------S-Function Block: <S179>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S173>/MCU_state' */
  ec_buffer_init(0,0,1,218089199);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S119>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S115>/CANReceive3' incorporates:
   *  SubSystem: '<S115>/ABS_BUS_state'
   */
  /* Start for function-call system: '<S115>/ABS_BUS_state' */

  /* Start for Enabled SubSystem: '<S123>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S124>/CAN Unpack1' */

  /*-----------S-Function Block: <S124>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S123>/IMU_state' */
  ec_buffer_init(1,16,0,1698);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S115>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' incorporates:
   *  SubSystem: '<S121>/StrWhSnis_state'
   */
  /* Start for function-call system: '<S121>/StrWhSnis_state' */

  /* Start for Enabled SubSystem: '<S191>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S192>/CAN Unpack1' */

  /*-----------S-Function Block: <S192>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S191>/IMU_state' */
  ec_buffer_init(1,7,0,330);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms4' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S129>/CANReceive3' incorporates:
   *  SubSystem: '<S129>/AMKMCU_state'
   */
  /* Start for function-call system: '<S129>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S131>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S134>/CAN Unpack' */

  /*-----------S-Function Block: <S134>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S131>/MCU_state' */
  ec_buffer_init(1,1,0,640);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S129>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S129>/CANReceive1' incorporates:
   *  SubSystem: '<S129>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S129>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S132>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S143>/CAN Unpack' */

  /*-----------S-Function Block: <S143>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S132>/MCU_state' */
  ec_buffer_init(1,2,0,642);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S129>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S129>/CANReceive2' incorporates:
   *  SubSystem: '<S129>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S129>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S133>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S145>/CAN Unpack' */

  /*-----------S-Function Block: <S145>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S133>/MCU_state' */
  ec_buffer_init(1,3,0,644);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S129>/CANReceive2' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive3' incorporates:
   *  SubSystem: '<S130>/AMKMCU_state'
   */
  /* Start for function-call system: '<S130>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S149>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S152>/CAN Unpack' */

  /*-----------S-Function Block: <S152>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S149>/MCU_state' */
  ec_buffer_init(1,4,0,641);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive1' incorporates:
   *  SubSystem: '<S130>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S130>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S150>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S160>/CAN Unpack' */

  /*-----------S-Function Block: <S160>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S150>/MCU_state' */
  ec_buffer_init(1,5,0,643);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive2' incorporates:
   *  SubSystem: '<S130>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S130>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S151>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S162>/CAN Unpack' */

  /*-----------S-Function Block: <S162>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S151>/MCU_state' */
  ec_buffer_init(1,0,0,645);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive2' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms5' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S120>/CANReceive3' incorporates:
   *  SubSystem: '<S120>/IMU_state'
   */
  /* Start for function-call system: '<S120>/IMU_state' */

  /* Start for Enabled SubSystem: '<S186>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S187>/CAN Unpack' */

  /*-----------S-Function Block: <S187>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S186>/MCU_state' */
  ec_buffer_init(1,17,0,513);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S120>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms2' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S118>/CANReceive3' */
  ec_buffer_init(0,3,1,408961267);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S118>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S334>/CANTransmit' */
  ec_buffer_init(1,8,0,386U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms2' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S335>/CANTransmit' */
  ec_buffer_init(1,9,0,387U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms4' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S336>/CANTransmit' */
  ec_buffer_init(0,8,1,146927393U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S5>/10ms6' incorporates:
   *  SubSystem: '<S5>/WP_OUTPUT'
   */
  /* Start for S-Function (ec5744_pdpslbu3): '<S337>/PowerDriverPWM' incorporates:
   *  Constant: '<S337>/Constant'
   */

  /* Initialize PWM output for channel 6 */
  SIUL2_MSCR42 = 0X02000003;           //PWM_A3

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S341>/100MS' incorporates:
   *  SubSystem: '<S341>/Function-Call Subsystem'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S345>/CANReceive' incorporates:
   *  SubSystem: '<S345>/Function-Call Subsystem'
   */
  /* Start for function-call system: '<S345>/Function-Call Subsystem' */

  /* Start for Enabled SubSystem: '<S346>/Enabled Subsystem' */
  /* Start for IfAction SubSystem: '<S347>/If Action Subsystem1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S352>/CANTransmit' incorporates:
   *  Constant: '<S352>/Constant'
   */
  ec_buffer_init(2,9,0,593U);

  /* End of Start for SubSystem: '<S347>/If Action Subsystem1' */
  /* End of Start for SubSystem: '<S346>/Enabled Subsystem' */
  ec_buffer_init(2,1,0,278);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S345>/CANReceive' */
  /* End of Start for S-Function (fcncallgen): '<S341>/100MS' */

  /* Start for S-Function (fcncallgen): '<S344>/Function-Call Generator' incorporates:
   *  SubSystem: '<S344>/CCPBackground'
   */
  /* Start for S-Function (ec5744_ccpslb): '<S359>/CCPBackground' */
  ccpInit();

  /* End of Start for S-Function (fcncallgen): '<S344>/Function-Call Generator' */

  /* Start for S-Function (ec5744_caninterruptslb1): '<S344>/ReceiveandTransmitInterrupt' incorporates:
   *  SubSystem: '<S344>/CCPReceive'
   */
  /* Start for function-call system: '<S344>/CCPReceive' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S360>/CANReceive' */
  ec_buffer_init(2,0,0,CCP_CRO_ID);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S360>/CANReceive' */
  ec_bufint_init(2,0);
  INTC_0.PSR[548].B.PRIN = 12;
  IntcIsrVectorTable[548] = (uint32_t)&ISR_FlexCAN_2_MB0;

  /* End of Start for S-Function (ec5744_caninterruptslb1): '<S344>/ReceiveandTransmitInterrupt' */

  /* Start for S-Function (ec5744_eeprombsbu3): '<Root>/EEPROMEnable' */
  Fls_Read(0xFA0010,ecflashdataold,4096);
  Fls_Read(0xFA0010,ecflashdatanew,4096);

  /* SystemInitialize for S-Function (fcncallgen): '<S3>/10ms7' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* SystemInitialize for SignalConversion generated from: '<S122>/Constant' */
  HVCUTOFF = true;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S3>/10ms7' */

  /* SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  /* InitializeConditions for UnitDelay: '<S263>/Unit Delay4' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn = 0.01F;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' */

  /* SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* SystemInitialize for Chart: '<S105>/Chart2' */
  VehCtrlMdel240926_2018b_amks_DW.sfEvent = -1;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S4>/10ms' incorporates:
   *  SubSystem: '<S4>/Function-Call Subsystem'
   */
  VehCtrlMdel240926_2018b_amks_DW.FunctionCallSubsystem_RESET_ELA = true;

  /* End of Enable for S-Function (fcncallgen): '<S4>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  VehCtrlMdel240926_2018b_amks_DW.Subsystem_RESET_ELAPS_T = true;

  /* End of Enable for S-Function (fcncallgen): '<S4>/10ms1' */

  /* Enable for S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* Enable for Chart: '<S105>/Chart2' */
  VehCtrlMdel240926_2018b_amks_DW.previousTicks =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;

  /* End of Enable for S-Function (fcncallgen): '<S2>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S1>/10ms1' incorporates:
   *  SubSystem: '<S1>/MoTrqReq'
   */
  VehCtrlMdel240926_2018b_amks_DW.MoTrqReq_RESET_ELAPS_T = true;

  /* End of Enable for S-Function (fcncallgen): '<S1>/10ms1' */
}

/* File trailer for ECUCoder generated file VehCtrlMdel240926_2018b_amkspdlimit.c.
 *
 * [EOF]
 */
