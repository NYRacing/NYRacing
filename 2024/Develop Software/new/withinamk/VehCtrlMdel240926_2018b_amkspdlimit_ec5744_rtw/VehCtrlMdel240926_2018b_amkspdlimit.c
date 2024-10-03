/*
 * Code generated for Simulink model VehCtrlMdel240926_2018b_amkspdlimit.
 *
 * FILE    : VehCtrlMdel240926_2018b_amkspdlimit.c
 *
 * VERSION : 1.212
 *
 * DATE    : Wed Oct  2 16:46:03 2024
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

/* Named constants for Chart: '<S118>/Timer' */
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

/* Named constants for Chart: '<S101>/Chart2' */
#define VehCtr_IN_MCFL_InverterOn_State ((uint8_T)6U)
#define VehCtr_IN_MCFR_InverterOn_State ((uint8_T)7U)
#define VehCtrlMdel240926_2018_IN_Guard ((uint8_T)1U)
#define VehCtrlMdel240926_2018_IN_Ready ((uint8_T)5U)
#define VehCtrlMdel240926_2018_IN_Trans ((uint8_T)7U)
#define VehCtrlMdel240926_2018_IN_start ((uint8_T)10U)
#define VehCtrlMdel240926_2018b_IN_Init ((uint8_T)2U)
#define VehCtrlMdel240926_2018b__IN_OFF ((uint8_T)1U)
#define VehCtrlMdel240926_2018b_a_IN_ON ((uint8_T)2U)
#define VehCtrlMdel240926_201_IN_AMKCAN ((uint8_T)1U)
#define VehCtrlMdel240926_20_IN_Standby ((uint8_T)6U)
#define VehCtrlMdel240926_IN_MC_DCready ((uint8_T)8U)
#define VehCtrlMdel240926_IN_SYSRDYCECK ((uint8_T)9U)
#define VehCtrlMdel240926_event_AMKDCON (3)
#define VehCtrlMdel240926_event_EbeepON (5)
#define VehCtrlMdel24092_IN_MCDCOncheck ((uint8_T)5U)
#define VehCtrlMdel24092_event_AMKCANON (1)
#define VehCtrlMdel24092_event_AMKDCOFF (2)
#define VehCtrlMdel24092_event_EbeepOFF (4)
#define VehCtrlMdel24092_event_TorqueON (15)
#define VehCtrlMdel2409_IN_MCUReadyFail ((uint8_T)4U)
#define VehCtrlMdel2409_event_AMKCANOFF (0)
#define VehCtrlMdel2409_event_TorqueOFF (14)
#define VehCtrlMdel240_IN_AMKDCOnFinish ((uint8_T)2U)
#define VehCtrlMdel240_IN_DCOnCheckPass ((uint8_T)3U)
#define VehCtrlMdel240_IN_InitStateBack ((uint8_T)3U)
#define VehCtrlMdel240_IN_WaitForEngine ((uint8_T)8U)
#define VehCtrlMdel2_IN_MCDCEnableState ((uint8_T)4U)
#define VehCtrlMdel2_event_InverterFLON (7)
#define VehCtrlMdel2_event_InverterFRON (9)
#define VehCtrlMdel2_event_MCDCEnableON (13)
#define VehCtrlMdel_event_InverterFLOFF (6)
#define VehCtrlMdel_event_InverterFROFF (8)
#define VehCtrlMdel_event_MCDCEnableOFF (12)

/* Named constants for Chart: '<S118>/Timer1' */
#define VehCtrlMdel240926_2018_IN_Out_b ((uint8_T)2U)
#define VehCtrlMdel240926__IN_Trigger_e ((uint8_T)3U)
#define VehCtrlMdel2409_IN_InterState_d ((uint8_T)1U)

EE_Data ecflashdataold[1024];
EE_Data ecflashdatanew[1024];

/* Exported block signals */
real_T Gear_Trs;                       /* '<S341>/Switch2' */
real_T Mode_Trs;                       /* '<S341>/Switch3' */
real_T AMKFL_Current;                  /* '<S205>/Switch' */
real_T AMKFR_Current;                  /* '<S205>/Switch1' */
real_T Trq_CUT;                        /* '<S202>/Timer' */
real_T AMKSWITCH;                      /* '<S118>/Timer1' */
real_T ignition;                       /* '<S118>/Timer' */
real_T L12V_error;                     /* '<S178>/CAN Unpack' */
real_T alarm;                          /* '<S178>/CAN Unpack' */
real_T controller_ready;               /* '<S178>/CAN Unpack' */
real_T selfcheck;                      /* '<S178>/CAN Unpack' */
real_T RPM;                            /* '<S178>/CAN Unpack' */
real_T trq;                            /* '<S178>/CAN Unpack' */
real_T AC_current;                     /* '<S172>/CAN Unpack' */
real_T DC_current;                     /* '<S172>/CAN Unpack' */
real_T MCU_Temp;                       /* '<S172>/CAN Unpack' */
real_T motor_Temp;                     /* '<S172>/CAN Unpack' */
real_T voltage;                        /* '<S172>/CAN Unpack' */
real_T MCFR_ActualTorque;              /* '<S149>/CAN Unpack' */
real_T MCFR_ActualVelocity;            /* '<S149>/CAN Unpack' */
real_T MCFR_DCVoltage;                 /* '<S149>/CAN Unpack' */
real_T MCFR_bDCOn;                     /* '<S149>/CAN Unpack' */
real_T MCFR_bError;                    /* '<S149>/CAN Unpack' */
real_T MCFR_bInverterOn;               /* '<S149>/CAN Unpack' */
real_T MCFR_bQuitInverterOn;           /* '<S149>/CAN Unpack' */
real_T MCFR_bSystemReady;              /* '<S149>/CAN Unpack' */
real_T MCFR_TempIGBT;                  /* '<S160>/CAN Unpack' */
real_T MCFR_TempInverter;              /* '<S160>/CAN Unpack' */
real_T MCFR_TempMotor;                 /* '<S160>/CAN Unpack' */
real_T MCFR_ErrorInfo;                 /* '<S158>/CAN Unpack' */
real_T MCFL_ActualTorque;              /* '<S130>/CAN Unpack' */
real_T MCFL_DCVoltage;                 /* '<S130>/CAN Unpack' */
real_T MCFL_bDCOn;                     /* '<S130>/CAN Unpack' */
real_T MCFL_bError;                    /* '<S130>/CAN Unpack' */
real_T MCFL_bInverterOn;               /* '<S130>/CAN Unpack' */
real_T MCFL_bQuitDCOn;                 /* '<S130>/CAN Unpack' */
real_T MCFL_bQuitInverterOn;           /* '<S130>/CAN Unpack' */
real_T MCFL_bSystemReady;              /* '<S130>/CAN Unpack' */
real_T MCFL_ActualVelocity;            /* '<S130>/Gain' */
real_T MCFL_TempIGBT;                  /* '<S142>/CAN Unpack' */
real_T MCFL_TempInverter;              /* '<S142>/CAN Unpack' */
real_T MCFL_TempMotor;                 /* '<S142>/CAN Unpack' */
real_T MCFL_ErrorInfo;                 /* '<S140>/CAN Unpack' */
real_T StrWhlAngAliveRollCnt;          /* '<S191>/CAN Unpack1' */
real_T StrWhlAng;                      /* '<S191>/CAN Unpack1' */
real_T StrWhlAngV;                     /* '<S191>/CAN Unpack1' */
real_T ABS_WS_FL;                      /* '<S120>/CAN Unpack1' */
real_T ABS_WS_FR;                      /* '<S120>/CAN Unpack1' */
real_T ABS_WS_RL;                      /* '<S120>/CAN Unpack1' */
real_T ABS_WS_RR;                      /* '<S120>/CAN Unpack1' */
real_T IMU_Ay_Value;                   /* '<S186>/CAN Unpack' */
real_T IMU_Ax_Value;                   /* '<S186>/CAN Unpack' */
real_T IMU_Yaw_Value;                  /* '<S186>/CAN Unpack' */
real_T EMRAX_Trq_CUT;                  /*  */
real_T AMK_Trq_CUT;                    /*  */
uint32_T Acc_vol2;                     /* '<S202>/Add3' */
uint32_T Acc_vol;                      /* '<S202>/Add2' */
uint32_T Acc_POS2;                     /* '<S202>/1-D Lookup Table3' */
real32_T VehVxEst_mps;                 /* '<S325>/Add' */
real32_T Acc_POS;                      /* '<S202>/MATLAB Function' */
real32_T EmraxTrqR_cmd;                /* '<S7>/Saturation1' */
real32_T AMKTrqFR_cmd;                 /* '<S7>/Saturation2' */
real32_T AMKTrqFL_cmd;                 /* '<S7>/Saturation3' */
uint16_T F_BrkPrs;                     /* '<S202>/1-D Lookup Table1' */
uint16_T Acc1;                         /* '<S113>/Acc3' */
uint16_T Acc2;                         /* '<S113>/Acc4' */
uint16_T Brk1;                         /* '<S113>/Brk1' */
uint16_T Brk2;                         /* '<S113>/Brk2' */
boolean_T HVCUTOFF;                    /* '<S118>/Constant' */
boolean_T KeyPressed;                  /* '<S101>/Cast To Boolean' */
boolean_T Brk;                         /* '<S103>/Compare' */
boolean_T ACC_Release;                 /* '<S104>/Compare' */
boolean_T beeper_state;                /* '<S101>/Chart2' */
boolean_T MCFL_DCOn_setpoints;         /* '<S101>/Chart2' */
boolean_T MCFR_DCEnable;               /* '<S101>/Chart2' */
boolean_T MCFR_InverterOn;             /* '<S101>/Chart2' */
boolean_T TrqR_cmd_raw;                /* '<S7>/Logical Operator1' */
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
static void VehCtrlMdel240926_2018b_VehStat(const real_T *controller_ready_e,
  const real_T *Switch_k, const real_T *Switch3, const real_T *Switch10, const
  real_T *Switch11);
static void VehCtrlMdel240926_20_AMKDCready(const real_T *MCFL_bDCOn_j, const
  real_T *MCFR_bDCOn_n, const real_T *Switch_k, const real_T *Switch3, const
  real_T *Switch10, const real_T *Switch11);
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
  /* Call the system: <S350>/CCPReceive */
  {
    /* S-Function (ec5744_caninterruptslb1): '<S350>/ReceiveandTransmitInterrupt' */

    /* Output and update for function-call system: '<S350>/CCPReceive' */

    /* S-Function (ec5744_canreceiveslb): '<S366>/CANReceive' */

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

    /* Nothing to do for system: <S366>/Nothing */

    /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S366>/CANReceive' */

    /* End of Outputs for S-Function (ec5744_caninterruptslb1): '<S350>/ReceiveandTransmitInterrupt' */
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
 *    '<S204>/Timer'
 *    '<S206>/Timer'
 *    '<S206>/Timer1'
 *    '<S206>/Timer2'
 *    '<S206>/Timer3'
 *    '<S267>/Timer'
 *    '<S267>/Timer1'
 *    '<S267>/Timer2'
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
 *    '<S118>/Timer'
 *    '<S202>/Timer'
 */
void VehCtrlMdel240926_201_Timer(boolean_T rtu_Trigger, real32_T rtu_CountTime,
  real_T *rty_Exit, DW_Timer_VehCtrlMdel240926_20_T *localDW)
{
  boolean_T sf_internal_predicateOutput;

  /* Chart: '<S118>/Timer' */
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

  /* End of Chart: '<S118>/Timer' */
}

/* Function for Chart: '<S101>/Chart2' */
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

  VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel_event_InverterFLOFF;
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MCFL_InverterOn !=
      0U) {
    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn) {
     case VehCtrlMdel240926_2018b__IN_OFF:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel2_event_InverterFLON) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn = 2U;
      }
      break;

     case VehCtrlMdel240926_2018b_a_IN_ON:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel_event_InverterFLOFF) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel_event_InverterFROFF;
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MCFR_InverterOn !=
      0U) {
    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn) {
     case VehCtrlMdel240926_2018b__IN_OFF:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel2_event_InverterFRON) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn = 2U;
      }
      break;

     case VehCtrlMdel240926_2018b_a_IN_ON:
      if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
          VehCtrlMdel_event_InverterFROFF) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn = 1U;
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

/* Function for Chart: '<S101>/Chart2' */
static void VehCtrlMdel240926_2018b_VehStat(const real_T *controller_ready_e,
  const real_T *Switch_k, const real_T *Switch3, const real_T *Switch10, const
  real_T *Switch11)
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
          (!(*Switch_k != 0.0)) || (!(*Switch3 != 0.0)));
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
      (!(*Switch_k != 0.0)) || (!(*Switch3 != 0.0)));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 4U;
    }
    break;

   case VehCtrlMdel240926_20_IN_Standby:
    sf_internal_predicateOutput = ((!(*Switch_k != 0.0)) || (!(*Switch3 != 0.0))
      || (!(*controller_ready_e != 0.0)));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 8U;
      VehC_enter_atomic_WaitForEngine();
    } else {
      sf_internal_predicateOutput = (KeyPressed && Brk && ACC_Release &&
        (*controller_ready_e != 0.0) && (*Switch_k != 0.0) && (*Switch3 != 0.0) &&
        (*Switch10 != 0.0) && (*Switch11 != 0.0));
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
    sf_internal_predicateOutput = ((*Switch_k != 0.0) && (*Switch3 != 0.0) &&
      (*controller_ready_e != 0.0));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 6U;
    }
    break;
  }
}

/* Function for Chart: '<S101>/Chart2' */
static void VehCtrlMdel240926_20_AMKDCready(const real_T *MCFL_bDCOn_j, const
  real_T *MCFR_bDCOn_n, const real_T *Switch_k, const real_T *Switch3, const
  real_T *Switch10, const real_T *Switch11)
{
  boolean_T sf_internal_predicateOutput;
  int32_T g_previousEvent;
  switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready) {
   case VehCtrlMdel240926_201_IN_AMKCAN:
    sf_internal_predicateOutput = (KeyPressed && Brk && ACC_Release);
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 8U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
    }
    break;

   case VehCtrlMdel240_IN_AMKDCOnFinish:
    break;

   case VehCtrlMdel240_IN_DCOnCheckPass:
    sf_internal_predicateOutput =
      ((VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 100U) &&
       (*MCFR_bDCOn_n != 0.0) && (*MCFL_bDCOn_j != 0.0));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 4U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
      g_previousEvent = VehCtrlMdel240926_2018b_amks_DW.sfEvent;
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

      VehCtrlMdel240926_2018b_amks_DW.sfEvent = g_previousEvent;
    }
    break;

   case VehCtrlMdel2_IN_MCDCEnableState:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 100U) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 6U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
      g_previousEvent = VehCtrlMdel240926_2018b_amks_DW.sfEvent;
      VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel2_event_InverterFLON;
      if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MCFL_InverterOn
          != 0U) {
        switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn)
    {
         case VehCtrlMdel240926_2018b__IN_OFF:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel2_event_InverterFLON) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn = 2U;
          }
          break;

         case VehCtrlMdel240926_2018b_a_IN_ON:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel_event_InverterFLOFF) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240926_2018b_amks_DW.sfEvent = g_previousEvent;
    }
    break;

   case VehCtrlMdel24092_IN_MCDCOncheck:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 100U) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 3U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
    }
    break;

   case VehCtr_IN_MCFL_InverterOn_State:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 500U) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 7U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
      g_previousEvent = VehCtrlMdel240926_2018b_amks_DW.sfEvent;
      VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel2_event_InverterFRON;
      if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MCFR_InverterOn
          != 0U) {
        switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn)
    {
         case VehCtrlMdel240926_2018b__IN_OFF:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel2_event_InverterFRON) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn = 2U;
          }
          break;

         case VehCtrlMdel240926_2018b_a_IN_ON:
          if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
              VehCtrlMdel_event_InverterFROFF) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240926_2018b_amks_DW.sfEvent = g_previousEvent;
    }
    break;

   case VehCtr_IN_MCFR_InverterOn_State:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 100U) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 9U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
    }
    break;

   case VehCtrlMdel240926_IN_MC_DCready:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 50U) {
      g_previousEvent = VehCtrlMdel240926_2018b_amks_DW.sfEvent;
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

      VehCtrlMdel240926_2018b_amks_DW.sfEvent = g_previousEvent;
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 5U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
    }
    break;

   case VehCtrlMdel240926_IN_SYSRDYCECK:
    sf_internal_predicateOutput =
      ((VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 800U) &&
       (*Switch_k != 0.0) && (*Switch3 != 0.0) && (*Switch10 != 0.0) &&
       (*Switch11 != 0.0));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 2U;
      g_previousEvent = VehCtrlMdel240926_2018b_amks_DW.sfEvent;
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

      VehCtrlMdel240926_2018b_amks_DW.sfEvent = g_previousEvent;
    }
    break;

   case VehCtrlMdel240926_2018_IN_start:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 >= 500U) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 1U;
      g_previousEvent = VehCtrlMdel240926_2018b_amks_DW.sfEvent;
      VehCtrlMdel240926_2018b_amks_DW.sfEvent = VehCtrlMdel24092_event_AMKCANON;
      if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_AMKCANenable !=
          0U) {
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

      VehCtrlMdel240926_2018b_amks_DW.sfEvent = g_previousEvent;
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
  /* S-Function (fcncallgen): '<S350>/Function-Call Generator' incorporates:
   *  SubSystem: '<S350>/CCPBackground'
   */

  /* S-Function (ec5744_ccpslb): '<S365>/CCPBackground' */
  ccpBackground();
  Lin0_Background();

  /* End of Outputs for S-Function (fcncallgen): '<S350>/Function-Call Generator' */
}

/* Model step function for TID2 */
void VehCtrlMdel240926_2018b_amkspdlimit_step2(void) /* Sample time: [0.005s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S348>/5ms' incorporates:
   *  SubSystem: '<S348>/daq5ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S363>/CCPDAQ' */
  ccpDaq(0);

  /* End of Outputs for S-Function (fcncallgen): '<S348>/5ms' */
}

/* Model step function for TID3 */
void VehCtrlMdel240926_2018b_amkspdlimit_step3(void) /* Sample time: [0.01s, 0.0s] */
{
  boolean_T rtb_ignition;
  real_T rtb_Gain5;
  real_T rtb_Yk1;
  real_T rtb_Gain4;
  real_T rtb_Switch2_on;
  real32_T rtb_Add;
  real32_T rtb_Acc_POS;
  boolean_T rtb_LogicalOperator2;
  boolean_T rtb_Compare;
  boolean_T rtb_LogicalOperator7;
  boolean_T rtb_Compare_am;
  boolean_T rtb_LowerRelop1_b;
  real_T elapseTime;
  real_T rtb_UkYk1;
  real_T rtb_g_mpss1;
  real32_T rtb_Add4_j;
  real32_T rtb_Add7;
  real32_T rtb_Add6_p;
  real32_T rtb_Switch2_mn;
  real32_T rtb_Divide_g;
  real32_T rtb_Switch2_b0;
  real32_T rtb_CastToDouble;
  boolean_T rtb_Compare_b;
  real32_T rtb_VxIMU_est;
  real32_T rtb_Ax;
  real32_T rtb_Add10;
  real_T rtb_deltafalllimit_iz;
  real_T rtb_deltafalllimit_i4;
  real32_T rtb_deltafalllimit_n;
  real32_T rtb_deltafalllimit_om;
  boolean_T rtb_UpperRelop_ir;
  real32_T rtb_MaxWhlSpd_mps_n;
  uint32_T rtb_Gain1;
  uint32_T rtb_Gain;
  uint32_T FunctionCallSubsystem_ELAPS_T;
  int32_T Brk_F;
  real_T WhlSpdFL;
  real_T WhlSpdFR;
  real_T WhlSpdRR_mps;
  real_T WhlSpdRL_mps;
  real_T FLWhlStrAng;
  real_T FRWhlStrAng;
  boolean_T rtb_LogicalOperator_idx_0;

  /* S-Function (fcncallgen): '<S3>/10ms7' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* SignalConversion generated from: '<S118>/Constant' */
  HVCUTOFF = true;

  /* S-Function (ec5744_swislbu3): '<S118>/SwitchInput' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240926_2018b_amksp_B.Drive_ready= ec_gpio_read(92);

  /* Logic: '<S118>/Logical Operator' */
  rtb_ignition = !VehCtrlMdel240926_2018b_amksp_B.Drive_ready;

  /* Chart: '<S118>/Timer' incorporates:
   *  Constant: '<S118>/Constant5'
   */
  VehCtrlMdel240926_201_Timer(rtb_ignition, 0.11F, &ignition,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer);

  /* Chart: '<S118>/Timer1' incorporates:
   *  Constant: '<S118>/Constant1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c23_VehCtrlMdel240926
      == 0U) {
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c23_VehCtrlMdel240926 =
      1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel240926_2018b_ =
      3U;
    VehCtrlMdel240926_2018b_amks_DW.x += 0.01;
    AMKSWITCH = 0.0;
  } else {
    switch
      (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel240926_2018b_)
    {
     case VehCtrlMdel2409_IN_InterState_d:
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel240926_2018b_
        = 3U;
      VehCtrlMdel240926_2018b_amks_DW.x += 0.01;
      AMKSWITCH = 0.0;
      break;

     case VehCtrlMdel240926_2018_IN_Out_b:
      AMKSWITCH = 1.0;
      break;

     default:
      /* case IN_Trigger: */
      AMKSWITCH = 0.0;
      rtb_ignition = (VehCtrlMdel240926_2018b_amks_DW.x < 0.10999999940395355);
      if (rtb_ignition) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel240926_2018b_
          = 3U;
        VehCtrlMdel240926_2018b_amks_DW.x += 0.01;
        AMKSWITCH = 0.0;
      } else {
        if (VehCtrlMdel240926_2018b_amks_DW.x >= 0.10999999940395355) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel240926_2018b_
            = 2U;
          AMKSWITCH = 1.0;
          VehCtrlMdel240926_2018b_amks_DW.x = 0.0;
        }
      }
      break;
    }
  }

  /* End of Chart: '<S118>/Timer1' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms7' */

  /* S-Function (fcncallgen): '<S3>/10ms6' incorporates:
   *  SubSystem: '<S3>/EMRAXMCU_RECIEVE'
   */
  /* S-Function (ec5744_canreceiveslb): '<S115>/CANReceive1' */

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

  /* Call the system: <S115>/MCU_pwr */

  /* Output and update for function-call system: '<S115>/MCU_pwr' */

  /* Outputs for Enabled SubSystem: '<S170>/MCU_VCUMeter1' incorporates:
   *  EnablePort: '<S172>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S172>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S172>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S172>/CAN Unpack' */
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
                voltage = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S170>/MCU_VCUMeter1' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S115>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S115>/CANReceive3' */

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

  /* Call the system: <S115>/MCU_state */

  /* Output and update for function-call system: '<S115>/MCU_state' */

  /* Outputs for Enabled SubSystem: '<S171>/MCU_state' incorporates:
   *  EnablePort: '<S178>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S178>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S178>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S178>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S171>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S115>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms6' */

  /* S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S111>/CANReceive3' */

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

  /* Call the system: <S111>/ABS_BUS_state */

  /* Output and update for function-call system: '<S111>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S119>/IMU_state' incorporates:
   *  EnablePort: '<S120>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_m > 0) {
    /* S-Function (ecucoder_canunmessage): '<S120>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S120>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S120>/CAN Unpack1' */
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

  /* End of Outputs for SubSystem: '<S119>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S111>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms3' */

  /* S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S117>/CANReceive3' */

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

  /* Call the system: <S117>/StrWhSnis_state */

  /* Output and update for function-call system: '<S117>/StrWhSnis_state' */

  /* Outputs for Enabled SubSystem: '<S190>/IMU_state' incorporates:
   *  EnablePort: '<S191>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S191>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S191>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S191>/CAN Unpack1' */
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

  /* End of Outputs for SubSystem: '<S190>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S117>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms4' */

  /* S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S125>/CANReceive3' */

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

  /* Call the system: <S125>/AMKMCU_state */

  /* Output and update for function-call system: '<S125>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S127>/MCU_state' incorporates:
   *  EnablePort: '<S130>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S130>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S130>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S130>/CAN Unpack' */
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
                MCFL_ActualTorque = result;
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

    /* Gain: '<S130>/Gain' */
    MCFL_ActualVelocity = -VehCtrlMdel240926_2018b_amksp_B.MCFL_ActualVelocity_p;
  }

  /* End of Outputs for SubSystem: '<S127>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S125>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S125>/CANReceive1' */

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

  /* Call the system: <S125>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S125>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S128>/MCU_state' incorporates:
   *  EnablePort: '<S140>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S140>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S140>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S140>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S128>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S125>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S125>/CANReceive2' */

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

  /* Call the system: <S125>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S125>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S129>/MCU_state' incorporates:
   *  EnablePort: '<S142>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S142>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S142>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S142>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S129>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S125>/CANReceive2' */

  /* S-Function (ec5744_canreceiveslb): '<S126>/CANReceive3' */

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

  /* Call the system: <S126>/AMKMCU_state */

  /* Output and update for function-call system: '<S126>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S146>/MCU_state' incorporates:
   *  EnablePort: '<S149>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_a > 0) {
    /* S-Function (ecucoder_canunmessage): '<S149>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S149>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S149>/CAN Unpack' */
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
                MCFR_ActualTorque = result;
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

  /* End of Outputs for SubSystem: '<S146>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S126>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S126>/CANReceive1' */

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

  /* Call the system: <S126>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S126>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S147>/MCU_state' incorporates:
   *  EnablePort: '<S158>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2_o > 0) {
    /* S-Function (ecucoder_canunmessage): '<S158>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S158>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S158>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S147>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S126>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S126>/CANReceive2' */

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

  /* Call the system: <S126>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S126>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S148>/MCU_state' incorporates:
   *  EnablePort: '<S160>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S160>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S160>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S160>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S148>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S126>/CANReceive2' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms5' */

  /* S-Function (fcncallgen): '<S3>/10ms' incorporates:
   *  SubSystem: '<S3>/AccBrk_BUS'
   */
  /* S-Function (ec5744_asislbu3): '<S113>/Acc3' */

  /* Read the ADC conversion result of the analog signal */
  Acc1= adc_read_chan(1,2);

  /* S-Function (ec5744_asislbu3): '<S113>/Acc4' */

  /* Read the ADC conversion result of the analog signal */
  Acc2= adc_read_chan(1,4);

  /* S-Function (ec5744_asislbu3): '<S113>/Brk1' */

  /* Read the ADC conversion result of the analog signal */
  Brk1= adc_read_chan(1,0);

  /* S-Function (ec5744_asislbu3): '<S113>/Brk2' */

  /* Read the ADC conversion result of the analog signal */
  Brk2= adc_read_chan(0,13);

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms' */

  /* S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* S-Function (ec5744_canreceiveslb): '<S116>/CANReceive3' */

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

  /* Call the system: <S116>/IMU_state */

  /* Output and update for function-call system: '<S116>/IMU_state' */

  /* Outputs for Enabled SubSystem: '<S185>/MCU_state' incorporates:
   *  EnablePort: '<S186>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_ma > 0) {
    /* S-Function (ecucoder_canunmessage): '<S186>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S186>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S186>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S185>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S116>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms2' */

  /* S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S114>/CANReceive3' */

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

  /* Call the system: <S114>/ABS_BUS_state */

  /* Output and update for function-call system: '<S114>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S168>/IMU_state' incorporates:
   *  EnablePort: '<S169>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_k > 0) {
    /* S-Function (ecucoder_canunmessage): '<S169>/CANUnPackMessage4' */

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

  /* End of Outputs for SubSystem: '<S168>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S114>/CANReceive3' */
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

  /* Lookup_n-D: '<S202>/1-D Lookup Table1' */
  F_BrkPrs = look1_iu16bflftfIu16_binlc(Brk1,
    VehCtrlMdel240926_2018b__ConstP.pooled57,
    VehCtrlMdel240926_2018b__ConstP.pooled57, 1U);

  /* DataTypeConversion: '<S202>/Data Type Conversion' */
  rtb_Add = F_BrkPrs;

  /* SignalConversion generated from: '<S200>/Out1' */
  Brk_F = (int32_T)rtb_Add;

  /* Gain: '<S202>/Gain2' */
  rtb_Gain1 = 45875U * Acc2;

  /* Gain: '<S202>/Gain3' incorporates:
   *  UnitDelay: '<S202>/Unit Delay1'
   */
  rtb_Gain = 39322U * VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S202>/Add3' */
  Acc_vol2 = (rtb_Gain >> 1) + rtb_Gain1;

  /* RelationalOperator: '<S211>/Compare' */
  rtb_ignition = (Acc_vol2 <= 32768000U);

  /* RelationalOperator: '<S212>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol2 >= 294912000U);

  /* Logic: '<S202>/Logical Operator1' */
  rtb_ignition = (rtb_ignition || rtb_LogicalOperator2);

  /* Gain: '<S202>/Gain' */
  rtb_Gain = 45875U * Acc1;

  /* UnitDelay: '<S202>/Unit Delay' incorporates:
   *  UnitDelay: '<S202>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_k;

  /* Gain: '<S202>/Gain1' incorporates:
   *  UnitDelay: '<S202>/Unit Delay1'
   */
  rtb_Gain1 = 39322U * VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S202>/Add2' */
  Acc_vol = (rtb_Gain1 >> 1) + rtb_Gain;

  /* RelationalOperator: '<S207>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol <= 32768000U);

  /* RelationalOperator: '<S208>/Compare' */
  rtb_Compare = (Acc_vol >= 294912000U);

  /* Logic: '<S202>/Logical Operator' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_Compare);

  /* Logic: '<S202>/Logical Operator2' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_ignition);

  /* DataTypeConversion: '<S202>/Data Type Conversion2' */
  rtb_Add = (real32_T)Acc_vol * 1.52587891E-5F;

  /* MATLAB Function: '<S202>/MATLAB Function' */
  Acc_POS = fmaxf(fminf((rtb_Add - 2300.0F) * -0.4F, 100.0F), 0.0F);

  /* Lookup_n-D: '<S202>/1-D Lookup Table3' */
  Acc_POS2 = look1_iu32n16bflftfIu32_binlc(Acc_vol2,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable3_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable3_tableData, 1U);

  /* DataTypeConversion: '<S202>/Data Type Conversion4' */
  rtb_Add = (real32_T)Acc_POS2 * 1.52587891E-5F;

  /* Sum: '<S202>/Add1' */
  rtb_Acc_POS = Acc_POS - rtb_Add;

  /* Abs: '<S202>/Abs' */
  rtb_Acc_POS = fabsf(rtb_Acc_POS);

  /* RelationalOperator: '<S215>/Compare' incorporates:
   *  Constant: '<S215>/Constant'
   */
  rtb_Compare = (rtb_Acc_POS > 15.0F);

  /* RelationalOperator: '<S213>/Compare' incorporates:
   *  Constant: '<S213>/Constant'
   */
  rtb_ignition = (Acc_POS > 100.0F);

  /* RelationalOperator: '<S214>/Compare' incorporates:
   *  Constant: '<S214>/Constant'
   */
  rtb_LogicalOperator7 = (rtb_Add > 100.0F);

  /* Logic: '<S202>/Logical Operator3' */
  rtb_ignition = (rtb_ignition || rtb_LogicalOperator7);

  /* RelationalOperator: '<S216>/Compare' incorporates:
   *  Constant: '<S216>/Constant'
   */
  rtb_LogicalOperator7 = (Brk1 <= 300);

  /* RelationalOperator: '<S217>/Compare' incorporates:
   *  Constant: '<S217>/Constant'
   */
  rtb_Compare_am = (Brk1 >= 4500);

  /* Logic: '<S202>/Logical Operator5' */
  rtb_LogicalOperator7 = (rtb_LogicalOperator7 || rtb_Compare_am);

  /* RelationalOperator: '<S209>/Compare' incorporates:
   *  Constant: '<S209>/Constant'
   */
  rtb_Compare_am = (Brk2 <= 300);

  /* RelationalOperator: '<S210>/Compare' incorporates:
   *  Constant: '<S210>/Constant'
   */
  rtb_LowerRelop1_b = (Brk2 >= 4500);

  /* Logic: '<S202>/Logical Operator6' */
  rtb_Compare_am = (rtb_Compare_am || rtb_LowerRelop1_b);

  /* Logic: '<S202>/Logical Operator7' */
  rtb_LogicalOperator7 = (rtb_LogicalOperator7 || rtb_Compare_am);

  /* Logic: '<S202>/Logical Operator4' */
  rtb_ignition = (rtb_ignition || rtb_Compare || rtb_LogicalOperator2 ||
                  rtb_LogicalOperator7);

  /* Chart: '<S202>/Timer' incorporates:
   *  Constant: '<S202>/Constant1'
   */
  VehCtrlMdel240926_201_Timer(rtb_ignition, 0.11F, &Trq_CUT,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_a);

  /* UnitDelay: '<S238>/Delay Input2'
   *
   * Block description for '<S238>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE;

  /* SampleTimeMath: '<S238>/sample time'
   *
   * About '<S238>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S238>/delta rise limit' */
  rtb_Gain5 = 1200.0 * elapseTime;

  /* Sum: '<S238>/Difference Inputs1'
   *
   * Block description for '<S238>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = StrWhlAng - rtb_Yk1;

  /* RelationalOperator: '<S241>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain5);

  /* Switch: '<S241>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S238>/delta fall limit' */
    rtb_deltafalllimit_iz = -1200.0 * elapseTime;

    /* RelationalOperator: '<S241>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S241>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S241>/Switch' */
    rtb_Gain5 = rtb_UkYk1;
  }

  /* End of Switch: '<S241>/Switch2' */

  /* Sum: '<S238>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S238>/Delay Input2'
   *
   * Block description for '<S238>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S238>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE = rtb_Gain5 + rtb_Yk1;

  /* Abs: '<S204>/Abs' incorporates:
   *  UnitDelay: '<S238>/Delay Input2'
   *
   * Block description for '<S238>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain5 = fabs(VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE);

  /* RelationalOperator: '<S237>/Compare' incorporates:
   *  Constant: '<S237>/Constant'
   */
  rtb_ignition = (rtb_Gain5 > 120.0);

  /* Chart: '<S204>/Timer' incorporates:
   *  Constant: '<S204>/Constant5'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_on,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_k);

  /* UnitDelay: '<S253>/Delay Input2'
   *
   * Block description for '<S253>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a;

  /* SampleTimeMath: '<S253>/sample time'
   *
   * About '<S253>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S253>/delta rise limit' */
  rtb_Yk1 = 10.0 * elapseTime;

  /* Sum: '<S253>/Difference Inputs1'
   *
   * Block description for '<S253>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RL - rtb_Gain5;

  /* RelationalOperator: '<S261>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Yk1);

  /* Switch: '<S261>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S253>/delta fall limit' */
    rtb_Yk1 = -10.0 * elapseTime;

    /* RelationalOperator: '<S261>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_Yk1);

    /* Switch: '<S261>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_Yk1;
    }

    /* End of Switch: '<S261>/Switch' */
    rtb_Yk1 = rtb_UkYk1;
  }

  /* End of Switch: '<S261>/Switch2' */

  /* Saturate: '<S206>/Saturation' incorporates:
   *  Sum: '<S253>/Difference Inputs2'
   *  UnitDelay: '<S253>/Delay Input2'
   *
   * Block description for '<S253>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S253>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a = rtb_Yk1 + rtb_Gain5;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a > 30.0) {
    rtb_Gain5 = 30.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a < 0.0) {
    rtb_Gain5 = 0.0;
  } else {
    rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a;
  }

  /* End of Saturate: '<S206>/Saturation' */

  /* Gain: '<S206>/Gain' */
  rtb_Gain5 *= 0.27777777777777779;

  /* RelationalOperator: '<S245>/Compare' incorporates:
   *  Constant: '<S245>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain5 >= 0.0);

  /* RelationalOperator: '<S246>/Compare' incorporates:
   *  Constant: '<S246>/Constant'
   */
  rtb_Compare_am = (rtb_Gain5 < 40.0);

  /* Logic: '<S206>/OR' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S206>/Timer' incorporates:
   *  Constant: '<S206>/Constant5'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_le,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_b);

  /* UnitDelay: '<S254>/Delay Input2'
   *
   * Block description for '<S254>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b;

  /* SampleTimeMath: '<S254>/sample time'
   *
   * About '<S254>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S254>/delta rise limit' */
  rtb_Gain4 = 10.0 * elapseTime;

  /* Sum: '<S254>/Difference Inputs1'
   *
   * Block description for '<S254>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RR - rtb_Yk1;

  /* RelationalOperator: '<S262>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain4);

  /* Switch: '<S262>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S254>/delta fall limit' */
    rtb_deltafalllimit_iz = -10.0 * elapseTime;

    /* RelationalOperator: '<S262>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S262>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S262>/Switch' */
    rtb_Gain4 = rtb_UkYk1;
  }

  /* End of Switch: '<S262>/Switch2' */

  /* Saturate: '<S206>/Saturation1' incorporates:
   *  Sum: '<S254>/Difference Inputs2'
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b = rtb_Gain4 + rtb_Yk1;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b > 30.0) {
    rtb_Gain4 = 30.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b < 0.0) {
    rtb_Gain4 = 0.0;
  } else {
    rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b;
  }

  /* End of Saturate: '<S206>/Saturation1' */

  /* Gain: '<S206>/Gain3' */
  rtb_Gain4 *= 0.27777777777777779;

  /* RelationalOperator: '<S247>/Compare' incorporates:
   *  Constant: '<S247>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain4 >= 0.0);

  /* RelationalOperator: '<S248>/Compare' incorporates:
   *  Constant: '<S248>/Constant'
   */
  rtb_Compare_am = (rtb_Gain4 < 40.0);

  /* Logic: '<S206>/OR1' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S206>/Timer1' incorporates:
   *  Constant: '<S206>/Constant1'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_i,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_n);

  /* UnitDelay: '<S255>/Delay Input2'
   *
   * Block description for '<S255>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h;

  /* SampleTimeMath: '<S255>/sample time'
   *
   * About '<S255>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S255>/delta rise limit' */
  rtb_Switch2_on = 10.0 * elapseTime;

  /* Sum: '<S255>/Difference Inputs1'
   *
   * Block description for '<S255>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = MCFR_ActualVelocity - rtb_Yk1;

  /* RelationalOperator: '<S263>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Switch2_on);

  /* Switch: '<S263>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S255>/delta fall limit' */
    rtb_deltafalllimit_iz = -10.0 * elapseTime;

    /* RelationalOperator: '<S263>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S263>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S263>/Switch' */
    rtb_Switch2_on = rtb_UkYk1;
  }

  /* End of Switch: '<S263>/Switch2' */

  /* Saturate: '<S206>/Saturation2' incorporates:
   *  Sum: '<S255>/Difference Inputs2'
   *  UnitDelay: '<S255>/Delay Input2'
   *
   * Block description for '<S255>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S255>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h = rtb_Switch2_on +
    rtb_Yk1;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h > 30.0) {
    rtb_Switch2_on = 30.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h < 0.0) {
    rtb_Switch2_on = 0.0;
  } else {
    rtb_Switch2_on = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h;
  }

  /* End of Saturate: '<S206>/Saturation2' */

  /* Gain: '<S206>/Gain1' */
  rtb_Switch2_on *= 0.1341030088495575;

  /* RelationalOperator: '<S249>/Compare' incorporates:
   *  Constant: '<S249>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Switch2_on >= 0.0);

  /* RelationalOperator: '<S250>/Compare' incorporates:
   *  Constant: '<S250>/Constant'
   */
  rtb_Compare_am = (rtb_Switch2_on < 40.0);

  /* Logic: '<S206>/OR2' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S206>/Timer2' incorporates:
   *  Constant: '<S206>/Constant4'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_o,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_l);

  /* UnitDelay: '<S256>/Delay Input2'
   *
   * Block description for '<S256>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n;

  /* SampleTimeMath: '<S256>/sample time'
   *
   * About '<S256>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S256>/delta rise limit' */
  rtb_UkYk1 = 10.0 * elapseTime;

  /* Sum: '<S256>/Difference Inputs1'
   *
   * Block description for '<S256>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_iz = MCFL_ActualVelocity - rtb_Yk1;

  /* RelationalOperator: '<S264>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_iz > rtb_UkYk1);

  /* Switch: '<S264>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S256>/delta fall limit' */
    rtb_UkYk1 = -10.0 * elapseTime;

    /* RelationalOperator: '<S264>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_iz < rtb_UkYk1);

    /* Switch: '<S264>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_iz = rtb_UkYk1;
    }

    /* End of Switch: '<S264>/Switch' */
    rtb_UkYk1 = rtb_deltafalllimit_iz;
  }

  /* End of Switch: '<S264>/Switch2' */

  /* Saturate: '<S206>/Saturation3' incorporates:
   *  Sum: '<S256>/Difference Inputs2'
   *  UnitDelay: '<S256>/Delay Input2'
   *
   * Block description for '<S256>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S256>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n = rtb_UkYk1 + rtb_Yk1;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n > 30.0) {
    rtb_UkYk1 = 30.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n < 0.0) {
    rtb_UkYk1 = 0.0;
  } else {
    rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n;
  }

  /* End of Saturate: '<S206>/Saturation3' */

  /* Gain: '<S206>/Gain2' */
  rtb_UkYk1 *= 0.1341030088495575;

  /* RelationalOperator: '<S251>/Compare' incorporates:
   *  Constant: '<S251>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_UkYk1 >= 0.0);

  /* RelationalOperator: '<S252>/Compare' incorporates:
   *  Constant: '<S252>/Constant'
   */
  rtb_Compare_am = (rtb_UkYk1 < 40.0);

  /* Logic: '<S206>/OR3' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S206>/Timer3' incorporates:
   *  Constant: '<S206>/Constant8'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_h,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer3);

  /* SignalConversion generated from: '<S200>/Out1' */
  WhlSpdFL = rtb_UkYk1;

  /* SignalConversion generated from: '<S200>/Out1' */
  WhlSpdFR = rtb_Switch2_on;

  /* SignalConversion generated from: '<S200>/Out1' */
  WhlSpdRR_mps = rtb_Gain4;

  /* SignalConversion generated from: '<S200>/Out1' */
  WhlSpdRL_mps = rtb_Gain5;

  /* Gain: '<S204>/Gain' incorporates:
   *  UnitDelay: '<S238>/Delay Input2'
   *
   * Block description for '<S238>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = 0.7 * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE;

  /* UnitDelay: '<S204>/Unit Delay' */
  rtb_Switch2_on = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE;

  /* Gain: '<S204>/Gain1' */
  rtb_Switch2_on *= 0.3;

  /* Sum: '<S204>/Add2' */
  rtb_UkYk1 += rtb_Switch2_on;

  /* Lookup_n-D: '<S204>/1-D Lookup Table' */
  rtb_Switch2_on = look1_binlx(rtb_UkYk1,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable_tableData, 23U);

  /* SignalConversion generated from: '<S200>/Out1' */
  FLWhlStrAng = rtb_Switch2_on;

  /* Lookup_n-D: '<S204>/1-D Lookup Table1' */
  rtb_Switch2_on = look1_binlx(rtb_UkYk1,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_tableData, 23U);

  /* SignalConversion generated from: '<S200>/Out1' */
  FRWhlStrAng = rtb_Switch2_on;

  /* SignalConversion generated from: '<S200>/Out1' */
  rtb_Yk1 = rtb_UkYk1;

  /* Sum: '<S202>/Add' */
  rtb_Add += Acc_POS;

  /* Product: '<S202>/Divide' incorporates:
   *  Constant: '<S202>/Constant'
   */
  rtb_Acc_POS = (real32_T)(rtb_Add / 2.0);

  /* UnitDelay: '<S228>/Delay Input2'
   *
   * Block description for '<S228>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l;

  /* SampleTimeMath: '<S228>/sample time'
   *
   * About '<S228>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S228>/delta rise limit' incorporates:
   *  Constant: '<S227>/Constant'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* UnitDelay: '<S227>/Unit Delay' */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_g;

  /* Gain: '<S227>/Gain1' */
  rtb_Gain4 *= 0.3;

  /* Gain: '<S203>/g_mpss' incorporates:
   *  UnitDelay: '<S227>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_g = 9.8 * IMU_Ay_Value;

  /* Gain: '<S227>/Gain' incorporates:
   *  UnitDelay: '<S227>/Unit Delay'
   */
  rtb_Gain5 = 0.7 * VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_g;

  /* Sum: '<S227>/Add2' */
  rtb_deltafalllimit_iz = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S228>/Difference Inputs1'
   *
   * Block description for '<S228>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_iz -= rtb_UkYk1;

  /* RelationalOperator: '<S234>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_iz > rtb_Switch2_on);

  /* Switch: '<S234>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S228>/delta fall limit' incorporates:
     *  Constant: '<S227>/Constant1'
     */
    rtb_deltafalllimit_i4 = -5000.0 * elapseTime;

    /* RelationalOperator: '<S234>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_iz < rtb_deltafalllimit_i4);

    /* Switch: '<S234>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_iz = rtb_deltafalllimit_i4;
    }

    /* End of Switch: '<S234>/Switch' */
    rtb_Switch2_on = rtb_deltafalllimit_iz;
  }

  /* End of Switch: '<S234>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l = rtb_Switch2_on +
    rtb_UkYk1;

  /* RelationalOperator: '<S231>/LowerRelop1' incorporates:
   *  Constant: '<S227>/Constant6'
   *  UnitDelay: '<S228>/Delay Input2'
   *
   * Block description for '<S228>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l >
                       1.5);

  /* Switch: '<S231>/Switch2' incorporates:
   *  Constant: '<S227>/Constant6'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S231>/UpperRelop' incorporates:
     *  Constant: '<S227>/Constant7'
     *  UnitDelay: '<S228>/Delay Input2'
     *
     * Block description for '<S228>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_ignition = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l < -1.5);

    /* Switch: '<S231>/Switch' incorporates:
     *  Constant: '<S227>/Constant7'
     *  UnitDelay: '<S228>/Delay Input2'
     *
     * Block description for '<S228>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_ignition) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l;
    }

    /* End of Switch: '<S231>/Switch' */
  }

  /* End of Switch: '<S231>/Switch2' */

  /* SignalConversion generated from: '<S200>/Out1' */
  rtb_deltafalllimit_iz = rtb_UkYk1;

  /* UnitDelay: '<S229>/Delay Input2'
   *
   * Block description for '<S229>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m;

  /* SampleTimeMath: '<S229>/sample time'
   *
   * About '<S229>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S229>/delta rise limit' incorporates:
   *  Constant: '<S227>/Constant2'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S203>/g_mpss1' */
  rtb_g_mpss1 = 9.8 * IMU_Ax_Value;

  /* Gain: '<S227>/Gain2' */
  rtb_Gain4 = 0.7 * rtb_g_mpss1;

  /* UnitDelay: '<S227>/Unit Delay1' */
  rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE;

  /* Gain: '<S227>/Gain3' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S227>/Add1' */
  rtb_deltafalllimit_i4 = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S229>/Difference Inputs1'
   *
   * Block description for '<S229>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_i4 -= rtb_UkYk1;

  /* RelationalOperator: '<S235>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_i4 > rtb_Switch2_on);

  /* Switch: '<S235>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S229>/delta fall limit' incorporates:
     *  Constant: '<S227>/Constant4'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S235>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_i4 < elapseTime);

    /* Switch: '<S235>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_i4 = elapseTime;
    }

    /* End of Switch: '<S235>/Switch' */
    rtb_Switch2_on = rtb_deltafalllimit_i4;
  }

  /* End of Switch: '<S235>/Switch2' */

  /* Sum: '<S229>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S229>/Delay Input2'
   *
   * Block description for '<S229>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S229>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m = rtb_Switch2_on +
    rtb_UkYk1;

  /* RelationalOperator: '<S232>/LowerRelop1' incorporates:
   *  Constant: '<S227>/Constant8'
   *  UnitDelay: '<S229>/Delay Input2'
   *
   * Block description for '<S229>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m >
                       1.5);

  /* Switch: '<S232>/Switch2' incorporates:
   *  Constant: '<S227>/Constant8'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S232>/UpperRelop' incorporates:
     *  Constant: '<S227>/Constant9'
     *  UnitDelay: '<S229>/Delay Input2'
     *
     * Block description for '<S229>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_ignition = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m < -1.5);

    /* Switch: '<S232>/Switch' incorporates:
     *  Constant: '<S227>/Constant9'
     *  UnitDelay: '<S229>/Delay Input2'
     *
     * Block description for '<S229>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_ignition) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m;
    }

    /* End of Switch: '<S232>/Switch' */
  }

  /* End of Switch: '<S232>/Switch2' */

  /* SignalConversion generated from: '<S200>/Out1' */
  rtb_deltafalllimit_i4 = rtb_UkYk1;

  /* SampleTimeMath: '<S239>/sample time'
   *
   * About '<S239>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S239>/delta rise limit' */
  rtb_UkYk1 = 1200.0 * elapseTime;

  /* UnitDelay: '<S239>/Delay Input2'
   *
   * Block description for '<S239>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_on = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j;

  /* Sum: '<S239>/Difference Inputs1'
   *
   * Block description for '<S239>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 = StrWhlAngV - rtb_Switch2_on;

  /* RelationalOperator: '<S242>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Gain5 > rtb_UkYk1);

  /* Switch: '<S242>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S239>/delta fall limit' */
    rtb_UkYk1 = -1200.0 * elapseTime;

    /* RelationalOperator: '<S242>/UpperRelop' */
    rtb_ignition = (rtb_Gain5 < rtb_UkYk1);

    /* Switch: '<S242>/Switch' */
    if (rtb_ignition) {
      rtb_Gain5 = rtb_UkYk1;
    }

    /* End of Switch: '<S242>/Switch' */
    rtb_UkYk1 = rtb_Gain5;
  }

  /* End of Switch: '<S242>/Switch2' */

  /* Sum: '<S239>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S239>/Delay Input2'
   *
   * Block description for '<S239>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S239>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j = rtb_UkYk1 +
    rtb_Switch2_on;

  /* UnitDelay: '<S230>/Delay Input2'
   *
   * Block description for '<S230>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k;

  /* SampleTimeMath: '<S230>/sample time'
   *
   * About '<S230>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S230>/delta rise limit' incorporates:
   *  Constant: '<S227>/Constant3'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S227>/Gain4' */
  rtb_Gain4 = 0.7 * IMU_Yaw_Value;

  /* UnitDelay: '<S227>/Unit Delay2' */
  rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE;

  /* Gain: '<S227>/Gain5' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S227>/Add3' */
  rtb_Gain5 += rtb_Gain4;

  /* Sum: '<S230>/Difference Inputs1'
   *
   * Block description for '<S230>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 -= rtb_UkYk1;

  /* RelationalOperator: '<S236>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Gain5 > rtb_Switch2_on);

  /* Switch: '<S236>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S230>/delta fall limit' incorporates:
     *  Constant: '<S227>/Constant5'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S236>/UpperRelop' */
    rtb_ignition = (rtb_Gain5 < elapseTime);

    /* Switch: '<S236>/Switch' */
    if (rtb_ignition) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S236>/Switch' */
    rtb_Switch2_on = rtb_Gain5;
  }

  /* End of Switch: '<S236>/Switch2' */

  /* Saturate: '<S227>/Saturation2' incorporates:
   *  Sum: '<S230>/Difference Inputs2'
   *  UnitDelay: '<S230>/Delay Input2'
   *
   * Block description for '<S230>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S230>/Delay Input2':
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

  /* End of Saturate: '<S227>/Saturation2' */

  /* Switch: '<S205>/Switch' incorporates:
   *  Constant: '<S205>/Constant4'
   */
  if (MCFL_DCVoltage != 0.0) {
    /* MinMax: '<S205>/Max' incorporates:
     *  Constant: '<S205>/Constant2'
     */
    elapseTime = fmax(MCFL_DCVoltage, 2.0);

    /* Product: '<S205>/Product' */
    rtb_Switch2_on = MCFL_ActualTorque * MCFL_ActualVelocity;

    /* Product: '<S205>/Divide' incorporates:
     *  Constant: '<S205>/Constant'
     */
    rtb_Switch2_on /= 9550.0;

    /* Product: '<S205>/Divide1' */
    AMKFL_Current = rtb_Switch2_on / elapseTime;
  } else {
    AMKFL_Current = 0.0;
  }

  /* End of Switch: '<S205>/Switch' */

  /* Switch: '<S205>/Switch1' incorporates:
   *  Constant: '<S205>/Constant5'
   */
  if (MCFR_DCVoltage != 0.0) {
    /* MinMax: '<S205>/Max1' incorporates:
     *  Constant: '<S205>/Constant3'
     */
    elapseTime = fmax(MCFR_DCVoltage, 2.0);

    /* Product: '<S205>/Product1' */
    rtb_Switch2_on = MCFR_ActualTorque * MCFR_ActualVelocity;

    /* Product: '<S205>/Divide2' incorporates:
     *  Constant: '<S205>/Constant1'
     */
    rtb_Switch2_on /= 9550.0;

    /* Product: '<S205>/Divide3' */
    AMKFR_Current = rtb_Switch2_on / elapseTime;
  } else {
    AMKFR_Current = 0.0;
  }

  /* End of Switch: '<S205>/Switch1' */

  /* Update for UnitDelay: '<S202>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm = Acc2;

  /* Update for UnitDelay: '<S202>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_k = Acc1;

  /* Update for UnitDelay: '<S204>/Unit Delay' incorporates:
   *  UnitDelay: '<S238>/Delay Input2'
   *
   * Block description for '<S238>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE;

  /* Update for UnitDelay: '<S227>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE = rtb_g_mpss1;

  /* Update for UnitDelay: '<S227>/Unit Delay2' */
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

  /* Gain: '<S201>/Gain5' */
  elapseTime = 10.0 * VehCtrlMdel240926_2018b_amksp_B.CANUnpack_o1;

  /* DataTypeConversion: '<S201>/Cast To Double' */
  rtb_CastToDouble = (real32_T)elapseTime;

  /* MinMax: '<S329>/Min3' incorporates:
   *  Gain: '<S283>/Gain'
   *  UnitDelay: '<S283>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p *= 0.3F;

  /* UnitDelay: '<S287>/Delay Input2'
   *
   * Block description for '<S287>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* SampleTimeMath: '<S287>/sample time'
   *
   * About '<S287>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S287>/delta rise limit' */
  rtb_Add4_j = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S201>/Cast To Double1' */
  rtb_Add7 = (real32_T)WhlSpdFL;

  /* DataTypeConversion: '<S201>/Cast To Double6' */
  rtb_Add6_p = (real32_T)FLWhlStrAng;

  /* Gain: '<S269>/Gain2' */
  rtb_Add6_p *= 0.0174532924F;

  /* Trigonometry: '<S269>/Asin' */
  rtb_Add6_p = cosf(rtb_Add6_p);

  /* Product: '<S269>/Product1' */
  rtb_Add7 *= rtb_Add6_p;

  /* DataTypeConversion: '<S201>/Cast To Double5' */
  rtb_Add6_p = (real32_T)rtb_UkYk1;

  /* Gain: '<S269>/Gain4' */
  rtb_Add6_p *= 0.0174532924F;

  /* Product: '<S269>/Product3' */
  rtb_Switch2_mn = 0.6F * rtb_Add6_p;

  /* Sum: '<S269>/Add2' */
  rtb_Add = rtb_Add7 - rtb_Switch2_mn;

  /* UnitDelay: '<S276>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_j;

  /* Sum: '<S276>/Add4' */
  rtb_Add7 = rtb_Add - rtb_Add7;

  /* Product: '<S276>/Divide' incorporates:
   *  Constant: '<S276>/steptime'
   */
  rtb_Divide_g = rtb_Add7 / 0.01F;

  /* RelationalOperator: '<S288>/LowerRelop1' incorporates:
   *  Constant: '<S283>/Constant1'
   */
  rtb_LogicalOperator2 = (rtb_Divide_g > 100.0F);

  /* Switch: '<S288>/Switch2' incorporates:
   *  Constant: '<S283>/Constant1'
   */
  if (rtb_LogicalOperator2) {
    rtb_Divide_g = 100.0F;
  } else {
    /* RelationalOperator: '<S288>/UpperRelop' incorporates:
     *  Constant: '<S283>/Constant'
     */
    rtb_ignition = (rtb_Divide_g < -100.0F);

    /* Switch: '<S288>/Switch' incorporates:
     *  Constant: '<S283>/Constant'
     */
    if (rtb_ignition) {
      rtb_Divide_g = -100.0F;
    }

    /* End of Switch: '<S288>/Switch' */
  }

  /* End of Switch: '<S288>/Switch2' */

  /* Sum: '<S287>/Difference Inputs1'
   *
   * Block description for '<S287>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Divide_g -= rtb_Switch2_b0;

  /* RelationalOperator: '<S289>/LowerRelop1' */
  rtb_LogicalOperator2 = (rtb_Divide_g > rtb_Add4_j);

  /* Switch: '<S289>/Switch2' */
  if (!rtb_LogicalOperator2) {
    /* Product: '<S287>/delta fall limit' */
    rtb_deltafalllimit_n = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S289>/UpperRelop' */
    rtb_ignition = (rtb_Divide_g < rtb_deltafalllimit_n);

    /* Switch: '<S289>/Switch' */
    if (rtb_ignition) {
      rtb_Divide_g = rtb_deltafalllimit_n;
    }

    /* End of Switch: '<S289>/Switch' */
    rtb_Add4_j = rtb_Divide_g;
  }

  /* End of Switch: '<S289>/Switch2' */

  /* Sum: '<S287>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S287>/Delay Input2'
   *
   * Block description for '<S287>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S287>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2 = rtb_Add4_j +
    rtb_Switch2_b0;

  /* Gain: '<S283>/Gain1' incorporates:
   *  UnitDelay: '<S287>/Delay Input2'
   *
   * Block description for '<S287>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add7 = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* MinMax: '<S329>/Min3' incorporates:
   *  Abs: '<S276>/Abs'
   *  Sum: '<S276>/Add'
   *  Sum: '<S283>/Add'
   *  UnitDelay: '<S283>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p += rtb_Add7;
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p -= rtb_CastToDouble;
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p = fabsf
    (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p);

  /* RelationalOperator: '<S279>/Compare' incorporates:
   *  Constant: '<S279>/Constant'
   *  UnitDelay: '<S283>/Unit Delay'
   */
  rtb_LogicalOperator2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p <=
    0.5F);

  /* UnitDelay: '<S284>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj;

  /* Gain: '<S284>/Gain' */
  rtb_Add7 *= 0.3F;

  /* UnitDelay: '<S290>/Delay Input2'
   *
   * Block description for '<S290>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* SampleTimeMath: '<S290>/sample time'
   *
   * About '<S290>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S290>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(100.0 * elapseTime);

  /* MinMax: '<S329>/Min3' incorporates:
   *  DataTypeConversion: '<S201>/Cast To Double2'
   *  UnitDelay: '<S283>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p = (real32_T)WhlSpdFR;

  /* DataTypeConversion: '<S201>/Cast To Double7' */
  rtb_Add10 = (real32_T)FRWhlStrAng;

  /* Gain: '<S269>/Gain3' */
  rtb_Add10 *= 0.0174532924F;

  /* Trigonometry: '<S269>/Asin1' */
  rtb_Add10 = cosf(rtb_Add10);

  /* Product: '<S269>/Product2' incorporates:
   *  UnitDelay: '<S283>/Unit Delay'
   */
  rtb_Add10 *= VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p;

  /* Sum: '<S269>/Add3' */
  rtb_Divide_g = rtb_Switch2_mn + rtb_Add10;

  /* UnitDelay: '<S276>/Unit Delay1' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n;

  /* Sum: '<S276>/Add5' */
  rtb_Add10 = rtb_Divide_g - rtb_Add10;

  /* Product: '<S276>/Divide1' incorporates:
   *  Constant: '<S276>/steptime1'
   */
  rtb_deltafalllimit_n = rtb_Add10 / 0.01F;

  /* RelationalOperator: '<S291>/LowerRelop1' incorporates:
   *  Constant: '<S284>/Constant1'
   */
  rtb_Compare = (rtb_deltafalllimit_n > 100.0F);

  /* Switch: '<S291>/Switch2' incorporates:
   *  Constant: '<S284>/Constant1'
   */
  if (rtb_Compare) {
    rtb_deltafalllimit_n = 100.0F;
  } else {
    /* RelationalOperator: '<S291>/UpperRelop' incorporates:
     *  Constant: '<S284>/Constant'
     */
    rtb_ignition = (rtb_deltafalllimit_n < -100.0F);

    /* Switch: '<S291>/Switch' incorporates:
     *  Constant: '<S284>/Constant'
     */
    if (rtb_ignition) {
      rtb_deltafalllimit_n = -100.0F;
    }

    /* End of Switch: '<S291>/Switch' */
  }

  /* End of Switch: '<S291>/Switch2' */

  /* Sum: '<S290>/Difference Inputs1'
   *
   * Block description for '<S290>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_n -= rtb_Add4_j;

  /* RelationalOperator: '<S292>/LowerRelop1' */
  rtb_Compare = (rtb_deltafalllimit_n > rtb_Switch2_b0);

  /* Switch: '<S292>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S290>/delta fall limit' */
    rtb_deltafalllimit_om = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S292>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_n < rtb_deltafalllimit_om);

    /* Switch: '<S292>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_n = rtb_deltafalllimit_om;
    }

    /* End of Switch: '<S292>/Switch' */
    rtb_Switch2_b0 = rtb_deltafalllimit_n;
  }

  /* End of Switch: '<S292>/Switch2' */

  /* Sum: '<S290>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S290>/Delay Input2'
   *
   * Block description for '<S290>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S290>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e = rtb_Switch2_b0 +
    rtb_Add4_j;

  /* Gain: '<S284>/Gain1' incorporates:
   *  UnitDelay: '<S290>/Delay Input2'
   *
   * Block description for '<S290>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* Sum: '<S284>/Add' */
  rtb_Add7 += rtb_Add10;

  /* Sum: '<S276>/Add1' */
  rtb_Add7 -= rtb_CastToDouble;

  /* Abs: '<S276>/Abs1' */
  rtb_Add7 = fabsf(rtb_Add7);

  /* RelationalOperator: '<S280>/Compare' incorporates:
   *  Constant: '<S280>/Constant'
   */
  rtb_Compare = (rtb_Add7 <= 0.5F);

  /* UnitDelay: '<S285>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a;

  /* Gain: '<S285>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S293>/Delay Input2'
   *
   * Block description for '<S293>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* SampleTimeMath: '<S293>/sample time'
   *
   * About '<S293>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S293>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S201>/Cast To Double3' */
  rtb_Add4_j = (real32_T)WhlSpdRL_mps;

  /* Product: '<S269>/Product' */
  rtb_Add6_p *= 0.58F;

  /* Sum: '<S269>/Add' */
  rtb_deltafalllimit_n = rtb_Add4_j - rtb_Add6_p;

  /* UnitDelay: '<S276>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_l;

  /* Sum: '<S276>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_n - rtb_Add4_j;

  /* Product: '<S276>/Divide2' incorporates:
   *  Constant: '<S276>/steptime2'
   */
  rtb_deltafalllimit_om = rtb_Add4_j / 0.01F;

  /* RelationalOperator: '<S294>/LowerRelop1' incorporates:
   *  Constant: '<S285>/Constant1'
   */
  rtb_LogicalOperator7 = (rtb_deltafalllimit_om > 100.0F);

  /* Switch: '<S294>/Switch2' incorporates:
   *  Constant: '<S285>/Constant1'
   */
  if (rtb_LogicalOperator7) {
    rtb_deltafalllimit_om = 100.0F;
  } else {
    /* RelationalOperator: '<S294>/UpperRelop' incorporates:
     *  Constant: '<S285>/Constant'
     */
    rtb_ignition = (rtb_deltafalllimit_om < -100.0F);

    /* Switch: '<S294>/Switch' incorporates:
     *  Constant: '<S285>/Constant'
     */
    if (rtb_ignition) {
      rtb_deltafalllimit_om = -100.0F;
    }

    /* End of Switch: '<S294>/Switch' */
  }

  /* End of Switch: '<S294>/Switch2' */

  /* Sum: '<S293>/Difference Inputs1'
   *
   * Block description for '<S293>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_om -= rtb_Switch2_mn;

  /* RelationalOperator: '<S295>/LowerRelop1' */
  rtb_LogicalOperator7 = (rtb_deltafalllimit_om > rtb_Add7);

  /* Switch: '<S295>/Switch2' */
  if (!rtb_LogicalOperator7) {
    /* Product: '<S293>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S295>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_om < rtb_Add7);

    /* Switch: '<S295>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_om = rtb_Add7;
    }

    /* End of Switch: '<S295>/Switch' */
    rtb_Add7 = rtb_deltafalllimit_om;
  }

  /* End of Switch: '<S295>/Switch2' */

  /* Sum: '<S293>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S293>/Delay Input2'
   *
   * Block description for '<S293>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S293>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk = rtb_Add7 +
    rtb_Switch2_mn;

  /* Gain: '<S285>/Gain1' incorporates:
   *  UnitDelay: '<S293>/Delay Input2'
   *
   * Block description for '<S293>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* Sum: '<S285>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S276>/Add2' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S276>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S281>/Compare' incorporates:
   *  Constant: '<S281>/Constant'
   */
  rtb_LogicalOperator7 = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S286>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nc;

  /* Gain: '<S286>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S296>/Delay Input2'
   *
   * Block description for '<S296>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* SampleTimeMath: '<S296>/sample time'
   *
   * About '<S296>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S296>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S201>/Cast To Double4' */
  rtb_Add4_j = (real32_T)WhlSpdRR_mps;

  /* Sum: '<S269>/Add1' */
  rtb_deltafalllimit_om = rtb_Add6_p + rtb_Add4_j;

  /* UnitDelay: '<S276>/Unit Delay3' */
  rtb_Add6_p = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE;

  /* Sum: '<S276>/Add7' */
  rtb_Add6_p = rtb_deltafalllimit_om - rtb_Add6_p;

  /* Product: '<S276>/Divide3' incorporates:
   *  Constant: '<S276>/steptime3'
   */
  rtb_Add6_p /= 0.01F;

  /* RelationalOperator: '<S297>/LowerRelop1' incorporates:
   *  Constant: '<S286>/Constant1'
   */
  rtb_ignition = (rtb_Add6_p > 100.0F);

  /* Switch: '<S297>/Switch2' incorporates:
   *  Constant: '<S286>/Constant1'
   */
  if (rtb_ignition) {
    rtb_Add6_p = 100.0F;
  } else {
    /* RelationalOperator: '<S297>/UpperRelop' incorporates:
     *  Constant: '<S286>/Constant'
     */
    rtb_ignition = (rtb_Add6_p < -100.0F);

    /* Switch: '<S297>/Switch' incorporates:
     *  Constant: '<S286>/Constant'
     */
    if (rtb_ignition) {
      rtb_Add6_p = -100.0F;
    }

    /* End of Switch: '<S297>/Switch' */
  }

  /* End of Switch: '<S297>/Switch2' */

  /* Sum: '<S296>/Difference Inputs1'
   *
   * Block description for '<S296>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add6_p -= rtb_Switch2_mn;

  /* RelationalOperator: '<S298>/LowerRelop1' */
  rtb_ignition = (rtb_Add6_p > rtb_Add7);

  /* Switch: '<S298>/Switch2' */
  if (!rtb_ignition) {
    /* Product: '<S296>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S298>/UpperRelop' */
    rtb_ignition = (rtb_Add6_p < rtb_Add7);

    /* Switch: '<S298>/Switch' */
    if (rtb_ignition) {
      rtb_Add6_p = rtb_Add7;
    }

    /* End of Switch: '<S298>/Switch' */
    rtb_Add7 = rtb_Add6_p;
  }

  /* End of Switch: '<S298>/Switch2' */

  /* Sum: '<S296>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S296>/Delay Input2'
   *
   * Block description for '<S296>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S296>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c = rtb_Add7 +
    rtb_Switch2_mn;

  /* Gain: '<S286>/Gain1' incorporates:
   *  UnitDelay: '<S296>/Delay Input2'
   *
   * Block description for '<S296>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* Sum: '<S286>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S276>/Add3' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S276>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S282>/Compare' incorporates:
   *  Constant: '<S282>/Constant'
   */
  rtb_ignition = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S303>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l;

  /* Gain: '<S303>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S307>/Delay Input2'
   *
   * Block description for '<S307>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* SampleTimeMath: '<S307>/sample time'
   *
   * About '<S307>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S307>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S277>/Unit Delay4' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i;

  /* UnitDelay: '<S277>/Unit Delay' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a0;

  /* Sum: '<S277>/Add4' */
  rtb_Add4_j = rtb_Add - rtb_Add4_j;

  /* Product: '<S277>/Divide' incorporates:
   *  Constant: '<S277>/steptime'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S277>/Add' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S277>/Add8' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i - rtb_Add7;

  /* Product: '<S277>/Divide4' incorporates:
   *  Constant: '<S277>/steptime4'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S308>/LowerRelop1' incorporates:
   *  Constant: '<S303>/Constant1'
   */
  rtb_Compare_am = (rtb_Add7 > 100.0F);

  /* Switch: '<S308>/Switch2' incorporates:
   *  Constant: '<S303>/Constant1'
   */
  if (rtb_Compare_am) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S308>/UpperRelop' incorporates:
     *  Constant: '<S303>/Constant'
     */
    rtb_Compare_am = (rtb_Add7 < -100.0F);

    /* Switch: '<S308>/Switch' incorporates:
     *  Constant: '<S303>/Constant'
     */
    if (rtb_Compare_am) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S308>/Switch' */
  }

  /* End of Switch: '<S308>/Switch2' */

  /* Sum: '<S307>/Difference Inputs1'
   *
   * Block description for '<S307>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S309>/LowerRelop1' */
  rtb_Compare_am = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S309>/Switch2' */
  if (!rtb_Compare_am) {
    /* Product: '<S307>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S309>/UpperRelop' */
    rtb_Compare_am = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S309>/Switch' */
    if (rtb_Compare_am) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S309>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S309>/Switch2' */

  /* Sum: '<S307>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S307>/Delay Input2'
   *
   * Block description for '<S307>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S307>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i = rtb_Add6_p +
    rtb_Switch2_mn;

  /* Gain: '<S303>/Gain1' incorporates:
   *  UnitDelay: '<S307>/Delay Input2'
   *
   * Block description for '<S307>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* Sum: '<S303>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S277>/Abs' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S299>/Compare' incorporates:
   *  Constant: '<S299>/Constant'
   */
  rtb_Compare_am = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S304>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ap;

  /* Gain: '<S304>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S310>/Delay Input2'
   *
   * Block description for '<S310>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* SampleTimeMath: '<S310>/sample time'
   *
   * About '<S310>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S310>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S277>/Unit Delay5' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE;

  /* UnitDelay: '<S277>/Unit Delay1' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_am;

  /* Sum: '<S277>/Add5' */
  rtb_Add4_j = rtb_Divide_g - rtb_Add4_j;

  /* Product: '<S277>/Divide1' incorporates:
   *  Constant: '<S277>/steptime1'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S277>/Add1' incorporates:
   *  UnitDelay: '<S277>/Unit Delay5'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S277>/Add10' incorporates:
   *  UnitDelay: '<S277>/Unit Delay5'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE - rtb_Add7;

  /* Product: '<S277>/Divide5' incorporates:
   *  Constant: '<S277>/steptime5'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S311>/LowerRelop1' incorporates:
   *  Constant: '<S304>/Constant1'
   */
  rtb_LowerRelop1_b = (rtb_Add7 > 100.0F);

  /* Switch: '<S311>/Switch2' incorporates:
   *  Constant: '<S304>/Constant1'
   */
  if (rtb_LowerRelop1_b) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S311>/UpperRelop' incorporates:
     *  Constant: '<S304>/Constant'
     */
    rtb_LowerRelop1_b = (rtb_Add7 < -100.0F);

    /* Switch: '<S311>/Switch' incorporates:
     *  Constant: '<S304>/Constant'
     */
    if (rtb_LowerRelop1_b) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S311>/Switch' */
  }

  /* End of Switch: '<S311>/Switch2' */

  /* Sum: '<S310>/Difference Inputs1'
   *
   * Block description for '<S310>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S312>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S312>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S310>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S312>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S312>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S312>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S312>/Switch2' */

  /* Sum: '<S310>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S310>/Delay Input2'
   *
   * Block description for '<S310>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S310>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el = rtb_Add6_p +
    rtb_Switch2_mn;

  /* Gain: '<S304>/Gain1' incorporates:
   *  UnitDelay: '<S310>/Delay Input2'
   *
   * Block description for '<S310>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* Sum: '<S304>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S277>/Abs1' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S300>/Compare' incorporates:
   *  Constant: '<S300>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S305>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o;

  /* Gain: '<S305>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S313>/Delay Input2'
   *
   * Block description for '<S313>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p;

  /* SampleTimeMath: '<S313>/sample time'
   *
   * About '<S313>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S313>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S277>/Unit Delay6' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE;

  /* UnitDelay: '<S277>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_c;

  /* Sum: '<S277>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_n - rtb_Add4_j;

  /* Product: '<S277>/Divide2' incorporates:
   *  Constant: '<S277>/steptime2'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S277>/Add2' incorporates:
   *  UnitDelay: '<S277>/Unit Delay6'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S277>/Add12' incorporates:
   *  UnitDelay: '<S277>/Unit Delay6'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE - rtb_Add7;

  /* Product: '<S277>/Divide6' incorporates:
   *  Constant: '<S277>/steptime6'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S314>/LowerRelop1' incorporates:
   *  Constant: '<S305>/Constant1'
   */
  rtb_Compare_b = (rtb_Add7 > 100.0F);

  /* Switch: '<S314>/Switch2' incorporates:
   *  Constant: '<S305>/Constant1'
   */
  if (rtb_Compare_b) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S314>/UpperRelop' incorporates:
     *  Constant: '<S305>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S314>/Switch' incorporates:
     *  Constant: '<S305>/Constant'
     */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S314>/Switch' */
  }

  /* End of Switch: '<S314>/Switch2' */

  /* Sum: '<S313>/Difference Inputs1'
   *
   * Block description for '<S313>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S315>/LowerRelop1' */
  rtb_Compare_b = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S315>/Switch2' */
  if (!rtb_Compare_b) {
    /* Product: '<S313>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S315>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S315>/Switch' */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S315>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S315>/Switch2' */

  /* Sum: '<S313>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S313>/Delay Input2'
   *
   * Block description for '<S313>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S313>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p = rtb_Add6_p +
    rtb_Switch2_mn;

  /* Gain: '<S305>/Gain1' incorporates:
   *  UnitDelay: '<S313>/Delay Input2'
   *
   * Block description for '<S313>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p;

  /* Sum: '<S305>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S277>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S301>/Compare' incorporates:
   *  Constant: '<S301>/Constant'
   */
  rtb_Compare_b = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S306>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ah;

  /* Gain: '<S306>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S316>/Delay Input2'
   *
   * Block description for '<S316>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* SampleTimeMath: '<S316>/sample time'
   *
   * About '<S316>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S316>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S277>/Unit Delay7' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE;

  /* UnitDelay: '<S277>/Unit Delay3' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_d;

  /* Sum: '<S277>/Add7' */
  rtb_Add4_j = rtb_deltafalllimit_om - rtb_Add4_j;

  /* Product: '<S277>/Divide3' incorporates:
   *  Constant: '<S277>/steptime3'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S277>/Add3' incorporates:
   *  UnitDelay: '<S277>/Unit Delay7'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S277>/Add14' incorporates:
   *  UnitDelay: '<S277>/Unit Delay7'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE - rtb_Add7;

  /* Product: '<S277>/Divide7' incorporates:
   *  Constant: '<S277>/steptime7'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S317>/LowerRelop1' incorporates:
   *  Constant: '<S306>/Constant1'
   */
  rtb_UpperRelop_ir = (rtb_Add7 > 100.0F);

  /* Switch: '<S317>/Switch2' incorporates:
   *  Constant: '<S306>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S317>/UpperRelop' incorporates:
     *  Constant: '<S306>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S317>/Switch' incorporates:
     *  Constant: '<S306>/Constant'
     */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S317>/Switch' */
  }

  /* End of Switch: '<S317>/Switch2' */

  /* Sum: '<S316>/Difference Inputs1'
   *
   * Block description for '<S316>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S318>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S318>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S316>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S318>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S318>/Switch' */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S318>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S318>/Switch2' */

  /* Sum: '<S316>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S316>/Delay Input2'
   *
   * Block description for '<S316>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S316>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt = rtb_Add6_p +
    rtb_Switch2_mn;

  /* Gain: '<S306>/Gain1' incorporates:
   *  UnitDelay: '<S316>/Delay Input2'
   *
   * Block description for '<S316>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* Sum: '<S306>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S277>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S302>/Compare' incorporates:
   *  Constant: '<S302>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.8F);

  /* Logic: '<S267>/Logical Operator' */
  rtb_LogicalOperator_idx_0 = (rtb_LogicalOperator2 || rtb_Compare_am);
  rtb_Compare = (rtb_Compare || rtb_LowerRelop1_b);
  rtb_LogicalOperator7 = (rtb_LogicalOperator7 || rtb_Compare_b);
  rtb_LogicalOperator2 = (rtb_ignition || rtb_UpperRelop_ir);

  /* UnitDelay: '<S201>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_lh;

  /* Sum: '<S278>/Add' */
  rtb_Switch2_mn = rtb_Add - rtb_Add10;

  /* Abs: '<S278>/Abs' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S319>/Compare' incorporates:
   *  Constant: '<S319>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S278>/AND3' */
  rtb_Compare_b = (rtb_UpperRelop_ir && (VehCtrlMdel240926_2018b_amksp_B.Exit_h
    != 0.0));

  /* Sum: '<S278>/Add1' */
  rtb_Switch2_mn = rtb_Divide_g - rtb_Add10;

  /* Abs: '<S278>/Abs1' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S320>/Compare' incorporates:
   *  Constant: '<S320>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S278>/AND2' */
  rtb_LowerRelop1_b = (rtb_UpperRelop_ir &&
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_o != 0.0));

  /* Sum: '<S278>/Add2' */
  rtb_Switch2_mn = rtb_deltafalllimit_n - rtb_Add10;

  /* Abs: '<S278>/Abs2' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S321>/Compare' incorporates:
   *  Constant: '<S321>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S278>/AND' */
  rtb_Compare_am = (rtb_UpperRelop_ir &&
                    (VehCtrlMdel240926_2018b_amksp_B.Exit_le != 0.0));

  /* Sum: '<S278>/Add3' */
  rtb_Add10 = rtb_deltafalllimit_om - rtb_Add10;

  /* Abs: '<S278>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S322>/Compare' incorporates:
   *  Constant: '<S322>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 2.0F);

  /* Logic: '<S278>/AND1' */
  rtb_ignition = (rtb_UpperRelop_ir && (VehCtrlMdel240926_2018b_amksp_B.Exit_i
    != 0.0));

  /* Logic: '<S267>/Logical Operator1' */
  rtb_UpperRelop_ir = (rtb_Compare_b && rtb_LogicalOperator_idx_0);
  rtb_Compare = (rtb_LowerRelop1_b && rtb_Compare);
  rtb_LogicalOperator7 = (rtb_Compare_am && rtb_LogicalOperator7);
  rtb_ignition = (rtb_ignition && rtb_LogicalOperator2);

  /* Chart: '<S267>/Timer' incorporates:
   *  Constant: '<S267>/Constant1'
   */
  VehCtrlMdel240926_20_Timer1(rtb_UpperRelop_ir, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_c,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_o);

  /* Chart: '<S267>/Timer1' incorporates:
   *  Constant: '<S267>/Constant2'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_lh4,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_m);

  /* Chart: '<S267>/Timer2' incorporates:
   *  Constant: '<S267>/Constant3'
   */
  VehCtrlMdel240926_20_Timer1(rtb_LogicalOperator7, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_lh,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_g);

  /* Chart: '<S267>/Timer3' incorporates:
   *  Constant: '<S267>/Constant4'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_a,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer3_i);

  /* Logic: '<S265>/Logical Operator' */
  rtb_UpperRelop_ir = ((VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0));

  /* Logic: '<S265>/Logical Operator1' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* UnitDelay: '<S265>/Unit Delay4' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_m;

  /* Sum: '<S265>/Add1' */
  rtb_Add10 = rtb_Acc_POS - rtb_Add10;

  /* RelationalOperator: '<S270>/Compare' incorporates:
   *  Constant: '<S270>/Constant'
   */
  rtb_Compare_b = (rtb_Add10 > 0.1F);

  /* Logic: '<S265>/Logical Operator2' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir || rtb_Compare_b);

  /* Logic: '<S265>/AND' */
  rtb_ignition = ((VehCtrlMdel240926_2018b_amksp_B.CANUnpack_o1 != 0.0) &&
                  rtb_UpperRelop_ir);

  /* UnitDelay: '<S265>/Unit Delay3' */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_f;

  /* Logic: '<S265>/Logical Operator3' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S265>/Switch3' incorporates:
   *  UnitDelay: '<S265>/Unit Delay1'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S265>/Switch4' incorporates:
     *  Constant: '<S265>/InitZORE'
     */
    if (!rtb_ignition) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k = 0.0F;
    }

    /* End of Switch: '<S265>/Switch4' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_o =
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Switch: '<S265>/Switch3' */

  /* UnitDelay: '<S268>/Unit Delay3' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_p;

  /* Sum: '<S268>/Add5' incorporates:
   *  UnitDelay: '<S268>/Unit Delay1'
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d - rtb_Add10;

  /* Product: '<S268>/Divide3' incorporates:
   *  Constant: '<S268>/steptime3'
   */
  rtb_Add10 /= 0.01F;

  /* UnitDelay: '<S268>/Unit Delay2' */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_f;

  /* Sum: '<S268>/Add9' */
  rtb_Switch2_mn -= rtb_Add10;

  /* UnitDelay: '<S268>/Unit Delay4' */
  rtb_Add6_p = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn;

  /* Sum: '<S268>/Add6' incorporates:
   *  Constant: '<S268>/steptime4'
   */
  rtb_Add6_p += 0.1F;

  /* Sum: '<S268>/Add8' incorporates:
   *  Constant: '<S268>/steptime6'
   */
  rtb_Add7 = rtb_Add6_p + 2.0F;

  /* Product: '<S268>/Divide5' */
  rtb_Add7 = 1.0F / rtb_Add7 * rtb_Add6_p;

  /* Logic: '<S268>/Logical Operator' */
  rtb_LogicalOperator2 = ((VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) ||
    (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) ||
    (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) ||
    (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0));

  /* Switch: '<S268>/Switch13' incorporates:
   *  Constant: '<S268>/Constant10'
   */
  if (rtb_LogicalOperator2) {
    rtb_Add4_j = rtb_Add7;
  } else {
    rtb_Add4_j = 1.0F;
  }

  /* End of Switch: '<S268>/Switch13' */

  /* Product: '<S268>/Divide6' */
  rtb_Switch2_mn *= rtb_Add4_j;

  /* Sum: '<S268>/Add10' */
  rtb_Ax = rtb_Switch2_mn + rtb_Add10;

  /* Switch: '<S265>/Switch1' */
  if (rtb_ignition) {
    /* Saturate: '<S265>/Saturation1' */
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d > 200.0F) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = 200.0F;
    } else {
      if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d < -10.0F) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = -10.0F;
      }
    }

    /* Product: '<S265>/Product' incorporates:
     *  Constant: '<S265>/steptime1'
     */
    rtb_Switch2_mn = rtb_Ax * 0.01F;

    /* Saturate: '<S265>/Saturation1' incorporates:
     *  Sum: '<S265>/Add'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d += rtb_Switch2_mn;
  } else {
    /* Saturate: '<S265>/Saturation1' incorporates:
     *  Constant: '<S265>/Constant'
     *  UnitDelay: '<S265>/Unit Delay'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = 0.0F;
  }

  /* End of Switch: '<S265>/Switch1' */

  /* Saturate: '<S265>/Saturation' */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d > 200.0F) {
    rtb_Add10 = 200.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d < -10.0F) {
    rtb_Add10 = -10.0F;
  } else {
    rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d;
  }

  /* End of Saturate: '<S265>/Saturation' */

  /* Sum: '<S265>/Add3' incorporates:
   *  UnitDelay: '<S265>/Unit Delay1'
   */
  rtb_VxIMU_est = rtb_Add10 +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_o;

  /* MinMax: '<S267>/Min1' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Add, rtb_Divide_g);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_n);
  rtb_Add10 = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_om);

  /* Sum: '<S265>/Add2' */
  rtb_Add10 -= rtb_VxIMU_est;

  /* RelationalOperator: '<S271>/Compare' incorporates:
   *  Constant: '<S271>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.0F);

  /* Switch: '<S265>/Switch6' incorporates:
   *  Constant: '<S265>/Reset'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S265>/Add10' incorporates:
     *  Constant: '<S265>/Steptime'
     */
    rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i + 0.01F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S265>/Switch6' */

  /* MinMax: '<S265>/Min' incorporates:
   *  Constant: '<S265>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i = fminf(rtb_Add10, 0.1F);

  /* RelationalOperator: '<S265>/Relational Operator9' incorporates:
   *  Constant: '<S265>/ResetDelay'
   *  UnitDelay: '<S265>/Unit Delay2'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i >= 0.1F);

  /* RelationalOperator: '<S324>/Compare' incorporates:
   *  Constant: '<S324>/Constant'
   */
  rtb_LogicalOperator7 = (rtb_Ax < -0.5F);

  /* Chart: '<S268>/Timer2' incorporates:
   *  Constant: '<S268>/Constant15'
   */
  VehCtrlMdel240926_20_Timer1(rtb_LogicalOperator7, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_j);

  /* UnitDelay: '<S328>/Delay Input2'
   *
   * Block description for '<S328>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;

  /* SampleTimeMath: '<S328>/sample time'
   *
   * About '<S328>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S328>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(10.0 * elapseTime);

  /* Sum: '<S329>/Add3' */
  rtb_Add4_j = ((rtb_deltafalllimit_om + rtb_deltafalllimit_n) + rtb_Divide_g) +
    rtb_Add;

  /* MinMax: '<S329>/Min4' */
  rtb_Switch2_b0 = fminf(rtb_Add, rtb_Divide_g);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_n);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_om);

  /* MinMax: '<S329>/Min3' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Add, rtb_Divide_g);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_n);
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p = fmaxf(rtb_MaxWhlSpd_mps_n,
    rtb_deltafalllimit_om);

  /* Sum: '<S329>/Add4' incorporates:
   *  UnitDelay: '<S283>/Unit Delay'
   */
  rtb_Add4_j = (rtb_Add4_j - rtb_Switch2_b0) -
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p;

  /* Gain: '<S329>/Gain1' */
  rtb_Add4_j *= 0.5F;

  /* Sum: '<S328>/Difference Inputs1'
   *
   * Block description for '<S328>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add4_j -= rtb_Add10;

  /* RelationalOperator: '<S337>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add4_j > rtb_Switch2_mn);

  /* Switch: '<S337>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S328>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-10.0 * elapseTime);

    /* RelationalOperator: '<S337>/UpperRelop' */
    rtb_LogicalOperator7 = (rtb_Add4_j < rtb_Switch2_mn);

    /* Switch: '<S337>/Switch' */
    if (rtb_LogicalOperator7) {
      rtb_Add4_j = rtb_Switch2_mn;
    }

    /* End of Switch: '<S337>/Switch' */
    rtb_Switch2_mn = rtb_Add4_j;
  }

  /* End of Switch: '<S337>/Switch2' */

  /* Sum: '<S328>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S328>/Delay Input2'
   *
   * Block description for '<S328>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S328>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g = rtb_Switch2_mn +
    rtb_Add10;

  /* RelationalOperator: '<S323>/Compare' incorporates:
   *  Constant: '<S323>/Constant'
   */
  rtb_LogicalOperator7 = (rtb_Ax > 0.5F);

  /* Chart: '<S268>/Timer1' incorporates:
   *  Constant: '<S268>/Constant14'
   */
  VehCtrlMdel240926_20_Timer1(rtb_LogicalOperator7, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_l,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_p);

  /* Logic: '<S268>/Logical Operator2' */
  rtb_UpperRelop_ir = !(VehCtrlMdel240926_2018b_amksp_B.Exit_l != 0.0);

  /* Switch: '<S268>/Switch6' incorporates:
   *  Switch: '<S268>/Switch4'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S268>/Switch5' incorporates:
     *  UnitDelay: '<S328>/Delay Input2'
     *
     * Block description for '<S328>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit != 0.0) {
      /* Switch: '<S268>/Switch11' incorporates:
       *  Constant: '<S268>/Constant7'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0) {
        rtb_Switch2_mn = rtb_deltafalllimit_om;
      } else {
        rtb_Switch2_mn = 0.0F;
      }

      /* End of Switch: '<S268>/Switch11' */

      /* Switch: '<S268>/Switch10' incorporates:
       *  Constant: '<S268>/Constant6'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) {
        rtb_Add10 = rtb_deltafalllimit_n;
      } else {
        rtb_Add10 = 0.0F;
      }

      /* End of Switch: '<S268>/Switch10' */

      /* Switch: '<S268>/Switch9' incorporates:
       *  Constant: '<S268>/Constant5'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) {
        rtb_Add4_j = rtb_Divide_g;
      } else {
        rtb_Add4_j = 0.0F;
      }

      /* End of Switch: '<S268>/Switch9' */

      /* Switch: '<S268>/Switch8' incorporates:
       *  Constant: '<S268>/Constant4'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) {
        rtb_Switch2_b0 = rtb_Add;
      } else {
        rtb_Switch2_b0 = 0.0F;
      }

      /* End of Switch: '<S268>/Switch8' */

      /* MinMax: '<S268>/Min1' */
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Switch2_b0, rtb_Add4_j);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Add10);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Switch2_mn);
    } else {
      rtb_MaxWhlSpd_mps_n = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;
    }

    /* End of Switch: '<S268>/Switch5' */
  } else {
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0) {
      /* Switch: '<S268>/Switch4' */
      rtb_Switch2_mn = rtb_deltafalllimit_om;
    } else {
      /* Switch: '<S268>/Switch4' incorporates:
       *  Constant: '<S268>/Constant3'
       */
      rtb_Switch2_mn = 9999.0F;
    }

    /* Switch: '<S268>/Switch3' incorporates:
     *  Constant: '<S268>/Constant2'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) {
      rtb_Add10 = rtb_deltafalllimit_n;
    } else {
      rtb_Add10 = 9999.0F;
    }

    /* End of Switch: '<S268>/Switch3' */

    /* Switch: '<S268>/Switch2' incorporates:
     *  Constant: '<S268>/Constant1'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) {
      rtb_Add4_j = rtb_Divide_g;
    } else {
      rtb_Add4_j = 9999.0F;
    }

    /* End of Switch: '<S268>/Switch2' */

    /* Switch: '<S268>/Switch1' incorporates:
     *  Constant: '<S268>/Constant'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) {
      rtb_Switch2_b0 = rtb_Add;
    } else {
      rtb_Switch2_b0 = 9999.0F;
    }

    /* End of Switch: '<S268>/Switch1' */

    /* MinMax: '<S268>/Min2' */
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add4_j);
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add10);
    rtb_MaxWhlSpd_mps_n = fminf(rtb_Switch2_b0, rtb_Switch2_mn);
  }

  /* End of Switch: '<S268>/Switch6' */

  /* Logic: '<S268>/NOT3' */
  rtb_UpperRelop_ir = !rtb_LogicalOperator2;

  /* Logic: '<S268>/Logical Operator3' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir && rtb_Compare);

  /* Logic: '<S268>/NOT4' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S268>/Switch7' incorporates:
   *  UnitDelay: '<S328>/Delay Input2'
   *
   * Block description for '<S328>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (rtb_UpperRelop_ir) {
    /* Logic: '<S268>/Logical Operator1' */
    rtb_Compare = (rtb_Compare || rtb_LogicalOperator2);

    /* Switch: '<S268>/Switch' */
    if (rtb_Compare) {
      rtb_VxIMU_est = rtb_MaxWhlSpd_mps_n;
    }

    /* End of Switch: '<S268>/Switch' */
  } else {
    rtb_VxIMU_est = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;
  }

  /* End of Switch: '<S268>/Switch7' */

  /* UnitDelay: '<S326>/Delay Input2'
   *
   * Block description for '<S326>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_af;

  /* Sum: '<S326>/Difference Inputs1'
   *
   * Block description for '<S326>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_VxIMU_est -= rtb_Add10;

  /* Switch: '<S268>/Switch12' incorporates:
   *  Constant: '<S268>/Constant8'
   *  Constant: '<S268>/Constant9'
   */
  if (rtb_LogicalOperator2) {
    rtb_Switch2_mn = 0.1F;
  } else {
    rtb_Switch2_mn = 0.05F;
  }

  /* End of Switch: '<S268>/Switch12' */

  /* Sum: '<S268>/Add4' */
  rtb_Add4_j = rtb_Ax + rtb_Switch2_mn;

  /* SampleTimeMath: '<S326>/sample time'
   *
   * About '<S326>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S326>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S335>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_VxIMU_est > rtb_Switch2_b0);

  /* Sum: '<S268>/Add3' */
  rtb_Ax -= rtb_Switch2_mn;

  /* Switch: '<S335>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S326>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S335>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_VxIMU_est < rtb_Switch2_mn);

    /* Switch: '<S335>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_VxIMU_est = rtb_Switch2_mn;
    }

    /* End of Switch: '<S335>/Switch' */
    rtb_Switch2_b0 = rtb_VxIMU_est;
  }

  /* End of Switch: '<S335>/Switch2' */

  /* Sum: '<S326>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S326>/Delay Input2'
   *
   * Block description for '<S326>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S326>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_af = rtb_Switch2_b0 +
    rtb_Add10;

  /* RelationalOperator: '<S333>/LowerRelop1' incorporates:
   *  Constant: '<S325>/Constant1'
   *  UnitDelay: '<S326>/Delay Input2'
   *
   * Block description for '<S326>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_af >
                       100.0F);

  /* Switch: '<S333>/Switch2' incorporates:
   *  Constant: '<S325>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_mn = 100.0F;
  } else {
    /* RelationalOperator: '<S333>/UpperRelop' incorporates:
     *  Constant: '<S325>/Constant'
     *  UnitDelay: '<S326>/Delay Input2'
     *
     * Block description for '<S326>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_LogicalOperator2 =
      (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_af < 0.0F);

    /* Switch: '<S333>/Switch' incorporates:
     *  Constant: '<S325>/Constant'
     *  UnitDelay: '<S326>/Delay Input2'
     *
     * Block description for '<S326>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_LogicalOperator2) {
      rtb_Switch2_mn = 0.0F;
    } else {
      rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_af;
    }

    /* End of Switch: '<S333>/Switch' */
  }

  /* End of Switch: '<S333>/Switch2' */

  /* UnitDelay: '<S332>/Delay Input2'
   *
   * Block description for '<S332>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* Sum: '<S332>/Difference Inputs1'
   *
   * Block description for '<S332>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_Switch2_mn - rtb_Add10;

  /* SampleTimeMath: '<S332>/sample time'
   *
   * About '<S332>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S332>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(15.0 * elapseTime);

  /* RelationalOperator: '<S334>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S334>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S332>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-15.0 * elapseTime);

    /* RelationalOperator: '<S334>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S334>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S334>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S334>/Switch2' */

  /* Sum: '<S332>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S332>/Delay Input2'
   *
   * Block description for '<S332>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S332>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f = rtb_Switch2_mn +
    rtb_Add10;

  /* UnitDelay: '<S325>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ncs;

  /* Gain: '<S325>/Gain' */
  rtb_Add10 *= 0.0F;

  /* Saturate: '<S30>/Saturation' incorporates:
   *  Sum: '<S325>/Add'
   *  UnitDelay: '<S332>/Delay Input2'
   *
   * Block description for '<S332>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehVxEst_mps = rtb_Add10 +
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* SampleTimeMath: '<S327>/sample time'
   *
   * About '<S327>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* UnitDelay: '<S327>/Delay Input2'
   *
   * Block description for '<S327>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu;

  /* Sum: '<S327>/Difference Inputs1'
   *
   * Block description for '<S327>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_MaxWhlSpd_mps_n - rtb_Add10;

  /* Product: '<S327>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S336>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S336>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S327>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S336>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S336>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S336>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S336>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu = rtb_Switch2_mn +
    rtb_Add10;

  /* Sum: '<S268>/Add7' incorporates:
   *  Constant: '<S268>/steptime5'
   */
  rtb_Add7 = 1.0F - rtb_Add7;

  /* Product: '<S268>/Divide4' incorporates:
   *  UnitDelay: '<S268>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn = rtb_Add7 * rtb_Add6_p;

  /* Update for MinMax: '<S329>/Min3' incorporates:
   *  UnitDelay: '<S283>/Unit Delay'
   *  UnitDelay: '<S287>/Delay Input2'
   *
   * Block description for '<S287>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* Update for UnitDelay: '<S276>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_j = rtb_Add;

  /* Update for UnitDelay: '<S284>/Unit Delay' incorporates:
   *  UnitDelay: '<S290>/Delay Input2'
   *
   * Block description for '<S290>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* Update for UnitDelay: '<S276>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n = rtb_Divide_g;

  /* Update for UnitDelay: '<S285>/Unit Delay' incorporates:
   *  UnitDelay: '<S293>/Delay Input2'
   *
   * Block description for '<S293>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* Update for UnitDelay: '<S276>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_l = rtb_deltafalllimit_n;

  /* Update for UnitDelay: '<S286>/Unit Delay' incorporates:
   *  UnitDelay: '<S296>/Delay Input2'
   *
   * Block description for '<S296>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nc =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* Update for UnitDelay: '<S276>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE = rtb_deltafalllimit_om;

  /* Update for UnitDelay: '<S303>/Unit Delay' incorporates:
   *  UnitDelay: '<S307>/Delay Input2'
   *
   * Block description for '<S307>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* Update for UnitDelay: '<S277>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a0 = rtb_Add;

  /* Update for UnitDelay: '<S304>/Unit Delay' incorporates:
   *  UnitDelay: '<S310>/Delay Input2'
   *
   * Block description for '<S310>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ap =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* Update for UnitDelay: '<S277>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_am = rtb_Divide_g;

  /* Update for UnitDelay: '<S305>/Unit Delay' incorporates:
   *  UnitDelay: '<S313>/Delay Input2'
   *
   * Block description for '<S313>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p;

  /* Update for UnitDelay: '<S277>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_c = rtb_deltafalllimit_n;

  /* Update for UnitDelay: '<S306>/Unit Delay' incorporates:
   *  UnitDelay: '<S316>/Delay Input2'
   *
   * Block description for '<S316>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ah =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* Update for UnitDelay: '<S277>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_d = rtb_deltafalllimit_om;

  /* Update for UnitDelay: '<S201>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_lh = VehVxEst_mps;

  /* Update for UnitDelay: '<S265>/Unit Delay4' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_m = rtb_Acc_POS;

  /* Update for UnitDelay: '<S201>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k = VehVxEst_mps;

  /* Update for UnitDelay: '<S265>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_f = rtb_ignition;

  /* Update for UnitDelay: '<S268>/Unit Delay3' incorporates:
   *  UnitDelay: '<S268>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_p =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d;

  /* Update for UnitDelay: '<S268>/Unit Delay1' incorporates:
   *  UnitDelay: '<S327>/Delay Input2'
   *
   * Block description for '<S327>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu;

  /* Update for UnitDelay: '<S268>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_f = rtb_CastToDouble;

  /* Update for UnitDelay: '<S325>/Unit Delay' incorporates:
   *  UnitDelay: '<S332>/Delay Input2'
   *
   * Block description for '<S332>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ncs =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms1' */

  /* S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* DataTypeConversion: '<S101>/Cast To Boolean' */
  KeyPressed = (ignition != 0.0);

  /* RelationalOperator: '<S103>/Compare' incorporates:
   *  Constant: '<S103>/Constant'
   */
  Brk = (Brk_F >= 600);

  /* RelationalOperator: '<S104>/Compare' incorporates:
   *  Constant: '<S104>/Constant'
   */
  ACC_Release = (rtb_Acc_POS <= 50.0F);

  /* Switch: '<S101>/Switch' incorporates:
   *  Constant: '<S101>/Constant1'
   *  Switch: '<S101>/Switch10'
   *  Switch: '<S101>/Switch11'
   *  Switch: '<S101>/Switch3'
   */
  if (AMKSWITCH != 0.0) {
    elapseTime = MCFL_bSystemReady;
    WhlSpdFL = MCFR_bSystemReady;
    WhlSpdFR = MCFL_bQuitInverterOn;
    WhlSpdRR_mps = MCFR_bQuitInverterOn;
  } else {
    elapseTime = 1.0;
    WhlSpdFL = 1.0;
    WhlSpdFR = 1.0;
    WhlSpdRR_mps = 1.0;
  }

  /* End of Switch: '<S101>/Switch' */

  /* Chart: '<S101>/Chart2' */
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
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready = 10U;
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = 0U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_Output = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_AMKCANenable = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MCFL_InverterOn = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MCFR_InverterOn = 1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn = 1U;
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_VehStat != 0U) {
      VehCtrlMdel240926_2018b_VehStat(&controller_ready, &elapseTime, &WhlSpdFL,
        &WhlSpdFR, &WhlSpdRR_mps);
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
      VehCtrlMdel240926_20_AMKDCready(&MCFL_bDCOn, &MCFR_bDCOn, &elapseTime,
        &WhlSpdFL, &WhlSpdFR, &WhlSpdRR_mps);
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
      VehCtrlMdel240926_2018b_amksp_B.AMKMCFL_InverterOn =
        (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn ==
         VehCtrlMdel240926_2018b_a_IN_ON);
      MCFR_InverterOn =
        (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn ==
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

    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MCFL_InverterOn !=
        0U) {
      switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn) {
       case VehCtrlMdel240926_2018b__IN_OFF:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel2_event_InverterFLON) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn = 2U;
        }
        break;

       case VehCtrlMdel240926_2018b_a_IN_ON:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel_event_InverterFLOFF) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_MCFR_InverterOn !=
        0U) {
      switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn) {
       case VehCtrlMdel240926_2018b__IN_OFF:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel2_event_InverterFRON) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn = 2U;
        }
        break;

       case VehCtrlMdel240926_2018b_a_IN_ON:
        if (VehCtrlMdel240926_2018b_amks_DW.sfEvent ==
            VehCtrlMdel_event_InverterFROFF) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn = 1U;
        }
        break;
      }
    }
  }

  /* End of Chart: '<S101>/Chart2' */

  /* Switch: '<S101>/Switch4' */
  VehCtrlMdel240926_2018b_amksp_B.MCFL_DCOn_setpoints_o = ((AMKSWITCH != 0.0) &&
    MCFL_DCOn_setpoints);

  /* End of Outputs for S-Function (fcncallgen): '<S2>/10ms' */

  /* S-Function (fcncallgen): '<S5>/10ms1' incorporates:
   *  SubSystem: '<S5>/Beeper'
   */
  /* RelationalOperator: '<S343>/Compare' incorporates:
   *  Constant: '<S343>/Constant'
   */
  rtb_ignition = (voltage > 60.0);

  /* Logic: '<S338>/NOT' */
  VehCtrlMdel240926_2018b_amksp_B.NOT = !rtb_ignition;

  /* S-Function (ec5744_pdsslbu3): '<S338>/PowerDriverSwitch(HS)' */

  /* Set level beeper_state for the specified power driver switch */
  ec_gpio_write(83,beeper_state);

  /* S-Function (ec5744_pdsslbu3): '<S338>/PowerDriverSwitch(HS)1' */

  /* Set level VehCtrlMdel240926_2018b_amksp_B.NOT for the specified power driver switch */
  ec_gpio_write(57,VehCtrlMdel240926_2018b_amksp_B.NOT);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms1' */

  /* S-Function (fcncallgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/PwrTrainTempPrtct'
   */
  /* MinMax: '<S8>/Max' */
  elapseTime = fmax(MCFL_TempInverter, MCFR_TempInverter);

  /* RelationalOperator: '<S94>/Compare' incorporates:
   *  Constant: '<S94>/Constant'
   */
  rtb_ignition = (elapseTime > 40.0);

  /* RelationalOperator: '<S95>/Compare' incorporates:
   *  Constant: '<S95>/Constant'
   */
  rtb_LogicalOperator2 = (elapseTime > 45.0);

  /* Logic: '<S8>/NOT' */
  rtb_Compare = !rtb_LogicalOperator2;

  /* Logic: '<S8>/AND' */
  rtb_ignition = (rtb_ignition && rtb_Compare);

  /* Switch: '<S8>/Switch' incorporates:
   *  Constant: '<S8>/Constant'
   */
  if (rtb_ignition) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table1' */
    WhlSpdRL_mps = look1_binlx(elapseTime,
      VehCtrlMdel240926_2018b__ConstP.pooled10,
      VehCtrlMdel240926_2018b__ConstP.pooled9, 7U);
  } else {
    WhlSpdRL_mps = 0.0;
  }

  /* End of Switch: '<S8>/Switch' */

  /* SignalConversion generated from: '<S8>/Out1' */
  WhlSpdFL = WhlSpdRL_mps;

  /* Chart: '<S8>/Timer1' incorporates:
   *  Constant: '<S8>/Constant2'
   */
  VehCtrlMdel240926_20_Timer1(rtb_LogicalOperator2, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_g,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1);

  /* RelationalOperator: '<S97>/Compare' incorporates:
   *  Constant: '<S97>/Constant'
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

  /* RelationalOperator: '<S96>/Compare' incorporates:
   *  Constant: '<S96>/Constant'
   */
  rtb_ignition = (MCU_Temp > 45.0);

  /* Logic: '<S8>/AND1' */
  rtb_ignition = (rtb_ignition && rtb_Compare);

  /* MinMax: '<S8>/Max1' */
  WhlSpdRR_mps = fmax(MCU_Temp, motor_Temp);

  /* Switch: '<S8>/Switch1' incorporates:
   *  Constant: '<S8>/Constant1'
   */
  if (rtb_ignition) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table3' */
    WhlSpdRL_mps = look1_binlx(WhlSpdRR_mps,
      VehCtrlMdel240926_2018b__ConstP.pooled10,
      VehCtrlMdel240926_2018b__ConstP.pooled9, 7U);
  } else {
    WhlSpdRL_mps = 0.0;
  }

  /* End of Switch: '<S8>/Switch1' */

  /* SignalConversion generated from: '<S8>/Out1' */
  WhlSpdFR = WhlSpdRL_mps;

  /* Lookup_n-D: '<S8>/2-D Lookup Table2' */
  WhlSpdRL_mps = look1_binlx(WhlSpdRR_mps,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable2_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable2_tableData, 6U);

  /* DataTypeConversion: '<S8>/Cast To Single' */
  rtb_CastToDouble = (real32_T)WhlSpdRL_mps;

  /* Gain: '<S8>/Gain' */
  rtb_CastToDouble *= 10.0F;

  /* SignalConversion generated from: '<S8>/Out1' */
  AMK_Trq_CUT = VehCtrlMdel240926_2018b_amksp_B.Exit_g;

  /* RelationalOperator: '<S98>/Compare' incorporates:
   *  Constant: '<S98>/Constant'
   */
  rtb_Compare = (elapseTime > 35.0);

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

  /* Lookup_n-D: '<S7>/2-D Lookup Table1' */
  rtb_Add6_p = look2_iflf_binlx(rtb_Acc_POS, VehVxEst_mps,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp01Data_l,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp02Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_tableData_g,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_maxIndex, 11U);

  /* UnitDelay: '<S77>/Unit Delay1' */
  rtb_ignition = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl;

  /* Saturate: '<S30>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    rtb_Switch2_b0 = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    rtb_Switch2_b0 = 0.0F;
  } else {
    rtb_Switch2_b0 = VehVxEst_mps;
  }

  /* Lookup_n-D: '<S30>/VehSpd_SlipTarget_mps' */
  rtb_Ax = look1_iflf_binlc(rtb_Switch2_b0,
    VehCtrlMdel240926_2018b__ConstP.pooled35,
    VehCtrlMdel240926_2018b__ConstP.pooled34, 3U);

  /* Sum: '<S30>/Add9' */
  rtb_Ax += rtb_Switch2_b0;

  /* Saturate: '<S30>/Saturation1' */
  if (rtb_Divide_g > 50.0F) {
    rtb_Divide_g = 50.0F;
  } else {
    if (rtb_Divide_g < 0.0F) {
      rtb_Divide_g = 0.0F;
    }
  }

  /* End of Saturate: '<S30>/Saturation1' */

  /* Sum: '<S30>/Add1' */
  rtb_Acc_POS = rtb_Ax - rtb_Divide_g;

  /* RelationalOperator: '<S30>/Relational Operator7' incorporates:
   *  Constant: '<S30>/Cal_DeltaV_mps'
   */
  rtb_LogicalOperator2 = (rtb_Acc_POS < 0.0F);

  /* Logic: '<S77>/Logical Operator4' */
  rtb_ignition = ((!rtb_ignition) && (!rtb_LogicalOperator2));

  /* Logic: '<S30>/Logical Operator2' */
  rtb_LogicalOperator2 = !rtb_LogicalOperator2;

  /* UnitDelay: '<S30>/Unit Delay4' */
  elapseTime = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE;

  /* RelationalOperator: '<S30>/Relational Operator8' incorporates:
   *  Constant: '<S30>/Cal_DeltaV_mps1'
   */
  rtb_Compare_am = (elapseTime > 235.0);

  /* Logic: '<S30>/Logical Operator1' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 && rtb_Compare_am);

  /* Switch: '<S78>/Switch6' incorporates:
   *  Constant: '<S78>/Reset'
   */
  if (rtb_LogicalOperator2) {
    /* Sum: '<S78>/Add10' incorporates:
     *  Constant: '<S78>/Steptime'
     */
    rtb_Ax = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 + 0.01F;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S78>/Switch6' */

  /* MinMax: '<S78>/Min' incorporates:
   *  Constant: '<S30>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 = fminf(rtb_Ax, 0.1F);

  /* RelationalOperator: '<S78>/Relational Operator9' incorporates:
   *  Constant: '<S30>/ResetDelay'
   *  UnitDelay: '<S78>/Unit Delay1'
   */
  rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 >= 0.1F);

  /* UnitDelay: '<S30>/Unit Delay3' */
  rtb_LogicalOperator2 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e;

  /* Logic: '<S30>/Logical Operator3' */
  rtb_Compare_am = (rtb_Compare_am || rtb_LogicalOperator2);

  /* Logic: '<S77>/Logical Operator5' incorporates:
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl = ((!rtb_ignition) &&
    (!rtb_Compare_am));

  /* UnitDelay: '<S88>/Unit Delay1' */
  rtb_Compare_am = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e;

  /* Saturate: '<S31>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    rtb_Ax = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    rtb_Ax = 0.0F;
  } else {
    rtb_Ax = VehVxEst_mps;
  }

  /* End of Saturate: '<S31>/Saturation' */

  /* Lookup_n-D: '<S31>/VehSpd_SlipTarget_mps' */
  rtb_Add4_j = look1_iflf_binlc(rtb_Ax, VehCtrlMdel240926_2018b__ConstP.pooled35,
    VehCtrlMdel240926_2018b__ConstP.pooled34, 3U);

  /* Sum: '<S31>/Add9' */
  rtb_Add4_j += rtb_Ax;

  /* Sum: '<S7>/Add1' */
  rtb_Add7 = rtb_deltafalllimit_n + rtb_deltafalllimit_om;

  /* Gain: '<S7>/Gain2' */
  rtb_Add7 *= 0.5F;

  /* Saturate: '<S31>/Saturation1' */
  if (rtb_Add7 < 0.0F) {
    rtb_VxIMU_est = 0.0F;
  } else {
    rtb_VxIMU_est = rtb_Add7;
  }

  /* End of Saturate: '<S31>/Saturation1' */

  /* Sum: '<S31>/Add1' */
  rtb_deltafalllimit_n = rtb_Add4_j - rtb_VxIMU_est;

  /* RelationalOperator: '<S31>/Relational Operator7' incorporates:
   *  Constant: '<S31>/Cal_DeltaV_mps'
   */
  rtb_LogicalOperator2 = (rtb_deltafalllimit_n < 0.0F);

  /* Logic: '<S88>/Logical Operator4' */
  rtb_Compare_am = ((!rtb_Compare_am) && (!rtb_LogicalOperator2));

  /* Logic: '<S31>/Logical Operator2' */
  rtb_LogicalOperator2 = !rtb_LogicalOperator2;

  /* UnitDelay: '<S31>/Unit Delay4' */
  elapseTime = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b;

  /* RelationalOperator: '<S31>/Relational Operator8' incorporates:
   *  Constant: '<S31>/Cal_DeltaV_mps1'
   */
  rtb_ignition = (elapseTime > 235.0);

  /* Logic: '<S31>/Logical Operator1' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 && rtb_ignition);

  /* Switch: '<S89>/Switch6' incorporates:
   *  Constant: '<S89>/Reset'
   */
  if (rtb_LogicalOperator2) {
    /* Sum: '<S89>/Add10' incorporates:
     *  Constant: '<S89>/Steptime'
     */
    rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i + 0.01F;
  } else {
    rtb_Add7 = 0.0F;
  }

  /* End of Switch: '<S89>/Switch6' */

  /* MinMax: '<S89>/Min' incorporates:
   *  Constant: '<S31>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i = fminf(rtb_Add7, 0.1F);

  /* RelationalOperator: '<S89>/Relational Operator9' incorporates:
   *  Constant: '<S31>/ResetDelay'
   *  UnitDelay: '<S89>/Unit Delay1'
   */
  rtb_LogicalOperator2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i >=
    0.1F);

  /* UnitDelay: '<S31>/Unit Delay3' */
  rtb_ignition = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_a;

  /* Logic: '<S31>/Logical Operator3' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_ignition);

  /* Logic: '<S88>/Logical Operator5' incorporates:
   *  UnitDelay: '<S88>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e = ((!rtb_Compare_am) &&
    (!rtb_LogicalOperator2));

  /* UnitDelay: '<S68>/Unit Delay1' */
  rtb_Compare_am = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp;

  /* Saturate: '<S29>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    rtb_Add7 = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    rtb_Add7 = 0.0F;
  } else {
    rtb_Add7 = VehVxEst_mps;
  }

  /* End of Saturate: '<S29>/Saturation' */

  /* Lookup_n-D: '<S29>/VehSpd_SlipTarget_mps' */
  rtb_Add4_j = look1_iflf_binlc(rtb_Add7,
    VehCtrlMdel240926_2018b__ConstP.pooled35,
    VehCtrlMdel240926_2018b__ConstP.pooled34, 3U);

  /* Sum: '<S29>/Add9' */
  rtb_Add4_j += rtb_Add7;

  /* Saturate: '<S29>/Saturation1' */
  if (rtb_Add < 0.0F) {
    rtb_Add = 0.0F;
  }

  /* End of Saturate: '<S29>/Saturation1' */

  /* Sum: '<S29>/Add1' */
  rtb_deltafalllimit_om = rtb_Add4_j - rtb_Add;

  /* RelationalOperator: '<S29>/Relational Operator7' incorporates:
   *  Constant: '<S29>/Cal_DeltaV_mps'
   */
  rtb_LogicalOperator2 = (rtb_deltafalllimit_om < 0.0F);

  /* Logic: '<S68>/Logical Operator4' */
  rtb_Compare_am = ((!rtb_Compare_am) && (!rtb_LogicalOperator2));

  /* Logic: '<S29>/Logical Operator2' */
  rtb_LogicalOperator2 = !rtb_LogicalOperator2;

  /* UnitDelay: '<S29>/Unit Delay4' */
  elapseTime = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l;

  /* RelationalOperator: '<S29>/Relational Operator8' incorporates:
   *  Constant: '<S29>/Cal_DeltaV_mps1'
   */
  rtb_ignition = (elapseTime > 235.0);

  /* Logic: '<S29>/Logical Operator1' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 && rtb_ignition);

  /* Switch: '<S69>/Switch6' incorporates:
   *  Constant: '<S69>/Reset'
   */
  if (rtb_LogicalOperator2) {
    /* Sum: '<S69>/Add10' incorporates:
     *  Constant: '<S69>/Steptime'
     */
    rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h + 0.01F;
  } else {
    rtb_Add4_j = 0.0F;
  }

  /* End of Switch: '<S69>/Switch6' */

  /* MinMax: '<S69>/Min' incorporates:
   *  Constant: '<S29>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h = fminf(rtb_Add4_j, 0.1F);

  /* RelationalOperator: '<S69>/Relational Operator9' incorporates:
   *  Constant: '<S29>/ResetDelay'
   *  UnitDelay: '<S69>/Unit Delay1'
   */
  rtb_LogicalOperator2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h >=
    0.1F);

  /* UnitDelay: '<S29>/Unit Delay3' */
  rtb_ignition = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i;

  /* Logic: '<S29>/Logical Operator3' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_ignition);

  /* Logic: '<S68>/Logical Operator5' incorporates:
   *  UnitDelay: '<S68>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp = ((!rtb_Compare_am) && (
    !rtb_LogicalOperator2));

  /* UnitDelay: '<S10>/Unit Delay3' */
  rtb_Compare_am = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_l;

  /* UnitDelay: '<S46>/Delay Input2'
   *
   * Block description for '<S46>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_d;

  /* SampleTimeMath: '<S46>/sample time'
   *
   * About '<S46>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S46>/delta rise limit' */
  rtb_Add10 = (real32_T)(1000.0 * elapseTime);

  /* MATLAB Function: '<S10>/Wtarget' incorporates:
   *  Constant: '<S10>/Constant14'
   *  Constant: '<S10>/Constant15'
   *  Constant: '<S10>/Constant16'
   *  Constant: '<S10>/Constant17'
   *  Constant: '<S10>/Constant18'
   *  Constant: '<S10>/Constant19'
   */
  rtb_Switch2_mn = VehVxEst_mps * (real32_T)rtb_Yk1 / (340.0F * VehVxEst_mps *
    VehVxEst_mps * 15.5799866F / 460.0F / 440.0F / 1.592F / 2.0F + 1.592F);
  if (rtb_Switch2_mn < 0.0F) {
    rtb_MaxWhlSpd_mps_n = -1.0F;
  } else if (rtb_Switch2_mn > 0.0F) {
    rtb_MaxWhlSpd_mps_n = 1.0F;
  } else if (rtb_Switch2_mn == 0.0F) {
    rtb_MaxWhlSpd_mps_n = 0.0F;
  } else {
    rtb_MaxWhlSpd_mps_n = (rtNaNF);
  }

  rtb_Switch2_mn = fminf((rtInfF), fabsf(rtb_Switch2_mn)) * rtb_MaxWhlSpd_mps_n;

  /* End of MATLAB Function: '<S10>/Wtarget' */

  /* Sum: '<S46>/Difference Inputs1'
   *
   * Block description for '<S46>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_mn -= rtb_Add4_j;

  /* RelationalOperator: '<S61>/LowerRelop1' */
  rtb_LogicalOperator2 = (rtb_Switch2_mn > rtb_Add10);

  /* Switch: '<S61>/Switch2' */
  if (!rtb_LogicalOperator2) {
    /* Product: '<S46>/delta fall limit' */
    rtb_Add10 = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S61>/UpperRelop' */
    rtb_ignition = (rtb_Switch2_mn < rtb_Add10);

    /* Switch: '<S61>/Switch' */
    if (rtb_ignition) {
      rtb_Switch2_mn = rtb_Add10;
    }

    /* End of Switch: '<S61>/Switch' */
    rtb_Add10 = rtb_Switch2_mn;
  }

  /* End of Switch: '<S61>/Switch2' */

  /* Sum: '<S46>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S46>/Delay Input2'
   *
   * Block description for '<S46>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S46>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_d = rtb_Add10 + rtb_Add4_j;

  /* Sum: '<S10>/Add' incorporates:
   *  UnitDelay: '<S46>/Delay Input2'
   *
   * Block description for '<S46>/Delay Input2':
   *
   *  Store in Global RAM
   */
  elapseTime = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_d - rtb_UkYk1;

  /* Abs: '<S10>/Abs' */
  WhlSpdRR_mps = fabs(elapseTime);

  /* RelationalOperator: '<S33>/Compare' incorporates:
   *  Constant: '<S33>/Constant'
   */
  rtb_LogicalOperator2 = (WhlSpdRR_mps > 5.0);

  /* Abs: '<S10>/Abs1' */
  WhlSpdRR_mps = fabs(rtb_UkYk1);

  /* RelationalOperator: '<S34>/Compare' incorporates:
   *  Constant: '<S34>/Constant'
   */
  rtb_ignition = (WhlSpdRR_mps > 5.0);

  /* RelationalOperator: '<S35>/Compare' incorporates:
   *  Constant: '<S35>/Constant'
   */
  rtb_Compare = (VehVxEst_mps > 3.0F);

  /* UnitDelay: '<S7>/Unit Delay' */
  WhlSpdRR_mps = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_n;

  /* Logic: '<S10>/AND' */
  rtb_LowerRelop1_b = (rtb_LogicalOperator2 && rtb_ignition && rtb_Compare &&
                       (WhlSpdRR_mps != 0.0));

  /* Logic: '<S10>/Logical Operator4' */
  rtb_Compare_am = ((!rtb_Compare_am) && (!rtb_LowerRelop1_b));

  /* Abs: '<S10>/Abs2' */
  elapseTime = fabs(elapseTime);

  /* RelationalOperator: '<S36>/Compare' incorporates:
   *  Constant: '<S36>/Constant'
   */
  rtb_LowerRelop1_b = (elapseTime < 10.0);

  /* Switch: '<S10>/Switch6' incorporates:
   *  Constant: '<S10>/Reset'
   */
  if (rtb_LowerRelop1_b) {
    /* Sum: '<S10>/Add10' incorporates:
     *  Constant: '<S10>/Steptime'
     */
    rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_n + 0.01F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S10>/Switch6' */

  /* MinMax: '<S10>/Min' incorporates:
   *  Constant: '<S10>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_n = fminf(rtb_Add10, 0.5F);

  /* RelationalOperator: '<S10>/Relational Operator9' incorporates:
   *  Constant: '<S10>/ResetDelay'
   *  UnitDelay: '<S10>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_n >=
                       0.5F);

  /* Logic: '<S10>/Logical Operator5' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_l = ((!rtb_Compare_am) &&
    (!rtb_LowerRelop1_b));

  /* Chart: '<S7>/Chart' */
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c7_VehCtrlMdel240926_
      == 0U) {
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c7_VehCtrlMdel240926_ =
      1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_B = 3U;
    VehCtrlMdel240926_2018b_amks_DW.b = rtb_deltafalllimit_iz *
      rtb_deltafalllimit_iz + rtb_deltafalllimit_i4 * rtb_deltafalllimit_i4;
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
      rtb_ignition = ((rtb_UkYk1 >= 50.0) || (rtb_deltafalllimit_i4 >= 5.0) ||
                      (VehCtrlMdel240926_2018b_amks_DW.b >= 5.0));
      if (rtb_ignition) {
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

    if (fabs(rtb_deltafalllimit_i4) > 0.5) {
      elapseTime = atan(rtb_deltafalllimit_iz / rtb_deltafalllimit_i4) * 180.0 /
        3.1415926535897931;
    } else {
      elapseTime = 0.0;
    }

    VehCtrlMdel240926_2018b_amks_DW.b = rtb_deltafalllimit_iz *
      rtb_deltafalllimit_iz + rtb_deltafalllimit_i4 * rtb_deltafalllimit_i4;
    VehCtrlMdel240926_2018b_amks_DW.b = sqrt(VehCtrlMdel240926_2018b_amks_DW.b);
    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_C) {
     case VehCtrlMdel240926_2018b_am_IN_B:
      rtb_ignition = ((!(VehCtrlMdel240926_2018b_amks_DW.DYC_flag != 0.0)) ||
                      (rtb_UkYk1 > 30.0) || (elapseTime > 30.0) || (elapseTime >
        -30.0) || (VehCtrlMdel240926_2018b_amks_DW.b > 3.0));
      if (rtb_ignition) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_C = 3U;
      }
      break;

     case VehCtrlMdel240926_2018b_am_IN_C:
      break;

     default:
      /* case IN_F_TVD_TCS_STATE: */
      rtb_ignition = ((!(VehCtrlMdel240926_2018b_amks_DW.DYC_flag != 0.0)) ||
                      (rtb_UkYk1 > 30.0) || (elapseTime > 30.0) || (elapseTime >
        -30.0) || (VehCtrlMdel240926_2018b_amks_DW.b > 3.0));
      if (rtb_ignition) {
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

  /* Switch: '<S31>/Switch6' incorporates:
   *  Constant: '<S31>/Verror_Reset'
   *  UnitDelay: '<S88>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e) {
    rtb_Add10 = rtb_deltafalllimit_n;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S31>/Switch6' */

  /* Product: '<S31>/Product' incorporates:
   *  Constant: '<S31>/P_Gain'
   */
  rtb_Switch2_mn = rtb_Add10 * 40.0F;

  /* Sum: '<S31>/Add11' */
  WhlSpdRR_mps = trq - rtb_Switch2_mn;

  /* UnitDelay: '<S31>/Unit Delay5' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_l;

  /* Product: '<S31>/Product2' */
  rtb_Add4_j *= rtb_deltafalllimit_n;

  /* RelationalOperator: '<S83>/Compare' incorporates:
   *  Constant: '<S83>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Add4_j <= 0.0F);

  /* UnitDelay: '<S31>/Unit Delay' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b;

  /* Switch: '<S31>/Switch3' incorporates:
   *  Constant: '<S31>/Verror_Reset1'
   */
  if (rtb_LowerRelop1_b) {
    rtb_Add4_j = 0.0F;
  }

  /* End of Switch: '<S31>/Switch3' */

  /* Sum: '<S31>/Add2' */
  rtb_Add4_j += rtb_Add10;

  /* Saturate: '<S31>/Saturation2' */
  if (rtb_Add4_j > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = 400.0F;
  } else if (rtb_Add4_j < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = rtb_Add4_j;
  }

  /* End of Saturate: '<S31>/Saturation2' */

  /* RelationalOperator: '<S93>/Compare' incorporates:
   *  UnitDelay: '<S88>/Unit Delay1'
   */
  rtb_ignition = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e;

  /* UnitDelay: '<S87>/Delay Input1'
   *
   * Block description for '<S87>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE;

  /* RelationalOperator: '<S87>/FixPt Relational Operator' */
  rtb_LowerRelop1_b = ((int32_T)rtb_ignition > (int32_T)rtb_LowerRelop1_b);

  /* Switch: '<S31>/Switch' incorporates:
   *  Constant: '<S31>/Integr_StartPoint'
   */
  if (rtb_LowerRelop1_b) {
    /* Sum: '<S31>/Add4' */
    elapseTime = trq - VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g;
  } else {
    elapseTime = 0.0;
  }

  /* End of Switch: '<S31>/Switch' */

  /* Lookup_n-D: '<S31>/VehicleStableTarget_mps' */
  rtb_Add10 = look1_iflf_binlc(rtb_Ax, VehCtrlMdel240926_2018b__ConstP.pooled35,
    VehCtrlMdel240926_2018b__ConstP.pooled51, 3U);

  /* Sum: '<S31>/Add5' */
  rtb_Add10 += rtb_Ax;

  /* Sum: '<S31>/Add10' */
  rtb_Add10 = rtb_VxIMU_est - rtb_Add10;

  /* RelationalOperator: '<S31>/Relational Operator' incorporates:
   *  Constant: '<S31>/Verror'
   */
  rtb_LowerRelop1_b = (rtb_Add10 < 0.0F);

  /* Logic: '<S31>/Logical Operator4' incorporates:
   *  UnitDelay: '<S88>/Unit Delay1'
   */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b &&
                       VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e);

  /* Switch: '<S31>/Switch1' incorporates:
   *  Constant: '<S31>/Trq_IReset'
   *  Constant: '<S31>/Trq_I_FF'
   */
  if (rtb_LowerRelop1_b) {
    rtb_Add10 = 20.0F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S31>/Switch1' */

  /* Sum: '<S31>/Add6' incorporates:
   *  UnitDelay: '<S31>/Unit Delay'
   */
  rtb_UkYk1 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b + elapseTime)
    + rtb_Add10;

  /* Product: '<S31>/Product1' */
  rtb_UkYk1 *= 10.0;

  /* RelationalOperator: '<S90>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > WhlSpdRR_mps);

  /* Switch: '<S90>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Gain: '<S31>/Gain3' */
    rtb_Add10 = -rtb_Switch2_mn;

    /* RelationalOperator: '<S90>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_UkYk1 < rtb_Add10);

    /* Switch: '<S90>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_UkYk1 = rtb_Add10;
    }

    /* End of Switch: '<S90>/Switch' */
    WhlSpdRR_mps = rtb_UkYk1;
  }

  /* End of Switch: '<S90>/Switch2' */

  /* Sum: '<S31>/Add7' incorporates:
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b = rtb_Switch2_mn +
    WhlSpdRR_mps;

  /* Lookup_n-D: '<S31>/VehicleStableTarget_mps1' */
  rtb_Add10 = look1_iflf_binlc(rtb_Ax, VehCtrlMdel240926_2018b__ConstP.pooled35,
    VehCtrlMdel240926_2018b__ConstP.pooled51, 3U);

  /* Sum: '<S31>/Add13' */
  rtb_Ax += rtb_Add10;

  /* Sum: '<S31>/Add12' */
  rtb_Ax = rtb_VxIMU_est - rtb_Ax;

  /* RelationalOperator: '<S31>/Relational Operator1' incorporates:
   *  Constant: '<S31>/Verror1'
   */
  rtb_LowerRelop1_b = (rtb_Ax < 0.0F);

  /* RelationalOperator: '<S31>/Relational Operator2' incorporates:
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b >= trq);

  /* RelationalOperator: '<S85>/Compare' incorporates:
   *  Constant: '<S85>/Constant'
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b <= 0.01);

  /* Logic: '<S31>/OR' */
  rtb_Compare = (rtb_Compare || rtb_Compare_am);

  /* Logic: '<S31>/Logical Operator5' incorporates:
   *  UnitDelay: '<S31>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_a = (rtb_LowerRelop1_b &&
    rtb_Compare);

  /* Switch: '<S31>/Switch2' incorporates:
   *  Switch: '<S31>/Switch7'
   *  UnitDelay: '<S31>/Unit Delay3'
   *  UnitDelay: '<S88>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e) {
    /* RelationalOperator: '<S91>/LowerRelop1' incorporates:
     *  Constant: '<S31>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S31>/Unit Delay4'
     */
    rtb_LogicalOperator2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b >
      235.0);

    /* Switch: '<S91>/Switch2' incorporates:
     *  Constant: '<S31>/TCS_TrqRequest_Max2'
     */
    if (rtb_LogicalOperator2) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 235.0;
    } else {
      /* RelationalOperator: '<S91>/UpperRelop' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S31>/Unit Delay4'
       */
      rtb_LogicalOperator2 =
        (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b < 0.0);

      /* Switch: '<S91>/Switch' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S31>/Unit Delay4'
       */
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b;
      }

      /* End of Switch: '<S91>/Switch' */
    }

    /* End of Switch: '<S91>/Switch2' */

    /* RelationalOperator: '<S92>/LowerRelop1' */
    rtb_LogicalOperator2 = (trq >
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g);

    /* Switch: '<S92>/Switch2' */
    if (!rtb_LogicalOperator2) {
      /* RelationalOperator: '<S92>/UpperRelop' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min1'
       */
      rtb_LogicalOperator2 = (trq < 0.0);

      /* Switch: '<S92>/Switch' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min1'
       */
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = trq;
      }

      /* End of Switch: '<S92>/Switch' */
    }

    /* End of Switch: '<S92>/Switch2' */
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_a) {
      /* Switch: '<S31>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = trq;
    }
  }

  /* End of Switch: '<S31>/Switch2' */

  /* Logic: '<S7>/Logical Operator2' */
  rtb_LowerRelop1_b = !VehCtrlMdel240926_2018b_amksp_B.VehReady;

  /* Logic: '<S7>/Logical Operator3' */
  rtb_Compare = (rtb_LowerRelop1_b || (Trq_CUT != 0.0));

  /* Logic: '<S7>/Logical Operator1' */
  TrqR_cmd_raw = ((EMRAX_Trq_CUT != 0.0) || rtb_Compare);

  /* Switch: '<S7>/Switch2' incorporates:
   *  Constant: '<S7>/Constant4'
   *  Switch: '<S7>/Switch5'
   */
  if (TrqR_cmd_raw) {
    rtb_Add4_j = 0.0F;
  } else {
    if (VehCtrlMdel240926_2018b_amksp_B.TCSR_Enable_OUT != 0.0) {
      /* Abs: '<S31>/Abs' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_Yk1 = fabs(rtb_Yk1);

      /* RelationalOperator: '<S84>/Compare' incorporates:
       *  Constant: '<S84>/Constant'
       *  Switch: '<S7>/Switch5'
       */
      rtb_LogicalOperator2 = (rtb_Yk1 <= 20.0);

      /* RelationalOperator: '<S86>/Compare' incorporates:
       *  Constant: '<S86>/Constant'
       *  Switch: '<S7>/Switch5'
       */
      rtb_LogicalOperator7 = (rtb_VxIMU_est > 0.0F);

      /* Logic: '<S31>/Logical Operator6' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_LogicalOperator7 = (rtb_LogicalOperator7 && rtb_LogicalOperator2);

      /* Switch: '<S31>/Switch5' incorporates:
       *  Switch: '<S7>/Switch5'
       *  UnitDelay: '<S31>/Unit Delay2'
       */
      if (rtb_LogicalOperator7) {
        rtb_Yk1 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g;
      } else {
        rtb_Yk1 = trq;
      }

      /* End of Switch: '<S31>/Switch5' */
    } else {
      /* Gain: '<S7>/Gain' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_Add10 = 0.8F * rtb_Add6_p;

      /* Sum: '<S28>/Add1' incorporates:
       *  Constant: '<S28>/RPM_min'
       *  Switch: '<S7>/Switch5'
       */
      rtb_UkYk1 = RPM + 10.0;

      /* MinMax: '<S28>/Max' incorporates:
       *  Constant: '<S28>/RPM_min1'
       *  Switch: '<S7>/Switch5'
       */
      rtb_Yk1 = fmax(rtb_UkYk1, 1.0);

      /* Sum: '<S28>/Add' incorporates:
       *  Constant: '<S28>/Constant3'
       *  Switch: '<S7>/Switch5'
       */
      rtb_UkYk1 = 1.0 - WhlSpdFR;

      /* Product: '<S28>/Product5' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_UkYk1 *= 63.0;

      /* Product: '<S28>/Product' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_UkYk1 *= 9550.0;

      /* Product: '<S28>/Divide' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_UkYk1 /= rtb_Yk1;

      /* RelationalOperator: '<S25>/LowerRelop1' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_LogicalOperator2 = (rtb_Add10 > rtb_UkYk1);

      /* Switch: '<S25>/Switch2' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      if (rtb_LogicalOperator2) {
        rtb_Add10 = (real32_T)rtb_UkYk1;
      } else {
        /* RelationalOperator: '<S25>/UpperRelop' incorporates:
         *  Constant: '<S7>/Constant15'
         */
        rtb_LogicalOperator2 = (rtb_Add10 < 0.0F);

        /* Switch: '<S25>/Switch' incorporates:
         *  Constant: '<S7>/Constant15'
         */
        if (rtb_LogicalOperator2) {
          rtb_Add10 = 0.0F;
        }

        /* End of Switch: '<S25>/Switch' */
      }

      /* End of Switch: '<S25>/Switch2' */

      /* Switch: '<S7>/Switch5' */
      rtb_Yk1 = rtb_Add10;
    }

    /* Gain: '<S7>/Gain6' */
    rtb_Yk1 *= 12.5;

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefRear' */
    rtb_Add4_j = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefRear_bp01Dat,
      VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefRear_tableDa, 1U);

    /* RelationalOperator: '<S22>/LowerRelop1' */
    rtb_LogicalOperator2 = (rtb_Yk1 > rtb_Add4_j);

    /* Switch: '<S22>/Switch2' */
    if (!rtb_LogicalOperator2) {
      /* RelationalOperator: '<S22>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      rtb_LogicalOperator2 = (rtb_Yk1 < 0.0);

      /* Switch: '<S22>/Switch' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      if (rtb_LogicalOperator2) {
        rtb_Add4_j = 0.0F;
      } else {
        rtb_Add4_j = (real32_T)rtb_Yk1;
      }

      /* End of Switch: '<S22>/Switch' */
    }

    /* End of Switch: '<S22>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch2' */

  /* UnitDelay: '<S19>/Delay Input2'
   *
   * Block description for '<S19>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd;

  /* Sum: '<S19>/Difference Inputs1'
   *
   * Block description for '<S19>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Ax = rtb_Add4_j - rtb_Add10;

  /* SampleTimeMath: '<S19>/sample time'
   *
   * About '<S19>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S19>/delta rise limit' */
  rtb_Add4_j = (real32_T)(25000.0 * elapseTime);

  /* RelationalOperator: '<S62>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Ax > rtb_Add4_j);

  /* Switch: '<S62>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S19>/delta fall limit' */
    rtb_Add4_j = (real32_T)(-25000.0 * elapseTime);

    /* RelationalOperator: '<S62>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Ax < rtb_Add4_j);

    /* Switch: '<S62>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Ax = rtb_Add4_j;
    }

    /* End of Switch: '<S62>/Switch' */
    rtb_Add4_j = rtb_Ax;
  }

  /* End of Switch: '<S62>/Switch2' */

  /* Saturate: '<S7>/Saturation1' incorporates:
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd = rtb_Add4_j + rtb_Add10;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd > 1000.0F) {
    EmraxTrqR_cmd = 1000.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd < 0.0F) {
    EmraxTrqR_cmd = 0.0F;
  } else {
    EmraxTrqR_cmd = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd;
  }

  /* End of Saturate: '<S7>/Saturation1' */

  /* Logic: '<S7>/Logical Operator5' */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amksp_B.MCFR_TorqueOn &&
                       VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueOn);

  /* Logic: '<S7>/Logical Operator6' */
  TroqueOn = !rtb_LowerRelop1_b;

  /* Logic: '<S7>/Logical Operator4' */
  Trq_CUT_final = (TroqueOn || (AMK_Trq_CUT != 0.0) || rtb_Compare ||
                   TrqR_cmd_raw);

  /* Switch: '<S30>/Switch6' incorporates:
   *  Constant: '<S30>/Verror_Reset'
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl) {
    rtb_Add10 = rtb_Acc_POS;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S30>/Switch6' */

  /* Product: '<S30>/Product' incorporates:
   *  Constant: '<S30>/P_Gain'
   */
  rtb_Ax = rtb_Add10 * 40.0F;

  /* Sum: '<S30>/Add11' */
  rtb_UkYk1 = MCFR_ActualTorque - rtb_Ax;

  /* UnitDelay: '<S30>/Unit Delay5' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* Product: '<S30>/Product2' */
  rtb_Add4_j *= rtb_Acc_POS;

  /* RelationalOperator: '<S74>/Compare' incorporates:
   *  Constant: '<S74>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Add4_j <= 0.0F);

  /* UnitDelay: '<S30>/Unit Delay' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f;

  /* Switch: '<S30>/Switch3' incorporates:
   *  Constant: '<S30>/Verror_Reset1'
   */
  if (rtb_LowerRelop1_b) {
    rtb_Add4_j = 0.0F;
  }

  /* End of Switch: '<S30>/Switch3' */

  /* Sum: '<S30>/Add2' */
  rtb_Add4_j += rtb_Add10;

  /* Saturate: '<S30>/Saturation2' */
  if (rtb_Add4_j > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f = 400.0F;
  } else if (rtb_Add4_j < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f = rtb_Add4_j;
  }

  /* End of Saturate: '<S30>/Saturation2' */

  /* RelationalOperator: '<S82>/Compare' incorporates:
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  rtb_LogicalOperator2 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl;

  /* UnitDelay: '<S76>/Delay Input1'
   *
   * Block description for '<S76>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_j;

  /* RelationalOperator: '<S76>/FixPt Relational Operator' */
  rtb_LowerRelop1_b = ((int32_T)rtb_LogicalOperator2 > (int32_T)
                       rtb_LowerRelop1_b);

  /* Switch: '<S30>/Switch' incorporates:
   *  Constant: '<S30>/Integr_StartPoint'
   */
  if (rtb_LowerRelop1_b) {
    /* Sum: '<S30>/Add4' */
    WhlSpdRR_mps = MCFR_ActualTorque -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_f;
  } else {
    WhlSpdRR_mps = 0.0;
  }

  /* End of Switch: '<S30>/Switch' */

  /* Lookup_n-D: '<S30>/VehicleStableTarget_mps' */
  rtb_Add10 = look1_iflf_binlc(rtb_Switch2_b0,
    VehCtrlMdel240926_2018b__ConstP.pooled35,
    VehCtrlMdel240926_2018b__ConstP.pooled51, 3U);

  /* Sum: '<S30>/Add5' */
  rtb_Add10 += rtb_Switch2_b0;

  /* Sum: '<S30>/Add10' */
  rtb_Add10 = rtb_Divide_g - rtb_Add10;

  /* RelationalOperator: '<S30>/Relational Operator' incorporates:
   *  Constant: '<S30>/Verror'
   */
  rtb_LowerRelop1_b = (rtb_Add10 < 0.0F);

  /* Logic: '<S30>/Logical Operator4' incorporates:
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b &&
                       VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl);

  /* Switch: '<S30>/Switch1' incorporates:
   *  Constant: '<S30>/Trq_IReset'
   *  Constant: '<S30>/Trq_I_FF'
   */
  if (rtb_LowerRelop1_b) {
    rtb_Add10 = 20.0F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S30>/Switch1' */

  /* Sum: '<S30>/Add6' incorporates:
   *  UnitDelay: '<S30>/Unit Delay'
   */
  elapseTime = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f +
                WhlSpdRR_mps) + rtb_Add10;

  /* Product: '<S30>/Product1' */
  rtb_Yk1 = elapseTime * 10.0;

  /* RelationalOperator: '<S79>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Yk1 > rtb_UkYk1);

  /* Switch: '<S79>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Gain: '<S30>/Gain3' */
    rtb_Add10 = -rtb_Ax;

    /* RelationalOperator: '<S79>/UpperRelop' */
    rtb_Compare = (rtb_Yk1 < rtb_Add10);

    /* Switch: '<S79>/Switch' */
    if (rtb_Compare) {
      rtb_Yk1 = rtb_Add10;
    }

    /* End of Switch: '<S79>/Switch' */
    rtb_UkYk1 = rtb_Yk1;
  }

  /* End of Switch: '<S79>/Switch2' */

  /* Sum: '<S30>/Add7' incorporates:
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE = rtb_Ax + rtb_UkYk1;

  /* Lookup_n-D: '<S30>/VehicleStableTarget_mps1' */
  rtb_Add10 = look1_iflf_binlc(rtb_Switch2_b0,
    VehCtrlMdel240926_2018b__ConstP.pooled35,
    VehCtrlMdel240926_2018b__ConstP.pooled51, 3U);

  /* Sum: '<S30>/Add13' */
  rtb_Switch2_b0 += rtb_Add10;

  /* Sum: '<S30>/Add12' */
  rtb_Divide_g -= rtb_Switch2_b0;

  /* RelationalOperator: '<S30>/Relational Operator1' incorporates:
   *  Constant: '<S30>/Verror1'
   */
  rtb_LowerRelop1_b = (rtb_Divide_g < 0.0F);

  /* RelationalOperator: '<S30>/Relational Operator2' incorporates:
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE >=
                 MCFR_ActualTorque);

  /* RelationalOperator: '<S75>/Compare' incorporates:
   *  Constant: '<S75>/Constant'
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE <= 0.01);

  /* Logic: '<S30>/OR' */
  rtb_Compare = (rtb_Compare || rtb_Compare_am);

  /* Logic: '<S30>/Logical Operator5' incorporates:
   *  UnitDelay: '<S30>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e = (rtb_LowerRelop1_b &&
    rtb_Compare);

  /* Switch: '<S30>/Switch2' incorporates:
   *  Switch: '<S30>/Switch7'
   *  UnitDelay: '<S30>/Unit Delay3'
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl) {
    /* RelationalOperator: '<S80>/LowerRelop1' incorporates:
     *  Constant: '<S30>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S30>/Unit Delay4'
     */
    rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE > 235.0);

    /* Switch: '<S80>/Switch2' incorporates:
     *  Constant: '<S30>/TCS_TrqRequest_Max2'
     */
    if (rtb_Compare) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j = 235.0;
    } else {
      /* RelationalOperator: '<S80>/UpperRelop' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S30>/Unit Delay4'
       */
      rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE < 0.0);

      /* Switch: '<S80>/Switch' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S30>/Unit Delay4'
       */
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE;
      }

      /* End of Switch: '<S80>/Switch' */
    }

    /* End of Switch: '<S80>/Switch2' */

    /* RelationalOperator: '<S81>/LowerRelop1' */
    rtb_Compare = (MCFR_ActualTorque >
                   VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j);

    /* Switch: '<S81>/Switch2' */
    if (!rtb_Compare) {
      /* RelationalOperator: '<S81>/UpperRelop' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min1'
       */
      rtb_Compare = (MCFR_ActualTorque < 0.0);

      /* Switch: '<S81>/Switch' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min1'
       */
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j = MCFR_ActualTorque;
      }

      /* End of Switch: '<S81>/Switch' */
    }

    /* End of Switch: '<S81>/Switch2' */
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e) {
      /* Switch: '<S30>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j = MCFR_ActualTorque;
    }
  }

  /* End of Switch: '<S30>/Switch2' */

  /* Sum: '<S28>/Add4' incorporates:
   *  Constant: '<S28>/Constant'
   */
  rtb_Yk1 = 1.0 - WhlSpdFL;

  /* Switch: '<S7>/Switch6' incorporates:
   *  UnitDelay: '<S30>/Unit Delay2'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.TCSF_Enable_OUT != 0.0) {
    rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j;
  } else {
    /* Gain: '<S7>/Gain7' */
    rtb_Divide_g = 0.1F * rtb_Add6_p;

    /* Sum: '<S28>/Add2' incorporates:
     *  Constant: '<S28>/RPM_min2'
     */
    rtb_deltafalllimit_iz = MCFR_ActualVelocity + 10.0;

    /* MinMax: '<S28>/Max1' incorporates:
     *  Constant: '<S28>/RPM_min3'
     */
    rtb_UkYk1 = fmax(rtb_deltafalllimit_iz, 1.0);

    /* Product: '<S28>/Product4' */
    rtb_deltafalllimit_iz = 7.0 * rtb_Yk1;

    /* Product: '<S28>/Product1' */
    rtb_deltafalllimit_iz *= 9550.0;

    /* Product: '<S28>/Divide1' */
    rtb_deltafalllimit_iz /= rtb_UkYk1;

    /* RelationalOperator: '<S26>/LowerRelop1' */
    rtb_Compare = (rtb_Divide_g > rtb_deltafalllimit_iz);

    /* Switch: '<S26>/Switch2' */
    if (rtb_Compare) {
      rtb_Divide_g = (real32_T)rtb_deltafalllimit_iz;
    } else {
      /* RelationalOperator: '<S26>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant15'
       */
      rtb_Compare = (rtb_Divide_g < 0.0F);

      /* Switch: '<S26>/Switch' incorporates:
       *  Constant: '<S7>/Constant15'
       */
      if (rtb_Compare) {
        rtb_Divide_g = 0.0F;
      }

      /* End of Switch: '<S26>/Switch' */
    }

    /* End of Switch: '<S26>/Switch2' */
    rtb_UkYk1 = rtb_Divide_g;
  }

  /* End of Switch: '<S7>/Switch6' */

  /* Lookup_n-D: '<S7>/BrakeCompensateCoefFront1' */
  rtb_Add10 = look1_iflf_binlc((real32_T)Brk_F,
    VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefFront1_bp01D,
    VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefFront1_table, 1U);

  /* RelationalOperator: '<S23>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Add10);

  /* Switch: '<S23>/Switch2' */
  if (rtb_LowerRelop1_b) {
    rtb_Add4_j = rtb_Add10;
  } else {
    /* RelationalOperator: '<S23>/UpperRelop' incorporates:
     *  Constant: '<S7>/Constant7'
     */
    rtb_Compare = (rtb_UkYk1 < 0.0);

    /* Switch: '<S23>/Switch' incorporates:
     *  Constant: '<S7>/Constant7'
     */
    if (rtb_Compare) {
      rtb_Add4_j = 0.0F;
    } else {
      rtb_Add4_j = (real32_T)rtb_UkYk1;
    }

    /* End of Switch: '<S23>/Switch' */
  }

  /* End of Switch: '<S23>/Switch2' */

  /* Switch: '<S7>/Switch3' */
  rtb_Divide_g = rtb_Add4_j;

  /* UnitDelay: '<S20>/Delay Input2'
   *
   * Block description for '<S20>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn;

  /* Sum: '<S20>/Difference Inputs1'
   *
   * Block description for '<S20>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_Divide_g - rtb_Add4_j;

  /* SampleTimeMath: '<S20>/sample time'
   *
   * About '<S20>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S20>/delta rise limit' */
  rtb_Divide_g = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S63>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Switch2_b0 > rtb_Divide_g);

  /* Switch: '<S63>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S20>/delta fall limit' */
    rtb_Divide_g = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S63>/UpperRelop' */
    rtb_Compare = (rtb_Switch2_b0 < rtb_Divide_g);

    /* Switch: '<S63>/Switch' */
    if (rtb_Compare) {
      rtb_Switch2_b0 = rtb_Divide_g;
    }

    /* End of Switch: '<S63>/Switch' */
    rtb_Divide_g = rtb_Switch2_b0;
  }

  /* End of Switch: '<S63>/Switch2' */

  /* Saturate: '<S7>/Saturation2' incorporates:
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn = rtb_Divide_g +
    rtb_Add4_j;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn > 2.0F) {
    AMKTrqFR_cmd = 2.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn < -2.0F) {
    AMKTrqFR_cmd = -2.0F;
  } else {
    AMKTrqFR_cmd = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn;
  }

  /* End of Saturate: '<S7>/Saturation2' */

  /* Gain: '<S7>/Gain1' */
  VehCtrlMdel240926_2018b_amksp_B.TrqFR_cmd = -AMKTrqFR_cmd;

  /* Switch: '<S29>/Switch6' incorporates:
   *  Constant: '<S29>/Verror_Reset'
   *  UnitDelay: '<S68>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp) {
    rtb_Add4_j = rtb_deltafalllimit_om;
  } else {
    rtb_Add4_j = 0.0F;
  }

  /* End of Switch: '<S29>/Switch6' */

  /* Product: '<S29>/Product' incorporates:
   *  Constant: '<S29>/P_Gain'
   */
  rtb_Switch2_b0 = rtb_Add4_j * 40.0F;

  /* Sum: '<S29>/Add11' */
  rtb_UkYk1 = MCFL_ActualTorque - rtb_Switch2_b0;

  /* UnitDelay: '<S29>/Unit Delay5' */
  rtb_Divide_g = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_ip;

  /* Product: '<S29>/Product2' */
  rtb_Divide_g *= rtb_deltafalllimit_om;

  /* RelationalOperator: '<S65>/Compare' incorporates:
   *  Constant: '<S65>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Divide_g <= 0.0F);

  /* UnitDelay: '<S29>/Unit Delay' */
  rtb_Divide_g = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr;

  /* Switch: '<S29>/Switch3' incorporates:
   *  Constant: '<S29>/Verror_Reset1'
   */
  if (rtb_LowerRelop1_b) {
    rtb_Divide_g = 0.0F;
  }

  /* End of Switch: '<S29>/Switch3' */

  /* Sum: '<S29>/Add2' */
  rtb_Divide_g += rtb_Add4_j;

  /* Saturate: '<S29>/Saturation2' */
  if (rtb_Divide_g > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = 400.0F;
  } else if (rtb_Divide_g < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = rtb_Divide_g;
  }

  /* End of Saturate: '<S29>/Saturation2' */

  /* RelationalOperator: '<S73>/Compare' incorporates:
   *  UnitDelay: '<S68>/Unit Delay1'
   */
  rtb_LogicalOperator7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp;

  /* UnitDelay: '<S67>/Delay Input1'
   *
   * Block description for '<S67>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_b;

  /* RelationalOperator: '<S67>/FixPt Relational Operator' */
  rtb_LowerRelop1_b = ((int32_T)rtb_LogicalOperator7 > (int32_T)
                       rtb_LowerRelop1_b);

  /* Switch: '<S29>/Switch' incorporates:
   *  Constant: '<S29>/Integr_StartPoint'
   */
  if (rtb_LowerRelop1_b) {
    /* Sum: '<S29>/Add4' */
    WhlSpdRR_mps = MCFL_ActualTorque -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g4;
  } else {
    WhlSpdRR_mps = 0.0;
  }

  /* End of Switch: '<S29>/Switch' */

  /* Lookup_n-D: '<S29>/VehicleStableTarget_mps' */
  rtb_Add4_j = look1_iflf_binlc(rtb_Add7,
    VehCtrlMdel240926_2018b__ConstP.pooled35,
    VehCtrlMdel240926_2018b__ConstP.pooled51, 3U);

  /* Sum: '<S29>/Add5' */
  rtb_Add4_j += rtb_Add7;

  /* Sum: '<S29>/Add10' */
  rtb_Add4_j = rtb_Add - rtb_Add4_j;

  /* RelationalOperator: '<S29>/Relational Operator' incorporates:
   *  Constant: '<S29>/Verror'
   */
  rtb_LowerRelop1_b = (rtb_Add4_j < 0.0F);

  /* Logic: '<S29>/Logical Operator4' incorporates:
   *  UnitDelay: '<S68>/Unit Delay1'
   */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b &&
                       VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp);

  /* Switch: '<S29>/Switch1' incorporates:
   *  Constant: '<S29>/Trq_IReset'
   *  Constant: '<S29>/Trq_I_FF'
   */
  if (rtb_LowerRelop1_b) {
    rtb_Add4_j = 20.0F;
  } else {
    rtb_Add4_j = 0.0F;
  }

  /* End of Switch: '<S29>/Switch1' */

  /* Sum: '<S29>/Add6' incorporates:
   *  UnitDelay: '<S29>/Unit Delay'
   */
  elapseTime = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr +
                WhlSpdRR_mps) + rtb_Add4_j;

  /* Product: '<S29>/Product1' */
  rtb_deltafalllimit_iz = elapseTime * 10.0;

  /* RelationalOperator: '<S70>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_iz > rtb_UkYk1);

  /* Switch: '<S70>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Gain: '<S29>/Gain3' */
    rtb_Divide_g = -rtb_Switch2_b0;

    /* RelationalOperator: '<S70>/UpperRelop' */
    rtb_Compare = (rtb_deltafalllimit_iz < rtb_Divide_g);

    /* Switch: '<S70>/Switch' */
    if (rtb_Compare) {
      rtb_deltafalllimit_iz = rtb_Divide_g;
    }

    /* End of Switch: '<S70>/Switch' */
    rtb_UkYk1 = rtb_deltafalllimit_iz;
  }

  /* End of Switch: '<S70>/Switch2' */

  /* Sum: '<S29>/Add7' incorporates:
   *  UnitDelay: '<S29>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l = rtb_Switch2_b0 +
    rtb_UkYk1;

  /* Lookup_n-D: '<S29>/VehicleStableTarget_mps1' */
  rtb_Add4_j = look1_iflf_binlc(rtb_Add7,
    VehCtrlMdel240926_2018b__ConstP.pooled35,
    VehCtrlMdel240926_2018b__ConstP.pooled51, 3U);

  /* Sum: '<S29>/Add13' */
  rtb_Add7 += rtb_Add4_j;

  /* Sum: '<S29>/Add12' */
  rtb_Add -= rtb_Add7;

  /* RelationalOperator: '<S29>/Relational Operator1' incorporates:
   *  Constant: '<S29>/Verror1'
   */
  rtb_LowerRelop1_b = (rtb_Add < 0.0F);

  /* RelationalOperator: '<S29>/Relational Operator2' incorporates:
   *  UnitDelay: '<S29>/Unit Delay4'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l >=
                 MCFL_ActualTorque);

  /* RelationalOperator: '<S66>/Compare' incorporates:
   *  Constant: '<S66>/Constant'
   *  UnitDelay: '<S29>/Unit Delay4'
   */
  rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l <= 0.01);

  /* Logic: '<S29>/OR' */
  rtb_Compare = (rtb_Compare || rtb_Compare_am);

  /* Logic: '<S29>/Logical Operator5' incorporates:
   *  UnitDelay: '<S29>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i = (rtb_LowerRelop1_b &&
    rtb_Compare);

  /* Switch: '<S29>/Switch2' incorporates:
   *  Switch: '<S29>/Switch7'
   *  UnitDelay: '<S29>/Unit Delay3'
   *  UnitDelay: '<S68>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp) {
    /* RelationalOperator: '<S71>/LowerRelop1' incorporates:
     *  Constant: '<S29>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S29>/Unit Delay4'
     */
    rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l > 235.0);

    /* Switch: '<S71>/Switch2' incorporates:
     *  Constant: '<S29>/TCS_TrqRequest_Max2'
     */
    if (rtb_Compare) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 235.0;
    } else {
      /* RelationalOperator: '<S71>/UpperRelop' incorporates:
       *  Constant: '<S29>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S29>/Unit Delay4'
       */
      rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l < 0.0);

      /* Switch: '<S71>/Switch' incorporates:
       *  Constant: '<S29>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S29>/Unit Delay4'
       */
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l;
      }

      /* End of Switch: '<S71>/Switch' */
    }

    /* End of Switch: '<S71>/Switch2' */

    /* RelationalOperator: '<S72>/LowerRelop1' */
    rtb_Compare = (MCFL_ActualTorque >
                   VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b);

    /* Switch: '<S72>/Switch2' */
    if (!rtb_Compare) {
      /* RelationalOperator: '<S72>/UpperRelop' incorporates:
       *  Constant: '<S29>/TCS_TrqRequest_Min1'
       */
      rtb_Compare = (MCFL_ActualTorque < 0.0);

      /* Switch: '<S72>/Switch' incorporates:
       *  Constant: '<S29>/TCS_TrqRequest_Min1'
       */
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = MCFL_ActualTorque;
      }

      /* End of Switch: '<S72>/Switch' */
    }

    /* End of Switch: '<S72>/Switch2' */
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i) {
      /* Switch: '<S29>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = MCFL_ActualTorque;
    }
  }

  /* End of Switch: '<S29>/Switch2' */

  /* Switch: '<S7>/Switch7' incorporates:
   *  UnitDelay: '<S29>/Unit Delay2'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.TCSF_Enable_OUT != 0.0) {
    rtb_Yk1 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b;
  } else {
    /* Gain: '<S7>/Gain8' */
    rtb_Add = 0.1F * rtb_Add6_p;

    /* Sum: '<S28>/Add3' incorporates:
     *  Constant: '<S28>/RPM_min4'
     */
    rtb_deltafalllimit_iz = MCFL_ActualVelocity + 10.0;

    /* MinMax: '<S28>/Max2' incorporates:
     *  Constant: '<S28>/RPM_min5'
     */
    rtb_UkYk1 = fmax(rtb_deltafalllimit_iz, 1.0);

    /* Product: '<S28>/Product3' */
    rtb_deltafalllimit_iz = 7.0 * rtb_Yk1;

    /* Product: '<S28>/Product2' */
    rtb_Yk1 = rtb_deltafalllimit_iz * 9550.0;

    /* Product: '<S28>/Divide2' */
    rtb_Yk1 /= rtb_UkYk1;

    /* RelationalOperator: '<S27>/LowerRelop1' */
    rtb_Compare = (rtb_Add > rtb_Yk1);

    /* Switch: '<S27>/Switch2' */
    if (rtb_Compare) {
      rtb_Add = (real32_T)rtb_Yk1;
    } else {
      /* RelationalOperator: '<S27>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant15'
       */
      rtb_Compare = (rtb_Add < 0.0F);

      /* Switch: '<S27>/Switch' incorporates:
       *  Constant: '<S7>/Constant15'
       */
      if (rtb_Compare) {
        rtb_Add = 0.0F;
      }

      /* End of Switch: '<S27>/Switch' */
    }

    /* End of Switch: '<S27>/Switch2' */
    rtb_Yk1 = rtb_Add;
  }

  /* End of Switch: '<S7>/Switch7' */

  /* RelationalOperator: '<S24>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Yk1 > rtb_Add10);

  /* Switch: '<S24>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* RelationalOperator: '<S24>/UpperRelop' incorporates:
     *  Constant: '<S7>/Constant9'
     */
    rtb_Compare = (rtb_Yk1 < 0.0);

    /* Switch: '<S24>/Switch' incorporates:
     *  Constant: '<S7>/Constant9'
     */
    if (rtb_Compare) {
      rtb_Add10 = 0.0F;
    } else {
      rtb_Add10 = (real32_T)rtb_Yk1;
    }

    /* End of Switch: '<S24>/Switch' */
  }

  /* End of Switch: '<S24>/Switch2' */

  /* Switch: '<S7>/Switch4' */
  rtb_Add = rtb_Add10;

  /* UnitDelay: '<S21>/Delay Input2'
   *
   * Block description for '<S21>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib;

  /* Sum: '<S21>/Difference Inputs1'
   *
   * Block description for '<S21>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Divide_g = rtb_Add - rtb_Add10;

  /* SampleTimeMath: '<S21>/sample time'
   *
   * About '<S21>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S21>/delta rise limit' */
  rtb_Add = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S64>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Divide_g > rtb_Add);

  /* Switch: '<S64>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S21>/delta fall limit' */
    rtb_Add = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S64>/UpperRelop' */
    rtb_Compare = (rtb_Divide_g < rtb_Add);

    /* Switch: '<S64>/Switch' */
    if (rtb_Compare) {
      rtb_Divide_g = rtb_Add;
    }

    /* End of Switch: '<S64>/Switch' */
    rtb_Add = rtb_Divide_g;
  }

  /* End of Switch: '<S64>/Switch2' */

  /* Saturate: '<S7>/Saturation3' incorporates:
   *  Sum: '<S21>/Difference Inputs2'
   *  UnitDelay: '<S21>/Delay Input2'
   *
   * Block description for '<S21>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S21>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib = rtb_Add + rtb_Add10;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib > 2.0F) {
    AMKTrqFL_cmd = 2.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib < -2.0F) {
    AMKTrqFL_cmd = -2.0F;
  } else {
    AMKTrqFL_cmd = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib;
  }

  /* End of Saturate: '<S7>/Saturation3' */

  /* SampleTimeMath: '<S7>/Weighted Sample Time1'
   *
   * About '<S7>/Weighted Sample Time1':
   *  y = u * K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;
  rtb_UkYk1 = 420000.0 * elapseTime;

  /* Sum: '<S7>/Add2' */
  rtb_UkYk1 += RPM;

  /* Saturate: '<S7>/RPM_Saturation1' */
  if (rtb_UkYk1 > 5000.0) {
    rtb_UkYk1 = 5000.0;
  } else {
    if (rtb_UkYk1 < -50.0) {
      rtb_UkYk1 = -50.0;
    }
  }

  /* End of Saturate: '<S7>/RPM_Saturation1' */

  /* Update for UnitDelay: '<S7>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_n =
    VehCtrlMdel240926_2018b_amksp_B.DYC_Enable_OUT;

  /* Update for UnitDelay: '<S31>/Unit Delay5' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_l = rtb_deltafalllimit_n;

  /* Update for UnitDelay: '<S31>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g = rtb_Switch2_mn;

  /* Update for UnitDelay: '<S87>/Delay Input1'
   *
   * Block description for '<S87>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE = rtb_ignition;

  /* Update for UnitDelay: '<S30>/Unit Delay5' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = rtb_Acc_POS;

  /* Update for UnitDelay: '<S30>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_f = rtb_Ax;

  /* Update for UnitDelay: '<S76>/Delay Input1'
   *
   * Block description for '<S76>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_j = rtb_LogicalOperator2;

  /* Update for UnitDelay: '<S29>/Unit Delay5' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_ip = rtb_deltafalllimit_om;

  /* Update for UnitDelay: '<S29>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g4 = rtb_Switch2_b0;

  /* Update for UnitDelay: '<S67>/Delay Input1'
   *
   * Block description for '<S67>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_b = rtb_LogicalOperator7;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/10ms1' */

  /* S-Function (fcncallgen): '<S5>/10ms2' incorporates:
   *  SubSystem: '<S5>/VCU2AMKMCUFL'
   */
  /* Switch: '<S339>/Switch1' incorporates:
   *  Constant: '<S339>/Constant1'
   *  Constant: '<S339>/Constant2'
   *  Switch: '<S339>/Switch'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueOn) {
    VehCtrlMdel240926_2018b_amksp_B.Switch1_l = -21.0;
    VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueLimitP = 21.0;
  } else {
    VehCtrlMdel240926_2018b_amksp_B.Switch1_l = 0.0;
    VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueLimitP = 0.0;
  }

  /* End of Switch: '<S339>/Switch1' */

  /* S-Function (scanpack): '<S339>/CAN Pack1' */
  /* S-Function (scanpack): '<S339>/CAN Pack1' */
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
        real32_T result = AMKTrqFL_cmd;

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
        real64_T result = VehCtrlMdel240926_2018b_amksp_B.Switch1_l;

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
        uint32_T result = (uint32_T)
          (VehCtrlMdel240926_2018b_amksp_B.MCFL_DCOn_setpoints_o);

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
          (VehCtrlMdel240926_2018b_amksp_B.AMKMCFL_InverterOn);

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

  /* S-Function (ecucoder_canmessage): '<S339>/CANPackMessage' */

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

  /* S-Function (ec5744_cantransmitslb): '<S339>/CANTransmit' */

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
  /* Switch: '<S340>/Switch1' incorporates:
   *  Constant: '<S340>/Constant'
   *  Constant: '<S340>/Constant1'
   *  Switch: '<S340>/Switch'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.MCFR_TorqueOn) {
    VehCtrlMdel240926_2018b_amksp_B.Switch1 = -21.0;
    VehCtrlMdel240926_2018b_amksp_B.Switch = 21.0;
  } else {
    VehCtrlMdel240926_2018b_amksp_B.Switch1 = 0.0;
    VehCtrlMdel240926_2018b_amksp_B.Switch = 0.0;
  }

  /* End of Switch: '<S340>/Switch1' */

  /* S-Function (scanpack): '<S340>/CAN Pack1' */
  /* S-Function (scanpack): '<S340>/CAN Pack1' */
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
        real32_T result = VehCtrlMdel240926_2018b_amksp_B.TrqFR_cmd;

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
        real64_T result = VehCtrlMdel240926_2018b_amksp_B.Switch1;

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
        real64_T result = VehCtrlMdel240926_2018b_amksp_B.Switch;

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
        uint32_T result = (uint32_T)
          (VehCtrlMdel240926_2018b_amksp_B.MCFL_DCOn_setpoints_o);

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
          (VehCtrlMdel240926_2018b_amksp_B.AMKMCFL_InverterOn);

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

  /* S-Function (ecucoder_canmessage): '<S340>/CANPackMessage' */

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

  /* S-Function (ec5744_cantransmitslb): '<S340>/CANTransmit' */

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
  /* Switch: '<S341>/Switch2' incorporates:
   *  Constant: '<S341>/Constant13'
   *  Constant: '<S341>/Constant17'
   *  Constant: '<S341>/Constant19'
   *  Constant: '<S341>/Constant20'
   *  Switch: '<S341>/Switch3'
   */
  if (TrqR_cmd_raw) {
    Gear_Trs = 0.0;
    Mode_Trs = 0.0;
  } else {
    Gear_Trs = 2.0;
    Mode_Trs = 2.0;
  }

  /* End of Switch: '<S341>/Switch2' */

  /* DataTypeConversion: '<S341>/Cast To Boolean4' */
  VehCtrlMdel240926_2018b_amksp_B.CastToBoolean4 = (uint16_T)Gear_Trs;

  /* DataTypeConversion: '<S341>/Cast To Boolean6' */
  VehCtrlMdel240926_2018b_amksp_B.CastToBoolean6 = (uint16_T)Mode_Trs;

  /* DataTypeConversion: '<S341>/Data Type Conversion2' */
  VehCtrlMdel240926_2018b_amksp_B.DataTypeConversion2 = (int32_T)floor(rtb_UkYk1);

  /* S-Function (scanpack): '<S341>/CAN Pack1' */
  /* S-Function (scanpack): '<S341>/CAN Pack1' */
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
        real32_T result = EmraxTrqR_cmd;

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

  /* S-Function (ecucoder_canmessage): '<S341>/CANPackMessage' */

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

  /* S-Function (ec5744_cantransmitslb): '<S341>/CANTransmit' */

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
  /* DataTypeConversion: '<S342>/Cast To Single1' */
  VehCtrlMdel240926_2018b_amksp_B.CastToSingle1 = (uint16_T)rtb_CastToDouble;

  /* S-Function (ec5744_pdsslbu3): '<S342>/PowerDriverSwitch(HS)' */

  /* Set level VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON for the specified power driver switch */
  ec_gpio_write(50,VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON);
  ec_gpio_write(42,VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON);

  /* S-Function (ec5744_pdpslbu3): '<S342>/PowerDriverPWM' incorporates:
   *  Constant: '<S342>/Constant'
   */

  /* Power driver PWM output for channel 6 */
  ec_pwm_output(6,((uint16_T)1000U),
                VehCtrlMdel240926_2018b_amksp_B.CastToSingle1);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms6' */

  /* S-Function (fcncallgen): '<S348>/10ms' incorporates:
   *  SubSystem: '<S348>/daq10ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S360>/CCPDAQ' */
  ccpDaq(1);

  /* End of Outputs for S-Function (fcncallgen): '<S348>/10ms' */

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
  /* S-Function (fcncallgen): '<S348>/50ms' incorporates:
   *  SubSystem: '<S348>/daq50ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S362>/CCPDAQ' */
  ccpDaq(2);

  /* End of Outputs for S-Function (fcncallgen): '<S348>/50ms' */
}

/* Model step function for TID5 */
void VehCtrlMdel240926_2018b_amkspdlimit_step5(void) /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S347>/100MS' incorporates:
   *  SubSystem: '<S347>/Function-Call Subsystem'
   */
  /* S-Function (ec5744_canreceiveslb): '<S351>/CANReceive' */

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

  /* Call the system: <S351>/Function-Call Subsystem */

  /* Output and update for function-call system: '<S351>/Function-Call Subsystem' */
  {
    uint8_T rtb_Add;
    uint8_T rtb_Compare;

    /* Outputs for Enabled SubSystem: '<S352>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S353>/Enable'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.CANReceive_o2_p > 0) {
      /* RelationalOperator: '<S354>/Compare' incorporates:
       *  Constant: '<S354>/Constant'
       */
      rtb_Add = (uint8_T)(VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[0] ==
                          83);

      /* RelationalOperator: '<S355>/Compare' incorporates:
       *  Constant: '<S355>/Constant'
       */
      rtb_Compare = (uint8_T)(VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[5]
        == 84);

      /* Sum: '<S353>/Add' */
      rtb_Add = (uint8_T)((uint32_T)rtb_Add + rtb_Compare);

      /* RelationalOperator: '<S356>/Compare' incorporates:
       *  Constant: '<S356>/Constant'
       */
      rtb_Compare = (uint8_T)(rtb_Add == 2);

      /* If: '<S353>/If' */
      if (rtb_Compare > 0) {
        /* Outputs for IfAction SubSystem: '<S353>/If Action Subsystem' incorporates:
         *  ActionPort: '<S357>/Action Port'
         */
        /* S-Function (ec5744_bootloaderslb): '<S357>/BootLoader' */
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

        /* S-Function (ec5744_cpuresetslb): '<S357>/CPUReset' */

        /* Perform a microcontroller reset */
        MC_ME.MCTL.R = 0X00005AF0;
        MC_ME.MCTL.R = 0X0000A50F;

        /* End of Outputs for SubSystem: '<S353>/If Action Subsystem' */
      } else {
        /* Outputs for IfAction SubSystem: '<S353>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S358>/Action Port'
         */
        /* S-Function (ec5744_cantransmitslb): '<S358>/CANTransmit' incorporates:
         *  Constant: '<S358>/Constant'
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

        /* End of Outputs for SubSystem: '<S353>/If Action Subsystem1' */
      }

      /* End of If: '<S353>/If' */
    }

    /* End of Outputs for SubSystem: '<S352>/Enabled Subsystem' */
  }

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S351>/CANReceive' */
  /* End of Outputs for S-Function (fcncallgen): '<S347>/100MS' */

  /* S-Function (fcncallgen): '<S348>/100ms' incorporates:
   *  SubSystem: '<S348>/daq100ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S359>/CCPDAQ' */
  ccpDaq(3);

  /* End of Outputs for S-Function (fcncallgen): '<S348>/100ms' */
}

/* Model step function for TID6 */
void VehCtrlMdel240926_2018b_amkspdlimit_step6(void) /* Sample time: [0.5s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S348>/500ms' incorporates:
   *  SubSystem: '<S348>/daq500ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S361>/CCPDAQ' */
  ccpDaq(4);

  /* End of Outputs for S-Function (fcncallgen): '<S348>/500ms' */

  /* S-Function (fcncallgen): '<S349>/500ms' incorporates:
   *  SubSystem: '<S349>/EEPROMOperation'
   */

  /* S-Function (ec5744_eepromoslb): '<S364>/EEPROMOperatin' */
#if defined EC_EEPROM_ENABLE

  /* Operate the EEPROM module on the MPC5744 */
  ec_flash_operation();

#endif

  /* End of Outputs for S-Function (fcncallgen): '<S349>/500ms' */
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
  /* Start for S-Function (ec5744_canreceiveslb): '<S115>/CANReceive1' incorporates:
   *  SubSystem: '<S115>/MCU_pwr'
   */
  /* Start for function-call system: '<S115>/MCU_pwr' */

  /* Start for Enabled SubSystem: '<S170>/MCU_VCUMeter1' */

  /* Start for S-Function (scanunpack): '<S172>/CAN Unpack' */

  /*-----------S-Function Block: <S172>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S170>/MCU_VCUMeter1' */
  ec_buffer_init(0,1,1,218089455);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S115>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S115>/CANReceive3' incorporates:
   *  SubSystem: '<S115>/MCU_state'
   */
  /* Start for function-call system: '<S115>/MCU_state' */

  /* Start for Enabled SubSystem: '<S171>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S178>/CAN Unpack' */

  /*-----------S-Function Block: <S178>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S171>/MCU_state' */
  ec_buffer_init(0,0,1,218089199);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S115>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S111>/CANReceive3' incorporates:
   *  SubSystem: '<S111>/ABS_BUS_state'
   */
  /* Start for function-call system: '<S111>/ABS_BUS_state' */

  /* Start for Enabled SubSystem: '<S119>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S120>/CAN Unpack1' */

  /*-----------S-Function Block: <S120>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S119>/IMU_state' */
  ec_buffer_init(1,16,0,1698);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S111>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S117>/CANReceive3' incorporates:
   *  SubSystem: '<S117>/StrWhSnis_state'
   */
  /* Start for function-call system: '<S117>/StrWhSnis_state' */

  /* Start for Enabled SubSystem: '<S190>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S191>/CAN Unpack1' */

  /*-----------S-Function Block: <S191>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S190>/IMU_state' */
  ec_buffer_init(1,7,0,330);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S117>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms4' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S125>/CANReceive3' incorporates:
   *  SubSystem: '<S125>/AMKMCU_state'
   */
  /* Start for function-call system: '<S125>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S127>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S130>/CAN Unpack' */

  /*-----------S-Function Block: <S130>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S127>/MCU_state' */
  ec_buffer_init(1,1,0,640);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S125>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S125>/CANReceive1' incorporates:
   *  SubSystem: '<S125>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S125>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S128>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S140>/CAN Unpack' */

  /*-----------S-Function Block: <S140>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S128>/MCU_state' */
  ec_buffer_init(1,2,0,642);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S125>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S125>/CANReceive2' incorporates:
   *  SubSystem: '<S125>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S125>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S129>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S142>/CAN Unpack' */

  /*-----------S-Function Block: <S142>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S129>/MCU_state' */
  ec_buffer_init(1,3,0,644);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S125>/CANReceive2' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S126>/CANReceive3' incorporates:
   *  SubSystem: '<S126>/AMKMCU_state'
   */
  /* Start for function-call system: '<S126>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S146>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S149>/CAN Unpack' */

  /*-----------S-Function Block: <S149>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S146>/MCU_state' */
  ec_buffer_init(1,4,0,641);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S126>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S126>/CANReceive1' incorporates:
   *  SubSystem: '<S126>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S126>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S147>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S158>/CAN Unpack' */

  /*-----------S-Function Block: <S158>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S147>/MCU_state' */
  ec_buffer_init(1,5,0,643);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S126>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S126>/CANReceive2' incorporates:
   *  SubSystem: '<S126>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S126>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S148>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S160>/CAN Unpack' */

  /*-----------S-Function Block: <S160>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S148>/MCU_state' */
  ec_buffer_init(1,0,0,645);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S126>/CANReceive2' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms5' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S116>/CANReceive3' incorporates:
   *  SubSystem: '<S116>/IMU_state'
   */
  /* Start for function-call system: '<S116>/IMU_state' */

  /* Start for Enabled SubSystem: '<S185>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S186>/CAN Unpack' */

  /*-----------S-Function Block: <S186>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S185>/MCU_state' */
  ec_buffer_init(1,17,0,513);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S116>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms2' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S114>/CANReceive3' */
  ec_buffer_init(0,3,1,408961267);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S114>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S339>/CANTransmit' */
  ec_buffer_init(1,8,0,386U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms2' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S340>/CANTransmit' */
  ec_buffer_init(1,9,0,387U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms4' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S341>/CANTransmit' */
  ec_buffer_init(0,8,1,146927393U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S5>/10ms6' incorporates:
   *  SubSystem: '<S5>/WP_OUTPUT'
   */
  /* Start for S-Function (ec5744_pdpslbu3): '<S342>/PowerDriverPWM' incorporates:
   *  Constant: '<S342>/Constant'
   */

  /* Initialize PWM output for channel 6 */
  SIUL2_MSCR42 = 0X02000003;           //PWM_A3

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S347>/100MS' incorporates:
   *  SubSystem: '<S347>/Function-Call Subsystem'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S351>/CANReceive' incorporates:
   *  SubSystem: '<S351>/Function-Call Subsystem'
   */
  /* Start for function-call system: '<S351>/Function-Call Subsystem' */

  /* Start for Enabled SubSystem: '<S352>/Enabled Subsystem' */
  /* Start for IfAction SubSystem: '<S353>/If Action Subsystem1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S358>/CANTransmit' incorporates:
   *  Constant: '<S358>/Constant'
   */
  ec_buffer_init(2,9,0,593U);

  /* End of Start for SubSystem: '<S353>/If Action Subsystem1' */
  /* End of Start for SubSystem: '<S352>/Enabled Subsystem' */
  ec_buffer_init(2,1,0,278);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S351>/CANReceive' */
  /* End of Start for S-Function (fcncallgen): '<S347>/100MS' */

  /* Start for S-Function (fcncallgen): '<S350>/Function-Call Generator' incorporates:
   *  SubSystem: '<S350>/CCPBackground'
   */
  /* Start for S-Function (ec5744_ccpslb): '<S365>/CCPBackground' */
  ccpInit();

  /* End of Start for S-Function (fcncallgen): '<S350>/Function-Call Generator' */

  /* Start for S-Function (ec5744_caninterruptslb1): '<S350>/ReceiveandTransmitInterrupt' incorporates:
   *  SubSystem: '<S350>/CCPReceive'
   */
  /* Start for function-call system: '<S350>/CCPReceive' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S366>/CANReceive' */
  ec_buffer_init(2,0,0,CCP_CRO_ID);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S366>/CANReceive' */
  ec_bufint_init(2,0);
  INTC_0.PSR[548].B.PRIN = 12;
  IntcIsrVectorTable[548] = (uint32_t)&ISR_FlexCAN_2_MB0;

  /* End of Start for S-Function (ec5744_caninterruptslb1): '<S350>/ReceiveandTransmitInterrupt' */

  /* Start for S-Function (ec5744_eeprombsbu3): '<Root>/EEPROMEnable' */
  Fls_Read(0xFA0010,ecflashdataold,4096);
  Fls_Read(0xFA0010,ecflashdatanew,4096);

  /* SystemInitialize for S-Function (fcncallgen): '<S3>/10ms7' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* SystemInitialize for SignalConversion generated from: '<S118>/Constant' */
  HVCUTOFF = true;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S3>/10ms7' */

  /* SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  /* InitializeConditions for UnitDelay: '<S268>/Unit Delay4' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn = 0.01F;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' */

  /* SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* SystemInitialize for Chart: '<S101>/Chart2' */
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
  /* Enable for Chart: '<S101>/Chart2' */
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
