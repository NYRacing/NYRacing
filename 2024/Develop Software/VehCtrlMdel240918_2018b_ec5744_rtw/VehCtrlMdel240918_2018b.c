/*
 * Code generated for Simulink model VehCtrlMdel240918_2018b.
 *
 * FILE    : VehCtrlMdel240918_2018b.c
 *
 * VERSION : 1.163
 *
 * DATE    : Tue Sep 24 23:15:25 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#include "VehCtrlMdel240918_2018b.h"
#include "VehCtrlMdel240918_2018b_private.h"

/* #include "myinclude.h" */

/* Named constants for Chart: '<S8>/Timer1' */
#define VehCtrlMdel240918_2018b_IN_Out ((uint8_T)2U)
#define VehCtrlMdel240918_20_IN_Trigger ((uint8_T)3U)
#define VehCtrlMdel240918_IN_InterState ((uint8_T)1U)

/* Named constants for Chart: '<S105>/Timer' */
#define VehCtrlMdel240918_2018_IN_Out_n ((uint8_T)2U)
#define VehCtrlMdel240918__IN_Trigger_c ((uint8_T)3U)
#define VehCtrlMdel2409_IN_InterState_n ((uint8_T)1U)

/* Named constants for Chart: '<S7>/Chart' */
#define VehCtrlMdel240918_2018b_IN_A   ((uint8_T)1U)
#define VehCtrlMdel240918_2018b_IN_B   ((uint8_T)2U)
#define VehCtrlMdel240918_2018b_IN_C   ((uint8_T)3U)
#define VehCtrlMdel240918_IN_DYC_Enable ((uint8_T)5U)
#define VehCtrlMdel240918__IN_InitState ((uint8_T)7U)
#define VehCtrlMdel24091_IN_TCSF_Enable ((uint8_T)9U)
#define VehCtrlMdel240_IN_DYC_Disenable ((uint8_T)4U)
#define VehCtrlMdel24_IN_TCSF_Disenable ((uint8_T)8U)
#define VehCtrlMdel24_IN_TCSR_Disenable ((uint8_T)10U)
#define VehCtrlMdel2_IN_F_TVD_TCS_STATE ((uint8_T)6U)

/* Named constants for Chart: '<S91>/Chart2' */
#define VehCtrlM_IN_MC_InverterOn_State ((uint8_T)8U)
#define VehCtrlMdel240918_2018_IN_Guard ((uint8_T)1U)
#define VehCtrlMdel240918_2018_IN_Ready ((uint8_T)5U)
#define VehCtrlMdel240918_2018_IN_Trans ((uint8_T)7U)
#define VehCtrlMdel240918_2018_IN_start ((uint8_T)10U)
#define VehCtrlMdel240918_2018b_IN_Init ((uint8_T)2U)
#define VehCtrlMdel240918_2018b_IN_OFF ((uint8_T)1U)
#define VehCtrlMdel240918_2018b_IN_ON  ((uint8_T)2U)
#define VehCtrlMdel240918_201_IN_AMKCAN ((uint8_T)1U)
#define VehCtrlMdel240918_20_IN_Standby ((uint8_T)6U)
#define VehCtrlMdel240918_IN_MC_DCready ((uint8_T)7U)
#define VehCtrlMdel240918_IN_SYSRDYCECK ((uint8_T)9U)
#define VehCtrlMdel240918_event_AMKDCON (3)
#define VehCtrlMdel240918_event_EbeepON (5)
#define VehCtrlMdel24091_IN_MCDCOncheck ((uint8_T)5U)
#define VehCtrlMdel24091_IN_MC_DCOnFail ((uint8_T)6U)
#define VehCtrlMdel24091_event_AMKCANON (1)
#define VehCtrlMdel24091_event_AMKDCOFF (2)
#define VehCtrlMdel24091_event_EbeepOFF (4)
#define VehCtrlMdel24091_event_TorqueON (11)
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
real_T Gear_Trs;                       /* '<S319>/Switch2' */
real_T Mode_Trs;                       /* '<S319>/Switch3' */
real_T Trq_CUT;                        /* '<S184>/Timer' */
real_T ignition;                       /* '<S105>/Timer' */
real_T L12V_error;                     /* '<S162>/CAN Unpack' */
real_T alarm;                          /* '<S162>/CAN Unpack' */
real_T controller_ready;               /* '<S162>/CAN Unpack' */
real_T selfcheck;                      /* '<S162>/CAN Unpack' */
real_T RPM;                            /* '<S162>/CAN Unpack' */
real_T trq;                            /* '<S162>/CAN Unpack' */
real_T AC_current;                     /* '<S157>/CAN Unpack' */
real_T DC_current;                     /* '<S157>/CAN Unpack' */
real_T MCU_Temp;                       /* '<S157>/CAN Unpack' */
real_T motor_Temp;                     /* '<S157>/CAN Unpack' */
real_T MCFR_ActualVelocity;            /* '<S135>/CAN Unpack' */
real_T MCFR_DCVoltage;                 /* '<S135>/CAN Unpack' */
real_T MCFR_bDCOn;                     /* '<S135>/CAN Unpack' */
real_T MCFR_bError;                    /* '<S135>/CAN Unpack' */
real_T MCFR_bInverterOn;               /* '<S135>/CAN Unpack' */
real_T MCFR_bQuitInverterOn;           /* '<S135>/CAN Unpack' */
real_T MCFR_bSystemReady;              /* '<S135>/CAN Unpack' */
real_T MCFR_TempIGBT;                  /* '<S145>/CAN Unpack' */
real_T MCFR_TempInverter;              /* '<S145>/CAN Unpack' */
real_T MCFR_TempMotor;                 /* '<S145>/CAN Unpack' */
real_T MCFR_ErrorInfo;                 /* '<S143>/CAN Unpack' */
real_T MCFL_DCVoltage;                 /* '<S117>/CAN Unpack' */
real_T MCFL_bDCOn;                     /* '<S117>/CAN Unpack' */
real_T MCFL_bError;                    /* '<S117>/CAN Unpack' */
real_T MCFL_bInverterOn;               /* '<S117>/CAN Unpack' */
real_T MCFL_bQuitDCOn;                 /* '<S117>/CAN Unpack' */
real_T MCFL_bQuitInverterOn;           /* '<S117>/CAN Unpack' */
real_T MCFL_bSystemReady;              /* '<S117>/CAN Unpack' */
real_T MCFL_ActualVelocity;            /* '<S117>/Gain' */
real_T MCFL_TempIGBT;                  /* '<S128>/CAN Unpack' */
real_T MCFL_TempInverter;              /* '<S128>/CAN Unpack' */
real_T MCFL_TempMotor;                 /* '<S128>/CAN Unpack' */
real_T MCFL_ErrorInfo;                 /* '<S126>/CAN Unpack' */
real_T StrWhlAngAliveRollCnt;          /* '<S175>/CAN Unpack1' */
real_T StrWhlAng;                      /* '<S175>/CAN Unpack1' */
real_T StrWhlAngV;                     /* '<S175>/CAN Unpack1' */
real_T ABS_WS_FL;                      /* '<S107>/CAN Unpack1' */
real_T ABS_WS_FR;                      /* '<S107>/CAN Unpack1' */
real_T ABS_WS_RL;                      /* '<S107>/CAN Unpack1' */
real_T ABS_WS_RR;                      /* '<S107>/CAN Unpack1' */
real_T IMU_Ay_Value;                   /* '<S170>/CAN Unpack' */
real_T IMU_Ax_Value;                   /* '<S170>/CAN Unpack' */
real_T IMU_Yaw_Value;                  /* '<S170>/CAN Unpack' */
uint32_T Acc_vol2;                     /* '<S184>/Add3' */
uint32_T Acc_vol;                      /* '<S184>/Add2' */
uint32_T Acc_POS2;                     /* '<S184>/1-D Lookup Table3' */
real32_T Acc_POS;                      /* '<S184>/MATLAB Function' */
uint16_T F_BrkPrs;                     /* '<S184>/1-D Lookup Table1' */
uint16_T Acc1;                         /* '<S100>/Acc3' */
uint16_T Acc2;                         /* '<S100>/Acc4' */
uint16_T Brk1;                         /* '<S100>/Brk1' */
uint16_T Brk2;                         /* '<S100>/Brk2' */
boolean_T HVCUTOFF;                    /* '<S105>/Constant' */
boolean_T KeyPressed;                  /* '<S91>/Cast To Boolean' */
boolean_T Brk;                         /* '<S93>/Compare' */
boolean_T ACC_Release;                 /* '<S94>/Compare' */
boolean_T beeper_state;                /* '<S91>/Chart2' */
boolean_T Trq_CUT_final;               /* '<S7>/Logical Operator4' */

/* Block signals (default storage) */
B_VehCtrlMdel240918_2018b_T VehCtrlMdel240918_2018b_B;

/* Block states (default storage) */
DW_VehCtrlMdel240918_2018b_T VehCtrlMdel240918_2018b_DW;

/* Real-time model */
RT_MODEL_VehCtrlMdel240918_20_T VehCtrlMdel240918_2018b_M_;
RT_MODEL_VehCtrlMdel240918_20_T *const VehCtrlMdel240918_2018b_M =
  &VehCtrlMdel240918_2018b_M_;

/* Forward declaration for local functions */
static void VehC_enter_atomic_WaitForEngine(void);
static void VehCtrlMdel240918_2018b_VehStat(const boolean_T *HVCUTOFF_o, const
  real_T *MCFL_bQuitInverterOn_k, const real_T *MCFL_bSystemReady_i, const
  real_T *MCFR_bQuitInverterOn_d, const real_T *MCFR_bSystemReady_g);
static void VehCtr_enter_atomic_MC_DCOnFail(void);
static void VehCtrlMdel240918_20_AMKDCready(const real_T *MCFL_bDCOn_j, const
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
  /* Call the system: <S327>/CCPReceive */
  {
    /* S-Function (ec5744_caninterruptslb1): '<S327>/ReceiveandTransmitInterrupt' */

    /* Output and update for function-call system: '<S327>/CCPReceive' */

    /* S-Function (ec5744_canreceiveslb): '<S343>/CANReceive' */

    /* Receive CAN message */
    {
      uint8 CAN2BUF0RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

      uint8 can2buf0looprx= 0;
      VehCtrlMdel240918_2018b_B.CANReceive_o3= 256;
      VehCtrlMdel240918_2018b_B.CANReceive_o5= 8;
      VehCtrlMdel240918_2018b_B.CANReceive_o2= ec_can_receive(2,0, CAN2BUF0RX);
      VehCtrlMdel240918_2018b_B.CANReceive_o4[0]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240918_2018b_B.CANReceive_o4[1]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240918_2018b_B.CANReceive_o4[2]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240918_2018b_B.CANReceive_o4[3]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240918_2018b_B.CANReceive_o4[4]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240918_2018b_B.CANReceive_o4[5]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240918_2018b_B.CANReceive_o4[6]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel240918_2018b_B.CANReceive_o4[7]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
    }

    /* Nothing to do for system: <S343>/Nothing */

    /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S343>/CANReceive' */

    /* End of Outputs for S-Function (ec5744_caninterruptslb1): '<S327>/ReceiveandTransmitInterrupt' */
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
void VehCtrlMdel240918_2018b_SetEventsForThisBaseStep(boolean_T *eventFlags)
{
  /* Task runs when its counter is zero, computed via rtmStepTask macro */
  eventFlags[1] = ((boolean_T)rtmStepTask(VehCtrlMdel240918_2018b_M, 1));
  eventFlags[2] = ((boolean_T)rtmStepTask(VehCtrlMdel240918_2018b_M, 2));
  eventFlags[3] = ((boolean_T)rtmStepTask(VehCtrlMdel240918_2018b_M, 3));
  eventFlags[4] = ((boolean_T)rtmStepTask(VehCtrlMdel240918_2018b_M, 4));
  eventFlags[5] = ((boolean_T)rtmStepTask(VehCtrlMdel240918_2018b_M, 5));
  eventFlags[6] = ((boolean_T)rtmStepTask(VehCtrlMdel240918_2018b_M, 6));
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
  (VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[1])++;
  if ((VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[1]) > 1) {/* Sample time: [0.001s, 0.0s] */
    VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[1] = 0;
  }

  (VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[2])++;
  if ((VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[2]) > 9) {/* Sample time: [0.005s, 0.0s] */
    VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[2] = 0;
  }

  (VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[3])++;
  if ((VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[3]) > 19) {/* Sample time: [0.01s, 0.0s] */
    VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[3] = 0;
  }

  (VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[4])++;
  if ((VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[4]) > 99) {/* Sample time: [0.05s, 0.0s] */
    VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[4] = 0;
  }

  (VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[5])++;
  if ((VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[5]) > 199) {/* Sample time: [0.1s, 0.0s] */
    VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[5] = 0;
  }

  (VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[6])++;
  if ((VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[6]) > 999) {/* Sample time: [0.5s, 0.0s] */
    VehCtrlMdel240918_2018b_M->Timing.TaskCounters.TID[6] = 0;
  }
}

/*
 * Output and update for atomic system:
 *    '<S8>/Timer1'
 *    '<S8>/Timer2'
 *    '<S186>/Timer'
 *    '<S187>/Timer'
 *    '<S187>/Timer1'
 *    '<S187>/Timer2'
 *    '<S187>/Timer3'
 *    '<S245>/Timer'
 *    '<S245>/Timer1'
 *    '<S245>/Timer2'
 *    ...
 */
void VehCtrlMdel240918_20_Timer1(boolean_T rtu_Trigger, real32_T rtu_CountTime,
  real_T *rty_Exit, DW_Timer1_VehCtrlMdel240918_2_T *localDW)
{
  boolean_T sf_internal_predicateOutput;

  /* Chart: '<S8>/Timer1' */
  if (localDW->bitsForTID3.is_active_c5_VehCtrlMdel240918_ == 0U) {
    localDW->bitsForTID3.is_active_c5_VehCtrlMdel240918_ = 1U;
    localDW->bitsForTID3.is_c5_VehCtrlMdel240918_2018b = 3U;
    localDW->x += 0.01;
    *rty_Exit = 0.0;
  } else {
    switch (localDW->bitsForTID3.is_c5_VehCtrlMdel240918_2018b) {
     case VehCtrlMdel240918_IN_InterState:
      if (rtu_Trigger) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel240918_2018b = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_Out:
      *rty_Exit = 1.0;
      if (!rtu_Trigger) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel240918_2018b = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      }
      break;

     default:
      /* case IN_Trigger: */
      *rty_Exit = 0.0;
      sf_internal_predicateOutput = (rtu_Trigger && (localDW->x < rtu_CountTime));
      if (sf_internal_predicateOutput) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel240918_2018b = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      } else if (localDW->x >= rtu_CountTime) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel240918_2018b = 2U;
        *rty_Exit = 1.0;
        localDW->x = 0.0;
      } else {
        sf_internal_predicateOutput = ((localDW->x < rtu_CountTime) &&
          (!rtu_Trigger));
        if (sf_internal_predicateOutput) {
          localDW->bitsForTID3.is_c5_VehCtrlMdel240918_2018b = 1U;
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
 *    '<S105>/Timer'
 *    '<S184>/Timer'
 */
void VehCtrlMdel240918_201_Timer(boolean_T rtu_Trigger, real32_T rtu_CountTime,
  real_T *rty_Exit, DW_Timer_VehCtrlMdel240918_20_T *localDW)
{
  boolean_T sf_internal_predicateOutput;

  /* Chart: '<S105>/Timer' */
  if (localDW->bitsForTID3.is_active_c21_VehCtrlMdel240918 == 0U) {
    localDW->bitsForTID3.is_active_c21_VehCtrlMdel240918 = 1U;
    localDW->bitsForTID3.is_c21_VehCtrlMdel240918_2018b = 3U;
    localDW->x += 0.01;
    *rty_Exit = 0.0;
  } else {
    switch (localDW->bitsForTID3.is_c21_VehCtrlMdel240918_2018b) {
     case VehCtrlMdel2409_IN_InterState_n:
      if (rtu_Trigger) {
        localDW->bitsForTID3.is_c21_VehCtrlMdel240918_2018b = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      }
      break;

     case VehCtrlMdel240918_2018_IN_Out_n:
      *rty_Exit = 1.0;
      if (!rtu_Trigger) {
        localDW->bitsForTID3.is_c21_VehCtrlMdel240918_2018b = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      }
      break;

     default:
      /* case IN_Trigger: */
      *rty_Exit = 0.0;
      sf_internal_predicateOutput = (rtu_Trigger && (localDW->x < rtu_CountTime));
      if (sf_internal_predicateOutput) {
        localDW->bitsForTID3.is_c21_VehCtrlMdel240918_2018b = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      } else if (localDW->x >= rtu_CountTime) {
        localDW->bitsForTID3.is_c21_VehCtrlMdel240918_2018b = 2U;
        *rty_Exit = 1.0;
        localDW->x = 0.0;
      } else {
        sf_internal_predicateOutput = ((localDW->x < rtu_CountTime) &&
          (!rtu_Trigger));
        if (sf_internal_predicateOutput) {
          localDW->bitsForTID3.is_c21_VehCtrlMdel240918_2018b = 1U;
          localDW->x = 0.0;
        }
      }
      break;
    }
  }

  /* End of Chart: '<S105>/Timer' */
}

/* Function for Chart: '<S91>/Chart2' */
static void VehC_enter_atomic_WaitForEngine(void)
{
  int32_T b_previousEvent;
  b_previousEvent = VehCtrlMdel240918_2018b_DW.sfEvent;
  VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel2409_event_AMKCANOFF;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKCANenable != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel24091_event_AMKCANON)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel2409_event_AMKCANOFF)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel24091_event_AMKDCOFF;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKDCon != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel240918_event_AMKDCON)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel24091_event_AMKDCOFF)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel_event_MCDCEnableOFF;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MCDCEnable != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel2_event_MCDCEnableON)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel_event_MCDCEnableOFF)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel24_event_InverterOFF;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_InverterOn != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel240_event_InverterON)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel24_event_InverterOFF)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel2409_event_TorqueOFF;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_TorqueCUT != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel24091_event_TorqueON)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel2409_event_TorqueOFF)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240918_2018b_DW.sfEvent = b_previousEvent;
}

/* Function for Chart: '<S91>/Chart2' */
static void VehCtrlMdel240918_2018b_VehStat(const boolean_T *HVCUTOFF_o, const
  real_T *MCFL_bQuitInverterOn_k, const real_T *MCFL_bSystemReady_i, const
  real_T *MCFR_bQuitInverterOn_d, const real_T *MCFR_bSystemReady_g)
{
  boolean_T sf_internal_predicateOutput;
  int32_T b_previousEvent;
  switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat) {
   case VehCtrlMdel240918_2018_IN_Guard:
    if (VehCtrlMdel240918_2018b_DW.temporalCounter_i1 >= 25U) {
      b_previousEvent = VehCtrlMdel240918_2018b_DW.sfEvent;
      VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel240918_event_EbeepON;
      if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_BeeperStat != 0U) {
        switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat) {
         case VehCtrlMdel240918_2018b_IN_OFF:
          if (VehCtrlMdel240918_2018b_DW.sfEvent ==
              VehCtrlMdel240918_event_EbeepON) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat = 2U;
          }
          break;

         case VehCtrlMdel240918_2018b_IN_ON:
          if (VehCtrlMdel240918_2018b_DW.sfEvent ==
              VehCtrlMdel24091_event_EbeepOFF) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240918_2018b_DW.sfEvent = b_previousEvent;
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 7U;
      VehCtrlMdel240918_2018b_DW.temporalCounter_i1 = 0U;
    } else {
      sf_internal_predicateOutput = ((!KeyPressed) || (!Brk) || (!ACC_Release));
      if (sf_internal_predicateOutput) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 6U;
      } else {
        sf_internal_predicateOutput = ((!(*MCFL_bSystemReady_i != 0.0)) ||
          (!(*MCFR_bSystemReady_g != 0.0)));
        if (sf_internal_predicateOutput) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 3U;
        }
      }
    }
    break;

   case VehCtrlMdel240918_2018b_IN_Init:
    VehCtrlMdel240918_2018b_B.errorReset = 1.0;
    if (VehCtrlMdel240918_2018b_DW.temporalCounter_i1 >= 100U) {
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 8U;
      VehC_enter_atomic_WaitForEngine();
    }
    break;

   case VehCtrlMdel240_IN_InitStateBack:
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 8U;
    VehC_enter_atomic_WaitForEngine();
    break;

   case VehCtrlMdel2409_IN_MCUReadyFail:
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 8U;
    VehC_enter_atomic_WaitForEngine();
    break;

   case VehCtrlMdel240918_2018_IN_Ready:
    sf_internal_predicateOutput = ((!(*MCFL_bSystemReady_i != 0.0)) ||
      (!(*MCFR_bSystemReady_g != 0.0)));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 4U;
    } else {
      if (!*HVCUTOFF_o) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 3U;
      }
    }
    break;

   case VehCtrlMdel240918_20_IN_Standby:
    sf_internal_predicateOutput = ((!(*MCFL_bSystemReady_i != 0.0)) ||
      (!(*MCFR_bSystemReady_g != 0.0)));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 8U;
      VehC_enter_atomic_WaitForEngine();
    } else {
      sf_internal_predicateOutput = (KeyPressed && Brk && ACC_Release &&
        (*MCFL_bSystemReady_i != 0.0) && (*MCFR_bSystemReady_g != 0.0) &&
        (*MCFL_bQuitInverterOn_k != 0.0) && (*MCFR_bQuitInverterOn_d != 0.0));
      if (sf_internal_predicateOutput) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 1U;
        VehCtrlMdel240918_2018b_DW.temporalCounter_i1 = 0U;
      }
    }
    break;

   case VehCtrlMdel240918_2018_IN_Trans:
    if (VehCtrlMdel240918_2018b_DW.temporalCounter_i1 >= 250U) {
      b_previousEvent = VehCtrlMdel240918_2018b_DW.sfEvent;
      VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel24091_event_EbeepOFF;
      if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_BeeperStat != 0U) {
        switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat) {
         case VehCtrlMdel240918_2018b_IN_OFF:
          if (VehCtrlMdel240918_2018b_DW.sfEvent ==
              VehCtrlMdel240918_event_EbeepON) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat = 2U;
          }
          break;

         case VehCtrlMdel240918_2018b_IN_ON:
          if (VehCtrlMdel240918_2018b_DW.sfEvent ==
              VehCtrlMdel24091_event_EbeepOFF) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240918_2018b_DW.sfEvent = b_previousEvent;
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 5U;
    }
    break;

   case VehCtrlMdel240_IN_WaitForEngine:
    sf_internal_predicateOutput = ((*MCFL_bSystemReady_i != 0.0) &&
      (*MCFR_bSystemReady_g != 0.0));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 6U;
    }
    break;
  }
}

/* Function for Chart: '<S91>/Chart2' */
static void VehCtr_enter_atomic_MC_DCOnFail(void)
{
  int32_T b_previousEvent;
  b_previousEvent = VehCtrlMdel240918_2018b_DW.sfEvent;
  VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel24091_event_AMKDCOFF;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKDCon != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel240918_event_AMKDCON)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel24091_event_AMKDCOFF)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel_event_MCDCEnableOFF;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MCDCEnable != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel2_event_MCDCEnableON)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel_event_MCDCEnableOFF)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel24_event_InverterOFF;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_InverterOn != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel240_event_InverterON)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel24_event_InverterOFF)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel2409_event_TorqueOFF;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_TorqueCUT != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel24091_event_TorqueON)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (VehCtrlMdel240918_2018b_DW.sfEvent == VehCtrlMdel2409_event_TorqueOFF)
      {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
      }
      break;
    }
  }

  VehCtrlMdel240918_2018b_DW.sfEvent = b_previousEvent;
}

/* Function for Chart: '<S91>/Chart2' */
static void VehCtrlMdel240918_20_AMKDCready(const real_T *MCFL_bDCOn_j, const
  real_T *MCFL_bQuitInverterOn_k, const real_T *MCFL_bSystemReady_i, const
  real_T *MCFR_bDCOn_n, const real_T *MCFR_bQuitInverterOn_d, const real_T
  *MCFR_bSystemReady_g)
{
  boolean_T sf_internal_predicateOutput;
  int32_T f_previousEvent;
  switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready) {
   case VehCtrlMdel240918_201_IN_AMKCAN:
    sf_internal_predicateOutput = (KeyPressed && Brk && ACC_Release);
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 7U;
      VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = 0U;
    }
    break;

   case VehCtrlMdel240_IN_AMKDCOnFinish:
   case VehCtrlMdel24091_IN_MC_DCOnFail:
    break;

   case VehCtrlMdel240_IN_DCOnCheckPass:
    sf_internal_predicateOutput =
      ((VehCtrlMdel240918_2018b_DW.temporalCounter_i2 >= 100U) && (*MCFL_bDCOn_j
        != 0.0) && (*MCFR_bDCOn_n != 0.0));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 4U;
      VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = 0U;
      f_previousEvent = VehCtrlMdel240918_2018b_DW.sfEvent;
      VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel2_event_MCDCEnableON;
      if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MCDCEnable != 0U) {
        switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable) {
         case VehCtrlMdel240918_2018b_IN_OFF:
          if (VehCtrlMdel240918_2018b_DW.sfEvent ==
              VehCtrlMdel2_event_MCDCEnableON) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 2U;
          }
          break;

         case VehCtrlMdel240918_2018b_IN_ON:
          if (VehCtrlMdel240918_2018b_DW.sfEvent ==
              VehCtrlMdel_event_MCDCEnableOFF) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240918_2018b_DW.sfEvent = f_previousEvent;
    }
    break;

   case VehCtrlMdel2_IN_MCDCEnableState:
    if (VehCtrlMdel240918_2018b_DW.temporalCounter_i2 >= 100U) {
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 8U;
      f_previousEvent = VehCtrlMdel240918_2018b_DW.sfEvent;
      VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel240_event_InverterON;
      if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_InverterOn != 0U)
      {
        switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn) {
         case VehCtrlMdel240918_2018b_IN_OFF:
          if (VehCtrlMdel240918_2018b_DW.sfEvent ==
              VehCtrlMdel240_event_InverterON) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 2U;
          }
          break;

         case VehCtrlMdel240918_2018b_IN_ON:
          if (VehCtrlMdel240918_2018b_DW.sfEvent ==
              VehCtrlMdel24_event_InverterOFF) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240918_2018b_DW.sfEvent = f_previousEvent;
    }
    break;

   case VehCtrlMdel24091_IN_MCDCOncheck:
    if (VehCtrlMdel240918_2018b_DW.temporalCounter_i2 >= 100U) {
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 3U;
      VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = 0U;
    } else {
      sf_internal_predicateOutput = ((!(*MCFR_bDCOn_n != 0.0)) ||
        (!(*MCFL_bDCOn_j != 0.0)));
      if (sf_internal_predicateOutput) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 6U;
        VehCtr_enter_atomic_MC_DCOnFail();
      }
    }
    break;

   case VehCtrlMdel240918_IN_MC_DCready:
    if (VehCtrlMdel240918_2018b_DW.temporalCounter_i2 >= 100U) {
      f_previousEvent = VehCtrlMdel240918_2018b_DW.sfEvent;
      VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel240918_event_AMKDCON;
      if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKDCon != 0U) {
        switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon) {
         case VehCtrlMdel240918_2018b_IN_OFF:
          if (VehCtrlMdel240918_2018b_DW.sfEvent ==
              VehCtrlMdel240918_event_AMKDCON) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 2U;
          }
          break;

         case VehCtrlMdel240918_2018b_IN_ON:
          if (VehCtrlMdel240918_2018b_DW.sfEvent ==
              VehCtrlMdel24091_event_AMKDCOFF) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240918_2018b_DW.sfEvent = f_previousEvent;
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 5U;
      VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = 0U;
    }
    break;

   case VehCtrlM_IN_MC_InverterOn_State:
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 9U;
    VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = 0U;
    break;

   case VehCtrlMdel240918_IN_SYSRDYCECK:
    sf_internal_predicateOutput =
      ((VehCtrlMdel240918_2018b_DW.temporalCounter_i2 >= 1000U) &&
       (*MCFL_bSystemReady_i != 0.0) && (*MCFR_bSystemReady_g != 0.0) &&
       (*MCFL_bQuitInverterOn_k != 0.0) && (*MCFR_bQuitInverterOn_d != 0.0));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 2U;
      f_previousEvent = VehCtrlMdel240918_2018b_DW.sfEvent;
      VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel24091_event_TorqueON;
      if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_TorqueCUT != 0U) {
        switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT) {
         case VehCtrlMdel240918_2018b_IN_OFF:
          if (VehCtrlMdel240918_2018b_DW.sfEvent ==
              VehCtrlMdel24091_event_TorqueON) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 2U;
          }
          break;

         case VehCtrlMdel240918_2018b_IN_ON:
          if (VehCtrlMdel240918_2018b_DW.sfEvent ==
              VehCtrlMdel2409_event_TorqueOFF) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240918_2018b_DW.sfEvent = f_previousEvent;
    } else {
      sf_internal_predicateOutput = ((!(*MCFL_bSystemReady_i != 0.0)) ||
        (!(*MCFR_bSystemReady_g != 0.0)) || (!(*MCFL_bQuitInverterOn_k != 0.0)) ||
        (!(*MCFR_bQuitInverterOn_d != 0.0)));
      if (sf_internal_predicateOutput) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 6U;
        VehCtr_enter_atomic_MC_DCOnFail();
      }
    }
    break;

   case VehCtrlMdel240918_2018_IN_start:
    if (VehCtrlMdel240918_2018b_DW.temporalCounter_i2 >= 1000U) {
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 1U;
      f_previousEvent = VehCtrlMdel240918_2018b_DW.sfEvent;
      VehCtrlMdel240918_2018b_DW.sfEvent = VehCtrlMdel24091_event_AMKCANON;
      if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKCANenable != 0U) {
        switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable) {
         case VehCtrlMdel240918_2018b_IN_OFF:
          if (VehCtrlMdel240918_2018b_DW.sfEvent ==
              VehCtrlMdel24091_event_AMKCANON) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable = 2U;
          }
          break;

         case VehCtrlMdel240918_2018b_IN_ON:
          if (VehCtrlMdel240918_2018b_DW.sfEvent ==
              VehCtrlMdel2409_event_AMKCANOFF) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable = 1U;
          }
          break;
        }
      }

      VehCtrlMdel240918_2018b_DW.sfEvent = f_previousEvent;
    }
    break;
  }
}

/* Model step function for TID0 */
void VehCtrlMdel240918_2018b_step0(void) /* Sample time: [0.0005s, 0.0s] */
{
  {                                    /* Sample time: [0.0005s, 0.0s] */
    rate_monotonic_scheduler();
  }
}

/* Model step function for TID1 */
void VehCtrlMdel240918_2018b_step1(void) /* Sample time: [0.001s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S327>/Function-Call Generator' incorporates:
   *  SubSystem: '<S327>/CCPBackground'
   */

  /* S-Function (ec5744_ccpslb): '<S342>/CCPBackground' */
  ccpBackground();
  Lin0_Background();

  /* End of Outputs for S-Function (fcncallgen): '<S327>/Function-Call Generator' */
}

/* Model step function for TID2 */
void VehCtrlMdel240918_2018b_step2(void) /* Sample time: [0.005s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S325>/5ms' incorporates:
   *  SubSystem: '<S325>/daq5ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S340>/CCPDAQ' */
  ccpDaq(0);

  /* End of Outputs for S-Function (fcncallgen): '<S325>/5ms' */
}

/* Model step function for TID3 */
void VehCtrlMdel240918_2018b_step3(void) /* Sample time: [0.01s, 0.0s] */
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
  real32_T rtb_UkYk1_ea;
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
  boolean_T rtb_LogicalOperator_idx_0;

  /* S-Function (fcncallgen): '<S3>/10ms7' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* SignalConversion generated from: '<S105>/Constant' */
  HVCUTOFF = true;

  /* S-Function (ec5744_swislbu3): '<S105>/SwitchInput' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240918_2018b_B.Drive_ready= ec_gpio_read(92);

  /* Logic: '<S105>/Logical Operator' */
  rtb_ignition = !VehCtrlMdel240918_2018b_B.Drive_ready;

  /* Chart: '<S105>/Timer' incorporates:
   *  Constant: '<S105>/Constant5'
   */
  VehCtrlMdel240918_201_Timer(rtb_ignition, 0.11F, &ignition,
    &VehCtrlMdel240918_2018b_DW.sf_Timer);

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms7' */

  /* S-Function (fcncallgen): '<S3>/10ms6' incorporates:
   *  SubSystem: '<S3>/EMRAXMCU_RECIEVE'
   */
  /* S-Function (ec5744_canreceiveslb): '<S102>/CANReceive1' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF1RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf1looprx= 0;
    VehCtrlMdel240918_2018b_B.CANReceive1_o3= 218089455;
    VehCtrlMdel240918_2018b_B.CANReceive1_o5= 8;
    VehCtrlMdel240918_2018b_B.CANReceive1_o2= ec_can_receive(0,1, CAN0BUF1RX);
    VehCtrlMdel240918_2018b_B.CANReceive1_o4[0]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4[1]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4[2]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4[3]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4[4]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4[5]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4[6]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4[7]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
  }

  /* Call the system: <S102>/MCU_pwr */

  /* Output and update for function-call system: '<S102>/MCU_pwr' */

  /* Outputs for Enabled SubSystem: '<S155>/MCU_VCUMeter1' incorporates:
   *  EnablePort: '<S157>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive1_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S157>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Length = 8;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.ID =
        VehCtrlMdel240918_2018b_B.CANReceive1_o3;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Extended = 1;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4[0];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4[1];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4[2];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4[3];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4[4];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4[5];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4[6];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S157>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S157>/CAN Unpack' */
      if ((8 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Length) &&
          (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.ID != INVALID_CAN_ID) )
      {
        if ((218089455 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.ID) &&
            (1U == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Extended) ) {
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[6]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[7]) <<
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[5]) <<
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[0]);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[1]);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_g.Data[3]) <<
                      8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.1;
                VehCtrlMdel240918_2018b_B.voltage = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S155>/MCU_VCUMeter1' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S102>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S102>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF0RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf0looprx= 0;
    VehCtrlMdel240918_2018b_B.CANReceive3_o3= 218089199;
    VehCtrlMdel240918_2018b_B.CANReceive3_o5= 8;
    VehCtrlMdel240918_2018b_B.CANReceive3_o2= ec_can_receive(0,0, CAN0BUF0RX);
    VehCtrlMdel240918_2018b_B.CANReceive3_o4[0]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4[1]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4[2]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4[3]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4[4]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4[5]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4[6]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4[7]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
  }

  /* Call the system: <S102>/MCU_state */

  /* Output and update for function-call system: '<S102>/MCU_state' */

  /* Outputs for Enabled SubSystem: '<S156>/MCU_state' incorporates:
   *  EnablePort: '<S162>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive3_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S162>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Length = 8;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4.ID =
        VehCtrlMdel240918_2018b_B.CANReceive3_o3;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Extended = 1;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4[0];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4[1];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4[2];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4[3];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4[4];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4[5];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4[6];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S162>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S162>/CAN Unpack' */
      if ((8 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Length) &&
          (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.ID != INVALID_CAN_ID) ) {
        if ((218089199 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4.ID) && (1U
             == VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Extended) ) {
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[5]) &
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[7]) &
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x40U)) >> 6);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.low_VOL = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[5]) &
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[6]) &
                      (uint8_T)(0x1U));
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.MCU_Temp_error = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[4]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.Mode = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[6]) &
                      (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.motorTemp_error = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.overCurrent = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x80U)) >> 7);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.overpower = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x10U)) >> 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.overvol = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.Precharge = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.Reslove_error = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[6]) &
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[0]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[1]) << 8);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4.Data[3]) << 8);
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

  /* End of Outputs for SubSystem: '<S156>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S102>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms6' */

  /* S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S98>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF16RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf16looprx= 0;
    VehCtrlMdel240918_2018b_B.CANReceive3_o3_m= 1698;
    VehCtrlMdel240918_2018b_B.CANReceive3_o5_b= 8;
    VehCtrlMdel240918_2018b_B.CANReceive3_o2_m= ec_can_receive(1,16, CAN1BUF16RX);
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[0]= CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[1]= CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[2]= CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[3]= CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[4]= CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[5]= CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[6]= CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[7]= CAN1BUF16RX[can1buf16looprx];
    can1buf16looprx++;
  }

  /* Call the system: <S98>/ABS_BUS_state */

  /* Output and update for function-call system: '<S98>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S106>/IMU_state' incorporates:
   *  EnablePort: '<S107>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive3_o2_m > 0) {
    /* S-Function (ecucoder_canunmessage): '<S107>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Length = 8;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.ID =
        VehCtrlMdel240918_2018b_B.CANReceive3_o3_m;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Extended = 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[0];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[1];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[2];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[3];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[4];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[5];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[6];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_lg[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S107>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S107>/CAN Unpack1' */
      if ((8 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Length) &&
          (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.ID != INVALID_CAN_ID) )
      {
        if ((1698 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.ID) && (0U ==
             VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Extended) ) {
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[1]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[0]) <<
                      8);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[3]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[2]) <<
                      8);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[5]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[4]) <<
                      8);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[7]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_ja.Data[6]) <<
                      8);
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

  /* End of Outputs for SubSystem: '<S106>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S98>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms3' */

  /* S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S104>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF7RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf7looprx= 0;
    VehCtrlMdel240918_2018b_B.CANReceive3_o3_c= 330;
    VehCtrlMdel240918_2018b_B.CANReceive3_o5_d= 8;
    VehCtrlMdel240918_2018b_B.CANReceive3_o2_p= ec_can_receive(1,7, CAN1BUF7RX);
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[0]= CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[1]= CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[2]= CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[3]= CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[4]= CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[5]= CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[6]= CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[7]= CAN1BUF7RX[can1buf7looprx];
    can1buf7looprx++;
  }

  /* Call the system: <S104>/StrWhSnis_state */

  /* Output and update for function-call system: '<S104>/StrWhSnis_state' */

  /* Outputs for Enabled SubSystem: '<S174>/IMU_state' incorporates:
   *  EnablePort: '<S175>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive3_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S175>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Length = 8;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.ID =
        VehCtrlMdel240918_2018b_B.CANReceive3_o3_c;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Extended = 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[0];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[1];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[2];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[3];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[4];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[5];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[6];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_k[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S175>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S175>/CAN Unpack1' */
      if ((8 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Length) &&
          (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.ID != INVALID_CAN_ID) )
      {
        if ((330 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.ID) && (0U ==
             VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Extended) ) {
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Data[0]);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Data[4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Data[3]) <<
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_l.Data[2]);
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

  /* End of Outputs for SubSystem: '<S174>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S104>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms4' */

  /* S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S112>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF1RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf1looprx= 0;
    VehCtrlMdel240918_2018b_B.CANReceive3_o3_e= 640;
    VehCtrlMdel240918_2018b_B.CANReceive3_o5_a= 8;
    VehCtrlMdel240918_2018b_B.CANReceive3_o2_l= ec_can_receive(1,1, CAN1BUF1RX);
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[0]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[1]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[2]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[3]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[4]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[5]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[6]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[7]= CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
  }

  /* Call the system: <S112>/AMKMCU_state */

  /* Output and update for function-call system: '<S112>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S114>/MCU_state' incorporates:
   *  EnablePort: '<S117>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive3_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S117>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Length = 8;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.ID =
        VehCtrlMdel240918_2018b_B.CANReceive3_o3_e;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Extended = 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[0];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[1];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[2];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[3];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[4];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[5];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[6];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_l[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S117>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S117>/CAN Unpack' */
      if ((8 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Length) &&
          (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.ID != INVALID_CAN_ID) )
      {
        if ((640 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.ID) && (0U ==
             VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Extended) ) {
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[5]) <<
                      8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.0098;
                VehCtrlMdel240918_2018b_B.MCFL_ActualTorque = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[6]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[7]) <<
                      8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.MCFL_ActualVelocity_p = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[3]) <<
                      8);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x10U)) >> 4);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x80U)) >> 7);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.MCFL_bDerating = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x2U)) >> 1);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x40U)) >> 6);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x8U)) >> 3);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x20U)) >> 5);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[0]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.MCFL_bReserve = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x1U));
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_e.Data[1]) &
                      (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.MCFL_bWarn = result;
              }
            }
          }
        }
      }
    }

    /* Gain: '<S117>/Gain' */
    MCFL_ActualVelocity = -VehCtrlMdel240918_2018b_B.MCFL_ActualVelocity_p;
  }

  /* End of Outputs for SubSystem: '<S114>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S112>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S112>/CANReceive1' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF2RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf2looprx= 0;
    VehCtrlMdel240918_2018b_B.CANReceive1_o3_n= 642;
    VehCtrlMdel240918_2018b_B.CANReceive1_o5_a= 8;
    VehCtrlMdel240918_2018b_B.CANReceive1_o2_l= ec_can_receive(1,2, CAN1BUF2RX);
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[0]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[1]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[2]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[3]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[4]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[5]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[6]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[7]= CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
  }

  /* Call the system: <S112>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S112>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S115>/MCU_state' incorporates:
   *  EnablePort: '<S126>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive1_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S126>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Length = 8;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.ID =
        VehCtrlMdel240918_2018b_B.CANReceive1_o3_n;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Extended = 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[0];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[1];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[2];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[3];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[4];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[5];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[6];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_c[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S126>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S126>/CAN Unpack' */
      if ((8 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Length) &&
          (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.ID != INVALID_CAN_ID) )
      {
        if ((642 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.ID) && (0U ==
             VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Extended) ) {
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[0]);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[1]) <<
                      8);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[2]) <<
                      16);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[3]) <<
                      24);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.MCFL_DiagnosticNum = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[4]);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[5]) <<
                      8);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[6]) <<
                      16);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_b.Data[7]) <<
                      24);
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

  /* End of Outputs for SubSystem: '<S115>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S112>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S112>/CANReceive2' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF3RX[6]= { 0, 0, 0, 0, 0, 0 };

    uint8 can1buf3looprx= 0;
    VehCtrlMdel240918_2018b_B.CANReceive2_o3= 644;
    VehCtrlMdel240918_2018b_B.CANReceive2_o5= 6;
    VehCtrlMdel240918_2018b_B.CANReceive2_o2= ec_can_receive(1,3, CAN1BUF3RX);
    VehCtrlMdel240918_2018b_B.CANReceive2_o4[0]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive2_o4[1]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive2_o4[2]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive2_o4[3]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive2_o4[4]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive2_o4[5]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
  }

  /* Call the system: <S112>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S112>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S116>/MCU_state' incorporates:
   *  EnablePort: '<S128>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive2_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S128>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Length = 6;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.ID =
        VehCtrlMdel240918_2018b_B.CANReceive2_o3;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Extended = 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive2_o4[0];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive2_o4[1];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive2_o4[2];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive2_o4[3];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive2_o4[4];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive2_o4[5];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S128>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S128>/CAN Unpack' */
      if ((6 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Length) &&
          (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.ID != INVALID_CAN_ID) )
      {
        if ((644 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.ID) && (0U ==
             VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Extended) ) {
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Data[4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Data[5]) <<
                      8);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Data[3]) <<
                      8);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Data[0]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_c.Data[1]) <<
                      8);
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

  /* End of Outputs for SubSystem: '<S116>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S112>/CANReceive2' */

  /* S-Function (ec5744_canreceiveslb): '<S113>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF4RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf4looprx= 0;
    VehCtrlMdel240918_2018b_B.CANReceive3_o3_i= 641;
    VehCtrlMdel240918_2018b_B.CANReceive3_o5_an= 8;
    VehCtrlMdel240918_2018b_B.CANReceive3_o2_a= ec_can_receive(1,4, CAN1BUF4RX);
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[0]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[1]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[2]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[3]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[4]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[5]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[6]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[7]= CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
  }

  /* Call the system: <S113>/AMKMCU_state */

  /* Output and update for function-call system: '<S113>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S132>/MCU_state' incorporates:
   *  EnablePort: '<S135>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive3_o2_a > 0) {
    /* S-Function (ecucoder_canunmessage): '<S135>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Length = 8;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.ID =
        VehCtrlMdel240918_2018b_B.CANReceive3_o3_i;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Extended = 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[0];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[1];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[2];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[3];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[4];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[5];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[6];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_g[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S135>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S135>/CAN Unpack' */
      if ((8 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Length) &&
          (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.ID != INVALID_CAN_ID) )
      {
        if ((641 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.ID) && (0U ==
             VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Extended) ) {
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[5]) <<
                      8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                result = result * 0.0098;
                VehCtrlMdel240918_2018b_B.MCFR_ActualTorque = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[6]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[7]) <<
                      8);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[3]) <<
                      8);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x10U)) >> 4);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x80U)) >> 7);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.MCFR_bDerating = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x2U)) >> 1);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x40U)) >> 6);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.MCFR_bQuitDCOn = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x20U)) >> 5);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[0]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.MCFR_bReserve = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x1U));
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_j.Data[1]) &
                      (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.MCFR_bWarn = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S132>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S113>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S113>/CANReceive1' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF5RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf5looprx= 0;
    VehCtrlMdel240918_2018b_B.CANReceive1_o3_h= 643;
    VehCtrlMdel240918_2018b_B.CANReceive1_o5_j= 8;
    VehCtrlMdel240918_2018b_B.CANReceive1_o2_o= ec_can_receive(1,5, CAN1BUF5RX);
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[0]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[1]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[2]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[3]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[4]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[5]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[6]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[7]= CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
  }

  /* Call the system: <S113>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S113>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S133>/MCU_state' incorporates:
   *  EnablePort: '<S143>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive1_o2_o > 0) {
    /* S-Function (ecucoder_canunmessage): '<S143>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Length = 8;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.ID =
        VehCtrlMdel240918_2018b_B.CANReceive1_o3_h;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Extended = 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[0];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[1];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[2];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[3];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[4];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[5];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[6];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive1_o4_j[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S143>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S143>/CAN Unpack' */
      if ((8 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Length) &&
          (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.ID != INVALID_CAN_ID) )
      {
        if ((643 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.ID) && (0U ==
             VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Extended) ) {
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[0]);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[1]) <<
                      8);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[2]) <<
                      16);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[3]) <<
                      24);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.MCFR_DiagnosticNum = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[4]);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[5]) <<
                      8);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[6]) <<
                      16);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_p.Data[7]) <<
                      24);
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

  /* End of Outputs for SubSystem: '<S133>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S113>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S113>/CANReceive2' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF0RX[6]= { 0, 0, 0, 0, 0, 0 };

    uint8 can1buf0looprx= 0;
    VehCtrlMdel240918_2018b_B.CANReceive2_o3_j= 645;
    VehCtrlMdel240918_2018b_B.CANReceive2_o5_e= 6;
    VehCtrlMdel240918_2018b_B.CANReceive2_o2_p= ec_can_receive(1,0, CAN1BUF0RX);
    VehCtrlMdel240918_2018b_B.CANReceive2_o4_k[0]= CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive2_o4_k[1]= CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive2_o4_k[2]= CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive2_o4_k[3]= CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive2_o4_k[4]= CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive2_o4_k[5]= CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
  }

  /* Call the system: <S113>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S113>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S134>/MCU_state' incorporates:
   *  EnablePort: '<S145>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive2_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S145>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Length = 6;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.ID =
        VehCtrlMdel240918_2018b_B.CANReceive2_o3_j;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Extended = 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive2_o4_k[0];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive2_o4_k[1];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive2_o4_k[2];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive2_o4_k[3];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive2_o4_k[4];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive2_o4_k[5];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S145>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S145>/CAN Unpack' */
      if ((6 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Length) &&
          (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.ID != INVALID_CAN_ID) )
      {
        if ((645 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.ID) && (0U ==
             VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Extended) ) {
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Data[4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Data[5]) <<
                      8);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Data[3]) <<
                      8);
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Data[0]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_n.Data[1]) <<
                      8);
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

  /* End of Outputs for SubSystem: '<S134>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S113>/CANReceive2' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms5' */

  /* S-Function (fcncallgen): '<S3>/10ms' incorporates:
   *  SubSystem: '<S3>/AccBrk_BUS'
   */
  /* S-Function (ec5744_asislbu3): '<S100>/Acc3' */

  /* Read the ADC conversion result of the analog signal */
  Acc1= adc_read_chan(1,2);

  /* S-Function (ec5744_asislbu3): '<S100>/Acc4' */

  /* Read the ADC conversion result of the analog signal */
  Acc2= adc_read_chan(1,4);

  /* S-Function (ec5744_asislbu3): '<S100>/Brk1' */

  /* Read the ADC conversion result of the analog signal */
  Brk1= adc_read_chan(1,0);

  /* S-Function (ec5744_asislbu3): '<S100>/Brk2' */

  /* Read the ADC conversion result of the analog signal */
  Brk2= adc_read_chan(0,13);

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms' */

  /* S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* S-Function (ec5744_canreceiveslb): '<S103>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF17RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf17looprx= 0;
    VehCtrlMdel240918_2018b_B.CANReceive3_o3_cz= 513;
    VehCtrlMdel240918_2018b_B.CANReceive3_o5_m= 8;
    VehCtrlMdel240918_2018b_B.CANReceive3_o2_ma= ec_can_receive(1,17,
      CAN1BUF17RX);
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[0]= CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[1]= CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[2]= CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[3]= CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[4]= CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[5]= CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[6]= CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[7]= CAN1BUF17RX[can1buf17looprx];
    can1buf17looprx++;
  }

  /* Call the system: <S103>/IMU_state */

  /* Output and update for function-call system: '<S103>/IMU_state' */

  /* Outputs for Enabled SubSystem: '<S169>/MCU_state' incorporates:
   *  EnablePort: '<S170>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive3_o2_ma > 0) {
    /* S-Function (ecucoder_canunmessage): '<S170>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Length = 8;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.ID =
        VehCtrlMdel240918_2018b_B.CANReceive3_o3_cz;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Extended = 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[0];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[1];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[2];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[3];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[4];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[5];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[6];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_i[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S170>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S170>/CAN Unpack' */
      if ((8 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Length) &&
          (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.ID != INVALID_CAN_ID) )
      {
        if ((513 == VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.ID) && (0U ==
             VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Extended) ) {
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[0]) &
                      (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.CANUnpack_o1 = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[5]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[6]) <<
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[0]) &
                      (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.CANUnpack_o3 = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[3]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[4]) <<
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[0]) &
                      (uint8_T)(0xF0U)) >> 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.CANUnpack_o5 = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[0]) &
                      (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel240918_2018b_B.CANUnpack_o6 = result;
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
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[1]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel240918_2018b_B.CANUnPackMessage4_h.Data[2]) <<
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

  /* End of Outputs for SubSystem: '<S169>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S103>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms2' */

  /* S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S101>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF3RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf3looprx= 0;
    VehCtrlMdel240918_2018b_B.CANReceive3_o3_l= 408961267;
    VehCtrlMdel240918_2018b_B.CANReceive3_o5_de= 8;
    VehCtrlMdel240918_2018b_B.CANReceive3_o2_k= ec_can_receive(0,3, CAN0BUF3RX);
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[0]= CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[1]= CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[2]= CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[3]= CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[4]= CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[5]= CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[6]= CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[7]= CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
  }

  /* Call the system: <S101>/ABS_BUS_state */

  /* Output and update for function-call system: '<S101>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S153>/IMU_state' incorporates:
   *  EnablePort: '<S154>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive3_o2_k > 0) {
    /* S-Function (ecucoder_canunmessage): '<S154>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_i.Length = 8;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_i.ID =
        VehCtrlMdel240918_2018b_B.CANReceive3_o3_l;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_i.Extended = 1;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[0];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[1];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[2];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[3];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[4];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[5];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[6];
      canunpackloop++;
      VehCtrlMdel240918_2018b_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel240918_2018b_B.CANReceive3_o4_p[7];
      canunpackloop++;
    }
  }

  /* End of Outputs for SubSystem: '<S153>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S101>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms1' */

  /* S-Function (fcncallgen): '<S4>/10ms' incorporates:
   *  SubSystem: '<S4>/Function-Call Subsystem'
   */
  if (VehCtrlMdel240918_2018b_DW.FunctionCallSubsystem_RESET_ELA) {
    FunctionCallSubsystem_ELAPS_T = 0U;
  } else {
    FunctionCallSubsystem_ELAPS_T = VehCtrlMdel240918_2018b_M->Timing.clockTick3
      - VehCtrlMdel240918_2018b_DW.FunctionCallSubsystem_PREV_T;
  }

  VehCtrlMdel240918_2018b_DW.FunctionCallSubsystem_PREV_T =
    VehCtrlMdel240918_2018b_M->Timing.clockTick3;
  VehCtrlMdel240918_2018b_DW.FunctionCallSubsystem_RESET_ELA = false;

  /* Lookup_n-D: '<S184>/1-D Lookup Table1' */
  F_BrkPrs = look1_iu16bflftfIu16_binlc(Brk1,
    VehCtrlMdel240918_2018b_ConstP.pooled65,
    VehCtrlMdel240918_2018b_ConstP.pooled65, 1U);

  /* DataTypeConversion: '<S184>/Data Type Conversion' */
  rtb_Add = F_BrkPrs;

  /* SignalConversion generated from: '<S182>/Out1' */
  Brk_F = (int32_T)rtb_Add;

  /* Gain: '<S184>/Gain2' */
  rtb_Gain1 = 45875U * Acc2;

  /* Gain: '<S184>/Gain3' incorporates:
   *  UnitDelay: '<S184>/Unit Delay1'
   */
  rtb_Gain = 39322U * VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S184>/Add3' */
  Acc_vol2 = (rtb_Gain >> 1) + rtb_Gain1;

  /* RelationalOperator: '<S192>/Compare' */
  rtb_ignition = (Acc_vol2 <= 32768000U);

  /* RelationalOperator: '<S193>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol2 >= 294912000U);

  /* Logic: '<S184>/Logical Operator1' */
  rtb_ignition = (rtb_ignition || rtb_LogicalOperator2);

  /* Gain: '<S184>/Gain' */
  rtb_Gain = 45875U * Acc1;

  /* UnitDelay: '<S184>/Unit Delay' incorporates:
   *  UnitDelay: '<S184>/Unit Delay1'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_fm =
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_k;

  /* Gain: '<S184>/Gain1' incorporates:
   *  UnitDelay: '<S184>/Unit Delay1'
   */
  rtb_Gain1 = 39322U * VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S184>/Add2' */
  Acc_vol = (rtb_Gain1 >> 1) + rtb_Gain;

  /* RelationalOperator: '<S188>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol <= 32768000U);

  /* RelationalOperator: '<S189>/Compare' */
  rtb_Compare = (Acc_vol >= 294912000U);

  /* Logic: '<S184>/Logical Operator' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_Compare);

  /* Logic: '<S184>/Logical Operator2' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_ignition);

  /* DataTypeConversion: '<S184>/Data Type Conversion2' */
  rtb_Add = (real32_T)Acc_vol * 1.52587891E-5F;

  /* MATLAB Function: '<S184>/MATLAB Function' */
  Acc_POS = fmaxf(fminf((rtb_Add - 3060.0F) * -0.4F, 100.0F), 0.0F);

  /* Lookup_n-D: '<S184>/1-D Lookup Table3' */
  Acc_POS2 = look1_iu32n16bflftfIu32_binlc(Acc_vol2,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable3_bp01Data,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable3_tableData, 1U);

  /* DataTypeConversion: '<S184>/Data Type Conversion4' */
  rtb_Add = (real32_T)Acc_POS2 * 1.52587891E-5F;

  /* Sum: '<S184>/Add1' */
  rtb_Acc_POS = Acc_POS - rtb_Add;

  /* Abs: '<S184>/Abs' */
  rtb_Acc_POS = fabsf(rtb_Acc_POS);

  /* RelationalOperator: '<S196>/Compare' incorporates:
   *  Constant: '<S196>/Constant'
   */
  rtb_Compare = (rtb_Acc_POS > 15.0F);

  /* RelationalOperator: '<S194>/Compare' incorporates:
   *  Constant: '<S194>/Constant'
   */
  rtb_ignition = (Acc_POS > 100.0F);

  /* RelationalOperator: '<S195>/Compare' incorporates:
   *  Constant: '<S195>/Constant'
   */
  rtb_Compare_am = (rtb_Add > 100.0F);

  /* Logic: '<S184>/Logical Operator3' */
  rtb_ignition = (rtb_ignition || rtb_Compare_am);

  /* RelationalOperator: '<S197>/Compare' incorporates:
   *  Constant: '<S197>/Constant'
   */
  rtb_Compare_am = (Brk1 <= 300);

  /* RelationalOperator: '<S198>/Compare' incorporates:
   *  Constant: '<S198>/Constant'
   */
  rtb_LowerRelop1_b = (Brk1 >= 4500);

  /* Logic: '<S184>/Logical Operator5' */
  rtb_Compare_am = (rtb_Compare_am || rtb_LowerRelop1_b);

  /* Logic: '<S184>/Logical Operator4' */
  rtb_ignition = (rtb_ignition || rtb_Compare || rtb_LogicalOperator2 ||
                  rtb_Compare_am);

  /* Chart: '<S184>/Timer' incorporates:
   *  Constant: '<S184>/Constant1'
   */
  VehCtrlMdel240918_201_Timer(rtb_ignition, 0.11F, &Trq_CUT,
    &VehCtrlMdel240918_2018b_DW.sf_Timer_a);

  /* UnitDelay: '<S219>/Delay Input2'
   *
   * Block description for '<S219>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE;

  /* SampleTimeMath: '<S219>/sample time'
   *
   * About '<S219>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S219>/delta rise limit' */
  rtb_Gain5 = 1200.0 * elapseTime;

  /* Sum: '<S219>/Difference Inputs1'
   *
   * Block description for '<S219>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = StrWhlAng - rtb_Yk1_l;

  /* RelationalOperator: '<S222>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain5);

  /* Switch: '<S222>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S219>/delta fall limit' */
    rtb_deltafalllimit_iz = -1200.0 * elapseTime;

    /* RelationalOperator: '<S222>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S222>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S222>/Switch' */
    rtb_Gain5 = rtb_UkYk1;
  }

  /* End of Switch: '<S222>/Switch2' */

  /* Sum: '<S219>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S219>/Delay Input2'
   *
   * Block description for '<S219>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S219>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE = rtb_Gain5 + rtb_Yk1_l;

  /* Abs: '<S186>/Abs' incorporates:
   *  UnitDelay: '<S219>/Delay Input2'
   *
   * Block description for '<S219>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain5 = fabs(VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE);

  /* RelationalOperator: '<S218>/Compare' incorporates:
   *  Constant: '<S218>/Constant'
   */
  rtb_ignition = (rtb_Gain5 > 120.0);

  /* Chart: '<S186>/Timer' incorporates:
   *  Constant: '<S186>/Constant5'
   */
  VehCtrlMdel240918_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_on, &VehCtrlMdel240918_2018b_DW.sf_Timer_k);

  /* UnitDelay: '<S232>/Delay Input2'
   *
   * Block description for '<S232>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain5 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_a;

  /* SampleTimeMath: '<S232>/sample time'
   *
   * About '<S232>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S232>/delta rise limit' */
  rtb_Yk1_l = 10.0 * elapseTime;

  /* Sum: '<S232>/Difference Inputs1'
   *
   * Block description for '<S232>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RL - rtb_Gain5;

  /* RelationalOperator: '<S240>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Yk1_l);

  /* Switch: '<S240>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S232>/delta fall limit' */
    rtb_Yk1_l = -10.0 * elapseTime;

    /* RelationalOperator: '<S240>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_Yk1_l);

    /* Switch: '<S240>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_Yk1_l;
    }

    /* End of Switch: '<S240>/Switch' */
    rtb_Yk1_l = rtb_UkYk1;
  }

  /* End of Switch: '<S240>/Switch2' */

  /* Saturate: '<S187>/Saturation' incorporates:
   *  Sum: '<S232>/Difference Inputs2'
   *  UnitDelay: '<S232>/Delay Input2'
   *
   * Block description for '<S232>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S232>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_a = rtb_Yk1_l + rtb_Gain5;
  if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_a > 30.0) {
    rtb_Gain5 = 30.0;
  } else if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_a < 0.0) {
    rtb_Gain5 = 0.0;
  } else {
    rtb_Gain5 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_a;
  }

  /* End of Saturate: '<S187>/Saturation' */

  /* Gain: '<S187>/Gain' */
  rtb_Gain5 *= 0.27777777777777779;

  /* RelationalOperator: '<S224>/Compare' incorporates:
   *  Constant: '<S224>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain5 >= 0.0);

  /* RelationalOperator: '<S225>/Compare' incorporates:
   *  Constant: '<S225>/Constant'
   */
  rtb_Compare_am = (rtb_Gain5 < 40.0);

  /* Logic: '<S187>/OR' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S187>/Timer' incorporates:
   *  Constant: '<S187>/Constant5'
   */
  VehCtrlMdel240918_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_le, &VehCtrlMdel240918_2018b_DW.sf_Timer_b);

  /* UnitDelay: '<S233>/Delay Input2'
   *
   * Block description for '<S233>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_b;

  /* SampleTimeMath: '<S233>/sample time'
   *
   * About '<S233>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S233>/delta rise limit' */
  rtb_Gain4 = 10.0 * elapseTime;

  /* Sum: '<S233>/Difference Inputs1'
   *
   * Block description for '<S233>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RR - rtb_Yk1_l;

  /* RelationalOperator: '<S241>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain4);

  /* Switch: '<S241>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S233>/delta fall limit' */
    rtb_deltafalllimit_iz = -10.0 * elapseTime;

    /* RelationalOperator: '<S241>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S241>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S241>/Switch' */
    rtb_Gain4 = rtb_UkYk1;
  }

  /* End of Switch: '<S241>/Switch2' */

  /* Saturate: '<S187>/Saturation1' incorporates:
   *  Sum: '<S233>/Difference Inputs2'
   *  UnitDelay: '<S233>/Delay Input2'
   *
   * Block description for '<S233>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S233>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_b = rtb_Gain4 + rtb_Yk1_l;
  if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_b > 30.0) {
    rtb_Gain4 = 30.0;
  } else if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_b < 0.0) {
    rtb_Gain4 = 0.0;
  } else {
    rtb_Gain4 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_b;
  }

  /* End of Saturate: '<S187>/Saturation1' */

  /* Gain: '<S187>/Gain3' */
  rtb_Gain4 *= 0.27777777777777779;

  /* RelationalOperator: '<S226>/Compare' incorporates:
   *  Constant: '<S226>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain4 >= 0.0);

  /* RelationalOperator: '<S227>/Compare' incorporates:
   *  Constant: '<S227>/Constant'
   */
  rtb_Compare_am = (rtb_Gain4 < 40.0);

  /* Logic: '<S187>/OR1' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S187>/Timer1' incorporates:
   *  Constant: '<S187>/Constant1'
   */
  VehCtrlMdel240918_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_i, &VehCtrlMdel240918_2018b_DW.sf_Timer1_n);

  /* UnitDelay: '<S234>/Delay Input2'
   *
   * Block description for '<S234>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_h;

  /* SampleTimeMath: '<S234>/sample time'
   *
   * About '<S234>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S234>/delta rise limit' */
  rtb_Switch2_on = 10.0 * elapseTime;

  /* Sum: '<S234>/Difference Inputs1'
   *
   * Block description for '<S234>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = MCFR_ActualVelocity - rtb_Yk1_l;

  /* RelationalOperator: '<S242>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Switch2_on);

  /* Switch: '<S242>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S234>/delta fall limit' */
    rtb_deltafalllimit_iz = -10.0 * elapseTime;

    /* RelationalOperator: '<S242>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S242>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S242>/Switch' */
    rtb_Switch2_on = rtb_UkYk1;
  }

  /* End of Switch: '<S242>/Switch2' */

  /* Saturate: '<S187>/Saturation2' incorporates:
   *  Sum: '<S234>/Difference Inputs2'
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_h = rtb_Switch2_on + rtb_Yk1_l;
  if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_h > 30.0) {
    rtb_Switch2_on = 30.0;
  } else if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_h < 0.0) {
    rtb_Switch2_on = 0.0;
  } else {
    rtb_Switch2_on = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_h;
  }

  /* End of Saturate: '<S187>/Saturation2' */

  /* Gain: '<S187>/Gain1' */
  rtb_Switch2_on *= 0.1341030088495575;

  /* RelationalOperator: '<S228>/Compare' incorporates:
   *  Constant: '<S228>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Switch2_on >= 0.0);

  /* RelationalOperator: '<S229>/Compare' incorporates:
   *  Constant: '<S229>/Constant'
   */
  rtb_Compare_am = (rtb_Switch2_on < 40.0);

  /* Logic: '<S187>/OR2' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S187>/Timer2' incorporates:
   *  Constant: '<S187>/Constant4'
   */
  VehCtrlMdel240918_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_o, &VehCtrlMdel240918_2018b_DW.sf_Timer2_l);

  /* UnitDelay: '<S235>/Delay Input2'
   *
   * Block description for '<S235>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n;

  /* SampleTimeMath: '<S235>/sample time'
   *
   * About '<S235>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S235>/delta rise limit' */
  rtb_UkYk1 = 10.0 * elapseTime;

  /* Sum: '<S235>/Difference Inputs1'
   *
   * Block description for '<S235>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_iz = MCFL_ActualVelocity - rtb_Yk1_l;

  /* RelationalOperator: '<S243>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_iz > rtb_UkYk1);

  /* Switch: '<S243>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S235>/delta fall limit' */
    rtb_UkYk1 = -10.0 * elapseTime;

    /* RelationalOperator: '<S243>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_iz < rtb_UkYk1);

    /* Switch: '<S243>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_iz = rtb_UkYk1;
    }

    /* End of Switch: '<S243>/Switch' */
    rtb_UkYk1 = rtb_deltafalllimit_iz;
  }

  /* End of Switch: '<S243>/Switch2' */

  /* Saturate: '<S187>/Saturation3' incorporates:
   *  Sum: '<S235>/Difference Inputs2'
   *  UnitDelay: '<S235>/Delay Input2'
   *
   * Block description for '<S235>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S235>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n = rtb_UkYk1 + rtb_Yk1_l;
  if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n > 30.0) {
    rtb_UkYk1 = 30.0;
  } else if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n < 0.0) {
    rtb_UkYk1 = 0.0;
  } else {
    rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n;
  }

  /* End of Saturate: '<S187>/Saturation3' */

  /* Gain: '<S187>/Gain2' */
  rtb_UkYk1 *= 0.1341030088495575;

  /* RelationalOperator: '<S230>/Compare' incorporates:
   *  Constant: '<S230>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_UkYk1 >= 0.0);

  /* RelationalOperator: '<S231>/Compare' incorporates:
   *  Constant: '<S231>/Constant'
   */
  rtb_Compare_am = (rtb_UkYk1 < 40.0);

  /* Logic: '<S187>/OR3' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S187>/Timer3' incorporates:
   *  Constant: '<S187>/Constant8'
   */
  VehCtrlMdel240918_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_h, &VehCtrlMdel240918_2018b_DW.sf_Timer3);

  /* SignalConversion generated from: '<S182>/Out1' */
  WhlSpdFL = rtb_UkYk1;

  /* SignalConversion generated from: '<S182>/Out1' */
  WhlSpdFR = rtb_Switch2_on;

  /* SignalConversion generated from: '<S182>/Out1' */
  WhlSpdRR_mps = rtb_Gain4;

  /* SignalConversion generated from: '<S182>/Out1' */
  WhlSpdRL_mps = rtb_Gain5;

  /* Gain: '<S186>/Gain' incorporates:
   *  UnitDelay: '<S219>/Delay Input2'
   *
   * Block description for '<S219>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = 0.7 * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE;

  /* UnitDelay: '<S186>/Unit Delay' */
  rtb_Switch2_on = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE;

  /* Gain: '<S186>/Gain1' */
  rtb_Switch2_on *= 0.3;

  /* Sum: '<S186>/Add2' */
  rtb_UkYk1 += rtb_Switch2_on;

  /* Lookup_n-D: '<S186>/1-D Lookup Table' */
  rtb_Switch2_on = look1_binlx(rtb_UkYk1,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable_bp01Data,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable_tableData, 23U);

  /* SignalConversion generated from: '<S182>/Out1' */
  FLWhlStrAng = rtb_Switch2_on;

  /* Lookup_n-D: '<S186>/1-D Lookup Table1' */
  rtb_Switch2_on = look1_binlx(rtb_UkYk1,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable1_bp01Data,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable1_tableData, 23U);

  /* SignalConversion generated from: '<S182>/Out1' */
  rtb_Yk1_l = rtb_Switch2_on;

  /* SignalConversion generated from: '<S182>/Out1' */
  rtb_deltafalllimit_iz = rtb_UkYk1;

  /* Sum: '<S184>/Add' */
  rtb_Add += Acc_POS;

  /* Product: '<S184>/Divide' incorporates:
   *  Constant: '<S184>/Constant'
   */
  rtb_Acc_POS = (real32_T)(rtb_Add / 2.0);

  /* UnitDelay: '<S209>/Delay Input2'
   *
   * Block description for '<S209>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_l;

  /* SampleTimeMath: '<S209>/sample time'
   *
   * About '<S209>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S209>/delta rise limit' incorporates:
   *  Constant: '<S208>/Constant'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* UnitDelay: '<S208>/Unit Delay' */
  rtb_Gain4 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_g;

  /* Gain: '<S208>/Gain1' */
  rtb_Gain4 *= 0.3;

  /* Gain: '<S185>/g_mpss' incorporates:
   *  UnitDelay: '<S208>/Unit Delay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_g = 9.8 * IMU_Ay_Value;

  /* Gain: '<S208>/Gain' incorporates:
   *  UnitDelay: '<S208>/Unit Delay'
   */
  rtb_Gain5 = 0.7 * VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_g;

  /* Sum: '<S208>/Add2' */
  rtb_Add2 = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S209>/Difference Inputs1'
   *
   * Block description for '<S209>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add2 -= rtb_UkYk1;

  /* RelationalOperator: '<S215>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Add2 > rtb_Switch2_on);

  /* Switch: '<S215>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S209>/delta fall limit' incorporates:
     *  Constant: '<S208>/Constant1'
     */
    rtb_deltafalllimit_i4 = -5000.0 * elapseTime;

    /* RelationalOperator: '<S215>/UpperRelop' */
    rtb_ignition = (rtb_Add2 < rtb_deltafalllimit_i4);

    /* Switch: '<S215>/Switch' */
    if (rtb_ignition) {
      rtb_Add2 = rtb_deltafalllimit_i4;
    }

    /* End of Switch: '<S215>/Switch' */
    rtb_Switch2_on = rtb_Add2;
  }

  /* End of Switch: '<S215>/Switch2' */

  /* Sum: '<S209>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S209>/Delay Input2'
   *
   * Block description for '<S209>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S209>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_l = rtb_Switch2_on + rtb_UkYk1;

  /* RelationalOperator: '<S212>/LowerRelop1' incorporates:
   *  Constant: '<S208>/Constant6'
   *  UnitDelay: '<S209>/Delay Input2'
   *
   * Block description for '<S209>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_l > 1.5);

  /* Switch: '<S212>/Switch2' incorporates:
   *  Constant: '<S208>/Constant6'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S212>/UpperRelop' incorporates:
     *  Constant: '<S208>/Constant7'
     *  UnitDelay: '<S209>/Delay Input2'
     *
     * Block description for '<S209>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_ignition = (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_l < -1.5);

    /* Switch: '<S212>/Switch' incorporates:
     *  Constant: '<S208>/Constant7'
     *  UnitDelay: '<S209>/Delay Input2'
     *
     * Block description for '<S209>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_ignition) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_l;
    }

    /* End of Switch: '<S212>/Switch' */
  }

  /* End of Switch: '<S212>/Switch2' */

  /* SignalConversion generated from: '<S182>/Out1' */
  rtb_Add2 = rtb_UkYk1;

  /* UnitDelay: '<S210>/Delay Input2'
   *
   * Block description for '<S210>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_m;

  /* SampleTimeMath: '<S210>/sample time'
   *
   * About '<S210>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S210>/delta rise limit' incorporates:
   *  Constant: '<S208>/Constant2'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S185>/g_mpss1' */
  rtb_g_mpss1 = 9.8 * IMU_Ax_Value;

  /* Gain: '<S208>/Gain2' */
  rtb_Gain4 = 0.7 * rtb_g_mpss1;

  /* UnitDelay: '<S208>/Unit Delay1' */
  rtb_Gain5 = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE;

  /* Gain: '<S208>/Gain3' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S208>/Add1' */
  rtb_deltafalllimit_i4 = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S210>/Difference Inputs1'
   *
   * Block description for '<S210>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_i4 -= rtb_UkYk1;

  /* RelationalOperator: '<S216>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_i4 > rtb_Switch2_on);

  /* Switch: '<S216>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S210>/delta fall limit' incorporates:
     *  Constant: '<S208>/Constant4'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S216>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_i4 < elapseTime);

    /* Switch: '<S216>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_i4 = elapseTime;
    }

    /* End of Switch: '<S216>/Switch' */
    rtb_Switch2_on = rtb_deltafalllimit_i4;
  }

  /* End of Switch: '<S216>/Switch2' */

  /* Sum: '<S210>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S210>/Delay Input2'
   *
   * Block description for '<S210>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S210>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_m = rtb_Switch2_on + rtb_UkYk1;

  /* RelationalOperator: '<S213>/LowerRelop1' incorporates:
   *  Constant: '<S208>/Constant8'
   *  UnitDelay: '<S210>/Delay Input2'
   *
   * Block description for '<S210>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_m > 1.5);

  /* Switch: '<S213>/Switch2' incorporates:
   *  Constant: '<S208>/Constant8'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S213>/UpperRelop' incorporates:
     *  Constant: '<S208>/Constant9'
     *  UnitDelay: '<S210>/Delay Input2'
     *
     * Block description for '<S210>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_ignition = (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_m < -1.5);

    /* Switch: '<S213>/Switch' incorporates:
     *  Constant: '<S208>/Constant9'
     *  UnitDelay: '<S210>/Delay Input2'
     *
     * Block description for '<S210>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_ignition) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_m;
    }

    /* End of Switch: '<S213>/Switch' */
  }

  /* End of Switch: '<S213>/Switch2' */

  /* SignalConversion generated from: '<S182>/Out1' */
  rtb_deltafalllimit_i4 = rtb_UkYk1;

  /* SampleTimeMath: '<S220>/sample time'
   *
   * About '<S220>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S220>/delta rise limit' */
  rtb_UkYk1 = 1200.0 * elapseTime;

  /* UnitDelay: '<S220>/Delay Input2'
   *
   * Block description for '<S220>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_on = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_j;

  /* Sum: '<S220>/Difference Inputs1'
   *
   * Block description for '<S220>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1_ix = StrWhlAngV - rtb_Switch2_on;

  /* RelationalOperator: '<S223>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1_ix > rtb_UkYk1);

  /* Switch: '<S223>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S220>/delta fall limit' */
    rtb_UkYk1 = -1200.0 * elapseTime;

    /* RelationalOperator: '<S223>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1_ix < rtb_UkYk1);

    /* Switch: '<S223>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1_ix = rtb_UkYk1;
    }

    /* End of Switch: '<S223>/Switch' */
    rtb_UkYk1 = rtb_UkYk1_ix;
  }

  /* End of Switch: '<S223>/Switch2' */

  /* Saturate: '<S186>/Saturation1' incorporates:
   *  Sum: '<S220>/Difference Inputs2'
   *  UnitDelay: '<S220>/Delay Input2'
   *
   * Block description for '<S220>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S220>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_j = rtb_UkYk1 + rtb_Switch2_on;
  if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_j > 1200.0) {
    rtb_StrWhlAngV = 1200.0;
  } else if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_j < 0.0) {
    rtb_StrWhlAngV = 0.0;
  } else {
    rtb_StrWhlAngV = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_j;
  }

  /* End of Saturate: '<S186>/Saturation1' */

  /* Gain: '<S186>/Gain2' */
  rtb_UkYk1 = 0.7 * rtb_StrWhlAngV;

  /* UnitDelay: '<S186>/Unit Delay1' */
  rtb_Switch2_on = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_a;

  /* Gain: '<S186>/Gain3' */
  rtb_Switch2_on *= 0.3;

  /* Sum: '<S186>/Add1' */
  rtb_UkYk1 += rtb_Switch2_on;

  /* SignalConversion generated from: '<S182>/Out1' */
  rtb_UkYk1_ix = rtb_UkYk1;

  /* UnitDelay: '<S211>/Delay Input2'
   *
   * Block description for '<S211>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_k;

  /* SampleTimeMath: '<S211>/sample time'
   *
   * About '<S211>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S211>/delta rise limit' incorporates:
   *  Constant: '<S208>/Constant3'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S208>/Gain4' */
  rtb_Gain4 = 0.7 * IMU_Yaw_Value;

  /* UnitDelay: '<S208>/Unit Delay2' */
  rtb_Gain5 = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE;

  /* Gain: '<S208>/Gain5' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S208>/Add3' */
  rtb_Gain5 += rtb_Gain4;

  /* Sum: '<S211>/Difference Inputs1'
   *
   * Block description for '<S211>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 -= rtb_UkYk1;

  /* RelationalOperator: '<S217>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Gain5 > rtb_Switch2_on);

  /* Switch: '<S217>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S211>/delta fall limit' incorporates:
     *  Constant: '<S208>/Constant5'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S217>/UpperRelop' */
    rtb_ignition = (rtb_Gain5 < elapseTime);

    /* Switch: '<S217>/Switch' */
    if (rtb_ignition) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S217>/Switch' */
    rtb_Switch2_on = rtb_Gain5;
  }

  /* End of Switch: '<S217>/Switch2' */

  /* Saturate: '<S208>/Saturation2' incorporates:
   *  Sum: '<S211>/Difference Inputs2'
   *  UnitDelay: '<S211>/Delay Input2'
   *
   * Block description for '<S211>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S211>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_k = rtb_Switch2_on + rtb_UkYk1;
  if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_k > 180.0) {
    rtb_UkYk1 = 180.0;
  } else if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_k < -180.0) {
    rtb_UkYk1 = -180.0;
  } else {
    rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_k;
  }

  /* End of Saturate: '<S208>/Saturation2' */

  /* Update for UnitDelay: '<S184>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_fm = Acc2;

  /* Update for UnitDelay: '<S184>/Unit Delay' */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_k = Acc1;

  /* Update for UnitDelay: '<S186>/Unit Delay' incorporates:
   *  UnitDelay: '<S219>/Delay Input2'
   *
   * Block description for '<S219>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE;

  /* Update for UnitDelay: '<S208>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE = rtb_g_mpss1;

  /* Update for UnitDelay: '<S186>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_a = rtb_StrWhlAngV;

  /* Update for UnitDelay: '<S208>/Unit Delay2' */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE = IMU_Yaw_Value;

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms' */

  /* S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  if (VehCtrlMdel240918_2018b_DW.Subsystem_RESET_ELAPS_T) {
    FunctionCallSubsystem_ELAPS_T = 0U;
  } else {
    FunctionCallSubsystem_ELAPS_T = VehCtrlMdel240918_2018b_M->Timing.clockTick3
      - VehCtrlMdel240918_2018b_DW.Subsystem_PREV_T;
  }

  VehCtrlMdel240918_2018b_DW.Subsystem_PREV_T =
    VehCtrlMdel240918_2018b_M->Timing.clockTick3;
  VehCtrlMdel240918_2018b_DW.Subsystem_RESET_ELAPS_T = false;

  /* Gain: '<S183>/Gain5' */
  elapseTime = 10.0 * VehCtrlMdel240918_2018b_B.CANUnpack_o1;

  /* DataTypeConversion: '<S183>/Cast To Double' */
  rtb_CastToDouble = (real32_T)elapseTime;

  /* MinMax: '<S307>/Min3' incorporates:
   *  Gain: '<S261>/Gain'
   *  UnitDelay: '<S261>/Unit Delay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p *= 0.3F;

  /* UnitDelay: '<S265>/Delay Input2'
   *
   * Block description for '<S265>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_b0 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n2;

  /* SampleTimeMath: '<S265>/sample time'
   *
   * About '<S265>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S265>/delta rise limit' */
  rtb_Add4_j = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S183>/Cast To Double1' */
  rtb_Add7 = (real32_T)WhlSpdFL;

  /* DataTypeConversion: '<S183>/Cast To Double6' */
  rtb_Add6_p = (real32_T)FLWhlStrAng;

  /* Gain: '<S247>/Gain2' */
  rtb_Add6_p *= 0.0174532924F;

  /* Trigonometry: '<S247>/Asin' */
  rtb_Add6_p = cosf(rtb_Add6_p);

  /* Product: '<S247>/Product1' */
  rtb_Add7 *= rtb_Add6_p;

  /* DataTypeConversion: '<S183>/Cast To Double5' */
  rtb_Add6_p = (real32_T)rtb_UkYk1;

  /* Gain: '<S247>/Gain4' */
  rtb_Add6_p *= 0.0174532924F;

  /* Product: '<S247>/Product3' */
  rtb_Switch2_mn = 0.6F * rtb_Add6_p;

  /* Sum: '<S247>/Add2' */
  rtb_Add = rtb_Add7 - rtb_Switch2_mn;

  /* UnitDelay: '<S254>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_j;

  /* Sum: '<S254>/Add4' */
  rtb_Add7 = rtb_Add - rtb_Add7;

  /* Product: '<S254>/Divide' incorporates:
   *  Constant: '<S254>/steptime'
   */
  rtb_Divide = rtb_Add7 / 0.01F;

  /* RelationalOperator: '<S266>/LowerRelop1' incorporates:
   *  Constant: '<S261>/Constant1'
   */
  rtb_LogicalOperator2 = (rtb_Divide > 100.0F);

  /* Switch: '<S266>/Switch2' incorporates:
   *  Constant: '<S261>/Constant1'
   */
  if (rtb_LogicalOperator2) {
    rtb_Divide = 100.0F;
  } else {
    /* RelationalOperator: '<S266>/UpperRelop' incorporates:
     *  Constant: '<S261>/Constant'
     */
    rtb_ignition = (rtb_Divide < -100.0F);

    /* Switch: '<S266>/Switch' incorporates:
     *  Constant: '<S261>/Constant'
     */
    if (rtb_ignition) {
      rtb_Divide = -100.0F;
    }

    /* End of Switch: '<S266>/Switch' */
  }

  /* End of Switch: '<S266>/Switch2' */

  /* Sum: '<S265>/Difference Inputs1'
   *
   * Block description for '<S265>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Divide -= rtb_Switch2_b0;

  /* RelationalOperator: '<S267>/LowerRelop1' */
  rtb_LogicalOperator2 = (rtb_Divide > rtb_Add4_j);

  /* Switch: '<S267>/Switch2' */
  if (!rtb_LogicalOperator2) {
    /* Product: '<S265>/delta fall limit' */
    rtb_deltafalllimit_n = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S267>/UpperRelop' */
    rtb_ignition = (rtb_Divide < rtb_deltafalllimit_n);

    /* Switch: '<S267>/Switch' */
    if (rtb_ignition) {
      rtb_Divide = rtb_deltafalllimit_n;
    }

    /* End of Switch: '<S267>/Switch' */
    rtb_Add4_j = rtb_Divide;
  }

  /* End of Switch: '<S267>/Switch2' */

  /* Sum: '<S265>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S265>/Delay Input2'
   *
   * Block description for '<S265>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S265>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n2 = rtb_Add4_j + rtb_Switch2_b0;

  /* Gain: '<S261>/Gain1' incorporates:
   *  UnitDelay: '<S265>/Delay Input2'
   *
   * Block description for '<S265>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add7 = 0.7F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n2;

  /* MinMax: '<S307>/Min3' incorporates:
   *  Abs: '<S254>/Abs'
   *  Sum: '<S254>/Add'
   *  Sum: '<S261>/Add'
   *  UnitDelay: '<S261>/Unit Delay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p += rtb_Add7;
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p -= rtb_CastToDouble;
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p = fabsf
    (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p);

  /* RelationalOperator: '<S257>/Compare' incorporates:
   *  Constant: '<S257>/Constant'
   *  UnitDelay: '<S261>/Unit Delay'
   */
  rtb_LogicalOperator2 = (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p <= 0.5F);

  /* UnitDelay: '<S262>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_pj;

  /* Gain: '<S262>/Gain' */
  rtb_Add7 *= 0.3F;

  /* UnitDelay: '<S268>/Delay Input2'
   *
   * Block description for '<S268>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add4_j = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_e;

  /* SampleTimeMath: '<S268>/sample time'
   *
   * About '<S268>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S268>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(100.0 * elapseTime);

  /* MinMax: '<S307>/Min3' incorporates:
   *  DataTypeConversion: '<S183>/Cast To Double2'
   *  UnitDelay: '<S261>/Unit Delay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p = (real32_T)WhlSpdFR;

  /* DataTypeConversion: '<S183>/Cast To Double7' */
  rtb_Add10 = (real32_T)rtb_Yk1_l;

  /* Gain: '<S247>/Gain3' */
  rtb_Add10 *= 0.0174532924F;

  /* Trigonometry: '<S247>/Asin1' */
  rtb_Add10 = cosf(rtb_Add10);

  /* Product: '<S247>/Product2' incorporates:
   *  UnitDelay: '<S261>/Unit Delay'
   */
  rtb_Add10 *= VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p;

  /* Sum: '<S247>/Add3' */
  rtb_Divide = rtb_Switch2_mn + rtb_Add10;

  /* UnitDelay: '<S254>/Unit Delay1' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_n;

  /* Sum: '<S254>/Add5' */
  rtb_Add10 = rtb_Divide - rtb_Add10;

  /* Product: '<S254>/Divide1' incorporates:
   *  Constant: '<S254>/steptime1'
   */
  rtb_deltafalllimit_n = rtb_Add10 / 0.01F;

  /* RelationalOperator: '<S269>/LowerRelop1' incorporates:
   *  Constant: '<S262>/Constant1'
   */
  rtb_Compare = (rtb_deltafalllimit_n > 100.0F);

  /* Switch: '<S269>/Switch2' incorporates:
   *  Constant: '<S262>/Constant1'
   */
  if (rtb_Compare) {
    rtb_deltafalllimit_n = 100.0F;
  } else {
    /* RelationalOperator: '<S269>/UpperRelop' incorporates:
     *  Constant: '<S262>/Constant'
     */
    rtb_ignition = (rtb_deltafalllimit_n < -100.0F);

    /* Switch: '<S269>/Switch' incorporates:
     *  Constant: '<S262>/Constant'
     */
    if (rtb_ignition) {
      rtb_deltafalllimit_n = -100.0F;
    }

    /* End of Switch: '<S269>/Switch' */
  }

  /* End of Switch: '<S269>/Switch2' */

  /* Sum: '<S268>/Difference Inputs1'
   *
   * Block description for '<S268>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_n -= rtb_Add4_j;

  /* RelationalOperator: '<S270>/LowerRelop1' */
  rtb_Compare = (rtb_deltafalllimit_n > rtb_Switch2_b0);

  /* Switch: '<S270>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S268>/delta fall limit' */
    rtb_deltafalllimit_om = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S270>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_n < rtb_deltafalllimit_om);

    /* Switch: '<S270>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_n = rtb_deltafalllimit_om;
    }

    /* End of Switch: '<S270>/Switch' */
    rtb_Switch2_b0 = rtb_deltafalllimit_n;
  }

  /* End of Switch: '<S270>/Switch2' */

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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_e = rtb_Switch2_b0 + rtb_Add4_j;

  /* Gain: '<S262>/Gain1' incorporates:
   *  UnitDelay: '<S268>/Delay Input2'
   *
   * Block description for '<S268>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = 0.7F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_e;

  /* Sum: '<S262>/Add' */
  rtb_Add7 += rtb_Add10;

  /* Sum: '<S254>/Add1' */
  rtb_Add7 -= rtb_CastToDouble;

  /* Abs: '<S254>/Abs1' */
  rtb_Add7 = fabsf(rtb_Add7);

  /* RelationalOperator: '<S258>/Compare' incorporates:
   *  Constant: '<S258>/Constant'
   */
  rtb_Compare = (rtb_Add7 <= 0.5F);

  /* UnitDelay: '<S263>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_a;

  /* Gain: '<S263>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S271>/Delay Input2'
   *
   * Block description for '<S271>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hk3;

  /* SampleTimeMath: '<S271>/sample time'
   *
   * About '<S271>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S271>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S183>/Cast To Double3' */
  rtb_Add4_j = (real32_T)WhlSpdRL_mps;

  /* Product: '<S247>/Product' */
  rtb_Add6_p *= 0.58F;

  /* Sum: '<S247>/Add' */
  rtb_deltafalllimit_n = rtb_Add4_j - rtb_Add6_p;

  /* UnitDelay: '<S254>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_l;

  /* Sum: '<S254>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_n - rtb_Add4_j;

  /* Product: '<S254>/Divide2' incorporates:
   *  Constant: '<S254>/steptime2'
   */
  rtb_deltafalllimit_om = rtb_Add4_j / 0.01F;

  /* RelationalOperator: '<S272>/LowerRelop1' incorporates:
   *  Constant: '<S263>/Constant1'
   */
  rtb_Compare_am = (rtb_deltafalllimit_om > 100.0F);

  /* Switch: '<S272>/Switch2' incorporates:
   *  Constant: '<S263>/Constant1'
   */
  if (rtb_Compare_am) {
    rtb_deltafalllimit_om = 100.0F;
  } else {
    /* RelationalOperator: '<S272>/UpperRelop' incorporates:
     *  Constant: '<S263>/Constant'
     */
    rtb_ignition = (rtb_deltafalllimit_om < -100.0F);

    /* Switch: '<S272>/Switch' incorporates:
     *  Constant: '<S263>/Constant'
     */
    if (rtb_ignition) {
      rtb_deltafalllimit_om = -100.0F;
    }

    /* End of Switch: '<S272>/Switch' */
  }

  /* End of Switch: '<S272>/Switch2' */

  /* Sum: '<S271>/Difference Inputs1'
   *
   * Block description for '<S271>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_om -= rtb_Switch2_mn;

  /* RelationalOperator: '<S273>/LowerRelop1' */
  rtb_Compare_am = (rtb_deltafalllimit_om > rtb_Add7);

  /* Switch: '<S273>/Switch2' */
  if (!rtb_Compare_am) {
    /* Product: '<S271>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S273>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_om < rtb_Add7);

    /* Switch: '<S273>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_om = rtb_Add7;
    }

    /* End of Switch: '<S273>/Switch' */
    rtb_Add7 = rtb_deltafalllimit_om;
  }

  /* End of Switch: '<S273>/Switch2' */

  /* Sum: '<S271>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S271>/Delay Input2'
   *
   * Block description for '<S271>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S271>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hk3 = rtb_Add7 + rtb_Switch2_mn;

  /* Gain: '<S263>/Gain1' incorporates:
   *  UnitDelay: '<S271>/Delay Input2'
   *
   * Block description for '<S271>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hk3;

  /* Sum: '<S263>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S254>/Add2' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S254>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S259>/Compare' incorporates:
   *  Constant: '<S259>/Constant'
   */
  rtb_Compare_am = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S264>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_nc;

  /* Gain: '<S264>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S274>/Delay Input2'
   *
   * Block description for '<S274>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_c;

  /* SampleTimeMath: '<S274>/sample time'
   *
   * About '<S274>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S274>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S183>/Cast To Double4' */
  rtb_Add4_j = (real32_T)WhlSpdRR_mps;

  /* Sum: '<S247>/Add1' */
  rtb_deltafalllimit_om = rtb_Add6_p + rtb_Add4_j;

  /* UnitDelay: '<S254>/Unit Delay3' */
  rtb_Add6_p = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE;

  /* Sum: '<S254>/Add7' */
  rtb_Add6_p = rtb_deltafalllimit_om - rtb_Add6_p;

  /* Product: '<S254>/Divide3' incorporates:
   *  Constant: '<S254>/steptime3'
   */
  rtb_Add6_p /= 0.01F;

  /* RelationalOperator: '<S275>/LowerRelop1' incorporates:
   *  Constant: '<S264>/Constant1'
   */
  rtb_ignition = (rtb_Add6_p > 100.0F);

  /* Switch: '<S275>/Switch2' incorporates:
   *  Constant: '<S264>/Constant1'
   */
  if (rtb_ignition) {
    rtb_Add6_p = 100.0F;
  } else {
    /* RelationalOperator: '<S275>/UpperRelop' incorporates:
     *  Constant: '<S264>/Constant'
     */
    rtb_ignition = (rtb_Add6_p < -100.0F);

    /* Switch: '<S275>/Switch' incorporates:
     *  Constant: '<S264>/Constant'
     */
    if (rtb_ignition) {
      rtb_Add6_p = -100.0F;
    }

    /* End of Switch: '<S275>/Switch' */
  }

  /* End of Switch: '<S275>/Switch2' */

  /* Sum: '<S274>/Difference Inputs1'
   *
   * Block description for '<S274>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add6_p -= rtb_Switch2_mn;

  /* RelationalOperator: '<S276>/LowerRelop1' */
  rtb_ignition = (rtb_Add6_p > rtb_Add7);

  /* Switch: '<S276>/Switch2' */
  if (!rtb_ignition) {
    /* Product: '<S274>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S276>/UpperRelop' */
    rtb_ignition = (rtb_Add6_p < rtb_Add7);

    /* Switch: '<S276>/Switch' */
    if (rtb_ignition) {
      rtb_Add6_p = rtb_Add7;
    }

    /* End of Switch: '<S276>/Switch' */
    rtb_Add7 = rtb_Add6_p;
  }

  /* End of Switch: '<S276>/Switch2' */

  /* Sum: '<S274>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S274>/Delay Input2'
   *
   * Block description for '<S274>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S274>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_c = rtb_Add7 + rtb_Switch2_mn;

  /* Gain: '<S264>/Gain1' incorporates:
   *  UnitDelay: '<S274>/Delay Input2'
   *
   * Block description for '<S274>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_c;

  /* Sum: '<S264>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S254>/Add3' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S254>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S260>/Compare' incorporates:
   *  Constant: '<S260>/Constant'
   */
  rtb_ignition = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S281>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_l;

  /* Gain: '<S281>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S285>/Delay Input2'
   *
   * Block description for '<S285>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_i;

  /* SampleTimeMath: '<S285>/sample time'
   *
   * About '<S285>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S285>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S255>/Unit Delay4' */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_ik;

  /* UnitDelay: '<S255>/Unit Delay' */
  rtb_Add4_j = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_a0;

  /* Sum: '<S255>/Add4' */
  rtb_Add4_j = rtb_Add - rtb_Add4_j;

  /* Product: '<S255>/Divide' incorporates:
   *  Constant: '<S255>/steptime'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S255>/Add' */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_ik = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S255>/Add8' */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_ik - rtb_Add7;

  /* Product: '<S255>/Divide4' incorporates:
   *  Constant: '<S255>/steptime4'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S286>/LowerRelop1' incorporates:
   *  Constant: '<S281>/Constant1'
   */
  rtb_LowerRelop1_b = (rtb_Add7 > 100.0F);

  /* Switch: '<S286>/Switch2' incorporates:
   *  Constant: '<S281>/Constant1'
   */
  if (rtb_LowerRelop1_b) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S286>/UpperRelop' incorporates:
     *  Constant: '<S281>/Constant'
     */
    rtb_LowerRelop1_b = (rtb_Add7 < -100.0F);

    /* Switch: '<S286>/Switch' incorporates:
     *  Constant: '<S281>/Constant'
     */
    if (rtb_LowerRelop1_b) {
      rtb_Add7 = -100.0F;
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
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S287>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S287>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S285>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S287>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S287>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S287>/Switch' */
    rtb_Add6_p = rtb_Add7;
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_i = rtb_Add6_p + rtb_Switch2_mn;

  /* Gain: '<S281>/Gain1' incorporates:
   *  UnitDelay: '<S285>/Delay Input2'
   *
   * Block description for '<S285>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_i;

  /* Sum: '<S281>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S255>/Abs' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S277>/Compare' incorporates:
   *  Constant: '<S277>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S282>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_ap;

  /* Gain: '<S282>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S288>/Delay Input2'
   *
   * Block description for '<S288>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_el;

  /* SampleTimeMath: '<S288>/sample time'
   *
   * About '<S288>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S288>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S255>/Unit Delay5' */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE;

  /* UnitDelay: '<S255>/Unit Delay1' */
  rtb_Add4_j = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_am;

  /* Sum: '<S255>/Add5' */
  rtb_Add4_j = rtb_Divide - rtb_Add4_j;

  /* Product: '<S255>/Divide1' incorporates:
   *  Constant: '<S255>/steptime1'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S255>/Add1' incorporates:
   *  UnitDelay: '<S255>/Unit Delay5'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE = rtb_Add4_j - rtb_CastToDouble;

  /* Sum: '<S255>/Add10' incorporates:
   *  UnitDelay: '<S255>/Unit Delay5'
   */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE - rtb_Add7;

  /* Product: '<S255>/Divide5' incorporates:
   *  Constant: '<S255>/steptime5'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S289>/LowerRelop1' incorporates:
   *  Constant: '<S282>/Constant1'
   */
  rtb_AND2 = (rtb_Add7 > 100.0F);

  /* Switch: '<S289>/Switch2' incorporates:
   *  Constant: '<S282>/Constant1'
   */
  if (rtb_AND2) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S289>/UpperRelop' incorporates:
     *  Constant: '<S282>/Constant'
     */
    rtb_AND2 = (rtb_Add7 < -100.0F);

    /* Switch: '<S289>/Switch' incorporates:
     *  Constant: '<S282>/Constant'
     */
    if (rtb_AND2) {
      rtb_Add7 = -100.0F;
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
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S290>/LowerRelop1' */
  rtb_AND2 = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S290>/Switch2' */
  if (!rtb_AND2) {
    /* Product: '<S288>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S290>/UpperRelop' */
    rtb_AND2 = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S290>/Switch' */
    if (rtb_AND2) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S290>/Switch' */
    rtb_Add6_p = rtb_Add7;
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_el = rtb_Add6_p + rtb_Switch2_mn;

  /* Gain: '<S282>/Gain1' incorporates:
   *  UnitDelay: '<S288>/Delay Input2'
   *
   * Block description for '<S288>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_el;

  /* Sum: '<S282>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S255>/Abs1' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S278>/Compare' incorporates:
   *  Constant: '<S278>/Constant'
   */
  rtb_AND2 = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S283>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_o;

  /* Gain: '<S283>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S291>/Delay Input2'
   *
   * Block description for '<S291>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_p;

  /* SampleTimeMath: '<S291>/sample time'
   *
   * About '<S291>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S291>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S255>/Unit Delay6' */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay6_DSTATE;

  /* UnitDelay: '<S255>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_c;

  /* Sum: '<S255>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_n - rtb_Add4_j;

  /* Product: '<S255>/Divide2' incorporates:
   *  Constant: '<S255>/steptime2'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S255>/Add2' incorporates:
   *  UnitDelay: '<S255>/Unit Delay6'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay6_DSTATE = rtb_Add4_j - rtb_CastToDouble;

  /* Sum: '<S255>/Add12' incorporates:
   *  UnitDelay: '<S255>/Unit Delay6'
   */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay6_DSTATE - rtb_Add7;

  /* Product: '<S255>/Divide6' incorporates:
   *  Constant: '<S255>/steptime6'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S292>/LowerRelop1' incorporates:
   *  Constant: '<S283>/Constant1'
   */
  rtb_Compare_b = (rtb_Add7 > 100.0F);

  /* Switch: '<S292>/Switch2' incorporates:
   *  Constant: '<S283>/Constant1'
   */
  if (rtb_Compare_b) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S292>/UpperRelop' incorporates:
     *  Constant: '<S283>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S292>/Switch' incorporates:
     *  Constant: '<S283>/Constant'
     */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = -100.0F;
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
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S293>/LowerRelop1' */
  rtb_Compare_b = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S293>/Switch2' */
  if (!rtb_Compare_b) {
    /* Product: '<S291>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S293>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S293>/Switch' */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S293>/Switch' */
    rtb_Add6_p = rtb_Add7;
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_p = rtb_Add6_p + rtb_Switch2_mn;

  /* Gain: '<S283>/Gain1' incorporates:
   *  UnitDelay: '<S291>/Delay Input2'
   *
   * Block description for '<S291>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_p;

  /* Sum: '<S283>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S255>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S279>/Compare' incorporates:
   *  Constant: '<S279>/Constant'
   */
  rtb_Compare_b = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S284>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_ah;

  /* Gain: '<S284>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S294>/Delay Input2'
   *
   * Block description for '<S294>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_mt;

  /* SampleTimeMath: '<S294>/sample time'
   *
   * About '<S294>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S294>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S255>/Unit Delay7' */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay7_DSTATE;

  /* UnitDelay: '<S255>/Unit Delay3' */
  rtb_Add4_j = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_d;

  /* Sum: '<S255>/Add7' */
  rtb_Add4_j = rtb_deltafalllimit_om - rtb_Add4_j;

  /* Product: '<S255>/Divide3' incorporates:
   *  Constant: '<S255>/steptime3'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S255>/Add3' incorporates:
   *  UnitDelay: '<S255>/Unit Delay7'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay7_DSTATE = rtb_Add4_j - rtb_CastToDouble;

  /* Sum: '<S255>/Add14' incorporates:
   *  UnitDelay: '<S255>/Unit Delay7'
   */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay7_DSTATE - rtb_Add7;

  /* Product: '<S255>/Divide7' incorporates:
   *  Constant: '<S255>/steptime7'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S295>/LowerRelop1' incorporates:
   *  Constant: '<S284>/Constant1'
   */
  rtb_UpperRelop_ir = (rtb_Add7 > 100.0F);

  /* Switch: '<S295>/Switch2' incorporates:
   *  Constant: '<S284>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S295>/UpperRelop' incorporates:
     *  Constant: '<S284>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S295>/Switch' incorporates:
     *  Constant: '<S284>/Constant'
     */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S295>/Switch' */
  }

  /* End of Switch: '<S295>/Switch2' */

  /* Sum: '<S294>/Difference Inputs1'
   *
   * Block description for '<S294>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S296>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S296>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S294>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S296>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S296>/Switch' */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S296>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S296>/Switch2' */

  /* Sum: '<S294>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S294>/Delay Input2'
   *
   * Block description for '<S294>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S294>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_mt = rtb_Add6_p + rtb_Switch2_mn;

  /* Gain: '<S284>/Gain1' incorporates:
   *  UnitDelay: '<S294>/Delay Input2'
   *
   * Block description for '<S294>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_mt;

  /* Sum: '<S284>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S255>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S280>/Compare' incorporates:
   *  Constant: '<S280>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.8F);

  /* Logic: '<S245>/Logical Operator' */
  rtb_LogicalOperator_idx_0 = (rtb_LogicalOperator2 || rtb_LowerRelop1_b);
  rtb_Compare = (rtb_Compare || rtb_AND2);
  rtb_Compare_am = (rtb_Compare_am || rtb_Compare_b);
  rtb_LogicalOperator2 = (rtb_ignition || rtb_UpperRelop_ir);

  /* UnitDelay: '<S183>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_lh;

  /* Sum: '<S256>/Add' */
  rtb_Switch2_mn = rtb_Add - rtb_Add10;

  /* Abs: '<S256>/Abs' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S297>/Compare' incorporates:
   *  Constant: '<S297>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S256>/AND3' */
  rtb_Compare_b = (rtb_UpperRelop_ir && (VehCtrlMdel240918_2018b_B.Exit_h != 0.0));

  /* Sum: '<S256>/Add1' */
  rtb_Switch2_mn = rtb_Divide - rtb_Add10;

  /* Abs: '<S256>/Abs1' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S298>/Compare' incorporates:
   *  Constant: '<S298>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S256>/AND2' */
  rtb_AND2 = (rtb_UpperRelop_ir && (VehCtrlMdel240918_2018b_B.Exit_o != 0.0));

  /* Sum: '<S256>/Add2' */
  rtb_Switch2_mn = rtb_deltafalllimit_n - rtb_Add10;

  /* Abs: '<S256>/Abs2' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S299>/Compare' incorporates:
   *  Constant: '<S299>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S256>/AND' */
  rtb_LowerRelop1_b = (rtb_UpperRelop_ir && (VehCtrlMdel240918_2018b_B.Exit_le
    != 0.0));

  /* Sum: '<S256>/Add3' */
  rtb_Add10 = rtb_deltafalllimit_om - rtb_Add10;

  /* Abs: '<S256>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S300>/Compare' incorporates:
   *  Constant: '<S300>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 2.0F);

  /* Logic: '<S256>/AND1' */
  rtb_ignition = (rtb_UpperRelop_ir && (VehCtrlMdel240918_2018b_B.Exit_i != 0.0));

  /* Logic: '<S245>/Logical Operator1' */
  rtb_UpperRelop_ir = (rtb_Compare_b && rtb_LogicalOperator_idx_0);
  rtb_Compare = (rtb_AND2 && rtb_Compare);
  rtb_Compare_am = (rtb_LowerRelop1_b && rtb_Compare_am);
  rtb_ignition = (rtb_ignition && rtb_LogicalOperator2);

  /* Chart: '<S245>/Timer' incorporates:
   *  Constant: '<S245>/Constant1'
   */
  VehCtrlMdel240918_20_Timer1(rtb_UpperRelop_ir, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_c, &VehCtrlMdel240918_2018b_DW.sf_Timer_o);

  /* Chart: '<S245>/Timer1' incorporates:
   *  Constant: '<S245>/Constant2'
   */
  VehCtrlMdel240918_20_Timer1(rtb_Compare, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_lh4, &VehCtrlMdel240918_2018b_DW.sf_Timer1_m);

  /* Chart: '<S245>/Timer2' incorporates:
   *  Constant: '<S245>/Constant3'
   */
  VehCtrlMdel240918_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_lh, &VehCtrlMdel240918_2018b_DW.sf_Timer2_g);

  /* Chart: '<S245>/Timer3' incorporates:
   *  Constant: '<S245>/Constant4'
   */
  VehCtrlMdel240918_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_a, &VehCtrlMdel240918_2018b_DW.sf_Timer3_i);

  /* Logic: '<S244>/Logical Operator' */
  rtb_UpperRelop_ir = ((VehCtrlMdel240918_2018b_B.Exit_c != 0.0) ||
                       (VehCtrlMdel240918_2018b_B.Exit_lh4 != 0.0) ||
                       (VehCtrlMdel240918_2018b_B.Exit_lh != 0.0) ||
                       (VehCtrlMdel240918_2018b_B.Exit_a != 0.0));

  /* Logic: '<S244>/Logical Operator1' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* UnitDelay: '<S244>/Unit Delay4' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_m;

  /* Sum: '<S244>/Add1' */
  rtb_Add10 = rtb_Acc_POS - rtb_Add10;

  /* RelationalOperator: '<S248>/Compare' incorporates:
   *  Constant: '<S248>/Constant'
   */
  rtb_Compare_b = (rtb_Add10 > 0.1F);

  /* Logic: '<S244>/Logical Operator2' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir || rtb_Compare_b);

  /* Logic: '<S244>/AND' */
  rtb_ignition = ((VehCtrlMdel240918_2018b_B.CANUnpack_o1 != 0.0) &&
                  rtb_UpperRelop_ir);

  /* UnitDelay: '<S244>/Unit Delay3' */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_f;

  /* Logic: '<S244>/Logical Operator3' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S244>/Switch3' incorporates:
   *  UnitDelay: '<S244>/Unit Delay1'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S244>/Switch4' */
    if (!rtb_ignition) {
      /* Saturate: '<S21>/Saturation' incorporates:
       *  Constant: '<S244>/InitZORE'
       */
      VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k = 0.0F;
    }

    /* End of Switch: '<S244>/Switch4' */
    VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_o =
      VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Switch: '<S244>/Switch3' */

  /* UnitDelay: '<S246>/Unit Delay3' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_p;

  /* Sum: '<S246>/Add5' incorporates:
   *  UnitDelay: '<S246>/Unit Delay1'
   */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_d - rtb_Add10;

  /* Product: '<S246>/Divide3' incorporates:
   *  Constant: '<S246>/steptime3'
   */
  rtb_Add10 /= 0.01F;

  /* UnitDelay: '<S246>/Unit Delay2' */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_f;

  /* Sum: '<S246>/Add9' */
  rtb_Switch2_mn -= rtb_Add10;

  /* UnitDelay: '<S246>/Unit Delay4' */
  rtb_Add6_p = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_mn;

  /* Sum: '<S246>/Add6' incorporates:
   *  Constant: '<S246>/steptime4'
   */
  rtb_Add6_p += 0.1F;

  /* Sum: '<S246>/Add8' incorporates:
   *  Constant: '<S246>/steptime6'
   */
  rtb_Add7 = rtb_Add6_p + 2.0F;

  /* Product: '<S246>/Divide5' */
  rtb_Add7 = 1.0F / rtb_Add7 * rtb_Add6_p;

  /* Logic: '<S246>/Logical Operator' */
  rtb_LogicalOperator2 = ((VehCtrlMdel240918_2018b_B.Exit_c != 0.0) ||
    (VehCtrlMdel240918_2018b_B.Exit_lh4 != 0.0) ||
    (VehCtrlMdel240918_2018b_B.Exit_lh != 0.0) ||
    (VehCtrlMdel240918_2018b_B.Exit_a != 0.0));

  /* Switch: '<S246>/Switch13' incorporates:
   *  Constant: '<S246>/Constant10'
   */
  if (rtb_LogicalOperator2) {
    rtb_Add4_j = rtb_Add7;
  } else {
    rtb_Add4_j = 1.0F;
  }

  /* End of Switch: '<S246>/Switch13' */

  /* Product: '<S246>/Divide6' */
  rtb_Switch2_mn *= rtb_Add4_j;

  /* Sum: '<S246>/Add10' */
  rtb_Ax = rtb_Switch2_mn + rtb_Add10;

  /* Switch: '<S244>/Switch1' */
  if (rtb_ignition) {
    /* Saturate: '<S244>/Saturation1' */
    if (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d > 200.0F) {
      VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d = 200.0F;
    } else {
      if (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d < -10.0F) {
        VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d = -10.0F;
      }
    }

    /* Product: '<S244>/Product' incorporates:
     *  Constant: '<S244>/steptime1'
     */
    rtb_Switch2_mn = rtb_Ax * 0.01F;

    /* Saturate: '<S244>/Saturation1' incorporates:
     *  Sum: '<S244>/Add'
     */
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d += rtb_Switch2_mn;
  } else {
    /* Saturate: '<S244>/Saturation1' incorporates:
     *  Constant: '<S244>/Constant'
     *  UnitDelay: '<S244>/Unit Delay'
     */
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d = 0.0F;
  }

  /* End of Switch: '<S244>/Switch1' */

  /* Saturate: '<S244>/Saturation' */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d > 200.0F) {
    rtb_Add10 = 200.0F;
  } else if (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d < -10.0F) {
    rtb_Add10 = -10.0F;
  } else {
    rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d;
  }

  /* End of Saturate: '<S244>/Saturation' */

  /* Sum: '<S244>/Add3' incorporates:
   *  UnitDelay: '<S244>/Unit Delay1'
   */
  rtb_VxIMU_est = rtb_Add10 + VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_o;

  /* MinMax: '<S245>/Min1' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Add, rtb_Divide);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_n);
  rtb_Add10 = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_om);

  /* Sum: '<S244>/Add2' */
  rtb_Add10 -= rtb_VxIMU_est;

  /* RelationalOperator: '<S249>/Compare' incorporates:
   *  Constant: '<S249>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.0F);

  /* Switch: '<S244>/Switch6' incorporates:
   *  Constant: '<S244>/Reset'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S244>/Add10' incorporates:
     *  Constant: '<S244>/Steptime'
     */
    rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_i + 0.01F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S244>/Switch6' */

  /* MinMax: '<S244>/Min' incorporates:
   *  Constant: '<S244>/ResetDelay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_i = fminf(rtb_Add10, 0.1F);

  /* RelationalOperator: '<S244>/Relational Operator9' incorporates:
   *  Constant: '<S244>/ResetDelay'
   *  UnitDelay: '<S244>/Unit Delay2'
   */
  rtb_Compare = (VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_i >= 0.1F);

  /* RelationalOperator: '<S302>/Compare' incorporates:
   *  Constant: '<S302>/Constant'
   */
  rtb_Compare_am = (rtb_Ax < -0.5F);

  /* Chart: '<S246>/Timer2' incorporates:
   *  Constant: '<S246>/Constant15'
   */
  VehCtrlMdel240918_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit, &VehCtrlMdel240918_2018b_DW.sf_Timer2_j);

  /* UnitDelay: '<S306>/Delay Input2'
   *
   * Block description for '<S306>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_g;

  /* SampleTimeMath: '<S306>/sample time'
   *
   * About '<S306>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S306>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(10.0 * elapseTime);

  /* Sum: '<S307>/Add3' */
  rtb_Add4_j = ((rtb_deltafalllimit_om + rtb_deltafalllimit_n) + rtb_Divide) +
    rtb_Add;

  /* MinMax: '<S307>/Min4' */
  rtb_Switch2_b0 = fminf(rtb_Add, rtb_Divide);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_n);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_om);

  /* MinMax: '<S307>/Min3' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Add, rtb_Divide);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_n);
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p = fmaxf(rtb_MaxWhlSpd_mps_n,
    rtb_deltafalllimit_om);

  /* Sum: '<S307>/Add4' incorporates:
   *  UnitDelay: '<S261>/Unit Delay'
   */
  rtb_Add4_j = (rtb_Add4_j - rtb_Switch2_b0) -
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p;

  /* Gain: '<S307>/Gain1' */
  rtb_Add4_j *= 0.5F;

  /* Sum: '<S306>/Difference Inputs1'
   *
   * Block description for '<S306>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add4_j -= rtb_Add10;

  /* RelationalOperator: '<S315>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add4_j > rtb_Switch2_mn);

  /* Switch: '<S315>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S306>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-10.0 * elapseTime);

    /* RelationalOperator: '<S315>/UpperRelop' */
    rtb_Compare_am = (rtb_Add4_j < rtb_Switch2_mn);

    /* Switch: '<S315>/Switch' */
    if (rtb_Compare_am) {
      rtb_Add4_j = rtb_Switch2_mn;
    }

    /* End of Switch: '<S315>/Switch' */
    rtb_Switch2_mn = rtb_Add4_j;
  }

  /* End of Switch: '<S315>/Switch2' */

  /* Sum: '<S306>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S306>/Delay Input2'
   *
   * Block description for '<S306>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S306>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_g = rtb_Switch2_mn + rtb_Add10;

  /* RelationalOperator: '<S301>/Compare' incorporates:
   *  Constant: '<S301>/Constant'
   */
  rtb_Compare_am = (rtb_Ax > 0.5F);

  /* Chart: '<S246>/Timer1' incorporates:
   *  Constant: '<S246>/Constant14'
   */
  VehCtrlMdel240918_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_l, &VehCtrlMdel240918_2018b_DW.sf_Timer1_p);

  /* Logic: '<S246>/Logical Operator2' */
  rtb_UpperRelop_ir = !(VehCtrlMdel240918_2018b_B.Exit_l != 0.0);

  /* Switch: '<S246>/Switch6' incorporates:
   *  Switch: '<S246>/Switch4'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S246>/Switch5' incorporates:
     *  UnitDelay: '<S306>/Delay Input2'
     *
     * Block description for '<S306>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (VehCtrlMdel240918_2018b_B.Exit != 0.0) {
      /* Switch: '<S246>/Switch11' incorporates:
       *  Constant: '<S246>/Constant7'
       */
      if (VehCtrlMdel240918_2018b_B.Exit_a != 0.0) {
        rtb_Switch2_mn = rtb_deltafalllimit_om;
      } else {
        rtb_Switch2_mn = 0.0F;
      }

      /* End of Switch: '<S246>/Switch11' */

      /* Switch: '<S246>/Switch10' incorporates:
       *  Constant: '<S246>/Constant6'
       */
      if (VehCtrlMdel240918_2018b_B.Exit_lh != 0.0) {
        rtb_Add10 = rtb_deltafalllimit_n;
      } else {
        rtb_Add10 = 0.0F;
      }

      /* End of Switch: '<S246>/Switch10' */

      /* Switch: '<S246>/Switch9' incorporates:
       *  Constant: '<S246>/Constant5'
       */
      if (VehCtrlMdel240918_2018b_B.Exit_lh4 != 0.0) {
        rtb_Add4_j = rtb_Divide;
      } else {
        rtb_Add4_j = 0.0F;
      }

      /* End of Switch: '<S246>/Switch9' */

      /* Switch: '<S246>/Switch8' incorporates:
       *  Constant: '<S246>/Constant4'
       */
      if (VehCtrlMdel240918_2018b_B.Exit_c != 0.0) {
        rtb_Switch2_b0 = rtb_Add;
      } else {
        rtb_Switch2_b0 = 0.0F;
      }

      /* End of Switch: '<S246>/Switch8' */

      /* MinMax: '<S246>/Min1' */
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Switch2_b0, rtb_Add4_j);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Add10);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Switch2_mn);
    } else {
      rtb_MaxWhlSpd_mps_n = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_g;
    }

    /* End of Switch: '<S246>/Switch5' */
  } else {
    if (VehCtrlMdel240918_2018b_B.Exit_a != 0.0) {
      /* Switch: '<S246>/Switch4' */
      rtb_Switch2_mn = rtb_deltafalllimit_om;
    } else {
      /* Switch: '<S246>/Switch4' incorporates:
       *  Constant: '<S246>/Constant3'
       */
      rtb_Switch2_mn = 9999.0F;
    }

    /* Switch: '<S246>/Switch3' incorporates:
     *  Constant: '<S246>/Constant2'
     */
    if (VehCtrlMdel240918_2018b_B.Exit_lh != 0.0) {
      rtb_Add10 = rtb_deltafalllimit_n;
    } else {
      rtb_Add10 = 9999.0F;
    }

    /* End of Switch: '<S246>/Switch3' */

    /* Switch: '<S246>/Switch2' incorporates:
     *  Constant: '<S246>/Constant1'
     */
    if (VehCtrlMdel240918_2018b_B.Exit_lh4 != 0.0) {
      rtb_Add4_j = rtb_Divide;
    } else {
      rtb_Add4_j = 9999.0F;
    }

    /* End of Switch: '<S246>/Switch2' */

    /* Switch: '<S246>/Switch1' incorporates:
     *  Constant: '<S246>/Constant'
     */
    if (VehCtrlMdel240918_2018b_B.Exit_c != 0.0) {
      rtb_Switch2_b0 = rtb_Add;
    } else {
      rtb_Switch2_b0 = 9999.0F;
    }

    /* End of Switch: '<S246>/Switch1' */

    /* MinMax: '<S246>/Min2' */
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add4_j);
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add10);
    rtb_MaxWhlSpd_mps_n = fminf(rtb_Switch2_b0, rtb_Switch2_mn);
  }

  /* End of Switch: '<S246>/Switch6' */

  /* Logic: '<S246>/NOT3' */
  rtb_UpperRelop_ir = !rtb_LogicalOperator2;

  /* Logic: '<S246>/Logical Operator3' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir && rtb_Compare);

  /* Logic: '<S246>/NOT4' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S246>/Switch7' incorporates:
   *  UnitDelay: '<S306>/Delay Input2'
   *
   * Block description for '<S306>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (rtb_UpperRelop_ir) {
    /* Logic: '<S246>/Logical Operator1' */
    rtb_Compare = (rtb_Compare || rtb_LogicalOperator2);

    /* Switch: '<S246>/Switch' */
    if (rtb_Compare) {
      rtb_VxIMU_est = rtb_MaxWhlSpd_mps_n;
    }

    /* End of Switch: '<S246>/Switch' */
  } else {
    rtb_VxIMU_est = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_g;
  }

  /* End of Switch: '<S246>/Switch7' */

  /* UnitDelay: '<S304>/Delay Input2'
   *
   * Block description for '<S304>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_af;

  /* Sum: '<S304>/Difference Inputs1'
   *
   * Block description for '<S304>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_VxIMU_est -= rtb_Add10;

  /* Switch: '<S246>/Switch12' incorporates:
   *  Constant: '<S246>/Constant8'
   *  Constant: '<S246>/Constant9'
   */
  if (rtb_LogicalOperator2) {
    rtb_Switch2_mn = 0.1F;
  } else {
    rtb_Switch2_mn = 0.05F;
  }

  /* End of Switch: '<S246>/Switch12' */

  /* Sum: '<S246>/Add4' */
  rtb_Add4_j = rtb_Ax + rtb_Switch2_mn;

  /* SampleTimeMath: '<S304>/sample time'
   *
   * About '<S304>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S304>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S313>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_VxIMU_est > rtb_Switch2_b0);

  /* Sum: '<S246>/Add3' */
  rtb_Ax -= rtb_Switch2_mn;

  /* Switch: '<S313>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S304>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S313>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_VxIMU_est < rtb_Switch2_mn);

    /* Switch: '<S313>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_VxIMU_est = rtb_Switch2_mn;
    }

    /* End of Switch: '<S313>/Switch' */
    rtb_Switch2_b0 = rtb_VxIMU_est;
  }

  /* End of Switch: '<S313>/Switch2' */

  /* Sum: '<S304>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S304>/Delay Input2'
   *
   * Block description for '<S304>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S304>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_af = rtb_Switch2_b0 + rtb_Add10;

  /* RelationalOperator: '<S311>/LowerRelop1' incorporates:
   *  Constant: '<S303>/Constant1'
   *  UnitDelay: '<S304>/Delay Input2'
   *
   * Block description for '<S304>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_af > 100.0F);

  /* Switch: '<S311>/Switch2' incorporates:
   *  Constant: '<S303>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_mn = 100.0F;
  } else {
    /* RelationalOperator: '<S311>/UpperRelop' incorporates:
     *  Constant: '<S303>/Constant'
     *  UnitDelay: '<S304>/Delay Input2'
     *
     * Block description for '<S304>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_LogicalOperator2 = (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_af <
      0.0F);

    /* Switch: '<S311>/Switch' incorporates:
     *  Constant: '<S303>/Constant'
     *  UnitDelay: '<S304>/Delay Input2'
     *
     * Block description for '<S304>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_LogicalOperator2) {
      rtb_Switch2_mn = 0.0F;
    } else {
      rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_af;
    }

    /* End of Switch: '<S311>/Switch' */
  }

  /* End of Switch: '<S311>/Switch2' */

  /* UnitDelay: '<S310>/Delay Input2'
   *
   * Block description for '<S310>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_f;

  /* Sum: '<S310>/Difference Inputs1'
   *
   * Block description for '<S310>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_Switch2_mn - rtb_Add10;

  /* SampleTimeMath: '<S310>/sample time'
   *
   * About '<S310>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S310>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(15.0 * elapseTime);

  /* RelationalOperator: '<S312>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S312>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S310>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-15.0 * elapseTime);

    /* RelationalOperator: '<S312>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S312>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S312>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_f = rtb_Switch2_mn + rtb_Add10;

  /* UnitDelay: '<S303>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_ncs;

  /* Gain: '<S303>/Gain' */
  rtb_Add10 *= 0.0F;

  /* Saturate: '<S21>/Saturation' incorporates:
   *  Sum: '<S303>/Add'
   *  UnitDelay: '<S183>/Unit Delay1'
   *  UnitDelay: '<S310>/Delay Input2'
   *
   * Block description for '<S310>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k = rtb_Add10 +
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_f;

  /* SampleTimeMath: '<S305>/sample time'
   *
   * About '<S305>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* UnitDelay: '<S305>/Delay Input2'
   *
   * Block description for '<S305>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hu;

  /* Sum: '<S305>/Difference Inputs1'
   *
   * Block description for '<S305>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_MaxWhlSpd_mps_n - rtb_Add10;

  /* Product: '<S305>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S314>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S314>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S305>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S314>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S314>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S314>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S314>/Switch2' */

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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hu = rtb_Switch2_mn + rtb_Add10;

  /* Sum: '<S246>/Add7' incorporates:
   *  Constant: '<S246>/steptime5'
   */
  rtb_Add7 = 1.0F - rtb_Add7;

  /* Product: '<S246>/Divide4' incorporates:
   *  UnitDelay: '<S246>/Unit Delay4'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_mn = rtb_Add7 * rtb_Add6_p;

  /* Update for MinMax: '<S307>/Min3' incorporates:
   *  UnitDelay: '<S261>/Unit Delay'
   *  UnitDelay: '<S265>/Delay Input2'
   *
   * Block description for '<S265>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n2;

  /* Update for UnitDelay: '<S254>/Unit Delay' */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_j = rtb_Add;

  /* Update for UnitDelay: '<S262>/Unit Delay' incorporates:
   *  UnitDelay: '<S268>/Delay Input2'
   *
   * Block description for '<S268>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_pj =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_e;

  /* Update for UnitDelay: '<S254>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_n = rtb_Divide;

  /* Update for UnitDelay: '<S263>/Unit Delay' incorporates:
   *  UnitDelay: '<S271>/Delay Input2'
   *
   * Block description for '<S271>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_a =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hk3;

  /* Update for UnitDelay: '<S254>/Unit Delay2' */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_l = rtb_deltafalllimit_n;

  /* Update for UnitDelay: '<S264>/Unit Delay' incorporates:
   *  UnitDelay: '<S274>/Delay Input2'
   *
   * Block description for '<S274>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_nc =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_c;

  /* Update for UnitDelay: '<S254>/Unit Delay3' */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE = rtb_deltafalllimit_om;

  /* Update for UnitDelay: '<S281>/Unit Delay' incorporates:
   *  UnitDelay: '<S285>/Delay Input2'
   *
   * Block description for '<S285>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_l =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_i;

  /* Update for UnitDelay: '<S255>/Unit Delay' */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_a0 = rtb_Add;

  /* Update for UnitDelay: '<S282>/Unit Delay' incorporates:
   *  UnitDelay: '<S288>/Delay Input2'
   *
   * Block description for '<S288>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_ap =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_el;

  /* Update for UnitDelay: '<S255>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_am = rtb_Divide;

  /* Update for UnitDelay: '<S283>/Unit Delay' incorporates:
   *  UnitDelay: '<S291>/Delay Input2'
   *
   * Block description for '<S291>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_o =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_p;

  /* Update for UnitDelay: '<S255>/Unit Delay2' */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_c = rtb_deltafalllimit_n;

  /* Update for UnitDelay: '<S284>/Unit Delay' incorporates:
   *  UnitDelay: '<S294>/Delay Input2'
   *
   * Block description for '<S294>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_ah =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_mt;

  /* Update for UnitDelay: '<S255>/Unit Delay3' */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_d = rtb_deltafalllimit_om;

  /* Update for UnitDelay: '<S183>/Unit Delay' incorporates:
   *  UnitDelay: '<S183>/Unit Delay1'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_lh =
    VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k;

  /* Update for UnitDelay: '<S244>/Unit Delay4' */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_m = rtb_Acc_POS;

  /* Update for UnitDelay: '<S244>/Unit Delay3' */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_f = rtb_ignition;

  /* Update for UnitDelay: '<S246>/Unit Delay3' incorporates:
   *  UnitDelay: '<S246>/Unit Delay1'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_p =
    VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_d;

  /* Update for UnitDelay: '<S246>/Unit Delay1' incorporates:
   *  UnitDelay: '<S305>/Delay Input2'
   *
   * Block description for '<S305>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_d =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hu;

  /* Update for UnitDelay: '<S246>/Unit Delay2' */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_f = rtb_CastToDouble;

  /* Update for UnitDelay: '<S303>/Unit Delay' incorporates:
   *  UnitDelay: '<S310>/Delay Input2'
   *
   * Block description for '<S310>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_ncs =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_f;

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms1' */

  /* S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* DataTypeConversion: '<S91>/Cast To Boolean' */
  KeyPressed = (ignition != 0.0);

  /* RelationalOperator: '<S93>/Compare' incorporates:
   *  Constant: '<S93>/Constant'
   */
  Brk = (Brk_F >= 450);

  /* RelationalOperator: '<S94>/Compare' incorporates:
   *  Constant: '<S94>/Constant'
   */
  ACC_Release = (rtb_Acc_POS <= 50.0F);

  /* Chart: '<S91>/Chart2' */
  FunctionCallSubsystem_ELAPS_T = VehCtrlMdel240918_2018b_M->Timing.clockTick3 -
    VehCtrlMdel240918_2018b_DW.previousTicks;
  VehCtrlMdel240918_2018b_DW.previousTicks =
    VehCtrlMdel240918_2018b_M->Timing.clockTick3;
  if (VehCtrlMdel240918_2018b_DW.temporalCounter_i1 +
      FunctionCallSubsystem_ELAPS_T <= 255U) {
    VehCtrlMdel240918_2018b_DW.temporalCounter_i1 = (uint8_T)
      (VehCtrlMdel240918_2018b_DW.temporalCounter_i1 +
       FunctionCallSubsystem_ELAPS_T);
  } else {
    VehCtrlMdel240918_2018b_DW.temporalCounter_i1 = MAX_uint8_T;
  }

  if (VehCtrlMdel240918_2018b_DW.temporalCounter_i2 +
      FunctionCallSubsystem_ELAPS_T <= 1023U) {
    VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = (uint16_T)
      (VehCtrlMdel240918_2018b_DW.temporalCounter_i2 +
       FunctionCallSubsystem_ELAPS_T);
  } else {
    VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = 1023U;
  }

  VehCtrlMdel240918_2018b_DW.sfEvent = -1;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_c1_VehCtrlMdel240918_ ==
      0U) {
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_c1_VehCtrlMdel240918_ = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_VehStat = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 2U;
    VehCtrlMdel240918_2018b_DW.temporalCounter_i1 = 0U;
    VehCtrlMdel240918_2018b_B.errorReset = 1.0;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_BeeperStat = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_Output = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKDCready = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 10U;
    VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = 0U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKCANenable = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKDCon = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MCDCEnable = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_InverterOn = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_TorqueCUT = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
  } else {
    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_VehStat != 0U) {
      VehCtrlMdel240918_2018b_VehStat(&HVCUTOFF, &MCFL_bQuitInverterOn,
        &MCFL_bSystemReady, &MCFR_bQuitInverterOn, &MCFR_bSystemReady);
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_BeeperStat != 0U) {
      switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat) {
       case VehCtrlMdel240918_2018b_IN_OFF:
        if (VehCtrlMdel240918_2018b_DW.sfEvent ==
            VehCtrlMdel240918_event_EbeepON) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat = 2U;
        }
        break;

       case VehCtrlMdel240918_2018b_IN_ON:
        if (VehCtrlMdel240918_2018b_DW.sfEvent ==
            VehCtrlMdel24091_event_EbeepOFF) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_Output != 0U) {
      VehCtrlMdel240918_2018b_B.VehReady =
        (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat ==
         VehCtrlMdel240918_2018_IN_Ready);
      beeper_state = (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat ==
                      VehCtrlMdel240918_2018b_IN_ON);
      VehCtrlMdel240918_2018b_B.MCFL_DCOn_setpoints =
        (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon ==
         VehCtrlMdel240918_2018b_IN_ON);
      VehCtrlMdel240918_2018b_B.MCFL_DCEnable =
        (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable ==
         VehCtrlMdel240918_2018b_IN_ON);
      VehCtrlMdel240918_2018b_B.MCFL_InverterOn =
        (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn ==
         VehCtrlMdel240918_2018b_IN_ON);
      VehCtrlMdel240918_2018b_B.MCFL_TorqueOn =
        (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT ==
         VehCtrlMdel240918_2018b_IN_ON);
      VehCtrlMdel240918_2018b_B.MCFR_TorqueOn =
        (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT ==
         VehCtrlMdel240918_2018b_IN_ON);
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKDCready != 0U) {
      VehCtrlMdel240918_20_AMKDCready(&MCFL_bDCOn, &MCFL_bQuitInverterOn,
        &MCFL_bSystemReady, &MCFR_bDCOn, &MCFR_bQuitInverterOn,
        &MCFR_bSystemReady);
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKCANenable != 0U) {
      switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable) {
       case VehCtrlMdel240918_2018b_IN_OFF:
        if (VehCtrlMdel240918_2018b_DW.sfEvent ==
            VehCtrlMdel24091_event_AMKCANON) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable = 2U;
        }
        break;

       case VehCtrlMdel240918_2018b_IN_ON:
        if (VehCtrlMdel240918_2018b_DW.sfEvent ==
            VehCtrlMdel2409_event_AMKCANOFF) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKDCon != 0U) {
      switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon) {
       case VehCtrlMdel240918_2018b_IN_OFF:
        if (VehCtrlMdel240918_2018b_DW.sfEvent ==
            VehCtrlMdel240918_event_AMKDCON) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 2U;
        }
        break;

       case VehCtrlMdel240918_2018b_IN_ON:
        if (VehCtrlMdel240918_2018b_DW.sfEvent ==
            VehCtrlMdel24091_event_AMKDCOFF) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MCDCEnable != 0U) {
      switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable) {
       case VehCtrlMdel240918_2018b_IN_OFF:
        if (VehCtrlMdel240918_2018b_DW.sfEvent ==
            VehCtrlMdel2_event_MCDCEnableON) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 2U;
        }
        break;

       case VehCtrlMdel240918_2018b_IN_ON:
        if (VehCtrlMdel240918_2018b_DW.sfEvent ==
            VehCtrlMdel_event_MCDCEnableOFF) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_InverterOn != 0U) {
      switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn) {
       case VehCtrlMdel240918_2018b_IN_OFF:
        if (VehCtrlMdel240918_2018b_DW.sfEvent ==
            VehCtrlMdel240_event_InverterON) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 2U;
        }
        break;

       case VehCtrlMdel240918_2018b_IN_ON:
        if (VehCtrlMdel240918_2018b_DW.sfEvent ==
            VehCtrlMdel24_event_InverterOFF) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_TorqueCUT != 0U) {
      switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT) {
       case VehCtrlMdel240918_2018b_IN_OFF:
        if (VehCtrlMdel240918_2018b_DW.sfEvent ==
            VehCtrlMdel24091_event_TorqueON) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 2U;
        }
        break;

       case VehCtrlMdel240918_2018b_IN_ON:
        if (VehCtrlMdel240918_2018b_DW.sfEvent ==
            VehCtrlMdel2409_event_TorqueOFF) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
        }
        break;
      }
    }
  }

  /* End of Chart: '<S91>/Chart2' */
  /* End of Outputs for S-Function (fcncallgen): '<S2>/10ms' */

  /* S-Function (fcncallgen): '<S5>/10ms1' incorporates:
   *  SubSystem: '<S5>/Beeper'
   */
  /* S-Function (ec5744_pdsslbu3): '<S316>/PowerDriverSwitch(HS)' */

  /* Set level beeper_state for the specified power driver switch */
  ec_gpio_write(83,beeper_state);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms1' */

  /* S-Function (fcncallgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/PwrTrainTempPrtct'
   */
  /* MinMax: '<S8>/Max' */
  elapseTime = fmax(MCFL_TempInverter, MCFR_TempInverter);

  /* RelationalOperator: '<S84>/Compare' incorporates:
   *  Constant: '<S84>/Constant'
   */
  rtb_ignition = (elapseTime > 40.0);

  /* RelationalOperator: '<S85>/Compare' incorporates:
   *  Constant: '<S85>/Constant'
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
    WhlSpdFL = look1_binlx(elapseTime, VehCtrlMdel240918_2018b_ConstP.pooled18,
      VehCtrlMdel240918_2018b_ConstP.pooled17, 7U);
  } else {
    WhlSpdFL = 0.0;
  }

  /* End of Switch: '<S8>/Switch' */

  /* SignalConversion generated from: '<S8>/Out1' */
  WhlSpdFR = WhlSpdFL;

  /* Chart: '<S8>/Timer1' incorporates:
   *  Constant: '<S8>/Constant2'
   */
  VehCtrlMdel240918_20_Timer1(rtb_LogicalOperator2, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_g, &VehCtrlMdel240918_2018b_DW.sf_Timer1);

  /* RelationalOperator: '<S87>/Compare' incorporates:
   *  Constant: '<S87>/Constant'
   */
  rtb_ignition = (MCU_Temp > 50.0);

  /* Chart: '<S8>/Timer2' incorporates:
   *  Constant: '<S8>/Constant3'
   */
  VehCtrlMdel240918_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_d, &VehCtrlMdel240918_2018b_DW.sf_Timer2);

  /* Logic: '<S8>/NOT1' */
  rtb_Compare = !rtb_ignition;

  /* RelationalOperator: '<S86>/Compare' incorporates:
   *  Constant: '<S86>/Constant'
   */
  rtb_ignition = (MCU_Temp > 45.0);

  /* Logic: '<S8>/AND1' */
  rtb_ignition = (rtb_ignition && rtb_Compare);

  /* MinMax: '<S8>/Max1' */
  rtb_Switch2_on = fmax(MCU_Temp, motor_Temp);

  /* Switch: '<S8>/Switch1' incorporates:
   *  Constant: '<S8>/Constant1'
   */
  if (rtb_ignition) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table3' */
    WhlSpdFL = look1_binlx(rtb_Switch2_on,
      VehCtrlMdel240918_2018b_ConstP.pooled18,
      VehCtrlMdel240918_2018b_ConstP.pooled17, 7U);
  } else {
    WhlSpdFL = 0.0;
  }

  /* End of Switch: '<S8>/Switch1' */

  /* SignalConversion generated from: '<S8>/Out1' */
  WhlSpdRL_mps = WhlSpdFL;

  /* Lookup_n-D: '<S8>/2-D Lookup Table2' */
  WhlSpdFL = look1_binlx(rtb_Switch2_on,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable2_bp01Data,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable2_tableData, 6U);

  /* DataTypeConversion: '<S8>/Cast To Single' */
  rtb_CastToDouble = (real32_T)WhlSpdFL;

  /* Gain: '<S8>/Gain' */
  rtb_CastToDouble *= 10.0F;

  /* RelationalOperator: '<S88>/Compare' incorporates:
   *  Constant: '<S88>/Constant'
   */
  rtb_Compare = (elapseTime > 35.0);

  /* SignalConversion generated from: '<S8>/Out1' */
  VehCtrlMdel240918_2018b_B.aWaterPumpON = rtb_Compare;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/Function-Call Generator' */

  /* S-Function (fcncallgen): '<S1>/10ms1' incorporates:
   *  SubSystem: '<S1>/MoTrqReq'
   */
  if (VehCtrlMdel240918_2018b_DW.MoTrqReq_RESET_ELAPS_T) {
    FunctionCallSubsystem_ELAPS_T = 0U;
  } else {
    FunctionCallSubsystem_ELAPS_T = VehCtrlMdel240918_2018b_M->Timing.clockTick3
      - VehCtrlMdel240918_2018b_DW.MoTrqReq_PREV_T;
  }

  VehCtrlMdel240918_2018b_DW.MoTrqReq_PREV_T =
    VehCtrlMdel240918_2018b_M->Timing.clockTick3;
  VehCtrlMdel240918_2018b_DW.MoTrqReq_RESET_ELAPS_T = false;

  /* Lookup_n-D: '<S7>/2-D Lookup Table1' incorporates:
   *  UnitDelay: '<S183>/Unit Delay1'
   */
  rtb_Add6_p = look2_iflf_binlx(rtb_Acc_POS,
    VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable1_bp01Data_l,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable1_bp02Data,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable1_tableData_g,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable1_maxIndex, 11U);

  /* Gain: '<S10>/Gain4' */
  rtb_Acc_POS = 0.1F * rtb_Add6_p;

  /* Gain: '<S10>/Gain24' */
  rtb_Acc_POS *= 0.5F;

  /* SampleTimeMath: '<S33>/sample time'
   *
   * About '<S33>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S33>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant42'
   */
  rtb_g_mpss1 = 2000.0 * elapseTime;

  /* UnitDelay: '<S36>/Delay Input2'
   *
   * Block description for '<S36>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_d;

  /* SampleTimeMath: '<S36>/sample time'
   *
   * About '<S36>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Gain4 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S36>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(1000.0 * rtb_Gain4);

  /* Gain: '<S10>/Gain21' */
  rtb_Switch2_on = 0.1020408163265306 * rtb_Add2;

  /* MATLAB Function: '<S10>/Wtarget' incorporates:
   *  Constant: '<S10>/Constant14'
   *  Constant: '<S10>/Constant15'
   *  Constant: '<S10>/Constant16'
   *  Constant: '<S10>/Constant17'
   *  Constant: '<S10>/Constant18'
   *  Constant: '<S10>/Constant19'
   *  UnitDelay: '<S183>/Unit Delay1'
   */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k * (real32_T)
    rtb_deltafalllimit_iz / (340.0F *
    VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k *
    VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k * 15.5799866F / 460.0F /
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

  /* Sum: '<S36>/Difference Inputs1'
   *
   * Block description for '<S36>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_mn -= rtb_Add7;

  /* RelationalOperator: '<S51>/LowerRelop1' */
  rtb_ignition = (rtb_Switch2_mn > rtb_Switch2_b0);

  /* Switch: '<S51>/Switch2' */
  if (!rtb_ignition) {
    /* Product: '<S36>/delta fall limit' */
    rtb_Add10 = (real32_T)(-1000.0 * rtb_Gain4);

    /* RelationalOperator: '<S51>/UpperRelop' */
    rtb_ignition = (rtb_Switch2_mn < rtb_Add10);

    /* Switch: '<S51>/Switch' */
    if (rtb_ignition) {
      rtb_Switch2_mn = rtb_Add10;
    }

    /* End of Switch: '<S51>/Switch' */
    rtb_Switch2_b0 = rtb_Switch2_mn;
  }

  /* End of Switch: '<S51>/Switch2' */

  /* Sum: '<S36>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S36>/Delay Input2'
   *
   * Block description for '<S36>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S36>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_d = rtb_Switch2_b0 + rtb_Add7;

  /* Sum: '<S10>/Add' incorporates:
   *  UnitDelay: '<S36>/Delay Input2'
   *
   * Block description for '<S36>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdFL = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_d - rtb_UkYk1;

  /* UnitDelay: '<S10>/Unit Delay3' */
  rtb_ignition = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_l;

  /* Abs: '<S10>/Abs' */
  rtb_Gain4 = fabs(WhlSpdFL);

  /* RelationalOperator: '<S23>/Compare' incorporates:
   *  Constant: '<S23>/Constant'
   */
  rtb_LogicalOperator2 = (rtb_Gain4 > 10.0);

  /* Abs: '<S10>/Abs1' */
  rtb_Gain4 = fabs(rtb_UkYk1);

  /* RelationalOperator: '<S24>/Compare' incorporates:
   *  Constant: '<S24>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain4 > 5.0);

  /* RelationalOperator: '<S25>/Compare' incorporates:
   *  Constant: '<S25>/Constant'
   *  UnitDelay: '<S183>/Unit Delay1'
   */
  rtb_AND2 = (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k > 3.0F);

  /* UnitDelay: '<S7>/Unit Delay' */
  rtb_Gain4 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_n;

  /* Logic: '<S10>/AND' */
  rtb_UpperRelop_ir = (rtb_LogicalOperator2 && rtb_LowerRelop1_b && rtb_AND2 &&
                       (rtb_Gain4 != 0.0));

  /* Logic: '<S10>/Logical Operator4' */
  rtb_ignition = ((!rtb_ignition) && (!rtb_UpperRelop_ir));

  /* Abs: '<S10>/Abs2' */
  rtb_Gain4 = fabs(WhlSpdFL);

  /* RelationalOperator: '<S26>/Compare' incorporates:
   *  Constant: '<S26>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Gain4 < 10.0);

  /* RelationalOperator: '<S27>/Compare' incorporates:
   *  Constant: '<S27>/Constant'
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
    rtb_Switch2_b0 = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_n + 0.01F;
  } else {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Switch: '<S10>/Switch6' */

  /* MinMax: '<S10>/Min' incorporates:
   *  Constant: '<S10>/ResetDelay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_n = fminf(rtb_Switch2_b0, 0.5F);

  /* RelationalOperator: '<S10>/Relational Operator9' incorporates:
   *  Constant: '<S10>/ResetDelay'
   *  UnitDelay: '<S10>/Unit Delay4'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_n >= 0.5F);

  /* Logic: '<S10>/Logical Operator5' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_l = ((!rtb_ignition) &&
    (!rtb_UpperRelop_ir));

  /* Switch: '<S10>/Switch1' incorporates:
   *  Constant: '<S10>/Constant1'
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_l) {
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
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d *= WhlSpdFL;

  /* RelationalOperator: '<S22>/Compare' incorporates:
   *  Constant: '<S22>/Constant'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d <= 0.0);

  /* UnitDelay: '<S10>/Unit Delay' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d =
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_m;

  /* Switch: '<S10>/Switch' incorporates:
   *  Constant: '<S10>/Constant'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (rtb_UpperRelop_ir) {
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 0.0;
  }

  /* End of Switch: '<S10>/Switch' */

  /* Product: '<S10>/Product4' */
  WhlSpdRR_mps = rtb_Gain4;

  /* UnitDelay: '<S10>/Unit Delay1' */
  rtb_Gain4 = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_p;

  /* Product: '<S10>/Product5' */
  rtb_Switch2_on = rtb_Gain4;

  /* Sum: '<S10>/Add2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_m =
    (VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d + WhlSpdRR_mps) -
    rtb_Switch2_on;

  /* MATLAB Function: '<S10>/MATLAB Function1' incorporates:
   *  Constant: '<S10>/Constant20'
   *  Constant: '<S10>/Constant21'
   *  Constant: '<S10>/Constant22'
   *  Constant: '<S10>/Constant23'
   *  Constant: '<S10>/Constant24'
   *  Constant: '<S10>/Constant25'
   *  UnitDelay: '<S183>/Unit Delay1'
   */
  rtb_Add7 = ((5.43088F / VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k *
               (-1.35294116F / VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k) -
               (-0.0458234884F / VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k /
                VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k - 1.0F) *
               -3.33390474F) * 105.0F * (real32_T)rtb_deltafalllimit_iz -
              -1.35294116F / VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k *
              105.0F * (real32_T)rtb_UkYk1_ix) / (-0.0458234884F /
    VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k /
    VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k - 1.0F) / 100.0F;

  /* UnitDelay: '<S10>/Unit Delay6' */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.UnitDelay6_DSTATE_i;

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
    if (!VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_l) {
      rtb_Add7 = 0.0F;
    }

    /* End of Switch: '<S10>/Switch4' */
    VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE_b = rtb_Add7;
  }

  /* End of Switch: '<S10>/Switch3' */

  /* Gain: '<S10>/Gain18' incorporates:
   *  UnitDelay: '<S10>/Unit Delay5'
   */
  rtb_Switch2_b0 = 0.01F * VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE_b;

  /* Sum: '<S10>/Add1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay'
   */
  rtb_Switch2_on = (rtb_Gain5 + VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_m) +
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

  /* Sum: '<S33>/Difference Inputs1' incorporates:
   *  UnitDelay: '<S33>/Delay Input2'
   *
   * Block description for '<S33>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S33>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = rtb_UkYk1_ix - VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_jk;

  /* RelationalOperator: '<S48>/LowerRelop1' */
  rtb_UpperRelop_ir = (WhlSpdRR_mps > rtb_g_mpss1);

  /* Switch: '<S48>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S33>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S48>/UpperRelop' */
    rtb_ignition = (WhlSpdRR_mps < elapseTime);

    /* Switch: '<S48>/Switch' */
    if (rtb_ignition) {
      WhlSpdRR_mps = elapseTime;
    }

    /* End of Switch: '<S48>/Switch' */
    rtb_g_mpss1 = WhlSpdRR_mps;
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_jk += rtb_g_mpss1;

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
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 90.0 - FLWhlStrAng;

  /* Gain: '<S10>/Gain9' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d *= 0.017453292519943295;

  /* Trigonometry: '<S10>/Cos1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = cos
    (VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d);

  /* Gain: '<S10>/Gain10' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d *= 1.522;

  /* Sum: '<S10>/Add7' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  WhlSpdRR_mps += VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d;

  /* Product: '<S10>/Product1' incorporates:
   *  UnitDelay: '<S33>/Delay Input2'
   *
   * Block description for '<S33>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps *= VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_jk;

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
  elapseTime = look1_binlx(RPM, VehCtrlMdel240918_2018b_ConstP.u28_bp01Data,
    VehCtrlMdel240918_2018b_ConstP.u28_tableData, 26U);

  /* Lookup_n-D: '<S10>/AMK' */
  WhlSpdRR_mps = look1_binlx(MCFL_ActualVelocity,
    VehCtrlMdel240918_2018b_ConstP.pooled14,
    VehCtrlMdel240918_2018b_ConstP.pooled13, 19U);

  /* Lookup_n-D: '<S10>/AMK1' */
  rtb_Gain5 = look1_binlx(MCFR_ActualVelocity,
    VehCtrlMdel240918_2018b_ConstP.pooled14,
    VehCtrlMdel240918_2018b_ConstP.pooled13, 19U);

  /* MATLAB Function: '<S10>/�غ�ת��' incorporates:
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
  rtb_g_mpss1 = 0.1020408163265306 * rtb_deltafalllimit_i4;

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
  rtb_Compare = (WhlSpdRR_mps < 0.0);

  /* Gain: '<S10>/Gain2' */
  rtb_Switch2_mn = 0.95F * rtb_Add10;

  /* Gain: '<S10>/Gain5' */
  rtb_Add6_p *= 0.8F;

  /* Sum: '<S10>/Add15' */
  rtb_Add10 = rtb_Switch2_mn - rtb_Add6_p;

  /* RelationalOperator: '<S10>/Relational Operator1' incorporates:
   *  Constant: '<S10>/Constant38'
   */
  rtb_Compare_am = (rtb_Add10 < 0.0F);

  /* Logic: '<S10>/AND1' */
  rtb_UpperRelop_ir = (rtb_Compare && rtb_Compare_am);

  /* Logic: '<S10>/OR1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_l ||
                       rtb_UpperRelop_ir);

  /* Switch: '<S10>/Switch2' incorporates:
   *  Constant: '<S10>/Constant32'
   */
  if (rtb_UpperRelop_ir) {
    WhlSpdRR_mps = 0.0;
  } else {
    /* Logic: '<S10>/NOT' */
    rtb_ignition = !rtb_Compare_am;

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

  /* UnitDelay: '<S35>/Delay Input2'
   *
   * Block description for '<S35>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n0;

  /* Sum: '<S35>/Difference Inputs1'
   *
   * Block description for '<S35>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 = elapseTime - WhlSpdRR_mps;

  /* SampleTimeMath: '<S35>/sample time'
   *
   * About '<S35>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S35>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant45'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 1000.0 * elapseTime;

  /* RelationalOperator: '<S50>/LowerRelop1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_UpperRelop_ir = (rtb_Gain5 >
                       VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d);

  /* Switch: '<S50>/Switch2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S35>/delta fall limit' */
    elapseTime *= -1000.0;

    /* RelationalOperator: '<S50>/UpperRelop' */
    rtb_ignition = (rtb_Gain5 < elapseTime);

    /* Switch: '<S50>/Switch' */
    if (rtb_ignition) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S50>/Switch' */
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = rtb_Gain5;
  }

  /* End of Switch: '<S50>/Switch2' */

  /* Sum: '<S35>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n0 =
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d + WhlSpdRR_mps;

  /* Gain: '<S10>/Gain19' incorporates:
   *  UnitDelay: '<S35>/Delay Input2'
   *
   * Block description for '<S35>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = -VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n0;

  /* RelationalOperator: '<S39>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add6_p > rtb_Switch2_mn);

  /* Switch: '<S39>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Add6_p = rtb_Switch2_mn;
  } else {
    /* RelationalOperator: '<S39>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    rtb_ignition = (rtb_Add6_p < 0.0F);

    /* Switch: '<S39>/Switch' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    if (rtb_ignition) {
      rtb_Add6_p = 0.0F;
    }

    /* End of Switch: '<S39>/Switch' */
  }

  /* End of Switch: '<S39>/Switch2' */

  /* Sum: '<S10>/Add13' */
  elapseTime = rtb_Add6_p + WhlSpdRR_mps;

  /* RelationalOperator: '<S42>/LowerRelop1' */
  rtb_UpperRelop_ir = (elapseTime > rtb_Switch2_mn);

  /* Switch: '<S42>/Switch2' */
  if (rtb_UpperRelop_ir) {
    elapseTime = rtb_Switch2_mn;
  } else {
    /* RelationalOperator: '<S42>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    rtb_ignition = (elapseTime < 0.0);

    /* Switch: '<S42>/Switch' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    if (rtb_ignition) {
      elapseTime = 0.0;
    }

    /* End of Switch: '<S42>/Switch' */
  }

  /* End of Switch: '<S42>/Switch2' */

  /* UnitDelay: '<S32>/Delay Input2'
   *
   * Block description for '<S32>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hk;

  /* Sum: '<S32>/Difference Inputs1'
   *
   * Block description for '<S32>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 = elapseTime - WhlSpdRR_mps;

  /* SampleTimeMath: '<S32>/sample time'
   *
   * About '<S32>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S32>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 2000.0 * elapseTime;

  /* RelationalOperator: '<S47>/LowerRelop1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_UpperRelop_ir = (rtb_Gain5 >
                       VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d);

  /* Switch: '<S47>/Switch2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S32>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S47>/UpperRelop' */
    rtb_ignition = (rtb_Gain5 < elapseTime);

    /* Switch: '<S47>/Switch' */
    if (rtb_ignition) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S47>/Switch' */
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = rtb_Gain5;
  }

  /* End of Switch: '<S47>/Switch2' */

  /* Sum: '<S32>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hk =
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d + WhlSpdRR_mps;

  /* Sum: '<S18>/Add' incorporates:
   *  Constant: '<S18>/Constant3'
   */
  WhlSpdRR_mps = 1.0 - WhlSpdRL_mps;

  /* Product: '<S18>/Product5' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 63.0 * WhlSpdRR_mps;

  /* Product: '<S18>/Product' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d * 9550.0;

  /* Sum: '<S18>/Add1' incorporates:
   *  Constant: '<S18>/RPM_min'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = RPM + 10.0;

  /* MinMax: '<S18>/Max' incorporates:
   *  Constant: '<S18>/RPM_min1'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_Gain5 = fmax(VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d, 1.0);

  /* Product: '<S18>/Divide' */
  WhlSpdRR_mps /= rtb_Gain5;

  /* MinMax: '<S7>/MinMax2' incorporates:
   *  UnitDelay: '<S32>/Delay Input2'
   *
   * Block description for '<S32>/Delay Input2':
   *
   *  Store in Global RAM
   */
  elapseTime = fmin(WhlSpdRR_mps,
                    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hk);

  /* Saturate: '<S21>/Saturation' */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k > 40.0F) {
    rtb_Switch2_mn = 40.0F;
  } else if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k < 0.0F) {
    rtb_Switch2_mn = 0.0F;
  } else {
    rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k;
  }

  /* Lookup_n-D: '<S21>/VehSpd_SlipTarget_mps' */
  rtb_Add6_p = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled54, 3U);

  /* Sum: '<S21>/Add9' */
  rtb_Add6_p += rtb_Switch2_mn;

  /* Sum: '<S7>/Add1' */
  rtb_Ax = rtb_deltafalllimit_n + rtb_deltafalllimit_om;

  /* Gain: '<S7>/Gain2' */
  rtb_Ax *= 0.5F;

  /* Saturate: '<S21>/Saturation1' */
  if (rtb_Ax < 0.0F) {
    rtb_VxIMU_est = 0.0F;
  } else {
    rtb_VxIMU_est = rtb_Ax;
  }

  /* End of Saturate: '<S21>/Saturation1' */

  /* Sum: '<S21>/Add1' */
  rtb_deltafalllimit_n = rtb_Add6_p - rtb_VxIMU_est;

  /* UnitDelay: '<S78>/Unit Delay1' */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_e;

  /* RelationalOperator: '<S21>/Relational Operator7' incorporates:
   *  Constant: '<S21>/Cal_DeltaV_mps'
   */
  rtb_AND2 = (rtb_deltafalllimit_n < 0.0F);

  /* Logic: '<S78>/Logical Operator4' */
  rtb_UpperRelop_ir = ((!rtb_UpperRelop_ir) && (!rtb_AND2));

  /* Logic: '<S21>/Logical Operator2' */
  rtb_AND2 = !rtb_AND2;

  /* UnitDelay: '<S21>/Unit Delay4' */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE;

  /* RelationalOperator: '<S21>/Relational Operator8' incorporates:
   *  Constant: '<S21>/Cal_DeltaV_mps1'
   */
  rtb_LowerRelop1_b = (WhlSpdRR_mps > 235.0);

  /* Logic: '<S21>/Logical Operator1' */
  rtb_AND2 = (rtb_AND2 && rtb_LowerRelop1_b);

  /* Switch: '<S79>/Switch6' incorporates:
   *  Constant: '<S79>/Reset'
   */
  if (rtb_AND2) {
    /* Sum: '<S79>/Add10' incorporates:
     *  Constant: '<S79>/Steptime'
     */
    rtb_Ax = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_i + 0.01F;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S79>/Switch6' */

  /* MinMax: '<S79>/Min' incorporates:
   *  Constant: '<S21>/ResetDelay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_i = fminf(rtb_Ax, 0.1F);

  /* RelationalOperator: '<S79>/Relational Operator9' incorporates:
   *  Constant: '<S21>/ResetDelay'
   *  UnitDelay: '<S79>/Unit Delay1'
   */
  rtb_AND2 = (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_i >= 0.1F);

  /* UnitDelay: '<S21>/Unit Delay3' */
  rtb_LowerRelop1_b = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_a;

  /* Logic: '<S21>/Logical Operator3' */
  rtb_AND2 = (rtb_AND2 || rtb_LowerRelop1_b);

  /* Logic: '<S78>/Logical Operator5' incorporates:
   *  UnitDelay: '<S78>/Unit Delay1'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_e = ((!rtb_UpperRelop_ir) &&
    (!rtb_AND2));

  /* Switch: '<S21>/Switch6' incorporates:
   *  Constant: '<S21>/Verror_Reset'
   *  UnitDelay: '<S78>/Unit Delay1'
   */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_e) {
    rtb_Ax = rtb_deltafalllimit_n;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S21>/Switch6' */

  /* Product: '<S21>/Product' incorporates:
   *  Constant: '<S21>/P_Gain'
   */
  rtb_deltafalllimit_om = rtb_Ax * 40.0F;

  /* Sum: '<S21>/Add11' */
  WhlSpdRR_mps = elapseTime - rtb_deltafalllimit_om;

  /* UnitDelay: '<S21>/Unit Delay5' */
  rtb_Add6_p = VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE_l;

  /* Product: '<S21>/Product2' */
  rtb_Add6_p *= rtb_deltafalllimit_n;

  /* RelationalOperator: '<S73>/Compare' incorporates:
   *  Constant: '<S73>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add6_p <= 0.0F);

  /* UnitDelay: '<S21>/Unit Delay' */
  rtb_Add6_p = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_b;

  /* Switch: '<S21>/Switch3' incorporates:
   *  Constant: '<S21>/Verror_Reset1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add6_p = 0.0F;
  }

  /* End of Switch: '<S21>/Switch3' */

  /* Sum: '<S21>/Add2' */
  rtb_Add6_p += rtb_Ax;

  /* Saturate: '<S21>/Saturation2' */
  if (rtb_Add6_p > 400.0F) {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_b = 400.0F;
  } else if (rtb_Add6_p < -100.0F) {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_b = -100.0F;
  } else {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_b = rtb_Add6_p;
  }

  /* End of Saturate: '<S21>/Saturation2' */

  /* RelationalOperator: '<S83>/Compare' incorporates:
   *  UnitDelay: '<S78>/Unit Delay1'
   */
  rtb_ignition = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_e;

  /* UnitDelay: '<S77>/Delay Input1'
   *
   * Block description for '<S77>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.DelayInput1_DSTATE;

  /* RelationalOperator: '<S77>/FixPt Relational Operator' */
  rtb_UpperRelop_ir = ((int32_T)rtb_ignition > (int32_T)rtb_UpperRelop_ir);

  /* Switch: '<S21>/Switch' incorporates:
   *  Constant: '<S21>/Integr_StartPoint'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S21>/Add4' */
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = elapseTime -
      VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_g;
  } else {
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 0.0;
  }

  /* End of Switch: '<S21>/Switch' */

  /* Lookup_n-D: '<S21>/VehicleStableTarget_mps' */
  rtb_Ax = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled61, 3U);

  /* Sum: '<S21>/Add5' */
  rtb_Ax += rtb_Switch2_mn;

  /* Sum: '<S21>/Add10' */
  rtb_Ax = rtb_VxIMU_est - rtb_Ax;

  /* RelationalOperator: '<S21>/Relational Operator' incorporates:
   *  Constant: '<S21>/Verror'
   */
  rtb_UpperRelop_ir = (rtb_Ax < 0.0F);

  /* Logic: '<S21>/Logical Operator4' incorporates:
   *  UnitDelay: '<S78>/Unit Delay1'
   */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir &&
                       VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_e);

  /* Switch: '<S21>/Switch1' incorporates:
   *  Constant: '<S21>/Trq_IReset'
   *  Constant: '<S21>/Trq_I_FF'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Ax = 20.0F;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S21>/Switch1' */

  /* Sum: '<S21>/Add6' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S21>/Unit Delay'
   */
  rtb_Gain5 = (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_b +
               VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d) + rtb_Ax;

  /* Product: '<S21>/Product1' */
  WhlSpdRL_mps = rtb_Gain5 * 10.0;

  /* RelationalOperator: '<S80>/LowerRelop1' */
  rtb_UpperRelop_ir = (WhlSpdRL_mps > WhlSpdRR_mps);

  /* Switch: '<S80>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Gain: '<S21>/Gain3' */
    rtb_Add6_p = -rtb_deltafalllimit_om;

    /* RelationalOperator: '<S80>/UpperRelop' */
    rtb_LogicalOperator2 = (WhlSpdRL_mps < rtb_Add6_p);

    /* Switch: '<S80>/Switch' */
    if (rtb_LogicalOperator2) {
      WhlSpdRL_mps = rtb_Add6_p;
    }

    /* End of Switch: '<S80>/Switch' */
    WhlSpdRR_mps = WhlSpdRL_mps;
  }

  /* End of Switch: '<S80>/Switch2' */

  /* Sum: '<S21>/Add7' incorporates:
   *  UnitDelay: '<S21>/Unit Delay4'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE = rtb_deltafalllimit_om +
    WhlSpdRR_mps;

  /* Lookup_n-D: '<S21>/VehicleStableTarget_mps1' */
  rtb_Ax = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled61, 3U);

  /* Sum: '<S21>/Add13' */
  rtb_Switch2_mn += rtb_Ax;

  /* Sum: '<S21>/Add12' */
  rtb_Switch2_mn = rtb_VxIMU_est - rtb_Switch2_mn;

  /* RelationalOperator: '<S21>/Relational Operator1' incorporates:
   *  Constant: '<S21>/Verror1'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn < 0.0F);

  /* RelationalOperator: '<S21>/Relational Operator2' incorporates:
   *  UnitDelay: '<S21>/Unit Delay4'
   */
  rtb_AND2 = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE >= elapseTime);

  /* RelationalOperator: '<S75>/Compare' incorporates:
   *  Constant: '<S75>/Constant'
   *  UnitDelay: '<S21>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE <= 0.01);

  /* Logic: '<S21>/OR' */
  rtb_AND2 = (rtb_AND2 || rtb_LowerRelop1_b);

  /* Logic: '<S21>/Logical Operator5' incorporates:
   *  UnitDelay: '<S21>/Unit Delay3'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_a = (rtb_UpperRelop_ir &&
    rtb_AND2);

  /* Switch: '<S21>/Switch2' incorporates:
   *  Switch: '<S21>/Switch7'
   *  UnitDelay: '<S21>/Unit Delay3'
   *  UnitDelay: '<S78>/Unit Delay1'
   */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_e) {
    /* RelationalOperator: '<S81>/LowerRelop1' incorporates:
     *  Constant: '<S21>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S21>/Unit Delay4'
     */
    rtb_LogicalOperator2 = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE > 235.0);

    /* Switch: '<S81>/Switch2' incorporates:
     *  Constant: '<S21>/TCS_TrqRequest_Max2'
     */
    if (rtb_LogicalOperator2) {
      VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g = 235.0;
    } else {
      /* RelationalOperator: '<S81>/UpperRelop' incorporates:
       *  Constant: '<S21>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S21>/Unit Delay4'
       */
      rtb_LogicalOperator2 = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE < 0.0);

      /* Switch: '<S81>/Switch' incorporates:
       *  Constant: '<S21>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S21>/Unit Delay4'
       */
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g = 0.0;
      } else {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g =
          VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE;
      }

      /* End of Switch: '<S81>/Switch' */
    }

    /* End of Switch: '<S81>/Switch2' */

    /* RelationalOperator: '<S82>/LowerRelop1' */
    rtb_LogicalOperator2 = (elapseTime >
      VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g);

    /* Switch: '<S82>/Switch2' */
    if (!rtb_LogicalOperator2) {
      /* RelationalOperator: '<S82>/UpperRelop' incorporates:
       *  Constant: '<S21>/TCS_TrqRequest_Min1'
       */
      rtb_LogicalOperator2 = (elapseTime < 0.0);

      /* Switch: '<S82>/Switch' incorporates:
       *  Constant: '<S21>/TCS_TrqRequest_Min1'
       */
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g = 0.0;
      } else {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g = elapseTime;
      }

      /* End of Switch: '<S82>/Switch' */
    }

    /* End of Switch: '<S82>/Switch2' */
  } else {
    if (VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_a) {
      /* Switch: '<S21>/Switch7' */
      VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g = elapseTime;
    }
  }

  /* End of Switch: '<S21>/Switch2' */

  /* UnitDelay: '<S67>/Unit Delay1' */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_gl;

  /* Saturate: '<S20>/Saturation' */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k > 40.0F) {
    rtb_Ax = 40.0F;
  } else if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k < 0.0F) {
    rtb_Ax = 0.0F;
  } else {
    rtb_Ax = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Saturate: '<S20>/Saturation' */

  /* Lookup_n-D: '<S20>/VehSpd_SlipTarget_mps' */
  rtb_Switch2_mn = look1_iflf_binlc(rtb_Ax,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled54, 3U);

  /* Sum: '<S20>/Add9' */
  rtb_Switch2_mn += rtb_Ax;

  /* Saturate: '<S20>/Saturation1' */
  if (rtb_Divide > 50.0F) {
    rtb_Add6_p = 50.0F;
  } else if (rtb_Divide < 0.0F) {
    rtb_Add6_p = 0.0F;
  } else {
    rtb_Add6_p = rtb_Divide;
  }

  /* End of Saturate: '<S20>/Saturation1' */

  /* Sum: '<S20>/Add1' */
  rtb_Divide = rtb_Switch2_mn - rtb_Add6_p;

  /* RelationalOperator: '<S20>/Relational Operator7' incorporates:
   *  Constant: '<S20>/Cal_DeltaV_mps'
   */
  rtb_AND2 = (rtb_Divide < 0.0F);

  /* Logic: '<S67>/Logical Operator4' */
  rtb_UpperRelop_ir = ((!rtb_UpperRelop_ir) && (!rtb_AND2));

  /* Logic: '<S20>/Logical Operator2' */
  rtb_AND2 = !rtb_AND2;

  /* UnitDelay: '<S20>/Unit Delay4' */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_i;

  /* RelationalOperator: '<S20>/Relational Operator8' incorporates:
   *  Constant: '<S20>/Cal_DeltaV_mps1'
   */
  rtb_LowerRelop1_b = (WhlSpdRR_mps > 235.0);

  /* Logic: '<S20>/Logical Operator1' */
  rtb_AND2 = (rtb_AND2 && rtb_LowerRelop1_b);

  /* Switch: '<S68>/Switch6' incorporates:
   *  Constant: '<S68>/Reset'
   */
  if (rtb_AND2) {
    /* Sum: '<S68>/Add10' incorporates:
     *  Constant: '<S68>/Steptime'
     */
    rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_n5 + 0.01F;
  } else {
    rtb_Switch2_mn = 0.0F;
  }

  /* End of Switch: '<S68>/Switch6' */

  /* MinMax: '<S68>/Min' incorporates:
   *  Constant: '<S20>/ResetDelay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_n5 = fminf(rtb_Switch2_mn, 0.1F);

  /* RelationalOperator: '<S68>/Relational Operator9' incorporates:
   *  Constant: '<S20>/ResetDelay'
   *  UnitDelay: '<S68>/Unit Delay1'
   */
  rtb_AND2 = (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_n5 >= 0.1F);

  /* UnitDelay: '<S20>/Unit Delay3' */
  rtb_LowerRelop1_b = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_e;

  /* Logic: '<S20>/Logical Operator3' */
  rtb_AND2 = (rtb_AND2 || rtb_LowerRelop1_b);

  /* Logic: '<S67>/Logical Operator5' incorporates:
   *  UnitDelay: '<S67>/Unit Delay1'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_gl = ((!rtb_UpperRelop_ir) &&
    (!rtb_AND2));

  /* UnitDelay: '<S58>/Unit Delay1' */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_dp;

  /* Saturate: '<S19>/Saturation' */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k > 40.0F) {
    rtb_Switch2_mn = 40.0F;
  } else if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k < 0.0F) {
    rtb_Switch2_mn = 0.0F;
  } else {
    rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Saturate: '<S19>/Saturation' */

  /* Lookup_n-D: '<S19>/VehSpd_SlipTarget_mps' */
  rtb_MaxWhlSpd_mps_n = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled54, 3U);

  /* Sum: '<S19>/Add9' */
  rtb_MaxWhlSpd_mps_n += rtb_Switch2_mn;

  /* Saturate: '<S19>/Saturation1' */
  if (rtb_Add < 0.0F) {
    rtb_Add = 0.0F;
  }

  /* End of Saturate: '<S19>/Saturation1' */

  /* Sum: '<S19>/Add1' */
  rtb_Add4_j = rtb_MaxWhlSpd_mps_n - rtb_Add;

  /* RelationalOperator: '<S19>/Relational Operator7' incorporates:
   *  Constant: '<S19>/Cal_DeltaV_mps'
   */
  rtb_AND2 = (rtb_Add4_j < 0.0F);

  /* Logic: '<S58>/Logical Operator4' */
  rtb_UpperRelop_ir = ((!rtb_UpperRelop_ir) && (!rtb_AND2));

  /* Logic: '<S19>/Logical Operator2' */
  rtb_AND2 = !rtb_AND2;

  /* UnitDelay: '<S19>/Unit Delay4' */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_l;

  /* RelationalOperator: '<S19>/Relational Operator8' incorporates:
   *  Constant: '<S19>/Cal_DeltaV_mps1'
   */
  rtb_LowerRelop1_b = (WhlSpdRR_mps > 235.0);

  /* Logic: '<S19>/Logical Operator1' */
  rtb_AND2 = (rtb_AND2 && rtb_LowerRelop1_b);

  /* Switch: '<S59>/Switch6' incorporates:
   *  Constant: '<S59>/Reset'
   */
  if (rtb_AND2) {
    /* Sum: '<S59>/Add10' incorporates:
     *  Constant: '<S59>/Steptime'
     */
    rtb_MaxWhlSpd_mps_n = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_h + 0.01F;
  } else {
    rtb_MaxWhlSpd_mps_n = 0.0F;
  }

  /* End of Switch: '<S59>/Switch6' */

  /* MinMax: '<S59>/Min' incorporates:
   *  Constant: '<S19>/ResetDelay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_h = fminf(rtb_MaxWhlSpd_mps_n,
    0.1F);

  /* RelationalOperator: '<S59>/Relational Operator9' incorporates:
   *  Constant: '<S19>/ResetDelay'
   *  UnitDelay: '<S59>/Unit Delay1'
   */
  rtb_AND2 = (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_h >= 0.1F);

  /* UnitDelay: '<S19>/Unit Delay3' */
  rtb_LowerRelop1_b = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_i;

  /* Logic: '<S19>/Logical Operator3' */
  rtb_AND2 = (rtb_AND2 || rtb_LowerRelop1_b);

  /* Logic: '<S58>/Logical Operator5' incorporates:
   *  UnitDelay: '<S58>/Unit Delay1'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_dp = ((!rtb_UpperRelop_ir) &&
    (!rtb_AND2));

  /* Chart: '<S7>/Chart' */
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_c7_VehCtrlMdel240918_ ==
      0U) {
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_c7_VehCtrlMdel240918_ = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240918_2018b = 7U;
  } else {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240918_2018b)
    {
     case VehCtrlMdel240918_2018b_IN_A:
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240918_2018b = 1U;
      if (fabs(rtb_deltafalllimit_i4) > 0.5) {
        VehCtrlMdel240918_2018b_DW.a = atan(rtb_Add2 / rtb_deltafalllimit_i4) *
          180.0 / 3.1415926535897931;
      } else {
        VehCtrlMdel240918_2018b_DW.a = 0.0;
      }

      VehCtrlMdel240918_2018b_DW.b = rtb_Add2 * rtb_Add2 + rtb_deltafalllimit_i4
        * rtb_deltafalllimit_i4;
      VehCtrlMdel240918_2018b_DW.b = sqrt(VehCtrlMdel240918_2018b_DW.b);
      break;

     case VehCtrlMdel240918_2018b_IN_B:
      VehCtrlMdel240918_2018b_B.TCSR_Enable_OUT = 1.0;
      rtb_LogicalOperator2 = ((!(VehCtrlMdel240918_2018b_DW.DYC_flag != 0.0)) ||
        (rtb_UkYk1 > 30.0) || (VehCtrlMdel240918_2018b_DW.a > 30.0) ||
        (VehCtrlMdel240918_2018b_DW.a > -30.0) || (VehCtrlMdel240918_2018b_DW.b >
        3.0));
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240918_2018b =
          6U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_C:
      VehCtrlMdel240918_2018b_B.TCSR_Enable_OUT = 0.0;
      rtb_LogicalOperator2 = ((VehCtrlMdel240918_2018b_DW.DYC_flag != 0.0) &&
        (rtb_UkYk1 <= 30.0) && (VehCtrlMdel240918_2018b_DW.a >= 30.0) &&
        (VehCtrlMdel240918_2018b_DW.a <= -30.0) && (VehCtrlMdel240918_2018b_DW.b
        <= 3.0));
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240918_2018b =
          6U;
      }
      break;

     case VehCtrlMdel240_IN_DYC_Disenable:
      VehCtrlMdel240918_2018b_B.DYC_Enable_OUT = 0.0;
      break;

     case VehCtrlMdel240918_IN_DYC_Enable:
      VehCtrlMdel240918_2018b_B.DYC_Enable_OUT = 1.0;
      rtb_LogicalOperator2 = ((rtb_UkYk1 >= 50.0) || (rtb_deltafalllimit_i4 >=
        5.0) || (VehCtrlMdel240918_2018b_DW.b >= 5.0));
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240918_2018b =
          7U;
      }
      break;

     case VehCtrlMdel2_IN_F_TVD_TCS_STATE:
      rtb_LogicalOperator2 = ((VehCtrlMdel240918_2018b_DW.DYC_flag != 0.0) &&
        (rtb_UkYk1 <= 30.0) && (VehCtrlMdel240918_2018b_DW.a >= 30.0) &&
        (VehCtrlMdel240918_2018b_DW.a <= -30.0) && (VehCtrlMdel240918_2018b_DW.b
        <= 3.0));
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240918_2018b =
          2U;
        VehCtrlMdel240918_2018b_B.TCSR_Enable_OUT = 1.0;
      } else {
        rtb_LogicalOperator2 = ((!(VehCtrlMdel240918_2018b_DW.DYC_flag != 0.0)) ||
          (rtb_UkYk1 > 30.0) || (VehCtrlMdel240918_2018b_DW.a > 30.0) ||
          (VehCtrlMdel240918_2018b_DW.a > -30.0) ||
          (VehCtrlMdel240918_2018b_DW.b > 3.0));
        if (rtb_LogicalOperator2) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240918_2018b =
            3U;
          VehCtrlMdel240918_2018b_B.TCSR_Enable_OUT = 0.0;
        }
      }
      break;

     case VehCtrlMdel240918__IN_InitState:
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240918_2018b = 4U;
      VehCtrlMdel240918_2018b_B.DYC_Enable_OUT = 0.0;
      VehCtrlMdel240918_2018b_DW.DYC_flag = 0.0;
      break;

     case VehCtrlMdel24_IN_TCSF_Disenable:
      VehCtrlMdel240918_2018b_B.TCSF_Enable_OUT = 0.0;
      break;

     case VehCtrlMdel24091_IN_TCSF_Enable:
      VehCtrlMdel240918_2018b_B.TCSF_Enable_OUT = 1.0;
      if (VehCtrlMdel240918_2018b_DW.b > 5.0) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240918_2018b =
          7U;
      }
      break;

     case VehCtrlMdel24_IN_TCSR_Disenable:
      VehCtrlMdel240918_2018b_B.TCSR_Enable_OUT = 0.0;
      break;

     default:
      /* case IN_TCSR_Enable: */
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_c7_VehCtrlMdel240918_2018b = 10U;
      VehCtrlMdel240918_2018b_B.TCSR_Enable_OUT = 0.0;
      break;
    }
  }

  /* End of Chart: '<S7>/Chart' */

  /* Logic: '<S7>/Logical Operator5' */
  rtb_UpperRelop_ir = (VehCtrlMdel240918_2018b_B.MCFR_TorqueOn &&
                       VehCtrlMdel240918_2018b_B.MCFL_TorqueOn);

  /* Logic: '<S7>/Logical Operator6' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Logic: '<S7>/Logical Operator2' */
  rtb_AND2 = !VehCtrlMdel240918_2018b_B.VehReady;

  /* Logic: '<S7>/Logical Operator3' */
  rtb_LowerRelop1_b = (rtb_UpperRelop_ir || rtb_AND2 || (Trq_CUT != 0.0));

  /* Logic: '<S7>/Logical Operator1' */
  rtb_LogicalOperator2 = ((VehCtrlMdel240918_2018b_B.Exit_d != 0.0) ||
    rtb_LowerRelop1_b);

  /* Switch: '<S7>/Switch2' incorporates:
   *  Constant: '<S7>/Constant4'
   *  Switch: '<S7>/Switch5'
   */
  if (rtb_LogicalOperator2) {
    rtb_VxIMU_est = 0.0F;
  } else {
    if (VehCtrlMdel240918_2018b_B.TCSR_Enable_OUT != 0.0) {
      /* Abs: '<S21>/Abs' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_UkYk1 = fabs(rtb_deltafalllimit_iz);

      /* RelationalOperator: '<S74>/Compare' incorporates:
       *  Constant: '<S74>/Constant'
       *  Switch: '<S7>/Switch5'
       */
      rtb_AND2 = (rtb_UkYk1 <= 20.0);

      /* RelationalOperator: '<S76>/Compare' incorporates:
       *  Constant: '<S76>/Constant'
       *  Switch: '<S7>/Switch5'
       */
      rtb_UpperRelop_ir = (rtb_VxIMU_est > 0.0F);

      /* Logic: '<S21>/Logical Operator6' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_UpperRelop_ir = (rtb_UpperRelop_ir && rtb_AND2);

      /* Switch: '<S21>/Switch5' incorporates:
       *  Switch: '<S7>/Switch5'
       *  UnitDelay: '<S21>/Unit Delay2'
       */
      if (rtb_UpperRelop_ir) {
        elapseTime = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g;
      }

      /* End of Switch: '<S21>/Switch5' */
    }

    /* Gain: '<S7>/Gain6' */
    rtb_UkYk1 = 3.8461538461538463 * elapseTime;

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefRear' */
    rtb_VxIMU_est = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240918_2018b_ConstP.pooled32,
      VehCtrlMdel240918_2018b_ConstP.BrakeCompensateCoefRear_tableDa, 1U);

    /* RelationalOperator: '<S15>/LowerRelop1' */
    rtb_AND2 = (rtb_UkYk1 > rtb_VxIMU_est);

    /* Switch: '<S15>/Switch2' */
    if (!rtb_AND2) {
      /* RelationalOperator: '<S15>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      rtb_AND2 = (rtb_UkYk1 < 0.0);

      /* Switch: '<S15>/Switch' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      if (rtb_AND2) {
        rtb_VxIMU_est = 0.0F;
      } else {
        rtb_VxIMU_est = (real32_T)rtb_UkYk1;
      }

      /* End of Switch: '<S15>/Switch' */
    }

    /* End of Switch: '<S15>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch2' */

  /* UnitDelay: '<S12>/Delay Input2'
   *
   * Block description for '<S12>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_MaxWhlSpd_mps_n = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_cd;

  /* Sum: '<S12>/Difference Inputs1'
   *
   * Block description for '<S12>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1_ea = rtb_VxIMU_est - rtb_MaxWhlSpd_mps_n;

  /* SampleTimeMath: '<S12>/sample time'
   *
   * About '<S12>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S12>/delta rise limit' */
  rtb_VxIMU_est = (real32_T)(25000.0 * elapseTime);

  /* RelationalOperator: '<S52>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1_ea > rtb_VxIMU_est);

  /* Switch: '<S52>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S12>/delta fall limit' */
    rtb_VxIMU_est = (real32_T)(-25000.0 * elapseTime);

    /* RelationalOperator: '<S52>/UpperRelop' */
    rtb_AND2 = (rtb_UkYk1_ea < rtb_VxIMU_est);

    /* Switch: '<S52>/Switch' */
    if (rtb_AND2) {
      rtb_UkYk1_ea = rtb_VxIMU_est;
    }

    /* End of Switch: '<S52>/Switch' */
    rtb_VxIMU_est = rtb_UkYk1_ea;
  }

  /* End of Switch: '<S52>/Switch2' */

  /* Saturate: '<S7>/Saturation1' incorporates:
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_cd = rtb_VxIMU_est +
    rtb_MaxWhlSpd_mps_n;
  if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_cd > 1000.0F) {
    VehCtrlMdel240918_2018b_B.TrqR_cmd = 1000.0F;
  } else if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_cd < 0.0F) {
    VehCtrlMdel240918_2018b_B.TrqR_cmd = 0.0F;
  } else {
    VehCtrlMdel240918_2018b_B.TrqR_cmd =
      VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_cd;
  }

  /* End of Saturate: '<S7>/Saturation1' */

  /* Logic: '<S7>/Logical Operator4' */
  Trq_CUT_final = ((VehCtrlMdel240918_2018b_B.Exit_g != 0.0) ||
                   rtb_LowerRelop1_b || rtb_LogicalOperator2);

  /* UnitDelay: '<S30>/Delay Input2'
   *
   * Block description for '<S30>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_mb;

  /* SampleTimeMath: '<S30>/sample time'
   *
   * About '<S30>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S30>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 2000.0 * elapseTime;

  /* RelationalOperator: '<S37>/LowerRelop1' */
  rtb_UpperRelop_ir = (FLWhlStrAng > rtb_Switch2_b0);

  /* Switch: '<S37>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Gain5 = rtb_Switch2_b0;
  } else {
    /* RelationalOperator: '<S37>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    rtb_LowerRelop1_b = (FLWhlStrAng < 0.0);

    /* Switch: '<S37>/Switch' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    if (rtb_LowerRelop1_b) {
      FLWhlStrAng = 0.0;
    }

    /* End of Switch: '<S37>/Switch' */
    rtb_Gain5 = FLWhlStrAng;
  }

  /* End of Switch: '<S37>/Switch2' */

  /* UnitDelay: '<S34>/Delay Input2'
   *
   * Block description for '<S34>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_VxIMU_est = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_fo;

  /* SampleTimeMath: '<S34>/sample time'
   *
   * About '<S34>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Gain4 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S34>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant34'
   */
  rtb_MaxWhlSpd_mps_n = (real32_T)(1000.0 * rtb_Gain4);

  /* Logic: '<S10>/AND2' */
  rtb_UpperRelop_ir = (rtb_Compare && rtb_Compare_am);

  /* Logic: '<S10>/OR2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_l ||
                       rtb_UpperRelop_ir);

  /* Switch: '<S10>/Switch7' incorporates:
   *  Constant: '<S10>/Constant39'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add10 = 0.0F;
  } else {
    /* Logic: '<S10>/NOT1' */
    rtb_Compare = !rtb_Compare;

    /* Switch: '<S10>/Switch8' incorporates:
     *  Constant: '<S10>/Constant40'
     */
    if (!rtb_Compare) {
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

  /* Sum: '<S34>/Difference Inputs1'
   *
   * Block description for '<S34>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add10 -= rtb_VxIMU_est;

  /* RelationalOperator: '<S49>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add10 > rtb_MaxWhlSpd_mps_n);

  /* Switch: '<S49>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S34>/delta fall limit' */
    rtb_MaxWhlSpd_mps_n = (real32_T)(-1000.0 * rtb_Gain4);

    /* RelationalOperator: '<S49>/UpperRelop' */
    rtb_Compare = (rtb_Add10 < rtb_MaxWhlSpd_mps_n);

    /* Switch: '<S49>/Switch' */
    if (rtb_Compare) {
      rtb_Add10 = rtb_MaxWhlSpd_mps_n;
    }

    /* End of Switch: '<S49>/Switch' */
    rtb_MaxWhlSpd_mps_n = rtb_Add10;
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_fo = rtb_MaxWhlSpd_mps_n +
    rtb_VxIMU_est;

  /* Gain: '<S10>/Gain20' incorporates:
   *  UnitDelay: '<S34>/Delay Input2'
   *
   * Block description for '<S34>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = -VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_fo;

  /* Sum: '<S10>/Add11' */
  rtb_UkYk1 = rtb_Gain5 + rtb_Add10;

  /* RelationalOperator: '<S40>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1 > rtb_Switch2_b0);

  /* Switch: '<S40>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_UkYk1 = rtb_Switch2_b0;
  } else {
    /* RelationalOperator: '<S40>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    rtb_Compare = (rtb_UkYk1 < 0.0);

    /* Switch: '<S40>/Switch' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    if (rtb_Compare) {
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
  rtb_UkYk1 -= WhlSpdRR_mps;

  /* RelationalOperator: '<S45>/LowerRelop1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_UpperRelop_ir = (rtb_UkYk1 >
                       VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d);

  /* Switch: '<S45>/Switch2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S30>/delta fall limit' */
    rtb_deltafalllimit_iz = -2000.0 * elapseTime;

    /* RelationalOperator: '<S45>/UpperRelop' */
    rtb_Compare = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S45>/Switch' */
    if (rtb_Compare) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S45>/Switch' */
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = rtb_UkYk1;
  }

  /* End of Switch: '<S45>/Switch2' */

  /* Sum: '<S30>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_mb =
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d + WhlSpdRR_mps;

  /* Sum: '<S18>/Add4' incorporates:
   *  Constant: '<S18>/Constant'
   */
  WhlSpdRR_mps = 1.0 - WhlSpdFR;

  /* Product: '<S18>/Product4' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 7.0 * WhlSpdRR_mps;

  /* Product: '<S18>/Product1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_Gain5 = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d * 9550.0;

  /* Sum: '<S18>/Add2' incorporates:
   *  Constant: '<S18>/RPM_min2'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = MCFR_ActualVelocity + 10.0;

  /* MinMax: '<S18>/Max1' incorporates:
   *  Constant: '<S18>/RPM_min3'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_Gain4 = fmax(VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d, 1.0);

  /* Product: '<S18>/Divide1' */
  rtb_Gain5 /= rtb_Gain4;

  /* MinMax: '<S7>/MinMax' incorporates:
   *  UnitDelay: '<S30>/Delay Input2'
   *
   * Block description for '<S30>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = fmin(rtb_Gain5, VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_mb);

  /* Switch: '<S20>/Switch6' incorporates:
   *  Constant: '<S20>/Verror_Reset'
   *  UnitDelay: '<S67>/Unit Delay1'
   */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_gl) {
    rtb_VxIMU_est = rtb_Divide;
  } else {
    rtb_VxIMU_est = 0.0F;
  }

  /* End of Switch: '<S20>/Switch6' */

  /* Product: '<S20>/Product' incorporates:
   *  Constant: '<S20>/P_Gain'
   */
  rtb_Switch2_b0 = rtb_VxIMU_est * 40.0F;

  /* Sum: '<S20>/Add11' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = rtb_UkYk1 - rtb_Switch2_b0;

  /* UnitDelay: '<S20>/Unit Delay5' */
  rtb_MaxWhlSpd_mps_n = VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE_i;

  /* Product: '<S20>/Product2' */
  rtb_MaxWhlSpd_mps_n *= rtb_Divide;

  /* RelationalOperator: '<S64>/Compare' incorporates:
   *  Constant: '<S64>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_MaxWhlSpd_mps_n <= 0.0F);

  /* UnitDelay: '<S20>/Unit Delay' */
  rtb_MaxWhlSpd_mps_n = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_f;

  /* Switch: '<S20>/Switch3' incorporates:
   *  Constant: '<S20>/Verror_Reset1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_MaxWhlSpd_mps_n = 0.0F;
  }

  /* End of Switch: '<S20>/Switch3' */

  /* Sum: '<S20>/Add2' */
  rtb_MaxWhlSpd_mps_n += rtb_VxIMU_est;

  /* Saturate: '<S20>/Saturation2' */
  if (rtb_MaxWhlSpd_mps_n > 400.0F) {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_f = 400.0F;
  } else if (rtb_MaxWhlSpd_mps_n < -100.0F) {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_f = -100.0F;
  } else {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_f = rtb_MaxWhlSpd_mps_n;
  }

  /* End of Saturate: '<S20>/Saturation2' */

  /* RelationalOperator: '<S72>/Compare' incorporates:
   *  UnitDelay: '<S67>/Unit Delay1'
   */
  rtb_Compare = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_gl;

  /* UnitDelay: '<S66>/Delay Input1'
   *
   * Block description for '<S66>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.DelayInput1_DSTATE_j;

  /* RelationalOperator: '<S66>/FixPt Relational Operator' */
  rtb_UpperRelop_ir = ((int32_T)rtb_Compare > (int32_T)rtb_UpperRelop_ir);

  /* Switch: '<S20>/Switch' incorporates:
   *  Constant: '<S20>/Integr_StartPoint'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S20>/Add4' */
    rtb_Gain5 = rtb_UkYk1 - VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_f;
  } else {
    rtb_Gain5 = 0.0;
  }

  /* End of Switch: '<S20>/Switch' */

  /* Lookup_n-D: '<S20>/VehicleStableTarget_mps' */
  rtb_VxIMU_est = look1_iflf_binlc(rtb_Ax,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled61, 3U);

  /* Sum: '<S20>/Add5' */
  rtb_VxIMU_est += rtb_Ax;

  /* Sum: '<S20>/Add10' */
  rtb_VxIMU_est = rtb_Add6_p - rtb_VxIMU_est;

  /* RelationalOperator: '<S20>/Relational Operator' incorporates:
   *  Constant: '<S20>/Verror'
   */
  rtb_UpperRelop_ir = (rtb_VxIMU_est < 0.0F);

  /* Logic: '<S20>/Logical Operator4' incorporates:
   *  UnitDelay: '<S67>/Unit Delay1'
   */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir &&
                       VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_gl);

  /* Switch: '<S20>/Switch1' incorporates:
   *  Constant: '<S20>/Trq_IReset'
   *  Constant: '<S20>/Trq_I_FF'
   */
  if (rtb_UpperRelop_ir) {
    rtb_VxIMU_est = 20.0F;
  } else {
    rtb_VxIMU_est = 0.0F;
  }

  /* End of Switch: '<S20>/Switch1' */

  /* Sum: '<S20>/Add6' incorporates:
   *  UnitDelay: '<S20>/Unit Delay'
   */
  rtb_Gain4 = (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_f + rtb_Gain5) +
    rtb_VxIMU_est;

  /* Product: '<S20>/Product1' */
  rtb_deltafalllimit_iz = rtb_Gain4 * 10.0;

  /* RelationalOperator: '<S69>/LowerRelop1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_UpperRelop_ir = (rtb_deltafalllimit_iz >
                       VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d);

  /* Switch: '<S69>/Switch2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (!rtb_UpperRelop_ir) {
    /* Gain: '<S20>/Gain3' */
    rtb_VxIMU_est = -rtb_Switch2_b0;

    /* RelationalOperator: '<S69>/UpperRelop' */
    rtb_Compare_am = (rtb_deltafalllimit_iz < rtb_VxIMU_est);

    /* Switch: '<S69>/Switch' */
    if (rtb_Compare_am) {
      rtb_deltafalllimit_iz = rtb_VxIMU_est;
    }

    /* End of Switch: '<S69>/Switch' */
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = rtb_deltafalllimit_iz;
  }

  /* End of Switch: '<S69>/Switch2' */

  /* Sum: '<S20>/Add7' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S20>/Unit Delay4'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_i = rtb_Switch2_b0 +
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d;

  /* Lookup_n-D: '<S20>/VehicleStableTarget_mps1' */
  rtb_VxIMU_est = look1_iflf_binlc(rtb_Ax,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled61, 3U);

  /* Sum: '<S20>/Add13' */
  rtb_Ax += rtb_VxIMU_est;

  /* Sum: '<S20>/Add12' */
  rtb_Add6_p -= rtb_Ax;

  /* RelationalOperator: '<S20>/Relational Operator1' incorporates:
   *  Constant: '<S20>/Verror1'
   */
  rtb_UpperRelop_ir = (rtb_Add6_p < 0.0F);

  /* RelationalOperator: '<S20>/Relational Operator2' incorporates:
   *  UnitDelay: '<S20>/Unit Delay4'
   */
  rtb_AND2 = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_i >= rtb_UkYk1);

  /* RelationalOperator: '<S65>/Compare' incorporates:
   *  Constant: '<S65>/Constant'
   *  UnitDelay: '<S20>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_i <= 0.01);

  /* Logic: '<S20>/OR' */
  rtb_AND2 = (rtb_AND2 || rtb_LowerRelop1_b);

  /* Logic: '<S20>/Logical Operator5' incorporates:
   *  UnitDelay: '<S20>/Unit Delay3'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_e = (rtb_UpperRelop_ir &&
    rtb_AND2);

  /* Switch: '<S20>/Switch2' incorporates:
   *  Switch: '<S20>/Switch7'
   *  UnitDelay: '<S20>/Unit Delay3'
   *  UnitDelay: '<S67>/Unit Delay1'
   */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_gl) {
    /* RelationalOperator: '<S70>/LowerRelop1' incorporates:
     *  Constant: '<S20>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S20>/Unit Delay4'
     */
    rtb_Compare_am = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_i > 235.0);

    /* Switch: '<S70>/Switch2' incorporates:
     *  Constant: '<S20>/TCS_TrqRequest_Max2'
     */
    if (rtb_Compare_am) {
      VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j = 235.0;
    } else {
      /* RelationalOperator: '<S70>/UpperRelop' incorporates:
       *  Constant: '<S20>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S20>/Unit Delay4'
       */
      rtb_Compare_am = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_i < 0.0);

      /* Switch: '<S70>/Switch' incorporates:
       *  Constant: '<S20>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S20>/Unit Delay4'
       */
      if (rtb_Compare_am) {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j = 0.0;
      } else {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j =
          VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_i;
      }

      /* End of Switch: '<S70>/Switch' */
    }

    /* End of Switch: '<S70>/Switch2' */

    /* RelationalOperator: '<S71>/LowerRelop1' */
    rtb_Compare_am = (rtb_UkYk1 > VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j);

    /* Switch: '<S71>/Switch2' */
    if (!rtb_Compare_am) {
      /* RelationalOperator: '<S71>/UpperRelop' incorporates:
       *  Constant: '<S20>/TCS_TrqRequest_Min1'
       */
      rtb_Compare_am = (rtb_UkYk1 < 0.0);

      /* Switch: '<S71>/Switch' incorporates:
       *  Constant: '<S20>/TCS_TrqRequest_Min1'
       */
      if (rtb_Compare_am) {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j = 0.0;
      } else {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j = rtb_UkYk1;
      }

      /* End of Switch: '<S71>/Switch' */
    }

    /* End of Switch: '<S71>/Switch2' */
  } else {
    if (VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_e) {
      /* Switch: '<S20>/Switch7' */
      VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j = rtb_UkYk1;
    }
  }

  /* End of Switch: '<S20>/Switch2' */

  /* Switch: '<S7>/Switch3' incorporates:
   *  Constant: '<S7>/Constant6'
   *  Switch: '<S7>/Switch6'
   */
  if (Trq_CUT_final) {
    rtb_Add6_p = 0.0F;
  } else {
    if (VehCtrlMdel240918_2018b_B.TCSF_Enable_OUT != 0.0) {
      /* Switch: '<S7>/Switch6' incorporates:
       *  UnitDelay: '<S20>/Unit Delay2'
       */
      rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j;
    }

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefFront1' */
    rtb_Add6_p = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240918_2018b_ConstP.pooled32,
      VehCtrlMdel240918_2018b_ConstP.pooled31, 1U);

    /* RelationalOperator: '<S16>/LowerRelop1' */
    rtb_Compare_am = (rtb_UkYk1 > rtb_Add6_p);

    /* Switch: '<S16>/Switch2' */
    if (!rtb_Compare_am) {
      /* RelationalOperator: '<S16>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      rtb_Compare_am = (rtb_UkYk1 < 0.0);

      /* Switch: '<S16>/Switch' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      if (rtb_Compare_am) {
        rtb_Add6_p = 0.0F;
      } else {
        rtb_Add6_p = (real32_T)rtb_UkYk1;
      }

      /* End of Switch: '<S16>/Switch' */
    }

    /* End of Switch: '<S16>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch3' */

  /* UnitDelay: '<S13>/Delay Input2'
   *
   * Block description for '<S13>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_VxIMU_est = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hn;

  /* Sum: '<S13>/Difference Inputs1'
   *
   * Block description for '<S13>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add6_p -= rtb_VxIMU_est;

  /* SampleTimeMath: '<S13>/sample time'
   *
   * About '<S13>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S13>/delta rise limit' */
  rtb_MaxWhlSpd_mps_n = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S53>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add6_p > rtb_MaxWhlSpd_mps_n);

  /* Switch: '<S53>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S13>/delta fall limit' */
    rtb_Ax = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S53>/UpperRelop' */
    rtb_Compare_am = (rtb_Add6_p < rtb_Ax);

    /* Switch: '<S53>/Switch' */
    if (rtb_Compare_am) {
      rtb_Add6_p = rtb_Ax;
    }

    /* End of Switch: '<S53>/Switch' */
    rtb_MaxWhlSpd_mps_n = rtb_Add6_p;
  }

  /* End of Switch: '<S53>/Switch2' */

  /* Saturate: '<S7>/Saturation2' incorporates:
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hn = rtb_MaxWhlSpd_mps_n +
    rtb_VxIMU_est;
  if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hn > 20.0F) {
    VehCtrlMdel240918_2018b_B.TrqFR_cmd = 20.0F;
  } else if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hn < 0.0F) {
    VehCtrlMdel240918_2018b_B.TrqFR_cmd = 0.0F;
  } else {
    VehCtrlMdel240918_2018b_B.TrqFR_cmd =
      VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hn;
  }

  /* End of Saturate: '<S7>/Saturation2' */

  /* Product: '<S18>/Product3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 7.0 * WhlSpdRR_mps;

  /* Product: '<S18>/Product2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d * 9550.0;

  /* Sum: '<S18>/Add3' incorporates:
   *  Constant: '<S18>/RPM_min4'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = MCFL_ActualVelocity + 10.0;

  /* MinMax: '<S18>/Max2' incorporates:
   *  Constant: '<S18>/RPM_min5'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_Gain5 = fmax(VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d, 1.0);

  /* Product: '<S18>/Divide2' */
  WhlSpdRR_mps /= rtb_Gain5;

  /* UnitDelay: '<S31>/Delay Input2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *
   * Block description for '<S31>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_o;

  /* SampleTimeMath: '<S31>/sample time'
   *
   * About '<S31>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S31>/delta rise limit' incorporates:
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
   *  UnitDelay: '<S33>/Delay Input2'
   *
   * Block description for '<S33>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain4 *= VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_jk;

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

  /* RelationalOperator: '<S38>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Gain4 > rtb_Add7);

  /* Switch: '<S38>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Gain4 = rtb_Add7;
  } else {
    /* RelationalOperator: '<S38>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    rtb_Compare_am = (rtb_Gain4 < 0.0);

    /* Switch: '<S38>/Switch' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    if (rtb_Compare_am) {
      rtb_Gain4 = 0.0;
    }

    /* End of Switch: '<S38>/Switch' */
  }

  /* End of Switch: '<S38>/Switch2' */

  /* Sum: '<S10>/Add12' */
  rtb_Yk1_l = rtb_Gain4 + rtb_Add10;

  /* RelationalOperator: '<S41>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Yk1_l > rtb_Add7);

  /* Switch: '<S41>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Yk1_l = rtb_Add7;
  } else {
    /* RelationalOperator: '<S41>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    rtb_Compare_am = (rtb_Yk1_l < 0.0);

    /* Switch: '<S41>/Switch' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    if (rtb_Compare_am) {
      rtb_Yk1_l = 0.0;
    }

    /* End of Switch: '<S41>/Switch' */
  }

  /* End of Switch: '<S41>/Switch2' */

  /* Sum: '<S31>/Difference Inputs1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *
   * Block description for '<S31>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Yk1_l -= VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d;

  /* RelationalOperator: '<S46>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Yk1_l > rtb_Gain5);

  /* Switch: '<S46>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S31>/delta fall limit' */
    rtb_UkYk1 = -2000.0 * elapseTime;

    /* RelationalOperator: '<S46>/UpperRelop' */
    rtb_Compare_am = (rtb_Yk1_l < rtb_UkYk1);

    /* Switch: '<S46>/Switch' */
    if (rtb_Compare_am) {
      rtb_Yk1_l = rtb_UkYk1;
    }

    /* End of Switch: '<S46>/Switch' */
    rtb_Gain5 = rtb_Yk1_l;
  }

  /* End of Switch: '<S46>/Switch2' */

  /* Sum: '<S31>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_o = rtb_Gain5 +
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d;

  /* MinMax: '<S7>/MinMax1' incorporates:
   *  UnitDelay: '<S31>/Delay Input2'
   *
   * Block description for '<S31>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = fmin(WhlSpdRR_mps, VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_o);

  /* Switch: '<S19>/Switch6' incorporates:
   *  Constant: '<S19>/Verror_Reset'
   *  UnitDelay: '<S58>/Unit Delay1'
   */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_dp) {
    rtb_Add10 = rtb_Add4_j;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S19>/Switch6' */

  /* Product: '<S19>/Product' incorporates:
   *  Constant: '<S19>/P_Gain'
   */
  rtb_Acc_POS = rtb_Add10 * 40.0F;

  /* Sum: '<S19>/Add11' */
  WhlSpdRR_mps = rtb_Yk1_l - rtb_Acc_POS;

  /* UnitDelay: '<S19>/Unit Delay5' */
  rtb_VxIMU_est = VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE_ip;

  /* Product: '<S19>/Product2' */
  rtb_VxIMU_est *= rtb_Add4_j;

  /* RelationalOperator: '<S55>/Compare' incorporates:
   *  Constant: '<S55>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_VxIMU_est <= 0.0F);

  /* UnitDelay: '<S19>/Unit Delay' */
  rtb_VxIMU_est = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_nr;

  /* Switch: '<S19>/Switch3' incorporates:
   *  Constant: '<S19>/Verror_Reset1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_VxIMU_est = 0.0F;
  }

  /* End of Switch: '<S19>/Switch3' */

  /* Sum: '<S19>/Add2' */
  rtb_VxIMU_est += rtb_Add10;

  /* Saturate: '<S19>/Saturation2' */
  if (rtb_VxIMU_est > 400.0F) {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_nr = 400.0F;
  } else if (rtb_VxIMU_est < -100.0F) {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_nr = -100.0F;
  } else {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_nr = rtb_VxIMU_est;
  }

  /* End of Saturate: '<S19>/Saturation2' */

  /* RelationalOperator: '<S63>/Compare' incorporates:
   *  UnitDelay: '<S58>/Unit Delay1'
   */
  rtb_Compare_am = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_dp;

  /* UnitDelay: '<S57>/Delay Input1'
   *
   * Block description for '<S57>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.DelayInput1_DSTATE_b;

  /* RelationalOperator: '<S57>/FixPt Relational Operator' */
  rtb_UpperRelop_ir = ((int32_T)rtb_Compare_am > (int32_T)rtb_UpperRelop_ir);

  /* Switch: '<S19>/Switch' incorporates:
   *  Constant: '<S19>/Integr_StartPoint'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S19>/Add4' */
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = rtb_Yk1_l -
      VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_g4;
  } else {
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 0.0;
  }

  /* End of Switch: '<S19>/Switch' */

  /* Lookup_n-D: '<S19>/VehicleStableTarget_mps' */
  rtb_Add10 = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled61, 3U);

  /* Sum: '<S19>/Add5' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S19>/Add10' */
  rtb_Add10 = rtb_Add - rtb_Add10;

  /* RelationalOperator: '<S19>/Relational Operator' incorporates:
   *  Constant: '<S19>/Verror'
   */
  rtb_UpperRelop_ir = (rtb_Add10 < 0.0F);

  /* Logic: '<S19>/Logical Operator4' incorporates:
   *  UnitDelay: '<S58>/Unit Delay1'
   */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir &&
                       VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_dp);

  /* Switch: '<S19>/Switch1' incorporates:
   *  Constant: '<S19>/Trq_IReset'
   *  Constant: '<S19>/Trq_I_FF'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add10 = 20.0F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S19>/Switch1' */

  /* Sum: '<S19>/Add6' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S19>/Unit Delay'
   */
  rtb_Gain5 = (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_nr +
               VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d) + rtb_Add10;

  /* Product: '<S19>/Product1' */
  rtb_UkYk1 = rtb_Gain5 * 10.0;

  /* RelationalOperator: '<S60>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1 > WhlSpdRR_mps);

  /* Switch: '<S60>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Gain: '<S19>/Gain3' */
    rtb_Add6_p = -rtb_Acc_POS;

    /* RelationalOperator: '<S60>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_UkYk1 < rtb_Add6_p);

    /* Switch: '<S60>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_UkYk1 = rtb_Add6_p;
    }

    /* End of Switch: '<S60>/Switch' */
    WhlSpdRR_mps = rtb_UkYk1;
  }

  /* End of Switch: '<S60>/Switch2' */

  /* Sum: '<S19>/Add7' incorporates:
   *  UnitDelay: '<S19>/Unit Delay4'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_l = rtb_Acc_POS + WhlSpdRR_mps;

  /* Lookup_n-D: '<S19>/VehicleStableTarget_mps1' */
  rtb_Add10 = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled61, 3U);

  /* Sum: '<S19>/Add13' */
  rtb_Switch2_mn += rtb_Add10;

  /* Sum: '<S19>/Add12' */
  rtb_Add -= rtb_Switch2_mn;

  /* RelationalOperator: '<S19>/Relational Operator1' incorporates:
   *  Constant: '<S19>/Verror1'
   */
  rtb_UpperRelop_ir = (rtb_Add < 0.0F);

  /* RelationalOperator: '<S19>/Relational Operator2' incorporates:
   *  UnitDelay: '<S19>/Unit Delay4'
   */
  rtb_AND2 = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_l >= rtb_Yk1_l);

  /* RelationalOperator: '<S56>/Compare' incorporates:
   *  Constant: '<S56>/Constant'
   *  UnitDelay: '<S19>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_l <= 0.01);

  /* Logic: '<S19>/OR' */
  rtb_AND2 = (rtb_AND2 || rtb_LowerRelop1_b);

  /* Logic: '<S19>/Logical Operator5' */
  rtb_LowerRelop1_b = (rtb_UpperRelop_ir && rtb_AND2);

  /* Switch: '<S19>/Switch2' incorporates:
   *  Switch: '<S19>/Switch7'
   *  UnitDelay: '<S58>/Unit Delay1'
   */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_dp) {
    /* RelationalOperator: '<S61>/LowerRelop1' incorporates:
     *  Constant: '<S19>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S19>/Unit Delay4'
     */
    rtb_AND2 = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_l > 235.0);

    /* Switch: '<S61>/Switch2' incorporates:
     *  Constant: '<S19>/TCS_TrqRequest_Max2'
     */
    if (rtb_AND2) {
      VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b = 235.0;
    } else {
      /* RelationalOperator: '<S61>/UpperRelop' incorporates:
       *  Constant: '<S19>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S19>/Unit Delay4'
       */
      rtb_AND2 = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_l < 0.0);

      /* Switch: '<S61>/Switch' incorporates:
       *  Constant: '<S19>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S19>/Unit Delay4'
       */
      if (rtb_AND2) {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b = 0.0;
      } else {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b =
          VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_l;
      }

      /* End of Switch: '<S61>/Switch' */
    }

    /* End of Switch: '<S61>/Switch2' */

    /* RelationalOperator: '<S62>/LowerRelop1' */
    rtb_AND2 = (rtb_Yk1_l > VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b);

    /* Switch: '<S62>/Switch2' */
    if (!rtb_AND2) {
      /* RelationalOperator: '<S62>/UpperRelop' incorporates:
       *  Constant: '<S19>/TCS_TrqRequest_Min1'
       */
      rtb_AND2 = (rtb_Yk1_l < 0.0);

      /* Switch: '<S62>/Switch' incorporates:
       *  Constant: '<S19>/TCS_TrqRequest_Min1'
       */
      if (rtb_AND2) {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b = 0.0;
      } else {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b = rtb_Yk1_l;
      }

      /* End of Switch: '<S62>/Switch' */
    }

    /* End of Switch: '<S62>/Switch2' */
  } else {
    if (rtb_LowerRelop1_b) {
      /* Switch: '<S19>/Switch7' */
      VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b = rtb_Yk1_l;
    }
  }

  /* End of Switch: '<S19>/Switch2' */

  /* Switch: '<S7>/Switch4' incorporates:
   *  Constant: '<S7>/Constant8'
   *  Switch: '<S7>/Switch7'
   */
  if (Trq_CUT_final) {
    rtb_Add = 0.0F;
  } else {
    if (VehCtrlMdel240918_2018b_B.TCSF_Enable_OUT != 0.0) {
      /* Switch: '<S7>/Switch7' incorporates:
       *  UnitDelay: '<S19>/Unit Delay2'
       */
      rtb_Yk1_l = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b;
    }

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefFront2' */
    rtb_Add = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240918_2018b_ConstP.pooled32,
      VehCtrlMdel240918_2018b_ConstP.pooled31, 1U);

    /* RelationalOperator: '<S17>/LowerRelop1' */
    rtb_AND2 = (rtb_Yk1_l > rtb_Add);

    /* Switch: '<S17>/Switch2' */
    if (!rtb_AND2) {
      /* RelationalOperator: '<S17>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      rtb_AND2 = (rtb_Yk1_l < 0.0);

      /* Switch: '<S17>/Switch' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      if (rtb_AND2) {
        rtb_Add = 0.0F;
      } else {
        rtb_Add = (real32_T)rtb_Yk1_l;
      }

      /* End of Switch: '<S17>/Switch' */
    }

    /* End of Switch: '<S17>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch4' */

  /* UnitDelay: '<S14>/Delay Input2'
   *
   * Block description for '<S14>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_ib;

  /* Sum: '<S14>/Difference Inputs1'
   *
   * Block description for '<S14>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add -= rtb_Add10;

  /* SampleTimeMath: '<S14>/sample time'
   *
   * About '<S14>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S14>/delta rise limit' */
  rtb_VxIMU_est = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S54>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add > rtb_VxIMU_est);

  /* Switch: '<S54>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S14>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S54>/UpperRelop' */
    rtb_AND2 = (rtb_Add < rtb_Add6_p);

    /* Switch: '<S54>/Switch' */
    if (rtb_AND2) {
      rtb_Add = rtb_Add6_p;
    }

    /* End of Switch: '<S54>/Switch' */
    rtb_VxIMU_est = rtb_Add;
  }

  /* End of Switch: '<S54>/Switch2' */

  /* Saturate: '<S7>/Saturation3' incorporates:
   *  Sum: '<S14>/Difference Inputs2'
   *  UnitDelay: '<S14>/Delay Input2'
   *
   * Block description for '<S14>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S14>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_ib = rtb_VxIMU_est + rtb_Add10;
  if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_ib > 20.0F) {
    VehCtrlMdel240918_2018b_B.TrqFL_cmd = 20.0F;
  } else if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_ib < 0.0F) {
    VehCtrlMdel240918_2018b_B.TrqFL_cmd = 0.0F;
  } else {
    VehCtrlMdel240918_2018b_B.TrqFL_cmd =
      VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_ib;
  }

  /* End of Saturate: '<S7>/Saturation3' */

  /* Sum: '<S10>/Add3' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_p = rtb_Switch2_on - rtb_UkYk1_ix;

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
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_n =
    VehCtrlMdel240918_2018b_B.DYC_Enable_OUT;

  /* Update for UnitDelay: '<S10>/Unit Delay2' */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = WhlSpdFL;

  /* Update for UnitDelay: '<S10>/Unit Delay6' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay6_DSTATE_i =
    VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_l;

  /* Update for UnitDelay: '<S21>/Unit Delay5' */
  VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE_l = rtb_deltafalllimit_n;

  /* Update for UnitDelay: '<S21>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_g = rtb_deltafalllimit_om;

  /* Update for UnitDelay: '<S77>/Delay Input1'
   *
   * Block description for '<S77>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput1_DSTATE = rtb_ignition;

  /* Update for UnitDelay: '<S19>/Unit Delay3' */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_i = rtb_LowerRelop1_b;

  /* Update for UnitDelay: '<S20>/Unit Delay5' */
  VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE_i = rtb_Divide;

  /* Update for UnitDelay: '<S20>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_f = rtb_Switch2_b0;

  /* Update for UnitDelay: '<S66>/Delay Input1'
   *
   * Block description for '<S66>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput1_DSTATE_j = rtb_Compare;

  /* Update for UnitDelay: '<S19>/Unit Delay5' */
  VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE_ip = rtb_Add4_j;

  /* Update for UnitDelay: '<S19>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_g4 = rtb_Acc_POS;

  /* Update for UnitDelay: '<S57>/Delay Input1'
   *
   * Block description for '<S57>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput1_DSTATE_b = rtb_Compare_am;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/10ms1' */

  /* S-Function (fcncallgen): '<S5>/10ms2' incorporates:
   *  SubSystem: '<S5>/VCU2AMKMCUFL'
   */
  /* S-Function (scanpack): '<S317>/CAN Pack1' incorporates:
   *  Constant: '<S317>/Constant'
   *  Constant: '<S317>/Constant1'
   */
  /* S-Function (scanpack): '<S317>/CAN Pack1' */
  VehCtrlMdel240918_2018b_B.CANPack1_d.ID = 386U;
  VehCtrlMdel240918_2018b_B.CANPack1_d.Length = 8U;
  VehCtrlMdel240918_2018b_B.CANPack1_d.Extended = 0U;
  VehCtrlMdel240918_2018b_B.CANPack1_d.Remote = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_d.Data[0] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_d.Data[1] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_d.Data[2] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_d.Data[3] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_d.Data[4] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_d.Data[5] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_d.Data[6] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_d.Data[7] = 0;

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
        real32_T result = VehCtrlMdel240918_2018b_B.TrqFL_cmd;

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
            VehCtrlMdel240918_2018b_B.CANPack1_d.Data[2] =
              VehCtrlMdel240918_2018b_B.CANPack1_d.Data[2] | (uint8_T)((uint16_T)
              (tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240918_2018b_B.CANPack1_d.Data[3] =
              VehCtrlMdel240918_2018b_B.CANPack1_d.Data[3] | (uint8_T)((uint16_T)
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
            VehCtrlMdel240918_2018b_B.CANPack1_d.Data[6] =
              VehCtrlMdel240918_2018b_B.CANPack1_d.Data[6] | (uint8_T)((uint16_T)
              (tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240918_2018b_B.CANPack1_d.Data[7] =
              VehCtrlMdel240918_2018b_B.CANPack1_d.Data[7] | (uint8_T)((uint16_T)
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
        real64_T result = 10.0;

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
            VehCtrlMdel240918_2018b_B.CANPack1_d.Data[4] =
              VehCtrlMdel240918_2018b_B.CANPack1_d.Data[4] | (uint8_T)((uint16_T)
              (tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240918_2018b_B.CANPack1_d.Data[5] =
              VehCtrlMdel240918_2018b_B.CANPack1_d.Data[5] | (uint8_T)((uint16_T)
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
          (VehCtrlMdel240918_2018b_B.MCFL_DCOn_setpoints);

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
            VehCtrlMdel240918_2018b_B.CANPack1_d.Data[1] =
              VehCtrlMdel240918_2018b_B.CANPack1_d.Data[1] | (uint8_T)((uint8_T)
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
        uint32_T result = (uint32_T) (VehCtrlMdel240918_2018b_B.MCFL_DCEnable);

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
            VehCtrlMdel240918_2018b_B.CANPack1_d.Data[1] =
              VehCtrlMdel240918_2018b_B.CANPack1_d.Data[1] | (uint8_T)((uint8_T)
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
        real64_T result = VehCtrlMdel240918_2018b_B.errorReset;

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
            VehCtrlMdel240918_2018b_B.CANPack1_d.Data[1] =
              VehCtrlMdel240918_2018b_B.CANPack1_d.Data[1] | (uint8_T)((uint8_T)
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
        uint32_T result = (uint32_T) (VehCtrlMdel240918_2018b_B.MCFL_InverterOn);

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
            VehCtrlMdel240918_2018b_B.CANPack1_d.Data[1] =
              VehCtrlMdel240918_2018b_B.CANPack1_d.Data[1] | (uint8_T)((uint8_T)
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

  /* S-Function (ecucoder_canmessage): '<S317>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    VehCtrlMdel240918_2018b_B.CANPackMessage_h[0]=
      VehCtrlMdel240918_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage_h[1]=
      VehCtrlMdel240918_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage_h[2]=
      VehCtrlMdel240918_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage_h[3]=
      VehCtrlMdel240918_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage_h[4]=
      VehCtrlMdel240918_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage_h[5]=
      VehCtrlMdel240918_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage_h[6]=
      VehCtrlMdel240918_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage_h[7]=
      VehCtrlMdel240918_2018b_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S317>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN1BUF8TX[8];
    uint8 can1buf8looptx= 0;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_h[0];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_h[1];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_h[2];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_h[3];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_h[4];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_h[5];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_h[6];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_h[7];
    can1buf8looptx++;
    VehCtrlMdel240918_2018b_B.CANTransmit_c= ec_can_transmit(1, 8, 0, 386U, 8,
      CAN1BUF8TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms2' */

  /* S-Function (fcncallgen): '<S5>/10ms4' incorporates:
   *  SubSystem: '<S5>/VCU2AMKMCUFR'
   */
  /* S-Function (scanpack): '<S318>/CAN Pack1' incorporates:
   *  Constant: '<S318>/Constant'
   *  Constant: '<S318>/Constant1'
   */
  /* S-Function (scanpack): '<S318>/CAN Pack1' */
  VehCtrlMdel240918_2018b_B.CANPack1.ID = 387U;
  VehCtrlMdel240918_2018b_B.CANPack1.Length = 8U;
  VehCtrlMdel240918_2018b_B.CANPack1.Extended = 0U;
  VehCtrlMdel240918_2018b_B.CANPack1.Remote = 0;
  VehCtrlMdel240918_2018b_B.CANPack1.Data[0] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1.Data[1] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1.Data[2] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1.Data[3] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1.Data[4] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1.Data[5] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1.Data[6] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1.Data[7] = 0;

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
        real32_T result = VehCtrlMdel240918_2018b_B.TrqFR_cmd;

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
            VehCtrlMdel240918_2018b_B.CANPack1.Data[2] =
              VehCtrlMdel240918_2018b_B.CANPack1.Data[2] | (uint8_T)((uint16_T)
              (tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240918_2018b_B.CANPack1.Data[3] =
              VehCtrlMdel240918_2018b_B.CANPack1.Data[3] | (uint8_T)((uint16_T)
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
            VehCtrlMdel240918_2018b_B.CANPack1.Data[6] =
              VehCtrlMdel240918_2018b_B.CANPack1.Data[6] | (uint8_T)((uint16_T)
              (tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240918_2018b_B.CANPack1.Data[7] =
              VehCtrlMdel240918_2018b_B.CANPack1.Data[7] | (uint8_T)((uint16_T)
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
        real64_T result = 10.0;

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
            VehCtrlMdel240918_2018b_B.CANPack1.Data[4] =
              VehCtrlMdel240918_2018b_B.CANPack1.Data[4] | (uint8_T)((uint16_T)
              (tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240918_2018b_B.CANPack1.Data[5] =
              VehCtrlMdel240918_2018b_B.CANPack1.Data[5] | (uint8_T)((uint16_T)
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
          (VehCtrlMdel240918_2018b_B.MCFL_DCOn_setpoints);

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
            VehCtrlMdel240918_2018b_B.CANPack1.Data[1] =
              VehCtrlMdel240918_2018b_B.CANPack1.Data[1] | (uint8_T)((uint8_T)
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
        uint32_T result = (uint32_T) (VehCtrlMdel240918_2018b_B.MCFL_DCEnable);

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
            VehCtrlMdel240918_2018b_B.CANPack1.Data[1] =
              VehCtrlMdel240918_2018b_B.CANPack1.Data[1] | (uint8_T)((uint8_T)
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
        real64_T result = VehCtrlMdel240918_2018b_B.errorReset;

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
            VehCtrlMdel240918_2018b_B.CANPack1.Data[1] =
              VehCtrlMdel240918_2018b_B.CANPack1.Data[1] | (uint8_T)((uint8_T)
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
        uint32_T result = (uint32_T) (VehCtrlMdel240918_2018b_B.MCFL_InverterOn);

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
            VehCtrlMdel240918_2018b_B.CANPack1.Data[1] =
              VehCtrlMdel240918_2018b_B.CANPack1.Data[1] | (uint8_T)((uint8_T)
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

  /* S-Function (ecucoder_canmessage): '<S318>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    VehCtrlMdel240918_2018b_B.CANPackMessage[0]=
      VehCtrlMdel240918_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage[1]=
      VehCtrlMdel240918_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage[2]=
      VehCtrlMdel240918_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage[3]=
      VehCtrlMdel240918_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage[4]=
      VehCtrlMdel240918_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage[5]=
      VehCtrlMdel240918_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage[6]=
      VehCtrlMdel240918_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage[7]=
      VehCtrlMdel240918_2018b_B.CANPack1.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S318>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN1BUF9TX[8];
    uint8 can1buf9looptx= 0;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage[0];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage[1];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage[2];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage[3];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage[4];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage[5];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage[6];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage[7];
    can1buf9looptx++;
    VehCtrlMdel240918_2018b_B.CANTransmit_l= ec_can_transmit(1, 9, 0, 387U, 8,
      CAN1BUF9TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms4' */

  /* S-Function (fcncallgen): '<S5>/10ms3' incorporates:
   *  SubSystem: '<S5>/VCU2EmraxMCU'
   */
  /* Switch: '<S319>/Switch2' incorporates:
   *  Constant: '<S319>/Constant13'
   *  Constant: '<S319>/Constant17'
   *  Constant: '<S319>/Constant19'
   *  Constant: '<S319>/Constant20'
   *  Switch: '<S319>/Switch3'
   */
  if (rtb_LogicalOperator2) {
    Gear_Trs = 0.0;
    Mode_Trs = 0.0;
  } else {
    Gear_Trs = 2.0;
    Mode_Trs = 2.0;
  }

  /* End of Switch: '<S319>/Switch2' */

  /* DataTypeConversion: '<S319>/Cast To Boolean4' */
  VehCtrlMdel240918_2018b_B.CastToBoolean4 = (uint16_T)Gear_Trs;

  /* DataTypeConversion: '<S319>/Cast To Boolean6' */
  VehCtrlMdel240918_2018b_B.CastToBoolean6 = (uint16_T)Mode_Trs;

  /* DataTypeConversion: '<S319>/Data Type Conversion2' */
  VehCtrlMdel240918_2018b_B.DataTypeConversion2 = (int32_T)floor(rtb_Switch2_on);

  /* S-Function (scanpack): '<S319>/CAN Pack1' */
  /* S-Function (scanpack): '<S319>/CAN Pack1' */
  VehCtrlMdel240918_2018b_B.CANPack1_a.ID = 146927393U;
  VehCtrlMdel240918_2018b_B.CANPack1_a.Length = 8U;
  VehCtrlMdel240918_2018b_B.CANPack1_a.Extended = 1U;
  VehCtrlMdel240918_2018b_B.CANPack1_a.Remote = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_a.Data[0] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_a.Data[1] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_a.Data[2] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_a.Data[3] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_a.Data[4] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_a.Data[5] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_a.Data[6] = 0;
  VehCtrlMdel240918_2018b_B.CANPack1_a.Data[7] = 0;

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
        uint32_T result = (uint32_T) (VehCtrlMdel240918_2018b_B.CastToBoolean4);

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
            VehCtrlMdel240918_2018b_B.CANPack1_a.Data[4] =
              VehCtrlMdel240918_2018b_B.CANPack1_a.Data[4] | (uint8_T)
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
        uint32_T result = (uint32_T) (VehCtrlMdel240918_2018b_B.CastToBoolean6);

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
            VehCtrlMdel240918_2018b_B.CANPack1_a.Data[5] =
              VehCtrlMdel240918_2018b_B.CANPack1_a.Data[5] | (uint8_T)((uint8_T)
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
        real32_T result = VehCtrlMdel240918_2018b_B.TrqR_cmd;

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
            VehCtrlMdel240918_2018b_B.CANPack1_a.Data[0] =
              VehCtrlMdel240918_2018b_B.CANPack1_a.Data[0] | (uint8_T)((uint16_T)
              (packedValue & (uint16_T)0xFFU));
            VehCtrlMdel240918_2018b_B.CANPack1_a.Data[1] =
              VehCtrlMdel240918_2018b_B.CANPack1_a.Data[1] | (uint8_T)((uint16_T)
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
          (VehCtrlMdel240918_2018b_B.DataTypeConversion2);

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
            VehCtrlMdel240918_2018b_B.CANPack1_a.Data[2] =
              VehCtrlMdel240918_2018b_B.CANPack1_a.Data[2] | (uint8_T)((uint16_T)
              (packedValue & (uint16_T)0xFFU));
            VehCtrlMdel240918_2018b_B.CANPack1_a.Data[3] =
              VehCtrlMdel240918_2018b_B.CANPack1_a.Data[3] | (uint8_T)((uint16_T)
              ((uint16_T)(packedValue & (uint16_T)0xFF00U) >> 8));
          }
        }
      }
    }
  }

  /* S-Function (ecucoder_canmessage): '<S319>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    VehCtrlMdel240918_2018b_B.CANPackMessage_d[0]=
      VehCtrlMdel240918_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage_d[1]=
      VehCtrlMdel240918_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage_d[2]=
      VehCtrlMdel240918_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage_d[3]=
      VehCtrlMdel240918_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage_d[4]=
      VehCtrlMdel240918_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage_d[5]=
      VehCtrlMdel240918_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage_d[6]=
      VehCtrlMdel240918_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240918_2018b_B.CANPackMessage_d[7]=
      VehCtrlMdel240918_2018b_B.CANPack1_a.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S319>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN0BUF8TX[8];
    uint8 can0buf8looptx= 0;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_d[0];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_d[1];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_d[2];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_d[3];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_d[4];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_d[5];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_d[6];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240918_2018b_B.CANPackMessage_d[7];
    can0buf8looptx++;
    VehCtrlMdel240918_2018b_B.CANTransmit_k= ec_can_transmit(0, 8, 1, 146927393U,
      8, CAN0BUF8TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms3' */

  /* S-Function (fcncallgen): '<S5>/10ms6' incorporates:
   *  SubSystem: '<S5>/WP_OUTPUT'
   */
  /* DataTypeConversion: '<S320>/Cast To Single1' */
  VehCtrlMdel240918_2018b_B.CastToSingle1 = (uint16_T)rtb_CastToDouble;

  /* S-Function (ec5744_pdsslbu3): '<S320>/PowerDriverSwitch(HS)' */

  /* Set level VehCtrlMdel240918_2018b_B.aWaterPumpON for the specified power driver switch */
  ec_gpio_write(50,VehCtrlMdel240918_2018b_B.aWaterPumpON);
  ec_gpio_write(42,VehCtrlMdel240918_2018b_B.aWaterPumpON);

  /* S-Function (ec5744_pdpslbu3): '<S320>/PowerDriverPWM' incorporates:
   *  Constant: '<S320>/Constant'
   */

  /* Power driver PWM output for channel 6 */
  ec_pwm_output(6,((uint16_T)1000U),VehCtrlMdel240918_2018b_B.CastToSingle1);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms6' */

  /* S-Function (fcncallgen): '<S325>/10ms' incorporates:
   *  SubSystem: '<S325>/daq10ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S337>/CCPDAQ' */
  ccpDaq(1);

  /* End of Outputs for S-Function (fcncallgen): '<S325>/10ms' */

  /* Update absolute time */
  /* The "clockTick3" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick3" ensures timer will not overflow during the
   * application lifespan selected.
   */
  VehCtrlMdel240918_2018b_M->Timing.clockTick3++;
}

/* Model step function for TID4 */
void VehCtrlMdel240918_2018b_step4(void) /* Sample time: [0.05s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S325>/50ms' incorporates:
   *  SubSystem: '<S325>/daq50ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S339>/CCPDAQ' */
  ccpDaq(2);

  /* End of Outputs for S-Function (fcncallgen): '<S325>/50ms' */
}

/* Model step function for TID5 */
void VehCtrlMdel240918_2018b_step5(void) /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S324>/100MS' incorporates:
   *  SubSystem: '<S324>/Function-Call Subsystem'
   */
  /* S-Function (ec5744_canreceiveslb): '<S328>/CANReceive' */

  /* Receive CAN message */
  {
    uint8 CAN2BUF1RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can2buf1looprx= 0;
    VehCtrlMdel240918_2018b_B.CANReceive_o3_l= 278;
    VehCtrlMdel240918_2018b_B.CANReceive_o5_l= 8;
    VehCtrlMdel240918_2018b_B.CANReceive_o2_p= ec_can_receive(2,1, CAN2BUF1RX);
    VehCtrlMdel240918_2018b_B.CANReceive_o4_i[0]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive_o4_i[1]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive_o4_i[2]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive_o4_i[3]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive_o4_i[4]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive_o4_i[5]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive_o4_i[6]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel240918_2018b_B.CANReceive_o4_i[7]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
  }

  /* Call the system: <S328>/Function-Call Subsystem */

  /* Output and update for function-call system: '<S328>/Function-Call Subsystem' */
  {
    uint8_T rtb_Add;
    uint8_T rtb_Compare;

    /* Outputs for Enabled SubSystem: '<S329>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S330>/Enable'
     */
    if (VehCtrlMdel240918_2018b_B.CANReceive_o2_p > 0) {
      /* RelationalOperator: '<S331>/Compare' incorporates:
       *  Constant: '<S331>/Constant'
       */
      rtb_Add = (uint8_T)(VehCtrlMdel240918_2018b_B.CANReceive_o4_i[0] == 83);

      /* RelationalOperator: '<S332>/Compare' incorporates:
       *  Constant: '<S332>/Constant'
       */
      rtb_Compare = (uint8_T)(VehCtrlMdel240918_2018b_B.CANReceive_o4_i[5] == 84);

      /* Sum: '<S330>/Add' */
      rtb_Add = (uint8_T)((uint32_T)rtb_Add + rtb_Compare);

      /* RelationalOperator: '<S333>/Compare' incorporates:
       *  Constant: '<S333>/Constant'
       */
      rtb_Compare = (uint8_T)(rtb_Add == 2);

      /* If: '<S330>/If' */
      if (rtb_Compare > 0) {
        /* Outputs for IfAction SubSystem: '<S330>/If Action Subsystem' incorporates:
         *  ActionPort: '<S334>/Action Port'
         */
        /* S-Function (ec5744_bootloaderslb): '<S334>/BootLoader' */
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

        /* S-Function (ec5744_cpuresetslb): '<S334>/CPUReset' */

        /* Perform a microcontroller reset */
        MC_ME.MCTL.R = 0X00005AF0;
        MC_ME.MCTL.R = 0X0000A50F;

        /* End of Outputs for SubSystem: '<S330>/If Action Subsystem' */
      } else {
        /* Outputs for IfAction SubSystem: '<S330>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S335>/Action Port'
         */
        /* S-Function (ec5744_cantransmitslb): '<S335>/CANTransmit' incorporates:
         *  Constant: '<S335>/Constant'
         */

        /*Transmit CAN message*/
        {
          uint8 CAN2BUF9TX[1];
          uint8 can2buf9looptx= 0;
          CAN2BUF9TX[can2buf9looptx]= ((uint8_T)1U);
          can2buf9looptx++;
          VehCtrlMdel240918_2018b_B.CANTransmit= ec_can_transmit(2, 9, 0, 593U,
            1, CAN2BUF9TX);
        }

        /* End of Outputs for SubSystem: '<S330>/If Action Subsystem1' */
      }

      /* End of If: '<S330>/If' */
    }

    /* End of Outputs for SubSystem: '<S329>/Enabled Subsystem' */
  }

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S328>/CANReceive' */
  /* End of Outputs for S-Function (fcncallgen): '<S324>/100MS' */

  /* S-Function (fcncallgen): '<S325>/100ms' incorporates:
   *  SubSystem: '<S325>/daq100ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S336>/CCPDAQ' */
  ccpDaq(3);

  /* End of Outputs for S-Function (fcncallgen): '<S325>/100ms' */
}

/* Model step function for TID6 */
void VehCtrlMdel240918_2018b_step6(void) /* Sample time: [0.5s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S325>/500ms' incorporates:
   *  SubSystem: '<S325>/daq500ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S338>/CCPDAQ' */
  ccpDaq(4);

  /* End of Outputs for S-Function (fcncallgen): '<S325>/500ms' */

  /* S-Function (fcncallgen): '<S326>/500ms' incorporates:
   *  SubSystem: '<S326>/EEPROMOperation'
   */

  /* S-Function (ec5744_eepromoslb): '<S341>/EEPROMOperatin' */
#if defined EC_EEPROM_ENABLE

  /* Operate the EEPROM module on the MPC5744 */
  ec_flash_operation();

#endif

  /* End of Outputs for S-Function (fcncallgen): '<S326>/500ms' */
}

/* Model step wrapper function for compatibility with a static main program */
void VehCtrlMdel240918_2018b_step(int_T tid)
{
  switch (tid) {
   case 0 :
    VehCtrlMdel240918_2018b_step0();
    break;

   case 1 :
    VehCtrlMdel240918_2018b_step1();
    break;

   case 2 :
    VehCtrlMdel240918_2018b_step2();
    break;

   case 3 :
    VehCtrlMdel240918_2018b_step3();
    break;

   case 4 :
    VehCtrlMdel240918_2018b_step4();
    break;

   case 5 :
    VehCtrlMdel240918_2018b_step5();
    break;

   case 6 :
    VehCtrlMdel240918_2018b_step6();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void VehCtrlMdel240918_2018b_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* Start for S-Function (fcncallgen): '<S3>/10ms6' incorporates:
   *  SubSystem: '<S3>/EMRAXMCU_RECIEVE'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S102>/CANReceive1' incorporates:
   *  SubSystem: '<S102>/MCU_pwr'
   */
  /* Start for function-call system: '<S102>/MCU_pwr' */

  /* Start for Enabled SubSystem: '<S155>/MCU_VCUMeter1' */

  /* Start for S-Function (scanunpack): '<S157>/CAN Unpack' */

  /*-----------S-Function Block: <S157>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S155>/MCU_VCUMeter1' */
  ec_buffer_init(0,1,1,218089455);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S102>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S102>/CANReceive3' incorporates:
   *  SubSystem: '<S102>/MCU_state'
   */
  /* Start for function-call system: '<S102>/MCU_state' */

  /* Start for Enabled SubSystem: '<S156>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S162>/CAN Unpack' */

  /*-----------S-Function Block: <S162>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S156>/MCU_state' */
  ec_buffer_init(0,0,1,218089199);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S102>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S98>/CANReceive3' incorporates:
   *  SubSystem: '<S98>/ABS_BUS_state'
   */
  /* Start for function-call system: '<S98>/ABS_BUS_state' */

  /* Start for Enabled SubSystem: '<S106>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S107>/CAN Unpack1' */

  /*-----------S-Function Block: <S107>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S106>/IMU_state' */
  ec_buffer_init(1,16,0,1698);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S98>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S104>/CANReceive3' incorporates:
   *  SubSystem: '<S104>/StrWhSnis_state'
   */
  /* Start for function-call system: '<S104>/StrWhSnis_state' */

  /* Start for Enabled SubSystem: '<S174>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S175>/CAN Unpack1' */

  /*-----------S-Function Block: <S175>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S174>/IMU_state' */
  ec_buffer_init(1,7,0,330);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S104>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms4' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S112>/CANReceive3' incorporates:
   *  SubSystem: '<S112>/AMKMCU_state'
   */
  /* Start for function-call system: '<S112>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S114>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S117>/CAN Unpack' */

  /*-----------S-Function Block: <S117>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S114>/MCU_state' */
  ec_buffer_init(1,1,0,640);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S112>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S112>/CANReceive1' incorporates:
   *  SubSystem: '<S112>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S112>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S115>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S126>/CAN Unpack' */

  /*-----------S-Function Block: <S126>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S115>/MCU_state' */
  ec_buffer_init(1,2,0,642);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S112>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S112>/CANReceive2' incorporates:
   *  SubSystem: '<S112>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S112>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S116>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S128>/CAN Unpack' */

  /*-----------S-Function Block: <S128>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S116>/MCU_state' */
  ec_buffer_init(1,3,0,644);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S112>/CANReceive2' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S113>/CANReceive3' incorporates:
   *  SubSystem: '<S113>/AMKMCU_state'
   */
  /* Start for function-call system: '<S113>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S132>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S135>/CAN Unpack' */

  /*-----------S-Function Block: <S135>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S132>/MCU_state' */
  ec_buffer_init(1,4,0,641);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S113>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S113>/CANReceive1' incorporates:
   *  SubSystem: '<S113>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S113>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S133>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S143>/CAN Unpack' */

  /*-----------S-Function Block: <S143>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S133>/MCU_state' */
  ec_buffer_init(1,5,0,643);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S113>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S113>/CANReceive2' incorporates:
   *  SubSystem: '<S113>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S113>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S134>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S145>/CAN Unpack' */

  /*-----------S-Function Block: <S145>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S134>/MCU_state' */
  ec_buffer_init(1,0,0,645);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S113>/CANReceive2' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms5' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S103>/CANReceive3' incorporates:
   *  SubSystem: '<S103>/IMU_state'
   */
  /* Start for function-call system: '<S103>/IMU_state' */

  /* Start for Enabled SubSystem: '<S169>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S170>/CAN Unpack' */

  /*-----------S-Function Block: <S170>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S169>/MCU_state' */
  ec_buffer_init(1,17,0,513);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S103>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms2' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S101>/CANReceive3' */
  ec_buffer_init(0,3,1,408961267);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S101>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S317>/CANTransmit' */
  ec_buffer_init(1,8,0,386U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms2' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S318>/CANTransmit' */
  ec_buffer_init(1,9,0,387U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms4' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S319>/CANTransmit' */
  ec_buffer_init(0,8,1,146927393U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S5>/10ms6' incorporates:
   *  SubSystem: '<S5>/WP_OUTPUT'
   */
  /* Start for S-Function (ec5744_pdpslbu3): '<S320>/PowerDriverPWM' incorporates:
   *  Constant: '<S320>/Constant'
   */

  /* Initialize PWM output for channel 6 */
  SIUL2_MSCR42 = 0X02000003;           //PWM_A3

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S324>/100MS' incorporates:
   *  SubSystem: '<S324>/Function-Call Subsystem'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S328>/CANReceive' incorporates:
   *  SubSystem: '<S328>/Function-Call Subsystem'
   */
  /* Start for function-call system: '<S328>/Function-Call Subsystem' */

  /* Start for Enabled SubSystem: '<S329>/Enabled Subsystem' */
  /* Start for IfAction SubSystem: '<S330>/If Action Subsystem1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S335>/CANTransmit' incorporates:
   *  Constant: '<S335>/Constant'
   */
  ec_buffer_init(2,9,0,593U);

  /* End of Start for SubSystem: '<S330>/If Action Subsystem1' */
  /* End of Start for SubSystem: '<S329>/Enabled Subsystem' */
  ec_buffer_init(2,1,0,278);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S328>/CANReceive' */
  /* End of Start for S-Function (fcncallgen): '<S324>/100MS' */

  /* Start for S-Function (fcncallgen): '<S327>/Function-Call Generator' incorporates:
   *  SubSystem: '<S327>/CCPBackground'
   */
  /* Start for S-Function (ec5744_ccpslb): '<S342>/CCPBackground' */
  ccpInit();

  /* End of Start for S-Function (fcncallgen): '<S327>/Function-Call Generator' */

  /* Start for S-Function (ec5744_caninterruptslb1): '<S327>/ReceiveandTransmitInterrupt' incorporates:
   *  SubSystem: '<S327>/CCPReceive'
   */
  /* Start for function-call system: '<S327>/CCPReceive' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S343>/CANReceive' */
  ec_buffer_init(2,0,0,CCP_CRO_ID);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S343>/CANReceive' */
  ec_bufint_init(2,0);
  INTC_0.PSR[548].B.PRIN = 12;
  IntcIsrVectorTable[548] = (uint32_t)&ISR_FlexCAN_2_MB0;

  /* End of Start for S-Function (ec5744_caninterruptslb1): '<S327>/ReceiveandTransmitInterrupt' */

  /* Start for S-Function (ec5744_eeprombsbu3): '<Root>/EEPROMEnable' */
  Fls_Read(0xFA0010,ecflashdataold,4096);
  Fls_Read(0xFA0010,ecflashdatanew,4096);

  /* SystemInitialize for S-Function (fcncallgen): '<S3>/10ms7' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* SystemInitialize for SignalConversion generated from: '<S105>/Constant' */
  HVCUTOFF = true;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S3>/10ms7' */

  /* SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  /* InitializeConditions for UnitDelay: '<S246>/Unit Delay4' */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_mn = 0.01F;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' */

  /* SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* SystemInitialize for Chart: '<S91>/Chart2' */
  VehCtrlMdel240918_2018b_DW.sfEvent = -1;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S4>/10ms' incorporates:
   *  SubSystem: '<S4>/Function-Call Subsystem'
   */
  VehCtrlMdel240918_2018b_DW.FunctionCallSubsystem_RESET_ELA = true;

  /* End of Enable for S-Function (fcncallgen): '<S4>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  VehCtrlMdel240918_2018b_DW.Subsystem_RESET_ELAPS_T = true;

  /* End of Enable for S-Function (fcncallgen): '<S4>/10ms1' */

  /* Enable for S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* Enable for Chart: '<S91>/Chart2' */
  VehCtrlMdel240918_2018b_DW.previousTicks =
    VehCtrlMdel240918_2018b_M->Timing.clockTick3;

  /* End of Enable for S-Function (fcncallgen): '<S2>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S1>/10ms1' incorporates:
   *  SubSystem: '<S1>/MoTrqReq'
   */
  VehCtrlMdel240918_2018b_DW.MoTrqReq_RESET_ELAPS_T = true;

  /* End of Enable for S-Function (fcncallgen): '<S1>/10ms1' */
}

/* File trailer for ECUCoder generated file VehCtrlMdel240918_2018b.c.
 *
 * [EOF]
 */