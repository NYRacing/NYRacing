/*
 * Code generated for Simulink model VehCtrlMdel240918_2018b.
 *
 * FILE    : VehCtrlMdel240918_2018b.c
 *
 * VERSION : 1.169
 *
 * DATE    : Wed Sep 25 22:39:30 2024
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

/* Named constants for Chart: '<S114>/Timer' */
#define VehCtrlMdel240918_2018_IN_Out_n ((uint8_T)2U)
#define VehCtrlMdel240918__IN_Trigger_c ((uint8_T)3U)
#define VehCtrlMdel2409_IN_InterState_n ((uint8_T)1U)

/* Named constants for Chart: '<S7>/Chart' */
#define VehCtrlMdel240918_2018b_IN_B   ((uint8_T)1U)
#define VehCtrlMdel240918_2018b_IN_C   ((uint8_T)2U)
#define VehCtrlMdel240918_IN_DYC_Enable ((uint8_T)2U)
#define VehCtrlMdel240918_IN_InitState1 ((uint8_T)1U)
#define VehCtrlMdel240918_IN_InitState2 ((uint8_T)1U)
#define VehCtrlMdel240918__IN_InitState ((uint8_T)3U)
#define VehCtrlMdel240_IN_DYC_Disenable ((uint8_T)1U)
#define VehCtrlMdel24_IN_TCSF_Disenable ((uint8_T)2U)
#define VehCtrlMdel24_IN_TCSR_Disenable ((uint8_T)2U)
#define VehCtrlMdel2_IN_F_TVD_TCS_STATE ((uint8_T)3U)

/* Named constants for Chart: '<S97>/Chart2' */
#define VehCtrlM_IN_MC_InverterOn_State ((uint8_T)7U)
#define VehCtrlMdel240918_2018_IN_Guard ((uint8_T)1U)
#define VehCtrlMdel240918_2018_IN_Ready ((uint8_T)5U)
#define VehCtrlMdel240918_2018_IN_Trans ((uint8_T)7U)
#define VehCtrlMdel240918_2018_IN_start ((uint8_T)9U)
#define VehCtrlMdel240918_2018b_IN_Init ((uint8_T)2U)
#define VehCtrlMdel240918_2018b_IN_OFF ((uint8_T)1U)
#define VehCtrlMdel240918_2018b_IN_ON  ((uint8_T)2U)
#define VehCtrlMdel240918_201_IN_AMKCAN ((uint8_T)1U)
#define VehCtrlMdel240918_20_IN_Standby ((uint8_T)6U)
#define VehCtrlMdel240918_IN_MC_DCready ((uint8_T)6U)
#define VehCtrlMdel240918_IN_SYSRDYCECK ((uint8_T)8U)
#define VehCtrlMdel240918_event_AMKDCON (3)
#define VehCtrlMdel240918_event_EbeepON (5)
#define VehCtrlMdel24091_IN_MCDCOncheck ((uint8_T)5U)
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
real_T Gear_Trs;                       /* '<S328>/Switch2' */
real_T Mode_Trs;                       /* '<S328>/Switch3' */
real_T Trq_CUT;                        /* '<S193>/Timer' */
real_T ignition;                       /* '<S114>/Timer' */
real_T L12V_error;                     /* '<S171>/CAN Unpack' */
real_T alarm;                          /* '<S171>/CAN Unpack' */
real_T controller_ready;               /* '<S171>/CAN Unpack' */
real_T selfcheck;                      /* '<S171>/CAN Unpack' */
real_T RPM;                            /* '<S171>/CAN Unpack' */
real_T trq;                            /* '<S171>/CAN Unpack' */
real_T AC_current;                     /* '<S166>/CAN Unpack' */
real_T DC_current;                     /* '<S166>/CAN Unpack' */
real_T MCU_Temp;                       /* '<S166>/CAN Unpack' */
real_T motor_Temp;                     /* '<S166>/CAN Unpack' */
real_T MCFR_ActualVelocity;            /* '<S144>/CAN Unpack' */
real_T MCFR_DCVoltage;                 /* '<S144>/CAN Unpack' */
real_T MCFR_bDCOn;                     /* '<S144>/CAN Unpack' */
real_T MCFR_bError;                    /* '<S144>/CAN Unpack' */
real_T MCFR_bInverterOn;               /* '<S144>/CAN Unpack' */
real_T MCFR_bQuitInverterOn;           /* '<S144>/CAN Unpack' */
real_T MCFR_bSystemReady;              /* '<S144>/CAN Unpack' */
real_T MCFR_TempIGBT;                  /* '<S154>/CAN Unpack' */
real_T MCFR_TempInverter;              /* '<S154>/CAN Unpack' */
real_T MCFR_TempMotor;                 /* '<S154>/CAN Unpack' */
real_T MCFR_ErrorInfo;                 /* '<S152>/CAN Unpack' */
real_T MCFL_DCVoltage;                 /* '<S126>/CAN Unpack' */
real_T MCFL_bDCOn;                     /* '<S126>/CAN Unpack' */
real_T MCFL_bError;                    /* '<S126>/CAN Unpack' */
real_T MCFL_bInverterOn;               /* '<S126>/CAN Unpack' */
real_T MCFL_bQuitDCOn;                 /* '<S126>/CAN Unpack' */
real_T MCFL_bQuitInverterOn;           /* '<S126>/CAN Unpack' */
real_T MCFL_bSystemReady;              /* '<S126>/CAN Unpack' */
real_T MCFL_ActualVelocity;            /* '<S126>/Gain' */
real_T MCFL_TempIGBT;                  /* '<S137>/CAN Unpack' */
real_T MCFL_TempInverter;              /* '<S137>/CAN Unpack' */
real_T MCFL_TempMotor;                 /* '<S137>/CAN Unpack' */
real_T MCFL_ErrorInfo;                 /* '<S135>/CAN Unpack' */
real_T StrWhlAngAliveRollCnt;          /* '<S184>/CAN Unpack1' */
real_T StrWhlAng;                      /* '<S184>/CAN Unpack1' */
real_T StrWhlAngV;                     /* '<S184>/CAN Unpack1' */
real_T ABS_WS_FL;                      /* '<S116>/CAN Unpack1' */
real_T ABS_WS_FR;                      /* '<S116>/CAN Unpack1' */
real_T ABS_WS_RL;                      /* '<S116>/CAN Unpack1' */
real_T ABS_WS_RR;                      /* '<S116>/CAN Unpack1' */
real_T IMU_Ay_Value;                   /* '<S179>/CAN Unpack' */
real_T IMU_Ax_Value;                   /* '<S179>/CAN Unpack' */
real_T IMU_Yaw_Value;                  /* '<S179>/CAN Unpack' */
real_T EMRAX_Trq_CUT;                  /*  */
real_T AMK_Trq_CUT;                    /*  */
uint32_T Acc_vol2;                     /* '<S193>/Add3' */
uint32_T Acc_vol;                      /* '<S193>/Add2' */
uint32_T Acc_POS2;                     /* '<S193>/1-D Lookup Table3' */
real32_T Acc_POS;                      /* '<S193>/MATLAB Function' */
real32_T TrqR_cmd;                     /* '<S7>/Saturation1' */
real32_T TrqFR_cmd;                    /* '<S7>/Saturation2' */
real32_T TrqFL_cmd;                    /* '<S7>/Saturation3' */
uint16_T F_BrkPrs;                     /* '<S193>/1-D Lookup Table1' */
uint16_T Acc1;                         /* '<S109>/Acc3' */
uint16_T Acc2;                         /* '<S109>/Acc4' */
uint16_T Brk1;                         /* '<S109>/Brk1' */
uint16_T Brk2;                         /* '<S109>/Brk2' */
boolean_T HVCUTOFF;                    /* '<S114>/Constant' */
boolean_T KeyPressed;                  /* '<S97>/Cast To Boolean' */
boolean_T Brk;                         /* '<S99>/Compare' */
boolean_T ACC_Release;                 /* '<S100>/Compare' */
boolean_T beeper_state;                /* '<S97>/Chart2' */
boolean_T MCFL_DCOn_setpoints;         /* '<S97>/Chart2' */
boolean_T MCFR_DCEnable;               /* '<S97>/Chart2' */
boolean_T MCFR_InverterOn;             /* '<S97>/Chart2' */
boolean_T Trq_CUT_final;               /* '<S7>/Logical Operator4' */
boolean_T TroqueOn;                    /* '<S7>/Logical Operator6' */

/* Block signals (default storage) */
B_VehCtrlMdel240918_2018b_T VehCtrlMdel240918_2018b_B;

/* Block states (default storage) */
DW_VehCtrlMdel240918_2018b_T VehCtrlMdel240918_2018b_DW;

/* Real-time model */
RT_MODEL_VehCtrlMdel240918_20_T VehCtrlMdel240918_2018b_M_;
RT_MODEL_VehCtrlMdel240918_20_T *const VehCtrlMdel240918_2018b_M =
  &VehCtrlMdel240918_2018b_M_;

/* Forward declaration for local functions */
static void VehC_enter_atomic_WaitForEngine(int32_T *sfEvent);
static void VehCtrlMdel240918_2018b_VehStat(const real_T *controller_ready_e,
  int32_T *sfEvent);
static void VehCtrlMdel_enter_atomic_AMKCAN(int32_T *sfEvent);
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
  /* Call the system: <S336>/CCPReceive */
  {
    /* S-Function (ec5744_caninterruptslb1): '<S336>/ReceiveandTransmitInterrupt' */

    /* Output and update for function-call system: '<S336>/CCPReceive' */

    /* S-Function (ec5744_canreceiveslb): '<S352>/CANReceive' */

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

    /* Nothing to do for system: <S352>/Nothing */

    /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S352>/CANReceive' */

    /* End of Outputs for S-Function (ec5744_caninterruptslb1): '<S336>/ReceiveandTransmitInterrupt' */
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
 *    '<S195>/Timer'
 *    '<S196>/Timer'
 *    '<S196>/Timer1'
 *    '<S196>/Timer2'
 *    '<S196>/Timer3'
 *    '<S254>/Timer'
 *    '<S254>/Timer1'
 *    '<S254>/Timer2'
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
 *    '<S114>/Timer'
 *    '<S193>/Timer'
 */
void VehCtrlMdel240918_201_Timer(boolean_T rtu_Trigger, real32_T rtu_CountTime,
  real_T *rty_Exit, DW_Timer_VehCtrlMdel240918_20_T *localDW)
{
  boolean_T sf_internal_predicateOutput;

  /* Chart: '<S114>/Timer' */
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

  /* End of Chart: '<S114>/Timer' */
}

/* Function for Chart: '<S97>/Chart2' */
static void VehC_enter_atomic_WaitForEngine(int32_T *sfEvent)
{
  int32_T b_previousEvent;
  b_previousEvent = *sfEvent;
  *sfEvent = VehCtrlMdel2409_event_AMKCANOFF;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKCANenable != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (*sfEvent == VehCtrlMdel24091_event_AMKCANON) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (*sfEvent == VehCtrlMdel2409_event_AMKCANOFF) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable = 1U;
      }
      break;
    }
  }

  *sfEvent = b_previousEvent;
  *sfEvent = VehCtrlMdel24091_event_AMKDCOFF;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKDCon != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (*sfEvent == VehCtrlMdel240918_event_AMKDCON) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (*sfEvent == VehCtrlMdel24091_event_AMKDCOFF) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 1U;
      }
      break;
    }
  }

  *sfEvent = b_previousEvent;
  *sfEvent = VehCtrlMdel_event_MCDCEnableOFF;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MCDCEnable != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (*sfEvent == VehCtrlMdel2_event_MCDCEnableON) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (*sfEvent == VehCtrlMdel_event_MCDCEnableOFF) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 1U;
      }
      break;
    }
  }

  *sfEvent = b_previousEvent;
  *sfEvent = VehCtrlMdel24_event_InverterOFF;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_InverterOn != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (*sfEvent == VehCtrlMdel240_event_InverterON) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (*sfEvent == VehCtrlMdel24_event_InverterOFF) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 1U;
      }
      break;
    }
  }

  *sfEvent = b_previousEvent;
  *sfEvent = VehCtrlMdel2409_event_TorqueOFF;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_TorqueCUT != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (*sfEvent == VehCtrlMdel24091_event_TorqueON) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (*sfEvent == VehCtrlMdel2409_event_TorqueOFF) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
      }
      break;
    }
  }

  *sfEvent = b_previousEvent;
  VehCtrlMdel240918_2018b_B.errorReset = 0.0;
}

/* Function for Chart: '<S97>/Chart2' */
static void VehCtrlMdel240918_2018b_VehStat(const real_T *controller_ready_e,
  int32_T *sfEvent)
{
  boolean_T sf_internal_predicateOutput;
  int32_T b_previousEvent;
  switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat) {
   case VehCtrlMdel240918_2018_IN_Guard:
    if (VehCtrlMdel240918_2018b_DW.temporalCounter_i1 >= 25U) {
      b_previousEvent = *sfEvent;
      *sfEvent = VehCtrlMdel240918_event_EbeepON;
      if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_BeeperStat != 0U) {
        switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat) {
         case VehCtrlMdel240918_2018b_IN_OFF:
          if (*sfEvent == VehCtrlMdel240918_event_EbeepON) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat = 2U;
          }
          break;

         case VehCtrlMdel240918_2018b_IN_ON:
          if (*sfEvent == VehCtrlMdel24091_event_EbeepOFF) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat = 1U;
          }
          break;
        }
      }

      *sfEvent = b_previousEvent;
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 7U;
      VehCtrlMdel240918_2018b_DW.temporalCounter_i1 = 0U;
    } else {
      sf_internal_predicateOutput = ((!KeyPressed) || (!Brk) || (!ACC_Release));
      if (sf_internal_predicateOutput) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 6U;
      } else {
        sf_internal_predicateOutput = !(*controller_ready_e != 0.0);
        if (sf_internal_predicateOutput) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 3U;
        }
      }
    }
    break;

   case VehCtrlMdel240918_2018b_IN_Init:
    VehCtrlMdel240918_2018b_B.errorReset = 1.0;
    if (VehCtrlMdel240918_2018b_DW.temporalCounter_i1 >= 10U) {
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 8U;
      VehC_enter_atomic_WaitForEngine(sfEvent);
    }
    break;

   case VehCtrlMdel240_IN_InitStateBack:
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 8U;
    VehC_enter_atomic_WaitForEngine(sfEvent);
    break;

   case VehCtrlMdel2409_IN_MCUReadyFail:
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 8U;
    VehC_enter_atomic_WaitForEngine(sfEvent);
    break;

   case VehCtrlMdel240918_2018_IN_Ready:
    sf_internal_predicateOutput = !(*controller_ready_e != 0.0);
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 4U;
    }
    break;

   case VehCtrlMdel240918_20_IN_Standby:
    sf_internal_predicateOutput = !(*controller_ready_e != 0.0);
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 8U;
      VehC_enter_atomic_WaitForEngine(sfEvent);
    } else {
      sf_internal_predicateOutput = (KeyPressed && Brk && ACC_Release &&
        (*controller_ready_e != 0.0));
      if (sf_internal_predicateOutput) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 1U;
        VehCtrlMdel240918_2018b_DW.temporalCounter_i1 = 0U;
      }
    }
    break;

   case VehCtrlMdel240918_2018_IN_Trans:
    if (VehCtrlMdel240918_2018b_DW.temporalCounter_i1 >= 250U) {
      b_previousEvent = *sfEvent;
      *sfEvent = VehCtrlMdel24091_event_EbeepOFF;
      if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_BeeperStat != 0U) {
        switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat) {
         case VehCtrlMdel240918_2018b_IN_OFF:
          if (*sfEvent == VehCtrlMdel240918_event_EbeepON) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat = 2U;
          }
          break;

         case VehCtrlMdel240918_2018b_IN_ON:
          if (*sfEvent == VehCtrlMdel24091_event_EbeepOFF) {
            VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat = 1U;
          }
          break;
        }
      }

      *sfEvent = b_previousEvent;
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 5U;
    }
    break;

   case VehCtrlMdel240_IN_WaitForEngine:
    VehCtrlMdel240918_2018b_B.errorReset = 0.0;
    sf_internal_predicateOutput = (*controller_ready_e != 0.0);
    if (sf_internal_predicateOutput) {
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 6U;
    }
    break;
  }
}

/* Function for Chart: '<S97>/Chart2' */
static void VehCtrlMdel_enter_atomic_AMKCAN(int32_T *sfEvent)
{
  int32_T b_previousEvent;
  b_previousEvent = *sfEvent;
  *sfEvent = VehCtrlMdel24091_event_AMKCANON;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKCANenable != 0U) {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable) {
     case VehCtrlMdel240918_2018b_IN_OFF:
      if (*sfEvent == VehCtrlMdel24091_event_AMKCANON) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable = 2U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_ON:
      if (*sfEvent == VehCtrlMdel2409_event_AMKCANOFF) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable = 1U;
      }
      break;
    }
  }

  *sfEvent = b_previousEvent;
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
  /* S-Function (fcncallgen): '<S336>/Function-Call Generator' incorporates:
   *  SubSystem: '<S336>/CCPBackground'
   */

  /* S-Function (ec5744_ccpslb): '<S351>/CCPBackground' */
  ccpBackground();
  Lin0_Background();

  /* End of Outputs for S-Function (fcncallgen): '<S336>/Function-Call Generator' */
}

/* Model step function for TID2 */
void VehCtrlMdel240918_2018b_step2(void) /* Sample time: [0.005s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S334>/5ms' incorporates:
   *  SubSystem: '<S334>/daq5ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S349>/CCPDAQ' */
  ccpDaq(0);

  /* End of Outputs for S-Function (fcncallgen): '<S334>/5ms' */
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
  int32_T sfEvent;
  boolean_T rtb_LogicalOperator_idx_0;

  /* S-Function (fcncallgen): '<S3>/10ms7' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* SignalConversion generated from: '<S114>/Constant' */
  HVCUTOFF = true;

  /* S-Function (ec5744_swislbu3): '<S114>/SwitchInput' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240918_2018b_B.Drive_ready= ec_gpio_read(92);

  /* Logic: '<S114>/Logical Operator' */
  rtb_ignition = !VehCtrlMdel240918_2018b_B.Drive_ready;

  /* Chart: '<S114>/Timer' incorporates:
   *  Constant: '<S114>/Constant5'
   */
  VehCtrlMdel240918_201_Timer(rtb_ignition, 0.11F, &ignition,
    &VehCtrlMdel240918_2018b_DW.sf_Timer);

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms7' */

  /* S-Function (fcncallgen): '<S3>/10ms6' incorporates:
   *  SubSystem: '<S3>/EMRAXMCU_RECIEVE'
   */
  /* S-Function (ec5744_canreceiveslb): '<S111>/CANReceive1' */

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

  /* Call the system: <S111>/MCU_pwr */

  /* Output and update for function-call system: '<S111>/MCU_pwr' */

  /* Outputs for Enabled SubSystem: '<S164>/MCU_VCUMeter1' incorporates:
   *  EnablePort: '<S166>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive1_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S166>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S166>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S166>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S164>/MCU_VCUMeter1' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S111>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S111>/CANReceive3' */

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

  /* Call the system: <S111>/MCU_state */

  /* Output and update for function-call system: '<S111>/MCU_state' */

  /* Outputs for Enabled SubSystem: '<S165>/MCU_state' incorporates:
   *  EnablePort: '<S171>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive3_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S171>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S171>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S171>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S165>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S111>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms6' */

  /* S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S107>/CANReceive3' */

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

  /* Call the system: <S107>/ABS_BUS_state */

  /* Output and update for function-call system: '<S107>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S115>/IMU_state' incorporates:
   *  EnablePort: '<S116>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive3_o2_m > 0) {
    /* S-Function (ecucoder_canunmessage): '<S116>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S116>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S116>/CAN Unpack1' */
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

  /* End of Outputs for SubSystem: '<S115>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S107>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms3' */

  /* S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S113>/CANReceive3' */

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

  /* Call the system: <S113>/StrWhSnis_state */

  /* Output and update for function-call system: '<S113>/StrWhSnis_state' */

  /* Outputs for Enabled SubSystem: '<S183>/IMU_state' incorporates:
   *  EnablePort: '<S184>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive3_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S184>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S184>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S184>/CAN Unpack1' */
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

  /* End of Outputs for SubSystem: '<S183>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S113>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms4' */

  /* S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' */

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

  /* Call the system: <S121>/AMKMCU_state */

  /* Output and update for function-call system: '<S121>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S123>/MCU_state' incorporates:
   *  EnablePort: '<S126>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive3_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S126>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S126>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S126>/CAN Unpack' */
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

    /* Gain: '<S126>/Gain' */
    MCFL_ActualVelocity = -VehCtrlMdel240918_2018b_B.MCFL_ActualVelocity_p;
  }

  /* End of Outputs for SubSystem: '<S123>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S121>/CANReceive1' */

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

  /* Call the system: <S121>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S121>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S124>/MCU_state' incorporates:
   *  EnablePort: '<S135>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive1_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S135>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S135>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S135>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S124>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S121>/CANReceive2' */

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

  /* Call the system: <S121>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S121>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S125>/MCU_state' incorporates:
   *  EnablePort: '<S137>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive2_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S137>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S137>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S137>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S125>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive2' */

  /* S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' */

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

  /* Call the system: <S122>/AMKMCU_state */

  /* Output and update for function-call system: '<S122>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S141>/MCU_state' incorporates:
   *  EnablePort: '<S144>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive3_o2_a > 0) {
    /* S-Function (ecucoder_canunmessage): '<S144>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S144>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S144>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S141>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S122>/CANReceive1' */

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

  /* Call the system: <S122>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S122>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S142>/MCU_state' incorporates:
   *  EnablePort: '<S152>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive1_o2_o > 0) {
    /* S-Function (ecucoder_canunmessage): '<S152>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S152>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S152>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S142>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S122>/CANReceive2' */

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

  /* Call the system: <S122>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S122>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S143>/MCU_state' incorporates:
   *  EnablePort: '<S154>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive2_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S154>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S154>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S154>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S143>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive2' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms5' */

  /* S-Function (fcncallgen): '<S3>/10ms' incorporates:
   *  SubSystem: '<S3>/AccBrk_BUS'
   */
  /* S-Function (ec5744_asislbu3): '<S109>/Acc3' */

  /* Read the ADC conversion result of the analog signal */
  Acc1= adc_read_chan(1,2);

  /* S-Function (ec5744_asislbu3): '<S109>/Acc4' */

  /* Read the ADC conversion result of the analog signal */
  Acc2= adc_read_chan(1,4);

  /* S-Function (ec5744_asislbu3): '<S109>/Brk1' */

  /* Read the ADC conversion result of the analog signal */
  Brk1= adc_read_chan(1,0);

  /* S-Function (ec5744_asislbu3): '<S109>/Brk2' */

  /* Read the ADC conversion result of the analog signal */
  Brk2= adc_read_chan(0,13);

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms' */

  /* S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* S-Function (ec5744_canreceiveslb): '<S112>/CANReceive3' */

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

  /* Call the system: <S112>/IMU_state */

  /* Output and update for function-call system: '<S112>/IMU_state' */

  /* Outputs for Enabled SubSystem: '<S178>/MCU_state' incorporates:
   *  EnablePort: '<S179>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive3_o2_ma > 0) {
    /* S-Function (ecucoder_canunmessage): '<S179>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S179>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S179>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S178>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S112>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms2' */

  /* S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S110>/CANReceive3' */

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

  /* Call the system: <S110>/ABS_BUS_state */

  /* Output and update for function-call system: '<S110>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S162>/IMU_state' incorporates:
   *  EnablePort: '<S163>/Enable'
   */
  if (VehCtrlMdel240918_2018b_B.CANReceive3_o2_k > 0) {
    /* S-Function (ecucoder_canunmessage): '<S163>/CANUnPackMessage4' */

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

  /* End of Outputs for SubSystem: '<S162>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S110>/CANReceive3' */
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

  /* Lookup_n-D: '<S193>/1-D Lookup Table1' */
  F_BrkPrs = look1_iu16bflftfIu16_binlc(Brk1,
    VehCtrlMdel240918_2018b_ConstP.pooled65,
    VehCtrlMdel240918_2018b_ConstP.pooled65, 1U);

  /* DataTypeConversion: '<S193>/Data Type Conversion' */
  rtb_Add = F_BrkPrs;

  /* SignalConversion generated from: '<S191>/Out1' */
  Brk_F = (int32_T)rtb_Add;

  /* Gain: '<S193>/Gain2' */
  rtb_Gain1 = 45875U * Acc2;

  /* Gain: '<S193>/Gain3' incorporates:
   *  UnitDelay: '<S193>/Unit Delay1'
   */
  rtb_Gain = 39322U * VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S193>/Add3' */
  Acc_vol2 = (rtb_Gain >> 1) + rtb_Gain1;

  /* RelationalOperator: '<S201>/Compare' */
  rtb_ignition = (Acc_vol2 <= 32768000U);

  /* RelationalOperator: '<S202>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol2 >= 294912000U);

  /* Logic: '<S193>/Logical Operator1' */
  rtb_ignition = (rtb_ignition || rtb_LogicalOperator2);

  /* Gain: '<S193>/Gain' */
  rtb_Gain = 45875U * Acc1;

  /* UnitDelay: '<S193>/Unit Delay' incorporates:
   *  UnitDelay: '<S193>/Unit Delay1'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_fm =
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_k;

  /* Gain: '<S193>/Gain1' incorporates:
   *  UnitDelay: '<S193>/Unit Delay1'
   */
  rtb_Gain1 = 39322U * VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S193>/Add2' */
  Acc_vol = (rtb_Gain1 >> 1) + rtb_Gain;

  /* RelationalOperator: '<S197>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol <= 32768000U);

  /* RelationalOperator: '<S198>/Compare' */
  rtb_Compare = (Acc_vol >= 294912000U);

  /* Logic: '<S193>/Logical Operator' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_Compare);

  /* Logic: '<S193>/Logical Operator2' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_ignition);

  /* DataTypeConversion: '<S193>/Data Type Conversion2' */
  rtb_Add = (real32_T)Acc_vol * 1.52587891E-5F;

  /* MATLAB Function: '<S193>/MATLAB Function' */
  Acc_POS = fmaxf(fminf((rtb_Add - 3060.0F) * -0.4F, 100.0F), 0.0F);

  /* Lookup_n-D: '<S193>/1-D Lookup Table3' */
  Acc_POS2 = look1_iu32n16bflftfIu32_binlc(Acc_vol2,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable3_bp01Data,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable3_tableData, 1U);

  /* DataTypeConversion: '<S193>/Data Type Conversion4' */
  rtb_Add = (real32_T)Acc_POS2 * 1.52587891E-5F;

  /* Sum: '<S193>/Add1' */
  rtb_Acc_POS = Acc_POS - rtb_Add;

  /* Abs: '<S193>/Abs' */
  rtb_Acc_POS = fabsf(rtb_Acc_POS);

  /* RelationalOperator: '<S205>/Compare' incorporates:
   *  Constant: '<S205>/Constant'
   */
  rtb_Compare = (rtb_Acc_POS > 15.0F);

  /* RelationalOperator: '<S203>/Compare' incorporates:
   *  Constant: '<S203>/Constant'
   */
  rtb_ignition = (Acc_POS > 100.0F);

  /* RelationalOperator: '<S204>/Compare' incorporates:
   *  Constant: '<S204>/Constant'
   */
  rtb_Compare_am = (rtb_Add > 100.0F);

  /* Logic: '<S193>/Logical Operator3' */
  rtb_ignition = (rtb_ignition || rtb_Compare_am);

  /* RelationalOperator: '<S206>/Compare' incorporates:
   *  Constant: '<S206>/Constant'
   */
  rtb_Compare_am = (Brk1 <= 300);

  /* RelationalOperator: '<S207>/Compare' incorporates:
   *  Constant: '<S207>/Constant'
   */
  rtb_LowerRelop1_b = (Brk1 >= 4500);

  /* Logic: '<S193>/Logical Operator5' */
  rtb_Compare_am = (rtb_Compare_am || rtb_LowerRelop1_b);

  /* Logic: '<S193>/Logical Operator4' */
  rtb_ignition = (rtb_ignition || rtb_Compare || rtb_LogicalOperator2 ||
                  rtb_Compare_am);

  /* Chart: '<S193>/Timer' incorporates:
   *  Constant: '<S193>/Constant1'
   */
  VehCtrlMdel240918_201_Timer(rtb_ignition, 0.11F, &Trq_CUT,
    &VehCtrlMdel240918_2018b_DW.sf_Timer_a);

  /* UnitDelay: '<S228>/Delay Input2'
   *
   * Block description for '<S228>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE;

  /* SampleTimeMath: '<S228>/sample time'
   *
   * About '<S228>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S228>/delta rise limit' */
  rtb_Gain5 = 1200.0 * elapseTime;

  /* Sum: '<S228>/Difference Inputs1'
   *
   * Block description for '<S228>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = StrWhlAng - rtb_Yk1_l;

  /* RelationalOperator: '<S231>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain5);

  /* Switch: '<S231>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S228>/delta fall limit' */
    rtb_deltafalllimit_iz = -1200.0 * elapseTime;

    /* RelationalOperator: '<S231>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S231>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S231>/Switch' */
    rtb_Gain5 = rtb_UkYk1;
  }

  /* End of Switch: '<S231>/Switch2' */

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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE = rtb_Gain5 + rtb_Yk1_l;

  /* Abs: '<S195>/Abs' incorporates:
   *  UnitDelay: '<S228>/Delay Input2'
   *
   * Block description for '<S228>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain5 = fabs(VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE);

  /* RelationalOperator: '<S227>/Compare' incorporates:
   *  Constant: '<S227>/Constant'
   */
  rtb_ignition = (rtb_Gain5 > 120.0);

  /* Chart: '<S195>/Timer' incorporates:
   *  Constant: '<S195>/Constant5'
   */
  VehCtrlMdel240918_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_on, &VehCtrlMdel240918_2018b_DW.sf_Timer_k);

  /* UnitDelay: '<S241>/Delay Input2'
   *
   * Block description for '<S241>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain5 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_a;

  /* SampleTimeMath: '<S241>/sample time'
   *
   * About '<S241>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S241>/delta rise limit' */
  rtb_Yk1_l = 10.0 * elapseTime;

  /* Sum: '<S241>/Difference Inputs1'
   *
   * Block description for '<S241>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RL - rtb_Gain5;

  /* RelationalOperator: '<S249>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Yk1_l);

  /* Switch: '<S249>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S241>/delta fall limit' */
    rtb_Yk1_l = -10.0 * elapseTime;

    /* RelationalOperator: '<S249>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_Yk1_l);

    /* Switch: '<S249>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_Yk1_l;
    }

    /* End of Switch: '<S249>/Switch' */
    rtb_Yk1_l = rtb_UkYk1;
  }

  /* End of Switch: '<S249>/Switch2' */

  /* Saturate: '<S196>/Saturation' incorporates:
   *  Sum: '<S241>/Difference Inputs2'
   *  UnitDelay: '<S241>/Delay Input2'
   *
   * Block description for '<S241>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S241>/Delay Input2':
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

  /* End of Saturate: '<S196>/Saturation' */

  /* Gain: '<S196>/Gain' */
  rtb_Gain5 *= 0.27777777777777779;

  /* RelationalOperator: '<S233>/Compare' incorporates:
   *  Constant: '<S233>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain5 >= 0.0);

  /* RelationalOperator: '<S234>/Compare' incorporates:
   *  Constant: '<S234>/Constant'
   */
  rtb_Compare_am = (rtb_Gain5 < 40.0);

  /* Logic: '<S196>/OR' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S196>/Timer' incorporates:
   *  Constant: '<S196>/Constant5'
   */
  VehCtrlMdel240918_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_le, &VehCtrlMdel240918_2018b_DW.sf_Timer_b);

  /* UnitDelay: '<S242>/Delay Input2'
   *
   * Block description for '<S242>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_b;

  /* SampleTimeMath: '<S242>/sample time'
   *
   * About '<S242>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S242>/delta rise limit' */
  rtb_Gain4 = 10.0 * elapseTime;

  /* Sum: '<S242>/Difference Inputs1'
   *
   * Block description for '<S242>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RR - rtb_Yk1_l;

  /* RelationalOperator: '<S250>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain4);

  /* Switch: '<S250>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S242>/delta fall limit' */
    rtb_deltafalllimit_iz = -10.0 * elapseTime;

    /* RelationalOperator: '<S250>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S250>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S250>/Switch' */
    rtb_Gain4 = rtb_UkYk1;
  }

  /* End of Switch: '<S250>/Switch2' */

  /* Saturate: '<S196>/Saturation1' incorporates:
   *  Sum: '<S242>/Difference Inputs2'
   *  UnitDelay: '<S242>/Delay Input2'
   *
   * Block description for '<S242>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S242>/Delay Input2':
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

  /* End of Saturate: '<S196>/Saturation1' */

  /* Gain: '<S196>/Gain3' */
  rtb_Gain4 *= 0.27777777777777779;

  /* RelationalOperator: '<S235>/Compare' incorporates:
   *  Constant: '<S235>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain4 >= 0.0);

  /* RelationalOperator: '<S236>/Compare' incorporates:
   *  Constant: '<S236>/Constant'
   */
  rtb_Compare_am = (rtb_Gain4 < 40.0);

  /* Logic: '<S196>/OR1' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S196>/Timer1' incorporates:
   *  Constant: '<S196>/Constant1'
   */
  VehCtrlMdel240918_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_i, &VehCtrlMdel240918_2018b_DW.sf_Timer1_n);

  /* UnitDelay: '<S243>/Delay Input2'
   *
   * Block description for '<S243>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_h;

  /* SampleTimeMath: '<S243>/sample time'
   *
   * About '<S243>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S243>/delta rise limit' */
  rtb_Switch2_on = 10.0 * elapseTime;

  /* Sum: '<S243>/Difference Inputs1'
   *
   * Block description for '<S243>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = MCFR_ActualVelocity - rtb_Yk1_l;

  /* RelationalOperator: '<S251>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Switch2_on);

  /* Switch: '<S251>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S243>/delta fall limit' */
    rtb_deltafalllimit_iz = -10.0 * elapseTime;

    /* RelationalOperator: '<S251>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S251>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S251>/Switch' */
    rtb_Switch2_on = rtb_UkYk1;
  }

  /* End of Switch: '<S251>/Switch2' */

  /* Saturate: '<S196>/Saturation2' incorporates:
   *  Sum: '<S243>/Difference Inputs2'
   *  UnitDelay: '<S243>/Delay Input2'
   *
   * Block description for '<S243>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S243>/Delay Input2':
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

  /* End of Saturate: '<S196>/Saturation2' */

  /* Gain: '<S196>/Gain1' */
  rtb_Switch2_on *= 0.1341030088495575;

  /* RelationalOperator: '<S237>/Compare' incorporates:
   *  Constant: '<S237>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Switch2_on >= 0.0);

  /* RelationalOperator: '<S238>/Compare' incorporates:
   *  Constant: '<S238>/Constant'
   */
  rtb_Compare_am = (rtb_Switch2_on < 40.0);

  /* Logic: '<S196>/OR2' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S196>/Timer2' incorporates:
   *  Constant: '<S196>/Constant4'
   */
  VehCtrlMdel240918_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_o, &VehCtrlMdel240918_2018b_DW.sf_Timer2_l);

  /* UnitDelay: '<S244>/Delay Input2'
   *
   * Block description for '<S244>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n;

  /* SampleTimeMath: '<S244>/sample time'
   *
   * About '<S244>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S244>/delta rise limit' */
  rtb_UkYk1 = 10.0 * elapseTime;

  /* Sum: '<S244>/Difference Inputs1'
   *
   * Block description for '<S244>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_iz = MCFL_ActualVelocity - rtb_Yk1_l;

  /* RelationalOperator: '<S252>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_iz > rtb_UkYk1);

  /* Switch: '<S252>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S244>/delta fall limit' */
    rtb_UkYk1 = -10.0 * elapseTime;

    /* RelationalOperator: '<S252>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_iz < rtb_UkYk1);

    /* Switch: '<S252>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_iz = rtb_UkYk1;
    }

    /* End of Switch: '<S252>/Switch' */
    rtb_UkYk1 = rtb_deltafalllimit_iz;
  }

  /* End of Switch: '<S252>/Switch2' */

  /* Saturate: '<S196>/Saturation3' incorporates:
   *  Sum: '<S244>/Difference Inputs2'
   *  UnitDelay: '<S244>/Delay Input2'
   *
   * Block description for '<S244>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S244>/Delay Input2':
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

  /* End of Saturate: '<S196>/Saturation3' */

  /* Gain: '<S196>/Gain2' */
  rtb_UkYk1 *= 0.1341030088495575;

  /* RelationalOperator: '<S239>/Compare' incorporates:
   *  Constant: '<S239>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_UkYk1 >= 0.0);

  /* RelationalOperator: '<S240>/Compare' incorporates:
   *  Constant: '<S240>/Constant'
   */
  rtb_Compare_am = (rtb_UkYk1 < 40.0);

  /* Logic: '<S196>/OR3' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S196>/Timer3' incorporates:
   *  Constant: '<S196>/Constant8'
   */
  VehCtrlMdel240918_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_h, &VehCtrlMdel240918_2018b_DW.sf_Timer3);

  /* SignalConversion generated from: '<S191>/Out1' */
  WhlSpdFL = rtb_UkYk1;

  /* SignalConversion generated from: '<S191>/Out1' */
  WhlSpdFR = rtb_Switch2_on;

  /* SignalConversion generated from: '<S191>/Out1' */
  WhlSpdRR_mps = rtb_Gain4;

  /* SignalConversion generated from: '<S191>/Out1' */
  WhlSpdRL_mps = rtb_Gain5;

  /* Gain: '<S195>/Gain' incorporates:
   *  UnitDelay: '<S228>/Delay Input2'
   *
   * Block description for '<S228>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = 0.7 * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE;

  /* UnitDelay: '<S195>/Unit Delay' */
  rtb_Switch2_on = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE;

  /* Gain: '<S195>/Gain1' */
  rtb_Switch2_on *= 0.3;

  /* Sum: '<S195>/Add2' */
  rtb_UkYk1 += rtb_Switch2_on;

  /* Lookup_n-D: '<S195>/1-D Lookup Table' */
  rtb_Switch2_on = look1_binlx(rtb_UkYk1,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable_bp01Data,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable_tableData, 23U);

  /* SignalConversion generated from: '<S191>/Out1' */
  FLWhlStrAng = rtb_Switch2_on;

  /* Lookup_n-D: '<S195>/1-D Lookup Table1' */
  rtb_Switch2_on = look1_binlx(rtb_UkYk1,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable1_bp01Data,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable1_tableData, 23U);

  /* SignalConversion generated from: '<S191>/Out1' */
  rtb_Yk1_l = rtb_Switch2_on;

  /* SignalConversion generated from: '<S191>/Out1' */
  rtb_deltafalllimit_iz = rtb_UkYk1;

  /* Sum: '<S193>/Add' */
  rtb_Add += Acc_POS;

  /* Product: '<S193>/Divide' incorporates:
   *  Constant: '<S193>/Constant'
   */
  rtb_Acc_POS = (real32_T)(rtb_Add / 2.0);

  /* UnitDelay: '<S218>/Delay Input2'
   *
   * Block description for '<S218>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_l;

  /* SampleTimeMath: '<S218>/sample time'
   *
   * About '<S218>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S218>/delta rise limit' incorporates:
   *  Constant: '<S217>/Constant'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* UnitDelay: '<S217>/Unit Delay' */
  rtb_Gain4 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_g;

  /* Gain: '<S217>/Gain1' */
  rtb_Gain4 *= 0.3;

  /* Gain: '<S194>/g_mpss' incorporates:
   *  UnitDelay: '<S217>/Unit Delay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_g = 9.8 * IMU_Ay_Value;

  /* Gain: '<S217>/Gain' incorporates:
   *  UnitDelay: '<S217>/Unit Delay'
   */
  rtb_Gain5 = 0.7 * VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_g;

  /* Sum: '<S217>/Add2' */
  rtb_Add2 = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S218>/Difference Inputs1'
   *
   * Block description for '<S218>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add2 -= rtb_UkYk1;

  /* RelationalOperator: '<S224>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Add2 > rtb_Switch2_on);

  /* Switch: '<S224>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S218>/delta fall limit' incorporates:
     *  Constant: '<S217>/Constant1'
     */
    rtb_deltafalllimit_i4 = -5000.0 * elapseTime;

    /* RelationalOperator: '<S224>/UpperRelop' */
    rtb_ignition = (rtb_Add2 < rtb_deltafalllimit_i4);

    /* Switch: '<S224>/Switch' */
    if (rtb_ignition) {
      rtb_Add2 = rtb_deltafalllimit_i4;
    }

    /* End of Switch: '<S224>/Switch' */
    rtb_Switch2_on = rtb_Add2;
  }

  /* End of Switch: '<S224>/Switch2' */

  /* Sum: '<S218>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S218>/Delay Input2'
   *
   * Block description for '<S218>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S218>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_l = rtb_Switch2_on + rtb_UkYk1;

  /* RelationalOperator: '<S221>/LowerRelop1' incorporates:
   *  Constant: '<S217>/Constant6'
   *  UnitDelay: '<S218>/Delay Input2'
   *
   * Block description for '<S218>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_l > 1.5);

  /* Switch: '<S221>/Switch2' incorporates:
   *  Constant: '<S217>/Constant6'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S221>/UpperRelop' incorporates:
     *  Constant: '<S217>/Constant7'
     *  UnitDelay: '<S218>/Delay Input2'
     *
     * Block description for '<S218>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_ignition = (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_l < -1.5);

    /* Switch: '<S221>/Switch' incorporates:
     *  Constant: '<S217>/Constant7'
     *  UnitDelay: '<S218>/Delay Input2'
     *
     * Block description for '<S218>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_ignition) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_l;
    }

    /* End of Switch: '<S221>/Switch' */
  }

  /* End of Switch: '<S221>/Switch2' */

  /* SignalConversion generated from: '<S191>/Out1' */
  rtb_Add2 = rtb_UkYk1;

  /* UnitDelay: '<S219>/Delay Input2'
   *
   * Block description for '<S219>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_m;

  /* SampleTimeMath: '<S219>/sample time'
   *
   * About '<S219>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S219>/delta rise limit' incorporates:
   *  Constant: '<S217>/Constant2'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S194>/g_mpss1' */
  rtb_g_mpss1 = 9.8 * IMU_Ax_Value;

  /* Gain: '<S217>/Gain2' */
  rtb_Gain4 = 0.7 * rtb_g_mpss1;

  /* UnitDelay: '<S217>/Unit Delay1' */
  rtb_Gain5 = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE;

  /* Gain: '<S217>/Gain3' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S217>/Add1' */
  rtb_deltafalllimit_i4 = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S219>/Difference Inputs1'
   *
   * Block description for '<S219>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_i4 -= rtb_UkYk1;

  /* RelationalOperator: '<S225>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_i4 > rtb_Switch2_on);

  /* Switch: '<S225>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S219>/delta fall limit' incorporates:
     *  Constant: '<S217>/Constant4'
     */
    rtb_Switch2_on = -5000.0 * elapseTime;

    /* RelationalOperator: '<S225>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_i4 < rtb_Switch2_on);

    /* Switch: '<S225>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_i4 = rtb_Switch2_on;
    }

    /* End of Switch: '<S225>/Switch' */
    rtb_Switch2_on = rtb_deltafalllimit_i4;
  }

  /* End of Switch: '<S225>/Switch2' */

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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_m = rtb_Switch2_on + rtb_UkYk1;

  /* RelationalOperator: '<S222>/LowerRelop1' incorporates:
   *  Constant: '<S217>/Constant8'
   *  UnitDelay: '<S219>/Delay Input2'
   *
   * Block description for '<S219>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_m > 1.5);

  /* Switch: '<S222>/Switch2' incorporates:
   *  Constant: '<S217>/Constant8'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S222>/UpperRelop' incorporates:
     *  Constant: '<S217>/Constant9'
     *  UnitDelay: '<S219>/Delay Input2'
     *
     * Block description for '<S219>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_ignition = (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_m < -1.5);

    /* Switch: '<S222>/Switch' incorporates:
     *  Constant: '<S217>/Constant9'
     *  UnitDelay: '<S219>/Delay Input2'
     *
     * Block description for '<S219>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_ignition) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_m;
    }

    /* End of Switch: '<S222>/Switch' */
  }

  /* End of Switch: '<S222>/Switch2' */

  /* SignalConversion generated from: '<S191>/Out1' */
  rtb_deltafalllimit_i4 = rtb_UkYk1;

  /* SampleTimeMath: '<S229>/sample time'
   *
   * About '<S229>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S229>/delta rise limit' */
  rtb_UkYk1 = 1200.0 * elapseTime;

  /* UnitDelay: '<S229>/Delay Input2'
   *
   * Block description for '<S229>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_on = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_j;

  /* Sum: '<S229>/Difference Inputs1'
   *
   * Block description for '<S229>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1_ix = StrWhlAngV - rtb_Switch2_on;

  /* RelationalOperator: '<S232>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1_ix > rtb_UkYk1);

  /* Switch: '<S232>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S229>/delta fall limit' */
    rtb_UkYk1 = -1200.0 * elapseTime;

    /* RelationalOperator: '<S232>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1_ix < rtb_UkYk1);

    /* Switch: '<S232>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1_ix = rtb_UkYk1;
    }

    /* End of Switch: '<S232>/Switch' */
    rtb_UkYk1 = rtb_UkYk1_ix;
  }

  /* End of Switch: '<S232>/Switch2' */

  /* Saturate: '<S195>/Saturation1' incorporates:
   *  Sum: '<S229>/Difference Inputs2'
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_j = rtb_UkYk1 + rtb_Switch2_on;
  if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_j > 1200.0) {
    rtb_StrWhlAngV = 1200.0;
  } else if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_j < 0.0) {
    rtb_StrWhlAngV = 0.0;
  } else {
    rtb_StrWhlAngV = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_j;
  }

  /* End of Saturate: '<S195>/Saturation1' */

  /* Gain: '<S195>/Gain2' */
  rtb_UkYk1 = 0.7 * rtb_StrWhlAngV;

  /* UnitDelay: '<S195>/Unit Delay1' */
  rtb_Switch2_on = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_a;

  /* Gain: '<S195>/Gain3' */
  rtb_Switch2_on *= 0.3;

  /* Sum: '<S195>/Add1' */
  rtb_UkYk1 += rtb_Switch2_on;

  /* SignalConversion generated from: '<S191>/Out1' */
  rtb_UkYk1_ix = rtb_UkYk1;

  /* UnitDelay: '<S220>/Delay Input2'
   *
   * Block description for '<S220>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_k;

  /* SampleTimeMath: '<S220>/sample time'
   *
   * About '<S220>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S220>/delta rise limit' incorporates:
   *  Constant: '<S217>/Constant3'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S217>/Gain4' */
  rtb_Gain4 = 0.7 * IMU_Yaw_Value;

  /* UnitDelay: '<S217>/Unit Delay2' */
  rtb_Gain5 = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE;

  /* Gain: '<S217>/Gain5' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S217>/Add3' */
  rtb_Gain5 += rtb_Gain4;

  /* Sum: '<S220>/Difference Inputs1'
   *
   * Block description for '<S220>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 -= rtb_UkYk1;

  /* RelationalOperator: '<S226>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Gain5 > rtb_Switch2_on);

  /* Switch: '<S226>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S220>/delta fall limit' incorporates:
     *  Constant: '<S217>/Constant5'
     */
    rtb_Switch2_on = -5000.0 * elapseTime;

    /* RelationalOperator: '<S226>/UpperRelop' */
    rtb_ignition = (rtb_Gain5 < rtb_Switch2_on);

    /* Switch: '<S226>/Switch' */
    if (rtb_ignition) {
      rtb_Gain5 = rtb_Switch2_on;
    }

    /* End of Switch: '<S226>/Switch' */
    rtb_Switch2_on = rtb_Gain5;
  }

  /* End of Switch: '<S226>/Switch2' */

  /* Saturate: '<S217>/Saturation2' incorporates:
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_k = rtb_Switch2_on + rtb_UkYk1;
  if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_k > 180.0) {
    rtb_UkYk1 = 180.0;
  } else if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_k < -180.0) {
    rtb_UkYk1 = -180.0;
  } else {
    rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_k;
  }

  /* End of Saturate: '<S217>/Saturation2' */

  /* Update for UnitDelay: '<S193>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_fm = Acc2;

  /* Update for UnitDelay: '<S193>/Unit Delay' */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_k = Acc1;

  /* Update for UnitDelay: '<S195>/Unit Delay' incorporates:
   *  UnitDelay: '<S228>/Delay Input2'
   *
   * Block description for '<S228>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE;

  /* Update for UnitDelay: '<S217>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE = rtb_g_mpss1;

  /* Update for UnitDelay: '<S195>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_a = rtb_StrWhlAngV;

  /* Update for UnitDelay: '<S217>/Unit Delay2' */
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

  /* Gain: '<S192>/Gain5' */
  rtb_Switch2_on = 10.0 * VehCtrlMdel240918_2018b_B.CANUnpack_o1;

  /* DataTypeConversion: '<S192>/Cast To Double' */
  rtb_CastToDouble = (real32_T)rtb_Switch2_on;

  /* MinMax: '<S316>/Min3' incorporates:
   *  Gain: '<S270>/Gain'
   *  UnitDelay: '<S270>/Unit Delay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p *= 0.3F;

  /* UnitDelay: '<S274>/Delay Input2'
   *
   * Block description for '<S274>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_b0 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n2;

  /* SampleTimeMath: '<S274>/sample time'
   *
   * About '<S274>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S274>/delta rise limit' */
  rtb_Add4_j = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S192>/Cast To Double1' */
  rtb_Add7 = (real32_T)WhlSpdFL;

  /* DataTypeConversion: '<S192>/Cast To Double6' */
  rtb_Add6_p = (real32_T)FLWhlStrAng;

  /* Gain: '<S256>/Gain2' */
  rtb_Add6_p *= 0.0174532924F;

  /* Trigonometry: '<S256>/Asin' */
  rtb_Add6_p = cosf(rtb_Add6_p);

  /* Product: '<S256>/Product1' */
  rtb_Add7 *= rtb_Add6_p;

  /* DataTypeConversion: '<S192>/Cast To Double5' */
  rtb_Add6_p = (real32_T)rtb_UkYk1;

  /* Gain: '<S256>/Gain4' */
  rtb_Add6_p *= 0.0174532924F;

  /* Product: '<S256>/Product3' */
  rtb_Switch2_mn = 0.6F * rtb_Add6_p;

  /* Sum: '<S256>/Add2' */
  rtb_Add = rtb_Add7 - rtb_Switch2_mn;

  /* UnitDelay: '<S263>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_j;

  /* Sum: '<S263>/Add4' */
  rtb_Add7 = rtb_Add - rtb_Add7;

  /* Product: '<S263>/Divide' incorporates:
   *  Constant: '<S263>/steptime'
   */
  rtb_Divide = rtb_Add7 / 0.01F;

  /* RelationalOperator: '<S275>/LowerRelop1' incorporates:
   *  Constant: '<S270>/Constant1'
   */
  rtb_LogicalOperator2 = (rtb_Divide > 100.0F);

  /* Switch: '<S275>/Switch2' incorporates:
   *  Constant: '<S270>/Constant1'
   */
  if (rtb_LogicalOperator2) {
    rtb_Divide = 100.0F;
  } else {
    /* RelationalOperator: '<S275>/UpperRelop' incorporates:
     *  Constant: '<S270>/Constant'
     */
    rtb_ignition = (rtb_Divide < -100.0F);

    /* Switch: '<S275>/Switch' incorporates:
     *  Constant: '<S270>/Constant'
     */
    if (rtb_ignition) {
      rtb_Divide = -100.0F;
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
  rtb_Divide -= rtb_Switch2_b0;

  /* RelationalOperator: '<S276>/LowerRelop1' */
  rtb_LogicalOperator2 = (rtb_Divide > rtb_Add4_j);

  /* Switch: '<S276>/Switch2' */
  if (!rtb_LogicalOperator2) {
    /* Product: '<S274>/delta fall limit' */
    rtb_deltafalllimit_n = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S276>/UpperRelop' */
    rtb_ignition = (rtb_Divide < rtb_deltafalllimit_n);

    /* Switch: '<S276>/Switch' */
    if (rtb_ignition) {
      rtb_Divide = rtb_deltafalllimit_n;
    }

    /* End of Switch: '<S276>/Switch' */
    rtb_Add4_j = rtb_Divide;
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n2 = rtb_Add4_j + rtb_Switch2_b0;

  /* Gain: '<S270>/Gain1' incorporates:
   *  UnitDelay: '<S274>/Delay Input2'
   *
   * Block description for '<S274>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add7 = 0.7F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n2;

  /* MinMax: '<S316>/Min3' incorporates:
   *  Abs: '<S263>/Abs'
   *  Sum: '<S263>/Add'
   *  Sum: '<S270>/Add'
   *  UnitDelay: '<S270>/Unit Delay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p += rtb_Add7;
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p -= rtb_CastToDouble;
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p = fabsf
    (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p);

  /* RelationalOperator: '<S266>/Compare' incorporates:
   *  Constant: '<S266>/Constant'
   *  UnitDelay: '<S270>/Unit Delay'
   */
  rtb_LogicalOperator2 = (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p <= 0.5F);

  /* UnitDelay: '<S271>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_pj;

  /* Gain: '<S271>/Gain' */
  rtb_Add7 *= 0.3F;

  /* UnitDelay: '<S277>/Delay Input2'
   *
   * Block description for '<S277>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add4_j = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_e;

  /* SampleTimeMath: '<S277>/sample time'
   *
   * About '<S277>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S277>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(100.0 * elapseTime);

  /* MinMax: '<S316>/Min3' incorporates:
   *  DataTypeConversion: '<S192>/Cast To Double2'
   *  UnitDelay: '<S270>/Unit Delay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p = (real32_T)WhlSpdFR;

  /* DataTypeConversion: '<S192>/Cast To Double7' */
  rtb_Add10 = (real32_T)rtb_Yk1_l;

  /* Gain: '<S256>/Gain3' */
  rtb_Add10 *= 0.0174532924F;

  /* Trigonometry: '<S256>/Asin1' */
  rtb_Add10 = cosf(rtb_Add10);

  /* Product: '<S256>/Product2' incorporates:
   *  UnitDelay: '<S270>/Unit Delay'
   */
  rtb_Add10 *= VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p;

  /* Sum: '<S256>/Add3' */
  rtb_Divide = rtb_Switch2_mn + rtb_Add10;

  /* UnitDelay: '<S263>/Unit Delay1' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_n;

  /* Sum: '<S263>/Add5' */
  rtb_Add10 = rtb_Divide - rtb_Add10;

  /* Product: '<S263>/Divide1' incorporates:
   *  Constant: '<S263>/steptime1'
   */
  rtb_deltafalllimit_n = rtb_Add10 / 0.01F;

  /* RelationalOperator: '<S278>/LowerRelop1' incorporates:
   *  Constant: '<S271>/Constant1'
   */
  rtb_Compare = (rtb_deltafalllimit_n > 100.0F);

  /* Switch: '<S278>/Switch2' incorporates:
   *  Constant: '<S271>/Constant1'
   */
  if (rtb_Compare) {
    rtb_deltafalllimit_n = 100.0F;
  } else {
    /* RelationalOperator: '<S278>/UpperRelop' incorporates:
     *  Constant: '<S271>/Constant'
     */
    rtb_ignition = (rtb_deltafalllimit_n < -100.0F);

    /* Switch: '<S278>/Switch' incorporates:
     *  Constant: '<S271>/Constant'
     */
    if (rtb_ignition) {
      rtb_deltafalllimit_n = -100.0F;
    }

    /* End of Switch: '<S278>/Switch' */
  }

  /* End of Switch: '<S278>/Switch2' */

  /* Sum: '<S277>/Difference Inputs1'
   *
   * Block description for '<S277>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_n -= rtb_Add4_j;

  /* RelationalOperator: '<S279>/LowerRelop1' */
  rtb_Compare = (rtb_deltafalllimit_n > rtb_Switch2_b0);

  /* Switch: '<S279>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S277>/delta fall limit' */
    rtb_deltafalllimit_om = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S279>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_n < rtb_deltafalllimit_om);

    /* Switch: '<S279>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_n = rtb_deltafalllimit_om;
    }

    /* End of Switch: '<S279>/Switch' */
    rtb_Switch2_b0 = rtb_deltafalllimit_n;
  }

  /* End of Switch: '<S279>/Switch2' */

  /* Sum: '<S277>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S277>/Delay Input2'
   *
   * Block description for '<S277>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S277>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_e = rtb_Switch2_b0 + rtb_Add4_j;

  /* Gain: '<S271>/Gain1' incorporates:
   *  UnitDelay: '<S277>/Delay Input2'
   *
   * Block description for '<S277>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = 0.7F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_e;

  /* Sum: '<S271>/Add' */
  rtb_Add7 += rtb_Add10;

  /* Sum: '<S263>/Add1' */
  rtb_Add7 -= rtb_CastToDouble;

  /* Abs: '<S263>/Abs1' */
  rtb_Add7 = fabsf(rtb_Add7);

  /* RelationalOperator: '<S267>/Compare' incorporates:
   *  Constant: '<S267>/Constant'
   */
  rtb_Compare = (rtb_Add7 <= 0.5F);

  /* UnitDelay: '<S272>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_a;

  /* Gain: '<S272>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S280>/Delay Input2'
   *
   * Block description for '<S280>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hk3;

  /* SampleTimeMath: '<S280>/sample time'
   *
   * About '<S280>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S280>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S192>/Cast To Double3' */
  rtb_Add4_j = (real32_T)WhlSpdRL_mps;

  /* Product: '<S256>/Product' */
  rtb_Add6_p *= 0.58F;

  /* Sum: '<S256>/Add' */
  rtb_deltafalllimit_n = rtb_Add4_j - rtb_Add6_p;

  /* UnitDelay: '<S263>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_l;

  /* Sum: '<S263>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_n - rtb_Add4_j;

  /* Product: '<S263>/Divide2' incorporates:
   *  Constant: '<S263>/steptime2'
   */
  rtb_deltafalllimit_om = rtb_Add4_j / 0.01F;

  /* RelationalOperator: '<S281>/LowerRelop1' incorporates:
   *  Constant: '<S272>/Constant1'
   */
  rtb_Compare_am = (rtb_deltafalllimit_om > 100.0F);

  /* Switch: '<S281>/Switch2' incorporates:
   *  Constant: '<S272>/Constant1'
   */
  if (rtb_Compare_am) {
    rtb_deltafalllimit_om = 100.0F;
  } else {
    /* RelationalOperator: '<S281>/UpperRelop' incorporates:
     *  Constant: '<S272>/Constant'
     */
    rtb_ignition = (rtb_deltafalllimit_om < -100.0F);

    /* Switch: '<S281>/Switch' incorporates:
     *  Constant: '<S272>/Constant'
     */
    if (rtb_ignition) {
      rtb_deltafalllimit_om = -100.0F;
    }

    /* End of Switch: '<S281>/Switch' */
  }

  /* End of Switch: '<S281>/Switch2' */

  /* Sum: '<S280>/Difference Inputs1'
   *
   * Block description for '<S280>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_om -= rtb_Switch2_mn;

  /* RelationalOperator: '<S282>/LowerRelop1' */
  rtb_Compare_am = (rtb_deltafalllimit_om > rtb_Add7);

  /* Switch: '<S282>/Switch2' */
  if (!rtb_Compare_am) {
    /* Product: '<S280>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S282>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_om < rtb_Add7);

    /* Switch: '<S282>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_om = rtb_Add7;
    }

    /* End of Switch: '<S282>/Switch' */
    rtb_Add7 = rtb_deltafalllimit_om;
  }

  /* End of Switch: '<S282>/Switch2' */

  /* Sum: '<S280>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S280>/Delay Input2'
   *
   * Block description for '<S280>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S280>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hk3 = rtb_Add7 + rtb_Switch2_mn;

  /* Gain: '<S272>/Gain1' incorporates:
   *  UnitDelay: '<S280>/Delay Input2'
   *
   * Block description for '<S280>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hk3;

  /* Sum: '<S272>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S263>/Add2' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S263>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S268>/Compare' incorporates:
   *  Constant: '<S268>/Constant'
   */
  rtb_Compare_am = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S273>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_nc;

  /* Gain: '<S273>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S283>/Delay Input2'
   *
   * Block description for '<S283>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_c;

  /* SampleTimeMath: '<S283>/sample time'
   *
   * About '<S283>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S283>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S192>/Cast To Double4' */
  rtb_Add4_j = (real32_T)WhlSpdRR_mps;

  /* Sum: '<S256>/Add1' */
  rtb_deltafalllimit_om = rtb_Add6_p + rtb_Add4_j;

  /* UnitDelay: '<S263>/Unit Delay3' */
  rtb_Add6_p = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE;

  /* Sum: '<S263>/Add7' */
  rtb_Add6_p = rtb_deltafalllimit_om - rtb_Add6_p;

  /* Product: '<S263>/Divide3' incorporates:
   *  Constant: '<S263>/steptime3'
   */
  rtb_Add6_p /= 0.01F;

  /* RelationalOperator: '<S284>/LowerRelop1' incorporates:
   *  Constant: '<S273>/Constant1'
   */
  rtb_ignition = (rtb_Add6_p > 100.0F);

  /* Switch: '<S284>/Switch2' incorporates:
   *  Constant: '<S273>/Constant1'
   */
  if (rtb_ignition) {
    rtb_Add6_p = 100.0F;
  } else {
    /* RelationalOperator: '<S284>/UpperRelop' incorporates:
     *  Constant: '<S273>/Constant'
     */
    rtb_ignition = (rtb_Add6_p < -100.0F);

    /* Switch: '<S284>/Switch' incorporates:
     *  Constant: '<S273>/Constant'
     */
    if (rtb_ignition) {
      rtb_Add6_p = -100.0F;
    }

    /* End of Switch: '<S284>/Switch' */
  }

  /* End of Switch: '<S284>/Switch2' */

  /* Sum: '<S283>/Difference Inputs1'
   *
   * Block description for '<S283>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add6_p -= rtb_Switch2_mn;

  /* RelationalOperator: '<S285>/LowerRelop1' */
  rtb_ignition = (rtb_Add6_p > rtb_Add7);

  /* Switch: '<S285>/Switch2' */
  if (!rtb_ignition) {
    /* Product: '<S283>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S285>/UpperRelop' */
    rtb_ignition = (rtb_Add6_p < rtb_Add7);

    /* Switch: '<S285>/Switch' */
    if (rtb_ignition) {
      rtb_Add6_p = rtb_Add7;
    }

    /* End of Switch: '<S285>/Switch' */
    rtb_Add7 = rtb_Add6_p;
  }

  /* End of Switch: '<S285>/Switch2' */

  /* Sum: '<S283>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S283>/Delay Input2'
   *
   * Block description for '<S283>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S283>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_c = rtb_Add7 + rtb_Switch2_mn;

  /* Gain: '<S273>/Gain1' incorporates:
   *  UnitDelay: '<S283>/Delay Input2'
   *
   * Block description for '<S283>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_c;

  /* Sum: '<S273>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S263>/Add3' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S263>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S269>/Compare' incorporates:
   *  Constant: '<S269>/Constant'
   */
  rtb_ignition = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S290>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_l;

  /* Gain: '<S290>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S294>/Delay Input2'
   *
   * Block description for '<S294>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_i;

  /* SampleTimeMath: '<S294>/sample time'
   *
   * About '<S294>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S294>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S264>/Unit Delay4' */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_ik;

  /* UnitDelay: '<S264>/Unit Delay' */
  rtb_Add4_j = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_a0;

  /* Sum: '<S264>/Add4' */
  rtb_Add4_j = rtb_Add - rtb_Add4_j;

  /* Product: '<S264>/Divide' incorporates:
   *  Constant: '<S264>/steptime'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S264>/Add' */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_ik = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S264>/Add8' */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_ik - rtb_Add7;

  /* Product: '<S264>/Divide4' incorporates:
   *  Constant: '<S264>/steptime4'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S295>/LowerRelop1' incorporates:
   *  Constant: '<S290>/Constant1'
   */
  rtb_LowerRelop1_b = (rtb_Add7 > 100.0F);

  /* Switch: '<S295>/Switch2' incorporates:
   *  Constant: '<S290>/Constant1'
   */
  if (rtb_LowerRelop1_b) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S295>/UpperRelop' incorporates:
     *  Constant: '<S290>/Constant'
     */
    rtb_LowerRelop1_b = (rtb_Add7 < -100.0F);

    /* Switch: '<S295>/Switch' incorporates:
     *  Constant: '<S290>/Constant'
     */
    if (rtb_LowerRelop1_b) {
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
  rtb_LowerRelop1_b = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S296>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S294>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S296>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S296>/Switch' */
    if (rtb_LowerRelop1_b) {
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_i = rtb_Add6_p + rtb_Switch2_mn;

  /* Gain: '<S290>/Gain1' incorporates:
   *  UnitDelay: '<S294>/Delay Input2'
   *
   * Block description for '<S294>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_i;

  /* Sum: '<S290>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S264>/Abs' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S286>/Compare' incorporates:
   *  Constant: '<S286>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S291>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_ap;

  /* Gain: '<S291>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S297>/Delay Input2'
   *
   * Block description for '<S297>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_el;

  /* SampleTimeMath: '<S297>/sample time'
   *
   * About '<S297>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S297>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S264>/Unit Delay5' */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE;

  /* UnitDelay: '<S264>/Unit Delay1' */
  rtb_Add4_j = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_am;

  /* Sum: '<S264>/Add5' */
  rtb_Add4_j = rtb_Divide - rtb_Add4_j;

  /* Product: '<S264>/Divide1' incorporates:
   *  Constant: '<S264>/steptime1'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S264>/Add1' incorporates:
   *  UnitDelay: '<S264>/Unit Delay5'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE = rtb_Add4_j - rtb_CastToDouble;

  /* Sum: '<S264>/Add10' incorporates:
   *  UnitDelay: '<S264>/Unit Delay5'
   */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE - rtb_Add7;

  /* Product: '<S264>/Divide5' incorporates:
   *  Constant: '<S264>/steptime5'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S298>/LowerRelop1' incorporates:
   *  Constant: '<S291>/Constant1'
   */
  rtb_AND2 = (rtb_Add7 > 100.0F);

  /* Switch: '<S298>/Switch2' incorporates:
   *  Constant: '<S291>/Constant1'
   */
  if (rtb_AND2) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S298>/UpperRelop' incorporates:
     *  Constant: '<S291>/Constant'
     */
    rtb_AND2 = (rtb_Add7 < -100.0F);

    /* Switch: '<S298>/Switch' incorporates:
     *  Constant: '<S291>/Constant'
     */
    if (rtb_AND2) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S298>/Switch' */
  }

  /* End of Switch: '<S298>/Switch2' */

  /* Sum: '<S297>/Difference Inputs1'
   *
   * Block description for '<S297>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S299>/LowerRelop1' */
  rtb_AND2 = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S299>/Switch2' */
  if (!rtb_AND2) {
    /* Product: '<S297>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S299>/UpperRelop' */
    rtb_AND2 = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S299>/Switch' */
    if (rtb_AND2) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S299>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S299>/Switch2' */

  /* Sum: '<S297>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S297>/Delay Input2'
   *
   * Block description for '<S297>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S297>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_el = rtb_Add6_p + rtb_Switch2_mn;

  /* Gain: '<S291>/Gain1' incorporates:
   *  UnitDelay: '<S297>/Delay Input2'
   *
   * Block description for '<S297>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_el;

  /* Sum: '<S291>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S264>/Abs1' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S287>/Compare' incorporates:
   *  Constant: '<S287>/Constant'
   */
  rtb_AND2 = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S292>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_o;

  /* Gain: '<S292>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S300>/Delay Input2'
   *
   * Block description for '<S300>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_p;

  /* SampleTimeMath: '<S300>/sample time'
   *
   * About '<S300>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S300>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S264>/Unit Delay6' */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay6_DSTATE;

  /* UnitDelay: '<S264>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_c;

  /* Sum: '<S264>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_n - rtb_Add4_j;

  /* Product: '<S264>/Divide2' incorporates:
   *  Constant: '<S264>/steptime2'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S264>/Add2' incorporates:
   *  UnitDelay: '<S264>/Unit Delay6'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay6_DSTATE = rtb_Add4_j - rtb_CastToDouble;

  /* Sum: '<S264>/Add12' incorporates:
   *  UnitDelay: '<S264>/Unit Delay6'
   */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay6_DSTATE - rtb_Add7;

  /* Product: '<S264>/Divide6' incorporates:
   *  Constant: '<S264>/steptime6'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S301>/LowerRelop1' incorporates:
   *  Constant: '<S292>/Constant1'
   */
  rtb_Compare_b = (rtb_Add7 > 100.0F);

  /* Switch: '<S301>/Switch2' incorporates:
   *  Constant: '<S292>/Constant1'
   */
  if (rtb_Compare_b) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S301>/UpperRelop' incorporates:
     *  Constant: '<S292>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S301>/Switch' incorporates:
     *  Constant: '<S292>/Constant'
     */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S301>/Switch' */
  }

  /* End of Switch: '<S301>/Switch2' */

  /* Sum: '<S300>/Difference Inputs1'
   *
   * Block description for '<S300>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S302>/LowerRelop1' */
  rtb_Compare_b = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S302>/Switch2' */
  if (!rtb_Compare_b) {
    /* Product: '<S300>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S302>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S302>/Switch' */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S302>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S302>/Switch2' */

  /* Sum: '<S300>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S300>/Delay Input2'
   *
   * Block description for '<S300>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S300>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_p = rtb_Add6_p + rtb_Switch2_mn;

  /* Gain: '<S292>/Gain1' incorporates:
   *  UnitDelay: '<S300>/Delay Input2'
   *
   * Block description for '<S300>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_p;

  /* Sum: '<S292>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S264>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S288>/Compare' incorporates:
   *  Constant: '<S288>/Constant'
   */
  rtb_Compare_b = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S293>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_ah;

  /* Gain: '<S293>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S303>/Delay Input2'
   *
   * Block description for '<S303>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_mt;

  /* SampleTimeMath: '<S303>/sample time'
   *
   * About '<S303>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S303>/delta rise limit' */
  rtb_Add6_p = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S264>/Unit Delay7' */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay7_DSTATE;

  /* UnitDelay: '<S264>/Unit Delay3' */
  rtb_Add4_j = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_d;

  /* Sum: '<S264>/Add7' */
  rtb_Add4_j = rtb_deltafalllimit_om - rtb_Add4_j;

  /* Product: '<S264>/Divide3' incorporates:
   *  Constant: '<S264>/steptime3'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S264>/Add3' incorporates:
   *  UnitDelay: '<S264>/Unit Delay7'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay7_DSTATE = rtb_Add4_j - rtb_CastToDouble;

  /* Sum: '<S264>/Add14' incorporates:
   *  UnitDelay: '<S264>/Unit Delay7'
   */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.UnitDelay7_DSTATE - rtb_Add7;

  /* Product: '<S264>/Divide7' incorporates:
   *  Constant: '<S264>/steptime7'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S304>/LowerRelop1' incorporates:
   *  Constant: '<S293>/Constant1'
   */
  rtb_UpperRelop_ir = (rtb_Add7 > 100.0F);

  /* Switch: '<S304>/Switch2' incorporates:
   *  Constant: '<S293>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S304>/UpperRelop' incorporates:
     *  Constant: '<S293>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S304>/Switch' incorporates:
     *  Constant: '<S293>/Constant'
     */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S304>/Switch' */
  }

  /* End of Switch: '<S304>/Switch2' */

  /* Sum: '<S303>/Difference Inputs1'
   *
   * Block description for '<S303>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S305>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add7 > rtb_Add6_p);

  /* Switch: '<S305>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S303>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S305>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6_p);

    /* Switch: '<S305>/Switch' */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = rtb_Add6_p;
    }

    /* End of Switch: '<S305>/Switch' */
    rtb_Add6_p = rtb_Add7;
  }

  /* End of Switch: '<S305>/Switch2' */

  /* Sum: '<S303>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S303>/Delay Input2'
   *
   * Block description for '<S303>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S303>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_mt = rtb_Add6_p + rtb_Switch2_mn;

  /* Gain: '<S293>/Gain1' incorporates:
   *  UnitDelay: '<S303>/Delay Input2'
   *
   * Block description for '<S303>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_mt;

  /* Sum: '<S293>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S264>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S289>/Compare' incorporates:
   *  Constant: '<S289>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.8F);

  /* Logic: '<S254>/Logical Operator' */
  rtb_LogicalOperator_idx_0 = (rtb_LogicalOperator2 || rtb_LowerRelop1_b);
  rtb_Compare = (rtb_Compare || rtb_AND2);
  rtb_Compare_am = (rtb_Compare_am || rtb_Compare_b);
  rtb_LogicalOperator2 = (rtb_ignition || rtb_UpperRelop_ir);

  /* UnitDelay: '<S192>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_lh;

  /* Sum: '<S265>/Add' */
  rtb_Switch2_mn = rtb_Add - rtb_Add10;

  /* Abs: '<S265>/Abs' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S306>/Compare' incorporates:
   *  Constant: '<S306>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S265>/AND3' */
  rtb_Compare_b = (rtb_UpperRelop_ir && (VehCtrlMdel240918_2018b_B.Exit_h != 0.0));

  /* Sum: '<S265>/Add1' */
  rtb_Switch2_mn = rtb_Divide - rtb_Add10;

  /* Abs: '<S265>/Abs1' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S307>/Compare' incorporates:
   *  Constant: '<S307>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S265>/AND2' */
  rtb_AND2 = (rtb_UpperRelop_ir && (VehCtrlMdel240918_2018b_B.Exit_o != 0.0));

  /* Sum: '<S265>/Add2' */
  rtb_Switch2_mn = rtb_deltafalllimit_n - rtb_Add10;

  /* Abs: '<S265>/Abs2' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S308>/Compare' incorporates:
   *  Constant: '<S308>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S265>/AND' */
  rtb_LowerRelop1_b = (rtb_UpperRelop_ir && (VehCtrlMdel240918_2018b_B.Exit_le
    != 0.0));

  /* Sum: '<S265>/Add3' */
  rtb_Add10 = rtb_deltafalllimit_om - rtb_Add10;

  /* Abs: '<S265>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S309>/Compare' incorporates:
   *  Constant: '<S309>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 2.0F);

  /* Logic: '<S265>/AND1' */
  rtb_ignition = (rtb_UpperRelop_ir && (VehCtrlMdel240918_2018b_B.Exit_i != 0.0));

  /* Logic: '<S254>/Logical Operator1' */
  rtb_UpperRelop_ir = (rtb_Compare_b && rtb_LogicalOperator_idx_0);
  rtb_Compare = (rtb_AND2 && rtb_Compare);
  rtb_Compare_am = (rtb_LowerRelop1_b && rtb_Compare_am);
  rtb_ignition = (rtb_ignition && rtb_LogicalOperator2);

  /* Chart: '<S254>/Timer' incorporates:
   *  Constant: '<S254>/Constant1'
   */
  VehCtrlMdel240918_20_Timer1(rtb_UpperRelop_ir, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_c, &VehCtrlMdel240918_2018b_DW.sf_Timer_o);

  /* Chart: '<S254>/Timer1' incorporates:
   *  Constant: '<S254>/Constant2'
   */
  VehCtrlMdel240918_20_Timer1(rtb_Compare, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_lh4, &VehCtrlMdel240918_2018b_DW.sf_Timer1_m);

  /* Chart: '<S254>/Timer2' incorporates:
   *  Constant: '<S254>/Constant3'
   */
  VehCtrlMdel240918_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_lh, &VehCtrlMdel240918_2018b_DW.sf_Timer2_g);

  /* Chart: '<S254>/Timer3' incorporates:
   *  Constant: '<S254>/Constant4'
   */
  VehCtrlMdel240918_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_a, &VehCtrlMdel240918_2018b_DW.sf_Timer3_i);

  /* Logic: '<S253>/Logical Operator' */
  rtb_UpperRelop_ir = ((VehCtrlMdel240918_2018b_B.Exit_c != 0.0) ||
                       (VehCtrlMdel240918_2018b_B.Exit_lh4 != 0.0) ||
                       (VehCtrlMdel240918_2018b_B.Exit_lh != 0.0) ||
                       (VehCtrlMdel240918_2018b_B.Exit_a != 0.0));

  /* Logic: '<S253>/Logical Operator1' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* UnitDelay: '<S253>/Unit Delay4' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_m;

  /* Sum: '<S253>/Add1' */
  rtb_Add10 = rtb_Acc_POS - rtb_Add10;

  /* RelationalOperator: '<S257>/Compare' incorporates:
   *  Constant: '<S257>/Constant'
   */
  rtb_Compare_b = (rtb_Add10 > 0.1F);

  /* Logic: '<S253>/Logical Operator2' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir || rtb_Compare_b);

  /* Logic: '<S253>/AND' */
  rtb_ignition = ((VehCtrlMdel240918_2018b_B.CANUnpack_o1 != 0.0) &&
                  rtb_UpperRelop_ir);

  /* UnitDelay: '<S253>/Unit Delay3' */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_f;

  /* Logic: '<S253>/Logical Operator3' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S253>/Switch3' incorporates:
   *  UnitDelay: '<S253>/Unit Delay1'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S253>/Switch4' */
    if (!rtb_ignition) {
      /* Saturate: '<S27>/Saturation' incorporates:
       *  Constant: '<S253>/InitZORE'
       */
      VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k = 0.0F;
    }

    /* End of Switch: '<S253>/Switch4' */
    VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_o =
      VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Switch: '<S253>/Switch3' */

  /* UnitDelay: '<S255>/Unit Delay3' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_p;

  /* Sum: '<S255>/Add5' incorporates:
   *  UnitDelay: '<S255>/Unit Delay1'
   */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_d - rtb_Add10;

  /* Product: '<S255>/Divide3' incorporates:
   *  Constant: '<S255>/steptime3'
   */
  rtb_Add10 /= 0.01F;

  /* UnitDelay: '<S255>/Unit Delay2' */
  rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_f;

  /* Sum: '<S255>/Add9' */
  rtb_Switch2_mn -= rtb_Add10;

  /* UnitDelay: '<S255>/Unit Delay4' */
  rtb_Add6_p = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_mn;

  /* Sum: '<S255>/Add6' incorporates:
   *  Constant: '<S255>/steptime4'
   */
  rtb_Add6_p += 0.1F;

  /* Sum: '<S255>/Add8' incorporates:
   *  Constant: '<S255>/steptime6'
   */
  rtb_Add7 = rtb_Add6_p + 2.0F;

  /* Product: '<S255>/Divide5' */
  rtb_Add7 = 1.0F / rtb_Add7 * rtb_Add6_p;

  /* Logic: '<S255>/Logical Operator' */
  rtb_LogicalOperator2 = ((VehCtrlMdel240918_2018b_B.Exit_c != 0.0) ||
    (VehCtrlMdel240918_2018b_B.Exit_lh4 != 0.0) ||
    (VehCtrlMdel240918_2018b_B.Exit_lh != 0.0) ||
    (VehCtrlMdel240918_2018b_B.Exit_a != 0.0));

  /* Switch: '<S255>/Switch13' incorporates:
   *  Constant: '<S255>/Constant10'
   */
  if (rtb_LogicalOperator2) {
    rtb_Add4_j = rtb_Add7;
  } else {
    rtb_Add4_j = 1.0F;
  }

  /* End of Switch: '<S255>/Switch13' */

  /* Product: '<S255>/Divide6' */
  rtb_Switch2_mn *= rtb_Add4_j;

  /* Sum: '<S255>/Add10' */
  rtb_Ax = rtb_Switch2_mn + rtb_Add10;

  /* Switch: '<S253>/Switch1' */
  if (rtb_ignition) {
    /* Saturate: '<S253>/Saturation1' */
    if (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d > 200.0F) {
      VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d = 200.0F;
    } else {
      if (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d < -10.0F) {
        VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d = -10.0F;
      }
    }

    /* Product: '<S253>/Product' incorporates:
     *  Constant: '<S253>/steptime1'
     */
    rtb_Switch2_mn = rtb_Ax * 0.01F;

    /* Saturate: '<S253>/Saturation1' incorporates:
     *  Sum: '<S253>/Add'
     */
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d += rtb_Switch2_mn;
  } else {
    /* Saturate: '<S253>/Saturation1' incorporates:
     *  Constant: '<S253>/Constant'
     *  UnitDelay: '<S253>/Unit Delay'
     */
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d = 0.0F;
  }

  /* End of Switch: '<S253>/Switch1' */

  /* Saturate: '<S253>/Saturation' */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d > 200.0F) {
    rtb_Add10 = 200.0F;
  } else if (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d < -10.0F) {
    rtb_Add10 = -10.0F;
  } else {
    rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_d;
  }

  /* End of Saturate: '<S253>/Saturation' */

  /* Sum: '<S253>/Add3' incorporates:
   *  UnitDelay: '<S253>/Unit Delay1'
   */
  rtb_VxIMU_est = rtb_Add10 + VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_o;

  /* MinMax: '<S254>/Min1' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Add, rtb_Divide);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_n);
  rtb_Add10 = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_om);

  /* Sum: '<S253>/Add2' */
  rtb_Add10 -= rtb_VxIMU_est;

  /* RelationalOperator: '<S258>/Compare' incorporates:
   *  Constant: '<S258>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.0F);

  /* Switch: '<S253>/Switch6' incorporates:
   *  Constant: '<S253>/Reset'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S253>/Add10' incorporates:
     *  Constant: '<S253>/Steptime'
     */
    rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_i + 0.01F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S253>/Switch6' */

  /* MinMax: '<S253>/Min' incorporates:
   *  Constant: '<S253>/ResetDelay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_i = fminf(rtb_Add10, 0.1F);

  /* RelationalOperator: '<S253>/Relational Operator9' incorporates:
   *  Constant: '<S253>/ResetDelay'
   *  UnitDelay: '<S253>/Unit Delay2'
   */
  rtb_Compare = (VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_i >= 0.1F);

  /* RelationalOperator: '<S311>/Compare' incorporates:
   *  Constant: '<S311>/Constant'
   */
  rtb_Compare_am = (rtb_Ax < -0.5F);

  /* Chart: '<S255>/Timer2' incorporates:
   *  Constant: '<S255>/Constant15'
   */
  VehCtrlMdel240918_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit, &VehCtrlMdel240918_2018b_DW.sf_Timer2_j);

  /* UnitDelay: '<S315>/Delay Input2'
   *
   * Block description for '<S315>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_g;

  /* SampleTimeMath: '<S315>/sample time'
   *
   * About '<S315>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S315>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(10.0 * elapseTime);

  /* Sum: '<S316>/Add3' */
  rtb_Add4_j = ((rtb_deltafalllimit_om + rtb_deltafalllimit_n) + rtb_Divide) +
    rtb_Add;

  /* MinMax: '<S316>/Min4' */
  rtb_Switch2_b0 = fminf(rtb_Add, rtb_Divide);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_n);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_om);

  /* MinMax: '<S316>/Min3' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Add, rtb_Divide);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_n);
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p = fmaxf(rtb_MaxWhlSpd_mps_n,
    rtb_deltafalllimit_om);

  /* Sum: '<S316>/Add4' incorporates:
   *  UnitDelay: '<S270>/Unit Delay'
   */
  rtb_Add4_j = (rtb_Add4_j - rtb_Switch2_b0) -
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p;

  /* Gain: '<S316>/Gain1' */
  rtb_Add4_j *= 0.5F;

  /* Sum: '<S315>/Difference Inputs1'
   *
   * Block description for '<S315>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add4_j -= rtb_Add10;

  /* RelationalOperator: '<S324>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add4_j > rtb_Switch2_mn);

  /* Switch: '<S324>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S315>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-10.0 * elapseTime);

    /* RelationalOperator: '<S324>/UpperRelop' */
    rtb_Compare_am = (rtb_Add4_j < rtb_Switch2_mn);

    /* Switch: '<S324>/Switch' */
    if (rtb_Compare_am) {
      rtb_Add4_j = rtb_Switch2_mn;
    }

    /* End of Switch: '<S324>/Switch' */
    rtb_Switch2_mn = rtb_Add4_j;
  }

  /* End of Switch: '<S324>/Switch2' */

  /* Sum: '<S315>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S315>/Delay Input2'
   *
   * Block description for '<S315>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S315>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_g = rtb_Switch2_mn + rtb_Add10;

  /* RelationalOperator: '<S310>/Compare' incorporates:
   *  Constant: '<S310>/Constant'
   */
  rtb_Compare_am = (rtb_Ax > 0.5F);

  /* Chart: '<S255>/Timer1' incorporates:
   *  Constant: '<S255>/Constant14'
   */
  VehCtrlMdel240918_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_l, &VehCtrlMdel240918_2018b_DW.sf_Timer1_p);

  /* Logic: '<S255>/Logical Operator2' */
  rtb_UpperRelop_ir = !(VehCtrlMdel240918_2018b_B.Exit_l != 0.0);

  /* Switch: '<S255>/Switch6' incorporates:
   *  Switch: '<S255>/Switch4'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S255>/Switch5' incorporates:
     *  UnitDelay: '<S315>/Delay Input2'
     *
     * Block description for '<S315>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (VehCtrlMdel240918_2018b_B.Exit != 0.0) {
      /* Switch: '<S255>/Switch11' incorporates:
       *  Constant: '<S255>/Constant7'
       */
      if (VehCtrlMdel240918_2018b_B.Exit_a != 0.0) {
        rtb_Switch2_mn = rtb_deltafalllimit_om;
      } else {
        rtb_Switch2_mn = 0.0F;
      }

      /* End of Switch: '<S255>/Switch11' */

      /* Switch: '<S255>/Switch10' incorporates:
       *  Constant: '<S255>/Constant6'
       */
      if (VehCtrlMdel240918_2018b_B.Exit_lh != 0.0) {
        rtb_Add10 = rtb_deltafalllimit_n;
      } else {
        rtb_Add10 = 0.0F;
      }

      /* End of Switch: '<S255>/Switch10' */

      /* Switch: '<S255>/Switch9' incorporates:
       *  Constant: '<S255>/Constant5'
       */
      if (VehCtrlMdel240918_2018b_B.Exit_lh4 != 0.0) {
        rtb_Add4_j = rtb_Divide;
      } else {
        rtb_Add4_j = 0.0F;
      }

      /* End of Switch: '<S255>/Switch9' */

      /* Switch: '<S255>/Switch8' incorporates:
       *  Constant: '<S255>/Constant4'
       */
      if (VehCtrlMdel240918_2018b_B.Exit_c != 0.0) {
        rtb_Switch2_b0 = rtb_Add;
      } else {
        rtb_Switch2_b0 = 0.0F;
      }

      /* End of Switch: '<S255>/Switch8' */

      /* MinMax: '<S255>/Min1' */
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Switch2_b0, rtb_Add4_j);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Add10);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Switch2_mn);
    } else {
      rtb_MaxWhlSpd_mps_n = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_g;
    }

    /* End of Switch: '<S255>/Switch5' */
  } else {
    if (VehCtrlMdel240918_2018b_B.Exit_a != 0.0) {
      /* Switch: '<S255>/Switch4' */
      rtb_Switch2_mn = rtb_deltafalllimit_om;
    } else {
      /* Switch: '<S255>/Switch4' incorporates:
       *  Constant: '<S255>/Constant3'
       */
      rtb_Switch2_mn = 9999.0F;
    }

    /* Switch: '<S255>/Switch3' incorporates:
     *  Constant: '<S255>/Constant2'
     */
    if (VehCtrlMdel240918_2018b_B.Exit_lh != 0.0) {
      rtb_Add10 = rtb_deltafalllimit_n;
    } else {
      rtb_Add10 = 9999.0F;
    }

    /* End of Switch: '<S255>/Switch3' */

    /* Switch: '<S255>/Switch2' incorporates:
     *  Constant: '<S255>/Constant1'
     */
    if (VehCtrlMdel240918_2018b_B.Exit_lh4 != 0.0) {
      rtb_Add4_j = rtb_Divide;
    } else {
      rtb_Add4_j = 9999.0F;
    }

    /* End of Switch: '<S255>/Switch2' */

    /* Switch: '<S255>/Switch1' incorporates:
     *  Constant: '<S255>/Constant'
     */
    if (VehCtrlMdel240918_2018b_B.Exit_c != 0.0) {
      rtb_Switch2_b0 = rtb_Add;
    } else {
      rtb_Switch2_b0 = 9999.0F;
    }

    /* End of Switch: '<S255>/Switch1' */

    /* MinMax: '<S255>/Min2' */
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add4_j);
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add10);
    rtb_MaxWhlSpd_mps_n = fminf(rtb_Switch2_b0, rtb_Switch2_mn);
  }

  /* End of Switch: '<S255>/Switch6' */

  /* Logic: '<S255>/NOT3' */
  rtb_UpperRelop_ir = !rtb_LogicalOperator2;

  /* Logic: '<S255>/Logical Operator3' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir && rtb_Compare);

  /* Logic: '<S255>/NOT4' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S255>/Switch7' incorporates:
   *  UnitDelay: '<S315>/Delay Input2'
   *
   * Block description for '<S315>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (rtb_UpperRelop_ir) {
    /* Logic: '<S255>/Logical Operator1' */
    rtb_Compare = (rtb_Compare || rtb_LogicalOperator2);

    /* Switch: '<S255>/Switch' */
    if (rtb_Compare) {
      rtb_VxIMU_est = rtb_MaxWhlSpd_mps_n;
    }

    /* End of Switch: '<S255>/Switch' */
  } else {
    rtb_VxIMU_est = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_g;
  }

  /* End of Switch: '<S255>/Switch7' */

  /* UnitDelay: '<S313>/Delay Input2'
   *
   * Block description for '<S313>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_af;

  /* Sum: '<S313>/Difference Inputs1'
   *
   * Block description for '<S313>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_VxIMU_est -= rtb_Add10;

  /* Switch: '<S255>/Switch12' incorporates:
   *  Constant: '<S255>/Constant8'
   *  Constant: '<S255>/Constant9'
   */
  if (rtb_LogicalOperator2) {
    rtb_Switch2_mn = 0.1F;
  } else {
    rtb_Switch2_mn = 0.05F;
  }

  /* End of Switch: '<S255>/Switch12' */

  /* Sum: '<S255>/Add4' */
  rtb_Add4_j = rtb_Ax + rtb_Switch2_mn;

  /* SampleTimeMath: '<S313>/sample time'
   *
   * About '<S313>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S313>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S322>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_VxIMU_est > rtb_Switch2_b0);

  /* Sum: '<S255>/Add3' */
  rtb_Ax -= rtb_Switch2_mn;

  /* Switch: '<S322>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S313>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S322>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_VxIMU_est < rtb_Switch2_mn);

    /* Switch: '<S322>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_VxIMU_est = rtb_Switch2_mn;
    }

    /* End of Switch: '<S322>/Switch' */
    rtb_Switch2_b0 = rtb_VxIMU_est;
  }

  /* End of Switch: '<S322>/Switch2' */

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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_af = rtb_Switch2_b0 + rtb_Add10;

  /* RelationalOperator: '<S320>/LowerRelop1' incorporates:
   *  Constant: '<S312>/Constant1'
   *  UnitDelay: '<S313>/Delay Input2'
   *
   * Block description for '<S313>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_af > 100.0F);

  /* Switch: '<S320>/Switch2' incorporates:
   *  Constant: '<S312>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_mn = 100.0F;
  } else {
    /* RelationalOperator: '<S320>/UpperRelop' incorporates:
     *  Constant: '<S312>/Constant'
     *  UnitDelay: '<S313>/Delay Input2'
     *
     * Block description for '<S313>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_LogicalOperator2 = (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_af <
      0.0F);

    /* Switch: '<S320>/Switch' incorporates:
     *  Constant: '<S312>/Constant'
     *  UnitDelay: '<S313>/Delay Input2'
     *
     * Block description for '<S313>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_LogicalOperator2) {
      rtb_Switch2_mn = 0.0F;
    } else {
      rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_af;
    }

    /* End of Switch: '<S320>/Switch' */
  }

  /* End of Switch: '<S320>/Switch2' */

  /* UnitDelay: '<S319>/Delay Input2'
   *
   * Block description for '<S319>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_f;

  /* Sum: '<S319>/Difference Inputs1'
   *
   * Block description for '<S319>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_Switch2_mn - rtb_Add10;

  /* SampleTimeMath: '<S319>/sample time'
   *
   * About '<S319>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S319>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(15.0 * elapseTime);

  /* RelationalOperator: '<S321>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S321>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S319>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-15.0 * elapseTime);

    /* RelationalOperator: '<S321>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S321>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S321>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S321>/Switch2' */

  /* Sum: '<S319>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S319>/Delay Input2'
   *
   * Block description for '<S319>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S319>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_f = rtb_Switch2_mn + rtb_Add10;

  /* UnitDelay: '<S312>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_ncs;

  /* Gain: '<S312>/Gain' */
  rtb_Add10 *= 0.0F;

  /* Saturate: '<S27>/Saturation' incorporates:
   *  Sum: '<S312>/Add'
   *  UnitDelay: '<S192>/Unit Delay1'
   *  UnitDelay: '<S319>/Delay Input2'
   *
   * Block description for '<S319>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k = rtb_Add10 +
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_f;

  /* SampleTimeMath: '<S314>/sample time'
   *
   * About '<S314>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* UnitDelay: '<S314>/Delay Input2'
   *
   * Block description for '<S314>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hu;

  /* Sum: '<S314>/Difference Inputs1'
   *
   * Block description for '<S314>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_MaxWhlSpd_mps_n - rtb_Add10;

  /* Product: '<S314>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S323>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S323>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S314>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S323>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S323>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S323>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S323>/Switch2' */

  /* Sum: '<S314>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S314>/Delay Input2'
   *
   * Block description for '<S314>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S314>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hu = rtb_Switch2_mn + rtb_Add10;

  /* Sum: '<S255>/Add7' incorporates:
   *  Constant: '<S255>/steptime5'
   */
  rtb_Add7 = 1.0F - rtb_Add7;

  /* Product: '<S255>/Divide4' incorporates:
   *  UnitDelay: '<S255>/Unit Delay4'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_mn = rtb_Add7 * rtb_Add6_p;

  /* Update for MinMax: '<S316>/Min3' incorporates:
   *  UnitDelay: '<S270>/Unit Delay'
   *  UnitDelay: '<S274>/Delay Input2'
   *
   * Block description for '<S274>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_p =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n2;

  /* Update for UnitDelay: '<S263>/Unit Delay' */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_j = rtb_Add;

  /* Update for UnitDelay: '<S271>/Unit Delay' incorporates:
   *  UnitDelay: '<S277>/Delay Input2'
   *
   * Block description for '<S277>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_pj =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_e;

  /* Update for UnitDelay: '<S263>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_n = rtb_Divide;

  /* Update for UnitDelay: '<S272>/Unit Delay' incorporates:
   *  UnitDelay: '<S280>/Delay Input2'
   *
   * Block description for '<S280>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_a =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hk3;

  /* Update for UnitDelay: '<S263>/Unit Delay2' */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_l = rtb_deltafalllimit_n;

  /* Update for UnitDelay: '<S273>/Unit Delay' incorporates:
   *  UnitDelay: '<S283>/Delay Input2'
   *
   * Block description for '<S283>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_nc =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_c;

  /* Update for UnitDelay: '<S263>/Unit Delay3' */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE = rtb_deltafalllimit_om;

  /* Update for UnitDelay: '<S290>/Unit Delay' incorporates:
   *  UnitDelay: '<S294>/Delay Input2'
   *
   * Block description for '<S294>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_l =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_i;

  /* Update for UnitDelay: '<S264>/Unit Delay' */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_a0 = rtb_Add;

  /* Update for UnitDelay: '<S291>/Unit Delay' incorporates:
   *  UnitDelay: '<S297>/Delay Input2'
   *
   * Block description for '<S297>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_ap =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_el;

  /* Update for UnitDelay: '<S264>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_am = rtb_Divide;

  /* Update for UnitDelay: '<S292>/Unit Delay' incorporates:
   *  UnitDelay: '<S300>/Delay Input2'
   *
   * Block description for '<S300>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_o =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_p;

  /* Update for UnitDelay: '<S264>/Unit Delay2' */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_c = rtb_deltafalllimit_n;

  /* Update for UnitDelay: '<S293>/Unit Delay' incorporates:
   *  UnitDelay: '<S303>/Delay Input2'
   *
   * Block description for '<S303>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_ah =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_mt;

  /* Update for UnitDelay: '<S264>/Unit Delay3' */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_d = rtb_deltafalllimit_om;

  /* Update for UnitDelay: '<S192>/Unit Delay' incorporates:
   *  UnitDelay: '<S192>/Unit Delay1'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_lh =
    VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k;

  /* Update for UnitDelay: '<S253>/Unit Delay4' */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_m = rtb_Acc_POS;

  /* Update for UnitDelay: '<S253>/Unit Delay3' */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_f = rtb_ignition;

  /* Update for UnitDelay: '<S255>/Unit Delay3' incorporates:
   *  UnitDelay: '<S255>/Unit Delay1'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_p =
    VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_d;

  /* Update for UnitDelay: '<S255>/Unit Delay1' incorporates:
   *  UnitDelay: '<S314>/Delay Input2'
   *
   * Block description for '<S314>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_d =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hu;

  /* Update for UnitDelay: '<S255>/Unit Delay2' */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_f = rtb_CastToDouble;

  /* Update for UnitDelay: '<S312>/Unit Delay' incorporates:
   *  UnitDelay: '<S319>/Delay Input2'
   *
   * Block description for '<S319>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_ncs =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_f;

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms1' */

  /* S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* DataTypeConversion: '<S97>/Cast To Boolean' */
  KeyPressed = (ignition != 0.0);

  /* RelationalOperator: '<S99>/Compare' incorporates:
   *  Constant: '<S99>/Constant'
   */
  Brk = (Brk_F >= 450);

  /* RelationalOperator: '<S100>/Compare' incorporates:
   *  Constant: '<S100>/Constant'
   */
  ACC_Release = (rtb_Acc_POS <= 50.0F);

  /* Chart: '<S97>/Chart2' */
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

  sfEvent = -1;
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_c1_VehCtrlMdel240918_ ==
      0U) {
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_c1_VehCtrlMdel240918_ = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_VehStat = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_VehStat = 2U;
    VehCtrlMdel240918_2018b_DW.temporalCounter_i1 = 0U;
    VehCtrlMdel240918_2018b_B.errorReset = 1.0;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_BeeperStat = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKDCon = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MCDCEnable = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_TorqueCUT = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKDCready = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 9U;
    VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = 0U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_Output = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKCANenable = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_InverterOn = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 1U;
  } else {
    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_VehStat != 0U) {
      VehCtrlMdel240918_2018b_VehStat(&controller_ready, &sfEvent);
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_BeeperStat != 0U) {
      switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat) {
       case VehCtrlMdel240918_2018b_IN_OFF:
        if (sfEvent == VehCtrlMdel240918_event_EbeepON) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat = 2U;
        }
        break;

       case VehCtrlMdel240918_2018b_IN_ON:
        if (sfEvent == VehCtrlMdel24091_event_EbeepOFF) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_BeeperStat = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKDCon != 0U) {
      switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon) {
       case VehCtrlMdel240918_2018b_IN_OFF:
        if (sfEvent == VehCtrlMdel240918_event_AMKDCON) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 2U;
        }
        break;

       case VehCtrlMdel240918_2018b_IN_ON:
        if (sfEvent == VehCtrlMdel24091_event_AMKDCOFF) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MCDCEnable != 0U) {
      switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable) {
       case VehCtrlMdel240918_2018b_IN_OFF:
        if (sfEvent == VehCtrlMdel2_event_MCDCEnableON) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 2U;
        }
        break;

       case VehCtrlMdel240918_2018b_IN_ON:
        if (sfEvent == VehCtrlMdel_event_MCDCEnableOFF) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_TorqueCUT != 0U) {
      switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT) {
       case VehCtrlMdel240918_2018b_IN_OFF:
        if (sfEvent == VehCtrlMdel24091_event_TorqueON) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 2U;
        }
        break;

       case VehCtrlMdel240918_2018b_IN_ON:
        if (sfEvent == VehCtrlMdel2409_event_TorqueOFF) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKDCready != 0U) {
      switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready) {
       case VehCtrlMdel240918_201_IN_AMKCAN:
        rtb_LogicalOperator2 = (KeyPressed && Brk && ACC_Release);
        if (rtb_LogicalOperator2) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 6U;
          VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = 0U;
        }
        break;

       case VehCtrlMdel240_IN_AMKDCOnFinish:
        if (KeyPressed) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 1U;
          VehCtrlMdel_enter_atomic_AMKCAN(&sfEvent);
        }
        break;

       case VehCtrlMdel240_IN_DCOnCheckPass:
        rtb_ignition = ((VehCtrlMdel240918_2018b_DW.temporalCounter_i2 >= 100U) &&
                        (MCFL_bDCOn != 0.0) && (MCFR_bDCOn != 0.0));
        if (rtb_ignition) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 4U;
          VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = 0U;
          if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MCDCEnable != 0U)
          {
            switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable) {
             case VehCtrlMdel240918_2018b_IN_OFF:
              VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable = 2U;
              break;

             case VehCtrlMdel240918_2018b_IN_ON:
              break;
            }
          }
        }
        break;

       case VehCtrlMdel2_IN_MCDCEnableState:
        if (VehCtrlMdel240918_2018b_DW.temporalCounter_i2 >= 100U) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 7U;
          VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = 0U;
          if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_InverterOn !=
              0U) {
            switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn) {
             case VehCtrlMdel240918_2018b_IN_OFF:
              VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 2U;
              break;

             case VehCtrlMdel240918_2018b_IN_ON:
              break;
            }
          }
        }
        break;

       case VehCtrlMdel24091_IN_MCDCOncheck:
        if (VehCtrlMdel240918_2018b_DW.temporalCounter_i2 >= 100U) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 3U;
          VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = 0U;
        }
        break;

       case VehCtrlMdel240918_IN_MC_DCready:
        if (VehCtrlMdel240918_2018b_DW.temporalCounter_i2 >= 100U) {
          if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKDCon != 0U) {
            switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon) {
             case VehCtrlMdel240918_2018b_IN_OFF:
              VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon = 2U;
              break;

             case VehCtrlMdel240918_2018b_IN_ON:
              break;
            }
          }

          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 5U;
          VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = 0U;
        }
        break;

       case VehCtrlM_IN_MC_InverterOn_State:
        if (VehCtrlMdel240918_2018b_DW.temporalCounter_i2 >= 100U) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 8U;
          VehCtrlMdel240918_2018b_DW.temporalCounter_i2 = 0U;
        }
        break;

       case VehCtrlMdel240918_IN_SYSRDYCECK:
        rtb_LogicalOperator2 = (VehCtrlMdel240918_2018b_DW.temporalCounter_i2 >=
          1000U);
        if (rtb_LogicalOperator2) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 2U;
          if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_TorqueCUT !=
              0U) {
            switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT) {
             case VehCtrlMdel240918_2018b_IN_OFF:
              VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT = 2U;
              break;

             case VehCtrlMdel240918_2018b_IN_ON:
              break;
            }
          }
        }
        break;

       case VehCtrlMdel240918_2018_IN_start:
        if (VehCtrlMdel240918_2018b_DW.temporalCounter_i2 >= 500U) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCready = 1U;
          VehCtrlMdel_enter_atomic_AMKCAN(&sfEvent);
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
      MCFL_DCOn_setpoints = (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKDCon ==
        VehCtrlMdel240918_2018b_IN_ON);
      VehCtrlMdel240918_2018b_B.MCFL_DCEnable =
        (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable ==
         VehCtrlMdel240918_2018b_IN_ON);
      MCFR_DCEnable = (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MCDCEnable ==
                       VehCtrlMdel240918_2018b_IN_ON);
      VehCtrlMdel240918_2018b_B.MCFL_InverterOn =
        (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn ==
         VehCtrlMdel240918_2018b_IN_ON);
      MCFR_InverterOn = (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn
                         == VehCtrlMdel240918_2018b_IN_ON);
      VehCtrlMdel240918_2018b_B.MCFL_TorqueOn =
        (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT ==
         VehCtrlMdel240918_2018b_IN_ON);
      VehCtrlMdel240918_2018b_B.MCFR_TorqueOn =
        (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_TorqueCUT ==
         VehCtrlMdel240918_2018b_IN_ON);
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_AMKCANenable != 0U) {
      switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable) {
       case VehCtrlMdel240918_2018b_IN_OFF:
        if (sfEvent == VehCtrlMdel24091_event_AMKCANON) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable = 2U;
        }
        break;

       case VehCtrlMdel240918_2018b_IN_ON:
        if (sfEvent == VehCtrlMdel2409_event_AMKCANOFF) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_AMKCANenable = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_MC_InverterOn != 0U) {
      switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn) {
       case VehCtrlMdel240918_2018b_IN_OFF:
        if (sfEvent == VehCtrlMdel240_event_InverterON) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 2U;
        }
        break;

       case VehCtrlMdel240918_2018b_IN_ON:
        if (sfEvent == VehCtrlMdel24_event_InverterOFF) {
          VehCtrlMdel240918_2018b_DW.bitsForTID3.is_MC_InverterOn = 1U;
        }
        break;
      }
    }
  }

  /* End of Chart: '<S97>/Chart2' */
  /* End of Outputs for S-Function (fcncallgen): '<S2>/10ms' */

  /* S-Function (fcncallgen): '<S5>/10ms1' incorporates:
   *  SubSystem: '<S5>/Beeper'
   */
  /* S-Function (ec5744_pdsslbu3): '<S325>/PowerDriverSwitch(HS)' */

  /* Set level beeper_state for the specified power driver switch */
  ec_gpio_write(83,beeper_state);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms1' */

  /* S-Function (fcncallgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/PwrTrainTempPrtct'
   */
  /* MinMax: '<S8>/Max' */
  rtb_Switch2_on = fmax(MCFL_TempInverter, MCFR_TempInverter);

  /* RelationalOperator: '<S90>/Compare' incorporates:
   *  Constant: '<S90>/Constant'
   */
  rtb_ignition = (rtb_Switch2_on > 40.0);

  /* RelationalOperator: '<S91>/Compare' incorporates:
   *  Constant: '<S91>/Constant'
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
    WhlSpdFL = look1_binlx(rtb_Switch2_on,
      VehCtrlMdel240918_2018b_ConstP.pooled18,
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

  /* RelationalOperator: '<S93>/Compare' incorporates:
   *  Constant: '<S93>/Constant'
   */
  rtb_ignition = (MCU_Temp > 50.0);

  /* Chart: '<S8>/Timer2' incorporates:
   *  Constant: '<S8>/Constant3'
   */
  VehCtrlMdel240918_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240918_2018b_B.Exit_d, &VehCtrlMdel240918_2018b_DW.sf_Timer2);

  /* SignalConversion generated from: '<S8>/Out1' */
  EMRAX_Trq_CUT = VehCtrlMdel240918_2018b_B.Exit_d;

  /* Logic: '<S8>/NOT1' */
  rtb_Compare = !rtb_ignition;

  /* RelationalOperator: '<S92>/Compare' incorporates:
   *  Constant: '<S92>/Constant'
   */
  rtb_ignition = (MCU_Temp > 45.0);

  /* Logic: '<S8>/AND1' */
  rtb_ignition = (rtb_ignition && rtb_Compare);

  /* MinMax: '<S8>/Max1' */
  elapseTime = fmax(MCU_Temp, motor_Temp);

  /* Switch: '<S8>/Switch1' incorporates:
   *  Constant: '<S8>/Constant1'
   */
  if (rtb_ignition) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table3' */
    WhlSpdFL = look1_binlx(elapseTime, VehCtrlMdel240918_2018b_ConstP.pooled18,
      VehCtrlMdel240918_2018b_ConstP.pooled17, 7U);
  } else {
    WhlSpdFL = 0.0;
  }

  /* End of Switch: '<S8>/Switch1' */

  /* SignalConversion generated from: '<S8>/Out1' */
  WhlSpdRL_mps = WhlSpdFL;

  /* Lookup_n-D: '<S8>/2-D Lookup Table2' */
  WhlSpdFL = look1_binlx(elapseTime,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable2_bp01Data,
    VehCtrlMdel240918_2018b_ConstP.uDLookupTable2_tableData, 6U);

  /* DataTypeConversion: '<S8>/Cast To Single' */
  rtb_CastToDouble = (real32_T)WhlSpdFL;

  /* Gain: '<S8>/Gain' */
  rtb_CastToDouble *= 10.0F;

  /* SignalConversion generated from: '<S8>/Out1' */
  AMK_Trq_CUT = VehCtrlMdel240918_2018b_B.Exit_g;

  /* RelationalOperator: '<S94>/Compare' incorporates:
   *  Constant: '<S94>/Constant'
   */
  rtb_Compare = (rtb_Switch2_on > 35.0);

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
   *  UnitDelay: '<S192>/Unit Delay1'
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

  /* SampleTimeMath: '<S39>/sample time'
   *
   * About '<S39>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S39>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant42'
   */
  rtb_g_mpss1 = 2000.0 * elapseTime;

  /* UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add7 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_d;

  /* SampleTimeMath: '<S42>/sample time'
   *
   * About '<S42>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Gain4 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S42>/delta rise limit' */
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
   *  UnitDelay: '<S192>/Unit Delay1'
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

  /* Sum: '<S42>/Difference Inputs1'
   *
   * Block description for '<S42>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_mn -= rtb_Add7;

  /* RelationalOperator: '<S57>/LowerRelop1' */
  rtb_ignition = (rtb_Switch2_mn > rtb_Switch2_b0);

  /* Switch: '<S57>/Switch2' */
  if (!rtb_ignition) {
    /* Product: '<S42>/delta fall limit' */
    rtb_Add10 = (real32_T)(-1000.0 * rtb_Gain4);

    /* RelationalOperator: '<S57>/UpperRelop' */
    rtb_ignition = (rtb_Switch2_mn < rtb_Add10);

    /* Switch: '<S57>/Switch' */
    if (rtb_ignition) {
      rtb_Switch2_mn = rtb_Add10;
    }

    /* End of Switch: '<S57>/Switch' */
    rtb_Switch2_b0 = rtb_Switch2_mn;
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_d = rtb_Switch2_b0 + rtb_Add7;

  /* Sum: '<S10>/Add' incorporates:
   *  UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdFL = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_d - rtb_UkYk1;

  /* UnitDelay: '<S10>/Unit Delay3' */
  rtb_ignition = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_l;

  /* Abs: '<S10>/Abs' */
  rtb_Gain4 = fabs(WhlSpdFL);

  /* RelationalOperator: '<S29>/Compare' incorporates:
   *  Constant: '<S29>/Constant'
   */
  rtb_LogicalOperator2 = (rtb_Gain4 > 10.0);

  /* Abs: '<S10>/Abs1' */
  rtb_Gain4 = fabs(rtb_UkYk1);

  /* RelationalOperator: '<S30>/Compare' incorporates:
   *  Constant: '<S30>/Constant'
   */
  rtb_AND2 = (rtb_Gain4 > 5.0);

  /* RelationalOperator: '<S31>/Compare' incorporates:
   *  Constant: '<S31>/Constant'
   *  UnitDelay: '<S192>/Unit Delay1'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k > 3.0F);

  /* UnitDelay: '<S7>/Unit Delay' */
  rtb_Gain4 = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_n;

  /* Logic: '<S10>/AND' */
  rtb_UpperRelop_ir = (rtb_LogicalOperator2 && rtb_AND2 && rtb_LowerRelop1_b &&
                       (rtb_Gain4 != 0.0));

  /* Logic: '<S10>/Logical Operator4' */
  rtb_ignition = ((!rtb_ignition) && (!rtb_UpperRelop_ir));

  /* Abs: '<S10>/Abs2' */
  rtb_Gain4 = fabs(WhlSpdFL);

  /* RelationalOperator: '<S32>/Compare' incorporates:
   *  Constant: '<S32>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Gain4 < 10.0);

  /* RelationalOperator: '<S33>/Compare' incorporates:
   *  Constant: '<S33>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Add2 < -5.0);

  /* Logic: '<S10>/OR' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir || rtb_LowerRelop1_b);

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

  /* RelationalOperator: '<S28>/Compare' incorporates:
   *  Constant: '<S28>/Constant'
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
   *  UnitDelay: '<S192>/Unit Delay1'
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

  /* Sum: '<S39>/Difference Inputs1' incorporates:
   *  UnitDelay: '<S39>/Delay Input2'
   *
   * Block description for '<S39>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S39>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = rtb_UkYk1_ix - VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_jk;

  /* RelationalOperator: '<S54>/LowerRelop1' */
  rtb_UpperRelop_ir = (WhlSpdRR_mps > rtb_g_mpss1);

  /* Switch: '<S54>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S39>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S54>/UpperRelop' */
    rtb_ignition = (WhlSpdRR_mps < elapseTime);

    /* Switch: '<S54>/Switch' */
    if (rtb_ignition) {
      WhlSpdRR_mps = elapseTime;
    }

    /* End of Switch: '<S54>/Switch' */
    rtb_g_mpss1 = WhlSpdRR_mps;
  }

  /* End of Switch: '<S54>/Switch2' */

  /* Sum: '<S39>/Difference Inputs2' incorporates:
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
   *  UnitDelay: '<S39>/Delay Input2'
   *
   * Block description for '<S39>/Delay Input2':
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

  /* UnitDelay: '<S41>/Delay Input2'
   *
   * Block description for '<S41>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n0;

  /* Sum: '<S41>/Difference Inputs1'
   *
   * Block description for '<S41>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 = elapseTime - WhlSpdRR_mps;

  /* SampleTimeMath: '<S41>/sample time'
   *
   * About '<S41>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S41>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant45'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 1000.0 * elapseTime;

  /* RelationalOperator: '<S56>/LowerRelop1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_UpperRelop_ir = (rtb_Gain5 >
                       VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d);

  /* Switch: '<S56>/Switch2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S41>/delta fall limit' */
    elapseTime *= -1000.0;

    /* RelationalOperator: '<S56>/UpperRelop' */
    rtb_ignition = (rtb_Gain5 < elapseTime);

    /* Switch: '<S56>/Switch' */
    if (rtb_ignition) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S56>/Switch' */
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = rtb_Gain5;
  }

  /* End of Switch: '<S56>/Switch2' */

  /* Sum: '<S41>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n0 =
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d + WhlSpdRR_mps;

  /* Gain: '<S10>/Gain19' incorporates:
   *  UnitDelay: '<S41>/Delay Input2'
   *
   * Block description for '<S41>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = -VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_n0;

  /* RelationalOperator: '<S45>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add6_p > rtb_Switch2_mn);

  /* Switch: '<S45>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Add6_p = rtb_Switch2_mn;
  } else {
    /* RelationalOperator: '<S45>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    rtb_ignition = (rtb_Add6_p < 0.0F);

    /* Switch: '<S45>/Switch' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    if (rtb_ignition) {
      rtb_Add6_p = 0.0F;
    }

    /* End of Switch: '<S45>/Switch' */
  }

  /* End of Switch: '<S45>/Switch2' */

  /* Sum: '<S10>/Add13' */
  elapseTime = rtb_Add6_p + WhlSpdRR_mps;

  /* RelationalOperator: '<S48>/LowerRelop1' */
  rtb_UpperRelop_ir = (elapseTime > rtb_Switch2_mn);

  /* Switch: '<S48>/Switch2' */
  if (rtb_UpperRelop_ir) {
    elapseTime = rtb_Switch2_mn;
  } else {
    /* RelationalOperator: '<S48>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    rtb_ignition = (elapseTime < 0.0);

    /* Switch: '<S48>/Switch' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    if (rtb_ignition) {
      elapseTime = 0.0;
    }

    /* End of Switch: '<S48>/Switch' */
  }

  /* End of Switch: '<S48>/Switch2' */

  /* UnitDelay: '<S38>/Delay Input2'
   *
   * Block description for '<S38>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hk;

  /* Sum: '<S38>/Difference Inputs1'
   *
   * Block description for '<S38>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 = elapseTime - WhlSpdRR_mps;

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
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 2000.0 * elapseTime;

  /* RelationalOperator: '<S53>/LowerRelop1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_UpperRelop_ir = (rtb_Gain5 >
                       VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d);

  /* Switch: '<S53>/Switch2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S38>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S53>/UpperRelop' */
    rtb_ignition = (rtb_Gain5 < elapseTime);

    /* Switch: '<S53>/Switch' */
    if (rtb_ignition) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S53>/Switch' */
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = rtb_Gain5;
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hk =
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d + WhlSpdRR_mps;

  /* Sum: '<S24>/Add' incorporates:
   *  Constant: '<S24>/Constant3'
   */
  WhlSpdRR_mps = 1.0 - WhlSpdRL_mps;

  /* Product: '<S24>/Product5' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 63.0 * WhlSpdRR_mps;

  /* Product: '<S24>/Product' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d * 9550.0;

  /* Sum: '<S24>/Add1' incorporates:
   *  Constant: '<S24>/RPM_min'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = RPM + 10.0;

  /* MinMax: '<S24>/Max' incorporates:
   *  Constant: '<S24>/RPM_min1'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_Gain5 = fmax(VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d, 1.0);

  /* Product: '<S24>/Divide' */
  WhlSpdRR_mps /= rtb_Gain5;

  /* MinMax: '<S7>/MinMax2' incorporates:
   *  UnitDelay: '<S38>/Delay Input2'
   *
   * Block description for '<S38>/Delay Input2':
   *
   *  Store in Global RAM
   */
  elapseTime = fmin(WhlSpdRR_mps,
                    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hk);

  /* Saturate: '<S27>/Saturation' */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k > 40.0F) {
    rtb_Switch2_mn = 40.0F;
  } else if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k < 0.0F) {
    rtb_Switch2_mn = 0.0F;
  } else {
    rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k;
  }

  /* Lookup_n-D: '<S27>/VehSpd_SlipTarget_mps' */
  rtb_Add6_p = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled54, 3U);

  /* Sum: '<S27>/Add9' */
  rtb_Add6_p += rtb_Switch2_mn;

  /* Sum: '<S7>/Add1' */
  rtb_Ax = rtb_deltafalllimit_n + rtb_deltafalllimit_om;

  /* Gain: '<S7>/Gain2' */
  rtb_Ax *= 0.5F;

  /* Saturate: '<S27>/Saturation1' */
  if (rtb_Ax < 0.0F) {
    rtb_VxIMU_est = 0.0F;
  } else {
    rtb_VxIMU_est = rtb_Ax;
  }

  /* End of Saturate: '<S27>/Saturation1' */

  /* Sum: '<S27>/Add1' */
  rtb_deltafalllimit_n = rtb_Add6_p - rtb_VxIMU_est;

  /* UnitDelay: '<S84>/Unit Delay1' */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_e;

  /* RelationalOperator: '<S27>/Relational Operator7' incorporates:
   *  Constant: '<S27>/Cal_DeltaV_mps'
   */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_n < 0.0F);

  /* Logic: '<S84>/Logical Operator4' */
  rtb_UpperRelop_ir = ((!rtb_UpperRelop_ir) && (!rtb_LowerRelop1_b));

  /* Logic: '<S27>/Logical Operator2' */
  rtb_LowerRelop1_b = !rtb_LowerRelop1_b;

  /* UnitDelay: '<S27>/Unit Delay4' */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE;

  /* RelationalOperator: '<S27>/Relational Operator8' incorporates:
   *  Constant: '<S27>/Cal_DeltaV_mps1'
   */
  rtb_AND2 = (WhlSpdRR_mps > 235.0);

  /* Logic: '<S27>/Logical Operator1' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b && rtb_AND2);

  /* Switch: '<S85>/Switch6' incorporates:
   *  Constant: '<S85>/Reset'
   */
  if (rtb_LowerRelop1_b) {
    /* Sum: '<S85>/Add10' incorporates:
     *  Constant: '<S85>/Steptime'
     */
    rtb_Ax = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_i + 0.01F;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S85>/Switch6' */

  /* MinMax: '<S85>/Min' incorporates:
   *  Constant: '<S27>/ResetDelay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_i = fminf(rtb_Ax, 0.1F);

  /* RelationalOperator: '<S85>/Relational Operator9' incorporates:
   *  Constant: '<S27>/ResetDelay'
   *  UnitDelay: '<S85>/Unit Delay1'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_i >= 0.1F);

  /* UnitDelay: '<S27>/Unit Delay3' */
  rtb_AND2 = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_a;

  /* Logic: '<S27>/Logical Operator3' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b || rtb_AND2);

  /* Logic: '<S84>/Logical Operator5' incorporates:
   *  UnitDelay: '<S84>/Unit Delay1'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_e = ((!rtb_UpperRelop_ir) &&
    (!rtb_LowerRelop1_b));

  /* Switch: '<S27>/Switch6' incorporates:
   *  Constant: '<S27>/Verror_Reset'
   *  UnitDelay: '<S84>/Unit Delay1'
   */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_e) {
    rtb_Ax = rtb_deltafalllimit_n;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S27>/Switch6' */

  /* Product: '<S27>/Product' incorporates:
   *  Constant: '<S27>/P_Gain'
   */
  rtb_deltafalllimit_om = rtb_Ax * 40.0F;

  /* Sum: '<S27>/Add11' */
  WhlSpdRR_mps = elapseTime - rtb_deltafalllimit_om;

  /* UnitDelay: '<S27>/Unit Delay5' */
  rtb_Add6_p = VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE_l;

  /* Product: '<S27>/Product2' */
  rtb_Add6_p *= rtb_deltafalllimit_n;

  /* RelationalOperator: '<S79>/Compare' incorporates:
   *  Constant: '<S79>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add6_p <= 0.0F);

  /* UnitDelay: '<S27>/Unit Delay' */
  rtb_Add6_p = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_b;

  /* Switch: '<S27>/Switch3' incorporates:
   *  Constant: '<S27>/Verror_Reset1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add6_p = 0.0F;
  }

  /* End of Switch: '<S27>/Switch3' */

  /* Sum: '<S27>/Add2' */
  rtb_Add6_p += rtb_Ax;

  /* Saturate: '<S27>/Saturation2' */
  if (rtb_Add6_p > 400.0F) {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_b = 400.0F;
  } else if (rtb_Add6_p < -100.0F) {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_b = -100.0F;
  } else {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_b = rtb_Add6_p;
  }

  /* End of Saturate: '<S27>/Saturation2' */

  /* RelationalOperator: '<S89>/Compare' incorporates:
   *  UnitDelay: '<S84>/Unit Delay1'
   */
  rtb_ignition = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_e;

  /* UnitDelay: '<S83>/Delay Input1'
   *
   * Block description for '<S83>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.DelayInput1_DSTATE;

  /* RelationalOperator: '<S83>/FixPt Relational Operator' */
  rtb_UpperRelop_ir = ((int32_T)rtb_ignition > (int32_T)rtb_UpperRelop_ir);

  /* Switch: '<S27>/Switch' incorporates:
   *  Constant: '<S27>/Integr_StartPoint'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S27>/Add4' */
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = elapseTime -
      VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_g;
  } else {
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 0.0;
  }

  /* End of Switch: '<S27>/Switch' */

  /* Lookup_n-D: '<S27>/VehicleStableTarget_mps' */
  rtb_Ax = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled61, 3U);

  /* Sum: '<S27>/Add5' */
  rtb_Ax += rtb_Switch2_mn;

  /* Sum: '<S27>/Add10' */
  rtb_Ax = rtb_VxIMU_est - rtb_Ax;

  /* RelationalOperator: '<S27>/Relational Operator' incorporates:
   *  Constant: '<S27>/Verror'
   */
  rtb_UpperRelop_ir = (rtb_Ax < 0.0F);

  /* Logic: '<S27>/Logical Operator4' incorporates:
   *  UnitDelay: '<S84>/Unit Delay1'
   */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir &&
                       VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_e);

  /* Switch: '<S27>/Switch1' incorporates:
   *  Constant: '<S27>/Trq_IReset'
   *  Constant: '<S27>/Trq_I_FF'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Ax = 20.0F;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S27>/Switch1' */

  /* Sum: '<S27>/Add6' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S27>/Unit Delay'
   */
  rtb_Gain5 = (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_b +
               VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d) + rtb_Ax;

  /* Product: '<S27>/Product1' */
  WhlSpdRL_mps = rtb_Gain5 * 10.0;

  /* RelationalOperator: '<S86>/LowerRelop1' */
  rtb_UpperRelop_ir = (WhlSpdRL_mps > WhlSpdRR_mps);

  /* Switch: '<S86>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Gain: '<S27>/Gain3' */
    rtb_Add6_p = -rtb_deltafalllimit_om;

    /* RelationalOperator: '<S86>/UpperRelop' */
    rtb_LogicalOperator2 = (WhlSpdRL_mps < rtb_Add6_p);

    /* Switch: '<S86>/Switch' */
    if (rtb_LogicalOperator2) {
      WhlSpdRL_mps = rtb_Add6_p;
    }

    /* End of Switch: '<S86>/Switch' */
    WhlSpdRR_mps = WhlSpdRL_mps;
  }

  /* End of Switch: '<S86>/Switch2' */

  /* Sum: '<S27>/Add7' incorporates:
   *  UnitDelay: '<S27>/Unit Delay4'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE = rtb_deltafalllimit_om +
    WhlSpdRR_mps;

  /* Lookup_n-D: '<S27>/VehicleStableTarget_mps1' */
  rtb_Ax = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled61, 3U);

  /* Sum: '<S27>/Add13' */
  rtb_Switch2_mn += rtb_Ax;

  /* Sum: '<S27>/Add12' */
  rtb_Switch2_mn = rtb_VxIMU_est - rtb_Switch2_mn;

  /* RelationalOperator: '<S27>/Relational Operator1' incorporates:
   *  Constant: '<S27>/Verror1'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn < 0.0F);

  /* RelationalOperator: '<S27>/Relational Operator2' incorporates:
   *  UnitDelay: '<S27>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE >=
                       elapseTime);

  /* RelationalOperator: '<S81>/Compare' incorporates:
   *  Constant: '<S81>/Constant'
   *  UnitDelay: '<S27>/Unit Delay4'
   */
  rtb_AND2 = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE <= 0.01);

  /* Logic: '<S27>/OR' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b || rtb_AND2);

  /* Logic: '<S27>/Logical Operator5' incorporates:
   *  UnitDelay: '<S27>/Unit Delay3'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_a = (rtb_UpperRelop_ir &&
    rtb_LowerRelop1_b);

  /* Switch: '<S27>/Switch2' incorporates:
   *  Switch: '<S27>/Switch7'
   *  UnitDelay: '<S27>/Unit Delay3'
   *  UnitDelay: '<S84>/Unit Delay1'
   */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_e) {
    /* RelationalOperator: '<S87>/LowerRelop1' incorporates:
     *  Constant: '<S27>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S27>/Unit Delay4'
     */
    rtb_LogicalOperator2 = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE > 235.0);

    /* Switch: '<S87>/Switch2' incorporates:
     *  Constant: '<S27>/TCS_TrqRequest_Max2'
     */
    if (rtb_LogicalOperator2) {
      VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g = 235.0;
    } else {
      /* RelationalOperator: '<S87>/UpperRelop' incorporates:
       *  Constant: '<S27>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S27>/Unit Delay4'
       */
      rtb_LogicalOperator2 = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE < 0.0);

      /* Switch: '<S87>/Switch' incorporates:
       *  Constant: '<S27>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S27>/Unit Delay4'
       */
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g = 0.0;
      } else {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g =
          VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE;
      }

      /* End of Switch: '<S87>/Switch' */
    }

    /* End of Switch: '<S87>/Switch2' */

    /* RelationalOperator: '<S88>/LowerRelop1' */
    rtb_LogicalOperator2 = (elapseTime >
      VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g);

    /* Switch: '<S88>/Switch2' */
    if (!rtb_LogicalOperator2) {
      /* RelationalOperator: '<S88>/UpperRelop' incorporates:
       *  Constant: '<S27>/TCS_TrqRequest_Min1'
       */
      rtb_LogicalOperator2 = (elapseTime < 0.0);

      /* Switch: '<S88>/Switch' incorporates:
       *  Constant: '<S27>/TCS_TrqRequest_Min1'
       */
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g = 0.0;
      } else {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g = elapseTime;
      }

      /* End of Switch: '<S88>/Switch' */
    }

    /* End of Switch: '<S88>/Switch2' */
  } else {
    if (VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_a) {
      /* Switch: '<S27>/Switch7' */
      VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g = elapseTime;
    }
  }

  /* End of Switch: '<S27>/Switch2' */

  /* UnitDelay: '<S73>/Unit Delay1' */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_gl;

  /* Saturate: '<S26>/Saturation' */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k > 40.0F) {
    rtb_Ax = 40.0F;
  } else if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k < 0.0F) {
    rtb_Ax = 0.0F;
  } else {
    rtb_Ax = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Saturate: '<S26>/Saturation' */

  /* Lookup_n-D: '<S26>/VehSpd_SlipTarget_mps' */
  rtb_Switch2_mn = look1_iflf_binlc(rtb_Ax,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled54, 3U);

  /* Sum: '<S26>/Add9' */
  rtb_Switch2_mn += rtb_Ax;

  /* Saturate: '<S26>/Saturation1' */
  if (rtb_Divide > 50.0F) {
    rtb_Add6_p = 50.0F;
  } else if (rtb_Divide < 0.0F) {
    rtb_Add6_p = 0.0F;
  } else {
    rtb_Add6_p = rtb_Divide;
  }

  /* End of Saturate: '<S26>/Saturation1' */

  /* Sum: '<S26>/Add1' */
  rtb_Divide = rtb_Switch2_mn - rtb_Add6_p;

  /* RelationalOperator: '<S26>/Relational Operator7' incorporates:
   *  Constant: '<S26>/Cal_DeltaV_mps'
   */
  rtb_LowerRelop1_b = (rtb_Divide < 0.0F);

  /* Logic: '<S73>/Logical Operator4' */
  rtb_UpperRelop_ir = ((!rtb_UpperRelop_ir) && (!rtb_LowerRelop1_b));

  /* Logic: '<S26>/Logical Operator2' */
  rtb_LowerRelop1_b = !rtb_LowerRelop1_b;

  /* UnitDelay: '<S26>/Unit Delay4' */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_i;

  /* RelationalOperator: '<S26>/Relational Operator8' incorporates:
   *  Constant: '<S26>/Cal_DeltaV_mps1'
   */
  rtb_AND2 = (WhlSpdRR_mps > 235.0);

  /* Logic: '<S26>/Logical Operator1' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b && rtb_AND2);

  /* Switch: '<S74>/Switch6' incorporates:
   *  Constant: '<S74>/Reset'
   */
  if (rtb_LowerRelop1_b) {
    /* Sum: '<S74>/Add10' incorporates:
     *  Constant: '<S74>/Steptime'
     */
    rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_n5 + 0.01F;
  } else {
    rtb_Switch2_mn = 0.0F;
  }

  /* End of Switch: '<S74>/Switch6' */

  /* MinMax: '<S74>/Min' incorporates:
   *  Constant: '<S26>/ResetDelay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_n5 = fminf(rtb_Switch2_mn, 0.1F);

  /* RelationalOperator: '<S74>/Relational Operator9' incorporates:
   *  Constant: '<S26>/ResetDelay'
   *  UnitDelay: '<S74>/Unit Delay1'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_n5 >= 0.1F);

  /* UnitDelay: '<S26>/Unit Delay3' */
  rtb_AND2 = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_e;

  /* Logic: '<S26>/Logical Operator3' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b || rtb_AND2);

  /* Logic: '<S73>/Logical Operator5' incorporates:
   *  UnitDelay: '<S73>/Unit Delay1'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_gl = ((!rtb_UpperRelop_ir) &&
    (!rtb_LowerRelop1_b));

  /* UnitDelay: '<S64>/Unit Delay1' */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_dp;

  /* Saturate: '<S25>/Saturation' */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k > 40.0F) {
    rtb_Switch2_mn = 40.0F;
  } else if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k < 0.0F) {
    rtb_Switch2_mn = 0.0F;
  } else {
    rtb_Switch2_mn = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Saturate: '<S25>/Saturation' */

  /* Lookup_n-D: '<S25>/VehSpd_SlipTarget_mps' */
  rtb_MaxWhlSpd_mps_n = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled54, 3U);

  /* Sum: '<S25>/Add9' */
  rtb_MaxWhlSpd_mps_n += rtb_Switch2_mn;

  /* Saturate: '<S25>/Saturation1' */
  if (rtb_Add < 0.0F) {
    rtb_Add = 0.0F;
  }

  /* End of Saturate: '<S25>/Saturation1' */

  /* Sum: '<S25>/Add1' */
  rtb_Add4_j = rtb_MaxWhlSpd_mps_n - rtb_Add;

  /* RelationalOperator: '<S25>/Relational Operator7' incorporates:
   *  Constant: '<S25>/Cal_DeltaV_mps'
   */
  rtb_LowerRelop1_b = (rtb_Add4_j < 0.0F);

  /* Logic: '<S64>/Logical Operator4' */
  rtb_UpperRelop_ir = ((!rtb_UpperRelop_ir) && (!rtb_LowerRelop1_b));

  /* Logic: '<S25>/Logical Operator2' */
  rtb_LowerRelop1_b = !rtb_LowerRelop1_b;

  /* UnitDelay: '<S25>/Unit Delay4' */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_l;

  /* RelationalOperator: '<S25>/Relational Operator8' incorporates:
   *  Constant: '<S25>/Cal_DeltaV_mps1'
   */
  rtb_AND2 = (WhlSpdRR_mps > 235.0);

  /* Logic: '<S25>/Logical Operator1' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b && rtb_AND2);

  /* Switch: '<S65>/Switch6' incorporates:
   *  Constant: '<S65>/Reset'
   */
  if (rtb_LowerRelop1_b) {
    /* Sum: '<S65>/Add10' incorporates:
     *  Constant: '<S65>/Steptime'
     */
    rtb_MaxWhlSpd_mps_n = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_h + 0.01F;
  } else {
    rtb_MaxWhlSpd_mps_n = 0.0F;
  }

  /* End of Switch: '<S65>/Switch6' */

  /* MinMax: '<S65>/Min' incorporates:
   *  Constant: '<S25>/ResetDelay'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_h = fminf(rtb_MaxWhlSpd_mps_n,
    0.1F);

  /* RelationalOperator: '<S65>/Relational Operator9' incorporates:
   *  Constant: '<S25>/ResetDelay'
   *  UnitDelay: '<S65>/Unit Delay1'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_h >= 0.1F);

  /* UnitDelay: '<S25>/Unit Delay3' */
  rtb_AND2 = VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_i;

  /* Logic: '<S25>/Logical Operator3' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b || rtb_AND2);

  /* Logic: '<S64>/Logical Operator5' incorporates:
   *  UnitDelay: '<S64>/Unit Delay1'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_dp = ((!rtb_UpperRelop_ir) &&
    (!rtb_LowerRelop1_b));

  /* Chart: '<S7>/Chart' */
  if (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_c7_VehCtrlMdel240918_ ==
      0U) {
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_active_c7_VehCtrlMdel240918_ = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_B = 3U;
    VehCtrlMdel240918_2018b_DW.b = rtb_Add2 * rtb_Add2 + rtb_deltafalllimit_i4 *
      rtb_deltafalllimit_i4;
    VehCtrlMdel240918_2018b_DW.b = sqrt(VehCtrlMdel240918_2018b_DW.b);
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_C = 3U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_D = 1U;
    VehCtrlMdel240918_2018b_DW.bitsForTID3.is_E = 1U;
  } else {
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_B) {
     case VehCtrlMdel240_IN_DYC_Disenable:
      VehCtrlMdel240918_2018b_B.DYC_Enable_OUT = 0.0;
      break;

     case VehCtrlMdel240918_IN_DYC_Enable:
      VehCtrlMdel240918_2018b_B.DYC_Enable_OUT = 1.0;
      rtb_LogicalOperator2 = ((rtb_UkYk1 >= 50.0) || (rtb_deltafalllimit_i4 >=
        5.0) || (VehCtrlMdel240918_2018b_DW.b >= 5.0));
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_B = 3U;
      }
      break;

     default:
      /* case IN_InitState: */
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_B = 1U;
      VehCtrlMdel240918_2018b_B.DYC_Enable_OUT = 0.0;
      VehCtrlMdel240918_2018b_DW.DYC_flag = 0.0;
      break;
    }

    if (fabs(rtb_deltafalllimit_i4) > 0.5) {
      WhlSpdRR_mps = atan(rtb_Add2 / rtb_deltafalllimit_i4) * 180.0 /
        3.1415926535897931;
    } else {
      WhlSpdRR_mps = 0.0;
    }

    VehCtrlMdel240918_2018b_DW.b = rtb_Add2 * rtb_Add2 + rtb_deltafalllimit_i4 *
      rtb_deltafalllimit_i4;
    VehCtrlMdel240918_2018b_DW.b = sqrt(VehCtrlMdel240918_2018b_DW.b);
    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_C) {
     case VehCtrlMdel240918_2018b_IN_B:
      rtb_LogicalOperator2 = ((!(VehCtrlMdel240918_2018b_DW.DYC_flag != 0.0)) ||
        (rtb_UkYk1 > 30.0) || (WhlSpdRR_mps > 30.0) || (WhlSpdRR_mps > -30.0) ||
        (VehCtrlMdel240918_2018b_DW.b > 3.0));
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_C = 3U;
      }
      break;

     case VehCtrlMdel240918_2018b_IN_C:
      break;

     default:
      /* case IN_F_TVD_TCS_STATE: */
      rtb_LogicalOperator2 = ((!(VehCtrlMdel240918_2018b_DW.DYC_flag != 0.0)) ||
        (rtb_UkYk1 > 30.0) || (WhlSpdRR_mps > 30.0) || (WhlSpdRR_mps > -30.0) ||
        (VehCtrlMdel240918_2018b_DW.b > 3.0));
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_C = 2U;
        VehCtrlMdel240918_2018b_B.TCSR_Enable_OUT = 0.0;
      }
      break;
    }

    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_D) {
     case VehCtrlMdel240918_IN_InitState2:
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_D = 2U;
      VehCtrlMdel240918_2018b_B.TCSR_Enable_OUT = 0.0;
      break;

     case VehCtrlMdel24_IN_TCSR_Disenable:
      break;

     default:
      /* case IN_TCSR_Enable: */
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_D = 2U;
      VehCtrlMdel240918_2018b_B.TCSR_Enable_OUT = 0.0;
      break;
    }

    switch (VehCtrlMdel240918_2018b_DW.bitsForTID3.is_E) {
     case VehCtrlMdel240918_IN_InitState1:
      VehCtrlMdel240918_2018b_DW.bitsForTID3.is_E = 2U;
      VehCtrlMdel240918_2018b_B.TCSF_Enable_OUT = 0.0;
      break;

     case VehCtrlMdel24_IN_TCSF_Disenable:
      VehCtrlMdel240918_2018b_B.TCSF_Enable_OUT = 0.0;
      break;

     default:
      /* case IN_TCSF_Enable: */
      VehCtrlMdel240918_2018b_B.TCSF_Enable_OUT = 1.0;
      if (VehCtrlMdel240918_2018b_DW.b > 5.0) {
        VehCtrlMdel240918_2018b_DW.bitsForTID3.is_E = 1U;
      }
      break;
    }
  }

  /* End of Chart: '<S7>/Chart' */

  /* Logic: '<S7>/Logical Operator2' */
  rtb_UpperRelop_ir = !VehCtrlMdel240918_2018b_B.VehReady;

  /* Logic: '<S7>/Logical Operator3' */
  rtb_LowerRelop1_b = (rtb_UpperRelop_ir || (Trq_CUT != 0.0));

  /* Logic: '<S7>/Logical Operator1' */
  rtb_LogicalOperator2 = ((EMRAX_Trq_CUT != 0.0) || rtb_LowerRelop1_b);

  /* Switch: '<S7>/Switch2' incorporates:
   *  Constant: '<S7>/Constant4'
   *  Switch: '<S7>/Switch5'
   */
  if (rtb_LogicalOperator2) {
    rtb_VxIMU_est = 0.0F;
  } else {
    if (VehCtrlMdel240918_2018b_B.TCSR_Enable_OUT != 0.0) {
      /* Abs: '<S27>/Abs' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_UkYk1 = fabs(rtb_deltafalllimit_iz);

      /* RelationalOperator: '<S80>/Compare' incorporates:
       *  Constant: '<S80>/Constant'
       *  Switch: '<S7>/Switch5'
       */
      rtb_AND2 = (rtb_UkYk1 <= 20.0);

      /* RelationalOperator: '<S82>/Compare' incorporates:
       *  Constant: '<S82>/Constant'
       *  Switch: '<S7>/Switch5'
       */
      rtb_UpperRelop_ir = (rtb_VxIMU_est > 0.0F);

      /* Logic: '<S27>/Logical Operator6' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_UpperRelop_ir = (rtb_UpperRelop_ir && rtb_AND2);

      /* Switch: '<S27>/Switch5' incorporates:
       *  Switch: '<S7>/Switch5'
       *  UnitDelay: '<S27>/Unit Delay2'
       */
      if (rtb_UpperRelop_ir) {
        elapseTime = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_g;
      }

      /* End of Switch: '<S27>/Switch5' */
    }

    /* Gain: '<S7>/Gain6' */
    rtb_UkYk1 = 5.5555555555555554 * elapseTime;

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefRear' */
    rtb_VxIMU_est = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240918_2018b_ConstP.BrakeCompensateCoefRear_bp01Dat,
      VehCtrlMdel240918_2018b_ConstP.BrakeCompensateCoefRear_tableDa, 1U);

    /* RelationalOperator: '<S21>/LowerRelop1' */
    rtb_AND2 = (rtb_UkYk1 > rtb_VxIMU_est);

    /* Switch: '<S21>/Switch2' */
    if (!rtb_AND2) {
      /* RelationalOperator: '<S21>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      rtb_AND2 = (rtb_UkYk1 < 0.0);

      /* Switch: '<S21>/Switch' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      if (rtb_AND2) {
        rtb_VxIMU_est = 0.0F;
      } else {
        rtb_VxIMU_est = (real32_T)rtb_UkYk1;
      }

      /* End of Switch: '<S21>/Switch' */
    }

    /* End of Switch: '<S21>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch2' */

  /* UnitDelay: '<S18>/Delay Input2'
   *
   * Block description for '<S18>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_MaxWhlSpd_mps_n = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_cd;

  /* Sum: '<S18>/Difference Inputs1'
   *
   * Block description for '<S18>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1_ea = rtb_VxIMU_est - rtb_MaxWhlSpd_mps_n;

  /* SampleTimeMath: '<S18>/sample time'
   *
   * About '<S18>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S18>/delta rise limit' */
  rtb_VxIMU_est = (real32_T)(25000.0 * elapseTime);

  /* RelationalOperator: '<S58>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1_ea > rtb_VxIMU_est);

  /* Switch: '<S58>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S18>/delta fall limit' */
    rtb_VxIMU_est = (real32_T)(-25000.0 * elapseTime);

    /* RelationalOperator: '<S58>/UpperRelop' */
    rtb_AND2 = (rtb_UkYk1_ea < rtb_VxIMU_est);

    /* Switch: '<S58>/Switch' */
    if (rtb_AND2) {
      rtb_UkYk1_ea = rtb_VxIMU_est;
    }

    /* End of Switch: '<S58>/Switch' */
    rtb_VxIMU_est = rtb_UkYk1_ea;
  }

  /* End of Switch: '<S58>/Switch2' */

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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_cd = rtb_VxIMU_est +
    rtb_MaxWhlSpd_mps_n;
  if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_cd > 1000.0F) {
    TrqR_cmd = 1000.0F;
  } else if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_cd < 0.0F) {
    TrqR_cmd = 0.0F;
  } else {
    TrqR_cmd = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_cd;
  }

  /* End of Saturate: '<S7>/Saturation1' */

  /* Logic: '<S7>/Logical Operator4' */
  Trq_CUT_final = ((AMK_Trq_CUT != 0.0) || rtb_LowerRelop1_b ||
                   rtb_LogicalOperator2);

  /* UnitDelay: '<S36>/Delay Input2'
   *
   * Block description for '<S36>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_mb;

  /* SampleTimeMath: '<S36>/sample time'
   *
   * About '<S36>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S36>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 2000.0 * elapseTime;

  /* RelationalOperator: '<S43>/LowerRelop1' */
  rtb_UpperRelop_ir = (FLWhlStrAng > rtb_Switch2_b0);

  /* Switch: '<S43>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Gain5 = rtb_Switch2_b0;
  } else {
    /* RelationalOperator: '<S43>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    rtb_LowerRelop1_b = (FLWhlStrAng < 0.0);

    /* Switch: '<S43>/Switch' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    if (rtb_LowerRelop1_b) {
      FLWhlStrAng = 0.0;
    }

    /* End of Switch: '<S43>/Switch' */
    rtb_Gain5 = FLWhlStrAng;
  }

  /* End of Switch: '<S43>/Switch2' */

  /* UnitDelay: '<S40>/Delay Input2'
   *
   * Block description for '<S40>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_VxIMU_est = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_fo;

  /* SampleTimeMath: '<S40>/sample time'
   *
   * About '<S40>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Gain4 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S40>/delta rise limit' incorporates:
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

  /* Sum: '<S40>/Difference Inputs1'
   *
   * Block description for '<S40>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add10 -= rtb_VxIMU_est;

  /* RelationalOperator: '<S55>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add10 > rtb_MaxWhlSpd_mps_n);

  /* Switch: '<S55>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S40>/delta fall limit' */
    rtb_MaxWhlSpd_mps_n = (real32_T)(-1000.0 * rtb_Gain4);

    /* RelationalOperator: '<S55>/UpperRelop' */
    rtb_Compare = (rtb_Add10 < rtb_MaxWhlSpd_mps_n);

    /* Switch: '<S55>/Switch' */
    if (rtb_Compare) {
      rtb_Add10 = rtb_MaxWhlSpd_mps_n;
    }

    /* End of Switch: '<S55>/Switch' */
    rtb_MaxWhlSpd_mps_n = rtb_Add10;
  }

  /* End of Switch: '<S55>/Switch2' */

  /* Sum: '<S40>/Difference Inputs2' incorporates:
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_fo = rtb_MaxWhlSpd_mps_n +
    rtb_VxIMU_est;

  /* Gain: '<S10>/Gain20' incorporates:
   *  UnitDelay: '<S40>/Delay Input2'
   *
   * Block description for '<S40>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = -VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_fo;

  /* Sum: '<S10>/Add11' */
  rtb_UkYk1 = rtb_Gain5 + rtb_Add10;

  /* RelationalOperator: '<S46>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1 > rtb_Switch2_b0);

  /* Switch: '<S46>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_UkYk1 = rtb_Switch2_b0;
  } else {
    /* RelationalOperator: '<S46>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    rtb_Compare = (rtb_UkYk1 < 0.0);

    /* Switch: '<S46>/Switch' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    if (rtb_Compare) {
      rtb_UkYk1 = 0.0;
    }

    /* End of Switch: '<S46>/Switch' */
  }

  /* End of Switch: '<S46>/Switch2' */

  /* Sum: '<S36>/Difference Inputs1'
   *
   * Block description for '<S36>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 -= WhlSpdRR_mps;

  /* RelationalOperator: '<S51>/LowerRelop1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_UpperRelop_ir = (rtb_UkYk1 >
                       VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d);

  /* Switch: '<S51>/Switch2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S36>/delta fall limit' */
    rtb_deltafalllimit_iz = -2000.0 * elapseTime;

    /* RelationalOperator: '<S51>/UpperRelop' */
    rtb_Compare = (rtb_UkYk1 < rtb_deltafalllimit_iz);

    /* Switch: '<S51>/Switch' */
    if (rtb_Compare) {
      rtb_UkYk1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S51>/Switch' */
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = rtb_UkYk1;
  }

  /* End of Switch: '<S51>/Switch2' */

  /* Sum: '<S36>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_mb =
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d + WhlSpdRR_mps;

  /* Sum: '<S24>/Add4' incorporates:
   *  Constant: '<S24>/Constant'
   */
  WhlSpdRR_mps = 1.0 - WhlSpdFR;

  /* Product: '<S24>/Product4' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 7.0 * WhlSpdRR_mps;

  /* Product: '<S24>/Product1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_Gain5 = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d * 9550.0;

  /* Sum: '<S24>/Add2' incorporates:
   *  Constant: '<S24>/RPM_min2'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = MCFR_ActualVelocity + 10.0;

  /* MinMax: '<S24>/Max1' incorporates:
   *  Constant: '<S24>/RPM_min3'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_Gain4 = fmax(VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d, 1.0);

  /* Product: '<S24>/Divide1' */
  rtb_Gain5 /= rtb_Gain4;

  /* MinMax: '<S7>/MinMax' incorporates:
   *  UnitDelay: '<S36>/Delay Input2'
   *
   * Block description for '<S36>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = fmin(rtb_Gain5, VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_mb);

  /* Switch: '<S26>/Switch6' incorporates:
   *  Constant: '<S26>/Verror_Reset'
   *  UnitDelay: '<S73>/Unit Delay1'
   */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_gl) {
    rtb_VxIMU_est = rtb_Divide;
  } else {
    rtb_VxIMU_est = 0.0F;
  }

  /* End of Switch: '<S26>/Switch6' */

  /* Product: '<S26>/Product' incorporates:
   *  Constant: '<S26>/P_Gain'
   */
  rtb_Switch2_b0 = rtb_VxIMU_est * 40.0F;

  /* Sum: '<S26>/Add11' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = rtb_UkYk1 - rtb_Switch2_b0;

  /* UnitDelay: '<S26>/Unit Delay5' */
  rtb_MaxWhlSpd_mps_n = VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE_i;

  /* Product: '<S26>/Product2' */
  rtb_MaxWhlSpd_mps_n *= rtb_Divide;

  /* RelationalOperator: '<S70>/Compare' incorporates:
   *  Constant: '<S70>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_MaxWhlSpd_mps_n <= 0.0F);

  /* UnitDelay: '<S26>/Unit Delay' */
  rtb_MaxWhlSpd_mps_n = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_f;

  /* Switch: '<S26>/Switch3' incorporates:
   *  Constant: '<S26>/Verror_Reset1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_MaxWhlSpd_mps_n = 0.0F;
  }

  /* End of Switch: '<S26>/Switch3' */

  /* Sum: '<S26>/Add2' */
  rtb_MaxWhlSpd_mps_n += rtb_VxIMU_est;

  /* Saturate: '<S26>/Saturation2' */
  if (rtb_MaxWhlSpd_mps_n > 400.0F) {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_f = 400.0F;
  } else if (rtb_MaxWhlSpd_mps_n < -100.0F) {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_f = -100.0F;
  } else {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_f = rtb_MaxWhlSpd_mps_n;
  }

  /* End of Saturate: '<S26>/Saturation2' */

  /* RelationalOperator: '<S78>/Compare' incorporates:
   *  UnitDelay: '<S73>/Unit Delay1'
   */
  rtb_Compare = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_gl;

  /* UnitDelay: '<S72>/Delay Input1'
   *
   * Block description for '<S72>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.DelayInput1_DSTATE_j;

  /* RelationalOperator: '<S72>/FixPt Relational Operator' */
  rtb_UpperRelop_ir = ((int32_T)rtb_Compare > (int32_T)rtb_UpperRelop_ir);

  /* Switch: '<S26>/Switch' incorporates:
   *  Constant: '<S26>/Integr_StartPoint'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S26>/Add4' */
    rtb_Gain5 = rtb_UkYk1 - VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_f;
  } else {
    rtb_Gain5 = 0.0;
  }

  /* End of Switch: '<S26>/Switch' */

  /* Lookup_n-D: '<S26>/VehicleStableTarget_mps' */
  rtb_VxIMU_est = look1_iflf_binlc(rtb_Ax,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled61, 3U);

  /* Sum: '<S26>/Add5' */
  rtb_VxIMU_est += rtb_Ax;

  /* Sum: '<S26>/Add10' */
  rtb_VxIMU_est = rtb_Add6_p - rtb_VxIMU_est;

  /* RelationalOperator: '<S26>/Relational Operator' incorporates:
   *  Constant: '<S26>/Verror'
   */
  rtb_UpperRelop_ir = (rtb_VxIMU_est < 0.0F);

  /* Logic: '<S26>/Logical Operator4' incorporates:
   *  UnitDelay: '<S73>/Unit Delay1'
   */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir &&
                       VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_gl);

  /* Switch: '<S26>/Switch1' incorporates:
   *  Constant: '<S26>/Trq_IReset'
   *  Constant: '<S26>/Trq_I_FF'
   */
  if (rtb_UpperRelop_ir) {
    rtb_VxIMU_est = 20.0F;
  } else {
    rtb_VxIMU_est = 0.0F;
  }

  /* End of Switch: '<S26>/Switch1' */

  /* Sum: '<S26>/Add6' incorporates:
   *  UnitDelay: '<S26>/Unit Delay'
   */
  rtb_Gain4 = (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_f + rtb_Gain5) +
    rtb_VxIMU_est;

  /* Product: '<S26>/Product1' */
  rtb_deltafalllimit_iz = rtb_Gain4 * 10.0;

  /* RelationalOperator: '<S75>/LowerRelop1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_UpperRelop_ir = (rtb_deltafalllimit_iz >
                       VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d);

  /* Switch: '<S75>/Switch2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (!rtb_UpperRelop_ir) {
    /* Gain: '<S26>/Gain3' */
    rtb_VxIMU_est = -rtb_Switch2_b0;

    /* RelationalOperator: '<S75>/UpperRelop' */
    rtb_Compare_am = (rtb_deltafalllimit_iz < rtb_VxIMU_est);

    /* Switch: '<S75>/Switch' */
    if (rtb_Compare_am) {
      rtb_deltafalllimit_iz = rtb_VxIMU_est;
    }

    /* End of Switch: '<S75>/Switch' */
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = rtb_deltafalllimit_iz;
  }

  /* End of Switch: '<S75>/Switch2' */

  /* Sum: '<S26>/Add7' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S26>/Unit Delay4'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_i = rtb_Switch2_b0 +
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d;

  /* Lookup_n-D: '<S26>/VehicleStableTarget_mps1' */
  rtb_VxIMU_est = look1_iflf_binlc(rtb_Ax,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled61, 3U);

  /* Sum: '<S26>/Add13' */
  rtb_Ax += rtb_VxIMU_est;

  /* Sum: '<S26>/Add12' */
  rtb_Add6_p -= rtb_Ax;

  /* RelationalOperator: '<S26>/Relational Operator1' incorporates:
   *  Constant: '<S26>/Verror1'
   */
  rtb_UpperRelop_ir = (rtb_Add6_p < 0.0F);

  /* RelationalOperator: '<S26>/Relational Operator2' incorporates:
   *  UnitDelay: '<S26>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_i >=
                       rtb_UkYk1);

  /* RelationalOperator: '<S71>/Compare' incorporates:
   *  Constant: '<S71>/Constant'
   *  UnitDelay: '<S26>/Unit Delay4'
   */
  rtb_AND2 = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_i <= 0.01);

  /* Logic: '<S26>/OR' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b || rtb_AND2);

  /* Logic: '<S26>/Logical Operator5' incorporates:
   *  UnitDelay: '<S26>/Unit Delay3'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_e = (rtb_UpperRelop_ir &&
    rtb_LowerRelop1_b);

  /* Switch: '<S26>/Switch2' incorporates:
   *  Switch: '<S26>/Switch7'
   *  UnitDelay: '<S26>/Unit Delay3'
   *  UnitDelay: '<S73>/Unit Delay1'
   */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_gl) {
    /* RelationalOperator: '<S76>/LowerRelop1' incorporates:
     *  Constant: '<S26>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S26>/Unit Delay4'
     */
    rtb_Compare_am = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_i > 235.0);

    /* Switch: '<S76>/Switch2' incorporates:
     *  Constant: '<S26>/TCS_TrqRequest_Max2'
     */
    if (rtb_Compare_am) {
      VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j = 235.0;
    } else {
      /* RelationalOperator: '<S76>/UpperRelop' incorporates:
       *  Constant: '<S26>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S26>/Unit Delay4'
       */
      rtb_Compare_am = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_i < 0.0);

      /* Switch: '<S76>/Switch' incorporates:
       *  Constant: '<S26>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S26>/Unit Delay4'
       */
      if (rtb_Compare_am) {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j = 0.0;
      } else {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j =
          VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_i;
      }

      /* End of Switch: '<S76>/Switch' */
    }

    /* End of Switch: '<S76>/Switch2' */

    /* RelationalOperator: '<S77>/LowerRelop1' */
    rtb_Compare_am = (rtb_UkYk1 > VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j);

    /* Switch: '<S77>/Switch2' */
    if (!rtb_Compare_am) {
      /* RelationalOperator: '<S77>/UpperRelop' incorporates:
       *  Constant: '<S26>/TCS_TrqRequest_Min1'
       */
      rtb_Compare_am = (rtb_UkYk1 < 0.0);

      /* Switch: '<S77>/Switch' incorporates:
       *  Constant: '<S26>/TCS_TrqRequest_Min1'
       */
      if (rtb_Compare_am) {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j = 0.0;
      } else {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j = rtb_UkYk1;
      }

      /* End of Switch: '<S77>/Switch' */
    }

    /* End of Switch: '<S77>/Switch2' */
  } else {
    if (VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_e) {
      /* Switch: '<S26>/Switch7' */
      VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j = rtb_UkYk1;
    }
  }

  /* End of Switch: '<S26>/Switch2' */

  /* Switch: '<S7>/Switch3' incorporates:
   *  Constant: '<S7>/Constant6'
   *  Switch: '<S7>/Switch6'
   */
  if (Trq_CUT_final) {
    rtb_Add6_p = 0.0F;
  } else {
    if (VehCtrlMdel240918_2018b_B.TCSF_Enable_OUT != 0.0) {
      /* Switch: '<S7>/Switch6' incorporates:
       *  UnitDelay: '<S26>/Unit Delay2'
       */
      rtb_UkYk1 = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_j;
    }

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefFront1' */
    rtb_Add6_p = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240918_2018b_ConstP.pooled32,
      VehCtrlMdel240918_2018b_ConstP.pooled31, 1U);

    /* RelationalOperator: '<S22>/LowerRelop1' */
    rtb_Compare_am = (rtb_UkYk1 > rtb_Add6_p);

    /* Switch: '<S22>/Switch2' */
    if (!rtb_Compare_am) {
      /* RelationalOperator: '<S22>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      rtb_Compare_am = (rtb_UkYk1 < 0.0);

      /* Switch: '<S22>/Switch' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      if (rtb_Compare_am) {
        rtb_Add6_p = 0.0F;
      } else {
        rtb_Add6_p = (real32_T)rtb_UkYk1;
      }

      /* End of Switch: '<S22>/Switch' */
    }

    /* End of Switch: '<S22>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch3' */

  /* UnitDelay: '<S19>/Delay Input2'
   *
   * Block description for '<S19>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_VxIMU_est = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hn;

  /* Sum: '<S19>/Difference Inputs1'
   *
   * Block description for '<S19>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add6_p -= rtb_VxIMU_est;

  /* SampleTimeMath: '<S19>/sample time'
   *
   * About '<S19>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S19>/delta rise limit' */
  rtb_MaxWhlSpd_mps_n = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S59>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add6_p > rtb_MaxWhlSpd_mps_n);

  /* Switch: '<S59>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S19>/delta fall limit' */
    rtb_Ax = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S59>/UpperRelop' */
    rtb_Compare_am = (rtb_Add6_p < rtb_Ax);

    /* Switch: '<S59>/Switch' */
    if (rtb_Compare_am) {
      rtb_Add6_p = rtb_Ax;
    }

    /* End of Switch: '<S59>/Switch' */
    rtb_MaxWhlSpd_mps_n = rtb_Add6_p;
  }

  /* End of Switch: '<S59>/Switch2' */

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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hn = rtb_MaxWhlSpd_mps_n +
    rtb_VxIMU_est;
  if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hn > 20.0F) {
    TrqFR_cmd = 20.0F;
  } else if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hn < 0.0F) {
    TrqFR_cmd = 0.0F;
  } else {
    TrqFR_cmd = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_hn;
  }

  /* End of Saturate: '<S7>/Saturation2' */

  /* Product: '<S24>/Product3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 7.0 * WhlSpdRR_mps;

  /* Product: '<S24>/Product2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  WhlSpdRR_mps = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d * 9550.0;

  /* Sum: '<S24>/Add3' incorporates:
   *  Constant: '<S24>/RPM_min4'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = MCFL_ActualVelocity + 10.0;

  /* MinMax: '<S24>/Max2' incorporates:
   *  Constant: '<S24>/RPM_min5'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  rtb_Gain5 = fmax(VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d, 1.0);

  /* Product: '<S24>/Divide2' */
  WhlSpdRR_mps /= rtb_Gain5;

  /* UnitDelay: '<S37>/Delay Input2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *
   * Block description for '<S37>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d =
    VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_o;

  /* SampleTimeMath: '<S37>/sample time'
   *
   * About '<S37>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S37>/delta rise limit' incorporates:
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
   *  UnitDelay: '<S39>/Delay Input2'
   *
   * Block description for '<S39>/Delay Input2':
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

  /* RelationalOperator: '<S44>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Gain4 > rtb_Add7);

  /* Switch: '<S44>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Gain4 = rtb_Add7;
  } else {
    /* RelationalOperator: '<S44>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    rtb_Compare_am = (rtb_Gain4 < 0.0);

    /* Switch: '<S44>/Switch' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    if (rtb_Compare_am) {
      rtb_Gain4 = 0.0;
    }

    /* End of Switch: '<S44>/Switch' */
  }

  /* End of Switch: '<S44>/Switch2' */

  /* Sum: '<S10>/Add12' */
  rtb_Yk1_l = rtb_Gain4 + rtb_Add10;

  /* RelationalOperator: '<S47>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Yk1_l > rtb_Add7);

  /* Switch: '<S47>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Yk1_l = rtb_Add7;
  } else {
    /* RelationalOperator: '<S47>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    rtb_Compare_am = (rtb_Yk1_l < 0.0);

    /* Switch: '<S47>/Switch' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    if (rtb_Compare_am) {
      rtb_Yk1_l = 0.0;
    }

    /* End of Switch: '<S47>/Switch' */
  }

  /* End of Switch: '<S47>/Switch2' */

  /* Sum: '<S37>/Difference Inputs1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *
   * Block description for '<S37>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Yk1_l -= VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d;

  /* RelationalOperator: '<S52>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Yk1_l > rtb_Gain5);

  /* Switch: '<S52>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S37>/delta fall limit' */
    rtb_UkYk1 = -2000.0 * elapseTime;

    /* RelationalOperator: '<S52>/UpperRelop' */
    rtb_Compare_am = (rtb_Yk1_l < rtb_UkYk1);

    /* Switch: '<S52>/Switch' */
    if (rtb_Compare_am) {
      rtb_Yk1_l = rtb_UkYk1;
    }

    /* End of Switch: '<S52>/Switch' */
    rtb_Gain5 = rtb_Yk1_l;
  }

  /* End of Switch: '<S52>/Switch2' */

  /* Sum: '<S37>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S37>/Delay Input2'
   *
   * Block description for '<S37>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S37>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_o = rtb_Gain5 +
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d;

  /* MinMax: '<S7>/MinMax1' incorporates:
   *  UnitDelay: '<S37>/Delay Input2'
   *
   * Block description for '<S37>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = fmin(WhlSpdRR_mps, VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_o);

  /* Switch: '<S25>/Switch6' incorporates:
   *  Constant: '<S25>/Verror_Reset'
   *  UnitDelay: '<S64>/Unit Delay1'
   */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_dp) {
    rtb_Add10 = rtb_Add4_j;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S25>/Switch6' */

  /* Product: '<S25>/Product' incorporates:
   *  Constant: '<S25>/P_Gain'
   */
  rtb_Acc_POS = rtb_Add10 * 40.0F;

  /* Sum: '<S25>/Add11' */
  WhlSpdRR_mps = rtb_Yk1_l - rtb_Acc_POS;

  /* UnitDelay: '<S25>/Unit Delay5' */
  rtb_VxIMU_est = VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE_ip;

  /* Product: '<S25>/Product2' */
  rtb_VxIMU_est *= rtb_Add4_j;

  /* RelationalOperator: '<S61>/Compare' incorporates:
   *  Constant: '<S61>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_VxIMU_est <= 0.0F);

  /* UnitDelay: '<S25>/Unit Delay' */
  rtb_VxIMU_est = VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_nr;

  /* Switch: '<S25>/Switch3' incorporates:
   *  Constant: '<S25>/Verror_Reset1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_VxIMU_est = 0.0F;
  }

  /* End of Switch: '<S25>/Switch3' */

  /* Sum: '<S25>/Add2' */
  rtb_VxIMU_est += rtb_Add10;

  /* Saturate: '<S25>/Saturation2' */
  if (rtb_VxIMU_est > 400.0F) {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_nr = 400.0F;
  } else if (rtb_VxIMU_est < -100.0F) {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_nr = -100.0F;
  } else {
    VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_nr = rtb_VxIMU_est;
  }

  /* End of Saturate: '<S25>/Saturation2' */

  /* RelationalOperator: '<S69>/Compare' incorporates:
   *  UnitDelay: '<S64>/Unit Delay1'
   */
  rtb_Compare_am = VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_dp;

  /* UnitDelay: '<S63>/Delay Input1'
   *
   * Block description for '<S63>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = VehCtrlMdel240918_2018b_DW.DelayInput1_DSTATE_b;

  /* RelationalOperator: '<S63>/FixPt Relational Operator' */
  rtb_UpperRelop_ir = ((int32_T)rtb_Compare_am > (int32_T)rtb_UpperRelop_ir);

  /* Switch: '<S25>/Switch' incorporates:
   *  Constant: '<S25>/Integr_StartPoint'
   *  UnitDelay: '<S10>/Unit Delay2'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S25>/Add4' */
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = rtb_Yk1_l -
      VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_g4;
  } else {
    VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d = 0.0;
  }

  /* End of Switch: '<S25>/Switch' */

  /* Lookup_n-D: '<S25>/VehicleStableTarget_mps' */
  rtb_Add10 = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled61, 3U);

  /* Sum: '<S25>/Add5' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S25>/Add10' */
  rtb_Add10 = rtb_Add - rtb_Add10;

  /* RelationalOperator: '<S25>/Relational Operator' incorporates:
   *  Constant: '<S25>/Verror'
   */
  rtb_UpperRelop_ir = (rtb_Add10 < 0.0F);

  /* Logic: '<S25>/Logical Operator4' incorporates:
   *  UnitDelay: '<S64>/Unit Delay1'
   */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir &&
                       VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_dp);

  /* Switch: '<S25>/Switch1' incorporates:
   *  Constant: '<S25>/Trq_IReset'
   *  Constant: '<S25>/Trq_I_FF'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add10 = 20.0F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S25>/Switch1' */

  /* Sum: '<S25>/Add6' incorporates:
   *  UnitDelay: '<S10>/Unit Delay2'
   *  UnitDelay: '<S25>/Unit Delay'
   */
  rtb_Gain5 = (VehCtrlMdel240918_2018b_DW.UnitDelay_DSTATE_nr +
               VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_d) + rtb_Add10;

  /* Product: '<S25>/Product1' */
  rtb_UkYk1 = rtb_Gain5 * 10.0;

  /* RelationalOperator: '<S66>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1 > WhlSpdRR_mps);

  /* Switch: '<S66>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Gain: '<S25>/Gain3' */
    rtb_Add6_p = -rtb_Acc_POS;

    /* RelationalOperator: '<S66>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_UkYk1 < rtb_Add6_p);

    /* Switch: '<S66>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_UkYk1 = rtb_Add6_p;
    }

    /* End of Switch: '<S66>/Switch' */
    WhlSpdRR_mps = rtb_UkYk1;
  }

  /* End of Switch: '<S66>/Switch2' */

  /* Sum: '<S25>/Add7' incorporates:
   *  UnitDelay: '<S25>/Unit Delay4'
   */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_l = rtb_Acc_POS + WhlSpdRR_mps;

  /* Lookup_n-D: '<S25>/VehicleStableTarget_mps1' */
  rtb_Add10 = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240918_2018b_ConstP.pooled55,
    VehCtrlMdel240918_2018b_ConstP.pooled61, 3U);

  /* Sum: '<S25>/Add13' */
  rtb_Switch2_mn += rtb_Add10;

  /* Sum: '<S25>/Add12' */
  rtb_Add -= rtb_Switch2_mn;

  /* RelationalOperator: '<S25>/Relational Operator1' incorporates:
   *  Constant: '<S25>/Verror1'
   */
  rtb_UpperRelop_ir = (rtb_Add < 0.0F);

  /* RelationalOperator: '<S25>/Relational Operator2' incorporates:
   *  UnitDelay: '<S25>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_l >=
                       rtb_Yk1_l);

  /* RelationalOperator: '<S62>/Compare' incorporates:
   *  Constant: '<S62>/Constant'
   *  UnitDelay: '<S25>/Unit Delay4'
   */
  rtb_AND2 = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_l <= 0.01);

  /* Logic: '<S25>/OR' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b || rtb_AND2);

  /* Logic: '<S25>/Logical Operator5' */
  rtb_AND2 = (rtb_UpperRelop_ir && rtb_LowerRelop1_b);

  /* Switch: '<S25>/Switch2' incorporates:
   *  Switch: '<S25>/Switch7'
   *  UnitDelay: '<S64>/Unit Delay1'
   */
  if (VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_dp) {
    /* RelationalOperator: '<S67>/LowerRelop1' incorporates:
     *  Constant: '<S25>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S25>/Unit Delay4'
     */
    rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_l > 235.0);

    /* Switch: '<S67>/Switch2' incorporates:
     *  Constant: '<S25>/TCS_TrqRequest_Max2'
     */
    if (rtb_LowerRelop1_b) {
      VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b = 235.0;
    } else {
      /* RelationalOperator: '<S67>/UpperRelop' incorporates:
       *  Constant: '<S25>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S25>/Unit Delay4'
       */
      rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_l < 0.0);

      /* Switch: '<S67>/Switch' incorporates:
       *  Constant: '<S25>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S25>/Unit Delay4'
       */
      if (rtb_LowerRelop1_b) {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b = 0.0;
      } else {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b =
          VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_l;
      }

      /* End of Switch: '<S67>/Switch' */
    }

    /* End of Switch: '<S67>/Switch2' */

    /* RelationalOperator: '<S68>/LowerRelop1' */
    rtb_LowerRelop1_b = (rtb_Yk1_l >
                         VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b);

    /* Switch: '<S68>/Switch2' */
    if (!rtb_LowerRelop1_b) {
      /* RelationalOperator: '<S68>/UpperRelop' incorporates:
       *  Constant: '<S25>/TCS_TrqRequest_Min1'
       */
      rtb_LowerRelop1_b = (rtb_Yk1_l < 0.0);

      /* Switch: '<S68>/Switch' incorporates:
       *  Constant: '<S25>/TCS_TrqRequest_Min1'
       */
      if (rtb_LowerRelop1_b) {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b = 0.0;
      } else {
        VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b = rtb_Yk1_l;
      }

      /* End of Switch: '<S68>/Switch' */
    }

    /* End of Switch: '<S68>/Switch2' */
  } else {
    if (rtb_AND2) {
      /* Switch: '<S25>/Switch7' */
      VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b = rtb_Yk1_l;
    }
  }

  /* End of Switch: '<S25>/Switch2' */

  /* Switch: '<S7>/Switch4' incorporates:
   *  Constant: '<S7>/Constant8'
   *  Switch: '<S7>/Switch7'
   */
  if (Trq_CUT_final) {
    rtb_Add = 0.0F;
  } else {
    if (VehCtrlMdel240918_2018b_B.TCSF_Enable_OUT != 0.0) {
      /* Switch: '<S7>/Switch7' incorporates:
       *  UnitDelay: '<S25>/Unit Delay2'
       */
      rtb_Yk1_l = VehCtrlMdel240918_2018b_DW.UnitDelay2_DSTATE_b;
    }

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefFront2' */
    rtb_Add = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240918_2018b_ConstP.pooled32,
      VehCtrlMdel240918_2018b_ConstP.pooled31, 1U);

    /* RelationalOperator: '<S23>/LowerRelop1' */
    rtb_LowerRelop1_b = (rtb_Yk1_l > rtb_Add);

    /* Switch: '<S23>/Switch2' */
    if (!rtb_LowerRelop1_b) {
      /* RelationalOperator: '<S23>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      rtb_LowerRelop1_b = (rtb_Yk1_l < 0.0);

      /* Switch: '<S23>/Switch' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      if (rtb_LowerRelop1_b) {
        rtb_Add = 0.0F;
      } else {
        rtb_Add = (real32_T)rtb_Yk1_l;
      }

      /* End of Switch: '<S23>/Switch' */
    }

    /* End of Switch: '<S23>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch4' */

  /* UnitDelay: '<S20>/Delay Input2'
   *
   * Block description for '<S20>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_ib;

  /* Sum: '<S20>/Difference Inputs1'
   *
   * Block description for '<S20>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add -= rtb_Add10;

  /* SampleTimeMath: '<S20>/sample time'
   *
   * About '<S20>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S20>/delta rise limit' */
  rtb_VxIMU_est = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S60>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add > rtb_VxIMU_est);

  /* Switch: '<S60>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S20>/delta fall limit' */
    rtb_Add6_p = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S60>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_Add < rtb_Add6_p);

    /* Switch: '<S60>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_Add = rtb_Add6_p;
    }

    /* End of Switch: '<S60>/Switch' */
    rtb_VxIMU_est = rtb_Add;
  }

  /* End of Switch: '<S60>/Switch2' */

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
  VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_ib = rtb_VxIMU_est + rtb_Add10;
  if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_ib > 20.0F) {
    TrqFL_cmd = 20.0F;
  } else if (VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_ib < 0.0F) {
    TrqFL_cmd = 0.0F;
  } else {
    TrqFL_cmd = VehCtrlMdel240918_2018b_DW.DelayInput2_DSTATE_ib;
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

  /* Logic: '<S7>/Logical Operator5' */
  rtb_LowerRelop1_b = (VehCtrlMdel240918_2018b_B.MCFR_TorqueOn &&
                       VehCtrlMdel240918_2018b_B.MCFL_TorqueOn);

  /* Logic: '<S7>/Logical Operator6' */
  TroqueOn = !rtb_LowerRelop1_b;

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

  /* Update for UnitDelay: '<S27>/Unit Delay5' */
  VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE_l = rtb_deltafalllimit_n;

  /* Update for UnitDelay: '<S27>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_g = rtb_deltafalllimit_om;

  /* Update for UnitDelay: '<S83>/Delay Input1'
   *
   * Block description for '<S83>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput1_DSTATE = rtb_ignition;

  /* Update for UnitDelay: '<S25>/Unit Delay3' */
  VehCtrlMdel240918_2018b_DW.UnitDelay3_DSTATE_i = rtb_AND2;

  /* Update for UnitDelay: '<S26>/Unit Delay5' */
  VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE_i = rtb_Divide;

  /* Update for UnitDelay: '<S26>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_f = rtb_Switch2_b0;

  /* Update for UnitDelay: '<S72>/Delay Input1'
   *
   * Block description for '<S72>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput1_DSTATE_j = rtb_Compare;

  /* Update for UnitDelay: '<S25>/Unit Delay5' */
  VehCtrlMdel240918_2018b_DW.UnitDelay5_DSTATE_ip = rtb_Add4_j;

  /* Update for UnitDelay: '<S25>/Unit Delay1' */
  VehCtrlMdel240918_2018b_DW.UnitDelay1_DSTATE_g4 = rtb_Acc_POS;

  /* Update for UnitDelay: '<S63>/Delay Input1'
   *
   * Block description for '<S63>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240918_2018b_DW.DelayInput1_DSTATE_b = rtb_Compare_am;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/10ms1' */

  /* S-Function (fcncallgen): '<S5>/10ms2' incorporates:
   *  SubSystem: '<S5>/VCU2AMKMCUFL'
   */
  /* Switch: '<S326>/Switch' incorporates:
   *  Constant: '<S326>/Constant2'
   *  Constant: '<S326>/Constant3'
   */
  if (VehCtrlMdel240918_2018b_B.MCFL_TorqueOn) {
    VehCtrlMdel240918_2018b_B.MCFL_TorqueLimitP_n = 10.0;
  } else {
    VehCtrlMdel240918_2018b_B.MCFL_TorqueLimitP_n = 0.0;
  }

  /* End of Switch: '<S326>/Switch' */

  /* S-Function (scanpack): '<S326>/CAN Pack1' incorporates:
   *  Constant: '<S326>/Constant1'
   */
  /* S-Function (scanpack): '<S326>/CAN Pack1' */
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
        real64_T result = VehCtrlMdel240918_2018b_B.MCFL_TorqueLimitP_n;

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

  /* S-Function (ecucoder_canmessage): '<S326>/CANPackMessage' */

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

  /* S-Function (ec5744_cantransmitslb): '<S326>/CANTransmit' */

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
  /* Switch: '<S327>/Switch' incorporates:
   *  Constant: '<S327>/Constant2'
   *  Constant: '<S327>/Constant3'
   */
  if (VehCtrlMdel240918_2018b_B.MCFR_TorqueOn) {
    VehCtrlMdel240918_2018b_B.MCFL_TorqueLimitP = 10.0;
  } else {
    VehCtrlMdel240918_2018b_B.MCFL_TorqueLimitP = 0.0;
  }

  /* End of Switch: '<S327>/Switch' */

  /* S-Function (scanpack): '<S327>/CAN Pack1' incorporates:
   *  Constant: '<S327>/Constant'
   */
  /* S-Function (scanpack): '<S327>/CAN Pack1' */
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
        real64_T result = VehCtrlMdel240918_2018b_B.MCFL_TorqueLimitP;

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

  /* S-Function (ecucoder_canmessage): '<S327>/CANPackMessage' */

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

  /* S-Function (ec5744_cantransmitslb): '<S327>/CANTransmit' */

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
  /* Switch: '<S328>/Switch2' incorporates:
   *  Constant: '<S328>/Constant13'
   *  Constant: '<S328>/Constant17'
   *  Constant: '<S328>/Constant19'
   *  Constant: '<S328>/Constant20'
   *  Switch: '<S328>/Switch3'
   */
  if (rtb_LogicalOperator2) {
    Gear_Trs = 0.0;
    Mode_Trs = 0.0;
  } else {
    Gear_Trs = 2.0;
    Mode_Trs = 2.0;
  }

  /* End of Switch: '<S328>/Switch2' */

  /* DataTypeConversion: '<S328>/Cast To Boolean4' */
  VehCtrlMdel240918_2018b_B.CastToBoolean4 = (uint16_T)Gear_Trs;

  /* DataTypeConversion: '<S328>/Cast To Boolean6' */
  VehCtrlMdel240918_2018b_B.CastToBoolean6 = (uint16_T)Mode_Trs;

  /* DataTypeConversion: '<S328>/Data Type Conversion2' */
  VehCtrlMdel240918_2018b_B.DataTypeConversion2 = (int32_T)floor(rtb_Switch2_on);

  /* S-Function (scanpack): '<S328>/CAN Pack1' */
  /* S-Function (scanpack): '<S328>/CAN Pack1' */
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

  /* S-Function (ecucoder_canmessage): '<S328>/CANPackMessage' */

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

  /* S-Function (ec5744_cantransmitslb): '<S328>/CANTransmit' */

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
  /* DataTypeConversion: '<S329>/Cast To Single1' */
  VehCtrlMdel240918_2018b_B.CastToSingle1 = (uint16_T)rtb_CastToDouble;

  /* S-Function (ec5744_pdsslbu3): '<S329>/PowerDriverSwitch(HS)' */

  /* Set level VehCtrlMdel240918_2018b_B.aWaterPumpON for the specified power driver switch */
  ec_gpio_write(50,VehCtrlMdel240918_2018b_B.aWaterPumpON);
  ec_gpio_write(42,VehCtrlMdel240918_2018b_B.aWaterPumpON);

  /* S-Function (ec5744_pdpslbu3): '<S329>/PowerDriverPWM' incorporates:
   *  Constant: '<S329>/Constant'
   */

  /* Power driver PWM output for channel 6 */
  ec_pwm_output(6,((uint16_T)1000U),VehCtrlMdel240918_2018b_B.CastToSingle1);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms6' */

  /* S-Function (fcncallgen): '<S334>/10ms' incorporates:
   *  SubSystem: '<S334>/daq10ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S346>/CCPDAQ' */
  ccpDaq(1);

  /* End of Outputs for S-Function (fcncallgen): '<S334>/10ms' */

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
  /* S-Function (fcncallgen): '<S334>/50ms' incorporates:
   *  SubSystem: '<S334>/daq50ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S348>/CCPDAQ' */
  ccpDaq(2);

  /* End of Outputs for S-Function (fcncallgen): '<S334>/50ms' */
}

/* Model step function for TID5 */
void VehCtrlMdel240918_2018b_step5(void) /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S333>/100MS' incorporates:
   *  SubSystem: '<S333>/Function-Call Subsystem'
   */
  /* S-Function (ec5744_canreceiveslb): '<S337>/CANReceive' */

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

  /* Call the system: <S337>/Function-Call Subsystem */

  /* Output and update for function-call system: '<S337>/Function-Call Subsystem' */
  {
    uint8_T rtb_Add;
    uint8_T rtb_Compare;

    /* Outputs for Enabled SubSystem: '<S338>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S339>/Enable'
     */
    if (VehCtrlMdel240918_2018b_B.CANReceive_o2_p > 0) {
      /* RelationalOperator: '<S340>/Compare' incorporates:
       *  Constant: '<S340>/Constant'
       */
      rtb_Add = (uint8_T)(VehCtrlMdel240918_2018b_B.CANReceive_o4_i[0] == 83);

      /* RelationalOperator: '<S341>/Compare' incorporates:
       *  Constant: '<S341>/Constant'
       */
      rtb_Compare = (uint8_T)(VehCtrlMdel240918_2018b_B.CANReceive_o4_i[5] == 84);

      /* Sum: '<S339>/Add' */
      rtb_Add = (uint8_T)((uint32_T)rtb_Add + rtb_Compare);

      /* RelationalOperator: '<S342>/Compare' incorporates:
       *  Constant: '<S342>/Constant'
       */
      rtb_Compare = (uint8_T)(rtb_Add == 2);

      /* If: '<S339>/If' */
      if (rtb_Compare > 0) {
        /* Outputs for IfAction SubSystem: '<S339>/If Action Subsystem' incorporates:
         *  ActionPort: '<S343>/Action Port'
         */
        /* S-Function (ec5744_bootloaderslb): '<S343>/BootLoader' */
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

        /* S-Function (ec5744_cpuresetslb): '<S343>/CPUReset' */

        /* Perform a microcontroller reset */
        MC_ME.MCTL.R = 0X00005AF0;
        MC_ME.MCTL.R = 0X0000A50F;

        /* End of Outputs for SubSystem: '<S339>/If Action Subsystem' */
      } else {
        /* Outputs for IfAction SubSystem: '<S339>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S344>/Action Port'
         */
        /* S-Function (ec5744_cantransmitslb): '<S344>/CANTransmit' incorporates:
         *  Constant: '<S344>/Constant'
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

        /* End of Outputs for SubSystem: '<S339>/If Action Subsystem1' */
      }

      /* End of If: '<S339>/If' */
    }

    /* End of Outputs for SubSystem: '<S338>/Enabled Subsystem' */
  }

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S337>/CANReceive' */
  /* End of Outputs for S-Function (fcncallgen): '<S333>/100MS' */

  /* S-Function (fcncallgen): '<S334>/100ms' incorporates:
   *  SubSystem: '<S334>/daq100ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S345>/CCPDAQ' */
  ccpDaq(3);

  /* End of Outputs for S-Function (fcncallgen): '<S334>/100ms' */
}

/* Model step function for TID6 */
void VehCtrlMdel240918_2018b_step6(void) /* Sample time: [0.5s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S334>/500ms' incorporates:
   *  SubSystem: '<S334>/daq500ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S347>/CCPDAQ' */
  ccpDaq(4);

  /* End of Outputs for S-Function (fcncallgen): '<S334>/500ms' */

  /* S-Function (fcncallgen): '<S335>/500ms' incorporates:
   *  SubSystem: '<S335>/EEPROMOperation'
   */

  /* S-Function (ec5744_eepromoslb): '<S350>/EEPROMOperatin' */
#if defined EC_EEPROM_ENABLE

  /* Operate the EEPROM module on the MPC5744 */
  ec_flash_operation();

#endif

  /* End of Outputs for S-Function (fcncallgen): '<S335>/500ms' */
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
  /* Start for S-Function (ec5744_canreceiveslb): '<S111>/CANReceive1' incorporates:
   *  SubSystem: '<S111>/MCU_pwr'
   */
  /* Start for function-call system: '<S111>/MCU_pwr' */

  /* Start for Enabled SubSystem: '<S164>/MCU_VCUMeter1' */

  /* Start for S-Function (scanunpack): '<S166>/CAN Unpack' */

  /*-----------S-Function Block: <S166>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S164>/MCU_VCUMeter1' */
  ec_buffer_init(0,1,1,218089455);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S111>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S111>/CANReceive3' incorporates:
   *  SubSystem: '<S111>/MCU_state'
   */
  /* Start for function-call system: '<S111>/MCU_state' */

  /* Start for Enabled SubSystem: '<S165>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S171>/CAN Unpack' */

  /*-----------S-Function Block: <S171>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S165>/MCU_state' */
  ec_buffer_init(0,0,1,218089199);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S111>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S107>/CANReceive3' incorporates:
   *  SubSystem: '<S107>/ABS_BUS_state'
   */
  /* Start for function-call system: '<S107>/ABS_BUS_state' */

  /* Start for Enabled SubSystem: '<S115>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S116>/CAN Unpack1' */

  /*-----------S-Function Block: <S116>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S115>/IMU_state' */
  ec_buffer_init(1,16,0,1698);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S107>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S113>/CANReceive3' incorporates:
   *  SubSystem: '<S113>/StrWhSnis_state'
   */
  /* Start for function-call system: '<S113>/StrWhSnis_state' */

  /* Start for Enabled SubSystem: '<S183>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S184>/CAN Unpack1' */

  /*-----------S-Function Block: <S184>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S183>/IMU_state' */
  ec_buffer_init(1,7,0,330);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S113>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms4' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' incorporates:
   *  SubSystem: '<S121>/AMKMCU_state'
   */
  /* Start for function-call system: '<S121>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S123>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S126>/CAN Unpack' */

  /*-----------S-Function Block: <S126>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S123>/MCU_state' */
  ec_buffer_init(1,1,0,640);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive1' incorporates:
   *  SubSystem: '<S121>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S121>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S124>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S135>/CAN Unpack' */

  /*-----------S-Function Block: <S135>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S124>/MCU_state' */
  ec_buffer_init(1,2,0,642);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive2' incorporates:
   *  SubSystem: '<S121>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S121>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S125>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S137>/CAN Unpack' */

  /*-----------S-Function Block: <S137>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S125>/MCU_state' */
  ec_buffer_init(1,3,0,644);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive2' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' incorporates:
   *  SubSystem: '<S122>/AMKMCU_state'
   */
  /* Start for function-call system: '<S122>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S141>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S144>/CAN Unpack' */

  /*-----------S-Function Block: <S144>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S141>/MCU_state' */
  ec_buffer_init(1,4,0,641);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive1' incorporates:
   *  SubSystem: '<S122>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S122>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S142>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S152>/CAN Unpack' */

  /*-----------S-Function Block: <S152>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S142>/MCU_state' */
  ec_buffer_init(1,5,0,643);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive2' incorporates:
   *  SubSystem: '<S122>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S122>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S143>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S154>/CAN Unpack' */

  /*-----------S-Function Block: <S154>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S143>/MCU_state' */
  ec_buffer_init(1,0,0,645);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive2' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms5' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S112>/CANReceive3' incorporates:
   *  SubSystem: '<S112>/IMU_state'
   */
  /* Start for function-call system: '<S112>/IMU_state' */

  /* Start for Enabled SubSystem: '<S178>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S179>/CAN Unpack' */

  /*-----------S-Function Block: <S179>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S178>/MCU_state' */
  ec_buffer_init(1,17,0,513);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S112>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms2' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S110>/CANReceive3' */
  ec_buffer_init(0,3,1,408961267);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S110>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S326>/CANTransmit' */
  ec_buffer_init(1,8,0,386U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms2' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S327>/CANTransmit' */
  ec_buffer_init(1,9,0,387U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms4' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S328>/CANTransmit' */
  ec_buffer_init(0,8,1,146927393U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S5>/10ms6' incorporates:
   *  SubSystem: '<S5>/WP_OUTPUT'
   */
  /* Start for S-Function (ec5744_pdpslbu3): '<S329>/PowerDriverPWM' incorporates:
   *  Constant: '<S329>/Constant'
   */

  /* Initialize PWM output for channel 6 */
  SIUL2_MSCR42 = 0X02000003;           //PWM_A3

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S333>/100MS' incorporates:
   *  SubSystem: '<S333>/Function-Call Subsystem'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S337>/CANReceive' incorporates:
   *  SubSystem: '<S337>/Function-Call Subsystem'
   */
  /* Start for function-call system: '<S337>/Function-Call Subsystem' */

  /* Start for Enabled SubSystem: '<S338>/Enabled Subsystem' */
  /* Start for IfAction SubSystem: '<S339>/If Action Subsystem1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S344>/CANTransmit' incorporates:
   *  Constant: '<S344>/Constant'
   */
  ec_buffer_init(2,9,0,593U);

  /* End of Start for SubSystem: '<S339>/If Action Subsystem1' */
  /* End of Start for SubSystem: '<S338>/Enabled Subsystem' */
  ec_buffer_init(2,1,0,278);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S337>/CANReceive' */
  /* End of Start for S-Function (fcncallgen): '<S333>/100MS' */

  /* Start for S-Function (fcncallgen): '<S336>/Function-Call Generator' incorporates:
   *  SubSystem: '<S336>/CCPBackground'
   */
  /* Start for S-Function (ec5744_ccpslb): '<S351>/CCPBackground' */
  ccpInit();

  /* End of Start for S-Function (fcncallgen): '<S336>/Function-Call Generator' */

  /* Start for S-Function (ec5744_caninterruptslb1): '<S336>/ReceiveandTransmitInterrupt' incorporates:
   *  SubSystem: '<S336>/CCPReceive'
   */
  /* Start for function-call system: '<S336>/CCPReceive' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S352>/CANReceive' */
  ec_buffer_init(2,0,0,CCP_CRO_ID);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S352>/CANReceive' */
  ec_bufint_init(2,0);
  INTC_0.PSR[548].B.PRIN = 12;
  IntcIsrVectorTable[548] = (uint32_t)&ISR_FlexCAN_2_MB0;

  /* End of Start for S-Function (ec5744_caninterruptslb1): '<S336>/ReceiveandTransmitInterrupt' */

  /* Start for S-Function (ec5744_eeprombsbu3): '<Root>/EEPROMEnable' */
  Fls_Read(0xFA0010,ecflashdataold,4096);
  Fls_Read(0xFA0010,ecflashdatanew,4096);

  /* SystemInitialize for S-Function (fcncallgen): '<S3>/10ms7' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* SystemInitialize for SignalConversion generated from: '<S114>/Constant' */
  HVCUTOFF = true;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S3>/10ms7' */

  /* SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  /* InitializeConditions for UnitDelay: '<S255>/Unit Delay4' */
  VehCtrlMdel240918_2018b_DW.UnitDelay4_DSTATE_mn = 0.01F;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' */

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
  /* Enable for Chart: '<S97>/Chart2' */
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
