/*
 * Code generated for Simulink model RP20231104WLR_231105.
 *
 * FILE    : RP20231104WLR_231105.c
 *
 * VERSION : 1.1
 *
 * DATE    : Wed Sep 18 01:37:22 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#include "RP20231104WLR_231105.h"
#include "RP20231104WLR_231105_private.h"

/* #include "myinclude.h" */

/* Named constants for Chart: '<S95>/Chart1' */
#define RP20231104WLR_231105_IN_Guard  ((uint8_T)1U)
#define RP20231104WLR_231105_IN_Init   ((uint8_T)2U)
#define RP20231104WLR_231105_IN_OFF    ((uint8_T)1U)
#define RP20231104WLR_231105_IN_ON     ((uint8_T)2U)
#define RP20231104WLR_231105_IN_Ready  ((uint8_T)3U)
#define RP20231104WLR_231105_IN_Standby ((uint8_T)4U)
#define RP20231104WLR_231105_IN_Trans  ((uint8_T)5U)
#define RP20231104WLR__IN_WaitForEngine ((uint8_T)6U)

uint16 ETIMER0_CH0CHAZHI;
uint32 ETIMER0_CH0RAEDFLAG;
uint16 ETIMER1_CH0CHAZHI;
uint32 ETIMER1_CH0RAEDFLAG;
EE_Data ecflashdataold[1024];
EE_Data ecflashdatanew[1024];

/* Exported block signals */
real_T Gear_Trs;                       /* '<S69>/Switch2' */
real_T Mode_Trs;                       /* '<S69>/Switch3' */
real_T L12V_error;                     /* '<S58>/CAN Unpack' */
real_T alarm;                          /* '<S58>/CAN Unpack' */
real_T controller_ready;               /* '<S58>/CAN Unpack' */
real_T selfcheck;                      /* '<S58>/CAN Unpack' */
real_T RPM;                            /* '<S58>/CAN Unpack' */
real_T trq;                            /* '<S58>/CAN Unpack' */
real_T Spd_R;                          /* '<S58>/rpm2kph' */
real_T motor_power;                    /* '<S52>/Divide' */
real_T Wheel_spd_F;                    /* '<S24>/Gain' */
real_T VCU_SpdCmd;                     /* '<S8>/RPM_Saturation' */
uint32_T Brk_F_vol;                    /* '<S38>/Data Type Conversion1' */
uint32_T Brk_R_vol;                    /* '<S39>/Data Type Conversion1' */
uint32_T R_BrkPrs;                     /* '<S23>/1-D Lookup Table2' */
uint32_T F_BrkPrs;                     /* '<S23>/1-D Lookup Table1' */
uint32_T Acc_vol;                      /* '<S23>/Add2' */
uint32_T Acc_vol2;                     /* '<S23>/Add3' */
uint32_T Acc_POS1;                     /* '<S23>/1-D Lookup Table' */
uint32_T Acc_POS2;                     /* '<S23>/1-D Lookup Table3' */
real32_T AC_current;                   /* '<S52>/CAN Unpack' */
real32_T DC_current;                   /* '<S52>/CAN Unpack' */
real32_T MCU_Temp;                     /* '<S52>/CAN Unpack' */
real32_T motor_Temp;                   /* '<S52>/CAN Unpack' */
real32_T VCU_TrqCmd;                   /* '<S8>/Saturation' */
boolean_T VehReady;                    /* '<S95>/Chart1' */
boolean_T beeper_state;                /* '<S95>/Chart1' */
boolean_T ignition;                    /* '<S26>/Logical Operator' */
boolean_T Trq_CUT;                     /* '<S23>/Logical Operator4' */

/* Block signals (default storage) */
B_RP20231104WLR_231105_T RP20231104WLR_231105_B;

/* Block states (default storage) */
DW_RP20231104WLR_231105_T RP20231104WLR_231105_DW;

/* Real-time model */
RT_MODEL_RP20231104WLR_231105_T RP20231104WLR_231105_M_;
RT_MODEL_RP20231104WLR_231105_T *const RP20231104WLR_231105_M =
  &RP20231104WLR_231105_M_;
static void rate_monotonic_scheduler(void);

/* eTimer Ch0 ISR function */
void ISR_eTimer_CH0(void)
{
  uint16_t mea0 = 0, mea1 = 0;
  ETIMER_0.CH[0].STS.B.ICF1 = 1;
  ETIMER0_CH0RAEDFLAG= 0;
  mea0 = ETIMER_0.CH[0].CAPT1.R;
  mea1 = ETIMER_0.CH[0].CAPT1.R;
  ETIMER0_CH0CHAZHI = (uint16_t)(mea1 - mea0);
}

/* eTimer1 Ch0 ISR function */
void ISR_eTimer1_CH0(void)
{
  uint16_t mea0 = 0, mea1 = 0;
  ETIMER_1.CH[0].STS.B.ICF1 = 1;
  ETIMER1_CH0RAEDFLAG= 0;
  mea0 = ETIMER_1.CH[0].CAPT1.R;
  mea1 = ETIMER_1.CH[0].CAPT1.R;
  ETIMER1_CH0CHAZHI = (uint16_t)(mea1 - mea0);
}

uint16 ec_etimer_fr0(void)
{
  uint16 frequency = 0;
  ETIMER0_CH0RAEDFLAG++;
  if (ETIMER0_CH0RAEDFLAG>10) {
    frequency = 0;
  } else {
    if (ETIMER0_CH0CHAZHI == 0) {
      frequency = 0;
    } else {
      frequency = 312500/ETIMER0_CH0CHAZHI;
    }
  }

  return frequency;
}

uint16 ec_etimer1_fr0(void)
{
  uint16 frequency = 0;
  ETIMER1_CH0RAEDFLAG++;
  if (ETIMER1_CH0RAEDFLAG>10) {
    frequency = 0;
  } else {
    if (ETIMER1_CH0CHAZHI == 0) {
      frequency = 0;
    } else {
      frequency = 312500/ETIMER1_CH0CHAZHI;
    }
  }

  return frequency;
}

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

uint32_T look1_iu32bflftfIu32_binlc(uint32_T u0, const real32_T bp0[], const
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
  return (uint32_T)((table[iLeft + 1U] - table[iLeft]) * frac) + (uint32_T)
    table[iLeft];
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

real_T look1_binlc(real_T u0, const real_T bp0[], const real_T table[], uint32_T
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
    frac = 0.0;
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
    frac = 1.0;
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
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
  /* Call the system: <S77>/CCPReceive */
  {
    /* S-Function (ec5744_caninterruptslb1): '<S77>/ReceiveandTransmitInterrupt' */

    /* Output and update for function-call system: '<S77>/CCPReceive' */

    /* S-Function (ec5744_canreceiveslb): '<S93>/CANReceive' */

    /* Receive CAN message */
    {
      uint8 CAN2BUF0RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

      uint8 can2buf0looprx= 0;
      RP20231104WLR_231105_B.CANReceive_o3= 256;
      RP20231104WLR_231105_B.CANReceive_o5= 8;
      RP20231104WLR_231105_B.CANReceive_o2= ec_can_receive(2,0, CAN2BUF0RX);
      RP20231104WLR_231105_B.CANReceive_o4[0]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      RP20231104WLR_231105_B.CANReceive_o4[1]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      RP20231104WLR_231105_B.CANReceive_o4[2]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      RP20231104WLR_231105_B.CANReceive_o4[3]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      RP20231104WLR_231105_B.CANReceive_o4[4]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      RP20231104WLR_231105_B.CANReceive_o4[5]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      RP20231104WLR_231105_B.CANReceive_o4[6]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      RP20231104WLR_231105_B.CANReceive_o4[7]= CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
    }

    /* Nothing to do for system: <S93>/Nothing */

    /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S93>/CANReceive' */

    /* End of Outputs for S-Function (ec5744_caninterruptslb1): '<S77>/ReceiveandTransmitInterrupt' */
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
void RP20231104WLR_231105_SetEventsForThisBaseStep(boolean_T *eventFlags)
{
  /* Task runs when its counter is zero, computed via rtmStepTask macro */
  eventFlags[1] = ((boolean_T)rtmStepTask(RP20231104WLR_231105_M, 1));
  eventFlags[2] = ((boolean_T)rtmStepTask(RP20231104WLR_231105_M, 2));
  eventFlags[3] = ((boolean_T)rtmStepTask(RP20231104WLR_231105_M, 3));
  eventFlags[4] = ((boolean_T)rtmStepTask(RP20231104WLR_231105_M, 4));
  eventFlags[5] = ((boolean_T)rtmStepTask(RP20231104WLR_231105_M, 5));
  eventFlags[6] = ((boolean_T)rtmStepTask(RP20231104WLR_231105_M, 6));
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
  (RP20231104WLR_231105_M->Timing.TaskCounters.TID[1])++;
  if ((RP20231104WLR_231105_M->Timing.TaskCounters.TID[1]) > 1) {/* Sample time: [0.001s, 0.0s] */
    RP20231104WLR_231105_M->Timing.TaskCounters.TID[1] = 0;
  }

  (RP20231104WLR_231105_M->Timing.TaskCounters.TID[2])++;
  if ((RP20231104WLR_231105_M->Timing.TaskCounters.TID[2]) > 9) {/* Sample time: [0.005s, 0.0s] */
    RP20231104WLR_231105_M->Timing.TaskCounters.TID[2] = 0;
  }

  (RP20231104WLR_231105_M->Timing.TaskCounters.TID[3])++;
  if ((RP20231104WLR_231105_M->Timing.TaskCounters.TID[3]) > 19) {/* Sample time: [0.01s, 0.0s] */
    RP20231104WLR_231105_M->Timing.TaskCounters.TID[3] = 0;
  }

  (RP20231104WLR_231105_M->Timing.TaskCounters.TID[4])++;
  if ((RP20231104WLR_231105_M->Timing.TaskCounters.TID[4]) > 99) {/* Sample time: [0.05s, 0.0s] */
    RP20231104WLR_231105_M->Timing.TaskCounters.TID[4] = 0;
  }

  (RP20231104WLR_231105_M->Timing.TaskCounters.TID[5])++;
  if ((RP20231104WLR_231105_M->Timing.TaskCounters.TID[5]) > 199) {/* Sample time: [0.1s, 0.0s] */
    RP20231104WLR_231105_M->Timing.TaskCounters.TID[5] = 0;
  }

  (RP20231104WLR_231105_M->Timing.TaskCounters.TID[6])++;
  if ((RP20231104WLR_231105_M->Timing.TaskCounters.TID[6]) > 999) {/* Sample time: [0.5s, 0.0s] */
    RP20231104WLR_231105_M->Timing.TaskCounters.TID[6] = 0;
  }
}

/* Model step function for TID0 */
void RP20231104WLR_231105_step0(void)  /* Sample time: [0.0005s, 0.0s] */
{
  {                                    /* Sample time: [0.0005s, 0.0s] */
    rate_monotonic_scheduler();
  }
}

/* Model step function for TID1 */
void RP20231104WLR_231105_step1(void)  /* Sample time: [0.001s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S77>/Function-Call Generator' incorporates:
   *  SubSystem: '<S77>/CCPBackground'
   */

  /* S-Function (ec5744_ccpslb): '<S92>/CCPBackground' */
  ccpBackground();
  Lin0_Background();

  /* End of Outputs for S-Function (fcncallgen): '<S77>/Function-Call Generator' */
}

/* Model step function for TID2 */
void RP20231104WLR_231105_step2(void)  /* Sample time: [0.005s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S75>/5ms' incorporates:
   *  SubSystem: '<S75>/daq5ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S90>/CCPDAQ' */
  ccpDaq(0);

  /* End of Outputs for S-Function (fcncallgen): '<S75>/5ms' */
}

/* Model step function for TID3 */
void RP20231104WLR_231105_step3(void)  /* Sample time: [0.01s, 0.0s] */
{
  int32_T maxV;
  real_T rtb_Divide1;
  real32_T rtb_UnitDelay;
  real32_T rtb_Product;
  real32_T rtb_Product1;
  real32_T rtb_Add;
  boolean_T rtb_LogicalOperator2;
  boolean_T rtb_LogicalOperator3;
  boolean_T rtb_Compare;
  real32_T rtb_APP_POS2;
  boolean_T rtb_LogicalOperator7;
  boolean_T rtb_Compare_m;
  boolean_T rtb_LogicalOperator6;
  real_T rtb_I_Portion;
  real_T rtb_Verror_mps;
  real_T elapseTime;
  real_T rtb_Add12;
  uint16_T rtb_Water_pump_i;
  uint32_T rtb_Gain3_m;
  uint32_T rtb_Gain2;

  /* S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/Function-Call Subsystem1'
   */
  /* S-Function (ec5744_ffrslbu3): '<S24>/FrequencyRead' */
  RP20231104WLR_231105_B.FrequencyRead= ec_etimer_fr0();

  /* MinMax: '<S24>/Max1' incorporates:
   *  Constant: '<S24>/Constant3'
   */
  maxV = (int32_T)fmax(RP20231104WLR_231105_B.FrequencyRead, 1.0);
  rtb_Divide1 = maxV;

  /* Product: '<S24>/Divide' incorporates:
   *  Constant: '<S24>/Constant'
   */
  rtb_Divide1 = 24.0 / rtb_Divide1;

  /* Product: '<S24>/Divide1' incorporates:
   *  Constant: '<S24>/Constant2'
   */
  rtb_Divide1 = 1.44 / rtb_Divide1;

  /* Gain: '<S24>/Gain' */
  Wheel_spd_F = 3.6 * rtb_Divide1;

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms3' */

  /* S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* S-Function (ec5744_klmslbu3): '<S26>/KL15Monitor' */

  /* Read the the value of the key switch input(KL15) */
  RP20231104WLR_231105_B.LVMS= ec_gpio_read(107);

  /* S-Function (ec5744_swislbu3): '<S26>/SwitchInput' */

  /* Read the the value of the specified switch input */
  RP20231104WLR_231105_B.Drive_ready= ec_gpio_read(92);

  /* Logic: '<S26>/Logical Operator' */
  ignition = !RP20231104WLR_231105_B.Drive_ready;

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms2' */

  /* S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/MCU_RECIEVE'
   */
  /* S-Function (ec5744_canreceiveslb): '<S25>/CANReceive1' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF1RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf1looprx= 0;
    RP20231104WLR_231105_B.CANReceive1_o3= 218089455;
    RP20231104WLR_231105_B.CANReceive1_o5= 8;
    RP20231104WLR_231105_B.CANReceive1_o2= ec_can_receive(0,1, CAN0BUF1RX);
    RP20231104WLR_231105_B.CANReceive1_o4[0]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    RP20231104WLR_231105_B.CANReceive1_o4[1]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    RP20231104WLR_231105_B.CANReceive1_o4[2]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    RP20231104WLR_231105_B.CANReceive1_o4[3]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    RP20231104WLR_231105_B.CANReceive1_o4[4]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    RP20231104WLR_231105_B.CANReceive1_o4[5]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    RP20231104WLR_231105_B.CANReceive1_o4[6]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    RP20231104WLR_231105_B.CANReceive1_o4[7]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
  }

  /* Call the system: <S25>/MCU_pwr */

  /* Output and update for function-call system: '<S25>/MCU_pwr' */
  {
    real32_T rtb_power;

    /* Outputs for Enabled SubSystem: '<S50>/MCU_VCUMeter1' incorporates:
     *  EnablePort: '<S52>/Enable'
     */
    if (RP20231104WLR_231105_B.CANReceive1_o2 > 0) {
      /* S-Function (ecucoder_canunmessage): '<S52>/CANUnPackMessage4' */

      /*Unpack CAN message*/
      {
        uint8 canunpackloop= 0;
        RP20231104WLR_231105_B.CANUnPackMessage4_b.Length = 8;
        RP20231104WLR_231105_B.CANUnPackMessage4_b.ID =
          RP20231104WLR_231105_B.CANReceive1_o3;
        RP20231104WLR_231105_B.CANUnPackMessage4_b.Extended = 1;
        RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[canunpackloop]=
          RP20231104WLR_231105_B.CANReceive1_o4[0];
        canunpackloop++;
        RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[canunpackloop]=
          RP20231104WLR_231105_B.CANReceive1_o4[1];
        canunpackloop++;
        RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[canunpackloop]=
          RP20231104WLR_231105_B.CANReceive1_o4[2];
        canunpackloop++;
        RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[canunpackloop]=
          RP20231104WLR_231105_B.CANReceive1_o4[3];
        canunpackloop++;
        RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[canunpackloop]=
          RP20231104WLR_231105_B.CANReceive1_o4[4];
        canunpackloop++;
        RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[canunpackloop]=
          RP20231104WLR_231105_B.CANReceive1_o4[5];
        canunpackloop++;
        RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[canunpackloop]=
          RP20231104WLR_231105_B.CANReceive1_o4[6];
        canunpackloop++;
        RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[canunpackloop]=
          RP20231104WLR_231105_B.CANReceive1_o4[7];
        canunpackloop++;
      }

      /* S-Function (scanunpack): '<S52>/CAN Unpack' */
      {
        /* S-Function (scanunpack): '<S52>/CAN Unpack' */
        if ((8 == RP20231104WLR_231105_B.CANUnPackMessage4_b.Length) &&
            (RP20231104WLR_231105_B.CANUnPackMessage4_b.ID != INVALID_CAN_ID) )
        {
          if ((218089455 == RP20231104WLR_231105_B.CANUnPackMessage4_b.ID) &&
              (1U == RP20231104WLR_231105_B.CANUnPackMessage4_b.Extended) ) {
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
                real32_T outValue = 0;

                {
                  uint16_T unpackedValue = 0;

                  {
                    uint16_T tempValue = (uint16_T) (0);

                    {
                      tempValue = tempValue | (uint16_T)
                        (RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[6]);
                      tempValue = tempValue | (uint16_T)((uint16_T)
                        (RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[7]) <<
                        8);
                    }

                    unpackedValue = tempValue;
                  }

                  outValue = (real32_T) (unpackedValue);
                }

                {
                  real32_T result = (real32_T) outValue;
                  result = (result * 0.1F) + -1600.0F;
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
                real32_T outValue = 0;

                {
                  uint16_T unpackedValue = 0;

                  {
                    uint16_T tempValue = (uint16_T) (0);

                    {
                      tempValue = tempValue | (uint16_T)
                        (RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[4]);
                      tempValue = tempValue | (uint16_T)((uint16_T)
                        (RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[5]) <<
                        8);
                    }

                    unpackedValue = tempValue;
                  }

                  outValue = (real32_T) (unpackedValue);
                }

                {
                  real32_T result = (real32_T) outValue;
                  result = (result * 0.1F) + -1600.0F;
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
                real32_T outValue = 0;

                {
                  uint8_T unpackedValue = 0;

                  {
                    uint8_T tempValue = (uint8_T) (0);

                    {
                      tempValue = tempValue | (uint8_T)
                        (RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[0]);
                    }

                    unpackedValue = tempValue;
                  }

                  outValue = (real32_T) (unpackedValue);
                }

                {
                  real32_T result = (real32_T) outValue;
                  result = result + -50.0F;
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
                real32_T outValue = 0;

                {
                  uint8_T unpackedValue = 0;

                  {
                    uint8_T tempValue = (uint8_T) (0);

                    {
                      tempValue = tempValue | (uint8_T)
                        (RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[1]);
                    }

                    unpackedValue = tempValue;
                  }

                  outValue = (real32_T) (unpackedValue);
                }

                {
                  real32_T result = (real32_T) outValue;
                  result = result + -50.0F;
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
                real32_T outValue = 0;

                {
                  uint16_T unpackedValue = 0;

                  {
                    uint16_T tempValue = (uint16_T) (0);

                    {
                      tempValue = tempValue | (uint16_T)
                        (RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[2]);
                      tempValue = tempValue | (uint16_T)((uint16_T)
                        (RP20231104WLR_231105_B.CANUnPackMessage4_b.Data[3]) <<
                        8);
                    }

                    unpackedValue = tempValue;
                  }

                  outValue = (real32_T) (unpackedValue);
                }

                {
                  real32_T result = (real32_T) outValue;
                  result = result * 0.1F;
                  RP20231104WLR_231105_B.voltage = result;
                }
              }
            }
          }
        }
      }

      /* S-Function (ec5744_eepromwslb): '<S52>/AC_current' */
#if defined EC_EEPROM_ENABLE

      /* Write data for EEPROM buffer 0 */
      ecflashdatanew[0].F= AC_current;

#endif

      /* S-Function (ec5744_eepromwslb): '<S52>/DC_current' */
#if defined EC_EEPROM_ENABLE

      /* Write data for EEPROM buffer 1 */
      ecflashdatanew[1].F= DC_current;

#endif

      /* S-Function (ec5744_eepromwslb): '<S52>/MCU_temp ' */
#if defined EC_EEPROM_ENABLE

      /* Write data for EEPROM buffer 2 */
      ecflashdatanew[2].F= MCU_Temp;

#endif

      /* S-Function (ec5744_eepromwslb): '<S52>/Motor_temp' */
#if defined EC_EEPROM_ENABLE

      /* Write data for EEPROM buffer 3 */
      ecflashdatanew[3].F= motor_Temp;

#endif

      /* S-Function (ec5744_eepromwslb): '<S52>/VOL' */
#if defined EC_EEPROM_ENABLE

      /* Write data for EEPROM buffer 4 */
      ecflashdatanew[4].F= RP20231104WLR_231105_B.voltage;

#endif

      /* Product: '<S52>/Product' */
      rtb_power = RP20231104WLR_231105_B.voltage * DC_current;

      /* Product: '<S52>/Divide' incorporates:
       *  Constant: '<S52>/Constant'
       */
      motor_power = rtb_power / 1000.0;
    }

    /* End of Outputs for SubSystem: '<S50>/MCU_VCUMeter1' */
  }

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S25>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S25>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF0RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf0looprx= 0;
    RP20231104WLR_231105_B.CANReceive3_o3= 218089199;
    RP20231104WLR_231105_B.CANReceive3_o5= 8;
    RP20231104WLR_231105_B.CANReceive3_o2= ec_can_receive(0,0, CAN0BUF0RX);
    RP20231104WLR_231105_B.CANReceive3_o4[0]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    RP20231104WLR_231105_B.CANReceive3_o4[1]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    RP20231104WLR_231105_B.CANReceive3_o4[2]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    RP20231104WLR_231105_B.CANReceive3_o4[3]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    RP20231104WLR_231105_B.CANReceive3_o4[4]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    RP20231104WLR_231105_B.CANReceive3_o4[5]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    RP20231104WLR_231105_B.CANReceive3_o4[6]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    RP20231104WLR_231105_B.CANReceive3_o4[7]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
  }

  /* Call the system: <S25>/MCU_state */

  /* Output and update for function-call system: '<S25>/MCU_state' */

  /* Outputs for Enabled SubSystem: '<S51>/MCU_state' incorporates:
   *  EnablePort: '<S58>/Enable'
   */
  if (RP20231104WLR_231105_B.CANReceive3_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S58>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      RP20231104WLR_231105_B.CANUnPackMessage4.Length = 8;
      RP20231104WLR_231105_B.CANUnPackMessage4.ID =
        RP20231104WLR_231105_B.CANReceive3_o3;
      RP20231104WLR_231105_B.CANUnPackMessage4.Extended = 1;
      RP20231104WLR_231105_B.CANUnPackMessage4.Data[canunpackloop]=
        RP20231104WLR_231105_B.CANReceive3_o4[0];
      canunpackloop++;
      RP20231104WLR_231105_B.CANUnPackMessage4.Data[canunpackloop]=
        RP20231104WLR_231105_B.CANReceive3_o4[1];
      canunpackloop++;
      RP20231104WLR_231105_B.CANUnPackMessage4.Data[canunpackloop]=
        RP20231104WLR_231105_B.CANReceive3_o4[2];
      canunpackloop++;
      RP20231104WLR_231105_B.CANUnPackMessage4.Data[canunpackloop]=
        RP20231104WLR_231105_B.CANReceive3_o4[3];
      canunpackloop++;
      RP20231104WLR_231105_B.CANUnPackMessage4.Data[canunpackloop]=
        RP20231104WLR_231105_B.CANReceive3_o4[4];
      canunpackloop++;
      RP20231104WLR_231105_B.CANUnPackMessage4.Data[canunpackloop]=
        RP20231104WLR_231105_B.CANReceive3_o4[5];
      canunpackloop++;
      RP20231104WLR_231105_B.CANUnPackMessage4.Data[canunpackloop]=
        RP20231104WLR_231105_B.CANReceive3_o4[6];
      canunpackloop++;
      RP20231104WLR_231105_B.CANUnPackMessage4.Data[canunpackloop]=
        RP20231104WLR_231105_B.CANReceive3_o4[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S58>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S58>/CAN Unpack' */
      if ((8 == RP20231104WLR_231105_B.CANUnPackMessage4.Length) &&
          (RP20231104WLR_231105_B.CANUnPackMessage4.ID != INVALID_CAN_ID) ) {
        if ((218089199 == RP20231104WLR_231105_B.CANUnPackMessage4.ID) && (1U ==
             RP20231104WLR_231105_B.CANUnPackMessage4.Extended) ) {
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[5]) &
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[7]) &
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x40U)) >> 6);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                RP20231104WLR_231105_B.low_VOL = result;
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[5]) &
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[6]) &
                      (uint8_T)(0x1U));
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                RP20231104WLR_231105_B.MCU_Temp_error = result;
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[4]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                RP20231104WLR_231105_B.Mode = result;
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[6]) &
                      (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                RP20231104WLR_231105_B.motorTemp_error = result;
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                RP20231104WLR_231105_B.overCurrent = result;
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x80U)) >> 7);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                RP20231104WLR_231105_B.overpower = result;
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x10U)) >> 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                RP20231104WLR_231105_B.overvol = result;
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                RP20231104WLR_231105_B.Precharge = result;
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[5]) &
                      (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                RP20231104WLR_231105_B.Reslove_error = result;
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[6]) &
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[0]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[1]) << 8);
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
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (RP20231104WLR_231105_B.CANUnPackMessage4.Data[3]) << 8);
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

    /* Gain: '<S58>/rpm2kph' */
    Spd_R = 0.0211 * RPM;
  }

  /* End of Outputs for SubSystem: '<S51>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S25>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms4' */

  /* S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/AccBrk_BUS'
   */
  /* S-Function (ec5744_asislbu3): '<S23>/Acc1' */

  /* Read the ADC conversion result of the analog signal */
  RP20231104WLR_231105_B.Acc1= adc_read_chan(1,4);

  /* Gain: '<S23>/Gain' */
  rtb_Gain3_m = 45875U * RP20231104WLR_231105_B.Acc1;

  /* Gain: '<S23>/Gain1' incorporates:
   *  UnitDelay: '<S23>/Unit Delay'
   */
  rtb_Gain2 = 39322U * RP20231104WLR_231105_DW.UnitDelay_DSTATE_f;

  /* Sum: '<S23>/Add2' */
  Acc_vol = (rtb_Gain2 >> 1) + rtb_Gain3_m;

  /* RelationalOperator: '<S27>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol <= 32768000U);

  /* RelationalOperator: '<S28>/Compare' */
  rtb_LogicalOperator3 = (Acc_vol >= 294912000U);

  /* Logic: '<S23>/Logical Operator' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_LogicalOperator3);

  /* S-Function (ec5744_asislbu3): '<S23>/Brk_F' */

  /* Read the ADC conversion result of the analog signal */
  RP20231104WLR_231105_B.Brk_F_l= adc_read_chan(1,0);

  /* S-Function (ec5744_asislbu3): '<S23>/Brk_R' */

  /* Read the ADC conversion result of the analog signal */
  RP20231104WLR_231105_B.Brk_R_a= adc_read_chan(0,13);

  /* S-Function (ec5744_asislbu3): '<S23>/Acc2' */

  /* Read the ADC conversion result of the analog signal */
  RP20231104WLR_231105_B.Acc2= adc_read_chan(1,2);

  /* Gain: '<S23>/Gain2' */
  rtb_Gain2 = 45875U * RP20231104WLR_231105_B.Acc2;

  /* UnitDelay: '<S23>/Unit Delay1' incorporates:
   *  UnitDelay: '<S23>/Unit Delay'
   */
  RP20231104WLR_231105_DW.UnitDelay_DSTATE_f =
    RP20231104WLR_231105_DW.UnitDelay1_DSTATE_n;

  /* Gain: '<S23>/Gain3' incorporates:
   *  UnitDelay: '<S23>/Unit Delay'
   */
  rtb_Gain3_m = 39322U * RP20231104WLR_231105_DW.UnitDelay_DSTATE_f;

  /* Sum: '<S23>/Add3' */
  Acc_vol2 = (rtb_Gain3_m >> 1) + rtb_Gain2;

  /* RelationalOperator: '<S30>/Compare' */
  rtb_LogicalOperator3 = (Acc_vol2 <= 32768000U);

  /* RelationalOperator: '<S31>/Compare' */
  rtb_Compare = (Acc_vol2 >= 294912000U);

  /* Logic: '<S23>/Logical Operator1' */
  rtb_LogicalOperator3 = (rtb_LogicalOperator3 || rtb_Compare);

  /* Logic: '<S23>/Logical Operator2' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_LogicalOperator3);

  /* Lookup_n-D: '<S23>/1-D Lookup Table' */
  Acc_POS1 = look1_iu32n16bflftfIu32_binlc(Acc_vol,
    RP20231104WLR_231105_ConstP.uDLookupTable_bp01Data_o,
    RP20231104WLR_231105_ConstP.pooled11, 1U);

  /* DataTypeConversion: '<S23>/Data Type Conversion1' */
  rtb_Add = (real32_T)Acc_POS1 * 1.52587891E-5F;

  /* Lookup_n-D: '<S23>/1-D Lookup Table3' */
  Acc_POS2 = look1_iu32n16bflftfIu32_binlc(Acc_vol2,
    RP20231104WLR_231105_ConstP.uDLookupTable3_bp01Data,
    RP20231104WLR_231105_ConstP.pooled11, 1U);

  /* DataTypeConversion: '<S23>/Data Type Conversion4' */
  rtb_APP_POS2 = (real32_T)Acc_POS2 * 1.52587891E-5F;

  /* Sum: '<S23>/Add1' */
  rtb_UnitDelay = rtb_Add - rtb_APP_POS2;

  /* Abs: '<S23>/Abs' */
  rtb_UnitDelay = fabsf(rtb_UnitDelay);

  /* RelationalOperator: '<S34>/Compare' incorporates:
   *  Constant: '<S34>/Constant'
   */
  rtb_Compare = (rtb_UnitDelay > 10.0F);

  /* RelationalOperator: '<S32>/Compare' incorporates:
   *  Constant: '<S32>/Constant'
   */
  rtb_LogicalOperator3 = (rtb_Add > 100.0F);

  /* RelationalOperator: '<S33>/Compare' incorporates:
   *  Constant: '<S33>/Constant'
   */
  rtb_LogicalOperator7 = (rtb_APP_POS2 > 100.0F);

  /* Logic: '<S23>/Logical Operator3' */
  rtb_LogicalOperator3 = (rtb_LogicalOperator3 || rtb_LogicalOperator7);

  /* DataTypeConversion: '<S38>/Data Type Conversion' */
  rtb_UnitDelay = RP20231104WLR_231105_B.Brk_F_l;

  /* Product: '<S38>/Product1' incorporates:
   *  Constant: '<S38>/Constant1'
   */
  rtb_UnitDelay *= 0.7F;

  /* UnitDelay: '<S38>/Unit Delay' */
  rtb_Product1 = RP20231104WLR_231105_DW.UnitDelay_DSTATE_c;

  /* Product: '<S38>/Product' */
  rtb_Product = 0.3F * rtb_Product1;

  /* Sum: '<S38>/Add' incorporates:
   *  UnitDelay: '<S38>/Unit Delay'
   */
  RP20231104WLR_231105_DW.UnitDelay_DSTATE_c = rtb_UnitDelay + rtb_Product;

  /* DataTypeConversion: '<S38>/Data Type Conversion1' incorporates:
   *  UnitDelay: '<S38>/Unit Delay'
   */
  Brk_F_vol = (uint32_T)RP20231104WLR_231105_DW.UnitDelay_DSTATE_c;

  /* RelationalOperator: '<S35>/Compare' incorporates:
   *  Constant: '<S35>/Constant'
   */
  rtb_LogicalOperator7 = (Brk_F_vol <= 300U);

  /* RelationalOperator: '<S36>/Compare' incorporates:
   *  Constant: '<S36>/Constant'
   */
  rtb_Compare_m = (Brk_F_vol >= 4500U);

  /* Logic: '<S23>/Logical Operator5' */
  rtb_LogicalOperator7 = (rtb_LogicalOperator7 || rtb_Compare_m);

  /* DataTypeConversion: '<S39>/Data Type Conversion' */
  rtb_Product1 = RP20231104WLR_231105_B.Brk_R_a;

  /* Product: '<S39>/Product1' incorporates:
   *  Constant: '<S39>/Constant1'
   */
  rtb_Product1 *= 0.7F;

  /* UnitDelay: '<S39>/Unit Delay' */
  rtb_UnitDelay = RP20231104WLR_231105_DW.UnitDelay_DSTATE_p;

  /* Product: '<S39>/Product' */
  rtb_Product = 0.3F * rtb_UnitDelay;

  /* Sum: '<S39>/Add' */
  RP20231104WLR_231105_DW.UnitDelay_DSTATE_p = rtb_Product1 + rtb_Product;

  /* DataTypeConversion: '<S39>/Data Type Conversion1' */
  Brk_R_vol = (uint32_T)RP20231104WLR_231105_DW.UnitDelay_DSTATE_p;

  /* RelationalOperator: '<S29>/Compare' incorporates:
   *  Constant: '<S29>/Constant'
   */
  rtb_Compare_m = (Brk_R_vol >= 4500U);

  /* RelationalOperator: '<S37>/Compare' incorporates:
   *  Constant: '<S37>/Constant'
   */
  rtb_LogicalOperator6 = (Brk_R_vol <= 300U);

  /* Logic: '<S23>/Logical Operator6' */
  rtb_LogicalOperator6 = (rtb_LogicalOperator6 || rtb_Compare_m);

  /* Logic: '<S23>/Logical Operator7' */
  rtb_LogicalOperator7 = (rtb_LogicalOperator7 || rtb_LogicalOperator6);

  /* Logic: '<S23>/Logical Operator4' */
  Trq_CUT = (rtb_LogicalOperator3 || rtb_Compare || rtb_LogicalOperator2 ||
             rtb_LogicalOperator7);

  /* Lookup_n-D: '<S23>/1-D Lookup Table2' */
  R_BrkPrs = look1_iu32bflftfIu32_binlc(Brk_R_vol,
    RP20231104WLR_231105_ConstP.uDLookupTable2_bp01Data,
    RP20231104WLR_231105_ConstP.pooled14, 1U);

  /* DataTypeConversion: '<S23>/Data Type Conversion3' */
  RP20231104WLR_231105_B.Brk_R = (real32_T)R_BrkPrs;

  /* Lookup_n-D: '<S23>/1-D Lookup Table1' */
  F_BrkPrs = look1_iu32bflftfIu32_binlc(Brk_F_vol,
    RP20231104WLR_231105_ConstP.uDLookupTable1_bp01Data,
    RP20231104WLR_231105_ConstP.pooled14, 1U);

  /* DataTypeConversion: '<S23>/Data Type Conversion' */
  RP20231104WLR_231105_B.Brk_F = (real32_T)F_BrkPrs;

  /* Sum: '<S23>/Add' */
  rtb_Add += rtb_APP_POS2;

  /* Product: '<S23>/Divide' incorporates:
   *  Constant: '<S23>/Constant'
   */
  RP20231104WLR_231105_B.Acc_POS = (real32_T)(rtb_Add / 2.0);

  /* Update for UnitDelay: '<S23>/Unit Delay' */
  RP20231104WLR_231105_DW.UnitDelay_DSTATE_f = RP20231104WLR_231105_B.Acc1;

  /* Update for UnitDelay: '<S23>/Unit Delay1' */
  RP20231104WLR_231105_DW.UnitDelay1_DSTATE_n = RP20231104WLR_231105_B.Acc2;

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms1' */

  /* S-Function (fcncallgen): '<S6>/10ms7' incorporates:
   *  SubSystem: '<S6>/Drive_ready'
   */
  /* RelationalOperator: '<S98>/Compare' incorporates:
   *  Constant: '<S98>/Constant'
   */
  rtb_LogicalOperator2 = (RP20231104WLR_231105_B.Brk_F >= 450.0F);

  /* RelationalOperator: '<S97>/Compare' incorporates:
   *  Constant: '<S97>/Constant'
   */
  rtb_LogicalOperator3 = (RP20231104WLR_231105_B.Acc_POS <= 50.0F);

  /* Chart: '<S95>/Chart1' */
  rtb_Gain3_m = RP20231104WLR_231105_M->Timing.clockTick3 -
    RP20231104WLR_231105_DW.previousTicks;
  RP20231104WLR_231105_DW.previousTicks =
    RP20231104WLR_231105_M->Timing.clockTick3;
  if (RP20231104WLR_231105_DW.temporalCounter_i1 + rtb_Gain3_m <= 255U) {
    RP20231104WLR_231105_DW.temporalCounter_i1 = (uint8_T)
      (RP20231104WLR_231105_DW.temporalCounter_i1 + rtb_Gain3_m);
  } else {
    RP20231104WLR_231105_DW.temporalCounter_i1 = MAX_uint8_T;
  }

  if (RP20231104WLR_231105_DW.bitsForTID3.is_active_c1_RP20231104WLR_2311 == 0U)
  {
    RP20231104WLR_231105_DW.bitsForTID3.is_active_c1_RP20231104WLR_2311 = 1U;
    RP20231104WLR_231105_DW.bitsForTID3.is_active_VehStat = 1U;
    RP20231104WLR_231105_DW.bitsForTID3.is_VehStat = 2U;
    RP20231104WLR_231105_DW.temporalCounter_i1 = 0U;
    RP20231104WLR_231105_DW.bitsForTID3.is_active_BeeperStat = 1U;
    RP20231104WLR_231105_DW.bitsForTID3.is_BeeperStat = 1U;
    RP20231104WLR_231105_DW.bitsForTID3.is_active_Output = 1U;
  } else {
    if (RP20231104WLR_231105_DW.bitsForTID3.is_active_VehStat != 0U) {
      switch (RP20231104WLR_231105_DW.bitsForTID3.is_VehStat) {
       case RP20231104WLR_231105_IN_Guard:
        if (RP20231104WLR_231105_DW.temporalCounter_i1 >= 25U) {
          if ((RP20231104WLR_231105_DW.bitsForTID3.is_active_BeeperStat != 0U) &&
              (RP20231104WLR_231105_DW.bitsForTID3.is_BeeperStat ==
               RP20231104WLR_231105_IN_OFF)) {
            RP20231104WLR_231105_DW.bitsForTID3.is_BeeperStat = 2U;
          }

          RP20231104WLR_231105_DW.bitsForTID3.is_VehStat = 5U;
          RP20231104WLR_231105_DW.temporalCounter_i1 = 0U;
        } else {
          rtb_LogicalOperator2 = ((!ignition) || (!rtb_LogicalOperator2) ||
            (!rtb_LogicalOperator3));
          if (rtb_LogicalOperator2) {
            RP20231104WLR_231105_DW.bitsForTID3.is_VehStat = 4U;
          }
        }
        break;

       case RP20231104WLR_231105_IN_Init:
        if (RP20231104WLR_231105_DW.temporalCounter_i1 >= 100U) {
          RP20231104WLR_231105_DW.bitsForTID3.is_VehStat = 6U;
        }
        break;

       case RP20231104WLR_231105_IN_Ready:
        break;

       case RP20231104WLR_231105_IN_Standby:
        rtb_LogicalOperator2 = (ignition && rtb_LogicalOperator2 &&
          rtb_LogicalOperator3);
        if (rtb_LogicalOperator2) {
          RP20231104WLR_231105_DW.bitsForTID3.is_VehStat = 1U;
          RP20231104WLR_231105_DW.temporalCounter_i1 = 0U;
        }
        break;

       case RP20231104WLR_231105_IN_Trans:
        if (RP20231104WLR_231105_DW.temporalCounter_i1 >= 250U) {
          if ((RP20231104WLR_231105_DW.bitsForTID3.is_active_BeeperStat != 0U) &&
              (RP20231104WLR_231105_DW.bitsForTID3.is_BeeperStat ==
               RP20231104WLR_231105_IN_ON)) {
            RP20231104WLR_231105_DW.bitsForTID3.is_BeeperStat = 1U;
          }

          RP20231104WLR_231105_DW.bitsForTID3.is_VehStat = 3U;
        }
        break;

       case RP20231104WLR__IN_WaitForEngine:
        RP20231104WLR_231105_DW.bitsForTID3.is_VehStat = 4U;
        break;
      }
    }

    if (RP20231104WLR_231105_DW.bitsForTID3.is_active_Output != 0U) {
      VehReady = (RP20231104WLR_231105_DW.bitsForTID3.is_VehStat ==
                  RP20231104WLR_231105_IN_Ready);
      beeper_state = (RP20231104WLR_231105_DW.bitsForTID3.is_BeeperStat ==
                      RP20231104WLR_231105_IN_ON);
    }
  }

  /* End of Chart: '<S95>/Chart1' */
  /* End of Outputs for S-Function (fcncallgen): '<S6>/10ms7' */

  /* S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Beeper'
   */
  /* S-Function (ec5744_pdsslbu3): '<S67>/PowerDriverSwitch(HS)' */

  /* Set level beeper_state for the specified power driver switch */
  ec_gpio_write(83,beeper_state);

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms1' */

  /* S-Function (fcncallgen): '<S1>/10ms1' incorporates:
   *  SubSystem: '<S1>/MoTrqReq'
   */
  if (RP20231104WLR_231105_DW.MoTrqReq_RESET_ELAPS_T) {
    rtb_Gain3_m = 0U;
  } else {
    rtb_Gain3_m = RP20231104WLR_231105_M->Timing.clockTick3 -
      RP20231104WLR_231105_DW.MoTrqReq_PREV_T;
  }

  RP20231104WLR_231105_DW.MoTrqReq_PREV_T =
    RP20231104WLR_231105_M->Timing.clockTick3;
  RP20231104WLR_231105_DW.MoTrqReq_RESET_ELAPS_T = false;

  /* SampleTimeMath: '<S12>/sample time'
   *
   * About '<S12>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)rtb_Gain3_m * 0.01;

  /* Product: '<S12>/delta rise limit' */
  rtb_Product1 = (real32_T)(25000.0 * elapseTime);

  /* Logic: '<S8>/Logical Operator1' */
  rtb_LogicalOperator2 = !VehReady;

  /* Logic: '<S8>/Logical Operator' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || Trq_CUT);

  /* DataTypeConversion: '<S8>/Data Type Conversion' */
  rtb_Add = (real32_T)RPM;

  /* Lookup_n-D: '<S8>/2-D Lookup Table' */
  rtb_APP_POS2 = look2_iflf_binlx(RP20231104WLR_231105_B.Acc_POS, rtb_Add,
    RP20231104WLR_231105_ConstP.uDLookupTable_bp01Data,
    RP20231104WLR_231105_ConstP.uDLookupTable_bp02Data,
    RP20231104WLR_231105_ConstP.uDLookupTable_tableData,
    RP20231104WLR_231105_ConstP.uDLookupTable_maxIndex, 11U);

  /* Sum: '<S11>/Add1' incorporates:
   *  Constant: '<S11>/RPM_min'
   */
  rtb_UnitDelay = rtb_Add + 10.0F;

  /* MinMax: '<S11>/Max' incorporates:
   *  Constant: '<S11>/RPM_min1'
   */
  rtb_UnitDelay = fmaxf(rtb_UnitDelay, 1.0F);

  /* Product: '<S11>/Divide' */
  rtb_UnitDelay = 668500.0F / rtb_UnitDelay;

  /* MinMax: '<S8>/Min' */
  rtb_APP_POS2 = fminf(rtb_APP_POS2, rtb_UnitDelay);

  /* Switch: '<S8>/Switch' incorporates:
   *  Constant: '<S8>/Constant1'
   */
  if (rtb_LogicalOperator2) {
    rtb_Product = 0.0F;
  } else {
    /* Gain: '<S8>/Gain' */
    rtb_Product = 3.84615374F * rtb_APP_POS2;

    /* Lookup_n-D: '<S8>/BrakeCompensateCoef' */
    rtb_UnitDelay = look1_iflf_binlc(RP20231104WLR_231105_B.Brk_F,
      RP20231104WLR_231105_ConstP.BrakeCompensateCoef_bp01Data,
      RP20231104WLR_231105_ConstP.BrakeCompensateCoef_tableData, 1U);

    /* RelationalOperator: '<S13>/LowerRelop1' */
    rtb_LogicalOperator2 = (rtb_Product > rtb_UnitDelay);

    /* Switch: '<S13>/Switch2' */
    if (rtb_LogicalOperator2) {
      rtb_Product = rtb_UnitDelay;
    } else {
      /* RelationalOperator: '<S13>/UpperRelop' incorporates:
       *  Constant: '<S8>/Constant3'
       */
      rtb_LogicalOperator2 = (rtb_Product < 0.0F);

      /* Switch: '<S13>/Switch' incorporates:
       *  Constant: '<S8>/Constant3'
       */
      if (rtb_LogicalOperator2) {
        rtb_Product = 0.0F;
      }

      /* End of Switch: '<S13>/Switch' */
    }

    /* End of Switch: '<S13>/Switch2' */
  }

  /* End of Switch: '<S8>/Switch' */

  /* UnitDelay: '<S12>/Delay Input2'
   *
   * Block description for '<S12>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UnitDelay = RP20231104WLR_231105_DW.DelayInput2_DSTATE;

  /* Sum: '<S12>/Difference Inputs1'
   *
   * Block description for '<S12>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Product -= rtb_UnitDelay;

  /* RelationalOperator: '<S15>/LowerRelop1' */
  rtb_LogicalOperator2 = (rtb_Product > rtb_Product1);

  /* Switch: '<S15>/Switch2' */
  if (!rtb_LogicalOperator2) {
    /* Product: '<S12>/delta fall limit' */
    rtb_Product1 = (real32_T)(-25000.0 * elapseTime);

    /* RelationalOperator: '<S15>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Product < rtb_Product1);

    /* Switch: '<S15>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Product = rtb_Product1;
    }

    /* End of Switch: '<S15>/Switch' */
    rtb_Product1 = rtb_Product;
  }

  /* End of Switch: '<S15>/Switch2' */

  /* Saturate: '<S8>/Saturation' incorporates:
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
  RP20231104WLR_231105_DW.DelayInput2_DSTATE = rtb_Product1 + rtb_UnitDelay;
  if (RP20231104WLR_231105_DW.DelayInput2_DSTATE > 1000.0F) {
    VCU_TrqCmd = 1000.0F;
  } else if (RP20231104WLR_231105_DW.DelayInput2_DSTATE < -1000.0F) {
    VCU_TrqCmd = -1000.0F;
  } else {
    VCU_TrqCmd = RP20231104WLR_231105_DW.DelayInput2_DSTATE;
  }

  /* End of Saturate: '<S8>/Saturation' */

  /* S-Function (ec5744_eepromwslb): '<S8>/trq_cmd' */
#if defined EC_EEPROM_ENABLE

  /* Write data for EEPROM buffer 6 */
  ecflashdatanew[6].F= VCU_TrqCmd;

#endif

  /* UnitDelay: '<S17>/Unit Delay1' */
  rtb_LogicalOperator2 = RP20231104WLR_231105_DW.UnitDelay1_DSTATE_j;

  /* Sum: '<S14>/Add' */
  elapseTime = Wheel_spd_F + Wheel_spd_F;

  /* Gain: '<S14>/Gain' */
  elapseTime *= 0.5;

  /* Gain: '<S14>/Gain1' */
  elapseTime *= 0.27777777777777779;

  /* Saturate: '<S14>/Saturation' */
  if (elapseTime > 40.0) {
    elapseTime = 40.0;
  } else {
    if (elapseTime < 0.0) {
      elapseTime = 0.0;
    }
  }

  /* End of Saturate: '<S14>/Saturation' */

  /* Lookup_n-D: '<S14>/VehSpd_SlipTarget_mps' */
  rtb_Verror_mps = look1_binlc(elapseTime, RP20231104WLR_231105_ConstP.pooled4,
    RP20231104WLR_231105_ConstP.VehSpd_SlipTarget_mps_tableData, 3U);

  /* Sum: '<S14>/Add9' */
  rtb_Verror_mps += elapseTime;

  /* Gain: '<S14>/Gain4' */
  rtb_Add12 = 0.27777777777777779 * Spd_R;

  /* Saturate: '<S14>/Saturation1' */
  if (rtb_Add12 > 50.0) {
    rtb_Add12 = 50.0;
  } else {
    if (rtb_Add12 < 0.0) {
      rtb_Add12 = 0.0;
    }
  }

  /* End of Saturate: '<S14>/Saturation1' */

  /* Sum: '<S14>/Add1' */
  rtb_Verror_mps -= rtb_Add12;

  /* RelationalOperator: '<S14>/Relational Operator7' incorporates:
   *  Constant: '<S14>/Cal_DeltaV_mps'
   */
  rtb_LogicalOperator3 = (rtb_Verror_mps < 0.0);

  /* Logic: '<S17>/Logical Operator4' */
  rtb_LogicalOperator2 = ((!rtb_LogicalOperator2) && (!rtb_LogicalOperator3));

  /* Logic: '<S14>/Logical Operator2' */
  rtb_LogicalOperator3 = !rtb_LogicalOperator3;

  /* Saturate: '<S8>/RPM_Saturation' incorporates:
   *  UnitDelay: '<S14>/Unit Delay4'
   */
  VCU_SpdCmd = RP20231104WLR_231105_DW.UnitDelay4_DSTATE;

  /* RelationalOperator: '<S14>/Relational Operator8' incorporates:
   *  Constant: '<S14>/Cal_DeltaV_mps1'
   */
  rtb_Compare = (VCU_SpdCmd > 235.0);

  /* Logic: '<S14>/Logical Operator1' */
  rtb_LogicalOperator3 = (rtb_LogicalOperator3 && rtb_Compare);

  /* Switch: '<S18>/Switch6' incorporates:
   *  Constant: '<S18>/Reset'
   */
  if (rtb_LogicalOperator3) {
    /* Sum: '<S18>/Add10' incorporates:
     *  Constant: '<S18>/Steptime'
     */
    rtb_UnitDelay = RP20231104WLR_231105_DW.UnitDelay1_DSTATE_d + 0.01F;
  } else {
    rtb_UnitDelay = 0.0F;
  }

  /* End of Switch: '<S18>/Switch6' */

  /* MinMax: '<S18>/Min' incorporates:
   *  Constant: '<S14>/ResetDelay'
   */
  RP20231104WLR_231105_DW.UnitDelay1_DSTATE_d = fminf(rtb_UnitDelay, 0.1F);

  /* RelationalOperator: '<S18>/Relational Operator9' incorporates:
   *  Constant: '<S14>/ResetDelay'
   *  UnitDelay: '<S18>/Unit Delay1'
   */
  rtb_Compare = (RP20231104WLR_231105_DW.UnitDelay1_DSTATE_d >= 0.1F);

  /* UnitDelay: '<S14>/Unit Delay3' */
  rtb_LogicalOperator3 = RP20231104WLR_231105_DW.UnitDelay3_DSTATE;

  /* Logic: '<S14>/Logical Operator3' */
  rtb_Compare = (rtb_Compare || rtb_LogicalOperator3);

  /* Logic: '<S17>/Logical Operator5' */
  RP20231104WLR_231105_B.LogicalOperator5 = ((!rtb_LogicalOperator2) &&
    (!rtb_Compare));

  /* RelationalOperator: '<S22>/Compare' */
  rtb_LogicalOperator2 = RP20231104WLR_231105_B.LogicalOperator5;

  /* UnitDelay: '<S16>/Delay Input1'
   *
   * Block description for '<S16>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_Compare = RP20231104WLR_231105_DW.DelayInput1_DSTATE;

  /* RelationalOperator: '<S16>/FixPt Relational Operator' */
  rtb_Compare = ((int32_T)rtb_LogicalOperator2 > (int32_T)rtb_Compare);

  /* Switch: '<S14>/Switch' */
  if (rtb_Compare) {
    /* Saturate: '<S8>/RPM_Saturation' incorporates:
     *  Sum: '<S14>/Add4'
     */
    VCU_SpdCmd = rtb_APP_POS2 - RP20231104WLR_231105_DW.UnitDelay1_DSTATE;
  } else {
    /* Saturate: '<S8>/RPM_Saturation' incorporates:
     *  Constant: '<S14>/Integr_StartPoint'
     */
    VCU_SpdCmd = 0.0;
  }

  /* End of Switch: '<S14>/Switch' */

  /* Switch: '<S14>/Switch6' incorporates:
   *  Constant: '<S14>/Verror_Reset'
   */
  if (!RP20231104WLR_231105_B.LogicalOperator5) {
    rtb_Verror_mps = 0.0;
  }

  /* End of Switch: '<S14>/Switch6' */

  /* UnitDelay: '<S14>/Unit Delay' */
  rtb_Divide1 = RP20231104WLR_231105_DW.UnitDelay_DSTATE;

  /* Sum: '<S14>/Add2' */
  rtb_Divide1 += rtb_Verror_mps;

  /* Saturate: '<S14>/Saturation2' */
  if (rtb_Divide1 > 400.0) {
    RP20231104WLR_231105_DW.UnitDelay_DSTATE = 400.0;
  } else if (rtb_Divide1 < -100.0) {
    RP20231104WLR_231105_DW.UnitDelay_DSTATE = -100.0;
  } else {
    RP20231104WLR_231105_DW.UnitDelay_DSTATE = rtb_Divide1;
  }

  /* End of Saturate: '<S14>/Saturation2' */

  /* Lookup_n-D: '<S14>/VehicleStableTarget_mps' */
  rtb_Divide1 = look1_binlc(elapseTime, RP20231104WLR_231105_ConstP.pooled4,
    RP20231104WLR_231105_ConstP.pooled5, 3U);

  /* Sum: '<S14>/Add5' */
  rtb_Divide1 += elapseTime;

  /* Sum: '<S14>/Add10' */
  rtb_Divide1 = rtb_Add12 - rtb_Divide1;

  /* RelationalOperator: '<S14>/Relational Operator' incorporates:
   *  Constant: '<S14>/Verror'
   */
  rtb_Compare = (rtb_Divide1 < 0.0);

  /* Logic: '<S14>/Logical Operator4' */
  rtb_Compare = (rtb_Compare && RP20231104WLR_231105_B.LogicalOperator5);

  /* Switch: '<S14>/Switch1' incorporates:
   *  Constant: '<S14>/Trq_IReset'
   *  Constant: '<S14>/Trq_I_FF'
   */
  if (rtb_Compare) {
    rtb_UnitDelay = 20.0F;
  } else {
    rtb_UnitDelay = 0.0F;
  }

  /* End of Switch: '<S14>/Switch1' */

  /* Sum: '<S14>/Add6' incorporates:
   *  UnitDelay: '<S14>/Unit Delay'
   */
  rtb_Divide1 = (RP20231104WLR_231105_DW.UnitDelay_DSTATE + VCU_SpdCmd) +
    rtb_UnitDelay;

  /* Product: '<S14>/Product1' */
  rtb_I_Portion = rtb_Divide1 * 10.0;

  /* Product: '<S14>/Product' incorporates:
   *  UnitDelay: '<S14>/Unit Delay1'
   */
  RP20231104WLR_231105_DW.UnitDelay1_DSTATE = rtb_Verror_mps * 40.0;

  /* Sum: '<S14>/Add11' incorporates:
   *  UnitDelay: '<S14>/Unit Delay1'
   */
  rtb_Divide1 = rtb_APP_POS2 - RP20231104WLR_231105_DW.UnitDelay1_DSTATE;

  /* RelationalOperator: '<S19>/LowerRelop1' */
  rtb_Compare = (rtb_I_Portion > rtb_Divide1);

  /* Switch: '<S19>/Switch2' */
  if (!rtb_Compare) {
    /* Gain: '<S14>/Gain3' incorporates:
     *  UnitDelay: '<S14>/Unit Delay1'
     */
    rtb_Divide1 = -RP20231104WLR_231105_DW.UnitDelay1_DSTATE;

    /* RelationalOperator: '<S19>/UpperRelop' */
    rtb_LogicalOperator3 = (rtb_I_Portion < rtb_Divide1);

    /* Switch: '<S19>/Switch' */
    if (rtb_LogicalOperator3) {
      rtb_I_Portion = rtb_Divide1;
    }

    /* End of Switch: '<S19>/Switch' */
    rtb_Divide1 = rtb_I_Portion;
  }

  /* End of Switch: '<S19>/Switch2' */

  /* Sum: '<S14>/Add7' incorporates:
   *  UnitDelay: '<S14>/Unit Delay1'
   *  UnitDelay: '<S14>/Unit Delay4'
   */
  RP20231104WLR_231105_DW.UnitDelay4_DSTATE =
    RP20231104WLR_231105_DW.UnitDelay1_DSTATE + rtb_Divide1;

  /* Lookup_n-D: '<S14>/VehicleStableTarget_mps1' */
  rtb_Divide1 = look1_binlc(elapseTime, RP20231104WLR_231105_ConstP.pooled4,
    RP20231104WLR_231105_ConstP.pooled5, 3U);

  /* Sum: '<S14>/Add13' */
  elapseTime += rtb_Divide1;

  /* Sum: '<S14>/Add12' */
  rtb_Add12 -= elapseTime;

  /* RelationalOperator: '<S14>/Relational Operator1' incorporates:
   *  Constant: '<S14>/Verror1'
   */
  rtb_Compare = (rtb_Add12 < 0.0);

  /* RelationalOperator: '<S14>/Relational Operator2' incorporates:
   *  UnitDelay: '<S14>/Unit Delay4'
   */
  rtb_LogicalOperator3 = (RP20231104WLR_231105_DW.UnitDelay4_DSTATE >
    rtb_APP_POS2);

  /* Logic: '<S14>/Logical Operator5' incorporates:
   *  UnitDelay: '<S14>/Unit Delay3'
   */
  RP20231104WLR_231105_DW.UnitDelay3_DSTATE = (rtb_Compare &&
    rtb_LogicalOperator3);

  /* Switch: '<S14>/Switch2' incorporates:
   *  Switch: '<S14>/Switch7'
   *  UnitDelay: '<S14>/Unit Delay3'
   */
  if (RP20231104WLR_231105_B.LogicalOperator5) {
    /* RelationalOperator: '<S20>/LowerRelop1' incorporates:
     *  Constant: '<S14>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S14>/Unit Delay4'
     */
    rtb_LogicalOperator3 = (RP20231104WLR_231105_DW.UnitDelay4_DSTATE > 235.0);

    /* Switch: '<S20>/Switch2' incorporates:
     *  Constant: '<S14>/TCS_TrqRequest_Max2'
     */
    if (rtb_LogicalOperator3) {
      rtb_Divide1 = 235.0;
    } else {
      /* RelationalOperator: '<S20>/UpperRelop' incorporates:
       *  Constant: '<S14>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S14>/Unit Delay4'
       */
      rtb_LogicalOperator3 = (RP20231104WLR_231105_DW.UnitDelay4_DSTATE < 0.0);

      /* Switch: '<S20>/Switch' incorporates:
       *  Constant: '<S14>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S14>/Unit Delay4'
       */
      if (rtb_LogicalOperator3) {
        rtb_Divide1 = 0.0;
      } else {
        rtb_Divide1 = RP20231104WLR_231105_DW.UnitDelay4_DSTATE;
      }

      /* End of Switch: '<S20>/Switch' */
    }

    /* End of Switch: '<S20>/Switch2' */

    /* RelationalOperator: '<S21>/LowerRelop1' */
    rtb_LogicalOperator3 = (rtb_APP_POS2 > rtb_Divide1);

    /* Switch: '<S21>/Switch2' */
    if (rtb_LogicalOperator3) {
      rtb_APP_POS2 = (real32_T)rtb_Divide1;
    } else {
      /* RelationalOperator: '<S21>/UpperRelop' incorporates:
       *  Constant: '<S14>/TCS_TrqRequest_Min1'
       */
      rtb_LogicalOperator3 = (rtb_APP_POS2 < 0.0F);

      /* Switch: '<S21>/Switch' incorporates:
       *  Constant: '<S14>/TCS_TrqRequest_Min1'
       */
      if (rtb_LogicalOperator3) {
        rtb_APP_POS2 = 0.0F;
      }

      /* End of Switch: '<S21>/Switch' */
    }

    /* End of Switch: '<S21>/Switch2' */
    RP20231104WLR_231105_B.TCS_TrqRequestFinal_Nm = rtb_APP_POS2;
  } else {
    if (RP20231104WLR_231105_DW.UnitDelay3_DSTATE) {
      /* Switch: '<S14>/Switch7' */
      RP20231104WLR_231105_DW.UnitDelay2_DSTATE = rtb_APP_POS2;
    }

    RP20231104WLR_231105_B.TCS_TrqRequestFinal_Nm =
      RP20231104WLR_231105_DW.UnitDelay2_DSTATE;
  }

  /* End of Switch: '<S14>/Switch2' */

  /* Switch: '<S8>/Switch1' */
  rtb_Divide1 = rtb_Add;

  /* SampleTimeMath: '<S8>/Weighted Sample Time'
   *
   * About '<S8>/Weighted Sample Time':
   *  y = u * K where K = ( w * Ts )
   */
  elapseTime = (real_T)rtb_Gain3_m * 0.01;

  /* Saturate: '<S8>/RPM_Saturation' incorporates:
   *  SampleTimeMath: '<S8>/Weighted Sample Time'
   *  Sum: '<S8>/Add'
   *
   * About '<S8>/Weighted Sample Time':
   *  y = u * K where K = ( w * Ts )
   */
  VCU_SpdCmd = 420000.0 * elapseTime;
  VCU_SpdCmd += rtb_Divide1;
  if (VCU_SpdCmd > 5000.0) {
    VCU_SpdCmd = 5000.0;
  } else {
    if (VCU_SpdCmd < -50.0) {
      VCU_SpdCmd = -50.0;
    }
  }

  /* Update for UnitDelay: '<S17>/Unit Delay1' */
  RP20231104WLR_231105_DW.UnitDelay1_DSTATE_j =
    RP20231104WLR_231105_B.LogicalOperator5;

  /* Update for UnitDelay: '<S16>/Delay Input1'
   *
   * Block description for '<S16>/Delay Input1':
   *
   *  Store in Global RAM
   */
  RP20231104WLR_231105_DW.DelayInput1_DSTATE = rtb_LogicalOperator2;

  /* Update for UnitDelay: '<S14>/Unit Delay2' */
  RP20231104WLR_231105_DW.UnitDelay2_DSTATE =
    RP20231104WLR_231105_B.TCS_TrqRequestFinal_Nm;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/10ms1' */

  /* S-Function (fcncallgen): '<S4>/10ms' incorporates:
   *  SubSystem: '<S4>/VCU2CLUSTER'
   */
  /* S-Function (scanpack): '<S68>/CAN Pack' */
  /* S-Function (scanpack): '<S68>/CAN Pack' */
  RP20231104WLR_231105_B.CANPack.ID = 291U;
  RP20231104WLR_231105_B.CANPack.Length = 8U;
  RP20231104WLR_231105_B.CANPack.Extended = 0U;
  RP20231104WLR_231105_B.CANPack.Remote = 0;
  RP20231104WLR_231105_B.CANPack.Data[0] = 0;
  RP20231104WLR_231105_B.CANPack.Data[1] = 0;
  RP20231104WLR_231105_B.CANPack.Data[2] = 0;
  RP20231104WLR_231105_B.CANPack.Data[3] = 0;
  RP20231104WLR_231105_B.CANPack.Data[4] = 0;
  RP20231104WLR_231105_B.CANPack.Data[5] = 0;
  RP20231104WLR_231105_B.CANPack.Data[6] = 0;
  RP20231104WLR_231105_B.CANPack.Data[7] = 0;

  {
    /* --------------- START Packing signal 0 ------------------
     *  startBit                = 56
     *  length                  = 8
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = SIGNED
     *  factor                  = 30.0
     *  offset                  = 100.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real32_T outValue = 0;

      {
        real32_T result = RP20231104WLR_231105_B.Brk_F;

        /* full scaling operation */
        result = (result - 100.0F) * (1 / 30.0F);

        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        int8_T packedValue;
        int32_T scaledValue;
        if (outValue > 2147483647.0) {
          scaledValue = 2147483647;
        } else if (outValue < -2147483648.0) {
          scaledValue = -2147483647 - 1;
        } else {
          scaledValue = (int32_T) outValue;
        }

        if (scaledValue > (int32_T) (127)) {
          packedValue = 127;
        } else if (scaledValue < (int32_T)((-(127)-1))) {
          packedValue = (-(127)-1);
        } else {
          packedValue = (int8_T) (scaledValue);
        }

        {
          uint8_T* tempValuePtr = (uint8_T*)&packedValue;
          uint8_T tempValue = *tempValuePtr;

          {
            RP20231104WLR_231105_B.CANPack.Data[7] =
              RP20231104WLR_231105_B.CANPack.Data[7] | (uint8_T)(tempValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 1 ------------------
     *  startBit                = 40
     *  length                  = 8
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
        real64_T result = RP20231104WLR_231105_B.TCS_TrqRequestFinal_Nm;

        /* no scaling required */
        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        uint8_T packedValue;
        if (outValue > (real64_T)(255)) {
          packedValue = (uint8_T) 255;
        } else if (outValue < (real64_T)(0)) {
          packedValue = (uint8_T) 0;
        } else {
          packedValue = (uint8_T) (outValue);
        }

        {
          {
            RP20231104WLR_231105_B.CANPack.Data[5] =
              RP20231104WLR_231105_B.CANPack.Data[5] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 2 ------------------
     *  startBit                = 32
     *  length                  = 8
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = SIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (RP20231104WLR_231105_B.LogicalOperator5);

        /* no scaling required */
        packingValue = result;
      }

      {
        int8_T packedValue;
        int32_T scaledValue;
        scaledValue = (int32_T) packingValue;
        if (scaledValue > (int32_T) (127)) {
          packedValue = 127;
        } else if (scaledValue < (int32_T)((-(127)-1))) {
          packedValue = (-(127)-1);
        } else {
          packedValue = (int8_T) (scaledValue);
        }

        {
          uint8_T* tempValuePtr = (uint8_T*)&packedValue;
          uint8_T tempValue = *tempValuePtr;

          {
            RP20231104WLR_231105_B.CANPack.Data[4] =
              RP20231104WLR_231105_B.CANPack.Data[4] | (uint8_T)(tempValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 3 ------------------
     *  startBit                = 48
     *  length                  = 8
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = SIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real64_T outValue = 0;

      {
        real64_T result = trq;

        /* no scaling required */
        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        int8_T packedValue;
        int32_T scaledValue;
        if (outValue > 2147483647.0) {
          scaledValue = 2147483647;
        } else if (outValue < -2147483648.0) {
          scaledValue = -2147483647 - 1;
        } else {
          scaledValue = (int32_T) outValue;
        }

        if (scaledValue > (int32_T) (127)) {
          packedValue = 127;
        } else if (scaledValue < (int32_T)((-(127)-1))) {
          packedValue = (-(127)-1);
        } else {
          packedValue = (int8_T) (scaledValue);
        }

        {
          uint8_T* tempValuePtr = (uint8_T*)&packedValue;
          uint8_T tempValue = *tempValuePtr;

          {
            RP20231104WLR_231105_B.CANPack.Data[6] =
              RP20231104WLR_231105_B.CANPack.Data[6] | (uint8_T)(tempValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 4 ------------------
     *  startBit                = 24
     *  length                  = 8
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real32_T outValue = 0;

      {
        real32_T result = RP20231104WLR_231105_B.Acc_POS;

        /* no scaling required */
        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        uint8_T packedValue;
        if (outValue > (real32_T)(255)) {
          packedValue = (uint8_T) 255;
        } else if (outValue < (real32_T)(0)) {
          packedValue = (uint8_T) 0;
        } else {
          packedValue = (uint8_T) (outValue);
        }

        {
          {
            RP20231104WLR_231105_B.CANPack.Data[3] =
              RP20231104WLR_231105_B.CANPack.Data[3] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 5 ------------------
     *  startBit                = 0
     *  length                  = 8
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = UNSIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real32_T outValue = 0;

      {
        real32_T result = motor_Temp;

        /* no scaling required */
        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        uint8_T packedValue;
        if (outValue > (real32_T)(255)) {
          packedValue = (uint8_T) 255;
        } else if (outValue < (real32_T)(0)) {
          packedValue = (uint8_T) 0;
        } else {
          packedValue = (uint8_T) (outValue);
        }

        {
          {
            RP20231104WLR_231105_B.CANPack.Data[0] =
              RP20231104WLR_231105_B.CANPack.Data[0] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 6 ------------------
     *  startBit                = 8
     *  length                  = 8
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
        real64_T result = Wheel_spd_F;

        /* no scaling required */
        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        uint8_T packedValue;
        if (outValue > (real64_T)(255)) {
          packedValue = (uint8_T) 255;
        } else if (outValue < (real64_T)(0)) {
          packedValue = (uint8_T) 0;
        } else {
          packedValue = (uint8_T) (outValue);
        }

        {
          {
            RP20231104WLR_231105_B.CANPack.Data[1] =
              RP20231104WLR_231105_B.CANPack.Data[1] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 7 ------------------
     *  startBit                = 16
     *  length                  = 8
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
        real64_T result = Spd_R;

        /* no scaling required */
        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        uint8_T packedValue;
        if (outValue > (real64_T)(255)) {
          packedValue = (uint8_T) 255;
        } else if (outValue < (real64_T)(0)) {
          packedValue = (uint8_T) 0;
        } else {
          packedValue = (uint8_T) (outValue);
        }

        {
          {
            RP20231104WLR_231105_B.CANPack.Data[2] =
              RP20231104WLR_231105_B.CANPack.Data[2] | (uint8_T)(packedValue);
          }
        }
      }
    }
  }

  /* S-Function (scanpack): '<S68>/CAN Pack1' */
  /* S-Function (scanpack): '<S68>/CAN Pack1' */
  RP20231104WLR_231105_B.CANPack1_d.ID = 292U;
  RP20231104WLR_231105_B.CANPack1_d.Length = 2U;
  RP20231104WLR_231105_B.CANPack1_d.Extended = 0U;
  RP20231104WLR_231105_B.CANPack1_d.Remote = 0;
  RP20231104WLR_231105_B.CANPack1_d.Data[0] = 0;
  RP20231104WLR_231105_B.CANPack1_d.Data[1] = 0;
  RP20231104WLR_231105_B.CANPack1_d.Data[2] = 0;
  RP20231104WLR_231105_B.CANPack1_d.Data[3] = 0;
  RP20231104WLR_231105_B.CANPack1_d.Data[4] = 0;
  RP20231104WLR_231105_B.CANPack1_d.Data[5] = 0;
  RP20231104WLR_231105_B.CANPack1_d.Data[6] = 0;
  RP20231104WLR_231105_B.CANPack1_d.Data[7] = 0;

  {
    /* --------------- START Packing signal 0 ------------------
     *  startBit                = 0
     *  length                  = 8
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = SIGNED
     *  factor                  = 30.0
     *  offset                  = 100.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real32_T outValue = 0;

      {
        real32_T result = RP20231104WLR_231105_B.Brk_R;

        /* full scaling operation */
        result = (result - 100.0F) * (1 / 30.0F);

        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        int8_T packedValue;
        int32_T scaledValue;
        if (outValue > 2147483647.0) {
          scaledValue = 2147483647;
        } else if (outValue < -2147483648.0) {
          scaledValue = -2147483647 - 1;
        } else {
          scaledValue = (int32_T) outValue;
        }

        if (scaledValue > (int32_T) (127)) {
          packedValue = 127;
        } else if (scaledValue < (int32_T)((-(127)-1))) {
          packedValue = (-(127)-1);
        } else {
          packedValue = (int8_T) (scaledValue);
        }

        {
          uint8_T* tempValuePtr = (uint8_T*)&packedValue;
          uint8_T tempValue = *tempValuePtr;

          {
            RP20231104WLR_231105_B.CANPack1_d.Data[0] =
              RP20231104WLR_231105_B.CANPack1_d.Data[0] | (uint8_T)(tempValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 1 ------------------
     *  startBit                = 8
     *  length                  = 8
     *  desiredSignalByteLayout = LITTLEENDIAN
     *  dataType                = SIGNED
     *  factor                  = 1.0
     *  offset                  = 0.0
     *  minimum                 = 0.0
     *  maximum                 = 0.0
     * -----------------------------------------------------------------------*/
    {
      real64_T outValue = 0;

      {
        real64_T result = RPM;

        /* no scaling required */
        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        int8_T packedValue;
        int32_T scaledValue;
        if (outValue > 2147483647.0) {
          scaledValue = 2147483647;
        } else if (outValue < -2147483648.0) {
          scaledValue = -2147483647 - 1;
        } else {
          scaledValue = (int32_T) outValue;
        }

        if (scaledValue > (int32_T) (127)) {
          packedValue = 127;
        } else if (scaledValue < (int32_T)((-(127)-1))) {
          packedValue = (-(127)-1);
        } else {
          packedValue = (int8_T) (scaledValue);
        }

        {
          uint8_T* tempValuePtr = (uint8_T*)&packedValue;
          uint8_T tempValue = *tempValuePtr;

          {
            RP20231104WLR_231105_B.CANPack1_d.Data[1] =
              RP20231104WLR_231105_B.CANPack1_d.Data[1] | (uint8_T)(tempValue);
          }
        }
      }
    }
  }

  /* S-Function (ecucoder_canmessage): '<S68>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    RP20231104WLR_231105_B.CANPackMessage_o[0]=
      RP20231104WLR_231105_B.CANPack.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage_o[1]=
      RP20231104WLR_231105_B.CANPack.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage_o[2]=
      RP20231104WLR_231105_B.CANPack.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage_o[3]=
      RP20231104WLR_231105_B.CANPack.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage_o[4]=
      RP20231104WLR_231105_B.CANPack.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage_o[5]=
      RP20231104WLR_231105_B.CANPack.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage_o[6]=
      RP20231104WLR_231105_B.CANPack.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage_o[7]=
      RP20231104WLR_231105_B.CANPack.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ecucoder_canmessage): '<S68>/CANPackMessage1' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    RP20231104WLR_231105_B.CANPackMessage1[0]=
      RP20231104WLR_231105_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage1[1]=
      RP20231104WLR_231105_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage1[2]=
      RP20231104WLR_231105_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage1[3]=
      RP20231104WLR_231105_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage1[4]=
      RP20231104WLR_231105_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage1[5]=
      RP20231104WLR_231105_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage1[6]=
      RP20231104WLR_231105_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage1[7]=
      RP20231104WLR_231105_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S68>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN0BUF9TX[8];
    uint8 can0buf9looptx= 0;
    CAN0BUF9TX[can0buf9looptx]= RP20231104WLR_231105_B.CANPackMessage_o[0];
    can0buf9looptx++;
    CAN0BUF9TX[can0buf9looptx]= RP20231104WLR_231105_B.CANPackMessage_o[1];
    can0buf9looptx++;
    CAN0BUF9TX[can0buf9looptx]= RP20231104WLR_231105_B.CANPackMessage_o[2];
    can0buf9looptx++;
    CAN0BUF9TX[can0buf9looptx]= RP20231104WLR_231105_B.CANPackMessage_o[3];
    can0buf9looptx++;
    CAN0BUF9TX[can0buf9looptx]= RP20231104WLR_231105_B.CANPackMessage_o[4];
    can0buf9looptx++;
    CAN0BUF9TX[can0buf9looptx]= RP20231104WLR_231105_B.CANPackMessage_o[5];
    can0buf9looptx++;
    CAN0BUF9TX[can0buf9looptx]= RP20231104WLR_231105_B.CANPackMessage_o[6];
    can0buf9looptx++;
    CAN0BUF9TX[can0buf9looptx]= RP20231104WLR_231105_B.CANPackMessage_o[7];
    can0buf9looptx++;
    RP20231104WLR_231105_B.CANTransmit_p= ec_can_transmit(0, 9, 0, 291U, 8,
      CAN0BUF9TX);
  }

  /* S-Function (ec5744_cantransmitslb): '<S68>/CANTransmit1' */

  /*Transmit CAN message*/
  {
    uint8 CAN0BUF10TX[8];
    uint8 can0buf10looptx= 0;
    CAN0BUF10TX[can0buf10looptx]= RP20231104WLR_231105_B.CANPackMessage1[0];
    can0buf10looptx++;
    CAN0BUF10TX[can0buf10looptx]= RP20231104WLR_231105_B.CANPackMessage1[1];
    can0buf10looptx++;
    CAN0BUF10TX[can0buf10looptx]= RP20231104WLR_231105_B.CANPackMessage1[2];
    can0buf10looptx++;
    CAN0BUF10TX[can0buf10looptx]= RP20231104WLR_231105_B.CANPackMessage1[3];
    can0buf10looptx++;
    CAN0BUF10TX[can0buf10looptx]= RP20231104WLR_231105_B.CANPackMessage1[4];
    can0buf10looptx++;
    CAN0BUF10TX[can0buf10looptx]= RP20231104WLR_231105_B.CANPackMessage1[5];
    can0buf10looptx++;
    CAN0BUF10TX[can0buf10looptx]= RP20231104WLR_231105_B.CANPackMessage1[6];
    can0buf10looptx++;
    CAN0BUF10TX[can0buf10looptx]= RP20231104WLR_231105_B.CANPackMessage1[7];
    can0buf10looptx++;
    RP20231104WLR_231105_B.CANTransmit1= ec_can_transmit(0, 10, 0, 292U, 8,
      CAN0BUF10TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms' */

  /* S-Function (fcncallgen): '<S4>/10ms3' incorporates:
   *  SubSystem: '<S4>/VCU2MCU'
   */
  /* Switch: '<S69>/Switch2' incorporates:
   *  Constant: '<S69>/Constant13'
   *  Constant: '<S69>/Constant17'
   *  Constant: '<S69>/Constant19'
   *  Constant: '<S69>/Constant20'
   *  Switch: '<S69>/Switch3'
   */
  if (Trq_CUT) {
    Gear_Trs = 0.0;
    Mode_Trs = 0.0;
  } else {
    Gear_Trs = 2.0;
    Mode_Trs = 2.0;
  }

  /* End of Switch: '<S69>/Switch2' */

  /* DataTypeConversion: '<S69>/Cast To Boolean4' */
  RP20231104WLR_231105_B.CastToBoolean4 = (uint16_T)Gear_Trs;

  /* DataTypeConversion: '<S69>/Cast To Boolean6' */
  RP20231104WLR_231105_B.CastToBoolean6 = (uint16_T)Mode_Trs;

  /* DataTypeConversion: '<S69>/Data Type Conversion1' */
  RP20231104WLR_231105_B.DataTypeConversion1 = (real32_T)VCU_SpdCmd;

  /* DataTypeConversion: '<S69>/Data Type Conversion2' */
  RP20231104WLR_231105_B.DataTypeConversion2 = (int32_T)floorf(VCU_TrqCmd);

  /* S-Function (scanpack): '<S69>/CAN Pack1' */
  /* S-Function (scanpack): '<S69>/CAN Pack1' */
  RP20231104WLR_231105_B.CANPack1.ID = 146927393U;
  RP20231104WLR_231105_B.CANPack1.Length = 8U;
  RP20231104WLR_231105_B.CANPack1.Extended = 1U;
  RP20231104WLR_231105_B.CANPack1.Remote = 0;
  RP20231104WLR_231105_B.CANPack1.Data[0] = 0;
  RP20231104WLR_231105_B.CANPack1.Data[1] = 0;
  RP20231104WLR_231105_B.CANPack1.Data[2] = 0;
  RP20231104WLR_231105_B.CANPack1.Data[3] = 0;
  RP20231104WLR_231105_B.CANPack1.Data[4] = 0;
  RP20231104WLR_231105_B.CANPack1.Data[5] = 0;
  RP20231104WLR_231105_B.CANPack1.Data[6] = 0;
  RP20231104WLR_231105_B.CANPack1.Data[7] = 0;

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
        uint32_T result = (uint32_T) (RP20231104WLR_231105_B.CastToBoolean4);

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
            RP20231104WLR_231105_B.CANPack1.Data[4] =
              RP20231104WLR_231105_B.CANPack1.Data[4] | (uint8_T)(packedValue);
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
        uint32_T result = (uint32_T) (RP20231104WLR_231105_B.CastToBoolean6);

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
            RP20231104WLR_231105_B.CANPack1.Data[5] =
              RP20231104WLR_231105_B.CANPack1.Data[5] | (uint8_T)((uint8_T)
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
        real32_T result = RP20231104WLR_231105_B.DataTypeConversion1;

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
            RP20231104WLR_231105_B.CANPack1.Data[0] =
              RP20231104WLR_231105_B.CANPack1.Data[0] | (uint8_T)((uint16_T)
              (packedValue & (uint16_T)0xFFU));
            RP20231104WLR_231105_B.CANPack1.Data[1] =
              RP20231104WLR_231105_B.CANPack1.Data[1] | (uint8_T)((uint16_T)
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
        int32_T result = (int32_T) (RP20231104WLR_231105_B.DataTypeConversion2);

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
            RP20231104WLR_231105_B.CANPack1.Data[2] =
              RP20231104WLR_231105_B.CANPack1.Data[2] | (uint8_T)((uint16_T)
              (packedValue & (uint16_T)0xFFU));
            RP20231104WLR_231105_B.CANPack1.Data[3] =
              RP20231104WLR_231105_B.CANPack1.Data[3] | (uint8_T)((uint16_T)
              ((uint16_T)(packedValue & (uint16_T)0xFF00U) >> 8));
          }
        }
      }
    }
  }

  /* S-Function (ecucoder_canmessage): '<S69>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    RP20231104WLR_231105_B.CANPackMessage[0]=
      RP20231104WLR_231105_B.CANPack1.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage[1]=
      RP20231104WLR_231105_B.CANPack1.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage[2]=
      RP20231104WLR_231105_B.CANPack1.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage[3]=
      RP20231104WLR_231105_B.CANPack1.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage[4]=
      RP20231104WLR_231105_B.CANPack1.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage[5]=
      RP20231104WLR_231105_B.CANPack1.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage[6]=
      RP20231104WLR_231105_B.CANPack1.Data[canpackloop];
    canpackloop++;
    RP20231104WLR_231105_B.CANPackMessage[7]=
      RP20231104WLR_231105_B.CANPack1.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S69>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN0BUF8TX[8];
    uint8 can0buf8looptx= 0;
    CAN0BUF8TX[can0buf8looptx]= RP20231104WLR_231105_B.CANPackMessage[0];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= RP20231104WLR_231105_B.CANPackMessage[1];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= RP20231104WLR_231105_B.CANPackMessage[2];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= RP20231104WLR_231105_B.CANPackMessage[3];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= RP20231104WLR_231105_B.CANPackMessage[4];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= RP20231104WLR_231105_B.CANPackMessage[5];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= RP20231104WLR_231105_B.CANPackMessage[6];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= RP20231104WLR_231105_B.CANPackMessage[7];
    can0buf8looptx++;
    RP20231104WLR_231105_B.CANTransmit_d= ec_can_transmit(0, 8, 1, 146927393U, 8,
      CAN0BUF8TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms3' */

  /* S-Function (fcncallgen): '<S1>/10ms5' incorporates:
   *  SubSystem: '<S1>/FAN_WP_CTRL'
   */
  /* Lookup_n-D: '<S7>/MCU__spd' */
  rtb_Add = look1_iflf_binlc(MCU_Temp,
    RP20231104WLR_231105_ConstP.MCU__spd_bp01Data,
    RP20231104WLR_231105_ConstP.MCU__spd_tableData, 13U);

  /* Lookup_n-D: '<S7>/moter__spd' */
  rtb_APP_POS2 = look1_iflf_binlc(motor_Temp,
    RP20231104WLR_231105_ConstP.moter__spd_bp01Data,
    RP20231104WLR_231105_ConstP.moter__spd_tableData, 12U);

  /* MinMax: '<S7>/MinMax' */
  rtb_Add = fmaxf(rtb_Add, rtb_APP_POS2);
  rtb_Water_pump_i = (uint16_T)rtb_Add;

  /* Gain: '<S7>/Gain' */
  rtb_Gain3_m = 40960U * rtb_Water_pump_i;

  /* DataTypeConversion: '<S7>/Data Type Conversion3' */
  RP20231104WLR_231105_B.Water_pump = (uint16_T)(rtb_Gain3_m >> 13);

  /* End of Outputs for S-Function (fcncallgen): '<S1>/10ms5' */

  /* S-Function (fcncallgen): '<S4>/10ms6' incorporates:
   *  SubSystem: '<S4>/WP_OUTPUT'
   */
  /* S-Function (ec5744_pdpslbu3): '<S70>/PowerDriverPWM' incorporates:
   *  Constant: '<S70>/Constant'
   */

  /* Power driver PWM output for channel 6 */
  ec_pwm_output(6,((uint16_T)1000U),RP20231104WLR_231105_B.Water_pump);

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms6' */

  /* S-Function (fcncallgen): '<S75>/10ms' incorporates:
   *  SubSystem: '<S75>/daq10ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S87>/CCPDAQ' */
  ccpDaq(1);

  /* End of Outputs for S-Function (fcncallgen): '<S75>/10ms' */

  /* Update absolute time */
  /* The "clockTick3" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick3" ensures timer will not overflow during the
   * application lifespan selected.
   */
  RP20231104WLR_231105_M->Timing.clockTick3++;
}

/* Model step function for TID4 */
void RP20231104WLR_231105_step4(void)  /* Sample time: [0.05s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S75>/50ms' incorporates:
   *  SubSystem: '<S75>/daq50ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S89>/CCPDAQ' */
  ccpDaq(2);

  /* End of Outputs for S-Function (fcncallgen): '<S75>/50ms' */
}

/* Model step function for TID5 */
void RP20231104WLR_231105_step5(void)  /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S74>/100MS' incorporates:
   *  SubSystem: '<S74>/Function-Call Subsystem'
   */
  /* S-Function (ec5744_canreceiveslb): '<S78>/CANReceive' */

  /* Receive CAN message */
  {
    uint8 CAN2BUF1RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can2buf1looprx= 0;
    RP20231104WLR_231105_B.CANReceive_o3_c= 278;
    RP20231104WLR_231105_B.CANReceive_o5_k= 8;
    RP20231104WLR_231105_B.CANReceive_o2_o= ec_can_receive(2,1, CAN2BUF1RX);
    RP20231104WLR_231105_B.CANReceive_o4_p[0]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    RP20231104WLR_231105_B.CANReceive_o4_p[1]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    RP20231104WLR_231105_B.CANReceive_o4_p[2]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    RP20231104WLR_231105_B.CANReceive_o4_p[3]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    RP20231104WLR_231105_B.CANReceive_o4_p[4]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    RP20231104WLR_231105_B.CANReceive_o4_p[5]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    RP20231104WLR_231105_B.CANReceive_o4_p[6]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    RP20231104WLR_231105_B.CANReceive_o4_p[7]= CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
  }

  /* Call the system: <S78>/Function-Call Subsystem */

  /* Output and update for function-call system: '<S78>/Function-Call Subsystem' */
  {
    uint8_T rtb_Add;
    uint8_T rtb_Compare;

    /* Outputs for Enabled SubSystem: '<S79>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S80>/Enable'
     */
    if (RP20231104WLR_231105_B.CANReceive_o2_o > 0) {
      /* RelationalOperator: '<S81>/Compare' incorporates:
       *  Constant: '<S81>/Constant'
       */
      rtb_Add = (uint8_T)(RP20231104WLR_231105_B.CANReceive_o4_p[0] == 83);

      /* RelationalOperator: '<S82>/Compare' incorporates:
       *  Constant: '<S82>/Constant'
       */
      rtb_Compare = (uint8_T)(RP20231104WLR_231105_B.CANReceive_o4_p[5] == 84);

      /* Sum: '<S80>/Add' */
      rtb_Add = (uint8_T)((uint32_T)rtb_Add + rtb_Compare);

      /* RelationalOperator: '<S83>/Compare' incorporates:
       *  Constant: '<S83>/Constant'
       */
      rtb_Compare = (uint8_T)(rtb_Add == 2);

      /* If: '<S80>/If' */
      if (rtb_Compare > 0) {
        /* Outputs for IfAction SubSystem: '<S80>/If Action Subsystem' incorporates:
         *  ActionPort: '<S84>/Action Port'
         */
        /* S-Function (ec5744_bootloaderslb): '<S84>/BootLoader' */
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

        /* S-Function (ec5744_cpuresetslb): '<S84>/CPUReset' */

        /* Perform a microcontroller reset */
        MC_ME.MCTL.R = 0X00005AF0;
        MC_ME.MCTL.R = 0X0000A50F;

        /* End of Outputs for SubSystem: '<S80>/If Action Subsystem' */
      } else {
        /* Outputs for IfAction SubSystem: '<S80>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S85>/Action Port'
         */
        /* S-Function (ec5744_cantransmitslb): '<S85>/CANTransmit' incorporates:
         *  Constant: '<S85>/Constant'
         */

        /*Transmit CAN message*/
        {
          uint8 CAN2BUF9TX[1];
          uint8 can2buf9looptx= 0;
          CAN2BUF9TX[can2buf9looptx]= ((uint8_T)1U);
          can2buf9looptx++;
          RP20231104WLR_231105_B.CANTransmit= ec_can_transmit(2, 9, 0, 593U, 1,
            CAN2BUF9TX);
        }

        /* End of Outputs for SubSystem: '<S80>/If Action Subsystem1' */
      }

      /* End of If: '<S80>/If' */
    }

    /* End of Outputs for SubSystem: '<S79>/Enabled Subsystem' */
  }

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S78>/CANReceive' */
  /* End of Outputs for S-Function (fcncallgen): '<S74>/100MS' */

  /* S-Function (fcncallgen): '<S75>/100ms' incorporates:
   *  SubSystem: '<S75>/daq100ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S86>/CCPDAQ' */
  ccpDaq(3);

  /* End of Outputs for S-Function (fcncallgen): '<S75>/100ms' */
}

/* Model step function for TID6 */
void RP20231104WLR_231105_step6(void)  /* Sample time: [0.5s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S75>/500ms' incorporates:
   *  SubSystem: '<S75>/daq500ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S88>/CCPDAQ' */
  ccpDaq(4);

  /* End of Outputs for S-Function (fcncallgen): '<S75>/500ms' */

  /* S-Function (fcncallgen): '<S76>/500ms' incorporates:
   *  SubSystem: '<S76>/EEPROMOperation'
   */

  /* S-Function (ec5744_eepromoslb): '<S91>/EEPROMOperatin' */
#if defined EC_EEPROM_ENABLE

  /* Operate the EEPROM module on the MPC5744 */
  ec_flash_operation();

#endif

  /* End of Outputs for S-Function (fcncallgen): '<S76>/500ms' */
}

/* Model step wrapper function for compatibility with a static main program */
void RP20231104WLR_231105_step(int_T tid)
{
  switch (tid) {
   case 0 :
    RP20231104WLR_231105_step0();
    break;

   case 1 :
    RP20231104WLR_231105_step1();
    break;

   case 2 :
    RP20231104WLR_231105_step2();
    break;

   case 3 :
    RP20231104WLR_231105_step3();
    break;

   case 4 :
    RP20231104WLR_231105_step4();
    break;

   case 5 :
    RP20231104WLR_231105_step5();
    break;

   case 6 :
    RP20231104WLR_231105_step6();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void RP20231104WLR_231105_initialize(void)
{
  /* Start for S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/Function-Call Subsystem1'
   */
  /* Start for S-Function (ec5744_ffrslbu3): '<S24>/FrequencyRead' */
  ec_etimer_init(0);
  IntcIsrVectorTable[611] = (uint32_t)&ISR_eTimer_CH0;

  /* End of Start for S-Function (fcncallgen): '<S3>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/MCU_RECIEVE'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S25>/CANReceive1' incorporates:
   *  SubSystem: '<S25>/MCU_pwr'
   */
  /* Start for function-call system: '<S25>/MCU_pwr' */

  /* Start for Enabled SubSystem: '<S50>/MCU_VCUMeter1' */

  /* Start for S-Function (scanunpack): '<S52>/CAN Unpack' */

  /*-----------S-Function Block: <S52>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S50>/MCU_VCUMeter1' */
  ec_buffer_init(0,1,1,218089455);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S25>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S25>/CANReceive3' incorporates:
   *  SubSystem: '<S25>/MCU_state'
   */
  /* Start for function-call system: '<S25>/MCU_state' */

  /* Start for Enabled SubSystem: '<S51>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S58>/CAN Unpack' */

  /*-----------S-Function Block: <S58>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S51>/MCU_state' */
  ec_buffer_init(0,0,1,218089199);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S25>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms4' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S68>/CANTransmit' */
  ec_buffer_init(0,9,0,291U);

  /* Start for S-Function (ec5744_cantransmitslb): '<S68>/CANTransmit1' */
  ec_buffer_init(0,10,0,292U);

  /* End of Start for S-Function (fcncallgen): '<S4>/10ms' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S69>/CANTransmit' */
  ec_buffer_init(0,8,1,146927393U);

  /* End of Start for S-Function (fcncallgen): '<S4>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S4>/10ms6' incorporates:
   *  SubSystem: '<S4>/WP_OUTPUT'
   */
  /* Start for S-Function (ec5744_pdpslbu3): '<S70>/PowerDriverPWM' incorporates:
   *  Constant: '<S70>/Constant'
   */

  /* Initialize PWM output for channel 6 */
  SIUL2_MSCR42 = 0X02000003;           //PWM_A3

  /* End of Start for S-Function (fcncallgen): '<S4>/10ms6' */

  /* Start for S-Function (ec5744_initinterruptslb): '<Root>/Initialization1' */

  /* Call the downstream function call subsystem <Root>/Init */

  /* End of Start for S-Function (ec5744_initinterruptslb): '<Root>/Initialization1' */

  /* Start for S-Function (fcncallgen): '<S74>/100MS' incorporates:
   *  SubSystem: '<S74>/Function-Call Subsystem'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S78>/CANReceive' incorporates:
   *  SubSystem: '<S78>/Function-Call Subsystem'
   */
  /* Start for function-call system: '<S78>/Function-Call Subsystem' */

  /* Start for Enabled SubSystem: '<S79>/Enabled Subsystem' */
  /* Start for IfAction SubSystem: '<S80>/If Action Subsystem1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S85>/CANTransmit' incorporates:
   *  Constant: '<S85>/Constant'
   */
  ec_buffer_init(2,9,0,593U);

  /* End of Start for SubSystem: '<S80>/If Action Subsystem1' */
  /* End of Start for SubSystem: '<S79>/Enabled Subsystem' */
  ec_buffer_init(2,1,0,278);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S78>/CANReceive' */
  /* End of Start for S-Function (fcncallgen): '<S74>/100MS' */

  /* Start for S-Function (fcncallgen): '<S77>/Function-Call Generator' incorporates:
   *  SubSystem: '<S77>/CCPBackground'
   */
  /* Start for S-Function (ec5744_ccpslb): '<S92>/CCPBackground' */
  ccpInit();

  /* End of Start for S-Function (fcncallgen): '<S77>/Function-Call Generator' */

  /* Start for S-Function (ec5744_caninterruptslb1): '<S77>/ReceiveandTransmitInterrupt' incorporates:
   *  SubSystem: '<S77>/CCPReceive'
   */
  /* Start for function-call system: '<S77>/CCPReceive' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S93>/CANReceive' */
  ec_buffer_init(2,0,0,CCP_CRO_ID);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S93>/CANReceive' */
  ec_bufint_init(2,0);
  INTC_0.PSR[548].B.PRIN = 12;
  IntcIsrVectorTable[548] = (uint32_t)&ISR_FlexCAN_2_MB0;

  /* End of Start for S-Function (ec5744_caninterruptslb1): '<S77>/ReceiveandTransmitInterrupt' */

  /* Start for S-Function (ec5744_eeprombsbu3): '<Root>/EEPROMEnable' */
  Fls_Read(0xFA0010,ecflashdataold,4096);
  Fls_Read(0xFA0010,ecflashdatanew,4096);

  /* ConstCode for S-Function (ec5744_initinterruptslb): '<Root>/Initialization1' incorporates:
   *  SubSystem: '<Root>/Init'
   */
  /* ConstCode for function-call system: '<Root>/Init' */

  /* ConstCode for S-Function (ec5744_arsslbu3): '<S2>/AfterRunSwitch1' incorporates:
   *  Constant: '<S2>/Constant'
   */

  /* Set level true for after run switch */
  AfterRunFlags[0] = true;
  if (AfterRunFlags[0] == 1) {
    ec_gpio_write(58,1);
  } else {
    if (AfterRunFlags[1] == 1) {
      AfterRunFlags[0] = 0;
    } else {
      AfterRunFlags[0] = 0;
      ec_gpio_write(58,0);
    }
  }

  /* End of ConstCode for S-Function (ec5744_initinterruptslb): '<Root>/Initialization1' */

  /* Enable for S-Function (fcncallgen): '<S6>/10ms7' incorporates:
   *  SubSystem: '<S6>/Drive_ready'
   */
  /* Enable for Chart: '<S95>/Chart1' */
  RP20231104WLR_231105_DW.previousTicks =
    RP20231104WLR_231105_M->Timing.clockTick3;

  /* End of Enable for S-Function (fcncallgen): '<S6>/10ms7' */

  /* Enable for S-Function (fcncallgen): '<S1>/10ms1' incorporates:
   *  SubSystem: '<S1>/MoTrqReq'
   */
  RP20231104WLR_231105_DW.MoTrqReq_RESET_ELAPS_T = true;

  /* End of Enable for S-Function (fcncallgen): '<S1>/10ms1' */
}

/* File trailer for ECUCoder generated file RP20231104WLR_231105.c.
 *
 * [EOF]
 */
