/*
 * Code generated for Simulink model VehCtrlMdel241025_2018b_amkspdlimit.
 *
 * FILE    : VehCtrlMdel241025_2018b_amkspdlimit.c
 *
 * VERSION : 1.305
 *
 * DATE    : Fri Oct 25 15:32:30 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#include "VehCtrlMdel241025_2018b_amkspdlimit.h"
#include "VehCtrlMdel241025_2018b_amkspdlimit_private.h"

/* #include "myinclude.h" */

/* Named constants for Chart: '<S8>/Timer1' */
#define VehCtrlMdel241025_2018b__IN_Out ((uint8_T)2U)
#define VehCtrlMdel241025_20_IN_Trigger ((uint8_T)3U)
#define VehCtrlMdel241025_IN_InterState ((uint8_T)1U)

/* Named constants for Chart: '<S130>/Timer' */
#define VehCtrlMdel241025_2018_IN_Out_n ((uint8_T)2U)
#define VehCtrlMdel241025__IN_Trigger_c ((uint8_T)3U)
#define VehCtrlMdel2410_IN_InterState_n ((uint8_T)1U)

/* Named constants for Chart: '<S7>/Chart' */
#define VehCtrlMd_IN_NO_ACTIVE_CHILD_kx ((uint8_T)0U)
#define VehCtrlMdel241025_2018b_am_IN_B ((uint8_T)1U)
#define VehCtrlMdel241025_2018b_am_IN_C ((uint8_T)2U)
#define VehCtrlMdel241025_IN_DYC_Enable ((uint8_T)2U)
#define VehCtrlMdel241025_IN_InitState1 ((uint8_T)1U)
#define VehCtrlMdel241025_IN_InitState2 ((uint8_T)1U)
#define VehCtrlMdel241025__IN_InitState ((uint8_T)3U)
#define VehCtrlMdel241_IN_DYC_Disenable ((uint8_T)1U)
#define VehCtrlMdel24_IN_TCSF_Disenable ((uint8_T)2U)
#define VehCtrlMdel24_IN_TCSR_Disenable ((uint8_T)2U)
#define VehCtrlMdel2_IN_F_TVD_TCS_STATE ((uint8_T)3U)

/* Named constants for Chart: '<S113>/Chart2' */
#define VehCtr_IN_MCFL_InverterOn_State ((uint8_T)6U)
#define VehCtr_IN_MCFR_InverterOn_State ((uint8_T)7U)
#define VehCtrlMdel241025_2018_IN_Guard ((uint8_T)1U)
#define VehCtrlMdel241025_2018_IN_Ready ((uint8_T)5U)
#define VehCtrlMdel241025_2018_IN_Trans ((uint8_T)7U)
#define VehCtrlMdel241025_2018_IN_start ((uint8_T)10U)
#define VehCtrlMdel241025_2018b_IN_Init ((uint8_T)2U)
#define VehCtrlMdel241025_2018b__IN_OFF ((uint8_T)1U)
#define VehCtrlMdel241025_2018b_a_IN_ON ((uint8_T)2U)
#define VehCtrlMdel241025_201_IN_AMKCAN ((uint8_T)1U)
#define VehCtrlMdel241025_20_IN_Standby ((uint8_T)6U)
#define VehCtrlMdel241025_IN_MC_DCready ((uint8_T)8U)
#define VehCtrlMdel241025_IN_SYSRDYCECK ((uint8_T)9U)
#define VehCtrlMdel241025_event_AMKDCON (3)
#define VehCtrlMdel241025_event_EbeepON (5)
#define VehCtrlMdel24102_IN_MCDCOncheck ((uint8_T)5U)
#define VehCtrlMdel24102_event_AMKCANON (1)
#define VehCtrlMdel24102_event_AMKDCOFF (2)
#define VehCtrlMdel24102_event_EbeepOFF (4)
#define VehCtrlMdel24102_event_TorqueON (15)
#define VehCtrlMdel2410_IN_MCUReadyFail ((uint8_T)4U)
#define VehCtrlMdel2410_event_AMKCANOFF (0)
#define VehCtrlMdel2410_event_TorqueOFF (14)
#define VehCtrlMdel241_IN_AMKDCOnFinish ((uint8_T)2U)
#define VehCtrlMdel241_IN_DCOnCheckPass ((uint8_T)3U)
#define VehCtrlMdel241_IN_InitStateBack ((uint8_T)3U)
#define VehCtrlMdel241_IN_WaitForEngine ((uint8_T)8U)
#define VehCtrlMdel2_IN_MCDCEnableState ((uint8_T)4U)
#define VehCtrlMdel2_event_InverterFLON (7)
#define VehCtrlMdel2_event_InverterFRON (9)
#define VehCtrlMdel2_event_MCDCEnableON (13)
#define VehCtrlMdel_event_InverterFLOFF (6)
#define VehCtrlMdel_event_InverterFROFF (8)
#define VehCtrlMdel_event_MCDCEnableOFF (12)

/* Named constants for Chart: '<S130>/Timer1' */
#define VehCtrlMdel241025_2018_IN_Out_b ((uint8_T)2U)
#define VehCtrlMdel241025__IN_Trigger_e ((uint8_T)3U)
#define VehCtrlMdel2410_IN_InterState_d ((uint8_T)1U)

/* Named constants for Chart: '<S221>/Chart' */
#define VehCtrlMdel241025_2018b_IN_ON_d ((uint8_T)1U)
#define VehCtrlMdel241025_20_IN_STATEON ((uint8_T)2U)
#define VehCtrlMdel241025_2_IN_STATEOFF ((uint8_T)1U)
#define VehCtrlMdel241025__IN_initstate ((uint8_T)2U)

/* Named constants for Chart: '<S358>/Chart' */
#define VehCtrlMdel2410_IN_initstate1_c ((uint8_T)2U)

/* Named constants for Chart: '<S366>/Chart' */
#define VehCtrlMdel241025_2018_IN_LEDON ((uint8_T)2U)
#define VehCtrlMdel241025_201_IN_Init_e ((uint8_T)1U)
#define VehCtrlMdel241025_201_IN_LEDOFF ((uint8_T)1U)
#define VehCtrlMdel241025_201_IN_StateA ((uint8_T)2U)
#define VehCtrlMdel241025_201_IN_StateB ((uint8_T)3U)
#define VehCtrlMdel241025_201_IN_StateC ((uint8_T)4U)

/* Named constants for Chart: '<S370>/Chart1' */
#define VehCtrlMdel241025_2018_IN_REDON ((uint8_T)2U)
#define VehCtrlMdel241025_201_IN_REDOFF ((uint8_T)1U)

boolean L9826VAR701[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

boolean L9826DIAG701[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

boolean L9826VAR702[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

boolean L9826DIAG702[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

/* Exported block signals */
real_T Gear_Trs;                       /* '<S361>/Switch2' */
real_T Mode_Trs;                       /* '<S361>/Switch3' */
real_T KeyPressed;     /* '<S215>/BusConversion_InsertedFor_Out1_at_inport_0' */
real_T AMKFL_Current;                  /* '<S220>/Switch' */
real_T AMKFR_Current;                  /* '<S220>/Switch1' */
real_T EmraxPwr;                       /* '<S220>/Product2' */
real_T Trq_CUT;                        /* '<S217>/Timer' */
real_T AMKSWITCH;                      /* '<S130>/Timer1' */
real_T ignition;                       /* '<S130>/Timer' */
real_T L12V_error;                     /* '<S190>/CAN Unpack' */
real_T alarm;                          /* '<S190>/CAN Unpack' */
real_T controller_ready;               /* '<S190>/CAN Unpack' */
real_T selfcheck;                      /* '<S190>/CAN Unpack' */
real_T RPM;                            /* '<S190>/CAN Unpack' */
real_T trq;                            /* '<S190>/CAN Unpack' */
real_T AC_current;                     /* '<S184>/CAN Unpack' */
real_T DC_current;                     /* '<S184>/CAN Unpack' */
real_T MCU_Temp;                       /* '<S184>/CAN Unpack' */
real_T motor_Temp;                     /* '<S184>/CAN Unpack' */
real_T voltage;                        /* '<S184>/CAN Unpack' */
real_T MCFR_ActualTorque;              /* '<S161>/CAN Unpack' */
real_T MCFR_ActualVelocity;            /* '<S161>/CAN Unpack' */
real_T MCFR_DCVoltage;                 /* '<S161>/CAN Unpack' */
real_T MCFR_bDCOn;                     /* '<S161>/CAN Unpack' */
real_T MCFR_bError;                    /* '<S161>/CAN Unpack' */
real_T MCFR_bInverterOn;               /* '<S161>/CAN Unpack' */
real_T MCFR_bQuitInverterOn;           /* '<S161>/CAN Unpack' */
real_T MCFR_bSystemReady;              /* '<S161>/CAN Unpack' */
real_T MCFR_TempIGBT;                  /* '<S172>/CAN Unpack' */
real_T MCFR_TempInverter;              /* '<S172>/CAN Unpack' */
real_T MCFR_TempMotor;                 /* '<S172>/CAN Unpack' */
real_T MCFR_ErrorInfo;                 /* '<S170>/CAN Unpack' */
real_T MCFL_ActualTorque;              /* '<S142>/CAN Unpack' */
real_T MCFL_ActualVelocity;            /* '<S142>/CAN Unpack' */
real_T MCFL_DCVoltage;                 /* '<S142>/CAN Unpack' */
real_T MCFL_bDCOn;                     /* '<S142>/CAN Unpack' */
real_T MCFL_bError;                    /* '<S142>/CAN Unpack' */
real_T MCFL_bInverterOn;               /* '<S142>/CAN Unpack' */
real_T MCFL_bQuitDCOn;                 /* '<S142>/CAN Unpack' */
real_T MCFL_bQuitInverterOn;           /* '<S142>/CAN Unpack' */
real_T MCFL_bSystemReady;              /* '<S142>/CAN Unpack' */
real_T MCFL_TempIGBT;                  /* '<S154>/CAN Unpack' */
real_T MCFL_TempInverter;              /* '<S154>/CAN Unpack' */
real_T MCFL_TempMotor;                 /* '<S154>/CAN Unpack' */
real_T MCFL_ErrorInfo;                 /* '<S152>/CAN Unpack' */
real_T StrWhlAngAliveRollCnt;          /* '<S203>/CAN Unpack1' */
real_T StrWhlAng;                      /* '<S203>/CAN Unpack1' */
real_T StrWhlAngV;                     /* '<S203>/CAN Unpack1' */
real_T ABS_WS_FL;                      /* '<S132>/CAN Unpack1' */
real_T ABS_WS_FR;                      /* '<S132>/CAN Unpack1' */
real_T ABS_WS_RL;                      /* '<S132>/CAN Unpack1' */
real_T ABS_WS_RR;                      /* '<S132>/CAN Unpack1' */
real_T IMU_Ay_Value;                   /* '<S198>/CAN Unpack' */
real_T IMU_Ax_Value;                   /* '<S198>/CAN Unpack' */
real_T IMU_Yaw_Value;                  /* '<S198>/CAN Unpack' */
real_T EMRAX_Trq_CUT;                  /*  */
real_T AMK_Trq_CUT;                    /*  */
uint32_T Acc_vol2;                     /* '<S217>/Add3' */
uint32_T Acc_vol;                      /* '<S217>/Add2' */
uint32_T Acc_POS;                      /* '<S217>/1-D Lookup Table4' */
uint32_T Acc_POS2;                     /* '<S217>/1-D Lookup Table3' */
real32_T VehVxEst_mps;                 /* '<S345>/Add' */
real32_T PwrALL;                       /* '<S28>/Gain3' */
real32_T EmraxTrqR_cmd;                /* '<S7>/Saturation1' */
real32_T AMKTrqFR_cmd;                 /* '<S7>/Saturation3' */
real32_T AMKTrqFL_cmd;                 /* '<S7>/Saturation4' */
uint16_T F_BrkPrs;                     /* '<S217>/1-D Lookup Table1' */
uint16_T Acc1;                         /* '<S125>/Acc3' */
uint16_T Acc2;                         /* '<S125>/Acc4' */
uint16_T Brk1;                         /* '<S125>/Brk1' */
uint16_T Brk2;                         /* '<S125>/Brk2' */
boolean_T STATEDISPLAY;                /* '<S358>/Switch1' */
boolean_T HVSWITCH;                    /* '<S358>/Chart' */
boolean_T TSAL_SW_IN;                  /* '<S130>/SwitchInput2' */
boolean_T HV_voltValid;                /* '<S207>/Compare' */
boolean_T Brk;                         /* '<S115>/Compare' */
boolean_T ACC_Release;                 /* '<S116>/Compare' */
boolean_T beeper_state;                /* '<S113>/Chart2' */
boolean_T MCFL_DCOn_setpoints;         /* '<S113>/Chart2' */
boolean_T MCFR_DCEnable;               /* '<S113>/Chart2' */
boolean_T MCFR_InverterOn;             /* '<S113>/Chart2' */
boolean_T TrqR_cmd_raw;                /* '<S7>/Logical Operator1' */
boolean_T TroqueOn;                    /* '<S7>/Logical Operator6' */
boolean_T Trq_CUT_final;               /* '<S7>/Logical Operator4' */

/* Block signals (default storage) */
B_VehCtrlMdel241025_2018b_amk_T VehCtrlMdel241025_2018b_amksp_B;

/* Block states (default storage) */
DW_VehCtrlMdel241025_2018b_am_T VehCtrlMdel241025_2018b_amks_DW;

/* Real-time model */
RT_MODEL_VehCtrlMdel241025_20_T VehCtrlMdel241025_2018b_amks_M_;
RT_MODEL_VehCtrlMdel241025_20_T *const VehCtrlMdel241025_2018b_amks_M =
  &VehCtrlMdel241025_2018b_amks_M_;

/* Forward declaration for local functions */
static void VehC_enter_atomic_WaitForEngine(void);
static void VehCtrlMdel241025_2018b_VehStat(const real_T *controller_ready_e,
  const boolean_T *AND_n, const real_T *Switch_k, const real_T *Switch3, const
  real_T *Switch10);
static void VehCtrlMdel241025_20_AMKDCready(const real_T *MCFL_bDCOn_j, const
  real_T *MCFR_bDCOn_n, const boolean_T *AND_n, const real_T *Switch_k, const
  real_T *Switch3, const real_T *Switch10);
static void rate_monotonic_scheduler(void);

/* L9826 control function */
void ec_l9826tr701_control(boolean SPITX[8])
{
  uint8 i;
  uint16 SPI1TX[1]= { 0x00 };

  uint16 SPI1RX[1]= { 0x00 };

  for (i=0;i<8;i++) {
    SPI1TX[0]= SPI1TX[0]|(SPITX[i]<<(7-i));
  }

  ec_spi_masterwriteread(0, 0, 1, SPI1TX, SPI1RX);
  L9826DIAG701[0] = (SPI1RX[0]&0b10000000)>>7;
  L9826DIAG701[1] = (SPI1RX[0]&0b01000000)>>6;
  L9826DIAG701[2] = (SPI1RX[0]&0b00100000)>>5;
  L9826DIAG701[3] = (SPI1RX[0]&0b00010000)>>4;
  L9826DIAG701[4] = (SPI1RX[0]&0b00001000)>>3;
  L9826DIAG701[5] = (SPI1RX[0]&0b00000100)>>2;
  L9826DIAG701[6] = (SPI1RX[0]&0b00000010)>>1;
  L9826DIAG701[7] = (SPI1RX[0]&0b00000001)>>0;
}

void ec_l9826tr702_control(boolean SPITX[8])
{
  uint8 i;
  uint16 SPI1TX[1]= { 0x00 };

  uint16 SPI1RX[1]= { 0x00 };

  for (i=0;i<8;i++) {
    SPI1TX[0]= SPI1TX[0]|(SPITX[i]<<(7-i));
  }

  ec_spi_masterwriteread(0, 1, 1, SPI1TX, SPI1RX);
  L9826DIAG702[0] = (SPI1RX[0]&0b10000000)>>7;
  L9826DIAG702[1] = (SPI1RX[0]&0b01000000)>>6;
  L9826DIAG702[2] = (SPI1RX[0]&0b00100000)>>5;
  L9826DIAG702[3] = (SPI1RX[0]&0b00010000)>>4;
  L9826DIAG702[4] = (SPI1RX[0]&0b00001000)>>3;
  L9826DIAG702[5] = (SPI1RX[0]&0b00000100)>>2;
  L9826DIAG702[6] = (SPI1RX[0]&0b00000010)>>1;
  L9826DIAG702[7] = (SPI1RX[0]&0b00000001)>>0;
}

void ISR_PIT_CH3(void)
{
  PIT_0.TIMER[3].TFLG.R = 1;
  ECUCoderModelBaseCounter++;
  rate_monotonic_scheduler();
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

real32_T look1_iflf_binlx(real32_T u0, const real32_T bp0[], const real32_T
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
  /* Call the system: <S381>/CCPReceive */
  {
    /* S-Function (ec5744_caninterruptslb1): '<S381>/ReceiveandTransmitInterrupt' */

    /* Output and update for function-call system: '<S381>/CCPReceive' */

    /* S-Function (ec5744_canreceiveslb): '<S397>/CANReceive' */

    /* Receive CAN message */
    {
      uint8 CAN2BUF0RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

      uint8 can2buf0looprx= 0;
      VehCtrlMdel241025_2018b_amksp_B.CANReceive_o3= 256;
      VehCtrlMdel241025_2018b_amksp_B.CANReceive_o5= 8;
      VehCtrlMdel241025_2018b_amksp_B.CANReceive_o2= ec_can_receive(2,0,
        CAN2BUF0RX);
      VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4[0]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4[1]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4[2]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4[3]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4[4]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4[5]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4[6]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
      VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4[7]=
        CAN2BUF0RX[can2buf0looprx];
      can2buf0looprx++;
    }

    /* Nothing to do for system: <S397>/Nothing */

    /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S397>/CANReceive' */

    /* End of Outputs for S-Function (ec5744_caninterruptslb1): '<S381>/ReceiveandTransmitInterrupt' */
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
void VehCtrlMdel241025_2018b_amkspdlimit_SetEventsForThisBaseStep(boolean_T
  *eventFlags)
{
  /* Task runs when its counter is zero, computed via rtmStepTask macro */
  eventFlags[1] = ((boolean_T)rtmStepTask(VehCtrlMdel241025_2018b_amks_M, 1));
  eventFlags[2] = ((boolean_T)rtmStepTask(VehCtrlMdel241025_2018b_amks_M, 2));
  eventFlags[3] = ((boolean_T)rtmStepTask(VehCtrlMdel241025_2018b_amks_M, 3));
  eventFlags[4] = ((boolean_T)rtmStepTask(VehCtrlMdel241025_2018b_amks_M, 4));
  eventFlags[5] = ((boolean_T)rtmStepTask(VehCtrlMdel241025_2018b_amks_M, 5));
  eventFlags[6] = ((boolean_T)rtmStepTask(VehCtrlMdel241025_2018b_amks_M, 6));
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
  (VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[1])++;
  if ((VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[1]) > 1) {/* Sample time: [0.001s, 0.0s] */
    VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[1] = 0;
  }

  (VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[2])++;
  if ((VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[2]) > 9) {/* Sample time: [0.005s, 0.0s] */
    VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[2] = 0;
  }

  (VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[3])++;
  if ((VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[3]) > 19) {/* Sample time: [0.01s, 0.0s] */
    VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[3] = 0;
  }

  (VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[4])++;
  if ((VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[4]) > 99) {/* Sample time: [0.05s, 0.0s] */
    VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[4] = 0;
  }

  (VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[5])++;
  if ((VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[5]) > 199) {/* Sample time: [0.1s, 0.0s] */
    VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[5] = 0;
  }

  (VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[6])++;
  if ((VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[6]) > 999) {/* Sample time: [0.5s, 0.0s] */
    VehCtrlMdel241025_2018b_amks_M->Timing.TaskCounters.TID[6] = 0;
  }
}

/*
 * Output and update for atomic system:
 *    '<S8>/Timer1'
 *    '<S8>/Timer2'
 *    '<S219>/Timer'
 *    '<S221>/Timer'
 *    '<S222>/Timer'
 *    '<S222>/Timer1'
 *    '<S222>/Timer2'
 *    '<S222>/Timer3'
 *    '<S287>/Timer'
 *    '<S287>/Timer1'
 *    ...
 */
void VehCtrlMdel241025_20_Timer1(boolean_T rtu_Trigger, real32_T rtu_CountTime,
  real_T *rty_Exit, DW_Timer1_VehCtrlMdel241025_2_T *localDW)
{
  boolean_T sf_internal_predicateOutput;

  /* Chart: '<S8>/Timer1' */
  if (localDW->bitsForTID3.is_active_c5_VehCtrlMdel241025_ == 0U) {
    localDW->bitsForTID3.is_active_c5_VehCtrlMdel241025_ = 1U;
    localDW->bitsForTID3.is_c5_VehCtrlMdel241025_2018b_a = 3U;
    localDW->x += 0.01;
    *rty_Exit = 0.0;
  } else {
    switch (localDW->bitsForTID3.is_c5_VehCtrlMdel241025_2018b_a) {
     case VehCtrlMdel241025_IN_InterState:
      if (rtu_Trigger) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel241025_2018b_a = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      }
      break;

     case VehCtrlMdel241025_2018b__IN_Out:
      *rty_Exit = 1.0;
      if (!rtu_Trigger) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel241025_2018b_a = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      }
      break;

     default:
      /* case IN_Trigger: */
      *rty_Exit = 0.0;
      sf_internal_predicateOutput = (rtu_Trigger && (localDW->x < rtu_CountTime));
      if (sf_internal_predicateOutput) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel241025_2018b_a = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      } else if (localDW->x >= rtu_CountTime) {
        localDW->bitsForTID3.is_c5_VehCtrlMdel241025_2018b_a = 2U;
        *rty_Exit = 1.0;
        localDW->x = 0.0;
      } else {
        sf_internal_predicateOutput = ((localDW->x < rtu_CountTime) &&
          (!rtu_Trigger));
        if (sf_internal_predicateOutput) {
          localDW->bitsForTID3.is_c5_VehCtrlMdel241025_2018b_a = 1U;
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
 *    '<S130>/Timer'
 *    '<S217>/Timer'
 */
void VehCtrlMdel241025_201_Timer(boolean_T rtu_Trigger, real32_T rtu_CountTime,
  real_T *rty_Exit, DW_Timer_VehCtrlMdel241025_20_T *localDW)
{
  boolean_T sf_internal_predicateOutput;

  /* Chart: '<S130>/Timer' */
  if (localDW->bitsForTID3.is_active_c21_VehCtrlMdel241025 == 0U) {
    localDW->bitsForTID3.is_active_c21_VehCtrlMdel241025 = 1U;
    localDW->bitsForTID3.is_c21_VehCtrlMdel241025_2018b_ = 3U;
    localDW->x += 0.01;
    *rty_Exit = 0.0;
  } else {
    switch (localDW->bitsForTID3.is_c21_VehCtrlMdel241025_2018b_) {
     case VehCtrlMdel2410_IN_InterState_n:
      if (rtu_Trigger) {
        localDW->bitsForTID3.is_c21_VehCtrlMdel241025_2018b_ = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      }
      break;

     case VehCtrlMdel241025_2018_IN_Out_n:
      *rty_Exit = 1.0;
      if (!rtu_Trigger) {
        localDW->bitsForTID3.is_c21_VehCtrlMdel241025_2018b_ = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      }
      break;

     default:
      /* case IN_Trigger: */
      *rty_Exit = 0.0;
      sf_internal_predicateOutput = (rtu_Trigger && (localDW->x < rtu_CountTime));
      if (sf_internal_predicateOutput) {
        localDW->bitsForTID3.is_c21_VehCtrlMdel241025_2018b_ = 3U;
        localDW->x += 0.01;
        *rty_Exit = 0.0;
      } else if (localDW->x >= rtu_CountTime) {
        localDW->bitsForTID3.is_c21_VehCtrlMdel241025_2018b_ = 2U;
        *rty_Exit = 1.0;
        localDW->x = 0.0;
      } else {
        sf_internal_predicateOutput = ((localDW->x < rtu_CountTime) &&
          (!rtu_Trigger));
        if (sf_internal_predicateOutput) {
          localDW->bitsForTID3.is_c21_VehCtrlMdel241025_2018b_ = 1U;
          localDW->x = 0.0;
        }
      }
      break;
    }
  }

  /* End of Chart: '<S130>/Timer' */
}

/* Function for Chart: '<S113>/Chart2' */
static void VehC_enter_atomic_WaitForEngine(void)
{
  int32_T b_previousEvent;
  b_previousEvent = VehCtrlMdel241025_2018b_amks_DW.sfEvent;
  VehCtrlMdel241025_2018b_amks_DW.sfEvent = VehCtrlMdel2410_event_AMKCANOFF;
  if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_AMKCANenable != 0U)
  {
    switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKCANenable) {
     case VehCtrlMdel241025_2018b__IN_OFF:
      if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
          VehCtrlMdel24102_event_AMKCANON) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 2U;
      }
      break;

     case VehCtrlMdel241025_2018b_a_IN_ON:
      if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
          VehCtrlMdel2410_event_AMKCANOFF) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 1U;
      }
      break;
    }
  }

  VehCtrlMdel241025_2018b_amks_DW.sfEvent = VehCtrlMdel24102_event_AMKDCOFF;
  if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_AMKDCon != 0U) {
    switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCon) {
     case VehCtrlMdel241025_2018b__IN_OFF:
      if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
          VehCtrlMdel241025_event_AMKDCON) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCon = 2U;
      }
      break;

     case VehCtrlMdel241025_2018b_a_IN_ON:
      if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
          VehCtrlMdel24102_event_AMKDCOFF) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCon = 1U;
      }
      break;
    }
  }

  VehCtrlMdel241025_2018b_amks_DW.sfEvent = VehCtrlMdel_event_MCDCEnableOFF;
  if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MCDCEnable != 0U) {
    switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCDCEnable) {
     case VehCtrlMdel241025_2018b__IN_OFF:
      if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
          VehCtrlMdel2_event_MCDCEnableON) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCDCEnable = 2U;
      }
      break;

     case VehCtrlMdel241025_2018b_a_IN_ON:
      if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
          VehCtrlMdel_event_MCDCEnableOFF) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCDCEnable = 1U;
      }
      break;
    }
  }

  VehCtrlMdel241025_2018b_amks_DW.sfEvent = VehCtrlMdel_event_InverterFLOFF;
  if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MCFL_InverterOn !=
      0U) {
    switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn) {
     case VehCtrlMdel241025_2018b__IN_OFF:
      if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
          VehCtrlMdel2_event_InverterFLON) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn = 2U;
      }
      break;

     case VehCtrlMdel241025_2018b_a_IN_ON:
      if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
          VehCtrlMdel_event_InverterFLOFF) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn = 1U;
      }
      break;
    }
  }

  VehCtrlMdel241025_2018b_amks_DW.sfEvent = VehCtrlMdel_event_InverterFROFF;
  if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MCFR_InverterOn !=
      0U) {
    switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn) {
     case VehCtrlMdel241025_2018b__IN_OFF:
      if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
          VehCtrlMdel2_event_InverterFRON) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn = 2U;
      }
      break;

     case VehCtrlMdel241025_2018b_a_IN_ON:
      if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
          VehCtrlMdel_event_InverterFROFF) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn = 1U;
      }
      break;
    }
  }

  VehCtrlMdel241025_2018b_amks_DW.sfEvent = VehCtrlMdel2410_event_TorqueOFF;
  if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MC_TorqueCUT != 0U)
  {
    switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT) {
     case VehCtrlMdel241025_2018b__IN_OFF:
      if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
          VehCtrlMdel24102_event_TorqueON) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT = 2U;
      }
      break;

     case VehCtrlMdel241025_2018b_a_IN_ON:
      if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
          VehCtrlMdel2410_event_TorqueOFF) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
      }
      break;
    }
  }

  VehCtrlMdel241025_2018b_amks_DW.sfEvent = b_previousEvent;
  VehCtrlMdel241025_2018b_amksp_B.errorReset = 0.0;
}

/* Function for Chart: '<S113>/Chart2' */
static void VehCtrlMdel241025_2018b_VehStat(const real_T *controller_ready_e,
  const boolean_T *AND_n, const real_T *Switch_k, const real_T *Switch3, const
  real_T *Switch10)
{
  boolean_T sf_internal_predicateOutput;
  int32_T b_previousEvent;
  switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_VehStat) {
   case VehCtrlMdel241025_2018_IN_Guard:
    if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_f >= 100U) {
      b_previousEvent = VehCtrlMdel241025_2018b_amks_DW.sfEvent;
      VehCtrlMdel241025_2018b_amks_DW.sfEvent = VehCtrlMdel241025_event_EbeepON;
      if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_BeeperStat != 0U)
      {
        switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_BeeperStat) {
         case VehCtrlMdel241025_2018b__IN_OFF:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel241025_event_EbeepON) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_BeeperStat = 2U;
          }
          break;

         case VehCtrlMdel241025_2018b_a_IN_ON:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel24102_event_EbeepOFF) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_BeeperStat = 1U;
          }
          break;
        }
      }

      VehCtrlMdel241025_2018b_amks_DW.sfEvent = b_previousEvent;
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_VehStat = 7U;
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_f = 0U;
    } else {
      sf_internal_predicateOutput = ((!*AND_n) || (!Brk) || (!ACC_Release));
      if (sf_internal_predicateOutput) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_VehStat = 6U;
      } else {
        sf_internal_predicateOutput = ((!(*controller_ready_e != 0.0)) ||
          (!(*Switch_k != 0.0)) || (!(*Switch3 != 0.0)));
        if (sf_internal_predicateOutput) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_VehStat = 3U;
        }
      }
    }
    break;

   case VehCtrlMdel241025_2018b_IN_Init:
    VehCtrlMdel241025_2018b_amksp_B.errorReset = 1.0;
    if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_f >= 10U) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_VehStat = 8U;
      VehC_enter_atomic_WaitForEngine();
    }
    break;

   case VehCtrlMdel241_IN_InitStateBack:
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_VehStat = 8U;
    VehC_enter_atomic_WaitForEngine();
    break;

   case VehCtrlMdel2410_IN_MCUReadyFail:
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_VehStat = 8U;
    VehC_enter_atomic_WaitForEngine();
    break;

   case VehCtrlMdel241025_2018_IN_Ready:
    sf_internal_predicateOutput = ((!(*controller_ready_e != 0.0)) ||
      (!(*Switch_k != 0.0)) || (!(*Switch3 != 0.0)));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_VehStat = 4U;
    }
    break;

   case VehCtrlMdel241025_20_IN_Standby:
    sf_internal_predicateOutput = ((!(*Switch_k != 0.0)) || (!(*Switch3 != 0.0))
      || (!(*controller_ready_e != 0.0)));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_VehStat = 8U;
      VehC_enter_atomic_WaitForEngine();
    } else {
      sf_internal_predicateOutput = ((*AND_n) && Brk && ACC_Release &&
        (*controller_ready_e != 0.0) && (*Switch_k != 0.0) && (*Switch3 != 0.0) &&
        (*Switch10 != 0.0) && (VehCtrlMdel241025_2018b_amksp_B.Switch11 != 0.0));
      if (sf_internal_predicateOutput) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_VehStat = 1U;
        VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_f = 0U;
      }
    }
    break;

   case VehCtrlMdel241025_2018_IN_Trans:
    if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_f >= 250U) {
      b_previousEvent = VehCtrlMdel241025_2018b_amks_DW.sfEvent;
      VehCtrlMdel241025_2018b_amks_DW.sfEvent = VehCtrlMdel24102_event_EbeepOFF;
      if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_BeeperStat != 0U)
      {
        switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_BeeperStat) {
         case VehCtrlMdel241025_2018b__IN_OFF:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel241025_event_EbeepON) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_BeeperStat = 2U;
          }
          break;

         case VehCtrlMdel241025_2018b_a_IN_ON:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel24102_event_EbeepOFF) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_BeeperStat = 1U;
          }
          break;
        }
      }

      VehCtrlMdel241025_2018b_amks_DW.sfEvent = b_previousEvent;
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_VehStat = 5U;
    }
    break;

   case VehCtrlMdel241_IN_WaitForEngine:
    VehCtrlMdel241025_2018b_amksp_B.errorReset = 0.0;
    sf_internal_predicateOutput = ((*Switch_k != 0.0) && (*Switch3 != 0.0) &&
      (*controller_ready_e != 0.0));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_VehStat = 6U;
    }
    break;
  }
}

/* Function for Chart: '<S113>/Chart2' */
static void VehCtrlMdel241025_20_AMKDCready(const real_T *MCFL_bDCOn_j, const
  real_T *MCFR_bDCOn_n, const boolean_T *AND_n, const real_T *Switch_k, const
  real_T *Switch3, const real_T *Switch10)
{
  boolean_T sf_internal_predicateOutput;
  int32_T g_previousEvent;
  switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCready) {
   case VehCtrlMdel241025_201_IN_AMKCAN:
    sf_internal_predicateOutput = ((*AND_n) && Brk && ACC_Release);
    if (sf_internal_predicateOutput) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCready = 8U;
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 = 0U;
    }
    break;

   case VehCtrlMdel241_IN_AMKDCOnFinish:
    break;

   case VehCtrlMdel241_IN_DCOnCheckPass:
    sf_internal_predicateOutput =
      ((VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 >= 100U) &&
       (*MCFR_bDCOn_n != 0.0) && (*MCFL_bDCOn_j != 0.0));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCready = 4U;
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 = 0U;
      g_previousEvent = VehCtrlMdel241025_2018b_amks_DW.sfEvent;
      VehCtrlMdel241025_2018b_amks_DW.sfEvent = VehCtrlMdel2_event_MCDCEnableON;
      if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MCDCEnable != 0U)
      {
        switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCDCEnable) {
         case VehCtrlMdel241025_2018b__IN_OFF:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel2_event_MCDCEnableON) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCDCEnable = 2U;
          }
          break;

         case VehCtrlMdel241025_2018b_a_IN_ON:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel_event_MCDCEnableOFF) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCDCEnable = 1U;
          }
          break;
        }
      }

      VehCtrlMdel241025_2018b_amks_DW.sfEvent = g_previousEvent;
    }
    break;

   case VehCtrlMdel2_IN_MCDCEnableState:
    if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 >= 100U) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCready = 6U;
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 = 0U;
      g_previousEvent = VehCtrlMdel241025_2018b_amks_DW.sfEvent;
      VehCtrlMdel241025_2018b_amks_DW.sfEvent = VehCtrlMdel2_event_InverterFLON;
      if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MCFL_InverterOn
          != 0U) {
        switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn)
    {
         case VehCtrlMdel241025_2018b__IN_OFF:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel2_event_InverterFLON) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn = 2U;
          }
          break;

         case VehCtrlMdel241025_2018b_a_IN_ON:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel_event_InverterFLOFF) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn = 1U;
          }
          break;
        }
      }

      VehCtrlMdel241025_2018b_amks_DW.sfEvent = g_previousEvent;
    }
    break;

   case VehCtrlMdel24102_IN_MCDCOncheck:
    if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 >= 100U) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCready = 3U;
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 = 0U;
    }
    break;

   case VehCtr_IN_MCFL_InverterOn_State:
    if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 >= 500U) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCready = 7U;
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 = 0U;
      g_previousEvent = VehCtrlMdel241025_2018b_amks_DW.sfEvent;
      VehCtrlMdel241025_2018b_amks_DW.sfEvent = VehCtrlMdel2_event_InverterFRON;
      if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MCFR_InverterOn
          != 0U) {
        switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn)
    {
         case VehCtrlMdel241025_2018b__IN_OFF:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel2_event_InverterFRON) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn = 2U;
          }
          break;

         case VehCtrlMdel241025_2018b_a_IN_ON:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel_event_InverterFROFF) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn = 1U;
          }
          break;
        }
      }

      VehCtrlMdel241025_2018b_amks_DW.sfEvent = g_previousEvent;
    }
    break;

   case VehCtr_IN_MCFR_InverterOn_State:
    if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 >= 100U) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCready = 9U;
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 = 0U;
    }
    break;

   case VehCtrlMdel241025_IN_MC_DCready:
    if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 >= 50U) {
      g_previousEvent = VehCtrlMdel241025_2018b_amks_DW.sfEvent;
      VehCtrlMdel241025_2018b_amks_DW.sfEvent = VehCtrlMdel241025_event_AMKDCON;
      if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_AMKDCon != 0U) {
        switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCon) {
         case VehCtrlMdel241025_2018b__IN_OFF:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel241025_event_AMKDCON) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCon = 2U;
          }
          break;

         case VehCtrlMdel241025_2018b_a_IN_ON:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel24102_event_AMKDCOFF) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCon = 1U;
          }
          break;
        }
      }

      VehCtrlMdel241025_2018b_amks_DW.sfEvent = g_previousEvent;
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCready = 5U;
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 = 0U;
    }
    break;

   case VehCtrlMdel241025_IN_SYSRDYCECK:
    sf_internal_predicateOutput =
      ((VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 >= 800U) &&
       (*Switch_k != 0.0) && (*Switch3 != 0.0) && (*Switch10 != 0.0) &&
       (VehCtrlMdel241025_2018b_amksp_B.Switch11 != 0.0));
    if (sf_internal_predicateOutput) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCready = 2U;
      g_previousEvent = VehCtrlMdel241025_2018b_amks_DW.sfEvent;
      VehCtrlMdel241025_2018b_amks_DW.sfEvent = VehCtrlMdel24102_event_TorqueON;
      if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MC_TorqueCUT !=
          0U) {
        switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT) {
         case VehCtrlMdel241025_2018b__IN_OFF:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel24102_event_TorqueON) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT = 2U;
          }
          break;

         case VehCtrlMdel241025_2018b_a_IN_ON:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel2410_event_TorqueOFF) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
          }
          break;
        }
      }

      VehCtrlMdel241025_2018b_amks_DW.sfEvent = g_previousEvent;
    }
    break;

   case VehCtrlMdel241025_2018_IN_start:
    if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 >= 500U) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCready = 1U;
      g_previousEvent = VehCtrlMdel241025_2018b_amks_DW.sfEvent;
      VehCtrlMdel241025_2018b_amks_DW.sfEvent = VehCtrlMdel24102_event_AMKCANON;
      if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_AMKCANenable !=
          0U) {
        switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKCANenable) {
         case VehCtrlMdel241025_2018b__IN_OFF:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel24102_event_AMKCANON) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 2U;
          }
          break;

         case VehCtrlMdel241025_2018b_a_IN_ON:
          if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
              VehCtrlMdel2410_event_AMKCANOFF) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 1U;
          }
          break;
        }
      }

      VehCtrlMdel241025_2018b_amks_DW.sfEvent = g_previousEvent;
    }
    break;
  }
}

/* Model step function for TID0 */
void VehCtrlMdel241025_2018b_amkspdlimit_step0(void) /* Sample time: [0.0005s, 0.0s] */
{
  {                                    /* Sample time: [0.0005s, 0.0s] */
    rate_monotonic_scheduler();
  }
}

/* Model step function for TID1 */
void VehCtrlMdel241025_2018b_amkspdlimit_step1(void) /* Sample time: [0.001s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S381>/Function-Call Generator' incorporates:
   *  SubSystem: '<S381>/CCPBackground'
   */

  /* S-Function (ec5744_ccpslb): '<S396>/CCPBackground' */
  ccpBackground();
  Lin0_Background();

  /* End of Outputs for S-Function (fcncallgen): '<S381>/Function-Call Generator' */
}

/* Model step function for TID2 */
void VehCtrlMdel241025_2018b_amkspdlimit_step2(void) /* Sample time: [0.005s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S379>/5ms' incorporates:
   *  SubSystem: '<S379>/daq5ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S394>/CCPDAQ' */
  ccpDaq(0);

  /* End of Outputs for S-Function (fcncallgen): '<S379>/5ms' */
}

/* Model step function for TID3 */
void VehCtrlMdel241025_2018b_amkspdlimit_step3(void) /* Sample time: [0.01s, 0.0s] */
{
  real32_T Wdes;
  boolean_T rtb_ignition_e;
  real32_T rtb_StrWhlAngV_c;
  real_T rtb_Gain5;
  real_T rtb_Gain4;
  real_T rtb_Switch2_on;
  real32_T rtb_Gain3_o;
  real32_T rtb_Acc_POS;
  boolean_T rtb_LogicalOperator2;
  boolean_T rtb_LogicalOperator7_m;
  boolean_T rtb_LogicalOperator3;
  boolean_T rtb_Compare;
  boolean_T rtb_LowerRelop1_b;
  real_T elapseTime;
  real32_T rtb_CastToBoolean;
  real_T rtb_Yk1_l;
  real_T rtb_UkYk1;
  real_T rtb_g_mpss1;
  real32_T rtb_Add4_j;
  real32_T rtb_Add7;
  real32_T rtb_Add6;
  real32_T rtb_Switch2_mn;
  real32_T rtb_Switch2_b0;
  real32_T rtb_CastToDouble;
  boolean_T rtb_Compare_i;
  boolean_T rtb_Compare_c;
  boolean_T rtb_AND_l;
  boolean_T rtb_AND2_e;
  boolean_T rtb_Compare_b;
  real32_T rtb_VxIMU_est;
  real32_T rtb_Ax;
  real_T rtb_Yk1;
  real_T rtb_Switch2_gd;
  real_T rtb_Switch2_cn;
  real_T elapseTime_0;
  real_T rtb_Add5;
  real_T rtb_Add4_f;
  real_T rtb_UkYk1_ll;
  real32_T rtb_Add10;
  real32_T rtb_deltafalllimit_iz;
  real_T rtb_deltafalllimit_le;
  real32_T rtb_deltafalllimit_n;
  real32_T rtb_deltafalllimit_om;
  boolean_T rtb_UpperRelop_ir;
  int32_T rtb_Switch4_o;
  real32_T rtb_MaxWhlSpd_mps_n;
  uint32_T rtb_Gain1_h;
  uint32_T rtb_Add1_k;
  uint32_T FunctionCallSubsystem_ELAPS_T;
  int32_T Brk_F;
  real_T WhlSpdFL;
  real_T WhlSpdFR;
  real_T WhlSpdRR_mps;
  real_T WhlSpdRL_mps;
  real32_T FRWhlStrAng;
  real32_T Acc_POS_n;
  boolean_T rtb_LogicalOperator_idx_0;
  real32_T y;

  /* S-Function (fcncallgen): '<S3>/10ms7' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* S-Function (ec5744_asislbu3): '<S130>/Acc4' */

  /* Read the ADC conversion result of the analog signal */
  VehCtrlMdel241025_2018b_amksp_B.HV_volt= adc_read_chan(0,11);

  /* S-Function (ec5744_swislbu3): '<S130>/SwitchInput' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel241025_2018b_amksp_B.Drive_ready= ec_gpio_read(99);

  /* Logic: '<S130>/Logical Operator' */
  rtb_ignition_e = !VehCtrlMdel241025_2018b_amksp_B.Drive_ready;

  /* Chart: '<S130>/Timer' incorporates:
   *  Constant: '<S130>/Constant5'
   */
  VehCtrlMdel241025_201_Timer(rtb_ignition_e, 0.11F, &ignition,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer);

  /* S-Function (ec5744_swislbu3): '<S130>/SwitchInput1' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel241025_2018b_amksp_B.SwitchInput1= ec_gpio_read(45);

  /* Chart: '<S130>/Timer1' incorporates:
   *  Constant: '<S130>/Constant1'
   */
  if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c23_VehCtrlMdel241025
      == 0U) {
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c23_VehCtrlMdel241025 =
      1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel241025_2018b_ =
      3U;
    VehCtrlMdel241025_2018b_amks_DW.x_j += 0.01;
    AMKSWITCH = 0.0;
  } else {
    switch
      (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel241025_2018b_)
    {
     case VehCtrlMdel2410_IN_InterState_d:
      if (!VehCtrlMdel241025_2018b_amksp_B.SwitchInput1) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel241025_2018b_
          = 3U;
        VehCtrlMdel241025_2018b_amks_DW.x_j += 0.01;
        AMKSWITCH = 0.0;
      }
      break;

     case VehCtrlMdel241025_2018_IN_Out_b:
      AMKSWITCH = 1.0;
      if (VehCtrlMdel241025_2018b_amksp_B.SwitchInput1) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel241025_2018b_
          = 3U;
        VehCtrlMdel241025_2018b_amks_DW.x_j += 0.01;
        AMKSWITCH = 0.0;
      }
      break;

     default:
      /* case IN_Trigger: */
      AMKSWITCH = 0.0;
      rtb_LogicalOperator3 = ((!VehCtrlMdel241025_2018b_amksp_B.SwitchInput1) &&
        (VehCtrlMdel241025_2018b_amks_DW.x_j < 0.10999999940395355));
      if (rtb_LogicalOperator3) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel241025_2018b_
          = 3U;
        VehCtrlMdel241025_2018b_amks_DW.x_j += 0.01;
        AMKSWITCH = 0.0;
      } else if (VehCtrlMdel241025_2018b_amks_DW.x_j >= 0.10999999940395355) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel241025_2018b_
          = 2U;
        AMKSWITCH = 1.0;
        VehCtrlMdel241025_2018b_amks_DW.x_j = 0.0;
      } else {
        rtb_LogicalOperator7_m = ((VehCtrlMdel241025_2018b_amks_DW.x_j <
          0.10999999940395355) && VehCtrlMdel241025_2018b_amksp_B.SwitchInput1);
        if (rtb_LogicalOperator7_m) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel241025_2018b_
            = 1U;
          VehCtrlMdel241025_2018b_amks_DW.x_j = 0.0;
        }
      }
      break;
    }
  }

  /* End of Chart: '<S130>/Timer1' */

  /* S-Function (ec5744_swislbu3): '<S130>/SwitchInput3' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel241025_2018b_amksp_B.out2_c= ec_gpio_read(92);

  /* S-Function (ec5744_swislbu3): '<S130>/SwitchInput2' */

  /* Read the the value of the specified switch input */
  TSAL_SW_IN= ec_gpio_read(152);

  /* S-Function (ec5744_swislbu3): '<S130>/SwitchInput4' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel241025_2018b_amksp_B.SwitchInput4= ec_gpio_read(62);

  /* Chart: '<S130>/Timer2' incorporates:
   *  Constant: '<S130>/Constant3'
   */
  if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c30_VehCtrlMdel241025
      == 0U) {
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c30_VehCtrlMdel241025 =
      1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c30_VehCtrlMdel241025_2018b_ =
      3U;
    VehCtrlMdel241025_2018b_amks_DW.x += 0.01;
    VehCtrlMdel241025_2018b_amksp_B.Exit_iy = 0.0;
  } else {
    switch
      (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c30_VehCtrlMdel241025_2018b_)
    {
     case VehCtrlMdel2410_IN_InterState_d:
      if (!VehCtrlMdel241025_2018b_amksp_B.SwitchInput4) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c30_VehCtrlMdel241025_2018b_
          = 3U;
        VehCtrlMdel241025_2018b_amks_DW.x += 0.01;
        VehCtrlMdel241025_2018b_amksp_B.Exit_iy = 0.0;
      }
      break;

     case VehCtrlMdel241025_2018_IN_Out_b:
      VehCtrlMdel241025_2018b_amksp_B.Exit_iy = 1.0;
      if (VehCtrlMdel241025_2018b_amksp_B.SwitchInput4) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c30_VehCtrlMdel241025_2018b_
          = 3U;
        VehCtrlMdel241025_2018b_amks_DW.x += 0.01;
        VehCtrlMdel241025_2018b_amksp_B.Exit_iy = 0.0;
      }
      break;

     default:
      /* case IN_Trigger: */
      VehCtrlMdel241025_2018b_amksp_B.Exit_iy = 0.0;
      rtb_LogicalOperator3 = ((!VehCtrlMdel241025_2018b_amksp_B.SwitchInput4) &&
        (VehCtrlMdel241025_2018b_amks_DW.x < 0.10999999940395355));
      if (rtb_LogicalOperator3) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c30_VehCtrlMdel241025_2018b_
          = 3U;
        VehCtrlMdel241025_2018b_amks_DW.x += 0.01;
        VehCtrlMdel241025_2018b_amksp_B.Exit_iy = 0.0;
      } else if (VehCtrlMdel241025_2018b_amks_DW.x >= 0.10999999940395355) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c30_VehCtrlMdel241025_2018b_
          = 2U;
        VehCtrlMdel241025_2018b_amksp_B.Exit_iy = 1.0;
        VehCtrlMdel241025_2018b_amks_DW.x = 0.0;
      } else {
        rtb_LogicalOperator7_m = ((VehCtrlMdel241025_2018b_amks_DW.x <
          0.10999999940395355) && VehCtrlMdel241025_2018b_amksp_B.SwitchInput4);
        if (rtb_LogicalOperator7_m) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c30_VehCtrlMdel241025_2018b_
            = 1U;
          VehCtrlMdel241025_2018b_amks_DW.x = 0.0;
        }
      }
      break;
    }
  }

  /* End of Chart: '<S130>/Timer2' */

  /* SignalConversion generated from: '<S130>/key' */
  VehCtrlMdel241025_2018b_amksp_B.ModeSW_o =
    VehCtrlMdel241025_2018b_amksp_B.Exit_iy;

  /* SignalConversion generated from: '<S130>/key' */
  VehCtrlMdel241025_2018b_amksp_B.TSAL_SW_IN_i2 = TSAL_SW_IN;

  /* SignalConversion generated from: '<S130>/key' */
  VehCtrlMdel241025_2018b_amksp_B.out2_h =
    VehCtrlMdel241025_2018b_amksp_B.out2_c;

  /* SignalConversion generated from: '<S130>/key' */
  VehCtrlMdel241025_2018b_amksp_B.AMKSWITCH_bx = AMKSWITCH;

  /* SignalConversion generated from: '<S130>/key' */
  VehCtrlMdel241025_2018b_amksp_B.ignition_d = ignition;

  /* RelationalOperator: '<S207>/Compare' incorporates:
   *  Constant: '<S207>/Constant'
   */
  HV_voltValid = (VehCtrlMdel241025_2018b_amksp_B.HV_volt <= 1500);

  /* SignalConversion generated from: '<S130>/key' */
  VehCtrlMdel241025_2018b_amksp_B.HV_voltValid_kx = HV_voltValid;

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms7' */

  /* S-Function (fcncallgen): '<S3>/10ms6' incorporates:
   *  SubSystem: '<S3>/EMRAXMCU_RECIEVE'
   */
  /* S-Function (ec5744_canreceiveslb): '<S127>/CANReceive1' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF1RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf1looprx= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o3= 218089455;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o5= 8;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o2= ec_can_receive(0,1,
      CAN0BUF1RX);
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[0]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[1]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[2]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[3]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[4]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[5]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[6]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[7]= CAN0BUF1RX[can0buf1looprx];
    can0buf1looprx++;
  }

  /* Call the system: <S127>/MCU_pwr */

  /* Output and update for function-call system: '<S127>/MCU_pwr' */

  /* Outputs for Enabled SubSystem: '<S182>/MCU_VCUMeter1' incorporates:
   *  EnablePort: '<S184>/Enable'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S184>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Length = 8;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.ID =
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o3;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Extended = 1;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[0];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[1];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[2];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[3];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[4];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[5];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[6];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S184>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S184>/CAN Unpack' */
      if ((8 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Length) &&
          (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.ID !=
           INVALID_CAN_ID) ) {
        if ((218089455 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.ID)
            && (1U ==
                VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Extended) )
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data
                       [6]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data
                       [4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data
                       [2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_g.Data
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

  /* End of Outputs for SubSystem: '<S182>/MCU_VCUMeter1' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S127>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S127>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF0RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf0looprx= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o3= 218089199;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o5= 8;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o2= ec_can_receive(0,0,
      CAN0BUF0RX);
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[0]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[1]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[2]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[3]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[4]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[5]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[6]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[7]= CAN0BUF0RX[can0buf0looprx];
    can0buf0looprx++;
  }

  /* Call the system: <S127>/MCU_state */

  /* Output and update for function-call system: '<S127>/MCU_state' */

  /* Outputs for Enabled SubSystem: '<S183>/MCU_state' incorporates:
   *  EnablePort: '<S190>/Enable'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S190>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Length = 8;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.ID =
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o3;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Extended = 1;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[0];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[1];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[2];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[3];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[4];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[5];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[6];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S190>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S190>/CAN Unpack' */
      if ((8 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Length) &&
          (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.ID !=
           INVALID_CAN_ID) ) {
        if ((218089199 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.ID) &&
            (1U == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Extended) )
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[5])
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[7])
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[5])
                      & (uint8_T)(0x40U)) >> 6);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.low_VOL = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[5])
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[6])
                      & (uint8_T)(0x1U));
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.MCU_Temp_error = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[4]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.Mode = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[6])
                      & (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.motorTemp_error = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[5])
                      & (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.overCurrent = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[5])
                      & (uint8_T)(0x80U)) >> 7);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.overpower = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[5])
                      & (uint8_T)(0x10U)) >> 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.overvol = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[5])
                      & (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.Precharge = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[5])
                      & (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.Reslove_error = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[6])
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[0]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[1])
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4.Data[3])
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

  /* End of Outputs for SubSystem: '<S183>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S127>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms6' */

  /* S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S123>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF50RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf50looprx= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o3_m= 1698;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o5_b= 8;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o2_m= ec_can_receive(0,50,
      CAN0BUF50RX);
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[0]=
      CAN0BUF50RX[can0buf50looprx];
    can0buf50looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[1]=
      CAN0BUF50RX[can0buf50looprx];
    can0buf50looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[2]=
      CAN0BUF50RX[can0buf50looprx];
    can0buf50looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[3]=
      CAN0BUF50RX[can0buf50looprx];
    can0buf50looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[4]=
      CAN0BUF50RX[can0buf50looprx];
    can0buf50looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[5]=
      CAN0BUF50RX[can0buf50looprx];
    can0buf50looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[6]=
      CAN0BUF50RX[can0buf50looprx];
    can0buf50looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[7]=
      CAN0BUF50RX[can0buf50looprx];
    can0buf50looprx++;
  }

  /* Call the system: <S123>/ABS_BUS_state */

  /* Output and update for function-call system: '<S123>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S131>/IMU_state' incorporates:
   *  EnablePort: '<S132>/Enable'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o2_m > 0) {
    /* S-Function (ecucoder_canunmessage): '<S132>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Length = 8;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.ID =
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o3_m;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Extended = 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[0];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[1];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[2];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[3];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[4];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[5];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[6];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_lg[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S132>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S132>/CAN Unpack1' */
      if ((8 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Length) &&
          (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.ID !=
           INVALID_CAN_ID) ) {
        if ((1698 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.ID) &&
            (0U == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Extended)
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[
                       1]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[
                       3]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[
                       5]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[
                       7]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_ja.Data[
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

  /* End of Outputs for SubSystem: '<S131>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S123>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms3' */

  /* S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S129>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF32RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf32looprx= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o3_c= 330;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o5_d= 8;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o2_p= ec_can_receive(0,32,
      CAN0BUF32RX);
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[0]=
      CAN0BUF32RX[can0buf32looprx];
    can0buf32looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[1]=
      CAN0BUF32RX[can0buf32looprx];
    can0buf32looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[2]=
      CAN0BUF32RX[can0buf32looprx];
    can0buf32looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[3]=
      CAN0BUF32RX[can0buf32looprx];
    can0buf32looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[4]=
      CAN0BUF32RX[can0buf32looprx];
    can0buf32looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[5]=
      CAN0BUF32RX[can0buf32looprx];
    can0buf32looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[6]=
      CAN0BUF32RX[can0buf32looprx];
    can0buf32looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[7]=
      CAN0BUF32RX[can0buf32looprx];
    can0buf32looprx++;
  }

  /* Call the system: <S129>/StrWhSnis_state */

  /* Output and update for function-call system: '<S129>/StrWhSnis_state' */

  /* Outputs for Enabled SubSystem: '<S202>/IMU_state' incorporates:
   *  EnablePort: '<S203>/Enable'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S203>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Length = 8;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.ID =
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o3_c;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Extended = 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[0];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[1];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[2];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[3];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[4];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[5];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[6];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_k[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S203>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S203>/CAN Unpack1' */
      if ((8 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Length) &&
          (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.ID !=
           INVALID_CAN_ID) ) {
        if ((330 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.ID) &&
            (0U == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Extended)
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Data
                       [4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_l.Data
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

  /* End of Outputs for SubSystem: '<S202>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S129>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms4' */

  /* S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S137>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF1RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf1looprx= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o3_e= 640;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o5_a= 8;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o2_l= ec_can_receive(1,1,
      CAN1BUF1RX);
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[0]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[1]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[2]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[3]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[4]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[5]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[6]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[7]=
      CAN1BUF1RX[can1buf1looprx];
    can1buf1looprx++;
  }

  /* Call the system: <S137>/AMKMCU_state */

  /* Output and update for function-call system: '<S137>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S139>/MCU_state' incorporates:
   *  EnablePort: '<S142>/Enable'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S142>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Length = 8;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.ID =
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o3_e;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Extended = 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[0];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[1];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[2];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[3];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[4];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[5];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[6];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_l[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S142>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S142>/CAN Unpack' */
      if ((8 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Length) &&
          (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.ID !=
           INVALID_CAN_ID) ) {
        if ((640 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.ID) &&
            (0U == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Extended)
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [6]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [7]) << 8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                MCFL_ActualVelocity = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [1]) & (uint8_T)(0x80U)) >> 7);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.MCFL_bDerating = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [0]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.MCFL_bReserve = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_e.Data
                       [1]) & (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.MCFL_bWarn = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S139>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S137>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S137>/CANReceive1' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF2RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf2looprx= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o3_n= 642;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o5_a= 8;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o2_l= ec_can_receive(1,2,
      CAN1BUF2RX);
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[0]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[1]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[2]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[3]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[4]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[5]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[6]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[7]=
      CAN1BUF2RX[can1buf2looprx];
    can1buf2looprx++;
  }

  /* Call the system: <S137>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S137>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S140>/MCU_state' incorporates:
   *  EnablePort: '<S152>/Enable'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S152>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Length = 8;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.ID =
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o3_n;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Extended = 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[0];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[1];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[2];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[3];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[4];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[5];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[6];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_c[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S152>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S152>/CAN Unpack' */
      if ((8 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Length) &&
          (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.ID !=
           INVALID_CAN_ID) ) {
        if ((642 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.ID) &&
            (0U == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Extended)
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [0]);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [1]) << 8);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [2]) << 16);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [3]) << 24);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.MCFL_DiagnosticNum = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [4]);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [5]) << 8);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data
                       [6]) << 16);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_b.Data
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

  /* End of Outputs for SubSystem: '<S140>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S137>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S137>/CANReceive2' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF3RX[6]= { 0, 0, 0, 0, 0, 0 };

    uint8 can1buf3looprx= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o3= 644;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o5= 6;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o2= ec_can_receive(1,3,
      CAN1BUF3RX);
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4[0]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4[1]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4[2]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4[3]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4[4]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4[5]= CAN1BUF3RX[can1buf3looprx];
    can1buf3looprx++;
  }

  /* Call the system: <S137>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S137>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S141>/MCU_state' incorporates:
   *  EnablePort: '<S154>/Enable'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S154>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Length = 6;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.ID =
        VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o3;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Extended = 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4[0];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4[1];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4[2];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4[3];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4[4];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4[5];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S154>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S154>/CAN Unpack' */
      if ((6 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Length) &&
          (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.ID !=
           INVALID_CAN_ID) ) {
        if ((644 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.ID) &&
            (0U == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Extended)
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Data
                       [4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Data
                       [2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Data
                       [0]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_c.Data
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

  /* End of Outputs for SubSystem: '<S141>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S137>/CANReceive2' */

  /* S-Function (ec5744_canreceiveslb): '<S138>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF4RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf4looprx= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o3_i= 641;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o5_an= 8;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o2_a= ec_can_receive(1,4,
      CAN1BUF4RX);
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[0]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[1]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[2]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[3]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[4]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[5]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[6]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[7]=
      CAN1BUF4RX[can1buf4looprx];
    can1buf4looprx++;
  }

  /* Call the system: <S138>/AMKMCU_state */

  /* Output and update for function-call system: '<S138>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S158>/MCU_state' incorporates:
   *  EnablePort: '<S161>/Enable'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o2_a > 0) {
    /* S-Function (ecucoder_canunmessage): '<S161>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Length = 8;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.ID =
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o3_i;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Extended = 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[0];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[1];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[2];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[3];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[4];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[5];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[6];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_g[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S161>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S161>/CAN Unpack' */
      if ((8 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Length) &&
          (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.ID !=
           INVALID_CAN_ID) ) {
        if ((641 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.ID) &&
            (0U == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Extended)
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [6]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [1]) & (uint8_T)(0x80U)) >> 7);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.MCFR_bDerating = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [1]) & (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.MCFR_bQuitDCOn = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [0]);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.MCFR_bReserve = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_j.Data
                       [1]) & (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.MCFR_bWarn = result;
              }
            }
          }
        }
      }
    }
  }

  /* End of Outputs for SubSystem: '<S158>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S138>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S138>/CANReceive1' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF5RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can1buf5looprx= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o3_h= 643;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o5_j= 8;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o2_o= ec_can_receive(1,5,
      CAN1BUF5RX);
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[0]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[1]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[2]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[3]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[4]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[5]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[6]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[7]=
      CAN1BUF5RX[can1buf5looprx];
    can1buf5looprx++;
  }

  /* Call the system: <S138>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S138>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S159>/MCU_state' incorporates:
   *  EnablePort: '<S170>/Enable'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o2_o > 0) {
    /* S-Function (ecucoder_canunmessage): '<S170>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Length = 8;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.ID =
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o3_h;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Extended = 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[0];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[1];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[2];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[3];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[4];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[5];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[6];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive1_o4_j[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S170>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S170>/CAN Unpack' */
      if ((8 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Length) &&
          (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.ID !=
           INVALID_CAN_ID) ) {
        if ((643 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.ID) &&
            (0U == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Extended)
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [0]);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [1]) << 8);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [2]) << 16);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [3]) << 24);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.MCFR_DiagnosticNum = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [4]);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [5]) << 8);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data
                       [6]) << 16);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_p.Data
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

  /* End of Outputs for SubSystem: '<S159>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S138>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S138>/CANReceive2' */

  /* Receive CAN message */
  {
    uint8 CAN1BUF0RX[6]= { 0, 0, 0, 0, 0, 0 };

    uint8 can1buf0looprx= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o3_j= 645;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o5_e= 6;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o2_p= ec_can_receive(1,0,
      CAN1BUF0RX);
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4_k[0]=
      CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4_k[1]=
      CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4_k[2]=
      CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4_k[3]=
      CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4_k[4]=
      CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4_k[5]=
      CAN1BUF0RX[can1buf0looprx];
    can1buf0looprx++;
  }

  /* Call the system: <S138>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S138>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S160>/MCU_state' incorporates:
   *  EnablePort: '<S172>/Enable'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S172>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Length = 6;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.ID =
        VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o3_j;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Extended = 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4_k[0];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4_k[1];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4_k[2];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4_k[3];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4_k[4];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive2_o4_k[5];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S172>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S172>/CAN Unpack' */
      if ((6 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Length) &&
          (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.ID !=
           INVALID_CAN_ID) ) {
        if ((645 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.ID) &&
            (0U == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Extended)
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Data
                       [4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Data
                       [2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Data
                       [0]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_n.Data
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

  /* End of Outputs for SubSystem: '<S160>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S138>/CANReceive2' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms5' */

  /* S-Function (fcncallgen): '<S3>/10ms' incorporates:
   *  SubSystem: '<S3>/AccBrk_BUS'
   */
  /* S-Function (ec5744_asislbu3): '<S125>/Acc3' */

  /* Read the ADC conversion result of the analog signal */
  Acc1= adc_read_chan(1,2);

  /* S-Function (ec5744_asislbu3): '<S125>/Acc4' */

  /* Read the ADC conversion result of the analog signal */
  Acc2= adc_read_chan(1,4);

  /* S-Function (ec5744_asislbu3): '<S125>/Brk1' */

  /* Read the ADC conversion result of the analog signal */
  Brk1= adc_read_chan(1,0);

  /* S-Function (ec5744_asislbu3): '<S125>/Brk2' */

  /* Read the ADC conversion result of the analog signal */
  Brk2= adc_read_chan(0,13);

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms' */

  /* S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* S-Function (ec5744_canreceiveslb): '<S128>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF27RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf27looprx= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o3_cz= 513;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o5_m= 8;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o2_ma= ec_can_receive(0,27,
      CAN0BUF27RX);
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[0]=
      CAN0BUF27RX[can0buf27looprx];
    can0buf27looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[1]=
      CAN0BUF27RX[can0buf27looprx];
    can0buf27looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[2]=
      CAN0BUF27RX[can0buf27looprx];
    can0buf27looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[3]=
      CAN0BUF27RX[can0buf27looprx];
    can0buf27looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[4]=
      CAN0BUF27RX[can0buf27looprx];
    can0buf27looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[5]=
      CAN0BUF27RX[can0buf27looprx];
    can0buf27looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[6]=
      CAN0BUF27RX[can0buf27looprx];
    can0buf27looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[7]=
      CAN0BUF27RX[can0buf27looprx];
    can0buf27looprx++;
  }

  /* Call the system: <S128>/IMU_state */

  /* Output and update for function-call system: '<S128>/IMU_state' */

  /* Outputs for Enabled SubSystem: '<S197>/MCU_state' incorporates:
   *  EnablePort: '<S198>/Enable'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o2_ma > 0) {
    /* S-Function (ecucoder_canunmessage): '<S198>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Length = 8;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.ID =
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o3_cz;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Extended = 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[0];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[1];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[2];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[3];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[4];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[5];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[6];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_i[7];
      canunpackloop++;
    }

    /* S-Function (scanunpack): '<S198>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S198>/CAN Unpack' */
      if ((8 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Length) &&
          (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.ID !=
           INVALID_CAN_ID) ) {
        if ((513 == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.ID) &&
            (0U == VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Extended)
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [0]) & (uint8_T)(0x2U)) >> 1);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.CANUnpack_o1 = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [5]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [0]) & (uint8_T)(0x4U)) >> 2);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.CANUnpack_o3 = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [3]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [0]) & (uint8_T)(0xF0U)) >> 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.CANUnpack_o5 = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [0]) & (uint8_T)(0x8U)) >> 3);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                VehCtrlMdel241025_2018b_amksp_B.CANUnpack_o6 = result;
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
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data
                       [1]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_h.Data
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

  /* End of Outputs for SubSystem: '<S197>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S128>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms2' */

  /* S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S126>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN0BUF3RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can0buf3looprx= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o3_l= 408961267;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o5_de= 8;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o2_k= ec_can_receive(0,3,
      CAN0BUF3RX);
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[0]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[1]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[2]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[3]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[4]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[5]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[6]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[7]=
      CAN0BUF3RX[can0buf3looprx];
    can0buf3looprx++;
  }

  /* Call the system: <S126>/ABS_BUS_state */

  /* Output and update for function-call system: '<S126>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S180>/IMU_state' incorporates:
   *  EnablePort: '<S181>/Enable'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o2_k > 0) {
    /* S-Function (ecucoder_canunmessage): '<S181>/CANUnPackMessage4' */

    /*Unpack CAN message*/
    {
      uint8 canunpackloop= 0;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_i.Length = 8;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_i.ID =
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o3_l;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_i.Extended = 1;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[0];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[1];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[2];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[3];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[4];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[5];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[6];
      canunpackloop++;
      VehCtrlMdel241025_2018b_amksp_B.CANUnPackMessage4_i.Data[canunpackloop]=
        VehCtrlMdel241025_2018b_amksp_B.CANReceive3_o4_p[7];
      canunpackloop++;
    }
  }

  /* End of Outputs for SubSystem: '<S180>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S126>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms1' */

  /* S-Function (fcncallgen): '<S4>/10ms' incorporates:
   *  SubSystem: '<S4>/Function-Call Subsystem'
   */
  if (VehCtrlMdel241025_2018b_amks_DW.FunctionCallSubsystem_RESET_ELA) {
    FunctionCallSubsystem_ELAPS_T = 0U;
  } else {
    FunctionCallSubsystem_ELAPS_T =
      VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3 -
      VehCtrlMdel241025_2018b_amks_DW.FunctionCallSubsystem_PREV_T;
  }

  VehCtrlMdel241025_2018b_amks_DW.FunctionCallSubsystem_PREV_T =
    VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;
  VehCtrlMdel241025_2018b_amks_DW.FunctionCallSubsystem_RESET_ELA = false;

  /* Lookup_n-D: '<S217>/1-D Lookup Table1' */
  F_BrkPrs = look1_iu16bflftfIu16_binlc(Brk1,
    VehCtrlMdel241025_2018b__ConstP.pooled74,
    VehCtrlMdel241025_2018b__ConstP.pooled74, 1U);

  /* DataTypeConversion: '<S217>/Data Type Conversion' */
  rtb_Acc_POS = F_BrkPrs;

  /* SignalConversion generated from: '<S215>/Out1' */
  Brk_F = (int32_T)rtb_Acc_POS;

  /* Gain: '<S217>/Gain2' */
  rtb_Gain1_h = 45875U * Acc2;

  /* Gain: '<S217>/Gain3' incorporates:
   *  UnitDelay: '<S217>/Unit Delay1'
   */
  rtb_Add1_k = 39322U * VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S217>/Add3' */
  Acc_vol2 = (rtb_Add1_k >> 1) + rtb_Gain1_h;

  /* Gain: '<S217>/Gain' */
  rtb_Add1_k = 45875U * Acc1;

  /* UnitDelay: '<S217>/Unit Delay' incorporates:
   *  UnitDelay: '<S217>/Unit Delay1'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_fm =
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_k;

  /* Gain: '<S217>/Gain1' incorporates:
   *  UnitDelay: '<S217>/Unit Delay1'
   */
  rtb_Gain1_h = 39322U * VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S217>/Add2' */
  Acc_vol = (rtb_Gain1_h >> 1) + rtb_Add1_k;

  /* Sum: '<S217>/Add1' */
  rtb_Add1_k = Acc_vol - Acc_vol2;

  /* RelationalOperator: '<S231>/Compare' */
  rtb_ignition_e = (rtb_Add1_k > 65536000U);

  /* RelationalOperator: '<S223>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol <= 32768000U);

  /* RelationalOperator: '<S224>/Compare' */
  rtb_LogicalOperator7_m = (Acc_vol >= 294912000U);

  /* Logic: '<S217>/Logical Operator' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_LogicalOperator7_m);

  /* RelationalOperator: '<S227>/Compare' */
  rtb_LogicalOperator7_m = (Acc_vol2 <= 32768000U);

  /* RelationalOperator: '<S228>/Compare' */
  rtb_LogicalOperator3 = (Acc_vol2 >= 294912000U);

  /* Logic: '<S217>/Logical Operator1' */
  rtb_LogicalOperator7_m = (rtb_LogicalOperator7_m || rtb_LogicalOperator3);

  /* Logic: '<S217>/Logical Operator2' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_LogicalOperator7_m);

  /* Lookup_n-D: '<S217>/1-D Lookup Table4' */
  Acc_POS = look1_iu32n16bflftfIu32_binlc(Acc_vol,
    VehCtrlMdel241025_2018b__ConstP.uDLookupTable4_bp01Data,
    VehCtrlMdel241025_2018b__ConstP.pooled75, 1U);

  /* DataTypeConversion: '<S217>/Data Type Conversion1' */
  rtb_Acc_POS = (real32_T)Acc_POS * 1.52587891E-5F;

  /* RelationalOperator: '<S229>/Compare' incorporates:
   *  Constant: '<S229>/Constant'
   */
  rtb_LogicalOperator3 = (rtb_Acc_POS > 100.0F);

  /* Lookup_n-D: '<S217>/1-D Lookup Table3' */
  Acc_POS2 = look1_iu32n16bflftfIu32_binlc(Acc_vol2,
    VehCtrlMdel241025_2018b__ConstP.uDLookupTable3_bp01Data_b,
    VehCtrlMdel241025_2018b__ConstP.pooled75, 1U);

  /* DataTypeConversion: '<S217>/Data Type Conversion4' */
  rtb_Acc_POS = (real32_T)Acc_POS2 * 1.52587891E-5F;

  /* RelationalOperator: '<S230>/Compare' incorporates:
   *  Constant: '<S230>/Constant'
   */
  rtb_LogicalOperator7_m = (rtb_Acc_POS > 100.0F);

  /* Logic: '<S217>/Logical Operator3' */
  rtb_LogicalOperator3 = (rtb_LogicalOperator3 || rtb_LogicalOperator7_m);

  /* RelationalOperator: '<S232>/Compare' incorporates:
   *  Constant: '<S232>/Constant'
   */
  rtb_LogicalOperator7_m = (Brk1 <= 300);

  /* RelationalOperator: '<S233>/Compare' incorporates:
   *  Constant: '<S233>/Constant'
   */
  rtb_Compare = (Brk1 >= 4500);

  /* Logic: '<S217>/Logical Operator5' */
  rtb_LogicalOperator7_m = (rtb_LogicalOperator7_m || rtb_Compare);

  /* RelationalOperator: '<S225>/Compare' incorporates:
   *  Constant: '<S225>/Constant'
   */
  rtb_Compare = (Brk2 <= 300);

  /* RelationalOperator: '<S226>/Compare' incorporates:
   *  Constant: '<S226>/Constant'
   */
  rtb_LowerRelop1_b = (Brk2 >= 4500);

  /* Logic: '<S217>/Logical Operator6' */
  rtb_Compare = (rtb_Compare || rtb_LowerRelop1_b);

  /* Logic: '<S217>/Logical Operator7' */
  rtb_LogicalOperator7_m = (rtb_LogicalOperator7_m || rtb_Compare);

  /* Logic: '<S217>/Logical Operator4' */
  rtb_ignition_e = (rtb_LogicalOperator3 || rtb_ignition_e ||
                    rtb_LogicalOperator2 || rtb_LogicalOperator7_m);

  /* Chart: '<S217>/Timer' incorporates:
   *  Constant: '<S217>/Constant1'
   */
  VehCtrlMdel241025_201_Timer(rtb_ignition_e, 0.11F, &Trq_CUT,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer_a);

  /* UnitDelay: '<S253>/Delay Input2'
   *
   * Block description for '<S253>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain3_o = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_l2;

  /* SampleTimeMath: '<S253>/sample time'
   *
   * About '<S253>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S253>/delta rise limit' */
  rtb_StrWhlAngV_c = (real32_T)(1200.0 * elapseTime);

  /* DataTypeConversion: '<S219>/Cast To Boolean' */
  rtb_CastToBoolean = (real32_T)StrWhlAng;

  /* Sum: '<S253>/Difference Inputs1'
   *
   * Block description for '<S253>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_CastToBoolean -= rtb_Gain3_o;

  /* RelationalOperator: '<S256>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_CastToBoolean > rtb_StrWhlAngV_c);

  /* Switch: '<S256>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S253>/delta fall limit' */
    rtb_deltafalllimit_iz = (real32_T)(-1200.0 * elapseTime);

    /* RelationalOperator: '<S256>/UpperRelop' */
    rtb_ignition_e = (rtb_CastToBoolean < rtb_deltafalllimit_iz);

    /* Switch: '<S256>/Switch' */
    if (rtb_ignition_e) {
      rtb_CastToBoolean = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S256>/Switch' */
    rtb_StrWhlAngV_c = rtb_CastToBoolean;
  }

  /* End of Switch: '<S256>/Switch2' */

  /* Sum: '<S253>/Difference Inputs2' incorporates:
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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_l2 = rtb_StrWhlAngV_c +
    rtb_Gain3_o;

  /* Abs: '<S219>/Abs' incorporates:
   *  UnitDelay: '<S253>/Delay Input2'
   *
   * Block description for '<S253>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_StrWhlAngV_c = fabsf(VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_l2);

  /* RelationalOperator: '<S252>/Compare' incorporates:
   *  Constant: '<S252>/Constant'
   */
  rtb_ignition_e = (rtb_StrWhlAngV_c > 120.0F);

  /* Chart: '<S219>/Timer' incorporates:
   *  Constant: '<S219>/Constant5'
   */
  VehCtrlMdel241025_20_Timer1(rtb_ignition_e, 0.11F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_on,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer_k);

  /* Logic: '<S221>/OR' */
  rtb_ignition_e = ((MCFL_bError != 0.0) || (MCFR_bError != 0.0));

  /* Chart: '<S221>/Timer' incorporates:
   *  Constant: '<S221>/Constant1'
   */
  VehCtrlMdel241025_20_Timer1(rtb_ignition_e, 0.2F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_n,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer_p);

  /* RelationalOperator: '<S264>/Compare' incorporates:
   *  Constant: '<S264>/Constant'
   */
  rtb_LogicalOperator7_m = (VehCtrlMdel241025_2018b_amksp_B.Exit_n > 0.0);

  /* UnitDelay: '<S262>/Delay Input1'
   *
   * Block description for '<S262>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = VehCtrlMdel241025_2018b_amks_DW.DelayInput1_DSTATE_a;

  /* RelationalOperator: '<S262>/FixPt Relational Operator' */
  rtb_Compare = ((int32_T)rtb_LogicalOperator7_m > (int32_T)rtb_LowerRelop1_b);

  /* Chart: '<S221>/Chart' */
  rtb_Add1_k = VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3 -
    VehCtrlMdel241025_2018b_amks_DW.previousTicks_j;
  VehCtrlMdel241025_2018b_amks_DW.previousTicks_j =
    VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;
  if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_h + rtb_Add1_k <= 31U)
  {
    VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_h = (uint8_T)
      (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_h + rtb_Add1_k);
  } else {
    VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_h = 31U;
  }

  if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c28_VehCtrlMdel241025
      == 0U) {
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c28_VehCtrlMdel241025 =
      1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c28_VehCtrlMdel241025_2018b_ =
      2U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_STATEON_d = 2U;
    VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_h = 0U;
    rtb_ignition_e = false;
  } else if
      (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c28_VehCtrlMdel241025_2018b_
       == VehCtrlMdel241025_2_IN_STATEOFF) {
    rtb_ignition_e = true;
  } else {
    /* case IN_STATEON: */
    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_STATEON_d ==
        VehCtrlMdel241025_2018b_IN_ON_d) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_STATEON_d = 0U;
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c28_VehCtrlMdel241025_2018b_
        = 1U;
      rtb_ignition_e = true;
    } else {
      /* case IN_initstate: */
      rtb_ignition_e = false;
      rtb_LogicalOperator3 =
        ((VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_h >= 20U) &&
         rtb_Compare);
      if (rtb_LogicalOperator3) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_STATEON_d = 1U;
        rtb_ignition_e = true;
      }
    }
  }

  /* End of Chart: '<S221>/Chart' */

  /* UnitDelay: '<S273>/Delay Input2'
   *
   * Block description for '<S273>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE;

  /* SampleTimeMath: '<S273>/sample time'
   *
   * About '<S273>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S273>/delta rise limit' */
  rtb_Gain5 = 10.0 * elapseTime;

  /* Sum: '<S273>/Difference Inputs1'
   *
   * Block description for '<S273>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RL - rtb_Yk1_l;

  /* RelationalOperator: '<S281>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain5);

  /* Switch: '<S281>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S273>/delta fall limit' */
    rtb_deltafalllimit_le = -10.0 * elapseTime;

    /* RelationalOperator: '<S281>/UpperRelop' */
    rtb_Compare = (rtb_UkYk1 < rtb_deltafalllimit_le);

    /* Switch: '<S281>/Switch' */
    if (rtb_Compare) {
      rtb_UkYk1 = rtb_deltafalllimit_le;
    }

    /* End of Switch: '<S281>/Switch' */
    rtb_Gain5 = rtb_UkYk1;
  }

  /* End of Switch: '<S281>/Switch2' */

  /* Saturate: '<S222>/Saturation' incorporates:
   *  Sum: '<S273>/Difference Inputs2'
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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE = rtb_Gain5 + rtb_Yk1_l;
  if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE > 30.0) {
    rtb_Gain5 = 30.0;
  } else if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE < 0.0) {
    rtb_Gain5 = 0.0;
  } else {
    rtb_Gain5 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE;
  }

  /* End of Saturate: '<S222>/Saturation' */

  /* Gain: '<S222>/Gain' */
  rtb_Gain5 *= 0.27777777777777779;

  /* RelationalOperator: '<S265>/Compare' incorporates:
   *  Constant: '<S265>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain5 >= 0.0);

  /* RelationalOperator: '<S266>/Compare' incorporates:
   *  Constant: '<S266>/Constant'
   */
  rtb_Compare = (rtb_Gain5 < 40.0);

  /* Logic: '<S222>/OR' */
  rtb_Compare = (rtb_LowerRelop1_b || rtb_Compare);

  /* Chart: '<S222>/Timer' incorporates:
   *  Constant: '<S222>/Constant5'
   */
  VehCtrlMdel241025_20_Timer1(rtb_Compare, 0.11F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_le,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer_b);

  /* UnitDelay: '<S274>/Delay Input2'
   *
   * Block description for '<S274>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_b;

  /* SampleTimeMath: '<S274>/sample time'
   *
   * About '<S274>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S274>/delta rise limit' */
  rtb_Gain4 = 10.0 * elapseTime;

  /* Sum: '<S274>/Difference Inputs1'
   *
   * Block description for '<S274>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RR - rtb_Yk1_l;

  /* RelationalOperator: '<S282>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain4);

  /* Switch: '<S282>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S274>/delta fall limit' */
    rtb_deltafalllimit_le = -10.0 * elapseTime;

    /* RelationalOperator: '<S282>/UpperRelop' */
    rtb_Compare = (rtb_UkYk1 < rtb_deltafalllimit_le);

    /* Switch: '<S282>/Switch' */
    if (rtb_Compare) {
      rtb_UkYk1 = rtb_deltafalllimit_le;
    }

    /* End of Switch: '<S282>/Switch' */
    rtb_Gain4 = rtb_UkYk1;
  }

  /* End of Switch: '<S282>/Switch2' */

  /* Saturate: '<S222>/Saturation1' incorporates:
   *  Sum: '<S274>/Difference Inputs2'
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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_b = rtb_Gain4 + rtb_Yk1_l;
  if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_b > 30.0) {
    rtb_Gain4 = 30.0;
  } else if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_b < 0.0) {
    rtb_Gain4 = 0.0;
  } else {
    rtb_Gain4 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_b;
  }

  /* End of Saturate: '<S222>/Saturation1' */

  /* Gain: '<S222>/Gain3' */
  rtb_Gain4 *= 0.27777777777777779;

  /* RelationalOperator: '<S267>/Compare' incorporates:
   *  Constant: '<S267>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain4 >= 0.0);

  /* RelationalOperator: '<S268>/Compare' incorporates:
   *  Constant: '<S268>/Constant'
   */
  rtb_Compare = (rtb_Gain4 < 40.0);

  /* Logic: '<S222>/OR1' */
  rtb_Compare = (rtb_LowerRelop1_b || rtb_Compare);

  /* Chart: '<S222>/Timer1' incorporates:
   *  Constant: '<S222>/Constant1'
   */
  VehCtrlMdel241025_20_Timer1(rtb_Compare, 0.11F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_is,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer1_n);

  /* UnitDelay: '<S275>/Delay Input2'
   *
   * Block description for '<S275>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_h;

  /* SampleTimeMath: '<S275>/sample time'
   *
   * About '<S275>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S275>/delta rise limit' */
  rtb_Switch2_on = 10.0 * elapseTime;

  /* Sum: '<S275>/Difference Inputs1'
   *
   * Block description for '<S275>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = MCFR_ActualVelocity - rtb_Yk1_l;

  /* RelationalOperator: '<S283>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Switch2_on);

  /* Switch: '<S283>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S275>/delta fall limit' */
    rtb_deltafalllimit_le = -10.0 * elapseTime;

    /* RelationalOperator: '<S283>/UpperRelop' */
    rtb_Compare = (rtb_UkYk1 < rtb_deltafalllimit_le);

    /* Switch: '<S283>/Switch' */
    if (rtb_Compare) {
      rtb_UkYk1 = rtb_deltafalllimit_le;
    }

    /* End of Switch: '<S283>/Switch' */
    rtb_Switch2_on = rtb_UkYk1;
  }

  /* End of Switch: '<S283>/Switch2' */

  /* Saturate: '<S222>/Saturation2' incorporates:
   *  Sum: '<S275>/Difference Inputs2'
   *  UnitDelay: '<S275>/Delay Input2'
   *
   * Block description for '<S275>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S275>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_h = rtb_Switch2_on +
    rtb_Yk1_l;
  if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_h > 30.0) {
    rtb_Switch2_on = 30.0;
  } else if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_h < 0.0) {
    rtb_Switch2_on = 0.0;
  } else {
    rtb_Switch2_on = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_h;
  }

  /* End of Saturate: '<S222>/Saturation2' */

  /* Gain: '<S222>/Gain1' */
  rtb_Switch2_on *= 0.002235050147492625;

  /* RelationalOperator: '<S269>/Compare' incorporates:
   *  Constant: '<S269>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Switch2_on >= 0.0);

  /* RelationalOperator: '<S270>/Compare' incorporates:
   *  Constant: '<S270>/Constant'
   */
  rtb_Compare = (rtb_Switch2_on < 40.0);

  /* Logic: '<S222>/OR2' */
  rtb_Compare = (rtb_LowerRelop1_b || rtb_Compare);

  /* Chart: '<S222>/Timer2' incorporates:
   *  Constant: '<S222>/Constant4'
   */
  VehCtrlMdel241025_20_Timer1(rtb_Compare, 0.11F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_o,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer2_l);

  /* UnitDelay: '<S276>/Delay Input2'
   *
   * Block description for '<S276>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_n;

  /* SampleTimeMath: '<S276>/sample time'
   *
   * About '<S276>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S276>/delta rise limit' */
  rtb_UkYk1 = 10.0 * elapseTime;

  /* Sum: '<S276>/Difference Inputs1'
   *
   * Block description for '<S276>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_le = MCFL_ActualVelocity - rtb_Yk1_l;

  /* RelationalOperator: '<S284>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_le > rtb_UkYk1);

  /* Switch: '<S284>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S276>/delta fall limit' */
    rtb_UkYk1 = -10.0 * elapseTime;

    /* RelationalOperator: '<S284>/UpperRelop' */
    rtb_Compare = (rtb_deltafalllimit_le < rtb_UkYk1);

    /* Switch: '<S284>/Switch' */
    if (rtb_Compare) {
      rtb_deltafalllimit_le = rtb_UkYk1;
    }

    /* End of Switch: '<S284>/Switch' */
    rtb_UkYk1 = rtb_deltafalllimit_le;
  }

  /* End of Switch: '<S284>/Switch2' */

  /* Saturate: '<S222>/Saturation3' incorporates:
   *  Sum: '<S276>/Difference Inputs2'
   *  UnitDelay: '<S276>/Delay Input2'
   *
   * Block description for '<S276>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S276>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_n = rtb_UkYk1 + rtb_Yk1_l;
  if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_n > 30.0) {
    rtb_UkYk1 = 30.0;
  } else if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_n < 0.0) {
    rtb_UkYk1 = 0.0;
  } else {
    rtb_UkYk1 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_n;
  }

  /* End of Saturate: '<S222>/Saturation3' */

  /* Gain: '<S222>/Gain2' */
  rtb_UkYk1 *= 0.002235050147492625;

  /* RelationalOperator: '<S271>/Compare' incorporates:
   *  Constant: '<S271>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_UkYk1 >= 0.0);

  /* RelationalOperator: '<S272>/Compare' incorporates:
   *  Constant: '<S272>/Constant'
   */
  rtb_Compare = (rtb_UkYk1 < 40.0);

  /* Logic: '<S222>/OR3' */
  rtb_Compare = (rtb_LowerRelop1_b || rtb_Compare);

  /* Chart: '<S222>/Timer3' incorporates:
   *  Constant: '<S222>/Constant8'
   */
  VehCtrlMdel241025_20_Timer1(rtb_Compare, 0.11F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_hj,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer3);

  /* SignalConversion generated from: '<S215>/Out1' */
  WhlSpdFL = rtb_UkYk1;

  /* SignalConversion generated from: '<S215>/Out1' */
  WhlSpdFR = rtb_Switch2_on;

  /* SignalConversion generated from: '<S215>/Out1' */
  WhlSpdRR_mps = rtb_Gain4;

  /* SignalConversion generated from: '<S215>/Out1' */
  WhlSpdRL_mps = rtb_Gain5;

  /* Gain: '<S219>/Gain' incorporates:
   *  UnitDelay: '<S253>/Delay Input2'
   *
   * Block description for '<S253>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_StrWhlAngV_c = 0.7F *
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_l2;

  /* UnitDelay: '<S219>/Unit Delay' */
  rtb_Gain3_o = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_o2;

  /* Gain: '<S219>/Gain1' */
  rtb_Gain3_o *= 0.3F;

  /* Sum: '<S219>/Add2' */
  rtb_StrWhlAngV_c += rtb_Gain3_o;

  /* Lookup_n-D: '<S219>/1-D Lookup Table' */
  rtb_Gain3_o = look1_iflf_binlx(rtb_StrWhlAngV_c,
    VehCtrlMdel241025_2018b__ConstP.uDLookupTable_bp01Data,
    VehCtrlMdel241025_2018b__ConstP.uDLookupTable_tableData, 23U);

  /* SignalConversion generated from: '<S215>/Out1' */
  rtb_deltafalllimit_iz = rtb_Gain3_o;

  /* Lookup_n-D: '<S219>/1-D Lookup Table1' */
  rtb_Gain3_o = look1_iflf_binlx(rtb_StrWhlAngV_c,
    VehCtrlMdel241025_2018b__ConstP.uDLookupTable1_bp01Data_h,
    VehCtrlMdel241025_2018b__ConstP.uDLookupTable1_tableData_b, 23U);

  /* SignalConversion generated from: '<S215>/Out1' */
  FRWhlStrAng = rtb_Gain3_o;

  /* SignalConversion generated from: '<S215>/Out1' */
  rtb_CastToBoolean = rtb_StrWhlAngV_c;

  /* Sum: '<S217>/Add' */
  rtb_StrWhlAngV_c = (real32_T)Acc_POS * 1.52587891E-5F + rtb_Acc_POS;

  /* Product: '<S217>/Divide' incorporates:
   *  Constant: '<S217>/Constant'
   */
  rtb_Gain3_o = (real32_T)(rtb_StrWhlAngV_c / 2.0);

  /* SignalConversion generated from: '<S215>/Out1' */
  Acc_POS_n = rtb_Gain3_o;

  /* UnitDelay: '<S243>/Delay Input2'
   *
   * Block description for '<S243>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_l;

  /* SampleTimeMath: '<S243>/sample time'
   *
   * About '<S243>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S243>/delta rise limit' incorporates:
   *  Constant: '<S242>/Constant'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* UnitDelay: '<S242>/Unit Delay' */
  rtb_Gain4 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE;

  /* Gain: '<S242>/Gain1' */
  rtb_Gain4 *= 0.3;

  /* Gain: '<S218>/g_mpss' incorporates:
   *  UnitDelay: '<S242>/Unit Delay'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE = 9.8 * IMU_Ay_Value;

  /* Gain: '<S242>/Gain' incorporates:
   *  UnitDelay: '<S242>/Unit Delay'
   */
  rtb_Gain5 = 0.7 * VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE;

  /* Sum: '<S242>/Add2' */
  rtb_Yk1_l = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S243>/Difference Inputs1'
   *
   * Block description for '<S243>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Yk1_l -= rtb_UkYk1;

  /* RelationalOperator: '<S249>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Yk1_l > rtb_Switch2_on);

  /* Switch: '<S249>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S243>/delta fall limit' incorporates:
     *  Constant: '<S242>/Constant1'
     */
    rtb_deltafalllimit_le = -5000.0 * elapseTime;

    /* RelationalOperator: '<S249>/UpperRelop' */
    rtb_Compare = (rtb_Yk1_l < rtb_deltafalllimit_le);

    /* Switch: '<S249>/Switch' */
    if (rtb_Compare) {
      rtb_Yk1_l = rtb_deltafalllimit_le;
    }

    /* End of Switch: '<S249>/Switch' */
    rtb_Switch2_on = rtb_Yk1_l;
  }

  /* End of Switch: '<S249>/Switch2' */

  /* Sum: '<S243>/Difference Inputs2' incorporates:
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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_l = rtb_Switch2_on +
    rtb_UkYk1;

  /* RelationalOperator: '<S246>/LowerRelop1' incorporates:
   *  Constant: '<S242>/Constant6'
   *  UnitDelay: '<S243>/Delay Input2'
   *
   * Block description for '<S243>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_l >
                       1.5);

  /* Switch: '<S246>/Switch2' incorporates:
   *  Constant: '<S242>/Constant6'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S246>/UpperRelop' incorporates:
     *  Constant: '<S242>/Constant7'
     *  UnitDelay: '<S243>/Delay Input2'
     *
     * Block description for '<S243>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_Compare = (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_l < -1.5);

    /* Switch: '<S246>/Switch' incorporates:
     *  Constant: '<S242>/Constant7'
     *  UnitDelay: '<S243>/Delay Input2'
     *
     * Block description for '<S243>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_Compare) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_l;
    }

    /* End of Switch: '<S246>/Switch' */
  }

  /* End of Switch: '<S246>/Switch2' */

  /* SignalConversion generated from: '<S215>/Out1' */
  rtb_Yk1_l = rtb_UkYk1;

  /* UnitDelay: '<S244>/Delay Input2'
   *
   * Block description for '<S244>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_m;

  /* SampleTimeMath: '<S244>/sample time'
   *
   * About '<S244>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S244>/delta rise limit' incorporates:
   *  Constant: '<S242>/Constant2'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S218>/g_mpss1' */
  rtb_g_mpss1 = 9.8 * IMU_Ax_Value;

  /* Gain: '<S242>/Gain2' */
  rtb_Gain4 = 0.7 * rtb_g_mpss1;

  /* UnitDelay: '<S242>/Unit Delay1' */
  rtb_Gain5 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE;

  /* Gain: '<S242>/Gain3' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S242>/Add1' */
  rtb_deltafalllimit_le = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S244>/Difference Inputs1'
   *
   * Block description for '<S244>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_le -= rtb_UkYk1;

  /* RelationalOperator: '<S250>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_le > rtb_Switch2_on);

  /* Switch: '<S250>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S244>/delta fall limit' incorporates:
     *  Constant: '<S242>/Constant4'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S250>/UpperRelop' */
    rtb_Compare = (rtb_deltafalllimit_le < elapseTime);

    /* Switch: '<S250>/Switch' */
    if (rtb_Compare) {
      rtb_deltafalllimit_le = elapseTime;
    }

    /* End of Switch: '<S250>/Switch' */
    rtb_Switch2_on = rtb_deltafalllimit_le;
  }

  /* End of Switch: '<S250>/Switch2' */

  /* Sum: '<S244>/Difference Inputs2' incorporates:
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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_m = rtb_Switch2_on +
    rtb_UkYk1;

  /* RelationalOperator: '<S247>/LowerRelop1' incorporates:
   *  Constant: '<S242>/Constant8'
   *  UnitDelay: '<S244>/Delay Input2'
   *
   * Block description for '<S244>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_m >
                       1.5);

  /* Switch: '<S247>/Switch2' incorporates:
   *  Constant: '<S242>/Constant8'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S247>/UpperRelop' incorporates:
     *  Constant: '<S242>/Constant9'
     *  UnitDelay: '<S244>/Delay Input2'
     *
     * Block description for '<S244>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_Compare = (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_m < -1.5);

    /* Switch: '<S247>/Switch' incorporates:
     *  Constant: '<S242>/Constant9'
     *  UnitDelay: '<S244>/Delay Input2'
     *
     * Block description for '<S244>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_Compare) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_m;
    }

    /* End of Switch: '<S247>/Switch' */
  }

  /* End of Switch: '<S247>/Switch2' */

  /* SignalConversion generated from: '<S215>/Out1' */
  rtb_deltafalllimit_le = rtb_UkYk1;

  /* SampleTimeMath: '<S254>/sample time'
   *
   * About '<S254>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S254>/delta rise limit' */
  rtb_StrWhlAngV_c = (real32_T)(1200.0 * elapseTime);

  /* DataTypeConversion: '<S219>/Cast To Boolean1' */
  rtb_Acc_POS = (real32_T)StrWhlAngV;

  /* UnitDelay: '<S254>/Delay Input2'
   *
   * Block description for '<S254>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain3_o = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_j;

  /* Sum: '<S254>/Difference Inputs1'
   *
   * Block description for '<S254>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Acc_POS -= rtb_Gain3_o;

  /* RelationalOperator: '<S257>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Acc_POS > rtb_StrWhlAngV_c);

  /* Switch: '<S257>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S254>/delta fall limit' */
    rtb_StrWhlAngV_c = (real32_T)(-1200.0 * elapseTime);

    /* RelationalOperator: '<S257>/UpperRelop' */
    rtb_Compare = (rtb_Acc_POS < rtb_StrWhlAngV_c);

    /* Switch: '<S257>/Switch' */
    if (rtb_Compare) {
      rtb_Acc_POS = rtb_StrWhlAngV_c;
    }

    /* End of Switch: '<S257>/Switch' */
    rtb_StrWhlAngV_c = rtb_Acc_POS;
  }

  /* End of Switch: '<S257>/Switch2' */

  /* Saturate: '<S219>/Saturation1' incorporates:
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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_j = rtb_StrWhlAngV_c +
    rtb_Gain3_o;
  if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_j > 1200.0F) {
    rtb_Acc_POS = 1200.0F;
  } else if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_j < 0.0F) {
    rtb_Acc_POS = 0.0F;
  } else {
    rtb_Acc_POS = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_j;
  }

  /* End of Saturate: '<S219>/Saturation1' */

  /* Gain: '<S219>/Gain2' */
  rtb_StrWhlAngV_c = 0.7F * rtb_Acc_POS;

  /* UnitDelay: '<S219>/Unit Delay1' */
  rtb_Gain3_o = VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_aq;

  /* Gain: '<S219>/Gain3' */
  rtb_Gain3_o *= 0.3F;

  /* Sum: '<S219>/Add1' */
  rtb_StrWhlAngV_c += rtb_Gain3_o;

  /* UnitDelay: '<S245>/Delay Input2'
   *
   * Block description for '<S245>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_k;

  /* SampleTimeMath: '<S245>/sample time'
   *
   * About '<S245>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S245>/delta rise limit' incorporates:
   *  Constant: '<S242>/Constant3'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S242>/Gain4' */
  rtb_Gain4 = 0.7 * IMU_Yaw_Value;

  /* UnitDelay: '<S242>/Unit Delay2' */
  rtb_Gain5 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE;

  /* Gain: '<S242>/Gain5' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S242>/Add3' */
  rtb_Gain5 += rtb_Gain4;

  /* Sum: '<S245>/Difference Inputs1'
   *
   * Block description for '<S245>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 -= rtb_UkYk1;

  /* RelationalOperator: '<S251>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Gain5 > rtb_Switch2_on);

  /* Switch: '<S251>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S245>/delta fall limit' incorporates:
     *  Constant: '<S242>/Constant5'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S251>/UpperRelop' */
    rtb_Compare = (rtb_Gain5 < elapseTime);

    /* Switch: '<S251>/Switch' */
    if (rtb_Compare) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S251>/Switch' */
    rtb_Switch2_on = rtb_Gain5;
  }

  /* End of Switch: '<S251>/Switch2' */

  /* Saturate: '<S242>/Saturation2' incorporates:
   *  Sum: '<S245>/Difference Inputs2'
   *  UnitDelay: '<S245>/Delay Input2'
   *
   * Block description for '<S245>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S245>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_k = rtb_Switch2_on +
    rtb_UkYk1;
  if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_k > 180.0) {
    rtb_UkYk1 = 180.0;
  } else if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_k < -180.0) {
    rtb_UkYk1 = -180.0;
  } else {
    rtb_UkYk1 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_k;
  }

  /* End of Saturate: '<S242>/Saturation2' */

  /* SignalConversion generated from: '<S215>/Out1' */
  KeyPressed = VehCtrlMdel241025_2018b_amksp_B.ignition_d;

  /* SignalConversion generated from: '<S215>/Out1' */
  rtb_LogicalOperator2 = VehCtrlMdel241025_2018b_amksp_B.out2_h;

  /* SignalConversion generated from: '<S215>/Out1' */
  rtb_Compare = VehCtrlMdel241025_2018b_amksp_B.TSAL_SW_IN_i2;

  /* SignalConversion generated from: '<S215>/Out1' */
  rtb_LowerRelop1_b = VehCtrlMdel241025_2018b_amksp_B.HV_voltValid_kx;

  /* Switch: '<S220>/Switch' incorporates:
   *  Constant: '<S220>/Constant4'
   */
  if (MCFL_DCVoltage != 0.0) {
    /* MinMax: '<S220>/Max' incorporates:
     *  Constant: '<S220>/Constant2'
     */
    elapseTime = fmax(MCFL_DCVoltage, 0.0099999997764825821);

    /* Product: '<S220>/Product' */
    rtb_Switch2_on = MCFL_ActualTorque * MCFL_ActualVelocity;

    /* Product: '<S220>/Divide' */
    rtb_Switch2_on /= 9550.0;

    /* Product: '<S220>/Divide1' */
    AMKFL_Current = rtb_Switch2_on / elapseTime;
  } else {
    AMKFL_Current = 0.0;
  }

  /* End of Switch: '<S220>/Switch' */

  /* Switch: '<S220>/Switch1' incorporates:
   *  Constant: '<S220>/Constant6'
   */
  if (MCFR_DCVoltage != 0.0) {
    /* MinMax: '<S220>/Max1' incorporates:
     *  Constant: '<S220>/Constant3'
     */
    elapseTime = fmax(MCFR_DCVoltage, 0.0099999997764825821);

    /* Product: '<S220>/Product1' */
    rtb_Switch2_on = MCFR_ActualTorque * MCFR_ActualVelocity;

    /* Product: '<S220>/Divide2' */
    rtb_Switch2_on /= 9550.0;

    /* Product: '<S220>/Divide3' */
    AMKFR_Current = rtb_Switch2_on / elapseTime;
  } else {
    AMKFR_Current = 0.0;
  }

  /* End of Switch: '<S220>/Switch1' */

  /* Product: '<S220>/Product2' */
  EmraxPwr = voltage * DC_current;

  /* Update for UnitDelay: '<S217>/Unit Delay1' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_fm = Acc2;

  /* Update for UnitDelay: '<S217>/Unit Delay' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_k = Acc1;

  /* Update for UnitDelay: '<S262>/Delay Input1'
   *
   * Block description for '<S262>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput1_DSTATE_a = rtb_LogicalOperator7_m;

  /* Update for UnitDelay: '<S219>/Unit Delay' incorporates:
   *  UnitDelay: '<S253>/Delay Input2'
   *
   * Block description for '<S253>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_o2 =
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_l2;

  /* Update for UnitDelay: '<S242>/Unit Delay1' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE = rtb_g_mpss1;

  /* Update for UnitDelay: '<S219>/Unit Delay1' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_aq = rtb_Acc_POS;

  /* Update for UnitDelay: '<S242>/Unit Delay2' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE = IMU_Yaw_Value;

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms' */

  /* S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  if (VehCtrlMdel241025_2018b_amks_DW.Subsystem_RESET_ELAPS_T) {
    FunctionCallSubsystem_ELAPS_T = 0U;
  } else {
    FunctionCallSubsystem_ELAPS_T =
      VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3 -
      VehCtrlMdel241025_2018b_amks_DW.Subsystem_PREV_T;
  }

  VehCtrlMdel241025_2018b_amks_DW.Subsystem_PREV_T =
    VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;
  VehCtrlMdel241025_2018b_amks_DW.Subsystem_RESET_ELAPS_T = false;

  /* Gain: '<S216>/Gain5' */
  elapseTime = 10.0 * VehCtrlMdel241025_2018b_amksp_B.CANUnpack_o1;

  /* DataTypeConversion: '<S216>/Cast To Double' */
  rtb_CastToDouble = (real32_T)elapseTime;

  /* MinMax: '<S349>/Min3' incorporates:
   *  Gain: '<S303>/Gain'
   *  UnitDelay: '<S303>/Unit Delay'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_p *= 0.3F;

  /* UnitDelay: '<S307>/Delay Input2'
   *
   * Block description for '<S307>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_b0 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* SampleTimeMath: '<S307>/sample time'
   *
   * About '<S307>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S307>/delta rise limit' */
  rtb_Add4_j = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S216>/Cast To Double1' */
  rtb_Add7 = (real32_T)WhlSpdFL;

  /* Gain: '<S289>/Gain2' */
  rtb_Add6 = 0.0174532924F * rtb_deltafalllimit_iz;

  /* Trigonometry: '<S289>/Asin' */
  rtb_Add6 = cosf(rtb_Add6);

  /* Product: '<S289>/Product1' */
  rtb_Add7 *= rtb_Add6;

  /* DataTypeConversion: '<S216>/Cast To Double5' */
  rtb_Add6 = (real32_T)rtb_UkYk1;

  /* Gain: '<S289>/Gain4' */
  rtb_Add6 *= 0.0174532924F;

  /* Product: '<S289>/Product3' */
  rtb_Switch2_mn = 0.6F * rtb_Add6;

  /* Sum: '<S289>/Add2' */
  rtb_Gain3_o = rtb_Add7 - rtb_Switch2_mn;

  /* UnitDelay: '<S296>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_j;

  /* Sum: '<S296>/Add4' */
  rtb_Add7 = rtb_Gain3_o - rtb_Add7;

  /* Product: '<S296>/Divide' incorporates:
   *  Constant: '<S296>/steptime'
   */
  rtb_Acc_POS = rtb_Add7 / 0.01F;

  /* RelationalOperator: '<S308>/LowerRelop1' incorporates:
   *  Constant: '<S303>/Constant1'
   */
  rtb_LogicalOperator3 = (rtb_Acc_POS > 100.0F);

  /* Switch: '<S308>/Switch2' incorporates:
   *  Constant: '<S303>/Constant1'
   */
  if (rtb_LogicalOperator3) {
    rtb_Acc_POS = 100.0F;
  } else {
    /* RelationalOperator: '<S308>/UpperRelop' incorporates:
     *  Constant: '<S303>/Constant'
     */
    rtb_LogicalOperator7_m = (rtb_Acc_POS < -100.0F);

    /* Switch: '<S308>/Switch' incorporates:
     *  Constant: '<S303>/Constant'
     */
    if (rtb_LogicalOperator7_m) {
      rtb_Acc_POS = -100.0F;
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
  rtb_Acc_POS -= rtb_Switch2_b0;

  /* RelationalOperator: '<S309>/LowerRelop1' */
  rtb_LogicalOperator3 = (rtb_Acc_POS > rtb_Add4_j);

  /* Switch: '<S309>/Switch2' */
  if (!rtb_LogicalOperator3) {
    /* Product: '<S307>/delta fall limit' */
    rtb_deltafalllimit_n = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S309>/UpperRelop' */
    rtb_LogicalOperator7_m = (rtb_Acc_POS < rtb_deltafalllimit_n);

    /* Switch: '<S309>/Switch' */
    if (rtb_LogicalOperator7_m) {
      rtb_Acc_POS = rtb_deltafalllimit_n;
    }

    /* End of Switch: '<S309>/Switch' */
    rtb_Add4_j = rtb_Acc_POS;
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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_n2 = rtb_Add4_j +
    rtb_Switch2_b0;

  /* Gain: '<S303>/Gain1' incorporates:
   *  UnitDelay: '<S307>/Delay Input2'
   *
   * Block description for '<S307>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add7 = 0.7F * VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* MinMax: '<S349>/Min3' incorporates:
   *  Abs: '<S296>/Abs'
   *  Sum: '<S296>/Add'
   *  Sum: '<S303>/Add'
   *  UnitDelay: '<S303>/Unit Delay'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_p += rtb_Add7;
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_p -= rtb_CastToDouble;
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_p = fabsf
    (VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_p);

  /* RelationalOperator: '<S299>/Compare' incorporates:
   *  Constant: '<S299>/Constant'
   *  UnitDelay: '<S303>/Unit Delay'
   */
  rtb_LogicalOperator3 = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_p <=
    0.5F);

  /* UnitDelay: '<S304>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_pj;

  /* Gain: '<S304>/Gain' */
  rtb_Add7 *= 0.3F;

  /* UnitDelay: '<S310>/Delay Input2'
   *
   * Block description for '<S310>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add4_j = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* SampleTimeMath: '<S310>/sample time'
   *
   * About '<S310>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S310>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(100.0 * elapseTime);

  /* MinMax: '<S349>/Min3' incorporates:
   *  DataTypeConversion: '<S216>/Cast To Double2'
   *  UnitDelay: '<S303>/Unit Delay'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_p = (real32_T)WhlSpdFR;

  /* Gain: '<S289>/Gain3' */
  rtb_Add10 = 0.0174532924F * FRWhlStrAng;

  /* Trigonometry: '<S289>/Asin1' */
  rtb_Add10 = cosf(rtb_Add10);

  /* Product: '<S289>/Product2' incorporates:
   *  UnitDelay: '<S303>/Unit Delay'
   */
  rtb_Add10 *= VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_p;

  /* Sum: '<S289>/Add3' */
  rtb_Acc_POS = rtb_Switch2_mn + rtb_Add10;

  /* UnitDelay: '<S296>/Unit Delay1' */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_n;

  /* Sum: '<S296>/Add5' */
  rtb_Add10 = rtb_Acc_POS - rtb_Add10;

  /* Product: '<S296>/Divide1' incorporates:
   *  Constant: '<S296>/steptime1'
   */
  rtb_deltafalllimit_n = rtb_Add10 / 0.01F;

  /* RelationalOperator: '<S311>/LowerRelop1' incorporates:
   *  Constant: '<S304>/Constant1'
   */
  rtb_Compare_i = (rtb_deltafalllimit_n > 100.0F);

  /* Switch: '<S311>/Switch2' incorporates:
   *  Constant: '<S304>/Constant1'
   */
  if (rtb_Compare_i) {
    rtb_deltafalllimit_n = 100.0F;
  } else {
    /* RelationalOperator: '<S311>/UpperRelop' incorporates:
     *  Constant: '<S304>/Constant'
     */
    rtb_LogicalOperator7_m = (rtb_deltafalllimit_n < -100.0F);

    /* Switch: '<S311>/Switch' incorporates:
     *  Constant: '<S304>/Constant'
     */
    if (rtb_LogicalOperator7_m) {
      rtb_deltafalllimit_n = -100.0F;
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
  rtb_deltafalllimit_n -= rtb_Add4_j;

  /* RelationalOperator: '<S312>/LowerRelop1' */
  rtb_Compare_i = (rtb_deltafalllimit_n > rtb_Switch2_b0);

  /* Switch: '<S312>/Switch2' */
  if (!rtb_Compare_i) {
    /* Product: '<S310>/delta fall limit' */
    rtb_deltafalllimit_om = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S312>/UpperRelop' */
    rtb_LogicalOperator7_m = (rtb_deltafalllimit_n < rtb_deltafalllimit_om);

    /* Switch: '<S312>/Switch' */
    if (rtb_LogicalOperator7_m) {
      rtb_deltafalllimit_n = rtb_deltafalllimit_om;
    }

    /* End of Switch: '<S312>/Switch' */
    rtb_Switch2_b0 = rtb_deltafalllimit_n;
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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_e = rtb_Switch2_b0 +
    rtb_Add4_j;

  /* Gain: '<S304>/Gain1' incorporates:
   *  UnitDelay: '<S310>/Delay Input2'
   *
   * Block description for '<S310>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = 0.7F * VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* Sum: '<S304>/Add' */
  rtb_Add7 += rtb_Add10;

  /* Sum: '<S296>/Add1' */
  rtb_Add7 -= rtb_CastToDouble;

  /* Abs: '<S296>/Abs1' */
  rtb_Add7 = fabsf(rtb_Add7);

  /* RelationalOperator: '<S300>/Compare' incorporates:
   *  Constant: '<S300>/Constant'
   */
  rtb_Compare_i = (rtb_Add7 <= 0.5F);

  /* UnitDelay: '<S305>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_a;

  /* Gain: '<S305>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S313>/Delay Input2'
   *
   * Block description for '<S313>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* SampleTimeMath: '<S313>/sample time'
   *
   * About '<S313>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S313>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S216>/Cast To Double3' */
  rtb_Add4_j = (real32_T)WhlSpdRL_mps;

  /* Product: '<S289>/Product' */
  rtb_Add6 *= 0.58F;

  /* Sum: '<S289>/Add' */
  rtb_deltafalllimit_n = rtb_Add4_j - rtb_Add6;

  /* UnitDelay: '<S296>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_l;

  /* Sum: '<S296>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_n - rtb_Add4_j;

  /* Product: '<S296>/Divide2' incorporates:
   *  Constant: '<S296>/steptime2'
   */
  rtb_deltafalllimit_om = rtb_Add4_j / 0.01F;

  /* RelationalOperator: '<S314>/LowerRelop1' incorporates:
   *  Constant: '<S305>/Constant1'
   */
  rtb_Compare_c = (rtb_deltafalllimit_om > 100.0F);

  /* Switch: '<S314>/Switch2' incorporates:
   *  Constant: '<S305>/Constant1'
   */
  if (rtb_Compare_c) {
    rtb_deltafalllimit_om = 100.0F;
  } else {
    /* RelationalOperator: '<S314>/UpperRelop' incorporates:
     *  Constant: '<S305>/Constant'
     */
    rtb_LogicalOperator7_m = (rtb_deltafalllimit_om < -100.0F);

    /* Switch: '<S314>/Switch' incorporates:
     *  Constant: '<S305>/Constant'
     */
    if (rtb_LogicalOperator7_m) {
      rtb_deltafalllimit_om = -100.0F;
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
  rtb_deltafalllimit_om -= rtb_Switch2_mn;

  /* RelationalOperator: '<S315>/LowerRelop1' */
  rtb_Compare_c = (rtb_deltafalllimit_om > rtb_Add7);

  /* Switch: '<S315>/Switch2' */
  if (!rtb_Compare_c) {
    /* Product: '<S313>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S315>/UpperRelop' */
    rtb_LogicalOperator7_m = (rtb_deltafalllimit_om < rtb_Add7);

    /* Switch: '<S315>/Switch' */
    if (rtb_LogicalOperator7_m) {
      rtb_deltafalllimit_om = rtb_Add7;
    }

    /* End of Switch: '<S315>/Switch' */
    rtb_Add7 = rtb_deltafalllimit_om;
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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hk = rtb_Add7 +
    rtb_Switch2_mn;

  /* Gain: '<S305>/Gain1' incorporates:
   *  UnitDelay: '<S313>/Delay Input2'
   *
   * Block description for '<S313>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* Sum: '<S305>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S296>/Add2' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S296>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S301>/Compare' incorporates:
   *  Constant: '<S301>/Constant'
   */
  rtb_Compare_c = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S306>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_nc;

  /* Gain: '<S306>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S316>/Delay Input2'
   *
   * Block description for '<S316>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* SampleTimeMath: '<S316>/sample time'
   *
   * About '<S316>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S316>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S216>/Cast To Double4' */
  rtb_Add4_j = (real32_T)WhlSpdRR_mps;

  /* Sum: '<S289>/Add1' */
  rtb_deltafalllimit_om = rtb_Add6 + rtb_Add4_j;

  /* UnitDelay: '<S296>/Unit Delay3' */
  rtb_Add6 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE;

  /* Sum: '<S296>/Add7' */
  rtb_Add6 = rtb_deltafalllimit_om - rtb_Add6;

  /* Product: '<S296>/Divide3' incorporates:
   *  Constant: '<S296>/steptime3'
   */
  rtb_Add6 /= 0.01F;

  /* RelationalOperator: '<S317>/LowerRelop1' incorporates:
   *  Constant: '<S306>/Constant1'
   */
  rtb_LogicalOperator7_m = (rtb_Add6 > 100.0F);

  /* Switch: '<S317>/Switch2' incorporates:
   *  Constant: '<S306>/Constant1'
   */
  if (rtb_LogicalOperator7_m) {
    rtb_Add6 = 100.0F;
  } else {
    /* RelationalOperator: '<S317>/UpperRelop' incorporates:
     *  Constant: '<S306>/Constant'
     */
    rtb_LogicalOperator7_m = (rtb_Add6 < -100.0F);

    /* Switch: '<S317>/Switch' incorporates:
     *  Constant: '<S306>/Constant'
     */
    if (rtb_LogicalOperator7_m) {
      rtb_Add6 = -100.0F;
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
  rtb_Add6 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S318>/LowerRelop1' */
  rtb_LogicalOperator7_m = (rtb_Add6 > rtb_Add7);

  /* Switch: '<S318>/Switch2' */
  if (!rtb_LogicalOperator7_m) {
    /* Product: '<S316>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S318>/UpperRelop' */
    rtb_LogicalOperator7_m = (rtb_Add6 < rtb_Add7);

    /* Switch: '<S318>/Switch' */
    if (rtb_LogicalOperator7_m) {
      rtb_Add6 = rtb_Add7;
    }

    /* End of Switch: '<S318>/Switch' */
    rtb_Add7 = rtb_Add6;
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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_c = rtb_Add7 +
    rtb_Switch2_mn;

  /* Gain: '<S306>/Gain1' incorporates:
   *  UnitDelay: '<S316>/Delay Input2'
   *
   * Block description for '<S316>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* Sum: '<S306>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S296>/Add3' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S296>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S302>/Compare' incorporates:
   *  Constant: '<S302>/Constant'
   */
  rtb_LogicalOperator7_m = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S323>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_lh;

  /* Gain: '<S323>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S327>/Delay Input2'
   *
   * Block description for '<S327>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* SampleTimeMath: '<S327>/sample time'
   *
   * About '<S327>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S327>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S297>/Unit Delay4' */
  rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE;

  /* UnitDelay: '<S297>/Unit Delay' */
  rtb_Add4_j = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_a0;

  /* Sum: '<S297>/Add4' */
  rtb_Add4_j = rtb_Gain3_o - rtb_Add4_j;

  /* Product: '<S297>/Divide' incorporates:
   *  Constant: '<S297>/steptime'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S297>/Add' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S297>/Add8' */
  rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE - rtb_Add7;

  /* Product: '<S297>/Divide4' incorporates:
   *  Constant: '<S297>/steptime4'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S328>/LowerRelop1' incorporates:
   *  Constant: '<S323>/Constant1'
   */
  rtb_AND_l = (rtb_Add7 > 100.0F);

  /* Switch: '<S328>/Switch2' incorporates:
   *  Constant: '<S323>/Constant1'
   */
  if (rtb_AND_l) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S328>/UpperRelop' incorporates:
     *  Constant: '<S323>/Constant'
     */
    rtb_AND_l = (rtb_Add7 < -100.0F);

    /* Switch: '<S328>/Switch' incorporates:
     *  Constant: '<S323>/Constant'
     */
    if (rtb_AND_l) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S328>/Switch' */
  }

  /* End of Switch: '<S328>/Switch2' */

  /* Sum: '<S327>/Difference Inputs1'
   *
   * Block description for '<S327>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S329>/LowerRelop1' */
  rtb_AND_l = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S329>/Switch2' */
  if (!rtb_AND_l) {
    /* Product: '<S327>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S329>/UpperRelop' */
    rtb_AND_l = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S329>/Switch' */
    if (rtb_AND_l) {
      rtb_Add7 = rtb_Add6;
    }

    /* End of Switch: '<S329>/Switch' */
    rtb_Add6 = rtb_Add7;
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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_i = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S323>/Gain1' incorporates:
   *  UnitDelay: '<S327>/Delay Input2'
   *
   * Block description for '<S327>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* Sum: '<S323>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S297>/Abs' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S319>/Compare' incorporates:
   *  Constant: '<S319>/Constant'
   */
  rtb_AND_l = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S324>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_ap;

  /* Gain: '<S324>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S330>/Delay Input2'
   *
   * Block description for '<S330>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* SampleTimeMath: '<S330>/sample time'
   *
   * About '<S330>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S330>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S297>/Unit Delay5' */
  rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay5_DSTATE;

  /* UnitDelay: '<S297>/Unit Delay1' */
  rtb_Add4_j = VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_a;

  /* Sum: '<S297>/Add5' */
  rtb_Add4_j = rtb_Acc_POS - rtb_Add4_j;

  /* Product: '<S297>/Divide1' incorporates:
   *  Constant: '<S297>/steptime1'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S297>/Add1' incorporates:
   *  UnitDelay: '<S297>/Unit Delay5'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay5_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S297>/Add10' incorporates:
   *  UnitDelay: '<S297>/Unit Delay5'
   */
  rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay5_DSTATE - rtb_Add7;

  /* Product: '<S297>/Divide5' incorporates:
   *  Constant: '<S297>/steptime5'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S331>/LowerRelop1' incorporates:
   *  Constant: '<S324>/Constant1'
   */
  rtb_AND2_e = (rtb_Add7 > 100.0F);

  /* Switch: '<S331>/Switch2' incorporates:
   *  Constant: '<S324>/Constant1'
   */
  if (rtb_AND2_e) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S331>/UpperRelop' incorporates:
     *  Constant: '<S324>/Constant'
     */
    rtb_AND2_e = (rtb_Add7 < -100.0F);

    /* Switch: '<S331>/Switch' incorporates:
     *  Constant: '<S324>/Constant'
     */
    if (rtb_AND2_e) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S331>/Switch' */
  }

  /* End of Switch: '<S331>/Switch2' */

  /* Sum: '<S330>/Difference Inputs1'
   *
   * Block description for '<S330>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S332>/LowerRelop1' */
  rtb_AND2_e = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S332>/Switch2' */
  if (!rtb_AND2_e) {
    /* Product: '<S330>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S332>/UpperRelop' */
    rtb_AND2_e = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S332>/Switch' */
    if (rtb_AND2_e) {
      rtb_Add7 = rtb_Add6;
    }

    /* End of Switch: '<S332>/Switch' */
    rtb_Add6 = rtb_Add7;
  }

  /* End of Switch: '<S332>/Switch2' */

  /* Sum: '<S330>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S330>/Delay Input2'
   *
   * Block description for '<S330>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S330>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_el = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S324>/Gain1' incorporates:
   *  UnitDelay: '<S330>/Delay Input2'
   *
   * Block description for '<S330>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* Sum: '<S324>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S297>/Abs1' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S320>/Compare' incorporates:
   *  Constant: '<S320>/Constant'
   */
  rtb_AND2_e = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S325>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_o;

  /* Gain: '<S325>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S333>/Delay Input2'
   *
   * Block description for '<S333>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_pdc;

  /* SampleTimeMath: '<S333>/sample time'
   *
   * About '<S333>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S333>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S297>/Unit Delay6' */
  rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay6_DSTATE;

  /* UnitDelay: '<S297>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_c;

  /* Sum: '<S297>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_n - rtb_Add4_j;

  /* Product: '<S297>/Divide2' incorporates:
   *  Constant: '<S297>/steptime2'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S297>/Add2' incorporates:
   *  UnitDelay: '<S297>/Unit Delay6'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay6_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S297>/Add12' incorporates:
   *  UnitDelay: '<S297>/Unit Delay6'
   */
  rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay6_DSTATE - rtb_Add7;

  /* Product: '<S297>/Divide6' incorporates:
   *  Constant: '<S297>/steptime6'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S334>/LowerRelop1' incorporates:
   *  Constant: '<S325>/Constant1'
   */
  rtb_Compare_b = (rtb_Add7 > 100.0F);

  /* Switch: '<S334>/Switch2' incorporates:
   *  Constant: '<S325>/Constant1'
   */
  if (rtb_Compare_b) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S334>/UpperRelop' incorporates:
     *  Constant: '<S325>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S334>/Switch' incorporates:
     *  Constant: '<S325>/Constant'
     */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S334>/Switch' */
  }

  /* End of Switch: '<S334>/Switch2' */

  /* Sum: '<S333>/Difference Inputs1'
   *
   * Block description for '<S333>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S335>/LowerRelop1' */
  rtb_Compare_b = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S335>/Switch2' */
  if (!rtb_Compare_b) {
    /* Product: '<S333>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S335>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S335>/Switch' */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = rtb_Add6;
    }

    /* End of Switch: '<S335>/Switch' */
    rtb_Add6 = rtb_Add7;
  }

  /* End of Switch: '<S335>/Switch2' */

  /* Sum: '<S333>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S333>/Delay Input2'
   *
   * Block description for '<S333>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S333>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_pdc = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S325>/Gain1' incorporates:
   *  UnitDelay: '<S333>/Delay Input2'
   *
   * Block description for '<S333>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_pdc;

  /* Sum: '<S325>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S297>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S321>/Compare' incorporates:
   *  Constant: '<S321>/Constant'
   */
  rtb_Compare_b = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S326>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_ah;

  /* Gain: '<S326>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S336>/Delay Input2'
   *
   * Block description for '<S336>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* SampleTimeMath: '<S336>/sample time'
   *
   * About '<S336>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S336>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S297>/Unit Delay7' */
  rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay7_DSTATE;

  /* UnitDelay: '<S297>/Unit Delay3' */
  rtb_Add4_j = VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_d;

  /* Sum: '<S297>/Add7' */
  rtb_Add4_j = rtb_deltafalllimit_om - rtb_Add4_j;

  /* Product: '<S297>/Divide3' incorporates:
   *  Constant: '<S297>/steptime3'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S297>/Add3' incorporates:
   *  UnitDelay: '<S297>/Unit Delay7'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay7_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S297>/Add14' incorporates:
   *  UnitDelay: '<S297>/Unit Delay7'
   */
  rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay7_DSTATE - rtb_Add7;

  /* Product: '<S297>/Divide7' incorporates:
   *  Constant: '<S297>/steptime7'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S337>/LowerRelop1' incorporates:
   *  Constant: '<S326>/Constant1'
   */
  rtb_UpperRelop_ir = (rtb_Add7 > 100.0F);

  /* Switch: '<S337>/Switch2' incorporates:
   *  Constant: '<S326>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S337>/UpperRelop' incorporates:
     *  Constant: '<S326>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S337>/Switch' incorporates:
     *  Constant: '<S326>/Constant'
     */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S337>/Switch' */
  }

  /* End of Switch: '<S337>/Switch2' */

  /* Sum: '<S336>/Difference Inputs1'
   *
   * Block description for '<S336>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S338>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S338>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S336>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S338>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S338>/Switch' */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = rtb_Add6;
    }

    /* End of Switch: '<S338>/Switch' */
    rtb_Add6 = rtb_Add7;
  }

  /* End of Switch: '<S338>/Switch2' */

  /* Sum: '<S336>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S336>/Delay Input2'
   *
   * Block description for '<S336>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S336>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_mt = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S326>/Gain1' incorporates:
   *  UnitDelay: '<S336>/Delay Input2'
   *
   * Block description for '<S336>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* Sum: '<S326>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S297>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S322>/Compare' incorporates:
   *  Constant: '<S322>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.8F);

  /* Logic: '<S287>/Logical Operator' */
  rtb_LogicalOperator_idx_0 = (rtb_LogicalOperator3 || rtb_AND_l);
  rtb_Compare_i = (rtb_Compare_i || rtb_AND2_e);
  rtb_Compare_c = (rtb_Compare_c || rtb_Compare_b);
  rtb_LogicalOperator3 = (rtb_LogicalOperator7_m || rtb_UpperRelop_ir);

  /* UnitDelay: '<S216>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_lha;

  /* Sum: '<S298>/Add' */
  rtb_Switch2_mn = rtb_Gain3_o - rtb_Add10;

  /* Abs: '<S298>/Abs' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S339>/Compare' incorporates:
   *  Constant: '<S339>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S298>/AND3' */
  rtb_Compare_b = (rtb_UpperRelop_ir && (VehCtrlMdel241025_2018b_amksp_B.Exit_hj
    != 0.0));

  /* Sum: '<S298>/Add1' */
  rtb_Switch2_mn = rtb_Acc_POS - rtb_Add10;

  /* Abs: '<S298>/Abs1' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S340>/Compare' incorporates:
   *  Constant: '<S340>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S298>/AND2' */
  rtb_AND2_e = (rtb_UpperRelop_ir && (VehCtrlMdel241025_2018b_amksp_B.Exit_o !=
    0.0));

  /* Sum: '<S298>/Add2' */
  rtb_Switch2_mn = rtb_deltafalllimit_n - rtb_Add10;

  /* Abs: '<S298>/Abs2' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S341>/Compare' incorporates:
   *  Constant: '<S341>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S298>/AND' */
  rtb_AND_l = (rtb_UpperRelop_ir && (VehCtrlMdel241025_2018b_amksp_B.Exit_le !=
    0.0));

  /* Sum: '<S298>/Add3' */
  rtb_Add10 = rtb_deltafalllimit_om - rtb_Add10;

  /* Abs: '<S298>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S342>/Compare' incorporates:
   *  Constant: '<S342>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 2.0F);

  /* Logic: '<S298>/AND1' */
  rtb_LogicalOperator7_m = (rtb_UpperRelop_ir &&
    (VehCtrlMdel241025_2018b_amksp_B.Exit_is != 0.0));

  /* Logic: '<S287>/Logical Operator1' */
  rtb_UpperRelop_ir = (rtb_Compare_b && rtb_LogicalOperator_idx_0);
  rtb_Compare_i = (rtb_AND2_e && rtb_Compare_i);
  rtb_Compare_c = (rtb_AND_l && rtb_Compare_c);
  rtb_LogicalOperator7_m = (rtb_LogicalOperator7_m && rtb_LogicalOperator3);

  /* Chart: '<S287>/Timer' incorporates:
   *  Constant: '<S287>/Constant1'
   */
  VehCtrlMdel241025_20_Timer1(rtb_UpperRelop_ir, 0.5F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_c,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer_o);

  /* Chart: '<S287>/Timer1' incorporates:
   *  Constant: '<S287>/Constant2'
   */
  VehCtrlMdel241025_20_Timer1(rtb_Compare_i, 0.5F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_lh4,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer1_m);

  /* Chart: '<S287>/Timer2' incorporates:
   *  Constant: '<S287>/Constant3'
   */
  VehCtrlMdel241025_20_Timer1(rtb_Compare_c, 0.5F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_lh,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer2_g);

  /* Chart: '<S287>/Timer3' incorporates:
   *  Constant: '<S287>/Constant4'
   */
  VehCtrlMdel241025_20_Timer1(rtb_LogicalOperator7_m, 0.5F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_a,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer3_i);

  /* Logic: '<S285>/Logical Operator' */
  rtb_UpperRelop_ir = ((VehCtrlMdel241025_2018b_amksp_B.Exit_c != 0.0) ||
                       (VehCtrlMdel241025_2018b_amksp_B.Exit_lh4 != 0.0) ||
                       (VehCtrlMdel241025_2018b_amksp_B.Exit_lh != 0.0) ||
                       (VehCtrlMdel241025_2018b_amksp_B.Exit_a != 0.0));

  /* Logic: '<S285>/Logical Operator1' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* UnitDelay: '<S285>/Unit Delay4' */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_m;

  /* Sum: '<S285>/Add1' */
  rtb_Add10 = Acc_POS_n - rtb_Add10;

  /* RelationalOperator: '<S290>/Compare' incorporates:
   *  Constant: '<S290>/Constant'
   */
  rtb_Compare_b = (rtb_Add10 > 0.1F);

  /* Logic: '<S285>/Logical Operator2' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir || rtb_Compare_b);

  /* Logic: '<S285>/AND' */
  rtb_LogicalOperator7_m = ((VehCtrlMdel241025_2018b_amksp_B.CANUnpack_o1 != 0.0)
    && rtb_UpperRelop_ir);

  /* UnitDelay: '<S285>/Unit Delay3' */
  rtb_UpperRelop_ir = VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_f;

  /* Logic: '<S285>/Logical Operator3' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S285>/Switch3' incorporates:
   *  UnitDelay: '<S285>/Unit Delay1'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S285>/Switch4' incorporates:
     *  Constant: '<S285>/InitZORE'
     */
    if (!rtb_LogicalOperator7_m) {
      VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_k = 0.0F;
    }

    /* End of Switch: '<S285>/Switch4' */
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_o =
      VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Switch: '<S285>/Switch3' */

  /* UnitDelay: '<S288>/Unit Delay3' */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_p;

  /* Sum: '<S288>/Add5' incorporates:
   *  UnitDelay: '<S288>/Unit Delay1'
   */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_d - rtb_Add10;

  /* Product: '<S288>/Divide3' incorporates:
   *  Constant: '<S288>/steptime3'
   */
  rtb_Add10 /= 0.01F;

  /* UnitDelay: '<S288>/Unit Delay2' */
  rtb_Switch2_mn = VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_f;

  /* Sum: '<S288>/Add9' */
  rtb_Switch2_mn -= rtb_Add10;

  /* UnitDelay: '<S288>/Unit Delay4' */
  rtb_Add6 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_mn;

  /* Sum: '<S288>/Add6' incorporates:
   *  Constant: '<S288>/steptime4'
   */
  rtb_Add6 += 0.1F;

  /* Sum: '<S288>/Add8' incorporates:
   *  Constant: '<S288>/steptime6'
   */
  rtb_Add7 = rtb_Add6 + 2.0F;

  /* Product: '<S288>/Divide5' */
  rtb_Add7 = 1.0F / rtb_Add7 * rtb_Add6;

  /* Logic: '<S288>/Logical Operator' */
  rtb_LogicalOperator3 = ((VehCtrlMdel241025_2018b_amksp_B.Exit_c != 0.0) ||
    (VehCtrlMdel241025_2018b_amksp_B.Exit_lh4 != 0.0) ||
    (VehCtrlMdel241025_2018b_amksp_B.Exit_lh != 0.0) ||
    (VehCtrlMdel241025_2018b_amksp_B.Exit_a != 0.0));

  /* Switch: '<S288>/Switch13' incorporates:
   *  Constant: '<S288>/Constant10'
   */
  if (rtb_LogicalOperator3) {
    rtb_Add4_j = rtb_Add7;
  } else {
    rtb_Add4_j = 1.0F;
  }

  /* End of Switch: '<S288>/Switch13' */

  /* Product: '<S288>/Divide6' */
  rtb_Switch2_mn *= rtb_Add4_j;

  /* Sum: '<S288>/Add10' */
  rtb_Ax = rtb_Switch2_mn + rtb_Add10;

  /* Switch: '<S285>/Switch1' */
  if (rtb_LogicalOperator7_m) {
    /* Saturate: '<S285>/Saturation1' */
    if (VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_d > 200.0F) {
      VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_d = 200.0F;
    } else {
      if (VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_d < -10.0F) {
        VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_d = -10.0F;
      }
    }

    /* Product: '<S285>/Product' incorporates:
     *  Constant: '<S285>/steptime1'
     */
    rtb_Switch2_mn = rtb_Ax * 0.01F;

    /* Saturate: '<S285>/Saturation1' incorporates:
     *  Sum: '<S285>/Add'
     */
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_d += rtb_Switch2_mn;
  } else {
    /* Saturate: '<S285>/Saturation1' incorporates:
     *  Constant: '<S285>/Constant'
     *  UnitDelay: '<S285>/Unit Delay'
     */
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_d = 0.0F;
  }

  /* End of Switch: '<S285>/Switch1' */

  /* Saturate: '<S285>/Saturation' */
  if (VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_d > 200.0F) {
    rtb_Add10 = 200.0F;
  } else if (VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_d < -10.0F) {
    rtb_Add10 = -10.0F;
  } else {
    rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_d;
  }

  /* End of Saturate: '<S285>/Saturation' */

  /* Sum: '<S285>/Add3' incorporates:
   *  UnitDelay: '<S285>/Unit Delay1'
   */
  rtb_VxIMU_est = rtb_Add10 +
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_o;

  /* MinMax: '<S287>/Min1' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Gain3_o, rtb_Acc_POS);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_n);
  rtb_Add10 = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_om);

  /* Sum: '<S285>/Add2' */
  rtb_Add10 -= rtb_VxIMU_est;

  /* RelationalOperator: '<S291>/Compare' incorporates:
   *  Constant: '<S291>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.0F);

  /* Switch: '<S285>/Switch6' incorporates:
   *  Constant: '<S285>/Reset'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S285>/Add10' incorporates:
     *  Constant: '<S285>/Steptime'
     */
    rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_i + 0.01F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S285>/Switch6' */

  /* MinMax: '<S285>/Min' incorporates:
   *  Constant: '<S285>/ResetDelay'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_i = fminf(rtb_Add10, 0.1F);

  /* RelationalOperator: '<S285>/Relational Operator9' incorporates:
   *  Constant: '<S285>/ResetDelay'
   *  UnitDelay: '<S285>/Unit Delay2'
   */
  rtb_Compare_i = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_i >= 0.1F);

  /* RelationalOperator: '<S344>/Compare' incorporates:
   *  Constant: '<S344>/Constant'
   */
  rtb_Compare_c = (rtb_Ax < -0.5F);

  /* Chart: '<S288>/Timer2' incorporates:
   *  Constant: '<S288>/Constant15'
   */
  VehCtrlMdel241025_20_Timer1(rtb_Compare_c, 0.11F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_i,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer2_j);

  /* UnitDelay: '<S348>/Delay Input2'
   *
   * Block description for '<S348>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_g;

  /* SampleTimeMath: '<S348>/sample time'
   *
   * About '<S348>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S348>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(10.0 * elapseTime);

  /* Sum: '<S349>/Add3' */
  rtb_Add4_j = ((rtb_deltafalllimit_om + rtb_deltafalllimit_n) + rtb_Acc_POS) +
    rtb_Gain3_o;

  /* MinMax: '<S349>/Min4' */
  rtb_Switch2_b0 = fminf(rtb_Gain3_o, rtb_Acc_POS);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_n);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_om);

  /* MinMax: '<S349>/Min3' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Gain3_o, rtb_Acc_POS);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_n);
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_p = fmaxf(rtb_MaxWhlSpd_mps_n,
    rtb_deltafalllimit_om);

  /* Sum: '<S349>/Add4' incorporates:
   *  UnitDelay: '<S303>/Unit Delay'
   */
  rtb_Add4_j = (rtb_Add4_j - rtb_Switch2_b0) -
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_p;

  /* Gain: '<S349>/Gain1' */
  rtb_Add4_j *= 0.5F;

  /* Sum: '<S348>/Difference Inputs1'
   *
   * Block description for '<S348>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add4_j -= rtb_Add10;

  /* RelationalOperator: '<S357>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add4_j > rtb_Switch2_mn);

  /* Switch: '<S357>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S348>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-10.0 * elapseTime);

    /* RelationalOperator: '<S357>/UpperRelop' */
    rtb_Compare_c = (rtb_Add4_j < rtb_Switch2_mn);

    /* Switch: '<S357>/Switch' */
    if (rtb_Compare_c) {
      rtb_Add4_j = rtb_Switch2_mn;
    }

    /* End of Switch: '<S357>/Switch' */
    rtb_Switch2_mn = rtb_Add4_j;
  }

  /* End of Switch: '<S357>/Switch2' */

  /* Sum: '<S348>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S348>/Delay Input2'
   *
   * Block description for '<S348>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S348>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_g = rtb_Switch2_mn +
    rtb_Add10;

  /* RelationalOperator: '<S343>/Compare' incorporates:
   *  Constant: '<S343>/Constant'
   */
  rtb_Compare_c = (rtb_Ax > 0.5F);

  /* Chart: '<S288>/Timer1' incorporates:
   *  Constant: '<S288>/Constant14'
   */
  VehCtrlMdel241025_20_Timer1(rtb_Compare_c, 0.11F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_l,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer1_p);

  /* Logic: '<S288>/Logical Operator2' */
  rtb_UpperRelop_ir = !(VehCtrlMdel241025_2018b_amksp_B.Exit_l != 0.0);

  /* Switch: '<S288>/Switch6' incorporates:
   *  Switch: '<S288>/Switch4'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S288>/Switch5' incorporates:
     *  UnitDelay: '<S348>/Delay Input2'
     *
     * Block description for '<S348>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (VehCtrlMdel241025_2018b_amksp_B.Exit_i != 0.0) {
      /* Switch: '<S288>/Switch11' incorporates:
       *  Constant: '<S288>/Constant7'
       */
      if (VehCtrlMdel241025_2018b_amksp_B.Exit_a != 0.0) {
        rtb_Switch2_mn = rtb_deltafalllimit_om;
      } else {
        rtb_Switch2_mn = 0.0F;
      }

      /* End of Switch: '<S288>/Switch11' */

      /* Switch: '<S288>/Switch10' incorporates:
       *  Constant: '<S288>/Constant6'
       */
      if (VehCtrlMdel241025_2018b_amksp_B.Exit_lh != 0.0) {
        rtb_Add10 = rtb_deltafalllimit_n;
      } else {
        rtb_Add10 = 0.0F;
      }

      /* End of Switch: '<S288>/Switch10' */

      /* Switch: '<S288>/Switch9' incorporates:
       *  Constant: '<S288>/Constant5'
       */
      if (VehCtrlMdel241025_2018b_amksp_B.Exit_lh4 != 0.0) {
        rtb_Add4_j = rtb_Acc_POS;
      } else {
        rtb_Add4_j = 0.0F;
      }

      /* End of Switch: '<S288>/Switch9' */

      /* Switch: '<S288>/Switch8' incorporates:
       *  Constant: '<S288>/Constant4'
       */
      if (VehCtrlMdel241025_2018b_amksp_B.Exit_c != 0.0) {
        rtb_Switch2_b0 = rtb_Gain3_o;
      } else {
        rtb_Switch2_b0 = 0.0F;
      }

      /* End of Switch: '<S288>/Switch8' */

      /* MinMax: '<S288>/Min1' */
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Switch2_b0, rtb_Add4_j);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Add10);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Switch2_mn);
    } else {
      rtb_MaxWhlSpd_mps_n = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_g;
    }

    /* End of Switch: '<S288>/Switch5' */
  } else {
    if (VehCtrlMdel241025_2018b_amksp_B.Exit_a != 0.0) {
      /* Switch: '<S288>/Switch4' */
      rtb_Switch2_mn = rtb_deltafalllimit_om;
    } else {
      /* Switch: '<S288>/Switch4' incorporates:
       *  Constant: '<S288>/Constant3'
       */
      rtb_Switch2_mn = 9999.0F;
    }

    /* Switch: '<S288>/Switch3' incorporates:
     *  Constant: '<S288>/Constant2'
     */
    if (VehCtrlMdel241025_2018b_amksp_B.Exit_lh != 0.0) {
      rtb_Add10 = rtb_deltafalllimit_n;
    } else {
      rtb_Add10 = 9999.0F;
    }

    /* End of Switch: '<S288>/Switch3' */

    /* Switch: '<S288>/Switch2' incorporates:
     *  Constant: '<S288>/Constant1'
     */
    if (VehCtrlMdel241025_2018b_amksp_B.Exit_lh4 != 0.0) {
      rtb_Add4_j = rtb_Acc_POS;
    } else {
      rtb_Add4_j = 9999.0F;
    }

    /* End of Switch: '<S288>/Switch2' */

    /* Switch: '<S288>/Switch1' incorporates:
     *  Constant: '<S288>/Constant'
     */
    if (VehCtrlMdel241025_2018b_amksp_B.Exit_c != 0.0) {
      rtb_Switch2_b0 = rtb_Gain3_o;
    } else {
      rtb_Switch2_b0 = 9999.0F;
    }

    /* End of Switch: '<S288>/Switch1' */

    /* MinMax: '<S288>/Min2' */
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add4_j);
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add10);
    rtb_MaxWhlSpd_mps_n = fminf(rtb_Switch2_b0, rtb_Switch2_mn);
  }

  /* End of Switch: '<S288>/Switch6' */

  /* Logic: '<S288>/NOT3' */
  rtb_UpperRelop_ir = !rtb_LogicalOperator3;

  /* Logic: '<S288>/Logical Operator3' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir && rtb_Compare_i);

  /* Logic: '<S288>/NOT4' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S288>/Switch7' incorporates:
   *  UnitDelay: '<S348>/Delay Input2'
   *
   * Block description for '<S348>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (rtb_UpperRelop_ir) {
    /* Logic: '<S288>/Logical Operator1' */
    rtb_Compare_i = (rtb_Compare_i || rtb_LogicalOperator3);

    /* Switch: '<S288>/Switch' */
    if (rtb_Compare_i) {
      rtb_VxIMU_est = rtb_MaxWhlSpd_mps_n;
    }

    /* End of Switch: '<S288>/Switch' */
  } else {
    rtb_VxIMU_est = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_g;
  }

  /* End of Switch: '<S288>/Switch7' */

  /* UnitDelay: '<S346>/Delay Input2'
   *
   * Block description for '<S346>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_a;

  /* Sum: '<S346>/Difference Inputs1'
   *
   * Block description for '<S346>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_VxIMU_est -= rtb_Add10;

  /* Switch: '<S288>/Switch12' incorporates:
   *  Constant: '<S288>/Constant8'
   *  Constant: '<S288>/Constant9'
   */
  if (rtb_LogicalOperator3) {
    rtb_Switch2_mn = 0.1F;
  } else {
    rtb_Switch2_mn = 0.05F;
  }

  /* End of Switch: '<S288>/Switch12' */

  /* Sum: '<S288>/Add4' */
  rtb_Add4_j = rtb_Ax + rtb_Switch2_mn;

  /* SampleTimeMath: '<S346>/sample time'
   *
   * About '<S346>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S346>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S355>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_VxIMU_est > rtb_Switch2_b0);

  /* Sum: '<S288>/Add3' */
  rtb_Ax -= rtb_Switch2_mn;

  /* Switch: '<S355>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S346>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S355>/UpperRelop' */
    rtb_LogicalOperator3 = (rtb_VxIMU_est < rtb_Switch2_mn);

    /* Switch: '<S355>/Switch' */
    if (rtb_LogicalOperator3) {
      rtb_VxIMU_est = rtb_Switch2_mn;
    }

    /* End of Switch: '<S355>/Switch' */
    rtb_Switch2_b0 = rtb_VxIMU_est;
  }

  /* End of Switch: '<S355>/Switch2' */

  /* Sum: '<S346>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S346>/Delay Input2'
   *
   * Block description for '<S346>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S346>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_a = rtb_Switch2_b0 +
    rtb_Add10;

  /* RelationalOperator: '<S353>/LowerRelop1' incorporates:
   *  Constant: '<S345>/Constant1'
   *  UnitDelay: '<S346>/Delay Input2'
   *
   * Block description for '<S346>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_a >
                       100.0F);

  /* Switch: '<S353>/Switch2' incorporates:
   *  Constant: '<S345>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_mn = 100.0F;
  } else {
    /* RelationalOperator: '<S353>/UpperRelop' incorporates:
     *  Constant: '<S345>/Constant'
     *  UnitDelay: '<S346>/Delay Input2'
     *
     * Block description for '<S346>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_LogicalOperator3 = (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_a
      < 0.0F);

    /* Switch: '<S353>/Switch' incorporates:
     *  Constant: '<S345>/Constant'
     *  UnitDelay: '<S346>/Delay Input2'
     *
     * Block description for '<S346>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_LogicalOperator3) {
      rtb_Switch2_mn = 0.0F;
    } else {
      rtb_Switch2_mn = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_a;
    }

    /* End of Switch: '<S353>/Switch' */
  }

  /* End of Switch: '<S353>/Switch2' */

  /* UnitDelay: '<S352>/Delay Input2'
   *
   * Block description for '<S352>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* Sum: '<S352>/Difference Inputs1'
   *
   * Block description for '<S352>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_Switch2_mn - rtb_Add10;

  /* SampleTimeMath: '<S352>/sample time'
   *
   * About '<S352>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S352>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(15.0 * elapseTime);

  /* RelationalOperator: '<S354>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S354>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S352>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-15.0 * elapseTime);

    /* RelationalOperator: '<S354>/UpperRelop' */
    rtb_LogicalOperator3 = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S354>/Switch' */
    if (rtb_LogicalOperator3) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S354>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S354>/Switch2' */

  /* Sum: '<S352>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S352>/Delay Input2'
   *
   * Block description for '<S352>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S352>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_f = rtb_Switch2_mn +
    rtb_Add10;

  /* UnitDelay: '<S345>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_ncs;

  /* Gain: '<S345>/Gain' */
  rtb_Add10 *= 0.0F;

  /* Saturate: '<S31>/Saturation' incorporates:
   *  Sum: '<S345>/Add'
   *  UnitDelay: '<S352>/Delay Input2'
   *
   * Block description for '<S352>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehVxEst_mps = rtb_Add10 +
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* SampleTimeMath: '<S347>/sample time'
   *
   * About '<S347>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* UnitDelay: '<S347>/Delay Input2'
   *
   * Block description for '<S347>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hu;

  /* Sum: '<S347>/Difference Inputs1'
   *
   * Block description for '<S347>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_MaxWhlSpd_mps_n - rtb_Add10;

  /* Product: '<S347>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S356>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S356>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S347>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S356>/UpperRelop' */
    rtb_LogicalOperator3 = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S356>/Switch' */
    if (rtb_LogicalOperator3) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S356>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S356>/Switch2' */

  /* Sum: '<S347>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S347>/Delay Input2'
   *
   * Block description for '<S347>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S347>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hu = rtb_Switch2_mn +
    rtb_Add10;

  /* Sum: '<S288>/Add7' incorporates:
   *  Constant: '<S288>/steptime5'
   */
  rtb_Add7 = 1.0F - rtb_Add7;

  /* Product: '<S288>/Divide4' incorporates:
   *  UnitDelay: '<S288>/Unit Delay4'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_mn = rtb_Add7 * rtb_Add6;

  /* Update for MinMax: '<S349>/Min3' incorporates:
   *  UnitDelay: '<S303>/Unit Delay'
   *  UnitDelay: '<S307>/Delay Input2'
   *
   * Block description for '<S307>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_p =
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* Update for UnitDelay: '<S296>/Unit Delay' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_j = rtb_Gain3_o;

  /* Update for UnitDelay: '<S304>/Unit Delay' incorporates:
   *  UnitDelay: '<S310>/Delay Input2'
   *
   * Block description for '<S310>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_pj =
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* Update for UnitDelay: '<S296>/Unit Delay1' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_n = rtb_Acc_POS;

  /* Update for UnitDelay: '<S305>/Unit Delay' incorporates:
   *  UnitDelay: '<S313>/Delay Input2'
   *
   * Block description for '<S313>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_a =
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* Update for UnitDelay: '<S296>/Unit Delay2' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_l = rtb_deltafalllimit_n;

  /* Update for UnitDelay: '<S306>/Unit Delay' incorporates:
   *  UnitDelay: '<S316>/Delay Input2'
   *
   * Block description for '<S316>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_nc =
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* Update for UnitDelay: '<S296>/Unit Delay3' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE = rtb_deltafalllimit_om;

  /* Update for UnitDelay: '<S323>/Unit Delay' incorporates:
   *  UnitDelay: '<S327>/Delay Input2'
   *
   * Block description for '<S327>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_lh =
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* Update for UnitDelay: '<S297>/Unit Delay' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_a0 = rtb_Gain3_o;

  /* Update for UnitDelay: '<S324>/Unit Delay' incorporates:
   *  UnitDelay: '<S330>/Delay Input2'
   *
   * Block description for '<S330>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_ap =
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* Update for UnitDelay: '<S297>/Unit Delay1' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_a = rtb_Acc_POS;

  /* Update for UnitDelay: '<S325>/Unit Delay' incorporates:
   *  UnitDelay: '<S333>/Delay Input2'
   *
   * Block description for '<S333>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_o =
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_pdc;

  /* Update for UnitDelay: '<S297>/Unit Delay2' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_c = rtb_deltafalllimit_n;

  /* Update for UnitDelay: '<S326>/Unit Delay' incorporates:
   *  UnitDelay: '<S336>/Delay Input2'
   *
   * Block description for '<S336>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_ah =
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* Update for UnitDelay: '<S297>/Unit Delay3' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_d = rtb_deltafalllimit_om;

  /* Update for UnitDelay: '<S216>/Unit Delay' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_lha = VehVxEst_mps;

  /* Update for UnitDelay: '<S285>/Unit Delay4' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_m = Acc_POS_n;

  /* Update for UnitDelay: '<S216>/Unit Delay1' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_k = VehVxEst_mps;

  /* Update for UnitDelay: '<S285>/Unit Delay3' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_f = rtb_LogicalOperator7_m;

  /* Update for UnitDelay: '<S288>/Unit Delay3' incorporates:
   *  UnitDelay: '<S288>/Unit Delay1'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_p =
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_d;

  /* Update for UnitDelay: '<S288>/Unit Delay1' incorporates:
   *  UnitDelay: '<S347>/Delay Input2'
   *
   * Block description for '<S347>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_d =
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hu;

  /* Update for UnitDelay: '<S288>/Unit Delay2' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_f = rtb_CastToDouble;

  /* Update for UnitDelay: '<S345>/Unit Delay' incorporates:
   *  UnitDelay: '<S352>/Delay Input2'
   *
   * Block description for '<S352>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_ncs =
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms1' */

  /* S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* Logic: '<S113>/NOT' */
  rtb_LogicalOperator7_m = !(Trq_CUT != 0.0);

  /* Logic: '<S113>/AND' */
  rtb_LogicalOperator3 = ((KeyPressed != 0.0) && rtb_LogicalOperator7_m);

  /* RelationalOperator: '<S115>/Compare' incorporates:
   *  Constant: '<S115>/Constant'
   */
  Brk = (Brk_F >= 600);

  /* RelationalOperator: '<S116>/Compare' incorporates:
   *  Constant: '<S116>/Constant'
   */
  ACC_Release = (Acc_POS_n <= 50.0F);

  /* Logic: '<S113>/NOT1' */
  rtb_LogicalOperator7_m = !(VehCtrlMdel241025_2018b_amksp_B.AMKSWITCH_bx != 0.0);

  /* Switch: '<S113>/Switch' incorporates:
   *  Constant: '<S113>/Constant1'
   *  Switch: '<S113>/Switch10'
   *  Switch: '<S113>/Switch11'
   *  Switch: '<S113>/Switch3'
   */
  if (rtb_LogicalOperator7_m) {
    elapseTime = MCFL_bSystemReady;
    WhlSpdFL = MCFR_bSystemReady;
    WhlSpdFR = MCFL_bQuitInverterOn;
    VehCtrlMdel241025_2018b_amksp_B.Switch11 = MCFR_bQuitInverterOn;
  } else {
    elapseTime = 1.0;
    WhlSpdFL = 1.0;
    WhlSpdFR = 1.0;
    VehCtrlMdel241025_2018b_amksp_B.Switch11 = 1.0;
  }

  /* End of Switch: '<S113>/Switch' */

  /* Chart: '<S113>/Chart2' */
  FunctionCallSubsystem_ELAPS_T =
    VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3 -
    VehCtrlMdel241025_2018b_amks_DW.previousTicks_g;
  VehCtrlMdel241025_2018b_amks_DW.previousTicks_g =
    VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;
  if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_f +
      FunctionCallSubsystem_ELAPS_T <= 255U) {
    VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_f = (uint8_T)
      (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_f +
       FunctionCallSubsystem_ELAPS_T);
  } else {
    VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_f = MAX_uint8_T;
  }

  if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 +
      FunctionCallSubsystem_ELAPS_T <= 1023U) {
    VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 = (uint16_T)
      (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 +
       FunctionCallSubsystem_ELAPS_T);
  } else {
    VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 = 1023U;
  }

  VehCtrlMdel241025_2018b_amks_DW.sfEvent = -1;
  if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c1_VehCtrlMdel241025_
      == 0U) {
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c1_VehCtrlMdel241025_ =
      1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_VehStat = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_VehStat = 2U;
    VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_f = 0U;
    VehCtrlMdel241025_2018b_amksp_B.errorReset = 1.0;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_BeeperStat = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_BeeperStat = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_AMKDCon = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCon = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MCDCEnable = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCDCEnable = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MC_TorqueCUT = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_AMKDCready = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCready = 10U;
    VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i2 = 0U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_Output = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_AMKCANenable = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MCFL_InverterOn = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MCFR_InverterOn = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn = 1U;
  } else {
    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_VehStat != 0U) {
      VehCtrlMdel241025_2018b_VehStat(&controller_ready, &rtb_LogicalOperator3,
        &elapseTime, &WhlSpdFL, &WhlSpdFR);
    }

    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_BeeperStat != 0U)
    {
      switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_BeeperStat) {
       case VehCtrlMdel241025_2018b__IN_OFF:
        if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
            VehCtrlMdel241025_event_EbeepON) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_BeeperStat = 2U;
        }
        break;

       case VehCtrlMdel241025_2018b_a_IN_ON:
        if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
            VehCtrlMdel24102_event_EbeepOFF) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_BeeperStat = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_AMKDCon != 0U) {
      switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCon) {
       case VehCtrlMdel241025_2018b__IN_OFF:
        if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
            VehCtrlMdel241025_event_AMKDCON) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCon = 2U;
        }
        break;

       case VehCtrlMdel241025_2018b_a_IN_ON:
        if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
            VehCtrlMdel24102_event_AMKDCOFF) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCon = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MCDCEnable != 0U)
    {
      switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCDCEnable) {
       case VehCtrlMdel241025_2018b__IN_OFF:
        if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
            VehCtrlMdel2_event_MCDCEnableON) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCDCEnable = 2U;
        }
        break;

       case VehCtrlMdel241025_2018b_a_IN_ON:
        if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
            VehCtrlMdel_event_MCDCEnableOFF) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCDCEnable = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MC_TorqueCUT != 0U)
    {
      switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT) {
       case VehCtrlMdel241025_2018b__IN_OFF:
        if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
            VehCtrlMdel24102_event_TorqueON) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT = 2U;
        }
        break;

       case VehCtrlMdel241025_2018b_a_IN_ON:
        if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
            VehCtrlMdel2410_event_TorqueOFF) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_AMKDCready != 0U)
    {
      VehCtrlMdel241025_20_AMKDCready(&MCFL_bDCOn, &MCFR_bDCOn,
        &rtb_LogicalOperator3, &elapseTime, &WhlSpdFL, &WhlSpdFR);
    }

    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_Output != 0U) {
      VehCtrlMdel241025_2018b_amksp_B.VehReady =
        (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_VehStat ==
         VehCtrlMdel241025_2018_IN_Ready);
      beeper_state = (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_BeeperStat ==
                      VehCtrlMdel241025_2018b_a_IN_ON);
      MCFL_DCOn_setpoints =
        (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKDCon ==
         VehCtrlMdel241025_2018b_a_IN_ON);
      VehCtrlMdel241025_2018b_amksp_B.MCFL_DCEnable =
        (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCDCEnable ==
         VehCtrlMdel241025_2018b_a_IN_ON);
      MCFR_DCEnable = (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCDCEnable
                       == VehCtrlMdel241025_2018b_a_IN_ON);
      VehCtrlMdel241025_2018b_amksp_B.AMKMCFL_InverterOn =
        (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn ==
         VehCtrlMdel241025_2018b_a_IN_ON);
      MCFR_InverterOn =
        (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn ==
         VehCtrlMdel241025_2018b_a_IN_ON);
      VehCtrlMdel241025_2018b_amksp_B.MCFL_TorqueOn =
        (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT ==
         VehCtrlMdel241025_2018b_a_IN_ON);
      VehCtrlMdel241025_2018b_amksp_B.MCFR_TorqueOn =
        (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MC_TorqueCUT ==
         VehCtrlMdel241025_2018b_a_IN_ON);
    }

    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_AMKCANenable != 0U)
    {
      switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKCANenable) {
       case VehCtrlMdel241025_2018b__IN_OFF:
        if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
            VehCtrlMdel24102_event_AMKCANON) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 2U;
        }
        break;

       case VehCtrlMdel241025_2018b_a_IN_ON:
        if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
            VehCtrlMdel2410_event_AMKCANOFF) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_AMKCANenable = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MCFL_InverterOn !=
        0U) {
      switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn) {
       case VehCtrlMdel241025_2018b__IN_OFF:
        if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
            VehCtrlMdel2_event_InverterFLON) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn = 2U;
        }
        break;

       case VehCtrlMdel241025_2018b_a_IN_ON:
        if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
            VehCtrlMdel_event_InverterFLOFF) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFL_InverterOn = 1U;
        }
        break;
      }
    }

    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_MCFR_InverterOn !=
        0U) {
      switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn) {
       case VehCtrlMdel241025_2018b__IN_OFF:
        if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
            VehCtrlMdel2_event_InverterFRON) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn = 2U;
        }
        break;

       case VehCtrlMdel241025_2018b_a_IN_ON:
        if (VehCtrlMdel241025_2018b_amks_DW.sfEvent ==
            VehCtrlMdel_event_InverterFROFF) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_MCFR_InverterOn = 1U;
        }
        break;
      }
    }
  }

  /* End of Chart: '<S113>/Chart2' */

  /* Switch: '<S113>/Switch4' */
  VehCtrlMdel241025_2018b_amksp_B.MCFL_DCOn_setpoints_o =
    (rtb_LogicalOperator7_m && MCFL_DCOn_setpoints);

  /* End of Outputs for S-Function (fcncallgen): '<S2>/10ms' */

  /* S-Function (fcncallgen): '<S5>/10ms1' incorporates:
   *  SubSystem: '<S5>/Beeper'
   */
  /* Logic: '<S358>/Logical Operator2' */
  rtb_LogicalOperator2 = !rtb_LogicalOperator2;

  /* S-Function (ec5744_pdsslb2u3): '<S358>/PowerDriverSwitch(LS)' */
  L9826VAR701[2]= beeper_state;
  ec_l9826tr701_control(L9826VAR701);

  /* S-Function (ec5744_pdsslb2u3): '<S358>/PowerDriverSwitch(LS)2' */
  L9826VAR701[6]= beeper_state;
  ec_l9826tr701_control(L9826VAR701);

  /* Chart: '<S358>/Timer2' incorporates:
   *  Constant: '<S358>/Constant3'
   */
  VehCtrlMdel241025_20_Timer1(rtb_LogicalOperator2, 1.0F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_h,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer2_h);

  /* RelationalOperator: '<S373>/Compare' incorporates:
   *  Constant: '<S373>/Constant'
   */
  rtb_LogicalOperator2 = (VehCtrlMdel241025_2018b_amksp_B.Exit_h > 0.0);

  /* RelationalOperator: '<S365>/FixPt Relational Operator' incorporates:
   *  UnitDelay: '<S365>/Delay Input1'
   *
   * Block description for '<S365>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput1_DSTATE = ((int32_T)
    rtb_LogicalOperator2 > (int32_T)
    VehCtrlMdel241025_2018b_amks_DW.DelayInput1_DSTATE);

  /* Logic: '<S358>/OR' incorporates:
   *  UnitDelay: '<S365>/Delay Input1'
   *
   * Block description for '<S365>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_LogicalOperator7_m = VehCtrlMdel241025_2018b_amks_DW.DelayInput1_DSTATE;

  /* Chart: '<S358>/Chart' */
  FunctionCallSubsystem_ELAPS_T =
    VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3 -
    VehCtrlMdel241025_2018b_amks_DW.previousTicks_m;
  VehCtrlMdel241025_2018b_amks_DW.previousTicks_m =
    VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;
  if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_p +
      FunctionCallSubsystem_ELAPS_T <= 31U) {
    VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_p = (uint8_T)
      (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_p +
       FunctionCallSubsystem_ELAPS_T);
  } else {
    VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_p = 31U;
  }

  if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c24_VehCtrlMdel241025
      == 0U) {
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c24_VehCtrlMdel241025 =
      1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c24_VehCtrlMdel241025_2018b_ =
      2U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_STATEON = 2U;
    VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_p = 0U;
    HVSWITCH = false;
  } else if
      (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c24_VehCtrlMdel241025_2018b_
       == VehCtrlMdel241025_2_IN_STATEOFF) {
    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_STATEOFF ==
        VehCtrlMdel241025_2018b__IN_OFF) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_STATEOFF = 0U;
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c24_VehCtrlMdel241025_2018b_
        = 2U;
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_STATEON = 2U;
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_p = 0U;
      HVSWITCH = false;
    } else {
      /* case IN_initstate1: */
      HVSWITCH = true;
      rtb_LogicalOperator3 =
        ((VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_p >= 20U) &&
         rtb_LogicalOperator7_m);
      if (rtb_LogicalOperator3) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_STATEOFF = 1U;
        HVSWITCH = false;
      }
    }
  } else {
    /* case IN_STATEON: */
    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_STATEON ==
        VehCtrlMdel241025_2018b_IN_ON_d) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_STATEON = 0U;
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c24_VehCtrlMdel241025_2018b_
        = 1U;
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_STATEOFF = 2U;
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_p = 0U;
      HVSWITCH = true;
    } else {
      /* case IN_initstate: */
      HVSWITCH = false;
      rtb_LogicalOperator7_m =
        ((VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_p >= 20U) &&
         rtb_LogicalOperator7_m);
      if (rtb_LogicalOperator7_m) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_STATEON = 1U;
        HVSWITCH = true;
      }
    }
  }

  /* End of Chart: '<S358>/Chart' */

  /* S-Function (ec5744_pdsslbu3): '<S358>/PowerDriverSwitch(HS)2' */

  /* Set level HVSWITCH for the specified power driver switch */
  ec_gpio_write(57,HVSWITCH);

  /* Logic: '<S358>/OR2' */
  rtb_Compare_i = ((MCFL_bError != 0.0) || (MCFR_bError != 0.0));

  /* Outputs for Enabled SubSystem: '<S358>/Enabled Subsystem1' incorporates:
   *  EnablePort: '<S366>/Enable'
   */
  if (rtb_Compare_i) {
    if (!VehCtrlMdel241025_2018b_amks_DW.EnabledSubsystem1_MODE) {
      /* Enable for Chart: '<S366>/Chart' */
      VehCtrlMdel241025_2018b_amks_DW.previousTicks_e =
        VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;
      VehCtrlMdel241025_2018b_amks_DW.EnabledSubsystem1_MODE = true;
    }

    /* Chart: '<S366>/Chart' */
    FunctionCallSubsystem_ELAPS_T =
      VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3 -
      VehCtrlMdel241025_2018b_amks_DW.previousTicks_e;
    VehCtrlMdel241025_2018b_amks_DW.previousTicks_e =
      VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;
    if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j +
        FunctionCallSubsystem_ELAPS_T <= 127U) {
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j = (uint8_T)
        (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j +
         FunctionCallSubsystem_ELAPS_T);
    } else {
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j = 127U;
    }

    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c27_VehCtrlMdel241025
        == 0U) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c27_VehCtrlMdel241025
        = 1U;
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel241025_2018b_
        = 1U;
    } else {
      switch
        (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel241025_2018b_)
      {
       case VehCtrlMdel241025_201_IN_Init_e:
        rtb_LogicalOperator3 = ((MCFL_bError != 0.0) && (!(MCFR_bError != 0.0)));
        if (rtb_LogicalOperator3) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel241025_2018b_
            = 2U;
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateA = 2U;
          VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j = 0U;
          VehCtrlMdel241025_2018b_amksp_B.LEDOn = 0.0;
        } else {
          rtb_LogicalOperator7_m = ((!(MCFL_bError != 0.0)) && (MCFR_bError !=
            0.0));
          if (rtb_LogicalOperator7_m) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel241025_2018b_
              = 3U;
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateB = 2U;
            VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j = 0U;
            VehCtrlMdel241025_2018b_amksp_B.LEDOn = 0.0;
          } else {
            rtb_LogicalOperator3 = ((MCFL_bError != 0.0) && (MCFR_bError != 0.0));
            if (rtb_LogicalOperator3) {
              VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel241025_2018b_
                = 4U;
              VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateC = 2U;
              VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j = 0U;
              VehCtrlMdel241025_2018b_amksp_B.LEDOn = 0.0;
            }
          }
        }
        break;

       case VehCtrlMdel241025_201_IN_StateA:
        rtb_LogicalOperator3 = ((!(MCFL_bError != 0.0)) || (MCFR_bError != 0.0));
        if (rtb_LogicalOperator3) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateA = 0U;
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel241025_2018b_
            = 1U;
        } else if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateA ==
                   VehCtrlMdel241025_201_IN_LEDOFF) {
          VehCtrlMdel241025_2018b_amksp_B.LEDOn = 1.0;
          if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j >= 80U) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateA = 2U;
            VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j = 0U;
            VehCtrlMdel241025_2018b_amksp_B.LEDOn = 0.0;
          }
        } else {
          /* case IN_LEDON: */
          VehCtrlMdel241025_2018b_amksp_B.LEDOn = 0.0;
          if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j >= 80U) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateA = 1U;
            VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j = 0U;
            VehCtrlMdel241025_2018b_amksp_B.LEDOn = 1.0;
          }
        }
        break;

       case VehCtrlMdel241025_201_IN_StateB:
        rtb_LogicalOperator7_m = ((MCFL_bError != 0.0) || (!(MCFR_bError != 0.0)));
        if (rtb_LogicalOperator7_m) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateB = 0U;
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel241025_2018b_
            = 1U;
        } else if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateB ==
                   VehCtrlMdel241025_201_IN_LEDOFF) {
          VehCtrlMdel241025_2018b_amksp_B.LEDOn = 1.0;
          if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j >= 40U) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateB = 2U;
            VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j = 0U;
            VehCtrlMdel241025_2018b_amksp_B.LEDOn = 0.0;
          }
        } else {
          /* case IN_LEDON: */
          VehCtrlMdel241025_2018b_amksp_B.LEDOn = 0.0;
          if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j >= 40U) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateB = 1U;
            VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j = 0U;
            VehCtrlMdel241025_2018b_amksp_B.LEDOn = 1.0;
          }
        }
        break;

       default:
        /* case IN_StateC: */
        rtb_LogicalOperator7_m = ((!(MCFL_bError != 0.0)) || (!(MCFR_bError !=
          0.0)));
        if (rtb_LogicalOperator7_m) {
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateC = 0U;
          VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel241025_2018b_
            = 1U;
        } else if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateC ==
                   VehCtrlMdel241025_201_IN_LEDOFF) {
          VehCtrlMdel241025_2018b_amksp_B.LEDOn = 1.0;
          if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j >= 10U) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateC = 2U;
            VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j = 0U;
            VehCtrlMdel241025_2018b_amksp_B.LEDOn = 0.0;
          }
        } else {
          /* case IN_LEDON: */
          VehCtrlMdel241025_2018b_amksp_B.LEDOn = 0.0;
          if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j >= 10U) {
            VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_StateC = 1U;
            VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j = 0U;
            VehCtrlMdel241025_2018b_amksp_B.LEDOn = 1.0;
          }
        }
        break;
      }
    }

    /* End of Chart: '<S366>/Chart' */
  } else {
    if (VehCtrlMdel241025_2018b_amks_DW.EnabledSubsystem1_MODE) {
      /* Disable for Chart: '<S366>/Chart' */
      FunctionCallSubsystem_ELAPS_T =
        VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3 -
        VehCtrlMdel241025_2018b_amks_DW.previousTicks_e;
      VehCtrlMdel241025_2018b_amks_DW.previousTicks_e =
        VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;
      if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j +
          FunctionCallSubsystem_ELAPS_T <= 127U) {
        VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j = (uint8_T)
          (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j +
           FunctionCallSubsystem_ELAPS_T);
      } else {
        VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1_j = 127U;
      }

      /* End of Disable for Chart: '<S366>/Chart' */
      VehCtrlMdel241025_2018b_amks_DW.EnabledSubsystem1_MODE = false;
    }
  }

  /* End of Outputs for SubSystem: '<S358>/Enabled Subsystem1' */

  /* Logic: '<S358>/Logical Operator5' incorporates:
   *  UnitDelay: '<S365>/Delay Input1'
   *
   * Block description for '<S365>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput1_DSTATE = !rtb_Compare_i;

  /* Switch: '<S358>/Switch1' incorporates:
   *  UnitDelay: '<S365>/Delay Input1'
   *
   * Block description for '<S365>/Delay Input1':
   *
   *  Store in Global RAM
   */
  STATEDISPLAY = (VehCtrlMdel241025_2018b_amks_DW.DelayInput1_DSTATE ||
                  (VehCtrlMdel241025_2018b_amksp_B.LEDOn != 0.0));

  /* S-Function (ec5744_pdsslb2u3): '<S358>/PowerDriverSwitch(LS)1' */
  L9826VAR701[3]= STATEDISPLAY;
  ec_l9826tr701_control(L9826VAR701);

  /* S-Function (ec5744_pdsslb2u3): '<S358>/PowerDriverSwitch(LS)5' */
  L9826VAR702[1]= STATEDISPLAY;
  ec_l9826tr702_control(L9826VAR702);

  /* Logic: '<S358>/AND' */
  VehCtrlMdel241025_2018b_amksp_B.AND = (rtb_Compare && rtb_LowerRelop1_b);

  /* S-Function (ec5744_pdsslb2u3): '<S358>/PowerDriverSwitch(LS)3' */
  L9826VAR702[6]= VehCtrlMdel241025_2018b_amksp_B.AND;
  ec_l9826tr702_control(L9826VAR702);

  /* S-Function (ec5744_pdsslb2u3): '<S358>/PowerDriverSwitch(LS)4' */
  L9826VAR702[3]= VehCtrlMdel241025_2018b_amksp_B.AND;
  ec_l9826tr702_control(L9826VAR702);

  /* RelationalOperator: '<S364>/Compare' incorporates:
   *  Constant: '<S364>/Constant'
   */
  rtb_Compare = (voltage > 61.0);

  /* Chart: '<S358>/Timer3' incorporates:
   *  Constant: '<S358>/Constant5'
   */
  VehCtrlMdel241025_20_Timer1(rtb_Compare, 0.11F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer3_f);

  /* Outputs for Enabled SubSystem: '<S358>/Subsystem' incorporates:
   *  EnablePort: '<S370>/Enable'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.Exit > 0.0) {
    if (!VehCtrlMdel241025_2018b_amks_DW.Subsystem_MODE) {
      /* Enable for Chart: '<S370>/Chart1' */
      VehCtrlMdel241025_2018b_amks_DW.previousTicks =
        VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;
      VehCtrlMdel241025_2018b_amks_DW.Subsystem_MODE = true;
    }

    /* Chart: '<S370>/Chart1' */
    FunctionCallSubsystem_ELAPS_T =
      VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3 -
      VehCtrlMdel241025_2018b_amks_DW.previousTicks;
    VehCtrlMdel241025_2018b_amks_DW.previousTicks =
      VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;
    if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1 +
        FunctionCallSubsystem_ELAPS_T <= 15U) {
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1 = (uint8_T)
        (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1 +
         FunctionCallSubsystem_ELAPS_T);
    } else {
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1 = 15U;
    }

    if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c32_VehCtrlMdel241025
        == 0U) {
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c32_VehCtrlMdel241025
        = 1U;
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c32_VehCtrlMdel241025_2018b_
        = 2U;
      VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1 = 0U;
      VehCtrlMdel241025_2018b_amksp_B.led = true;
    } else if
        (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c32_VehCtrlMdel241025_2018b_
         == VehCtrlMdel241025_201_IN_REDOFF) {
      VehCtrlMdel241025_2018b_amksp_B.led = false;
      if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1 >= 13U) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c32_VehCtrlMdel241025_2018b_
          = 2U;
        VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1 = 0U;
        VehCtrlMdel241025_2018b_amksp_B.led = true;
      }
    } else {
      /* case IN_REDON: */
      VehCtrlMdel241025_2018b_amksp_B.led = true;
      if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1 >= 13U) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_c32_VehCtrlMdel241025_2018b_
          = 1U;
        VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1 = 0U;
        VehCtrlMdel241025_2018b_amksp_B.led = false;
      }
    }

    /* End of Chart: '<S370>/Chart1' */
  } else {
    if (VehCtrlMdel241025_2018b_amks_DW.Subsystem_MODE) {
      /* Disable for Chart: '<S370>/Chart1' */
      FunctionCallSubsystem_ELAPS_T =
        VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3 -
        VehCtrlMdel241025_2018b_amks_DW.previousTicks;
      VehCtrlMdel241025_2018b_amks_DW.previousTicks =
        VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;
      if (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1 +
          FunctionCallSubsystem_ELAPS_T <= 15U) {
        VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1 = (uint8_T)
          (VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1 +
           FunctionCallSubsystem_ELAPS_T);
      } else {
        VehCtrlMdel241025_2018b_amks_DW.temporalCounter_i1 = 15U;
      }

      /* End of Disable for Chart: '<S370>/Chart1' */
      VehCtrlMdel241025_2018b_amks_DW.Subsystem_MODE = false;
    }
  }

  /* End of Outputs for SubSystem: '<S358>/Subsystem' */

  /* S-Function (ec5744_pdsslb2u3): '<S358>/PowerDriverSwitch(LS)6' */
  L9826VAR701[5]= VehCtrlMdel241025_2018b_amksp_B.led;
  ec_l9826tr701_control(L9826VAR701);

  /* S-Function (ec5744_pdsslb2u3): '<S358>/PowerDriverSwitch(LS)7' */
  L9826VAR701[7]= VehCtrlMdel241025_2018b_amksp_B.led;
  ec_l9826tr701_control(L9826VAR701);

  /* Update for UnitDelay: '<S365>/Delay Input1'
   *
   * Block description for '<S365>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput1_DSTATE = rtb_LogicalOperator2;

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms1' */

  /* S-Function (fcncallgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/PwrTrainTempPrtct'
   */
  /* MinMax: '<S8>/Max' */
  elapseTime = fmax(MCFL_TempIGBT, MCFR_TempIGBT);
  elapseTime = fmax(elapseTime, MCFL_TempInverter);
  elapseTime = fmax(elapseTime, MCFR_TempInverter);

  /* RelationalOperator: '<S103>/Compare' incorporates:
   *  Constant: '<S103>/Constant'
   */
  rtb_LowerRelop1_b = (elapseTime > 35.0);

  /* RelationalOperator: '<S104>/Compare' incorporates:
   *  Constant: '<S104>/Constant'
   */
  rtb_LogicalOperator2 = (elapseTime > 45.0);

  /* Logic: '<S8>/NOT' */
  rtb_LogicalOperator7_m = !rtb_LogicalOperator2;

  /* Logic: '<S8>/AND' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b && rtb_LogicalOperator7_m);

  /* Switch: '<S8>/Switch' incorporates:
   *  Constant: '<S8>/Constant'
   */
  if (rtb_LowerRelop1_b) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table1' */
    WhlSpdFR = look1_binlx(elapseTime, VehCtrlMdel241025_2018b__ConstP.pooled19,
      VehCtrlMdel241025_2018b__ConstP.pooled18, 7U);
  } else {
    WhlSpdFR = 0.0;
  }

  /* End of Switch: '<S8>/Switch' */

  /* MinMax: '<S8>/Max2' */
  WhlSpdFL = fmax(MCFL_TempMotor, MCFR_TempMotor);

  /* RelationalOperator: '<S108>/Compare' incorporates:
   *  Constant: '<S108>/Constant'
   */
  rtb_LogicalOperator7_m = (WhlSpdFL > 45.0);

  /* Logic: '<S8>/NOT2' */
  rtb_LowerRelop1_b = !rtb_LogicalOperator7_m;

  /* RelationalOperator: '<S109>/Compare' incorporates:
   *  Constant: '<S109>/Constant'
   */
  rtb_Compare = (WhlSpdFL > 35.0);

  /* Logic: '<S8>/AND3' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b && rtb_Compare);

  /* Switch: '<S8>/Switch2' incorporates:
   *  Constant: '<S8>/Constant4'
   */
  if (rtb_LowerRelop1_b) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table4' */
    WhlSpdRL_mps = look1_binlx(WhlSpdFL,
      VehCtrlMdel241025_2018b__ConstP.pooled19,
      VehCtrlMdel241025_2018b__ConstP.pooled18, 7U);
  } else {
    WhlSpdRL_mps = 0.0;
  }

  /* End of Switch: '<S8>/Switch2' */

  /* MinMax: '<S8>/Max3' */
  WhlSpdRL_mps = fmax(WhlSpdRL_mps, WhlSpdFR);

  /* SignalConversion generated from: '<S8>/Out1' */
  WhlSpdFR = WhlSpdRL_mps;

  /* Logic: '<S8>/AND2' */
  rtb_Compare = (rtb_LogicalOperator7_m && rtb_LogicalOperator2);

  /* Chart: '<S8>/Timer1' incorporates:
   *  Constant: '<S8>/Constant2'
   */
  VehCtrlMdel241025_20_Timer1(rtb_Compare, 0.11F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_g,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer1);

  /* RelationalOperator: '<S106>/Compare' incorporates:
   *  Constant: '<S106>/Constant'
   */
  rtb_Compare = (MCU_Temp > 80.0);

  /* Chart: '<S8>/Timer2' incorporates:
   *  Constant: '<S8>/Constant3'
   */
  VehCtrlMdel241025_20_Timer1(rtb_Compare, 0.11F,
    &VehCtrlMdel241025_2018b_amksp_B.Exit_d,
    &VehCtrlMdel241025_2018b_amks_DW.sf_Timer2);

  /* SignalConversion generated from: '<S8>/Out1' */
  EMRAX_Trq_CUT = VehCtrlMdel241025_2018b_amksp_B.Exit_d;

  /* Logic: '<S8>/NOT1' */
  rtb_Compare = !rtb_Compare;

  /* RelationalOperator: '<S105>/Compare' incorporates:
   *  Constant: '<S105>/Constant'
   */
  rtb_LogicalOperator7_m = (MCU_Temp > 45.0);

  /* Logic: '<S8>/AND1' */
  rtb_LogicalOperator7_m = (rtb_LogicalOperator7_m && rtb_Compare);

  /* MinMax: '<S8>/Max1' */
  rtb_g_mpss1 = fmax(MCU_Temp, motor_Temp);

  /* Switch: '<S8>/Switch1' incorporates:
   *  Constant: '<S8>/Constant1'
   */
  if (rtb_LogicalOperator7_m) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table3' */
    WhlSpdRL_mps = look1_binlx(rtb_g_mpss1,
      VehCtrlMdel241025_2018b__ConstP.uDLookupTable3_bp01Data,
      VehCtrlMdel241025_2018b__ConstP.pooled18, 7U);
  } else {
    WhlSpdRL_mps = 0.0;
  }

  /* End of Switch: '<S8>/Switch1' */

  /* SignalConversion generated from: '<S8>/Out1' */
  WhlSpdRR_mps = WhlSpdRL_mps;

  /* Lookup_n-D: '<S8>/2-D Lookup Table2' */
  WhlSpdRL_mps = look1_binlx(rtb_g_mpss1,
    VehCtrlMdel241025_2018b__ConstP.uDLookupTable2_bp01Data,
    VehCtrlMdel241025_2018b__ConstP.uDLookupTable2_tableData, 6U);

  /* DataTypeConversion: '<S8>/Cast To Single' */
  rtb_CastToDouble = (real32_T)WhlSpdRL_mps;

  /* Gain: '<S8>/Gain' */
  rtb_CastToDouble *= 10.0F;

  /* SignalConversion generated from: '<S8>/Out1' */
  AMK_Trq_CUT = VehCtrlMdel241025_2018b_amksp_B.Exit_g;

  /* RelationalOperator: '<S110>/Compare' incorporates:
   *  Constant: '<S110>/Constant'
   */
  rtb_Compare = (WhlSpdFL > 50.0);

  /* SignalConversion generated from: '<S8>/Out1' */
  VehCtrlMdel241025_2018b_amksp_B.bWaterPumpON = rtb_Compare;

  /* RelationalOperator: '<S107>/Compare' incorporates:
   *  Constant: '<S107>/Constant'
   */
  rtb_Compare = (elapseTime > 30.0);

  /* SignalConversion generated from: '<S8>/Out1' */
  VehCtrlMdel241025_2018b_amksp_B.aWaterPumpON = rtb_Compare;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/Function-Call Generator' */

  /* S-Function (fcncallgen): '<S1>/10ms1' incorporates:
   *  SubSystem: '<S1>/MoTrqReq'
   */
  if (VehCtrlMdel241025_2018b_amks_DW.MoTrqReq_RESET_ELAPS_T) {
    FunctionCallSubsystem_ELAPS_T = 0U;
  } else {
    FunctionCallSubsystem_ELAPS_T =
      VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3 -
      VehCtrlMdel241025_2018b_amks_DW.MoTrqReq_PREV_T;
  }

  VehCtrlMdel241025_2018b_amks_DW.MoTrqReq_PREV_T =
    VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;
  VehCtrlMdel241025_2018b_amks_DW.MoTrqReq_RESET_ELAPS_T = false;

  /* DataTypeConversion: '<S28>/Cast To Single1' */
  rtb_Add6 = (real32_T)MCFL_ActualVelocity;
  rtb_Add7 = (real32_T)MCFR_ActualVelocity;
  rtb_Switch2_mn = (real32_T)RPM;

  /* DataTypeConversion: '<S28>/Cast To Single' */
  rtb_Switch2_b0 = (real32_T)MCFL_ActualTorque;
  rtb_Add10 = (real32_T)MCFR_ActualTorque;
  rtb_Add4_j = (real32_T)trq;

  /* Product: '<S28>/Product6' */
  rtb_Add6 *= rtb_Switch2_b0;
  rtb_Add6 += rtb_Add7 * rtb_Add10;
  rtb_Add6 += rtb_Switch2_mn * rtb_Add4_j;

  /* Gain: '<S28>/Gain3' */
  PwrALL = 0.000104712039F * rtb_Add6;

  /* RelationalOperator: '<S28>/Relational Operator7' incorporates:
   *  Constant: '<S28>/Constant10'
   */
  rtb_Compare = (PwrALL > 72.0F);

  /* Outputs for Enabled SubSystem: '<S71>/POSITIVE Edge' incorporates:
   *  EnablePort: '<S73>/Enable'
   */
  if (VehCtrlMdel241025_2018b__ConstB.MultiportSwitch[0] > 0.0) {
    /* RelationalOperator: '<S73>/Relational Operator1' incorporates:
     *  UnitDelay: '<S71>/Unit Delay'
     */
    VehCtrlMdel241025_2018b_amksp_B.RelationalOperator1 = ((int32_T)rtb_Compare >
      (int32_T)VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_pl);
  }

  /* End of Outputs for SubSystem: '<S71>/POSITIVE Edge' */

  /* Outputs for Enabled SubSystem: '<S71>/NEGATIVE Edge' incorporates:
   *  EnablePort: '<S72>/Enable'
   */
  if (VehCtrlMdel241025_2018b__ConstB.MultiportSwitch[1] > 0.0) {
    /* RelationalOperator: '<S72>/Relational Operator1' incorporates:
     *  UnitDelay: '<S71>/Unit Delay'
     */
    VehCtrlMdel241025_2018b_amksp_B.RelationalOperator1_c = ((int32_T)
      VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_pl > (int32_T)rtb_Compare);
  }

  /* End of Outputs for SubSystem: '<S71>/NEGATIVE Edge' */

  /* Logic: '<S71>/Logical Operator1' */
  rtb_LowerRelop1_b = (VehCtrlMdel241025_2018b_amksp_B.RelationalOperator1 ||
                       VehCtrlMdel241025_2018b_amksp_B.RelationalOperator1_c);

  /* Logic: '<S69>/NOT' */
  rtb_LogicalOperator7_m = !rtb_Compare;

  /* Logic: '<S69>/AND1' */
  rtb_LogicalOperator7_m = (rtb_LogicalOperator7_m || rtb_LowerRelop1_b);

  /* Switch: '<S69>/Switch1' */
  if (rtb_LogicalOperator7_m) {
    /* MinMax: '<S69>/MinMax' incorporates:
     *  Constant: '<S69>/Constant8'
     *  UnitDelay: '<S69>/Unit Delay'
     */
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_l = 0.0;
  } else {
    /* Sum: '<S69>/Add5' incorporates:
     *  Constant: '<S28>/Constant9'
     */
    elapseTime = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_l +
      0.0099999997764825821;

    /* MinMax: '<S69>/MinMax' incorporates:
     *  Constant: '<S28>/Constant8'
     */
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_l = fmin
      (0.10999999940395355, elapseTime);
  }

  /* End of Switch: '<S69>/Switch1' */

  /* RelationalOperator: '<S69>/Relational Operator' incorporates:
   *  Constant: '<S28>/Constant8'
   *  UnitDelay: '<S69>/Unit Delay'
   */
  rtb_LogicalOperator7_m = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_l >=
    0.10999999940395355);

  /* Logic: '<S69>/AND' */
  rtb_LogicalOperator7_m = (rtb_Compare && rtb_LogicalOperator7_m);

  /* Sum: '<S28>/Add4' incorporates:
   *  Constant: '<S28>/Constant'
   */
  WhlSpdRL_mps = 1.0 - WhlSpdFR;

  /* Switch: '<S28>/Switch1' incorporates:
   *  Constant: '<S28>/Constant12'
   */
  if (rtb_LogicalOperator7_m) {
    rtb_Gain5 = 2.0;
  } else {
    /* Sum: '<S28>/Add2' incorporates:
     *  Constant: '<S28>/RPM_min2'
     */
    WhlSpdFL = MCFR_ActualVelocity + 10.0;

    /* MinMax: '<S28>/Max1' incorporates:
     *  Constant: '<S28>/RPM_min3'
     */
    elapseTime = fmax(WhlSpdFL, 1.0);

    /* Switch: '<S28>/Switch4' incorporates:
     *  Constant: '<S28>/Constant15'
     *  Constant: '<S28>/Constant4'
     */
    if (VehCtrlMdel241025_2018b_amksp_B.ModeSW_o != 0.0) {
      rtb_Switch4_o = 6;
    } else {
      rtb_Switch4_o = 11;
    }

    /* End of Switch: '<S28>/Switch4' */

    /* Product: '<S28>/Product4' */
    WhlSpdFL = (real_T)rtb_Switch4_o * WhlSpdRL_mps;

    /* Product: '<S28>/Product1' */
    WhlSpdFL *= 9550.0;

    /* Product: '<S28>/Divide1' */
    rtb_Gain5 = WhlSpdFL / elapseTime;
  }

  /* End of Switch: '<S28>/Switch1' */

  /* UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain4 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_p;

  /* SampleTimeMath: '<S42>/sample time'
   *
   * About '<S42>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S42>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  rtb_Yk1 = 2000.0 * elapseTime;

  /* MATLAB Function: '<S10>/ÔØºÉ×ªÒÆ' incorporates:
   *  Constant: '<S10>/Constant10'
   *  Constant: '<S10>/Constant2'
   *  Constant: '<S10>/Constant3'
   *  Constant: '<S10>/Constant4'
   *  Constant: '<S10>/Constant5'
   *  Constant: '<S10>/Constant9'
   */
  rtb_Add7 = (1666.0F - 340.0F * (real32_T)rtb_deltafalllimit_le * 0.29F / 1.2F)
    * 0.521984875F - 170.0F * (real32_T)rtb_Yk1_l * 0.29F / 1.592F;
  rtb_Switch2_mn = (340.0F * (real32_T)rtb_deltafalllimit_le * 0.29F / 1.2F +
                    1666.0F) * 0.521984875F - 170.0F * (real32_T)rtb_Yk1_l *
    0.29F / 1.592F;
  rtb_Switch2_b0 = (1666.0F - 340.0F * (real32_T)rtb_deltafalllimit_le * 0.29F /
                    1.2F) * 0.521984875F + 170.0F * (real32_T)rtb_Yk1_l * 0.29F /
    1.592F;
  rtb_Add10 = (340.0F * (real32_T)rtb_deltafalllimit_le * 0.29F / 1.2F + 1666.0F)
    * 0.521984875F + 170.0F * (real32_T)rtb_Yk1_l * 0.29F / 1.592F;

  /* Lookup_n-D: '<S10>/AMK' */
  WhlSpdFL = look1_binlx(MCFL_ActualVelocity,
    VehCtrlMdel241025_2018b__ConstP.pooled8,
    VehCtrlMdel241025_2018b__ConstP.pooled7, 19U);

  /* Lookup_n-D: '<S10>/AMK1' */
  WhlSpdFR = look1_binlx(MCFR_ActualVelocity,
    VehCtrlMdel241025_2018b__ConstP.pooled8,
    VehCtrlMdel241025_2018b__ConstP.pooled7, 19U);

  /* Lookup_n-D: '<S10>/228' */
  rtb_g_mpss1 = look1_binlx(RPM, VehCtrlMdel241025_2018b__ConstP.pooled4,
    VehCtrlMdel241025_2018b__ConstP.u28_tableData, 26U);

  /* Gain: '<S10>/Gain3' */
  rtb_Switch2_on = 0.1020408163265306 * rtb_deltafalllimit_le;

  /* MATLAB Function: '<S10>/MATLAB Function' incorporates:
   *  Constant: '<S10>/Constant11'
   *  Constant: '<S10>/Constant12'
   *  Constant: '<S10>/Constant13'
   *  Constant: '<S10>/Constant26'
   */
  rtb_Add6 = rtb_Add7 * 0.75F;
  rtb_Add7 = rtb_Add7 * (real32_T)rtb_Switch2_on / 9.8F;
  rtb_Add4_j = rtb_Switch2_mn * 0.75F;
  rtb_VxIMU_est = rtb_Switch2_mn * (real32_T)rtb_Switch2_on / 9.8F;
  rtb_Switch2_mn = rtb_Switch2_b0 * 0.75F;
  rtb_Switch2_b0 = rtb_Switch2_b0 * (real32_T)rtb_Switch2_on / 9.8F;
  rtb_Ax = rtb_Add10 * 0.75F;
  rtb_MaxWhlSpd_mps_n = rtb_Add10 * (real32_T)rtb_Switch2_on / 9.8F;
  rtb_Add10 = fminf(sqrtf(rtb_Add4_j * rtb_Add4_j - rtb_VxIMU_est *
    rtb_VxIMU_est) * 0.2F / 11.4F, (real32_T)WhlSpdFR);
  rtb_Add6 = fminf(sqrtf(rtb_Add6 * rtb_Add6 - rtb_Add7 * rtb_Add7) * 0.2F /
                   11.4F, (real32_T)WhlSpdFL);
  rtb_Add7 = fminf(fminf(sqrtf(rtb_Ax * rtb_Ax - rtb_MaxWhlSpd_mps_n *
    rtb_MaxWhlSpd_mps_n), sqrtf(rtb_Switch2_mn * rtb_Switch2_mn - rtb_Switch2_b0
    * rtb_Switch2_b0)) * 0.2F / 3.4F, (real32_T)rtb_g_mpss1);

  /* Gain: '<S10>/Gain1' */
  rtb_Add6 *= 0.95F;

  /* Gain: '<S7>/Gain' */
  rtb_Switch2_mn = 3.6F * VehVxEst_mps;

  /* Logic: '<S7>/OR3' */
  rtb_ignition_e = ((VehCtrlMdel241025_2018b_amksp_B.AMKSWITCH_bx != 0.0) ||
                    rtb_ignition_e);

  /* Switch: '<S29>/Switch' */
  if (rtb_ignition_e) {
    /* Lookup_n-D: '<S29>/4WD_Table' */
    rtb_Switch2_mn = look2_iflf_binlx(Acc_POS_n, rtb_Switch2_mn,
      VehCtrlMdel241025_2018b__ConstP.pooled34,
      VehCtrlMdel241025_2018b__ConstP.pooled35,
      VehCtrlMdel241025_2018b__ConstP.pooled33,
      VehCtrlMdel241025_2018b__ConstP.pooled86, 11U);
  } else {
    /* Lookup_n-D: '<S29>/RWD_Table' */
    rtb_Switch2_mn = look2_iflf_binlx(Acc_POS_n, rtb_Switch2_mn,
      VehCtrlMdel241025_2018b__ConstP.pooled34,
      VehCtrlMdel241025_2018b__ConstP.pooled35,
      VehCtrlMdel241025_2018b__ConstP.pooled33,
      VehCtrlMdel241025_2018b__ConstP.pooled86, 11U);
  }

  /* End of Switch: '<S29>/Switch' */

  /* Gain: '<S10>/Gain' */
  rtb_Add4_j = 0.95F * rtb_Add10;

  /* MinMax: '<S10>/Min1' */
  rtb_VxIMU_est = fminf(rtb_Add4_j, rtb_Add6);

  /* Gain: '<S10>/Gain2' */
  rtb_Add10 = 0.95F * rtb_Add7;

  /* Product: '<S10>/Divide3' */
  rtb_Ax = rtb_VxIMU_est / rtb_Add10;

  /* UnitDelay: '<S44>/Delay Input2'
   *
   * Block description for '<S44>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_b0 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_l4;

  /* SampleTimeMath: '<S44>/sample time'
   *
   * About '<S44>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime_0 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S44>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant48'
   */
  rtb_Add7 = (real32_T)(4.0 * elapseTime_0);

  /* Abs: '<S10>/Abs5' */
  Acc_POS_n = fabsf(rtb_CastToBoolean);

  /* Lookup_n-D: '<S10>/2-D Lookup Table1' */
  Acc_POS_n = look2_iflf_binlx(Acc_POS_n, VehVxEst_mps,
    VehCtrlMdel241025_2018b__ConstP.uDLookupTable1_bp01Data,
    VehCtrlMdel241025_2018b__ConstP.uDLookupTable1_bp02Data,
    VehCtrlMdel241025_2018b__ConstP.uDLookupTable1_tableData,
    VehCtrlMdel241025_2018b__ConstP.uDLookupTable1_maxIndex, 5U);

  /* Sum: '<S44>/Difference Inputs1'
   *
   * Block description for '<S44>/Difference Inputs1':
   *
   *  Add in CPU
   */
  Acc_POS_n -= rtb_Switch2_b0;

  /* RelationalOperator: '<S61>/LowerRelop1' */
  rtb_LowerRelop1_b = (Acc_POS_n > rtb_Add7);

  /* Switch: '<S61>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S44>/delta fall limit' */
    rtb_Add7 = (real32_T)(-4.0 * elapseTime_0);

    /* RelationalOperator: '<S61>/UpperRelop' */
    rtb_LowerRelop1_b = (Acc_POS_n < rtb_Add7);

    /* Switch: '<S61>/Switch' */
    if (rtb_LowerRelop1_b) {
      Acc_POS_n = rtb_Add7;
    }

    /* End of Switch: '<S61>/Switch' */
    rtb_Add7 = Acc_POS_n;
  }

  /* End of Switch: '<S61>/Switch2' */

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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_l4 = rtb_Add7 +
    rtb_Switch2_b0;

  /* Sum: '<S10>/Add18' incorporates:
   *  UnitDelay: '<S44>/Delay Input2'
   *
   * Block description for '<S44>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Ax += VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_l4;

  /* Saturate: '<S10>/Saturation3' */
  if (rtb_Ax > 0.7F) {
    rtb_Ax = 0.7F;
  } else {
    if (rtb_Ax < 0.1F) {
      rtb_Ax = 0.1F;
    }
  }

  /* End of Saturate: '<S10>/Saturation3' */

  /* Gain: '<S10>/Gain4' */
  Acc_POS_n = 0.5F * rtb_Ax;

  /* Product: '<S10>/Product2' */
  Acc_POS_n *= rtb_Switch2_mn;

  /* Gain: '<S10>/Gain26' */
  rtb_MaxWhlSpd_mps_n = 0.8F * Acc_POS_n;

  /* UnitDelay: '<S45>/Delay Input2'
   *
   * Block description for '<S45>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_j3;

  /* SampleTimeMath: '<S45>/sample time'
   *
   * About '<S45>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime_0 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S45>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(4000.0 * elapseTime_0);

  /* Gain: '<S10>/Gain21' */
  WhlSpdFL = 0.1020408163265306 * rtb_Yk1_l;

  /* MATLAB Function: '<S10>/Wtarget' incorporates:
   *  Constant: '<S10>/Constant14'
   *  Constant: '<S10>/Constant15'
   *  Constant: '<S10>/Constant16'
   *  Constant: '<S10>/Constant17'
   *  Constant: '<S10>/Constant18'
   *  Constant: '<S10>/Constant19'
   */
  Wdes = VehVxEst_mps * rtb_CastToBoolean / (340.0F * VehVxEst_mps *
    VehVxEst_mps * 15.5799866F / 460.0F / 440.0F / 1.592F / 2.0F + 1.592F);
  if (Wdes < 0.0F) {
    y = -1.0F;
  } else if (Wdes > 0.0F) {
    y = 1.0F;
  } else if (Wdes == 0.0F) {
    y = 0.0F;
  } else {
    y = (rtNaNF);
  }

  Wdes = fminf(fabsf(5.88F / (real32_T)WhlSpdFL), fabsf(Wdes)) * 0.8F * y;

  /* End of MATLAB Function: '<S10>/Wtarget' */

  /* Sum: '<S45>/Difference Inputs1'
   *
   * Block description for '<S45>/Difference Inputs1':
   *
   *  Add in CPU
   */
  Wdes -= rtb_Add7;

  /* RelationalOperator: '<S62>/LowerRelop1' */
  rtb_LowerRelop1_b = (Wdes > rtb_Switch2_b0);

  /* Switch: '<S62>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S45>/delta fall limit' */
    rtb_Switch2_b0 = (real32_T)(-4000.0 * elapseTime_0);

    /* RelationalOperator: '<S62>/UpperRelop' */
    rtb_LowerRelop1_b = (Wdes < rtb_Switch2_b0);

    /* Switch: '<S62>/Switch' */
    if (rtb_LowerRelop1_b) {
      Wdes = rtb_Switch2_b0;
    }

    /* End of Switch: '<S62>/Switch' */
    rtb_Switch2_b0 = Wdes;
  }

  /* End of Switch: '<S62>/Switch2' */

  /* Sum: '<S45>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S45>/Delay Input2'
   *
   * Block description for '<S45>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S45>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_j3 = rtb_Switch2_b0 +
    rtb_Add7;

  /* Sum: '<S10>/Add' incorporates:
   *  UnitDelay: '<S45>/Delay Input2'
   *
   * Block description for '<S45>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_gd = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_j3 -
    rtb_UkYk1;

  /* UnitDelay: '<S10>/Unit Delay3' */
  rtb_LowerRelop1_b = VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_i;

  /* Abs: '<S10>/Abs' */
  rtb_Switch2_cn = fabs(rtb_Switch2_gd);

  /* RelationalOperator: '<S34>/Compare' incorporates:
   *  Constant: '<S34>/Constant'
   */
  rtb_AND2_e = (rtb_Switch2_cn > 4.0);

  /* Abs: '<S10>/Abs1' */
  rtb_Switch2_cn = fabs(rtb_UkYk1);

  /* RelationalOperator: '<S35>/Compare' incorporates:
   *  Constant: '<S35>/Constant'
   */
  TrqR_cmd_raw = (rtb_Switch2_cn > 1.0);

  /* RelationalOperator: '<S36>/Compare' incorporates:
   *  Constant: '<S36>/Constant'
   */
  rtb_AND_l = (VehVxEst_mps > 2.0F);

  /* Logic: '<S10>/AND' */
  rtb_AND2_e = (rtb_AND2_e && TrqR_cmd_raw && rtb_AND_l);

  /* Logic: '<S10>/Logical Operator4' */
  rtb_LowerRelop1_b = ((!rtb_LowerRelop1_b) && (!rtb_AND2_e));

  /* Abs: '<S10>/Abs2' */
  rtb_Switch2_cn = fabs(rtb_Switch2_gd);

  /* RelationalOperator: '<S37>/Compare' incorporates:
   *  Constant: '<S37>/Constant'
   */
  rtb_AND_l = (rtb_Switch2_cn < 3.0);

  /* RelationalOperator: '<S38>/Compare' incorporates:
   *  Constant: '<S38>/Constant'
   */
  TrqR_cmd_raw = (rtb_Yk1_l < -5.0);

  /* Logic: '<S10>/OR' */
  rtb_AND_l = (rtb_AND_l || TrqR_cmd_raw);

  /* Switch: '<S10>/Switch6' incorporates:
   *  Constant: '<S10>/Reset'
   */
  if (rtb_AND_l) {
    /* Sum: '<S10>/Add10' incorporates:
     *  Constant: '<S10>/Steptime'
     */
    rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_j + 0.01F;
  } else {
    rtb_Add7 = 0.0F;
  }

  /* End of Switch: '<S10>/Switch6' */

  /* MinMax: '<S10>/Min' incorporates:
   *  Constant: '<S10>/ResetDelay'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_j = fminf(rtb_Add7, 1.0F);

  /* RelationalOperator: '<S10>/Relational Operator9' incorporates:
   *  Constant: '<S10>/ResetDelay'
   *  UnitDelay: '<S10>/Unit Delay4'
   */
  rtb_AND_l = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_j >= 1.0F);

  /* Logic: '<S10>/Logical Operator5' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_i = ((!rtb_LowerRelop1_b) &&
    (!rtb_AND_l));

  /* Logic: '<S10>/AND3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   *  UnitDelay: '<S7>/Unit Delay'
   */
  rtb_AND_l = ((VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_n != 0.0) &&
               VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_i);

  /* Switch: '<S10>/Switch1' incorporates:
   *  Constant: '<S10>/Constant1'
   */
  if (!rtb_AND_l) {
    rtb_Switch2_gd = 0.0;
  }

  /* End of Switch: '<S10>/Switch1' */

  /* Product: '<S10>/Product3' */
  rtb_Switch2_cn = 2.0 * rtb_Switch2_gd;

  /* UnitDelay: '<S10>/Unit Delay2' */
  rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_j;

  /* Product: '<S10>/Product' */
  rtb_Add7 *= rtb_CastToBoolean;

  /* RelationalOperator: '<S33>/Compare' incorporates:
   *  Constant: '<S33>/Constant'
   */
  rtb_AND_l = (rtb_Add7 <= 0.0F);

  /* UnitDelay: '<S10>/Unit Delay' */
  rtb_Add5 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_h;

  /* Switch: '<S10>/Switch' incorporates:
   *  Constant: '<S10>/Constant'
   */
  if (rtb_AND_l) {
    rtb_Add5 = 0.0;
  }

  /* End of Switch: '<S10>/Switch' */

  /* Product: '<S10>/Product4' */
  WhlSpdFL = rtb_Switch2_gd;

  /* UnitDelay: '<S10>/Unit Delay1' */
  rtb_Switch2_gd = VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_c;

  /* Product: '<S10>/Product5' */
  WhlSpdFR = rtb_Switch2_gd;

  /* Sum: '<S10>/Add2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_h = (rtb_Add5 + WhlSpdFL) -
    WhlSpdFR;

  /* MATLAB Function: '<S10>/MATLAB Function1' incorporates:
   *  Constant: '<S10>/Constant20'
   *  Constant: '<S10>/Constant21'
   *  Constant: '<S10>/Constant22'
   *  Constant: '<S10>/Constant23'
   *  Constant: '<S10>/Constant24'
   *  Constant: '<S10>/Constant25'
   */
  rtb_StrWhlAngV_c = ((5.43088F / VehVxEst_mps * (-1.35294116F / VehVxEst_mps) -
                       (-0.0458234884F / VehVxEst_mps / VehVxEst_mps - 1.0F) *
                       -3.33390474F) * 105.0F * rtb_CastToBoolean - -1.35294116F
                      / VehVxEst_mps * 105.0F * rtb_StrWhlAngV_c) /
    (-0.0458234884F / VehVxEst_mps / VehVxEst_mps - 1.0F) / 100.0F;

  /* UnitDelay: '<S10>/Unit Delay6' */
  rtb_AND_l = VehCtrlMdel241025_2018b_amks_DW.UnitDelay6_DSTATE_b;

  /* Logic: '<S10>/Logical Operator3' */
  rtb_AND_l = !rtb_AND_l;

  /* Switch: '<S10>/Switch3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay5'
   */
  if (rtb_AND_l) {
    /* Switch: '<S10>/Switch4' incorporates:
     *  Constant: '<S10>/InitZORE'
     *  UnitDelay: '<S10>/Unit Delay3'
     */
    if (!VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_i) {
      rtb_StrWhlAngV_c = 0.0F;
    }

    /* End of Switch: '<S10>/Switch4' */
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay5_DSTATE_k = rtb_StrWhlAngV_c;
  }

  /* End of Switch: '<S10>/Switch3' */

  /* Sum: '<S10>/Add1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay'
   *  UnitDelay: '<S10>/Unit Delay5'
   */
  WhlSpdFR = (rtb_Switch2_cn +
              VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_h) +
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay5_DSTATE_k;

  /* Saturate: '<S10>/Saturation' */
  if (WhlSpdFR > 5000.0) {
    WhlSpdFL = 5000.0;
  } else if (WhlSpdFR < -5000.0) {
    WhlSpdFL = -5000.0;
  } else {
    WhlSpdFL = WhlSpdFR;
  }

  /* End of Saturate: '<S10>/Saturation' */

  /* Gain: '<S10>/Gain14' */
  rtb_Add7 = 0.0174532924F * FRWhlStrAng;

  /* Trigonometry: '<S10>/Cos2' */
  rtb_Add7 = cosf(rtb_Add7);

  /* Gain: '<S10>/Gain13' */
  rtb_Add7 *= 1.2F;

  /* Sum: '<S10>/Add8' incorporates:
   *  Constant: '<S10>/Constant28'
   */
  rtb_Switch2_b0 = 90.0F - FRWhlStrAng;

  /* Gain: '<S10>/Gain15' */
  rtb_Switch2_b0 *= 0.0174532924F;

  /* Trigonometry: '<S10>/Cos3' */
  rtb_Switch2_b0 = cosf(rtb_Switch2_b0);

  /* Gain: '<S10>/Gain12' */
  rtb_Switch2_b0 *= 1.522F;

  /* Sum: '<S10>/Add9' */
  rtb_Add7 += rtb_Switch2_b0;

  /* Product: '<S10>/Divide2' */
  rtb_Add5 = WhlSpdFL / rtb_Add7;

  /* Gain: '<S10>/Gain24' */
  rtb_Add5 *= 0.2;

  /* RelationalOperator: '<S55>/LowerRelop1' */
  rtb_AND_l = (rtb_Add5 > rtb_MaxWhlSpd_mps_n);

  /* Switch: '<S55>/Switch2' */
  if (rtb_AND_l) {
    rtb_Add5 = rtb_MaxWhlSpd_mps_n;
  } else {
    /* Gain: '<S10>/Gain28' */
    rtb_StrWhlAngV_c = -rtb_MaxWhlSpd_mps_n;

    /* RelationalOperator: '<S55>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_Add5 < rtb_StrWhlAngV_c);

    /* Switch: '<S55>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_Add5 = rtb_StrWhlAngV_c;
    }

    /* End of Switch: '<S55>/Switch' */
  }

  /* End of Switch: '<S55>/Switch2' */

  /* Sum: '<S10>/Add5' */
  rtb_Add5 = Acc_POS_n - rtb_Add5;

  /* RelationalOperator: '<S49>/LowerRelop1' */
  rtb_AND_l = (rtb_Add5 > rtb_Add6);

  /* Switch: '<S49>/Switch2' */
  if (rtb_AND_l) {
    rtb_Add5 = rtb_Add6;
  } else {
    /* RelationalOperator: '<S49>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    rtb_LowerRelop1_b = (rtb_Add5 < 0.0);

    /* Switch: '<S49>/Switch' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    if (rtb_LowerRelop1_b) {
      rtb_Add5 = 0.0;
    }

    /* End of Switch: '<S49>/Switch' */
  }

  /* End of Switch: '<S49>/Switch2' */

  /* UnitDelay: '<S47>/Delay Input2'
   *
   * Block description for '<S47>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_cn = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_lt;

  /* SampleTimeMath: '<S47>/sample time'
   *
   * About '<S47>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime_0 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S47>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant44'
   */
  rtb_Switch2_gd = 1000.0 * elapseTime_0;

  /* Gain: '<S10>/Gain8' */
  rtb_Add7 = 0.0174532924F * rtb_deltafalllimit_iz;

  /* Trigonometry: '<S10>/Cos' */
  rtb_Add7 = cosf(rtb_Add7);

  /* Gain: '<S10>/Gain11' */
  rtb_Add7 *= 1.2F;

  /* Sum: '<S10>/Add6' incorporates:
   *  Constant: '<S10>/Constant27'
   */
  rtb_Switch2_b0 = 90.0F - rtb_deltafalllimit_iz;

  /* Gain: '<S10>/Gain9' */
  rtb_Switch2_b0 *= 0.0174532924F;

  /* Trigonometry: '<S10>/Cos1' */
  rtb_Switch2_b0 = cosf(rtb_Switch2_b0);

  /* Gain: '<S10>/Gain10' */
  rtb_Switch2_b0 *= 1.522F;

  /* Sum: '<S10>/Add7' */
  rtb_Add7 += rtb_Switch2_b0;

  /* Product: '<S10>/Divide1' */
  rtb_UkYk1_ll = WhlSpdFL / rtb_Add7;

  /* Gain: '<S10>/Gain25' */
  rtb_UkYk1_ll *= 0.2;

  /* RelationalOperator: '<S54>/LowerRelop1' */
  rtb_AND_l = (rtb_UkYk1_ll > rtb_MaxWhlSpd_mps_n);

  /* Switch: '<S54>/Switch2' */
  if (rtb_AND_l) {
    rtb_UkYk1_ll = rtb_MaxWhlSpd_mps_n;
  } else {
    /* Gain: '<S10>/Gain27' */
    rtb_deltafalllimit_iz = -rtb_MaxWhlSpd_mps_n;

    /* RelationalOperator: '<S54>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_UkYk1_ll < rtb_deltafalllimit_iz);

    /* Switch: '<S54>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_UkYk1_ll = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S54>/Switch' */
  }

  /* End of Switch: '<S54>/Switch2' */

  /* Sum: '<S10>/Add4' */
  rtb_Add4_f = Acc_POS_n + rtb_UkYk1_ll;

  /* Sum: '<S10>/Add14' */
  rtb_g_mpss1 = rtb_VxIMU_est - rtb_Add4_f;

  /* RelationalOperator: '<S10>/Relational Operator' incorporates:
   *  Constant: '<S10>/Constant37'
   */
  rtb_Compare_i = (rtb_g_mpss1 < 0.0);

  /* Sum: '<S10>/Add17' incorporates:
   *  Constant: '<S10>/Constant47'
   */
  rtb_UkYk1_ll = 1.0 - rtb_Ax;

  /* Product: '<S10>/Product1' */
  rtb_Switch2_on = rtb_Switch2_mn * rtb_UkYk1_ll;

  /* Sum: '<S10>/Add15' */
  rtb_UkYk1_ll = rtb_Add10 - rtb_Switch2_on;

  /* RelationalOperator: '<S10>/Relational Operator1' incorporates:
   *  Constant: '<S10>/Constant38'
   */
  rtb_Compare_c = (rtb_UkYk1_ll < 0.0);

  /* Logic: '<S10>/AND2' */
  rtb_AND_l = (rtb_Compare_i && rtb_Compare_c);

  /* Logic: '<S10>/OR2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_AND_l = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_i || rtb_AND_l);

  /* Switch: '<S10>/Switch7' incorporates:
   *  Constant: '<S10>/Constant39'
   */
  if (rtb_AND_l) {
    rtb_UkYk1_ll = 0.0;
  } else {
    /* Logic: '<S10>/NOT' */
    rtb_LowerRelop1_b = !rtb_Compare_i;

    /* Switch: '<S10>/Switch8' incorporates:
     *  Constant: '<S10>/Constant40'
     */
    if (!rtb_LowerRelop1_b) {
      rtb_UkYk1_ll = 0.0;
    }

    /* End of Switch: '<S10>/Switch8' */
  }

  /* End of Switch: '<S10>/Switch7' */

  /* Gain: '<S10>/Gain20' */
  rtb_UkYk1_ll = -rtb_UkYk1_ll;

  /* Saturate: '<S10>/Saturation2' */
  if (rtb_UkYk1_ll > 100.0) {
    rtb_UkYk1_ll = 100.0;
  } else {
    if (rtb_UkYk1_ll < 0.0) {
      rtb_UkYk1_ll = 0.0;
    }
  }

  /* End of Saturate: '<S10>/Saturation2' */

  /* Sum: '<S47>/Difference Inputs1'
   *
   * Block description for '<S47>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1_ll -= rtb_Switch2_cn;

  /* RelationalOperator: '<S64>/LowerRelop1' */
  rtb_AND_l = (rtb_UkYk1_ll > rtb_Switch2_gd);

  /* Switch: '<S64>/Switch2' */
  if (!rtb_AND_l) {
    /* Product: '<S47>/delta fall limit' */
    elapseTime_0 *= -1000.0;

    /* RelationalOperator: '<S64>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_UkYk1_ll < elapseTime_0);

    /* Switch: '<S64>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_UkYk1_ll = elapseTime_0;
    }

    /* End of Switch: '<S64>/Switch' */
    rtb_Switch2_gd = rtb_UkYk1_ll;
  }

  /* End of Switch: '<S64>/Switch2' */

  /* Sum: '<S47>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S47>/Delay Input2'
   *
   * Block description for '<S47>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S47>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_lt = rtb_Switch2_gd +
    rtb_Switch2_cn;

  /* Sum: '<S10>/Add12' incorporates:
   *  UnitDelay: '<S47>/Delay Input2'
   *
   * Block description for '<S47>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add5 += VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_lt;

  /* RelationalOperator: '<S52>/LowerRelop1' */
  rtb_AND_l = (rtb_Add5 > rtb_Add6);

  /* Switch: '<S52>/Switch2' */
  if (rtb_AND_l) {
    rtb_Add5 = rtb_Add6;
  } else {
    /* RelationalOperator: '<S52>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    rtb_LowerRelop1_b = (rtb_Add5 < 0.0);

    /* Switch: '<S52>/Switch' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    if (rtb_LowerRelop1_b) {
      rtb_Add5 = 0.0;
    }

    /* End of Switch: '<S52>/Switch' */
  }

  /* End of Switch: '<S52>/Switch2' */

  /* Sum: '<S42>/Difference Inputs1'
   *
   * Block description for '<S42>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add5 -= rtb_Gain4;

  /* RelationalOperator: '<S59>/LowerRelop1' */
  rtb_AND_l = (rtb_Add5 > rtb_Yk1);

  /* Switch: '<S59>/Switch2' */
  if (!rtb_AND_l) {
    /* Product: '<S42>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S59>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_Add5 < elapseTime);

    /* Switch: '<S59>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_Add5 = elapseTime;
    }

    /* End of Switch: '<S59>/Switch' */
    rtb_Yk1 = rtb_Add5;
  }

  /* End of Switch: '<S59>/Switch2' */

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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_p = rtb_Yk1 + rtb_Gain4;

  /* Switch: '<S7>/Switch8' incorporates:
   *  UnitDelay: '<S42>/Delay Input2'
   *  UnitDelay: '<S7>/Unit Delay'
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_n != 0.0) {
    elapseTime = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_p;
  } else {
    /* Product: '<S7>/Product' incorporates:
     *  Constant: '<S7>/Constant17'
     */
    elapseTime = rtb_Switch2_mn * 0.099999999999999978;
  }

  /* End of Switch: '<S7>/Switch8' */

  /* RelationalOperator: '<S26>/LowerRelop1' */
  rtb_AND_l = (elapseTime > rtb_Gain5);

  /* Switch: '<S26>/Switch2' */
  if (rtb_AND_l) {
    rtb_MaxWhlSpd_mps_n = (real32_T)rtb_Gain5;
  } else {
    /* RelationalOperator: '<S26>/UpperRelop' incorporates:
     *  Constant: '<S7>/Constant15'
     */
    rtb_LowerRelop1_b = (elapseTime < 0.0);

    /* Switch: '<S26>/Switch' incorporates:
     *  Constant: '<S7>/Constant15'
     */
    if (rtb_LowerRelop1_b) {
      rtb_MaxWhlSpd_mps_n = 0.0F;
    } else {
      rtb_MaxWhlSpd_mps_n = (real32_T)elapseTime;
    }

    /* End of Switch: '<S26>/Switch' */
  }

  /* End of Switch: '<S26>/Switch2' */

  /* UnitDelay: '<S86>/Unit Delay1' */
  rtb_AND_l = VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_gl;

  /* Saturate: '<S31>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    Acc_POS_n = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    Acc_POS_n = 0.0F;
  } else {
    Acc_POS_n = VehVxEst_mps;
  }

  /* Lookup_n-D: '<S31>/VehSpd_SlipTarget_mps' */
  rtb_Add7 = look1_iflf_binlc(Acc_POS_n,
    VehCtrlMdel241025_2018b__ConstP.pooled63,
    VehCtrlMdel241025_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S31>/Add9' */
  rtb_Add7 += Acc_POS_n;

  /* Saturate: '<S31>/Saturation1' */
  if (rtb_Acc_POS > 50.0F) {
    rtb_Switch2_b0 = 50.0F;
  } else if (rtb_Acc_POS < 0.0F) {
    rtb_Switch2_b0 = 0.0F;
  } else {
    rtb_Switch2_b0 = rtb_Acc_POS;
  }

  /* End of Saturate: '<S31>/Saturation1' */

  /* Sum: '<S31>/Add1' */
  rtb_deltafalllimit_iz = rtb_Add7 - rtb_Switch2_b0;

  /* RelationalOperator: '<S31>/Relational Operator7' incorporates:
   *  Constant: '<S31>/Cal_DeltaV_mps'
   */
  TrqR_cmd_raw = (rtb_deltafalllimit_iz < 0.0F);

  /* Logic: '<S86>/Logical Operator4' */
  rtb_AND_l = ((!rtb_AND_l) && (!TrqR_cmd_raw));

  /* Logic: '<S31>/Logical Operator2' */
  TrqR_cmd_raw = !TrqR_cmd_raw;

  /* UnitDelay: '<S31>/Unit Delay4' */
  rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_i;

  /* RelationalOperator: '<S31>/Relational Operator8' incorporates:
   *  Constant: '<S31>/Cal_DeltaV_mps1'
   */
  rtb_AND2_e = (rtb_Add7 > 235.0F);

  /* Logic: '<S31>/Logical Operator1' */
  TrqR_cmd_raw = (TrqR_cmd_raw && rtb_AND2_e);

  /* Switch: '<S87>/Switch6' incorporates:
   *  Constant: '<S87>/Reset'
   */
  if (TrqR_cmd_raw) {
    /* Sum: '<S87>/Add10' incorporates:
     *  Constant: '<S87>/Steptime'
     */
    rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_n5 + 0.01F;
  } else {
    rtb_Add7 = 0.0F;
  }

  /* End of Switch: '<S87>/Switch6' */

  /* MinMax: '<S87>/Min' incorporates:
   *  Constant: '<S31>/ResetDelay'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_n5 = fminf(rtb_Add7, 0.1F);

  /* RelationalOperator: '<S87>/Relational Operator9' incorporates:
   *  Constant: '<S31>/ResetDelay'
   *  UnitDelay: '<S87>/Unit Delay1'
   */
  TrqR_cmd_raw = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_n5 >= 0.1F);

  /* UnitDelay: '<S31>/Unit Delay3' */
  rtb_AND2_e = VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_e;

  /* Logic: '<S31>/Logical Operator3' */
  TrqR_cmd_raw = (TrqR_cmd_raw || rtb_AND2_e);

  /* Logic: '<S86>/Logical Operator5' incorporates:
   *  UnitDelay: '<S86>/Unit Delay1'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_gl = ((!rtb_AND_l) &&
    (!TrqR_cmd_raw));

  /* RelationalOperator: '<S91>/Compare' incorporates:
   *  UnitDelay: '<S86>/Unit Delay1'
   */
  rtb_LowerRelop1_b = VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_gl;

  /* UnitDelay: '<S85>/Delay Input1'
   *
   * Block description for '<S85>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_AND_l = VehCtrlMdel241025_2018b_amks_DW.DelayInput1_DSTATE_j;

  /* RelationalOperator: '<S85>/FixPt Relational Operator' */
  rtb_AND_l = ((int32_T)rtb_LowerRelop1_b > (int32_T)rtb_AND_l);

  /* Switch: '<S31>/Switch' incorporates:
   *  Constant: '<S31>/Integr_StartPoint'
   */
  if (rtb_AND_l) {
    /* Sum: '<S31>/Add4' */
    rtb_Add7 = rtb_MaxWhlSpd_mps_n -
      VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_f;
  } else {
    rtb_Add7 = 0.0F;
  }

  /* End of Switch: '<S31>/Switch' */

  /* Switch: '<S31>/Switch6' incorporates:
   *  Constant: '<S31>/Verror_Reset'
   *  UnitDelay: '<S86>/Unit Delay1'
   */
  if (VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_gl) {
    rtb_Ax = rtb_deltafalllimit_iz;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S31>/Switch6' */

  /* UnitDelay: '<S31>/Unit Delay5' */
  rtb_VxIMU_est = VehCtrlMdel241025_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* Product: '<S31>/Product2' */
  rtb_VxIMU_est *= rtb_deltafalllimit_iz;

  /* RelationalOperator: '<S83>/Compare' incorporates:
   *  Constant: '<S83>/Constant'
   */
  rtb_AND_l = (rtb_VxIMU_est <= 0.0F);

  /* UnitDelay: '<S31>/Unit Delay' */
  rtb_VxIMU_est = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_f;

  /* Switch: '<S31>/Switch3' incorporates:
   *  Constant: '<S31>/Verror_Reset1'
   */
  if (rtb_AND_l) {
    rtb_VxIMU_est = 0.0F;
  }

  /* End of Switch: '<S31>/Switch3' */

  /* Sum: '<S31>/Add2' */
  rtb_VxIMU_est += rtb_Ax;

  /* Saturate: '<S31>/Saturation2' */
  if (rtb_VxIMU_est > 400.0F) {
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_f = 400.0F;
  } else if (rtb_VxIMU_est < -100.0F) {
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_f = -100.0F;
  } else {
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_f = rtb_VxIMU_est;
  }

  /* End of Saturate: '<S31>/Saturation2' */

  /* Lookup_n-D: '<S31>/VehicleStableTarget_mps' */
  rtb_VxIMU_est = look1_iflf_binlc(Acc_POS_n,
    VehCtrlMdel241025_2018b__ConstP.pooled63,
    VehCtrlMdel241025_2018b__ConstP.pooled69, 3U);

  /* Sum: '<S31>/Add5' */
  rtb_VxIMU_est += Acc_POS_n;

  /* Sum: '<S31>/Add10' */
  rtb_VxIMU_est = rtb_Switch2_b0 - rtb_VxIMU_est;

  /* RelationalOperator: '<S31>/Relational Operator' incorporates:
   *  Constant: '<S31>/Verror'
   */
  rtb_AND_l = (rtb_VxIMU_est < 0.0F);

  /* Logic: '<S31>/Logical Operator4' incorporates:
   *  UnitDelay: '<S86>/Unit Delay1'
   */
  rtb_AND_l = (rtb_AND_l && VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_gl);

  /* Switch: '<S31>/Switch1' incorporates:
   *  Constant: '<S31>/Trq_IReset'
   *  Constant: '<S31>/Trq_I_FF'
   */
  if (rtb_AND_l) {
    rtb_VxIMU_est = 20.0F;
  } else {
    rtb_VxIMU_est = 0.0F;
  }

  /* End of Switch: '<S31>/Switch1' */

  /* Sum: '<S31>/Add6' incorporates:
   *  UnitDelay: '<S31>/Unit Delay'
   */
  rtb_Add7 = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_f + rtb_Add7) +
    rtb_VxIMU_est;

  /* Product: '<S31>/Product1' incorporates:
   *  Constant: '<S31>/I_Gain'
   */
  rtb_StrWhlAngV_c = rtb_Add7 * 10.0F;

  /* Product: '<S31>/Product' incorporates:
   *  Constant: '<S31>/P_Gain'
   *  UnitDelay: '<S31>/Unit Delay1'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_f = rtb_Ax * 40.0F;

  /* Sum: '<S31>/Add11' incorporates:
   *  UnitDelay: '<S31>/Unit Delay1'
   */
  rtb_Add7 = rtb_MaxWhlSpd_mps_n -
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_f;

  /* RelationalOperator: '<S88>/LowerRelop1' */
  rtb_AND_l = (rtb_StrWhlAngV_c > rtb_Add7);

  /* Switch: '<S88>/Switch2' */
  if (!rtb_AND_l) {
    /* Gain: '<S31>/Gain3' incorporates:
     *  UnitDelay: '<S31>/Unit Delay1'
     */
    FRWhlStrAng = -VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_f;

    /* RelationalOperator: '<S88>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_StrWhlAngV_c < FRWhlStrAng);

    /* Switch: '<S88>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_StrWhlAngV_c = FRWhlStrAng;
    }

    /* End of Switch: '<S88>/Switch' */
    rtb_Add7 = rtb_StrWhlAngV_c;
  }

  /* End of Switch: '<S88>/Switch2' */

  /* Sum: '<S31>/Add7' incorporates:
   *  UnitDelay: '<S31>/Unit Delay1'
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_i =
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_f + rtb_Add7;

  /* Lookup_n-D: '<S31>/VehicleStableTarget_mps1' */
  rtb_Add7 = look1_iflf_binlc(Acc_POS_n,
    VehCtrlMdel241025_2018b__ConstP.pooled63,
    VehCtrlMdel241025_2018b__ConstP.pooled69, 3U);

  /* Sum: '<S31>/Add13' */
  Acc_POS_n += rtb_Add7;

  /* Sum: '<S31>/Add12' */
  rtb_Switch2_b0 -= Acc_POS_n;

  /* RelationalOperator: '<S31>/Relational Operator1' incorporates:
   *  Constant: '<S31>/Verror1'
   */
  rtb_AND_l = (rtb_Switch2_b0 < 0.0F);

  /* RelationalOperator: '<S31>/Relational Operator2' incorporates:
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  TrqR_cmd_raw = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_i >=
                  rtb_MaxWhlSpd_mps_n);

  /* RelationalOperator: '<S84>/Compare' incorporates:
   *  Constant: '<S84>/Constant'
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  rtb_AND2_e = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_i <= 5.0F);

  /* Logic: '<S31>/OR' */
  TrqR_cmd_raw = (TrqR_cmd_raw || rtb_AND2_e);

  /* Logic: '<S31>/Logical Operator5' incorporates:
   *  UnitDelay: '<S31>/Unit Delay3'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_e = (rtb_AND_l &&
    TrqR_cmd_raw);

  /* Switch: '<S31>/Switch2' incorporates:
   *  Switch: '<S31>/Switch7'
   *  UnitDelay: '<S31>/Unit Delay3'
   *  UnitDelay: '<S86>/Unit Delay1'
   */
  if (VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_gl) {
    /* RelationalOperator: '<S89>/LowerRelop1' incorporates:
     *  Constant: '<S31>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S31>/Unit Delay4'
     */
    rtb_LogicalOperator2 = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_i >
      235.0F);

    /* Switch: '<S89>/Switch2' incorporates:
     *  Constant: '<S31>/TCS_TrqRequest_Max2'
     */
    if (rtb_LogicalOperator2) {
      VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_jr = 235.0F;
    } else {
      /* RelationalOperator: '<S89>/UpperRelop' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S31>/Unit Delay4'
       */
      rtb_LogicalOperator2 =
        (VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_i < 0.0F);

      /* Switch: '<S89>/Switch' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S31>/Unit Delay4'
       */
      if (rtb_LogicalOperator2) {
        VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_jr = 0.0F;
      } else {
        VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_jr =
          VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_i;
      }

      /* End of Switch: '<S89>/Switch' */
    }

    /* End of Switch: '<S89>/Switch2' */

    /* RelationalOperator: '<S90>/LowerRelop1' */
    rtb_LogicalOperator2 = (rtb_MaxWhlSpd_mps_n >
      VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_jr);

    /* Switch: '<S90>/Switch2' */
    if (!rtb_LogicalOperator2) {
      /* RelationalOperator: '<S90>/UpperRelop' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min1'
       */
      rtb_LogicalOperator2 = (rtb_MaxWhlSpd_mps_n < 0.0F);

      /* Switch: '<S90>/Switch' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min1'
       */
      if (rtb_LogicalOperator2) {
        VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_jr = 0.0F;
      } else {
        VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_jr =
          rtb_MaxWhlSpd_mps_n;
      }

      /* End of Switch: '<S90>/Switch' */
    }

    /* End of Switch: '<S90>/Switch2' */
  } else {
    if (VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_e) {
      /* Switch: '<S31>/Switch7' */
      VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_jr = rtb_MaxWhlSpd_mps_n;
    }
  }

  /* End of Switch: '<S31>/Switch2' */

  /* UnitDelay: '<S97>/Unit Delay1' */
  rtb_AND_l = VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_e;

  /* Saturate: '<S32>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    Acc_POS_n = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    Acc_POS_n = 0.0F;
  } else {
    Acc_POS_n = VehVxEst_mps;
  }

  /* End of Saturate: '<S32>/Saturation' */

  /* Lookup_n-D: '<S32>/VehSpd_SlipTarget_mps' */
  rtb_Add7 = look1_iflf_binlc(Acc_POS_n,
    VehCtrlMdel241025_2018b__ConstP.pooled63,
    VehCtrlMdel241025_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S32>/Add9' */
  rtb_Add7 += Acc_POS_n;

  /* Sum: '<S7>/Add1' */
  rtb_Switch2_b0 = rtb_deltafalllimit_n + rtb_deltafalllimit_om;

  /* Gain: '<S7>/Gain2' */
  rtb_Switch2_b0 *= 0.5F;

  /* Saturate: '<S32>/Saturation1' */
  if (rtb_Switch2_b0 < 0.0F) {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Saturate: '<S32>/Saturation1' */

  /* Sum: '<S32>/Add1' */
  rtb_StrWhlAngV_c = rtb_Add7 - rtb_Switch2_b0;

  /* RelationalOperator: '<S32>/Relational Operator7' incorporates:
   *  Constant: '<S32>/Cal_DeltaV_mps'
   */
  TrqR_cmd_raw = (rtb_StrWhlAngV_c < 0.0F);

  /* Logic: '<S97>/Logical Operator4' */
  rtb_AND_l = ((!rtb_AND_l) && (!TrqR_cmd_raw));

  /* Logic: '<S32>/Logical Operator2' */
  TrqR_cmd_raw = !TrqR_cmd_raw;

  /* UnitDelay: '<S32>/Unit Delay4' */
  rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_b;

  /* RelationalOperator: '<S32>/Relational Operator8' incorporates:
   *  Constant: '<S32>/Cal_DeltaV_mps1'
   */
  rtb_AND2_e = (rtb_Add7 > 235.0F);

  /* Logic: '<S32>/Logical Operator1' */
  TrqR_cmd_raw = (TrqR_cmd_raw && rtb_AND2_e);

  /* Switch: '<S98>/Switch6' incorporates:
   *  Constant: '<S98>/Reset'
   */
  if (TrqR_cmd_raw) {
    /* Sum: '<S98>/Add10' incorporates:
     *  Constant: '<S98>/Steptime'
     */
    rtb_Add7 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_i + 0.01F;
  } else {
    rtb_Add7 = 0.0F;
  }

  /* End of Switch: '<S98>/Switch6' */

  /* MinMax: '<S98>/Min' incorporates:
   *  Constant: '<S32>/ResetDelay'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_i = fminf(rtb_Add7, 0.1F);

  /* RelationalOperator: '<S98>/Relational Operator9' incorporates:
   *  Constant: '<S32>/ResetDelay'
   *  UnitDelay: '<S98>/Unit Delay1'
   */
  TrqR_cmd_raw = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_i >= 0.1F);

  /* UnitDelay: '<S32>/Unit Delay3' */
  rtb_AND2_e = VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_a;

  /* Logic: '<S32>/Logical Operator3' */
  TrqR_cmd_raw = (TrqR_cmd_raw || rtb_AND2_e);

  /* Logic: '<S97>/Logical Operator5' incorporates:
   *  UnitDelay: '<S97>/Unit Delay1'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_e = ((!rtb_AND_l) &&
    (!TrqR_cmd_raw));

  /* RelationalOperator: '<S95>/Compare' incorporates:
   *  Constant: '<S95>/Constant'
   */
  rtb_AND_l = (rtb_Switch2_b0 > 0.0F);

  /* Abs: '<S32>/Abs' */
  rtb_Add7 = fabsf(rtb_CastToBoolean);

  /* RelationalOperator: '<S93>/Compare' incorporates:
   *  Constant: '<S93>/Constant'
   */
  TrqR_cmd_raw = (rtb_Add7 <= 20.0F);

  /* Logic: '<S32>/Logical Operator6' */
  rtb_AND_l = (rtb_AND_l && TrqR_cmd_raw);

  /* Logic: '<S32>/Logical Operator7' incorporates:
   *  UnitDelay: '<S97>/Unit Delay1'
   */
  rtb_LogicalOperator2 = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_e &&
    rtb_AND_l);

  /* UnitDelay: '<S77>/Unit Delay1' */
  rtb_AND_l = VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_dp;

  /* Saturate: '<S30>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    rtb_Add7 = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    rtb_Add7 = 0.0F;
  } else {
    rtb_Add7 = VehVxEst_mps;
  }

  /* End of Saturate: '<S30>/Saturation' */

  /* Lookup_n-D: '<S30>/VehSpd_SlipTarget_mps' */
  rtb_Ax = look1_iflf_binlc(rtb_Add7, VehCtrlMdel241025_2018b__ConstP.pooled63,
    VehCtrlMdel241025_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S30>/Add9' */
  rtb_Ax += rtb_Add7;

  /* Saturate: '<S30>/Saturation1' */
  if (rtb_Gain3_o < 0.0F) {
    rtb_VxIMU_est = 0.0F;
  } else {
    rtb_VxIMU_est = rtb_Gain3_o;
  }

  /* End of Saturate: '<S30>/Saturation1' */

  /* Sum: '<S30>/Add1' */
  rtb_Gain3_o = rtb_Ax - rtb_VxIMU_est;

  /* RelationalOperator: '<S30>/Relational Operator7' incorporates:
   *  Constant: '<S30>/Cal_DeltaV_mps'
   */
  TrqR_cmd_raw = (rtb_Gain3_o < 0.0F);

  /* Logic: '<S77>/Logical Operator4' */
  rtb_AND_l = ((!rtb_AND_l) && (!TrqR_cmd_raw));

  /* Logic: '<S30>/Logical Operator2' */
  TrqR_cmd_raw = !TrqR_cmd_raw;

  /* UnitDelay: '<S30>/Unit Delay4' */
  rtb_Ax = VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_l;

  /* RelationalOperator: '<S30>/Relational Operator8' incorporates:
   *  Constant: '<S30>/Cal_DeltaV_mps1'
   */
  rtb_AND2_e = (rtb_Ax > 235.0F);

  /* Logic: '<S30>/Logical Operator1' */
  TrqR_cmd_raw = (TrqR_cmd_raw && rtb_AND2_e);

  /* Switch: '<S78>/Switch6' incorporates:
   *  Constant: '<S78>/Reset'
   */
  if (TrqR_cmd_raw) {
    /* Sum: '<S78>/Add10' incorporates:
     *  Constant: '<S78>/Steptime'
     */
    rtb_Ax = VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_h + 0.01F;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S78>/Switch6' */

  /* MinMax: '<S78>/Min' incorporates:
   *  Constant: '<S30>/ResetDelay'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_h = fminf(rtb_Ax, 0.1F);

  /* RelationalOperator: '<S78>/Relational Operator9' incorporates:
   *  Constant: '<S30>/ResetDelay'
   *  UnitDelay: '<S78>/Unit Delay1'
   */
  TrqR_cmd_raw = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_h >= 0.1F);

  /* UnitDelay: '<S30>/Unit Delay3' */
  rtb_AND2_e = VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_ip;

  /* Logic: '<S30>/Logical Operator3' */
  TrqR_cmd_raw = (TrqR_cmd_raw || rtb_AND2_e);

  /* Logic: '<S77>/Logical Operator5' incorporates:
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_dp = ((!rtb_AND_l) &&
    (!TrqR_cmd_raw));

  /* Chart: '<S7>/Chart' */
  if (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c7_VehCtrlMdel241025_
      == 0U) {
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_active_c7_VehCtrlMdel241025_ =
      1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_B = 3U;
    VehCtrlMdel241025_2018b_amks_DW.b = rtb_Yk1_l * rtb_Yk1_l +
      rtb_deltafalllimit_le * rtb_deltafalllimit_le;
    VehCtrlMdel241025_2018b_amks_DW.b = sqrt(VehCtrlMdel241025_2018b_amks_DW.b);
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_C = 3U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_D = 1U;
    VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_E = 1U;
  } else {
    switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_B) {
     case VehCtrlMdel241_IN_DYC_Disenable:
      VehCtrlMdel241025_2018b_amksp_B.DYC_Enable_OUT = 0.0;
      break;

     case VehCtrlMdel241025_IN_DYC_Enable:
      VehCtrlMdel241025_2018b_amksp_B.DYC_Enable_OUT = 1.0;
      rtb_LogicalOperator3 = ((rtb_UkYk1 >= 50.0) || (rtb_deltafalllimit_le >=
        5.0) || (VehCtrlMdel241025_2018b_amks_DW.b >= 5.0));
      if (rtb_LogicalOperator3) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_B = 3U;
      }
      break;

     default:
      /* case IN_InitState: */
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_B = 1U;
      VehCtrlMdel241025_2018b_amksp_B.DYC_Enable_OUT = 0.0;
      VehCtrlMdel241025_2018b_amks_DW.DYC_flag = 0.0;
      break;
    }

    if (fabs(rtb_deltafalllimit_le) > 0.5) {
      elapseTime = atan(rtb_Yk1_l / rtb_deltafalllimit_le) * 180.0 /
        3.1415926535897931;
    } else {
      elapseTime = 0.0;
    }

    VehCtrlMdel241025_2018b_amks_DW.b = rtb_Yk1_l * rtb_Yk1_l +
      rtb_deltafalllimit_le * rtb_deltafalllimit_le;
    VehCtrlMdel241025_2018b_amks_DW.b = sqrt(VehCtrlMdel241025_2018b_amks_DW.b);
    switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_C) {
     case VehCtrlMdel241025_2018b_am_IN_B:
      rtb_LogicalOperator3 = ((!(VehCtrlMdel241025_2018b_amks_DW.DYC_flag != 0.0))
        || (rtb_UkYk1 > 30.0) || (elapseTime > 30.0) || (elapseTime > -30.0) ||
        (VehCtrlMdel241025_2018b_amks_DW.b > 3.0));
      if (rtb_LogicalOperator3) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_C = 3U;
      }
      break;

     case VehCtrlMdel241025_2018b_am_IN_C:
      break;

     default:
      /* case IN_F_TVD_TCS_STATE: */
      rtb_LogicalOperator3 = ((!(VehCtrlMdel241025_2018b_amks_DW.DYC_flag != 0.0))
        || (rtb_UkYk1 > 30.0) || (elapseTime > 30.0) || (elapseTime > -30.0) ||
        (VehCtrlMdel241025_2018b_amks_DW.b > 3.0));
      if (rtb_LogicalOperator3) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_C = 2U;
        VehCtrlMdel241025_2018b_amksp_B.TCSR_Enable_OUT = 0.0;
      }
      break;
    }

    switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_D) {
     case VehCtrlMdel241025_IN_InitState2:
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_D = 2U;
      VehCtrlMdel241025_2018b_amksp_B.TCSR_Enable_OUT = 0.0;
      break;

     case VehCtrlMdel24_IN_TCSR_Disenable:
      break;

     default:
      /* case IN_TCSR_Enable: */
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_D = 2U;
      VehCtrlMdel241025_2018b_amksp_B.TCSR_Enable_OUT = 0.0;
      break;
    }

    switch (VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_E) {
     case VehCtrlMdel241025_IN_InitState1:
      VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_E = 2U;
      VehCtrlMdel241025_2018b_amksp_B.TCSF_Enable_OUT = 0.0;
      break;

     case VehCtrlMdel24_IN_TCSF_Disenable:
      VehCtrlMdel241025_2018b_amksp_B.TCSF_Enable_OUT = 0.0;
      break;

     default:
      /* case IN_TCSF_Enable: */
      VehCtrlMdel241025_2018b_amksp_B.TCSF_Enable_OUT = 1.0;
      if (VehCtrlMdel241025_2018b_amks_DW.b > 5.0) {
        VehCtrlMdel241025_2018b_amks_DW.bitsForTID3.is_E = 1U;
      }
      break;
    }
  }

  /* End of Chart: '<S7>/Chart' */

  /* Switch: '<S7>/Switch6' incorporates:
   *  UnitDelay: '<S31>/Unit Delay2'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.TCSF_Enable_OUT != 0.0) {
    rtb_MaxWhlSpd_mps_n = VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_jr;
  }

  /* End of Switch: '<S7>/Switch6' */

  /* Lookup_n-D: '<S7>/AMK2' */
  rtb_UkYk1_ll = look1_binlx(MCFR_ActualVelocity,
    VehCtrlMdel241025_2018b__ConstP.pooled8,
    VehCtrlMdel241025_2018b__ConstP.pooled17, 19U);

  /* Gain: '<S7>/Gain8' */
  rtb_UkYk1 = 0.95 * rtb_UkYk1_ll;

  /* Switch: '<S28>/Switch' incorporates:
   *  Constant: '<S28>/Constant11'
   */
  if (rtb_LogicalOperator7_m) {
    rtb_UkYk1_ll = 2.0;
  } else {
    /* Sum: '<S28>/Add3' incorporates:
     *  Constant: '<S28>/RPM_min4'
     */
    rtb_deltafalllimit_le = MCFL_ActualVelocity + 10.0;

    /* MinMax: '<S28>/Max2' incorporates:
     *  Constant: '<S28>/RPM_min5'
     */
    rtb_Yk1_l = fmax(rtb_deltafalllimit_le, 1.0);

    /* Switch: '<S28>/Switch3' incorporates:
     *  Constant: '<S28>/Constant14'
     *  Constant: '<S28>/Constant7'
     */
    if (VehCtrlMdel241025_2018b_amksp_B.ModeSW_o != 0.0) {
      rtb_Switch4_o = 6;
    } else {
      rtb_Switch4_o = 11;
    }

    /* End of Switch: '<S28>/Switch3' */

    /* Product: '<S28>/Product3' */
    rtb_deltafalllimit_le = (real_T)rtb_Switch4_o * WhlSpdRL_mps;

    /* Product: '<S28>/Product2' */
    rtb_deltafalllimit_le *= 9550.0;

    /* Product: '<S28>/Divide2' */
    rtb_UkYk1_ll = rtb_deltafalllimit_le / rtb_Yk1_l;
  }

  /* End of Switch: '<S28>/Switch' */

  /* UnitDelay: '<S41>/Delay Input2'
   *
   * Block description for '<S41>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add5 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hj;

  /* SampleTimeMath: '<S41>/sample time'
   *
   * About '<S41>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S41>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  rtb_Switch2_cn = 2000.0 * elapseTime;

  /* RelationalOperator: '<S48>/LowerRelop1' */
  rtb_AND_l = (rtb_Add4_f > rtb_Add4_j);

  /* Switch: '<S48>/Switch2' */
  if (rtb_AND_l) {
    rtb_Switch2_gd = rtb_Add4_j;
  } else {
    /* RelationalOperator: '<S48>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    rtb_LogicalOperator3 = (rtb_Add4_f < 0.0);

    /* Switch: '<S48>/Switch' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    if (rtb_LogicalOperator3) {
      rtb_Add4_f = 0.0;
    }

    /* End of Switch: '<S48>/Switch' */
    rtb_Switch2_gd = rtb_Add4_f;
  }

  /* End of Switch: '<S48>/Switch2' */

  /* Sum: '<S10>/Add11' incorporates:
   *  UnitDelay: '<S47>/Delay Input2'
   *
   * Block description for '<S47>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = rtb_Switch2_gd +
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_lt;

  /* RelationalOperator: '<S51>/LowerRelop1' */
  rtb_AND_l = (rtb_Yk1_l > rtb_Add4_j);

  /* Switch: '<S51>/Switch2' */
  if (rtb_AND_l) {
    rtb_Yk1_l = rtb_Add4_j;
  } else {
    /* RelationalOperator: '<S51>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    rtb_LogicalOperator3 = (rtb_Yk1_l < 0.0);

    /* Switch: '<S51>/Switch' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    if (rtb_LogicalOperator3) {
      rtb_Yk1_l = 0.0;
    }

    /* End of Switch: '<S51>/Switch' */
  }

  /* End of Switch: '<S51>/Switch2' */

  /* Sum: '<S41>/Difference Inputs1'
   *
   * Block description for '<S41>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Yk1_l -= rtb_Add5;

  /* RelationalOperator: '<S58>/LowerRelop1' */
  rtb_AND_l = (rtb_Yk1_l > rtb_Switch2_cn);

  /* Switch: '<S58>/Switch2' */
  if (!rtb_AND_l) {
    /* Product: '<S41>/delta fall limit' */
    rtb_deltafalllimit_le = -2000.0 * elapseTime;

    /* RelationalOperator: '<S58>/UpperRelop' */
    rtb_LogicalOperator3 = (rtb_Yk1_l < rtb_deltafalllimit_le);

    /* Switch: '<S58>/Switch' */
    if (rtb_LogicalOperator3) {
      rtb_Yk1_l = rtb_deltafalllimit_le;
    }

    /* End of Switch: '<S58>/Switch' */
    rtb_Switch2_cn = rtb_Yk1_l;
  }

  /* End of Switch: '<S58>/Switch2' */

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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hj = rtb_Switch2_cn +
    rtb_Add5;

  /* Switch: '<S7>/Switch9' incorporates:
   *  UnitDelay: '<S41>/Delay Input2'
   *  UnitDelay: '<S7>/Unit Delay'
   *
   * Block description for '<S41>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_n != 0.0) {
    rtb_Yk1_l = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hj;
  } else {
    /* Product: '<S7>/Product1' incorporates:
     *  Constant: '<S7>/Constant17'
     */
    rtb_Yk1_l = rtb_Switch2_mn * 0.099999999999999978;
  }

  /* End of Switch: '<S7>/Switch9' */

  /* RelationalOperator: '<S27>/LowerRelop1' */
  rtb_AND_l = (rtb_Yk1_l > rtb_UkYk1_ll);

  /* Switch: '<S27>/Switch2' */
  if (rtb_AND_l) {
    rtb_Acc_POS = (real32_T)rtb_UkYk1_ll;
  } else {
    /* RelationalOperator: '<S27>/UpperRelop' incorporates:
     *  Constant: '<S7>/Constant2'
     */
    rtb_LogicalOperator3 = (rtb_Yk1_l < 0.0);

    /* Switch: '<S27>/Switch' incorporates:
     *  Constant: '<S7>/Constant2'
     */
    if (rtb_LogicalOperator3) {
      rtb_Acc_POS = 0.0F;
    } else {
      rtb_Acc_POS = (real32_T)rtb_Yk1_l;
    }

    /* End of Switch: '<S27>/Switch' */
  }

  /* End of Switch: '<S27>/Switch2' */

  /* Switch: '<S30>/Switch6' incorporates:
   *  Constant: '<S30>/Verror_Reset'
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  if (VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_dp) {
    rtb_Ax = rtb_Gain3_o;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S30>/Switch6' */

  /* Product: '<S30>/Product' incorporates:
   *  Constant: '<S30>/P_Gain'
   */
  FRWhlStrAng = rtb_Ax * 40.0F;

  /* Sum: '<S30>/Add11' */
  rtb_Add4_j = rtb_Acc_POS - FRWhlStrAng;

  /* UnitDelay: '<S30>/Unit Delay5' */
  rtb_Add6 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay5_DSTATE_ip;

  /* Product: '<S30>/Product2' */
  rtb_Add6 *= rtb_Gain3_o;

  /* RelationalOperator: '<S74>/Compare' incorporates:
   *  Constant: '<S74>/Constant'
   */
  rtb_AND_l = (rtb_Add6 <= 0.0F);

  /* UnitDelay: '<S30>/Unit Delay' */
  rtb_Add6 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_nr;

  /* Switch: '<S30>/Switch3' incorporates:
   *  Constant: '<S30>/Verror_Reset1'
   */
  if (rtb_AND_l) {
    rtb_Add6 = 0.0F;
  }

  /* End of Switch: '<S30>/Switch3' */

  /* Sum: '<S30>/Add2' */
  rtb_Add6 += rtb_Ax;

  /* Saturate: '<S30>/Saturation2' */
  if (rtb_Add6 > 400.0F) {
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_nr = 400.0F;
  } else if (rtb_Add6 < -100.0F) {
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_nr = -100.0F;
  } else {
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_nr = rtb_Add6;
  }

  /* End of Saturate: '<S30>/Saturation2' */

  /* RelationalOperator: '<S82>/Compare' incorporates:
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  rtb_LogicalOperator3 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_dp;

  /* UnitDelay: '<S76>/Delay Input1'
   *
   * Block description for '<S76>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_AND_l = VehCtrlMdel241025_2018b_amks_DW.DelayInput1_DSTATE_b;

  /* RelationalOperator: '<S76>/FixPt Relational Operator' */
  rtb_AND_l = ((int32_T)rtb_LogicalOperator3 > (int32_T)rtb_AND_l);

  /* Switch: '<S30>/Switch' incorporates:
   *  Constant: '<S30>/Integr_StartPoint'
   */
  if (rtb_AND_l) {
    /* Sum: '<S30>/Add4' */
    rtb_Ax = rtb_Acc_POS - VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_g;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S30>/Switch' */

  /* Lookup_n-D: '<S30>/VehicleStableTarget_mps' */
  rtb_Add6 = look1_iflf_binlc(rtb_Add7, VehCtrlMdel241025_2018b__ConstP.pooled63,
    VehCtrlMdel241025_2018b__ConstP.pooled69, 3U);

  /* Sum: '<S30>/Add5' */
  rtb_Add6 += rtb_Add7;

  /* Sum: '<S30>/Add10' */
  rtb_Add6 = rtb_VxIMU_est - rtb_Add6;

  /* RelationalOperator: '<S30>/Relational Operator' incorporates:
   *  Constant: '<S30>/Verror'
   */
  rtb_AND_l = (rtb_Add6 < 0.0F);

  /* Logic: '<S30>/Logical Operator4' incorporates:
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  rtb_AND_l = (rtb_AND_l && VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_dp);

  /* Switch: '<S30>/Switch1' incorporates:
   *  Constant: '<S30>/Trq_IReset'
   *  Constant: '<S30>/Trq_I_FF'
   */
  if (rtb_AND_l) {
    rtb_Add6 = 20.0F;
  } else {
    rtb_Add6 = 0.0F;
  }

  /* End of Switch: '<S30>/Switch1' */

  /* Sum: '<S30>/Add6' incorporates:
   *  UnitDelay: '<S30>/Unit Delay'
   */
  rtb_Ax = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_nr + rtb_Ax) +
    rtb_Add6;

  /* Product: '<S30>/Product1' incorporates:
   *  Constant: '<S30>/I_Gain'
   */
  rtb_deltafalllimit_n = rtb_Ax * 10.0F;

  /* RelationalOperator: '<S79>/LowerRelop1' */
  rtb_AND_l = (rtb_deltafalllimit_n > rtb_Add4_j);

  /* Switch: '<S79>/Switch2' */
  if (!rtb_AND_l) {
    /* Gain: '<S30>/Gain3' */
    rtb_deltafalllimit_om = -FRWhlStrAng;

    /* RelationalOperator: '<S79>/UpperRelop' */
    rtb_AND_l = (rtb_deltafalllimit_n < rtb_deltafalllimit_om);

    /* Switch: '<S79>/Switch' */
    if (rtb_AND_l) {
      rtb_deltafalllimit_n = rtb_deltafalllimit_om;
    }

    /* End of Switch: '<S79>/Switch' */
    rtb_Add4_j = rtb_deltafalllimit_n;
  }

  /* End of Switch: '<S79>/Switch2' */

  /* Sum: '<S30>/Add7' incorporates:
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_l = FRWhlStrAng + rtb_Add4_j;

  /* Lookup_n-D: '<S30>/VehicleStableTarget_mps1' */
  rtb_Ax = look1_iflf_binlc(rtb_Add7, VehCtrlMdel241025_2018b__ConstP.pooled63,
    VehCtrlMdel241025_2018b__ConstP.pooled69, 3U);

  /* Sum: '<S30>/Add13' */
  rtb_Add7 += rtb_Ax;

  /* Sum: '<S30>/Add12' */
  rtb_VxIMU_est -= rtb_Add7;

  /* RelationalOperator: '<S30>/Relational Operator1' incorporates:
   *  Constant: '<S30>/Verror1'
   */
  rtb_AND_l = (rtb_VxIMU_est < 0.0F);

  /* RelationalOperator: '<S30>/Relational Operator2' incorporates:
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  TrqR_cmd_raw = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_l >=
                  rtb_Acc_POS);

  /* RelationalOperator: '<S75>/Compare' incorporates:
   *  Constant: '<S75>/Constant'
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  rtb_AND2_e = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_l <= 5.0F);

  /* Logic: '<S30>/OR' */
  TrqR_cmd_raw = (TrqR_cmd_raw || rtb_AND2_e);

  /* Logic: '<S30>/Logical Operator5' incorporates:
   *  UnitDelay: '<S30>/Unit Delay3'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_ip = (rtb_AND_l &&
    TrqR_cmd_raw);

  /* Switch: '<S30>/Switch2' incorporates:
   *  Switch: '<S30>/Switch7'
   *  UnitDelay: '<S30>/Unit Delay3'
   *  UnitDelay: '<S77>/Unit Delay1'
   */
  if (VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_dp) {
    /* RelationalOperator: '<S80>/LowerRelop1' incorporates:
     *  Constant: '<S30>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S30>/Unit Delay4'
     */
    rtb_AND_l = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_l > 235.0F);

    /* Switch: '<S80>/Switch2' incorporates:
     *  Constant: '<S30>/TCS_TrqRequest_Max2'
     */
    if (rtb_AND_l) {
      VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_b = 235.0F;
    } else {
      /* RelationalOperator: '<S80>/UpperRelop' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S30>/Unit Delay4'
       */
      rtb_AND_l = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_l < 0.0F);

      /* Switch: '<S80>/Switch' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S30>/Unit Delay4'
       */
      if (rtb_AND_l) {
        VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_b = 0.0F;
      } else {
        VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_b =
          VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_l;
      }

      /* End of Switch: '<S80>/Switch' */
    }

    /* End of Switch: '<S80>/Switch2' */

    /* RelationalOperator: '<S81>/LowerRelop1' */
    rtb_AND_l = (rtb_Acc_POS >
                 VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_b);

    /* Switch: '<S81>/Switch2' */
    if (!rtb_AND_l) {
      /* RelationalOperator: '<S81>/UpperRelop' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min1'
       */
      rtb_AND_l = (rtb_Acc_POS < 0.0F);

      /* Switch: '<S81>/Switch' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min1'
       */
      if (rtb_AND_l) {
        VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_b = 0.0F;
      } else {
        VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_b = rtb_Acc_POS;
      }

      /* End of Switch: '<S81>/Switch' */
    }

    /* End of Switch: '<S81>/Switch2' */
  } else {
    if (VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_ip) {
      /* Switch: '<S30>/Switch7' */
      VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_b = rtb_Acc_POS;
    }
  }

  /* End of Switch: '<S30>/Switch2' */

  /* Switch: '<S7>/Switch7' incorporates:
   *  UnitDelay: '<S30>/Unit Delay2'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.TCSF_Enable_OUT != 0.0) {
    rtb_Acc_POS = VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_b;
  }

  /* End of Switch: '<S7>/Switch7' */

  /* Lookup_n-D: '<S7>/AMK3' */
  rtb_UkYk1_ll = look1_binlx(MCFL_ActualVelocity,
    VehCtrlMdel241025_2018b__ConstP.pooled8,
    VehCtrlMdel241025_2018b__ConstP.pooled17, 19U);

  /* Gain: '<S7>/Gain7' */
  rtb_Yk1_l = 0.95 * rtb_UkYk1_ll;

  /* Switch: '<S28>/Switch2' incorporates:
   *  Constant: '<S28>/Constant13'
   */
  if (rtb_LogicalOperator7_m) {
    rtb_UkYk1_ll = 6.0;
  } else {
    /* Sum: '<S28>/Add1' incorporates:
     *  Constant: '<S28>/RPM_min'
     */
    elapseTime = RPM + 10.0;

    /* MinMax: '<S28>/Max' incorporates:
     *  Constant: '<S28>/RPM_min1'
     */
    rtb_deltafalllimit_le = fmax(elapseTime, 1.0);

    /* Sum: '<S28>/Add' incorporates:
     *  Constant: '<S28>/Constant3'
     */
    elapseTime = 1.0 - WhlSpdRR_mps;

    /* Switch: '<S28>/Switch5' incorporates:
     *  Constant: '<S28>/Constant16'
     *  Constant: '<S28>/Constant6'
     */
    if (VehCtrlMdel241025_2018b_amksp_B.ModeSW_o != 0.0) {
      rtb_Switch4_o = 36;
    } else {
      rtb_Switch4_o = 50;
    }

    /* End of Switch: '<S28>/Switch5' */

    /* Product: '<S28>/Product5' */
    elapseTime *= (real_T)rtb_Switch4_o;

    /* Product: '<S28>/Product' */
    elapseTime *= 9550.0;

    /* Product: '<S28>/Divide' */
    rtb_UkYk1_ll = elapseTime / rtb_deltafalllimit_le;
  }

  /* End of Switch: '<S28>/Switch2' */

  /* UnitDelay: '<S43>/Delay Input2'
   *
   * Block description for '<S43>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add5 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_pd;

  /* SampleTimeMath: '<S43>/sample time'
   *
   * About '<S43>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S43>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  rtb_Switch2_cn = 2000.0 * elapseTime;

  /* RelationalOperator: '<S50>/LowerRelop1' */
  rtb_AND_l = (rtb_Switch2_on > rtb_Add10);

  /* Switch: '<S50>/Switch2' */
  if (rtb_AND_l) {
    rtb_Switch2_gd = rtb_Add10;
  } else {
    /* RelationalOperator: '<S50>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    rtb_LogicalOperator7_m = (rtb_Switch2_on < 0.0);

    /* Switch: '<S50>/Switch' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    if (rtb_LogicalOperator7_m) {
      rtb_Switch2_on = 0.0;
    }

    /* End of Switch: '<S50>/Switch' */
    rtb_Switch2_gd = rtb_Switch2_on;
  }

  /* End of Switch: '<S50>/Switch2' */

  /* UnitDelay: '<S46>/Delay Input2'
   *
   * Block description for '<S46>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1 = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_nk;

  /* SampleTimeMath: '<S46>/sample time'
   *
   * About '<S46>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime_0 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S46>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant45'
   */
  rtb_Gain4 = 1000.0 * elapseTime_0;

  /* Logic: '<S10>/AND1' */
  rtb_AND_l = (rtb_Compare_i && rtb_Compare_c);

  /* Logic: '<S10>/OR1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_AND_l = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_i || rtb_AND_l);

  /* Switch: '<S10>/Switch2' incorporates:
   *  Constant: '<S10>/Constant32'
   */
  if (rtb_AND_l) {
    rtb_Gain5 = 0.0;
  } else {
    /* Logic: '<S10>/NOT1' */
    rtb_LogicalOperator7_m = !rtb_Compare_c;

    /* Switch: '<S10>/Switch5' incorporates:
     *  Constant: '<S10>/Constant33'
     */
    if (!rtb_LogicalOperator7_m) {
      rtb_g_mpss1 = 0.0;
    }

    /* End of Switch: '<S10>/Switch5' */
    rtb_Gain5 = rtb_g_mpss1;
  }

  /* End of Switch: '<S10>/Switch2' */

  /* Gain: '<S10>/Gain19' */
  rtb_Gain5 = -rtb_Gain5;

  /* Saturate: '<S10>/Saturation1' */
  if (rtb_Gain5 > 100.0) {
    rtb_Gain5 = 100.0;
  } else {
    if (rtb_Gain5 < 0.0) {
      rtb_Gain5 = 0.0;
    }
  }

  /* End of Saturate: '<S10>/Saturation1' */

  /* Sum: '<S46>/Difference Inputs1'
   *
   * Block description for '<S46>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_le = rtb_Gain5 - rtb_Yk1;

  /* RelationalOperator: '<S63>/LowerRelop1' */
  rtb_AND_l = (rtb_deltafalllimit_le > rtb_Gain4);

  /* Switch: '<S63>/Switch2' */
  if (!rtb_AND_l) {
    /* Product: '<S46>/delta fall limit' */
    WhlSpdRR_mps = -1000.0 * elapseTime_0;

    /* RelationalOperator: '<S63>/UpperRelop' */
    rtb_LogicalOperator7_m = (rtb_deltafalllimit_le < WhlSpdRR_mps);

    /* Switch: '<S63>/Switch' */
    if (rtb_LogicalOperator7_m) {
      rtb_deltafalllimit_le = WhlSpdRR_mps;
    }

    /* End of Switch: '<S63>/Switch' */
    rtb_Gain4 = rtb_deltafalllimit_le;
  }

  /* End of Switch: '<S63>/Switch2' */

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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_nk = rtb_Gain4 + rtb_Yk1;

  /* Sum: '<S10>/Add13' incorporates:
   *  UnitDelay: '<S46>/Delay Input2'
   *
   * Block description for '<S46>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_deltafalllimit_le = rtb_Switch2_gd +
    VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_nk;

  /* RelationalOperator: '<S53>/LowerRelop1' */
  rtb_AND_l = (rtb_deltafalllimit_le > rtb_Add10);

  /* Switch: '<S53>/Switch2' */
  if (rtb_AND_l) {
    rtb_deltafalllimit_le = rtb_Add10;
  } else {
    /* RelationalOperator: '<S53>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    rtb_LogicalOperator7_m = (rtb_deltafalllimit_le < 0.0);

    /* Switch: '<S53>/Switch' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    if (rtb_LogicalOperator7_m) {
      rtb_deltafalllimit_le = 0.0;
    }

    /* End of Switch: '<S53>/Switch' */
  }

  /* End of Switch: '<S53>/Switch2' */

  /* Sum: '<S43>/Difference Inputs1'
   *
   * Block description for '<S43>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_le -= rtb_Add5;

  /* RelationalOperator: '<S60>/LowerRelop1' */
  rtb_AND_l = (rtb_deltafalllimit_le > rtb_Switch2_cn);

  /* Switch: '<S60>/Switch2' */
  if (!rtb_AND_l) {
    /* Product: '<S43>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S60>/UpperRelop' */
    rtb_LogicalOperator7_m = (rtb_deltafalllimit_le < elapseTime);

    /* Switch: '<S60>/Switch' */
    if (rtb_LogicalOperator7_m) {
      rtb_deltafalllimit_le = elapseTime;
    }

    /* End of Switch: '<S60>/Switch' */
    rtb_Switch2_cn = rtb_deltafalllimit_le;
  }

  /* End of Switch: '<S60>/Switch2' */

  /* Sum: '<S43>/Difference Inputs2' incorporates:
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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_pd = rtb_Switch2_cn +
    rtb_Add5;

  /* Switch: '<S7>/Switch1' incorporates:
   *  UnitDelay: '<S43>/Delay Input2'
   *  UnitDelay: '<S7>/Unit Delay'
   *
   * Block description for '<S43>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_n != 0.0) {
    rtb_deltafalllimit_le =
      VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_pd;
  } else {
    /* Logic: '<S7>/NOT1' */
    rtb_LogicalOperator7_m = !rtb_ignition_e;

    /* Switch: '<S7>/Switch10' */
    if (rtb_LogicalOperator7_m) {
      /* Product: '<S7>/Product2' incorporates:
       *  Constant: '<S7>/Constant'
       */
      rtb_deltafalllimit_le = rtb_Switch2_mn * 0.8;
    } else {
      rtb_deltafalllimit_le = rtb_Switch2_mn;
    }

    /* End of Switch: '<S7>/Switch10' */
  }

  /* End of Switch: '<S7>/Switch1' */

  /* RelationalOperator: '<S25>/LowerRelop1' */
  rtb_AND_l = (rtb_deltafalllimit_le > rtb_UkYk1_ll);

  /* Switch: '<S25>/Switch2' */
  if (rtb_AND_l) {
    rtb_deltafalllimit_om = (real32_T)rtb_UkYk1_ll;
  } else {
    /* RelationalOperator: '<S25>/UpperRelop' incorporates:
     *  Constant: '<S7>/Constant1'
     */
    rtb_LogicalOperator7_m = (rtb_deltafalllimit_le < 0.0);

    /* Switch: '<S25>/Switch' incorporates:
     *  Constant: '<S7>/Constant1'
     */
    if (rtb_LogicalOperator7_m) {
      rtb_deltafalllimit_om = 0.0F;
    } else {
      rtb_deltafalllimit_om = (real32_T)rtb_deltafalllimit_le;
    }

    /* End of Switch: '<S25>/Switch' */
  }

  /* End of Switch: '<S25>/Switch2' */

  /* Switch: '<S32>/Switch6' incorporates:
   *  Constant: '<S32>/Verror_Reset'
   */
  if (rtb_LogicalOperator2) {
    rtb_Add7 = rtb_StrWhlAngV_c;
  } else {
    rtb_Add7 = 0.0F;
  }

  /* End of Switch: '<S32>/Switch6' */

  /* Product: '<S32>/Product' incorporates:
   *  Constant: '<S32>/P_Gain'
   */
  rtb_deltafalllimit_n = rtb_Add7 * 40.0F;

  /* Sum: '<S32>/Add11' */
  rtb_Ax = rtb_deltafalllimit_om - rtb_deltafalllimit_n;

  /* UnitDelay: '<S32>/Unit Delay5' */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay5_DSTATE_l;

  /* Product: '<S32>/Product2' */
  rtb_Add10 *= rtb_StrWhlAngV_c;

  /* RelationalOperator: '<S92>/Compare' incorporates:
   *  Constant: '<S92>/Constant'
   */
  rtb_AND_l = (rtb_Add10 <= 0.0F);

  /* UnitDelay: '<S32>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_b;

  /* Switch: '<S32>/Switch3' incorporates:
   *  Constant: '<S32>/Verror_Reset1'
   */
  if (rtb_AND_l) {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S32>/Switch3' */

  /* Sum: '<S32>/Add2' */
  rtb_Add10 += rtb_Add7;

  /* Saturate: '<S32>/Saturation2' */
  if (rtb_Add10 > 400.0F) {
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_b = 400.0F;
  } else if (rtb_Add10 < -100.0F) {
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_b = -100.0F;
  } else {
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_b = rtb_Add10;
  }

  /* End of Saturate: '<S32>/Saturation2' */

  /* UnitDelay: '<S96>/Delay Input1'
   *
   * Block description for '<S96>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_AND_l = VehCtrlMdel241025_2018b_amks_DW.DelayInput1_DSTATE_e;

  /* RelationalOperator: '<S96>/FixPt Relational Operator' */
  rtb_AND_l = ((int32_T)rtb_LogicalOperator2 > (int32_T)rtb_AND_l);

  /* Switch: '<S32>/Switch' incorporates:
   *  Constant: '<S32>/Integr_StartPoint'
   */
  if (rtb_AND_l) {
    /* Sum: '<S32>/Add4' */
    rtb_Add7 = rtb_deltafalllimit_om -
      VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_gu;
  } else {
    rtb_Add7 = 0.0F;
  }

  /* End of Switch: '<S32>/Switch' */

  /* Lookup_n-D: '<S32>/VehicleStableTarget_mps' */
  rtb_Add10 = look1_iflf_binlc(Acc_POS_n,
    VehCtrlMdel241025_2018b__ConstP.pooled63,
    VehCtrlMdel241025_2018b__ConstP.pooled69, 3U);

  /* Sum: '<S32>/Add5' */
  rtb_Add10 += Acc_POS_n;

  /* Sum: '<S32>/Add10' */
  rtb_Add10 = rtb_Switch2_b0 - rtb_Add10;

  /* RelationalOperator: '<S32>/Relational Operator' incorporates:
   *  Constant: '<S32>/Verror'
   */
  rtb_AND_l = (rtb_Add10 < 0.0F);

  /* Logic: '<S32>/Logical Operator4' */
  rtb_AND_l = (rtb_AND_l && rtb_LogicalOperator2);

  /* Switch: '<S32>/Switch1' incorporates:
   *  Constant: '<S32>/Trq_IReset'
   *  Constant: '<S32>/Trq_I_FF'
   */
  if (rtb_AND_l) {
    rtb_Add10 = 20.0F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S32>/Switch1' */

  /* Sum: '<S32>/Add6' incorporates:
   *  UnitDelay: '<S32>/Unit Delay'
   */
  rtb_Add7 = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_b + rtb_Add7) +
    rtb_Add10;

  /* Product: '<S32>/Product1' incorporates:
   *  Constant: '<S32>/I_Gain'
   */
  rtb_Add6 = rtb_Add7 * 10.0F;

  /* RelationalOperator: '<S99>/LowerRelop1' */
  rtb_AND_l = (rtb_Add6 > rtb_Ax);

  /* Switch: '<S99>/Switch2' */
  if (!rtb_AND_l) {
    /* Gain: '<S32>/Gain3' */
    rtb_Add7 = -rtb_deltafalllimit_n;

    /* RelationalOperator: '<S99>/UpperRelop' */
    rtb_LogicalOperator7_m = (rtb_Add6 < rtb_Add7);

    /* Switch: '<S99>/Switch' */
    if (rtb_LogicalOperator7_m) {
      rtb_Add6 = rtb_Add7;
    }

    /* End of Switch: '<S99>/Switch' */
    rtb_Ax = rtb_Add6;
  }

  /* End of Switch: '<S99>/Switch2' */

  /* Sum: '<S32>/Add7' incorporates:
   *  UnitDelay: '<S32>/Unit Delay4'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_b = rtb_deltafalllimit_n +
    rtb_Ax;

  /* Lookup_n-D: '<S32>/VehicleStableTarget_mps1' */
  rtb_Add7 = look1_iflf_binlc(Acc_POS_n,
    VehCtrlMdel241025_2018b__ConstP.pooled63,
    VehCtrlMdel241025_2018b__ConstP.pooled69, 3U);

  /* Sum: '<S32>/Add13' */
  Acc_POS_n += rtb_Add7;

  /* Sum: '<S32>/Add12' */
  rtb_Switch2_b0 -= Acc_POS_n;

  /* RelationalOperator: '<S32>/Relational Operator1' incorporates:
   *  Constant: '<S32>/Verror1'
   */
  rtb_AND_l = (rtb_Switch2_b0 < 0.0F);

  /* RelationalOperator: '<S32>/Relational Operator2' incorporates:
   *  UnitDelay: '<S32>/Unit Delay4'
   */
  TrqR_cmd_raw = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_b >=
                  rtb_deltafalllimit_om);

  /* RelationalOperator: '<S94>/Compare' incorporates:
   *  Constant: '<S94>/Constant'
   *  UnitDelay: '<S32>/Unit Delay4'
   */
  rtb_AND2_e = (VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_b <= 5.0F);

  /* Logic: '<S32>/OR' */
  TrqR_cmd_raw = (TrqR_cmd_raw || rtb_AND2_e);

  /* Logic: '<S32>/Logical Operator5' incorporates:
   *  UnitDelay: '<S32>/Unit Delay3'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_a = (rtb_AND_l &&
    TrqR_cmd_raw);

  /* Switch: '<S32>/Switch2' incorporates:
   *  Switch: '<S32>/Switch7'
   *  UnitDelay: '<S32>/Unit Delay3'
   */
  if (rtb_LogicalOperator2) {
    /* RelationalOperator: '<S100>/LowerRelop1' incorporates:
     *  Constant: '<S32>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S32>/Unit Delay4'
     */
    rtb_LogicalOperator7_m =
      (VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_b > 235.0F);

    /* Switch: '<S100>/Switch2' incorporates:
     *  Constant: '<S32>/TCS_TrqRequest_Max2'
     */
    if (rtb_LogicalOperator7_m) {
      VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_g = 235.0F;
    } else {
      /* RelationalOperator: '<S100>/UpperRelop' incorporates:
       *  Constant: '<S32>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S32>/Unit Delay4'
       */
      rtb_LogicalOperator7_m =
        (VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_b < 0.0F);

      /* Switch: '<S100>/Switch' incorporates:
       *  Constant: '<S32>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S32>/Unit Delay4'
       */
      if (rtb_LogicalOperator7_m) {
        VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_g = 0.0F;
      } else {
        VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_g =
          VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_b;
      }

      /* End of Switch: '<S100>/Switch' */
    }

    /* End of Switch: '<S100>/Switch2' */

    /* RelationalOperator: '<S101>/LowerRelop1' */
    rtb_LogicalOperator7_m = (rtb_deltafalllimit_om >
      VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_g);

    /* Switch: '<S101>/Switch2' */
    if (!rtb_LogicalOperator7_m) {
      /* RelationalOperator: '<S101>/UpperRelop' incorporates:
       *  Constant: '<S32>/TCS_TrqRequest_Min1'
       */
      rtb_LogicalOperator7_m = (rtb_deltafalllimit_om < 0.0F);

      /* Switch: '<S101>/Switch' incorporates:
       *  Constant: '<S32>/TCS_TrqRequest_Min1'
       */
      if (rtb_LogicalOperator7_m) {
        VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_g = 0.0F;
      } else {
        VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_g =
          rtb_deltafalllimit_om;
      }

      /* End of Switch: '<S101>/Switch' */
    }

    /* End of Switch: '<S101>/Switch2' */
  } else {
    if (VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_a) {
      /* Switch: '<S32>/Switch7' */
      VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_g =
        rtb_deltafalllimit_om;
    }
  }

  /* End of Switch: '<S32>/Switch2' */

  /* Logic: '<S7>/Logical Operator2' */
  rtb_AND_l = !VehCtrlMdel241025_2018b_amksp_B.VehReady;

  /* Logic: '<S7>/Logical Operator3' */
  TrqR_cmd_raw = (rtb_AND_l || (Trq_CUT != 0.0));

  /* Switch: '<S7>/Switch2' incorporates:
   *  Constant: '<S7>/Constant4'
   *  Switch: '<S7>/Switch12'
   */
  if (TrqR_cmd_raw) {
    rtb_deltafalllimit_om = 0.0F;
  } else {
    if (rtb_ignition_e) {
      /* Lookup_n-D: '<S7>/228RWD' incorporates:
       *  Switch: '<S7>/Switch12'
       */
      rtb_deltafalllimit_le = look1_binlx(RPM,
        VehCtrlMdel241025_2018b__ConstP.pooled4,
        VehCtrlMdel241025_2018b__ConstP.u28RWD_tableData, 26U);

      /* Gain: '<S7>/Gain9' incorporates:
       *  Switch: '<S7>/Switch12'
       */
      rtb_deltafalllimit_le *= 0.95;
    } else {
      /* Lookup_n-D: '<S7>/228RWD1' incorporates:
       *  Switch: '<S7>/Switch12'
       */
      rtb_deltafalllimit_le = look1_binlx(RPM,
        VehCtrlMdel241025_2018b__ConstP.pooled4,
        VehCtrlMdel241025_2018b__ConstP.u28RWD1_tableData, 26U);

      /* Gain: '<S7>/Gain11' incorporates:
       *  Switch: '<S7>/Switch12'
       */
      rtb_deltafalllimit_le *= 0.95;
    }

    /* Switch: '<S7>/Switch5' incorporates:
     *  UnitDelay: '<S32>/Unit Delay2'
     */
    if (VehCtrlMdel241025_2018b_amksp_B.TCSR_Enable_OUT != 0.0) {
      rtb_deltafalllimit_om =
        VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_g;
    }

    /* End of Switch: '<S7>/Switch5' */

    /* Logic: '<S7>/NOT3' */
    rtb_LogicalOperator7_m = !rtb_ignition_e;

    /* Switch: '<S7>/Switch11' incorporates:
     *  Constant: '<S7>/Constant23'
     */
    if (rtb_LogicalOperator7_m) {
      /* Sum: '<S7>/Add2' */
      elapseTime = rtb_Acc_POS - rtb_Yk1_l;

      /* Saturate: '<S7>/Saturation' */
      if (elapseTime > 10.0) {
        elapseTime = 10.0;
      } else {
        if (elapseTime < 0.0) {
          elapseTime = 0.0;
        }
      }

      /* End of Saturate: '<S7>/Saturation' */

      /* Sum: '<S7>/Add' */
      WhlSpdRR_mps = rtb_MaxWhlSpd_mps_n - rtb_UkYk1;

      /* Saturate: '<S7>/Saturation2' */
      if (WhlSpdRR_mps > 10.0) {
        WhlSpdRR_mps = 10.0;
      } else {
        if (WhlSpdRR_mps < 0.0) {
          WhlSpdRR_mps = 0.0;
        }
      }

      /* End of Saturate: '<S7>/Saturation2' */

      /* MinMax: '<S7>/Min3' */
      WhlSpdRR_mps = fmin(WhlSpdRR_mps, elapseTime);

      /* Gain: '<S7>/Gain1' */
      WhlSpdRR_mps *= 2.0;

      /* Gain: '<S7>/Gain10' */
      elapseTime = 3.0483870967741935 * WhlSpdRR_mps;
    } else {
      elapseTime = 0.0;
    }

    /* End of Switch: '<S7>/Switch11' */

    /* Sum: '<S7>/Add3' */
    elapseTime += rtb_deltafalllimit_om;

    /* MinMax: '<S7>/Min2' */
    rtb_deltafalllimit_le = fmin(rtb_deltafalllimit_le, elapseTime);

    /* Gain: '<S7>/Gain6' */
    rtb_deltafalllimit_le *= 4.7619047619047619;

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefRear' */
    rtb_deltafalllimit_om = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel241025_2018b__ConstP.BrakeCompensateCoefRear_bp01Dat,
      VehCtrlMdel241025_2018b__ConstP.BrakeCompensateCoefRear_tableDa, 1U);

    /* RelationalOperator: '<S22>/LowerRelop1' */
    rtb_LogicalOperator7_m = (rtb_deltafalllimit_le > rtb_deltafalllimit_om);

    /* Switch: '<S22>/Switch2' */
    if (!rtb_LogicalOperator7_m) {
      /* RelationalOperator: '<S22>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      rtb_LogicalOperator7_m = (rtb_deltafalllimit_le < 0.0);

      /* Switch: '<S22>/Switch' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      if (rtb_LogicalOperator7_m) {
        rtb_deltafalllimit_om = 0.0F;
      } else {
        rtb_deltafalllimit_om = (real32_T)rtb_deltafalllimit_le;
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
  Acc_POS_n = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_cd;

  /* Sum: '<S19>/Difference Inputs1'
   *
   * Block description for '<S19>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_om -= Acc_POS_n;

  /* SampleTimeMath: '<S19>/sample time'
   *
   * About '<S19>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S19>/delta rise limit' */
  rtb_Add7 = (real32_T)(25000.0 * elapseTime);

  /* RelationalOperator: '<S65>/LowerRelop1' */
  rtb_AND_l = (rtb_deltafalllimit_om > rtb_Add7);

  /* Switch: '<S65>/Switch2' */
  if (!rtb_AND_l) {
    /* Product: '<S19>/delta fall limit' */
    rtb_Add6 = (real32_T)(-25000.0 * elapseTime);

    /* RelationalOperator: '<S65>/UpperRelop' */
    rtb_LogicalOperator7_m = (rtb_deltafalllimit_om < rtb_Add6);

    /* Switch: '<S65>/Switch' */
    if (rtb_LogicalOperator7_m) {
      rtb_deltafalllimit_om = rtb_Add6;
    }

    /* End of Switch: '<S65>/Switch' */
    rtb_Add7 = rtb_deltafalllimit_om;
  }

  /* End of Switch: '<S65>/Switch2' */

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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_cd = rtb_Add7 + Acc_POS_n;
  if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_cd > 1000.0F) {
    EmraxTrqR_cmd = 1000.0F;
  } else if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_cd < 0.0F) {
    EmraxTrqR_cmd = 0.0F;
  } else {
    EmraxTrqR_cmd = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_cd;
  }

  /* End of Saturate: '<S7>/Saturation1' */

  /* Logic: '<S7>/Logical Operator5' */
  rtb_AND_l = (VehCtrlMdel241025_2018b_amksp_B.MCFR_TorqueOn &&
               VehCtrlMdel241025_2018b_amksp_B.MCFL_TorqueOn);

  /* Logic: '<S7>/Logical Operator6' */
  TroqueOn = !rtb_AND_l;

  /* Logic: '<S7>/Logical Operator4' */
  Trq_CUT_final = (TroqueOn || (AMK_Trq_CUT != 0.0) || TrqR_cmd_raw ||
                   TrqR_cmd_raw);

  /* Logic: '<S7>/OR' */
  rtb_AND_l = (Trq_CUT_final || rtb_ignition_e);

  /* Lookup_n-D: '<S7>/BrakeCompensateCoefFront1' */
  rtb_deltafalllimit_om = look1_iflf_binlc((real32_T)Brk_F,
    VehCtrlMdel241025_2018b__ConstP.BrakeCompensateCoefFront1_bp01D,
    VehCtrlMdel241025_2018b__ConstP.BrakeCompensateCoefFront1_table, 1U);

  /* Switch: '<S7>/Switch3' incorporates:
   *  Constant: '<S7>/Constant19'
   */
  if (rtb_AND_l) {
    rtb_Add6 = 0.0F;
  } else {
    /* MinMax: '<S7>/Min1' */
    rtb_UkYk1 = fmin(rtb_MaxWhlSpd_mps_n, rtb_UkYk1);

    /* RelationalOperator: '<S23>/LowerRelop1' */
    rtb_LogicalOperator7_m = (rtb_UkYk1 > rtb_deltafalllimit_om);

    /* Switch: '<S23>/Switch2' */
    if (rtb_LogicalOperator7_m) {
      rtb_Add6 = rtb_deltafalllimit_om;
    } else {
      /* RelationalOperator: '<S23>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      rtb_LogicalOperator7_m = (rtb_UkYk1 < 0.0);

      /* Switch: '<S23>/Switch' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      if (rtb_LogicalOperator7_m) {
        rtb_Add6 = 0.0F;
      } else {
        rtb_Add6 = (real32_T)rtb_UkYk1;
      }

      /* End of Switch: '<S23>/Switch' */
    }

    /* End of Switch: '<S23>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch3' */

  /* UnitDelay: '<S20>/Delay Input2'
   *
   * Block description for '<S20>/Delay Input2':
   *
   *  Store in Global RAM
   */
  Acc_POS_n = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hn;

  /* Sum: '<S20>/Difference Inputs1'
   *
   * Block description for '<S20>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add6 -= Acc_POS_n;

  /* SampleTimeMath: '<S20>/sample time'
   *
   * About '<S20>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S20>/delta rise limit' */
  rtb_Add7 = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S66>/LowerRelop1' */
  rtb_AND_l = (rtb_Add6 > rtb_Add7);

  /* Switch: '<S66>/Switch2' */
  if (!rtb_AND_l) {
    /* Product: '<S20>/delta fall limit' */
    rtb_Add7 = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S66>/UpperRelop' */
    rtb_LogicalOperator7_m = (rtb_Add6 < rtb_Add7);

    /* Switch: '<S66>/Switch' */
    if (rtb_LogicalOperator7_m) {
      rtb_Add6 = rtb_Add7;
    }

    /* End of Switch: '<S66>/Switch' */
    rtb_Add7 = rtb_Add6;
  }

  /* End of Switch: '<S66>/Switch2' */

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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hn = rtb_Add7 + Acc_POS_n;
  if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hn > 21.0F) {
    AMKTrqFR_cmd = 21.0F;
  } else if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hn < -21.0F) {
    AMKTrqFR_cmd = -21.0F;
  } else {
    AMKTrqFR_cmd = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_hn;
  }

  /* End of Saturate: '<S7>/Saturation3' */

  /* Logic: '<S7>/OR1' */
  rtb_AND_l = (Trq_CUT_final || rtb_ignition_e);

  /* Switch: '<S7>/Switch4' incorporates:
   *  Constant: '<S7>/Constant8'
   */
  if (rtb_AND_l) {
    rtb_deltafalllimit_om = 0.0F;
  } else {
    /* MinMax: '<S7>/Min' */
    rtb_UkYk1 = fmin(rtb_Yk1_l, rtb_Acc_POS);

    /* RelationalOperator: '<S24>/LowerRelop1' */
    rtb_ignition_e = (rtb_UkYk1 > rtb_deltafalllimit_om);

    /* Switch: '<S24>/Switch2' */
    if (!rtb_ignition_e) {
      /* RelationalOperator: '<S24>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      rtb_ignition_e = (rtb_UkYk1 < 0.0);

      /* Switch: '<S24>/Switch' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      if (rtb_ignition_e) {
        rtb_deltafalllimit_om = 0.0F;
      } else {
        rtb_deltafalllimit_om = (real32_T)rtb_UkYk1;
      }

      /* End of Switch: '<S24>/Switch' */
    }

    /* End of Switch: '<S24>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch4' */

  /* UnitDelay: '<S21>/Delay Input2'
   *
   * Block description for '<S21>/Delay Input2':
   *
   *  Store in Global RAM
   */
  Acc_POS_n = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_ib;

  /* Sum: '<S21>/Difference Inputs1'
   *
   * Block description for '<S21>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Acc_POS = rtb_deltafalllimit_om - Acc_POS_n;

  /* SampleTimeMath: '<S21>/sample time'
   *
   * About '<S21>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S21>/delta rise limit' */
  rtb_Add7 = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S67>/LowerRelop1' */
  rtb_AND_l = (rtb_Acc_POS > rtb_Add7);

  /* Switch: '<S67>/Switch2' */
  if (!rtb_AND_l) {
    /* Product: '<S21>/delta fall limit' */
    rtb_deltafalllimit_om = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S67>/UpperRelop' */
    rtb_ignition_e = (rtb_Acc_POS < rtb_deltafalllimit_om);

    /* Switch: '<S67>/Switch' */
    if (rtb_ignition_e) {
      rtb_Acc_POS = rtb_deltafalllimit_om;
    }

    /* End of Switch: '<S67>/Switch' */
    rtb_Add7 = rtb_Acc_POS;
  }

  /* End of Switch: '<S67>/Switch2' */

  /* Saturate: '<S7>/Saturation4' incorporates:
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
  VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_ib = rtb_Add7 + Acc_POS_n;
  if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_ib > 21.0F) {
    AMKTrqFL_cmd = 21.0F;
  } else if (VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_ib < -21.0F) {
    AMKTrqFL_cmd = -21.0F;
  } else {
    AMKTrqFL_cmd = VehCtrlMdel241025_2018b_amks_DW.DelayInput2_DSTATE_ib;
  }

  /* End of Saturate: '<S7>/Saturation4' */

  /* Sum: '<S10>/Add3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay1'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_c = WhlSpdFR - WhlSpdFL;

  /* SignalConversion generated from: '<S7>/Constant13' incorporates:
   *  Constant: '<S7>/Constant13'
   */
  VehCtrlMdel241025_2018b_amksp_B.VCU_SpdCmd_Emrax = 4200.0F;

  /* Update for UnitDelay: '<S71>/Unit Delay' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_pl = rtb_Compare;

  /* Update for UnitDelay: '<S7>/Unit Delay' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay_DSTATE_n =
    VehCtrlMdel241025_2018b_amksp_B.DYC_Enable_OUT;

  /* Update for UnitDelay: '<S10>/Unit Delay2' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay2_DSTATE_j = rtb_CastToBoolean;

  /* Update for UnitDelay: '<S10>/Unit Delay6' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay6_DSTATE_b =
    VehCtrlMdel241025_2018b_amks_DW.UnitDelay3_DSTATE_i;

  /* Update for UnitDelay: '<S85>/Delay Input1'
   *
   * Block description for '<S85>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput1_DSTATE_j = rtb_LowerRelop1_b;

  /* Update for UnitDelay: '<S31>/Unit Delay5' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay5_DSTATE_i = rtb_deltafalllimit_iz;

  /* Update for UnitDelay: '<S30>/Unit Delay5' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay5_DSTATE_ip = rtb_Gain3_o;

  /* Update for UnitDelay: '<S30>/Unit Delay1' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_g = FRWhlStrAng;

  /* Update for UnitDelay: '<S76>/Delay Input1'
   *
   * Block description for '<S76>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput1_DSTATE_b = rtb_LogicalOperator3;

  /* Update for UnitDelay: '<S32>/Unit Delay5' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay5_DSTATE_l = rtb_StrWhlAngV_c;

  /* Update for UnitDelay: '<S32>/Unit Delay1' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay1_DSTATE_gu = rtb_deltafalllimit_n;

  /* Update for UnitDelay: '<S96>/Delay Input1'
   *
   * Block description for '<S96>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel241025_2018b_amks_DW.DelayInput1_DSTATE_e = rtb_LogicalOperator2;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/10ms1' */

  /* S-Function (fcncallgen): '<S5>/10ms2' incorporates:
   *  SubSystem: '<S5>/VCU2AMKMCUFL'
   */
  /* Switch: '<S359>/Switch1' incorporates:
   *  Constant: '<S359>/Constant1'
   *  Constant: '<S359>/Constant2'
   *  Switch: '<S359>/Switch'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.MCFL_TorqueOn) {
    VehCtrlMdel241025_2018b_amksp_B.Switch1_l = -21.0;
    VehCtrlMdel241025_2018b_amksp_B.MCFL_TorqueLimitP = 21.0;
  } else {
    VehCtrlMdel241025_2018b_amksp_B.Switch1_l = 0.0;
    VehCtrlMdel241025_2018b_amksp_B.MCFL_TorqueLimitP = 0.0;
  }

  /* End of Switch: '<S359>/Switch1' */

  /* S-Function (scanpack): '<S359>/CAN Pack1' */
  /* S-Function (scanpack): '<S359>/CAN Pack1' */
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.ID = 386U;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Length = 8U;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Extended = 0U;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Remote = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[0] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[1] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[2] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[3] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[4] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[5] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[6] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[7] = 0;

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[2] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[2] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[3] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[3] | (uint8_T)
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
        real64_T result = VehCtrlMdel241025_2018b_amksp_B.Switch1_l;

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[6] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[6] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[7] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[7] | (uint8_T)
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
        real64_T result = VehCtrlMdel241025_2018b_amksp_B.MCFL_TorqueLimitP;

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[4] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[4] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[5] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[5] | (uint8_T)
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
          (VehCtrlMdel241025_2018b_amksp_B.MCFL_DCOn_setpoints_o);

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[1] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[1] | (uint8_T)
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
          (VehCtrlMdel241025_2018b_amksp_B.MCFL_DCEnable);

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[1] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[1] | (uint8_T)
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
        real64_T result = VehCtrlMdel241025_2018b_amksp_B.errorReset;

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[1] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[1] | (uint8_T)
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
          (VehCtrlMdel241025_2018b_amksp_B.AMKMCFL_InverterOn);

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[1] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[1] | (uint8_T)
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

  /* S-Function (ecucoder_canmessage): '<S359>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[0]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[1]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[2]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[3]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[4]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[5]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[6]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[7]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_d.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S359>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN1BUF8TX[8];
    uint8 can1buf8looptx= 0;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[0];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[1];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[2];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[3];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[4];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[5];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[6];
    can1buf8looptx++;
    CAN1BUF8TX[can1buf8looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_h[7];
    can1buf8looptx++;
    VehCtrlMdel241025_2018b_amksp_B.CANTransmit_c= ec_can_transmit(1, 8, 0, 386U,
      8, CAN1BUF8TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms2' */

  /* S-Function (fcncallgen): '<S5>/10ms4' incorporates:
   *  SubSystem: '<S5>/VCU2AMKMCUFR'
   */
  /* Switch: '<S360>/Switch1' incorporates:
   *  Constant: '<S360>/Constant'
   *  Constant: '<S360>/Constant1'
   *  Switch: '<S360>/Switch'
   */
  if (VehCtrlMdel241025_2018b_amksp_B.MCFR_TorqueOn) {
    VehCtrlMdel241025_2018b_amksp_B.Switch1 = -21.0;
    VehCtrlMdel241025_2018b_amksp_B.Switch = 21.0;
  } else {
    VehCtrlMdel241025_2018b_amksp_B.Switch1 = 0.0;
    VehCtrlMdel241025_2018b_amksp_B.Switch = 0.0;
  }

  /* End of Switch: '<S360>/Switch1' */

  /* S-Function (scanpack): '<S360>/CAN Pack1' */
  /* S-Function (scanpack): '<S360>/CAN Pack1' */
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.ID = 387U;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Length = 8U;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Extended = 0U;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Remote = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[0] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[1] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[2] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[3] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[4] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[5] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[6] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[7] = 0;

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
        real32_T result = AMKTrqFR_cmd;

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[2] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[2] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[3] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[3] | (uint8_T)
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
        real64_T result = VehCtrlMdel241025_2018b_amksp_B.Switch1;

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[6] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[6] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[7] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[7] | (uint8_T)
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
        real64_T result = VehCtrlMdel241025_2018b_amksp_B.Switch;

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[4] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[4] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[5] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[5] | (uint8_T)
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
          (VehCtrlMdel241025_2018b_amksp_B.MCFL_DCOn_setpoints_o);

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[1] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[1] | (uint8_T)
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
          (VehCtrlMdel241025_2018b_amksp_B.MCFL_DCEnable);

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[1] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[1] | (uint8_T)
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
        real64_T result = VehCtrlMdel241025_2018b_amksp_B.errorReset;

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[1] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[1] | (uint8_T)
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
          (VehCtrlMdel241025_2018b_amksp_B.AMKMCFL_InverterOn);

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[1] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[1] | (uint8_T)
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

  /* S-Function (ecucoder_canmessage): '<S360>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[0]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[1]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[2]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[3]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[4]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[5]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[6]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[7]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S360>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN1BUF9TX[8];
    uint8 can1buf9looptx= 0;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[0];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[1];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[2];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[3];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[4];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[5];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[6];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel241025_2018b_amksp_B.CANPackMessage_f[7];
    can1buf9looptx++;
    VehCtrlMdel241025_2018b_amksp_B.CANTransmit_l= ec_can_transmit(1, 9, 0, 387U,
      8, CAN1BUF9TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms4' */

  /* S-Function (fcncallgen): '<S5>/50ms3' incorporates:
   *  SubSystem: '<S5>/VCU2EmraxMCU'
   */
  /* Switch: '<S361>/Switch2' incorporates:
   *  Constant: '<S361>/Constant13'
   *  Constant: '<S361>/Constant17'
   *  Constant: '<S361>/Constant19'
   *  Constant: '<S361>/Constant20'
   *  Switch: '<S361>/Switch3'
   */
  if (TrqR_cmd_raw) {
    Gear_Trs = 0.0;
    Mode_Trs = 0.0;
  } else {
    Gear_Trs = 2.0;
    Mode_Trs = 2.0;
  }

  /* End of Switch: '<S361>/Switch2' */

  /* DataTypeConversion: '<S361>/Cast To Boolean4' */
  VehCtrlMdel241025_2018b_amksp_B.CastToBoolean4 = (real32_T)Gear_Trs;

  /* DataTypeConversion: '<S361>/Cast To Boolean6' */
  VehCtrlMdel241025_2018b_amksp_B.CastToBoolean6 = (real32_T)Mode_Trs;

  /* DataTypeConversion: '<S361>/Data Type Conversion2' */
  VehCtrlMdel241025_2018b_amksp_B.DataTypeConversion2 = (int32_T)floorf
    (EmraxTrqR_cmd);

  /* S-Function (scanpack): '<S361>/CAN Pack1' */
  /* S-Function (scanpack): '<S361>/CAN Pack1' */
  VehCtrlMdel241025_2018b_amksp_B.CANPack1.ID = 146927393U;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1.Length = 8U;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1.Extended = 1U;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1.Remote = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[0] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[1] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[2] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[3] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[4] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[5] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[6] = 0;
  VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[7] = 0;

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
      real32_T outValue = 0;

      {
        real32_T result = VehCtrlMdel241025_2018b_amksp_B.CastToBoolean4;

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[4] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[4] | (uint8_T)
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
      real32_T outValue = 0;

      {
        real32_T result = VehCtrlMdel241025_2018b_amksp_B.CastToBoolean6;

        /* no scaling required */
        /* round to closest integer value for integer CAN signal */
        if (result >= 0)
          outValue = (result - floor(result) < 0.5)?floor(result):ceil(result);
        else
          outValue = (result - floor(result) <= 0.5)?floor(result):ceil(result);
      }

      {
        uint8_T packedValue;
        if (outValue > (real32_T)(3)) {
          packedValue = (uint8_T) 3;
        } else if (outValue < (real32_T)(0)) {
          packedValue = (uint8_T) 0;
        } else {
          packedValue = (uint8_T) (outValue);
        }

        {
          {
            VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[5] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[5] | (uint8_T)
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
        real32_T result = VehCtrlMdel241025_2018b_amksp_B.VCU_SpdCmd_Emrax;

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[0] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[0] | (uint8_T)
              ((uint16_T)(packedValue & (uint16_T)0xFFU));
            VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[1] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[1] | (uint8_T)
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
          (VehCtrlMdel241025_2018b_amksp_B.DataTypeConversion2);

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
            VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[2] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[2] | (uint8_T)
              ((uint16_T)(packedValue & (uint16_T)0xFFU));
            VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[3] =
              VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[3] | (uint8_T)
              ((uint16_T)((uint16_T)(packedValue & (uint16_T)0xFF00U) >> 8));
          }
        }
      }
    }
  }

  /* S-Function (ecucoder_canmessage): '<S361>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[0]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[1]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[2]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[3]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[4]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[5]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[6]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[7]=
      VehCtrlMdel241025_2018b_amksp_B.CANPack1.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S361>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN0BUF8TX[8];
    uint8 can0buf8looptx= 0;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[0];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[1];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[2];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[3];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[4];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[5];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[6];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel241025_2018b_amksp_B.CANPackMessage[7];
    can0buf8looptx++;
    VehCtrlMdel241025_2018b_amksp_B.CANTransmit_k= ec_can_transmit(0, 8, 1,
      146927393U, 8, CAN0BUF8TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S5>/50ms3' */

  /* S-Function (fcncallgen): '<S5>/10ms6' incorporates:
   *  SubSystem: '<S5>/WP_OUTPUT'
   */
  /* DataTypeConversion: '<S362>/Cast To Single1' */
  VehCtrlMdel241025_2018b_amksp_B.CastToSingle1 = (uint16_T)rtb_CastToDouble;

  /* S-Function (ec5744_pdsslbu3): '<S362>/PowerDriverSwitch(HS)' */

  /* Set level VehCtrlMdel241025_2018b_amksp_B.aWaterPumpON for the specified power driver switch */
  ec_gpio_write(83,VehCtrlMdel241025_2018b_amksp_B.aWaterPumpON);

  /* S-Function (ec5744_pdsslbu3): '<S362>/PowerDriverSwitch(HS)1' */

  /* Set level VehCtrlMdel241025_2018b_amksp_B.bWaterPumpON for the specified power driver switch */
  ec_gpio_write(55,VehCtrlMdel241025_2018b_amksp_B.bWaterPumpON);

  /* S-Function (ec5744_pdpslbu3): '<S362>/PowerDriverPWM' incorporates:
   *  Constant: '<S362>/Constant'
   */

  /* Power driver PWM output for channel 6 */
  ec_pwm_output(6,((uint16_T)1000U),
                VehCtrlMdel241025_2018b_amksp_B.CastToSingle1);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms6' */

  /* S-Function (fcncallgen): '<S379>/10ms' incorporates:
   *  SubSystem: '<S379>/daq10ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S391>/CCPDAQ' */
  ccpDaq(1);

  /* End of Outputs for S-Function (fcncallgen): '<S379>/10ms' */

  /* Update absolute time */
  /* The "clockTick3" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick3" ensures timer will not overflow during the
   * application lifespan selected.
   */
  VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3++;
}

/* Model step function for TID4 */
void VehCtrlMdel241025_2018b_amkspdlimit_step4(void) /* Sample time: [0.05s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S379>/50ms' incorporates:
   *  SubSystem: '<S379>/daq50ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S393>/CCPDAQ' */
  ccpDaq(2);

  /* End of Outputs for S-Function (fcncallgen): '<S379>/50ms' */
}

/* Model step function for TID5 */
void VehCtrlMdel241025_2018b_amkspdlimit_step5(void) /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S378>/100MS' incorporates:
   *  SubSystem: '<S378>/Function-Call Subsystem'
   */
  /* S-Function (ec5744_canreceiveslb): '<S382>/CANReceive' */

  /* Receive CAN message */
  {
    uint8 CAN2BUF1RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can2buf1looprx= 0;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive_o3_l= 278;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive_o5_l= 8;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive_o2_p= ec_can_receive(2,1,
      CAN2BUF1RX);
    VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4_i[0]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4_i[1]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4_i[2]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4_i[3]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4_i[4]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4_i[5]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4_i[6]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
    VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4_i[7]=
      CAN2BUF1RX[can2buf1looprx];
    can2buf1looprx++;
  }

  /* Call the system: <S382>/Function-Call Subsystem */

  /* Output and update for function-call system: '<S382>/Function-Call Subsystem' */
  {
    uint8_T rtb_Add;
    uint8_T rtb_Compare;

    /* Outputs for Enabled SubSystem: '<S383>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S384>/Enable'
     */
    if (VehCtrlMdel241025_2018b_amksp_B.CANReceive_o2_p > 0) {
      /* RelationalOperator: '<S385>/Compare' incorporates:
       *  Constant: '<S385>/Constant'
       */
      rtb_Add = (uint8_T)(VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4_i[0] ==
                          83);

      /* RelationalOperator: '<S386>/Compare' incorporates:
       *  Constant: '<S386>/Constant'
       */
      rtb_Compare = (uint8_T)(VehCtrlMdel241025_2018b_amksp_B.CANReceive_o4_i[5]
        == 84);

      /* Sum: '<S384>/Add' */
      rtb_Add = (uint8_T)((uint32_T)rtb_Add + rtb_Compare);

      /* RelationalOperator: '<S387>/Compare' incorporates:
       *  Constant: '<S387>/Constant'
       */
      rtb_Compare = (uint8_T)(rtb_Add == 2);

      /* If: '<S384>/If' */
      if (rtb_Compare > 0) {
        /* Outputs for IfAction SubSystem: '<S384>/If Action Subsystem' incorporates:
         *  ActionPort: '<S388>/Action Port'
         */
        /* S-Function (ec5744_bootloaderslb): '<S388>/BootLoader' */
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

        /* S-Function (ec5744_cpuresetslb): '<S388>/CPUReset' */

        /* Perform a microcontroller reset */
        MC_ME.MCTL.R = 0X00005AF0;
        MC_ME.MCTL.R = 0X0000A50F;

        /* End of Outputs for SubSystem: '<S384>/If Action Subsystem' */
      } else {
        /* Outputs for IfAction SubSystem: '<S384>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S389>/Action Port'
         */
        /* S-Function (ec5744_cantransmitslb): '<S389>/CANTransmit' incorporates:
         *  Constant: '<S389>/Constant'
         */

        /*Transmit CAN message*/
        {
          uint8 CAN2BUF9TX[1];
          uint8 can2buf9looptx= 0;
          CAN2BUF9TX[can2buf9looptx]= ((uint8_T)1U);
          can2buf9looptx++;
          VehCtrlMdel241025_2018b_amksp_B.CANTransmit= ec_can_transmit(2, 9, 0,
            593U, 1, CAN2BUF9TX);
        }

        /* End of Outputs for SubSystem: '<S384>/If Action Subsystem1' */
      }

      /* End of If: '<S384>/If' */
    }

    /* End of Outputs for SubSystem: '<S383>/Enabled Subsystem' */
  }

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S382>/CANReceive' */
  /* End of Outputs for S-Function (fcncallgen): '<S378>/100MS' */

  /* S-Function (fcncallgen): '<S379>/100ms' incorporates:
   *  SubSystem: '<S379>/daq100ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S390>/CCPDAQ' */
  ccpDaq(3);

  /* End of Outputs for S-Function (fcncallgen): '<S379>/100ms' */
}

/* Model step function for TID6 */
void VehCtrlMdel241025_2018b_amkspdlimit_step6(void) /* Sample time: [0.5s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S379>/500ms' incorporates:
   *  SubSystem: '<S379>/daq500ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S392>/CCPDAQ' */
  ccpDaq(4);

  /* End of Outputs for S-Function (fcncallgen): '<S379>/500ms' */

  /* S-Function (fcncallgen): '<S380>/500ms' incorporates:
   *  SubSystem: '<S380>/EEPROMOperation'
   */

  /* S-Function (ec5744_eepromoslb): '<S395>/EEPROMOperatin' */
#if defined EC_EEPROM_ENABLE

  /* Operate the EEPROM module on the MPC5744 */
  ec_flash_operation();

#endif

  /* End of Outputs for S-Function (fcncallgen): '<S380>/500ms' */
}

/* Model step wrapper function for compatibility with a static main program */
void VehCtrlMdel241025_2018b_amkspdlimit_step(int_T tid)
{
  switch (tid) {
   case 0 :
    VehCtrlMdel241025_2018b_amkspdlimit_step0();
    break;

   case 1 :
    VehCtrlMdel241025_2018b_amkspdlimit_step1();
    break;

   case 2 :
    VehCtrlMdel241025_2018b_amkspdlimit_step2();
    break;

   case 3 :
    VehCtrlMdel241025_2018b_amkspdlimit_step3();
    break;

   case 4 :
    VehCtrlMdel241025_2018b_amkspdlimit_step4();
    break;

   case 5 :
    VehCtrlMdel241025_2018b_amkspdlimit_step5();
    break;

   case 6 :
    VehCtrlMdel241025_2018b_amkspdlimit_step6();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void VehCtrlMdel241025_2018b_amkspdlimit_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* Start for S-Function (fcncallgen): '<S3>/10ms6' incorporates:
   *  SubSystem: '<S3>/EMRAXMCU_RECIEVE'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S127>/CANReceive1' incorporates:
   *  SubSystem: '<S127>/MCU_pwr'
   */
  /* Start for function-call system: '<S127>/MCU_pwr' */

  /* Start for Enabled SubSystem: '<S182>/MCU_VCUMeter1' */

  /* Start for S-Function (scanunpack): '<S184>/CAN Unpack' */

  /*-----------S-Function Block: <S184>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S182>/MCU_VCUMeter1' */
  ec_buffer_init(0,1,1,218089455);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S127>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S127>/CANReceive3' incorporates:
   *  SubSystem: '<S127>/MCU_state'
   */
  /* Start for function-call system: '<S127>/MCU_state' */

  /* Start for Enabled SubSystem: '<S183>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S190>/CAN Unpack' */

  /*-----------S-Function Block: <S190>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S183>/MCU_state' */
  ec_buffer_init(0,0,1,218089199);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S127>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S123>/CANReceive3' incorporates:
   *  SubSystem: '<S123>/ABS_BUS_state'
   */
  /* Start for function-call system: '<S123>/ABS_BUS_state' */

  /* Start for Enabled SubSystem: '<S131>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S132>/CAN Unpack1' */

  /*-----------S-Function Block: <S132>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S131>/IMU_state' */
  ec_buffer_init(0,50,0,1698);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S123>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S129>/CANReceive3' incorporates:
   *  SubSystem: '<S129>/StrWhSnis_state'
   */
  /* Start for function-call system: '<S129>/StrWhSnis_state' */

  /* Start for Enabled SubSystem: '<S202>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S203>/CAN Unpack1' */

  /*-----------S-Function Block: <S203>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S202>/IMU_state' */
  ec_buffer_init(0,32,0,330);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S129>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms4' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S137>/CANReceive3' incorporates:
   *  SubSystem: '<S137>/AMKMCU_state'
   */
  /* Start for function-call system: '<S137>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S139>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S142>/CAN Unpack' */

  /*-----------S-Function Block: <S142>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S139>/MCU_state' */
  ec_buffer_init(1,1,0,640);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S137>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S137>/CANReceive1' incorporates:
   *  SubSystem: '<S137>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S137>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S140>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S152>/CAN Unpack' */

  /*-----------S-Function Block: <S152>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S140>/MCU_state' */
  ec_buffer_init(1,2,0,642);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S137>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S137>/CANReceive2' incorporates:
   *  SubSystem: '<S137>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S137>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S141>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S154>/CAN Unpack' */

  /*-----------S-Function Block: <S154>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S141>/MCU_state' */
  ec_buffer_init(1,3,0,644);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S137>/CANReceive2' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S138>/CANReceive3' incorporates:
   *  SubSystem: '<S138>/AMKMCU_state'
   */
  /* Start for function-call system: '<S138>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S158>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S161>/CAN Unpack' */

  /*-----------S-Function Block: <S161>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S158>/MCU_state' */
  ec_buffer_init(1,4,0,641);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S138>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S138>/CANReceive1' incorporates:
   *  SubSystem: '<S138>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S138>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S159>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S170>/CAN Unpack' */

  /*-----------S-Function Block: <S170>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S159>/MCU_state' */
  ec_buffer_init(1,5,0,643);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S138>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S138>/CANReceive2' incorporates:
   *  SubSystem: '<S138>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S138>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S160>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S172>/CAN Unpack' */

  /*-----------S-Function Block: <S172>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S160>/MCU_state' */
  ec_buffer_init(1,0,0,645);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S138>/CANReceive2' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms5' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S128>/CANReceive3' incorporates:
   *  SubSystem: '<S128>/IMU_state'
   */
  /* Start for function-call system: '<S128>/IMU_state' */

  /* Start for Enabled SubSystem: '<S197>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S198>/CAN Unpack' */

  /*-----------S-Function Block: <S198>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S197>/MCU_state' */
  ec_buffer_init(0,27,0,513);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S128>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms2' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S126>/CANReceive3' */
  ec_buffer_init(0,3,1,408961267);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S126>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S359>/CANTransmit' */
  ec_buffer_init(1,8,0,386U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms2' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S360>/CANTransmit' */
  ec_buffer_init(1,9,0,387U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms4' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S361>/CANTransmit' */
  ec_buffer_init(0,8,1,146927393U);

  /* End of Start for S-Function (fcncallgen): '<S5>/50ms3' */

  /* Start for S-Function (fcncallgen): '<S5>/10ms6' incorporates:
   *  SubSystem: '<S5>/WP_OUTPUT'
   */
  /* Start for S-Function (ec5744_pdpslbu3): '<S362>/PowerDriverPWM' incorporates:
   *  Constant: '<S362>/Constant'
   */

  /* Initialize PWM output for channel 6 */
  SIUL2_MSCR42 = 0X02000003;           //PWM_A3

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S378>/100MS' incorporates:
   *  SubSystem: '<S378>/Function-Call Subsystem'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S382>/CANReceive' incorporates:
   *  SubSystem: '<S382>/Function-Call Subsystem'
   */
  /* Start for function-call system: '<S382>/Function-Call Subsystem' */

  /* Start for Enabled SubSystem: '<S383>/Enabled Subsystem' */
  /* Start for IfAction SubSystem: '<S384>/If Action Subsystem1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S389>/CANTransmit' incorporates:
   *  Constant: '<S389>/Constant'
   */
  ec_buffer_init(2,9,0,593U);

  /* End of Start for SubSystem: '<S384>/If Action Subsystem1' */
  /* End of Start for SubSystem: '<S383>/Enabled Subsystem' */
  ec_buffer_init(2,1,0,278);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S382>/CANReceive' */
  /* End of Start for S-Function (fcncallgen): '<S378>/100MS' */

  /* Start for S-Function (fcncallgen): '<S381>/Function-Call Generator' incorporates:
   *  SubSystem: '<S381>/CCPBackground'
   */
  /* Start for S-Function (ec5744_ccpslb): '<S396>/CCPBackground' */
  ccpInit();

  /* End of Start for S-Function (fcncallgen): '<S381>/Function-Call Generator' */

  /* Start for S-Function (ec5744_caninterruptslb1): '<S381>/ReceiveandTransmitInterrupt' incorporates:
   *  SubSystem: '<S381>/CCPReceive'
   */
  /* Start for function-call system: '<S381>/CCPReceive' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S397>/CANReceive' */
  ec_buffer_init(2,0,0,CCP_CRO_ID);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S397>/CANReceive' */
  ec_bufint_init(2,0);
  INTC_0.PSR[548].B.PRIN = 12;
  IntcIsrVectorTable[548] = (uint32_t)&ISR_FlexCAN_2_MB0;

  /* End of Start for S-Function (ec5744_caninterruptslb1): '<S381>/ReceiveandTransmitInterrupt' */

  /* SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  /* InitializeConditions for UnitDelay: '<S288>/Unit Delay4' */
  VehCtrlMdel241025_2018b_amks_DW.UnitDelay4_DSTATE_mn = 0.01F;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' */

  /* SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* SystemInitialize for Chart: '<S113>/Chart2' */
  VehCtrlMdel241025_2018b_amks_DW.sfEvent = -1;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S4>/10ms' incorporates:
   *  SubSystem: '<S4>/Function-Call Subsystem'
   */
  VehCtrlMdel241025_2018b_amks_DW.FunctionCallSubsystem_RESET_ELA = true;

  /* Enable for Chart: '<S221>/Chart' */
  VehCtrlMdel241025_2018b_amks_DW.previousTicks_j =
    VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;

  /* End of Enable for S-Function (fcncallgen): '<S4>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  VehCtrlMdel241025_2018b_amks_DW.Subsystem_RESET_ELAPS_T = true;

  /* End of Enable for S-Function (fcncallgen): '<S4>/10ms1' */

  /* Enable for S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* Enable for Chart: '<S113>/Chart2' */
  VehCtrlMdel241025_2018b_amks_DW.previousTicks_g =
    VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;

  /* End of Enable for S-Function (fcncallgen): '<S2>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S5>/10ms1' incorporates:
   *  SubSystem: '<S5>/Beeper'
   */
  /* Enable for Chart: '<S358>/Chart' */
  VehCtrlMdel241025_2018b_amks_DW.previousTicks_m =
    VehCtrlMdel241025_2018b_amks_M->Timing.clockTick3;

  /* End of Enable for S-Function (fcncallgen): '<S5>/10ms1' */

  /* Enable for S-Function (fcncallgen): '<S1>/10ms1' incorporates:
   *  SubSystem: '<S1>/MoTrqReq'
   */
  VehCtrlMdel241025_2018b_amks_DW.MoTrqReq_RESET_ELAPS_T = true;

  /* End of Enable for S-Function (fcncallgen): '<S1>/10ms1' */
}

/* File trailer for ECUCoder generated file VehCtrlMdel241025_2018b_amkspdlimit.c.
 *
 * [EOF]
 */
