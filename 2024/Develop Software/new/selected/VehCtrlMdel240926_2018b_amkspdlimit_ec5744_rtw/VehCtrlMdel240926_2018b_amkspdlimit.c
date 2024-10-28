/*
 * Code generated for Simulink model VehCtrlMdel240926_2018b_amkspdlimit.
 *
 * FILE    : VehCtrlMdel240926_2018b_amkspdlimit.c
 *
 * VERSION : 1.301
 *
 * DATE    : Thu Oct 24 15:52:09 2024
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

/* Named constants for Chart: '<S125>/Timer' */
#define VehCtrlMdel240926_2018_IN_Out_n ((uint8_T)2U)
#define VehCtrlMdel240926__IN_Trigger_c ((uint8_T)3U)
#define VehCtrlMdel2409_IN_InterState_n ((uint8_T)1U)

/* Named constants for Chart: '<S7>/Chart' */
#define VehCtrlMd_IN_NO_ACTIVE_CHILD_kx ((uint8_T)0U)
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

/* Named constants for Chart: '<S108>/Chart2' */
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

/* Named constants for Chart: '<S125>/Timer1' */
#define VehCtrlMdel240926_2018_IN_Out_b ((uint8_T)2U)
#define VehCtrlMdel240926__IN_Trigger_e ((uint8_T)3U)
#define VehCtrlMdel2409_IN_InterState_d ((uint8_T)1U)

/* Named constants for Chart: '<S215>/Chart' */
#define VehCtrlMdel240926_2018b_IN_ON_d ((uint8_T)1U)
#define VehCtrlMdel240926_20_IN_STATEON ((uint8_T)2U)
#define VehCtrlMdel240926_2_IN_STATEOFF ((uint8_T)1U)
#define VehCtrlMdel240926__IN_initstate ((uint8_T)2U)

/* Named constants for Chart: '<S352>/Chart' */
#define VehCtrlMdel2409_IN_initstate1_c ((uint8_T)2U)

/* Named constants for Chart: '<S359>/Chart' */
#define VehCtrlMdel240926_2018_IN_LEDON ((uint8_T)2U)
#define VehCtrlMdel240926_201_IN_Init_e ((uint8_T)1U)
#define VehCtrlMdel240926_201_IN_LEDOFF ((uint8_T)1U)
#define VehCtrlMdel240926_201_IN_StateA ((uint8_T)2U)
#define VehCtrlMdel240926_201_IN_StateB ((uint8_T)3U)
#define VehCtrlMdel240926_201_IN_StateC ((uint8_T)4U)

boolean L9826VAR701[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

boolean L9826DIAG701[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

boolean L9826VAR702[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

boolean L9826DIAG702[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

/* Exported block signals */
real_T Gear_Trs;                       /* '<S355>/Switch2' */
real_T Mode_Trs;                       /* '<S355>/Switch3' */
real_T KeyPressed;     /* '<S209>/BusConversion_InsertedFor_Out1_at_inport_0' */
real_T AMKFL_Current;                  /* '<S214>/Switch' */
real_T AMKFR_Current;                  /* '<S214>/Switch1' */
real_T EmraxPwr;                       /* '<S214>/Product2' */
real_T Trq_CUT;                        /* '<S211>/Timer' */
real_T AMKSWITCH;                      /* '<S125>/Timer1' */
real_T ignition;                       /* '<S125>/Timer' */
real_T L12V_error;                     /* '<S185>/CAN Unpack' */
real_T alarm;                          /* '<S185>/CAN Unpack' */
real_T controller_ready;               /* '<S185>/CAN Unpack' */
real_T selfcheck;                      /* '<S185>/CAN Unpack' */
real_T RPM;                            /* '<S185>/CAN Unpack' */
real_T trq;                            /* '<S185>/CAN Unpack' */
real_T AC_current;                     /* '<S179>/CAN Unpack' */
real_T DC_current;                     /* '<S179>/CAN Unpack' */
real_T MCU_Temp;                       /* '<S179>/CAN Unpack' */
real_T motor_Temp;                     /* '<S179>/CAN Unpack' */
real_T voltage;                        /* '<S179>/CAN Unpack' */
real_T MCFR_ActualTorque;              /* '<S156>/CAN Unpack' */
real_T MCFR_ActualVelocity;            /* '<S156>/CAN Unpack' */
real_T MCFR_DCVoltage;                 /* '<S156>/CAN Unpack' */
real_T MCFR_bDCOn;                     /* '<S156>/CAN Unpack' */
real_T MCFR_bError;                    /* '<S156>/CAN Unpack' */
real_T MCFR_bInverterOn;               /* '<S156>/CAN Unpack' */
real_T MCFR_bQuitInverterOn;           /* '<S156>/CAN Unpack' */
real_T MCFR_bSystemReady;              /* '<S156>/CAN Unpack' */
real_T MCFR_TempIGBT;                  /* '<S167>/CAN Unpack' */
real_T MCFR_TempInverter;              /* '<S167>/CAN Unpack' */
real_T MCFR_TempMotor;                 /* '<S167>/CAN Unpack' */
real_T MCFR_ErrorInfo;                 /* '<S165>/CAN Unpack' */
real_T MCFL_ActualTorque;              /* '<S137>/CAN Unpack' */
real_T MCFL_ActualVelocity;            /* '<S137>/CAN Unpack' */
real_T MCFL_DCVoltage;                 /* '<S137>/CAN Unpack' */
real_T MCFL_bDCOn;                     /* '<S137>/CAN Unpack' */
real_T MCFL_bError;                    /* '<S137>/CAN Unpack' */
real_T MCFL_bInverterOn;               /* '<S137>/CAN Unpack' */
real_T MCFL_bQuitDCOn;                 /* '<S137>/CAN Unpack' */
real_T MCFL_bQuitInverterOn;           /* '<S137>/CAN Unpack' */
real_T MCFL_bSystemReady;              /* '<S137>/CAN Unpack' */
real_T MCFL_TempIGBT;                  /* '<S149>/CAN Unpack' */
real_T MCFL_TempInverter;              /* '<S149>/CAN Unpack' */
real_T MCFL_TempMotor;                 /* '<S149>/CAN Unpack' */
real_T MCFL_ErrorInfo;                 /* '<S147>/CAN Unpack' */
real_T StrWhlAngAliveRollCnt;          /* '<S198>/CAN Unpack1' */
real_T StrWhlAng;                      /* '<S198>/CAN Unpack1' */
real_T StrWhlAngV;                     /* '<S198>/CAN Unpack1' */
real_T ABS_WS_FL;                      /* '<S127>/CAN Unpack1' */
real_T ABS_WS_FR;                      /* '<S127>/CAN Unpack1' */
real_T ABS_WS_RL;                      /* '<S127>/CAN Unpack1' */
real_T ABS_WS_RR;                      /* '<S127>/CAN Unpack1' */
real_T IMU_Ay_Value;                   /* '<S193>/CAN Unpack' */
real_T IMU_Ax_Value;                   /* '<S193>/CAN Unpack' */
real_T IMU_Yaw_Value;                  /* '<S193>/CAN Unpack' */
real_T EMRAX_Trq_CUT;                  /*  */
real_T AMK_Trq_CUT;                    /*  */
uint32_T Acc_vol2;                     /* '<S211>/Add3' */
uint32_T Acc_vol;                      /* '<S211>/Add2' */
uint32_T Acc_POS;                      /* '<S211>/1-D Lookup Table4' */
uint32_T Acc_POS2;                     /* '<S211>/1-D Lookup Table3' */
real32_T VehVxEst_mps;                 /* '<S339>/Add' */
real32_T PwrALL;                       /* '<S28>/Gain3' */
real32_T EmraxTrqR_cmd;                /* '<S7>/Saturation1' */
real32_T AMKTrqFR_cmd;                 /* '<S7>/Saturation3' */
real32_T AMKTrqFL_cmd;                 /* '<S7>/Saturation4' */
uint16_T F_BrkPrs;                     /* '<S211>/1-D Lookup Table1' */
uint16_T Acc1;                         /* '<S120>/Acc3' */
uint16_T Acc2;                         /* '<S120>/Acc4' */
uint16_T Brk1;                         /* '<S120>/Brk1' */
uint16_T Brk2;                         /* '<S120>/Brk2' */
boolean_T STATEDISPLAY;                /* '<S352>/Switch1' */
boolean_T HVSWITCH;                    /* '<S352>/Chart' */
boolean_T TSAL_SW_IN;                  /* '<S125>/SwitchInput2' */
boolean_T HV_voltValid;                /* '<S202>/Compare' */
boolean_T Brk;                         /* '<S110>/Compare' */
boolean_T ACC_Release;                 /* '<S111>/Compare' */
boolean_T beeper_state;                /* '<S108>/Chart2' */
boolean_T MCFL_DCOn_setpoints;         /* '<S108>/Chart2' */
boolean_T MCFR_DCEnable;               /* '<S108>/Chart2' */
boolean_T MCFR_InverterOn;             /* '<S108>/Chart2' */
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
  const boolean_T *AND_n, const real_T *Switch_k, const real_T *Switch3, const
  real_T *Switch10);
static void VehCtrlMdel240926_20_AMKDCready(const real_T *MCFL_bDCOn_j, const
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

void ISR_FlexCAN_2_MB0(void)
{
  /* Call the system: <S371>/CCPReceive */
  {
    /* S-Function (ec5744_caninterruptslb1): '<S371>/ReceiveandTransmitInterrupt' */

    /* Output and update for function-call system: '<S371>/CCPReceive' */

    /* S-Function (ec5744_canreceiveslb): '<S387>/CANReceive' */

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

    /* Nothing to do for system: <S387>/Nothing */

    /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S387>/CANReceive' */

    /* End of Outputs for S-Function (ec5744_caninterruptslb1): '<S371>/ReceiveandTransmitInterrupt' */
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
 *    '<S213>/Timer'
 *    '<S215>/Timer'
 *    '<S216>/Timer'
 *    '<S216>/Timer1'
 *    '<S216>/Timer2'
 *    '<S216>/Timer3'
 *    '<S281>/Timer'
 *    '<S281>/Timer1'
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
 *    '<S125>/Timer'
 *    '<S211>/Timer'
 */
void VehCtrlMdel240926_201_Timer(boolean_T rtu_Trigger, real32_T rtu_CountTime,
  real_T *rty_Exit, DW_Timer_VehCtrlMdel240926_20_T *localDW)
{
  boolean_T sf_internal_predicateOutput;

  /* Chart: '<S125>/Timer' */
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

  /* End of Chart: '<S125>/Timer' */
}

/* Function for Chart: '<S108>/Chart2' */
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

/* Function for Chart: '<S108>/Chart2' */
static void VehCtrlMdel240926_2018b_VehStat(const real_T *controller_ready_e,
  const boolean_T *AND_n, const real_T *Switch_k, const real_T *Switch3, const
  real_T *Switch10)
{
  boolean_T sf_internal_predicateOutput;
  int32_T b_previousEvent;
  switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat) {
   case VehCtrlMdel240926_2018_IN_Guard:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_f >= 25U) {
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
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_f = 0U;
    } else {
      sf_internal_predicateOutput = ((!*AND_n) || (!Brk) || (!ACC_Release));
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
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_f >= 10U) {
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
      sf_internal_predicateOutput = ((*AND_n) && Brk && ACC_Release &&
        (*controller_ready_e != 0.0) && (*Switch_k != 0.0) && (*Switch3 != 0.0) &&
        (*Switch10 != 0.0) && (VehCtrlMdel240926_2018b_amksp_B.Switch11 != 0.0));
      if (sf_internal_predicateOutput) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_VehStat = 1U;
        VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_f = 0U;
      }
    }
    break;

   case VehCtrlMdel240926_2018_IN_Trans:
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_f >= 250U) {
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

/* Function for Chart: '<S108>/Chart2' */
static void VehCtrlMdel240926_20_AMKDCready(const real_T *MCFL_bDCOn_j, const
  real_T *MCFR_bDCOn_n, const boolean_T *AND_n, const real_T *Switch_k, const
  real_T *Switch3, const real_T *Switch10)
{
  boolean_T sf_internal_predicateOutput;
  int32_T g_previousEvent;
  switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready) {
   case VehCtrlMdel240926_201_IN_AMKCAN:
    sf_internal_predicateOutput = ((*AND_n) && Brk && ACC_Release);
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
       (VehCtrlMdel240926_2018b_amksp_B.Switch11 != 0.0));
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
  /* S-Function (fcncallgen): '<S371>/Function-Call Generator' incorporates:
   *  SubSystem: '<S371>/CCPBackground'
   */

  /* S-Function (ec5744_ccpslb): '<S386>/CCPBackground' */
  ccpBackground();
  Lin0_Background();

  /* End of Outputs for S-Function (fcncallgen): '<S371>/Function-Call Generator' */
}

/* Model step function for TID2 */
void VehCtrlMdel240926_2018b_amkspdlimit_step2(void) /* Sample time: [0.005s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S369>/5ms' incorporates:
   *  SubSystem: '<S369>/daq5ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S384>/CCPDAQ' */
  ccpDaq(0);

  /* End of Outputs for S-Function (fcncallgen): '<S369>/5ms' */
}

/* Model step function for TID3 */
void VehCtrlMdel240926_2018b_amkspdlimit_step3(void) /* Sample time: [0.01s, 0.0s] */
{
  boolean_T rtb_ignition_e;
  real32_T rtb_FRWhlStrAng;
  real_T rtb_Gain5;
  real_T rtb_Gain4;
  real_T rtb_Switch2_on;
  real32_T rtb_StrWhlAngV_c;
  real32_T rtb_Gain3_o;
  boolean_T rtb_LogicalOperator2;
  boolean_T rtb_Compare;
  boolean_T rtb_LogicalOperator7_m;
  boolean_T rtb_Compare_am;
  boolean_T rtb_LowerRelop1_b;
  real_T elapseTime;
  real32_T rtb_CastToBoolean;
  real_T rtb_Yk1_l;
  real_T rtb_UkYk1;
  real_T rtb_UkYk1_nc;
  real_T rtb_g_mpss1;
  real_T rtb_Add1;
  real32_T rtb_CastToBoolean1;
  real32_T rtb_Add4_j;
  real32_T rtb_Add7;
  real32_T rtb_Add6;
  real32_T rtb_Switch2_mn;
  real32_T rtb_Switch2_b0;
  real32_T rtb_CastToDouble;
  boolean_T rtb_Compare_nd;
  boolean_T rtb_Compare_b;
  boolean_T rtb_AND_l;
  boolean_T rtb_AND2_e;
  boolean_T rtb_Compare_ar;
  real32_T rtb_VxIMU_est;
  real32_T rtb_Ax;
  real32_T rtb_Min1_i;
  real32_T rtb_Product2_j;
  real_T rtb_Product1;
  real_T rtb_Switch2_ow;
  real_T elapseTime_0;
  real32_T rtb_Gain26;
  real_T rtb_Add5;
  real32_T rtb_Add10;
  real32_T rtb_Add10_ey;
  real32_T rtb_Fz3;
  real32_T rtb_deltafalllimit_cz;
  real32_T rtb_deltafalllimit_ap;
  boolean_T rtb_UpperRelop_ir;
  real32_T rtb_MaxWhlSpd_mps_n;
  uint32_T rtb_Gain1_h;
  uint32_T rtb_Gain;
  uint32_T FunctionCallSubsystem_ELAPS_T;
  int32_T Brk_F;
  real_T WhlSpdFR;
  real_T WhlSpdRR_mps;
  real_T WhlSpdRL_mps;
  real32_T FLWhlStrAng;
  real32_T Acc_POS_n;
  boolean_T rtb_LogicalOperator_idx_0;

  /* S-Function (fcncallgen): '<S3>/10ms7' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* S-Function (ec5744_asislbu3): '<S125>/Acc4' */
  VehCtrlMdel240926_2018b_amksp_B.HV_volt= adc_read_chan(1,6)*57/10;

  /* S-Function (ec5744_swislbu3): '<S125>/SwitchInput' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240926_2018b_amksp_B.Drive_ready= ec_gpio_read(99);

  /* Logic: '<S125>/Logical Operator' */
  rtb_ignition_e = !VehCtrlMdel240926_2018b_amksp_B.Drive_ready;

  /* Chart: '<S125>/Timer' incorporates:
   *  Constant: '<S125>/Constant5'
   */
  VehCtrlMdel240926_201_Timer(rtb_ignition_e, 0.11F, &ignition,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer);

  /* S-Function (ec5744_swislbu3): '<S125>/SwitchInput1' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240926_2018b_amksp_B.SwitchInput1= ec_gpio_read(45);

  /* Chart: '<S125>/Timer1' incorporates:
   *  Constant: '<S125>/Constant1'
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
      if (!VehCtrlMdel240926_2018b_amksp_B.SwitchInput1) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel240926_2018b_
          = 3U;
        VehCtrlMdel240926_2018b_amks_DW.x += 0.01;
        AMKSWITCH = 0.0;
      }
      break;

     case VehCtrlMdel240926_2018_IN_Out_b:
      AMKSWITCH = 1.0;
      if (VehCtrlMdel240926_2018b_amksp_B.SwitchInput1) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel240926_2018b_
          = 3U;
        VehCtrlMdel240926_2018b_amks_DW.x += 0.01;
        AMKSWITCH = 0.0;
      }
      break;

     default:
      /* case IN_Trigger: */
      AMKSWITCH = 0.0;
      rtb_LogicalOperator2 = ((!VehCtrlMdel240926_2018b_amksp_B.SwitchInput1) &&
        (VehCtrlMdel240926_2018b_amks_DW.x < 0.10999999940395355));
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel240926_2018b_
          = 3U;
        VehCtrlMdel240926_2018b_amks_DW.x += 0.01;
        AMKSWITCH = 0.0;
      } else if (VehCtrlMdel240926_2018b_amks_DW.x >= 0.10999999940395355) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel240926_2018b_
          = 2U;
        AMKSWITCH = 1.0;
        VehCtrlMdel240926_2018b_amks_DW.x = 0.0;
      } else {
        rtb_LogicalOperator2 = ((VehCtrlMdel240926_2018b_amks_DW.x <
          0.10999999940395355) && VehCtrlMdel240926_2018b_amksp_B.SwitchInput1);
        if (rtb_LogicalOperator2) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel240926_2018b_
            = 1U;
          VehCtrlMdel240926_2018b_amks_DW.x = 0.0;
        }
      }
      break;
    }
  }

  /* End of Chart: '<S125>/Timer1' */

  /* S-Function (ec5744_swislbu3): '<S125>/SwitchInput3' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240926_2018b_amksp_B.out2_c= ec_gpio_read(92);

  /* S-Function (ec5744_swislbu3): '<S125>/SwitchInput2' */

  /* Read the the value of the specified switch input */
  TSAL_SW_IN= ec_gpio_read(152);

  /* SignalConversion generated from: '<S125>/key' */
  VehCtrlMdel240926_2018b_amksp_B.TSAL_SW_IN_i2 = TSAL_SW_IN;

  /* SignalConversion generated from: '<S125>/key' */
  VehCtrlMdel240926_2018b_amksp_B.out2_h =
    VehCtrlMdel240926_2018b_amksp_B.out2_c;

  /* SignalConversion generated from: '<S125>/key' */
  VehCtrlMdel240926_2018b_amksp_B.AMKSWITCH_bx = AMKSWITCH;

  /* SignalConversion generated from: '<S125>/key' */
  VehCtrlMdel240926_2018b_amksp_B.ignition_d = ignition;

  /* RelationalOperator: '<S202>/Compare' incorporates:
   *  Constant: '<S202>/Constant'
   */
  HV_voltValid = (VehCtrlMdel240926_2018b_amksp_B.HV_volt <= 1500);

  /* SignalConversion generated from: '<S125>/key' */
  VehCtrlMdel240926_2018b_amksp_B.HV_voltValid_kx = HV_voltValid;

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms7' */

  /* S-Function (fcncallgen): '<S3>/10ms6' incorporates:
   *  SubSystem: '<S3>/EMRAXMCU_RECIEVE'
   */
  /* S-Function (ec5744_canreceiveslb): '<S122>/CANReceive1' */

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

  /* Call the system: <S122>/MCU_pwr */

  /* Output and update for function-call system: '<S122>/MCU_pwr' */

  /* Outputs for Enabled SubSystem: '<S177>/MCU_VCUMeter1' incorporates:
   *  EnablePort: '<S179>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S179>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S179>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S179>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S177>/MCU_VCUMeter1' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' */

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

  /* Call the system: <S122>/MCU_state */

  /* Output and update for function-call system: '<S122>/MCU_state' */

  /* Outputs for Enabled SubSystem: '<S178>/MCU_state' incorporates:
   *  EnablePort: '<S185>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S185>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S185>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S185>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S178>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms6' */

  /* S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S118>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN2BUF16RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can2buf16looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_m= 1698;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o5_b= 8;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_m= ec_can_receive(2,16,
      CAN2BUF16RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[0]=
      CAN2BUF16RX[can2buf16looprx];
    can2buf16looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[1]=
      CAN2BUF16RX[can2buf16looprx];
    can2buf16looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[2]=
      CAN2BUF16RX[can2buf16looprx];
    can2buf16looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[3]=
      CAN2BUF16RX[can2buf16looprx];
    can2buf16looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[4]=
      CAN2BUF16RX[can2buf16looprx];
    can2buf16looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[5]=
      CAN2BUF16RX[can2buf16looprx];
    can2buf16looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[6]=
      CAN2BUF16RX[can2buf16looprx];
    can2buf16looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_lg[7]=
      CAN2BUF16RX[can2buf16looprx];
    can2buf16looprx++;
  }

  /* Call the system: <S118>/ABS_BUS_state */

  /* Output and update for function-call system: '<S118>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S126>/IMU_state' incorporates:
   *  EnablePort: '<S127>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_m > 0) {
    /* S-Function (ecucoder_canunmessage): '<S127>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S127>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S127>/CAN Unpack1' */
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

  /* End of Outputs for SubSystem: '<S126>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S118>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms3' */

  /* S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S124>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN2BUF7RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can2buf7looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_c= 330;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o5_d= 8;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_p= ec_can_receive(2,7,
      CAN2BUF7RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[0]=
      CAN2BUF7RX[can2buf7looprx];
    can2buf7looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[1]=
      CAN2BUF7RX[can2buf7looprx];
    can2buf7looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[2]=
      CAN2BUF7RX[can2buf7looprx];
    can2buf7looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[3]=
      CAN2BUF7RX[can2buf7looprx];
    can2buf7looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[4]=
      CAN2BUF7RX[can2buf7looprx];
    can2buf7looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[5]=
      CAN2BUF7RX[can2buf7looprx];
    can2buf7looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[6]=
      CAN2BUF7RX[can2buf7looprx];
    can2buf7looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_k[7]=
      CAN2BUF7RX[can2buf7looprx];
    can2buf7looprx++;
  }

  /* Call the system: <S124>/StrWhSnis_state */

  /* Output and update for function-call system: '<S124>/StrWhSnis_state' */

  /* Outputs for Enabled SubSystem: '<S197>/IMU_state' incorporates:
   *  EnablePort: '<S198>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S198>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S198>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S198>/CAN Unpack1' */
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

  /* End of Outputs for SubSystem: '<S197>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S124>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms4' */

  /* S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S132>/CANReceive3' */

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

  /* Call the system: <S132>/AMKMCU_state */

  /* Output and update for function-call system: '<S132>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S134>/MCU_state' incorporates:
   *  EnablePort: '<S137>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S137>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S137>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S137>/CAN Unpack' */
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
  }

  /* End of Outputs for SubSystem: '<S134>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S132>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S132>/CANReceive1' */

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

  /* Call the system: <S132>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S132>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S135>/MCU_state' incorporates:
   *  EnablePort: '<S147>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S147>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S147>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S147>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S135>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S132>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S132>/CANReceive2' */

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

  /* Call the system: <S132>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S132>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S136>/MCU_state' incorporates:
   *  EnablePort: '<S149>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S149>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S149>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S149>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S136>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S132>/CANReceive2' */

  /* S-Function (ec5744_canreceiveslb): '<S133>/CANReceive3' */

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

  /* Call the system: <S133>/AMKMCU_state */

  /* Output and update for function-call system: '<S133>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S153>/MCU_state' incorporates:
   *  EnablePort: '<S156>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_a > 0) {
    /* S-Function (ecucoder_canunmessage): '<S156>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S156>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S156>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S153>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S133>/CANReceive1' */

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

  /* Call the system: <S133>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S133>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S154>/MCU_state' incorporates:
   *  EnablePort: '<S165>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2_o > 0) {
    /* S-Function (ecucoder_canunmessage): '<S165>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S165>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S165>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S154>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S133>/CANReceive2' */

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

  /* Call the system: <S133>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S133>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S155>/MCU_state' incorporates:
   *  EnablePort: '<S167>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S167>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S167>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S167>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S155>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive2' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms5' */

  /* S-Function (fcncallgen): '<S3>/10ms' incorporates:
   *  SubSystem: '<S3>/AccBrk_BUS'
   */
  /* S-Function (ec5744_asislbu3): '<S120>/Acc3' */

  /* Read the ADC conversion result of the analog signal */
  Acc1= adc_read_chan(1,2);

  /* S-Function (ec5744_asislbu3): '<S120>/Acc4' */

  /* Read the ADC conversion result of the analog signal */
  Acc2= adc_read_chan(1,4);

  /* S-Function (ec5744_asislbu3): '<S120>/Brk1' */

  /* Read the ADC conversion result of the analog signal */
  Brk1= adc_read_chan(1,0);

  /* S-Function (ec5744_asislbu3): '<S120>/Brk2' */

  /* Read the ADC conversion result of the analog signal */
  Brk2= adc_read_chan(0,13);

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms' */

  /* S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* S-Function (ec5744_canreceiveslb): '<S123>/CANReceive3' */

  /* Receive CAN message */
  {
    uint8 CAN2BUF17RX[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

    uint8 can2buf17looprx= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o3_cz= 513;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o5_m= 8;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_ma= ec_can_receive(2,17,
      CAN2BUF17RX);
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[0]=
      CAN2BUF17RX[can2buf17looprx];
    can2buf17looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[1]=
      CAN2BUF17RX[can2buf17looprx];
    can2buf17looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[2]=
      CAN2BUF17RX[can2buf17looprx];
    can2buf17looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[3]=
      CAN2BUF17RX[can2buf17looprx];
    can2buf17looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[4]=
      CAN2BUF17RX[can2buf17looprx];
    can2buf17looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[5]=
      CAN2BUF17RX[can2buf17looprx];
    can2buf17looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[6]=
      CAN2BUF17RX[can2buf17looprx];
    can2buf17looprx++;
    VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o4_i[7]=
      CAN2BUF17RX[can2buf17looprx];
    can2buf17looprx++;
  }

  /* Call the system: <S123>/IMU_state */

  /* Output and update for function-call system: '<S123>/IMU_state' */

  /* Outputs for Enabled SubSystem: '<S192>/MCU_state' incorporates:
   *  EnablePort: '<S193>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_ma > 0) {
    /* S-Function (ecucoder_canunmessage): '<S193>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S193>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S193>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S192>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S123>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms2' */

  /* S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' */

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

  /* Call the system: <S121>/ABS_BUS_state */

  /* Output and update for function-call system: '<S121>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S175>/IMU_state' incorporates:
   *  EnablePort: '<S176>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_k > 0) {
    /* S-Function (ecucoder_canunmessage): '<S176>/CANUnPackMessage4' */

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

  /* End of Outputs for SubSystem: '<S175>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' */
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

  /* Lookup_n-D: '<S211>/1-D Lookup Table1' */
  F_BrkPrs = look1_iu16bflftfIu16_binlc(Brk1,
    VehCtrlMdel240926_2018b__ConstP.pooled73,
    VehCtrlMdel240926_2018b__ConstP.pooled73, 1U);

  /* DataTypeConversion: '<S211>/Data Type Conversion' */
  rtb_FRWhlStrAng = F_BrkPrs;

  /* SignalConversion generated from: '<S209>/Out1' */
  Brk_F = (int32_T)rtb_FRWhlStrAng;

  /* Gain: '<S211>/Gain2' */
  rtb_Gain1_h = 45875U * Acc2;

  /* Gain: '<S211>/Gain3' incorporates:
   *  UnitDelay: '<S211>/Unit Delay1'
   */
  rtb_Gain = 39322U * VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S211>/Add3' */
  Acc_vol2 = (rtb_Gain >> 1) + rtb_Gain1_h;

  /* RelationalOperator: '<S221>/Compare' */
  rtb_ignition_e = (Acc_vol2 <= 32768000U);

  /* RelationalOperator: '<S222>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol2 >= 294912000U);

  /* Logic: '<S211>/Logical Operator1' */
  rtb_ignition_e = (rtb_ignition_e || rtb_LogicalOperator2);

  /* Gain: '<S211>/Gain' */
  rtb_Gain = 45875U * Acc1;

  /* UnitDelay: '<S211>/Unit Delay' incorporates:
   *  UnitDelay: '<S211>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_k;

  /* Gain: '<S211>/Gain1' incorporates:
   *  UnitDelay: '<S211>/Unit Delay1'
   */
  rtb_Gain1_h = 39322U * VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S211>/Add2' */
  Acc_vol = (rtb_Gain1_h >> 1) + rtb_Gain;

  /* RelationalOperator: '<S217>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol <= 32768000U);

  /* RelationalOperator: '<S218>/Compare' */
  rtb_Compare = (Acc_vol >= 294912000U);

  /* Logic: '<S211>/Logical Operator' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_Compare);

  /* Logic: '<S211>/Logical Operator2' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_ignition_e);

  /* Lookup_n-D: '<S211>/1-D Lookup Table4' */
  Acc_POS = look1_iu32n16bflftfIu32_binlc(Acc_vol,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable4_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.pooled74, 1U);

  /* DataTypeConversion: '<S211>/Data Type Conversion1' */
  rtb_FRWhlStrAng = (real32_T)Acc_POS * 1.52587891E-5F;

  /* Lookup_n-D: '<S211>/1-D Lookup Table3' */
  Acc_POS2 = look1_iu32n16bflftfIu32_binlc(Acc_vol2,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable3_bp01Data_b,
    VehCtrlMdel240926_2018b__ConstP.pooled74, 1U);

  /* DataTypeConversion: '<S211>/Data Type Conversion4' */
  rtb_Gain3_o = (real32_T)Acc_POS2 * 1.52587891E-5F;

  /* Sum: '<S211>/Add1' */
  rtb_StrWhlAngV_c = rtb_FRWhlStrAng - rtb_Gain3_o;

  /* Abs: '<S211>/Abs' */
  rtb_StrWhlAngV_c = fabsf(rtb_StrWhlAngV_c);

  /* RelationalOperator: '<S225>/Compare' incorporates:
   *  Constant: '<S225>/Constant'
   */
  rtb_Compare = (rtb_StrWhlAngV_c > 10.0F);

  /* RelationalOperator: '<S223>/Compare' incorporates:
   *  Constant: '<S223>/Constant'
   */
  rtb_ignition_e = (rtb_FRWhlStrAng > 100.0F);

  /* RelationalOperator: '<S224>/Compare' incorporates:
   *  Constant: '<S224>/Constant'
   */
  rtb_LogicalOperator7_m = (rtb_Gain3_o > 100.0F);

  /* Logic: '<S211>/Logical Operator3' */
  rtb_ignition_e = (rtb_ignition_e || rtb_LogicalOperator7_m);

  /* RelationalOperator: '<S226>/Compare' incorporates:
   *  Constant: '<S226>/Constant'
   */
  rtb_LogicalOperator7_m = (Brk1 <= 300);

  /* RelationalOperator: '<S227>/Compare' incorporates:
   *  Constant: '<S227>/Constant'
   */
  rtb_Compare_am = (Brk1 >= 4500);

  /* Logic: '<S211>/Logical Operator5' */
  rtb_LogicalOperator7_m = (rtb_LogicalOperator7_m || rtb_Compare_am);

  /* RelationalOperator: '<S219>/Compare' incorporates:
   *  Constant: '<S219>/Constant'
   */
  rtb_Compare_am = (Brk2 <= 300);

  /* RelationalOperator: '<S220>/Compare' incorporates:
   *  Constant: '<S220>/Constant'
   */
  rtb_LowerRelop1_b = (Brk2 >= 4500);

  /* Logic: '<S211>/Logical Operator6' */
  rtb_Compare_am = (rtb_Compare_am || rtb_LowerRelop1_b);

  /* Logic: '<S211>/Logical Operator7' */
  rtb_LogicalOperator7_m = (rtb_LogicalOperator7_m || rtb_Compare_am);

  /* Logic: '<S211>/Logical Operator4' */
  rtb_ignition_e = (rtb_ignition_e || rtb_Compare || rtb_LogicalOperator2 ||
                    rtb_LogicalOperator7_m);

  /* Chart: '<S211>/Timer' incorporates:
   *  Constant: '<S211>/Constant1'
   */
  VehCtrlMdel240926_201_Timer(rtb_ignition_e, 0.11F, &Trq_CUT,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_a);

  /* UnitDelay: '<S247>/Delay Input2'
   *
   * Block description for '<S247>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_StrWhlAngV_c = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2;

  /* SampleTimeMath: '<S247>/sample time'
   *
   * About '<S247>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S247>/delta rise limit' */
  rtb_FRWhlStrAng = (real32_T)(1200.0 * elapseTime);

  /* DataTypeConversion: '<S213>/Cast To Boolean' */
  rtb_CastToBoolean = (real32_T)StrWhlAng;

  /* Sum: '<S247>/Difference Inputs1'
   *
   * Block description for '<S247>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_CastToBoolean -= rtb_StrWhlAngV_c;

  /* RelationalOperator: '<S250>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_CastToBoolean > rtb_FRWhlStrAng);

  /* Switch: '<S250>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S247>/delta fall limit' */
    rtb_FRWhlStrAng = (real32_T)(-1200.0 * elapseTime);

    /* RelationalOperator: '<S250>/UpperRelop' */
    rtb_ignition_e = (rtb_CastToBoolean < rtb_FRWhlStrAng);

    /* Switch: '<S250>/Switch' */
    if (rtb_ignition_e) {
      rtb_CastToBoolean = rtb_FRWhlStrAng;
    }

    /* End of Switch: '<S250>/Switch' */
    rtb_FRWhlStrAng = rtb_CastToBoolean;
  }

  /* End of Switch: '<S250>/Switch2' */

  /* Sum: '<S247>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S247>/Delay Input2'
   *
   * Block description for '<S247>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S247>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2 = rtb_FRWhlStrAng +
    rtb_StrWhlAngV_c;

  /* Abs: '<S213>/Abs' incorporates:
   *  UnitDelay: '<S247>/Delay Input2'
   *
   * Block description for '<S247>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_StrWhlAngV_c = fabsf(VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2);

  /* RelationalOperator: '<S246>/Compare' incorporates:
   *  Constant: '<S246>/Constant'
   */
  rtb_ignition_e = (rtb_StrWhlAngV_c > 120.0F);

  /* Chart: '<S213>/Timer' incorporates:
   *  Constant: '<S213>/Constant5'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition_e, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_on,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_k);

  /* Logic: '<S215>/OR' */
  rtb_ignition_e = ((MCFL_bError != 0.0) || (MCFR_bError != 0.0));

  /* Chart: '<S215>/Timer' incorporates:
   *  Constant: '<S215>/Constant1'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition_e, 0.2F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_n,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_p);

  /* RelationalOperator: '<S258>/Compare' incorporates:
   *  Constant: '<S258>/Constant'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amksp_B.Exit_n > 0.0);

  /* UnitDelay: '<S256>/Delay Input1'
   *
   * Block description for '<S256>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_a;

  /* RelationalOperator: '<S256>/FixPt Relational Operator' */
  rtb_Compare_am = ((int32_T)rtb_Compare > (int32_T)rtb_LowerRelop1_b);

  /* Chart: '<S215>/Chart' */
  rtb_Gain1_h = VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3 -
    VehCtrlMdel240926_2018b_amks_DW.previousTicks_j;
  VehCtrlMdel240926_2018b_amks_DW.previousTicks_j =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
  if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_h + rtb_Gain1_h <= 31U)
  {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_h = (uint8_T)
      (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_h + rtb_Gain1_h);
  } else {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_h = 31U;
  }

  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c28_VehCtrlMdel240926
      == 0U) {
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c28_VehCtrlMdel240926 =
      1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c28_VehCtrlMdel240926_2018b_ =
      2U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON_d = 2U;
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_h = 0U;
    rtb_ignition_e = false;
  } else if
      (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c28_VehCtrlMdel240926_2018b_
       == VehCtrlMdel240926_2_IN_STATEOFF) {
    rtb_ignition_e = true;
  } else {
    /* case IN_STATEON: */
    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON_d ==
        VehCtrlMdel240926_2018b_IN_ON_d) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON_d = 0U;
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c28_VehCtrlMdel240926_2018b_
        = 1U;
      rtb_ignition_e = true;
    } else {
      /* case IN_initstate: */
      rtb_ignition_e = false;
      rtb_LogicalOperator2 =
        ((VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_h >= 20U) &&
         rtb_Compare_am);
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON_d = 1U;
        rtb_ignition_e = true;
      }
    }
  }

  /* End of Chart: '<S215>/Chart' */

  /* UnitDelay: '<S267>/Delay Input2'
   *
   * Block description for '<S267>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE;

  /* SampleTimeMath: '<S267>/sample time'
   *
   * About '<S267>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S267>/delta rise limit' */
  rtb_Gain5 = 10.0 * elapseTime;

  /* Sum: '<S267>/Difference Inputs1'
   *
   * Block description for '<S267>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RL - rtb_Yk1_l;

  /* RelationalOperator: '<S275>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain5);

  /* Switch: '<S275>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S267>/delta fall limit' */
    elapseTime *= -10.0;

    /* RelationalOperator: '<S275>/UpperRelop' */
    rtb_Compare_am = (rtb_UkYk1 < elapseTime);

    /* Switch: '<S275>/Switch' */
    if (rtb_Compare_am) {
      rtb_UkYk1 = elapseTime;
    }

    /* End of Switch: '<S275>/Switch' */
    rtb_Gain5 = rtb_UkYk1;
  }

  /* End of Switch: '<S275>/Switch2' */

  /* Saturate: '<S216>/Saturation' incorporates:
   *  Sum: '<S267>/Difference Inputs2'
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE = rtb_Gain5 + rtb_Yk1_l;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE > 30.0) {
    rtb_Gain5 = 30.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE < 0.0) {
    rtb_Gain5 = 0.0;
  } else {
    rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE;
  }

  /* End of Saturate: '<S216>/Saturation' */

  /* Gain: '<S216>/Gain' */
  rtb_Gain5 *= 0.27777777777777779;

  /* RelationalOperator: '<S259>/Compare' incorporates:
   *  Constant: '<S259>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain5 >= 0.0);

  /* RelationalOperator: '<S260>/Compare' incorporates:
   *  Constant: '<S260>/Constant'
   */
  rtb_Compare_am = (rtb_Gain5 < 40.0);

  /* Logic: '<S216>/OR' */
  rtb_Compare_am = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S216>/Timer' incorporates:
   *  Constant: '<S216>/Constant5'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_le,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_b);

  /* UnitDelay: '<S268>/Delay Input2'
   *
   * Block description for '<S268>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b;

  /* SampleTimeMath: '<S268>/sample time'
   *
   * About '<S268>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S268>/delta rise limit' */
  rtb_Gain4 = 10.0 * elapseTime;

  /* Sum: '<S268>/Difference Inputs1'
   *
   * Block description for '<S268>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RR - rtb_Yk1_l;

  /* RelationalOperator: '<S276>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain4);

  /* Switch: '<S276>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S268>/delta fall limit' */
    elapseTime *= -10.0;

    /* RelationalOperator: '<S276>/UpperRelop' */
    rtb_Compare_am = (rtb_UkYk1 < elapseTime);

    /* Switch: '<S276>/Switch' */
    if (rtb_Compare_am) {
      rtb_UkYk1 = elapseTime;
    }

    /* End of Switch: '<S276>/Switch' */
    rtb_Gain4 = rtb_UkYk1;
  }

  /* End of Switch: '<S276>/Switch2' */

  /* Saturate: '<S216>/Saturation1' incorporates:
   *  Sum: '<S268>/Difference Inputs2'
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b = rtb_Gain4 + rtb_Yk1_l;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b > 30.0) {
    rtb_Gain4 = 30.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b < 0.0) {
    rtb_Gain4 = 0.0;
  } else {
    rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b;
  }

  /* End of Saturate: '<S216>/Saturation1' */

  /* Gain: '<S216>/Gain3' */
  rtb_Gain4 *= 0.27777777777777779;

  /* RelationalOperator: '<S261>/Compare' incorporates:
   *  Constant: '<S261>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain4 >= 0.0);

  /* RelationalOperator: '<S262>/Compare' incorporates:
   *  Constant: '<S262>/Constant'
   */
  rtb_Compare_am = (rtb_Gain4 < 40.0);

  /* Logic: '<S216>/OR1' */
  rtb_Compare_am = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S216>/Timer1' incorporates:
   *  Constant: '<S216>/Constant1'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_is,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_n);

  /* UnitDelay: '<S269>/Delay Input2'
   *
   * Block description for '<S269>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h;

  /* SampleTimeMath: '<S269>/sample time'
   *
   * About '<S269>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S269>/delta rise limit' */
  rtb_Switch2_on = 10.0 * elapseTime;

  /* Sum: '<S269>/Difference Inputs1'
   *
   * Block description for '<S269>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = MCFR_ActualVelocity - rtb_Yk1_l;

  /* RelationalOperator: '<S277>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Switch2_on);

  /* Switch: '<S277>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S269>/delta fall limit' */
    elapseTime *= -10.0;

    /* RelationalOperator: '<S277>/UpperRelop' */
    rtb_Compare_am = (rtb_UkYk1 < elapseTime);

    /* Switch: '<S277>/Switch' */
    if (rtb_Compare_am) {
      rtb_UkYk1 = elapseTime;
    }

    /* End of Switch: '<S277>/Switch' */
    rtb_Switch2_on = rtb_UkYk1;
  }

  /* End of Switch: '<S277>/Switch2' */

  /* Saturate: '<S216>/Saturation2' incorporates:
   *  Sum: '<S269>/Difference Inputs2'
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h = rtb_Switch2_on +
    rtb_Yk1_l;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h > 30.0) {
    rtb_Switch2_on = 30.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h < 0.0) {
    rtb_Switch2_on = 0.0;
  } else {
    rtb_Switch2_on = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h;
  }

  /* End of Saturate: '<S216>/Saturation2' */

  /* Gain: '<S216>/Gain1' */
  rtb_Switch2_on *= 0.002235050147492625;

  /* RelationalOperator: '<S263>/Compare' incorporates:
   *  Constant: '<S263>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Switch2_on >= 0.0);

  /* RelationalOperator: '<S264>/Compare' incorporates:
   *  Constant: '<S264>/Constant'
   */
  rtb_Compare_am = (rtb_Switch2_on < 40.0);

  /* Logic: '<S216>/OR2' */
  rtb_Compare_am = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S216>/Timer2' incorporates:
   *  Constant: '<S216>/Constant4'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_o,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_l);

  /* UnitDelay: '<S270>/Delay Input2'
   *
   * Block description for '<S270>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n;

  /* SampleTimeMath: '<S270>/sample time'
   *
   * About '<S270>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S270>/delta rise limit' */
  rtb_UkYk1 = 10.0 * elapseTime;

  /* Sum: '<S270>/Difference Inputs1'
   *
   * Block description for '<S270>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1_nc = MCFL_ActualVelocity - rtb_Yk1_l;

  /* RelationalOperator: '<S278>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1_nc > rtb_UkYk1);

  /* Switch: '<S278>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S270>/delta fall limit' */
    elapseTime *= -10.0;

    /* RelationalOperator: '<S278>/UpperRelop' */
    rtb_Compare_am = (rtb_UkYk1_nc < elapseTime);

    /* Switch: '<S278>/Switch' */
    if (rtb_Compare_am) {
      rtb_UkYk1_nc = elapseTime;
    }

    /* End of Switch: '<S278>/Switch' */
    rtb_UkYk1 = rtb_UkYk1_nc;
  }

  /* End of Switch: '<S278>/Switch2' */

  /* Saturate: '<S216>/Saturation3' incorporates:
   *  Sum: '<S270>/Difference Inputs2'
   *  UnitDelay: '<S270>/Delay Input2'
   *
   * Block description for '<S270>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S270>/Delay Input2':
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

  /* End of Saturate: '<S216>/Saturation3' */

  /* Gain: '<S216>/Gain2' */
  rtb_UkYk1 *= 0.002235050147492625;

  /* RelationalOperator: '<S265>/Compare' incorporates:
   *  Constant: '<S265>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_UkYk1 >= 0.0);

  /* RelationalOperator: '<S266>/Compare' incorporates:
   *  Constant: '<S266>/Constant'
   */
  rtb_Compare_am = (rtb_UkYk1 < 40.0);

  /* Logic: '<S216>/OR3' */
  rtb_Compare_am = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S216>/Timer3' incorporates:
   *  Constant: '<S216>/Constant8'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_h,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer3);

  /* SignalConversion generated from: '<S209>/Out1' */
  rtb_Yk1_l = rtb_UkYk1;

  /* SignalConversion generated from: '<S209>/Out1' */
  WhlSpdFR = rtb_Switch2_on;

  /* SignalConversion generated from: '<S209>/Out1' */
  WhlSpdRR_mps = rtb_Gain4;

  /* SignalConversion generated from: '<S209>/Out1' */
  WhlSpdRL_mps = rtb_Gain5;

  /* Gain: '<S213>/Gain' incorporates:
   *  UnitDelay: '<S247>/Delay Input2'
   *
   * Block description for '<S247>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_StrWhlAngV_c = 0.7F *
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2;

  /* UnitDelay: '<S213>/Unit Delay' */
  rtb_FRWhlStrAng = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o2;

  /* Gain: '<S213>/Gain1' */
  rtb_FRWhlStrAng *= 0.3F;

  /* Sum: '<S213>/Add2' */
  rtb_StrWhlAngV_c += rtb_FRWhlStrAng;

  /* Lookup_n-D: '<S213>/1-D Lookup Table' */
  rtb_FRWhlStrAng = look1_iflf_binlx(rtb_StrWhlAngV_c,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable_tableData, 23U);

  /* SignalConversion generated from: '<S209>/Out1' */
  FLWhlStrAng = rtb_FRWhlStrAng;

  /* Lookup_n-D: '<S213>/1-D Lookup Table1' */
  rtb_FRWhlStrAng = look1_iflf_binlx(rtb_StrWhlAngV_c,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp01Data_h,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_tableData_b, 23U);

  /* SignalConversion generated from: '<S209>/Out1' */
  rtb_CastToBoolean = rtb_StrWhlAngV_c;

  /* Sum: '<S211>/Add' */
  rtb_StrWhlAngV_c = (real32_T)Acc_POS * 1.52587891E-5F + rtb_Gain3_o;

  /* Product: '<S211>/Divide' incorporates:
   *  Constant: '<S211>/Constant'
   */
  rtb_Gain3_o = (real32_T)(rtb_StrWhlAngV_c / 2.0);

  /* SignalConversion generated from: '<S209>/Out1' */
  Acc_POS_n = rtb_Gain3_o;

  /* UnitDelay: '<S237>/Delay Input2'
   *
   * Block description for '<S237>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l;

  /* SampleTimeMath: '<S237>/sample time'
   *
   * About '<S237>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S237>/delta rise limit' incorporates:
   *  Constant: '<S236>/Constant'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* UnitDelay: '<S236>/Unit Delay' */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE;

  /* Gain: '<S236>/Gain1' */
  rtb_Gain4 *= 0.3;

  /* Gain: '<S212>/g_mpss' incorporates:
   *  UnitDelay: '<S236>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE = 9.8 * IMU_Ay_Value;

  /* Gain: '<S236>/Gain' incorporates:
   *  UnitDelay: '<S236>/Unit Delay'
   */
  rtb_Gain5 = 0.7 * VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE;

  /* Sum: '<S236>/Add2' */
  rtb_UkYk1_nc = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S237>/Difference Inputs1'
   *
   * Block description for '<S237>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1_nc -= rtb_UkYk1;

  /* RelationalOperator: '<S243>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1_nc > rtb_Switch2_on);

  /* Switch: '<S243>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S237>/delta fall limit' incorporates:
     *  Constant: '<S236>/Constant1'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S243>/UpperRelop' */
    rtb_Compare_am = (rtb_UkYk1_nc < elapseTime);

    /* Switch: '<S243>/Switch' */
    if (rtb_Compare_am) {
      rtb_UkYk1_nc = elapseTime;
    }

    /* End of Switch: '<S243>/Switch' */
    rtb_Switch2_on = rtb_UkYk1_nc;
  }

  /* End of Switch: '<S243>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l = rtb_Switch2_on +
    rtb_UkYk1;

  /* RelationalOperator: '<S240>/LowerRelop1' incorporates:
   *  Constant: '<S236>/Constant6'
   *  UnitDelay: '<S237>/Delay Input2'
   *
   * Block description for '<S237>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l >
                       1.5);

  /* Switch: '<S240>/Switch2' incorporates:
   *  Constant: '<S236>/Constant6'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S240>/UpperRelop' incorporates:
     *  Constant: '<S236>/Constant7'
     *  UnitDelay: '<S237>/Delay Input2'
     *
     * Block description for '<S237>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l <
                      -1.5);

    /* Switch: '<S240>/Switch' incorporates:
     *  Constant: '<S236>/Constant7'
     *  UnitDelay: '<S237>/Delay Input2'
     *
     * Block description for '<S237>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_Compare_am) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l;
    }

    /* End of Switch: '<S240>/Switch' */
  }

  /* End of Switch: '<S240>/Switch2' */

  /* SignalConversion generated from: '<S209>/Out1' */
  rtb_UkYk1_nc = rtb_UkYk1;

  /* UnitDelay: '<S238>/Delay Input2'
   *
   * Block description for '<S238>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m;

  /* SampleTimeMath: '<S238>/sample time'
   *
   * About '<S238>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S238>/delta rise limit' incorporates:
   *  Constant: '<S236>/Constant2'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S212>/g_mpss1' */
  rtb_g_mpss1 = 9.8 * IMU_Ax_Value;

  /* Gain: '<S236>/Gain2' */
  rtb_Gain4 = 0.7 * rtb_g_mpss1;

  /* UnitDelay: '<S236>/Unit Delay1' */
  rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE;

  /* Gain: '<S236>/Gain3' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S236>/Add1' */
  rtb_Add1 = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S238>/Difference Inputs1'
   *
   * Block description for '<S238>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add1 -= rtb_UkYk1;

  /* RelationalOperator: '<S244>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Add1 > rtb_Switch2_on);

  /* Switch: '<S244>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S238>/delta fall limit' incorporates:
     *  Constant: '<S236>/Constant4'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S244>/UpperRelop' */
    rtb_Compare_am = (rtb_Add1 < elapseTime);

    /* Switch: '<S244>/Switch' */
    if (rtb_Compare_am) {
      rtb_Add1 = elapseTime;
    }

    /* End of Switch: '<S244>/Switch' */
    rtb_Switch2_on = rtb_Add1;
  }

  /* End of Switch: '<S244>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m = rtb_Switch2_on +
    rtb_UkYk1;

  /* RelationalOperator: '<S241>/LowerRelop1' incorporates:
   *  Constant: '<S236>/Constant8'
   *  UnitDelay: '<S238>/Delay Input2'
   *
   * Block description for '<S238>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m >
                       1.5);

  /* Switch: '<S241>/Switch2' incorporates:
   *  Constant: '<S236>/Constant8'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S241>/UpperRelop' incorporates:
     *  Constant: '<S236>/Constant9'
     *  UnitDelay: '<S238>/Delay Input2'
     *
     * Block description for '<S238>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m <
                      -1.5);

    /* Switch: '<S241>/Switch' incorporates:
     *  Constant: '<S236>/Constant9'
     *  UnitDelay: '<S238>/Delay Input2'
     *
     * Block description for '<S238>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_Compare_am) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m;
    }

    /* End of Switch: '<S241>/Switch' */
  }

  /* End of Switch: '<S241>/Switch2' */

  /* SignalConversion generated from: '<S209>/Out1' */
  rtb_Add1 = rtb_UkYk1;

  /* SampleTimeMath: '<S248>/sample time'
   *
   * About '<S248>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S248>/delta rise limit' */
  rtb_StrWhlAngV_c = (real32_T)(1200.0 * elapseTime);

  /* DataTypeConversion: '<S213>/Cast To Boolean1' */
  rtb_CastToBoolean1 = (real32_T)StrWhlAngV;

  /* UnitDelay: '<S248>/Delay Input2'
   *
   * Block description for '<S248>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain3_o = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j;

  /* Sum: '<S248>/Difference Inputs1'
   *
   * Block description for '<S248>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_CastToBoolean1 -= rtb_Gain3_o;

  /* RelationalOperator: '<S251>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_CastToBoolean1 > rtb_StrWhlAngV_c);

  /* Switch: '<S251>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S248>/delta fall limit' */
    rtb_StrWhlAngV_c = (real32_T)(-1200.0 * elapseTime);

    /* RelationalOperator: '<S251>/UpperRelop' */
    rtb_Compare_am = (rtb_CastToBoolean1 < rtb_StrWhlAngV_c);

    /* Switch: '<S251>/Switch' */
    if (rtb_Compare_am) {
      rtb_CastToBoolean1 = rtb_StrWhlAngV_c;
    }

    /* End of Switch: '<S251>/Switch' */
    rtb_StrWhlAngV_c = rtb_CastToBoolean1;
  }

  /* End of Switch: '<S251>/Switch2' */

  /* Saturate: '<S213>/Saturation1' incorporates:
   *  Sum: '<S248>/Difference Inputs2'
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j = rtb_StrWhlAngV_c +
    rtb_Gain3_o;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j > 1200.0F) {
    rtb_CastToBoolean1 = 1200.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j < 0.0F) {
    rtb_CastToBoolean1 = 0.0F;
  } else {
    rtb_CastToBoolean1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j;
  }

  /* End of Saturate: '<S213>/Saturation1' */

  /* Gain: '<S213>/Gain2' */
  rtb_StrWhlAngV_c = 0.7F * rtb_CastToBoolean1;

  /* UnitDelay: '<S213>/Unit Delay1' */
  rtb_Gain3_o = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_aq;

  /* Gain: '<S213>/Gain3' */
  rtb_Gain3_o *= 0.3F;

  /* Sum: '<S213>/Add1' */
  rtb_StrWhlAngV_c += rtb_Gain3_o;

  /* UnitDelay: '<S239>/Delay Input2'
   *
   * Block description for '<S239>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k;

  /* SampleTimeMath: '<S239>/sample time'
   *
   * About '<S239>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S239>/delta rise limit' incorporates:
   *  Constant: '<S236>/Constant3'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S236>/Gain4' */
  rtb_Gain4 = 0.7 * IMU_Yaw_Value;

  /* UnitDelay: '<S236>/Unit Delay2' */
  rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE;

  /* Gain: '<S236>/Gain5' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S236>/Add3' */
  rtb_Gain5 += rtb_Gain4;

  /* Sum: '<S239>/Difference Inputs1'
   *
   * Block description for '<S239>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 -= rtb_UkYk1;

  /* RelationalOperator: '<S245>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Gain5 > rtb_Switch2_on);

  /* Switch: '<S245>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S239>/delta fall limit' incorporates:
     *  Constant: '<S236>/Constant5'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S245>/UpperRelop' */
    rtb_Compare_am = (rtb_Gain5 < elapseTime);

    /* Switch: '<S245>/Switch' */
    if (rtb_Compare_am) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S245>/Switch' */
    rtb_Switch2_on = rtb_Gain5;
  }

  /* End of Switch: '<S245>/Switch2' */

  /* Saturate: '<S236>/Saturation2' incorporates:
   *  Sum: '<S239>/Difference Inputs2'
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k = rtb_Switch2_on +
    rtb_UkYk1;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k > 180.0) {
    rtb_UkYk1 = 180.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k < -180.0) {
    rtb_UkYk1 = -180.0;
  } else {
    rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k;
  }

  /* End of Saturate: '<S236>/Saturation2' */

  /* SignalConversion generated from: '<S209>/Out1' */
  KeyPressed = VehCtrlMdel240926_2018b_amksp_B.ignition_d;

  /* SignalConversion generated from: '<S209>/Out1' */
  rtb_LogicalOperator2 = VehCtrlMdel240926_2018b_amksp_B.out2_h;

  /* SignalConversion generated from: '<S209>/Out1' */
  rtb_Compare_am = VehCtrlMdel240926_2018b_amksp_B.TSAL_SW_IN_i2;

  /* SignalConversion generated from: '<S209>/Out1' */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amksp_B.HV_voltValid_kx;

  /* Switch: '<S214>/Switch' incorporates:
   *  Constant: '<S214>/Constant4'
   */
  if (MCFL_DCVoltage != 0.0) {
    /* MinMax: '<S214>/Max' incorporates:
     *  Constant: '<S214>/Constant2'
     */
    elapseTime = fmax(MCFL_DCVoltage, 0.0099999997764825821);

    /* Product: '<S214>/Product' */
    rtb_Switch2_on = MCFL_ActualTorque * MCFL_ActualVelocity;

    /* Product: '<S214>/Divide' */
    rtb_Switch2_on /= 9550.0;

    /* Product: '<S214>/Divide1' */
    AMKFL_Current = rtb_Switch2_on / elapseTime;
  } else {
    AMKFL_Current = 0.0;
  }

  /* End of Switch: '<S214>/Switch' */

  /* Switch: '<S214>/Switch1' incorporates:
   *  Constant: '<S214>/Constant6'
   */
  if (MCFR_DCVoltage != 0.0) {
    /* MinMax: '<S214>/Max1' incorporates:
     *  Constant: '<S214>/Constant3'
     */
    elapseTime = fmax(MCFR_DCVoltage, 0.0099999997764825821);

    /* Product: '<S214>/Product1' */
    rtb_Switch2_on = MCFR_ActualTorque * MCFR_ActualVelocity;

    /* Product: '<S214>/Divide2' */
    rtb_Switch2_on /= 9550.0;

    /* Product: '<S214>/Divide3' */
    AMKFR_Current = rtb_Switch2_on / elapseTime;
  } else {
    AMKFR_Current = 0.0;
  }

  /* End of Switch: '<S214>/Switch1' */

  /* Product: '<S214>/Product2' */
  EmraxPwr = voltage * DC_current;

  /* Update for UnitDelay: '<S211>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm = Acc2;

  /* Update for UnitDelay: '<S211>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_k = Acc1;

  /* Update for UnitDelay: '<S256>/Delay Input1'
   *
   * Block description for '<S256>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_a = rtb_Compare;

  /* Update for UnitDelay: '<S213>/Unit Delay' incorporates:
   *  UnitDelay: '<S247>/Delay Input2'
   *
   * Block description for '<S247>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o2 =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2;

  /* Update for UnitDelay: '<S236>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE = rtb_g_mpss1;

  /* Update for UnitDelay: '<S213>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_aq = rtb_CastToBoolean1;

  /* Update for UnitDelay: '<S236>/Unit Delay2' */
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

  /* Gain: '<S210>/Gain5' */
  elapseTime = 10.0 * VehCtrlMdel240926_2018b_amksp_B.CANUnpack_o1;

  /* DataTypeConversion: '<S210>/Cast To Double' */
  rtb_CastToDouble = (real32_T)elapseTime;

  /* MinMax: '<S343>/Min3' incorporates:
   *  Gain: '<S297>/Gain'
   *  UnitDelay: '<S297>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p *= 0.3F;

  /* UnitDelay: '<S301>/Delay Input2'
   *
   * Block description for '<S301>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* SampleTimeMath: '<S301>/sample time'
   *
   * About '<S301>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S301>/delta rise limit' */
  rtb_Add4_j = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S210>/Cast To Double1' */
  rtb_Add7 = (real32_T)rtb_Yk1_l;

  /* Gain: '<S283>/Gain2' */
  rtb_Add6 = 0.0174532924F * FLWhlStrAng;

  /* Trigonometry: '<S283>/Asin' */
  rtb_Add6 = cosf(rtb_Add6);

  /* Product: '<S283>/Product1' */
  rtb_Add7 *= rtb_Add6;

  /* DataTypeConversion: '<S210>/Cast To Double5' */
  rtb_Add6 = (real32_T)rtb_UkYk1;

  /* Gain: '<S283>/Gain4' */
  rtb_Add6 *= 0.0174532924F;

  /* Product: '<S283>/Product3' */
  rtb_Switch2_mn = 0.6F * rtb_Add6;

  /* Sum: '<S283>/Add2' */
  rtb_Gain3_o = rtb_Add7 - rtb_Switch2_mn;

  /* UnitDelay: '<S290>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_j;

  /* Sum: '<S290>/Add4' */
  rtb_Add7 = rtb_Gain3_o - rtb_Add7;

  /* Product: '<S290>/Divide' incorporates:
   *  Constant: '<S290>/steptime'
   */
  rtb_CastToBoolean1 = rtb_Add7 / 0.01F;

  /* RelationalOperator: '<S302>/LowerRelop1' incorporates:
   *  Constant: '<S297>/Constant1'
   */
  rtb_LogicalOperator7_m = (rtb_CastToBoolean1 > 100.0F);

  /* Switch: '<S302>/Switch2' incorporates:
   *  Constant: '<S297>/Constant1'
   */
  if (rtb_LogicalOperator7_m) {
    rtb_CastToBoolean1 = 100.0F;
  } else {
    /* RelationalOperator: '<S302>/UpperRelop' incorporates:
     *  Constant: '<S297>/Constant'
     */
    rtb_Compare = (rtb_CastToBoolean1 < -100.0F);

    /* Switch: '<S302>/Switch' incorporates:
     *  Constant: '<S297>/Constant'
     */
    if (rtb_Compare) {
      rtb_CastToBoolean1 = -100.0F;
    }

    /* End of Switch: '<S302>/Switch' */
  }

  /* End of Switch: '<S302>/Switch2' */

  /* Sum: '<S301>/Difference Inputs1'
   *
   * Block description for '<S301>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_CastToBoolean1 -= rtb_Switch2_b0;

  /* RelationalOperator: '<S303>/LowerRelop1' */
  rtb_LogicalOperator7_m = (rtb_CastToBoolean1 > rtb_Add4_j);

  /* Switch: '<S303>/Switch2' */
  if (!rtb_LogicalOperator7_m) {
    /* Product: '<S301>/delta fall limit' */
    rtb_deltafalllimit_cz = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S303>/UpperRelop' */
    rtb_Compare = (rtb_CastToBoolean1 < rtb_deltafalllimit_cz);

    /* Switch: '<S303>/Switch' */
    if (rtb_Compare) {
      rtb_CastToBoolean1 = rtb_deltafalllimit_cz;
    }

    /* End of Switch: '<S303>/Switch' */
    rtb_Add4_j = rtb_CastToBoolean1;
  }

  /* End of Switch: '<S303>/Switch2' */

  /* Sum: '<S301>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S301>/Delay Input2'
   *
   * Block description for '<S301>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S301>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2 = rtb_Add4_j +
    rtb_Switch2_b0;

  /* Gain: '<S297>/Gain1' incorporates:
   *  UnitDelay: '<S301>/Delay Input2'
   *
   * Block description for '<S301>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add7 = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* MinMax: '<S343>/Min3' incorporates:
   *  Abs: '<S290>/Abs'
   *  Sum: '<S290>/Add'
   *  Sum: '<S297>/Add'
   *  UnitDelay: '<S297>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p += rtb_Add7;
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p -= rtb_CastToDouble;
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p = fabsf
    (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p);

  /* RelationalOperator: '<S293>/Compare' incorporates:
   *  Constant: '<S293>/Constant'
   *  UnitDelay: '<S297>/Unit Delay'
   */
  rtb_LogicalOperator7_m = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p <=
    0.5F);

  /* UnitDelay: '<S298>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj;

  /* Gain: '<S298>/Gain' */
  rtb_Add7 *= 0.3F;

  /* UnitDelay: '<S304>/Delay Input2'
   *
   * Block description for '<S304>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* SampleTimeMath: '<S304>/sample time'
   *
   * About '<S304>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S304>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(100.0 * elapseTime);

  /* MinMax: '<S343>/Min3' incorporates:
   *  DataTypeConversion: '<S210>/Cast To Double2'
   *  UnitDelay: '<S297>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p = (real32_T)WhlSpdFR;

  /* Gain: '<S283>/Gain3' */
  rtb_Add10 = 0.0174532924F * rtb_FRWhlStrAng;

  /* Trigonometry: '<S283>/Asin1' */
  rtb_Add10 = cosf(rtb_Add10);

  /* Product: '<S283>/Product2' incorporates:
   *  UnitDelay: '<S297>/Unit Delay'
   */
  rtb_Add10 *= VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p;

  /* Sum: '<S283>/Add3' */
  rtb_CastToBoolean1 = rtb_Switch2_mn + rtb_Add10;

  /* UnitDelay: '<S290>/Unit Delay1' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n;

  /* Sum: '<S290>/Add5' */
  rtb_Add10 = rtb_CastToBoolean1 - rtb_Add10;

  /* Product: '<S290>/Divide1' incorporates:
   *  Constant: '<S290>/steptime1'
   */
  rtb_deltafalllimit_cz = rtb_Add10 / 0.01F;

  /* RelationalOperator: '<S305>/LowerRelop1' incorporates:
   *  Constant: '<S298>/Constant1'
   */
  rtb_Compare_nd = (rtb_deltafalllimit_cz > 100.0F);

  /* Switch: '<S305>/Switch2' incorporates:
   *  Constant: '<S298>/Constant1'
   */
  if (rtb_Compare_nd) {
    rtb_deltafalllimit_cz = 100.0F;
  } else {
    /* RelationalOperator: '<S305>/UpperRelop' incorporates:
     *  Constant: '<S298>/Constant'
     */
    rtb_Compare = (rtb_deltafalllimit_cz < -100.0F);

    /* Switch: '<S305>/Switch' incorporates:
     *  Constant: '<S298>/Constant'
     */
    if (rtb_Compare) {
      rtb_deltafalllimit_cz = -100.0F;
    }

    /* End of Switch: '<S305>/Switch' */
  }

  /* End of Switch: '<S305>/Switch2' */

  /* Sum: '<S304>/Difference Inputs1'
   *
   * Block description for '<S304>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_cz -= rtb_Add4_j;

  /* RelationalOperator: '<S306>/LowerRelop1' */
  rtb_Compare_nd = (rtb_deltafalllimit_cz > rtb_Switch2_b0);

  /* Switch: '<S306>/Switch2' */
  if (!rtb_Compare_nd) {
    /* Product: '<S304>/delta fall limit' */
    rtb_deltafalllimit_ap = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S306>/UpperRelop' */
    rtb_Compare = (rtb_deltafalllimit_cz < rtb_deltafalllimit_ap);

    /* Switch: '<S306>/Switch' */
    if (rtb_Compare) {
      rtb_deltafalllimit_cz = rtb_deltafalllimit_ap;
    }

    /* End of Switch: '<S306>/Switch' */
    rtb_Switch2_b0 = rtb_deltafalllimit_cz;
  }

  /* End of Switch: '<S306>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e = rtb_Switch2_b0 +
    rtb_Add4_j;

  /* Gain: '<S298>/Gain1' incorporates:
   *  UnitDelay: '<S304>/Delay Input2'
   *
   * Block description for '<S304>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* Sum: '<S298>/Add' */
  rtb_Add7 += rtb_Add10;

  /* Sum: '<S290>/Add1' */
  rtb_Add7 -= rtb_CastToDouble;

  /* Abs: '<S290>/Abs1' */
  rtb_Add7 = fabsf(rtb_Add7);

  /* RelationalOperator: '<S294>/Compare' incorporates:
   *  Constant: '<S294>/Constant'
   */
  rtb_Compare_nd = (rtb_Add7 <= 0.5F);

  /* UnitDelay: '<S299>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a;

  /* Gain: '<S299>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S307>/Delay Input2'
   *
   * Block description for '<S307>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* SampleTimeMath: '<S307>/sample time'
   *
   * About '<S307>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S307>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S210>/Cast To Double3' */
  rtb_Add4_j = (real32_T)WhlSpdRL_mps;

  /* Product: '<S283>/Product' */
  rtb_Add6 *= 0.58F;

  /* Sum: '<S283>/Add' */
  rtb_deltafalllimit_cz = rtb_Add4_j - rtb_Add6;

  /* UnitDelay: '<S290>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_l;

  /* Sum: '<S290>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_cz - rtb_Add4_j;

  /* Product: '<S290>/Divide2' incorporates:
   *  Constant: '<S290>/steptime2'
   */
  rtb_deltafalllimit_ap = rtb_Add4_j / 0.01F;

  /* RelationalOperator: '<S308>/LowerRelop1' incorporates:
   *  Constant: '<S299>/Constant1'
   */
  rtb_Compare_b = (rtb_deltafalllimit_ap > 100.0F);

  /* Switch: '<S308>/Switch2' incorporates:
   *  Constant: '<S299>/Constant1'
   */
  if (rtb_Compare_b) {
    rtb_deltafalllimit_ap = 100.0F;
  } else {
    /* RelationalOperator: '<S308>/UpperRelop' incorporates:
     *  Constant: '<S299>/Constant'
     */
    rtb_Compare = (rtb_deltafalllimit_ap < -100.0F);

    /* Switch: '<S308>/Switch' incorporates:
     *  Constant: '<S299>/Constant'
     */
    if (rtb_Compare) {
      rtb_deltafalllimit_ap = -100.0F;
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
  rtb_deltafalllimit_ap -= rtb_Switch2_mn;

  /* RelationalOperator: '<S309>/LowerRelop1' */
  rtb_Compare_b = (rtb_deltafalllimit_ap > rtb_Add7);

  /* Switch: '<S309>/Switch2' */
  if (!rtb_Compare_b) {
    /* Product: '<S307>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S309>/UpperRelop' */
    rtb_Compare = (rtb_deltafalllimit_ap < rtb_Add7);

    /* Switch: '<S309>/Switch' */
    if (rtb_Compare) {
      rtb_deltafalllimit_ap = rtb_Add7;
    }

    /* End of Switch: '<S309>/Switch' */
    rtb_Add7 = rtb_deltafalllimit_ap;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk = rtb_Add7 +
    rtb_Switch2_mn;

  /* Gain: '<S299>/Gain1' incorporates:
   *  UnitDelay: '<S307>/Delay Input2'
   *
   * Block description for '<S307>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* Sum: '<S299>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S290>/Add2' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S290>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S295>/Compare' incorporates:
   *  Constant: '<S295>/Constant'
   */
  rtb_Compare_b = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S300>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nc;

  /* Gain: '<S300>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S310>/Delay Input2'
   *
   * Block description for '<S310>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* SampleTimeMath: '<S310>/sample time'
   *
   * About '<S310>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S310>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S210>/Cast To Double4' */
  rtb_Add4_j = (real32_T)WhlSpdRR_mps;

  /* Sum: '<S283>/Add1' */
  rtb_deltafalllimit_ap = rtb_Add6 + rtb_Add4_j;

  /* UnitDelay: '<S290>/Unit Delay3' */
  rtb_Add6 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE;

  /* Sum: '<S290>/Add7' */
  rtb_Add6 = rtb_deltafalllimit_ap - rtb_Add6;

  /* Product: '<S290>/Divide3' incorporates:
   *  Constant: '<S290>/steptime3'
   */
  rtb_Add6 /= 0.01F;

  /* RelationalOperator: '<S311>/LowerRelop1' incorporates:
   *  Constant: '<S300>/Constant1'
   */
  rtb_Compare = (rtb_Add6 > 100.0F);

  /* Switch: '<S311>/Switch2' incorporates:
   *  Constant: '<S300>/Constant1'
   */
  if (rtb_Compare) {
    rtb_Add6 = 100.0F;
  } else {
    /* RelationalOperator: '<S311>/UpperRelop' incorporates:
     *  Constant: '<S300>/Constant'
     */
    rtb_Compare = (rtb_Add6 < -100.0F);

    /* Switch: '<S311>/Switch' incorporates:
     *  Constant: '<S300>/Constant'
     */
    if (rtb_Compare) {
      rtb_Add6 = -100.0F;
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
  rtb_Add6 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S312>/LowerRelop1' */
  rtb_Compare = (rtb_Add6 > rtb_Add7);

  /* Switch: '<S312>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S310>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S312>/UpperRelop' */
    rtb_Compare = (rtb_Add6 < rtb_Add7);

    /* Switch: '<S312>/Switch' */
    if (rtb_Compare) {
      rtb_Add6 = rtb_Add7;
    }

    /* End of Switch: '<S312>/Switch' */
    rtb_Add7 = rtb_Add6;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c = rtb_Add7 +
    rtb_Switch2_mn;

  /* Gain: '<S300>/Gain1' incorporates:
   *  UnitDelay: '<S310>/Delay Input2'
   *
   * Block description for '<S310>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* Sum: '<S300>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S290>/Add3' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S290>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S296>/Compare' incorporates:
   *  Constant: '<S296>/Constant'
   */
  rtb_Compare = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S317>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l;

  /* Gain: '<S317>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S321>/Delay Input2'
   *
   * Block description for '<S321>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* SampleTimeMath: '<S321>/sample time'
   *
   * About '<S321>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S321>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S291>/Unit Delay4' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE;

  /* UnitDelay: '<S291>/Unit Delay' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a0;

  /* Sum: '<S291>/Add4' */
  rtb_Add4_j = rtb_Gain3_o - rtb_Add4_j;

  /* Product: '<S291>/Divide' incorporates:
   *  Constant: '<S291>/steptime'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S291>/Add' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S291>/Add8' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE - rtb_Add7;

  /* Product: '<S291>/Divide4' incorporates:
   *  Constant: '<S291>/steptime4'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S322>/LowerRelop1' incorporates:
   *  Constant: '<S317>/Constant1'
   */
  rtb_AND_l = (rtb_Add7 > 100.0F);

  /* Switch: '<S322>/Switch2' incorporates:
   *  Constant: '<S317>/Constant1'
   */
  if (rtb_AND_l) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S322>/UpperRelop' incorporates:
     *  Constant: '<S317>/Constant'
     */
    rtb_AND_l = (rtb_Add7 < -100.0F);

    /* Switch: '<S322>/Switch' incorporates:
     *  Constant: '<S317>/Constant'
     */
    if (rtb_AND_l) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S322>/Switch' */
  }

  /* End of Switch: '<S322>/Switch2' */

  /* Sum: '<S321>/Difference Inputs1'
   *
   * Block description for '<S321>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S323>/LowerRelop1' */
  rtb_AND_l = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S323>/Switch2' */
  if (!rtb_AND_l) {
    /* Product: '<S321>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S323>/UpperRelop' */
    rtb_AND_l = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S323>/Switch' */
    if (rtb_AND_l) {
      rtb_Add7 = rtb_Add6;
    }

    /* End of Switch: '<S323>/Switch' */
    rtb_Add6 = rtb_Add7;
  }

  /* End of Switch: '<S323>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S317>/Gain1' incorporates:
   *  UnitDelay: '<S321>/Delay Input2'
   *
   * Block description for '<S321>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* Sum: '<S317>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S291>/Abs' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S313>/Compare' incorporates:
   *  Constant: '<S313>/Constant'
   */
  rtb_AND_l = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S318>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ap;

  /* Gain: '<S318>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S324>/Delay Input2'
   *
   * Block description for '<S324>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* SampleTimeMath: '<S324>/sample time'
   *
   * About '<S324>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S324>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S291>/Unit Delay5' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE;

  /* UnitDelay: '<S291>/Unit Delay1' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_a;

  /* Sum: '<S291>/Add5' */
  rtb_Add4_j = rtb_CastToBoolean1 - rtb_Add4_j;

  /* Product: '<S291>/Divide1' incorporates:
   *  Constant: '<S291>/steptime1'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S291>/Add1' incorporates:
   *  UnitDelay: '<S291>/Unit Delay5'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S291>/Add10' incorporates:
   *  UnitDelay: '<S291>/Unit Delay5'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE - rtb_Add7;

  /* Product: '<S291>/Divide5' incorporates:
   *  Constant: '<S291>/steptime5'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S325>/LowerRelop1' incorporates:
   *  Constant: '<S318>/Constant1'
   */
  rtb_AND2_e = (rtb_Add7 > 100.0F);

  /* Switch: '<S325>/Switch2' incorporates:
   *  Constant: '<S318>/Constant1'
   */
  if (rtb_AND2_e) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S325>/UpperRelop' incorporates:
     *  Constant: '<S318>/Constant'
     */
    rtb_AND2_e = (rtb_Add7 < -100.0F);

    /* Switch: '<S325>/Switch' incorporates:
     *  Constant: '<S318>/Constant'
     */
    if (rtb_AND2_e) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S325>/Switch' */
  }

  /* End of Switch: '<S325>/Switch2' */

  /* Sum: '<S324>/Difference Inputs1'
   *
   * Block description for '<S324>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S326>/LowerRelop1' */
  rtb_AND2_e = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S326>/Switch2' */
  if (!rtb_AND2_e) {
    /* Product: '<S324>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S326>/UpperRelop' */
    rtb_AND2_e = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S326>/Switch' */
    if (rtb_AND2_e) {
      rtb_Add7 = rtb_Add6;
    }

    /* End of Switch: '<S326>/Switch' */
    rtb_Add6 = rtb_Add7;
  }

  /* End of Switch: '<S326>/Switch2' */

  /* Sum: '<S324>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S324>/Delay Input2'
   *
   * Block description for '<S324>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S324>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S318>/Gain1' incorporates:
   *  UnitDelay: '<S324>/Delay Input2'
   *
   * Block description for '<S324>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* Sum: '<S318>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S291>/Abs1' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S314>/Compare' incorporates:
   *  Constant: '<S314>/Constant'
   */
  rtb_AND2_e = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S319>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o;

  /* Gain: '<S319>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S327>/Delay Input2'
   *
   * Block description for '<S327>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pd;

  /* SampleTimeMath: '<S327>/sample time'
   *
   * About '<S327>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S327>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S291>/Unit Delay6' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE;

  /* UnitDelay: '<S291>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_c;

  /* Sum: '<S291>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_cz - rtb_Add4_j;

  /* Product: '<S291>/Divide2' incorporates:
   *  Constant: '<S291>/steptime2'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S291>/Add2' incorporates:
   *  UnitDelay: '<S291>/Unit Delay6'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S291>/Add12' incorporates:
   *  UnitDelay: '<S291>/Unit Delay6'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE - rtb_Add7;

  /* Product: '<S291>/Divide6' incorporates:
   *  Constant: '<S291>/steptime6'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S328>/LowerRelop1' incorporates:
   *  Constant: '<S319>/Constant1'
   */
  rtb_Compare_ar = (rtb_Add7 > 100.0F);

  /* Switch: '<S328>/Switch2' incorporates:
   *  Constant: '<S319>/Constant1'
   */
  if (rtb_Compare_ar) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S328>/UpperRelop' incorporates:
     *  Constant: '<S319>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S328>/Switch' incorporates:
     *  Constant: '<S319>/Constant'
     */
    if (rtb_UpperRelop_ir) {
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
  rtb_Compare_ar = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S329>/Switch2' */
  if (!rtb_Compare_ar) {
    /* Product: '<S327>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S329>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S329>/Switch' */
    if (rtb_UpperRelop_ir) {
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pd = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S319>/Gain1' incorporates:
   *  UnitDelay: '<S327>/Delay Input2'
   *
   * Block description for '<S327>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pd;

  /* Sum: '<S319>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S291>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S315>/Compare' incorporates:
   *  Constant: '<S315>/Constant'
   */
  rtb_Compare_ar = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S320>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ah;

  /* Gain: '<S320>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S330>/Delay Input2'
   *
   * Block description for '<S330>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* SampleTimeMath: '<S330>/sample time'
   *
   * About '<S330>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S330>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S291>/Unit Delay7' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE;

  /* UnitDelay: '<S291>/Unit Delay3' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_d;

  /* Sum: '<S291>/Add7' */
  rtb_Add4_j = rtb_deltafalllimit_ap - rtb_Add4_j;

  /* Product: '<S291>/Divide3' incorporates:
   *  Constant: '<S291>/steptime3'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S291>/Add3' incorporates:
   *  UnitDelay: '<S291>/Unit Delay7'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S291>/Add14' incorporates:
   *  UnitDelay: '<S291>/Unit Delay7'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE - rtb_Add7;

  /* Product: '<S291>/Divide7' incorporates:
   *  Constant: '<S291>/steptime7'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S331>/LowerRelop1' incorporates:
   *  Constant: '<S320>/Constant1'
   */
  rtb_UpperRelop_ir = (rtb_Add7 > 100.0F);

  /* Switch: '<S331>/Switch2' incorporates:
   *  Constant: '<S320>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S331>/UpperRelop' incorporates:
     *  Constant: '<S320>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S331>/Switch' incorporates:
     *  Constant: '<S320>/Constant'
     */
    if (rtb_UpperRelop_ir) {
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
  rtb_UpperRelop_ir = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S332>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S330>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S332>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S332>/Switch' */
    if (rtb_UpperRelop_ir) {
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S320>/Gain1' incorporates:
   *  UnitDelay: '<S330>/Delay Input2'
   *
   * Block description for '<S330>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* Sum: '<S320>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S291>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S316>/Compare' incorporates:
   *  Constant: '<S316>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.8F);

  /* Logic: '<S281>/Logical Operator' */
  rtb_LogicalOperator_idx_0 = (rtb_LogicalOperator7_m || rtb_AND_l);
  rtb_Compare_nd = (rtb_Compare_nd || rtb_AND2_e);
  rtb_Compare_b = (rtb_Compare_b || rtb_Compare_ar);
  rtb_LogicalOperator7_m = (rtb_Compare || rtb_UpperRelop_ir);

  /* UnitDelay: '<S210>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_lh;

  /* Sum: '<S292>/Add' */
  rtb_Switch2_mn = rtb_Gain3_o - rtb_Add10;

  /* Abs: '<S292>/Abs' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S333>/Compare' incorporates:
   *  Constant: '<S333>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S292>/AND3' */
  rtb_Compare_ar = (rtb_UpperRelop_ir && (VehCtrlMdel240926_2018b_amksp_B.Exit_h
    != 0.0));

  /* Sum: '<S292>/Add1' */
  rtb_Switch2_mn = rtb_CastToBoolean1 - rtb_Add10;

  /* Abs: '<S292>/Abs1' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S334>/Compare' incorporates:
   *  Constant: '<S334>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S292>/AND2' */
  rtb_AND2_e = (rtb_UpperRelop_ir && (VehCtrlMdel240926_2018b_amksp_B.Exit_o !=
    0.0));

  /* Sum: '<S292>/Add2' */
  rtb_Switch2_mn = rtb_deltafalllimit_cz - rtb_Add10;

  /* Abs: '<S292>/Abs2' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S335>/Compare' incorporates:
   *  Constant: '<S335>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S292>/AND' */
  rtb_AND_l = (rtb_UpperRelop_ir && (VehCtrlMdel240926_2018b_amksp_B.Exit_le !=
    0.0));

  /* Sum: '<S292>/Add3' */
  rtb_Add10 = rtb_deltafalllimit_ap - rtb_Add10;

  /* Abs: '<S292>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S336>/Compare' incorporates:
   *  Constant: '<S336>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 2.0F);

  /* Logic: '<S292>/AND1' */
  rtb_Compare = (rtb_UpperRelop_ir && (VehCtrlMdel240926_2018b_amksp_B.Exit_is
    != 0.0));

  /* Logic: '<S281>/Logical Operator1' */
  rtb_UpperRelop_ir = (rtb_Compare_ar && rtb_LogicalOperator_idx_0);
  rtb_Compare_nd = (rtb_AND2_e && rtb_Compare_nd);
  rtb_Compare_b = (rtb_AND_l && rtb_Compare_b);
  rtb_Compare = (rtb_Compare && rtb_LogicalOperator7_m);

  /* Chart: '<S281>/Timer' incorporates:
   *  Constant: '<S281>/Constant1'
   */
  VehCtrlMdel240926_20_Timer1(rtb_UpperRelop_ir, 0.5F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_c,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_o);

  /* Chart: '<S281>/Timer1' incorporates:
   *  Constant: '<S281>/Constant2'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_nd, 0.5F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_lh4,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_m);

  /* Chart: '<S281>/Timer2' incorporates:
   *  Constant: '<S281>/Constant3'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_b, 0.5F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_lh,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_g);

  /* Chart: '<S281>/Timer3' incorporates:
   *  Constant: '<S281>/Constant4'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare, 0.5F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_a,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer3_i);

  /* Logic: '<S279>/Logical Operator' */
  rtb_UpperRelop_ir = ((VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0));

  /* Logic: '<S279>/Logical Operator1' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* UnitDelay: '<S279>/Unit Delay4' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_m;

  /* Sum: '<S279>/Add1' */
  rtb_Add10 = Acc_POS_n - rtb_Add10;

  /* RelationalOperator: '<S284>/Compare' incorporates:
   *  Constant: '<S284>/Constant'
   */
  rtb_Compare_ar = (rtb_Add10 > 0.1F);

  /* Logic: '<S279>/Logical Operator2' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir || rtb_Compare_ar);

  /* Logic: '<S279>/AND' */
  rtb_Compare = ((VehCtrlMdel240926_2018b_amksp_B.CANUnpack_o1 != 0.0) &&
                 rtb_UpperRelop_ir);

  /* UnitDelay: '<S279>/Unit Delay3' */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_f;

  /* Logic: '<S279>/Logical Operator3' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S279>/Switch3' incorporates:
   *  UnitDelay: '<S279>/Unit Delay1'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S279>/Switch4' incorporates:
     *  Constant: '<S279>/InitZORE'
     */
    if (!rtb_Compare) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k = 0.0F;
    }

    /* End of Switch: '<S279>/Switch4' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_o =
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Switch: '<S279>/Switch3' */

  /* UnitDelay: '<S282>/Unit Delay3' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_p;

  /* Sum: '<S282>/Add5' incorporates:
   *  UnitDelay: '<S282>/Unit Delay1'
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d - rtb_Add10;

  /* Product: '<S282>/Divide3' incorporates:
   *  Constant: '<S282>/steptime3'
   */
  rtb_Add10 /= 0.01F;

  /* UnitDelay: '<S282>/Unit Delay2' */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_f;

  /* Sum: '<S282>/Add9' */
  rtb_Switch2_mn -= rtb_Add10;

  /* UnitDelay: '<S282>/Unit Delay4' */
  rtb_Add6 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn;

  /* Sum: '<S282>/Add6' incorporates:
   *  Constant: '<S282>/steptime4'
   */
  rtb_Add6 += 0.1F;

  /* Sum: '<S282>/Add8' incorporates:
   *  Constant: '<S282>/steptime6'
   */
  rtb_Add7 = rtb_Add6 + 2.0F;

  /* Product: '<S282>/Divide5' */
  rtb_Add7 = 1.0F / rtb_Add7 * rtb_Add6;

  /* Logic: '<S282>/Logical Operator' */
  rtb_LogicalOperator7_m = ((VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) ||
    (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) ||
    (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) ||
    (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0));

  /* Switch: '<S282>/Switch13' incorporates:
   *  Constant: '<S282>/Constant10'
   */
  if (rtb_LogicalOperator7_m) {
    rtb_Add4_j = rtb_Add7;
  } else {
    rtb_Add4_j = 1.0F;
  }

  /* End of Switch: '<S282>/Switch13' */

  /* Product: '<S282>/Divide6' */
  rtb_Switch2_mn *= rtb_Add4_j;

  /* Sum: '<S282>/Add10' */
  rtb_Ax = rtb_Switch2_mn + rtb_Add10;

  /* Switch: '<S279>/Switch1' */
  if (rtb_Compare) {
    /* Saturate: '<S279>/Saturation1' */
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d > 200.0F) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = 200.0F;
    } else {
      if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d < -10.0F) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = -10.0F;
      }
    }

    /* Product: '<S279>/Product' incorporates:
     *  Constant: '<S279>/steptime1'
     */
    rtb_Switch2_mn = rtb_Ax * 0.01F;

    /* Saturate: '<S279>/Saturation1' incorporates:
     *  Sum: '<S279>/Add'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d += rtb_Switch2_mn;
  } else {
    /* Saturate: '<S279>/Saturation1' incorporates:
     *  Constant: '<S279>/Constant'
     *  UnitDelay: '<S279>/Unit Delay'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = 0.0F;
  }

  /* End of Switch: '<S279>/Switch1' */

  /* Saturate: '<S279>/Saturation' */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d > 200.0F) {
    rtb_Add10 = 200.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d < -10.0F) {
    rtb_Add10 = -10.0F;
  } else {
    rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d;
  }

  /* End of Saturate: '<S279>/Saturation' */

  /* Sum: '<S279>/Add3' incorporates:
   *  UnitDelay: '<S279>/Unit Delay1'
   */
  rtb_VxIMU_est = rtb_Add10 +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_o;

  /* MinMax: '<S281>/Min1' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Gain3_o, rtb_CastToBoolean1);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_cz);
  rtb_Add10 = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_ap);

  /* Sum: '<S279>/Add2' */
  rtb_Add10 -= rtb_VxIMU_est;

  /* RelationalOperator: '<S285>/Compare' incorporates:
   *  Constant: '<S285>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.0F);

  /* Switch: '<S279>/Switch6' incorporates:
   *  Constant: '<S279>/Reset'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S279>/Add10' incorporates:
     *  Constant: '<S279>/Steptime'
     */
    rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i + 0.01F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S279>/Switch6' */

  /* MinMax: '<S279>/Min' incorporates:
   *  Constant: '<S279>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i = fminf(rtb_Add10, 0.1F);

  /* RelationalOperator: '<S279>/Relational Operator9' incorporates:
   *  Constant: '<S279>/ResetDelay'
   *  UnitDelay: '<S279>/Unit Delay2'
   */
  rtb_Compare_nd = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i >= 0.1F);

  /* RelationalOperator: '<S338>/Compare' incorporates:
   *  Constant: '<S338>/Constant'
   */
  rtb_Compare_b = (rtb_Ax < -0.5F);

  /* Chart: '<S282>/Timer2' incorporates:
   *  Constant: '<S282>/Constant15'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_b, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_i,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_j);

  /* UnitDelay: '<S342>/Delay Input2'
   *
   * Block description for '<S342>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;

  /* SampleTimeMath: '<S342>/sample time'
   *
   * About '<S342>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S342>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(10.0 * elapseTime);

  /* Sum: '<S343>/Add3' */
  rtb_Add4_j = ((rtb_deltafalllimit_ap + rtb_deltafalllimit_cz) +
                rtb_CastToBoolean1) + rtb_Gain3_o;

  /* MinMax: '<S343>/Min4' */
  rtb_Switch2_b0 = fminf(rtb_Gain3_o, rtb_CastToBoolean1);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_cz);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_ap);

  /* MinMax: '<S343>/Min3' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Gain3_o, rtb_CastToBoolean1);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_cz);
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p = fmaxf(rtb_MaxWhlSpd_mps_n,
    rtb_deltafalllimit_ap);

  /* Sum: '<S343>/Add4' incorporates:
   *  UnitDelay: '<S297>/Unit Delay'
   */
  rtb_Add4_j = (rtb_Add4_j - rtb_Switch2_b0) -
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p;

  /* Gain: '<S343>/Gain1' */
  rtb_Add4_j *= 0.5F;

  /* Sum: '<S342>/Difference Inputs1'
   *
   * Block description for '<S342>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add4_j -= rtb_Add10;

  /* RelationalOperator: '<S351>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add4_j > rtb_Switch2_mn);

  /* Switch: '<S351>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S342>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-10.0 * elapseTime);

    /* RelationalOperator: '<S351>/UpperRelop' */
    rtb_Compare_b = (rtb_Add4_j < rtb_Switch2_mn);

    /* Switch: '<S351>/Switch' */
    if (rtb_Compare_b) {
      rtb_Add4_j = rtb_Switch2_mn;
    }

    /* End of Switch: '<S351>/Switch' */
    rtb_Switch2_mn = rtb_Add4_j;
  }

  /* End of Switch: '<S351>/Switch2' */

  /* Sum: '<S342>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S342>/Delay Input2'
   *
   * Block description for '<S342>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S342>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g = rtb_Switch2_mn +
    rtb_Add10;

  /* RelationalOperator: '<S337>/Compare' incorporates:
   *  Constant: '<S337>/Constant'
   */
  rtb_Compare_b = (rtb_Ax > 0.5F);

  /* Chart: '<S282>/Timer1' incorporates:
   *  Constant: '<S282>/Constant14'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_b, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_l,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_p);

  /* Logic: '<S282>/Logical Operator2' */
  rtb_UpperRelop_ir = !(VehCtrlMdel240926_2018b_amksp_B.Exit_l != 0.0);

  /* Switch: '<S282>/Switch6' incorporates:
   *  Switch: '<S282>/Switch4'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S282>/Switch5' incorporates:
     *  UnitDelay: '<S342>/Delay Input2'
     *
     * Block description for '<S342>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_i != 0.0) {
      /* Switch: '<S282>/Switch11' incorporates:
       *  Constant: '<S282>/Constant7'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0) {
        rtb_Switch2_mn = rtb_deltafalllimit_ap;
      } else {
        rtb_Switch2_mn = 0.0F;
      }

      /* End of Switch: '<S282>/Switch11' */

      /* Switch: '<S282>/Switch10' incorporates:
       *  Constant: '<S282>/Constant6'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) {
        rtb_Add10 = rtb_deltafalllimit_cz;
      } else {
        rtb_Add10 = 0.0F;
      }

      /* End of Switch: '<S282>/Switch10' */

      /* Switch: '<S282>/Switch9' incorporates:
       *  Constant: '<S282>/Constant5'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) {
        rtb_Add4_j = rtb_CastToBoolean1;
      } else {
        rtb_Add4_j = 0.0F;
      }

      /* End of Switch: '<S282>/Switch9' */

      /* Switch: '<S282>/Switch8' incorporates:
       *  Constant: '<S282>/Constant4'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) {
        rtb_Switch2_b0 = rtb_Gain3_o;
      } else {
        rtb_Switch2_b0 = 0.0F;
      }

      /* End of Switch: '<S282>/Switch8' */

      /* MinMax: '<S282>/Min1' */
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Switch2_b0, rtb_Add4_j);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Add10);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Switch2_mn);
    } else {
      rtb_MaxWhlSpd_mps_n = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;
    }

    /* End of Switch: '<S282>/Switch5' */
  } else {
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0) {
      /* Switch: '<S282>/Switch4' */
      rtb_Switch2_mn = rtb_deltafalllimit_ap;
    } else {
      /* Switch: '<S282>/Switch4' incorporates:
       *  Constant: '<S282>/Constant3'
       */
      rtb_Switch2_mn = 9999.0F;
    }

    /* Switch: '<S282>/Switch3' incorporates:
     *  Constant: '<S282>/Constant2'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) {
      rtb_Add10 = rtb_deltafalllimit_cz;
    } else {
      rtb_Add10 = 9999.0F;
    }

    /* End of Switch: '<S282>/Switch3' */

    /* Switch: '<S282>/Switch2' incorporates:
     *  Constant: '<S282>/Constant1'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) {
      rtb_Add4_j = rtb_CastToBoolean1;
    } else {
      rtb_Add4_j = 9999.0F;
    }

    /* End of Switch: '<S282>/Switch2' */

    /* Switch: '<S282>/Switch1' incorporates:
     *  Constant: '<S282>/Constant'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) {
      rtb_Switch2_b0 = rtb_Gain3_o;
    } else {
      rtb_Switch2_b0 = 9999.0F;
    }

    /* End of Switch: '<S282>/Switch1' */

    /* MinMax: '<S282>/Min2' */
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add4_j);
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add10);
    rtb_MaxWhlSpd_mps_n = fminf(rtb_Switch2_b0, rtb_Switch2_mn);
  }

  /* End of Switch: '<S282>/Switch6' */

  /* Logic: '<S282>/NOT3' */
  rtb_UpperRelop_ir = !rtb_LogicalOperator7_m;

  /* Logic: '<S282>/Logical Operator3' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir && rtb_Compare_nd);

  /* Logic: '<S282>/NOT4' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S282>/Switch7' incorporates:
   *  UnitDelay: '<S342>/Delay Input2'
   *
   * Block description for '<S342>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (rtb_UpperRelop_ir) {
    /* Logic: '<S282>/Logical Operator1' */
    rtb_Compare_nd = (rtb_Compare_nd || rtb_LogicalOperator7_m);

    /* Switch: '<S282>/Switch' */
    if (rtb_Compare_nd) {
      rtb_VxIMU_est = rtb_MaxWhlSpd_mps_n;
    }

    /* End of Switch: '<S282>/Switch' */
  } else {
    rtb_VxIMU_est = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;
  }

  /* End of Switch: '<S282>/Switch7' */

  /* UnitDelay: '<S340>/Delay Input2'
   *
   * Block description for '<S340>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a;

  /* Sum: '<S340>/Difference Inputs1'
   *
   * Block description for '<S340>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_VxIMU_est -= rtb_Add10;

  /* Switch: '<S282>/Switch12' incorporates:
   *  Constant: '<S282>/Constant8'
   *  Constant: '<S282>/Constant9'
   */
  if (rtb_LogicalOperator7_m) {
    rtb_Switch2_mn = 0.1F;
  } else {
    rtb_Switch2_mn = 0.05F;
  }

  /* End of Switch: '<S282>/Switch12' */

  /* Sum: '<S282>/Add4' */
  rtb_Add4_j = rtb_Ax + rtb_Switch2_mn;

  /* SampleTimeMath: '<S340>/sample time'
   *
   * About '<S340>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S340>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S349>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_VxIMU_est > rtb_Switch2_b0);

  /* Sum: '<S282>/Add3' */
  rtb_Ax -= rtb_Switch2_mn;

  /* Switch: '<S349>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S340>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S349>/UpperRelop' */
    rtb_LogicalOperator7_m = (rtb_VxIMU_est < rtb_Switch2_mn);

    /* Switch: '<S349>/Switch' */
    if (rtb_LogicalOperator7_m) {
      rtb_VxIMU_est = rtb_Switch2_mn;
    }

    /* End of Switch: '<S349>/Switch' */
    rtb_Switch2_b0 = rtb_VxIMU_est;
  }

  /* End of Switch: '<S349>/Switch2' */

  /* Sum: '<S340>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S340>/Delay Input2'
   *
   * Block description for '<S340>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S340>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a = rtb_Switch2_b0 +
    rtb_Add10;

  /* RelationalOperator: '<S347>/LowerRelop1' incorporates:
   *  Constant: '<S339>/Constant1'
   *  UnitDelay: '<S340>/Delay Input2'
   *
   * Block description for '<S340>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a >
                       100.0F);

  /* Switch: '<S347>/Switch2' incorporates:
   *  Constant: '<S339>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_mn = 100.0F;
  } else {
    /* RelationalOperator: '<S347>/UpperRelop' incorporates:
     *  Constant: '<S339>/Constant'
     *  UnitDelay: '<S340>/Delay Input2'
     *
     * Block description for '<S340>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_LogicalOperator7_m =
      (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a < 0.0F);

    /* Switch: '<S347>/Switch' incorporates:
     *  Constant: '<S339>/Constant'
     *  UnitDelay: '<S340>/Delay Input2'
     *
     * Block description for '<S340>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_LogicalOperator7_m) {
      rtb_Switch2_mn = 0.0F;
    } else {
      rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a;
    }

    /* End of Switch: '<S347>/Switch' */
  }

  /* End of Switch: '<S347>/Switch2' */

  /* UnitDelay: '<S346>/Delay Input2'
   *
   * Block description for '<S346>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* Sum: '<S346>/Difference Inputs1'
   *
   * Block description for '<S346>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_Switch2_mn - rtb_Add10;

  /* SampleTimeMath: '<S346>/sample time'
   *
   * About '<S346>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S346>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(15.0 * elapseTime);

  /* RelationalOperator: '<S348>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S348>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S346>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-15.0 * elapseTime);

    /* RelationalOperator: '<S348>/UpperRelop' */
    rtb_LogicalOperator7_m = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S348>/Switch' */
    if (rtb_LogicalOperator7_m) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S348>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S348>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f = rtb_Switch2_mn +
    rtb_Add10;

  /* UnitDelay: '<S339>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ncs;

  /* Gain: '<S339>/Gain' */
  rtb_Add10 *= 0.0F;

  /* Saturate: '<S31>/Saturation' incorporates:
   *  Sum: '<S339>/Add'
   *  UnitDelay: '<S346>/Delay Input2'
   *
   * Block description for '<S346>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehVxEst_mps = rtb_Add10 +
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* SampleTimeMath: '<S341>/sample time'
   *
   * About '<S341>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* UnitDelay: '<S341>/Delay Input2'
   *
   * Block description for '<S341>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu;

  /* Sum: '<S341>/Difference Inputs1'
   *
   * Block description for '<S341>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_MaxWhlSpd_mps_n - rtb_Add10;

  /* Product: '<S341>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S350>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S350>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S341>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S350>/UpperRelop' */
    rtb_LogicalOperator7_m = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S350>/Switch' */
    if (rtb_LogicalOperator7_m) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S350>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S350>/Switch2' */

  /* Sum: '<S341>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S341>/Delay Input2'
   *
   * Block description for '<S341>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S341>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu = rtb_Switch2_mn +
    rtb_Add10;

  /* Sum: '<S282>/Add7' incorporates:
   *  Constant: '<S282>/steptime5'
   */
  rtb_Add7 = 1.0F - rtb_Add7;

  /* Product: '<S282>/Divide4' incorporates:
   *  UnitDelay: '<S282>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn = rtb_Add7 * rtb_Add6;

  /* Update for MinMax: '<S343>/Min3' incorporates:
   *  UnitDelay: '<S297>/Unit Delay'
   *  UnitDelay: '<S301>/Delay Input2'
   *
   * Block description for '<S301>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* Update for UnitDelay: '<S290>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_j = rtb_Gain3_o;

  /* Update for UnitDelay: '<S298>/Unit Delay' incorporates:
   *  UnitDelay: '<S304>/Delay Input2'
   *
   * Block description for '<S304>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* Update for UnitDelay: '<S290>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n = rtb_CastToBoolean1;

  /* Update for UnitDelay: '<S299>/Unit Delay' incorporates:
   *  UnitDelay: '<S307>/Delay Input2'
   *
   * Block description for '<S307>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* Update for UnitDelay: '<S290>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_l = rtb_deltafalllimit_cz;

  /* Update for UnitDelay: '<S300>/Unit Delay' incorporates:
   *  UnitDelay: '<S310>/Delay Input2'
   *
   * Block description for '<S310>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nc =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* Update for UnitDelay: '<S290>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE = rtb_deltafalllimit_ap;

  /* Update for UnitDelay: '<S317>/Unit Delay' incorporates:
   *  UnitDelay: '<S321>/Delay Input2'
   *
   * Block description for '<S321>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* Update for UnitDelay: '<S291>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a0 = rtb_Gain3_o;

  /* Update for UnitDelay: '<S318>/Unit Delay' incorporates:
   *  UnitDelay: '<S324>/Delay Input2'
   *
   * Block description for '<S324>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ap =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* Update for UnitDelay: '<S291>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_a = rtb_CastToBoolean1;

  /* Update for UnitDelay: '<S319>/Unit Delay' incorporates:
   *  UnitDelay: '<S327>/Delay Input2'
   *
   * Block description for '<S327>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pd;

  /* Update for UnitDelay: '<S291>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_c = rtb_deltafalllimit_cz;

  /* Update for UnitDelay: '<S320>/Unit Delay' incorporates:
   *  UnitDelay: '<S330>/Delay Input2'
   *
   * Block description for '<S330>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ah =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* Update for UnitDelay: '<S291>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_d = rtb_deltafalllimit_ap;

  /* Update for UnitDelay: '<S210>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_lh = VehVxEst_mps;

  /* Update for UnitDelay: '<S279>/Unit Delay4' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_m = Acc_POS_n;

  /* Update for UnitDelay: '<S210>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k = VehVxEst_mps;

  /* Update for UnitDelay: '<S279>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_f = rtb_Compare;

  /* Update for UnitDelay: '<S282>/Unit Delay3' incorporates:
   *  UnitDelay: '<S282>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_p =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d;

  /* Update for UnitDelay: '<S282>/Unit Delay1' incorporates:
   *  UnitDelay: '<S341>/Delay Input2'
   *
   * Block description for '<S341>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu;

  /* Update for UnitDelay: '<S282>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_f = rtb_CastToDouble;

  /* Update for UnitDelay: '<S339>/Unit Delay' incorporates:
   *  UnitDelay: '<S346>/Delay Input2'
   *
   * Block description for '<S346>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ncs =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms1' */

  /* S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* Logic: '<S108>/NOT' */
  rtb_Compare = !(Trq_CUT != 0.0);

  /* Logic: '<S108>/AND' */
  rtb_LogicalOperator7_m = ((KeyPressed != 0.0) && rtb_Compare);

  /* RelationalOperator: '<S110>/Compare' incorporates:
   *  Constant: '<S110>/Constant'
   */
  Brk = (Brk_F >= 600);

  /* RelationalOperator: '<S111>/Compare' incorporates:
   *  Constant: '<S111>/Constant'
   */
  ACC_Release = (Acc_POS_n <= 50.0F);

  /* Logic: '<S108>/NOT1' */
  rtb_Compare = !(VehCtrlMdel240926_2018b_amksp_B.AMKSWITCH_bx != 0.0);

  /* Switch: '<S108>/Switch' incorporates:
   *  Constant: '<S108>/Constant1'
   *  Switch: '<S108>/Switch10'
   *  Switch: '<S108>/Switch11'
   *  Switch: '<S108>/Switch3'
   */
  if (rtb_Compare) {
    elapseTime = MCFL_bSystemReady;
    rtb_Yk1_l = MCFR_bSystemReady;
    WhlSpdFR = MCFL_bQuitInverterOn;
    VehCtrlMdel240926_2018b_amksp_B.Switch11 = MCFR_bQuitInverterOn;
  } else {
    elapseTime = 1.0;
    rtb_Yk1_l = 1.0;
    WhlSpdFR = 1.0;
    VehCtrlMdel240926_2018b_amksp_B.Switch11 = 1.0;
  }

  /* End of Switch: '<S108>/Switch' */

  /* Chart: '<S108>/Chart2' */
  FunctionCallSubsystem_ELAPS_T =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3 -
    VehCtrlMdel240926_2018b_amks_DW.previousTicks_g;
  VehCtrlMdel240926_2018b_amks_DW.previousTicks_g =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
  if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_f +
      FunctionCallSubsystem_ELAPS_T <= 255U) {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_f = (uint8_T)
      (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_f +
       FunctionCallSubsystem_ELAPS_T);
  } else {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_f = MAX_uint8_T;
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
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_f = 0U;
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
      VehCtrlMdel240926_2018b_VehStat(&controller_ready, &rtb_LogicalOperator7_m,
        &elapseTime, &rtb_Yk1_l, &WhlSpdFR);
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
      VehCtrlMdel240926_20_AMKDCready(&MCFL_bDCOn, &MCFR_bDCOn,
        &rtb_LogicalOperator7_m, &elapseTime, &rtb_Yk1_l, &WhlSpdFR);
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

  /* End of Chart: '<S108>/Chart2' */

  /* Switch: '<S108>/Switch4' */
  VehCtrlMdel240926_2018b_amksp_B.MCFL_DCOn_setpoints_o = (rtb_Compare &&
    MCFL_DCOn_setpoints);

  /* End of Outputs for S-Function (fcncallgen): '<S2>/10ms' */

  /* S-Function (fcncallgen): '<S5>/10ms1' incorporates:
   *  SubSystem: '<S5>/Beeper'
   */
  /* Logic: '<S352>/Logical Operator2' */
  rtb_LogicalOperator2 = !rtb_LogicalOperator2;

  /* S-Function (ec5744_pdsslb2u3): '<S352>/PowerDriverSwitch(LS)' */
  L9826VAR701[2]= beeper_state;
  ec_l9826tr701_control(L9826VAR701);

  /* S-Function (ec5744_pdsslb2u3): '<S352>/PowerDriverSwitch(LS)2' */
  L9826VAR701[6]= beeper_state;
  ec_l9826tr701_control(L9826VAR701);

  /* Chart: '<S352>/Timer2' incorporates:
   *  Constant: '<S352>/Constant3'
   */
  VehCtrlMdel240926_20_Timer1(rtb_LogicalOperator2, 1.0F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_h);

  /* RelationalOperator: '<S364>/Compare' incorporates:
   *  Constant: '<S364>/Constant'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amksp_B.Exit > 0.0);

  /* RelationalOperator: '<S358>/FixPt Relational Operator' incorporates:
   *  UnitDelay: '<S358>/Delay Input1'
   *
   * Block description for '<S358>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE = ((int32_T)rtb_Compare >
    (int32_T)VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE);

  /* Logic: '<S352>/OR' incorporates:
   *  UnitDelay: '<S358>/Delay Input1'
   *
   * Block description for '<S358>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_LogicalOperator2 = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE;

  /* Chart: '<S352>/Chart' */
  FunctionCallSubsystem_ELAPS_T =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3 -
    VehCtrlMdel240926_2018b_amks_DW.previousTicks_m;
  VehCtrlMdel240926_2018b_amks_DW.previousTicks_m =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
  if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_p +
      FunctionCallSubsystem_ELAPS_T <= 31U) {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_p = (uint8_T)
      (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_p +
       FunctionCallSubsystem_ELAPS_T);
  } else {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_p = 31U;
  }

  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c24_VehCtrlMdel240926
      == 0U) {
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c24_VehCtrlMdel240926 =
      1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c24_VehCtrlMdel240926_2018b_ =
      2U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON = 2U;
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_p = 0U;
    HVSWITCH = false;
  } else if
      (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c24_VehCtrlMdel240926_2018b_
       == VehCtrlMdel240926_2_IN_STATEOFF) {
    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEOFF ==
        VehCtrlMdel240926_2018b__IN_OFF) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEOFF = 0U;
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c24_VehCtrlMdel240926_2018b_
        = 2U;
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON = 2U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_p = 0U;
      HVSWITCH = false;
    } else {
      /* case IN_initstate1: */
      HVSWITCH = true;
      rtb_LogicalOperator2 =
        ((VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_p >= 20U) &&
         rtb_LogicalOperator2);
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEOFF = 1U;
        HVSWITCH = false;
      }
    }
  } else {
    /* case IN_STATEON: */
    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON ==
        VehCtrlMdel240926_2018b_IN_ON_d) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON = 0U;
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c24_VehCtrlMdel240926_2018b_
        = 1U;
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEOFF = 2U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_p = 0U;
      HVSWITCH = true;
    } else {
      /* case IN_initstate: */
      HVSWITCH = false;
      rtb_LogicalOperator2 =
        ((VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_p >= 20U) &&
         rtb_LogicalOperator2);
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON = 1U;
        HVSWITCH = true;
      }
    }
  }

  /* End of Chart: '<S352>/Chart' */

  /* S-Function (ec5744_pdsslbu3): '<S352>/PowerDriverSwitch(HS)2' */

  /* Set level HVSWITCH for the specified power driver switch */
  ec_gpio_write(57,HVSWITCH);

  /* Logic: '<S352>/OR2' */
  rtb_LogicalOperator7_m = ((MCFL_bError != 0.0) || (MCFR_bError != 0.0));

  /* Outputs for Enabled SubSystem: '<S352>/Enabled Subsystem1' incorporates:
   *  EnablePort: '<S359>/Enable'
   */
  if (rtb_LogicalOperator7_m) {
    if (!VehCtrlMdel240926_2018b_amks_DW.EnabledSubsystem1_MODE) {
      /* Enable for Chart: '<S359>/Chart' */
      VehCtrlMdel240926_2018b_amks_DW.previousTicks =
        VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
      VehCtrlMdel240926_2018b_amks_DW.EnabledSubsystem1_MODE = true;
    }

    /* Chart: '<S359>/Chart' */
    FunctionCallSubsystem_ELAPS_T =
      VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3 -
      VehCtrlMdel240926_2018b_amks_DW.previousTicks;
    VehCtrlMdel240926_2018b_amks_DW.previousTicks =
      VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
    if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 +
        FunctionCallSubsystem_ELAPS_T <= 127U) {
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = (uint8_T)
        (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 +
         FunctionCallSubsystem_ELAPS_T);
    } else {
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 127U;
    }

    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c27_VehCtrlMdel240926
        == 0U) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c27_VehCtrlMdel240926
        = 1U;
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel240926_2018b_
        = 1U;
    } else {
      switch
        (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel240926_2018b_)
      {
       case VehCtrlMdel240926_201_IN_Init_e:
        rtb_LogicalOperator2 = ((MCFL_bError != 0.0) && (!(MCFR_bError != 0.0)));
        if (rtb_LogicalOperator2) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel240926_2018b_
            = 2U;
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateA = 2U;
          VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
          VehCtrlMdel240926_2018b_amksp_B.LEDOn = 0.0;
        } else {
          rtb_LogicalOperator2 = ((!(MCFL_bError != 0.0)) && (MCFR_bError != 0.0));
          if (rtb_LogicalOperator2) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel240926_2018b_
              = 3U;
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateB = 2U;
            VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
            VehCtrlMdel240926_2018b_amksp_B.LEDOn = 0.0;
          } else {
            rtb_LogicalOperator2 = ((MCFL_bError != 0.0) && (MCFR_bError != 0.0));
            if (rtb_LogicalOperator2) {
              VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel240926_2018b_
                = 4U;
              VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateC = 2U;
              VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
              VehCtrlMdel240926_2018b_amksp_B.LEDOn = 0.0;
            }
          }
        }
        break;

       case VehCtrlMdel240926_201_IN_StateA:
        rtb_LogicalOperator2 = ((!(MCFL_bError != 0.0)) || (MCFR_bError != 0.0));
        if (rtb_LogicalOperator2) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateA = 0U;
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel240926_2018b_
            = 1U;
        } else if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateA ==
                   VehCtrlMdel240926_201_IN_LEDOFF) {
          VehCtrlMdel240926_2018b_amksp_B.LEDOn = 1.0;
          if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 >= 80U) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateA = 2U;
            VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
            VehCtrlMdel240926_2018b_amksp_B.LEDOn = 0.0;
          }
        } else {
          /* case IN_LEDON: */
          VehCtrlMdel240926_2018b_amksp_B.LEDOn = 0.0;
          if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 >= 80U) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateA = 1U;
            VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
            VehCtrlMdel240926_2018b_amksp_B.LEDOn = 1.0;
          }
        }
        break;

       case VehCtrlMdel240926_201_IN_StateB:
        rtb_LogicalOperator2 = ((MCFL_bError != 0.0) || (!(MCFR_bError != 0.0)));
        if (rtb_LogicalOperator2) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateB = 0U;
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel240926_2018b_
            = 1U;
        } else if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateB ==
                   VehCtrlMdel240926_201_IN_LEDOFF) {
          VehCtrlMdel240926_2018b_amksp_B.LEDOn = 1.0;
          if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 >= 40U) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateB = 2U;
            VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
            VehCtrlMdel240926_2018b_amksp_B.LEDOn = 0.0;
          }
        } else {
          /* case IN_LEDON: */
          VehCtrlMdel240926_2018b_amksp_B.LEDOn = 0.0;
          if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 >= 40U) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateB = 1U;
            VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
            VehCtrlMdel240926_2018b_amksp_B.LEDOn = 1.0;
          }
        }
        break;

       default:
        /* case IN_StateC: */
        rtb_LogicalOperator2 = ((!(MCFL_bError != 0.0)) || (!(MCFR_bError != 0.0)));
        if (rtb_LogicalOperator2) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateC = 0U;
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel240926_2018b_
            = 1U;
        } else if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateC ==
                   VehCtrlMdel240926_201_IN_LEDOFF) {
          VehCtrlMdel240926_2018b_amksp_B.LEDOn = 1.0;
          if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 >= 10U) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateC = 2U;
            VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
            VehCtrlMdel240926_2018b_amksp_B.LEDOn = 0.0;
          }
        } else {
          /* case IN_LEDON: */
          VehCtrlMdel240926_2018b_amksp_B.LEDOn = 0.0;
          if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 >= 10U) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateC = 1U;
            VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
            VehCtrlMdel240926_2018b_amksp_B.LEDOn = 1.0;
          }
        }
        break;
      }
    }

    /* End of Chart: '<S359>/Chart' */
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.EnabledSubsystem1_MODE) {
      /* Disable for Chart: '<S359>/Chart' */
      FunctionCallSubsystem_ELAPS_T =
        VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3 -
        VehCtrlMdel240926_2018b_amks_DW.previousTicks;
      VehCtrlMdel240926_2018b_amks_DW.previousTicks =
        VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
      if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 +
          FunctionCallSubsystem_ELAPS_T <= 127U) {
        VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = (uint8_T)
          (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 +
           FunctionCallSubsystem_ELAPS_T);
      } else {
        VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 127U;
      }

      /* End of Disable for Chart: '<S359>/Chart' */
      VehCtrlMdel240926_2018b_amks_DW.EnabledSubsystem1_MODE = false;
    }
  }

  /* End of Outputs for SubSystem: '<S352>/Enabled Subsystem1' */

  /* Logic: '<S352>/Logical Operator5' incorporates:
   *  UnitDelay: '<S358>/Delay Input1'
   *
   * Block description for '<S358>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE = !rtb_LogicalOperator7_m;

  /* Switch: '<S352>/Switch1' incorporates:
   *  UnitDelay: '<S358>/Delay Input1'
   *
   * Block description for '<S358>/Delay Input1':
   *
   *  Store in Global RAM
   */
  STATEDISPLAY = (VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE ||
                  (VehCtrlMdel240926_2018b_amksp_B.LEDOn != 0.0));

  /* S-Function (ec5744_pdsslb2u3): '<S352>/PowerDriverSwitch(LS)1' */
  L9826VAR701[3]= STATEDISPLAY;
  ec_l9826tr701_control(L9826VAR701);

  /* S-Function (ec5744_pdsslb2u3): '<S352>/PowerDriverSwitch(LS)5' */
  L9826VAR702[1]= STATEDISPLAY;
  ec_l9826tr702_control(L9826VAR702);

  /* Logic: '<S352>/AND' */
  VehCtrlMdel240926_2018b_amksp_B.AND = (rtb_Compare_am && rtb_LowerRelop1_b);

  /* S-Function (ec5744_pdsslb2u3): '<S352>/PowerDriverSwitch(LS)3' */
  L9826VAR702[6]= VehCtrlMdel240926_2018b_amksp_B.AND;
  ec_l9826tr702_control(L9826VAR702);

  /* S-Function (ec5744_pdsslb2u3): '<S352>/PowerDriverSwitch(LS)4' */
  L9826VAR702[3]= VehCtrlMdel240926_2018b_amksp_B.AND;
  ec_l9826tr702_control(L9826VAR702);

  /* Update for UnitDelay: '<S358>/Delay Input1'
   *
   * Block description for '<S358>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE = rtb_Compare;

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms1' */

  /* S-Function (fcncallgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/PwrTrainTempPrtct'
   */
  /* MinMax: '<S8>/Max' */
  elapseTime = fmax(MCFL_TempIGBT, MCFR_TempIGBT);
  elapseTime = fmax(elapseTime, MCFL_TempInverter);
  elapseTime = fmax(elapseTime, MCFR_TempInverter);

  /* RelationalOperator: '<S98>/Compare' incorporates:
   *  Constant: '<S98>/Constant'
   */
  rtb_LowerRelop1_b = (elapseTime > 35.0);

  /* RelationalOperator: '<S99>/Compare' incorporates:
   *  Constant: '<S99>/Constant'
   */
  rtb_LogicalOperator2 = (elapseTime > 45.0);

  /* Logic: '<S8>/NOT' */
  rtb_Compare = !rtb_LogicalOperator2;

  /* Logic: '<S8>/AND' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b && rtb_Compare);

  /* Switch: '<S8>/Switch' incorporates:
   *  Constant: '<S8>/Constant'
   */
  if (rtb_LowerRelop1_b) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table1' */
    WhlSpdFR = look1_binlx(elapseTime, VehCtrlMdel240926_2018b__ConstP.pooled19,
      VehCtrlMdel240926_2018b__ConstP.pooled18, 7U);
  } else {
    WhlSpdFR = 0.0;
  }

  /* End of Switch: '<S8>/Switch' */

  /* MinMax: '<S8>/Max2' */
  rtb_Yk1_l = fmax(MCFL_TempMotor, MCFR_TempMotor);

  /* RelationalOperator: '<S103>/Compare' incorporates:
   *  Constant: '<S103>/Constant'
   */
  rtb_Compare = (rtb_Yk1_l > 45.0);

  /* Logic: '<S8>/NOT2' */
  rtb_LowerRelop1_b = !rtb_Compare;

  /* RelationalOperator: '<S104>/Compare' incorporates:
   *  Constant: '<S104>/Constant'
   */
  rtb_Compare_am = (rtb_Yk1_l > 35.0);

  /* Logic: '<S8>/AND3' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b && rtb_Compare_am);

  /* Switch: '<S8>/Switch2' incorporates:
   *  Constant: '<S8>/Constant4'
   */
  if (rtb_LowerRelop1_b) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table4' */
    WhlSpdRL_mps = look1_binlx(rtb_Yk1_l,
      VehCtrlMdel240926_2018b__ConstP.pooled19,
      VehCtrlMdel240926_2018b__ConstP.pooled18, 7U);
  } else {
    WhlSpdRL_mps = 0.0;
  }

  /* End of Switch: '<S8>/Switch2' */

  /* MinMax: '<S8>/Max3' */
  WhlSpdRL_mps = fmax(WhlSpdRL_mps, WhlSpdFR);

  /* SignalConversion generated from: '<S8>/Out1' */
  WhlSpdRR_mps = WhlSpdRL_mps;

  /* Logic: '<S8>/AND2' */
  rtb_Compare_am = (rtb_Compare && rtb_LogicalOperator2);

  /* Chart: '<S8>/Timer1' incorporates:
   *  Constant: '<S8>/Constant2'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_g,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1);

  /* RelationalOperator: '<S101>/Compare' incorporates:
   *  Constant: '<S101>/Constant'
   */
  rtb_Compare_am = (MCU_Temp > 80.0);

  /* Chart: '<S8>/Timer2' incorporates:
   *  Constant: '<S8>/Constant3'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_d,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2);

  /* SignalConversion generated from: '<S8>/Out1' */
  EMRAX_Trq_CUT = VehCtrlMdel240926_2018b_amksp_B.Exit_d;

  /* Logic: '<S8>/NOT1' */
  rtb_Compare_am = !rtb_Compare_am;

  /* RelationalOperator: '<S100>/Compare' incorporates:
   *  Constant: '<S100>/Constant'
   */
  rtb_Compare = (MCU_Temp > 45.0);

  /* Logic: '<S8>/AND1' */
  rtb_Compare = (rtb_Compare && rtb_Compare_am);

  /* MinMax: '<S8>/Max1' */
  rtb_g_mpss1 = fmax(MCU_Temp, motor_Temp);

  /* Switch: '<S8>/Switch1' incorporates:
   *  Constant: '<S8>/Constant1'
   */
  if (rtb_Compare) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table3' */
    WhlSpdRL_mps = look1_binlx(rtb_g_mpss1,
      VehCtrlMdel240926_2018b__ConstP.uDLookupTable3_bp01Data,
      VehCtrlMdel240926_2018b__ConstP.pooled18, 7U);
  } else {
    WhlSpdRL_mps = 0.0;
  }

  /* End of Switch: '<S8>/Switch1' */

  /* SignalConversion generated from: '<S8>/Out1' */
  WhlSpdFR = WhlSpdRL_mps;

  /* Lookup_n-D: '<S8>/2-D Lookup Table2' */
  WhlSpdRL_mps = look1_binlx(rtb_g_mpss1,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable2_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable2_tableData, 6U);

  /* DataTypeConversion: '<S8>/Cast To Single' */
  rtb_CastToDouble = (real32_T)WhlSpdRL_mps;

  /* Gain: '<S8>/Gain' */
  rtb_CastToDouble *= 10.0F;

  /* SignalConversion generated from: '<S8>/Out1' */
  AMK_Trq_CUT = VehCtrlMdel240926_2018b_amksp_B.Exit_g;

  /* RelationalOperator: '<S105>/Compare' incorporates:
   *  Constant: '<S105>/Constant'
   */
  rtb_Compare_am = (rtb_Yk1_l > 50.0);

  /* SignalConversion generated from: '<S8>/Out1' */
  VehCtrlMdel240926_2018b_amksp_B.bWaterPumpON = rtb_Compare_am;

  /* RelationalOperator: '<S102>/Compare' incorporates:
   *  Constant: '<S102>/Constant'
   */
  rtb_Compare_am = (elapseTime > 30.0);

  /* SignalConversion generated from: '<S8>/Out1' */
  VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON = rtb_Compare_am;

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

  /* DataTypeConversion: '<S28>/Cast To Single' */
  rtb_Add7 = (real32_T)MCFL_ActualTorque;
  rtb_Add6 = (real32_T)MCFR_ActualTorque;
  rtb_Switch2_mn = (real32_T)trq;

  /* DataTypeConversion: '<S28>/Cast To Single1' */
  rtb_Switch2_b0 = (real32_T)MCFL_ActualVelocity;
  rtb_Add10 = (real32_T)MCFR_ActualVelocity;
  rtb_Add4_j = (real32_T)RPM;

  /* Product: '<S28>/Product6' */
  rtb_Add7 *= rtb_Switch2_b0;
  rtb_Add7 += rtb_Add10 * rtb_Add6;
  rtb_Add7 += rtb_Add4_j * rtb_Switch2_mn;

  /* Gain: '<S28>/Gain3' */
  PwrALL = 0.000104712039F * rtb_Add7;

  /* Logic: '<S7>/OR3' */
  rtb_Compare_am = ((VehCtrlMdel240926_2018b_amksp_B.AMKSWITCH_bx != 0.0) ||
                    rtb_ignition_e);

  /* UnitDelay: '<S81>/Unit Delay1' */
  rtb_ignition_e = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl;

  /* Saturate: '<S31>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    rtb_Add7 = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    rtb_Add7 = 0.0F;
  } else {
    rtb_Add7 = VehVxEst_mps;
  }

  /* Lookup_n-D: '<S31>/VehSpd_SlipTarget_mps' */
  rtb_Switch2_mn = look1_iflf_binlc(rtb_Add7,
    VehCtrlMdel240926_2018b__ConstP.pooled39,
    VehCtrlMdel240926_2018b__ConstP.pooled38, 3U);

  /* Sum: '<S31>/Add9' */
  rtb_Switch2_mn += rtb_Add7;

  /* Saturate: '<S31>/Saturation1' */
  if (rtb_CastToBoolean1 > 50.0F) {
    rtb_CastToBoolean1 = 50.0F;
  } else {
    if (rtb_CastToBoolean1 < 0.0F) {
      rtb_CastToBoolean1 = 0.0F;
    }
  }

  /* End of Saturate: '<S31>/Saturation1' */

  /* Sum: '<S31>/Add1' */
  rtb_Add6 = rtb_Switch2_mn - rtb_CastToBoolean1;

  /* RelationalOperator: '<S31>/Relational Operator7' incorporates:
   *  Constant: '<S31>/Cal_DeltaV_mps'
   */
  rtb_LogicalOperator7_m = (rtb_Add6 < 0.0F);

  /* Logic: '<S81>/Logical Operator4' */
  rtb_ignition_e = ((!rtb_ignition_e) && (!rtb_LogicalOperator7_m));

  /* Logic: '<S31>/Logical Operator2' */
  rtb_LogicalOperator7_m = !rtb_LogicalOperator7_m;

  /* UnitDelay: '<S31>/Unit Delay4' */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i;

  /* RelationalOperator: '<S31>/Relational Operator8' incorporates:
   *  Constant: '<S31>/Cal_DeltaV_mps1'
   */
  TrqR_cmd_raw = (rtb_Switch2_mn > 235.0F);

  /* Logic: '<S31>/Logical Operator1' */
  rtb_LogicalOperator7_m = (rtb_LogicalOperator7_m && TrqR_cmd_raw);

  /* Switch: '<S82>/Switch6' incorporates:
   *  Constant: '<S82>/Reset'
   */
  if (rtb_LogicalOperator7_m) {
    /* Sum: '<S82>/Add10' incorporates:
     *  Constant: '<S82>/Steptime'
     */
    rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 +
      0.01F;
  } else {
    rtb_Switch2_mn = 0.0F;
  }

  /* End of Switch: '<S82>/Switch6' */

  /* MinMax: '<S82>/Min' incorporates:
   *  Constant: '<S31>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 = fminf(rtb_Switch2_mn,
    0.1F);

  /* RelationalOperator: '<S82>/Relational Operator9' incorporates:
   *  Constant: '<S31>/ResetDelay'
   *  UnitDelay: '<S82>/Unit Delay1'
   */
  TrqR_cmd_raw = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 >= 0.1F);

  /* UnitDelay: '<S31>/Unit Delay3' */
  rtb_LogicalOperator7_m = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e;

  /* Logic: '<S31>/Logical Operator3' */
  TrqR_cmd_raw = (TrqR_cmd_raw || rtb_LogicalOperator7_m);

  /* Logic: '<S81>/Logical Operator5' incorporates:
   *  UnitDelay: '<S81>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl = ((!rtb_ignition_e) && (
    !TrqR_cmd_raw));

  /* UnitDelay: '<S10>/Unit Delay3' */
  TrqR_cmd_raw = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i;

  /* UnitDelay: '<S45>/Delay Input2'
   *
   * Block description for '<S45>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j3;

  /* SampleTimeMath: '<S45>/sample time'
   *
   * About '<S45>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S45>/delta rise limit' */
  rtb_Add10 = (real32_T)(4000.0 * elapseTime);

  /* Gain: '<S10>/Gain21' */
  rtb_Yk1_l = 0.1020408163265306 * rtb_UkYk1_nc;

  /* MATLAB Function: '<S10>/Wtarget' incorporates:
   *  Constant: '<S10>/Constant14'
   *  Constant: '<S10>/Constant15'
   *  Constant: '<S10>/Constant16'
   *  Constant: '<S10>/Constant17'
   *  Constant: '<S10>/Constant18'
   *  Constant: '<S10>/Constant19'
   */
  rtb_Add4_j = VehVxEst_mps * rtb_CastToBoolean / (340.0F * VehVxEst_mps *
    VehVxEst_mps * 15.5799866F / 460.0F / 440.0F / 1.592F / 2.0F + 1.592F);
  if (rtb_Add4_j < 0.0F) {
    rtb_Switch2_b0 = -1.0F;
  } else if (rtb_Add4_j > 0.0F) {
    rtb_Switch2_b0 = 1.0F;
  } else if (rtb_Add4_j == 0.0F) {
    rtb_Switch2_b0 = 0.0F;
  } else {
    rtb_Switch2_b0 = (rtNaNF);
  }

  rtb_Add4_j = fminf(fabsf(5.88F / (real32_T)rtb_Yk1_l), fabsf(rtb_Add4_j)) *
    0.8F * rtb_Switch2_b0;

  /* End of MATLAB Function: '<S10>/Wtarget' */

  /* Sum: '<S45>/Difference Inputs1'
   *
   * Block description for '<S45>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add4_j -= rtb_Switch2_mn;

  /* RelationalOperator: '<S62>/LowerRelop1' */
  rtb_LogicalOperator7_m = (rtb_Add4_j > rtb_Add10);

  /* Switch: '<S62>/Switch2' */
  if (!rtb_LogicalOperator7_m) {
    /* Product: '<S45>/delta fall limit' */
    rtb_Add10 = (real32_T)(-4000.0 * elapseTime);

    /* RelationalOperator: '<S62>/UpperRelop' */
    rtb_ignition_e = (rtb_Add4_j < rtb_Add10);

    /* Switch: '<S62>/Switch' */
    if (rtb_ignition_e) {
      rtb_Add4_j = rtb_Add10;
    }

    /* End of Switch: '<S62>/Switch' */
    rtb_Add10 = rtb_Add4_j;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j3 = rtb_Add10 +
    rtb_Switch2_mn;

  /* Sum: '<S10>/Add' incorporates:
   *  UnitDelay: '<S45>/Delay Input2'
   *
   * Block description for '<S45>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j3 - rtb_UkYk1;

  /* Abs: '<S10>/Abs' */
  WhlSpdRL_mps = fabs(rtb_Yk1_l);

  /* RelationalOperator: '<S34>/Compare' incorporates:
   *  Constant: '<S34>/Constant'
   */
  rtb_LogicalOperator7_m = (WhlSpdRL_mps > 4.0);

  /* Abs: '<S10>/Abs1' */
  WhlSpdRL_mps = fabs(rtb_UkYk1);

  /* RelationalOperator: '<S35>/Compare' incorporates:
   *  Constant: '<S35>/Constant'
   */
  rtb_ignition_e = (WhlSpdRL_mps > 1.0);

  /* RelationalOperator: '<S36>/Compare' incorporates:
   *  Constant: '<S36>/Constant'
   */
  rtb_Compare = (VehVxEst_mps > 2.0F);

  /* Logic: '<S10>/AND' */
  rtb_LogicalOperator7_m = (rtb_LogicalOperator7_m && rtb_ignition_e &&
    rtb_Compare);

  /* Logic: '<S10>/Logical Operator4' */
  TrqR_cmd_raw = ((!TrqR_cmd_raw) && (!rtb_LogicalOperator7_m));

  /* Abs: '<S10>/Abs2' */
  WhlSpdRL_mps = fabs(rtb_Yk1_l);

  /* RelationalOperator: '<S37>/Compare' incorporates:
   *  Constant: '<S37>/Constant'
   */
  rtb_Compare = (WhlSpdRL_mps < 3.0);

  /* RelationalOperator: '<S38>/Compare' incorporates:
   *  Constant: '<S38>/Constant'
   */
  rtb_LogicalOperator7_m = (rtb_UkYk1_nc < -5.0);

  /* Logic: '<S10>/OR' */
  rtb_Compare = (rtb_Compare || rtb_LogicalOperator7_m);

  /* Switch: '<S10>/Switch6' incorporates:
   *  Constant: '<S10>/Reset'
   */
  if (rtb_Compare) {
    /* Sum: '<S10>/Add10' incorporates:
     *  Constant: '<S10>/Steptime'
     */
    rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_j + 0.01F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S10>/Switch6' */

  /* MinMax: '<S10>/Min' incorporates:
   *  Constant: '<S10>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_j = fminf(rtb_Add10, 1.0F);

  /* RelationalOperator: '<S10>/Relational Operator9' incorporates:
   *  Constant: '<S10>/ResetDelay'
   *  UnitDelay: '<S10>/Unit Delay4'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_j >= 1.0F);

  /* Logic: '<S10>/Logical Operator5' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i = ((!TrqR_cmd_raw) &&
    (!rtb_Compare));

  /* UnitDelay: '<S92>/Unit Delay1' */
  rtb_Compare = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e;

  /* Saturate: '<S32>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    rtb_Add10 = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    rtb_Add10 = 0.0F;
  } else {
    rtb_Add10 = VehVxEst_mps;
  }

  /* End of Saturate: '<S32>/Saturation' */

  /* Lookup_n-D: '<S32>/VehSpd_SlipTarget_mps' */
  rtb_Switch2_mn = look1_iflf_binlc(rtb_Add10,
    VehCtrlMdel240926_2018b__ConstP.pooled39,
    VehCtrlMdel240926_2018b__ConstP.pooled38, 3U);

  /* Sum: '<S32>/Add9' */
  rtb_Switch2_mn += rtb_Add10;

  /* Sum: '<S7>/Add1' */
  rtb_Add4_j = rtb_deltafalllimit_cz + rtb_deltafalllimit_ap;

  /* Gain: '<S7>/Gain2' */
  rtb_Add4_j *= 0.5F;

  /* Saturate: '<S32>/Saturation1' */
  if (rtb_Add4_j < 0.0F) {
    rtb_Add4_j = 0.0F;
  }

  /* End of Saturate: '<S32>/Saturation1' */

  /* Sum: '<S32>/Add1' */
  rtb_deltafalllimit_cz = rtb_Switch2_mn - rtb_Add4_j;

  /* RelationalOperator: '<S32>/Relational Operator7' incorporates:
   *  Constant: '<S32>/Cal_DeltaV_mps'
   */
  TrqR_cmd_raw = (rtb_deltafalllimit_cz < 0.0F);

  /* Logic: '<S92>/Logical Operator4' */
  rtb_Compare = ((!rtb_Compare) && (!TrqR_cmd_raw));

  /* Logic: '<S32>/Logical Operator2' */
  TrqR_cmd_raw = !TrqR_cmd_raw;

  /* UnitDelay: '<S32>/Unit Delay4' */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b;

  /* RelationalOperator: '<S32>/Relational Operator8' incorporates:
   *  Constant: '<S32>/Cal_DeltaV_mps1'
   */
  rtb_LogicalOperator7_m = (rtb_Switch2_mn > 235.0F);

  /* Logic: '<S32>/Logical Operator1' */
  TrqR_cmd_raw = (TrqR_cmd_raw && rtb_LogicalOperator7_m);

  /* Switch: '<S93>/Switch6' incorporates:
   *  Constant: '<S93>/Reset'
   */
  if (TrqR_cmd_raw) {
    /* Sum: '<S93>/Add10' incorporates:
     *  Constant: '<S93>/Steptime'
     */
    rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i + 0.01F;
  } else {
    rtb_Switch2_mn = 0.0F;
  }

  /* End of Switch: '<S93>/Switch6' */

  /* MinMax: '<S93>/Min' incorporates:
   *  Constant: '<S32>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i = fminf(rtb_Switch2_mn,
    0.1F);

  /* RelationalOperator: '<S93>/Relational Operator9' incorporates:
   *  Constant: '<S32>/ResetDelay'
   *  UnitDelay: '<S93>/Unit Delay1'
   */
  TrqR_cmd_raw = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i >= 0.1F);

  /* UnitDelay: '<S32>/Unit Delay3' */
  rtb_LogicalOperator7_m = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_a;

  /* Logic: '<S32>/Logical Operator3' */
  TrqR_cmd_raw = (TrqR_cmd_raw || rtb_LogicalOperator7_m);

  /* Logic: '<S92>/Logical Operator5' incorporates:
   *  UnitDelay: '<S92>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e = ((!rtb_Compare) &&
    (!TrqR_cmd_raw));

  /* RelationalOperator: '<S90>/Compare' incorporates:
   *  Constant: '<S90>/Constant'
   */
  rtb_Compare = (rtb_Add4_j > 0.0F);

  /* Abs: '<S32>/Abs' */
  rtb_Switch2_mn = fabsf(rtb_CastToBoolean);

  /* RelationalOperator: '<S88>/Compare' incorporates:
   *  Constant: '<S88>/Constant'
   */
  TrqR_cmd_raw = (rtb_Switch2_mn <= 20.0F);

  /* Logic: '<S32>/Logical Operator6' */
  rtb_Compare = (rtb_Compare && TrqR_cmd_raw);

  /* Logic: '<S32>/Logical Operator7' incorporates:
   *  UnitDelay: '<S92>/Unit Delay1'
   */
  rtb_ignition_e = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e &&
                    rtb_Compare);

  /* UnitDelay: '<S72>/Unit Delay1' */
  rtb_Compare = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp;

  /* Saturate: '<S30>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    rtb_Switch2_mn = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    rtb_Switch2_mn = 0.0F;
  } else {
    rtb_Switch2_mn = VehVxEst_mps;
  }

  /* End of Saturate: '<S30>/Saturation' */

  /* Lookup_n-D: '<S30>/VehSpd_SlipTarget_mps' */
  rtb_Add10_ey = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240926_2018b__ConstP.pooled39,
    VehCtrlMdel240926_2018b__ConstP.pooled38, 3U);

  /* Sum: '<S30>/Add9' */
  rtb_Add10_ey += rtb_Switch2_mn;

  /* Saturate: '<S30>/Saturation1' */
  if (rtb_Gain3_o < 0.0F) {
    rtb_Gain3_o = 0.0F;
  }

  /* End of Saturate: '<S30>/Saturation1' */

  /* Sum: '<S30>/Add1' */
  rtb_deltafalllimit_ap = rtb_Add10_ey - rtb_Gain3_o;

  /* RelationalOperator: '<S30>/Relational Operator7' incorporates:
   *  Constant: '<S30>/Cal_DeltaV_mps'
   */
  TrqR_cmd_raw = (rtb_deltafalllimit_ap < 0.0F);

  /* Logic: '<S72>/Logical Operator4' */
  rtb_Compare = ((!rtb_Compare) && (!TrqR_cmd_raw));

  /* Logic: '<S30>/Logical Operator2' */
  TrqR_cmd_raw = !TrqR_cmd_raw;

  /* UnitDelay: '<S30>/Unit Delay4' */
  rtb_Add10_ey = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l;

  /* RelationalOperator: '<S30>/Relational Operator8' incorporates:
   *  Constant: '<S30>/Cal_DeltaV_mps1'
   */
  rtb_LogicalOperator7_m = (rtb_Add10_ey > 235.0F);

  /* Logic: '<S30>/Logical Operator1' */
  TrqR_cmd_raw = (TrqR_cmd_raw && rtb_LogicalOperator7_m);

  /* Switch: '<S73>/Switch6' incorporates:
   *  Constant: '<S73>/Reset'
   */
  if (TrqR_cmd_raw) {
    /* Sum: '<S73>/Add10' incorporates:
     *  Constant: '<S73>/Steptime'
     */
    rtb_Add10_ey = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h + 0.01F;
  } else {
    rtb_Add10_ey = 0.0F;
  }

  /* End of Switch: '<S73>/Switch6' */

  /* MinMax: '<S73>/Min' incorporates:
   *  Constant: '<S30>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h = fminf(rtb_Add10_ey, 0.1F);

  /* RelationalOperator: '<S73>/Relational Operator9' incorporates:
   *  Constant: '<S30>/ResetDelay'
   *  UnitDelay: '<S73>/Unit Delay1'
   */
  TrqR_cmd_raw = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h >= 0.1F);

  /* UnitDelay: '<S30>/Unit Delay3' */
  rtb_LogicalOperator7_m = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_ip;

  /* Logic: '<S30>/Logical Operator3' */
  TrqR_cmd_raw = (TrqR_cmd_raw || rtb_LogicalOperator7_m);

  /* Logic: '<S72>/Logical Operator5' incorporates:
   *  UnitDelay: '<S72>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp = ((!rtb_Compare) &&
    (!TrqR_cmd_raw));

  /* Chart: '<S7>/Chart' */
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c7_VehCtrlMdel240926_
      == 0U) {
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c7_VehCtrlMdel240926_ =
      1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_B = 3U;
    VehCtrlMdel240926_2018b_amks_DW.b = rtb_UkYk1_nc * rtb_UkYk1_nc + rtb_Add1 *
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
      rtb_LogicalOperator2 = ((rtb_UkYk1 >= 50.0) || (rtb_Add1 >= 5.0) ||
        (VehCtrlMdel240926_2018b_amks_DW.b >= 5.0));
      if (rtb_LogicalOperator2) {
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
      elapseTime = atan(rtb_UkYk1_nc / rtb_Add1) * 180.0 / 3.1415926535897931;
    } else {
      elapseTime = 0.0;
    }

    VehCtrlMdel240926_2018b_amks_DW.b = rtb_UkYk1_nc * rtb_UkYk1_nc + rtb_Add1 *
      rtb_Add1;
    VehCtrlMdel240926_2018b_amks_DW.b = sqrt(VehCtrlMdel240926_2018b_amks_DW.b);
    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_C) {
     case VehCtrlMdel240926_2018b_am_IN_B:
      rtb_LogicalOperator2 = ((!(VehCtrlMdel240926_2018b_amks_DW.DYC_flag != 0.0))
        || (rtb_UkYk1 > 30.0) || (elapseTime > 30.0) || (elapseTime > -30.0) ||
        (VehCtrlMdel240926_2018b_amks_DW.b > 3.0));
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_C = 3U;
      }
      break;

     case VehCtrlMdel240926_2018b_am_IN_C:
      break;

     default:
      /* case IN_F_TVD_TCS_STATE: */
      rtb_LogicalOperator2 = ((!(VehCtrlMdel240926_2018b_amks_DW.DYC_flag != 0.0))
        || (rtb_UkYk1 > 30.0) || (elapseTime > 30.0) || (elapseTime > -30.0) ||
        (VehCtrlMdel240926_2018b_amks_DW.b > 3.0));
      if (rtb_LogicalOperator2) {
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

  /* Sum: '<S28>/Add' incorporates:
   *  Constant: '<S28>/Constant3'
   */
  WhlSpdRL_mps = 1.0 - WhlSpdFR;

  /* Product: '<S28>/Product5' */
  rtb_g_mpss1 = 50.0 * WhlSpdRL_mps;

  /* Product: '<S28>/Product' */
  WhlSpdRL_mps = rtb_g_mpss1 * 9550.0;

  /* Sum: '<S28>/Add1' incorporates:
   *  Constant: '<S28>/RPM_min'
   */
  rtb_g_mpss1 = RPM + 10.0;

  /* MinMax: '<S28>/Max' incorporates:
   *  Constant: '<S28>/RPM_min1'
   */
  WhlSpdFR = fmax(rtb_g_mpss1, 1.0);

  /* Product: '<S28>/Divide' */
  WhlSpdRL_mps /= WhlSpdFR;

  /* UnitDelay: '<S7>/Unit Delay' */
  WhlSpdFR = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_n;

  /* UnitDelay: '<S43>/Delay Input2'
   *
   * Block description for '<S43>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_g_mpss1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p;

  /* SampleTimeMath: '<S43>/sample time'
   *
   * About '<S43>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S43>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  rtb_Switch2_on = 2000.0 * elapseTime;

  /* MATLAB Function: '<S10>/ÔØºÉ×ªÒÆ' incorporates:
   *  Constant: '<S10>/Constant10'
   *  Constant: '<S10>/Constant2'
   *  Constant: '<S10>/Constant3'
   *  Constant: '<S10>/Constant4'
   *  Constant: '<S10>/Constant5'
   *  Constant: '<S10>/Constant9'
   */
  rtb_Ax = (1666.0F - 340.0F * (real32_T)rtb_Add1 * 0.29F / 1.2F) * 0.521984875F
    - 170.0F * (real32_T)rtb_UkYk1_nc * 0.29F / 1.592F;
  rtb_MaxWhlSpd_mps_n = (340.0F * (real32_T)rtb_Add1 * 0.29F / 1.2F + 1666.0F) *
    0.521984875F - 170.0F * (real32_T)rtb_UkYk1_nc * 0.29F / 1.592F;
  rtb_Fz3 = (1666.0F - 340.0F * (real32_T)rtb_Add1 * 0.29F / 1.2F) *
    0.521984875F + 170.0F * (real32_T)rtb_UkYk1_nc * 0.29F / 1.592F;
  rtb_VxIMU_est = (340.0F * (real32_T)rtb_Add1 * 0.29F / 1.2F + 1666.0F) *
    0.521984875F + 170.0F * (real32_T)rtb_UkYk1_nc * 0.29F / 1.592F;

  /* Lookup_n-D: '<S10>/AMK' */
  rtb_UkYk1 = look1_binlx(MCFL_ActualVelocity,
    VehCtrlMdel240926_2018b__ConstP.pooled10,
    VehCtrlMdel240926_2018b__ConstP.pooled9, 19U);

  /* Lookup_n-D: '<S10>/AMK1' */
  rtb_UkYk1_nc = look1_binlx(MCFR_ActualVelocity,
    VehCtrlMdel240926_2018b__ConstP.pooled10,
    VehCtrlMdel240926_2018b__ConstP.pooled9, 19U);

  /* Lookup_n-D: '<S10>/228' */
  rtb_Gain5 = look1_binlx(RPM, VehCtrlMdel240926_2018b__ConstP.pooled4,
    VehCtrlMdel240926_2018b__ConstP.u28_tableData, 26U);

  /* Gain: '<S10>/Gain3' */
  rtb_Add1 *= 0.1020408163265306;

  /* MATLAB Function: '<S10>/MATLAB Function' incorporates:
   *  Constant: '<S10>/Constant11'
   *  Constant: '<S10>/Constant12'
   *  Constant: '<S10>/Constant13'
   *  Constant: '<S10>/Constant26'
   */
  rtb_Switch2_b0 = rtb_Ax * 0.75F;
  rtb_Ax = rtb_Ax * (real32_T)rtb_Add1 / 9.8F;
  rtb_Product2_j = rtb_MaxWhlSpd_mps_n * 0.75F;
  rtb_Gain26 = rtb_MaxWhlSpd_mps_n * (real32_T)rtb_Add1 / 9.8F;
  rtb_MaxWhlSpd_mps_n = rtb_Fz3 * 0.75F;
  rtb_Fz3 = rtb_Fz3 * (real32_T)rtb_Add1 / 9.8F;
  rtb_Add10_ey = rtb_VxIMU_est * 0.75F;
  rtb_Min1_i = rtb_VxIMU_est * (real32_T)rtb_Add1 / 9.8F;
  rtb_VxIMU_est = fminf(sqrtf(rtb_Product2_j * rtb_Product2_j - rtb_Gain26 *
    rtb_Gain26) * 0.2F / 11.4F, (real32_T)rtb_UkYk1_nc);
  rtb_Switch2_b0 = fminf(sqrtf(rtb_Switch2_b0 * rtb_Switch2_b0 - rtb_Ax * rtb_Ax)
    * 0.2F / 11.4F, (real32_T)rtb_UkYk1);
  rtb_Ax = fminf(fminf(sqrtf(rtb_Add10_ey * rtb_Add10_ey - rtb_Min1_i *
    rtb_Min1_i), sqrtf(rtb_MaxWhlSpd_mps_n * rtb_MaxWhlSpd_mps_n - rtb_Fz3 *
                       rtb_Fz3)) * 0.2F / 3.4F, (real32_T)rtb_Gain5);

  /* Gain: '<S10>/Gain2' */
  rtb_Add10_ey = 0.95F * rtb_Ax;

  /* Gain: '<S7>/Gain' */
  rtb_Ax = 3.6F * VehVxEst_mps;

  /* Switch: '<S29>/Switch' */
  if (rtb_Compare_am) {
    /* Lookup_n-D: '<S29>/4WD_Table' */
    rtb_Ax = look2_iflf_binlx(Acc_POS_n, rtb_Ax,
      VehCtrlMdel240926_2018b__ConstP.pooled34,
      VehCtrlMdel240926_2018b__ConstP.pooled35,
      VehCtrlMdel240926_2018b__ConstP.pooled33,
      VehCtrlMdel240926_2018b__ConstP.pooled85, 11U);
  } else {
    /* Lookup_n-D: '<S29>/RWD_Table' */
    rtb_Ax = look2_iflf_binlx(Acc_POS_n, rtb_Ax,
      VehCtrlMdel240926_2018b__ConstP.pooled34,
      VehCtrlMdel240926_2018b__ConstP.pooled35,
      VehCtrlMdel240926_2018b__ConstP.pooled33,
      VehCtrlMdel240926_2018b__ConstP.pooled85, 11U);
  }

  /* End of Switch: '<S29>/Switch' */

  /* Gain: '<S10>/Gain1' */
  rtb_MaxWhlSpd_mps_n = 0.95F * rtb_Switch2_b0;

  /* Gain: '<S10>/Gain' */
  rtb_VxIMU_est *= 0.95F;

  /* MinMax: '<S10>/Min1' */
  rtb_Min1_i = fminf(rtb_VxIMU_est, rtb_MaxWhlSpd_mps_n);

  /* Product: '<S10>/Divide3' */
  rtb_Product2_j = rtb_Min1_i / rtb_Add10_ey;

  /* UnitDelay: '<S44>/Delay Input2'
   *
   * Block description for '<S44>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Fz3 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l4;

  /* SampleTimeMath: '<S44>/sample time'
   *
   * About '<S44>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime_0 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S44>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant48'
   */
  Acc_POS_n = (real32_T)(4.0 * elapseTime_0);

  /* Abs: '<S10>/Abs5' */
  rtb_Switch2_b0 = fabsf(rtb_CastToBoolean);

  /* Lookup_n-D: '<S10>/2-D Lookup Table1' */
  rtb_Switch2_b0 = look2_iflf_binlx(rtb_Switch2_b0, VehVxEst_mps,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp02Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_tableData,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_maxIndex, 5U);

  /* Sum: '<S44>/Difference Inputs1'
   *
   * Block description for '<S44>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 -= rtb_Fz3;

  /* RelationalOperator: '<S61>/LowerRelop1' */
  rtb_Compare = (rtb_Switch2_b0 > Acc_POS_n);

  /* Switch: '<S61>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S44>/delta fall limit' */
    Acc_POS_n = (real32_T)(-4.0 * elapseTime_0);

    /* RelationalOperator: '<S61>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_Switch2_b0 < Acc_POS_n);

    /* Switch: '<S61>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_Switch2_b0 = Acc_POS_n;
    }

    /* End of Switch: '<S61>/Switch' */
    Acc_POS_n = rtb_Switch2_b0;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l4 = Acc_POS_n + rtb_Fz3;

  /* Sum: '<S10>/Add18' incorporates:
   *  UnitDelay: '<S44>/Delay Input2'
   *
   * Block description for '<S44>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Product2_j += VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l4;

  /* Saturate: '<S10>/Saturation3' */
  if (rtb_Product2_j > 0.7F) {
    rtb_Product2_j = 0.7F;
  } else {
    if (rtb_Product2_j < 0.1F) {
      rtb_Product2_j = 0.1F;
    }
  }

  /* End of Saturate: '<S10>/Saturation3' */

  /* Sum: '<S10>/Add17' incorporates:
   *  Constant: '<S10>/Constant47'
   */
  rtb_Add5 = 1.0 - rtb_Product2_j;

  /* Product: '<S10>/Product1' */
  rtb_Product1 = rtb_Ax * rtb_Add5;

  /* RelationalOperator: '<S50>/LowerRelop1' */
  rtb_Compare = (rtb_Product1 > rtb_Add10_ey);

  /* Switch: '<S50>/Switch2' */
  if (rtb_Compare) {
    rtb_Add5 = rtb_Add10_ey;
  } else {
    /* RelationalOperator: '<S50>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    rtb_LowerRelop1_b = (rtb_Product1 < 0.0);

    /* Switch: '<S50>/Switch' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    if (rtb_LowerRelop1_b) {
      rtb_Add5 = 0.0;
    } else {
      rtb_Add5 = rtb_Product1;
    }

    /* End of Switch: '<S50>/Switch' */
  }

  /* End of Switch: '<S50>/Switch2' */

  /* UnitDelay: '<S46>/Delay Input2'
   *
   * Block description for '<S46>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_ow = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_nk;

  /* SampleTimeMath: '<S46>/sample time'
   *
   * About '<S46>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime_0 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S46>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant45'
   */
  rtb_Gain5 = 1000.0 * elapseTime_0;

  /* Gain: '<S10>/Gain4' */
  rtb_Product2_j *= 0.5F;

  /* Product: '<S10>/Product2' */
  rtb_Product2_j *= rtb_Ax;

  /* Gain: '<S10>/Gain26' */
  rtb_Gain26 = 0.8F * rtb_Product2_j;

  /* Logic: '<S10>/AND3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_Compare = ((WhlSpdFR != 0.0) &&
                 VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i);

  /* Switch: '<S10>/Switch1' incorporates:
   *  Constant: '<S10>/Constant1'
   */
  if (!rtb_Compare) {
    rtb_Yk1_l = 0.0;
  }

  /* End of Switch: '<S10>/Switch1' */

  /* Product: '<S10>/Product3' */
  rtb_Gain4 = 2.0 * rtb_Yk1_l;

  /* UnitDelay: '<S10>/Unit Delay2' */
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j;

  /* Product: '<S10>/Product' */
  rtb_Switch2_b0 *= rtb_CastToBoolean;

  /* RelationalOperator: '<S33>/Compare' incorporates:
   *  Constant: '<S33>/Constant'
   */
  rtb_Compare = (rtb_Switch2_b0 <= 0.0F);

  /* UnitDelay: '<S10>/Unit Delay' */
  rtb_Add1 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_h;

  /* Switch: '<S10>/Switch' incorporates:
   *  Constant: '<S10>/Constant'
   */
  if (rtb_Compare) {
    rtb_Add1 = 0.0;
  }

  /* End of Switch: '<S10>/Switch' */

  /* Product: '<S10>/Product4' */
  rtb_UkYk1 = rtb_Yk1_l;

  /* UnitDelay: '<S10>/Unit Delay1' */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_c;

  /* Sum: '<S10>/Add2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_h = (rtb_Add1 + rtb_UkYk1) -
    rtb_Yk1_l;

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
  rtb_Compare = VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE_b;

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
    if (!VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i) {
      rtb_StrWhlAngV_c = 0.0F;
    }

    /* End of Switch: '<S10>/Switch4' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_k = rtb_StrWhlAngV_c;
  }

  /* End of Switch: '<S10>/Switch3' */

  /* Sum: '<S10>/Add1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay'
   *  UnitDelay: '<S10>/Unit Delay5'
   */
  rtb_Yk1_l = (rtb_Gain4 + VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_h) +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_k;

  /* Saturate: '<S10>/Saturation' */
  if (rtb_Yk1_l > 5000.0) {
    rtb_UkYk1 = 5000.0;
  } else if (rtb_Yk1_l < -5000.0) {
    rtb_UkYk1 = -5000.0;
  } else {
    rtb_UkYk1 = rtb_Yk1_l;
  }

  /* End of Saturate: '<S10>/Saturation' */

  /* Gain: '<S10>/Gain8' */
  rtb_Switch2_b0 = 0.0174532924F * FLWhlStrAng;

  /* Trigonometry: '<S10>/Cos' */
  rtb_Switch2_b0 = cosf(rtb_Switch2_b0);

  /* Gain: '<S10>/Gain11' */
  rtb_Switch2_b0 *= 1.2F;

  /* Sum: '<S10>/Add6' incorporates:
   *  Constant: '<S10>/Constant27'
   */
  Acc_POS_n = 90.0F - FLWhlStrAng;

  /* Gain: '<S10>/Gain9' */
  Acc_POS_n *= 0.0174532924F;

  /* Trigonometry: '<S10>/Cos1' */
  Acc_POS_n = cosf(Acc_POS_n);

  /* Gain: '<S10>/Gain10' */
  Acc_POS_n *= 1.522F;

  /* Sum: '<S10>/Add7' */
  rtb_Switch2_b0 += Acc_POS_n;

  /* Product: '<S10>/Divide1' */
  rtb_Add1 = rtb_UkYk1 / rtb_Switch2_b0;

  /* Gain: '<S10>/Gain25' */
  rtb_Add1 *= 0.2;

  /* RelationalOperator: '<S54>/LowerRelop1' */
  rtb_Compare = (rtb_Add1 > rtb_Gain26);

  /* Switch: '<S54>/Switch2' */
  if (rtb_Compare) {
    rtb_Add1 = rtb_Gain26;
  } else {
    /* Gain: '<S10>/Gain27' */
    rtb_StrWhlAngV_c = -rtb_Gain26;

    /* RelationalOperator: '<S54>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_Add1 < rtb_StrWhlAngV_c);

    /* Switch: '<S54>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_Add1 = rtb_StrWhlAngV_c;
    }

    /* End of Switch: '<S54>/Switch' */
  }

  /* End of Switch: '<S54>/Switch2' */

  /* Sum: '<S10>/Add4' */
  rtb_UkYk1_nc = rtb_Product2_j + rtb_Add1;

  /* Sum: '<S10>/Add14' */
  rtb_Add1 = rtb_Min1_i - rtb_UkYk1_nc;

  /* RelationalOperator: '<S10>/Relational Operator' incorporates:
   *  Constant: '<S10>/Constant37'
   */
  rtb_LowerRelop1_b = (rtb_Add1 < 0.0);

  /* Sum: '<S10>/Add15' */
  rtb_Product1 = rtb_Add10_ey - rtb_Product1;

  /* RelationalOperator: '<S10>/Relational Operator1' incorporates:
   *  Constant: '<S10>/Constant38'
   */
  rtb_LogicalOperator2 = (rtb_Product1 < 0.0);

  /* Logic: '<S10>/AND1' */
  rtb_Compare = (rtb_LowerRelop1_b && rtb_LogicalOperator2);

  /* Logic: '<S10>/OR1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i ||
                 rtb_Compare);

  /* Switch: '<S10>/Switch2' incorporates:
   *  Constant: '<S10>/Constant32'
   */
  if (rtb_Compare) {
    rtb_Add1 = 0.0;
  } else {
    /* Logic: '<S10>/NOT1' */
    rtb_Compare = !rtb_LogicalOperator2;

    /* Switch: '<S10>/Switch5' incorporates:
     *  Constant: '<S10>/Constant33'
     */
    if (!rtb_Compare) {
      rtb_Add1 = 0.0;
    }

    /* End of Switch: '<S10>/Switch5' */
  }

  /* End of Switch: '<S10>/Switch2' */

  /* Gain: '<S10>/Gain19' */
  rtb_Add1 = -rtb_Add1;

  /* Saturate: '<S10>/Saturation1' */
  if (rtb_Add1 > 100.0) {
    rtb_Add1 = 100.0;
  } else {
    if (rtb_Add1 < 0.0) {
      rtb_Add1 = 0.0;
    }
  }

  /* End of Saturate: '<S10>/Saturation1' */

  /* Sum: '<S46>/Difference Inputs1'
   *
   * Block description for '<S46>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add1 -= rtb_Switch2_ow;

  /* RelationalOperator: '<S63>/LowerRelop1' */
  rtb_Compare = (rtb_Add1 > rtb_Gain5);

  /* Switch: '<S63>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S46>/delta fall limit' */
    rtb_Gain5 = -1000.0 * elapseTime_0;

    /* RelationalOperator: '<S63>/UpperRelop' */
    rtb_Compare = (rtb_Add1 < rtb_Gain5);

    /* Switch: '<S63>/Switch' */
    if (rtb_Compare) {
      rtb_Add1 = rtb_Gain5;
    }

    /* End of Switch: '<S63>/Switch' */
    rtb_Gain5 = rtb_Add1;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_nk = rtb_Gain5 +
    rtb_Switch2_ow;

  /* Sum: '<S10>/Add13' incorporates:
   *  UnitDelay: '<S46>/Delay Input2'
   *
   * Block description for '<S46>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add1 = rtb_Add5 + VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_nk;

  /* RelationalOperator: '<S53>/LowerRelop1' */
  rtb_Compare = (rtb_Add1 > rtb_Add10_ey);

  /* Switch: '<S53>/Switch2' */
  if (rtb_Compare) {
    rtb_Add1 = rtb_Add10_ey;
  } else {
    /* RelationalOperator: '<S53>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    rtb_Compare = (rtb_Add1 < 0.0);

    /* Switch: '<S53>/Switch' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    if (rtb_Compare) {
      rtb_Add1 = 0.0;
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
  rtb_Add1 -= rtb_g_mpss1;

  /* RelationalOperator: '<S60>/LowerRelop1' */
  rtb_Compare = (rtb_Add1 > rtb_Switch2_on);

  /* Switch: '<S60>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S43>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S60>/UpperRelop' */
    rtb_Compare = (rtb_Add1 < elapseTime);

    /* Switch: '<S60>/Switch' */
    if (rtb_Compare) {
      rtb_Add1 = elapseTime;
    }

    /* End of Switch: '<S60>/Switch' */
    rtb_Switch2_on = rtb_Add1;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p = rtb_Switch2_on +
    rtb_g_mpss1;

  /* Switch: '<S7>/Switch1' incorporates:
   *  UnitDelay: '<S43>/Delay Input2'
   *
   * Block description for '<S43>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (WhlSpdFR != 0.0) {
    elapseTime = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p;
  } else {
    /* Logic: '<S7>/NOT1' */
    rtb_Compare = !rtb_Compare_am;

    /* Switch: '<S7>/Switch10' */
    if (rtb_Compare) {
      /* Product: '<S7>/Product2' incorporates:
       *  Constant: '<S7>/Constant'
       */
      elapseTime = rtb_Ax * 0.8;
    } else {
      elapseTime = rtb_Ax;
    }

    /* End of Switch: '<S7>/Switch10' */
  }

  /* End of Switch: '<S7>/Switch1' */

  /* RelationalOperator: '<S25>/LowerRelop1' */
  rtb_Compare = (elapseTime > WhlSpdRL_mps);

  /* Switch: '<S25>/Switch2' */
  if (rtb_Compare) {
    FLWhlStrAng = (real32_T)WhlSpdRL_mps;
  } else {
    /* RelationalOperator: '<S25>/UpperRelop' incorporates:
     *  Constant: '<S7>/Constant1'
     */
    rtb_Compare = (elapseTime < 0.0);

    /* Switch: '<S25>/Switch' incorporates:
     *  Constant: '<S7>/Constant1'
     */
    if (rtb_Compare) {
      FLWhlStrAng = 0.0F;
    } else {
      FLWhlStrAng = (real32_T)elapseTime;
    }

    /* End of Switch: '<S25>/Switch' */
  }

  /* End of Switch: '<S25>/Switch2' */

  /* Switch: '<S32>/Switch6' incorporates:
   *  Constant: '<S32>/Verror_Reset'
   */
  if (rtb_ignition_e) {
    rtb_Switch2_b0 = rtb_deltafalllimit_cz;
  } else {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Switch: '<S32>/Switch6' */

  /* Product: '<S32>/Product' incorporates:
   *  Constant: '<S32>/P_Gain'
   */
  rtb_StrWhlAngV_c = rtb_Switch2_b0 * 40.0F;

  /* Sum: '<S32>/Add11' */
  Acc_POS_n = FLWhlStrAng - rtb_StrWhlAngV_c;

  /* UnitDelay: '<S32>/Unit Delay5' */
  rtb_Fz3 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_l;

  /* Product: '<S32>/Product2' */
  rtb_Fz3 *= rtb_deltafalllimit_cz;

  /* RelationalOperator: '<S87>/Compare' incorporates:
   *  Constant: '<S87>/Constant'
   */
  rtb_Compare = (rtb_Fz3 <= 0.0F);

  /* UnitDelay: '<S32>/Unit Delay' */
  rtb_Fz3 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b;

  /* Switch: '<S32>/Switch3' incorporates:
   *  Constant: '<S32>/Verror_Reset1'
   */
  if (rtb_Compare) {
    rtb_Fz3 = 0.0F;
  }

  /* End of Switch: '<S32>/Switch3' */

  /* Sum: '<S32>/Add2' */
  rtb_Fz3 += rtb_Switch2_b0;

  /* Saturate: '<S32>/Saturation2' */
  if (rtb_Fz3 > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = 400.0F;
  } else if (rtb_Fz3 < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = rtb_Fz3;
  }

  /* End of Saturate: '<S32>/Saturation2' */

  /* UnitDelay: '<S91>/Delay Input1'
   *
   * Block description for '<S91>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_Compare = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_e;

  /* RelationalOperator: '<S91>/FixPt Relational Operator' */
  rtb_Compare = ((int32_T)rtb_ignition_e > (int32_T)rtb_Compare);

  /* Switch: '<S32>/Switch' incorporates:
   *  Constant: '<S32>/Integr_StartPoint'
   */
  if (rtb_Compare) {
    /* Sum: '<S32>/Add4' */
    rtb_Switch2_b0 = FLWhlStrAng -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g;
  } else {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Switch: '<S32>/Switch' */

  /* Lookup_n-D: '<S32>/VehicleStableTarget_mps' */
  rtb_Fz3 = look1_iflf_binlc(rtb_Add10, VehCtrlMdel240926_2018b__ConstP.pooled39,
    VehCtrlMdel240926_2018b__ConstP.pooled66, 3U);

  /* Sum: '<S32>/Add5' */
  rtb_Fz3 += rtb_Add10;

  /* Sum: '<S32>/Add10' */
  rtb_Fz3 = rtb_Add4_j - rtb_Fz3;

  /* RelationalOperator: '<S32>/Relational Operator' incorporates:
   *  Constant: '<S32>/Verror'
   */
  rtb_Compare = (rtb_Fz3 < 0.0F);

  /* Logic: '<S32>/Logical Operator4' */
  rtb_Compare = (rtb_Compare && rtb_ignition_e);

  /* Switch: '<S32>/Switch1' incorporates:
   *  Constant: '<S32>/Trq_IReset'
   *  Constant: '<S32>/Trq_I_FF'
   */
  if (rtb_Compare) {
    rtb_Fz3 = 20.0F;
  } else {
    rtb_Fz3 = 0.0F;
  }

  /* End of Switch: '<S32>/Switch1' */

  /* Sum: '<S32>/Add6' incorporates:
   *  UnitDelay: '<S32>/Unit Delay'
   */
  rtb_Switch2_b0 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b +
                    rtb_Switch2_b0) + rtb_Fz3;

  /* Product: '<S32>/Product1' incorporates:
   *  Constant: '<S32>/I_Gain'
   */
  rtb_Switch2_b0 *= 10.0F;

  /* RelationalOperator: '<S94>/LowerRelop1' */
  rtb_Compare = (rtb_Switch2_b0 > Acc_POS_n);

  /* Switch: '<S94>/Switch2' */
  if (!rtb_Compare) {
    /* Gain: '<S32>/Gain3' */
    Acc_POS_n = -rtb_StrWhlAngV_c;

    /* RelationalOperator: '<S94>/UpperRelop' */
    rtb_Compare = (rtb_Switch2_b0 < Acc_POS_n);

    /* Switch: '<S94>/Switch' */
    if (rtb_Compare) {
      rtb_Switch2_b0 = Acc_POS_n;
    }

    /* End of Switch: '<S94>/Switch' */
    Acc_POS_n = rtb_Switch2_b0;
  }

  /* End of Switch: '<S94>/Switch2' */

  /* Sum: '<S32>/Add7' incorporates:
   *  UnitDelay: '<S32>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b = rtb_StrWhlAngV_c +
    Acc_POS_n;

  /* Lookup_n-D: '<S32>/VehicleStableTarget_mps1' */
  rtb_Switch2_b0 = look1_iflf_binlc(rtb_Add10,
    VehCtrlMdel240926_2018b__ConstP.pooled39,
    VehCtrlMdel240926_2018b__ConstP.pooled66, 3U);

  /* Sum: '<S32>/Add13' */
  rtb_Add10 += rtb_Switch2_b0;

  /* Sum: '<S32>/Add12' */
  rtb_Add4_j -= rtb_Add10;

  /* RelationalOperator: '<S32>/Relational Operator1' incorporates:
   *  Constant: '<S32>/Verror1'
   */
  rtb_Compare = (rtb_Add4_j < 0.0F);

  /* RelationalOperator: '<S32>/Relational Operator2' incorporates:
   *  UnitDelay: '<S32>/Unit Delay4'
   */
  TrqR_cmd_raw = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b >=
                  FLWhlStrAng);

  /* RelationalOperator: '<S89>/Compare' incorporates:
   *  Constant: '<S89>/Constant'
   *  UnitDelay: '<S32>/Unit Delay4'
   */
  rtb_LogicalOperator7_m = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b <=
    5.0F);

  /* Logic: '<S32>/OR' */
  TrqR_cmd_raw = (TrqR_cmd_raw || rtb_LogicalOperator7_m);

  /* Logic: '<S32>/Logical Operator5' incorporates:
   *  UnitDelay: '<S32>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_a = (rtb_Compare &&
    TrqR_cmd_raw);

  /* Switch: '<S32>/Switch2' incorporates:
   *  Switch: '<S32>/Switch7'
   *  UnitDelay: '<S32>/Unit Delay3'
   */
  if (rtb_ignition_e) {
    /* RelationalOperator: '<S95>/LowerRelop1' incorporates:
     *  Constant: '<S32>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S32>/Unit Delay4'
     */
    rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b > 235.0F);

    /* Switch: '<S95>/Switch2' incorporates:
     *  Constant: '<S32>/TCS_TrqRequest_Max2'
     */
    if (rtb_Compare) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 235.0F;
    } else {
      /* RelationalOperator: '<S95>/UpperRelop' incorporates:
       *  Constant: '<S32>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S32>/Unit Delay4'
       */
      rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b < 0.0F);

      /* Switch: '<S95>/Switch' incorporates:
       *  Constant: '<S32>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S32>/Unit Delay4'
       */
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 0.0F;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b;
      }

      /* End of Switch: '<S95>/Switch' */
    }

    /* End of Switch: '<S95>/Switch2' */

    /* RelationalOperator: '<S96>/LowerRelop1' */
    rtb_Compare = (FLWhlStrAng >
                   VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g);

    /* Switch: '<S96>/Switch2' */
    if (!rtb_Compare) {
      /* RelationalOperator: '<S96>/UpperRelop' incorporates:
       *  Constant: '<S32>/TCS_TrqRequest_Min1'
       */
      rtb_Compare = (FLWhlStrAng < 0.0F);

      /* Switch: '<S96>/Switch' incorporates:
       *  Constant: '<S32>/TCS_TrqRequest_Min1'
       */
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 0.0F;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = FLWhlStrAng;
      }

      /* End of Switch: '<S96>/Switch' */
    }

    /* End of Switch: '<S96>/Switch2' */
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_a) {
      /* Switch: '<S32>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = FLWhlStrAng;
    }
  }

  /* End of Switch: '<S32>/Switch2' */

  /* Sum: '<S28>/Add4' incorporates:
   *  Constant: '<S28>/Constant'
   */
  rtb_Add1 = 1.0 - WhlSpdRR_mps;

  /* Product: '<S28>/Product4' */
  rtb_Gain4 = 12.5 * rtb_Add1;

  /* Product: '<S28>/Product1' */
  rtb_Gain5 = rtb_Gain4 * 9550.0;

  /* Sum: '<S28>/Add2' incorporates:
   *  Constant: '<S28>/RPM_min2'
   */
  rtb_Gain4 = MCFR_ActualVelocity + 10.0;

  /* MinMax: '<S28>/Max1' incorporates:
   *  Constant: '<S28>/RPM_min3'
   */
  rtb_Switch2_ow = fmax(rtb_Gain4, 1.0);

  /* Product: '<S28>/Divide1' */
  rtb_Gain5 /= rtb_Switch2_ow;

  /* UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pt;

  /* SampleTimeMath: '<S42>/sample time'
   *
   * About '<S42>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S42>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  rtb_Switch2_ow = 2000.0 * elapseTime;

  /* Gain: '<S10>/Gain14' */
  rtb_Switch2_b0 = 0.0174532924F * rtb_FRWhlStrAng;

  /* Trigonometry: '<S10>/Cos2' */
  rtb_Switch2_b0 = cosf(rtb_Switch2_b0);

  /* Gain: '<S10>/Gain13' */
  rtb_Switch2_b0 *= 1.2F;

  /* Sum: '<S10>/Add8' incorporates:
   *  Constant: '<S10>/Constant28'
   */
  Acc_POS_n = 90.0F - rtb_FRWhlStrAng;

  /* Gain: '<S10>/Gain15' */
  Acc_POS_n *= 0.0174532924F;

  /* Trigonometry: '<S10>/Cos3' */
  Acc_POS_n = cosf(Acc_POS_n);

  /* Gain: '<S10>/Gain12' */
  Acc_POS_n *= 1.522F;

  /* Sum: '<S10>/Add9' */
  rtb_Switch2_b0 += Acc_POS_n;

  /* Product: '<S10>/Divide2' */
  rtb_Add5 = rtb_UkYk1 / rtb_Switch2_b0;

  /* Gain: '<S10>/Gain24' */
  rtb_Add5 *= 0.2;

  /* RelationalOperator: '<S55>/LowerRelop1' */
  rtb_Compare = (rtb_Add5 > rtb_Gain26);

  /* Switch: '<S55>/Switch2' */
  if (rtb_Compare) {
    rtb_Add5 = rtb_Gain26;
  } else {
    /* Gain: '<S10>/Gain28' */
    rtb_FRWhlStrAng = -rtb_Gain26;

    /* RelationalOperator: '<S55>/UpperRelop' */
    rtb_Compare = (rtb_Add5 < rtb_FRWhlStrAng);

    /* Switch: '<S55>/Switch' */
    if (rtb_Compare) {
      rtb_Add5 = rtb_FRWhlStrAng;
    }

    /* End of Switch: '<S55>/Switch' */
  }

  /* End of Switch: '<S55>/Switch2' */

  /* Sum: '<S10>/Add5' */
  rtb_Add5 = rtb_Product2_j - rtb_Add5;

  /* RelationalOperator: '<S49>/LowerRelop1' */
  rtb_Compare = (rtb_Add5 > rtb_MaxWhlSpd_mps_n);

  /* Switch: '<S49>/Switch2' */
  if (rtb_Compare) {
    rtb_Add5 = rtb_MaxWhlSpd_mps_n;
  } else {
    /* RelationalOperator: '<S49>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    rtb_Compare = (rtb_Add5 < 0.0);

    /* Switch: '<S49>/Switch' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    if (rtb_Compare) {
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
  rtb_Switch2_on = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_lt;

  /* SampleTimeMath: '<S47>/sample time'
   *
   * About '<S47>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime_0 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S47>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant44'
   */
  rtb_g_mpss1 = 1000.0 * elapseTime_0;

  /* Logic: '<S10>/AND2' */
  rtb_Compare = (rtb_LowerRelop1_b && rtb_LogicalOperator2);

  /* Logic: '<S10>/OR2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i ||
                 rtb_Compare);

  /* Switch: '<S10>/Switch7' incorporates:
   *  Constant: '<S10>/Constant39'
   */
  if (rtb_Compare) {
    WhlSpdRL_mps = 0.0;
  } else {
    /* Logic: '<S10>/NOT' */
    rtb_LowerRelop1_b = !rtb_LowerRelop1_b;

    /* Switch: '<S10>/Switch8' incorporates:
     *  Constant: '<S10>/Constant40'
     */
    if (!rtb_LowerRelop1_b) {
      rtb_Product1 = 0.0;
    }

    /* End of Switch: '<S10>/Switch8' */
    WhlSpdRL_mps = rtb_Product1;
  }

  /* End of Switch: '<S10>/Switch7' */

  /* Gain: '<S10>/Gain20' */
  WhlSpdRL_mps = -WhlSpdRL_mps;

  /* Saturate: '<S10>/Saturation2' */
  if (WhlSpdRL_mps > 100.0) {
    WhlSpdRL_mps = 100.0;
  } else {
    if (WhlSpdRL_mps < 0.0) {
      WhlSpdRL_mps = 0.0;
    }
  }

  /* End of Saturate: '<S10>/Saturation2' */

  /* Sum: '<S47>/Difference Inputs1'
   *
   * Block description for '<S47>/Difference Inputs1':
   *
   *  Add in CPU
   */
  WhlSpdRR_mps = WhlSpdRL_mps - rtb_Switch2_on;

  /* RelationalOperator: '<S64>/LowerRelop1' */
  rtb_Compare = (WhlSpdRR_mps > rtb_g_mpss1);

  /* Switch: '<S64>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S47>/delta fall limit' */
    WhlSpdRL_mps = -1000.0 * elapseTime_0;

    /* RelationalOperator: '<S64>/UpperRelop' */
    rtb_LowerRelop1_b = (WhlSpdRR_mps < WhlSpdRL_mps);

    /* Switch: '<S64>/Switch' */
    if (rtb_LowerRelop1_b) {
      WhlSpdRR_mps = WhlSpdRL_mps;
    }

    /* End of Switch: '<S64>/Switch' */
    rtb_g_mpss1 = WhlSpdRR_mps;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_lt = rtb_g_mpss1 +
    rtb_Switch2_on;

  /* Sum: '<S10>/Add12' incorporates:
   *  UnitDelay: '<S47>/Delay Input2'
   *
   * Block description for '<S47>/Delay Input2':
   *
   *  Store in Global RAM
   */
  WhlSpdRR_mps = rtb_Add5 +
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_lt;

  /* RelationalOperator: '<S52>/LowerRelop1' */
  rtb_Compare = (WhlSpdRR_mps > rtb_MaxWhlSpd_mps_n);

  /* Switch: '<S52>/Switch2' */
  if (rtb_Compare) {
    WhlSpdRR_mps = rtb_MaxWhlSpd_mps_n;
  } else {
    /* RelationalOperator: '<S52>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    rtb_LowerRelop1_b = (WhlSpdRR_mps < 0.0);

    /* Switch: '<S52>/Switch' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    if (rtb_LowerRelop1_b) {
      WhlSpdRR_mps = 0.0;
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
  WhlSpdRR_mps -= rtb_Gain4;

  /* RelationalOperator: '<S59>/LowerRelop1' */
  rtb_Compare = (WhlSpdRR_mps > rtb_Switch2_ow);

  /* Switch: '<S59>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S42>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S59>/UpperRelop' */
    rtb_LowerRelop1_b = (WhlSpdRR_mps < elapseTime);

    /* Switch: '<S59>/Switch' */
    if (rtb_LowerRelop1_b) {
      WhlSpdRR_mps = elapseTime;
    }

    /* End of Switch: '<S59>/Switch' */
    rtb_Switch2_ow = WhlSpdRR_mps;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pt = rtb_Switch2_ow +
    rtb_Gain4;

  /* Switch: '<S7>/Switch8' incorporates:
   *  UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (WhlSpdFR != 0.0) {
    elapseTime = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pt;
  } else {
    /* Product: '<S7>/Product' incorporates:
     *  Constant: '<S7>/Constant17'
     */
    elapseTime = rtb_Ax * 0.099999999999999978;
  }

  /* End of Switch: '<S7>/Switch8' */

  /* RelationalOperator: '<S26>/LowerRelop1' */
  rtb_Compare = (elapseTime > rtb_Gain5);

  /* Switch: '<S26>/Switch2' */
  if (rtb_Compare) {
    rtb_Add10 = (real32_T)rtb_Gain5;
  } else {
    /* RelationalOperator: '<S26>/UpperRelop' incorporates:
     *  Constant: '<S7>/Constant15'
     */
    rtb_LowerRelop1_b = (elapseTime < 0.0);

    /* Switch: '<S26>/Switch' incorporates:
     *  Constant: '<S7>/Constant15'
     */
    if (rtb_LowerRelop1_b) {
      rtb_Add10 = 0.0F;
    } else {
      rtb_Add10 = (real32_T)elapseTime;
    }

    /* End of Switch: '<S26>/Switch' */
  }

  /* End of Switch: '<S26>/Switch2' */

  /* Switch: '<S31>/Switch6' incorporates:
   *  Constant: '<S31>/Verror_Reset'
   *  UnitDelay: '<S81>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl) {
    rtb_Switch2_b0 = rtb_Add6;
  } else {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Switch: '<S31>/Switch6' */

  /* Product: '<S31>/Product' incorporates:
   *  Constant: '<S31>/P_Gain'
   */
  rtb_FRWhlStrAng = rtb_Switch2_b0 * 40.0F;

  /* Sum: '<S31>/Add11' */
  Acc_POS_n = rtb_Add10 - rtb_FRWhlStrAng;

  /* UnitDelay: '<S31>/Unit Delay5' */
  rtb_Fz3 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* Product: '<S31>/Product2' */
  rtb_Fz3 *= rtb_Add6;

  /* RelationalOperator: '<S78>/Compare' incorporates:
   *  Constant: '<S78>/Constant'
   */
  rtb_Compare = (rtb_Fz3 <= 0.0F);

  /* UnitDelay: '<S31>/Unit Delay' */
  rtb_Fz3 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f;

  /* Switch: '<S31>/Switch3' incorporates:
   *  Constant: '<S31>/Verror_Reset1'
   */
  if (rtb_Compare) {
    rtb_Fz3 = 0.0F;
  }

  /* End of Switch: '<S31>/Switch3' */

  /* Sum: '<S31>/Add2' */
  rtb_Fz3 += rtb_Switch2_b0;

  /* Saturate: '<S31>/Saturation2' */
  if (rtb_Fz3 > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f = 400.0F;
  } else if (rtb_Fz3 < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f = rtb_Fz3;
  }

  /* End of Saturate: '<S31>/Saturation2' */

  /* RelationalOperator: '<S86>/Compare' incorporates:
   *  UnitDelay: '<S81>/Unit Delay1'
   */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl;

  /* UnitDelay: '<S80>/Delay Input1'
   *
   * Block description for '<S80>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_Compare = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_j;

  /* RelationalOperator: '<S80>/FixPt Relational Operator' */
  rtb_Compare = ((int32_T)rtb_LowerRelop1_b > (int32_T)rtb_Compare);

  /* Switch: '<S31>/Switch' incorporates:
   *  Constant: '<S31>/Integr_StartPoint'
   */
  if (rtb_Compare) {
    /* Sum: '<S31>/Add4' */
    rtb_Switch2_b0 = rtb_Add10 -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_f;
  } else {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Switch: '<S31>/Switch' */

  /* Lookup_n-D: '<S31>/VehicleStableTarget_mps' */
  rtb_Fz3 = look1_iflf_binlc(rtb_Add7, VehCtrlMdel240926_2018b__ConstP.pooled39,
    VehCtrlMdel240926_2018b__ConstP.pooled66, 3U);

  /* Sum: '<S31>/Add5' */
  rtb_Fz3 += rtb_Add7;

  /* Sum: '<S31>/Add10' */
  rtb_Fz3 = rtb_CastToBoolean1 - rtb_Fz3;

  /* RelationalOperator: '<S31>/Relational Operator' incorporates:
   *  Constant: '<S31>/Verror'
   */
  rtb_Compare = (rtb_Fz3 < 0.0F);

  /* Logic: '<S31>/Logical Operator4' incorporates:
   *  UnitDelay: '<S81>/Unit Delay1'
   */
  rtb_Compare = (rtb_Compare &&
                 VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl);

  /* Switch: '<S31>/Switch1' incorporates:
   *  Constant: '<S31>/Trq_IReset'
   *  Constant: '<S31>/Trq_I_FF'
   */
  if (rtb_Compare) {
    rtb_Fz3 = 20.0F;
  } else {
    rtb_Fz3 = 0.0F;
  }

  /* End of Switch: '<S31>/Switch1' */

  /* Sum: '<S31>/Add6' incorporates:
   *  UnitDelay: '<S31>/Unit Delay'
   */
  rtb_Switch2_b0 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f +
                    rtb_Switch2_b0) + rtb_Fz3;

  /* Product: '<S31>/Product1' incorporates:
   *  Constant: '<S31>/I_Gain'
   */
  rtb_Add4_j = rtb_Switch2_b0 * 10.0F;

  /* RelationalOperator: '<S83>/LowerRelop1' */
  rtb_Compare = (rtb_Add4_j > Acc_POS_n);

  /* Switch: '<S83>/Switch2' */
  if (!rtb_Compare) {
    /* Gain: '<S31>/Gain3' */
    Acc_POS_n = -rtb_FRWhlStrAng;

    /* RelationalOperator: '<S83>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Add4_j < Acc_POS_n);

    /* Switch: '<S83>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Add4_j = Acc_POS_n;
    }

    /* End of Switch: '<S83>/Switch' */
    Acc_POS_n = rtb_Add4_j;
  }

  /* End of Switch: '<S83>/Switch2' */

  /* Sum: '<S31>/Add7' incorporates:
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i = rtb_FRWhlStrAng +
    Acc_POS_n;

  /* Lookup_n-D: '<S31>/VehicleStableTarget_mps1' */
  rtb_Switch2_b0 = look1_iflf_binlc(rtb_Add7,
    VehCtrlMdel240926_2018b__ConstP.pooled39,
    VehCtrlMdel240926_2018b__ConstP.pooled66, 3U);

  /* Sum: '<S31>/Add13' */
  rtb_Add7 += rtb_Switch2_b0;

  /* Sum: '<S31>/Add12' */
  rtb_CastToBoolean1 -= rtb_Add7;

  /* RelationalOperator: '<S31>/Relational Operator1' incorporates:
   *  Constant: '<S31>/Verror1'
   */
  rtb_Compare = (rtb_CastToBoolean1 < 0.0F);

  /* RelationalOperator: '<S31>/Relational Operator2' incorporates:
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  TrqR_cmd_raw = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i >=
                  rtb_Add10);

  /* RelationalOperator: '<S79>/Compare' incorporates:
   *  Constant: '<S79>/Constant'
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  rtb_LogicalOperator7_m = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i <=
    5.0F);

  /* Logic: '<S31>/OR' */
  TrqR_cmd_raw = (TrqR_cmd_raw || rtb_LogicalOperator7_m);

  /* Logic: '<S31>/Logical Operator5' incorporates:
   *  UnitDelay: '<S31>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e = (rtb_Compare &&
    TrqR_cmd_raw);

  /* Switch: '<S31>/Switch2' incorporates:
   *  Switch: '<S31>/Switch7'
   *  UnitDelay: '<S31>/Unit Delay3'
   *  UnitDelay: '<S81>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl) {
    /* RelationalOperator: '<S84>/LowerRelop1' incorporates:
     *  Constant: '<S31>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S31>/Unit Delay4'
     */
    rtb_LogicalOperator2 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i >
      235.0F);

    /* Switch: '<S84>/Switch2' incorporates:
     *  Constant: '<S31>/TCS_TrqRequest_Max2'
     */
    if (rtb_LogicalOperator2) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr = 235.0F;
    } else {
      /* RelationalOperator: '<S84>/UpperRelop' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S31>/Unit Delay4'
       */
      rtb_LogicalOperator2 =
        (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i < 0.0F);

      /* Switch: '<S84>/Switch' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S31>/Unit Delay4'
       */
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr = 0.0F;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i;
      }

      /* End of Switch: '<S84>/Switch' */
    }

    /* End of Switch: '<S84>/Switch2' */

    /* RelationalOperator: '<S85>/LowerRelop1' */
    rtb_LogicalOperator2 = (rtb_Add10 >
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr);

    /* Switch: '<S85>/Switch2' */
    if (!rtb_LogicalOperator2) {
      /* RelationalOperator: '<S85>/UpperRelop' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min1'
       */
      rtb_LogicalOperator2 = (rtb_Add10 < 0.0F);

      /* Switch: '<S85>/Switch' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min1'
       */
      if (rtb_LogicalOperator2) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr = 0.0F;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr = rtb_Add10;
      }

      /* End of Switch: '<S85>/Switch' */
    }

    /* End of Switch: '<S85>/Switch2' */
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e) {
      /* Switch: '<S31>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr = rtb_Add10;
    }
  }

  /* End of Switch: '<S31>/Switch2' */

  /* Switch: '<S7>/Switch6' incorporates:
   *  UnitDelay: '<S31>/Unit Delay2'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.TCSF_Enable_OUT != 0.0) {
    rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr;
  }

  /* End of Switch: '<S7>/Switch6' */

  /* Lookup_n-D: '<S7>/AMK2' */
  rtb_Gain4 = look1_binlx(MCFR_ActualVelocity,
    VehCtrlMdel240926_2018b__ConstP.pooled10,
    VehCtrlMdel240926_2018b__ConstP.pooled17, 19U);

  /* Gain: '<S7>/Gain8' */
  WhlSpdRR_mps = 0.95 * rtb_Gain4;

  /* Product: '<S28>/Product3' */
  rtb_Gain4 = 12.5 * rtb_Add1;

  /* Product: '<S28>/Product2' */
  rtb_Add1 = rtb_Gain4 * 9550.0;

  /* Sum: '<S28>/Add3' incorporates:
   *  Constant: '<S28>/RPM_min4'
   */
  rtb_Gain4 = MCFL_ActualVelocity + 10.0;

  /* MinMax: '<S28>/Max2' incorporates:
   *  Constant: '<S28>/RPM_min5'
   */
  rtb_Gain5 = fmax(rtb_Gain4, 1.0);

  /* Product: '<S28>/Divide2' */
  rtb_Add1 /= rtb_Gain5;

  /* UnitDelay: '<S41>/Delay Input2'
   *
   * Block description for '<S41>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hj;

  /* SampleTimeMath: '<S41>/sample time'
   *
   * About '<S41>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S41>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  rtb_Gain5 = 2000.0 * elapseTime;

  /* RelationalOperator: '<S48>/LowerRelop1' */
  rtb_Compare = (rtb_UkYk1_nc > rtb_VxIMU_est);

  /* Switch: '<S48>/Switch2' */
  if (rtb_Compare) {
    rtb_Switch2_ow = rtb_VxIMU_est;
  } else {
    /* RelationalOperator: '<S48>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    rtb_LogicalOperator2 = (rtb_UkYk1_nc < 0.0);

    /* Switch: '<S48>/Switch' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    if (rtb_LogicalOperator2) {
      rtb_UkYk1_nc = 0.0;
    }

    /* End of Switch: '<S48>/Switch' */
    rtb_Switch2_ow = rtb_UkYk1_nc;
  }

  /* End of Switch: '<S48>/Switch2' */

  /* Sum: '<S10>/Add11' incorporates:
   *  UnitDelay: '<S47>/Delay Input2'
   *
   * Block description for '<S47>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1_nc = rtb_Switch2_ow +
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_lt;

  /* RelationalOperator: '<S51>/LowerRelop1' */
  rtb_Compare = (rtb_UkYk1_nc > rtb_VxIMU_est);

  /* Switch: '<S51>/Switch2' */
  if (rtb_Compare) {
    rtb_UkYk1_nc = rtb_VxIMU_est;
  } else {
    /* RelationalOperator: '<S51>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    rtb_LogicalOperator2 = (rtb_UkYk1_nc < 0.0);

    /* Switch: '<S51>/Switch' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    if (rtb_LogicalOperator2) {
      rtb_UkYk1_nc = 0.0;
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
  rtb_UkYk1_nc -= rtb_Gain4;

  /* RelationalOperator: '<S58>/LowerRelop1' */
  rtb_Compare = (rtb_UkYk1_nc > rtb_Gain5);

  /* Switch: '<S58>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S41>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S58>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_UkYk1_nc < elapseTime);

    /* Switch: '<S58>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_UkYk1_nc = elapseTime;
    }

    /* End of Switch: '<S58>/Switch' */
    rtb_Gain5 = rtb_UkYk1_nc;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hj = rtb_Gain5 + rtb_Gain4;

  /* Switch: '<S7>/Switch9' incorporates:
   *  UnitDelay: '<S41>/Delay Input2'
   *
   * Block description for '<S41>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (WhlSpdFR != 0.0) {
    elapseTime = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hj;
  } else {
    /* Product: '<S7>/Product1' incorporates:
     *  Constant: '<S7>/Constant17'
     */
    elapseTime = rtb_Ax * 0.099999999999999978;
  }

  /* End of Switch: '<S7>/Switch9' */

  /* RelationalOperator: '<S27>/LowerRelop1' */
  rtb_Compare = (elapseTime > rtb_Add1);

  /* Switch: '<S27>/Switch2' */
  if (rtb_Compare) {
    rtb_Add7 = (real32_T)rtb_Add1;
  } else {
    /* RelationalOperator: '<S27>/UpperRelop' incorporates:
     *  Constant: '<S7>/Constant2'
     */
    rtb_LogicalOperator2 = (elapseTime < 0.0);

    /* Switch: '<S27>/Switch' incorporates:
     *  Constant: '<S7>/Constant2'
     */
    if (rtb_LogicalOperator2) {
      rtb_Add7 = 0.0F;
    } else {
      rtb_Add7 = (real32_T)elapseTime;
    }

    /* End of Switch: '<S27>/Switch' */
  }

  /* End of Switch: '<S27>/Switch2' */

  /* Switch: '<S30>/Switch6' incorporates:
   *  Constant: '<S30>/Verror_Reset'
   *  UnitDelay: '<S72>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp) {
    rtb_Switch2_b0 = rtb_deltafalllimit_ap;
  } else {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Switch: '<S30>/Switch6' */

  /* Product: '<S30>/Product' incorporates:
   *  Constant: '<S30>/P_Gain'
   */
  rtb_CastToBoolean1 = rtb_Switch2_b0 * 40.0F;

  /* Sum: '<S30>/Add11' */
  Acc_POS_n = rtb_Add7 - rtb_CastToBoolean1;

  /* UnitDelay: '<S30>/Unit Delay5' */
  rtb_Fz3 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_ip;

  /* Product: '<S30>/Product2' */
  rtb_Fz3 *= rtb_deltafalllimit_ap;

  /* RelationalOperator: '<S69>/Compare' incorporates:
   *  Constant: '<S69>/Constant'
   */
  rtb_Compare = (rtb_Fz3 <= 0.0F);

  /* UnitDelay: '<S30>/Unit Delay' */
  rtb_Fz3 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr;

  /* Switch: '<S30>/Switch3' incorporates:
   *  Constant: '<S30>/Verror_Reset1'
   */
  if (rtb_Compare) {
    rtb_Fz3 = 0.0F;
  }

  /* End of Switch: '<S30>/Switch3' */

  /* Sum: '<S30>/Add2' */
  rtb_Fz3 += rtb_Switch2_b0;

  /* Saturate: '<S30>/Saturation2' */
  if (rtb_Fz3 > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = 400.0F;
  } else if (rtb_Fz3 < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = rtb_Fz3;
  }

  /* End of Saturate: '<S30>/Saturation2' */

  /* RelationalOperator: '<S77>/Compare' incorporates:
   *  UnitDelay: '<S72>/Unit Delay1'
   */
  rtb_LogicalOperator2 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp;

  /* UnitDelay: '<S71>/Delay Input1'
   *
   * Block description for '<S71>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_Compare = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_b;

  /* RelationalOperator: '<S71>/FixPt Relational Operator' */
  rtb_Compare = ((int32_T)rtb_LogicalOperator2 > (int32_T)rtb_Compare);

  /* Switch: '<S30>/Switch' incorporates:
   *  Constant: '<S30>/Integr_StartPoint'
   */
  if (rtb_Compare) {
    /* Sum: '<S30>/Add4' */
    rtb_Switch2_b0 = rtb_Add7 -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g4;
  } else {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Switch: '<S30>/Switch' */

  /* Lookup_n-D: '<S30>/VehicleStableTarget_mps' */
  rtb_Fz3 = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240926_2018b__ConstP.pooled39,
    VehCtrlMdel240926_2018b__ConstP.pooled66, 3U);

  /* Sum: '<S30>/Add5' */
  rtb_Fz3 += rtb_Switch2_mn;

  /* Sum: '<S30>/Add10' */
  rtb_Fz3 = rtb_Gain3_o - rtb_Fz3;

  /* RelationalOperator: '<S30>/Relational Operator' incorporates:
   *  Constant: '<S30>/Verror'
   */
  rtb_Compare = (rtb_Fz3 < 0.0F);

  /* Logic: '<S30>/Logical Operator4' incorporates:
   *  UnitDelay: '<S72>/Unit Delay1'
   */
  rtb_Compare = (rtb_Compare &&
                 VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp);

  /* Switch: '<S30>/Switch1' incorporates:
   *  Constant: '<S30>/Trq_IReset'
   *  Constant: '<S30>/Trq_I_FF'
   */
  if (rtb_Compare) {
    rtb_Fz3 = 20.0F;
  } else {
    rtb_Fz3 = 0.0F;
  }

  /* End of Switch: '<S30>/Switch1' */

  /* Sum: '<S30>/Add6' incorporates:
   *  UnitDelay: '<S30>/Unit Delay'
   */
  rtb_Switch2_b0 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr +
                    rtb_Switch2_b0) + rtb_Fz3;

  /* Product: '<S30>/Product1' incorporates:
   *  Constant: '<S30>/I_Gain'
   */
  rtb_Add4_j = rtb_Switch2_b0 * 10.0F;

  /* RelationalOperator: '<S74>/LowerRelop1' */
  rtb_Compare = (rtb_Add4_j > Acc_POS_n);

  /* Switch: '<S74>/Switch2' */
  if (!rtb_Compare) {
    /* Gain: '<S30>/Gain3' */
    Acc_POS_n = -rtb_CastToBoolean1;

    /* RelationalOperator: '<S74>/UpperRelop' */
    rtb_Compare = (rtb_Add4_j < Acc_POS_n);

    /* Switch: '<S74>/Switch' */
    if (rtb_Compare) {
      rtb_Add4_j = Acc_POS_n;
    }

    /* End of Switch: '<S74>/Switch' */
    Acc_POS_n = rtb_Add4_j;
  }

  /* End of Switch: '<S74>/Switch2' */

  /* Sum: '<S30>/Add7' incorporates:
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l = rtb_CastToBoolean1 +
    Acc_POS_n;

  /* Lookup_n-D: '<S30>/VehicleStableTarget_mps1' */
  rtb_Switch2_b0 = look1_iflf_binlc(rtb_Switch2_mn,
    VehCtrlMdel240926_2018b__ConstP.pooled39,
    VehCtrlMdel240926_2018b__ConstP.pooled66, 3U);

  /* Sum: '<S30>/Add13' */
  rtb_Switch2_mn += rtb_Switch2_b0;

  /* Sum: '<S30>/Add12' */
  rtb_Gain3_o -= rtb_Switch2_mn;

  /* RelationalOperator: '<S30>/Relational Operator1' incorporates:
   *  Constant: '<S30>/Verror1'
   */
  rtb_Compare = (rtb_Gain3_o < 0.0F);

  /* RelationalOperator: '<S30>/Relational Operator2' incorporates:
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  TrqR_cmd_raw = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l >=
                  rtb_Add7);

  /* RelationalOperator: '<S70>/Compare' incorporates:
   *  Constant: '<S70>/Constant'
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  rtb_LogicalOperator7_m = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l <=
    5.0F);

  /* Logic: '<S30>/OR' */
  TrqR_cmd_raw = (TrqR_cmd_raw || rtb_LogicalOperator7_m);

  /* Logic: '<S30>/Logical Operator5' incorporates:
   *  UnitDelay: '<S30>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_ip = (rtb_Compare &&
    TrqR_cmd_raw);

  /* Switch: '<S30>/Switch2' incorporates:
   *  Switch: '<S30>/Switch7'
   *  UnitDelay: '<S30>/Unit Delay3'
   *  UnitDelay: '<S72>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp) {
    /* RelationalOperator: '<S75>/LowerRelop1' incorporates:
     *  Constant: '<S30>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S30>/Unit Delay4'
     */
    rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l > 235.0F);

    /* Switch: '<S75>/Switch2' incorporates:
     *  Constant: '<S30>/TCS_TrqRequest_Max2'
     */
    if (rtb_Compare) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 235.0F;
    } else {
      /* RelationalOperator: '<S75>/UpperRelop' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S30>/Unit Delay4'
       */
      rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l < 0.0F);

      /* Switch: '<S75>/Switch' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S30>/Unit Delay4'
       */
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 0.0F;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l;
      }

      /* End of Switch: '<S75>/Switch' */
    }

    /* End of Switch: '<S75>/Switch2' */

    /* RelationalOperator: '<S76>/LowerRelop1' */
    rtb_Compare = (rtb_Add7 >
                   VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b);

    /* Switch: '<S76>/Switch2' */
    if (!rtb_Compare) {
      /* RelationalOperator: '<S76>/UpperRelop' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min1'
       */
      rtb_Compare = (rtb_Add7 < 0.0F);

      /* Switch: '<S76>/Switch' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min1'
       */
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 0.0F;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = rtb_Add7;
      }

      /* End of Switch: '<S76>/Switch' */
    }

    /* End of Switch: '<S76>/Switch2' */
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_ip) {
      /* Switch: '<S30>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = rtb_Add7;
    }
  }

  /* End of Switch: '<S30>/Switch2' */

  /* Switch: '<S7>/Switch7' incorporates:
   *  UnitDelay: '<S30>/Unit Delay2'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.TCSF_Enable_OUT != 0.0) {
    rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b;
  }

  /* End of Switch: '<S7>/Switch7' */

  /* Lookup_n-D: '<S7>/AMK3' */
  rtb_Add1 = look1_binlx(MCFL_ActualVelocity,
    VehCtrlMdel240926_2018b__ConstP.pooled10,
    VehCtrlMdel240926_2018b__ConstP.pooled17, 19U);

  /* Gain: '<S7>/Gain7' */
  rtb_UkYk1_nc = 0.95 * rtb_Add1;

  /* Logic: '<S7>/Logical Operator2' */
  rtb_Compare = !VehCtrlMdel240926_2018b_amksp_B.VehReady;

  /* Logic: '<S7>/Logical Operator3' */
  TrqR_cmd_raw = (rtb_Compare || (Trq_CUT != 0.0));

  /* Switch: '<S7>/Switch2' incorporates:
   *  Constant: '<S7>/Constant4'
   */
  if (TrqR_cmd_raw) {
    FLWhlStrAng = 0.0F;
  } else {
    /* Logic: '<S7>/NOT3' */
    rtb_Compare = !rtb_Compare_am;

    /* Switch: '<S7>/Switch11' incorporates:
     *  Constant: '<S7>/Constant23'
     */
    if (rtb_Compare) {
      /* Sum: '<S7>/Add2' */
      elapseTime = rtb_Add7 - rtb_UkYk1_nc;

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
      rtb_Add1 = rtb_Add10 - WhlSpdRR_mps;

      /* Saturate: '<S7>/Saturation2' */
      if (rtb_Add1 > 10.0) {
        rtb_Add1 = 10.0;
      } else {
        if (rtb_Add1 < 0.0) {
          rtb_Add1 = 0.0;
        }
      }

      /* End of Saturate: '<S7>/Saturation2' */

      /* MinMax: '<S7>/Min3' */
      rtb_Add1 = fmin(rtb_Add1, elapseTime);

      /* Gain: '<S7>/Gain1' */
      rtb_Add1 *= 2.0;

      /* Gain: '<S7>/Gain10' */
      rtb_Add1 *= 3.0483870967741935;
    } else {
      rtb_Add1 = 0.0;
    }

    /* End of Switch: '<S7>/Switch11' */

    /* Switch: '<S7>/Switch5' incorporates:
     *  UnitDelay: '<S32>/Unit Delay2'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.TCSR_Enable_OUT != 0.0) {
      FLWhlStrAng = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g;
    }

    /* End of Switch: '<S7>/Switch5' */

    /* Sum: '<S7>/Add3' */
    elapseTime = FLWhlStrAng + rtb_Add1;

    /* Switch: '<S7>/Switch12' */
    if (rtb_Compare_am) {
      /* Lookup_n-D: '<S7>/228RWD' */
      rtb_Add1 = look1_binlx(RPM, VehCtrlMdel240926_2018b__ConstP.pooled4,
        VehCtrlMdel240926_2018b__ConstP.u28RWD_tableData, 26U);

      /* Gain: '<S7>/Gain9' */
      rtb_Add1 *= 0.95;
    } else {
      /* Lookup_n-D: '<S7>/228RWD1' */
      rtb_Add1 = look1_binlx(RPM, VehCtrlMdel240926_2018b__ConstP.pooled4,
        VehCtrlMdel240926_2018b__ConstP.u28RWD1_tableData, 26U);

      /* Gain: '<S7>/Gain11' */
      rtb_Add1 *= 0.95;
    }

    /* End of Switch: '<S7>/Switch12' */

    /* MinMax: '<S7>/Min2' */
    rtb_Add1 = fmin(rtb_Add1, elapseTime);

    /* Gain: '<S7>/Gain6' */
    elapseTime = 4.7619047619047619 * rtb_Add1;

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefRear' */
    FLWhlStrAng = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefRear_bp01Dat,
      VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefRear_tableDa, 1U);

    /* RelationalOperator: '<S22>/LowerRelop1' */
    rtb_Compare = (elapseTime > FLWhlStrAng);

    /* Switch: '<S22>/Switch2' */
    if (!rtb_Compare) {
      /* RelationalOperator: '<S22>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      rtb_Compare = (elapseTime < 0.0);

      /* Switch: '<S22>/Switch' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      if (rtb_Compare) {
        FLWhlStrAng = 0.0F;
      } else {
        FLWhlStrAng = (real32_T)elapseTime;
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
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd;

  /* Sum: '<S19>/Difference Inputs1'
   *
   * Block description for '<S19>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain3_o = FLWhlStrAng - rtb_Switch2_b0;

  /* SampleTimeMath: '<S19>/sample time'
   *
   * About '<S19>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S19>/delta rise limit' */
  Acc_POS_n = (real32_T)(25000.0 * elapseTime);

  /* RelationalOperator: '<S65>/LowerRelop1' */
  rtb_Compare = (rtb_Gain3_o > Acc_POS_n);

  /* Switch: '<S65>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S19>/delta fall limit' */
    FLWhlStrAng = (real32_T)(-25000.0 * elapseTime);

    /* RelationalOperator: '<S65>/UpperRelop' */
    rtb_Compare = (rtb_Gain3_o < FLWhlStrAng);

    /* Switch: '<S65>/Switch' */
    if (rtb_Compare) {
      rtb_Gain3_o = FLWhlStrAng;
    }

    /* End of Switch: '<S65>/Switch' */
    Acc_POS_n = rtb_Gain3_o;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd = Acc_POS_n +
    rtb_Switch2_b0;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd > 1000.0F) {
    EmraxTrqR_cmd = 1000.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd < 0.0F) {
    EmraxTrqR_cmd = 0.0F;
  } else {
    EmraxTrqR_cmd = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd;
  }

  /* End of Saturate: '<S7>/Saturation1' */

  /* Logic: '<S7>/Logical Operator5' */
  rtb_Compare = (VehCtrlMdel240926_2018b_amksp_B.MCFR_TorqueOn &&
                 VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueOn);

  /* Logic: '<S7>/Logical Operator6' */
  TroqueOn = !rtb_Compare;

  /* Logic: '<S7>/Logical Operator4' */
  Trq_CUT_final = (TroqueOn || (AMK_Trq_CUT != 0.0) || TrqR_cmd_raw ||
                   TrqR_cmd_raw);

  /* Logic: '<S7>/OR' */
  rtb_Compare = (Trq_CUT_final || rtb_Compare_am);

  /* Lookup_n-D: '<S7>/BrakeCompensateCoefFront1' */
  rtb_Gain3_o = look1_iflf_binlc((real32_T)Brk_F,
    VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefFront1_bp01D,
    VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefFront1_table, 1U);

  /* Switch: '<S7>/Switch3' incorporates:
   *  Constant: '<S7>/Constant19'
   */
  if (rtb_Compare) {
    FLWhlStrAng = 0.0F;
  } else {
    /* MinMax: '<S7>/Min1' */
    elapseTime = fmin(rtb_Add10, WhlSpdRR_mps);

    /* RelationalOperator: '<S23>/LowerRelop1' */
    rtb_Compare = (elapseTime > rtb_Gain3_o);

    /* Switch: '<S23>/Switch2' */
    if (rtb_Compare) {
      FLWhlStrAng = rtb_Gain3_o;
    } else {
      /* RelationalOperator: '<S23>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      rtb_Compare = (elapseTime < 0.0);

      /* Switch: '<S23>/Switch' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      if (rtb_Compare) {
        FLWhlStrAng = 0.0F;
      } else {
        FLWhlStrAng = (real32_T)elapseTime;
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
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn;

  /* Sum: '<S20>/Difference Inputs1'
   *
   * Block description for '<S20>/Difference Inputs1':
   *
   *  Add in CPU
   */
  FLWhlStrAng -= rtb_Switch2_b0;

  /* SampleTimeMath: '<S20>/sample time'
   *
   * About '<S20>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S20>/delta rise limit' */
  Acc_POS_n = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S66>/LowerRelop1' */
  rtb_Compare = (FLWhlStrAng > Acc_POS_n);

  /* Switch: '<S66>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S20>/delta fall limit' */
    Acc_POS_n = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S66>/UpperRelop' */
    rtb_Compare = (FLWhlStrAng < Acc_POS_n);

    /* Switch: '<S66>/Switch' */
    if (rtb_Compare) {
      FLWhlStrAng = Acc_POS_n;
    }

    /* End of Switch: '<S66>/Switch' */
    Acc_POS_n = FLWhlStrAng;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn = Acc_POS_n +
    rtb_Switch2_b0;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn > 21.0F) {
    AMKTrqFR_cmd = 21.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn < -21.0F) {
    AMKTrqFR_cmd = -21.0F;
  } else {
    AMKTrqFR_cmd = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn;
  }

  /* End of Saturate: '<S7>/Saturation3' */

  /* Logic: '<S7>/OR1' */
  rtb_Compare = (Trq_CUT_final || rtb_Compare_am);

  /* Switch: '<S7>/Switch4' incorporates:
   *  Constant: '<S7>/Constant8'
   */
  if (rtb_Compare) {
    rtb_Gain3_o = 0.0F;
  } else {
    /* MinMax: '<S7>/Min' */
    elapseTime = fmin(rtb_UkYk1_nc, rtb_Add7);

    /* RelationalOperator: '<S24>/LowerRelop1' */
    rtb_Compare_am = (elapseTime > rtb_Gain3_o);

    /* Switch: '<S24>/Switch2' */
    if (!rtb_Compare_am) {
      /* RelationalOperator: '<S24>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      rtb_Compare_am = (elapseTime < 0.0);

      /* Switch: '<S24>/Switch' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      if (rtb_Compare_am) {
        rtb_Gain3_o = 0.0F;
      } else {
        rtb_Gain3_o = (real32_T)elapseTime;
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
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib;

  /* Sum: '<S21>/Difference Inputs1'
   *
   * Block description for '<S21>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain3_o -= rtb_Switch2_b0;

  /* SampleTimeMath: '<S21>/sample time'
   *
   * About '<S21>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S21>/delta rise limit' */
  Acc_POS_n = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S67>/LowerRelop1' */
  rtb_Compare = (rtb_Gain3_o > Acc_POS_n);

  /* Switch: '<S67>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S21>/delta fall limit' */
    FLWhlStrAng = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S67>/UpperRelop' */
    rtb_Compare_am = (rtb_Gain3_o < FLWhlStrAng);

    /* Switch: '<S67>/Switch' */
    if (rtb_Compare_am) {
      rtb_Gain3_o = FLWhlStrAng;
    }

    /* End of Switch: '<S67>/Switch' */
    Acc_POS_n = rtb_Gain3_o;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib = Acc_POS_n +
    rtb_Switch2_b0;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib > 21.0F) {
    AMKTrqFL_cmd = 21.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib < -21.0F) {
    AMKTrqFL_cmd = -21.0F;
  } else {
    AMKTrqFL_cmd = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib;
  }

  /* End of Saturate: '<S7>/Saturation4' */

  /* Sum: '<S10>/Add3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_c = rtb_Yk1_l - rtb_UkYk1;

  /* SignalConversion generated from: '<S7>/Constant13' incorporates:
   *  Constant: '<S7>/Constant13'
   */
  VehCtrlMdel240926_2018b_amksp_B.VCU_SpdCmd_Emrax = 4200.0F;

  /* Update for UnitDelay: '<S7>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_n =
    VehCtrlMdel240926_2018b_amksp_B.DYC_Enable_OUT;

  /* Update for UnitDelay: '<S10>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j = rtb_CastToBoolean;

  /* Update for UnitDelay: '<S10>/Unit Delay6' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE_b =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i;

  /* Update for UnitDelay: '<S32>/Unit Delay5' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_l = rtb_deltafalllimit_cz;

  /* Update for UnitDelay: '<S32>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g = rtb_StrWhlAngV_c;

  /* Update for UnitDelay: '<S91>/Delay Input1'
   *
   * Block description for '<S91>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_e = rtb_ignition_e;

  /* Update for UnitDelay: '<S31>/Unit Delay5' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = rtb_Add6;

  /* Update for UnitDelay: '<S31>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_f = rtb_FRWhlStrAng;

  /* Update for UnitDelay: '<S80>/Delay Input1'
   *
   * Block description for '<S80>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_j = rtb_LowerRelop1_b;

  /* Update for UnitDelay: '<S30>/Unit Delay5' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_ip = rtb_deltafalllimit_ap;

  /* Update for UnitDelay: '<S30>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g4 = rtb_CastToBoolean1;

  /* Update for UnitDelay: '<S71>/Delay Input1'
   *
   * Block description for '<S71>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_b = rtb_LogicalOperator2;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/10ms1' */

  /* S-Function (fcncallgen): '<S5>/10ms2' incorporates:
   *  SubSystem: '<S5>/VCU2AMKMCUFL'
   */
  /* Switch: '<S353>/Switch1' incorporates:
   *  Constant: '<S353>/Constant1'
   *  Constant: '<S353>/Constant2'
   *  Switch: '<S353>/Switch'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueOn) {
    VehCtrlMdel240926_2018b_amksp_B.Switch1_l = -21.0;
    VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueLimitP = 21.0;
  } else {
    VehCtrlMdel240926_2018b_amksp_B.Switch1_l = 0.0;
    VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueLimitP = 0.0;
  }

  /* End of Switch: '<S353>/Switch1' */

  /* S-Function (scanpack): '<S353>/CAN Pack1' */
  /* S-Function (scanpack): '<S353>/CAN Pack1' */
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

  /* S-Function (ecucoder_canmessage): '<S353>/CANPackMessage' */

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

  /* S-Function (ec5744_cantransmitslb): '<S353>/CANTransmit' */

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
  /* Switch: '<S354>/Switch1' incorporates:
   *  Constant: '<S354>/Constant'
   *  Constant: '<S354>/Constant1'
   *  Switch: '<S354>/Switch'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.MCFR_TorqueOn) {
    VehCtrlMdel240926_2018b_amksp_B.Switch1 = -21.0;
    VehCtrlMdel240926_2018b_amksp_B.Switch = 21.0;
  } else {
    VehCtrlMdel240926_2018b_amksp_B.Switch1 = 0.0;
    VehCtrlMdel240926_2018b_amksp_B.Switch = 0.0;
  }

  /* End of Switch: '<S354>/Switch1' */

  /* S-Function (scanpack): '<S354>/CAN Pack1' */
  /* S-Function (scanpack): '<S354>/CAN Pack1' */
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.ID = 387U;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Length = 8U;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Extended = 0U;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Remote = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[0] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[1] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[2] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[3] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[4] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[5] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[6] = 0;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[7] = 0;

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[2] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[2] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[3] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[3] | (uint8_T)
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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[6] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[6] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[7] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[7] | (uint8_T)
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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[4] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[4] | (uint8_T)
              ((uint16_T)(tempValue & (uint16_T)0xFFU));
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[5] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[5] | (uint8_T)
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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[1] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[1] | (uint8_T)
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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[1] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[1] | (uint8_T)
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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[1] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[1] | (uint8_T)
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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[1] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[1] | (uint8_T)
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

  /* S-Function (ecucoder_canmessage): '<S354>/CANPackMessage' */

  /*Pack CAN message*/
  {
    uint8 canpackloop= 0;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[0]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[1]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[2]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[3]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[4]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[5]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[6]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
    VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[7]=
      VehCtrlMdel240926_2018b_amksp_B.CANPack1_b.Data[canpackloop];
    canpackloop++;
  }

  /* S-Function (ec5744_cantransmitslb): '<S354>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN1BUF9TX[8];
    uint8 can1buf9looptx= 0;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[0];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[1];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[2];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[3];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[4];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[5];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[6];
    can1buf9looptx++;
    CAN1BUF9TX[can1buf9looptx]=
      VehCtrlMdel240926_2018b_amksp_B.CANPackMessage_f[7];
    can1buf9looptx++;
    VehCtrlMdel240926_2018b_amksp_B.CANTransmit_l= ec_can_transmit(1, 9, 0, 387U,
      8, CAN1BUF9TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms4' */

  /* S-Function (fcncallgen): '<S5>/50ms3' incorporates:
   *  SubSystem: '<S5>/VCU2EmraxMCU'
   */
  /* Switch: '<S355>/Switch2' incorporates:
   *  Constant: '<S355>/Constant13'
   *  Constant: '<S355>/Constant17'
   *  Constant: '<S355>/Constant19'
   *  Constant: '<S355>/Constant20'
   *  Switch: '<S355>/Switch3'
   */
  if (TrqR_cmd_raw) {
    Gear_Trs = 0.0;
    Mode_Trs = 0.0;
  } else {
    Gear_Trs = 2.0;
    Mode_Trs = 2.0;
  }

  /* End of Switch: '<S355>/Switch2' */

  /* DataTypeConversion: '<S355>/Cast To Boolean4' */
  VehCtrlMdel240926_2018b_amksp_B.CastToBoolean4 = (real32_T)Gear_Trs;

  /* DataTypeConversion: '<S355>/Cast To Boolean6' */
  VehCtrlMdel240926_2018b_amksp_B.CastToBoolean6 = (real32_T)Mode_Trs;

  /* DataTypeConversion: '<S355>/Data Type Conversion2' */
  VehCtrlMdel240926_2018b_amksp_B.DataTypeConversion2 = (int32_T)floorf
    (EmraxTrqR_cmd);

  /* S-Function (scanpack): '<S355>/CAN Pack1' */
  /* S-Function (scanpack): '<S355>/CAN Pack1' */
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.ID = 146927393U;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.Length = 8U;
  VehCtrlMdel240926_2018b_amksp_B.CANPack1.Extended = 1U;
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
        real32_T result = VehCtrlMdel240926_2018b_amksp_B.CastToBoolean4;

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[4] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[4] | (uint8_T)
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
        real32_T result = VehCtrlMdel240926_2018b_amksp_B.CastToBoolean6;

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[5] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[5] | (uint8_T)
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
        real32_T result = VehCtrlMdel240926_2018b_amksp_B.VCU_SpdCmd_Emrax;

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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[0] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[0] | (uint8_T)
              ((uint16_T)(packedValue & (uint16_T)0xFFU));
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[1] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[1] | (uint8_T)
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
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[2] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[2] | (uint8_T)
              ((uint16_T)(packedValue & (uint16_T)0xFFU));
            VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[3] =
              VehCtrlMdel240926_2018b_amksp_B.CANPack1.Data[3] | (uint8_T)
              ((uint16_T)((uint16_T)(packedValue & (uint16_T)0xFF00U) >> 8));
          }
        }
      }
    }
  }

  /* S-Function (ecucoder_canmessage): '<S355>/CANPackMessage' */

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

  /* S-Function (ec5744_cantransmitslb): '<S355>/CANTransmit' */

  /*Transmit CAN message*/
  {
    uint8 CAN0BUF8TX[8];
    uint8 can0buf8looptx= 0;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[0];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[1];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[2];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[3];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[4];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[5];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[6];
    can0buf8looptx++;
    CAN0BUF8TX[can0buf8looptx]= VehCtrlMdel240926_2018b_amksp_B.CANPackMessage[7];
    can0buf8looptx++;
    VehCtrlMdel240926_2018b_amksp_B.CANTransmit_k= ec_can_transmit(0, 8, 1,
      146927393U, 8, CAN0BUF8TX);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S5>/50ms3' */

  /* S-Function (fcncallgen): '<S5>/10ms6' incorporates:
   *  SubSystem: '<S5>/WP_OUTPUT'
   */
  /* DataTypeConversion: '<S356>/Cast To Single1' */
  VehCtrlMdel240926_2018b_amksp_B.CastToSingle1 = (uint16_T)rtb_CastToDouble;

  /* S-Function (ec5744_pdsslbu3): '<S356>/PowerDriverSwitch(HS)' */

  /* Set level VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON for the specified power driver switch */
  ec_gpio_write(83,VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON);

  /* S-Function (ec5744_pdsslbu3): '<S356>/PowerDriverSwitch(HS)1' */

  /* Set level VehCtrlMdel240926_2018b_amksp_B.bWaterPumpON for the specified power driver switch */
  ec_gpio_write(55,VehCtrlMdel240926_2018b_amksp_B.bWaterPumpON);

  /* S-Function (ec5744_pdpslbu3): '<S356>/PowerDriverPWM' incorporates:
   *  Constant: '<S356>/Constant'
   */

  /* Power driver PWM output for channel 6 */
  ec_pwm_output(6,((uint16_T)1000U),
                VehCtrlMdel240926_2018b_amksp_B.CastToSingle1);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms6' */

  /* S-Function (fcncallgen): '<S369>/10ms' incorporates:
   *  SubSystem: '<S369>/daq10ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S381>/CCPDAQ' */
  ccpDaq(1);

  /* End of Outputs for S-Function (fcncallgen): '<S369>/10ms' */

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
  /* S-Function (fcncallgen): '<S369>/50ms' incorporates:
   *  SubSystem: '<S369>/daq50ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S383>/CCPDAQ' */
  ccpDaq(2);

  /* End of Outputs for S-Function (fcncallgen): '<S369>/50ms' */
}

/* Model step function for TID5 */
void VehCtrlMdel240926_2018b_amkspdlimit_step5(void) /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S368>/100MS' incorporates:
   *  SubSystem: '<S368>/Function-Call Subsystem'
   */
  /* S-Function (ec5744_canreceiveslb): '<S372>/CANReceive' */

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

  /* Call the system: <S372>/Function-Call Subsystem */

  /* Output and update for function-call system: '<S372>/Function-Call Subsystem' */
  {
    uint8_T rtb_Add;
    uint8_T rtb_Compare;

    /* Outputs for Enabled SubSystem: '<S373>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S374>/Enable'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.CANReceive_o2_p > 0) {
      /* RelationalOperator: '<S375>/Compare' incorporates:
       *  Constant: '<S375>/Constant'
       */
      rtb_Add = (uint8_T)(VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[0] ==
                          83);

      /* RelationalOperator: '<S376>/Compare' incorporates:
       *  Constant: '<S376>/Constant'
       */
      rtb_Compare = (uint8_T)(VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[5]
        == 84);

      /* Sum: '<S374>/Add' */
      rtb_Add = (uint8_T)((uint32_T)rtb_Add + rtb_Compare);

      /* RelationalOperator: '<S377>/Compare' incorporates:
       *  Constant: '<S377>/Constant'
       */
      rtb_Compare = (uint8_T)(rtb_Add == 2);

      /* If: '<S374>/If' */
      if (rtb_Compare > 0) {
        /* Outputs for IfAction SubSystem: '<S374>/If Action Subsystem' incorporates:
         *  ActionPort: '<S378>/Action Port'
         */
        /* S-Function (ec5744_bootloaderslb): '<S378>/BootLoader' */
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

        /* S-Function (ec5744_cpuresetslb): '<S378>/CPUReset' */

        /* Perform a microcontroller reset */
        MC_ME.MCTL.R = 0X00005AF0;
        MC_ME.MCTL.R = 0X0000A50F;

        /* End of Outputs for SubSystem: '<S374>/If Action Subsystem' */
      } else {
        /* Outputs for IfAction SubSystem: '<S374>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S379>/Action Port'
         */
        /* S-Function (ec5744_cantransmitslb): '<S379>/CANTransmit' incorporates:
         *  Constant: '<S379>/Constant'
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

        /* End of Outputs for SubSystem: '<S374>/If Action Subsystem1' */
      }

      /* End of If: '<S374>/If' */
    }

    /* End of Outputs for SubSystem: '<S373>/Enabled Subsystem' */
  }

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S372>/CANReceive' */
  /* End of Outputs for S-Function (fcncallgen): '<S368>/100MS' */

  /* S-Function (fcncallgen): '<S369>/100ms' incorporates:
   *  SubSystem: '<S369>/daq100ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S380>/CCPDAQ' */
  ccpDaq(3);

  /* End of Outputs for S-Function (fcncallgen): '<S369>/100ms' */
}

/* Model step function for TID6 */
void VehCtrlMdel240926_2018b_amkspdlimit_step6(void) /* Sample time: [0.5s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S369>/500ms' incorporates:
   *  SubSystem: '<S369>/daq500ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S382>/CCPDAQ' */
  ccpDaq(4);

  /* End of Outputs for S-Function (fcncallgen): '<S369>/500ms' */

  /* S-Function (fcncallgen): '<S370>/500ms' incorporates:
   *  SubSystem: '<S370>/EEPROMOperation'
   */

  /* S-Function (ec5744_eepromoslb): '<S385>/EEPROMOperatin' */
#if defined EC_EEPROM_ENABLE

  /* Operate the EEPROM module on the MPC5744 */
  ec_flash_operation();

#endif

  /* End of Outputs for S-Function (fcncallgen): '<S370>/500ms' */
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
  /* Start for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive1' incorporates:
   *  SubSystem: '<S122>/MCU_pwr'
   */
  /* Start for function-call system: '<S122>/MCU_pwr' */

  /* Start for Enabled SubSystem: '<S177>/MCU_VCUMeter1' */

  /* Start for S-Function (scanunpack): '<S179>/CAN Unpack' */

  /*-----------S-Function Block: <S179>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S177>/MCU_VCUMeter1' */
  ec_buffer_init(0,1,1,218089455);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' incorporates:
   *  SubSystem: '<S122>/MCU_state'
   */
  /* Start for function-call system: '<S122>/MCU_state' */

  /* Start for Enabled SubSystem: '<S178>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S185>/CAN Unpack' */

  /*-----------S-Function Block: <S185>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S178>/MCU_state' */
  ec_buffer_init(0,0,1,218089199);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S118>/CANReceive3' incorporates:
   *  SubSystem: '<S118>/ABS_BUS_state'
   */
  /* Start for function-call system: '<S118>/ABS_BUS_state' */

  /* Start for Enabled SubSystem: '<S126>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S127>/CAN Unpack1' */

  /*-----------S-Function Block: <S127>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S126>/IMU_state' */
  ec_buffer_init(2,16,0,1698);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S118>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S124>/CANReceive3' incorporates:
   *  SubSystem: '<S124>/StrWhSnis_state'
   */
  /* Start for function-call system: '<S124>/StrWhSnis_state' */

  /* Start for Enabled SubSystem: '<S197>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S198>/CAN Unpack1' */

  /*-----------S-Function Block: <S198>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S197>/IMU_state' */
  ec_buffer_init(2,7,0,330);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S124>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms4' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S132>/CANReceive3' incorporates:
   *  SubSystem: '<S132>/AMKMCU_state'
   */
  /* Start for function-call system: '<S132>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S134>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S137>/CAN Unpack' */

  /*-----------S-Function Block: <S137>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S134>/MCU_state' */
  ec_buffer_init(1,1,0,640);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S132>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S132>/CANReceive1' incorporates:
   *  SubSystem: '<S132>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S132>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S135>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S147>/CAN Unpack' */

  /*-----------S-Function Block: <S147>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S135>/MCU_state' */
  ec_buffer_init(1,2,0,642);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S132>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S132>/CANReceive2' incorporates:
   *  SubSystem: '<S132>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S132>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S136>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S149>/CAN Unpack' */

  /*-----------S-Function Block: <S149>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S136>/MCU_state' */
  ec_buffer_init(1,3,0,644);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S132>/CANReceive2' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive3' incorporates:
   *  SubSystem: '<S133>/AMKMCU_state'
   */
  /* Start for function-call system: '<S133>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S153>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S156>/CAN Unpack' */

  /*-----------S-Function Block: <S156>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S153>/MCU_state' */
  ec_buffer_init(1,4,0,641);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive1' incorporates:
   *  SubSystem: '<S133>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S133>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S154>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S165>/CAN Unpack' */

  /*-----------S-Function Block: <S165>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S154>/MCU_state' */
  ec_buffer_init(1,5,0,643);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive2' incorporates:
   *  SubSystem: '<S133>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S133>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S155>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S167>/CAN Unpack' */

  /*-----------S-Function Block: <S167>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S155>/MCU_state' */
  ec_buffer_init(1,0,0,645);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive2' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms5' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S123>/CANReceive3' incorporates:
   *  SubSystem: '<S123>/IMU_state'
   */
  /* Start for function-call system: '<S123>/IMU_state' */

  /* Start for Enabled SubSystem: '<S192>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S193>/CAN Unpack' */

  /*-----------S-Function Block: <S193>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S192>/MCU_state' */
  ec_buffer_init(2,17,0,513);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S123>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms2' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' */
  ec_buffer_init(0,3,1,408961267);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S353>/CANTransmit' */
  ec_buffer_init(1,8,0,386U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms2' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S354>/CANTransmit' */
  ec_buffer_init(1,9,0,387U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms4' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S355>/CANTransmit' */
  ec_buffer_init(0,8,1,146927393U);

  /* End of Start for S-Function (fcncallgen): '<S5>/50ms3' */

  /* Start for S-Function (fcncallgen): '<S5>/10ms6' incorporates:
   *  SubSystem: '<S5>/WP_OUTPUT'
   */
  /* Start for S-Function (ec5744_pdpslbu3): '<S356>/PowerDriverPWM' incorporates:
   *  Constant: '<S356>/Constant'
   */

  /* Initialize PWM output for channel 6 */
  SIUL2_MSCR42 = 0X02000003;           //PWM_A3

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S368>/100MS' incorporates:
   *  SubSystem: '<S368>/Function-Call Subsystem'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S372>/CANReceive' incorporates:
   *  SubSystem: '<S372>/Function-Call Subsystem'
   */
  /* Start for function-call system: '<S372>/Function-Call Subsystem' */

  /* Start for Enabled SubSystem: '<S373>/Enabled Subsystem' */
  /* Start for IfAction SubSystem: '<S374>/If Action Subsystem1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S379>/CANTransmit' incorporates:
   *  Constant: '<S379>/Constant'
   */
  ec_buffer_init(2,9,0,593U);

  /* End of Start for SubSystem: '<S374>/If Action Subsystem1' */
  /* End of Start for SubSystem: '<S373>/Enabled Subsystem' */
  ec_buffer_init(2,1,0,278);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S372>/CANReceive' */
  /* End of Start for S-Function (fcncallgen): '<S368>/100MS' */

  /* Start for S-Function (fcncallgen): '<S371>/Function-Call Generator' incorporates:
   *  SubSystem: '<S371>/CCPBackground'
   */
  /* Start for S-Function (ec5744_ccpslb): '<S386>/CCPBackground' */
  ccpInit();

  /* End of Start for S-Function (fcncallgen): '<S371>/Function-Call Generator' */

  /* Start for S-Function (ec5744_caninterruptslb1): '<S371>/ReceiveandTransmitInterrupt' incorporates:
   *  SubSystem: '<S371>/CCPReceive'
   */
  /* Start for function-call system: '<S371>/CCPReceive' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S387>/CANReceive' */
  ec_buffer_init(2,0,0,CCP_CRO_ID);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S387>/CANReceive' */
  ec_bufint_init(2,0);
  INTC_0.PSR[548].B.PRIN = 12;
  IntcIsrVectorTable[548] = (uint32_t)&ISR_FlexCAN_2_MB0;

  /* End of Start for S-Function (ec5744_caninterruptslb1): '<S371>/ReceiveandTransmitInterrupt' */

  /* SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  /* InitializeConditions for UnitDelay: '<S282>/Unit Delay4' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn = 0.01F;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' */

  /* SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* SystemInitialize for Chart: '<S108>/Chart2' */
  VehCtrlMdel240926_2018b_amks_DW.sfEvent = -1;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S4>/10ms' incorporates:
   *  SubSystem: '<S4>/Function-Call Subsystem'
   */
  VehCtrlMdel240926_2018b_amks_DW.FunctionCallSubsystem_RESET_ELA = true;

  /* Enable for Chart: '<S215>/Chart' */
  VehCtrlMdel240926_2018b_amks_DW.previousTicks_j =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;

  /* End of Enable for S-Function (fcncallgen): '<S4>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  VehCtrlMdel240926_2018b_amks_DW.Subsystem_RESET_ELAPS_T = true;

  /* End of Enable for S-Function (fcncallgen): '<S4>/10ms1' */

  /* Enable for S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* Enable for Chart: '<S108>/Chart2' */
  VehCtrlMdel240926_2018b_amks_DW.previousTicks_g =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;

  /* End of Enable for S-Function (fcncallgen): '<S2>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S5>/10ms1' incorporates:
   *  SubSystem: '<S5>/Beeper'
   */
  /* Enable for Chart: '<S352>/Chart' */
  VehCtrlMdel240926_2018b_amks_DW.previousTicks_m =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;

  /* End of Enable for S-Function (fcncallgen): '<S5>/10ms1' */

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
