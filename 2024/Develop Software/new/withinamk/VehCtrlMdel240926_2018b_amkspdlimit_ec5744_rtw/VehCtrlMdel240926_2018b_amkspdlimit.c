/*
 * Code generated for Simulink model VehCtrlMdel240926_2018b_amkspdlimit.
 *
 * FILE    : VehCtrlMdel240926_2018b_amkspdlimit.c
 *
 * VERSION : 1.224
 *
 * DATE    : Wed Oct  9 20:01:02 2024
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

/* Named constants for Chart: '<S123>/Timer' */
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

/* Named constants for Chart: '<S106>/Chart2' */
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

/* Named constants for Chart: '<S123>/Chart' */
#define VehCtrlMdel240926_2018b_IN_ON_h ((uint8_T)1U)
#define VehCtrlMdel240926_20_IN_STATEON ((uint8_T)2U)
#define VehCtrlMdel240926_2_IN_STATEOFF ((uint8_T)1U)
#define VehCtrlMdel240926_IN_initstate1 ((uint8_T)2U)
#define VehCtrlMdel240926__IN_initstate ((uint8_T)2U)

/* Named constants for Chart: '<S123>/Timer1' */
#define VehCtrlMdel240926_2018_IN_Out_b ((uint8_T)2U)
#define VehCtrlMdel240926__IN_Trigger_e ((uint8_T)3U)
#define VehCtrlMdel2409_IN_InterState_d ((uint8_T)1U)

boolean L9826VAR701[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

boolean L9826DIAG701[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

boolean L9826VAR702[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

boolean L9826DIAG702[8]= { 0, 0, 0, 0, 0, 0, 0, 0 };

EE_Data ecflashdataold[1024];
EE_Data ecflashdatanew[1024];

/* Exported block signals */
real_T Gear_Trs;                       /* '<S349>/Switch2' */
real_T Mode_Trs;                       /* '<S349>/Switch3' */
real_T AMKFL_Current;                  /* '<S213>/Switch' */
real_T AMKFR_Current;                  /* '<S213>/Switch1' */
real_T Trq_CUT;                        /* '<S210>/Timer' */
real_T AMKSWITCH;                      /* '<S123>/Timer1' */
real_T ignition;                       /* '<S123>/Timer' */
real_T L12V_error;                     /* '<S183>/CAN Unpack' */
real_T alarm;                          /* '<S183>/CAN Unpack' */
real_T controller_ready;               /* '<S183>/CAN Unpack' */
real_T selfcheck;                      /* '<S183>/CAN Unpack' */
real_T RPM;                            /* '<S183>/CAN Unpack' */
real_T trq;                            /* '<S183>/CAN Unpack' */
real_T AC_current;                     /* '<S177>/CAN Unpack' */
real_T DC_current;                     /* '<S177>/CAN Unpack' */
real_T MCU_Temp;                       /* '<S177>/CAN Unpack' */
real_T motor_Temp;                     /* '<S177>/CAN Unpack' */
real_T voltage;                        /* '<S177>/CAN Unpack' */
real_T MCFR_ActualTorque;              /* '<S154>/CAN Unpack' */
real_T MCFR_ActualVelocity;            /* '<S154>/CAN Unpack' */
real_T MCFR_DCVoltage;                 /* '<S154>/CAN Unpack' */
real_T MCFR_bDCOn;                     /* '<S154>/CAN Unpack' */
real_T MCFR_bError;                    /* '<S154>/CAN Unpack' */
real_T MCFR_bInverterOn;               /* '<S154>/CAN Unpack' */
real_T MCFR_bQuitInverterOn;           /* '<S154>/CAN Unpack' */
real_T MCFR_bSystemReady;              /* '<S154>/CAN Unpack' */
real_T MCFR_TempIGBT;                  /* '<S165>/CAN Unpack' */
real_T MCFR_TempInverter;              /* '<S165>/CAN Unpack' */
real_T MCFR_TempMotor;                 /* '<S165>/CAN Unpack' */
real_T MCFR_ErrorInfo;                 /* '<S163>/CAN Unpack' */
real_T MCFL_ActualTorque;              /* '<S135>/CAN Unpack' */
real_T MCFL_DCVoltage;                 /* '<S135>/CAN Unpack' */
real_T MCFL_bDCOn;                     /* '<S135>/CAN Unpack' */
real_T MCFL_bError;                    /* '<S135>/CAN Unpack' */
real_T MCFL_bInverterOn;               /* '<S135>/CAN Unpack' */
real_T MCFL_bQuitDCOn;                 /* '<S135>/CAN Unpack' */
real_T MCFL_bQuitInverterOn;           /* '<S135>/CAN Unpack' */
real_T MCFL_bSystemReady;              /* '<S135>/CAN Unpack' */
real_T MCFL_ActualVelocity;            /* '<S135>/Gain' */
real_T MCFL_TempIGBT;                  /* '<S147>/CAN Unpack' */
real_T MCFL_TempInverter;              /* '<S147>/CAN Unpack' */
real_T MCFL_TempMotor;                 /* '<S147>/CAN Unpack' */
real_T MCFL_ErrorInfo;                 /* '<S145>/CAN Unpack' */
real_T StrWhlAngAliveRollCnt;          /* '<S196>/CAN Unpack1' */
real_T StrWhlAng;                      /* '<S196>/CAN Unpack1' */
real_T StrWhlAngV;                     /* '<S196>/CAN Unpack1' */
real_T ABS_WS_FL;                      /* '<S125>/CAN Unpack1' */
real_T ABS_WS_FR;                      /* '<S125>/CAN Unpack1' */
real_T ABS_WS_RL;                      /* '<S125>/CAN Unpack1' */
real_T ABS_WS_RR;                      /* '<S125>/CAN Unpack1' */
real_T IMU_Ay_Value;                   /* '<S191>/CAN Unpack' */
real_T IMU_Ax_Value;                   /* '<S191>/CAN Unpack' */
real_T IMU_Yaw_Value;                  /* '<S191>/CAN Unpack' */
real_T EMRAX_Trq_CUT;                  /*  */
real_T AMK_Trq_CUT;                    /*  */
uint32_T Acc_vol2;                     /* '<S210>/Add3' */
uint32_T Acc_vol;                      /* '<S210>/Add2' */
uint32_T Acc_POS2;                     /* '<S210>/1-D Lookup Table3' */
real32_T VehVxEst_mps;                 /* '<S333>/Add' */
real32_T Acc_POS;                      /* '<S210>/MATLAB Function' */
real32_T AMKTrqFR_cmd;                 /* '<S7>/Saturation2' */
real32_T AMKTrqFL_cmd;                 /* '<S7>/Saturation3' */
real32_T EmraxTrqR_cmd;                /* '<S7>/Saturation1' */
uint16_T F_BrkPrs;                     /* '<S210>/1-D Lookup Table1' */
uint16_T Acc1;                         /* '<S118>/Acc3' */
uint16_T Acc2;                         /* '<S118>/Acc4' */
uint16_T Brk1;                         /* '<S118>/Brk1' */
uint16_T Brk2;                         /* '<S118>/Brk2' */
boolean_T VCU2BMS;     /* '<S208>/BusConversion_InsertedFor_Out1_at_inport_0' */
boolean_T KeyPressed;                  /* '<S106>/Cast To Boolean' */
boolean_T Brk;                         /* '<S108>/Compare' */
boolean_T ACC_Release;                 /* '<S109>/Compare' */
boolean_T beeper_state;                /* '<S106>/Chart2' */
boolean_T MCFL_DCOn_setpoints;         /* '<S106>/Chart2' */
boolean_T MCFR_DCEnable;               /* '<S106>/Chart2' */
boolean_T MCFR_InverterOn;             /* '<S106>/Chart2' */
boolean_T TroqueOn;                    /* '<S7>/Logical Operator6' */
boolean_T TrqR_cmd_raw;                /* '<S7>/Logical Operator1' */
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
  /* Call the system: <S359>/CCPReceive */
  {
    /* S-Function (ec5744_caninterruptslb1): '<S359>/ReceiveandTransmitInterrupt' */

    /* Output and update for function-call system: '<S359>/CCPReceive' */

    /* S-Function (ec5744_canreceiveslb): '<S375>/CANReceive' */

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

    /* Nothing to do for system: <S375>/Nothing */

    /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S375>/CANReceive' */

    /* End of Outputs for S-Function (ec5744_caninterruptslb1): '<S359>/ReceiveandTransmitInterrupt' */
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
 *    '<S123>/Timer2'
 *    '<S212>/Timer'
 *    '<S214>/Timer'
 *    '<S214>/Timer1'
 *    '<S214>/Timer2'
 *    '<S214>/Timer3'
 *    '<S275>/Timer'
 *    '<S275>/Timer1'
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
 *    '<S123>/Timer'
 *    '<S210>/Timer'
 */
void VehCtrlMdel240926_201_Timer(boolean_T rtu_Trigger, real32_T rtu_CountTime,
  real_T *rty_Exit, DW_Timer_VehCtrlMdel240926_20_T *localDW)
{
  boolean_T sf_internal_predicateOutput;

  /* Chart: '<S123>/Timer' */
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

  /* End of Chart: '<S123>/Timer' */
}

/* Function for Chart: '<S106>/Chart2' */
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

/* Function for Chart: '<S106>/Chart2' */
static void VehCtrlMdel240926_2018b_VehStat(const real_T *controller_ready_e,
  const real_T *Switch_k, const real_T *Switch3, const real_T *Switch10, const
  real_T *Switch11)
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
      sf_internal_predicateOutput = (KeyPressed && Brk && ACC_Release &&
        (*controller_ready_e != 0.0) && (*Switch_k != 0.0) && (*Switch3 != 0.0) &&
        (*Switch10 != 0.0) && (*Switch11 != 0.0));
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

/* Function for Chart: '<S106>/Chart2' */
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
  /* S-Function (fcncallgen): '<S359>/Function-Call Generator' incorporates:
   *  SubSystem: '<S359>/CCPBackground'
   */

  /* S-Function (ec5744_ccpslb): '<S374>/CCPBackground' */
  ccpBackground();
  Lin0_Background();

  /* End of Outputs for S-Function (fcncallgen): '<S359>/Function-Call Generator' */
}

/* Model step function for TID2 */
void VehCtrlMdel240926_2018b_amkspdlimit_step2(void) /* Sample time: [0.005s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S357>/5ms' incorporates:
   *  SubSystem: '<S357>/daq5ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S372>/CCPDAQ' */
  ccpDaq(0);

  /* End of Outputs for S-Function (fcncallgen): '<S357>/5ms' */
}

/* Model step function for TID3 */
void VehCtrlMdel240926_2018b_amkspdlimit_step3(void) /* Sample time: [0.01s, 0.0s] */
{
  boolean_T rtb_ignition;
  boolean_T rtb_FixPtRelationalOperator;
  real32_T rtb_StrWhlAngV_c;
  real_T rtb_Gain5;
  real_T rtb_Gain4;
  real_T rtb_Switch2_on;
  real32_T rtb_Add;
  boolean_T rtb_Compare;
  real32_T rtb_APP_POS1;
  real32_T rtb_Gain3_o;
  boolean_T rtb_LogicalOperator7;
  boolean_T rtb_Compare_am;
  boolean_T rtb_LowerRelop1_b;
  real_T elapseTime;
  real_T rtb_Yk1_l;
  real_T rtb_UkYk1;
  real_T rtb_g_mpss1;
  real32_T rtb_Add4_j;
  real32_T rtb_Add7;
  real32_T rtb_Add6;
  real32_T rtb_Switch2_mn;
  real32_T rtb_Switch2_b0;
  real32_T rtb_CastToDouble;
  boolean_T rtb_Compare_b;
  real32_T rtb_VxIMU_est;
  real32_T rtb_Ax;
  real_T rtb_Gain19;
  real_T rtb_Yk1_h;
  real_T rtb_Switch2_gd;
  real_T rtb_Switch2_bp;
  real_T elapseTime_0;
  real32_T rtb_Add10;
  real32_T rtb_deltafalllimit_iz;
  real_T rtb_deltafalllimit_le;
  real_T rtb_deltafalllimit_pe;
  real32_T rtb_deltafalllimit_a;
  real32_T rtb_deltafalllimit_o4;
  boolean_T rtb_UpperRelop_ir;
  real_T rtb_deltafalllimit_m5;
  real32_T rtb_MaxWhlSpd_mps_n;
  uint32_T rtb_Gain1;
  uint32_T rtb_Gain;
  int32_T Brk_F;
  real_T WhlSpdFR;
  real_T WhlSpdRR_mps;
  real_T WhlSpdRL_mps;
  real32_T FLWhlStrAng;
  real32_T Acc_POS_n;
  uint32_T elapsedTicks;
  boolean_T rtb_LogicalOperator_idx_0;

  /* S-Function (fcncallgen): '<S3>/10ms7' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* S-Function (ec5744_swislbu3): '<S123>/SwitchInput3' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240926_2018b_amksp_B.VCU2BMS_o= ec_gpio_read(92);

  /* Logic: '<S123>/Logical Operator2' */
  rtb_ignition = !VehCtrlMdel240926_2018b_amksp_B.VCU2BMS_o;

  /* Chart: '<S123>/Timer2' incorporates:
   *  Constant: '<S123>/Constant3'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 1.0F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_hg,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_h);

  /* RelationalOperator: '<S207>/Compare' incorporates:
   *  Constant: '<S207>/Constant'
   */
  rtb_ignition = (VehCtrlMdel240926_2018b_amksp_B.Exit_hg > 0.0);

  /* RelationalOperator: '<S201>/FixPt Relational Operator' incorporates:
   *  UnitDelay: '<S201>/Delay Input1'
   *
   * Block description for '<S201>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_FixPtRelationalOperator = ((int32_T)rtb_ignition > (int32_T)
    VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE);

  /* Chart: '<S123>/Chart' */
  elapsedTicks = VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3 -
    VehCtrlMdel240926_2018b_amks_DW.previousTicks;
  VehCtrlMdel240926_2018b_amks_DW.previousTicks =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
  if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 + elapsedTicks <= 31U)
  {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = (uint8_T)
      (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 + elapsedTicks);
  } else {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 31U;
  }

  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c24_VehCtrlMdel240926
      == 0U) {
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c24_VehCtrlMdel240926 =
      1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c24_VehCtrlMdel240926_2018b_ =
      2U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON = 2U;
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
    VehCtrlMdel240926_2018b_amksp_B.VCUEnable = false;
  } else if
      (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c24_VehCtrlMdel240926_2018b_
       == VehCtrlMdel240926_2_IN_STATEOFF) {
    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEOFF ==
        VehCtrlMdel240926_2018b__IN_OFF) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEOFF = 0U;
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c24_VehCtrlMdel240926_2018b_
        = 2U;
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON = 2U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
      VehCtrlMdel240926_2018b_amksp_B.VCUEnable = false;
    } else {
      /* case IN_initstate1: */
      VehCtrlMdel240926_2018b_amksp_B.VCUEnable = true;
      rtb_LogicalOperator7 =
        ((VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 >= 20U) &&
         rtb_FixPtRelationalOperator);
      if (rtb_LogicalOperator7) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEOFF = 1U;
        VehCtrlMdel240926_2018b_amksp_B.VCUEnable = false;
      }
    }
  } else {
    /* case IN_STATEON: */
    if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON ==
        VehCtrlMdel240926_2018b_IN_ON_h) {
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON = 0U;
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c24_VehCtrlMdel240926_2018b_
        = 1U;
      VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEOFF = 2U;
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
      VehCtrlMdel240926_2018b_amksp_B.VCUEnable = true;
    } else {
      /* case IN_initstate: */
      VehCtrlMdel240926_2018b_amksp_B.VCUEnable = false;
      rtb_FixPtRelationalOperator =
        ((VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 >= 20U) &&
         rtb_FixPtRelationalOperator);
      if (rtb_FixPtRelationalOperator) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON = 1U;
        VehCtrlMdel240926_2018b_amksp_B.VCUEnable = true;
      }
    }
  }

  /* End of Chart: '<S123>/Chart' */

  /* S-Function (ec5744_swislbu3): '<S123>/SwitchInput' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240926_2018b_amksp_B.Drive_ready= ec_gpio_read(99);

  /* Logic: '<S123>/Logical Operator' */
  rtb_FixPtRelationalOperator = !VehCtrlMdel240926_2018b_amksp_B.Drive_ready;

  /* Chart: '<S123>/Timer' incorporates:
   *  Constant: '<S123>/Constant5'
   */
  VehCtrlMdel240926_201_Timer(rtb_FixPtRelationalOperator, 0.11F, &ignition,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer);

  /* Chart: '<S123>/Timer1' incorporates:
   *  Constant: '<S123>/Constant1'
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
      rtb_LogicalOperator7 = (VehCtrlMdel240926_2018b_amks_DW.x <
        0.10999999940395355);
      if (rtb_LogicalOperator7) {
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

  /* End of Chart: '<S123>/Timer1' */

  /* Update for UnitDelay: '<S201>/Delay Input1'
   *
   * Block description for '<S201>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE = rtb_ignition;

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms7' */

  /* S-Function (fcncallgen): '<S3>/10ms6' incorporates:
   *  SubSystem: '<S3>/EMRAXMCU_RECIEVE'
   */
  /* S-Function (ec5744_canreceiveslb): '<S120>/CANReceive1' */

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

  /* Call the system: <S120>/MCU_pwr */

  /* Output and update for function-call system: '<S120>/MCU_pwr' */

  /* Outputs for Enabled SubSystem: '<S175>/MCU_VCUMeter1' incorporates:
   *  EnablePort: '<S177>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S177>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S177>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S177>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S175>/MCU_VCUMeter1' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S120>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S120>/CANReceive3' */

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

  /* Call the system: <S120>/MCU_state */

  /* Output and update for function-call system: '<S120>/MCU_state' */

  /* Outputs for Enabled SubSystem: '<S176>/MCU_state' incorporates:
   *  EnablePort: '<S183>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S183>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S183>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S183>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S176>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S120>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms6' */

  /* S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S116>/CANReceive3' */

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

  /* Call the system: <S116>/ABS_BUS_state */

  /* Output and update for function-call system: '<S116>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S124>/IMU_state' incorporates:
   *  EnablePort: '<S125>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_m > 0) {
    /* S-Function (ecucoder_canunmessage): '<S125>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S125>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S125>/CAN Unpack1' */
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

  /* End of Outputs for SubSystem: '<S124>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S116>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms3' */

  /* S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' */

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

  /* Call the system: <S122>/StrWhSnis_state */

  /* Output and update for function-call system: '<S122>/StrWhSnis_state' */

  /* Outputs for Enabled SubSystem: '<S195>/IMU_state' incorporates:
   *  EnablePort: '<S196>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S196>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S196>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S196>/CAN Unpack1' */
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

  /* End of Outputs for SubSystem: '<S195>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms4' */

  /* S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S130>/CANReceive3' */

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

  /* Call the system: <S130>/AMKMCU_state */

  /* Output and update for function-call system: '<S130>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S132>/MCU_state' incorporates:
   *  EnablePort: '<S135>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S135>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S135>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S135>/CAN Unpack' */
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

    /* Gain: '<S135>/Gain' */
    MCFL_ActualVelocity = -VehCtrlMdel240926_2018b_amksp_B.MCFL_ActualVelocity_p;
  }

  /* End of Outputs for SubSystem: '<S132>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S130>/CANReceive1' */

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

  /* Call the system: <S130>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S130>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S133>/MCU_state' incorporates:
   *  EnablePort: '<S145>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S145>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S145>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S145>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S133>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S130>/CANReceive2' */

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

  /* Call the system: <S130>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S130>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S134>/MCU_state' incorporates:
   *  EnablePort: '<S147>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S147>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S147>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S147>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S134>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive2' */

  /* S-Function (ec5744_canreceiveslb): '<S131>/CANReceive3' */

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

  /* Call the system: <S131>/AMKMCU_state */

  /* Output and update for function-call system: '<S131>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S151>/MCU_state' incorporates:
   *  EnablePort: '<S154>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_a > 0) {
    /* S-Function (ecucoder_canunmessage): '<S154>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S154>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S154>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S151>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S131>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S131>/CANReceive1' */

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

  /* Call the system: <S131>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S131>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S152>/MCU_state' incorporates:
   *  EnablePort: '<S163>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2_o > 0) {
    /* S-Function (ecucoder_canunmessage): '<S163>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S163>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S163>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S152>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S131>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S131>/CANReceive2' */

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

  /* Call the system: <S131>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S131>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S153>/MCU_state' incorporates:
   *  EnablePort: '<S165>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S165>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S165>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S165>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S153>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S131>/CANReceive2' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms5' */

  /* S-Function (fcncallgen): '<S3>/10ms' incorporates:
   *  SubSystem: '<S3>/AccBrk_BUS'
   */
  /* S-Function (ec5744_asislbu3): '<S118>/Acc3' */

  /* Read the ADC conversion result of the analog signal */
  Acc1= adc_read_chan(1,2);

  /* S-Function (ec5744_asislbu3): '<S118>/Acc4' */

  /* Read the ADC conversion result of the analog signal */
  Acc2= adc_read_chan(1,4);

  /* S-Function (ec5744_asislbu3): '<S118>/Brk1' */

  /* Read the ADC conversion result of the analog signal */
  Brk1= adc_read_chan(1,0);

  /* S-Function (ec5744_asislbu3): '<S118>/Brk2' */

  /* Read the ADC conversion result of the analog signal */
  Brk2= adc_read_chan(0,13);

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms' */

  /* S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' */

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

  /* Call the system: <S121>/IMU_state */

  /* Output and update for function-call system: '<S121>/IMU_state' */

  /* Outputs for Enabled SubSystem: '<S190>/MCU_state' incorporates:
   *  EnablePort: '<S191>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_ma > 0) {
    /* S-Function (ecucoder_canunmessage): '<S191>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S191>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S191>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S190>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms2' */

  /* S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S119>/CANReceive3' */

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

  /* Call the system: <S119>/ABS_BUS_state */

  /* Output and update for function-call system: '<S119>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S173>/IMU_state' incorporates:
   *  EnablePort: '<S174>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_k > 0) {
    /* S-Function (ecucoder_canunmessage): '<S174>/CANUnPackMessage4' */

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

  /* End of Outputs for SubSystem: '<S173>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S119>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms1' */

  /* S-Function (fcncallgen): '<S4>/10ms' incorporates:
   *  SubSystem: '<S4>/Function-Call Subsystem'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.FunctionCallSubsystem_RESET_ELA) {
    elapsedTicks = 0U;
  } else {
    elapsedTicks = VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3 -
      VehCtrlMdel240926_2018b_amks_DW.FunctionCallSubsystem_PREV_T;
  }

  VehCtrlMdel240926_2018b_amks_DW.FunctionCallSubsystem_PREV_T =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
  VehCtrlMdel240926_2018b_amks_DW.FunctionCallSubsystem_RESET_ELA = false;

  /* Lookup_n-D: '<S210>/1-D Lookup Table1' */
  F_BrkPrs = look1_iu16bflftfIu16_binlc(Brk1,
    VehCtrlMdel240926_2018b__ConstP.pooled65,
    VehCtrlMdel240926_2018b__ConstP.pooled65, 1U);

  /* DataTypeConversion: '<S210>/Data Type Conversion' */
  rtb_Add = F_BrkPrs;

  /* SignalConversion generated from: '<S208>/Out1' */
  Brk_F = (int32_T)rtb_Add;

  /* Gain: '<S210>/Gain2' */
  rtb_Gain1 = 45875U * Acc2;

  /* Gain: '<S210>/Gain3' incorporates:
   *  UnitDelay: '<S210>/Unit Delay1'
   */
  rtb_Gain = 39322U * VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S210>/Add3' */
  Acc_vol2 = (rtb_Gain >> 1) + rtb_Gain1;

  /* RelationalOperator: '<S219>/Compare' */
  rtb_ignition = (Acc_vol2 <= 32768000U);

  /* RelationalOperator: '<S220>/Compare' */
  rtb_FixPtRelationalOperator = (Acc_vol2 >= 294912000U);

  /* Logic: '<S210>/Logical Operator1' */
  rtb_ignition = (rtb_ignition || rtb_FixPtRelationalOperator);

  /* Gain: '<S210>/Gain' */
  rtb_Gain = 45875U * Acc1;

  /* UnitDelay: '<S210>/Unit Delay' incorporates:
   *  UnitDelay: '<S210>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_k;

  /* Gain: '<S210>/Gain1' incorporates:
   *  UnitDelay: '<S210>/Unit Delay1'
   */
  rtb_Gain1 = 39322U * VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S210>/Add2' */
  Acc_vol = (rtb_Gain1 >> 1) + rtb_Gain;

  /* RelationalOperator: '<S215>/Compare' */
  rtb_FixPtRelationalOperator = (Acc_vol <= 32768000U);

  /* RelationalOperator: '<S216>/Compare' */
  rtb_Compare = (Acc_vol >= 294912000U);

  /* Logic: '<S210>/Logical Operator' */
  rtb_FixPtRelationalOperator = (rtb_FixPtRelationalOperator || rtb_Compare);

  /* Logic: '<S210>/Logical Operator2' */
  rtb_FixPtRelationalOperator = (rtb_FixPtRelationalOperator || rtb_ignition);

  /* DataTypeConversion: '<S210>/Data Type Conversion2' */
  rtb_APP_POS1 = (real32_T)Acc_vol * 1.52587891E-5F;

  /* MATLAB Function: '<S210>/MATLAB Function' */
  Acc_POS = fmaxf(fminf((rtb_APP_POS1 - 2300.0F) * -0.4F, 100.0F), 0.0F);

  /* Lookup_n-D: '<S210>/1-D Lookup Table3' */
  Acc_POS2 = look1_iu32n16bflftfIu32_binlc(Acc_vol2,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable3_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable3_tableData, 1U);

  /* DataTypeConversion: '<S210>/Data Type Conversion4' */
  rtb_Add = (real32_T)Acc_POS2 * 1.52587891E-5F;

  /* Sum: '<S210>/Add1' */
  rtb_Gain3_o = Acc_POS - rtb_Add;

  /* Abs: '<S210>/Abs' */
  rtb_Gain3_o = fabsf(rtb_Gain3_o);

  /* RelationalOperator: '<S223>/Compare' incorporates:
   *  Constant: '<S223>/Constant'
   */
  rtb_Compare = (rtb_Gain3_o > 15.0F);

  /* RelationalOperator: '<S221>/Compare' incorporates:
   *  Constant: '<S221>/Constant'
   */
  rtb_ignition = (Acc_POS > 100.0F);

  /* RelationalOperator: '<S222>/Compare' incorporates:
   *  Constant: '<S222>/Constant'
   */
  rtb_LogicalOperator7 = (rtb_Add > 100.0F);

  /* Logic: '<S210>/Logical Operator3' */
  rtb_ignition = (rtb_ignition || rtb_LogicalOperator7);

  /* RelationalOperator: '<S224>/Compare' incorporates:
   *  Constant: '<S224>/Constant'
   */
  rtb_LogicalOperator7 = (Brk1 <= 300);

  /* RelationalOperator: '<S225>/Compare' incorporates:
   *  Constant: '<S225>/Constant'
   */
  rtb_Compare_am = (Brk1 >= 4500);

  /* Logic: '<S210>/Logical Operator5' */
  rtb_LogicalOperator7 = (rtb_LogicalOperator7 || rtb_Compare_am);

  /* RelationalOperator: '<S217>/Compare' incorporates:
   *  Constant: '<S217>/Constant'
   */
  rtb_Compare_am = (Brk2 <= 300);

  /* RelationalOperator: '<S218>/Compare' incorporates:
   *  Constant: '<S218>/Constant'
   */
  rtb_LowerRelop1_b = (Brk2 >= 4500);

  /* Logic: '<S210>/Logical Operator6' */
  rtb_Compare_am = (rtb_Compare_am || rtb_LowerRelop1_b);

  /* Logic: '<S210>/Logical Operator7' */
  rtb_LogicalOperator7 = (rtb_LogicalOperator7 || rtb_Compare_am);

  /* Logic: '<S210>/Logical Operator4' */
  rtb_ignition = (rtb_ignition || rtb_Compare || rtb_FixPtRelationalOperator ||
                  rtb_LogicalOperator7);

  /* Chart: '<S210>/Timer' incorporates:
   *  Constant: '<S210>/Constant1'
   */
  VehCtrlMdel240926_201_Timer(rtb_ignition, 0.11F, &Trq_CUT,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_a);

  /* UnitDelay: '<S246>/Delay Input2'
   *
   * Block description for '<S246>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain3_o = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2;

  /* SampleTimeMath: '<S246>/sample time'
   *
   * About '<S246>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S246>/delta rise limit' */
  rtb_StrWhlAngV_c = (real32_T)(1200.0 * elapseTime);

  /* DataTypeConversion: '<S212>/Cast To Boolean' */
  rtb_APP_POS1 = (real32_T)StrWhlAng;

  /* Sum: '<S246>/Difference Inputs1'
   *
   * Block description for '<S246>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_APP_POS1 -= rtb_Gain3_o;

  /* RelationalOperator: '<S249>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_APP_POS1 > rtb_StrWhlAngV_c);

  /* Switch: '<S249>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S246>/delta fall limit' */
    rtb_deltafalllimit_iz = (real32_T)(-1200.0 * elapseTime);

    /* RelationalOperator: '<S249>/UpperRelop' */
    rtb_ignition = (rtb_APP_POS1 < rtb_deltafalllimit_iz);

    /* Switch: '<S249>/Switch' */
    if (rtb_ignition) {
      rtb_APP_POS1 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S249>/Switch' */
    rtb_StrWhlAngV_c = rtb_APP_POS1;
  }

  /* End of Switch: '<S249>/Switch2' */

  /* Sum: '<S246>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S246>/Delay Input2'
   *
   * Block description for '<S246>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S246>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2 = rtb_StrWhlAngV_c +
    rtb_Gain3_o;

  /* Abs: '<S212>/Abs' incorporates:
   *  UnitDelay: '<S246>/Delay Input2'
   *
   * Block description for '<S246>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_StrWhlAngV_c = fabsf(VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2);

  /* RelationalOperator: '<S245>/Compare' incorporates:
   *  Constant: '<S245>/Constant'
   */
  rtb_ignition = (rtb_StrWhlAngV_c > 120.0F);

  /* Chart: '<S212>/Timer' incorporates:
   *  Constant: '<S212>/Constant5'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_on,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_k);

  /* UnitDelay: '<S261>/Delay Input2'
   *
   * Block description for '<S261>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE;

  /* SampleTimeMath: '<S261>/sample time'
   *
   * About '<S261>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S261>/delta rise limit' */
  rtb_Gain5 = 10.0 * elapseTime;

  /* Sum: '<S261>/Difference Inputs1'
   *
   * Block description for '<S261>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RL - rtb_Yk1_l;

  /* RelationalOperator: '<S269>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain5);

  /* Switch: '<S269>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S261>/delta fall limit' */
    rtb_deltafalllimit_le = -10.0 * elapseTime;

    /* RelationalOperator: '<S269>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_le);

    /* Switch: '<S269>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_le;
    }

    /* End of Switch: '<S269>/Switch' */
    rtb_Gain5 = rtb_UkYk1;
  }

  /* End of Switch: '<S269>/Switch2' */

  /* Saturate: '<S214>/Saturation' incorporates:
   *  Sum: '<S261>/Difference Inputs2'
   *  UnitDelay: '<S261>/Delay Input2'
   *
   * Block description for '<S261>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S261>/Delay Input2':
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

  /* End of Saturate: '<S214>/Saturation' */

  /* Gain: '<S214>/Gain' */
  rtb_Gain5 *= 0.27777777777777779;

  /* RelationalOperator: '<S253>/Compare' incorporates:
   *  Constant: '<S253>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain5 >= 0.0);

  /* RelationalOperator: '<S254>/Compare' incorporates:
   *  Constant: '<S254>/Constant'
   */
  rtb_Compare_am = (rtb_Gain5 < 40.0);

  /* Logic: '<S214>/OR' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S214>/Timer' incorporates:
   *  Constant: '<S214>/Constant5'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_le,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_b);

  /* UnitDelay: '<S262>/Delay Input2'
   *
   * Block description for '<S262>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b;

  /* SampleTimeMath: '<S262>/sample time'
   *
   * About '<S262>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S262>/delta rise limit' */
  rtb_Gain4 = 10.0 * elapseTime;

  /* Sum: '<S262>/Difference Inputs1'
   *
   * Block description for '<S262>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RR - rtb_Yk1_l;

  /* RelationalOperator: '<S270>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain4);

  /* Switch: '<S270>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S262>/delta fall limit' */
    rtb_deltafalllimit_le = -10.0 * elapseTime;

    /* RelationalOperator: '<S270>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_le);

    /* Switch: '<S270>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_le;
    }

    /* End of Switch: '<S270>/Switch' */
    rtb_Gain4 = rtb_UkYk1;
  }

  /* End of Switch: '<S270>/Switch2' */

  /* Saturate: '<S214>/Saturation1' incorporates:
   *  Sum: '<S262>/Difference Inputs2'
   *  UnitDelay: '<S262>/Delay Input2'
   *
   * Block description for '<S262>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S262>/Delay Input2':
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

  /* End of Saturate: '<S214>/Saturation1' */

  /* Gain: '<S214>/Gain3' */
  rtb_Gain4 *= 0.27777777777777779;

  /* RelationalOperator: '<S255>/Compare' incorporates:
   *  Constant: '<S255>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain4 >= 0.0);

  /* RelationalOperator: '<S256>/Compare' incorporates:
   *  Constant: '<S256>/Constant'
   */
  rtb_Compare_am = (rtb_Gain4 < 40.0);

  /* Logic: '<S214>/OR1' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S214>/Timer1' incorporates:
   *  Constant: '<S214>/Constant1'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_i,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_n);

  /* UnitDelay: '<S263>/Delay Input2'
   *
   * Block description for '<S263>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h;

  /* SampleTimeMath: '<S263>/sample time'
   *
   * About '<S263>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S263>/delta rise limit' */
  rtb_Switch2_on = 10.0 * elapseTime;

  /* Sum: '<S263>/Difference Inputs1'
   *
   * Block description for '<S263>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = MCFR_ActualVelocity - rtb_Yk1_l;

  /* RelationalOperator: '<S271>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Switch2_on);

  /* Switch: '<S271>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S263>/delta fall limit' */
    rtb_deltafalllimit_le = -10.0 * elapseTime;

    /* RelationalOperator: '<S271>/UpperRelop' */
    rtb_ignition = (rtb_UkYk1 < rtb_deltafalllimit_le);

    /* Switch: '<S271>/Switch' */
    if (rtb_ignition) {
      rtb_UkYk1 = rtb_deltafalllimit_le;
    }

    /* End of Switch: '<S271>/Switch' */
    rtb_Switch2_on = rtb_UkYk1;
  }

  /* End of Switch: '<S271>/Switch2' */

  /* Saturate: '<S214>/Saturation2' incorporates:
   *  Sum: '<S263>/Difference Inputs2'
   *  UnitDelay: '<S263>/Delay Input2'
   *
   * Block description for '<S263>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S263>/Delay Input2':
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

  /* End of Saturate: '<S214>/Saturation2' */

  /* Gain: '<S214>/Gain1' */
  rtb_Switch2_on *= 0.1341030088495575;

  /* RelationalOperator: '<S257>/Compare' incorporates:
   *  Constant: '<S257>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Switch2_on >= 0.0);

  /* RelationalOperator: '<S258>/Compare' incorporates:
   *  Constant: '<S258>/Constant'
   */
  rtb_Compare_am = (rtb_Switch2_on < 40.0);

  /* Logic: '<S214>/OR2' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S214>/Timer2' incorporates:
   *  Constant: '<S214>/Constant4'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_o,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_l);

  /* UnitDelay: '<S264>/Delay Input2'
   *
   * Block description for '<S264>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n;

  /* SampleTimeMath: '<S264>/sample time'
   *
   * About '<S264>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S264>/delta rise limit' */
  rtb_UkYk1 = 10.0 * elapseTime;

  /* Sum: '<S264>/Difference Inputs1'
   *
   * Block description for '<S264>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_le = MCFL_ActualVelocity - rtb_Yk1_l;

  /* RelationalOperator: '<S272>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_le > rtb_UkYk1);

  /* Switch: '<S272>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S264>/delta fall limit' */
    rtb_UkYk1 = -10.0 * elapseTime;

    /* RelationalOperator: '<S272>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_le < rtb_UkYk1);

    /* Switch: '<S272>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_le = rtb_UkYk1;
    }

    /* End of Switch: '<S272>/Switch' */
    rtb_UkYk1 = rtb_deltafalllimit_le;
  }

  /* End of Switch: '<S272>/Switch2' */

  /* Saturate: '<S214>/Saturation3' incorporates:
   *  Sum: '<S264>/Difference Inputs2'
   *  UnitDelay: '<S264>/Delay Input2'
   *
   * Block description for '<S264>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S264>/Delay Input2':
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

  /* End of Saturate: '<S214>/Saturation3' */

  /* Gain: '<S214>/Gain2' */
  rtb_UkYk1 *= 0.1341030088495575;

  /* RelationalOperator: '<S259>/Compare' incorporates:
   *  Constant: '<S259>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_UkYk1 >= 0.0);

  /* RelationalOperator: '<S260>/Compare' incorporates:
   *  Constant: '<S260>/Constant'
   */
  rtb_Compare_am = (rtb_UkYk1 < 40.0);

  /* Logic: '<S214>/OR3' */
  rtb_ignition = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S214>/Timer3' incorporates:
   *  Constant: '<S214>/Constant8'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_h,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer3);

  /* SignalConversion generated from: '<S208>/Out1' */
  rtb_deltafalllimit_le = rtb_UkYk1;

  /* SignalConversion generated from: '<S208>/Out1' */
  WhlSpdFR = rtb_Switch2_on;

  /* SignalConversion generated from: '<S208>/Out1' */
  WhlSpdRR_mps = rtb_Gain4;

  /* SignalConversion generated from: '<S208>/Out1' */
  WhlSpdRL_mps = rtb_Gain5;

  /* Gain: '<S212>/Gain' incorporates:
   *  UnitDelay: '<S246>/Delay Input2'
   *
   * Block description for '<S246>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_StrWhlAngV_c = 0.7F *
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2;

  /* UnitDelay: '<S212>/Unit Delay' */
  rtb_Gain3_o = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o2;

  /* Gain: '<S212>/Gain1' */
  rtb_Gain3_o *= 0.3F;

  /* Sum: '<S212>/Add2' */
  rtb_StrWhlAngV_c += rtb_Gain3_o;

  /* Lookup_n-D: '<S212>/1-D Lookup Table' */
  rtb_Gain3_o = look1_iflf_binlx(rtb_StrWhlAngV_c,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable_tableData, 23U);

  /* SignalConversion generated from: '<S208>/Out1' */
  FLWhlStrAng = rtb_Gain3_o;

  /* Lookup_n-D: '<S212>/1-D Lookup Table1' */
  rtb_Gain3_o = look1_iflf_binlx(rtb_StrWhlAngV_c,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp01Data_h,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_tableData_b, 23U);

  /* SignalConversion generated from: '<S208>/Out1' */
  rtb_deltafalllimit_iz = rtb_Gain3_o;

  /* SignalConversion generated from: '<S208>/Out1' */
  rtb_APP_POS1 = rtb_StrWhlAngV_c;

  /* Sum: '<S210>/Add' */
  rtb_Add += Acc_POS;

  /* Product: '<S210>/Divide' incorporates:
   *  Constant: '<S210>/Constant'
   */
  rtb_StrWhlAngV_c = (real32_T)(rtb_Add / 2.0);

  /* SignalConversion generated from: '<S208>/Out1' */
  Acc_POS_n = rtb_StrWhlAngV_c;

  /* UnitDelay: '<S236>/Delay Input2'
   *
   * Block description for '<S236>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l;

  /* SampleTimeMath: '<S236>/sample time'
   *
   * About '<S236>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S236>/delta rise limit' incorporates:
   *  Constant: '<S235>/Constant'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* UnitDelay: '<S235>/Unit Delay' */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE;

  /* Gain: '<S235>/Gain1' */
  rtb_Gain4 *= 0.3;

  /* Gain: '<S211>/g_mpss' incorporates:
   *  UnitDelay: '<S235>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE = 9.8 * IMU_Ay_Value;

  /* Gain: '<S235>/Gain' incorporates:
   *  UnitDelay: '<S235>/Unit Delay'
   */
  rtb_Gain5 = 0.7 * VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE;

  /* Sum: '<S235>/Add2' */
  rtb_Yk1_l = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S236>/Difference Inputs1'
   *
   * Block description for '<S236>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Yk1_l -= rtb_UkYk1;

  /* RelationalOperator: '<S242>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Yk1_l > rtb_Switch2_on);

  /* Switch: '<S242>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S236>/delta fall limit' incorporates:
     *  Constant: '<S235>/Constant1'
     */
    rtb_deltafalllimit_pe = -5000.0 * elapseTime;

    /* RelationalOperator: '<S242>/UpperRelop' */
    rtb_ignition = (rtb_Yk1_l < rtb_deltafalllimit_pe);

    /* Switch: '<S242>/Switch' */
    if (rtb_ignition) {
      rtb_Yk1_l = rtb_deltafalllimit_pe;
    }

    /* End of Switch: '<S242>/Switch' */
    rtb_Switch2_on = rtb_Yk1_l;
  }

  /* End of Switch: '<S242>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l = rtb_Switch2_on +
    rtb_UkYk1;

  /* RelationalOperator: '<S239>/LowerRelop1' incorporates:
   *  Constant: '<S235>/Constant6'
   *  UnitDelay: '<S236>/Delay Input2'
   *
   * Block description for '<S236>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l >
                       1.5);

  /* Switch: '<S239>/Switch2' incorporates:
   *  Constant: '<S235>/Constant6'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S239>/UpperRelop' incorporates:
     *  Constant: '<S235>/Constant7'
     *  UnitDelay: '<S236>/Delay Input2'
     *
     * Block description for '<S236>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_ignition = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l < -1.5);

    /* Switch: '<S239>/Switch' incorporates:
     *  Constant: '<S235>/Constant7'
     *  UnitDelay: '<S236>/Delay Input2'
     *
     * Block description for '<S236>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_ignition) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l;
    }

    /* End of Switch: '<S239>/Switch' */
  }

  /* End of Switch: '<S239>/Switch2' */

  /* SignalConversion generated from: '<S208>/Out1' */
  rtb_Yk1_l = rtb_UkYk1;

  /* UnitDelay: '<S237>/Delay Input2'
   *
   * Block description for '<S237>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m;

  /* SampleTimeMath: '<S237>/sample time'
   *
   * About '<S237>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S237>/delta rise limit' incorporates:
   *  Constant: '<S235>/Constant2'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S211>/g_mpss1' */
  rtb_g_mpss1 = 9.8 * IMU_Ax_Value;

  /* Gain: '<S235>/Gain2' */
  rtb_Gain4 = 0.7 * rtb_g_mpss1;

  /* UnitDelay: '<S235>/Unit Delay1' */
  rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE;

  /* Gain: '<S235>/Gain3' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S235>/Add1' */
  rtb_deltafalllimit_pe = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S237>/Difference Inputs1'
   *
   * Block description for '<S237>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_pe -= rtb_UkYk1;

  /* RelationalOperator: '<S243>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_pe > rtb_Switch2_on);

  /* Switch: '<S243>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S237>/delta fall limit' incorporates:
     *  Constant: '<S235>/Constant4'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S243>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_pe < elapseTime);

    /* Switch: '<S243>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_pe = elapseTime;
    }

    /* End of Switch: '<S243>/Switch' */
    rtb_Switch2_on = rtb_deltafalllimit_pe;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m = rtb_Switch2_on +
    rtb_UkYk1;

  /* RelationalOperator: '<S240>/LowerRelop1' incorporates:
   *  Constant: '<S235>/Constant8'
   *  UnitDelay: '<S237>/Delay Input2'
   *
   * Block description for '<S237>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m >
                       1.5);

  /* Switch: '<S240>/Switch2' incorporates:
   *  Constant: '<S235>/Constant8'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S240>/UpperRelop' incorporates:
     *  Constant: '<S235>/Constant9'
     *  UnitDelay: '<S237>/Delay Input2'
     *
     * Block description for '<S237>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_ignition = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m < -1.5);

    /* Switch: '<S240>/Switch' incorporates:
     *  Constant: '<S235>/Constant9'
     *  UnitDelay: '<S237>/Delay Input2'
     *
     * Block description for '<S237>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_ignition) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m;
    }

    /* End of Switch: '<S240>/Switch' */
  }

  /* End of Switch: '<S240>/Switch2' */

  /* SignalConversion generated from: '<S208>/Out1' */
  rtb_deltafalllimit_pe = rtb_UkYk1;

  /* SampleTimeMath: '<S247>/sample time'
   *
   * About '<S247>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S247>/delta rise limit' */
  rtb_StrWhlAngV_c = (real32_T)(1200.0 * elapseTime);

  /* DataTypeConversion: '<S212>/Cast To Boolean1' */
  rtb_Add = (real32_T)StrWhlAngV;

  /* UnitDelay: '<S247>/Delay Input2'
   *
   * Block description for '<S247>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain3_o = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j;

  /* Sum: '<S247>/Difference Inputs1'
   *
   * Block description for '<S247>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add -= rtb_Gain3_o;

  /* RelationalOperator: '<S250>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Add > rtb_StrWhlAngV_c);

  /* Switch: '<S250>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S247>/delta fall limit' */
    rtb_StrWhlAngV_c = (real32_T)(-1200.0 * elapseTime);

    /* RelationalOperator: '<S250>/UpperRelop' */
    rtb_ignition = (rtb_Add < rtb_StrWhlAngV_c);

    /* Switch: '<S250>/Switch' */
    if (rtb_ignition) {
      rtb_Add = rtb_StrWhlAngV_c;
    }

    /* End of Switch: '<S250>/Switch' */
    rtb_StrWhlAngV_c = rtb_Add;
  }

  /* End of Switch: '<S250>/Switch2' */

  /* Saturate: '<S212>/Saturation1' incorporates:
   *  Sum: '<S247>/Difference Inputs2'
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j = rtb_StrWhlAngV_c +
    rtb_Gain3_o;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j > 1200.0F) {
    rtb_Add = 1200.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j < 0.0F) {
    rtb_Add = 0.0F;
  } else {
    rtb_Add = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j;
  }

  /* End of Saturate: '<S212>/Saturation1' */

  /* Gain: '<S212>/Gain2' */
  rtb_StrWhlAngV_c = 0.7F * rtb_Add;

  /* UnitDelay: '<S212>/Unit Delay1' */
  rtb_Gain3_o = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_aq;

  /* Gain: '<S212>/Gain3' */
  rtb_Gain3_o *= 0.3F;

  /* Sum: '<S212>/Add1' */
  rtb_StrWhlAngV_c += rtb_Gain3_o;

  /* UnitDelay: '<S238>/Delay Input2'
   *
   * Block description for '<S238>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k;

  /* SampleTimeMath: '<S238>/sample time'
   *
   * About '<S238>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S238>/delta rise limit' incorporates:
   *  Constant: '<S235>/Constant3'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S235>/Gain4' */
  rtb_Gain4 = 0.7 * IMU_Yaw_Value;

  /* UnitDelay: '<S235>/Unit Delay2' */
  rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE;

  /* Gain: '<S235>/Gain5' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S235>/Add3' */
  rtb_Gain5 += rtb_Gain4;

  /* Sum: '<S238>/Difference Inputs1'
   *
   * Block description for '<S238>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 -= rtb_UkYk1;

  /* RelationalOperator: '<S244>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Gain5 > rtb_Switch2_on);

  /* Switch: '<S244>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S238>/delta fall limit' incorporates:
     *  Constant: '<S235>/Constant5'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S244>/UpperRelop' */
    rtb_ignition = (rtb_Gain5 < elapseTime);

    /* Switch: '<S244>/Switch' */
    if (rtb_ignition) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S244>/Switch' */
    rtb_Switch2_on = rtb_Gain5;
  }

  /* End of Switch: '<S244>/Switch2' */

  /* Saturate: '<S235>/Saturation2' incorporates:
   *  Sum: '<S238>/Difference Inputs2'
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k = rtb_Switch2_on +
    rtb_UkYk1;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k > 180.0) {
    rtb_UkYk1 = 180.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k < -180.0) {
    rtb_UkYk1 = -180.0;
  } else {
    rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k;
  }

  /* End of Saturate: '<S235>/Saturation2' */

  /* SignalConversion generated from: '<S208>/Out1' */
  VCU2BMS = VehCtrlMdel240926_2018b_amksp_B.VCUEnable;

  /* Switch: '<S213>/Switch' incorporates:
   *  Constant: '<S213>/Constant4'
   */
  if (MCFL_DCVoltage != 0.0) {
    /* MinMax: '<S213>/Max' incorporates:
     *  Constant: '<S213>/Constant2'
     */
    elapseTime = fmax(MCFL_DCVoltage, 2.0);

    /* Product: '<S213>/Product' */
    rtb_Switch2_on = MCFL_ActualTorque * MCFL_ActualVelocity;

    /* Product: '<S213>/Divide' incorporates:
     *  Constant: '<S213>/Constant'
     */
    rtb_Switch2_on /= 9550.0;

    /* Product: '<S213>/Divide1' */
    AMKFL_Current = rtb_Switch2_on / elapseTime;
  } else {
    AMKFL_Current = 0.0;
  }

  /* End of Switch: '<S213>/Switch' */

  /* Switch: '<S213>/Switch1' incorporates:
   *  Constant: '<S213>/Constant5'
   */
  if (MCFR_DCVoltage != 0.0) {
    /* MinMax: '<S213>/Max1' incorporates:
     *  Constant: '<S213>/Constant3'
     */
    elapseTime = fmax(MCFR_DCVoltage, 2.0);

    /* Product: '<S213>/Product1' */
    rtb_Switch2_on = MCFR_ActualTorque * MCFR_ActualVelocity;

    /* Product: '<S213>/Divide2' incorporates:
     *  Constant: '<S213>/Constant1'
     */
    rtb_Switch2_on /= 9550.0;

    /* Product: '<S213>/Divide3' */
    AMKFR_Current = rtb_Switch2_on / elapseTime;
  } else {
    AMKFR_Current = 0.0;
  }

  /* End of Switch: '<S213>/Switch1' */

  /* Update for UnitDelay: '<S210>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm = Acc2;

  /* Update for UnitDelay: '<S210>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_k = Acc1;

  /* Update for UnitDelay: '<S212>/Unit Delay' incorporates:
   *  UnitDelay: '<S246>/Delay Input2'
   *
   * Block description for '<S246>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o2 =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2;

  /* Update for UnitDelay: '<S235>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE = rtb_g_mpss1;

  /* Update for UnitDelay: '<S212>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_aq = rtb_Add;

  /* Update for UnitDelay: '<S235>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE = IMU_Yaw_Value;

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms' */

  /* S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.Subsystem_RESET_ELAPS_T) {
    elapsedTicks = 0U;
  } else {
    elapsedTicks = VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3 -
      VehCtrlMdel240926_2018b_amks_DW.Subsystem_PREV_T;
  }

  VehCtrlMdel240926_2018b_amks_DW.Subsystem_PREV_T =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
  VehCtrlMdel240926_2018b_amks_DW.Subsystem_RESET_ELAPS_T = false;

  /* Gain: '<S209>/Gain5' */
  elapseTime = 10.0 * VehCtrlMdel240926_2018b_amksp_B.CANUnpack_o1;

  /* DataTypeConversion: '<S209>/Cast To Double' */
  rtb_CastToDouble = (real32_T)elapseTime;

  /* MinMax: '<S337>/Min3' incorporates:
   *  Gain: '<S291>/Gain'
   *  UnitDelay: '<S291>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p *= 0.3F;

  /* UnitDelay: '<S295>/Delay Input2'
   *
   * Block description for '<S295>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* SampleTimeMath: '<S295>/sample time'
   *
   * About '<S295>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S295>/delta rise limit' */
  rtb_Add4_j = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S209>/Cast To Double1' */
  rtb_Add7 = (real32_T)rtb_deltafalllimit_le;

  /* Gain: '<S277>/Gain2' */
  rtb_Add6 = 0.0174532924F * FLWhlStrAng;

  /* Trigonometry: '<S277>/Asin' */
  rtb_Add6 = cosf(rtb_Add6);

  /* Product: '<S277>/Product1' */
  rtb_Add7 *= rtb_Add6;

  /* DataTypeConversion: '<S209>/Cast To Double5' */
  rtb_Add6 = (real32_T)rtb_UkYk1;

  /* Gain: '<S277>/Gain4' */
  rtb_Add6 *= 0.0174532924F;

  /* Product: '<S277>/Product3' */
  rtb_Switch2_mn = 0.6F * rtb_Add6;

  /* Sum: '<S277>/Add2' */
  rtb_Gain3_o = rtb_Add7 - rtb_Switch2_mn;

  /* UnitDelay: '<S284>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_j;

  /* Sum: '<S284>/Add4' */
  rtb_Add7 = rtb_Gain3_o - rtb_Add7;

  /* Product: '<S284>/Divide' incorporates:
   *  Constant: '<S284>/steptime'
   */
  rtb_Add = rtb_Add7 / 0.01F;

  /* RelationalOperator: '<S296>/LowerRelop1' incorporates:
   *  Constant: '<S291>/Constant1'
   */
  rtb_FixPtRelationalOperator = (rtb_Add > 100.0F);

  /* Switch: '<S296>/Switch2' incorporates:
   *  Constant: '<S291>/Constant1'
   */
  if (rtb_FixPtRelationalOperator) {
    rtb_Add = 100.0F;
  } else {
    /* RelationalOperator: '<S296>/UpperRelop' incorporates:
     *  Constant: '<S291>/Constant'
     */
    rtb_ignition = (rtb_Add < -100.0F);

    /* Switch: '<S296>/Switch' incorporates:
     *  Constant: '<S291>/Constant'
     */
    if (rtb_ignition) {
      rtb_Add = -100.0F;
    }

    /* End of Switch: '<S296>/Switch' */
  }

  /* End of Switch: '<S296>/Switch2' */

  /* Sum: '<S295>/Difference Inputs1'
   *
   * Block description for '<S295>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add -= rtb_Switch2_b0;

  /* RelationalOperator: '<S297>/LowerRelop1' */
  rtb_FixPtRelationalOperator = (rtb_Add > rtb_Add4_j);

  /* Switch: '<S297>/Switch2' */
  if (!rtb_FixPtRelationalOperator) {
    /* Product: '<S295>/delta fall limit' */
    rtb_deltafalllimit_a = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S297>/UpperRelop' */
    rtb_ignition = (rtb_Add < rtb_deltafalllimit_a);

    /* Switch: '<S297>/Switch' */
    if (rtb_ignition) {
      rtb_Add = rtb_deltafalllimit_a;
    }

    /* End of Switch: '<S297>/Switch' */
    rtb_Add4_j = rtb_Add;
  }

  /* End of Switch: '<S297>/Switch2' */

  /* Sum: '<S295>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S295>/Delay Input2'
   *
   * Block description for '<S295>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S295>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2 = rtb_Add4_j +
    rtb_Switch2_b0;

  /* Gain: '<S291>/Gain1' incorporates:
   *  UnitDelay: '<S295>/Delay Input2'
   *
   * Block description for '<S295>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add7 = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* MinMax: '<S337>/Min3' incorporates:
   *  Abs: '<S284>/Abs'
   *  Sum: '<S284>/Add'
   *  Sum: '<S291>/Add'
   *  UnitDelay: '<S291>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p += rtb_Add7;
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p -= rtb_CastToDouble;
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p = fabsf
    (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p);

  /* RelationalOperator: '<S287>/Compare' incorporates:
   *  Constant: '<S287>/Constant'
   *  UnitDelay: '<S291>/Unit Delay'
   */
  rtb_FixPtRelationalOperator =
    (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p <= 0.5F);

  /* UnitDelay: '<S292>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj;

  /* Gain: '<S292>/Gain' */
  rtb_Add7 *= 0.3F;

  /* UnitDelay: '<S298>/Delay Input2'
   *
   * Block description for '<S298>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* SampleTimeMath: '<S298>/sample time'
   *
   * About '<S298>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S298>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(100.0 * elapseTime);

  /* MinMax: '<S337>/Min3' incorporates:
   *  DataTypeConversion: '<S209>/Cast To Double2'
   *  UnitDelay: '<S291>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p = (real32_T)WhlSpdFR;

  /* Gain: '<S277>/Gain3' */
  rtb_Add10 = 0.0174532924F * rtb_deltafalllimit_iz;

  /* Trigonometry: '<S277>/Asin1' */
  rtb_Add10 = cosf(rtb_Add10);

  /* Product: '<S277>/Product2' incorporates:
   *  UnitDelay: '<S291>/Unit Delay'
   */
  rtb_Add10 *= VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p;

  /* Sum: '<S277>/Add3' */
  rtb_Add = rtb_Switch2_mn + rtb_Add10;

  /* UnitDelay: '<S284>/Unit Delay1' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n;

  /* Sum: '<S284>/Add5' */
  rtb_Add10 = rtb_Add - rtb_Add10;

  /* Product: '<S284>/Divide1' incorporates:
   *  Constant: '<S284>/steptime1'
   */
  rtb_deltafalllimit_a = rtb_Add10 / 0.01F;

  /* RelationalOperator: '<S299>/LowerRelop1' incorporates:
   *  Constant: '<S292>/Constant1'
   */
  rtb_Compare = (rtb_deltafalllimit_a > 100.0F);

  /* Switch: '<S299>/Switch2' incorporates:
   *  Constant: '<S292>/Constant1'
   */
  if (rtb_Compare) {
    rtb_deltafalllimit_a = 100.0F;
  } else {
    /* RelationalOperator: '<S299>/UpperRelop' incorporates:
     *  Constant: '<S292>/Constant'
     */
    rtb_ignition = (rtb_deltafalllimit_a < -100.0F);

    /* Switch: '<S299>/Switch' incorporates:
     *  Constant: '<S292>/Constant'
     */
    if (rtb_ignition) {
      rtb_deltafalllimit_a = -100.0F;
    }

    /* End of Switch: '<S299>/Switch' */
  }

  /* End of Switch: '<S299>/Switch2' */

  /* Sum: '<S298>/Difference Inputs1'
   *
   * Block description for '<S298>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_a -= rtb_Add4_j;

  /* RelationalOperator: '<S300>/LowerRelop1' */
  rtb_Compare = (rtb_deltafalllimit_a > rtb_Switch2_b0);

  /* Switch: '<S300>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S298>/delta fall limit' */
    rtb_deltafalllimit_o4 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S300>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_a < rtb_deltafalllimit_o4);

    /* Switch: '<S300>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_a = rtb_deltafalllimit_o4;
    }

    /* End of Switch: '<S300>/Switch' */
    rtb_Switch2_b0 = rtb_deltafalllimit_a;
  }

  /* End of Switch: '<S300>/Switch2' */

  /* Sum: '<S298>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S298>/Delay Input2'
   *
   * Block description for '<S298>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S298>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e = rtb_Switch2_b0 +
    rtb_Add4_j;

  /* Gain: '<S292>/Gain1' incorporates:
   *  UnitDelay: '<S298>/Delay Input2'
   *
   * Block description for '<S298>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* Sum: '<S292>/Add' */
  rtb_Add7 += rtb_Add10;

  /* Sum: '<S284>/Add1' */
  rtb_Add7 -= rtb_CastToDouble;

  /* Abs: '<S284>/Abs1' */
  rtb_Add7 = fabsf(rtb_Add7);

  /* RelationalOperator: '<S288>/Compare' incorporates:
   *  Constant: '<S288>/Constant'
   */
  rtb_Compare = (rtb_Add7 <= 0.5F);

  /* UnitDelay: '<S293>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a;

  /* Gain: '<S293>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S301>/Delay Input2'
   *
   * Block description for '<S301>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* SampleTimeMath: '<S301>/sample time'
   *
   * About '<S301>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S301>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S209>/Cast To Double3' */
  rtb_Add4_j = (real32_T)WhlSpdRL_mps;

  /* Product: '<S277>/Product' */
  rtb_Add6 *= 0.58F;

  /* Sum: '<S277>/Add' */
  rtb_deltafalllimit_a = rtb_Add4_j - rtb_Add6;

  /* UnitDelay: '<S284>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_l;

  /* Sum: '<S284>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_a - rtb_Add4_j;

  /* Product: '<S284>/Divide2' incorporates:
   *  Constant: '<S284>/steptime2'
   */
  rtb_deltafalllimit_o4 = rtb_Add4_j / 0.01F;

  /* RelationalOperator: '<S302>/LowerRelop1' incorporates:
   *  Constant: '<S293>/Constant1'
   */
  rtb_LogicalOperator7 = (rtb_deltafalllimit_o4 > 100.0F);

  /* Switch: '<S302>/Switch2' incorporates:
   *  Constant: '<S293>/Constant1'
   */
  if (rtb_LogicalOperator7) {
    rtb_deltafalllimit_o4 = 100.0F;
  } else {
    /* RelationalOperator: '<S302>/UpperRelop' incorporates:
     *  Constant: '<S293>/Constant'
     */
    rtb_ignition = (rtb_deltafalllimit_o4 < -100.0F);

    /* Switch: '<S302>/Switch' incorporates:
     *  Constant: '<S293>/Constant'
     */
    if (rtb_ignition) {
      rtb_deltafalllimit_o4 = -100.0F;
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
  rtb_deltafalllimit_o4 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S303>/LowerRelop1' */
  rtb_LogicalOperator7 = (rtb_deltafalllimit_o4 > rtb_Add7);

  /* Switch: '<S303>/Switch2' */
  if (!rtb_LogicalOperator7) {
    /* Product: '<S301>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S303>/UpperRelop' */
    rtb_ignition = (rtb_deltafalllimit_o4 < rtb_Add7);

    /* Switch: '<S303>/Switch' */
    if (rtb_ignition) {
      rtb_deltafalllimit_o4 = rtb_Add7;
    }

    /* End of Switch: '<S303>/Switch' */
    rtb_Add7 = rtb_deltafalllimit_o4;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk = rtb_Add7 +
    rtb_Switch2_mn;

  /* Gain: '<S293>/Gain1' incorporates:
   *  UnitDelay: '<S301>/Delay Input2'
   *
   * Block description for '<S301>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* Sum: '<S293>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S284>/Add2' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S284>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S289>/Compare' incorporates:
   *  Constant: '<S289>/Constant'
   */
  rtb_LogicalOperator7 = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S294>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nc;

  /* Gain: '<S294>/Gain' */
  rtb_Add10 *= 0.3F;

  /* UnitDelay: '<S304>/Delay Input2'
   *
   * Block description for '<S304>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* SampleTimeMath: '<S304>/sample time'
   *
   * About '<S304>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S304>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S209>/Cast To Double4' */
  rtb_Add4_j = (real32_T)WhlSpdRR_mps;

  /* Sum: '<S277>/Add1' */
  rtb_deltafalllimit_o4 = rtb_Add6 + rtb_Add4_j;

  /* UnitDelay: '<S284>/Unit Delay3' */
  rtb_Add6 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE;

  /* Sum: '<S284>/Add7' */
  rtb_Add6 = rtb_deltafalllimit_o4 - rtb_Add6;

  /* Product: '<S284>/Divide3' incorporates:
   *  Constant: '<S284>/steptime3'
   */
  rtb_Add6 /= 0.01F;

  /* RelationalOperator: '<S305>/LowerRelop1' incorporates:
   *  Constant: '<S294>/Constant1'
   */
  rtb_ignition = (rtb_Add6 > 100.0F);

  /* Switch: '<S305>/Switch2' incorporates:
   *  Constant: '<S294>/Constant1'
   */
  if (rtb_ignition) {
    rtb_Add6 = 100.0F;
  } else {
    /* RelationalOperator: '<S305>/UpperRelop' incorporates:
     *  Constant: '<S294>/Constant'
     */
    rtb_ignition = (rtb_Add6 < -100.0F);

    /* Switch: '<S305>/Switch' incorporates:
     *  Constant: '<S294>/Constant'
     */
    if (rtb_ignition) {
      rtb_Add6 = -100.0F;
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
  rtb_Add6 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S306>/LowerRelop1' */
  rtb_ignition = (rtb_Add6 > rtb_Add7);

  /* Switch: '<S306>/Switch2' */
  if (!rtb_ignition) {
    /* Product: '<S304>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S306>/UpperRelop' */
    rtb_ignition = (rtb_Add6 < rtb_Add7);

    /* Switch: '<S306>/Switch' */
    if (rtb_ignition) {
      rtb_Add6 = rtb_Add7;
    }

    /* End of Switch: '<S306>/Switch' */
    rtb_Add7 = rtb_Add6;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c = rtb_Add7 +
    rtb_Switch2_mn;

  /* Gain: '<S294>/Gain1' incorporates:
   *  UnitDelay: '<S304>/Delay Input2'
   *
   * Block description for '<S304>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* Sum: '<S294>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Sum: '<S284>/Add3' */
  rtb_Add10 -= rtb_CastToDouble;

  /* Abs: '<S284>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S290>/Compare' incorporates:
   *  Constant: '<S290>/Constant'
   */
  rtb_ignition = (rtb_Add10 <= 0.5F);

  /* UnitDelay: '<S311>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l;

  /* Gain: '<S311>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S315>/Delay Input2'
   *
   * Block description for '<S315>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* SampleTimeMath: '<S315>/sample time'
   *
   * About '<S315>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S315>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S285>/Unit Delay4' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i;

  /* UnitDelay: '<S285>/Unit Delay' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a0;

  /* Sum: '<S285>/Add4' */
  rtb_Add4_j = rtb_Gain3_o - rtb_Add4_j;

  /* Product: '<S285>/Divide' incorporates:
   *  Constant: '<S285>/steptime'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S285>/Add' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S285>/Add8' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i - rtb_Add7;

  /* Product: '<S285>/Divide4' incorporates:
   *  Constant: '<S285>/steptime4'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S316>/LowerRelop1' incorporates:
   *  Constant: '<S311>/Constant1'
   */
  rtb_Compare_am = (rtb_Add7 > 100.0F);

  /* Switch: '<S316>/Switch2' incorporates:
   *  Constant: '<S311>/Constant1'
   */
  if (rtb_Compare_am) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S316>/UpperRelop' incorporates:
     *  Constant: '<S311>/Constant'
     */
    rtb_Compare_am = (rtb_Add7 < -100.0F);

    /* Switch: '<S316>/Switch' incorporates:
     *  Constant: '<S311>/Constant'
     */
    if (rtb_Compare_am) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S316>/Switch' */
  }

  /* End of Switch: '<S316>/Switch2' */

  /* Sum: '<S315>/Difference Inputs1'
   *
   * Block description for '<S315>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S317>/LowerRelop1' */
  rtb_Compare_am = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S317>/Switch2' */
  if (!rtb_Compare_am) {
    /* Product: '<S315>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S317>/UpperRelop' */
    rtb_Compare_am = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S317>/Switch' */
    if (rtb_Compare_am) {
      rtb_Add7 = rtb_Add6;
    }

    /* End of Switch: '<S317>/Switch' */
    rtb_Add6 = rtb_Add7;
  }

  /* End of Switch: '<S317>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S311>/Gain1' incorporates:
   *  UnitDelay: '<S315>/Delay Input2'
   *
   * Block description for '<S315>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* Sum: '<S311>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S285>/Abs' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S307>/Compare' incorporates:
   *  Constant: '<S307>/Constant'
   */
  rtb_Compare_am = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S312>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ap;

  /* Gain: '<S312>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S318>/Delay Input2'
   *
   * Block description for '<S318>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* SampleTimeMath: '<S318>/sample time'
   *
   * About '<S318>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S318>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S285>/Unit Delay5' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE;

  /* UnitDelay: '<S285>/Unit Delay1' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_a;

  /* Sum: '<S285>/Add5' */
  rtb_Add4_j = rtb_Add - rtb_Add4_j;

  /* Product: '<S285>/Divide1' incorporates:
   *  Constant: '<S285>/steptime1'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S285>/Add1' incorporates:
   *  UnitDelay: '<S285>/Unit Delay5'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S285>/Add10' incorporates:
   *  UnitDelay: '<S285>/Unit Delay5'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE - rtb_Add7;

  /* Product: '<S285>/Divide5' incorporates:
   *  Constant: '<S285>/steptime5'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S319>/LowerRelop1' incorporates:
   *  Constant: '<S312>/Constant1'
   */
  rtb_LowerRelop1_b = (rtb_Add7 > 100.0F);

  /* Switch: '<S319>/Switch2' incorporates:
   *  Constant: '<S312>/Constant1'
   */
  if (rtb_LowerRelop1_b) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S319>/UpperRelop' incorporates:
     *  Constant: '<S312>/Constant'
     */
    rtb_LowerRelop1_b = (rtb_Add7 < -100.0F);

    /* Switch: '<S319>/Switch' incorporates:
     *  Constant: '<S312>/Constant'
     */
    if (rtb_LowerRelop1_b) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S319>/Switch' */
  }

  /* End of Switch: '<S319>/Switch2' */

  /* Sum: '<S318>/Difference Inputs1'
   *
   * Block description for '<S318>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S320>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S320>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S318>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S320>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S320>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_Add7 = rtb_Add6;
    }

    /* End of Switch: '<S320>/Switch' */
    rtb_Add6 = rtb_Add7;
  }

  /* End of Switch: '<S320>/Switch2' */

  /* Sum: '<S318>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S318>/Delay Input2'
   *
   * Block description for '<S318>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S318>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S312>/Gain1' incorporates:
   *  UnitDelay: '<S318>/Delay Input2'
   *
   * Block description for '<S318>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* Sum: '<S312>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S285>/Abs1' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S308>/Compare' incorporates:
   *  Constant: '<S308>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S313>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o;

  /* Gain: '<S313>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S321>/Delay Input2'
   *
   * Block description for '<S321>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pdc;

  /* SampleTimeMath: '<S321>/sample time'
   *
   * About '<S321>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S321>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S285>/Unit Delay6' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE;

  /* UnitDelay: '<S285>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_c;

  /* Sum: '<S285>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_a - rtb_Add4_j;

  /* Product: '<S285>/Divide2' incorporates:
   *  Constant: '<S285>/steptime2'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S285>/Add2' incorporates:
   *  UnitDelay: '<S285>/Unit Delay6'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S285>/Add12' incorporates:
   *  UnitDelay: '<S285>/Unit Delay6'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE - rtb_Add7;

  /* Product: '<S285>/Divide6' incorporates:
   *  Constant: '<S285>/steptime6'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S322>/LowerRelop1' incorporates:
   *  Constant: '<S313>/Constant1'
   */
  rtb_Compare_b = (rtb_Add7 > 100.0F);

  /* Switch: '<S322>/Switch2' incorporates:
   *  Constant: '<S313>/Constant1'
   */
  if (rtb_Compare_b) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S322>/UpperRelop' incorporates:
     *  Constant: '<S313>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S322>/Switch' incorporates:
     *  Constant: '<S313>/Constant'
     */
    if (rtb_UpperRelop_ir) {
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
  rtb_Compare_b = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S323>/Switch2' */
  if (!rtb_Compare_b) {
    /* Product: '<S321>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S323>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S323>/Switch' */
    if (rtb_UpperRelop_ir) {
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pdc = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S313>/Gain1' incorporates:
   *  UnitDelay: '<S321>/Delay Input2'
   *
   * Block description for '<S321>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pdc;

  /* Sum: '<S313>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S285>/Abs2' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S309>/Compare' incorporates:
   *  Constant: '<S309>/Constant'
   */
  rtb_Compare_b = (rtb_Add10 <= 0.8F);

  /* UnitDelay: '<S314>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ah;

  /* Gain: '<S314>/Gain' */
  rtb_Add10 *= 0.5F;

  /* UnitDelay: '<S324>/Delay Input2'
   *
   * Block description for '<S324>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* SampleTimeMath: '<S324>/sample time'
   *
   * About '<S324>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S324>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S285>/Unit Delay7' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE;

  /* UnitDelay: '<S285>/Unit Delay3' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_d;

  /* Sum: '<S285>/Add7' */
  rtb_Add4_j = rtb_deltafalllimit_o4 - rtb_Add4_j;

  /* Product: '<S285>/Divide3' incorporates:
   *  Constant: '<S285>/steptime3'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S285>/Add3' incorporates:
   *  UnitDelay: '<S285>/Unit Delay7'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S285>/Add14' incorporates:
   *  UnitDelay: '<S285>/Unit Delay7'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE - rtb_Add7;

  /* Product: '<S285>/Divide7' incorporates:
   *  Constant: '<S285>/steptime7'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S325>/LowerRelop1' incorporates:
   *  Constant: '<S314>/Constant1'
   */
  rtb_UpperRelop_ir = (rtb_Add7 > 100.0F);

  /* Switch: '<S325>/Switch2' incorporates:
   *  Constant: '<S314>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S325>/UpperRelop' incorporates:
     *  Constant: '<S314>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S325>/Switch' incorporates:
     *  Constant: '<S314>/Constant'
     */
    if (rtb_UpperRelop_ir) {
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
  rtb_UpperRelop_ir = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S326>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S324>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S326>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S326>/Switch' */
    if (rtb_UpperRelop_ir) {
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S314>/Gain1' incorporates:
   *  UnitDelay: '<S324>/Delay Input2'
   *
   * Block description for '<S324>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* Sum: '<S314>/Add' */
  rtb_Add10 += rtb_Switch2_mn;

  /* Abs: '<S285>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S310>/Compare' incorporates:
   *  Constant: '<S310>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.8F);

  /* Logic: '<S275>/Logical Operator' */
  rtb_LogicalOperator_idx_0 = (rtb_FixPtRelationalOperator || rtb_Compare_am);
  rtb_Compare = (rtb_Compare || rtb_LowerRelop1_b);
  rtb_LogicalOperator7 = (rtb_LogicalOperator7 || rtb_Compare_b);
  rtb_FixPtRelationalOperator = (rtb_ignition || rtb_UpperRelop_ir);

  /* UnitDelay: '<S209>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_lh;

  /* Sum: '<S286>/Add' */
  rtb_Switch2_mn = rtb_Gain3_o - rtb_Add10;

  /* Abs: '<S286>/Abs' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S327>/Compare' incorporates:
   *  Constant: '<S327>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S286>/AND3' */
  rtb_Compare_b = (rtb_UpperRelop_ir && (VehCtrlMdel240926_2018b_amksp_B.Exit_h
    != 0.0));

  /* Sum: '<S286>/Add1' */
  rtb_Switch2_mn = rtb_Add - rtb_Add10;

  /* Abs: '<S286>/Abs1' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S328>/Compare' incorporates:
   *  Constant: '<S328>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S286>/AND2' */
  rtb_LowerRelop1_b = (rtb_UpperRelop_ir &&
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_o != 0.0));

  /* Sum: '<S286>/Add2' */
  rtb_Switch2_mn = rtb_deltafalllimit_a - rtb_Add10;

  /* Abs: '<S286>/Abs2' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S329>/Compare' incorporates:
   *  Constant: '<S329>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S286>/AND' */
  rtb_Compare_am = (rtb_UpperRelop_ir &&
                    (VehCtrlMdel240926_2018b_amksp_B.Exit_le != 0.0));

  /* Sum: '<S286>/Add3' */
  rtb_Add10 = rtb_deltafalllimit_o4 - rtb_Add10;

  /* Abs: '<S286>/Abs3' */
  rtb_Add10 = fabsf(rtb_Add10);

  /* RelationalOperator: '<S330>/Compare' incorporates:
   *  Constant: '<S330>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 2.0F);

  /* Logic: '<S286>/AND1' */
  rtb_ignition = (rtb_UpperRelop_ir && (VehCtrlMdel240926_2018b_amksp_B.Exit_i
    != 0.0));

  /* Logic: '<S275>/Logical Operator1' */
  rtb_UpperRelop_ir = (rtb_Compare_b && rtb_LogicalOperator_idx_0);
  rtb_Compare = (rtb_LowerRelop1_b && rtb_Compare);
  rtb_LogicalOperator7 = (rtb_Compare_am && rtb_LogicalOperator7);
  rtb_ignition = (rtb_ignition && rtb_FixPtRelationalOperator);

  /* Chart: '<S275>/Timer' incorporates:
   *  Constant: '<S275>/Constant1'
   */
  VehCtrlMdel240926_20_Timer1(rtb_UpperRelop_ir, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_c,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_o);

  /* Chart: '<S275>/Timer1' incorporates:
   *  Constant: '<S275>/Constant2'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_lh4,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_m);

  /* Chart: '<S275>/Timer2' incorporates:
   *  Constant: '<S275>/Constant3'
   */
  VehCtrlMdel240926_20_Timer1(rtb_LogicalOperator7, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_lh,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_g);

  /* Chart: '<S275>/Timer3' incorporates:
   *  Constant: '<S275>/Constant4'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_a,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer3_i);

  /* Logic: '<S273>/Logical Operator' */
  rtb_UpperRelop_ir = ((VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0));

  /* Logic: '<S273>/Logical Operator1' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* UnitDelay: '<S273>/Unit Delay4' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_m;

  /* Sum: '<S273>/Add1' */
  rtb_Add10 = Acc_POS_n - rtb_Add10;

  /* RelationalOperator: '<S278>/Compare' incorporates:
   *  Constant: '<S278>/Constant'
   */
  rtb_Compare_b = (rtb_Add10 > 0.1F);

  /* Logic: '<S273>/Logical Operator2' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir || rtb_Compare_b);

  /* Logic: '<S273>/AND' */
  rtb_ignition = ((VehCtrlMdel240926_2018b_amksp_B.CANUnpack_o1 != 0.0) &&
                  rtb_UpperRelop_ir);

  /* UnitDelay: '<S273>/Unit Delay3' */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_f;

  /* Logic: '<S273>/Logical Operator3' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S273>/Switch3' incorporates:
   *  UnitDelay: '<S273>/Unit Delay1'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S273>/Switch4' incorporates:
     *  Constant: '<S273>/InitZORE'
     */
    if (!rtb_ignition) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k = 0.0F;
    }

    /* End of Switch: '<S273>/Switch4' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_o =
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Switch: '<S273>/Switch3' */

  /* UnitDelay: '<S276>/Unit Delay3' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_p;

  /* Sum: '<S276>/Add5' incorporates:
   *  UnitDelay: '<S276>/Unit Delay1'
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d - rtb_Add10;

  /* Product: '<S276>/Divide3' incorporates:
   *  Constant: '<S276>/steptime3'
   */
  rtb_Add10 /= 0.01F;

  /* UnitDelay: '<S276>/Unit Delay2' */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_f;

  /* Sum: '<S276>/Add9' */
  rtb_Switch2_mn -= rtb_Add10;

  /* UnitDelay: '<S276>/Unit Delay4' */
  rtb_Add6 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn;

  /* Sum: '<S276>/Add6' incorporates:
   *  Constant: '<S276>/steptime4'
   */
  rtb_Add6 += 0.1F;

  /* Sum: '<S276>/Add8' incorporates:
   *  Constant: '<S276>/steptime6'
   */
  rtb_Add7 = rtb_Add6 + 2.0F;

  /* Product: '<S276>/Divide5' */
  rtb_Add7 = 1.0F / rtb_Add7 * rtb_Add6;

  /* Logic: '<S276>/Logical Operator' */
  rtb_FixPtRelationalOperator = ((VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0)
    || (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) ||
    (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) ||
    (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0));

  /* Switch: '<S276>/Switch13' incorporates:
   *  Constant: '<S276>/Constant10'
   */
  if (rtb_FixPtRelationalOperator) {
    rtb_Add4_j = rtb_Add7;
  } else {
    rtb_Add4_j = 1.0F;
  }

  /* End of Switch: '<S276>/Switch13' */

  /* Product: '<S276>/Divide6' */
  rtb_Switch2_mn *= rtb_Add4_j;

  /* Sum: '<S276>/Add10' */
  rtb_Ax = rtb_Switch2_mn + rtb_Add10;

  /* Switch: '<S273>/Switch1' */
  if (rtb_ignition) {
    /* Saturate: '<S273>/Saturation1' */
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d > 200.0F) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = 200.0F;
    } else {
      if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d < -10.0F) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = -10.0F;
      }
    }

    /* Product: '<S273>/Product' incorporates:
     *  Constant: '<S273>/steptime1'
     */
    rtb_Switch2_mn = rtb_Ax * 0.01F;

    /* Saturate: '<S273>/Saturation1' incorporates:
     *  Sum: '<S273>/Add'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d += rtb_Switch2_mn;
  } else {
    /* Saturate: '<S273>/Saturation1' incorporates:
     *  Constant: '<S273>/Constant'
     *  UnitDelay: '<S273>/Unit Delay'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = 0.0F;
  }

  /* End of Switch: '<S273>/Switch1' */

  /* Saturate: '<S273>/Saturation' */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d > 200.0F) {
    rtb_Add10 = 200.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d < -10.0F) {
    rtb_Add10 = -10.0F;
  } else {
    rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d;
  }

  /* End of Saturate: '<S273>/Saturation' */

  /* Sum: '<S273>/Add3' incorporates:
   *  UnitDelay: '<S273>/Unit Delay1'
   */
  rtb_VxIMU_est = rtb_Add10 +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_o;

  /* MinMax: '<S275>/Min1' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Gain3_o, rtb_Add);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_a);
  rtb_Add10 = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_o4);

  /* Sum: '<S273>/Add2' */
  rtb_Add10 -= rtb_VxIMU_est;

  /* RelationalOperator: '<S279>/Compare' incorporates:
   *  Constant: '<S279>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.0F);

  /* Switch: '<S273>/Switch6' incorporates:
   *  Constant: '<S273>/Reset'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S273>/Add10' incorporates:
     *  Constant: '<S273>/Steptime'
     */
    rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i + 0.01F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S273>/Switch6' */

  /* MinMax: '<S273>/Min' incorporates:
   *  Constant: '<S273>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i = fminf(rtb_Add10, 0.1F);

  /* RelationalOperator: '<S273>/Relational Operator9' incorporates:
   *  Constant: '<S273>/ResetDelay'
   *  UnitDelay: '<S273>/Unit Delay2'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i >= 0.1F);

  /* RelationalOperator: '<S332>/Compare' incorporates:
   *  Constant: '<S332>/Constant'
   */
  rtb_LogicalOperator7 = (rtb_Ax < -0.5F);

  /* Chart: '<S276>/Timer2' incorporates:
   *  Constant: '<S276>/Constant15'
   */
  VehCtrlMdel240926_20_Timer1(rtb_LogicalOperator7, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_j);

  /* UnitDelay: '<S336>/Delay Input2'
   *
   * Block description for '<S336>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;

  /* SampleTimeMath: '<S336>/sample time'
   *
   * About '<S336>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S336>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(10.0 * elapseTime);

  /* Sum: '<S337>/Add3' */
  rtb_Add4_j = ((rtb_deltafalllimit_o4 + rtb_deltafalllimit_a) + rtb_Add) +
    rtb_Gain3_o;

  /* MinMax: '<S337>/Min4' */
  rtb_Switch2_b0 = fminf(rtb_Gain3_o, rtb_Add);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_a);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_o4);

  /* MinMax: '<S337>/Min3' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Gain3_o, rtb_Add);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_a);
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p = fmaxf(rtb_MaxWhlSpd_mps_n,
    rtb_deltafalllimit_o4);

  /* Sum: '<S337>/Add4' incorporates:
   *  UnitDelay: '<S291>/Unit Delay'
   */
  rtb_Add4_j = (rtb_Add4_j - rtb_Switch2_b0) -
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p;

  /* Gain: '<S337>/Gain1' */
  rtb_Add4_j *= 0.5F;

  /* Sum: '<S336>/Difference Inputs1'
   *
   * Block description for '<S336>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add4_j -= rtb_Add10;

  /* RelationalOperator: '<S345>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add4_j > rtb_Switch2_mn);

  /* Switch: '<S345>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S336>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-10.0 * elapseTime);

    /* RelationalOperator: '<S345>/UpperRelop' */
    rtb_LogicalOperator7 = (rtb_Add4_j < rtb_Switch2_mn);

    /* Switch: '<S345>/Switch' */
    if (rtb_LogicalOperator7) {
      rtb_Add4_j = rtb_Switch2_mn;
    }

    /* End of Switch: '<S345>/Switch' */
    rtb_Switch2_mn = rtb_Add4_j;
  }

  /* End of Switch: '<S345>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g = rtb_Switch2_mn +
    rtb_Add10;

  /* RelationalOperator: '<S331>/Compare' incorporates:
   *  Constant: '<S331>/Constant'
   */
  rtb_LogicalOperator7 = (rtb_Ax > 0.5F);

  /* Chart: '<S276>/Timer1' incorporates:
   *  Constant: '<S276>/Constant14'
   */
  VehCtrlMdel240926_20_Timer1(rtb_LogicalOperator7, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_l,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_p);

  /* Logic: '<S276>/Logical Operator2' */
  rtb_UpperRelop_ir = !(VehCtrlMdel240926_2018b_amksp_B.Exit_l != 0.0);

  /* Switch: '<S276>/Switch6' incorporates:
   *  Switch: '<S276>/Switch4'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S276>/Switch5' incorporates:
     *  UnitDelay: '<S336>/Delay Input2'
     *
     * Block description for '<S336>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit != 0.0) {
      /* Switch: '<S276>/Switch11' incorporates:
       *  Constant: '<S276>/Constant7'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0) {
        rtb_Switch2_mn = rtb_deltafalllimit_o4;
      } else {
        rtb_Switch2_mn = 0.0F;
      }

      /* End of Switch: '<S276>/Switch11' */

      /* Switch: '<S276>/Switch10' incorporates:
       *  Constant: '<S276>/Constant6'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) {
        rtb_Add10 = rtb_deltafalllimit_a;
      } else {
        rtb_Add10 = 0.0F;
      }

      /* End of Switch: '<S276>/Switch10' */

      /* Switch: '<S276>/Switch9' incorporates:
       *  Constant: '<S276>/Constant5'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) {
        rtb_Add4_j = rtb_Add;
      } else {
        rtb_Add4_j = 0.0F;
      }

      /* End of Switch: '<S276>/Switch9' */

      /* Switch: '<S276>/Switch8' incorporates:
       *  Constant: '<S276>/Constant4'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) {
        rtb_Switch2_b0 = rtb_Gain3_o;
      } else {
        rtb_Switch2_b0 = 0.0F;
      }

      /* End of Switch: '<S276>/Switch8' */

      /* MinMax: '<S276>/Min1' */
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Switch2_b0, rtb_Add4_j);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Add10);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Switch2_mn);
    } else {
      rtb_MaxWhlSpd_mps_n = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;
    }

    /* End of Switch: '<S276>/Switch5' */
  } else {
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0) {
      /* Switch: '<S276>/Switch4' */
      rtb_Switch2_mn = rtb_deltafalllimit_o4;
    } else {
      /* Switch: '<S276>/Switch4' incorporates:
       *  Constant: '<S276>/Constant3'
       */
      rtb_Switch2_mn = 9999.0F;
    }

    /* Switch: '<S276>/Switch3' incorporates:
     *  Constant: '<S276>/Constant2'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) {
      rtb_Add10 = rtb_deltafalllimit_a;
    } else {
      rtb_Add10 = 9999.0F;
    }

    /* End of Switch: '<S276>/Switch3' */

    /* Switch: '<S276>/Switch2' incorporates:
     *  Constant: '<S276>/Constant1'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) {
      rtb_Add4_j = rtb_Add;
    } else {
      rtb_Add4_j = 9999.0F;
    }

    /* End of Switch: '<S276>/Switch2' */

    /* Switch: '<S276>/Switch1' incorporates:
     *  Constant: '<S276>/Constant'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) {
      rtb_Switch2_b0 = rtb_Gain3_o;
    } else {
      rtb_Switch2_b0 = 9999.0F;
    }

    /* End of Switch: '<S276>/Switch1' */

    /* MinMax: '<S276>/Min2' */
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add4_j);
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add10);
    rtb_MaxWhlSpd_mps_n = fminf(rtb_Switch2_b0, rtb_Switch2_mn);
  }

  /* End of Switch: '<S276>/Switch6' */

  /* Logic: '<S276>/NOT3' */
  rtb_UpperRelop_ir = !rtb_FixPtRelationalOperator;

  /* Logic: '<S276>/Logical Operator3' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir && rtb_Compare);

  /* Logic: '<S276>/NOT4' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S276>/Switch7' incorporates:
   *  UnitDelay: '<S336>/Delay Input2'
   *
   * Block description for '<S336>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (rtb_UpperRelop_ir) {
    /* Logic: '<S276>/Logical Operator1' */
    rtb_Compare = (rtb_Compare || rtb_FixPtRelationalOperator);

    /* Switch: '<S276>/Switch' */
    if (rtb_Compare) {
      rtb_VxIMU_est = rtb_MaxWhlSpd_mps_n;
    }

    /* End of Switch: '<S276>/Switch' */
  } else {
    rtb_VxIMU_est = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;
  }

  /* End of Switch: '<S276>/Switch7' */

  /* UnitDelay: '<S334>/Delay Input2'
   *
   * Block description for '<S334>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a;

  /* Sum: '<S334>/Difference Inputs1'
   *
   * Block description for '<S334>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_VxIMU_est -= rtb_Add10;

  /* Switch: '<S276>/Switch12' incorporates:
   *  Constant: '<S276>/Constant8'
   *  Constant: '<S276>/Constant9'
   */
  if (rtb_FixPtRelationalOperator) {
    rtb_Switch2_mn = 0.1F;
  } else {
    rtb_Switch2_mn = 0.05F;
  }

  /* End of Switch: '<S276>/Switch12' */

  /* Sum: '<S276>/Add4' */
  rtb_Add4_j = rtb_Ax + rtb_Switch2_mn;

  /* SampleTimeMath: '<S334>/sample time'
   *
   * About '<S334>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S334>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S343>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_VxIMU_est > rtb_Switch2_b0);

  /* Sum: '<S276>/Add3' */
  rtb_Ax -= rtb_Switch2_mn;

  /* Switch: '<S343>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S334>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S343>/UpperRelop' */
    rtb_FixPtRelationalOperator = (rtb_VxIMU_est < rtb_Switch2_mn);

    /* Switch: '<S343>/Switch' */
    if (rtb_FixPtRelationalOperator) {
      rtb_VxIMU_est = rtb_Switch2_mn;
    }

    /* End of Switch: '<S343>/Switch' */
    rtb_Switch2_b0 = rtb_VxIMU_est;
  }

  /* End of Switch: '<S343>/Switch2' */

  /* Sum: '<S334>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S334>/Delay Input2'
   *
   * Block description for '<S334>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S334>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a = rtb_Switch2_b0 +
    rtb_Add10;

  /* RelationalOperator: '<S341>/LowerRelop1' incorporates:
   *  Constant: '<S333>/Constant1'
   *  UnitDelay: '<S334>/Delay Input2'
   *
   * Block description for '<S334>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a >
                       100.0F);

  /* Switch: '<S341>/Switch2' incorporates:
   *  Constant: '<S333>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_mn = 100.0F;
  } else {
    /* RelationalOperator: '<S341>/UpperRelop' incorporates:
     *  Constant: '<S333>/Constant'
     *  UnitDelay: '<S334>/Delay Input2'
     *
     * Block description for '<S334>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_FixPtRelationalOperator =
      (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a < 0.0F);

    /* Switch: '<S341>/Switch' incorporates:
     *  Constant: '<S333>/Constant'
     *  UnitDelay: '<S334>/Delay Input2'
     *
     * Block description for '<S334>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_FixPtRelationalOperator) {
      rtb_Switch2_mn = 0.0F;
    } else {
      rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a;
    }

    /* End of Switch: '<S341>/Switch' */
  }

  /* End of Switch: '<S341>/Switch2' */

  /* UnitDelay: '<S340>/Delay Input2'
   *
   * Block description for '<S340>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* Sum: '<S340>/Difference Inputs1'
   *
   * Block description for '<S340>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_Switch2_mn - rtb_Add10;

  /* SampleTimeMath: '<S340>/sample time'
   *
   * About '<S340>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S340>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(15.0 * elapseTime);

  /* RelationalOperator: '<S342>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S342>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S340>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-15.0 * elapseTime);

    /* RelationalOperator: '<S342>/UpperRelop' */
    rtb_FixPtRelationalOperator = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S342>/Switch' */
    if (rtb_FixPtRelationalOperator) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S342>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S342>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f = rtb_Switch2_mn +
    rtb_Add10;

  /* UnitDelay: '<S333>/Unit Delay' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ncs;

  /* Gain: '<S333>/Gain' */
  rtb_Add10 *= 0.0F;

  /* Saturate: '<S30>/Saturation' incorporates:
   *  Sum: '<S333>/Add'
   *  UnitDelay: '<S340>/Delay Input2'
   *
   * Block description for '<S340>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehVxEst_mps = rtb_Add10 +
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* SampleTimeMath: '<S335>/sample time'
   *
   * About '<S335>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* UnitDelay: '<S335>/Delay Input2'
   *
   * Block description for '<S335>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu;

  /* Sum: '<S335>/Difference Inputs1'
   *
   * Block description for '<S335>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_MaxWhlSpd_mps_n - rtb_Add10;

  /* Product: '<S335>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S344>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S344>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S335>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S344>/UpperRelop' */
    rtb_FixPtRelationalOperator = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S344>/Switch' */
    if (rtb_FixPtRelationalOperator) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S344>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S344>/Switch2' */

  /* Sum: '<S335>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S335>/Delay Input2'
   *
   * Block description for '<S335>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S335>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu = rtb_Switch2_mn +
    rtb_Add10;

  /* Sum: '<S276>/Add7' incorporates:
   *  Constant: '<S276>/steptime5'
   */
  rtb_Add7 = 1.0F - rtb_Add7;

  /* Product: '<S276>/Divide4' incorporates:
   *  UnitDelay: '<S276>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn = rtb_Add7 * rtb_Add6;

  /* Update for MinMax: '<S337>/Min3' incorporates:
   *  UnitDelay: '<S291>/Unit Delay'
   *  UnitDelay: '<S295>/Delay Input2'
   *
   * Block description for '<S295>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* Update for UnitDelay: '<S284>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_j = rtb_Gain3_o;

  /* Update for UnitDelay: '<S292>/Unit Delay' incorporates:
   *  UnitDelay: '<S298>/Delay Input2'
   *
   * Block description for '<S298>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* Update for UnitDelay: '<S284>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n = rtb_Add;

  /* Update for UnitDelay: '<S293>/Unit Delay' incorporates:
   *  UnitDelay: '<S301>/Delay Input2'
   *
   * Block description for '<S301>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* Update for UnitDelay: '<S284>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_l = rtb_deltafalllimit_a;

  /* Update for UnitDelay: '<S294>/Unit Delay' incorporates:
   *  UnitDelay: '<S304>/Delay Input2'
   *
   * Block description for '<S304>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nc =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* Update for UnitDelay: '<S284>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE = rtb_deltafalllimit_o4;

  /* Update for UnitDelay: '<S311>/Unit Delay' incorporates:
   *  UnitDelay: '<S315>/Delay Input2'
   *
   * Block description for '<S315>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* Update for UnitDelay: '<S285>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a0 = rtb_Gain3_o;

  /* Update for UnitDelay: '<S312>/Unit Delay' incorporates:
   *  UnitDelay: '<S318>/Delay Input2'
   *
   * Block description for '<S318>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ap =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* Update for UnitDelay: '<S285>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_a = rtb_Add;

  /* Update for UnitDelay: '<S313>/Unit Delay' incorporates:
   *  UnitDelay: '<S321>/Delay Input2'
   *
   * Block description for '<S321>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pdc;

  /* Update for UnitDelay: '<S285>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_c = rtb_deltafalllimit_a;

  /* Update for UnitDelay: '<S314>/Unit Delay' incorporates:
   *  UnitDelay: '<S324>/Delay Input2'
   *
   * Block description for '<S324>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ah =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* Update for UnitDelay: '<S285>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_d = rtb_deltafalllimit_o4;

  /* Update for UnitDelay: '<S209>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_lh = VehVxEst_mps;

  /* Update for UnitDelay: '<S273>/Unit Delay4' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_m = Acc_POS_n;

  /* Update for UnitDelay: '<S209>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k = VehVxEst_mps;

  /* Update for UnitDelay: '<S273>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_f = rtb_ignition;

  /* Update for UnitDelay: '<S276>/Unit Delay3' incorporates:
   *  UnitDelay: '<S276>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_p =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d;

  /* Update for UnitDelay: '<S276>/Unit Delay1' incorporates:
   *  UnitDelay: '<S335>/Delay Input2'
   *
   * Block description for '<S335>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu;

  /* Update for UnitDelay: '<S276>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_f = rtb_CastToDouble;

  /* Update for UnitDelay: '<S333>/Unit Delay' incorporates:
   *  UnitDelay: '<S340>/Delay Input2'
   *
   * Block description for '<S340>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ncs =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms1' */

  /* S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* DataTypeConversion: '<S106>/Cast To Boolean' */
  KeyPressed = (ignition != 0.0);

  /* RelationalOperator: '<S108>/Compare' incorporates:
   *  Constant: '<S108>/Constant'
   */
  Brk = (Brk_F >= 600);

  /* RelationalOperator: '<S109>/Compare' incorporates:
   *  Constant: '<S109>/Constant'
   */
  ACC_Release = (Acc_POS_n <= 50.0F);

  /* Switch: '<S106>/Switch' incorporates:
   *  Constant: '<S106>/Constant1'
   *  Switch: '<S106>/Switch10'
   *  Switch: '<S106>/Switch11'
   *  Switch: '<S106>/Switch3'
   */
  if (AMKSWITCH != 0.0) {
    rtb_deltafalllimit_le = MCFL_bSystemReady;
    WhlSpdFR = MCFR_bSystemReady;
    elapseTime = MCFL_bQuitInverterOn;
    WhlSpdRR_mps = MCFR_bQuitInverterOn;
  } else {
    rtb_deltafalllimit_le = 1.0;
    WhlSpdFR = 1.0;
    elapseTime = 1.0;
    WhlSpdRR_mps = 1.0;
  }

  /* End of Switch: '<S106>/Switch' */

  /* Chart: '<S106>/Chart2' */
  elapsedTicks = VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3 -
    VehCtrlMdel240926_2018b_amks_DW.previousTicks_g;
  VehCtrlMdel240926_2018b_amks_DW.previousTicks_g =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
  if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_f + elapsedTicks <=
      255U) {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_f = (uint8_T)
      (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_f + elapsedTicks);
  } else {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_f = MAX_uint8_T;
  }

  if (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 + elapsedTicks <= 1023U)
  {
    VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 = (uint16_T)
      (VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i2 + elapsedTicks);
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
      VehCtrlMdel240926_2018b_VehStat(&controller_ready, &rtb_deltafalllimit_le,
        &WhlSpdFR, &elapseTime, &WhlSpdRR_mps);
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
        &rtb_deltafalllimit_le, &WhlSpdFR, &elapseTime, &WhlSpdRR_mps);
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

  /* End of Chart: '<S106>/Chart2' */

  /* Switch: '<S106>/Switch4' */
  VehCtrlMdel240926_2018b_amksp_B.MCFL_DCOn_setpoints_o = ((AMKSWITCH != 0.0) &&
    MCFL_DCOn_setpoints);

  /* End of Outputs for S-Function (fcncallgen): '<S2>/10ms' */

  /* S-Function (fcncallgen): '<S5>/10ms1' incorporates:
   *  SubSystem: '<S5>/Beeper'
   */
  /* S-Function (ec5744_pdsslb2u3): '<S346>/PowerDriverSwitch(LS)' */
  L9826VAR701[5]= beeper_state;
  ec_l9826tr701_control(L9826VAR701);

  /* S-Function (ec5744_pdsslbu3): '<S346>/PowerDriverSwitch(HS)2' */

  /* Set level VCU2BMS for the specified power driver switch */
  ec_gpio_write(57,VCU2BMS);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms1' */

  /* S-Function (fcncallgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/PwrTrainTempPrtct'
   */
  /* MinMax: '<S8>/Max' */
  rtb_deltafalllimit_le = fmax(MCFL_TempIGBT, MCFR_TempIGBT);
  rtb_deltafalllimit_le = fmax(rtb_deltafalllimit_le, MCFL_TempInverter);
  WhlSpdFR = fmax(rtb_deltafalllimit_le, MCFR_TempInverter);

  /* RelationalOperator: '<S96>/Compare' incorporates:
   *  Constant: '<S96>/Constant'
   */
  rtb_FixPtRelationalOperator = (WhlSpdFR > 35.0);

  /* RelationalOperator: '<S97>/Compare' incorporates:
   *  Constant: '<S97>/Constant'
   */
  rtb_Compare = (WhlSpdFR > 45.0);

  /* Logic: '<S8>/NOT' */
  rtb_LogicalOperator7 = !rtb_Compare;

  /* Logic: '<S8>/AND' */
  rtb_FixPtRelationalOperator = (rtb_FixPtRelationalOperator &&
    rtb_LogicalOperator7);

  /* Switch: '<S8>/Switch' incorporates:
   *  Constant: '<S8>/Constant'
   */
  if (rtb_FixPtRelationalOperator) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table1' */
    rtb_deltafalllimit_le = look1_binlx(WhlSpdFR,
      VehCtrlMdel240926_2018b__ConstP.pooled15,
      VehCtrlMdel240926_2018b__ConstP.pooled14, 7U);
  } else {
    rtb_deltafalllimit_le = 0.0;
  }

  /* End of Switch: '<S8>/Switch' */

  /* MinMax: '<S8>/Max2' */
  elapseTime = fmax(MCFL_TempMotor, MCFR_TempMotor);

  /* RelationalOperator: '<S101>/Compare' incorporates:
   *  Constant: '<S101>/Constant'
   */
  rtb_LogicalOperator7 = (elapseTime > 90.0);

  /* Logic: '<S8>/NOT2' */
  rtb_FixPtRelationalOperator = !rtb_LogicalOperator7;

  /* RelationalOperator: '<S102>/Compare' incorporates:
   *  Constant: '<S102>/Constant'
   */
  rtb_ignition = (elapseTime > 60.0);

  /* Logic: '<S8>/AND3' */
  rtb_FixPtRelationalOperator = (rtb_FixPtRelationalOperator && rtb_ignition);

  /* Switch: '<S8>/Switch2' incorporates:
   *  Constant: '<S8>/Constant4'
   */
  if (rtb_FixPtRelationalOperator) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table4' */
    WhlSpdRR_mps = look1_binlx(elapseTime,
      VehCtrlMdel240926_2018b__ConstP.pooled15,
      VehCtrlMdel240926_2018b__ConstP.pooled14, 7U);
  } else {
    WhlSpdRR_mps = 0.0;
  }

  /* End of Switch: '<S8>/Switch2' */

  /* MinMax: '<S8>/Max3' */
  WhlSpdRR_mps = fmin(WhlSpdRR_mps, rtb_deltafalllimit_le);

  /* SignalConversion generated from: '<S8>/Out1' */
  rtb_Switch2_on = WhlSpdRR_mps;

  /* Logic: '<S8>/AND2' */
  rtb_ignition = (rtb_LogicalOperator7 && rtb_Compare);

  /* Chart: '<S8>/Timer1' incorporates:
   *  Constant: '<S8>/Constant2'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_g,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1);

  /* RelationalOperator: '<S99>/Compare' incorporates:
   *  Constant: '<S99>/Constant'
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
  rtb_ignition = !rtb_ignition;

  /* RelationalOperator: '<S98>/Compare' incorporates:
   *  Constant: '<S98>/Constant'
   */
  rtb_LogicalOperator7 = (MCU_Temp > 45.0);

  /* Logic: '<S8>/AND1' */
  rtb_LogicalOperator7 = (rtb_LogicalOperator7 && rtb_ignition);

  /* MinMax: '<S8>/Max1' */
  WhlSpdRL_mps = fmax(MCU_Temp, motor_Temp);

  /* Switch: '<S8>/Switch1' incorporates:
   *  Constant: '<S8>/Constant1'
   */
  if (rtb_LogicalOperator7) {
    /* Lookup_n-D: '<S8>/2-D Lookup Table3' */
    WhlSpdRR_mps = look1_binlx(WhlSpdRL_mps,
      VehCtrlMdel240926_2018b__ConstP.pooled15,
      VehCtrlMdel240926_2018b__ConstP.pooled14, 7U);
  } else {
    WhlSpdRR_mps = 0.0;
  }

  /* End of Switch: '<S8>/Switch1' */

  /* SignalConversion generated from: '<S8>/Out1' */
  rtb_deltafalllimit_le = WhlSpdRR_mps;

  /* Lookup_n-D: '<S8>/2-D Lookup Table2' */
  WhlSpdRR_mps = look1_binlx(WhlSpdRL_mps,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable2_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable2_tableData, 6U);

  /* DataTypeConversion: '<S8>/Cast To Single' */
  rtb_CastToDouble = (real32_T)WhlSpdRR_mps;

  /* Gain: '<S8>/Gain' */
  rtb_CastToDouble *= 10.0F;

  /* SignalConversion generated from: '<S8>/Out1' */
  AMK_Trq_CUT = VehCtrlMdel240926_2018b_amksp_B.Exit_g;

  /* RelationalOperator: '<S103>/Compare' incorporates:
   *  Constant: '<S103>/Constant'
   */
  rtb_ignition = (elapseTime > 70.0);

  /* SignalConversion generated from: '<S8>/Out1' */
  VehCtrlMdel240926_2018b_amksp_B.bWaterPumpON = rtb_ignition;

  /* RelationalOperator: '<S100>/Compare' incorporates:
   *  Constant: '<S100>/Constant'
   */
  rtb_ignition = (WhlSpdFR > 35.0);

  /* SignalConversion generated from: '<S8>/Out1' */
  VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON = rtb_ignition;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/Function-Call Generator' */

  /* S-Function (fcncallgen): '<S1>/10ms1' incorporates:
   *  SubSystem: '<S1>/MoTrqReq'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.MoTrqReq_RESET_ELAPS_T) {
    elapsedTicks = 0U;
  } else {
    elapsedTicks = VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3 -
      VehCtrlMdel240926_2018b_amks_DW.MoTrqReq_PREV_T;
  }

  VehCtrlMdel240926_2018b_amks_DW.MoTrqReq_PREV_T =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
  VehCtrlMdel240926_2018b_amks_DW.MoTrqReq_RESET_ELAPS_T = false;

  /* Lookup_n-D: '<S7>/2-D Lookup Table1' */
  Acc_POS_n = look2_iflf_binlx(Acc_POS_n, VehVxEst_mps,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp02Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_tableData,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_maxIndex, 11U);

  /* SampleTimeMath: '<S41>/sample time'
   *
   * About '<S41>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S41>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  rtb_Gain19 = 2000.0 * elapseTime;

  /* Lookup_n-D: '<S10>/228' */
  WhlSpdFR = look1_binlx(RPM, VehCtrlMdel240926_2018b__ConstP.u28_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.u28_tableData, 26U);

  /* Lookup_n-D: '<S10>/AMK' */
  WhlSpdRR_mps = look1_binlx(MCFL_ActualVelocity,
    VehCtrlMdel240926_2018b__ConstP.pooled4,
    VehCtrlMdel240926_2018b__ConstP.pooled3, 19U);

  /* Lookup_n-D: '<S10>/AMK1' */
  WhlSpdRL_mps = look1_binlx(MCFR_ActualVelocity,
    VehCtrlMdel240926_2018b__ConstP.pooled4,
    VehCtrlMdel240926_2018b__ConstP.pooled3, 19U);

  /* MATLAB Function: '<S10>/ÔØºÉ×ªÒÆ' incorporates:
   *  Constant: '<S10>/Constant10'
   *  Constant: '<S10>/Constant2'
   *  Constant: '<S10>/Constant3'
   *  Constant: '<S10>/Constant4'
   *  Constant: '<S10>/Constant5'
   *  Constant: '<S10>/Constant9'
   */
  rtb_Add7 = (1666.0F - 340.0F * (real32_T)rtb_deltafalllimit_pe * 0.29F / 1.2F)
    * 0.521984875F - 170.0F * (real32_T)rtb_Yk1_l * 0.29F / 1.592F;
  rtb_Add10 = (340.0F * (real32_T)rtb_deltafalllimit_pe * 0.29F / 1.2F + 1666.0F)
    * 0.521984875F - 170.0F * (real32_T)rtb_Yk1_l * 0.29F / 1.592F;
  rtb_Switch2_b0 = (1666.0F - 340.0F * (real32_T)rtb_deltafalllimit_pe * 0.29F /
                    1.2F) * 0.521984875F + 170.0F * (real32_T)rtb_Yk1_l * 0.29F /
    1.592F;
  rtb_Switch2_mn = (340.0F * (real32_T)rtb_deltafalllimit_pe * 0.29F / 1.2F +
                    1666.0F) * 0.521984875F + 170.0F * (real32_T)rtb_Yk1_l *
    0.29F / 1.592F;

  /* Gain: '<S10>/Gain3' */
  rtb_g_mpss1 = 0.1020408163265306 * rtb_deltafalllimit_pe;

  /* MATLAB Function: '<S10>/MATLAB Function' incorporates:
   *  Constant: '<S10>/Constant11'
   *  Constant: '<S10>/Constant12'
   *  Constant: '<S10>/Constant13'
   *  Constant: '<S10>/Constant26'
   */
  rtb_Add6 = rtb_Add7 * 0.75F;
  rtb_Add7 = rtb_Add7 * (real32_T)rtb_g_mpss1 / 9.8F;
  rtb_Add4_j = rtb_Add10 * 0.75F;
  rtb_VxIMU_est = rtb_Add10 * (real32_T)rtb_g_mpss1 / 9.8F;
  rtb_Add10 = rtb_Switch2_b0 * 0.75F;
  rtb_Switch2_b0 = rtb_Switch2_b0 * (real32_T)rtb_g_mpss1 / 9.8F;
  rtb_Ax = rtb_Switch2_mn * 0.75F;
  rtb_MaxWhlSpd_mps_n = rtb_Switch2_mn * (real32_T)rtb_g_mpss1 / 9.8F;
  rtb_Switch2_mn = fminf(sqrtf(rtb_Add4_j * rtb_Add4_j - rtb_VxIMU_est *
    rtb_VxIMU_est) * 0.2F / 11.4F, (real32_T)WhlSpdRL_mps);
  rtb_Add6 = fminf(sqrtf(rtb_Add6 * rtb_Add6 - rtb_Add7 * rtb_Add7) * 0.2F /
                   11.4F, (real32_T)WhlSpdRR_mps);
  rtb_Add10 = fminf(fminf(sqrtf(rtb_Ax * rtb_Ax - rtb_MaxWhlSpd_mps_n *
    rtb_MaxWhlSpd_mps_n), sqrtf(rtb_Add10 * rtb_Add10 - rtb_Switch2_b0 *
    rtb_Switch2_b0)) * 0.2F / 3.4F, (real32_T)WhlSpdFR);

  /* Gain: '<S10>/Gain1' */
  rtb_Add6 *= 0.95F;

  /* UnitDelay: '<S46>/Delay Input2'
   *
   * Block description for '<S46>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_bp = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_lt;

  /* SampleTimeMath: '<S46>/sample time'
   *
   * About '<S46>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime_0 = (real_T)elapsedTicks * 0.01;

  /* Product: '<S46>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant44'
   */
  rtb_Yk1_h = 1000.0 * elapseTime_0;

  /* UnitDelay: '<S10>/Unit Delay3' */
  rtb_ignition = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i;

  /* UnitDelay: '<S44>/Delay Input2'
   *
   * Block description for '<S44>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Ax = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j3;

  /* SampleTimeMath: '<S44>/sample time'
   *
   * About '<S44>/sample time':
   *  y = K where K = ( w * Ts )
   */
  WhlSpdFR = (real_T)elapsedTicks * 0.01;

  /* Product: '<S44>/delta rise limit' */
  rtb_Add7 = (real32_T)(4000.0 * WhlSpdFR);

  /* Gain: '<S10>/Gain21' */
  WhlSpdRR_mps = 0.1020408163265306 * rtb_Yk1_l;

  /* MATLAB Function: '<S10>/Wtarget' incorporates:
   *  Constant: '<S10>/Constant14'
   *  Constant: '<S10>/Constant15'
   *  Constant: '<S10>/Constant16'
   *  Constant: '<S10>/Constant17'
   *  Constant: '<S10>/Constant18'
   *  Constant: '<S10>/Constant19'
   */
  rtb_Add4_j = VehVxEst_mps * rtb_APP_POS1 / (340.0F * VehVxEst_mps *
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

  rtb_Add4_j = fminf(fabsf(5.88F / (real32_T)WhlSpdRR_mps), fabsf(rtb_Add4_j)) *
    0.8F * rtb_Switch2_b0;

  /* End of MATLAB Function: '<S10>/Wtarget' */

  /* Sum: '<S44>/Difference Inputs1'
   *
   * Block description for '<S44>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add4_j -= rtb_Ax;

  /* RelationalOperator: '<S61>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Add4_j > rtb_Add7);

  /* Switch: '<S61>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S44>/delta fall limit' */
    rtb_Add7 = (real32_T)(-4000.0 * WhlSpdFR);

    /* RelationalOperator: '<S61>/UpperRelop' */
    rtb_FixPtRelationalOperator = (rtb_Add4_j < rtb_Add7);

    /* Switch: '<S61>/Switch' */
    if (rtb_FixPtRelationalOperator) {
      rtb_Add4_j = rtb_Add7;
    }

    /* End of Switch: '<S61>/Switch' */
    rtb_Add7 = rtb_Add4_j;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j3 = rtb_Add7 + rtb_Ax;

  /* Sum: '<S10>/Add' incorporates:
   *  UnitDelay: '<S44>/Delay Input2'
   *
   * Block description for '<S44>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_gd = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j3 -
    rtb_UkYk1;

  /* Abs: '<S10>/Abs' */
  rtb_deltafalllimit_m5 = fabs(rtb_Switch2_gd);

  /* RelationalOperator: '<S33>/Compare' incorporates:
   *  Constant: '<S33>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_deltafalllimit_m5 > 4.0);

  /* Abs: '<S10>/Abs1' */
  rtb_deltafalllimit_m5 = fabs(rtb_UkYk1);

  /* RelationalOperator: '<S34>/Compare' incorporates:
   *  Constant: '<S34>/Constant'
   */
  rtb_Compare_b = (rtb_deltafalllimit_m5 > 1.0);

  /* RelationalOperator: '<S35>/Compare' incorporates:
   *  Constant: '<S35>/Constant'
   */
  rtb_UpperRelop_ir = (VehVxEst_mps > 2.0F);

  /* Logic: '<S10>/AND' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b && rtb_Compare_b && rtb_UpperRelop_ir);

  /* Logic: '<S10>/Logical Operator4' */
  rtb_ignition = ((!rtb_ignition) && (!rtb_LowerRelop1_b));

  /* Abs: '<S10>/Abs2' */
  rtb_deltafalllimit_m5 = fabs(rtb_Switch2_gd);

  /* RelationalOperator: '<S36>/Compare' incorporates:
   *  Constant: '<S36>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_deltafalllimit_m5 < 3.0);

  /* RelationalOperator: '<S37>/Compare' incorporates:
   *  Constant: '<S37>/Constant'
   */
  rtb_Compare_b = (rtb_Yk1_l < -5.0);

  /* Logic: '<S10>/OR' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir || rtb_Compare_b);

  /* Switch: '<S10>/Switch6' incorporates:
   *  Constant: '<S10>/Reset'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S10>/Add10' incorporates:
     *  Constant: '<S10>/Steptime'
     */
    rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_j + 0.01F;
  } else {
    rtb_Add7 = 0.0F;
  }

  /* End of Switch: '<S10>/Switch6' */

  /* MinMax: '<S10>/Min' incorporates:
   *  Constant: '<S10>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_j = fminf(rtb_Add7, 1.0F);

  /* RelationalOperator: '<S10>/Relational Operator9' incorporates:
   *  Constant: '<S10>/ResetDelay'
   *  UnitDelay: '<S10>/Unit Delay4'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_j >=
                       1.0F);

  /* Logic: '<S10>/Logical Operator5' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i = ((!rtb_ignition) &&
    (!rtb_UpperRelop_ir));

  /* Gain: '<S10>/Gain' */
  rtb_Add7 = 0.95F * rtb_Switch2_mn;

  /* MinMax: '<S10>/Min1' */
  rtb_Ax = fminf(rtb_Add7, rtb_Add6);

  /* Gain: '<S10>/Gain2' */
  rtb_Switch2_mn = 0.95F * rtb_Add10;

  /* Product: '<S10>/Divide3' */
  rtb_Add4_j = rtb_Ax / rtb_Switch2_mn;

  /* UnitDelay: '<S43>/Delay Input2'
   *
   * Block description for '<S43>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_VxIMU_est = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l4;

  /* SampleTimeMath: '<S43>/sample time'
   *
   * About '<S43>/sample time':
   *  y = K where K = ( w * Ts )
   */
  WhlSpdFR = (real_T)elapsedTicks * 0.01;

  /* Product: '<S43>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant48'
   */
  rtb_Add10 = (real32_T)(4.0 * WhlSpdFR);

  /* Abs: '<S10>/Abs5' */
  rtb_Switch2_b0 = fabsf(rtb_APP_POS1);

  /* Lookup_n-D: '<S10>/2-D Lookup Table1' */
  rtb_Switch2_b0 = look2_iflf_binlx(rtb_Switch2_b0, VehVxEst_mps,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp01Data_n,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp02Data_h,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_tableData_i,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_maxIndex_h, 5U);

  /* Sum: '<S43>/Difference Inputs1'
   *
   * Block description for '<S43>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 -= rtb_VxIMU_est;

  /* RelationalOperator: '<S60>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Add10);

  /* Switch: '<S60>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S43>/delta fall limit' */
    rtb_Add10 = (real32_T)(-4.0 * WhlSpdFR);

    /* RelationalOperator: '<S60>/UpperRelop' */
    rtb_ignition = (rtb_Switch2_b0 < rtb_Add10);

    /* Switch: '<S60>/Switch' */
    if (rtb_ignition) {
      rtb_Switch2_b0 = rtb_Add10;
    }

    /* End of Switch: '<S60>/Switch' */
    rtb_Add10 = rtb_Switch2_b0;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l4 = rtb_Add10 +
    rtb_VxIMU_est;

  /* Sum: '<S10>/Add18' incorporates:
   *  UnitDelay: '<S43>/Delay Input2'
   *
   * Block description for '<S43>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add4_j += VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l4;

  /* Saturate: '<S10>/Saturation3' */
  if (rtb_Add4_j > 0.7F) {
    rtb_Add4_j = 0.7F;
  } else {
    if (rtb_Add4_j < 0.1F) {
      rtb_Add4_j = 0.1F;
    }
  }

  /* End of Saturate: '<S10>/Saturation3' */

  /* Gain: '<S10>/Gain4' */
  rtb_Switch2_b0 = 0.5F * rtb_Add4_j;

  /* Product: '<S10>/Product2' */
  rtb_Switch2_b0 *= Acc_POS_n;

  /* Gain: '<S10>/Gain26' */
  rtb_MaxWhlSpd_mps_n = 0.8F * rtb_Switch2_b0;

  /* Logic: '<S10>/AND3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_UpperRelop_ir = ((VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_n !=
                        0.0) &&
                       VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i);

  /* Switch: '<S10>/Switch1' incorporates:
   *  Constant: '<S10>/Constant1'
   */
  if (!rtb_UpperRelop_ir) {
    rtb_Switch2_gd = 0.0;
  }

  /* End of Switch: '<S10>/Switch1' */

  /* Product: '<S10>/Product3' */
  rtb_deltafalllimit_m5 = 2.0 * rtb_Switch2_gd;

  /* UnitDelay: '<S10>/Unit Delay2' */
  rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jp;

  /* Product: '<S10>/Product' */
  rtb_Add10 *= rtb_APP_POS1;

  /* RelationalOperator: '<S32>/Compare' incorporates:
   *  Constant: '<S32>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10 <= 0.0F);

  /* UnitDelay: '<S10>/Unit Delay' */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_h;

  /* Switch: '<S10>/Switch' incorporates:
   *  Constant: '<S10>/Constant'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Gain4 = 0.0;
  }

  /* End of Switch: '<S10>/Switch' */

  /* Product: '<S10>/Product4' */
  WhlSpdRR_mps = rtb_Switch2_gd;

  /* UnitDelay: '<S10>/Unit Delay1' */
  rtb_Switch2_gd = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_c;

  /* Product: '<S10>/Product5' */
  WhlSpdFR = rtb_Switch2_gd;

  /* Sum: '<S10>/Add2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_h = (rtb_Gain4 + WhlSpdRR_mps)
    - WhlSpdFR;

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
                       -3.33390474F) * 105.0F * rtb_APP_POS1 - -1.35294116F /
                      VehVxEst_mps * 105.0F * rtb_StrWhlAngV_c) /
    (-0.0458234884F / VehVxEst_mps / VehVxEst_mps - 1.0F) / 100.0F;

  /* UnitDelay: '<S10>/Unit Delay6' */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE_b;

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
  WhlSpdFR = (rtb_deltafalllimit_m5 +
              VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_h) +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_k;

  /* Saturate: '<S10>/Saturation' */
  if (WhlSpdFR > 5000.0) {
    WhlSpdRR_mps = 5000.0;
  } else if (WhlSpdFR < -5000.0) {
    WhlSpdRR_mps = -5000.0;
  } else {
    WhlSpdRR_mps = WhlSpdFR;
  }

  /* End of Saturate: '<S10>/Saturation' */

  /* Gain: '<S10>/Gain8' */
  rtb_Add10 = 0.0174532924F * FLWhlStrAng;

  /* Trigonometry: '<S10>/Cos' */
  rtb_Add10 = cosf(rtb_Add10);

  /* Gain: '<S10>/Gain11' */
  rtb_Add10 *= 1.2F;

  /* Sum: '<S10>/Add6' incorporates:
   *  Constant: '<S10>/Constant27'
   */
  rtb_VxIMU_est = 90.0F - FLWhlStrAng;

  /* Gain: '<S10>/Gain9' */
  rtb_VxIMU_est *= 0.0174532924F;

  /* Trigonometry: '<S10>/Cos1' */
  rtb_VxIMU_est = cosf(rtb_VxIMU_est);

  /* Gain: '<S10>/Gain10' */
  rtb_VxIMU_est *= 1.522F;

  /* Sum: '<S10>/Add7' */
  rtb_Add10 += rtb_VxIMU_est;

  /* Product: '<S10>/Divide1' */
  rtb_Gain4 = WhlSpdRR_mps / rtb_Add10;

  /* Gain: '<S10>/Gain25' */
  rtb_Gain4 *= 0.2;

  /* RelationalOperator: '<S53>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Gain4 > rtb_MaxWhlSpd_mps_n);

  /* Switch: '<S53>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Gain4 = rtb_MaxWhlSpd_mps_n;
  } else {
    /* Gain: '<S10>/Gain27' */
    rtb_StrWhlAngV_c = -rtb_MaxWhlSpd_mps_n;

    /* RelationalOperator: '<S53>/UpperRelop' */
    rtb_ignition = (rtb_Gain4 < rtb_StrWhlAngV_c);

    /* Switch: '<S53>/Switch' */
    if (rtb_ignition) {
      rtb_Gain4 = rtb_StrWhlAngV_c;
    }

    /* End of Switch: '<S53>/Switch' */
  }

  /* End of Switch: '<S53>/Switch2' */

  /* Sum: '<S10>/Add4' */
  rtb_Gain5 = rtb_Switch2_b0 + rtb_Gain4;

  /* Sum: '<S10>/Add14' */
  WhlSpdRL_mps = rtb_Ax - rtb_Gain5;

  /* RelationalOperator: '<S10>/Relational Operator' incorporates:
   *  Constant: '<S10>/Constant37'
   */
  rtb_FixPtRelationalOperator = (WhlSpdRL_mps < 0.0);

  /* Sum: '<S10>/Add17' incorporates:
   *  Constant: '<S10>/Constant47'
   */
  rtb_Gain4 = 1.0 - rtb_Add4_j;

  /* Product: '<S10>/Product1' */
  rtb_g_mpss1 = Acc_POS_n * rtb_Gain4;

  /* Sum: '<S10>/Add15' */
  rtb_Gain4 = rtb_Switch2_mn - rtb_g_mpss1;

  /* RelationalOperator: '<S10>/Relational Operator1' incorporates:
   *  Constant: '<S10>/Constant38'
   */
  rtb_Compare = (rtb_Gain4 < 0.0);

  /* Logic: '<S10>/AND2' */
  rtb_UpperRelop_ir = (rtb_FixPtRelationalOperator && rtb_Compare);

  /* Logic: '<S10>/OR2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i ||
                       rtb_UpperRelop_ir);

  /* Switch: '<S10>/Switch7' incorporates:
   *  Constant: '<S10>/Constant39'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Gain4 = 0.0;
  } else {
    /* Logic: '<S10>/NOT' */
    rtb_ignition = !rtb_FixPtRelationalOperator;

    /* Switch: '<S10>/Switch8' incorporates:
     *  Constant: '<S10>/Constant40'
     */
    if (!rtb_ignition) {
      rtb_Gain4 = 0.0;
    }

    /* End of Switch: '<S10>/Switch8' */
  }

  /* End of Switch: '<S10>/Switch7' */

  /* Gain: '<S10>/Gain20' */
  rtb_Gain4 = -rtb_Gain4;

  /* Saturate: '<S10>/Saturation2' */
  if (rtb_Gain4 > 100.0) {
    rtb_Gain4 = 100.0;
  } else {
    if (rtb_Gain4 < 0.0) {
      rtb_Gain4 = 0.0;
    }
  }

  /* End of Saturate: '<S10>/Saturation2' */

  /* Sum: '<S46>/Difference Inputs1'
   *
   * Block description for '<S46>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain4 -= rtb_Switch2_bp;

  /* RelationalOperator: '<S63>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Gain4 > rtb_Yk1_h);

  /* Switch: '<S63>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S46>/delta fall limit' */
    rtb_deltafalllimit_m5 = -1000.0 * elapseTime_0;

    /* RelationalOperator: '<S63>/UpperRelop' */
    rtb_ignition = (rtb_Gain4 < rtb_deltafalllimit_m5);

    /* Switch: '<S63>/Switch' */
    if (rtb_ignition) {
      rtb_Gain4 = rtb_deltafalllimit_m5;
    }

    /* End of Switch: '<S63>/Switch' */
    rtb_Yk1_h = rtb_Gain4;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_lt = rtb_Yk1_h +
    rtb_Switch2_bp;

  /* Gain: '<S10>/Gain14' */
  rtb_Add10 = 0.0174532924F * rtb_deltafalllimit_iz;

  /* Trigonometry: '<S10>/Cos2' */
  rtb_Add10 = cosf(rtb_Add10);

  /* Gain: '<S10>/Gain13' */
  rtb_Add10 *= 1.2F;

  /* Sum: '<S10>/Add8' incorporates:
   *  Constant: '<S10>/Constant28'
   */
  rtb_VxIMU_est = 90.0F - rtb_deltafalllimit_iz;

  /* Gain: '<S10>/Gain15' */
  rtb_VxIMU_est *= 0.0174532924F;

  /* Trigonometry: '<S10>/Cos3' */
  rtb_VxIMU_est = cosf(rtb_VxIMU_est);

  /* Gain: '<S10>/Gain12' */
  rtb_VxIMU_est *= 1.522F;

  /* Sum: '<S10>/Add9' */
  rtb_Add10 += rtb_VxIMU_est;

  /* Product: '<S10>/Divide2' */
  rtb_Gain4 = WhlSpdRR_mps / rtb_Add10;

  /* Gain: '<S10>/Gain24' */
  rtb_Gain4 *= 0.2;

  /* RelationalOperator: '<S54>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Gain4 > rtb_MaxWhlSpd_mps_n);

  /* Switch: '<S54>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Gain4 = rtb_MaxWhlSpd_mps_n;
  } else {
    /* Gain: '<S10>/Gain28' */
    rtb_deltafalllimit_iz = -rtb_MaxWhlSpd_mps_n;

    /* RelationalOperator: '<S54>/UpperRelop' */
    rtb_ignition = (rtb_Gain4 < rtb_deltafalllimit_iz);

    /* Switch: '<S54>/Switch' */
    if (rtb_ignition) {
      rtb_Gain4 = rtb_deltafalllimit_iz;
    }

    /* End of Switch: '<S54>/Switch' */
  }

  /* End of Switch: '<S54>/Switch2' */

  /* Sum: '<S10>/Add5' */
  rtb_Gain4 = rtb_Switch2_b0 - rtb_Gain4;

  /* RelationalOperator: '<S48>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Gain4 > rtb_Add6);

  /* Switch: '<S48>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Gain4 = rtb_Add6;
  } else {
    /* RelationalOperator: '<S48>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    rtb_ignition = (rtb_Gain4 < 0.0);

    /* Switch: '<S48>/Switch' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    if (rtb_ignition) {
      rtb_Gain4 = 0.0;
    }

    /* End of Switch: '<S48>/Switch' */
  }

  /* End of Switch: '<S48>/Switch2' */

  /* Sum: '<S10>/Add12' incorporates:
   *  UnitDelay: '<S46>/Delay Input2'
   *
   * Block description for '<S46>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain4 += VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_lt;

  /* RelationalOperator: '<S51>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Gain4 > rtb_Add6);

  /* Switch: '<S51>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Gain4 = rtb_Add6;
  } else {
    /* RelationalOperator: '<S51>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    rtb_ignition = (rtb_Gain4 < 0.0);

    /* Switch: '<S51>/Switch' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    if (rtb_ignition) {
      rtb_Gain4 = 0.0;
    }

    /* End of Switch: '<S51>/Switch' */
  }

  /* End of Switch: '<S51>/Switch2' */

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
  rtb_Gain4 -= VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p;

  /* RelationalOperator: '<S58>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Gain4 > rtb_Gain19);

  /* Switch: '<S58>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S41>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S58>/UpperRelop' */
    rtb_ignition = (rtb_Gain4 < elapseTime);

    /* Switch: '<S58>/Switch' */
    if (rtb_ignition) {
      rtb_Gain4 = elapseTime;
    }

    /* End of Switch: '<S58>/Switch' */
    rtb_Gain19 = rtb_Gain4;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p += rtb_Gain19;

  /* Sum: '<S28>/Add4' incorporates:
   *  Constant: '<S28>/Constant'
   */
  rtb_Switch2_on = 1.0 - rtb_Switch2_on;

  /* UnitDelay: '<S79>/Unit Delay1' */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl;

  /* Saturate: '<S30>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    rtb_Switch2_b0 = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    rtb_Switch2_b0 = 0.0F;
  } else {
    rtb_Switch2_b0 = VehVxEst_mps;
  }

  /* Lookup_n-D: '<S30>/VehSpd_SlipTarget_mps' */
  rtb_Add10 = look1_iflf_binlc(rtb_Switch2_b0,
    VehCtrlMdel240926_2018b__ConstP.pooled55,
    VehCtrlMdel240926_2018b__ConstP.pooled54, 3U);

  /* Sum: '<S30>/Add9' */
  rtb_Add10 += rtb_Switch2_b0;

  /* Saturate: '<S30>/Saturation1' */
  if (rtb_Add > 50.0F) {
    rtb_VxIMU_est = 50.0F;
  } else if (rtb_Add < 0.0F) {
    rtb_VxIMU_est = 0.0F;
  } else {
    rtb_VxIMU_est = rtb_Add;
  }

  /* End of Saturate: '<S30>/Saturation1' */

  /* Sum: '<S30>/Add1' */
  rtb_deltafalllimit_iz = rtb_Add10 - rtb_VxIMU_est;

  /* RelationalOperator: '<S30>/Relational Operator7' incorporates:
   *  Constant: '<S30>/Cal_DeltaV_mps'
   */
  rtb_Compare_b = (rtb_deltafalllimit_iz < 0.0F);

  /* Logic: '<S79>/Logical Operator4' */
  rtb_UpperRelop_ir = ((!rtb_UpperRelop_ir) && (!rtb_Compare_b));

  /* Logic: '<S30>/Logical Operator2' */
  rtb_Compare_b = !rtb_Compare_b;

  /* UnitDelay: '<S30>/Unit Delay4' */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE;

  /* RelationalOperator: '<S30>/Relational Operator8' incorporates:
   *  Constant: '<S30>/Cal_DeltaV_mps1'
   */
  rtb_LowerRelop1_b = (rtb_Gain4 > 235.0);

  /* Logic: '<S30>/Logical Operator1' */
  rtb_Compare_b = (rtb_Compare_b && rtb_LowerRelop1_b);

  /* Switch: '<S80>/Switch6' incorporates:
   *  Constant: '<S80>/Reset'
   */
  if (rtb_Compare_b) {
    /* Sum: '<S80>/Add10' incorporates:
     *  Constant: '<S80>/Steptime'
     */
    rtb_Add10 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 + 0.01F;
  } else {
    rtb_Add10 = 0.0F;
  }

  /* End of Switch: '<S80>/Switch6' */

  /* MinMax: '<S80>/Min' incorporates:
   *  Constant: '<S30>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 = fminf(rtb_Add10, 0.1F);

  /* RelationalOperator: '<S80>/Relational Operator9' incorporates:
   *  Constant: '<S30>/ResetDelay'
   *  UnitDelay: '<S80>/Unit Delay1'
   */
  rtb_Compare_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 >= 0.1F);

  /* UnitDelay: '<S30>/Unit Delay3' */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e;

  /* Logic: '<S30>/Logical Operator3' */
  rtb_Compare_b = (rtb_Compare_b || rtb_LowerRelop1_b);

  /* Logic: '<S79>/Logical Operator5' incorporates:
   *  UnitDelay: '<S79>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl = ((!rtb_UpperRelop_ir) &&
    (!rtb_Compare_b));

  /* UnitDelay: '<S90>/Unit Delay1' */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e;

  /* Saturate: '<S31>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    rtb_Add10 = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    rtb_Add10 = 0.0F;
  } else {
    rtb_Add10 = VehVxEst_mps;
  }

  /* End of Saturate: '<S31>/Saturation' */

  /* Lookup_n-D: '<S31>/VehSpd_SlipTarget_mps' */
  rtb_Add4_j = look1_iflf_binlc(rtb_Add10,
    VehCtrlMdel240926_2018b__ConstP.pooled55,
    VehCtrlMdel240926_2018b__ConstP.pooled54, 3U);

  /* Sum: '<S31>/Add9' */
  rtb_Add4_j += rtb_Add10;

  /* Sum: '<S7>/Add1' */
  rtb_Ax = rtb_deltafalllimit_a + rtb_deltafalllimit_o4;

  /* Gain: '<S7>/Gain2' */
  rtb_Ax *= 0.5F;

  /* Saturate: '<S31>/Saturation1' */
  if (rtb_Ax < 0.0F) {
    rtb_Add = 0.0F;
  } else {
    rtb_Add = rtb_Ax;
  }

  /* End of Saturate: '<S31>/Saturation1' */

  /* Sum: '<S31>/Add1' */
  rtb_StrWhlAngV_c = rtb_Add4_j - rtb_Add;

  /* RelationalOperator: '<S31>/Relational Operator7' incorporates:
   *  Constant: '<S31>/Cal_DeltaV_mps'
   */
  rtb_Compare_b = (rtb_StrWhlAngV_c < 0.0F);

  /* Logic: '<S90>/Logical Operator4' */
  rtb_UpperRelop_ir = ((!rtb_UpperRelop_ir) && (!rtb_Compare_b));

  /* Logic: '<S31>/Logical Operator2' */
  rtb_Compare_b = !rtb_Compare_b;

  /* UnitDelay: '<S31>/Unit Delay4' */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b;

  /* RelationalOperator: '<S31>/Relational Operator8' incorporates:
   *  Constant: '<S31>/Cal_DeltaV_mps1'
   */
  rtb_LowerRelop1_b = (rtb_Gain4 > 235.0);

  /* Logic: '<S31>/Logical Operator1' */
  rtb_Compare_b = (rtb_Compare_b && rtb_LowerRelop1_b);

  /* Switch: '<S91>/Switch6' incorporates:
   *  Constant: '<S91>/Reset'
   */
  if (rtb_Compare_b) {
    /* Sum: '<S91>/Add10' incorporates:
     *  Constant: '<S91>/Steptime'
     */
    rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i + 0.01F;
  } else {
    rtb_Add4_j = 0.0F;
  }

  /* End of Switch: '<S91>/Switch6' */

  /* MinMax: '<S91>/Min' incorporates:
   *  Constant: '<S31>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i = fminf(rtb_Add4_j, 0.1F);

  /* RelationalOperator: '<S91>/Relational Operator9' incorporates:
   *  Constant: '<S31>/ResetDelay'
   *  UnitDelay: '<S91>/Unit Delay1'
   */
  rtb_Compare_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i >= 0.1F);

  /* UnitDelay: '<S31>/Unit Delay3' */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_a;

  /* Logic: '<S31>/Logical Operator3' */
  rtb_Compare_b = (rtb_Compare_b || rtb_LowerRelop1_b);

  /* Logic: '<S90>/Logical Operator5' incorporates:
   *  UnitDelay: '<S90>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e = ((!rtb_UpperRelop_ir) &&
    (!rtb_Compare_b));

  /* UnitDelay: '<S70>/Unit Delay1' */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp;

  /* Saturate: '<S29>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    rtb_Add4_j = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    rtb_Add4_j = 0.0F;
  } else {
    rtb_Add4_j = VehVxEst_mps;
  }

  /* End of Saturate: '<S29>/Saturation' */

  /* Lookup_n-D: '<S29>/VehSpd_SlipTarget_mps' */
  rtb_Ax = look1_iflf_binlc(rtb_Add4_j, VehCtrlMdel240926_2018b__ConstP.pooled55,
    VehCtrlMdel240926_2018b__ConstP.pooled54, 3U);

  /* Sum: '<S29>/Add9' */
  rtb_Ax += rtb_Add4_j;

  /* Saturate: '<S29>/Saturation1' */
  if (rtb_Gain3_o < 0.0F) {
    rtb_Add6 = 0.0F;
  } else {
    rtb_Add6 = rtb_Gain3_o;
  }

  /* End of Saturate: '<S29>/Saturation1' */

  /* Sum: '<S29>/Add1' */
  FLWhlStrAng = rtb_Ax - rtb_Add6;

  /* RelationalOperator: '<S29>/Relational Operator7' incorporates:
   *  Constant: '<S29>/Cal_DeltaV_mps'
   */
  rtb_Compare_b = (FLWhlStrAng < 0.0F);

  /* Logic: '<S70>/Logical Operator4' */
  rtb_UpperRelop_ir = ((!rtb_UpperRelop_ir) && (!rtb_Compare_b));

  /* Logic: '<S29>/Logical Operator2' */
  rtb_Compare_b = !rtb_Compare_b;

  /* UnitDelay: '<S29>/Unit Delay4' */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l;

  /* RelationalOperator: '<S29>/Relational Operator8' incorporates:
   *  Constant: '<S29>/Cal_DeltaV_mps1'
   */
  rtb_LowerRelop1_b = (rtb_Gain4 > 235.0);

  /* Logic: '<S29>/Logical Operator1' */
  rtb_Compare_b = (rtb_Compare_b && rtb_LowerRelop1_b);

  /* Switch: '<S71>/Switch6' incorporates:
   *  Constant: '<S71>/Reset'
   */
  if (rtb_Compare_b) {
    /* Sum: '<S71>/Add10' incorporates:
     *  Constant: '<S71>/Steptime'
     */
    rtb_Ax = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h + 0.01F;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S71>/Switch6' */

  /* MinMax: '<S71>/Min' incorporates:
   *  Constant: '<S29>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h = fminf(rtb_Ax, 0.1F);

  /* RelationalOperator: '<S71>/Relational Operator9' incorporates:
   *  Constant: '<S29>/ResetDelay'
   *  UnitDelay: '<S71>/Unit Delay1'
   */
  rtb_Compare_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h >= 0.1F);

  /* UnitDelay: '<S29>/Unit Delay3' */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_ip;

  /* Logic: '<S29>/Logical Operator3' */
  rtb_Compare_b = (rtb_Compare_b || rtb_LowerRelop1_b);

  /* Logic: '<S70>/Logical Operator5' incorporates:
   *  UnitDelay: '<S70>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp = ((!rtb_UpperRelop_ir) &&
    (!rtb_Compare_b));

  /* Chart: '<S7>/Chart' */
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c7_VehCtrlMdel240926_
      == 0U) {
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c7_VehCtrlMdel240926_ =
      1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_B = 3U;
    VehCtrlMdel240926_2018b_amks_DW.b = rtb_Yk1_l * rtb_Yk1_l +
      rtb_deltafalllimit_pe * rtb_deltafalllimit_pe;
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
      rtb_LogicalOperator7 = ((rtb_UkYk1 >= 50.0) || (rtb_deltafalllimit_pe >=
        5.0) || (VehCtrlMdel240926_2018b_amks_DW.b >= 5.0));
      if (rtb_LogicalOperator7) {
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

    if (fabs(rtb_deltafalllimit_pe) > 0.5) {
      elapseTime = atan(rtb_Yk1_l / rtb_deltafalllimit_pe) * 180.0 /
        3.1415926535897931;
    } else {
      elapseTime = 0.0;
    }

    VehCtrlMdel240926_2018b_amks_DW.b = rtb_Yk1_l * rtb_Yk1_l +
      rtb_deltafalllimit_pe * rtb_deltafalllimit_pe;
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

  /* Switch: '<S30>/Switch6' incorporates:
   *  Constant: '<S30>/Verror_Reset'
   *  UnitDelay: '<S79>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl) {
    rtb_Ax = rtb_deltafalllimit_iz;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S30>/Switch6' */

  /* Product: '<S30>/Product' incorporates:
   *  Constant: '<S30>/P_Gain'
   */
  rtb_Gain3_o = rtb_Ax * 40.0F;

  /* Sum: '<S30>/Add11' */
  rtb_Gain4 = MCFR_ActualTorque - rtb_Gain3_o;

  /* Saturate: '<S30>/Saturation2' incorporates:
   *  Product: '<S30>/Product2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i *= rtb_deltafalllimit_iz;

  /* RelationalOperator: '<S76>/Compare' incorporates:
   *  Constant: '<S76>/Constant'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i <=
                       0.0F);

  /* Saturate: '<S30>/Saturation2' incorporates:
   *  UnitDelay: '<S30>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f;

  /* Switch: '<S30>/Switch3' */
  if (rtb_UpperRelop_ir) {
    /* Saturate: '<S30>/Saturation2' incorporates:
     *  Constant: '<S30>/Verror_Reset1'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = 0.0F;
  }

  /* End of Switch: '<S30>/Switch3' */

  /* Saturate: '<S30>/Saturation2' incorporates:
   *  Sum: '<S30>/Add2'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i += rtb_Ax;
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f = 400.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f =
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;
  }

  /* RelationalOperator: '<S84>/Compare' incorporates:
   *  UnitDelay: '<S79>/Unit Delay1'
   */
  rtb_ignition = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl;

  /* UnitDelay: '<S78>/Delay Input1'
   *
   * Block description for '<S78>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_j;

  /* RelationalOperator: '<S78>/FixPt Relational Operator' */
  rtb_UpperRelop_ir = ((int32_T)rtb_ignition > (int32_T)rtb_UpperRelop_ir);

  /* Switch: '<S30>/Switch' incorporates:
   *  Constant: '<S30>/Integr_StartPoint'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S30>/Add4' */
    rtb_deltafalllimit_m5 = MCFR_ActualTorque -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_f;
  } else {
    rtb_deltafalllimit_m5 = 0.0;
  }

  /* End of Switch: '<S30>/Switch' */

  /* Saturate: '<S30>/Saturation2' incorporates:
   *  Lookup_n-D: '<S30>/VehicleStableTarget_mps'
   *  Sum: '<S30>/Add10'
   *  Sum: '<S30>/Add5'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = look1_iflf_binlc
    (rtb_Switch2_b0, VehCtrlMdel240926_2018b__ConstP.pooled55,
     VehCtrlMdel240926_2018b__ConstP.pooled61, 3U);
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i += rtb_Switch2_b0;
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = rtb_VxIMU_est -
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* RelationalOperator: '<S30>/Relational Operator' incorporates:
   *  Constant: '<S30>/Verror'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i <
                       0.0F);

  /* Logic: '<S30>/Logical Operator4' incorporates:
   *  UnitDelay: '<S79>/Unit Delay1'
   */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir &&
                       VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl);

  /* Switch: '<S30>/Switch1' */
  if (rtb_UpperRelop_ir) {
    /* Saturate: '<S30>/Saturation2' incorporates:
     *  Constant: '<S30>/Trq_I_FF'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = 20.0F;
  } else {
    /* Saturate: '<S30>/Saturation2' incorporates:
     *  Constant: '<S30>/Trq_IReset'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = 0.0F;
  }

  /* End of Switch: '<S30>/Switch1' */

  /* Sum: '<S30>/Add6' incorporates:
   *  UnitDelay: '<S30>/Unit Delay'
   */
  rtb_Switch2_gd = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f +
                    rtb_deltafalllimit_m5) +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* Product: '<S30>/Product1' */
  rtb_UkYk1 = rtb_Switch2_gd * 10.0;

  /* RelationalOperator: '<S81>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1 > rtb_Gain4);

  /* Switch: '<S81>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Gain: '<S30>/Gain3' */
    rtb_deltafalllimit_a = -rtb_Gain3_o;

    /* RelationalOperator: '<S81>/UpperRelop' */
    rtb_LogicalOperator7 = (rtb_UkYk1 < rtb_deltafalllimit_a);

    /* Switch: '<S81>/Switch' */
    if (rtb_LogicalOperator7) {
      rtb_UkYk1 = rtb_deltafalllimit_a;
    }

    /* End of Switch: '<S81>/Switch' */
    rtb_Gain4 = rtb_UkYk1;
  }

  /* End of Switch: '<S81>/Switch2' */

  /* Sum: '<S30>/Add7' incorporates:
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE = rtb_Gain3_o + rtb_Gain4;

  /* Saturate: '<S30>/Saturation2' incorporates:
   *  Lookup_n-D: '<S30>/VehicleStableTarget_mps1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = look1_iflf_binlc
    (rtb_Switch2_b0, VehCtrlMdel240926_2018b__ConstP.pooled55,
     VehCtrlMdel240926_2018b__ConstP.pooled61, 3U);

  /* Sum: '<S30>/Add13' */
  rtb_Switch2_b0 += VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* Sum: '<S30>/Add12' */
  rtb_VxIMU_est -= rtb_Switch2_b0;

  /* RelationalOperator: '<S30>/Relational Operator1' incorporates:
   *  Constant: '<S30>/Verror1'
   */
  rtb_UpperRelop_ir = (rtb_VxIMU_est < 0.0F);

  /* RelationalOperator: '<S30>/Relational Operator2' incorporates:
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  rtb_Compare_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE >=
                   MCFR_ActualTorque);

  /* RelationalOperator: '<S77>/Compare' incorporates:
   *  Constant: '<S77>/Constant'
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE <= 0.01);

  /* Logic: '<S30>/OR' */
  rtb_Compare_b = (rtb_Compare_b || rtb_LowerRelop1_b);

  /* Logic: '<S30>/Logical Operator5' incorporates:
   *  UnitDelay: '<S30>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e = (rtb_UpperRelop_ir &&
    rtb_Compare_b);

  /* Switch: '<S30>/Switch2' incorporates:
   *  Switch: '<S30>/Switch7'
   *  UnitDelay: '<S30>/Unit Delay3'
   *  UnitDelay: '<S79>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl) {
    /* RelationalOperator: '<S82>/LowerRelop1' incorporates:
     *  Constant: '<S30>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S30>/Unit Delay4'
     */
    rtb_LogicalOperator7 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE >
      235.0);

    /* Switch: '<S82>/Switch2' incorporates:
     *  Constant: '<S30>/TCS_TrqRequest_Max2'
     */
    if (rtb_LogicalOperator7) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j = 235.0;
    } else {
      /* RelationalOperator: '<S82>/UpperRelop' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S30>/Unit Delay4'
       */
      rtb_LogicalOperator7 = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE <
        0.0);

      /* Switch: '<S82>/Switch' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S30>/Unit Delay4'
       */
      if (rtb_LogicalOperator7) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE;
      }

      /* End of Switch: '<S82>/Switch' */
    }

    /* End of Switch: '<S82>/Switch2' */

    /* RelationalOperator: '<S83>/LowerRelop1' */
    rtb_LogicalOperator7 = (MCFR_ActualTorque >
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j);

    /* Switch: '<S83>/Switch2' */
    if (!rtb_LogicalOperator7) {
      /* RelationalOperator: '<S83>/UpperRelop' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min1'
       */
      rtb_LogicalOperator7 = (MCFR_ActualTorque < 0.0);

      /* Switch: '<S83>/Switch' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min1'
       */
      if (rtb_LogicalOperator7) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j = MCFR_ActualTorque;
      }

      /* End of Switch: '<S83>/Switch' */
    }

    /* End of Switch: '<S83>/Switch2' */
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e) {
      /* Switch: '<S30>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j = MCFR_ActualTorque;
    }
  }

  /* End of Switch: '<S30>/Switch2' */

  /* Lookup_n-D: '<S7>/BrakeCompensateCoefFront1' */
  rtb_deltafalllimit_o4 = look1_iflf_binlc((real32_T)Brk_F,
    VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefFront1_bp01D,
    VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefFront1_table, 1U);

  /* Logic: '<S7>/Logical Operator5' */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amksp_B.MCFR_TorqueOn &&
                       VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueOn);

  /* Logic: '<S7>/Logical Operator6' */
  TroqueOn = !rtb_UpperRelop_ir;

  /* Logic: '<S7>/Logical Operator2' */
  rtb_UpperRelop_ir = !VehCtrlMdel240926_2018b_amksp_B.VehReady;

  /* Logic: '<S7>/Logical Operator3' */
  rtb_Compare_b = (rtb_UpperRelop_ir || (Trq_CUT != 0.0));

  /* Logic: '<S7>/Logical Operator1' */
  TrqR_cmd_raw = ((EMRAX_Trq_CUT != 0.0) || rtb_Compare_b);

  /* Logic: '<S7>/Logical Operator4' */
  Trq_CUT_final = (TroqueOn || (AMK_Trq_CUT != 0.0) || rtb_Compare_b ||
                   TrqR_cmd_raw);

  /* Switch: '<S7>/Switch3' incorporates:
   *  Constant: '<S7>/Constant6'
   *  Switch: '<S7>/Switch6'
   *  Switch: '<S7>/Switch8'
   */
  if (Trq_CUT_final) {
    rtb_deltafalllimit_a = 0.0F;
  } else {
    if (VehCtrlMdel240926_2018b_amksp_B.TCSF_Enable_OUT != 0.0) {
      /* Switch: '<S7>/Switch6' incorporates:
       *  UnitDelay: '<S30>/Unit Delay2'
       */
      rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j;
    } else {
      if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_n != 0.0) {
        /* Switch: '<S7>/Switch8' incorporates:
         *  Switch: '<S7>/Switch6'
         *  UnitDelay: '<S41>/Delay Input2'
         *
         * Block description for '<S41>/Delay Input2':
         *
         *  Store in Global RAM
         */
        rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p;
      } else {
        /* Product: '<S7>/Product' incorporates:
         *  Constant: '<S7>/Constant14'
         *  Switch: '<S7>/Switch6'
         *  Switch: '<S7>/Switch8'
         */
        rtb_UkYk1 = Acc_POS_n * 0.0346938775510204;
      }

      /* Sum: '<S28>/Add2' incorporates:
       *  Constant: '<S28>/RPM_min2'
       *  Switch: '<S7>/Switch6'
       */
      rtb_deltafalllimit_pe = MCFR_ActualVelocity + 10.0;

      /* MinMax: '<S28>/Max1' incorporates:
       *  Constant: '<S28>/RPM_min3'
       *  Switch: '<S7>/Switch6'
       */
      rtb_Yk1_l = fmax(rtb_deltafalllimit_pe, 1.0);

      /* Product: '<S28>/Product4' incorporates:
       *  Switch: '<S7>/Switch6'
       */
      rtb_deltafalllimit_pe = 7.0 * rtb_Switch2_on;

      /* Product: '<S28>/Product1' incorporates:
       *  Switch: '<S7>/Switch6'
       */
      rtb_deltafalllimit_pe *= 9550.0;

      /* Product: '<S28>/Divide1' incorporates:
       *  Switch: '<S7>/Switch6'
       */
      rtb_deltafalllimit_pe /= rtb_Yk1_l;

      /* RelationalOperator: '<S26>/LowerRelop1' incorporates:
       *  Switch: '<S7>/Switch6'
       */
      rtb_LogicalOperator7 = (rtb_UkYk1 > rtb_deltafalllimit_pe);

      /* Switch: '<S26>/Switch2' incorporates:
       *  Switch: '<S7>/Switch6'
       */
      if (rtb_LogicalOperator7) {
        rtb_deltafalllimit_a = (real32_T)rtb_deltafalllimit_pe;
      } else {
        /* RelationalOperator: '<S26>/UpperRelop' incorporates:
         *  Constant: '<S7>/Constant15'
         */
        rtb_LogicalOperator7 = (rtb_UkYk1 < 0.0);

        /* Switch: '<S26>/Switch' incorporates:
         *  Constant: '<S7>/Constant15'
         */
        if (rtb_LogicalOperator7) {
          rtb_deltafalllimit_a = 0.0F;
        } else {
          rtb_deltafalllimit_a = (real32_T)rtb_UkYk1;
        }

        /* End of Switch: '<S26>/Switch' */
      }

      /* End of Switch: '<S26>/Switch2' */

      /* Switch: '<S7>/Switch6' */
      rtb_UkYk1 = rtb_deltafalllimit_a;
    }

    /* RelationalOperator: '<S23>/LowerRelop1' incorporates:
     *  Switch: '<S7>/Switch6'
     *  Switch: '<S7>/Switch8'
     */
    rtb_LogicalOperator7 = (rtb_UkYk1 > rtb_deltafalllimit_o4);

    /* Switch: '<S23>/Switch2' */
    if (rtb_LogicalOperator7) {
      rtb_deltafalllimit_a = rtb_deltafalllimit_o4;
    } else {
      /* RelationalOperator: '<S23>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      rtb_LogicalOperator7 = (rtb_UkYk1 < 0.0);

      /* Switch: '<S23>/Switch' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      if (rtb_LogicalOperator7) {
        rtb_deltafalllimit_a = 0.0F;
      } else {
        rtb_deltafalllimit_a = (real32_T)rtb_UkYk1;
      }

      /* End of Switch: '<S23>/Switch' */
    }

    /* End of Switch: '<S23>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch3' */

  /* Saturate: '<S30>/Saturation2' incorporates:
   *  UnitDelay: '<S20>/Delay Input2'
   *
   * Block description for '<S20>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn;

  /* Sum: '<S20>/Difference Inputs1'
   *
   * Block description for '<S20>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_a -= VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* SampleTimeMath: '<S20>/sample time'
   *
   * About '<S20>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S20>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S65>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_deltafalllimit_a > rtb_Switch2_b0);

  /* Switch: '<S65>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S20>/delta fall limit' */
    rtb_Switch2_b0 = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S65>/UpperRelop' */
    rtb_LogicalOperator7 = (rtb_deltafalllimit_a < rtb_Switch2_b0);

    /* Switch: '<S65>/Switch' */
    if (rtb_LogicalOperator7) {
      rtb_deltafalllimit_a = rtb_Switch2_b0;
    }

    /* End of Switch: '<S65>/Switch' */
    rtb_Switch2_b0 = rtb_deltafalllimit_a;
  }

  /* End of Switch: '<S65>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn = rtb_Switch2_b0 +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;
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
   *  UnitDelay: '<S70>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp) {
    /* Saturate: '<S30>/Saturation2' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = FLWhlStrAng;
  } else {
    /* Saturate: '<S30>/Saturation2' incorporates:
     *  Constant: '<S29>/Verror_Reset'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = 0.0F;
  }

  /* End of Switch: '<S29>/Switch6' */

  /* Product: '<S29>/Product' incorporates:
   *  Constant: '<S29>/P_Gain'
   */
  rtb_deltafalllimit_a = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i *
    40.0F;

  /* Sum: '<S29>/Add11' */
  rtb_Gain4 = MCFL_ActualTorque - rtb_deltafalllimit_a;

  /* UnitDelay: '<S29>/Unit Delay5' */
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_ip;

  /* Product: '<S29>/Product2' */
  rtb_Switch2_b0 *= FLWhlStrAng;

  /* RelationalOperator: '<S67>/Compare' incorporates:
   *  Constant: '<S67>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 <= 0.0F);

  /* UnitDelay: '<S29>/Unit Delay' */
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr;

  /* Switch: '<S29>/Switch3' incorporates:
   *  Constant: '<S29>/Verror_Reset1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Switch: '<S29>/Switch3' */

  /* Sum: '<S29>/Add2' */
  rtb_Switch2_b0 += VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* Saturate: '<S29>/Saturation2' */
  if (rtb_Switch2_b0 > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = 400.0F;
  } else if (rtb_Switch2_b0 < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = rtb_Switch2_b0;
  }

  /* End of Saturate: '<S29>/Saturation2' */

  /* RelationalOperator: '<S75>/Compare' incorporates:
   *  UnitDelay: '<S70>/Unit Delay1'
   */
  rtb_LogicalOperator7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp;

  /* UnitDelay: '<S69>/Delay Input1'
   *
   * Block description for '<S69>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_b;

  /* RelationalOperator: '<S69>/FixPt Relational Operator' */
  rtb_UpperRelop_ir = ((int32_T)rtb_LogicalOperator7 > (int32_T)
                       rtb_UpperRelop_ir);

  /* Switch: '<S29>/Switch' incorporates:
   *  Constant: '<S29>/Integr_StartPoint'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S29>/Add4' */
    rtb_deltafalllimit_m5 = MCFL_ActualTorque -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g;
  } else {
    rtb_deltafalllimit_m5 = 0.0;
  }

  /* End of Switch: '<S29>/Switch' */

  /* Saturate: '<S30>/Saturation2' incorporates:
   *  Lookup_n-D: '<S29>/VehicleStableTarget_mps'
   *  Sum: '<S29>/Add10'
   *  Sum: '<S29>/Add5'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = look1_iflf_binlc
    (rtb_Add4_j, VehCtrlMdel240926_2018b__ConstP.pooled55,
     VehCtrlMdel240926_2018b__ConstP.pooled61, 3U);
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i += rtb_Add4_j;
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = rtb_Add6 -
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* RelationalOperator: '<S29>/Relational Operator' incorporates:
   *  Constant: '<S29>/Verror'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i <
                       0.0F);

  /* Logic: '<S29>/Logical Operator4' incorporates:
   *  UnitDelay: '<S70>/Unit Delay1'
   */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir &&
                       VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp);

  /* Switch: '<S29>/Switch1' */
  if (rtb_UpperRelop_ir) {
    /* Saturate: '<S30>/Saturation2' incorporates:
     *  Constant: '<S29>/Trq_I_FF'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = 20.0F;
  } else {
    /* Saturate: '<S30>/Saturation2' incorporates:
     *  Constant: '<S29>/Trq_IReset'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = 0.0F;
  }

  /* End of Switch: '<S29>/Switch1' */

  /* Sum: '<S29>/Add6' incorporates:
   *  UnitDelay: '<S29>/Unit Delay'
   */
  rtb_Switch2_gd = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr +
                    rtb_deltafalllimit_m5) +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* Product: '<S29>/Product1' */
  rtb_UkYk1 = rtb_Switch2_gd * 10.0;

  /* RelationalOperator: '<S72>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1 > rtb_Gain4);

  /* Switch: '<S72>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Gain: '<S29>/Gain3' */
    rtb_Switch2_b0 = -rtb_deltafalllimit_a;

    /* RelationalOperator: '<S72>/UpperRelop' */
    rtb_Compare_am = (rtb_UkYk1 < rtb_Switch2_b0);

    /* Switch: '<S72>/Switch' */
    if (rtb_Compare_am) {
      rtb_UkYk1 = rtb_Switch2_b0;
    }

    /* End of Switch: '<S72>/Switch' */
    rtb_Gain4 = rtb_UkYk1;
  }

  /* End of Switch: '<S72>/Switch2' */

  /* Sum: '<S29>/Add7' incorporates:
   *  UnitDelay: '<S29>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l = rtb_deltafalllimit_a +
    rtb_Gain4;

  /* Saturate: '<S30>/Saturation2' incorporates:
   *  Lookup_n-D: '<S29>/VehicleStableTarget_mps1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = look1_iflf_binlc
    (rtb_Add4_j, VehCtrlMdel240926_2018b__ConstP.pooled55,
     VehCtrlMdel240926_2018b__ConstP.pooled61, 3U);

  /* Sum: '<S29>/Add13' */
  rtb_Add4_j += VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* Sum: '<S29>/Add12' */
  rtb_Add6 -= rtb_Add4_j;

  /* RelationalOperator: '<S29>/Relational Operator1' incorporates:
   *  Constant: '<S29>/Verror1'
   */
  rtb_UpperRelop_ir = (rtb_Add6 < 0.0F);

  /* RelationalOperator: '<S29>/Relational Operator2' incorporates:
   *  UnitDelay: '<S29>/Unit Delay4'
   */
  rtb_Compare_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l >=
                   MCFL_ActualTorque);

  /* RelationalOperator: '<S68>/Compare' incorporates:
   *  Constant: '<S68>/Constant'
   *  UnitDelay: '<S29>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l <=
                       0.01);

  /* Logic: '<S29>/OR' */
  rtb_Compare_b = (rtb_Compare_b || rtb_LowerRelop1_b);

  /* Logic: '<S29>/Logical Operator5' incorporates:
   *  UnitDelay: '<S29>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_ip = (rtb_UpperRelop_ir &&
    rtb_Compare_b);

  /* Switch: '<S29>/Switch2' incorporates:
   *  Switch: '<S29>/Switch7'
   *  UnitDelay: '<S29>/Unit Delay3'
   *  UnitDelay: '<S70>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp) {
    /* RelationalOperator: '<S73>/LowerRelop1' incorporates:
     *  Constant: '<S29>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S29>/Unit Delay4'
     */
    rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l >
                      235.0);

    /* Switch: '<S73>/Switch2' incorporates:
     *  Constant: '<S29>/TCS_TrqRequest_Max2'
     */
    if (rtb_Compare_am) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 235.0;
    } else {
      /* RelationalOperator: '<S73>/UpperRelop' incorporates:
       *  Constant: '<S29>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S29>/Unit Delay4'
       */
      rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l <
                        0.0);

      /* Switch: '<S73>/Switch' incorporates:
       *  Constant: '<S29>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S29>/Unit Delay4'
       */
      if (rtb_Compare_am) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l;
      }

      /* End of Switch: '<S73>/Switch' */
    }

    /* End of Switch: '<S73>/Switch2' */

    /* RelationalOperator: '<S74>/LowerRelop1' */
    rtb_Compare_am = (MCFL_ActualTorque >
                      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b);

    /* Switch: '<S74>/Switch2' */
    if (!rtb_Compare_am) {
      /* RelationalOperator: '<S74>/UpperRelop' incorporates:
       *  Constant: '<S29>/TCS_TrqRequest_Min1'
       */
      rtb_Compare_am = (MCFL_ActualTorque < 0.0);

      /* Switch: '<S74>/Switch' incorporates:
       *  Constant: '<S29>/TCS_TrqRequest_Min1'
       */
      if (rtb_Compare_am) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = MCFL_ActualTorque;
      }

      /* End of Switch: '<S74>/Switch' */
    }

    /* End of Switch: '<S74>/Switch2' */
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_ip) {
      /* Switch: '<S29>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = MCFL_ActualTorque;
    }
  }

  /* End of Switch: '<S29>/Switch2' */

  /* UnitDelay: '<S40>/Delay Input2'
   *
   * Block description for '<S40>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hj;

  /* SampleTimeMath: '<S40>/sample time'
   *
   * About '<S40>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S40>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  rtb_deltafalllimit_m5 = 2000.0 * elapseTime;

  /* RelationalOperator: '<S47>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Gain5 > rtb_Add7);

  /* Switch: '<S47>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_gd = rtb_Add7;
  } else {
    /* RelationalOperator: '<S47>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    rtb_Compare_am = (rtb_Gain5 < 0.0);

    /* Switch: '<S47>/Switch' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    if (rtb_Compare_am) {
      rtb_Gain5 = 0.0;
    }

    /* End of Switch: '<S47>/Switch' */
    rtb_Switch2_gd = rtb_Gain5;
  }

  /* End of Switch: '<S47>/Switch2' */

  /* Sum: '<S10>/Add11' incorporates:
   *  UnitDelay: '<S46>/Delay Input2'
   *
   * Block description for '<S46>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = rtb_Switch2_gd +
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_lt;

  /* RelationalOperator: '<S50>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1 > rtb_Add7);

  /* Switch: '<S50>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_UkYk1 = rtb_Add7;
  } else {
    /* RelationalOperator: '<S50>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    rtb_Compare_am = (rtb_UkYk1 < 0.0);

    /* Switch: '<S50>/Switch' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    if (rtb_Compare_am) {
      rtb_UkYk1 = 0.0;
    }

    /* End of Switch: '<S50>/Switch' */
  }

  /* End of Switch: '<S50>/Switch2' */

  /* Sum: '<S40>/Difference Inputs1'
   *
   * Block description for '<S40>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 -= rtb_Gain4;

  /* RelationalOperator: '<S57>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1 > rtb_deltafalllimit_m5);

  /* Switch: '<S57>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S40>/delta fall limit' */
    rtb_Yk1_l = -2000.0 * elapseTime;

    /* RelationalOperator: '<S57>/UpperRelop' */
    rtb_Compare_am = (rtb_UkYk1 < rtb_Yk1_l);

    /* Switch: '<S57>/Switch' */
    if (rtb_Compare_am) {
      rtb_UkYk1 = rtb_Yk1_l;
    }

    /* End of Switch: '<S57>/Switch' */
    rtb_deltafalllimit_m5 = rtb_UkYk1;
  }

  /* End of Switch: '<S57>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hj = rtb_deltafalllimit_m5
    + rtb_Gain4;

  /* Switch: '<S7>/Switch4' incorporates:
   *  Constant: '<S7>/Constant8'
   *  Switch: '<S7>/Switch7'
   *  Switch: '<S7>/Switch9'
   */
  if (Trq_CUT_final) {
    rtb_deltafalllimit_o4 = 0.0F;
  } else {
    if (VehCtrlMdel240926_2018b_amksp_B.TCSF_Enable_OUT != 0.0) {
      /* Switch: '<S7>/Switch7' incorporates:
       *  UnitDelay: '<S29>/Unit Delay2'
       */
      rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b;
    } else {
      if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_n != 0.0) {
        /* Switch: '<S7>/Switch9' incorporates:
         *  Switch: '<S7>/Switch7'
         *  UnitDelay: '<S40>/Delay Input2'
         *
         * Block description for '<S40>/Delay Input2':
         *
         *  Store in Global RAM
         */
        rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hj;
      } else {
        /* Product: '<S7>/Product1' incorporates:
         *  Constant: '<S7>/Constant14'
         *  Switch: '<S7>/Switch7'
         *  Switch: '<S7>/Switch9'
         */
        rtb_UkYk1 = Acc_POS_n * 0.0346938775510204;
      }

      /* Sum: '<S28>/Add3' incorporates:
       *  Constant: '<S28>/RPM_min4'
       *  Switch: '<S7>/Switch7'
       */
      rtb_deltafalllimit_pe = MCFL_ActualVelocity + 10.0;

      /* MinMax: '<S28>/Max2' incorporates:
       *  Constant: '<S28>/RPM_min5'
       *  Switch: '<S7>/Switch7'
       */
      rtb_Yk1_l = fmax(rtb_deltafalllimit_pe, 1.0);

      /* Product: '<S28>/Product3' incorporates:
       *  Switch: '<S7>/Switch7'
       */
      rtb_deltafalllimit_pe = 7.0 * rtb_Switch2_on;

      /* Product: '<S28>/Product2' incorporates:
       *  Switch: '<S7>/Switch7'
       */
      rtb_deltafalllimit_pe *= 9550.0;

      /* Product: '<S28>/Divide2' incorporates:
       *  Switch: '<S7>/Switch7'
       */
      rtb_deltafalllimit_pe /= rtb_Yk1_l;

      /* RelationalOperator: '<S27>/LowerRelop1' incorporates:
       *  Switch: '<S7>/Switch7'
       */
      rtb_Compare_am = (rtb_UkYk1 > rtb_deltafalllimit_pe);

      /* Switch: '<S27>/Switch2' incorporates:
       *  Switch: '<S7>/Switch7'
       */
      if (rtb_Compare_am) {
        rtb_Add6 = (real32_T)rtb_deltafalllimit_pe;
      } else {
        /* RelationalOperator: '<S27>/UpperRelop' incorporates:
         *  Constant: '<S7>/Constant2'
         */
        rtb_Compare_am = (rtb_UkYk1 < 0.0);

        /* Switch: '<S27>/Switch' incorporates:
         *  Constant: '<S7>/Constant2'
         */
        if (rtb_Compare_am) {
          rtb_Add6 = 0.0F;
        } else {
          rtb_Add6 = (real32_T)rtb_UkYk1;
        }

        /* End of Switch: '<S27>/Switch' */
      }

      /* End of Switch: '<S27>/Switch2' */

      /* Switch: '<S7>/Switch7' */
      rtb_UkYk1 = rtb_Add6;
    }

    /* RelationalOperator: '<S24>/LowerRelop1' incorporates:
     *  Switch: '<S7>/Switch7'
     *  Switch: '<S7>/Switch9'
     */
    rtb_Compare_am = (rtb_UkYk1 > rtb_deltafalllimit_o4);

    /* Switch: '<S24>/Switch2' */
    if (!rtb_Compare_am) {
      /* RelationalOperator: '<S24>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      rtb_Compare_am = (rtb_UkYk1 < 0.0);

      /* Switch: '<S24>/Switch' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      if (rtb_Compare_am) {
        rtb_deltafalllimit_o4 = 0.0F;
      } else {
        rtb_deltafalllimit_o4 = (real32_T)rtb_UkYk1;
      }

      /* End of Switch: '<S24>/Switch' */
    }

    /* End of Switch: '<S24>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch4' */

  /* Saturate: '<S30>/Saturation2' incorporates:
   *  UnitDelay: '<S21>/Delay Input2'
   *
   * Block description for '<S21>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib;

  /* Sum: '<S21>/Difference Inputs1'
   *
   * Block description for '<S21>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_o4 -= VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* SampleTimeMath: '<S21>/sample time'
   *
   * About '<S21>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S21>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S66>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_deltafalllimit_o4 > rtb_Switch2_b0);

  /* Switch: '<S66>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S21>/delta fall limit' */
    rtb_Add6 = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S66>/UpperRelop' */
    rtb_Compare_am = (rtb_deltafalllimit_o4 < rtb_Add6);

    /* Switch: '<S66>/Switch' */
    if (rtb_Compare_am) {
      rtb_deltafalllimit_o4 = rtb_Add6;
    }

    /* End of Switch: '<S66>/Switch' */
    rtb_Switch2_b0 = rtb_deltafalllimit_o4;
  }

  /* End of Switch: '<S66>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib = rtb_Switch2_b0 +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib > 2.0F) {
    AMKTrqFL_cmd = 2.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib < -2.0F) {
    AMKTrqFL_cmd = -2.0F;
  } else {
    AMKTrqFL_cmd = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib;
  }

  /* End of Saturate: '<S7>/Saturation3' */

  /* Switch: '<S31>/Switch6' incorporates:
   *  UnitDelay: '<S90>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e) {
    /* Saturate: '<S30>/Saturation2' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = rtb_StrWhlAngV_c;
  } else {
    /* Saturate: '<S30>/Saturation2' incorporates:
     *  Constant: '<S31>/Verror_Reset'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = 0.0F;
  }

  /* End of Switch: '<S31>/Switch6' */

  /* Product: '<S31>/Product' incorporates:
   *  Constant: '<S31>/P_Gain'
   */
  rtb_deltafalllimit_o4 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i *
    40.0F;

  /* Sum: '<S31>/Add11' */
  rtb_Gain4 = trq - rtb_deltafalllimit_o4;

  /* UnitDelay: '<S31>/Unit Delay5' */
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_l;

  /* Product: '<S31>/Product2' */
  rtb_Switch2_b0 *= rtb_StrWhlAngV_c;

  /* RelationalOperator: '<S85>/Compare' incorporates:
   *  Constant: '<S85>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 <= 0.0F);

  /* UnitDelay: '<S31>/Unit Delay' */
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b;

  /* Switch: '<S31>/Switch3' incorporates:
   *  Constant: '<S31>/Verror_Reset1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Switch: '<S31>/Switch3' */

  /* Sum: '<S31>/Add2' */
  rtb_Switch2_b0 += VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* Saturate: '<S31>/Saturation2' */
  if (rtb_Switch2_b0 > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = 400.0F;
  } else if (rtb_Switch2_b0 < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = rtb_Switch2_b0;
  }

  /* End of Saturate: '<S31>/Saturation2' */

  /* RelationalOperator: '<S95>/Compare' incorporates:
   *  UnitDelay: '<S90>/Unit Delay1'
   */
  rtb_Compare_am = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e;

  /* UnitDelay: '<S89>/Delay Input1'
   *
   * Block description for '<S89>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_e;

  /* RelationalOperator: '<S89>/FixPt Relational Operator' */
  rtb_UpperRelop_ir = ((int32_T)rtb_Compare_am > (int32_T)rtb_UpperRelop_ir);

  /* Switch: '<S31>/Switch' incorporates:
   *  Constant: '<S31>/Integr_StartPoint'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S31>/Add4' */
    rtb_deltafalllimit_m5 = trq -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gu;
  } else {
    rtb_deltafalllimit_m5 = 0.0;
  }

  /* End of Switch: '<S31>/Switch' */

  /* Saturate: '<S30>/Saturation2' incorporates:
   *  Lookup_n-D: '<S31>/VehicleStableTarget_mps'
   *  Sum: '<S31>/Add10'
   *  Sum: '<S31>/Add5'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = look1_iflf_binlc
    (rtb_Add10, VehCtrlMdel240926_2018b__ConstP.pooled55,
     VehCtrlMdel240926_2018b__ConstP.pooled61, 3U);
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i += rtb_Add10;
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = rtb_Add -
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* RelationalOperator: '<S31>/Relational Operator' incorporates:
   *  Constant: '<S31>/Verror'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i <
                       0.0F);

  /* Logic: '<S31>/Logical Operator4' incorporates:
   *  UnitDelay: '<S90>/Unit Delay1'
   */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir &&
                       VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e);

  /* Switch: '<S31>/Switch1' */
  if (rtb_UpperRelop_ir) {
    /* Saturate: '<S30>/Saturation2' incorporates:
     *  Constant: '<S31>/Trq_I_FF'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = 20.0F;
  } else {
    /* Saturate: '<S30>/Saturation2' incorporates:
     *  Constant: '<S31>/Trq_IReset'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = 0.0F;
  }

  /* End of Switch: '<S31>/Switch1' */

  /* Sum: '<S31>/Add6' incorporates:
   *  UnitDelay: '<S31>/Unit Delay'
   */
  rtb_Switch2_gd = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b +
                    rtb_deltafalllimit_m5) +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* Product: '<S31>/Product1' */
  rtb_UkYk1 = rtb_Switch2_gd * 10.0;

  /* RelationalOperator: '<S92>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1 > rtb_Gain4);

  /* Switch: '<S92>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Gain: '<S31>/Gain3' */
    rtb_Add6 = -rtb_deltafalllimit_o4;

    /* RelationalOperator: '<S92>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_UkYk1 < rtb_Add6);

    /* Switch: '<S92>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_UkYk1 = rtb_Add6;
    }

    /* End of Switch: '<S92>/Switch' */
    rtb_Gain4 = rtb_UkYk1;
  }

  /* End of Switch: '<S92>/Switch2' */

  /* Sum: '<S31>/Add7' incorporates:
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b = rtb_deltafalllimit_o4 +
    rtb_Gain4;

  /* Saturate: '<S30>/Saturation2' incorporates:
   *  Lookup_n-D: '<S31>/VehicleStableTarget_mps1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = look1_iflf_binlc
    (rtb_Add10, VehCtrlMdel240926_2018b__ConstP.pooled55,
     VehCtrlMdel240926_2018b__ConstP.pooled61, 3U);

  /* Sum: '<S31>/Add13' */
  rtb_Add10 += VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* Sum: '<S31>/Add12' */
  rtb_Add10 = rtb_Add - rtb_Add10;

  /* RelationalOperator: '<S31>/Relational Operator1' incorporates:
   *  Constant: '<S31>/Verror1'
   */
  rtb_UpperRelop_ir = (rtb_Add10 < 0.0F);

  /* RelationalOperator: '<S31>/Relational Operator2' incorporates:
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  rtb_Compare_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b >= trq);

  /* RelationalOperator: '<S87>/Compare' incorporates:
   *  Constant: '<S87>/Constant'
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b <=
                       0.01);

  /* Logic: '<S31>/OR' */
  rtb_Compare_b = (rtb_Compare_b || rtb_LowerRelop1_b);

  /* Logic: '<S31>/Logical Operator5' */
  rtb_LowerRelop1_b = (rtb_UpperRelop_ir && rtb_Compare_b);

  /* Switch: '<S31>/Switch2' incorporates:
   *  Switch: '<S31>/Switch7'
   *  UnitDelay: '<S90>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e) {
    /* RelationalOperator: '<S93>/LowerRelop1' incorporates:
     *  Constant: '<S31>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S31>/Unit Delay4'
     */
    rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b >
                         235.0);

    /* Switch: '<S93>/Switch2' incorporates:
     *  Constant: '<S31>/TCS_TrqRequest_Max2'
     */
    if (rtb_UpperRelop_ir) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 235.0;
    } else {
      /* RelationalOperator: '<S93>/UpperRelop' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S31>/Unit Delay4'
       */
      rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b <
                           0.0);

      /* Switch: '<S93>/Switch' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S31>/Unit Delay4'
       */
      if (rtb_UpperRelop_ir) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b;
      }

      /* End of Switch: '<S93>/Switch' */
    }

    /* End of Switch: '<S93>/Switch2' */

    /* RelationalOperator: '<S94>/LowerRelop1' */
    rtb_UpperRelop_ir = (trq >
                         VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g);

    /* Switch: '<S94>/Switch2' */
    if (!rtb_UpperRelop_ir) {
      /* RelationalOperator: '<S94>/UpperRelop' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min1'
       */
      rtb_UpperRelop_ir = (trq < 0.0);

      /* Switch: '<S94>/Switch' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min1'
       */
      if (rtb_UpperRelop_ir) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 0.0;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = trq;
      }

      /* End of Switch: '<S94>/Switch' */
    }

    /* End of Switch: '<S94>/Switch2' */
  } else {
    if (rtb_LowerRelop1_b) {
      /* Switch: '<S31>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = trq;
    }
  }

  /* End of Switch: '<S31>/Switch2' */

  /* UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pd;

  /* SampleTimeMath: '<S42>/sample time'
   *
   * About '<S42>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S42>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  rtb_deltafalllimit_m5 = 2000.0 * elapseTime;

  /* RelationalOperator: '<S49>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_g_mpss1 > rtb_Switch2_mn);

  /* Switch: '<S49>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_gd = rtb_Switch2_mn;
  } else {
    /* RelationalOperator: '<S49>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    rtb_UpperRelop_ir = (rtb_g_mpss1 < 0.0);

    /* Switch: '<S49>/Switch' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    if (rtb_UpperRelop_ir) {
      rtb_g_mpss1 = 0.0;
    }

    /* End of Switch: '<S49>/Switch' */
    rtb_Switch2_gd = rtb_g_mpss1;
  }

  /* End of Switch: '<S49>/Switch2' */

  /* UnitDelay: '<S45>/Delay Input2'
   *
   * Block description for '<S45>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_h = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_nk;

  /* SampleTimeMath: '<S45>/sample time'
   *
   * About '<S45>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime_0 = (real_T)elapsedTicks * 0.01;

  /* Product: '<S45>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant45'
   */
  rtb_Switch2_bp = 1000.0 * elapseTime_0;

  /* Logic: '<S10>/AND1' */
  rtb_UpperRelop_ir = (rtb_FixPtRelationalOperator && rtb_Compare);

  /* Logic: '<S10>/OR1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i ||
                       rtb_UpperRelop_ir);

  /* Switch: '<S10>/Switch2' incorporates:
   *  Constant: '<S10>/Constant32'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Gain19 = 0.0;
  } else {
    /* Logic: '<S10>/NOT1' */
    rtb_FixPtRelationalOperator = !rtb_Compare;

    /* Switch: '<S10>/Switch5' incorporates:
     *  Constant: '<S10>/Constant33'
     */
    if (!rtb_FixPtRelationalOperator) {
      WhlSpdRL_mps = 0.0;
    }

    /* End of Switch: '<S10>/Switch5' */
    rtb_Gain19 = WhlSpdRL_mps;
  }

  /* End of Switch: '<S10>/Switch2' */

  /* Gain: '<S10>/Gain19' */
  rtb_Gain19 = -rtb_Gain19;

  /* Saturate: '<S10>/Saturation1' */
  if (rtb_Gain19 > 100.0) {
    rtb_Gain19 = 100.0;
  } else {
    if (rtb_Gain19 < 0.0) {
      rtb_Gain19 = 0.0;
    }
  }

  /* End of Saturate: '<S10>/Saturation1' */

  /* Sum: '<S45>/Difference Inputs1'
   *
   * Block description for '<S45>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = rtb_Gain19 - rtb_Yk1_h;

  /* RelationalOperator: '<S62>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1 > rtb_Switch2_bp);

  /* Switch: '<S62>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S45>/delta fall limit' */
    rtb_Yk1_l = -1000.0 * elapseTime_0;

    /* RelationalOperator: '<S62>/UpperRelop' */
    rtb_FixPtRelationalOperator = (rtb_UkYk1 < rtb_Yk1_l);

    /* Switch: '<S62>/Switch' */
    if (rtb_FixPtRelationalOperator) {
      rtb_UkYk1 = rtb_Yk1_l;
    }

    /* End of Switch: '<S62>/Switch' */
    rtb_Switch2_bp = rtb_UkYk1;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_nk = rtb_Switch2_bp +
    rtb_Yk1_h;

  /* Sum: '<S10>/Add13' incorporates:
   *  UnitDelay: '<S45>/Delay Input2'
   *
   * Block description for '<S45>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = rtb_Switch2_gd +
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_nk;

  /* RelationalOperator: '<S52>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1 > rtb_Switch2_mn);

  /* Switch: '<S52>/Switch2' */
  if (rtb_UpperRelop_ir) {
    rtb_UkYk1 = rtb_Switch2_mn;
  } else {
    /* RelationalOperator: '<S52>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    rtb_FixPtRelationalOperator = (rtb_UkYk1 < 0.0);

    /* Switch: '<S52>/Switch' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    if (rtb_FixPtRelationalOperator) {
      rtb_UkYk1 = 0.0;
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
  rtb_UkYk1 -= rtb_Gain4;

  /* RelationalOperator: '<S59>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_UkYk1 > rtb_deltafalllimit_m5);

  /* Switch: '<S59>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S42>/delta fall limit' */
    rtb_Yk1_l = -2000.0 * elapseTime;

    /* RelationalOperator: '<S59>/UpperRelop' */
    rtb_FixPtRelationalOperator = (rtb_UkYk1 < rtb_Yk1_l);

    /* Switch: '<S59>/Switch' */
    if (rtb_FixPtRelationalOperator) {
      rtb_UkYk1 = rtb_Yk1_l;
    }

    /* End of Switch: '<S59>/Switch' */
    rtb_deltafalllimit_m5 = rtb_UkYk1;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pd = rtb_deltafalllimit_m5
    + rtb_Gain4;

  /* Switch: '<S7>/Switch2' incorporates:
   *  Constant: '<S7>/Constant4'
   *  Switch: '<S7>/Switch1'
   *  Switch: '<S7>/Switch5'
   */
  if (TrqR_cmd_raw) {
    rtb_Add = 0.0F;
  } else {
    if (VehCtrlMdel240926_2018b_amksp_B.TCSR_Enable_OUT != 0.0) {
      /* Abs: '<S31>/Abs' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      Acc_POS_n = fabsf(rtb_APP_POS1);

      /* RelationalOperator: '<S86>/Compare' incorporates:
       *  Constant: '<S86>/Constant'
       *  Switch: '<S7>/Switch5'
       */
      rtb_FixPtRelationalOperator = (Acc_POS_n <= 20.0F);

      /* RelationalOperator: '<S88>/Compare' incorporates:
       *  Constant: '<S88>/Constant'
       *  Switch: '<S7>/Switch5'
       */
      rtb_Compare = (rtb_Add > 0.0F);

      /* Logic: '<S31>/Logical Operator6' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_Compare = (rtb_Compare && rtb_FixPtRelationalOperator);

      /* Switch: '<S31>/Switch5' incorporates:
       *  Switch: '<S7>/Switch5'
       *  UnitDelay: '<S31>/Unit Delay2'
       */
      if (rtb_Compare) {
        rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g;
      } else {
        rtb_UkYk1 = trq;
      }

      /* End of Switch: '<S31>/Switch5' */
    } else {
      if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_n != 0.0) {
        /* Switch: '<S7>/Switch1' incorporates:
         *  Switch: '<S7>/Switch5'
         *  UnitDelay: '<S42>/Delay Input2'
         *
         * Block description for '<S42>/Delay Input2':
         *
         *  Store in Global RAM
         */
        rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pd;
      } else {
        /* Product: '<S7>/Product2' incorporates:
         *  Constant: '<S7>/Constant16'
         *  Switch: '<S7>/Switch1'
         *  Switch: '<S7>/Switch5'
         */
        rtb_UkYk1 = Acc_POS_n * 0.93061224489795935;
      }

      /* Sum: '<S28>/Add1' incorporates:
       *  Constant: '<S28>/RPM_min'
       *  Switch: '<S7>/Switch5'
       */
      rtb_deltafalllimit_pe = RPM + 10.0;

      /* MinMax: '<S28>/Max' incorporates:
       *  Constant: '<S28>/RPM_min1'
       *  Switch: '<S7>/Switch5'
       */
      rtb_Yk1_l = fmax(rtb_deltafalllimit_pe, 1.0);

      /* Sum: '<S28>/Add' incorporates:
       *  Constant: '<S28>/Constant3'
       *  Switch: '<S7>/Switch5'
       */
      rtb_deltafalllimit_pe = 1.0 - rtb_deltafalllimit_le;

      /* Product: '<S28>/Product5' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_deltafalllimit_le = 63.0 * rtb_deltafalllimit_pe;

      /* Product: '<S28>/Product' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_deltafalllimit_pe = rtb_deltafalllimit_le * 9550.0;

      /* Product: '<S28>/Divide' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_deltafalllimit_pe /= rtb_Yk1_l;

      /* RelationalOperator: '<S25>/LowerRelop1' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      rtb_FixPtRelationalOperator = (rtb_UkYk1 > rtb_deltafalllimit_pe);

      /* Switch: '<S25>/Switch2' incorporates:
       *  Switch: '<S7>/Switch5'
       */
      if (rtb_FixPtRelationalOperator) {
        rtb_Add = (real32_T)rtb_deltafalllimit_pe;
      } else {
        /* RelationalOperator: '<S25>/UpperRelop' incorporates:
         *  Constant: '<S7>/Constant1'
         */
        rtb_FixPtRelationalOperator = (rtb_UkYk1 < 0.0);

        /* Switch: '<S25>/Switch' incorporates:
         *  Constant: '<S7>/Constant1'
         */
        if (rtb_FixPtRelationalOperator) {
          rtb_Add = 0.0F;
        } else {
          rtb_Add = (real32_T)rtb_UkYk1;
        }

        /* End of Switch: '<S25>/Switch' */
      }

      /* End of Switch: '<S25>/Switch2' */

      /* Switch: '<S7>/Switch5' */
      rtb_UkYk1 = rtb_Add;
    }

    /* Gain: '<S7>/Gain6' incorporates:
     *  Switch: '<S7>/Switch1'
     *  Switch: '<S7>/Switch5'
     */
    rtb_UkYk1 *= 12.5;

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefRear' */
    rtb_Add = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefRear_bp01Dat,
      VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefRear_tableDa, 1U);

    /* RelationalOperator: '<S22>/LowerRelop1' */
    rtb_FixPtRelationalOperator = (rtb_UkYk1 > rtb_Add);

    /* Switch: '<S22>/Switch2' */
    if (!rtb_FixPtRelationalOperator) {
      /* RelationalOperator: '<S22>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      rtb_FixPtRelationalOperator = (rtb_UkYk1 < 0.0);

      /* Switch: '<S22>/Switch' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      if (rtb_FixPtRelationalOperator) {
        rtb_Add = 0.0F;
      } else {
        rtb_Add = (real32_T)rtb_UkYk1;
      }

      /* End of Switch: '<S22>/Switch' */
    }

    /* End of Switch: '<S22>/Switch2' */
  }

  /* End of Switch: '<S7>/Switch2' */

  /* Saturate: '<S30>/Saturation2' incorporates:
   *  UnitDelay: '<S19>/Delay Input2'
   *
   * Block description for '<S19>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd;

  /* Sum: '<S19>/Difference Inputs1'
   *
   * Block description for '<S19>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add -= VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* SampleTimeMath: '<S19>/sample time'
   *
   * About '<S19>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;

  /* Product: '<S19>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(25000.0 * elapseTime);

  /* RelationalOperator: '<S64>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add > rtb_Switch2_b0);

  /* Switch: '<S64>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S19>/delta fall limit' */
    Acc_POS_n = (real32_T)(-25000.0 * elapseTime);

    /* RelationalOperator: '<S64>/UpperRelop' */
    rtb_FixPtRelationalOperator = (rtb_Add < Acc_POS_n);

    /* Switch: '<S64>/Switch' */
    if (rtb_FixPtRelationalOperator) {
      rtb_Add = Acc_POS_n;
    }

    /* End of Switch: '<S64>/Switch' */
    rtb_Switch2_b0 = rtb_Add;
  }

  /* End of Switch: '<S64>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd = rtb_Switch2_b0 +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd > 1000.0F) {
    EmraxTrqR_cmd = 1000.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd < 0.0F) {
    EmraxTrqR_cmd = 0.0F;
  } else {
    EmraxTrqR_cmd = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd;
  }

  /* End of Saturate: '<S7>/Saturation1' */

  /* Sum: '<S10>/Add3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_c = WhlSpdFR - WhlSpdRR_mps;

  /* SampleTimeMath: '<S7>/Weighted Sample Time1'
   *
   * About '<S7>/Weighted Sample Time1':
   *  y = u * K where K = ( w * Ts )
   */
  elapseTime = (real_T)elapsedTicks * 0.01;
  WhlSpdFR = 420000.0 * elapseTime;

  /* Sum: '<S7>/Add2' */
  WhlSpdFR += RPM;

  /* Saturate: '<S7>/RPM_Saturation1' */
  if (WhlSpdFR > 5000.0) {
    WhlSpdFR = 5000.0;
  } else {
    if (WhlSpdFR < -50.0) {
      WhlSpdFR = -50.0;
    }
  }

  /* End of Saturate: '<S7>/RPM_Saturation1' */

  /* Update for UnitDelay: '<S7>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_n =
    VehCtrlMdel240926_2018b_amksp_B.DYC_Enable_OUT;

  /* Update for UnitDelay: '<S10>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jp = rtb_APP_POS1;

  /* Update for UnitDelay: '<S10>/Unit Delay6' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE_b =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i;

  /* Update for UnitDelay: '<S31>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_a = rtb_LowerRelop1_b;

  /* Update for Saturate: '<S30>/Saturation2' incorporates:
   *  UnitDelay: '<S30>/Unit Delay5'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = rtb_deltafalllimit_iz;

  /* Update for UnitDelay: '<S30>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_f = rtb_Gain3_o;

  /* Update for UnitDelay: '<S78>/Delay Input1'
   *
   * Block description for '<S78>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_j = rtb_ignition;

  /* Update for UnitDelay: '<S29>/Unit Delay5' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_ip = FLWhlStrAng;

  /* Update for UnitDelay: '<S29>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g = rtb_deltafalllimit_a;

  /* Update for UnitDelay: '<S69>/Delay Input1'
   *
   * Block description for '<S69>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_b = rtb_LogicalOperator7;

  /* Update for UnitDelay: '<S31>/Unit Delay5' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_l = rtb_StrWhlAngV_c;

  /* Update for UnitDelay: '<S31>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gu = rtb_deltafalllimit_o4;

  /* Update for UnitDelay: '<S89>/Delay Input1'
   *
   * Block description for '<S89>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_e = rtb_Compare_am;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/10ms1' */

  /* S-Function (fcncallgen): '<S5>/10ms2' incorporates:
   *  SubSystem: '<S5>/VCU2AMKMCUFL'
   */
  /* Switch: '<S347>/Switch1' incorporates:
   *  Constant: '<S347>/Constant1'
   *  Constant: '<S347>/Constant2'
   *  Switch: '<S347>/Switch'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueOn) {
    VehCtrlMdel240926_2018b_amksp_B.Switch1_l = -21.0;
    VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueLimitP = 21.0;
  } else {
    VehCtrlMdel240926_2018b_amksp_B.Switch1_l = 0.0;
    VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueLimitP = 0.0;
  }

  /* End of Switch: '<S347>/Switch1' */

  /* S-Function (scanpack): '<S347>/CAN Pack1' */
  /* S-Function (scanpack): '<S347>/CAN Pack1' */
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

  /* S-Function (ecucoder_canmessage): '<S347>/CANPackMessage' */

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

  /* S-Function (ec5744_cantransmitslb): '<S347>/CANTransmit' */

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
  /* Switch: '<S348>/Switch1' incorporates:
   *  Constant: '<S348>/Constant'
   *  Constant: '<S348>/Constant1'
   *  Switch: '<S348>/Switch'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.MCFR_TorqueOn) {
    VehCtrlMdel240926_2018b_amksp_B.Switch1 = -21.0;
    VehCtrlMdel240926_2018b_amksp_B.Switch = 21.0;
  } else {
    VehCtrlMdel240926_2018b_amksp_B.Switch1 = 0.0;
    VehCtrlMdel240926_2018b_amksp_B.Switch = 0.0;
  }

  /* End of Switch: '<S348>/Switch1' */

  /* S-Function (scanpack): '<S348>/CAN Pack1' */
  /* S-Function (scanpack): '<S348>/CAN Pack1' */
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

  /* S-Function (ecucoder_canmessage): '<S348>/CANPackMessage' */

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

  /* S-Function (ec5744_cantransmitslb): '<S348>/CANTransmit' */

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
  /* Switch: '<S349>/Switch2' incorporates:
   *  Constant: '<S349>/Constant13'
   *  Constant: '<S349>/Constant17'
   *  Constant: '<S349>/Constant19'
   *  Constant: '<S349>/Constant20'
   *  Switch: '<S349>/Switch3'
   */
  if (TrqR_cmd_raw) {
    Gear_Trs = 0.0;
    Mode_Trs = 0.0;
  } else {
    Gear_Trs = 2.0;
    Mode_Trs = 2.0;
  }

  /* End of Switch: '<S349>/Switch2' */

  /* DataTypeConversion: '<S349>/Cast To Boolean4' */
  VehCtrlMdel240926_2018b_amksp_B.CastToBoolean4 = (uint16_T)Gear_Trs;

  /* DataTypeConversion: '<S349>/Cast To Boolean6' */
  VehCtrlMdel240926_2018b_amksp_B.CastToBoolean6 = (uint16_T)Mode_Trs;

  /* DataTypeConversion: '<S349>/Data Type Conversion2' */
  VehCtrlMdel240926_2018b_amksp_B.DataTypeConversion2 = (int32_T)floor(WhlSpdFR);

  /* S-Function (scanpack): '<S349>/CAN Pack1' */
  /* S-Function (scanpack): '<S349>/CAN Pack1' */
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

  /* S-Function (ecucoder_canmessage): '<S349>/CANPackMessage' */

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

  /* S-Function (ec5744_cantransmitslb): '<S349>/CANTransmit' */

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
  /* DataTypeConversion: '<S350>/Cast To Single1' */
  VehCtrlMdel240926_2018b_amksp_B.CastToSingle1 = (uint16_T)rtb_CastToDouble;

  /* S-Function (ec5744_pdsslbu3): '<S350>/PowerDriverSwitch(HS)' */

  /* Set level VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON for the specified power driver switch */
  ec_gpio_write(83,VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON);

  /* S-Function (ec5744_pdsslbu3): '<S350>/PowerDriverSwitch(HS)1' */

  /* Set level VehCtrlMdel240926_2018b_amksp_B.bWaterPumpON for the specified power driver switch */
  ec_gpio_write(55,VehCtrlMdel240926_2018b_amksp_B.bWaterPumpON);

  /* S-Function (ec5744_pdpslbu3): '<S350>/PowerDriverPWM' incorporates:
   *  Constant: '<S350>/Constant'
   */

  /* Power driver PWM output for channel 6 */
  ec_pwm_output(6,((uint16_T)1000U),
                VehCtrlMdel240926_2018b_amksp_B.CastToSingle1);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms6' */

  /* S-Function (fcncallgen): '<S357>/10ms' incorporates:
   *  SubSystem: '<S357>/daq10ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S369>/CCPDAQ' */
  ccpDaq(1);

  /* End of Outputs for S-Function (fcncallgen): '<S357>/10ms' */

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
  /* S-Function (fcncallgen): '<S357>/50ms' incorporates:
   *  SubSystem: '<S357>/daq50ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S371>/CCPDAQ' */
  ccpDaq(2);

  /* End of Outputs for S-Function (fcncallgen): '<S357>/50ms' */
}

/* Model step function for TID5 */
void VehCtrlMdel240926_2018b_amkspdlimit_step5(void) /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S356>/100MS' incorporates:
   *  SubSystem: '<S356>/Function-Call Subsystem'
   */
  /* S-Function (ec5744_canreceiveslb): '<S360>/CANReceive' */

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

  /* Call the system: <S360>/Function-Call Subsystem */

  /* Output and update for function-call system: '<S360>/Function-Call Subsystem' */
  {
    uint8_T rtb_Add;
    uint8_T rtb_Compare;

    /* Outputs for Enabled SubSystem: '<S361>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S362>/Enable'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.CANReceive_o2_p > 0) {
      /* RelationalOperator: '<S363>/Compare' incorporates:
       *  Constant: '<S363>/Constant'
       */
      rtb_Add = (uint8_T)(VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[0] ==
                          83);

      /* RelationalOperator: '<S364>/Compare' incorporates:
       *  Constant: '<S364>/Constant'
       */
      rtb_Compare = (uint8_T)(VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[5]
        == 84);

      /* Sum: '<S362>/Add' */
      rtb_Add = (uint8_T)((uint32_T)rtb_Add + rtb_Compare);

      /* RelationalOperator: '<S365>/Compare' incorporates:
       *  Constant: '<S365>/Constant'
       */
      rtb_Compare = (uint8_T)(rtb_Add == 2);

      /* If: '<S362>/If' */
      if (rtb_Compare > 0) {
        /* Outputs for IfAction SubSystem: '<S362>/If Action Subsystem' incorporates:
         *  ActionPort: '<S366>/Action Port'
         */
        /* S-Function (ec5744_bootloaderslb): '<S366>/BootLoader' */
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

        /* S-Function (ec5744_cpuresetslb): '<S366>/CPUReset' */

        /* Perform a microcontroller reset */
        MC_ME.MCTL.R = 0X00005AF0;
        MC_ME.MCTL.R = 0X0000A50F;

        /* End of Outputs for SubSystem: '<S362>/If Action Subsystem' */
      } else {
        /* Outputs for IfAction SubSystem: '<S362>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S367>/Action Port'
         */
        /* S-Function (ec5744_cantransmitslb): '<S367>/CANTransmit' incorporates:
         *  Constant: '<S367>/Constant'
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

        /* End of Outputs for SubSystem: '<S362>/If Action Subsystem1' */
      }

      /* End of If: '<S362>/If' */
    }

    /* End of Outputs for SubSystem: '<S361>/Enabled Subsystem' */
  }

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S360>/CANReceive' */
  /* End of Outputs for S-Function (fcncallgen): '<S356>/100MS' */

  /* S-Function (fcncallgen): '<S357>/100ms' incorporates:
   *  SubSystem: '<S357>/daq100ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S368>/CCPDAQ' */
  ccpDaq(3);

  /* End of Outputs for S-Function (fcncallgen): '<S357>/100ms' */
}

/* Model step function for TID6 */
void VehCtrlMdel240926_2018b_amkspdlimit_step6(void) /* Sample time: [0.5s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S357>/500ms' incorporates:
   *  SubSystem: '<S357>/daq500ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S370>/CCPDAQ' */
  ccpDaq(4);

  /* End of Outputs for S-Function (fcncallgen): '<S357>/500ms' */

  /* S-Function (fcncallgen): '<S358>/500ms' incorporates:
   *  SubSystem: '<S358>/EEPROMOperation'
   */

  /* S-Function (ec5744_eepromoslb): '<S373>/EEPROMOperatin' */
#if defined EC_EEPROM_ENABLE

  /* Operate the EEPROM module on the MPC5744 */
  ec_flash_operation();

#endif

  /* End of Outputs for S-Function (fcncallgen): '<S358>/500ms' */
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
  /* Start for S-Function (ec5744_canreceiveslb): '<S120>/CANReceive1' incorporates:
   *  SubSystem: '<S120>/MCU_pwr'
   */
  /* Start for function-call system: '<S120>/MCU_pwr' */

  /* Start for Enabled SubSystem: '<S175>/MCU_VCUMeter1' */

  /* Start for S-Function (scanunpack): '<S177>/CAN Unpack' */

  /*-----------S-Function Block: <S177>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S175>/MCU_VCUMeter1' */
  ec_buffer_init(0,1,1,218089455);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S120>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S120>/CANReceive3' incorporates:
   *  SubSystem: '<S120>/MCU_state'
   */
  /* Start for function-call system: '<S120>/MCU_state' */

  /* Start for Enabled SubSystem: '<S176>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S183>/CAN Unpack' */

  /*-----------S-Function Block: <S183>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S176>/MCU_state' */
  ec_buffer_init(0,0,1,218089199);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S120>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S116>/CANReceive3' incorporates:
   *  SubSystem: '<S116>/ABS_BUS_state'
   */
  /* Start for function-call system: '<S116>/ABS_BUS_state' */

  /* Start for Enabled SubSystem: '<S124>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S125>/CAN Unpack1' */

  /*-----------S-Function Block: <S125>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S124>/IMU_state' */
  ec_buffer_init(1,16,0,1698);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S116>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' incorporates:
   *  SubSystem: '<S122>/StrWhSnis_state'
   */
  /* Start for function-call system: '<S122>/StrWhSnis_state' */

  /* Start for Enabled SubSystem: '<S195>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S196>/CAN Unpack1' */

  /*-----------S-Function Block: <S196>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S195>/IMU_state' */
  ec_buffer_init(1,7,0,330);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms4' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive3' incorporates:
   *  SubSystem: '<S130>/AMKMCU_state'
   */
  /* Start for function-call system: '<S130>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S132>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S135>/CAN Unpack' */

  /*-----------S-Function Block: <S135>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S132>/MCU_state' */
  ec_buffer_init(1,1,0,640);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive1' incorporates:
   *  SubSystem: '<S130>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S130>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S133>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S145>/CAN Unpack' */

  /*-----------S-Function Block: <S145>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S133>/MCU_state' */
  ec_buffer_init(1,2,0,642);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive2' incorporates:
   *  SubSystem: '<S130>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S130>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S134>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S147>/CAN Unpack' */

  /*-----------S-Function Block: <S147>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S134>/MCU_state' */
  ec_buffer_init(1,3,0,644);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S130>/CANReceive2' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S131>/CANReceive3' incorporates:
   *  SubSystem: '<S131>/AMKMCU_state'
   */
  /* Start for function-call system: '<S131>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S151>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S154>/CAN Unpack' */

  /*-----------S-Function Block: <S154>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S151>/MCU_state' */
  ec_buffer_init(1,4,0,641);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S131>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S131>/CANReceive1' incorporates:
   *  SubSystem: '<S131>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S131>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S152>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S163>/CAN Unpack' */

  /*-----------S-Function Block: <S163>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S152>/MCU_state' */
  ec_buffer_init(1,5,0,643);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S131>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S131>/CANReceive2' incorporates:
   *  SubSystem: '<S131>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S131>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S153>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S165>/CAN Unpack' */

  /*-----------S-Function Block: <S165>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S153>/MCU_state' */
  ec_buffer_init(1,0,0,645);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S131>/CANReceive2' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms5' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' incorporates:
   *  SubSystem: '<S121>/IMU_state'
   */
  /* Start for function-call system: '<S121>/IMU_state' */

  /* Start for Enabled SubSystem: '<S190>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S191>/CAN Unpack' */

  /*-----------S-Function Block: <S191>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S190>/MCU_state' */
  ec_buffer_init(1,17,0,513);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S121>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms2' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S119>/CANReceive3' */
  ec_buffer_init(0,3,1,408961267);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S119>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S347>/CANTransmit' */
  ec_buffer_init(1,8,0,386U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms2' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S348>/CANTransmit' */
  ec_buffer_init(1,9,0,387U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms4' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S349>/CANTransmit' */
  ec_buffer_init(0,8,1,146927393U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S5>/10ms6' incorporates:
   *  SubSystem: '<S5>/WP_OUTPUT'
   */
  /* Start for S-Function (ec5744_pdpslbu3): '<S350>/PowerDriverPWM' incorporates:
   *  Constant: '<S350>/Constant'
   */

  /* Initialize PWM output for channel 6 */
  SIUL2_MSCR42 = 0X02000003;           //PWM_A3

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S356>/100MS' incorporates:
   *  SubSystem: '<S356>/Function-Call Subsystem'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S360>/CANReceive' incorporates:
   *  SubSystem: '<S360>/Function-Call Subsystem'
   */
  /* Start for function-call system: '<S360>/Function-Call Subsystem' */

  /* Start for Enabled SubSystem: '<S361>/Enabled Subsystem' */
  /* Start for IfAction SubSystem: '<S362>/If Action Subsystem1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S367>/CANTransmit' incorporates:
   *  Constant: '<S367>/Constant'
   */
  ec_buffer_init(2,9,0,593U);

  /* End of Start for SubSystem: '<S362>/If Action Subsystem1' */
  /* End of Start for SubSystem: '<S361>/Enabled Subsystem' */
  ec_buffer_init(2,1,0,278);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S360>/CANReceive' */
  /* End of Start for S-Function (fcncallgen): '<S356>/100MS' */

  /* Start for S-Function (fcncallgen): '<S359>/Function-Call Generator' incorporates:
   *  SubSystem: '<S359>/CCPBackground'
   */
  /* Start for S-Function (ec5744_ccpslb): '<S374>/CCPBackground' */
  ccpInit();

  /* End of Start for S-Function (fcncallgen): '<S359>/Function-Call Generator' */

  /* Start for S-Function (ec5744_caninterruptslb1): '<S359>/ReceiveandTransmitInterrupt' incorporates:
   *  SubSystem: '<S359>/CCPReceive'
   */
  /* Start for function-call system: '<S359>/CCPReceive' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S375>/CANReceive' */
  ec_buffer_init(2,0,0,CCP_CRO_ID);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S375>/CANReceive' */
  ec_bufint_init(2,0);
  INTC_0.PSR[548].B.PRIN = 12;
  IntcIsrVectorTable[548] = (uint32_t)&ISR_FlexCAN_2_MB0;

  /* End of Start for S-Function (ec5744_caninterruptslb1): '<S359>/ReceiveandTransmitInterrupt' */

  /* Start for S-Function (ec5744_eeprombsbu3): '<Root>/EEPROMEnable' */
  Fls_Read(0xFA0010,ecflashdataold,4096);
  Fls_Read(0xFA0010,ecflashdatanew,4096);

  /* SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  /* InitializeConditions for UnitDelay: '<S276>/Unit Delay4' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn = 0.01F;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' */

  /* SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* SystemInitialize for Chart: '<S106>/Chart2' */
  VehCtrlMdel240926_2018b_amks_DW.sfEvent = -1;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S3>/10ms7' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* Enable for Chart: '<S123>/Chart' */
  VehCtrlMdel240926_2018b_amks_DW.previousTicks =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;

  /* End of Enable for S-Function (fcncallgen): '<S3>/10ms7' */

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
  /* Enable for Chart: '<S106>/Chart2' */
  VehCtrlMdel240926_2018b_amks_DW.previousTicks_g =
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
