/*
 * Code generated for Simulink model VehCtrlMdel240926_2018b_amkspdlimit.
 *
 * FILE    : VehCtrlMdel240926_2018b_amkspdlimit.c
 *
 * VERSION : 1.275
 *
 * DATE    : Thu Oct 17 13:17:54 2024
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

/* Named constants for Chart: '<S126>/Timer' */
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

/* Named constants for Chart: '<S109>/Chart2' */
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

/* Named constants for Chart: '<S126>/Timer1' */
#define VehCtrlMdel240926_2018_IN_Out_b ((uint8_T)2U)
#define VehCtrlMdel240926__IN_Trigger_e ((uint8_T)3U)
#define VehCtrlMdel2409_IN_InterState_d ((uint8_T)1U)

/* Named constants for Chart: '<S344>/Chart' */
#define VehCtrlMdel240926_2018b_IN_ON_h ((uint8_T)1U)
#define VehCtrlMdel240926_20_IN_STATEON ((uint8_T)2U)
#define VehCtrlMdel240926_2_IN_STATEOFF ((uint8_T)1U)
#define VehCtrlMdel240926_IN_initstate1 ((uint8_T)2U)
#define VehCtrlMdel240926__IN_initstate ((uint8_T)2U)

/* Named constants for Chart: '<S354>/Chart' */
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

EE_Data ecflashdataold[1024];
EE_Data ecflashdatanew[1024];

/* Exported block signals */
real_T Gear_Trs;                       /* '<S347>/Switch2' */
real_T Mode_Trs;                       /* '<S347>/Switch3' */
real_T KeyPressed;     /* '<S207>/BusConversion_InsertedFor_Out1_at_inport_0' */
real_T AMKFL_Current;                  /* '<S212>/Switch' */
real_T AMKFR_Current;                  /* '<S212>/Switch1' */
real_T Trq_CUT;                        /* '<S209>/Timer' */
real_T AMKSWITCH;                      /* '<S126>/Timer1' */
real_T ignition;                       /* '<S126>/Timer' */
real_T L12V_error;                     /* '<S186>/CAN Unpack' */
real_T alarm;                          /* '<S186>/CAN Unpack' */
real_T controller_ready;               /* '<S186>/CAN Unpack' */
real_T selfcheck;                      /* '<S186>/CAN Unpack' */
real_T RPM;                            /* '<S186>/CAN Unpack' */
real_T trq;                            /* '<S186>/CAN Unpack' */
real_T AC_current;                     /* '<S180>/CAN Unpack' */
real_T DC_current;                     /* '<S180>/CAN Unpack' */
real_T MCU_Temp;                       /* '<S180>/CAN Unpack' */
real_T motor_Temp;                     /* '<S180>/CAN Unpack' */
real_T voltage;                        /* '<S180>/CAN Unpack' */
real_T MCFR_ActualTorque;              /* '<S157>/CAN Unpack' */
real_T MCFR_ActualVelocity;            /* '<S157>/CAN Unpack' */
real_T MCFR_DCVoltage;                 /* '<S157>/CAN Unpack' */
real_T MCFR_bDCOn;                     /* '<S157>/CAN Unpack' */
real_T MCFR_bError;                    /* '<S157>/CAN Unpack' */
real_T MCFR_bInverterOn;               /* '<S157>/CAN Unpack' */
real_T MCFR_bQuitInverterOn;           /* '<S157>/CAN Unpack' */
real_T MCFR_bSystemReady;              /* '<S157>/CAN Unpack' */
real_T MCFR_TempIGBT;                  /* '<S168>/CAN Unpack' */
real_T MCFR_TempInverter;              /* '<S168>/CAN Unpack' */
real_T MCFR_TempMotor;                 /* '<S168>/CAN Unpack' */
real_T MCFR_ErrorInfo;                 /* '<S166>/CAN Unpack' */
real_T MCFL_ActualTorque;              /* '<S138>/CAN Unpack' */
real_T MCFL_ActualVelocity;            /* '<S138>/CAN Unpack' */
real_T MCFL_DCVoltage;                 /* '<S138>/CAN Unpack' */
real_T MCFL_bDCOn;                     /* '<S138>/CAN Unpack' */
real_T MCFL_bError;                    /* '<S138>/CAN Unpack' */
real_T MCFL_bInverterOn;               /* '<S138>/CAN Unpack' */
real_T MCFL_bQuitDCOn;                 /* '<S138>/CAN Unpack' */
real_T MCFL_bQuitInverterOn;           /* '<S138>/CAN Unpack' */
real_T MCFL_bSystemReady;              /* '<S138>/CAN Unpack' */
real_T MCFL_TempIGBT;                  /* '<S150>/CAN Unpack' */
real_T MCFL_TempInverter;              /* '<S150>/CAN Unpack' */
real_T MCFL_TempMotor;                 /* '<S150>/CAN Unpack' */
real_T MCFL_ErrorInfo;                 /* '<S148>/CAN Unpack' */
real_T StrWhlAngAliveRollCnt;          /* '<S199>/CAN Unpack1' */
real_T StrWhlAng;                      /* '<S199>/CAN Unpack1' */
real_T StrWhlAngV;                     /* '<S199>/CAN Unpack1' */
real_T ABS_WS_FL;                      /* '<S128>/CAN Unpack1' */
real_T ABS_WS_FR;                      /* '<S128>/CAN Unpack1' */
real_T ABS_WS_RL;                      /* '<S128>/CAN Unpack1' */
real_T ABS_WS_RR;                      /* '<S128>/CAN Unpack1' */
real_T IMU_Ay_Value;                   /* '<S194>/CAN Unpack' */
real_T IMU_Ax_Value;                   /* '<S194>/CAN Unpack' */
real_T IMU_Yaw_Value;                  /* '<S194>/CAN Unpack' */
real_T EMRAX_Trq_CUT;                  /*  */
real_T AMK_Trq_CUT;                    /*  */
uint32_T Acc_vol2;                     /* '<S209>/Add3' */
uint32_T Acc_vol;                      /* '<S209>/Add2' */
uint32_T Acc_POS;                      /* '<S209>/1-D Lookup Table4' */
uint32_T Acc_POS2;                     /* '<S209>/1-D Lookup Table3' */
real32_T VehVxEst_mps;                 /* '<S331>/Add' */
real32_T PwrALL;                       /* '<S28>/Gain' */
real32_T EmraxTrqR_cmd;                /* '<S7>/Saturation1' */
real32_T AMKTrqFR_cmd;                 /* '<S7>/Saturation2' */
real32_T AMKTrqFL_cmd;                 /* '<S7>/Saturation3' */
uint16_T F_BrkPrs;                     /* '<S209>/1-D Lookup Table1' */
uint16_T Acc1;                         /* '<S121>/Acc3' */
uint16_T Acc2;                         /* '<S121>/Acc4' */
uint16_T Brk1;                         /* '<S121>/Brk1' */
uint16_T Brk2;                         /* '<S121>/Brk2' */
boolean_T STATEDISPLAY;                /* '<S344>/Switch1' */
boolean_T HVSWITCH;                    /* '<S344>/Chart' */
boolean_T Brk;                         /* '<S111>/Compare' */
boolean_T ACC_Release;                 /* '<S112>/Compare' */
boolean_T beeper_state;                /* '<S109>/Chart2' */
boolean_T MCFL_DCOn_setpoints;         /* '<S109>/Chart2' */
boolean_T MCFR_DCEnable;               /* '<S109>/Chart2' */
boolean_T MCFR_InverterOn;             /* '<S109>/Chart2' */
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
  const boolean_T *AND, const real_T *Switch_k, const real_T *Switch3, const
  real_T *Switch10);
static void VehCtrlMdel240926_20_AMKDCready(const real_T *MCFL_bDCOn_j, const
  real_T *MCFR_bDCOn_n, const boolean_T *AND, const real_T *Switch_k, const
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
  /* Call the system: <S369>/CCPReceive */
  {
    /* S-Function (ec5744_caninterruptslb1): '<S369>/ReceiveandTransmitInterrupt' */

    /* Output and update for function-call system: '<S369>/CCPReceive' */

    /* S-Function (ec5744_canreceiveslb): '<S385>/CANReceive' */

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

    /* Nothing to do for system: <S385>/Nothing */

    /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S385>/CANReceive' */

    /* End of Outputs for S-Function (ec5744_caninterruptslb1): '<S369>/ReceiveandTransmitInterrupt' */
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
 *    '<S211>/Timer'
 *    '<S213>/Timer'
 *    '<S213>/Timer1'
 *    '<S213>/Timer2'
 *    '<S213>/Timer3'
 *    '<S273>/Timer'
 *    '<S273>/Timer1'
 *    '<S273>/Timer2'
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
 *    '<S126>/Timer'
 *    '<S209>/Timer'
 */
void VehCtrlMdel240926_201_Timer(boolean_T rtu_Trigger, real32_T rtu_CountTime,
  real_T *rty_Exit, DW_Timer_VehCtrlMdel240926_20_T *localDW)
{
  boolean_T sf_internal_predicateOutput;

  /* Chart: '<S126>/Timer' */
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

  /* End of Chart: '<S126>/Timer' */
}

/* Function for Chart: '<S109>/Chart2' */
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

/* Function for Chart: '<S109>/Chart2' */
static void VehCtrlMdel240926_2018b_VehStat(const real_T *controller_ready_e,
  const boolean_T *AND, const real_T *Switch_k, const real_T *Switch3, const
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
      sf_internal_predicateOutput = ((!*AND) || (!Brk) || (!ACC_Release));
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
      sf_internal_predicateOutput = ((*AND) && Brk && ACC_Release &&
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

/* Function for Chart: '<S109>/Chart2' */
static void VehCtrlMdel240926_20_AMKDCready(const real_T *MCFL_bDCOn_j, const
  real_T *MCFR_bDCOn_n, const boolean_T *AND, const real_T *Switch_k, const
  real_T *Switch3, const real_T *Switch10)
{
  boolean_T sf_internal_predicateOutput;
  int32_T g_previousEvent;
  switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_AMKDCready) {
   case VehCtrlMdel240926_201_IN_AMKCAN:
    sf_internal_predicateOutput = ((*AND) && Brk && ACC_Release);
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
  /* S-Function (fcncallgen): '<S369>/Function-Call Generator' incorporates:
   *  SubSystem: '<S369>/CCPBackground'
   */

  /* S-Function (ec5744_ccpslb): '<S384>/CCPBackground' */
  ccpBackground();
  Lin0_Background();

  /* End of Outputs for S-Function (fcncallgen): '<S369>/Function-Call Generator' */
}

/* Model step function for TID2 */
void VehCtrlMdel240926_2018b_amkspdlimit_step2(void) /* Sample time: [0.005s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S367>/5ms' incorporates:
   *  SubSystem: '<S367>/daq5ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S382>/CCPDAQ' */
  ccpDaq(0);

  /* End of Outputs for S-Function (fcncallgen): '<S367>/5ms' */
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
  real32_T rtb_CastToBoolean1;
  real32_T rtb_Add4_j;
  real32_T rtb_Add7;
  real32_T rtb_Add6;
  real32_T rtb_Switch2_mn;
  real32_T rtb_Switch2_b0;
  real32_T rtb_CastToDouble;
  boolean_T rtb_AND2_e;
  boolean_T rtb_Compare_b;
  real32_T rtb_VxIMU_est;
  real32_T rtb_Ax;
  real_T rtb_Switch2_hly;
  real_T elapseTime_0;
  real32_T rtb_Add12_j;
  real32_T rtb_Gain26;
  real_T rtb_Gain20;
  real_T rtb_Add5;
  real32_T rtb_Add10_b;
  real_T rtb_Switch_jz;
  real32_T rtb_Fz3;
  real32_T rtb_deltafalllimit_cz;
  real32_T rtb_deltafalllimit_ap;
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
  real32_T FLWhlStrAng;
  real32_T Acc_POS_n;
  boolean_T rtb_LogicalOperator_idx_0;

  /* S-Function (fcncallgen): '<S3>/10ms7' incorporates:
   *  SubSystem: '<S3>/key'
   */
  /* S-Function (ec5744_swislbu3): '<S126>/SwitchInput' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240926_2018b_amksp_B.Drive_ready= ec_gpio_read(99);

  /* Logic: '<S126>/Logical Operator' */
  rtb_ignition_e = !VehCtrlMdel240926_2018b_amksp_B.Drive_ready;

  /* Chart: '<S126>/Timer' incorporates:
   *  Constant: '<S126>/Constant5'
   */
  VehCtrlMdel240926_201_Timer(rtb_ignition_e, 0.11F, &ignition,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer);

  /* S-Function (ec5744_swislbu3): '<S126>/SwitchInput1' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240926_2018b_amksp_B.SwitchInput1= ec_gpio_read(45);

  /* Chart: '<S126>/Timer1' incorporates:
   *  Constant: '<S126>/Constant1'
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
      rtb_Compare = ((!VehCtrlMdel240926_2018b_amksp_B.SwitchInput1) &&
                     (VehCtrlMdel240926_2018b_amks_DW.x < 0.10999999940395355));
      if (rtb_Compare) {
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
        rtb_Compare = ((VehCtrlMdel240926_2018b_amks_DW.x < 0.10999999940395355)
                       && VehCtrlMdel240926_2018b_amksp_B.SwitchInput1);
        if (rtb_Compare) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c23_VehCtrlMdel240926_2018b_
            = 1U;
          VehCtrlMdel240926_2018b_amks_DW.x = 0.0;
        }
      }
      break;
    }
  }

  /* End of Chart: '<S126>/Timer1' */

  /* S-Function (ec5744_swislbu3): '<S126>/SwitchInput3' */

  /* Read the the value of the specified switch input */
  VehCtrlMdel240926_2018b_amksp_B.out2_c= ec_gpio_read(92);

  /* SignalConversion generated from: '<S126>/key' */
  VehCtrlMdel240926_2018b_amksp_B.out2_h =
    VehCtrlMdel240926_2018b_amksp_B.out2_c;

  /* SignalConversion generated from: '<S126>/key' */
  VehCtrlMdel240926_2018b_amksp_B.AMKSWITCH_bx = AMKSWITCH;

  /* SignalConversion generated from: '<S126>/key' */
  VehCtrlMdel240926_2018b_amksp_B.ignition_d = ignition;

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms7' */

  /* S-Function (fcncallgen): '<S3>/10ms6' incorporates:
   *  SubSystem: '<S3>/EMRAXMCU_RECIEVE'
   */
  /* S-Function (ec5744_canreceiveslb): '<S123>/CANReceive1' */

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

  /* Call the system: <S123>/MCU_pwr */

  /* Output and update for function-call system: '<S123>/MCU_pwr' */

  /* Outputs for Enabled SubSystem: '<S178>/MCU_VCUMeter1' incorporates:
   *  EnablePort: '<S180>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S180>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S180>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S180>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S178>/MCU_VCUMeter1' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S123>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S123>/CANReceive3' */

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

  /* Call the system: <S123>/MCU_state */

  /* Output and update for function-call system: '<S123>/MCU_state' */

  /* Outputs for Enabled SubSystem: '<S179>/MCU_state' incorporates:
   *  EnablePort: '<S186>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S186>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S186>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S186>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S179>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S123>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms6' */

  /* S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S119>/CANReceive3' */

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

  /* Call the system: <S119>/ABS_BUS_state */

  /* Output and update for function-call system: '<S119>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S127>/IMU_state' incorporates:
   *  EnablePort: '<S128>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_m > 0) {
    /* S-Function (ecucoder_canunmessage): '<S128>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S128>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S128>/CAN Unpack1' */
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

  /* End of Outputs for SubSystem: '<S127>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S119>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms3' */

  /* S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S125>/CANReceive3' */

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

  /* Call the system: <S125>/StrWhSnis_state */

  /* Output and update for function-call system: '<S125>/StrWhSnis_state' */

  /* Outputs for Enabled SubSystem: '<S198>/IMU_state' incorporates:
   *  EnablePort: '<S199>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S199>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S199>/CAN Unpack1' */
    {
      /* S-Function (scanunpack): '<S199>/CAN Unpack1' */
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

  /* End of Outputs for SubSystem: '<S198>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S125>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms4' */

  /* S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S133>/CANReceive3' */

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

  /* Call the system: <S133>/AMKMCU_state */

  /* Output and update for function-call system: '<S133>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S135>/MCU_state' incorporates:
   *  EnablePort: '<S138>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S138>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S138>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S138>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S135>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S133>/CANReceive1' */

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

  /* Call the system: <S133>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S133>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S136>/MCU_state' incorporates:
   *  EnablePort: '<S148>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2_l > 0) {
    /* S-Function (ecucoder_canunmessage): '<S148>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S148>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S148>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S136>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S133>/CANReceive2' */

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

  /* Call the system: <S133>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S133>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S137>/MCU_state' incorporates:
   *  EnablePort: '<S150>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o2 > 0) {
    /* S-Function (ecucoder_canunmessage): '<S150>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S150>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S150>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S137>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive2' */

  /* S-Function (ec5744_canreceiveslb): '<S134>/CANReceive3' */

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

  /* Call the system: <S134>/AMKMCU_state */

  /* Output and update for function-call system: '<S134>/AMKMCU_state' */

  /* Outputs for Enabled SubSystem: '<S154>/MCU_state' incorporates:
   *  EnablePort: '<S157>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_a > 0) {
    /* S-Function (ecucoder_canunmessage): '<S157>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S157>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S157>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S154>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S134>/CANReceive3' */

  /* S-Function (ec5744_canreceiveslb): '<S134>/CANReceive1' */

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

  /* Call the system: <S134>/AMKMCU_state1 */

  /* Output and update for function-call system: '<S134>/AMKMCU_state1' */

  /* Outputs for Enabled SubSystem: '<S155>/MCU_state' incorporates:
   *  EnablePort: '<S166>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive1_o2_o > 0) {
    /* S-Function (ecucoder_canunmessage): '<S166>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S166>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S166>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S155>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S134>/CANReceive1' */

  /* S-Function (ec5744_canreceiveslb): '<S134>/CANReceive2' */

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

  /* Call the system: <S134>/AMKMCU_state2 */

  /* Output and update for function-call system: '<S134>/AMKMCU_state2' */

  /* Outputs for Enabled SubSystem: '<S156>/MCU_state' incorporates:
   *  EnablePort: '<S168>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive2_o2_p > 0) {
    /* S-Function (ecucoder_canunmessage): '<S168>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S168>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S168>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S156>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S134>/CANReceive2' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms5' */

  /* S-Function (fcncallgen): '<S3>/10ms' incorporates:
   *  SubSystem: '<S3>/AccBrk_BUS'
   */
  /* S-Function (ec5744_asislbu3): '<S121>/Acc3' */

  /* Read the ADC conversion result of the analog signal */
  Acc1= adc_read_chan(1,2);

  /* S-Function (ec5744_asislbu3): '<S121>/Acc4' */

  /* Read the ADC conversion result of the analog signal */
  Acc2= adc_read_chan(1,4);

  /* S-Function (ec5744_asislbu3): '<S121>/Brk1' */

  /* Read the ADC conversion result of the analog signal */
  Brk1= adc_read_chan(1,0);

  /* S-Function (ec5744_asislbu3): '<S121>/Brk2' */

  /* Read the ADC conversion result of the analog signal */
  Brk2= adc_read_chan(0,13);

  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms' */

  /* S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* S-Function (ec5744_canreceiveslb): '<S124>/CANReceive3' */

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

  /* Call the system: <S124>/IMU_state */

  /* Output and update for function-call system: '<S124>/IMU_state' */

  /* Outputs for Enabled SubSystem: '<S193>/MCU_state' incorporates:
   *  EnablePort: '<S194>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_ma > 0) {
    /* S-Function (ecucoder_canunmessage): '<S194>/CANUnPackMessage4' */

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

    /* S-Function (scanunpack): '<S194>/CAN Unpack' */
    {
      /* S-Function (scanunpack): '<S194>/CAN Unpack' */
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

  /* End of Outputs for SubSystem: '<S193>/MCU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S124>/CANReceive3' */
  /* End of Outputs for S-Function (fcncallgen): '<S3>/10ms2' */

  /* S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' */

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

  /* Call the system: <S122>/ABS_BUS_state */

  /* Output and update for function-call system: '<S122>/ABS_BUS_state' */

  /* Outputs for Enabled SubSystem: '<S176>/IMU_state' incorporates:
   *  EnablePort: '<S177>/Enable'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.CANReceive3_o2_k > 0) {
    /* S-Function (ecucoder_canunmessage): '<S177>/CANUnPackMessage4' */

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

  /* End of Outputs for SubSystem: '<S176>/IMU_state' */

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' */
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

  /* Lookup_n-D: '<S209>/1-D Lookup Table1' */
  F_BrkPrs = look1_iu16bflftfIu16_binlc(Brk1,
    VehCtrlMdel240926_2018b__ConstP.pooled67,
    VehCtrlMdel240926_2018b__ConstP.pooled67, 1U);

  /* DataTypeConversion: '<S209>/Data Type Conversion' */
  rtb_FRWhlStrAng = F_BrkPrs;

  /* SignalConversion generated from: '<S207>/Out1' */
  Brk_F = (int32_T)rtb_FRWhlStrAng;

  /* Gain: '<S209>/Gain2' */
  rtb_Gain1 = 45875U * Acc2;

  /* Gain: '<S209>/Gain3' incorporates:
   *  UnitDelay: '<S209>/Unit Delay1'
   */
  rtb_Gain = 39322U * VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S209>/Add3' */
  Acc_vol2 = (rtb_Gain >> 1) + rtb_Gain1;

  /* RelationalOperator: '<S218>/Compare' */
  rtb_ignition_e = (Acc_vol2 <= 32768000U);

  /* RelationalOperator: '<S219>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol2 >= 294912000U);

  /* Logic: '<S209>/Logical Operator1' */
  rtb_ignition_e = (rtb_ignition_e || rtb_LogicalOperator2);

  /* Gain: '<S209>/Gain' */
  rtb_Gain = 45875U * Acc1;

  /* UnitDelay: '<S209>/Unit Delay' incorporates:
   *  UnitDelay: '<S209>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_k;

  /* Gain: '<S209>/Gain1' incorporates:
   *  UnitDelay: '<S209>/Unit Delay1'
   */
  rtb_Gain1 = 39322U * VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm;

  /* Sum: '<S209>/Add2' */
  Acc_vol = (rtb_Gain1 >> 1) + rtb_Gain;

  /* RelationalOperator: '<S214>/Compare' */
  rtb_LogicalOperator2 = (Acc_vol <= 32768000U);

  /* RelationalOperator: '<S215>/Compare' */
  rtb_Compare = (Acc_vol >= 294912000U);

  /* Logic: '<S209>/Logical Operator' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_Compare);

  /* Logic: '<S209>/Logical Operator2' */
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 || rtb_ignition_e);

  /* Lookup_n-D: '<S209>/1-D Lookup Table4' */
  Acc_POS = look1_iu32n16bflftfIu32_binlc(Acc_vol,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable4_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.pooled68, 1U);

  /* DataTypeConversion: '<S209>/Data Type Conversion1' */
  rtb_FRWhlStrAng = (real32_T)Acc_POS * 1.52587891E-5F;

  /* Lookup_n-D: '<S209>/1-D Lookup Table3' */
  Acc_POS2 = look1_iu32n16bflftfIu32_binlc(Acc_vol2,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable3_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.pooled68, 1U);

  /* DataTypeConversion: '<S209>/Data Type Conversion4' */
  rtb_Gain3_o = (real32_T)Acc_POS2 * 1.52587891E-5F;

  /* Sum: '<S209>/Add1' */
  rtb_StrWhlAngV_c = rtb_FRWhlStrAng - rtb_Gain3_o;

  /* Abs: '<S209>/Abs' */
  rtb_StrWhlAngV_c = fabsf(rtb_StrWhlAngV_c);

  /* RelationalOperator: '<S222>/Compare' incorporates:
   *  Constant: '<S222>/Constant'
   */
  rtb_Compare = (rtb_StrWhlAngV_c > 10.0F);

  /* RelationalOperator: '<S220>/Compare' incorporates:
   *  Constant: '<S220>/Constant'
   */
  rtb_ignition_e = (rtb_FRWhlStrAng > 100.0F);

  /* RelationalOperator: '<S221>/Compare' incorporates:
   *  Constant: '<S221>/Constant'
   */
  rtb_LogicalOperator7_m = (rtb_Gain3_o > 100.0F);

  /* Logic: '<S209>/Logical Operator3' */
  rtb_ignition_e = (rtb_ignition_e || rtb_LogicalOperator7_m);

  /* RelationalOperator: '<S223>/Compare' incorporates:
   *  Constant: '<S223>/Constant'
   */
  rtb_LogicalOperator7_m = (Brk1 <= 300);

  /* RelationalOperator: '<S224>/Compare' incorporates:
   *  Constant: '<S224>/Constant'
   */
  rtb_Compare_am = (Brk1 >= 4500);

  /* Logic: '<S209>/Logical Operator5' */
  rtb_LogicalOperator7_m = (rtb_LogicalOperator7_m || rtb_Compare_am);

  /* RelationalOperator: '<S216>/Compare' incorporates:
   *  Constant: '<S216>/Constant'
   */
  rtb_Compare_am = (Brk2 <= 300);

  /* RelationalOperator: '<S217>/Compare' incorporates:
   *  Constant: '<S217>/Constant'
   */
  rtb_LowerRelop1_b = (Brk2 >= 4500);

  /* Logic: '<S209>/Logical Operator6' */
  rtb_Compare_am = (rtb_Compare_am || rtb_LowerRelop1_b);

  /* Logic: '<S209>/Logical Operator7' */
  rtb_LogicalOperator7_m = (rtb_LogicalOperator7_m || rtb_Compare_am);

  /* Logic: '<S209>/Logical Operator4' */
  rtb_ignition_e = (rtb_ignition_e || rtb_Compare || rtb_LogicalOperator2 ||
                    rtb_LogicalOperator7_m);

  /* Chart: '<S209>/Timer' incorporates:
   *  Constant: '<S209>/Constant1'
   */
  VehCtrlMdel240926_201_Timer(rtb_ignition_e, 0.11F, &Trq_CUT,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_a);

  /* UnitDelay: '<S244>/Delay Input2'
   *
   * Block description for '<S244>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_StrWhlAngV_c = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2;

  /* SampleTimeMath: '<S244>/sample time'
   *
   * About '<S244>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S244>/delta rise limit' */
  rtb_FRWhlStrAng = (real32_T)(1200.0 * elapseTime);

  /* DataTypeConversion: '<S211>/Cast To Boolean' */
  rtb_CastToBoolean = (real32_T)StrWhlAng;

  /* Sum: '<S244>/Difference Inputs1'
   *
   * Block description for '<S244>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_CastToBoolean -= rtb_StrWhlAngV_c;

  /* RelationalOperator: '<S247>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_CastToBoolean > rtb_FRWhlStrAng);

  /* Switch: '<S247>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S244>/delta fall limit' */
    rtb_FRWhlStrAng = (real32_T)(-1200.0 * elapseTime);

    /* RelationalOperator: '<S247>/UpperRelop' */
    rtb_ignition_e = (rtb_CastToBoolean < rtb_FRWhlStrAng);

    /* Switch: '<S247>/Switch' */
    if (rtb_ignition_e) {
      rtb_CastToBoolean = rtb_FRWhlStrAng;
    }

    /* End of Switch: '<S247>/Switch' */
    rtb_FRWhlStrAng = rtb_CastToBoolean;
  }

  /* End of Switch: '<S247>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2 = rtb_FRWhlStrAng +
    rtb_StrWhlAngV_c;

  /* Abs: '<S211>/Abs' incorporates:
   *  UnitDelay: '<S244>/Delay Input2'
   *
   * Block description for '<S244>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_StrWhlAngV_c = fabsf(VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2);

  /* RelationalOperator: '<S243>/Compare' incorporates:
   *  Constant: '<S243>/Constant'
   */
  rtb_ignition_e = (rtb_StrWhlAngV_c > 120.0F);

  /* Chart: '<S211>/Timer' incorporates:
   *  Constant: '<S211>/Constant5'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition_e, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_on,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_k);

  /* UnitDelay: '<S259>/Delay Input2'
   *
   * Block description for '<S259>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE;

  /* SampleTimeMath: '<S259>/sample time'
   *
   * About '<S259>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S259>/delta rise limit' */
  rtb_Gain5 = 10.0 * elapseTime;

  /* Sum: '<S259>/Difference Inputs1'
   *
   * Block description for '<S259>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RL - rtb_Yk1_l;

  /* RelationalOperator: '<S267>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain5);

  /* Switch: '<S267>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S259>/delta fall limit' */
    elapseTime *= -10.0;

    /* RelationalOperator: '<S267>/UpperRelop' */
    rtb_ignition_e = (rtb_UkYk1 < elapseTime);

    /* Switch: '<S267>/Switch' */
    if (rtb_ignition_e) {
      rtb_UkYk1 = elapseTime;
    }

    /* End of Switch: '<S267>/Switch' */
    rtb_Gain5 = rtb_UkYk1;
  }

  /* End of Switch: '<S267>/Switch2' */

  /* Saturate: '<S213>/Saturation' incorporates:
   *  Sum: '<S259>/Difference Inputs2'
   *  UnitDelay: '<S259>/Delay Input2'
   *
   * Block description for '<S259>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S259>/Delay Input2':
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

  /* End of Saturate: '<S213>/Saturation' */

  /* Gain: '<S213>/Gain' */
  rtb_Gain5 *= 0.27777777777777779;

  /* RelationalOperator: '<S251>/Compare' incorporates:
   *  Constant: '<S251>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain5 >= 0.0);

  /* RelationalOperator: '<S252>/Compare' incorporates:
   *  Constant: '<S252>/Constant'
   */
  rtb_Compare_am = (rtb_Gain5 < 40.0);

  /* Logic: '<S213>/OR' */
  rtb_ignition_e = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S213>/Timer' incorporates:
   *  Constant: '<S213>/Constant5'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition_e, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_le,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_b);

  /* UnitDelay: '<S260>/Delay Input2'
   *
   * Block description for '<S260>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_b;

  /* SampleTimeMath: '<S260>/sample time'
   *
   * About '<S260>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S260>/delta rise limit' */
  rtb_Gain4 = 10.0 * elapseTime;

  /* Sum: '<S260>/Difference Inputs1'
   *
   * Block description for '<S260>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = ABS_WS_RR - rtb_Yk1_l;

  /* RelationalOperator: '<S268>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Gain4);

  /* Switch: '<S268>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S260>/delta fall limit' */
    elapseTime *= -10.0;

    /* RelationalOperator: '<S268>/UpperRelop' */
    rtb_ignition_e = (rtb_UkYk1 < elapseTime);

    /* Switch: '<S268>/Switch' */
    if (rtb_ignition_e) {
      rtb_UkYk1 = elapseTime;
    }

    /* End of Switch: '<S268>/Switch' */
    rtb_Gain4 = rtb_UkYk1;
  }

  /* End of Switch: '<S268>/Switch2' */

  /* Saturate: '<S213>/Saturation1' incorporates:
   *  Sum: '<S260>/Difference Inputs2'
   *  UnitDelay: '<S260>/Delay Input2'
   *
   * Block description for '<S260>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S260>/Delay Input2':
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

  /* End of Saturate: '<S213>/Saturation1' */

  /* Gain: '<S213>/Gain3' */
  rtb_Gain4 *= 0.27777777777777779;

  /* RelationalOperator: '<S253>/Compare' incorporates:
   *  Constant: '<S253>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain4 >= 0.0);

  /* RelationalOperator: '<S254>/Compare' incorporates:
   *  Constant: '<S254>/Constant'
   */
  rtb_Compare_am = (rtb_Gain4 < 40.0);

  /* Logic: '<S213>/OR1' */
  rtb_ignition_e = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S213>/Timer1' incorporates:
   *  Constant: '<S213>/Constant1'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition_e, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_is,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_n);

  /* UnitDelay: '<S261>/Delay Input2'
   *
   * Block description for '<S261>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h;

  /* SampleTimeMath: '<S261>/sample time'
   *
   * About '<S261>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S261>/delta rise limit' */
  rtb_Switch2_on = 10.0 * elapseTime;

  /* Sum: '<S261>/Difference Inputs1'
   *
   * Block description for '<S261>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = MCFR_ActualVelocity - rtb_Yk1_l;

  /* RelationalOperator: '<S269>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1 > rtb_Switch2_on);

  /* Switch: '<S269>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S261>/delta fall limit' */
    elapseTime *= -10.0;

    /* RelationalOperator: '<S269>/UpperRelop' */
    rtb_ignition_e = (rtb_UkYk1 < elapseTime);

    /* Switch: '<S269>/Switch' */
    if (rtb_ignition_e) {
      rtb_UkYk1 = elapseTime;
    }

    /* End of Switch: '<S269>/Switch' */
    rtb_Switch2_on = rtb_UkYk1;
  }

  /* End of Switch: '<S269>/Switch2' */

  /* Saturate: '<S213>/Saturation2' incorporates:
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h = rtb_Switch2_on +
    rtb_Yk1_l;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h > 30.0) {
    rtb_Switch2_on = 30.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h < 0.0) {
    rtb_Switch2_on = 0.0;
  } else {
    rtb_Switch2_on = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_h;
  }

  /* End of Saturate: '<S213>/Saturation2' */

  /* Gain: '<S213>/Gain1' */
  rtb_Switch2_on *= 0.002235050147492625;

  /* RelationalOperator: '<S255>/Compare' incorporates:
   *  Constant: '<S255>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Switch2_on >= 0.0);

  /* RelationalOperator: '<S256>/Compare' incorporates:
   *  Constant: '<S256>/Constant'
   */
  rtb_Compare_am = (rtb_Switch2_on < 40.0);

  /* Logic: '<S213>/OR2' */
  rtb_ignition_e = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S213>/Timer2' incorporates:
   *  Constant: '<S213>/Constant4'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition_e, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_o4,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_l);

  /* UnitDelay: '<S262>/Delay Input2'
   *
   * Block description for '<S262>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Yk1_l = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n;

  /* SampleTimeMath: '<S262>/sample time'
   *
   * About '<S262>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S262>/delta rise limit' */
  rtb_UkYk1 = 10.0 * elapseTime;

  /* Sum: '<S262>/Difference Inputs1'
   *
   * Block description for '<S262>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1_nc = MCFL_ActualVelocity - rtb_Yk1_l;

  /* RelationalOperator: '<S270>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1_nc > rtb_UkYk1);

  /* Switch: '<S270>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S262>/delta fall limit' */
    rtb_UkYk1 = -10.0 * elapseTime;

    /* RelationalOperator: '<S270>/UpperRelop' */
    rtb_ignition_e = (rtb_UkYk1_nc < rtb_UkYk1);

    /* Switch: '<S270>/Switch' */
    if (rtb_ignition_e) {
      rtb_UkYk1_nc = rtb_UkYk1;
    }

    /* End of Switch: '<S270>/Switch' */
    rtb_UkYk1 = rtb_UkYk1_nc;
  }

  /* End of Switch: '<S270>/Switch2' */

  /* Saturate: '<S213>/Saturation3' incorporates:
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n = rtb_UkYk1 + rtb_Yk1_l;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n > 30.0) {
    rtb_UkYk1 = 30.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n < 0.0) {
    rtb_UkYk1 = 0.0;
  } else {
    rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n;
  }

  /* End of Saturate: '<S213>/Saturation3' */

  /* Gain: '<S213>/Gain2' */
  rtb_UkYk1 *= 0.002235050147492625;

  /* RelationalOperator: '<S257>/Compare' incorporates:
   *  Constant: '<S257>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_UkYk1 >= 0.0);

  /* RelationalOperator: '<S258>/Compare' incorporates:
   *  Constant: '<S258>/Constant'
   */
  rtb_Compare_am = (rtb_UkYk1 < 40.0);

  /* Logic: '<S213>/OR3' */
  rtb_ignition_e = (rtb_LowerRelop1_b || rtb_Compare_am);

  /* Chart: '<S213>/Timer3' incorporates:
   *  Constant: '<S213>/Constant8'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition_e, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_h,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer3);

  /* SignalConversion generated from: '<S207>/Out1' */
  WhlSpdFL = rtb_UkYk1;

  /* SignalConversion generated from: '<S207>/Out1' */
  WhlSpdFR = rtb_Switch2_on;

  /* SignalConversion generated from: '<S207>/Out1' */
  WhlSpdRR_mps = rtb_Gain4;

  /* SignalConversion generated from: '<S207>/Out1' */
  WhlSpdRL_mps = rtb_Gain5;

  /* Gain: '<S211>/Gain' incorporates:
   *  UnitDelay: '<S244>/Delay Input2'
   *
   * Block description for '<S244>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_StrWhlAngV_c = 0.7F *
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2;

  /* UnitDelay: '<S211>/Unit Delay' */
  rtb_FRWhlStrAng = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o2;

  /* Gain: '<S211>/Gain1' */
  rtb_FRWhlStrAng *= 0.3F;

  /* Sum: '<S211>/Add2' */
  rtb_StrWhlAngV_c += rtb_FRWhlStrAng;

  /* Lookup_n-D: '<S211>/1-D Lookup Table' */
  rtb_FRWhlStrAng = look1_iflf_binlx(rtb_StrWhlAngV_c,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable_tableData, 23U);

  /* SignalConversion generated from: '<S207>/Out1' */
  FLWhlStrAng = rtb_FRWhlStrAng;

  /* Lookup_n-D: '<S211>/1-D Lookup Table1' */
  rtb_FRWhlStrAng = look1_iflf_binlx(rtb_StrWhlAngV_c,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_bp01Data_h,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable1_tableData_b, 23U);

  /* SignalConversion generated from: '<S207>/Out1' */
  rtb_CastToBoolean = rtb_StrWhlAngV_c;

  /* Sum: '<S209>/Add' */
  rtb_StrWhlAngV_c = (real32_T)Acc_POS * 1.52587891E-5F + rtb_Gain3_o;

  /* Product: '<S209>/Divide' incorporates:
   *  Constant: '<S209>/Constant'
   */
  rtb_Gain3_o = (real32_T)(rtb_StrWhlAngV_c / 2.0);

  /* SignalConversion generated from: '<S207>/Out1' */
  Acc_POS_n = rtb_Gain3_o;

  /* UnitDelay: '<S234>/Delay Input2'
   *
   * Block description for '<S234>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l;

  /* SampleTimeMath: '<S234>/sample time'
   *
   * About '<S234>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S234>/delta rise limit' incorporates:
   *  Constant: '<S233>/Constant'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* UnitDelay: '<S233>/Unit Delay' */
  rtb_Gain4 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE;

  /* Gain: '<S233>/Gain1' */
  rtb_Gain4 *= 0.3;

  /* Gain: '<S210>/g_mpss' incorporates:
   *  UnitDelay: '<S233>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE = 9.8 * IMU_Ay_Value;

  /* Gain: '<S233>/Gain' incorporates:
   *  UnitDelay: '<S233>/Unit Delay'
   */
  rtb_Gain5 = 0.7 * VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE;

  /* Sum: '<S233>/Add2' */
  rtb_Yk1_l = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S234>/Difference Inputs1'
   *
   * Block description for '<S234>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Yk1_l -= rtb_UkYk1;

  /* RelationalOperator: '<S240>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Yk1_l > rtb_Switch2_on);

  /* Switch: '<S240>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S234>/delta fall limit' incorporates:
     *  Constant: '<S233>/Constant1'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S240>/UpperRelop' */
    rtb_ignition_e = (rtb_Yk1_l < elapseTime);

    /* Switch: '<S240>/Switch' */
    if (rtb_ignition_e) {
      rtb_Yk1_l = elapseTime;
    }

    /* End of Switch: '<S240>/Switch' */
    rtb_Switch2_on = rtb_Yk1_l;
  }

  /* End of Switch: '<S240>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l = rtb_Switch2_on +
    rtb_UkYk1;

  /* RelationalOperator: '<S237>/LowerRelop1' incorporates:
   *  Constant: '<S233>/Constant6'
   *  UnitDelay: '<S234>/Delay Input2'
   *
   * Block description for '<S234>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l >
                       1.5);

  /* Switch: '<S237>/Switch2' incorporates:
   *  Constant: '<S233>/Constant6'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S237>/UpperRelop' incorporates:
     *  Constant: '<S233>/Constant7'
     *  UnitDelay: '<S234>/Delay Input2'
     *
     * Block description for '<S234>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_ignition_e = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l <
                      -1.5);

    /* Switch: '<S237>/Switch' incorporates:
     *  Constant: '<S233>/Constant7'
     *  UnitDelay: '<S234>/Delay Input2'
     *
     * Block description for '<S234>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_ignition_e) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l;
    }

    /* End of Switch: '<S237>/Switch' */
  }

  /* End of Switch: '<S237>/Switch2' */

  /* SignalConversion generated from: '<S207>/Out1' */
  rtb_Yk1_l = rtb_UkYk1;

  /* UnitDelay: '<S235>/Delay Input2'
   *
   * Block description for '<S235>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m;

  /* SampleTimeMath: '<S235>/sample time'
   *
   * About '<S235>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S235>/delta rise limit' incorporates:
   *  Constant: '<S233>/Constant2'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S210>/g_mpss1' */
  rtb_g_mpss1 = 9.8 * IMU_Ax_Value;

  /* Gain: '<S233>/Gain2' */
  rtb_Gain4 = 0.7 * rtb_g_mpss1;

  /* UnitDelay: '<S233>/Unit Delay1' */
  rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE;

  /* Gain: '<S233>/Gain3' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S233>/Add1' */
  rtb_UkYk1_nc = rtb_Gain4 + rtb_Gain5;

  /* Sum: '<S235>/Difference Inputs1'
   *
   * Block description for '<S235>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1_nc -= rtb_UkYk1;

  /* RelationalOperator: '<S241>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_UkYk1_nc > rtb_Switch2_on);

  /* Switch: '<S241>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S235>/delta fall limit' incorporates:
     *  Constant: '<S233>/Constant4'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S241>/UpperRelop' */
    rtb_ignition_e = (rtb_UkYk1_nc < elapseTime);

    /* Switch: '<S241>/Switch' */
    if (rtb_ignition_e) {
      rtb_UkYk1_nc = elapseTime;
    }

    /* End of Switch: '<S241>/Switch' */
    rtb_Switch2_on = rtb_UkYk1_nc;
  }

  /* End of Switch: '<S241>/Switch2' */

  /* Sum: '<S235>/Difference Inputs2' incorporates:
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m = rtb_Switch2_on +
    rtb_UkYk1;

  /* RelationalOperator: '<S238>/LowerRelop1' incorporates:
   *  Constant: '<S233>/Constant8'
   *  UnitDelay: '<S235>/Delay Input2'
   *
   * Block description for '<S235>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m >
                       1.5);

  /* Switch: '<S238>/Switch2' incorporates:
   *  Constant: '<S233>/Constant8'
   */
  if (rtb_LowerRelop1_b) {
    rtb_UkYk1 = 1.5;
  } else {
    /* RelationalOperator: '<S238>/UpperRelop' incorporates:
     *  Constant: '<S233>/Constant9'
     *  UnitDelay: '<S235>/Delay Input2'
     *
     * Block description for '<S235>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_ignition_e = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m <
                      -1.5);

    /* Switch: '<S238>/Switch' incorporates:
     *  Constant: '<S233>/Constant9'
     *  UnitDelay: '<S235>/Delay Input2'
     *
     * Block description for '<S235>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_ignition_e) {
      rtb_UkYk1 = -1.5;
    } else {
      rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_m;
    }

    /* End of Switch: '<S238>/Switch' */
  }

  /* End of Switch: '<S238>/Switch2' */

  /* SignalConversion generated from: '<S207>/Out1' */
  rtb_UkYk1_nc = rtb_UkYk1;

  /* SampleTimeMath: '<S245>/sample time'
   *
   * About '<S245>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S245>/delta rise limit' */
  rtb_StrWhlAngV_c = (real32_T)(1200.0 * elapseTime);

  /* DataTypeConversion: '<S211>/Cast To Boolean1' */
  rtb_CastToBoolean1 = (real32_T)StrWhlAngV;

  /* UnitDelay: '<S245>/Delay Input2'
   *
   * Block description for '<S245>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Gain3_o = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j;

  /* Sum: '<S245>/Difference Inputs1'
   *
   * Block description for '<S245>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_CastToBoolean1 -= rtb_Gain3_o;

  /* RelationalOperator: '<S248>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_CastToBoolean1 > rtb_StrWhlAngV_c);

  /* Switch: '<S248>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S245>/delta fall limit' */
    rtb_StrWhlAngV_c = (real32_T)(-1200.0 * elapseTime);

    /* RelationalOperator: '<S248>/UpperRelop' */
    rtb_ignition_e = (rtb_CastToBoolean1 < rtb_StrWhlAngV_c);

    /* Switch: '<S248>/Switch' */
    if (rtb_ignition_e) {
      rtb_CastToBoolean1 = rtb_StrWhlAngV_c;
    }

    /* End of Switch: '<S248>/Switch' */
    rtb_StrWhlAngV_c = rtb_CastToBoolean1;
  }

  /* End of Switch: '<S248>/Switch2' */

  /* Saturate: '<S211>/Saturation1' incorporates:
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j = rtb_StrWhlAngV_c +
    rtb_Gain3_o;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j > 1200.0F) {
    rtb_CastToBoolean1 = 1200.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j < 0.0F) {
    rtb_CastToBoolean1 = 0.0F;
  } else {
    rtb_CastToBoolean1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j;
  }

  /* End of Saturate: '<S211>/Saturation1' */

  /* Gain: '<S211>/Gain2' */
  rtb_StrWhlAngV_c = 0.7F * rtb_CastToBoolean1;

  /* UnitDelay: '<S211>/Unit Delay1' */
  rtb_Gain3_o = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_aq;

  /* Gain: '<S211>/Gain3' */
  rtb_Gain3_o *= 0.3F;

  /* Sum: '<S211>/Add1' */
  rtb_StrWhlAngV_c += rtb_Gain3_o;

  /* UnitDelay: '<S236>/Delay Input2'
   *
   * Block description for '<S236>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k;

  /* SampleTimeMath: '<S236>/sample time'
   *
   * About '<S236>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S236>/delta rise limit' incorporates:
   *  Constant: '<S233>/Constant3'
   */
  rtb_Switch2_on = 5000.0 * elapseTime;

  /* Gain: '<S233>/Gain4' */
  rtb_Gain4 = 0.7 * IMU_Yaw_Value;

  /* UnitDelay: '<S233>/Unit Delay2' */
  rtb_Gain5 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE;

  /* Gain: '<S233>/Gain5' */
  rtb_Gain5 *= 0.3;

  /* Sum: '<S233>/Add3' */
  rtb_Gain5 += rtb_Gain4;

  /* Sum: '<S236>/Difference Inputs1'
   *
   * Block description for '<S236>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Gain5 -= rtb_UkYk1;

  /* RelationalOperator: '<S242>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Gain5 > rtb_Switch2_on);

  /* Switch: '<S242>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S236>/delta fall limit' incorporates:
     *  Constant: '<S233>/Constant5'
     */
    elapseTime *= -5000.0;

    /* RelationalOperator: '<S242>/UpperRelop' */
    rtb_ignition_e = (rtb_Gain5 < elapseTime);

    /* Switch: '<S242>/Switch' */
    if (rtb_ignition_e) {
      rtb_Gain5 = elapseTime;
    }

    /* End of Switch: '<S242>/Switch' */
    rtb_Switch2_on = rtb_Gain5;
  }

  /* End of Switch: '<S242>/Switch2' */

  /* Saturate: '<S233>/Saturation2' incorporates:
   *  Sum: '<S236>/Difference Inputs2'
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k = rtb_Switch2_on +
    rtb_UkYk1;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k > 180.0) {
    rtb_UkYk1 = 180.0;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k < -180.0) {
    rtb_UkYk1 = -180.0;
  } else {
    rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_k;
  }

  /* End of Saturate: '<S233>/Saturation2' */

  /* SignalConversion generated from: '<S207>/Out1' */
  KeyPressed = VehCtrlMdel240926_2018b_amksp_B.ignition_d;

  /* SignalConversion generated from: '<S207>/Out1' */
  rtb_ignition_e = VehCtrlMdel240926_2018b_amksp_B.out2_h;

  /* Switch: '<S212>/Switch' incorporates:
   *  Constant: '<S212>/Constant4'
   */
  if (MCFL_DCVoltage != 0.0) {
    /* MinMax: '<S212>/Max' incorporates:
     *  Constant: '<S212>/Constant2'
     */
    elapseTime = fmax(MCFL_DCVoltage, 2.0);

    /* Product: '<S212>/Product' */
    rtb_Switch2_on = MCFL_ActualTorque * MCFL_ActualVelocity;

    /* Product: '<S212>/Divide' incorporates:
     *  Constant: '<S212>/Constant'
     */
    rtb_Switch2_on /= 9550.0;

    /* Product: '<S212>/Divide1' */
    AMKFL_Current = rtb_Switch2_on / elapseTime;
  } else {
    AMKFL_Current = 0.0;
  }

  /* End of Switch: '<S212>/Switch' */

  /* Switch: '<S212>/Switch1' incorporates:
   *  Constant: '<S212>/Constant5'
   */
  if (MCFR_DCVoltage != 0.0) {
    /* MinMax: '<S212>/Max1' incorporates:
     *  Constant: '<S212>/Constant3'
     */
    elapseTime = fmax(MCFR_DCVoltage, 2.0);

    /* Product: '<S212>/Product1' */
    rtb_Switch2_on = MCFR_ActualTorque * MCFR_ActualVelocity;

    /* Product: '<S212>/Divide2' incorporates:
     *  Constant: '<S212>/Constant1'
     */
    rtb_Switch2_on /= 9550.0;

    /* Product: '<S212>/Divide3' */
    AMKFR_Current = rtb_Switch2_on / elapseTime;
  } else {
    AMKFR_Current = 0.0;
  }

  /* End of Switch: '<S212>/Switch1' */

  /* Update for UnitDelay: '<S209>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_fm = Acc2;

  /* Update for UnitDelay: '<S209>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_k = Acc1;

  /* Update for UnitDelay: '<S211>/Unit Delay' incorporates:
   *  UnitDelay: '<S244>/Delay Input2'
   *
   * Block description for '<S244>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o2 =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l2;

  /* Update for UnitDelay: '<S233>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE = rtb_g_mpss1;

  /* Update for UnitDelay: '<S211>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_aq = rtb_CastToBoolean1;

  /* Update for UnitDelay: '<S233>/Unit Delay2' */
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

  /* Gain: '<S208>/Gain5' */
  elapseTime = 10.0 * VehCtrlMdel240926_2018b_amksp_B.CANUnpack_o1;

  /* DataTypeConversion: '<S208>/Cast To Double' */
  rtb_CastToDouble = (real32_T)elapseTime;

  /* MinMax: '<S335>/Min3' incorporates:
   *  Gain: '<S289>/Gain'
   *  UnitDelay: '<S289>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p *= 0.3F;

  /* UnitDelay: '<S293>/Delay Input2'
   *
   * Block description for '<S293>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* SampleTimeMath: '<S293>/sample time'
   *
   * About '<S293>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S293>/delta rise limit' */
  rtb_Add4_j = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S208>/Cast To Double1' */
  rtb_Add7 = (real32_T)WhlSpdFL;

  /* Gain: '<S275>/Gain2' */
  rtb_Add6 = 0.0174532924F * FLWhlStrAng;

  /* Trigonometry: '<S275>/Asin' */
  rtb_Add6 = cosf(rtb_Add6);

  /* Product: '<S275>/Product1' */
  rtb_Add7 *= rtb_Add6;

  /* DataTypeConversion: '<S208>/Cast To Double5' */
  rtb_Add6 = (real32_T)rtb_UkYk1;

  /* Gain: '<S275>/Gain4' */
  rtb_Add6 *= 0.0174532924F;

  /* Product: '<S275>/Product3' */
  rtb_Switch2_mn = 0.6F * rtb_Add6;

  /* Sum: '<S275>/Add2' */
  rtb_Gain3_o = rtb_Add7 - rtb_Switch2_mn;

  /* UnitDelay: '<S282>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_j;

  /* Sum: '<S282>/Add4' */
  rtb_Add7 = rtb_Gain3_o - rtb_Add7;

  /* Product: '<S282>/Divide' incorporates:
   *  Constant: '<S282>/steptime'
   */
  rtb_CastToBoolean1 = rtb_Add7 / 0.01F;

  /* RelationalOperator: '<S294>/LowerRelop1' incorporates:
   *  Constant: '<S289>/Constant1'
   */
  rtb_Compare = (rtb_CastToBoolean1 > 100.0F);

  /* Switch: '<S294>/Switch2' incorporates:
   *  Constant: '<S289>/Constant1'
   */
  if (rtb_Compare) {
    rtb_CastToBoolean1 = 100.0F;
  } else {
    /* RelationalOperator: '<S294>/UpperRelop' incorporates:
     *  Constant: '<S289>/Constant'
     */
    rtb_LogicalOperator2 = (rtb_CastToBoolean1 < -100.0F);

    /* Switch: '<S294>/Switch' incorporates:
     *  Constant: '<S289>/Constant'
     */
    if (rtb_LogicalOperator2) {
      rtb_CastToBoolean1 = -100.0F;
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
  rtb_CastToBoolean1 -= rtb_Switch2_b0;

  /* RelationalOperator: '<S295>/LowerRelop1' */
  rtb_Compare = (rtb_CastToBoolean1 > rtb_Add4_j);

  /* Switch: '<S295>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S293>/delta fall limit' */
    rtb_deltafalllimit_cz = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S295>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_CastToBoolean1 < rtb_deltafalllimit_cz);

    /* Switch: '<S295>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_CastToBoolean1 = rtb_deltafalllimit_cz;
    }

    /* End of Switch: '<S295>/Switch' */
    rtb_Add4_j = rtb_CastToBoolean1;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2 = rtb_Add4_j +
    rtb_Switch2_b0;

  /* Gain: '<S289>/Gain1' incorporates:
   *  UnitDelay: '<S293>/Delay Input2'
   *
   * Block description for '<S293>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add7 = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* MinMax: '<S335>/Min3' incorporates:
   *  Abs: '<S282>/Abs'
   *  Sum: '<S282>/Add'
   *  Sum: '<S289>/Add'
   *  UnitDelay: '<S289>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p += rtb_Add7;
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p -= rtb_CastToDouble;
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p = fabsf
    (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p);

  /* RelationalOperator: '<S285>/Compare' incorporates:
   *  Constant: '<S285>/Constant'
   *  UnitDelay: '<S289>/Unit Delay'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p <= 0.5F);

  /* UnitDelay: '<S290>/Unit Delay' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj;

  /* Gain: '<S290>/Gain' */
  rtb_Add7 *= 0.3F;

  /* UnitDelay: '<S296>/Delay Input2'
   *
   * Block description for '<S296>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* SampleTimeMath: '<S296>/sample time'
   *
   * About '<S296>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S296>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(100.0 * elapseTime);

  /* MinMax: '<S335>/Min3' incorporates:
   *  DataTypeConversion: '<S208>/Cast To Double2'
   *  UnitDelay: '<S289>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p = (real32_T)WhlSpdFR;

  /* Gain: '<S275>/Gain3' */
  rtb_Add10_b = 0.0174532924F * rtb_FRWhlStrAng;

  /* Trigonometry: '<S275>/Asin1' */
  rtb_Add10_b = cosf(rtb_Add10_b);

  /* Product: '<S275>/Product2' incorporates:
   *  UnitDelay: '<S289>/Unit Delay'
   */
  rtb_Add10_b *= VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p;

  /* Sum: '<S275>/Add3' */
  rtb_CastToBoolean1 = rtb_Switch2_mn + rtb_Add10_b;

  /* UnitDelay: '<S282>/Unit Delay1' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n;

  /* Sum: '<S282>/Add5' */
  rtb_Add10_b = rtb_CastToBoolean1 - rtb_Add10_b;

  /* Product: '<S282>/Divide1' incorporates:
   *  Constant: '<S282>/steptime1'
   */
  rtb_deltafalllimit_cz = rtb_Add10_b / 0.01F;

  /* RelationalOperator: '<S297>/LowerRelop1' incorporates:
   *  Constant: '<S290>/Constant1'
   */
  rtb_LogicalOperator7_m = (rtb_deltafalllimit_cz > 100.0F);

  /* Switch: '<S297>/Switch2' incorporates:
   *  Constant: '<S290>/Constant1'
   */
  if (rtb_LogicalOperator7_m) {
    rtb_deltafalllimit_cz = 100.0F;
  } else {
    /* RelationalOperator: '<S297>/UpperRelop' incorporates:
     *  Constant: '<S290>/Constant'
     */
    rtb_LogicalOperator2 = (rtb_deltafalllimit_cz < -100.0F);

    /* Switch: '<S297>/Switch' incorporates:
     *  Constant: '<S290>/Constant'
     */
    if (rtb_LogicalOperator2) {
      rtb_deltafalllimit_cz = -100.0F;
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
  rtb_deltafalllimit_cz -= rtb_Add4_j;

  /* RelationalOperator: '<S298>/LowerRelop1' */
  rtb_LogicalOperator7_m = (rtb_deltafalllimit_cz > rtb_Switch2_b0);

  /* Switch: '<S298>/Switch2' */
  if (!rtb_LogicalOperator7_m) {
    /* Product: '<S296>/delta fall limit' */
    rtb_deltafalllimit_ap = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S298>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_deltafalllimit_cz < rtb_deltafalllimit_ap);

    /* Switch: '<S298>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_deltafalllimit_cz = rtb_deltafalllimit_ap;
    }

    /* End of Switch: '<S298>/Switch' */
    rtb_Switch2_b0 = rtb_deltafalllimit_cz;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e = rtb_Switch2_b0 +
    rtb_Add4_j;

  /* Gain: '<S290>/Gain1' incorporates:
   *  UnitDelay: '<S296>/Delay Input2'
   *
   * Block description for '<S296>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10_b = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* Sum: '<S290>/Add' */
  rtb_Add7 += rtb_Add10_b;

  /* Sum: '<S282>/Add1' */
  rtb_Add7 -= rtb_CastToDouble;

  /* Abs: '<S282>/Abs1' */
  rtb_Add7 = fabsf(rtb_Add7);

  /* RelationalOperator: '<S286>/Compare' incorporates:
   *  Constant: '<S286>/Constant'
   */
  rtb_LogicalOperator7_m = (rtb_Add7 <= 0.5F);

  /* UnitDelay: '<S291>/Unit Delay' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a;

  /* Gain: '<S291>/Gain' */
  rtb_Add10_b *= 0.3F;

  /* UnitDelay: '<S299>/Delay Input2'
   *
   * Block description for '<S299>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* SampleTimeMath: '<S299>/sample time'
   *
   * About '<S299>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S299>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S208>/Cast To Double3' */
  rtb_Add4_j = (real32_T)WhlSpdRL_mps;

  /* Product: '<S275>/Product' */
  rtb_Add6 *= 0.58F;

  /* Sum: '<S275>/Add' */
  rtb_deltafalllimit_cz = rtb_Add4_j - rtb_Add6;

  /* UnitDelay: '<S282>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_l;

  /* Sum: '<S282>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_cz - rtb_Add4_j;

  /* Product: '<S282>/Divide2' incorporates:
   *  Constant: '<S282>/steptime2'
   */
  rtb_deltafalllimit_ap = rtb_Add4_j / 0.01F;

  /* RelationalOperator: '<S300>/LowerRelop1' incorporates:
   *  Constant: '<S291>/Constant1'
   */
  rtb_Compare_am = (rtb_deltafalllimit_ap > 100.0F);

  /* Switch: '<S300>/Switch2' incorporates:
   *  Constant: '<S291>/Constant1'
   */
  if (rtb_Compare_am) {
    rtb_deltafalllimit_ap = 100.0F;
  } else {
    /* RelationalOperator: '<S300>/UpperRelop' incorporates:
     *  Constant: '<S291>/Constant'
     */
    rtb_LogicalOperator2 = (rtb_deltafalllimit_ap < -100.0F);

    /* Switch: '<S300>/Switch' incorporates:
     *  Constant: '<S291>/Constant'
     */
    if (rtb_LogicalOperator2) {
      rtb_deltafalllimit_ap = -100.0F;
    }

    /* End of Switch: '<S300>/Switch' */
  }

  /* End of Switch: '<S300>/Switch2' */

  /* Sum: '<S299>/Difference Inputs1'
   *
   * Block description for '<S299>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_deltafalllimit_ap -= rtb_Switch2_mn;

  /* RelationalOperator: '<S301>/LowerRelop1' */
  rtb_Compare_am = (rtb_deltafalllimit_ap > rtb_Add7);

  /* Switch: '<S301>/Switch2' */
  if (!rtb_Compare_am) {
    /* Product: '<S299>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S301>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_deltafalllimit_ap < rtb_Add7);

    /* Switch: '<S301>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_deltafalllimit_ap = rtb_Add7;
    }

    /* End of Switch: '<S301>/Switch' */
    rtb_Add7 = rtb_deltafalllimit_ap;
  }

  /* End of Switch: '<S301>/Switch2' */

  /* Sum: '<S299>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S299>/Delay Input2'
   *
   * Block description for '<S299>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S299>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk = rtb_Add7 +
    rtb_Switch2_mn;

  /* Gain: '<S291>/Gain1' incorporates:
   *  UnitDelay: '<S299>/Delay Input2'
   *
   * Block description for '<S299>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* Sum: '<S291>/Add' */
  rtb_Add10_b += rtb_Switch2_mn;

  /* Sum: '<S282>/Add2' */
  rtb_Add10_b -= rtb_CastToDouble;

  /* Abs: '<S282>/Abs2' */
  rtb_Add10_b = fabsf(rtb_Add10_b);

  /* RelationalOperator: '<S287>/Compare' incorporates:
   *  Constant: '<S287>/Constant'
   */
  rtb_Compare_am = (rtb_Add10_b <= 0.5F);

  /* UnitDelay: '<S292>/Unit Delay' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nc;

  /* Gain: '<S292>/Gain' */
  rtb_Add10_b *= 0.3F;

  /* UnitDelay: '<S302>/Delay Input2'
   *
   * Block description for '<S302>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* SampleTimeMath: '<S302>/sample time'
   *
   * About '<S302>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S302>/delta rise limit' */
  rtb_Add7 = (real32_T)(100.0 * elapseTime);

  /* DataTypeConversion: '<S208>/Cast To Double4' */
  rtb_Add4_j = (real32_T)WhlSpdRR_mps;

  /* Sum: '<S275>/Add1' */
  rtb_deltafalllimit_ap = rtb_Add6 + rtb_Add4_j;

  /* UnitDelay: '<S282>/Unit Delay3' */
  rtb_Add6 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE;

  /* Sum: '<S282>/Add7' */
  rtb_Add6 = rtb_deltafalllimit_ap - rtb_Add6;

  /* Product: '<S282>/Divide3' incorporates:
   *  Constant: '<S282>/steptime3'
   */
  rtb_Add6 /= 0.01F;

  /* RelationalOperator: '<S303>/LowerRelop1' incorporates:
   *  Constant: '<S292>/Constant1'
   */
  rtb_LogicalOperator2 = (rtb_Add6 > 100.0F);

  /* Switch: '<S303>/Switch2' incorporates:
   *  Constant: '<S292>/Constant1'
   */
  if (rtb_LogicalOperator2) {
    rtb_Add6 = 100.0F;
  } else {
    /* RelationalOperator: '<S303>/UpperRelop' incorporates:
     *  Constant: '<S292>/Constant'
     */
    rtb_LogicalOperator2 = (rtb_Add6 < -100.0F);

    /* Switch: '<S303>/Switch' incorporates:
     *  Constant: '<S292>/Constant'
     */
    if (rtb_LogicalOperator2) {
      rtb_Add6 = -100.0F;
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
  rtb_Add6 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S304>/LowerRelop1' */
  rtb_LogicalOperator2 = (rtb_Add6 > rtb_Add7);

  /* Switch: '<S304>/Switch2' */
  if (!rtb_LogicalOperator2) {
    /* Product: '<S302>/delta fall limit' */
    rtb_Add7 = (real32_T)(-100.0 * elapseTime);

    /* RelationalOperator: '<S304>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Add6 < rtb_Add7);

    /* Switch: '<S304>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Add6 = rtb_Add7;
    }

    /* End of Switch: '<S304>/Switch' */
    rtb_Add7 = rtb_Add6;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c = rtb_Add7 +
    rtb_Switch2_mn;

  /* Gain: '<S292>/Gain1' incorporates:
   *  UnitDelay: '<S302>/Delay Input2'
   *
   * Block description for '<S302>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.7F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* Sum: '<S292>/Add' */
  rtb_Add10_b += rtb_Switch2_mn;

  /* Sum: '<S282>/Add3' */
  rtb_Add10_b -= rtb_CastToDouble;

  /* Abs: '<S282>/Abs3' */
  rtb_Add10_b = fabsf(rtb_Add10_b);

  /* RelationalOperator: '<S288>/Compare' incorporates:
   *  Constant: '<S288>/Constant'
   */
  rtb_LogicalOperator2 = (rtb_Add10_b <= 0.5F);

  /* UnitDelay: '<S309>/Unit Delay' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l;

  /* Gain: '<S309>/Gain' */
  rtb_Add10_b *= 0.5F;

  /* UnitDelay: '<S313>/Delay Input2'
   *
   * Block description for '<S313>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* SampleTimeMath: '<S313>/sample time'
   *
   * About '<S313>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S313>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S283>/Unit Delay4' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE;

  /* UnitDelay: '<S283>/Unit Delay' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a0;

  /* Sum: '<S283>/Add4' */
  rtb_Add4_j = rtb_Gain3_o - rtb_Add4_j;

  /* Product: '<S283>/Divide' incorporates:
   *  Constant: '<S283>/steptime'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S283>/Add' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S283>/Add8' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE - rtb_Add7;

  /* Product: '<S283>/Divide4' incorporates:
   *  Constant: '<S283>/steptime4'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S314>/LowerRelop1' incorporates:
   *  Constant: '<S309>/Constant1'
   */
  rtb_LowerRelop1_b = (rtb_Add7 > 100.0F);

  /* Switch: '<S314>/Switch2' incorporates:
   *  Constant: '<S309>/Constant1'
   */
  if (rtb_LowerRelop1_b) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S314>/UpperRelop' incorporates:
     *  Constant: '<S309>/Constant'
     */
    rtb_LowerRelop1_b = (rtb_Add7 < -100.0F);

    /* Switch: '<S314>/Switch' incorporates:
     *  Constant: '<S309>/Constant'
     */
    if (rtb_LowerRelop1_b) {
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
  rtb_LowerRelop1_b = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S315>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S313>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S315>/UpperRelop' */
    rtb_LowerRelop1_b = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S315>/Switch' */
    if (rtb_LowerRelop1_b) {
      rtb_Add7 = rtb_Add6;
    }

    /* End of Switch: '<S315>/Switch' */
    rtb_Add6 = rtb_Add7;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S309>/Gain1' incorporates:
   *  UnitDelay: '<S313>/Delay Input2'
   *
   * Block description for '<S313>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* Sum: '<S309>/Add' */
  rtb_Add10_b += rtb_Switch2_mn;

  /* Abs: '<S283>/Abs' */
  rtb_Add10_b = fabsf(rtb_Add10_b);

  /* RelationalOperator: '<S305>/Compare' incorporates:
   *  Constant: '<S305>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Add10_b <= 0.8F);

  /* UnitDelay: '<S310>/Unit Delay' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ap;

  /* Gain: '<S310>/Gain' */
  rtb_Add10_b *= 0.5F;

  /* UnitDelay: '<S316>/Delay Input2'
   *
   * Block description for '<S316>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* SampleTimeMath: '<S316>/sample time'
   *
   * About '<S316>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S316>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S283>/Unit Delay5' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE;

  /* UnitDelay: '<S283>/Unit Delay1' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_a;

  /* Sum: '<S283>/Add5' */
  rtb_Add4_j = rtb_CastToBoolean1 - rtb_Add4_j;

  /* Product: '<S283>/Divide1' incorporates:
   *  Constant: '<S283>/steptime1'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S283>/Add1' incorporates:
   *  UnitDelay: '<S283>/Unit Delay5'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S283>/Add10' incorporates:
   *  UnitDelay: '<S283>/Unit Delay5'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE - rtb_Add7;

  /* Product: '<S283>/Divide5' incorporates:
   *  Constant: '<S283>/steptime5'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S317>/LowerRelop1' incorporates:
   *  Constant: '<S310>/Constant1'
   */
  rtb_AND2_e = (rtb_Add7 > 100.0F);

  /* Switch: '<S317>/Switch2' incorporates:
   *  Constant: '<S310>/Constant1'
   */
  if (rtb_AND2_e) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S317>/UpperRelop' incorporates:
     *  Constant: '<S310>/Constant'
     */
    rtb_AND2_e = (rtb_Add7 < -100.0F);

    /* Switch: '<S317>/Switch' incorporates:
     *  Constant: '<S310>/Constant'
     */
    if (rtb_AND2_e) {
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
  rtb_AND2_e = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S318>/Switch2' */
  if (!rtb_AND2_e) {
    /* Product: '<S316>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S318>/UpperRelop' */
    rtb_AND2_e = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S318>/Switch' */
    if (rtb_AND2_e) {
      rtb_Add7 = rtb_Add6;
    }

    /* End of Switch: '<S318>/Switch' */
    rtb_Add6 = rtb_Add7;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S310>/Gain1' incorporates:
   *  UnitDelay: '<S316>/Delay Input2'
   *
   * Block description for '<S316>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* Sum: '<S310>/Add' */
  rtb_Add10_b += rtb_Switch2_mn;

  /* Abs: '<S283>/Abs1' */
  rtb_Add10_b = fabsf(rtb_Add10_b);

  /* RelationalOperator: '<S306>/Compare' incorporates:
   *  Constant: '<S306>/Constant'
   */
  rtb_AND2_e = (rtb_Add10_b <= 0.8F);

  /* UnitDelay: '<S311>/Unit Delay' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o;

  /* Gain: '<S311>/Gain' */
  rtb_Add10_b *= 0.5F;

  /* UnitDelay: '<S319>/Delay Input2'
   *
   * Block description for '<S319>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pd;

  /* SampleTimeMath: '<S319>/sample time'
   *
   * About '<S319>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S319>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S283>/Unit Delay6' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE;

  /* UnitDelay: '<S283>/Unit Delay2' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_c;

  /* Sum: '<S283>/Add6' */
  rtb_Add4_j = rtb_deltafalllimit_cz - rtb_Add4_j;

  /* Product: '<S283>/Divide2' incorporates:
   *  Constant: '<S283>/steptime2'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S283>/Add2' incorporates:
   *  UnitDelay: '<S283>/Unit Delay6'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S283>/Add12' incorporates:
   *  UnitDelay: '<S283>/Unit Delay6'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay6_DSTATE - rtb_Add7;

  /* Product: '<S283>/Divide6' incorporates:
   *  Constant: '<S283>/steptime6'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S320>/LowerRelop1' incorporates:
   *  Constant: '<S311>/Constant1'
   */
  rtb_Compare_b = (rtb_Add7 > 100.0F);

  /* Switch: '<S320>/Switch2' incorporates:
   *  Constant: '<S311>/Constant1'
   */
  if (rtb_Compare_b) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S320>/UpperRelop' incorporates:
     *  Constant: '<S311>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S320>/Switch' incorporates:
     *  Constant: '<S311>/Constant'
     */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S320>/Switch' */
  }

  /* End of Switch: '<S320>/Switch2' */

  /* Sum: '<S319>/Difference Inputs1'
   *
   * Block description for '<S319>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S321>/LowerRelop1' */
  rtb_Compare_b = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S321>/Switch2' */
  if (!rtb_Compare_b) {
    /* Product: '<S319>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S321>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S321>/Switch' */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = rtb_Add6;
    }

    /* End of Switch: '<S321>/Switch' */
    rtb_Add6 = rtb_Add7;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pd = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S311>/Gain1' incorporates:
   *  UnitDelay: '<S319>/Delay Input2'
   *
   * Block description for '<S319>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pd;

  /* Sum: '<S311>/Add' */
  rtb_Add10_b += rtb_Switch2_mn;

  /* Abs: '<S283>/Abs2' */
  rtb_Add10_b = fabsf(rtb_Add10_b);

  /* RelationalOperator: '<S307>/Compare' incorporates:
   *  Constant: '<S307>/Constant'
   */
  rtb_Compare_b = (rtb_Add10_b <= 0.8F);

  /* UnitDelay: '<S312>/Unit Delay' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ah;

  /* Gain: '<S312>/Gain' */
  rtb_Add10_b *= 0.5F;

  /* UnitDelay: '<S322>/Delay Input2'
   *
   * Block description for '<S322>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* SampleTimeMath: '<S322>/sample time'
   *
   * About '<S322>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S322>/delta rise limit' */
  rtb_Add6 = (real32_T)(20.0 * elapseTime);

  /* UnitDelay: '<S283>/Unit Delay7' */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE;

  /* UnitDelay: '<S283>/Unit Delay3' */
  rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_d;

  /* Sum: '<S283>/Add7' */
  rtb_Add4_j = rtb_deltafalllimit_ap - rtb_Add4_j;

  /* Product: '<S283>/Divide3' incorporates:
   *  Constant: '<S283>/steptime3'
   */
  rtb_Add4_j /= 0.01F;

  /* Sum: '<S283>/Add3' incorporates:
   *  UnitDelay: '<S283>/Unit Delay7'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE = rtb_Add4_j -
    rtb_CastToDouble;

  /* Sum: '<S283>/Add14' incorporates:
   *  UnitDelay: '<S283>/Unit Delay7'
   */
  rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay7_DSTATE - rtb_Add7;

  /* Product: '<S283>/Divide7' incorporates:
   *  Constant: '<S283>/steptime7'
   */
  rtb_Add7 /= 0.01F;

  /* RelationalOperator: '<S323>/LowerRelop1' incorporates:
   *  Constant: '<S312>/Constant1'
   */
  rtb_UpperRelop_ir = (rtb_Add7 > 100.0F);

  /* Switch: '<S323>/Switch2' incorporates:
   *  Constant: '<S312>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Add7 = 100.0F;
  } else {
    /* RelationalOperator: '<S323>/UpperRelop' incorporates:
     *  Constant: '<S312>/Constant'
     */
    rtb_UpperRelop_ir = (rtb_Add7 < -100.0F);

    /* Switch: '<S323>/Switch' incorporates:
     *  Constant: '<S312>/Constant'
     */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = -100.0F;
    }

    /* End of Switch: '<S323>/Switch' */
  }

  /* End of Switch: '<S323>/Switch2' */

  /* Sum: '<S322>/Difference Inputs1'
   *
   * Block description for '<S322>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add7 -= rtb_Switch2_mn;

  /* RelationalOperator: '<S324>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add7 > rtb_Add6);

  /* Switch: '<S324>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S322>/delta fall limit' */
    rtb_Add6 = (real32_T)(-20.0 * elapseTime);

    /* RelationalOperator: '<S324>/UpperRelop' */
    rtb_UpperRelop_ir = (rtb_Add7 < rtb_Add6);

    /* Switch: '<S324>/Switch' */
    if (rtb_UpperRelop_ir) {
      rtb_Add7 = rtb_Add6;
    }

    /* End of Switch: '<S324>/Switch' */
    rtb_Add6 = rtb_Add7;
  }

  /* End of Switch: '<S324>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt = rtb_Add6 +
    rtb_Switch2_mn;

  /* Gain: '<S312>/Gain1' incorporates:
   *  UnitDelay: '<S322>/Delay Input2'
   *
   * Block description for '<S322>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_mn = 0.5F * VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* Sum: '<S312>/Add' */
  rtb_Add10_b += rtb_Switch2_mn;

  /* Abs: '<S283>/Abs3' */
  rtb_Add10_b = fabsf(rtb_Add10_b);

  /* RelationalOperator: '<S308>/Compare' incorporates:
   *  Constant: '<S308>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10_b <= 0.8F);

  /* Logic: '<S273>/Logical Operator' */
  rtb_LogicalOperator_idx_0 = (rtb_Compare || rtb_LowerRelop1_b);
  rtb_LogicalOperator7_m = (rtb_LogicalOperator7_m || rtb_AND2_e);
  rtb_Compare_am = (rtb_Compare_am || rtb_Compare_b);
  rtb_Compare = (rtb_LogicalOperator2 || rtb_UpperRelop_ir);

  /* UnitDelay: '<S208>/Unit Delay' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_lh;

  /* Sum: '<S284>/Add' */
  rtb_Switch2_mn = rtb_Gain3_o - rtb_Add10_b;

  /* Abs: '<S284>/Abs' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S325>/Compare' incorporates:
   *  Constant: '<S325>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S284>/AND3' */
  rtb_Compare_b = (rtb_UpperRelop_ir && (VehCtrlMdel240926_2018b_amksp_B.Exit_h
    != 0.0));

  /* Sum: '<S284>/Add1' */
  rtb_Switch2_mn = rtb_CastToBoolean1 - rtb_Add10_b;

  /* Abs: '<S284>/Abs1' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S326>/Compare' incorporates:
   *  Constant: '<S326>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S284>/AND2' */
  rtb_AND2_e = (rtb_UpperRelop_ir && (VehCtrlMdel240926_2018b_amksp_B.Exit_o4 !=
    0.0));

  /* Sum: '<S284>/Add2' */
  rtb_Switch2_mn = rtb_deltafalllimit_cz - rtb_Add10_b;

  /* Abs: '<S284>/Abs2' */
  rtb_Switch2_mn = fabsf(rtb_Switch2_mn);

  /* RelationalOperator: '<S327>/Compare' incorporates:
   *  Constant: '<S327>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Switch2_mn <= 2.0F);

  /* Logic: '<S284>/AND' */
  rtb_LowerRelop1_b = (rtb_UpperRelop_ir &&
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_le != 0.0));

  /* Sum: '<S284>/Add3' */
  rtb_Add10_b = rtb_deltafalllimit_ap - rtb_Add10_b;

  /* Abs: '<S284>/Abs3' */
  rtb_Add10_b = fabsf(rtb_Add10_b);

  /* RelationalOperator: '<S328>/Compare' incorporates:
   *  Constant: '<S328>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10_b <= 2.0F);

  /* Logic: '<S284>/AND1' */
  rtb_LogicalOperator2 = (rtb_UpperRelop_ir &&
    (VehCtrlMdel240926_2018b_amksp_B.Exit_is != 0.0));

  /* Logic: '<S273>/Logical Operator1' */
  rtb_UpperRelop_ir = (rtb_Compare_b && rtb_LogicalOperator_idx_0);
  rtb_LogicalOperator7_m = (rtb_AND2_e && rtb_LogicalOperator7_m);
  rtb_Compare_am = (rtb_LowerRelop1_b && rtb_Compare_am);
  rtb_LogicalOperator2 = (rtb_LogicalOperator2 && rtb_Compare);

  /* Chart: '<S273>/Timer' incorporates:
   *  Constant: '<S273>/Constant1'
   */
  VehCtrlMdel240926_20_Timer1(rtb_UpperRelop_ir, 0.5F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_c,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer_o);

  /* Chart: '<S273>/Timer1' incorporates:
   *  Constant: '<S273>/Constant2'
   */
  VehCtrlMdel240926_20_Timer1(rtb_LogicalOperator7_m, 0.5F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_lh4,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_m);

  /* Chart: '<S273>/Timer2' incorporates:
   *  Constant: '<S273>/Constant3'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_am, 0.5F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_lh,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_g);

  /* Chart: '<S273>/Timer3' incorporates:
   *  Constant: '<S273>/Constant4'
   */
  VehCtrlMdel240926_20_Timer1(rtb_LogicalOperator2, 0.5F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_a,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer3_i);

  /* Logic: '<S271>/Logical Operator' */
  rtb_UpperRelop_ir = ((VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) ||
                       (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0));

  /* Logic: '<S271>/Logical Operator1' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* UnitDelay: '<S271>/Unit Delay4' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_m;

  /* Sum: '<S271>/Add1' */
  rtb_Add10_b = Acc_POS_n - rtb_Add10_b;

  /* RelationalOperator: '<S276>/Compare' incorporates:
   *  Constant: '<S276>/Constant'
   */
  rtb_Compare_b = (rtb_Add10_b > 0.1F);

  /* Logic: '<S271>/Logical Operator2' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir || rtb_Compare_b);

  /* Logic: '<S271>/AND' */
  rtb_LogicalOperator2 = ((VehCtrlMdel240926_2018b_amksp_B.CANUnpack_o1 != 0.0) &&
    rtb_UpperRelop_ir);

  /* UnitDelay: '<S271>/Unit Delay3' */
  rtb_UpperRelop_ir = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_f;

  /* Logic: '<S271>/Logical Operator3' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S271>/Switch3' incorporates:
   *  UnitDelay: '<S271>/Unit Delay1'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S271>/Switch4' incorporates:
     *  Constant: '<S271>/InitZORE'
     */
    if (!rtb_LogicalOperator2) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k = 0.0F;
    }

    /* End of Switch: '<S271>/Switch4' */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_o =
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k;
  }

  /* End of Switch: '<S271>/Switch3' */

  /* UnitDelay: '<S274>/Unit Delay3' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_p;

  /* Sum: '<S274>/Add5' incorporates:
   *  UnitDelay: '<S274>/Unit Delay1'
   */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d -
    rtb_Add10_b;

  /* Product: '<S274>/Divide3' incorporates:
   *  Constant: '<S274>/steptime3'
   */
  rtb_Add10_b /= 0.01F;

  /* UnitDelay: '<S274>/Unit Delay2' */
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_f;

  /* Sum: '<S274>/Add9' */
  rtb_Switch2_mn -= rtb_Add10_b;

  /* UnitDelay: '<S274>/Unit Delay4' */
  rtb_Add6 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn;

  /* Sum: '<S274>/Add6' incorporates:
   *  Constant: '<S274>/steptime4'
   */
  rtb_Add6 += 0.1F;

  /* Sum: '<S274>/Add8' incorporates:
   *  Constant: '<S274>/steptime6'
   */
  rtb_Add7 = rtb_Add6 + 2.0F;

  /* Product: '<S274>/Divide5' */
  rtb_Add7 = 1.0F / rtb_Add7 * rtb_Add6;

  /* Logic: '<S274>/Logical Operator' */
  rtb_Compare = ((VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) ||
                 (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) ||
                 (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) ||
                 (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0));

  /* Switch: '<S274>/Switch13' incorporates:
   *  Constant: '<S274>/Constant10'
   */
  if (rtb_Compare) {
    rtb_Add4_j = rtb_Add7;
  } else {
    rtb_Add4_j = 1.0F;
  }

  /* End of Switch: '<S274>/Switch13' */

  /* Product: '<S274>/Divide6' */
  rtb_Switch2_mn *= rtb_Add4_j;

  /* Sum: '<S274>/Add10' */
  rtb_Ax = rtb_Switch2_mn + rtb_Add10_b;

  /* Switch: '<S271>/Switch1' */
  if (rtb_LogicalOperator2) {
    /* Saturate: '<S271>/Saturation1' */
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d > 200.0F) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = 200.0F;
    } else {
      if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d < -10.0F) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = -10.0F;
      }
    }

    /* Product: '<S271>/Product' incorporates:
     *  Constant: '<S271>/steptime1'
     */
    rtb_Switch2_mn = rtb_Ax * 0.01F;

    /* Saturate: '<S271>/Saturation1' incorporates:
     *  Sum: '<S271>/Add'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d += rtb_Switch2_mn;
  } else {
    /* Saturate: '<S271>/Saturation1' incorporates:
     *  Constant: '<S271>/Constant'
     *  UnitDelay: '<S271>/Unit Delay'
     */
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d = 0.0F;
  }

  /* End of Switch: '<S271>/Switch1' */

  /* Saturate: '<S271>/Saturation' */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d > 200.0F) {
    rtb_Add10_b = 200.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d < -10.0F) {
    rtb_Add10_b = -10.0F;
  } else {
    rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_d;
  }

  /* End of Saturate: '<S271>/Saturation' */

  /* Sum: '<S271>/Add3' incorporates:
   *  UnitDelay: '<S271>/Unit Delay1'
   */
  rtb_VxIMU_est = rtb_Add10_b +
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_o;

  /* MinMax: '<S273>/Min1' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Gain3_o, rtb_CastToBoolean1);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_cz);
  rtb_Add10_b = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_ap);

  /* Sum: '<S271>/Add2' */
  rtb_Add10_b -= rtb_VxIMU_est;

  /* RelationalOperator: '<S277>/Compare' incorporates:
   *  Constant: '<S277>/Constant'
   */
  rtb_UpperRelop_ir = (rtb_Add10_b <= 0.0F);

  /* Switch: '<S271>/Switch6' incorporates:
   *  Constant: '<S271>/Reset'
   */
  if (rtb_UpperRelop_ir) {
    /* Sum: '<S271>/Add10' incorporates:
     *  Constant: '<S271>/Steptime'
     */
    rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i + 0.01F;
  } else {
    rtb_Add10_b = 0.0F;
  }

  /* End of Switch: '<S271>/Switch6' */

  /* MinMax: '<S271>/Min' incorporates:
   *  Constant: '<S271>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i = fminf(rtb_Add10_b, 0.1F);

  /* RelationalOperator: '<S271>/Relational Operator9' incorporates:
   *  Constant: '<S271>/ResetDelay'
   *  UnitDelay: '<S271>/Unit Delay2'
   */
  rtb_LogicalOperator7_m = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_i >=
    0.1F);

  /* RelationalOperator: '<S330>/Compare' incorporates:
   *  Constant: '<S330>/Constant'
   */
  rtb_Compare_am = (rtb_Ax < -0.5F);

  /* Chart: '<S274>/Timer2' incorporates:
   *  Constant: '<S274>/Constant15'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_i,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_j);

  /* UnitDelay: '<S334>/Delay Input2'
   *
   * Block description for '<S334>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;

  /* SampleTimeMath: '<S334>/sample time'
   *
   * About '<S334>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S334>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(10.0 * elapseTime);

  /* Sum: '<S335>/Add3' */
  rtb_Add4_j = ((rtb_deltafalllimit_ap + rtb_deltafalllimit_cz) +
                rtb_CastToBoolean1) + rtb_Gain3_o;

  /* MinMax: '<S335>/Min4' */
  rtb_Switch2_b0 = fminf(rtb_Gain3_o, rtb_CastToBoolean1);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_cz);
  rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_deltafalllimit_ap);

  /* MinMax: '<S335>/Min3' */
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Gain3_o, rtb_CastToBoolean1);
  rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_deltafalllimit_cz);
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p = fmaxf(rtb_MaxWhlSpd_mps_n,
    rtb_deltafalllimit_ap);

  /* Sum: '<S335>/Add4' incorporates:
   *  UnitDelay: '<S289>/Unit Delay'
   */
  rtb_Add4_j = (rtb_Add4_j - rtb_Switch2_b0) -
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p;

  /* Gain: '<S335>/Gain1' */
  rtb_Add4_j *= 0.5F;

  /* Sum: '<S334>/Difference Inputs1'
   *
   * Block description for '<S334>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add4_j -= rtb_Add10_b;

  /* RelationalOperator: '<S343>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Add4_j > rtb_Switch2_mn);

  /* Switch: '<S343>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S334>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-10.0 * elapseTime);

    /* RelationalOperator: '<S343>/UpperRelop' */
    rtb_Compare_am = (rtb_Add4_j < rtb_Switch2_mn);

    /* Switch: '<S343>/Switch' */
    if (rtb_Compare_am) {
      rtb_Add4_j = rtb_Switch2_mn;
    }

    /* End of Switch: '<S343>/Switch' */
    rtb_Switch2_mn = rtb_Add4_j;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g = rtb_Switch2_mn +
    rtb_Add10_b;

  /* RelationalOperator: '<S329>/Compare' incorporates:
   *  Constant: '<S329>/Constant'
   */
  rtb_Compare_am = (rtb_Ax > 0.5F);

  /* Chart: '<S274>/Timer1' incorporates:
   *  Constant: '<S274>/Constant14'
   */
  VehCtrlMdel240926_20_Timer1(rtb_Compare_am, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_l,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_p);

  /* Logic: '<S274>/Logical Operator2' */
  rtb_UpperRelop_ir = !(VehCtrlMdel240926_2018b_amksp_B.Exit_l != 0.0);

  /* Switch: '<S274>/Switch6' incorporates:
   *  Switch: '<S274>/Switch4'
   */
  if (rtb_UpperRelop_ir) {
    /* Switch: '<S274>/Switch5' incorporates:
     *  UnitDelay: '<S334>/Delay Input2'
     *
     * Block description for '<S334>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_i != 0.0) {
      /* Switch: '<S274>/Switch11' incorporates:
       *  Constant: '<S274>/Constant7'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0) {
        rtb_Switch2_mn = rtb_deltafalllimit_ap;
      } else {
        rtb_Switch2_mn = 0.0F;
      }

      /* End of Switch: '<S274>/Switch11' */

      /* Switch: '<S274>/Switch10' incorporates:
       *  Constant: '<S274>/Constant6'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) {
        rtb_Add10_b = rtb_deltafalllimit_cz;
      } else {
        rtb_Add10_b = 0.0F;
      }

      /* End of Switch: '<S274>/Switch10' */

      /* Switch: '<S274>/Switch9' incorporates:
       *  Constant: '<S274>/Constant5'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) {
        rtb_Add4_j = rtb_CastToBoolean1;
      } else {
        rtb_Add4_j = 0.0F;
      }

      /* End of Switch: '<S274>/Switch9' */

      /* Switch: '<S274>/Switch8' incorporates:
       *  Constant: '<S274>/Constant4'
       */
      if (VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) {
        rtb_Switch2_b0 = rtb_Gain3_o;
      } else {
        rtb_Switch2_b0 = 0.0F;
      }

      /* End of Switch: '<S274>/Switch8' */

      /* MinMax: '<S274>/Min1' */
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_Switch2_b0, rtb_Add4_j);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Add10_b);
      rtb_MaxWhlSpd_mps_n = fmaxf(rtb_MaxWhlSpd_mps_n, rtb_Switch2_mn);
    } else {
      rtb_MaxWhlSpd_mps_n = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;
    }

    /* End of Switch: '<S274>/Switch5' */
  } else {
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_a != 0.0) {
      /* Switch: '<S274>/Switch4' */
      rtb_Switch2_mn = rtb_deltafalllimit_ap;
    } else {
      /* Switch: '<S274>/Switch4' incorporates:
       *  Constant: '<S274>/Constant3'
       */
      rtb_Switch2_mn = 9999.0F;
    }

    /* Switch: '<S274>/Switch3' incorporates:
     *  Constant: '<S274>/Constant2'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh != 0.0) {
      rtb_Add10_b = rtb_deltafalllimit_cz;
    } else {
      rtb_Add10_b = 9999.0F;
    }

    /* End of Switch: '<S274>/Switch3' */

    /* Switch: '<S274>/Switch2' incorporates:
     *  Constant: '<S274>/Constant1'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_lh4 != 0.0) {
      rtb_Add4_j = rtb_CastToBoolean1;
    } else {
      rtb_Add4_j = 9999.0F;
    }

    /* End of Switch: '<S274>/Switch2' */

    /* Switch: '<S274>/Switch1' incorporates:
     *  Constant: '<S274>/Constant'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.Exit_c != 0.0) {
      rtb_Switch2_b0 = rtb_Gain3_o;
    } else {
      rtb_Switch2_b0 = 9999.0F;
    }

    /* End of Switch: '<S274>/Switch1' */

    /* MinMax: '<S274>/Min2' */
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add4_j);
    rtb_Switch2_b0 = fminf(rtb_Switch2_b0, rtb_Add10_b);
    rtb_MaxWhlSpd_mps_n = fminf(rtb_Switch2_b0, rtb_Switch2_mn);
  }

  /* End of Switch: '<S274>/Switch6' */

  /* Logic: '<S274>/NOT3' */
  rtb_UpperRelop_ir = !rtb_Compare;

  /* Logic: '<S274>/Logical Operator3' */
  rtb_UpperRelop_ir = (rtb_UpperRelop_ir && rtb_LogicalOperator7_m);

  /* Logic: '<S274>/NOT4' */
  rtb_UpperRelop_ir = !rtb_UpperRelop_ir;

  /* Switch: '<S274>/Switch7' incorporates:
   *  UnitDelay: '<S334>/Delay Input2'
   *
   * Block description for '<S334>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (rtb_UpperRelop_ir) {
    /* Logic: '<S274>/Logical Operator1' */
    rtb_LogicalOperator7_m = (rtb_LogicalOperator7_m || rtb_Compare);

    /* Switch: '<S274>/Switch' */
    if (rtb_LogicalOperator7_m) {
      rtb_VxIMU_est = rtb_MaxWhlSpd_mps_n;
    }

    /* End of Switch: '<S274>/Switch' */
  } else {
    rtb_VxIMU_est = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_g;
  }

  /* End of Switch: '<S274>/Switch7' */

  /* UnitDelay: '<S332>/Delay Input2'
   *
   * Block description for '<S332>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a;

  /* Sum: '<S332>/Difference Inputs1'
   *
   * Block description for '<S332>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_VxIMU_est -= rtb_Add10_b;

  /* Switch: '<S274>/Switch12' incorporates:
   *  Constant: '<S274>/Constant8'
   *  Constant: '<S274>/Constant9'
   */
  if (rtb_Compare) {
    rtb_Switch2_mn = 0.1F;
  } else {
    rtb_Switch2_mn = 0.05F;
  }

  /* End of Switch: '<S274>/Switch12' */

  /* Sum: '<S274>/Add4' */
  rtb_Add4_j = rtb_Ax + rtb_Switch2_mn;

  /* SampleTimeMath: '<S332>/sample time'
   *
   * About '<S332>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S332>/delta rise limit' */
  rtb_Switch2_b0 = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S341>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_VxIMU_est > rtb_Switch2_b0);

  /* Sum: '<S274>/Add3' */
  rtb_Ax -= rtb_Switch2_mn;

  /* Switch: '<S341>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S332>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S341>/UpperRelop' */
    rtb_Compare = (rtb_VxIMU_est < rtb_Switch2_mn);

    /* Switch: '<S341>/Switch' */
    if (rtb_Compare) {
      rtb_VxIMU_est = rtb_Switch2_mn;
    }

    /* End of Switch: '<S341>/Switch' */
    rtb_Switch2_b0 = rtb_VxIMU_est;
  }

  /* End of Switch: '<S341>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a = rtb_Switch2_b0 +
    rtb_Add10_b;

  /* RelationalOperator: '<S339>/LowerRelop1' incorporates:
   *  Constant: '<S331>/Constant1'
   *  UnitDelay: '<S332>/Delay Input2'
   *
   * Block description for '<S332>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UpperRelop_ir = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a >
                       100.0F);

  /* Switch: '<S339>/Switch2' incorporates:
   *  Constant: '<S331>/Constant1'
   */
  if (rtb_UpperRelop_ir) {
    rtb_Switch2_mn = 100.0F;
  } else {
    /* RelationalOperator: '<S339>/UpperRelop' incorporates:
     *  Constant: '<S331>/Constant'
     *  UnitDelay: '<S332>/Delay Input2'
     *
     * Block description for '<S332>/Delay Input2':
     *
     *  Store in Global RAM
     */
    rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a < 0.0F);

    /* Switch: '<S339>/Switch' incorporates:
     *  Constant: '<S331>/Constant'
     *  UnitDelay: '<S332>/Delay Input2'
     *
     * Block description for '<S332>/Delay Input2':
     *
     *  Store in Global RAM
     */
    if (rtb_Compare) {
      rtb_Switch2_mn = 0.0F;
    } else {
      rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_a;
    }

    /* End of Switch: '<S339>/Switch' */
  }

  /* End of Switch: '<S339>/Switch2' */

  /* UnitDelay: '<S338>/Delay Input2'
   *
   * Block description for '<S338>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* Sum: '<S338>/Difference Inputs1'
   *
   * Block description for '<S338>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_Switch2_mn - rtb_Add10_b;

  /* SampleTimeMath: '<S338>/sample time'
   *
   * About '<S338>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S338>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(15.0 * elapseTime);

  /* RelationalOperator: '<S340>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S340>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S338>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(-15.0 * elapseTime);

    /* RelationalOperator: '<S340>/UpperRelop' */
    rtb_Compare = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S340>/Switch' */
    if (rtb_Compare) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S340>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S340>/Switch2' */

  /* Sum: '<S338>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S338>/Delay Input2'
   *
   * Block description for '<S338>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S338>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f = rtb_Switch2_mn +
    rtb_Add10_b;

  /* UnitDelay: '<S331>/Unit Delay' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ncs;

  /* Gain: '<S331>/Gain' */
  rtb_Add10_b *= 0.0F;

  /* Saturate: '<S32>/Saturation' incorporates:
   *  Sum: '<S331>/Add'
   *  UnitDelay: '<S338>/Delay Input2'
   *
   * Block description for '<S338>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehVxEst_mps = rtb_Add10_b +
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* SampleTimeMath: '<S333>/sample time'
   *
   * About '<S333>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* UnitDelay: '<S333>/Delay Input2'
   *
   * Block description for '<S333>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu;

  /* Sum: '<S333>/Difference Inputs1'
   *
   * Block description for '<S333>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_b0 = rtb_MaxWhlSpd_mps_n - rtb_Add10_b;

  /* Product: '<S333>/delta rise limit' */
  rtb_Switch2_mn = (real32_T)(rtb_Add4_j * elapseTime);

  /* RelationalOperator: '<S342>/LowerRelop1' */
  rtb_UpperRelop_ir = (rtb_Switch2_b0 > rtb_Switch2_mn);

  /* Switch: '<S342>/Switch2' */
  if (!rtb_UpperRelop_ir) {
    /* Product: '<S333>/delta fall limit' */
    rtb_Switch2_mn = (real32_T)(rtb_Ax * elapseTime);

    /* RelationalOperator: '<S342>/UpperRelop' */
    rtb_Compare = (rtb_Switch2_b0 < rtb_Switch2_mn);

    /* Switch: '<S342>/Switch' */
    if (rtb_Compare) {
      rtb_Switch2_b0 = rtb_Switch2_mn;
    }

    /* End of Switch: '<S342>/Switch' */
    rtb_Switch2_mn = rtb_Switch2_b0;
  }

  /* End of Switch: '<S342>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu = rtb_Switch2_mn +
    rtb_Add10_b;

  /* Sum: '<S274>/Add7' incorporates:
   *  Constant: '<S274>/steptime5'
   */
  rtb_Add7 = 1.0F - rtb_Add7;

  /* Product: '<S274>/Divide4' incorporates:
   *  UnitDelay: '<S274>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn = rtb_Add7 * rtb_Add6;

  /* Update for MinMax: '<S335>/Min3' incorporates:
   *  UnitDelay: '<S289>/Unit Delay'
   *  UnitDelay: '<S293>/Delay Input2'
   *
   * Block description for '<S293>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_p =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_n2;

  /* Update for UnitDelay: '<S282>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_j = rtb_Gain3_o;

  /* Update for UnitDelay: '<S290>/Unit Delay' incorporates:
   *  UnitDelay: '<S296>/Delay Input2'
   *
   * Block description for '<S296>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_pj =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_e;

  /* Update for UnitDelay: '<S282>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n = rtb_CastToBoolean1;

  /* Update for UnitDelay: '<S291>/Unit Delay' incorporates:
   *  UnitDelay: '<S299>/Delay Input2'
   *
   * Block description for '<S299>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hk;

  /* Update for UnitDelay: '<S282>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_l = rtb_deltafalllimit_cz;

  /* Update for UnitDelay: '<S292>/Unit Delay' incorporates:
   *  UnitDelay: '<S302>/Delay Input2'
   *
   * Block description for '<S302>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nc =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_c;

  /* Update for UnitDelay: '<S282>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE = rtb_deltafalllimit_ap;

  /* Update for UnitDelay: '<S309>/Unit Delay' incorporates:
   *  UnitDelay: '<S313>/Delay Input2'
   *
   * Block description for '<S313>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_i;

  /* Update for UnitDelay: '<S283>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_a0 = rtb_Gain3_o;

  /* Update for UnitDelay: '<S310>/Unit Delay' incorporates:
   *  UnitDelay: '<S316>/Delay Input2'
   *
   * Block description for '<S316>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ap =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_el;

  /* Update for UnitDelay: '<S283>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_a = rtb_CastToBoolean1;

  /* Update for UnitDelay: '<S311>/Unit Delay' incorporates:
   *  UnitDelay: '<S319>/Delay Input2'
   *
   * Block description for '<S319>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_o =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pd;

  /* Update for UnitDelay: '<S283>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_c = rtb_deltafalllimit_cz;

  /* Update for UnitDelay: '<S312>/Unit Delay' incorporates:
   *  UnitDelay: '<S322>/Delay Input2'
   *
   * Block description for '<S322>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ah =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_mt;

  /* Update for UnitDelay: '<S283>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_d = rtb_deltafalllimit_ap;

  /* Update for UnitDelay: '<S208>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_lh = VehVxEst_mps;

  /* Update for UnitDelay: '<S271>/Unit Delay4' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_m = Acc_POS_n;

  /* Update for UnitDelay: '<S208>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_k = VehVxEst_mps;

  /* Update for UnitDelay: '<S271>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_f = rtb_LogicalOperator2;

  /* Update for UnitDelay: '<S274>/Unit Delay3' incorporates:
   *  UnitDelay: '<S274>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_p =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d;

  /* Update for UnitDelay: '<S274>/Unit Delay1' incorporates:
   *  UnitDelay: '<S333>/Delay Input2'
   *
   * Block description for '<S333>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_d =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hu;

  /* Update for UnitDelay: '<S274>/Unit Delay2' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_f = rtb_CastToDouble;

  /* Update for UnitDelay: '<S331>/Unit Delay' incorporates:
   *  UnitDelay: '<S338>/Delay Input2'
   *
   * Block description for '<S338>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_ncs =
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_f;

  /* End of Outputs for S-Function (fcncallgen): '<S4>/10ms1' */

  /* S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* Logic: '<S109>/NOT' */
  rtb_LogicalOperator2 = !(Trq_CUT != 0.0);

  /* Logic: '<S109>/AND' */
  rtb_Compare = ((KeyPressed != 0.0) && rtb_LogicalOperator2);

  /* RelationalOperator: '<S111>/Compare' incorporates:
   *  Constant: '<S111>/Constant'
   */
  Brk = (Brk_F >= 600);

  /* RelationalOperator: '<S112>/Compare' incorporates:
   *  Constant: '<S112>/Constant'
   */
  ACC_Release = (Acc_POS_n <= 50.0F);

  /* Logic: '<S109>/NOT1' */
  rtb_LogicalOperator2 = !(VehCtrlMdel240926_2018b_amksp_B.AMKSWITCH_bx != 0.0);

  /* Switch: '<S109>/Switch' incorporates:
   *  Constant: '<S109>/Constant1'
   *  Switch: '<S109>/Switch10'
   *  Switch: '<S109>/Switch11'
   *  Switch: '<S109>/Switch3'
   */
  if (rtb_LogicalOperator2) {
    elapseTime = MCFL_bSystemReady;
    WhlSpdFL = MCFR_bSystemReady;
    WhlSpdFR = MCFL_bQuitInverterOn;
    VehCtrlMdel240926_2018b_amksp_B.Switch11 = MCFR_bQuitInverterOn;
  } else {
    elapseTime = 1.0;
    WhlSpdFL = 1.0;
    WhlSpdFR = 1.0;
    VehCtrlMdel240926_2018b_amksp_B.Switch11 = 1.0;
  }

  /* End of Switch: '<S109>/Switch' */

  /* Chart: '<S109>/Chart2' */
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
      VehCtrlMdel240926_2018b_VehStat(&controller_ready, &rtb_Compare,
        &elapseTime, &WhlSpdFL, &WhlSpdFR);
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
      VehCtrlMdel240926_20_AMKDCready(&MCFL_bDCOn, &MCFR_bDCOn, &rtb_Compare,
        &elapseTime, &WhlSpdFL, &WhlSpdFR);
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

  /* End of Chart: '<S109>/Chart2' */

  /* Switch: '<S109>/Switch4' */
  VehCtrlMdel240926_2018b_amksp_B.MCFL_DCOn_setpoints_o = (rtb_LogicalOperator2 &&
    MCFL_DCOn_setpoints);

  /* End of Outputs for S-Function (fcncallgen): '<S2>/10ms' */

  /* S-Function (fcncallgen): '<S5>/10ms1' incorporates:
   *  SubSystem: '<S5>/Beeper'
   */
  /* Logic: '<S344>/Logical Operator2' */
  rtb_ignition_e = !rtb_ignition_e;

  /* S-Function (ec5744_pdsslb2u3): '<S344>/PowerDriverSwitch(LS)' */
  L9826VAR701[3]= beeper_state;
  ec_l9826tr701_control(L9826VAR701);

  /* Chart: '<S344>/Timer2' incorporates:
   *  Constant: '<S344>/Constant3'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition_e, 1.0F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2_h);

  /* RelationalOperator: '<S362>/Compare' incorporates:
   *  Constant: '<S362>/Constant'
   */
  rtb_ignition_e = (VehCtrlMdel240926_2018b_amksp_B.Exit > 0.0);

  /* RelationalOperator: '<S353>/FixPt Relational Operator' incorporates:
   *  UnitDelay: '<S353>/Delay Input1'
   *
   * Block description for '<S353>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE = ((int32_T)rtb_ignition_e >
    (int32_T)VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE);

  /* RelationalOperator: '<S350>/Compare' incorporates:
   *  Constant: '<S350>/Constant'
   */
  rtb_LogicalOperator2 = (voltage > 300.0);

  /* Chart: '<S344>/Timer1' incorporates:
   *  Constant: '<S344>/Constant1'
   */
  VehCtrlMdel240926_20_Timer1(rtb_LogicalOperator2, 2.0F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_o,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1_h);

  /* RelationalOperator: '<S361>/Compare' incorporates:
   *  Constant: '<S361>/Constant'
   */
  rtb_LogicalOperator2 = (VehCtrlMdel240926_2018b_amksp_B.Exit_o <= 0.0);

  /* UnitDelay: '<S352>/Delay Input1'
   *
   * Block description for '<S352>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_LogicalOperator7_m = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_n;

  /* RelationalOperator: '<S352>/FixPt Relational Operator' */
  rtb_LogicalOperator7_m = ((int32_T)rtb_LogicalOperator2 > (int32_T)
    rtb_LogicalOperator7_m);

  /* Logic: '<S344>/Logical Operator1' */
  rtb_Compare_am = !rtb_LogicalOperator7_m;

  /* RelationalOperator: '<S351>/Compare' incorporates:
   *  Constant: '<S351>/Constant'
   */
  rtb_LowerRelop1_b = (voltage < 300.0);

  /* Switch: '<S358>/Switch6' incorporates:
   *  Constant: '<S358>/Reset'
   *  UnitDelay: '<S344>/Unit Delay'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l3) {
    /* Sum: '<S358>/Add10' incorporates:
     *  Constant: '<S358>/Steptime'
     */
    rtb_CastToDouble = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_c0 +
      0.01F;
  } else {
    rtb_CastToDouble = 0.0F;
  }

  /* End of Switch: '<S358>/Switch6' */

  /* MinMax: '<S358>/Min' incorporates:
   *  Constant: '<S344>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_c0 = fminf(rtb_CastToDouble,
    10.0F);

  /* RelationalOperator: '<S358>/Relational Operator9' incorporates:
   *  Constant: '<S344>/ResetDelay'
   *  UnitDelay: '<S358>/Unit Delay1'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_c0 >= 10.0F);

  /* Logic: '<S344>/AND1' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b && rtb_Compare);

  /* Logic: '<S344>/Logical Operator3' incorporates:
   *  UnitDelay: '<S344>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l3 =
    !VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l3;

  /* Logic: '<S344>/AND' incorporates:
   *  UnitDelay: '<S344>/Unit Delay'
   */
  rtb_Compare_am = (rtb_Compare_am && rtb_LowerRelop1_b &&
                    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l3);

  /* Logic: '<S344>/OR' incorporates:
   *  UnitDelay: '<S353>/Delay Input1'
   *
   * Block description for '<S353>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE ||
                 rtb_LogicalOperator7_m || rtb_Compare_am);

  /* Chart: '<S344>/Chart' */
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
      rtb_Compare = ((VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_p >=
                      20U) && rtb_Compare);
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEOFF = 1U;
        HVSWITCH = false;
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
      VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_p = 0U;
      HVSWITCH = true;
    } else {
      /* case IN_initstate: */
      HVSWITCH = false;
      rtb_Compare = ((VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1_p >=
                      20U) && rtb_Compare);
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_STATEON = 1U;
        HVSWITCH = true;
      }
    }
  }

  /* End of Chart: '<S344>/Chart' */

  /* S-Function (ec5744_pdsslbu3): '<S344>/PowerDriverSwitch(HS)2' */

  /* Set level HVSWITCH for the specified power driver switch */
  ec_gpio_write(57,HVSWITCH);

  /* Logic: '<S344>/OR2' */
  rtb_LogicalOperator7_m = ((MCFL_bError != 0.0) || (MCFR_bError != 0.0));

  /* Outputs for Enabled SubSystem: '<S344>/Enabled Subsystem1' incorporates:
   *  EnablePort: '<S354>/Enable'
   */
  if (rtb_LogicalOperator7_m) {
    if (!VehCtrlMdel240926_2018b_amks_DW.EnabledSubsystem1_MODE) {
      /* Enable for Chart: '<S354>/Chart' */
      VehCtrlMdel240926_2018b_amks_DW.previousTicks =
        VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;
      VehCtrlMdel240926_2018b_amks_DW.EnabledSubsystem1_MODE = true;
    }

    /* Chart: '<S354>/Chart' */
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
        rtb_Compare = ((MCFL_bError != 0.0) && (!(MCFR_bError != 0.0)));
        if (rtb_Compare) {
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel240926_2018b_
            = 2U;
          VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateA = 2U;
          VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
          VehCtrlMdel240926_2018b_amksp_B.LEDOn = 0.0;
        } else {
          rtb_Compare = ((!(MCFL_bError != 0.0)) && (MCFR_bError != 0.0));
          if (rtb_Compare) {
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_c27_VehCtrlMdel240926_2018b_
              = 3U;
            VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_StateB = 2U;
            VehCtrlMdel240926_2018b_amks_DW.temporalCounter_i1 = 0U;
            VehCtrlMdel240926_2018b_amksp_B.LEDOn = 0.0;
          } else {
            rtb_Compare = ((MCFL_bError != 0.0) && (MCFR_bError != 0.0));
            if (rtb_Compare) {
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
        rtb_Compare = ((!(MCFL_bError != 0.0)) || (MCFR_bError != 0.0));
        if (rtb_Compare) {
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
        rtb_Compare = ((MCFL_bError != 0.0) || (!(MCFR_bError != 0.0)));
        if (rtb_Compare) {
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
        rtb_Compare = ((!(MCFL_bError != 0.0)) || (!(MCFR_bError != 0.0)));
        if (rtb_Compare) {
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

    /* End of Chart: '<S354>/Chart' */
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.EnabledSubsystem1_MODE) {
      /* Disable for Chart: '<S354>/Chart' */
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

      /* End of Disable for Chart: '<S354>/Chart' */
      VehCtrlMdel240926_2018b_amks_DW.EnabledSubsystem1_MODE = false;
    }
  }

  /* End of Outputs for SubSystem: '<S344>/Enabled Subsystem1' */

  /* Logic: '<S344>/Logical Operator5' */
  rtb_Compare = !rtb_LogicalOperator7_m;

  /* Switch: '<S344>/Switch1' */
  STATEDISPLAY = (rtb_Compare || (VehCtrlMdel240926_2018b_amksp_B.LEDOn != 0.0));

  /* S-Function (ec5744_pdsslb2u3): '<S344>/PowerDriverSwitch(LS)1' */
  L9826VAR701[2]= STATEDISPLAY;
  ec_l9826tr701_control(L9826VAR701);

  /* Update for UnitDelay: '<S353>/Delay Input1'
   *
   * Block description for '<S353>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE = rtb_ignition_e;

  /* Update for UnitDelay: '<S352>/Delay Input1'
   *
   * Block description for '<S352>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_n = rtb_LogicalOperator2;

  /* Update for UnitDelay: '<S344>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_l3 = HVSWITCH;

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms1' */

  /* S-Function (fcncallgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/PwrTrainTempPrtct'
   */
  /* MinMax: '<S8>/Max1' */
  elapseTime = fmax(MCU_Temp, motor_Temp);

  /* Lookup_n-D: '<S8>/2-D Lookup Table2' */
  elapseTime = look1_binlx(elapseTime,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable2_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.uDLookupTable2_tableData, 6U);

  /* DataTypeConversion: '<S8>/Cast To Single' */
  rtb_CastToDouble = (real32_T)elapseTime;

  /* Gain: '<S8>/Gain' */
  rtb_CastToDouble *= 10.0F;

  /* MinMax: '<S8>/Max2' */
  elapseTime = fmax(MCFL_TempMotor, MCFR_TempMotor);

  /* RelationalOperator: '<S104>/Compare' incorporates:
   *  Constant: '<S104>/Constant'
   */
  rtb_LogicalOperator2 = (elapseTime > 45.0);

  /* MinMax: '<S8>/Max' */
  WhlSpdFL = fmax(MCFL_TempIGBT, MCFR_TempIGBT);
  WhlSpdFL = fmax(WhlSpdFL, MCFL_TempInverter);
  WhlSpdFL = fmax(WhlSpdFL, MCFR_TempInverter);

  /* RelationalOperator: '<S100>/Compare' incorporates:
   *  Constant: '<S100>/Constant'
   */
  rtb_ignition_e = (WhlSpdFL > 45.0);

  /* Logic: '<S8>/AND2' */
  rtb_ignition_e = (rtb_LogicalOperator2 && rtb_ignition_e);

  /* Chart: '<S8>/Timer1' incorporates:
   *  Constant: '<S8>/Constant2'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition_e, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_g,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer1);

  /* RelationalOperator: '<S102>/Compare' incorporates:
   *  Constant: '<S102>/Constant'
   */
  rtb_ignition_e = (MCU_Temp > 80.0);

  /* Chart: '<S8>/Timer2' incorporates:
   *  Constant: '<S8>/Constant3'
   */
  VehCtrlMdel240926_20_Timer1(rtb_ignition_e, 0.11F,
    &VehCtrlMdel240926_2018b_amksp_B.Exit_d,
    &VehCtrlMdel240926_2018b_amks_DW.sf_Timer2);

  /* SignalConversion generated from: '<S8>/Out1' */
  EMRAX_Trq_CUT = VehCtrlMdel240926_2018b_amksp_B.Exit_d;

  /* SignalConversion generated from: '<S8>/Out1' */
  AMK_Trq_CUT = VehCtrlMdel240926_2018b_amksp_B.Exit_g;

  /* RelationalOperator: '<S103>/Compare' incorporates:
   *  Constant: '<S103>/Constant'
   */
  rtb_ignition_e = (WhlSpdFL > 30.0);

  /* SignalConversion generated from: '<S8>/Out1' */
  VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON = rtb_ignition_e;

  /* RelationalOperator: '<S106>/Compare' incorporates:
   *  Constant: '<S106>/Constant'
   */
  rtb_ignition_e = (elapseTime > 50.0);

  /* SignalConversion generated from: '<S8>/Out1' */
  VehCtrlMdel240926_2018b_amksp_B.bWaterPumpON = rtb_ignition_e;

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

  /* DataTypeConversion: '<S28>/Cast To Single1' */
  rtb_Add6 = (real32_T)MCFL_ActualVelocity;
  rtb_VxIMU_est = (real32_T)MCFR_ActualVelocity;
  rtb_Ax = (real32_T)RPM;

  /* DataTypeConversion: '<S28>/Cast To Single' */
  rtb_Add10_b = (real32_T)MCFL_ActualTorque;
  rtb_Switch2_mn = (real32_T)MCFR_ActualTorque;
  rtb_Add7 = (real32_T)trq;

  /* Product: '<S28>/Product4' */
  rtb_Switch2_b0 = rtb_Add10_b * rtb_Add6;

  /* Product: '<S28>/Product' */
  rtb_Add4_j = rtb_Add6 * rtb_Add10_b;

  /* Product: '<S28>/Product4' */
  rtb_Add6 = rtb_VxIMU_est;
  rtb_Add10_b = rtb_Switch2_mn * rtb_Add6;

  /* Product: '<S28>/Product' */
  rtb_Add4_j += rtb_Add6 * rtb_Switch2_mn;

  /* Product: '<S28>/Product4' */
  rtb_Add6 = rtb_Ax;
  rtb_Switch2_mn = rtb_Add7 * rtb_Add6;

  /* Product: '<S28>/Product' */
  rtb_Add4_j += rtb_Add6 * rtb_Add7;

  /* Gain: '<S28>/Gain' */
  PwrALL = 0.000104712039F * rtb_Add4_j;

  /* Sum: '<S28>/Add' incorporates:
   *  Constant: '<S28>/Constant'
   */
  rtb_Add4_j = 78.0F - PwrALL;

  /* RelationalOperator: '<S28>/Relational Operator' incorporates:
   *  Constant: '<S28>/Constant1'
   */
  rtb_LogicalOperator7_m = (rtb_Add4_j < 0.0F);

  /* Switch: '<S28>/Switch' incorporates:
   *  Constant: '<S28>/Constant2'
   */
  if (rtb_LogicalOperator7_m) {
    rtb_Add6 = rtb_Add4_j;
  } else {
    rtb_Add6 = 0.0F;
  }

  /* End of Switch: '<S28>/Switch' */

  /* Product: '<S28>/Product1' incorporates:
   *  Constant: '<S28>/Constant3'
   */
  rtb_Add4_j = 30.0F * rtb_Add6;

  /* Sum: '<S28>/Add8' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m = rtb_Add6 -
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m;

  /* Product: '<S28>/Product3' incorporates:
   *  Constant: '<S28>/Constant8'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m *= 10.0F;

  /* Sum: '<S28>/Add7' */
  rtb_Add4_j += VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m;

  /* MATLAB Function: '<S28>/MATLAB Function' */
  rtb_Ax = rtb_Switch2_b0 / ((rtb_Switch2_b0 + rtb_Add10_b) + rtb_Switch2_mn);
  rtb_Add7 = rtb_Add10_b / ((rtb_Switch2_b0 + rtb_Add10_b) + rtb_Switch2_mn);
  rtb_Switch2_mn /= (rtb_Switch2_b0 + rtb_Add10_b) + rtb_Switch2_mn;

  /* Sum: '<S28>/Add6' incorporates:
   *  Constant: '<S28>/Constant12'
   */
  WhlSpdFL = MCFL_ActualVelocity + 2.2204460492503131E-16;

  /* Product: '<S28>/Divide' incorporates:
   *  Constant: '<S28>/Constant6'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m = (real32_T)(rtb_Ax *
    9550.0F / WhlSpdFL);

  /* Sum: '<S28>/Add5' incorporates:
   *  Constant: '<S28>/Constant11'
   */
  WhlSpdFL = MCFR_ActualVelocity + 2.2204460492503131E-16;

  /* Product: '<S28>/Divide1' incorporates:
   *  Constant: '<S28>/Constant4'
   */
  rtb_Add7 = (real32_T)(rtb_Add7 * 9550.0F / WhlSpdFL);

  /* Sum: '<S28>/Add4' incorporates:
   *  Constant: '<S28>/Constant7'
   */
  WhlSpdFL = RPM + 2.2204460492503131E-16;

  /* Product: '<S28>/Divide2' incorporates:
   *  Constant: '<S28>/Constant5'
   */
  rtb_Switch2_b0 = (real32_T)(rtb_Switch2_mn * 9550.0F / WhlSpdFL);

  /* Product: '<S28>/Product2' */
  rtb_Add10_b = rtb_Add4_j * VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m;
  rtb_Switch2_mn = rtb_Add4_j * rtb_Add7;
  rtb_Add7 = rtb_Add4_j * rtb_Switch2_b0;

  /* Switch: '<S29>/Switch' */
  if (VehCtrlMdel240926_2018b_amksp_B.AMKSWITCH_bx != 0.0) {
    /* Lookup_n-D: '<S29>/4WD_Table' */
    Acc_POS_n = look2_iflf_binlx(Acc_POS_n, VehVxEst_mps,
      VehCtrlMdel240926_2018b__ConstP.pooled31,
      VehCtrlMdel240926_2018b__ConstP.pooled32,
      VehCtrlMdel240926_2018b__ConstP.pooled30,
      VehCtrlMdel240926_2018b__ConstP.pooled79, 11U);
  } else {
    /* Lookup_n-D: '<S29>/RWD_Table' */
    Acc_POS_n = look2_iflf_binlx(Acc_POS_n, VehVxEst_mps,
      VehCtrlMdel240926_2018b__ConstP.pooled31,
      VehCtrlMdel240926_2018b__ConstP.pooled32,
      VehCtrlMdel240926_2018b__ConstP.pooled30,
      VehCtrlMdel240926_2018b__ConstP.pooled79, 11U);
  }

  /* End of Switch: '<S29>/Switch' */

  /* Gain: '<S28>/Gain2' */
  rtb_Switch2_b0 = 0.8F * Acc_POS_n;

  /* Sum: '<S28>/Add3' */
  rtb_Add7 += rtb_Switch2_b0;

  /* Saturate: '<S28>/Saturation2' */
  if (rtb_Add7 <= 0.0F) {
    rtb_Add7 = 0.0F;
  }

  /* End of Saturate: '<S28>/Saturation2' */

  /* MinMax: '<S28>/Max3' */
  rtb_Add7 = fminf(rtb_Add7, rtb_Switch2_b0);

  /* UnitDelay: '<S7>/Unit Delay' */
  WhlSpdFL = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_n;

  /* SampleTimeMath: '<S43>/sample time'
   *
   * About '<S43>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S43>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  rtb_g_mpss1 = 2000.0 * elapseTime;

  /* Lookup_n-D: '<S10>/228' */
  WhlSpdFR = look1_binlx(RPM, VehCtrlMdel240926_2018b__ConstP.u28_bp01Data,
    VehCtrlMdel240926_2018b__ConstP.u28_tableData, 26U);

  /* Lookup_n-D: '<S10>/AMK' */
  WhlSpdRR_mps = look1_binlx(MCFL_ActualVelocity,
    VehCtrlMdel240926_2018b__ConstP.pooled6,
    VehCtrlMdel240926_2018b__ConstP.pooled5, 19U);

  /* Lookup_n-D: '<S10>/AMK1' */
  WhlSpdRL_mps = look1_binlx(MCFR_ActualVelocity,
    VehCtrlMdel240926_2018b__ConstP.pooled6,
    VehCtrlMdel240926_2018b__ConstP.pooled5, 19U);

  /* MATLAB Function: '<S10>/ÔØºÉ×ªÒÆ' incorporates:
   *  Constant: '<S10>/Constant10'
   *  Constant: '<S10>/Constant2'
   *  Constant: '<S10>/Constant3'
   *  Constant: '<S10>/Constant4'
   *  Constant: '<S10>/Constant5'
   *  Constant: '<S10>/Constant9'
   */
  rtb_Switch2_b0 = (1666.0F - 340.0F * (real32_T)rtb_UkYk1_nc * 0.29F / 1.2F) *
    0.521984875F - 170.0F * (real32_T)rtb_Yk1_l * 0.29F / 1.592F;
  rtb_Ax = (340.0F * (real32_T)rtb_UkYk1_nc * 0.29F / 1.2F + 1666.0F) *
    0.521984875F - 170.0F * (real32_T)rtb_Yk1_l * 0.29F / 1.592F;
  rtb_Fz3 = (1666.0F - 340.0F * (real32_T)rtb_UkYk1_nc * 0.29F / 1.2F) *
    0.521984875F + 170.0F * (real32_T)rtb_Yk1_l * 0.29F / 1.592F;
  rtb_VxIMU_est = (340.0F * (real32_T)rtb_UkYk1_nc * 0.29F / 1.2F + 1666.0F) *
    0.521984875F + 170.0F * (real32_T)rtb_Yk1_l * 0.29F / 1.592F;

  /* Gain: '<S10>/Gain3' */
  rtb_Switch2_on = 0.1020408163265306 * rtb_UkYk1_nc;

  /* MATLAB Function: '<S10>/MATLAB Function' incorporates:
   *  Constant: '<S10>/Constant11'
   *  Constant: '<S10>/Constant12'
   *  Constant: '<S10>/Constant13'
   *  Constant: '<S10>/Constant26'
   */
  rtb_Add4_j = rtb_Switch2_b0 * 0.75F;
  rtb_Switch2_b0 = rtb_Switch2_b0 * (real32_T)rtb_Switch2_on / 9.8F;
  rtb_MaxWhlSpd_mps_n = rtb_Ax * 0.75F;
  rtb_Gain26 = rtb_Ax * (real32_T)rtb_Switch2_on / 9.8F;
  rtb_Ax = rtb_Fz3 * 0.75F;
  rtb_Fz3 = rtb_Fz3 * (real32_T)rtb_Switch2_on / 9.8F;
  rtb_Add12_j = rtb_VxIMU_est * 0.75F;
  rtb_VxIMU_est = rtb_VxIMU_est * (real32_T)rtb_Switch2_on / 9.8F;
  rtb_MaxWhlSpd_mps_n = fminf(sqrtf(rtb_MaxWhlSpd_mps_n * rtb_MaxWhlSpd_mps_n -
    rtb_Gain26 * rtb_Gain26) * 0.2F / 11.4F, (real32_T)WhlSpdRL_mps);
  rtb_Add4_j = fminf(sqrtf(rtb_Add4_j * rtb_Add4_j - rtb_Switch2_b0 *
    rtb_Switch2_b0) * 0.2F / 11.4F, (real32_T)WhlSpdRR_mps);
  rtb_Switch2_b0 = fminf(fminf(sqrtf(rtb_Add12_j * rtb_Add12_j - rtb_VxIMU_est *
    rtb_VxIMU_est), sqrtf(rtb_Ax * rtb_Ax - rtb_Fz3 * rtb_Fz3)) * 0.2F / 3.4F,
    (real32_T)WhlSpdFR);

  /* Gain: '<S10>/Gain2' */
  rtb_Switch2_b0 *= 0.95F;

  /* Gain: '<S10>/Gain' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m = 0.95F *
    rtb_MaxWhlSpd_mps_n;

  /* Gain: '<S10>/Gain1' */
  rtb_Add4_j *= 0.95F;

  /* MinMax: '<S10>/Min1' */
  rtb_Ax = fminf(VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m, rtb_Add4_j);

  /* Product: '<S10>/Divide3' */
  rtb_MaxWhlSpd_mps_n = rtb_Ax / rtb_Switch2_b0;

  /* UnitDelay: '<S44>/Delay Input2'
   *
   * Block description for '<S44>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add12_j = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l4;

  /* SampleTimeMath: '<S44>/sample time'
   *
   * About '<S44>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime_0 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S44>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant48'
   */
  rtb_VxIMU_est = (real32_T)(4.0 * elapseTime_0);

  /* Abs: '<S10>/Abs5' */
  rtb_Fz3 = fabsf(rtb_CastToBoolean);

  /* Lookup_n-D: '<S10>/2-D Lookup Table1' */
  rtb_Fz3 = look2_iflf_binlx(rtb_Fz3, VehVxEst_mps,
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
  rtb_Fz3 -= rtb_Add12_j;

  /* RelationalOperator: '<S61>/LowerRelop1' */
  rtb_LogicalOperator7_m = (rtb_Fz3 > rtb_VxIMU_est);

  /* Switch: '<S61>/Switch2' */
  if (!rtb_LogicalOperator7_m) {
    /* Product: '<S44>/delta fall limit' */
    rtb_VxIMU_est = (real32_T)(-4.0 * elapseTime_0);

    /* RelationalOperator: '<S61>/UpperRelop' */
    rtb_ignition_e = (rtb_Fz3 < rtb_VxIMU_est);

    /* Switch: '<S61>/Switch' */
    if (rtb_ignition_e) {
      rtb_Fz3 = rtb_VxIMU_est;
    }

    /* End of Switch: '<S61>/Switch' */
    rtb_VxIMU_est = rtb_Fz3;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l4 = rtb_VxIMU_est +
    rtb_Add12_j;

  /* Sum: '<S10>/Add18' incorporates:
   *  UnitDelay: '<S44>/Delay Input2'
   *
   * Block description for '<S44>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_MaxWhlSpd_mps_n += VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_l4;

  /* Saturate: '<S10>/Saturation3' */
  if (rtb_MaxWhlSpd_mps_n > 0.7F) {
    rtb_MaxWhlSpd_mps_n = 0.7F;
  } else {
    if (rtb_MaxWhlSpd_mps_n < 0.1F) {
      rtb_MaxWhlSpd_mps_n = 0.1F;
    }
  }

  /* End of Saturate: '<S10>/Saturation3' */

  /* Sum: '<S10>/Add17' incorporates:
   *  Constant: '<S10>/Constant47'
   */
  rtb_Switch_jz = 1.0 - rtb_MaxWhlSpd_mps_n;

  /* Product: '<S10>/Product1' */
  rtb_Gain4 = Acc_POS_n * rtb_Switch_jz;

  /* RelationalOperator: '<S50>/LowerRelop1' */
  rtb_LogicalOperator7_m = (rtb_Gain4 > rtb_Switch2_b0);

  /* Switch: '<S50>/Switch2' */
  if (rtb_LogicalOperator7_m) {
    rtb_Switch_jz = rtb_Switch2_b0;
  } else {
    /* RelationalOperator: '<S50>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    rtb_ignition_e = (rtb_Gain4 < 0.0);

    /* Switch: '<S50>/Switch' incorporates:
     *  Constant: '<S10>/Constant8'
     */
    if (rtb_ignition_e) {
      rtb_Switch_jz = 0.0;
    } else {
      rtb_Switch_jz = rtb_Gain4;
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
  rtb_Gain20 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_nk;

  /* SampleTimeMath: '<S46>/sample time'
   *
   * About '<S46>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime_0 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S46>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant45'
   */
  rtb_Switch2_hly = 1000.0 * elapseTime_0;

  /* UnitDelay: '<S10>/Unit Delay3' */
  rtb_LogicalOperator7_m = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i;

  /* UnitDelay: '<S45>/Delay Input2'
   *
   * Block description for '<S45>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Fz3 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j3;

  /* SampleTimeMath: '<S45>/sample time'
   *
   * About '<S45>/sample time':
   *  y = K where K = ( w * Ts )
   */
  WhlSpdFR = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S45>/delta rise limit' */
  rtb_VxIMU_est = (real32_T)(4000.0 * WhlSpdFR);

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
  rtb_Add12_j = VehVxEst_mps * rtb_CastToBoolean / (340.0F * VehVxEst_mps *
    VehVxEst_mps * 15.5799866F / 460.0F / 440.0F / 1.592F / 2.0F + 1.592F);
  if (rtb_Add12_j < 0.0F) {
    rtb_Gain26 = -1.0F;
  } else if (rtb_Add12_j > 0.0F) {
    rtb_Gain26 = 1.0F;
  } else if (rtb_Add12_j == 0.0F) {
    rtb_Gain26 = 0.0F;
  } else {
    rtb_Gain26 = (rtNaNF);
  }

  rtb_Add12_j = fminf(fabsf(5.88F / (real32_T)WhlSpdRR_mps), fabsf(rtb_Add12_j))
    * 0.8F * rtb_Gain26;

  /* End of MATLAB Function: '<S10>/Wtarget' */

  /* Sum: '<S45>/Difference Inputs1'
   *
   * Block description for '<S45>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add12_j -= rtb_Fz3;

  /* RelationalOperator: '<S62>/LowerRelop1' */
  rtb_LowerRelop1_b = (rtb_Add12_j > rtb_VxIMU_est);

  /* Switch: '<S62>/Switch2' */
  if (!rtb_LowerRelop1_b) {
    /* Product: '<S45>/delta fall limit' */
    rtb_VxIMU_est = (real32_T)(-4000.0 * WhlSpdFR);

    /* RelationalOperator: '<S62>/UpperRelop' */
    rtb_ignition_e = (rtb_Add12_j < rtb_VxIMU_est);

    /* Switch: '<S62>/Switch' */
    if (rtb_ignition_e) {
      rtb_Add12_j = rtb_VxIMU_est;
    }

    /* End of Switch: '<S62>/Switch' */
    rtb_VxIMU_est = rtb_Add12_j;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j3 = rtb_VxIMU_est +
    rtb_Fz3;

  /* Sum: '<S10>/Add' incorporates:
   *  UnitDelay: '<S45>/Delay Input2'
   *
   * Block description for '<S45>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add5 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_j3 - rtb_UkYk1;

  /* Abs: '<S10>/Abs' */
  rtb_Gain5 = fabs(rtb_Add5);

  /* RelationalOperator: '<S34>/Compare' incorporates:
   *  Constant: '<S34>/Constant'
   */
  rtb_LowerRelop1_b = (rtb_Gain5 > 4.0);

  /* Abs: '<S10>/Abs1' */
  rtb_Gain5 = fabs(rtb_UkYk1);

  /* RelationalOperator: '<S35>/Compare' incorporates:
   *  Constant: '<S35>/Constant'
   */
  rtb_Compare_am = (rtb_Gain5 > 1.0);

  /* RelationalOperator: '<S36>/Compare' incorporates:
   *  Constant: '<S36>/Constant'
   */
  rtb_Compare = (VehVxEst_mps > 2.0F);

  /* Logic: '<S10>/AND' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b && rtb_Compare_am && rtb_Compare);

  /* Logic: '<S10>/Logical Operator4' */
  rtb_LogicalOperator7_m = ((!rtb_LogicalOperator7_m) && (!rtb_LowerRelop1_b));

  /* Abs: '<S10>/Abs2' */
  rtb_Gain5 = fabs(rtb_Add5);

  /* RelationalOperator: '<S37>/Compare' incorporates:
   *  Constant: '<S37>/Constant'
   */
  rtb_Compare = (rtb_Gain5 < 3.0);

  /* RelationalOperator: '<S38>/Compare' incorporates:
   *  Constant: '<S38>/Constant'
   */
  rtb_Compare_am = (rtb_Yk1_l < -5.0);

  /* Logic: '<S10>/OR' */
  rtb_Compare = (rtb_Compare || rtb_Compare_am);

  /* Switch: '<S10>/Switch6' incorporates:
   *  Constant: '<S10>/Reset'
   */
  if (rtb_Compare) {
    /* Sum: '<S10>/Add10' incorporates:
     *  Constant: '<S10>/Steptime'
     */
    rtb_Fz3 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_j + 0.01F;
  } else {
    rtb_Fz3 = 0.0F;
  }

  /* End of Switch: '<S10>/Switch6' */

  /* MinMax: '<S10>/Min' incorporates:
   *  Constant: '<S10>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_j = fminf(rtb_Fz3, 1.0F);

  /* RelationalOperator: '<S10>/Relational Operator9' incorporates:
   *  Constant: '<S10>/ResetDelay'
   *  UnitDelay: '<S10>/Unit Delay4'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_j >= 1.0F);

  /* Logic: '<S10>/Logical Operator5' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i =
    ((!rtb_LogicalOperator7_m) && (!rtb_Compare));

  /* Gain: '<S10>/Gain4' */
  rtb_MaxWhlSpd_mps_n *= 0.5F;

  /* Product: '<S10>/Product2' */
  rtb_MaxWhlSpd_mps_n *= Acc_POS_n;

  /* Gain: '<S10>/Gain26' */
  rtb_Gain26 = 0.8F * rtb_MaxWhlSpd_mps_n;

  /* Logic: '<S10>/AND3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_Compare = ((WhlSpdFL != 0.0) &&
                 VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i);

  /* Switch: '<S10>/Switch1' incorporates:
   *  Constant: '<S10>/Constant1'
   */
  if (!rtb_Compare) {
    rtb_Add5 = 0.0;
  }

  /* End of Switch: '<S10>/Switch1' */

  /* Product: '<S10>/Product3' */
  rtb_Gain5 = 2.0 * rtb_Add5;

  /* UnitDelay: '<S10>/Unit Delay2' */
  rtb_Fz3 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_j;

  /* Product: '<S10>/Product' */
  rtb_Fz3 *= rtb_CastToBoolean;

  /* RelationalOperator: '<S33>/Compare' incorporates:
   *  Constant: '<S33>/Constant'
   */
  rtb_Compare = (rtb_Fz3 <= 0.0F);

  /* UnitDelay: '<S10>/Unit Delay' */
  rtb_Switch2_on = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_h;

  /* Switch: '<S10>/Switch' incorporates:
   *  Constant: '<S10>/Constant'
   */
  if (rtb_Compare) {
    rtb_Switch2_on = 0.0;
  }

  /* End of Switch: '<S10>/Switch' */

  /* Product: '<S10>/Product4' */
  WhlSpdFR = rtb_Add5;

  /* UnitDelay: '<S10>/Unit Delay1' */
  rtb_Add5 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_c;

  /* Product: '<S10>/Product5' */
  WhlSpdRR_mps = rtb_Add5;

  /* Sum: '<S10>/Add2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_h = (rtb_Switch2_on +
    WhlSpdFR) - WhlSpdRR_mps;

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
  WhlSpdRR_mps = (rtb_Gain5 + VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_h)
    + VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_k;

  /* Saturate: '<S10>/Saturation' */
  if (WhlSpdRR_mps > 5000.0) {
    WhlSpdFR = 5000.0;
  } else if (WhlSpdRR_mps < -5000.0) {
    WhlSpdFR = -5000.0;
  } else {
    WhlSpdFR = WhlSpdRR_mps;
  }

  /* End of Saturate: '<S10>/Saturation' */

  /* Gain: '<S10>/Gain8' */
  rtb_Fz3 = 0.0174532924F * FLWhlStrAng;

  /* Trigonometry: '<S10>/Cos' */
  rtb_Fz3 = cosf(rtb_Fz3);

  /* Gain: '<S10>/Gain11' */
  rtb_Fz3 *= 1.2F;

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
  rtb_Fz3 += rtb_VxIMU_est;

  /* Product: '<S10>/Divide1' */
  rtb_Switch2_on = WhlSpdFR / rtb_Fz3;

  /* Gain: '<S10>/Gain25' */
  rtb_Switch2_on *= 0.2;

  /* RelationalOperator: '<S54>/LowerRelop1' */
  rtb_Compare = (rtb_Switch2_on > rtb_Gain26);

  /* Switch: '<S54>/Switch2' */
  if (rtb_Compare) {
    rtb_Switch2_on = rtb_Gain26;
  } else {
    /* Gain: '<S10>/Gain27' */
    rtb_StrWhlAngV_c = -rtb_Gain26;

    /* RelationalOperator: '<S54>/UpperRelop' */
    rtb_ignition_e = (rtb_Switch2_on < rtb_StrWhlAngV_c);

    /* Switch: '<S54>/Switch' */
    if (rtb_ignition_e) {
      rtb_Switch2_on = rtb_StrWhlAngV_c;
    }

    /* End of Switch: '<S54>/Switch' */
  }

  /* End of Switch: '<S54>/Switch2' */

  /* Sum: '<S10>/Add4' */
  WhlSpdRL_mps = rtb_MaxWhlSpd_mps_n + rtb_Switch2_on;

  /* Sum: '<S10>/Add14' */
  rtb_Switch2_on = rtb_Ax - WhlSpdRL_mps;

  /* RelationalOperator: '<S10>/Relational Operator' incorporates:
   *  Constant: '<S10>/Constant37'
   */
  rtb_LogicalOperator2 = (rtb_Switch2_on < 0.0);

  /* Sum: '<S10>/Add15' */
  rtb_Gain4 = rtb_Switch2_b0 - rtb_Gain4;

  /* RelationalOperator: '<S10>/Relational Operator1' incorporates:
   *  Constant: '<S10>/Constant38'
   */
  rtb_LogicalOperator7_m = (rtb_Gain4 < 0.0);

  /* Logic: '<S10>/AND1' */
  rtb_Compare = (rtb_LogicalOperator2 && rtb_LogicalOperator7_m);

  /* Logic: '<S10>/OR1' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i ||
                 rtb_Compare);

  /* Switch: '<S10>/Switch2' incorporates:
   *  Constant: '<S10>/Constant32'
   */
  if (rtb_Compare) {
    rtb_Switch2_on = 0.0;
  } else {
    /* Logic: '<S10>/NOT1' */
    rtb_ignition_e = !rtb_LogicalOperator7_m;

    /* Switch: '<S10>/Switch5' incorporates:
     *  Constant: '<S10>/Constant33'
     */
    if (!rtb_ignition_e) {
      rtb_Switch2_on = 0.0;
    }

    /* End of Switch: '<S10>/Switch5' */
  }

  /* End of Switch: '<S10>/Switch2' */

  /* Gain: '<S10>/Gain19' */
  rtb_Switch2_on = -rtb_Switch2_on;

  /* Saturate: '<S10>/Saturation1' */
  if (rtb_Switch2_on > 100.0) {
    rtb_Switch2_on = 100.0;
  } else {
    if (rtb_Switch2_on < 0.0) {
      rtb_Switch2_on = 0.0;
    }
  }

  /* End of Saturate: '<S10>/Saturation1' */

  /* Sum: '<S46>/Difference Inputs1'
   *
   * Block description for '<S46>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Switch2_on -= rtb_Gain20;

  /* RelationalOperator: '<S63>/LowerRelop1' */
  rtb_Compare = (rtb_Switch2_on > rtb_Switch2_hly);

  /* Switch: '<S63>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S46>/delta fall limit' */
    rtb_Gain5 = -1000.0 * elapseTime_0;

    /* RelationalOperator: '<S63>/UpperRelop' */
    rtb_ignition_e = (rtb_Switch2_on < rtb_Gain5);

    /* Switch: '<S63>/Switch' */
    if (rtb_ignition_e) {
      rtb_Switch2_on = rtb_Gain5;
    }

    /* End of Switch: '<S63>/Switch' */
    rtb_Switch2_hly = rtb_Switch2_on;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_nk = rtb_Switch2_hly +
    rtb_Gain20;

  /* Sum: '<S10>/Add13' incorporates:
   *  UnitDelay: '<S46>/Delay Input2'
   *
   * Block description for '<S46>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_on = rtb_Switch_jz +
    VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_nk;

  /* RelationalOperator: '<S53>/LowerRelop1' */
  rtb_Compare = (rtb_Switch2_on > rtb_Switch2_b0);

  /* Switch: '<S53>/Switch2' */
  if (rtb_Compare) {
    rtb_Switch2_on = rtb_Switch2_b0;
  } else {
    /* RelationalOperator: '<S53>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    rtb_ignition_e = (rtb_Switch2_on < 0.0);

    /* Switch: '<S53>/Switch' incorporates:
     *  Constant: '<S10>/Constant31'
     */
    if (rtb_ignition_e) {
      rtb_Switch2_on = 0.0;
    }

    /* End of Switch: '<S53>/Switch' */
  }

  /* End of Switch: '<S53>/Switch2' */

  /* Sum: '<S43>/Difference Inputs1' incorporates:
   *  UnitDelay: '<S43>/Delay Input2'
   *
   * Block description for '<S43>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S43>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_on -= VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p;

  /* RelationalOperator: '<S60>/LowerRelop1' */
  rtb_Compare = (rtb_Switch2_on > rtb_g_mpss1);

  /* Switch: '<S60>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S43>/delta fall limit' */
    elapseTime *= -2000.0;

    /* RelationalOperator: '<S60>/UpperRelop' */
    rtb_ignition_e = (rtb_Switch2_on < elapseTime);

    /* Switch: '<S60>/Switch' */
    if (rtb_ignition_e) {
      rtb_Switch2_on = elapseTime;
    }

    /* End of Switch: '<S60>/Switch' */
    rtb_g_mpss1 = rtb_Switch2_on;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p += rtb_g_mpss1;

  /* Switch: '<S7>/Switch1' incorporates:
   *  UnitDelay: '<S43>/Delay Input2'
   *
   * Block description for '<S43>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (WhlSpdFL != 0.0) {
    elapseTime = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_p;
  } else {
    /* Logic: '<S7>/NOT1' */
    rtb_ignition_e = !(VehCtrlMdel240926_2018b_amksp_B.AMKSWITCH_bx != 0.0);

    /* Switch: '<S7>/Switch10' */
    if (rtb_ignition_e) {
      /* Product: '<S7>/Product2' incorporates:
       *  Constant: '<S7>/Constant'
       */
      elapseTime = Acc_POS_n * 0.8;
    } else {
      elapseTime = Acc_POS_n;
    }

    /* End of Switch: '<S7>/Switch10' */
  }

  /* End of Switch: '<S7>/Switch1' */

  /* RelationalOperator: '<S25>/LowerRelop1' */
  rtb_Compare = (elapseTime > rtb_Add7);

  /* Switch: '<S25>/Switch2' */
  if (!rtb_Compare) {
    /* RelationalOperator: '<S25>/UpperRelop' incorporates:
     *  Constant: '<S7>/Constant1'
     */
    rtb_ignition_e = (elapseTime < 0.0);

    /* Switch: '<S25>/Switch' incorporates:
     *  Constant: '<S7>/Constant1'
     */
    if (rtb_ignition_e) {
      rtb_Add7 = 0.0F;
    } else {
      rtb_Add7 = (real32_T)elapseTime;
    }

    /* End of Switch: '<S25>/Switch' */
  }

  /* End of Switch: '<S25>/Switch2' */

  /* UnitDelay: '<S93>/Unit Delay1' */
  rtb_Compare = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e;

  /* Saturate: '<S32>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    rtb_Fz3 = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    rtb_Fz3 = 0.0F;
  } else {
    rtb_Fz3 = VehVxEst_mps;
  }

  /* Lookup_n-D: '<S32>/VehSpd_SlipTarget_mps' */
  rtb_VxIMU_est = look1_iflf_binlc(rtb_Fz3,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled55, 3U);

  /* Sum: '<S32>/Add9' */
  rtb_VxIMU_est += rtb_Fz3;

  /* Sum: '<S7>/Add1' */
  rtb_Add12_j = rtb_deltafalllimit_cz + rtb_deltafalllimit_ap;

  /* Gain: '<S7>/Gain2' */
  rtb_Add12_j *= 0.5F;

  /* Saturate: '<S32>/Saturation1' */
  if (rtb_Add12_j < 0.0F) {
    rtb_Add12_j = 0.0F;
  }

  /* End of Saturate: '<S32>/Saturation1' */

  /* Sum: '<S32>/Add1' */
  rtb_StrWhlAngV_c = rtb_VxIMU_est - rtb_Add12_j;

  /* RelationalOperator: '<S32>/Relational Operator7' incorporates:
   *  Constant: '<S32>/Cal_DeltaV_mps'
   */
  rtb_Compare_am = (rtb_StrWhlAngV_c < 0.0F);

  /* Logic: '<S93>/Logical Operator4' */
  rtb_Compare = ((!rtb_Compare) && (!rtb_Compare_am));

  /* Logic: '<S32>/Logical Operator2' */
  rtb_Compare_am = !rtb_Compare_am;

  /* UnitDelay: '<S32>/Unit Delay4' */
  rtb_VxIMU_est = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b;

  /* RelationalOperator: '<S32>/Relational Operator8' incorporates:
   *  Constant: '<S32>/Cal_DeltaV_mps1'
   */
  rtb_LowerRelop1_b = (rtb_VxIMU_est > 235.0F);

  /* Logic: '<S32>/Logical Operator1' */
  rtb_Compare_am = (rtb_Compare_am && rtb_LowerRelop1_b);

  /* Switch: '<S94>/Switch6' incorporates:
   *  Constant: '<S94>/Reset'
   */
  if (rtb_Compare_am) {
    /* Sum: '<S94>/Add10' incorporates:
     *  Constant: '<S94>/Steptime'
     */
    rtb_VxIMU_est = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i + 0.01F;
  } else {
    rtb_VxIMU_est = 0.0F;
  }

  /* End of Switch: '<S94>/Switch6' */

  /* MinMax: '<S94>/Min' incorporates:
   *  Constant: '<S32>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i = fminf(rtb_VxIMU_est,
    0.1F);

  /* RelationalOperator: '<S94>/Relational Operator9' incorporates:
   *  Constant: '<S32>/ResetDelay'
   *  UnitDelay: '<S94>/Unit Delay1'
   */
  rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_i >= 0.1F);

  /* UnitDelay: '<S32>/Unit Delay3' */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_a;

  /* Logic: '<S32>/Logical Operator3' */
  rtb_Compare_am = (rtb_Compare_am || rtb_LowerRelop1_b);

  /* Logic: '<S93>/Logical Operator5' incorporates:
   *  UnitDelay: '<S93>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e = ((!rtb_Compare) &&
    (!rtb_Compare_am));

  /* RelationalOperator: '<S91>/Compare' incorporates:
   *  Constant: '<S91>/Constant'
   */
  rtb_Compare = (rtb_Add12_j > 0.0F);

  /* Abs: '<S32>/Abs' */
  rtb_VxIMU_est = fabsf(rtb_CastToBoolean);

  /* RelationalOperator: '<S89>/Compare' incorporates:
   *  Constant: '<S89>/Constant'
   */
  rtb_Compare_am = (rtb_VxIMU_est <= 20.0F);

  /* Logic: '<S32>/Logical Operator6' */
  rtb_Compare = (rtb_Compare && rtb_Compare_am);

  /* Logic: '<S32>/Logical Operator7' incorporates:
   *  UnitDelay: '<S93>/Unit Delay1'
   */
  rtb_ignition_e = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_e &&
                    rtb_Compare);

  /* UnitDelay: '<S92>/Delay Input1'
   *
   * Block description for '<S92>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_Compare = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_e;

  /* RelationalOperator: '<S92>/FixPt Relational Operator' */
  rtb_Compare = ((int32_T)rtb_ignition_e > (int32_T)rtb_Compare);

  /* Switch: '<S32>/Switch' incorporates:
   *  Constant: '<S32>/Integr_StartPoint'
   */
  if (rtb_Compare) {
    /* Sum: '<S32>/Add4' */
    rtb_VxIMU_est = rtb_Add7 -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g;
  } else {
    rtb_VxIMU_est = 0.0F;
  }

  /* End of Switch: '<S32>/Switch' */

  /* Switch: '<S32>/Switch6' incorporates:
   *  Constant: '<S32>/Verror_Reset'
   */
  if (rtb_ignition_e) {
    rtb_Ax = rtb_StrWhlAngV_c;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S32>/Switch6' */

  /* UnitDelay: '<S32>/Unit Delay5' */
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_l;

  /* Product: '<S32>/Product2' */
  rtb_Switch2_b0 *= rtb_StrWhlAngV_c;

  /* RelationalOperator: '<S88>/Compare' incorporates:
   *  Constant: '<S88>/Constant'
   */
  rtb_Compare = (rtb_Switch2_b0 <= 0.0F);

  /* UnitDelay: '<S32>/Unit Delay' */
  rtb_Switch2_b0 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b;

  /* Switch: '<S32>/Switch3' incorporates:
   *  Constant: '<S32>/Verror_Reset1'
   */
  if (rtb_Compare) {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Switch: '<S32>/Switch3' */

  /* Sum: '<S32>/Add2' */
  rtb_Switch2_b0 += rtb_Ax;

  /* Saturate: '<S32>/Saturation2' */
  if (rtb_Switch2_b0 > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = 400.0F;
  } else if (rtb_Switch2_b0 < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b = rtb_Switch2_b0;
  }

  /* End of Saturate: '<S32>/Saturation2' */

  /* Lookup_n-D: '<S32>/VehicleStableTarget_mps' */
  rtb_Switch2_b0 = look1_iflf_binlc(rtb_Fz3,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S32>/Add5' */
  rtb_Switch2_b0 += rtb_Fz3;

  /* Sum: '<S32>/Add10' */
  rtb_Switch2_b0 = rtb_Add12_j - rtb_Switch2_b0;

  /* RelationalOperator: '<S32>/Relational Operator' incorporates:
   *  Constant: '<S32>/Verror'
   */
  rtb_Compare = (rtb_Switch2_b0 < 0.0F);

  /* Logic: '<S32>/Logical Operator4' */
  rtb_Compare = (rtb_Compare && rtb_ignition_e);

  /* Switch: '<S32>/Switch1' incorporates:
   *  Constant: '<S32>/Trq_IReset'
   *  Constant: '<S32>/Trq_I_FF'
   */
  if (rtb_Compare) {
    rtb_Switch2_b0 = 20.0F;
  } else {
    rtb_Switch2_b0 = 0.0F;
  }

  /* End of Switch: '<S32>/Switch1' */

  /* Sum: '<S32>/Add6' incorporates:
   *  UnitDelay: '<S32>/Unit Delay'
   */
  rtb_VxIMU_est = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_b +
                   rtb_VxIMU_est) + rtb_Switch2_b0;

  /* Product: '<S32>/Product1' incorporates:
   *  Constant: '<S32>/I_Gain'
   */
  FLWhlStrAng = rtb_VxIMU_est * 10.0F;

  /* Product: '<S32>/Product' incorporates:
   *  Constant: '<S32>/P_Gain'
   *  UnitDelay: '<S32>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g = rtb_Ax * 40.0F;

  /* Sum: '<S32>/Add11' incorporates:
   *  UnitDelay: '<S32>/Unit Delay1'
   */
  rtb_VxIMU_est = rtb_Add7 - VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g;

  /* RelationalOperator: '<S95>/LowerRelop1' */
  rtb_Compare = (FLWhlStrAng > rtb_VxIMU_est);

  /* Switch: '<S95>/Switch2' */
  if (!rtb_Compare) {
    /* Gain: '<S32>/Gain3' incorporates:
     *  UnitDelay: '<S32>/Unit Delay1'
     */
    rtb_deltafalllimit_cz = -VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g;

    /* RelationalOperator: '<S95>/UpperRelop' */
    rtb_Compare = (FLWhlStrAng < rtb_deltafalllimit_cz);

    /* Switch: '<S95>/Switch' */
    if (rtb_Compare) {
      FLWhlStrAng = rtb_deltafalllimit_cz;
    }

    /* End of Switch: '<S95>/Switch' */
    rtb_VxIMU_est = FLWhlStrAng;
  }

  /* End of Switch: '<S95>/Switch2' */

  /* Sum: '<S32>/Add7' incorporates:
   *  UnitDelay: '<S32>/Unit Delay1'
   *  UnitDelay: '<S32>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g + rtb_VxIMU_est;

  /* Lookup_n-D: '<S32>/VehicleStableTarget_mps1' */
  rtb_VxIMU_est = look1_iflf_binlc(rtb_Fz3,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S32>/Add13' */
  rtb_Fz3 += rtb_VxIMU_est;

  /* Sum: '<S32>/Add12' */
  rtb_Add12_j -= rtb_Fz3;

  /* RelationalOperator: '<S32>/Relational Operator1' incorporates:
   *  Constant: '<S32>/Verror1'
   */
  rtb_Compare = (rtb_Add12_j < 0.0F);

  /* RelationalOperator: '<S32>/Relational Operator2' incorporates:
   *  UnitDelay: '<S32>/Unit Delay4'
   */
  rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b >=
                    rtb_Add7);

  /* RelationalOperator: '<S90>/Compare' incorporates:
   *  Constant: '<S90>/Constant'
   *  UnitDelay: '<S32>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b <=
                       5.0F);

  /* Logic: '<S32>/OR' */
  rtb_Compare_am = (rtb_Compare_am || rtb_LowerRelop1_b);

  /* Logic: '<S32>/Logical Operator5' incorporates:
   *  UnitDelay: '<S32>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_a = (rtb_Compare &&
    rtb_Compare_am);

  /* Switch: '<S32>/Switch2' incorporates:
   *  Switch: '<S32>/Switch7'
   *  UnitDelay: '<S32>/Unit Delay3'
   */
  if (rtb_ignition_e) {
    /* RelationalOperator: '<S96>/LowerRelop1' incorporates:
     *  Constant: '<S32>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S32>/Unit Delay4'
     */
    rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b > 235.0F);

    /* Switch: '<S96>/Switch2' incorporates:
     *  Constant: '<S32>/TCS_TrqRequest_Max2'
     */
    if (rtb_Compare) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 235.0F;
    } else {
      /* RelationalOperator: '<S96>/UpperRelop' incorporates:
       *  Constant: '<S32>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S32>/Unit Delay4'
       */
      rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b < 0.0F);

      /* Switch: '<S96>/Switch' incorporates:
       *  Constant: '<S32>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S32>/Unit Delay4'
       */
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 0.0F;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_b;
      }

      /* End of Switch: '<S96>/Switch' */
    }

    /* End of Switch: '<S96>/Switch2' */

    /* RelationalOperator: '<S97>/LowerRelop1' */
    rtb_Compare = (rtb_Add7 >
                   VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g);

    /* Switch: '<S97>/Switch2' */
    if (!rtb_Compare) {
      /* RelationalOperator: '<S97>/UpperRelop' incorporates:
       *  Constant: '<S32>/TCS_TrqRequest_Min1'
       */
      rtb_Compare = (rtb_Add7 < 0.0F);

      /* Switch: '<S97>/Switch' incorporates:
       *  Constant: '<S32>/TCS_TrqRequest_Min1'
       */
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = 0.0F;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = rtb_Add7;
      }

      /* End of Switch: '<S97>/Switch' */
    }

    /* End of Switch: '<S97>/Switch2' */
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_a) {
      /* Switch: '<S32>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g = rtb_Add7;
    }
  }

  /* End of Switch: '<S32>/Switch2' */

  /* UnitDelay: '<S82>/Unit Delay1' */
  rtb_Compare = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl;

  /* Saturate: '<S31>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    rtb_Fz3 = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    rtb_Fz3 = 0.0F;
  } else {
    rtb_Fz3 = VehVxEst_mps;
  }

  /* End of Saturate: '<S31>/Saturation' */

  /* Lookup_n-D: '<S31>/VehSpd_SlipTarget_mps' */
  rtb_VxIMU_est = look1_iflf_binlc(rtb_Fz3,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled55, 3U);

  /* Sum: '<S31>/Add9' */
  rtb_VxIMU_est += rtb_Fz3;

  /* Saturate: '<S31>/Saturation1' */
  if (rtb_CastToBoolean1 > 50.0F) {
    rtb_Add12_j = 50.0F;
  } else if (rtb_CastToBoolean1 < 0.0F) {
    rtb_Add12_j = 0.0F;
  } else {
    rtb_Add12_j = rtb_CastToBoolean1;
  }

  /* End of Saturate: '<S31>/Saturation1' */

  /* Sum: '<S31>/Add1' */
  FLWhlStrAng = rtb_VxIMU_est - rtb_Add12_j;

  /* RelationalOperator: '<S31>/Relational Operator7' incorporates:
   *  Constant: '<S31>/Cal_DeltaV_mps'
   */
  rtb_Compare_am = (FLWhlStrAng < 0.0F);

  /* Logic: '<S82>/Logical Operator4' */
  rtb_Compare = ((!rtb_Compare) && (!rtb_Compare_am));

  /* Logic: '<S31>/Logical Operator2' */
  rtb_Compare_am = !rtb_Compare_am;

  /* UnitDelay: '<S31>/Unit Delay4' */
  rtb_VxIMU_est = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i;

  /* RelationalOperator: '<S31>/Relational Operator8' incorporates:
   *  Constant: '<S31>/Cal_DeltaV_mps1'
   */
  rtb_LowerRelop1_b = (rtb_VxIMU_est > 235.0F);

  /* Logic: '<S31>/Logical Operator1' */
  rtb_Compare_am = (rtb_Compare_am && rtb_LowerRelop1_b);

  /* Switch: '<S83>/Switch6' incorporates:
   *  Constant: '<S83>/Reset'
   */
  if (rtb_Compare_am) {
    /* Sum: '<S83>/Add10' incorporates:
     *  Constant: '<S83>/Steptime'
     */
    rtb_VxIMU_est = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 + 0.01F;
  } else {
    rtb_VxIMU_est = 0.0F;
  }

  /* End of Switch: '<S83>/Switch6' */

  /* MinMax: '<S83>/Min' incorporates:
   *  Constant: '<S31>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 = fminf(rtb_VxIMU_est,
    0.1F);

  /* RelationalOperator: '<S83>/Relational Operator9' incorporates:
   *  Constant: '<S31>/ResetDelay'
   *  UnitDelay: '<S83>/Unit Delay1'
   */
  rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_n5 >= 0.1F);

  /* UnitDelay: '<S31>/Unit Delay3' */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e;

  /* Logic: '<S31>/Logical Operator3' */
  rtb_Compare_am = (rtb_Compare_am || rtb_LowerRelop1_b);

  /* Logic: '<S82>/Logical Operator5' incorporates:
   *  UnitDelay: '<S82>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl = ((!rtb_Compare) &&
    (!rtb_Compare_am));

  /* UnitDelay: '<S73>/Unit Delay1' */
  rtb_Compare = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp;

  /* Saturate: '<S30>/Saturation' */
  if (VehVxEst_mps > 40.0F) {
    rtb_VxIMU_est = 40.0F;
  } else if (VehVxEst_mps < 0.0F) {
    rtb_VxIMU_est = 0.0F;
  } else {
    rtb_VxIMU_est = VehVxEst_mps;
  }

  /* End of Saturate: '<S30>/Saturation' */

  /* Lookup_n-D: '<S30>/VehSpd_SlipTarget_mps' */
  rtb_Ax = look1_iflf_binlc(rtb_VxIMU_est,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled55, 3U);

  /* Sum: '<S30>/Add9' */
  rtb_Ax += rtb_VxIMU_est;

  /* Saturate: '<S30>/Saturation1' */
  if (rtb_Gain3_o < 0.0F) {
    rtb_Switch2_b0 = 0.0F;
  } else {
    rtb_Switch2_b0 = rtb_Gain3_o;
  }

  /* End of Saturate: '<S30>/Saturation1' */

  /* Sum: '<S30>/Add1' */
  rtb_Gain3_o = rtb_Ax - rtb_Switch2_b0;

  /* RelationalOperator: '<S30>/Relational Operator7' incorporates:
   *  Constant: '<S30>/Cal_DeltaV_mps'
   */
  rtb_Compare_am = (rtb_Gain3_o < 0.0F);

  /* Logic: '<S73>/Logical Operator4' */
  rtb_Compare = ((!rtb_Compare) && (!rtb_Compare_am));

  /* Logic: '<S30>/Logical Operator2' */
  rtb_Compare_am = !rtb_Compare_am;

  /* UnitDelay: '<S30>/Unit Delay4' */
  rtb_Ax = VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l;

  /* RelationalOperator: '<S30>/Relational Operator8' incorporates:
   *  Constant: '<S30>/Cal_DeltaV_mps1'
   */
  rtb_LowerRelop1_b = (rtb_Ax > 235.0F);

  /* Logic: '<S30>/Logical Operator1' */
  rtb_Compare_am = (rtb_Compare_am && rtb_LowerRelop1_b);

  /* Switch: '<S74>/Switch6' incorporates:
   *  Constant: '<S74>/Reset'
   */
  if (rtb_Compare_am) {
    /* Sum: '<S74>/Add10' incorporates:
     *  Constant: '<S74>/Steptime'
     */
    rtb_Ax = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h + 0.01F;
  } else {
    rtb_Ax = 0.0F;
  }

  /* End of Switch: '<S74>/Switch6' */

  /* MinMax: '<S74>/Min' incorporates:
   *  Constant: '<S30>/ResetDelay'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h = fminf(rtb_Ax, 0.1F);

  /* RelationalOperator: '<S74>/Relational Operator9' incorporates:
   *  Constant: '<S30>/ResetDelay'
   *  UnitDelay: '<S74>/Unit Delay1'
   */
  rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_h >= 0.1F);

  /* UnitDelay: '<S30>/Unit Delay3' */
  rtb_LowerRelop1_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_ip;

  /* Logic: '<S30>/Logical Operator3' */
  rtb_Compare_am = (rtb_Compare_am || rtb_LowerRelop1_b);

  /* Logic: '<S73>/Logical Operator5' incorporates:
   *  UnitDelay: '<S73>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp = ((!rtb_Compare) &&
    (!rtb_Compare_am));

  /* Chart: '<S7>/Chart' */
  if (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c7_VehCtrlMdel240926_
      == 0U) {
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_active_c7_VehCtrlMdel240926_ =
      1U;
    VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_B = 3U;
    VehCtrlMdel240926_2018b_amks_DW.b = rtb_Yk1_l * rtb_Yk1_l + rtb_UkYk1_nc *
      rtb_UkYk1_nc;
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
      rtb_Compare = ((rtb_UkYk1 >= 50.0) || (rtb_UkYk1_nc >= 5.0) ||
                     (VehCtrlMdel240926_2018b_amks_DW.b >= 5.0));
      if (rtb_Compare) {
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

    if (fabs(rtb_UkYk1_nc) > 0.5) {
      elapseTime = atan(rtb_Yk1_l / rtb_UkYk1_nc) * 180.0 / 3.1415926535897931;
    } else {
      elapseTime = 0.0;
    }

    VehCtrlMdel240926_2018b_amks_DW.b = rtb_Yk1_l * rtb_Yk1_l + rtb_UkYk1_nc *
      rtb_UkYk1_nc;
    VehCtrlMdel240926_2018b_amks_DW.b = sqrt(VehCtrlMdel240926_2018b_amks_DW.b);
    switch (VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_C) {
     case VehCtrlMdel240926_2018b_am_IN_B:
      rtb_Compare = ((!(VehCtrlMdel240926_2018b_amks_DW.DYC_flag != 0.0)) ||
                     (rtb_UkYk1 > 30.0) || (elapseTime > 30.0) || (elapseTime >
        -30.0) || (VehCtrlMdel240926_2018b_amks_DW.b > 3.0));
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.bitsForTID3.is_C = 3U;
      }
      break;

     case VehCtrlMdel240926_2018b_am_IN_C:
      break;

     default:
      /* case IN_F_TVD_TCS_STATE: */
      rtb_Compare = ((!(VehCtrlMdel240926_2018b_amks_DW.DYC_flag != 0.0)) ||
                     (rtb_UkYk1 > 30.0) || (elapseTime > 30.0) || (elapseTime >
        -30.0) || (VehCtrlMdel240926_2018b_amks_DW.b > 3.0));
      if (rtb_Compare) {
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

  /* Logic: '<S7>/Logical Operator2' */
  rtb_Compare = !VehCtrlMdel240926_2018b_amksp_B.VehReady;

  /* Logic: '<S7>/Logical Operator3' */
  rtb_Compare_am = (rtb_Compare || (Trq_CUT != 0.0));

  /* Logic: '<S7>/Logical Operator1' */
  TrqR_cmd_raw = rtb_Compare_am;

  /* Switch: '<S7>/Switch2' incorporates:
   *  Constant: '<S7>/Constant4'
   *  Switch: '<S7>/Switch5'
   */
  if (TrqR_cmd_raw) {
    rtb_CastToBoolean1 = 0.0F;
  } else {
    if (VehCtrlMdel240926_2018b_amksp_B.TCSR_Enable_OUT != 0.0) {
      /* Switch: '<S7>/Switch5' incorporates:
       *  UnitDelay: '<S32>/Unit Delay2'
       */
      rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_g;
    }

    /* Gain: '<S7>/Gain6' */
    rtb_CastToBoolean1 = 4.76190472F * rtb_Add7;

    /* Lookup_n-D: '<S7>/BrakeCompensateCoefRear' */
    rtb_Add7 = look1_iflf_binlc((real32_T)Brk_F,
      VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefRear_bp01Dat,
      VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefRear_tableDa, 1U);

    /* RelationalOperator: '<S22>/LowerRelop1' */
    rtb_Compare = (rtb_CastToBoolean1 > rtb_Add7);

    /* Switch: '<S22>/Switch2' */
    if (rtb_Compare) {
      rtb_CastToBoolean1 = rtb_Add7;
    } else {
      /* RelationalOperator: '<S22>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      rtb_Compare = (rtb_CastToBoolean1 < 0.0F);

      /* Switch: '<S22>/Switch' incorporates:
       *  Constant: '<S7>/Constant5'
       */
      if (rtb_Compare) {
        rtb_CastToBoolean1 = 0.0F;
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
  rtb_Ax = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd;

  /* Sum: '<S19>/Difference Inputs1'
   *
   * Block description for '<S19>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_CastToBoolean1 -= rtb_Ax;

  /* SampleTimeMath: '<S19>/sample time'
   *
   * About '<S19>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S19>/delta rise limit' */
  rtb_Add7 = (real32_T)(25000.0 * elapseTime);

  /* RelationalOperator: '<S65>/LowerRelop1' */
  rtb_Compare = (rtb_CastToBoolean1 > rtb_Add7);

  /* Switch: '<S65>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S19>/delta fall limit' */
    rtb_deltafalllimit_cz = (real32_T)(-25000.0 * elapseTime);

    /* RelationalOperator: '<S65>/UpperRelop' */
    rtb_Compare = (rtb_CastToBoolean1 < rtb_deltafalllimit_cz);

    /* Switch: '<S65>/Switch' */
    if (rtb_Compare) {
      rtb_CastToBoolean1 = rtb_deltafalllimit_cz;
    }

    /* End of Switch: '<S65>/Switch' */
    rtb_Add7 = rtb_CastToBoolean1;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_cd = rtb_Add7 + rtb_Ax;
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
  Trq_CUT_final = (TroqueOn || (AMK_Trq_CUT != 0.0) || rtb_Compare_am ||
                   TrqR_cmd_raw);

  /* Logic: '<S7>/OR' */
  rtb_Compare = (Trq_CUT_final || (VehCtrlMdel240926_2018b_amksp_B.AMKSWITCH_bx
    != 0.0));

  /* Lookup_n-D: '<S7>/BrakeCompensateCoefFront1' */
  rtb_CastToBoolean1 = look1_iflf_binlc((real32_T)Brk_F,
    VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefFront1_bp01D,
    VehCtrlMdel240926_2018b__ConstP.BrakeCompensateCoefFront1_table, 1U);

  /* Gain: '<S28>/Gain1' */
  rtb_Ax = 0.1F * Acc_POS_n;

  /* Product: '<S28>/Product5' */
  rtb_Add7 = rtb_Ax;

  /* MinMax: '<S28>/Max' */
  rtb_deltafalllimit_cz = fminf(rtb_Add10_b, rtb_Switch2_mn);

  /* Sum: '<S28>/Add1' */
  rtb_Add10_b = rtb_Ax + rtb_deltafalllimit_cz;

  /* Saturate: '<S28>/Saturation' */
  if (rtb_Add10_b <= 0.0F) {
    rtb_Add10_b = 0.0F;
  }

  /* End of Saturate: '<S28>/Saturation' */

  /* MinMax: '<S28>/Max1' */
  rtb_Add10_b = fminf(rtb_Add7, rtb_Add10_b);

  /* UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_on = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pt;

  /* SampleTimeMath: '<S42>/sample time'
   *
   * About '<S42>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S42>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant41'
   */
  rtb_Gain5 = 2000.0 * elapseTime;

  /* UnitDelay: '<S47>/Delay Input2'
   *
   * Block description for '<S47>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add5 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_lt;

  /* SampleTimeMath: '<S47>/sample time'
   *
   * About '<S47>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime_0 = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S47>/delta rise limit' incorporates:
   *  Constant: '<S10>/Constant44'
   */
  rtb_Switch2_hly = 1000.0 * elapseTime_0;

  /* Logic: '<S10>/AND2' */
  rtb_Compare_am = (rtb_LogicalOperator2 && rtb_LogicalOperator7_m);

  /* Logic: '<S10>/OR2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay3'
   */
  rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_i ||
                    rtb_Compare_am);

  /* Switch: '<S10>/Switch7' incorporates:
   *  Constant: '<S10>/Constant39'
   */
  if (rtb_Compare_am) {
    rtb_Gain20 = 0.0;
  } else {
    /* Logic: '<S10>/NOT' */
    rtb_LogicalOperator2 = !rtb_LogicalOperator2;

    /* Switch: '<S10>/Switch8' incorporates:
     *  Constant: '<S10>/Constant40'
     */
    if (!rtb_LogicalOperator2) {
      rtb_Gain4 = 0.0;
    }

    /* End of Switch: '<S10>/Switch8' */
    rtb_Gain20 = rtb_Gain4;
  }

  /* End of Switch: '<S10>/Switch7' */

  /* Gain: '<S10>/Gain20' */
  rtb_Gain20 = -rtb_Gain20;

  /* Saturate: '<S10>/Saturation2' */
  if (rtb_Gain20 > 100.0) {
    rtb_Gain20 = 100.0;
  } else {
    if (rtb_Gain20 < 0.0) {
      rtb_Gain20 = 0.0;
    }
  }

  /* End of Saturate: '<S10>/Saturation2' */

  /* Sum: '<S47>/Difference Inputs1'
   *
   * Block description for '<S47>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_UkYk1 = rtb_Gain20 - rtb_Add5;

  /* RelationalOperator: '<S64>/LowerRelop1' */
  rtb_Compare_am = (rtb_UkYk1 > rtb_Switch2_hly);

  /* Switch: '<S64>/Switch2' */
  if (!rtb_Compare_am) {
    /* Product: '<S47>/delta fall limit' */
    rtb_Yk1_l = -1000.0 * elapseTime_0;

    /* RelationalOperator: '<S64>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_UkYk1 < rtb_Yk1_l);

    /* Switch: '<S64>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_UkYk1 = rtb_Yk1_l;
    }

    /* End of Switch: '<S64>/Switch' */
    rtb_Switch2_hly = rtb_UkYk1;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_lt = rtb_Switch2_hly +
    rtb_Add5;

  /* Gain: '<S10>/Gain14' */
  rtb_deltafalllimit_ap = 0.0174532924F * rtb_FRWhlStrAng;

  /* Trigonometry: '<S10>/Cos2' */
  rtb_deltafalllimit_ap = cosf(rtb_deltafalllimit_ap);

  /* Gain: '<S10>/Gain13' */
  rtb_deltafalllimit_ap *= 1.2F;

  /* Sum: '<S10>/Add8' incorporates:
   *  Constant: '<S10>/Constant28'
   */
  rtb_Switch2_mn = 90.0F - rtb_FRWhlStrAng;

  /* Gain: '<S10>/Gain15' */
  rtb_Switch2_mn *= 0.0174532924F;

  /* Trigonometry: '<S10>/Cos3' */
  rtb_Switch2_mn = cosf(rtb_Switch2_mn);

  /* Gain: '<S10>/Gain12' */
  rtb_Switch2_mn *= 1.522F;

  /* Sum: '<S10>/Add9' */
  rtb_deltafalllimit_ap += rtb_Switch2_mn;

  /* Product: '<S10>/Divide2' */
  rtb_Add5 = WhlSpdFR / rtb_deltafalllimit_ap;

  /* Gain: '<S10>/Gain24' */
  rtb_Add5 *= 0.2;

  /* RelationalOperator: '<S55>/LowerRelop1' */
  rtb_Compare_am = (rtb_Add5 > rtb_Gain26);

  /* Switch: '<S55>/Switch2' */
  if (rtb_Compare_am) {
    rtb_Add5 = rtb_Gain26;
  } else {
    /* Gain: '<S10>/Gain28' */
    rtb_FRWhlStrAng = -rtb_Gain26;

    /* RelationalOperator: '<S55>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_Add5 < rtb_FRWhlStrAng);

    /* Switch: '<S55>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_Add5 = rtb_FRWhlStrAng;
    }

    /* End of Switch: '<S55>/Switch' */
  }

  /* End of Switch: '<S55>/Switch2' */

  /* Sum: '<S10>/Add5' */
  rtb_Add5 = rtb_MaxWhlSpd_mps_n - rtb_Add5;

  /* RelationalOperator: '<S49>/LowerRelop1' */
  rtb_Compare_am = (rtb_Add5 > rtb_Add4_j);

  /* Switch: '<S49>/Switch2' */
  if (rtb_Compare_am) {
    rtb_Add5 = rtb_Add4_j;
  } else {
    /* RelationalOperator: '<S49>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    rtb_LogicalOperator2 = (rtb_Add5 < 0.0);

    /* Switch: '<S49>/Switch' incorporates:
     *  Constant: '<S10>/Constant7'
     */
    if (rtb_LogicalOperator2) {
      rtb_Add5 = 0.0;
    }

    /* End of Switch: '<S49>/Switch' */
  }

  /* End of Switch: '<S49>/Switch2' */

  /* Sum: '<S10>/Add12' incorporates:
   *  UnitDelay: '<S47>/Delay Input2'
   *
   * Block description for '<S47>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = rtb_Add5 + VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_lt;

  /* RelationalOperator: '<S52>/LowerRelop1' */
  rtb_Compare_am = (rtb_UkYk1 > rtb_Add4_j);

  /* Switch: '<S52>/Switch2' */
  if (rtb_Compare_am) {
    rtb_UkYk1 = rtb_Add4_j;
  } else {
    /* RelationalOperator: '<S52>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    rtb_LogicalOperator2 = (rtb_UkYk1 < 0.0);

    /* Switch: '<S52>/Switch' incorporates:
     *  Constant: '<S10>/Constant30'
     */
    if (rtb_LogicalOperator2) {
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
  rtb_UkYk1 -= rtb_Switch2_on;

  /* RelationalOperator: '<S59>/LowerRelop1' */
  rtb_Compare_am = (rtb_UkYk1 > rtb_Gain5);

  /* Switch: '<S59>/Switch2' */
  if (!rtb_Compare_am) {
    /* Product: '<S42>/delta fall limit' */
    rtb_Yk1_l = -2000.0 * elapseTime;

    /* RelationalOperator: '<S59>/UpperRelop' */
    rtb_LogicalOperator2 = (rtb_UkYk1 < rtb_Yk1_l);

    /* Switch: '<S59>/Switch' */
    if (rtb_LogicalOperator2) {
      rtb_UkYk1 = rtb_Yk1_l;
    }

    /* End of Switch: '<S59>/Switch' */
    rtb_Gain5 = rtb_UkYk1;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pt = rtb_Gain5 +
    rtb_Switch2_on;

  /* Switch: '<S7>/Switch8' incorporates:
   *  UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (WhlSpdFL != 0.0) {
    rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_pt;
  } else {
    /* Product: '<S7>/Product' incorporates:
     *  Constant: '<S7>/Constant17'
     */
    rtb_UkYk1 = Acc_POS_n * 0.099999999999999978;
  }

  /* End of Switch: '<S7>/Switch8' */

  /* RelationalOperator: '<S26>/LowerRelop1' */
  rtb_Compare_am = (rtb_UkYk1 > rtb_Add10_b);

  /* Switch: '<S26>/Switch2' */
  if (rtb_Compare_am) {
    rtb_Add4_j = rtb_Add10_b;
  } else {
    /* RelationalOperator: '<S26>/UpperRelop' incorporates:
     *  Constant: '<S7>/Constant15'
     */
    rtb_LogicalOperator2 = (rtb_UkYk1 < 0.0);

    /* Switch: '<S26>/Switch' incorporates:
     *  Constant: '<S7>/Constant15'
     */
    if (rtb_LogicalOperator2) {
      rtb_Add4_j = 0.0F;
    } else {
      rtb_Add4_j = (real32_T)rtb_UkYk1;
    }

    /* End of Switch: '<S26>/Switch' */
  }

  /* End of Switch: '<S26>/Switch2' */

  /* Switch: '<S31>/Switch6' incorporates:
   *  Constant: '<S31>/Verror_Reset'
   *  UnitDelay: '<S82>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl) {
    rtb_Switch2_mn = FLWhlStrAng;
  } else {
    rtb_Switch2_mn = 0.0F;
  }

  /* End of Switch: '<S31>/Switch6' */

  /* Product: '<S31>/Product' incorporates:
   *  Constant: '<S31>/P_Gain'
   */
  rtb_FRWhlStrAng = rtb_Switch2_mn * 40.0F;

  /* Sum: '<S31>/Add11' */
  rtb_deltafalllimit_ap = rtb_Add4_j - rtb_FRWhlStrAng;

  /* UnitDelay: '<S31>/Unit Delay5' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i;

  /* Product: '<S31>/Product2' */
  rtb_Add10_b *= FLWhlStrAng;

  /* RelationalOperator: '<S79>/Compare' incorporates:
   *  Constant: '<S79>/Constant'
   */
  rtb_Compare_am = (rtb_Add10_b <= 0.0F);

  /* UnitDelay: '<S31>/Unit Delay' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f;

  /* Switch: '<S31>/Switch3' incorporates:
   *  Constant: '<S31>/Verror_Reset1'
   */
  if (rtb_Compare_am) {
    rtb_Add10_b = 0.0F;
  }

  /* End of Switch: '<S31>/Switch3' */

  /* Sum: '<S31>/Add2' */
  rtb_Add10_b += rtb_Switch2_mn;

  /* Saturate: '<S31>/Saturation2' */
  if (rtb_Add10_b > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f = 400.0F;
  } else if (rtb_Add10_b < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f = rtb_Add10_b;
  }

  /* End of Saturate: '<S31>/Saturation2' */

  /* RelationalOperator: '<S87>/Compare' incorporates:
   *  UnitDelay: '<S82>/Unit Delay1'
   */
  rtb_LogicalOperator2 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl;

  /* UnitDelay: '<S81>/Delay Input1'
   *
   * Block description for '<S81>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_Compare_am = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_j;

  /* RelationalOperator: '<S81>/FixPt Relational Operator' */
  rtb_Compare_am = ((int32_T)rtb_LogicalOperator2 > (int32_T)rtb_Compare_am);

  /* Switch: '<S31>/Switch' incorporates:
   *  Constant: '<S31>/Integr_StartPoint'
   */
  if (rtb_Compare_am) {
    /* Sum: '<S31>/Add4' */
    rtb_Switch2_mn = rtb_Add4_j -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_f;
  } else {
    rtb_Switch2_mn = 0.0F;
  }

  /* End of Switch: '<S31>/Switch' */

  /* Lookup_n-D: '<S31>/VehicleStableTarget_mps' */
  rtb_Add10_b = look1_iflf_binlc(rtb_Fz3,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S31>/Add5' */
  rtb_Add10_b += rtb_Fz3;

  /* Sum: '<S31>/Add10' */
  rtb_Add10_b = rtb_Add12_j - rtb_Add10_b;

  /* RelationalOperator: '<S31>/Relational Operator' incorporates:
   *  Constant: '<S31>/Verror'
   */
  rtb_Compare_am = (rtb_Add10_b < 0.0F);

  /* Logic: '<S31>/Logical Operator4' incorporates:
   *  UnitDelay: '<S82>/Unit Delay1'
   */
  rtb_Compare_am = (rtb_Compare_am &&
                    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl);

  /* Switch: '<S31>/Switch1' incorporates:
   *  Constant: '<S31>/Trq_IReset'
   *  Constant: '<S31>/Trq_I_FF'
   */
  if (rtb_Compare_am) {
    rtb_Add10_b = 20.0F;
  } else {
    rtb_Add10_b = 0.0F;
  }

  /* End of Switch: '<S31>/Switch1' */

  /* Sum: '<S31>/Add6' incorporates:
   *  UnitDelay: '<S31>/Unit Delay'
   */
  rtb_Switch2_mn = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_f +
                    rtb_Switch2_mn) + rtb_Add10_b;

  /* Product: '<S31>/Product1' incorporates:
   *  Constant: '<S31>/I_Gain'
   */
  rtb_Switch2_mn *= 10.0F;

  /* RelationalOperator: '<S84>/LowerRelop1' */
  rtb_Compare_am = (rtb_Switch2_mn > rtb_deltafalllimit_ap);

  /* Switch: '<S84>/Switch2' */
  if (!rtb_Compare_am) {
    /* Gain: '<S31>/Gain3' */
    rtb_deltafalllimit_ap = -rtb_FRWhlStrAng;

    /* RelationalOperator: '<S84>/UpperRelop' */
    rtb_LogicalOperator7_m = (rtb_Switch2_mn < rtb_deltafalllimit_ap);

    /* Switch: '<S84>/Switch' */
    if (rtb_LogicalOperator7_m) {
      rtb_Switch2_mn = rtb_deltafalllimit_ap;
    }

    /* End of Switch: '<S84>/Switch' */
    rtb_deltafalllimit_ap = rtb_Switch2_mn;
  }

  /* End of Switch: '<S84>/Switch2' */

  /* Sum: '<S31>/Add7' incorporates:
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i = rtb_FRWhlStrAng +
    rtb_deltafalllimit_ap;

  /* Lookup_n-D: '<S31>/VehicleStableTarget_mps1' */
  rtb_Switch2_mn = look1_iflf_binlc(rtb_Fz3,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S31>/Add13' */
  rtb_Fz3 += rtb_Switch2_mn;

  /* Sum: '<S31>/Add12' */
  rtb_Add12_j -= rtb_Fz3;

  /* RelationalOperator: '<S31>/Relational Operator1' incorporates:
   *  Constant: '<S31>/Verror1'
   */
  rtb_Compare_am = (rtb_Add12_j < 0.0F);

  /* RelationalOperator: '<S31>/Relational Operator2' incorporates:
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i >=
                       rtb_Add4_j);

  /* RelationalOperator: '<S80>/Compare' incorporates:
   *  Constant: '<S80>/Constant'
   *  UnitDelay: '<S31>/Unit Delay4'
   */
  rtb_LogicalOperator7_m = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i <=
    5.0F);

  /* Logic: '<S31>/OR' */
  rtb_LowerRelop1_b = (rtb_LowerRelop1_b || rtb_LogicalOperator7_m);

  /* Logic: '<S31>/Logical Operator5' incorporates:
   *  UnitDelay: '<S31>/Unit Delay3'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e = (rtb_Compare_am &&
    rtb_LowerRelop1_b);

  /* Switch: '<S31>/Switch2' incorporates:
   *  Switch: '<S31>/Switch7'
   *  UnitDelay: '<S31>/Unit Delay3'
   *  UnitDelay: '<S82>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_gl) {
    /* RelationalOperator: '<S85>/LowerRelop1' incorporates:
     *  Constant: '<S31>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S31>/Unit Delay4'
     */
    rtb_LogicalOperator7_m =
      (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i > 235.0F);

    /* Switch: '<S85>/Switch2' incorporates:
     *  Constant: '<S31>/TCS_TrqRequest_Max2'
     */
    if (rtb_LogicalOperator7_m) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr = 235.0F;
    } else {
      /* RelationalOperator: '<S85>/UpperRelop' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S31>/Unit Delay4'
       */
      rtb_LogicalOperator7_m =
        (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i < 0.0F);

      /* Switch: '<S85>/Switch' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S31>/Unit Delay4'
       */
      if (rtb_LogicalOperator7_m) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr = 0.0F;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_i;
      }

      /* End of Switch: '<S85>/Switch' */
    }

    /* End of Switch: '<S85>/Switch2' */

    /* RelationalOperator: '<S86>/LowerRelop1' */
    rtb_LogicalOperator7_m = (rtb_Add4_j >
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr);

    /* Switch: '<S86>/Switch2' */
    if (!rtb_LogicalOperator7_m) {
      /* RelationalOperator: '<S86>/UpperRelop' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min1'
       */
      rtb_LogicalOperator7_m = (rtb_Add4_j < 0.0F);

      /* Switch: '<S86>/Switch' incorporates:
       *  Constant: '<S31>/TCS_TrqRequest_Min1'
       */
      if (rtb_LogicalOperator7_m) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr = 0.0F;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr = rtb_Add4_j;
      }

      /* End of Switch: '<S86>/Switch' */
    }

    /* End of Switch: '<S86>/Switch2' */
  } else {
    if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_e) {
      /* Switch: '<S31>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr = rtb_Add4_j;
    }
  }

  /* End of Switch: '<S31>/Switch2' */

  /* Switch: '<S7>/Switch3' incorporates:
   *  Constant: '<S7>/Constant6'
   *  Switch: '<S7>/Switch6'
   */
  if (rtb_Compare) {
    rtb_Add4_j = 0.0F;
  } else {
    if (VehCtrlMdel240926_2018b_amksp_B.TCSF_Enable_OUT != 0.0) {
      /* Switch: '<S7>/Switch6' incorporates:
       *  UnitDelay: '<S31>/Unit Delay2'
       */
      rtb_Add4_j = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_jr;
    }

    /* RelationalOperator: '<S23>/LowerRelop1' */
    rtb_Compare = (rtb_Add4_j > rtb_CastToBoolean1);

    /* Switch: '<S23>/Switch2' */
    if (rtb_Compare) {
      rtb_Add4_j = rtb_CastToBoolean1;
    } else {
      /* RelationalOperator: '<S23>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      rtb_Compare = (rtb_Add4_j < 0.0F);

      /* Switch: '<S23>/Switch' incorporates:
       *  Constant: '<S7>/Constant7'
       */
      if (rtb_Compare) {
        rtb_Add4_j = 0.0F;
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
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn;

  /* Sum: '<S20>/Difference Inputs1'
   *
   * Block description for '<S20>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Add10_b = rtb_Add4_j - rtb_Switch2_mn;

  /* SampleTimeMath: '<S20>/sample time'
   *
   * About '<S20>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S20>/delta rise limit' */
  rtb_deltafalllimit_ap = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S66>/LowerRelop1' */
  rtb_Compare = (rtb_Add10_b > rtb_deltafalllimit_ap);

  /* Switch: '<S66>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S20>/delta fall limit' */
    rtb_deltafalllimit_ap = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S66>/UpperRelop' */
    rtb_Compare = (rtb_Add10_b < rtb_deltafalllimit_ap);

    /* Switch: '<S66>/Switch' */
    if (rtb_Compare) {
      rtb_Add10_b = rtb_deltafalllimit_ap;
    }

    /* End of Switch: '<S66>/Switch' */
    rtb_deltafalllimit_ap = rtb_Add10_b;
  }

  /* End of Switch: '<S66>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn = rtb_deltafalllimit_ap
    + rtb_Switch2_mn;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn > 2.0F) {
    AMKTrqFR_cmd = 2.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn < -2.0F) {
    AMKTrqFR_cmd = -2.0F;
  } else {
    AMKTrqFR_cmd = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hn;
  }

  /* End of Saturate: '<S7>/Saturation2' */

  /* UnitDelay: '<S41>/Delay Input2'
   *
   * Block description for '<S41>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Switch2_on = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hj;

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
  rtb_Compare = (WhlSpdRL_mps >
                 VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m);

  /* Switch: '<S48>/Switch2' */
  if (rtb_Compare) {
    rtb_Add5 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m;
  } else {
    /* RelationalOperator: '<S48>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    rtb_Compare = (WhlSpdRL_mps < 0.0);

    /* Switch: '<S48>/Switch' incorporates:
     *  Constant: '<S10>/Constant6'
     */
    if (rtb_Compare) {
      WhlSpdRL_mps = 0.0;
    }

    /* End of Switch: '<S48>/Switch' */
    rtb_Add5 = WhlSpdRL_mps;
  }

  /* End of Switch: '<S48>/Switch2' */

  /* Sum: '<S10>/Add11' incorporates:
   *  UnitDelay: '<S47>/Delay Input2'
   *
   * Block description for '<S47>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1 = rtb_Add5 + VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_lt;

  /* RelationalOperator: '<S51>/LowerRelop1' */
  rtb_Compare = (rtb_UkYk1 > VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m);

  /* Switch: '<S51>/Switch2' */
  if (rtb_Compare) {
    rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m;
  } else {
    /* RelationalOperator: '<S51>/UpperRelop' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    rtb_Compare = (rtb_UkYk1 < 0.0);

    /* Switch: '<S51>/Switch' incorporates:
     *  Constant: '<S10>/Constant29'
     */
    if (rtb_Compare) {
      rtb_UkYk1 = 0.0;
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
  rtb_UkYk1 -= rtb_Switch2_on;

  /* RelationalOperator: '<S58>/LowerRelop1' */
  rtb_Compare = (rtb_UkYk1 > rtb_Gain5);

  /* Switch: '<S58>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S41>/delta fall limit' */
    rtb_Yk1_l = -2000.0 * elapseTime;

    /* RelationalOperator: '<S58>/UpperRelop' */
    rtb_Compare = (rtb_UkYk1 < rtb_Yk1_l);

    /* Switch: '<S58>/Switch' */
    if (rtb_Compare) {
      rtb_UkYk1 = rtb_Yk1_l;
    }

    /* End of Switch: '<S58>/Switch' */
    rtb_Gain5 = rtb_UkYk1;
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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hj = rtb_Gain5 +
    rtb_Switch2_on;

  /* Switch: '<S7>/Switch9' incorporates:
   *  UnitDelay: '<S41>/Delay Input2'
   *
   * Block description for '<S41>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (WhlSpdFL != 0.0) {
    rtb_UkYk1 = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_hj;
  } else {
    /* Product: '<S7>/Product1' incorporates:
     *  Constant: '<S7>/Constant17'
     */
    rtb_UkYk1 = Acc_POS_n * 0.099999999999999978;
  }

  /* End of Switch: '<S7>/Switch9' */

  /* Sum: '<S28>/Add2' */
  rtb_Ax += rtb_deltafalllimit_cz;

  /* Saturate: '<S28>/Saturation1' */
  if (rtb_Ax <= 0.0F) {
    rtb_Ax = 0.0F;
  }

  /* End of Saturate: '<S28>/Saturation1' */

  /* MinMax: '<S28>/Max2' */
  rtb_Add7 = fminf(rtb_Add7, rtb_Ax);

  /* RelationalOperator: '<S27>/LowerRelop1' */
  rtb_Compare = (rtb_UkYk1 > rtb_Add7);

  /* Switch: '<S27>/Switch2' */
  if (!rtb_Compare) {
    /* RelationalOperator: '<S27>/UpperRelop' incorporates:
     *  Constant: '<S7>/Constant2'
     */
    rtb_Compare = (rtb_UkYk1 < 0.0);

    /* Switch: '<S27>/Switch' incorporates:
     *  Constant: '<S7>/Constant2'
     */
    if (rtb_Compare) {
      rtb_Add7 = 0.0F;
    } else {
      rtb_Add7 = (real32_T)rtb_UkYk1;
    }

    /* End of Switch: '<S27>/Switch' */
  }

  /* End of Switch: '<S27>/Switch2' */

  /* RelationalOperator: '<S78>/Compare' incorporates:
   *  UnitDelay: '<S73>/Unit Delay1'
   */
  rtb_LogicalOperator7_m = VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp;

  /* UnitDelay: '<S72>/Delay Input1'
   *
   * Block description for '<S72>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_Compare = VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_b;

  /* RelationalOperator: '<S72>/FixPt Relational Operator' */
  rtb_Compare = ((int32_T)rtb_LogicalOperator7_m > (int32_T)rtb_Compare);

  /* Switch: '<S30>/Switch' incorporates:
   *  Constant: '<S30>/Integr_StartPoint'
   */
  if (rtb_Compare) {
    /* Sum: '<S30>/Add4' */
    rtb_Switch2_mn = rtb_Add7 -
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g4;
  } else {
    rtb_Switch2_mn = 0.0F;
  }

  /* End of Switch: '<S30>/Switch' */

  /* Switch: '<S30>/Switch6' incorporates:
   *  Constant: '<S30>/Verror_Reset'
   *  UnitDelay: '<S73>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp) {
    rtb_deltafalllimit_ap = rtb_Gain3_o;
  } else {
    rtb_deltafalllimit_ap = 0.0F;
  }

  /* End of Switch: '<S30>/Switch6' */

  /* UnitDelay: '<S30>/Unit Delay5' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_ip;

  /* Product: '<S30>/Product2' */
  rtb_Add10_b *= rtb_Gain3_o;

  /* RelationalOperator: '<S70>/Compare' incorporates:
   *  Constant: '<S70>/Constant'
   */
  rtb_Compare = (rtb_Add10_b <= 0.0F);

  /* UnitDelay: '<S30>/Unit Delay' */
  rtb_Add10_b = VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr;

  /* Switch: '<S30>/Switch3' incorporates:
   *  Constant: '<S30>/Verror_Reset1'
   */
  if (rtb_Compare) {
    rtb_Add10_b = 0.0F;
  }

  /* End of Switch: '<S30>/Switch3' */

  /* Sum: '<S30>/Add2' */
  rtb_Add10_b += rtb_deltafalllimit_ap;

  /* Saturate: '<S30>/Saturation2' */
  if (rtb_Add10_b > 400.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = 400.0F;
  } else if (rtb_Add10_b < -100.0F) {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = -100.0F;
  } else {
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr = rtb_Add10_b;
  }

  /* End of Saturate: '<S30>/Saturation2' */

  /* Lookup_n-D: '<S30>/VehicleStableTarget_mps' */
  rtb_Add10_b = look1_iflf_binlc(rtb_VxIMU_est,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S30>/Add5' */
  rtb_Add10_b += rtb_VxIMU_est;

  /* Sum: '<S30>/Add10' */
  rtb_Add10_b = rtb_Switch2_b0 - rtb_Add10_b;

  /* RelationalOperator: '<S30>/Relational Operator' incorporates:
   *  Constant: '<S30>/Verror'
   */
  rtb_Compare = (rtb_Add10_b < 0.0F);

  /* Logic: '<S30>/Logical Operator4' incorporates:
   *  UnitDelay: '<S73>/Unit Delay1'
   */
  rtb_Compare = (rtb_Compare &&
                 VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp);

  /* Switch: '<S30>/Switch1' incorporates:
   *  Constant: '<S30>/Trq_IReset'
   *  Constant: '<S30>/Trq_I_FF'
   */
  if (rtb_Compare) {
    rtb_Add10_b = 20.0F;
  } else {
    rtb_Add10_b = 0.0F;
  }

  /* End of Switch: '<S30>/Switch1' */

  /* Sum: '<S30>/Add6' incorporates:
   *  UnitDelay: '<S30>/Unit Delay'
   */
  rtb_Switch2_mn = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_nr +
                    rtb_Switch2_mn) + rtb_Add10_b;

  /* Product: '<S30>/Product1' incorporates:
   *  Constant: '<S30>/I_Gain'
   */
  rtb_deltafalllimit_cz = rtb_Switch2_mn * 10.0F;

  /* Product: '<S30>/Product' incorporates:
   *  Constant: '<S30>/P_Gain'
   *  UnitDelay: '<S30>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g4 = rtb_deltafalllimit_ap *
    40.0F;

  /* Sum: '<S30>/Add11' incorporates:
   *  UnitDelay: '<S30>/Unit Delay1'
   */
  rtb_Switch2_mn = rtb_Add7 -
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g4;

  /* RelationalOperator: '<S75>/LowerRelop1' */
  rtb_Compare = (rtb_deltafalllimit_cz > rtb_Switch2_mn);

  /* Switch: '<S75>/Switch2' */
  if (!rtb_Compare) {
    /* Gain: '<S30>/Gain3' incorporates:
     *  UnitDelay: '<S30>/Unit Delay1'
     */
    rtb_deltafalllimit_ap =
      -VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g4;

    /* RelationalOperator: '<S75>/UpperRelop' */
    rtb_Compare = (rtb_deltafalllimit_cz < rtb_deltafalllimit_ap);

    /* Switch: '<S75>/Switch' */
    if (rtb_Compare) {
      rtb_deltafalllimit_cz = rtb_deltafalllimit_ap;
    }

    /* End of Switch: '<S75>/Switch' */
    rtb_Switch2_mn = rtb_deltafalllimit_cz;
  }

  /* End of Switch: '<S75>/Switch2' */

  /* Sum: '<S30>/Add7' incorporates:
   *  UnitDelay: '<S30>/Unit Delay1'
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l =
    VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_g4 + rtb_Switch2_mn;

  /* Lookup_n-D: '<S30>/VehicleStableTarget_mps1' */
  rtb_Switch2_mn = look1_iflf_binlc(rtb_VxIMU_est,
    VehCtrlMdel240926_2018b__ConstP.pooled56,
    VehCtrlMdel240926_2018b__ConstP.pooled62, 3U);

  /* Sum: '<S30>/Add13' */
  rtb_VxIMU_est += rtb_Switch2_mn;

  /* Sum: '<S30>/Add12' */
  rtb_Switch2_b0 -= rtb_VxIMU_est;

  /* RelationalOperator: '<S30>/Relational Operator1' incorporates:
   *  Constant: '<S30>/Verror1'
   */
  rtb_Compare = (rtb_Switch2_b0 < 0.0F);

  /* RelationalOperator: '<S30>/Relational Operator2' incorporates:
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  rtb_Compare_am = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l >=
                    rtb_Add7);

  /* RelationalOperator: '<S71>/Compare' incorporates:
   *  Constant: '<S71>/Constant'
   *  UnitDelay: '<S30>/Unit Delay4'
   */
  rtb_LowerRelop1_b = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l <=
                       5.0F);

  /* Logic: '<S30>/OR' */
  rtb_Compare_am = (rtb_Compare_am || rtb_LowerRelop1_b);

  /* Logic: '<S30>/Logical Operator5' */
  rtb_Compare_am = (rtb_Compare && rtb_Compare_am);

  /* Switch: '<S30>/Switch2' incorporates:
   *  Switch: '<S30>/Switch7'
   *  UnitDelay: '<S73>/Unit Delay1'
   */
  if (VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_dp) {
    /* RelationalOperator: '<S76>/LowerRelop1' incorporates:
     *  Constant: '<S30>/TCS_TrqRequest_Max2'
     *  UnitDelay: '<S30>/Unit Delay4'
     */
    rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l > 235.0F);

    /* Switch: '<S76>/Switch2' incorporates:
     *  Constant: '<S30>/TCS_TrqRequest_Max2'
     */
    if (rtb_Compare) {
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 235.0F;
    } else {
      /* RelationalOperator: '<S76>/UpperRelop' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S30>/Unit Delay4'
       */
      rtb_Compare = (VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l < 0.0F);

      /* Switch: '<S76>/Switch' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min2'
       *  UnitDelay: '<S30>/Unit Delay4'
       */
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 0.0F;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b =
          VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_l;
      }

      /* End of Switch: '<S76>/Switch' */
    }

    /* End of Switch: '<S76>/Switch2' */

    /* RelationalOperator: '<S77>/LowerRelop1' */
    rtb_Compare = (rtb_Add7 >
                   VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b);

    /* Switch: '<S77>/Switch2' */
    if (!rtb_Compare) {
      /* RelationalOperator: '<S77>/UpperRelop' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min1'
       */
      rtb_Compare = (rtb_Add7 < 0.0F);

      /* Switch: '<S77>/Switch' incorporates:
       *  Constant: '<S30>/TCS_TrqRequest_Min1'
       */
      if (rtb_Compare) {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = 0.0F;
      } else {
        VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = rtb_Add7;
      }

      /* End of Switch: '<S77>/Switch' */
    }

    /* End of Switch: '<S77>/Switch2' */
  } else {
    if (rtb_Compare_am) {
      /* Switch: '<S30>/Switch7' */
      VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b = rtb_Add7;
    }
  }

  /* End of Switch: '<S30>/Switch2' */

  /* Logic: '<S7>/OR1' */
  rtb_Compare = (Trq_CUT_final || (VehCtrlMdel240926_2018b_amksp_B.AMKSWITCH_bx
    != 0.0));

  /* Switch: '<S7>/Switch4' incorporates:
   *  Constant: '<S7>/Constant8'
   *  Switch: '<S7>/Switch7'
   */
  if (rtb_Compare) {
    rtb_Add7 = 0.0F;
  } else {
    if (VehCtrlMdel240926_2018b_amksp_B.TCSF_Enable_OUT != 0.0) {
      /* Switch: '<S7>/Switch7' incorporates:
       *  UnitDelay: '<S30>/Unit Delay2'
       */
      rtb_Add7 = VehCtrlMdel240926_2018b_amks_DW.UnitDelay2_DSTATE_b;
    }

    /* RelationalOperator: '<S24>/LowerRelop1' */
    rtb_Compare = (rtb_Add7 > rtb_CastToBoolean1);

    /* Switch: '<S24>/Switch2' */
    if (rtb_Compare) {
      rtb_Add7 = rtb_CastToBoolean1;
    } else {
      /* RelationalOperator: '<S24>/UpperRelop' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      rtb_Compare = (rtb_Add7 < 0.0F);

      /* Switch: '<S24>/Switch' incorporates:
       *  Constant: '<S7>/Constant9'
       */
      if (rtb_Compare) {
        rtb_Add7 = 0.0F;
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
  rtb_Switch2_mn = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib;

  /* Sum: '<S21>/Difference Inputs1'
   *
   * Block description for '<S21>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_CastToBoolean1 = rtb_Add7 - rtb_Switch2_mn;

  /* SampleTimeMath: '<S21>/sample time'
   *
   * About '<S21>/sample time':
   *  y = K where K = ( w * Ts )
   */
  elapseTime = (real_T)FunctionCallSubsystem_ELAPS_T * 0.01;

  /* Product: '<S21>/delta rise limit' */
  rtb_deltafalllimit_ap = (real32_T)(1000.0 * elapseTime);

  /* RelationalOperator: '<S67>/LowerRelop1' */
  rtb_Compare = (rtb_CastToBoolean1 > rtb_deltafalllimit_ap);

  /* Switch: '<S67>/Switch2' */
  if (!rtb_Compare) {
    /* Product: '<S21>/delta fall limit' */
    rtb_deltafalllimit_cz = (real32_T)(-1000.0 * elapseTime);

    /* RelationalOperator: '<S67>/UpperRelop' */
    rtb_Compare = (rtb_CastToBoolean1 < rtb_deltafalllimit_cz);

    /* Switch: '<S67>/Switch' */
    if (rtb_Compare) {
      rtb_CastToBoolean1 = rtb_deltafalllimit_cz;
    }

    /* End of Switch: '<S67>/Switch' */
    rtb_deltafalllimit_ap = rtb_CastToBoolean1;
  }

  /* End of Switch: '<S67>/Switch2' */

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
  VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib = rtb_deltafalllimit_ap
    + rtb_Switch2_mn;
  if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib > 2.0F) {
    AMKTrqFL_cmd = 2.0F;
  } else if (VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib < -2.0F) {
    AMKTrqFL_cmd = -2.0F;
  } else {
    AMKTrqFL_cmd = VehCtrlMdel240926_2018b_amks_DW.DelayInput2_DSTATE_ib;
  }

  /* End of Saturate: '<S7>/Saturation3' */

  /* Sum: '<S10>/Add3' incorporates:
   *  UnitDelay: '<S10>/Unit Delay1'
   */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_c = WhlSpdRR_mps - WhlSpdFR;

  /* SignalConversion generated from: '<S7>/Constant13' incorporates:
   *  Constant: '<S7>/Constant13'
   */
  VehCtrlMdel240926_2018b_amksp_B.VCU_SpdCmd_Emrax = 4200.0F;

  /* Update for UnitDelay: '<S28>/Unit Delay' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay_DSTATE_m = rtb_Add6;

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

  /* Update for UnitDelay: '<S92>/Delay Input1'
   *
   * Block description for '<S92>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_e = rtb_ignition_e;

  /* Update for UnitDelay: '<S32>/Unit Delay5' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_l = rtb_StrWhlAngV_c;

  /* Update for UnitDelay: '<S30>/Unit Delay3' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay3_DSTATE_ip = rtb_Compare_am;

  /* Update for UnitDelay: '<S31>/Unit Delay5' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_i = FLWhlStrAng;

  /* Update for UnitDelay: '<S31>/Unit Delay1' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay1_DSTATE_f = rtb_FRWhlStrAng;

  /* Update for UnitDelay: '<S81>/Delay Input1'
   *
   * Block description for '<S81>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_j = rtb_LogicalOperator2;

  /* Update for UnitDelay: '<S72>/Delay Input1'
   *
   * Block description for '<S72>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_b = rtb_LogicalOperator7_m;

  /* Update for UnitDelay: '<S30>/Unit Delay5' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay5_DSTATE_ip = rtb_Gain3_o;

  /* End of Outputs for S-Function (fcncallgen): '<S1>/10ms1' */

  /* S-Function (fcncallgen): '<S5>/10ms2' incorporates:
   *  SubSystem: '<S5>/VCU2AMKMCUFL'
   */
  /* Switch: '<S345>/Switch1' incorporates:
   *  Constant: '<S345>/Constant1'
   *  Constant: '<S345>/Constant2'
   *  Switch: '<S345>/Switch'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueOn) {
    VehCtrlMdel240926_2018b_amksp_B.Switch1_l = -21.0;
    VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueLimitP = 21.0;
  } else {
    VehCtrlMdel240926_2018b_amksp_B.Switch1_l = 0.0;
    VehCtrlMdel240926_2018b_amksp_B.MCFL_TorqueLimitP = 0.0;
  }

  /* End of Switch: '<S345>/Switch1' */

  /* S-Function (scanpack): '<S345>/CAN Pack1' */
  /* S-Function (scanpack): '<S345>/CAN Pack1' */
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

  /* S-Function (ecucoder_canmessage): '<S345>/CANPackMessage' */

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

  /* S-Function (ec5744_cantransmitslb): '<S345>/CANTransmit' */

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
  /* Switch: '<S346>/Switch1' incorporates:
   *  Constant: '<S346>/Constant'
   *  Constant: '<S346>/Constant1'
   *  Switch: '<S346>/Switch'
   */
  if (VehCtrlMdel240926_2018b_amksp_B.MCFR_TorqueOn) {
    VehCtrlMdel240926_2018b_amksp_B.Switch1 = -21.0;
    VehCtrlMdel240926_2018b_amksp_B.Switch = 21.0;
  } else {
    VehCtrlMdel240926_2018b_amksp_B.Switch1 = 0.0;
    VehCtrlMdel240926_2018b_amksp_B.Switch = 0.0;
  }

  /* End of Switch: '<S346>/Switch1' */

  /* S-Function (scanpack): '<S346>/CAN Pack1' */
  /* S-Function (scanpack): '<S346>/CAN Pack1' */
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

  /* S-Function (ecucoder_canmessage): '<S346>/CANPackMessage' */

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

  /* S-Function (ec5744_cantransmitslb): '<S346>/CANTransmit' */

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
  /* Switch: '<S347>/Switch2' incorporates:
   *  Constant: '<S347>/Constant13'
   *  Constant: '<S347>/Constant17'
   *  Constant: '<S347>/Constant19'
   *  Constant: '<S347>/Constant20'
   *  Switch: '<S347>/Switch3'
   */
  if (TrqR_cmd_raw) {
    Gear_Trs = 0.0;
    Mode_Trs = 0.0;
  } else {
    Gear_Trs = 2.0;
    Mode_Trs = 2.0;
  }

  /* End of Switch: '<S347>/Switch2' */

  /* DataTypeConversion: '<S347>/Cast To Boolean4' */
  VehCtrlMdel240926_2018b_amksp_B.CastToBoolean4 = (real32_T)Gear_Trs;

  /* DataTypeConversion: '<S347>/Cast To Boolean6' */
  VehCtrlMdel240926_2018b_amksp_B.CastToBoolean6 = (real32_T)Mode_Trs;

  /* DataTypeConversion: '<S347>/Data Type Conversion2' */
  VehCtrlMdel240926_2018b_amksp_B.DataTypeConversion2 = (int32_T)floorf
    (EmraxTrqR_cmd);

  /* S-Function (scanpack): '<S347>/CAN Pack1' */
  /* S-Function (scanpack): '<S347>/CAN Pack1' */
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

  /* S-Function (ecucoder_canmessage): '<S347>/CANPackMessage' */

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

  /* S-Function (ec5744_cantransmitslb): '<S347>/CANTransmit' */

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
  /* DataTypeConversion: '<S348>/Cast To Single1' */
  VehCtrlMdel240926_2018b_amksp_B.CastToSingle1 = (uint16_T)rtb_CastToDouble;

  /* S-Function (ec5744_pdsslbu3): '<S348>/PowerDriverSwitch(HS)' */

  /* Set level VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON for the specified power driver switch */
  ec_gpio_write(83,VehCtrlMdel240926_2018b_amksp_B.aWaterPumpON);

  /* S-Function (ec5744_pdsslbu3): '<S348>/PowerDriverSwitch(HS)1' */

  /* Set level VehCtrlMdel240926_2018b_amksp_B.bWaterPumpON for the specified power driver switch */
  ec_gpio_write(55,VehCtrlMdel240926_2018b_amksp_B.bWaterPumpON);

  /* S-Function (ec5744_pdpslbu3): '<S348>/PowerDriverPWM' incorporates:
   *  Constant: '<S348>/Constant'
   */

  /* Power driver PWM output for channel 6 */
  ec_pwm_output(6,((uint16_T)1000U),
                VehCtrlMdel240926_2018b_amksp_B.CastToSingle1);

  /* End of Outputs for S-Function (fcncallgen): '<S5>/10ms6' */

  /* S-Function (fcncallgen): '<S367>/10ms' incorporates:
   *  SubSystem: '<S367>/daq10ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S379>/CCPDAQ' */
  ccpDaq(1);

  /* End of Outputs for S-Function (fcncallgen): '<S367>/10ms' */

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
  /* S-Function (fcncallgen): '<S367>/50ms' incorporates:
   *  SubSystem: '<S367>/daq50ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S381>/CCPDAQ' */
  ccpDaq(2);

  /* End of Outputs for S-Function (fcncallgen): '<S367>/50ms' */
}

/* Model step function for TID5 */
void VehCtrlMdel240926_2018b_amkspdlimit_step5(void) /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S366>/100MS' incorporates:
   *  SubSystem: '<S366>/Function-Call Subsystem'
   */
  /* S-Function (ec5744_canreceiveslb): '<S370>/CANReceive' */

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

  /* Call the system: <S370>/Function-Call Subsystem */

  /* Output and update for function-call system: '<S370>/Function-Call Subsystem' */
  {
    uint8_T rtb_Add;
    uint8_T rtb_Compare;

    /* Outputs for Enabled SubSystem: '<S371>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S372>/Enable'
     */
    if (VehCtrlMdel240926_2018b_amksp_B.CANReceive_o2_p > 0) {
      /* RelationalOperator: '<S373>/Compare' incorporates:
       *  Constant: '<S373>/Constant'
       */
      rtb_Add = (uint8_T)(VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[0] ==
                          83);

      /* RelationalOperator: '<S374>/Compare' incorporates:
       *  Constant: '<S374>/Constant'
       */
      rtb_Compare = (uint8_T)(VehCtrlMdel240926_2018b_amksp_B.CANReceive_o4_i[5]
        == 84);

      /* Sum: '<S372>/Add' */
      rtb_Add = (uint8_T)((uint32_T)rtb_Add + rtb_Compare);

      /* RelationalOperator: '<S375>/Compare' incorporates:
       *  Constant: '<S375>/Constant'
       */
      rtb_Compare = (uint8_T)(rtb_Add == 2);

      /* If: '<S372>/If' */
      if (rtb_Compare > 0) {
        /* Outputs for IfAction SubSystem: '<S372>/If Action Subsystem' incorporates:
         *  ActionPort: '<S376>/Action Port'
         */
        /* S-Function (ec5744_bootloaderslb): '<S376>/BootLoader' */
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

        /* S-Function (ec5744_cpuresetslb): '<S376>/CPUReset' */

        /* Perform a microcontroller reset */
        MC_ME.MCTL.R = 0X00005AF0;
        MC_ME.MCTL.R = 0X0000A50F;

        /* End of Outputs for SubSystem: '<S372>/If Action Subsystem' */
      } else {
        /* Outputs for IfAction SubSystem: '<S372>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S377>/Action Port'
         */
        /* S-Function (ec5744_cantransmitslb): '<S377>/CANTransmit' incorporates:
         *  Constant: '<S377>/Constant'
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

        /* End of Outputs for SubSystem: '<S372>/If Action Subsystem1' */
      }

      /* End of If: '<S372>/If' */
    }

    /* End of Outputs for SubSystem: '<S371>/Enabled Subsystem' */
  }

  /* End of Outputs for S-Function (ec5744_canreceiveslb): '<S370>/CANReceive' */
  /* End of Outputs for S-Function (fcncallgen): '<S366>/100MS' */

  /* S-Function (fcncallgen): '<S367>/100ms' incorporates:
   *  SubSystem: '<S367>/daq100ms'
   */
  /* S-Function (ec5744_ccpslb1): '<S378>/CCPDAQ' */
  ccpDaq(3);

  /* End of Outputs for S-Function (fcncallgen): '<S367>/100ms' */
}

/* Model step function for TID6 */
void VehCtrlMdel240926_2018b_amkspdlimit_step6(void) /* Sample time: [0.5s, 0.0s] */
{
  /* S-Function (fcncallgen): '<S367>/500ms' incorporates:
   *  SubSystem: '<S367>/daq500ms'
   */

  /* S-Function (ec5744_ccpslb1): '<S380>/CCPDAQ' */
  ccpDaq(4);

  /* End of Outputs for S-Function (fcncallgen): '<S367>/500ms' */

  /* S-Function (fcncallgen): '<S368>/500ms' incorporates:
   *  SubSystem: '<S368>/EEPROMOperation'
   */

  /* S-Function (ec5744_eepromoslb): '<S383>/EEPROMOperatin' */
#if defined EC_EEPROM_ENABLE

  /* Operate the EEPROM module on the MPC5744 */
  ec_flash_operation();

#endif

  /* End of Outputs for S-Function (fcncallgen): '<S368>/500ms' */
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
  /* Start for S-Function (ec5744_canreceiveslb): '<S123>/CANReceive1' incorporates:
   *  SubSystem: '<S123>/MCU_pwr'
   */
  /* Start for function-call system: '<S123>/MCU_pwr' */

  /* Start for Enabled SubSystem: '<S178>/MCU_VCUMeter1' */

  /* Start for S-Function (scanunpack): '<S180>/CAN Unpack' */

  /*-----------S-Function Block: <S180>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S178>/MCU_VCUMeter1' */
  ec_buffer_init(0,1,1,218089455);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S123>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S123>/CANReceive3' incorporates:
   *  SubSystem: '<S123>/MCU_state'
   */
  /* Start for function-call system: '<S123>/MCU_state' */

  /* Start for Enabled SubSystem: '<S179>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S186>/CAN Unpack' */

  /*-----------S-Function Block: <S186>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S179>/MCU_state' */
  ec_buffer_init(0,0,1,218089199);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S123>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms3' incorporates:
   *  SubSystem: '<S3>/ABS_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S119>/CANReceive3' incorporates:
   *  SubSystem: '<S119>/ABS_BUS_state'
   */
  /* Start for function-call system: '<S119>/ABS_BUS_state' */

  /* Start for Enabled SubSystem: '<S127>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S128>/CAN Unpack1' */

  /*-----------S-Function Block: <S128>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S127>/IMU_state' */
  ec_buffer_init(2,16,0,1698);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S119>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms3' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms4' incorporates:
   *  SubSystem: '<S3>/StrSnis_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S125>/CANReceive3' incorporates:
   *  SubSystem: '<S125>/StrWhSnis_state'
   */
  /* Start for function-call system: '<S125>/StrWhSnis_state' */

  /* Start for Enabled SubSystem: '<S198>/IMU_state' */

  /* Start for S-Function (scanunpack): '<S199>/CAN Unpack1' */

  /*-----------S-Function Block: <S199>/CAN Unpack1 -----------------*/

  /* End of Start for SubSystem: '<S198>/IMU_state' */
  ec_buffer_init(2,7,0,330);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S125>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms4' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms5' incorporates:
   *  SubSystem: '<S3>/AMKMCU_Receive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive3' incorporates:
   *  SubSystem: '<S133>/AMKMCU_state'
   */
  /* Start for function-call system: '<S133>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S135>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S138>/CAN Unpack' */

  /*-----------S-Function Block: <S138>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S135>/MCU_state' */
  ec_buffer_init(1,1,0,640);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive1' incorporates:
   *  SubSystem: '<S133>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S133>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S136>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S148>/CAN Unpack' */

  /*-----------S-Function Block: <S148>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S136>/MCU_state' */
  ec_buffer_init(1,2,0,642);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive2' incorporates:
   *  SubSystem: '<S133>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S133>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S137>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S150>/CAN Unpack' */

  /*-----------S-Function Block: <S150>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S137>/MCU_state' */
  ec_buffer_init(1,3,0,644);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S133>/CANReceive2' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S134>/CANReceive3' incorporates:
   *  SubSystem: '<S134>/AMKMCU_state'
   */
  /* Start for function-call system: '<S134>/AMKMCU_state' */

  /* Start for Enabled SubSystem: '<S154>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S157>/CAN Unpack' */

  /*-----------S-Function Block: <S157>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S154>/MCU_state' */
  ec_buffer_init(1,4,0,641);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S134>/CANReceive3' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S134>/CANReceive1' incorporates:
   *  SubSystem: '<S134>/AMKMCU_state1'
   */
  /* Start for function-call system: '<S134>/AMKMCU_state1' */

  /* Start for Enabled SubSystem: '<S155>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S166>/CAN Unpack' */

  /*-----------S-Function Block: <S166>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S155>/MCU_state' */
  ec_buffer_init(1,5,0,643);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S134>/CANReceive1' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S134>/CANReceive2' incorporates:
   *  SubSystem: '<S134>/AMKMCU_state2'
   */
  /* Start for function-call system: '<S134>/AMKMCU_state2' */

  /* Start for Enabled SubSystem: '<S156>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S168>/CAN Unpack' */

  /*-----------S-Function Block: <S168>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S156>/MCU_state' */
  ec_buffer_init(1,0,0,645);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S134>/CANReceive2' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms5' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms2' incorporates:
   *  SubSystem: '<S3>/IMU_Recieve'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S124>/CANReceive3' incorporates:
   *  SubSystem: '<S124>/IMU_state'
   */
  /* Start for function-call system: '<S124>/IMU_state' */

  /* Start for Enabled SubSystem: '<S193>/MCU_state' */

  /* Start for S-Function (scanunpack): '<S194>/CAN Unpack' */

  /*-----------S-Function Block: <S194>/CAN Unpack -----------------*/

  /* End of Start for SubSystem: '<S193>/MCU_state' */
  ec_buffer_init(2,17,0,513);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S124>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms2' */

  /* Start for S-Function (fcncallgen): '<S3>/10ms1' incorporates:
   *  SubSystem: '<S3>/BMS_Recive'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' */
  ec_buffer_init(0,3,1,408961267);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S122>/CANReceive3' */
  /* End of Start for S-Function (fcncallgen): '<S3>/10ms1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S345>/CANTransmit' */
  ec_buffer_init(1,8,0,386U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms2' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S346>/CANTransmit' */
  ec_buffer_init(1,9,0,387U);

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms4' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S347>/CANTransmit' */
  ec_buffer_init(0,8,1,146927393U);

  /* End of Start for S-Function (fcncallgen): '<S5>/50ms3' */

  /* Start for S-Function (fcncallgen): '<S5>/10ms6' incorporates:
   *  SubSystem: '<S5>/WP_OUTPUT'
   */
  /* Start for S-Function (ec5744_pdpslbu3): '<S348>/PowerDriverPWM' incorporates:
   *  Constant: '<S348>/Constant'
   */

  /* Initialize PWM output for channel 6 */
  SIUL2_MSCR42 = 0X02000003;           //PWM_A3

  /* End of Start for S-Function (fcncallgen): '<S5>/10ms6' */

  /* Start for S-Function (fcncallgen): '<S366>/100MS' incorporates:
   *  SubSystem: '<S366>/Function-Call Subsystem'
   */
  /* Start for S-Function (ec5744_canreceiveslb): '<S370>/CANReceive' incorporates:
   *  SubSystem: '<S370>/Function-Call Subsystem'
   */
  /* Start for function-call system: '<S370>/Function-Call Subsystem' */

  /* Start for Enabled SubSystem: '<S371>/Enabled Subsystem' */
  /* Start for IfAction SubSystem: '<S372>/If Action Subsystem1' */
  /* Start for S-Function (ec5744_cantransmitslb): '<S377>/CANTransmit' incorporates:
   *  Constant: '<S377>/Constant'
   */
  ec_buffer_init(2,9,0,593U);

  /* End of Start for SubSystem: '<S372>/If Action Subsystem1' */
  /* End of Start for SubSystem: '<S371>/Enabled Subsystem' */
  ec_buffer_init(2,1,0,278);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S370>/CANReceive' */
  /* End of Start for S-Function (fcncallgen): '<S366>/100MS' */

  /* Start for S-Function (fcncallgen): '<S369>/Function-Call Generator' incorporates:
   *  SubSystem: '<S369>/CCPBackground'
   */
  /* Start for S-Function (ec5744_ccpslb): '<S384>/CCPBackground' */
  ccpInit();

  /* End of Start for S-Function (fcncallgen): '<S369>/Function-Call Generator' */

  /* Start for S-Function (ec5744_caninterruptslb1): '<S369>/ReceiveandTransmitInterrupt' incorporates:
   *  SubSystem: '<S369>/CCPReceive'
   */
  /* Start for function-call system: '<S369>/CCPReceive' */

  /* Start for S-Function (ec5744_canreceiveslb): '<S385>/CANReceive' */
  ec_buffer_init(2,0,0,CCP_CRO_ID);

  /* End of Start for S-Function (ec5744_canreceiveslb): '<S385>/CANReceive' */
  ec_bufint_init(2,0);
  INTC_0.PSR[548].B.PRIN = 12;
  IntcIsrVectorTable[548] = (uint32_t)&ISR_FlexCAN_2_MB0;

  /* End of Start for S-Function (ec5744_caninterruptslb1): '<S369>/ReceiveandTransmitInterrupt' */

  /* Start for S-Function (ec5744_eeprombsbu3): '<Root>/EEPROMEnable' */
  Fls_Read(0xFA0010,ecflashdataold,4096);
  Fls_Read(0xFA0010,ecflashdatanew,4096);

  /* SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  /* InitializeConditions for UnitDelay: '<S274>/Unit Delay4' */
  VehCtrlMdel240926_2018b_amks_DW.UnitDelay4_DSTATE_mn = 0.01F;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S4>/10ms1' */

  /* SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' incorporates:
   *  SubSystem: '<S2>/Subsystem'
   */
  /* SystemInitialize for Chart: '<S109>/Chart2' */
  VehCtrlMdel240926_2018b_amks_DW.sfEvent = -1;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S2>/10ms' */

  /* SystemInitialize for S-Function (fcncallgen): '<S5>/10ms1' incorporates:
   *  SubSystem: '<S5>/Beeper'
   */
  /* InitializeConditions for UnitDelay: '<S352>/Delay Input1'
   *
   * Block description for '<S352>/Delay Input1':
   *
   *  Store in Global RAM
   */
  VehCtrlMdel240926_2018b_amks_DW.DelayInput1_DSTATE_n = true;

  /* End of SystemInitialize for S-Function (fcncallgen): '<S5>/10ms1' */

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
  /* Enable for Chart: '<S109>/Chart2' */
  VehCtrlMdel240926_2018b_amks_DW.previousTicks_g =
    VehCtrlMdel240926_2018b_amks_M->Timing.clockTick3;

  /* End of Enable for S-Function (fcncallgen): '<S2>/10ms' */

  /* Enable for S-Function (fcncallgen): '<S5>/10ms1' incorporates:
   *  SubSystem: '<S5>/Beeper'
   */
  /* Enable for Chart: '<S344>/Chart' */
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
