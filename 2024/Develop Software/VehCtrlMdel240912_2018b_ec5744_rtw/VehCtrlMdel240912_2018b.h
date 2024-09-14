/*
 * Code generated for Simulink model VehCtrlMdel240912_2018b.
 *
 * FILE    : VehCtrlMdel240912_2018b.h
 *
 * VERSION : 1.123
 *
 * DATE    : Sat Sep 14 22:02:59 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#ifndef RTW_HEADER_VehCtrlMdel240912_2018b_h_
#define RTW_HEADER_VehCtrlMdel240912_2018b_h_
#include <math.h>
#include "MPC5744P.h"
#include "Std_Types.h"
#include "can.h"
#include "flash.h"
#include "crc.h"
#ifndef VehCtrlMdel240912_2018b_COMMON_INCLUDES_
# define VehCtrlMdel240912_2018b_COMMON_INCLUDES_
#include <string.h>
#include <math.h>
#include "rtwtypes.h"
#include "can_message.h"
#endif                            /* VehCtrlMdel240912_2018b_COMMON_INCLUDES_ */

#include "VehCtrlMdel240912_2018b_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#define EC_EEPROM_ENABLE

typedef union{
  uint32 U;
  float F;
} EE_Data;

/* Macros for accessing real-time model data structure */
#ifndef rtmStepTask
# define rtmStepTask(rtm, idx)         ((rtm)->Timing.TaskCounters.TID[(idx)] == 0)
#endif

#ifndef rtmTaskCounter
# define rtmTaskCounter(rtm, idx)      ((rtm)->Timing.TaskCounters.TID[(idx)])
#endif

#define EnableInterrupts()             asm(" wrteei 1")
#define DisableInterrupts()            asm(" wrteei 0")

/* user code (top of export header file) */
#include "can_message.h"

/* Block states (default storage) for system '<S8>/Timer1' */
typedef struct {
  real_T x;                            /* '<S8>/Timer1' */
  struct {
    uint_T is_c5_VehCtrlMdel240912_2018b:2;/* '<S8>/Timer1' */
    uint_T is_active_c5_VehCtrlMdel240912_:1;/* '<S8>/Timer1' */
  } bitsForTID3;
} DW_Timer1_VehCtrlMdel240912_2_T;

/* Block signals (default storage) */
typedef struct {
  CAN_DATATYPE CANPack1;               /* '<S281>/CAN Pack1' */
  CAN_DATATYPE CANPack1_a;             /* '<S282>/CAN Pack1' */
  CAN_DATATYPE CANPack1_d;             /* '<S280>/CAN Pack1' */
  CAN_DATATYPE CANUnPackMessage4;      /* '<S127>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_g;    /* '<S122>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_j;    /* '<S115>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_n;    /* '<S117>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_p;    /* '<S116>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_e;    /* '<S109>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_c;    /* '<S111>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_b;    /* '<S110>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_l;    /* '<S144>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_ja;   /* '<S103>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_h;    /* '<S135>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_i;    /* '<S119>/CANUnPackMessage4' */
  real_T Exit;                         /* '<S209>/Timer2' */
  real_T Exit_l;                       /* '<S209>/Timer1' */
  real_T Exit_a;                       /* '<S208>/Timer3' */
  real_T Exit_lh;                      /* '<S208>/Timer2' */
  real_T Exit_lh4;                     /* '<S208>/Timer1' */
  real_T Exit_c;                       /* '<S208>/Timer' */
  real_T Exit_h;                       /* '<S154>/Timer3' */
  real_T Exit_o;                       /* '<S154>/Timer2' */
  real_T Exit_i;                       /* '<S154>/Timer1' */
  real_T Exit_le;                      /* '<S154>/Timer' */
  real_T Exit_on;                      /* '<S153>/Timer' */
  real_T low_VOL;                      /* '<S127>/CAN Unpack' */
  real_T MCU_Temp_error;               /* '<S127>/CAN Unpack' */
  real_T Mode;                         /* '<S127>/CAN Unpack' */
  real_T motorTemp_error;              /* '<S127>/CAN Unpack' */
  real_T overCurrent;                  /* '<S127>/CAN Unpack' */
  real_T overpower;                    /* '<S127>/CAN Unpack' */
  real_T overvol;                      /* '<S127>/CAN Unpack' */
  real_T Precharge;                    /* '<S127>/CAN Unpack' */
  real_T Reslove_error;                /* '<S127>/CAN Unpack' */
  real_T voltage;                      /* '<S122>/CAN Unpack' */
  real_T MCFR_ActualTorque;            /* '<S115>/CAN Unpack' */
  real_T MCFR_ActualVelocity_k;        /* '<S115>/CAN Unpack' */
  real_T MCFR_DCVoltage;               /* '<S115>/CAN Unpack' */
  real_T MCFR_bDCOn_j;                 /* '<S115>/CAN Unpack' */
  real_T MCFR_bDerating;               /* '<S115>/CAN Unpack' */
  real_T MCFR_bError;                  /* '<S115>/CAN Unpack' */
  real_T MCFR_bInverterOn;             /* '<S115>/CAN Unpack' */
  real_T MCFR_bQuitDCOn;               /* '<S115>/CAN Unpack' */
  real_T MCFR_bQuitInverterOn;         /* '<S115>/CAN Unpack' */
  real_T MCFR_bReserve;                /* '<S115>/CAN Unpack' */
  real_T MCFR_bSystemReady_j;          /* '<S115>/CAN Unpack' */
  real_T MCFR_bWarn;                   /* '<S115>/CAN Unpack' */
  real_T MCFR_TempIGBT;                /* '<S117>/CAN Unpack' */
  real_T MCFR_TempInverter_i;          /* '<S117>/CAN Unpack' */
  real_T MCFR_TempMotor;               /* '<S117>/CAN Unpack' */
  real_T CANUnpack_o4;                 /* '<S117>/CAN Unpack' */
  real_T CANUnpack_o5;                 /* '<S117>/CAN Unpack' */
  real_T CANUnpack_o6;                 /* '<S117>/CAN Unpack' */
  real_T CANUnpack_o7;                 /* '<S117>/CAN Unpack' */
  real_T CANUnpack_o8;                 /* '<S117>/CAN Unpack' */
  real_T CANUnpack_o9;                 /* '<S117>/CAN Unpack' */
  real_T CANUnpack_o10;                /* '<S117>/CAN Unpack' */
  real_T CANUnpack_o11;                /* '<S117>/CAN Unpack' */
  real_T CANUnpack_o12;                /* '<S117>/CAN Unpack' */
  real_T MCFL_ActualTorque;            /* '<S109>/CAN Unpack' */
  real_T MCFL_ActualVelocity_p;        /* '<S109>/CAN Unpack' */
  real_T MCFL_DCVoltage;               /* '<S109>/CAN Unpack' */
  real_T MCFL_bDCOn_l;                 /* '<S109>/CAN Unpack' */
  real_T MCFL_bDerating;               /* '<S109>/CAN Unpack' */
  real_T MCFL_bError;                  /* '<S109>/CAN Unpack' */
  real_T MCFL_bInverterOn;             /* '<S109>/CAN Unpack' */
  real_T MCFL_bQuitDCOn;               /* '<S109>/CAN Unpack' */
  real_T MCFL_bQuitInverterOn;         /* '<S109>/CAN Unpack' */
  real_T MCFL_bReserve;                /* '<S109>/CAN Unpack' */
  real_T MCFL_bSystemReady_c;          /* '<S109>/CAN Unpack' */
  real_T MCFL_bWarn;                   /* '<S109>/CAN Unpack' */
  real_T MCFL_TempIGBT;                /* '<S111>/CAN Unpack' */
  real_T MCFL_TempInverter_p;          /* '<S111>/CAN Unpack' */
  real_T MCFL_TempMotor;               /* '<S111>/CAN Unpack' */
  real_T ABS_WS_FL;                    /* '<S103>/CAN Unpack1' */
  real_T ABS_WS_FR;                    /* '<S103>/CAN Unpack1' */
  real_T ABS_WS_RL;                    /* '<S103>/CAN Unpack1' */
  real_T ABS_WS_RR;                    /* '<S103>/CAN Unpack1' */
  real_T errorReset;                   /* '<S90>/Chart2' */
  real_T Exit_d;                       /* '<S8>/Timer2' */
  real_T Exit_g;                       /* '<S8>/Timer1' */
  real_T DYC_Enable_OUT;               /* '<S7>/Chart' */
  real_T TCSR_Enable_OUT;              /* '<S7>/Chart' */
  real_T TCSF_Enable_OUT;              /* '<S7>/Chart' */
  uint32_T CANReceive_o3;              /* '<S306>/CANReceive' */
  uint32_T CANReceive_o3_l;            /* '<S291>/CANReceive' */
  uint32_T CANReceive1_o3;             /* '<S98>/CANReceive1' */
  uint32_T CANReceive3_o3;             /* '<S98>/CANReceive3' */
  uint32_T CANReceive3_o3_e;           /* '<S104>/CANReceive3' */
  uint32_T CANReceive1_o3_n;           /* '<S104>/CANReceive1' */
  uint32_T CANReceive2_o3;             /* '<S104>/CANReceive2' */
  uint32_T CANReceive3_o3_i;           /* '<S105>/CANReceive3' */
  uint32_T CANReceive1_o3_h;           /* '<S105>/CANReceive1' */
  uint32_T CANReceive2_o3_j;           /* '<S105>/CANReceive2' */
  uint32_T CANReceive3_o3_c;           /* '<S100>/CANReceive3' */
  uint32_T CANReceive3_o3_m;           /* '<S94>/CANReceive3' */
  uint32_T CANReceive3_o3_cz;          /* '<S99>/CANReceive3' */
  uint32_T CANReceive3_o3_l;           /* '<S97>/CANReceive3' */
  real32_T TrqR_cmd;                   /* '<S7>/Saturation1' */
  real32_T TrqFR_cmd;                  /* '<S7>/Saturation2' */
  real32_T TrqFL_cmd;                  /* '<S7>/Saturation3' */
  int32_T DataTypeConversion2;         /* '<S282>/Data Type Conversion2' */
  uint16_T CastToSingle1;              /* '<S283>/Cast To Single1' */
  uint16_T CastToBoolean4;             /* '<S282>/Cast To Boolean4' */
  uint16_T CastToBoolean6;             /* '<S282>/Cast To Boolean6' */
  uint16_T Acc1;                       /* '<S96>/Acc1' */
  uint16_T Acc2;                       /* '<S96>/Acc2' */
  uint16_T Brk1;                       /* '<S96>/Brk1' */
  uint8_T CANReceive_o2;               /* '<S306>/CANReceive' */
  uint8_T CANReceive_o4[8];            /* '<S306>/CANReceive' */
  uint8_T CANReceive_o5;               /* '<S306>/CANReceive' */
  uint8_T CANReceive_o2_p;             /* '<S291>/CANReceive' */
  uint8_T CANReceive_o4_i[8];          /* '<S291>/CANReceive' */
  uint8_T CANReceive_o5_l;             /* '<S291>/CANReceive' */
  uint8_T CANTransmit;                 /* '<S298>/CANTransmit' */
  uint8_T CANPackMessage[8];           /* '<S281>/CANPackMessage' */
  uint8_T CANTransmit_l;               /* '<S281>/CANTransmit' */
  uint8_T CANPackMessage_d[8];         /* '<S282>/CANPackMessage' */
  uint8_T CANTransmit_k;               /* '<S282>/CANTransmit' */
  uint8_T CANPackMessage_h[8];         /* '<S280>/CANPackMessage' */
  uint8_T CANTransmit_c;               /* '<S280>/CANTransmit' */
  uint8_T CANReceive1_o2;              /* '<S98>/CANReceive1' */
  uint8_T CANReceive1_o4[8];           /* '<S98>/CANReceive1' */
  uint8_T CANReceive1_o5;              /* '<S98>/CANReceive1' */
  uint8_T CANReceive3_o2;              /* '<S98>/CANReceive3' */
  uint8_T CANReceive3_o4[8];           /* '<S98>/CANReceive3' */
  uint8_T CANReceive3_o5;              /* '<S98>/CANReceive3' */
  uint8_T CANReceive3_o2_l;            /* '<S104>/CANReceive3' */
  uint8_T CANReceive3_o4_l[8];         /* '<S104>/CANReceive3' */
  uint8_T CANReceive3_o5_a;            /* '<S104>/CANReceive3' */
  uint8_T CANReceive1_o2_l;            /* '<S104>/CANReceive1' */
  uint8_T CANReceive1_o4_c[8];         /* '<S104>/CANReceive1' */
  uint8_T CANReceive1_o5_a;            /* '<S104>/CANReceive1' */
  uint8_T CANReceive2_o2;              /* '<S104>/CANReceive2' */
  uint8_T CANReceive2_o4[8];           /* '<S104>/CANReceive2' */
  uint8_T CANReceive2_o5;              /* '<S104>/CANReceive2' */
  uint8_T CANReceive3_o2_a;            /* '<S105>/CANReceive3' */
  uint8_T CANReceive3_o4_g[8];         /* '<S105>/CANReceive3' */
  uint8_T CANReceive3_o5_an;           /* '<S105>/CANReceive3' */
  uint8_T CANReceive1_o2_o;            /* '<S105>/CANReceive1' */
  uint8_T CANReceive1_o4_j[8];         /* '<S105>/CANReceive1' */
  uint8_T CANReceive1_o5_j;            /* '<S105>/CANReceive1' */
  uint8_T CANReceive2_o2_p;            /* '<S105>/CANReceive2' */
  uint8_T CANReceive2_o4_k[8];         /* '<S105>/CANReceive2' */
  uint8_T CANReceive2_o5_e;            /* '<S105>/CANReceive2' */
  uint8_T CANReceive3_o2_p;            /* '<S100>/CANReceive3' */
  uint8_T CANReceive3_o4_k[8];         /* '<S100>/CANReceive3' */
  uint8_T CANReceive3_o5_d;            /* '<S100>/CANReceive3' */
  uint8_T CANReceive3_o2_m;            /* '<S94>/CANReceive3' */
  uint8_T CANReceive3_o4_lg[8];        /* '<S94>/CANReceive3' */
  uint8_T CANReceive3_o5_b;            /* '<S94>/CANReceive3' */
  uint8_T CANReceive3_o2_ma;           /* '<S99>/CANReceive3' */
  uint8_T CANReceive3_o4_i[8];         /* '<S99>/CANReceive3' */
  uint8_T CANReceive3_o5_m;            /* '<S99>/CANReceive3' */
  uint8_T CANReceive3_o2_k;            /* '<S97>/CANReceive3' */
  uint8_T CANReceive3_o4_p[8];         /* '<S97>/CANReceive3' */
  uint8_T CANReceive3_o5_de;           /* '<S97>/CANReceive3' */
  boolean_T Drive_ready;               /* '<S101>/SwitchInput' */
  boolean_T AMKCLDCON_e;               /* '<S101>/SwitchInput1' */
  boolean_T HVCUTOFF_j;                /* '<S101>/SwitchInput2' */
  boolean_T Compare_p;                 /* '<S93>/Compare' */
  boolean_T VehReady;                  /* '<S90>/Chart2' */
  boolean_T MCFL_DCOn_setpoints;       /* '<S90>/Chart2' */
  boolean_T MCFL_InverterOn;           /* '<S90>/Chart2' */
  boolean_T MCFL_DCEnable;             /* '<S90>/Chart2' */
  boolean_T aWaterPumpON;
} B_VehCtrlMdel240912_2018b_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DelayInput2_DSTATE;           /* '<S182>/Delay Input2' */
  real_T DelayInput2_DSTATE_a;         /* '<S195>/Delay Input2' */
  real_T DelayInput2_DSTATE_b;         /* '<S196>/Delay Input2' */
  real_T DelayInput2_DSTATE_h;         /* '<S197>/Delay Input2' */
  real_T DelayInput2_DSTATE_n;         /* '<S198>/Delay Input2' */
  real_T UnitDelay_DSTATE;             /* '<S153>/Unit Delay' */
  real_T DelayInput2_DSTATE_l;         /* '<S172>/Delay Input2' */
  real_T UnitDelay_DSTATE_g;           /* '<S171>/Unit Delay' */
  real_T DelayInput2_DSTATE_m;         /* '<S173>/Delay Input2' */
  real_T UnitDelay1_DSTATE;            /* '<S171>/Unit Delay1' */
  real_T DelayInput2_DSTATE_j;         /* '<S183>/Delay Input2' */
  real_T UnitDelay1_DSTATE_a;          /* '<S153>/Unit Delay1' */
  real_T DelayInput2_DSTATE_k;         /* '<S174>/Delay Input2' */
  real_T UnitDelay2_DSTATE;            /* '<S171>/Unit Delay2' */
  real_T DelayInput2_DSTATE_jk;        /* '<S32>/Delay Input2' */
  real_T UnitDelay_DSTATE_n;           /* '<S7>/Unit Delay' */
  real_T UnitDelay2_DSTATE_d;          /* '<S10>/Unit Delay2' */
  real_T UnitDelay_DSTATE_m;           /* '<S10>/Unit Delay' */
  real_T UnitDelay1_DSTATE_p;          /* '<S10>/Unit Delay1' */
  real_T DelayInput2_DSTATE_n0;        /* '<S34>/Delay Input2' */
  real_T DelayInput2_DSTATE_hk;        /* '<S31>/Delay Input2' */
  real_T UnitDelay4_DSTATE;            /* '<S20>/Unit Delay4' */
  real_T UnitDelay2_DSTATE_g;          /* '<S20>/Unit Delay2' */
  real_T UnitDelay4_DSTATE_i;          /* '<S19>/Unit Delay4' */
  real_T UnitDelay4_DSTATE_l;          /* '<S18>/Unit Delay4' */
  real_T DelayInput2_DSTATE_mb;        /* '<S29>/Delay Input2' */
  real_T UnitDelay2_DSTATE_j;          /* '<S19>/Unit Delay2' */
  real_T DelayInput2_DSTATE_o;         /* '<S30>/Delay Input2' */
  real_T UnitDelay2_DSTATE_b;          /* '<S18>/Unit Delay2' */
  real_T x;                            /* '<S151>/Timer' */
  real_T a;                            /* '<S7>/Chart' */
  real_T b;                            /* '<S7>/Chart' */
  real_T DYC_flag;                     /* '<S7>/Chart' */
  real32_T UnitDelay_DSTATE_p;         /* '<S224>/Unit Delay' */
  real32_T DelayInput2_DSTATE_n2;      /* '<S228>/Delay Input2' */
  real32_T UnitDelay_DSTATE_j;         /* '<S217>/Unit Delay' */
  real32_T UnitDelay_DSTATE_pj;        /* '<S225>/Unit Delay' */
  real32_T DelayInput2_DSTATE_e;       /* '<S231>/Delay Input2' */
  real32_T UnitDelay1_DSTATE_n;        /* '<S217>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_a;         /* '<S226>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hk3;     /* '<S234>/Delay Input2' */
  real32_T UnitDelay2_DSTATE_l;        /* '<S217>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_nc;        /* '<S227>/Unit Delay' */
  real32_T DelayInput2_DSTATE_c;       /* '<S237>/Delay Input2' */
  real32_T UnitDelay3_DSTATE;          /* '<S217>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_l;         /* '<S244>/Unit Delay' */
  real32_T DelayInput2_DSTATE_i;       /* '<S248>/Delay Input2' */
  real32_T UnitDelay4_DSTATE_ik;       /* '<S218>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_a0;        /* '<S218>/Unit Delay' */
  real32_T UnitDelay_DSTATE_ap;        /* '<S245>/Unit Delay' */
  real32_T DelayInput2_DSTATE_el;      /* '<S251>/Delay Input2' */
  real32_T UnitDelay5_DSTATE;          /* '<S218>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_am;       /* '<S218>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_o;         /* '<S246>/Unit Delay' */
  real32_T DelayInput2_DSTATE_p;       /* '<S254>/Delay Input2' */
  real32_T UnitDelay6_DSTATE;          /* '<S218>/Unit Delay6' */
  real32_T UnitDelay2_DSTATE_c;        /* '<S218>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_ah;        /* '<S247>/Unit Delay' */
  real32_T DelayInput2_DSTATE_mt;      /* '<S257>/Delay Input2' */
  real32_T UnitDelay7_DSTATE;          /* '<S218>/Unit Delay7' */
  real32_T UnitDelay3_DSTATE_d;        /* '<S218>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_lh;        /* '<S150>/Unit Delay' */
  real32_T UnitDelay4_DSTATE_m;        /* '<S207>/Unit Delay4' */
  real32_T UnitDelay1_DSTATE_k;        /* '<S150>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_o;        /* '<S207>/Unit Delay1' */
  real32_T UnitDelay3_DSTATE_p;        /* '<S209>/Unit Delay3' */
  real32_T UnitDelay1_DSTATE_d;        /* '<S209>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_f;        /* '<S209>/Unit Delay2' */
  real32_T UnitDelay4_DSTATE_mn;       /* '<S209>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_d;         /* '<S207>/Unit Delay' */
  real32_T UnitDelay2_DSTATE_i;        /* '<S207>/Unit Delay2' */
  real32_T DelayInput2_DSTATE_g;       /* '<S269>/Delay Input2' */
  real32_T DelayInput2_DSTATE_af;      /* '<S267>/Delay Input2' */
  real32_T DelayInput2_DSTATE_f;       /* '<S273>/Delay Input2' */
  real32_T UnitDelay_DSTATE_ncs;       /* '<S266>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hu;      /* '<S268>/Delay Input2' */
  real32_T DelayInput2_DSTATE_d;       /* '<S35>/Delay Input2' */
  real32_T UnitDelay4_DSTATE_n;        /* '<S10>/Unit Delay4' */
  real32_T UnitDelay5_DSTATE_b;        /* '<S10>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_i;        /* '<S78>/Unit Delay1' */
  real32_T UnitDelay5_DSTATE_l;        /* '<S20>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_b;         /* '<S20>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_g;        /* '<S20>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_n5;       /* '<S67>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_h;        /* '<S58>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_cd;      /* '<S11>/Delay Input2' */
  real32_T DelayInput2_DSTATE_fo;      /* '<S33>/Delay Input2' */
  real32_T UnitDelay5_DSTATE_i;        /* '<S19>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_f;         /* '<S19>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_f;        /* '<S19>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_hn;      /* '<S12>/Delay Input2' */
  real32_T UnitDelay5_DSTATE_ip;       /* '<S18>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_nr;        /* '<S18>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_g4;       /* '<S18>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_ib;      /* '<S13>/Delay Input2' */
  int32_T sfEvent;                     /* '<S90>/Chart2' */
  uint32_T Subsystem_PREV_T;           /* '<S4>/Subsystem' */
  uint32_T FunctionCallSubsystem_PREV_T;/* '<S4>/Function-Call Subsystem' */
  uint32_T previousTicks;              /* '<S90>/Chart2' */
  uint32_T MoTrqReq_PREV_T;            /* '<S1>/MoTrqReq' */
  int_T CANPack1_ModeSignalID;         /* '<S281>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_l;       /* '<S282>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_f;       /* '<S280>/CAN Pack1' */
  int_T CANUnpack_ModeSignalID;        /* '<S127>/CAN Unpack' */
  int_T CANUnpack_StatusPortID;        /* '<S127>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_b;      /* '<S122>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_m;      /* '<S122>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_a;      /* '<S115>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_o;      /* '<S115>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_n;      /* '<S117>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_f;      /* '<S117>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_c;      /* '<S109>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_ok;     /* '<S109>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_m;      /* '<S111>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_j;      /* '<S111>/CAN Unpack' */
  int_T CANUnpack1_ModeSignalID;       /* '<S144>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID;       /* '<S144>/CAN Unpack1' */
  int_T CANUnpack1_ModeSignalID_m;     /* '<S103>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID_n;     /* '<S103>/CAN Unpack1' */
  int_T CANUnpack_ModeSignalID_f;      /* '<S135>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_k;      /* '<S135>/CAN Unpack' */
  struct {
    uint_T is_VehStat:4;               /* '<S90>/Chart2' */
    uint_T is_AMKDCready:4;            /* '<S90>/Chart2' */
    uint_T is_c7_VehCtrlMdel240912_2018b:4;/* '<S7>/Chart' */
    uint_T is_c3_VehCtrlMdel240912_2018b:2;/* '<S151>/Timer' */
    uint_T is_BeeperStat:2;            /* '<S90>/Chart2' */
    uint_T is_AMKDCon:2;               /* '<S90>/Chart2' */
    uint_T is_MCDCEnable:2;            /* '<S90>/Chart2' */
    uint_T is_AMKCANenable:2;          /* '<S90>/Chart2' */
    uint_T is_MC_InverterOn:2;         /* '<S90>/Chart2' */
    uint_T is_active_c3_VehCtrlMdel240912_:1;/* '<S151>/Timer' */
    uint_T is_active_c1_VehCtrlMdel240912_:1;/* '<S90>/Chart2' */
    uint_T is_active_VehStat:1;        /* '<S90>/Chart2' */
    uint_T is_active_BeeperStat:1;     /* '<S90>/Chart2' */
    uint_T is_active_AMKDCon:1;        /* '<S90>/Chart2' */
    uint_T is_active_MCDCEnable:1;     /* '<S90>/Chart2' */
    uint_T is_active_AMKDCready:1;     /* '<S90>/Chart2' */
    uint_T is_active_Output:1;         /* '<S90>/Chart2' */
    uint_T is_active_AMKCANenable:1;   /* '<S90>/Chart2' */
    uint_T is_active_MC_InverterOn:1;  /* '<S90>/Chart2' */
    uint_T is_active_c7_VehCtrlMdel240912_:1;/* '<S7>/Chart' */
  } bitsForTID3;

  uint16_T UnitDelay_DSTATE_k;         /* '<S151>/Unit Delay' */
  uint16_T UnitDelay1_DSTATE_fm;       /* '<S151>/Unit Delay1' */
  uint16_T temporalCounter_i2;         /* '<S90>/Chart2' */
  boolean_T UnitDelay3_DSTATE_f;       /* '<S207>/Unit Delay3' */
  boolean_T UnitDelay3_DSTATE_l;       /* '<S10>/Unit Delay3' */
  boolean_T UnitDelay6_DSTATE_i;       /* '<S10>/Unit Delay6' */
  boolean_T UnitDelay1_DSTATE_e;       /* '<S77>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_a;       /* '<S20>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE;        /* '<S76>/Delay Input1' */
  boolean_T UnitDelay1_DSTATE_gl;      /* '<S66>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_e;       /* '<S19>/Unit Delay3' */
  boolean_T UnitDelay1_DSTATE_dp;      /* '<S57>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_i;       /* '<S18>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE_j;      /* '<S65>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_b;      /* '<S56>/Delay Input1' */
  uint8_T temporalCounter_i1;          /* '<S90>/Chart2' */
  boolean_T Subsystem_RESET_ELAPS_T;   /* '<S4>/Subsystem' */
  boolean_T FunctionCallSubsystem_RESET_ELA;/* '<S4>/Function-Call Subsystem' */
  boolean_T MoTrqReq_RESET_ELAPS_T;    /* '<S1>/MoTrqReq' */
  DW_Timer1_VehCtrlMdel240912_2_T sf_Timer2_j;/* '<S209>/Timer2' */
  DW_Timer1_VehCtrlMdel240912_2_T sf_Timer1_p;/* '<S209>/Timer1' */
  DW_Timer1_VehCtrlMdel240912_2_T sf_Timer3_i;/* '<S208>/Timer3' */
  DW_Timer1_VehCtrlMdel240912_2_T sf_Timer2_g;/* '<S208>/Timer2' */
  DW_Timer1_VehCtrlMdel240912_2_T sf_Timer1_m;/* '<S208>/Timer1' */
  DW_Timer1_VehCtrlMdel240912_2_T sf_Timer_o;/* '<S208>/Timer' */
  DW_Timer1_VehCtrlMdel240912_2_T sf_Timer3;/* '<S154>/Timer3' */
  DW_Timer1_VehCtrlMdel240912_2_T sf_Timer2_l;/* '<S154>/Timer2' */
  DW_Timer1_VehCtrlMdel240912_2_T sf_Timer1_n;/* '<S154>/Timer1' */
  DW_Timer1_VehCtrlMdel240912_2_T sf_Timer_b;/* '<S154>/Timer' */
  DW_Timer1_VehCtrlMdel240912_2_T sf_Timer_k;/* '<S153>/Timer' */
  DW_Timer1_VehCtrlMdel240912_2_T sf_Timer2;/* '<S8>/Timer2' */
  DW_Timer1_VehCtrlMdel240912_2_T sf_Timer1;/* '<S8>/Timer1' */
} DW_VehCtrlMdel240912_2018b_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: [3 5 8;5 8 10;6 10 13;8 12 14]
   * Referenced by: '<S10>/2-D Lookup Table'
   */
  real_T uDLookupTable_tableData[12];

  /* Expression: [0 5 20 25]
   * Referenced by: '<S10>/2-D Lookup Table'
   */
  real_T uDLookupTable_bp01Data[4];

  /* Expression: [20 40 60]
   * Referenced by: '<S10>/2-D Lookup Table'
   */
  real_T uDLookupTable_bp02Data[3];

  /* Expression: [229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.46;229.16;228.86;228.86;228.86;228.86;228.86;227.86;227.86;226.87;225.86;224.86;220.86;218.87;215.87]
   * Referenced by: '<S10>/228'
   */
  real_T u28_tableData[27];

  /* Expression: [200;400;600;800;1000;1200;1400;1600;1800;2000;2200;2400;2600;2800;3000;3200;3400;3600;3800;4000;4200;4400;4600;4800;5000;5200;5400]
   * Referenced by: '<S10>/228'
   */
  real_T u28_bp01Data[27];

  /* Pooled Parameter (Expression: [21;21;21;21;21;21;21;21;21;21;21;21;21;21;20;17.5;15;12.5;8;0])
   * Referenced by:
   *   '<S10>/AMK'
   *   '<S10>/AMK1'
   */
  real_T pooled13[20];

  /* Pooled Parameter (Expression: [0;1000;2000;3000;4000;5000;6000;7000;8000;9000;10000;11000;12000;13000;14000;15000;16000;17000;18000;19000])
   * Referenced by:
   *   '<S10>/AMK'
   *   '<S10>/AMK1'
   */
  real_T pooled14[20];

  /* Pooled Parameter (Expression: [0,0,0.2,0.4,0.6,0.8,1,1])
   * Referenced by:
   *   '<S8>/2-D Lookup Table1'
   *   '<S8>/2-D Lookup Table3'
   */
  real_T pooled17[8];

  /* Pooled Parameter (Expression: [35,40,41,42,43,44,45,50])
   * Referenced by:
   *   '<S8>/2-D Lookup Table1'
   *   '<S8>/2-D Lookup Table3'
   */
  real_T pooled18[8];

  /* Expression: [0;0;20;40;70;100;100]
   * Referenced by: '<S8>/2-D Lookup Table2'
   */
  real_T uDLookupTable2_tableData[7];

  /* Expression: [0;25;30;35;37;40;50]
   * Referenced by: '<S8>/2-D Lookup Table2'
   */
  real_T uDLookupTable2_bp01Data[7];

  /* Computed Parameter: uDLookupTable_tableData_f
   * Referenced by: '<S153>/1-D Lookup Table'
   */
  real_T uDLookupTable_tableData_f[24];

  /* Computed Parameter: uDLookupTable_bp01Data_o
   * Referenced by: '<S153>/1-D Lookup Table'
   */
  real_T uDLookupTable_bp01Data_o[24];

  /* Computed Parameter: uDLookupTable1_tableData
   * Referenced by: '<S153>/1-D Lookup Table1'
   */
  real_T uDLookupTable1_tableData[24];

  /* Computed Parameter: uDLookupTable1_bp01Data
   * Referenced by: '<S153>/1-D Lookup Table1'
   */
  real_T uDLookupTable1_bp01Data[24];

  /* Pooled Parameter (Expression: single([20,0]);)
   * Referenced by:
   *   '<S7>/BrakeCompensateCoefFront1'
   *   '<S7>/BrakeCompensateCoefFront2'
   */
  real32_T pooled31[2];

  /* Pooled Parameter (Expression: single([550,2000]);)
   * Referenced by:
   *   '<S7>/BrakeCompensateCoefFront1'
   *   '<S7>/BrakeCompensateCoefFront2'
   *   '<S7>/BrakeCompensateCoefRear'
   */
  real32_T pooled32[2];

  /* Expression: single([1000,0]);
   * Referenced by: '<S7>/BrakeCompensateCoefRear'
   */
  real32_T BrakeCompensateCoefRear_tableDa[2];

  /* Expression: single([0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;20 20 20 20 20 20 20 18 18 18 17 17 16 16 15 15 15 14 14 14;35 35 35 35 35 35 30 30 25 25 25 25 20 20 20 20 20 20 20 18;50 50 50 50 50 50 50 50 50 40 40 40 40 40 40 40 40 40 38 35;79 79 79 79 85 85 85 90 95 100 110 110 110 100 90 90 80 72 70 65;100 100 100 105 105 105 110 110 110 110 115 115 125 125 120 110 90 80 76 70;120 120 120 120 125 125 125 135 135 135 135 135 135 130 130 120 100 90 82 74;125 125 125 130 135 140 145 155 155 155 155 155 155 155 155 135 110 100 88 82;130 160 160 160 160 170 160 160 160 160 160 160 160 160 160 142 120 105 94 88;130 160 160 160 160 170 160 160 160 160 160 160 160 160 160 142 120 105 95 91;170 170 180 180 180 210 200 200 200 200 200 200 190 180 180 152 133 118 106 100])
   * Referenced by: '<S7>/2-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData_g[220];

  /* Expression: single([0 10 20 30 40 50 60 70 80 90 100])
   * Referenced by: '<S7>/2-D Lookup Table1'
   */
  real32_T uDLookupTable1_bp01Data_l[11];

  /* Expression: single([5.40000009536743 10.8000001907349 16.2000007629395 21.6000003814697 27 32.4000015258789 37.7999992370605 43.2000007629395 48.5999984741211 54 59.4000015258789 64.8000030517578 70.1999969482422 75.5999984741211 81 86.4000015258789 91.8000030517578 97.1999969482422 102.599998474121 108])
   * Referenced by: '<S7>/2-D Lookup Table1'
   */
  real32_T uDLookupTable1_bp02Data[20];

  /* Pooled Parameter (Expression: [0.3,0.3,2.5,2.5])
   * Referenced by:
   *   '<S18>/VehSpd_SlipTarget_mps'
   *   '<S19>/VehSpd_SlipTarget_mps'
   *   '<S20>/VehSpd_SlipTarget_mps'
   */
  real32_T pooled51[4];

  /* Pooled Parameter (Expression: [0,3,25,30])
   * Referenced by:
   *   '<S18>/VehSpd_SlipTarget_mps'
   *   '<S18>/VehicleStableTarget_mps'
   *   '<S18>/VehicleStableTarget_mps1'
   *   '<S19>/VehSpd_SlipTarget_mps'
   *   '<S19>/VehicleStableTarget_mps'
   *   '<S19>/VehicleStableTarget_mps1'
   *   '<S20>/VehSpd_SlipTarget_mps'
   *   '<S20>/VehicleStableTarget_mps'
   *   '<S20>/VehicleStableTarget_mps1'
   */
  real32_T pooled52[4];

  /* Pooled Parameter (Expression: [0.4,0.4,1.2,1.2])
   * Referenced by:
   *   '<S18>/VehicleStableTarget_mps'
   *   '<S18>/VehicleStableTarget_mps1'
   *   '<S19>/VehicleStableTarget_mps'
   *   '<S19>/VehicleStableTarget_mps1'
   *   '<S20>/VehicleStableTarget_mps'
   *   '<S20>/VehicleStableTarget_mps1'
   */
  real32_T pooled59[4];

  /* Pooled Parameter (Expression: single([0,100]);)
   * Referenced by:
   *   '<S151>/1-D Lookup Table'
   *   '<S151>/1-D Lookup Table3'
   */
  real32_T pooled63[2];

  /* Expression: single([1330,1590]);
   * Referenced by: '<S151>/1-D Lookup Table'
   */
  real32_T uDLookupTable_bp01Data_h[2];

  /* Expression: single([2324,2600]);
   * Referenced by: '<S151>/1-D Lookup Table3'
   */
  real32_T uDLookupTable3_bp01Data[2];

  /* Expression: single([0,10000]);
   * Referenced by: '<S151>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData_m[2];

  /* Expression: single([500,4500]);
   * Referenced by: '<S151>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_bp01Data_f[2];

  /* Computed Parameter: uDLookupTable1_maxIndex
   * Referenced by: '<S7>/2-D Lookup Table1'
   */
  uint32_T uDLookupTable1_maxIndex[2];

  /* Computed Parameter: uDLookupTable_maxIndex
   * Referenced by: '<S10>/2-D Lookup Table'
   */
  uint32_T uDLookupTable_maxIndex[2];
} ConstP_VehCtrlMdel240912_2018_T;

/* Real-time Model Data Structure */
struct tag_RTM_VehCtrlMdel240912_201_T {
  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick3;
    struct {
      uint16_T TID[7];
    } TaskCounters;
  } Timing;
};

/* Block signals (default storage) */
extern B_VehCtrlMdel240912_2018b_T VehCtrlMdel240912_2018b_B;

/* Block states (default storage) */
extern DW_VehCtrlMdel240912_2018b_T VehCtrlMdel240912_2018b_DW;

/* Constant parameters (default storage) */
extern const ConstP_VehCtrlMdel240912_2018_T VehCtrlMdel240912_2018b_ConstP;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real_T Gear_Trs;                /* '<S282>/Switch2' */
extern real_T Mode_Trs;                /* '<S282>/Switch3' */
extern real_T Trq_CUT;                 /* '<S151>/Timer' */
extern real_T L12V_error;              /* '<S127>/CAN Unpack' */
extern real_T alarm;                   /* '<S127>/CAN Unpack' */
extern real_T controller_ready;        /* '<S127>/CAN Unpack' */
extern real_T selfcheck;               /* '<S127>/CAN Unpack' */
extern real_T RPM;                     /* '<S127>/CAN Unpack' */
extern real_T trq;                     /* '<S127>/CAN Unpack' */
extern real_T AC_current;              /* '<S122>/CAN Unpack' */
extern real_T DC_current;              /* '<S122>/CAN Unpack' */
extern real_T MCU_Temp;                /* '<S122>/CAN Unpack' */
extern real_T motor_Temp;              /* '<S122>/CAN Unpack' */
extern real_T StrWhlAngAliveRollCnt;   /* '<S144>/CAN Unpack1' */
extern real_T StrWhlAng;               /* '<S144>/CAN Unpack1' */
extern real_T StrWhlAngV;              /* '<S144>/CAN Unpack1' */
extern real_T IMU_Ax_Valid;            /* '<S135>/CAN Unpack' */
extern real_T IMU_Ax_Value;            /* '<S135>/CAN Unpack' */
extern real_T IMU_Ay_Valid;            /* '<S135>/CAN Unpack' */
extern real_T IMU_Ay_Value;            /* '<S135>/CAN Unpack' */
extern real_T IMU_DTC;                 /* '<S135>/CAN Unpack' */
extern real_T IMU_Yaw_Valid;           /* '<S135>/CAN Unpack' */
extern real_T IMU_Yaw_Value;           /* '<S135>/CAN Unpack' */
extern uint32_T Acc_vol;               /* '<S151>/Add2' */
extern uint32_T Acc_vol2;              /* '<S151>/Add3' */
extern uint32_T Acc_POS1;              /* '<S151>/1-D Lookup Table' */
extern uint32_T Acc_POS2;              /* '<S151>/1-D Lookup Table3' */
extern uint16_T F_BrkPrs;              /* '<S151>/1-D Lookup Table1' */
extern boolean_T ignition;             /* '<S101>/Logical Operator' */
extern boolean_T beeper_state;         /* '<S90>/Chart2' */

/* External function called from main */
extern void VehCtrlMdel240912_2018b_SetEventsForThisBaseStep(boolean_T
  *eventFlags);

/* Model entry point functions */
extern void VehCtrlMdel240912_2018b_SetEventsForThisBaseStep(boolean_T
  *eventFlags);
extern void VehCtrlMdel240912_2018b_initialize(void);
extern void VehCtrlMdel240912_2018b_step(int_T tid);
extern uint8_T ECUCoderModelBaseCounter;
extern uint32_t IntcIsrVectorTable[];
extern uint8_T AfterRunFlags[2];
extern SSD_CONFIG ssdConfig;
extern void ISR_PIT_CH3(void);

/* Real-time Model object */
extern RT_MODEL_VehCtrlMdel240912_20_T *const VehCtrlMdel240912_2018b_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S29>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S44>/Data Type Duplicate' : Unused code path elimination
 * Block '<S44>/Data Type Propagation' : Unused code path elimination
 * Block '<S30>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S45>/Data Type Duplicate' : Unused code path elimination
 * Block '<S45>/Data Type Propagation' : Unused code path elimination
 * Block '<S31>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S46>/Data Type Duplicate' : Unused code path elimination
 * Block '<S46>/Data Type Propagation' : Unused code path elimination
 * Block '<S32>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S47>/Data Type Duplicate' : Unused code path elimination
 * Block '<S47>/Data Type Propagation' : Unused code path elimination
 * Block '<S33>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S48>/Data Type Duplicate' : Unused code path elimination
 * Block '<S48>/Data Type Propagation' : Unused code path elimination
 * Block '<S34>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S49>/Data Type Duplicate' : Unused code path elimination
 * Block '<S49>/Data Type Propagation' : Unused code path elimination
 * Block '<S35>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S50>/Data Type Duplicate' : Unused code path elimination
 * Block '<S50>/Data Type Propagation' : Unused code path elimination
 * Block '<S36>/Data Type Duplicate' : Unused code path elimination
 * Block '<S36>/Data Type Propagation' : Unused code path elimination
 * Block '<S37>/Data Type Duplicate' : Unused code path elimination
 * Block '<S37>/Data Type Propagation' : Unused code path elimination
 * Block '<S38>/Data Type Duplicate' : Unused code path elimination
 * Block '<S38>/Data Type Propagation' : Unused code path elimination
 * Block '<S39>/Data Type Duplicate' : Unused code path elimination
 * Block '<S39>/Data Type Propagation' : Unused code path elimination
 * Block '<S40>/Data Type Duplicate' : Unused code path elimination
 * Block '<S40>/Data Type Propagation' : Unused code path elimination
 * Block '<S41>/Data Type Duplicate' : Unused code path elimination
 * Block '<S41>/Data Type Propagation' : Unused code path elimination
 * Block '<S11>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S51>/Data Type Duplicate' : Unused code path elimination
 * Block '<S51>/Data Type Propagation' : Unused code path elimination
 * Block '<S12>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S52>/Data Type Duplicate' : Unused code path elimination
 * Block '<S52>/Data Type Propagation' : Unused code path elimination
 * Block '<S13>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S53>/Data Type Duplicate' : Unused code path elimination
 * Block '<S53>/Data Type Propagation' : Unused code path elimination
 * Block '<S14>/Data Type Duplicate' : Unused code path elimination
 * Block '<S14>/Data Type Propagation' : Unused code path elimination
 * Block '<S15>/Data Type Duplicate' : Unused code path elimination
 * Block '<S15>/Data Type Propagation' : Unused code path elimination
 * Block '<S16>/Data Type Duplicate' : Unused code path elimination
 * Block '<S16>/Data Type Propagation' : Unused code path elimination
 * Block '<S59>/Data Type Duplicate' : Unused code path elimination
 * Block '<S59>/Data Type Propagation' : Unused code path elimination
 * Block '<S60>/Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Propagation' : Unused code path elimination
 * Block '<S61>/Data Type Duplicate' : Unused code path elimination
 * Block '<S61>/Data Type Propagation' : Unused code path elimination
 * Block '<S68>/Data Type Duplicate' : Unused code path elimination
 * Block '<S68>/Data Type Propagation' : Unused code path elimination
 * Block '<S69>/Data Type Duplicate' : Unused code path elimination
 * Block '<S69>/Data Type Propagation' : Unused code path elimination
 * Block '<S70>/Data Type Duplicate' : Unused code path elimination
 * Block '<S70>/Data Type Propagation' : Unused code path elimination
 * Block '<S79>/Data Type Duplicate' : Unused code path elimination
 * Block '<S79>/Data Type Propagation' : Unused code path elimination
 * Block '<S80>/Data Type Duplicate' : Unused code path elimination
 * Block '<S80>/Data Type Propagation' : Unused code path elimination
 * Block '<S81>/Data Type Duplicate' : Unused code path elimination
 * Block '<S81>/Data Type Propagation' : Unused code path elimination
 * Block '<S110>/CAN Unpack' : Unused code path elimination
 * Block '<S116>/CAN Unpack' : Unused code path elimination
 * Block '<S119>/CAN Unpack1' : Unused code path elimination
 * Block '<S172>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S178>/Data Type Duplicate' : Unused code path elimination
 * Block '<S178>/Data Type Propagation' : Unused code path elimination
 * Block '<S173>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S179>/Data Type Duplicate' : Unused code path elimination
 * Block '<S179>/Data Type Propagation' : Unused code path elimination
 * Block '<S174>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S180>/Data Type Duplicate' : Unused code path elimination
 * Block '<S180>/Data Type Propagation' : Unused code path elimination
 * Block '<S175>/Data Type Duplicate' : Unused code path elimination
 * Block '<S175>/Data Type Propagation' : Unused code path elimination
 * Block '<S176>/Data Type Duplicate' : Unused code path elimination
 * Block '<S176>/Data Type Propagation' : Unused code path elimination
 * Block '<S177>/Data Type Duplicate' : Unused code path elimination
 * Block '<S177>/Data Type Propagation' : Unused code path elimination
 * Block '<S182>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S185>/Data Type Duplicate' : Unused code path elimination
 * Block '<S185>/Data Type Propagation' : Unused code path elimination
 * Block '<S183>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S186>/Data Type Duplicate' : Unused code path elimination
 * Block '<S186>/Data Type Propagation' : Unused code path elimination
 * Block '<S195>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S203>/Data Type Duplicate' : Unused code path elimination
 * Block '<S203>/Data Type Propagation' : Unused code path elimination
 * Block '<S196>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S204>/Data Type Duplicate' : Unused code path elimination
 * Block '<S204>/Data Type Propagation' : Unused code path elimination
 * Block '<S197>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S205>/Data Type Duplicate' : Unused code path elimination
 * Block '<S205>/Data Type Propagation' : Unused code path elimination
 * Block '<S198>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S206>/Data Type Duplicate' : Unused code path elimination
 * Block '<S206>/Data Type Propagation' : Unused code path elimination
 * Block '<S228>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S230>/Data Type Duplicate' : Unused code path elimination
 * Block '<S230>/Data Type Propagation' : Unused code path elimination
 * Block '<S229>/Data Type Duplicate' : Unused code path elimination
 * Block '<S229>/Data Type Propagation' : Unused code path elimination
 * Block '<S231>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S233>/Data Type Duplicate' : Unused code path elimination
 * Block '<S233>/Data Type Propagation' : Unused code path elimination
 * Block '<S232>/Data Type Duplicate' : Unused code path elimination
 * Block '<S232>/Data Type Propagation' : Unused code path elimination
 * Block '<S234>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S236>/Data Type Duplicate' : Unused code path elimination
 * Block '<S236>/Data Type Propagation' : Unused code path elimination
 * Block '<S235>/Data Type Duplicate' : Unused code path elimination
 * Block '<S235>/Data Type Propagation' : Unused code path elimination
 * Block '<S237>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S239>/Data Type Duplicate' : Unused code path elimination
 * Block '<S239>/Data Type Propagation' : Unused code path elimination
 * Block '<S238>/Data Type Duplicate' : Unused code path elimination
 * Block '<S238>/Data Type Propagation' : Unused code path elimination
 * Block '<S248>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S250>/Data Type Duplicate' : Unused code path elimination
 * Block '<S250>/Data Type Propagation' : Unused code path elimination
 * Block '<S249>/Data Type Duplicate' : Unused code path elimination
 * Block '<S249>/Data Type Propagation' : Unused code path elimination
 * Block '<S251>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S253>/Data Type Duplicate' : Unused code path elimination
 * Block '<S253>/Data Type Propagation' : Unused code path elimination
 * Block '<S252>/Data Type Duplicate' : Unused code path elimination
 * Block '<S252>/Data Type Propagation' : Unused code path elimination
 * Block '<S254>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S256>/Data Type Duplicate' : Unused code path elimination
 * Block '<S256>/Data Type Propagation' : Unused code path elimination
 * Block '<S255>/Data Type Duplicate' : Unused code path elimination
 * Block '<S255>/Data Type Propagation' : Unused code path elimination
 * Block '<S257>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S259>/Data Type Duplicate' : Unused code path elimination
 * Block '<S259>/Data Type Propagation' : Unused code path elimination
 * Block '<S258>/Data Type Duplicate' : Unused code path elimination
 * Block '<S258>/Data Type Propagation' : Unused code path elimination
 * Block '<S209>/Discrete-Time Integrator' : Unused code path elimination
 * Block '<S273>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S275>/Data Type Duplicate' : Unused code path elimination
 * Block '<S275>/Data Type Propagation' : Unused code path elimination
 * Block '<S274>/Data Type Duplicate' : Unused code path elimination
 * Block '<S274>/Data Type Propagation' : Unused code path elimination
 * Block '<S267>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S276>/Data Type Duplicate' : Unused code path elimination
 * Block '<S276>/Data Type Propagation' : Unused code path elimination
 * Block '<S268>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S277>/Data Type Duplicate' : Unused code path elimination
 * Block '<S277>/Data Type Propagation' : Unused code path elimination
 * Block '<S269>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S278>/Data Type Duplicate' : Unused code path elimination
 * Block '<S278>/Data Type Propagation' : Unused code path elimination
 * Block '<S6>/Constant' : Unused code path elimination
 * Block '<S7>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S7>/Data Type Conversion10' : Eliminate redundant data type conversion
 * Block '<S7>/Data Type Conversion11' : Eliminate redundant data type conversion
 * Block '<S7>/Data Type Conversion12' : Eliminate redundant data type conversion
 * Block '<S7>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S7>/Data Type Conversion6' : Eliminate redundant data type conversion
 * Block '<S7>/Data Type Conversion7' : Eliminate redundant data type conversion
 * Block '<S7>/Data Type Conversion8' : Eliminate redundant data type conversion
 * Block '<S7>/Data Type Conversion9' : Eliminate redundant data type conversion
 * Block '<S8>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S8>/Cast To Boolean1' : Eliminate redundant data type conversion
 * Block '<S8>/Cast To Boolean2' : Eliminate redundant data type conversion
 * Block '<S90>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S207>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S207>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S208>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S208>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S208>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S208>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S266>/Gain1' : Eliminated nontunable gain of 1
 * Block '<S282>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S283>/Cast To Single' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'VehCtrlMdel240912_2018b'
 * '<S1>'   : 'VehCtrlMdel240912_2018b/CTRL'
 * '<S2>'   : 'VehCtrlMdel240912_2018b/Drive_Ready'
 * '<S3>'   : 'VehCtrlMdel240912_2018b/Input'
 * '<S4>'   : 'VehCtrlMdel240912_2018b/Input_Processing'
 * '<S5>'   : 'VehCtrlMdel240912_2018b/OUTPUT'
 * '<S6>'   : 'VehCtrlMdel240912_2018b/RapidECUSetting'
 * '<S7>'   : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq'
 * '<S8>'   : 'VehCtrlMdel240912_2018b/CTRL/PwrTrainTempPrtct'
 * '<S9>'   : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/Chart'
 * '<S10>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC'
 * '<S11>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic1'
 * '<S12>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic2'
 * '<S13>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic3'
 * '<S14>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/Saturation Dynamic1'
 * '<S15>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/Saturation Dynamic2'
 * '<S16>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/Saturation Dynamic3'
 * '<S17>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/Subsystem'
 * '<S18>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL'
 * '<S19>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR'
 * '<S20>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R'
 * '<S21>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Compare To Constant'
 * '<S22>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Compare To Constant1'
 * '<S23>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Compare To Constant2'
 * '<S24>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Compare To Constant3'
 * '<S25>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Compare To Constant4'
 * '<S26>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Compare To Constant5'
 * '<S27>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/MATLAB Function'
 * '<S28>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/MATLAB Function1'
 * '<S29>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic'
 * '<S30>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic1'
 * '<S31>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic2'
 * '<S32>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic3'
 * '<S33>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic4'
 * '<S34>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic5'
 * '<S35>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic6'
 * '<S36>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic'
 * '<S37>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic1'
 * '<S38>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic2'
 * '<S39>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic3'
 * '<S40>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic4'
 * '<S41>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic5'
 * '<S42>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Wtarget'
 * '<S43>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/载荷转移'
 * '<S44>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S45>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S46>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S47>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S48>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic4/Saturation Dynamic'
 * '<S49>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic5/Saturation Dynamic'
 * '<S50>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic6/Saturation Dynamic'
 * '<S51>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S52>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S53>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S54>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Compare To Constant'
 * '<S55>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Compare To Constant2'
 * '<S56>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Detect Rise Positive'
 * '<S57>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Latch_on'
 * '<S58>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/RisingTimer'
 * '<S59>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic'
 * '<S60>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic1'
 * '<S61>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic2'
 * '<S62>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Detect Rise Positive/Positive'
 * '<S63>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Compare To Constant'
 * '<S64>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Compare To Constant2'
 * '<S65>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Detect Rise Positive'
 * '<S66>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Latch_on'
 * '<S67>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/RisingTimer'
 * '<S68>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic'
 * '<S69>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic1'
 * '<S70>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic2'
 * '<S71>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Detect Rise Positive/Positive'
 * '<S72>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant'
 * '<S73>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant1'
 * '<S74>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant2'
 * '<S75>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Zero'
 * '<S76>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Detect Rise Positive'
 * '<S77>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Latch_on'
 * '<S78>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/RisingTimer'
 * '<S79>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic'
 * '<S80>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic1'
 * '<S81>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic2'
 * '<S82>'  : 'VehCtrlMdel240912_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Detect Rise Positive/Positive'
 * '<S83>'  : 'VehCtrlMdel240912_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant'
 * '<S84>'  : 'VehCtrlMdel240912_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant1'
 * '<S85>'  : 'VehCtrlMdel240912_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant2'
 * '<S86>'  : 'VehCtrlMdel240912_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant3'
 * '<S87>'  : 'VehCtrlMdel240912_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant4'
 * '<S88>'  : 'VehCtrlMdel240912_2018b/CTRL/PwrTrainTempPrtct/Timer1'
 * '<S89>'  : 'VehCtrlMdel240912_2018b/CTRL/PwrTrainTempPrtct/Timer2'
 * '<S90>'  : 'VehCtrlMdel240912_2018b/Drive_Ready/Subsystem'
 * '<S91>'  : 'VehCtrlMdel240912_2018b/Drive_Ready/Subsystem/Chart2'
 * '<S92>'  : 'VehCtrlMdel240912_2018b/Drive_Ready/Subsystem/Compare To Constant'
 * '<S93>'  : 'VehCtrlMdel240912_2018b/Drive_Ready/Subsystem/Compare To Constant1'
 * '<S94>'  : 'VehCtrlMdel240912_2018b/Input/ABS_Receive'
 * '<S95>'  : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive'
 * '<S96>'  : 'VehCtrlMdel240912_2018b/Input/AccBrk_BUS'
 * '<S97>'  : 'VehCtrlMdel240912_2018b/Input/BMS_Recive'
 * '<S98>'  : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE'
 * '<S99>'  : 'VehCtrlMdel240912_2018b/Input/IMU_Recieve'
 * '<S100>' : 'VehCtrlMdel240912_2018b/Input/StrSnis_Receive'
 * '<S101>' : 'VehCtrlMdel240912_2018b/Input/key'
 * '<S102>' : 'VehCtrlMdel240912_2018b/Input/ABS_Receive/ABS_BUS_state'
 * '<S103>' : 'VehCtrlMdel240912_2018b/Input/ABS_Receive/ABS_BUS_state/IMU_state'
 * '<S104>' : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive/FLAMKMCU'
 * '<S105>' : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive/FRAMKMCU'
 * '<S106>' : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state'
 * '<S107>' : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1'
 * '<S108>' : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2'
 * '<S109>' : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state'
 * '<S110>' : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state'
 * '<S111>' : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state'
 * '<S112>' : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state'
 * '<S113>' : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1'
 * '<S114>' : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2'
 * '<S115>' : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state'
 * '<S116>' : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state'
 * '<S117>' : 'VehCtrlMdel240912_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state'
 * '<S118>' : 'VehCtrlMdel240912_2018b/Input/BMS_Recive/ABS_BUS_state'
 * '<S119>' : 'VehCtrlMdel240912_2018b/Input/BMS_Recive/ABS_BUS_state/IMU_state'
 * '<S120>' : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr'
 * '<S121>' : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE/MCU_state'
 * '<S122>' : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1'
 * '<S123>' : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule1'
 * '<S124>' : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule2'
 * '<S125>' : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule3'
 * '<S126>' : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule4'
 * '<S127>' : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state'
 * '<S128>' : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule'
 * '<S129>' : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule1'
 * '<S130>' : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule2'
 * '<S131>' : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule3'
 * '<S132>' : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule5'
 * '<S133>' : 'VehCtrlMdel240912_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule6'
 * '<S134>' : 'VehCtrlMdel240912_2018b/Input/IMU_Recieve/IMU_state'
 * '<S135>' : 'VehCtrlMdel240912_2018b/Input/IMU_Recieve/IMU_state/MCU_state'
 * '<S136>' : 'VehCtrlMdel240912_2018b/Input/IMU_Recieve/IMU_state/MeaModule1'
 * '<S137>' : 'VehCtrlMdel240912_2018b/Input/IMU_Recieve/IMU_state/MeaModule2'
 * '<S138>' : 'VehCtrlMdel240912_2018b/Input/IMU_Recieve/IMU_state/MeaModule3'
 * '<S139>' : 'VehCtrlMdel240912_2018b/Input/IMU_Recieve/IMU_state/MeaModule4'
 * '<S140>' : 'VehCtrlMdel240912_2018b/Input/IMU_Recieve/IMU_state/MeaModule5'
 * '<S141>' : 'VehCtrlMdel240912_2018b/Input/IMU_Recieve/IMU_state/MeaModule6'
 * '<S142>' : 'VehCtrlMdel240912_2018b/Input/IMU_Recieve/IMU_state/MeaModule7'
 * '<S143>' : 'VehCtrlMdel240912_2018b/Input/StrSnis_Receive/StrWhSnis_state'
 * '<S144>' : 'VehCtrlMdel240912_2018b/Input/StrSnis_Receive/StrWhSnis_state/IMU_state'
 * '<S145>' : 'VehCtrlMdel240912_2018b/Input/StrSnis_Receive/StrWhSnis_state/MeaModule1'
 * '<S146>' : 'VehCtrlMdel240912_2018b/Input/StrSnis_Receive/StrWhSnis_state/MeaModule2'
 * '<S147>' : 'VehCtrlMdel240912_2018b/Input/StrSnis_Receive/StrWhSnis_state/MeaModule6'
 * '<S148>' : 'VehCtrlMdel240912_2018b/Input/key/MeaModule'
 * '<S149>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem'
 * '<S150>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem'
 * '<S151>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal'
 * '<S152>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/IMU'
 * '<S153>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/SWAS'
 * '<S154>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps'
 * '<S155>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant'
 * '<S156>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant1'
 * '<S157>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant2'
 * '<S158>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant3'
 * '<S159>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant4'
 * '<S160>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant5'
 * '<S161>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant6'
 * '<S162>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant7'
 * '<S163>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant8'
 * '<S164>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule1'
 * '<S165>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule2'
 * '<S166>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule3'
 * '<S167>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule5'
 * '<S168>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule7'
 * '<S169>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule8'
 * '<S170>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Timer'
 * '<S171>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem'
 * '<S172>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic'
 * '<S173>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1'
 * '<S174>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2'
 * '<S175>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic'
 * '<S176>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic1'
 * '<S177>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic2'
 * '<S178>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S179>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S180>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S181>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/SWAS/Compare To Constant'
 * '<S182>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic'
 * '<S183>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1'
 * '<S184>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/SWAS/Timer'
 * '<S185>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S186>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S187>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant'
 * '<S188>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant1'
 * '<S189>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant2'
 * '<S190>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant3'
 * '<S191>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant4'
 * '<S192>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant5'
 * '<S193>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant6'
 * '<S194>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant7'
 * '<S195>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic'
 * '<S196>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1'
 * '<S197>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2'
 * '<S198>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3'
 * '<S199>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer'
 * '<S200>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer1'
 * '<S201>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer2'
 * '<S202>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer3'
 * '<S203>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S204>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S205>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S206>' : 'VehCtrlMdel240912_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S207>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/IntglJudgment '
 * '<S208>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment'
 * '<S209>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect'
 * '<S210>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/坐标系转换'
 * '<S211>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/IntglJudgment /Compare To Constant'
 * '<S212>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/IntglJudgment /Compare To Constant1'
 * '<S213>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/Timer'
 * '<S214>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/Timer1'
 * '<S215>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/Timer2'
 * '<S216>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/Timer3'
 * '<S217>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断'
 * '<S218>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断'
 * '<S219>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断'
 * '<S220>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant'
 * '<S221>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant1'
 * '<S222>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant2'
 * '<S223>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant3'
 * '<S224>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter'
 * '<S225>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1'
 * '<S226>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2'
 * '<S227>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3'
 * '<S228>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic'
 * '<S229>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Saturation Dynamic'
 * '<S230>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S231>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic'
 * '<S232>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Saturation Dynamic'
 * '<S233>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S234>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic'
 * '<S235>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Saturation Dynamic'
 * '<S236>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S237>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic'
 * '<S238>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Saturation Dynamic'
 * '<S239>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S240>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant'
 * '<S241>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant1'
 * '<S242>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant2'
 * '<S243>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant3'
 * '<S244>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter'
 * '<S245>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1'
 * '<S246>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2'
 * '<S247>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3'
 * '<S248>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic'
 * '<S249>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Saturation Dynamic'
 * '<S250>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S251>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic'
 * '<S252>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Saturation Dynamic'
 * '<S253>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S254>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic'
 * '<S255>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Saturation Dynamic'
 * '<S256>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S257>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic'
 * '<S258>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Saturation Dynamic'
 * '<S259>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S260>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant'
 * '<S261>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant1'
 * '<S262>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant2'
 * '<S263>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant3'
 * '<S264>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant'
 * '<S265>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant1'
 * '<S266>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Filter'
 * '<S267>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic'
 * '<S268>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1'
 * '<S269>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2'
 * '<S270>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Subsystem'
 * '<S271>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Timer1'
 * '<S272>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Timer2'
 * '<S273>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic'
 * '<S274>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Filter/Saturation Dynamic'
 * '<S275>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S276>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S277>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S278>' : 'VehCtrlMdel240912_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S279>' : 'VehCtrlMdel240912_2018b/OUTPUT/Beeper'
 * '<S280>' : 'VehCtrlMdel240912_2018b/OUTPUT/VCU2AMKMCUFL'
 * '<S281>' : 'VehCtrlMdel240912_2018b/OUTPUT/VCU2AMKMCUFR'
 * '<S282>' : 'VehCtrlMdel240912_2018b/OUTPUT/VCU2EmraxMCU'
 * '<S283>' : 'VehCtrlMdel240912_2018b/OUTPUT/WP_OUTPUT'
 * '<S284>' : 'VehCtrlMdel240912_2018b/OUTPUT/Beeper/MeaModule'
 * '<S285>' : 'VehCtrlMdel240912_2018b/OUTPUT/VCU2EmraxMCU/MeaModule1'
 * '<S286>' : 'VehCtrlMdel240912_2018b/OUTPUT/VCU2EmraxMCU/MeaModule2'
 * '<S287>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/BL'
 * '<S288>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/DAQ'
 * '<S289>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/EEPROM'
 * '<S290>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/Polling'
 * '<S291>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/BL/Function-Call Subsystem'
 * '<S292>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem'
 * '<S293>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem'
 * '<S294>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com0'
 * '<S295>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com1'
 * '<S296>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com2'
 * '<S297>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem'
 * '<S298>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem1'
 * '<S299>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/DAQ/daq100ms'
 * '<S300>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/DAQ/daq10ms'
 * '<S301>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/DAQ/daq500ms'
 * '<S302>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/DAQ/daq50ms'
 * '<S303>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/DAQ/daq5ms'
 * '<S304>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/EEPROM/EEPROMOperation'
 * '<S305>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/Polling/CCPBackground'
 * '<S306>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/Polling/CCPReceive'
 * '<S307>' : 'VehCtrlMdel240912_2018b/RapidECUSetting/Polling/CCPReceive/Nothing'
 */
#endif                               /* RTW_HEADER_VehCtrlMdel240912_2018b_h_ */

/* File trailer for ECUCoder generated file VehCtrlMdel240912_2018b.h.
 *
 * [EOF]
 */
