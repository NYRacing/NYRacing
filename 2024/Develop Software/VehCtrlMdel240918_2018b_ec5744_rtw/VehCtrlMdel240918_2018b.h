/*
 * Code generated for Simulink model VehCtrlMdel240918_2018b.
 *
 * FILE    : VehCtrlMdel240918_2018b.h
 *
 * VERSION : 1.163
 *
 * DATE    : Tue Sep 24 23:15:25 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#ifndef RTW_HEADER_VehCtrlMdel240918_2018b_h_
#define RTW_HEADER_VehCtrlMdel240918_2018b_h_
#include <math.h>
#include "MPC5744P.h"
#include "Std_Types.h"
#include "can.h"
#include "flash.h"
#include "crc.h"
#ifndef VehCtrlMdel240918_2018b_COMMON_INCLUDES_
# define VehCtrlMdel240918_2018b_COMMON_INCLUDES_
#include <string.h>
#include <math.h>
#include "rtwtypes.h"
#include "can_message.h"
#endif                            /* VehCtrlMdel240918_2018b_COMMON_INCLUDES_ */

#include "VehCtrlMdel240918_2018b_types.h"
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
    uint_T is_c5_VehCtrlMdel240918_2018b:2;/* '<S8>/Timer1' */
    uint_T is_active_c5_VehCtrlMdel240918_:1;/* '<S8>/Timer1' */
  } bitsForTID3;
} DW_Timer1_VehCtrlMdel240918_2_T;

/* Block states (default storage) for system '<S105>/Timer' */
typedef struct {
  real_T x;                            /* '<S105>/Timer' */
  struct {
    uint_T is_c21_VehCtrlMdel240918_2018b:2;/* '<S105>/Timer' */
    uint_T is_active_c21_VehCtrlMdel240918:1;/* '<S105>/Timer' */
  } bitsForTID3;
} DW_Timer_VehCtrlMdel240918_20_T;

/* Block signals (default storage) */
typedef struct {
  CAN_DATATYPE CANPack1;               /* '<S318>/CAN Pack1' */
  CAN_DATATYPE CANPack1_a;             /* '<S319>/CAN Pack1' */
  CAN_DATATYPE CANPack1_d;             /* '<S317>/CAN Pack1' */
  CAN_DATATYPE CANUnPackMessage4;      /* '<S162>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_g;    /* '<S157>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_j;    /* '<S135>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_n;    /* '<S145>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_p;    /* '<S143>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_e;    /* '<S117>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_c;    /* '<S128>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_b;    /* '<S126>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_l;    /* '<S175>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_ja;   /* '<S107>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_h;    /* '<S170>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_i;    /* '<S154>/CANUnPackMessage4' */
  real_T Exit;                         /* '<S246>/Timer2' */
  real_T Exit_l;                       /* '<S246>/Timer1' */
  real_T Exit_a;                       /* '<S245>/Timer3' */
  real_T Exit_lh;                      /* '<S245>/Timer2' */
  real_T Exit_lh4;                     /* '<S245>/Timer1' */
  real_T Exit_c;                       /* '<S245>/Timer' */
  real_T Exit_h;                       /* '<S187>/Timer3' */
  real_T Exit_o;                       /* '<S187>/Timer2' */
  real_T Exit_i;                       /* '<S187>/Timer1' */
  real_T Exit_le;                      /* '<S187>/Timer' */
  real_T Exit_on;                      /* '<S186>/Timer' */
  real_T low_VOL;                      /* '<S162>/CAN Unpack' */
  real_T MCU_Temp_error;               /* '<S162>/CAN Unpack' */
  real_T Mode;                         /* '<S162>/CAN Unpack' */
  real_T motorTemp_error;              /* '<S162>/CAN Unpack' */
  real_T overCurrent;                  /* '<S162>/CAN Unpack' */
  real_T overpower;                    /* '<S162>/CAN Unpack' */
  real_T overvol;                      /* '<S162>/CAN Unpack' */
  real_T Precharge;                    /* '<S162>/CAN Unpack' */
  real_T Reslove_error;                /* '<S162>/CAN Unpack' */
  real_T voltage;                      /* '<S157>/CAN Unpack' */
  real_T MCFR_ActualTorque;            /* '<S135>/CAN Unpack' */
  real_T MCFR_bDerating;               /* '<S135>/CAN Unpack' */
  real_T MCFR_bQuitDCOn;               /* '<S135>/CAN Unpack' */
  real_T MCFR_bReserve;                /* '<S135>/CAN Unpack' */
  real_T MCFR_bWarn;                   /* '<S135>/CAN Unpack' */
  real_T MCFR_DiagnosticNum;           /* '<S143>/CAN Unpack' */
  real_T MCFL_ActualTorque;            /* '<S117>/CAN Unpack' */
  real_T MCFL_ActualVelocity_p;        /* '<S117>/CAN Unpack' */
  real_T MCFL_bDerating;               /* '<S117>/CAN Unpack' */
  real_T MCFL_bReserve;                /* '<S117>/CAN Unpack' */
  real_T MCFL_bWarn;                   /* '<S117>/CAN Unpack' */
  real_T MCFL_DiagnosticNum;           /* '<S126>/CAN Unpack' */
  real_T CANUnpack_o1;                 /* '<S170>/CAN Unpack' */
  real_T CANUnpack_o3;                 /* '<S170>/CAN Unpack' */
  real_T CANUnpack_o5;                 /* '<S170>/CAN Unpack' */
  real_T CANUnpack_o6;                 /* '<S170>/CAN Unpack' */
  real_T errorReset;                   /* '<S91>/Chart2' */
  real_T Exit_d;                       /* '<S8>/Timer2' */
  real_T Exit_g;                       /* '<S8>/Timer1' */
  real_T DYC_Enable_OUT;               /* '<S7>/Chart' */
  real_T TCSR_Enable_OUT;              /* '<S7>/Chart' */
  real_T TCSF_Enable_OUT;              /* '<S7>/Chart' */
  uint32_T CANReceive_o3;              /* '<S343>/CANReceive' */
  uint32_T CANReceive_o3_l;            /* '<S328>/CANReceive' */
  uint32_T CANReceive1_o3;             /* '<S102>/CANReceive1' */
  uint32_T CANReceive3_o3;             /* '<S102>/CANReceive3' */
  uint32_T CANReceive3_o3_e;           /* '<S112>/CANReceive3' */
  uint32_T CANReceive1_o3_n;           /* '<S112>/CANReceive1' */
  uint32_T CANReceive2_o3;             /* '<S112>/CANReceive2' */
  uint32_T CANReceive3_o3_i;           /* '<S113>/CANReceive3' */
  uint32_T CANReceive1_o3_h;           /* '<S113>/CANReceive1' */
  uint32_T CANReceive2_o3_j;           /* '<S113>/CANReceive2' */
  uint32_T CANReceive3_o3_c;           /* '<S104>/CANReceive3' */
  uint32_T CANReceive3_o3_m;           /* '<S98>/CANReceive3' */
  uint32_T CANReceive3_o3_cz;          /* '<S103>/CANReceive3' */
  uint32_T CANReceive3_o3_l;           /* '<S101>/CANReceive3' */
  real32_T TrqR_cmd;                   /* '<S7>/Saturation1' */
  real32_T TrqFR_cmd;                  /* '<S7>/Saturation2' */
  real32_T TrqFL_cmd;                  /* '<S7>/Saturation3' */
  int32_T DataTypeConversion2;         /* '<S319>/Data Type Conversion2' */
  uint16_T CastToSingle1;              /* '<S320>/Cast To Single1' */
  uint16_T CastToBoolean4;             /* '<S319>/Cast To Boolean4' */
  uint16_T CastToBoolean6;             /* '<S319>/Cast To Boolean6' */
  uint8_T CANReceive_o2;               /* '<S343>/CANReceive' */
  uint8_T CANReceive_o4[8];            /* '<S343>/CANReceive' */
  uint8_T CANReceive_o5;               /* '<S343>/CANReceive' */
  uint8_T CANReceive_o2_p;             /* '<S328>/CANReceive' */
  uint8_T CANReceive_o4_i[8];          /* '<S328>/CANReceive' */
  uint8_T CANReceive_o5_l;             /* '<S328>/CANReceive' */
  uint8_T CANTransmit;                 /* '<S335>/CANTransmit' */
  uint8_T CANPackMessage[8];           /* '<S318>/CANPackMessage' */
  uint8_T CANTransmit_l;               /* '<S318>/CANTransmit' */
  uint8_T CANPackMessage_d[8];         /* '<S319>/CANPackMessage' */
  uint8_T CANTransmit_k;               /* '<S319>/CANTransmit' */
  uint8_T CANPackMessage_h[8];         /* '<S317>/CANPackMessage' */
  uint8_T CANTransmit_c;               /* '<S317>/CANTransmit' */
  uint8_T CANReceive1_o2;              /* '<S102>/CANReceive1' */
  uint8_T CANReceive1_o4[8];           /* '<S102>/CANReceive1' */
  uint8_T CANReceive1_o5;              /* '<S102>/CANReceive1' */
  uint8_T CANReceive3_o2;              /* '<S102>/CANReceive3' */
  uint8_T CANReceive3_o4[8];           /* '<S102>/CANReceive3' */
  uint8_T CANReceive3_o5;              /* '<S102>/CANReceive3' */
  uint8_T CANReceive3_o2_l;            /* '<S112>/CANReceive3' */
  uint8_T CANReceive3_o4_l[8];         /* '<S112>/CANReceive3' */
  uint8_T CANReceive3_o5_a;            /* '<S112>/CANReceive3' */
  uint8_T CANReceive1_o2_l;            /* '<S112>/CANReceive1' */
  uint8_T CANReceive1_o4_c[8];         /* '<S112>/CANReceive1' */
  uint8_T CANReceive1_o5_a;            /* '<S112>/CANReceive1' */
  uint8_T CANReceive2_o2;              /* '<S112>/CANReceive2' */
  uint8_T CANReceive2_o4[6];           /* '<S112>/CANReceive2' */
  uint8_T CANReceive2_o5;              /* '<S112>/CANReceive2' */
  uint8_T CANReceive3_o2_a;            /* '<S113>/CANReceive3' */
  uint8_T CANReceive3_o4_g[8];         /* '<S113>/CANReceive3' */
  uint8_T CANReceive3_o5_an;           /* '<S113>/CANReceive3' */
  uint8_T CANReceive1_o2_o;            /* '<S113>/CANReceive1' */
  uint8_T CANReceive1_o4_j[8];         /* '<S113>/CANReceive1' */
  uint8_T CANReceive1_o5_j;            /* '<S113>/CANReceive1' */
  uint8_T CANReceive2_o2_p;            /* '<S113>/CANReceive2' */
  uint8_T CANReceive2_o4_k[6];         /* '<S113>/CANReceive2' */
  uint8_T CANReceive2_o5_e;            /* '<S113>/CANReceive2' */
  uint8_T CANReceive3_o2_p;            /* '<S104>/CANReceive3' */
  uint8_T CANReceive3_o4_k[8];         /* '<S104>/CANReceive3' */
  uint8_T CANReceive3_o5_d;            /* '<S104>/CANReceive3' */
  uint8_T CANReceive3_o2_m;            /* '<S98>/CANReceive3' */
  uint8_T CANReceive3_o4_lg[8];        /* '<S98>/CANReceive3' */
  uint8_T CANReceive3_o5_b;            /* '<S98>/CANReceive3' */
  uint8_T CANReceive3_o2_ma;           /* '<S103>/CANReceive3' */
  uint8_T CANReceive3_o4_i[8];         /* '<S103>/CANReceive3' */
  uint8_T CANReceive3_o5_m;            /* '<S103>/CANReceive3' */
  uint8_T CANReceive3_o2_k;            /* '<S101>/CANReceive3' */
  uint8_T CANReceive3_o4_p[8];         /* '<S101>/CANReceive3' */
  uint8_T CANReceive3_o5_de;           /* '<S101>/CANReceive3' */
  boolean_T Drive_ready;               /* '<S105>/SwitchInput' */
  boolean_T VehReady;                  /* '<S91>/Chart2' */
  boolean_T MCFL_DCOn_setpoints;       /* '<S91>/Chart2' */
  boolean_T MCFL_InverterOn;           /* '<S91>/Chart2' */
  boolean_T MCFL_DCEnable;             /* '<S91>/Chart2' */
  boolean_T MCFR_TorqueOn;             /* '<S91>/Chart2' */
  boolean_T MCFL_TorqueOn;             /* '<S91>/Chart2' */
  boolean_T aWaterPumpON;
} B_VehCtrlMdel240918_2018b_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DelayInput2_DSTATE;           /* '<S219>/Delay Input2' */
  real_T DelayInput2_DSTATE_a;         /* '<S232>/Delay Input2' */
  real_T DelayInput2_DSTATE_b;         /* '<S233>/Delay Input2' */
  real_T DelayInput2_DSTATE_h;         /* '<S234>/Delay Input2' */
  real_T DelayInput2_DSTATE_n;         /* '<S235>/Delay Input2' */
  real_T UnitDelay_DSTATE;             /* '<S186>/Unit Delay' */
  real_T DelayInput2_DSTATE_l;         /* '<S209>/Delay Input2' */
  real_T UnitDelay_DSTATE_g;           /* '<S208>/Unit Delay' */
  real_T DelayInput2_DSTATE_m;         /* '<S210>/Delay Input2' */
  real_T UnitDelay1_DSTATE;            /* '<S208>/Unit Delay1' */
  real_T DelayInput2_DSTATE_j;         /* '<S220>/Delay Input2' */
  real_T UnitDelay1_DSTATE_a;          /* '<S186>/Unit Delay1' */
  real_T DelayInput2_DSTATE_k;         /* '<S211>/Delay Input2' */
  real_T UnitDelay2_DSTATE;            /* '<S208>/Unit Delay2' */
  real_T DelayInput2_DSTATE_jk;        /* '<S33>/Delay Input2' */
  real_T UnitDelay_DSTATE_n;           /* '<S7>/Unit Delay' */
  real_T UnitDelay2_DSTATE_d;          /* '<S10>/Unit Delay2' */
  real_T UnitDelay_DSTATE_m;           /* '<S10>/Unit Delay' */
  real_T UnitDelay1_DSTATE_p;          /* '<S10>/Unit Delay1' */
  real_T DelayInput2_DSTATE_n0;        /* '<S35>/Delay Input2' */
  real_T DelayInput2_DSTATE_hk;        /* '<S32>/Delay Input2' */
  real_T UnitDelay4_DSTATE;            /* '<S21>/Unit Delay4' */
  real_T UnitDelay2_DSTATE_g;          /* '<S21>/Unit Delay2' */
  real_T UnitDelay4_DSTATE_i;          /* '<S20>/Unit Delay4' */
  real_T UnitDelay4_DSTATE_l;          /* '<S19>/Unit Delay4' */
  real_T DelayInput2_DSTATE_mb;        /* '<S30>/Delay Input2' */
  real_T UnitDelay2_DSTATE_j;          /* '<S20>/Unit Delay2' */
  real_T DelayInput2_DSTATE_o;         /* '<S31>/Delay Input2' */
  real_T UnitDelay2_DSTATE_b;          /* '<S19>/Unit Delay2' */
  real_T a;                            /* '<S7>/Chart' */
  real_T b;                            /* '<S7>/Chart' */
  real_T DYC_flag;                     /* '<S7>/Chart' */
  real32_T UnitDelay_DSTATE_p;         /* '<S261>/Unit Delay' */
  real32_T DelayInput2_DSTATE_n2;      /* '<S265>/Delay Input2' */
  real32_T UnitDelay_DSTATE_j;         /* '<S254>/Unit Delay' */
  real32_T UnitDelay_DSTATE_pj;        /* '<S262>/Unit Delay' */
  real32_T DelayInput2_DSTATE_e;       /* '<S268>/Delay Input2' */
  real32_T UnitDelay1_DSTATE_n;        /* '<S254>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_a;         /* '<S263>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hk3;     /* '<S271>/Delay Input2' */
  real32_T UnitDelay2_DSTATE_l;        /* '<S254>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_nc;        /* '<S264>/Unit Delay' */
  real32_T DelayInput2_DSTATE_c;       /* '<S274>/Delay Input2' */
  real32_T UnitDelay3_DSTATE;          /* '<S254>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_l;         /* '<S281>/Unit Delay' */
  real32_T DelayInput2_DSTATE_i;       /* '<S285>/Delay Input2' */
  real32_T UnitDelay4_DSTATE_ik;       /* '<S255>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_a0;        /* '<S255>/Unit Delay' */
  real32_T UnitDelay_DSTATE_ap;        /* '<S282>/Unit Delay' */
  real32_T DelayInput2_DSTATE_el;      /* '<S288>/Delay Input2' */
  real32_T UnitDelay5_DSTATE;          /* '<S255>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_am;       /* '<S255>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_o;         /* '<S283>/Unit Delay' */
  real32_T DelayInput2_DSTATE_p;       /* '<S291>/Delay Input2' */
  real32_T UnitDelay6_DSTATE;          /* '<S255>/Unit Delay6' */
  real32_T UnitDelay2_DSTATE_c;        /* '<S255>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_ah;        /* '<S284>/Unit Delay' */
  real32_T DelayInput2_DSTATE_mt;      /* '<S294>/Delay Input2' */
  real32_T UnitDelay7_DSTATE;          /* '<S255>/Unit Delay7' */
  real32_T UnitDelay3_DSTATE_d;        /* '<S255>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_lh;        /* '<S183>/Unit Delay' */
  real32_T UnitDelay4_DSTATE_m;        /* '<S244>/Unit Delay4' */
  real32_T UnitDelay1_DSTATE_k;        /* '<S183>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_o;        /* '<S244>/Unit Delay1' */
  real32_T UnitDelay3_DSTATE_p;        /* '<S246>/Unit Delay3' */
  real32_T UnitDelay1_DSTATE_d;        /* '<S246>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_f;        /* '<S246>/Unit Delay2' */
  real32_T UnitDelay4_DSTATE_mn;       /* '<S246>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_d;         /* '<S244>/Unit Delay' */
  real32_T UnitDelay2_DSTATE_i;        /* '<S244>/Unit Delay2' */
  real32_T DelayInput2_DSTATE_g;       /* '<S306>/Delay Input2' */
  real32_T DelayInput2_DSTATE_af;      /* '<S304>/Delay Input2' */
  real32_T DelayInput2_DSTATE_f;       /* '<S310>/Delay Input2' */
  real32_T UnitDelay_DSTATE_ncs;       /* '<S303>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hu;      /* '<S305>/Delay Input2' */
  real32_T DelayInput2_DSTATE_d;       /* '<S36>/Delay Input2' */
  real32_T UnitDelay4_DSTATE_n;        /* '<S10>/Unit Delay4' */
  real32_T UnitDelay5_DSTATE_b;        /* '<S10>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_i;        /* '<S79>/Unit Delay1' */
  real32_T UnitDelay5_DSTATE_l;        /* '<S21>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_b;         /* '<S21>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_g;        /* '<S21>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_n5;       /* '<S68>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_h;        /* '<S59>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_cd;      /* '<S12>/Delay Input2' */
  real32_T DelayInput2_DSTATE_fo;      /* '<S34>/Delay Input2' */
  real32_T UnitDelay5_DSTATE_i;        /* '<S20>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_f;         /* '<S20>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_f;        /* '<S20>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_hn;      /* '<S13>/Delay Input2' */
  real32_T UnitDelay5_DSTATE_ip;       /* '<S19>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_nr;        /* '<S19>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_g4;       /* '<S19>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_ib;      /* '<S14>/Delay Input2' */
  int32_T sfEvent;                     /* '<S91>/Chart2' */
  uint32_T Subsystem_PREV_T;           /* '<S4>/Subsystem' */
  uint32_T FunctionCallSubsystem_PREV_T;/* '<S4>/Function-Call Subsystem' */
  uint32_T previousTicks;              /* '<S91>/Chart2' */
  uint32_T MoTrqReq_PREV_T;            /* '<S1>/MoTrqReq' */
  int_T CANPack1_ModeSignalID;         /* '<S318>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_l;       /* '<S319>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_f;       /* '<S317>/CAN Pack1' */
  int_T CANUnpack_ModeSignalID;        /* '<S162>/CAN Unpack' */
  int_T CANUnpack_StatusPortID;        /* '<S162>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_b;      /* '<S157>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_m;      /* '<S157>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_a;      /* '<S135>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_o;      /* '<S135>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_n;      /* '<S145>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_f;      /* '<S145>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_p;      /* '<S143>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_e;      /* '<S143>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_c;      /* '<S117>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_ok;     /* '<S117>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_m;      /* '<S128>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_j;      /* '<S128>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_i;      /* '<S126>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_mj;     /* '<S126>/CAN Unpack' */
  int_T CANUnpack1_ModeSignalID;       /* '<S175>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID;       /* '<S175>/CAN Unpack1' */
  int_T CANUnpack1_ModeSignalID_m;     /* '<S107>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID_n;     /* '<S107>/CAN Unpack1' */
  int_T CANUnpack_ModeSignalID_f;      /* '<S170>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_k;      /* '<S170>/CAN Unpack' */
  struct {
    uint_T is_VehStat:4;               /* '<S91>/Chart2' */
    uint_T is_AMKDCready:4;            /* '<S91>/Chart2' */
    uint_T is_c7_VehCtrlMdel240918_2018b:4;/* '<S7>/Chart' */
    uint_T is_BeeperStat:2;            /* '<S91>/Chart2' */
    uint_T is_AMKDCon:2;               /* '<S91>/Chart2' */
    uint_T is_MCDCEnable:2;            /* '<S91>/Chart2' */
    uint_T is_MC_TorqueCUT:2;          /* '<S91>/Chart2' */
    uint_T is_AMKCANenable:2;          /* '<S91>/Chart2' */
    uint_T is_MC_InverterOn:2;         /* '<S91>/Chart2' */
    uint_T is_active_c1_VehCtrlMdel240918_:1;/* '<S91>/Chart2' */
    uint_T is_active_VehStat:1;        /* '<S91>/Chart2' */
    uint_T is_active_BeeperStat:1;     /* '<S91>/Chart2' */
    uint_T is_active_AMKDCon:1;        /* '<S91>/Chart2' */
    uint_T is_active_MCDCEnable:1;     /* '<S91>/Chart2' */
    uint_T is_active_MC_TorqueCUT:1;   /* '<S91>/Chart2' */
    uint_T is_active_AMKDCready:1;     /* '<S91>/Chart2' */
    uint_T is_active_Output:1;         /* '<S91>/Chart2' */
    uint_T is_active_AMKCANenable:1;   /* '<S91>/Chart2' */
    uint_T is_active_MC_InverterOn:1;  /* '<S91>/Chart2' */
    uint_T is_active_c7_VehCtrlMdel240918_:1;/* '<S7>/Chart' */
  } bitsForTID3;

  uint16_T UnitDelay1_DSTATE_fm;       /* '<S184>/Unit Delay1' */
  uint16_T UnitDelay_DSTATE_k;         /* '<S184>/Unit Delay' */
  uint16_T temporalCounter_i2;         /* '<S91>/Chart2' */
  boolean_T UnitDelay3_DSTATE_f;       /* '<S244>/Unit Delay3' */
  boolean_T UnitDelay3_DSTATE_l;       /* '<S10>/Unit Delay3' */
  boolean_T UnitDelay6_DSTATE_i;       /* '<S10>/Unit Delay6' */
  boolean_T UnitDelay1_DSTATE_e;       /* '<S78>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_a;       /* '<S21>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE;        /* '<S77>/Delay Input1' */
  boolean_T UnitDelay1_DSTATE_gl;      /* '<S67>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_e;       /* '<S20>/Unit Delay3' */
  boolean_T UnitDelay1_DSTATE_dp;      /* '<S58>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_i;       /* '<S19>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE_j;      /* '<S66>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_b;      /* '<S57>/Delay Input1' */
  uint8_T temporalCounter_i1;          /* '<S91>/Chart2' */
  boolean_T Subsystem_RESET_ELAPS_T;   /* '<S4>/Subsystem' */
  boolean_T FunctionCallSubsystem_RESET_ELA;/* '<S4>/Function-Call Subsystem' */
  boolean_T MoTrqReq_RESET_ELAPS_T;    /* '<S1>/MoTrqReq' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer2_j;/* '<S246>/Timer2' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer1_p;/* '<S246>/Timer1' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer3_i;/* '<S245>/Timer3' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer2_g;/* '<S245>/Timer2' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer1_m;/* '<S245>/Timer1' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer_o;/* '<S245>/Timer' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer3;/* '<S187>/Timer3' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer2_l;/* '<S187>/Timer2' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer1_n;/* '<S187>/Timer1' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer_b;/* '<S187>/Timer' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer_k;/* '<S186>/Timer' */
  DW_Timer_VehCtrlMdel240918_20_T sf_Timer_a;/* '<S184>/Timer' */
  DW_Timer_VehCtrlMdel240918_20_T sf_Timer;/* '<S105>/Timer' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer2;/* '<S8>/Timer2' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer1;/* '<S8>/Timer1' */
} DW_VehCtrlMdel240918_2018b_T;

/* Constant parameters (default storage) */
typedef struct {
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

  /* Computed Parameter: uDLookupTable_tableData
   * Referenced by: '<S186>/1-D Lookup Table'
   */
  real_T uDLookupTable_tableData[24];

  /* Computed Parameter: uDLookupTable_bp01Data
   * Referenced by: '<S186>/1-D Lookup Table'
   */
  real_T uDLookupTable_bp01Data[24];

  /* Computed Parameter: uDLookupTable1_tableData
   * Referenced by: '<S186>/1-D Lookup Table1'
   */
  real_T uDLookupTable1_tableData[24];

  /* Computed Parameter: uDLookupTable1_bp01Data
   * Referenced by: '<S186>/1-D Lookup Table1'
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
   *   '<S19>/VehSpd_SlipTarget_mps'
   *   '<S20>/VehSpd_SlipTarget_mps'
   *   '<S21>/VehSpd_SlipTarget_mps'
   */
  real32_T pooled54[4];

  /* Pooled Parameter (Expression: [0,3,25,30])
   * Referenced by:
   *   '<S19>/VehSpd_SlipTarget_mps'
   *   '<S19>/VehicleStableTarget_mps'
   *   '<S19>/VehicleStableTarget_mps1'
   *   '<S20>/VehSpd_SlipTarget_mps'
   *   '<S20>/VehicleStableTarget_mps'
   *   '<S20>/VehicleStableTarget_mps1'
   *   '<S21>/VehSpd_SlipTarget_mps'
   *   '<S21>/VehicleStableTarget_mps'
   *   '<S21>/VehicleStableTarget_mps1'
   */
  real32_T pooled55[4];

  /* Pooled Parameter (Expression: [0.4,0.4,1.2,1.2])
   * Referenced by:
   *   '<S19>/VehicleStableTarget_mps'
   *   '<S19>/VehicleStableTarget_mps1'
   *   '<S20>/VehicleStableTarget_mps'
   *   '<S20>/VehicleStableTarget_mps1'
   *   '<S21>/VehicleStableTarget_mps'
   *   '<S21>/VehicleStableTarget_mps1'
   */
  real32_T pooled61[4];

  /* Pooled Parameter (Expression: single([500,4500]);)
   * Referenced by: '<S184>/1-D Lookup Table1'
   */
  real32_T pooled65[2];

  /* Expression: single([0,100]);
   * Referenced by: '<S184>/1-D Lookup Table3'
   */
  real32_T uDLookupTable3_tableData[2];

  /* Expression: single([3540,3790])
   * Referenced by: '<S184>/1-D Lookup Table3'
   */
  real32_T uDLookupTable3_bp01Data[2];

  /* Computed Parameter: uDLookupTable1_maxIndex
   * Referenced by: '<S7>/2-D Lookup Table1'
   */
  uint32_T uDLookupTable1_maxIndex[2];
} ConstP_VehCtrlMdel240918_2018_T;

/* Real-time Model Data Structure */
struct tag_RTM_VehCtrlMdel240918_201_T {
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
extern B_VehCtrlMdel240918_2018b_T VehCtrlMdel240918_2018b_B;

/* Block states (default storage) */
extern DW_VehCtrlMdel240918_2018b_T VehCtrlMdel240918_2018b_DW;

/* Constant parameters (default storage) */
extern const ConstP_VehCtrlMdel240918_2018_T VehCtrlMdel240918_2018b_ConstP;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real_T Gear_Trs;                /* '<S319>/Switch2' */
extern real_T Mode_Trs;                /* '<S319>/Switch3' */
extern real_T Trq_CUT;                 /* '<S184>/Timer' */
extern real_T ignition;                /* '<S105>/Timer' */
extern real_T L12V_error;              /* '<S162>/CAN Unpack' */
extern real_T alarm;                   /* '<S162>/CAN Unpack' */
extern real_T controller_ready;        /* '<S162>/CAN Unpack' */
extern real_T selfcheck;               /* '<S162>/CAN Unpack' */
extern real_T RPM;                     /* '<S162>/CAN Unpack' */
extern real_T trq;                     /* '<S162>/CAN Unpack' */
extern real_T AC_current;              /* '<S157>/CAN Unpack' */
extern real_T DC_current;              /* '<S157>/CAN Unpack' */
extern real_T MCU_Temp;                /* '<S157>/CAN Unpack' */
extern real_T motor_Temp;              /* '<S157>/CAN Unpack' */
extern real_T MCFR_ActualVelocity;     /* '<S135>/CAN Unpack' */
extern real_T MCFR_DCVoltage;          /* '<S135>/CAN Unpack' */
extern real_T MCFR_bDCOn;              /* '<S135>/CAN Unpack' */
extern real_T MCFR_bError;             /* '<S135>/CAN Unpack' */
extern real_T MCFR_bInverterOn;        /* '<S135>/CAN Unpack' */
extern real_T MCFR_bQuitInverterOn;    /* '<S135>/CAN Unpack' */
extern real_T MCFR_bSystemReady;       /* '<S135>/CAN Unpack' */
extern real_T MCFR_TempIGBT;           /* '<S145>/CAN Unpack' */
extern real_T MCFR_TempInverter;       /* '<S145>/CAN Unpack' */
extern real_T MCFR_TempMotor;          /* '<S145>/CAN Unpack' */
extern real_T MCFR_ErrorInfo;          /* '<S143>/CAN Unpack' */
extern real_T MCFL_DCVoltage;          /* '<S117>/CAN Unpack' */
extern real_T MCFL_bDCOn;              /* '<S117>/CAN Unpack' */
extern real_T MCFL_bError;             /* '<S117>/CAN Unpack' */
extern real_T MCFL_bInverterOn;        /* '<S117>/CAN Unpack' */
extern real_T MCFL_bQuitDCOn;          /* '<S117>/CAN Unpack' */
extern real_T MCFL_bQuitInverterOn;    /* '<S117>/CAN Unpack' */
extern real_T MCFL_bSystemReady;       /* '<S117>/CAN Unpack' */
extern real_T MCFL_ActualVelocity;     /* '<S117>/Gain' */
extern real_T MCFL_TempIGBT;           /* '<S128>/CAN Unpack' */
extern real_T MCFL_TempInverter;       /* '<S128>/CAN Unpack' */
extern real_T MCFL_TempMotor;          /* '<S128>/CAN Unpack' */
extern real_T MCFL_ErrorInfo;          /* '<S126>/CAN Unpack' */
extern real_T StrWhlAngAliveRollCnt;   /* '<S175>/CAN Unpack1' */
extern real_T StrWhlAng;               /* '<S175>/CAN Unpack1' */
extern real_T StrWhlAngV;              /* '<S175>/CAN Unpack1' */
extern real_T ABS_WS_FL;               /* '<S107>/CAN Unpack1' */
extern real_T ABS_WS_FR;               /* '<S107>/CAN Unpack1' */
extern real_T ABS_WS_RL;               /* '<S107>/CAN Unpack1' */
extern real_T ABS_WS_RR;               /* '<S107>/CAN Unpack1' */
extern real_T IMU_Ay_Value;            /* '<S170>/CAN Unpack' */
extern real_T IMU_Ax_Value;            /* '<S170>/CAN Unpack' */
extern real_T IMU_Yaw_Value;           /* '<S170>/CAN Unpack' */
extern uint32_T Acc_vol2;              /* '<S184>/Add3' */
extern uint32_T Acc_vol;               /* '<S184>/Add2' */
extern uint32_T Acc_POS2;              /* '<S184>/1-D Lookup Table3' */
extern real32_T Acc_POS;               /* '<S184>/MATLAB Function' */
extern uint16_T F_BrkPrs;              /* '<S184>/1-D Lookup Table1' */
extern uint16_T Acc1;                  /* '<S100>/Acc3' */
extern uint16_T Acc2;                  /* '<S100>/Acc4' */
extern uint16_T Brk1;                  /* '<S100>/Brk1' */
extern uint16_T Brk2;                  /* '<S100>/Brk2' */
extern boolean_T HVCUTOFF;             /* '<S105>/Constant' */
extern boolean_T KeyPressed;           /* '<S91>/Cast To Boolean' */
extern boolean_T Brk;                  /* '<S93>/Compare' */
extern boolean_T ACC_Release;          /* '<S94>/Compare' */
extern boolean_T beeper_state;         /* '<S91>/Chart2' */
extern boolean_T Trq_CUT_final;        /* '<S7>/Logical Operator4' */

/* External function called from main */
extern void VehCtrlMdel240918_2018b_SetEventsForThisBaseStep(boolean_T
  *eventFlags);

/* Model entry point functions */
extern void VehCtrlMdel240918_2018b_SetEventsForThisBaseStep(boolean_T
  *eventFlags);
extern void VehCtrlMdel240918_2018b_initialize(void);
extern void VehCtrlMdel240918_2018b_step(int_T tid);
extern uint8_T ECUCoderModelBaseCounter;
extern uint32_t IntcIsrVectorTable[];
extern uint8_T AfterRunFlags[2];
extern SSD_CONFIG ssdConfig;
extern void ISR_PIT_CH3(void);

/* Real-time Model object */
extern RT_MODEL_VehCtrlMdel240918_20_T *const VehCtrlMdel240918_2018b_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S10>/2-D Lookup Table' : Unused code path elimination
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
 * Block '<S36>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S51>/Data Type Duplicate' : Unused code path elimination
 * Block '<S51>/Data Type Propagation' : Unused code path elimination
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
 * Block '<S42>/Data Type Duplicate' : Unused code path elimination
 * Block '<S42>/Data Type Propagation' : Unused code path elimination
 * Block '<S12>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S52>/Data Type Duplicate' : Unused code path elimination
 * Block '<S52>/Data Type Propagation' : Unused code path elimination
 * Block '<S13>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S53>/Data Type Duplicate' : Unused code path elimination
 * Block '<S53>/Data Type Propagation' : Unused code path elimination
 * Block '<S14>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S54>/Data Type Duplicate' : Unused code path elimination
 * Block '<S54>/Data Type Propagation' : Unused code path elimination
 * Block '<S15>/Data Type Duplicate' : Unused code path elimination
 * Block '<S15>/Data Type Propagation' : Unused code path elimination
 * Block '<S16>/Data Type Duplicate' : Unused code path elimination
 * Block '<S16>/Data Type Propagation' : Unused code path elimination
 * Block '<S17>/Data Type Duplicate' : Unused code path elimination
 * Block '<S17>/Data Type Propagation' : Unused code path elimination
 * Block '<S60>/Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Propagation' : Unused code path elimination
 * Block '<S61>/Data Type Duplicate' : Unused code path elimination
 * Block '<S61>/Data Type Propagation' : Unused code path elimination
 * Block '<S62>/Data Type Duplicate' : Unused code path elimination
 * Block '<S62>/Data Type Propagation' : Unused code path elimination
 * Block '<S69>/Data Type Duplicate' : Unused code path elimination
 * Block '<S69>/Data Type Propagation' : Unused code path elimination
 * Block '<S70>/Data Type Duplicate' : Unused code path elimination
 * Block '<S70>/Data Type Propagation' : Unused code path elimination
 * Block '<S71>/Data Type Duplicate' : Unused code path elimination
 * Block '<S71>/Data Type Propagation' : Unused code path elimination
 * Block '<S80>/Data Type Duplicate' : Unused code path elimination
 * Block '<S80>/Data Type Propagation' : Unused code path elimination
 * Block '<S81>/Data Type Duplicate' : Unused code path elimination
 * Block '<S81>/Data Type Propagation' : Unused code path elimination
 * Block '<S82>/Data Type Duplicate' : Unused code path elimination
 * Block '<S82>/Data Type Propagation' : Unused code path elimination
 * Block '<S154>/CAN Unpack1' : Unused code path elimination
 * Block '<S184>/1-D Lookup Table2' : Unused code path elimination
 * Block '<S184>/Abs1' : Unused code path elimination
 * Block '<S184>/Add4' : Unused code path elimination
 * Block '<S190>/Compare' : Unused code path elimination
 * Block '<S190>/Constant' : Unused code path elimination
 * Block '<S191>/Compare' : Unused code path elimination
 * Block '<S191>/Constant' : Unused code path elimination
 * Block '<S199>/Compare' : Unused code path elimination
 * Block '<S199>/Constant' : Unused code path elimination
 * Block '<S184>/Logical Operator6' : Unused code path elimination
 * Block '<S184>/Logical Operator7' : Unused code path elimination
 * Block '<S209>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S215>/Data Type Duplicate' : Unused code path elimination
 * Block '<S215>/Data Type Propagation' : Unused code path elimination
 * Block '<S210>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S216>/Data Type Duplicate' : Unused code path elimination
 * Block '<S216>/Data Type Propagation' : Unused code path elimination
 * Block '<S211>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S217>/Data Type Duplicate' : Unused code path elimination
 * Block '<S217>/Data Type Propagation' : Unused code path elimination
 * Block '<S212>/Data Type Duplicate' : Unused code path elimination
 * Block '<S212>/Data Type Propagation' : Unused code path elimination
 * Block '<S213>/Data Type Duplicate' : Unused code path elimination
 * Block '<S213>/Data Type Propagation' : Unused code path elimination
 * Block '<S214>/Data Type Duplicate' : Unused code path elimination
 * Block '<S214>/Data Type Propagation' : Unused code path elimination
 * Block '<S219>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S222>/Data Type Duplicate' : Unused code path elimination
 * Block '<S222>/Data Type Propagation' : Unused code path elimination
 * Block '<S220>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S223>/Data Type Duplicate' : Unused code path elimination
 * Block '<S223>/Data Type Propagation' : Unused code path elimination
 * Block '<S232>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S240>/Data Type Duplicate' : Unused code path elimination
 * Block '<S240>/Data Type Propagation' : Unused code path elimination
 * Block '<S233>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S241>/Data Type Duplicate' : Unused code path elimination
 * Block '<S241>/Data Type Propagation' : Unused code path elimination
 * Block '<S234>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S242>/Data Type Duplicate' : Unused code path elimination
 * Block '<S242>/Data Type Propagation' : Unused code path elimination
 * Block '<S235>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S243>/Data Type Duplicate' : Unused code path elimination
 * Block '<S243>/Data Type Propagation' : Unused code path elimination
 * Block '<S265>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S267>/Data Type Duplicate' : Unused code path elimination
 * Block '<S267>/Data Type Propagation' : Unused code path elimination
 * Block '<S266>/Data Type Duplicate' : Unused code path elimination
 * Block '<S266>/Data Type Propagation' : Unused code path elimination
 * Block '<S268>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S270>/Data Type Duplicate' : Unused code path elimination
 * Block '<S270>/Data Type Propagation' : Unused code path elimination
 * Block '<S269>/Data Type Duplicate' : Unused code path elimination
 * Block '<S269>/Data Type Propagation' : Unused code path elimination
 * Block '<S271>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S273>/Data Type Duplicate' : Unused code path elimination
 * Block '<S273>/Data Type Propagation' : Unused code path elimination
 * Block '<S272>/Data Type Duplicate' : Unused code path elimination
 * Block '<S272>/Data Type Propagation' : Unused code path elimination
 * Block '<S274>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S276>/Data Type Duplicate' : Unused code path elimination
 * Block '<S276>/Data Type Propagation' : Unused code path elimination
 * Block '<S275>/Data Type Duplicate' : Unused code path elimination
 * Block '<S275>/Data Type Propagation' : Unused code path elimination
 * Block '<S285>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S287>/Data Type Duplicate' : Unused code path elimination
 * Block '<S287>/Data Type Propagation' : Unused code path elimination
 * Block '<S286>/Data Type Duplicate' : Unused code path elimination
 * Block '<S286>/Data Type Propagation' : Unused code path elimination
 * Block '<S288>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S290>/Data Type Duplicate' : Unused code path elimination
 * Block '<S290>/Data Type Propagation' : Unused code path elimination
 * Block '<S289>/Data Type Duplicate' : Unused code path elimination
 * Block '<S289>/Data Type Propagation' : Unused code path elimination
 * Block '<S291>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S293>/Data Type Duplicate' : Unused code path elimination
 * Block '<S293>/Data Type Propagation' : Unused code path elimination
 * Block '<S292>/Data Type Duplicate' : Unused code path elimination
 * Block '<S292>/Data Type Propagation' : Unused code path elimination
 * Block '<S294>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S296>/Data Type Duplicate' : Unused code path elimination
 * Block '<S296>/Data Type Propagation' : Unused code path elimination
 * Block '<S295>/Data Type Duplicate' : Unused code path elimination
 * Block '<S295>/Data Type Propagation' : Unused code path elimination
 * Block '<S246>/Discrete-Time Integrator' : Unused code path elimination
 * Block '<S310>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S312>/Data Type Duplicate' : Unused code path elimination
 * Block '<S312>/Data Type Propagation' : Unused code path elimination
 * Block '<S311>/Data Type Duplicate' : Unused code path elimination
 * Block '<S311>/Data Type Propagation' : Unused code path elimination
 * Block '<S304>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S313>/Data Type Duplicate' : Unused code path elimination
 * Block '<S313>/Data Type Propagation' : Unused code path elimination
 * Block '<S305>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S314>/Data Type Duplicate' : Unused code path elimination
 * Block '<S314>/Data Type Propagation' : Unused code path elimination
 * Block '<S306>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S315>/Data Type Duplicate' : Unused code path elimination
 * Block '<S315>/Data Type Propagation' : Unused code path elimination
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
 * Block '<S184>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S244>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S244>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S245>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S245>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S245>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S245>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S303>/Gain1' : Eliminated nontunable gain of 1
 * Block '<S319>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S320>/Cast To Single' : Eliminate redundant data type conversion
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
 * '<Root>' : 'VehCtrlMdel240918_2018b'
 * '<S1>'   : 'VehCtrlMdel240918_2018b/CTRL'
 * '<S2>'   : 'VehCtrlMdel240918_2018b/Drive_Ready'
 * '<S3>'   : 'VehCtrlMdel240918_2018b/Input'
 * '<S4>'   : 'VehCtrlMdel240918_2018b/Input_Processing'
 * '<S5>'   : 'VehCtrlMdel240918_2018b/OUTPUT'
 * '<S6>'   : 'VehCtrlMdel240918_2018b/RapidECUSetting'
 * '<S7>'   : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq'
 * '<S8>'   : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct'
 * '<S9>'   : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Chart'
 * '<S10>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC'
 * '<S11>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/MeaModule'
 * '<S12>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic1'
 * '<S13>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic2'
 * '<S14>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic3'
 * '<S15>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Saturation Dynamic1'
 * '<S16>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Saturation Dynamic2'
 * '<S17>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Saturation Dynamic3'
 * '<S18>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Subsystem'
 * '<S19>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL'
 * '<S20>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR'
 * '<S21>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R'
 * '<S22>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Compare To Constant'
 * '<S23>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Compare To Constant1'
 * '<S24>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Compare To Constant2'
 * '<S25>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Compare To Constant3'
 * '<S26>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Compare To Constant4'
 * '<S27>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Compare To Constant5'
 * '<S28>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/MATLAB Function'
 * '<S29>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/MATLAB Function1'
 * '<S30>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic'
 * '<S31>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic1'
 * '<S32>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic2'
 * '<S33>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic3'
 * '<S34>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic4'
 * '<S35>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic5'
 * '<S36>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic6'
 * '<S37>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic'
 * '<S38>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic1'
 * '<S39>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic2'
 * '<S40>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic3'
 * '<S41>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic4'
 * '<S42>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic5'
 * '<S43>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Wtarget'
 * '<S44>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/载荷转移'
 * '<S45>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S46>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S47>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S48>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S49>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic4/Saturation Dynamic'
 * '<S50>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic5/Saturation Dynamic'
 * '<S51>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic6/Saturation Dynamic'
 * '<S52>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S53>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S54>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S55>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Compare To Constant'
 * '<S56>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Compare To Constant2'
 * '<S57>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Detect Rise Positive'
 * '<S58>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Latch_on'
 * '<S59>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/RisingTimer'
 * '<S60>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic'
 * '<S61>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic1'
 * '<S62>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic2'
 * '<S63>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Detect Rise Positive/Positive'
 * '<S64>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Compare To Constant'
 * '<S65>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Compare To Constant2'
 * '<S66>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Detect Rise Positive'
 * '<S67>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Latch_on'
 * '<S68>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/RisingTimer'
 * '<S69>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic'
 * '<S70>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic1'
 * '<S71>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic2'
 * '<S72>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Detect Rise Positive/Positive'
 * '<S73>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant'
 * '<S74>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant1'
 * '<S75>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant2'
 * '<S76>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Zero'
 * '<S77>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Detect Rise Positive'
 * '<S78>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Latch_on'
 * '<S79>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/RisingTimer'
 * '<S80>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic'
 * '<S81>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic1'
 * '<S82>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic2'
 * '<S83>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Detect Rise Positive/Positive'
 * '<S84>'  : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant'
 * '<S85>'  : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant1'
 * '<S86>'  : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant2'
 * '<S87>'  : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant3'
 * '<S88>'  : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant4'
 * '<S89>'  : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct/Timer1'
 * '<S90>'  : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct/Timer2'
 * '<S91>'  : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem'
 * '<S92>'  : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/Chart2'
 * '<S93>'  : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/Compare To Constant'
 * '<S94>'  : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/Compare To Constant1'
 * '<S95>'  : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/MeaModule'
 * '<S96>'  : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/MeaModule1'
 * '<S97>'  : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/MeaModule2'
 * '<S98>'  : 'VehCtrlMdel240918_2018b/Input/ABS_Receive'
 * '<S99>'  : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive'
 * '<S100>' : 'VehCtrlMdel240918_2018b/Input/AccBrk_BUS'
 * '<S101>' : 'VehCtrlMdel240918_2018b/Input/BMS_Recive'
 * '<S102>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE'
 * '<S103>' : 'VehCtrlMdel240918_2018b/Input/IMU_Recieve'
 * '<S104>' : 'VehCtrlMdel240918_2018b/Input/StrSnis_Receive'
 * '<S105>' : 'VehCtrlMdel240918_2018b/Input/key'
 * '<S106>' : 'VehCtrlMdel240918_2018b/Input/ABS_Receive/ABS_BUS_state'
 * '<S107>' : 'VehCtrlMdel240918_2018b/Input/ABS_Receive/ABS_BUS_state/IMU_state'
 * '<S108>' : 'VehCtrlMdel240918_2018b/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule1'
 * '<S109>' : 'VehCtrlMdel240918_2018b/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule2'
 * '<S110>' : 'VehCtrlMdel240918_2018b/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule3'
 * '<S111>' : 'VehCtrlMdel240918_2018b/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule4'
 * '<S112>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU'
 * '<S113>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU'
 * '<S114>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state'
 * '<S115>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1'
 * '<S116>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2'
 * '<S117>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state'
 * '<S118>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule1'
 * '<S119>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule2'
 * '<S120>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule3'
 * '<S121>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule4'
 * '<S122>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule5'
 * '<S123>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule6'
 * '<S124>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule7'
 * '<S125>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule8'
 * '<S126>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state'
 * '<S127>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state/MeaModule1'
 * '<S128>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state'
 * '<S129>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule1'
 * '<S130>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule2'
 * '<S131>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule3'
 * '<S132>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state'
 * '<S133>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1'
 * '<S134>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2'
 * '<S135>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state'
 * '<S136>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule1'
 * '<S137>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule2'
 * '<S138>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule3'
 * '<S139>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule4'
 * '<S140>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule5'
 * '<S141>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule6'
 * '<S142>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule7'
 * '<S143>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state'
 * '<S144>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state/MeaModule1'
 * '<S145>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state'
 * '<S146>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule1'
 * '<S147>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule2'
 * '<S148>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule3'
 * '<S149>' : 'VehCtrlMdel240918_2018b/Input/AccBrk_BUS/MeaModule1'
 * '<S150>' : 'VehCtrlMdel240918_2018b/Input/AccBrk_BUS/MeaModule2'
 * '<S151>' : 'VehCtrlMdel240918_2018b/Input/AccBrk_BUS/MeaModule3'
 * '<S152>' : 'VehCtrlMdel240918_2018b/Input/AccBrk_BUS/MeaModule4'
 * '<S153>' : 'VehCtrlMdel240918_2018b/Input/BMS_Recive/ABS_BUS_state'
 * '<S154>' : 'VehCtrlMdel240918_2018b/Input/BMS_Recive/ABS_BUS_state/IMU_state'
 * '<S155>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr'
 * '<S156>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state'
 * '<S157>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1'
 * '<S158>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule1'
 * '<S159>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule2'
 * '<S160>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule3'
 * '<S161>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule4'
 * '<S162>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state'
 * '<S163>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule'
 * '<S164>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule1'
 * '<S165>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule2'
 * '<S166>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule3'
 * '<S167>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule5'
 * '<S168>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule6'
 * '<S169>' : 'VehCtrlMdel240918_2018b/Input/IMU_Recieve/IMU_state'
 * '<S170>' : 'VehCtrlMdel240918_2018b/Input/IMU_Recieve/IMU_state/MCU_state'
 * '<S171>' : 'VehCtrlMdel240918_2018b/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule2'
 * '<S172>' : 'VehCtrlMdel240918_2018b/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule3'
 * '<S173>' : 'VehCtrlMdel240918_2018b/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule4'
 * '<S174>' : 'VehCtrlMdel240918_2018b/Input/StrSnis_Receive/StrWhSnis_state'
 * '<S175>' : 'VehCtrlMdel240918_2018b/Input/StrSnis_Receive/StrWhSnis_state/IMU_state'
 * '<S176>' : 'VehCtrlMdel240918_2018b/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule1'
 * '<S177>' : 'VehCtrlMdel240918_2018b/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule2'
 * '<S178>' : 'VehCtrlMdel240918_2018b/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule3'
 * '<S179>' : 'VehCtrlMdel240918_2018b/Input/key/MeaModule'
 * '<S180>' : 'VehCtrlMdel240918_2018b/Input/key/MeaModule1'
 * '<S181>' : 'VehCtrlMdel240918_2018b/Input/key/Timer'
 * '<S182>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem'
 * '<S183>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem'
 * '<S184>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal'
 * '<S185>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU'
 * '<S186>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/SWAS'
 * '<S187>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps'
 * '<S188>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant'
 * '<S189>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant1'
 * '<S190>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant10'
 * '<S191>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant11'
 * '<S192>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant2'
 * '<S193>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant3'
 * '<S194>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant4'
 * '<S195>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant5'
 * '<S196>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant6'
 * '<S197>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant7'
 * '<S198>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant8'
 * '<S199>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant9'
 * '<S200>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MATLAB Function'
 * '<S201>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule1'
 * '<S202>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule2'
 * '<S203>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule3'
 * '<S204>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule5'
 * '<S205>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule7'
 * '<S206>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule8'
 * '<S207>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Timer'
 * '<S208>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem'
 * '<S209>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic'
 * '<S210>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1'
 * '<S211>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2'
 * '<S212>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic'
 * '<S213>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic1'
 * '<S214>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic2'
 * '<S215>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S216>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S217>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S218>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/SWAS/Compare To Constant'
 * '<S219>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic'
 * '<S220>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1'
 * '<S221>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/SWAS/Timer'
 * '<S222>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S223>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S224>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant'
 * '<S225>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant1'
 * '<S226>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant2'
 * '<S227>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant3'
 * '<S228>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant4'
 * '<S229>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant5'
 * '<S230>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant6'
 * '<S231>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant7'
 * '<S232>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic'
 * '<S233>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1'
 * '<S234>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2'
 * '<S235>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3'
 * '<S236>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer'
 * '<S237>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer1'
 * '<S238>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer2'
 * '<S239>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer3'
 * '<S240>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S241>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S242>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S243>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S244>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/IntglJudgment '
 * '<S245>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment'
 * '<S246>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect'
 * '<S247>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/坐标系转换'
 * '<S248>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/IntglJudgment /Compare To Constant'
 * '<S249>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/IntglJudgment /Compare To Constant1'
 * '<S250>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/Timer'
 * '<S251>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/Timer1'
 * '<S252>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/Timer2'
 * '<S253>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/Timer3'
 * '<S254>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断'
 * '<S255>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断'
 * '<S256>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断'
 * '<S257>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant'
 * '<S258>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant1'
 * '<S259>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant2'
 * '<S260>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant3'
 * '<S261>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter'
 * '<S262>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1'
 * '<S263>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2'
 * '<S264>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3'
 * '<S265>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic'
 * '<S266>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Saturation Dynamic'
 * '<S267>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S268>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic'
 * '<S269>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Saturation Dynamic'
 * '<S270>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S271>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic'
 * '<S272>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Saturation Dynamic'
 * '<S273>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S274>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic'
 * '<S275>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Saturation Dynamic'
 * '<S276>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S277>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant'
 * '<S278>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant1'
 * '<S279>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant2'
 * '<S280>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant3'
 * '<S281>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter'
 * '<S282>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1'
 * '<S283>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2'
 * '<S284>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3'
 * '<S285>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic'
 * '<S286>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Saturation Dynamic'
 * '<S287>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S288>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic'
 * '<S289>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Saturation Dynamic'
 * '<S290>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S291>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic'
 * '<S292>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Saturation Dynamic'
 * '<S293>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S294>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic'
 * '<S295>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Saturation Dynamic'
 * '<S296>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S297>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant'
 * '<S298>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant1'
 * '<S299>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant2'
 * '<S300>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant3'
 * '<S301>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant'
 * '<S302>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant1'
 * '<S303>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Filter'
 * '<S304>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic'
 * '<S305>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1'
 * '<S306>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2'
 * '<S307>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Subsystem'
 * '<S308>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Timer1'
 * '<S309>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Timer2'
 * '<S310>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic'
 * '<S311>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Filter/Saturation Dynamic'
 * '<S312>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S313>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S314>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S315>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S316>' : 'VehCtrlMdel240918_2018b/OUTPUT/Beeper'
 * '<S317>' : 'VehCtrlMdel240918_2018b/OUTPUT/VCU2AMKMCUFL'
 * '<S318>' : 'VehCtrlMdel240918_2018b/OUTPUT/VCU2AMKMCUFR'
 * '<S319>' : 'VehCtrlMdel240918_2018b/OUTPUT/VCU2EmraxMCU'
 * '<S320>' : 'VehCtrlMdel240918_2018b/OUTPUT/WP_OUTPUT'
 * '<S321>' : 'VehCtrlMdel240918_2018b/OUTPUT/Beeper/MeaModule'
 * '<S322>' : 'VehCtrlMdel240918_2018b/OUTPUT/VCU2EmraxMCU/MeaModule1'
 * '<S323>' : 'VehCtrlMdel240918_2018b/OUTPUT/VCU2EmraxMCU/MeaModule2'
 * '<S324>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL'
 * '<S325>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/DAQ'
 * '<S326>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/EEPROM'
 * '<S327>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/Polling'
 * '<S328>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem'
 * '<S329>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem'
 * '<S330>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem'
 * '<S331>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com0'
 * '<S332>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com1'
 * '<S333>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com2'
 * '<S334>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem'
 * '<S335>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem1'
 * '<S336>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/DAQ/daq100ms'
 * '<S337>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/DAQ/daq10ms'
 * '<S338>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/DAQ/daq500ms'
 * '<S339>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/DAQ/daq50ms'
 * '<S340>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/DAQ/daq5ms'
 * '<S341>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/EEPROM/EEPROMOperation'
 * '<S342>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/Polling/CCPBackground'
 * '<S343>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/Polling/CCPReceive'
 * '<S344>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/Polling/CCPReceive/Nothing'
 */
#endif                               /* RTW_HEADER_VehCtrlMdel240918_2018b_h_ */

/* File trailer for ECUCoder generated file VehCtrlMdel240918_2018b.h.
 *
 * [EOF]
 */
