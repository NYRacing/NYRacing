/*
 * Code generated for Simulink model VehCtrlMdel240918_2018b.
 *
 * FILE    : VehCtrlMdel240918_2018b.h
 *
 * VERSION : 1.169
 *
 * DATE    : Wed Sep 25 21:43:24 2024
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

/* Block states (default storage) for system '<S114>/Timer' */
typedef struct {
  real_T x;                            /* '<S114>/Timer' */
  struct {
    uint_T is_c21_VehCtrlMdel240918_2018b:2;/* '<S114>/Timer' */
    uint_T is_active_c21_VehCtrlMdel240918:1;/* '<S114>/Timer' */
  } bitsForTID3;
} DW_Timer_VehCtrlMdel240918_20_T;

/* Block signals (default storage) */
typedef struct {
  CAN_DATATYPE CANPack1;               /* '<S327>/CAN Pack1' */
  CAN_DATATYPE CANPack1_a;             /* '<S328>/CAN Pack1' */
  CAN_DATATYPE CANPack1_d;             /* '<S326>/CAN Pack1' */
  CAN_DATATYPE CANUnPackMessage4;      /* '<S171>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_g;    /* '<S166>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_j;    /* '<S144>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_n;    /* '<S154>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_p;    /* '<S152>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_e;    /* '<S126>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_c;    /* '<S137>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_b;    /* '<S135>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_l;    /* '<S184>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_ja;   /* '<S116>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_h;    /* '<S179>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_i;    /* '<S163>/CANUnPackMessage4' */
  real_T MCFL_TorqueLimitP;            /* '<S327>/Switch' */
  real_T MCFL_TorqueLimitP_n;          /* '<S326>/Switch' */
  real_T Exit;                         /* '<S255>/Timer2' */
  real_T Exit_l;                       /* '<S255>/Timer1' */
  real_T Exit_a;                       /* '<S254>/Timer3' */
  real_T Exit_lh;                      /* '<S254>/Timer2' */
  real_T Exit_lh4;                     /* '<S254>/Timer1' */
  real_T Exit_c;                       /* '<S254>/Timer' */
  real_T Exit_h;                       /* '<S196>/Timer3' */
  real_T Exit_o;                       /* '<S196>/Timer2' */
  real_T Exit_i;                       /* '<S196>/Timer1' */
  real_T Exit_le;                      /* '<S196>/Timer' */
  real_T Exit_on;                      /* '<S195>/Timer' */
  real_T low_VOL;                      /* '<S171>/CAN Unpack' */
  real_T MCU_Temp_error;               /* '<S171>/CAN Unpack' */
  real_T Mode;                         /* '<S171>/CAN Unpack' */
  real_T motorTemp_error;              /* '<S171>/CAN Unpack' */
  real_T overCurrent;                  /* '<S171>/CAN Unpack' */
  real_T overpower;                    /* '<S171>/CAN Unpack' */
  real_T overvol;                      /* '<S171>/CAN Unpack' */
  real_T Precharge;                    /* '<S171>/CAN Unpack' */
  real_T Reslove_error;                /* '<S171>/CAN Unpack' */
  real_T voltage;                      /* '<S166>/CAN Unpack' */
  real_T MCFR_ActualTorque;            /* '<S144>/CAN Unpack' */
  real_T MCFR_bDerating;               /* '<S144>/CAN Unpack' */
  real_T MCFR_bQuitDCOn;               /* '<S144>/CAN Unpack' */
  real_T MCFR_bReserve;                /* '<S144>/CAN Unpack' */
  real_T MCFR_bWarn;                   /* '<S144>/CAN Unpack' */
  real_T MCFR_DiagnosticNum;           /* '<S152>/CAN Unpack' */
  real_T MCFL_ActualTorque;            /* '<S126>/CAN Unpack' */
  real_T MCFL_ActualVelocity_p;        /* '<S126>/CAN Unpack' */
  real_T MCFL_bDerating;               /* '<S126>/CAN Unpack' */
  real_T MCFL_bReserve;                /* '<S126>/CAN Unpack' */
  real_T MCFL_bWarn;                   /* '<S126>/CAN Unpack' */
  real_T MCFL_DiagnosticNum;           /* '<S135>/CAN Unpack' */
  real_T CANUnpack_o1;                 /* '<S179>/CAN Unpack' */
  real_T CANUnpack_o3;                 /* '<S179>/CAN Unpack' */
  real_T CANUnpack_o5;                 /* '<S179>/CAN Unpack' */
  real_T CANUnpack_o6;                 /* '<S179>/CAN Unpack' */
  real_T errorReset;                   /* '<S97>/Chart2' */
  real_T Exit_d;                       /* '<S8>/Timer2' */
  real_T Exit_g;                       /* '<S8>/Timer1' */
  real_T DYC_Enable_OUT;               /* '<S7>/Chart' */
  real_T TCSR_Enable_OUT;              /* '<S7>/Chart' */
  real_T TCSF_Enable_OUT;              /* '<S7>/Chart' */
  uint32_T CANReceive_o3;              /* '<S352>/CANReceive' */
  uint32_T CANReceive_o3_l;            /* '<S337>/CANReceive' */
  uint32_T CANReceive1_o3;             /* '<S111>/CANReceive1' */
  uint32_T CANReceive3_o3;             /* '<S111>/CANReceive3' */
  uint32_T CANReceive3_o3_e;           /* '<S121>/CANReceive3' */
  uint32_T CANReceive1_o3_n;           /* '<S121>/CANReceive1' */
  uint32_T CANReceive2_o3;             /* '<S121>/CANReceive2' */
  uint32_T CANReceive3_o3_i;           /* '<S122>/CANReceive3' */
  uint32_T CANReceive1_o3_h;           /* '<S122>/CANReceive1' */
  uint32_T CANReceive2_o3_j;           /* '<S122>/CANReceive2' */
  uint32_T CANReceive3_o3_c;           /* '<S113>/CANReceive3' */
  uint32_T CANReceive3_o3_m;           /* '<S107>/CANReceive3' */
  uint32_T CANReceive3_o3_cz;          /* '<S112>/CANReceive3' */
  uint32_T CANReceive3_o3_l;           /* '<S110>/CANReceive3' */
  int32_T DataTypeConversion2;         /* '<S328>/Data Type Conversion2' */
  uint16_T CastToSingle1;              /* '<S329>/Cast To Single1' */
  uint16_T CastToBoolean4;             /* '<S328>/Cast To Boolean4' */
  uint16_T CastToBoolean6;             /* '<S328>/Cast To Boolean6' */
  uint8_T CANReceive_o2;               /* '<S352>/CANReceive' */
  uint8_T CANReceive_o4[8];            /* '<S352>/CANReceive' */
  uint8_T CANReceive_o5;               /* '<S352>/CANReceive' */
  uint8_T CANReceive_o2_p;             /* '<S337>/CANReceive' */
  uint8_T CANReceive_o4_i[8];          /* '<S337>/CANReceive' */
  uint8_T CANReceive_o5_l;             /* '<S337>/CANReceive' */
  uint8_T CANTransmit;                 /* '<S344>/CANTransmit' */
  uint8_T CANPackMessage[8];           /* '<S327>/CANPackMessage' */
  uint8_T CANTransmit_l;               /* '<S327>/CANTransmit' */
  uint8_T CANPackMessage_d[8];         /* '<S328>/CANPackMessage' */
  uint8_T CANTransmit_k;               /* '<S328>/CANTransmit' */
  uint8_T CANPackMessage_h[8];         /* '<S326>/CANPackMessage' */
  uint8_T CANTransmit_c;               /* '<S326>/CANTransmit' */
  uint8_T CANReceive1_o2;              /* '<S111>/CANReceive1' */
  uint8_T CANReceive1_o4[8];           /* '<S111>/CANReceive1' */
  uint8_T CANReceive1_o5;              /* '<S111>/CANReceive1' */
  uint8_T CANReceive3_o2;              /* '<S111>/CANReceive3' */
  uint8_T CANReceive3_o4[8];           /* '<S111>/CANReceive3' */
  uint8_T CANReceive3_o5;              /* '<S111>/CANReceive3' */
  uint8_T CANReceive3_o2_l;            /* '<S121>/CANReceive3' */
  uint8_T CANReceive3_o4_l[8];         /* '<S121>/CANReceive3' */
  uint8_T CANReceive3_o5_a;            /* '<S121>/CANReceive3' */
  uint8_T CANReceive1_o2_l;            /* '<S121>/CANReceive1' */
  uint8_T CANReceive1_o4_c[8];         /* '<S121>/CANReceive1' */
  uint8_T CANReceive1_o5_a;            /* '<S121>/CANReceive1' */
  uint8_T CANReceive2_o2;              /* '<S121>/CANReceive2' */
  uint8_T CANReceive2_o4[6];           /* '<S121>/CANReceive2' */
  uint8_T CANReceive2_o5;              /* '<S121>/CANReceive2' */
  uint8_T CANReceive3_o2_a;            /* '<S122>/CANReceive3' */
  uint8_T CANReceive3_o4_g[8];         /* '<S122>/CANReceive3' */
  uint8_T CANReceive3_o5_an;           /* '<S122>/CANReceive3' */
  uint8_T CANReceive1_o2_o;            /* '<S122>/CANReceive1' */
  uint8_T CANReceive1_o4_j[8];         /* '<S122>/CANReceive1' */
  uint8_T CANReceive1_o5_j;            /* '<S122>/CANReceive1' */
  uint8_T CANReceive2_o2_p;            /* '<S122>/CANReceive2' */
  uint8_T CANReceive2_o4_k[6];         /* '<S122>/CANReceive2' */
  uint8_T CANReceive2_o5_e;            /* '<S122>/CANReceive2' */
  uint8_T CANReceive3_o2_p;            /* '<S113>/CANReceive3' */
  uint8_T CANReceive3_o4_k[8];         /* '<S113>/CANReceive3' */
  uint8_T CANReceive3_o5_d;            /* '<S113>/CANReceive3' */
  uint8_T CANReceive3_o2_m;            /* '<S107>/CANReceive3' */
  uint8_T CANReceive3_o4_lg[8];        /* '<S107>/CANReceive3' */
  uint8_T CANReceive3_o5_b;            /* '<S107>/CANReceive3' */
  uint8_T CANReceive3_o2_ma;           /* '<S112>/CANReceive3' */
  uint8_T CANReceive3_o4_i[8];         /* '<S112>/CANReceive3' */
  uint8_T CANReceive3_o5_m;            /* '<S112>/CANReceive3' */
  uint8_T CANReceive3_o2_k;            /* '<S110>/CANReceive3' */
  uint8_T CANReceive3_o4_p[8];         /* '<S110>/CANReceive3' */
  uint8_T CANReceive3_o5_de;           /* '<S110>/CANReceive3' */
  boolean_T Drive_ready;               /* '<S114>/SwitchInput' */
  boolean_T VehReady;                  /* '<S97>/Chart2' */
  boolean_T MCFL_InverterOn;           /* '<S97>/Chart2' */
  boolean_T MCFL_DCEnable;             /* '<S97>/Chart2' */
  boolean_T MCFR_TorqueOn;             /* '<S97>/Chart2' */
  boolean_T MCFL_TorqueOn;             /* '<S97>/Chart2' */
  boolean_T aWaterPumpON;
} B_VehCtrlMdel240918_2018b_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DelayInput2_DSTATE;           /* '<S228>/Delay Input2' */
  real_T DelayInput2_DSTATE_a;         /* '<S241>/Delay Input2' */
  real_T DelayInput2_DSTATE_b;         /* '<S242>/Delay Input2' */
  real_T DelayInput2_DSTATE_h;         /* '<S243>/Delay Input2' */
  real_T DelayInput2_DSTATE_n;         /* '<S244>/Delay Input2' */
  real_T UnitDelay_DSTATE;             /* '<S195>/Unit Delay' */
  real_T DelayInput2_DSTATE_l;         /* '<S218>/Delay Input2' */
  real_T UnitDelay_DSTATE_g;           /* '<S217>/Unit Delay' */
  real_T DelayInput2_DSTATE_m;         /* '<S219>/Delay Input2' */
  real_T UnitDelay1_DSTATE;            /* '<S217>/Unit Delay1' */
  real_T DelayInput2_DSTATE_j;         /* '<S229>/Delay Input2' */
  real_T UnitDelay1_DSTATE_a;          /* '<S195>/Unit Delay1' */
  real_T DelayInput2_DSTATE_k;         /* '<S220>/Delay Input2' */
  real_T UnitDelay2_DSTATE;            /* '<S217>/Unit Delay2' */
  real_T DelayInput2_DSTATE_jk;        /* '<S39>/Delay Input2' */
  real_T UnitDelay_DSTATE_n;           /* '<S7>/Unit Delay' */
  real_T UnitDelay2_DSTATE_d;          /* '<S10>/Unit Delay2' */
  real_T UnitDelay_DSTATE_m;           /* '<S10>/Unit Delay' */
  real_T UnitDelay1_DSTATE_p;          /* '<S10>/Unit Delay1' */
  real_T DelayInput2_DSTATE_n0;        /* '<S41>/Delay Input2' */
  real_T DelayInput2_DSTATE_hk;        /* '<S38>/Delay Input2' */
  real_T UnitDelay4_DSTATE;            /* '<S27>/Unit Delay4' */
  real_T UnitDelay2_DSTATE_g;          /* '<S27>/Unit Delay2' */
  real_T UnitDelay4_DSTATE_i;          /* '<S26>/Unit Delay4' */
  real_T UnitDelay4_DSTATE_l;          /* '<S25>/Unit Delay4' */
  real_T DelayInput2_DSTATE_mb;        /* '<S36>/Delay Input2' */
  real_T UnitDelay2_DSTATE_j;          /* '<S26>/Unit Delay2' */
  real_T DelayInput2_DSTATE_o;         /* '<S37>/Delay Input2' */
  real_T UnitDelay2_DSTATE_b;          /* '<S25>/Unit Delay2' */
  real_T b;                            /* '<S7>/Chart' */
  real_T DYC_flag;                     /* '<S7>/Chart' */
  real32_T UnitDelay_DSTATE_p;         /* '<S270>/Unit Delay' */
  real32_T DelayInput2_DSTATE_n2;      /* '<S274>/Delay Input2' */
  real32_T UnitDelay_DSTATE_j;         /* '<S263>/Unit Delay' */
  real32_T UnitDelay_DSTATE_pj;        /* '<S271>/Unit Delay' */
  real32_T DelayInput2_DSTATE_e;       /* '<S277>/Delay Input2' */
  real32_T UnitDelay1_DSTATE_n;        /* '<S263>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_a;         /* '<S272>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hk3;     /* '<S280>/Delay Input2' */
  real32_T UnitDelay2_DSTATE_l;        /* '<S263>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_nc;        /* '<S273>/Unit Delay' */
  real32_T DelayInput2_DSTATE_c;       /* '<S283>/Delay Input2' */
  real32_T UnitDelay3_DSTATE;          /* '<S263>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_l;         /* '<S290>/Unit Delay' */
  real32_T DelayInput2_DSTATE_i;       /* '<S294>/Delay Input2' */
  real32_T UnitDelay4_DSTATE_ik;       /* '<S264>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_a0;        /* '<S264>/Unit Delay' */
  real32_T UnitDelay_DSTATE_ap;        /* '<S291>/Unit Delay' */
  real32_T DelayInput2_DSTATE_el;      /* '<S297>/Delay Input2' */
  real32_T UnitDelay5_DSTATE;          /* '<S264>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_am;       /* '<S264>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_o;         /* '<S292>/Unit Delay' */
  real32_T DelayInput2_DSTATE_p;       /* '<S300>/Delay Input2' */
  real32_T UnitDelay6_DSTATE;          /* '<S264>/Unit Delay6' */
  real32_T UnitDelay2_DSTATE_c;        /* '<S264>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_ah;        /* '<S293>/Unit Delay' */
  real32_T DelayInput2_DSTATE_mt;      /* '<S303>/Delay Input2' */
  real32_T UnitDelay7_DSTATE;          /* '<S264>/Unit Delay7' */
  real32_T UnitDelay3_DSTATE_d;        /* '<S264>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_lh;        /* '<S192>/Unit Delay' */
  real32_T UnitDelay4_DSTATE_m;        /* '<S253>/Unit Delay4' */
  real32_T UnitDelay1_DSTATE_k;        /* '<S192>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_o;        /* '<S253>/Unit Delay1' */
  real32_T UnitDelay3_DSTATE_p;        /* '<S255>/Unit Delay3' */
  real32_T UnitDelay1_DSTATE_d;        /* '<S255>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_f;        /* '<S255>/Unit Delay2' */
  real32_T UnitDelay4_DSTATE_mn;       /* '<S255>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_d;         /* '<S253>/Unit Delay' */
  real32_T UnitDelay2_DSTATE_i;        /* '<S253>/Unit Delay2' */
  real32_T DelayInput2_DSTATE_g;       /* '<S315>/Delay Input2' */
  real32_T DelayInput2_DSTATE_af;      /* '<S313>/Delay Input2' */
  real32_T DelayInput2_DSTATE_f;       /* '<S319>/Delay Input2' */
  real32_T UnitDelay_DSTATE_ncs;       /* '<S312>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hu;      /* '<S314>/Delay Input2' */
  real32_T DelayInput2_DSTATE_d;       /* '<S42>/Delay Input2' */
  real32_T UnitDelay4_DSTATE_n;        /* '<S10>/Unit Delay4' */
  real32_T UnitDelay5_DSTATE_b;        /* '<S10>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_i;        /* '<S85>/Unit Delay1' */
  real32_T UnitDelay5_DSTATE_l;        /* '<S27>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_b;         /* '<S27>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_g;        /* '<S27>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_n5;       /* '<S74>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_h;        /* '<S65>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_cd;      /* '<S18>/Delay Input2' */
  real32_T DelayInput2_DSTATE_fo;      /* '<S40>/Delay Input2' */
  real32_T UnitDelay5_DSTATE_i;        /* '<S26>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_f;         /* '<S26>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_f;        /* '<S26>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_hn;      /* '<S19>/Delay Input2' */
  real32_T UnitDelay5_DSTATE_ip;       /* '<S25>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_nr;        /* '<S25>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_g4;       /* '<S25>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_ib;      /* '<S20>/Delay Input2' */
  uint32_T Subsystem_PREV_T;           /* '<S4>/Subsystem' */
  uint32_T FunctionCallSubsystem_PREV_T;/* '<S4>/Function-Call Subsystem' */
  uint32_T previousTicks;              /* '<S97>/Chart2' */
  uint32_T MoTrqReq_PREV_T;            /* '<S1>/MoTrqReq' */
  int_T CANPack1_ModeSignalID;         /* '<S327>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_l;       /* '<S328>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_f;       /* '<S326>/CAN Pack1' */
  int_T CANUnpack_ModeSignalID;        /* '<S171>/CAN Unpack' */
  int_T CANUnpack_StatusPortID;        /* '<S171>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_b;      /* '<S166>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_m;      /* '<S166>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_a;      /* '<S144>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_o;      /* '<S144>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_n;      /* '<S154>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_f;      /* '<S154>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_p;      /* '<S152>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_e;      /* '<S152>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_c;      /* '<S126>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_ok;     /* '<S126>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_m;      /* '<S137>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_j;      /* '<S137>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_i;      /* '<S135>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_mj;     /* '<S135>/CAN Unpack' */
  int_T CANUnpack1_ModeSignalID;       /* '<S184>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID;       /* '<S184>/CAN Unpack1' */
  int_T CANUnpack1_ModeSignalID_m;     /* '<S116>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID_n;     /* '<S116>/CAN Unpack1' */
  int_T CANUnpack_ModeSignalID_f;      /* '<S179>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_k;      /* '<S179>/CAN Unpack' */
  struct {
    uint_T is_VehStat:4;               /* '<S97>/Chart2' */
    uint_T is_AMKDCready:4;            /* '<S97>/Chart2' */
    uint_T is_BeeperStat:2;            /* '<S97>/Chart2' */
    uint_T is_AMKDCon:2;               /* '<S97>/Chart2' */
    uint_T is_MCDCEnable:2;            /* '<S97>/Chart2' */
    uint_T is_MC_TorqueCUT:2;          /* '<S97>/Chart2' */
    uint_T is_AMKCANenable:2;          /* '<S97>/Chart2' */
    uint_T is_MC_InverterOn:2;         /* '<S97>/Chart2' */
    uint_T is_B:2;                     /* '<S7>/Chart' */
    uint_T is_C:2;                     /* '<S7>/Chart' */
    uint_T is_D:2;                     /* '<S7>/Chart' */
    uint_T is_E:2;                     /* '<S7>/Chart' */
    uint_T is_active_c1_VehCtrlMdel240918_:1;/* '<S97>/Chart2' */
    uint_T is_active_VehStat:1;        /* '<S97>/Chart2' */
    uint_T is_active_BeeperStat:1;     /* '<S97>/Chart2' */
    uint_T is_active_AMKDCon:1;        /* '<S97>/Chart2' */
    uint_T is_active_MCDCEnable:1;     /* '<S97>/Chart2' */
    uint_T is_active_MC_TorqueCUT:1;   /* '<S97>/Chart2' */
    uint_T is_active_AMKDCready:1;     /* '<S97>/Chart2' */
    uint_T is_active_Output:1;         /* '<S97>/Chart2' */
    uint_T is_active_AMKCANenable:1;   /* '<S97>/Chart2' */
    uint_T is_active_MC_InverterOn:1;  /* '<S97>/Chart2' */
    uint_T is_active_c7_VehCtrlMdel240918_:1;/* '<S7>/Chart' */
  } bitsForTID3;

  uint16_T UnitDelay1_DSTATE_fm;       /* '<S193>/Unit Delay1' */
  uint16_T UnitDelay_DSTATE_k;         /* '<S193>/Unit Delay' */
  uint16_T temporalCounter_i2;         /* '<S97>/Chart2' */
  boolean_T UnitDelay3_DSTATE_f;       /* '<S253>/Unit Delay3' */
  boolean_T UnitDelay3_DSTATE_l;       /* '<S10>/Unit Delay3' */
  boolean_T UnitDelay6_DSTATE_i;       /* '<S10>/Unit Delay6' */
  boolean_T UnitDelay1_DSTATE_e;       /* '<S84>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_a;       /* '<S27>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE;        /* '<S83>/Delay Input1' */
  boolean_T UnitDelay1_DSTATE_gl;      /* '<S73>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_e;       /* '<S26>/Unit Delay3' */
  boolean_T UnitDelay1_DSTATE_dp;      /* '<S64>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_i;       /* '<S25>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE_j;      /* '<S72>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_b;      /* '<S63>/Delay Input1' */
  uint8_T temporalCounter_i1;          /* '<S97>/Chart2' */
  boolean_T Subsystem_RESET_ELAPS_T;   /* '<S4>/Subsystem' */
  boolean_T FunctionCallSubsystem_RESET_ELA;/* '<S4>/Function-Call Subsystem' */
  boolean_T MoTrqReq_RESET_ELAPS_T;    /* '<S1>/MoTrqReq' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer2_j;/* '<S255>/Timer2' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer1_p;/* '<S255>/Timer1' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer3_i;/* '<S254>/Timer3' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer2_g;/* '<S254>/Timer2' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer1_m;/* '<S254>/Timer1' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer_o;/* '<S254>/Timer' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer3;/* '<S196>/Timer3' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer2_l;/* '<S196>/Timer2' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer1_n;/* '<S196>/Timer1' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer_b;/* '<S196>/Timer' */
  DW_Timer1_VehCtrlMdel240918_2_T sf_Timer_k;/* '<S195>/Timer' */
  DW_Timer_VehCtrlMdel240918_20_T sf_Timer_a;/* '<S193>/Timer' */
  DW_Timer_VehCtrlMdel240918_20_T sf_Timer;/* '<S114>/Timer' */
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
   * Referenced by: '<S195>/1-D Lookup Table'
   */
  real_T uDLookupTable_tableData[24];

  /* Computed Parameter: uDLookupTable_bp01Data
   * Referenced by: '<S195>/1-D Lookup Table'
   */
  real_T uDLookupTable_bp01Data[24];

  /* Computed Parameter: uDLookupTable1_tableData
   * Referenced by: '<S195>/1-D Lookup Table1'
   */
  real_T uDLookupTable1_tableData[24];

  /* Computed Parameter: uDLookupTable1_bp01Data
   * Referenced by: '<S195>/1-D Lookup Table1'
   */
  real_T uDLookupTable1_bp01Data[24];

  /* Pooled Parameter (Expression: single([20,0]);)
   * Referenced by:
   *   '<S7>/BrakeCompensateCoefFront1'
   *   '<S7>/BrakeCompensateCoefFront2'
   */
  real32_T pooled31[2];

  /* Pooled Parameter (Expression: single([550,1700]);)
   * Referenced by:
   *   '<S7>/BrakeCompensateCoefFront1'
   *   '<S7>/BrakeCompensateCoefFront2'
   */
  real32_T pooled32[2];

  /* Expression: single([1000,0]);
   * Referenced by: '<S7>/BrakeCompensateCoefRear'
   */
  real32_T BrakeCompensateCoefRear_tableDa[2];

  /* Expression: single([550,1500]);
   * Referenced by: '<S7>/BrakeCompensateCoefRear'
   */
  real32_T BrakeCompensateCoefRear_bp01Dat[2];

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
   *   '<S25>/VehSpd_SlipTarget_mps'
   *   '<S26>/VehSpd_SlipTarget_mps'
   *   '<S27>/VehSpd_SlipTarget_mps'
   */
  real32_T pooled54[4];

  /* Pooled Parameter (Expression: [0,3,25,30])
   * Referenced by:
   *   '<S25>/VehSpd_SlipTarget_mps'
   *   '<S25>/VehicleStableTarget_mps'
   *   '<S25>/VehicleStableTarget_mps1'
   *   '<S26>/VehSpd_SlipTarget_mps'
   *   '<S26>/VehicleStableTarget_mps'
   *   '<S26>/VehicleStableTarget_mps1'
   *   '<S27>/VehSpd_SlipTarget_mps'
   *   '<S27>/VehicleStableTarget_mps'
   *   '<S27>/VehicleStableTarget_mps1'
   */
  real32_T pooled55[4];

  /* Pooled Parameter (Expression: [0.4,0.4,1.2,1.2])
   * Referenced by:
   *   '<S25>/VehicleStableTarget_mps'
   *   '<S25>/VehicleStableTarget_mps1'
   *   '<S26>/VehicleStableTarget_mps'
   *   '<S26>/VehicleStableTarget_mps1'
   *   '<S27>/VehicleStableTarget_mps'
   *   '<S27>/VehicleStableTarget_mps1'
   */
  real32_T pooled61[4];

  /* Pooled Parameter (Expression: single([500,4500]);)
   * Referenced by: '<S193>/1-D Lookup Table1'
   */
  real32_T pooled65[2];

  /* Expression: single([0,100]);
   * Referenced by: '<S193>/1-D Lookup Table3'
   */
  real32_T uDLookupTable3_tableData[2];

  /* Expression: single([3540,3790])
   * Referenced by: '<S193>/1-D Lookup Table3'
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
extern real_T Gear_Trs;                /* '<S328>/Switch2' */
extern real_T Mode_Trs;                /* '<S328>/Switch3' */
extern real_T Trq_CUT;                 /* '<S193>/Timer' */
extern real_T ignition;                /* '<S114>/Timer' */
extern real_T L12V_error;              /* '<S171>/CAN Unpack' */
extern real_T alarm;                   /* '<S171>/CAN Unpack' */
extern real_T controller_ready;        /* '<S171>/CAN Unpack' */
extern real_T selfcheck;               /* '<S171>/CAN Unpack' */
extern real_T RPM;                     /* '<S171>/CAN Unpack' */
extern real_T trq;                     /* '<S171>/CAN Unpack' */
extern real_T AC_current;              /* '<S166>/CAN Unpack' */
extern real_T DC_current;              /* '<S166>/CAN Unpack' */
extern real_T MCU_Temp;                /* '<S166>/CAN Unpack' */
extern real_T motor_Temp;              /* '<S166>/CAN Unpack' */
extern real_T MCFR_ActualVelocity;     /* '<S144>/CAN Unpack' */
extern real_T MCFR_DCVoltage;          /* '<S144>/CAN Unpack' */
extern real_T MCFR_bDCOn;              /* '<S144>/CAN Unpack' */
extern real_T MCFR_bError;             /* '<S144>/CAN Unpack' */
extern real_T MCFR_bInverterOn;        /* '<S144>/CAN Unpack' */
extern real_T MCFR_bQuitInverterOn;    /* '<S144>/CAN Unpack' */
extern real_T MCFR_bSystemReady;       /* '<S144>/CAN Unpack' */
extern real_T MCFR_TempIGBT;           /* '<S154>/CAN Unpack' */
extern real_T MCFR_TempInverter;       /* '<S154>/CAN Unpack' */
extern real_T MCFR_TempMotor;          /* '<S154>/CAN Unpack' */
extern real_T MCFR_ErrorInfo;          /* '<S152>/CAN Unpack' */
extern real_T MCFL_DCVoltage;          /* '<S126>/CAN Unpack' */
extern real_T MCFL_bDCOn;              /* '<S126>/CAN Unpack' */
extern real_T MCFL_bError;             /* '<S126>/CAN Unpack' */
extern real_T MCFL_bInverterOn;        /* '<S126>/CAN Unpack' */
extern real_T MCFL_bQuitDCOn;          /* '<S126>/CAN Unpack' */
extern real_T MCFL_bQuitInverterOn;    /* '<S126>/CAN Unpack' */
extern real_T MCFL_bSystemReady;       /* '<S126>/CAN Unpack' */
extern real_T MCFL_ActualVelocity;     /* '<S126>/Gain' */
extern real_T MCFL_TempIGBT;           /* '<S137>/CAN Unpack' */
extern real_T MCFL_TempInverter;       /* '<S137>/CAN Unpack' */
extern real_T MCFL_TempMotor;          /* '<S137>/CAN Unpack' */
extern real_T MCFL_ErrorInfo;          /* '<S135>/CAN Unpack' */
extern real_T StrWhlAngAliveRollCnt;   /* '<S184>/CAN Unpack1' */
extern real_T StrWhlAng;               /* '<S184>/CAN Unpack1' */
extern real_T StrWhlAngV;              /* '<S184>/CAN Unpack1' */
extern real_T ABS_WS_FL;               /* '<S116>/CAN Unpack1' */
extern real_T ABS_WS_FR;               /* '<S116>/CAN Unpack1' */
extern real_T ABS_WS_RL;               /* '<S116>/CAN Unpack1' */
extern real_T ABS_WS_RR;               /* '<S116>/CAN Unpack1' */
extern real_T IMU_Ay_Value;            /* '<S179>/CAN Unpack' */
extern real_T IMU_Ax_Value;            /* '<S179>/CAN Unpack' */
extern real_T IMU_Yaw_Value;           /* '<S179>/CAN Unpack' */
extern real_T EMRAX_Trq_CUT;           /*  */
extern real_T AMK_Trq_CUT;             /*  */
extern uint32_T Acc_vol2;              /* '<S193>/Add3' */
extern uint32_T Acc_vol;               /* '<S193>/Add2' */
extern uint32_T Acc_POS2;              /* '<S193>/1-D Lookup Table3' */
extern real32_T Acc_POS;               /* '<S193>/MATLAB Function' */
extern real32_T TrqR_cmd;              /* '<S7>/Saturation1' */
extern real32_T TrqFR_cmd;             /* '<S7>/Saturation2' */
extern real32_T TrqFL_cmd;             /* '<S7>/Saturation3' */
extern uint16_T F_BrkPrs;              /* '<S193>/1-D Lookup Table1' */
extern uint16_T Acc1;                  /* '<S109>/Acc3' */
extern uint16_T Acc2;                  /* '<S109>/Acc4' */
extern uint16_T Brk1;                  /* '<S109>/Brk1' */
extern uint16_T Brk2;                  /* '<S109>/Brk2' */
extern boolean_T HVCUTOFF;             /* '<S114>/Constant' */
extern boolean_T KeyPressed;           /* '<S97>/Cast To Boolean' */
extern boolean_T Brk;                  /* '<S99>/Compare' */
extern boolean_T ACC_Release;          /* '<S100>/Compare' */
extern boolean_T beeper_state;         /* '<S97>/Chart2' */
extern boolean_T MCFL_DCOn_setpoints;  /* '<S97>/Chart2' */
extern boolean_T MCFR_DCEnable;        /* '<S97>/Chart2' */
extern boolean_T MCFR_InverterOn;      /* '<S97>/Chart2' */
extern boolean_T TroqueOn;             /* '<S7>/Logical Operator6' */
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
 * Block '<S36>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S51>/Data Type Duplicate' : Unused code path elimination
 * Block '<S51>/Data Type Propagation' : Unused code path elimination
 * Block '<S37>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S52>/Data Type Duplicate' : Unused code path elimination
 * Block '<S52>/Data Type Propagation' : Unused code path elimination
 * Block '<S38>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S53>/Data Type Duplicate' : Unused code path elimination
 * Block '<S53>/Data Type Propagation' : Unused code path elimination
 * Block '<S39>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S54>/Data Type Duplicate' : Unused code path elimination
 * Block '<S54>/Data Type Propagation' : Unused code path elimination
 * Block '<S40>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S55>/Data Type Duplicate' : Unused code path elimination
 * Block '<S55>/Data Type Propagation' : Unused code path elimination
 * Block '<S41>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S56>/Data Type Duplicate' : Unused code path elimination
 * Block '<S56>/Data Type Propagation' : Unused code path elimination
 * Block '<S42>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S57>/Data Type Duplicate' : Unused code path elimination
 * Block '<S57>/Data Type Propagation' : Unused code path elimination
 * Block '<S43>/Data Type Duplicate' : Unused code path elimination
 * Block '<S43>/Data Type Propagation' : Unused code path elimination
 * Block '<S44>/Data Type Duplicate' : Unused code path elimination
 * Block '<S44>/Data Type Propagation' : Unused code path elimination
 * Block '<S45>/Data Type Duplicate' : Unused code path elimination
 * Block '<S45>/Data Type Propagation' : Unused code path elimination
 * Block '<S46>/Data Type Duplicate' : Unused code path elimination
 * Block '<S46>/Data Type Propagation' : Unused code path elimination
 * Block '<S47>/Data Type Duplicate' : Unused code path elimination
 * Block '<S47>/Data Type Propagation' : Unused code path elimination
 * Block '<S48>/Data Type Duplicate' : Unused code path elimination
 * Block '<S48>/Data Type Propagation' : Unused code path elimination
 * Block '<S18>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S58>/Data Type Duplicate' : Unused code path elimination
 * Block '<S58>/Data Type Propagation' : Unused code path elimination
 * Block '<S19>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S59>/Data Type Duplicate' : Unused code path elimination
 * Block '<S59>/Data Type Propagation' : Unused code path elimination
 * Block '<S20>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Propagation' : Unused code path elimination
 * Block '<S21>/Data Type Duplicate' : Unused code path elimination
 * Block '<S21>/Data Type Propagation' : Unused code path elimination
 * Block '<S22>/Data Type Duplicate' : Unused code path elimination
 * Block '<S22>/Data Type Propagation' : Unused code path elimination
 * Block '<S23>/Data Type Duplicate' : Unused code path elimination
 * Block '<S23>/Data Type Propagation' : Unused code path elimination
 * Block '<S66>/Data Type Duplicate' : Unused code path elimination
 * Block '<S66>/Data Type Propagation' : Unused code path elimination
 * Block '<S67>/Data Type Duplicate' : Unused code path elimination
 * Block '<S67>/Data Type Propagation' : Unused code path elimination
 * Block '<S68>/Data Type Duplicate' : Unused code path elimination
 * Block '<S68>/Data Type Propagation' : Unused code path elimination
 * Block '<S75>/Data Type Duplicate' : Unused code path elimination
 * Block '<S75>/Data Type Propagation' : Unused code path elimination
 * Block '<S76>/Data Type Duplicate' : Unused code path elimination
 * Block '<S76>/Data Type Propagation' : Unused code path elimination
 * Block '<S77>/Data Type Duplicate' : Unused code path elimination
 * Block '<S77>/Data Type Propagation' : Unused code path elimination
 * Block '<S86>/Data Type Duplicate' : Unused code path elimination
 * Block '<S86>/Data Type Propagation' : Unused code path elimination
 * Block '<S87>/Data Type Duplicate' : Unused code path elimination
 * Block '<S87>/Data Type Propagation' : Unused code path elimination
 * Block '<S88>/Data Type Duplicate' : Unused code path elimination
 * Block '<S88>/Data Type Propagation' : Unused code path elimination
 * Block '<S163>/CAN Unpack1' : Unused code path elimination
 * Block '<S193>/1-D Lookup Table2' : Unused code path elimination
 * Block '<S193>/Abs1' : Unused code path elimination
 * Block '<S193>/Add4' : Unused code path elimination
 * Block '<S199>/Compare' : Unused code path elimination
 * Block '<S199>/Constant' : Unused code path elimination
 * Block '<S200>/Compare' : Unused code path elimination
 * Block '<S200>/Constant' : Unused code path elimination
 * Block '<S208>/Compare' : Unused code path elimination
 * Block '<S208>/Constant' : Unused code path elimination
 * Block '<S193>/Logical Operator6' : Unused code path elimination
 * Block '<S193>/Logical Operator7' : Unused code path elimination
 * Block '<S218>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S224>/Data Type Duplicate' : Unused code path elimination
 * Block '<S224>/Data Type Propagation' : Unused code path elimination
 * Block '<S219>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S225>/Data Type Duplicate' : Unused code path elimination
 * Block '<S225>/Data Type Propagation' : Unused code path elimination
 * Block '<S220>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S226>/Data Type Duplicate' : Unused code path elimination
 * Block '<S226>/Data Type Propagation' : Unused code path elimination
 * Block '<S221>/Data Type Duplicate' : Unused code path elimination
 * Block '<S221>/Data Type Propagation' : Unused code path elimination
 * Block '<S222>/Data Type Duplicate' : Unused code path elimination
 * Block '<S222>/Data Type Propagation' : Unused code path elimination
 * Block '<S223>/Data Type Duplicate' : Unused code path elimination
 * Block '<S223>/Data Type Propagation' : Unused code path elimination
 * Block '<S228>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S231>/Data Type Duplicate' : Unused code path elimination
 * Block '<S231>/Data Type Propagation' : Unused code path elimination
 * Block '<S229>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S232>/Data Type Duplicate' : Unused code path elimination
 * Block '<S232>/Data Type Propagation' : Unused code path elimination
 * Block '<S241>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S249>/Data Type Duplicate' : Unused code path elimination
 * Block '<S249>/Data Type Propagation' : Unused code path elimination
 * Block '<S242>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S250>/Data Type Duplicate' : Unused code path elimination
 * Block '<S250>/Data Type Propagation' : Unused code path elimination
 * Block '<S243>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S251>/Data Type Duplicate' : Unused code path elimination
 * Block '<S251>/Data Type Propagation' : Unused code path elimination
 * Block '<S244>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S252>/Data Type Duplicate' : Unused code path elimination
 * Block '<S252>/Data Type Propagation' : Unused code path elimination
 * Block '<S274>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S276>/Data Type Duplicate' : Unused code path elimination
 * Block '<S276>/Data Type Propagation' : Unused code path elimination
 * Block '<S275>/Data Type Duplicate' : Unused code path elimination
 * Block '<S275>/Data Type Propagation' : Unused code path elimination
 * Block '<S277>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S279>/Data Type Duplicate' : Unused code path elimination
 * Block '<S279>/Data Type Propagation' : Unused code path elimination
 * Block '<S278>/Data Type Duplicate' : Unused code path elimination
 * Block '<S278>/Data Type Propagation' : Unused code path elimination
 * Block '<S280>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S282>/Data Type Duplicate' : Unused code path elimination
 * Block '<S282>/Data Type Propagation' : Unused code path elimination
 * Block '<S281>/Data Type Duplicate' : Unused code path elimination
 * Block '<S281>/Data Type Propagation' : Unused code path elimination
 * Block '<S283>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S285>/Data Type Duplicate' : Unused code path elimination
 * Block '<S285>/Data Type Propagation' : Unused code path elimination
 * Block '<S284>/Data Type Duplicate' : Unused code path elimination
 * Block '<S284>/Data Type Propagation' : Unused code path elimination
 * Block '<S294>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S296>/Data Type Duplicate' : Unused code path elimination
 * Block '<S296>/Data Type Propagation' : Unused code path elimination
 * Block '<S295>/Data Type Duplicate' : Unused code path elimination
 * Block '<S295>/Data Type Propagation' : Unused code path elimination
 * Block '<S297>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S299>/Data Type Duplicate' : Unused code path elimination
 * Block '<S299>/Data Type Propagation' : Unused code path elimination
 * Block '<S298>/Data Type Duplicate' : Unused code path elimination
 * Block '<S298>/Data Type Propagation' : Unused code path elimination
 * Block '<S300>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S302>/Data Type Duplicate' : Unused code path elimination
 * Block '<S302>/Data Type Propagation' : Unused code path elimination
 * Block '<S301>/Data Type Duplicate' : Unused code path elimination
 * Block '<S301>/Data Type Propagation' : Unused code path elimination
 * Block '<S303>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S305>/Data Type Duplicate' : Unused code path elimination
 * Block '<S305>/Data Type Propagation' : Unused code path elimination
 * Block '<S304>/Data Type Duplicate' : Unused code path elimination
 * Block '<S304>/Data Type Propagation' : Unused code path elimination
 * Block '<S255>/Discrete-Time Integrator' : Unused code path elimination
 * Block '<S319>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S321>/Data Type Duplicate' : Unused code path elimination
 * Block '<S321>/Data Type Propagation' : Unused code path elimination
 * Block '<S320>/Data Type Duplicate' : Unused code path elimination
 * Block '<S320>/Data Type Propagation' : Unused code path elimination
 * Block '<S313>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S322>/Data Type Duplicate' : Unused code path elimination
 * Block '<S322>/Data Type Propagation' : Unused code path elimination
 * Block '<S314>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S323>/Data Type Duplicate' : Unused code path elimination
 * Block '<S323>/Data Type Propagation' : Unused code path elimination
 * Block '<S315>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S324>/Data Type Duplicate' : Unused code path elimination
 * Block '<S324>/Data Type Propagation' : Unused code path elimination
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
 * Block '<S193>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S253>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S253>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S254>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S254>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S254>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S254>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S312>/Gain1' : Eliminated nontunable gain of 1
 * Block '<S328>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S329>/Cast To Single' : Eliminate redundant data type conversion
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
 * '<S12>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/MeaModule1'
 * '<S13>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/MeaModule2'
 * '<S14>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/MeaModule3'
 * '<S15>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/MeaModule4'
 * '<S16>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/MeaModule5'
 * '<S17>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/MeaModule6'
 * '<S18>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic1'
 * '<S19>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic2'
 * '<S20>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic3'
 * '<S21>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Saturation Dynamic1'
 * '<S22>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Saturation Dynamic2'
 * '<S23>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Saturation Dynamic3'
 * '<S24>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Subsystem'
 * '<S25>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL'
 * '<S26>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR'
 * '<S27>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R'
 * '<S28>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Compare To Constant'
 * '<S29>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Compare To Constant1'
 * '<S30>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Compare To Constant2'
 * '<S31>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Compare To Constant3'
 * '<S32>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Compare To Constant4'
 * '<S33>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Compare To Constant5'
 * '<S34>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/MATLAB Function'
 * '<S35>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/MATLAB Function1'
 * '<S36>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic'
 * '<S37>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic1'
 * '<S38>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic2'
 * '<S39>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic3'
 * '<S40>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic4'
 * '<S41>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic5'
 * '<S42>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic6'
 * '<S43>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic'
 * '<S44>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic1'
 * '<S45>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic2'
 * '<S46>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic3'
 * '<S47>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic4'
 * '<S48>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Saturation Dynamic5'
 * '<S49>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Wtarget'
 * '<S50>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/载荷转移'
 * '<S51>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S52>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S53>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S54>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S55>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic4/Saturation Dynamic'
 * '<S56>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic5/Saturation Dynamic'
 * '<S57>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic6/Saturation Dynamic'
 * '<S58>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S59>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S60>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S61>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Compare To Constant'
 * '<S62>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Compare To Constant2'
 * '<S63>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Detect Rise Positive'
 * '<S64>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Latch_on'
 * '<S65>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/RisingTimer'
 * '<S66>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic'
 * '<S67>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic1'
 * '<S68>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic2'
 * '<S69>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Detect Rise Positive/Positive'
 * '<S70>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Compare To Constant'
 * '<S71>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Compare To Constant2'
 * '<S72>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Detect Rise Positive'
 * '<S73>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Latch_on'
 * '<S74>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/RisingTimer'
 * '<S75>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic'
 * '<S76>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic1'
 * '<S77>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic2'
 * '<S78>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Detect Rise Positive/Positive'
 * '<S79>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant'
 * '<S80>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant1'
 * '<S81>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant2'
 * '<S82>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Zero'
 * '<S83>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Detect Rise Positive'
 * '<S84>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Latch_on'
 * '<S85>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/RisingTimer'
 * '<S86>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic'
 * '<S87>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic1'
 * '<S88>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic2'
 * '<S89>'  : 'VehCtrlMdel240918_2018b/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Detect Rise Positive/Positive'
 * '<S90>'  : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant'
 * '<S91>'  : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant1'
 * '<S92>'  : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant2'
 * '<S93>'  : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant3'
 * '<S94>'  : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct/Compare To Constant4'
 * '<S95>'  : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct/Timer1'
 * '<S96>'  : 'VehCtrlMdel240918_2018b/CTRL/PwrTrainTempPrtct/Timer2'
 * '<S97>'  : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem'
 * '<S98>'  : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/Chart2'
 * '<S99>'  : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/Compare To Constant'
 * '<S100>' : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/Compare To Constant1'
 * '<S101>' : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/MeaModule'
 * '<S102>' : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/MeaModule1'
 * '<S103>' : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/MeaModule2'
 * '<S104>' : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/MeaModule3'
 * '<S105>' : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/MeaModule4'
 * '<S106>' : 'VehCtrlMdel240918_2018b/Drive_Ready/Subsystem/MeaModule5'
 * '<S107>' : 'VehCtrlMdel240918_2018b/Input/ABS_Receive'
 * '<S108>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive'
 * '<S109>' : 'VehCtrlMdel240918_2018b/Input/AccBrk_BUS'
 * '<S110>' : 'VehCtrlMdel240918_2018b/Input/BMS_Recive'
 * '<S111>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE'
 * '<S112>' : 'VehCtrlMdel240918_2018b/Input/IMU_Recieve'
 * '<S113>' : 'VehCtrlMdel240918_2018b/Input/StrSnis_Receive'
 * '<S114>' : 'VehCtrlMdel240918_2018b/Input/key'
 * '<S115>' : 'VehCtrlMdel240918_2018b/Input/ABS_Receive/ABS_BUS_state'
 * '<S116>' : 'VehCtrlMdel240918_2018b/Input/ABS_Receive/ABS_BUS_state/IMU_state'
 * '<S117>' : 'VehCtrlMdel240918_2018b/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule1'
 * '<S118>' : 'VehCtrlMdel240918_2018b/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule2'
 * '<S119>' : 'VehCtrlMdel240918_2018b/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule3'
 * '<S120>' : 'VehCtrlMdel240918_2018b/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule4'
 * '<S121>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU'
 * '<S122>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU'
 * '<S123>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state'
 * '<S124>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1'
 * '<S125>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2'
 * '<S126>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state'
 * '<S127>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule1'
 * '<S128>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule2'
 * '<S129>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule3'
 * '<S130>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule4'
 * '<S131>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule5'
 * '<S132>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule6'
 * '<S133>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule7'
 * '<S134>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule8'
 * '<S135>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state'
 * '<S136>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state/MeaModule1'
 * '<S137>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state'
 * '<S138>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule1'
 * '<S139>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule2'
 * '<S140>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule3'
 * '<S141>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state'
 * '<S142>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1'
 * '<S143>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2'
 * '<S144>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state'
 * '<S145>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule1'
 * '<S146>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule2'
 * '<S147>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule3'
 * '<S148>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule4'
 * '<S149>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule5'
 * '<S150>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule6'
 * '<S151>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule7'
 * '<S152>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state'
 * '<S153>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state/MeaModule1'
 * '<S154>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state'
 * '<S155>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule1'
 * '<S156>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule2'
 * '<S157>' : 'VehCtrlMdel240918_2018b/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule3'
 * '<S158>' : 'VehCtrlMdel240918_2018b/Input/AccBrk_BUS/MeaModule1'
 * '<S159>' : 'VehCtrlMdel240918_2018b/Input/AccBrk_BUS/MeaModule2'
 * '<S160>' : 'VehCtrlMdel240918_2018b/Input/AccBrk_BUS/MeaModule3'
 * '<S161>' : 'VehCtrlMdel240918_2018b/Input/AccBrk_BUS/MeaModule4'
 * '<S162>' : 'VehCtrlMdel240918_2018b/Input/BMS_Recive/ABS_BUS_state'
 * '<S163>' : 'VehCtrlMdel240918_2018b/Input/BMS_Recive/ABS_BUS_state/IMU_state'
 * '<S164>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr'
 * '<S165>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state'
 * '<S166>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1'
 * '<S167>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule1'
 * '<S168>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule2'
 * '<S169>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule3'
 * '<S170>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule4'
 * '<S171>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state'
 * '<S172>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule'
 * '<S173>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule1'
 * '<S174>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule2'
 * '<S175>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule3'
 * '<S176>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule5'
 * '<S177>' : 'VehCtrlMdel240918_2018b/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule6'
 * '<S178>' : 'VehCtrlMdel240918_2018b/Input/IMU_Recieve/IMU_state'
 * '<S179>' : 'VehCtrlMdel240918_2018b/Input/IMU_Recieve/IMU_state/MCU_state'
 * '<S180>' : 'VehCtrlMdel240918_2018b/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule2'
 * '<S181>' : 'VehCtrlMdel240918_2018b/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule3'
 * '<S182>' : 'VehCtrlMdel240918_2018b/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule4'
 * '<S183>' : 'VehCtrlMdel240918_2018b/Input/StrSnis_Receive/StrWhSnis_state'
 * '<S184>' : 'VehCtrlMdel240918_2018b/Input/StrSnis_Receive/StrWhSnis_state/IMU_state'
 * '<S185>' : 'VehCtrlMdel240918_2018b/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule1'
 * '<S186>' : 'VehCtrlMdel240918_2018b/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule2'
 * '<S187>' : 'VehCtrlMdel240918_2018b/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule3'
 * '<S188>' : 'VehCtrlMdel240918_2018b/Input/key/MeaModule'
 * '<S189>' : 'VehCtrlMdel240918_2018b/Input/key/MeaModule1'
 * '<S190>' : 'VehCtrlMdel240918_2018b/Input/key/Timer'
 * '<S191>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem'
 * '<S192>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem'
 * '<S193>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal'
 * '<S194>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU'
 * '<S195>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/SWAS'
 * '<S196>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps'
 * '<S197>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant'
 * '<S198>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant1'
 * '<S199>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant10'
 * '<S200>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant11'
 * '<S201>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant2'
 * '<S202>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant3'
 * '<S203>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant4'
 * '<S204>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant5'
 * '<S205>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant6'
 * '<S206>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant7'
 * '<S207>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant8'
 * '<S208>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant9'
 * '<S209>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MATLAB Function'
 * '<S210>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule1'
 * '<S211>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule2'
 * '<S212>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule3'
 * '<S213>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule5'
 * '<S214>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule7'
 * '<S215>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule8'
 * '<S216>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/AccBrkPedal/Timer'
 * '<S217>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem'
 * '<S218>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic'
 * '<S219>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1'
 * '<S220>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2'
 * '<S221>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic'
 * '<S222>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic1'
 * '<S223>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic2'
 * '<S224>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S225>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S226>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S227>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/SWAS/Compare To Constant'
 * '<S228>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic'
 * '<S229>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1'
 * '<S230>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/SWAS/Timer'
 * '<S231>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S232>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S233>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant'
 * '<S234>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant1'
 * '<S235>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant2'
 * '<S236>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant3'
 * '<S237>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant4'
 * '<S238>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant5'
 * '<S239>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant6'
 * '<S240>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant7'
 * '<S241>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic'
 * '<S242>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1'
 * '<S243>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2'
 * '<S244>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3'
 * '<S245>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer'
 * '<S246>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer1'
 * '<S247>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer2'
 * '<S248>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer3'
 * '<S249>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S250>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S251>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S252>' : 'VehCtrlMdel240918_2018b/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S253>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/IntglJudgment '
 * '<S254>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment'
 * '<S255>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect'
 * '<S256>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/坐标系转换'
 * '<S257>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/IntglJudgment /Compare To Constant'
 * '<S258>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/IntglJudgment /Compare To Constant1'
 * '<S259>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/Timer'
 * '<S260>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/Timer1'
 * '<S261>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/Timer2'
 * '<S262>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/Timer3'
 * '<S263>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断'
 * '<S264>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断'
 * '<S265>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断'
 * '<S266>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant'
 * '<S267>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant1'
 * '<S268>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant2'
 * '<S269>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant3'
 * '<S270>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter'
 * '<S271>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1'
 * '<S272>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2'
 * '<S273>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3'
 * '<S274>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic'
 * '<S275>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Saturation Dynamic'
 * '<S276>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S277>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic'
 * '<S278>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Saturation Dynamic'
 * '<S279>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S280>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic'
 * '<S281>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Saturation Dynamic'
 * '<S282>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S283>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic'
 * '<S284>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Saturation Dynamic'
 * '<S285>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S286>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant'
 * '<S287>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant1'
 * '<S288>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant2'
 * '<S289>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant3'
 * '<S290>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter'
 * '<S291>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1'
 * '<S292>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2'
 * '<S293>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3'
 * '<S294>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic'
 * '<S295>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Saturation Dynamic'
 * '<S296>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S297>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic'
 * '<S298>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Saturation Dynamic'
 * '<S299>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S300>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic'
 * '<S301>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Saturation Dynamic'
 * '<S302>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S303>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic'
 * '<S304>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Saturation Dynamic'
 * '<S305>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S306>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant'
 * '<S307>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant1'
 * '<S308>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant2'
 * '<S309>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant3'
 * '<S310>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant'
 * '<S311>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant1'
 * '<S312>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Filter'
 * '<S313>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic'
 * '<S314>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1'
 * '<S315>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2'
 * '<S316>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Subsystem'
 * '<S317>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Timer1'
 * '<S318>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Timer2'
 * '<S319>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic'
 * '<S320>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Filter/Saturation Dynamic'
 * '<S321>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S322>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S323>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S324>' : 'VehCtrlMdel240918_2018b/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S325>' : 'VehCtrlMdel240918_2018b/OUTPUT/Beeper'
 * '<S326>' : 'VehCtrlMdel240918_2018b/OUTPUT/VCU2AMKMCUFL'
 * '<S327>' : 'VehCtrlMdel240918_2018b/OUTPUT/VCU2AMKMCUFR'
 * '<S328>' : 'VehCtrlMdel240918_2018b/OUTPUT/VCU2EmraxMCU'
 * '<S329>' : 'VehCtrlMdel240918_2018b/OUTPUT/WP_OUTPUT'
 * '<S330>' : 'VehCtrlMdel240918_2018b/OUTPUT/Beeper/MeaModule'
 * '<S331>' : 'VehCtrlMdel240918_2018b/OUTPUT/VCU2EmraxMCU/MeaModule1'
 * '<S332>' : 'VehCtrlMdel240918_2018b/OUTPUT/VCU2EmraxMCU/MeaModule2'
 * '<S333>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL'
 * '<S334>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/DAQ'
 * '<S335>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/EEPROM'
 * '<S336>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/Polling'
 * '<S337>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem'
 * '<S338>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem'
 * '<S339>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem'
 * '<S340>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com0'
 * '<S341>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com1'
 * '<S342>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com2'
 * '<S343>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem'
 * '<S344>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem1'
 * '<S345>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/DAQ/daq100ms'
 * '<S346>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/DAQ/daq10ms'
 * '<S347>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/DAQ/daq500ms'
 * '<S348>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/DAQ/daq50ms'
 * '<S349>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/DAQ/daq5ms'
 * '<S350>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/EEPROM/EEPROMOperation'
 * '<S351>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/Polling/CCPBackground'
 * '<S352>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/Polling/CCPReceive'
 * '<S353>' : 'VehCtrlMdel240918_2018b/RapidECUSetting/Polling/CCPReceive/Nothing'
 */
#endif                               /* RTW_HEADER_VehCtrlMdel240918_2018b_h_ */

/* File trailer for ECUCoder generated file VehCtrlMdel240918_2018b.h.
 *
 * [EOF]
 */
