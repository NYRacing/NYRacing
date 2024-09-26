/*
 * Code generated for Simulink model VehCtrlMdel240926_2018b_amkspdlimit.
 *
 * FILE    : VehCtrlMdel240926_2018b_amkspdlimit.h
 *
 * VERSION : 1.175
 *
 * DATE    : Thu Sep 26 15:36:01 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#ifndef RTW_HEADER_VehCtrlMdel240926_2018b_amkspdlimit_h_
#define RTW_HEADER_VehCtrlMdel240926_2018b_amkspdlimit_h_
#include <math.h>
#include "MPC5744P.h"
#include "Std_Types.h"
#include "can.h"
#include "flash.h"
#include "crc.h"
#ifndef VehCtrlMdel240926_2018b_amkspdlimit_COMMON_INCLUDES_
# define VehCtrlMdel240926_2018b_amkspdlimit_COMMON_INCLUDES_
#include <string.h>
#include <math.h>
#include "rtwtypes.h"
#include "can_message.h"
#endif                /* VehCtrlMdel240926_2018b_amkspdlimit_COMMON_INCLUDES_ */

#include "VehCtrlMdel240926_2018b_amkspdlimit_types.h"
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

#define VehCtrlMdel240926_2018b_amkspdlimit_M (VehCtrlMdel240926_2018b_amks_M)
#define EnableInterrupts()             asm(" wrteei 1")
#define DisableInterrupts()            asm(" wrteei 0")

/* user code (top of export header file) */
#include "can_message.h"

/* Block states (default storage) for system '<S8>/Timer1' */
typedef struct {
  real_T x;                            /* '<S8>/Timer1' */
  struct {
    uint_T is_c5_VehCtrlMdel240926_2018b_a:2;/* '<S8>/Timer1' */
    uint_T is_active_c5_VehCtrlMdel240926_:1;/* '<S8>/Timer1' */
  } bitsForTID3;
} DW_Timer1_VehCtrlMdel240926_2_T;

/* Block states (default storage) for system '<S123>/Timer' */
typedef struct {
  real_T x;                            /* '<S123>/Timer' */
  struct {
    uint_T is_c21_VehCtrlMdel240926_2018b_:2;/* '<S123>/Timer' */
    uint_T is_active_c21_VehCtrlMdel240926:1;/* '<S123>/Timer' */
  } bitsForTID3;
} DW_Timer_VehCtrlMdel240926_20_T;

/* Block signals (default storage) */
typedef struct {
  CAN_DATATYPE CANPack1;               /* '<S336>/CAN Pack1' */
  CAN_DATATYPE CANPack1_a;             /* '<S337>/CAN Pack1' */
  CAN_DATATYPE CANPack1_d;             /* '<S335>/CAN Pack1' */
  CAN_DATATYPE CANUnPackMessage4;      /* '<S180>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_g;    /* '<S175>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_j;    /* '<S153>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_n;    /* '<S163>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_p;    /* '<S161>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_e;    /* '<S135>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_c;    /* '<S146>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_b;    /* '<S144>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_l;    /* '<S193>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_ja;   /* '<S125>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_h;    /* '<S188>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_i;    /* '<S172>/CANUnPackMessage4' */
  real_T MCFR_TorqueLimitP;            /* '<S336>/Switch' */
  real_T MCFL_TorqueLimitP;            /* '<S335>/Switch' */
  real_T Exit;                         /* '<S264>/Timer2' */
  real_T Exit_l;                       /* '<S264>/Timer1' */
  real_T Exit_a;                       /* '<S263>/Timer3' */
  real_T Exit_lh;                      /* '<S263>/Timer2' */
  real_T Exit_lh4;                     /* '<S263>/Timer1' */
  real_T Exit_c;                       /* '<S263>/Timer' */
  real_T Exit_h;                       /* '<S205>/Timer3' */
  real_T Exit_o;                       /* '<S205>/Timer2' */
  real_T Exit_i;                       /* '<S205>/Timer1' */
  real_T Exit_le;                      /* '<S205>/Timer' */
  real_T Exit_on;                      /* '<S204>/Timer' */
  real_T low_VOL;                      /* '<S180>/CAN Unpack' */
  real_T MCU_Temp_error;               /* '<S180>/CAN Unpack' */
  real_T Mode;                         /* '<S180>/CAN Unpack' */
  real_T motorTemp_error;              /* '<S180>/CAN Unpack' */
  real_T overCurrent;                  /* '<S180>/CAN Unpack' */
  real_T overpower;                    /* '<S180>/CAN Unpack' */
  real_T overvol;                      /* '<S180>/CAN Unpack' */
  real_T Precharge;                    /* '<S180>/CAN Unpack' */
  real_T Reslove_error;                /* '<S180>/CAN Unpack' */
  real_T voltage;                      /* '<S175>/CAN Unpack' */
  real_T MCFR_ActualTorque;            /* '<S153>/CAN Unpack' */
  real_T MCFR_bDerating;               /* '<S153>/CAN Unpack' */
  real_T MCFR_bQuitDCOn;               /* '<S153>/CAN Unpack' */
  real_T MCFR_bReserve;                /* '<S153>/CAN Unpack' */
  real_T MCFR_bWarn;                   /* '<S153>/CAN Unpack' */
  real_T MCFR_DiagnosticNum;           /* '<S161>/CAN Unpack' */
  real_T MCFL_ActualTorque;            /* '<S135>/CAN Unpack' */
  real_T MCFL_ActualVelocity_p;        /* '<S135>/CAN Unpack' */
  real_T MCFL_bDerating;               /* '<S135>/CAN Unpack' */
  real_T MCFL_bReserve;                /* '<S135>/CAN Unpack' */
  real_T MCFL_bWarn;                   /* '<S135>/CAN Unpack' */
  real_T MCFL_DiagnosticNum;           /* '<S144>/CAN Unpack' */
  real_T CANUnpack_o1;                 /* '<S188>/CAN Unpack' */
  real_T CANUnpack_o3;                 /* '<S188>/CAN Unpack' */
  real_T CANUnpack_o5;                 /* '<S188>/CAN Unpack' */
  real_T CANUnpack_o6;                 /* '<S188>/CAN Unpack' */
  real_T errorReset;                   /* '<S106>/Chart2' */
  real_T Exit_d;                       /* '<S8>/Timer2' */
  real_T Exit_g;                       /* '<S8>/Timer1' */
  real_T DYC_Enable_OUT;               /* '<S7>/Chart' */
  real_T TCSR_Enable_OUT;              /* '<S7>/Chart' */
  real_T TCSF_Enable_OUT;              /* '<S7>/Chart' */
  uint32_T CANReceive_o3;              /* '<S361>/CANReceive' */
  uint32_T CANReceive_o3_l;            /* '<S346>/CANReceive' */
  uint32_T CANReceive1_o3;             /* '<S120>/CANReceive1' */
  uint32_T CANReceive3_o3;             /* '<S120>/CANReceive3' */
  uint32_T CANReceive3_o3_e;           /* '<S130>/CANReceive3' */
  uint32_T CANReceive1_o3_n;           /* '<S130>/CANReceive1' */
  uint32_T CANReceive2_o3;             /* '<S130>/CANReceive2' */
  uint32_T CANReceive3_o3_i;           /* '<S131>/CANReceive3' */
  uint32_T CANReceive1_o3_h;           /* '<S131>/CANReceive1' */
  uint32_T CANReceive2_o3_j;           /* '<S131>/CANReceive2' */
  uint32_T CANReceive3_o3_c;           /* '<S122>/CANReceive3' */
  uint32_T CANReceive3_o3_m;           /* '<S116>/CANReceive3' */
  uint32_T CANReceive3_o3_cz;          /* '<S121>/CANReceive3' */
  uint32_T CANReceive3_o3_l;           /* '<S119>/CANReceive3' */
  int32_T DataTypeConversion2;         /* '<S337>/Data Type Conversion2' */
  uint16_T CastToSingle1;              /* '<S338>/Cast To Single1' */
  uint16_T CastToBoolean4;             /* '<S337>/Cast To Boolean4' */
  uint16_T CastToBoolean6;             /* '<S337>/Cast To Boolean6' */
  uint8_T CANReceive_o2;               /* '<S361>/CANReceive' */
  uint8_T CANReceive_o4[8];            /* '<S361>/CANReceive' */
  uint8_T CANReceive_o5;               /* '<S361>/CANReceive' */
  uint8_T CANReceive_o2_p;             /* '<S346>/CANReceive' */
  uint8_T CANReceive_o4_i[8];          /* '<S346>/CANReceive' */
  uint8_T CANReceive_o5_l;             /* '<S346>/CANReceive' */
  uint8_T CANTransmit;                 /* '<S353>/CANTransmit' */
  uint8_T CANPackMessage[8];           /* '<S336>/CANPackMessage' */
  uint8_T CANTransmit_l;               /* '<S336>/CANTransmit' */
  uint8_T CANPackMessage_d[8];         /* '<S337>/CANPackMessage' */
  uint8_T CANTransmit_k;               /* '<S337>/CANTransmit' */
  uint8_T CANPackMessage_h[8];         /* '<S335>/CANPackMessage' */
  uint8_T CANTransmit_c;               /* '<S335>/CANTransmit' */
  uint8_T CANReceive1_o2;              /* '<S120>/CANReceive1' */
  uint8_T CANReceive1_o4[8];           /* '<S120>/CANReceive1' */
  uint8_T CANReceive1_o5;              /* '<S120>/CANReceive1' */
  uint8_T CANReceive3_o2;              /* '<S120>/CANReceive3' */
  uint8_T CANReceive3_o4[8];           /* '<S120>/CANReceive3' */
  uint8_T CANReceive3_o5;              /* '<S120>/CANReceive3' */
  uint8_T CANReceive3_o2_l;            /* '<S130>/CANReceive3' */
  uint8_T CANReceive3_o4_l[8];         /* '<S130>/CANReceive3' */
  uint8_T CANReceive3_o5_a;            /* '<S130>/CANReceive3' */
  uint8_T CANReceive1_o2_l;            /* '<S130>/CANReceive1' */
  uint8_T CANReceive1_o4_c[8];         /* '<S130>/CANReceive1' */
  uint8_T CANReceive1_o5_a;            /* '<S130>/CANReceive1' */
  uint8_T CANReceive2_o2;              /* '<S130>/CANReceive2' */
  uint8_T CANReceive2_o4[6];           /* '<S130>/CANReceive2' */
  uint8_T CANReceive2_o5;              /* '<S130>/CANReceive2' */
  uint8_T CANReceive3_o2_a;            /* '<S131>/CANReceive3' */
  uint8_T CANReceive3_o4_g[8];         /* '<S131>/CANReceive3' */
  uint8_T CANReceive3_o5_an;           /* '<S131>/CANReceive3' */
  uint8_T CANReceive1_o2_o;            /* '<S131>/CANReceive1' */
  uint8_T CANReceive1_o4_j[8];         /* '<S131>/CANReceive1' */
  uint8_T CANReceive1_o5_j;            /* '<S131>/CANReceive1' */
  uint8_T CANReceive2_o2_p;            /* '<S131>/CANReceive2' */
  uint8_T CANReceive2_o4_k[6];         /* '<S131>/CANReceive2' */
  uint8_T CANReceive2_o5_e;            /* '<S131>/CANReceive2' */
  uint8_T CANReceive3_o2_p;            /* '<S122>/CANReceive3' */
  uint8_T CANReceive3_o4_k[8];         /* '<S122>/CANReceive3' */
  uint8_T CANReceive3_o5_d;            /* '<S122>/CANReceive3' */
  uint8_T CANReceive3_o2_m;            /* '<S116>/CANReceive3' */
  uint8_T CANReceive3_o4_lg[8];        /* '<S116>/CANReceive3' */
  uint8_T CANReceive3_o5_b;            /* '<S116>/CANReceive3' */
  uint8_T CANReceive3_o2_ma;           /* '<S121>/CANReceive3' */
  uint8_T CANReceive3_o4_i[8];         /* '<S121>/CANReceive3' */
  uint8_T CANReceive3_o5_m;            /* '<S121>/CANReceive3' */
  uint8_T CANReceive3_o2_k;            /* '<S119>/CANReceive3' */
  uint8_T CANReceive3_o4_p[8];         /* '<S119>/CANReceive3' */
  uint8_T CANReceive3_o5_de;           /* '<S119>/CANReceive3' */
  boolean_T Drive_ready;               /* '<S123>/SwitchInput' */
  boolean_T MCFL_DCOn_setpoints_d;     /* '<S106>/Switch2' */
  boolean_T MCFL_InverterOn;           /* '<S106>/Switch6' */
  boolean_T MCFL_DCEnable;             /* '<S106>/Switch7' */
  boolean_T VehReady;                  /* '<S106>/Chart2' */
  boolean_T MCFR_TorqueOn;             /* '<S106>/Chart2' */
  boolean_T MCFL_TorqueOn;             /* '<S106>/Chart2' */
  boolean_T aWaterPumpON;
} B_VehCtrlMdel240926_2018b_amk_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DelayInput2_DSTATE;           /* '<S237>/Delay Input2' */
  real_T DelayInput2_DSTATE_a;         /* '<S250>/Delay Input2' */
  real_T DelayInput2_DSTATE_b;         /* '<S251>/Delay Input2' */
  real_T DelayInput2_DSTATE_h;         /* '<S252>/Delay Input2' */
  real_T DelayInput2_DSTATE_n;         /* '<S253>/Delay Input2' */
  real_T UnitDelay_DSTATE;             /* '<S204>/Unit Delay' */
  real_T DelayInput2_DSTATE_l;         /* '<S227>/Delay Input2' */
  real_T UnitDelay_DSTATE_g;           /* '<S226>/Unit Delay' */
  real_T DelayInput2_DSTATE_m;         /* '<S228>/Delay Input2' */
  real_T UnitDelay1_DSTATE;            /* '<S226>/Unit Delay1' */
  real_T DelayInput2_DSTATE_j;         /* '<S238>/Delay Input2' */
  real_T UnitDelay1_DSTATE_a;          /* '<S204>/Unit Delay1' */
  real_T DelayInput2_DSTATE_k;         /* '<S229>/Delay Input2' */
  real_T UnitDelay2_DSTATE;            /* '<S226>/Unit Delay2' */
  real_T DelayInput2_DSTATE_jk;        /* '<S42>/Delay Input2' */
  real_T UnitDelay_DSTATE_n;           /* '<S7>/Unit Delay' */
  real_T UnitDelay2_DSTATE_d;          /* '<S10>/Unit Delay2' */
  real_T UnitDelay_DSTATE_m;           /* '<S10>/Unit Delay' */
  real_T UnitDelay1_DSTATE_p;          /* '<S10>/Unit Delay1' */
  real_T DelayInput2_DSTATE_n0;        /* '<S44>/Delay Input2' */
  real_T DelayInput2_DSTATE_hk;        /* '<S41>/Delay Input2' */
  real_T UnitDelay4_DSTATE;            /* '<S30>/Unit Delay4' */
  real_T UnitDelay2_DSTATE_g;          /* '<S30>/Unit Delay2' */
  real_T UnitDelay4_DSTATE_i;          /* '<S29>/Unit Delay4' */
  real_T UnitDelay4_DSTATE_l;          /* '<S28>/Unit Delay4' */
  real_T DelayInput2_DSTATE_mb;        /* '<S39>/Delay Input2' */
  real_T UnitDelay2_DSTATE_j;          /* '<S29>/Unit Delay2' */
  real_T UnitDelay3_DSTATE;            /* '<S26>/Unit Delay3' */
  real_T UnitDelay_DSTATE_p;           /* '<S26>/Unit Delay' */
  real_T DelayInput2_DSTATE_o;         /* '<S40>/Delay Input2' */
  real_T UnitDelay2_DSTATE_b;          /* '<S28>/Unit Delay2' */
  real_T UnitDelay3_DSTATE_k;          /* '<S27>/Unit Delay3' */
  real_T UnitDelay_DSTATE_gb;          /* '<S27>/Unit Delay' */
  real_T b;                            /* '<S7>/Chart' */
  real_T DYC_flag;                     /* '<S7>/Chart' */
  real32_T UnitDelay_DSTATE_pj;        /* '<S279>/Unit Delay' */
  real32_T DelayInput2_DSTATE_n2;      /* '<S283>/Delay Input2' */
  real32_T UnitDelay_DSTATE_j;         /* '<S272>/Unit Delay' */
  real32_T UnitDelay_DSTATE_pjk;       /* '<S280>/Unit Delay' */
  real32_T DelayInput2_DSTATE_e;       /* '<S286>/Delay Input2' */
  real32_T UnitDelay1_DSTATE_n;        /* '<S272>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_a;         /* '<S281>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hk3;     /* '<S289>/Delay Input2' */
  real32_T UnitDelay2_DSTATE_l;        /* '<S272>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_nc;        /* '<S282>/Unit Delay' */
  real32_T DelayInput2_DSTATE_c;       /* '<S292>/Delay Input2' */
  real32_T UnitDelay3_DSTATE_h;        /* '<S272>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_l;         /* '<S299>/Unit Delay' */
  real32_T DelayInput2_DSTATE_i;       /* '<S303>/Delay Input2' */
  real32_T UnitDelay4_DSTATE_ik;       /* '<S273>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_a0;        /* '<S273>/Unit Delay' */
  real32_T UnitDelay_DSTATE_ap;        /* '<S300>/Unit Delay' */
  real32_T DelayInput2_DSTATE_el;      /* '<S306>/Delay Input2' */
  real32_T UnitDelay5_DSTATE;          /* '<S273>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_am;       /* '<S273>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_o;         /* '<S301>/Unit Delay' */
  real32_T DelayInput2_DSTATE_p;       /* '<S309>/Delay Input2' */
  real32_T UnitDelay6_DSTATE;          /* '<S273>/Unit Delay6' */
  real32_T UnitDelay2_DSTATE_c;        /* '<S273>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_ah;        /* '<S302>/Unit Delay' */
  real32_T DelayInput2_DSTATE_mt;      /* '<S312>/Delay Input2' */
  real32_T UnitDelay7_DSTATE;          /* '<S273>/Unit Delay7' */
  real32_T UnitDelay3_DSTATE_d;        /* '<S273>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_lh;        /* '<S201>/Unit Delay' */
  real32_T UnitDelay4_DSTATE_m;        /* '<S262>/Unit Delay4' */
  real32_T UnitDelay1_DSTATE_k;        /* '<S201>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_o;        /* '<S262>/Unit Delay1' */
  real32_T UnitDelay3_DSTATE_p;        /* '<S264>/Unit Delay3' */
  real32_T UnitDelay1_DSTATE_d;        /* '<S264>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_f;        /* '<S264>/Unit Delay2' */
  real32_T UnitDelay4_DSTATE_mn;       /* '<S264>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_d;         /* '<S262>/Unit Delay' */
  real32_T UnitDelay2_DSTATE_i;        /* '<S262>/Unit Delay2' */
  real32_T DelayInput2_DSTATE_g;       /* '<S324>/Delay Input2' */
  real32_T DelayInput2_DSTATE_af;      /* '<S322>/Delay Input2' */
  real32_T DelayInput2_DSTATE_f;       /* '<S328>/Delay Input2' */
  real32_T UnitDelay_DSTATE_ncs;       /* '<S321>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hu;      /* '<S323>/Delay Input2' */
  real32_T DelayInput2_DSTATE_d;       /* '<S45>/Delay Input2' */
  real32_T UnitDelay4_DSTATE_n;        /* '<S10>/Unit Delay4' */
  real32_T UnitDelay5_DSTATE_b;        /* '<S10>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_i;        /* '<S94>/Unit Delay1' */
  real32_T UnitDelay5_DSTATE_l;        /* '<S30>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_b;         /* '<S30>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_g;        /* '<S30>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_n5;       /* '<S83>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_h;        /* '<S74>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_cd;      /* '<S19>/Delay Input2' */
  real32_T DelayInput2_DSTATE_fo;      /* '<S43>/Delay Input2' */
  real32_T UnitDelay5_DSTATE_i;        /* '<S29>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_f;         /* '<S29>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_f;        /* '<S29>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_hn;      /* '<S20>/Delay Input2' */
  real32_T UnitDelay5_DSTATE_ip;       /* '<S28>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_nr;        /* '<S28>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_g4;       /* '<S28>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_ib;      /* '<S21>/Delay Input2' */
  int32_T sfEvent;                     /* '<S106>/Chart2' */
  uint32_T Subsystem_PREV_T;           /* '<S4>/Subsystem' */
  uint32_T FunctionCallSubsystem_PREV_T;/* '<S4>/Function-Call Subsystem' */
  uint32_T previousTicks;              /* '<S106>/Chart2' */
  uint32_T MoTrqReq_PREV_T;            /* '<S1>/MoTrqReq' */
  int_T CANPack1_ModeSignalID;         /* '<S336>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_l;       /* '<S337>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_f;       /* '<S335>/CAN Pack1' */
  int_T CANUnpack_ModeSignalID;        /* '<S180>/CAN Unpack' */
  int_T CANUnpack_StatusPortID;        /* '<S180>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_b;      /* '<S175>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_m;      /* '<S175>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_a;      /* '<S153>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_o;      /* '<S153>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_n;      /* '<S163>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_f;      /* '<S163>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_p;      /* '<S161>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_e;      /* '<S161>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_c;      /* '<S135>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_ok;     /* '<S135>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_m;      /* '<S146>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_j;      /* '<S146>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_i;      /* '<S144>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_mj;     /* '<S144>/CAN Unpack' */
  int_T CANUnpack1_ModeSignalID;       /* '<S193>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID;       /* '<S193>/CAN Unpack1' */
  int_T CANUnpack1_ModeSignalID_m;     /* '<S125>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID_n;     /* '<S125>/CAN Unpack1' */
  int_T CANUnpack_ModeSignalID_f;      /* '<S188>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_k;      /* '<S188>/CAN Unpack' */
  struct {
    uint_T is_VehStat:4;               /* '<S106>/Chart2' */
    uint_T is_AMKDCready:4;            /* '<S106>/Chart2' */
    uint_T is_BeeperStat:2;            /* '<S106>/Chart2' */
    uint_T is_AMKDCon:2;               /* '<S106>/Chart2' */
    uint_T is_MCDCEnable:2;            /* '<S106>/Chart2' */
    uint_T is_MC_TorqueCUT:2;          /* '<S106>/Chart2' */
    uint_T is_AMKCANenable:2;          /* '<S106>/Chart2' */
    uint_T is_MC_InverterOn:2;         /* '<S106>/Chart2' */
    uint_T is_B:2;                     /* '<S7>/Chart' */
    uint_T is_C:2;                     /* '<S7>/Chart' */
    uint_T is_D:2;                     /* '<S7>/Chart' */
    uint_T is_E:2;                     /* '<S7>/Chart' */
    uint_T is_active_c1_VehCtrlMdel240926_:1;/* '<S106>/Chart2' */
    uint_T is_active_VehStat:1;        /* '<S106>/Chart2' */
    uint_T is_active_BeeperStat:1;     /* '<S106>/Chart2' */
    uint_T is_active_AMKDCon:1;        /* '<S106>/Chart2' */
    uint_T is_active_MCDCEnable:1;     /* '<S106>/Chart2' */
    uint_T is_active_MC_TorqueCUT:1;   /* '<S106>/Chart2' */
    uint_T is_active_AMKDCready:1;     /* '<S106>/Chart2' */
    uint_T is_active_Output:1;         /* '<S106>/Chart2' */
    uint_T is_active_AMKCANenable:1;   /* '<S106>/Chart2' */
    uint_T is_active_MC_InverterOn:1;  /* '<S106>/Chart2' */
    uint_T is_active_c7_VehCtrlMdel240926_:1;/* '<S7>/Chart' */
  } bitsForTID3;

  uint16_T UnitDelay1_DSTATE_fm;       /* '<S202>/Unit Delay1' */
  uint16_T UnitDelay_DSTATE_k;         /* '<S202>/Unit Delay' */
  uint16_T temporalCounter_i2;         /* '<S106>/Chart2' */
  boolean_T UnitDelay3_DSTATE_f;       /* '<S262>/Unit Delay3' */
  boolean_T UnitDelay3_DSTATE_l;       /* '<S10>/Unit Delay3' */
  boolean_T UnitDelay6_DSTATE_i;       /* '<S10>/Unit Delay6' */
  boolean_T UnitDelay1_DSTATE_e;       /* '<S93>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_a;       /* '<S30>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE;        /* '<S92>/Delay Input1' */
  boolean_T UnitDelay1_DSTATE_gl;      /* '<S82>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_e;       /* '<S29>/Unit Delay3' */
  boolean_T UnitDelay1_DSTATE_dp;      /* '<S73>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_i;       /* '<S28>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE_j;      /* '<S81>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_b;      /* '<S72>/Delay Input1' */
  uint8_T temporalCounter_i1;          /* '<S106>/Chart2' */
  boolean_T Subsystem_RESET_ELAPS_T;   /* '<S4>/Subsystem' */
  boolean_T FunctionCallSubsystem_RESET_ELA;/* '<S4>/Function-Call Subsystem' */
  boolean_T MoTrqReq_RESET_ELAPS_T;    /* '<S1>/MoTrqReq' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2_j;/* '<S264>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1_p;/* '<S264>/Timer1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer3_i;/* '<S263>/Timer3' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2_g;/* '<S263>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1_m;/* '<S263>/Timer1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer_o;/* '<S263>/Timer' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer3;/* '<S205>/Timer3' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2_l;/* '<S205>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1_n;/* '<S205>/Timer1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer_b;/* '<S205>/Timer' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer_k;/* '<S204>/Timer' */
  DW_Timer_VehCtrlMdel240926_20_T sf_Timer_a;/* '<S202>/Timer' */
  DW_Timer_VehCtrlMdel240926_20_T sf_Timer;/* '<S123>/Timer' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2;/* '<S8>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1;/* '<S8>/Timer1' */
} DW_VehCtrlMdel240926_2018b_am_T;

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
  real_T pooled14[20];

  /* Pooled Parameter (Expression: [0;1000;2000;3000;4000;5000;6000;7000;8000;9000;10000;11000;12000;13000;14000;15000;16000;17000;18000;19000])
   * Referenced by:
   *   '<S10>/AMK'
   *   '<S10>/AMK1'
   */
  real_T pooled15[20];

  /* Pooled Parameter (Expression: [0,0,0.2,0.4,0.6,0.8,1,1])
   * Referenced by:
   *   '<S8>/2-D Lookup Table1'
   *   '<S8>/2-D Lookup Table3'
   */
  real_T pooled18[8];

  /* Pooled Parameter (Expression: [35,40,41,42,43,44,45,50])
   * Referenced by:
   *   '<S8>/2-D Lookup Table1'
   *   '<S8>/2-D Lookup Table3'
   */
  real_T pooled19[8];

  /* Expression: [0;0;20;40;70;100;100]
   * Referenced by: '<S8>/2-D Lookup Table2'
   */
  real_T uDLookupTable2_tableData[7];

  /* Expression: [0;25;30;35;37;40;50]
   * Referenced by: '<S8>/2-D Lookup Table2'
   */
  real_T uDLookupTable2_bp01Data[7];

  /* Computed Parameter: uDLookupTable_tableData
   * Referenced by: '<S204>/1-D Lookup Table'
   */
  real_T uDLookupTable_tableData[24];

  /* Computed Parameter: uDLookupTable_bp01Data
   * Referenced by: '<S204>/1-D Lookup Table'
   */
  real_T uDLookupTable_bp01Data[24];

  /* Computed Parameter: uDLookupTable1_tableData
   * Referenced by: '<S204>/1-D Lookup Table1'
   */
  real_T uDLookupTable1_tableData[24];

  /* Computed Parameter: uDLookupTable1_bp01Data
   * Referenced by: '<S204>/1-D Lookup Table1'
   */
  real_T uDLookupTable1_bp01Data[24];

  /* Pooled Parameter (Expression: single([20,0]);)
   * Referenced by:
   *   '<S7>/BrakeCompensateCoefFront1'
   *   '<S7>/BrakeCompensateCoefFront2'
   */
  real32_T pooled33[2];

  /* Pooled Parameter (Expression: single([550,1700]);)
   * Referenced by:
   *   '<S7>/BrakeCompensateCoefFront1'
   *   '<S7>/BrakeCompensateCoefFront2'
   */
  real32_T pooled34[2];

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
   *   '<S28>/VehSpd_SlipTarget_mps'
   *   '<S29>/VehSpd_SlipTarget_mps'
   *   '<S30>/VehSpd_SlipTarget_mps'
   */
  real32_T pooled55[4];

  /* Pooled Parameter (Expression: [0,3,25,30])
   * Referenced by:
   *   '<S28>/VehSpd_SlipTarget_mps'
   *   '<S28>/VehicleStableTarget_mps'
   *   '<S28>/VehicleStableTarget_mps1'
   *   '<S29>/VehSpd_SlipTarget_mps'
   *   '<S29>/VehicleStableTarget_mps'
   *   '<S29>/VehicleStableTarget_mps1'
   *   '<S30>/VehSpd_SlipTarget_mps'
   *   '<S30>/VehicleStableTarget_mps'
   *   '<S30>/VehicleStableTarget_mps1'
   */
  real32_T pooled56[4];

  /* Pooled Parameter (Expression: [0.4,0.4,1.2,1.2])
   * Referenced by:
   *   '<S28>/VehicleStableTarget_mps'
   *   '<S28>/VehicleStableTarget_mps1'
   *   '<S29>/VehicleStableTarget_mps'
   *   '<S29>/VehicleStableTarget_mps1'
   *   '<S30>/VehicleStableTarget_mps'
   *   '<S30>/VehicleStableTarget_mps1'
   */
  real32_T pooled62[4];

  /* Expression: single([200,0]);
   * Referenced by: '<S7>/BrakeCompensateCoefRear'
   */
  real32_T BrakeCompensateCoefRear_tableDa[2];

  /* Expression: single([550,1500]);
   * Referenced by: '<S7>/BrakeCompensateCoefRear'
   */
  real32_T BrakeCompensateCoefRear_bp01Dat[2];

  /* Pooled Parameter (Expression: single([500,4500]);)
   * Referenced by: '<S202>/1-D Lookup Table1'
   */
  real32_T pooled67[2];

  /* Expression: single([0,100]);
   * Referenced by: '<S202>/1-D Lookup Table3'
   */
  real32_T uDLookupTable3_tableData[2];

  /* Expression: single([3540,3790])
   * Referenced by: '<S202>/1-D Lookup Table3'
   */
  real32_T uDLookupTable3_bp01Data[2];

  /* Computed Parameter: uDLookupTable1_maxIndex
   * Referenced by: '<S7>/2-D Lookup Table1'
   */
  uint32_T uDLookupTable1_maxIndex[2];
} ConstP_VehCtrlMdel240926_2018_T;

/* Real-time Model Data Structure */
struct tag_RTM_VehCtrlMdel240926_201_T {
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
extern B_VehCtrlMdel240926_2018b_amk_T VehCtrlMdel240926_2018b_amksp_B;

/* Block states (default storage) */
extern DW_VehCtrlMdel240926_2018b_am_T VehCtrlMdel240926_2018b_amks_DW;

/* Constant parameters (default storage) */
extern const ConstP_VehCtrlMdel240926_2018_T VehCtrlMdel240926_2018b__ConstP;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real_T Gear_Trs;                /* '<S337>/Switch2' */
extern real_T Mode_Trs;                /* '<S337>/Switch3' */
extern real_T Trq_CUT;                 /* '<S202>/Timer' */
extern real_T ignition;                /* '<S123>/Timer' */
extern real_T L12V_error;              /* '<S180>/CAN Unpack' */
extern real_T alarm;                   /* '<S180>/CAN Unpack' */
extern real_T controller_ready;        /* '<S180>/CAN Unpack' */
extern real_T selfcheck;               /* '<S180>/CAN Unpack' */
extern real_T RPM;                     /* '<S180>/CAN Unpack' */
extern real_T trq;                     /* '<S180>/CAN Unpack' */
extern real_T AC_current;              /* '<S175>/CAN Unpack' */
extern real_T DC_current;              /* '<S175>/CAN Unpack' */
extern real_T MCU_Temp;                /* '<S175>/CAN Unpack' */
extern real_T motor_Temp;              /* '<S175>/CAN Unpack' */
extern real_T MCFR_ActualVelocity;     /* '<S153>/CAN Unpack' */
extern real_T MCFR_DCVoltage;          /* '<S153>/CAN Unpack' */
extern real_T MCFR_bDCOn;              /* '<S153>/CAN Unpack' */
extern real_T MCFR_bError;             /* '<S153>/CAN Unpack' */
extern real_T MCFR_bInverterOn;        /* '<S153>/CAN Unpack' */
extern real_T MCFR_bQuitInverterOn;    /* '<S153>/CAN Unpack' */
extern real_T MCFR_bSystemReady;       /* '<S153>/CAN Unpack' */
extern real_T MCFR_TempIGBT;           /* '<S163>/CAN Unpack' */
extern real_T MCFR_TempInverter;       /* '<S163>/CAN Unpack' */
extern real_T MCFR_TempMotor;          /* '<S163>/CAN Unpack' */
extern real_T MCFR_ErrorInfo;          /* '<S161>/CAN Unpack' */
extern real_T MCFL_DCVoltage;          /* '<S135>/CAN Unpack' */
extern real_T MCFL_bDCOn;              /* '<S135>/CAN Unpack' */
extern real_T MCFL_bError;             /* '<S135>/CAN Unpack' */
extern real_T MCFL_bInverterOn;        /* '<S135>/CAN Unpack' */
extern real_T MCFL_bQuitDCOn;          /* '<S135>/CAN Unpack' */
extern real_T MCFL_bQuitInverterOn;    /* '<S135>/CAN Unpack' */
extern real_T MCFL_bSystemReady;       /* '<S135>/CAN Unpack' */
extern real_T MCFL_ActualVelocity;     /* '<S135>/Gain' */
extern real_T MCFL_TempIGBT;           /* '<S146>/CAN Unpack' */
extern real_T MCFL_TempInverter;       /* '<S146>/CAN Unpack' */
extern real_T MCFL_TempMotor;          /* '<S146>/CAN Unpack' */
extern real_T MCFL_ErrorInfo;          /* '<S144>/CAN Unpack' */
extern real_T StrWhlAngAliveRollCnt;   /* '<S193>/CAN Unpack1' */
extern real_T StrWhlAng;               /* '<S193>/CAN Unpack1' */
extern real_T StrWhlAngV;              /* '<S193>/CAN Unpack1' */
extern real_T ABS_WS_FL;               /* '<S125>/CAN Unpack1' */
extern real_T ABS_WS_FR;               /* '<S125>/CAN Unpack1' */
extern real_T ABS_WS_RL;               /* '<S125>/CAN Unpack1' */
extern real_T ABS_WS_RR;               /* '<S125>/CAN Unpack1' */
extern real_T IMU_Ay_Value;            /* '<S188>/CAN Unpack' */
extern real_T IMU_Ax_Value;            /* '<S188>/CAN Unpack' */
extern real_T IMU_Yaw_Value;           /* '<S188>/CAN Unpack' */
extern real_T EMRAX_Trq_CUT;           /*  */
extern real_T AMK_Trq_CUT;             /*  */
extern uint32_T Acc_vol2;              /* '<S202>/Add3' */
extern uint32_T Acc_vol;               /* '<S202>/Add2' */
extern uint32_T Acc_POS2;              /* '<S202>/1-D Lookup Table3' */
extern real32_T Acc_POS;               /* '<S202>/MATLAB Function' */
extern real32_T Trq_nm;                /* '<S7>/2-D Lookup Table1' */
extern real32_T TrqR_cmd;              /* '<S7>/Saturation1' */
extern real32_T TrqFR_cmd;             /* '<S7>/Saturation2' */
extern real32_T TrqFL_cmd;             /* '<S7>/Saturation3' */
extern uint16_T F_BrkPrs;              /* '<S202>/1-D Lookup Table1' */
extern uint16_T Acc1;                  /* '<S118>/Acc3' */
extern uint16_T Acc2;                  /* '<S118>/Acc4' */
extern uint16_T Brk1;                  /* '<S118>/Brk1' */
extern uint16_T Brk2;                  /* '<S118>/Brk2' */
extern boolean_T HVCUTOFF;             /* '<S123>/Constant' */
extern boolean_T KeyPressed;           /* '<S106>/Cast To Boolean' */
extern boolean_T Brk;                  /* '<S108>/Compare' */
extern boolean_T ACC_Release;          /* '<S109>/Compare' */
extern boolean_T beeper_state;         /* '<S106>/Chart2' */
extern boolean_T MCFL_DCOn_setpoints;  /* '<S106>/Chart2' */
extern boolean_T MCFR_DCEnable;        /* '<S106>/Chart2' */
extern boolean_T MCFR_InverterOn;      /* '<S106>/Chart2' */
extern boolean_T Trq_CUT_final;        /* '<S7>/Logical Operator4' */
extern boolean_T TroqueOn;             /* '<S7>/Logical Operator6' */

/* External function called from main */
extern void VehCtrlMdel240926_2018b_amkspdlimit_SetEventsForThisBaseStep
  (boolean_T *eventFlags);

/* Model entry point functions */
extern void VehCtrlMdel240926_2018b_amkspdlimit_SetEventsForThisBaseStep
  (boolean_T *eventFlags);
extern void VehCtrlMdel240926_2018b_amkspdlimit_initialize(void);
extern void VehCtrlMdel240926_2018b_amkspdlimit_step(int_T tid);
extern uint8_T ECUCoderModelBaseCounter;
extern uint32_t IntcIsrVectorTable[];
extern uint8_T AfterRunFlags[2];
extern SSD_CONFIG ssdConfig;
extern void ISR_PIT_CH3(void);

/* Real-time Model object */
extern RT_MODEL_VehCtrlMdel240926_20_T *const VehCtrlMdel240926_2018b_amks_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S10>/2-D Lookup Table' : Unused code path elimination
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
 * Block '<S43>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S58>/Data Type Duplicate' : Unused code path elimination
 * Block '<S58>/Data Type Propagation' : Unused code path elimination
 * Block '<S44>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S59>/Data Type Duplicate' : Unused code path elimination
 * Block '<S59>/Data Type Propagation' : Unused code path elimination
 * Block '<S45>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Propagation' : Unused code path elimination
 * Block '<S46>/Data Type Duplicate' : Unused code path elimination
 * Block '<S46>/Data Type Propagation' : Unused code path elimination
 * Block '<S47>/Data Type Duplicate' : Unused code path elimination
 * Block '<S47>/Data Type Propagation' : Unused code path elimination
 * Block '<S48>/Data Type Duplicate' : Unused code path elimination
 * Block '<S48>/Data Type Propagation' : Unused code path elimination
 * Block '<S49>/Data Type Duplicate' : Unused code path elimination
 * Block '<S49>/Data Type Propagation' : Unused code path elimination
 * Block '<S50>/Data Type Duplicate' : Unused code path elimination
 * Block '<S50>/Data Type Propagation' : Unused code path elimination
 * Block '<S51>/Data Type Duplicate' : Unused code path elimination
 * Block '<S51>/Data Type Propagation' : Unused code path elimination
 * Block '<S19>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S61>/Data Type Duplicate' : Unused code path elimination
 * Block '<S61>/Data Type Propagation' : Unused code path elimination
 * Block '<S20>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S62>/Data Type Duplicate' : Unused code path elimination
 * Block '<S62>/Data Type Propagation' : Unused code path elimination
 * Block '<S21>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S63>/Data Type Duplicate' : Unused code path elimination
 * Block '<S63>/Data Type Propagation' : Unused code path elimination
 * Block '<S22>/Data Type Duplicate' : Unused code path elimination
 * Block '<S22>/Data Type Propagation' : Unused code path elimination
 * Block '<S23>/Data Type Duplicate' : Unused code path elimination
 * Block '<S23>/Data Type Propagation' : Unused code path elimination
 * Block '<S24>/Data Type Duplicate' : Unused code path elimination
 * Block '<S24>/Data Type Propagation' : Unused code path elimination
 * Block '<S65>/Data Type Duplicate' : Unused code path elimination
 * Block '<S65>/Data Type Propagation' : Unused code path elimination
 * Block '<S66>/Data Type Duplicate' : Unused code path elimination
 * Block '<S66>/Data Type Propagation' : Unused code path elimination
 * Block '<S68>/Data Type Duplicate' : Unused code path elimination
 * Block '<S68>/Data Type Propagation' : Unused code path elimination
 * Block '<S69>/Data Type Duplicate' : Unused code path elimination
 * Block '<S69>/Data Type Propagation' : Unused code path elimination
 * Block '<S75>/Data Type Duplicate' : Unused code path elimination
 * Block '<S75>/Data Type Propagation' : Unused code path elimination
 * Block '<S76>/Data Type Duplicate' : Unused code path elimination
 * Block '<S76>/Data Type Propagation' : Unused code path elimination
 * Block '<S77>/Data Type Duplicate' : Unused code path elimination
 * Block '<S77>/Data Type Propagation' : Unused code path elimination
 * Block '<S84>/Data Type Duplicate' : Unused code path elimination
 * Block '<S84>/Data Type Propagation' : Unused code path elimination
 * Block '<S85>/Data Type Duplicate' : Unused code path elimination
 * Block '<S85>/Data Type Propagation' : Unused code path elimination
 * Block '<S86>/Data Type Duplicate' : Unused code path elimination
 * Block '<S86>/Data Type Propagation' : Unused code path elimination
 * Block '<S95>/Data Type Duplicate' : Unused code path elimination
 * Block '<S95>/Data Type Propagation' : Unused code path elimination
 * Block '<S96>/Data Type Duplicate' : Unused code path elimination
 * Block '<S96>/Data Type Propagation' : Unused code path elimination
 * Block '<S97>/Data Type Duplicate' : Unused code path elimination
 * Block '<S97>/Data Type Propagation' : Unused code path elimination
 * Block '<S106>/Switch3' : Unused code path elimination
 * Block '<S106>/Switch4' : Unused code path elimination
 * Block '<S106>/Switch5' : Unused code path elimination
 * Block '<S172>/CAN Unpack1' : Unused code path elimination
 * Block '<S202>/1-D Lookup Table2' : Unused code path elimination
 * Block '<S202>/Abs1' : Unused code path elimination
 * Block '<S202>/Add4' : Unused code path elimination
 * Block '<S208>/Compare' : Unused code path elimination
 * Block '<S208>/Constant' : Unused code path elimination
 * Block '<S209>/Compare' : Unused code path elimination
 * Block '<S209>/Constant' : Unused code path elimination
 * Block '<S217>/Compare' : Unused code path elimination
 * Block '<S217>/Constant' : Unused code path elimination
 * Block '<S202>/Logical Operator6' : Unused code path elimination
 * Block '<S202>/Logical Operator7' : Unused code path elimination
 * Block '<S227>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S233>/Data Type Duplicate' : Unused code path elimination
 * Block '<S233>/Data Type Propagation' : Unused code path elimination
 * Block '<S228>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S234>/Data Type Duplicate' : Unused code path elimination
 * Block '<S234>/Data Type Propagation' : Unused code path elimination
 * Block '<S229>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S235>/Data Type Duplicate' : Unused code path elimination
 * Block '<S235>/Data Type Propagation' : Unused code path elimination
 * Block '<S230>/Data Type Duplicate' : Unused code path elimination
 * Block '<S230>/Data Type Propagation' : Unused code path elimination
 * Block '<S231>/Data Type Duplicate' : Unused code path elimination
 * Block '<S231>/Data Type Propagation' : Unused code path elimination
 * Block '<S232>/Data Type Duplicate' : Unused code path elimination
 * Block '<S232>/Data Type Propagation' : Unused code path elimination
 * Block '<S237>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S240>/Data Type Duplicate' : Unused code path elimination
 * Block '<S240>/Data Type Propagation' : Unused code path elimination
 * Block '<S238>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S241>/Data Type Duplicate' : Unused code path elimination
 * Block '<S241>/Data Type Propagation' : Unused code path elimination
 * Block '<S250>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S258>/Data Type Duplicate' : Unused code path elimination
 * Block '<S258>/Data Type Propagation' : Unused code path elimination
 * Block '<S251>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S259>/Data Type Duplicate' : Unused code path elimination
 * Block '<S259>/Data Type Propagation' : Unused code path elimination
 * Block '<S252>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S260>/Data Type Duplicate' : Unused code path elimination
 * Block '<S260>/Data Type Propagation' : Unused code path elimination
 * Block '<S253>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S261>/Data Type Duplicate' : Unused code path elimination
 * Block '<S261>/Data Type Propagation' : Unused code path elimination
 * Block '<S283>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S285>/Data Type Duplicate' : Unused code path elimination
 * Block '<S285>/Data Type Propagation' : Unused code path elimination
 * Block '<S284>/Data Type Duplicate' : Unused code path elimination
 * Block '<S284>/Data Type Propagation' : Unused code path elimination
 * Block '<S286>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S288>/Data Type Duplicate' : Unused code path elimination
 * Block '<S288>/Data Type Propagation' : Unused code path elimination
 * Block '<S287>/Data Type Duplicate' : Unused code path elimination
 * Block '<S287>/Data Type Propagation' : Unused code path elimination
 * Block '<S289>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S291>/Data Type Duplicate' : Unused code path elimination
 * Block '<S291>/Data Type Propagation' : Unused code path elimination
 * Block '<S290>/Data Type Duplicate' : Unused code path elimination
 * Block '<S290>/Data Type Propagation' : Unused code path elimination
 * Block '<S292>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S294>/Data Type Duplicate' : Unused code path elimination
 * Block '<S294>/Data Type Propagation' : Unused code path elimination
 * Block '<S293>/Data Type Duplicate' : Unused code path elimination
 * Block '<S293>/Data Type Propagation' : Unused code path elimination
 * Block '<S303>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S305>/Data Type Duplicate' : Unused code path elimination
 * Block '<S305>/Data Type Propagation' : Unused code path elimination
 * Block '<S304>/Data Type Duplicate' : Unused code path elimination
 * Block '<S304>/Data Type Propagation' : Unused code path elimination
 * Block '<S306>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S308>/Data Type Duplicate' : Unused code path elimination
 * Block '<S308>/Data Type Propagation' : Unused code path elimination
 * Block '<S307>/Data Type Duplicate' : Unused code path elimination
 * Block '<S307>/Data Type Propagation' : Unused code path elimination
 * Block '<S309>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S311>/Data Type Duplicate' : Unused code path elimination
 * Block '<S311>/Data Type Propagation' : Unused code path elimination
 * Block '<S310>/Data Type Duplicate' : Unused code path elimination
 * Block '<S310>/Data Type Propagation' : Unused code path elimination
 * Block '<S312>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S314>/Data Type Duplicate' : Unused code path elimination
 * Block '<S314>/Data Type Propagation' : Unused code path elimination
 * Block '<S313>/Data Type Duplicate' : Unused code path elimination
 * Block '<S313>/Data Type Propagation' : Unused code path elimination
 * Block '<S264>/Discrete-Time Integrator' : Unused code path elimination
 * Block '<S328>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S330>/Data Type Duplicate' : Unused code path elimination
 * Block '<S330>/Data Type Propagation' : Unused code path elimination
 * Block '<S329>/Data Type Duplicate' : Unused code path elimination
 * Block '<S329>/Data Type Propagation' : Unused code path elimination
 * Block '<S322>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S331>/Data Type Duplicate' : Unused code path elimination
 * Block '<S331>/Data Type Propagation' : Unused code path elimination
 * Block '<S323>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S332>/Data Type Duplicate' : Unused code path elimination
 * Block '<S332>/Data Type Propagation' : Unused code path elimination
 * Block '<S324>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S333>/Data Type Duplicate' : Unused code path elimination
 * Block '<S333>/Data Type Propagation' : Unused code path elimination
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
 * Block '<S202>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S262>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S262>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S263>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S263>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S263>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S263>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S321>/Gain1' : Eliminated nontunable gain of 1
 * Block '<S337>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S338>/Cast To Single' : Eliminate redundant data type conversion
 * Block '<S7>/Constant' : Unused code path elimination
 * Block '<S7>/Constant4' : Unused code path elimination
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
 * '<Root>' : 'VehCtrlMdel240926_2018b_amkspdlimit'
 * '<S1>'   : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL'
 * '<S2>'   : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready'
 * '<S3>'   : 'VehCtrlMdel240926_2018b_amkspdlimit/Input'
 * '<S4>'   : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing'
 * '<S5>'   : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT'
 * '<S6>'   : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting'
 * '<S7>'   : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq'
 * '<S8>'   : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct'
 * '<S9>'   : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Chart'
 * '<S10>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC'
 * '<S11>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule'
 * '<S12>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule1'
 * '<S13>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule2'
 * '<S14>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule3'
 * '<S15>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule4'
 * '<S16>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule5'
 * '<S17>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule6'
 * '<S18>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule7'
 * '<S19>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic1'
 * '<S20>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic2'
 * '<S21>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic3'
 * '<S22>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Saturation Dynamic1'
 * '<S23>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Saturation Dynamic2'
 * '<S24>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Saturation Dynamic3'
 * '<S25>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem'
 * '<S26>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem1'
 * '<S27>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem2'
 * '<S28>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL'
 * '<S29>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR'
 * '<S30>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R'
 * '<S31>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant'
 * '<S32>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant1'
 * '<S33>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant2'
 * '<S34>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant3'
 * '<S35>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant4'
 * '<S36>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant5'
 * '<S37>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/MATLAB Function'
 * '<S38>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/MATLAB Function1'
 * '<S39>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic'
 * '<S40>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic1'
 * '<S41>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic2'
 * '<S42>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic3'
 * '<S43>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic4'
 * '<S44>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic5'
 * '<S45>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic6'
 * '<S46>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic'
 * '<S47>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic1'
 * '<S48>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic2'
 * '<S49>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic3'
 * '<S50>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic4'
 * '<S51>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic5'
 * '<S52>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Wtarget'
 * '<S53>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/载荷转移'
 * '<S54>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S55>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S56>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S57>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S58>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic4/Saturation Dynamic'
 * '<S59>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic5/Saturation Dynamic'
 * '<S60>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic6/Saturation Dynamic'
 * '<S61>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S62>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S63>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S64>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem1/Compare To Constant'
 * '<S65>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem1/Saturation Dynamic'
 * '<S66>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem1/Saturation Dynamic1'
 * '<S67>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem2/Compare To Constant'
 * '<S68>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem2/Saturation Dynamic'
 * '<S69>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem2/Saturation Dynamic1'
 * '<S70>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Compare To Constant'
 * '<S71>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Compare To Constant2'
 * '<S72>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Detect Rise Positive'
 * '<S73>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Latch_on'
 * '<S74>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/RisingTimer'
 * '<S75>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic'
 * '<S76>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic1'
 * '<S77>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic2'
 * '<S78>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Detect Rise Positive/Positive'
 * '<S79>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Compare To Constant'
 * '<S80>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Compare To Constant2'
 * '<S81>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Detect Rise Positive'
 * '<S82>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Latch_on'
 * '<S83>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/RisingTimer'
 * '<S84>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic'
 * '<S85>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic1'
 * '<S86>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic2'
 * '<S87>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Detect Rise Positive/Positive'
 * '<S88>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant'
 * '<S89>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant1'
 * '<S90>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant2'
 * '<S91>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Zero'
 * '<S92>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Detect Rise Positive'
 * '<S93>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Latch_on'
 * '<S94>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/RisingTimer'
 * '<S95>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic'
 * '<S96>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic1'
 * '<S97>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic2'
 * '<S98>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Detect Rise Positive/Positive'
 * '<S99>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant'
 * '<S100>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant1'
 * '<S101>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant2'
 * '<S102>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant3'
 * '<S103>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant4'
 * '<S104>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Timer1'
 * '<S105>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Timer2'
 * '<S106>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem'
 * '<S107>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/Chart2'
 * '<S108>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/Compare To Constant'
 * '<S109>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/Compare To Constant1'
 * '<S110>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule'
 * '<S111>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule1'
 * '<S112>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule2'
 * '<S113>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule3'
 * '<S114>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule4'
 * '<S115>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule5'
 * '<S116>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive'
 * '<S117>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive'
 * '<S118>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS'
 * '<S119>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/BMS_Recive'
 * '<S120>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE'
 * '<S121>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve'
 * '<S122>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive'
 * '<S123>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key'
 * '<S124>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state'
 * '<S125>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state'
 * '<S126>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule1'
 * '<S127>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule2'
 * '<S128>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule3'
 * '<S129>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule4'
 * '<S130>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU'
 * '<S131>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU'
 * '<S132>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state'
 * '<S133>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1'
 * '<S134>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2'
 * '<S135>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state'
 * '<S136>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule1'
 * '<S137>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule2'
 * '<S138>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule3'
 * '<S139>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule4'
 * '<S140>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule5'
 * '<S141>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule6'
 * '<S142>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule7'
 * '<S143>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule8'
 * '<S144>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state'
 * '<S145>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state/MeaModule1'
 * '<S146>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state'
 * '<S147>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule1'
 * '<S148>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule2'
 * '<S149>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule3'
 * '<S150>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state'
 * '<S151>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1'
 * '<S152>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2'
 * '<S153>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state'
 * '<S154>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule1'
 * '<S155>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule2'
 * '<S156>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule3'
 * '<S157>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule4'
 * '<S158>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule5'
 * '<S159>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule6'
 * '<S160>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule7'
 * '<S161>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state'
 * '<S162>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state/MeaModule1'
 * '<S163>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state'
 * '<S164>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule1'
 * '<S165>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule2'
 * '<S166>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule3'
 * '<S167>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule1'
 * '<S168>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule2'
 * '<S169>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule3'
 * '<S170>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule4'
 * '<S171>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/BMS_Recive/ABS_BUS_state'
 * '<S172>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/BMS_Recive/ABS_BUS_state/IMU_state'
 * '<S173>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr'
 * '<S174>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state'
 * '<S175>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1'
 * '<S176>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule1'
 * '<S177>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule2'
 * '<S178>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule3'
 * '<S179>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule4'
 * '<S180>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state'
 * '<S181>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule'
 * '<S182>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule1'
 * '<S183>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule2'
 * '<S184>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule3'
 * '<S185>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule5'
 * '<S186>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule6'
 * '<S187>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state'
 * '<S188>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state'
 * '<S189>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule2'
 * '<S190>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule3'
 * '<S191>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule4'
 * '<S192>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state'
 * '<S193>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state'
 * '<S194>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule1'
 * '<S195>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule2'
 * '<S196>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule3'
 * '<S197>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/MeaModule'
 * '<S198>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/MeaModule1'
 * '<S199>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/Timer'
 * '<S200>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem'
 * '<S201>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem'
 * '<S202>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal'
 * '<S203>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU'
 * '<S204>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS'
 * '<S205>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps'
 * '<S206>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant'
 * '<S207>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant1'
 * '<S208>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant10'
 * '<S209>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant11'
 * '<S210>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant2'
 * '<S211>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant3'
 * '<S212>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant4'
 * '<S213>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant5'
 * '<S214>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant6'
 * '<S215>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant7'
 * '<S216>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant8'
 * '<S217>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant9'
 * '<S218>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MATLAB Function'
 * '<S219>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule1'
 * '<S220>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule2'
 * '<S221>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule3'
 * '<S222>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule5'
 * '<S223>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule7'
 * '<S224>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule8'
 * '<S225>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Timer'
 * '<S226>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem'
 * '<S227>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic'
 * '<S228>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1'
 * '<S229>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2'
 * '<S230>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic'
 * '<S231>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic1'
 * '<S232>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic2'
 * '<S233>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S234>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S235>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S236>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Compare To Constant'
 * '<S237>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic'
 * '<S238>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1'
 * '<S239>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Timer'
 * '<S240>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S241>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S242>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant'
 * '<S243>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant1'
 * '<S244>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant2'
 * '<S245>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant3'
 * '<S246>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant4'
 * '<S247>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant5'
 * '<S248>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant6'
 * '<S249>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant7'
 * '<S250>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic'
 * '<S251>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1'
 * '<S252>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2'
 * '<S253>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3'
 * '<S254>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer'
 * '<S255>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer1'
 * '<S256>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer2'
 * '<S257>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer3'
 * '<S258>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S259>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S260>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S261>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S262>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment '
 * '<S263>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment'
 * '<S264>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect'
 * '<S265>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/坐标系转换'
 * '<S266>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment /Compare To Constant'
 * '<S267>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment /Compare To Constant1'
 * '<S268>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer'
 * '<S269>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer1'
 * '<S270>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer2'
 * '<S271>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer3'
 * '<S272>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断'
 * '<S273>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断'
 * '<S274>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断'
 * '<S275>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant'
 * '<S276>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant1'
 * '<S277>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant2'
 * '<S278>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant3'
 * '<S279>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter'
 * '<S280>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1'
 * '<S281>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2'
 * '<S282>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3'
 * '<S283>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic'
 * '<S284>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Saturation Dynamic'
 * '<S285>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S286>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic'
 * '<S287>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Saturation Dynamic'
 * '<S288>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S289>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic'
 * '<S290>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Saturation Dynamic'
 * '<S291>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S292>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic'
 * '<S293>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Saturation Dynamic'
 * '<S294>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S295>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant'
 * '<S296>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant1'
 * '<S297>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant2'
 * '<S298>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant3'
 * '<S299>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter'
 * '<S300>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1'
 * '<S301>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2'
 * '<S302>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3'
 * '<S303>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic'
 * '<S304>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Saturation Dynamic'
 * '<S305>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S306>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic'
 * '<S307>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Saturation Dynamic'
 * '<S308>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S309>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic'
 * '<S310>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Saturation Dynamic'
 * '<S311>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S312>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic'
 * '<S313>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Saturation Dynamic'
 * '<S314>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S315>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant'
 * '<S316>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant1'
 * '<S317>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant2'
 * '<S318>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant3'
 * '<S319>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant'
 * '<S320>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant1'
 * '<S321>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter'
 * '<S322>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic'
 * '<S323>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1'
 * '<S324>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2'
 * '<S325>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Subsystem'
 * '<S326>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Timer1'
 * '<S327>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Timer2'
 * '<S328>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic'
 * '<S329>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Saturation Dynamic'
 * '<S330>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S331>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S332>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S333>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S334>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper'
 * '<S335>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2AMKMCUFL'
 * '<S336>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2AMKMCUFR'
 * '<S337>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU'
 * '<S338>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/WP_OUTPUT'
 * '<S339>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/MeaModule'
 * '<S340>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU/MeaModule1'
 * '<S341>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU/MeaModule2'
 * '<S342>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL'
 * '<S343>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ'
 * '<S344>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/EEPROM'
 * '<S345>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling'
 * '<S346>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem'
 * '<S347>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem'
 * '<S348>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem'
 * '<S349>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com0'
 * '<S350>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com1'
 * '<S351>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com2'
 * '<S352>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem'
 * '<S353>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem1'
 * '<S354>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq100ms'
 * '<S355>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq10ms'
 * '<S356>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq500ms'
 * '<S357>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq50ms'
 * '<S358>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq5ms'
 * '<S359>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/EEPROM/EEPROMOperation'
 * '<S360>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling/CCPBackground'
 * '<S361>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling/CCPReceive'
 * '<S362>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling/CCPReceive/Nothing'
 */
#endif                   /* RTW_HEADER_VehCtrlMdel240926_2018b_amkspdlimit_h_ */

/* File trailer for ECUCoder generated file VehCtrlMdel240926_2018b_amkspdlimit.h.
 *
 * [EOF]
 */
