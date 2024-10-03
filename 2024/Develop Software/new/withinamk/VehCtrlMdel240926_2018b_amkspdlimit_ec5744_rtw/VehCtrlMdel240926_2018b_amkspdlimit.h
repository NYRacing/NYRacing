/*
 * Code generated for Simulink model VehCtrlMdel240926_2018b_amkspdlimit.
 *
 * FILE    : VehCtrlMdel240926_2018b_amkspdlimit.h
 *
 * VERSION : 1.212
 *
 * DATE    : Wed Oct  2 16:46:03 2024
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

/* Block states (default storage) for system '<S118>/Timer' */
typedef struct {
  real_T x;                            /* '<S118>/Timer' */
  struct {
    uint_T is_c21_VehCtrlMdel240926_2018b_:2;/* '<S118>/Timer' */
    uint_T is_active_c21_VehCtrlMdel240926:1;/* '<S118>/Timer' */
  } bitsForTID3;
} DW_Timer_VehCtrlMdel240926_20_T;

/* Block signals (default storage) */
typedef struct {
  CAN_DATATYPE CANPack1;               /* '<S340>/CAN Pack1' */
  CAN_DATATYPE CANPack1_a;             /* '<S341>/CAN Pack1' */
  CAN_DATATYPE CANPack1_d;             /* '<S339>/CAN Pack1' */
  CAN_DATATYPE CANUnPackMessage4;      /* '<S178>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_g;    /* '<S172>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_j;    /* '<S149>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_n;    /* '<S160>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_p;    /* '<S158>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_e;    /* '<S130>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_c;    /* '<S142>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_b;    /* '<S140>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_l;    /* '<S191>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_ja;   /* '<S120>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_h;    /* '<S186>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_i;    /* '<S169>/CANUnPackMessage4' */
  real_T Switch1;                      /* '<S340>/Switch1' */
  real_T Switch;                       /* '<S340>/Switch' */
  real_T Switch1_l;                    /* '<S339>/Switch1' */
  real_T MCFL_TorqueLimitP;            /* '<S339>/Switch' */
  real_T Exit;                         /* '<S268>/Timer2' */
  real_T Exit_l;                       /* '<S268>/Timer1' */
  real_T Exit_a;                       /* '<S267>/Timer3' */
  real_T Exit_lh;                      /* '<S267>/Timer2' */
  real_T Exit_lh4;                     /* '<S267>/Timer1' */
  real_T Exit_c;                       /* '<S267>/Timer' */
  real_T Exit_h;                       /* '<S206>/Timer3' */
  real_T Exit_o;                       /* '<S206>/Timer2' */
  real_T Exit_i;                       /* '<S206>/Timer1' */
  real_T Exit_le;                      /* '<S206>/Timer' */
  real_T Exit_on;                      /* '<S204>/Timer' */
  real_T low_VOL;                      /* '<S178>/CAN Unpack' */
  real_T MCU_Temp_error;               /* '<S178>/CAN Unpack' */
  real_T Mode;                         /* '<S178>/CAN Unpack' */
  real_T motorTemp_error;              /* '<S178>/CAN Unpack' */
  real_T overCurrent;                  /* '<S178>/CAN Unpack' */
  real_T overpower;                    /* '<S178>/CAN Unpack' */
  real_T overvol;                      /* '<S178>/CAN Unpack' */
  real_T Precharge;                    /* '<S178>/CAN Unpack' */
  real_T Reslove_error;                /* '<S178>/CAN Unpack' */
  real_T MCFR_bDerating;               /* '<S149>/CAN Unpack' */
  real_T MCFR_bQuitDCOn;               /* '<S149>/CAN Unpack' */
  real_T MCFR_bReserve;                /* '<S149>/CAN Unpack' */
  real_T MCFR_bWarn;                   /* '<S149>/CAN Unpack' */
  real_T MCFR_DiagnosticNum;           /* '<S158>/CAN Unpack' */
  real_T MCFL_ActualVelocity_p;        /* '<S130>/CAN Unpack' */
  real_T MCFL_bDerating;               /* '<S130>/CAN Unpack' */
  real_T MCFL_bReserve;                /* '<S130>/CAN Unpack' */
  real_T MCFL_bWarn;                   /* '<S130>/CAN Unpack' */
  real_T MCFL_DiagnosticNum;           /* '<S140>/CAN Unpack' */
  real_T CANUnpack_o1;                 /* '<S186>/CAN Unpack' */
  real_T CANUnpack_o3;                 /* '<S186>/CAN Unpack' */
  real_T CANUnpack_o5;                 /* '<S186>/CAN Unpack' */
  real_T CANUnpack_o6;                 /* '<S186>/CAN Unpack' */
  real_T errorReset;                   /* '<S101>/Chart2' */
  real_T Exit_d;                       /* '<S8>/Timer2' */
  real_T Exit_g;                       /* '<S8>/Timer1' */
  real_T DYC_Enable_OUT;               /* '<S7>/Chart' */
  real_T TCSR_Enable_OUT;              /* '<S7>/Chart' */
  real_T TCSF_Enable_OUT;              /* '<S7>/Chart' */
  uint32_T CANReceive_o3;              /* '<S366>/CANReceive' */
  uint32_T CANReceive_o3_l;            /* '<S351>/CANReceive' */
  uint32_T CANReceive1_o3;             /* '<S115>/CANReceive1' */
  uint32_T CANReceive3_o3;             /* '<S115>/CANReceive3' */
  uint32_T CANReceive3_o3_e;           /* '<S125>/CANReceive3' */
  uint32_T CANReceive1_o3_n;           /* '<S125>/CANReceive1' */
  uint32_T CANReceive2_o3;             /* '<S125>/CANReceive2' */
  uint32_T CANReceive3_o3_i;           /* '<S126>/CANReceive3' */
  uint32_T CANReceive1_o3_h;           /* '<S126>/CANReceive1' */
  uint32_T CANReceive2_o3_j;           /* '<S126>/CANReceive2' */
  uint32_T CANReceive3_o3_c;           /* '<S117>/CANReceive3' */
  uint32_T CANReceive3_o3_m;           /* '<S111>/CANReceive3' */
  uint32_T CANReceive3_o3_cz;          /* '<S116>/CANReceive3' */
  uint32_T CANReceive3_o3_l;           /* '<S114>/CANReceive3' */
  real32_T TrqFR_cmd;                  /* '<S7>/Gain1' */
  int32_T DataTypeConversion2;         /* '<S341>/Data Type Conversion2' */
  uint16_T CastToSingle1;              /* '<S342>/Cast To Single1' */
  uint16_T CastToBoolean4;             /* '<S341>/Cast To Boolean4' */
  uint16_T CastToBoolean6;             /* '<S341>/Cast To Boolean6' */
  uint8_T CANReceive_o2;               /* '<S366>/CANReceive' */
  uint8_T CANReceive_o4[8];            /* '<S366>/CANReceive' */
  uint8_T CANReceive_o5;               /* '<S366>/CANReceive' */
  uint8_T CANReceive_o2_p;             /* '<S351>/CANReceive' */
  uint8_T CANReceive_o4_i[8];          /* '<S351>/CANReceive' */
  uint8_T CANReceive_o5_l;             /* '<S351>/CANReceive' */
  uint8_T CANTransmit;                 /* '<S358>/CANTransmit' */
  uint8_T CANPackMessage[8];           /* '<S340>/CANPackMessage' */
  uint8_T CANTransmit_l;               /* '<S340>/CANTransmit' */
  uint8_T CANPackMessage_d[8];         /* '<S341>/CANPackMessage' */
  uint8_T CANTransmit_k;               /* '<S341>/CANTransmit' */
  uint8_T CANPackMessage_h[8];         /* '<S339>/CANPackMessage' */
  uint8_T CANTransmit_c;               /* '<S339>/CANTransmit' */
  uint8_T CANReceive1_o2;              /* '<S115>/CANReceive1' */
  uint8_T CANReceive1_o4[8];           /* '<S115>/CANReceive1' */
  uint8_T CANReceive1_o5;              /* '<S115>/CANReceive1' */
  uint8_T CANReceive3_o2;              /* '<S115>/CANReceive3' */
  uint8_T CANReceive3_o4[8];           /* '<S115>/CANReceive3' */
  uint8_T CANReceive3_o5;              /* '<S115>/CANReceive3' */
  uint8_T CANReceive3_o2_l;            /* '<S125>/CANReceive3' */
  uint8_T CANReceive3_o4_l[8];         /* '<S125>/CANReceive3' */
  uint8_T CANReceive3_o5_a;            /* '<S125>/CANReceive3' */
  uint8_T CANReceive1_o2_l;            /* '<S125>/CANReceive1' */
  uint8_T CANReceive1_o4_c[8];         /* '<S125>/CANReceive1' */
  uint8_T CANReceive1_o5_a;            /* '<S125>/CANReceive1' */
  uint8_T CANReceive2_o2;              /* '<S125>/CANReceive2' */
  uint8_T CANReceive2_o4[6];           /* '<S125>/CANReceive2' */
  uint8_T CANReceive2_o5;              /* '<S125>/CANReceive2' */
  uint8_T CANReceive3_o2_a;            /* '<S126>/CANReceive3' */
  uint8_T CANReceive3_o4_g[8];         /* '<S126>/CANReceive3' */
  uint8_T CANReceive3_o5_an;           /* '<S126>/CANReceive3' */
  uint8_T CANReceive1_o2_o;            /* '<S126>/CANReceive1' */
  uint8_T CANReceive1_o4_j[8];         /* '<S126>/CANReceive1' */
  uint8_T CANReceive1_o5_j;            /* '<S126>/CANReceive1' */
  uint8_T CANReceive2_o2_p;            /* '<S126>/CANReceive2' */
  uint8_T CANReceive2_o4_k[6];         /* '<S126>/CANReceive2' */
  uint8_T CANReceive2_o5_e;            /* '<S126>/CANReceive2' */
  uint8_T CANReceive3_o2_p;            /* '<S117>/CANReceive3' */
  uint8_T CANReceive3_o4_k[8];         /* '<S117>/CANReceive3' */
  uint8_T CANReceive3_o5_d;            /* '<S117>/CANReceive3' */
  uint8_T CANReceive3_o2_m;            /* '<S111>/CANReceive3' */
  uint8_T CANReceive3_o4_lg[8];        /* '<S111>/CANReceive3' */
  uint8_T CANReceive3_o5_b;            /* '<S111>/CANReceive3' */
  uint8_T CANReceive3_o2_ma;           /* '<S116>/CANReceive3' */
  uint8_T CANReceive3_o4_i[8];         /* '<S116>/CANReceive3' */
  uint8_T CANReceive3_o5_m;            /* '<S116>/CANReceive3' */
  uint8_T CANReceive3_o2_k;            /* '<S114>/CANReceive3' */
  uint8_T CANReceive3_o4_p[8];         /* '<S114>/CANReceive3' */
  uint8_T CANReceive3_o5_de;           /* '<S114>/CANReceive3' */
  boolean_T NOT;                       /* '<S338>/NOT' */
  boolean_T Drive_ready;               /* '<S118>/SwitchInput' */
  boolean_T MCFL_DCOn_setpoints_o;     /* '<S101>/Switch4' */
  boolean_T VehReady;                  /* '<S101>/Chart2' */
  boolean_T MCFL_DCEnable;             /* '<S101>/Chart2' */
  boolean_T MCFR_TorqueOn;             /* '<S101>/Chart2' */
  boolean_T MCFL_TorqueOn;             /* '<S101>/Chart2' */
  boolean_T AMKMCFL_InverterOn;        /* '<S101>/Chart2' */
  boolean_T aWaterPumpON;
} B_VehCtrlMdel240926_2018b_amk_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DelayInput2_DSTATE;           /* '<S238>/Delay Input2' */
  real_T DelayInput2_DSTATE_a;         /* '<S253>/Delay Input2' */
  real_T DelayInput2_DSTATE_b;         /* '<S254>/Delay Input2' */
  real_T DelayInput2_DSTATE_h;         /* '<S255>/Delay Input2' */
  real_T DelayInput2_DSTATE_n;         /* '<S256>/Delay Input2' */
  real_T UnitDelay_DSTATE;             /* '<S204>/Unit Delay' */
  real_T DelayInput2_DSTATE_l;         /* '<S228>/Delay Input2' */
  real_T UnitDelay_DSTATE_g;           /* '<S227>/Unit Delay' */
  real_T DelayInput2_DSTATE_m;         /* '<S229>/Delay Input2' */
  real_T UnitDelay1_DSTATE;            /* '<S227>/Unit Delay1' */
  real_T DelayInput2_DSTATE_j;         /* '<S239>/Delay Input2' */
  real_T DelayInput2_DSTATE_k;         /* '<S230>/Delay Input2' */
  real_T UnitDelay2_DSTATE;            /* '<S227>/Unit Delay2' */
  real_T UnitDelay4_DSTATE;            /* '<S30>/Unit Delay4' */
  real_T UnitDelay4_DSTATE_b;          /* '<S31>/Unit Delay4' */
  real_T UnitDelay4_DSTATE_l;          /* '<S29>/Unit Delay4' */
  real_T UnitDelay_DSTATE_n;           /* '<S7>/Unit Delay' */
  real_T UnitDelay2_DSTATE_g;          /* '<S31>/Unit Delay2' */
  real_T UnitDelay2_DSTATE_j;          /* '<S30>/Unit Delay2' */
  real_T UnitDelay2_DSTATE_b;          /* '<S29>/Unit Delay2' */
  real_T x;                            /* '<S118>/Timer1' */
  real_T b;                            /* '<S7>/Chart' */
  real_T DYC_flag;                     /* '<S7>/Chart' */
  real32_T UnitDelay_DSTATE_p;         /* '<S283>/Unit Delay' */
  real32_T DelayInput2_DSTATE_n2;      /* '<S287>/Delay Input2' */
  real32_T UnitDelay_DSTATE_j;         /* '<S276>/Unit Delay' */
  real32_T UnitDelay_DSTATE_pj;        /* '<S284>/Unit Delay' */
  real32_T DelayInput2_DSTATE_e;       /* '<S290>/Delay Input2' */
  real32_T UnitDelay1_DSTATE_n;        /* '<S276>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_a;         /* '<S285>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hk;      /* '<S293>/Delay Input2' */
  real32_T UnitDelay2_DSTATE_l;        /* '<S276>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_nc;        /* '<S286>/Unit Delay' */
  real32_T DelayInput2_DSTATE_c;       /* '<S296>/Delay Input2' */
  real32_T UnitDelay3_DSTATE;          /* '<S276>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_l;         /* '<S303>/Unit Delay' */
  real32_T DelayInput2_DSTATE_i;       /* '<S307>/Delay Input2' */
  real32_T UnitDelay4_DSTATE_i;        /* '<S277>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_a0;        /* '<S277>/Unit Delay' */
  real32_T UnitDelay_DSTATE_ap;        /* '<S304>/Unit Delay' */
  real32_T DelayInput2_DSTATE_el;      /* '<S310>/Delay Input2' */
  real32_T UnitDelay5_DSTATE;          /* '<S277>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_am;       /* '<S277>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_o;         /* '<S305>/Unit Delay' */
  real32_T DelayInput2_DSTATE_p;       /* '<S313>/Delay Input2' */
  real32_T UnitDelay6_DSTATE;          /* '<S277>/Unit Delay6' */
  real32_T UnitDelay2_DSTATE_c;        /* '<S277>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_ah;        /* '<S306>/Unit Delay' */
  real32_T DelayInput2_DSTATE_mt;      /* '<S316>/Delay Input2' */
  real32_T UnitDelay7_DSTATE;          /* '<S277>/Unit Delay7' */
  real32_T UnitDelay3_DSTATE_d;        /* '<S277>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_lh;        /* '<S201>/Unit Delay' */
  real32_T UnitDelay4_DSTATE_m;        /* '<S265>/Unit Delay4' */
  real32_T UnitDelay1_DSTATE_k;        /* '<S201>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_o;        /* '<S265>/Unit Delay1' */
  real32_T UnitDelay3_DSTATE_p;        /* '<S268>/Unit Delay3' */
  real32_T UnitDelay1_DSTATE_d;        /* '<S268>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_f;        /* '<S268>/Unit Delay2' */
  real32_T UnitDelay4_DSTATE_mn;       /* '<S268>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_d;         /* '<S265>/Unit Delay' */
  real32_T UnitDelay2_DSTATE_i;        /* '<S265>/Unit Delay2' */
  real32_T DelayInput2_DSTATE_g;       /* '<S328>/Delay Input2' */
  real32_T DelayInput2_DSTATE_af;      /* '<S326>/Delay Input2' */
  real32_T DelayInput2_DSTATE_f;       /* '<S332>/Delay Input2' */
  real32_T UnitDelay_DSTATE_ncs;       /* '<S325>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hu;      /* '<S327>/Delay Input2' */
  real32_T UnitDelay1_DSTATE_n5;       /* '<S78>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_i;        /* '<S89>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_h;        /* '<S69>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_d;       /* '<S46>/Delay Input2' */
  real32_T UnitDelay4_DSTATE_n;        /* '<S10>/Unit Delay4' */
  real32_T UnitDelay5_DSTATE_l;        /* '<S31>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_b;         /* '<S31>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_g;        /* '<S31>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_cd;      /* '<S19>/Delay Input2' */
  real32_T UnitDelay5_DSTATE_i;        /* '<S30>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_f;         /* '<S30>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_f;        /* '<S30>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_hn;      /* '<S20>/Delay Input2' */
  real32_T UnitDelay5_DSTATE_ip;       /* '<S29>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_nr;        /* '<S29>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_g4;       /* '<S29>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_ib;      /* '<S21>/Delay Input2' */
  int32_T sfEvent;                     /* '<S101>/Chart2' */
  uint32_T Subsystem_PREV_T;           /* '<S4>/Subsystem' */
  uint32_T FunctionCallSubsystem_PREV_T;/* '<S4>/Function-Call Subsystem' */
  uint32_T previousTicks;              /* '<S101>/Chart2' */
  uint32_T MoTrqReq_PREV_T;            /* '<S1>/MoTrqReq' */
  int_T CANPack1_ModeSignalID;         /* '<S340>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_l;       /* '<S341>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_f;       /* '<S339>/CAN Pack1' */
  int_T CANUnpack_ModeSignalID;        /* '<S178>/CAN Unpack' */
  int_T CANUnpack_StatusPortID;        /* '<S178>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_b;      /* '<S172>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_m;      /* '<S172>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_a;      /* '<S149>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_o;      /* '<S149>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_n;      /* '<S160>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_f;      /* '<S160>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_p;      /* '<S158>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_e;      /* '<S158>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_c;      /* '<S130>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_ok;     /* '<S130>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_m;      /* '<S142>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_j;      /* '<S142>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_i;      /* '<S140>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_mj;     /* '<S140>/CAN Unpack' */
  int_T CANUnpack1_ModeSignalID;       /* '<S191>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID;       /* '<S191>/CAN Unpack1' */
  int_T CANUnpack1_ModeSignalID_m;     /* '<S120>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID_n;     /* '<S120>/CAN Unpack1' */
  int_T CANUnpack_ModeSignalID_f;      /* '<S186>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_k;      /* '<S186>/CAN Unpack' */
  struct {
    uint_T is_VehStat:4;               /* '<S101>/Chart2' */
    uint_T is_AMKDCready:4;            /* '<S101>/Chart2' */
    uint_T is_c23_VehCtrlMdel240926_2018b_:2;/* '<S118>/Timer1' */
    uint_T is_BeeperStat:2;            /* '<S101>/Chart2' */
    uint_T is_AMKDCon:2;               /* '<S101>/Chart2' */
    uint_T is_MCDCEnable:2;            /* '<S101>/Chart2' */
    uint_T is_MC_TorqueCUT:2;          /* '<S101>/Chart2' */
    uint_T is_AMKCANenable:2;          /* '<S101>/Chart2' */
    uint_T is_MCFL_InverterOn:2;       /* '<S101>/Chart2' */
    uint_T is_MCFR_InverterOn:2;       /* '<S101>/Chart2' */
    uint_T is_B:2;                     /* '<S7>/Chart' */
    uint_T is_C:2;                     /* '<S7>/Chart' */
    uint_T is_D:2;                     /* '<S7>/Chart' */
    uint_T is_E:2;                     /* '<S7>/Chart' */
    uint_T is_active_c23_VehCtrlMdel240926:1;/* '<S118>/Timer1' */
    uint_T is_active_c1_VehCtrlMdel240926_:1;/* '<S101>/Chart2' */
    uint_T is_active_VehStat:1;        /* '<S101>/Chart2' */
    uint_T is_active_BeeperStat:1;     /* '<S101>/Chart2' */
    uint_T is_active_AMKDCon:1;        /* '<S101>/Chart2' */
    uint_T is_active_MCDCEnable:1;     /* '<S101>/Chart2' */
    uint_T is_active_MC_TorqueCUT:1;   /* '<S101>/Chart2' */
    uint_T is_active_AMKDCready:1;     /* '<S101>/Chart2' */
    uint_T is_active_Output:1;         /* '<S101>/Chart2' */
    uint_T is_active_AMKCANenable:1;   /* '<S101>/Chart2' */
    uint_T is_active_MCFL_InverterOn:1;/* '<S101>/Chart2' */
    uint_T is_active_MCFR_InverterOn:1;/* '<S101>/Chart2' */
    uint_T is_active_c7_VehCtrlMdel240926_:1;/* '<S7>/Chart' */
  } bitsForTID3;

  uint16_T UnitDelay1_DSTATE_fm;       /* '<S202>/Unit Delay1' */
  uint16_T UnitDelay_DSTATE_k;         /* '<S202>/Unit Delay' */
  uint16_T temporalCounter_i2;         /* '<S101>/Chart2' */
  boolean_T UnitDelay3_DSTATE_f;       /* '<S265>/Unit Delay3' */
  boolean_T UnitDelay1_DSTATE_gl;      /* '<S77>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_e;       /* '<S30>/Unit Delay3' */
  boolean_T UnitDelay1_DSTATE_e;       /* '<S88>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_a;       /* '<S31>/Unit Delay3' */
  boolean_T UnitDelay1_DSTATE_dp;      /* '<S68>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_i;       /* '<S29>/Unit Delay3' */
  boolean_T UnitDelay3_DSTATE_l;       /* '<S10>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE;        /* '<S87>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_j;      /* '<S76>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_b;      /* '<S67>/Delay Input1' */
  uint8_T temporalCounter_i1;          /* '<S101>/Chart2' */
  boolean_T Subsystem_RESET_ELAPS_T;   /* '<S4>/Subsystem' */
  boolean_T FunctionCallSubsystem_RESET_ELA;/* '<S4>/Function-Call Subsystem' */
  boolean_T MoTrqReq_RESET_ELAPS_T;    /* '<S1>/MoTrqReq' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2_j;/* '<S268>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1_p;/* '<S268>/Timer1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer3_i;/* '<S267>/Timer3' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2_g;/* '<S267>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1_m;/* '<S267>/Timer1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer_o;/* '<S267>/Timer' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer3;/* '<S206>/Timer3' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2_l;/* '<S206>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1_n;/* '<S206>/Timer1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer_b;/* '<S206>/Timer' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer_k;/* '<S204>/Timer' */
  DW_Timer_VehCtrlMdel240926_20_T sf_Timer_a;/* '<S202>/Timer' */
  DW_Timer_VehCtrlMdel240926_20_T sf_Timer;/* '<S118>/Timer' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2;/* '<S8>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1;/* '<S8>/Timer1' */
} DW_VehCtrlMdel240926_2018b_am_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Expression: [0,0,0.2,0.4,0.6,0.8,1,1])
   * Referenced by:
   *   '<S8>/2-D Lookup Table1'
   *   '<S8>/2-D Lookup Table3'
   */
  real_T pooled9[8];

  /* Pooled Parameter (Expression: [35,40,41,42,43,44,45,50])
   * Referenced by:
   *   '<S8>/2-D Lookup Table1'
   *   '<S8>/2-D Lookup Table3'
   */
  real_T pooled10[8];

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
   *   '<S29>/VehSpd_SlipTarget_mps'
   *   '<S30>/VehSpd_SlipTarget_mps'
   *   '<S31>/VehSpd_SlipTarget_mps'
   */
  real32_T pooled34[4];

  /* Pooled Parameter (Expression: [0,3,25,30])
   * Referenced by:
   *   '<S29>/VehSpd_SlipTarget_mps'
   *   '<S29>/VehicleStableTarget_mps'
   *   '<S29>/VehicleStableTarget_mps1'
   *   '<S30>/VehSpd_SlipTarget_mps'
   *   '<S30>/VehicleStableTarget_mps'
   *   '<S30>/VehicleStableTarget_mps1'
   *   '<S31>/VehSpd_SlipTarget_mps'
   *   '<S31>/VehicleStableTarget_mps'
   *   '<S31>/VehicleStableTarget_mps1'
   */
  real32_T pooled35[4];

  /* Pooled Parameter (Expression: [0.4,0.4,1.2,1.2])
   * Referenced by:
   *   '<S29>/VehicleStableTarget_mps'
   *   '<S29>/VehicleStableTarget_mps1'
   *   '<S30>/VehicleStableTarget_mps'
   *   '<S30>/VehicleStableTarget_mps1'
   *   '<S31>/VehicleStableTarget_mps'
   *   '<S31>/VehicleStableTarget_mps1'
   */
  real32_T pooled51[4];

  /* Expression: single([20,0]);
   * Referenced by: '<S7>/BrakeCompensateCoefFront1'
   */
  real32_T BrakeCompensateCoefFront1_table[2];

  /* Expression: single([550,1700]);
   * Referenced by: '<S7>/BrakeCompensateCoefFront1'
   */
  real32_T BrakeCompensateCoefFront1_bp01D[2];

  /* Pooled Parameter (Expression: single([500,4500]);)
   * Referenced by: '<S202>/1-D Lookup Table1'
   */
  real32_T pooled57[2];

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
extern real_T Gear_Trs;                /* '<S341>/Switch2' */
extern real_T Mode_Trs;                /* '<S341>/Switch3' */
extern real_T AMKFL_Current;           /* '<S205>/Switch' */
extern real_T AMKFR_Current;           /* '<S205>/Switch1' */
extern real_T Trq_CUT;                 /* '<S202>/Timer' */
extern real_T AMKSWITCH;               /* '<S118>/Timer1' */
extern real_T ignition;                /* '<S118>/Timer' */
extern real_T L12V_error;              /* '<S178>/CAN Unpack' */
extern real_T alarm;                   /* '<S178>/CAN Unpack' */
extern real_T controller_ready;        /* '<S178>/CAN Unpack' */
extern real_T selfcheck;               /* '<S178>/CAN Unpack' */
extern real_T RPM;                     /* '<S178>/CAN Unpack' */
extern real_T trq;                     /* '<S178>/CAN Unpack' */
extern real_T AC_current;              /* '<S172>/CAN Unpack' */
extern real_T DC_current;              /* '<S172>/CAN Unpack' */
extern real_T MCU_Temp;                /* '<S172>/CAN Unpack' */
extern real_T motor_Temp;              /* '<S172>/CAN Unpack' */
extern real_T voltage;                 /* '<S172>/CAN Unpack' */
extern real_T MCFR_ActualTorque;       /* '<S149>/CAN Unpack' */
extern real_T MCFR_ActualVelocity;     /* '<S149>/CAN Unpack' */
extern real_T MCFR_DCVoltage;          /* '<S149>/CAN Unpack' */
extern real_T MCFR_bDCOn;              /* '<S149>/CAN Unpack' */
extern real_T MCFR_bError;             /* '<S149>/CAN Unpack' */
extern real_T MCFR_bInverterOn;        /* '<S149>/CAN Unpack' */
extern real_T MCFR_bQuitInverterOn;    /* '<S149>/CAN Unpack' */
extern real_T MCFR_bSystemReady;       /* '<S149>/CAN Unpack' */
extern real_T MCFR_TempIGBT;           /* '<S160>/CAN Unpack' */
extern real_T MCFR_TempInverter;       /* '<S160>/CAN Unpack' */
extern real_T MCFR_TempMotor;          /* '<S160>/CAN Unpack' */
extern real_T MCFR_ErrorInfo;          /* '<S158>/CAN Unpack' */
extern real_T MCFL_ActualTorque;       /* '<S130>/CAN Unpack' */
extern real_T MCFL_DCVoltage;          /* '<S130>/CAN Unpack' */
extern real_T MCFL_bDCOn;              /* '<S130>/CAN Unpack' */
extern real_T MCFL_bError;             /* '<S130>/CAN Unpack' */
extern real_T MCFL_bInverterOn;        /* '<S130>/CAN Unpack' */
extern real_T MCFL_bQuitDCOn;          /* '<S130>/CAN Unpack' */
extern real_T MCFL_bQuitInverterOn;    /* '<S130>/CAN Unpack' */
extern real_T MCFL_bSystemReady;       /* '<S130>/CAN Unpack' */
extern real_T MCFL_ActualVelocity;     /* '<S130>/Gain' */
extern real_T MCFL_TempIGBT;           /* '<S142>/CAN Unpack' */
extern real_T MCFL_TempInverter;       /* '<S142>/CAN Unpack' */
extern real_T MCFL_TempMotor;          /* '<S142>/CAN Unpack' */
extern real_T MCFL_ErrorInfo;          /* '<S140>/CAN Unpack' */
extern real_T StrWhlAngAliveRollCnt;   /* '<S191>/CAN Unpack1' */
extern real_T StrWhlAng;               /* '<S191>/CAN Unpack1' */
extern real_T StrWhlAngV;              /* '<S191>/CAN Unpack1' */
extern real_T ABS_WS_FL;               /* '<S120>/CAN Unpack1' */
extern real_T ABS_WS_FR;               /* '<S120>/CAN Unpack1' */
extern real_T ABS_WS_RL;               /* '<S120>/CAN Unpack1' */
extern real_T ABS_WS_RR;               /* '<S120>/CAN Unpack1' */
extern real_T IMU_Ay_Value;            /* '<S186>/CAN Unpack' */
extern real_T IMU_Ax_Value;            /* '<S186>/CAN Unpack' */
extern real_T IMU_Yaw_Value;           /* '<S186>/CAN Unpack' */
extern real_T EMRAX_Trq_CUT;           /*  */
extern real_T AMK_Trq_CUT;             /*  */
extern uint32_T Acc_vol2;              /* '<S202>/Add3' */
extern uint32_T Acc_vol;               /* '<S202>/Add2' */
extern uint32_T Acc_POS2;              /* '<S202>/1-D Lookup Table3' */
extern real32_T VehVxEst_mps;          /* '<S325>/Add' */
extern real32_T Acc_POS;               /* '<S202>/MATLAB Function' */
extern real32_T EmraxTrqR_cmd;         /* '<S7>/Saturation1' */
extern real32_T AMKTrqFR_cmd;          /* '<S7>/Saturation2' */
extern real32_T AMKTrqFL_cmd;          /* '<S7>/Saturation3' */
extern uint16_T F_BrkPrs;              /* '<S202>/1-D Lookup Table1' */
extern uint16_T Acc1;                  /* '<S113>/Acc3' */
extern uint16_T Acc2;                  /* '<S113>/Acc4' */
extern uint16_T Brk1;                  /* '<S113>/Brk1' */
extern uint16_T Brk2;                  /* '<S113>/Brk2' */
extern boolean_T HVCUTOFF;             /* '<S118>/Constant' */
extern boolean_T KeyPressed;           /* '<S101>/Cast To Boolean' */
extern boolean_T Brk;                  /* '<S103>/Compare' */
extern boolean_T ACC_Release;          /* '<S104>/Compare' */
extern boolean_T beeper_state;         /* '<S101>/Chart2' */
extern boolean_T MCFL_DCOn_setpoints;  /* '<S101>/Chart2' */
extern boolean_T MCFR_DCEnable;        /* '<S101>/Chart2' */
extern boolean_T MCFR_InverterOn;      /* '<S101>/Chart2' */
extern boolean_T TrqR_cmd_raw;         /* '<S7>/Logical Operator1' */
extern boolean_T TroqueOn;             /* '<S7>/Logical Operator6' */
extern boolean_T Trq_CUT_final;        /* '<S7>/Logical Operator4' */

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
 * Block '<S10>/AND1' : Unused code path elimination
 * Block '<S10>/AND2' : Unused code path elimination
 * Block '<S10>/Add1' : Unused code path elimination
 * Block '<S10>/Add11' : Unused code path elimination
 * Block '<S10>/Add12' : Unused code path elimination
 * Block '<S10>/Add13' : Unused code path elimination
 * Block '<S10>/Add14' : Unused code path elimination
 * Block '<S10>/Add15' : Unused code path elimination
 * Block '<S10>/Add2' : Unused code path elimination
 * Block '<S10>/Add3' : Unused code path elimination
 * Block '<S10>/Add4' : Unused code path elimination
 * Block '<S10>/Add5' : Unused code path elimination
 * Block '<S10>/Add6' : Unused code path elimination
 * Block '<S10>/Add7' : Unused code path elimination
 * Block '<S10>/Add8' : Unused code path elimination
 * Block '<S10>/Add9' : Unused code path elimination
 * Block '<S32>/Compare' : Unused code path elimination
 * Block '<S32>/Constant' : Unused code path elimination
 * Block '<S10>/Constant' : Unused code path elimination
 * Block '<S10>/Constant1' : Unused code path elimination
 * Block '<S10>/Constant27' : Unused code path elimination
 * Block '<S10>/Constant28' : Unused code path elimination
 * Block '<S10>/Constant29' : Unused code path elimination
 * Block '<S10>/Constant30' : Unused code path elimination
 * Block '<S10>/Constant31' : Unused code path elimination
 * Block '<S10>/Constant32' : Unused code path elimination
 * Block '<S10>/Constant33' : Unused code path elimination
 * Block '<S10>/Constant34' : Unused code path elimination
 * Block '<S10>/Constant35' : Unused code path elimination
 * Block '<S10>/Constant36' : Unused code path elimination
 * Block '<S10>/Constant37' : Unused code path elimination
 * Block '<S10>/Constant38' : Unused code path elimination
 * Block '<S10>/Constant39' : Unused code path elimination
 * Block '<S10>/Constant40' : Unused code path elimination
 * Block '<S10>/Constant41' : Unused code path elimination
 * Block '<S10>/Constant42' : Unused code path elimination
 * Block '<S10>/Constant45' : Unused code path elimination
 * Block '<S10>/Constant46' : Unused code path elimination
 * Block '<S10>/Constant6' : Unused code path elimination
 * Block '<S10>/Constant7' : Unused code path elimination
 * Block '<S10>/Constant8' : Unused code path elimination
 * Block '<S10>/Cos' : Unused code path elimination
 * Block '<S10>/Cos1' : Unused code path elimination
 * Block '<S10>/Cos2' : Unused code path elimination
 * Block '<S10>/Cos3' : Unused code path elimination
 * Block '<S10>/Divide' : Unused code path elimination
 * Block '<S10>/Divide1' : Unused code path elimination
 * Block '<S10>/Gain' : Unused code path elimination
 * Block '<S10>/Gain1' : Unused code path elimination
 * Block '<S10>/Gain10' : Unused code path elimination
 * Block '<S10>/Gain11' : Unused code path elimination
 * Block '<S10>/Gain12' : Unused code path elimination
 * Block '<S10>/Gain13' : Unused code path elimination
 * Block '<S10>/Gain14' : Unused code path elimination
 * Block '<S10>/Gain15' : Unused code path elimination
 * Block '<S10>/Gain16' : Unused code path elimination
 * Block '<S10>/Gain17' : Unused code path elimination
 * Block '<S10>/Gain18' : Unused code path elimination
 * Block '<S10>/Gain19' : Unused code path elimination
 * Block '<S10>/Gain2' : Unused code path elimination
 * Block '<S10>/Gain20' : Unused code path elimination
 * Block '<S10>/Gain22' : Unused code path elimination
 * Block '<S10>/Gain23' : Unused code path elimination
 * Block '<S10>/Gain24' : Unused code path elimination
 * Block '<S10>/Gain25' : Unused code path elimination
 * Block '<S10>/Gain26' : Unused code path elimination
 * Block '<S10>/Gain4' : Unused code path elimination
 * Block '<S10>/Gain5' : Unused code path elimination
 * Block '<S10>/Gain6' : Unused code path elimination
 * Block '<S10>/Gain7' : Unused code path elimination
 * Block '<S10>/Gain8' : Unused code path elimination
 * Block '<S10>/Gain9' : Unused code path elimination
 * Block '<S10>/InitZORE' : Unused code path elimination
 * Block '<S10>/Logical Operator3' : Unused code path elimination
 * Block '<S10>/Min1' : Unused code path elimination
 * Block '<S10>/NOT' : Unused code path elimination
 * Block '<S10>/NOT1' : Unused code path elimination
 * Block '<S10>/OR1' : Unused code path elimination
 * Block '<S10>/OR2' : Unused code path elimination
 * Block '<S10>/Product' : Unused code path elimination
 * Block '<S10>/Product3' : Unused code path elimination
 * Block '<S10>/Product4' : Unused code path elimination
 * Block '<S10>/Product5' : Unused code path elimination
 * Block '<S40>/Delay Input2' : Unused code path elimination
 * Block '<S40>/Difference Inputs1' : Unused code path elimination
 * Block '<S40>/Difference Inputs2' : Unused code path elimination
 * Block '<S40>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S55>/Data Type Duplicate' : Unused code path elimination
 * Block '<S55>/Data Type Propagation' : Unused code path elimination
 * Block '<S55>/LowerRelop1' : Unused code path elimination
 * Block '<S55>/Switch' : Unused code path elimination
 * Block '<S55>/Switch2' : Unused code path elimination
 * Block '<S55>/UpperRelop' : Unused code path elimination
 * Block '<S40>/delta fall limit' : Unused code path elimination
 * Block '<S40>/delta rise limit' : Unused code path elimination
 * Block '<S40>/sample time' : Unused code path elimination
 * Block '<S41>/Delay Input2' : Unused code path elimination
 * Block '<S41>/Difference Inputs1' : Unused code path elimination
 * Block '<S41>/Difference Inputs2' : Unused code path elimination
 * Block '<S41>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S56>/Data Type Duplicate' : Unused code path elimination
 * Block '<S56>/Data Type Propagation' : Unused code path elimination
 * Block '<S56>/LowerRelop1' : Unused code path elimination
 * Block '<S56>/Switch' : Unused code path elimination
 * Block '<S56>/Switch2' : Unused code path elimination
 * Block '<S56>/UpperRelop' : Unused code path elimination
 * Block '<S41>/delta fall limit' : Unused code path elimination
 * Block '<S41>/delta rise limit' : Unused code path elimination
 * Block '<S41>/sample time' : Unused code path elimination
 * Block '<S42>/Delay Input2' : Unused code path elimination
 * Block '<S42>/Difference Inputs1' : Unused code path elimination
 * Block '<S42>/Difference Inputs2' : Unused code path elimination
 * Block '<S42>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S57>/Data Type Duplicate' : Unused code path elimination
 * Block '<S57>/Data Type Propagation' : Unused code path elimination
 * Block '<S57>/LowerRelop1' : Unused code path elimination
 * Block '<S57>/Switch' : Unused code path elimination
 * Block '<S57>/Switch2' : Unused code path elimination
 * Block '<S57>/UpperRelop' : Unused code path elimination
 * Block '<S42>/delta fall limit' : Unused code path elimination
 * Block '<S42>/delta rise limit' : Unused code path elimination
 * Block '<S42>/sample time' : Unused code path elimination
 * Block '<S43>/Delay Input2' : Unused code path elimination
 * Block '<S43>/Difference Inputs1' : Unused code path elimination
 * Block '<S43>/Difference Inputs2' : Unused code path elimination
 * Block '<S43>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S58>/Data Type Duplicate' : Unused code path elimination
 * Block '<S58>/Data Type Propagation' : Unused code path elimination
 * Block '<S58>/LowerRelop1' : Unused code path elimination
 * Block '<S58>/Switch' : Unused code path elimination
 * Block '<S58>/Switch2' : Unused code path elimination
 * Block '<S58>/UpperRelop' : Unused code path elimination
 * Block '<S43>/delta fall limit' : Unused code path elimination
 * Block '<S43>/delta rise limit' : Unused code path elimination
 * Block '<S43>/sample time' : Unused code path elimination
 * Block '<S44>/Delay Input2' : Unused code path elimination
 * Block '<S44>/Difference Inputs1' : Unused code path elimination
 * Block '<S44>/Difference Inputs2' : Unused code path elimination
 * Block '<S44>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S59>/Data Type Duplicate' : Unused code path elimination
 * Block '<S59>/Data Type Propagation' : Unused code path elimination
 * Block '<S59>/LowerRelop1' : Unused code path elimination
 * Block '<S59>/Switch' : Unused code path elimination
 * Block '<S59>/Switch2' : Unused code path elimination
 * Block '<S59>/UpperRelop' : Unused code path elimination
 * Block '<S44>/delta fall limit' : Unused code path elimination
 * Block '<S44>/delta rise limit' : Unused code path elimination
 * Block '<S44>/sample time' : Unused code path elimination
 * Block '<S45>/Delay Input2' : Unused code path elimination
 * Block '<S45>/Difference Inputs1' : Unused code path elimination
 * Block '<S45>/Difference Inputs2' : Unused code path elimination
 * Block '<S45>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Propagation' : Unused code path elimination
 * Block '<S60>/LowerRelop1' : Unused code path elimination
 * Block '<S60>/Switch' : Unused code path elimination
 * Block '<S60>/Switch2' : Unused code path elimination
 * Block '<S60>/UpperRelop' : Unused code path elimination
 * Block '<S45>/delta fall limit' : Unused code path elimination
 * Block '<S45>/delta rise limit' : Unused code path elimination
 * Block '<S45>/sample time' : Unused code path elimination
 * Block '<S46>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S61>/Data Type Duplicate' : Unused code path elimination
 * Block '<S61>/Data Type Propagation' : Unused code path elimination
 * Block '<S10>/Relational Operator' : Unused code path elimination
 * Block '<S10>/Relational Operator1' : Unused code path elimination
 * Block '<S10>/Saturation' : Unused code path elimination
 * Block '<S47>/Data Type Duplicate' : Unused code path elimination
 * Block '<S47>/Data Type Propagation' : Unused code path elimination
 * Block '<S47>/LowerRelop1' : Unused code path elimination
 * Block '<S47>/Switch' : Unused code path elimination
 * Block '<S47>/Switch2' : Unused code path elimination
 * Block '<S47>/UpperRelop' : Unused code path elimination
 * Block '<S48>/Data Type Duplicate' : Unused code path elimination
 * Block '<S48>/Data Type Propagation' : Unused code path elimination
 * Block '<S48>/LowerRelop1' : Unused code path elimination
 * Block '<S48>/Switch' : Unused code path elimination
 * Block '<S48>/Switch2' : Unused code path elimination
 * Block '<S48>/UpperRelop' : Unused code path elimination
 * Block '<S49>/Data Type Duplicate' : Unused code path elimination
 * Block '<S49>/Data Type Propagation' : Unused code path elimination
 * Block '<S49>/LowerRelop1' : Unused code path elimination
 * Block '<S49>/Switch' : Unused code path elimination
 * Block '<S49>/Switch2' : Unused code path elimination
 * Block '<S49>/UpperRelop' : Unused code path elimination
 * Block '<S50>/Data Type Duplicate' : Unused code path elimination
 * Block '<S50>/Data Type Propagation' : Unused code path elimination
 * Block '<S50>/LowerRelop1' : Unused code path elimination
 * Block '<S50>/Switch' : Unused code path elimination
 * Block '<S50>/Switch2' : Unused code path elimination
 * Block '<S50>/UpperRelop' : Unused code path elimination
 * Block '<S51>/Data Type Duplicate' : Unused code path elimination
 * Block '<S51>/Data Type Propagation' : Unused code path elimination
 * Block '<S51>/LowerRelop1' : Unused code path elimination
 * Block '<S51>/Switch' : Unused code path elimination
 * Block '<S51>/Switch2' : Unused code path elimination
 * Block '<S51>/UpperRelop' : Unused code path elimination
 * Block '<S52>/Data Type Duplicate' : Unused code path elimination
 * Block '<S52>/Data Type Propagation' : Unused code path elimination
 * Block '<S52>/LowerRelop1' : Unused code path elimination
 * Block '<S52>/Switch' : Unused code path elimination
 * Block '<S52>/Switch2' : Unused code path elimination
 * Block '<S52>/UpperRelop' : Unused code path elimination
 * Block '<S10>/Saturation1' : Unused code path elimination
 * Block '<S10>/Saturation2' : Unused code path elimination
 * Block '<S10>/Saturation3' : Unused code path elimination
 * Block '<S10>/Saturation4' : Unused code path elimination
 * Block '<S10>/Switch' : Unused code path elimination
 * Block '<S10>/Switch1' : Unused code path elimination
 * Block '<S10>/Switch2' : Unused code path elimination
 * Block '<S10>/Switch3' : Unused code path elimination
 * Block '<S10>/Switch4' : Unused code path elimination
 * Block '<S10>/Switch5' : Unused code path elimination
 * Block '<S10>/Switch7' : Unused code path elimination
 * Block '<S10>/Switch8' : Unused code path elimination
 * Block '<S10>/Unit Delay' : Unused code path elimination
 * Block '<S10>/Unit Delay1' : Unused code path elimination
 * Block '<S10>/Unit Delay2' : Unused code path elimination
 * Block '<S10>/Unit Delay5' : Unused code path elimination
 * Block '<S10>/Unit Delay6' : Unused code path elimination
 * Block '<S19>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S62>/Data Type Duplicate' : Unused code path elimination
 * Block '<S62>/Data Type Propagation' : Unused code path elimination
 * Block '<S20>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S63>/Data Type Duplicate' : Unused code path elimination
 * Block '<S63>/Data Type Propagation' : Unused code path elimination
 * Block '<S21>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S64>/Data Type Duplicate' : Unused code path elimination
 * Block '<S64>/Data Type Propagation' : Unused code path elimination
 * Block '<S22>/Data Type Duplicate' : Unused code path elimination
 * Block '<S22>/Data Type Propagation' : Unused code path elimination
 * Block '<S23>/Data Type Duplicate' : Unused code path elimination
 * Block '<S23>/Data Type Propagation' : Unused code path elimination
 * Block '<S24>/Data Type Duplicate' : Unused code path elimination
 * Block '<S24>/Data Type Propagation' : Unused code path elimination
 * Block '<S25>/Data Type Duplicate' : Unused code path elimination
 * Block '<S25>/Data Type Propagation' : Unused code path elimination
 * Block '<S26>/Data Type Duplicate' : Unused code path elimination
 * Block '<S26>/Data Type Propagation' : Unused code path elimination
 * Block '<S27>/Data Type Duplicate' : Unused code path elimination
 * Block '<S27>/Data Type Propagation' : Unused code path elimination
 * Block '<S70>/Data Type Duplicate' : Unused code path elimination
 * Block '<S70>/Data Type Propagation' : Unused code path elimination
 * Block '<S71>/Data Type Duplicate' : Unused code path elimination
 * Block '<S71>/Data Type Propagation' : Unused code path elimination
 * Block '<S72>/Data Type Duplicate' : Unused code path elimination
 * Block '<S72>/Data Type Propagation' : Unused code path elimination
 * Block '<S79>/Data Type Duplicate' : Unused code path elimination
 * Block '<S79>/Data Type Propagation' : Unused code path elimination
 * Block '<S80>/Data Type Duplicate' : Unused code path elimination
 * Block '<S80>/Data Type Propagation' : Unused code path elimination
 * Block '<S81>/Data Type Duplicate' : Unused code path elimination
 * Block '<S81>/Data Type Propagation' : Unused code path elimination
 * Block '<S90>/Data Type Duplicate' : Unused code path elimination
 * Block '<S90>/Data Type Propagation' : Unused code path elimination
 * Block '<S91>/Data Type Duplicate' : Unused code path elimination
 * Block '<S91>/Data Type Propagation' : Unused code path elimination
 * Block '<S92>/Data Type Duplicate' : Unused code path elimination
 * Block '<S92>/Data Type Propagation' : Unused code path elimination
 * Block '<S101>/Switch5' : Unused code path elimination
 * Block '<S169>/CAN Unpack1' : Unused code path elimination
 * Block '<S202>/1-D Lookup Table2' : Unused code path elimination
 * Block '<S202>/Abs1' : Unused code path elimination
 * Block '<S202>/Add4' : Unused code path elimination
 * Block '<S218>/Compare' : Unused code path elimination
 * Block '<S218>/Constant' : Unused code path elimination
 * Block '<S228>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S234>/Data Type Duplicate' : Unused code path elimination
 * Block '<S234>/Data Type Propagation' : Unused code path elimination
 * Block '<S229>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S235>/Data Type Duplicate' : Unused code path elimination
 * Block '<S235>/Data Type Propagation' : Unused code path elimination
 * Block '<S230>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S236>/Data Type Duplicate' : Unused code path elimination
 * Block '<S236>/Data Type Propagation' : Unused code path elimination
 * Block '<S231>/Data Type Duplicate' : Unused code path elimination
 * Block '<S231>/Data Type Propagation' : Unused code path elimination
 * Block '<S232>/Data Type Duplicate' : Unused code path elimination
 * Block '<S232>/Data Type Propagation' : Unused code path elimination
 * Block '<S233>/Data Type Duplicate' : Unused code path elimination
 * Block '<S233>/Data Type Propagation' : Unused code path elimination
 * Block '<S238>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S241>/Data Type Duplicate' : Unused code path elimination
 * Block '<S241>/Data Type Propagation' : Unused code path elimination
 * Block '<S239>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S242>/Data Type Duplicate' : Unused code path elimination
 * Block '<S242>/Data Type Propagation' : Unused code path elimination
 * Block '<S253>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S261>/Data Type Duplicate' : Unused code path elimination
 * Block '<S261>/Data Type Propagation' : Unused code path elimination
 * Block '<S254>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S262>/Data Type Duplicate' : Unused code path elimination
 * Block '<S262>/Data Type Propagation' : Unused code path elimination
 * Block '<S255>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S263>/Data Type Duplicate' : Unused code path elimination
 * Block '<S263>/Data Type Propagation' : Unused code path elimination
 * Block '<S256>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S264>/Data Type Duplicate' : Unused code path elimination
 * Block '<S264>/Data Type Propagation' : Unused code path elimination
 * Block '<S287>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S289>/Data Type Duplicate' : Unused code path elimination
 * Block '<S289>/Data Type Propagation' : Unused code path elimination
 * Block '<S288>/Data Type Duplicate' : Unused code path elimination
 * Block '<S288>/Data Type Propagation' : Unused code path elimination
 * Block '<S290>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S292>/Data Type Duplicate' : Unused code path elimination
 * Block '<S292>/Data Type Propagation' : Unused code path elimination
 * Block '<S291>/Data Type Duplicate' : Unused code path elimination
 * Block '<S291>/Data Type Propagation' : Unused code path elimination
 * Block '<S293>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S295>/Data Type Duplicate' : Unused code path elimination
 * Block '<S295>/Data Type Propagation' : Unused code path elimination
 * Block '<S294>/Data Type Duplicate' : Unused code path elimination
 * Block '<S294>/Data Type Propagation' : Unused code path elimination
 * Block '<S296>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S298>/Data Type Duplicate' : Unused code path elimination
 * Block '<S298>/Data Type Propagation' : Unused code path elimination
 * Block '<S297>/Data Type Duplicate' : Unused code path elimination
 * Block '<S297>/Data Type Propagation' : Unused code path elimination
 * Block '<S307>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S309>/Data Type Duplicate' : Unused code path elimination
 * Block '<S309>/Data Type Propagation' : Unused code path elimination
 * Block '<S308>/Data Type Duplicate' : Unused code path elimination
 * Block '<S308>/Data Type Propagation' : Unused code path elimination
 * Block '<S310>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S312>/Data Type Duplicate' : Unused code path elimination
 * Block '<S312>/Data Type Propagation' : Unused code path elimination
 * Block '<S311>/Data Type Duplicate' : Unused code path elimination
 * Block '<S311>/Data Type Propagation' : Unused code path elimination
 * Block '<S313>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S315>/Data Type Duplicate' : Unused code path elimination
 * Block '<S315>/Data Type Propagation' : Unused code path elimination
 * Block '<S314>/Data Type Duplicate' : Unused code path elimination
 * Block '<S314>/Data Type Propagation' : Unused code path elimination
 * Block '<S316>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S318>/Data Type Duplicate' : Unused code path elimination
 * Block '<S318>/Data Type Propagation' : Unused code path elimination
 * Block '<S317>/Data Type Duplicate' : Unused code path elimination
 * Block '<S317>/Data Type Propagation' : Unused code path elimination
 * Block '<S268>/Discrete-Time Integrator' : Unused code path elimination
 * Block '<S332>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S334>/Data Type Duplicate' : Unused code path elimination
 * Block '<S334>/Data Type Propagation' : Unused code path elimination
 * Block '<S333>/Data Type Duplicate' : Unused code path elimination
 * Block '<S333>/Data Type Propagation' : Unused code path elimination
 * Block '<S326>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S335>/Data Type Duplicate' : Unused code path elimination
 * Block '<S335>/Data Type Propagation' : Unused code path elimination
 * Block '<S327>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S336>/Data Type Duplicate' : Unused code path elimination
 * Block '<S336>/Data Type Propagation' : Unused code path elimination
 * Block '<S328>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S337>/Data Type Duplicate' : Unused code path elimination
 * Block '<S337>/Data Type Propagation' : Unused code path elimination
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
 * Block '<S265>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S265>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S267>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S267>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S267>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S267>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S325>/Gain1' : Eliminated nontunable gain of 1
 * Block '<S338>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S341>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S342>/Cast To Single' : Eliminate redundant data type conversion
 * Block '<S7>/Constant14' : Unused code path elimination
 * Block '<S7>/Constant2' : Unused code path elimination
 * Block '<S7>/Constant6' : Unused code path elimination
 * Block '<S7>/Constant8' : Unused code path elimination
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
 * '<S25>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Saturation Dynamic4'
 * '<S26>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Saturation Dynamic5'
 * '<S27>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Saturation Dynamic6'
 * '<S28>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem'
 * '<S29>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL'
 * '<S30>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR'
 * '<S31>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R'
 * '<S32>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant'
 * '<S33>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant1'
 * '<S34>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant2'
 * '<S35>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant3'
 * '<S36>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant4'
 * '<S37>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant5'
 * '<S38>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/MATLAB Function'
 * '<S39>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/MATLAB Function1'
 * '<S40>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic'
 * '<S41>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic1'
 * '<S42>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic2'
 * '<S43>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic3'
 * '<S44>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic4'
 * '<S45>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic5'
 * '<S46>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic6'
 * '<S47>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic'
 * '<S48>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic1'
 * '<S49>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic2'
 * '<S50>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic3'
 * '<S51>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic4'
 * '<S52>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic5'
 * '<S53>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Wtarget'
 * '<S54>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/载荷转移'
 * '<S55>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S56>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S57>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S58>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S59>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic4/Saturation Dynamic'
 * '<S60>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic5/Saturation Dynamic'
 * '<S61>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic6/Saturation Dynamic'
 * '<S62>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S63>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S64>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S65>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Compare To Constant'
 * '<S66>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Compare To Constant2'
 * '<S67>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Detect Rise Positive'
 * '<S68>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Latch_on'
 * '<S69>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/RisingTimer'
 * '<S70>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic'
 * '<S71>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic1'
 * '<S72>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic2'
 * '<S73>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Detect Rise Positive/Positive'
 * '<S74>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Compare To Constant'
 * '<S75>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Compare To Constant2'
 * '<S76>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Detect Rise Positive'
 * '<S77>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Latch_on'
 * '<S78>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/RisingTimer'
 * '<S79>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic'
 * '<S80>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic1'
 * '<S81>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic2'
 * '<S82>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Detect Rise Positive/Positive'
 * '<S83>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant'
 * '<S84>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant1'
 * '<S85>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant2'
 * '<S86>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Zero'
 * '<S87>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Detect Rise Positive'
 * '<S88>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Latch_on'
 * '<S89>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/RisingTimer'
 * '<S90>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic'
 * '<S91>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic1'
 * '<S92>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic2'
 * '<S93>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Detect Rise Positive/Positive'
 * '<S94>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant'
 * '<S95>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant1'
 * '<S96>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant2'
 * '<S97>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant3'
 * '<S98>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant4'
 * '<S99>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Timer1'
 * '<S100>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Timer2'
 * '<S101>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem'
 * '<S102>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/Chart2'
 * '<S103>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/Compare To Constant'
 * '<S104>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/Compare To Constant1'
 * '<S105>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule'
 * '<S106>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule1'
 * '<S107>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule2'
 * '<S108>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule3'
 * '<S109>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule4'
 * '<S110>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule5'
 * '<S111>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive'
 * '<S112>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive'
 * '<S113>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS'
 * '<S114>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/BMS_Recive'
 * '<S115>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE'
 * '<S116>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve'
 * '<S117>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive'
 * '<S118>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key'
 * '<S119>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state'
 * '<S120>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state'
 * '<S121>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule1'
 * '<S122>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule2'
 * '<S123>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule3'
 * '<S124>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule4'
 * '<S125>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU'
 * '<S126>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU'
 * '<S127>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state'
 * '<S128>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1'
 * '<S129>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2'
 * '<S130>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state'
 * '<S131>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule1'
 * '<S132>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule2'
 * '<S133>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule3'
 * '<S134>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule4'
 * '<S135>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule5'
 * '<S136>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule6'
 * '<S137>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule7'
 * '<S138>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule8'
 * '<S139>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule9'
 * '<S140>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state'
 * '<S141>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state/MeaModule1'
 * '<S142>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state'
 * '<S143>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule1'
 * '<S144>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule2'
 * '<S145>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule3'
 * '<S146>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state'
 * '<S147>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1'
 * '<S148>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2'
 * '<S149>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state'
 * '<S150>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule1'
 * '<S151>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule2'
 * '<S152>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule3'
 * '<S153>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule4'
 * '<S154>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule5'
 * '<S155>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule6'
 * '<S156>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule7'
 * '<S157>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule8'
 * '<S158>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state'
 * '<S159>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state/MeaModule1'
 * '<S160>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state'
 * '<S161>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule1'
 * '<S162>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule2'
 * '<S163>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule3'
 * '<S164>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule1'
 * '<S165>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule2'
 * '<S166>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule3'
 * '<S167>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule4'
 * '<S168>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/BMS_Recive/ABS_BUS_state'
 * '<S169>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/BMS_Recive/ABS_BUS_state/IMU_state'
 * '<S170>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr'
 * '<S171>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state'
 * '<S172>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1'
 * '<S173>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule1'
 * '<S174>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule2'
 * '<S175>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule3'
 * '<S176>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule4'
 * '<S177>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule5'
 * '<S178>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state'
 * '<S179>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule'
 * '<S180>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule1'
 * '<S181>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule2'
 * '<S182>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule3'
 * '<S183>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule5'
 * '<S184>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule6'
 * '<S185>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state'
 * '<S186>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state'
 * '<S187>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule2'
 * '<S188>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule3'
 * '<S189>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule4'
 * '<S190>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state'
 * '<S191>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state'
 * '<S192>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule1'
 * '<S193>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule2'
 * '<S194>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule3'
 * '<S195>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/MeaModule'
 * '<S196>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/MeaModule1'
 * '<S197>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/MeaModule2'
 * '<S198>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/Timer'
 * '<S199>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/Timer1'
 * '<S200>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem'
 * '<S201>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem'
 * '<S202>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal'
 * '<S203>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU'
 * '<S204>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS'
 * '<S205>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem'
 * '<S206>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps'
 * '<S207>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant'
 * '<S208>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant1'
 * '<S209>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant10'
 * '<S210>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant11'
 * '<S211>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant2'
 * '<S212>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant3'
 * '<S213>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant4'
 * '<S214>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant5'
 * '<S215>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant6'
 * '<S216>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant7'
 * '<S217>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant8'
 * '<S218>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant9'
 * '<S219>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MATLAB Function'
 * '<S220>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule1'
 * '<S221>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule2'
 * '<S222>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule3'
 * '<S223>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule5'
 * '<S224>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule7'
 * '<S225>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule8'
 * '<S226>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Timer'
 * '<S227>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem'
 * '<S228>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic'
 * '<S229>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1'
 * '<S230>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2'
 * '<S231>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic'
 * '<S232>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic1'
 * '<S233>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic2'
 * '<S234>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S235>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S236>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S237>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Compare To Constant'
 * '<S238>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic'
 * '<S239>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1'
 * '<S240>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Timer'
 * '<S241>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S242>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S243>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem/MeaModule'
 * '<S244>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem/MeaModule1'
 * '<S245>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant'
 * '<S246>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant1'
 * '<S247>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant2'
 * '<S248>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant3'
 * '<S249>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant4'
 * '<S250>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant5'
 * '<S251>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant6'
 * '<S252>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant7'
 * '<S253>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic'
 * '<S254>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1'
 * '<S255>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2'
 * '<S256>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3'
 * '<S257>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer'
 * '<S258>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer1'
 * '<S259>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer2'
 * '<S260>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer3'
 * '<S261>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S262>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S263>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S264>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S265>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment '
 * '<S266>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/MeaModule'
 * '<S267>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment'
 * '<S268>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect'
 * '<S269>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/坐标系转换'
 * '<S270>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment /Compare To Constant'
 * '<S271>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment /Compare To Constant1'
 * '<S272>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer'
 * '<S273>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer1'
 * '<S274>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer2'
 * '<S275>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer3'
 * '<S276>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断'
 * '<S277>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断'
 * '<S278>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断'
 * '<S279>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant'
 * '<S280>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant1'
 * '<S281>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant2'
 * '<S282>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant3'
 * '<S283>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter'
 * '<S284>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1'
 * '<S285>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2'
 * '<S286>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3'
 * '<S287>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic'
 * '<S288>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Saturation Dynamic'
 * '<S289>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S290>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic'
 * '<S291>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Saturation Dynamic'
 * '<S292>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S293>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic'
 * '<S294>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Saturation Dynamic'
 * '<S295>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S296>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic'
 * '<S297>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Saturation Dynamic'
 * '<S298>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S299>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant'
 * '<S300>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant1'
 * '<S301>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant2'
 * '<S302>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant3'
 * '<S303>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter'
 * '<S304>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1'
 * '<S305>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2'
 * '<S306>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3'
 * '<S307>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic'
 * '<S308>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Saturation Dynamic'
 * '<S309>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S310>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic'
 * '<S311>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Saturation Dynamic'
 * '<S312>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S313>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic'
 * '<S314>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Saturation Dynamic'
 * '<S315>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S316>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic'
 * '<S317>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Saturation Dynamic'
 * '<S318>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S319>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant'
 * '<S320>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant1'
 * '<S321>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant2'
 * '<S322>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant3'
 * '<S323>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant'
 * '<S324>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant1'
 * '<S325>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter'
 * '<S326>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic'
 * '<S327>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1'
 * '<S328>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2'
 * '<S329>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Subsystem'
 * '<S330>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Timer1'
 * '<S331>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Timer2'
 * '<S332>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic'
 * '<S333>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Saturation Dynamic'
 * '<S334>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S335>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S336>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S337>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S338>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper'
 * '<S339>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2AMKMCUFL'
 * '<S340>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2AMKMCUFR'
 * '<S341>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU'
 * '<S342>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/WP_OUTPUT'
 * '<S343>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Compare To Constant'
 * '<S344>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/MeaModule'
 * '<S345>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU/MeaModule1'
 * '<S346>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU/MeaModule2'
 * '<S347>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL'
 * '<S348>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ'
 * '<S349>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/EEPROM'
 * '<S350>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling'
 * '<S351>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem'
 * '<S352>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem'
 * '<S353>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem'
 * '<S354>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com0'
 * '<S355>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com1'
 * '<S356>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com2'
 * '<S357>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem'
 * '<S358>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem1'
 * '<S359>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq100ms'
 * '<S360>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq10ms'
 * '<S361>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq500ms'
 * '<S362>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq50ms'
 * '<S363>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq5ms'
 * '<S364>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/EEPROM/EEPROMOperation'
 * '<S365>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling/CCPBackground'
 * '<S366>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling/CCPReceive'
 * '<S367>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling/CCPReceive/Nothing'
 */
#endif                   /* RTW_HEADER_VehCtrlMdel240926_2018b_amkspdlimit_h_ */

/* File trailer for ECUCoder generated file VehCtrlMdel240926_2018b_amkspdlimit.h.
 *
 * [EOF]
 */
