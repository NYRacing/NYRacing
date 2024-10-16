/*
 * Code generated for Simulink model VehCtrlMdel240926_2018b_amkspdlimit.
 *
 * FILE    : VehCtrlMdel240926_2018b_amkspdlimit.h
 *
 * VERSION : 1.265
 *
 * DATE    : Thu Oct 17 04:21:16 2024
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

/* Block states (default storage) for system '<S125>/Timer' */
typedef struct {
  real_T x;                            /* '<S125>/Timer' */
  struct {
    uint_T is_c21_VehCtrlMdel240926_2018b_:2;/* '<S125>/Timer' */
    uint_T is_active_c21_VehCtrlMdel240926:1;/* '<S125>/Timer' */
  } bitsForTID3;
} DW_Timer_VehCtrlMdel240926_20_T;

/* Block signals (default storage) */
typedef struct {
  CAN_DATATYPE CANPack1;               /* '<S346>/CAN Pack1' */
  CAN_DATATYPE CANPack1_b;             /* '<S345>/CAN Pack1' */
  CAN_DATATYPE CANPack1_d;             /* '<S344>/CAN Pack1' */
  CAN_DATATYPE CANUnPackMessage4;      /* '<S185>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_g;    /* '<S179>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_j;    /* '<S156>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_n;    /* '<S167>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_p;    /* '<S165>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_e;    /* '<S137>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_c;    /* '<S149>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_b;    /* '<S147>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_l;    /* '<S198>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_ja;   /* '<S127>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_h;    /* '<S193>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_i;    /* '<S176>/CANUnPackMessage4' */
  real_T Switch1;                      /* '<S345>/Switch1' */
  real_T Switch;                       /* '<S345>/Switch' */
  real_T Switch1_l;                    /* '<S344>/Switch1' */
  real_T MCFL_TorqueLimitP;            /* '<S344>/Switch' */
  real_T Exit;                         /* '<S343>/Timer2' */
  real_T Exit_o;                       /* '<S343>/Timer1' */
  real_T LEDOn;                        /* '<S353>/Chart' */
  real_T Exit_i;                       /* '<S273>/Timer2' */
  real_T Exit_l;                       /* '<S273>/Timer1' */
  real_T Exit_a;                       /* '<S272>/Timer3' */
  real_T Exit_lh;                      /* '<S272>/Timer2' */
  real_T Exit_lh4;                     /* '<S272>/Timer1' */
  real_T Exit_c;                       /* '<S272>/Timer' */
  real_T Exit_h;                       /* '<S212>/Timer3' */
  real_T Exit_o4;                      /* '<S212>/Timer2' */
  real_T Exit_is;                      /* '<S212>/Timer1' */
  real_T Exit_le;                      /* '<S212>/Timer' */
  real_T Exit_on;                      /* '<S210>/Timer' */
  real_T AMKSWITCH_bx;
  real_T ignition_dq;
  real_T low_VOL;                      /* '<S185>/CAN Unpack' */
  real_T MCU_Temp_error;               /* '<S185>/CAN Unpack' */
  real_T Mode;                         /* '<S185>/CAN Unpack' */
  real_T motorTemp_error;              /* '<S185>/CAN Unpack' */
  real_T overCurrent;                  /* '<S185>/CAN Unpack' */
  real_T overpower;                    /* '<S185>/CAN Unpack' */
  real_T overvol;                      /* '<S185>/CAN Unpack' */
  real_T Precharge;                    /* '<S185>/CAN Unpack' */
  real_T Reslove_error;                /* '<S185>/CAN Unpack' */
  real_T MCFR_bDerating;               /* '<S156>/CAN Unpack' */
  real_T MCFR_bQuitDCOn;               /* '<S156>/CAN Unpack' */
  real_T MCFR_bReserve;                /* '<S156>/CAN Unpack' */
  real_T MCFR_bWarn;                   /* '<S156>/CAN Unpack' */
  real_T MCFR_DiagnosticNum;           /* '<S165>/CAN Unpack' */
  real_T MCFL_bDerating;               /* '<S137>/CAN Unpack' */
  real_T MCFL_bReserve;                /* '<S137>/CAN Unpack' */
  real_T MCFL_bWarn;                   /* '<S137>/CAN Unpack' */
  real_T MCFL_DiagnosticNum;           /* '<S147>/CAN Unpack' */
  real_T CANUnpack_o1;                 /* '<S193>/CAN Unpack' */
  real_T CANUnpack_o3;                 /* '<S193>/CAN Unpack' */
  real_T CANUnpack_o5;                 /* '<S193>/CAN Unpack' */
  real_T CANUnpack_o6;                 /* '<S193>/CAN Unpack' */
  real_T Switch11;                     /* '<S108>/Switch11' */
  real_T errorReset;                   /* '<S108>/Chart2' */
  real_T Exit_d;                       /* '<S8>/Timer2' */
  real_T Exit_g;                       /* '<S8>/Timer1' */
  real_T DYC_Enable_OUT;               /* '<S7>/Chart' */
  real_T TCSR_Enable_OUT;              /* '<S7>/Chart' */
  real_T TCSF_Enable_OUT;              /* '<S7>/Chart' */
  uint32_T CANReceive_o3;              /* '<S384>/CANReceive' */
  uint32_T CANReceive_o3_l;            /* '<S369>/CANReceive' */
  uint32_T CANReceive1_o3;             /* '<S122>/CANReceive1' */
  uint32_T CANReceive3_o3;             /* '<S122>/CANReceive3' */
  uint32_T CANReceive3_o3_e;           /* '<S132>/CANReceive3' */
  uint32_T CANReceive1_o3_n;           /* '<S132>/CANReceive1' */
  uint32_T CANReceive2_o3;             /* '<S132>/CANReceive2' */
  uint32_T CANReceive3_o3_i;           /* '<S133>/CANReceive3' */
  uint32_T CANReceive1_o3_h;           /* '<S133>/CANReceive1' */
  uint32_T CANReceive2_o3_j;           /* '<S133>/CANReceive2' */
  uint32_T CANReceive3_o3_c;           /* '<S124>/CANReceive3' */
  uint32_T CANReceive3_o3_m;           /* '<S118>/CANReceive3' */
  uint32_T CANReceive3_o3_cz;          /* '<S123>/CANReceive3' */
  uint32_T CANReceive3_o3_l;           /* '<S121>/CANReceive3' */
  real32_T CastToBoolean4;             /* '<S346>/Cast To Boolean4' */
  real32_T CastToBoolean6;             /* '<S346>/Cast To Boolean6' */
  real32_T VCU_SpdCmd_Emrax;           /* '<S7>/Constant13' */
  int32_T DataTypeConversion2;         /* '<S346>/Data Type Conversion2' */
  uint16_T CastToSingle1;              /* '<S347>/Cast To Single1' */
  uint8_T CANReceive_o2;               /* '<S384>/CANReceive' */
  uint8_T CANReceive_o4[8];            /* '<S384>/CANReceive' */
  uint8_T CANReceive_o5;               /* '<S384>/CANReceive' */
  uint8_T CANReceive_o2_p;             /* '<S369>/CANReceive' */
  uint8_T CANReceive_o4_i[8];          /* '<S369>/CANReceive' */
  uint8_T CANReceive_o5_l;             /* '<S369>/CANReceive' */
  uint8_T CANTransmit;                 /* '<S376>/CANTransmit' */
  uint8_T CANPackMessage[8];           /* '<S346>/CANPackMessage' */
  uint8_T CANTransmit_k;               /* '<S346>/CANTransmit' */
  uint8_T CANPackMessage_f[8];         /* '<S345>/CANPackMessage' */
  uint8_T CANTransmit_l;               /* '<S345>/CANTransmit' */
  uint8_T CANPackMessage_h[8];         /* '<S344>/CANPackMessage' */
  uint8_T CANTransmit_c;               /* '<S344>/CANTransmit' */
  uint8_T CANReceive1_o2;              /* '<S122>/CANReceive1' */
  uint8_T CANReceive1_o4[8];           /* '<S122>/CANReceive1' */
  uint8_T CANReceive1_o5;              /* '<S122>/CANReceive1' */
  uint8_T CANReceive3_o2;              /* '<S122>/CANReceive3' */
  uint8_T CANReceive3_o4[8];           /* '<S122>/CANReceive3' */
  uint8_T CANReceive3_o5;              /* '<S122>/CANReceive3' */
  uint8_T CANReceive3_o2_l;            /* '<S132>/CANReceive3' */
  uint8_T CANReceive3_o4_l[8];         /* '<S132>/CANReceive3' */
  uint8_T CANReceive3_o5_a;            /* '<S132>/CANReceive3' */
  uint8_T CANReceive1_o2_l;            /* '<S132>/CANReceive1' */
  uint8_T CANReceive1_o4_c[8];         /* '<S132>/CANReceive1' */
  uint8_T CANReceive1_o5_a;            /* '<S132>/CANReceive1' */
  uint8_T CANReceive2_o2;              /* '<S132>/CANReceive2' */
  uint8_T CANReceive2_o4[6];           /* '<S132>/CANReceive2' */
  uint8_T CANReceive2_o5;              /* '<S132>/CANReceive2' */
  uint8_T CANReceive3_o2_a;            /* '<S133>/CANReceive3' */
  uint8_T CANReceive3_o4_g[8];         /* '<S133>/CANReceive3' */
  uint8_T CANReceive3_o5_an;           /* '<S133>/CANReceive3' */
  uint8_T CANReceive1_o2_o;            /* '<S133>/CANReceive1' */
  uint8_T CANReceive1_o4_j[8];         /* '<S133>/CANReceive1' */
  uint8_T CANReceive1_o5_j;            /* '<S133>/CANReceive1' */
  uint8_T CANReceive2_o2_p;            /* '<S133>/CANReceive2' */
  uint8_T CANReceive2_o4_k[6];         /* '<S133>/CANReceive2' */
  uint8_T CANReceive2_o5_e;            /* '<S133>/CANReceive2' */
  uint8_T CANReceive3_o2_p;            /* '<S124>/CANReceive3' */
  uint8_T CANReceive3_o4_k[8];         /* '<S124>/CANReceive3' */
  uint8_T CANReceive3_o5_d;            /* '<S124>/CANReceive3' */
  uint8_T CANReceive3_o2_m;            /* '<S118>/CANReceive3' */
  uint8_T CANReceive3_o4_lg[8];        /* '<S118>/CANReceive3' */
  uint8_T CANReceive3_o5_b;            /* '<S118>/CANReceive3' */
  uint8_T CANReceive3_o2_ma;           /* '<S123>/CANReceive3' */
  uint8_T CANReceive3_o4_i[8];         /* '<S123>/CANReceive3' */
  uint8_T CANReceive3_o5_m;            /* '<S123>/CANReceive3' */
  uint8_T CANReceive3_o2_k;            /* '<S121>/CANReceive3' */
  uint8_T CANReceive3_o4_p[8];         /* '<S121>/CANReceive3' */
  uint8_T CANReceive3_o5_de;           /* '<S121>/CANReceive3' */
  boolean_T Drive_ready;               /* '<S125>/SwitchInput' */
  boolean_T SwitchInput1;              /* '<S125>/SwitchInput1' */
  boolean_T out2_c;                    /* '<S125>/SwitchInput3' */
  boolean_T out2_h;
  boolean_T MCFL_DCOn_setpoints_o;     /* '<S108>/Switch4' */
  boolean_T VehReady;                  /* '<S108>/Chart2' */
  boolean_T MCFL_DCEnable;             /* '<S108>/Chart2' */
  boolean_T MCFR_TorqueOn;             /* '<S108>/Chart2' */
  boolean_T MCFL_TorqueOn;             /* '<S108>/Chart2' */
  boolean_T AMKMCFL_InverterOn;        /* '<S108>/Chart2' */
  boolean_T aWaterPumpON;
  boolean_T bWaterPumpON;
} B_VehCtrlMdel240926_2018b_amk_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DelayInput2_DSTATE;           /* '<S258>/Delay Input2' */
  real_T DelayInput2_DSTATE_b;         /* '<S259>/Delay Input2' */
  real_T DelayInput2_DSTATE_h;         /* '<S260>/Delay Input2' */
  real_T DelayInput2_DSTATE_n;         /* '<S261>/Delay Input2' */
  real_T DelayInput2_DSTATE_l;         /* '<S233>/Delay Input2' */
  real_T UnitDelay_DSTATE;             /* '<S232>/Unit Delay' */
  real_T DelayInput2_DSTATE_m;         /* '<S234>/Delay Input2' */
  real_T UnitDelay1_DSTATE;            /* '<S232>/Unit Delay1' */
  real_T DelayInput2_DSTATE_k;         /* '<S235>/Delay Input2' */
  real_T UnitDelay2_DSTATE;            /* '<S232>/Unit Delay2' */
  real_T UnitDelay_DSTATE_n;           /* '<S7>/Unit Delay' */
  real_T DelayInput2_DSTATE_p;         /* '<S43>/Delay Input2' */
  real_T DelayInput2_DSTATE_nk;        /* '<S46>/Delay Input2' */
  real_T UnitDelay_DSTATE_h;           /* '<S10>/Unit Delay' */
  real_T UnitDelay1_DSTATE_c;          /* '<S10>/Unit Delay1' */
  real_T DelayInput2_DSTATE_pt;        /* '<S42>/Delay Input2' */
  real_T DelayInput2_DSTATE_lt;        /* '<S47>/Delay Input2' */
  real_T DelayInput2_DSTATE_hj;        /* '<S41>/Delay Input2' */
  real_T x;                            /* '<S125>/Timer1' */
  real_T b;                            /* '<S7>/Chart' */
  real_T DYC_flag;                     /* '<S7>/Chart' */
  real32_T UnitDelay1_DSTATE_c0;       /* '<S357>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_p;         /* '<S288>/Unit Delay' */
  real32_T DelayInput2_DSTATE_n2;      /* '<S292>/Delay Input2' */
  real32_T UnitDelay_DSTATE_j;         /* '<S281>/Unit Delay' */
  real32_T UnitDelay_DSTATE_pj;        /* '<S289>/Unit Delay' */
  real32_T DelayInput2_DSTATE_e;       /* '<S295>/Delay Input2' */
  real32_T UnitDelay1_DSTATE_n;        /* '<S281>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_a;         /* '<S290>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hk;      /* '<S298>/Delay Input2' */
  real32_T UnitDelay2_DSTATE_l;        /* '<S281>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_nc;        /* '<S291>/Unit Delay' */
  real32_T DelayInput2_DSTATE_c;       /* '<S301>/Delay Input2' */
  real32_T UnitDelay3_DSTATE;          /* '<S281>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_l;         /* '<S308>/Unit Delay' */
  real32_T DelayInput2_DSTATE_i;       /* '<S312>/Delay Input2' */
  real32_T UnitDelay4_DSTATE;          /* '<S282>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_a0;        /* '<S282>/Unit Delay' */
  real32_T UnitDelay_DSTATE_ap;        /* '<S309>/Unit Delay' */
  real32_T DelayInput2_DSTATE_el;      /* '<S315>/Delay Input2' */
  real32_T UnitDelay5_DSTATE;          /* '<S282>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_a;        /* '<S282>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_o;         /* '<S310>/Unit Delay' */
  real32_T DelayInput2_DSTATE_pd;      /* '<S318>/Delay Input2' */
  real32_T UnitDelay6_DSTATE;          /* '<S282>/Unit Delay6' */
  real32_T UnitDelay2_DSTATE_c;        /* '<S282>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_ah;        /* '<S311>/Unit Delay' */
  real32_T DelayInput2_DSTATE_mt;      /* '<S321>/Delay Input2' */
  real32_T UnitDelay7_DSTATE;          /* '<S282>/Unit Delay7' */
  real32_T UnitDelay3_DSTATE_d;        /* '<S282>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_lh;        /* '<S207>/Unit Delay' */
  real32_T UnitDelay4_DSTATE_m;        /* '<S270>/Unit Delay4' */
  real32_T UnitDelay1_DSTATE_k;        /* '<S207>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_o;        /* '<S270>/Unit Delay1' */
  real32_T UnitDelay3_DSTATE_p;        /* '<S273>/Unit Delay3' */
  real32_T UnitDelay1_DSTATE_d;        /* '<S273>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_f;        /* '<S273>/Unit Delay2' */
  real32_T UnitDelay4_DSTATE_mn;       /* '<S273>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_d;         /* '<S270>/Unit Delay' */
  real32_T UnitDelay2_DSTATE_i;        /* '<S270>/Unit Delay2' */
  real32_T DelayInput2_DSTATE_g;       /* '<S333>/Delay Input2' */
  real32_T DelayInput2_DSTATE_a;       /* '<S331>/Delay Input2' */
  real32_T DelayInput2_DSTATE_f;       /* '<S337>/Delay Input2' */
  real32_T UnitDelay_DSTATE_ncs;       /* '<S330>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hu;      /* '<S332>/Delay Input2' */
  real32_T DelayInput2_DSTATE_l2;      /* '<S243>/Delay Input2' */
  real32_T UnitDelay_DSTATE_o2;        /* '<S210>/Unit Delay' */
  real32_T DelayInput2_DSTATE_j;       /* '<S244>/Delay Input2' */
  real32_T UnitDelay1_DSTATE_aq;       /* '<S210>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_l4;      /* '<S44>/Delay Input2' */
  real32_T DelayInput2_DSTATE_j3;      /* '<S45>/Delay Input2' */
  real32_T UnitDelay4_DSTATE_j;        /* '<S10>/Unit Delay4' */
  real32_T UnitDelay2_DSTATE_j;        /* '<S10>/Unit Delay2' */
  real32_T UnitDelay5_DSTATE_k;        /* '<S10>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_g;        /* '<S32>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_i;        /* '<S93>/Unit Delay1' */
  real32_T UnitDelay4_DSTATE_b;        /* '<S32>/Unit Delay4' */
  real32_T UnitDelay5_DSTATE_l;        /* '<S32>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_b;         /* '<S32>/Unit Delay' */
  real32_T UnitDelay2_DSTATE_g;        /* '<S32>/Unit Delay2' */
  real32_T UnitDelay1_DSTATE_n5;       /* '<S82>/Unit Delay1' */
  real32_T UnitDelay4_DSTATE_i;        /* '<S31>/Unit Delay4' */
  real32_T UnitDelay1_DSTATE_h;        /* '<S73>/Unit Delay1' */
  real32_T UnitDelay4_DSTATE_l;        /* '<S30>/Unit Delay4' */
  real32_T DelayInput2_DSTATE_cd;      /* '<S19>/Delay Input2' */
  real32_T UnitDelay5_DSTATE_i;        /* '<S31>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_f;         /* '<S31>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_f;        /* '<S31>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_jr;       /* '<S31>/Unit Delay2' */
  real32_T DelayInput2_DSTATE_hn;      /* '<S20>/Delay Input2' */
  real32_T UnitDelay1_DSTATE_g4;       /* '<S30>/Unit Delay1' */
  real32_T UnitDelay5_DSTATE_ip;       /* '<S30>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_nr;        /* '<S30>/Unit Delay' */
  real32_T UnitDelay2_DSTATE_b;        /* '<S30>/Unit Delay2' */
  real32_T DelayInput2_DSTATE_ib;      /* '<S21>/Delay Input2' */
  int32_T sfEvent;                     /* '<S108>/Chart2' */
  uint32_T previousTicks;              /* '<S353>/Chart' */
  uint32_T previousTicks_m;            /* '<S343>/Chart' */
  uint32_T Subsystem_PREV_T;           /* '<S4>/Subsystem' */
  uint32_T FunctionCallSubsystem_PREV_T;/* '<S4>/Function-Call Subsystem' */
  uint32_T previousTicks_g;            /* '<S108>/Chart2' */
  uint32_T MoTrqReq_PREV_T;            /* '<S1>/MoTrqReq' */
  int_T CANPack1_ModeSignalID;         /* '<S346>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_g;       /* '<S345>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_f;       /* '<S344>/CAN Pack1' */
  int_T CANUnpack_ModeSignalID;        /* '<S185>/CAN Unpack' */
  int_T CANUnpack_StatusPortID;        /* '<S185>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_b;      /* '<S179>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_m;      /* '<S179>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_a;      /* '<S156>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_o;      /* '<S156>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_n;      /* '<S167>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_f;      /* '<S167>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_p;      /* '<S165>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_e;      /* '<S165>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_c;      /* '<S137>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_ok;     /* '<S137>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_m;      /* '<S149>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_j;      /* '<S149>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_i;      /* '<S147>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_mj;     /* '<S147>/CAN Unpack' */
  int_T CANUnpack1_ModeSignalID;       /* '<S198>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID;       /* '<S198>/CAN Unpack1' */
  int_T CANUnpack1_ModeSignalID_m;     /* '<S127>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID_n;     /* '<S127>/CAN Unpack1' */
  int_T CANUnpack_ModeSignalID_f;      /* '<S193>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_k;      /* '<S193>/CAN Unpack' */
  struct {
    uint_T is_VehStat:4;               /* '<S108>/Chart2' */
    uint_T is_AMKDCready:4;            /* '<S108>/Chart2' */
    uint_T is_c27_VehCtrlMdel240926_2018b_:3;/* '<S353>/Chart' */
    uint_T is_StateA:2;                /* '<S353>/Chart' */
    uint_T is_StateB:2;                /* '<S353>/Chart' */
    uint_T is_StateC:2;                /* '<S353>/Chart' */
    uint_T is_c24_VehCtrlMdel240926_2018b_:2;/* '<S343>/Chart' */
    uint_T is_STATEON:2;               /* '<S343>/Chart' */
    uint_T is_STATEOFF:2;              /* '<S343>/Chart' */
    uint_T is_c23_VehCtrlMdel240926_2018b_:2;/* '<S125>/Timer1' */
    uint_T is_BeeperStat:2;            /* '<S108>/Chart2' */
    uint_T is_AMKDCon:2;               /* '<S108>/Chart2' */
    uint_T is_MCDCEnable:2;            /* '<S108>/Chart2' */
    uint_T is_MC_TorqueCUT:2;          /* '<S108>/Chart2' */
    uint_T is_AMKCANenable:2;          /* '<S108>/Chart2' */
    uint_T is_MCFL_InverterOn:2;       /* '<S108>/Chart2' */
    uint_T is_MCFR_InverterOn:2;       /* '<S108>/Chart2' */
    uint_T is_B:2;                     /* '<S7>/Chart' */
    uint_T is_C:2;                     /* '<S7>/Chart' */
    uint_T is_D:2;                     /* '<S7>/Chart' */
    uint_T is_E:2;                     /* '<S7>/Chart' */
    uint_T is_active_c27_VehCtrlMdel240926:1;/* '<S353>/Chart' */
    uint_T is_active_c24_VehCtrlMdel240926:1;/* '<S343>/Chart' */
    uint_T is_active_c23_VehCtrlMdel240926:1;/* '<S125>/Timer1' */
    uint_T is_active_c1_VehCtrlMdel240926_:1;/* '<S108>/Chart2' */
    uint_T is_active_VehStat:1;        /* '<S108>/Chart2' */
    uint_T is_active_BeeperStat:1;     /* '<S108>/Chart2' */
    uint_T is_active_AMKDCon:1;        /* '<S108>/Chart2' */
    uint_T is_active_MCDCEnable:1;     /* '<S108>/Chart2' */
    uint_T is_active_MC_TorqueCUT:1;   /* '<S108>/Chart2' */
    uint_T is_active_AMKDCready:1;     /* '<S108>/Chart2' */
    uint_T is_active_Output:1;         /* '<S108>/Chart2' */
    uint_T is_active_AMKCANenable:1;   /* '<S108>/Chart2' */
    uint_T is_active_MCFL_InverterOn:1;/* '<S108>/Chart2' */
    uint_T is_active_MCFR_InverterOn:1;/* '<S108>/Chart2' */
    uint_T is_active_c7_VehCtrlMdel240926_:1;/* '<S7>/Chart' */
  } bitsForTID3;

  uint16_T UnitDelay1_DSTATE_fm;       /* '<S208>/Unit Delay1' */
  uint16_T UnitDelay_DSTATE_k;         /* '<S208>/Unit Delay' */
  uint16_T temporalCounter_i2;         /* '<S108>/Chart2' */
  boolean_T DelayInput1_DSTATE;        /* '<S352>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_n;      /* '<S351>/Delay Input1' */
  boolean_T UnitDelay_DSTATE_l3;       /* '<S343>/Unit Delay' */
  boolean_T UnitDelay3_DSTATE_f;       /* '<S270>/Unit Delay3' */
  boolean_T UnitDelay3_DSTATE_i;       /* '<S10>/Unit Delay3' */
  boolean_T UnitDelay6_DSTATE_b;       /* '<S10>/Unit Delay6' */
  boolean_T UnitDelay1_DSTATE_e;       /* '<S92>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_a;       /* '<S32>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE_e;      /* '<S91>/Delay Input1' */
  boolean_T UnitDelay1_DSTATE_gl;      /* '<S81>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_e;       /* '<S31>/Unit Delay3' */
  boolean_T UnitDelay1_DSTATE_dp;      /* '<S72>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_ip;      /* '<S30>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE_j;      /* '<S80>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_b;      /* '<S71>/Delay Input1' */
  uint8_T temporalCounter_i1;          /* '<S353>/Chart' */
  uint8_T temporalCounter_i1_p;        /* '<S343>/Chart' */
  uint8_T temporalCounter_i1_f;        /* '<S108>/Chart2' */
  boolean_T Subsystem_RESET_ELAPS_T;   /* '<S4>/Subsystem' */
  boolean_T FunctionCallSubsystem_RESET_ELA;/* '<S4>/Function-Call Subsystem' */
  boolean_T MoTrqReq_RESET_ELAPS_T;    /* '<S1>/MoTrqReq' */
  boolean_T EnabledSubsystem1_MODE;    /* '<S343>/Enabled Subsystem1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2_h;/* '<S343>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1_h;/* '<S343>/Timer1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2_j;/* '<S273>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1_p;/* '<S273>/Timer1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer3_i;/* '<S272>/Timer3' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2_g;/* '<S272>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1_m;/* '<S272>/Timer1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer_o;/* '<S272>/Timer' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer3;/* '<S212>/Timer3' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2_l;/* '<S212>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1_n;/* '<S212>/Timer1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer_b;/* '<S212>/Timer' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer_k;/* '<S210>/Timer' */
  DW_Timer_VehCtrlMdel240926_20_T sf_Timer_a;/* '<S208>/Timer' */
  DW_Timer_VehCtrlMdel240926_20_T sf_Timer;/* '<S125>/Timer' */
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
  real_T pooled5[20];

  /* Pooled Parameter (Expression: [0;1000;2000;3000;4000;5000;6000;7000;8000;9000;10000;11000;12000;13000;14000;15000;16000;17000;18000;19000])
   * Referenced by:
   *   '<S10>/AMK'
   *   '<S10>/AMK1'
   */
  real_T pooled6[20];

  /* Expression: [0;0;20;40;70;100;100]
   * Referenced by: '<S8>/2-D Lookup Table2'
   */
  real_T uDLookupTable2_tableData[7];

  /* Expression: [0;25;30;35;37;40;50]
   * Referenced by: '<S8>/2-D Lookup Table2'
   */
  real_T uDLookupTable2_bp01Data[7];

  /* Expression: single([1000,0]);
   * Referenced by: '<S7>/BrakeCompensateCoefRear'
   */
  real32_T BrakeCompensateCoefRear_tableDa[2];

  /* Expression: single([550,1500]);
   * Referenced by: '<S7>/BrakeCompensateCoefRear'
   */
  real32_T BrakeCompensateCoefRear_bp01Dat[2];

  /* Expression: single(reshape([0,20,35,50,79,100,120,125,130,130,170,0,20,35,50,79,100,120,125,160,160,170,0,20,35,50,79,100,120,125,160,160,180,0,20,35,50,79,105,120,130,160,160,180,0,20,35,50,85,105,125,135,160,160,180,0,20,35,50,85,105,125,140,170,170,210,0,20,30,50,85,110,125,145,160,160,200,0,18,30,50,90,110,135,155,160,160,200,0,18,25,50,95,110,135,155,160,160,200,0,18,25,40,100,110,135,155,160,160,200,0,17,25,40,110,115,135,155,160,160,200,0,17,25,40,110,115,135,155,160,160,200,0,16,20,40,110,125,135,155,160,160,190,0,16,20,40,100,125,130,155,160,160,180,0,15,20,40,90,120,130,155,160,160,180,0,15,20,40,90,110,120,135,142,142,152,0,15,20,40,80,90,100,110,120,120,133,0,14,20,40,72,80,90,100,105,105,118,0,14,20,38,70,76,82,88,94,95,106,0,14,18,35,65,70,74,82,88,91,100,0,14,18,35,65,70,74,82,88,91,100],11,21));
   * Referenced by: '<S29>/2-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData[231];

  /* Expression: single([0,10,20,30,40,50,60,70,80,90,100]);
   * Referenced by: '<S29>/2-D Lookup Table1'
   */
  real32_T uDLookupTable1_bp01Data[11];

  /* Expression: single([5.400000095367432,10.800000190734863,16.200000762939453,21.600000381469727,27,32.400001525878906,37.79999923706055,43.20000076293945,48.599998474121094,54,59.400001525878906,64.80000305175781,70.19999694824219,75.5999984741211,81,86.4000015258789,91.80000305175781,97.19999694824219,102.5999984741211,108,114]);
   * Referenced by: '<S29>/2-D Lookup Table1'
   */
  real32_T uDLookupTable1_bp02Data[21];

  /* Computed Parameter: uDLookupTable1_tableData_i
   * Referenced by: '<S10>/2-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData_i[35];

  /* Computed Parameter: uDLookupTable1_bp01Data_n
   * Referenced by: '<S10>/2-D Lookup Table1'
   */
  real32_T uDLookupTable1_bp01Data_n[5];

  /* Computed Parameter: uDLookupTable1_bp02Data_h
   * Referenced by: '<S10>/2-D Lookup Table1'
   */
  real32_T uDLookupTable1_bp02Data_h[7];

  /* Pooled Parameter (Expression: [0.3,0.3,2.5,2.5])
   * Referenced by:
   *   '<S30>/VehSpd_SlipTarget_mps'
   *   '<S31>/VehSpd_SlipTarget_mps'
   *   '<S32>/VehSpd_SlipTarget_mps'
   */
  real32_T pooled51[4];

  /* Pooled Parameter (Expression: [0,3,25,30])
   * Referenced by:
   *   '<S30>/VehSpd_SlipTarget_mps'
   *   '<S30>/VehicleStableTarget_mps'
   *   '<S30>/VehicleStableTarget_mps1'
   *   '<S31>/VehSpd_SlipTarget_mps'
   *   '<S31>/VehicleStableTarget_mps'
   *   '<S31>/VehicleStableTarget_mps1'
   *   '<S32>/VehSpd_SlipTarget_mps'
   *   '<S32>/VehicleStableTarget_mps'
   *   '<S32>/VehicleStableTarget_mps1'
   */
  real32_T pooled52[4];

  /* Pooled Parameter (Expression: [0.4,0.4,1.2,1.2])
   * Referenced by:
   *   '<S30>/VehicleStableTarget_mps'
   *   '<S30>/VehicleStableTarget_mps1'
   *   '<S31>/VehicleStableTarget_mps'
   *   '<S31>/VehicleStableTarget_mps1'
   *   '<S32>/VehicleStableTarget_mps'
   *   '<S32>/VehicleStableTarget_mps1'
   */
  real32_T pooled57[4];

  /* Expression: single([20,0]);
   * Referenced by: '<S7>/BrakeCompensateCoefFront1'
   */
  real32_T BrakeCompensateCoefFront1_table[2];

  /* Expression: single([550,1700]);
   * Referenced by: '<S7>/BrakeCompensateCoefFront1'
   */
  real32_T BrakeCompensateCoefFront1_bp01D[2];

  /* Pooled Parameter (Expression: single([500,4500]);)
   * Referenced by: '<S208>/1-D Lookup Table1'
   */
  real32_T pooled63[2];

  /* Pooled Parameter (Expression: single([0,100]);)
   * Referenced by:
   *   '<S208>/1-D Lookup Table3'
   *   '<S208>/1-D Lookup Table4'
   */
  real32_T pooled64[2];

  /* Expression: single([2589,2754])
   * Referenced by: '<S208>/1-D Lookup Table4'
   */
  real32_T uDLookupTable4_bp01Data[2];

  /* Expression: single([2471,2642])
   * Referenced by: '<S208>/1-D Lookup Table3'
   */
  real32_T uDLookupTable3_bp01Data[2];

  /* Expression: single([-26.072928
     -23.62248
     -20.834568
     -18.002448
     -15.197904
     -12.620232
     -10.131048
     -8.139672
     -6.2976672
     -4.605012
     -3.0893688
     -1.4907528
     -0.07467624
     1.40778
     2.9621448
     4.7211768
     6.5521152
     8.737056
     10.944144
     13.743144
     17.25012
     20.375424
     24.070536
     27.505584
     ]);
   * Referenced by: '<S210>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData[24];

  /* Expression: single([-119.993
     -109.997
     -100
     -90.0034
     -80.0067
     -69.9933
     -59.9966
     -50
     -40.0034
     -30.0067
     -19.9933
     -9.99664
     0
     9.99664
     19.9933
     30.0067
     40.0034
     50
     59.9966
     69.9933
     80.0067
     90.0034
     100
     109.997
     ]);
   * Referenced by: '<S210>/1-D Lookup Table'
   */
  real32_T uDLookupTable_bp01Data[24];

  /* Expression: single([-26.816256
     -23.524848
     -19.851048
     -16.282584
     -13.40676
     -10.314792
     -7.65504
     -5.3554968
     -3.4548912
     -1.5044184
     0.02493504
     1.4379192
     2.8675296
     4.2638976
     5.76
     7.35588
     9.101304
     11.04624
     12.94128
     15.35724
     18.244152
     20.937168
     23.630112
     26.306496
     ]);
   * Referenced by: '<S210>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData_b[24];

  /* Expression: single([-119.995
     -109.993
     -99.9916
     -90.0067
     -80.005
     -70.0034
     -60.0017
     -50
     -39.9983
     -29.9966
     -19.995
     -9.99328
     0.00840477
     9.99328
     19.995
     29.9966
     39.9983
     50
     60.0017
     70.0034
     80.005
     90.0067
     100.008
     109.993
     ]);
   * Referenced by: '<S210>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_bp01Data_h[24];

  /* Computed Parameter: uDLookupTable1_maxIndex
   * Referenced by: '<S29>/2-D Lookup Table1'
   */
  uint32_T uDLookupTable1_maxIndex[2];

  /* Computed Parameter: uDLookupTable1_maxIndex_h
   * Referenced by: '<S10>/2-D Lookup Table1'
   */
  uint32_T uDLookupTable1_maxIndex_h[2];
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
extern real_T Gear_Trs;                /* '<S346>/Switch2' */
extern real_T Mode_Trs;                /* '<S346>/Switch3' */
extern real_T AMKFL_Current;           /* '<S211>/Switch' */
extern real_T AMKFR_Current;           /* '<S211>/Switch1' */
extern real_T Trq_CUT;                 /* '<S208>/Timer' */
extern real_T AMKSWITCH;               /* '<S125>/Timer1' */
extern real_T ignition;                /* '<S125>/Timer' */
extern real_T L12V_error;              /* '<S185>/CAN Unpack' */
extern real_T alarm;                   /* '<S185>/CAN Unpack' */
extern real_T controller_ready;        /* '<S185>/CAN Unpack' */
extern real_T selfcheck;               /* '<S185>/CAN Unpack' */
extern real_T RPM;                     /* '<S185>/CAN Unpack' */
extern real_T trq;                     /* '<S185>/CAN Unpack' */
extern real_T AC_current;              /* '<S179>/CAN Unpack' */
extern real_T DC_current;              /* '<S179>/CAN Unpack' */
extern real_T MCU_Temp;                /* '<S179>/CAN Unpack' */
extern real_T motor_Temp;              /* '<S179>/CAN Unpack' */
extern real_T voltage;                 /* '<S179>/CAN Unpack' */
extern real_T MCFR_ActualTorque;       /* '<S156>/CAN Unpack' */
extern real_T MCFR_ActualVelocity;     /* '<S156>/CAN Unpack' */
extern real_T MCFR_DCVoltage;          /* '<S156>/CAN Unpack' */
extern real_T MCFR_bDCOn;              /* '<S156>/CAN Unpack' */
extern real_T MCFR_bError;             /* '<S156>/CAN Unpack' */
extern real_T MCFR_bInverterOn;        /* '<S156>/CAN Unpack' */
extern real_T MCFR_bQuitInverterOn;    /* '<S156>/CAN Unpack' */
extern real_T MCFR_bSystemReady;       /* '<S156>/CAN Unpack' */
extern real_T MCFR_TempIGBT;           /* '<S167>/CAN Unpack' */
extern real_T MCFR_TempInverter;       /* '<S167>/CAN Unpack' */
extern real_T MCFR_TempMotor;          /* '<S167>/CAN Unpack' */
extern real_T MCFR_ErrorInfo;          /* '<S165>/CAN Unpack' */
extern real_T MCFL_ActualTorque;       /* '<S137>/CAN Unpack' */
extern real_T MCFL_ActualVelocity;     /* '<S137>/CAN Unpack' */
extern real_T MCFL_DCVoltage;          /* '<S137>/CAN Unpack' */
extern real_T MCFL_bDCOn;              /* '<S137>/CAN Unpack' */
extern real_T MCFL_bError;             /* '<S137>/CAN Unpack' */
extern real_T MCFL_bInverterOn;        /* '<S137>/CAN Unpack' */
extern real_T MCFL_bQuitDCOn;          /* '<S137>/CAN Unpack' */
extern real_T MCFL_bQuitInverterOn;    /* '<S137>/CAN Unpack' */
extern real_T MCFL_bSystemReady;       /* '<S137>/CAN Unpack' */
extern real_T MCFL_TempIGBT;           /* '<S149>/CAN Unpack' */
extern real_T MCFL_TempInverter;       /* '<S149>/CAN Unpack' */
extern real_T MCFL_TempMotor;          /* '<S149>/CAN Unpack' */
extern real_T MCFL_ErrorInfo;          /* '<S147>/CAN Unpack' */
extern real_T StrWhlAngAliveRollCnt;   /* '<S198>/CAN Unpack1' */
extern real_T StrWhlAng;               /* '<S198>/CAN Unpack1' */
extern real_T StrWhlAngV;              /* '<S198>/CAN Unpack1' */
extern real_T ABS_WS_FL;               /* '<S127>/CAN Unpack1' */
extern real_T ABS_WS_FR;               /* '<S127>/CAN Unpack1' */
extern real_T ABS_WS_RL;               /* '<S127>/CAN Unpack1' */
extern real_T ABS_WS_RR;               /* '<S127>/CAN Unpack1' */
extern real_T IMU_Ay_Value;            /* '<S193>/CAN Unpack' */
extern real_T IMU_Ax_Value;            /* '<S193>/CAN Unpack' */
extern real_T IMU_Yaw_Value;           /* '<S193>/CAN Unpack' */
extern real_T EMRAX_Trq_CUT;           /*  */
extern real_T AMK_Trq_CUT;             /*  */
extern uint32_T Acc_vol2;              /* '<S208>/Add3' */
extern uint32_T Acc_vol;               /* '<S208>/Add2' */
extern uint32_T Acc_POS;               /* '<S208>/1-D Lookup Table4' */
extern uint32_T Acc_POS2;              /* '<S208>/1-D Lookup Table3' */
extern real32_T VehVxEst_mps;          /* '<S330>/Add' */
extern real32_T EmraxTrqR_cmd;         /* '<S7>/Saturation1' */
extern real32_T AMKTrqFR_cmd;          /* '<S7>/Saturation2' */
extern real32_T AMKTrqFL_cmd;          /* '<S7>/Saturation3' */
extern uint16_T F_BrkPrs;              /* '<S208>/1-D Lookup Table1' */
extern uint16_T Acc1;                  /* '<S120>/Acc3' */
extern uint16_T Acc2;                  /* '<S120>/Acc4' */
extern uint16_T Brk1;                  /* '<S120>/Brk1' */
extern uint16_T Brk2;                  /* '<S120>/Brk2' */
extern boolean_T STATEDISPLAY;         /* '<S343>/Switch1' */
extern boolean_T HVSWITCH;             /* '<S343>/Chart' */
extern boolean_T KeyPressed;           /* '<S108>/Cast To Boolean' */
extern boolean_T Brk;                  /* '<S110>/Compare' */
extern boolean_T ACC_Release;          /* '<S111>/Compare' */
extern boolean_T beeper_state;         /* '<S108>/Chart2' */
extern boolean_T MCFL_DCOn_setpoints;  /* '<S108>/Chart2' */
extern boolean_T MCFR_DCEnable;        /* '<S108>/Chart2' */
extern boolean_T MCFR_InverterOn;      /* '<S108>/Chart2' */
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
 * Block '<S10>/Abs4' : Unused code path elimination
 * Block '<S41>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S58>/Data Type Duplicate' : Unused code path elimination
 * Block '<S58>/Data Type Propagation' : Unused code path elimination
 * Block '<S42>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S59>/Data Type Duplicate' : Unused code path elimination
 * Block '<S59>/Data Type Propagation' : Unused code path elimination
 * Block '<S43>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Propagation' : Unused code path elimination
 * Block '<S44>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S61>/Data Type Duplicate' : Unused code path elimination
 * Block '<S61>/Data Type Propagation' : Unused code path elimination
 * Block '<S45>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S62>/Data Type Duplicate' : Unused code path elimination
 * Block '<S62>/Data Type Propagation' : Unused code path elimination
 * Block '<S46>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S63>/Data Type Duplicate' : Unused code path elimination
 * Block '<S63>/Data Type Propagation' : Unused code path elimination
 * Block '<S47>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S64>/Data Type Duplicate' : Unused code path elimination
 * Block '<S64>/Data Type Propagation' : Unused code path elimination
 * Block '<S48>/Data Type Duplicate' : Unused code path elimination
 * Block '<S48>/Data Type Propagation' : Unused code path elimination
 * Block '<S49>/Data Type Duplicate' : Unused code path elimination
 * Block '<S49>/Data Type Propagation' : Unused code path elimination
 * Block '<S50>/Data Type Duplicate' : Unused code path elimination
 * Block '<S50>/Data Type Propagation' : Unused code path elimination
 * Block '<S51>/Data Type Duplicate' : Unused code path elimination
 * Block '<S51>/Data Type Propagation' : Unused code path elimination
 * Block '<S52>/Data Type Duplicate' : Unused code path elimination
 * Block '<S52>/Data Type Propagation' : Unused code path elimination
 * Block '<S53>/Data Type Duplicate' : Unused code path elimination
 * Block '<S53>/Data Type Propagation' : Unused code path elimination
 * Block '<S54>/Data Type Duplicate' : Unused code path elimination
 * Block '<S54>/Data Type Propagation' : Unused code path elimination
 * Block '<S55>/Data Type Duplicate' : Unused code path elimination
 * Block '<S55>/Data Type Propagation' : Unused code path elimination
 * Block '<S19>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S65>/Data Type Duplicate' : Unused code path elimination
 * Block '<S65>/Data Type Propagation' : Unused code path elimination
 * Block '<S20>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S66>/Data Type Duplicate' : Unused code path elimination
 * Block '<S66>/Data Type Propagation' : Unused code path elimination
 * Block '<S21>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S67>/Data Type Duplicate' : Unused code path elimination
 * Block '<S67>/Data Type Propagation' : Unused code path elimination
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
 * Block '<S74>/Data Type Duplicate' : Unused code path elimination
 * Block '<S74>/Data Type Propagation' : Unused code path elimination
 * Block '<S75>/Data Type Duplicate' : Unused code path elimination
 * Block '<S75>/Data Type Propagation' : Unused code path elimination
 * Block '<S76>/Data Type Duplicate' : Unused code path elimination
 * Block '<S76>/Data Type Propagation' : Unused code path elimination
 * Block '<S83>/Data Type Duplicate' : Unused code path elimination
 * Block '<S83>/Data Type Propagation' : Unused code path elimination
 * Block '<S84>/Data Type Duplicate' : Unused code path elimination
 * Block '<S84>/Data Type Propagation' : Unused code path elimination
 * Block '<S85>/Data Type Duplicate' : Unused code path elimination
 * Block '<S85>/Data Type Propagation' : Unused code path elimination
 * Block '<S94>/Data Type Duplicate' : Unused code path elimination
 * Block '<S94>/Data Type Propagation' : Unused code path elimination
 * Block '<S95>/Data Type Duplicate' : Unused code path elimination
 * Block '<S95>/Data Type Propagation' : Unused code path elimination
 * Block '<S96>/Data Type Duplicate' : Unused code path elimination
 * Block '<S96>/Data Type Propagation' : Unused code path elimination
 * Block '<S8>/2-D Lookup Table1' : Unused code path elimination
 * Block '<S8>/2-D Lookup Table3' : Unused code path elimination
 * Block '<S8>/2-D Lookup Table4' : Unused code path elimination
 * Block '<S8>/AND' : Unused code path elimination
 * Block '<S8>/AND1' : Unused code path elimination
 * Block '<S8>/AND3' : Unused code path elimination
 * Block '<S98>/Compare' : Unused code path elimination
 * Block '<S98>/Constant' : Unused code path elimination
 * Block '<S100>/Compare' : Unused code path elimination
 * Block '<S100>/Constant' : Unused code path elimination
 * Block '<S104>/Compare' : Unused code path elimination
 * Block '<S104>/Constant' : Unused code path elimination
 * Block '<S8>/Constant' : Unused code path elimination
 * Block '<S8>/Constant1' : Unused code path elimination
 * Block '<S8>/Constant4' : Unused code path elimination
 * Block '<S8>/Max3' : Unused code path elimination
 * Block '<S8>/NOT' : Unused code path elimination
 * Block '<S8>/NOT1' : Unused code path elimination
 * Block '<S8>/NOT2' : Unused code path elimination
 * Block '<S8>/Switch' : Unused code path elimination
 * Block '<S8>/Switch1' : Unused code path elimination
 * Block '<S8>/Switch2' : Unused code path elimination
 * Block '<S108>/Switch5' : Unused code path elimination
 * Block '<S176>/CAN Unpack1' : Unused code path elimination
 * Block '<S208>/1-D Lookup Table2' : Unused code path elimination
 * Block '<S208>/Abs1' : Unused code path elimination
 * Block '<S208>/Add4' : Unused code path elimination
 * Block '<S224>/Compare' : Unused code path elimination
 * Block '<S224>/Constant' : Unused code path elimination
 * Block '<S233>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S239>/Data Type Duplicate' : Unused code path elimination
 * Block '<S239>/Data Type Propagation' : Unused code path elimination
 * Block '<S234>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S240>/Data Type Duplicate' : Unused code path elimination
 * Block '<S240>/Data Type Propagation' : Unused code path elimination
 * Block '<S235>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S241>/Data Type Duplicate' : Unused code path elimination
 * Block '<S241>/Data Type Propagation' : Unused code path elimination
 * Block '<S236>/Data Type Duplicate' : Unused code path elimination
 * Block '<S236>/Data Type Propagation' : Unused code path elimination
 * Block '<S237>/Data Type Duplicate' : Unused code path elimination
 * Block '<S237>/Data Type Propagation' : Unused code path elimination
 * Block '<S238>/Data Type Duplicate' : Unused code path elimination
 * Block '<S238>/Data Type Propagation' : Unused code path elimination
 * Block '<S243>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S246>/Data Type Duplicate' : Unused code path elimination
 * Block '<S246>/Data Type Propagation' : Unused code path elimination
 * Block '<S244>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S247>/Data Type Duplicate' : Unused code path elimination
 * Block '<S247>/Data Type Propagation' : Unused code path elimination
 * Block '<S258>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S266>/Data Type Duplicate' : Unused code path elimination
 * Block '<S266>/Data Type Propagation' : Unused code path elimination
 * Block '<S259>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S267>/Data Type Duplicate' : Unused code path elimination
 * Block '<S267>/Data Type Propagation' : Unused code path elimination
 * Block '<S260>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S268>/Data Type Duplicate' : Unused code path elimination
 * Block '<S268>/Data Type Propagation' : Unused code path elimination
 * Block '<S261>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S269>/Data Type Duplicate' : Unused code path elimination
 * Block '<S269>/Data Type Propagation' : Unused code path elimination
 * Block '<S292>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S294>/Data Type Duplicate' : Unused code path elimination
 * Block '<S294>/Data Type Propagation' : Unused code path elimination
 * Block '<S293>/Data Type Duplicate' : Unused code path elimination
 * Block '<S293>/Data Type Propagation' : Unused code path elimination
 * Block '<S295>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S297>/Data Type Duplicate' : Unused code path elimination
 * Block '<S297>/Data Type Propagation' : Unused code path elimination
 * Block '<S296>/Data Type Duplicate' : Unused code path elimination
 * Block '<S296>/Data Type Propagation' : Unused code path elimination
 * Block '<S298>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S300>/Data Type Duplicate' : Unused code path elimination
 * Block '<S300>/Data Type Propagation' : Unused code path elimination
 * Block '<S299>/Data Type Duplicate' : Unused code path elimination
 * Block '<S299>/Data Type Propagation' : Unused code path elimination
 * Block '<S301>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S303>/Data Type Duplicate' : Unused code path elimination
 * Block '<S303>/Data Type Propagation' : Unused code path elimination
 * Block '<S302>/Data Type Duplicate' : Unused code path elimination
 * Block '<S302>/Data Type Propagation' : Unused code path elimination
 * Block '<S312>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S314>/Data Type Duplicate' : Unused code path elimination
 * Block '<S314>/Data Type Propagation' : Unused code path elimination
 * Block '<S313>/Data Type Duplicate' : Unused code path elimination
 * Block '<S313>/Data Type Propagation' : Unused code path elimination
 * Block '<S315>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S317>/Data Type Duplicate' : Unused code path elimination
 * Block '<S317>/Data Type Propagation' : Unused code path elimination
 * Block '<S316>/Data Type Duplicate' : Unused code path elimination
 * Block '<S316>/Data Type Propagation' : Unused code path elimination
 * Block '<S318>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S320>/Data Type Duplicate' : Unused code path elimination
 * Block '<S320>/Data Type Propagation' : Unused code path elimination
 * Block '<S319>/Data Type Duplicate' : Unused code path elimination
 * Block '<S319>/Data Type Propagation' : Unused code path elimination
 * Block '<S321>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S323>/Data Type Duplicate' : Unused code path elimination
 * Block '<S323>/Data Type Propagation' : Unused code path elimination
 * Block '<S322>/Data Type Duplicate' : Unused code path elimination
 * Block '<S322>/Data Type Propagation' : Unused code path elimination
 * Block '<S273>/Discrete-Time Integrator' : Unused code path elimination
 * Block '<S337>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S339>/Data Type Duplicate' : Unused code path elimination
 * Block '<S339>/Data Type Propagation' : Unused code path elimination
 * Block '<S338>/Data Type Duplicate' : Unused code path elimination
 * Block '<S338>/Data Type Propagation' : Unused code path elimination
 * Block '<S331>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S340>/Data Type Duplicate' : Unused code path elimination
 * Block '<S340>/Data Type Propagation' : Unused code path elimination
 * Block '<S332>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S341>/Data Type Duplicate' : Unused code path elimination
 * Block '<S341>/Data Type Propagation' : Unused code path elimination
 * Block '<S333>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S342>/Data Type Duplicate' : Unused code path elimination
 * Block '<S342>/Data Type Propagation' : Unused code path elimination
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
 * Block '<S8>/Cast To Boolean3' : Eliminate redundant data type conversion
 * Block '<S207>/Cast To Double6' : Eliminate redundant data type conversion
 * Block '<S207>/Cast To Double7' : Eliminate redundant data type conversion
 * Block '<S270>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S270>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S272>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S272>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S272>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S272>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S330>/Gain1' : Eliminated nontunable gain of 1
 * Block '<S343>/Cast To Boolean1' : Eliminate redundant data type conversion
 * Block '<S346>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S347>/Cast To Single' : Eliminate redundant data type conversion
 * Block '<S347>/Cast To Single2' : Eliminate redundant data type conversion
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
 * '<S29>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem1'
 * '<S30>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL'
 * '<S31>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR'
 * '<S32>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R'
 * '<S33>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant'
 * '<S34>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant1'
 * '<S35>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant2'
 * '<S36>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant3'
 * '<S37>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant4'
 * '<S38>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant5'
 * '<S39>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/MATLAB Function'
 * '<S40>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/MATLAB Function1'
 * '<S41>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic'
 * '<S42>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic1'
 * '<S43>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic2'
 * '<S44>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic3'
 * '<S45>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic4'
 * '<S46>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic5'
 * '<S47>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic6'
 * '<S48>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic'
 * '<S49>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic1'
 * '<S50>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic2'
 * '<S51>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic3'
 * '<S52>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic4'
 * '<S53>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic5'
 * '<S54>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic6'
 * '<S55>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic7'
 * '<S56>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Wtarget'
 * '<S57>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/载荷转移'
 * '<S58>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S59>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S60>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S61>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S62>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic4/Saturation Dynamic'
 * '<S63>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic5/Saturation Dynamic'
 * '<S64>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic6/Saturation Dynamic'
 * '<S65>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S66>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S67>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S68>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem/MATLAB Function'
 * '<S69>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Compare To Constant'
 * '<S70>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Compare To Constant2'
 * '<S71>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Detect Rise Positive'
 * '<S72>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Latch_on'
 * '<S73>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/RisingTimer'
 * '<S74>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic'
 * '<S75>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic1'
 * '<S76>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic2'
 * '<S77>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Detect Rise Positive/Positive'
 * '<S78>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Compare To Constant'
 * '<S79>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Compare To Constant2'
 * '<S80>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Detect Rise Positive'
 * '<S81>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Latch_on'
 * '<S82>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/RisingTimer'
 * '<S83>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic'
 * '<S84>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic1'
 * '<S85>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic2'
 * '<S86>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Detect Rise Positive/Positive'
 * '<S87>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant'
 * '<S88>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant1'
 * '<S89>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant2'
 * '<S90>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Zero'
 * '<S91>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Detect Rise Positive'
 * '<S92>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Latch_on'
 * '<S93>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/RisingTimer'
 * '<S94>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic'
 * '<S95>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic1'
 * '<S96>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic2'
 * '<S97>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Detect Rise Positive/Positive'
 * '<S98>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant'
 * '<S99>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant1'
 * '<S100>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant2'
 * '<S101>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant3'
 * '<S102>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant4'
 * '<S103>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant5'
 * '<S104>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant6'
 * '<S105>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant7'
 * '<S106>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Timer1'
 * '<S107>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Timer2'
 * '<S108>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem'
 * '<S109>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/Chart2'
 * '<S110>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/Compare To Constant'
 * '<S111>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/Compare To Constant1'
 * '<S112>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule'
 * '<S113>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule1'
 * '<S114>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule2'
 * '<S115>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule3'
 * '<S116>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule4'
 * '<S117>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule5'
 * '<S118>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive'
 * '<S119>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive'
 * '<S120>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS'
 * '<S121>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/BMS_Recive'
 * '<S122>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE'
 * '<S123>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve'
 * '<S124>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive'
 * '<S125>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key'
 * '<S126>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state'
 * '<S127>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state'
 * '<S128>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule1'
 * '<S129>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule2'
 * '<S130>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule3'
 * '<S131>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule4'
 * '<S132>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU'
 * '<S133>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU'
 * '<S134>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state'
 * '<S135>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1'
 * '<S136>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2'
 * '<S137>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state'
 * '<S138>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule1'
 * '<S139>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule2'
 * '<S140>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule3'
 * '<S141>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule4'
 * '<S142>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule5'
 * '<S143>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule6'
 * '<S144>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule7'
 * '<S145>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule8'
 * '<S146>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule9'
 * '<S147>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state'
 * '<S148>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state/MeaModule1'
 * '<S149>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state'
 * '<S150>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule1'
 * '<S151>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule2'
 * '<S152>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule3'
 * '<S153>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state'
 * '<S154>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1'
 * '<S155>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2'
 * '<S156>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state'
 * '<S157>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule1'
 * '<S158>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule2'
 * '<S159>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule3'
 * '<S160>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule4'
 * '<S161>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule5'
 * '<S162>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule6'
 * '<S163>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule7'
 * '<S164>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule8'
 * '<S165>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state'
 * '<S166>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state/MeaModule1'
 * '<S167>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state'
 * '<S168>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule1'
 * '<S169>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule2'
 * '<S170>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule3'
 * '<S171>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule1'
 * '<S172>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule2'
 * '<S173>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule3'
 * '<S174>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule4'
 * '<S175>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/BMS_Recive/ABS_BUS_state'
 * '<S176>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/BMS_Recive/ABS_BUS_state/IMU_state'
 * '<S177>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr'
 * '<S178>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state'
 * '<S179>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1'
 * '<S180>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule1'
 * '<S181>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule2'
 * '<S182>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule3'
 * '<S183>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule4'
 * '<S184>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule5'
 * '<S185>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state'
 * '<S186>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule'
 * '<S187>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule1'
 * '<S188>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule2'
 * '<S189>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule3'
 * '<S190>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule5'
 * '<S191>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule6'
 * '<S192>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state'
 * '<S193>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state'
 * '<S194>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule2'
 * '<S195>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule3'
 * '<S196>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule4'
 * '<S197>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state'
 * '<S198>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state'
 * '<S199>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule1'
 * '<S200>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule2'
 * '<S201>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule3'
 * '<S202>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/MeaModule1'
 * '<S203>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/MeaModule2'
 * '<S204>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/Timer'
 * '<S205>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/Timer1'
 * '<S206>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem'
 * '<S207>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem'
 * '<S208>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal'
 * '<S209>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU'
 * '<S210>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS'
 * '<S211>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem'
 * '<S212>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps'
 * '<S213>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant'
 * '<S214>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant1'
 * '<S215>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant10'
 * '<S216>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant11'
 * '<S217>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant2'
 * '<S218>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant3'
 * '<S219>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant4'
 * '<S220>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant5'
 * '<S221>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant6'
 * '<S222>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant7'
 * '<S223>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant8'
 * '<S224>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant9'
 * '<S225>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule1'
 * '<S226>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule2'
 * '<S227>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule3'
 * '<S228>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule5'
 * '<S229>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule7'
 * '<S230>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule8'
 * '<S231>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Timer'
 * '<S232>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem'
 * '<S233>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic'
 * '<S234>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1'
 * '<S235>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2'
 * '<S236>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic'
 * '<S237>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic1'
 * '<S238>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic2'
 * '<S239>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S240>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S241>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S242>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Compare To Constant'
 * '<S243>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic'
 * '<S244>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1'
 * '<S245>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Timer'
 * '<S246>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S247>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S248>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem/MeaModule'
 * '<S249>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem/MeaModule1'
 * '<S250>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant'
 * '<S251>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant1'
 * '<S252>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant2'
 * '<S253>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant3'
 * '<S254>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant4'
 * '<S255>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant5'
 * '<S256>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant6'
 * '<S257>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant7'
 * '<S258>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic'
 * '<S259>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1'
 * '<S260>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2'
 * '<S261>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3'
 * '<S262>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer'
 * '<S263>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer1'
 * '<S264>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer2'
 * '<S265>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer3'
 * '<S266>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S267>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S268>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S269>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S270>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment '
 * '<S271>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/MeaModule'
 * '<S272>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment'
 * '<S273>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect'
 * '<S274>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/坐标系转换'
 * '<S275>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment /Compare To Constant'
 * '<S276>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment /Compare To Constant1'
 * '<S277>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer'
 * '<S278>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer1'
 * '<S279>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer2'
 * '<S280>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer3'
 * '<S281>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断'
 * '<S282>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断'
 * '<S283>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断'
 * '<S284>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant'
 * '<S285>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant1'
 * '<S286>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant2'
 * '<S287>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant3'
 * '<S288>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter'
 * '<S289>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1'
 * '<S290>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2'
 * '<S291>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3'
 * '<S292>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic'
 * '<S293>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Saturation Dynamic'
 * '<S294>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S295>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic'
 * '<S296>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Saturation Dynamic'
 * '<S297>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S298>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic'
 * '<S299>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Saturation Dynamic'
 * '<S300>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S301>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic'
 * '<S302>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Saturation Dynamic'
 * '<S303>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S304>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant'
 * '<S305>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant1'
 * '<S306>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant2'
 * '<S307>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant3'
 * '<S308>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter'
 * '<S309>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1'
 * '<S310>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2'
 * '<S311>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3'
 * '<S312>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic'
 * '<S313>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Saturation Dynamic'
 * '<S314>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S315>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic'
 * '<S316>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Saturation Dynamic'
 * '<S317>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S318>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic'
 * '<S319>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Saturation Dynamic'
 * '<S320>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S321>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic'
 * '<S322>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Saturation Dynamic'
 * '<S323>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S324>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant'
 * '<S325>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant1'
 * '<S326>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant2'
 * '<S327>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant3'
 * '<S328>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant'
 * '<S329>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant1'
 * '<S330>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter'
 * '<S331>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic'
 * '<S332>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1'
 * '<S333>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2'
 * '<S334>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Subsystem'
 * '<S335>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Timer1'
 * '<S336>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Timer2'
 * '<S337>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic'
 * '<S338>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Saturation Dynamic'
 * '<S339>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S340>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S341>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S342>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S343>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper'
 * '<S344>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2AMKMCUFL'
 * '<S345>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2AMKMCUFR'
 * '<S346>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU'
 * '<S347>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/WP_OUTPUT'
 * '<S348>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Chart'
 * '<S349>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Compare To Constant1'
 * '<S350>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Compare To Constant2'
 * '<S351>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Detect Fall Nonpositive'
 * '<S352>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Detect Rise Positive'
 * '<S353>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Enabled Subsystem1'
 * '<S354>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/MeaModule'
 * '<S355>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/MeaModule1'
 * '<S356>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/MeaModule2'
 * '<S357>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/RisingTimer'
 * '<S358>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Timer1'
 * '<S359>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Timer2'
 * '<S360>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Detect Fall Nonpositive/Nonpositive'
 * '<S361>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Detect Rise Positive/Positive'
 * '<S362>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Enabled Subsystem1/Chart'
 * '<S363>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU/MeaModule1'
 * '<S364>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU/MeaModule2'
 * '<S365>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL'
 * '<S366>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ'
 * '<S367>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/EEPROM'
 * '<S368>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling'
 * '<S369>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem'
 * '<S370>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem'
 * '<S371>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem'
 * '<S372>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com0'
 * '<S373>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com1'
 * '<S374>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com2'
 * '<S375>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem'
 * '<S376>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem1'
 * '<S377>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq100ms'
 * '<S378>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq10ms'
 * '<S379>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq500ms'
 * '<S380>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq50ms'
 * '<S381>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq5ms'
 * '<S382>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/EEPROM/EEPROMOperation'
 * '<S383>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling/CCPBackground'
 * '<S384>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling/CCPReceive'
 * '<S385>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling/CCPReceive/Nothing'
 */
#endif                   /* RTW_HEADER_VehCtrlMdel240926_2018b_amkspdlimit_h_ */

/* File trailer for ECUCoder generated file VehCtrlMdel240926_2018b_amkspdlimit.h.
 *
 * [EOF]
 */
