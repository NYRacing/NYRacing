/*
 * Code generated for Simulink model VehCtrlMdel240926_2018b_amkspdlimit.
 *
 * FILE    : VehCtrlMdel240926_2018b_amkspdlimit.h
 *
 * VERSION : 1.275
 *
 * DATE    : Thu Oct 17 13:17:54 2024
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

/* Block states (default storage) for system '<S126>/Timer' */
typedef struct {
  real_T x;                            /* '<S126>/Timer' */
  struct {
    uint_T is_c21_VehCtrlMdel240926_2018b_:2;/* '<S126>/Timer' */
    uint_T is_active_c21_VehCtrlMdel240926:1;/* '<S126>/Timer' */
  } bitsForTID3;
} DW_Timer_VehCtrlMdel240926_20_T;

/* Block signals (default storage) */
typedef struct {
  CAN_DATATYPE CANPack1;               /* '<S347>/CAN Pack1' */
  CAN_DATATYPE CANPack1_b;             /* '<S346>/CAN Pack1' */
  CAN_DATATYPE CANPack1_d;             /* '<S345>/CAN Pack1' */
  CAN_DATATYPE CANUnPackMessage4;      /* '<S186>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_g;    /* '<S180>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_j;    /* '<S157>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_n;    /* '<S168>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_p;    /* '<S166>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_e;    /* '<S138>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_c;    /* '<S150>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_b;    /* '<S148>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_l;    /* '<S199>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_ja;   /* '<S128>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_h;    /* '<S194>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_i;    /* '<S177>/CANUnPackMessage4' */
  real_T Switch1;                      /* '<S346>/Switch1' */
  real_T Switch;                       /* '<S346>/Switch' */
  real_T Switch1_l;                    /* '<S345>/Switch1' */
  real_T MCFL_TorqueLimitP;            /* '<S345>/Switch' */
  real_T Exit;                         /* '<S344>/Timer2' */
  real_T Exit_o;                       /* '<S344>/Timer1' */
  real_T LEDOn;                        /* '<S354>/Chart' */
  real_T Exit_i;                       /* '<S274>/Timer2' */
  real_T Exit_l;                       /* '<S274>/Timer1' */
  real_T Exit_a;                       /* '<S273>/Timer3' */
  real_T Exit_lh;                      /* '<S273>/Timer2' */
  real_T Exit_lh4;                     /* '<S273>/Timer1' */
  real_T Exit_c;                       /* '<S273>/Timer' */
  real_T Exit_h;                       /* '<S213>/Timer3' */
  real_T Exit_o4;                      /* '<S213>/Timer2' */
  real_T Exit_is;                      /* '<S213>/Timer1' */
  real_T Exit_le;                      /* '<S213>/Timer' */
  real_T Exit_on;                      /* '<S211>/Timer' */
  real_T AMKSWITCH_bx;
  real_T ignition_d;
  real_T low_VOL;                      /* '<S186>/CAN Unpack' */
  real_T MCU_Temp_error;               /* '<S186>/CAN Unpack' */
  real_T Mode;                         /* '<S186>/CAN Unpack' */
  real_T motorTemp_error;              /* '<S186>/CAN Unpack' */
  real_T overCurrent;                  /* '<S186>/CAN Unpack' */
  real_T overpower;                    /* '<S186>/CAN Unpack' */
  real_T overvol;                      /* '<S186>/CAN Unpack' */
  real_T Precharge;                    /* '<S186>/CAN Unpack' */
  real_T Reslove_error;                /* '<S186>/CAN Unpack' */
  real_T MCFR_bDerating;               /* '<S157>/CAN Unpack' */
  real_T MCFR_bQuitDCOn;               /* '<S157>/CAN Unpack' */
  real_T MCFR_bReserve;                /* '<S157>/CAN Unpack' */
  real_T MCFR_bWarn;                   /* '<S157>/CAN Unpack' */
  real_T MCFR_DiagnosticNum;           /* '<S166>/CAN Unpack' */
  real_T MCFL_bDerating;               /* '<S138>/CAN Unpack' */
  real_T MCFL_bReserve;                /* '<S138>/CAN Unpack' */
  real_T MCFL_bWarn;                   /* '<S138>/CAN Unpack' */
  real_T MCFL_DiagnosticNum;           /* '<S148>/CAN Unpack' */
  real_T CANUnpack_o1;                 /* '<S194>/CAN Unpack' */
  real_T CANUnpack_o3;                 /* '<S194>/CAN Unpack' */
  real_T CANUnpack_o5;                 /* '<S194>/CAN Unpack' */
  real_T CANUnpack_o6;                 /* '<S194>/CAN Unpack' */
  real_T Switch11;                     /* '<S109>/Switch11' */
  real_T errorReset;                   /* '<S109>/Chart2' */
  real_T Exit_d;                       /* '<S8>/Timer2' */
  real_T Exit_g;                       /* '<S8>/Timer1' */
  real_T DYC_Enable_OUT;               /* '<S7>/Chart' */
  real_T TCSR_Enable_OUT;              /* '<S7>/Chart' */
  real_T TCSF_Enable_OUT;              /* '<S7>/Chart' */
  uint32_T CANReceive_o3;              /* '<S385>/CANReceive' */
  uint32_T CANReceive_o3_l;            /* '<S370>/CANReceive' */
  uint32_T CANReceive1_o3;             /* '<S123>/CANReceive1' */
  uint32_T CANReceive3_o3;             /* '<S123>/CANReceive3' */
  uint32_T CANReceive3_o3_e;           /* '<S133>/CANReceive3' */
  uint32_T CANReceive1_o3_n;           /* '<S133>/CANReceive1' */
  uint32_T CANReceive2_o3;             /* '<S133>/CANReceive2' */
  uint32_T CANReceive3_o3_i;           /* '<S134>/CANReceive3' */
  uint32_T CANReceive1_o3_h;           /* '<S134>/CANReceive1' */
  uint32_T CANReceive2_o3_j;           /* '<S134>/CANReceive2' */
  uint32_T CANReceive3_o3_c;           /* '<S125>/CANReceive3' */
  uint32_T CANReceive3_o3_m;           /* '<S119>/CANReceive3' */
  uint32_T CANReceive3_o3_cz;          /* '<S124>/CANReceive3' */
  uint32_T CANReceive3_o3_l;           /* '<S122>/CANReceive3' */
  real32_T CastToBoolean4;             /* '<S347>/Cast To Boolean4' */
  real32_T CastToBoolean6;             /* '<S347>/Cast To Boolean6' */
  real32_T VCU_SpdCmd_Emrax;           /* '<S7>/Constant13' */
  int32_T DataTypeConversion2;         /* '<S347>/Data Type Conversion2' */
  uint16_T CastToSingle1;              /* '<S348>/Cast To Single1' */
  uint8_T CANReceive_o2;               /* '<S385>/CANReceive' */
  uint8_T CANReceive_o4[8];            /* '<S385>/CANReceive' */
  uint8_T CANReceive_o5;               /* '<S385>/CANReceive' */
  uint8_T CANReceive_o2_p;             /* '<S370>/CANReceive' */
  uint8_T CANReceive_o4_i[8];          /* '<S370>/CANReceive' */
  uint8_T CANReceive_o5_l;             /* '<S370>/CANReceive' */
  uint8_T CANTransmit;                 /* '<S377>/CANTransmit' */
  uint8_T CANPackMessage[8];           /* '<S347>/CANPackMessage' */
  uint8_T CANTransmit_k;               /* '<S347>/CANTransmit' */
  uint8_T CANPackMessage_f[8];         /* '<S346>/CANPackMessage' */
  uint8_T CANTransmit_l;               /* '<S346>/CANTransmit' */
  uint8_T CANPackMessage_h[8];         /* '<S345>/CANPackMessage' */
  uint8_T CANTransmit_c;               /* '<S345>/CANTransmit' */
  uint8_T CANReceive1_o2;              /* '<S123>/CANReceive1' */
  uint8_T CANReceive1_o4[8];           /* '<S123>/CANReceive1' */
  uint8_T CANReceive1_o5;              /* '<S123>/CANReceive1' */
  uint8_T CANReceive3_o2;              /* '<S123>/CANReceive3' */
  uint8_T CANReceive3_o4[8];           /* '<S123>/CANReceive3' */
  uint8_T CANReceive3_o5;              /* '<S123>/CANReceive3' */
  uint8_T CANReceive3_o2_l;            /* '<S133>/CANReceive3' */
  uint8_T CANReceive3_o4_l[8];         /* '<S133>/CANReceive3' */
  uint8_T CANReceive3_o5_a;            /* '<S133>/CANReceive3' */
  uint8_T CANReceive1_o2_l;            /* '<S133>/CANReceive1' */
  uint8_T CANReceive1_o4_c[8];         /* '<S133>/CANReceive1' */
  uint8_T CANReceive1_o5_a;            /* '<S133>/CANReceive1' */
  uint8_T CANReceive2_o2;              /* '<S133>/CANReceive2' */
  uint8_T CANReceive2_o4[6];           /* '<S133>/CANReceive2' */
  uint8_T CANReceive2_o5;              /* '<S133>/CANReceive2' */
  uint8_T CANReceive3_o2_a;            /* '<S134>/CANReceive3' */
  uint8_T CANReceive3_o4_g[8];         /* '<S134>/CANReceive3' */
  uint8_T CANReceive3_o5_an;           /* '<S134>/CANReceive3' */
  uint8_T CANReceive1_o2_o;            /* '<S134>/CANReceive1' */
  uint8_T CANReceive1_o4_j[8];         /* '<S134>/CANReceive1' */
  uint8_T CANReceive1_o5_j;            /* '<S134>/CANReceive1' */
  uint8_T CANReceive2_o2_p;            /* '<S134>/CANReceive2' */
  uint8_T CANReceive2_o4_k[6];         /* '<S134>/CANReceive2' */
  uint8_T CANReceive2_o5_e;            /* '<S134>/CANReceive2' */
  uint8_T CANReceive3_o2_p;            /* '<S125>/CANReceive3' */
  uint8_T CANReceive3_o4_k[8];         /* '<S125>/CANReceive3' */
  uint8_T CANReceive3_o5_d;            /* '<S125>/CANReceive3' */
  uint8_T CANReceive3_o2_m;            /* '<S119>/CANReceive3' */
  uint8_T CANReceive3_o4_lg[8];        /* '<S119>/CANReceive3' */
  uint8_T CANReceive3_o5_b;            /* '<S119>/CANReceive3' */
  uint8_T CANReceive3_o2_ma;           /* '<S124>/CANReceive3' */
  uint8_T CANReceive3_o4_i[8];         /* '<S124>/CANReceive3' */
  uint8_T CANReceive3_o5_m;            /* '<S124>/CANReceive3' */
  uint8_T CANReceive3_o2_k;            /* '<S122>/CANReceive3' */
  uint8_T CANReceive3_o4_p[8];         /* '<S122>/CANReceive3' */
  uint8_T CANReceive3_o5_de;           /* '<S122>/CANReceive3' */
  boolean_T Drive_ready;               /* '<S126>/SwitchInput' */
  boolean_T SwitchInput1;              /* '<S126>/SwitchInput1' */
  boolean_T out2_c;                    /* '<S126>/SwitchInput3' */
  boolean_T out2_h;
  boolean_T MCFL_DCOn_setpoints_o;     /* '<S109>/Switch4' */
  boolean_T VehReady;                  /* '<S109>/Chart2' */
  boolean_T MCFL_DCEnable;             /* '<S109>/Chart2' */
  boolean_T MCFR_TorqueOn;             /* '<S109>/Chart2' */
  boolean_T MCFL_TorqueOn;             /* '<S109>/Chart2' */
  boolean_T AMKMCFL_InverterOn;        /* '<S109>/Chart2' */
  boolean_T aWaterPumpON;
  boolean_T bWaterPumpON;
} B_VehCtrlMdel240926_2018b_amk_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DelayInput2_DSTATE;           /* '<S259>/Delay Input2' */
  real_T DelayInput2_DSTATE_b;         /* '<S260>/Delay Input2' */
  real_T DelayInput2_DSTATE_h;         /* '<S261>/Delay Input2' */
  real_T DelayInput2_DSTATE_n;         /* '<S262>/Delay Input2' */
  real_T DelayInput2_DSTATE_l;         /* '<S234>/Delay Input2' */
  real_T UnitDelay_DSTATE;             /* '<S233>/Unit Delay' */
  real_T DelayInput2_DSTATE_m;         /* '<S235>/Delay Input2' */
  real_T UnitDelay1_DSTATE;            /* '<S233>/Unit Delay1' */
  real_T DelayInput2_DSTATE_k;         /* '<S236>/Delay Input2' */
  real_T UnitDelay2_DSTATE;            /* '<S233>/Unit Delay2' */
  real_T UnitDelay_DSTATE_n;           /* '<S7>/Unit Delay' */
  real_T DelayInput2_DSTATE_p;         /* '<S43>/Delay Input2' */
  real_T DelayInput2_DSTATE_nk;        /* '<S46>/Delay Input2' */
  real_T UnitDelay_DSTATE_h;           /* '<S10>/Unit Delay' */
  real_T UnitDelay1_DSTATE_c;          /* '<S10>/Unit Delay1' */
  real_T DelayInput2_DSTATE_pt;        /* '<S42>/Delay Input2' */
  real_T DelayInput2_DSTATE_lt;        /* '<S47>/Delay Input2' */
  real_T DelayInput2_DSTATE_hj;        /* '<S41>/Delay Input2' */
  real_T x;                            /* '<S126>/Timer1' */
  real_T b;                            /* '<S7>/Chart' */
  real_T DYC_flag;                     /* '<S7>/Chart' */
  real32_T UnitDelay1_DSTATE_c0;       /* '<S358>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_p;         /* '<S289>/Unit Delay' */
  real32_T DelayInput2_DSTATE_n2;      /* '<S293>/Delay Input2' */
  real32_T UnitDelay_DSTATE_j;         /* '<S282>/Unit Delay' */
  real32_T UnitDelay_DSTATE_pj;        /* '<S290>/Unit Delay' */
  real32_T DelayInput2_DSTATE_e;       /* '<S296>/Delay Input2' */
  real32_T UnitDelay1_DSTATE_n;        /* '<S282>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_a;         /* '<S291>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hk;      /* '<S299>/Delay Input2' */
  real32_T UnitDelay2_DSTATE_l;        /* '<S282>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_nc;        /* '<S292>/Unit Delay' */
  real32_T DelayInput2_DSTATE_c;       /* '<S302>/Delay Input2' */
  real32_T UnitDelay3_DSTATE;          /* '<S282>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_l;         /* '<S309>/Unit Delay' */
  real32_T DelayInput2_DSTATE_i;       /* '<S313>/Delay Input2' */
  real32_T UnitDelay4_DSTATE;          /* '<S283>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_a0;        /* '<S283>/Unit Delay' */
  real32_T UnitDelay_DSTATE_ap;        /* '<S310>/Unit Delay' */
  real32_T DelayInput2_DSTATE_el;      /* '<S316>/Delay Input2' */
  real32_T UnitDelay5_DSTATE;          /* '<S283>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_a;        /* '<S283>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_o;         /* '<S311>/Unit Delay' */
  real32_T DelayInput2_DSTATE_pd;      /* '<S319>/Delay Input2' */
  real32_T UnitDelay6_DSTATE;          /* '<S283>/Unit Delay6' */
  real32_T UnitDelay2_DSTATE_c;        /* '<S283>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_ah;        /* '<S312>/Unit Delay' */
  real32_T DelayInput2_DSTATE_mt;      /* '<S322>/Delay Input2' */
  real32_T UnitDelay7_DSTATE;          /* '<S283>/Unit Delay7' */
  real32_T UnitDelay3_DSTATE_d;        /* '<S283>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_lh;        /* '<S208>/Unit Delay' */
  real32_T UnitDelay4_DSTATE_m;        /* '<S271>/Unit Delay4' */
  real32_T UnitDelay1_DSTATE_k;        /* '<S208>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_o;        /* '<S271>/Unit Delay1' */
  real32_T UnitDelay3_DSTATE_p;        /* '<S274>/Unit Delay3' */
  real32_T UnitDelay1_DSTATE_d;        /* '<S274>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_f;        /* '<S274>/Unit Delay2' */
  real32_T UnitDelay4_DSTATE_mn;       /* '<S274>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_d;         /* '<S271>/Unit Delay' */
  real32_T UnitDelay2_DSTATE_i;        /* '<S271>/Unit Delay2' */
  real32_T DelayInput2_DSTATE_g;       /* '<S334>/Delay Input2' */
  real32_T DelayInput2_DSTATE_a;       /* '<S332>/Delay Input2' */
  real32_T DelayInput2_DSTATE_f;       /* '<S338>/Delay Input2' */
  real32_T UnitDelay_DSTATE_ncs;       /* '<S331>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hu;      /* '<S333>/Delay Input2' */
  real32_T DelayInput2_DSTATE_l2;      /* '<S244>/Delay Input2' */
  real32_T UnitDelay_DSTATE_o2;        /* '<S211>/Unit Delay' */
  real32_T DelayInput2_DSTATE_j;       /* '<S245>/Delay Input2' */
  real32_T UnitDelay1_DSTATE_aq;       /* '<S211>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_m;         /* '<S28>/Unit Delay' */
  real32_T DelayInput2_DSTATE_l4;      /* '<S44>/Delay Input2' */
  real32_T DelayInput2_DSTATE_j3;      /* '<S45>/Delay Input2' */
  real32_T UnitDelay4_DSTATE_j;        /* '<S10>/Unit Delay4' */
  real32_T UnitDelay2_DSTATE_j;        /* '<S10>/Unit Delay2' */
  real32_T UnitDelay5_DSTATE_k;        /* '<S10>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_g;        /* '<S32>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_i;        /* '<S94>/Unit Delay1' */
  real32_T UnitDelay4_DSTATE_b;        /* '<S32>/Unit Delay4' */
  real32_T UnitDelay5_DSTATE_l;        /* '<S32>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_b;         /* '<S32>/Unit Delay' */
  real32_T UnitDelay2_DSTATE_g;        /* '<S32>/Unit Delay2' */
  real32_T UnitDelay1_DSTATE_n5;       /* '<S83>/Unit Delay1' */
  real32_T UnitDelay4_DSTATE_i;        /* '<S31>/Unit Delay4' */
  real32_T UnitDelay1_DSTATE_h;        /* '<S74>/Unit Delay1' */
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
  int32_T sfEvent;                     /* '<S109>/Chart2' */
  uint32_T previousTicks;              /* '<S354>/Chart' */
  uint32_T previousTicks_m;            /* '<S344>/Chart' */
  uint32_T Subsystem_PREV_T;           /* '<S4>/Subsystem' */
  uint32_T FunctionCallSubsystem_PREV_T;/* '<S4>/Function-Call Subsystem' */
  uint32_T previousTicks_g;            /* '<S109>/Chart2' */
  uint32_T MoTrqReq_PREV_T;            /* '<S1>/MoTrqReq' */
  int_T CANPack1_ModeSignalID;         /* '<S347>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_g;       /* '<S346>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_f;       /* '<S345>/CAN Pack1' */
  int_T CANUnpack_ModeSignalID;        /* '<S186>/CAN Unpack' */
  int_T CANUnpack_StatusPortID;        /* '<S186>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_b;      /* '<S180>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_m;      /* '<S180>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_a;      /* '<S157>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_o;      /* '<S157>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_n;      /* '<S168>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_f;      /* '<S168>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_p;      /* '<S166>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_e;      /* '<S166>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_c;      /* '<S138>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_ok;     /* '<S138>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_m;      /* '<S150>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_j;      /* '<S150>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_i;      /* '<S148>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_mj;     /* '<S148>/CAN Unpack' */
  int_T CANUnpack1_ModeSignalID;       /* '<S199>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID;       /* '<S199>/CAN Unpack1' */
  int_T CANUnpack1_ModeSignalID_m;     /* '<S128>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID_n;     /* '<S128>/CAN Unpack1' */
  int_T CANUnpack_ModeSignalID_f;      /* '<S194>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_k;      /* '<S194>/CAN Unpack' */
  struct {
    uint_T is_VehStat:4;               /* '<S109>/Chart2' */
    uint_T is_AMKDCready:4;            /* '<S109>/Chart2' */
    uint_T is_c27_VehCtrlMdel240926_2018b_:3;/* '<S354>/Chart' */
    uint_T is_StateA:2;                /* '<S354>/Chart' */
    uint_T is_StateB:2;                /* '<S354>/Chart' */
    uint_T is_StateC:2;                /* '<S354>/Chart' */
    uint_T is_c24_VehCtrlMdel240926_2018b_:2;/* '<S344>/Chart' */
    uint_T is_STATEON:2;               /* '<S344>/Chart' */
    uint_T is_STATEOFF:2;              /* '<S344>/Chart' */
    uint_T is_c23_VehCtrlMdel240926_2018b_:2;/* '<S126>/Timer1' */
    uint_T is_BeeperStat:2;            /* '<S109>/Chart2' */
    uint_T is_AMKDCon:2;               /* '<S109>/Chart2' */
    uint_T is_MCDCEnable:2;            /* '<S109>/Chart2' */
    uint_T is_MC_TorqueCUT:2;          /* '<S109>/Chart2' */
    uint_T is_AMKCANenable:2;          /* '<S109>/Chart2' */
    uint_T is_MCFL_InverterOn:2;       /* '<S109>/Chart2' */
    uint_T is_MCFR_InverterOn:2;       /* '<S109>/Chart2' */
    uint_T is_B:2;                     /* '<S7>/Chart' */
    uint_T is_C:2;                     /* '<S7>/Chart' */
    uint_T is_D:2;                     /* '<S7>/Chart' */
    uint_T is_E:2;                     /* '<S7>/Chart' */
    uint_T is_active_c27_VehCtrlMdel240926:1;/* '<S354>/Chart' */
    uint_T is_active_c24_VehCtrlMdel240926:1;/* '<S344>/Chart' */
    uint_T is_active_c23_VehCtrlMdel240926:1;/* '<S126>/Timer1' */
    uint_T is_active_c1_VehCtrlMdel240926_:1;/* '<S109>/Chart2' */
    uint_T is_active_VehStat:1;        /* '<S109>/Chart2' */
    uint_T is_active_BeeperStat:1;     /* '<S109>/Chart2' */
    uint_T is_active_AMKDCon:1;        /* '<S109>/Chart2' */
    uint_T is_active_MCDCEnable:1;     /* '<S109>/Chart2' */
    uint_T is_active_MC_TorqueCUT:1;   /* '<S109>/Chart2' */
    uint_T is_active_AMKDCready:1;     /* '<S109>/Chart2' */
    uint_T is_active_Output:1;         /* '<S109>/Chart2' */
    uint_T is_active_AMKCANenable:1;   /* '<S109>/Chart2' */
    uint_T is_active_MCFL_InverterOn:1;/* '<S109>/Chart2' */
    uint_T is_active_MCFR_InverterOn:1;/* '<S109>/Chart2' */
    uint_T is_active_c7_VehCtrlMdel240926_:1;/* '<S7>/Chart' */
  } bitsForTID3;

  uint16_T UnitDelay1_DSTATE_fm;       /* '<S209>/Unit Delay1' */
  uint16_T UnitDelay_DSTATE_k;         /* '<S209>/Unit Delay' */
  uint16_T temporalCounter_i2;         /* '<S109>/Chart2' */
  boolean_T DelayInput1_DSTATE;        /* '<S353>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_n;      /* '<S352>/Delay Input1' */
  boolean_T UnitDelay_DSTATE_l3;       /* '<S344>/Unit Delay' */
  boolean_T UnitDelay3_DSTATE_f;       /* '<S271>/Unit Delay3' */
  boolean_T UnitDelay3_DSTATE_i;       /* '<S10>/Unit Delay3' */
  boolean_T UnitDelay6_DSTATE_b;       /* '<S10>/Unit Delay6' */
  boolean_T UnitDelay1_DSTATE_e;       /* '<S93>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_a;       /* '<S32>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE_e;      /* '<S92>/Delay Input1' */
  boolean_T UnitDelay1_DSTATE_gl;      /* '<S82>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_e;       /* '<S31>/Unit Delay3' */
  boolean_T UnitDelay1_DSTATE_dp;      /* '<S73>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_ip;      /* '<S30>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE_j;      /* '<S81>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_b;      /* '<S72>/Delay Input1' */
  uint8_T temporalCounter_i1;          /* '<S354>/Chart' */
  uint8_T temporalCounter_i1_p;        /* '<S344>/Chart' */
  uint8_T temporalCounter_i1_f;        /* '<S109>/Chart2' */
  boolean_T Subsystem_RESET_ELAPS_T;   /* '<S4>/Subsystem' */
  boolean_T FunctionCallSubsystem_RESET_ELA;/* '<S4>/Function-Call Subsystem' */
  boolean_T MoTrqReq_RESET_ELAPS_T;    /* '<S1>/MoTrqReq' */
  boolean_T EnabledSubsystem1_MODE;    /* '<S344>/Enabled Subsystem1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2_h;/* '<S344>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1_h;/* '<S344>/Timer1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2_j;/* '<S274>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1_p;/* '<S274>/Timer1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer3_i;/* '<S273>/Timer3' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2_g;/* '<S273>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1_m;/* '<S273>/Timer1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer_o;/* '<S273>/Timer' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer3;/* '<S213>/Timer3' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer2_l;/* '<S213>/Timer2' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer1_n;/* '<S213>/Timer1' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer_b;/* '<S213>/Timer' */
  DW_Timer1_VehCtrlMdel240926_2_T sf_Timer_k;/* '<S211>/Timer' */
  DW_Timer_VehCtrlMdel240926_20_T sf_Timer_a;/* '<S209>/Timer' */
  DW_Timer_VehCtrlMdel240926_20_T sf_Timer;/* '<S126>/Timer' */
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

  /* Pooled Parameter (Expression: single(reshape([0,20,35,50,79,100,120,125,130,130,170,0,20,35,50,79,100,120,125,160,160,170,0,20,35,50,79,100,120,125,160,160,180,0,20,35,50,79,105,120,130,160,160,180,0,20,35,50,85,105,125,135,160,160,180,0,20,35,50,85,105,125,140,170,170,210,0,20,30,50,85,110,125,145,160,160,200,0,18,30,50,90,110,135,155,160,160,200,0,18,25,50,95,110,135,155,160,160,200,0,18,25,40,100,110,135,155,160,160,200,0,17,25,40,110,115,135,155,160,160,200,0,17,25,40,110,115,135,155,160,160,200,0,16,20,40,110,125,135,155,160,160,190,0,16,20,40,100,125,130,155,160,160,180,0,15,20,40,90,120,130,155,160,160,180,0,15,20,40,90,110,120,135,142,142,152,0,15,20,40,80,90,100,110,120,120,133,0,14,20,40,72,80,90,100,105,105,118,0,14,20,38,70,76,82,88,94,95,106,0,14,18,35,65,70,74,82,88,91,100,0,14,18,35,65,70,74,82,88,91,100],11,21));)
   * Referenced by:
   *   '<S29>/4WD_Table'
   *   '<S29>/RWD_Table'
   */
  real32_T pooled30[231];

  /* Pooled Parameter (Expression: single([0,10,20,30,40,50,60,70,80,90,100]);)
   * Referenced by:
   *   '<S29>/4WD_Table'
   *   '<S29>/RWD_Table'
   */
  real32_T pooled31[11];

  /* Pooled Parameter (Expression: single([5.400000095367432,10.800000190734863,16.200000762939453,21.600000381469727,27,32.400001525878906,37.79999923706055,43.20000076293945,48.599998474121094,54,59.400001525878906,64.80000305175781,70.19999694824219,75.5999984741211,81,86.4000015258789,91.80000305175781,97.19999694824219,102.5999984741211,108,114]);)
   * Referenced by:
   *   '<S29>/4WD_Table'
   *   '<S29>/RWD_Table'
   */
  real32_T pooled32[21];

  /* Computed Parameter: uDLookupTable1_tableData
   * Referenced by: '<S10>/2-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData[35];

  /* Computed Parameter: uDLookupTable1_bp01Data
   * Referenced by: '<S10>/2-D Lookup Table1'
   */
  real32_T uDLookupTable1_bp01Data[5];

  /* Computed Parameter: uDLookupTable1_bp02Data
   * Referenced by: '<S10>/2-D Lookup Table1'
   */
  real32_T uDLookupTable1_bp02Data[7];

  /* Pooled Parameter (Expression: [0.3,0.3,2.5,2.5])
   * Referenced by:
   *   '<S30>/VehSpd_SlipTarget_mps'
   *   '<S31>/VehSpd_SlipTarget_mps'
   *   '<S32>/VehSpd_SlipTarget_mps'
   */
  real32_T pooled55[4];

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
  real32_T pooled56[4];

  /* Pooled Parameter (Expression: [0.4,0.4,1.2,1.2])
   * Referenced by:
   *   '<S30>/VehicleStableTarget_mps'
   *   '<S30>/VehicleStableTarget_mps1'
   *   '<S31>/VehicleStableTarget_mps'
   *   '<S31>/VehicleStableTarget_mps1'
   *   '<S32>/VehicleStableTarget_mps'
   *   '<S32>/VehicleStableTarget_mps1'
   */
  real32_T pooled62[4];

  /* Expression: single([20,0]);
   * Referenced by: '<S7>/BrakeCompensateCoefFront1'
   */
  real32_T BrakeCompensateCoefFront1_table[2];

  /* Expression: single([550,1700]);
   * Referenced by: '<S7>/BrakeCompensateCoefFront1'
   */
  real32_T BrakeCompensateCoefFront1_bp01D[2];

  /* Pooled Parameter (Expression: single([500,4500]);)
   * Referenced by: '<S209>/1-D Lookup Table1'
   */
  real32_T pooled67[2];

  /* Pooled Parameter (Expression: single([0,100]);)
   * Referenced by:
   *   '<S209>/1-D Lookup Table3'
   *   '<S209>/1-D Lookup Table4'
   */
  real32_T pooled68[2];

  /* Expression: single([2589,2754])
   * Referenced by: '<S209>/1-D Lookup Table4'
   */
  real32_T uDLookupTable4_bp01Data[2];

  /* Expression: single([2471,2642])
   * Referenced by: '<S209>/1-D Lookup Table3'
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
   * Referenced by: '<S211>/1-D Lookup Table'
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
   * Referenced by: '<S211>/1-D Lookup Table'
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
   * Referenced by: '<S211>/1-D Lookup Table1'
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
   * Referenced by: '<S211>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_bp01Data_h[24];

  /* Pooled Parameter (Expression: )
   * Referenced by:
   *   '<S29>/4WD_Table'
   *   '<S29>/RWD_Table'
   */
  uint32_T pooled79[2];

  /* Computed Parameter: uDLookupTable1_maxIndex
   * Referenced by: '<S10>/2-D Lookup Table1'
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
extern real_T Gear_Trs;                /* '<S347>/Switch2' */
extern real_T Mode_Trs;                /* '<S347>/Switch3' */
extern real_T KeyPressed;
                       /* '<S207>/BusConversion_InsertedFor_Out1_at_inport_0' */
extern real_T AMKFL_Current;           /* '<S212>/Switch' */
extern real_T AMKFR_Current;           /* '<S212>/Switch1' */
extern real_T Trq_CUT;                 /* '<S209>/Timer' */
extern real_T AMKSWITCH;               /* '<S126>/Timer1' */
extern real_T ignition;                /* '<S126>/Timer' */
extern real_T L12V_error;              /* '<S186>/CAN Unpack' */
extern real_T alarm;                   /* '<S186>/CAN Unpack' */
extern real_T controller_ready;        /* '<S186>/CAN Unpack' */
extern real_T selfcheck;               /* '<S186>/CAN Unpack' */
extern real_T RPM;                     /* '<S186>/CAN Unpack' */
extern real_T trq;                     /* '<S186>/CAN Unpack' */
extern real_T AC_current;              /* '<S180>/CAN Unpack' */
extern real_T DC_current;              /* '<S180>/CAN Unpack' */
extern real_T MCU_Temp;                /* '<S180>/CAN Unpack' */
extern real_T motor_Temp;              /* '<S180>/CAN Unpack' */
extern real_T voltage;                 /* '<S180>/CAN Unpack' */
extern real_T MCFR_ActualTorque;       /* '<S157>/CAN Unpack' */
extern real_T MCFR_ActualVelocity;     /* '<S157>/CAN Unpack' */
extern real_T MCFR_DCVoltage;          /* '<S157>/CAN Unpack' */
extern real_T MCFR_bDCOn;              /* '<S157>/CAN Unpack' */
extern real_T MCFR_bError;             /* '<S157>/CAN Unpack' */
extern real_T MCFR_bInverterOn;        /* '<S157>/CAN Unpack' */
extern real_T MCFR_bQuitInverterOn;    /* '<S157>/CAN Unpack' */
extern real_T MCFR_bSystemReady;       /* '<S157>/CAN Unpack' */
extern real_T MCFR_TempIGBT;           /* '<S168>/CAN Unpack' */
extern real_T MCFR_TempInverter;       /* '<S168>/CAN Unpack' */
extern real_T MCFR_TempMotor;          /* '<S168>/CAN Unpack' */
extern real_T MCFR_ErrorInfo;          /* '<S166>/CAN Unpack' */
extern real_T MCFL_ActualTorque;       /* '<S138>/CAN Unpack' */
extern real_T MCFL_ActualVelocity;     /* '<S138>/CAN Unpack' */
extern real_T MCFL_DCVoltage;          /* '<S138>/CAN Unpack' */
extern real_T MCFL_bDCOn;              /* '<S138>/CAN Unpack' */
extern real_T MCFL_bError;             /* '<S138>/CAN Unpack' */
extern real_T MCFL_bInverterOn;        /* '<S138>/CAN Unpack' */
extern real_T MCFL_bQuitDCOn;          /* '<S138>/CAN Unpack' */
extern real_T MCFL_bQuitInverterOn;    /* '<S138>/CAN Unpack' */
extern real_T MCFL_bSystemReady;       /* '<S138>/CAN Unpack' */
extern real_T MCFL_TempIGBT;           /* '<S150>/CAN Unpack' */
extern real_T MCFL_TempInverter;       /* '<S150>/CAN Unpack' */
extern real_T MCFL_TempMotor;          /* '<S150>/CAN Unpack' */
extern real_T MCFL_ErrorInfo;          /* '<S148>/CAN Unpack' */
extern real_T StrWhlAngAliveRollCnt;   /* '<S199>/CAN Unpack1' */
extern real_T StrWhlAng;               /* '<S199>/CAN Unpack1' */
extern real_T StrWhlAngV;              /* '<S199>/CAN Unpack1' */
extern real_T ABS_WS_FL;               /* '<S128>/CAN Unpack1' */
extern real_T ABS_WS_FR;               /* '<S128>/CAN Unpack1' */
extern real_T ABS_WS_RL;               /* '<S128>/CAN Unpack1' */
extern real_T ABS_WS_RR;               /* '<S128>/CAN Unpack1' */
extern real_T IMU_Ay_Value;            /* '<S194>/CAN Unpack' */
extern real_T IMU_Ax_Value;            /* '<S194>/CAN Unpack' */
extern real_T IMU_Yaw_Value;           /* '<S194>/CAN Unpack' */
extern real_T EMRAX_Trq_CUT;           /*  */
extern real_T AMK_Trq_CUT;             /*  */
extern uint32_T Acc_vol2;              /* '<S209>/Add3' */
extern uint32_T Acc_vol;               /* '<S209>/Add2' */
extern uint32_T Acc_POS;               /* '<S209>/1-D Lookup Table4' */
extern uint32_T Acc_POS2;              /* '<S209>/1-D Lookup Table3' */
extern real32_T VehVxEst_mps;          /* '<S331>/Add' */
extern real32_T PwrALL;                /* '<S28>/Gain' */
extern real32_T EmraxTrqR_cmd;         /* '<S7>/Saturation1' */
extern real32_T AMKTrqFR_cmd;          /* '<S7>/Saturation2' */
extern real32_T AMKTrqFL_cmd;          /* '<S7>/Saturation3' */
extern uint16_T F_BrkPrs;              /* '<S209>/1-D Lookup Table1' */
extern uint16_T Acc1;                  /* '<S121>/Acc3' */
extern uint16_T Acc2;                  /* '<S121>/Acc4' */
extern uint16_T Brk1;                  /* '<S121>/Brk1' */
extern uint16_T Brk2;                  /* '<S121>/Brk2' */
extern boolean_T STATEDISPLAY;         /* '<S344>/Switch1' */
extern boolean_T HVSWITCH;             /* '<S344>/Chart' */
extern boolean_T Brk;                  /* '<S111>/Compare' */
extern boolean_T ACC_Release;          /* '<S112>/Compare' */
extern boolean_T beeper_state;         /* '<S109>/Chart2' */
extern boolean_T MCFL_DCOn_setpoints;  /* '<S109>/Chart2' */
extern boolean_T MCFR_DCEnable;        /* '<S109>/Chart2' */
extern boolean_T MCFR_InverterOn;      /* '<S109>/Chart2' */
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
 * Block '<S8>/2-D Lookup Table1' : Unused code path elimination
 * Block '<S8>/2-D Lookup Table3' : Unused code path elimination
 * Block '<S8>/2-D Lookup Table4' : Unused code path elimination
 * Block '<S8>/AND' : Unused code path elimination
 * Block '<S8>/AND1' : Unused code path elimination
 * Block '<S8>/AND3' : Unused code path elimination
 * Block '<S99>/Compare' : Unused code path elimination
 * Block '<S99>/Constant' : Unused code path elimination
 * Block '<S101>/Compare' : Unused code path elimination
 * Block '<S101>/Constant' : Unused code path elimination
 * Block '<S105>/Compare' : Unused code path elimination
 * Block '<S105>/Constant' : Unused code path elimination
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
 * Block '<S109>/Switch5' : Unused code path elimination
 * Block '<S177>/CAN Unpack1' : Unused code path elimination
 * Block '<S209>/1-D Lookup Table2' : Unused code path elimination
 * Block '<S209>/Abs1' : Unused code path elimination
 * Block '<S209>/Add4' : Unused code path elimination
 * Block '<S225>/Compare' : Unused code path elimination
 * Block '<S225>/Constant' : Unused code path elimination
 * Block '<S234>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S240>/Data Type Duplicate' : Unused code path elimination
 * Block '<S240>/Data Type Propagation' : Unused code path elimination
 * Block '<S235>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S241>/Data Type Duplicate' : Unused code path elimination
 * Block '<S241>/Data Type Propagation' : Unused code path elimination
 * Block '<S236>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S242>/Data Type Duplicate' : Unused code path elimination
 * Block '<S242>/Data Type Propagation' : Unused code path elimination
 * Block '<S237>/Data Type Duplicate' : Unused code path elimination
 * Block '<S237>/Data Type Propagation' : Unused code path elimination
 * Block '<S238>/Data Type Duplicate' : Unused code path elimination
 * Block '<S238>/Data Type Propagation' : Unused code path elimination
 * Block '<S239>/Data Type Duplicate' : Unused code path elimination
 * Block '<S239>/Data Type Propagation' : Unused code path elimination
 * Block '<S244>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S247>/Data Type Duplicate' : Unused code path elimination
 * Block '<S247>/Data Type Propagation' : Unused code path elimination
 * Block '<S245>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S248>/Data Type Duplicate' : Unused code path elimination
 * Block '<S248>/Data Type Propagation' : Unused code path elimination
 * Block '<S259>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S267>/Data Type Duplicate' : Unused code path elimination
 * Block '<S267>/Data Type Propagation' : Unused code path elimination
 * Block '<S260>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S268>/Data Type Duplicate' : Unused code path elimination
 * Block '<S268>/Data Type Propagation' : Unused code path elimination
 * Block '<S261>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S269>/Data Type Duplicate' : Unused code path elimination
 * Block '<S269>/Data Type Propagation' : Unused code path elimination
 * Block '<S262>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S270>/Data Type Duplicate' : Unused code path elimination
 * Block '<S270>/Data Type Propagation' : Unused code path elimination
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
 * Block '<S299>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S301>/Data Type Duplicate' : Unused code path elimination
 * Block '<S301>/Data Type Propagation' : Unused code path elimination
 * Block '<S300>/Data Type Duplicate' : Unused code path elimination
 * Block '<S300>/Data Type Propagation' : Unused code path elimination
 * Block '<S302>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S304>/Data Type Duplicate' : Unused code path elimination
 * Block '<S304>/Data Type Propagation' : Unused code path elimination
 * Block '<S303>/Data Type Duplicate' : Unused code path elimination
 * Block '<S303>/Data Type Propagation' : Unused code path elimination
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
 * Block '<S319>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S321>/Data Type Duplicate' : Unused code path elimination
 * Block '<S321>/Data Type Propagation' : Unused code path elimination
 * Block '<S320>/Data Type Duplicate' : Unused code path elimination
 * Block '<S320>/Data Type Propagation' : Unused code path elimination
 * Block '<S322>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S324>/Data Type Duplicate' : Unused code path elimination
 * Block '<S324>/Data Type Propagation' : Unused code path elimination
 * Block '<S323>/Data Type Duplicate' : Unused code path elimination
 * Block '<S323>/Data Type Propagation' : Unused code path elimination
 * Block '<S274>/Discrete-Time Integrator' : Unused code path elimination
 * Block '<S338>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S340>/Data Type Duplicate' : Unused code path elimination
 * Block '<S340>/Data Type Propagation' : Unused code path elimination
 * Block '<S339>/Data Type Duplicate' : Unused code path elimination
 * Block '<S339>/Data Type Propagation' : Unused code path elimination
 * Block '<S332>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S341>/Data Type Duplicate' : Unused code path elimination
 * Block '<S341>/Data Type Propagation' : Unused code path elimination
 * Block '<S333>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S342>/Data Type Duplicate' : Unused code path elimination
 * Block '<S342>/Data Type Propagation' : Unused code path elimination
 * Block '<S334>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S343>/Data Type Duplicate' : Unused code path elimination
 * Block '<S343>/Data Type Propagation' : Unused code path elimination
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
 * Block '<S208>/Cast To Double6' : Eliminate redundant data type conversion
 * Block '<S208>/Cast To Double7' : Eliminate redundant data type conversion
 * Block '<S271>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S271>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S273>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S273>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S273>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S273>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S331>/Gain1' : Eliminated nontunable gain of 1
 * Block '<S344>/Cast To Boolean1' : Eliminate redundant data type conversion
 * Block '<S347>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S348>/Cast To Single' : Eliminate redundant data type conversion
 * Block '<S348>/Cast To Single2' : Eliminate redundant data type conversion
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
 * '<S69>'  : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem/MeaModule4'
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
 * '<S104>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant5'
 * '<S105>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant6'
 * '<S106>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant7'
 * '<S107>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Timer1'
 * '<S108>' : 'VehCtrlMdel240926_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Timer2'
 * '<S109>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem'
 * '<S110>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/Chart2'
 * '<S111>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/Compare To Constant'
 * '<S112>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/Compare To Constant1'
 * '<S113>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule'
 * '<S114>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule1'
 * '<S115>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule2'
 * '<S116>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule3'
 * '<S117>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule4'
 * '<S118>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule5'
 * '<S119>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive'
 * '<S120>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive'
 * '<S121>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS'
 * '<S122>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/BMS_Recive'
 * '<S123>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE'
 * '<S124>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve'
 * '<S125>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive'
 * '<S126>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key'
 * '<S127>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state'
 * '<S128>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state'
 * '<S129>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule1'
 * '<S130>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule2'
 * '<S131>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule3'
 * '<S132>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule4'
 * '<S133>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU'
 * '<S134>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU'
 * '<S135>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state'
 * '<S136>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1'
 * '<S137>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2'
 * '<S138>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state'
 * '<S139>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule1'
 * '<S140>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule2'
 * '<S141>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule3'
 * '<S142>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule4'
 * '<S143>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule5'
 * '<S144>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule6'
 * '<S145>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule7'
 * '<S146>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule8'
 * '<S147>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule9'
 * '<S148>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state'
 * '<S149>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state/MeaModule1'
 * '<S150>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state'
 * '<S151>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule1'
 * '<S152>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule2'
 * '<S153>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule3'
 * '<S154>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state'
 * '<S155>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1'
 * '<S156>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2'
 * '<S157>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state'
 * '<S158>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule1'
 * '<S159>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule2'
 * '<S160>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule3'
 * '<S161>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule4'
 * '<S162>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule5'
 * '<S163>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule6'
 * '<S164>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule7'
 * '<S165>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule8'
 * '<S166>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state'
 * '<S167>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state/MeaModule1'
 * '<S168>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state'
 * '<S169>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule1'
 * '<S170>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule2'
 * '<S171>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule3'
 * '<S172>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule1'
 * '<S173>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule2'
 * '<S174>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule3'
 * '<S175>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule4'
 * '<S176>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/BMS_Recive/ABS_BUS_state'
 * '<S177>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/BMS_Recive/ABS_BUS_state/IMU_state'
 * '<S178>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr'
 * '<S179>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state'
 * '<S180>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1'
 * '<S181>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule1'
 * '<S182>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule2'
 * '<S183>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule3'
 * '<S184>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule4'
 * '<S185>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule5'
 * '<S186>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state'
 * '<S187>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule'
 * '<S188>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule1'
 * '<S189>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule2'
 * '<S190>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule3'
 * '<S191>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule5'
 * '<S192>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule6'
 * '<S193>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state'
 * '<S194>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state'
 * '<S195>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule2'
 * '<S196>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule3'
 * '<S197>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule4'
 * '<S198>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state'
 * '<S199>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state'
 * '<S200>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule1'
 * '<S201>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule2'
 * '<S202>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule3'
 * '<S203>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/MeaModule1'
 * '<S204>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/MeaModule2'
 * '<S205>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/Timer'
 * '<S206>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input/key/Timer1'
 * '<S207>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem'
 * '<S208>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem'
 * '<S209>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal'
 * '<S210>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU'
 * '<S211>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS'
 * '<S212>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem'
 * '<S213>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps'
 * '<S214>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant'
 * '<S215>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant1'
 * '<S216>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant10'
 * '<S217>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant11'
 * '<S218>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant2'
 * '<S219>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant3'
 * '<S220>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant4'
 * '<S221>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant5'
 * '<S222>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant6'
 * '<S223>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant7'
 * '<S224>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant8'
 * '<S225>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant9'
 * '<S226>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule1'
 * '<S227>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule2'
 * '<S228>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule3'
 * '<S229>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule5'
 * '<S230>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule7'
 * '<S231>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule8'
 * '<S232>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Timer'
 * '<S233>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem'
 * '<S234>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic'
 * '<S235>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1'
 * '<S236>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2'
 * '<S237>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic'
 * '<S238>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic1'
 * '<S239>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic2'
 * '<S240>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S241>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S242>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S243>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Compare To Constant'
 * '<S244>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic'
 * '<S245>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1'
 * '<S246>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Timer'
 * '<S247>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S248>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S249>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem/MeaModule'
 * '<S250>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem/MeaModule1'
 * '<S251>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant'
 * '<S252>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant1'
 * '<S253>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant2'
 * '<S254>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant3'
 * '<S255>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant4'
 * '<S256>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant5'
 * '<S257>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant6'
 * '<S258>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant7'
 * '<S259>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic'
 * '<S260>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1'
 * '<S261>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2'
 * '<S262>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3'
 * '<S263>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer'
 * '<S264>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer1'
 * '<S265>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer2'
 * '<S266>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer3'
 * '<S267>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S268>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S269>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S270>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S271>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment '
 * '<S272>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/MeaModule'
 * '<S273>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment'
 * '<S274>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect'
 * '<S275>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/坐标系转换'
 * '<S276>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment /Compare To Constant'
 * '<S277>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment /Compare To Constant1'
 * '<S278>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer'
 * '<S279>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer1'
 * '<S280>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer2'
 * '<S281>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer3'
 * '<S282>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断'
 * '<S283>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断'
 * '<S284>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断'
 * '<S285>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant'
 * '<S286>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant1'
 * '<S287>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant2'
 * '<S288>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant3'
 * '<S289>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter'
 * '<S290>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1'
 * '<S291>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2'
 * '<S292>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3'
 * '<S293>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic'
 * '<S294>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Saturation Dynamic'
 * '<S295>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S296>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic'
 * '<S297>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Saturation Dynamic'
 * '<S298>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S299>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic'
 * '<S300>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Saturation Dynamic'
 * '<S301>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S302>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic'
 * '<S303>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Saturation Dynamic'
 * '<S304>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S305>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant'
 * '<S306>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant1'
 * '<S307>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant2'
 * '<S308>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant3'
 * '<S309>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter'
 * '<S310>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1'
 * '<S311>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2'
 * '<S312>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3'
 * '<S313>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic'
 * '<S314>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Saturation Dynamic'
 * '<S315>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S316>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic'
 * '<S317>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Saturation Dynamic'
 * '<S318>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S319>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic'
 * '<S320>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Saturation Dynamic'
 * '<S321>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S322>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic'
 * '<S323>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Saturation Dynamic'
 * '<S324>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S325>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant'
 * '<S326>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant1'
 * '<S327>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant2'
 * '<S328>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant3'
 * '<S329>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant'
 * '<S330>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant1'
 * '<S331>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter'
 * '<S332>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic'
 * '<S333>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1'
 * '<S334>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2'
 * '<S335>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Subsystem'
 * '<S336>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Timer1'
 * '<S337>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Timer2'
 * '<S338>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic'
 * '<S339>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Saturation Dynamic'
 * '<S340>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S341>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S342>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S343>' : 'VehCtrlMdel240926_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S344>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper'
 * '<S345>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2AMKMCUFL'
 * '<S346>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2AMKMCUFR'
 * '<S347>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU'
 * '<S348>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/WP_OUTPUT'
 * '<S349>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Chart'
 * '<S350>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Compare To Constant1'
 * '<S351>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Compare To Constant2'
 * '<S352>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Detect Fall Nonpositive'
 * '<S353>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Detect Rise Positive'
 * '<S354>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Enabled Subsystem1'
 * '<S355>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/MeaModule'
 * '<S356>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/MeaModule1'
 * '<S357>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/MeaModule2'
 * '<S358>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/RisingTimer'
 * '<S359>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Timer1'
 * '<S360>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Timer2'
 * '<S361>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Detect Fall Nonpositive/Nonpositive'
 * '<S362>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Detect Rise Positive/Positive'
 * '<S363>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/Beeper/Enabled Subsystem1/Chart'
 * '<S364>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU/MeaModule1'
 * '<S365>' : 'VehCtrlMdel240926_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU/MeaModule2'
 * '<S366>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL'
 * '<S367>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ'
 * '<S368>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/EEPROM'
 * '<S369>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling'
 * '<S370>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem'
 * '<S371>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem'
 * '<S372>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem'
 * '<S373>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com0'
 * '<S374>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com1'
 * '<S375>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com2'
 * '<S376>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem'
 * '<S377>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem1'
 * '<S378>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq100ms'
 * '<S379>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq10ms'
 * '<S380>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq500ms'
 * '<S381>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq50ms'
 * '<S382>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/DAQ/daq5ms'
 * '<S383>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/EEPROM/EEPROMOperation'
 * '<S384>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling/CCPBackground'
 * '<S385>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling/CCPReceive'
 * '<S386>' : 'VehCtrlMdel240926_2018b_amkspdlimit/RapidECUSetting/Polling/CCPReceive/Nothing'
 */
#endif                   /* RTW_HEADER_VehCtrlMdel240926_2018b_amkspdlimit_h_ */

/* File trailer for ECUCoder generated file VehCtrlMdel240926_2018b_amkspdlimit.h.
 *
 * [EOF]
 */
